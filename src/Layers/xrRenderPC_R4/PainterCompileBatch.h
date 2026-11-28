#pragma once
#include <d3dcompiler.h>
#include <algorithm>
#include <atomic>
#include "PainterTearCompiler.h"
#include "PainterBurnTearCompiler.h"
#include "PainterShadowCastCompiler.h"
#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "PainterSurfaceSelector.h"
namespace painter_compile_batch {
using Blob = std::shared_ptr<ID3DBlob>;
inline Blob own(ID3DBlob* p) { return Blob(p, [](ID3DBlob* b) { if (b) b->Release(); }); }
inline bool dxbc(const void* data, size_t len) {
    if (!data || len<32 || memcmp(data,"DXBC",4)) return false;
    unsigned total=0, chunks=0;
    memcpy(&total,static_cast<const char*>(data)+24,4);
    memcpy(&chunks,static_cast<const char*>(data)+28,4);
    return total==len && chunks>0 && chunks<=(len-32)/4;
}

// These files have already passed through the mounted filesystem and its source-edit rules.
// No worker opens a file or consults renderer state. Stable string storage also supports empty includes.
class IncludeStore : public ID3DInclude {
    std::map<std::string,std::string> files;
public:
    HRESULT STDMETHODCALLTYPE Open(D3D_INCLUDE_TYPE, LPCSTR name, LPCVOID, LPCVOID* data, UINT* size) override {
        const auto& readable=files;
        const auto f=readable.find(name);
        if (f==readable.end()) return E_FAIL;
        *data=f->second.c_str(); *size=static_cast<UINT>(f->second.size());
        return S_OK;
    }
    HRESULT STDMETHODCALLTYPE Close(LPCVOID) override { return S_OK; }
    void insert(const char* name, const void* data, UINT size) {
        files.emplace(name,std::string(static_cast<const char*>(data),size));
    }
    const std::string& at(const std::string& n) const {return files.at(n);}
    size_t size() const { return files.size(); }
};
class RecordIncludes : public ID3DInclude {
    IncludeStore& saved;
    ID3DInclude& owner;
    std::set<std::string>& names;
public:
    RecordIncludes(IncludeStore& s,ID3DInclude& o,std::set<std::string>& used):saved(s),owner(o),names(used){}
    HRESULT STDMETHODCALLTYPE Open(D3D_INCLUDE_TYPE type,LPCSTR name,LPCVOID parent,LPCVOID* data,UINT* size) override {
        if (SUCCEEDED(saved.Open(type,name,parent,data,size))){names.emplace(name);return S_OK;}
        const void* raw=nullptr; UINT bytes=0;
        HRESULT hr=owner.Open(type,name,parent,&raw,&bytes);
        if (FAILED(hr)) return hr;
        names.emplace(name);
        saved.insert(name,raw,bytes);
        owner.Close(raw);
        return saved.Open(type,name,parent,data,size);
    }
    HRESULT STDMETHODCALLTYPE Close(LPCVOID) override { return S_OK; }
};
struct Define { std::string name, value; bool hasValue; };
struct Job {
    std::string path, name, source, entry, target, error;
    unsigned flags=0, sourceCrc=0;
    std::set<std::string> includeNames;
    std::vector<Define> defs;
    painter_tear_link::Recipe torn;
    painter_burn_tear_link::Recipe burnedTorn;
    painter_shadow_cast::Recipe burnedShadow;
    Blob code;
    painter_surface_selector::Recipe exterior;
    std::string ordinaryPath;
    bool requiresRendererCompiler() const {return torn.ready() || burnedTorn.ready() || burnedShadow.ready() || exterior.entry!=nullptr;}
    HRESULT status=E_PENDING;
};
struct Batch {
    IncludeStore includes;
    painter_tear_modules::Cache tornLibraries;
    std::vector<std::pair<painter_tear_modules::SharedLibrary,UINT>> tornParts;
    std::vector<Job> jobs;
    std::set<std::string> paths;
    unsigned cached=0,failedReads=0,duplicate=0;
    bool capture(const char* path,const char* name,unsigned crc,const void* source,unsigned bytes,
                 const D3D_SHADER_MACRO* defs,const char* entry,const char* target,unsigned flags,ID3DInclude& owner) {
        if (!paths.insert(path).second) { ++duplicate; return true; }
        Job j; j.path=path; j.name=name; j.sourceCrc=crc; j.source.assign(static_cast<const char*>(source),bytes);
        j.entry=entry; j.target=target; j.flags=flags;
        for (auto d=defs;d && d->Name;++d) j.defs.push_back({d->Name,d->Definition?d->Definition:"",d->Definition!=nullptr});
        RecordIncludes record(includes,owner,j.includeNames);
        ID3DBlob *text=nullptr,*message=nullptr;
        j.status=D3DPreprocess(source,bytes,"",defs,&record,&text,&message);
        auto output=own(text), error=own(message);
        if (FAILED(j.status)) {
            j.error=error?static_cast<const char*>(error->GetBufferPointer()):"preprocess failed";
            jobs.push_back(std::move(j));
            return false;
        }
        if (painter_tear_link::eligible(name,entry,target,flags)) {
            j.torn=painter_tear_link::capture(tornLibraries,name,entry,target,flags,source,bytes,defs,&includes);
            if (j.torn.ready()) {
                for (const auto& part : j.torn.parts) tornParts.emplace_back(part,flags);
            }
        }
        j.status=E_PENDING;
        jobs.push_back(std::move(j)); return true;
    }
    void compile(Job& j) {
        if (j.status!=E_PENDING) return;
        if(j.burnedTorn.ready()) {
            ID3DBlob* value=nullptr;
            if (painter_burn_tear_link::make(j.burnedTorn,j.flags,&value)) {
                j.code=own(value);j.status=S_OK;return;
            }
            j.path=j.ordinaryPath;
        }
        if(j.burnedShadow.ready()) {
            ID3DBlob* value=nullptr;
            if (painter_shadow_cast::make(j.burnedShadow,j.flags,&value)) {
                j.code=own(value);j.status=S_OK;return;
            }
            j.path=j.ordinaryPath;
        }
        if(j.exterior.entry) {
            ID3DBlob* value=nullptr;
            if(SUCCEEDED(painter_surface_selector::compile(j.exterior,&value))){
                j.code=own(value);j.status=S_OK;return;
            }
            j.path=j.ordinaryPath;
        }
        std::vector<D3D_SHADER_MACRO> defs; defs.reserve(j.defs.size()+1);
        for (const auto& d:j.defs) defs.push_back({d.name.c_str(),d.hasValue?d.value.c_str():nullptr});
        defs.push_back({nullptr,nullptr});
        ID3DBlob *code=nullptr,*message=nullptr;
        if (j.torn.ready() && painter_tear_link::make(j.torn,j.flags,&code)) {
            j.code=own(code);
            j.status=S_OK;
            return;
        }
        j.status=D3DCompile(j.source.data(),j.source.size(),"",defs.data(),&includes,
            j.entry.c_str(),j.target.c_str(),j.flags,0,&code,&message);
        j.code=own(code); auto error=own(message);
        if (SUCCEEDED(j.status) && (!code || !dxbc(code->GetBufferPointer(),code->GetBufferSize()))) j.status=E_FAIL;
        if (FAILED(j.status)) j.error=error?static_cast<const char*>(error->GetBufferPointer()):"invalid shader bytecode";
    }
    unsigned run(unsigned available) {
        if (jobs.empty()) return 0;
        const auto tasks=jobs.size()+tornParts.size();
        unsigned count=(std::min)(static_cast<unsigned>(tasks),(std::max)(1u,available));
        std::atomic<size_t> next{0};
        auto work=[&] { for (;;) {
            const auto task=next.fetch_add(1,std::memory_order_relaxed);
            if (task>=tasks) return;
            if (task<tornParts.size()) {
                auto& part=tornParts[task];
                part.first->compile(part.second);
                continue;
            }
            const auto i=task-tornParts.size();
            compile(jobs[i]); } };
        std::vector<std::thread> workers;
        workers.reserve(count-1);
        for (unsigned i=1;i<count;++i) {
            try { workers.emplace_back(work); }
            catch (const std::system_error&) { break; }
        }
        work();
        for (auto& worker:workers) worker.join();
        return static_cast<unsigned>(workers.size()+1);
    }
};
}
