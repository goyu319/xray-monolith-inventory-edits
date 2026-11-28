#pragma once
#include <windows.h>
#include <d3dcompiler.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace painter_menu {
struct Define {
    std::string name, value;
    bool hasValue = false;
    bool operator==(const Define& b) const { return name==b.name && value==b.value && hasValue==b.hasValue; }
};
struct File {
    std::string name, bytes;
    bool operator==(const File& b) const { return name==b.name && bytes==b.bytes; }
};
struct Input {
    std::string path, name, source, entry, target;
    std::uint32_t flags=0, sourceCrc=0;
    std::vector<Define> defs;
    std::vector<File> includes;
    bool same(const Input& b) const {
        return flags==b.flags && sourceCrc==b.sourceCrc && path==b.path && name==b.name &&
               entry==b.entry && target==b.target && defs==b.defs && source==b.source && includes==b.includes;
    }
};
struct Result {
    std::vector<unsigned char> bytes;
    long status=E_FAIL;
};

namespace wire {
constexpr char command[]="--painter-compile-input-v1";
constexpr std::uint32_t magic=0x31524350, version=1, maxPacket=64u*1024u*1024u;
inline bool transfer(HANDLE h,void* p,size_t size,bool write) {
    auto* cursor=static_cast<unsigned char*>(p);
    while(size) {
        const DWORD want=static_cast<DWORD>((std::min)(size,size_t(65536)));
        DWORD done=0;
        if (!(write ? WriteFile(h,cursor,want,&done,nullptr) : ReadFile(h,cursor,want,&done,nullptr)) || !done) return false;
        cursor+=done; size-=done;
    }
    return true;
}
inline bool send(HANDLE h,const void* p,size_t size) {return transfer(h,const_cast<void*>(p),size,true);}
inline bool receive(HANDLE h,void* p,size_t size) {return transfer(h,p,size,false);}
inline bool word(HANDLE h,std::uint32_t n){return send(h,&n,sizeof(n));}
inline bool stringOut(HANDLE h,const std::string& s) {
    return s.size()<=maxPacket && word(h,static_cast<std::uint32_t>(s.size())) && send(h,s.data(),s.size());
}
struct Reader {
    HANDLE h; size_t remaining=maxPacket;
    bool exact(void* p,size_t n) {
        if(n>remaining)return false;
        remaining-=n;return receive(h,p,n);
    }
    bool number(std::uint32_t& n){return exact(&n,sizeof(n));}
    bool text(std::string& str){
        std::uint32_t n=0;
        if(!number(n)||n>remaining)return false;
        str.resize(n);return exact(str.data(),n);
    }
};
// Check the actual mapped compiler on both sides. The child inherits the same
// image location and working directory, but a virtual filesystem can redirect DLLs.
inline bool compilerFile(std::array<DWORD,7>& id) {
    HMODULE mod=nullptr;
    const HMODULE imported=GetModuleHandleW(D3DCOMPILER_DLL_W);
    const auto entry=imported?GetProcAddress(imported,"D3DCompile"):nullptr;
    if(!entry || !GetModuleHandleExW(0x00000004u|0x00000002u,reinterpret_cast<LPCWSTR>(entry),&mod))return false;
    wchar_t path[32768];
    const DWORD n=GetModuleFileNameW(mod,path,DWORD(sizeof(path)/sizeof(*path)));
    if(!n||n>=sizeof(path)/sizeof(*path))return false;
    HANDLE file=CreateFileW(path,FILE_READ_ATTRIBUTES,FILE_SHARE_READ|FILE_SHARE_WRITE|FILE_SHARE_DELETE,
                            nullptr,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr);
    if(file==INVALID_HANDLE_VALUE)return false;
    BY_HANDLE_FILE_INFORMATION info{};
    bool ok=GetFileInformationByHandle(file,&info)!=FALSE;
    CloseHandle(file);
    if(ok)id={info.dwVolumeSerialNumber,info.nFileIndexHigh,info.nFileIndexLow,info.nFileSizeHigh,
              info.nFileSizeLow,info.ftLastWriteTime.dwHighDateTime,info.ftLastWriteTime.dwLowDateTime};
    return ok;
}
inline bool validDXBC(const void* p,size_t n) {
    if(!p||n<32||std::memcmp(p,"DXBC",4))return false;
    std::uint32_t len=0,chunks=0;auto* bytes=static_cast<const unsigned char*>(p);
    std::memcpy(&len,bytes+24,4);std::memcpy(&chunks,bytes+28,4);
    return len==n && chunks && chunks<=(n-32)/4;
}
inline bool fitsPacket(const Input& i) {
    if(i.defs.size()>4096||i.includes.size()>4096)return false;
    std::uint64_t n=7*sizeof(std::uint32_t)+i.source.size()+i.entry.size()+i.target.size();
    for(const auto& d:i.defs)n+=3*sizeof(std::uint32_t)+d.name.size()+d.value.size();
    for(const auto& f:i.includes)n+=2*sizeof(std::uint32_t)+f.name.size()+f.bytes.size();
    return n<=maxPacket;
}
inline bool sendInput(HANDLE h,const Input& i) {
    if(i.defs.size()>4096||i.includes.size()>4096)return false;
    if(!word(h,magic)||!word(h,i.flags)||!stringOut(h,i.source)||!stringOut(h,i.entry)||!stringOut(h,i.target)||
       !word(h,static_cast<std::uint32_t>(i.defs.size())))return false;
    for(const auto& d:i.defs)
        if(!stringOut(h,d.name)||!stringOut(h,d.value)||!word(h,d.hasValue?1:0))return false;
    if(!word(h,static_cast<std::uint32_t>(i.includes.size())))return false;
    for(const auto& f:i.includes)if(!stringOut(h,f.name)||!stringOut(h,f.bytes))return false;
    return true;
}
inline bool readInput(HANDLE h,Input& i) {
    Reader r{h};
    std::uint32_t sig=0,n=0;
    if(!r.number(sig)||sig!=magic||!r.number(i.flags)||!r.text(i.source)||!r.text(i.entry)||!r.text(i.target)||
       !r.number(n)||n>4096)return false;
    i.defs.resize(n);
    for(auto& d:i.defs) {
        std::uint32_t v=0;
        if(!r.text(d.name)||!r.text(d.value)||!r.number(v)||v>1)return false;
        d.hasValue=v!=0;
    }
    if(!r.number(n)||n>4096)return false;
    i.includes.resize(n);
    for(auto& f:i.includes)if(!r.text(f.name)||!r.text(f.bytes))return false;
    return true;
}
class Includes final:public ID3DInclude {
    const std::vector<File>& files;
public:
    explicit Includes(const std::vector<File>& f):files(f){}
    HRESULT STDMETHODCALLTYPE Open(D3D_INCLUDE_TYPE,LPCSTR name,LPCVOID,LPCVOID* data,UINT* size) override {
        for(const auto& file:files)if(file.name==name){
            *data=file.bytes.c_str();*size=static_cast<UINT>(file.bytes.size());return S_OK;
        }
        return E_FAIL;
    }
    HRESULT STDMETHODCALLTYPE Close(LPCVOID) override {return S_OK;}
};
inline int childMain() {
    HANDLE in=GetStdHandle(STD_INPUT_HANDLE),out=GetStdHandle(STD_OUTPUT_HANDLE);
    if(GetFileType(in)!=FILE_TYPE_PIPE||GetFileType(out)!=FILE_TYPE_PIPE)return 2;
    std::array<DWORD,7> id{};
    if(!compilerFile(id)||!word(out,version)||!send(out,id.data(),sizeof(id)))return 3;
    try {
        for(;;) {
            Input i;if(!readInput(in,i))return 0;
            std::vector<D3D_SHADER_MACRO> defs;
            for(const auto& d:i.defs)defs.push_back({d.name.c_str(),d.hasValue?d.value.c_str():nullptr});
            defs.push_back({nullptr,nullptr});Includes files(i.includes);
            ID3DBlob* code=nullptr;
            const bool library=i.target=="lib_5_0" && i.entry.empty() && i.defs.empty() && i.includes.empty();
            HRESULT hr=D3DCompile(i.source.data(),i.source.size(),library?nullptr:"",library?nullptr:defs.data(),
                                  library?nullptr:&files,library?nullptr:i.entry.c_str(),i.target.c_str(),
                                  i.flags,0,&code,nullptr);
            if(SUCCEEDED(hr)&&(!code||code->GetBufferSize()>maxPacket||
                                !validDXBC(code->GetBufferPointer(),code->GetBufferSize())))hr=E_FAIL;
            const auto n=SUCCEEDED(hr)?static_cast<std::uint32_t>(code->GetBufferSize()):0u;
            bool ok=word(out,static_cast<std::uint32_t>(hr))&&word(out,n)&&(!n||send(out,code->GetBufferPointer(),n));
            if(code)code->Release();
            if(!ok)return 4;
        }
    } catch(...) {return 5;}
}
} // namespace wire
} // namespace painter_menu
