#pragma once
#include "PainterPortableShader.h"
#include <mutex>
#include <memory>
namespace painter_surface_selector {
struct Entry {
    const char *normal,*alpha,*normalDigest,*alphaDigest,*canonical,*code;
};
#include "PainterSurfaceSelectorList.h"
struct Recipe {
    const Entry* entry=nullptr;
    std::string source;
};
inline std::string hash(const void* p,size_t n) {
    painter_portable_shader::Hash h;
    painter_portable_shader::Digest d{};
    h.append(p,n);
    return h.finish(d)?painter_portable_shader::hex(d):std::string();
}
inline std::string flat(const std::string& s) {
    std::string out;out.reserve(s.size());
    bool space=false;
    for(char c:s){
        if(c==' '||c=='\r'||c=='\n'||c=='\t'){space=!out.empty();continue;}
        if(space)out+=' ';
        out+=c;space=false;
    }
    return out;
}
inline void change(std::string& text,const char* from,const char* to) {
    const size_t n=std::strlen(from),m=std::strlen(to);
    size_t pos=0;
    while((pos=text.find(from,pos))!=std::string::npos){text.replace(pos,n,to);pos+=m;}
}
inline Recipe fromText(const Entry& entry,bool alpha,const void* raw,size_t size) {
    const char* bytes=static_cast<const char*>(raw);
    while(size && bytes[size-1]=='\0')--size;
    std::string text;
    if(!painter_portable_shader::normalize(bytes,size,text) || text.find_first_of("#\\'\"")!=std::string::npos)return {};
    text=flat(text);
    if(hash(text.data(),text.size())!=(alpha?entry.alphaDigest:entry.normalDigest))return {};
    if (std::strstr(entry.normal,"_flat")) {
        const char* clip="clip ( D . w - float ( 200.0 / 255.0 ) ) ;";
        change(text,clip,"");
        change(text,"float3 D = tbase ( I . tcdh ) ;","float4 D = tbase ( I . tcdh ) ;");
        change(text,"normalize ( ( float3 ) I . N . xyz )","normalize ( I . N . xyz )");
        text=flat(text);
        change(text,"float4 D = tbase ( I . tcdh ) ;","float4 D = tbase ( I . tcdh ) ; [branch] if(alphaMode) clip ( D . w - float ( 200.0 / 255.0 ) ) ;");
        change(text,"painter_material_base ( painter_io_p_flat I )","painter_material_base ( painter_io_p_flat I , bool alphaMode )");
        change(text,"painter_blood_original_main ( painter_io_p_flat I )","painter_blood_original_main ( painter_io_p_flat I , bool alphaMode )");
    } else {
        const char* clip="clip ( S . base . w - float ( 200.0 / 255.0 ) ) ;";
        change(text,clip,"");
        change(text,"S . base . rgb","S . base . xyz");
        text=flat(text);
        change(text,"surface_bumped S = sload ( I ) ;",
            "surface_bumped S = sload ( I ) ; [branch] if(alphaMode) clip ( S . base . w - float ( 200.0 / 255.0 ) ) ;");
        change(text,"painter_material_base ( painter_io_p_bumped I )","painter_material_base ( painter_io_p_bumped I , bool alphaMode )");
        change(text,"painter_blood_original_main ( painter_io_p_bumped I )","painter_blood_original_main ( painter_io_p_bumped I , bool alphaMode )");
    }
    change(text,"painter_material_base ( I )","painter_material_base ( I , alphaMode )");
    change(text,"f_deffer main (","uniform float4 painter_surface_variant; f_deffer main (");
    change(text,"painter_blood_original_main ( I )","painter_blood_original_main ( I , painter_surface_variant.x>.5 )");
    if(hash(text.data(),text.size())!=entry.canonical)return {};
    return {&entry,std::move(text)};
}
inline Recipe identify(const char* name,const char* entry,const char* profile,UINT flags,
        const void* src,size_t size,const D3D_SHADER_MACRO* defs,ID3DInclude& resolver) {
    if(!name||!entry||!profile||strcmp(entry,"main")||strcmp(profile,"ps_5_0")||
        flags!=D3DCOMPILE_PACK_MATRIX_ROW_MAJOR||strncmp(name,"painter_burn_painter_blood_",27))return {};
    for(const auto& e:entries){
        const bool alpha=strcmp(name,e.alpha)==0;
        if(!alpha && strcmp(name,e.normal))continue;
        if(painter_portable_shader::compilerId()!="81494adc53aae6a9b2cda729da5d20b0d15bffcb663dff7ae0242cab2e1f1a58")return {};
        painter_portable_shader::CheckedIncludes includes(resolver);
        if(painter_portable_shader::sensitive(static_cast<const char*>(src),size))return {};
        ID3DBlob* pp=nullptr;
        if(FAILED(D3DPreprocess(src,size,"",defs,&includes,&pp,nullptr))||!pp){if(pp)pp->Release();return {};}
        Recipe r;
        if(!includes.refuse)r=fromText(e,alpha,pp->GetBufferPointer(),pp->GetBufferSize());
        pp->Release();return r;
    }
    return {};
}
inline bool valid(const Recipe& r,const void* data,size_t size) {
    return r.entry && size && size<1024*1024 && hash(data,size)==r.entry->code;
}
inline HRESULT compile(const Recipe& r,ID3DBlob** out) {
    if(!r.entry||!out)return E_INVALIDARG;
    struct Slot{std::mutex mutex;std::shared_ptr<ID3DBlob> result;};
    static Slot slots[sizeof(entries)/sizeof(entries[0])];
    auto& slot=slots[r.entry-entries];std::lock_guard<std::mutex> guard(slot.mutex);
    if(!slot.result){
        ID3DBlob* raw=nullptr;
        auto hr=D3DCompile(r.source.data(),r.source.size(),"",nullptr,nullptr,"main","ps_5_0",D3DCOMPILE_PACK_MATRIX_ROW_MAJOR,0,&raw,nullptr);
        if(FAILED(hr)||!raw){if(raw)raw->Release();return E_FAIL;}
        if(!valid(r,raw->GetBufferPointer(),raw->GetBufferSize())){raw->Release();return E_FAIL;}
        slot.result.reset(raw,[](ID3DBlob* p){p->Release();});
    }
    *out=slot.result.get();(*out)->AddRef();return S_OK;
}
}
