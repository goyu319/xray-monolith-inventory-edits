#pragma once
#include <d3dcompiler.h>
#include <bcrypt.h>
#include <array>
#include <cstdio>
#include <string>
#include <vector>
#include <cstring>
#include <cstdint>
#include <algorithm>
#pragma comment(lib,"bcrypt.lib")

namespace painter_portable_shader {
using Digest=std::array<unsigned char,32>;
class Hash {
    BCRYPT_HASH_HANDLE hash=nullptr;
    bool valid=false;
    std::vector<UCHAR> storage;
    static BCRYPT_ALG_HANDLE provider() {
        static BCRYPT_ALG_HANDLE handle=[](){
            BCRYPT_ALG_HANDLE h=nullptr;
            if (BCryptOpenAlgorithmProvider(&h,BCRYPT_SHA256_ALGORITHM,nullptr,0)<0) return BCRYPT_ALG_HANDLE(nullptr);
            return h;
        }();
        return handle;
    }
public:
    Hash() {
        DWORD n=0,size=0;
        auto alg=provider();
        if(!alg || BCryptGetProperty(alg,BCRYPT_OBJECT_LENGTH,reinterpret_cast<PUCHAR>(&size),sizeof(size),&n,0)<0 || !size) return;
        storage.resize(size);
        valid=BCryptCreateHash(alg,&hash,storage.data(),size,nullptr,0,0)>=0;
    }
    Hash(const Hash&)=delete;
    ~Hash(){if(hash)BCryptDestroyHash(hash);}
    void append(const void* p,size_t n) {
        if(!valid || !n)return;
        if(n>0xffffffffu || BCryptHashData(hash,(PUCHAR)p,(ULONG)n,0)<0) valid=false;
    }
    void field(const void* p,size_t n) {const uint64_t len=n;append(&len,sizeof(len));append(p,n);}
    void stringField(const char* s){field(s,std::strlen(s));}
    bool finish(Digest& result){return valid && BCryptFinishHash(hash,result.data(),(ULONG)result.size(),0)>=0;}
};
inline std::string hex(const Digest& d) {
    static const char digits[]="0123456789abcdef";
    std::string s; s.reserve(64);
    for (auto b:d){s.push_back(digits[b>>4]);s.push_back(digits[b&15]);}
    return s;
}
inline const std::string& compilerId() {
    static const std::string id=[](){
        HMODULE module=GetModuleHandleW(D3DCOMPILER_DLL_W);
        if(!module || !GetProcAddress(module,"D3DCompile") || !GetProcAddress(module,"D3DPreprocess"))return std::string();
        wchar_t path[32768];
        auto n=GetModuleFileNameW(module,path,_countof(path));
        if(!n || n>=_countof(path))return std::string();
        const wchar_t* filename=wcsrchr(path,L'\\');
        if(!filename || _wcsnicmp(filename+1,L"d3dcompiler_",12)!=0)return std::string();
        HANDLE f=CreateFileW(path,GENERIC_READ,FILE_SHARE_READ|FILE_SHARE_DELETE,nullptr,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr);
        if(f==INVALID_HANDLE_VALUE)return std::string();
        Hash hash;std::array<unsigned char,16384> buf;DWORD got=0;bool ok=true;
        while((ok=ReadFile(f,buf.data(),(DWORD)buf.size(),&got,nullptr)!=FALSE)&&got)hash.append(buf.data(),got);
        CloseHandle(f);Digest d{};
        return ok && hash.finish(d)?hex(d):std::string();
    }();
    return id;
}
inline bool sensitive(const char* p,size_t size) {
    static const char* const tokens[]={"__FILE__","__LINE__","__DATE__","__TIME__","__COUNTER__","__TIMESTAMP__","##","??="};
    for(auto token:tokens)if(std::search(p,p+size,token,token+std::strlen(token))!=p+size)return true;
    return false;
}
class CheckedIncludes: public ID3DInclude {
    ID3DInclude& parent;
public:
    bool refuse=false;
    explicit CheckedIncludes(ID3DInclude& incl):parent(incl){}
    HRESULT STDMETHODCALLTYPE Open(D3D_INCLUDE_TYPE type,LPCSTR filename,LPCVOID from,LPCVOID* data,UINT* size) override {
        auto hr=parent.Open(type,filename,from,data,size);
        if(SUCCEEDED(hr) && data && *data && size) refuse|=sensitive(static_cast<const char*>(*data),*size);
        return hr;
    }
    HRESULT STDMETHODCALLTYPE Close(LPCVOID data) override{return parent.Close(data);}
};
inline bool horizontal(char c){return c==' '||c=='\t';}
inline bool lineDirective(const std::string& s) {
    if(s.compare(0,6,"#line ")!=0)return false;
    size_t i=6;
    while(i<s.size() && horizontal(s[i]))++i;
    const auto first=i;while(i<s.size() && s[i]>='0'&&s[i]<='9')++i;
    if(i==first)return false;
    while(i<s.size() && horizontal(s[i]))++i;
    if(i==s.size())return true;
    if(s[i++]!='"')return false;
    while(i<s.size()) {
        if(s[i]=='\\'){if(++i>=s.size())return false;++i;continue;}
        if(s[i++]=='"')return i==s.size();
    }
    return false;
}
inline bool normalize(const char* data,size_t n,std::string& result) {
    // Preserve physical newlines and all boundaries within nonblank lines. Collapsing an
    // existing horizontal separator never joins two tokens. Unknown directives keep their bytes.
    // Reject continued lines and multiline string literals rather than guessing their meaning.
    result.clear();result.reserve(n);
    size_t s=0;
    while(s<n) {
        size_t e=s;while(e<n && data[e]!='\r' && data[e]!='\n')++e;
        size_t a=s,z=e;
        while(a<z && horizontal(data[a]))++a;
        while(z>a && horizontal(data[z-1]))--z;
        if(z>a) {
            std::string line(data+a,z-a);
            if(!lineDirective(line)) {
                if(line.back()=='\\')return false;
                if(line.front()=='#') {
                    result.append(line);result.push_back('\n');
                } else {
                    char quote=0; bool space=false;
                    for(size_t i=0;i<line.size();++i) {
                        char c=line[i];
                        if(!c)return false;
                        if(quote) {
                            result.push_back(c);
                            if(c=='\\'){if(++i==line.size())return false;result.push_back(line[i]);}
                            else if(c==quote)quote=0;
                        } else if(horizontal(c))space=true;
                        else {
                            if(space){result.push_back(' ');space=false;}
                            if(c=='"' || c=='\'')quote=c;
                            result.push_back(c);
                        }
                    }
                    if(quote)return false;
                    result.push_back('\n');
                }
            }
        }
        s=e;while(s<n && (data[s]=='\r'||data[s]=='\n'))++s;
    }
    return !result.empty();
}
inline bool key(const char* name,const char* entry,const char* target,UINT flags,const void* root,size_t n,
                const D3D_SHADER_MACRO* defs,ID3DInclude& incl,Digest& digest) {
    if(!name || strncmp(name,"painter_",8) || !entry || strcmp(entry,"main") || !target ||
        flags!=D3DCOMPILE_PACK_MATRIX_ROW_MAJOR || compilerId().empty() || !root || !n ||
        sensitive(static_cast<const char*>(root),n))return false;
    if (strcmp(target,"ps_5_0") && strcmp(target,"vs_5_0") && strcmp(target,"cs_5_0") &&
        strcmp(target,"gs_5_0") && strcmp(target,"hs_5_0") && strcmp(target,"ds_5_0"))return false;
    CheckedIncludes checked(incl);ID3DBlob* out=nullptr;
    auto hr=D3DPreprocess(root,n,"",defs,&checked,&out,nullptr);
    if(FAILED(hr)||!out){if(out)out->Release();return false;}
    const char* p=static_cast<const char*>(out->GetBufferPointer());size_t count=out->GetBufferSize();
    while(count && !p[count-1])--count;
    std::string canonical;
    bool ok=!checked.refuse && !sensitive(p,count) && normalize(p,count,canonical);
    out->Release(); if(!ok)return false;
    Hash hash;hash.stringField("r4-owned-sm5-fxc-source-1");hash.stringField(compilerId().c_str());hash.stringField(name);
    hash.stringField(entry);hash.stringField(target);hash.field(&flags,sizeof(flags));hash.field(canonical.data(),canonical.size());
    return hash.finish(digest);
}
}

