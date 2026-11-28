#pragma once
#include "PainterPortableShader.h"
namespace painter_portable_shader {
struct Reference { uint32_t sourceCRC=0; Digest expected{}; std::string option; };
inline bool reference(const void* src,size_t size,const Digest& semantic,Reference& selected) {
    static const char magic[]="PSEIDX01";
    if(!src || size<12 || size>16384 || memcmp(src,magic,8))return false;
    const auto* data=static_cast<const unsigned char*>(src);uint32_t count=0;
    memcpy(&count,data+8,4);if(!count||count>64)return false;
    size_t i=12;bool found=false;
    for(uint32_t j=0;j<count;++j){
        if(size-i<69)return false;
        const auto* digest=data+i;i+=32;uint32_t crc;memcpy(&crc,data+i,4);i+=4;
        Digest code{};memcpy(code.data(),data+i,32);i+=32;
        auto n=data[i++];if(!n || n>96 || size-i<n)return false;
        for(size_t k=i;k<i+n;++k) {
            auto c=data[k];if(!((c>='0'&&c<='9')||(c>='a'&&c<='z')||(c>='A'&&c<='Z')||c=='_'||c=='-'))return false;
        }
        if(memcmp(digest,semantic.data(),32)==0){
            if(found)return false;
            selected.sourceCRC=crc;selected.expected=code;selected.option.assign((const char*)data+i,n);found=true;
        }
        i+=n;
    }
    return found && i==size;
}
inline bool payload(const void* bytes,size_t n,const Digest& expected) {
    if(!bytes || n<32 || n>1048568 || memcmp(bytes,"DXBC",4))return false;
    Hash h;Digest d{};h.append(bytes,n);return h.finish(d) && d==expected;
}
}

