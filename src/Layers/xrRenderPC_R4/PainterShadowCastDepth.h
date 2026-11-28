#pragma once
#include "PainterTearDepth.h"
namespace painter_shadow_depth {
inline bool normalize(unsigned char* input, size_t size, unsigned expectedWrites)
{
    if (!input || size < 36 || size > 64*1024*1024 || !expectedWrites || expectedWrites > 16) return false;
    if (std::memcmp(input,"DXBC",4) || word(input+20)!=1 || word(input+24)!=size) return false;
    unsigned long original[4];
    CalculateDXBCChecksum(input,static_cast<unsigned long>(size),original);
    if (std::memcmp(original,input+4,16)) return false;
    const uint32_t chunks=word(input+28);
    if (!chunks || chunks>(size-32)/4) return false;
    std::vector<size_t> writes;
    bool pixelProgram=false;
    for(uint32_t i=0;i<chunks;++i) {
        const size_t offset=word(input+32+4*i);
        if(offset<32+4*chunks || offset>size-8) return false;
        const size_t bytes=word(input+offset+4);
        if(bytes>size-offset-8) return false;
        if(std::memcmp(input+offset,"SHEX",4)) continue;
        if(pixelProgram || bytes%4 || bytes<12) return false;
        const auto* program=input+offset+8;
        const size_t n=bytes/4;
        if(word(program)!=kPixel50 || word(program+4)!=n) return false;
        pixelProgram=true;
        for(size_t pc=2;pc<n;) {
            const uint32_t op=word(program+pc*4);
            const uint32_t code=op&0x7ffu;
            if(code==53 && n-pc<2) return false;
            const size_t length=code==53 ? word(program+4*(pc+1)) : ((op>>24)&0x7fu);
            if(!length || length>n-pc) return false;
            if((code==52 || code==54) && !(op>>31) && length>=3 &&
               word(program+(pc+1)*4)==kDepthComponent) writes.push_back(offset+8+(pc+1)*4);
            pc+=length;
        }
    }
    if(!pixelProgram || writes.size()!=expectedWrites) return false;
    std::vector<unsigned char> copy(input,input+size);
    for(size_t dest:writes) store(copy.data()+dest,kDepthScalar);
    // The SM5 linker leaves the final two ABI padding bytes of each legacy signature record undefined.
    // Require the observed formats before clearing just that padding. All named fields remain intact.
    unsigned seen = 0;
    for (uint32_t i=0; i<chunks; ++i) {
        const size_t offset=word(copy.data()+32+4*i);
        auto* chunk=copy.data()+offset;
        const unsigned type=!std::memcmp(chunk,"ISGN",4) ? 1u : !std::memcmp(chunk,"OSGN",4) ? 2u : 0u;
        if (!type) continue;
        const size_t size=word(chunk+4);
        if ((seen&type) || size<8) return false;
        seen|=type;
        const auto count=word(chunk+8), start=word(chunk+12);
        if (count!=(type==1 ? 9u : 2u) || start!=8 || size_t(start)+24*count>size) return false;
        for (unsigned parameter=0;parameter<count;++parameter) {
            auto* record=chunk+8+start+24*parameter;
            record[22]=record[23]=0;
        }
    }
    if (seen!=3) return false;

    unsigned long updated[4];
    CalculateDXBCChecksum(copy.data(),static_cast<unsigned long>(size),updated);
    std::memcpy(copy.data()+4,updated,sizeof(updated));
    ID3DBlob* parsed=nullptr;
    const HRESULT validated=D3DStripShader(copy.data(),size,0,&parsed);
    if(FAILED(validated)||!parsed) return false;
    parsed->Release();
    std::memcpy(input,copy.data(),size);
    return true;
}
}
