#pragma once
#include <array>
#include <cstdint>
#include <cstring>
#if defined(__AVX__) || defined(__SSSE3__)
#include <immintrin.h>
#endif

// The only two absent 7-bit labels are 0 and 16. Bit 64 (cloth padding) is present.
inline unsigned grenadeLabelPresent(unsigned value) { return (value & 0x6fu) != 0; }

struct GrenadePresencePage {
    const std::uint8_t *palette=nullptr,*indices=nullptr;
    std::uint16_t paletteMask=0;
    std::uint8_t widthMask=0, encoding=0;
    enum : unsigned { empty=1, full=2, twoLabels=3, fourLabels=4, sixteenLabels=5, direct=6 };
    void open(const painter_detail::TissueField& f,int tx,int ty,int tz) {
        encoding=empty;
        if(tx<0 || unsigned(tx)>=f.pages[0]) return;
        const int valid=int(f.size[0])-tx*8;
        if(valid<=0) return;
        widthMask=std::uint8_t((1u<<std::min(8,valid))-1);
        const auto page=f.page[16+(std::size_t(tz)*f.pages[1]+unsigned(ty))*f.pages[0]+unsigned(tx)];
        if(page&0x80000000u){encoding=grenadeLabelPresent(page)?full:empty;return;}
        const unsigned bits=1u<<((page>>28)&3);
        palette=f.data.data()+(page&0x0fffffffu);
        if(bits==8){encoding=direct;indices=palette;return;}
        const unsigned count=1u<<bits;
        for(unsigned k=0;k<count;++k)paletteMask|=std::uint16_t(grenadeLabelPresent(palette[k])<<k);
        indices=palette+count;
        if(!paletteMask) return;
        if(paletteMask==((1u<<count)-1)){encoding=full;return;}
        encoding=bits==1?twoLabels:bits==2?fourLabels:sixteenLabels;
    }
    unsigned row(unsigned i) const {
        switch(encoding){
        case empty:return 0;
        case full:return widthMask;
        case twoLabels:return widthMask&(indices[i]^((paletteMask&1)?255u:0u));
        case fourLabels:{
            static const auto table=[]{
                std::array<std::array<std::uint8_t,256>,16> lut{};
                for(unsigned m=0;m<16;++m)for(unsigned b=0;b<256;++b)
                    for(unsigned s=0;s<4;++s)
                        lut[m][b]|=std::uint8_t(((m>>((b>>(2*s))&3))&1)<<s);
                return lut;
            }();
            const auto& mask=table[paletteMask];const auto* p=indices+2*i;
            return widthMask&(mask[p[0]]|(unsigned(mask[p[1]])<<4));
        }
        case sixteenLabels:{
            const auto* p=indices+4*i;
#if defined(__AVX__) || defined(__SSSE3__)
            std::uint32_t raw;std::memcpy(&raw,p,4);
            const __m128i bytes=_mm_cvtsi32_si128(int(raw));
            const __m128i nibbles=_mm_and_si128(_mm_unpacklo_epi8(bytes,_mm_srli_epi16(bytes,4)),_mm_set1_epi8(15));
            const __m128i slots=_mm_loadu_si128(reinterpret_cast<const __m128i*>(palette));
            const __m128i presence=_mm_cmpgt_epi8(_mm_and_si128(slots,_mm_set1_epi8(0x6f)),_mm_setzero_si128());
            return widthMask&unsigned(_mm_movemask_epi8(_mm_shuffle_epi8(presence,nibbles)));
#else
            unsigned m=0;
            for(unsigned j=0;j<8;++j)m|=((paletteMask>>((p[j>>1]>>(4*(j&1)))&15))&1u)<<j;
            return m&widthMask;
#endif
        }
        default:{
            const auto* p=indices+8*i;
#if defined(__AVX__) || defined(__SSSE3__)
            const auto bytes=_mm_loadl_epi64(reinterpret_cast<const __m128i*>(p));
            const auto positive=_mm_cmpgt_epi8(_mm_and_si128(bytes,_mm_set1_epi8(0x6f)),_mm_setzero_si128());
            return widthMask&unsigned(_mm_movemask_epi8(positive));
#else
            unsigned m=0;for(unsigned j=0;j<8;++j)m|=grenadeLabelPresent(p[j])<<j;return m&widthMask;
#endif
        }}
    }
};

// A worker owns these descriptors for the lifetime of one changed brick or face halo.
// Ghost bricks and untouched assets incur no persistent allocation.
template<int Ratio> struct GrenadePresenceRows {
    static constexpr int side=Ratio+1;
    const painter_detail::TissueField& field;
    const int tx0,ty0,tz0;
    const unsigned shift;
    std::array<GrenadePresencePage,side*side*side> pages{};
    GrenadePresenceRows(const painter_detail::TissueField& f,int x,int y,int z)
        :field(f),tx0(x>>3),ty0(y>>3),tz0(z>>3),shift(unsigned(x)&7){}
    unsigned get(int y,int z){
        if(y<0||z<0||unsigned(y)>=field.size[1]||unsigned(z)>=field.size[2])return 0;
        const int ty=y>>3,tz=z>>3;
        const unsigned local=(unsigned(z&7)*8+unsigned(y&7));
        auto* line=pages.data()+((tz-tz0)*side+ty-ty0)*side;
        std::uint64_t bits=0;
        const unsigned count=Ratio+(shift!=0);
        for(unsigned k=0;k<count;++k){
            auto& p=line[k];
            if(!p.encoding)p.open(field,tx0+int(k),ty,tz);
            bits|=std::uint64_t(p.row(local))<<(8*k);
        }
        return unsigned(bits>>shift);
    }
};
