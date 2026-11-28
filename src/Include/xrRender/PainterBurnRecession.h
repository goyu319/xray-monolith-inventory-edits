#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

// Rest metres. An exposure grows only when its source witnesses the same site
// again. The worker owns the history; drawing and sleeping do not advance it.
namespace painter_burn {
struct RecessionFront {
    std::array<float,3> centre{};
    float radius{};
    std::uint16_t charMm{};
    std::uint8_t maturity{};
};
inline std::uint32_t burnBits(float v) noexcept {
    std::uint32_t n; std::memcpy(&n,&v,sizeof n); return n;
}
inline float burnFloat(std::uint32_t n) noexcept {
    float v; std::memcpy(&v,&n,sizeof v); return v;
}
struct Recession {
    static constexpr unsigned Limit=6, Words=8;
    std::array<RecessionFront,Limit> fronts{};
    unsigned count{};

    // At seven seconds the original surface has charred. Subsequent sustained
    // contact consumes soft layers at 14 mm/s. Limit covers a whole adult rig.
    static RecessionFront contact(const std::array<float,3>& centre,float surfaceRadius,unsigned exposedTicks) noexcept {
        float depth=std::min(2.2f,float(std::min(3000u,exposedTicks>112u?exposedTicks-112u:0u))*(.014f/16.f));
        // Quantization bounds both atlas traffic and subpixel changes.
        depth=float(unsigned(depth*1000.f+.001f))*.001f;
        const float charRadius=std::max(surfaceRadius,depth>0.f?depth+.040f:0.f);
        const unsigned charMm=unsigned(std::lround(std::min(2.241f,charRadius)*1000.f));
        const unsigned maturity=20u+(235u*std::min(112u,exposedTicks))/112u;
        return {centre,depth,std::uint16_t(charMm),std::uint8_t(maturity)};
    }
    static bool valid(const RecessionFront& f) noexcept {
        if(!std::isfinite(f.radius)||f.radius<0.f||f.radius>2.201f||!f.charMm||f.charMm>2241||
           float(f.charMm)+1.f<f.radius*1000.f||!f.maturity)return false;
        for(float coordinate:f.centre)if(!std::isfinite(coordinate)||std::abs(coordinate)>10000.f)return false;
        return true;
    }
    bool update(const RecessionFront& incoming) noexcept {
        if(!valid(incoming))return false;
        unsigned nearest=0;float distance2=1.e30f;
        for(unsigned i=0;i<count;++i){
            float d2=0.f;
            for(unsigned axis=0;axis<3;++axis){
                const auto v=fronts[i].centre[axis]-incoming.centre[axis];d2+=v*v;
            }
            if(d2<distance2){nearest=i;distance2=d2;}
        }
        if(distance2>.000004f && count<Limit){fronts[count++]=incoming;return true;}
        auto& f=fronts[nearest];
        // When all slots are occupied, only grow an existing sphere to the
        // inscribed region proven by this contact. Never move or undo old loss.
        const float shift=std::sqrt(distance2);
        const float radius=std::max(f.radius,incoming.radius-shift);
        const int extent=std::max(int(f.charMm),int(incoming.charMm)-int(std::ceil(shift*1000.f)));
        const auto maturity=shift<.002f?std::max(f.maturity,incoming.maturity):f.maturity;
        if(radius==f.radius&&extent==int(f.charMm)&&f.maturity==maturity)return false;
        f.radius=radius;f.charMm=std::uint16_t(extent);f.maturity=maturity;return true;
    }
    template<class Texel> std::array<Texel,Words> encode() const noexcept {
        std::array<Texel,Words> row{};
        row[0].x=count;
        std::array<std::uint32_t,3> widths{};std::array<std::uint32_t,2> maturity{};
        for(unsigned i=0;i<count;++i){
            const auto& f=fronts[i];
            row[1+i]={burnBits(f.centre[0]),burnBits(f.centre[1]),burnBits(f.centre[2]),burnBits(f.radius)};
            widths[i/2]|=std::uint32_t(f.charMm)<<((i%2)*16);
            maturity[i/4]|=std::uint32_t(f.maturity)<<((i%4)*8);
        }
        row[0].y=widths[0];row[0].z=widths[1];row[0].w=widths[2];
        row[7].x=maturity[0];row[7].y=maturity[1];
        return row;
    }
};
}

