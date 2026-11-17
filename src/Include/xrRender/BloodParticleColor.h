#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

// Instance RGB only. Authored particle alpha, emission and motion are untouched.
struct BloodParticleColor
{
    float r=1.f,g=1.f,b=1.f;
    static bool Valid(float r,float g,float b) noexcept
    { return std::isfinite(r)&&std::isfinite(g)&&std::isfinite(b)&&r>=0&&g>=0&&b>=0&&r<=2&&g<=2&&b<=2; }
    bool Set(float red,float green,float blue) noexcept
    { if(!Valid(red,green,blue))return false;r=red;g=green;b=blue;return true; }
    bool Neutral() const noexcept { return r==1.f&&g==1.f&&b==1.f; }
    float Red(float value) const noexcept { return value*r; }
    float Green(float value) const noexcept { return value*g; }
    float Blue(float value) const noexcept { return value*b; }
    std::uint32_t Packed(std::uint32_t color) const noexcept
    {
        if(Neutral())return color;
        auto channel=[](unsigned v,float scale){return unsigned((std::min)(255.f,std::floor(v*scale)));};
        return (color&0xff000000u)|(channel((color>>16)&255,r)<<16)|
            (channel((color>>8)&255,g)<<8)|channel(color&255,b);
    }
};
