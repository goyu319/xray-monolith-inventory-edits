#pragma once
#include <algorithm>
#include <cmath>

namespace PainterWrapBounds
{
struct Bounds
{
    float x, y, z, w, h, d, thickness;
};

// Axis-aligned proxy of the tilted and tapered annulus at all sampled heights.
// Extrapolation is clamped to the same interval in the pixel shader.
template<class Section>
Bounds get(const Section& s, float lowY, float height, float scale)
{
    const float thick=std::min(.34f,std::max(.13f,.11f+.13f*scale));
    const float margin=thick*3.90f;
    float loX=1.e20f,loZ=loX,hiX=-loX,hiZ=-loX;
    for(int i=0;i<2;++i)
    {
        const float k=i?1.35f:-.35f;
        const float x=s.x0+k*(s.x1-s.x0),z=s.z0+k*(s.z1-s.z0);
        const float rx=std::max(.04f,s.rx0+k*(s.rx1-s.rx0)),rz=std::max(.04f,s.rz0+k*(s.rz1-s.rz0));
        loX=std::min(loX,x-rx-margin);hiX=std::max(hiX,x+rx+margin);
        loZ=std::min(loZ,z-rz-margin);hiZ=std::max(hiZ,z+rz+margin);
    }
    return {loX,lowY,loZ,hiX-loX,height,hiZ-loZ,thick};
}
}
