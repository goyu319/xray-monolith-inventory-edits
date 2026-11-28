#pragma once
namespace PainterSurfaceHeat
{
// Game seconds since extinction. The zero timestamp denotes burning fuel.
inline float level(float now,float extinguished,bool cooling)
{
    if(extinguished<=0.f)return .86f;
    if(!cooling)return .63f;
    const float age=now>extinguished?now-extinguished:0.f;
    if(age>=300.f)return 0.f;
    const float remaining=1.f-age/300.f;
    return .63f*remaining*remaining;
}
inline unsigned char byte(float value){return static_cast<unsigned char>(255.f*value+.5f);}
}
