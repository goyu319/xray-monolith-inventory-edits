#pragma once
#include <algorithm>
#include <cmath>

namespace PainterTreeDates
{
struct Span { float start=0.f,end=0.f; };
inline float progress(Span s,float now){
    const float x=std::clamp((now-s.start)/std::max(s.end-s.start,.25f),0.f,1.f);
    return x*x*(3.f-2.f*x);
}
inline float heat(Span s,float now){
    return std::clamp((now-s.start)/2.f,0.f,1.f)*std::clamp(1.f-(now-s.end)/24.f,0.f,1.f);
}
inline float thermal(Span s,float now)
{
    if(now<=s.start||now>=s.end+300.f)return 0.f;
    const float lit=std::clamp((now-s.start)/2.f,0.f,1.f);
    if(now<=s.end)return lit*.85f;
    const float age=now-s.end;
    const float residual=1.f-age/300.f;
    return lit*(.45f*std::exp2(-age/6.f)+.40f*residual*residual);
}
inline void include(Span& a,Span b){
    a.start=std::min(a.start,b.start);a.end=std::max(a.end,b.end);
}
}
