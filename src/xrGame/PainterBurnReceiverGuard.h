#pragma once
#include <cmath>
#include <algorithm>

namespace painter_burn_capture {
// A missing exact body pose remains opaque to other targets at this event.
template<class P>
inline double blockedAt(P o,P d,P centre,double radius,double reach) {
    const double x=o.x-centre.x,y=o.y-centre.y,z=o.z-centre.z;
    const double c=x*x+y*y+z*z-radius*radius;
    if(c<=0.)return 0.;
    const double b=x*d.x+y*d.y+z*d.z;
    if(b>=0.)return reach;
    const double discriminant=b*b-c;
    return discriminant<0.?reach:std::min(reach,-b-std::sqrt(discriminant));
}
}
