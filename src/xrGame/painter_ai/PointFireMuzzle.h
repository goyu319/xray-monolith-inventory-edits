#pragma once
#include "core/PointFireAuthority.h"
#include <cmath>
#ifdef _MSC_VER
// This portable helper deliberately uses double-precision standard math.
// Engine headers mark those names deprecated in favor of float intrinsics.
#pragma warning(push)
#pragma warning(disable:4995)
#endif

namespace painter { namespace combat {
// Actual native direction, including pitch; desired sight is only a request.
inline bool point_muzzle_aligned(const Vec3& actual,const Vec3& wanted,float radians){
    if(!core::point_finite(actual)||!core::point_finite(wanted)||!std::isfinite(radians)||radians<0||radians>=1.570796327f)return false;
    const double aa=double(actual.x)*actual.x+double(actual.y)*actual.y+double(actual.z)*actual.z;
    const double bb=double(wanted.x)*wanted.x+double(wanted.y)*wanted.y+double(wanted.z)*wanted.z;
    if(aa<=0||bb<=0)return false;
    const double dot=double(actual.x)*wanted.x+double(actual.y)*wanted.y+double(actual.z)*wanted.z;
    return dot/std::sqrt(aa*bb)>=std::cos(double(radians));
}
}}
#ifdef _MSC_VER
#pragma warning(pop)
#endif
