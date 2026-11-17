#pragma once
#include <cmath>

namespace painter_arm_pose_math {
inline bool rigid(const Fmatrix& m)
{
    if (!_valid(m)) return false;
    Fvector cross; cross.crossproduct(m.i,m.j);
    return _abs(m.i.square_magnitude()-1.f)<.002f &&
        _abs(m.j.square_magnitude()-1.f)<.002f &&
        _abs(m.k.square_magnitude()-1.f)<.002f &&
        _abs(m.i.dotproduct(m.j))<.002f && _abs(m.i.dotproduct(m.k))<.002f &&
        _abs(m.j.dotproduct(m.k))<.002f && _abs(cross.dotproduct(m.k)-1.f)<.003f &&
        _abs(m._14)<1e-5f && _abs(m._24)<1e-5f && _abs(m._34)<1e-5f && _abs(m._44-1.f)<1e-5f;
}

// World-space logarithmic angular difference. COM velocity is measured at the
// actual newly built element's local COM, not at the bone origin or NPC spine.
inline bool velocity(const Fmatrix& previous, const Fmatrix& current,
                     const Fvector& local_com, float dt, Fvector& linear, Fvector& angular)
{
    if (!rigid(previous)||!rigid(current)||!_valid(local_com)||
        !std::isfinite(dt)||dt<.004f||dt>.250f) return false;
    Fvector before,after;
    previous.transform_tiny(before,local_com); current.transform_tiny(after,local_com);
    linear.sub(after,before).div(dt);
    Fmatrix inverse,delta; inverse.invert(previous); delta.mul_43(current,inverse);
    Fquaternion q; q.set(delta); q.normalize();
    if (q.w<0.f) { q.x=-q.x;q.y=-q.y;q.z=-q.z;q.w=-q.w; }
    Fvector axis; float angle; q.get_axis_angle(axis,angle);
    // XRay stores basis vectors in rows; PHDynamicData transposes them for
    // ODE. Its matrix->quaternion convention therefore has the opposite vector
    // sign from ODE's world angular velocity. Verified using actual xrCore
    // headers against +X -> +Y physical rotation, including nonzero starts.
    angular.mul(axis,-angle/dt);
    // Fail closed on a teleport/stale sample. Do not clamp and call it inherited.
    return _valid(linear)&&_valid(angular)&&linear.square_magnitude()<=10000.f&&angular.square_magnitude()<=10000.f;
}
}
