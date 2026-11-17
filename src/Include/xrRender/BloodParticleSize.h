#pragma once
#include <cmath>

// Per spawned effect. Authored definitions, velocities, emission and neighbors
// stay immutable. The same factor applies after velocity stretching.
struct BloodParticleSize
{
    float factor=1.f;
    bool Set(float value) noexcept
    {
        if (!std::isfinite(value) || value<.1f || value>4.f) return false;
        factor=value;return true;
    }
    float Radius(float diameter,float stretch=0.f,float speed=0.f) const noexcept
    { return (.5f*diameter+stretch*speed)*factor; }
    float Bounds(float diameter) const noexcept { return diameter*factor; }
};
