#pragma once
#include <algorithm>
#include <cmath>
#include <atomic>
#include <memory>

// One retained footprint. Growth changes its UV sampling, never its triangles
// or the number of overlapping decals. The blood shader clips UVs to [0,1].
struct BloodWallmarkGrowth
{
    static std::atomic<unsigned>& Live() noexcept { static std::atomic<unsigned> count{0}; return count; }
    struct Lease { ~Lease() { Live().fetch_sub(1,std::memory_order_relaxed); } };
    static std::shared_ptr<Lease> Reserve() noexcept
    {
        unsigned n=Live().load(std::memory_order_relaxed);
        do { if(n>=48)return {}; }
        while(!Live().compare_exchange_weak(n,n+1,std::memory_order_relaxed));
        try { return std::make_shared<Lease>(); }
        catch(...) { Live().fetch_sub(1,std::memory_order_relaxed);return {}; }
    }
    float opacity = 1.f;
    float seconds = 0.f;
    float initial_scale = 1.f;

    static bool Valid(float alpha, float duration) noexcept
    {
        return std::isfinite(alpha) && alpha >= 0.f && alpha <= .65f &&
            std::isfinite(duration) && duration >= .05f && duration <= 30.f;
    }
    float Scale(float age) const noexcept
    {
        if (seconds <= 0.f) return 1.f;
        const float t = (std::max)(0.f, (std::min)(1.f, age / seconds));
        const float ease = t * t * (3.f - 2.f * t);
        return initial_scale + (1.f - initial_scale) * ease;
    }
    bool Growing(float age) const noexcept { return seconds > 0.f && age < seconds; }
    float Fade(float age, float lifetime) const noexcept
    {
        const float fade = lifetime == -1.f ? 0.f :
            (std::max)(0.f, (std::min)(1.f, age / lifetime));
        // Existing pixel shaders multiply alpha by (1 - vertex alpha).
        return 1.f - opacity * (1.f - fade);
    }
    float UV(float uv, float age) const noexcept { return .5f + (uv - .5f) / Scale(age); }
};
