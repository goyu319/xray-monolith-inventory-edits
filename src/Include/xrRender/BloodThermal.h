#pragma once
#include <cstdint>
#include <cmath>
namespace BloodThermal {
// Native CEntity linear cooling, including its u32 clamp for finite positive
// configured durations. An explicit zero duration is immediately cold, not NaN.
inline float Corpse(std::uint32_t now, std::uint32_t death, float duration_ms, bool cooling) noexcept {
    if (!cooling) return 1.f;
    if (!(duration_ms > 0.f) || !std::isfinite(duration_ms)) return 0.f;
    const std::uint32_t elapsed=now-death;
    const std::uint32_t cap=static_cast<std::uint32_t>(duration_ms);
    return 1.f-static_cast<float>(elapsed>cap ? cap : elapsed)/duration_ms;
}
}
