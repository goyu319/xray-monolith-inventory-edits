#pragma once
#include <cstdint>
// Explicit authored IDs. Unknown/unregistered effects never acquire blood semantics
// from a parent group, source object, colour, material or texture.
namespace BloodEffect {
enum Id : std::uint32_t { None=0, LiquidParticle=1, MistParticle=2, Decal=3, Film=4 };
inline bool IsParticle(std::uint32_t id) noexcept { return id==LiquidParticle || id==MistParticle; }
inline bool IsSurface(std::uint32_t id) noexcept { return id==Decal || id==Film; }
inline bool IsBlood(std::uint32_t id) noexcept { return IsParticle(id) || IsSurface(id); }
}
