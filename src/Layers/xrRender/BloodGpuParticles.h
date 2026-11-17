#pragma once
#include <cstdint>

struct ID3D11Buffer;
namespace BloodGpuParticles
{
// Ordinary billboard input. Simulation, interpolation decisions, collision,
// visibility and graph/material order remain authoritative on the CPU.
struct Input
{
    float position[3]; std::uint32_t color;
    float top[3]; float radius_x;
    float right[3]; float radius_y;
    float sine, cosine, reserved[2];
};
static_assert(sizeof(Input) == 64, "particle compute input ABI");
constexpr std::uint32_t MaxSprites = 16384;
struct Counters
{
    std::uint64_t prepares = 0, batches = 0, sprites_prepared = 0, sprites_drawn = 0;
    std::uint64_t fallbacks = 0, capacity_fallbacks = 0, resource_failures = 0;
    std::uint64_t gpu_bytes = 0;
};
// Render thread, after the existing particle-worker Wait. No additional wait,
// readback, altered simulation step, draw reorder, or effect admission cap.
void Prepare(bool enabled);
bool Warmup() noexcept;
void Reset() noexcept;
Counters GetCounters() noexcept;

// Internal bridge shared with ParticleEffect.cpp. Exceeding the fixed batch
// capacity leaves those effects on their original CPU rendering path.
bool Dispatch(const Input* input, std::uint32_t count) noexcept;
ID3D11Buffer* Vertices() noexcept;
std::uint64_t Generation() noexcept;
void BeginPreparation() noexcept;
void CountDraw() noexcept;
void CountFallback(bool capacity = false) noexcept;
}
