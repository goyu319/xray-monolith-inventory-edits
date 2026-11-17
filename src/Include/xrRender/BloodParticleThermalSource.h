#pragma once
#include <cstdint>
// Values only. Never retain a game object in an effect/group or render task.
struct BloodParticleThermalSource {
    bool known=false;
    bool corpse=false;
    std::uint32_t death_time=0;
};
