#pragma once
#include <cstdint>

// Owner-thread state only. No engine pointer reaches an extraction worker.
namespace painter_voxel_binding {
enum class Phase { living, awaiting_death_shell, dead, rejected };
struct Lifecycle {
    bool observed_dead = false;
    std::uintptr_t death_shell = 0;

    Phase observe(bool alive, std::uintptr_t shell) noexcept {
        if (alive) {
            if (observed_dead || shell) return Phase::rejected;
            return Phase::living;
        }
        if (death_shell && shell != death_shell) return Phase::rejected;
        observed_dead = true;
        if (!shell) return Phase::awaiting_death_shell;
        if (!death_shell) death_shell = shell;
        return Phase::dead;
    }
};
}
