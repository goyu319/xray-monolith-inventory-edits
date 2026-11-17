#pragma once
#include <cstddef>
// Shared admission bounds for CPU fitting and native shell installation.
// Large, obliquely cut torso elements need more boxes than a small forearm
// to preserve the same 25 mm exterior-error limit. These are maxima, not
// target counts: fitting stops as soon as its geometric certificate passes.
namespace painter_collision_limits {
constexpr std::size_t boxes_per_element=96;
constexpr std::size_t boxes_per_shell=128;
}
