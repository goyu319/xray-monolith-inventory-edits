#pragma once
#include "PartChildOwnership.h"

namespace painter_damaged_fragment {
// A retained original child has no exported weighted-bone palette to validate
// against the fitted shell. Packed replacement children carry those weights.
inline bool has_uncertified_original_accessory(
    const std::vector<std::vector<part_child_ownership::Decision>>& plans) noexcept {
    for(const auto& plan:plans)
        for(const auto& decision:plan)
            if(decision.action==part_child_ownership::Action::RetainOriginalAccessory)
                return true;
    return false;
}
}
