#pragma once
// Version 1 native deformation policy. Plain value snapshot: no engine/Lua pointers on workers.
#include <cmath>

namespace gfy_live {
struct GoreControls {
    double surface_burst_frequency_mult=1.0;
    double surface_burst_intensity_mult=1.0;
    double head_blowout_frequency_mult=1.0;
    double head_blowout_intensity_mult=1.0;
    double lateral_vent_frequency_mult=1.0;
    double lateral_vent_intensity_mult=1.0;
    double cavitation_mult=1.0;
    double exit_tearing_mult=1.0;
    double wound_variation_mult=1.0;
    bool head_remnants_enabled=true;
    double head_tether_strength_mult=1.0;
    int eye_remnants_mode=1;
    bool intestines_enabled=true;
    double intestine_amount_mult=1.0;
    double intestine_frequency_mult=1.0;
    double intestine_tether_strength_mult=1.0;
};
inline bool valid_gore_controls(const GoreControls& c) noexcept {
    return std::isfinite(c.surface_burst_frequency_mult)&&c.surface_burst_frequency_mult>=0&&c.surface_burst_frequency_mult<=2
        && std::isfinite(c.surface_burst_intensity_mult)&&c.surface_burst_intensity_mult>=0&&c.surface_burst_intensity_mult<=2
        && std::isfinite(c.head_blowout_frequency_mult)&&c.head_blowout_frequency_mult>=0&&c.head_blowout_frequency_mult<=2
        && std::isfinite(c.head_blowout_intensity_mult)&&c.head_blowout_intensity_mult>=0&&c.head_blowout_intensity_mult<=2
        && std::isfinite(c.lateral_vent_frequency_mult)&&c.lateral_vent_frequency_mult>=0&&c.lateral_vent_frequency_mult<=2
        && std::isfinite(c.lateral_vent_intensity_mult)&&c.lateral_vent_intensity_mult>=0&&c.lateral_vent_intensity_mult<=2
        && std::isfinite(c.cavitation_mult)&&c.cavitation_mult>=0&&c.cavitation_mult<=2
        && std::isfinite(c.exit_tearing_mult)&&c.exit_tearing_mult>=0&&c.exit_tearing_mult<=2
        && std::isfinite(c.wound_variation_mult)&&c.wound_variation_mult>=0&&c.wound_variation_mult<=2
        && std::isfinite(c.head_tether_strength_mult)&&c.head_tether_strength_mult>=0.25&&c.head_tether_strength_mult<=4
        && c.eye_remnants_mode>=0&&c.eye_remnants_mode<=1
        && std::isfinite(c.intestine_amount_mult)&&c.intestine_amount_mult>=0&&c.intestine_amount_mult<=3
        && std::isfinite(c.intestine_frequency_mult)&&c.intestine_frequency_mult>=0&&c.intestine_frequency_mult<=2
        && std::isfinite(c.intestine_tether_strength_mult)&&c.intestine_tether_strength_mult>=0.25&&c.intestine_tether_strength_mult<=4;
}
}

