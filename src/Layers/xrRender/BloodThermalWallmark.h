#pragma once
#include "../../Include/xrRender/BloodThermal.h"
#include "../../Include/xrRender/BloodThermalStamp.h"
#include <algorithm>
#include "xrRender_console.h"
namespace BloodThermalWallmark {
inline bool Compatible(const ref_shader& shader) {
    if (!shader || !shader->E[0] || shader->E[0]->passes.empty()) return false;
    for (const auto& pass : shader->E[0]->passes) {
        if (!pass || !pass->ps) return false;
        xr_string name=*pass->ps->cName;
        if (name.size()>2 && name[name.size()-2]=='_' && name.back()>='0' && name.back()<='7') name.resize(name.size()-2);
        if (name!="effects_wallmark_blood") return false;
    }
    return true;
}
inline u32 Color(float age_seconds,unsigned alpha,bool blood,const BloodThermalStamp& stamp={}) {
    // This channel is unused by the blood normal-view program; never repurpose
    // RGB on a different material, or outside the active thermal render mode.
    if (ps_r2_heatvision<=0 || !blood) return color_rgba(128,128,128,alpha);
    // Compatible material selects transport; the explicit stamp ID alone admits heat.
    // thermal3: a mark without a stamp is blood by its material (wm_blood_* textures select effects_wallmark_blood: Compatible); it
    // cools from its own spawn time (age_seconds) below. A stamp of another kind stays cold.
    if (stamp.effect_id!=BloodEffect::None && !BloodEffect::IsSurface(stamp.effect_id)) return color_rgba(0,128,128,alpha);
    float heat=0.f;
    if (std::isfinite(age_seconds) && age_seconds>=0.f) {
        if (!heat_vision_cooldown) heat=1.f;
        else if (heat_vision_cooldown_time>0.f && age_seconds*1000.f<heat_vision_cooldown_time)
            heat=BloodThermal::Corpse(static_cast<u32>(age_seconds*1000.f),0,heat_vision_cooldown_time,true);
    }
    if(stamp.birth_known)
        heat=BloodThermal::Corpse(RDEVICE.dwTimeGlobal,stamp.blood_birth,heat_vision_cooldown_time,heat_vision_cooldown);
    if(stamp.source_known && stamp.source_corpse)
        heat=std::min(heat,BloodThermal::Corpse(RDEVICE.dwTimeGlobal,stamp.source_death,heat_vision_cooldown_time,heat_vision_cooldown));
    return color_rgba(iFloor(heat*255.f),128,128,alpha);
}
}
