#pragma once
#include <string>
#include <set>
#include <tuple>

namespace painter_material_warm {
struct Settings {
    int maskMode;
    bool burn;
    bool atoc;
    bool detailBump;
    int samples;
};

inline bool begins(const std::string& s, const char* prefix) { return s.find(prefix) == 0; }
inline bool has(const std::string& s, const char* text) { return s.find(text) != std::string::npos; }

// Called for files in the mounted program directory. Emit is (stage, source stem, skin macro, sample).
// PS skin is a compile option. It never forms part of the resource's PS name.
template<class Emit> void visit(const std::string& filename, const Settings& s, Emit emit) {
    const auto dot = filename.rfind('.');
    if (dot == std::string::npos) return;
    const auto stage = filename.substr(dot+1);
    const auto name = filename.substr(0,dot);
    if (!begins(name,"painter_")) return;
    if (stage == "cs") {
        if (s.burn && name == "painter_burn_bake") emit("cs",name,-1,-1);
        return;
    }
    if (stage == "vs") {
        if (begins(name,"painter_blood_deffer_model_") || name == "painter_blood_model_def_lq" || name == "painter_blood_ssfx_glass") {
            // OGFs can enter after the level opens. Their hardware vertex types are 0 through 4.
            for (int skin=0; skin<=4; ++skin) emit("vs",name,skin,-1);
        } else if (name == "painter_wallmark_mask") emit("vs",name,0,-1);
        else if (begins(name,"painter_mask_") || begins(name,"painter_maskd_") || begins(name,"painter_blood_painter_mask") ||
          name == "painter_anatomy" || (s.maskMode<2 && name=="painter_channel_shadow") ||
          (s.maskMode>0 && (begins(name,"painter_vmi") || begins(name,"painter_wall_bounds_")))) emit("vs",name,4,-1);
        else if (begins(name,"painter_blood_thermal")) emit("vs",name,-1,-1);
        return;
    }
    if (stage != "ps") return;
    if (name == "painter_vmi_shadow_piece" || name == "painter_burn_painter_vmi_shadow_piece") return;
    if (begins(name,"painter_region_impl_")) return; // included shader fragments, no entry point
    if (!s.burn && begins(name,"painter_burn_")) return;
    if (!s.atoc && has(name,"_atoc_")) return;
    if (!s.detailBump && has(name,"_db-")) return;
    const bool tree = begins(name,"painter_leaf_") || begins(name,"painter_wood_deffer_");
    // Model VS recipes have no lightmap outputs. Tree visuals can carry an authored level lightmap.
    if (!tree && has(name,"_lmh_")) return;
    // Native model/tree blenders leave canUseSteepParallax false. Explicit tree-branch recipes are scripts.
    // Authored/custom shader passes already loaded are additionally resolved through the actual pair collector.
    if (has(name,"_steep")) return;
    if (has(name,"_vmi_")) {
        const bool piece = has(name,"_piece"), tear = has(name,"_tear");
        if (s.maskMode < (tear?3:(piece?2:1))) return;
    }
    if (has(name,"_maskt_")) {
        if (s.maskMode<3) return;
        if (begins(name,"painter_maskt_") && !has(name,"shadow") && !has(name,"_atoc_")) return;
    }
    if (begins(name,"painter_channel") && s.maskMode>=2) return;
    if (begins(name,"painter_static_blood")) {
        emit("ps",name,-1,-1);
        if (s.samples>1) for (int i=0;i<s.samples;++i) emit("ps",name,-1,i);
        return;
    }
    const bool material = begins(name,"painter_mask") || begins(name,"painter_owned_") || begins(name,"painter_region_") ||
        begins(name,"painter_blood_") || begins(name,"painter_burn_") || begins(name,"painter_vmi_") || tree ||
        begins(name,"painter_wallmark_mask") || begins(name,"painter_channel") || name=="painter_anatomy" || begins(name,"painter_hitfx_");
    if (!material) return;
    // WarmLevelInteriors requests these before the blood pump. Use its exact compile permutation.
    const bool cloned = begins(name,"painter_mask") || begins(name,"painter_owned_") || begins(name,"painter_region_") || begins(name,"painter_blood_painter_maskt_") || begins(name,"painter_blood_painter_vmi_") || begins(name,"painter_vmi_") || begins(name,"painter_channel") || name=="painter_anatomy";
    const int skin = begins(name,"painter_wallmark_mask") ? 0 : (cloned ? 4 : -1);
    emit("ps",name,skin,-1);
}

template<class Emit>
void visitLiveSettings(const std::string& filename, Settings settings, Emit emit) {
    settings.burn = true;
    std::set<std::tuple<std::string,int,int>> seen;
    for (int mode = 0; mode <= 3; ++mode) {
        settings.maskMode = mode;
        visit(filename,settings,[&](const char* stage,const std::string& name,int skin,int sample) {
            // Each invocation covers only one file, so its stage is already fixed.
            if (seen.emplace(name,skin,sample).second) emit(stage,name,skin,sample);
        });
    }
}
}

