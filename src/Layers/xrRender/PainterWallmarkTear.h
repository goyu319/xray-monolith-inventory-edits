#pragma once
// Mask-only wounds, value 3 (painter_voxel_mask_only 3; maskdecal, 2026-11-12): the blood decals (skeleton wallmarks) of a piece
// with decide-once tears are hidden where the piece's exterior is hidden. Its exterior's painter_vmaskt_ programs hide a pixel that
// is removed (the removal field) OR lies in the gap / on the other side of one of its tears (the tear set and the anatomy field);
// the masked decal programs painter_wallmark_mask_tear[_owner] apply that same rule, with the same functions and textures, at the
// decal pixel's rest position. Values 0, 1 and 2 never reach this (no piece has a tear set): their decals keep today's
// painter_wallmark_mask[_owner|_owner_only] programs, which clip by the same removal field as their exteriors.
//  PainterWallmarkTear : what a masked decal of that piece binds besides the removal field (CKinematics::LL_RuntimeTearMask fills it
//                        from the piece's current publication, only when its exterior draws with the tear programs).
//  DrawsTear           : a cloned exterior material whose deferred pass is a painter_vmaskt_ program (RuntimeBindRequiredFields).
#ifdef USE_DX11
#include <cstring>
#include <memory>
struct PainterWallmarkTear {
    std::shared_ptr<const void> keep; // the publication's tear textures (PainterTearGpu): the identity and lifetime of the masked shader
    ref_texture params,map,bake;      // s_tear_params, s_tear_map, s_tear_bake (the exterior's own)
    ref_texture anatomyPages,anatomyData; // s_anatomy_pages, s_anatomy_data (the exterior's own)
    explicit operator bool()const noexcept{return bool(keep)&&bool(params)&&bool(map)&&bool(bake)&&bool(anatomyPages)&&bool(anatomyData);}
};
namespace PainterWallmarkTearMaterial {
inline bool DrawsTear(const ref_shader& shader){
    if(!shader)return false;
    for(unsigned n=0;n<2&&n<SHADER_ELEMENTS_MAX;++n){const auto& element=shader->E[n];if(!element)continue;
        for(const auto& pass:element->passes)
            if(pass&&pass->ps&&pass->ps->cName.c_str()&&std::strstr(pass->ps->cName.c_str(),"painter_maskt_deffer_base_"))return true;} // shaderslim: the tear piece's exterior program (painter_blood_painter_maskt_*, its atoc pre-pass painter_maskt_*)
    return false;
}
} // namespace PainterWallmarkTearMaterial
#endif
