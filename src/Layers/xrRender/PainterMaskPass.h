#pragma once
// Mask-only wounds (painter_voxel_mask_only 1..3): the passes beside the deferred volume pass (maskpass, 2026-11-12).
//  volumeEligible : the removal-clone passes that try the volume programs (PainterRemovalMaterial::clone, attempt order and
//                   fallback unchanged: painter_vmaskt_ / painter_vmaskp_ / painter_vmask_ <same variant>, then today's
//                   painter_mask_ program when a file is missing, a compile fails or a binding is refused):
//                     - the deferred exterior (slices 1-3, as before),
//                     - its alpha-to-coverage pre-pass (DX10.0 MSAA alpha test: painter_mask_deffer_base_atoc_*),
//                     - the shadow passes (painter_mask_shadow_direct_model[_aref] + painter_mask_shadow_opaque /
//                       painter_mask_shadow_direct_base_aref): a hidden exterior pixel stays in the shadow map where the volume
//                       keeps this piece's tissue behind it along the light ray.
//                   Switch 0 never reaches it (no anatomy material is passed to the clone).
//  IsBox / BoxOnly: the coverage box child of a mask piece (slices 2, 3: every vertex carries u = -1000) is drawn only by the
//                   deferred volume passes of the normal elements; no shadow element, no other pass, no wallmarks.
#ifdef USE_DX11
#include "../../Include/xrRender/RuntimePackedMesh.h"
namespace PainterMaskPass {
inline bool volumeEligible(const xr_string& vertex,const xr_string& pixel){
    if(vertex.find("painter_mask_deffer_model_")==0&&pixel.find("painter_mask_deffer_base_")==0)return true;
    if(vertex=="painter_mask_shadow_direct_model"&&pixel=="painter_mask_shadow_opaque")return true;
    if(vertex=="painter_mask_shadow_direct_model_aref"&&pixel=="painter_mask_shadow_direct_base_aref")return true;
    return false;
}
// slices 2 and 3 (MaskPieces.h, MaskPieces3.h): the coverage box child, 24 vertices with texture u = -1000
inline bool IsBox(const gfy_packed::Child& c){
    if(c.material||c.vertices.empty())return false;
    for(const auto& v:c.vertices)if(!(v.surface[12]< -500.f))return false;
    return true;
}
// The box keeps only the deferred volume passes (painter_vmaskp_ / painter_vmaskt_ deferred exterior programs and their
// alpha-to-coverage pre-pass) of SE_R2_NORMAL_HQ / SE_R2_NORMAL_LQ. Returns the number of elements kept; 0 = the box draws nothing
// (no volume program was made for its material: today's clip would draw it as flat textured patches).
inline unsigned BoxOnly(CResourceManager& resources,ref_shader& shader){
    if(!shader)return 0;
    Shader box;unsigned kept=0;
    for(unsigned n=0;n<2&&n<SHADER_ELEMENTS_MAX;++n){
        const auto& source=shader->E[n];if(!source)continue;
        ShaderElement element;element._copy(*source);element.passes.clear();
        for(const auto& pass:source->passes){
            if(!pass||!pass->ps||!pass->ps->cName.c_str())continue;
            const xr_string ps=pass->ps->cName.c_str();
            // shaderslim: the shared interior programs of a piece (deferred: the blood-capable painter_blood_painter_vmi_*; its atoc pre-pass)
            if(ps.find("painter_blood_painter_vmi_")==0||ps.find("painter_vmi_")==0)element.passes.push_back(pass);
        }
        if(element.passes.empty())continue;
        box.E[n]=resources._CreateElement(element);++kept;
    }
    ref_shader complete;complete=resources._CreateShader(&box);shader=complete;
    return kept;
}
} // namespace PainterMaskPass
#endif
