#pragma once
#ifdef USE_DX11
#include "PainterRemovalGpu.h"
#include "PainterBloodOwner.h"
#include "PainterWallmarkOwnership.h"
#include "ResourceManager.h"
#include "PainterSoftShader.h"
#include "BloodNativeProfile.h"
#include "PainterWallmarkTear.h" // maskdecal: a value-3 piece's tear set for its decals
namespace PainterWallmarkMask {
struct Entry {std::weak_ptr<PainterRemovalGpu> field;std::weak_ptr<PainterWallmarkOwnershipGpu> ownership;bool has_field=false,has_ownership=false;ref_shader original,masked;std::weak_ptr<const void> tear;bool has_tear=false;}; // maskdecal: tear = the piece's tear set (value 3)
inline std::vector<Entry>&entries(){static auto*v=new std::vector<Entry>;return *v;}
inline void Collect(){auto&v=entries();v.erase(std::remove_if(v.begin(),v.end(),[](const auto&e){return (e.has_field&&e.field.expired())||(e.has_ownership&&e.ownership.expired())||(e.has_tear&&e.tear.expired());}),v.end());}
inline void Clear(){entries().clear();}
inline unsigned long long& assemblies(){static unsigned long long n=0;return n;} // R37: shaders assembled (cache misses)
// All preparation occurs before masked draws. Unsupported original material
// passes require same-candidate geometry fallback, never an unmasked overlay.
// maskdecal: tearMask = a value-3 piece's tear set and anatomy field (CKinematics::LL_RuntimeTearMask); null or empty = today exactly.
inline bool Prepare(CResourceManager&resources,const std::shared_ptr<PainterRemovalGpu>&field,const std::shared_ptr<PainterWallmarkOwnershipGpu>&ownership,
    const ref_shader&original,ref_shader&result,xr_string&reason,const PainterWallmarkTear*tearMask=nullptr){
    if(!original||(!field&&!ownership)||(field&&(!field->registered||field->retired||!field->page_texture||!field->data_texture))||
        (ownership&&(!ownership->registered||ownership->retired||!ownership->plane_texture||!ownership->region_texture))){reason="wallmark_mask_field";return false;}
    const bool tear=tearMask&&bool(*tearMask)&&bool(field); // maskdecal: with the removal field only (as the exterior's tear programs)
    for(auto&e:entries())if(e.original==original&&e.has_field==bool(field)&&e.has_ownership==bool(ownership)&&e.field.lock()==field&&e.ownership.lock()==ownership&&
        e.has_tear==tear&&(!tear||e.tear.lock()==tearMask->keep)){result=e.masked;return true;} // R22: cheap lookup first; collect only before inserting
    Collect();
    ++assemblies();BloodNativeProfile::Scope profileAssembly(BloodNativeProfile::Stage::WallmarkMaskAssembly,Device.dwFrame);
    if(!FS.exist("$game_shaders$","r3\\painter_wallmark_mask.vs")||!FS.exist("$game_shaders$","r3\\painter_wallmark_mask.ps")){reason="wallmark_mask_shader_missing";return false;}
    const char*pixel=ownership?(field?"painter_wallmark_mask_owner":"painter_wallmark_mask_owner_only"):"painter_wallmark_mask";
    struct Restore {int skin=Engine.External.GetSkinningMode(),sample=::Render->m_MSAASample;~Restore(){Engine.External.SetSkinningMode(skin);::Render->m_MSAASample=sample;}}restore;
    // maskdecal: one masked copy of the original's passes with pixel program `program`; withTear: + the tear set and the anatomy field
    auto assemble=[&](const char*program,const PainterWallmarkTear*withTear,ref_shader&completed)->bool{
    Shader assembled;unsigned passes=0;
    for(unsigned e=0;e<SHADER_ELEMENTS_MAX;++e){const auto&oldElement=original->E[e];if(!oldElement)continue;
        ShaderElement element;element._copy(*oldElement);element.passes.clear();
        for(const auto&old:oldElement->passes){
            if(!old||!old->vs||!old->ps||!old->vs->vs||!old->ps->ps||old->vs->skinning>0||
                (old->gs&&old->gs->gs)||(old->hs&&old->hs->sh)||(old->ds&&old->ds->sh)||(old->cs&&old->cs->sh)){
                reason="wallmark_mask_unsupported_stage";return false;}
            xr_string vs=*old->vs->cName,ps=*old->ps->cName;int sample=-1;
            if(ps.size()>2&&ps[ps.size()-2]=='_'&&ps.back()>='0'&&ps.back()<='7'){sample=ps.back()-'0';ps.resize(ps.size()-2);}
            if(vs!="effects_wallmark_blood"||ps!="effects_wallmark_blood"){reason="wallmark_mask_unsupported_material";return false;}
            Engine.External.SetSkinningMode(0);::Render->m_MSAASample=sample;
            SPass pass;pass.state=old->state;pass.C=old->C;pass.gs=old->gs;pass.hs=old->hs;pass.ds=old->ds;pass.cs=old->cs;
            {PainterSoftShaderGuard soft;pass.vs=resources._CreateVS("painter_wallmark_mask");pass.ps=resources._CreatePS(program);}
            if(!pass.vs||!pass.ps||!pass.vs->vs||!pass.ps->ps){reason="wallmark_mask_program_creation";return false;}
            R_constant_table constants;constants.merge(&pass.ps->constants);constants.merge(&pass.vs->constants);
            STextureList textures;unsigned bindings=0;
            for(auto&c:constants.table){
                const ref_texture*binding=nullptr;
                if(field&&!xr_strcmp(*c->name,"s_painter_removal_pages"))binding=&field->page_texture;
                if(field&&!xr_strcmp(*c->name,"s_painter_removal_data"))binding=&field->data_texture;
                if(ownership&&!xr_strcmp(*c->name,"s_painter_decal_planes"))binding=&ownership->plane_texture;
                if(ownership&&!xr_strcmp(*c->name,"s_painter_decal_regions"))binding=&ownership->region_texture;
                if(withTear&&!xr_strcmp(*c->name,"s_anatomy_pages"))binding=&withTear->anatomyPages; // maskdecal: the exterior's own
                if(withTear&&!xr_strcmp(*c->name,"s_anatomy_data"))binding=&withTear->anatomyData;
                if(withTear&&!xr_strcmp(*c->name,"s_tear_params"))binding=&withTear->params;
                if(withTear&&!xr_strcmp(*c->name,"s_tear_map"))binding=&withTear->map;
                if(withTear&&!xr_strcmp(*c->name,"s_tear_bake"))binding=&withTear->bake;
                if(binding){if(c->type!=RC_dx10texture){reason="wallmark_mask_resource_type";return false;}textures.emplace_back(c->samp.index,*binding);++bindings;continue;}
                // The installed decal pass may shade blood without using the SSS console vector.
                if (!xr_strcmp(*c->name,"ssfx_blood_decals") && c->type==RC_float) {
                    c->handler=PainterBloodDecalControls();
                    continue;
                }
                auto*prior=old->constants?old->constants->get(*c->name):nullptr;
                if(!prior||prior->type!=c->type){reason="wallmark_mask_constant";return false;}
                c->handler=prior->handler;
                if(c->type==RC_sampler&&c->samp.index!=prior->samp.index){reason="wallmark_mask_sampler";return false;}
                if(c->type==RC_dx10texture){bool found=false;if(old->T)for(const auto&t:*old->T)if(t.first==prior->samp.index){textures.emplace_back(c->samp.index,t.second);found=true;break;}
                    if(!found){reason="wallmark_mask_original_texture";return false;}}
            }
            if(bindings!=unsigned((field?2:0)+(ownership?2:0)+(withTear?5:0))){reason="wallmark_mask_reflection";return false;}
            pass.constants=resources._CreateConstantTable(constants);pass.T=resources._CreateTextureList(textures);
            element.passes.push_back(resources._CreatePass(pass));++passes;
        }
        assembled.E[e]=resources._CreateElement(element);
    }
    if(!passes){reason="wallmark_mask_empty";return false;}
    completed=resources._CreateShader(&assembled);return true;
    };
    ref_shader completed;
    if(tear){ // maskdecal (value 3): refused -> today's removal-only program for this publication (logged once per publication and material)
        const char*tearPixel=ownership?"painter_wallmark_mask_tear_owner":"painter_wallmark_mask_tear";
        string_path path;strconcat(sizeof(path),path,"r3\\",tearPixel,".ps");
        if(!FS.exist("$game_shaders$",path))reason="wallmark_mask_tear_shader_missing";else assemble(tearPixel,tearMask,completed);
        if(!completed)Msg("! [painter_mask_only] decal_tear_program_refused why=%s ps=%s (this piece's decals clip by its removal field only)",reason.c_str(),tearPixel);
    }
    if(!completed&&!assemble(pixel,nullptr,completed))return false;
    entries().push_back({field,ownership,bool(field),bool(ownership),original,completed,tear?tearMask->keep:std::shared_ptr<const void>(),tear});result=completed;reason.clear();return true;
}
} // namespace PainterWallmarkMask
#endif
