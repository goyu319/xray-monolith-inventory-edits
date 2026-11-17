#pragma once
#ifdef USE_DX11
// Main-thread-only immutable material cloning. No global shader-cache entry or
// source visual is modified. Caller owns the returned handle and mask textures.
#include "ResourceManager.h"
#include "PainterSoftShader.h"
#include "PainterRemovalPrograms.h"
#include "PainterMaskPass.h" // maskpass: the shadow passes and the alpha-to-coverage pre-pass try the volume programs too
#include "PainterBloodOwner.h" // shaderslim: PainterBloodDecalControls (the blood-capable volume programs' live decal controls)
#include "PainterBloodPrograms.h" // bloodfix: the warm blood aliases of the plain surface programs
namespace PainterRemovalMaterial {
struct ProgramPair {xr_string vertex,pixel;int sample=-1;bool glass=false,forwardModel=false;};
inline bool programPair(const SPass& pass,ProgramPair& pair){
    if(!pass.vs||!pass.ps||!pass.vs->vs||!pass.ps->ps||pass.vs->skinning!=4)return false;
    if((pass.gs&&pass.gs->gs)||(pass.hs&&pass.hs->sh)||(pass.ds&&pass.ds->sh)||(pass.cs&&pass.cs->sh))return false;
    xr_string vertex=*pass.vs->cName,pixel=*pass.ps->cName;
    if(vertex.size()<2||vertex.substr(vertex.size()-2)!="_4")return false;
    vertex.resize(vertex.size()-2);
    pair.sample=-1;
    if(pixel.size()>2&&pixel[pixel.size()-2]=='_'&&pixel.back()>='0'&&pixel.back()<='7'){
        pair.sample=pixel.back()-'0';pixel.resize(pixel.size()-2);
    }
    pair.glass=vertex=="ssfx_glass"&&pixel=="ssfx_glass";
    pair.forwardModel=vertex=="model_def_lq"&&pixel=="model_def_lq";
    const bool shadow=vertex=="shadow_direct_model",alphaShadow=vertex=="shadow_direct_model_aref";
    if(pair.glass||pair.forwardModel){}
    else if(shadow){if(pixel!="dumb")return false;}
    else if(alphaShadow){if(pixel!="shadow_direct_base_aref")return false;}
    else {
        const bool flat=vertex.find("flat")!=xr_string::npos;
        if(pixel.find("deffer_base")!=0||((pixel.find("flat")!=xr_string::npos)!=flat))return false;
    }
    const char* vs=maskedProgram(vertex.c_str(),true);const char* ps=maskedProgram(pixel.c_str(),false);
    if(!vs||!ps)return false;
    pair.vertex=vs;pair.pixel=ps;
    return true;
}
inline bool canClone(const ref_shader& source){
    if(!source)return false;unsigned passes=0;
    for(const auto& e:source->E)if(e){for(const auto& p:e->passes){ProgramPair pair;if(!p||!programPair(*p,pair)||(e->flags.bStrictB2F&&!pair.glass&&!pair.forwardModel))return false;++passes;}}
    return passes!=0;
}
// shaderslim: every program pair the clone can create for one original pass - (vertex, pixel, MSAA sample) - for the warm queue
// (RuntimeShaderCache): created behind the loading screen when a Painter body binds, so no clone compiles in play. mode = the
// value of painter_voxel_mask_only: 0 the plain clip only; 1 + the wound walls' interior pair; 2 + a detached piece's; 3 + the
// tear pieces' pairs (a value-3 piece that came off without a tear still draws with the piece pair).
// A name without a file is skipped by the queue.
// The volume passes' programs by name (clone and warm queue use the same rule). The deferred pass's tear exterior and interior
// programs are the BLOOD-CAPABLE ones (painter_blood_painter_maskt_<recipe>, painter_blood_painter_vmi_<job>): a body without baked
// blood draws them with the blood layer off (clone), a bloodied one with its blood (PainterBloodBake::clone rebinds the same
// programs) - one compile instead of a plain and a blood twin. Shadow / alpha-to-coverage passes carry no blood: plain names.
struct VolumeNames {xr_string vertexA,dentA,pixelA,vertexB,dentB,pixelB;};
inline VolumeNames volumeNames(const ProgramPair& pair,bool withTear,const char* job){
    const bool shadowPass=pair.vertex.find("painter_mask_shadow_direct_model")==0,atocPass=pair.pixel.find("_atoc_")!=xr_string::npos;
    const bool deferred=!shadowPass&&!atocPass;
    VolumeNames names;
    names.vertexA=pair.vertex;names.dentA=xr_string("painter_maskd_")+pair.vertex.substr(13);names.pixelA=pair.pixel;
    if(withTear){
        names.pixelA=xr_string("painter_maskt_")+pair.pixel.substr(13);
        if(deferred){names.vertexA="painter_blood_"+names.vertexA;names.dentA="painter_blood_"+names.dentA;names.pixelA="painter_blood_"+names.pixelA;}
    }
    names.vertexB=shadowPass?"painter_vmi_shadow_model":"painter_vmi_model";names.dentB=shadowPass?"painter_vmid_shadow_model":"painter_vmid_model";
    names.pixelB=xr_string(deferred?"painter_blood_painter_vmi_":(shadowPass?"painter_vmi_shadow_":"painter_vmi_atoc_"))+job;
    return names;
}
template<class Request> inline void clonePrograms(const SPass& pass,int mode,Request&& request){
    ProgramPair pair;if(!programPair(pass,pair))return;
    request(pair.vertex,pair.pixel,pair.sample);request(xr_string("painter_maskd_")+pair.vertex.substr(13),pair.pixel,pair.sample);
    if(mode<=0||pair.sample>=0||pair.glass||pair.forwardModel||!PainterMaskPass::volumeEligible(pair.vertex,pair.pixel))return;
    for(const char* job:{"wall","piece","tear"}){
        const bool withTear=!xr_strcmp(job,"tear");
        if((withTear&&mode<3)||(!xr_strcmp(job,"piece")&&mode<2))continue;
        const VolumeNames names=volumeNames(pair,withTear,job);
        if(withTear){request(names.vertexA,names.pixelA,-1);request(names.dentA,names.pixelA,-1);}
        request(names.vertexB,names.pixelB,-1);request(names.dentB,names.pixelB,-1);
    }
}
// shaderslim: the blood layer's constants of a blood-capable pass on a body without baked blood: off (painter_blood_cells.h reads
// nothing while painter_blood_clock.w is 0; painter_blood_target 0 = no clamp)
struct BloodOff: R_constant_setup { void setup(R_constant* c) override { RCache.set_c(c,0.f,0.f,0.f,0.f); } };
struct LiveBlood {ref_texture table;R_constant_setup* clock=nullptr;R_constant_setup* target=nullptr;};
inline const LiveBlood*& liveBlood(){static const LiveBlood* value=nullptr;return value;}
inline BloodOff& bloodOff(){static BloodOff value;return value;}
// tissue/anatomyPages/anatomyData: mask-only wounds (RuntimeRequiredFields mask_volume); null = today's clone exactly.
inline bool clone(CResourceManager& resources,const ref_shader& source,const ref_texture& pages,const ref_texture& bits,const ref_texture& planes,const ref_texture& regions,bool removal,ref_shader& destination,xr_string& why,
    const Shader* tissue=nullptr,const ref_texture& anatomyPages=ref_texture(),const ref_texture& anatomyData=ref_texture(),bool piece=false,
    const ref_texture& tearParams=ref_texture(),const ref_texture& tearMap=ref_texture(),const ref_texture& tearBake=ref_texture(),
    const ref_texture& dents=ref_texture(), // melee (value 3): a body's dents (PainterDentGpu): every pass uses the dent twin of its vertex program
    const ref_texture& slits=ref_texture()){ // melee: the blade slits every value-3 exterior entry reads (s_painter_slits; painter_slit.h) // slice 3: a piece's baked tear
    const bool ownership=bool(planes)&&bool(regions);
    if((removal&&(!pages||!bits))||(!removal&&!ownership)||bool(planes)!=bool(regions)||!canClone(source)){why="unsupported_material";return false;}
    struct Restore {
        int skin=Engine.External.GetSkinningMode(),sample=::Render->m_MSAASample;
        ~Restore(){Engine.External.SetSkinningMode(skin);::Render->m_MSAASample=sample;}
    } restore;
    Shader shader; // default resource flags are unregistered
    // shaderslim (2026-11-13): a volume-eligible pass is cloned as TWO passes. A = the material's own program, which only keeps
    // or hides its pixel (painter_mask_<recipe>: the cell clip; a piece with tears: painter_maskt_<recipe>, + the tear's exterior
    // test). B = the SHARED interior program (painter_vmi_model / painter_vmi_{wall|piece|tear}, the shadow / alpha-to-coverage
    // variants), which discards every kept pixel and draws what lies behind the hidden ones - the wound walls, the torn faces - for
    // every material alike. The march, the tear set and the tissue look are compiled once per job, not once per recipe.
    for(unsigned n=0;n<SHADER_ELEMENTS_MAX;++n){
        const auto& originalElement=source->E[n];if(!originalElement)continue;
        ShaderElement element;element._copy(*originalElement);element.passes.clear();
        for(const auto& original:originalElement->passes){
            ProgramPair pair;if(!programPair(*original,pair)){why="unsupported_pass";return false;}
            if(ownership){
                const xr_string prefix="painter_mask_";if(pair.pixel.find(prefix)!=0){why="ownership_program_prefix";return false;}
                pair.pixel=(removal?"painter_owned_":"painter_region_")+pair.pixel.substr(prefix.size());
            }
            string_path path;
            auto exists=[&path](const xr_string& name,const char* extension){strconcat(sizeof(path),path,"r3\\",name.c_str(),extension);return !!FS.exist("$game_shaders$",path);};
            if(!exists(pair.vertex,".vs")){why="missing_mask_vs";return false;}
            if(!exists(pair.pixel,".ps")){why="missing_mask_ps";return false;}
            // Mask-only wounds: an intact body's deferred exterior pass (no MSAA sample variant), its alpha-to-coverage pre-pass
            // and its shadow passes get the interior pass: a removed pixel steps into the anatomy volume instead of leaving a hole.
            // Missing files, a failed compile or a refused binding fall back to the plain clip for the pass.
            const SPass* tissuePass=nullptr;
            if(tissue&&removal&&!ownership&&anatomyPages&&anatomyData&&pair.sample<0&&!pair.glass&&!pair.forwardModel&&
               PainterMaskPass::volumeEligible(pair.vertex,pair.pixel)){ // maskpass: deferred exterior, its atoc pre-pass, shadow passes
                const auto&te=tissue->E[n]?tissue->E[n]:tissue->E[0];
                if(te&&!te->passes.empty()&&te->passes[0])tissuePass=te->passes[0]._get();
            }
            // ONE pass of the clone: its programs by name (melee: a body with dents draws EVERY pass with the dent twin of its
            // vertex program; a missing file or a failed compile keeps the plain one, logged), the mask / volume / tear resources
            // by name, every other constant from the original material (volume: what the exterior lacks - the tissue textures,
            // the look's constants - from the anatomy material). Re-reflection is essential: the mask shifts texture / CB indices.
            // bloodfix: liveOn = this body's blood is at hand; allowAlias is off only for the plain retry of a refused alias
            const LiveBlood* const live=liveBlood();const bool liveOn=live&&bool(live->table)&&live->clock&&live->target;bool allowAlias=true;
            auto make=[&](const xr_string& vertexPlain,const xr_string& vertexDent,const xr_string& pixelName,bool volume,bool withTear,ref_pass& made,xr_string& refusal)->bool{
                xr_string vertexName=vertexPlain;bool dented=false;
                if(dents&&!vertexDent.empty()){
                    if(exists(vertexDent,".vs")){vertexName=vertexDent;dented=true;}else Msg("! [painter_mask_only] dent_program_missing vs=%s",vertexDent.c_str());}
                Engine.External.SetSkinningMode(4);::Render->m_MSAASample=pair.sample;
                SPass pass;pass.state=original->state;pass.C=original->C;
                pass.gs=original->gs;pass.hs=original->hs;pass.ds=original->ds;pass.cs=original->cs;
                // bloodfix: a plain surface pass is created as its blood alias when the pair is warm (PainterBloodPrograms; the
                // rule of PainterBloodBake::clone: no shadow pass, no pre-pass); else the plain pair, as before
                bool aliased=false;
                if(liveOn&&allowAlias&&pixelName.find("painter_blood_")!=0&&pixelName!="dumb"&&pixelName.find("shadow")==xr_string::npos&&pixelName.find("_atoc")==xr_string::npos)
                    aliased=PainterBloodPrograms::find((xr_string("painter_blood_")+vertexName).c_str(),4,(xr_string("painter_blood_")+pixelName).c_str(),pair.sample,pass.vs,pass.ps)&&pass.vs&&pass.ps&&pass.vs->vs&&pass.ps->ps;
                if(!aliased){PainterSoftShaderGuard soft;pass.vs=resources._CreateVS(vertexName.c_str());pass.ps=resources._CreatePS(pixelName.c_str());}
                if(dented&&(!pass.vs||!pass.vs->vs)){Msg("! [painter_mask_only] dent_program_refused vs=%s",vertexName.c_str());dented=false;
                    vertexName=vertexPlain;PainterSoftShaderGuard soft;pass.vs=resources._CreateVS(vertexName.c_str());}
                if(!pass.vs||!pass.ps||!pass.vs->vs||!pass.ps->ps){refusal="mask_program_creation";return false;}
                R_constant_table constants;constants.merge(&pass.ps->constants);constants.merge(&pass.vs->constants);
                STextureList textures;
                for(auto& c:constants.table){
                    const ref_texture*field=nullptr;
                    if(!xr_strcmp(*c->name,"s_painter_removal_pages"))field=&pages;
                    if(!xr_strcmp(*c->name,"s_painter_removal_data"))field=&bits;
                    if(!xr_strcmp(*c->name,"s_painter_skin_planes"))field=&planes;
                    if(!xr_strcmp(*c->name,"s_painter_skin_regions"))field=&regions;
                    if(volume&&!xr_strcmp(*c->name,"s_anatomy_pages"))field=&anatomyPages;
                    if(volume&&!xr_strcmp(*c->name,"s_anatomy_data"))field=&anatomyData;
                    if(withTear&&!xr_strcmp(*c->name,"s_tear_params"))field=&tearParams;
                    if(withTear&&!xr_strcmp(*c->name,"s_tear_map"))field=&tearMap;
                    if(withTear&&!xr_strcmp(*c->name,"s_tear_bake"))field=&tearBake;
                    if(dented&&!xr_strcmp(*c->name,"s_painter_dents"))field=&dents; // melee
                    if(!xr_strcmp(*c->name,"s_painter_slits"))field=&slits; // melee: the value-3 exterior tests (painter_slit.h)
                    // a blood-capable program before any blood is baked on the body: the layer is off (bloodOff), its three
                    // textures only have to be bound (never read); PainterBloodBake::clone binds the body's own when blood lands
                    if(!xr_strcmp(*c->name,"s_painter_blood_pages")||!xr_strcmp(*c->name,"s_painter_blood_cells")||!xr_strcmp(*c->name,"s_painter_blood_records"))field=liveOn?&live->table:&pages; // bloodfix: the body's own table when it is at hand
                    if(field){if(c->type!=RC_dx10texture||!(*field)){refusal="mask_resource_type";return false;}textures.emplace_back(c->samp.index,*field);continue;}
                    if(!xr_strcmp(*c->name,"painter_blood_clock")){if(liveOn)c->handler=live->clock;else c->handler=&bloodOff();continue;} // bloodfix
                    if(!xr_strcmp(*c->name,"painter_blood_target")){if(liveOn)c->handler=live->target;else c->handler=&bloodOff();continue;}
                    if(!xr_strcmp(*c->name,"ssfx_blood_decals")){c->handler=PainterBloodDecalControls();continue;}
                    const SPass* from=original._get();
                    auto* old=original->constants?original->constants->get(*c->name):nullptr;
                    if(!old&&volume&&tissuePass&&tissuePass->constants){from=tissuePass;old=tissuePass->constants->get(*c->name);}
                    if(!old||old->type!=c->type){refusal="constant_contract_changed";return false;}
                    c->handler=old->handler;
                    if(c->type==RC_sampler&&c->samp.index!=old->samp.index){refusal="sampler_contract_changed";return false;}
                    if(c->type==RC_dx10texture){
                        bool found=false;if(from->T)for(const auto& binding:*from->T)if(binding.first==old->samp.index){textures.emplace_back(c->samp.index,binding.second);found=true;break;}
                        if(!found){refusal="missing_original_texture";return false;}
                    }
                }
                if(removal&&(!constants.get("s_painter_removal_pages")||!constants.get("s_painter_removal_data"))){refusal="missing_mask_reflection";return false;}
                if(ownership&&(!constants.get("s_painter_skin_planes")||!constants.get("s_painter_skin_regions"))){refusal="missing_skin_owner_reflection";return false;}
                if(volume&&(!constants.get("s_anatomy_pages")||!constants.get("s_anatomy_data"))){refusal="missing_volume_reflection";return false;}
                if(withTear&&(!constants.get("s_tear_params")||!constants.get("s_tear_map")||!constants.get("s_tear_bake"))){refusal="missing_tear_reflection";return false;}
                pass.constants=resources._CreateConstantTable(constants);pass.T=resources._CreateTextureList(textures);
                made=resources._CreatePass(pass);return true;
            };
            const bool tearPiece=piece&&bool(tearParams)&&bool(tearMap)&&bool(tearBake); // slice 3: attempt -1 = the tear pair
            bool done=false;
            // attempt -1: a piece with tears (painter_maskt_ + painter_vmi_*tear); 0: the interior without tears (painter_mask_ +
            // painter_vmi_*piece / wall); 1: the plain clip, today's value-0 pass
            for(int attempt=tissuePass?(tearPiece?-1:0):1;attempt<2&&!done;++attempt){
                const bool volume=attempt<=0,withTear=attempt<0;
                const VolumeNames names=volumeNames(pair,withTear,withTear?"tear":(piece?"piece":"wall"));
                if(volume){
                    if(element.passes.size()+2>SHADER_PASSES_MAX)continue;
                    if(!exists(names.vertexA,".vs")||!exists(names.pixelA,".ps")||!exists(names.vertexB,".vs")||!exists(names.pixelB,".ps"))continue;
                }
                ref_pass a,b;xr_string refusal;
                bool madeA=make(names.vertexA,names.dentA,names.pixelA,withTear,withTear,a,refusal);
                if(!madeA&&liveOn){allowAlias=false;refusal.clear();madeA=make(names.vertexA,names.dentA,names.pixelA,withTear,withTear,a,refusal);allowAlias=true;} // bloodfix: a refused alias: the plain pass, as before
                if(!madeA){
                    if(volume){Msg("! [painter_mask_only] volume_program_refused why=%s ps=%s",refusal.c_str(),names.pixelA.c_str());continue;}
                    why=refusal;return false;}
                if(volume&&!make(names.vertexB,names.dentB,names.pixelB,true,withTear,b,refusal)){
                    Msg("! [painter_mask_only] volume_program_refused why=%s ps=%s",refusal.c_str(),names.pixelB.c_str());continue;}
                if(element.passes.size()+(volume?2u:1u)>SHADER_PASSES_MAX){why="too_many_passes";return false;}
                element.passes.push_back(a);if(volume)element.passes.push_back(b);
                done=true;
            }
            if(!done){if(why.empty())why="mask_program_creation";return false;}
        }
        shader.E[n]=resources._CreateElement(element);
    }
    ref_shader complete;complete=resources._CreateShader(&shader);destination=complete;why.clear();return true;
}
} // namespace PainterRemovalMaterial
#endif
