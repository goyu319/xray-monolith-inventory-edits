#pragma once
#ifdef USE_DX11
#include "BloodCellLayer.h"
#include "PainterBloodUpload.h"
#include "PainterBloodPrograms.h"
#include "PainterBloodOwner.h"
#include "PainterSoftShader.h"
#include "ResourceManager.h"
#include "dxRenderDeviceRender.h"
#include "xrRender_console.h"
#include "../../xrEngine/xr_ioconsole.h"
#include <mutex>

namespace PainterBloodBake {
inline bool enabled() { int lo=0,hi=0;return Console&&Console->GetInteger("painter_voxel_mask_only",lo,hi)>0; }
struct Clock: R_constant_setup {
    void setup(R_constant* c) override {
        RCache.set_c(c,Device.fTimeGlobal,heat_vision_cooldown_time*.001f,heat_vision_cooldown?1.f:0.f,enabled()?1.f:0.f);
    }
};
inline Clock& clock(){static Clock v;return v;}
struct Target: R_constant_setup {
    void setup(R_constant* c) override {
        const bool normalized=RImplementation.Target&&RImplementation.Target->rt_Color&&RImplementation.Target->rt_Color->fmt==D3DFMT_A8R8G8B8;
        RCache.set_c(c,normalized?1.f:0.f,0.f,0.f,0.f);
    }
};
inline Target& target(){static Target v;return v;}
void retireDelta(const BloodBakeAsync::Delta* packet);
// bloodfix 2026-11-13 (version-4 cell table): a body's blood is ONE texture. A body whose layer the worker dropped under
// memory pressure (a `cleared` delta), or whose snapshot is empty, binds this six-row empty table: a zero header means "no
// blood" to painter_blood_cells.h, so the body's blood materials stay as they are and draw none, and binding it releases
// the body's own table. One per device, made on first use.
struct EmptyTable {ID3D11Device* device=nullptr;std::shared_ptr<PainterBloodUpload::Texture> table;};
inline EmptyTable& emptyTableSlot(){static auto* value=new EmptyTable;return *value;}
inline std::shared_ptr<PainterBloodUpload::Texture> emptyTable(ID3D11Device* device){
    auto& slot=emptyTableSlot();
    if(!device){slot.table.reset();slot.device=nullptr;return {};} // shutdown: released before the device goes
    if(slot.device!=device||!slot.table){
        try{
            const unsigned rows=BloodCellLayer::FixedRows+1u;const std::vector<BloodCellLayer::U4> zero(std::size_t(rows)*BloodCellLayer::Width);
            slot.table=PainterBloodUpload::makeTexture(device,BloodCellLayer::Width,rows,zero.data());slot.device=device;
        }catch(...){return {};}
    }
    return slot.table;
}
struct Gpu {
    std::shared_ptr<PainterBloodUpload::Preparer> preparer;
    PainterBloodUpload::Upload upload;
    ref_texture cellTexture;
    ID3D11Device* device=nullptr;
    std::shared_ptr<const BloodBakeAsync::Delta> heldPacket;
    explicit Gpu(ID3D11Device* d):preparer(std::make_shared<PainterBloodUpload::Preparer>(d)),device(d){}
    bool bind(const PainterBloodUpload::Texture& table){
        // Resource and SRV are already complete. This path never creates a
        // native resource, waits for loading, or decodes on the owner context.
        if(!cellTexture){string128 n;xr_sprintf(n,"$user$painter_blood_cells_%p",this);cellTexture=xr_new<CTexture>();cellTexture->set_name(n);}
        if(!cellTexture||cellTexture->flags.bLoading)return false;
        return cellTexture->surface_set_prepared(table.texture.p,table.view.p);
    }
    // bloodfix (2026-11-13, second session): the engine texture the body's materials bind, there from the body's first
    // wound - it shows the empty table until blood is published. The removal clone binds it into every new material
    // (PainterRemovalMaterial::liveBlood), so a new publication of the body draws its blood from its first frame.
    bool texture(){
        if(cellTexture)return true;
        const auto empty=emptyTable(device);
        return empty&&bind(*empty);
    }
    bool begin(std::unique_ptr<BloodBakeAsync::Delta>& packet){
        if(!packet||!upload.idle())return false;
        heldPacket=std::shared_ptr<const BloodBakeAsync::Delta>(packet.release(),retireDelta);
        if(!upload.begin(heldPacket))return false;heldPacket.reset();return true;
    }
    void pump(PainterBloodUpload::FrameBudget& budget){
        upload.pump(HW.pContext,budget,[this](const PainterBloodUpload::Texture& table,std::uint64_t){return bind(table);});
    }
    // A `cleared` delta: the body draws no blood until its next delta and its table is released. False: try next frame.
    bool clear(){
        if(!upload.idle())return false;
        if(cellTexture){const auto empty=emptyTable(device);if(!empty||!bind(*empty))return false;}
        return upload.clear();
    }
};
// The clone of one body material, one shader element per call (bloodfix): each call is one measured operation of the frame
// budget. A whole material at once cost a few hundred microseconds, and two or three of them in one frame overran it.
// `source` pins the material the steps belong to: the progress is kept per material address, and without the reference a
// retired material's address could come back as another material and take over its half-built elements.
struct CloneProgress {ref_shader source;ref_selement elements[SHADER_ELEMENTS_MAX];unsigned next=0,changed=0,seen=~0u;};
// True when every surface pass of the material already draws this body's blood: the removal clone created it so
// (PainterRemovalMaterial::liveBlood). Shadow passes and pre-passes carry no blood, as in clone().
inline bool bound(const ref_shader& source,const Gpu& gpu){
    if(!source||!gpu.cellTexture)return false;
    unsigned surface=0;
    for(unsigned n=0;n<SHADER_ELEMENTS_MAX;++n){const auto& e=source->E[n];if(!e)continue;
        for(const auto& p:e->passes){
            if(!p||!p->vs||!p->ps)return false;
            const xr_string ps=*p->ps->cName;
            if(ps=="dumb"||ps.find("shadow")!=xr_string::npos||ps.find("_atoc")!=xr_string::npos)continue;
            if(ps.find("painter_blood_")!=0||!p->T)return false;
            bool table=false;for(const auto& b:*p->T)if(b.second._get()==gpu.cellTexture._get()){table=true;break;}
            if(!table)return false;
            ++surface;
        }}
    return surface!=0;
}
inline bool clone(const ref_shader& source,Gpu& gpu,ref_shader& result,xr_string& why,CloneProgress* progress=nullptr){
    if(!source||!gpu.cellTexture){why="blood_material_missing";return false;}
    xrCriticalSectionTryGuard creation(DEV->PainterBloodCreationGuard());if(!creation.owns_lock()){why="blood_material_busy";return false;}
    CloneProgress whole;CloneProgress& state=progress?*progress:whole;
    if(state.source._get()!=source._get()){const unsigned seen=state.seen;state=CloneProgress();state.source=source;state.seen=seen;} // steps of another material never continue here
    unsigned& changed=state.changed;
    for(unsigned n=state.next;n<SHADER_ELEMENTS_MAX;++n){const auto& oe=source->E[n];if(!oe){state.next=n+1;continue;}
        ShaderElement e;e._copy(*oe);e.passes.clear();
        for(const auto& op:oe->passes){
            if(!op||!op->vs||!op->ps){why="blood_material_pass";return false;}
            xr_string vs=*op->vs->cName,ps=*op->ps->cName;int sample=-1;
            if(ps.size()>2&&ps[ps.size()-2]=='_'&&ps.back()>='0'&&ps.back()<='7'){sample=ps.back()-'0';ps.resize(ps.size()-2);}
            // Shadows and pre-passes preserve their complete original pass.
            if(ps=="dumb"||ps.find("shadow")!=xr_string::npos||ps.find("_atoc")!=xr_string::npos){e.passes.push_back(op);continue;}
            if(op->vs->skinning>=0&&vs.size()>2&&vs[vs.size()-2]=='_'&&vs.back()>='0'&&vs.back()<='4')vs.resize(vs.size()-2);
            const xr_string vertex="painter_blood_"+vs,pixel="painter_blood_"+ps;
            if((op->gs&&op->gs->gs)||(op->hs&&op->hs->sh)||(op->ds&&op->ds->sh)||(op->cs&&op->cs->sh)){why="blood_unsupported_shader_stage";return false;}
            SPass p;p.state=op->state;p.C=op->C;p.gs=op->gs;p.hs=op->hs;p.ds=op->ds;p.cs=op->cs;
            // shaderslim: a pass the removal clone created blood-capable (the tear exterior, the shared interior: painter_blood_*) keeps
            // its two programs - only the body's blood is bound below; any other pass takes the blood alias of its programs
            if(ps.find("painter_blood_")==0){p.vs=op->vs;p.ps=op->ps;}
            else if(!PainterBloodPrograms::find(vertex.c_str(),op->vs->skinning,pixel.c_str(),sample,p.vs,p.ps)){
                // bloodfix: a recipe without alias files never draws baked blood: said once, not asked for every frame
                if(!PainterBloodPrograms::file(vertex.c_str(),".vs")||!PainterBloodPrograms::file(pixel.c_str(),".ps")){why="blood_alias_missing";return false;}
                PainterBloodPrograms::requestPair(vs.c_str(),op->vs->skinning,ps.c_str(),sample); // shaderslim: asked for now, created by the pump; this clone is retried
                why="blood_programs_not_warmed";return false;}
            if(!p.vs||!p.ps||!p.vs->vs||!p.ps->ps){why="blood_shader_compile";return false;}
            R_constant_table constants;constants.merge(&p.ps->constants);constants.merge(&p.vs->constants);STextureList textures;
            for(auto& c:constants.table){
                const ref_texture* field=nullptr;
                // Version 4 reads one table. An include of an earlier version left in the shader folder names two more
                // textures: they take the table too, and that include's own version check then draws no blood.
                if(!xr_strcmp(*c->name,"s_painter_blood_cells")||!xr_strcmp(*c->name,"s_painter_blood_pages")||!xr_strcmp(*c->name,"s_painter_blood_records"))field=&gpu.cellTexture;
                if(field){if(c->type!=RC_dx10texture){why="blood_texture_type";return false;}textures.emplace_back(c->samp.index,*field);continue;}
                if(!xr_strcmp(*c->name,"painter_blood_clock")){c->handler=&clock();continue;}
                if(!xr_strcmp(*c->name,"painter_blood_target")){c->handler=&target();continue;}
                if(!xr_strcmp(*c->name,"ssfx_blood_decals")){c->handler=PainterBloodDecalControls();continue;}
                auto* old=op->constants?op->constants->get(*c->name):nullptr;
                if(!old||old->type!=c->type){why="blood_constant_contract";return false;}c->handler=old->handler;
                if(c->type==RC_sampler&&c->samp.index!=old->samp.index){why="blood_sampler_contract";return false;}
                if(c->type==RC_dx10texture){bool found=false;if(op->T)for(const auto& b:*op->T)if(b.first==old->samp.index){textures.emplace_back(c->samp.index,b.second);found=true;break;}
                    if(!found){why="blood_source_texture";return false;}}
            }
            if(!constants.get("s_painter_blood_cells")||!constants.get("ssfx_blood_decals")){why="blood_resource_reflection";return false;}
            p.constants=DEV->_CreateConstantTable(constants);p.T=DEV->_CreateTextureList(textures);e.passes.push_back(DEV->_CreatePass(p));++changed;
        }
        state.elements[n]=DEV->_CreateElement(e);state.next=n+1;
        if(progress){why="blood_material_step";return false;}
    }
    if(!changed){why="blood_no_surface_pass";return false;}
    Shader shader;for(unsigned n=0;n<SHADER_ELEMENTS_MAX;++n)shader.E[n]=state.elements[n];
    result=DEV->_CreateShader(&shader);return !!result;
}
}
#endif
