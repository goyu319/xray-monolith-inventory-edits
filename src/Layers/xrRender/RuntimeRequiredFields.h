#pragma once
#ifdef USE_DX11
#include "PainterTissueField.h"
#include "PainterChannelGpu.h"
#include "PainterRemovalGpu.h"
#include "PainterSkinOwnershipGpu.h"
#include "PainterWallmarkOwnership.h"
#include "PainterWallmarkTear.h" // maskdecal
#include "PainterDentGpu.h" // melee (value 3): a body's dents
#include "PainterDent.h"
#include "../../Include/xrRender/RuntimePackedMesh.h"
#include "dxRenderDeviceRender.h"
// Concrete adapter for the packet contract. Completion is distinct from
// admission: a refused required resource finishes waiting but never publishes.
namespace RuntimeRequiredFieldCache {
inline std::uint64_t&epoch(){static std::uint64_t value=1;return value;}
inline void Clear(bool rewarm=false){
    if(rewarm&&HW.pDevice){PainterChannelCache::Clear(HW.pDevice);PainterRemovalCache::Clear(HW.pDevice);PainterSkinOwnershipCache::Clear(HW.pDevice);PainterWallmarkOwnershipCache::Clear(HW.pDevice);return;}
    ++epoch();if(!epoch())++epoch();PainterChannelCache::Clear();PainterRemovalCache::Clear();PainterSkinOwnershipCache::Clear();PainterWallmarkOwnershipCache::Clear();}
inline void Collect(){PainterWallmarkOwnershipCache::Collect();PainterSkinOwnershipCache::Collect();PainterChannelCache::Collect();PainterRemovalCache::Collect();}
}
// Mask-only wounds, slice 3: one piece's baked decide-once tear on the GPU (painter_vmask_tear.h: s_tear_params, s_tear_map, s_tear_bake)
struct PainterTearGpu { // slice 4: a piece's drawing set (all of its torn ends, TearAtlas.h)
    ID3D11Texture2D *params=nullptr,*map=nullptr;ID3D11Texture3D *bake=nullptr;ref_texture params_texture,map_texture,bake_texture;
    ~PainterTearGpu(){_RELEASE(params);_RELEASE(map);_RELEASE(bake);}
    bool Build(ID3D11Device*device,const gfy_packed::TearAtlas&a,const void*key){
        // cutfix: a set by reference (TearAtlas.h: blocks + places, no arrays): its textures are made empty and each block uploaded
        const bool byRef=!a.blocks.empty();
        if(!device||!a.K||!a.paramsRows||a.paramsRows>16384||a.params.size()!=std::size_t(a.paramsRows)*104||!a.mapW||!a.mapH||a.mapW>16384||a.mapH>16384||
           a.fieldDim[2]>2048||!a.fieldDim[0]||!a.fieldDim[1]||!a.fieldDim[2])return false;
        if(byRef){if(a.blocks.size()!=a.K||a.place.size()!=a.K||!HW.pContext)return false;
            for(std::size_t k=0;k<a.K;++k){const auto*b=a.blocks[k].get();const auto&q=a.place[k];
                if(!b||b->map.size()!=std::size_t(b->map_dim[0])*b->map_dim[1]*20||b->bake.size()!=std::size_t(b->bake_dim[0])*b->bake_dim[1]*b->bake_dim[2]||
                   std::uint64_t(q[0])+5ull*b->map_dim[0]>a.mapW||std::uint64_t(q[1])+b->map_dim[1]>a.mapH||std::uint64_t(q[2])+b->bake_dim[0]>a.fieldDim[0]||
                   std::uint64_t(q[3])+b->bake_dim[1]>a.fieldDim[1]||std::uint64_t(q[4])+b->bake_dim[2]>a.fieldDim[2])return false;}}
        else if(a.map.size()!=std::size_t(a.mapW)*a.mapH*4||a.field.size()!=std::size_t(a.fieldDim[0])*a.fieldDim[1]*a.fieldDim[2])return false;
        if(byRef){D3D11_TEXTURE2D_DESC d={};d.Width=26;d.Height=a.paramsRows;d.MipLevels=1;d.ArraySize=1;d.Format=DXGI_FORMAT_R32G32B32A32_UINT;d.SampleDesc.Count=1;
            d.Usage=D3D11_USAGE_IMMUTABLE;d.BindFlags=D3D11_BIND_SHADER_RESOURCE;D3D11_SUBRESOURCE_DATA s={};s.pSysMem=a.params.data();s.SysMemPitch=26*16;
            if(FAILED(device->CreateTexture2D(&d,&s,&params)))return false;
            d.Width=a.mapW;d.Height=a.mapH;d.Format=DXGI_FORMAT_R32G32B32A32_FLOAT;d.Usage=D3D11_USAGE_DEFAULT;
            if(FAILED(device->CreateTexture2D(&d,nullptr,&map))){_RELEASE(params);return false;}
            D3D11_TEXTURE3D_DESC v={};v.Width=a.fieldDim[0];v.Height=a.fieldDim[1];v.Depth=a.fieldDim[2];v.MipLevels=1;v.Format=DXGI_FORMAT_R8_UINT;
            v.Usage=D3D11_USAGE_DEFAULT;v.BindFlags=D3D11_BIND_SHADER_RESOURCE;
            if(FAILED(device->CreateTexture3D(&v,nullptr,&bake))){_RELEASE(params);_RELEASE(map);return false;}
            for(std::size_t k=0;k<a.K;++k){const auto&b=*a.blocks[k];const auto&q=a.place[k];
                const D3D11_BOX mb={q[0],q[1],0u,q[0]+5u*b.map_dim[0],q[1]+b.map_dim[1],1u};
                HW.pContext->UpdateSubresource(map,0,&mb,b.map.data(),5u*b.map_dim[0]*16u,0);
                const D3D11_BOX fb={q[2],q[3],q[4],q[2]+b.bake_dim[0],q[3]+b.bake_dim[1],q[4]+b.bake_dim[2]};
                HW.pContext->UpdateSubresource(bake,0,&fb,b.bake.data(),b.bake_dim[0],b.bake_dim[0]*b.bake_dim[1]);}
            auto bind=[&](ref_texture&texture,const char*kind,ID3D11Resource*surface){string128 name;xr_sprintf(name,"$user$painter_tear_%s_%p",kind,key);
                texture.create(name);if(!texture)return false;texture->surface_set(surface);texture->flags.bLoaded=true;texture->PostLoad();return true;};
            if(!bind(params_texture,"params",params)||!bind(map_texture,"map",map)||!bind(bake_texture,"bake",bake))return false;
            Msg("[painter_mask_only] tear_piece_bound tears=%u map=%ux%u field=%ux%ux%u kb=%u by_reference=1",a.K,a.mapW,a.mapH,a.fieldDim[0],a.fieldDim[1],a.fieldDim[2],unsigned(a.bytes/1024));
            return true;}
        D3D11_TEXTURE2D_DESC d={};d.Width=26;d.Height=a.paramsRows;d.MipLevels=1;d.ArraySize=1;d.Format=DXGI_FORMAT_R32G32B32A32_UINT;d.SampleDesc.Count=1;
        d.Usage=D3D11_USAGE_IMMUTABLE;d.BindFlags=D3D11_BIND_SHADER_RESOURCE;D3D11_SUBRESOURCE_DATA s={};s.pSysMem=a.params.data();s.SysMemPitch=26*16;
        if(FAILED(device->CreateTexture2D(&d,&s,&params)))return false;
        d.Width=a.mapW;d.Height=a.mapH;d.Format=DXGI_FORMAT_R32G32B32A32_FLOAT;s.pSysMem=a.map.data();s.SysMemPitch=a.mapW*16;
        if(FAILED(device->CreateTexture2D(&d,&s,&map))){_RELEASE(params);return false;}
        D3D11_TEXTURE3D_DESC v={};v.Width=a.fieldDim[0];v.Height=a.fieldDim[1];v.Depth=a.fieldDim[2];v.MipLevels=1;v.Format=DXGI_FORMAT_R8_UINT;
        v.Usage=D3D11_USAGE_IMMUTABLE;v.BindFlags=D3D11_BIND_SHADER_RESOURCE;D3D11_SUBRESOURCE_DATA t={};t.pSysMem=a.field.data();
        t.SysMemPitch=a.fieldDim[0];t.SysMemSlicePitch=a.fieldDim[0]*a.fieldDim[1];
        if(FAILED(device->CreateTexture3D(&v,&t,&bake))){_RELEASE(params);_RELEASE(map);return false;}
        auto bind=[&](ref_texture&texture,const char*kind,ID3D11Resource*surface){string128 name;xr_sprintf(name,"$user$painter_tear_%s_%p",kind,key);
            texture.create(name);if(!texture)return false;texture->surface_set(surface);texture->flags.bLoaded=true;texture->PostLoad();return true;};
        if(!bind(params_texture,"params",params)||!bind(map_texture,"map",map)||!bind(bake_texture,"bake",bake))return false;
        Msg("[painter_mask_only] tear_piece_bound tears=%u map=%ux%u field=%ux%ux%u kb=%u",a.K,a.mapW,a.mapH,a.fieldDim[0],a.fieldDim[1],a.fieldDim[2],unsigned(a.bytes/1024));return true;}
};
struct RuntimeRequiredFields {
    std::shared_ptr<PainterTissueGpu> anatomy;
    std::shared_ptr<PainterChannelGpu> channels;
    std::shared_ptr<PainterRemovalGpu> removal;
    std::shared_ptr<PainterSkinOwnershipGpu> skin_ownership;
    std::shared_ptr<PainterWallmarkOwnershipGpu> wallmark_ownership;
    std::shared_ptr<const painter_detail::AnalyticChannelField> channel_source;
    std::shared_ptr<const painter_detail::DamageField> removal_source;
    std::shared_ptr<const painter_detail::SkinOwnershipField> skin_source;
    std::uint64_t epoch{},cookie{},generation{},revision{};
    // Material clones made for this publish (main thread): a pure function of the source shader (and slot) and this
    // publish's field textures, so chunks that share a source share the clone.
    std::vector<std::tuple<const Shader*,unsigned,ref_shader>> removal_clones;
    std::vector<std::tuple<const Shader*,unsigned,ref_shader>> channel_clones;
    bool need_anatomy=false,need_channels=false,need_removal=false,need_skin_ownership=false,need_wallmark_ownership=false;
    bool mask_volume=false; // mask-only wounds: skin_clip exteriors draw the anatomy volume (painter_vmask_* programs)
    bool mask_piece=false; // slice 2: a detached piece: painter_vmaskp_* pixel programs (the coverage box)
    std::shared_ptr<const gfy_packed::TearAtlas> tear_atlas;std::shared_ptr<PainterTearGpu> tear_gpu; // slices 3 + 4: all of a piece's torn ends
    std::shared_ptr<const std::vector<gfy_packed::Dent>> dents;std::shared_ptr<PainterDentGpu> dent_gpu; // melee (value 3): the dents and their texture
    std::uint64_t dent_key=0; // melee: painter_dent::key of `dents` (its blood decals follow it)
    std::shared_ptr<const std::vector<gfy_packed::Slit>> slits;std::shared_ptr<PainterDentGpu> slit_gpu; // melee: the blade slits and their texture
    std::shared_ptr<const gfy_packed::Mesh> shot_mesh; // shotdump (diagnostic, -painter_shot_dump only): this publication's packed mesh, for painter_shot_dump
    bool tear_exterior=false; // maskdecal: an exterior of this publication draws with painter_vmaskt_ programs (its decals hide by the tear too)
    bool allocation_refused=false,registration_refused=false;
    const char* registration_component="none";
    bool DecisionReady()const noexcept{
        if(allocation_refused||registration_refused||epoch!=RuntimeRequiredFieldCache::epoch())return true;
        auto refused=[](const auto&p){return !p||p->native.load(std::memory_order_acquire)==3;};
        if((need_channels&&refused(channels))||(need_removal&&refused(removal))||(need_anatomy&&refused(anatomy))||(need_skin_ownership&&refused(skin_ownership))||(need_wallmark_ownership&&refused(wallmark_ownership)))return true;
        auto terminal=[](const auto&p){return p&&p->native.load(std::memory_order_acquire)>=2;};
        if(need_channels&&!terminal(channels))return false;
        if(need_removal&&!terminal(removal))return false;
        if(need_skin_ownership&&!terminal(skin_ownership))return false;
        if(need_wallmark_ownership&&!terminal(wallmark_ownership))return false;
        if(need_anatomy){if(!anatomy)return true;const auto state=anatomy->native.load(std::memory_order_acquire);
            if(state==3)return true;if(state!=2||!anatomy->warmed.load(std::memory_order_acquire))return false;}
        return true;
    }
    bool AdmitNative(ID3D11Device*device)const noexcept{
        if(allocation_refused||registration_refused||epoch!=RuntimeRequiredFieldCache::epoch())return false;
        // The acquire precedes reads of device_identity written by BuildNative.
        if(need_anatomy&&(!anatomy||anatomy->native.load(std::memory_order_acquire)!=2||anatomy->retired||anatomy->device_identity!=device||
            !anatomy->warmed.load(std::memory_order_acquire)))return false;
        if(need_channels&&(!channels||channels->native.load(std::memory_order_acquire)!=2||channels->retired||channels->field!=channel_source||channels->device_identity!=device||
            channels->device_epoch!=epoch))return false;
        if(need_removal&&(!removal||removal->native.load(std::memory_order_acquire)!=2||removal->retired||removal->field!=removal_source||removal->device_identity!=device||
            removal->device_epoch!=epoch))return false;
        if(need_skin_ownership&&(!skin_ownership||skin_ownership->native.load(std::memory_order_acquire)!=2||skin_ownership->retired||skin_ownership->field!=skin_source||skin_ownership->device_identity!=device||
            skin_ownership->device_epoch!=epoch))return false;
        if(need_wallmark_ownership&&(!wallmark_ownership||wallmark_ownership->native.load(std::memory_order_acquire)!=2||wallmark_ownership->retired||!wallmark_ownership->field||wallmark_ownership->field->source!=skin_source||wallmark_ownership->device_identity!=device||
            wallmark_ownership->device_epoch!=epoch))return false;
        return true;
    }
    bool Register(ID3D11Device*device){
        if(!AdmitNative(device))return false;
        auto bind=[&](bool need,const auto&p,const char*name){if(!need||p->Register(device,epoch))return true;registration_component=name;registration_refused=true;return false;};
        if(!bind(need_channels,channels,"channels")||!bind(need_removal,removal,"removal")||!bind(need_skin_ownership,skin_ownership,"skin_ownership")||!bind(need_wallmark_ownership,wallmark_ownership,"wallmark_ownership"))return false;
        return true;
    }
    xr_string RefusalDetail(ID3D11Device*device)const{
        const char* component="packet",*phase="unknown";unsigned state=0;HRESULT result=E_PENDING;std::uint64_t field_epoch=epoch;
        bool found=false;
        if(epoch!=RuntimeRequiredFieldCache::epoch()){phase="epoch";found=true;}
        auto inspect=[&](bool need,const auto&p,const char*name,std::uint64_t resource_epoch,const auto&source_matches,bool warmed){
            if(found||!need)return;
            const char* why=nullptr;const unsigned n=p?p->native.load(std::memory_order_acquire):0;
            if(!p)why="allocation_or_prepare";else if(n!=2)why=n==3?"native":"not_ready";
            else if(p->retired)why="retired";else if(p->device_identity!=device)why="device";
            else if(resource_epoch!=epoch)why="field_epoch";else if(!source_matches())why="source_identity";else if(!warmed)why="warmup";
            if(why){found=true;component=name;phase=why;state=n;field_epoch=resource_epoch;if(p&&n>=2)result=p->native_result;}
        };
        inspect(need_anatomy,anatomy,"anatomy",epoch,[]{return true;},anatomy&&anatomy->warmed.load(std::memory_order_acquire));
        inspect(need_channels,channels,"channels",channels?channels->device_epoch:0,[&]{return channels->field==channel_source;},true);
        inspect(need_removal,removal,"removal",removal?removal->device_epoch:0,[&]{return removal->field==removal_source;},true);
        inspect(need_skin_ownership,skin_ownership,"skin_ownership",skin_ownership?skin_ownership->device_epoch:0,[&]{return skin_ownership->field==skin_source;},true);
        inspect(need_wallmark_ownership,wallmark_ownership,"wallmark_ownership",wallmark_ownership?wallmark_ownership->device_epoch:0,[&]{return wallmark_ownership->field&&wallmark_ownership->field->source==skin_source;},true);
        std::size_t need=0,at_failure=0;
        if(found&&!std::strcmp(component,"wallmark_ownership")&&state==3){phase=wallmark_ownership->failure_phase;need=wallmark_ownership->requested_bytes;at_failure=wallmark_ownership->budget_at_failure;}
        if(!found&&registration_refused){component=registration_component;phase="registration";}
        else if(!found&&allocation_refused)phase="allocation";
        unsigned cached_channels=0,unpinned_channels=0,cached_wallmarks=0,pinned_wallmarks=0;std::size_t unpinned_bytes=0;
        for(const auto&p:PainterChannelCache::entries()){++cached_channels;if(p.use_count()==1){++unpinned_channels;unpinned_bytes+=p->budget.bytes;}}
        for(const auto&p:PainterWallmarkOwnershipCache::entries()){++cached_wallmarks;if(p.use_count()>1)++pinned_wallmarks;}
        char detail[768];xr_sprintf(detail,sizeof(detail),"required_field_refused component=%s phase=%s native=%u hr=0x%08lx field_epoch=%llu admission_epoch=%llu current_epoch=%llu shared_used=%llu requested=%llu used_at_failure=%llu channel_cache=%u channel_unpinned=%u channel_unpinned_bytes=%llu wallmark_cache=%u wallmark_pinned=%u",component,phase,state,static_cast<unsigned long>(result),field_epoch,epoch,RuntimeRequiredFieldCache::epoch(),static_cast<unsigned long long>(PainterFieldBudget::used().load(std::memory_order_acquire)),static_cast<unsigned long long>(need),static_cast<unsigned long long>(at_failure),cached_channels,unpinned_channels,static_cast<unsigned long long>(unpinned_bytes),cached_wallmarks,pinned_wallmarks);
        return detail;
    }
    bool AdmittedAtCommit(ID3D11Device*device,std::uint64_t owner,std::uint64_t expected,std::uint64_t damage)const noexcept{
        return cookie==owner&&generation==expected&&revision==damage&&AdmitNative(device)&&
            (!need_channels||channels->registered)&&(!need_removal||removal->registered)&&(!need_skin_ownership||skin_ownership->registered)&&(!need_wallmark_ownership||wallmark_ownership->registered);
    }
};
inline bool RuntimeHasRequiredFields(const gfy_packed::Mesh&mesh){
    if(mesh.analytic_channels||mesh.skin_ownership)return true;for(const auto&c:mesh.children)if(c.skin_clip||c.skin_owned)return true;return false;
}
inline const char*RuntimeValidateRequiredFields(const gfy_packed::Mesh&mesh,std::uint64_t cookie,std::uint64_t generation,std::uint64_t revision){
    try{gfy_packed::validate_skin_ownership(mesh);}catch(...){return "skin_owner_required_manifest";}
    bool skin=false;for(const auto&c:mesh.children)if(c.skin_clip){skin=true;if(c.material||c.indices.empty())return "masked_child_shape";}
    if(skin&&(!mesh.removal_field||!mesh.removal_field->available))return "required_removal_field";
    // Masked exterior wallmarks are clipped against this packet's removal field by
    // WallmarksEngine (CKinematics::LL_RuntimeRemovalGpu + PainterWallmarkMask).
    if(mesh.skin_ownership){
        const auto&field=*mesh.skin_ownership;const auto&id=field.identity;
        if(!id.snapshot||id.cookie!=cookie||id.generation!=generation||id.revision!=revision)return "skin_owner_candidate_identity";
        if(!PainterSkinOwnershipCache::valid(field))return "skin_owner_invalid_field";
        for(const auto&part:field.children){
            const gfy_packed::Child*found=nullptr;for(const auto&c:mesh.children)if(c.child_index==part.child){if(found)return "skin_owner_duplicate_child";found=&c;}
            if(!found||found->material||found->indices.empty()||found->indices.size()%3||part.count!=found->indices.size()/3)return "skin_owner_primitive_manifest";
        }
    }
    if(!mesh.analytic_channels)return nullptr;
    if(!mesh.analytic_coverage_admitted)return "analytic_coverage_unqualified";
    const auto&field=*mesh.analytic_channels;const auto&id=field.identity;
    if(!id.snapshot||id.cookie!=cookie||id.generation!=generation||id.revision!=revision)return "required_field_candidate_identity";
    if(field.limitations&~painter_detail::AnalyticChannelField::renderable_limitations)return "unsupported_analytic_history";
    if(!mesh.tissue_field||!mesh.tissue_field->available||!mesh.removal_field||!mesh.removal_field->available)return "required_analytic_fields";
    // Whole candidate manifest: every cavity triangle has exactly one metadata
    // row in its child's local primitive-ID table. No partially bound cavity.
    std::size_t found=0;
    for(const auto&c:mesh.children)if(c.material&&!c.indices.empty()){
        const auto*part=field.child(c.child_index);if(!part||part->count!=c.indices.size()/3)return "channel_primitive_manifest";++found;
    }
    if(found!=field.children.size())return "channel_unbound_child";
    return nullptr;
}
// Called after the normal original-material resolution. Mandatory anatomy
// must have bound successfully first; no legacy shader fallback is admitted.
inline bool RuntimeBindRequiredFields(const std::shared_ptr<RuntimeRequiredFields>&fields,dxRender_Visual&child,
    unsigned slot,bool skin_clip,bool skin_owned,bool analytic,bool receives_wallmarks,bool anatomy_bound,xr_string& why){
    if(!fields){why="required_fields_missing";return !skin_clip&&!skin_owned&&!analytic;}
    if(!fields->Register(HW.pDevice)){why="required_field_registration";return false;}
    (void)receives_wallmarks; // masked marks clip in WallmarksEngine, never unmasked
    xr_string reason;ref_shader bound;
    if(analytic){
        if(!anatomy_bound||!fields->channels||!fields->anatomy){why=anatomy_bound?"channel_fields_missing":"anatomy_material_not_bound";return false;}
        PainterChannelMaterial::Bindings bindings;
        if(!fields->channels->GetBindings(fields->anatomy->page_texture,fields->anatomy->data_texture,slot,bindings)){why="channel_bindings";return false;}
        if(fields->removal){bindings.removal_pages=fields->removal->page_texture;bindings.removal_data=fields->removal->data_texture;}
        const Shader* channelSource=child.shader._get();ref_shader memo;
        for(const auto&m:fields->channel_clones)if(std::get<0>(m)==channelSource&&std::get<1>(m)==slot){memo=std::get<2>(m);break;}
        if(memo._get())child.shader=memo;
        else{if(!PainterChannelMaterial::clone(*DEV,child.shader,bindings,bound,reason)){why=reason;return false;}
            fields->channel_clones.emplace_back(channelSource,slot,bound);child.shader=bound;}
    }
    if(skin_clip||skin_owned){
        if(skin_clip&&!fields->removal){why="removal_field_missing";return false;}
        ref_texture pages,bits,planes,regions;
        if(skin_clip){pages=fields->removal->page_texture;bits=fields->removal->data_texture;}
        if(skin_owned&&(!fields->skin_ownership||!fields->skin_ownership->GetBindings(slot,planes,regions))){why="skin_owner_bindings";return false;}
        const Shader* removalSource=child.shader._get();ref_shader memo;const unsigned key=skin_owned?slot:~0u;
        for(const auto&m:fields->removal_clones)if(std::get<0>(m)==removalSource&&std::get<1>(m)==key){memo=std::get<2>(m);break;}
        if(memo._get())child.shader=memo;
        else{
            // Mask-only wounds: the tissue look's textures and constants the exterior material lacks come from this model's
            // anatomy material (models\\painter_anatomy, any warmed slot: they differ only in s_base, which the look never reads).
            const Shader* tissue=nullptr;ref_texture anatomyPages,anatomyData;
            if(fields->mask_volume&&skin_clip&&!skin_owned&&fields->anatomy){
                for(const auto&s:fields->anatomy->shaders)if(s.shader._get()){tissue=s.shader._get();break;}
                anatomyPages=fields->anatomy->page_texture;anatomyData=fields->anatomy->data_texture;}
            ref_texture tearParams,tearMap,tearBake; // slice 3: a piece's baked decide-once tear (built once per publish, main thread)
            if(tissue&&fields->mask_piece&&fields->tear_atlas){
                if(!fields->tear_gpu){auto t=std::make_shared<PainterTearGpu>();if(t->Build(HW.pDevice,*fields->tear_atlas,fields.get()))fields->tear_gpu=t;
                    else{Msg("! [painter_mask_only] tear_textures_refused");fields->tear_atlas.reset();}}
                if(fields->tear_gpu){tearParams=fields->tear_gpu->params_texture;tearMap=fields->tear_gpu->map_texture;tearBake=fields->tear_gpu->bake_texture;}}
            ref_texture dentTexture; // melee (value 3): the body's dents for every pass's dent vertex program (painter_vmaskd_* / painter_maskd_*)
            if(skin_clip&&fields->dents&&!fields->dents->empty()){
                if(!fields->dent_gpu){auto t=std::make_shared<PainterDentGpu>();if(t->Build(HW.pDevice,*fields->dents,fields.get()))fields->dent_gpu=t;
                    else{Msg("! [painter_mask_only] dent_texture_refused dents=%u",unsigned(fields->dents->size()));fields->dents.reset();}}
                if(fields->dent_gpu)dentTexture=fields->dent_gpu->texture;}
            ref_texture slitTexture; // melee (value 3): the blade slits (painter_slit.h); none: the shared no-slit texel
            if(fields->slits&&!fields->slits->empty()){
                if(!fields->slit_gpu){auto t=std::make_shared<PainterDentGpu>();if(t->BuildSlits(HW.pDevice,*fields->slits,fields.get()))fields->slit_gpu=t;
                    else{Msg("! [painter_mask_only] slit_texture_refused slits=%u",unsigned(fields->slits->size()));fields->slits.reset();}}
                if(fields->slit_gpu)slitTexture=fields->slit_gpu->texture;}
            if(!slitTexture)slitTexture=PainterDentGpu::NoSlits(HW.pDevice);
            if(!PainterRemovalMaterial::clone(*DEV,child.shader,pages,bits,planes,regions,skin_clip,bound,reason,tissue,anatomyPages,anatomyData,fields->mask_piece,tearParams,tearMap,tearBake,dentTexture,slitTexture)){why=reason;return false;}
            if(tearParams&&PainterWallmarkTearMaterial::DrawsTear(bound))fields->tear_exterior=true; // maskdecal: the exterior hides by the tear
            fields->removal_clones.emplace_back(removalSource,key,bound);child.shader=bound;}
    }
    return true;
}
#endif
