#pragma once
#ifdef USE_DX11
#include "../../Include/xrRender/SparseTissueField.h"
#include "RuntimeShaderCache.h"
#include <atomic>
#include <memory>
#include <vector>
// The cache owns the last reference until a main-thread eviction, including
// canceled stages. Therefore ResourceManager handles never die on a worker.
struct PainterTissueGpu {
    std::shared_ptr<const painter_detail::TissueField> field;
    HRESULT native_result=E_PENDING; // Published by the terminal native release store.
    std::atomic<unsigned> native{0}; // 0 unqueued, 1 preparing, 2 ready, 3 refused
    std::atomic<bool> warmed{false};bool retired=false;ID3D11Device* device_identity=nullptr;
    ID3D11Texture2D* page_surface=nullptr;ID3D11Texture2D* data_surface=nullptr;
    ref_texture page_texture,data_texture;
    xr_string page_name,data_name;
    xr_vector<shared_str> fallback_textures;
    xr_vector<RuntimeShaderEntry> shaders;
    u32 last_frame=0;unsigned warm_next=1;bool registered=false;
    ~PainterTissueGpu(){_RELEASE(page_surface);_RELEASE(data_surface);}
    void BuildNative(ID3D11Device* device)noexcept{
        if(native.load(std::memory_order_acquire)!=1)return; // reserved on the owner thread before enqueue
        device_identity=device;
        const auto&f=*field;
        auto make=[&](ID3D11Texture2D**out,DXGI_FORMAT format,unsigned width,unsigned height,unsigned stride,const void*data){
            D3D11_TEXTURE2D_DESC d={};d.Width=width;d.Height=height;d.MipLevels=1;d.ArraySize=1;d.Format=format;d.SampleDesc.Count=1;d.Usage=D3D11_USAGE_IMMUTABLE;d.BindFlags=D3D11_BIND_SHADER_RESOURCE;
            D3D11_SUBRESOURCE_DATA s={};s.pSysMem=data;s.SysMemPitch=stride;return device->CreateTexture2D(&d,&s,out);
        };
        HRESULT p=make(&page_surface,DXGI_FORMAT_R32_UINT,f.pageWidth,unsigned(f.page.size()/f.pageWidth),f.pageWidth*4,f.page.data());
        HRESULT d=SUCCEEDED(p)?make(&data_surface,DXGI_FORMAT_R8_UINT,f.dataWidth,unsigned(f.data.size()/f.dataWidth),f.dataWidth,f.data.data()):p;
        if(FAILED(p)||FAILED(d)){_RELEASE(page_surface);_RELEASE(data_surface);}
        native_result=FAILED(p)?p:d;
        native.store(SUCCEEDED(p)&&SUCCEEDED(d)?2:3,std::memory_order_release);
    }
};
namespace PainterTissueCache {
inline std::vector<std::shared_ptr<PainterTissueGpu>>& entries(){static auto*e=new std::vector<std::shared_ptr<PainterTissueGpu>>;return *e;}
inline std::size_t budget(){
    // At most 1/32 of dedicated VRAM, capped at256MiB. Keep a modest floor for
    // shared-memory adapters; allocation failure still takes the old material path.
    DXGI_ADAPTER_DESC desc={};
    const std::size_t memory=HW.m_pAdapter&&SUCCEEDED(HW.m_pAdapter->GetDesc(&desc))?desc.DedicatedVideoMemory:0;
    return std::min<std::size_t>(512u<<20,std::max<std::size_t>(96u<<20,memory/32)); // 2026-09-29: 256 MiB hid 2 wounds
}
inline bool installed(){return FS.exist("$game_shaders$","r3\\models_painter_anatomy.s");}
inline std::shared_ptr<PainterTissueGpu> Find(const std::shared_ptr<const painter_detail::TissueField>&field){
    if(!field)return {};for(auto&e:entries())if(e->field==field&&!e->retired){e->last_frame=Device.dwFrame;return e;}return {};
}
inline std::shared_ptr<PainterTissueGpu> Prepare(const std::shared_ptr<const painter_detail::TissueField>&field,const xr_vector<shared_str>*textures){
    if(!field||!field->available||!textures||textures->size()<16||!installed())return {};
    if(auto e=Find(field))return e;
    const std::size_t cap=budget(),need=field->bytes();auto&e=entries();std::size_t used=0;for(auto&p:e)used+=p->field->bytes();
    while(used+need>cap){auto victim=e.end();for(auto it=e.begin();it!=e.end();++it)if(it->use_count()==1&&(victim==e.end()||(*it)->last_frame<(*victim)->last_frame))victim=it;
        if(victim==e.end()){Msg("[painter_tissue] fallback=live_budget used=%llu need=%llu cap=%llu",u64(used),u64(need),u64(cap));return {};}
        used-=(*victim)->field->bytes();e.erase(victim);}
    auto p=std::make_shared<PainterTissueGpu>();p->field=field;p->fallback_textures=*textures;p->shaders.resize(textures->size());p->last_frame=Device.dwFrame;
    static u64 serial=0;char name[96];xr_sprintf(name,"$user$painter_anatomy_%llu_pages",++serial);p->page_name=name;xr_sprintf(name,"$user$painter_anatomy_%llu_data",serial);p->data_name=name;e.push_back(p);return p;
}
// every: behind the loading screen (RuntimeShaderCache::Warm) all ready fields are bound in one call, else one per frame.
inline bool Warm(unsigned budget,bool every=false){
    if(!budget)return false;static u32 frame=u32(-1);if(!every&&frame==Device.dwFrame)return false;frame=Device.dwFrame;
    bool did=false;
    auto&all=entries();all.erase(std::remove_if(all.begin(),all.end(),[](const auto&p){return p->retired&&p.use_count()==1;}),all.end());
    for(auto&e:all)if(!e->retired&&e->native.load(std::memory_order_acquire)==2&&!e->warmed.load(std::memory_order_acquire)){
        if(!e->registered){e->page_texture.create(e->page_name.c_str());e->data_texture.create(e->data_name.c_str());
            e->page_texture->surface_set(e->page_surface);e->data_texture->surface_set(e->data_surface);
            e->page_texture->flags.bLoaded=true;e->data_texture->flags.bLoaded=true;e->page_texture->PostLoad();e->data_texture->PostLoad();e->registered=true;
            Msg("[painter_tissue] ready bytes=%llu cache_cap=%llu",u64(e->field->bytes()),u64(PainterTissueCache::budget()));}
        while(e->warm_next<e->fallback_textures.size()){
            const unsigned slot=e->warm_next++;if(!e->fallback_textures[slot].size())continue;
            xr_string textures=*e->fallback_textures[slot];textures+=",";textures+=e->page_name;textures+=",";textures+=e->data_name;
            dxRender_Visual scratch;scratch.hud=false;scratch.skinning=4;int old=Engine.External.GetSkinningMode();scratch.SetShaderTexture("models\\painter_anatomy",textures.c_str());Engine.External.SetSkinningMode(old);
            auto&s=e->shaders[slot];s.dbg_shader=scratch.dbg_shader;s.dbg_texture=scratch.dbg_texture;s.shader=scratch.shader;
            // R62: the channel programs the first wound clones from this material (painter_channel was never warmed: 2-13 s
            // compile at the first wound); one request per distinct program, compiled by the warm drain.
            if(s.shader._get())RuntimeShaderCache::RequestClonePrograms(s.shader);
        }
        e->warmed.store(true,std::memory_order_release);
        did=true;if(!every)return true; // One VS/PS program; the material pairs reuse that program.
    }return did;
}
inline bool Bind(const std::shared_ptr<PainterTissueGpu>&e,dxRender_Visual&child){
    if(!e||!e->registered||child.hud||child.skinning!=4)return false;
    for(unsigned slot=1;slot<e->fallback_textures.size();++slot)if(e->fallback_textures[slot].equal(child.dbg_texture_def)&&e->shaders[slot].shader._get()){
        const auto&s=e->shaders[slot];child.dbg_shader_def="models\\painter_anatomy";child.dbg_texture_def=s.dbg_texture;child.dbg_shader=s.dbg_shader;child.dbg_texture=s.dbg_texture;child.shader=s.shader;
        child.flags.set(IRenderVisualFlags::eNoShadow,FALSE);e->last_frame=Device.dwFrame;return true;}return false;
}
inline void Clear(bool rewarm){
    auto&e=entries();if(rewarm){for(auto&p:e){p->shaders.clear();p->shaders.resize(p->fallback_textures.size());p->warm_next=1;p->registered=false;p->warmed.store(false,std::memory_order_release);}}
    else {for(auto&p:e)p->retired=true;e.erase(std::remove_if(e.begin(),e.end(),[](const auto&p){return p.use_count()==1;}),e.end());}
}
} // namespace PainterTissueCache
#endif
