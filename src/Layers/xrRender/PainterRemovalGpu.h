#pragma once
#ifdef USE_DX11
#include "../../Include/xrRender/SparseDamageField.h"
#include "PainterRemovalMaterial.h"
#include <atomic>
#include <memory>
#include <vector>
#include <algorithm>
// Exact production-resolution removal payload, immutable D3D textures.
// Cache/registration operations are owner-thread only; BuildNative is device-only.
struct PainterRemovalGpu {
    std::shared_ptr<const painter_detail::DamageField> field;
    HRESULT native_result=E_PENDING; // Published by the terminal native release store.
    std::atomic<unsigned> native{0};
    std::uint64_t device_epoch{};ID3D11Device* device_identity=nullptr;
    ID3D11Texture2D *page_surface=nullptr,*data_surface=nullptr;
    ref_texture page_texture,data_texture;xr_string page_name,data_name;
    bool registered=false,retired=false;u32 last_frame{};
    ~PainterRemovalGpu(){_RELEASE(page_surface);_RELEASE(data_surface);}
    bool ReserveNative()noexcept{unsigned zero=0;return native.compare_exchange_strong(zero,1,std::memory_order_acq_rel);}
    void BuildNative(ID3D11Device*device)noexcept{
        if(native.load(std::memory_order_acquire)!=1)return;device_identity=device;
        auto make=[&](ID3D11Texture2D**out,unsigned width,const std::vector<std::uint32_t>&values){
            if(!device||values.empty()||values.size()%width||values.size()/width>D3D11_REQ_TEXTURE2D_U_OR_V_DIMENSION)return E_INVALIDARG;
            D3D11_TEXTURE2D_DESC d={};d.Width=width;d.Height=unsigned(values.size()/width);d.MipLevels=1;d.ArraySize=1;
            d.Format=DXGI_FORMAT_R32_UINT;d.SampleDesc.Count=1;d.Usage=D3D11_USAGE_IMMUTABLE;d.BindFlags=D3D11_BIND_SHADER_RESOURCE;
            D3D11_SUBRESOURCE_DATA s={};s.pSysMem=values.data();s.SysMemPitch=width*4;return device->CreateTexture2D(&d,&s,out);
        };
        HRESULT hr=make(&page_surface,field->pageWidth,field->page);
        if(SUCCEEDED(hr))hr=make(&data_surface,field->dataWidth,field->data);
        if(FAILED(hr)){_RELEASE(page_surface);_RELEASE(data_surface);}native_result=hr;native.store(SUCCEEDED(hr)?2:3,std::memory_order_release);
    }
    bool Register(ID3D11Device*device,std::uint64_t epoch){
        if(native.load(std::memory_order_acquire)!=2||retired||epoch!=device_epoch||device!=device_identity)return false;
        if(registered)return true;
        auto bind=[](ref_texture&texture,const xr_string&name,ID3D11Texture2D*surface){
            if(!surface)return false;texture.create(name.c_str());if(!texture)return false;
            texture->surface_set(surface);texture->flags.bLoaded=true;texture->PostLoad();return true;
        };
        registered=bind(page_texture,page_name,page_surface)&&bind(data_texture,data_name,data_surface);return registered;
    }
};
namespace PainterRemovalCache {
inline std::vector<std::shared_ptr<PainterRemovalGpu>>&entries(){static auto*v=new std::vector<std::shared_ptr<PainterRemovalGpu>>;return *v;}
inline std::size_t charge(const painter_detail::DamageField&field){
    auto padded=[](std::size_t bytes){return ((bytes+65535)/65536)*65536;};
    return padded(field.page.size()*4)+padded(field.data.size()*4);
}
inline void Collect(){auto&v=entries();v.erase(std::remove_if(v.begin(),v.end(),[](const auto&p){
    return p.use_count()==1&&(p->retired||Device.dwFrame-p->last_frame>120);}),v.end());}
inline std::shared_ptr<PainterRemovalGpu>Prepare(const std::shared_ptr<const painter_detail::DamageField>&field,std::uint64_t epoch){
    if(!field||!field->available||!epoch)return {};
    for(auto&p:entries())if(!p->retired&&p->field==field&&p->device_epoch==epoch){p->last_frame=Device.dwFrame;return p;}
    Collect();auto&v=entries();const std::size_t cap=256u<<20,need=charge(*field);std::size_t used=0;
    for(const auto&p:v)used+=charge(*p->field);if(need>cap)return {};
    while(used>cap-need){auto oldest=v.end();for(auto it=v.begin();it!=v.end();++it)
        if(it->use_count()==1&&(oldest==v.end()||(*it)->last_frame<(*oldest)->last_frame))oldest=it;
        if(oldest==v.end())return {};used-=charge(*(*oldest)->field);v.erase(oldest);}
    auto p=std::make_shared<PainterRemovalGpu>();p->field=field;p->device_epoch=epoch;p->last_frame=Device.dwFrame;
    static std::uint64_t serial=0;char name[128];const auto id=++serial;
    xr_sprintf(name,"$user$painter_removal_%llu_pages",id);p->page_name=name;
    xr_sprintf(name,"$user$painter_removal_%llu_data",id);p->data_name=name;
    v.push_back(p);return p;
}
// R25: keep = the device that survived a reset; entries built on it stay valid, the rest retire.
inline void Clear(ID3D11Device*keep=nullptr){for(auto&p:entries())if(!keep||(p->device_identity&&p->device_identity!=keep))p->retired=true;Collect();}
}
#endif
