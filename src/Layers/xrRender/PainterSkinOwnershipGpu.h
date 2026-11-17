#pragma once
#ifdef USE_DX11
#include "../../Include/xrRender/SkinOwnershipField.h"
#include "PainterChannelGpu.h"
#include <atomic>
#include <memory>
#include <vector>
#include <cmath>
struct PainterSkinOwnershipGpu {
    struct Child {unsigned slot{},count{};ID3D11Texture2D*surface=nullptr;ref_texture texture;xr_string name;};
    std::shared_ptr<const painter_detail::SkinOwnershipField> field;
    PainterFieldBudget::Lease budget;
    HRESULT native_result=E_PENDING; // Published by the terminal native release store.
    std::atomic<unsigned> native{0};
    std::uint64_t device_epoch{};ID3D11Device*device_identity=nullptr;
    ID3D11Texture2D*plane_surface=nullptr;ref_texture plane_texture;xr_string plane_name;
    std::vector<Child> children;bool registered=false,retired=false;u32 last_frame{};
    ~PainterSkinOwnershipGpu(){_RELEASE(plane_surface);for(auto&c:children)_RELEASE(c.surface);}
    bool ReserveNative()noexcept{unsigned zero=0;return native.compare_exchange_strong(zero,1,std::memory_order_acq_rel);}
    void BuildNative(ID3D11Device*device)noexcept{
        if(native.load(std::memory_order_acquire)!=1)return;device_identity=device;
        HRESULT hr=E_FAIL;
        try{
            auto make=[&](ID3D11Texture2D**out,DXGI_FORMAT format,unsigned width,std::size_t count,unsigned stride,const void*data){
                if(!device||!count||count%width||count/width>D3D11_REQ_TEXTURE2D_U_OR_V_DIMENSION)return E_INVALIDARG;
                D3D11_TEXTURE2D_DESC d={};d.Width=width;d.Height=unsigned(count/width);d.MipLevels=1;d.ArraySize=1;d.Format=format;
                d.SampleDesc.Count=1;d.Usage=D3D11_USAGE_IMMUTABLE;d.BindFlags=D3D11_BIND_SHADER_RESOURCE;
                D3D11_SUBRESOURCE_DATA s={};s.pSysMem=data;s.SysMemPitch=width*stride;return device->CreateTexture2D(&d,&s,out);
            };
            auto planes=field->planes;planes.resize(((planes.size()+4095)/4096)*4096);
            hr=make(&plane_surface,DXGI_FORMAT_R32G32B32A32_FLOAT,4096,planes.size(),16,planes.data());
            for(std::size_t i=0;i<children.size()&&SUCCEEDED(hr);++i){const auto&c=field->children[i];
                hr=make(&children[i].surface,DXGI_FORMAT_R32G32_UINT,c.width,c.data.size(),8,c.data.data());}
        }catch(...){hr=E_OUTOFMEMORY;}
        if(FAILED(hr)){_RELEASE(plane_surface);for(auto&c:children)_RELEASE(c.surface);}
        native_result=hr;native.store(SUCCEEDED(hr)?2:3,std::memory_order_release);
    }
    bool Register(ID3D11Device*device,std::uint64_t epoch){
        if(native.load(std::memory_order_acquire)!=2||retired||epoch!=device_epoch||device!=device_identity)return false;
        if(registered)return true;
        auto bind=[](ref_texture&texture,const xr_string&name,ID3D11Texture2D*surface){
            if(!surface)return false;texture.create(name.c_str());if(!texture)return false;
            texture->surface_set(surface);texture->flags.bLoaded=true;texture->PostLoad();return true;
        };
        if(!bind(plane_texture,plane_name,plane_surface))return false;
        for(auto&c:children)if(!bind(c.texture,c.name,c.surface))return false;
        registered=true;return true;
    }
    bool GetBindings(unsigned slot,ref_texture&planes,ref_texture&regions)const{
        if(retired||!registered||native.load(std::memory_order_acquire)!=2)return false;
        for(const auto&c:children)if(c.slot==slot){planes=plane_texture;regions=c.texture;return planes&&regions;}
        return false;
    }
};
namespace PainterSkinOwnershipCache {
inline std::vector<std::shared_ptr<PainterSkinOwnershipGpu>>&entries(){static auto*p=new std::vector<std::shared_ptr<PainterSkinOwnershipGpu>>;return *p;}
inline bool valid(const painter_detail::SkinOwnershipField&f){
    try{f.validate();}catch(...){return false;}
    if(f.planes.size()>4096u*16384u)return false;
    for(const auto&c:f.children)if(c.width!=4096||c.data.size()/c.width>16384)return false;
    return true;
}
inline std::size_t charge(const painter_detail::SkinOwnershipField&f){
    auto aligned=[](std::size_t n){return ((n+65535)/65536)*65536;};
    std::size_t bytes=aligned(((f.planes.size()+4095)/4096)*4096*16);
    for(const auto&c:f.children)bytes+=aligned(c.data.size()*8);return bytes;
}
inline void Collect(){auto&v=entries();v.erase(std::remove_if(v.begin(),v.end(),[](const auto&p){return p.use_count()==1;}),v.end());}
inline std::shared_ptr<PainterSkinOwnershipGpu>Prepare(const std::shared_ptr<const painter_detail::SkinOwnershipField>&field,std::uint64_t epoch){
    if(!field||!epoch||!valid(*field))return {};
    for(auto&p:entries())if(!p->retired&&p->field==field&&p->device_epoch==epoch){p->last_frame=Device.dwFrame;return p;}
    Collect();const auto need=charge(*field);if(need>PainterFieldBudget::capacity)return {};
    auto p=std::make_shared<PainterSkinOwnershipGpu>();
    while(!p->budget.Acquire(need))if(!PainterChannelCache::EvictUnpinned())return {};
    p->field=field;p->device_epoch=epoch;p->last_frame=Device.dwFrame;
    static std::uint64_t serial=0;const auto id=++serial;char name[128];
    xr_sprintf(name,"$user$painter_skin_%llu_planes",id);p->plane_name=name;
    p->children.reserve(field->children.size());
    for(const auto&c:field->children){PainterSkinOwnershipGpu::Child child;child.slot=c.child;child.count=c.count;
        xr_sprintf(name,"$user$painter_skin_%llu_child_%u",id,c.child);child.name=name;p->children.push_back(std::move(child));}
    entries().push_back(p);return p;
}
inline void Clear(ID3D11Device*keep=nullptr){for(auto&p:entries())if(!keep||(p->device_identity&&p->device_identity!=keep))p->retired=true;Collect();}
}
#endif
