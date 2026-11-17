#pragma once
#ifdef USE_DX11
#include "AnalyticChannelField.h"
#include "PainterChannelMaterial.h"
#include "PainterFieldBudget.h"
#include <atomic>
#include <memory>
#include <vector>
// Prepare/Register/Collect/Clear are renderer-owner-thread operations. Workers
// touch only immutable CPU data and D3D11 device creation in BuildNative.
// The cache pins the final ResourceManager reference until owner-thread GC.
struct PainterChannelGpu {
    struct Child {
        unsigned slot{},count{};
        ID3D11Texture2D* surface=nullptr;
        ref_texture texture;
        xr_string name;
    };
    std::shared_ptr<const painter_detail::AnalyticChannelField> field;
    PainterFieldBudget::Lease budget;
    HRESULT native_result=E_PENDING; // Published by the terminal native release store.
    std::atomic<unsigned> native{0}; // 0 unscheduled,1building,2ready,3refused
    std::uint64_t device_epoch{};
    ID3D11Device* device_identity=nullptr;
    ID3D11Texture2D *word_surface=nullptr,*profile_surface=nullptr;
    ref_texture word_texture,profile_texture;
    xr_string word_name,profile_name;
    std::vector<Child> children;
    bool registered=false,retired=false;
    u32 last_frame{};
    ~PainterChannelGpu(){
        _RELEASE(word_surface);_RELEASE(profile_surface);
        for(auto&c:children)_RELEASE(c.surface);
    }
    bool ReserveNative()noexcept {unsigned empty=0;return native.compare_exchange_strong(empty,1,std::memory_order_acq_rel);}
    void BuildNative(ID3D11Device*device)noexcept {
        if(native.load(std::memory_order_acquire)!=1)return;
        device_identity=device;
        auto make=[&](ID3D11Texture2D**out,DXGI_FORMAT format,unsigned width,std::size_t elements,unsigned bytes,const void*data){
            if(!device||!elements||elements%width||elements/width>D3D11_REQ_TEXTURE2D_U_OR_V_DIMENSION)return E_INVALIDARG;
            D3D11_TEXTURE2D_DESC d={};d.Width=width;d.Height=unsigned(elements/width);d.MipLevels=1;d.ArraySize=1;d.Format=format;
            d.SampleDesc.Count=1;d.Usage=D3D11_USAGE_IMMUTABLE;d.BindFlags=D3D11_BIND_SHADER_RESOURCE;
            D3D11_SUBRESOURCE_DATA s={};s.pSysMem=data;s.SysMemPitch=width*bytes;return device->CreateTexture2D(&d,&s,out);
        };
        const auto&f=*field;
        HRESULT hr=make(&word_surface,DXGI_FORMAT_R32G32B32A32_FLOAT,f.wordWidth,f.words.size(),16,f.words.data());
        if(SUCCEEDED(hr))hr=make(&profile_surface,DXGI_FORMAT_R32_FLOAT,f.profileWidth,f.profiles.size(),4,f.profiles.data());
        for(std::size_t i=0;i<children.size()&&SUCCEEDED(hr);++i){const auto&c=f.children[i];
            hr=make(&children[i].surface,DXGI_FORMAT_R32G32_UINT,c.width,c.data.size(),8,c.data.data());}
        if(FAILED(hr)){_RELEASE(word_surface);_RELEASE(profile_surface);for(auto&c:children)_RELEASE(c.surface);}
        native_result=hr;native.store(SUCCEEDED(hr)?2:3,std::memory_order_release);
    }
    bool Register(ID3D11Device*device,std::uint64_t epoch){
        // Acquire the worker's state==2 publication BEFORE reading its device_identity write.
        if(native.load(std::memory_order_acquire)!=2||retired||epoch!=device_epoch||device!=device_identity)return false;
        if(registered)return true;
        auto bind=[](ref_texture&texture,const xr_string&name,ID3D11Texture2D*surface){
            if(!surface)return false;texture.create(name.c_str());if(!texture)return false;
            texture->surface_set(surface);texture->flags.bLoaded=true;texture->PostLoad();return true;
        };
        if(!bind(word_texture,word_name,word_surface)||!bind(profile_texture,profile_name,profile_surface))return false;
        for(auto&c:children)if(!bind(c.texture,c.name,c.surface))return false;
        registered=true;return true;
    }
    bool GetBindings(const ref_texture&pages,const ref_texture&data,unsigned slot,PainterChannelMaterial::Bindings&out)const {
        if(retired||!registered||!pages||!data||native.load(std::memory_order_acquire)!=2)return false;
        for(const auto&c:children)if(c.slot==slot){
            PainterChannelMaterial::Bindings b;b.anatomy_pages=pages;b.anatomy_data=data;
            b.cut_words=word_texture;b.cut_profiles=profile_texture;b.primitive_owners=c.texture;
            if(!b.ready())return false;out=b;return true;
        }
        return false;
    }
};
namespace PainterChannelCache {
inline std::vector<std::shared_ptr<PainterChannelGpu>>&entries(){static auto*p=new std::vector<std::shared_ptr<PainterChannelGpu>>;return *p;}
// Charge padded texture payloads and conservative64KiB allocation units per
// resource, not triangle counts. Current, pending and fenced-retired entries
// stay charged while their stage/packet pins exist.
inline std::size_t charge(const painter_detail::AnalyticChannelField&f){
    auto aligned=[](std::size_t n){return ((n+65535)/65536)*65536;};
    std::size_t n=aligned(f.words.size()*16)+aligned(f.profiles.size()*4);
    for(const auto&c:f.children)n+=aligned(c.data.size()*8);return n;
}
inline void Collect(){auto&v=entries();v.erase(std::remove_if(v.begin(),v.end(),[](const auto&p){
    return p.use_count()==1&&(p->retired||Device.dwFrame-p->last_frame>120);
}),v.end());}
inline void EvictAllUnpinned(){auto&v=entries();v.erase(std::remove_if(v.begin(),v.end(),[](const auto&p){return p.use_count()==1;}),v.end());}
inline bool EvictUnpinned(){
    auto&v=entries();auto oldest=v.end();for(auto i=v.begin();i!=v.end();++i)
        if(i->use_count()==1&&(oldest==v.end()||(*i)->last_frame<(*oldest)->last_frame))oldest=i;
    if(oldest==v.end())return false;v.erase(oldest);return true;
}
inline std::shared_ptr<PainterChannelGpu> Prepare(const std::shared_ptr<const painter_detail::AnalyticChannelField>&field,std::uint64_t epoch){
    if(!field||!epoch||!field->identity.snapshot||!field->cuts||field->cuts>field->maxCuts||field->children.empty())return {};
    for(auto&p:entries())if(!p->retired&&p->field==field&&p->device_epoch==epoch){p->last_frame=Device.dwFrame;return p;}
    Collect();const auto need=charge(*field);if(need>PainterFieldBudget::capacity)return {};
    auto p=std::make_shared<PainterChannelGpu>();
    while(!p->budget.Acquire(need))if(!EvictUnpinned())return {};
    p->field=field;p->device_epoch=epoch;p->last_frame=Device.dwFrame;
    static std::uint64_t serial=0;const auto id=++serial;char name[128];
    xr_sprintf(name,"$user$painter_channel_%llu_words",id);p->word_name=name;
    xr_sprintf(name,"$user$painter_channel_%llu_profiles",id);p->profile_name=name;
    p->children.reserve(field->children.size());
    for(const auto&c:field->children){PainterChannelGpu::Child child;child.slot=c.child;child.count=c.count;
        xr_sprintf(name,"$user$painter_channel_%llu_child_%u",id,c.child);child.name=name;p->children.push_back(std::move(child));}
    entries().push_back(p);return p;
}
// R25: keep = the device that survived a reset; entries built on it stay valid, the rest retire.
inline void Clear(ID3D11Device*keep=nullptr){for(auto&p:entries())if(!keep||(p->device_identity&&p->device_identity!=keep))p->retired=true;Collect();}
} // namespace PainterChannelCache
#endif
