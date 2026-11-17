#pragma once
#ifdef USE_DX11
// melee (2026-11-12, painter_voxel_mask_only 3): a body's dents on the GPU - painter_dent.h's s_painter_dents, two texels per dent
// (rest space: centre + radius, push + depth). Immutable, built once per publication on the main thread (as PainterTearGpu), bound
// to the value-3 dent vertex programs painter_vmaskd_* by PainterRemovalMaterial::clone.
#include "../../Include/xrRender/RuntimePackedMesh.h"
#include <cmath>
#include <vector>
struct PainterDentGpu {
    ID3D11Texture2D* surface=nullptr;ref_texture texture;
    ~PainterDentGpu(){_RELEASE(surface);}
    bool Build(ID3D11Device*device,const std::vector<gfy_packed::Dent>&dents,const void*key){
        if(!device||dents.empty()||dents.size()>1024)return false;
        std::vector<float> data;data.reserve(dents.size()*8);
        for(const auto&d:dents){const float row[8]={d.centre[0],d.centre[1],d.centre[2],d.radius,d.push[0],d.push[1],d.push[2],d.depth};
            for(float v:row)if(!std::isfinite(v))return false;
            if(!(d.radius>0.f)||d.radius>.5f||d.depth<0.f||d.depth>.1f)return false;
            data.insert(data.end(),row,row+8);}
        D3D11_TEXTURE2D_DESC desc={};desc.Width=UINT(2*dents.size());desc.Height=1;desc.MipLevels=1;desc.ArraySize=1;
        desc.Format=DXGI_FORMAT_R32G32B32A32_FLOAT;desc.SampleDesc.Count=1;desc.Usage=D3D11_USAGE_IMMUTABLE;desc.BindFlags=D3D11_BIND_SHADER_RESOURCE;
        D3D11_SUBRESOURCE_DATA s={};s.pSysMem=data.data();s.SysMemPitch=desc.Width*16;
        if(FAILED(device->CreateTexture2D(&desc,&s,&surface)))return false;
        string128 name;xr_sprintf(name,"$user$painter_dents_%p",key);texture.create(name);
        if(!texture){_RELEASE(surface);return false;}
        texture->surface_set(surface);texture->flags.bLoaded=true;texture->PostLoad();
        Msg("[painter_mask_only] dents_bound dents=%u",unsigned(dents.size()));return true;}
    // melee (round 2): a body's blade slits for painter_slit.h's s_painter_slits, three texels per slit (centre + half length, side +
    // half width, normal + half depth; rest space)
    bool BuildSlits(ID3D11Device*device,const std::vector<gfy_packed::Slit>&slits,const void*key){
        if(!device||slits.empty()||slits.size()>256)return false;
        std::vector<float> data;data.reserve(slits.size()*12);
        for(const auto&s:slits){const float row[12]={s.centre[0],s.centre[1],s.centre[2],s.half_side,s.side[0],s.side[1],s.side[2],s.half_normal,
                s.normal[0],s.normal[1],s.normal[2],s.half_depth};
            for(float v:row)if(!std::isfinite(v))return false;
            data.insert(data.end(),row,row+12);}
        if(!Create(device,data.data(),UINT(3*slits.size())))return false;
        string128 name;xr_sprintf(name,"$user$painter_slits_%p",key);texture.create(name);
        if(!texture){_RELEASE(surface);return false;}
        texture->surface_set(surface);texture->flags.bLoaded=true;texture->PostLoad();
        Msg("[painter_mask_only] slits_bound slits=%u",unsigned(slits.size()));return true;}
    // the texture of a body without slits: one zero texel (painter_slit.h reads no slit), one per device, never released
    static ref_texture NoSlits(ID3D11Device*device){
        static ID3D11Device*made=nullptr;static PainterDentGpu*t=nullptr;
        if(device&&made!=device){auto*n=new PainterDentGpu;const float zero[4]={0,0,0,0};
            if(n->Create(device,zero,1)){string128 name;xr_sprintf(name,"$user$painter_no_slits_%p",device);n->texture.create(name);
                if(n->texture){n->texture->surface_set(n->surface);n->texture->flags.bLoaded=true;n->texture->PostLoad();t=n;made=device;}}}
        return t?t->texture:ref_texture();}
private:
    bool Create(ID3D11Device*device,const float*data,UINT texels){
        D3D11_TEXTURE2D_DESC desc={};desc.Width=texels;desc.Height=1;desc.MipLevels=1;desc.ArraySize=1;
        desc.Format=DXGI_FORMAT_R32G32B32A32_FLOAT;desc.SampleDesc.Count=1;desc.Usage=D3D11_USAGE_IMMUTABLE;desc.BindFlags=D3D11_BIND_SHADER_RESOURCE;
        D3D11_SUBRESOURCE_DATA s={};s.pSysMem=data;s.SysMemPitch=texels*16;
        return SUCCEEDED(device->CreateTexture2D(&desc,&s,&surface));}
};
#endif
