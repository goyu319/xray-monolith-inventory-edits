#pragma once
#include "PainterBurnSessions.h"
#include <d3d11.h>
#include <wrl/client.h>
#include <functional>

namespace painter_burn {
// Construct on the resource worker with a multithread-capable native device.
// Pump on the immediate-context thread, under the existing global upload budget.
class GpuFamily {
    using Texture=Microsoft::WRL::ComPtr<ID3D11Texture2D>;
    using View=Microsoft::WRL::ComPtr<ID3D11ShaderResourceView>;
    Texture tex[2];View views[2];
    std::shared_ptr<const Delta> current,previous,pending;
    unsigned front=0;std::size_t cursor=0;bool replayPrevious{};
    U64 family{},serial{};
public:
    HRESULT prepare(ID3D11Device* device,U64 lineage,U64 birthSerial) {
        if(!device||tex[0]||!lineage||!birthSerial)return E_INVALIDARG;
        D3D11_TEXTURE2D_DESC desc{};desc.Width=TextureWidth;desc.Height=Family::GpuTexels/TextureWidth;desc.MipLevels=desc.ArraySize=desc.SampleDesc.Count=1;
        desc.Format=DXGI_FORMAT_R32G32B32A32_UINT;desc.Usage=D3D11_USAGE_DEFAULT;desc.BindFlags=D3D11_BIND_SHADER_RESOURCE;
        static const std::vector<Texel> zero(Family::GpuTexels);
        D3D11_SUBRESOURCE_DATA data{};data.pSysMem=zero.data();data.SysMemPitch=TextureWidth*sizeof(Texel);
        Texture textures[2];View resources[2];
        for(unsigned i=0;i<2;++i){
            HRESULT hr=device->CreateTexture2D(&desc,&data,&textures[i]);if(FAILED(hr))return hr;
            hr=device->CreateShaderResourceView(textures[i].Get(),nullptr,&resources[i]);if(FAILED(hr))return hr;
        }
        for(unsigned i=0;i<2;++i){tex[i]=textures[i];views[i]=resources[i];}
        family=lineage;serial=birthSerial;return S_OK;
    }
    bool submit(std::shared_ptr<const Delta> d) {
        if(!d||!tex[0]||d->lineage!=family||d->serial!=serial||d->retired||pending)return false;
        if(current&&d->revision<=current->revision)return false;
        for(const auto& s:d->spans){
            if(s.data.empty()||s.data.size()>TextureWidth||s.first>=Family::GpuTexels||
               s.first%TextureWidth+s.data.size()>TextureWidth||s.first+s.data.size()>Family::GpuTexels)return false;
        }
        pending=std::move(d);cursor=0;replayPrevious=bool(previous);return true;
    }
    // hasTime must check the caller's shared deadline before each native update.
    // Returning false retains the old complete SRV plus the exact continuation point.
    template<class HasTime> bool pump(ID3D11DeviceContext* ctx,unsigned& byteCredit,HasTime&& hasTime) {
        if(!pending||!ctx)return false;
        while(true){
            const auto& v=replayPrevious?previous->spans:pending->spans;
            while(cursor<v.size()){
                const auto& s=v[cursor];const unsigned n=unsigned(s.data.size()),bytes=n*sizeof(Texel);
                if(bytes>byteCredit||!hasTime())return false;
                const unsigned x=s.first%TextureWidth,y=s.first/TextureWidth;
                D3D11_BOX box{x,y,0,x+n,y+1,1};
                ctx->UpdateSubresource(tex[front^1u].Get(),0,&box,s.data.data(),bytes,0);
                byteCredit-=bytes;++cursor;
            }
            if(replayPrevious){replayPrevious=false;cursor=0;continue;}
            previous=pending;current=std::move(pending);front^=1;return true;
        }
    }
    ID3D11ShaderResourceView* view()const{return current?views[front].Get():nullptr;}
    std::array<float,4> meta(OwnerKey owner,U32 tick)const{
        const unsigned tag=current?current->tag(owner):0;
        return {tag?float(Slots-1):0.f,float(tag),float(tick&65535u),float(tick>>16)};
    }
};
}
