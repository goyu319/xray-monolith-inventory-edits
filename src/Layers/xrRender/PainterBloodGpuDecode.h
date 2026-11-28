#pragma once
#include <d3d11.h>
#include <stdexcept>
#include <vector>
#include <array>
#include <cstring>
#include "PainterBloodTextureCodec.h"
#include "PainterBloodDecodeProgram.h"
namespace PainterBloodGpuDecode {
template<class T> struct Com {T* p=nullptr;~Com(){if(p)p->Release();}Com()=default;Com(const Com&)=delete;Com&operator=(const Com&)=delete;};
inline void require(HRESULT hr,const char* message){if(FAILED(hr))throw std::runtime_error(message);}
struct SavedCompute {
    ID3D11DeviceContext* context;
    Com<ID3D11ComputeShader> shader;
    std::array<ID3D11ClassInstance*,256> instances{};UINT count=256;
    Com<ID3D11ShaderResourceView> source;
    Com<ID3D11UnorderedAccessView> output;
    explicit SavedCompute(ID3D11DeviceContext* c):context(c){
        c->CSGetShader(&shader.p,instances.data(),&count);
        c->CSGetShaderResources(0,1,&source.p);c->CSGetUnorderedAccessViews(0,1,&output.p);
    }
    ~SavedCompute(){
        ID3D11ShaderResourceView* noSource=nullptr;ID3D11UnorderedAccessView* noOutput=nullptr;UINT keep=UINT(-1);
        context->CSSetShaderResources(0,1,&noSource);context->CSSetUnorderedAccessViews(0,1,&noOutput,&keep);
        context->CSSetShader(shader.p,instances.data(),count);
        context->CSSetShaderResources(0,1,&source.p);context->CSSetUnorderedAccessViews(0,1,&output.p,&keep);
        for(UINT i=0;i<count;++i)if(instances[i])instances[i]->Release();
    }
};
inline void createProgram(ID3D11Device* device,ID3D11ComputeShader** destination){
    require(device->CreateComputeShader(painter_blood_decode_bytecode,sizeof(painter_blood_decode_bytecode),nullptr,destination),"blood_decode_shader");
}
inline PainterBloodTextureCodec::Image readWithProgram(ID3D11Device* device,ID3D11DeviceContext* context,ID3D11ComputeShader* program,ID3D11ShaderResourceView* source,unsigned width,unsigned height){
    if(!device||!context||!program||!source||!width||!height||width>16384||height>16384||std::size_t(width)*height>16u*1024u*1024u)
        throw std::runtime_error("blood_decode_dimensions_or_device");
    // All allocations precede context changes. No runtime compiler or shader
    // file access: bytecode is generated from painter_blood_decode.cs offline.
    D3D11_TEXTURE2D_DESC d{};d.Width=width;d.Height=height;d.MipLevels=d.ArraySize=1;d.Format=DXGI_FORMAT_R32G32B32A32_FLOAT;
    d.SampleDesc.Count=1;d.Usage=D3D11_USAGE_DEFAULT;d.BindFlags=D3D11_BIND_UNORDERED_ACCESS;
    Com<ID3D11Texture2D> target,staging;Com<ID3D11UnorderedAccessView> output;
    require(device->CreateTexture2D(&d,nullptr,&target.p),"blood_decode_target");
    require(device->CreateUnorderedAccessView(target.p,nullptr,&output.p),"blood_decode_uav");
    d.Usage=D3D11_USAGE_STAGING;d.BindFlags=0;d.CPUAccessFlags=D3D11_CPU_ACCESS_READ;
    require(device->CreateTexture2D(&d,nullptr,&staging.p),"blood_decode_staging");
    PainterBloodTextureCodec::Image result;result.width=width;result.height=height;result.pixels.resize(std::size_t(width)*height);
    SavedCompute saved(context);
    context->CSSetShader(program,nullptr,0);context->CSSetShaderResources(0,1,&source);
    UINT keep=UINT(-1);context->CSSetUnorderedAccessViews(0,1,&output.p,&keep);
    context->Dispatch((width+7)/8,(height+7)/8,1);
    ID3D11UnorderedAccessView* nil=nullptr;context->CSSetUnorderedAccessViews(0,1,&nil,&keep);
    context->CopyResource(staging.p,target.p);
    D3D11_MAPPED_SUBRESOURCE mapped{};require(context->Map(staging.p,0,D3D11_MAP_READ,0,&mapped),"blood_decode_map");
    for(unsigned y=0;y<height;++y)std::memcpy(result.pixels.data()+std::size_t(y)*width,static_cast<const unsigned char*>(mapped.pData)+std::size_t(y)*mapped.RowPitch,std::size_t(width)*sizeof(PainterBloodTextureCodec::Pixel));
    context->Unmap(staging.p,0);
    return result;
}
// One-shot entry for direct callers on an externally owned device.
inline PainterBloodTextureCodec::Image read(ID3D11Device* device,ID3D11DeviceContext* context,ID3D11ShaderResourceView* source,unsigned width,unsigned height){
    if(!device)throw std::runtime_error("blood_decode_dimensions_or_device");
    Com<ID3D11ComputeShader> shader;createProgram(device,&shader.p);
    return readWithProgram(device,context,shader.p,source,width,height);
}
} // namespace PainterBloodGpuDecode
