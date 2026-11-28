#pragma once
#include <d3d11.h>
#include <d3dcompiler.h>
#include <wrl/client.h>
#include <atomic>
#include <new>
#include <cstring>
#include <bcrypt.h>

namespace painter_torn_draw {
using Microsoft::WRL::ComPtr;

// Private data on the exact resource retains only the two optional programs.
// The marker owns no reference back to the resource that holds it.
struct __declspec(uuid("14736d57-2601-4726-8ef0-4a4762638865")) IPrograms : IUnknown {
    virtual unsigned sampleCount() const = 0;
    virtual ID3D11PixelShader* nonBaked() const = 0;
    virtual ID3D11PixelShader* baked() const = 0;
};
class Programs final : public IPrograms {
    std::atomic<ULONG> references{1};
    ComPtr<ID3D11PixelShader> first,second;
    const unsigned samples;
public:
    Programs(ID3D11PixelShader* base,ID3D11PixelShader* a,ID3D11PixelShader* b,unsigned sampleCount)
        : first(base==a?nullptr:a),second(base==b?nullptr:b),samples(sampleCount) {}
    unsigned sampleCount() const override { return samples; }
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID id,void** result) override {
        if(!result)return E_POINTER;
        *result=nullptr;
        if(id!=__uuidof(IUnknown)&&id!=__uuidof(IPrograms))return E_NOINTERFACE;
        *result=static_cast<IPrograms*>(this);AddRef();return S_OK;
    }
    ULONG STDMETHODCALLTYPE AddRef() override {return ++references;}
    ULONG STDMETHODCALLTYPE Release() override {const auto n=--references;if(!n)delete this;return n;}
    ID3D11PixelShader* nonBaked() const override {return first.Get();}
    ID3D11PixelShader* baked() const override {return second.Get();}
};

inline bool signatures(ID3D11ShaderReflection& a,ID3D11ShaderReflection& b,UINT count,bool output) {
    for(UINT i=0;i<count;++i){
        D3D11_SIGNATURE_PARAMETER_DESC x{},y{};
        HRESULT ax=output?a.GetOutputParameterDesc(i,&x):a.GetInputParameterDesc(i,&x);
        HRESULT by=output?b.GetOutputParameterDesc(i,&y):b.GetInputParameterDesc(i,&y);
        if(FAILED(ax)||FAILED(by)||!x.SemanticName||!y.SemanticName||
           std::strcmp(x.SemanticName,y.SemanticName)||x.SemanticIndex!=y.SemanticIndex||
           x.Register!=y.Register||x.SystemValueType!=y.SystemValueType||x.ComponentType!=y.ComponentType||
           x.Mask!=y.Mask||x.Stream!=y.Stream||x.MinPrecision!=y.MinPrecision)return false;
    }
    return true;
}
inline bool sameType(ID3D11ShaderReflectionType* base,ID3D11ShaderReflectionType* target){
    if(!base||!target)return false;
    D3D11_SHADER_TYPE_DESC a{},b{};
    if(FAILED(base->GetDesc(&a))||FAILED(target->GetDesc(&b))||a.Class!=b.Class||a.Type!=b.Type||
       a.Rows!=b.Rows||a.Columns!=b.Columns||a.Elements!=b.Elements||a.Members!=b.Members||a.Offset!=b.Offset)return false;
    for(UINT i=0;i<b.Members;++i){
        const char* x=base->GetMemberTypeName(i);const char* y=target->GetMemberTypeName(i);
        if(!x||!y||std::strcmp(x,y)||!sameType(base->GetMemberTypeByIndex(i),target->GetMemberTypeByIndex(i)))return false;
    }
    return true;
}
inline bool bufferCompatible(ID3D11ShaderReflectionConstantBuffer* base,ID3D11ShaderReflectionConstantBuffer* target){
    if(!base||!target)return false;
    D3D11_SHADER_BUFFER_DESC a{},b{};
    if(FAILED(base->GetDesc(&a))||FAILED(target->GetDesc(&b))||a.Type!=b.Type||a.Size<b.Size)return false;
    for(UINT i=0;i<b.Variables;++i){
        auto* v=target->GetVariableByIndex(i);D3D11_SHADER_VARIABLE_DESC y{};
        if(!v||FAILED(v->GetDesc(&y))||!y.Name)return false;
        if(!(y.uFlags&D3D_SVF_USED))continue;
        auto* w=base->GetVariableByName(y.Name);D3D11_SHADER_VARIABLE_DESC x{};
        if(!w||FAILED(w->GetDesc(&x))||x.Size!=y.Size||
           x.StartOffset!=y.StartOffset||!sameType(w->GetType(),v->GetType()))return false;
    }
    return true;
}
inline bool compatible(const void* source,size_t sourceSize,const void* target,size_t targetSize) {
    ComPtr<ID3D11ShaderReflection> a,b;
    if(!source||!target||FAILED(D3DReflect(source,sourceSize,IID_PPV_ARGS(&a)))||
       FAILED(D3DReflect(target,targetSize,IID_PPV_ARGS(&b))))return false;
    D3D11_SHADER_DESC x{},y{};
    if(FAILED(a->GetDesc(&x))||FAILED(b->GetDesc(&y))||x.Version!=y.Version||
       x.InputParameters!=y.InputParameters||x.OutputParameters!=y.OutputParameters||
       !signatures(*a.Get(),*b.Get(),x.InputParameters,false)||!signatures(*a.Get(),*b.Get(),x.OutputParameters,true))return false;
    for(UINT i=0;i<y.BoundResources;++i){
        D3D11_SHADER_INPUT_BIND_DESC t{};
        if(FAILED(b->GetResourceBindingDesc(i,&t))||!t.Name)return false;
        bool found=false;
        for(UINT j=0;j<x.BoundResources&&!found;++j){
            D3D11_SHADER_INPUT_BIND_DESC s{};
            if(FAILED(a->GetResourceBindingDesc(j,&s))||!s.Name)return false;
            if(s.Type!=t.Type||s.BindPoint!=t.BindPoint||s.BindCount!=t.BindCount||s.Dimension!=t.Dimension||
               s.ReturnType!=t.ReturnType||s.NumSamples!=t.NumSamples)continue;
            found=t.Type==D3D_SIT_CBUFFER?
                bufferCompatible(a->GetConstantBufferByName(s.Name),b->GetConstantBufferByName(t.Name)):
                !std::strcmp(s.Name,t.Name);
        }
        if(!found)return false;
    }
    return true;
}
// Source/profile qualification is performed by the load path. These native payloads
// ensure that unknown draw states can always fall back to the conventional shader.
inline bool nativeDigest(const void* bytes,size_t size,const unsigned char (&expected)[32]){
    if(!bytes||!size||size>0xffffffffu)return false;
    BCRYPT_ALG_HANDLE alg=nullptr;unsigned char out[32]{};
    if(BCryptOpenAlgorithmProvider(&alg,BCRYPT_SHA256_ALGORITHM,nullptr,0)<0)return false;
    BCRYPT_HASH_HANDLE hash=nullptr;
    bool ok=BCryptCreateHash(alg,&hash,nullptr,0,nullptr,0,0)>=0;
    if(ok)ok=BCryptHashData(hash,reinterpret_cast<PUCHAR>(const_cast<void*>(bytes)),static_cast<ULONG>(size),0)>=0 &&
             BCryptFinishHash(hash,out,sizeof(out),0)>=0;
    if(hash)BCryptDestroyHash(hash);
    BCryptCloseAlgorithmProvider(alg,0);
    return ok&&!std::memcmp(out,expected,32);
}
inline bool certifiedPrimary(const void* bytes,size_t size,unsigned samples){
    if(samples==1u){
        static constexpr unsigned char conventional[]={0x5c,0x37,0x5e,0x42,0x95,0xde,0x01,0xd7,0x42,0xd0,0xba,0x60,0xa8,0x83,0x28,0x39,0x3b,0x5d,0x31,0x19,0xfb,0xcc,0x44,0xcb,0xd9,0x32,0x42,0xce,0x43,0x58,0x4d,0x34},linked[]={0xd6,0xfa,0xe7,0x5f,0xed,0x87,0x59,0x16,0xee,0xf2,0xfd,0x6d,0x2e,0xb9,0x7c,0x22,0x5e,0x00,0x3d,0x22,0x94,0x4a,0x33,0xb8,0x7d,0xd8,0xfe,0xc6,0x46,0xa8,0x0f,0xed};
        return nativeDigest(bytes,size,conventional)||nativeDigest(bytes,size,linked);
    }
    if(samples==2u){
        static constexpr unsigned char conventional[]={0x80,0x55,0x1f,0x93,0x5c,0x5f,0x28,0x73,0x3f,0x81,0x3a,0xda,0xf0,0x8c,0xbf,0x99,0xea,0x72,0x07,0x88,0xd9,0x02,0xb7,0xb4,0x65,0xde,0xdd,0x3f,0xcb,0x04,0x01,0xd9},linked[]={0xd6,0xfa,0xe7,0x5f,0xed,0x87,0x59,0x16,0xee,0xf2,0xfd,0x6d,0x2e,0xb9,0x7c,0x22,0x5e,0x00,0x3d,0x22,0x94,0x4a,0x33,0xb8,0x7d,0xd8,0xfe,0xc6,0x46,0xa8,0x0f,0xed};
        return nativeDigest(bytes,size,conventional)||nativeDigest(bytes,size,linked);
    }
    if(samples==4u){
        static constexpr unsigned char conventional[]={0xa3,0xe1,0xe7,0xb7,0xce,0x0a,0x45,0x54,0xc3,0xf1,0x70,0x1d,0x8d,0x9b,0xbb,0x52,0x52,0x35,0x3a,0x2d,0xad,0x6f,0x05,0xe2,0x5f,0xe0,0xf3,0x2f,0x4d,0x0e,0x58,0x30},linked[]={0xff,0xe6,0x74,0x70,0xf1,0xbd,0xee,0x46,0xe9,0xbf,0xd2,0xca,0xb2,0x62,0x55,0x96,0x8b,0xf9,0xc4,0x1e,0xd5,0xc1,0x07,0x27,0x05,0xb4,0x60,0x3e,0xef,0x9a,0x37,0x3e};
        return nativeDigest(bytes,size,conventional)||nativeDigest(bytes,size,linked);
    }
    return false;
}
inline bool publish(ID3D11PixelShader* ordinary,ID3D11PixelShader* general,ID3D11PixelShader* baked,
    const void* ordinaryBytes,size_t ordinarySize,const void* generalBytes,size_t generalSize,const void* bakedBytes,size_t bakedSize,unsigned samples) {
    if(!ordinary||!general||!baked||ordinary==general||ordinary==baked||general==baked)return false;
    static constexpr unsigned char local[]={0x48,0x83,0x12,0xa5,0xa3,0xf1,0x7a,0x4d,0x93,0xd6,0x43,0x62,0xc8,0xfb,0x6a,0xad,0x3b,0x67,0x9,0xf6,0xf0,0x4f,0x62,0x93,0xfb,0x1d,0x9c,0x72,0x5d,0x34,0xd,0xb9};
    static constexpr unsigned char ready[]={0x73,0x26,0x17,0x2c,0x5e,0x9a,0x35,0xe6,0x45,0xa5,0x33,0xc7,0xd0,0x6b,0x78,0xc3,0xdf,0xb0,0xdf,0xc2,0x82,0xa3,0x93,0x13,0x1,0x76,0x69,0x8,0x3,0x14,0x29,0x95};
    if(!certifiedPrimary(ordinaryBytes,ordinarySize,samples)||!nativeDigest(generalBytes,generalSize,local)||!nativeDigest(bakedBytes,bakedSize,ready))return false;
    if(!compatible(ordinaryBytes,ordinarySize,generalBytes,generalSize)||!compatible(ordinaryBytes,ordinarySize,bakedBytes,bakedSize))return false;
    ComPtr<ID3D11Device> a,b,c;ordinary->GetDevice(&a);general->GetDevice(&b);baked->GetDevice(&c);
    if(!a||a.Get()!=b.Get()||a.Get()!=c.Get())return false;
    ComPtr<IPrograms> p;p.Attach(new(std::nothrow) Programs(ordinary,general,baked,samples));
    return p&&SUCCEEDED(ordinary->SetPrivateDataInterface(__uuidof(IPrograms),p.Get()));
}
// Cache the immutable sample layout on the view itself. No render target is retained across a reset.
inline unsigned standardSamples(ID3D11RenderTargetView* view){
    static constexpr GUID key={0xc091e7eb,0x4989,0x45ed,{0xb0,0x0f,0x98,0x8c,0xd4,0xa7,0xce,0x4a}};
    unsigned samples=0;
    UINT bytes=sizeof(samples);
    if(SUCCEEDED(view->GetPrivateData(key,&bytes,&samples)) && bytes==sizeof(samples))return samples;
    ComPtr<ID3D11Resource> resource;
    view->GetResource(resource.GetAddressOf());
    ComPtr<ID3D11Texture2D> color;
    if(resource && SUCCEEDED(resource.As(&color))){
        D3D11_TEXTURE2D_DESC desc{};
        color->GetDesc(&desc);
        if(desc.SampleDesc.Quality==D3D11_STANDARD_MULTISAMPLE_PATTERN)samples=desc.SampleDesc.Count;
    }
    view->SetPrivateData(key,sizeof(samples),&samples);
    return samples;
}
struct Binding {
    unsigned samples=0;
    ComPtr<ID3D11PixelShader> ordinary,nonBaked,baked;
    bool read(ID3D11PixelShader* base){
        ComPtr<IUnknown> token;UINT size=sizeof(IUnknown*);
        if(!base||FAILED(base->GetPrivateData(__uuidof(IPrograms),&size,token.GetAddressOf()))||!token||size!=sizeof(IUnknown*))return false;
        ComPtr<IPrograms> pair;if(FAILED(token.As(&pair)))return false;
        samples=pair->sampleCount();
        ordinary=base;nonBaked=pair->nonBaked()?pair->nonBaked():base;baked=pair->baked()?pair->baked():base;
        return true;
    }
    ID3D11PixelShader* choose(ID3D11PixelShader* active,bool bakedFlag,bool live,bool fieldKnown,ID3D11RenderTargetView* albedo)const {
        if(!ordinary||active!=ordinary.Get()||!albedo)return nullptr;
        D3D11_RENDER_TARGET_VIEW_DESC d{};albedo->GetDesc(&d);
        if(!fieldKnown||!live||d.Format!=DXGI_FORMAT_B8G8R8A8_UNORM)return nullptr;
        if(samples==1){
            if(d.ViewDimension!=D3D11_RTV_DIMENSION_TEXTURE2D)return nullptr;
        }else if(d.ViewDimension!=D3D11_RTV_DIMENSION_TEXTURE2DMS||standardSamples(albedo)!=samples)return nullptr;
        return bakedFlag?baked.Get():nonBaked.Get();
    }
};
}

