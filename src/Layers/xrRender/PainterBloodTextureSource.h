#pragma once
#ifdef USE_DX11
#include <d3d11.h>
#include <D3DX11tex.h>
#include <bcrypt.h>
#include <atomic>
#include <memory>
#include <mutex>
#include <array>
#include <string>
#include <vector>
#include <stdexcept>
#include <cstring>
#pragma comment(lib,"bcrypt.lib")
// Exact immutable DDS provenance captured by the existing texture loader. It
// retains no DDS payload. Every later read authenticates the bytes the loader
// actually consumed, including VFS precedence/fallback and its load-mip policy.
namespace PainterBloodTextureSource {
template<class T> struct Com {
 T* p=nullptr;~Com(){if(p)p->Release();}Com()=default;
 Com(const Com&)=delete;Com&operator=(const Com&)=delete;
};
inline void require(HRESULT hr,const char* why){if(FAILED(hr))throw std::runtime_error(why);}
using Digest=std::array<unsigned char,32>;
inline Digest hash(const void* data,std::size_t size){
 if(!data||!size||size>0xffffffffu)throw std::runtime_error("blood_source_hash_size");
 struct Algorithm {BCRYPT_ALG_HANDLE h=nullptr;Algorithm(){if(BCryptOpenAlgorithmProvider(&h,BCRYPT_SHA256_ALGORITHM,nullptr,0)<0)throw std::runtime_error("blood_source_sha256_provider");}~Algorithm(){if(h)BCryptCloseAlgorithmProvider(h,0);}};
 static Algorithm algorithm;Digest out{};
 DWORD objectSize=0,returned=0;
 if(BCryptGetProperty(algorithm.h,BCRYPT_OBJECT_LENGTH,reinterpret_cast<PUCHAR>(&objectSize),sizeof(objectSize),&returned,0)<0)throw std::runtime_error("blood_source_sha256_property");
 std::vector<unsigned char> object(objectSize);
 struct Hash {BCRYPT_HASH_HANDLE h=nullptr;~Hash(){if(h)BCryptDestroyHash(h);}} state;
 if(BCryptCreateHash(algorithm.h,&state.h,object.data(),ULONG(object.size()),nullptr,0,0)<0||BCryptHashData(state.h,(PUCHAR)data,ULONG(size),0)<0||BCryptFinishHash(state.h,out.data(),ULONG(out.size()),0)<0)throw std::runtime_error("blood_source_sha256");
 return out;
}
// Private interface is process-local: only this loader sets this GUID.
inline const GUID& sourceGuid(){static const GUID v={0x052e52eb,0x316e,0x4788,{0xa9,0x74,0x6c,0x11,0x9a,0xb7,0x53,0x06}};return v;}
struct Source final : IUnknown {
 std::atomic<ULONG> refs{1};std::string path;Digest digest{};std::size_t byteCount=0;
 D3DX11_IMAGE_INFO info{};D3DX11_IMAGE_LOAD_INFO load{};
 HRESULT STDMETHODCALLTYPE QueryInterface(REFIID iid,void** out) override{
  if(!out)return E_POINTER;*out=nullptr;if(iid==__uuidof(IUnknown)){*out=static_cast<IUnknown*>(this);AddRef();return S_OK;}return E_NOINTERFACE;
 }
 ULONG STDMETHODCALLTYPE AddRef() override{return refs.fetch_add(1,std::memory_order_relaxed)+1;}
 ULONG STDMETHODCALLTYPE Release() override{const ULONG left=refs.fetch_sub(1,std::memory_order_acq_rel)-1;if(!left)delete this;return left;}
};
inline void remember(ID3D11Resource* resource,const char* resolvedPath,const void* bytes,std::size_t size,const D3DX11_IMAGE_INFO& info,const D3DX11_IMAGE_LOAD_INFO& load){
 if(!resource)return;Com<Source> source;source.p=new Source;source.p->path=resolvedPath;source.p->digest=hash(bytes,size);source.p->byteCount=size;source.p->info=info;source.p->load=load;source.p->load.pSrcInfo=nullptr;
 require(resource->SetPrivateDataInterface(sourceGuid(),source.p),"blood_source_attach");
}
inline void propagate(ID3D11Resource* from,ID3D11Resource* to){
 if(!from||!to)return;Com<Source> source;UINT bytes=sizeof(source.p);
 if(SUCCEEDED(from->GetPrivateData(sourceGuid(),&bytes,&source.p))&&source.p)require(to->SetPrivateDataInterface(sourceGuid(),source.p),"blood_source_propagate");
}
struct Snapshot {
 Com<ID3D11Device> device;Com<Source> source;
 D3D11_TEXTURE2D_DESC texture{};D3D11_SHADER_RESOURCE_VIEW_DESC view{};
 std::string name;bool supported=false;
};
using SnapshotPtr=std::shared_ptr<const Snapshot>;
struct SnapshotSlot { mutable std::mutex mutex;SnapshotPtr value; };
// Called at existing loader/publication boundaries, never by selected().
inline void publish(SnapshotSlot* slot,ID3D11Resource* resource,ID3D11ShaderResourceView* view,const char* name){
 SnapshotPtr value;
 if(resource){
  auto next=std::make_shared<Snapshot>();next->name=name?name:"";resource->GetDevice(&next->device.p);
  Com<ID3D11Texture2D> texture;
  if(SUCCEEDED(resource->QueryInterface(__uuidof(ID3D11Texture2D),reinterpret_cast<void**>(&texture.p)))){
   texture.p->GetDesc(&next->texture);
   if(view)view->GetDesc(&next->view);
   else {next->view.Format=next->texture.Format;next->view.ViewDimension=D3D11_SRV_DIMENSION_TEXTURE2D;next->view.Texture2D.MostDetailedMip=0;next->view.Texture2D.MipLevels=next->texture.MipLevels;}
   UINT size=sizeof(next->source.p);
   next->supported=SUCCEEDED(resource->GetPrivateData(sourceGuid(),&size,&next->source.p))&&next->source.p&&next->texture.ArraySize==1&&next->texture.SampleDesc.Count==1&&next->view.ViewDimension==D3D11_SRV_DIMENSION_TEXTURE2D;
  }
  value=std::move(next);
 }
 std::lock_guard<std::mutex> lock(slot->mutex);slot->value=std::move(value);
}
inline SnapshotPtr capture(const SnapshotSlot* slot){std::unique_lock<std::mutex> lock(slot->mutex,std::try_to_lock);return lock.owns_lock()?slot->value:SnapshotPtr{};}
inline void clear(SnapshotSlot* slot){std::lock_guard<std::mutex> lock(slot->mutex);slot->value.reset();}
} // namespace PainterBloodTextureSource
#endif
