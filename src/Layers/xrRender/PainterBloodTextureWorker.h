#pragma once
#ifdef USE_DX11
#include "PainterBloodTextureSource.h"
#include "PainterBloodGpuDecode.h"
#include <dxgi.h>
#include <algorithm>
#include <thread>
#include <mutex>
#include <memory>
namespace PainterBloodTextureWorker {
using namespace PainterBloodTextureSource;
// One worker owns this context for its entire lifetime. The main immediate
// context is neither passed into this type nor accessed anywhere in this file.
class PreparedDecoder {
 Com<ID3D11Device> device;Com<ID3D11DeviceContext> context;
 Com<ID3D11ComputeShader> program;
 LUID adapterId{};
 std::mutex useContext;
public:
 explicit PreparedDecoder(ID3D11Device* original){
  if(!original)throw std::runtime_error("blood_decode_source_device");
  Com<IDXGIDevice> dxgi;Com<IDXGIAdapter> adapter;
  require(original->QueryInterface(__uuidof(IDXGIDevice),reinterpret_cast<void**>(&dxgi.p)),"blood_decode_dxgi_device");
  require(dxgi.p->GetAdapter(&adapter.p),"blood_decode_adapter");DXGI_ADAPTER_DESC desc{};require(adapter.p->GetDesc(&desc),"blood_decode_adapter_desc");adapterId=desc.AdapterLuid;
  const D3D_FEATURE_LEVEL requested=original->GetFeatureLevel();D3D_FEATURE_LEVEL actual{};
  require(D3D11CreateDevice(adapter.p,D3D_DRIVER_TYPE_UNKNOWN,nullptr,0,&requested,1,D3D11_SDK_VERSION,&device.p,&actual,&context.p),"blood_decode_private_device");
  if(actual!=requested)throw std::runtime_error("blood_decode_feature_level");
  PainterBloodGpuDecode::createProgram(device.p,&program.p);
 }
 bool matches(ID3D11Device* original){
  if(!original || FAILED(device.p->GetDeviceRemovedReason()) || original->GetFeatureLevel()!=device.p->GetFeatureLevel())return false;
  Com<IDXGIDevice> dxgi;Com<IDXGIAdapter> adapter;DXGI_ADAPTER_DESC desc{};
  require(original->QueryInterface(__uuidof(IDXGIDevice),reinterpret_cast<void**>(&dxgi.p)),"blood_decode_dxgi_device");require(dxgi.p->GetAdapter(&adapter.p),"blood_decode_adapter");require(adapter.p->GetDesc(&desc),"blood_decode_adapter_desc");
  return desc.AdapterLuid.HighPart==adapterId.HighPart&&desc.AdapterLuid.LowPart==adapterId.LowPart;
 }
 PainterBloodTextureCodec::Image decode(const Snapshot& snapshot,const std::vector<unsigned char>& bytes){
  std::lock_guard<std::mutex> singleUser(useContext);
  if(!snapshot.supported||!snapshot.source.p)throw std::runtime_error("blood_texture_immutable_dds_required");
  const Source& source=*snapshot.source.p;
  if(bytes.size()!=source.byteCount||hash(bytes.data(),bytes.size())!=source.digest)throw std::runtime_error("blood_texture_source_changed");
  D3DX11_IMAGE_INFO info=source.info;D3DX11_IMAGE_LOAD_INFO load=source.load;load.pSrcInfo=&info;
  // Only residency/bind usage differ. Format/size/mip/filter parameters remain
  // byte-for-byte the actual loader's values (including D3DX defaults).
  load.Usage=D3D11_USAGE_IMMUTABLE;load.BindFlags=D3D11_BIND_SHADER_RESOURCE;load.CpuAccessFlags=0;
  Com<ID3D11Resource> resource;
  require(D3DX11CreateTextureFromMemory(device.p,bytes.data(),bytes.size(),&load,nullptr,&resource.p,nullptr),"blood_texture_worker_load");
  Com<ID3D11Texture2D> texture;require(resource.p->QueryInterface(__uuidof(ID3D11Texture2D),reinterpret_cast<void**>(&texture.p)),"blood_texture_worker_2d");
  D3D11_TEXTURE2D_DESC desc{};texture.p->GetDesc(&desc);const auto& expected=snapshot.texture;
  if(desc.Width!=expected.Width||desc.Height!=expected.Height||desc.MipLevels!=expected.MipLevels||desc.ArraySize!=expected.ArraySize||desc.Format!=expected.Format||desc.SampleDesc.Count!=expected.SampleDesc.Count)
   throw std::runtime_error("blood_texture_worker_descriptor_changed");
  const unsigned mip=snapshot.view.Texture2D.MostDetailedMip;if(mip>=desc.MipLevels)throw std::runtime_error("blood_texture_worker_mip");
  Com<ID3D11ShaderResourceView> view;require(device.p->CreateShaderResourceView(resource.p,&snapshot.view,&view.p),"blood_texture_worker_view");
  return PainterBloodGpuDecode::readWithProgram(device.p,context.p,program.p,view.p,(std::max)(1u,desc.Width>>mip),(std::max)(1u,desc.Height>>mip));
 }
};

enum class Channel { Body, World };

struct DecoderBank {
    std::atomic<unsigned> creations{0};
    std::mutex lock;
    std::shared_ptr<PreparedDecoder> slots[2];
};
inline DecoderBank& bank() {
    // The device owner explicitly drains the bank. No driver calls at DLL unload.
    static auto* value = new DecoderBank();
    return *value;
}

inline std::shared_ptr<PreparedDecoder> prepared(ID3D11Device* device,Channel which) {
    auto& b=bank();
    const auto i=static_cast<unsigned>(which);
    {
        std::lock_guard<std::mutex> hold(b.lock);
        if (b.slots[i] && b.slots[i]->matches(device)) return b.slots[i];
    }
    // Also permits recovery after actual device removal. Normal play consumes
    // the already prepared slot, even after a worker's texture cache is reset.
    auto fresh=std::make_shared<PreparedDecoder>(device);
    b.creations.fetch_add(1,std::memory_order_relaxed);
    std::lock_guard<std::mutex> hold(b.lock);
    if (!b.slots[i] || !b.slots[i]->matches(device)) b.slots[i]=fresh;
    return b.slots[i];
}
inline void preload(ID3D11Device* device) {
    if (!device || device->GetFeatureLevel()<D3D_FEATURE_LEVEL_11_0) return;
    prepared(device,Channel::Body);
    prepared(device,Channel::World);
}
inline void clearPrepared() {
    std::shared_ptr<PreparedDecoder> retired[2];
    auto& b=bank();
    std::lock_guard<std::mutex> hold(b.lock);
    for (unsigned i=0;i<2;++i) retired[i].swap(b.slots[i]);
}

class Decoder {
    std::shared_ptr<PreparedDecoder> state;
    std::thread::id owner=std::this_thread::get_id();
    void onOwner()const {
        if (owner!=std::this_thread::get_id()) throw std::runtime_error("blood_decode_wrong_thread");
    }
public:
    explicit Decoder(ID3D11Device* device,Channel which=Channel::Body):state(prepared(device,which)){}
    bool matches(ID3D11Device* device) {onOwner();return state->matches(device);}
    PainterBloodTextureCodec::Image decode(const Snapshot& snapshot,const std::vector<unsigned char>& bytes) {
        onOwner();
        return state->decode(snapshot,bytes);
    }
};

} // namespace PainterBloodTextureWorker
#endif
