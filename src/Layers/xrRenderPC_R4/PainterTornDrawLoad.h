#pragma once
#include "PainterPortableRecord.h"
#include "../xrRender/PainterTornDrawPair.h"

namespace painter_torn_optional {
static constexpr char recordPath[] = "r3\\painter_wound_cache\\torn_draw\\ldr1.bin";
static constexpr size_t localBytes = 739812, bakedBytes = 703824;

struct Record {
    IReader* file;
    ~Record() { if(file) FS.r_close(file); }
};

// Called during load after the ordinary renderer has created and reflected the original PS.
inline bool attach(ID3D11PixelShader* ordinary,const void* bytecode,size_t size,
                   const char* name,const char* entry,const char* target,UINT flags,
                   const void* root,size_t rootSize,const D3D_SHADER_MACRO* defs,ID3DInclude& includes,unsigned samples) {
    Record record{FS.r_open("$game_shaders$",recordPath)};
    if(!ordinary || !record.file || record.file->length()!=8+localBytes+bakedBytes)return false;
    const auto* bytes=static_cast<const unsigned char*>(static_cast<const void*>(record.file->pointer()));
    if(memcmp(bytes,"PTDRAW01",8))return false;
    // Authenticate both inputs before handing the optional bytecode to the driver.
    static constexpr painter_portable_shader::Digest local={0x48,0x83,0x12,0xa5,0xa3,0xf1,0x7a,0x4d,0x93,0xd6,0x43,0x62,0xc8,0xfb,0x6a,0xad,0x3b,0x67,0x09,0xf6,0xf0,0x4f,0x62,0x93,0xfb,0x1d,0x9c,0x72,0x5d,0x34,0x0d,0xb9},baked={0x73,0x26,0x17,0x2c,0x5e,0x9a,0x35,0xe6,0x45,0xa5,0x33,0xc7,0xd0,0x6b,0x78,0xc3,0xdf,0xb0,0xdf,0xc2,0x82,0xa3,0x93,0x13,0x01,0x76,0x69,0x08,0x03,0x14,0x29,0x95};
    const auto* localCode=bytes+8;
    const auto* bakedCode=localCode+localBytes;
    if(!painter_portable_shader::payload(localCode,localBytes,local) ||
       !painter_portable_shader::payload(bakedCode,bakedBytes,baked))return false;
    painter_portable_shader::Digest active{};
    if(!painter_portable_shader::key(name,entry,target,flags,root,rootSize,defs,includes,active))return false;
    const auto key=painter_portable_shader::hex(active);
    if(samples==1u && key!="0ec2825b9bfaaa6c5dfc43c625ccc45313aeaf99999e32980b2dbef03841bdf1")return false;
    if(samples==2u && key!="0a05cbd7f481169e24f8b32567cea46e05c6c0a33ff51c1762132f0b8d757c1e")return false;
    if(samples==4u && key!="c1de2fdd23ca30b50d5cf6eb5af428b2f17af3ddb50c4ac65e687339e43a8f6d")return false;
    if(!painter_torn_draw::certifiedPrimary(bytecode,size,samples))return false;
    Microsoft::WRL::ComPtr<ID3D11Device> device;
    ordinary->GetDevice(device.GetAddressOf());
    Microsoft::WRL::ComPtr<ID3D11PixelShader> localPS,bakedPS;
    if(!device || FAILED(device->CreatePixelShader(localCode,localBytes,nullptr,localPS.GetAddressOf())) ||
       FAILED(device->CreatePixelShader(bakedCode,bakedBytes,nullptr,bakedPS.GetAddressOf())))return false;
    return painter_torn_draw::publish(ordinary,localPS.Get(),bakedPS.Get(),bytecode,size,localCode,localBytes,bakedCode,bakedBytes,samples);
}
}
