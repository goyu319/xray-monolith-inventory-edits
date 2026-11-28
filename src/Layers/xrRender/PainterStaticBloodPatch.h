#pragma once
#include "PainterBloodTextureCodec.h"
#include <array>
#include <cstdint>
#include <memory>
#include <vector>
namespace PainterStaticBlood {
using Image=PainterBloodTextureCodec::Image;
using Id=std::uint64_t;
struct V3 {float x=0,y=0,z=0;};
struct Vertex {V3 p; std::uint32_t color=0xffffffffu; float u=0,v=0;};
struct Receiver {V3 p[3];};
struct Stamp {
    std::uint32_t landedMs=0,lifetimeMs=~0u,birthMs=0,deathMs=0;
    std::uint32_t flags=1; float opacity=1; std::uint32_t pad0=0,pad1=0;
};
static_assert(sizeof(Vertex)==24 && sizeof(Stamp)==32,"static wallmark draw format");
struct Mark {
    Id id=0,sector=0,material=0;
    Stamp stamp;
    std::shared_ptr<const Image> image;
    std::vector<Vertex> clipped;
    std::vector<Receiver> receivers;
};
struct Config {
    unsigned minimum=8, maximum=64, maxDimension=2048;
    std::size_t byteCap=32u*1024u*1024u;
    float nearDistance=1.f, planeTolerance=.006f, normalCos=.9986f;
};
struct Frame {V3 origin,u,v,n;double left=0,bottom=0,pitch=0;};
struct Rect {unsigned x=0,y=0,w=0,h=0;};
struct Patch {
    Id sector=0,material=0;
    unsigned width=0,height=0;
    Frame frame;
    std::vector<Vertex> mesh;
    std::vector<Receiver> receivers;
    std::vector<std::uint32_t> rgba;
    std::vector<std::uint16_t> owner;
    std::vector<Id> ids;
    std::vector<Stamp> stamps; // 0 is empty
    std::size_t textureBytes()const{return std::size_t(width)*height*6+512*sizeof(Stamp)+mesh.size()*sizeof(Vertex);}
};
struct Delta {
    Rect rect;
    std::vector<std::uint32_t> rgba;
    std::vector<std::uint16_t> owner;
    unsigned stampIndex=0;
    Stamp stamp;
    Id id=0;
};
bool build(const std::vector<const Mark*>& pending,std::uint32_t nowMs,const Config& config,Patch& out);
bool prepareAppend(const Patch& patch,const Mark& mark,std::uint32_t nowMs,const Config& config,Delta& out);
bool commit(Patch& patch,const Delta& delta);
}
#include "PainterStaticBloodPatch.inl"
