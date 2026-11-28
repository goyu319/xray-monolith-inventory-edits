#pragma once
#include "PainterBurnSessions.h"
namespace painter_burn {
struct WallWitness{std::array<float,3> rest{},normal{};U32 label{},token{};unsigned domain=Anatomy;};
struct SourceFaces{
    std::vector<std::vector<U32>> byChild;
    std::vector<std::uint8_t> roles;
    std::vector<WallWitness> walls;
    // A new cut revision can replace only the wall witnesses while retaining
    // the same authenticated face topology and material roles.
    std::shared_ptr<const SourceFaces> authored;
};
inline std::vector<Texel> sourceTable(const SourceFaces& geometry,U32 child,const std::vector<U32>* remap,U32& count){
    count=0;if(child>=geometry.byChild.size())return {};
    const auto& base=geometry.byChild[child];
    count=U32(remap?remap->size():base.size());
    std::vector<Texel> tex((count+255u)&~255u);
    for(unsigned i=0;i<count;++i){
        const unsigned local=remap?(*remap)[i]:i;if(local>=base.size())continue;
        const U32 token=base[local];if(!token||token>262143u)continue;
        tex[i]={token,token<=geometry.roles.size()?unsigned(geometry.roles[token-1]):0u};
    }
    return tex;
}
inline Key wallKey(const WallWitness& q){
    const auto& v=q.normal;unsigned i=0;
    if(std::abs(v[1])>std::abs(v[i]))i=1;if(std::abs(v[2])>std::abs(v[i]))i=2;
    return {int(std::floor(q.rest[0]*1000.f)),int(std::floor(q.rest[1]*1000.f)),int(std::floor(q.rest[2]*1000.f)),
            1u+8u*(q.label&127u)+2u*i+unsigned(v[i]<0.f)};
}
inline bool validWall(const WallWitness& q){
    if(!(q.label&127u)||((q.label&127u)==16u?q.domain!=Backing:q.domain!=Anatomy)||q.token<262144||q.token>MaxSurface)return false;
    for(float n:q.rest)if(!std::isfinite(n)||std::abs(n)>1000000.f)return false;
    float n2=0;for(float a:q.normal)n2+=a*a;return std::isfinite(n2)&&n2>1.e-12f;
}
inline U32 wallWitness(const WallWitness& q){return makeWitness(q.rest[0],q.rest[1],q.rest[2],q.normal[0],q.normal[1],q.normal[2]);}
inline U32 interiorSurfaceToken(const SourceFaces& src,const std::array<float,3>& p,const std::array<float,3>& n,U32 label){
    WallWitness query{p,n,label,262144,(label&127u)==16u?unsigned(Backing):unsigned(Anatomy)};if(!validWall(query))return 0;
    const auto key=wallKey(query);U32 result=0,w=0;
    for(const auto& q:src.walls)if(validWall(q)&&wallKey(q)==key){
        auto sw=wallWitness(q);if(result&&(result!=q.token||sw!=w))return 0;result=q.token;w=sw;
    }
    const float len=std::sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]);
    return result&&matchesWitness(w,p[0],p[1],p[2],n[0]/len,n[1]/len,n[2]/len)?result:0;
}
inline std::vector<Texel> wallTable(const std::vector<WallWitness>& walls,unsigned& mask){
    mask=0;if(walls.empty()||walls.size()>65536)return {};
    unsigned slots=2;while(slots<2*walls.size())slots*=2;
    std::vector<Texel> out((2*slots+255u)&~255u);mask=slots-1;
    for(const auto& q:walls)if(validWall(q)){
        auto k=wallKey(q);bool placed=false;
        for(unsigned n=0;n<64;++n){
            auto i=2*((hash(k)+n)&mask);auto& a=out[i];auto& b=out[i+1];auto w=wallWitness(q);
            if(!a.w){a={U32(k.x),U32(k.y),U32(k.z),k.tag};b={q.token,w};placed=true;break;}
            if(a.x==U32(k.x)&&a.y==U32(k.y)&&a.z==U32(k.z)&&a.w==k.tag){
                if(b.x!=q.token||b.y!=w)b.x=0;placed=true;break;
            }
        }
        if(!placed){mask=0;return {};}
    }
    return out;
}
}
