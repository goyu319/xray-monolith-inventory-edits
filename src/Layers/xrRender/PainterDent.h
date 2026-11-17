#pragma once
// melee (2026-11-12, painter_voxel_mask_only 3): the C++ twin of painter_dent_offset (W/melee/src/painter_dent_core.hlsl; the exterior's
// dent vertex programs and the wallmark skinning kernels carry the HLSL text) - the rest-space displacement of a point by a body's dents,
// for the blood decals skinned on the CPU (CKinematics::RenderWallmarkPrepared and the masked CPU decal path in WallmarksEngine.cpp),
// so a decal on a dented area lies on the dented skin. texels: 8 floats per dent (centre xyz, radius, push xyz, depth), rest space.
#include <cmath>
#include <cstddef>
namespace painter_dent {
inline void apply(const float* texels,std::size_t dents,float& x,float& y,float& z){
    float ox=0,oy=0,oz=0;
    for(std::size_t i=0;i<dents;++i){const float* a=texels+8*i;const float* q=a+4;
        const float dx=x-a[0],dy=y-a[1],dz=z-a[2],r=a[3],d2=dx*dx+dy*dy+dz*dz;
        if(!(r>0)||d2>=r*r)continue;
        const float x2=d2/(r*r),f=(1-x2)*(1-x2);
        float g=1-(dx*q[0]+dy*q[1]+dz*q[2])/(0.55f*r);g=g<0?0:(g>1?1:g);
        const float s=q[3]*f*g;ox+=q[0]*s;oy+=q[1]*s;oz+=q[2]*s;}
    x+=ox;y+=oy;z+=oz;}
// a key of a dent list (FNV-1a of its bytes, never 0): a decal re-skins / re-projects when its body's key changes
inline unsigned long long key(const float* texels,std::size_t dents){unsigned long long h=1469598103934665603ull;
    const unsigned char* b=reinterpret_cast<const unsigned char*>(texels);
    for(std::size_t i=0;i<dents*8*sizeof(float);++i){h^=b[i];h*=1099511628211ull;}return h|1ull;}
}
