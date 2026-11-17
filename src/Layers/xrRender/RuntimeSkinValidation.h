#pragma once
// Same pure CPU admission function is exercised by the packed-stream probe.
// Include after RuntimeSkinGeometry.h; no GPU allocation or owner mutation here.
#include <cmath>
#include "../../Include/xrRender/RuntimeSignedSkin.h"
struct RuntimeSkinValidationError {const char* reason="none";size_t vertex=0;unsigned influence=0;};
inline bool ValidateRuntimeSkinChild(const RuntimeSkinChild& data,unsigned bone_count,unsigned registers,RuntimeSkinValidationError& error)
{
    auto reject=[&](const char* why){error.reason=why;return false;};
    if(data.vertices.empty() || data.vertices.size()>65535)return reject("vertex_count");
    if(data.indices.empty() || data.indices.size()%3 || data.indices.size()/3>65535)return reject("index_count");
    unsigned max_bone=0;
    for(size_t i=0;i<data.vertices.size();++i){error.vertex=i;const auto& v=data.vertices[i];
        const float values[]={v.position.x,v.position.y,v.position.z,v.normal.x,v.normal.y,v.normal.z,v.tangent.x,v.tangent.y,v.tangent.z,v.binormal.x,v.binormal.y,v.binormal.z,v.uv.x,v.uv.y};
        for(float value:values)if(!std::isfinite(value))return reject("nonfinite_surface");
        const bool signed4=v.weight[3]<0.f&&gfy_signed_hw::valid4(v.weight,true);float sum=0.f;
        for(unsigned k=0;k<4;++k){error.influence=k;
            if(v.bone[k]>=bone_count)return reject("bone_index");
            if(!std::isfinite(v.weight[k]) || (v.weight[k]<0.f&&!signed4) || v.weight[k]>1.f)return reject("weight_range");
            sum+=v.weight[k];if(v.bone[k]>max_bone)max_bone=v.bone[k];
        }
        if(std::fabs(sum-1.f)>1e-5f)return reject("weight_sum");
        const float left=((1.f-v.weight[0])-v.weight[1])-v.weight[2];
        const float grouped=1.f-((v.weight[0]+v.weight[1])+v.weight[2]);
        if((!signed4&&(left<0.f || grouped<0.f)) || std::fabs(left-v.weight[3])>1e-5f)return reject("reconstructed_fourth_weight");
        // vertHW_4W::_Load_hw stores first three weights in alpha UNORM8.
        // Validate their integer residual, not a potentially negative rounded
        // float evaluation of a mathematically zero fourth influence.
        unsigned quantized=0;
        for(unsigned k=0;k<3;++k)quantized+=unsigned(std::floor(v.weight[k]*255.f+.5f));
        if(!signed4&&quantized>255)return reject("gpu_quantized_fourth_weight");
    }
    for(auto index:data.indices)if(index>=data.vertices.size())return reject("index_range");
    // Padded indices count, exactly as the native 4W palette upload does.
    if(registers<25 || max_bone>(registers-25)/3)return reject("hardware_palette");
    error={};return true;
}
