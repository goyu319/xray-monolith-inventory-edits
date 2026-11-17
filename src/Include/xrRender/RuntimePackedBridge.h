#pragma once
// Include after the engine's RuntimeSkinGeometry.h. This is the actual in-memory
// worker-output conversion; it never opens OGF/model/files or touches the rig.
#include "RuntimePackedMesh.h"
inline bool AppendPackedRuntimeChildren(const gfy_packed::Mesh& packed,
    RuntimeSkinUpdate& update,LPCSTR cavity_shader,LPCSTR cavity_texture,
    const xr_vector<shared_str>* tissue_textures=nullptr)
{
    static_assert(sizeof(RuntimeSkinVertex)==80,"Native request ABI changed");
    // Required resources are supported only by the atomic staged adapter.
    // Refuse before unpacking; caller rebuilds this same resolved candidate.
    if(packed.analytic_channels||packed.skin_ownership)return false;
    for(const auto&c:packed.children)if(c.skin_clip||c.skin_owned)return false;
    if(tissue_textures)for(const auto& src:packed.children)
        if(src.material&&(src.material>=tissue_textures->size()||!(*tissue_textures)[src.material].size()))return false;
    update.replaced_children.clear();
    for(const auto& src:packed.children)
    {
        RuntimeSkinChild dst;
        if(src.material)dst.tissue_field=packed.tissue_field;
        dst.child_index=src.child_index;dst.material_source_child=src.material_source_child;
        dst.remove_surface=src.vertices.empty() && src.indices.empty();
        if(dst.remove_surface)dst.receives_wallmarks=false;
        if(src.material){dst.shader=cavity_shader;dst.texture=tissue_textures?(*tissue_textures)[src.material]:shared_str(cavity_texture);dst.receives_wallmarks=false;}
        dst.vertices.resize(src.vertices.size());
        dst.indices.assign(src.indices.begin(),src.indices.end());
        for(size_t i=0;i<src.vertices.size();++i)
        {
            const auto& v=src.vertices[i];auto& out=dst.vertices[i];
            out.position.set(v.surface[0],v.surface[1],v.surface[2]);
            out.normal.set(v.surface[3],v.surface[4],v.surface[5]);
            out.tangent.set(v.surface[6],v.surface[7],v.surface[8]);
            out.binormal.set(v.surface[9],v.surface[10],v.surface[11]);
            out.uv.set(v.surface[12],v.surface[13]);
            for(unsigned k=0;k<4;++k){out.bone[k]=v.bone[k];out.weight[k]=v.weight[k];}
        }
        update.replaced_children.push_back(std::move(dst));
    }
    return true;
}
