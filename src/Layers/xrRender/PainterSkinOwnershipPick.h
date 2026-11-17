#pragma once
#include "../../Include/xrRender/SkinOwnershipField.h"
// Evaluate at the ray's barycentric point on the original rest triangle.
// The posed hit cannot be inverted through one bone when corners use different palettes.
namespace PainterSkinOwnershipPick {
inline bool accepts(const painter_detail::SkinOwnershipField*field,unsigned child,unsigned primitive,
    float x,float y,float z)noexcept {
    if(!field)return true;
    const auto*part=field->child(child);if(!part)return true;
    if(primitive>=part->count||primitive>=part->data.size())return false;
    const auto&r=part->data[primitive];
    if(r[0]>field->planes.size()||r[1]>field->planes.size()-r[0])return false;
    for(unsigned i=0;i<r[1];++i){const auto&p=field->planes[r[0]+i];
        if(!(p[0]*x+p[1]*y+p[2]*z+p[3]>=0.f))return false;}
    return true;
}
template<class Buffer,class Index>
bool at_barycentric(const painter_detail::SkinOwnershipField*field,unsigned child,unsigned primitive,
    const Buffer&vertices,const Index*indices,float u,float v)noexcept {
    if(!field||!field->child(child))return true;
    const auto&a=vertices[indices[std::size_t(primitive)*3]].P;
    const auto&b=vertices[indices[std::size_t(primitive)*3+1]].P;
    const auto&c=vertices[indices[std::size_t(primitive)*3+2]].P;
    const float w=1.f-u-v;
    return accepts(field,child,primitive,w*a.x+u*b.x+v*c.x,w*a.y+u*b.y+v*c.y,w*a.z+u*b.z+v*c.z);
}
}
