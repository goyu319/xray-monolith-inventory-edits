#pragma once
#ifdef USE_DX11
#include "BloodCellLayer.h"
#include <atomic>
#include <memory>
#include <vector>
namespace PainterBloodProjection {
using Triangle=std::array<BloodCellLayer::Vertex,3>;
struct Input {
    std::shared_ptr<const CSkeletonWallmark::ProjectionSnapshot> projection;
    CSkeletonWallmark::WMFacesVec faces;
    Fvector contact{};
    Input* next=nullptr;
};
// Projection sources may own engine texture handles. Their final destruction
// stays on the owner even when a body despawns while its bake is running.
inline std::atomic<Input*>& retired(){static auto* value=new std::atomic<Input*>{nullptr};return *value;}
inline void retire(Input* p){auto* head=retired().load(std::memory_order_relaxed);do{p->next=head;}while(!retired().compare_exchange_weak(head,p,std::memory_order_release,std::memory_order_relaxed));}
inline void drainOne(){auto* p=retired().load(std::memory_order_acquire);while(p&&!retired().compare_exchange_weak(p,p->next,std::memory_order_acq_rel,std::memory_order_acquire)){}if(p)delete p;}
inline void drain(){auto* p=retired().exchange(nullptr,std::memory_order_acquire);while(p){auto* next=p->next;delete p;p=next;}}
inline std::shared_ptr<Input> take(CSkeletonWallmark& mark){
    std::shared_ptr<Input> result(new Input,retire);
    result->projection=mark.m_Projection;result->faces.swap(mark.m_Faces);result->contact=mark.ContactPoint();return result;
}
// Same immutable hit-pose projection/predicates as ResolveProjection. This
// worker path reads no parent, live bone, shared_str, shader or GPU context.
inline std::vector<Triangle> resolve(const Input& input){
    std::vector<Triangle> out;
    if(!input.projection){out.reserve(input.faces.size());for(const auto& face:input.faces){Triangle tri;
        for(unsigned k=0;k<3;++k)tri[k]={{face.vert[k].x,face.vert[k].y,face.vert[k].z},{face.uv[k].x,face.uv[k].y}};
        out.push_back(tri);}return out;}
    const auto& hit=*input.projection;
    for(const auto& range:hit.ranges){const auto& source=*range.source;
        if(std::size_t(range.first_vertex)+range.vertex_count>source.indices.size()||range.vertex_count%3)throw std::runtime_error("blood_projection_range");
        const auto* indices=source.indices.data()+range.first_vertex;
        for(u32 i=0;i<range.vertex_count;i+=3){Fvector p[3];
            for(u32 k=0;k<3;++k){if(indices[i+k]>=source.vertices.size())throw std::runtime_error("blood_projection_index");
                const auto& v=source.vertices[indices[i+k]];if(v.job<1||v.job>4)throw std::runtime_error("blood_projection_weights");
                const Fvector original={v.p[0],v.p[1],v.p[2]};Fvector pb[4];
                for(u32 j=0;j<v.job;++j){if(v.ids[j]>=hit.hit_pose.size())throw std::runtime_error("blood_projection_pose");hit.hit_pose[v.ids[j]].transform_tiny(pb[j],original);}
                if(v.job==1)p[k]=pb[0];else if(v.job==2)p[k].lerp(pb[0],pb[1],v.weights[0]);
                else{float remainder=1.f-v.weights[0]-v.weights[1];if(v.job==4)remainder-=v.weights[2];
                    for(u32 j=0;j+1<v.job;++j)pb[j].mul(v.weights[j]);pb[v.job-1].mul(remainder);p[k]=pb[0];for(u32 j=1;j<v.job;++j)p[k].add(pb[j]);}
            }
            Fvector normal;normal.mknormal(p[0],p[1],p[2]);
            if(normal.dotproduct(hit.normal)<EPS||!BloodProjectionFootprint::Outer(input.contact,hit.normal,hit.size,p,hit.outer_depth)||
               !(hit.square_footprint?BloodProjectionFootprint::Intersects(hit.view,p):CDB::TestSphereTri(input.contact,hit.size,p)))continue;
            Triangle tri;for(u32 k=0;k<3;++k){const auto& v=source.vertices[indices[i+k]];Fvector uv;hit.view.transform_tiny(uv,p[k]);
                tri[k]={{v.p[0],v.p[1],v.p[2]},{(1.f+uv.x)*.5f,(1.f-uv.y)*.5f}};}out.push_back(tri);
        }
    }return out;
}
}
#endif
