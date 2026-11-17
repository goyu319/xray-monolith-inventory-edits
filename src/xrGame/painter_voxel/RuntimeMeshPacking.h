#pragma once
// Include after the enhanced joined-kernel types. Pure CPU conversion; no I/O.
#include "RuntimePackedMesh.h"
namespace packing {
struct PackingInfluenceError : std::runtime_error { using std::runtime_error::runtime_error; };
inline void packingRequire(bool value,const char* message){if(!value)throw PackingInfluenceError(message);}
inline gfy_packed::Vertex vertex(const JoinedVertex& v,double* discarded=nullptr){
    gfy_packed::Vertex out;
    const Vec vectors[4]={v.p,v.normal,v.tangent,v.binormal};
    for(unsigned i=0;i<4;++i)for(unsigned k=0;k<3;++k)out.surface[i*3+k]=float(vectors[i][int(k)]);
    out.surface[12]=float(v.uv[0]);out.surface[13]=float(v.uv[1]);
    // VXHEAD02 original corners preserve original slots and effective UNORM8
    // coefficients, including positive rows and repeated bone IDs.
    if(v.hardware_corner){const auto full=gfy_signed_hw::full(v.hardware_ids,v.hardware_w);
        packingRequire(gfy_signed_hw::valid4(v.hardware_w,true),"Authored hardware layout");
        for(unsigned b=0;b<64;++b)packingRequire(std::abs(full[b]-v.w[b])<1e-8,"Authored hardware corner weights changed");
        out.bone=v.hardware_ids;out.weight=v.hardware_w;for(float value:out.surface)packingRequire(std::isfinite(value),"Nonfinite surface");if(discarded)*discarded=0;return out;}
    // Native 4W stores only the first three weights. Put the largest actual
    // influence in slot 3, giving reconstruction at least a 1/4 margin instead
    // of reconstructing a padded zero from rounded floats. Bone IDs still refer
    // to the strongest influences. Generated vertices may blend more than
    // four source bones; deterministically reduce those vertices and normalize
    // the retained weights. Shared seam vertices start with the same weight
    // field and therefore reduce identically on skin and cavity materials.
    unsigned influences=0,largest=0;double sum=0;
    std::array<unsigned,4> bones{};std::array<double,4> weights{};
    std::array<unsigned,64> all{};unsigned count=0;double total=0;bool signedSkin=false;
    for(unsigned k=0;k<64;++k){packingRequire(std::isfinite(v.w[k]),"Invalid skin influence");signedSkin|=v.w[k]<-1e-12;if(v.w[k]>1e-12){all[count++]=k;total+=v.w[k];}}
    if(signedSkin){const auto error=gfy_signed_hw::pack(v.w,out.bone,out.weight);if(discarded)*discarded=error.positive_discarded+error.negative_discarded;for(float value:out.surface)packingRequire(std::isfinite(value),"Nonfinite surface");return out;}
    packingRequire(count&&std::abs(total-1)<1e-6,"Skin sum");
    if(count>4){std::sort(all.begin(),all.begin()+count,[&](unsigned a,unsigned b){return v.w[a]==v.w[b]?a<b:v.w[a]>v.w[b];});count=4;std::sort(all.begin(),all.begin()+count);}
    for(unsigned at=0;at<count;++at){unsigned k=all[at];
        bones[influences]=k;weights[influences]=v.w[k];
        if(weights[influences]>weights[largest])largest=influences;
        ++influences;sum+=v.w[k];}
    packingRequire(influences && sum>0,"Skin retained sum");
    if(discarded)*discarded=std::max(0.,(total-sum)/total);
    for(unsigned k=0;k<4;++k)out.bone[k]=std::uint16_t(bones[largest]);
    unsigned next=0;
    for(unsigned k=0;k<influences;++k)if(k!=largest){out.bone[next]=std::uint16_t(bones[k]);out.weight[next]=float(weights[k]/sum);++next;}
    out.weight[3]=1.f-out.weight[0]-out.weight[1]-out.weight[2];
    float shader_sum=out.weight[0]+out.weight[1]+out.weight[2];
    int hardware_sum=0;for(unsigned k=0;k<3;++k)hardware_sum+=int(std::floor(out.weight[k]*255.f+.5f));
    packingRequire(out.weight[3]>=0.f && shader_sum<=1.f && hardware_sum<=255,"Native 4W reconstruction");
    for(float w:out.weight)packingRequire(std::isfinite(w) && w>=0.f && w<=1.f,"Native skin range");
    for(float value:out.surface)packingRequire(std::isfinite(value),"Nonfinite surface");
    return out;
}
using Key=std::array<std::uint8_t,80>;
inline Key key(const gfy_packed::Vertex& v){Key k;std::memcpy(k.data(),&v,sizeof(v));return k;}
// Packed vertex identity is exact byte equality of the 80-byte ABI row, as it
// was with std::map<Key>: +0/-0 and distinct float bit patterns stay distinct,
// and vertex() has already rejected nonfinite values. Row indices are assigned
// in first-encounter order, so the hash affects speed only, never output bytes.
static_assert(sizeof(gfy_packed::Vertex)==sizeof(Key)&&sizeof(Key)%8==0,"Packed vertex key is the padding-free ABI row");
inline std::uint64_t vertexHash(const gfy_packed::Vertex& v){
    const auto* bytes=reinterpret_cast<const unsigned char*>(&v);std::uint64_t h=0x243F6A8885A308D3ull;
    for(unsigned at=0;at<sizeof(Key);at+=8){std::uint64_t word;std::memcpy(&word,bytes+at,8);h=(h^word)*0x9E3779B97F4A7C15ull;h^=h>>29;}
    return h^(h>>32);
}
// Open-addressed index over the rows already emitted for one child. Slots hold
// row+1 (0 = empty); rows are compared in place, so no key copies are stored.
// reset(n) promises at most n rows before the next reset/clear, and the table is
// kept at load <= 1/2. Callers cap n at 65535 rows, bounding it at 512 KiB.
class VertexIndex {
    std::vector<std::uint32_t> slots;std::size_t mask{};
public:
    void reset(std::size_t rows){std::size_t n=16;while(n<2*rows)n*=2;slots.assign(n,0);mask=n-1;}
    void clear(){std::fill(slots.begin(),slots.end(),0u);}
    // Row index of v, or -1 when absent.
    long find(const std::vector<gfy_packed::Vertex>& rows,const gfy_packed::Vertex& v,std::uint64_t hash)const{
        for(std::size_t at=std::size_t(hash)&mask;;at=(at+1)&mask){const auto slot=slots[at];
            if(!slot)return -1;if(!std::memcmp(&rows[slot-1],&v,sizeof(v)))return long(slot-1);}
    }
    // Registers rows that are already unique and in first-encounter order (a child this index built), without appending:
    // the slots end exactly as if the rows had been inserted one by one.
    void seed(const std::vector<gfy_packed::Vertex>& rows){for(std::size_t r=0;r<rows.size();++r){const auto h=vertexHash(rows[r]);
        for(std::size_t at=std::size_t(h)&mask;;at=(at+1)&mask)if(!slots[at]){slots[at]=std::uint32_t(r+1);break;}}}
    // Existing row index of v, or appends v to rows. Second member is true when appended.
    std::pair<std::uint16_t,bool> insert(std::vector<gfy_packed::Vertex>& rows,const gfy_packed::Vertex& v,std::uint64_t hash){
        for(std::size_t at=std::size_t(hash)&mask;;at=(at+1)&mask){const auto slot=slots[at];
            if(!slot){slots[at]=std::uint32_t(rows.size()+1);rows.push_back(v);return {std::uint16_t(rows.size()-1),true};}
            if(!std::memcmp(&rows[slot-1],&v,sizeof(v)))return {std::uint16_t(slot-1),false};}
    }
};
inline gfy_packed::Mesh packMulti(const JoinResult& result,
    const std::vector<std::uint16_t>& faceChildren,const std::vector<std::uint32_t>& faceIndices,
    std::uint16_t originalChildren){
    packingRequire(faceChildren.size()==faceIndices.size()&&!faceChildren.empty(),"Source child map");
    packingRequire(originalChildren>0&&originalChildren<gfy_packed::max_child_slots,"Original child slots");
    using Group=std::pair<unsigned,std::uint16_t>;
    std::map<Group,std::vector<const JoinedTriangle*>> groups;
    for(auto child:faceChildren){packingRequire(child<originalChildren,"Source child outside original");groups[{0,child}];}
    const auto fallback=faceChildren.front();
    for(const auto& triangle:result.triangles){
        std::uint16_t source=fallback;
        if(!triangle.material){const auto face=triangle.v[0].sourceFace;
            packingRequire(face<faceChildren.size()&&triangle.v[1].sourceFace==face&&triangle.v[2].sourceFace==face,"Exterior source face");
            source=faceChildren[face];
        }
        packingRequire(triangle.material<64,"Interior material slot limit");groups[{triangle.material,source}].push_back(&triangle);
    }
    gfy_packed::Mesh out;unsigned next=originalChildren;
    auto appendSlot=[&]{packingRequire(next<gfy_packed::max_child_slots,"Runtime child slots exhausted");return std::uint16_t(next++);};
    VertexIndex vertices;
    for(const auto& group:groups){
        const auto& triangles=group.second;
        gfy_packed::Child child;child.material=group.first.first;child.material_source_child=group.first.second;
        child.child_index=child.material?appendSlot():group.first.second;
        // A child never holds more than 65535 rows or triangles; indices and
        // faces are reserved for the triangles that can still land in it.
        vertices.reset(std::min<std::size_t>(3*triangles.size(),65535));
        auto reserve=[&](std::size_t remaining){remaining=std::min<std::size_t>(remaining,65535);child.indices.reserve(3*remaining);child.original_faces.reserve(remaining);};
        reserve(triangles.size());
        auto flush=[&]{packingRequire(child.child_index<gfy_packed::max_child_slots,"Runtime child slots exhausted");out.children.push_back(std::move(child));};
        for(std::size_t t=0;t<triangles.size();++t){const auto* triangle=triangles[t];
            std::array<gfy_packed::Vertex,3> rows;std::array<std::uint64_t,3> hashes;std::array<long,3> found;std::array<double,3> discarded{};unsigned fresh=0;
            // Like the former count() pass, a row repeated within one triangle
            // counts as fresh each time; the split decision depends on it.
            for(unsigned k=0;k<3;++k){rows[k]=vertex(triangle->v[k],&discarded[k]);hashes[k]=vertexHash(rows[k]);found[k]=vertices.find(child.vertices,rows[k],hashes[k]);if(found[k]<0)++fresh;}
            if(child.vertices.size()+fresh>65535||child.indices.size()/3>=65535){flush();child={};child.material=group.first.first;child.material_source_child=group.first.second;child.child_index=appendSlot();vertices.clear();found.fill(-1);reserve(triangles.size()-t);}
            for(unsigned k=0;k<3;++k){
                if(found[k]>=0){child.indices.push_back(std::uint16_t(found[k]));continue;}
                const auto [index,inserted]=vertices.insert(child.vertices,rows[k],hashes[k]);
                if(inserted){out.reduced_skin_vertices+=discarded[k]>1e-12;out.max_discarded_skin_weight=std::max(out.max_discarded_skin_weight,discarded[k]);}
                child.indices.push_back(index);
            }
            child.original_faces.push_back(child.material?~std::uint32_t(0):faceIndices[triangle->v[0].sourceFace]);
        }
        // An empty exterior slot explicitly removes that render child. It does
        // not hide a bone or alter live animation/physics hierarchy.
        flush();
    }
    return out;
}
inline gfy_packed::Mesh pack(const JoinResult& result,std::uint16_t source_child,std::uint16_t original_children){
    gfy_packed::Mesh out;std::uint16_t next_append=original_children;
    bool exterior_remains=false;std::array<std::size_t,2> counts{};
    for(const auto& triangle:result.triangles){packingRequire(triangle.material<=1,"Material not supported by this first packing gate");exterior_remains=exterior_remains||triangle.material==0;++counts[triangle.material];}
    packingRequire(exterior_remains,"Fully removed source child requires an explicit empty-child renderer contract");
    VertexIndex vertices;
    for(unsigned material=0;material<2;++material){gfy_packed::Child child;child.material=material;child.material_source_child=source_child;
        child.child_index=material?next_append++:source_child;vertices.reset(std::min<std::size_t>(3*counts[material],65535));
        auto flush=[&]{if(child.indices.empty())return;out.children.push_back(std::move(child));child={};child.material=material;child.material_source_child=source_child;child.child_index=next_append++;vertices.clear();};
        for(auto& triangle:result.triangles){if(triangle.material!=material)continue;
            std::array<gfy_packed::Vertex,3> rows;std::array<std::uint64_t,3> hashes;std::array<long,3> found;unsigned fresh=0;
            for(unsigned k=0;k<3;++k){rows[k]=vertex(triangle.v[k]);hashes[k]=vertexHash(rows[k]);found[k]=vertices.find(child.vertices,rows[k],hashes[k]);if(found[k]<0)++fresh;}
            if(child.vertices.size()+fresh>65535 || child.indices.size()/3>=65535){flush();found.fill(-1);}
            for(unsigned k=0;k<3;++k)child.indices.push_back(found[k]>=0?std::uint16_t(found[k]):vertices.insert(child.vertices,rows[k],hashes[k]).first);
            auto face=triangle.v[0].sourceFace;
            packingRequire(material || (triangle.v[1].sourceFace==face && triangle.v[2].sourceFace==face),"Exterior mixed source face");
            child.original_faces.push_back(material?~std::uint32_t(0):face);
        }
        // Do not reserve a phantom appended slot between material streams.
        if(!child.indices.empty())out.children.push_back(std::move(child));
    }
    return out;
}
}
