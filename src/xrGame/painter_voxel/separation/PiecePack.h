#pragma once
#ifndef GFY_SEP_PIECE_PACK_INCLUDED
#define GFY_SEP_PIECE_PACK_INCLUDED
// Separation-track P2 S1: core::packing::packMulti (RuntimeMeshPacking.h) over
// triangle REFERENCES whose packed rows were converted once, in parallel,
// before assembly. Included by BoundSurfacePartition.h inside the kernel
// translation unit (needs gfy_live::core::packing and gfy_packed).
//
// Exactness: packRefs() runs packMulti's statements in packMulti's order
// (group map, the 65535-row / 65535-triangle split rule, appendSlot, flush,
// reduced_skin_vertices and max_discarded_skin_weight). The only difference is
// WHERE vertex() runs: a Row holds vertex(v,&discarded) and vertexHash() of
// the same JoinedVertex bytes, or the exception vertex() threw, which is
// rethrown (same type and message) exactly where packMulti would have called
// vertex() for that corner. vertex() is a pure function of the JoinedVertex.
//
// A Source provides, for a triangle reference r and corner k < 3:
//   unsigned material(r); std::uint32_t sourceFace(r,k); const Row& row(r,k).
// Any triangle source (union join, published mesh rows) can use this packer.
#include <algorithm>
#include <cstdint>
#include <exception>
#include <utility>
#include <vector>
#include "ThreadBudget.h"
namespace gfy_live { namespace piece_pack {
struct Row {gfy_packed::Vertex vertex;std::uint64_t hash{};double discarded{};std::exception_ptr error;};
// Row of one joined vertex: packing::vertex + vertexHash, or the failure.
inline void convert(const core::JoinedVertex&v,Row&row){
 try{row.vertex=core::packing::vertex(v,&row.discarded);row.hash=core::packing::vertexHash(row.vertex);}
 catch(...){row.error=std::current_exception();}
}
// Throws a stored conversion failure as packMulti would have thrown it: a
// fresh PackingInfluenceError with the same message (so concurrent pieces
// never share one exception object); any other type is rethrown unchanged.
[[noreturn]] inline void rethrow(const std::exception_ptr&e){
 try{std::rethrow_exception(e);}
 catch(const core::packing::PackingInfluenceError&x){throw core::packing::PackingInfluenceError(x.what());}
}
// Rows in slot order, stored as up to four contiguous parts so that each part
// is constructed (and zeroed) by the thread that fills it.
struct RowStore {
 std::vector<std::vector<Row>> parts;std::vector<std::size_t> first; // part i: slots [first[i], first[i]+parts[i].size())
 const Row& at(std::size_t slot)const{std::size_t i=parts.size()-1;while(slot<first[i])--i;return parts[i][slot-first[i]];}
 std::size_t size()const{std::size_t n=0;for(const auto&p:parts)n+=p.size();return n;}
 // Fills slots [0,count) with make(slot,row) on up to maxThreads threads.
 template<class Make>void fill(std::size_t count,unsigned maxThreads,const Make&make){
  const std::size_t chunks=sep::chunkCount(count,4096,maxThreads);
  parts.assign(chunks,{});first.assign(chunks,0);
  for(std::size_t c=0;c<chunks;++c)first[c]=count*c/chunks;
  sep::parallel(count,4096,maxThreads,[&](std::size_t a,std::size_t b,std::size_t c){
   auto&part=parts[c];part.resize(b-a);for(std::size_t s=a;s<b;++s)make(s,part[s-a]);});
 }
};
template<class Source>
inline gfy_packed::Mesh packRefs(const std::vector<std::uint32_t>&refs,const Source&src,
    const std::vector<std::uint16_t>&faceChildren,const std::vector<std::uint32_t>&faceIndices,std::uint16_t originalChildren){
 using core::packing::packingRequire;
 packingRequire(faceChildren.size()==faceIndices.size()&&!faceChildren.empty(),"Source child map");
 packingRequire(originalChildren>0&&originalChildren<gfy_packed::max_child_slots,"Original child slots");
 // std::map<Group,...> of packMulti: a dense (material<64, child<originalChildren)
 // slot table, then the groups in ascending (material, child) order.
 struct Group {unsigned material;std::uint16_t child;std::vector<std::uint32_t> refs;};
 std::vector<std::int32_t> slot(std::size_t(64)*originalChildren,-1);std::vector<Group> groups;
 auto group=[&](unsigned material,std::uint16_t child)->Group&{auto&s=slot[std::size_t(material)*originalChildren+child];
  if(s<0){s=std::int32_t(groups.size());groups.push_back({material,child,{}});}return groups[std::size_t(s)];};
 for(auto child:faceChildren){packingRequire(child<originalChildren,"Source child outside original");group(0,child);}
 const auto fallback=faceChildren.front();
 for(const auto r:refs){
  std::uint16_t source=fallback;const unsigned material=src.material(r);
  if(!material){const auto face=src.sourceFace(r,0);
   packingRequire(face<faceChildren.size()&&src.sourceFace(r,1)==face&&src.sourceFace(r,2)==face,"Exterior source face");
   source=faceChildren[face];
  }
  packingRequire(material<64,"Interior material slot limit");group(material,source).refs.push_back(r);
 }
 std::sort(groups.begin(),groups.end(),[](const Group&a,const Group&b){return a.material!=b.material?a.material<b.material:a.child<b.child;});
 gfy_packed::Mesh out;unsigned next=originalChildren;
 auto appendSlot=[&]{packingRequire(next<gfy_packed::max_child_slots,"Runtime child slots exhausted");return std::uint16_t(next++);};
 core::packing::VertexIndex vertices;
 for(const auto&group:groups){
  const auto&triangles=group.refs;
  gfy_packed::Child child;child.material=group.material;child.material_source_child=group.child;
  child.child_index=child.material?appendSlot():group.child;
  vertices.reset(std::min<std::size_t>(3*triangles.size(),65535));
  auto reserve=[&](std::size_t remaining){remaining=std::min<std::size_t>(remaining,65535);child.indices.reserve(3*remaining);child.original_faces.reserve(remaining);};
  reserve(triangles.size());
  auto flush=[&]{packingRequire(child.child_index<gfy_packed::max_child_slots,"Runtime child slots exhausted");out.children.push_back(std::move(child));};
  for(std::size_t t=0;t<triangles.size();++t){const auto r=triangles[t];
   const Row* rows[3];std::array<long,3> found;unsigned fresh=0;
   for(unsigned k=0;k<3;++k){rows[k]=&src.row(r,int(k));if(rows[k]->error)rethrow(rows[k]->error);
    found[k]=vertices.find(child.vertices,rows[k]->vertex,rows[k]->hash);if(found[k]<0)++fresh;}
   if(child.vertices.size()+fresh>65535||child.indices.size()/3>=65535){flush();child={};child.material=group.material;child.material_source_child=group.child;child.child_index=appendSlot();vertices.clear();found.fill(-1);reserve(triangles.size()-t);}
   for(unsigned k=0;k<3;++k){
    if(found[k]>=0){child.indices.push_back(std::uint16_t(found[k]));continue;}
    const auto [index,inserted]=vertices.insert(child.vertices,rows[k]->vertex,rows[k]->hash);
    if(inserted){out.reduced_skin_vertices+=rows[k]->discarded>1e-12;out.max_discarded_skin_weight=std::max(out.max_discarded_skin_weight,rows[k]->discarded);}
    child.indices.push_back(index);
   }
   child.original_faces.push_back(child.material?~std::uint32_t(0):faceIndices[src.sourceFace(r,0)]);
  }
  flush();
 }
 return out;
}
// Byte equality of two packed meshes (GFY_RX_VERIFY).
inline bool sameMesh(const gfy_packed::Mesh&a,const gfy_packed::Mesh&b){
 if(a.children.size()!=b.children.size()||a.reduced_skin_vertices!=b.reduced_skin_vertices||
    std::memcmp(&a.max_discarded_skin_weight,&b.max_discarded_skin_weight,sizeof(double)))return false;
 for(std::size_t i=0;i<a.children.size();++i){const auto&x=a.children[i];const auto&y=b.children[i];
  if(x.child_index!=y.child_index||x.material_source_child!=y.material_source_child||x.material!=y.material||
     x.vertices.size()!=y.vertices.size()||x.indices!=y.indices||x.original_faces!=y.original_faces)return false;
  if(!x.vertices.empty()&&std::memcmp(x.vertices.data(),y.vertices.data(),x.vertices.size()*sizeof(gfy_packed::Vertex)))return false;}
 return true;
}
}}
#endif
