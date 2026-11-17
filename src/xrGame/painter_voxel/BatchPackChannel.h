#pragma once
namespace packedchannel {
struct BatchOutput {gfy_packed::Mesh mesh;std::vector<std::vector<OwnerTriangle>> owners;};
// Deterministic attribute-row batching, not geometry reduction. Each input
// index triple is appended exactly once and its typed owner/kind travels with
// it. No geometric operations or material splits are performed.
inline BatchOutput batchPacked(const std::vector<Output>& cuts,std::uint16_t firstChild,bool seedFirst=false){
 BatchOutput out;if(cuts.empty())return out;gfy_packed::Child child;std::vector<OwnerTriangle> owners;core::packing::VertexIndex index;
 unsigned next=firstChild;bool active=false;std::size_t expected=0;
 auto flush=[&]{if(!active)return;out.mesh.children.push_back(std::move(child));out.owners.push_back(std::move(owners));active=false;};
 auto begin=[&](const gfy_packed::Child&source){if(next>=gfy_packed::max_child_slots)throw std::runtime_error("channel_batch_child_slots");child={};child.child_index=std::uint16_t(next++);child.material_source_child=source.material_source_child;child.material=source.material;owners.clear();index.reset(65535);active=true;};
 bool seeded=false;
 for(const auto&cut:cuts){if(cut.mesh.tissue_field){if(out.mesh.tissue_field&&out.mesh.tissue_field!=cut.mesh.tissue_field)throw std::runtime_error("channel_batch_field_identity");out.mesh.tissue_field=cut.mesh.tissue_field;}std::size_t primitive=0;
  out.mesh.reduced_skin_vertices+=cut.mesh.reduced_skin_vertices;out.mesh.max_discarded_skin_weight=std::max(out.mesh.max_discarded_skin_weight,cut.mesh.max_discarded_skin_weight);
  // Tail reuse: the first cut is the open tail child this function built before (unique rows in first-encounter order,
  // within the row/triangle limits): taking it as is equals re-adding its triangles one by one.
  if(seedFirst&&!seeded&&&cut==&cuts.front()&&cut.mesh.children.size()==1){const auto&source=cut.mesh.children.front();seeded=true;
   if(source.indices.size()%3||source.original_faces.size()!=source.indices.size()/3||cut.triangleOwners.size()!=source.indices.size()/3)throw std::runtime_error("channel_batch_index_triples");
   begin(source);child.vertices=source.vertices;child.indices=source.indices;child.original_faces=source.original_faces;owners=cut.triangleOwners;index.seed(child.vertices);
   expected+=source.indices.size()/3;continue;}
  for(const auto&source:cut.mesh.children){if(source.indices.size()%3||source.original_faces.size()!=source.indices.size()/3)throw std::runtime_error("channel_batch_index_triples");
   if(active&&(child.material_source_child!=source.material_source_child||child.material!=source.material)){flush();}if(!active)begin(source);
   for(std::size_t i=0;i<source.indices.size();i+=3){if(primitive>=cut.triangleOwners.size())throw std::runtime_error("channel_batch_owner_count");std::array<const gfy_packed::Vertex*,3>rows;std::array<long,3>found;std::array<std::uint64_t,3>hashes;unsigned fresh=0;
    for(int c=0;c<3;++c){auto id=source.indices[i+c];if(id>=source.vertices.size())throw std::runtime_error("channel_batch_index");rows[c]=&source.vertices[id];hashes[c]=core::packing::vertexHash(*rows[c]);found[c]=index.find(child.vertices,*rows[c],hashes[c]);if(found[c]<0)++fresh;}
    if(child.vertices.size()+fresh>65535||child.indices.size()/3>=65535){flush();begin(source);found.fill(-1);}
    for(int c=0;c<3;++c)child.indices.push_back(found[c]>=0?std::uint16_t(found[c]):index.insert(child.vertices,*rows[c],hashes[c]).first);
    child.original_faces.push_back(source.original_faces[i/3]);owners.push_back(cut.triangleOwners[primitive++]);++expected;
   }}if(primitive!=cut.triangleOwners.size())throw std::runtime_error("channel_batch_unused_owners");}
 flush();std::size_t actual=0;for(std::size_t i=0;i<out.mesh.children.size();++i){const auto n=out.mesh.children[i].indices.size()/3;if(n!=out.owners[i].size())throw std::runtime_error("channel_batch_final_owners");actual+=n;}if(actual!=expected)throw std::runtime_error("channel_batch_triangle_count");return out;
}
}
