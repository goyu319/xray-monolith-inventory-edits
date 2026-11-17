#pragma once
// Included after the actual native core and RuntimeMeshPacking types.
// This adapter ONLY packs existing admitted triangle connectivity. No clipping,
// material boundaries, surface splitting, reduction, or triangle deletion.
namespace packedchannel {
// R62: kinds (per triangle, 3 + tissue slot) and normals (per point, into the cavity) are given for a torn tissue surface
// (TissueTear.h): its triangles are not a cutter's wall/caps and its normals are the softened field's.
struct Input {std::uint32_t owner{};std::string name;core::Vec entry{},direction{};double inlet{},penetration{},exitDistance{};std::vector<core::Vec>p;std::vector<std::array<std::uint32_t,3>> triangles;
 std::vector<std::uint32_t> kinds;std::vector<core::Vec> normals;};
struct OwnerChunk {std::uint32_t child{},owner{},firstTriangle{},triangleCount{};};
enum class PrimitiveKind:std::uint32_t {wall=0,entry_cap=1,exit_cap=2,tissue=3}; // tissue+slot: R62 torn surface (mode-4 owner)
struct OwnerTriangle {std::uint32_t owner{};PrimitiveKind kind{PrimitiveKind::wall};};
struct Output {
 gfy_packed::Mesh mesh;
 std::vector<OwnerChunk> owners; // REQUIRED typed metadata, not encoded in UV/material/provenance.
 std::vector<OwnerTriangle> triangleOwners; // Same order as concatenated child index triples. Never interpolate.
 std::vector<std::vector<std::uint32_t>> positionOfRow;
 std::vector<std::array<double,64>> fullWeights;
 std::size_t inputPositions{},inputTriangles{},creaseRows{};
 bool requiresTissueField{true},requiresOwnerBinding{true};
};
inline core::Vec unit(core::Vec p){double n=core::length(p);if(!(n>1e-20))throw std::runtime_error("channel_normal_degenerate");return p*(1/n);}
inline PrimitiveKind primitiveKind(const Input& in,const std::array<std::uint32_t,3>& tri){
 const double eps=64*std::numeric_limits<double>::epsilon()*std::max({1.,std::abs(in.entry.x),std::abs(in.entry.y),std::abs(in.entry.z),in.inlet,in.penetration});
 double lo=1e300,hi=-1e300;for(auto i:tri){if(i>=in.p.size())throw std::runtime_error("channel_index");const double t=core::dot(in.p[i]-in.entry,in.direction);if(!std::isfinite(t))throw std::runtime_error("channel_depth_nonfinite");lo=std::min(lo,t);hi=std::max(hi,t);}
 if(lo< -in.inlet-eps||hi>in.penetration+eps)throw std::runtime_error("channel_triangle_straddles_cap");
 if(std::abs(lo+in.inlet)<=eps&&std::abs(hi+in.inlet)<=eps)return PrimitiveKind::entry_cap;
 if(std::abs(lo-in.penetration)<=eps&&std::abs(hi-in.penetration)<=eps){if(in.exitDistance<0)throw std::runtime_error("channel_closed_tip_has_cap_triangle");return PrimitiveKind::exit_cap;}
 return PrimitiveKind::wall;
}
inline Output pack(const Input& in,WallWeights& field,std::uint32_t boneCount,std::uint16_t firstChild,std::uint16_t materialSource){
 Output out;out.inputPositions=in.p.size();out.inputTriangles=in.triangles.size();
 if(boneCount==0||boneCount>64)throw std::runtime_error("channel_source_bone_limit");
 if(!(in.inlet>=0&&in.penetration>0)||std::abs(core::dot(in.direction,in.direction)-1)>1e-10)throw std::runtime_error("channel_axis_domain");
 if(!in.kinds.empty()&&in.kinds.size()!=in.triangles.size())throw std::runtime_error("channel_kind_count");
 if(!in.normals.empty()&&in.normals.size()!=in.p.size())throw std::runtime_error("channel_normal_count");
 for(std::size_t t=0;t<in.triangles.size();++t)out.triangleOwners.push_back({in.owner,in.kinds.empty()?primitiveKind(in,in.triangles[t]):PrimitiveKind(in.kinds[t])});
 std::vector<core::Vec> p(in.p.size()),normal(in.p.size()),faceNormal(in.triangles.size());out.fullWeights.resize(in.p.size());
 for(std::size_t i=0;i<in.p.size();++i){p[i]={double(float(in.p[i].x)),double(float(in.p[i].y)),double(float(in.p[i].z))};
  if(!field.sample(p[i],out.fullWeights[i]))throw std::runtime_error("channel_wall_skin_unavailable");
  for(unsigned b=boneCount;b<64;++b)if(out.fullWeights[i][b]!=0)throw std::runtime_error("channel_bone_outside_source");}
 for(std::size_t k=0;k<in.triangles.size();++k){const auto&t=in.triangles[k];for(auto id:t)if(id>=p.size())throw std::runtime_error("channel_index");
  // Input cutter points outward. Cavity rendering points INTO removed volume.
  const auto n=core::cross(p[t[2]]-p[t[0]],p[t[1]]-p[t[0]]);faceNormal[k]=unit(n);for(auto id:t)normal[id]=normal[id]+n;}
 for(std::size_t i=0;i<normal.size();++i)normal[i]=!in.normals.empty()&&core::length(in.normals[i])>.5?unit(in.normals[i]):unit(normal[i]); // R62: a torn surface's own normals
 gfy_packed::Child child;std::vector<std::uint32_t> positionOfRow;core::packing::VertexIndex index;
 unsigned next=firstChild;std::uint32_t firstTriangle=0;
 auto begin=[&]{if(next>=gfy_packed::max_child_slots)throw std::runtime_error("channel_child_slots");child={};child.child_index=std::uint16_t(next++);child.material_source_child=materialSource;child.material=1;positionOfRow.clear();index.reset(std::min<std::size_t>(3*in.triangles.size(),65535));};
 auto flush=[&]{out.owners.push_back({child.child_index,in.owner,firstTriangle,std::uint32_t(child.indices.size()/3)});firstTriangle+=std::uint32_t(child.indices.size()/3);out.mesh.children.push_back(std::move(child));out.positionOfRow.push_back(std::move(positionOfRow));};
 begin();
 for(std::size_t t=0;t<in.triangles.size();++t){const auto& tri=in.triangles[t];const std::uint32_t ids[3]={tri[0],tri[2],tri[1]};
  std::array<gfy_packed::Vertex,3>rows;std::array<std::uint64_t,3>hash;std::array<long,3>found;std::array<double,3>discarded{};unsigned fresh=0;
  for(unsigned c=0;c<3;++c){auto id=ids[c];core::JoinedVertex j;j.p=p[id];j.w=out.fullWeights[id];j.sourceFace=~std::uint32_t(0);
   j.normal=normal[id];if(core::dot(j.normal,faceNormal[t])<.2){j.normal=faceNormal[t];++out.creaseRows;}
   // Native wall tangent choice and 8 cm wall-texture UV convention.
   j.tangent=std::abs(j.normal.x)>.9?core::Vec{0,0,1}:core::Vec{1,0,0};j.binormal=core::cross(j.normal,j.tangent);
   j.uv={core::dot(j.p,j.tangent)/.08,core::dot(j.p,j.binormal)/.08};
   rows[c]=core::packing::vertex(j,&discarded[c]);hash[c]=core::packing::vertexHash(rows[c]);found[c]=index.find(child.vertices,rows[c],hash[c]);if(found[c]<0)++fresh;}
  if(child.vertices.size()+fresh>65535||child.indices.size()/3>=65535){flush();begin();found.fill(-1);}
  for(unsigned c=0;c<3;++c){if(found[c]>=0){child.indices.push_back(std::uint16_t(found[c]));continue;}
   auto inserted=index.insert(child.vertices,rows[c],hash[c]);child.indices.push_back(inserted.first);
   if(inserted.second){positionOfRow.push_back(ids[c]);out.mesh.reduced_skin_vertices+=discarded[c]>1e-12;out.mesh.max_discarded_skin_weight=std::max(out.mesh.max_discarded_skin_weight,discarded[c]);}}
  child.original_faces.push_back(~std::uint32_t(0));}
 flush();std::size_t emitted=0;for(const auto&c:out.mesh.children)emitted+=c.indices.size()/3;
 if(emitted!=in.triangles.size())throw std::runtime_error("channel_triangle_count_changed");return out;
}
}
