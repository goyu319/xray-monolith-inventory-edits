#pragma once
#include "OriginalSkinProxySource.h"
#include "RuntimePackedMesh.h"
#include <chrono>
#include <map>
#include <set>
namespace painter_skin_proxy {
struct ProofStats {unsigned triangles=0,planes=0,pages=0,work=0;double ms=0;bool exhausted=false;};
// Pure CPU proof over the exact immutable staged publication. A missing or
// unsupported fact declines the whole physical group, never the whole shell.
inline std::vector<std::uint16_t> prove_groups(const Source& source,const gfy_packed::Mesh& mesh,
 const std::vector<std::uint16_t>& retained,const std::vector<std::uint16_t>& owners,ProofStats* stats=nullptr){
 const auto start=std::chrono::steady_clock::now();ProofStats s;std::vector<std::uint16_t> accepted;
 auto finish=[&](){s.ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();if(stats)*stats=s;return accepted;};
 if(!source.complete_triangles||source.vertices.empty()||source.vertices.size()>16384||source.triangles.empty()||source.triangles.size()>131072||source.prepared.size()>8||owners.size()!=source.rig.size()||source.rig.size()>64||mesh.children.size()>1024)return finish();
 try{gfy_packed::validate_skin_ownership(mesh);}catch(...){return finish();}
 auto budget=[&](){if(++s.work>2000000){s.exhausted=true;return false;}return true;};
 std::set<std::uint16_t> original(retained.begin(),retained.end());
 struct Primitive {const gfy_packed::Child* child;unsigned triangle;};
 using Key=std::pair<std::uint16_t,std::uint32_t>;
 std::map<Key,std::vector<Primitive>> output;
 for(const auto& child:mesh.children){
  if(child.indices.size()%3||child.indices.size()>196605||child.vertices.size()>65535)return finish();
  if(child.material)continue; // Analytic cavity predicates cannot certify skin.
  if(child.original_faces.size()!=child.indices.size()/3)return finish();
  for(unsigned t=0;t<child.original_faces.size();++t){if(!budget())return finish();if(child.original_faces[t]==~std::uint32_t(0))continue;
   const Key key{child.material_source_child,child.original_faces[t]};auto& rows=output[key];if(rows.size()>=8)return finish();rows.push_back({&child,t});}
 }
 auto valid_vertex=[&](const Vertex& v){double sum=0;for(unsigned a=0;a<4;++a){const float w=v.weights[a];if(!std::isfinite(w)||w<0||w>1)return false;sum+=w;if(w>0){const auto b=v.bones[a];if(b>=owners.size()||source.rig[b].owner==65535||owners[b]!=source.rig[b].owner||!original.count(owners[b]))return false;}}
  for(auto x:v.position)if(!std::isfinite(x))return false;for(auto x:v.uv)if(!std::isfinite(x))return false;return std::abs(sum-1.)<1e-5;};
 auto same_vertex=[&](const Vertex& wanted,const gfy_packed::Vertex& actual){
  if(!valid_vertex(wanted))return false;
  for(unsigned a=0;a<3;++a)if(actual.surface[a]!=wanted.position[a])return false;
  if(actual.surface[12]!=wanted.uv[0]||actual.surface[13]!=wanted.uv[1])return false;
  gfy_packed::Vertex a,b;a.bone=wanted.bones;a.weight=wanted.weights;b=actual;
  for(auto w:b.weight)if(!std::isfinite(w)||w<0||w>1)return false;
  gfy_packed::canonical_weights(a);gfy_packed::canonical_weights(b);
  return a.bone==b.bone&&a.weight==b.weight;
 };
 for(const auto& group:source.prepared){
  if(!original.count(group.anchor)||group.source_vertices.empty()||group.source_triangles.empty()||group.source_vertices.size()>16384||group.source_triangles.size()>65535)continue;
  bool kept=true,clipped=false;std::array<double,3> lo{1e100,1e100,1e100},hi{-1e100,-1e100,-1e100};
  std::vector<unsigned char> touched(source.vertices.size());
  for(auto ti:group.source_triangles){if(!budget()){kept=false;break;}if(ti>=source.triangles.size()){kept=false;break;}const auto& src=source.triangles[ti];
   if(std::find(source.opaque_children.begin(),source.opaque_children.end(),src.child)==source.opaque_children.end()){kept=false;break;}
   for(auto id:src.vertices){if(id>=source.vertices.size()||!valid_vertex(source.vertices[id])){kept=false;break;}touched[id]=1;for(unsigned a=0;a<3;++a){lo[a]=std::min(lo[a],double(source.vertices[id].position[a]));hi[a]=std::max(hi[a],double(source.vertices[id].position[a]));}}
   if(!kept)break;const auto found=output.find({src.child,src.face});if(found==output.end()){kept=false;break;}
   // One complete primitive must retain the entire source triangle. Accepting
   // separate corners in different clipped rows does not prove the interior.
   bool full=false;
   for(const auto& p:found->second){if(!budget()){kept=false;break;}const auto& child=*p.child;bool exact=false;
    for(unsigned rotation=0;rotation<3&&!exact;++rotation){exact=true;for(unsigned a=0;a<3;++a){const auto id=child.indices[p.triangle*3+(a+rotation)%3];if(id>=child.vertices.size()||!same_vertex(source.vertices[src.vertices[a]],child.vertices[id])){exact=false;break;}}}
    if(!exact)continue;
    const auto* table=mesh.skin_ownership?mesh.skin_ownership->child(child.child_index):nullptr;
    if(child.skin_owned&&!table)continue;
    if(table){if(p.triangle>=table->count||p.triangle>=table->data.size())continue;const auto row=table->data[p.triangle];bool planes=true;
     for(unsigned n=0;n<row[1]&&planes;++n){if(!budget()){planes=false;kept=false;break;}++s.planes;const auto& plane=mesh.skin_ownership->planes[row[0]+n];
      for(auto id:src.vertices){const auto& x=source.vertices[id].position;const double d=double(plane[0])*x[0]+double(plane[1])*x[1]+double(plane[2])*x[2]+plane[3];
       const double error=2e-6*(1.+std::abs(double(plane[0])*x[0])+std::abs(double(plane[1])*x[1])+std::abs(double(plane[2])*x[2])+std::abs(double(plane[3])));if(!(d>error)){planes=false;break;}}}
     if(!planes)continue;
    }
    full=true;clipped=clipped||child.skin_clip;break;
   }
   ++s.triangles;if(!kept||!full){kept=false;break;}
  }
  if(!kept)continue;
  for(auto id:group.source_vertices)if(id>=touched.size()||!touched[id]){kept=false;break;}
  if(!kept)continue;
  if(clipped){const auto field=mesh.removal_field;if(!field||!field->available){continue;}
   const auto& f=*field;
   if(f.page.size()<painter_detail::DamageField::headerWords||f.page.size()>4u*1024u*1024u||!std::isfinite(f.pitch)||f.pitch<=0)continue;
   // A truly empty validated removal field has no spatial pages and keeps all.
   const bool empty=!f.size[0]&&!f.size[1]&&!f.size[2]&&!f.pages[0]&&!f.pages[1]&&!f.pages[2];
   if(!empty){std::size_t count=1;bool dimensions=true;for(unsigned a=0;a<3;++a){dimensions=dimensions&&f.size[a]&&f.pages[a]&&f.pages[a]==(f.size[a]+7)/8&&std::isfinite(f.origin[a]);if(!dimensions||count>(f.page.size()-16)/f.pages[a]){dimensions=false;break;}count*=f.pages[a];}
    if(!dimensions||count+16>f.page.size())continue;
    std::array<int,3> first{},last{};for(unsigned a=0;a<3;++a){const double epsilon=double(f.pitch)+1e-5*(1.+std::max(std::abs(lo[a]),std::abs(hi[a])));const double l=std::floor((lo[a]-epsilon-f.origin[a])/(double(f.pitch)*8)),h=std::floor((hi[a]+epsilon-f.origin[a])/(double(f.pitch)*8));first[a]=int(std::max(0.,std::min(double(f.pages[a]),l)));last[a]=int(std::max(-1.,std::min(double(f.pages[a])-1.,h)));}
    unsigned pages=0;for(int z=first[2];z<=last[2]&&kept;++z)for(int y=first[1];y<=last[1]&&kept;++y)for(int x=first[0];x<=last[0];++x){if(!budget()||++pages>4096){kept=false;s.exhausted=true;break;}++s.pages;const std::size_t index=(std::size_t(z)*f.pages[1]+unsigned(y))*f.pages[0]+unsigned(x)+16;if(f.page[index]!=painter_detail::DamageField::keptPage){kept=false;break;}}
   }
  }
  if(kept)accepted.push_back(group.anchor);
 }
 return finish();
}
}
