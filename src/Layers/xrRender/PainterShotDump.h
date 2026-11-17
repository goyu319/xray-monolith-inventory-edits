#pragma once
// shotdump (2026-11-17): a DIAGNOSTIC, off unless the game is started with -painter_shot_dump. With it, every screenshot also
// writes what the renderer draws of every wounded body and piece at that moment - the worker's own publication (the exterior
// children, the removal field, the anatomy field, the tear set) - as MDDUMP01 files (the offline renderer's input; the layout
// of the probe's decalDumpMesh) into <screenshots>/<screenshot name>.parts/, with a parts.txt naming them.
// Without the switch no mesh is kept and nothing is written. Compiled in SkeletonCustom.cpp (SkeletonRuntimeGeometry_impl.h).
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <direct.h>
#include "../../Include/xrRender/RuntimePackedMesh.h"
namespace painter_shot_dump {
inline bool enabled(){static const bool on=Core.Params&&strstr(Core.Params,"-painter_shot_dump")!=nullptr;return on;}
struct Live {std::shared_ptr<const gfy_packed::Mesh> mesh;std::uint64_t cookie=0,revision=0;};
inline std::mutex& lock(){static std::mutex m;return m;}
inline std::map<const void*,Live>& table(){static std::map<const void*,Live> t;return t;}
// the publication a model draws from now on (the commit); no mesh: none
inline void live(const void* model,const std::shared_ptr<const gfy_packed::Mesh>& mesh,std::uint64_t cookie,std::uint64_t revision){
 if(!enabled())return;std::lock_guard<std::mutex> g(lock());
 if(mesh){Live l;l.mesh=mesh;l.cookie=cookie;l.revision=revision;table()[model]=l;}else table().erase(model);}
inline void gone(const void* model){if(!enabled())return;std::lock_guard<std::mutex> g(lock());table().erase(model);}
// one part: MDDUMP01. tissue = false leaves the anatomy arrays out (the reader takes them from the part named in parts.txt)
inline bool write(const std::string& path,const gfy_packed::Mesh& m,std::uint32_t k,bool tissue){
 if(!m.removal_field||!m.tissue_field)return false;
 static const gfy_packed::TearAtlas none{};
 std::ofstream o(path,std::ios::binary);if(!o)return false;
 auto u32=[&](std::uint64_t v){const std::uint32_t x=std::uint32_t(v);o.write(reinterpret_cast<const char*>(&x),4);};
 auto f32=[&](double v){const float x=float(v);o.write(reinterpret_cast<const char*>(&x),4);};
 auto f64=[&](double v){o.write(reinterpret_cast<const char*>(&v),8);};
 auto raw=[&](const void*p,std::size_t n){if(n)o.write(static_cast<const char*>(p),std::streamsize(n));};
 o.write("MDDUMP01",8);u32(k);u32(m.mask_piece?1:0);u32(m.mask_volume?1:0);u32(0);
 const auto&r=*m.removal_field;for(int a=0;a<3;++a)f32(r.origin[a]);f32(r.pitch);for(int a=0;a<3;++a)u32(r.size[a]);for(int a=0;a<3;++a)u32(r.pages[a]);
 u32(r.page.size());raw(r.page.data(),r.page.size()*4);u32(r.data.size());raw(r.data.data(),r.data.size()*4);
 const auto&f=*m.tissue_field;for(int a=0;a<3;++a)f32(f.origin[a]);f32(f.pitch);for(int a=0;a<3;++a)u32(f.size[a]);for(int a=0;a<3;++a)u32(f.pages[a]);
 if(!tissue){u32(0);u32(0);}
 else{u32(f.page.size());raw(f.page.data(),f.page.size()*4);u32(f.data.size());raw(f.data.data(),f.data.size());}
 const auto&a=m.tear_atlas?*m.tear_atlas:none;u32(a.K);u32(a.paramsRows);u32(a.params.size());raw(a.params.data(),a.params.size()*4);
 // a set packed by reference (blocks + places): the arrays the renderer's upload makes, assembled
 std::vector<float> amap=a.map;std::vector<std::uint8_t> afield=a.field;
 if(!a.blocks.empty()){amap.assign(std::size_t(a.mapW)*a.mapH*4,0.f);afield.assign(std::size_t(a.fieldDim[0])*a.fieldDim[1]*a.fieldDim[2],255);
  for(std::size_t q=0;q<a.blocks.size()&&q<a.place.size();++q){if(!a.blocks[q])continue;const auto&b=*a.blocks[q];const auto&pl=a.place[q];const std::size_t rowF=std::size_t(5*b.map_dim[0])*4;
   for(std::uint32_t rr=0;rr<b.map_dim[1];++rr){const std::size_t at=(std::size_t(pl[1]+rr)*a.mapW+pl[0])*4;if(at+rowF<=amap.size()&&std::size_t(rr+1)*rowF<=b.map.size())std::memcpy(&amap[at],&b.map[std::size_t(rr)*rowF],rowF*sizeof(float));}
   for(std::uint32_t s=0;s<b.bake_dim[2];++s)for(std::uint32_t rr=0;rr<b.bake_dim[1];++rr){
    const std::size_t at=(std::size_t(pl[4]+s)*a.fieldDim[1]+pl[3]+rr)*a.fieldDim[0]+pl[2],from=(std::size_t(s)*b.bake_dim[1]+rr)*b.bake_dim[0];
    if(at+b.bake_dim[0]<=afield.size()&&from+b.bake_dim[0]<=b.bake.size())std::memcpy(&afield[at],&b.bake[from],b.bake_dim[0]);}}}
 u32(a.mapW);u32(a.mapH);u32(amap.size());raw(amap.data(),amap.size()*4);
 for(int d=0;d<3;++d)u32(a.fieldDim[d]);u32(afield.size());raw(afield.data(),afield.size());
 u32(m.tears.size());for(const auto&tr:m.tears){u32(tr.side);const auto*b=tr.block.get();u32(b?1:0);
  if(b){for(int d=0;d<3;++d)f64(b->centre[d]);for(int d=0;d<3;++d)f64(b->normal[d]);for(int d=0;d<3;++d)f64(b->u[d]);for(int d=0;d<3;++d)f64(b->w[d]);f64(b->reach);f64(b->pitch);}}
 u32(m.children.size());for(const auto&c:m.children){u32(c.child_index);u32(c.material);u32(c.skin_clip?1:0);u32(c.vertices.size());
  for(const auto&v:c.vertices){f32(v.surface[0]);f32(v.surface[1]);f32(v.surface[2]);f32(v.surface[12]);f32(v.surface[13]);}
  u32(c.indices.size());raw(c.indices.data(),c.indices.size()*2);}
 o.write("MDEND001",8);
 return bool(o);}
// every live publication into <screenshots>/<shot>.parts/ (the screenshot's own file name)
inline void now(const char* shot){
 if(!enabled()||!shot||!*shot)return;
 std::vector<Live> parts;{std::lock_guard<std::mutex> g(lock());for(const auto&e:table())if(e.second.mesh)parts.push_back(e.second);}
 string_path dir;FS.update_path(dir,"$screenshots$",shot);std::string d(dir);d+=".parts";_mkdir(d.c_str());
 std::ofstream list(d+"/parts.txt");
 list<<"# part file cookie revision mask_piece mask_volume children triangles tears atlas_tears anatomy_in box_lo box_hi first_bone\n";
 std::map<const void*,std::size_t> anatomyIn; // the anatomy field -> the part that holds its arrays
 std::size_t written=0,bytes=0;
 for(std::size_t i=0;i<parts.size();++i){const auto&m=*parts[i].mesh;if(!m.removal_field||!m.tissue_field)continue;
  const auto it=anatomyIn.find(m.tissue_field.get());const bool first=it==anatomyIn.end();const std::size_t holder=first?i:it->second;
  if(first)anatomyIn[m.tissue_field.get()]=i;
  char name[64];std::snprintf(name,sizeof(name),"mddump_shot-s0_%u.bin",unsigned(i));
  if(!write(d+"/"+name,m,std::uint32_t(i),first))continue;
  ++written;
  std::size_t tris=0;float lo[3]={1e30f,1e30f,1e30f},hi[3]={-1e30f,-1e30f,-1e30f};unsigned bone=65535u;
  for(const auto&c:m.children){tris+=c.indices.size()/3;if(!c.skin_clip)continue;if(bone==65535u&&!c.vertices.empty())bone=c.vertices[0].bone[0];
   for(const auto&v:c.vertices){if(v.surface[12]<-500.f)continue;for(int a=0;a<3;++a){lo[a]=(std::min)(lo[a],v.surface[a]);hi[a]=(std::max)(hi[a],v.surface[a]);}}}
  list<<i<<' '<<name<<' '<<parts[i].cookie<<' '<<parts[i].revision<<' '<<(m.mask_piece?1:0)<<' '<<(m.mask_volume?1:0)<<' '<<m.children.size()<<' '<<tris<<' '
      <<m.tears.size()<<' '<<(m.tear_atlas?m.tear_atlas->K:0u)<<' '<<holder<<' '<<lo[0]<<','<<lo[1]<<','<<lo[2]<<' '<<hi[0]<<','<<hi[1]<<','<<hi[2]<<' '<<bone<<"\n";}
 list.close();
 Msg("[painter_shot_dump] shot=%s parts=%u of %u folder=%s",shot,unsigned(written),unsigned(parts.size()),d.c_str());}
} // namespace painter_shot_dump
