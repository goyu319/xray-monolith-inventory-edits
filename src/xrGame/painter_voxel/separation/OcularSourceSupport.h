#pragma once
// Ocular ownership is proved from the ORIGINAL source island and the authored
// slot-14 anatomy occupying it. No texture-name, bone-name or nearest search.
namespace gfy_live { namespace ocular_source {
struct SupportSpan {int x{},y{},lo{},hi{};};
struct Island {core::Bounds bounds;std::vector<std::uint32_t> faces;std::set<std::uint16_t> labels;std::vector<SupportSpan> support;bool qualified{};};
struct Map {std::vector<Island> islands;std::vector<std::uint32_t> face_island;std::set<std::uint16_t> children;};
inline bool ocular(const Asset&asset,const Vec&p){
 if(!asset.base.anatomy||!asset.base.material_slots)return false;
 const auto label=asset.base.anatomy->point(p.x,p.y,p.z);return label<asset.base.material_slots->size()&&(*asset.base.material_slots)[label]==14;
}
inline bool triangleWitness(const Asset&asset,const bound_surface::ExtraSurface&extra,std::uint32_t face,std::size_t&work,std::uint16_t&label){
 const auto&a=*asset.base.anatomy;const auto&slots=*asset.base.material_slots;core::Poly triangle;core::Bounds bounds;
 for(unsigned k=0;k<3;++k){core::V v;v.p=extra.source.v[extra.source.f[face][k]].p;v.b[k]=1;triangle.push_back(v);bounds.add(v.p);}
 int lo[3],hi[3];for(unsigned k=0;k<3;++k){lo[k]=std::max(0,int(std::floor((bounds.lo[k]-a.origin[k])/a.pitch)));
  hi[k]=std::min(int(a.size[k])-1,int(std::floor((bounds.hi[k]-a.origin[k])/a.pitch)));}
 for(int x=lo[0];x<=hi[0];++x)for(int y=lo[1];y<=hi[1];++y){const auto col=std::size_t(y)*a.size[0]+x;
  for(auto r=a.offsets[col];r<a.offsets[col+1];++r){const auto&run=a.runs[r];if(run.label>=slots.size()||slots[run.label]!=14)continue;
   for(int z=std::max(lo[2],int(run.lo));z<=std::min(hi[2],int(run.hi)-1);++z){need(++work<=4000000,"ocular_source_witness_budget");
    auto polygon=triangle;const int cell[3]={x,y,z};for(unsigned k=0;k<3&&!polygon.empty();++k){
     polygon=bound_surface::clip(polygon,k,a.origin[k]+cell[k]*a.pitch,1);
     polygon=bound_surface::clip(polygon,k,a.origin[k]+(cell[k]+1)*a.pitch,-1);}
    if(bound_surface::fraction(polygon)>1e-14){label=run.label;return true;}}}}
 return false;
}
inline Map qualify(const Asset&asset,const bound_surface::ExtraSurface&extra){
 Map out;out.face_island.assign(extra.source.f.size(),~std::uint32_t(0));
 using Key=std::tuple<std::uint16_t,double,double,double>;std::map<Key,std::vector<std::uint32_t>>uses;
 for(std::uint32_t f=0;f<extra.source.f.size();++f)for(auto v:extra.source.f[f]){
  const auto&p=extra.source.v[v].p;uses[{extra.face_children[f],p.x,p.y,p.z}].push_back(f);}
 std::size_t witnessWork=0;
 for(std::uint32_t first=0;first<extra.source.f.size();++first){if(out.face_island[first]!=~std::uint32_t(0))continue;
  const auto id=std::uint32_t(out.islands.size());out.islands.emplace_back();auto&island=out.islands.back();
  std::vector<std::uint32_t>pending{first};out.face_island[first]=id;bool witness=false;
  while(!pending.empty()){const auto f=pending.back();pending.pop_back();island.faces.push_back(f);
   for(auto v:extra.source.f[f]){const auto&p=extra.source.v[v].p;const auto&a=extra.bindings.vertices[v].point;island.bounds.add(p);
    if(ocular(asset,p)){island.labels.insert(asset.base.anatomy->point(p.x,p.y,p.z));witness=true;}
    if(length(p-a)<=.03&&ocular(asset,a)){island.labels.insert(asset.base.anatomy->point(a.x,a.y,a.z));witness=true;}
    for(auto q:uses.at({extra.face_children[f],p.x,p.y,p.z}))if(out.face_island[q]==~std::uint32_t(0)){out.face_island[q]=id;pending.push_back(q);}}}
  const Vec extent=island.bounds.hi-island.bounds.lo;
  if(!witness&&asset.base.anatomy&&asset.base.material_slots&&std::max({extent.x,extent.y,extent.z})<=.06)
   for(const auto f:island.faces){std::uint16_t label=0;if(triangleWitness(asset,extra,f,witnessWork,label)){island.labels.insert(label);witness=true;break;}}
  // An actual eye surface can be open at its authored back. Requiring a
  // closed sphere would reject real models; bounded exact ocular overlap is
  // the semantic witness. A connected face/head/helmet is not an eye island.
  island.qualified=witness&&std::max({extent.x,extent.y,extent.z})<=.06;
  if(island.qualified)out.children.insert(extra.face_children[first]);
 }
 // Retain the entire exact authored ocular LABEL proven by the overlap. The
 // physical source can meet the fitted atlas only at its rim; destroying that
 // overlap must not erase a still-supported original eye. This map is built
 // once per immutable asset, without nearest-label expansion or refitting.
 std::set<std::uint16_t>wanted;for(const auto&i:out.islands)if(i.qualified)wanted.insert(i.labels.begin(),i.labels.end());
 std::map<std::uint16_t,std::vector<SupportSpan>>byLabel;
 if(!wanted.empty()){const auto&a=*asset.base.anatomy;for(std::size_t col=0;col+1<a.offsets.size();++col)
  for(auto r=a.offsets[col];r<a.offsets[col+1];++r){const auto&run=a.runs[r];if(wanted.count(run.label))
   byLabel[run.label].push_back({int(col%a.size[0]),int(col/a.size[0]),int(run.lo),int(run.hi)});}}
 for(auto&i:out.islands)if(i.qualified)for(auto label:i.labels){const auto&spans=byLabel[label];i.support.insert(i.support.end(),spans.begin(),spans.end());}
 return out;
}
template<class F>inline void supportCells(const Asset&asset,const Island&island,F&&visit){
 if(!island.qualified||!asset.base.anatomy||!asset.base.material_slots)return;
 const auto&a=*asset.base.anatomy;for(const auto&s:island.support)for(int z=s.lo;z<s.hi;++z)
  visit(Vec{a.origin[0]+(s.x+.5)*a.pitch,a.origin[1]+(s.y+.5)*a.pitch,a.origin[2]+(z+.5)*a.pitch});
}
inline bool survivingSupport(const Asset&asset,const Island&island,const Grid&grid){
 bool found=false;supportCells(asset,island,[&](const Vec&p){const Cell c{int(std::floor((p.x-grid.origin.x)/grid.pitch)),int(std::floor((p.y-grid.origin.y)/grid.pitch)),int(std::floor((p.z-grid.origin.z)/grid.pitch))};
  if(grid.valid(c)&&!grid.removed(c))found=true;});return found;
}
inline std::uint32_t uniqueOwner(const Asset&asset,const Island&island,const Grid&grid,const gfy_connectivity::Topology&volume){
 std::uint32_t owner=0;bool ambiguous=false;
 supportCells(asset,island,[&](const Vec&p){const auto id=bound_surface::at(p,grid,volume);if(!id)return;
  if(owner&&id!=owner)ambiguous=true;else owner=id;});return ambiguous?0:owner;
}
} }
