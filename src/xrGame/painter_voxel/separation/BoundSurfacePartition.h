#pragma once
#ifndef GFY_BOUND_SURFACE_PARTITION_INCLUDED
#define GFY_BOUND_SURFACE_PARTITION_INCLUDED
#include <atomic>
#include <algorithm>
#include <iterator>
#include <deque>
#include <memory>
#include <optional>
#include <typeinfo>
#include <unordered_map>
#include <cstdlib>
#include <type_traits>
// A fixed, original-source attachment map. Damage never changes these anchors.
// This partitions SURFACES only. Loose-garment roots and physics still require
// validation before the resulting candidates may be published in the game.
#include "SurfaceSupport.h"
#include "SurfaceOptions.h"
#include "PiecePack.h"
namespace gfy_live { namespace bound_surface {
struct Binding {Vec point;float distance{};std::uint32_t anatomy_face{};};
struct Bindings {Hash source{};std::uint32_t faces{},bones{};std::vector<Binding> vertices;std::vector<std::uint8_t> coat_faces;};
struct GarmentAdmission {
 std::array<std::uint16_t,64> bone_to_element;
 std::map<std::uint32_t,std::set<std::uint16_t>> component_elements;
 GarmentAdmission(){bone_to_element.fill(65535);}
};
inline Bindings read(const std::vector<std::uint8_t>&bytes,const Hash&sourceHash,const Source&source,const Grid&base,double maxSurfaceDistance=.35){
 need(bytes.size()>=52&&std::memcmp(bytes.data(),"VXATTCH1",8)==0,"source_binding_header");
 Bindings result;std::copy(bytes.begin()+8,bytes.begin()+40,result.source.begin());
 need(result.source==sourceHash,"source_binding_identity");
 std::uint32_t count=0;std::memcpy(&count,bytes.data()+40,4);
 std::memcpy(&result.faces,bytes.data()+44,4);std::memcpy(&result.bones,bytes.data()+48,4);
 need(count==source.v.size()&&result.faces==source.f.size()&&result.bones==source.bones,"source_binding_topology");
 need(bytes.size()==52ull+32ull*count,"source_binding_size");result.vertices.resize(count);
 for(std::size_t i=0;i<count;++i){auto&b=result.vertices[i];const auto*raw=bytes.data()+52+32*i;
  std::memcpy(&b.point,raw,24);std::memcpy(&b.distance,raw+24,4);std::memcpy(&b.anatomy_face,raw+28,4);
  for(int a=0;a<3;++a)need(std::isfinite(b.point[a]),"source_binding_nonfinite");
  need(std::isfinite(b.distance)&&b.distance>=0&&b.distance<=maxSurfaceDistance,"source_binding_distance");
  // The original 1mm anatomical asset is authoritative, independent of the
  // selected wound grid density. A coarse center can lie outside thin tissue.
  need(base.anatomy&&base.anatomy->point(b.point.x,b.point.y,b.point.z),"source_binding_without_pristine_material");
 }
 return result;
}
inline Bindings read(const std::vector<std::uint8_t>&bytes,const Asset&asset){return read(bytes,asset.source_hash,asset.source,asset.base);}
struct ExtraSurface {Source source;Bindings bindings;std::vector<std::uint16_t>face_children;std::vector<std::uint32_t>face_indices;};
inline void applyCoatRoots(Bindings&bindings,const std::vector<std::uint8_t>&bytes,const Asset&asset){
 need(bytes.size()>=52&&std::memcmp(bytes.data(),"VXROOT01",8)==0,"coat_roots_header");
 Hash source{};std::copy(bytes.begin()+8,bytes.begin()+40,source.begin());
 need(source==asset.source_hash&&source==bindings.source,"coat_roots_identity");
 std::uint32_t nv=0,nf=0,count=0;std::memcpy(&nv,bytes.data()+40,4);std::memcpy(&nf,bytes.data()+44,4);std::memcpy(&count,bytes.data()+48,4);
 need(nv==bindings.vertices.size()&&nf==bindings.faces&&count<=nv,"coat_roots_topology");
 need(bytes.size()==52ull+28ull*count+nf,"coat_roots_size");
 auto updated=bindings;std::vector<std::uint8_t> rooted(nv);
 for(std::size_t i=0;i<count;++i){std::uint32_t index=0;Vec point;const auto*raw=bytes.data()+52+28*i;
  std::memcpy(&index,raw,4);std::memcpy(&point,raw+4,24);
  need(index<nv&&!rooted[index],"coat_roots_duplicate_or_invalid_vertex");
  for(int a=0;a<3;++a)need(std::isfinite(point[a]),"coat_roots_nonfinite");
  need(asset.base.anatomy&&asset.base.anatomy->point(point.x,point.y,point.z),"coat_root_without_pristine_material");
  updated.vertices[index].point=point;rooted[index]=1;
 }
 updated.coat_faces.assign(bytes.begin()+52+28ull*count,bytes.end());
 for(std::size_t f=0;f<nf;++f){const auto&face=asset.source.f[f];
  const bool expected=rooted[face[0]]||rooted[face[1]]||rooted[face[2]];
  need(updated.coat_faces[f]==std::uint8_t(expected),"coat_root_face_provenance");
 }
 bindings=std::move(updated);
}
struct Stats {
 std::size_t input{},kept{},split{},trimmed{},output{},air_cavity{},unsupported_coat_triangles{};
 std::uint64_t work{};
 double original_area{},emitted_area{},trimmed_area{},air_cavity_area{};
};
// deferKeep: an intact kept input triangle is recorded as (deferred, owner)
// instead of being copied; the caller appends the source triangle itself.
struct Result {std::map<std::uint32_t,core::JoinResult> meshes;Stats stats;bool deferKeep{},deferred{};std::uint32_t deferredOwner{};
 void keep(std::uint32_t owner,const core::JoinedTriangle&t){if(deferKeep){deferred=true;deferredOwner=owner;}else meshes[owner].triangles.push_back(t);}};
inline core::JoinedVertex interpolate(const core::JoinedTriangle&t,const std::array<double,3>&b);
struct Piece {std::map<std::uint16_t,std::uint16_t> constrained_anchors;std::uint32_t volume_component{};std::size_t triangles{};gfy_packed::Mesh mesh;};
struct Packed {std::vector<Piece> pieces;Stats stats;bool garment_roots_validated{};};
inline double area(const core::JoinedTriangle&t){return length(cross(t.v[1].p-t.v[0].p,t.v[2].p-t.v[0].p))*.5;}
// An anchor corner that an earlier partition cut on a cell plane returns from its packed float32 row (and the
// barycentric solve in original_accessory::barycentric) a few float steps off that plane: <= ~5e-7 m for positions up
// to 2 m. Re-clipping it per cell then shaved a band thinner than the output can represent off every garment
// fragment on every later split (soldier gear: 246 faces -> 4,412 fragments after two re-splits, then coincident
// packed rows the accessory collector refuses). Put such a corner back onto the plane: eta = 1e-6 m is twice the
// largest re-anchoring error and 1000x under the 1 mm cell pitch, so no real cut (>= one cell) can be covered by it.
constexpr double anchorSnapDistance=1e-6;
inline Vec snapToCellPlanes(const Vec&p,const Grid&grid){
 Vec out=p;for(int a=0;a<3;++a){const double rel=(p[a]-grid.origin[a])/grid.pitch,k=std::round(rel);
  if(std::abs(rel-k)*grid.pitch<=anchorSnapDistance)out[a]=grid.origin[a]+k*grid.pitch;}
 return out;
}
// Degeneracy of the PACKED triangle, decided on the float32 corners core::packing::vertex will store: the corners are
// not pairwise distinct, or they are exactly collinear (the cross product of the float differences, computed in
// double, is exactly zero: float differences and their products are exact in double, and a difference of two
// doubles is zero only when they are equal). Such a triangle has no representable area. Any other sub-triangle,
// however thin, is kept: a one-float-step-tall triangle still has nonzero packed area.
inline bool packedDegenerate(const core::JoinedTriangle&t){
 std::array<std::array<double,3>,3> f{};for(int k=0;k<3;++k)for(int a=0;a<3;++a)f[k][a]=double(float(t.v[k].p[a]));
 if(f[0]==f[1]||f[1]==f[2]||f[0]==f[2])return true;
 const double ux=f[1][0]-f[0][0],uy=f[1][1]-f[0][1],uz=f[1][2]-f[0][2],vx=f[2][0]-f[0][0],vy=f[2][1]-f[0][1],vz=f[2][2]-f[0][2];
 return uy*vz-uz*vy==0&&uz*vx-ux*vz==0&&ux*vy-uy*vx==0;
}
// The corners of a clipped polygon that are distinct at float32 position precision, interpolated once.
// Preconditions, both guaranteed by the producer: the polygon is convex (clip() keeps convexity; mergeConvex accepts a
// union only when every corner lies on the interior side of every edge, so no pinched or self-touching polygon
// reaches the fan), and it is a sub-region of ONE source triangle t, so every attribute (normal, tangent, UV, skin
// weights) is a linear function of the barycentrics: two corners at the same float position cannot carry distinct
// attributes beyond float rounding. A repeated position in a convex polygon is a zero-width spike (two consecutive
// collinear edges through the same point); fanning over it emits the same packed triangle twice. The first corner's
// attributes are kept.
inline std::vector<core::JoinedVertex> distinctRing(const core::JoinedTriangle&t,const core::Poly&poly){
 std::vector<core::JoinedVertex> ring;ring.reserve(poly.size());
 for(const auto&v:poly){auto j=interpolate(t,v.b);bool repeated=false;
  for(const auto&r:ring)if(float(r.p.x)==float(j.p.x)&&float(r.p.y)==float(j.p.y)&&float(r.p.z)==float(j.p.z)){repeated=true;break;}
  if(!repeated)ring.push_back(std::move(j));}
 return ring;
}
inline void trimUnsupportedCoat(Result&result,const Bindings&bindings,const GarmentAdmission&garments){
 for(auto&entry:result.meshes){
  const auto admitted=garments.component_elements.find(entry.first);
  need(admitted!=garments.component_elements.end(),"coat_partition_missing_component_elements");
  auto&triangles=entry.second.triangles;
  auto unsupported=[&](const core::JoinedTriangle&t){
   if(t.material||t.v[0].sourceFace>=bindings.coat_faces.size()||!bindings.coat_faces[t.v[0].sourceFace])return false;
   for(const auto&v:t.v)for(std::size_t bone=0;bone<bindings.bones;++bone)
    if((v.w[bone]>1e-6||v.w[bone]<0)&&!admitted->second.count(garments.bone_to_element[bone]))return true;
   return false;
  };
  triangles.erase(std::remove_if(triangles.begin(),triangles.end(),[&](const core::JoinedTriangle&t){
   if(!unsupported(t))return false;
   const auto a=area(t);result.stats.emitted_area-=a;result.stats.trimmed_area+=a;
   --result.stats.output;++result.stats.unsupported_coat_triangles;return true;
  }),triangles.end());
 }
}
inline double fraction(const core::Poly&p){
 double result=0;
 for(std::size_t i=1;i+1<p.size();++i){
  const auto&a=p[0].b;const auto&b=p[i].b;const auto&c=p[i+1].b;
  result+=std::abs((b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1]));
 }
 return result;
}
inline core::Poly clip(const core::Poly&p,int axis,double bound,double sign){
 core::Poly out;if(p.empty())return out;
 auto same=[](const core::V&a,const core::V&b){double d=0;for(int i=0;i<3;++i)d=std::max(d,std::abs(a.b[i]-b.b[i]));return d<1e-14;};
 auto append=[&](const core::V&v){if(out.empty()||!same(out.back(),v))out.push_back(v);};
 for(std::size_t i=0;i<p.size();++i){const auto&a=p[i];const auto&b=p[(i+1)%p.size()];
  const auto da=(a.p[axis]-bound)*sign,db=(b.p[axis]-bound)*sign;const bool ia=da>=0,ib=db>=0;
  if(ia)append(a);
  if(ia!=ib){const auto u=std::clamp(da/(da-db),0.,1.);core::V v;v.p=a.p+(b.p-a.p)*u;v.p[axis]=bound;
   for(int k=0;k<3;++k)v.b[k]=a.b[k]+(b.b[k]-a.b[k])*u;append(v);}
 }
 if(out.size()>1&&same(out.front(),out.back()))out.pop_back();
 if(out.size()<3||fraction(out)<1e-14)out.clear();return out;
}
inline double crossBary(const core::V&a,const core::V&b,const core::V&c){
 return (b.b[1]-a.b[1])*(c.b[2]-b.b[2])-(b.b[2]-a.b[2])*(c.b[1]-b.b[1]);
}
inline std::vector<core::Poly> mergeConvex(std::vector<core::Poly>input){
 // Remove cell-grid tessellation without smoothing the cut contour. Merge
 // only across the exact same barycentric edge when the union is convex.
 // Preserve collinear edge sites until all neighbor merges have finished.
 using Key=std::array<std::int64_t,2>;using Edge=std::array<Key,2>;
 auto key=[](const core::V&v){return Key{{std::llround(v.b[1]*1e11),std::llround(v.b[2]*1e11)}};};
 struct Use{std::size_t polygon{},edge{};unsigned version{};};
 std::map<Edge,std::vector<Use>>edges;std::vector<unsigned>versions(input.size());
 std::vector<bool>alive(input.size(),true);std::deque<std::size_t>pending;
 for(std::size_t i=0;i<input.size();++i)pending.push_back(i);
 std::size_t work=0;
 while(!pending.empty()){const auto id=pending.front();pending.pop_front();if(!alive[id])continue;
  bool merged=false;
  for(std::size_t i=0;i<input[id].size()&&!merged;++i){
   need(++work<2000000,"source_partition_polygon_merge_budget");auto&p=input[id];
   const Edge edge{{key(p[i]),key(p[(i+1)%p.size()])}},reverse{{edge[1],edge[0]}};
   auto found=edges.find(reverse);
   if(found!=edges.end())for(const auto use:found->second){
    if(use.polygon==id||!alive[use.polygon]||versions[use.polygon]!=use.version)continue;
    const auto&q=input[use.polygon];core::Poly candidate;candidate.reserve(p.size()+q.size()-2);
    for(std::size_t k=1;k<=p.size();++k)candidate.push_back(p[(i+k)%p.size()]);
    for(std::size_t k=2;k<q.size();++k)candidate.push_back(q[(use.edge+k)%q.size()]);
    bool convex=true;std::set<Key>unique;
    for(const auto&v:candidate)if(!unique.insert(key(v)).second){convex=false;break;}
    // Local turns alone also admit a self-intersecting star. Every vertex
    // must be on the interior side of every edge before we fan-triangulate.
    for(std::size_t k=0;k<candidate.size()&&convex;++k)for(std::size_t qv=0;qv<candidate.size();++qv)
     if(crossBary(candidate[k],candidate[(k+1)%candidate.size()],candidate[qv])<-1e-13){convex=false;break;}
    if(!convex)continue;
    if(std::abs(fraction(candidate)-fraction(p)-fraction(q))>1e-10)continue;
    alive[use.polygon]=false;input[id]=std::move(candidate);++versions[id];pending.push_back(id);merged=true;break;
   }
   if(!merged)edges[edge].push_back({id,i,versions[id]});
  }
 }
 std::vector<core::Poly>result;
 for(std::size_t i=0;i<input.size();++i)if(alive[i]){auto p=std::move(input[i]);const double originalFraction=fraction(p);
  bool changed=true;while(changed&&p.size()>3){changed=false;
   for(std::size_t k=0;k<p.size();++k)if(std::abs(crossBary(p[(k+p.size()-1)%p.size()],p[k],p[(k+1)%p.size()]))<1e-13){
    p.erase(p.begin()+k);changed=true;break;}}
  need(std::abs(fraction(p)-originalFraction)<1e-10,"source_partition_polygon_cleanup_area");
  result.push_back(std::move(p));}
 return result;
}
inline std::uint32_t at(const Vec&p,const Grid&grid,const gfy_connectivity::Topology&topology){
 return topology.at(int(std::floor((p.x-grid.origin.x)/grid.pitch)),
                    int(std::floor((p.y-grid.origin.y)/grid.pitch)),
                    int(std::floor((p.z-grid.origin.z)/grid.pitch)));
}
// Carry local triangle barycentrics through every clip. Physical positions,
// source provenance, UVs and the full skin palette are interpolated afterward.
inline core::JoinedVertex interpolate(const core::JoinedTriangle&t,const std::array<double,3>&b){
 core::JoinedVertex out;out.sourceFace=t.v[0].sourceFace;
 for(int k=0;k<3;++k){out.p=out.p+t.v[k].p*b[k];out.normal=out.normal+t.v[k].normal*b[k];
  out.tangent=out.tangent+t.v[k].tangent*b[k];out.binormal=out.binormal+t.v[k].binormal*b[k];
  for(int a=0;a<3;++a)out.sourceBary[a]+=t.v[k].sourceBary[a]*b[k];
  for(int a=0;a<2;++a)out.uv[a]+=t.v[k].uv[a]*b[k];
  for(int a=0;a<64;++a)out.w[a]+=t.v[k].w[a]*b[k];}
 return out;
}
inline bool coversRemovedAnatomy(const core::Poly&polygon,const core::Bounds&bounds,const Grid&grid,Stats&stats){
 Cell lo,hi;for(int a=0;a<3;++a){
  (&lo.x)[a]=std::max(0,int(std::floor((bounds.lo[a]-grid.origin[a])/grid.pitch)));
  (&hi.x)[a]=std::min(grid.size[a]-1,int(std::floor((bounds.hi[a]-grid.origin[a])/grid.pitch)));}
 for(int tx=lo.x/8;tx<=hi.x/8;++tx)for(int ty=lo.y/8;ty<=hi.y/8;++ty)for(int tz=lo.z/8;tz<=hi.z/8;++tz){
  need(++stats.work<=160000000,"source_partition_work_budget");
  if(grid.tiles.find({tx,ty,tz})==grid.tiles.end())continue;
  for(int x=std::max(lo.x,tx*8);x<=std::min(hi.x,tx*8+7);++x)
   for(int y=std::max(lo.y,ty*8);y<=std::min(hi.y,ty*8+7);++y)
    for(int z=std::max(lo.z,tz*8);z<=std::min(hi.z,tz*8+7);++z){
     need(++stats.work<=160000000,"source_partition_work_budget");Cell c{x,y,z};
     if(!grid.removed(c)||!grid.solidBase(c))continue;
     auto clipped=polygon;
     for(int a=0;a<3&&!clipped.empty();++a){clipped=bound_surface::clip(clipped,a,grid.origin[a]+c[a]*grid.pitch,1);
      clipped=bound_surface::clip(clipped,a,grid.origin[a]+(c[a]+1)*grid.pitch,-1);}
     if(fraction(clipped)>1e-14)return true;
    }
 }
 return false;
}
inline bool intactOverOneOwner(const std::array<Vec,3>&anchors,const Grid&grid,const gfy_connectivity::Topology&volume,
                               const std::set<std::uint32_t>&retained,std::uint32_t&owner,std::uint64_t&workBound){
 core::Poly polygon;core::Bounds bounds;
 for(int k=0;k<3;++k){core::V v;v.p=snapToCellPlanes(anchors[k],grid);v.b[k]=1;polygon.push_back(v);bounds.add(v.p);}
 for(int a=0;a<3;++a)if(!std::isfinite(bounds.lo[a])||!std::isfinite(bounds.hi[a]))return false;
 auto plane=[&](int a,int k){return grid.origin[a]+k*grid.pitch;}; // the clip planes' own expression (origin + cell * pitch)
 int ref[2][2],lo[3],hi[3]; // ref: the partition's x / y cell range (clamped as it clamps); lo / hi: the cells the triangle enters
 for(int a=0;a<2;++a){const int l=int(std::floor((bounds.lo[a]-grid.origin[a])/grid.pitch)),h=int(std::floor((bounds.hi[a]-grid.origin[a])/grid.pitch));
  ref[a][0]=std::max(0,l);ref[a][1]=std::min(grid.size[a]-1,h);
  int el=l,eh=h;if(eh>el&&plane(a,eh)>=bounds.hi[a])--eh;if(el<eh&&plane(a,el+1)<=bounds.lo[a])++el;
  if(el<0||eh>grid.size[a]-1||el>eh)return false;lo[a]=el;hi[a]=eh;}
 const bool flatZ=bounds.hi.z-bounds.lo.z<1e-12;const int flatCell=int(std::floor((bounds.lo.z-grid.origin.z)/grid.pitch));
 const Vec e1=polygon[1].p-polygon[0].p,e2=polygon[2].p-polygon[0].p,n=cross(e1,e2);const double nn=length(n);
 if(!(nn>1e-18))return false; // no area: the partition's own rules decide
 const bool slopeOk=std::abs(n.z)>=.05*nn; // z a function of x, y with slope <= 20 on this triangle's plane
 const Vec P0=polygon[0].p;
 auto zAt=[&](double x,double y){return P0.z-(n.x*(x-P0.x)+n.y*(y-P0.y))/n.z;};
 bool any=false;owner=0;
 for(int y=lo[1];y<=hi[1];++y){
  auto column=std::lower_bound(volume.columns.begin(),volume.columns.end(),std::array<int,2>{{lo[0],y}},
   [](const gfy_connectivity::Column&c,const std::array<int,2>&p){return gfy_connectivity::xyLess(c.x,c.y,p[0],p[1]);});
  for(int x=lo[0];x<=hi[0];++x,++column){
   if(column==volume.columns.end()||column->y!=y||column->x!=x)return false; // a column the triangle enters holds no span
   int zl,zh;
   if(flatZ){zl=flatCell;zh=std::max(flatCell,int(std::floor((bounds.hi.z-grid.origin.z)/grid.pitch)));} // its span covers it all
   else{double zmin=bounds.lo.z,zmax=bounds.hi.z;
    if(slopeOk){const double xs[2]={plane(0,x),plane(0,x+1)},ys[2]={plane(1,y),plane(1,y+1)};double a=1e300,b=-1e300;
     for(const double px:xs)for(const double py:ys){const double z=zAt(px,py);a=std::min(a,z);b=std::max(b,z);}
     if(std::isfinite(a)&&std::isfinite(b)){zmin=std::max(zmin,a-1e-9);zmax=std::min(zmax,b+1e-9);}}
    if(zmin>zmax)return false;
    zl=int(std::floor((zmin-grid.origin.z)/grid.pitch));zh=int(std::floor((zmax-grid.origin.z)/grid.pitch));
    if(zh>zl&&plane(2,zh)>=zmax)--zh;if(zl<zh&&plane(2,zl+1)<=zmin)++zl;}
   bool covered=false;
   for(auto index=column->first;index<column->first+column->count;++index){const auto&span=volume.spans[index];
    if(flatZ){if(flatCell<span.lo||flatCell>=span.hi)continue;} // the partition reads only spans holding flatCell
    else if(span.hi<=zl||span.lo>zh)continue; // meets no cell of the range
    if(any&&span.component!=owner)return false;
    owner=span.component;any=true;
    if(span.lo<=zl&&span.hi>=zh+1)covered=true;}
   if(!covered)return false;}}
 if(!any||!retained.count(owner))return false;
 Stats removed{};if(coversRemovedAnatomy(polygon,bounds,grid,removed))return false;
 std::uint64_t work=removed.work; // the partition's most: a row each, a column and its spans each, in its clamped box
 for(int y=ref[1][0];y<=ref[1][1];++y){++work;
  auto column=std::lower_bound(volume.columns.begin(),volume.columns.end(),std::array<int,2>{{ref[0][0],y}},
   [](const gfy_connectivity::Column&c,const std::array<int,2>&p){return gfy_connectivity::xyLess(c.x,c.y,p[0],p[1]);});
  for(;column!=volume.columns.end()&&column->y==y&&column->x<=ref[0][1];++column)work+=1+column->count;}
 workBound=work;return true;
}
// ---- R51 partition certificate (exact; read-only facts of ONE partition stage) ------------------------------------
// Built once by the caller after the grid, the remaining volume and the retained set are final for the stage
// (buildPartitionCertificate), then passed to partitionTriangle. A certificate never decides an outcome. It only proves,
// per 8^3 tile, that a reference loop cannot find something:
//  * tileOwner(): every cell of the tile is air or belongs to at most one retained component, so the early-accept
//    foreign scan finds no OTHER retained span there;
//  * present/damaged(): coversRemovedAnatomy() examines a tile only when grid.tiles holds its key, and a clean tile has
//    no cell with removed(c)&&solidBase(c), so the reference clips nothing in it.
// An uncertified tile sends the triangle through the reference statements unchanged. Per-tile facts are computed on
// first use and stored atomically; concurrent first uses compute the same value (bindingRegions runs on worker threads).
struct PartitionCertificate {
 const Grid*grid{};const gfy_connectivity::Topology*volume{};std::vector<std::uint32_t> retainedIds; // ascending
 // split2 part 12 (2026-11-12): the three per-tile arrays are zeroed memory from std::calloc (0 = no grid tile / unknown /
 // unknown). The former code wrote the zero into each of the 3.6 million entries of a body's grid (21.5 MB, 8 ms per
 // separation); the entries a separation never asks about are now never touched.
 struct ZeroedBlock {void operator()(void*block)const noexcept{std::free(block);}};
 int tx{},ty{},tz{};std::unique_ptr<std::uint8_t[],ZeroedBlock> present;
 std::unique_ptr<std::atomic<std::uint32_t>[],ZeroedBlock> owners; // 0 unknown, 1 no retained cell, id+2 one component, UINT32_MAX several
 std::unique_ptr<std::atomic<std::uint8_t>[],ZeroedBlock> damage;  // 0 unknown, 1 no removed solid cell, 2 holds one
 std::size_t slot(int x,int y,int z)const{return (std::size_t(x)*std::size_t(ty)+std::size_t(y))*std::size_t(tz)+std::size_t(z);}
 bool matches(const Grid&g,const gfy_connectivity::Topology&v,const std::set<std::uint32_t>&r)const{
  return grid==&g&&volume==&v&&retainedIds.size()==r.size()&&std::equal(retainedIds.begin(),retainedIds.end(),r.begin());}
 bool isRetained(std::uint32_t c)const{return std::binary_search(retainedIds.begin(),retainedIds.end(),c);}
 std::uint32_t tileOwner(int x,int y,int z)const{
  auto&cached=owners[slot(x,y,z)];std::uint32_t value=cached.load(std::memory_order_acquire);if(value)return value;
  value=1;const int z0=z*8,z1=z*8+7;
  for(int row=y*8;row<std::min(y*8+8,grid->size.y)&&value!=UINT32_MAX;++row){
   auto column=std::lower_bound(volume->columns.begin(),volume->columns.end(),std::array<int,2>{{x*8,row}},
    [](const gfy_connectivity::Column&c,const std::array<int,2>&p){return gfy_connectivity::xyLess(c.x,c.y,p[0],p[1]);});
   for(;column!=volume->columns.end()&&column->y==row&&column->x<x*8+8&&value!=UINT32_MAX;++column)
    for(auto i=column->first;i<column->first+column->count;++i){const auto&span=volume->spans[i];
     if(span.hi<=z0||span.lo>z1||!isRetained(span.component))continue;
     if(span.component>=UINT32_MAX-2){value=UINT32_MAX;break;}
     const std::uint32_t id=span.component+2;if(value==1)value=id;else if(value!=id){value=UINT32_MAX;break;}}}
  cached.store(value,std::memory_order_release);return value;}
 bool damaged(int x,int y,int z)const{
  auto&cached=damage[slot(x,y,z)];std::uint8_t value=cached.load(std::memory_order_acquire);if(value)return value==2;
  value=1;const auto found=grid->tiles.find(Cell{x,y,z});
  if(found!=grid->tiles.end()){const auto&tile=found->second;
   for(int a=x*8;a<std::min(x*8+8,grid->size.x)&&value==1;++a)
    for(int b=y*8;b<std::min(y*8+8,grid->size.y)&&value==1;++b)
     for(int c=z*8;c<std::min(z*8+8,grid->size.z);++c){const Cell cell{a,b,c};
      // Grid::removed() without ownership reads exactly this tile's bit (the key is in grid.tiles).
      const bool removed=grid->ownership?grid->removed(cell):core::tileBit(tile,cell);
      if(removed&&grid->solidBase(cell)){value=2;break;}}}
  cached.store(value,std::memory_order_release);return value==2;}
 // The early-accept foreign scan reads columns x in [lo0,hi0], y in [lo1,hi1] and every span with span.hi>=zlo and
 // span.lo<=zhi, i.e. a span with a cell in [zlo-1,zhi]. True when no tile meeting those cells holds a retained
 // component other than `owner`: then the scan finds no foreign span (it counts no work, so skipping it is exact).
 bool noForeign(std::uint32_t owner,const int*lo,const int*hi,int zlo,int zhi)const{
  if(owner>=UINT32_MAX-2)return false;
  const int z0=std::max(0,zlo-1),z1=std::min(grid->size.z-1,zhi);
  if(lo[0]>hi[0]||lo[1]>hi[1]||z0>z1)return true;
  for(int x=lo[0]>>3;x<=hi[0]>>3;++x)for(int y=lo[1]>>3;y<=hi[1]>>3;++y)for(int z=z0>>3;z<=z1>>3;++z){
   const auto o=tileOwner(x,y,z);if(o!=1&&o!=owner+2)return false;}
  return true;}
};
inline std::shared_ptr<const PartitionCertificate> buildPartitionCertificate(const Grid&grid,const gfy_connectivity::Topology&volume,
    const std::set<std::uint32_t>&retained){
 // This is an optional accelerator. A large sparse domain must not allocate
 // a dense cache without a bound or turn allocation pressure into a refusal.
 constexpr std::size_t cacheLimit=32u*1024u*1024u;
 constexpr std::size_t perTile=sizeof(std::uint8_t)+sizeof(std::atomic<std::uint32_t>)+sizeof(std::atomic<std::uint8_t>);
 const std::size_t overhead=sizeof(PartitionCertificate)+64u; // shared control block allowance
 if(retained.size()>(cacheLimit-overhead)/sizeof(std::uint32_t))return {};
 const std::size_t remaining=cacheLimit-overhead-retained.size()*sizeof(std::uint32_t);
 std::size_t n=1;int dims[3];
 for(unsigned a=0;a<3;++a){if(grid.size[a]<=0)return {};dims[a]=grid.size[a]/8+(grid.size[a]%8!=0);
  const auto dim=std::size_t(dims[a]);if(n>remaining/perTile/dim)return {};n*=dim;}
 try{
 auto c=std::make_shared<PartitionCertificate>();c->grid=&grid;c->volume=&volume;c->retainedIds.assign(retained.begin(),retained.end());
 c->tx=dims[0];c->ty=dims[1];c->tz=dims[2];
 // split2 part 12: zeroed blocks; an atomic of these two integer types is the integer itself (no lock, no constructor work)
 static_assert(sizeof(std::atomic<std::uint32_t>)==sizeof(std::uint32_t)&&sizeof(std::atomic<std::uint8_t>)==sizeof(std::uint8_t)&&
  std::is_trivially_destructible<std::atomic<std::uint32_t>>::value&&std::is_trivially_destructible<std::atomic<std::uint8_t>>::value&&
  std::atomic<std::uint32_t>::is_always_lock_free&&std::atomic<std::uint8_t>::is_always_lock_free,"partition_certificate_atomic_layout");
 c->present.reset(static_cast<std::uint8_t*>(std::calloc(n,sizeof(std::uint8_t))));
 c->owners.reset(static_cast<std::atomic<std::uint32_t>*>(std::calloc(n,sizeof(std::atomic<std::uint32_t>))));
 c->damage.reset(static_cast<std::atomic<std::uint8_t>*>(std::calloc(n,sizeof(std::atomic<std::uint8_t>))));
 if(!c->present||!c->owners||!c->damage)return {}; // no memory for the accelerator: the reference statements run without it
 for(const auto&entry:grid.tiles){const Cell&k=entry.first;
  if(k.x>=0&&k.y>=0&&k.z>=0&&k.x<c->tx&&k.y<c->ty&&k.z<c->tz)c->present[c->slot(k.x,k.y,k.z)]=1;}
 return c;
 }catch(const std::bad_alloc&){return {};}
}
// coversRemovedAnatomy() under a certificate. When no tile it would examine (grid.tiles holds the key) has a removed
// solid cell, the reference visits every tile and cell, clips nothing and returns false: add exactly the work it
// counts (one per tile, one per cell of each examined tile inside the box) and return false. Otherwise the reference
// function runs unchanged from the start. The budget check sees the same total (one check instead of one per step).
inline bool coversRemovedAnatomyCertified(const core::Poly&polygon,const core::Bounds&bounds,const Grid&grid,Stats&stats,const PartitionCertificate&cert){
 Cell lo,hi;for(int a=0;a<3;++a){
  (&lo.x)[a]=std::max(0,int(std::floor((bounds.lo[a]-grid.origin[a])/grid.pitch)));
  (&hi.x)[a]=std::min(grid.size[a]-1,int(std::floor((bounds.hi[a]-grid.origin[a])/grid.pitch)));}
 std::uint64_t work=0;
 for(int tx=lo.x/8;tx<=hi.x/8;++tx)for(int ty=lo.y/8;ty<=hi.y/8;++ty)for(int tz=lo.z/8;tz<=hi.z/8;++tz){
  ++work;if(!cert.present[cert.slot(tx,ty,tz)])continue;
  if(cert.damaged(tx,ty,tz))return coversRemovedAnatomy(polygon,bounds,grid,stats);
  const std::uint64_t nx=std::uint64_t(std::max(0,std::min(hi.x,tx*8+7)-std::max(lo.x,tx*8)+1));
  const std::uint64_t ny=std::uint64_t(std::max(0,std::min(hi.y,ty*8+7)-std::max(lo.y,ty*8)+1));
  const std::uint64_t nz=std::uint64_t(std::max(0,std::min(hi.z,tz*8+7)-std::max(lo.z,tz*8)+1));
  work+=nx*ny*nz;}
 stats.work+=work;need(stats.work<=160000000,"source_partition_work_budget");return false;
}
// R66: mergeConvexFast() with this thread's scratch (defined after it).
inline std::vector<core::Poly> mergeConvexThreadLocal(std::vector<core::Poly>input);
// R66: when set (parallel accessory partitions), the highest work count a certified call reaches before its counter reset.
inline thread_local std::uint64_t* partitionWorkPeak=nullptr;
inline void partitionTriangleCertified(const core::JoinedTriangle&t,const std::array<Vec,3>&anchors,const Grid&grid,
    const gfy_connectivity::Topology&volume,const std::set<std::uint32_t>&retained,Result&out,const PartitionCertificate&cert);
#ifdef GFY_PARTITION_CERT_VERIFY
inline void partitionCertifiedVerified(const core::JoinedTriangle&t,const std::array<Vec,3>&anchors,const Grid&grid,
    const gfy_connectivity::Topology&volume,const std::set<std::uint32_t>&retained,Result&out,const PartitionCertificate&cert);
#endif
inline void partitionTriangle(const core::JoinedTriangle&t,const std::array<Vec,3>&anchors,
                             const Grid&grid,const gfy_connectivity::Topology&volume,
                             const std::set<std::uint32_t>&retained,Result&out,const PartitionCertificate*cert=nullptr){
 // R51: a certificate of this exact stage (same grid, volume and retained set) takes the certified path; any other
 // call, or a certificate of another stage, runs the reference statements below unchanged.
 if(cert&&cert->matches(grid,volume,retained)){
#ifdef GFY_PARTITION_CERT_VERIFY
  partitionCertifiedVerified(t,anchors,grid,volume,retained,out,*cert);
#else
  partitionTriangleCertified(t,anchors,grid,volume,retained,out,*cert);
#endif
  return;}
 core::Poly polygon;core::Bounds bounds;std::array<std::uint32_t,3> cornerOwners{};
 for(int k=0;k<3;++k){core::V v;v.p=snapToCellPlanes(anchors[k],grid);v.b[k]=1;polygon.push_back(v);bounds.add(v.p);cornerOwners[k]=at(v.p,grid,volume);}
 const auto originalArea=area(t);++out.stats.input;out.stats.original_area+=originalArea;
 std::map<std::uint32_t,std::vector<core::Poly>> pieces;
 auto spend=[&](){need(++out.stats.work<=160000000,"source_partition_work_budget");};
 int lo[2],hi[2];
 for(int a=0;a<2;++a){lo[a]=std::max(0,int(std::floor((bounds.lo[a]-grid.origin[a])/grid.pitch)));
  hi[a]=std::min(grid.size[a]-1,int(std::floor((bounds.hi[a]-grid.origin[a])/grid.pitch)));}
 const bool flatZ=bounds.hi.z-bounds.lo.z<1e-12;
 const int flatCell=int(std::floor((bounds.lo.z-grid.origin.z)/grid.pitch));
 // Exact early acceptance of an intact single-owner triangle. The reference
 // below keeps the original triangle iff (a) all corners have one retained
 // owner C, (b) the clipped pieces contain exactly the key C and (c) no
 // removed solid cell is covered. If no span of another RETAINED component
 // lies within the triangle's cell box (one-cell z margin), the piece keys
 // are a subset of {C}; the first positive C piece of the same clip sequence
 // then proves (b). Otherwise, or if (c) fails, the full reference runs.
 if(cornerOwners[0]==cornerOwners[1]&&cornerOwners[1]==cornerOwners[2]&&retained.count(cornerOwners[0])){
  const auto owner=cornerOwners[0];
  const int zlo=int(std::floor((bounds.lo.z-grid.origin.z)/grid.pitch))-1,zhi=int(std::floor((bounds.hi.z-grid.origin.z)/grid.pitch))+1;
  bool foreign=false;
  for(int y=lo[1];y<=hi[1]&&!foreign;++y){
   auto column=std::lower_bound(volume.columns.begin(),volume.columns.end(),std::array<int,2>{{lo[0],y}},
    [](const gfy_connectivity::Column&c,const std::array<int,2>&p){return gfy_connectivity::xyLess(c.x,c.y,p[0],p[1]);});
   for(;column!=volume.columns.end()&&column->y==y&&column->x<=hi[0]&&!foreign;++column)
    for(auto index=column->first;index<column->first+column->count;++index){const auto&span=volume.spans[index];
     if(span.hi<zlo||span.lo>zhi||span.component==owner)continue;
     if(retained.count(span.component)){foreign=true;break;}}
  }
  bool positive=false;
  for(int y=lo[1];y<=hi[1]&&!foreign&&!positive;++y){spend();
   auto row=bound_surface::clip(polygon,1,grid.origin.y+y*grid.pitch,1);
   row=bound_surface::clip(row,1,grid.origin.y+(y+1)*grid.pitch,-1);if(fraction(row)<1e-14)continue;
   auto column=std::lower_bound(volume.columns.begin(),volume.columns.end(),std::array<int,2>{{lo[0],y}},
    [](const gfy_connectivity::Column&c,const std::array<int,2>&p){return gfy_connectivity::xyLess(c.x,c.y,p[0],p[1]);});
   for(;column!=volume.columns.end()&&column->y==y&&column->x<=hi[0]&&!positive;++column){spend();
    bool ownerSpan=false;
    for(auto index=column->first;index<column->first+column->count;++index){const auto&span=volume.spans[index];
     if(span.component==owner&&!(flatZ&&(flatCell<span.lo||flatCell>=span.hi))){ownerSpan=true;break;}}
    if(!ownerSpan)continue;
    auto slab=bound_surface::clip(row,0,grid.origin.x+column->x*grid.pitch,1);
    slab=bound_surface::clip(slab,0,grid.origin.x+(column->x+1)*grid.pitch,-1);if(fraction(slab)<1e-14)continue;
    for(auto index=column->first;index<column->first+column->count&&!positive;++index){spend();const auto&span=volume.spans[index];
     if(span.component!=owner||(flatZ&&(flatCell<span.lo||flatCell>=span.hi)))continue;
     auto cut=bound_surface::clip(slab,2,grid.origin.z+span.lo*grid.pitch,1);
     cut=bound_surface::clip(cut,2,grid.origin.z+span.hi*grid.pitch,-1);
     positive=fraction(cut)>1e-14;
    }
   }
  }
  if(positive&&!coversRemovedAnatomy(polygon,bounds,grid,out.stats)){
   out.keep(owner,t);++out.stats.kept;++out.stats.output;out.stats.emitted_area+=originalArea;return;
  }
 }
 for(int y=lo[1];y<=hi[1];++y){spend();
  auto row=bound_surface::clip(polygon,1,grid.origin.y+y*grid.pitch,1);
  row=bound_surface::clip(row,1,grid.origin.y+(y+1)*grid.pitch,-1);if(fraction(row)<1e-14)continue;
  auto column=std::lower_bound(volume.columns.begin(),volume.columns.end(),std::array<int,2>{{lo[0],y}},
   [](const gfy_connectivity::Column&c,const std::array<int,2>&p){return gfy_connectivity::xyLess(c.x,c.y,p[0],p[1]);});
  for(;column!=volume.columns.end()&&column->y==y&&column->x<=hi[0];++column){spend();
   auto slab=bound_surface::clip(row,0,grid.origin.x+column->x*grid.pitch,1);
   slab=bound_surface::clip(slab,0,grid.origin.x+(column->x+1)*grid.pitch,-1);if(fraction(slab)<1e-14)continue;
   for(auto index=column->first;index<column->first+column->count;++index){spend();const auto&span=volume.spans[index];
    if(!retained.count(span.component)||(flatZ&&(flatCell<span.lo||flatCell>=span.hi)))continue;
    auto cut=bound_surface::clip(slab,2,grid.origin.z+span.lo*grid.pitch,1);
    cut=bound_surface::clip(cut,2,grid.origin.z+span.hi*grid.pitch,-1);
    if(fraction(cut)>1e-14)pieces[span.component].push_back(std::move(cut));
   }
  }
 }
 // Normal curved clothing triangles can bridge pristine air between all three
 // anchors. If every anchor and every covered material cell has one owner,
 // preserve the intact original triangle, including its existing curvature.
 if(pieces.size()==1&&cornerOwners[0]==pieces.begin()->first&&cornerOwners[1]==cornerOwners[0]&&cornerOwners[2]==cornerOwners[0]
    &&!coversRemovedAnatomy(polygon,bounds,grid,out.stats)){
  out.keep(cornerOwners[0],t);++out.stats.kept;++out.stats.output;out.stats.emitted_area+=originalArea;return;
 }
 // A triangle whose clipped pieces all belong to ONE retained owner and sum to its whole area (to clipping arithmetic
 // noise; 1e-10 is the merge code's own area-conservation tolerance) lies entirely over that owner's remaining tissue,
 // whatever its anchor cells read: a fragment cut on a cell plane has corners exactly on it (snapToCellPlanes), so
 // at() reads the cell beyond. The same removed-anatomy test as the rules above still has to pass. Nothing is split
 // or trimmed, so the intact triangle is the same geometry as its pieces.
 if(pieces.size()==1){double whole=0;for(const auto&p:pieces.begin()->second)whole+=fraction(p);
  if(whole>=1-1e-10&&!coversRemovedAnatomy(polygon,bounds,grid,out.stats)){
   out.keep(pieces.begin()->first,t);++out.stats.kept;++out.stats.output;out.stats.emitted_area+=originalArea;return;}}
 double emitted=0;
 double beforeMerge=0;for(const auto&entry:pieces)for(const auto&p:entry.second)beforeMerge+=fraction(p);
 for(auto&entry:pieces)entry.second=mergeConvexThreadLocal(std::move(entry.second)); // R66: = mergeConvex (same merges, same order)
 for(auto&entry:pieces)for(const auto&poly:entry.second){
  emitted+=fraction(poly);const auto ring=distinctRing(t,poly);
  for(std::size_t i=1;i+1<ring.size();++i){core::JoinedTriangle triangle;triangle.material=t.material;
   triangle.v={{ring[0],ring[i],ring[i+1]}};
   if(area(triangle)<1e-18||packedDegenerate(triangle))continue;
   need(out.stats.output<4000000,"source_partition_triangle_budget");
   out.stats.emitted_area+=area(triangle);++out.stats.output;out.meshes[entry.first].triangles.push_back(std::move(triangle));
  }
 }
 if(emitted>1.00001){std::ostringstream message;message<<std::setprecision(17)<<"source_partition_overlap fraction="<<emitted<<" before_merge="<<beforeMerge<<" source="<<t.v[0].sourceFace<<" material="<<t.material;
  for(auto p:anchors)message<<" anchor="<<p.x<<','<<p.y<<','<<p.z;throw std::runtime_error(message.str());}
 if(pieces.empty())++out.stats.trimmed;else ++out.stats.split;
 out.stats.trimmed_area+=originalArea*std::max(0.,1-emitted);
}
// joinFullBody (LiveDamageWorker.cpp) with core::joinIndexed, which the
// cavity-surface work verified field-identical to core::join, materialized
// back into JoinedTriangles in the same order. Untouched faces (every source
// face outside the selected patch) are interpolated on up to four threads
// into their final slots. Output is the same JoinResult.
inline core::JoinResult joinFullBodyMaterialized(const Asset&asset,const Grid&grid,const std::vector<core::Quad>&boundary){
 if(grid.ownership&&!grid.ownership->source_is_piece_local)
  throw RecoverableImpactError("piece_join_requires_piece_local_source");
 std::set<std::uint32_t> selected;
 for(const auto&tile:grid.tiles){auto c=tile.first;core::Bounds box;
  box.lo=grid.origin+Vec{double(c.x*8-2),double(c.y*8-2),double(c.z*8-2)}*grid.pitch;
  box.hi=grid.origin+Vec{double(c.x*8+10),double(c.y*8+10),double(c.z*8+10)}*grid.pitch;
  std::vector<std::uint32_t> faces;asset.source_tree.query(0,box,faces);selected.insert(faces.begin(),faces.end());
 }
 need(!selected.empty(),"fullbody_cut_without_exterior_patch");
 Source patch;patch.bones=asset.source.bones;patch.v=asset.source.v;
 std::vector<std::uint32_t> originals(selected.begin(),selected.end());patch.f.reserve(originals.size());
 for(auto face:originals)patch.f.push_back(asset.source.f[face]);
 const auto outside=core::exterior(patch,grid);
 auto indexed=core::joinIndexed(patch,asset.envelope_tree,grid,outside,boundary);
 core::JoinResult joined=std::move(indexed.stats);
 std::vector<std::uint32_t> untouched;untouched.reserve(asset.source.f.size()-selected.size());
 for(std::uint32_t f=0;f<asset.source.f.size();++f)if(!selected.count(f))untouched.push_back(f);
 joined.triangles.clear();joined.triangles.resize(indexed.triangles.size()+untouched.size());
 for(std::size_t i=0;i<indexed.triangles.size();++i){const auto&it=indexed.triangles[i];auto&t=joined.triangles[i];t.material=it.material;
  for(int k=0;k<3;++k)t.v[k]=*it.v[k];
  if(!t.material)for(auto&v:t.v){need(v.sourceFace<originals.size(),"fullbody_patch_provenance");v.sourceFace=originals[v.sourceFace];}}
 const std::size_t base=indexed.triangles.size();
 core::parallelChunks(untouched.size(),std::max<std::size_t>(1024,untouched.size()/4),[&](std::size_t first,std::size_t last,std::size_t){
  for(std::size_t i=first;i<last;++i){const auto f=untouched[i];auto&t=joined.triangles[base+i];
   for(int k=0;k<3;++k){core::V v;v.p=asset.source.v[asset.source.f[f][k]].p;v.b[k]=1;t.v[k]=core::interpolateVertex(asset.source,f,v);}}});
 return joined;
}
// The int14t union-path surface stage, VERBATIM (it was prepare()). It is the
// reference of the separation-track P2 rewrite below: GFY_RX_VERIFY builds run
// both and require identical pieces, stats, work counters and failures.
inline Packed prepareReference(const Asset&asset,const Grid&grid,const gfy_connectivity::Topology&volume,
                      const Bindings&bindings,double minVolume,const GarmentAdmission*garments=nullptr,const ExtraSurface*extra=nullptr,
                      const SurfaceOptions&options={}){
 (void)options;
 need(bindings.source==asset.source_hash&&bindings.vertices.size()==asset.source.v.size(),"source_partition_identity");
 need(bindings.coat_faces.empty()||garments,"coat_partition_missing_element_admission");
 std::set<std::uint32_t>retained;
 for(std::uint32_t i=1;i<volume.components.size();++i)if(volume.components[i].cells*grid.pitch*grid.pitch*grid.pitch>=minVolume)retained.insert(i);
 need(retained.size()>=2,"no_substantial_anatomy_separation");
 #ifdef GFY_COLLIDER_DIAGNOSTICS
 auto mark=[t0=core::Clock::now()](const char*n)mutable{std::cerr<<"surface_stage "<<n<<" ms="<<core::ms(t0)<<std::endl;t0=core::Clock::now();};
 #else
 auto mark=[](const char*){};
 #endif
 std::set<Cell>dirty;for(const auto&t:grid.tiles)dirty.insert(t.first);
 std::vector<core::Quad>boundary;
 if(!grid.ownership){
  // Same quads, same tile order as core::dirtyBoundary(grid,dirty): the exact
  // per-tile bit-parallel fastwound::tileBoundary, on up to four threads over
  // contiguous ranges of the ordered tile keys, concatenated in order; then
  // fastwound::mergeBoundaryRects (exact copy of core::mergeBoundaryQuads).
  const std::vector<Cell>keys(dirty.begin(),dirty.end());std::vector<std::vector<core::Quad>>parts(core::parallelWidth());
  core::parallelChunks(keys.size(),std::max<std::size_t>(64,keys.size()/4),[&](std::size_t first,std::size_t last,std::size_t chunk){
   for(std::size_t i=first;i<last;++i)fastwound::tileBoundary(grid,keys[i],parts[chunk]);});
  std::vector<core::Quad>quads;std::size_t total=0;for(auto&part:parts)total+=part.size();quads.reserve(total);
  for(auto&part:parts)quads.insert(quads.end(),part.begin(),part.end());
  std::vector<fastwound::RectMeta>meta;std::map<fastwound::PlaneKey,std::vector<int>>planeRows;
  boundary=fastwound::mergeBoundaryRects(grid,quads,meta,planeRows);
 }else boundary=core::mergeBoundaryQuads(grid,core::dirtyBoundary(grid,dirty));
 mark("boundary");auto joined=joinFullBodyMaterialized(asset,grid,boundary);mark("join");
#ifdef GFY_COLLIDER_DIAGNOSTICS
 std::cerr<<"parent_join new_open="<<joined.newOpen<<" nonmanifold="<<joined.nonmanifold<<" conflict="<<joined.maxJoinWeightConflict<<" original_open="<<joined.originalOpen<<std::endl;
#endif
 need(joined.newOpen<=8&&joined.nonmanifold<=8&&joined.maxJoinWeightConflict<1e-6,"bound_partition_parent_geometry_gate"); // a few defects from rough cut edges must not cancel a sever
 Result result;surface_support::Budget budget;budget.limit=160000000;
 // Triangles are partitioned independently (pure functions of read-only
 // data) on up to four threads, in blocks, each into its own Result; the
 // blocks are then replayed serially in triangle order: every stats counter,
 // every floating area accumulation (one addition per emitted triangle,
 // recomputed with area()), every budget and triangle-limit check and the
 // first exception occur in the same sequence as the former serial loop.
 // (Only the order of a budget overflow relative to another error INSIDE one
 // triangle can differ; both are failures.)
 struct Record{Result local;surface_support::Budget budget;std::exception_ptr error;};
 auto replay=[&](Record&r,const core::JoinedTriangle*source){
  budget.visited+=r.budget.visited;need(budget.visited<=budget.limit,"support_face_budget");
  result.stats.work+=r.local.stats.work;need(result.stats.work<=160000000,"source_partition_work_budget");
  if(r.error)std::rethrow_exception(r.error);
  const auto&l=r.local.stats;auto&o=result.stats;
  o.input+=l.input;o.original_area+=l.original_area;o.air_cavity+=l.air_cavity;o.air_cavity_area+=l.air_cavity_area;
  if(r.local.deferred){const auto&tri=*source; // kept: l.kept==1, no triangle-limit check (as before)
   o.emitted_area+=area(tri);++o.output;result.meshes[r.local.deferredOwner].triangles.push_back(tri);}
  for(auto&entry:r.local.meshes)for(auto&tri:entry.second.triangles){
   if(!l.kept)need(o.output<4000000,"source_partition_triangle_budget");
   o.emitted_area+=area(tri);++o.output;result.meshes[entry.first].triangles.push_back(std::move(tri));}
  o.kept+=l.kept;o.split+=l.split;o.trimmed+=l.trimmed;o.trimmed_area+=l.trimmed_area;
 };
 double rxCompute=0,rxReplay=0;(void)rxCompute;(void)rxReplay;
 // All triangles (joined, then accessory) are computed first (dynamic chunks
 // of 256 on up to four threads), then replayed serially in the former order.
 // The final per-owner counts are known before the replay, so each owner's
 // vector is reserved once and every triangle is copied exactly once.
 auto computeAll=[&](std::size_t count,const auto&one,bool defer){
  std::vector<Record>records(count);for(auto&r:records){r.budget.limit=budget.limit;r.local.deferKeep=defer;}
#ifdef GFY_RX_PROF
  auto t0=core::Clock::now();
#endif
  constexpr std::size_t chunk=256;const std::size_t chunks=(count+chunk-1)/chunk;std::atomic<std::size_t>cursor{0};
  core::parallelChunks(count>=1024?4:1,1,[&](std::size_t,std::size_t,std::size_t){
   for(std::size_t c;(c=cursor.fetch_add(1))<chunks;)for(std::size_t i=c*chunk;i<std::min(count,(c+1)*chunk);++i){auto&r=records[i];
    try{one(i,r.local,r.budget);}catch(...){r.error=std::current_exception();}}});
#ifdef GFY_RX_PROF
  rxCompute+=core::ms(t0);
#endif
  return records;
 };
 auto countInto=[](const std::vector<Record>&records,std::map<std::uint32_t,std::size_t>&counts){
  for(const auto&r:records){if(r.error)return false;if(r.local.deferred)++counts[r.local.deferredOwner];for(const auto&e:r.local.meshes)counts[e.first]+=e.second.triangles.size();}
  return true;};
 auto replayAll=[&](std::vector<Record>&records,const core::JoinedTriangle*sources){
#ifdef GFY_RX_PROF
  auto t0=core::Clock::now();
#endif
  for(std::size_t i=0;i<records.size();++i)replay(records[i],sources?sources+i:nullptr);
  records=std::vector<Record>();
#ifdef GFY_RX_PROF
  rxReplay+=core::ms(t0);
#endif
 };
 // Accessory triangles are precomputed only when the identity checks that
 // guard them (still evaluated at their former place below) will pass.
 const bool extraValid=extra&&extra->bindings.source==asset.source_hash&&extra->bindings.vertices.size()==extra->source.v.size()&&extra->source.bones==asset.source.bones
  &&extra->face_children.size()==extra->source.f.size()&&extra->face_indices.size()==extra->source.f.size();
 auto extraOne=[&](std::size_t f,Result&out,surface_support::Budget&){
   core::JoinedTriangle t;std::array<Vec,3>anchors;
   for(unsigned k=0;k<3;++k){core::V vertex;vertex.p=extra->source.v[extra->source.f[f][k]].p;vertex.b[k]=1;
    t.v[k]=core::interpolateVertex(extra->source,std::uint32_t(f),vertex);t.v[k].sourceFace+=std::uint32_t(asset.source.f.size());
    anchors[k]=extra->bindings.vertices[extra->source.f[f][k]].point;}
   partitionTriangle(t,anchors,grid,volume,retained,out);};
 auto joinedRecords=computeAll(joined.triangles.size(),[&](std::size_t index,Result&out,surface_support::Budget&localBudget){
  const auto&t=joined.triangles[index];
  if(t.material){
   auto contacts=surface_support::cavity(t,grid,volume,localBudget);
   std::set<std::uint32_t>owners;for(auto c:contacts)if(retained.count(c.first))owners.insert(c.first);
   if(owners.size()==1){out.keep(*owners.begin(),t);++out.stats.input;++out.stats.kept;++out.stats.output;
    out.stats.original_area+=area(t);out.stats.emitted_area+=area(t);return;}
   if(owners.empty()){++out.stats.air_cavity;out.stats.air_cavity_area+=area(t);return;}
   // Cavity surfaces sit exactly on cell boundaries: querying their inward
   // support needs the opposite normal half-cell offset.
   std::array<Vec,3>anchors;
   for(int k=0;k<3;++k){auto n=t.v[k].normal;need(length(n)>1e-10,"partition_cavity_normal");anchors[k]=t.v[k].p-n*(grid.pitch*.5/length(n));}
   partitionTriangle(t,anchors,grid,volume,retained,out);return;
  }
  const auto face=t.v[0].sourceFace;need(face<asset.source.f.size(),"source_partition_face");
  std::array<Vec,3>anchors;
  for(int k=0;k<3;++k){need(t.v[k].sourceFace==face,"source_partition_mixed_provenance");double sum=0;
   for(int j=0;j<3;++j){const auto b=t.v[k].sourceBary[j];need(std::isfinite(b)&&b>=-1e-6&&b<=1.000001,"source_partition_barycentric");sum+=b;
    anchors[k]=anchors[k]+bindings.vertices[asset.source.f[face][j]].point*b;}
   need(std::abs(sum-1)<1e-6,"source_partition_barycentric_sum");}
  partitionTriangle(t,anchors,grid,volume,retained,out);
 },true);
 std::vector<Record>extraRecords;if(extraValid)extraRecords=computeAll(extra->source.f.size(),extraOne,false);
 {std::map<std::uint32_t,std::size_t>counts;if(countInto(joinedRecords,counts)&&countInto(extraRecords,counts))
   for(const auto&c:counts)if(c.second)result.meshes[c.first].triangles.reserve(c.second);}
 replayAll(joinedRecords,joined.triangles.data());
#ifdef GFY_RX_PROF
 std::cerr<<"rxprof partition compute="<<rxCompute<<" replay="<<rxReplay<<" triangles="<<joined.triangles.size()<<std::endl;
#endif
 mark("partition");auto faceChildren=asset.face_children;auto faceIndices=asset.face_indices;
 if(extra){
  need(extra->bindings.source==asset.source_hash&&extra->bindings.vertices.size()==extra->source.v.size()&&extra->source.bones==asset.source.bones,"accessory_partition_identity");
  need(extra->face_children.size()==extra->source.f.size()&&extra->face_indices.size()==extra->source.f.size(),"accessory_partition_provenance");
  replayAll(extraRecords,nullptr);
  faceChildren.insert(faceChildren.end(),extra->face_children.begin(),extra->face_children.end());
  faceIndices.insert(faceIndices.end(),extra->face_indices.begin(),extra->face_indices.end());
 }
 // Coats have no independent cloth solver. Waist ownership cannot leave a
 // panel skinned to a calf on another rigid body. Remove only those panels
 // during separation; ordinary wound generation never calls this stage.
 mark("extra");if(!bindings.coat_faces.empty())trimUnsupportedCoat(result,bindings,*garments);
 Packed packed;packed.stats=result.stats;packed.garment_roots_validated=!bindings.coat_faces.empty()&&garments;
 // Pieces pack independently (up to four threads, contiguous in map order;
 // the first failure in piece order is rethrown, as in the serial loop).
 std::vector<std::pair<const std::uint32_t,core::JoinResult>*>entries;for(auto&entry:result.meshes)entries.push_back(&entry);
 packed.pieces.resize(entries.size());
 if(const char*prefix=std::getenv("GFY_BOUND_PIECE_DUMP"))for(auto*entry:entries)core::writeJoined(std::string(prefix)+"-"+std::to_string(entry->first)+".joined",entry->second,asset.source.bones);
 core::parallelChunks(entries.size(),1,[&](std::size_t first,std::size_t last,std::size_t){
  for(std::size_t i=first;i<last;++i){auto&p=packed.pieces[i];const auto&entry=*entries[i];p.volume_component=entry.first;p.triangles=entry.second.triangles.size();
   p.mesh=core::packing::packMulti(entry.second,faceChildren,faceIndices,asset.original_children);}});
 mark("pack");for(auto id:retained)need(result.meshes.count(id)!=0,"bound_partition_missing_piece");
 need(std::abs(result.stats.original_area-result.stats.emitted_area-result.stats.trimmed_area)<1e-6,"bound_partition_area_conservation");
 return packed;
}
// ============================================================================
// Separation-track P2: exact rewrite of the union-path surface stage.
// Everything above this line is the int14t implementation (prepareReference()
// runs it verbatim). prepare() below returns byte-identical pieces and bitwise
// identical Stats, spends the same work / support-face budget in the same
// replay order, and fails with the same exception (type and message) at the
// same point. GFY_RX_VERIFY builds check all of that on every call.
// ============================================================================

// ---- S1: zero-copy triangle universe of one union surface stage ------------
// A triangle reference (uint32) addresses:
//   [0,base)        join triangles, part by part (B1: one part per region join;
//                   the union path: the one union join), corner pointers into
//                   the part's vertex store,
//   [base,joined)   untouched source faces (B1: the passive faces first, then
//                   the unselected ones; the union path: the unselected faces),
//                   each group ascending,
//   [joined,split)  accessory faces (ExtraSurface), in face order,
//   [split,...)     triangles emitted by splitting (owned by SplitOut records).
// The stage replays the references [0,joined) in the former joined order (an
// explicit order, see prepareFast), so nothing downstream depends on the part
// layout. Nothing is materialized as a 1976-byte JoinedTriangle except the
// input and output of a split. Every accessor returns the value the former
// materialized JoinedTriangle held: join corners, with sourceFace remapped
// through the part's patch originals for material 0 only; untouched and
// accessory corners = core::interpolateVertex(source,face,{p,e_k}) (accessory
// sourceFace offset by the asset face count).
struct JoinPart {
 const core::IndexedJoin*indexed{};const std::vector<std::uint32_t>*originals{};
 std::uint32_t triBase{},vertexBase{}; // first reference / first vertex id of the part
 std::unordered_map<const core::JoinedVertex*,std::uint32_t> extraIds; // part-local ids of IndexedJoin::extra (deque) vertices
};
struct UnionTriangles {
 const Asset*asset{};const ExtraSurface*extra{};
 std::vector<JoinPart> parts;std::vector<std::uint32_t> triStarts,vertexStarts; // parts.size()+1 entries each
 const std::vector<std::uint32_t>*untouched{};
 std::uint32_t base{},joined{},splitBase{},vertexTotal{};
 std::vector<const core::JoinedTriangle*> splits;
 UnionTriangles(const Asset&a,const ExtraSurface*e,bool extraValid,std::vector<JoinPart>&&p,const std::vector<std::uint32_t>&u)
  :asset(&a),extra(e),parts(std::move(p)),untouched(&u){
  std::uint32_t t=0,v=0;
  for(auto&part:parts){part.triBase=t;part.vertexBase=v;triStarts.push_back(t);vertexStarts.push_back(v);
   t+=std::uint32_t(part.indexed->triangles.size());const auto first=std::uint32_t(part.indexed->vertices.size());
   for(std::size_t i=0;i<part.indexed->extra.size();++i)part.extraIds.emplace(&part.indexed->extra[i],first+std::uint32_t(i));
   v+=first+std::uint32_t(part.indexed->extra.size());}
  triStarts.push_back(t);vertexStarts.push_back(v);
  base=t;vertexTotal=v;joined=base+std::uint32_t(u.size());
  splitBase=joined+std::uint32_t(extraValid?e->source.f.size():0);
 }
 static const std::array<double,3>&unit(int k){static const std::array<std::array<double,3>,3>e{{{{1.,0.,0.}},{{0.,1.,0.}},{{0.,0.,1.}}}};return e[std::size_t(k)];}
 std::uint32_t vertexCount()const{return vertexTotal;}
 const JoinPart&partOfRef(std::uint32_t r)const{const auto it=std::upper_bound(triStarts.begin(),triStarts.end(),r);return parts[std::size_t(it-triStarts.begin())-1];}
 const JoinPart&partOfVertex(std::uint32_t id)const{const auto it=std::upper_bound(vertexStarts.begin(),vertexStarts.end(),id);return parts[std::size_t(it-vertexStarts.begin())-1];}
 const core::IndexedTriangle&joinTriangle(std::uint32_t r)const{const auto&part=partOfRef(r);return part.indexed->triangles[r-part.triBase];}
 // Global vertex id of a join corner of `part`.
 std::uint32_t vertexId(const JoinPart&part,const core::JoinedVertex*v)const{
  const auto at=reinterpret_cast<std::uintptr_t>(v),first=reinterpret_cast<std::uintptr_t>(part.indexed->vertices.data());
  if(at>=first&&at<first+part.indexed->vertices.size()*sizeof(core::JoinedVertex))return part.vertexBase+std::uint32_t((at-first)/sizeof(core::JoinedVertex));
  return part.vertexBase+part.extraIds.at(v);}
 std::uint32_t vertexId(std::uint32_t r,int k)const{const auto&part=partOfRef(r);return vertexId(part,part.indexed->triangles[r-part.triBase].v[std::size_t(k)]);}
 const core::JoinedVertex&vertexOf(std::uint32_t id)const{const auto&part=partOfVertex(id);const auto local=id-part.vertexBase;
  return local<part.indexed->vertices.size()?part.indexed->vertices[local]:part.indexed->extra[local-part.indexed->vertices.size()];}
 unsigned material(std::uint32_t r)const{return r<base?joinTriangle(r).material:r<splitBase?0u:splits[r-splitBase]->material;}
 std::uint32_t sourceFace(std::uint32_t r,int k)const{
  if(r<base){const auto&part=partOfRef(r);const auto&t=part.indexed->triangles[r-part.triBase];const auto raw=t.v[std::size_t(k)]->sourceFace;return t.material?raw:(*part.originals)[raw];}
  if(r<joined)return (*untouched)[r-base];
  if(r<splitBase)return std::uint32_t(r-joined)+std::uint32_t(asset->source.f.size());
  return splits[r-splitBase]->v[std::size_t(k)].sourceFace;}
 const std::array<double,3>&sourceBary(std::uint32_t r,int k)const{
  if(r<base)return joinTriangle(r).v[std::size_t(k)]->sourceBary;
  if(r<splitBase)return unit(k);
  return splits[r-splitBase]->v[std::size_t(k)].sourceBary;}
 const Vec&p(std::uint32_t r,int k)const{
  if(r<base)return joinTriangle(r).v[std::size_t(k)]->p;
  if(r<joined){const auto f=(*untouched)[r-base];return asset->source.v[asset->source.f[f][std::size_t(k)]].p;}
  if(r<splitBase){const auto f=r-joined;return extra->source.v[extra->source.f[f][std::size_t(k)]].p;}
  return splits[r-splitBase]->v[std::size_t(k)].p;}
 // area(JoinedTriangle) of the former materialized triangle.
 double area(std::uint32_t r)const{const Vec&a=p(r,0);return length(cross(p(r,1)-a,p(r,2)-a))*.5;}
 core::JoinedVertex corner(std::uint32_t r,int k)const{
  const auto c=std::size_t(k);
  if(r<base){const auto&part=partOfRef(r);const auto&t=part.indexed->triangles[r-part.triBase];core::JoinedVertex v=*t.v[c];if(!t.material)v.sourceFace=(*part.originals)[v.sourceFace];return v;}
  if(r<joined){const auto f=(*untouched)[r-base];core::V v;v.p=asset->source.v[asset->source.f[f][c]].p;v.b[c]=1;return core::interpolateVertex(asset->source,f,v);}
  if(r<splitBase){const auto f=std::uint32_t(r-joined);core::V v;v.p=extra->source.v[extra->source.f[f][c]].p;v.b[c]=1;
   auto out=core::interpolateVertex(extra->source,f,v);out.sourceFace+=std::uint32_t(asset->source.f.size());return out;}
  return splits[r-splitBase]->v[c];}
 void triangle(std::uint32_t r,core::JoinedTriangle&t)const{t.material=material(r);for(int k=0;k<3;++k)t.v[std::size_t(k)]=corner(r,k);}
 // Skin weights (JoinedVertex::w) of corner k of the former materialized triangle.
 const std::array<double,64>&weights(std::uint32_t r,int k,std::array<double,64>&scratch)const{
  const auto c=std::size_t(k);
  if(r<base)return joinTriangle(r).v[c]->w;
  if(r<joined){const auto f=(*untouched)[r-base];core::V v;v.p=asset->source.v[asset->source.f[f][c]].p;v.b[c]=1;scratch=core::weights(asset->source,f,v);return scratch;}
  if(r<splitBase){const auto f=std::uint32_t(r-joined);core::V v;v.p=extra->source.v[extra->source.f[f][c]].p;v.b[c]=1;scratch=core::weights(extra->source,f,v);return scratch;}
  return splits[r-splitBase]->v[c].w;}
};

struct SplitOut {std::vector<std::pair<std::uint32_t,core::JoinedTriangle>> triangles;std::size_t split{},trimmed{};double trimmed_area{};};
struct Outcome {
 enum Kind:std::uint8_t{unset,kept,air,split};
 surface_support::Budget budget;std::uint64_t work{};double area{};std::uint32_t owner{};Kind kind{unset};
 std::unique_ptr<SplitOut> side;std::exception_ptr error;
#ifdef GFY_RX_PROF
 // S0 path class: 1 early-accept kept, 2 full-reference kept, 3 split, 4 trimmed,
 // 5 cavity single owner, 6 cavity air, 7 cavity via the reference cavity().
 std::uint8_t path{};float ms{};
#endif
};
// ---- S3: allocation-free partition of one triangle -------------------------
// bound_surface::clip() into a caller-owned buffer (cleared first): the same
// arithmetic, dedupe, closing and emptiness rules. out must not alias p.
inline void clipTo(const core::Poly&p,int axis,double bound,double sign,core::Poly&out){
 out.clear();if(p.empty())return;
 auto same=[](const core::V&a,const core::V&b){double d=0;for(int i=0;i<3;++i)d=std::max(d,std::abs(a.b[i]-b.b[i]));return d<1e-14;};
 auto append=[&](const core::V&v){if(out.empty()||!same(out.back(),v))out.push_back(v);};
 for(std::size_t i=0;i<p.size();++i){const auto&a=p[i];const auto&b=p[(i+1)%p.size()];
  const auto da=(a.p[axis]-bound)*sign,db=(b.p[axis]-bound)*sign;const bool ia=da>=0,ib=db>=0;
  if(ia)append(a);
  if(ia!=ib){const auto u=std::clamp(da/(da-db),0.,1.);core::V v;v.p=a.p+(b.p-a.p)*u;v.p[axis]=bound;
   for(int k=0;k<3;++k)v.b[k]=a.b[k]+(b.b[k]-a.b[k])*u;append(v);}
 }
 if(out.size()>1&&same(out.front(),out.back()))out.pop_back();
 if(out.size()<3||fraction(out)<1e-14)out.clear();
}
// mergeConvex() with its std::map<Edge,vector<Use>> (only ever probed with
// find()/operator[], never iterated) replaced by an open-addressed table whose
// per-edge use lists keep insertion order, and its per-candidate std::set of
// keys replaced by facts kept per polygon (R66, below). Same merges in the same order.
struct MergeScratch {
 using Key=std::array<std::int64_t,2>;using Edge=std::array<Key,2>;
 struct Use {std::size_t polygon{},edge{};unsigned version{};};
 struct Slot {Edge key{};std::uint32_t head{UINT32_MAX},tail{UINT32_MAX};};
 struct Node {Use use;std::uint32_t next{UINT32_MAX};};
 std::vector<Slot> slots;std::vector<Node> nodes;std::size_t used{},mask{};core::Poly candidate;std::vector<Key> keys;
 // R66: per polygon of one call: sorted keys of large polygons, distinct / verified (-1 unknown), cached fraction.
 std::vector<std::vector<Key>> sorted;std::vector<std::int8_t> distinct,verified,areaKnown;std::vector<double> areas;
 static std::size_t hash(const Edge&e){std::uint64_t h=0x243F6A8885A308D3ull;
  for(const auto&k:e)for(const auto x:k){h^=std::uint64_t(x);h*=0x9E3779B97F4A7C15ull;h^=h>>29;}return std::size_t(h^(h>>32));}
 void reset(std::size_t expected){std::size_t n=64;while(n<2*expected)n*=2;slots.assign(n,Slot{});nodes.clear();used=0;mask=n-1;}
 std::uint32_t find(const Edge&e)const{
  for(std::size_t at=hash(e)&mask;;at=(at+1)&mask){const auto&s=slots[at];if(s.head==UINT32_MAX)return UINT32_MAX;if(s.key==e)return s.head;}}
 void push(const Edge&e,const Use&u){
  for(std::size_t at=hash(e)&mask;;at=(at+1)&mask){auto&s=slots[at];
   if(s.head==UINT32_MAX){s.key=e;s.head=s.tail=std::uint32_t(nodes.size());nodes.push_back({u,UINT32_MAX});if(2*++used>slots.size())grow();return;}
   if(s.key==e){nodes[s.tail].next=std::uint32_t(nodes.size());s.tail=std::uint32_t(nodes.size());nodes.push_back({u,UINT32_MAX});return;}}}
 void grow(){std::vector<Slot> old;old.swap(slots);slots.assign(old.size()*2,Slot{});mask=slots.size()-1;
  for(const auto&s:old)if(s.head!=UINT32_MAX){std::size_t at=hash(s.key)&mask;while(slots[at].head!=UINT32_MAX)at=(at+1)&mask;slots[at]=s;}}
};
inline std::vector<core::Poly> mergeConvexFast(std::vector<core::Poly>input,MergeScratch&m){
 using Key=MergeScratch::Key;using Edge=MergeScratch::Edge;
 auto key=[](const core::V&v){return Key{{std::llround(v.b[1]*1e11),std::llround(v.b[2]*1e11)}};};
 std::size_t edgeCount=0;for(const auto&p:input)edgeCount+=p.size();m.reset(edgeCount);
 const std::size_t count=input.size();
 std::vector<unsigned>versions(count);
 std::vector<bool>alive(count,true);std::deque<std::size_t>pending;
 for(std::size_t i=0;i<count;++i)pending.push_back(i);
 // R66 facts (see the file header of patch_merge66): exact restatements of the full per-candidate tests.
 static constexpr std::size_t large=32; // polygons with at least this many corners keep their keys sorted
 auto&sorted=m.sorted;auto&distinct=m.distinct;auto&verified=m.verified;auto&areaKnown=m.areaKnown;auto&areas=m.areas;
 if(sorted.size()<count)sorted.resize(count);
 distinct.assign(count,-1);verified.assign(count,-1);areaKnown.assign(count,0);areas.assign(count,0.);
 auto sortedKeys=[&](const core::Poly&p,std::vector<Key>&k){k.clear();k.reserve(p.size());for(const auto&v:p)k.push_back(key(v));std::sort(k.begin(),k.end());};
 auto isDistinct=[&](std::size_t id){auto&d=distinct[id];
  if(d<0){const auto&p=input[id];
   if(p.size()>=large){auto&k=sorted[id];sortedKeys(p,k);d=std::adjacent_find(k.begin(),k.end())==k.end()?1:0;}
   else{Key k[large];std::size_t n=0;for(const auto&v:p)k[n++]=key(v);std::sort(k,k+n);d=std::adjacent_find(k,k+n)==k+n?1:0;}}
  return d==1;};
 // Is k one of polygon id's keys? A large polygon: its sorted keys (built by isDistinct or when it was made); a small one: a scan.
 auto hasKey=[&](std::size_t id,const Key&k){const auto&p=input[id];
  if(p.size()>=large){const auto&s=sorted[id];return std::binary_search(s.begin(),s.end(),k);}
  for(const auto&v:p)if(key(v)==k)return true;
  return false;};
 auto fails=[](const core::V&a,const core::V&b,const core::V&c){return crossBary(a,b,c)<-1e-13;};
 auto isVerified=[&](std::size_t id){auto&f=verified[id];
  if(f<0){const auto&p=input[id];const std::size_t n=p.size();bool ok=true;
   for(std::size_t k=0;k<n&&ok;++k)for(std::size_t v=0;v<n;++v)if(fails(p[k],p[(k+1)%n],p[v])){ok=false;break;}
   f=ok?1:0;}
  return f==1;};
 auto areaOf=[&](std::size_t id){if(!areaKnown[id]){areas[id]=fraction(input[id]);areaKnown[id]=1;}return areas[id];};
 std::size_t work=0;
 while(!pending.empty()){const auto id=pending.front();pending.pop_front();if(!alive[id])continue;
  bool merged=false;
  for(std::size_t i=0;i<input[id].size()&&!merged;++i){
   need(++work<2000000,"source_partition_polygon_merge_budget");auto&p=input[id];
   const Edge edge{{key(p[i]),key(p[(i+1)%p.size()])}},reverse{{edge[1],edge[0]}};
   for(auto node=m.find(reverse);node!=UINT32_MAX;node=m.nodes[node].next){const auto use=m.nodes[node].use;
    if(use.polygon==id||!alive[use.polygon]||versions[use.polygon]!=use.version)continue;
    const auto&q=input[use.polygon];const std::size_t P=p.size(),Q=q.size(),N=P+Q-2,e=use.edge;
    // The candidate ring, by index: p[i+1] .. p[i], then q[e+2] .. q[e-1].
    auto at=[&](std::size_t j)->const core::V&{return j<P?p[(i+1+j)%P]:q[(e+2+(j-P))%Q];};
    // Duplicate keys.
    bool convex=isDistinct(id)&&isDistinct(use.polygon);
    if(convex){
     if(Q-2<=P){for(std::size_t t=2;t<Q;++t)if(hasKey(id,key(q[(e+t)%Q]))){convex=false;break;}}
     else{const std::size_t i1=(i+1)%P;
      for(std::size_t j=0;j<P;++j){if(j==i||j==i1)continue;if(hasKey(use.polygon,key(p[j]))){convex=false;break;}}}}
    // Local turns alone also admit a self-intersecting star. Every vertex
    // must be on the interior side of every edge before we fan-triangulate.
    if(convex){
     if(!isVerified(id)||!isVerified(use.polygon)){
      for(std::size_t k=0;k<N&&convex;++k){const auto&a=at(k),&b=at((k+1)%N);for(std::size_t v=0;v<N;++v)if(fails(a,b,at(v))){convex=false;break;}}}
     else{
      for(std::size_t k=0;k+1<P&&convex;++k){const auto&a=at(k),&b=at(k+1);for(std::size_t v=P;v<N;++v)if(fails(a,b,at(v))){convex=false;break;}}
      for(std::size_t k=P;k+1<N&&convex;++k){const auto&a=at(k),&b=at(k+1);for(std::size_t v=0;v<P;++v)if(fails(a,b,at(v))){convex=false;break;}}
      for(const std::size_t k:{P-1,N-1}){if(!convex)break;const auto&a=at(k),&b=at((k+1)%N);for(std::size_t v=0;v<N;++v)if(fails(a,b,at(v))){convex=false;break;}}}}
    if(!convex)continue;
    double whole=0;{const auto&a=at(0).b;for(std::size_t j=1;j+1<N;++j){const auto&b=at(j).b;const auto&c=at(j+1).b;
      whole+=std::abs((b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1]));}}
    if(std::abs(whole-areaOf(id)-areaOf(use.polygon))>1e-10)continue;
    auto&candidate=m.candidate;candidate.clear();candidate.reserve(N);for(std::size_t j=0;j<N;++j)candidate.push_back(at(j));
    if(N>=large){std::vector<Key> a,b;const std::vector<Key>*pa=&sorted[id],*pb=&sorted[use.polygon];
     if(P<large){sortedKeys(p,a);pa=&a;}if(Q<large){sortedKeys(q,b);pb=&b;}
     auto&k=m.keys;k.clear();k.reserve(pa->size()+pb->size());std::set_union(pa->begin(),pa->end(),pb->begin(),pb->end(),std::back_inserter(k));sorted[id].swap(k);
     if(Q>=large)std::vector<Key>().swap(sorted[use.polygon]);} // the merged-away polygon's keys are not read again
    else sorted[id].clear();
    distinct[id]=1;verified[id]=1;areas[id]=whole;areaKnown[id]=1;
    alive[use.polygon]=false;input[id].swap(candidate);++versions[id];pending.push_back(id);merged=true;break;
   }
   if(!merged)m.push(edge,{id,i,versions[id]});
  }
 }
 std::vector<core::Poly>result;
 for(std::size_t i=0;i<count;++i)if(alive[i]){auto p=std::move(input[i]);const double originalFraction=fraction(p);
  std::size_t from=0,skipLo=1,skipHi=0;bool changed=true;
  while(changed&&p.size()>3){changed=false;
   for(std::size_t k=from;k<p.size();++k){
    if(skipLo<=skipHi&&k>=skipLo&&k<=skipHi){k=skipHi;continue;}
    if(std::abs(crossBary(p[(k+p.size()-1)%p.size()],p[k],p[(k+1)%p.size()]))<1e-13){
     const std::size_t n=p.size();p.erase(p.begin()+std::ptrdiff_t(k));changed=true;skipLo=1;skipHi=0;
     if(k==0)from=0;
     else if(k+1==n){from=0;if(p.size()>=3){skipLo=1;skipHi=p.size()-2;}}
     else from=k-1;
     break;}}}
  need(std::abs(fraction(p)-originalFraction)<1e-10,"source_partition_polygon_cleanup_area");
  result.push_back(std::move(p));}
 if(count>2048){std::vector<std::vector<Key>>().swap(sorted);std::vector<std::int8_t>().swap(distinct);std::vector<std::int8_t>().swap(verified);
  std::vector<std::int8_t>().swap(areaKnown);std::vector<double>().swap(areas);std::vector<Key>().swap(m.keys);core::Poly().swap(m.candidate);}
 return result;
}
inline std::vector<core::Poly> mergeConvexThreadLocal(std::vector<core::Poly>input){thread_local MergeScratch scratch;return mergeConvexFast(std::move(input),scratch);}
#ifdef GFY_RX_PROF
// Early-accept section profile (profile builds only): ms and counters per section.
struct EarlyProfile {double ms[6]{};std::uint64_t n[6]{};};
inline EarlyProfile&earlyProfile(){thread_local EarlyProfile p;return p;}
inline std::vector<EarlyProfile*>&earlyProfiles(){static std::vector<EarlyProfile*> all;return all;}
#define GFY_EARLY_T0 auto rxE=core::Clock::now();
#define GFY_EARLY_MARK(SECTION,AMOUNT) {auto now=core::Clock::now();earlyProfile().ms[SECTION]+=std::chrono::duration<double,std::milli>(now-rxE).count();earlyProfile().n[SECTION]+=(AMOUNT);rxE=now;}
#else
#define GFY_EARLY_T0
#define GFY_EARLY_MARK(SECTION,AMOUNT)
#endif
// Read-only lookups of one surface stage and one worker's scratch buffers.
struct PartitionContext {
 const Grid&grid;const gfy_connectivity::Topology&volume;const surface_support::ColumnRows&rows;const surface_support::TileIndex&tiles;
 const std::vector<std::uint8_t>&retained;
 const std::vector<std::uint32_t>&retainedIds;
 // A foreign span can only be in its component's cell bounds. This rejects
 // impossible candidates before the exact column scan; its work counter is
 // unchanged because the original foreign scan does not consume that budget.
 bool foreignMayOverlap(std::uint32_t owner,const int*lo,const int*hi,int zlo,int zhi)const{
  for(auto id:retainedIds){if(id==owner)continue;const auto&c=volume.components[id];
   if(c.lo[0]>hi[0]||c.hi[0]<=lo[0]||c.lo[1]>hi[1]||c.hi[1]<=lo[1]||c.lo[2]>zhi||c.hi[2]<zlo)continue;
   return true;}
  return false;
 }
 bool isRetained(std::uint32_t c)const{return c<retained.size()&&retained[c]!=0;} // == std::set::count
 // bound_surface::at()
 std::uint32_t at(const Vec&p)const{return rows.at(int(std::floor((p.x-grid.origin.x)/grid.pitch)),
                                                   int(std::floor((p.y-grid.origin.y)/grid.pitch)),
                                                   int(std::floor((p.z-grid.origin.z)/grid.pitch)));}
};
struct PartitionScratch {core::Poly polygon,rowA,row,slabA,slab,cutA,cut,coverA,coverB;MergeScratch merge;};
// coversRemovedAnatomy(): the same tile and cell loops, work counts and clips.
// A tile's existence comes from the stage's tile bitset, and removed(c) of a
// cell inside that tile is its bit (Grid::removed without ownership: every
// visited cell is valid and lies in that tile); owned grids call removed().
inline bool coversRemovedAnatomyFast(const core::Poly&polygon,const core::Bounds&bounds,const PartitionContext&ctx,std::uint64_t&work,PartitionScratch&s){
 const auto&grid=ctx.grid;
 Cell lo,hi;for(int a=0;a<3;++a){
  (&lo.x)[a]=std::max(0,int(std::floor((bounds.lo[a]-grid.origin[a])/grid.pitch)));
  (&hi.x)[a]=std::min(grid.size[a]-1,int(std::floor((bounds.hi[a]-grid.origin[a])/grid.pitch)));}
 for(int tx=lo.x/8;tx<=hi.x/8;++tx)for(int ty=lo.y/8;ty<=hi.y/8;++ty)for(int tz=lo.z/8;tz<=hi.z/8;++tz){
  need(++work<=160000000,"source_partition_work_budget");
  const core::Tile*tile=ctx.tiles.find({tx,ty,tz});if(!tile)continue;
  for(int x=std::max(lo.x,tx*8);x<=std::min(hi.x,tx*8+7);++x)
   for(int y=std::max(lo.y,ty*8);y<=std::min(hi.y,ty*8+7);++y)
    for(int z=std::max(lo.z,tz*8);z<=std::min(hi.z,tz*8+7);++z){
     need(++work<=160000000,"source_partition_work_budget");Cell c{x,y,z};
     const bool removed=grid.ownership?grid.removed(c):core::tileBit(*tile,c);
     if(!removed||!grid.solidBase(c))continue;
     const core::Poly*clipped=&polygon;
     for(int a=0;a<3&&!clipped->empty();++a){clipTo(*clipped,a,grid.origin[a]+c[a]*grid.pitch,1,s.coverA);
      clipTo(s.coverA,a,grid.origin[a]+(c[a]+1)*grid.pitch,-1,s.coverB);clipped=&s.coverB;}
     if(fraction(*clipped)>1e-14)return true;
    }
 }
 return false;
}
// partitionTriangle() recording an Outcome, with the row index, scratch clip
// buffers, a sorted small vector for the per-owner pieces (std::map order),
// the tile-bit coverage test and mergeConvexFast(). Every loop, clip, work
// count, check and emission is the reference's, in its order. The full
// triangle is materialized only to split (full()).
template<class Full>
inline void partitionFast(const std::array<Vec,3>&anchors,double originalArea,const PartitionContext&ctx,PartitionScratch&s,Outcome&out,const Full&full){
 GFY_EARLY_T0
 const auto&grid=ctx.grid;const auto&volume=ctx.volume;const auto*end=ctx.rows.end();
 auto&polygon=s.polygon;polygon.clear();core::Bounds bounds;std::array<std::uint32_t,3> cornerOwners{};
 for(int k=0;k<3;++k){core::V v;v.p=snapToCellPlanes(anchors[k],grid);v.b[k]=1;polygon.push_back(v);bounds.add(v.p);cornerOwners[k]=ctx.at(v.p);}
 out.area=originalArea;
 std::vector<std::pair<std::uint32_t,std::vector<core::Poly>>> pieces; // ascending owner, like std::map
 auto spend=[&](){need(++out.work<=160000000,"source_partition_work_budget");};
 int lo[2],hi[2];
 for(int a=0;a<2;++a){lo[a]=std::max(0,int(std::floor((bounds.lo[a]-grid.origin[a])/grid.pitch)));
  hi[a]=std::min(grid.size[a]-1,int(std::floor((bounds.hi[a]-grid.origin[a])/grid.pitch)));}
 const bool flatZ=bounds.hi.z-bounds.lo.z<1e-12;
 const int flatCell=int(std::floor((bounds.lo.z-grid.origin.z)/grid.pitch));
 GFY_EARLY_MARK(0,1)
 // Exact early acceptance of an intact single-owner triangle (see partitionTriangle).
 if(cornerOwners[0]==cornerOwners[1]&&cornerOwners[1]==cornerOwners[2]&&ctx.isRetained(cornerOwners[0])){
  const auto owner=cornerOwners[0];
  const int zlo=int(std::floor((bounds.lo.z-grid.origin.z)/grid.pitch))-1,zhi=int(std::floor((bounds.hi.z-grid.origin.z)/grid.pitch))+1;
  bool foreign=false;
  if(ctx.foreignMayOverlap(owner,lo,hi,zlo,zhi))
  for(int y=lo[1];y<=hi[1]&&!foreign;++y)
   for(auto column=ctx.rows.lower(lo[0],y);column!=end&&column->y==y&&column->x<=hi[0]&&!foreign;++column)
    for(auto index=column->first;index<column->first+column->count;++index){const auto&span=volume.spans[index];
     if(span.hi<zlo||span.lo>zhi||span.component==owner)continue;
     if(ctx.isRetained(span.component)){foreign=true;break;}}
  GFY_EARLY_MARK(1,std::uint64_t(hi[1]-lo[1]+1))
  bool positive=false;
  for(int y=lo[1];y<=hi[1]&&!foreign&&!positive;++y){spend();
   clipTo(polygon,1,grid.origin.y+y*grid.pitch,1,s.rowA);
   clipTo(s.rowA,1,grid.origin.y+(y+1)*grid.pitch,-1,s.row);if(fraction(s.row)<1e-14)continue;
   for(auto column=ctx.rows.lower(lo[0],y);column!=end&&column->y==y&&column->x<=hi[0]&&!positive;++column){spend();
    bool ownerSpan=false;
    for(auto index=column->first;index<column->first+column->count;++index){const auto&span=volume.spans[index];
     if(span.component==owner&&!(flatZ&&(flatCell<span.lo||flatCell>=span.hi))){ownerSpan=true;break;}}
    if(!ownerSpan)continue;
    clipTo(s.row,0,grid.origin.x+column->x*grid.pitch,1,s.slabA);
    clipTo(s.slabA,0,grid.origin.x+(column->x+1)*grid.pitch,-1,s.slab);if(fraction(s.slab)<1e-14)continue;
    for(auto index=column->first;index<column->first+column->count&&!positive;++index){spend();const auto&span=volume.spans[index];
     if(span.component!=owner||(flatZ&&(flatCell<span.lo||flatCell>=span.hi)))continue;
     clipTo(s.slab,2,grid.origin.z+span.lo*grid.pitch,1,s.cutA);
     clipTo(s.cutA,2,grid.origin.z+span.hi*grid.pitch,-1,s.cut);
     positive=fraction(s.cut)>1e-14;
    }
   }
  }
  GFY_EARLY_MARK(2,1)
  const bool covered=positive&&coversRemovedAnatomyFast(polygon,bounds,ctx,out.work,s);
  GFY_EARLY_MARK(3,positive)
  if(positive&&!covered){
#ifdef GFY_RX_PROF
   out.path=1;
#endif
   out.kind=Outcome::kept;out.owner=owner;return;
  }
 }
 for(int y=lo[1];y<=hi[1];++y){spend();
  clipTo(polygon,1,grid.origin.y+y*grid.pitch,1,s.rowA);
  clipTo(s.rowA,1,grid.origin.y+(y+1)*grid.pitch,-1,s.row);if(fraction(s.row)<1e-14)continue;
  for(auto column=ctx.rows.lower(lo[0],y);column!=end&&column->y==y&&column->x<=hi[0];++column){spend();
   clipTo(s.row,0,grid.origin.x+column->x*grid.pitch,1,s.slabA);
   clipTo(s.slabA,0,grid.origin.x+(column->x+1)*grid.pitch,-1,s.slab);if(fraction(s.slab)<1e-14)continue;
   for(auto index=column->first;index<column->first+column->count;++index){spend();const auto&span=volume.spans[index];
    if(!ctx.isRetained(span.component)||(flatZ&&(flatCell<span.lo||flatCell>=span.hi)))continue;
    clipTo(s.slab,2,grid.origin.z+span.lo*grid.pitch,1,s.cutA);
    clipTo(s.cutA,2,grid.origin.z+span.hi*grid.pitch,-1,s.cut);
    if(fraction(s.cut)>1e-14){
     auto at=std::lower_bound(pieces.begin(),pieces.end(),span.component,[](const auto&e,std::uint32_t c){return e.first<c;});
     if(at==pieces.end()||at->first!=span.component)at=pieces.insert(at,{span.component,{}});
     at->second.push_back(s.cut);}
   }
  }
 }
 // Normal curved clothing triangles can bridge pristine air between all three
 // anchors. If every anchor and every covered material cell has one owner,
 // preserve the intact original triangle, including its existing curvature.
 if(pieces.size()==1&&cornerOwners[0]==pieces.front().first&&cornerOwners[1]==cornerOwners[0]&&cornerOwners[2]==cornerOwners[0]
    &&!coversRemovedAnatomyFast(polygon,bounds,ctx,out.work,s)){
#ifdef GFY_RX_PROF
  out.path=2;
#endif
  out.kind=Outcome::kept;out.owner=cornerOwners[0];return;
 }
 // Whole-owner acceptance to clipping arithmetic noise, same removed-anatomy test (see partitionTriangle).
 if(pieces.size()==1){double whole=0;for(const auto&p:pieces.front().second)whole+=fraction(p);
  if(whole>=1-1e-10&&!coversRemovedAnatomyFast(polygon,bounds,ctx,out.work,s)){
#ifdef GFY_RX_PROF
   out.path=2;
#endif
   out.kind=Outcome::kept;out.owner=pieces.front().first;return;}}
 GFY_EARLY_MARK(4,1)
 const core::JoinedTriangle&t=full();
 auto side=std::make_unique<SplitOut>();std::size_t output=0;
 double emitted=0;
 double beforeMerge=0;for(const auto&entry:pieces)for(const auto&p:entry.second)beforeMerge+=fraction(p);
 for(auto&entry:pieces)entry.second=mergeConvexFast(std::move(entry.second),s.merge);
 for(auto&entry:pieces)for(const auto&poly:entry.second){
  emitted+=fraction(poly);const auto ring=distinctRing(t,poly);
  for(std::size_t i=1;i+1<ring.size();++i){core::JoinedTriangle triangle;triangle.material=t.material;
   triangle.v={{ring[0],ring[i],ring[i+1]}};
   if(area(triangle)<1e-18||packedDegenerate(triangle))continue;
   need(output<4000000,"source_partition_triangle_budget");
   ++output;side->triangles.emplace_back(entry.first,std::move(triangle));
  }
 }
 if(emitted>1.00001){std::ostringstream message;message<<std::setprecision(17)<<"source_partition_overlap fraction="<<emitted<<" before_merge="<<beforeMerge<<" source="<<t.v[0].sourceFace<<" material="<<t.material;
  for(auto p:anchors)message<<" anchor="<<p.x<<','<<p.y<<','<<p.z;throw std::runtime_error(message.str());}
 if(pieces.empty())side->trimmed=1;else side->split=1;
 side->trimmed_area+=originalArea*std::max(0.,1-emitted);
#ifdef GFY_RX_PROF
 out.path=pieces.empty()?4:3;
#endif
 out.kind=Outcome::split;out.side=std::move(side);
}

// ---- GFY_RX_VERIFY: per-triangle and end-to-end comparison with int14t -----
inline std::string failureOf(const std::exception_ptr&e){
 if(!e)return {};try{std::rethrow_exception(e);}catch(const std::exception&x){return std::string(typeid(x).name())+": "+x.what();}catch(...){return "non-std exception";}
}
inline bool sameBits(double a,double b){return !std::memcmp(&a,&b,sizeof a);}
inline bool sameVertex(const core::JoinedVertex&a,const core::JoinedVertex&b){
 return !std::memcmp(&a.p,&b.p,sizeof(Vec))&&!std::memcmp(&a.normal,&b.normal,sizeof(Vec))&&!std::memcmp(&a.tangent,&b.tangent,sizeof(Vec))
  &&!std::memcmp(&a.binormal,&b.binormal,sizeof(Vec))&&a.sourceFace==b.sourceFace&&!std::memcmp(a.sourceBary.data(),b.sourceBary.data(),sizeof(a.sourceBary))
  &&!std::memcmp(a.uv.data(),b.uv.data(),sizeof(a.uv))&&!std::memcmp(a.w.data(),b.w.data(),sizeof(a.w));
}
inline bool sameTriangle(const core::JoinedTriangle&a,const core::JoinedTriangle&b){return a.material==b.material&&sameVertex(a.v[0],b.v[0])&&sameVertex(a.v[1],b.v[1])&&sameVertex(a.v[2],b.v[2]);}
inline bool sameStats(const Stats&a,const Stats&b){
 return a.input==b.input&&a.kept==b.kept&&a.split==b.split&&a.trimmed==b.trimmed&&a.output==b.output&&a.air_cavity==b.air_cavity
  &&a.unsupported_coat_triangles==b.unsupported_coat_triangles&&a.work==b.work&&sameBits(a.original_area,b.original_area)
  &&sameBits(a.emitted_area,b.emitted_area)&&sameBits(a.trimmed_area,b.trimmed_area)&&sameBits(a.air_cavity_area,b.air_cavity_area);
}
// ---- R51: partitionTriangle() under a PartitionCertificate ------------------------------------------------------------
// Every statement, clip, work count, check and emission of the reference, in its order, with three exact substitutions:
//  1. the early-accept foreign scan is skipped when cert.noForeign() proves it finds nothing (the scan counts no work);
//  2. coversRemovedAnatomy() becomes coversRemovedAnatomyCertified() (same result, same work total);
//  3. corners on one retained owner with a foreign span in the box (the reference then runs its full loop and keeps the
//     triangle only when the pieces are exactly {owner}): a first pass replays the full loop's rows, columns, spans and
//     work counts but clips owner spans only until one is positive. No positive foreign piece and a positive owner piece
//     mean the reference's pieces are exactly {owner}: its first keep rule decides with the same covers test. Anything
//     else restores the work counter and runs the full loop.
// Clips use thread-local scratch buffers (clipTo, the same arithmetic as clip()) and merges use mergeConvexFast (the
// same merges in the same order as mergeConvex()).
inline void partitionTriangleCertified(const core::JoinedTriangle&t,const std::array<Vec,3>&anchors,const Grid&grid,
    const gfy_connectivity::Topology&volume,const std::set<std::uint32_t>&retained,Result&out,const PartitionCertificate&cert){
 thread_local PartitionScratch s;
 core::Poly polygon;core::Bounds bounds;std::array<std::uint32_t,3> cornerOwners{};
 for(int k=0;k<3;++k){core::V v;v.p=snapToCellPlanes(anchors[k],grid);v.b[k]=1;polygon.push_back(v);bounds.add(v.p);cornerOwners[k]=at(v.p,grid,volume);}
 const auto originalArea=area(t);++out.stats.input;out.stats.original_area+=originalArea;
 std::map<std::uint32_t,std::vector<core::Poly>> pieces;
 auto spend=[&](){need(++out.stats.work<=160000000,"source_partition_work_budget");};
 int lo[2],hi[2];
 for(int a=0;a<2;++a){lo[a]=std::max(0,int(std::floor((bounds.lo[a]-grid.origin[a])/grid.pitch)));
  hi[a]=std::min(grid.size[a]-1,int(std::floor((bounds.hi[a]-grid.origin[a])/grid.pitch)));}
 const bool flatZ=bounds.hi.z-bounds.lo.z<1e-12;
 const int flatCell=int(std::floor((bounds.lo.z-grid.origin.z)/grid.pitch));
 auto firstColumn=[&](int y){return std::lower_bound(volume.columns.begin(),volume.columns.end(),std::array<int,2>{{lo[0],y}},
  [](const gfy_connectivity::Column&c,const std::array<int,2>&p){return gfy_connectivity::xyLess(c.x,c.y,p[0],p[1]);});};
 if(cornerOwners[0]==cornerOwners[1]&&cornerOwners[1]==cornerOwners[2]&&retained.count(cornerOwners[0])){
  const auto owner=cornerOwners[0];
  const int zlo=int(std::floor((bounds.lo.z-grid.origin.z)/grid.pitch))-1,zhi=int(std::floor((bounds.hi.z-grid.origin.z)/grid.pitch))+1;
  bool foreign=false;
  if(!cert.noForeign(owner,lo,hi,zlo,zhi))
  for(int y=lo[1];y<=hi[1]&&!foreign;++y){
   for(auto column=firstColumn(y);column!=volume.columns.end()&&column->y==y&&column->x<=hi[0]&&!foreign;++column)
    for(auto index=column->first;index<column->first+column->count;++index){const auto&span=volume.spans[index];
     if(span.hi<zlo||span.lo>zhi||span.component==owner)continue;
     if(retained.count(span.component)){foreign=true;break;}}
  }
  bool positive=false;
  for(int y=lo[1];y<=hi[1]&&!foreign&&!positive;++y){spend();
   clipTo(polygon,1,grid.origin.y+y*grid.pitch,1,s.rowA);
   clipTo(s.rowA,1,grid.origin.y+(y+1)*grid.pitch,-1,s.row);if(fraction(s.row)<1e-14)continue;
   for(auto column=firstColumn(y);column!=volume.columns.end()&&column->y==y&&column->x<=hi[0]&&!positive;++column){spend();
    bool ownerSpan=false;
    for(auto index=column->first;index<column->first+column->count;++index){const auto&span=volume.spans[index];
     if(span.component==owner&&!(flatZ&&(flatCell<span.lo||flatCell>=span.hi))){ownerSpan=true;break;}}
    if(!ownerSpan)continue;
    clipTo(s.row,0,grid.origin.x+column->x*grid.pitch,1,s.slabA);
    clipTo(s.slabA,0,grid.origin.x+(column->x+1)*grid.pitch,-1,s.slab);if(fraction(s.slab)<1e-14)continue;
    for(auto index=column->first;index<column->first+column->count&&!positive;++index){spend();const auto&span=volume.spans[index];
     if(span.component!=owner||(flatZ&&(flatCell<span.lo||flatCell>=span.hi)))continue;
     clipTo(s.slab,2,grid.origin.z+span.lo*grid.pitch,1,s.cutA);
     clipTo(s.cutA,2,grid.origin.z+span.hi*grid.pitch,-1,s.cut);
     positive=fraction(s.cut)>1e-14;
    }
   }
  }
  if(positive&&!coversRemovedAnatomyCertified(polygon,bounds,grid,out.stats,cert)){
   out.keep(owner,t);++out.stats.kept;++out.stats.output;out.stats.emitted_area+=originalArea;return;
  }
  if(foreign){
   const auto mark=out.stats.work;bool decided=true,ownerPositive=false;
   for(int y=lo[1];y<=hi[1]&&decided;++y){spend();
    clipTo(polygon,1,grid.origin.y+y*grid.pitch,1,s.rowA);
    clipTo(s.rowA,1,grid.origin.y+(y+1)*grid.pitch,-1,s.row);if(fraction(s.row)<1e-14)continue;
    for(auto column=firstColumn(y);column!=volume.columns.end()&&column->y==y&&column->x<=hi[0]&&decided;++column){spend();
     clipTo(s.row,0,grid.origin.x+column->x*grid.pitch,1,s.slabA);
     clipTo(s.slabA,0,grid.origin.x+(column->x+1)*grid.pitch,-1,s.slab);if(fraction(s.slab)<1e-14)continue;
     for(auto index=column->first;index<column->first+column->count;++index){spend();const auto&span=volume.spans[index];
      if(!retained.count(span.component)||(flatZ&&(flatCell<span.lo||flatCell>=span.hi)))continue;
      if(span.component==owner&&ownerPositive)continue; // its piece only decides positivity, already proved
      clipTo(s.slab,2,grid.origin.z+span.lo*grid.pitch,1,s.cutA);
      clipTo(s.cutA,2,grid.origin.z+span.hi*grid.pitch,-1,s.cut);
      if(fraction(s.cut)>1e-14){if(span.component!=owner){decided=false;break;}ownerPositive=true;}
     }
    }
   }
   // pieces == {owner} exactly: the reference's first keep rule (corners agree) with its covers test.
   if(decided&&ownerPositive&&!coversRemovedAnatomyCertified(polygon,bounds,grid,out.stats,cert)){
    out.keep(cornerOwners[0],t);++out.stats.kept;++out.stats.output;out.stats.emitted_area+=originalArea;return;
   }
   if(partitionWorkPeak&&out.stats.work>*partitionWorkPeak)*partitionWorkPeak=out.stats.work; // R66
   out.stats.work=mark; // not decided here: the full loop below counts exactly as the reference
  }
 }
 for(int y=lo[1];y<=hi[1];++y){spend();
  clipTo(polygon,1,grid.origin.y+y*grid.pitch,1,s.rowA);
  clipTo(s.rowA,1,grid.origin.y+(y+1)*grid.pitch,-1,s.row);if(fraction(s.row)<1e-14)continue;
  for(auto column=firstColumn(y);column!=volume.columns.end()&&column->y==y&&column->x<=hi[0];++column){spend();
   clipTo(s.row,0,grid.origin.x+column->x*grid.pitch,1,s.slabA);
   clipTo(s.slabA,0,grid.origin.x+(column->x+1)*grid.pitch,-1,s.slab);if(fraction(s.slab)<1e-14)continue;
   for(auto index=column->first;index<column->first+column->count;++index){spend();const auto&span=volume.spans[index];
    if(!retained.count(span.component)||(flatZ&&(flatCell<span.lo||flatCell>=span.hi)))continue;
    clipTo(s.slab,2,grid.origin.z+span.lo*grid.pitch,1,s.cutA);
    clipTo(s.cutA,2,grid.origin.z+span.hi*grid.pitch,-1,s.cut);
    if(fraction(s.cut)>1e-14)pieces[span.component].push_back(s.cut);
   }
  }
 }
 if(pieces.size()==1&&cornerOwners[0]==pieces.begin()->first&&cornerOwners[1]==cornerOwners[0]&&cornerOwners[2]==cornerOwners[0]
    &&!coversRemovedAnatomyCertified(polygon,bounds,grid,out.stats,cert)){
  out.keep(cornerOwners[0],t);++out.stats.kept;++out.stats.output;out.stats.emitted_area+=originalArea;return;
 }
 if(pieces.size()==1){double whole=0;for(const auto&p:pieces.begin()->second)whole+=fraction(p);
  if(whole>=1-1e-10&&!coversRemovedAnatomyCertified(polygon,bounds,grid,out.stats,cert)){
   out.keep(pieces.begin()->first,t);++out.stats.kept;++out.stats.output;out.stats.emitted_area+=originalArea;return;}}
 double emitted=0;
 double beforeMerge=0;for(const auto&entry:pieces)for(const auto&p:entry.second)beforeMerge+=fraction(p);
 for(auto&entry:pieces)entry.second=mergeConvexFast(std::move(entry.second),s.merge);
 for(auto&entry:pieces)for(const auto&poly:entry.second){
  emitted+=fraction(poly);const auto ring=distinctRing(t,poly);
  for(std::size_t i=1;i+1<ring.size();++i){core::JoinedTriangle triangle;triangle.material=t.material;
   triangle.v={{ring[0],ring[i],ring[i+1]}};
   if(area(triangle)<1e-18||packedDegenerate(triangle))continue;
   need(out.stats.output<4000000,"source_partition_triangle_budget");
   out.stats.emitted_area+=area(triangle);++out.stats.output;out.meshes[entry.first].triangles.push_back(std::move(triangle));
  }
 }
 if(emitted>1.00001){std::ostringstream message;message<<std::setprecision(17)<<"source_partition_overlap fraction="<<emitted<<" before_merge="<<beforeMerge<<" source="<<t.v[0].sourceFace<<" material="<<t.material;
  for(auto p:anchors)message<<" anchor="<<p.x<<','<<p.y<<','<<p.z;throw std::runtime_error(message.str());}
 if(pieces.empty())++out.stats.trimmed;else ++out.stats.split;
 out.stats.trimmed_area+=originalArea*std::max(0.,1-emitted);
}
#ifdef GFY_PARTITION_CERT_VERIFY
// A/B of one triangle (verify builds only): the certified path writes into `out`; the reference runs into a scratch
// Result starting from the same stats and deferKeep. The triangles appended per owner (in order), a deferred keep and
// every stats field must be bit-identical, and the two must throw the same message or neither; else std::logic_error.
inline void partitionCertifiedVerified(const core::JoinedTriangle&t,const std::array<Vec,3>&anchors,const Grid&grid,
    const gfy_connectivity::Topology&volume,const std::set<std::uint32_t>&retained,Result&out,const PartitionCertificate&cert){
 Result reference;reference.stats=out.stats;reference.deferKeep=out.deferKeep;
 std::map<std::uint32_t,std::size_t> before;for(const auto&m:out.meshes)before[m.first]=m.second.triangles.size();
 const bool deferredBefore=out.deferred;
 std::string referenceError;
 try{partitionTriangle(t,anchors,grid,volume,retained,reference);}catch(const std::exception&e){referenceError=e.what();}
 auto fail=[&](const std::string&why){throw std::logic_error("GFY_PARTITION_CERT_VERIFY source="+std::to_string(t.v[0].sourceFace)+" "+why);};
 try{partitionTriangleCertified(t,anchors,grid,volume,retained,out,cert);}
 catch(const std::exception&e){if(referenceError!=e.what())fail(std::string("exception certified=")+e.what()+" reference="+referenceError);throw;}
 if(!referenceError.empty())fail("reference threw, certified did not: "+referenceError);
 if(!sameStats(reference.stats,out.stats))fail("stats");
 if(out.deferKeep&&!deferredBefore&&(reference.deferred!=out.deferred||(reference.deferred&&reference.deferredOwner!=out.deferredOwner)))fail("deferred keep");
 for(const auto&m:out.meshes){const auto b=before.count(m.first)?before[m.first]:std::size_t(0);const auto&got=m.second.triangles;
  const auto r=reference.meshes.find(m.first);const std::size_t want=r==reference.meshes.end()?0:r->second.triangles.size();
  if(got.size()-b!=want)fail("owner "+std::to_string(m.first)+" triangles "+std::to_string(got.size()-b)+" reference "+std::to_string(want));
  for(std::size_t i=0;i<want;++i)if(!sameTriangle(got[b+i],r->second.triangles[i]))fail("owner "+std::to_string(m.first)+" triangle "+std::to_string(i));}
 for(const auto&r:reference.meshes)if(!r.second.triangles.empty()&&!out.meshes.count(r.first))fail("owner "+std::to_string(r.first)+" missing");
}
#endif
// The former per-triangle computations (bodies of the int14t computeAll lambdas).
inline void referenceJoinedOne(const core::JoinedTriangle&t,const Asset&asset,const Grid&grid,const gfy_connectivity::Topology&volume,
                               const Bindings&bindings,const std::set<std::uint32_t>&retained,Result&out,surface_support::Budget&localBudget){
 if(t.material){
  auto contacts=surface_support::cavity(t,grid,volume,localBudget);
  std::set<std::uint32_t>owners;for(auto c:contacts)if(retained.count(c.first))owners.insert(c.first);
  if(owners.size()==1){out.keep(*owners.begin(),t);++out.stats.input;++out.stats.kept;++out.stats.output;
   out.stats.original_area+=area(t);out.stats.emitted_area+=area(t);return;}
  if(owners.empty()){++out.stats.air_cavity;out.stats.air_cavity_area+=area(t);return;}
  std::array<Vec,3>anchors;
  for(int k=0;k<3;++k){auto n=t.v[k].normal;need(length(n)>1e-10,"partition_cavity_normal");anchors[k]=t.v[k].p-n*(grid.pitch*.5/length(n));}
  partitionTriangle(t,anchors,grid,volume,retained,out);return;
 }
 const auto face=t.v[0].sourceFace;need(face<asset.source.f.size(),"source_partition_face");
 std::array<Vec,3>anchors;
 for(int k=0;k<3;++k){need(t.v[k].sourceFace==face,"source_partition_mixed_provenance");double sum=0;
  for(int j=0;j<3;++j){const auto b=t.v[k].sourceBary[j];need(std::isfinite(b)&&b>=-1e-6&&b<=1.000001,"source_partition_barycentric");sum+=b;
   anchors[k]=anchors[k]+bindings.vertices[asset.source.f[face][j]].point*b;}
  need(std::abs(sum-1)<1e-6,"source_partition_barycentric_sum");}
 partitionTriangle(t,anchors,grid,volume,retained,out);
}
inline void referenceExtraOne(const ExtraSurface&extra,const Asset&asset,std::size_t f,const Grid&grid,const gfy_connectivity::Topology&volume,
                              const std::set<std::uint32_t>&retained,Result&out){
 core::JoinedTriangle t;std::array<Vec,3>anchors;
 for(unsigned k=0;k<3;++k){core::V vertex;vertex.p=extra.source.v[extra.source.f[f][k]].p;vertex.b[k]=1;
  t.v[k]=core::interpolateVertex(extra.source,std::uint32_t(f),vertex);t.v[k].sourceFace+=std::uint32_t(asset.source.f.size());
  anchors[k]=extra.bindings.vertices[extra.source.f[f][k]].point;}
 partitionTriangle(t,anchors,grid,volume,retained,out);
}
inline void verifyOutcome(const char*what,std::size_t index,const Outcome&o,const Result&r,const surface_support::Budget&b,const std::exception_ptr&e){
 auto fail=[&](const std::string&why){throw std::logic_error(std::string("GFY_RX_VERIFY partition ")+what+" "+std::to_string(index)+": "+why);};
 if(b.visited!=o.budget.visited)fail("visited "+std::to_string(b.visited)+" vs "+std::to_string(o.budget.visited));
 if(r.stats.work!=o.work)fail("work "+std::to_string(r.stats.work)+" vs "+std::to_string(o.work));
 const auto x=failureOf(e),y=failureOf(o.error);if(x!=y)fail("failure '"+x+"' vs '"+y+"'");
 if(e)return;
 const auto&s=r.stats;const bool kept=s.kept==1,air=s.air_cavity==1,splitting=s.split+s.trimmed==1;
 if(int(kept)+int(air)+int(splitting)!=1)fail("reference class");
 if(kept){
  if(o.kind!=Outcome::kept)fail("kind kept");
  std::uint32_t owner=0;
  if(r.deferred)owner=r.deferredOwner;
  else{if(r.meshes.size()!=1||r.meshes.begin()->second.triangles.size()!=1)fail("kept copy");owner=r.meshes.begin()->first;}
  if(owner!=o.owner)fail("owner "+std::to_string(owner)+" vs "+std::to_string(o.owner));
  if(s.input!=1||s.output!=1||!sameBits(s.original_area,o.area)||!sameBits(s.emitted_area,o.area)||!sameBits(s.air_cavity_area,0.)||!sameBits(s.trimmed_area,0.))fail("kept stats");
 }
 if(air){if(o.kind!=Outcome::air)fail("kind air");
  if(s.input!=0||s.output!=0||!sameBits(s.air_cavity_area,o.area)||!sameBits(s.original_area,0.)||!sameBits(s.trimmed_area,0.)||!r.meshes.empty())fail("air stats");}
 if(splitting){
  if(o.kind!=Outcome::split||!o.side)fail("kind split");const auto&side=*o.side;
  if(s.input!=1||!sameBits(s.original_area,o.area)||s.split!=side.split||s.trimmed!=side.trimmed||!sameBits(s.trimmed_area,side.trimmed_area)||!sameBits(s.air_cavity_area,0.))fail("split stats");
  std::size_t n=0;
  for(const auto&m:r.meshes)for(const auto&t:m.second.triangles){
   if(n>=side.triangles.size()||side.triangles[n].first!=m.first||!sameTriangle(side.triangles[n].second,t))fail("split triangle "+std::to_string(n));++n;}
  if(n!=side.triangles.size()||s.output!=n)fail("split count");
 }
}

// ---- S5: boundary quads from integer face keys ------------------------------
// Exactly fastwound::mergeBoundaryRects(grid, tileBoundary(grid,t) for every
// tile t, concatenated) on an owner-free grid: the same quads in the same
// order. tileBoundary() emits one quad per open face (removed cell c,
// direction a, neighbour nb not removed) with corners origin+(c+corner)*pitch.
// mergeBoundaryRects() derives its sort Entry from those float corners; with
// P(k)=origin+k*pitch strictly increasing and (P(k)-origin)/pitch within far
// less than 1/2 of k (guaranteed for finite pitch>0, |origin|<1e9*pitch and
// k<2^17, see NOTE.txt), that Entry is, for axis=a/2, u=(axis+1)%3, v=(axis+2)%3:
//   {axis, c[axis]+(a&1), (a&1)?+1:-1, renderMaterial(nb)}, row=c[v], u=c[u].
// It is encoded as the 60-bit key axis:2|plane:17|sign:1|material:8|row:16|u:16,
// whose numeric order is the Entry order. Keys are bucketed by (axis, plane),
// each bucket sorted and swept (the map-free equivalent of the active/next run
// maps) on up to maxThreads threads, and the buckets' quads concatenated in
// bucket order. Returns false (the caller runs the reference path) when the
// lattice or a value does not fit these assumptions.
inline bool boundaryFast(const Grid&g,unsigned maxThreads,std::vector<core::Quad>&out){
 if(g.ownership||!(std::isfinite(g.pitch)&&g.pitch>0))return false;
 for(int a=0;a<3;++a)if(!(std::isfinite(g.origin[a])&&std::abs(g.origin[a])<1e9*g.pitch))return false;
 std::vector<Cell> keys;keys.reserve(g.tiles.size());for(const auto&t:g.tiles)keys.push_back(t.first);
#ifdef GFY_RX_PROF
 auto rxT=core::Clock::now();auto rxMark=[&](const char*n,std::size_t count){std::cerr<<"rxprof boundary."<<n<<" ms="<<core::ms(rxT)<<" count="<<count<<std::endl;rxT=core::Clock::now();};
#else
 auto rxMark=[](const char*,std::size_t){};
#endif
 const std::size_t chunks=sep::chunkCount(keys.size(),64,maxThreads);
 std::vector<std::vector<std::uint64_t>> faces(chunks);std::vector<std::uint32_t> maxPlane(chunks,0);std::vector<char> overflow(chunks,0);
 sep::parallel(keys.size(),64,maxThreads,[&](std::size_t first,std::size_t last,std::size_t chunk){
  auto&list=faces[chunk];std::uint32_t top=0;
  constexpr std::uint64_t Z0=0x0101010101010101ull,Z7=Z0<<7;
  static const int delta[6][3]={{-1,0,0},{1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,0,1}};
  for(std::size_t i=first;i<last;++i){const Cell t=keys[i];
   if(t.x<0||t.y<0||t.z<0)continue;auto it=g.tiles.find(t);if(it==g.tiles.end())continue;
   const auto&w=it->second.removed;
   auto neighbour=[&](int dx,int dy,int dz)->const std::array<std::uint64_t,8>*{auto f=g.tiles.find(Cell{t.x+dx,t.y+dy,t.z+dz});return f==g.tiles.end()?nullptr:&f->second.removed;};
   const auto*mx=neighbour(-1,0,0);const auto*px=neighbour(1,0,0);const auto*my=neighbour(0,-1,0);
   const auto*py=neighbour(0,1,0);const auto*mz=neighbour(0,0,-1);const auto*pz=neighbour(0,0,1);
   for(int x=0;x<8;++x){const std::uint64_t c=w[std::size_t(x)];if(!c)continue;
    std::uint64_t n[6];
    n[0]=x>0?w[std::size_t(x-1)]:(mx?(*mx)[7]:0);
    n[1]=x<7?w[std::size_t(x+1)]:(px?(*px)[0]:0);
    n[2]=(c<<8)|(my?((*my)[std::size_t(x)]>>56):0);
    n[3]=(c>>8)|(py?(((*py)[std::size_t(x)]&0xFFull)<<56):0);
    n[4]=((c<<1)&~Z0)|(mz?(((*mz)[std::size_t(x)]&Z7)>>7):0);
    n[5]=((c>>1)&~Z7)|(pz?(((*pz)[std::size_t(x)]&Z0)<<7):0);
    std::uint64_t open[6],any=0;for(int a=0;a<6;++a){open[a]=c&~n[a];any|=open[a];}
    while(any){const unsigned bit=core::lowestBit(any);any&=any-1;
     const Cell cell{t.x*8+x,t.y*8+int(bit>>3),t.z*8+int(bit&7)};
     for(int a=0;a<6;++a){if(!((open[a]>>bit)&1u))continue;
      const Cell nb{cell.x+delta[a][0],cell.y+delta[a][1],cell.z+delta[a][2]};
      const unsigned material=g.renderMaterial(nb);
      const int axis=a/2;const std::int64_t plane=std::int64_t(cell[axis])+(a&1),row=cell[(axis+2)%3],col=cell[(axis+1)%3];
      if(material>255||plane>131071||row>65535||col>65535){overflow[chunk]=1;return;}
      top=std::max(top,std::uint32_t(plane));
      list.push_back(std::uint64_t(axis)<<58|std::uint64_t(plane)<<41|std::uint64_t(a&1)<<40|std::uint64_t(material)<<32|std::uint64_t(row)<<16|std::uint64_t(col));
     }
    }
   }
  }
  maxPlane[chunk]=top;});
 for(const auto o:overflow)if(o)return false;
 {std::size_t n=0;for(const auto&f:faces)n+=f.size();rxMark("faces",n);}
 // Bucket (axis, plane) = counting sort on the leading key fields.
 std::uint32_t planes=0;for(const auto m:maxPlane)planes=std::max(planes,m);
 const std::size_t P=std::size_t(planes)+1,B=3*P;
 auto bucketOf=[&](std::uint64_t k){return std::size_t(k>>58)*P+std::size_t((k>>41)&0x1FFFFu);};
 std::vector<std::vector<std::uint32_t>> counts(chunks);
 sep::parallel(chunks,1,maxThreads,[&](std::size_t first,std::size_t last,std::size_t){
  for(std::size_t c=first;c<last;++c){auto&h=counts[c];h.assign(B,0);for(const auto k:faces[c])++h[bucketOf(k)];}});
 std::vector<std::size_t> start(B+1,0);
 for(std::size_t b=0;b<B;++b){std::size_t n=0;for(std::size_t c=0;c<chunks;++c)n+=counts[c][b];start[b+1]=start[b]+n;}
 std::vector<std::uint64_t> sorted(start[B]);
 sep::parallel(chunks,1,maxThreads,[&](std::size_t first,std::size_t last,std::size_t){
  for(std::size_t c=first;c<last;++c){std::vector<std::size_t> at(B);
   for(std::size_t b=0;b<B;++b){std::size_t o=start[b];for(std::size_t d=0;d<c;++d)o+=counts[d][b];at[b]=o;}
   for(const auto k:faces[c])sorted[at[bucketOf(k)]++]=k;
   faces[c]=std::vector<std::uint64_t>();}});
 rxMark("bucket",B);
 // Contiguous bucket ranges of about equal size; each sorted and swept.
 const std::size_t parts=std::max<std::size_t>(1,std::min<std::size_t>(sep::chunkCount(sorted.size(),4096,maxThreads),B));
 std::vector<std::size_t> cut(parts+1,B);cut[0]=0;
 for(std::size_t p=1,b=0;p<parts;++p){const auto target=sorted.size()*p/parts;while(b<B&&start[b]<target)++b;cut[p]=b;}
 std::vector<std::vector<core::Quad>> quads(parts);
 sep::parallel(parts,1,maxThreads,[&](std::size_t first,std::size_t last,std::size_t){
  struct Rect{int u0,u1,v0,v1;};std::vector<Rect> active,next;std::vector<char> used;
  for(std::size_t p=first;p<last;++p){auto&emitted=quads[p];emitted.reserve(start[cut[p+1]]-start[cut[p]]); // quads <= face keys
   for(std::size_t b=cut[p];b<cut[p+1];++b){
    auto*begin=sorted.data()+start[b];auto*end=sorted.data()+start[b+1];if(begin==end)continue;
    std::sort(begin,end);end=std::unique(begin,end);
    for(auto*plane=begin;plane<end;){
     const std::uint64_t key=*plane>>32;auto*stop=plane;while(stop<end&&(*stop>>32)==key)++stop;
     const int axis=int(key>>26),u=(axis+1)%3,v=(axis+2)%3,level=int((key>>9)&0x1FFFFu),sign=(key>>8)&1u?1:-1;
     const unsigned material=unsigned(key&0xFFu);
     auto emit=[&](const Rect&r){core::Quad q;q.material=material;int uv[4][2]={{r.u0,r.v0},{r.u1,r.v0},{r.u1,r.v1},{r.u0,r.v1}};
      for(int k=0;k<4;++k){Vec p;p[axis]=level;p[u]=uv[k][0];p[v]=uv[k][1];q.p[std::size_t(k)]=g.origin+p*g.pitch;}
      if(sign<0)std::reverse(q.p.begin(),q.p.end());
      (&q.removedCell.x)[axis]=level-(sign>0?1:0);
      (&q.removedCell.x)[u]=r.u0;(&q.removedCell.x)[v]=r.v0;emitted.push_back(q);};
     active.clear();
     for(auto*at=plane;at<stop;){const int row=int((*at>>16)&0xFFFFu);auto*rowEnd=at;while(rowEnd<stop&&int((*rowEnd>>16)&0xFFFFu)==row)++rowEnd;
      next.clear();used.assign(active.size(),0);std::size_t cursor=0;
      for(auto*k=at;k<rowEnd;){const int lo=int(*k&0xFFFFu);int hi=lo+1;++k;while(k<rowEnd&&int(*k&0xFFFFu)==hi){++hi;++k;}
       while(cursor<active.size()&&(active[cursor].u0<lo||(active[cursor].u0==lo&&active[cursor].u1<hi)))++cursor;
       if(cursor<active.size()&&active[cursor].u0==lo&&active[cursor].u1==hi&&active[cursor].v1==row){Rect r=active[cursor];r.v1=row+1;next.push_back(r);used[cursor]=1;}
       else next.push_back(Rect{lo,hi,row,row+1});
      }
      for(std::size_t i=0;i<active.size();++i)if(!used[i])emit(active[i]);
      active.swap(next);at=rowEnd;
     }
     for(const auto&r:active)emit(r);
     plane=stop;
    }
   }
  }});
 std::size_t total=0;for(const auto&q:quads)total+=q.size();
 rxMark("sweep",total);
 out.clear();out.reserve(total);for(auto&q:quads)out.insert(out.end(),q.begin(),q.end());
 rxMark("concat",total);
 return true;
}
// S5: boundary + union patch selection of one separation.
inline void boundaryQuads(const Grid&grid,unsigned maxThreads,std::vector<core::Quad>&boundary){
 if(boundaryFast(grid,maxThreads,boundary)){
#ifdef GFY_RX_VERIFY
  std::vector<core::Quad>reference;{std::set<Cell>dirty;for(const auto&t:grid.tiles)dirty.insert(t.first);
   const std::vector<Cell>keys(dirty.begin(),dirty.end());std::vector<core::Quad>quads;for(const auto&k:keys)fastwound::tileBoundary(grid,k,quads);
   std::vector<fastwound::RectMeta>meta;std::map<fastwound::PlaneKey,std::vector<int>>planeRows;reference=fastwound::mergeBoundaryRects(grid,quads,meta,planeRows);}
  bool same=reference.size()==boundary.size();
  for(std::size_t i=0;same&&i<boundary.size();++i){const auto&a=boundary[i];const auto&b=reference[i];
   same=a.material==b.material&&a.removedCell==b.removedCell&&!std::memcmp(a.p.data(),b.p.data(),sizeof(a.p));}
  if(!same)throw std::logic_error("GFY_RX_VERIFY boundary mismatch");
  std::cerr<<"GFY_RX_VERIFY boundary identical quads="<<boundary.size()<<std::endl;
#endif
  return;
 }
 // Reference (int14t) path: owned grids, or a lattice outside the key's range.
 std::set<Cell>dirty;for(const auto&t:grid.tiles)dirty.insert(t.first);
 if(!grid.ownership){
  const std::vector<Cell>keys(dirty.begin(),dirty.end());std::array<std::vector<core::Quad>,4>parts;
  sep::parallel(keys.size(),std::max<std::size_t>(64,keys.size()/4),maxThreads,[&](std::size_t first,std::size_t last,std::size_t chunk){
   for(std::size_t i=first;i<last;++i)fastwound::tileBoundary(grid,keys[i],parts[chunk]);});
  std::vector<core::Quad>quads;std::size_t total=0;for(auto&part:parts)total+=part.size();quads.reserve(total);
  for(auto&part:parts)quads.insert(quads.end(),part.begin(),part.end());
  std::vector<fastwound::RectMeta>meta;std::map<fastwound::PlaneKey,std::vector<int>>planeRows;
  boundary=fastwound::mergeBoundaryRects(grid,quads,meta,planeRows);
 }else boundary=core::mergeBoundaryQuads(grid,core::dirtyBoundary(grid,dirty));
}
// Source faces within two cells of any removed tile (the former per-tile BVH
// queries into a std::set): per-thread face flags, ORed, scanned ascending.
inline void selectPatch(const Asset&asset,const Grid&grid,unsigned maxThreads,std::vector<std::uint32_t>&originals,std::vector<std::uint32_t>&untouched){
 std::vector<Cell> keys;keys.reserve(grid.tiles.size());for(const auto&t:grid.tiles)keys.push_back(t.first);
 const std::size_t faces=asset.source.f.size();
 const std::size_t chunks=sep::chunkCount(keys.size(),64,maxThreads);
 std::vector<std::vector<std::uint8_t>> flags(chunks);
 sep::parallel(keys.size(),64,maxThreads,[&](std::size_t first,std::size_t last,std::size_t chunk){
  auto&mine=flags[chunk];mine.assign(faces,0);std::vector<std::uint32_t> found;
  for(std::size_t i=first;i<last;++i){auto c=keys[i];core::Bounds box;
   box.lo=grid.origin+Vec{double(c.x*8-2),double(c.y*8-2),double(c.z*8-2)}*grid.pitch;
   box.hi=grid.origin+Vec{double(c.x*8+10),double(c.y*8+10),double(c.z*8+10)}*grid.pitch;
   found.clear();asset.source_tree.query(0,box,found);for(const auto f:found)mine[f]=1;}});
 originals.clear();untouched.clear();
 for(std::uint32_t f=0;f<faces;++f){bool hit=false;for(const auto&mine:flags)hit|=mine[f]!=0;(hit?originals:untouched).push_back(f);}
#ifdef GFY_RX_VERIFY
 {std::set<std::uint32_t> selected;
  for(const auto&tile:grid.tiles){auto c=tile.first;core::Bounds box;
   box.lo=grid.origin+Vec{double(c.x*8-2),double(c.y*8-2),double(c.z*8-2)}*grid.pitch;
   box.hi=grid.origin+Vec{double(c.x*8+10),double(c.y*8+10),double(c.z*8+10)}*grid.pitch;
   std::vector<std::uint32_t> found;asset.source_tree.query(0,box,found);selected.insert(found.begin(),found.end());}
  if(!std::equal(originals.begin(),originals.end(),selected.begin(),selected.end()))throw std::logic_error("GFY_RX_VERIFY patch selection mismatch");}
#endif
}
}}
// B1: per-region exact joins (needs boundaryFast above).
#include "RegionJoin.h"
namespace gfy_live { namespace bound_surface {
// Join path override for verification builds: 0 = as configured
// (GFY_SEP_REGION_JOIN), 1 = the union join, 2 = per-region joins.
inline int&joinPathOverride(){static int mode=0;return mode;}
// ---- prepare(): the P2 union-path surface stage ------------------------------
inline Packed prepareFast(const Asset&asset,const Grid&grid,const gfy_connectivity::Topology&volume,
                          const Bindings&bindings,double minVolume,const GarmentAdmission*garments,const ExtraSurface*extra,
                          const SurfaceOptions&options){
 const unsigned maxThreads=options.max_threads;
#ifdef GFY_COLLIDER_DIAGNOSTICS
 // S0: declared first, destroyed last: times the destruction of every local
 // after the "pack" mark (the formerly unmarked frees).
 struct FreesMark{core::Clock::time_point t{};bool armed{};~FreesMark(){if(armed)std::cerr<<"surface_stage frees ms="<<core::ms(t)<<std::endl;}}freesMark;
#endif
 need(bindings.source==asset.source_hash&&bindings.vertices.size()==asset.source.v.size(),"source_partition_identity");
 need(bindings.coat_faces.empty()||garments,"coat_partition_missing_element_admission");
 std::set<std::uint32_t>retained;
 for(std::uint32_t i=1;i<volume.components.size();++i)if(volume.components[i].cells*grid.pitch*grid.pitch*grid.pitch>=minVolume)retained.insert(i);
 need(retained.size()>=2,"no_substantial_anatomy_separation");
 #ifdef GFY_COLLIDER_DIAGNOSTICS
 auto mark=[t0=core::Clock::now()](const char*n)mutable{std::cerr<<"surface_stage "<<n<<" ms="<<core::ms(t0)<<std::endl;t0=core::Clock::now();};
 // S0 sub-stage marks (own timer; the reference-named marks above keep their meaning).
 auto sub=[t0=core::Clock::now()](const char*n)mutable{if(n)std::cerr<<"surface_stage "<<n<<" ms="<<core::ms(t0)<<std::endl;t0=core::Clock::now();};
 #else
 auto mark=[](const char*){};auto sub=[](const char*){};
 #endif
 // ---- boundary (S5 integer keys; the int14t path for owned grids) ----
 std::vector<core::Quad>boundary;boundaryQuads(grid,maxThreads,boundary);
 mark("boundary");sub(nullptr);
 // ---- join (the IndexedJoins are kept; nothing is materialized) ----
 if(grid.ownership&&!grid.ownership->source_is_piece_local)
  throw RecoverableImpactError("piece_join_requires_piece_local_source");
 // B1 (separation/RegionJoin.h): one exact join per region of removed tiles,
 // cached by content, in parallel. Owned grids, assets without incremental
 // wound data and GFY_SEP_REGION_JOIN=0 take the one union join as before.
 std::vector<std::uint32_t>originals,untouched;std::size_t passiveCount=0;
 core::IndexedJoin indexed; // the union join (union path only)
 region_join::RegionJoinSet regionSet;std::vector<JoinPart> parts;
 std::size_t gateOpen=0,gateNonmanifold=0;double gateConflict=0;
 const int joinMode=joinPathOverride();
 bool regional=joinMode==2||(joinMode==0&&region_join::regionJoinEnabled());
 if(regional)regional=region_join::findRegions(asset,grid,maxThreads,regionSet.set);
 if(regional){
  {std::size_t active=0;for(const auto&r:regionSet.set.regions)active+=r.faces.size();need(active,"fullbody_cut_without_exterior_patch");}
  sub("join_regions_find");
  region_join::joinRegions(asset,grid,boundary,maxThreads,regionSet);
  boundary=std::vector<core::Quad>();
  sub("join_indexed");
  untouched=regionSet.set.passive;passiveCount=untouched.size();
  untouched.insert(untouched.end(),regionSet.set.unselected.begin(),regionSet.set.unselected.end());
  for(const auto&join:regionSet.joins){parts.push_back({&join->indexed,&join->faces});
   gateOpen+=join->indexed.stats.newOpen;gateNonmanifold+=join->indexed.stats.nonmanifold;gateConflict=std::max(gateConflict,join->indexed.stats.maxJoinWeightConflict);}
#ifdef GFY_COLLIDER_DIAGNOSTICS
  std::cerr<<"region_join regions="<<regionSet.set.regions.size()<<" hits="<<regionSet.hits<<" misses="<<regionSet.misses<<" unlocated="<<regionSet.unlocated
           <<" passive="<<passiveCount<<" unselected="<<regionSet.set.unselected.size()<<" cached_mb="<<double(regionSet.cachedBytes)/1048576.
           <<" split_ms="<<regionSet.split_ms<<" join_ms="<<regionSet.join_ms<<std::endl;
  for(std::size_t r=0;r<regionSet.joins.size();++r){const auto&j=*regionSet.joins[r];const auto&reg=regionSet.set.regions[r];
   std::cerr<<"region_join_region "<<r<<" tiles="<<reg.keys.size()<<" cells="<<reg.cells<<" faces="<<reg.faces.size()<<" quads="<<regionSet.globalRect[r].size()
            <<" tris="<<j.indexed.triangles.size()<<" verts="<<j.indexed.vertices.size()+j.indexed.extra.size()<<" mb="<<double(j.bytes)/1048576.
            <<" exterior_ms="<<j.exterior_ms<<" join_ms="<<j.join_ms<<" open="<<j.indexed.stats.newOpen<<" nonmanifold="<<j.indexed.stats.nonmanifold<<" conflict="<<j.indexed.stats.maxJoinWeightConflict<<std::endl;}
#endif
 }else{
  selectPatch(asset,grid,maxThreads,originals,untouched);
  need(!originals.empty(),"fullbody_cut_without_exterior_patch");
  Source patch;
  patch.bones=asset.source.bones;patch.v=asset.source.v;patch.f.reserve(originals.size());
  for(auto face:originals)patch.f.push_back(asset.source.f[face]);
  sub("join_selection");
  auto outside=core::exterior(patch,grid);sub("join_exterior");
  indexed=core::joinIndexed(patch,asset.envelope_tree,grid,outside,boundary);sub("join_indexed");
  outside=core::ClipResult{};patch=Source{};boundary=std::vector<core::Quad>();
  // The former materialization checked every material-0 corner's patch face
  // ("fullbody_patch_provenance") before the parent geometry gate.
  {std::atomic<bool>bad{false};const auto patchFaces=originals.size();
   sep::parallel(indexed.triangles.size(),16384,maxThreads,[&](std::size_t first,std::size_t last,std::size_t){
    for(std::size_t i=first;i<last;++i){const auto&t=indexed.triangles[i];if(t.material)continue;
     if(t.v[0]->sourceFace>=patchFaces||t.v[1]->sourceFace>=patchFaces||t.v[2]->sourceFace>=patchFaces){bad=true;return;}}});
   need(!bad,"fullbody_patch_provenance");}
  sub("join_provenance");
  parts.push_back({&indexed,&originals});
  gateOpen=indexed.stats.newOpen;gateNonmanifold=indexed.stats.nonmanifold;gateConflict=indexed.stats.maxJoinWeightConflict;
 }
 mark("join");sub(nullptr);
#ifdef GFY_COLLIDER_DIAGNOSTICS
 std::cerr<<"parent_join_indexed new_open="<<gateOpen<<" nonmanifold="<<gateNonmanifold<<" conflict="<<gateConflict<<std::endl;
#endif
 need(gateOpen<=8&&gateNonmanifold<=8&&gateConflict<1e-6,"bound_partition_parent_geometry_gate"); // a few defects from rough cut edges must not cancel a sever
 const bool extraValid=extra&&extra->bindings.source==asset.source_hash&&extra->bindings.vertices.size()==extra->source.v.size()&&extra->source.bones==asset.source.bones
  &&extra->face_children.size()==extra->source.f.size()&&extra->face_indices.size()==extra->source.f.size();
 UnionTriangles tris(asset,extra,extraValid,std::move(parts),untouched);
 // The former joined order of the references [0,joined): the union join's
 // exterior fragments in patch face order (B1: every region's exterior
 // triangles and the passive faces, merged by source face), then the cavity
 // triangles in whole-grid rectangle order (B1: merged by rectangle index),
 // then the unselected faces ascending. A cavity triangle the forward scan
 // could not place (never observed) makes the cavity order region-major:
 // deterministic and valid, but not the union order.
 std::vector<std::uint32_t> replayOrder;replayOrder.reserve(tris.joined);
 if(regional){
  std::vector<std::uint64_t> ext,cav;ext.reserve(tris.joined);cav.reserve(tris.base);
  const bool located=regionSet.unlocated==0;
  for(std::size_t pi=0;pi<tris.parts.size();++pi){const auto&part=tris.parts[pi];const auto&join=*regionSet.joins[pi];const auto&globalRect=regionSet.globalRect[pi];
   for(std::uint32_t t=0;t<join.indexed.triangles.size();++t){const std::uint32_t r=part.triBase+t;const auto&it=join.indexed.triangles[t];
    if(!it.material)ext.push_back((std::uint64_t((*part.originals)[it.v[0]->sourceFace])<<32)|r);
    else cav.push_back(located?((std::uint64_t(globalRect[join.rectOf[t]])<<32)|r):std::uint64_t(r));}}
  for(std::uint32_t i=0;i<passiveCount;++i){const std::uint32_t r=tris.base+i;ext.push_back((std::uint64_t(untouched[i])<<32)|r);}
  std::sort(ext.begin(),ext.end());std::sort(cav.begin(),cav.end());
  for(const auto e:ext)replayOrder.push_back(std::uint32_t(e&0xFFFFFFFFu));
  for(const auto c:cav)replayOrder.push_back(std::uint32_t(c&0xFFFFFFFFu));
  for(std::uint32_t i=std::uint32_t(passiveCount);i<untouched.size();++i)replayOrder.push_back(tris.base+i);
 }else for(std::uint32_t r=0;r<tris.joined;++r)replayOrder.push_back(r);
 need(replayOrder.size()==tris.joined,"bound_partition_replay_order");
 sub("join_order");
 // Exact lookup accelerators for this stage (S2/S3): column rows of the
 // remaining topology, a retained-component mask, tile existence bits.
 const surface_support::ColumnRows columnRows(volume);
 std::vector<std::uint8_t>retainedMask(volume.components.size(),0);for(const auto id:retained)if(id<retainedMask.size())retainedMask[id]=1;
 const surface_support::TileIndex tileIndex(grid);
 const std::vector<std::uint32_t>retainedIds(retained.begin(),retained.end());
 const PartitionContext ctx{grid,volume,columnRows,tileIndex,retainedMask,retainedIds};
 // ---- partition: compute every triangle's outcome in parallel, replay serially ----
 surface_support::Budget budget;budget.limit=160000000;Stats st;
 struct Scratch {core::TileCache tiles;surface_support::CavityScratch cavity;PartitionScratch part;}; // one per worker thread
 auto computeAll=[&](std::size_t count,const auto&one,const auto&verify){
  std::vector<Outcome>outcomes(count);for(auto&o:outcomes)o.budget.limit=budget.limit;
  constexpr std::size_t chunk=256;const std::size_t chunks=(count+chunk-1)/chunk;std::atomic<std::size_t>cursor{0};
  sep::parallel(count>=1024?4:1,1,maxThreads,[&](std::size_t,std::size_t,std::size_t){
   Scratch scratch{core::TileCache(grid),{},{}};
#ifdef GFY_RX_PROF
   {static std::mutex m;std::lock_guard<std::mutex> g(m);auto&all=earlyProfiles();if(std::find(all.begin(),all.end(),&earlyProfile())==all.end())all.push_back(&earlyProfile());}
#endif
   for(std::size_t c;(c=cursor.fetch_add(1))<chunks;)for(std::size_t i=c*chunk;i<std::min(count,(c+1)*chunk);++i){auto&o=outcomes[i];
#ifdef GFY_RX_PROF
    const auto p0=core::Clock::now();
#endif
    try{one(i,o,scratch);}catch(...){o.error=std::current_exception();}
#ifdef GFY_RX_PROF
    o.ms=float(core::ms(p0));
#endif
   }});
#ifdef GFY_RX_VERIFY
  for(std::size_t i=0;i<count;++i)verify(i,outcomes[i]);
#else
  (void)verify;
#endif
  return outcomes;};
 auto joinedOne=[&](std::size_t index,Outcome&o,Scratch&s){
  const auto r=std::uint32_t(index);
  if(r<tris.base&&tris.joinTriangle(r).material){
   const auto&it=tris.joinTriangle(r);std::uint32_t owner=0;
   if(surface_support::cavityOwner(it.material,it.v.data(),grid,columnRows,s.tiles,retainedMask,o.budget,owner,s.cavity)){
#ifdef GFY_RX_PROF
    o.path=owner?5:6;
#endif
    o.area=tris.area(r);o.kind=owner?Outcome::kept:Outcome::air;o.owner=owner;return;
   }
#ifdef GFY_RX_PROF
   o.path=7;
#endif
   core::JoinedTriangle t;tris.triangle(r,t);
   auto contacts=surface_support::cavity(t,grid,volume,o.budget);
   std::set<std::uint32_t>owners;for(auto c:contacts)if(retained.count(c.first))owners.insert(c.first);
   if(owners.size()==1){o.kind=Outcome::kept;o.owner=*owners.begin();o.area=area(t);return;}
   if(owners.empty()){o.kind=Outcome::air;o.area=area(t);return;}
   // Cavity surfaces sit exactly on cell boundaries: querying their inward
   // support needs the opposite normal half-cell offset.
   std::array<Vec,3>anchors;
   for(int k=0;k<3;++k){auto n=t.v[k].normal;need(length(n)>1e-10,"partition_cavity_normal");anchors[k]=t.v[k].p-n*(grid.pitch*.5/length(n));}
   partitionFast(anchors,area(t),ctx,s.part,o,[&]()->const core::JoinedTriangle&{return t;});
   return;
  }
  const auto face=tris.sourceFace(r,0);need(face<asset.source.f.size(),"source_partition_face");
  std::array<Vec,3>anchors;
  for(int k=0;k<3;++k){need(tris.sourceFace(r,k)==face,"source_partition_mixed_provenance");double sum=0;const auto&bary=tris.sourceBary(r,k);
   for(int j=0;j<3;++j){const auto b=bary[std::size_t(j)];need(std::isfinite(b)&&b>=-1e-6&&b<=1.000001,"source_partition_barycentric");sum+=b;
    anchors[k]=anchors[k]+bindings.vertices[asset.source.f[face][std::size_t(j)]].point*b;}
   need(std::abs(sum-1)<1e-6,"source_partition_barycentric_sum");}
  std::optional<core::JoinedTriangle>full;
  partitionFast(anchors,tris.area(r),ctx,s.part,o,[&]()->const core::JoinedTriangle&{if(!full){full.emplace();tris.triangle(r,*full);}return *full;});
 };
 auto extraOne=[&](std::size_t f,Outcome&o,Scratch&s){
  const auto r=tris.joined+std::uint32_t(f);std::array<Vec,3>anchors;
  for(unsigned k=0;k<3;++k)anchors[k]=extra->bindings.vertices[extra->source.f[f][k]].point;
  std::optional<core::JoinedTriangle>full;
  partitionFast(anchors,tris.area(r),ctx,s.part,o,[&]()->const core::JoinedTriangle&{if(!full){full.emplace();tris.triangle(r,*full);}return *full;});
 };
#ifdef GFY_RX_VERIFY
 auto verifyJoined=[&](std::size_t i,const Outcome&o){core::JoinedTriangle t;tris.triangle(std::uint32_t(i),t);
  Result r;r.deferKeep=true;surface_support::Budget b;b.limit=budget.limit;std::exception_ptr e;
  try{referenceJoinedOne(t,asset,grid,volume,bindings,retained,r,b);}catch(...){e=std::current_exception();}
  verifyOutcome("joined",i,o,r,b,e);};
 auto verifyExtra=[&](std::size_t f,const Outcome&o){Result r;surface_support::Budget b;b.limit=budget.limit;std::exception_ptr e;
  try{referenceExtraOne(*extra,asset,f,grid,volume,retained,r);}catch(...){e=std::current_exception();}
  verifyOutcome("extra",f,o,r,b,e);};
#else
 auto verifyJoined=[](std::size_t,const Outcome&){};auto verifyExtra=[](std::size_t,const Outcome&){};
#endif
 auto joinedOutcomes=computeAll(tris.joined,joinedOne,verifyJoined);
#ifdef GFY_RX_PROF
 {static const char*names[]={"setup","foreign","positive","covers","full"};EarlyProfile sum;
  for(auto*p:earlyProfiles()){for(int i=0;i<6;++i){sum.ms[i]+=p->ms[i];sum.n[i]+=p->n[i];}*p=EarlyProfile{};}
  for(int i=0;i<5;++i)std::cerr<<"rxprof early."<<names[i]<<" ms="<<sum.ms[i]<<" n="<<sum.n[i]<<std::endl;}
#endif
 std::vector<Outcome>extraOutcomes;if(extraValid)extraOutcomes=computeAll(extra->source.f.size(),extraOne,verifyExtra);
 sub("partition_compute");
#ifdef GFY_RX_PROF
 {static const char*names[]={"other","early_kept","full_kept","split","trimmed","single_owner","air","cavity_reference"};
  std::map<std::string,std::pair<std::size_t,double>>agg;
  auto add=[&](const std::vector<Outcome>&outcomes,bool joinedSet){
   for(std::size_t i=0;i<outcomes.size();++i){const auto&o=outcomes[i];std::string name=joinedSet?"joined":"extra";
    if(joinedSet)name+=i<tris.base?(tris.joinTriangle(std::uint32_t(i)).material?"_cavity":"_patch"):"_untouched";
    name+='_';name+=o.error?"error":names[o.path<8?o.path:0];auto&a=agg[name];++a.first;a.second+=o.ms;}};
  add(joinedOutcomes,true);add(extraOutcomes,false);
  for(const auto&a:agg)std::cerr<<"rxprof class "<<a.first<<" count="<<a.second.first<<" ms="<<a.second.second<<std::endl;}
#endif
 // Serial replay in the former triangle order: the same counter updates, area
 // additions, budget checks and first failure as the int14t record replay.
 std::map<std::uint32_t,std::vector<std::uint32_t>> meshes; // owner -> triangle references in replay order
 std::vector<std::unique_ptr<SplitOut>> splitRecords;
 auto replay=[&](Outcome&o,std::uint32_t ref){
  budget.visited+=o.budget.visited;need(budget.visited<=budget.limit,"support_face_budget");
  st.work+=o.work;need(st.work<=160000000,"source_partition_work_budget");
  if(o.error)std::rethrow_exception(o.error);
  if(o.kind==Outcome::kept){
   st.input+=1;st.original_area+=o.area;st.air_cavity+=0;st.air_cavity_area+=0.;
   st.emitted_area+=o.area;++st.output;meshes[o.owner].push_back(ref);
   st.kept+=1;st.split+=0;st.trimmed+=0;st.trimmed_area+=0.;
  }else if(o.kind==Outcome::air){
   st.input+=0;st.original_area+=0.;st.air_cavity+=1;st.air_cavity_area+=o.area;
   st.kept+=0;st.split+=0;st.trimmed+=0;st.trimmed_area+=0.;
  }else{
   auto&side=*o.side; // kind==split: every non-failed outcome is kept, air or split
   st.input+=1;st.original_area+=o.area;st.air_cavity+=0;st.air_cavity_area+=0.;
   for(auto&e:side.triangles){need(st.output<4000000,"source_partition_triangle_budget");
    st.emitted_area+=area(e.second);++st.output;
    tris.splits.push_back(&e.second);meshes[e.first].push_back(tris.splitBase+std::uint32_t(tris.splits.size()-1));}
   st.kept+=0;st.split+=side.split;st.trimmed+=side.trimmed;st.trimmed_area+=side.trimmed_area;
   splitRecords.push_back(std::move(o.side));
  }
 };
#ifdef GFY_RX_PROF
 const auto rxReplay0=core::Clock::now();
#endif
 for(const auto r:replayOrder)replay(joinedOutcomes[r],r);
 joinedOutcomes=std::vector<Outcome>();replayOrder=std::vector<std::uint32_t>();
#ifdef GFY_RX_PROF
 std::cerr<<"rxprof partition replay="<<core::ms(rxReplay0)<<" triangles="<<tris.joined<<" vertices="<<tris.vertexCount()<<std::endl;
#endif
 sub("partition_replay");
 mark("partition");auto faceChildren=asset.face_children;auto faceIndices=asset.face_indices;
 if(extra){
  need(extra->bindings.source==asset.source_hash&&extra->bindings.vertices.size()==extra->source.v.size()&&extra->source.bones==asset.source.bones,"accessory_partition_identity");
  need(extra->face_children.size()==extra->source.f.size()&&extra->face_indices.size()==extra->source.f.size(),"accessory_partition_provenance");
  for(std::size_t i=0;i<extraOutcomes.size();++i)replay(extraOutcomes[i],tris.joined+std::uint32_t(i));
  extraOutcomes=std::vector<Outcome>();
  faceChildren.insert(faceChildren.end(),extra->face_children.begin(),extra->face_children.end());
  faceIndices.insert(faceIndices.end(),extra->face_indices.begin(),extra->face_indices.end());
 }
 // Coats have no independent cloth solver (see trimUnsupportedCoat): the
 // same predicate, order and stats updates on triangle references.
 mark("extra");sub(nullptr);
 if(!bindings.coat_faces.empty()){
  std::array<double,64>scratch;
  for(auto&entry:meshes){
   const auto admitted=garments->component_elements.find(entry.first);
   need(admitted!=garments->component_elements.end(),"coat_partition_missing_component_elements");
   auto unsupported=[&](std::uint32_t r){
    if(tris.material(r))return false;const auto face=tris.sourceFace(r,0);
    if(face>=bindings.coat_faces.size()||!bindings.coat_faces[face])return false;
    for(int k=0;k<3;++k){const auto&w=tris.weights(r,k,scratch);
     for(std::size_t bone=0;bone<bindings.bones;++bone)if((w[bone]>1e-6||w[bone]<0)&&!admitted->second.count(garments->bone_to_element[bone]))return true;}
    return false;};
   auto&refs=entry.second;
   refs.erase(std::remove_if(refs.begin(),refs.end(),[&](std::uint32_t r){
    if(!unsupported(r))return false;
    const auto a=tris.area(r);st.emitted_area-=a;st.trimmed_area+=a;--st.output;++st.unsupported_coat_triangles;return true;}),refs.end());
  }
 }
 sub("trim_coat");
 Packed packed;packed.stats=st;packed.garment_roots_validated=!bindings.coat_faces.empty()&&garments;
 std::vector<std::pair<const std::uint32_t,std::vector<std::uint32_t>>*>entries;for(auto&entry:meshes)entries.push_back(&entry);
 packed.pieces.resize(entries.size());
 if(const char*prefix=std::getenv("GFY_BOUND_PIECE_DUMP"))for(auto*entry:entries){
  core::JoinResult dump;dump.triangles.resize(entry->second.size());
  for(std::size_t i=0;i<entry->second.size();++i)tris.triangle(entry->second[i],dump.triangles[i]);
  core::writeJoined(std::string(prefix)+"-"+std::to_string(entry->first)+".joined",dump,asset.source.bones);}
 // Packed rows (S1 memo): each IndexedJoin vertex used by a kept triangle is
 // converted once; untouched, accessory and split triangles convert their
 // three corners once. Slots are assigned in id order, then filled in parallel.
 const std::uint64_t vertexCount=tris.vertexCount();
 std::vector<std::uint32_t> vertexSlot(vertexCount,UINT32_MAX),ownSlot(tris.splitBase+tris.splits.size()-tris.base,UINT32_MAX);
 for(const auto&entry:meshes)for(const auto r:entry.second){
  if(r<tris.base){for(int k=0;k<3;++k)vertexSlot[tris.vertexId(r,k)]=0;}
  else ownSlot[r-tris.base]=0;}
 std::vector<std::uint64_t> jobs;
 for(std::uint32_t id=0;id<vertexSlot.size();++id)if(vertexSlot[id]!=UINT32_MAX){vertexSlot[id]=std::uint32_t(jobs.size());jobs.push_back(id);}
 for(std::uint32_t o=0;o<ownSlot.size();++o)if(ownSlot[o]!=UINT32_MAX){ownSlot[o]=std::uint32_t(jobs.size());for(std::uint64_t k=0;k<3;++k)jobs.push_back(vertexCount+3ull*o+k);}
 sub("pack_slots");
 piece_pack::RowStore rows;
 rows.fill(jobs.size(),maxThreads,[&](std::size_t slot,piece_pack::Row&row){const auto job=jobs[slot];
  if(job<vertexCount)piece_pack::convert(tris.vertexOf(std::uint32_t(job)),row);
  else{const auto own=(job-vertexCount)/3;piece_pack::convert(tris.corner(tris.base+std::uint32_t(own),int((job-vertexCount)%3)),row);}});
 sub("pack_rows");
 struct PackSource {
  const UnionTriangles&t;const std::vector<std::uint32_t>&vertexSlot,&ownSlot;const piece_pack::RowStore&rows;
  unsigned material(std::uint32_t r)const{return t.material(r);}
  std::uint32_t sourceFace(std::uint32_t r,int k)const{return t.sourceFace(r,k);}
  const piece_pack::Row&row(std::uint32_t r,int k)const{
   if(r<t.base)return rows.at(vertexSlot[t.vertexId(r,k)]);
   return rows.at(ownSlot[r-t.base]+std::uint32_t(k));}
 };
 const PackSource source{tris,vertexSlot,ownSlot,rows};
 // S5: a piece the caller does not want (SurfaceOptions::wanted; the owned
 // continuation) is never read (LiveSeparationWorker skips it). It is not
 // assembled when packRefs() provably cannot fail on it: the child-map and
 // per-triangle group checks pass, none of its rows failed conversion (rows
 // are still converted, so every conversion failure is kept), and even the
 // most child splits possible (one per 21844 triangles of a group: a flushed
 // child holds >= 65533 rows or 65535 triangles) cannot exhaust the child
 // slots. Otherwise it is packed as before, so every failure is unchanged.
 auto cannotFail=[&](const std::vector<std::uint32_t>&refs){
  if(faceChildren.size()!=faceIndices.size()||faceChildren.empty()||!asset.original_children||asset.original_children>=gfy_packed::max_child_slots)return false;
  for(const auto child:faceChildren)if(child>=asset.original_children)return false;
  std::map<std::pair<unsigned,std::uint16_t>,std::size_t> groups;
  for(const auto r:refs){const unsigned material=source.material(r);std::uint16_t child=faceChildren.front();
   if(!material){const auto face=source.sourceFace(r,0);
    if(face>=faceChildren.size()||source.sourceFace(r,1)!=face||source.sourceFace(r,2)!=face)return false;child=faceChildren[face];}
   if(material>=64)return false;++groups[{material,child}];
   for(int k=0;k<3;++k)if(source.row(r,k).error)return false;}
  std::size_t appended=0;for(const auto&g:groups)appended+=(g.first.first?1:0)+g.second/21844;
  return asset.original_children+appended<gfy_packed::max_child_slots;};
 // Pieces pack independently (up to four threads, contiguous in map order;
 // the first failure in piece order is rethrown, as in the serial loop).
 sep::parallel(entries.size(),1,maxThreads,[&](std::size_t first,std::size_t last,std::size_t){
  for(std::size_t i=first;i<last;++i){auto&p=packed.pieces[i];const auto&entry=*entries[i];p.volume_component=entry.first;p.triangles=entry.second.size();
   if(options.wanted&&!options.wanted->count(entry.first)&&cannotFail(entry.second))continue;
   p.mesh=piece_pack::packRefs(entry.second,source,faceChildren,faceIndices,asset.original_children);}});
 sub("pack_assemble");
 mark("pack");
#ifdef GFY_COLLIDER_DIAGNOSTICS
 freesMark.t=core::Clock::now();freesMark.armed=true;
#endif
 for(auto id:retained)need(meshes.count(id)!=0,"bound_partition_missing_piece");
 need(std::abs(st.original_area-st.emitted_area-st.trimmed_area)<1e-6,"bound_partition_area_conservation");
 return packed;
}
// GFY_RX_VERIFY: prepareFast() and the verbatim int14t prepareReference() on
// the same inputs must return identical pieces (every mesh byte), bitwise
// identical Stats, or the same failure (exception type and message).
inline Packed verifyPrepare(const Asset&asset,const Grid&grid,const gfy_connectivity::Topology&volume,
                            const Bindings&bindings,double minVolume,const GarmentAdmission*garments,const ExtraSurface*extra,
                            const SurfaceOptions&options){
 Packed fast,reference;std::exception_ptr fastError,referenceError;
 struct Mode{int previous;explicit Mode(int m):previous(joinPathOverride()){joinPathOverride()=m;}~Mode(){joinPathOverride()=previous;}};
 try{const Mode unionJoin(1);fast=prepareFast(asset,grid,volume,bindings,minVolume,garments,extra,options);}catch(...){fastError=std::current_exception();}
 try{reference=prepareReference(asset,grid,volume,bindings,minVolume,garments,extra,options);}catch(...){referenceError=std::current_exception();}
 const auto a=failureOf(fastError),b=failureOf(referenceError);
 if(a!=b)throw std::logic_error("GFY_RX_VERIFY prepare failure mismatch: fast='"+a+"' reference='"+b+"'");
 if(fastError){std::cerr<<"GFY_RX_VERIFY prepare identical failure: "<<a<<std::endl;std::rethrow_exception(fastError);}
 if(!sameStats(fast.stats,reference.stats)||fast.garment_roots_validated!=reference.garment_roots_validated||fast.pieces.size()!=reference.pieces.size())
  throw std::logic_error("GFY_RX_VERIFY prepare stats or piece count mismatch");
 std::size_t triangles=0;
 for(std::size_t i=0;i<fast.pieces.size();++i){const auto&x=fast.pieces[i];const auto&y=reference.pieces[i];
  // An unwanted piece's mesh is never read and may be left unassembled (S5).
  const bool wanted=!options.wanted||options.wanted->count(x.volume_component);
  if(x.volume_component!=y.volume_component||x.triangles!=y.triangles||(wanted&&!piece_pack::sameMesh(x.mesh,y.mesh)))
   throw std::logic_error("GFY_RX_VERIFY prepare piece mismatch at "+std::to_string(i));
  triangles+=x.triangles;}
 std::cerr<<"GFY_RX_VERIFY prepare identical pieces="<<fast.pieces.size()<<" triangles="<<triangles<<" work="<<fast.stats.work<<std::endl;
 return fast;
}
// GFY_RX_VERIFY_REGIONS: the per-region join path (B1) against the union join
// on the same inputs. Every piece is compared byte for byte; where they differ,
// the pieces' triangles are compared as sorted multisets (material, source
// child, original face, the three corners rotated to start at the smallest
// corner, winding kept) once over positions/normals/UVs only and once over the
// whole packed vertex, and the skin-weight differences of vertices at equal
// positions are summarized. Prints one line per piece; throws on a different
// piece count, piece identity or triangle multiset. Returns the region result.
inline std::vector<std::string> triangleKeys(const gfy_packed::Mesh&m,bool full){
 std::vector<std::string> keys;const std::size_t n=full?sizeof(gfy_packed::Vertex):sizeof(float)*14;
 for(const auto&c:m.children)for(std::size_t t=0;t+2<c.indices.size();t+=3){
  const gfy_packed::Vertex*v[3]={&c.vertices[c.indices[t]],&c.vertices[c.indices[t+1]],&c.vertices[c.indices[t+2]]};
  int start=0;for(int k=1;k<3;++k)if(std::memcmp(v[k],v[start],n)<0)start=k;
  std::string key;key.append(reinterpret_cast<const char*>(&c.material),4);key.append(reinterpret_cast<const char*>(&c.material_source_child),2);
  key.append(reinterpret_cast<const char*>(&c.original_faces[t/3]),4);
  for(int k=0;k<3;++k)key.append(reinterpret_cast<const char*>(v[(start+k)%3]),n);
  keys.push_back(std::move(key));}
 std::sort(keys.begin(),keys.end());return keys;
}
struct WeightDiff {std::size_t vertices{},differing{},cavityDiffering{},unmatched{};double maxDelta{};};
inline WeightDiff weightDiff(const gfy_packed::Mesh&a,const gfy_packed::Mesh&b){
 auto full=[](const gfy_packed::Vertex&v){std::array<float,64> w{};for(int k=0;k<4;++k)if(v.bone[k]<64)w[v.bone[k]]+=v.weight[k];return w;};
 std::unordered_map<std::string,std::vector<std::array<float,64>>> byPosition;
 for(const auto&c:b.children)for(const auto&v:c.vertices)byPosition[std::string(reinterpret_cast<const char*>(v.surface.data()),12)].push_back(full(v));
 WeightDiff out;
 for(const auto&c:a.children)for(const auto&v:c.vertices){++out.vertices;auto it=byPosition.find(std::string(reinterpret_cast<const char*>(v.surface.data()),12));
  if(it==byPosition.end()){++out.unmatched;continue;}
  const auto wa=full(v);double best=1e300;
  for(const auto&wb:it->second){double d=0;for(int k=0;k<64;++k)d=std::max(d,double(std::abs(wa[k]-wb[k])));best=std::min(best,d);}
  if(best>0){++out.differing;if(c.material)++out.cavityDiffering;out.maxDelta=std::max(out.maxDelta,best);}}
 return out;
}
inline Packed verifyRegions(const Asset&asset,const Grid&grid,const gfy_connectivity::Topology&volume,
                            const Bindings&bindings,double minVolume,const GarmentAdmission*garments,const ExtraSurface*extra,
                            const SurfaceOptions&options){
 struct Mode{int previous;explicit Mode(int m):previous(joinPathOverride()){joinPathOverride()=m;}~Mode(){joinPathOverride()=previous;}};
 Packed regional,unionResult;std::exception_ptr regionalError,unionError;
 const auto t0=core::Clock::now();
 try{const Mode mode(2);regional=prepareFast(asset,grid,volume,bindings,minVolume,garments,extra,options);}catch(...){regionalError=std::current_exception();}
 const double regionalMs=core::ms(t0);const auto t1=core::Clock::now();
 try{const Mode mode(1);unionResult=prepareFast(asset,grid,volume,bindings,minVolume,garments,extra,options);}catch(...){unionError=std::current_exception();}
 const double unionMs=core::ms(t1);
 const auto a=failureOf(regionalError),b=failureOf(unionError);
 std::cerr<<"GFY_RX_VERIFY_REGIONS regional_ms="<<regionalMs<<" union_ms="<<unionMs<<" regional_failure='"<<a<<"' union_failure='"<<b<<"'"<<std::endl;
 if(regionalError){if(a!=b)std::cerr<<"GFY_RX_VERIFY_REGIONS failure differs"<<std::endl;std::rethrow_exception(regionalError);}
 if(unionError)throw std::logic_error("GFY_RX_VERIFY_REGIONS union path failed where the region path succeeded: "+b);
 if(regional.pieces.size()!=unionResult.pieces.size())throw std::logic_error("GFY_RX_VERIFY_REGIONS piece count mismatch");
 const bool sameStatsFlag=sameStats(regional.stats,unionResult.stats);
 std::cerr<<"GFY_RX_VERIFY_REGIONS stats "<<(sameStatsFlag?"identical":"DIFFERENT")<<" output="<<regional.stats.output<<"/"<<unionResult.stats.output
          <<" kept="<<regional.stats.kept<<"/"<<unionResult.stats.kept<<" split="<<regional.stats.split<<"/"<<unionResult.stats.split
          <<" trimmed="<<regional.stats.trimmed<<"/"<<unionResult.stats.trimmed<<" air="<<regional.stats.air_cavity<<"/"<<unionResult.stats.air_cavity<<std::endl;
 for(std::size_t i=0;i<regional.pieces.size();++i){const auto&x=regional.pieces[i];const auto&y=unionResult.pieces[i];
  if(x.volume_component!=y.volume_component)throw std::logic_error("GFY_RX_VERIFY_REGIONS piece identity mismatch at "+std::to_string(i));
  const bool wanted=!options.wanted||options.wanted->count(x.volume_component);
  if(!wanted){std::cerr<<"GFY_RX_VERIFY_REGIONS piece "<<x.volume_component<<" unwanted (not assembled)"<<std::endl;continue;}
  if(piece_pack::sameMesh(x.mesh,y.mesh)){std::cerr<<"GFY_RX_VERIFY_REGIONS piece "<<x.volume_component<<" identical triangles="<<x.triangles<<std::endl;continue;}
  const auto ga=triangleKeys(x.mesh,false),gb=triangleKeys(y.mesh,false);
  const bool geometry=ga==gb,whole=triangleKeys(x.mesh,true)==triangleKeys(y.mesh,true);
  const auto w=weightDiff(x.mesh,y.mesh),wr=weightDiff(y.mesh,x.mesh);
  std::cerr<<"GFY_RX_VERIFY_REGIONS piece "<<x.volume_component<<" bytes=DIFFERENT triangles="<<x.triangles<<"/"<<y.triangles
           <<" multiset_geometry="<<(geometry?"identical":"DIFFERENT")<<" multiset_full="<<(whole?"identical":"DIFFERENT")
           <<" weight_vertices="<<w.vertices<<" differing="<<w.differing<<" cavity_differing="<<w.cavityDiffering<<" unmatched="<<w.unmatched<<"/"<<wr.unmatched<<" max_delta="<<w.maxDelta<<std::endl;
  if(!geometry){
   // Keys only on one side: counts by material, and the first examples decoded.
   std::vector<std::string> onlyA,onlyB;
   std::set_difference(ga.begin(),ga.end(),gb.begin(),gb.end(),std::back_inserter(onlyA));
   std::set_difference(gb.begin(),gb.end(),ga.begin(),ga.end(),std::back_inserter(onlyB));
   auto describe=[&](const char*side,const std::vector<std::string>&keys){std::size_t cavity=0;for(const auto&k:keys){unsigned material=0;std::memcpy(&material,k.data(),4);cavity+=material!=0;}
    std::cerr<<"GFY_RX_VERIFY_REGIONS   only_"<<side<<" triangles="<<keys.size()<<" cavity="<<cavity<<std::endl;
    for(std::size_t i=0;i<keys.size()&&i<3;++i){const auto&k=keys[i];unsigned material=0;std::uint16_t child=0;std::uint32_t face=0;
     std::memcpy(&material,k.data(),4);std::memcpy(&child,k.data()+4,2);std::memcpy(&face,k.data()+6,4);
     std::cerr<<"GFY_RX_VERIFY_REGIONS     material="<<material<<" child="<<child<<" face="<<face;
     for(int c=0;c<3;++c){float s[14];std::memcpy(s,k.data()+10+std::size_t(c)*56,56);
      std::cerr<<" | p="<<s[0]<<","<<s[1]<<","<<s[2]<<" n="<<s[3]<<","<<s[4]<<","<<s[5]<<" t="<<s[6]<<","<<s[7]<<","<<s[8]<<" b="<<s[9]<<","<<s[10]<<","<<s[11]<<" uv="<<s[12]<<","<<s[13];}
     std::cerr<<std::endl;}};
   describe("regional",onlyA);describe("union",onlyB);
   // Position-only multiset (material, child, face, three positions): the triangulation itself.
   auto positionKeys=[](const gfy_packed::Mesh&m){std::vector<std::string> keys;
    for(const auto&c:m.children)for(std::size_t t=0;t+2<c.indices.size();t+=3){
     const gfy_packed::Vertex*v[3]={&c.vertices[c.indices[t]],&c.vertices[c.indices[t+1]],&c.vertices[c.indices[t+2]]};
     int start=0;for(int k=1;k<3;++k)if(std::memcmp(v[k],v[start],12)<0)start=k;
     std::string key;key.append(reinterpret_cast<const char*>(&c.material),4);key.append(reinterpret_cast<const char*>(&c.material_source_child),2);
     key.append(reinterpret_cast<const char*>(&c.original_faces[t/3]),4);
     for(int k=0;k<3;++k)key.append(reinterpret_cast<const char*>(v[(start+k)%3]),12);keys.push_back(std::move(key));}
    std::sort(keys.begin(),keys.end());return keys;};
   const bool positions=positionKeys(x.mesh)==positionKeys(y.mesh);
   std::cerr<<"GFY_RX_VERIFY_REGIONS   multiset_positions="<<(positions?"identical":"DIFFERENT")<<std::endl;
   if(!positions)throw std::logic_error("GFY_RX_VERIFY_REGIONS triangle position multiset mismatch at piece "+std::to_string(x.volume_component));
  }
 }
 return regional;
}
// options: separation-track interface hook (separation/SurfaceOptions.h).
// max_threads caps every parallel section of this stage (sep::parallel).
inline Packed prepare(const Asset&asset,const Grid&grid,const gfy_connectivity::Topology&volume,
                      const Bindings&bindings,double minVolume,const GarmentAdmission*garments=nullptr,const ExtraSurface*extra=nullptr,
                      const SurfaceOptions&options={}){
#ifdef GFY_RX_VERIFY
 return verifyPrepare(asset,grid,volume,bindings,minVolume,garments,extra,options);
#elif defined(GFY_RX_VERIFY_REGIONS)
 return verifyRegions(asset,grid,volume,bindings,minVolume,garments,extra,options);
#else
 return prepareFast(asset,grid,volume,bindings,minVolume,garments,extra,options);
#endif
}
}}

#endif
