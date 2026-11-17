#pragma once
#ifndef GFY_SEP_REGION_JOIN_INCLUDED
#define GFY_SEP_REGION_JOIN_INCLUDED
// Separation-track B1: per-region exact joins for the surface stage.
//
// The union-path surface stage (prepareFast below) joined every removed tile
// of the corpse as ONE exact core::joinIndexed over the union patch: measured
// 469-520 ms of a 586-650 ms stage on the thigh fixture, on one thread, and
// every retry or superseded attempt paid it again. This header replaces that
// single join by one exact join per independent REGION of removed tiles, run
// concurrently under the separation thread budget, each region's IndexedJoin
// cached process-wide by content.
//
// Region rule (the wound path's rule, FastWound_impl.h findRegions): a face
// SELECTED by a tile lies within two cells of it (the former patch selection);
// it is ACTIVE for the tile when its triangle comes within one cell of the tile
// (it can be clipped or receive a cut point) or is unclean; every union-selected
// face sharing a vertex position with an active face is active too (conform
// points can land on it). Tiles that are 26-neighbours, or that share an active
// face, form one region. Active faces are disjoint between regions and no join
// tolerance (all <= 1e-6 m) couples two regions. A selected face active in no
// region is PASSIVE: the exact join returns it exactly as its untouched rows
// (FastWoundData.h clean[]), so the partition takes it as an untouched face at
// its former patch position.
//
// Order. The union join emitted exterior fragments in patch face order, then
// the cavity polygons in the order of the merged boundary rectangles
// (boundaryFast: by axis, plane, sign, material, then the sweep's emission
// order over ALL tiles). Every region receives ITS rectangles in that whole-grid
// order (a rectangle never spans two regions), so a region's triangles come out
// in the union order restricted to the region, and the union order is
// recovered by merging: exterior triangles by source face (passive faces in
// between, by face), cavity triangles by whole-grid rectangle index, then the
// unselected faces. The cache key includes the region's rectangle sequence, so
// a cached value is a pure function of its key.
//
// Semantics: the kernel's documented region semantics (FastWound_impl.h:21-24).
// One region is byte-identical to the union join. With several regions the
// cavity-skin mode (conformed retry; harmonic smoothing vs seed copy) is decided
// per region instead of by the union of all wounds on the corpse, so wall skin
// weights of a region can differ from the union join's where the union smoothed
// because of ANOTHER region. Positions, provenance, materials and the partition
// rules are unchanged. The parent gate is the sum (open, nonmanifold) and the
// maximum (weight conflict) over regions.
#include <atomic>
#include <bitset>
#include <cstdlib>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include "ThreadBudget.h"
#include "SeparationCache.h"
namespace gfy_live { namespace bound_surface { namespace region_join {
// Conservative triangle / axis-aligned box overlap (separating axis test); the
// wound path's test, kept here so this stage does not depend on its header.
inline bool triangleBoxOverlap(Vec a,Vec b,Vec c,Vec lo,Vec hi){
 const Vec center=(lo+hi)*.5,half=(hi-lo)*.5;const Vec v0=a-center,v1=b-center,v2=c-center;
 const Vec e[3]={v1-v0,v2-v1,v0-v2};
 for(int k=0;k<3;++k){const double mn=std::min({v0[k],v1[k],v2[k]}),mx=std::max({v0[k],v1[k],v2[k]});if(mn>half[k]||mx<-half[k])return false;}
 const Vec n=cross(e[0],e[1]);const double r=half.x*std::abs(n.x)+half.y*std::abs(n.y)+half.z*std::abs(n.z);const double s=dot(n,v0);if(s>r||s<-r)return false;
 for(int i=0;i<3;++i)for(int k=0;k<3;++k){Vec axis;axis[k]=1;const Vec ax=cross(axis,e[i]);if(dot(ax,ax)<1e-30)continue;
  const double p0=dot(ax,v0),p1=dot(ax,v1),p2=dot(ax,v2);const double rr=half.x*std::abs(ax.x)+half.y*std::abs(ax.y)+half.z*std::abs(ax.z);
  if(std::min({p0,p1,p2})>rr||std::max({p0,p1,p2})<-rr)return false;}
 return true;
}
struct TileFaces {std::vector<std::uint32_t> selected,active;};
// Per tile (pure function of the tile key, asset and lattice): selected = faces
// whose bounds meet the tile box grown by two cells (the former patch
// selection, ascending); active = selected faces whose triangle meets the box
// grown by one cell, plus unclean ones.
inline void tileFaces(const Grid&grid,const core::BodyBVH&tree,const std::vector<char>&clean,Cell c,TileFaces&out){
 core::Bounds box;
 box.lo=grid.origin+Vec{double(c.x*8-2),double(c.y*8-2),double(c.z*8-2)}*grid.pitch;
 box.hi=grid.origin+Vec{double(c.x*8+10),double(c.y*8+10),double(c.z*8+10)}*grid.pitch;
 out.selected.clear();out.active.clear();tree.query(0,box,out.selected);std::sort(out.selected.begin(),out.selected.end());
 const Vec lo=grid.origin+Vec{double(c.x*8-1),double(c.y*8-1),double(c.z*8-1)}*grid.pitch,hi=grid.origin+Vec{double(c.x*8+9),double(c.y*8+9),double(c.z*8+9)}*grid.pitch;
 for(auto f:out.selected){const auto&ids=tree.source.f[f];
  const bool touch=triangleBoxOverlap(tree.source.v[ids[0]].p,tree.source.v[ids[1]].p,tree.source.v[ids[2]].p,lo,hi);
  if(f>=clean.size()||!clean[f]||touch)out.active.push_back(f);}
}
struct Region {std::vector<Cell> keys;std::vector<core::Tile> masks;std::vector<std::uint32_t> faces;std::uint64_t hash{};std::uint64_t cells{};};
// The regions of a grid (ascending first tile key), the grid's sorted tile keys
// with each tile's region, and the faces that are in no region: passive
// (selected by some tile, active in none) and unselected, both ascending.
struct RegionSet {
 std::vector<Region> regions;std::vector<Cell> keys;std::vector<int> tileRegion;
 core::FlatMap<Cell,int,core::CellHash> regionByTile; // tile key -> region index
 std::vector<std::uint32_t> passive,unselected;
 int regionOfTile(Cell t)const{const auto it=regionByTile.find(t);return it!=regionByTile.end()?it->second:-1;}
};
// findRegions of the wound path, on the sorted tile keys of `grid`. False when
// the asset has no incremental wound data (the caller runs the union path).
inline bool findRegions(const Asset&asset,const Grid&grid,unsigned maxThreads,RegionSet&out){
 if(!asset.fast||!asset.fast->valid||grid.ownership)return false;
 const auto&data=*asset.fast;const auto&tree=asset.source_tree;const auto&clean=data.clean;
 const std::size_t faces=asset.source.f.size();
 if(data.ringFirst.size()!=faces+1||clean.size()!=faces)return false;
 auto&keys=out.keys;keys.clear();keys.reserve(grid.tiles.size());for(const auto&t:grid.tiles)keys.push_back(t.first);
 std::sort(keys.begin(),keys.end());
 const int n=int(keys.size());
 core::FlatMap<Cell,int,core::CellHash> index;index.reserve(keys.size());
 for(int i=0;i<n;++i)index.emplace(keys[i],i);
 std::vector<int> parent(keys.size());for(int i=0;i<n;++i)parent[std::size_t(i)]=i;
 auto root=[&](int i){while(parent[std::size_t(i)]!=i){parent[std::size_t(i)]=parent[std::size_t(parent[std::size_t(i)])];i=parent[std::size_t(i)];}return i;};
 auto unite=[&](int a,int b){a=root(a);b=root(b);if(a!=b)parent[std::size_t(std::max(a,b))]=std::min(a,b);};
 // Adjacency is symmetric: the 13 lexicographically greater neighbours suffice.
 for(int i=0;i<n;++i)for(int dx=0;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)for(int dz=-1;dz<=1;++dz){
  if(dx==0&&(dy<0||(dy==0&&dz<=0)))continue;auto f=index.find(Cell{keys[std::size_t(i)].x+dx,keys[std::size_t(i)].y+dy,keys[std::size_t(i)].z+dz});if(f!=index.end())unite(i,f->second);}
 // Per-tile face lists: independent BVH queries, on up to maxThreads threads.
 std::vector<TileFaces> tf(keys.size());
 sep::parallel(keys.size(),64,maxThreads,[&](std::size_t first,std::size_t last,std::size_t){
  for(std::size_t i=first;i<last;++i)tileFaces(grid,tree,clean,keys[i],tf[i]);});
 std::vector<char> unionSelected(faces,0);
 for(const auto&t:tf)for(auto f:t.selected)unionSelected[f]=1;
 // Tile i's active faces: its own, plus every union-selected face sharing a
 // vertex position with them.
 std::vector<std::vector<std::uint32_t>> tileActive(keys.size());
 for(int i=0;i<n;++i){auto&list=tileActive[std::size_t(i)];const auto&own=tf[std::size_t(i)].active;list=own;
  for(auto f:own)for(auto r=data.ringFirst[f];r<data.ringFirst[f+1];++r)if(unionSelected[data.ringFaces[r]])list.push_back(data.ringFaces[r]);
  std::sort(list.begin(),list.end());list.erase(std::unique(list.begin(),list.end()),list.end());}
 // A face active for several tiles may be clipped or conformed by all of them: one region.
 std::vector<int> faceOwner(faces,-1);
 for(int i=0;i<n;++i)for(auto f:tileActive[std::size_t(i)]){if(faceOwner[f]<0)faceOwner[f]=i;else unite(i,faceOwner[f]);}
 std::map<int,std::vector<int>> members;for(int i=0;i<n;++i)members[root(i)].push_back(i);
 out.regions.clear();out.regions.reserve(members.size());out.tileRegion.assign(keys.size(),-1);
 std::vector<char> activeAny(faces,0);
 for(auto&m:members){Region region;std::uint64_t h=1469598103934665603ull;auto mix=[&](std::uint64_t v){h=core::quantMix(h^v);};
  for(int i:m.second){const Cell c=keys[std::size_t(i)];region.keys.push_back(c);const core::Tile&tile=grid.tiles.at(c);region.masks.push_back(tile);
   mix(std::uint64_t(std::uint32_t(c.x))|(std::uint64_t(std::uint32_t(c.y))<<32));mix(std::uint64_t(std::uint32_t(c.z)));
   for(auto word:tile.removed){mix(word);region.cells+=std::bitset<64>(word).count();}
   region.faces.insert(region.faces.end(),tileActive[std::size_t(i)].begin(),tileActive[std::size_t(i)].end());
   out.tileRegion[std::size_t(i)]=int(out.regions.size());}
  std::sort(region.faces.begin(),region.faces.end());region.faces.erase(std::unique(region.faces.begin(),region.faces.end()),region.faces.end());
  for(auto f:region.faces){mix(0x9E3779B97F4A7C15ull^f);activeAny[f]=1;}
  region.hash=h;out.regions.push_back(std::move(region));}
 out.regionByTile=core::FlatMap<Cell,int,core::CellHash>();out.regionByTile.reserve(keys.size());
 for(int i=0;i<n;++i)out.regionByTile.emplace(keys[std::size_t(i)],out.tileRegion[std::size_t(i)]);
 out.passive.clear();out.unselected.clear();
 for(std::uint32_t f=0;f<faces;++f){if(activeAny[f])continue;(unionSelected[f]?out.passive:out.unselected).push_back(f);}
 return true;
}
// A merged boundary rectangle's lattice description, derived from its quad
// exactly as fastwound::mergeBoundaryRects derives its sort entry (the same
// llround expressions on the corner positions). Used to locate the rectangle
// of a cavity triangle and as part of the cache key.
struct QuadMeta {int axis{},plane{},sign{};unsigned material{};int u0{},u1{},v0{},v1{};
 bool operator==(const QuadMeta&o)const{return axis==o.axis&&plane==o.plane&&sign==o.sign&&material==o.material&&u0==o.u0&&u1==o.u1&&v0==o.v0&&v1==o.v1;}};
inline QuadMeta quadMeta(const Grid&g,const core::Quad&q){
 Vec n=cross(q.p[1]-q.p[0],q.p[2]-q.p[0]);int axis=0;for(int a=1;a<3;++a)if(std::abs(n[a])>std::abs(n[axis]))axis=a;
 const int u=(axis+1)%3,v=(axis+2)%3;Vec lo=q.p[0],hi=q.p[0];for(auto p:q.p)for(int a=0;a<3;++a){lo[a]=std::min(lo[a],p[a]);hi[a]=std::max(hi[a],p[a]);}
 QuadMeta m;m.axis=axis;m.plane=int(std::llround((lo[axis]-g.origin[axis])/g.pitch));m.sign=n[axis]>0?1:-1;m.material=q.material;
 m.u0=int(std::llround((lo[u]-g.origin[u])/g.pitch));m.u1=int(std::llround((hi[u]-g.origin[u])/g.pitch));
 m.v0=int(std::llround((lo[v]-g.origin[v])/g.pitch));m.v1=int(std::llround((hi[v]-g.origin[v])/g.pitch));return m;
}
// One region's exact join (the cache value). Material-0 corners keep the
// patch-local sourceFace (an index into `faces`); the stage remaps on access.
// rectOf[t]: the region-local rectangle of cavity triangle t (~0u: exterior,
// or a cavity triangle the forward scan could not place).
struct RegionJoin {
 std::vector<Cell> keys;std::vector<core::Tile> masks;std::vector<std::uint32_t> faces;std::uint64_t hash{};
 std::vector<QuadMeta> metas;
 core::IndexedJoin indexed;std::vector<std::uint32_t> rectOf;std::size_t unlocated{};
 std::size_t bytes{};double exterior_ms{},join_ms{};
 bool same(const Region&r,const std::vector<QuadMeta>&m)const{
  if(hash!=r.hash||keys.size()!=r.keys.size()||faces!=r.faces||metas.size()!=m.size())return false;
  for(std::size_t i=0;i<keys.size();++i)if(!(keys[i]==r.keys[i])||masks[i].removed!=r.masks[i].removed)return false;
  for(std::size_t i=0;i<m.size();++i)if(!(metas[i]==m[i]))return false;
  return true;}
};
// Process-wide LRU of region joins, bounded by bytes (GFY_SEP_REGION_CACHE_MB,
// default 256; 0 disables). A value larger than the budget is never kept.
class RegionJoinCache {
 struct Entry {std::string key;std::shared_ptr<const RegionJoin> value;std::size_t bytes{};};
 std::mutex mutex_;std::vector<Entry> entries_; // LRU order: least recently used first
 std::size_t budget_,used_{};std::uint64_t hits_{},misses_{};
public:
 explicit RegionJoinCache(std::size_t budget):budget_(budget){}
 std::shared_ptr<const RegionJoin> find(const std::string&key){
  std::lock_guard<std::mutex> lock(mutex_);
  for(std::size_t i=0;i<entries_.size();++i)if(entries_[i].key==key){Entry e=std::move(entries_[i]);entries_.erase(entries_.begin()+std::ptrdiff_t(i));entries_.push_back(std::move(e));++hits_;return entries_.back().value;}
  ++misses_;return {};}
 void insert(const std::string&key,std::shared_ptr<const RegionJoin> value,std::size_t bytes){
  if(!value||bytes>budget_)return;std::lock_guard<std::mutex> lock(mutex_);
  for(std::size_t i=0;i<entries_.size();++i)if(entries_[i].key==key){used_-=entries_[i].bytes;entries_.erase(entries_.begin()+std::ptrdiff_t(i));break;}
  entries_.push_back({key,std::move(value),bytes});used_+=bytes;
  while(used_>budget_&&!entries_.empty()){used_-=entries_.front().bytes;entries_.erase(entries_.begin());}}
 void clear(){std::lock_guard<std::mutex> lock(mutex_);entries_.clear();used_=0;}
 std::size_t bytes(){std::lock_guard<std::mutex> lock(mutex_);return used_;}
 std::size_t budget()const{return budget_;}
 std::pair<std::uint64_t,std::uint64_t> counters(){std::lock_guard<std::mutex> lock(mutex_);return {hits_,misses_};}
};
inline std::size_t regionJoinCacheBudget(){
 static const std::size_t budget=[]{const char*e=std::getenv("GFY_SEP_REGION_CACHE_MB");long mb=e?std::atol(e):256;if(mb<0)mb=0;return std::size_t(mb)<<20;}();
 return budget;}
inline RegionJoinCache&regionJoinCache(){static RegionJoinCache cache(regionJoinCacheBudget());return cache;}
inline bool regionJoinEnabled(){static const bool on=[]{const char*e=std::getenv("GFY_SEP_REGION_JOIN");return !e||std::atoi(e)!=0;}();return on;}
inline std::string cacheKey(const Asset&asset,const Grid&grid,const Region&region,const std::vector<QuadMeta>&metas){
 sep::KeyBytes key;key.add(asset.source_hash).add(asset.rig_hash).add(grid.pitch).add(grid.origin.x).add(grid.origin.y).add(grid.origin.z)
  .add(grid.size.x).add(grid.size.y).add(grid.size.z).add(region.hash).add(std::uint64_t(region.keys.size())).add(std::uint64_t(region.faces.size()));
 std::uint64_t h=0x51ED270B4A9C3E17ull;for(const auto&m:metas){for(int v:{m.axis,m.plane,m.sign,int(m.material),m.u0,m.u1,m.v0,m.v1})h=core::quantMix(h^std::uint64_t(std::uint32_t(v)));}
 key.add(h).add(std::uint64_t(metas.size()));return key.str();
}
// The exact join of one region: the region's active faces as the patch, the
// region's merged rectangles (in the order given) as the cavity boundary.
inline std::shared_ptr<RegionJoin> computeRegionJoin(const Asset&asset,const Grid&grid,const Region&region,const std::vector<core::Quad>&quads,const std::vector<QuadMeta>&metas){
 auto result=std::make_shared<RegionJoin>();auto&out=*result;
 out.keys=region.keys;out.masks=region.masks;out.faces=region.faces;out.hash=region.hash;out.metas=metas;
 // The patch shares the asset's vertex array: one copy per thread and asset.
 thread_local Source patch;thread_local Hash patchSource{};thread_local std::size_t patchVertices=~std::size_t(0);
 if(patchSource!=asset.source_hash||patchVertices!=asset.source.v.size()){patch.v=asset.source.v;patch.bones=asset.source.bones;patchSource=asset.source_hash;patchVertices=asset.source.v.size();}
 patch.f.clear();patch.f.reserve(region.faces.size());
 for(auto f:region.faces)patch.f.push_back(asset.source.f[f]);
 auto t0=core::Clock::now();
 auto outside=core::exterior(patch,grid);out.exterior_ms=core::ms(t0);t0=core::Clock::now();
 out.indexed=core::joinIndexed(patch,asset.envelope_tree,grid,outside,quads);out.join_ms=core::ms(t0);
 outside=core::ClipResult{};
 // The former materialization checked every material-0 corner's patch face.
 const auto patchFaces=region.faces.size();
 for(const auto&t:out.indexed.triangles){if(t.material)continue;
  need(t.v[0]->sourceFace<patchFaces&&t.v[1]->sourceFace<patchFaces&&t.v[2]->sourceFace<patchFaces,"fullbody_patch_provenance");}
 // Cavity triangles follow their rectangles in emission order (quads are
 // clipped, conformed and fanned in order): one forward scan places each.
 out.rectOf.assign(out.indexed.triangles.size(),~0u);std::size_t cursor=0;
 for(std::size_t i=0;i<out.indexed.triangles.size();++i){const auto&t=out.indexed.triangles[i];if(!t.material)continue;
  const Vec p0=t.v[0]->p,p1=t.v[1]->p,p2=t.v[2]->p;
  const Vec n=cross(p1-p0,p2-p0);int axis=0;for(int a=1;a<3;++a)if(std::abs(n[a])>std::abs(n[axis]))axis=a;
  const int u=(axis+1)%3,v=(axis+2)%3;const Vec c=(p0+p1+p2)*(1./3);
  const int plane=int(std::llround((c[axis]-grid.origin[axis])/grid.pitch));
  const int a=int(std::floor((c[u]-grid.origin[u])/grid.pitch)),b=int(std::floor((c[v]-grid.origin[v])/grid.pitch));
  for(std::size_t j=cursor;j<metas.size();++j){const auto&m=metas[j];
   if(m.axis==axis&&m.plane==plane&&m.material==t.material&&a>=m.u0&&a<m.u1&&b>=m.v0&&b<m.v1){cursor=j;out.rectOf[i]=std::uint32_t(j);break;}}
  if(out.rectOf[i]==~0u)++out.unlocated;}
 out.bytes=(out.indexed.vertices.size()+out.indexed.extra.size())*sizeof(core::JoinedVertex)+out.indexed.triangles.size()*sizeof(core::IndexedTriangle)
  +out.rectOf.size()*4+out.keys.size()*(sizeof(Cell)+sizeof(core::Tile))+out.faces.size()*4+out.metas.size()*sizeof(QuadMeta)+sizeof(RegionJoin);
 return result;
}
struct RegionJoinSet {
 RegionSet set;
 std::vector<std::shared_ptr<const RegionJoin>> joins; // per region
 std::vector<std::vector<std::uint32_t>> globalRect;    // per region: whole-grid quad index of each region-local rectangle
 std::size_t hits{},misses{},unlocated{},cachedBytes{};double regions_ms{},split_ms{},join_ms{};
};
// Joins every region of `set` over the whole-grid merged boundary `boundary`
// (boundaryFast order): cache lookups first, then the missing regions largest
// first on up to maxThreads threads (the caller included). The largest missing
// region keeps the join's own internal fan-out (as the union join had); the
// others run their internals inline so the job's threads stay within budget.
// The first failing region in region order rethrows (std::bad_alloc first).
inline void joinRegions(const Asset&asset,const Grid&grid,const std::vector<core::Quad>&boundary,unsigned maxThreads,RegionJoinSet&out){
 const auto&set=out.set;const std::size_t R=set.regions.size();
 auto t0=core::Clock::now();
 std::vector<std::vector<core::Quad>> quads(R);std::vector<std::vector<QuadMeta>> metas(R);out.globalRect.assign(R,{});
 auto floorDiv=[](int x,int d){return x>=0?x/d:-((-x+d-1)/d);};
 Cell lastTile{INT_MIN,INT_MIN,INT_MIN};int lastRegion=-1; // consecutive quads mostly share a tile
 for(std::uint32_t i=0;i<boundary.size();++i){const auto&q=boundary[i];
  const Cell tile{floorDiv(q.removedCell.x,8),floorDiv(q.removedCell.y,8),floorDiv(q.removedCell.z,8)};
  if(!(tile==lastTile)){lastTile=tile;lastRegion=set.regionOfTile(tile);}
  const int r=lastRegion;need(r>=0,"region_join_quad_outside_regions");
  quads[std::size_t(r)].push_back(q);metas[std::size_t(r)].push_back(quadMeta(grid,q));out.globalRect[std::size_t(r)].push_back(i);}
 out.split_ms=core::ms(t0);t0=core::Clock::now();
 out.joins.assign(R,nullptr);std::vector<std::string> keys(R);std::vector<char> hit(R,0);
 auto&cache=regionJoinCache();
 for(std::size_t r=0;r<R;++r){keys[r]=cacheKey(asset,grid,set.regions[r],metas[r]);
  if(auto value=cache.find(keys[r]))if(value->same(set.regions[r],metas[r])){out.joins[r]=std::move(value);hit[r]=1;++out.hits;}}
 std::vector<std::size_t> todo;for(std::size_t r=0;r<R;++r)if(!out.joins[r])todo.push_back(r);
 std::sort(todo.begin(),todo.end(),[&](std::size_t a,std::size_t b){const auto&x=set.regions[a],&y=set.regions[b];
  return x.cells!=y.cells?x.cells>y.cells:a<b;});
 out.misses=todo.size();
 std::vector<std::exception_ptr> failure(R);std::atomic<std::size_t> cursor{0};
 sep::parallel(todo.size(),1,maxThreads,[&](std::size_t,std::size_t,std::size_t){
  for(std::size_t k;(k=cursor.fetch_add(1))<todo.size();){const auto r=todo[k];
   if(sep::cancelRequested())return;
   struct Inline{bool previous;explicit Inline(bool on):previous(core::painterInlineParallel){core::painterInlineParallel=on;}~Inline(){core::painterInlineParallel=previous;}};
   const Inline scope(k!=0&&todo.size()>1);
   try{out.joins[r]=computeRegionJoin(asset,grid,set.regions[r],quads[r],metas[r]);}catch(...){failure[r]=std::current_exception();}}});
 sep::throwIfCanceled();
 for(const auto&f:failure)if(f){try{std::rethrow_exception(f);}catch(const std::bad_alloc&){throw;}catch(...){}}
 for(const auto&f:failure)if(f)std::rethrow_exception(f);
 for(std::size_t r=0;r<R;++r){need(bool(out.joins[r]),"region_join_missing");out.unlocated+=out.joins[r]->unlocated;
  if(!hit[r])cache.insert(keys[r],out.joins[r],out.joins[r]->bytes);}
 out.cachedBytes=cache.bytes();out.join_ms=core::ms(t0);
}
}}}
#endif
