// body_hit_outcomes_v1
#include "LiveDamageWorker.h"
#include "MeleeImpact.h" // melee (value 3): knife / blade / blunt events, the body's dents
#include "third_party/meshoptimizer/meshoptimizer.h" // low-poly wound display (LowPolyDisplay.h)
#include "CompressedAsset.h"
#include "WoundEdgeProfile.h"
#include "FullTileErase.h"
#include "TileTunnelErase.h"
#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <climits>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <limits>
#include <queue>
#include <tuple>
#include <mutex>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <bitset>
#include "AnalyticChannelField.h"
#include "ClosureField.h"
#include "PolyTriangulate.h" // tile-local exterior (global namespace polytri)
#include "OwnerSafe.h" // ownersafe: test-only fault switches (none in the game build)
namespace gfy_live {
namespace core {
#define main isolated_original_probe_main
#include "core/native_voxel_probe.cpp"
#include "AnatomyFill.h"
#undef main
#include "RuntimeMeshPacking.h"
}
using core::Vec;using core::Cell;using core::Grid;using core::Source;
using core::operator+;using core::operator-;using core::operator*;
using core::dot;using core::cross;using core::length;
#include "FastWoundData.h"
namespace {
struct PhaseTime {
 double& total;core::Clock::time_point start=core::Clock::now();bool active=true;
 explicit PhaseTime(double&value):total(value){}
 void finish(){if(active){total+=core::ms(start);active=false;}}
 ~PhaseTime(){finish();}
};
thread_local char failure[256]{};
void fail(const char*why)noexcept{std::strncpy(failure,why,sizeof(failure)-1);failure[sizeof(failure)-1]=0;}
void need(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
// ---- Allocation failure policy and memory admission (see LiveDamageWorker.h) ----
// Join estimate coefficients (bytes); overridable only for calibration builds.
// Calibrated 2026-09-25 on the direct path with the exposed-face admission input (probe_oom, 86 jobs: in-game replays,
// heavy corpse, slugs, bursts): MEASURED peak transient 28-66 MB per job, dominated by a fixed part; 32 MB + 64 B per
// face over-estimates most jobs and under-estimates none by more than 49 MB (inside the 128 MB commit margin).
// The former 1 KB per 3 x popcount(removed) over-charged the same jobs 5-7x (424-772 MB in game).
#ifndef GFY_JOIN_BYTES_PER_QUAD
#define GFY_JOIN_BYTES_PER_QUAD 64ull
#endif
#ifndef GFY_JOIN_BYTES_PER_FACE
#define GFY_JOIN_BYTES_PER_FACE 0ull
#endif
#ifndef GFY_JOIN_BYTES_PER_REPLACED_TRIANGLE
#define GFY_JOIN_BYTES_PER_REPLACED_TRIANGLE 0ull
#endif
#ifndef GFY_JOIN_BYTES_PER_BODY_TRIANGLE
#define GFY_JOIN_BYTES_PER_BODY_TRIANGLE 0ull
#endif
#ifndef GFY_JOIN_BYTES_FIXED
#define GFY_JOIN_BYTES_FIXED (32ull<<20)
#endif
std::atomic<ThreadAllocationPolicy> allocationPolicy{nullptr};
std::atomic<MemoryQuery> memoryQueryHook{nullptr};
struct AtomicMemoryCounters {std::atomic<std::uint64_t> allocation_failures{0},oom_deferred{0},oom_failed{0},admission_deferred{0},admission_failed{0},
 heavy_joins{0},heavy_waits{0},phys_waits{0},dispatch_waits{0},separation_refused{0},separation_oom{0},piece_oom{0},preparation_oom{0},evicted_sessions{0},evicted_bytes{0},max_join_estimate{0};};
AtomicMemoryCounters memoryCounters;
std::mutex memoryPolicyMutex;MemoryPolicy memoryPolicyValue;
void counterMax(std::atomic<std::uint64_t>&counter,std::uint64_t value){auto seen=counter.load(std::memory_order_relaxed);while(seen<value&&!counter.compare_exchange_weak(seen,value,std::memory_order_relaxed)){}}
// Per-thread reserve for small requests. The runtime also allocates inside
// noexcept code (std::exception message copies, exception_ptr captures) with
// plain malloc, which reaches the same handler under new_mode 1: a throw there
// would be std::terminate. A small request that fails first releases this
// reserve and is retried once; only then is the failure a std::bad_alloc.
// Re-armed at every thread entry and job start (never throws: while arming,
// the hook reports a plain failure and the thread just has no reserve).
constexpr std::size_t smallReserveBytes=64*1024;
struct SmallReserve {void* block{};bool arming{};~SmallReserve(){std::free(block);}};
thread_local SmallReserve smallReserve;
// The hook the engine's out-of-memory handler calls on Painter threads instead
// of its fatal report. Allocates nothing; returns only to retry (see above).
int throwAllocationFailure(std::size_t size){
 auto&r=smallReserve;if(r.arming)return 0;
 if(size<=smallReserveBytes&&r.block){void*block=r.block;r.block=nullptr;std::free(block);return 1;}
 memoryCounters.allocation_failures.fetch_add(1,std::memory_order_relaxed);throw std::bad_alloc();
}
void armSmallReserve()noexcept{auto&r=smallReserve;if(r.block)return;r.arming=true;r.block=std::malloc(smallReserveBytes);r.arming=false;}
void painterThreadEntry()noexcept{if(auto policy=allocationPolicy.load(std::memory_order_acquire)){policy(&throwAllocationFailure);armSmallReserve();}}
struct AllocationScope {
 AllocationFailureHook previous{};ThreadAllocationPolicy policy{};
 AllocationScope()noexcept:policy(allocationPolicy.load(std::memory_order_acquire)){if(policy){previous=policy(&throwAllocationFailure);armSmallReserve();}}
 ~AllocationScope(){if(policy)policy(previous);}
 AllocationScope(const AllocationScope&)=delete;AllocationScope&operator=(const AllocationScope&)=delete;
};
// Short reasons (std::string SSO: assigning them never allocates).
constexpr const char* outOfMemoryText="out_of_memory";
constexpr const char* lowMemoryText="low_memory";
// Never-drop memory policy: a batch short of memory returns to the queue every time; a multi-event batch is
// isolated (one event per job) after isolateAfterDeferrals; a job is reported not applied only after
// maxBatchDeferrals (dispatch waits up to max_wait_ms between them: about a minute of starvation).
constexpr unsigned isolateAfterDeferrals=3,maxBatchDeferrals=16;
const char* allocationFailureText(){return outOfMemoryText;}
// Pre-join admission: not enough commit for this heavy join (see MemoryPolicy).
struct MemoryShortage {std::uint64_t need{};};
// A chunk of a split cluster failed: the cluster is joined whole on a second tile-local attempt.
struct ChunkRetry : std::runtime_error {ChunkRetry():std::runtime_error("chunk_retry"){}};
// The owner was canceled while this job waited for admission: publish nothing.
struct AdmissionCanceled {};
struct HeavyJoinGate {std::mutex mutex;std::condition_variable wake;bool busy{};};
HeavyJoinGate& heavyJoinGate(){static HeavyJoinGate gate;return gate;}
// Held by one job from its first heavy join until the job ends (also on unwind).
struct HeavyJoinHold {bool held{};HeavyJoinHold()=default;HeavyJoinHold(const HeavyJoinHold&)=delete;HeavyJoinHold&operator=(const HeavyJoinHold&)=delete;
 ~HeavyJoinHold(){if(!held)return;auto&g=heavyJoinGate();{std::lock_guard<std::mutex>lock(g.mutex);g.busy=false;}g.wake.notify_all();}};
Vec vec(Point p){return {p.x,p.y,p.z};}Point point(Vec p){return {p.x,p.y,p.z};}
Vec unit(Vec p){const double n=length(p);need(std::isfinite(n)&&n>1e-10,"invalid_ray_direction");return p*(1/n);}
Vec transform(const Matrix&m,Vec p,bool direction=false){auto&a=m.row_major;return {a[0]*p.x+a[1]*p.y+a[2]*p.z+(direction?0:a[3]),a[4]*p.x+a[5]*p.y+a[6]*p.z+(direction?0:a[7]),a[8]*p.x+a[9]*p.y+a[10]*p.z+(direction?0:a[11])};}
Vec inverseDirection(const Matrix&m,Vec d){auto&a=m.row_major;Vec c0{a[0],a[4],a[8]},c1{a[1],a[5],a[9]},c2{a[2],a[6],a[10]};double det=dot(c0,cross(c1,c2));need(std::isfinite(det)&&std::abs(det)>1e-6,"singular_skin_mapping");return {dot(d,cross(c1,c2))/det,dot(d,cross(c2,c0))/det,dot(d,cross(c0,c1))/det};}
struct Cursor {
 const std::vector<std::uint8_t>&bytes;std::size_t at{};
 template<class T>T take(){need(sizeof(T)<=bytes.size()-at,"truncated_asset");T v;std::memcpy(&v,bytes.data()+at,sizeof(v));at+=sizeof(v);return v;}
 void magic(const char*s){auto m=take<std::array<char,8>>();need(!std::memcmp(m.data(),s,8),"asset_magic");}
 void done(){need(at==bytes.size(),"asset_trailing_bytes");}
};
Source sourceBytes(const std::vector<std::uint8_t>&bytes,bool occlusion=false){Cursor c{bytes};const auto magic=c.take<std::array<char,8>>();const bool hardware=std::memcmp(magic.data(),"VXHEAD02",8)==0;need(hardware||std::memcmp(magic.data(),"VXHEAD01",8)==0,"asset_magic");auto nv=c.take<std::uint32_t>(),nf=c.take<std::uint32_t>(),nb=c.take<std::uint32_t>();need(((nv&&nf)||(occlusion&&!nv&&!nf))&&nv<=300000u&&nf<=600000u&&nb&&nb<=64,"asset_geometry_limits");Source s;s.bones=nb;s.v.resize(nv);s.f.resize(nf);
 for(auto&v:s.v){v.p={c.take<float>(),c.take<float>(),c.take<float>()};v.uv=c.take<std::array<float,2>>();v.ids=c.take<std::array<std::uint16_t,4>>();v.w=c.take<std::array<float,4>>();double sum=0;
  v.authored_hardware=hardware;if(hardware){need(gfy_signed_hw::valid4(v.w,true),"asset_authored_hardware_skin");for(auto b:v.ids)need(b<nb,"asset_skin_bone");need(std::isfinite(v.p.x)&&std::isfinite(v.p.y)&&std::isfinite(v.p.z),"asset_positions");continue;}
  std::array<double,64>combined{};
  for(unsigned k=0;k<4;++k){need(v.ids[k]<nb&&std::isfinite(v.w[k])&&v.w[k]>=-2e-7,"asset_skin");sum+=v.w[k];combined[v.ids[k]]+=v.w[k];}
  need(std::abs(sum-1)<1e-6,"asset_skin_sum");
  // Authored 4W vertices commonly repeat the first bone in the residual slot.
  // 1-(w0+w1+w2) can be -one float ULP. Combine identical influences first;
  // clamp only that bounded arithmetic residue, never an actual negative skin.
  double positive=0;for(auto&w:combined){need(w>=-2e-7,"asset_combined_skin");w=std::max(0.,w);positive+=w;}
  need(std::abs(positive-sum)<3e-7,"asset_skin_residue");v.ids={};v.w={};unsigned slot=0;
  for(unsigned b=0;b<nb;++b)if(combined[b]>0){need(slot<4,"asset_combined_influence_count");v.ids[slot]=std::uint16_t(b);v.w[slot]=float(combined[b]/positive);++slot;}
  need(std::isfinite(v.p.x)&&std::isfinite(v.p.y)&&std::isfinite(v.p.z),"asset_positions");}
 for(auto&f:s.f){f=c.take<std::array<std::uint32_t,3>>();for(auto id:f)need(id<nv,"asset_index");}c.done();return s;}
Grid gridBytes(const std::vector<std::uint8_t>&bytes){
 if(bytes.size()>=2&&bytes[0]==31&&bytes[1]==139)return gridBytes(gfy_asset_codec::gunzip(bytes));
 if(bytes.size()>=8&&!std::memcmp(bytes.data(),"ANATCOL1",8)){
  Grid g;g.anatomy=core::material_volume::decode(bytes);const auto&a=*g.anatomy;
  g.size={int(a.size[0]),int(a.size[1]),int(a.size[2])};g.origin={a.origin[0],a.origin[1],a.origin[2]};g.pitch=a.pitch;return g;
 }
 Cursor c{bytes};c.magic("VXBASE01");auto nx=c.take<std::uint32_t>(),ny=c.take<std::uint32_t>(),nz=c.take<std::uint32_t>();need(nx&&ny&&nz&&nx<=1024&&ny<=2048&&nz<=1024&&std::uint64_t(nx)*ny*nz<=134217728,"asset_grid_limit");Grid g;g.size={int(nx),int(ny),int(nz)};g.origin={c.take<double>(),c.take<double>(),c.take<double>()};g.pitch=c.take<double>();need(std::isfinite(g.pitch)&&g.pitch>=.0005&&g.pitch<=.01,"asset_grid_pitch");auto count=(std::size_t(nx)*ny*nz+7)/8;need(bytes.size()-c.at==count,"asset_grid_bytes");g.base=std::make_shared<const std::vector<std::uint8_t>>(bytes.begin()+c.at,bytes.end());return g;}
struct March {bool found{};Vec entry;Cell cell;};
March firstSolid(const Grid&g,Vec origin,Vec direction){direction=unit(direction);double enter=0,leave=1e6;
 for(int a=0;a<3;++a){double low=g.origin[a],high=low+g.size[a]*g.pitch;if(std::abs(direction[a])<1e-14){if(origin[a]<low||origin[a]>=high)return {};continue;}double t0=(low-origin[a])/direction[a],t1=(high-origin[a])/direction[a];if(t0>t1)std::swap(t0,t1);enter=std::max(enter,t0);leave=std::min(leave,t1);}
 if(enter>=leave)return {};double t=enter;Vec p=origin+direction*(t+g.pitch*1e-7);Cell c;std::array<int,3> step{};std::array<double,3> next{},delta{};
 for(int a=0;a<3;++a){(&c.x)[a]=std::clamp(int(std::floor((p[a]-g.origin[a])/g.pitch)),0,g.size[a]-1);step[a]=direction[a]>0?1:-1;if(std::abs(direction[a])<1e-14){next[a]=delta[a]=1e100;}else{double plane=g.origin[a]+(c[a]+(step[a]>0?1:0))*g.pitch;next[a]=(plane-origin[a])/direction[a];delta[a]=g.pitch/std::abs(direction[a]);}}
 const int maxsteps=g.size.x+g.size.y+g.size.z+6;
 for(int n=0;n<maxsteps&&g.valid(c)&&t<leave;++n){if(g.solidBase(c)&&!g.removed(c))return {true,origin+direction*t,c};double target=std::min({next[0],next[1],next[2]});if(target>=leave)return {};for(int a=0;a<3;++a)if(next[a]<=target+1e-11){(&c.x)[a]+=step[a];next[a]+=delta[a];}t=target;}
 return {};
}
// The removal mask can have two diagonally touching removed cells
// around a voxel edge. Their four cavity faces form a geometrically
// nonmanifold cross. Resolve only such edge contacts by rasterizing ONE
// additional cell that the same analytic cutter intersects. Include cells
// whose centers are outside sampled occupancy: the exact skin can cross them
// and cavityPolygons clips these same cells against the exact body envelope.
// No cell is restored and the radius/depth are unchanged. Other nonmanifold
// edges still fail at the joined-mesh gate.
class RecoverableImpactError final:public std::runtime_error {public:using std::runtime_error::runtime_error;};
std::uint64_t checkedCutterCells(Cell low,Cell high){
 std::uint64_t count=1;for(int a=0;a<3;++a){if(high[a]<low[a])return 0;count*=std::uint64_t(high[a]-low[a]+1);}
 // Match the maximum admitted grid. A legitimate high-density slug must not
 // fail merely because it visits more cells than the same cut at Medium.
 if(count>134217728)throw RecoverableImpactError("impact_cell_work_budget");
 return count;
}
template<class Intersects>
std::size_t resolveEdgeContacts(Grid&g,const std::vector<Cell>&fresh,Vec cutter,Intersects intersects,std::set<Cell>&touched,std::size_t* extraSolid=nullptr,bool freshIsBoundary=false){
 // Only the mask boundary can contain a diagonal edge contact. Avoid building
 // millions of redundant edge keys for the interior of a wide slug cut.
 std::vector<Cell>frontier;std::size_t extras=0;
 // Large inputs use up to four threads on contiguous index ranges; the mask is
 // read-only inside each parallel section and results merge in index order
 // (frontier) or as a set union (repairs), so outputs match the serial loop.
 auto grainFor=[](std::size_t n){return n>2048?std::max<std::size_t>(512,n/core::parallelWidth()):n+1;};
 if(freshIsBoundary)frontier=fresh;
 else{std::vector<std::vector<Cell>>parts(core::parallelWidth());
  core::parallelChunks(fresh.size(),grainFor(fresh.size()),[&](std::size_t first,std::size_t last,std::size_t chunk){
   core::TileCache removed(g);auto&out=parts[chunk];
   for(std::size_t i=first;i<last;++i){const Cell c=fresh[i];bool boundary=false;for(int a=0;a<3&&!boundary;++a)for(int sign:{-1,1}){
    Cell n=c;(&n.x)[a]+=sign;if(!removed.removed(n)){boundary=true;break;}}
    if(boundary)out.push_back(c);}});
  for(auto&part:parts)frontier.insert(frontier.end(),part.begin(),part.end());}
 while(!frontier.empty()){
  std::set<Cell>add;
  // Grids without piece ownership: the same edges from tile-word shifts (TileTunnelErase.h edgeContactRepairs).
  if(!g.ownership&&painter_detail::edgeWordsEnabled()){
   const auto found=painter_detail::edgeContactRepairs(g,frontier,cutter,intersects,[&](std::size_t n,std::size_t grain,auto&&body){core::parallelChunks(n,grain,body);});
   add.insert(found.begin(),found.end());}
  else{
  std::vector<std::set<Cell>>addParts(core::parallelWidth());
  // Each boundary cell visits its incident edges. Repeated visits are safe:
  // the mask is unchanged for this entire round and `add` deduplicates the
  // chosen repair cells. This avoids storing a large set of edge keys.
  // Every frontier cell c is removed and is one of the four cells around each
  // incident edge, so a checkerboard needs both of c's in-plane face
  // neighbours valid and not removed. That exact precondition skips the
  // four-cell evaluation for all other edges; the evaluation itself and its
  // tie order are unchanged. The mask is not mutated during a round.
  core::parallelChunks(frontier.size(),grainFor(frontier.size()),[&](std::size_t first,std::size_t last,std::size_t chunk){
  core::TileCache lookup(g);auto removedAt=[&](Cell q){return lookup.removed(q);};auto&add=addParts[chunk];
  for(std::size_t index=first;index<last;++index){const Cell c=frontier[index];
   // open[a][s]: face neighbour c+(s?+1:-1)e_a is valid and not removed.
   bool open[3][2];
   for(int a=0;a<3;++a)for(int s=0;s<2;++s){Cell n=c;(&n.x)[a]+=s?1:-1;open[a][s]=g.valid(n)&&!removedAt(n);}
   for(int axis=0;axis<3;++axis){int a=(axis+1)%3,b=(axis+2)%3;
   for(int da=0;da<=1;++da)for(int db=0;db<=1;++db){
   // c = q[(1-da)*2+(1-db)]; its in-plane neighbours are offset by +-1 along a and b.
   if(!open[a][da]||!open[b][db])continue;
   Cell e=c;(&e.x)[a]+=da;(&e.x)[b]+=db;Cell q[4];bool removed[4];bool eligible=true;
   for(int ia=0;ia<=1;++ia)for(int ib=0;ib<=1;++ib){int n=ia*2+ib;q[n]=e;(&q[n].x)[a]+=ia-1;(&q[n].x)[b]+=ib-1;if(!g.valid(q[n])){eligible=false;break;}removed[n]=removedAt(q[n]);}
   if(!eligible||removed[0]!=removed[3]||removed[1]!=removed[2]||removed[0]==removed[1])continue;
   int best=-1;double bestDistance=1e100;for(int n=0;n<4;++n)if(!removed[n]&&intersects(q[n])){Vec d=g.center(q[n])-cutter;double distance=dot(d,d);if(distance<bestDistance){bestDistance=distance;best=n;}}
   if(best>=0){add.insert(q[best]);if(add.size()>8192-extras)throw RecoverableImpactError("edge_contact_regularization_budget");}
  }}}});
  for(auto&part:addParts)add.insert(part.begin(),part.end());}
  if(add.size()>8192-extras)throw RecoverableImpactError("edge_contact_regularization_budget");
  frontier.clear();for(Cell c:add)if(!g.removed(c)){if(extras>=8192)throw RecoverableImpactError("edge_contact_regularization_budget");g.erase(c);if(extraSolid&&g.solidBase(c))++*extraSolid;touched.insert(Grid::tileKey(c));frontier.push_back(c);++extras;}
 }
 return extras;
}
std::size_t resolveVoxelEdgeContacts(Grid&g,const std::vector<Cell>&fresh,Vec cutter,double radius,std::set<Cell>&touched,std::size_t* extraSolid=nullptr){
 auto intersects=[&](Cell c){Vec low=g.origin+Vec{double(c.x),double(c.y),double(c.z)}*g.pitch;double d2=0;for(int a=0;a<3;++a){double v=std::max({low[a]-cutter[a],cutter[a]-(low[a]+g.pitch),0.});d2+=v*v;}return d2<radius*radius-1e-14;};
 return resolveEdgeContacts(g,fresh,cutter,intersects,touched,extraSolid);
}
core::HitResult eraseRay(Grid&g,Vec origin,Vec direction,double radius,double depth,std::size_t* edgeContactCells=nullptr){need(std::isfinite(radius)&&radius>=g.pitch&&radius<=.08&&std::isfinite(depth)&&depth>=0&&depth<=.08,"impact_size_limits");direction=unit(direction);auto march=firstSolid(g,origin,direction);core::HitResult result;if(!march.found)return result;result.cutter=march.entry+direction*depth;result.surface=length(march.entry-origin);Cell lo,hi;
 for(int a=0;a<3;++a){(&lo.x)[a]=std::max(0,int(std::floor((result.cutter[a]-radius-g.origin[a])/g.pitch)));(&hi.x)[a]=std::min(g.size[a]-1,int(std::floor((result.cutter[a]+radius-g.origin[a])/g.pitch)));}
 checkedCutterCells(lo,hi); // Before any occupancy mutation.
 std::set<Cell>touched;std::vector<Cell>fresh;for(int x=lo.x;x<=hi.x;++x)for(int y=lo.y;y<=hi.y;++y)for(int z=lo.z;z<=hi.z;++z){Cell c{x,y,z};Vec d=g.center(c)-result.cutter;if(dot(d,d)>radius*radius)continue;if(g.erase(c)){++result.freshMask;if(g.solidBase(c))++result.freshSolid;fresh.push_back(c);touched.insert(Grid::tileKey(c));}}
 std::size_t extraSolid=0;auto extras=resolveVoxelEdgeContacts(g,fresh,result.cutter,radius,touched,&extraSolid);if(edgeContactCells)*edgeContactCells=extras;
 result.freshMask+=extras;result.freshSolid+=extraSolid;
 for(auto c:touched)for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)for(int z=-1;z<=1;++z)result.dirty.insert({c.x+x,c.y+y,c.z+z});return result;
}
// A fixed-width entry bore followed by a rounded distal cap. The tip is
// exactly penetration metres from the first surviving material along the ray.
// If penetration < radius, a shallow half ellipsoid keeps the requested entry
// radius while shortening the cap. Voxel faces add only sampling-sized error.
struct Tunnel {
 Vec entry,direction;double radius,penetration,cap;double inlet=0;
 // Slab (R18 explosion blades): an oriented box, |n.q|<=slab_h and |s.q|<=slab_w across the channel axis, instead
 // of a round channel. No edge profile, cavitation, flare or cracks; classification is exact for a box.
 bool slab=false;Vec slab_n,slab_s;double slab_h=0,slab_w=0;
 std::uint64_t slab_seed=0;RaggedBlade ragged;
 double slabExtent()const{return ragged.active?ragged.extent():slab_h;}
 int raggedClassify(double t0,double t1,double n0,double n1,double s0,double s1)const{
  const double e=1e-9;
  if(t1< -inlet-e||t0>penetration+e||s1< -slab_w-e||s0>slab_w+e)return -1;
  const auto b=ragged.bounds(t0,t1,s0,s1);
  if(n1<b.lowerMin-e||n0>b.upperMax+e)return -1;
  return t0> -inlet+e&&t1<penetration-e&&s0> -slab_w+e&&s1<slab_w-e&&n0>b.lowerMax+e&&n1<b.upperMin-e?1:0;
 }
 // R60: exact SAT for each convex prism of the torn sheet the cell's footprint can meet: two lattice triangles per cell,
 // and per side cell the inlet block before the entry (its relief repeats the entry row). No enlarged repair witness.
 bool raggedIntersects(Vec lo,Vec hi)const{
  const Vec cc=(lo+hi)*.5,ch=(hi-lo)*.5;
  auto span=[&](const Vec&a){return std::abs(a.x)*ch.x+std::abs(a.y)*ch.y+std::abs(a.z)*ch.z;};
  const Vec q=cc-entry;const double tc=dot(q,direction),sc=dot(q,slab_s),te=span(direction),se=span(slab_s);
  if(tc+te< -inlet||tc-te>penetration||sc+se< -slab_w||sc-se>slab_w)return false;
  const Vec ca[3]={{1,0,0},{0,1,0},{0,0,1}};
  auto P=[&](double t,double s,double n){return entry+direction*t+slab_s*s+slab_n*n;};
  auto separated=[&](const Vec*v,unsigned nv,std::initializer_list<Vec> normals,std::initializer_list<Vec> edges){
   auto test=[&](const Vec&axis){double p0=1e300,p1=-1e300;for(unsigned k=0;k<nv;++k){const double p=dot(v[k]-cc,axis);p0=std::min(p0,p);p1=std::max(p1,p);}
    return p0>span(axis)+1e-12||p1< -span(axis)-1e-12;};
   for(const auto&a:ca)if(test(a))return true;
   for(const auto&a:normals)if(dot(a,a)>1e-24&&test(a))return true;
   for(const auto&c:ca)for(const auto&e:edges){const Vec axis=cross(c,e);if(dot(axis,axis)>1e-24&&test(axis))return true;}
   return false;};
  const double up=ragged.upperFace,down=ragged.lowerFace;
  int i0,j0,i1,j1;double u,v;ragged.cell(tc-te,sc-se,i0,j0,u,v);ragged.cell(tc+te,sc+se,i1,j1,u,v);
  for(int j=j0;j<=j1;++j){
   const double s0=-slab_w+j*ragged.ds,s1=s0+ragged.ds;
   if(inlet>0&&tc-te<0){const double a=ragged.node(0,j),b=ragged.node(0,j+1);
    const Vec w[8]={P(-inlet,s0,a-down),P(-inlet,s1,b-down),P(0,s0,a-down),P(0,s1,b-down),P(-inlet,s0,a+up),P(-inlet,s1,b+up),P(0,s0,a+up),P(0,s1,b+up)};
    const Vec es=slab_s*(s1-s0)+slab_n*(b-a);
    if(!separated(w,8,{cross(direction,es),direction,slab_s},{direction,es,slab_n}))return true;}
   if(tc+te<0)continue;
   for(int i=i0;i<=i1;++i){const double t0=i*ragged.dt,t1=t0+ragged.dt;
    const double d00=ragged.node(i,j),d10=ragged.node(i+1,j),d01=ragged.node(i,j+1),d11=ragged.node(i+1,j+1);
    struct Node{double t,s,d;};Node tri[2][3];
    if(RaggedBlade::evenSplit(i,j)){tri[0][0]={t0,s0,d00};tri[0][1]={t1,s0,d10};tri[0][2]={t1,s1,d11};tri[1][0]={t0,s0,d00};tri[1][1]={t1,s1,d11};tri[1][2]={t0,s1,d01};}
    else{tri[0][0]={t0,s0,d00};tri[0][1]={t1,s0,d10};tri[0][2]={t0,s1,d01};tri[1][0]={t1,s1,d11};tri[1][1]={t0,s1,d01};tri[1][2]={t1,s0,d10};}
    for(const auto&T:tri){Vec w[6];for(unsigned k=0;k<3;++k){w[k]=P(T[k].t,T[k].s,T[k].d-down);w[k+3]=P(T[k].t,T[k].s,T[k].d+up);}
     const Vec e0=w[1]-w[0],e1=w[2]-w[1],e2=w[0]-w[2];
     if(!separated(w,6,{cross(e0,w[2]-w[0]),cross(e0,slab_n),cross(e1,slab_n),cross(e2,slab_n)},{e0,e1,e2,slab_n}))return true;}
   }
  }
  return false;
 }
 int slabInterval(double lo,double hi,double rlo,double rhi)const {const double e=1e-9;if(hi<rlo-e||lo>rhi+e)return -1;return lo>=rlo+e&&hi<=rhi-e?1:0;}
 double exit_distance=-1,exit_mult=1;
 WoundEdgeProfile edge;Vec edge_u,edge_v;
 // Cavitation: the channel swells to radius*(1+cav_amp) at the middle of [cav_t0,cav_t1] (sin^2 profile), a
 // stretch that ends before the exit flare or the tip taper begins, so radiusAt is: swell, constant, then flare.
 double cav_amp=0,cav_t0=0,cav_t1=0;
 // Burst (far-side exit of a head hit): the exit flare grows to exit_mult*(1+burst_amp) and `cracks` narrow radial
 // cracks (seeded angles, ~6 deg wide) reach a further burst_amp*radius over the last burst_len of the channel.
 double burst_amp=0,burst_len=0;int cracks=0;double crack_angle[8]{},crack_width=.10;
 double crack_u[8]{},crack_v[8]{},crack_center[8]{},crack_cos2=1;
 void setEdgeProfile(std::uint64_t seed,double variation=1.){
  edge=WoundEdgeProfile(seed,variation);if(!edge.enabled())return;
  const Vec axis=std::abs(direction.x)<.8?Vec{1,0,0}:Vec{0,1,0};
  edge_u=unit(cross(direction,axis));edge_v=cross(direction,edge_u);
 }
 // Called after exit_distance is known. cavitation/burst in [0,2]; seed picks the crack angles.
 void setForensics(double cavitation,double burst,std::uint64_t seed){
  auto h=[&](unsigned k){std::uint64_t x=seed*0x9E3779B97F4A7C15ull+k*0xC2B2AE3D27D4EB4Full;x^=x>>29;x*=0xBF58476D1CE4E5B9ull;x^=x>>32;return double(x>>11)*(1./9007199254740992.);};
  const double coreEnd=exit_distance<0?penetration-cap:exit_distance-std::min(exit_distance,radius*1.5);
  if(cavitation>0&&coreEnd>radius*4){
   // Where the cavity sits and how long it runs vary per shot: it starts 1.5-3.5 bores (at most 15-40 % of the channel)
   // behind the entry and runs 6-12 bores, capped at 10-26 cm or 60-90 % of the channel.
   cav_amp=std::min(2.,cavitation);cav_t0=std::min(radius*(1.5+2*h(20)),coreEnd*(.15+.25*h(21)));
   cav_t1=std::min(coreEnd,cav_t0+std::max(radius*(6+6*h(22)),std::min(.10+.16*h(23),coreEnd*(.6+.3*h(23)))));
   if(cav_t1-cav_t0<radius*2){cav_amp=0;}}
  if(burst>0&&exit_distance>0){
   burst_amp=std::min(1.5,burst*1.2);burst_len=std::min(exit_distance,std::max(radius*3,.025));
   if(!edge.enabled()){const Vec axis=std::abs(direction.x)<.8?Vec{1,0,0}:Vec{0,1,0};edge_u=unit(cross(direction,axis));edge_v=cross(direction,edge_u);}
   cracks=4+int(h(1)*4);const double base=6.283185307179586*h(2);
   for(int c=0;c<cracks;++c)crack_angle[c]=base+6.283185307179586*(c+.5*(h(10+unsigned(c))-.5))/cracks;
   crack_width=.08+.06*h(3);
   const double cosine=std::cos(crack_width+1e-8);crack_cos2=cosine*cosine;
   for(int c=0;c<cracks;++c){crack_u[c]=std::cos(crack_angle[c]);crack_v[c]=std::sin(crack_angle[c]);crack_center[c]=std::remainder(crack_angle[c],6.283185307179586);}}
 }
 double swell(double t)const {if(cav_amp<=0||t<=cav_t0||t>=cav_t1)return 0;const double s=std::sin(3.141592653589793*(t-cav_t0)/(cav_t1-cav_t0));return cav_amp*s*s;}
 double flareAt(double t)const {
  if(exit_distance<0)return 0;
  const double flare=std::min(exit_distance,radius*1.5);
  const double blend=flare>1e-12?std::clamp((t-(exit_distance-flare))/flare,0.,1.):1.;
  return (exit_mult*(1+burst_amp)-1)*blend;
 }
 double radiusAt(double t)const {if(slab)return maxRadius();return radius*(1+flareAt(t)+swell(t));}
 // Bounds of radiusAt over [a,b]: the swell is unimodal on [cav_t0,cav_t1], radiusAt is constant between the swell
 // and the flare and nondecreasing in the flare, so the extremes lie at the ends, the swell peak or the swell edges.
 double radiusMaxOver(double a,double b)const {double m=std::max(radiusAt(a),radiusAt(b));const double p=.5*(cav_t0+cav_t1);if(cav_amp>0&&p>a&&p<b)m=std::max(m,radiusAt(p));return m;}
 double radiusMinOver(double a,double b)const {double m=std::min(radiusAt(a),radiusAt(b));if(cav_amp>0){if(cav_t1>a&&cav_t1<b)m=std::min(m,radiusAt(cav_t1));if(cav_t0>a&&cav_t0<b)m=std::min(m,radiusAt(cav_t0));}return m;}
 double maxRadius()const {if(slab)return std::sqrt(slabExtent()*slabExtent()+slab_w*slab_w);return radius*std::max(exit_mult*(1+burst_amp)*(1+burst_amp),1+cav_amp);}
 // Crack multiplier (>= 1) at angle (eu,ev) and depth t: narrow radial spikes over the last burst_len.
 double crackScale(double eu,double ev,double t)const {
  if(cracks<=0||t<exit_distance-burst_len)return 1;
  // Reject directions outside every crack cone before the expensive exact
  // angular evaluation. The widened cone sends numerical boundary cases to
  // the original calculation, preserving its result.
  const double len2=eu*eu+ev*ev;bool possible=!(len2>1e-24);
  for(int c=0;c<cracks&&!possible;++c){const double d=eu*crack_u[c]+ev*crack_v[c];possible=d>0&&d*d>=len2*crack_cos2;}
  if(!possible)return 1;
  const double ramp=std::clamp((t-(exit_distance-burst_len))/burst_len,0.,1.);const double th=std::atan2(ev,eu);double best=0;
  for(int c=0;c<cracks;++c){double d=std::fmod(std::abs(th-crack_angle[c]),6.283185307179586);if(d>3.141592653589793)d=6.283185307179586-d;
   if(d<crack_width)best=std::max(best,1-d/crack_width);}
  return 1+burst_amp*ramp*ramp*best;
 }
 // Upper bound for the crack multiplier across a straight segment. The
 // projected segment sweeps the minor angular arc between its endpoints;
 // widen the approximate angles, then bound every crack over that arc.
 // Ambiguous/near-axis segments retain the former full-amplitude bound.
 double crackBound(Vec q0,Vec q1,double thi,double tlo,double& lower)const {
  lower=1;
  if(cracks<=0||thi<exit_distance-burst_len)return 1;
  const double ramp=std::clamp((thi-(exit_distance-burst_len))/burst_len,0.,1.);
  const double fallback=1+burst_amp*ramp*ramp;
  const double x0=dot(q0,edge_u),y0=dot(q0,edge_v),x1=dot(q1,edge_u),y1=dot(q1,edge_v);
  if(std::max(std::abs(x0),std::abs(y0))<1e-10||std::max(std::abs(x1),std::abs(y1))<1e-10)return fallback;
  constexpr double tau=6.283185307179586;
  const double a0=WoundEdgeProfile::approximateAngle(x0,y0);
  double delta=WoundEdgeProfile::approximateAngle(x1,y1)-a0;
  if(delta>tau*.5)delta-=tau;if(delta< -tau*.5)delta+=tau;
  if(std::abs(delta)>tau*.25)return fallback;
  const double lo=std::min(0.,delta)-WoundEdgeProfile::angle_error;
  const double hi=std::max(0.,delta)+WoundEdgeProfile::angle_error;
  double best=0,least=0;
  for(int c=0;c<cracks;++c){double angle=crack_center[c]-a0;
   if(angle>tau*.5)angle-=tau;if(angle< -tau*.5)angle+=tau;
   const double d=std::max({lo-angle,angle-hi,0.});
   if(d<crack_width)best=std::max(best,1-d/crack_width);
   const double far=std::max(std::abs(lo-angle),std::abs(hi-angle));
   if(far<crack_width)least=std::max(least,1-far/crack_width);}
  const double lowRamp=std::clamp((tlo-(exit_distance-burst_len))/burst_len,0.,1.);
  lower=1+burst_amp*lowRamp*lowRamp*least;
  return 1+burst_amp*ramp*ramp*best;
 }
 bool contains(Vec p)const {
  Vec q=p-entry;double t=dot(q,direction);
  if(t< -inlet||t>penetration)return false;
  if(slab)return ragged.active?ragged.contains(dot(q,slab_n),dot(q,slab_s),t):std::abs(dot(q,slab_n))<=slab_h&&std::abs(dot(q,slab_s))<=slab_w;
  double radial2=std::max(0.,dot(q,q)-t*t);
  double z=exit_distance<0?std::max(0.,t-(penetration-cap))/cap:0;
  double r=radiusAt(t);
  double limit=r*r*std::max(0.,1-z*z);
  if(cracks>0&&t>=exit_distance-burst_len){
   const double ramp=std::clamp((t-(exit_distance-burst_len))/burst_len,0.,1.);
   const double bound=1+burst_amp*ramp*ramp;
   if(radial2>limit*bound*bound*(1+1e-12))return false;
   const double guaranteed=edge.enabled()?WoundEdgeProfile::minimum_radius*WoundEdgeProfile::minimum_radius:1.;
   if(radial2<limit*guaranteed*(1-1e-12))return true;
   const double cs=crackScale(dot(q,edge_u),dot(q,edge_v),t);limit*=cs*cs;}
  if(radial2>limit)return false;
  if(!edge.enabled()||radial2<=limit*WoundEdgeProfile::minimum_radius*WoundEdgeProfile::minimum_radius)return true;
  const double eu=dot(q,edge_u),ev=dot(q,edge_v);
  // Exact shortcut: decide from per-sector multiplier bounds unless radial2
  // lies within a 1e-9 relative band of them; otherwise evaluate exactly.
  double low,high;
  if(edge.sectorBounds(eu,ev,low,high)){
   if(radial2<limit*low*low*(1-1e-9))return true;
   if(radial2>limit*high*high*(1+1e-9))return false;
  }
  const double scale=edge.radiusMultiplier(eu,ev,std::clamp(t/penetration,0.,1.));
  return radial2<=limit*scale*scale;
 }
 // Conservative classification of the n points p0+s*step (s=0..n-1):
 // 1 when contains() is certainly true for all, -1 when certainly false for
 // all, 0 when undecided. Margins (1e-9 relative, 1e-9 m in t, and a
 // |q|^2-scaled term) dominate the rounding of the per-point evaluation. The
 // squared axis distance is convex along the segment (max at an end, min at
 // the analytic vertex); t is linear; radiusAt is nondecreasing in t; the
 // edge multiplier is bounded per angular sector over both depth tables.
 int classifySegment(Vec p0,Vec step,int n)const {
  const Vec q0=p0-entry;const double t0=dot(q0,direction),dt=dot(step,direction),t1=t0+dt*(n-1);
  const double tlo=std::min(t0,t1),thi=std::max(t0,t1),tEps=1e-9;
  if(thi< -inlet-tEps||tlo>penetration+tEps)return -1;
  if(slab){const int r=slabInterval(tlo,thi,-inlet,penetration);if(r<0)return -1;
   const double n0=dot(q0,slab_n),n1=n0+dot(step,slab_n)*(n-1),s0=dot(q0,slab_s),s1=s0+dot(step,slab_s)*(n-1);
   if(ragged.active)return raggedClassify(tlo,thi,std::min(n0,n1),std::max(n0,n1),std::min(s0,s1),std::max(s0,s1));
   const int rn=slabInterval(std::min(n0,n1),std::max(n0,n1),-slab_h,slab_h),rs=slabInterval(std::min(s0,s1),std::max(s0,s1),-slab_w,slab_w);
   if(rn<0||rs<0)return -1;return r>0&&rn>0&&rs>0?1:0;}
  const double qq=dot(q0,q0),c1=2*(dot(q0,step)-t0*dt),c2=dot(step,step)-dt*dt;
  const double c0=qq-t0*t0,last=double(n-1);
  auto radial=[&](double u){return c0+u*(c1+u*c2);};
  const double rA=radial(0),rB=radial(last);double rMax=std::max(rA,rB),rMin=std::min(rA,rB);
  if(c2>0){const double u=-c1/(2*c2);if(u>0&&u<last)rMin=std::min(rMin,radial(u));}
  const double rEps=1e-13*(qq+last*last*dot(step,step)+1e-6);
  double crackLow=1;double rHigh=radiusMaxOver(tlo,thi);if(cracks>0&&thi>=exit_distance-burst_len)rHigh*=crackBound(q0,q0+step*last,thi,tlo,crackLow);const double limitMax=rHigh*rHigh;
  if(rMin>limitMax*(1+1e-9)+rEps)return -1;
  const double coreEnd=exit_distance<0?penetration-cap:penetration;
  const bool core=tlo> -inlet+tEps&&thi<coreEnd-tEps;
  const double rLow=radiusMinOver(tlo,thi)*crackLow,limitMin=rLow*rLow;
  if(!edge.enabled())return core&&rMax<limitMin*(1-1e-9)-rEps?1:0;
  const double m2=WoundEdgeProfile::minimum_radius*WoundEdgeProfile::minimum_radius;
  if(core&&rMax<limitMin*m2*(1-1e-9)-rEps)return 1;
  // Angular sweep of the axis-normal offset (a straight 2-D segment).
  if(!(rMin>limitMin*m2*.25))return 0;
  const Vec q1=q0+step*last;double low,high;
  // Depth fraction clamp(t/penetration) over the segment, widened by 1e-9.
  const double d0=std::clamp(tlo/penetration-1e-9,0.,1.),d1=std::clamp(thi/penetration+1e-9,0.,1.);
  if(!edge.arcDepthBounds(dot(q0,edge_u),dot(q0,edge_v),dot(q1,edge_u),dot(q1,edge_v),d0,d1,low,high))return 0;
  if(rMin>limitMax*high*high*(1+1e-9)+rEps)return -1;
  if(core&&rMax<limitMin*low*low*(1-1e-9)-rEps)return 1;
  return 0;
 }
 // Conservative certificate for every point in an axis-aligned box.
 // The containing sphere bounds radial distance and angular sweep; ambiguous
 // boundary boxes retain the exact segment/point path.
 int classifyBox(Vec center,Vec half)const {
  const Vec q=center-entry;const double tc=dot(q,direction);
  const double te=std::abs(direction.x)*half.x+std::abs(direction.y)*half.y+std::abs(direction.z)*half.z+1e-9;
  const double tlo=tc-te,thi=tc+te;
  if(thi< -inlet||tlo>penetration)return -1;
  if(slab){auto span=[&](const Vec&a){return std::abs(a.x)*half.x+std::abs(a.y)*half.y+std::abs(a.z)*half.z+1e-9;};
   const double nc=dot(q,slab_n),ne=span(slab_n),sc=dot(q,slab_s),se=span(slab_s);
   if(ragged.active)return raggedClassify(tlo,thi,nc-ne,nc+ne,sc-se,sc+se);
   const int r=slabInterval(tlo,thi,-inlet,penetration),rn=slabInterval(nc-ne,nc+ne,-slab_h,slab_h),rs=slabInterval(sc-se,sc+se,-slab_w,slab_w);
   if(r<0||rn<0||rs<0)return -1;return r>0&&rn>0&&rs>0?1:0;}
  const double e=length(half)+1e-9;
  const Vec radial=q-direction*tc;const double rc=length(radial);
  const double near=std::max(0.,rc-e),far=rc+e;
  const double coreEnd=exit_distance<0?penetration-cap:penetration;
  const bool core=tlo> -inlet&&thi<coreEnd;
  double rlo=radiusMinOver(tlo,thi),rhi=radiusMaxOver(tlo,thi);
  const double m=edge.enabled()?WoundEdgeProfile::minimum_radius:1.;
  if(core&&far<rlo*m*(1-1e-9))return 1;
  double cl=1,ch=1;
  const bool angular=rc>e*1.5;
  Vec qa,qb;
  if(angular){
   const Vec tangent=cross(direction,radial)*(e/std::sqrt(rc*rc-e*e));
   qa=radial-tangent;qb=radial+tangent;
  }
  if(cracks>0&&thi>=exit_distance-burst_len){
   if(angular)ch=crackBound(qa,qb,thi,tlo,cl);
   else{const double ramp=std::clamp((thi-(exit_distance-burst_len))/burst_len,0.,1.);ch=1+burst_amp*ramp*ramp;}
  }
  rlo*=cl;rhi*=ch;
  if(near>rhi*(1+1e-9))return -1;
  if(core&&far<rlo*m*(1-1e-9))return 1;
  if(edge.enabled()&&angular){double el,eh;
   if(edge.arcDepthBounds(dot(qa,edge_u),dot(qa,edge_v),dot(qb,edge_u),dot(qb,edge_v),
     std::clamp(tlo/penetration-1e-9,0.,1.),std::clamp(thi/penetration+1e-9,0.,1.),el,eh)){
    if(near>rhi*eh*(1+1e-9))return -1;
    if(core&&far<rlo*el*(1-1e-9))return 1;
   }
  }
  return 0;
 }
 // Only return true with an actual point in both the closed cell box and
 // analytic cut. A missed witness fails closed at the existing manifold gate.
 // This never expands the radius or penetration to repair ambiguous cells.
 bool intersectsCell(const Grid&g,Cell c)const {
  Vec lo=g.origin+Vec{double(c.x),double(c.y),double(c.z)}*g.pitch;
  Vec hi=lo+Vec{g.pitch,g.pitch,g.pitch};
  if(slab&&ragged.active)return raggedIntersects(lo,hi);
  if(slab){ // exact box/box test (separating axes): the cell's axes, the slab's axes and their cross products
   const Vec cc=(lo+hi)*.5,ch=(hi-lo)*.5,bc=entry+direction*((penetration-inlet)*.5),d=bc-cc;
   const double bh[3]={(penetration+inlet)*.5,slab_h,slab_w};const Vec ba[3]={direction,slab_n,slab_s};
   const Vec cellAxes[3]={Vec{1,0,0},Vec{0,1,0},Vec{0,0,1}};
   std::vector<Vec> axes(std::begin(cellAxes),std::end(cellAxes));axes.insert(axes.end(),std::begin(ba),std::end(ba));
   for(const Vec& a:cellAxes)for(int k=0;k<3;++k){const Vec x=cross(a,ba[k]);if(dot(x,x)>1e-18)axes.push_back(x);}
   for(const Vec& a:axes){const double pc=std::abs(a.x)*ch.x+std::abs(a.y)*ch.y+std::abs(a.z)*ch.z;
    double pb=0;for(int k=0;k<3;++k)pb+=std::abs(dot(a,ba[k]))*bh[k];
    if(std::abs(dot(d,a))>pc+pb+1e-12)return false;}
   return true;}
  auto clampPoint=[&](Vec p){return Vec{std::clamp(p.x,lo.x,hi.x),std::clamp(p.y,lo.y,hi.y),std::clamp(p.z,lo.z,hi.z)};};
  if(contains(g.center(c)))return true;
  for(int x=0;x<=1;++x)for(int y=0;y<=1;++y)for(int z=0;z<=1;++z)if(contains({x?hi.x:lo.x,y?hi.y:lo.y,z?hi.z:lo.z}))return true;
  std::vector<double> t{-inlet,0.,penetration-cap,penetration};
  for(int a=0;a<3;++a)if(std::abs(direction[a])>1e-14)for(double p:{lo[a],hi[a]}){double u=(p-entry[a])/direction[a];if(u> -inlet&&u<penetration)t.push_back(u);}
  std::sort(t.begin(),t.end());
  for(double u:t)if(contains(clampPoint(entry+direction*u)))return true;
  for(std::size_t k=1;k<t.size();++k){double mid=(t[k-1]+t[k])*.5,a=0,b=0;
   for(int axis=0;axis<3;++axis){double p=entry[axis]+direction[axis]*mid;if(p<lo[axis]||p>hi[axis]){double bound=p<lo[axis]?lo[axis]:hi[axis];a+=direction[axis]*direction[axis];b+=direction[axis]*(entry[axis]-bound);}}
   double u=a>0?std::clamp(-b/a,t[k-1],t[k]):mid;
   if(contains(clampPoint(entry+direction*u))||contains(clampPoint(entry+direction*mid)))return true;
  }
  return false;
 }
};
// A wide round whose centre line grazes past the tissue (a gas mask or hood 4 cm in front of the face, an oblique hit on a
// limb edge) still cuts what its radius overlaps: when the axis misses, rings of parallel rays at 0.35/0.7/1.0 radius
// (12 angles) find the first tissue; the entry is that contact projected on the axis (it may lie in air; the tunnel's inlet
// covers it). Axis hits are returned unchanged.
March firstSolidCylinder(const Grid&g,Vec origin,Vec direction,double radius){
 direction=unit(direction);auto axis=firstSolid(g,origin,direction);if(axis.found||radius<=0)return axis;
 const Vec ref=std::abs(direction.x)<.8?Vec{1,0,0}:Vec{0,1,0};const Vec u=unit(cross(direction,ref)),v=cross(direction,u);
 double best=std::numeric_limits<double>::infinity();March out;
 for(double f:{.35,.7,1.})for(int k=0;k<12;++k){const double a=6.283185307179586*k/12;const Vec off=(u*std::cos(a)+v*std::sin(a))*(radius*f);
  auto m=firstSolid(g,origin+off,direction);if(!m.found)continue;const double t=dot(m.entry-(origin+off),direction);if(t<best){best=t;out=m;}}
 if(!std::isfinite(best))return axis;
 out.entry=origin+direction*best;return out;
}
// nothinghits (2026-11-16): the first uncut anatomy cell a shot's cut itself holds. 65 rays across the cut's disc (the axis and
// rings of 16 at .25 / .5 / .75 / .95 of the radius), each followed through uncut anatomy (firstSolid) until a cell whose centre
// lies ahead of the origin and inside the cut started at that ray's contact (Tunnel::contains with the same seeded ragged edge;
// the cut's swell, exit flare and cracks only widen it). The nearest such contact within `limit` along the shot. Not found: {}.
March heldContact(const Grid&g,Vec origin,Vec direction,double radius,double penetration,double limit,std::uint64_t seed,double variation){
 direction=unit(direction);const Vec ref=std::abs(direction.x)<.8?Vec{1,0,0}:Vec{0,1,0};const Vec u=unit(cross(direction,ref)),v=cross(direction,u);
 Tunnel held{origin,direction,radius,penetration,std::min(radius,penetration)};held.setEdgeProfile(seed,variation);
 double best=std::numeric_limits<double>::infinity();March out;
 for(double f:{0.,.25,.5,.75,.95})for(int k=0;k<(f>0?16:1);++k){const double a=6.283185307179586*k/16;const Vec o=origin+(u*std::cos(a)+v*std::sin(a))*(radius*f);
  double t=0;
  for(int n=0;n<2048;++n){const auto m=firstSolid(g,o+direction*t,direction);if(!m.found)break;
   const double tm=dot(m.entry-o,direction);if(tm>=best||tm>limit)break;
   const Vec c=g.center(m.cell);held.entry=origin+direction*tm;held.inlet=tm;
   if(dot(c-origin,direction)>=0&&held.contains(c)){best=tm;out=m;break;}
   t=tm+g.pitch*.5;}}
 if(!std::isfinite(best))return {};
 out.entry=origin+direction*best;return out;
}
// Called only after a successful mutating erasure, while the event journal
// is active. Allocation failure follows that event's normal rollback route.
ResolvedTunnel resolvedTunnel(const Tunnel&cut,std::uint64_t seed,std::size_t repairs){
 ResolvedTunnel out;out.entry={cut.entry.x,cut.entry.y,cut.entry.z};out.direction={cut.direction.x,cut.direction.y,cut.direction.z};
 out.edge_u={cut.edge_u.x,cut.edge_u.y,cut.edge_u.z};out.edge_v={cut.edge_v.x,cut.edge_v.y,cut.edge_v.z};
 out.radius=cut.radius;out.penetration=cut.penetration;out.cap=cut.cap;out.inlet=cut.inlet;
 out.exit_distance=cut.exit_distance;out.exit_mult=cut.exit_mult;out.edge_seed=seed;
 out.cav_amp=cut.cav_amp;out.cav_t0=cut.cav_t0;out.cav_t1=cut.cav_t1;out.burst_amp=cut.burst_amp;out.burst_len=cut.burst_len;
 out.crack_width=cut.crack_width;out.cracks=unsigned(cut.cracks);out.edge_contact_cells=repairs;
 if(cut.slab){ // R24: the box's own frame and sizes; a flat far end at the reach
  out.slab=true;out.slab_seed=cut.slab_seed;out.slab_half_width=cut.slab_w;out.radius=cut.slab_h;out.cap=std::min(cut.slab_h,cut.penetration);
  out.edge_u={cut.slab_n.x,cut.slab_n.y,cut.slab_n.z};out.edge_v={cut.slab_s.x,cut.slab_s.y,cut.slab_s.z};
  out.exit_distance=cut.penetration;out.exit_mult=1;out.cav_amp=out.cav_t0=out.cav_t1=out.burst_amp=out.burst_len=0;out.cracks=0;}
 std::copy(std::begin(cut.crack_angle),std::end(cut.crack_angle),out.crack_angle.begin());
 if(cut.edge.enabled()){auto data=std::make_shared<ResolvedEdgeProfile>();data->radius=cut.edge.radiusTables();out.edge=std::move(data);}
 return out;
}
struct TunnelExit {bool reached{};Vec point;};
struct SlabSpec {Vec normal,side;double half_thickness,half_width;std::uint64_t seed{};}; // R18 explosion blades (rest space)
struct TunnelColumn {Cell first;int end;};
// Traverse a conservative cylinder one column at a time. An oblique High
// quality cut must not pay for its mostly empty axis-aligned bounding cube.
// Exact membership remains Tunnel::contains; these intervals only cull work.
std::vector<TunnelColumn> tunnelColumns(const Grid&g,const Tunnel&cut,Cell low,Cell high,int&axis,std::uint64_t&count){
 axis=0;for(int a=1;a<3;++a)if(std::abs(cut.direction[a])>std::abs(cut.direction[axis]))axis=a;
 const int u=(axis+1)%3,v=(axis+2)%3;
 const double d=cut.direction[axis],a=std::max(0.,1-d*d),radius=cut.maxRadius();
 std::vector<TunnelColumn> out;count=0;
 for(int i=low[u];i<=high[u];++i)for(int j=low[v];j<=high[v];++j){
  const double qu=g.origin[u]+(i+.5)*g.pitch-cut.entry[u],qv=g.origin[v]+(j+.5)*g.pitch-cut.entry[v];
  const double t=qu*cut.direction[u]+qv*cut.direction[v];
  double lo=(-cut.inlet-t)/d,hi=(cut.penetration-t)/d;if(lo>hi)std::swap(lo,hi);
  const double b=-d*t,c=qu*qu+qv*qv-t*t-radius*radius;
  if(a>1e-14){const double disc=b*b-a*c;if(disc< -1e-14)continue;
   const double root=std::sqrt(std::max(0.,disc));lo=std::max(lo,(-b-root)/a);hi=std::min(hi,(-b+root)/a);
  }else if(c+std::min(2*b*lo,2*b*hi)>1e-14)continue;
  if(cut.slab){
   // Intersect the column with the actual finite sheet before charging work.
   // The enclosing cylinder remains a conservative first cull, but charging
   // its empty volume rejected wide, thin blades that fit the same cell cap.
   const std::array<Vec,2> normals{{cut.slab_n,cut.slab_s}};
   const std::array<double,2> extents{{cut.slabExtent(),cut.slab_w}};
   for(unsigned plane=0;plane<2;++plane){const auto&n=normals[plane];
    const double c0=qu*n[u]+qv*n[v],slope=n[axis],extent=extents[plane];
    if(std::abs(slope)<1e-14){if(std::abs(c0)>extent+1e-12){lo=1;hi=0;break;}}
    else{double x0=(-extent-c0)/slope,x1=(extent-c0)/slope;if(x0>x1)std::swap(x0,x1);
     lo=std::max(lo,x0);hi=std::min(hi,x1);}
   }
  }
  if(lo>hi+1e-12)continue;
  // One cell of roundoff guard is filtered by exact analytic membership.
  const int first=std::max(low[axis],int(std::floor((lo+cut.entry[axis]-g.origin[axis])/g.pitch))-1);
  const int last=std::min(high[axis],int(std::ceil((hi+cut.entry[axis]-g.origin[axis])/g.pitch))+1);
  if(first>last)continue;
  count+=std::uint64_t(last-first+1);if(count>134217728)throw RecoverableImpactError("impact_cell_work_budget");
  Cell start{};(&start.x)[axis]=first;(&start.x)[u]=i;(&start.x)[v]=j;out.push_back({start,last});
 }
 return out;
}
core::HitResult eraseTunnel(Grid&g,Vec origin,Vec direction,double radius,double penetration,std::size_t* edgeContactCells=nullptr,const core::BodyBVH* envelope=nullptr,double exitMult=1,TunnelExit* exit=nullptr,std::uint64_t edgeSeed=0,double cavitation=0,double burst=0,const painter_detail::TissueField* countField=nullptr,ResolvedTunnel* resolved=nullptr,bool* resolvedWritten=nullptr,double exitAt=-1,const March* knownMarch=nullptr,double variation=1.,const SlabSpec* slab=nullptr){
 if(resolvedWritten)*resolvedWritten=false;
 need(std::isfinite(radius)&&radius>=.0005&&radius<=.08&&std::isfinite(penetration)&&penetration>0&&penetration<=.5,"tunnel_size_limits");
#ifdef GFY_RX_PROF
 auto tx=[t=core::Clock::now()](const char*n)mutable{std::cerr<<"rxprof tunnel."<<n<<' '<<core::ms(t)<<std::endl;t=core::Clock::now();};
#else
 auto tx=[](const char*){};
#endif
 direction=unit(direction);auto march=knownMarch?*knownMarch:firstSolidCylinder(g,origin,direction,radius);core::HitResult result;tx("march");if(!march.found)return result;
 Tunnel cut{march.entry,direction,radius,penetration,std::min(radius,penetration)};
 if(slab){cut.slab=true;cut.slab_n=unit(slab->normal);cut.slab_s=unit(slab->side);cut.slab_h=slab->half_thickness;cut.slab_w=slab->half_width;
  need(std::isfinite(cut.slab_h)&&cut.slab_h>=.0005&&cut.slab_h<=.05&&std::isfinite(cut.slab_w)&&cut.slab_w>=.005&&cut.slab_w<=.5,"slab_size_limits");cut.slab_seed=slab->seed;cut.ragged=RaggedBlade(slab->seed,cut.slab_h,cut.slab_w,penetration);}
 else cut.setEdgeProfile(edgeSeed,variation);
 need(std::isfinite(exitMult)&&exitMult>=1&&exitMult<=2,"exit_size_limits");
 if(slab){} // a slab has no far-side flare: the box simply ends at its reach
 else if(exitAt>=0&&exitAt<=penetration){cut.exit_distance=exitAt;cut.exit_mult=exitMult;if(exit){exit->reached=true;exit->point=march.entry+direction*exitAt;}}
 else if(envelope){
  // Intersections with the closed anatomical envelope distinguish an actual
  // far-side exit from a channel that merely reaches a pre-existing wound.
  auto hits=envelope->crossingDistances(march.entry,direction);
  const std::size_t exitIndex=hits.size()%2?0:1;
  if(exitIndex<hits.size()&&penetration>=hits[exitIndex]){
   cut.exit_distance=hits[exitIndex];cut.exit_mult=exitMult;
   if(exit){exit->reached=true;exit->point=march.entry+direction*cut.exit_distance;}
  }
 }
 // Filled voxel centers are only a sampled interior, not the visible skin.
 // Keep the mouth connected back to the mapped surface's exterior ray origin.
 // The same removal mask clips the skin and the cavity; stopping it at the
 // first filled cell can leave an untouched skin sheet over an internal cut.
 // This extends the inlet only: radius and the distal penetration tip stay fixed.
 cut.inlet=std::max(0.,dot(march.entry-origin,direction));
 if(!slab)cut.setForensics(cavitation,burst,edgeSeed?edgeSeed:0x5EED1234ull);
 result.cutter=march.entry+direction*(penetration*.5);result.surface=length(march.entry-origin);
 Vec begin=march.entry-direction*cut.inlet,end=march.entry+direction*penetration;Cell low,high;
 const double boundRadius=cut.maxRadius();
 for(int a=0;a<3;++a){(&low.x)[a]=std::max(0,int(std::floor((std::min(begin[a],end[a])-boundRadius-g.origin[a])/g.pitch)));(&high.x)[a]=std::min(g.size[a]-1,int(std::floor((std::max(begin[a],end[a])+boundRadius-g.origin[a])/g.pitch)));}
 tx("envelope");int axis=0;std::uint64_t estimate=0;std::vector<TunnelColumn> columns;
 // tunnelColumns rejects a cut over 2^27 cells (impact_cell_work_budget) before any mutation; its columns lie in the
 // window, so a window of at most that many cells cannot be rejected, and the list is built only for the column
 // backend below (a tile-planner refusal).
 const std::uint64_t windowCells=std::uint64_t(std::max(0,high.x-low.x+1))*std::uint64_t(std::max(0,high.y-low.y+1))*std::uint64_t(std::max(0,high.z-low.z+1));
 bool haveColumns=false;if(windowCells>134217728){columns=tunnelColumns(g,cut,low,high,axis,estimate);haveColumns=true;} // Reject before mutation.
 tx("columns");

 // Exact block-native backend. Keep the original analytic work admission
 // above, and keep the column backend for owned grids or scratch refusal.
 if(!std::getenv("GFY_RX_NO_TILE_NATIVE")){
  auto plan=painter_detail::planTileTunnel(g,cut,low,high,countField,
   [&](std::size_t n,std::size_t grain,auto&&body){core::parallelChunks(n,grain,body);});
  tx("plan");
  if(plan.supported){
#ifdef GFY_RX_VERIFY
   // Independent cell oracle BEFORE mutation: every planned bit must be the
   // exact analytic cut union old state; all anatomy counters remain exact.
   std::size_t mask=0,solid=0;std::array<std::size_t,64> slots{};
   std::vector<Cell> referenceBoundary;
   for(int x=low.x;x<=high.x;++x)for(int y=low.y;y<=high.y;++y)for(int z=low.z;z<=high.z;++z){
    const Cell c{x,y,z};const bool was=g.removed(c),inside=cut.contains(g.center(c));
    const Cell key=Grid::tileKey(c);const auto index=plan.index(key);const auto*final=plan.finalTile(g,key);
    need(bool(final&&core::tileBit(*final,c))==(was||inside),"tile_verify_mask");
    const bool fresh=!was&&inside;
    need(bool(index!=decltype(plan)::noStage&&core::tileBit(plan.stages[index].fresh,c))==fresh,"tile_verify_fresh");
    if(!fresh)continue;++mask;
    if(const auto label=g.materialBase(c)){++solid;const auto slot=g.material_slots&&label<g.material_slots->size()?unsigned((*g.material_slots)[label]&63u):1u;++slots[slot];}
    bool edge=false;for(int a=0;a<3&&!edge;++a)for(int sign:{-1,1}){Cell q=c;(&q.x)[a]+=sign;
     if(!g.valid(q)||(!g.removed(q)&&!cut.contains(g.center(q)))){edge=true;break;}}
    if(edge)referenceBoundary.push_back(c);
   }
   need(mask==plan.freshMask&&solid==plan.freshSolid,"tile_verify_counts");
   need(slots==plan.freshBySlot,"tile_verify_slots");
   if(solid){auto planned=plan.boundary;std::sort(planned.begin(),planned.end());
    need(referenceBoundary==planned,"tile_verify_boundary");}
#endif
   if(!plan.freshSolid)return result;
   result.freshMask=plan.freshMask;result.freshSolid=plan.freshSolid;result.freshBySlot=plan.freshBySlot;
   const auto mutationStart=core::Clock::now();
   // Allocate/journal every destination before any mask assignment. References
   // into unordered_map nodes survive later rehashes. The outer event journal
   // restores both new and existing tiles if any later phase throws.
   std::vector<core::Tile*> destinations(plan.touched.size());
   for(std::size_t i=0;i<plan.touched.size();++i)destinations[i]=&core::journaledTile(g,plan.touched[i]); // journaled, no dense window
   core::parallelChunks(plan.touched.size(),4096,[&](std::size_t first,std::size_t last,std::size_t){
    for(std::size_t i=first;i<last;++i)*destinations[i]=plan.stages[plan.index(plan.touched[i])].finalMask;
   });
   std::set<Cell> touched(plan.touched.begin(),plan.touched.end());
#ifdef GFY_RX_PROF
   std::cerr<<"tileprof classify="<<plan.classify_ms<<" boundary="<<plan.boundary_ms
    <<" mutate="<<core::ms(mutationStart)<<" full="<<plan.fullTiles<<" partial="<<plan.unresolvedTiles
    <<" old_full="<<plan.oldFullTiles<<" bytes="<<plan.scratchBytes<<std::endl;
#endif
   tx("mutate");decltype(plan.stages)().swap(plan.stages);decltype(plan.stageAt)().swap(plan.stageAt);decltype(plan.stageKeys)().swap(plan.stageKeys);
   tx("release");std::size_t extraSolid=0;const auto extras=resolveEdgeContacts(g,plan.boundary,result.cutter,
    [&](Cell c){return cut.intersectsCell(g,c);},touched,&extraSolid,!g.ownership);tx("edge_contacts");
   if(edgeContactCells)*edgeContactCells=extras;
   result.freshMask+=extras;result.freshSolid+=extraSolid;
   result.dirty=core::expandDirtyTiles(touched);tx("dirty");
   if(resolved){*resolved=resolvedTunnel(cut,edgeSeed,extras);if(resolvedWritten)*resolvedWritten=true;} // R24: slabs are recorded too
   return result;
  }
#ifdef GFY_RX_PROF
  std::cerr<<"tileprof fallback="<<plan.refusal<<std::endl;
#endif
 }
#ifdef GFY_RX_PROF
 std::size_t segStats[3]{},segCells[3]{};
 auto rx=[t=core::Clock::now()](const char*n)mutable{std::cerr<<"rxprof erase."<<n<<' '<<core::ms(t)<<std::endl;t=core::Clock::now();};
#else
 auto rx=[](const char*){};
#endif
 if(!haveColumns)columns=tunnelColumns(g,cut,low,high,axis,estimate);
 // Bounded bit raster over exactly the conservative columns: one inside bit
 // (Tunnel::contains at the cell center) and one fresh bit per column cell.
 // Memory is 2 bits per visited cell (the work budget caps it at 32 MiB), not
 // a Cell per fresh voxel. Membership, boundary prediction, mutation order and
 // tile creation order are identical to the former per-cell traversal.
 const int u=(axis+1)%3,v=(axis+2)%3;
 const int nu=high[u]-low[u]+1,nv=high[v]-low[v]+1;
 std::vector<std::int32_t>columnAt(std::size_t(std::max(nu,0))*std::size_t(std::max(nv,0)),-1);
 std::vector<std::uint64_t>offset(columns.size()+1);
 std::uint64_t bitCount=0;
 // Column bit ranges start on 64-bit words so columns never share a word
 // (parallel classification below writes disjoint words).
 for(std::size_t k=0;k<columns.size();++k){const auto&col=columns[k];offset[k]=bitCount;bitCount+=(std::uint64_t(col.end-col.first[axis]+1)+63)&~std::uint64_t(63);
  columnAt[std::size_t(col.first[u]-low[u])*nv+std::size_t(col.first[v]-low[v])]=std::int32_t(k);}
 offset[columns.size()]=bitCount;
 std::vector<std::uint64_t>inside((bitCount+63)/64+1),fresh((bitCount+63)/64+1);
 auto setBit=[](std::vector<std::uint64_t>&bits,std::uint64_t i){bits[i>>6]|=std::uint64_t(1)<<(i&63);};
 auto getBit=[](const std::vector<std::uint64_t>&bits,std::uint64_t i){return ((bits[i>>6]>>(i&63))&1u)!=0;};
 // OR the low `len` (<=64) bits of `mask` at bit position `pos` (vectors carry one pad word).
 auto orBits=[](std::vector<std::uint64_t>&bits,std::uint64_t pos,std::uint64_t mask,int len){
  if(len<64)mask&=(std::uint64_t(1)<<len)-1;const unsigned sh=unsigned(pos&63);
  // Touch the next word only when bits land there: a read-modify-write of a
  // neighbouring column's word (even OR 0) would race with its worker.
  bits[pos>>6]|=mask<<sh;if(sh){const std::uint64_t spill=mask>>(64-sh);if(spill)bits[(pos>>6)+1]|=spill;}};
 // Up to 64 bits starting at an arbitrary bit position (vectors carry one pad word).
 auto bitsAt=[](const std::vector<std::uint64_t>&bits,std::uint64_t pos,int len)->std::uint64_t{
  const std::uint64_t w=pos>>6;const unsigned sh=unsigned(pos&63);std::uint64_t r=bits[w]>>sh;
  if(sh&&int(64-sh)<len)r|=bits[w+1]<<(64-sh);
  return len>=64?r:(r&((std::uint64_t(1)<<len)-1));};
 const bool owned=bool(g.ownership);
 core::TileWindow window(g,low,high); // No tile is created before classification ends.
 // Exact Grid::solidBase for an unowned anatomy grid along the lattice z axis:
 // the same center/floor arithmetic as material_volume::Sparse::point, with a
 // monotone run cursor replacing the per-cell binary search.
 const bool zAnatomy=!owned&&g.anatomy&&axis==2;
 // The field is built from this prepared asset's immutable anatomy/slot map.
 // Reuse its compressed exact tissue slots for non-z raster counts instead
 // of binary-searching an anatomical z-run at each newly removed cell.
 // Keep the faster monotone z-run path. Owned fragments retain materialBase
 // because ownership and cuttable tissue are different predicates.
 const bool fieldCounts=!owned&&!zAnatomy&&g.anatomy&&g.material_slots&&
  countField&&countField->available&&countField->size==g.anatomy->size;
 // Reuse the exact source-double coordinate conversion along each lattice
 // axis. Coordinates are separable, so the same floor need not be recomputed
 // for every cell in a large volume. Only the bounded cutter window is cached.
 std::array<std::vector<int>,3> anatomyCoordinates;
 if(zAnatomy||fieldCounts){const auto& anatomy=*g.anatomy;
  for(int a=0;a<3;++a){auto& map=anatomyCoordinates[a];map.resize(std::size_t(high[a]-low[a]+1));
   for(int c=low[a];c<=high[a];++c){Cell cell{};(&cell.x)[a]=c;
    map[std::size_t(c-low[a])]=int(std::floor((g.center(cell)[a]-anatomy.origin[a])/anatomy.pitch));}}}
 auto anatomyCoordinate=[&](Cell c,int a){return anatomyCoordinates[a][std::size_t(c[a]-low[a])];};
 auto fieldSlotAt=[&](Cell c)->unsigned{
  return countField->at(anatomyCoordinate(c,0),anatomyCoordinate(c,1),anatomyCoordinate(c,2));
 };
 // One certificate per 8^3 brick, reused by its 64 columns. At most one MiB;
 // outside that bounded dense window retain the original rasterizer.
 std::vector<std::int8_t> certificates;
 // A full valid cutter tile can be counted and written as eight words. The
 // raster is still populated exactly for the existing boundary/repair path.
 // Owned grids and clipped edge tiles retain their former paths completely.
 struct FullTile {std::uint16_t fresh{};unsigned char firstColumn{255};signed char slot{-1};};
 std::vector<FullTile> fullTiles;
 static const bool fullTilesEnabled=std::getenv("GFY_RX_NO_FULL_TILE")==nullptr;
 const bool fullTileField=!owned&&g.anatomy&&g.material_slots&&countField&&countField->available&&countField->size==g.anatomy->size;
 if(window.enabled&&bitCount>(1ull<<20)){
  certificates.resize(window.ptr.size());
  if(!owned&&fullTilesEnabled)fullTiles.resize(window.ptr.size());
  core::parallelChunks(certificates.size(),4096,[&](std::size_t first,std::size_t last,std::size_t){
   for(std::size_t i=first;i<last;++i){
    // A stored all-removed tile cannot gain fresh cells. Treat it as known
    // removed in the boundary raster without re-evaluating the cutter. This
    // is valid for anatomy and original air, and also for owned fragments:
    // stored removal implies removed regardless of the ownership mask.
    const auto* previous=window.ptr[i];
    if(previous){std::uint64_t all=~std::uint64_t(0);for(auto word:previous->removed)all&=word;
     if(all==~std::uint64_t(0)){certificates[i]=2;continue;}}
    const int z=int(i%window.n.z),y=int((i/window.n.z)%window.n.y),x=int(i/(std::size_t(window.n.y)*window.n.z));
    const Vec center=g.origin+Vec{double((window.lo.x+x)*8+4),double((window.lo.y+y)*8+4),double((window.lo.z+z)*8+4)}*g.pitch;
    certificates[i]=std::int8_t(cut.classifyBox(center,Vec{3.5,3.5,3.5}*g.pitch));
    if(certificates[i]!=1||fullTiles.empty())continue;
    const Cell firstCell{(window.lo.x+x)*8,(window.lo.y+y)*8,(window.lo.z+z)*8};
    const Cell lastCell{firstCell.x+7,firstCell.y+7,firstCell.z+7};
    bool complete=true;for(int a=0;a<3;++a)complete=complete&&firstCell[a]>=low[a]&&lastCell[a]<=high[a];
    if(!complete)continue; // low/high already lie inside the valid grid
    auto& full=fullTiles[i];full.firstColumn=painter_detail::fullTileFirstFreshColumn(previous?&previous->removed:nullptr,axis);
    if(fullTileField){std::array<int,3> al,ah;for(int a=0;a<3;++a){al[a]=anatomyCoordinate(firstCell,a);ah[a]=anatomyCoordinate(lastCell,a);}
     full.slot=static_cast<signed char>(painter_detail::fullTileUniformSlot(*countField,al,ah));
     if(full.slot>=0){unsigned count=0;for(unsigned w=0;w<8;++w)count+=unsigned(__popcnt64(previous?~previous->removed[w]:~std::uint64_t(0)));full.fresh=std::uint16_t(count);}}
   }
  });
 }

 Vec step{};step[axis]=g.pitch;
 std::size_t freshMask=0,freshSolid=0;
 // Large cuts classify disjoint column ranges on up to four threads
 // (core::parallelChunks); every column is a pure function of read-only data
 // and writes only its own bit words; integer counts are summed afterwards.
 // Large cuts: blocks of 64 columns pulled dynamically by the parallel width
 // (core::parallelChunks); column cost varies strongly across the section.
 const bool big=bitCount>(std::uint64_t(1)<<20);const std::size_t blockColumns=64,blocks=big?(columns.size()+blockColumns-1)/blockColumns:1;
 auto forBlocks=[&](auto&&body){ // body(firstColumn,lastColumn,block,worker)
  if(!big){body(std::size_t(0),columns.size(),std::size_t(0),std::size_t(0));return;}
  std::atomic<std::size_t>cursor{0};
  core::parallelChunks(core::parallelWidth(),1,[&](std::size_t worker,std::size_t,std::size_t){
   for(std::size_t b;(b=cursor.fetch_add(1))<blocks;)body(b*blockColumns,std::min(columns.size(),(b+1)*blockColumns),b,worker);});};
 struct ChunkCounts{std::size_t mask{},solid{};std::array<std::size_t,64> slot{};};std::vector<ChunkCounts>chunkCounts(core::parallelWidth());
 const auto*slots=g.material_slots.get();auto slotOf=[slots](std::uint16_t label)->unsigned{return slots&&label<slots->size()?unsigned((*slots)[label]&63u):1u;};
 std::vector<std::vector<Cell>>chunkBoundary(blocks);
 forBlocks([&](std::size_t firstColumn,std::size_t lastColumn,std::size_t,std::size_t chunk){
 auto&freshMask=chunkCounts[chunk].mask;auto&freshSolid=chunkCounts[chunk].solid;auto&freshSlot=chunkCounts[chunk].slot;
#ifdef GFY_RX_PROF
 std::size_t segStats[3]{},segCells[3]{};
#endif
 for(std::size_t k=firstColumn;k<lastColumn;++k){const auto&col=columns[k];std::uint64_t bit=offset[k];
  const core::material_volume::Run*run=nullptr,*runEnd=nullptr,*runBegin=nullptr;bool columnAir=true;
  if(zAnatomy){const auto&a=*g.anatomy;const Vec p=g.center(col.first);
   const int ax=anatomyCoordinate(col.first,0),ay=anatomyCoordinate(col.first,1);
   if(ax>=0&&ay>=0&&std::uint32_t(ax)<a.size[0]&&std::uint32_t(ay)<a.size[1]){const auto cc=std::size_t(ay)*a.size[0]+std::size_t(ax);
    runBegin=run=a.runs.data()+a.offsets[cc];runEnd=a.runs.data()+a.offsets[cc+1];columnAir=false;}}
  // Exact per-cell classification of cell a (inside bit already known).
  auto classifyCell=[&](Cell c,std::uint64_t bit){
   setBit(inside,bit);
   if(owned&&!g.ownerCuttable(c))return;
   if(window.bit(c))return;
   setBit(fresh,bit);++freshMask;
   if(zAnatomy){if(columnAir)return;const auto&a=*g.anatomy;
    const int az=anatomyCoordinate(c,2);
    if(az<0||std::uint32_t(az)>=a.size[2])return;
    while(run!=runEnd&&int(run->lo)<=az)++run;
    if(run!=runBegin&&az<int((run-1)->hi)&&(run-1)->label){++freshSolid;++freshSlot[slotOf((run-1)->label)];}}
   else if(fieldCounts){if(const auto slot=fieldSlotAt(c)){++freshSolid;++freshSlot[slot];}}
   else if(const auto label=g.materialBase(c)){++freshSolid;++freshSlot[slotOf(label)];}
  };
  const int f=col.first[axis];
  // Certified-inside range [a0,a0+n): same bits and counts as classifyCell.
  auto insideRange=[&](int a0,int n){
   Cell c0=col.first;(&c0.x)[axis]=a0;const std::uint64_t bit0=offset[k]+std::uint64_t(a0-f);
   if(!zAnatomy){
    if(!fieldCounts){for(int i=0;i<n;++i){Cell c=c0;(&c.x)[axis]=a0+i;classifyCell(c,bit0+std::uint64_t(i));}return;}
    // Certified unowned range: write mask words once and inspect a tile once
    // per eight axis cells. Uniform immutable anatomy bricks count by popcount;
    // mixed/border spans keep exact source-double cell-to-anatomy arithmetic.
    for(int i=0;i<n;i+=64)orBits(inside,bit0+std::uint64_t(i),~std::uint64_t(0),std::min(64,n-i));
    const auto& an=*g.anatomy;const Vec firstPoint=g.center(c0);
    int sample[3];for(int a=0;a<3;++a)sample[a]=anatomyCoordinate(c0,a);
    auto anatomyAxis=[&](int at){Cell c=c0;(&c.x)[axis]=at;return anatomyCoordinate(c,axis);};
    for(int at=a0;at<a0+n;){const int lo=at,hi=std::min(a0+n-1,(at&~7)+7);at=hi+1;
     Cell c=c0;(&c.x)[axis]=lo;const auto* tile=window.find(Grid::tileKey(c));unsigned oldRow=0;
     if(tile){if(axis==0){const unsigned shift=unsigned((c.y&7)*8+(c.z&7));for(unsigned b=0;b<8;++b)oldRow|=unsigned((tile->removed[b]>>shift)&1u)<<b;}
      else{const std::uint64_t word=tile->removed[c.x&7];for(unsigned b=0;b<8;++b)oldRow|=unsigned((word>>(b*8+unsigned(c.z&7)))&1u)<<b;}}
     const unsigned span=((1u<<(hi-lo+1))-1u)<<(lo&7),freshRow=span&~oldRow;if(!freshRow)continue;
     orBits(fresh,bit0+std::uint64_t(lo-a0),std::uint64_t(freshRow>>(lo&7)),hi-lo+1);
     if(!fullTiles.empty()&&fullTiles[window.index(Grid::tileKey(c))].slot>=0)continue; // counted once per full tile below
     const std::size_t count=std::size_t(__popcnt(freshRow));freshMask+=count;
     const int al=anatomyAxis(lo),ah=anatomyAxis(hi);sample[axis]=al;
     bool in=al>=0&&ah>=al&&std::uint32_t(ah)<an.size[axis];
     for(int a=0;a<3&&in;++a)if(a!=axis)in=sample[a]>=0&&std::uint32_t(sample[a])<an.size[a];
     if(in&&(al>>3)==(ah>>3)){
      const std::size_t page=(std::size_t(sample[2]>>3)*countField->pages[1]+unsigned(sample[1]>>3))*countField->pages[0]+unsigned(sample[0]>>3)+painter_detail::TissueField::headerWords;
      const auto value=countField->page[page];if(value&painter_detail::TissueField::uniformBit){const unsigned slot=value&63u;if(slot){freshSolid+=count;freshSlot[slot]+=count;}continue;}}
     for(unsigned bits=freshRow;bits;bits&=bits-1){sample[axis]=anatomyAxis((lo&~7)+int(core::lowestBit(bits)));
      if(const auto slot=countField->at(sample[0],sample[1],sample[2])){++freshSolid;++freshSlot[slot];}}
    }
    return;
   }
   // Lattice z column of an unowned anatomy grid: one tile byte row at a time.
   for(int i=0;i<n;i+=64)orBits(inside,bit0+std::uint64_t(i),~std::uint64_t(0),std::min(64,n-i));
   for(int z=a0;z<a0+n;){const int zlo=z,zhi=std::min(a0+n-1,(z&~7)+7);z=zhi+1;
    Cell c=c0;c.z=zlo;const core::Tile*t=window.find(Grid::tileKey(c));
    const unsigned oldRow=t?unsigned((t->removed[c.x&7]>>((c.y&7)*8))&255u):0u;
    const unsigned span=((1u<<(zhi-zlo+1))-1u)<<(zlo&7);
    const unsigned freshRow=~oldRow&span;if(!freshRow)continue;
    orBits(fresh,bit0+std::uint64_t(zlo-a0),std::uint64_t(freshRow>>(zlo&7)),zhi-zlo+1);
    if(!fullTiles.empty()&&fullTiles[window.index(Grid::tileKey(c))].slot>=0)continue; // counted once per full tile below
    const std::size_t count=std::size_t(__popcnt(freshRow));freshMask+=count;
    if(columnAir)continue;const auto&an=*g.anatomy;
    auto anatomyZ=[&](int zc){Cell q=c0;q.z=zc;return anatomyCoordinate(q,2);};
    const int azLo=anatomyZ(zlo),azHi=anatomyZ(zhi);
    if(azLo>=0&&std::uint32_t(azHi)<an.size[2]){
     while(run!=runEnd&&int(run->lo)<=azLo)++run;
     const auto*prev=run!=runBegin?run-1:nullptr;
     if(prev&&azLo<int(prev->hi)&&azHi<int(prev->hi)){if(prev->label){freshSolid+=count;freshSlot[slotOf(prev->label)]+=count;}continue;}  // one run (az monotone)
     if((!prev||azLo>=int(prev->hi))&&(run==runEnd||azHi<int(run->lo)))continue;                      // one gap
    }
    for(unsigned m=freshRow;m;m&=m-1){const int zz=(zlo&~7)+int(core::lowestBit(m));
     const int az=anatomyZ(zz);if(az<0||std::uint32_t(az)>=an.size[2])continue;
     while(run!=runEnd&&int(run->lo)<=az)++run;
     if(run!=runBegin&&az<int((run-1)->hi)&&(run-1)->label){++freshSolid;++freshSlot[slotOf((run-1)->label)];}}
   }
  };
  // Conservative 16-cell segment certification (bisecting undecided segments was measured slower;
  // ascending order); remaining cells use the exact
  // Tunnel::contains, so every inside bit equals contains(center).
  auto segment=[&](auto&&self,int a0,int n)->void{
   Cell c0=col.first;(&c0.x)[axis]=a0;
   const int certain=cut.classifySegment(g.center(c0),step,n);
#ifdef GFY_RX_PROF
   ++segStats[certain+1];segCells[certain+1]+=std::size_t(n);
#endif
   if(certain<0)return;
   if(certain>0){insideRange(a0,n);return;}
   const std::uint64_t bit0=offset[k]+std::uint64_t(a0-f);
   for(int i=0;i<n;++i){Cell c=c0;(&c.x)[axis]=a0+i;if(cut.contains(g.center(c)))classifyCell(c,bit0+std::uint64_t(i));}
  };
  // Long certification spans only where t is strictly inside the core t range
  // (then classifySegment can accept whole runs of a column); elsewhere, and
  // wherever a long span is undecided, 16-cell segments as before.
  auto blocks=[&](int from,int to){for(int a0=from;a0<=to;a0+=16)segment(segment,a0,std::min(16,to-a0+1));};
  auto span=[&](auto&&self,int from,int to)->void{
   const int n=to-from+1;if(n<=32){blocks(from,to);return;}
   Cell c0=col.first;(&c0.x)[axis]=from;const int certain=cut.classifySegment(g.center(c0),step,n);
   if(certain<0)return;if(certain>0){insideRange(from,n);return;}
   const int mid=from+((n/2+15)/16)*16-1;self(self,from,mid);self(self,mid+1,to);};
  if(!certificates.empty()){
   for(int a0=f;a0<=col.end;){Cell c=col.first;(&c.x)[axis]=a0;
    const int certain=certificates[window.index(Grid::tileKey(c))];
    int to=std::min(col.end,(a0&~7)+7);
    // Coalesce adjacent bricks with the same certificate. Unresolved spans
    // still use the established recursive segment certificates.
    while(to<col.end){Cell next=col.first;(&next.x)[axis]=to+1;
     if(certificates[window.index(Grid::tileKey(next))]!=certain)break;
     to=std::min(col.end,(to+1)+7);
    }
    if(certain==2){
     // The raster is an inside-or-previously-removed certificate. No fresh
     // bit, material count, journal entry or tile creation is generated.
     const auto begin=offset[k]+std::uint64_t(a0-f);
     for(int d=0;d<=to-a0;d+=64)orBits(inside,begin+std::uint64_t(d),~std::uint64_t(0),std::min(64,to-a0+1-d));
    }else if(certain>0)insideRange(a0,to-a0+1);else if(!certain)span(span,a0,to);
    a0=to+1;
   }
  }else {const Vec q0=g.center(col.first)-cut.entry;const double t0=dot(q0,cut.direction),dt=dot(step,cut.direction);
   const double coreEnd=cut.exit_distance<0?cut.penetration-cut.cap:cut.penetration;
   int from=f,to=col.end; // cells whose t lies in (-inlet, coreEnd) with a one-cell guard
   if(std::abs(dt)>1e-12){double s0=(-cut.inlet-t0)/dt,s1=(coreEnd-t0)/dt;if(s0>s1)std::swap(s0,s1);
    from=std::max(f,f+int(std::ceil(s0))+1);to=std::min(col.end,f+int(std::floor(s1))-1);}
   if(from>to){blocks(f,col.end);}
   else{if(from>f)blocks(f,from-1);span(span,from,to);if(to<col.end)blocks(to+1,col.end);}}
  }});
 for(const auto&c:chunkCounts){freshMask+=c.mask;freshSolid+=c.solid;for(unsigned s=0;s<64;++s)result.freshBySlot[s]+=c.slot[s];}
 for(const auto&full:fullTiles)if(full.slot>=0){freshMask+=full.fresh;if(full.slot){freshSolid+=full.fresh;result.freshBySlot[unsigned(full.slot)]+=full.fresh;}}
 rx("classify");
#ifdef GFY_RX_VERIFY
 // TEST-ONLY: the raster must equal the per-cell definition exactly.
 {core::TileCache verify(g);std::size_t mask=0,solid=0;std::array<std::size_t,64> bySlot{};
  for(std::size_t k=0;k<columns.size();++k){const auto&col=columns[k];std::uint64_t bit=offset[k];
   for(Cell c=col.first;c[axis]<=col.end;++(&c.x)[axis],++bit){const bool in=cut.contains(g.center(c));
    need(getBit(inside,bit)==in||(getBit(inside,bit)&&window.bit(c)),"rx_verify_inside_or_old");const bool fr=in&&!verify.removed(c);need(getBit(fresh,bit)==fr,"rx_verify_fresh");
    if(fr){++mask;if(const auto label=g.materialBase(c)){++solid;++bySlot[slotOf(label)];}}}}
  need(mask==freshMask&&solid==freshSolid,"rx_verify_counts");need(bySlot==result.freshBySlot,"rx_verify_slot_counts");}
#endif
 if(!freshSolid)return result; // A sub-cell cut is an explicit unmodified no-op.
 result.freshMask=freshMask;result.freshSolid=freshSolid;
 // Exact cut-or-old-removal certificate for any cell: raster bit when the cell
 // lies in a visited column span, direct evaluation otherwise.
 auto insideCell=[&](Cell n)->bool{
  const int i=n[u]-low[u],j=n[v]-low[v];
  if(i>=0&&i<nu&&j>=0&&j<nv){const auto id=columnAt[std::size_t(i)*nv+std::size_t(j)];
   if(id>=0){const auto&col=columns[std::size_t(id)];const int a=n[axis];
    if(a>=col.first[axis]&&a<=col.end)return getBit(inside,offset[std::size_t(id)]+std::uint64_t(a-col.first[axis]));}}
  return cut.contains(g.center(n));
 };
 // Raster inside bits of column id for axis positions a0..a0+63 (0 outside its span).
 auto knownInside=[&](std::int32_t id,int a0)->std::uint64_t{
  if(id<0)return 0;const auto&col=columns[std::size_t(id)];const int f=col.first[axis];
  const int lo=std::max(a0,f),hi=std::min(a0+63,col.end);if(lo>hi)return 0;
  return bitsAt(inside,offset[std::size_t(id)]+std::uint64_t(lo-f),hi-lo+1)<<(lo-a0);};
 auto columnId=[&](int i,int j)->std::int32_t{return i<0||j<0||i>=nu||j>=nv?-1:columnAt[std::size_t(i)*nv+std::size_t(j)];};
 // Predict the final pre-repair mask. A neighbor that is removed before the
 // cut, foreign-owned (virtual removed) or admitted by this same analytic cut
 // cannot make this cell a boundary; an invalid neighbor always does. Cells
 // whose six neighbors all carry an inside raster bit are interior; only the
 // rest take the exact per-neighbor test.
 forBlocks([&](std::size_t firstColumn,std::size_t lastColumn,std::size_t block,std::size_t){
 auto&boundary=chunkBoundary[block];
 for(std::size_t k=firstColumn;k<lastColumn;++k){const auto&col=columns[k];const int f=col.first[axis];
  const int i=col.first[u]-low[u],j=col.first[v]-low[v];
  const std::int32_t side[4]={columnId(i-1,j),columnId(i+1,j),columnId(i,j-1),columnId(i,j+1)};
  for(int a0=f;a0<=col.end;a0+=64){const int len=std::min(64,col.end-a0+1);
   const std::uint64_t mine=bitsAt(fresh,offset[k]+std::uint64_t(a0-f),len);if(!mine)continue;
   const std::uint64_t interior=knownInside(std::int32_t(k),a0-1)&knownInside(std::int32_t(k),a0+1)&
    knownInside(side[0],a0)&knownInside(side[1],a0)&knownInside(side[2],a0)&knownInside(side[3],a0);
   for(std::uint64_t candidates=mine&~interior;candidates;candidates&=candidates-1){
    const unsigned b=core::lowestBit(candidates);
    Cell c=col.first;(&c.x)[axis]=a0+int(b);
    bool edge=false;
    for(int a=0;a<3&&!edge;++a)for(int sign:{-1,1}){
     Cell n=c;(&n.x)[a]+=sign;
     if(!g.valid(n)){edge=true;break;}
     if(insideCell(n))continue;
     if(owned&&!g.ownerCuttable(n))continue;
     if(window.bit(n))continue;
     edge=true;break;}
    if(edge)boundary.push_back(c);
   }}}});
 // Blocks are contiguous ascending column ranges: concatenation in block order keeps the order.
 std::vector<Cell>boundary;for(auto&part:chunkBoundary)boundary.insert(boundary.end(),part.begin(),part.end());
 rx("boundary");
#ifdef GFY_RX_VERIFY
 std::size_t beforeMutation=0;for(const auto& tile:g.tiles)for(auto word:tile.second.removed)beforeMutation+=std::size_t(__popcnt64(word));
#endif
 // Mutate in the former traversal order so tile creation order is unchanged.
 std::vector<Cell>touchedKeys;
 // Word 8 is a full-tile assignment at that tile's first formerly-fresh
 // column. It replaces all subsequent writes without advancing first touch.
 auto visitFullTile=[&](Cell c,Cell tile,auto&&visit)->bool{
  if(fullTiles.empty())return false;
  const auto& full=fullTiles[window.index(tile)];if(full.firstColumn==255)return false;
  if(unsigned((c[u]&7)*8+(c[v]&7))==full.firstColumn)visit(c,tile,std::size_t(8),~std::uint64_t(0));
  return true;
 };
 // For each fresh 8-aligned axis chunk (z columns) or fresh cell: visit(k,cell,mask-or-bit).
 auto visitFresh=[&](std::size_t k,auto&&visit){const auto&col=columns[k];const int f=col.first[axis];
  if(axis==2){for(int zlo=f;zlo<=col.end;){const int zhi=std::min(col.end,(zlo&~7)+7);
    const std::uint64_t row=bitsAt(fresh,offset[k]+std::uint64_t(zlo-f),zhi-zlo+1);
    if(row){Cell c=col.first;c.z=zlo;const Cell tile=Grid::tileKey(c);
     if(!visitFullTile(c,tile,visit))visit(c,tile,std::size_t(c.x&7),(row<<(zlo&7))<<((c.y&7)*8));}
    zlo=zhi+1;}return;}
  for(int a=f&~7;a<=col.end;a+=8){const int lo=std::max(a,f),hi=std::min(a+7,col.end);
   const std::uint64_t row=bitsAt(fresh,offset[k]+std::uint64_t(lo-f),hi-lo+1)<<(lo-a);if(!row)continue;
   Cell c=col.first;(&c.x)[axis]=a;const Cell tile=Grid::tileKey(c);
   if(visitFullTile(c,tile,visit))continue;
   if(axis==1){
    // Spread this column's eight y bits into the tile's eight y rows.
    std::uint64_t spread=row;spread=(spread|(spread<<28))&0x0000000F0000000Full;
    spread=(spread|(spread<<14))&0x0003000300030003ull;
    spread=(spread|(spread<<7))&0x0101010101010101ull;
    visit(c,tile,std::size_t(c.x&7),spread<<(c.z&7));
   }else{
    const std::uint64_t bit=std::uint64_t(1)<<((c.y&7)*8+(c.z&7));
    for(unsigned x=0;x<8;++x)if(row&(1ull<<x)){c.x=a+int(x);visit(c,tile,std::size_t(x),bit);}
   }
  }};
 if(big&&window.enabled){
  // Groups of consecutive columns sharing one tile coordinate along u touch
  // disjoint tiles. (A) per group, first-touched tile keys in traversal
  // order (parallel); (B) create tiles serially in group order = the former
  // global first-touch order; (C) OR the bits per group (parallel).
  std::vector<std::size_t>groupStart;for(std::size_t k=0;k<columns.size();++k)if(!k||(columns[k].first[u]>>3)!=(columns[k-1].first[u]>>3))groupStart.push_back(k);
  groupStart.push_back(columns.size());const std::size_t groups=groupStart.size()-1;
  std::vector<std::vector<Cell>>groupKeys(groups);std::vector<std::uint8_t>seen(window.ptr.size(),0);
  auto forGroups=[&](auto&&body){std::atomic<std::size_t>cursor{0};
   core::parallelChunks(core::parallelWidth(),1,[&](std::size_t,std::size_t,std::size_t){for(std::size_t gi;(gi=cursor.fetch_add(1))<groups;)body(gi);});};
  forGroups([&](std::size_t gi){auto&keys=groupKeys[gi];Cell last{INT_MIN,INT_MIN,INT_MIN};
   for(std::size_t k=groupStart[gi];k<groupStart[gi+1];++k)visitFresh(k,[&](Cell,Cell t,std::size_t,std::uint64_t){
    if(t==last)return;last=t;auto&flag=seen[window.index(t)];if(!flag){flag=1;keys.push_back(t);}});});
  for(const auto&keys:groupKeys)for(const Cell t:keys){window.tile(t);if(window.firstTouch(t))touchedKeys.push_back(t);}
  forGroups([&](std::size_t gi){Cell last{INT_MIN,INT_MIN,INT_MIN};core::Tile*tile=nullptr;
   for(std::size_t k=groupStart[gi];k<groupStart[gi+1];++k)visitFresh(k,[&](Cell,Cell t,std::size_t word,std::uint64_t bits){
    if(!(t==last)){last=t;tile=window.ptr[window.index(t)];}
    if(word==8)tile->removed.fill(~std::uint64_t(0));else tile->removed[word]|=bits;});});
 }else
 {Cell key{INT_MIN,INT_MIN,INT_MIN};core::Tile*tile=nullptr;
  for(std::size_t k=0;k<columns.size();++k){const auto&col=columns[k];const int f=col.first[axis];
   if(axis==2){
    // z column: one tile byte row per 8-aligned z chunk, in the same column
    // and z order, so tiles are created exactly when the per-cell loop did.
    for(int zlo=f;zlo<=col.end;){const int zhi=std::min(col.end,(zlo&~7)+7);
     const std::uint64_t row=bitsAt(fresh,offset[k]+std::uint64_t(zlo-f),zhi-zlo+1);
     if(row){Cell c=col.first;c.z=zlo;const Cell t=Grid::tileKey(c);
      if(!(t==key)){key=t;tile=&window.tile(t);if(window.firstTouch(t))touchedKeys.push_back(t);}
      tile->removed[c.x&7]|=(row<<(zlo&7))<<((c.y&7)*8);}
     zlo=zhi+1;}
    continue;}
   for(int a0=f;a0<=col.end;a0+=64){const int len=std::min(64,col.end-a0+1);
    for(std::uint64_t mine=bitsAt(fresh,offset[k]+std::uint64_t(a0-f),len);mine;mine&=mine-1){
     const unsigned b=core::lowestBit(mine);Cell c=col.first;(&c.x)[axis]=a0+int(b);
     const Cell t=Grid::tileKey(c);if(!(t==key)){key=t;tile=&window.tile(t);if(window.firstTouch(t))touchedKeys.push_back(t);}
     const auto local=Grid::local(c);tile->removed[local/64]|=std::uint64_t(1)<<(local%64);
    }}}}
#ifdef GFY_RX_VERIFY
 // TEST-ONLY: every fresh cell is now removed, and the removed count grew by exactly freshMask.
 {std::size_t now=0;for(const auto&t:g.tiles)for(auto w:t.second.removed)now+=std::size_t(__popcnt64(w));
  std::size_t fresh2=0;for(std::size_t k=0;k<columns.size();++k){const auto&col=columns[k];std::uint64_t bit=offset[k];
   for(Cell c=col.first;c[axis]<=col.end;++(&c.x)[axis],++bit)if(getBit(fresh,bit)){++fresh2;need(g.removed(c),"rx_verify_mutate");}}
  need(fresh2==freshMask,"rx_verify_mutate_count");need(now==beforeMutation+freshMask,"rx_verify_mutate_delta");}
#endif
 std::sort(touchedKeys.begin(),touchedKeys.end());touchedKeys.erase(std::unique(touchedKeys.begin(),touchedKeys.end()),touchedKeys.end());
 std::set<Cell>touched(touchedKeys.begin(),touchedKeys.end());
 inside=std::vector<std::uint64_t>();fresh=std::vector<std::uint64_t>();
 rx("mutate");std::size_t extraSolid=0;auto extras=resolveEdgeContacts(g,boundary,result.cutter,[&](Cell c){return cut.intersectsCell(g,c);},touched,&extraSolid);
 if(edgeContactCells)*edgeContactCells=extras;result.freshMask+=extras;result.freshSolid+=extraSolid;
 rx("edges");result.dirty=core::expandDirtyTiles(touched);rx("dirty");
 if(resolved){*resolved=resolvedTunnel(cut,edgeSeed,extras);if(resolvedWritten)*resolvedWritten=true;} // R24: slabs are recorded too
 return result;
}

struct Mapping {double posed_distance{};Vec entry,direction;std::uint32_t face{};std::array<double,3>bary{};Matrix blend{};};
// Rest -> posed skinned positions (the former inline loops, same arithmetic).
std::vector<Vec> posedVertices(const Source&s,const std::vector<Matrix>&palette){
 // gearbody: on 4 threads above 8192 vertices (the merged gear grows a GFY source by 27 %); each vertex as before
 std::vector<Vec>posed(s.v.size());auto pose=[&](std::size_t first,std::size_t last){for(std::size_t i=first;i<last;++i)for(unsigned k=0;k<4;++k)posed[i]=posed[i]+transform(palette[s.v[i].ids[k]],s.v[i].p)*s.v[i].w[k];};
 if(s.v.size()<8192)pose(0,s.v.size());else core::parallelChunks(s.v.size(),(s.v.size()+3)/4,[&](std::size_t first,std::size_t last,std::size_t){pose(first,last);});
 return posed;}
// Posed vertices shared by consecutive events of one batch that carry a
// bit-identical bone palette (e.g. several hits captured in the same frame).
struct PoseCache {
 std::vector<Matrix>palette;std::vector<Vec>source,occlusion;bool haveSource{},haveOcclusion{};
 const std::vector<Vec>&get(const Source&s,const std::vector<Matrix>&p,bool occlusionMesh){
  bool same=palette.size()==p.size();for(std::size_t i=0;same&&i<p.size();++i)same=palette[i].row_major==p[i].row_major;
  if(!same){palette=p;haveSource=haveOcclusion=false;}
  auto&v=occlusionMesh?occlusion:source;bool&have=occlusionMesh?haveOcclusion:haveSource;
  if(!have){v=posedVertices(s,p);have=true;}return v;}
};
Mapping sourceRay(const Source&s,const Impact&event,PoseCache*cache=nullptr,const std::vector<char>*allowed=nullptr,const gfy_packed::SkinOwnershipQuery*ownership=nullptr,const std::vector<std::uint16_t>*faceChildren=nullptr,const std::vector<std::uint32_t>*faceIndices=nullptr,unsigned defaultChild=0,const std::vector<Vec>*posedIn=nullptr){need(event.bone_model_render.size()==s.bones,"pose_palette_count");need(std::isfinite(event.source_search_distance)&&event.source_search_distance>0&&event.source_search_distance<=.5,"mapping_search_limit");for(auto&m:event.bone_model_render)for(double x:m.row_major)need(std::isfinite(x),"nonfinite_pose");
 Vec direction=unit(vec(event.accepted_pose_direction)),anchor=vec(event.accepted_pose_point);need(std::isfinite(anchor.x)&&std::isfinite(anchor.y)&&std::isfinite(anchor.z),"nonfinite_hit");Vec start=anchor-direction*event.source_search_distance;
 std::vector<Vec>local;if(!cache&&!posedIn)local=posedVertices(s,event.bone_model_render);const auto&posed=posedIn?*posedIn:cache?cache->get(s,event.bone_model_render,false):local; // R69: posedIn = the posed vertices of this palette
 // Face loop on up to four threads; each keeps its first strictly-smallest distance, the chunks merge by (distance,
 // lowest face): exactly the serial "first minimal face" rule.
 std::atomic<bool> ownerRejected{false};
 struct Best {double d=1e100;std::uint32_t f=~0u;double u=0,v=0,t=0;};std::vector<Best> bests(core::parallelWidth());const std::size_t F=s.f.size();
 core::parallelChunks(F,std::max<std::size_t>(1,(F+3)/4),[&](std::size_t a0,std::size_t b0,std::size_t chunk){Best best;
  for(std::uint32_t i=std::uint32_t(a0);i<b0;++i){if(allowed&&(i>=allowed->size()||!(*allowed)[i]))continue;auto ids=s.f[i];Vec a=posed[ids[0]],e1=posed[ids[1]]-a,e2=posed[ids[2]]-a,h=cross(direction,e2);double det=dot(e1,h);if(std::abs(det)<1e-14)continue;Vec ss=start-a;double u=dot(ss,h)/det;Vec q=cross(ss,e1);double v=dot(direction,q)/det,t=dot(e2,q)/det;if(u< -1e-8||v< -1e-8||u+v>1+1e-8||t<0||t>2*event.source_search_distance)continue;
   if(ownership){const auto rest=s.v[ids[0]].p*(1-u-v)+s.v[ids[1]].p*u+s.v[ids[2]].p*v;
    const auto key=faceChildren?((std::uint64_t(faceChildren->empty()?defaultChild:(*faceChildren)[i])<<32)|(faceIndices->empty()?i:(*faceIndices)[i])):std::uint64_t(i);
    if(!ownership->accepts(key,{float(rest.x),float(rest.y),float(rest.z)})){ownerRejected.store(true,std::memory_order_relaxed);continue;}}
   Vec restN=cross(s.v[ids[1]].p-s.v[ids[0]].p,s.v[ids[2]].p-s.v[ids[0]].p),normal=s.v[ids[0]].normal+s.v[ids[1]].normal+s.v[ids[2]].normal;double orientation=dot(restN,normal)>=0?1:-1;if(dot(cross(e1,e2),direction)*orientation>=0)continue;
   double distance=std::abs(t-event.source_search_distance);if(distance>=best.d)continue;best.d=distance;best.f=i;best.u=u;best.v=v;best.t=t;}
  bests[chunk]=best;});
 Best w;for(const auto&b:bests)if(b.f!=~0u&&(b.d<w.d||(b.d==w.d&&b.f<w.f)))w=b;
 need(w.f!=~0u,ownerRejected.load(std::memory_order_relaxed)?"source_surface_outside_owner":"no_source_surface_mapping");
 Mapping result;auto ids=s.f[w.f];result.posed_distance=w.t;result.face=w.f;result.bary={1-w.u-w.v,w.u,w.v};result.entry={};Matrix blend;
 for(unsigned j=0;j<3;++j){const auto&vert=s.v[ids[j]];result.entry=result.entry+vert.p*result.bary[j];for(unsigned k=0;k<4;++k)for(unsigned n=0;n<16;++n)blend.row_major[n]+=event.bone_model_render[vert.ids[k]].row_major[n]*vert.w[k]*result.bary[j];}
 result.direction=unit(inverseDirection(blend,direction));result.blend=blend;
 return result;
}
bool occluded(const Source&s,const Impact&event,double surface_distance,PoseCache*cache=nullptr,const gfy_packed::SkinOwnershipQuery*ownership=nullptr){
 Vec direction=unit(vec(event.accepted_pose_direction)),start=vec(event.accepted_pose_point)-direction*event.source_search_distance;
 std::vector<Vec>local;if(!cache)local=posedVertices(s,event.bone_model_render);const auto&posed=cache?cache->get(s,event.bone_model_render,true):local;
 std::atomic<bool> hit{false};const std::size_t F=s.f.size();
 core::parallelChunks(F,std::max<std::size_t>(1,(F+3)/4),[&](std::size_t a0,std::size_t b0,std::size_t){
  for(std::size_t i=a0;i<b0&&!hit.load(std::memory_order_relaxed);++i){auto ids=s.f[i];Vec a=posed[ids[0]],e1=posed[ids[1]]-a,e2=posed[ids[2]]-a,h=cross(direction,e2);double det=dot(e1,h);if(std::abs(det)<1e-14)continue;Vec ss=start-a;double u=dot(ss,h)/det;Vec q=cross(ss,e1);double v=dot(direction,q)/det,t=dot(e2,q)/det;if(u>=-1e-8&&v>=-1e-8&&u+v<=1+1e-8&&t>=0&&t<surface_distance-1e-5){
   if(ownership){const auto rest=s.v[ids[0]].p*(1-u-v)+s.v[ids[1]].p*u+s.v[ids[2]].p*v;
    if(!ownership->accepts(std::uint64_t(i),{float(rest.x),float(rest.y),float(rest.z)}))continue;}
   hit.store(true,std::memory_order_relaxed);return;}}});
 return hit.load();
}

}
WoundSurface woundSurface(const Source&source,std::uint32_t face,Vec p){
 WoundSurface out;if(face>=source.f.size())return out;
 const auto&ids=source.f[face];Vec a=source.v[ids[0]].p,b=source.v[ids[1]].p,c=source.v[ids[2]].p;
 Vec u=b-a,v=c-a,q=p-a;double uu=dot(u,u),uv=dot(u,v),vv=dot(v,v),qu=dot(q,u),qv=dot(q,v),den=uu*vv-uv*uv;
 if(std::abs(den)<1e-20)return out;
 double y=(vv*qu-uv*qv)/den,z=(uu*qv-uv*qu)/den;std::array<double,3>w{1-y-z,y,z};
 Vec normal;double sum=0;
 for(unsigned k=0;k<3;++k){if(w[k]<-1e-5)return out;w[k]=std::max(0.,w[k]);sum+=w[k];}
 for(unsigned k=0;k<3;++k){w[k]/=sum;const auto&vertex=source.v[ids[k]];normal=normal+vertex.normal*w[k];
  for(unsigned j=0;j<4;++j)out.weights[vertex.ids[j]]+=vertex.w[j]*w[k];}
 if(length(normal)<1e-10)normal=cross(u,v);
 if(length(normal)<1e-10)return out;
 out.rest_point=point(p);out.rest_normal=point(unit(normal));out.valid=true;return out;
}
bool recoverableMapping(const char* reason) noexcept {
 return !std::strcmp(reason,"source_surface_outside_owner") || !std::strcmp(reason,"no_source_surface_mapping") ||
        !std::strcmp(reason,"singular_skin_mapping");
}
Matrix Matrix::identity(){Matrix m;for(unsigned i=0;i<4;++i)m.row_major[i*5]=1;return m;}
bool Identity::operator==(const Identity&b)const noexcept{return spawn_cookie==b.spawn_cookie&&visual_lineage==b.visual_lineage&&topology==b.topology&&source==b.source&&exact_rig==b.exact_rig;}
#include "JointDamage.h"
#include "SeverCheck.h"
#include "DamageTear.h"
#include "CranialDamageMeasure.h"
#include "HeadFracture.h"
#include "SeamTear.h" // seam (value 3): lined-up hits split bodies
#include "BulletBurst.h" // bulletburst (value 3): bullet bursts and surface blowouts on the body
// T1 (2026-10-29): the torn end of a sever for the release log (ImpactResult::torn_end; one line per sever, pieces included)
inline void noteTornEnd(ImpactResult&r,const DamageTearVerdict&v){r.torn_end_ms=v.tissueMs;r.torn_end_triangles=v.tissueTriangles;
 if(v.tissueRan&&v.tissueTriangles){r.torn_end=1;r.torn_end_refusal.clear();return;}
 r.torn_end=2;r.torn_end_refusal=v.tissueRefusal?v.tissueRefusal:(v.tissueRan?"tissue_tear_no_surface":"tissue_tear_not_run");}
inline void noteTornEnd(ImpactResult&r,const head_fracture::Result&h){r.torn_end_ms=h.tissueMs;r.torn_end_triangles=h.tissueTriangles;
 if(h.tissueBuilt){r.torn_end=1;r.torn_end_refusal.clear();return;}
 r.torn_end=2;r.torn_end_refusal=h.tissueRefusal?h.tissueRefusal:"tissue_tear_not_run";}
struct OcularAccessoryData;
namespace fastwound {struct LimbRegions;}
std::shared_ptr<painter_detail::TissueField> renderFieldBytes(const std::vector<std::uint8_t>&raw){
 using TF=painter_detail::TissueField;
 const std::vector<std::uint8_t> bytes=raw.size()>=2&&raw[0]==31&&raw[1]==139?gfy_asset_codec::gunzip(raw):raw;
 Cursor c{bytes};c.magic("VXTFLD01");auto f=std::make_shared<TF>();
 for(auto&o:f->origin){o=c.take<float>();need(std::isfinite(o)&&std::abs(o)<100,"render_field_origin");}
 f->pitch=c.take<float>();need(f->pitch>=.0005f&&f->pitch<=.01f,"render_field_pitch");
 for(auto&n:f->size){n=c.take<std::uint32_t>();need(n&&n<65536,"render_field_size");}
 for(int a=0;a<3;++a){f->pages[a]=c.take<std::uint32_t>();need(f->pages[a]==(f->size[a]+7)/8,"render_field_pages");}
 f->mixed=c.take<std::uint32_t>();f->uniform=c.take<std::uint32_t>();f->empty=c.take<std::uint32_t>();
 const auto pageCount=c.take<std::uint64_t>(),dataCount=c.take<std::uint64_t>();
 const std::uint64_t bricks=std::uint64_t(f->pages[0])*f->pages[1]*f->pages[2];
 need(pageCount>=bricks+TF::headerWords&&pageCount%TF::pageWidth==0&&dataCount%TF::dataWidth==0&&pageCount*4+dataCount<=(96ull<<20),"render_field_storage");
 need(bytes.size()-c.at==pageCount*4+dataCount,"render_field_bytes");
 f->page.resize(std::size_t(pageCount));std::memcpy(f->page.data(),bytes.data()+c.at,std::size_t(pageCount)*4);c.at+=std::size_t(pageCount)*4;
 f->data.assign(bytes.begin()+std::ptrdiff_t(c.at),bytes.end());c.at=bytes.size();c.done();
 auto fbits=[](float v){std::uint32_t u;std::memcpy(&u,&v,4);return u;};
 for(int a=0;a<3;++a)need(f->page[a]==fbits(f->origin[a])&&f->page[4+a]==f->size[a]&&f->page[8+a]==f->pages[a],"render_field_header");
 need(f->page[3]==fbits(1.f/f->pitch)&&f->page[7]==TF::brickShift&&f->page[11]==TF::dataWidth&&f->page[12]==TF::pageWidth&&f->page[13]==TF::headerWords,"render_field_header");
 for(std::uint64_t b=0;b<bricks;++b){const std::uint32_t v=f->page[std::size_t(b)+TF::headerWords];
  if(v&TF::uniformBit){need((v&~(TF::uniformBit|63u))==0,"render_field_uniform");continue;}
  const unsigned bits=1u<<((v>>28)&3u),palette=bits==8?0:1u<<bits;
  need(std::uint64_t(v&0x0fffffffu)+palette+512u*bits/8<=dataCount,"render_field_brick");}
 f->available=true;return f;}
#include "GearCells.h" // cutfix: the outfit's gear cells (ownership of every exterior pixel at a separation)
#include "CrevicePadding.h" // crevice: the clothing padding is built behind the outfit's own triangles, none in front of them
// gearbody (GearBody.h): the outfit's gear surfaces join the body source at preparation (defined after the separation decoders)
class Asset;
struct GearBodyState {bool deferred{};std::uint32_t gear_faces{},body_vertices{},body_faces{};std::size_t merged_faces{},kept_faces{};
 std::vector<std::uint32_t> vertices; // the gear vertex of each merged source vertex, in source order
 std::vector<std::uint16_t> merged; // merged gear children (sorted)
 std::string note;std::shared_ptr<painter_detail::TissueField> baked;};
GearBodyState gearBodyStart(const AssetInput&input);
void gearBodyMerge(Asset&asset,const AssetInput&input);
std::vector<std::uint8_t> gearBodyBindings(const Asset&asset,std::vector<std::uint8_t> body,const std::vector<std::uint8_t>&gear);
std::vector<std::uint8_t> gearBodyCoatRoots(const Asset&asset,std::vector<std::uint8_t> roots);
void gearBodyTrace(const Asset&asset,const Impact&event,const Mapping&mapping,PoseCache&cache,ImpactResult&result);
template<class Frames> void gearBodyFlesh(const Asset&asset,SeparationPiece&piece,const Frames&frames);
class Asset {
public:
 bool record_resolved_cuts{},allow_renderer_rebuild{},analytic_channels{};
 std::shared_ptr<const painter_detail::TissueField> tissue_field;
 std::shared_ptr<const painter_detail::TissueField> closure_field; // render-only (ClosureField.h); analytic walls only
 std::shared_ptr<const gear_cells::GearCells> gear_cells; // cutfix: a band of cells around every exterior triangle (GearCells.h), full-body assets
 GearBodyState gear_body; // gearbody: before `source` - when deferred, the source is read in the body and its tree built after the merge
 Source source,envelope,occlusion;Grid base;core::BodyBVH source_tree,envelope_tree;Hash source_hash,rig_hash;std::uint16_t source_child,original_children;
 std::vector<std::uint16_t> face_children;
 std::vector<std::uint32_t> face_indices;
 // Hash-admitted original excluded surfaces. This immutable source belongs to
 // the asset; decoded ocular provenance is shared by all owner snapshots.
 std::vector<std::uint8_t> accessory_surface_bytes;
 mutable std::mutex ocular_mutex;
 mutable std::shared_ptr<const OcularAccessoryData> ocular_accessories;
 std::vector<JointMeasure> joints;
 CranialMeasure cranial_measure;
 std::vector<SeverSegment> sever_segments; // damage-threshold severing stations (SeverCheck.h), from the same joint profile
 std::shared_ptr<const std::unordered_map<Cell,SeverState::TileMaterial,core::CellHash>> tile_material; // SeverState::table
 std::shared_ptr<StationMasks> station_masks; // SeverState::masks (SeverCheck.h): station-slab masks shared by the model's bodies
 std::shared_ptr<const fastwound::AssetData> fast; // Incremental wound assembly data; built during async preparation.
 std::shared_ptr<const core::WeldBase> weld_base; // the source's render-weld set/index, built once (every region join reuses it)
 // Chunk seams: position-only skin weights (top 4, normalized) by exact point, shared by every job of the asset.
 struct SeamWeights {std::mutex mutex;std::unordered_map<core::Quant,std::array<std::pair<std::uint16_t,float>,4>,core::QuantHash> map;};
 mutable SeamWeights seam_weights;
 // Wound walls: exact nearest-surface skin (top 6) at the nodes of a fixed 2 cm lattice, computed on first use;
 // a wall vertex interpolates its cell's 8 nodes (FastWound_impl.h wallWeights).
 struct SkinNodes {std::mutex mutex;std::unordered_map<std::uint64_t,std::array<std::pair<std::uint16_t,float>,6>> map;};
 mutable SkinNodes skin_nodes;
 mutable std::mutex limb_mutex;mutable std::shared_ptr<const fastwound::LimbRegions> limb_regions;
 std::shared_ptr<const fastwound::LimbRegions> limbRegions()const{std::lock_guard<std::mutex> lock(limb_mutex);return limb_regions;}
 // Depth below the skin (distance to the closed envelope) at the nodes of a fixed 1 cm lattice, on first use
 // (LowPolyDisplay.h DepthField: the wall's depth tiers).
 struct DepthNodes {std::mutex mutex;std::unordered_map<std::uint64_t,float> map;};
 mutable DepthNodes depth_nodes;
 // Wound walls: closed-envelope faces meeting each 8^3 tile box (FastWound_impl.h buildDirect), on first use.
 struct EnvelopeTiles {std::mutex mutex;std::unordered_map<std::uint64_t,std::shared_ptr<const std::vector<std::uint32_t>>> map;
  std::unordered_map<std::uint64_t,signed char> sides;}; // tiles no envelope face touches: 1 inside, 0 outside (one inside() per tile per asset)
 mutable EnvelopeTiles envelope_tiles;
 explicit Asset(const AssetInput&input):record_resolved_cuts(input.record_resolved_cuts||input.analytic_channels),allow_renderer_rebuild(input.allow_renderer_rebuild),analytic_channels(input.analytic_channels),gear_body(gearBodyStart(input)),source(gear_body.deferred?Source{}:sourceBytes(input.source_geometry)),envelope(sourceBytes(input.closed_envelope)),occlusion(sourceBytes(input.occlusion_geometry,true)),base(gridBytes(input.base_occupancy)),source_tree(source),envelope_tree(envelope,!input.source_children.empty()),source_hash(input.source_hash),rig_hash(input.rig_hash),source_child(input.source_child),original_children(input.original_child_count){
  if(gear_body.deferred)source=sourceBytes(input.source_geometry); // gearbody: read here; the gear joins it below, then its tree is built
  need(source_child<original_children&&original_children<gfy_packed::max_child_slots,"source_child_profile");
  need(input.accessory_surfaces.empty()||!input.source_children.empty(),"accessory_surface_requires_fullbody");
  accessory_surface_bytes=input.accessory_surfaces;
  need(source.bones==envelope.bones&&source.bones==occlusion.bones,"envelope_rig");
  Cursor c{input.source_shading};c.magic("VXSHAD01");need(c.take<std::uint32_t>()==source.v.size(),"shading_count");
  for(auto&v:source.v){v.normal={c.take<float>(),c.take<float>(),c.take<float>()};v.tangent={c.take<float>(),c.take<float>(),c.take<float>()};v.binormal={c.take<float>(),c.take<float>(),c.take<float>()};}c.done();
  if(!input.source_children.empty()){
   Cursor map{input.source_children};map.magic("VXCHLD01");need(map.take<std::uint32_t>()==source.f.size(),"source_child_map_count");
   face_children.reserve(source.f.size());face_indices.reserve(source.f.size());
   for(std::size_t n=0;n<source.f.size();++n){auto child=map.take<std::uint16_t>();auto face=map.take<std::uint32_t>();
    // Provenance indexes the authored child, including faces classified as gear.
    need(child<original_children&&face<source.f.size()+occlusion.f.size(),"source_child_map_range");face_children.push_back(child);face_indices.push_back(face);}
   map.done();
  }
  if(!input.material_slots.empty()){
   need(bool(base.anatomy),"material_table_requires_anatomy");Cursor table{input.material_slots};
   // VXMATS02 = VXMATS01 + one class byte per label + the slots of the five anatomy-fill labels (AnatomyFill.h).
   const bool classed=input.material_slots.size()>=8&&!std::memcmp(input.material_slots.data(),"VXMATS02",8);
   table.magic(classed?"VXMATS02":"VXMATS01");auto count=table.take<std::uint32_t>();need(count>=2&&count<=65536-5,"material_table_count");
   auto slots=std::make_shared<std::vector<std::uint16_t>>();slots->reserve(count+5);
   for(unsigned n=0;n<count;++n){auto slot=table.take<std::uint16_t>();need(slot<64&&(n!=0||slot==0),"material_slot_range");slots->push_back(slot);}
   if(classed){
    std::vector<std::uint8_t> classes(count);for(auto&k:classes)k=table.take<std::uint8_t>();
    std::uint16_t fillSlot[5];for(auto&s:fillSlot){s=table.take<std::uint16_t>();need(s>0&&s<64,"material_fill_slot_range");}table.done();
    const core::anatomy_fill::Labels labels{std::uint16_t(count),std::uint16_t(count+1),std::uint16_t(count+2),std::uint16_t(count+3),std::uint16_t(count+4)};
    for(auto s:fillSlot)slots->push_back(s);
#ifdef GFY_VOXEL_STAGE_PROFILE
    const auto fillStart=core::Clock::now();const std::size_t runsBefore=base.anatomy->runs.size();
#endif
    base.anatomy=core::anatomy_fill::fill(*base.anatomy,*slots,classes,labels,[](std::size_t n,auto&&body){
     core::parallelChunks(n,std::max<std::size_t>(1,n/(core::parallelWidth()*4)),[&](std::size_t first,std::size_t last,std::size_t){body(first,last);});});
#ifdef GFY_VOXEL_STAGE_PROFILE
    std::cout<<"voxel_stage asset.anatomy_fill "<<core::ms(fillStart)<<" runs "<<runsBefore<<" -> "<<base.anatomy->runs.size()<<" inline="<<core::painterInlineParallel<<std::endl;
#endif
   }else table.done();
   // Stable atlas IDs may have holes for excluded, invalid source structures.
   // Those unused IDs may remain zero; every actual occupied label must map.
   for(const auto&run:base.anatomy->runs)need(run.label<slots->size()&&(*slots)[run.label]>0,"anatomy_label_missing_material");base.material_slots=std::move(slots);
  }
  if(base.anatomy&&base.material_slots){
   try{tissue_field=painter_detail::buildTissueField(*base.anatomy,*base.material_slots);}catch(const std::bad_alloc&){if(analytic_channels&&!face_children.empty())throw;tissue_field.reset();}
  }
  if(gear_body.deferred)gearBodyMerge(*this,input); // gearbody: the gear rows join the source (eyes and out-of-reach gear stay), tree built
  if(input.damage_pitch){
   need(base.anatomy&&!face_children.empty(),"damage_pitch_requires_fullbody_anatomy");
   need(input.damage_pitch==.002||input.damage_pitch==.001||input.damage_pitch==.0005,"damage_pitch_quality");
   const double ratio=base.pitch/input.damage_pitch;
   for(int axis=0;axis<3;++axis){const double cells=std::ceil(base.size[axis]*ratio);
    need(cells>0&&cells<65536,"damage_pitch_extent");(&base.size.x)[axis]=int(cells);}
   // Anatomical labels remain in their original immutable 1 mm field.
   // Only wound/removal cells change density; finer cuts do not invent new
   // organ detail or allocate an eight-times-larger whole-body volume.
   base.pitch=input.damage_pitch;
  }
  if(base.anatomy&&!face_children.empty()){
   // The damage mask must reach loose clothes outside the anatomical volume.
   // Extend only its sparse coordinate domain, aligned to the same lattice.
   // materialBase still samples the immutable anatomy in world coordinates:
   // added collar space is air and never counts as removed body material.
   const auto bounds=source_tree.nodes.front().bounds;Vec origin=base.origin;
   for(int a=0;a<3;++a){const int low=std::min(0,int(std::floor((bounds.lo[a]-origin[a])/base.pitch))-2);
    const int high=std::max(base.size[a],int(std::ceil((bounds.hi[a]-origin[a])/base.pitch))+2);
    need(high>low&&high-low<65536,"fullbody_mask_extent");base.origin[a]=origin[a]+low*base.pitch;(&base.size.x)[a]=high-low;}
  }
  if(!face_children.empty())try{gear_cells=gear_cells::build(source,base);}catch(const std::bad_alloc&){gear_cells.reset();} // cutfix
  joints=prepareJointMeasures(base,input.joints,source.bones);
  cranial_measure=prepareCranialMeasure(base);
  sever_segments=prepareSeverSegments(base,input.joints,source.bones);
  if(!sever_segments.empty())try{tile_material=buildTileMaterial(base);}catch(const std::bad_alloc&){tile_material.reset();}
  if(!sever_segments.empty())try{station_masks=std::make_shared<StationMasks>(base.pitch*base.pitch*base.pitch);}catch(const std::bad_alloc&){station_masks.reset();}
  if(!face_children.empty()){weld_base=core::buildWeldBase(source);fast=fastwound::buildAssetData(source,face_children,face_indices,original_children);
   // Warm the per-envelope caches off the hit path (same skin fingerprint and
   // weld tolerance core::join uses for the full-body patch).
   core::envelopeSeedField(source,envelope_tree);core::envelopeClosureIndex(source,envelope_tree,3e-7);
  }
  if(analytic_channels&&!face_children.empty()){
   // Anatomy-driven cut surfaces require the supplied offline-baked render field.
   // Missing, invalid or misaligned data must not be replaced with generic flesh.
   need(tissue_field&&tissue_field->available,"anatomy_cut_tissue_field_required");
   need(!input.render_field.empty(),"anatomy_cut_baked_render_field_required");
   auto baked=gear_body.baked?std::move(gear_body.baked):renderFieldBytes(input.render_field); /* gearbody: read once by the merge */const auto&sb=source_tree.nodes.front().bounds;
    bool fits=std::abs(double(baked->pitch)-double(tissue_field->pitch))<1e-9;
    for(int a=0;a<3&&fits;++a){const double p=double(baked->pitch),o=double(baked->origin[a]),e=o+double(baked->size[a])*p;
     const double shift=(double(tissue_field->origin[a])-o)/p;
     fits=std::abs(shift-std::round(shift))<1e-3&&std::lround(shift)%8==0&&shift>-.5&&
      e>=double(tissue_field->origin[a])+double(tissue_field->size[a])*double(tissue_field->pitch)-1e-6&&o<=sb.lo[a]&&e>=sb.hi[a];}
    need(fits,"anatomy_cut_baked_render_field_grid_or_coverage");
    if(fits)closure_field=std::move(baked);
  }
  // R72 (2026-10-22): the closure marks its clothing padding (label bit 64 where the anatomy has no tissue) so the wall shader
  // tells padding from flesh (floating wound channels); on failure the unmarked closure stays (walls as before).
  if(closure_field&&closure_field->available&&tissue_field&&tissue_field->available){
   try{auto parallel=[](std::size_t n,const std::function<void(std::size_t,std::size_t)>&body){
     core::parallelChunks(n,std::max<std::size_t>(64,n/(core::parallelWidth()*16)),[&](std::size_t first,std::size_t last,std::size_t){body(first,last);});};
    std::shared_ptr<const crevice_padding::Plan> cplan;
    const std::size_t gearOutside=gear_body.gear_faces>gear_body.merged_faces?std::size_t(gear_body.gear_faces)-gear_body.merged_faces:0;
    if(gearOutside>2000)std::cout<<"voxel_stage asset.crevice_padding skipped gear_triangles_outside_source="<<gearOutside<<std::endl;
    else if(gear_cells&&!std::getenv("GFY_CREVICE_OFF"))try{cplan=crevice_padding::plan(*closure_field,*tissue_field,source,base);}catch(const std::bad_alloc&){cplan.reset();}
    if(cplan)std::cout<<"voxel_stage asset.crevice_padding "<<cplan->ms<<" envelope_cells="<<cplan->padding<<" votes="<<cplan->votesOut<<"/"<<cplan->votesIn<<" against="<<cplan->against
     <<" backing_cells="<<cplan->kept<<" no_padding_cells="<<cplan->cleared<<" with_sheets="<<cplan->dropped<<std::endl;
    auto marked=cplan&&cplan->dropped?painter_detail::markClosurePadding(*closure_field,*tissue_field,parallel,crevice_padding::Drop{cplan.get(),closure_field.get()},std::size_t(128u<<20))
     :painter_detail::markClosurePadding(*closure_field,*tissue_field,parallel);if(marked&&marked->available)closure_field=std::move(marked);}
   catch(const std::bad_alloc&){}
  }
  // cutfix: warm the tiles that hold clothing padding (GearCells.h padTiles: read by every separation of this outfit)
  if(gear_cells)try{const auto&f=closure_field?closure_field:tissue_field;if(f&&f->available)(void)gear_cells::padTileFlags(f,base);}catch(const std::bad_alloc&){}
 }
};
std::size_t ocularAccessoryBytes(const Asset&asset)noexcept;
std::string clipOcularAccessories(const Asset&asset,const Grid&grid,const gfy_packed::Mesh*published,gfy_packed::Mesh&mesh,bool owned,
                                  const std::set<Cell>*dirty); // ownersafe: what it skipped; dirty = the tiles this job changed
const char* last_failure()noexcept{return failure;}
// ---- OOM safety: policy installation, memory query, estimates ----
void install_thread_allocation_policy(ThreadAllocationPolicy policy)noexcept{
 allocationPolicy.store(policy,std::memory_order_release);
 core::threadEntryHook.store(policy?&painterThreadEntry:nullptr,std::memory_order_release);
}
void install_memory_query(MemoryQuery query)noexcept{memoryQueryHook.store(query,std::memory_order_release);}
MemoryStatus query_memory()noexcept{
 // Polled by admission loops and every dispatch: one system query per 5 ms.
 static std::mutex cacheMutex;static MemoryStatus cached;static core::Clock::time_point at;static MemoryQuery from=nullptr;
 const auto query=memoryQueryHook.load(std::memory_order_acquire);if(!query)return {};
 std::lock_guard<std::mutex>lock(cacheMutex);
 if(from!=query||!cached.valid||core::ms(at)>=5){cached=query();at=core::Clock::now();from=query;}
 return cached;
}
void set_memory_policy(const MemoryPolicy&policy)noexcept{std::lock_guard<std::mutex>lock(memoryPolicyMutex);memoryPolicyValue=policy;}
MemoryPolicy memory_policy()noexcept{std::lock_guard<std::mutex>lock(memoryPolicyMutex);return memoryPolicyValue;}
MemoryCounters memory_counters()noexcept{auto&c=memoryCounters;MemoryCounters o;auto r=[](const std::atomic<std::uint64_t>&a){return a.load(std::memory_order_relaxed);};
 o.allocation_failures=r(c.allocation_failures);o.oom_deferred=r(c.oom_deferred);o.oom_failed=r(c.oom_failed);o.admission_deferred=r(c.admission_deferred);o.admission_failed=r(c.admission_failed);
 o.heavy_joins=r(c.heavy_joins);o.heavy_waits=r(c.heavy_waits);o.phys_waits=r(c.phys_waits);o.dispatch_waits=r(c.dispatch_waits);o.separation_refused=r(c.separation_refused);
 o.separation_oom=r(c.separation_oom);o.piece_oom=r(c.piece_oom);o.preparation_oom=r(c.preparation_oom);o.evicted_sessions=r(c.evicted_sessions);o.evicted_bytes=r(c.evicted_bytes);o.max_join_estimate=r(c.max_join_estimate);return o;}
std::uint64_t join_estimate_bytes(std::size_t quads,std::size_t faces,std::size_t replacedTriangles,std::size_t bodyTriangles)noexcept{
 // Calibrated as an upper bound of MEASURED per-job peak transient bytes on
 // 1 mm heavy-corpse replays (build2-fixes/oom-safety REPORT, calibration.txt).
 return std::uint64_t(quads)*GFY_JOIN_BYTES_PER_QUAD+std::uint64_t(faces)*GFY_JOIN_BYTES_PER_FACE+std::uint64_t(replacedTriangles)*GFY_JOIN_BYTES_PER_REPLACED_TRIANGLE+
  std::uint64_t(bodyTriangles)*GFY_JOIN_BYTES_PER_BODY_TRIANGLE+GFY_JOIN_BYTES_FIXED;
}
std::size_t asset_resident_bytes(const std::shared_ptr<const Asset>&asset)noexcept{
 if(!asset)return 0;const auto&a=*asset;std::size_t bytes=sizeof(Asset);
 auto source=[](const Source&s){return s.v.capacity()*sizeof(core::SourceVertex)+s.f.capacity()*sizeof(s.f[0]);};
 auto tree=[](const core::BodyBVH&t){std::size_t b=t.order.capacity()*4+t.nodes.capacity()*sizeof(core::BodyBVH::Node);
  if(t.rayGrid)b+=(t.rayGrid->first.capacity()+t.rayGrid->items.capacity())*4+(t.rayGrid->leafOf.capacity()+t.rayGrid->parent.capacity())*sizeof(int);
  {std::lock_guard<std::mutex>lock(*t.seedMutex);
   if(t.seedField)b+=t.seedField->first.capacity()*4+t.seedField->entries.capacity()*sizeof(t.seedField->entries[0])+t.seedField->valid.capacity();
   if(t.closureIndex)b+=t.closureIndex->rims.size()*(sizeof(std::array<core::Quant,3>)+64)+t.closureIndex->rims.bucket_count()*8;}
  return b;};
 bytes+=source(a.source)+source(a.envelope)+source(a.occlusion)+tree(a.source_tree)+tree(a.envelope_tree);
 if(a.tissue_field)bytes+=sizeof(painter_detail::TissueField)+a.tissue_field->page.capacity()*4+a.tissue_field->data.capacity();
 if(a.closure_field)bytes+=sizeof(painter_detail::TissueField)+a.closure_field->page.capacity()*4+a.closure_field->data.capacity();
 if(a.gear_cells)bytes+=a.gear_cells->bytes; // cutfix
 bytes+=a.gear_body.vertices.capacity()*4; // gearbody
 if(a.base.base)bytes+=a.base.base->capacity();
 if(a.base.anatomy)bytes+=a.base.anatomy->offsets.capacity()*4+a.base.anatomy->runs.capacity()*sizeof(a.base.anatomy->runs[0]);
 if(a.base.material_slots)bytes+=a.base.material_slots->capacity()*2;
 bytes+=a.face_children.capacity()*2+a.face_indices.capacity()*4+a.joints.capacity()*sizeof(a.joints[0]);
 bytes+=a.accessory_surface_bytes.capacity()+ocularAccessoryBytes(a)+a.cranial_measure.bytes();
 if(a.fast){const auto&d=*a.fast;bytes+=d.rows.capacity()*sizeof(gfy_packed::Vertex)+(d.hashes.capacity()+d.discarded.capacity())*8+d.corner.capacity()*4+d.clean.capacity()+
  (d.ringFirst.capacity()+d.ringFaces.capacity()+d.weightFirst.capacity()+d.edgeNeighbourFirst.capacity()+d.edgeNeighbours.capacity())*4+d.weightEntries.capacity()*16+
  d.edgeBad.capacity()+d.positionConsistent.capacity()+d.positionIndex.size()*40;
  for(const auto&faces:d.child_faces)bytes+=faces.capacity()*4+sizeof(faces);
  for(const auto&s:d.static_child)for(const auto&ch:s.chunks)bytes+=ch.vertices.capacity()*sizeof(gfy_packed::Vertex)+ch.indices.capacity()*2+ch.original_faces.capacity()*4;}
 return bytes;
}
namespace {
core::JoinResult joinFullBody(const Asset&asset,const Grid&grid,const std::vector<core::Quad>&boundary,WorkerPhaseTimes&times,const core::ExteriorMemo*reuse=nullptr,core::ExteriorMemo*produce=nullptr){
 PhaseTime preparationTime(times.join_ms);
 if(grid.ownership&&!grid.ownership->source_is_piece_local)
  throw RecoverableImpactError("piece_join_requires_piece_local_source");
 // Only geometry next to an existing removal mask participates in wound
 // construction. Unrelated clothes/accessories can contain authored seams,
 // touching layers or duplicate faces; preserve them verbatim instead of
 // globally welding/revalidating a model on every bullet.
 core::StageProfile stages("fullbody");std::set<std::uint32_t> selected;
 for(const auto&tile:grid.tiles){auto c=tile.first;core::Bounds box;
  box.lo=grid.origin+Vec{double(c.x*8-2),double(c.y*8-2),double(c.z*8-2)}*grid.pitch;
  box.hi=grid.origin+Vec{double(c.x*8+10),double(c.y*8+10),double(c.z*8+10)}*grid.pitch;
  std::vector<std::uint32_t> faces;asset.source_tree.query(0,box,faces);selected.insert(faces.begin(),faces.end());
 }
 need(!selected.empty(),"fullbody_cut_without_exterior_patch");
 Source patch;patch.bones=asset.source.bones;patch.v=asset.source.v;
 std::vector<std::uint32_t> originals(selected.begin(),selected.end());patch.f.reserve(originals.size());
 for(auto face:originals)patch.f.push_back(asset.source.f[face]);
 stages.mark("select_patch");core::BodyBVH tree(patch);preparationTime.finish();core::ClipResult outside;
 {PhaseTime exteriorTime(times.exterior_ms);outside=core::exterior(patch,grid,reuse,&originals,produce);}stages.mark("exterior");
 PhaseTime joinTime(times.join_ms);auto joined=core::join(patch,tree,asset.envelope_tree,grid,outside,boundary);stages.mark("join");
 for(auto&t:joined.triangles)if(!t.material)for(auto&v:t.v){need(v.sourceFace<originals.size(),"fullbody_patch_provenance");v.sourceFace=originals[v.sourceFace];}
 for(std::uint32_t f=0;f<asset.source.f.size();++f)if(!selected.count(f)){
  core::JoinedTriangle t;for(int k=0;k<3;++k){core::V v;v.p=asset.source.v[asset.source.f[f][k]].p;v.b[k]=1;
   t.v[k]=core::interpolateVertex(asset.source,f,v);}joined.triangles.push_back(std::move(t));
 }
 stages.mark("untouched_triangles");return joined;
}
core::JoinResult joinFullBody(const Asset&asset,const Grid&grid,const std::vector<core::Quad>&boundary){
 WorkerPhaseTimes times;return joinFullBody(asset,grid,boundary,times);
}
}
#include "FastWound_impl.h"
#include "ChannelWorkerIncludes.h"
std::shared_ptr<const Asset>prepare_asset(const AssetInput&input)noexcept{AllocationScope scope;try{return std::make_shared<const Asset>(input);}catch(const std::bad_alloc&){memoryCounters.preparation_oom.fetch_add(1,std::memory_order_relaxed);fail(allocationFailureText());return {};}catch(const std::exception&e){fail(e.what());return {};}catch(...){fail("unknown_asset_failure");return {};}}
struct AssetPreparation::Impl {mutable std::mutex mutex;std::shared_ptr<const Asset> asset;std::string error;bool ready=false;std::atomic<bool> promoted{false};};
AssetPreparation::AssetPreparation(std::shared_ptr<Impl>state):impl_(std::move(state)){}
void AssetPreparation::promote()const noexcept{impl_->promoted.store(true,std::memory_order_relaxed);}
bool AssetPreparation::poll(std::shared_ptr<const Asset>&asset,std::string&error)const {
 std::lock_guard<std::mutex>lock(impl_->mutex);if(!impl_->ready)return false;asset=impl_->asset;error=impl_->error;return true;
}
std::shared_ptr<AssetPreparation>prepare_asset_async(const AssetInput& input,AssetLoader loader,bool priority)noexcept{AllocationScope scope;try{
 // One bounded preparation worker. All file/rig snapshots are already owned
 // CPU bytes; decompression and BVH construction never touch live NPCs or FS.
 struct Service {
  struct Job {AssetInput input;AssetLoader loader;std::shared_ptr<AssetPreparation::Impl> output;};
  std::mutex mutex;std::condition_variable wake;std::deque<Job> jobs;bool stop=false;std::thread worker;
  // The player's target (promoted, or submitted with priority) runs next; otherwise first come, first served.
  Service():worker([this]{core::enterPainterThread();for(;;){Job job;{std::unique_lock<std::mutex>lock(mutex);wake.wait(lock,[this]{return stop||!jobs.empty();});if(stop)return;
    auto next=std::find_if(jobs.begin(),jobs.end(),[](const Job&j){return j.output->promoted.load(std::memory_order_relaxed);});
    if(next==jobs.end())next=jobs.begin();job=std::move(*next);jobs.erase(next);}
   painterThreadEntry(); // the engine may install its policy after this thread started
   std::shared_ptr<const Asset>asset;std::string error;
   try{if(job.loader)job.loader(job.input);asset=prepare_asset(job.input);if(!asset)error=last_failure();}
   catch(const std::bad_alloc&){error=allocationFailureText();}catch(const std::exception&e){error=e.what();}catch(...){error="unknown_asset_file_failure";}
   {std::lock_guard<std::mutex>lock(job.output->mutex);job.output->asset=std::move(asset);job.output->error=std::move(error);job.output->ready=true;}
  }}){}
  void quit()noexcept{{std::lock_guard<std::mutex>lock(mutex);stop=true;jobs.clear();}wake.notify_all();if(worker.joinable())worker.join();} // quitfix: queued preparations are dropped, the build in flight ends
  ~Service(){quit();}
 };
 static Service& service=::gfy_live::core::worker_quit::immortal<Service>(0);auto state=std::make_shared<AssetPreparation::Impl>();state->promoted.store(priority,std::memory_order_relaxed);
 auto handle=std::shared_ptr<AssetPreparation>(new AssetPreparation(state));
 // Admission precedes the owned byte snapshot: a full queue must not copy
 // tens of megabytes every time another NPC asks to prepare its model.
 {std::lock_guard<std::mutex>lock(service.mutex);if(service.jobs.size()>=(priority?4u:2u)){fail("asset_preparation_queue_full");return {};}service.jobs.push_back({input,std::move(loader),state});}
 service.wake.notify_one();return handle;
}catch(const std::bad_alloc&){fail(outOfMemoryText);return {};}catch(const std::exception&e){fail(e.what());return {};}catch(...){fail("unknown_asset_preparation_failure");return {};}}
std::shared_ptr<AssetPreparation>prepare_asset_files_async(const AssetInput& metadata,const AssetFiles& files,AssetHashFunction hash,bool priority)noexcept{AllocationScope scope;try{
 need(hash!=nullptr,"asset_hash_function_missing");
 return prepare_asset_async(metadata,[files,hash](AssetInput&input){
  std::vector<std::uint8_t>* output[]={nullptr,&input.source_geometry,&input.closed_envelope,&input.base_occupancy,&input.source_shading,&input.occlusion_geometry,&input.source_children,&input.material_slots,&input.accessory_surfaces,&input.render_field};
  for(std::size_t i=0;i<files.size();++i){const auto&spec=files[i];
   if(i==8&&spec.path.empty()){input.accessory_surfaces.clear();continue;}
   if(i==9&&spec.path.empty()){input.render_field.clear();continue;}
   need(!spec.path.empty()&&spec.max_bytes>0&&spec.max_bytes<=64u*1024u*1024u,"asset_file_descriptor");
   std::ifstream stream(spec.path,std::ios::binary|std::ios::ate);need(bool(stream),"asset_file_open");
   const auto size=stream.tellg();need(size>0&&std::uint64_t(size)<=spec.max_bytes,"asset_file_size");
   std::vector<std::uint8_t>data(static_cast<std::size_t>(size));stream.seekg(0);
   stream.read(reinterpret_cast<char*>(data.data()),static_cast<std::streamsize>(data.size()));need(bool(stream),"asset_file_read");
   need(hash(data.data(),data.size())==spec.expected,"asset_file_sha256");
   if(output[i])*output[i]=std::move(data);
  }
 },priority);
}catch(const std::bad_alloc&){fail(outOfMemoryText);return {};}catch(const std::exception&e){fail(e.what());return {};}catch(...){fail("unknown_asset_file_admission_failure");return {};}}
#include "PieceSeed.h"
struct DamageSnapshot::Impl {
 GoreControls gore; // Last accepted cut; inherited by separation and repeat damage.
 std::shared_ptr<const Asset>asset;
 std::shared_ptr<const Grid>grid;
 std::shared_ptr<const ResolvedCutHistory>history;
 std::shared_ptr<const SeverState>sever; // stations torn up to this revision (inherited by separated pieces)
 std::shared_ptr<const PieceSeed>piece;std::shared_ptr<const gfy_packed::Mesh>piece_mesh; // a piece session's seed and birth mesh
 mutable std::shared_ptr<const Output>geometry; // Replaced only while private/unacknowledged by renderer rebuild.
 mutable Stamp stamp; // Set to the published generation once, before owner export.
};
DamageSnapshot::DamageSnapshot(std::shared_ptr<const Impl>state):impl_(std::move(state)){}
const Stamp& DamageSnapshot::stamp()const noexcept{return impl_->stamp;}
const gfy_packed::Mesh* DamageSnapshot::mesh()const noexcept{return impl_->geometry?&impl_->geometry->mesh:impl_->piece_mesh.get();}
double DamageSnapshot::damage_pitch()const noexcept{return impl_->grid->pitch;}
std::shared_ptr<const ResolvedCutHistory> DamageSnapshot::resolved_cut_history()const noexcept{return impl_->history;}
std::size_t DamageSnapshot::sparse_tiles()const noexcept{return impl_->grid->tiles.size();}
DamageSample DamageSnapshot::sample_rest(Point point)const noexcept{
 const auto&g=*impl_->grid;Vec p=vec(point);Cell c;
 for(int axis=0;axis<3;++axis){double value=(p[axis]-g.origin[axis])/g.pitch;
  if(!std::isfinite(value)||value<0||value>=g.size[axis])return {};
  (&c.x)[axis]=int(std::floor(value));}
 auto label=g.materialBase(c);return {label,std::uint16_t(label?g.renderMaterial(c):0),g.removed(c)};
}
struct Session::Impl {
 std::shared_ptr<const Asset>asset;Identity owner;fastwound::BoundaryCache boundary_cache;bool boundary_valid=false;
 PoseCache pose_cache; // posed source/occlusion vertices, kept while the bone palette is bit-identical (a corpse never re-skins)
 // Committed per-region wound results (immutable, shared) and a worker-only
 // tile->source-face selection memo (pure function of the asset and lattice).
 fastwound::RegionList region_cache,ready_regions;std::shared_ptr<fastwound::TileFaceMemo> tile_faces=std::make_shared<fastwound::TileFaceMemo>();
 std::shared_ptr<fastwound::TileRegionState> tile_state=std::make_shared<fastwound::TileRegionState>(); // worker-only lattice memo
 std::shared_ptr<SeverState> ready_sever;
 std::shared_ptr<SeverState> sever_state=std::make_shared<SeverState>(); // stations already torn (SeverCheck.h)
 std::shared_ptr<const PieceSeed> piece; // continuation of a separated piece (PieceSeed.h); null for a whole body
 std::shared_ptr<const gfy_packed::SkinOwnershipQuery> piece_ownership;
 std::shared_ptr<const gfy_packed::Mesh> piece_published; // the piece mesh its owner committed at birth
 Identity lineage; // owner of the cut history (the whole body's); `owner` stamps this session's publications
 // Worker-only piece caches (PieceSession_impl.h): wall verdicts per cut since birth, and other pieces' marks per tile.
 struct PieceWalls {std::uint64_t sequence{};unsigned role{};std::array<double,3> entry{};std::vector<signed char> verdict;};
 std::vector<PieceWalls> piece_walls;std::vector<std::pair<Cell,core::Tile>> piece_marks;std::set<Cell> piece_mark_tiles;
 std::vector<std::array<double,6>> spare_static_boxes; // static chunk vertex boxes (worker-only, pure function of the asset)
 std::vector<std::array<int,6>> spare_joint_tiles; // per joint, the tile box its evaluate() reads (worker-only)
 // grid recycling. A published grid returns to `spare_spare` when its last reference drops (custom deleter);
 // the next job turns it into the committed grid by copying only the tiles the committed revisions after it may have
 // changed (their dirty sets, `spare_deltas`, appended at acknowledge) instead of copying the whole grid.
 struct SpareGrid {std::mutex mutex;std::unique_ptr<Grid> grid;std::uint64_t serial{};};
 std::shared_ptr<SpareGrid> spare_spare=std::make_shared<SpareGrid>();
 std::uint64_t spare_grid_serial{},spare_committed_serial{},spare_ready_serial{}; // serial 0: untracked grid
 std::shared_ptr<const std::vector<Cell>> spare_ready_dirty;
 std::deque<std::pair<std::uint64_t,std::shared_ptr<const std::vector<Cell>>>> spare_deltas; // under mutex
 std::shared_ptr<const DamageSnapshot>committed,ready_snapshot;
 std::shared_ptr<const channelworker::State>channel_cache,ready_channel_cache;
 std::atomic<bool> analytic_disabled{false}; // sticky per owner after a renderer refusal
 bool mask_only_off=false; // a sever happened on this body: mask-only wounds end for it (worker thread only)
 // melee (value 3): the dents this job publishes - the committed mesh's and this batch's blunt blows (worker thread only)
 std::vector<gfy_packed::Dent> job_dents;std::vector<gfy_packed::Slit> job_slits;bool job_dents_on=false; // + its blade slits
 std::uint64_t melee_severed[8]{};unsigned melee_severed_at=0; // melee: the last blows that cut a part through (their seeds)
 // melee (round 3): the blade cuts on this body / piece (the chop splash fields; worker thread only; the last 32)
 struct MeleeCut {std::uint32_t id{},blows{};Vec centre{},normal{},edge{};double half{},depth{},fraction{-1};std::uint64_t last_sequence{~std::uint64_t(0)};};
 std::vector<MeleeCut> melee_cuts;std::uint32_t melee_cut_next=1;
 // pending: fresh events (<=16), dispatched as one normal batch.
 // retry_pending: isolated suffix (<=15) of a failed normal batch. Set only by
 // the worker publishing that batch (while it is empty), popped one event per
 // dispatch ahead of pending, cleared by cancel and by fatal acknowledgement.
 mutable std::mutex mutex;std::deque<Impact>pending;std::vector<Impact>retry_pending;std::shared_ptr<const Output>done;
 // Worker output is provisional until the renderer owner accepts publication.
 std::shared_ptr<const Grid>ready_grid;fastwound::BoundaryCache ready_boundary;
 // Cross-hit exterior clip memo. Content-validated against its own basis grid
 // (see core::ExteriorMemo), committed with the accepted candidate like
 // boundary_cache and never shared with a provisional output.
 std::shared_ptr<const core::ExteriorMemo>exterior_memo,ready_exterior_memo;
 RendererRebuildState renderer_rebuild=RendererRebuildState::none;
 const char* renderer_rebuild_reason="none";
 bool canceled{},inflight{},failed{};std::uint64_t submitted{},acknowledged{};
 body_hit_outcomes::WaitMemo hit_wait;
 std::function<void()>ready_hook; // owner wake-up after a job (Session::set_ready_hook): copied under mutex, run outside it
 // Memory admission (under mutex). need_commit: available commit the deferred
 // batch waits for before dispatch (bounded by MemoryPolicy::max_wait_ms).
 // memory_deferred: first sequence of the batch that already used its one
 // memory deferral; a second shortage reports that batch not applied.
 std::uint64_t need_commit{},deferrals{},memory_failures{},memory_deferred{};std::uint32_t batch_deferrals{};
 core::Clock::time_point last_activity=core::Clock::now(),memory_wait_since{};bool memory_waiting{};
 // The governor dropped boundary_cache; the next job rebuilds it from the
 // committed occupancy first (identical content, see rebuildBoundaryCache).
 bool boundary_rebuild{};
 Impl(std::shared_ptr<const PieceSeed>seed,Identity id,std::shared_ptr<const gfy_packed::Mesh>published,std::uint64_t revision,std::uint64_t generation)
  :asset(seed->asset),owner(id),piece(seed),piece_published(std::move(published)){
  piece_ownership=gfy_packed::skin_ownership_query(*piece_published,true);
  lineage=seed->history->owner;submitted=acknowledged=revision;
  if(seed->sever)sever_state=std::make_shared<SeverState>(*seed->sever);
  sever_state->table=asset->tile_material.get();sever_state->masks=asset->station_masks.get();
  {std::uint32_t held=0;for(const auto&t:piece_published->tears)held+=t.block&&t.side<=2?1u:0u;sever_state->decided=held;} // slice 4: the tears this piece holds count toward its 8
  auto state=std::make_shared<DamageSnapshot::Impl>();state->asset=asset;state->grid=seed->grid;state->history=seed->history;state->sever=seed->sever;
  state->piece=piece;state->piece_mesh=piece_published;state->gore=seed->gore;state->stamp.owner=owner;state->stamp.revision=revision;state->stamp.expected_generation=generation;
  committed=std::shared_ptr<const DamageSnapshot>(new DamageSnapshot(std::move(state)));
  // Cuts at birth are drawn by the birth mesh (piece_session::channelPrior).
  auto prior=std::make_shared<channelworker::State>();prior->asset_pin=asset;prior->history=seed->history;prior->tolerance=.009;prior->total_cuts=seed->history->total_cuts;
  channel_cache=std::move(prior);
 }
 Impl(std::shared_ptr<const Asset>a,Identity id):asset(std::move(a)),owner(id){
  lineage=owner;sever_state->table=asset->tile_material.get();sever_state->masks=asset->station_masks.get();
  auto state=std::make_shared<DamageSnapshot::Impl>();state->asset=asset;
  state->grid=std::shared_ptr<const Grid>(asset,&asset->base);state->stamp.owner=owner;
  if(asset->record_resolved_cuts){auto history=ResolvedCutHistory::create();history->owner=owner;
   const auto&g=asset->base;history->grid_origin={g.origin.x,g.origin.y,g.origin.z};history->grid_pitch=g.pitch;
   history->grid_size={unsigned(g.size.x),unsigned(g.size.y),unsigned(g.size.z)};
   if(g.ownership)history->limitations|=resolved_cut_owned_grid;
   state->history=std::move(history);}
  committed=std::shared_ptr<const DamageSnapshot>(new DamageSnapshot(std::move(state)));
 }
};
#include "PieceSession_impl.h"
Session::Session(std::unique_ptr<Impl>impl):impl_(std::move(impl)){}Session::~Session()=default;
bool Session::piece()const noexcept{return bool(impl_->piece);}
bool Session::enqueue(Impact event)noexcept{AllocationScope scope;
 const auto trace=event.hit_trace;const auto sequence=event.sequence;std::size_t pending_count=0;unsigned inflight_state=0;
 auto reject=[&](const char*why){body_hit_outcomes::report("not_queued",why,impl_->owner.spawn_cookie,sequence,trace,0,pending_count,inflight_state);return false;};
 try{auto&i=*impl_;std::lock_guard<std::mutex>lock(i.mutex);
 pending_count=i.pending.size();inflight_state=unsigned(i.inflight);
 if(const char*why=body_hit_outcomes::enqueue_reason(i.canceled,i.failed,i.pending.size(),event.sequence,i.submitted))return reject(why);
 if(!valid_gore_controls(event.gore))return reject("invalid_gore_controls");
 if(!std::isfinite(event.penetration_m)||event.penetration_m<0||event.penetration_m>.5)return reject("invalid_penetration");
 if(event.penetration_m>0){if(!(std::isfinite(event.radius)&&event.radius>=.0005&&event.radius<=.08))return reject("invalid_radius");}
 else if(!(event.radius>=i.asset->base.pitch&&event.radius<=.08&&event.depth>=0&&event.depth<=.08))return reject("invalid_legacy_radius_or_depth");
 i.pending.push_back(std::move(event));++i.submitted;i.last_activity=core::Clock::now();
 body_hit_outcomes::report("queued",i.done?"waiting_for_publication":i.inflight?"waiting_for_inflight":"pending_dispatch",i.owner.spawn_cookie,sequence,trace,0,i.pending.size(),unsigned(i.inflight));return true;
 }catch(const std::bad_alloc&){fail(outOfMemoryText);return reject(outOfMemoryText);}catch(const std::exception&e){fail(e.what());return reject(e.what());}catch(...){fail("unknown_enqueue_failure");return reject("unknown_enqueue_failure");}}
std::shared_ptr<const Output>Session::completed()const{std::lock_guard<std::mutex>lock(impl_->mutex);return impl_->renderer_rebuild==RendererRebuildState::queued?std::shared_ptr<const Output>{}:impl_->done;}
void Session::set_ready_hook(std::function<void()>hook)noexcept{try{auto&i=*impl_;std::lock_guard<std::mutex>lock(i.mutex);i.ready_hook=std::move(hook);}catch(...){}}
static std::atomic<int> mask_only_mode{3}; // console painter_voxel_mask_only (modes: default 3, the owner's setting; was 0)
void set_mask_only(bool on)noexcept{mask_only_mode.store(on?1:0,std::memory_order_relaxed);}
void set_mask_only_mode(int mode)noexcept{mask_only_mode.store(mode<0?0:(mode>3?3:mode),std::memory_order_relaxed);} // 3: slice 3
int mask_only_mode_value()noexcept{return mask_only_mode.load(std::memory_order_relaxed);}
static std::atomic<int> seam_rule{1};
void set_seam_rule(int on)noexcept{seam_rule.store(on?1:0,std::memory_order_relaxed);}
int seam_rule_value()noexcept{return seam_rule.load(std::memory_order_relaxed);}
static std::atomic<int> sever_method{1};
void set_sever_method(int m)noexcept{sever_method.store(m==0?0:1,std::memory_order_relaxed);}
int sever_method_value()noexcept{return sever_method.load(std::memory_order_relaxed);}
static std::atomic<int> holding_cut_on{0};
void set_holding_cut(int on)noexcept{holding_cut_on.store(on?1:0,std::memory_order_relaxed);}
int holding_cut_value()noexcept{return holding_cut_on.load(std::memory_order_relaxed);}
// raggedsever: console painter_kill_sever_ragged - 1 (default) a killing shot's limb tear runs after the batch's damage and through it
// (the event loop's end), 0 today's cut (at the killing event, the limb axis leaned 10-30 degrees with the shot); read at every killing
// shot. GFY_KILL_SEVER_RAGGED=0|1: the start value (offline A/B runs).
static std::atomic<int>& kill_sever_ragged_on()noexcept{static std::atomic<int> v{[]{const char*e=std::getenv("GFY_KILL_SEVER_RAGGED");return e?(std::atoi(e)!=0?1:0):1;}()};return v;}
void set_kill_sever_ragged(int on)noexcept{kill_sever_ragged_on().store(on?1:0,std::memory_order_relaxed);}
int kill_sever_ragged_value()noexcept{return kill_sever_ragged_on().load(std::memory_order_relaxed);}
static std::atomic<int>& kill_sever_blowout_on()noexcept{static std::atomic<int> v{[]{const char*e=std::getenv("GFY_KILL_SEVER_BLOWOUT");return e?(std::atoi(e)!=0?1:0):0;}()};return v;}
void set_kill_sever_blowout(int on)noexcept{kill_sever_blowout_on().store(on?1:0,std::memory_order_relaxed);}
int kill_sever_blowout_value()noexcept{return kill_sever_blowout_on().load(std::memory_order_relaxed);}
static std::atomic<int> island_fill_on{1};
void set_island_fill(int on)noexcept{island_fill_on.store(on?1:0,std::memory_order_relaxed);}
int island_fill_value()noexcept{return island_fill_on.load(std::memory_order_relaxed);}
// nothinghits: console painter_hole_reach - 1 (default) a bullet into a crater with flesh behind it wounds that flesh, 0 it does
// nothing (f14c); read at every hit that removed nothing
static std::atomic<int> hole_reach_on{1};
void set_hole_reach(int on)noexcept{hole_reach_on.store(on?1:0,std::memory_order_relaxed);}
int hole_reach_value()noexcept{return hole_reach_on.load(std::memory_order_relaxed);}
// headfix: console painter_body_burst_chance (the chance that a blast batch breaks the body with a body burst; else the legacy cuts)
static std::atomic<float> body_burst_chance{.35f};
void set_body_burst_chance(float chance)noexcept{body_burst_chance.store(chance>0.f?(chance<1.f?chance:1.f):0.f,std::memory_order_relaxed);}
float body_burst_chance_value()noexcept{return body_burst_chance.load(std::memory_order_relaxed);}
// the draw of one blast batch, u in [0, 1) (splitmix64 of its seed)
// modes (2026-11-13): console painter_damage_mode, the Painter menu's Damage mode (painter_deformation_mcm.script): 0 Normal
// (today), 1 Bullet holes only, 2 Extreme. The worker reads it once per job (dmode).
static std::atomic<int> damage_mode{0};
void set_damage_mode(int mode)noexcept{damage_mode.store(mode==1||mode==2?mode:0,std::memory_order_relaxed);}
int damage_mode_value()noexcept{return damage_mode.load(std::memory_order_relaxed);}
// bulletburst (2026-11-14): console painter_body_blowout_frequency 0..2 (default 1 = the head's chance table as it is), the Painter
// menu's Body blowout frequency: scales the bullet body burst / surface blowout chances (BulletBurst.h); read at every bullet hit
static std::atomic<float> body_blowout_frequency{1.f};
void set_body_blowout_frequency(float f)noexcept{body_blowout_frequency.store(f>0.f?(f<2.f?f:2.f):0.f,std::memory_order_relaxed);}
float body_blowout_frequency_value()noexcept{return body_blowout_frequency.load(std::memory_order_relaxed);}
static std::atomic<int> body_blowout_chunks{1};
void set_body_blowout_chunks(int v)noexcept{body_blowout_chunks.store(v?1:0,std::memory_order_relaxed);}
int body_blowout_chunks_value()noexcept{return body_blowout_chunks.load(std::memory_order_relaxed);}
namespace damage_modes {
constexpr double holeDepth=.020;
// ... except a part at most this thick along the shot that the round would cross: holed through as today (two 2 cm craters
// and a 5 mm plug do not fit in it anyway: hands, wrists, fingers)
constexpr double thinPart=.045;
// Extreme: every damage size doubled
constexpr double extremeSize=2.;
// every sever path of the worker: off in Bullet holes only, as GFY_NO_SEVER turns them off
inline bool noSever(int mode){return mode==1||std::getenv("GFY_NO_SEVER")!=nullptr;}
// Extreme: the sizes of one event's copy (the decisions read the event itself). A round's travel (a bullet's or fragment's
// penetration) is not a size; a crater's, a blade's and a knocked-out chunk's depth is. Clamped to the cut limits.
inline void doubleSizes(Impact&e){
 const bool sheet=e.slab&&e.penetration_m>0;
 e.radius=std::min(sheet?.05:.08,e.radius*extremeSize);
 if(e.slab_half_width>0)e.slab_half_width=std::min(.5,e.slab_half_width*extremeSize);
 if((e.blast||e.melee)&&e.penetration_m>0)e.penetration_m=std::min(.5,e.penetration_m*extremeSize);
 e.dent_radius=std::min(.15,e.dent_radius*extremeSize);e.dent_depth=std::min(.05,e.dent_depth*extremeSize);}
}
static double body_burst_draw(std::uint64_t x)noexcept{x+=0x9E3779B97F4A7C15ull;x=(x^(x>>30))*0xBF58476D1CE4E5B9ull;x=(x^(x>>27))*0x94D049BB133111EBull;x^=x>>31;
 return double(x>>11)*(1./9007199254740992.);}
void Session::disable_analytic_channels()noexcept{impl_->analytic_disabled.store(true,std::memory_order_relaxed);}
RendererRebuildStatus Session::renderer_rebuild_status()const noexcept {auto&i=*impl_;std::lock_guard<std::mutex>lock(i.mutex);return {i.renderer_rebuild,i.renderer_rebuild_reason};}
// Retirement thread (62a): state an acknowledgement replaces or drops is freed here, off the acknowledging thread.
namespace {
struct Retirement {
 std::mutex mutex;std::condition_variable wake;std::vector<std::shared_ptr<void>> queue;bool stop=false;std::thread worker;
 Retirement():worker([this]{for(;;){std::vector<std::shared_ptr<void>> batch;
  {std::unique_lock<std::mutex> lock(mutex);wake.wait(lock,[&]{return stop||!queue.empty();});if(queue.empty()&&stop)return;batch.swap(queue);}
  batch.clear();}}){}
 void quit()noexcept{{std::lock_guard<std::mutex> lock(mutex);stop=true;}wake.notify_one();if(worker.joinable())worker.join();} // quitfix
 ~Retirement(){quit();}
 void push(std::shared_ptr<void> value){if(!value)return;{std::lock_guard<std::mutex> lock(mutex);queue.push_back(std::move(value));}wake.notify_one();}
};
// ACK has already committed by this point. Failure to start/enqueue the
// optional retirement thread must not escape and make its caller retry a
// transaction the Session has already consumed. Under memory pressure the
// same owned state is freed synchronously instead; numerical output is intact.
void retire(std::shared_ptr<void> value)noexcept{
 try{static Retirement& retirement=::gfy_live::core::worker_quit::immortal<Retirement>(1);retirement.push(std::move(value));}catch(...){} // quitfix: never destroyed
}
}
// Cancel publication before releasing the owner reference. Any in-flight Work
// still owns the Session until its original worker exits. The normal last
// release occurs off the owner thread; retirement failure safely frees inline.
void retire_session(std::shared_ptr<Session> session)noexcept{
 if(session)session->cancel();retire(std::move(session));
}
// speedfix (LiveSeparationWorker_impl.h): the fits of a whole body's damaged elements made ahead after a hit
bool prefit_after_commit(const std::shared_ptr<const Asset>&asset,const std::shared_ptr<const Grid>&grid,const Stamp&stamp)noexcept;
bool Session::acknowledge(const Stamp&stamp,bool accepted,std::uint64_t generation){return acknowledge_impl(stamp,accepted,generation,nullptr);}
bool Session::acknowledge_output(const std::shared_ptr<const Output>&out,bool accepted,std::uint64_t generation){return out&&acknowledge_impl(out->stamp,accepted,generation,out.get());}
bool Session::acknowledge_impl(const Stamp&stamp,bool accepted,std::uint64_t published_generation,const Output* expected){auto&i=*impl_;
 struct Retired {std::decay_t<decltype(i.committed)> committed,ready_snapshot;std::decay_t<decltype(i.channel_cache)> channels,ready_channels;
  std::decay_t<decltype(i.sever_state)> sever,ready_sever;std::decay_t<decltype(i.boundary_cache)> boundary,ready_boundary;
  std::decay_t<decltype(i.region_cache)> regions,ready_regions;std::decay_t<decltype(i.exterior_memo)> memo,ready_memo;
  std::decay_t<decltype(i.ready_grid)> ready_grid;std::decay_t<decltype(i.done)> done;};
 auto retired=std::make_shared<Retired>();std::shared_ptr<const DamageSnapshot> prefitSnapshot;Stamp prefitStamp; /* speedfix */const bool acknowledged=[&]{
 std::lock_guard<std::mutex>lock(i.mutex);if(i.canceled||!i.done||!(stamp.owner==i.done->stamp.owner)||stamp.revision!=i.done->stamp.revision||stamp.expected_generation!=i.done->stamp.expected_generation)return false;
 if(i.renderer_rebuild==RendererRebuildState::queued || (expected&&expected!=i.done.get()) ||
    (i.renderer_rebuild!=RendererRebuildState::none&&!expected) ||
    (accepted&&i.renderer_rebuild==RendererRebuildState::refused))return false;
 if(accepted&&published_generation!=UINT64_MAX&&published_generation<stamp.expected_generation)return false;
 // ownersafe (2026-11-15): a failed job no longer fails its owner for good (f13ab: one accessory refusal, then 237 pellets
 // "not_queued reason=owner_failed"). The failed batch is not applied (the committed state stays as published); the hits queued
 // after it and every later hit go on. Only an out-of-memory failure still stops the owner.
 const bool jobError=!i.done->error.empty();i.failed=jobError&&(i.done->error=="out_of_memory"||i.done->error=="low_memory");
 body_hit_outcomes::output(!accepted||jobError?"discarded":!i.done->resource_failure.empty()?"not_applied":i.done->no_geometry_change?"committed_no_op":"committed",
 jobError?i.done->error.c_str():!accepted?"native_gate_or_stale_generation":!i.done->resource_failure.empty()?i.done->resource_failure.c_str():nullptr,i.owner.spawn_cookie,*i.done);
 if(accepted&&!jobError&&i.ready_snapshot){
  if(published_generation!=UINT64_MAX){
   // This candidate has not been exported. The worker's done pointer and its
   // output stamp remain unchanged; only the accepted snapshot is stamped.
   i.ready_snapshot->impl_->stamp.expected_generation=published_generation;
  }
  retired->committed=std::move(i.committed);i.committed=std::move(i.ready_snapshot);
  if(!i.piece&&i.committed){prefitSnapshot=i.committed;prefitStamp=i.committed->impl_->stamp;} // speedfix
  if(i.ready_grid){if(i.spare_ready_serial&&i.spare_ready_dirty){i.spare_deltas.emplace_back(i.spare_ready_serial,std::move(i.spare_ready_dirty));while(i.spare_deltas.size()>16)i.spare_deltas.pop_front();i.spare_committed_serial=i.spare_ready_serial;}
   else{i.spare_deltas.clear();i.spare_committed_serial=0;}}
  if(i.ready_channel_cache){retired->channels=std::move(i.channel_cache);i.channel_cache=std::move(i.ready_channel_cache);}
  if(i.ready_sever){retired->sever=std::move(i.sever_state);i.sever_state=std::move(i.ready_sever);}
 if(i.ready_grid){retired->boundary=std::move(i.boundary_cache);retired->regions=std::move(i.region_cache);i.boundary_cache=std::move(i.ready_boundary);i.region_cache=std::move(i.ready_regions);}
  if(i.ready_exterior_memo){retired->memo=std::move(i.exterior_memo);i.exterior_memo=std::move(i.ready_exterior_memo);}
 }
 retired->ready_channels=std::move(i.ready_channel_cache);retired->ready_sever=std::move(i.ready_sever);retired->ready_snapshot=std::move(i.ready_snapshot);
 retired->ready_grid=std::move(i.ready_grid);retired->ready_boundary=std::move(i.ready_boundary);retired->ready_regions=std::move(i.ready_regions);retired->ready_memo=std::move(i.ready_exterior_memo);
 i.spare_ready_serial=0;i.spare_ready_dirty.reset();
 i.ready_channel_cache.reset();i.ready_sever.reset();i.ready_snapshot.reset();i.ready_grid.reset();i.ready_boundary.clear();i.ready_regions.clear();i.ready_exterior_memo.reset();i.acknowledged=accepted?stamp.revision:i.acknowledged;i.renderer_rebuild=RendererRebuildState::none;i.renderer_rebuild_reason="none";retired->done=std::move(i.done);i.done.reset();i.inflight=false;if(i.failed){body_hit_outcomes::batch("discarded","owner_failed_after_output",i.owner.spawn_cookie,i.pending);body_hit_outcomes::batch("discarded","owner_failed_after_output",i.owner.spawn_cookie,i.retry_pending);i.pending.clear();i.retry_pending.clear();}i.last_activity=core::Clock::now();return true;}();
 if(acknowledged)retire(std::move(retired));
 else body_hit_outcomes::report("acknowledgement_refused","stamp_state_or_generation_retained",stamp.owner.spawn_cookie,stamp.revision);
 if(prefitSnapshot)(void)prefit_after_commit(prefitSnapshot->impl_->asset,prefitSnapshot->impl_->grid,prefitStamp); // speedfix
 return acknowledged;}
void Session::cancel()noexcept{auto&i=*impl_;std::lock_guard<std::mutex>lock(i.mutex);
 body_hit_outcomes::batch("discarded","owner_canceled_pending",i.owner.spawn_cookie,i.pending);body_hit_outcomes::batch("discarded","owner_canceled_retry",i.owner.spawn_cookie,i.retry_pending);if(i.done)body_hit_outcomes::output("discarded","owner_canceled_unpublished",i.owner.spawn_cookie,*i.done);i.canceled=true;i.ready_channel_cache.reset();i.ready_sever.reset();i.pending.clear();i.retry_pending.clear();i.done.reset();i.ready_regions.clear();i.ready_exterior_memo.reset();}
bool Session::pending()const{auto&i=*impl_;std::lock_guard<std::mutex>lock(i.mutex);return !i.canceled&&!i.failed&&(i.inflight||!i.pending.empty()||!i.retry_pending.empty()||bool(i.done));}
std::shared_ptr<const DamageSnapshot>Session::capture_damage()const{
 auto&i=*impl_;std::lock_guard<std::mutex>lock(i.mutex);
 if(i.canceled||i.failed||i.inflight||i.done||!i.pending.empty()||!i.retry_pending.empty())return {};
 return i.committed;
}
namespace {
// ESTIMATED bytes of an owner's recomputable caches (container sizes; shared
// region results counted once). Caller holds the session mutex; never allocates.
std::size_t regionResultBytes(const fastwound::RegionResult&r){
 std::size_t tris=0;for(const auto&g:r.groups)tris+=g.second.size();
 return sizeof(r)+r.keys.size()*sizeof(Cell)+r.masks.size()*sizeof(core::Tile)+(r.faces.size()+r.selected.size())*4+
  r.rows.size()*(sizeof(gfy_packed::Vertex)+16)+tris*sizeof(fastwound::PackedTri)+r.groups.size()*48+r.rects.size()*sizeof(fastwound::RectMeta)+
  r.planeRows.size()*64+r.expanded.size()*sizeof(Cell)+r.protect.size()+r.relevant.size();
}
}
std::size_t Session::cache_bytes_locked(const Impl&i)noexcept{
 std::size_t bytes=0;if(i.channel_cache){bytes+=sizeof(channelworker::State);for(const auto&c:i.channel_cache->children)bytes+=sizeof(channelworker::ChildPacket)+c->geometry.vertices.size()*sizeof(gfy_packed::Vertex)+c->geometry.indices.size()*2+c->geometry.original_faces.size()*4+c->owners.size()*8;}
 std::array<const fastwound::RegionResult*,1024>seen{};std::size_t seenCount=0;
 auto region=[&](const std::shared_ptr<const fastwound::RegionResult>&r){if(!r)return;
  for(std::size_t k=0;k<seenCount;++k)if(seen[k]==r.get())return;if(seenCount<seen.size())seen[seenCount++]=r.get();bytes+=regionResultBytes(*r);};
 for(const auto&r:i.region_cache)region(r);
 for(const auto&q:i.boundary_cache)bytes+=64+(q.second?q.second->size()*sizeof(core::Quad):0);
 if(i.tile_state){const auto&s=*i.tile_state;for(const auto&r:s.lastTileLocal)region(r);for(const auto&r:s.lastFallback)region(r);
  for(const auto&l:s.lattices)if(l.second){bytes+=96;for(const auto&p:l.second->lattice.parts)bytes+=sizeof(p)+p.poly.size()*sizeof(fastwound::LatticeVertex);}
  for(const auto&st:s.current)if(st.second)bytes+=96+st.second->rows.size()*sizeof(fastwound::LatticeRow)+st.second->tris.size()*12;
  for(const auto&st:s.staticMemo)if(st.second.second)bytes+=96+st.second.first.regions.size()*8+st.second.second->rows.size()*sizeof(fastwound::LatticeRow)+st.second.second->tris.size()*12;
  for(const auto&k:s.staticKeys)bytes+=48+k.second.regions.size()*8;
  for(const auto&ch:s.part.chunks)bytes+=96+(ch.second.tiles.size()+ch.second.E.size())*sizeof(Cell);bytes+=(s.part.tiles.size()*24+s.part.haloCount.size()*12)*2;
  for(const auto&j:s.joinedFaces)bytes+=96+j.second.first.size()*sizeof(Cell)+j.second.second.size()*4;
  bytes+=(s.mask.size()+s.current.size()+s.failedTileLocal.size())*48;}
 if(i.tile_faces)for(const auto&t:*i.tile_faces)bytes+=96+(t.second.selected.size()+t.second.active.size()+t.second.touching.size())*4;
 if(i.exterior_memo)bytes+=i.exterior_memo->faces.size()*256;
 return bytes;
}
Session::MemoryState Session::memory_state()const noexcept{
 auto&i=*impl_;std::lock_guard<std::mutex>lock(i.mutex);MemoryState out;
 out.need_commit=i.need_commit;out.deferrals=i.deferrals;out.failures=i.memory_failures;out.waiting=i.memory_waiting;
 if(!i.inflight&&!i.done)out.cache_bytes=cache_bytes_locked(i);return out;
}
std::size_t Session::evict_caches()noexcept{
 auto&i=*impl_;std::lock_guard<std::mutex>lock(i.mutex);
 // Never while a job runs (it reads these caches without the mutex) or while
 // an unacknowledged output still has to move its ready caches in.
 if(i.inflight||i.done)return 0;
 // Nothing here allocates (it may run inside an engine out-of-memory handler).
 const std::size_t bytes=cache_bytes_locked(i);
 i.channel_cache.reset();i.ready_channel_cache.reset();
 i.region_cache.clear();i.region_cache.shrink_to_fit();
 if(!i.boundary_cache.empty()){i.boundary_cache.clear();i.boundary_rebuild=!i.canceled;}i.boundary_valid=false;
 i.tile_state.reset();i.tile_faces.reset();i.exterior_memo.reset();
 return bytes;
}
#ifdef GFY_VOXEL_WORKER_TESTS
// ---- TEST-ONLY FAULT INJECTION: exists only with GFY_VOXEL_WORKER_TESTS. ----
// Production builds contain neither this hook nor its call. It runs at the
// real changed-candidate geometry gate, before boundary/join work, and throws
// the kernel's own exception types, so dispatch, rough/smooth retry, prefix
// splitting, publication, acknowledgement and commit remain production code.
namespace worker_test {
enum class Fault {none,joined_gate,recoverable,cavity_provenance,packing,fatal};
struct Gate {const Session* session{};std::vector<std::uint64_t> sequences;unsigned attempt{};bool used_rough{};};
bool suppress_resolved_written=false;
std::function<void()> renderer_rebuild_hook; // TEST ONLY, before actual geometry work.
bool fail_history_after_sphere=false; // TEST ONLY: fail inside the real event journal.
std::function<Fault(const Gate&)> gate_fault; // Replace only while every worker is idle.
void gate(const Session* session,const std::vector<Impact>& events,std::size_t active,unsigned attempt,bool used_rough){
 if(!gate_fault)return;Gate context{session,{},attempt,used_rough};
 for(std::size_t n=0;n<active;++n)context.sequences.push_back(events[n].sequence);
 switch(gate_fault(context)){
  case Fault::none:return;
  case Fault::joined_gate:throw RecoverableImpactError("joined_boundary_gate test_injected");
  case Fault::recoverable:throw RecoverableImpactError("test_injected_recoverable");
  case Fault::cavity_provenance:throw core::CavityProvenanceError("test_injected_cavity_provenance");
  case Fault::packing:throw core::packing::PackingInfluenceError("test_injected_packing");
  case Fault::fatal:throw std::runtime_error("test_injected_fatal");
 }
}
}
#define GFY_WORKER_TEST_GATE() worker_test::gate(task.session.get(),task.events,activeEvents,attempt,usedRough)
#else
#define GFY_WORKER_TEST_GATE() ((void)0)
#endif
struct Pool::Impl {
 // R68 D5: the damage job's joint measures on two worker-pool threads beside the analytic pack. The touched joints are decided
 // exactly as the joint stage decides them; each value is stored at its joint index; the cranial halves are combined as
 // CranialMeasure::evaluate combines them. Measures are claimed under the lock; finish() closes the claims and waits only for the
 // measures in progress (a helper that starts later finds nothing to claim and touches only this state, which it co-owns).
 // done: every measure ran (or one failed: rethrown at the joint stage).
 struct JointMeasuresBeside {
  const Asset* asset{};const Grid* grid{};
  std::vector<char> touched;std::vector<JointDamage> values;JointDamage cranialTissue,cranialBrain;CranialDamage cranial;bool valid=false,done=false;
  std::vector<std::size_t> tasks;std::mutex m;std::condition_variable cv;std::size_t next=0,inflight=0,finished=0;bool closed=false;std::exception_ptr error;
  void rethrow()const{if(error)std::rethrow_exception(error);}
  void measure(std::size_t j){const std::size_t J=asset->joints.size();const auto mt=core::Clock::now();
   if(j==J)cranialTissue=asset->cranial_measure.tissue.evaluate(*grid);else if(j==J+1)cranialBrain=asset->cranial_measure.brain.evaluate(*grid);
   else values[j]=asset->joints[j].evaluate(*grid);
   static const bool jl=std::getenv("GFY_MS_JOINTLOG")!=nullptr;if(jl)std::fprintf(stderr,"M5JOINT j=%zu J=%zu ms=%.3f\n",j,J,core::ms(mt));} // probe read-out
  bool claim(std::size_t&k){std::lock_guard<std::mutex> l(m);if(closed||next>=tasks.size())return false;k=next++;++inflight;return true;}
  // takes measures until none is left to claim (a failure closes the claims)
  void run(){for(std::size_t k;claim(k);){std::exception_ptr failure;try{measure(tasks[k]);}catch(...){failure=std::current_exception();}
    std::lock_guard<std::mutex> l(m);--inflight;if(failure){if(!error)error=failure;closed=true;}else ++finished;
    if(!inflight)cv.notify_all();if(failure)return;}}
  // after the pack: the job thread takes what is left (abandon: the pack threw, nothing more starts), closes the claims and waits
  // for the measures in progress
  void finish(bool abandon){if(!abandon)run();
   std::unique_lock<std::mutex> l(m);closed=true;cv.wait(l,[&]{return inflight==0;});
   done=!abandon&&(error||finished==tasks.size());
   if(done&&!error&&valid){const auto&a=cranialTissue;const auto&b=cranialBrain;cranial.valid=true;
    cranial.original_m3=a.original_m3;cranial.removed_m3=a.removed_m3;cranial.original_bone_m3=a.original_bone_m3;cranial.removed_bone_m3=a.removed_bone_m3;
    cranial.original_brain_m3=b.original_m3;cranial.removed_brain_m3=b.removed_m3;}}
  // a joint's tile box: the tiles its evaluate() reads (the session's cache; asset and grid geometry only)
  static void cacheJointTiles(Session::Impl&i,const Grid&grid){i.spare_joint_tiles.clear();
   for(const auto&joint:i.asset->joints){std::array<int,6> box{INT_MAX,INT_MAX,INT_MAX,INT_MIN,INT_MIN,INT_MIN};
    for(const auto&block:joint.blocks){const Vec blockLo=joint.origin+Vec{double(block.key.x*8),double(block.key.y*8),double(block.key.z*8)}*joint.pitch;
     for(int a=0;a<3;++a){const int lo=std::max(0,int(std::floor((blockLo[a]-grid.origin[a])/grid.pitch+1e-7)))/8;
      const int hi=std::min(grid.size[a]-1,int(std::ceil((blockLo[a]+8*joint.pitch-grid.origin[a])/grid.pitch-1e-7))-1)/8;box[a]=std::min(box[a],lo);box[3+a]=std::max(box[3+a],hi);}}
    i.spare_joint_tiles.push_back(box);}}
  static std::shared_ptr<JointMeasuresBeside> start(Session::Impl&i,const Grid&grid,const std::set<Cell>&dirty)noexcept{
   std::shared_ptr<JointMeasuresBeside> s;
   try{const std::size_t J=i.asset->joints.size();const std::vector<JointDamage>* prevJoints=nullptr;
    if(!grid.ownership&&i.committed->impl_->geometry&&i.committed->impl_->geometry->joints.size()==J)prevJoints=&i.committed->impl_->geometry->joints;
    if(prevJoints&&i.spare_joint_tiles.size()!=J)cacheJointTiles(i,grid); // as the joint stage would
    s=std::make_shared<JointMeasuresBeside>();s->asset=i.asset.get();s->grid=&grid;s->touched.assign(J,1);s->values.resize(J);
    for(std::size_t j=0;j<J;++j)if(prevJoints){const auto&b=i.spare_joint_tiles[j];bool t=false;
      for(auto it=dirty.lower_bound(Cell{b[0],INT_MIN,INT_MIN});it!=dirty.end()&&it->x<=b[3]&&!t;++it)
       t=it->y>=b[1]&&it->y<=b[4]&&it->z>=b[2]&&it->z<=b[5];s->touched[j]=t;}
    const auto&cm=i.asset->cranial_measure;s->valid=cm.valid;if(cm.valid){s->tasks.push_back(J);s->tasks.push_back(J+1);}
    for(std::size_t j=0;j<J;++j)if(s->touched[j])s->tasks.push_back(j);
    auto&pool=core::workerPool();
    for(std::size_t h=0,n=std::min<std::size_t>(2,s->tasks.size());h<n;++h)pool.submit([s]{s->run();});
    return s;}
   catch(...){if(s)s->finish(true);return nullptr;}}
 };
 // failure: preallocated at dispatch on the owner thread (one impact per event,
 // sequences filled), so a job that runs out of memory can always report its
 // batch without allocating. deferrals: memory deferrals this batch already had.
 struct Work {std::shared_ptr<Session>session;std::vector<Impact>events;Stamp stamp;bool isolated{};bool trace_transferred{};std::shared_ptr<Output>failure;std::uint32_t deferrals{};
  std::shared_ptr<const Output> renderer_source;
  std::shared_ptr<const DamageSnapshot> renderer_snapshot;
 };
 // Admission record of one job (worker thread only).
 struct JobMemory {std::uint64_t estimate{};std::size_t quads{},faces{},triangles{},body{},regions{};bool heavy{};HeavyJoinHold hold;};
 std::mutex mutex;std::condition_variable wake;std::deque<Work>work;std::vector<std::weak_ptr<Session>>owners;std::vector<std::thread>threads;unsigned max_owners;bool stop{};
 // Other engine systems' value-only tasks (Pool::submit_task) on their own lane: own threads and wake, never wound work.
 std::deque<std::function<void()>> tasks;std::condition_variable task_wake;std::vector<std::thread> task_threads;unsigned task_workers{1};
 Impl(unsigned count,unsigned maximum):max_owners(maximum){need(count&&count<=16&&maximum&&maximum<=pool_max_owners,"pool_limits");try{for(unsigned n=0;n<count;++n)threads.emplace_back([this]{run();});}catch(...){ {std::lock_guard<std::mutex> lock(mutex);stop=true;}wake.notify_all();for(auto&t:threads)t.join();throw;}}
 void quit()noexcept{{std::lock_guard<std::mutex>lock(mutex);stop=true;for(const auto&t:work)body_hit_outcomes::batch("discarded","pool_quit_queued_work",t.stamp.owner.spawn_cookie,t.events);work.clear();tasks.clear();}wake.notify_all();task_wake.notify_all();for(auto&t:threads)if(t.joinable())t.join();for(auto&t:task_threads)if(t.joinable())t.join();} // quitfix: Pool::quit
 ~Impl(){quit();}
 // Capture only strong references while holding the pool lock. Cache traversal
 // and per-session locks must not serialize every owner dispatch behind a governor.
 std::array<std::shared_ptr<Session>,pool_max_owners> ownerSnapshot(){
  std::array<std::shared_ptr<Session>,pool_max_owners> snapshot{};std::size_t count=0;
  std::lock_guard<std::mutex>lock(mutex);
  for(const auto&weak:owners){if(count==snapshot.size())break;if(auto session=weak.lock())snapshot[count++]=std::move(session);}
  return snapshot;
 }
 // The task lane: a wound backlog cannot hold a task back, and a task never occupies a wound worker.
 void run_tasks(){painterThreadEntry();for(;;){std::function<void()> next;{std::unique_lock<std::mutex>lock(mutex);task_wake.wait(lock,[&]{return stop||!tasks.empty();});if(stop)return;
   next=std::move(tasks.front());tasks.pop_front();}
  painterThreadEntry(); // the engine may install its policy after this thread started
  try{next();}catch(...){} // another system's task: it owns its errors; nothing of the kernel changes
 }}
 void run(){painterThreadEntry();for(;;){Work task;{std::unique_lock<std::mutex>lock(mutex);wake.wait(lock,[&]{return stop||!work.empty();});if(stop)return;
   task=std::move(work.front());work.pop_front();}
  painterThreadEntry(); // the engine may install its policy after this thread started
  auto&i=*task.session->impl_;{std::lock_guard<std::mutex>lock(i.mutex);if(i.canceled){body_hit_outcomes::batch("discarded","owner_canceled_before_worker",i.owner.spawn_cookie,task.events);continue;}}
  // Nothing escapes a pool thread. Every transient of the job is unwound
  // before these handlers run. A memory shortage defers the batch (its
  // events return unchanged to the owner queue); only a batch starved for a
  // long time reports every event not applied (see tryDefer); committed
  // occupancy and published geometry never
  // change on these paths and the owner stays usable.
  {JobMemory memory;
  try{if(task.renderer_source)runRendererRebuild(task,memory);else runJob(task,memory);}
  catch(const AdmissionCanceled&){} // canceled owner: nothing is published
  catch(const MemoryShortage&shortage){if(task.renderer_source)refuseRendererRebuild(task,"low_memory");else memoryShortage(task,memory,shortage.need,false);}
  catch(const std::bad_alloc&){if(task.renderer_source)refuseRendererRebuild(task,"out_of_memory");else memoryShortage(task,memory,0,true);}
  catch(...){if(task.renderer_source)refuseRendererRebuild(task,"complete_geometry_rebuild_failed");else{i.tile_state.reset();publishFailure(task,"worker_failure");}}}
  {std::lock_guard<std::mutex>lock(i.mutex);if(i.canceled&&!task.trace_transferred)body_hit_outcomes::batch("discarded","owner_canceled_during_worker",i.owner.spawn_cookie,task.events);
   }
  // Owner wake-up (Session::set_ready_hook): after the heavy-join hold above is
  // released and outside every kernel lock. A failing hook changes nothing (the
  // owner still polls); the output is identical with or without a hook.
  try{std::function<void()>ready;{std::lock_guard<std::mutex>lock(i.mutex);if(i.done&&!i.canceled)ready=i.ready_hook;}if(ready)ready();}catch(...){}
 }}
 enum class Deferral {canceled,deferred,refused};
 Deferral tryDefer(Work&task,std::uint64_t need)noexcept{
  auto&i=*task.session->impl_;
  try{std::lock_guard<std::mutex>lock(i.mutex);
   if(i.canceled)return Deferral::canceled; // a canceled owner publishes nothing
   if(task.events.empty())return Deferral::refused;
   // Never drop a hit for memory: the batch returns to the queue every time. After isolateAfterDeferrals a
   // multi-event batch is isolated (one event per job, ahead of fresh events, like a failed batch) so smaller
   // jobs can fit; only a job still short after maxBatchDeferrals is reported not applied.
   const auto first=task.events.front().sequence;
   i.batch_deferrals=i.memory_deferred==first?i.batch_deferrals+1:1;
   if(i.batch_deferrals>maxBatchDeferrals)return Deferral::refused;
   const bool isolate=!task.isolated&&task.events.size()>1&&i.batch_deferrals>=isolateAfterDeferrals&&i.retry_pending.empty();
   // Same order, in front of anything that arrived meanwhile.
   body_hit_outcomes::batch("deferred","memory_retry_not_dropped",i.owner.spawn_cookie,task.events);
   if(task.isolated||isolate)i.retry_pending.insert(i.retry_pending.begin(),std::make_move_iterator(task.events.begin()),std::make_move_iterator(task.events.end()));
   else i.pending.insert(i.pending.begin(),std::make_move_iterator(task.events.begin()),std::make_move_iterator(task.events.end()));
   i.memory_deferred=first;i.need_commit=need;++i.deferrals;
   const auto now=core::Clock::now();i.memory_waiting=true;i.memory_wait_since=now;i.last_activity=now;
   task.trace_transferred=true;i.inflight=false;return Deferral::deferred;
  }catch(...){return Deferral::refused;}
 }
 // Reports the whole batch not applied from the output preallocated at
 // dispatch (short reasons: nothing allocates). The owner acknowledges it as
 // a no-op; occupancy, caches and published geometry are unchanged.
 void publishFailure(Work&task,const char*reason)noexcept{
  auto&i=*task.session->impl_;
  try{std::lock_guard<std::mutex>lock(i.mutex);
   if(i.canceled||!task.failure)return;
   auto&f=*task.failure;f.no_geometry_change=true;f.resource_failure=reason;for(auto&impact:f.impacts)impact.rejection=reason;
   f.tiles=i.committed->sparse_tiles();f.deferred_events=0;
   ++i.memory_failures;i.need_commit=0;i.memory_waiting=false;i.last_activity=core::Clock::now();
   i.ready_channel_cache.reset();i.ready_sever.reset();i.ready_snapshot.reset();i.ready_grid.reset();i.ready_boundary.clear();i.ready_regions.clear();i.ready_exterior_memo.reset();
   body_hit_outcomes::output("worker_result",reason,i.owner.spawn_cookie,*task.failure);task.trace_transferred=true;i.done=task.failure;
  }catch(...){}
 }
 void memoryShortage(Work&task,JobMemory&memory,std::uint64_t need,bool allocationFailed)noexcept{
  auto&i=*task.session->impl_;
  // An allocation can fail in the middle of a worker-only memo update. The
  // memo is pure (identical outputs, see evict_caches): start it fresh.
  if(allocationFailed)i.tile_state.reset();
  const auto policy=memory_policy();
  if(!need)need=std::max(memory.estimate,policy.heavy_bytes)+policy.commit_margin;
  switch(tryDefer(task,need)){
   case Deferral::canceled:return;
   case Deferral::deferred:(allocationFailed?memoryCounters.oom_deferred:memoryCounters.admission_deferred).fetch_add(1,std::memory_order_relaxed);return;
   case Deferral::refused:break;}
  (allocationFailed?memoryCounters.oom_failed:memoryCounters.admission_failed).fetch_add(1,std::memory_order_relaxed);
  publishFailure(task,allocationFailed?outOfMemoryText:lowMemoryText);
 }
 // Heavy join admission (worker thread, before the join allocates anything):
 // one heavy job joins at a time, and only with commit for it.
 void admitJoin(Session::Impl&i,JobMemory&memory,std::size_t quads,std::size_t faces,std::size_t triangles,std::size_t regions){
  // The whole-body output is rebuilt every job: its committed size (or the
  // untouched source mesh before the first wound) scales the pack stage.
  const auto&committedGeometry=i.committed->impl_->geometry;
  const std::size_t body=committedGeometry&&committedGeometry->triangles?committedGeometry->triangles:i.asset->source.f.size();
  // Job-level maxima over every join of this job (tile-local, per-region
  // fallback, former-path retry): the estimate only grows within a job.
  memory.quads=std::max(memory.quads,quads);memory.faces=std::max(memory.faces,faces);memory.triangles=std::max(memory.triangles,triangles);
  memory.body=std::max(memory.body,body);memory.regions=std::max(memory.regions,regions);
  const auto estimate=join_estimate_bytes(memory.quads,memory.faces,memory.triangles,memory.body);memory.estimate=std::max(memory.estimate,estimate);
  counterMax(memoryCounters.max_join_estimate,estimate);
  const auto policy=memory_policy();if(!policy.enabled||estimate<policy.heavy_bytes)return;
  memory.heavy=true;
  auto canceled=[&]{std::lock_guard<std::mutex>own(i.mutex);return i.canceled;};
  if(!memory.hold.held){auto&g=heavyJoinGate();std::unique_lock<std::mutex>lock(g.mutex);bool waited=false;
   while(g.busy){waited=true;g.wake.wait_for(lock,std::chrono::milliseconds(50));if(canceled())throw AdmissionCanceled{};}
   g.busy=true;memory.hold.held=true;memoryCounters.heavy_joins.fetch_add(1,std::memory_order_relaxed);
   if(waited)memoryCounters.heavy_waits.fetch_add(1,std::memory_order_relaxed);}
  const auto first=core::Clock::now();bool physCounted=false;
  for(;;){const auto m=query_memory();if(!m.valid)return;
   const bool commitOk=m.avail_commit>=estimate+policy.commit_margin;
   const bool physOk=m.avail_phys>=estimate+policy.phys_margin||core::ms(first)>=policy.phys_wait_ms;
   if(commitOk&&physOk)return;
   if(canceled())throw AdmissionCanceled{};
   // Brief wait for memory another job or the governor is releasing.
   if(!commitOk&&core::ms(first)>=policy.admission_wait_ms)throw MemoryShortage{estimate+policy.commit_margin};
   if(!physOk&&!physCounted){physCounted=true;memoryCounters.phys_waits.fetch_add(1,std::memory_order_relaxed);}
   std::this_thread::sleep_for(std::chrono::milliseconds(20));}
 }
 // Exposed faces of one tile's removed cells: faces between a removed cell and a cell that is not removed, inside
 // the tile or in a neighbour tile of the same region (a face towards a tile outside the region counts as exposed).
 // Bit (y*8+z) of removed[x] is cell (x,y,z). This is the number of unit quads tileBoundary emits for the tile
 // before the envelope discards the outside ones, i.e. the direct path's real per-tile input (the former
 // 3 x popcount(removed) over-counted it 12-47x on 1 mm masks).
 static std::size_t tileExposedFaces(const core::Tile&t,const core::Tile*xm,const core::Tile*xp,const core::Tile*ym,const core::Tile*yp,const core::Tile*zm,const core::Tile*zp)noexcept{
  constexpr std::uint64_t z7=0x8080808080808080ull,z0=0x0101010101010101ull,y7=0xFF00000000000000ull,y0=0x00000000000000FFull;
  auto pc=[](std::uint64_t v){return std::size_t(std::bitset<64>(v).count());};
  std::size_t n=0;
  for(int x=0;x<8;++x){const std::uint64_t v=t.removed[x];if(!v)continue;
   n+=pc(v&~z7&~(v>>1))+pc(v&z7&~(zp?(zp->removed[x]&z0)<<7:0ull));   // +z (z=7 looks into zp's z=0)
   n+=pc(v&~z0&~(v<<1))+pc(v&z0&~(zm?(zm->removed[x]&z7)>>7:0ull));   // -z
   n+=pc(v&~y7&~(v>>8))+pc(v&y7&~(yp?(yp->removed[x]&y0)<<56:0ull));  // +y (y=7 looks into yp's y=0)
   n+=pc(v&~y0&~(v<<8))+pc(v&y0&~(ym?(ym->removed[x]&y7)>>56:0ull));  // -y
   n+=pc(v&~(x<7?t.removed[x+1]:(xp?xp->removed[0]:0ull)));            // +x
   n+=pc(v&~(x>0?t.removed[x-1]:(xm?xm->removed[7]:0ull)));}           // -x
  return n;}
 // Estimated join size of the regions computeMissingRegions will join.
 void admitRegions(Session::Impl&i,JobMemory&memory,const std::vector<fastwound::Region>&regions,const fastwound::RegionList&results,const fastwound::BoundaryCache&boundary){
  std::size_t quads=0,faces=0,count=0;
  for(std::size_t n=0;n<regions.size();++n){if(results[n])continue;++count;faces+=regions[n].faces.size();
   if(boundary.empty()){const auto&r=regions[n];const std::size_t tiles=std::min(r.keys.size(),r.masks.size());
    std::unordered_map<Cell,const core::Tile*,core::CellHash>at;at.reserve(tiles);
    for(std::size_t k=0;k<tiles;++k)at.emplace(r.keys[k],&r.masks[k]);
    auto nb=[&](Cell c,int dx,int dy,int dz)->const core::Tile*{auto f=at.find(Cell{c.x+dx,c.y+dy,c.z+dz});return f==at.end()?nullptr:f->second;};
    for(std::size_t k=0;k<tiles;++k){const Cell c=r.keys[k];
     quads+=tileExposedFaces(r.masks[k],nb(c,-1,0,0),nb(c,1,0,0),nb(c,0,-1,0),nb(c,0,1,0),nb(c,0,0,-1),nb(c,0,0,1));}}
   else for(const auto&key:regions[n].keys){auto found=boundary.find(key);if(found!=boundary.end())quads+=found->second->size();}}
  if(!count)return;
  // The committed regions these replace: cached results this candidate does
  // not reuse that share a tile with a region to join (a growing wound
  // re-joins its whole connected region). After a governor eviction there is
  // no cache: every committed triangle counts.
  std::size_t triangles=0;
  if(i.region_cache.empty()){if(const auto&g=i.committed->impl_->geometry)triangles=g->triangles;}
  else{std::unordered_set<Cell,core::CellHash>joined;
   for(std::size_t n=0;n<regions.size();++n)if(!results[n])joined.insert(regions[n].keys.begin(),regions[n].keys.end());
   for(const auto&old:i.region_cache)if(std::find(results.begin(),results.end(),old)==results.end())
    for(const auto&key:old->keys)if(joined.count(key)){triangles+=old->triangles;break;}}
  admitJoin(i,memory,quads,faces,triangles,count);
 }
 // Rebuild an evicted boundary cache from committed occupancy: every tile's
 // quads are a pure function of the grid, and the incremental cache holds
 // exactly the non-empty tiles (dirty sets cover every changed neighbour).
 static fastwound::BoundaryCache rebuildBoundaryCache(const Grid&grid,bool incremental){
  fastwound::BoundaryCache rebuilt;
  for(const auto&tile:grid.tiles){std::vector<core::Quad>quads;
   if(incremental)fastwound::tileBoundary(grid,tile.first,quads);else quads=core::dirtyBoundary(grid,{tile.first});
   if(!quads.empty())rebuilt[tile.first]=std::make_shared<const std::vector<core::Quad>>(std::move(quads));}
  return rebuilt;
 }
 void packIncrementalCandidate(Session::Impl&i,JobMemory&memory,const Grid&grid,const std::set<Cell>&dirty,
     fastwound::BoundaryCache&boundary_cache,fastwound::RegionList&regionResults,
     const std::shared_ptr<Output>&out,WorkerPhaseTimes&workerPhases,core::StageProfile&stages){

    // Incremental full-body path: only regions whose exact tile masks changed
    // are merged, clipped, joined and gated again; see FastWound_impl.h.
    // Tile-local regions first (TileRegions.h); if that candidate fails any
    // check, the former region path computes it (then the usual rejection).
    // Set when a per-region fallback region (former path) itself fails its gate:
    // that failure is the former path's own verdict, so it is not retried globally.
    bool formerRegionFailed=false;
    auto incrementalAttempt=[&](bool tileLocal){
    std::vector<fastwound::Region> regions;
    {PhaseTime boundaryTime(workerPhases.boundary_ms);need(grid.tiles.size()<=262144,"owner_sparse_tile_budget");
     const bool directPath=tileLocal&&fastwound::directDisplay()&&fastwound::chunkTiles()>0&&i.tile_state->noSplitTiles.empty();
     if(directPath){
      // Regions take their tiles' voxel faces straight from the grid (computeRegion): no quad cache to keep per hit
      // (it was 112 B per face, ~120 MB on a heavily shot body, and the bytes the governor evicted first).
      if(!i.boundary_cache.empty())fastwound::BoundaryCache().swap(i.boundary_cache);i.boundary_valid=false;}
     else if(!i.boundary_valid){i.boundary_cache=rebuildBoundaryCache(grid,true);i.boundary_valid=true;}
     else{// Each tile's quads are a pure function of this candidate grid: computed
      // concurrently, then stored in the former (ascending tile) order.
      const std::vector<Cell> tiles(dirty.begin(),dirty.end());std::vector<std::vector<core::Quad>> quads(tiles.size());
      core::parallelChunks(tiles.size(),256,[&](std::size_t first,std::size_t last,std::size_t){for(std::size_t n=first;n<last;++n)fastwound::tileBoundary(grid,tiles[n],quads[n]);});
      for(std::size_t n=0;n<tiles.size();++n){if(quads[n].empty())boundary_cache.erase(tiles[n]);else boundary_cache[tiles[n]]=std::make_shared<const std::vector<core::Quad>>(std::move(quads[n]));}}
     stages.mark("tile_boundary");regions=tileLocal?(directPath
         ?fastwound::findChunkRegions(grid,i.asset->source_tree,*i.asset->fast,*i.tile_state,fastwound::LatticeSpec{grid.origin,grid.pitch})
         :fastwound::findTileRegions(grid,i.asset->source_tree,*i.asset->fast,*i.tile_faces,*i.tile_state,fastwound::LatticeSpec{grid.origin,grid.pitch})):fastwound::findRegions(grid,i.asset->source_tree,*i.asset->fast,*i.tile_faces);}
    stages.mark("boundary");
    {std::size_t faces=0;for(const auto&r:regions)faces+=r.selected.size();need(faces>0,"fullbody_cut_without_exterior_patch");}
    fastwound::RegionList next;next.reserve(regions.size());std::size_t reused=0;
    for(const auto&region:regions){std::shared_ptr<const fastwound::RegionResult>reuse;
     for(const auto&old:i.region_cache)if(old->hash==region.hash&&old->tileLocal==region.tileLocal&&old->same(region.keys,region.masks,region.faces,region.protect,region.relevant)){reuse=old;break;}
     if(!reuse&&tileLocal)for(const auto&old:i.tile_state->lastTileLocal)if(old->hash==region.hash&&old->tileLocal&&old->same(region.keys,region.masks,region.faces,region.protect,region.relevant)){reuse=old;break;}
     reused+=bool(reuse);next.push_back(reuse);}
    std::vector<std::string> failures;
    if(tileLocal){// A tile-local region known to fail goes straight to the per-region fallback.
     for(std::size_t n=0;n<regions.size();++n)if(!next[n]&&i.tile_state->failedTileLocal.count(regions[n].hash))next[n]=std::make_shared<fastwound::RegionResult>();}
    admitRegions(i,memory,regions,next,boundary_cache);
    fastwound::computeMissingRegions(*i.asset,grid,regions,boundary_cache,next,workerPhases,tileLocal?i.tile_state.get():nullptr,tileLocal?&failures:nullptr);
    i.tile_state->fallbackRegions=0;
    if(tileLocal){
     // Per-region fallback: a tile-local region whose join throws, fails its gate
     // or shows a numerical defect is replaced by the former region of its removed
     // tiles -- sound only when that region's faces neither are nor share an edge
     // with a joined face of any other region (else the whole candidate falls back).
     std::vector<std::size_t> failed;bool chunkFailed=false;
     for(std::size_t n=0;n<regions.size();++n){std::string why=failures[n];const auto& r=next[n];
      // A chunk of a split cluster never falls back alone (its seams need the neighbour chunks): its tiles
      // are marked so the cluster is joined whole, and the tile-local attempt runs again (below).
      if(regions[n].chunked){if(why.empty()&&r&&(r->newOpen||r->nonmanifold||r->maxConflict>=1e-6))why="chunk_gate";if(why.empty()&&r)why=fastwound::regionDefect(*i.asset,*r);
       if(!why.empty()){chunkFailed=true;
        // First failure of a chunk: join its 3x3x3 chunk neighbourhood as one box. A failed box: the cluster joins whole.
        if(regions[n].boxed)for(const auto& c:regions[n].keys)i.tile_state->noSplitTiles.insert(c);
        else{const int ct=fastwound::chunkTiles()*8;i.tile_state->mergeChunks.insert(Cell{regions[n].seamLo[0]/ct,regions[n].seamLo[1]/ct,regions[n].seamLo[2]/ct});}
#ifdef GFY_VOXEL_STAGE_PROFILE
        std::cout<<"chunk_region_failed "<<why<<" new_open="<<(r?r->newOpen:0)<<" nonmanifold="<<(r?r->nonmanifold:0)<<" conflict="<<(r?r->maxConflict:0.)<<" chunk_lo="<<regions[n].seamLo[0]<<','<<regions[n].seamLo[1]<<','<<regions[n].seamLo[2]<<" keys="<<regions[n].keys.size()<<" lo_m="<<(r?r->seamLoM[0]:0.)<<','<<(r?r->seamLoM[1]:0.)<<','<<(r?r->seamLoM[2]:0.)<<" hi_m="<<(r?r->seamHiM[0]:0.)<<','<<(r?r->seamHiM[1]:0.)<<','<<(r?r->seamHiM[2]:0.)<<'\n';
#ifdef GFY_CUT_DIAGNOSTICS
        std::cout<<"chunk_tiles keys";for(const auto& c:regions[n].keys)std::cout<<' '<<c.x<<','<<c.y<<','<<c.z;std::cout<<" E";for(const auto& c:regions[n].expanded)std::cout<<' '<<c.x<<','<<c.y<<','<<c.z;std::cout<<'\n';
#endif
#endif
       }continue;}
      if(why.empty()&&r&&!r->tileLocal&&r->keys.empty())why="known_failure";
      if(why.empty()&&r&&(r->newOpen||r->nonmanifold||r->maxConflict>=1e-6)){
       // A gross failure (more than 64 rejected edges) is not a numerical rim
       // artefact of the tile-local exterior: it is the candidate's own verdict,
       // reported exactly like the former path (no fallback recomputation).
       // Likewise when every non-manifold edge is two overlapping CAVITY polygons:
       // cavity polygons come from the boundary quads and the source surface
       // alone, so the former path fails the same way (observed: 0.5 mm chest).
       if(r->newOpen+r->nonmanifold>64||(r->nonmanifold&&r->nonmanifoldCavityDoubled==r->nonmanifold)){formerRegionFailed=true;throw RecoverableImpactError("joined_boundary_gate new_open="+std::to_string(r->newOpen)+" nonmanifold="+std::to_string(r->nonmanifold)+" weight_conflict="+std::to_string(r->maxConflict));}
       why="gate new_open="+std::to_string(r->newOpen)+" nonmanifold="+std::to_string(r->nonmanifold);}
      if(why.empty()&&r)why=fastwound::regionDefect(*i.asset,*r);
      if(!why.empty()){failed.push_back(n);i.tile_state->failedTileLocal.insert(regions[n].hash);
#ifdef GFY_VOXEL_STAGE_PROFILE
       std::cout<<"tile_local_region_fallback "<<why<<'\n';
#endif
      }}
     if(chunkFailed)throw ChunkRetry{};
     if(!failed.empty()){
      std::vector<fastwound::Region> fb;for(auto n:failed)fb.push_back(fastwound::fallbackRegion(grid,i.asset->source_tree,*i.asset->fast,*i.tile_faces,regions[n]));
      // Faces of the fallback regions vs joined faces of the other tile-local regions.
      std::vector<char> other(i.asset->source.f.size(),0),mine(i.asset->source.f.size(),0);
      for(std::size_t n=0;n<regions.size();++n)if(std::find(failed.begin(),failed.end(),n)==failed.end())for(auto f:regions[n].faces)other[f]=1;
      const auto& data=*i.asset->fast;
      for(const auto& r:fb)for(auto f:r.faces){need(!mine[f],"tile_local_fallback_regions_overlap");mine[f]=1;
       need(!other[f],"tile_local_fallback_shares_face");
       for(int e=0;e<3;++e)for(auto m=data.edgeNeighbourFirst[3*f+e];m<data.edgeNeighbourFirst[3*f+e+1];++m)need(!other[data.edgeNeighbours[m]],"tile_local_fallback_edge_neighbour");}
      fastwound::RegionList fbResults(fb.size());
      for(std::size_t k=0;k<fb.size();++k){const auto& region=fb[k];
       for(const auto* cache:{&i.region_cache,&i.tile_state->lastFallback})for(const auto& old:*cache)if(!fbResults[k]&&old->hash==region.hash&&!old->tileLocal&&old->same(region.keys,region.masks,region.faces)){fbResults[k]=old;}}
      admitRegions(i,memory,fb,fbResults,boundary_cache);
      try{fastwound::computeMissingRegions(*i.asset,grid,fb,boundary_cache,fbResults,workerPhases,nullptr);}catch(const std::bad_alloc&){throw;}catch(...){formerRegionFailed=true;throw;}
      for(const auto&r:fbResults)if(r->newOpen||r->nonmanifold||r->maxConflict>=1e-6){formerRegionFailed=true;
       throw RecoverableImpactError("joined_boundary_gate new_open="+std::to_string(r->newOpen)+" nonmanifold="+std::to_string(r->nonmanifold)+" weight_conflict="+std::to_string(r->maxConflict));}
      for(std::size_t k=0;k<failed.size();++k)next[failed[k]]=fbResults[k];
      i.tile_state->lastFallback=fbResults;i.tile_state->fallbackRegions=failed.size();}
    }
#ifdef GFY_VOXEL_STAGE_PROFILE
    std::cout<<"voxel_regions total "<<next.size()<<" reused "<<reused<<'\n';
#endif
    stages.mark("join");
    std::size_t newOpen=0,nonmanifold=0,openEdges=0,seams=0,spurs=0,sheets=0,regionTriangles=0;double conflict=0,openLength=0;
    for(const auto&r:next){newOpen+=r->newOpen;nonmanifold+=r->nonmanifold;openEdges+=r->numericalOpenEdges;openLength+=r->numericalOpenLength;seams+=r->numericalSeams;spurs+=r->numericalSpurs;sheets+=r->interiorSheetJunctions;regionTriangles+=r->triangles;conflict=std::max(conflict,r->maxConflict);}
    if(newOpen||nonmanifold||conflict>=1e-6)throw RecoverableImpactError("joined_boundary_gate new_open="+std::to_string(newOpen)+" nonmanifold="+std::to_string(nonmanifold)+" weight_conflict="+std::to_string(conflict));
    // The numerical render-residual allowance stays one budget per published mesh.
    if(openEdges>16||openLength>1e-5)throw RecoverableImpactError("joined_boundary_gate new_open="+std::to_string(openEdges)+" nonmanifold=0 weight_conflict=0 numerical_budget");
    for(const auto&r:next)if(!r->packingError.empty())throw core::packing::PackingInfluenceError(r->packingError);
    std::size_t untouched=0;
    {PhaseTime packTime(workerPhases.pack_ms);if(tileLocal){fastwound::prepareStatics(i.asset->source,*i.asset->fast,next,*i.tile_state);fastwound::checkStatics(*i.tile_state);stages.mark("pack_statics");auto display=fastwound::mergedDisplayList(*i.asset,regions,next,*i.tile_state);stages.mark("pack_merge");out->mesh=fastwound::assemble(*i.asset,*i.asset->fast,display,untouched,i.tile_state.get());stages.mark("pack_assemble");}else out->mesh=fastwound::assemble(*i.asset,*i.asset->fast,next,untouched);}stages.mark("pack");
#ifdef GFY_VOXEL_STAGE_PROFILE
    {std::size_t ext=0,cav=0;for(const auto&r:next){cav+=r->cavityTriangles;ext+=r->triangles-r->cavityTriangles;}
     std::cout<<"lattice_stats joined_ext="<<ext<<" joined_cav="<<cav<<" static_tris="<<i.tile_state->staticTriangles<<" static_built="<<i.tile_state->staticBuilt<<" static_faces="<<i.tile_state->current.size()<<" static_dropped="<<i.tile_state->staticDropped<<" untouched_total="<<untouched<<"\n";}
#endif
    out->triangles=0;for(const auto& ch:out->mesh.children)out->triangles+=ch.indices.size()/3; // the published (low-poly) mesh
    (void)regionTriangles;(void)untouched;out->numerical_open_edges=openEdges;out->numerical_open_length_m=openLength;out->numerical_seams=seams;out->numerical_spurs=spurs;out->interior_sheet_junctions=sheets;
    if(tileLocal)i.tile_state->lastTileLocal=next;
    regionResults=std::move(next);
    };
    try{try{incrementalAttempt(true);}catch(const ChunkRetry&){out->mesh={};try{incrementalAttempt(true);}catch(const ChunkRetry&){out->mesh={};incrementalAttempt(true);}}}
    catch(const std::bad_alloc&){throw;} // resource failure, never a geometry verdict
    catch(const std::exception&tileLocalFailure){
     if(formerRegionFailed)throw;
#ifdef GFY_VOXEL_STAGE_PROFILE
     std::cout<<"tile_local_fallback "<<tileLocalFailure.what()<<'\n';
#endif
     (void)tileLocalFailure;out->mesh={};incrementalAttempt(false);}
 }
 struct RebuildCacheScope {
  Session::Impl&i;fastwound::BoundaryCache boundary;fastwound::RegionList regions;
  std::shared_ptr<fastwound::TileRegionState>state;std::shared_ptr<fastwound::TileFaceMemo>faces;bool valid;
  explicit RebuildCacheScope(Session::Impl&owner):i(owner),boundary(std::move(i.boundary_cache)),regions(std::move(i.region_cache)),
      state(std::move(i.tile_state)),faces(std::move(i.tile_faces)),valid(i.boundary_valid){i.boundary_valid=false;}
  ~RebuildCacheScope(){i.boundary_cache=std::move(boundary);i.region_cache=std::move(regions);
      i.tile_state=std::move(state);i.tile_faces=std::move(faces);i.boundary_valid=valid;}
 };
 void packWholeCandidate(Session::Impl&i,JobMemory&memory,const Grid&grid,const std::vector<core::Quad>&boundary,
     const std::shared_ptr<Output>&out,WorkerPhaseTimes&workerPhases,std::shared_ptr<core::ExteriorMemo>&candidate_memo,
     const core::ExteriorMemo*memoIn,core::StageProfile&stages){
    candidate_memo=std::make_shared<core::ExteriorMemo>();
    core::JoinResult joined;if(i.asset->face_children.empty()){core::ClipResult outside;{PhaseTime exteriorTime(workerPhases.exterior_ms);outside=core::exterior(i.asset->source,grid,memoIn,nullptr,candidate_memo.get());}{PhaseTime joinTime(workerPhases.join_ms);joined=core::join(i.asset->source,i.asset->source_tree,i.asset->envelope_tree,grid,outside,boundary);}}else joined=joinFullBody(*i.asset,grid,boundary,workerPhases,memoIn,candidate_memo.get());stages.mark("join");
    if(joined.newOpen||joined.nonmanifold||joined.maxJoinWeightConflict>=1e-6)throw RecoverableImpactError("joined_boundary_gate new_open="+std::to_string(joined.newOpen)+" nonmanifold="+std::to_string(joined.nonmanifold)+" weight_conflict="+std::to_string(joined.maxJoinWeightConflict));
    {PhaseTime packTime(workerPhases.pack_ms);out->mesh=i.asset->face_children.empty()?core::packing::pack(joined,i.asset->source_child,i.asset->original_children):core::packing::packMulti(joined,i.asset->face_children,i.asset->face_indices,i.asset->original_children);}stages.mark("pack");out->triangles=joined.triangles.size();out->numerical_open_edges=joined.numericalOpenEdges;out->numerical_open_length_m=joined.numericalOpenLength;out->numerical_seams=joined.numericalSeams;out->numerical_spurs=joined.numericalSpurs;out->interior_sheet_junctions=joined.interiorSheetJunctions;
 }
 void refuseRendererRebuild(Work&task,const char*reason)noexcept{
  auto&i=*task.session->impl_;std::lock_guard<std::mutex>lock(i.mutex);
  if(i.canceled||i.done!=task.renderer_source||i.ready_snapshot!=task.renderer_snapshot)return;
  // Keep the original immutable candidate/receipts. Never report it as a
  // no-op and never acknowledge it as accepted after a failed rebuild.
  i.renderer_rebuild=RendererRebuildState::refused;i.renderer_rebuild_reason=reason;
 }
 void runRendererRebuild(Work&task,JobMemory&memory){
  auto&i=*task.session->impl_;const auto start=core::Clock::now();
#ifdef GFY_VOXEL_WORKER_TESTS
  if(worker_test::renderer_rebuild_hook)worker_test::renderer_rebuild_hook();
#endif
  {std::lock_guard<std::mutex>lock(i.mutex);if(i.canceled)throw AdmissionCanceled{};}
  const Grid&grid=*task.renderer_snapshot->impl_->grid;
  // Copy original receipts/joints/diagnostics exactly; replace only geometry.
  // Existing Output has value-owned rows, so this temporary copy is charged by
  // normal allocator policy; an OOM preserves the original candidate.
  auto out=std::make_shared<Output>(*task.renderer_source);out->mesh={};
  WorkerPhaseTimes phases;core::StageProfile stages("renderer_rebuild");
  need(grid.tiles.size()<=262144,"owner_sparse_tile_budget");
  fastwound::BoundaryCache boundary_cache;fastwound::RegionList regions;
  std::shared_ptr<core::ExteriorMemo>memo;
  const bool incremental=!i.asset->face_children.empty()&&!grid.ownership&&i.asset->fast&&i.asset->fast->valid;
  {PhaseTime time(phases.boundary_ms);boundary_cache=rebuildBoundaryCache(grid,incremental);}
  if(incremental){
   RebuildCacheScope restore(i); // Restore original caches even if packing throws.
   i.tile_state=std::make_shared<fastwound::TileRegionState>();i.tile_faces=std::make_shared<fastwound::TileFaceMemo>();
   i.boundary_cache=boundary_cache;i.boundary_valid=true;
   // Full candidate boundary and empty region caches: all affected parts are
   // rebuilt; no masked/analytic result can leak through a cache hit.
   packIncrementalCandidate(i,memory,grid,{},boundary_cache,regions,out,phases,stages);
  }else{
   std::vector<core::Quad>boundary;
   for(const auto&item:boundary_cache)boundary.insert(boundary.end(),item.second->begin(),item.second->end());
   boundary=core::mergeBoundaryQuads(grid,boundary);
   admitJoin(i,memory,boundary.size(),std::min<std::size_t>(i.asset->source.f.size(),grid.tiles.size()*32),
       i.committed->impl_->geometry?i.committed->impl_->geometry->triangles:0,1);
   packWholeCandidate(i,memory,grid,boundary,out,phases,memo,nullptr,stages);
  }
  out->mesh.tissue_field=i.asset->tissue_field;
  out->complete_geometry_rebuild=true;out->renderer_rebuild_ms=core::ms(start);
  if(memo)memo->basis=task.renderer_snapshot->impl_->grid;
  std::lock_guard<std::mutex>lock(i.mutex);
  if(i.canceled||i.done!=task.renderer_source||i.ready_snapshot!=task.renderer_snapshot||
     i.renderer_rebuild!=RendererRebuildState::queued)return;
  // The ready snapshot has never been exported. Its grid/history/stamp and
  // identity stay exact; only the private presentation payload changes.
  i.ready_snapshot->impl_->geometry=out;
  i.ready_boundary=std::move(boundary_cache);i.ready_regions=std::move(regions);i.ready_exterior_memo=std::move(memo);
  i.done=std::move(out);i.renderer_rebuild=RendererRebuildState::complete;i.renderer_rebuild_reason="none";
  // Existing provisional mode caches are disposable, never semantic state.
  i.tile_state.reset();i.tile_faces.reset();i.boundary_valid=false;i.boundary_rebuild=true;
 }
 // painter_detail::buildDamageField with one traversal of the tile map instead of three (the two later passes
 // walk a contiguous list of the removed tiles). Same rules, same page order, byte-identical field.
 static std::shared_ptr<const painter_detail::DamageField> singlePassDamageField(const Grid&grid,std::size_t limit=16u<<20){
  using F=painter_detail::DamageField;auto out=std::make_shared<F>();
#ifdef GFY_RX_PROF
  auto fT=core::Clock::now();auto fMark=[&](const char*n){std::cerr<<"rxprof field."<<n<<' '<<core::ms(fT)<<std::endl;fT=core::Clock::now();};
#else
  auto fMark=[](const char*){};
#endif
  if(grid.ownership||!(grid.pitch>0)||!std::isfinite(grid.pitch))return out;
  struct E{Cell c;bool uniform;const core::Tile*t;};std::vector<E> ents;ents.reserve(grid.tiles.size());
  std::array<int,3> lo{INT32_MAX,INT32_MAX,INT32_MAX},hi{INT32_MIN,INT32_MIN,INT32_MIN};
  for(const auto&item:grid.tiles){std::uint64_t any=0,all=~std::uint64_t(0);for(auto w:item.second.removed){any|=w;all&=w;}if(!any)continue;
   ents.push_back({item.first,all==~std::uint64_t(0),&item.second});const int c[3]={item.first.x,item.first.y,item.first.z};
   for(int a=0;a<3;++a){lo[a]=std::min(lo[a],c[a]);hi[a]=std::max(hi[a],c[a]);}}
  fMark("traverse");out->pitch=float(grid.pitch);
  if(lo[0]==INT32_MAX){if(limit<(F::pageWidth+F::dataWidth)*4)return out;out->page.assign(F::pageWidth,0);out->data.assign(F::dataWidth,0);out->available=true;return out;}
  std::size_t count=1;const int dimensions[3]={grid.size.x,grid.size.y,grid.size.z};
  for(int a=0;a<3;++a){if(lo[a]<0||hi[a]>=((dimensions[a]+7)/8))return out;const std::uint64_t n=std::uint64_t(hi[a])-lo[a]+1;
   if(n>UINT32_MAX||count>limit/4/n)return out;count*=std::size_t(n);out->pages[a]=std::uint32_t(n);
   out->size[a]=std::uint32_t(std::min<std::int64_t>(std::int64_t(n)*8,std::int64_t(dimensions[a])-std::int64_t(lo[a])*8));out->origin[a]=float(grid.origin[a]+double(lo[a])*8*grid.pitch);}
  const std::size_t padded=((count+F::headerWords+F::pageWidth-1)/F::pageWidth)*F::pageWidth;
  if(padded>limit/4||F::dataWidth>limit/4-padded)return out;
  out->page.assign(padded,0);fMark("page_alloc");
  auto pageOf=[&](const Cell&c){return (std::size_t(c.z-lo[2])*out->pages[1]+unsigned(c.y-lo[1]))*out->pages[0]+unsigned(c.x-lo[0])+F::headerWords;};
  for(const auto&e:ents){const auto i=pageOf(e.c);if(e.uniform){out->page[i]=F::removedPage;++out->uniform;}else{out->page[i]=1;++out->mixed;}}
  fMark("page_mark");const std::size_t words=std::size_t(out->mixed)*16;
  const std::size_t paddedData=std::max<std::size_t>(F::dataWidth,((words+F::dataWidth-1)/F::dataWidth)*F::dataWidth);
  if(words>=F::removedPage||paddedData>limit/4-padded){out->page.clear();return out;}
  out->data.assign(paddedData,0);fMark("data_alloc");std::uint32_t cursor=0;
  for(std::size_t i=F::headerWords;i<F::headerWords+count;++i)if(out->page[i]==1){out->page[i]=cursor+1;cursor+=16;}
  fMark("prefix");
  for(const auto&e:ents){if(e.uniform)continue;const auto p=out->page[pageOf(e.c)];
   for(unsigned x=0;x<8;++x){const auto w=e.t->removed[x];out->data[p-1+2*x]=std::uint32_t(w);out->data[p+2*x]=std::uint32_t(w>>32);}}
  fMark("fill");auto fbits=[](float f){std::uint32_t u;std::memcpy(&u,&f,4);return u;};
  for(int a=0;a<3;++a){out->page[a]=fbits(out->origin[a]);out->page[4+a]=out->size[a];out->page[8+a]=out->pages[a];}
  out->page[3]=fbits(1.f/out->pitch);out->page[7]=3;out->page[11]=F::dataWidth;out->page[12]=F::pageWidth;out->page[13]=F::headerWords;
  out->available=true;return out;
 }
 void packAnalyticCandidate(Session::Impl&i,const Grid&grid,
    const std::shared_ptr<const ResolvedCutHistory>&history,
    std::shared_ptr<const channelworker::State>&candidate,
    const std::shared_ptr<Output>&out,WorkerPhaseTimes&phases,const std::set<Cell>*dirty=nullptr){
  PhaseTime whole(phases.pack_ms);fastwound::WallWeights weights(*i.asset);
#ifdef GFY_RX_PROF
  auto packT=core::Clock::now();auto packMark=[&](const char*n){std::cerr<<"rxprof pack."<<n<<' '<<core::ms(packT)<<std::endl;packT=core::Clock::now();};
#else
  auto packMark=[](const char*){};
#endif
  packMark("weights_ctor");
  packedchannel::WeightExecutor executor=[](unsigned jobs,const std::function<void(unsigned)>&work){
   core::parallelChunks(jobs,1,[&](std::size_t first,std::size_t last,std::size_t){for(auto n=first;n<last;++n)work(unsigned(n));});};
  auto prepared=channelworker::prepareChannels(i.channel_cache,history,i.asset,i.owner,weights,executor);
  packMark("prepare_channels");
  const auto maskStart=core::Clock::now();auto mask=false?singlePassDamageField(grid):painter_detail::buildDamageField(grid);
  need(mask&&mask->available,"analytic_exact_removal_field_unavailable");out->analytic_mask_ms=core::ms(maskStart);
  packMark("damage_field");
  gfy_packed::Mesh mesh;unsigned next=i.asset->original_children;
  // skin_clip is monotonic on a whole body (removed bits are never cleared): a chunk the committed analytic
  // publication clipped stays clipped; an unclipped chunk is tested only against this job's dirty tiles (a superset of
  // every tile that gained removed bits), with chunk boxes computed once per session.
  const gfy_packed::Mesh* prevMesh=nullptr;std::size_t staticTotal=0;
  for(const auto&source:i.asset->fast->static_child)staticTotal+=source.chunks.size();
  if(true&&dirty&&!i.piece&&!grid.ownership&&i.committed->impl_->geometry&&i.committed->impl_->geometry->mesh.analytic_channels&&i.committed->impl_->geometry->mesh.children.size()>=staticTotal)prevMesh=&i.committed->impl_->geometry->mesh;
  std::vector<std::array<double,6>> removedTiles;
  if(!prevMesh){removedTiles.reserve(grid.tiles.size());
   for(const auto&t:grid.tiles){bool any=false;for(auto w:t.second.removed)any|=w!=0;if(!any)continue;std::array<double,6>b;
    for(int a=0;a<3;++a){b[a]=grid.origin[a]+double((&t.first.x)[a])*8*grid.pitch;b[3+a]=b[a]+8*grid.pitch;}removedTiles.push_back(b);}}
  else for(const Cell&k:*dirty){auto it=grid.tiles.find(k);if(it==grid.tiles.end())continue;bool any=false;for(auto w:it->second.removed)any|=w!=0;if(!any)continue;std::array<double,6>b;
    for(int a=0;a<3;++a){b[a]=grid.origin[a]+double((&k.x)[a])*8*grid.pitch;b[3+a]=b[a]+8*grid.pitch;}removedTiles.push_back(b);}
  packMark("removed_tiles");
  if(prevMesh&&i.spare_static_boxes.size()!=staticTotal){i.spare_static_boxes.clear();
   for(const auto&source:i.asset->fast->static_child)for(const auto&c:source.chunks){std::array<double,6> box{1e300,1e300,1e300,-1e300,-1e300,-1e300};
    for(const auto&v:c.vertices)for(int a=0;a<3;++a){box[a]=std::min(box[a],double(v.surface[a]));box[3+a]=std::max(box[3+a],double(v.surface[a]));}i.spare_static_boxes.push_back(box);}}
  std::size_t staticIndex=0;
  auto removedNear=[&](const gfy_packed::Child&c){double lo[3]={1e300,1e300,1e300},hi[3]={-1e300,-1e300,-1e300};
   if(prevMesh){const auto&p=prevMesh->children[staticIndex];need(p.child_index==c.child_index,"static_child_order");if(p.skin_clip)return true;
    for(int a=0;a<3;++a){lo[a]=i.spare_static_boxes[staticIndex][a];hi[a]=i.spare_static_boxes[staticIndex][3+a];}}
   else for(const auto&v:c.vertices)for(int a=0;a<3;++a){lo[a]=std::min(lo[a],double(v.surface[a]));hi[a]=std::max(hi[a],double(v.surface[a]));}
   for(const auto&b:removedTiles){bool overlap=true;for(int a=0;a<3&&overlap;++a)overlap=b[a]<=hi[a]+grid.pitch&&b[3+a]>=lo[a]-grid.pitch;if(overlap)return true;}return false;};
#ifdef GFY_RX_PROF
  double copyMs=0,nearMs=0;std::size_t staticChunks=0,staticVerts=0,staticIdx=0,clipped=0;
  for(const auto&source:i.asset->fast->static_child)for(std::size_t part=0;part<source.chunks.size();++part){auto c0=core::Clock::now();auto child=source.chunks[part];copyMs+=core::ms(c0);
   if(part){need(next<gfy_packed::max_child_slots,"analytic_child_capacity");child.child_index=std::uint16_t(next++);}c0=core::Clock::now();child.skin_clip=removedNear(child);++staticIndex;nearMs+=core::ms(c0);
   ++staticChunks;staticVerts+=child.vertices.size();staticIdx+=child.indices.size();clipped+=child.skin_clip;mesh.children.push_back(std::move(child));}
  std::cerr<<"rxprof pack.static_copy "<<copyMs<<std::endl<<"rxprof pack.static_near "<<nearMs<<std::endl;
  std::cerr<<"rxsize pack static_chunks="<<staticChunks<<" static_vertices="<<staticVerts<<" static_indices="<<staticIdx<<" clipped="<<clipped<<" removed_tiles="<<removedTiles.size()<<" grid_tiles="<<grid.tiles.size()<<" vertex_bytes="<<sizeof(gfy_packed::Vertex)<<" mask_bytes="<<mask->bytes()<<" mixed="<<mask->mixed<<" uniform="<<mask->uniform<<std::endl;
  packT=core::Clock::now();
#else
  for(const auto&source:i.asset->fast->static_child)for(std::size_t part=0;part<source.chunks.size();++part){auto child=source.chunks[part];
   if(part){need(next<gfy_packed::max_child_slots,"analytic_child_capacity");child.child_index=std::uint16_t(next++);}child.skin_clip=removedNear(child);++staticIndex;mesh.children.push_back(std::move(child));}
#endif
  painter_detail::ChannelIdentity identity;identity.source=i.asset->source_hash;identity.rig=i.asset->rig_hash;identity.cookie=i.owner.spawn_cookie;identity.lineage=i.owner.visual_lineage;identity.topology=i.owner.topology;
  identity.generation=out->stamp.expected_generation;identity.revision=out->stamp.revision;identity.snapshot=history;
  painter_detail::ChannelFieldBuilder builder(identity,history->limitations);std::vector<const ResolvedCutHistory*>nodes;
  if(true)builder.reserveCuts(history->total_cuts);
  for(auto p=history;p;p=p->parent)nodes.push_back(p.get());std::reverse(nodes.begin(),nodes.end());
  for(auto node:nodes)for(const auto&record:node->appended){auto resolved=record;
   if(!resolved.edge&&!resolved.cracks){channelworker::ResolvedView view(resolved);resolved.edge_u={view.edge_u.x,view.edge_u.y,view.edge_u.z};resolved.edge_v={view.edge_v.x,view.edge_v.y,view.edge_v.z};}builder.append(resolved);}
  packMark("history_builder");
#ifdef GFY_RX_PROF
  std::size_t chanVerts=0,chanIdx=0,chanOwners=0;for(const auto&packed:prepared.state->children){chanVerts+=packed->geometry.vertices.size();chanIdx+=packed->geometry.indices.size();chanOwners+=packed->owners.size();}
  std::cerr<<"rxsize pack channel_children="<<prepared.state->children.size()<<" channel_vertices="<<chanVerts<<" channel_indices="<<chanIdx<<" owners="<<chanOwners<<" history_cuts="<<history->total_cuts<<" history_nodes="<<nodes.size()<<" planned_cuts="<<prepared.stats.planned_cuts<<" plan_ms="<<prepared.stats.plan_ms<<" weight_ms="<<prepared.stats.weight_ms<<" split_ms="<<prepared.stats.split_ms<<" split_added="<<prepared.stats.split_added<<" split_cut_edges="<<prepared.stats.split_cut_edges<<" split_by_ends="<<prepared.stats.split_by_ends<<" split_by_mid="<<prepared.stats.split_by_mid<<" split_rounds="<<prepared.stats.split_rounds<<" split_deep="<<prepared.stats.split_deep<<" split_hidden="<<prepared.stats.split_hidden<<" split_sample_ms="<<prepared.stats.split_sample_ms<<" split_cut_ms="<<prepared.stats.split_cut_ms<<" split_eval_ms="<<prepared.stats.split_eval_ms<<" packch_ms="<<prepared.stats.pack_ms<<" append_ms="<<prepared.stats.append_ms<<" copied_tail_rows="<<prepared.stats.copied_tail_rows<<std::endl;
  packT=core::Clock::now();
#endif
  for(const auto&packed:prepared.state->children){need(next<gfy_packed::max_child_slots,"analytic_child_capacity");auto child=packed->geometry;child.child_index=std::uint16_t(next++);builder.addChild(child.child_index,packed->owners);mesh.children.push_back(std::move(child));}
  packMark("channel_children");
  static_assert(unsigned(resolved_cut_edge_repairs)==painter_detail::AnalyticChannelField::renderable_limitations,"renderable history limitations");
  mesh.analytic_channels=builder.finish();mesh.removal_field=std::move(mask);mesh.tissue_field=i.asset->closure_field;
  packMark("builder_finish");
  // Producer coverage proof: walls sample the closure field (legacy closed-envelope
  // closure, checked on actual repaired masks).
  mesh.analytic_coverage_admitted=bool(mesh.tissue_field);
  out->mesh=std::move(mesh);out->triangles=0;for(const auto&c:out->mesh.children)out->triangles+=c.indices.size()/3;
  out->analytic_diagnostic=true;out->analytic_coverage_requirements=prepared.state->coverage_requirements;
  out->analytic_new_cuts=prepared.stats.planned_cuts;out->analytic_new_triangles=prepared.stats.planned_triangles;out->analytic_cavity_triangles=prepared.state->triangles;
  out->analytic_plan_ms=prepared.stats.plan_ms;out->analytic_weight_ms=prepared.stats.weight_ms;out->analytic_pack_ms=prepared.stats.pack_ms;out->analytic_append_ms=prepared.stats.append_ms;
  candidate=prepared.state;
 }
 // Mask-only wounds (painter_voxel_mask_only 1, intact whole body): the static exterior children with skin_clip and the
 // removal field, nothing else - no channel planning, weights, wall refinement, channel packets or field builder. Same
 // children, child indices and skin_clip rule as packAnalyticCandidate's static part.
 void packMaskOnlyCandidate(Session::Impl&i,const Grid&grid,const std::shared_ptr<Output>&out,WorkerPhaseTimes&phases,const std::set<Cell>*dirty){
  PhaseTime whole(phases.pack_ms);
  const auto maskStart=core::Clock::now();auto mask=painter_detail::buildDamageField(grid);
  need(mask&&mask->available,"mask_only_removal_field_unavailable");out->analytic_mask_ms=core::ms(maskStart);
  gfy_packed::Mesh mesh;unsigned next=i.asset->original_children;
  const gfy_packed::Mesh* prevMesh=nullptr;std::size_t staticTotal=0;
  for(const auto&source:i.asset->fast->static_child)staticTotal+=source.chunks.size();
  if(dirty&&!i.piece&&!grid.ownership&&i.committed->impl_->geometry){const auto&pm=i.committed->impl_->geometry->mesh;
   if((pm.analytic_channels||pm.mask_volume)&&pm.children.size()>=staticTotal)prevMesh=&pm;}
  std::vector<std::array<double,6>> removedTiles;
  if(!prevMesh){removedTiles.reserve(grid.tiles.size());
   for(const auto&t:grid.tiles){bool any=false;for(auto w:t.second.removed)any|=w!=0;if(!any)continue;std::array<double,6>b;
    for(int a=0;a<3;++a){b[a]=grid.origin[a]+double((&t.first.x)[a])*8*grid.pitch;b[3+a]=b[a]+8*grid.pitch;}removedTiles.push_back(b);}}
  else for(const Cell&k:*dirty){auto it=grid.tiles.find(k);if(it==grid.tiles.end())continue;bool any=false;for(auto w:it->second.removed)any|=w!=0;if(!any)continue;std::array<double,6>b;
    for(int a=0;a<3;++a){b[a]=grid.origin[a]+double((&k.x)[a])*8*grid.pitch;b[3+a]=b[a]+8*grid.pitch;}removedTiles.push_back(b);}
  if(prevMesh&&i.spare_static_boxes.size()!=staticTotal){i.spare_static_boxes.clear();
   for(const auto&source:i.asset->fast->static_child)for(const auto&c:source.chunks){std::array<double,6> box{1e300,1e300,1e300,-1e300,-1e300,-1e300};
    for(const auto&v:c.vertices)for(int a=0;a<3;++a){box[a]=std::min(box[a],double(v.surface[a]));box[3+a]=std::max(box[3+a],double(v.surface[a]));}i.spare_static_boxes.push_back(box);}}
  // melee (value 3): a child within a dent draws with the mask material too (its exterior takes the dent vertex program)
  if(i.job_dents_on)for(const auto&d:i.job_dents){std::array<double,6>b;for(int a=0;a<3;++a){b[a]=double(d.centre[a])-d.radius;b[3+a]=double(d.centre[a])+d.radius;}removedTiles.push_back(b);}
  std::size_t staticIndex=0;
  auto removedNear=[&](const gfy_packed::Child&c){double lo[3]={1e300,1e300,1e300},hi[3]={-1e300,-1e300,-1e300};
   if(prevMesh){const auto&p=prevMesh->children[staticIndex];need(p.child_index==c.child_index,"static_child_order");if(p.skin_clip)return true;
    for(int a=0;a<3;++a){lo[a]=i.spare_static_boxes[staticIndex][a];hi[a]=i.spare_static_boxes[staticIndex][3+a];}}
   else for(const auto&v:c.vertices)for(int a=0;a<3;++a){lo[a]=std::min(lo[a],double(v.surface[a]));hi[a]=std::max(hi[a],double(v.surface[a]));}
   for(const auto&b:removedTiles){bool overlap=true;for(int a=0;a<3&&overlap;++a)overlap=b[a]<=hi[a]+grid.pitch&&b[3+a]>=lo[a]-grid.pitch;if(overlap)return true;}return false;};
  for(const auto&source:i.asset->fast->static_child)for(std::size_t part=0;part<source.chunks.size();++part){auto child=source.chunks[part];
   if(part){need(next<gfy_packed::max_child_slots,"analytic_child_capacity");child.child_index=std::uint16_t(next++);}child.skin_clip=removedNear(child);++staticIndex;mesh.children.push_back(std::move(child));}
  mesh.removal_field=std::move(mask);mesh.tissue_field=i.asset->closure_field;mesh.mask_volume=true;
  out->mesh=std::move(mesh);out->triangles=0;for(const auto&c:out->mesh.children)out->triangles+=c.indices.size()/3;
 }
 void runJob(Work&task,JobMemory&memory){auto&i=*task.session->impl_;
  const tear_bake::JobKeep bakeKeep(i.asset?std::shared_ptr<const void>(i.asset->closure_field):nullptr); // speed step: the job's tear bakes may outlive it
  const auto start=core::Clock::now(); // Include cache reconstruction in worker_ms.
#ifdef GFY_RX_PROF
  auto jobT=core::Clock::now();auto jobMark=[&](const char*n){std::cerr<<"rxprof job."<<n<<' '<<core::ms(jobT)<<std::endl;jobT=core::Clock::now();};
#else
  auto jobMark=[](const char*){};
#endif
  // Governor-evicted caches come back first (worker-only while in flight).
  if(!i.tile_state)i.tile_state=std::make_shared<fastwound::TileRegionState>();
  if(!i.tile_faces)i.tile_faces=std::make_shared<fastwound::TileFaceMemo>();
  if(i.boundary_rebuild){const auto&committedGrid=*i.committed->impl_->grid;
   const bool inc=!i.asset->face_children.empty()&&!committedGrid.ownership&&i.asset->fast&&i.asset->fast->valid;
   if(inc)i.boundary_valid=false; // an incremental session rebuilds only when a non-direct attempt needs the cache
   else i.boundary_cache=rebuildBoundaryCache(committedGrid,false);
   i.boundary_rebuild=false;}
  auto out=std::make_shared<Output>();out->stamp=task.stamp;out->batch_events=task.events.size();
  bool geometryRejected=false;std::string geometryRejection;
  auto rejectGeometry=[&](const char* why){
   geometryRejected=true;geometryRejection=why;
   // The candidate is transactional. Preserve the last published geometry and
   // occupancy, acknowledge this rejected batch, and let later hits proceed.
   out->mesh={};out->joints.clear();out->triangles=0;out->no_geometry_change=true;out->tiles=i.committed->sparse_tiles();out->dirty_tiles=0;
   for(auto&impact:out->impacts){
    // Receipt admission already excludes rejected/no-removal events. Still
    // discard every uncommitted effect payload, including sever, tissue
    // volume, exit and vent data; future fields get safe defaults as well.
    const auto trace=impact.hit_trace;const auto sequence=impact.sequence;const bool miss=impact.miss;const auto gore=impact.gore;
    const auto explosion_world=impact.explosion_world,explosion_cookie=impact.explosion_cookie,
        explosion_target_birth=impact.explosion_target_birth,explosion_member_token=impact.explosion_member_token;
    // Retain mapping for impact_replay diagnostics; none of these admit effects.
    const auto restEntry=impact.rest_entry,restDirection=impact.rest_direction,cutter=impact.cutter;
    const auto sourceFace=impact.source_face;const auto barycentric=impact.barycentric;const auto radius=impact.radius;
    auto rejection=impact.removed?std::string("geometry_rejected: ")+why:std::move(impact.rejection);
    impact=ImpactResult{};impact.hit_trace=trace;impact.sequence=sequence;impact.miss=miss;impact.gore=gore;impact.rejection=std::move(rejection);
    impact.explosion_world=explosion_world;impact.explosion_cookie=explosion_cookie;
    impact.explosion_target_birth=explosion_target_birth;impact.explosion_member_token=explosion_member_token;
    impact.rest_entry=restEntry;impact.rest_direction=restDirection;impact.cutter=cutter;
    impact.source_face=sourceFace;impact.barycentric=barycentric;impact.radius=radius;
   }
  };
  std::shared_ptr<const ResolvedCutHistory>candidate_history;
  std::shared_ptr<const channelworker::State>candidate_channels;
  std::shared_ptr<SeverState>candidate_sever;
  std::shared_ptr<const Grid>candidate_grid;fastwound::BoundaryCache candidate_boundary;fastwound::RegionList candidate_regions;
  std::uint64_t candidate_spare_serial=0;std::shared_ptr<const std::vector<Cell>> candidate_spare_dirty;
  std::shared_ptr<core::ExteriorMemo>candidate_memo;
  std::shared_ptr<const DamageSnapshot>candidate_snapshot;
  std::string retryReason,batchReason;WorkerPhaseTimes workerPhases;std::size_t activeEvents=task.events.size(),batchSplits=0;
  for(;;){
  geometryRejected=false;geometryRejection.clear();retryReason.clear();candidate_channels.reset();candidate_history.reset();candidate_sever.reset();candidate_grid.reset();candidate_boundary.clear();candidate_regions.clear();candidate_memo.reset();
  candidate_spare_serial=0;candidate_spare_dirty.reset();
  *out=Output{};task.stamp.revision=task.events[activeEvents-1].sequence;out->stamp=task.stamp;out->batch_events=activeEvents;
  for(unsigned attempt=0;attempt<2;++attempt){bool usedRough=false,retrySmooth=false;
  jobMark("pre");
  try {auto attemptSever=std::make_shared<SeverState>(*i.sever_state);jobMark("sever_copy");core::StageProfile stages("worker");PhaseTime removalTime(workerPhases.removal_ms);
   Grid grid;bool recycled=false;
   if(true&&!i.piece&&i.spare_committed_serial){std::unique_ptr<Grid> spare;std::uint64_t spareSerial=0;
    {std::lock_guard<std::mutex> lock(i.spare_spare->mutex);spare=std::move(i.spare_spare->grid);spareSerial=i.spare_spare->serial;}
    std::vector<std::shared_ptr<const std::vector<Cell>>> later;bool found=false;
    if(spare){std::lock_guard<std::mutex> lock(i.mutex);
     if(!i.spare_deltas.empty()&&i.spare_deltas.back().first==i.spare_committed_serial)
      for(const auto&d:i.spare_deltas){if(found)later.push_back(d.second);else if(d.first==spareSerial)found=true;}}
    if(found&&!later.empty()){const Grid&committedGrid=*i.committed->impl_->grid;
     for(const auto&keys:later)for(const Cell&k:*keys){auto c=committedGrid.tiles.find(k);
      if(c!=committedGrid.tiles.end())spare->tiles[k]=c->second;else spare->tiles.erase(k);}
     grid=std::move(*spare);recycled=true;}
    else if(spare)retire(std::shared_ptr<Grid>(spare.release()));} // stale spare: freed off the worker, as before
   if(!recycled)grid=*i.committed->impl_->grid;
   jobMark(recycled?"grid_recycle":"grid_copy");auto boundary_cache=i.boundary_cache;std::set<Cell>dirty;bool changed=false;jobMark("boundary_copy");
   const auto historyBase=i.committed->impl_->history;candidate_channels=i.channel_cache;
   std::vector<ResolvedTunnel>historyCuts;std::uint32_t historyLimits=historyBase?historyBase->limitations:0;
   auto historyRoom=[&]{return historyBase&&(!grid.ownership||i.piece)&&historyBase->total_cuts+historyCuts.size()<ResolvedCutHistory::max_cuts;};
   auto remember=[&](ResolvedTunnel&&cut,std::uint64_t sequence,const Mapping&mapping,ResolvedTunnel::Role role){
    cut.sequence=sequence;cut.source_face=mapping.face;cut.source_bary=mapping.bary;cut.role=role;historyCuts.push_back(std::move(cut));};
   PoseCache& poseCache=i.pose_cache; // posed once per distinct palette, kept across jobs
   std::set<Cell> blastDirty;std::vector<Cell> blastDirtyList;Mapping blastMapping;std::uint64_t blastSequence=0;std::set<std::uint64_t> blastSequences; std::vector<std::pair<Vec,Vec>> blastRays; // headfix item 5: each crater's ray (rest space)
   // Slabs carry their real planes even when there is only one per part.
   // Their tear check may prove an existing cut, never invent another cut.
   const bool bladeSlabBatch=std::any_of(task.events.begin(),task.events.begin()+activeEvents,
    [](const Impact&e){return e.blast&&e.blade&&e.slab;});
   std::vector<TearPlane> slabPlanes;
   struct BladeSlot {Vec a,b,dir;unsigned n=0;};std::map<std::uint32_t,BladeSlot> bladeSlots; // grenade blades of this batch (rest space)
   struct TearRequest {std::uint16_t bone;Vec point,push;std::uint64_t sequence,seed;std::size_t result;};std::vector<TearRequest> tearRequests; // R62 BlastSevers.h
   // raggedsever: the killing shots' limb tears of this batch, made after its events (below the event loop)
   struct KillSever {std::size_t result;std::uint64_t sequence;Mapping mapping;Vec e0,d;double penetration;std::uint16_t hitBone;std::uint64_t leanSeed;};std::vector<KillSever> killSevers;
   bool bulletBurstBatch=false; // bulletburst: one body burst or blowout per batch (a shotgun's pellets draw until one fires)
   // melee (value 3): the body's dents for this attempt = the committed mesh's; a blunt blow below adds its own
   i.job_dents_on=mask_only_mode.load(std::memory_order_relaxed)==3;i.job_dents.clear();i.job_slits.clear();bool dentsAdded=false;
   const int dmode=damage_mode_value();const bool holesMode=dmode==1;static const bool modesLog=std::getenv("GFY_MODES_LOG")!=nullptr; // modes: this job's Damage mode
   if(i.job_dents_on&&i.committed)if(const auto*cm=i.committed->mesh()){i.job_dents=cm->dents;i.job_slits=cm->slits;}
   // R69: the source rays side by side before the loop (they never read the grid); one palette for every ray event: the pose
   // cache asked once, as the first event's sourceRay asks it; an event's stored mapping or failure is taken in the loop
   std::vector<char> rayReady;std::vector<Mapping> rayMapping;std::vector<std::exception_ptr> rayError;
   {static const bool gearOccludes=std::getenv("GFY_GEAR_OCCLUDES")!=nullptr;std::vector<std::size_t> rayEvents;
    try{for(std::size_t e=0;e<activeEvents;++e)if(!task.events[e].tear_request)rayEvents.push_back(e);
     if(rayEvents.size()>=2&&!gearOccludes){const auto&p0=task.events[rayEvents[0]].bone_model_render;bool same=p0.size()==i.asset->source.bones;
     for(std::size_t k=1;same&&k<rayEvents.size();++k){const auto&pk=task.events[rayEvents[k]].bone_model_render;same=pk.size()==p0.size();
      for(std::size_t q=0;same&&q<pk.size();++q)same=pk[q].row_major==p0[q].row_major;}
     if(same){rayReady.assign(activeEvents,0);rayMapping.resize(activeEvents);rayError.resize(activeEvents);
      const auto&posed=poseCache.get(i.asset->source,p0,false);
      core::parallelChunks(rayEvents.size(),1,[&](std::size_t a,std::size_t b,std::size_t){
       struct InlineOnly{bool previous;InlineOnly():previous(core::painterInlineParallel){core::painterInlineParallel=true;}~InlineOnly(){core::painterInlineParallel=previous;}} inlineOnly; // each ray's face loop on its own thread
       for(std::size_t k=a;k<b;++k){const auto e=rayEvents[k];
       try{rayMapping[e]=sourceRay(i.asset->source,task.events[e],nullptr,i.piece?&i.piece->faces:nullptr,i.piece_ownership.get(),&i.asset->face_children,&i.asset->face_indices,i.asset->source_child,&posed);rayReady[e]=1;}
       catch(const std::bad_alloc&){} // left to the loop's serial sourceRay
       catch(...){rayError[e]=std::current_exception();rayReady[e]=1;}}});}}}
    catch(const std::bad_alloc&){rayReady.clear();rayMapping.clear();rayError.clear();}}
   for(std::size_t eventIndex=0;eventIndex<activeEvents;++eventIndex){const auto&eventIn=task.events[eventIndex];
    // modes: Extreme cuts with a copy whose damage sizes are doubled (damage_modes::doubleSizes); the decisions that are chances
    // or sever thresholds (head event, sever scale, killing-shot round) keep reading eventIn. Bullet holes only: holeMarch,
    // holeCapEnd, holeHold and holeThin are set where the event is cut.
    Impact eventX;const bool sizedX=dmode==2&&!eventIn.tear_request;if(sizedX){eventX=eventIn;damage_modes::doubleSizes(eventX);
     if(modesLog)std::cerr<<"modes_extreme seq="<<eventIn.sequence<<" radius_mm="<<eventIn.radius*1e3<<"->"<<eventX.radius*1e3<<" depth_mm="<<eventIn.penetration_m*1e3<<"->"<<eventX.penetration_m*1e3
      <<" half_width_mm="<<eventIn.slab_half_width*1e3<<"->"<<eventX.slab_half_width*1e3<<" dent_mm="<<eventIn.dent_radius*1e3<<"/"<<eventIn.dent_depth*1e3<<"->"<<eventX.dent_radius*1e3<<"/"<<eventX.dent_depth*1e3<<std::endl;}
    const auto&event=sizedX?eventX:eventIn;March holeMarch;double holeCapEnd=1e9,holeThick=-1;bool holeHold=false,holeThin=false;
    ImpactResult result;result.hit_trace=event.hit_trace;result.sequence=event.sequence;result.gore=event.gore;Mapping mapping;
    result.explosion_world=event.explosion_world;result.explosion_cookie=event.explosion_cookie;
    result.explosion_target_birth=event.explosion_target_birth;result.explosion_member_token=event.explosion_member_token;
    if(event.tear_request&&event.tear_survived){result.rejection="tear_request_survived";out->impacts.push_back(std::move(result));continue;} // legacy rule
    if(event.tear_request){ // R62: a blast's forced tear (BlastSevers.h), made after the batch's craters; it carves nothing itself
     result.ammo_section=event.ammo_section;result.hit_bone=event.hit_bone;result.hit_power=event.hit_power;result.initiator=event.initiator;
     try{const Mapping bone=piece_session::boneRay(event);
      tearRequests.push_back({event.hit_bone,bone.entry,bone.direction,event.sequence,event.ammo_seed?event.ammo_seed:event.sequence,out->impacts.size()});}
     catch(const std::bad_alloc&){throw;}catch(const std::exception&){result.rejection="tear_request_bone";}
     out->impacts.push_back(std::move(result));continue;}
#ifdef GFY_RX_PROF
    auto evStart=core::Clock::now();auto evMark=[&](const char*n){std::cerr<<"rxprof event."<<n<<' '<<core::ms(evStart)<<std::endl;evStart=core::Clock::now();};
#else
    auto evMark=[](const char*){};
#endif
    try {if(!rayReady.empty()&&rayReady[eventIndex]){if(rayError[eventIndex])std::rethrow_exception(rayError[eventIndex]);mapping=rayMapping[eventIndex];}
     else mapping=sourceRay(i.asset->source,event,&poseCache,i.piece?&i.piece->faces:nullptr,i.piece_ownership.get(),&i.asset->face_children,&i.asset->face_indices,i.asset->source_child);}
    catch(const std::bad_alloc&){throw;}
    catch(const std::exception& e){bool mapped=false;
     if(i.piece&&!std::strcmp(e.what(),"no_source_surface_mapping"))try{mapping=piece_session::boneRay(event);mapped=true;}catch(const std::bad_alloc&){throw;}catch(const std::exception&){}
     if(!mapped){if(!recoverableMapping(e.what()))throw;result.rejection=e.what();out->impacts.push_back(std::move(result));continue;}}
    evMark("sourceRay");
    if(event.trace_gear)gearBodyTrace(*i.asset,event,mapping,poseCache,result); // gearbody: where a shot through merged gear was mapped (verbose)
    // Slab (R18): the sheet's plane axes follow the same skin blend as the direction, then are squared up against it.
    SlabSpec slabSpec{};const bool slabEvent=event.slab&&event.penetration_m>0;
    if(slabEvent){bool ok=true;
     try{Vec n=unit(inverseDirection(mapping.blend,unit(vec(event.slab_normal))));const Vec d=unit(mapping.direction);
      n=n-d*dot(n,d);ok=dot(n,n)>=1e-10;if(ok){n=unit(n);slabSpec={n,cross(d,n),event.radius,event.slab_half_width,event.ragged_slab?(event.ammo_seed?event.ammo_seed:std::max(std::uint64_t(1),event.sequence)):0};}}
     catch(const std::bad_alloc&){throw;}catch(const std::exception&){ok=false;}
     if(!ok){result.rejection="slab_axes";out->impacts.push_back(std::move(result));continue;}}
    // melee (value 3 only, MeleeImpact.h): a mode switch between the hit and its job leaves a melee event uncut
    if(event.melee&&mask_only_mode.load(std::memory_order_relaxed)!=3){result.rejection="melee_value3_only";out->impacts.push_back(std::move(result));continue;}
    // a blade blow that cut the part through: its gape slabs are not cut (the two clean faces meet at rest, skin to skin)
    if(event.melee_part&&event.melee_blow&&melee::blade(event.melee)&&std::find(std::begin(i.melee_severed),std::end(i.melee_severed),event.melee_blow)!=std::end(i.melee_severed)){
     result.rejection="melee_part_of_a_severed_blow";out->impacts.push_back(std::move(result));continue;}
    // a blunt blow's dent: the source point it struck (rest space), pushed along its travel; a dent alone carves nothing
    if(event.melee==melee::blunt&&i.job_dents_on&&event.dent_radius>.005&&event.dent_depth>.001){gfy_packed::Dent dent;const Vec dd=unit(mapping.direction);
     for(int a=0;a<3;++a){dent.centre[a]=float(mapping.entry[a]);dent.push[a]=float(dd[a]);}dent.radius=float(event.dent_radius);dent.depth=float(event.dent_depth);
     if(holesMode)dent.depth=std::min(dent.depth,float(damage_modes::holeDepth)); // modes: no dent deeper than the cap
     melee::addDent(i.job_dents,dent,holesMode?damage_modes::holeDepth:.05);dentsAdded=true;
     if(!(event.penetration_m>0)){result.rejection="melee_dent_only";result.rest_entry=point(mapping.entry);result.rest_direction=point(dd);
      result.source_face=mapping.face;result.barycentric=mapping.bary;out->impacts.push_back(std::move(result));continue;}}
    if(std::getenv("GFY_GEAR_OCCLUDES")&&occluded(i.asset->occlusion,event,mapping.posed_distance,&poseCache)){result.rejection="frontmost_surface_is_other_child";out->impacts.push_back(std::move(result));continue;}
    bool inside=true;for(int axis=0;axis<3;++axis)inside=inside&&mapping.entry[axis]>=grid.origin[axis]-grid.pitch&&mapping.entry[axis]<=grid.origin[axis]+grid.size[axis]*grid.pitch+grid.pitch;
    if(!inside){result.rejection="source_hit_outside_profile_grid";out->impacts.push_back(std::move(result));continue;}
    // Start outside the wound footprint, not inside its voxel approximation.
    // Legacy prepared sphere cuts retain their original origin and hashes.
    Vec origin=mapping.entry-mapping.direction*(grid.pitch*2+(event.penetration_m>0?event.radius:0));const bool meleeEdge=melee::blade(event.melee)&&!event.melee_part&&event.penetration_m>0; // melee: down an existing cut to its bottom
    auto march=event.penetration_m>0&&!meleeEdge?firstSolidCylinder(grid,origin,mapping.direction,event.radius):firstSolid(grid,origin,mapping.direction);
    // The primary tunnel's march equals this one when unit() is a fixed point of the direction (eraseTunnel normalises once
    // more before marching): then it is passed in instead of marched again.
    const March* marchReuse=nullptr;
    {const Vec d1=unit(mapping.direction),d2=unit(d1),d3=unit(d2);if(event.penetration_m>0&&d1.x==d2.x&&d1.y==d2.y&&d1.z==d2.z&&d2.x==d3.x&&d2.y==d3.y&&d2.z==d3.z)marchReuse=&march;}
    if(meleeEdge)marchReuse=&march; // melee: the blade's tunnel starts where its centre line met solid tissue
    result.source_face=mapping.face;result.barycentric=mapping.bary;result.rest_direction=point(mapping.direction);result.rest_entry=point(march.found?march.entry:mapping.entry);result.miss=!march.found;
    if(march.found){
     core::HitResult removal;
     TunnelExit exit,ventExit;double ventRadiusOut=0,entryMouth=0;
     unsigned bulletKind=0;Vec bulletPoint{0,0,0},bulletMouthAt{0,0,0},bulletDir{0,0,1};double bulletStrength=0,bulletMouth=0;bool bulletMouthExit=false; // bulletburst: this shot's burst / blowout
     // Cut in place with an undo journal (was: a full grid copy per event, O(total damage)).
     std::unordered_map<Cell,std::pair<bool,core::Tile>,core::CellHash> journal;
     auto priorSever=attemptSever->failed;
     const auto priorHistoryCount=historyCuts.size();const auto priorHistoryLimits=historyLimits;
     auto rollback=[&]{historyCuts.resize(priorHistoryCount);historyLimits=priorHistoryLimits;attemptSever->failed.swap(priorSever);for(auto& e:journal){if(e.second.first)grid.tiles[e.first]=e.second.second;else grid.tiles.erase(e.first);}grid.journal=nullptr;};
     const char* nothingWhy=nullptr; // nothinghits: why a bullet removed nothing (passed_through_hole, no_anatomy_in_cut, crater_bottom_not_reached)
     try {Grid& candidate=grid;candidate.journal=&journal;evMark("copy");
      // Replays of the same accepted shot keep the same connected irregular
      // edge. Legacy sphere/profile probes retain their original footprint.
      // Per-shot seed from the engine (bullet, sequence, bone, entity): two NPCs' first hits must not share an edge shape
      // or crack pattern. Offline probes leave ammo_seed 0 and keep the per-session sequence.
      const std::uint64_t edgeSeed=attempt==0&&event.penetration_m>0&&!i.asset->face_children.empty()&&!event.ammo_section.empty()?(event.ammo_seed?event.ammo_seed:event.sequence):0;
      ResolvedTunnel primaryRecord;bool primaryWritten=false;const bool recordPrimary=event.penetration_m>0&&historyRoom();
      double entryPenetration=event.penetration_m,craterDepth=0;Vec craterPoint{};bool closedExit=false;
      if(event.penetration_m>0&&event.burst<=0&&std::getenv("GFY_CLOSED_PASS_THROUGH")){
       const auto crossings=i.asset->envelope_tree.crossingDistances(march.entry,unit(mapping.direction));
       const std::size_t exitIndex=crossings.size()%2?0:1;
       if(exitIndex<crossings.size()&&event.penetration_m>=crossings[exitIndex]){const double path=crossings[exitIndex];
        const double plug=std::max(.006,.2*path),open=path-plug;
        if(path>=.12&&open>event.radius*2){closedExit=true;entryPenetration=std::max(event.radius*1.5,open*.55);craterDepth=std::max(event.radius*1.5,open*.45);
         craterPoint=march.entry+unit(mapping.direction)*path;}}}
      bool chewed=false;
      if(event.living_hit&&event.penetration_m>0&&!event.blast&&!slabEvent&&!std::getenv("GFY_LIVING_FULL_CHANNELS")){
       const double chew=std::clamp(2.5*event.radius,.015,.035),exitChew=std::clamp(2.*event.radius,.012,.03);
       const auto crossings=i.asset->envelope_tree.crossingDistances(march.entry,unit(mapping.direction));
       const std::size_t exitIndex=crossings.size()%2?0:1;
       if(exitIndex<crossings.size()&&event.penetration_m>=crossings[exitIndex]){const double path=crossings[exitIndex];
        if(path>chew+exitChew+.01){closedExit=true;entryPenetration=chew;craterDepth=exitChew;craterPoint=march.entry+unit(mapping.direction)*path;chewed=true;}
        else if(path>chew){entryPenetration=std::min(entryPenetration,chew);chewed=true;}}
       else if(entryPenetration>chew){entryPenetration=chew;chewed=true;}}
      if(holesMode&&event.penetration_m>0){
       const Vec hd=unit(mapping.direction);const Grid&base=i.asset->base;
       // the SHALLOWEST original surface under the cut's footprint (axis + rings at 0.5, 1 and 1.5 x its radius, a blade: its half
       // width): the cut's far end then lies at most the cap below every part of the surface it opens
       auto shallowest=[&](const Vec&from,const Vec&dir,double radius){double best=1e9;const Vec u=unit(cross(dir,std::abs(dir.x)<.8?Vec{1,0,0}:Vec{0,1,0})),v=cross(dir,u);
        for(int k=-1;k<36;++k){const double a=.5235987755982988*(k<0?0:k%12),f=k<0?0.:(k<12?.5:(k<24?1.:1.5));
         const March m=firstSolid(base,from+(u*std::cos(a)+v*std::sin(a))*(radius*f),dir);const double tm=m.found?dot(m.entry-from,dir):-1.;if(tm>1e-4)best=std::min(best,tm);} // (a ray that starts inside another part is not a surface this cut opens)
        return best;};
       const double here=dot(march.entry-origin,hd),surf=std::min(shallowest(origin,hd,std::max(event.radius,slabEvent?event.slab_half_width:0.)),here);
       // the part's thickness on the shot's axis: the original tissue from its surface to the last tissue before 2 cm of air
       const March axis0=firstSolid(base,origin,hd);double thick=1e9,axisSurf=surf;
       if(axis0.found){axisSurf=dot(axis0.entry-origin,hd);double last=axisSurf;
        for(double t=axisSurf;t<=axisSurf+.5;t+=grid.pitch*.5){const Cell c=piece_session::cellOf(base,origin+hd*t);if(base.valid(c)&&base.solidBase(c))last=t;else if(t-last>.02)break;}
        thick=last-axisSurf+grid.pitch*.5;holeThick=thick;}
       const bool roundLike=!event.blast&&!event.melee&&!slabEvent;
       holeThin=roundLike&&axis0.found&&thick<=damage_modes::thinPart&&event.penetration_m>=thick;
       if(!holeThin){
        closedExit=false;craterDepth=0;chewed=true;holeCapEnd=surf+damage_modes::holeDepth;
        const double reach=std::min(here+event.penetration_m,holeCapEnd);
        if(reach-here>=.0005)entryPenetration=reach-here;
        else{holeMarch=march;holeMarch.found=true;holeMarch.entry=origin+hd*(holeCapEnd-.0005);marchReuse=&holeMarch;entryPenetration=.0005;holeHold=true;}
        if(roundLike&&event.penetration_m>=thick&&thick<1e8){ // the round would have left the body: its exit crater from the far skin
         const double craterRadius=std::clamp(event.radius*event.exit_diameter_mult,.0005,.08);
         craterPoint=origin+hd*(axisSurf+thick);const Vec back=hd*-1.,craterOrigin=craterPoint+hd*(grid.pitch*2+craterRadius);
         const March farHere=firstSolidCylinder(candidate,craterOrigin,back,craterRadius);
         if(farHere.found){const double farHereD=dot(farHere.entry-craterOrigin,back),farSurf=std::min(shallowest(craterOrigin,back,craterRadius),farHereD);
          closedExit=true;craterDepth=std::min(std::min(std::clamp(2.*event.radius,.012,.03),damage_modes::holeDepth),farSurf+damage_modes::holeDepth-farHereD);
          if(craterDepth<.0005)craterDepth=0;}} // at the cap there: the exit's blood, no deeper crater
       }
       if(modesLog)std::cerr<<"modes_hole seq="<<event.sequence<<" thin="<<holeThin<<" surface_mm="<<surf*1e3<<" start_mm="<<here*1e3<<" thick_mm="<<(thick<1e8?thick*1e3:-1.)
        <<" entry_mm="<<entryPenetration*1e3<<" cut_end_below_surface_mm="<<(holeThin?-1.:((holeHold?holeCapEnd:here+entryPenetration)-surf)*1e3)<<" hold="<<holeHold
        <<" exit_crater_mm="<<(closedExit?craterDepth*1e3:-1.)<<std::endl;}
      removal=event.penetration_m>0 ? eraseTunnel(candidate,origin,mapping.direction,event.radius,entryPenetration,&result.edge_contact_cells,&i.asset->envelope_tree,event.exit_diameter_mult,&exit,edgeSeed,chewed?std::min(event.cavitation,.4):event.cavitation,chewed?0.:event.burst,i.asset->tissue_field.get(),recordPrimary?&primaryRecord:nullptr,&primaryWritten,-1,marchReuse,event.gore.wound_variation_mult,slabEvent?&slabSpec:nullptr) : eraseRay(candidate,origin,mapping.direction,event.radius,event.depth,&result.edge_contact_cells);evMark("erase");
      if(removal.freshSolid==0&&event.penetration_m>0&&!slabEvent&&!meleeEdge&&!event.melee&&!event.blast&&!holesMode&&!holeHold&&!closedExit&&
         !std::getenv("GFY_NO_CORE_CONTACT")){
       const Vec hd=unit(mapping.direction);const auto crossings=i.asset->envelope_tree.crossingDistances(origin,hd);
       const double first=dot(march.entry-origin,hd);double limit=first+entryPenetration+2*event.radius;
       {bool inside=crossings.size()%2==1;double from=0;
        for(const double x:crossings){if(inside){if(first>=from&&first<=x)limit=std::max(limit,x);inside=false;}else{from=x;inside=true;}}}
       const March core=heldContact(candidate,origin,hd,event.radius,entryPenetration,limit,edgeSeed,event.gore.wound_variation_mult);
       const double until=core.found?dot(core.entry-origin,hd):limit;bool hole=false;
       for(double s=0;s<=until&&!hole;s+=candidate.pitch*.5){const Cell c=piece_session::cellOf(candidate,origin+hd*s);
        hole=candidate.valid(c)&&candidate.solidBase(c)&&candidate.removed(c);}
       if(!core.found)nothingWhy=hole?"passed_through_hole":"no_anatomy_in_cut";
       else if(hole&&!hole_reach_value())nothingWhy="crater_bottom_not_reached";
       else{const auto firstRemoval=removal;const TunnelExit firstExit=exit;exit=TunnelExit{};
        removal=eraseTunnel(candidate,origin,mapping.direction,event.radius,entryPenetration,&result.edge_contact_cells,&i.asset->envelope_tree,event.exit_diameter_mult,&exit,edgeSeed,chewed?std::min(event.cavitation,.4):event.cavitation,chewed?0.:event.burst,i.asset->tissue_field.get(),recordPrimary?&primaryRecord:nullptr,&primaryWritten,-1,&core,event.gore.wound_variation_mult,nullptr);evMark("held_contact");
        if(removal.freshSolid){march=core;result.rest_entry=point(core.entry);result.held_contact=true;}
        else{removal=firstRemoval;exit=firstExit;}}}
      // Air-only sphere mask edits must explicitly mark unsupported history.
      if(historyBase&&event.penetration_m<=0&&removal.freshMask)historyLimits|=resolved_cut_legacy_sphere;
      if(historyBase&&removal.freshSolid){
       if(grid.ownership&&!i.piece)historyLimits|=resolved_cut_owned_grid;
       else if(event.penetration_m<=0)historyLimits|=resolved_cut_legacy_sphere;
       else if(recordPrimary){
#ifdef GFY_VOXEL_WORKER_TESTS
        if(worker_test::suppress_resolved_written)primaryWritten=false;
#endif
        if(primaryWritten){remember(std::move(primaryRecord),event.sequence,mapping,ResolvedTunnel::Role::primary);
         if(mask_only_mode_value()==3)historyCuts.back().seam_reach=seam_tear::reachAt(event);} // seam: the round's reach
        else historyLimits|=resolved_cut_unwritten;
       }
       else historyLimits|=resolved_cut_capacity;
       if(result.edge_contact_cells)historyLimits|=resolved_cut_edge_repairs;
      }
      // Test-only failure exercises this event's actual undo handler.
#ifdef GFY_VOXEL_WORKER_TESTS
      if(historyBase&&event.penetration_m<=0&&removal.freshMask&&worker_test::fail_history_after_sphere)
       throw RecoverableImpactError("test_history_sphere_rollback");
#endif
      usedRough=usedRough||(edgeSeed!=0&&removal.freshSolid>0);
      if(closedExit){ // the exit crater: the exit size, carved inward from the far skin
       const Vec d=unit(mapping.direction);const double craterRadius=std::clamp(event.radius*event.exit_diameter_mult,.0005,.08);
       const Vec craterOrigin=craterPoint+d*(grid.pitch*2+craterRadius);
       std::size_t craterEdge=0;ResolvedTunnel craterRecord;bool craterWritten=false;const bool recordCrater=historyRoom();
       auto crater=craterDepth<=0?core::HitResult{}:eraseTunnel(candidate,craterOrigin,d*-1.,craterRadius,std::min(.5,craterDepth),&craterEdge,&i.asset->envelope_tree,1.,nullptr,
        edgeSeed?(edgeSeed^0xE817C4A7E5ull):0,0.,0.,i.asset->tissue_field.get(),recordCrater?&craterRecord:nullptr,&craterWritten,-1,nullptr,event.gore.wound_variation_mult);
       if(historyBase&&crater.freshSolid){
        if(grid.ownership&&!i.piece)historyLimits|=resolved_cut_owned_grid;
        else if(recordCrater){if(craterWritten)remember(std::move(craterRecord),event.sequence,mapping,ResolvedTunnel::Role::vent);else historyLimits|=resolved_cut_unwritten;}
        else historyLimits|=resolved_cut_capacity;
        if(craterEdge)historyLimits|=resolved_cut_edge_repairs;}
       removal.dirty.insert(crater.dirty.begin(),crater.dirty.end());removal.freshSolid+=crater.freshSolid;removal.freshMask+=crater.freshMask;
       for(unsigned s=0;s<64;++s)removal.freshBySlot[s]+=crater.freshBySlot[s];result.edge_contact_cells+=craterEdge;
       exit.reached=true;exit.point=craterPoint;}
      // Entry burst: a bore from inside the channel back out through the entry skin, flaring and cracking at the mouth
      // like an exit (its bore wider than the channel so it starts in tissue). Rough first attempt only.
      if(edgeSeed!=0&&event.entry_burst>0&&event.penetration_m>0&&removal.freshSolid>0){
       const Vec d=unit(mapping.direction);const Vec mouth=origin+d*removal.surface;const double s=std::clamp(event.entry_burst,0.,1.);
       // Sized against a moderate exit of this shot (75-90 %), never past this shot's own exit mouth when it bursts.
       const double moderate=event.radius*event.exit_diameter_mult*(1+std::min(1.5,1.2*(.35+.25*s)));
       const double exitMouth=event.radius*event.exit_diameter_mult*(1+std::min(1.5,event.burst*1.2));
       double target=std::max(event.radius*1.3,(.75+.15*s)*moderate);if(event.burst>0)target=std::min(target,.9*exitMouth);
       target=std::clamp(target,.0005,.08);
       const double bore=std::clamp(event.radius*(1.15+.1*s),.0005,.08);
       // mouth = bore x flare x (1 + burst growth): the flare carries what the burst growth does not
       double burstE=std::clamp(.5+.5*s,0.,1.);double growth=1+std::min(1.5,burstE*1.2);double flare=target/(bore*growth);
       if(flare<1){flare=1;growth=std::max(1.,target/bore);burstE=(growth-1)/1.2;}flare=std::min(flare,2.);
       const double depth=std::min(std::max(bore*2.5,target*1.2),std::max(0.,holeCapEnd-removal.surface)); // modes: capped in Bullet holes only
       // Its exit is the tissue surface (the mouth); it runs on at the full mouth width out through the clothing to
       // the surface the shot hit, so the skin shows the crater.
       const double reach=std::min(.5,depth+std::max(0.,dot(mouth-mapping.entry,d))+bore*2.);
       std::size_t craterEdge=0;ResolvedTunnel craterRecord;bool craterWritten=false;const bool recordCrater=historyRoom();TunnelExit mouthExit;
       auto crater=eraseTunnel(candidate,mouth+d*depth,d*-1.,bore,reach,&craterEdge,&i.asset->envelope_tree,flare,&mouthExit,
        edgeSeed^0xE17B0B5ull,0.,burstE,i.asset->tissue_field.get(),recordCrater?&craterRecord:nullptr,&craterWritten,depth,nullptr,event.gore.wound_variation_mult);
       if(historyBase&&crater.freshSolid){
        if(grid.ownership&&!i.piece)historyLimits|=resolved_cut_owned_grid;
        else if(recordCrater){if(craterWritten)remember(std::move(craterRecord),event.sequence,mapping,ResolvedTunnel::Role::vent);else historyLimits|=resolved_cut_unwritten;}
        else historyLimits|=resolved_cut_capacity;
        if(craterEdge)historyLimits|=resolved_cut_edge_repairs;}
       removal.dirty.insert(crater.dirty.begin(),crater.dirty.end());removal.freshSolid+=crater.freshSolid;removal.freshMask+=crater.freshMask;
       for(unsigned q=0;q<64;++q)removal.freshBySlot[q]+=crater.freshBySlot[q];result.edge_contact_cells+=craterEdge;
       if(mouthExit.reached&&crater.freshSolid)entryMouth=target;}
      // Lateral vent: a second seeded tunnel from the cavitation stretch out of the side or top of the part (never
      // downward in rest space), with the burst crater and cracks at its own exit. Rough first attempt only; the smooth
      // retry keeps the plain channel. Skipped when no skin lies within 45 cm along the vent (no blind pockets).
      if(edgeSeed!=0&&event.vent>0&&!chewed&&!holesMode&&event.penetration_m>0&&removal.freshSolid>0){ // modes: no vent in Bullet holes only
       auto h=[&](unsigned k){std::uint64_t x=(edgeSeed^0x5EEDB00Bull)*0x9E3779B97F4A7C15ull+k*0xC2B2AE3D27D4EB4Full;x^=x>>29;x*=0xBF58476D1CE4E5B9ull;x^=x>>32;return double(x>>11)*(1./9007199254740992.);};
       const Vec dir=unit(mapping.direction);const Vec entryPoint=origin+dir*removal.surface;
       const double along=exit.reached?length(exit.point-entryPoint):std::min(event.penetration_m,.5);
       const Vec axis=std::abs(dir.x)<.8?Vec{1,0,0}:Vec{0,1,0};const Vec u=unit(cross(dir,axis)),v=cross(dir,u);
       const double a=6.283185307179586*h(1);Vec lateral=u*std::cos(a)+v*std::sin(a);
       if(lateral.y<-.25)lateral=lateral*-1.;
       const Vec ventDir=unit(lateral+dir*(.10+.25*h(2)));
       const Vec ventOrigin=entryPoint+dir*(along*(.30+.35*h(3)));
       // The vent bore grows with the cavity that drives it: a violent pistol cavity (cavitation 2) vents through a bore
       // about twice the bullet's; the mouth then flares and cracks like an exit burst.
       const double ventRadius=std::clamp(event.radius*(.55+.35*h(4))*(1+.45*event.cavitation),.0005,.08);
       const auto crossings=i.asset->envelope_tree.crossingDistances(ventOrigin,ventDir);
       const double reach=(crossings.size()%2==1)?crossings.front():-1;
       if(reach>ventRadius&&reach<.45){
        const double ventExitMult=std::min(2.,1.4+.5*h(5)),ventBurst=std::clamp(.6+.4*event.vent,0.,1.);
        std::size_t ventEdge=0;ResolvedTunnel ventRecord;bool ventWritten=false;const bool recordVent=historyRoom();
        auto ventRemoval=eraseTunnel(candidate,ventOrigin,ventDir,ventRadius,std::min(.5,reach+ventRadius*2.5),&ventEdge,&i.asset->envelope_tree,ventExitMult,&ventExit,edgeSeed^0x5EEDB00Bull,.35,ventBurst,i.asset->tissue_field.get(),recordVent?&ventRecord:nullptr,&ventWritten,-1,nullptr,event.gore.wound_variation_mult);
        if(historyBase&&ventRemoval.freshSolid){
         if(grid.ownership&&!i.piece)historyLimits|=resolved_cut_owned_grid;
         else if(recordVent){
#ifdef GFY_VOXEL_WORKER_TESTS
          if(worker_test::suppress_resolved_written)ventWritten=false;
#endif
          if(ventWritten)remember(std::move(ventRecord),event.sequence,mapping,ResolvedTunnel::Role::vent);
          else historyLimits|=resolved_cut_unwritten;
         }
         else historyLimits|=resolved_cut_capacity;
         if(ventEdge)historyLimits|=resolved_cut_edge_repairs;
        }
        removal.dirty.insert(ventRemoval.dirty.begin(),ventRemoval.dirty.end());removal.freshSolid+=ventRemoval.freshSolid;removal.freshMask+=ventRemoval.freshMask;
        for(unsigned s=0;s<64;++s)removal.freshBySlot[s]+=ventRemoval.freshBySlot[s];result.edge_contact_cells+=ventEdge;
        ventRadiusOut=ventRadius*ventExitMult*(1+std::min(1.5,ventBurst*1.2));}
      }evMark("vent");
      if(holesMode&&modesLog&&event.penetration_m>0){ // modes test read-out (GFY_MODES_LOG, offline probes): the removal near this shot
       // measured on the grid itself. Every removed tissue cell on the axis and 32 parallel rays (0.5, 1 and 1.5 x the radius; the
       // hole and its widest swell) out to 0.6 m: its TRUE depth = the distance from its centre to the nearest cell outside the
       // original body (base grid) less half a cell. Reports the deepest, and the cells deeper than the cap + one cell.
       const Vec hd=unit(mapping.direction);const Vec u0=unit(cross(hd,std::abs(hd.x)<.8?Vec{1,0,0}:Vec{0,1,0})),v0=cross(hd,u0);
       const Grid&b0=i.asset->base;std::set<Cell> gone;
       for(int k=-1;k<36;++k){const double a=.5235987755982988*(k<0?0:k%12),f=k<0?0.:(k<12?.5:(k<24?1.:1.5));const Vec o=origin+(u0*std::cos(a)+v0*std::sin(a))*(event.radius*f);
        for(double t=0;t<.6;t+=grid.pitch*.25){const Cell c=piece_session::cellOf(candidate,o+hd*t);if(candidate.valid(c)&&candidate.solidBase(c)&&candidate.removed(c))gone.insert(c);}}
       auto airAt=[&](const Cell&c){const int R=16;double best=1e9;
        for(int r=1;r<=R&&r<best;++r)for(int x=-r;x<=r;++x)for(int y=-r;y<=r;++y)for(int z=-r;z<=r;++z){
         if(std::max(std::abs(x),std::max(std::abs(y),std::abs(z)))!=r)continue;const Cell q{c.x+x,c.y+y,c.z+z};
         if(!b0.valid(q)||!b0.solidBase(q))best=std::min(best,std::sqrt(double(x*x+y*y+z*z)));}
        return best>=1e8?double(R)*grid.pitch:(best-.5)*grid.pitch;};
       double deepest=0;std::size_t over=0;for(const Cell&c:gone){const double dd=airAt(c);deepest=std::max(deepest,dd);if(dd>damage_modes::holeDepth+grid.pitch)++over;}
       std::cerr<<"modes_measure seq="<<event.sequence<<" thin="<<holeThin<<" thick_mm="<<(holeThick>=0?holeThick*1e3:-1.)<<" removed_cells_near_shot="<<gone.size()
        <<" deepest_removed_below_surface_mm="<<deepest*1e3<<" cells_deeper_than_cap_plus_one_cell="<<over<<std::endl;}
      // R62 head burst events (HeadFracture.h): a big burst through the brain draws its event; the skull cap or the whole vault
      // in pieces cracks free here (same journal: a refusal leaves the shot as it was) and comes apart with the corpse.
      // headsize: a NECK HIT - a bullet the game booked on the head bone whose path has no opening on the head (it passes under the
      // head's floor: HeadFracture.h onHead). It draws no head event but the BODY's event below, as a shoulder or chest hit does
      // (the same draw, size and rules; alive or dead), and that event stays above the neck's base joint (head_burst::clipAbove):
      // never the shoulders. Normal mode bullets only (Extreme keeps the large head events). Decided once per shot, only when one
      // of the two paths below asks.
      int neckState=-1;Vec neckFloor{0,0,0};
      auto neckHit=[&]{if(neckState<0)neckState=(dmode!=2&&!event.blast&&!event.melee&&event.cranial_bone!=65535&&event.hit_bone==event.cranial_bone&&mask_only_mode_value()==3&&
        i.asset->cranial_measure.valid&&!head_fracture::onHead(i.asset->sever_segments,i.asset->cranial_measure,i.asset->closure_field.get(),march.entry,mapping.direction,eventIn.penetration_m,neckFloor))?1:0;
       return neckState==1;};
      bool headCracked=false;
      if(!event.blast&&(!event.melee||event.melee_head)&&!event.living_hit&&historyBase&&event.penetration_m>0&&event.hit_bone==event.cranial_bone&&(mask_only_mode_value()==3?removal.freshSolid>0:removal.freshBySlot[3]>0)&& // T1: pieces too; headfix: value 3, any round that enters the head
         i.asset->cranial_measure.valid&&!i.asset->sever_segments.empty()&&!damage_modes::noSever(dmode)&&!std::getenv("GFY_NO_HEAD_FRAGMENTS")&&!neckHit()&&(eventIn.blast||(eventIn.explosion_world==0&&eventIn.explosion_cookie==0))){ // headsize: not a neck hit; fragwound: no head event from a grenade fragment
       const std::uint64_t headSeed=edgeSeed?edgeSeed:event.sequence;
       auto kind=mask_only_mode_value()==3?head_fracture::event3(eventIn.round_class,eventIn.radius,event.gore.head_blowout_frequency_mult,headSeed) // headfix: any round, today's chances
        :head_fracture::event(eventIn.radius,eventIn.penetration_m,eventIn.burst,event.gore.head_blowout_frequency_mult,headSeed,eventIn.round_class); // klass: Lua's class when given (modes: unscaled)
       if(event.melee)kind=event.gore.head_blowout_frequency_mult>0?head_fracture::Event(std::min<unsigned>(event.melee_head,2u)):head_fracture::none; // melee (value 3): a sledge's killing blow - Lua's draw
       if(const char*force=std::getenv("GFY_HEAD_EVENT"))kind=head_fracture::Event(std::clamp(std::atoi(force),0,2)); // tests
       if(kind!=head_fracture::none){
        std::vector<const ResolvedTunnel*> tearCuts;
        {std::vector<const ResolvedCutHistory*> chain;for(auto p=historyBase.get();p;p=p->parent.get())chain.push_back(p);
         for(auto it=chain.rbegin();it!=chain.rend();++it)for(const auto&rr:(*it)->appended)tearCuts.push_back(&rr);}
        for(const auto&rr:historyCuts)tearCuts.push_back(&rr);
        // melee (value 3): a sledge's burst is sized by its blow (Lua's head_strength 0.2-1, MeleeImpact.h): its class (1 rifle-sized,
        // 2 heavy) and the true-size term of HeadBurst.h strength() set so that strength() returns it (within the class's +-0.1; above
        // 0.92 the burst and cavitation terms add up to 0.08 more)
        const double meleeS=eventIn.melee_head_strength,meleeBase=eventIn.round_class>=2?.82:.50,meleeTypical=eventIn.round_class>=2?.0085:.0037;
        const double headSize=dmode==2?damage_modes::extremeSize:1.; // modes: Extreme doubles the burst (HeadBurst.h Round::size)
        const head_burst::Round headRound=eventIn.melee?head_burst::Round{eventIn.round_class,meleeTypical+.004*std::clamp(meleeS-meleeBase,-.1,.1),eventIn.penetration_m,
            meleeS>.92?.5+std::min(.5,(meleeS-.92)*6.25):.5,meleeS>.92?.8+std::min(.8,(meleeS-.92)*10.):.8,headSize}
         :head_burst::Round{eventIn.round_class,eventIn.radius,eventIn.penetration_m,eventIn.burst,eventIn.cavitation,headSize}; // headfix: the round sizes the burst
        auto hf=head_fracture::fracture(candidate,i.asset->sever_segments,*attemptSever,i.asset->cranial_measure,tearCuts,i.asset->tissue_field.get(),
         march.entry,unit(mapping.direction),headSeed,event.hit_bone,kind,i.asset->closure_field.get(),&headRound,
         event.melee!=0||dmode==2); // headsize: a bullet in Normal: a section of the head, the body event's size; Extreme and a crushing blow: the large head events, as before
        tearCuts.clear();GFY_HEAD_FRACTURE_REPORT(hf,kind);
        if(!hf.records.empty()){
         for(auto&rr:hf.records){if(historyRoom())remember(std::move(rr),event.sequence,mapping,ResolvedTunnel::Role::tear);else historyLimits|=resolved_cut_capacity;}
         if(hf.edgeCells)historyLimits|=resolved_cut_edge_repairs;
         removal.dirty.insert(hf.dirty.begin(),hf.dirty.end());removal.freshSolid+=hf.cells;removal.freshMask+=hf.mask;
         for(unsigned s=0;s<64;++s)removal.freshBySlot[s]+=hf.bySlot[s];}
        if(hf.done){headCracked=true;result.sever=true;result.sever_dangling=false;result.sever_bone=event.hit_bone;result.sever_point=point(hf.centre);
         result.sever_axis=point(hf.up);result.sever_remaining_tissue=0;result.sever_remaining_bone=0;result.sever_cells=hf.cells;noteTornEnd(result,hf);}}}
      bool bulletSplit=false;
      if(!headCracked&&!bulletBurstBatch&&!event.blast&&!event.melee&&!slabEvent&&!event.tear_request&&event.explosion_world==0&&event.explosion_cookie==0&&
         historyBase&&event.penetration_m>0&&removal.freshSolid>0&&(event.hit_bone!=event.cranial_bone||neckHit())/* headsize: or a neck hit */&&mask_only_mode_value()==3&&
         !i.asset->sever_segments.empty()&&!damage_modes::noSever(dmode)&&!holesMode&&!std::getenv("GFY_NO_BULLET_BURST")){
       const std::uint64_t bseed=core::quantMix((edgeSeed?edgeSeed:event.sequence)^0xB0D1B0D1ull);
       auto kind=bullet_burst::event(eventIn.round_class,eventIn.radius,double(body_blowout_frequency_value()),bseed); // the decision reads the unscaled event
       if(const char*force=std::getenv("GFY_BODY_EVENT"))kind=bullet_burst::Event(std::clamp(std::atoi(force),0,2)); // tests
       if(kind!=bullet_burst::none){bulletBurstBatch=true;
        const Vec d=unit(mapping.direction);
        const auto*cl=i.asset->closure_field.get();
        auto tissueAt=[&](const Vec&q){if(!cl||!cl->available)return true;const unsigned l=cl->sample(q.x,q.y,q.z)&63u;return l!=0&&l!=16;};
        // the opening: a burst at the shot's way out (its exit; its entry when it stopped inside), a blowout at the exit for 70 % of
        // the shots that go through, else at the entry (the round's own travel through the tissue, not the chewed channel)
        Vec skin,away;const bool through=bullet_burst::wayOut(march.entry,d,std::min(.5,eventIn.penetration_m),tissueAt,skin,away);
        const bool atExit=through&&(kind==bullet_burst::burst||damage_tear::hash01(bseed,0xB0D1E7ull,7)<.7);
        const Vec mouth=atExit?away:skin,outward=atExit?d:d*-1.;
        const head_burst::Round bbRound{eventIn.round_class,eventIn.radius,eventIn.penetration_m,eventIn.burst,eventIn.cavitation,dmode==2?damage_modes::extremeSize:1.}; // modes: Extreme doubles it
        const auto planT0=core::Clock::now();auto plan=bullet_burst::plan(kind,mouth,outward,bseed,bbRound,tissueAt);const double planMs=core::ms(planT0); // burstshell: the plan's time for the test log
        if(neckHit())head_burst::clipAbove(plan,neckFloor,Vec{0,1,0}); // headsize: a neck hit's event stays above the neck's base joint
        // a living body never separates: its burst is blown out; so is a corpse burst too small for its fragments to come off
        const bool pieces=kind==bullet_burst::burst&&!event.living_hit&&plan.rf>=bullet_burst::piecesMinRf&&body_blowout_chunks_value()!=0; // nochunks: the menu's switch
        std::size_t bbErased=0,bbCells=0,bbEdge=0;double bbMs=0,bbLabel=0,bbErase=0,bbRepair=0;const char*bbRefusal=nullptr;bool bbDone=false;
        if(pieces){auto bb=head_burst::burst(candidate,i.asset->sever_segments,*attemptSever,plan);
         if(!bb.records.empty()){for(auto&rr:bb.records){if(historyRoom())remember(std::move(rr),event.sequence,mapping,ResolvedTunnel::Role::tear);else historyLimits|=resolved_cut_capacity;}}
         if(bb.edgeCells)historyLimits|=resolved_cut_edge_repairs;
         removal.dirty.insert(bb.dirty.begin(),bb.dirty.end());removal.freshSolid+=bb.cells;removal.freshMask+=bb.mask;
         for(unsigned s=0;s<64;++s)removal.freshBySlot[s]+=bb.bySlot[s];
         bbErased=bb.erased;bbCells=bb.cells;bbEdge=bb.edgeCells;bbMs=bb.ms;bbLabel=bb.labelMs;bbErase=bb.eraseMs;bbRepair=bb.repairMs;bbRefusal=bb.refusal;bbDone=bb.done;bulletSplit=bb.done;}
        else{auto cr=bullet_burst::crater(candidate,i.asset->sever_segments,*attemptSever,plan);
         if(cr.edgeCells)historyLimits|=resolved_cut_edge_repairs;
         removal.dirty.insert(cr.dirty.begin(),cr.dirty.end());removal.freshSolid+=cr.cells;removal.freshMask+=cr.mask;
         for(unsigned s=0;s<64;++s)removal.freshBySlot[s]+=cr.bySlot[s];
         bbErased=cr.erased;bbCells=cr.cells;bbEdge=cr.edgeCells;bbMs=cr.ms;bbLabel=cr.labelMs;bbErase=cr.eraseMs;bbRepair=cr.repairMs;bbRefusal=cr.refusal;bbDone=cr.done;}
        if(bbDone){bulletKind=pieces?2u:(kind==bullet_burst::burst?3u:1u);bulletPoint=plan.B-outward*plan.rf;bulletStrength=plan.sigma;
         bulletMouth=std::min(.08,plan.rf);bulletMouthExit=atExit;bulletMouthAt=mouth;bulletDir=d;}
        if(std::getenv("GFY_M3_LOG")||std::getenv("GFY_TEAR_SELECT_LOG"))std::cerr<<"bullet_burst seq="<<event.sequence<<" kind="<<bullet_burst::name(kind)<<" pieces="<<pieces
         <<" living="<<event.living_hit<<" killing="<<event.killing_hit<<" bone="<<event.hit_bone<<" class="<<head_burst::roundClass(bbRound)<<" strength="<<plan.sigma<<" rf_mm="<<plan.rf*1000
         <<" near="<<plan.nNear<<" groups="<<plan.groups<<" through="<<through<<" at="<<(atExit?"exit":"entry")<<" mouth=("<<mouth.x<<","<<mouth.y<<","<<mouth.z<<") blast=("<<plan.B.x<<","<<plan.B.y<<","<<plan.B.z
         <<") erased="<<bbErased<<" cells="<<bbCells<<" edge_cells="<<bbEdge<<" plan_ms="<<planMs<<" label_ms="<<bbLabel<<" erase_ms="<<bbErase<<" repair_ms="<<bbRepair<<" ms="<<bbMs<<" done="<<bbDone<<" refusal="<<(bbRefusal?bbRefusal:"")<<std::endl;}}
      // Damage-threshold severing: the stations this cut touched are re-measured; a failed one tears (same journal).
      // Damage-driven dismemberment: a failed station tears through its wounds and remaining tissue (DamageTear.h).
      if(event.blast&&removal.freshSolid>0){blastDirtyList.insert(blastDirtyList.end(),removal.dirty.begin(),removal.dirty.end());blastMapping=mapping;blastSequence=event.sequence;blastSequences.insert(event.sequence);blastRays.push_back({mapping.entry,unit(mapping.direction)});
       if(event.blade&&event.slab){const Vec end=mapping.entry+unit(mapping.direction)*event.penetration_m;
        slabPlanes.push_back({(mapping.entry+end)*.5,slabSpec.normal,mapping.entry,end});
        // R61: the blade's recorded torn sheet, so its tear reads the sheet's centre surface (DamageTear.h)
        for(auto r=historyCuts.rbegin();r!=historyCuts.rend();++r)if(r->sequence==event.sequence&&r->slab&&r->role==ResolvedTunnel::Role::primary){
         if(r->slab_seed){auto&pl=slabPlanes.back();pl.sheet=true;pl.entry=vec(r->entry);pl.dir=vec(r->direction);pl.side=vec(r->edge_v);
          pl.half=r->radius;pl.width=r->slab_half_width;pl.reach=r->penetration;pl.inlet=r->inlet;pl.seed=r->slab_seed;}
         break;}}
       if(event.blade){auto&slot=bladeSlots[event.blade];if(!slot.n)slot.a=mapping.entry;slot.b=mapping.entry;slot.dir=slot.dir+unit(mapping.direction);++slot.n;}}
      else if(!headCracked&&!bulletSplit/* bulletburst */&&historyBase&&removal.freshSolid>0&&event.penetration_m>0&&!melee::blade(event.melee)&&!i.asset->sever_segments.empty()&&!damage_modes::noSever(dmode)){ // melee: a blade cuts clean (below), never by the damage threshold
       const double severScale=eventIn.sever_scale>0?eventIn.sever_scale:std::clamp(eventIn.radius/.008,.3,2.); // modes: unscaled
       std::vector<const ResolvedTunnel*> tearCuts;
       {std::vector<const ResolvedCutHistory*> chain;for(auto p=historyBase.get();p;p=p->parent.get())chain.push_back(p);
        for(auto it=chain.rbegin();it!=chain.rend();++it)for(const auto&r:(*it)->appended)tearCuts.push_back(&r);}
       for(const auto&r:historyCuts)tearCuts.push_back(&r);
       const Vec shotDir=unit(mapping.direction);
       static const bool bulletTissueTear=std::getenv("GFY_NO_BULLET_TISSUE_TEAR")==nullptr;
       auto verdict=damageTear(candidate,i.asset->sever_segments,*attemptSever,removal.dirty,severScale,tearCuts,i.asset->tissue_field.get(),nullptr,nullptr,nullptr,nullptr,bulletTissueTear?i.asset->closure_field.get():nullptr,&shotDir);
       tearCuts.clear();
       if(!verdict.records.empty()){
        for(auto&r:verdict.records){if(historyRoom())remember(std::move(r),event.sequence,mapping,ResolvedTunnel::Role::tear);else historyLimits|=resolved_cut_capacity;}
        if(verdict.edgeCells)historyLimits|=resolved_cut_edge_repairs;
        removal.dirty.insert(verdict.dirty.begin(),verdict.dirty.end());removal.freshSolid+=verdict.cells;removal.freshMask+=verdict.mask;
        for(unsigned s=0;s<64;++s)removal.freshBySlot[s]+=verdict.bySlot[s];}
       if(verdict.tear){result.sever=true;result.sever_dangling=verdict.dangling;result.sever_bone=verdict.bone;result.sever_point=point(verdict.point);result.sever_axis=point(verdict.axis);
        result.sever_remaining_tissue=verdict.remainingTissue;result.sever_remaining_bone=verdict.remainingBone;result.sever_cells=verdict.cells;noteTornEnd(result,verdict);}
       else if(verdict.probeBone!=65535){result.sever_probe_bone=verdict.probeBone;result.sever_probe_tissue=verdict.probeTissue;result.sever_probe_bone_left=verdict.probeBoneLeft;}
      }
      if(event.killing_hit&&!result.sever&&!headCracked&&!bulletSplit/* bulletburst */&&!event.blast&&historyBase&&removal.freshSolid>0&&event.penetration_m>0&&

         killSeverRound(eventIn)&&(eventIn.blast||(eventIn.explosion_world==0&&eventIn.explosion_cookie==0))&&!i.asset->sever_segments.empty()&&!damage_modes::noSever(dmode)&&!std::getenv("GFY_NO_KILL_SEVER")){ // fragwound: a fragment has no killing-shot tear
       // raggedsever: the tear is made after the batch's events, through the damage (below the event loop); 0 = today's cut, here
       // (a melee blow keeps today's: its own blade cut below reads this tear's result)
       if(kill_sever_ragged_value()&&!event.melee)killSevers.push_back({out->impacts.size(),event.sequence,mapping,march.entry,unit(mapping.direction),event.penetration_m,event.hit_bone,edgeSeed?edgeSeed:event.sequence});
       else{
       const Vec d=unit(mapping.direction),e0=march.entry;const SeverSegment*seg=nullptr;const SeverStation*st=nullptr;double best=1e30;bool torn=false;
       std::size_t stationTotal=0;for(const auto&s:i.asset->sever_segments)stationTotal+=s.stations.size();
       if(attemptSever->failed.size()!=stationTotal)attemptSever->failed.assign(stationTotal,0);
       std::size_t k0=0;
       for(const auto&s:i.asset->sever_segments){const std::size_t base=k0;k0+=s.stations.size();if(s.radius>.13)continue; // limbs and neck
        for(std::size_t q=0;q<s.stations.size();++q){const Vec c=s.stations[q].center-e0;const double t=std::clamp(dot(c,d),0.,event.penetration_m);
         const double off=length(c-d*t);if(off>s.radius+.01)continue;const double score=off+(s.bone==event.hit_bone?0.:.02);
         if(score<best){best=score;seg=&s;st=&s.stations[q];torn=attemptSever->failed[base+q]!=0;}}}
       if(seg&&!torn){
        Vec n=seg->axis;{const Vec across=d-n*dot(d,n);
         if(length(across)>1e-6){const double th=(10.+20.*damage_tear::hash01(edgeSeed?edgeSeed:event.sequence,92,1))*3.141592653589793/180;n=unit(seg->axis*std::cos(th)+unit(across)*std::sin(th));}}
        TearPlane plane;plane.point=st->center;plane.normal=n;plane.a=plane.b=st->center;const TearForce force{seg,st,&plane};
        TearGuide forced;forced.tear_radius=.008;
        std::vector<const ResolvedTunnel*> tearCuts;
        {std::vector<const ResolvedCutHistory*> chain;for(auto p=historyBase.get();p;p=p->parent.get())chain.push_back(p);
         for(auto it=chain.rbegin();it!=chain.rend();++it)for(const auto&rr:(*it)->appended)tearCuts.push_back(&rr);}
        for(const auto&rr:historyCuts)tearCuts.push_back(&rr);
        const std::set<Cell> where{Grid::tileKey(piece_session::cellOf(candidate,st->center))};
        static const bool bulletTissueTear=std::getenv("GFY_NO_BULLET_TISSUE_TEAR")==nullptr; // R64: the A/B switch above
        auto verdict=damageTear(candidate,i.asset->sever_segments,*attemptSever,where,2.,tearCuts,i.asset->tissue_field.get(),nullptr,nullptr,&forced,&force,bulletTissueTear?i.asset->closure_field.get():nullptr,&d);
        tearCuts.clear();
        if(!verdict.records.empty()){
         for(auto&r:verdict.records){if(historyRoom())remember(std::move(r),event.sequence,mapping,ResolvedTunnel::Role::tear);else historyLimits|=resolved_cut_capacity;}
         if(verdict.edgeCells)historyLimits|=resolved_cut_edge_repairs;
         removal.dirty.insert(verdict.dirty.begin(),verdict.dirty.end());removal.freshSolid+=verdict.cells;removal.freshMask+=verdict.mask;
         for(unsigned s=0;s<64;++s)removal.freshBySlot[s]+=verdict.bySlot[s];}
        if(verdict.tear&&(verdict.remainingPixels||verdict.woundPixels)){result.sever=true;result.sever_dangling=false;result.sever_bone=verdict.bone;
         result.sever_point=point(verdict.point);result.sever_axis=point(verdict.axis);result.sever_remaining_tissue=verdict.remainingTissue;
         result.sever_remaining_bone=verdict.remainingBone;result.sever_cells=verdict.cells;noteTornEnd(result,verdict);}}}}
      double meleeCutFraction=-1; // melee: the fraction of the struck part's section this blade's cut has severed (-1: not measured)
      if(melee::blade(event.melee)&&!event.melee_part&&slabEvent&&!result.sever&&!headCracked&&historyBase&&removal.freshSolid>0&&
         !i.asset->sever_segments.empty()&&!damage_modes::noSever(dmode)){ // modes
       const Vec d=unit(mapping.direction),e0=mapping.entry,bn=unit(slabSpec.normal),bs=unit(slabSpec.side);const double hw=event.slab_half_width;
       const SeverSegment*seg=nullptr;const SeverStation*st=nullptr;double best=1e30;bool torn=false;
       std::size_t stationTotal=0;for(const auto&s:i.asset->sever_segments)stationTotal+=s.stations.size();
       if(attemptSever->failed.size()!=stationTotal)attemptSever->failed.assign(stationTotal,0);
       // only the part the blade struck or the one across its joint (a chop on the chest never takes the arm beside it)
       const SeverSegment*hs=nullptr;for(const auto&s:i.asset->sever_segments)if(s.bone==event.hit_bone){hs=&s;break;}
       auto joined=[&](const SeverSegment&s){if(!hs)return false;if(&s==hs)return true;const Vec se=s.pivot+s.axis*s.length,he=hs->pivot+hs->axis*hs->length;
        return length(se-hs->pivot)<.04||length(he-s.pivot)<.04||length(s.pivot-hs->pivot)<.02||length(se-he)<.02;};
       // the struck part itself whenever the blade's plane crosses it (its own section): a part's stations start .9 radius in, so a
       // neck's lie at the skull's base and the jaw's reach down into the neck; the hand's region runs back up the forearm
       std::size_t k0=0;
       if(hs&&hs->radius<=.13&&std::abs(dot(hs->axis,bn))>=.45){const double t=dot(e0-hs->pivot,bn)/dot(hs->axis,bn);
        if(t>=-.02&&t<=hs->length+.5*hs->radius){std::size_t base=0;for(const auto&s:i.asset->sever_segments){if(&s==hs)break;base+=s.stations.size();}
         for(std::size_t q=0;q<hs->stations.size();++q){const double off=std::abs(dot(hs->stations[q].center-e0,bn));
          if(off<best){best=off;seg=hs;st=&hs->stations[q];torn=attemptSever->failed[base+q]!=0;}}}}
       if(!seg)for(const auto&s:i.asset->sever_segments){const std::size_t base=k0;k0+=s.stations.size();if(s.radius>.13||!joined(s))continue; // limbs and neck
        for(std::size_t q=0;q<s.stations.size();++q){const Vec c=s.stations[q].center-e0;const double off=std::abs(dot(c,bn));
         if(off>.6*s.radius+.012)continue;const double along=dot(c,bs),depth=dot(c,d);
         if(std::abs(along)>hw+s.radius||depth< -s.radius||depth>event.penetration_m+2*s.radius+.02)continue;
         const double score=off+(s.bone==event.hit_bone?0.:.02);
         if(score<best){best=score;seg=&s;st=&s.stations[q];torn=attemptSever->failed[base+q]!=0;}}}
       // the cut's section through the chosen part: tissue / bone columns across the plane and how many of them the cut has severed
       double cutTissueLeft=1,cutBoneLeft=1;std::size_t cutColumns=0;
       if(seg&&!torn){const double h=candidate.pitch,den=std::abs(dot(seg->axis,bn));
        if(den>=.45){const double t=dot(e0-seg->pivot,bn)/dot(seg->axis,bn);const Vec c0=seg->pivot+seg->axis*t;
         const Vec e1=unit(cross(bn,std::abs(bn.x)<.9?Vec{1,0,0}:Vec{0,1,0})),e2=cross(bn,e1);const double R=seg->radius/den+h;
         const int nb=int(std::ceil(.008/h));std::size_t tis=0,tisCut=0,bon=0,bonCut=0;
         for(double x=-R;x<=R;x+=h)for(double y=-R;y<=R;y+=h){const Vec q=c0+e1*x+e2*y,dq=q-seg->pivot;const double ax=dot(dq,seg->axis);
          if(ax< -.02||ax>seg->length+.02||dot(dq,dq)-ax*ax>seg->radius*seg->radius)continue;
          bool tissue=false,bone=false,cut=false;
          for(int k=-nb;k<=nb;++k){const Cell cc=piece_session::cellOf(candidate,q+bn*(k*h));const auto lab=candidate.rawMaterialBase(cc);if(!lab)continue;tissue=true;
           if(candidate.material_slots&&lab<candidate.material_slots->size()&&(*candidate.material_slots)[lab]==2)bone=true;if(candidate.removed(cc))cut=true;}
          if(tissue){++tis;if(cut)++tisCut;}if(bone){++bon;if(cut)++bonCut;}}
         cutColumns=tis;cutTissueLeft=tis?1-double(tisCut)/double(tis):1;cutBoneLeft=bon?1-double(bonCut)/double(bon):cutTissueLeft;}}
       if(cutColumns)meleeCutFraction=1-cutTissueLeft;
       const bool through=event.melee_sever&&seg&&!torn&&cutTissueLeft<=.45&&cutBoneLeft<=.4;
       const char*why=!seg?(hs?"no_station_under_cut":"struck_part_not_a_limb"):(torn?"already_torn":(through?"":
        (!event.melee_sever&&cutTissueLeft<=.45&&cutBoneLeft<=.4?"through_but_cut_through_off":"not_through")));
       if(through){
        TearPlane plane;plane.point=e0;plane.normal=bn;plane.a=plane.b=st->center;TearForce force{seg,st,&plane};force.clean=!std::getenv("GFY_MELEE_TORN_TEST");force.tissueDepthScale=force.clean?.03:1.; // (probe test knob: the torn look)
        std::vector<const ResolvedTunnel*> tearCuts;
        {std::vector<const ResolvedCutHistory*> chain;for(auto p=historyBase.get();p;p=p->parent.get())chain.push_back(p);
         for(auto it=chain.rbegin();it!=chain.rend();++it)for(const auto&rr:(*it)->appended)tearCuts.push_back(&rr);}
        for(const auto&rr:historyCuts)tearCuts.push_back(&rr);
        const std::set<Cell> where{Grid::tileKey(piece_session::cellOf(candidate,st->center))};
        auto verdict=damageTear(candidate,i.asset->sever_segments,*attemptSever,where,2.,tearCuts,i.asset->tissue_field.get(),nullptr,nullptr,nullptr,&force,i.asset->closure_field.get(),&d);
        tearCuts.clear();
        if(!verdict.records.empty()){
         for(auto&r:verdict.records){if(historyRoom())remember(std::move(r),event.sequence,mapping,ResolvedTunnel::Role::tear);else historyLimits|=resolved_cut_capacity;}
         if(verdict.edgeCells)historyLimits|=resolved_cut_edge_repairs;
         removal.dirty.insert(verdict.dirty.begin(),verdict.dirty.end());removal.freshSolid+=verdict.cells;removal.freshMask+=verdict.mask;
         for(unsigned s=0;s<64;++s)removal.freshBySlot[s]+=verdict.bySlot[s];}
        if(verdict.tear&&(verdict.remainingPixels||verdict.woundPixels)){result.sever=true;result.sever_dangling=false;result.sever_bone=verdict.bone;
         result.sever_point=point(verdict.point);result.sever_axis=point(verdict.axis);result.sever_remaining_tissue=verdict.remainingTissue;
         result.sever_remaining_bone=verdict.remainingBone;result.sever_cells=verdict.cells;noteTornEnd(result,verdict);
         i.melee_severed[i.melee_severed_at++%8u]=event.melee_blow;} // its gape slabs are skipped below (the faces meet at rest)
        else why=verdict.refusal?verdict.refusal:"tear_refused";}
       if(std::getenv("GFY_TEAR_SELECT_LOG")||std::getenv("GFY_M3_LOG")||std::getenv("GFY_MELEE_LOG"))std::cerr<<"melee_sever kind="<<melee::name(event.melee)
        <<" sequence="<<event.sequence<<" bone="<<(seg?seg->bone:65535)<<" part_radius="<<(seg?seg->radius:0.)<<" plane_off="<<(seg?best:0.)
        <<" cut_entry_depth="<<(march.found?dot(march.entry-mapping.entry,d):0.)<<" tissue_left="<<cutTissueLeft<<" bone_left="<<cutBoneLeft<<" columns="<<cutColumns
        <<" initiator="<<event.initiator<<" severed="<<result.sever<<" why="<<why<<std::endl;}
      // melee (round 3, 2026-11-13): the chop splash fields (the game side hands them to Lua as receipt.melee_cut): the cut on this body /
      // piece session this blade blow made or went on with - a blow whose skin point lies within 8 mm of a cut's plane, its own plane
      // within 25 degrees of it and its mouth along the cut's length goes on with that cut, else it is a new cut (ids from 1, per
      // session; the last 32 cuts are kept) - its blow number on that cut, the cut's mouth centre on the skin (rest space, the first
      // blow's skin point), this blow's plane normal, travel and edge (rest space), the cut's length and its depth so far from the mouth
      // (m), the fraction of the part's section cut (0-1; 1 once severed; -1 when not measured: the trunk, no station, a stab past a
      // part), and the sever (result.sever). A repeated attempt of the same event updates its record, never adds a blow.
      if(melee::blade(event.melee)&&!event.melee_part&&slabEvent&&(removal.freshSolid>0||result.sever)){
       const Vec d=unit(mapping.direction),bn=unit(slabSpec.normal),bs=unit(slabSpec.side);const double hw=event.slab_half_width;
       std::size_t ci=i.melee_cuts.size();
       for(std::size_t q=0;q<i.melee_cuts.size();++q)if(i.melee_cuts[q].last_sequence==event.sequence){ci=q;break;}
       if(ci==i.melee_cuts.size())for(std::size_t q=0;q<i.melee_cuts.size();++q){const auto&c=i.melee_cuts[q];const Vec o=mapping.entry-c.centre;
        if(std::abs(dot(o,c.normal))<=.008&&std::abs(dot(bn,c.normal))>=.9&&std::abs(dot(o,c.edge))<=c.half+hw){ci=q;break;}}
       if(ci==i.melee_cuts.size()){if(i.melee_cuts.size()>=32)i.melee_cuts.erase(i.melee_cuts.begin());
        i.melee_cuts.emplace_back();ci=i.melee_cuts.size()-1;auto&n=i.melee_cuts[ci];n.id=i.melee_cut_next++;n.centre=mapping.entry;n.normal=bn;n.edge=bs;n.half=hw;}
       auto&c=i.melee_cuts[ci];if(c.last_sequence!=event.sequence){++c.blows;c.last_sequence=event.sequence;}
       const Vec bottom=(march.found?march.entry:mapping.entry)+d*event.penetration_m;
       c.depth=std::max(c.depth,dot(bottom-c.centre,d));c.half=std::max(c.half,hw);if(meleeCutFraction>=0)c.fraction=std::max(c.fraction,meleeCutFraction);
       result.melee_cut_id=c.id;result.melee_cut_blow=c.blows;result.melee_cut_centre=point(c.centre);result.melee_cut_normal=point(bn);
       result.melee_cut_dir=point(d);result.melee_cut_edge=point(bs);result.melee_cut_length=2*c.half;result.melee_cut_depth=c.depth;
       result.melee_cut_fraction=result.sever?1.:c.fraction;
       if(std::getenv("GFY_MELEE_LOG"))std::cerr<<"melee_cut sequence="<<event.sequence<<" id="<<c.id<<" blow="<<c.blows<<" length="<<result.melee_cut_length
        <<" depth="<<c.depth<<" fraction="<<result.melee_cut_fraction<<" severed="<<result.sever<<" centre=("<<c.centre.x<<","<<c.centre.y<<","<<c.centre.z<<")"<<std::endl;}
      evMark("sever");
      constexpr bool station_kerf_severing=false; // interim: replaced by damage-driven low-poly dismemberment
      if(station_kerf_severing&&removal.freshSolid>0&&event.penetration_m>0&&!i.asset->sever_segments.empty()&&!std::getenv("GFY_NO_SEVER")){
       const double severScale=event.sever_scale>0?event.sever_scale:std::clamp(event.radius/.008,.3,2.);
       auto verdict=severCheck(candidate,i.asset->sever_segments,*attemptSever,removal.dirty,severScale,edgeSeed?edgeSeed:event.sequence);
       if(verdict.tear){if(historyBase)historyLimits|=resolved_cut_sever;removal.dirty.insert(verdict.dirty.begin(),verdict.dirty.end());removal.freshSolid+=verdict.cells;
        result.sever=true;result.sever_dangling=verdict.dangling;result.sever_bone=verdict.bone;result.sever_point=point(verdict.point);result.sever_axis=point(verdict.axis);
        result.sever_remaining_tissue=verdict.remainingTissue;result.sever_remaining_bone=verdict.remainingBone;result.sever_cells=verdict.cells;
#ifdef GFY_VOXEL_STAGE_PROFILE
        std::cout<<"sever bone="<<verdict.bone<<" dangling="<<verdict.dangling<<" tissue_left="<<verdict.remainingTissue<<" bone_left="<<verdict.remainingBone<<" cells="<<verdict.cells<<" ms="<<verdict.ms<<std::endl;
#endif
       }}
      grid.journal=nullptr;}
     catch(const RecoverableImpactError&e){rollback();attemptSever->invalidate();result.rejection=e.what();out->impacts.push_back(std::move(result));continue;}
     catch(...){rollback();throw;}
     result.cutter=point(removal.cutter);result.removed_mask_cells=removal.freshMask;result.removed_solid_cells=removal.freshSolid;result.removed=removal.freshSolid>0||holeHold; // modes: a hit held at the cap still bleeds
     for(unsigned s=0;s<64;++s)result.removed_m3_by_slot[s]=double(removal.freshBySlot[s])*grid.pitch*grid.pitch*grid.pitch;
     if(!result.removed){
      // hitadmission: a nominal-cylinder contact with zero newly cut anatomy is not a whole-shot geometry failure.
      std::ostringstream why;why<<(nothingWhy?nothingWhy:"impact_below_grid_sample")<<" radius_mm="<<event.radius*1000<<" pitch_mm="<<grid.pitch*1000
       <<" penetration_mm="<<event.penetration_m*1000<<" bone="<<event.hit_bone<<" source_face="<<mapping.face
       <<" rest_entry="<<mapping.entry.x<<","<<mapping.entry.y<<","<<mapping.entry.z
       <<" rest_direction="<<mapping.direction.x<<","<<mapping.direction.y<<","<<mapping.direction.z
       <<" contact="<<march.entry.x<<","<<march.entry.y<<","<<march.entry.z;
      result.rejection=why.str();
     }
     if(result.removed){
      result.ammo_section=event.ammo_section;result.hit_bone=event.hit_bone;result.hit_power=event.hit_power;result.initiator=event.initiator;
      result.radius=event.radius;result.exit_radius=event.radius*event.exit_diameter_mult;
      result.cavitation=event.cavitation;result.burst=event.burst;result.vent=event.vent;
      if(entryMouth>0){result.entry_burst=event.entry_burst;result.entry_radius=entryMouth;result.radius=std::max(result.radius,entryMouth);}
      result.entry_surface=woundSurface(i.asset->source,mapping.face,mapping.entry);
      if(exit.reached){
       // A closure across an original mouth/eye opening is not exterior skin.
       double nearest=std::numeric_limits<double>::infinity();std::uint32_t face=0;Vec p;
       i.asset->source_tree.nearest(0,exit.point,nearest,face,p);
       if(nearest<1e-10){result.exit_surface=woundSurface(i.asset->source,face,p);result.penetrated=result.exit_surface.valid;}
      }
      // The burst crater is wider than the plain exit flare (Tunnel::flareAt): report the size Lua will see on the skin.
      if(result.penetrated&&event.burst>0)result.exit_radius*=1+std::min(1.5,event.burst*1.2);
      // bulletburst: a burst's / blowout's opening is the wound's mouth (Lua: the blowout blood spray, decals, debris from the removed
      // volume); a corpse burst's point throws its fragments out (PainterCommittedEffects_impl.h). Never a sever.
      if(bulletKind){result.bullet_burst=std::uint8_t(bulletKind);result.bullet_burst_point=point(bulletPoint);result.bullet_burst_strength=bulletStrength;
       if(bulletMouthExit){ // the exit's own surface: the source mesh's outermost layer within 4 cm past the last tissue (the tunnel's
        // envelope exit can lie on a gear shell in front of a chest)
        if(!i.asset->source_tree.nodes.empty()){const auto hits=i.asset->source_tree.crossingDistances(bulletMouthAt,bulletDir);double t=-1;for(const double h:hits)if(h<=.04)t=h;
         if(t>=0){double best=1e-6;std::uint32_t face=std::uint32_t(-1);Vec p=bulletMouthAt;i.asset->source_tree.nearest(0,bulletMouthAt+bulletDir*t,best,face,p);
          if(face!=std::uint32_t(-1)){const WoundSurface s=woundSurface(i.asset->source,face,p);if(s.valid){result.exit_surface=s;result.penetrated=true;}}}}
        if(result.penetrated)result.exit_radius=std::max(result.exit_radius,bulletMouth);}
       else{result.entry_burst=std::max(result.entry_burst,1.);result.entry_radius=std::max(result.entry_radius,bulletMouth);result.radius=std::max(result.radius,bulletMouth);}
       result.burst=std::max(result.burst,1.);}
      if(ventExit.reached){double nearest=std::numeric_limits<double>::infinity();std::uint32_t face=0;Vec p;
       i.asset->source_tree.nearest(0,ventExit.point,nearest,face,p);
       if(nearest<1e-10){result.vent_surface=woundSurface(i.asset->source,face,p);result.vented=result.vent_surface.valid;result.vent_radius=ventRadiusOut;}}
     }
     if(result.removed&&event.melee)melee::report(result,event); // melee: the wound's size for its blood (a slash's length, not its kerf)
     // melee (value 3): a blade slab's box (or a blunt crater bore's) with a margin - near it the exterior's edge is the walls' smooth
     // surface (painter_slit.h): a slash, a stab and a knocked-out chunk end in smooth / ragged rims, not 2 mm cell steps
     if(result.removed&&event.melee&&i.job_dents_on&&event.penetration_m>0){const Vec sd=unit(mapping.direction);
      Vec sn,ss;double hs,hn;if(slabEvent){sn=unit(slabSpec.normal);ss=unit(slabSpec.side);hs=event.slab_half_width+.004;hn=event.radius+.004;}
      else{ss=unit(cross(sd,std::abs(sd.y)<.9?Vec{0,1,0}:Vec{1,0,0}));sn=cross(ss,sd);hs=hn=1.5*event.radius+.004;}
      const Vec sc=mapping.entry+sd*(.5*event.penetration_m);gfy_packed::Slit sl;
      for(int a=0;a<3;++a){sl.centre[a]=float(sc[a]);sl.side[a]=float(ss[a]);sl.normal[a]=float(sn[a]);}
      sl.half_side=float(hs);sl.half_normal=float(hn);sl.half_depth=float(.5*event.penetration_m+.008);
      melee::addSlit(i.job_slits,sl);}
     evMark("surfaces");changed=changed||(holeHold?removal.freshSolid>0:result.removed);if(dirty.empty())dirty.swap(removal.dirty);else dirty.merge(removal.dirty);evMark("dirty_merge");
    }out->impacts.push_back(std::move(result));
   }
   jobMark("events");
   for(const auto&ks:killSevers){if(ks.result>=out->impacts.size())continue;auto&result=out->impacts[ks.result];
    if(result.sequence!=ks.sequence||!result.removed||!result.rejection.empty()||result.sever)continue;
    const Vec d=ks.d,e0=ks.e0;const SeverSegment*seg=nullptr;const SeverStation*st=nullptr;double best=1e30;bool torn=false;std::size_t segBase=0;
    std::size_t stationTotal=0;for(const auto&s:i.asset->sever_segments)stationTotal+=s.stations.size();
    if(attemptSever->failed.size()!=stationTotal)attemptSever->failed.assign(stationTotal,0);
    std::size_t k0=0;
    for(const auto&s:i.asset->sever_segments){const std::size_t base=k0;k0+=s.stations.size();if(s.radius>.13)continue; // limbs and neck
     for(std::size_t q=0;q<s.stations.size();++q){const Vec c=s.stations[q].center-e0;const double t=std::clamp(dot(c,d),0.,ks.penetration);
      const double off=length(c-d*t);if(off>s.radius+.01)continue;const double score=off+(s.bone==ks.hitBone?0.:.02);
      if(score<best){best=score;seg=&s;st=&s.stations[q];segBase=base;torn=attemptSever->failed[base+q]!=0;}}}
    if(!seg||torn)continue;
    {bool taken=false;for(const auto&r:out->impacts)if(r.sever&&r.sever_bone==seg->bone)taken=true;if(taken)continue;}
    // killblow: the crater first (a slug's blast for a thick limb, a rifle's for a thin one; a body burst's near groups erased, no
    // pieces), its blast point between the shot's path and the limb's axis so a hit off the axis opens the middle too. Journaled; a
    // refused repair undoes the crater only and the tear below runs as before.
    if(kill_sever_blowout_value()&&mask_only_mode_value()==3&&!std::getenv("GFY_NO_BULLET_BURST")){
     const auto*kbCl=i.asset->closure_field.get();
     auto kbTissue=[&](const Vec&q){if(!kbCl||!kbCl->available)return true;const unsigned l=kbCl->sample(q.x,q.y,q.z)&63u;return l!=0&&l!=16;};
     const Vec kbC=st->center-e0;const Vec kbPath=e0+d*std::clamp(dot(kbC,d),0.,ks.penetration); // the shot's path beside the station's centre
     const bool kbBig=seg->radius>=.10;const head_burst::Round kbRound{kbBig?2:1,kbBig?.0085:.0037,ks.penetration,.5,.8,1.};
     auto kbPlan=bullet_burst::plan(bullet_burst::burst,kbPath,d,ks.leanSeed^0x4B42B10Full,kbRound,kbTissue);
     {const Vec kbTo=(kbPath+st->center)*.5-kbPlan.B;kbPlan.B=kbPlan.B+kbTo;for(unsigned q=0;q<kbPlan.S;++q)kbPlan.site[q]=kbPlan.site[q]+kbTo;
      kbPlan.entry=kbPlan.entry+kbTo;kbPlan.exit=kbPlan.exit+kbTo;}
     std::unordered_map<Cell,std::pair<bool,core::Tile>,core::CellHash> kbJournal;auto kbPriorSever=attemptSever->failed;const auto kbPriorLimits=historyLimits;
     bullet_burst::Crater kbCr;bool kbOk=true;grid.journal=&kbJournal;
     try{kbCr=bullet_burst::crater(grid,i.asset->sever_segments,*attemptSever,kbPlan);}
     catch(const RecoverableImpactError&){kbOk=false;}
     catch(...){grid.journal=nullptr;throw;}
     grid.journal=nullptr;
     if(!kbOk){historyLimits=kbPriorLimits;attemptSever->failed.swap(kbPriorSever);
      for(auto&e:kbJournal){if(e.second.first)grid.tiles[e.first]=e.second.second;else grid.tiles.erase(e.first);}attemptSever->invalidate();}
     else if(kbCr.done&&kbCr.cells){if(kbCr.edgeCells)historyLimits|=resolved_cut_edge_repairs;
      dirty.insert(kbCr.dirty.begin(),kbCr.dirty.end());changed=true;result.removed_mask_cells+=kbCr.mask;result.removed_solid_cells+=kbCr.cells;
      for(unsigned s=0;s<64;++s)result.removed_m3_by_slot[s]+=double(kbCr.bySlot[s])*grid.pitch*grid.pitch*grid.pitch;
      if(!result.bullet_burst){result.bullet_burst=3;result.bullet_burst_point=point(kbPlan.B);result.bullet_burst_strength=kbPlan.sigma;
       result.entry_burst=std::max(result.entry_burst,1.);result.burst=std::max(result.burst,1.);}}
     if(tear_decide::logOn())std::cerr<<"kill_sever_blowout seq="<<ks.sequence<<" bone="<<seg->bone<<" radius_mm="<<seg->radius*1000<<" big="<<kbBig<<" rf_mm="<<kbPlan.rf*1000<<" cells="<<kbCr.cells
      <<" done="<<kbCr.done<<" ok="<<kbOk<<" refusal="<<(kbCr.refusal?kbCr.refusal:"")<<" ms="<<kbCr.ms<<std::endl;}
    std::vector<const ResolvedTunnel*> tearCuts;
    {std::vector<const ResolvedCutHistory*> chain;for(auto p=historyBase.get();p;p=p->parent.get())chain.push_back(p);
     for(auto it=chain.rbegin();it!=chain.rend();++it)for(const auto&rr:(*it)->appended)tearCuts.push_back(&rr);}
    for(const auto&rr:historyCuts)tearCuts.push_back(&rr);
    static const bool bulletTissueTear=std::getenv("GFY_NO_BULLET_TISSUE_TEAR")==nullptr; // R64: the A/B switch of the event loop
    const auto kp=damage_tear::killPlane(grid,*attemptSever,*seg,segBase,*st,d);
    // pass 0: through the damage, one try on its own plane (TearForce::planeOnly; a plane that does not part the limb is refused and
    // changes nothing); pass 1 (no damage plane, or pass 0 refused): today's plane, exactly as before
    DamageTearVerdict verdict;bool through=false;
    for(int pass=kp.found?0:1;pass<2;++pass){const SeverStation*ts=pass==0?kp.station:st;Vec n=seg->axis;TearPlane plane;
     if(pass==0){n=kp.normal;plane.point=kp.point;}
     else{plane.point=st->center;const Vec across=d-n*dot(d,n);
      if(length(across)>1e-6){const double th=(10.+20.*damage_tear::hash01(ks.leanSeed,92,1))*3.141592653589793/180;n=unit(seg->axis*std::cos(th)+unit(across)*std::sin(th));}}
     plane.normal=n;plane.a=plane.b=ts->center;TearForce force{seg,ts,&plane};force.planeOnly=pass==0;
     TearGuide forced;forced.tear_radius=.008;
     const std::set<Cell> where{Grid::tileKey(piece_session::cellOf(grid,ts->center))};
     std::unordered_map<Cell,std::pair<bool,core::Tile>,core::CellHash> ksJournal;auto ksPriorSever=attemptSever->failed;
     const auto ksPriorHistory=historyCuts.size();const auto ksPriorLimits=historyLimits;grid.journal=&ksJournal;
     try{verdict=damageTear(grid,i.asset->sever_segments,*attemptSever,where,2.,tearCuts,i.asset->tissue_field.get(),nullptr,nullptr,&forced,&force,bulletTissueTear?i.asset->closure_field.get():nullptr,&d);}
     catch(const RecoverableImpactError&){grid.journal=nullptr;historyCuts.resize(ksPriorHistory);historyLimits=ksPriorLimits;attemptSever->failed.swap(ksPriorSever);
      for(auto&e:ksJournal){if(e.second.first)grid.tiles[e.first]=e.second.second;else grid.tiles.erase(e.first);}attemptSever->invalidate();verdict=DamageTearVerdict{};continue;}
     catch(...){grid.journal=nullptr;throw;}
     grid.journal=nullptr;
     if(pass==0&&!verdict.tear&&verdict.records.empty())continue; // the damage's plane did not part the limb: today's plane
     through=pass==0;break;}
    if(tear_decide::logOn())std::cerr<<"kill_sever_tear seq="<<ks.sequence<<" through_damage="<<through<<" tear="<<verdict.tear<<" refusal="<<(verdict.refusal?verdict.refusal:"")
     <<" wound_px="<<verdict.woundPixels<<" cells="<<verdict.cells<<" ms="<<verdict.ms<<std::endl;
    tearCuts.clear();
    if(!verdict.records.empty()){
     for(auto&r:verdict.records){if(historyRoom())remember(std::move(r),ks.sequence,ks.mapping,ResolvedTunnel::Role::tear);else historyLimits|=resolved_cut_capacity;}
     if(verdict.edgeCells)historyLimits|=resolved_cut_edge_repairs;
     dirty.insert(verdict.dirty.begin(),verdict.dirty.end());changed=true;
     result.removed_mask_cells+=verdict.mask;result.removed_solid_cells+=verdict.cells;
     for(unsigned s=0;s<64;++s)result.removed_m3_by_slot[s]+=double(verdict.bySlot[s])*grid.pitch*grid.pitch*grid.pitch;}
    if(verdict.tear&&(verdict.remainingPixels||verdict.woundPixels)){result.sever=true;result.sever_dangling=false;result.sever_bone=verdict.bone;
     result.sever_point=point(verdict.point);result.sever_axis=point(verdict.axis);result.sever_remaining_tissue=verdict.remainingTissue;
     result.sever_remaining_bone=verdict.remainingBone;result.sever_cells=verdict.cells;noteTornEnd(result,verdict);}}
   if(!killSevers.empty())jobMark("kill_severs");
   // One sever check for the explosion's craters: tear every station they failed (weakest first, up to 12), each
   // reported on one of the batch's crater results.
   if(!blastDirtyList.empty()){std::sort(blastDirtyList.begin(),blastDirtyList.end());
    blastDirtyList.erase(std::unique(blastDirtyList.begin(),blastDirtyList.end()),blastDirtyList.end());
    blastDirty.insert(blastDirtyList.begin(),blastDirtyList.end());std::vector<Cell>().swap(blastDirtyList);}
   bool bodyBurst=false,bodyRoll=false;
   bool burstReported=false;std::size_t burstOwn=0,burstCells=0;std::uint16_t burstBone=65535;Vec burstAt{},burstAxis{};std::array<std::size_t,64> burstBySlot{};
   if(mask_only_mode_value()==3&&!tearRequests.empty()&&historyBase&&!i.asset->sever_segments.empty()&&!damage_modes::noSever(dmode)&&
      !std::getenv("GFY_NO_BODY_BURST")){
    std::uint64_t rs=tearRequests.front().seed^(blastSequence*0x9E3779B97F4A7C15ull)^0xC4A7CE5Eull;
    if(const char*e=std::getenv("GFY_BODY_ROLL_SEED"))rs^=std::strtoull(e,nullptr,10)*0xD6E8FEB86659FD93ull;
    const double chance=double(body_burst_chance_value()),u=body_burst_draw(rs);bodyRoll=u<chance;
    if(std::getenv("GFY_M3_LOG")||std::getenv("GFY_TEAR_SELECT_LOG"))std::cerr<<"body_burst_roll chance="<<chance<<" u="<<u<<" burst="<<(bodyRoll?1:0)<<std::endl;}
   if(bodyRoll){
    // the explosion: the point nearest every crater's ray and every forced site's push line (least squares; the rays leave it)
    double M[3][3]={},v[3]={};
    auto line=[&](const Vec&o,const Vec&dd){const double D[3]={dd.x,dd.y,dd.z},O[3]={o.x,o.y,o.z};
     for(int r=0;r<3;++r)for(int c=0;c<3;++c){const double m=(r==c?1.:0.)-D[r]*D[c];M[r][c]+=m;v[r]+=m*O[c];}};
    for(const auto&ray:blastRays)line(ray.first,ray.second);for(const auto&q:tearRequests)if(length(q.push)>1e-9)line(q.point,unit(q.push));
    const double det=M[0][0]*(M[1][1]*M[2][2]-M[1][2]*M[2][1])-M[0][1]*(M[1][0]*M[2][2]-M[1][2]*M[2][0])+M[0][2]*(M[1][0]*M[2][1]-M[1][1]*M[2][0]);
    Vec E=blastRays.empty()?tearRequests.front().point:blastRays.front().first-blastRays.front().second*.5; // grenadeburst: a job without craters (set below)
    if(std::abs(det)>1e-9){auto col=[&](int k,int r){return k==r?v[r]:0.;};(void)col;double X[3];
     for(int k=0;k<3;++k){double A[3][3];for(int r=0;r<3;++r)for(int c=0;c<3;++c)A[r][c]=c==k?v[r]:M[r][c];
      X[k]=(A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1])-A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])+A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]))/det;}
     E=Vec{X[0],X[1],X[2]};}
    // the point it hit hardest: the crater nearest the explosion; its proximity as the blast severs read it (reach 2.5 m, start .35)
    Vec B0=blastRays.empty()?tearRequests.front().point:blastRays.front().first;double dmin=1e30;for(const auto&ray:blastRays){const double dd=length(ray.first-E);if(dd<dmin){dmin=dd;B0=ray.first;}}
    double pmax=std::clamp(1-dmin/2.5,0.,1.);const TearRequest*hardest=&tearRequests.front();
    // grenadeburst: tear requests alone (the game's batch). The request the blast hit hardest (the highest proximity: hit_power =
    // 100 x the blast severs' proximity, PainterVoxelLive_impl.h) gives the part and the push (the explosion lies back along it).
    // The point the blast hit is that part's skin toward the explosion: back from the request's point against the push while
    // there is tissue (at most 30 cm); its proximity is the request's plus that way (the craters' reading: the skin, reach 2.5 m).
    if(blastRays.empty()){double ptop=-1;
     for(const auto&q:tearRequests){const double p=out->impacts[q.result].hit_power*.01;if(p>ptop){ptop=p;hardest=&q;}}
     const Vec push=length(hardest->push)>1e-9?unit(hardest->push):Vec{0,-1,0};double back=0;
     if(const auto*cf=i.asset->closure_field.get())if(cf->available)for(double t=.004;t<=.30;t+=.004){const Vec q=hardest->point-push*t;
      const unsigned l=cf->sample(q.x,q.y,q.z)&63u;if(l==0||l==16)break;back=t;}
     B0=hardest->point-push*back;pmax=std::clamp(std::clamp(ptop,0.,1.)+back/2.5,0.,1.);dmin=2.5*(1-pmax);E=B0-push*std::max(dmin,.05);}
    const double sigma=std::clamp((pmax-.35)/.65,0.,1.);
    if(sigma>0){
     const auto*cl=i.asset->closure_field.get();
     auto tissueAt=[&](const Vec&q){if(!cl||!cl->available)return true;const unsigned l=cl->sample(q.x,q.y,q.z)&63u;return l!=0&&l!=16;};
     const std::uint64_t bseed=core::quantMix(tearRequests.front().seed^(blastSequence*0x9E3779B97F4A7C15ull)^0xB0D7ull);
     const auto plan=head_burst::bodyPlan(E,B0,sigma,bseed,tissueAt,dmode==2?damage_modes::extremeSize:1.); // modes: Extreme doubles the body burst
     grid.journal=nullptr;auto bb=head_burst::burst(grid,i.asset->sever_segments,*attemptSever,plan);
     if(std::getenv("GFY_M3_LOG")||std::getenv("GFY_TEAR_SELECT_LOG")){const auto&pl=bb.plan;
      std::cerr<<"body_burst explosion=("<<E.x<<","<<E.y<<","<<E.z<<") hit=("<<B0.x<<","<<B0.y<<","<<B0.z<<") distance_m="<<dmin<<" proximity="<<pmax
       <<" strength="<<pl.sigma<<" rf_mm="<<pl.rf*1000<<" near="<<pl.nNear<<" groups="<<pl.groups<<" rcut_mm="<<pl.Rcut*1000<<" tiles="<<bb.tiles<<" mixed_tiles="<<bb.mixedTiles
       <<" labelled="<<bb.labelled<<" erased="<<bb.erased<<" edge_cells="<<bb.edgeCells<<" samples="<<bb.samples<<" label_ms="<<bb.labelMs<<" erase_ms="<<bb.eraseMs
       <<" repair_ms="<<bb.repairMs<<" ms="<<bb.ms<<" refusal="<<(bb.refusal?bb.refusal:"")<<std::endl;}
     if(!bb.records.empty()){const Mapping mapping0{};const Mapping&recordAt=blastSequence?blastMapping:mapping0;
      for(auto&rr:bb.records){if(historyRoom())remember(std::move(rr),blastSequence?blastSequence:tearRequests.front().sequence,recordAt,ResolvedTunnel::Role::tear);else historyLimits|=resolved_cut_capacity;}
      if(bb.edgeCells)historyLimits|=resolved_cut_edge_repairs;
      dirty.insert(bb.dirty.begin(),bb.dirty.end());blastDirty.insert(bb.dirty.begin(),bb.dirty.end());changed=true;}
     if(bb.done){bodyBurst=true;
      ImpactResult*report=nullptr;for(auto&r:out->impacts)if(blastSequences.count(r.sequence)&&r.removed&&!r.sever){report=&r;break;}
      // rpgblast: the forced tears run after the burst (below): no request is marked as replaced, and without a crater result the
      // burst's own sever is reported after them (burstReported)
      burstOwn=hardest->result;burstBone=hardest->bone;burstCells=bb.cells;burstAt=plan.B;burstAxis=unit(plan.B-E);burstBySlot=bb.bySlot;
      if(report){auto&r=*report;r.sever=true;r.sever_dangling=false;r.sever_bone=hardest->bone;r.sever_point=point(plan.B);
       r.sever_axis=point(unit(plan.B-E));r.sever_remaining_tissue=0;r.sever_remaining_bone=0;r.sever_cells=bb.cells;burstReported=true;}}}}
   if(!bodyBurst&&!blastDirty.empty()&&historyBase&&!i.asset->sever_segments.empty()&&!damage_modes::noSever(dmode)){
    grid.journal=nullptr;std::set<Cell> changedTiles=blastDirty;std::vector<StationRef> touchedStations;
    // Grenade tears follow the blades that crossed a limb; the rest get a seeded tilt (DamageTear.h).
    TearGuide guide;guide.exact_only=bladeSlabBatch;guide.tilt=!bladeSlabBatch;guide.planes=slabPlanes;
    // R62: a blast's severs are the sites it picked by force (BlastSevers.h, torn below); its craters tear a limb only where
    // they left no flesh to hold it (nothing still drawn hangs by an invisible thread).
    guide.flesh_gone_only=!bladeSlabBatch&&!std::getenv("GFY_BLAST_CRATER_SEVERS");
    for(const auto&e:bladeSlots){if(bladeSlabBatch)break;const auto&slot=e.second;if(slot.n<2)continue;const Vec along=slot.b-slot.a;if(length(along)<.02||length(slot.dir)<1e-9)continue;
     const Vec normal=cross(unit(along),unit(slot.dir));if(length(normal)<.2)continue;
     guide.planes.push_back({(slot.a+slot.b)*.5,unit(normal),slot.a,slot.b});}
    for(unsigned round=0;round<12;++round){
     std::vector<const ResolvedTunnel*> tearCuts;
     {std::vector<const ResolvedCutHistory*> chain;for(auto p=historyBase.get();p;p=p->parent.get())chain.push_back(p);
      for(auto it=chain.rbegin();it!=chain.rend();++it)for(const auto&r:(*it)->appended)tearCuts.push_back(&r);}
     for(const auto&r:historyCuts)tearCuts.push_back(&r);
     const Vec blastDir=length(blastMapping.direction)>1e-9?unit(blastMapping.direction):Vec{0,0,0};
     auto verdict=damageTear(grid,i.asset->sever_segments,*attemptSever,blastDirty,2.,tearCuts,i.asset->tissue_field.get(),&changedTiles,&touchedStations,&guide,nullptr,i.asset->closure_field.get(),&blastDir);
     if(!guide.exact_only&&!verdict.tear&&verdict.refusal){ // the angled tear did not free the limb (it reached into wider tissue): keep its rips, tear square
      if(!verdict.records.empty()){
       for(auto&r:verdict.records){if(historyRoom())remember(std::move(r),blastSequence,blastMapping,ResolvedTunnel::Role::tear);else historyLimits|=resolved_cut_capacity;}
       if(verdict.edgeCells)historyLimits|=resolved_cut_edge_repairs;
       dirty.insert(verdict.dirty.begin(),verdict.dirty.end());blastDirty.insert(verdict.dirty.begin(),verdict.dirty.end());changed=true;}
      std::set<Cell> again=verdict.dirty;tearCuts.clear();
      {std::vector<const ResolvedCutHistory*> chain;for(auto p=historyBase.get();p;p=p->parent.get())chain.push_back(p);
       for(auto it=chain.rbegin();it!=chain.rend();++it)for(const auto&r:(*it)->appended)tearCuts.push_back(&r);}
      for(const auto&r:historyCuts)tearCuts.push_back(&r);
      TearGuide square;square.flesh_gone_only=guide.flesh_gone_only; // the square retry keeps the flesh-gone rule
      verdict=damageTear(grid,i.asset->sever_segments,*attemptSever,blastDirty,2.,tearCuts,i.asset->tissue_field.get(),&again,&touchedStations,square.flesh_gone_only?&square:nullptr,nullptr,i.asset->closure_field.get(),&blastDir);}
     changedTiles=verdict.dirty; // later rounds re-measure only what this tear changed
     tearCuts.clear();
     if(!verdict.records.empty()){
      for(auto&r:verdict.records){if(historyRoom())remember(std::move(r),blastSequence,blastMapping,ResolvedTunnel::Role::tear);else historyLimits|=resolved_cut_capacity;}
      if(verdict.edgeCells)historyLimits|=resolved_cut_edge_repairs;
      dirty.insert(verdict.dirty.begin(),verdict.dirty.end());blastDirty.insert(verdict.dirty.begin(),verdict.dirty.end());changed=true;}
     if(!verdict.tear)break;
     for(auto&r:out->impacts)if(blastSequences.count(r.sequence)&&r.removed&&!r.sever){
      r.sever=true;r.sever_dangling=false;r.sever_bone=verdict.bone;r.sever_point=point(verdict.point);r.sever_axis=point(verdict.axis);
      r.sever_remaining_tissue=verdict.remainingTissue;r.sever_remaining_bone=verdict.remainingBone;r.sever_cells=verdict.cells;noteTornEnd(r,verdict);break;}}
    // R61: a torn blade that crossed a limb (within 60 degrees of square) and is open there on its sheet tears it even when
    // no station failed by its tissue share (the sheet's relief spreads the kerf over several 12 mm stations): one tear at
    // the station nearest the blade's plane, where the blade's damage reached the station.
    if(bladeSlabBatch&&!attemptSever->failed.empty())for(const auto&pl:slabPlanes){if(!pl.sheet)continue;std::size_t k0=0;
     for(const auto&s:i.asset->sever_segments){const std::size_t base=k0;k0+=s.stations.size();
      const double c=dot(s.axis,pl.normal);if(std::abs(c)<.5||s.stations.empty())continue;
      const double at=dot(pl.point-s.pivot,pl.normal)/c;std::size_t best=s.stations.size();double gap=1e30;
      for(std::size_t q=0;q<s.stations.size();++q){const double d=std::abs(s.stations[q].s-at);if(d<gap){gap=d;best=q;}}
      if(best==s.stations.size()||gap>.012||base+best>=attemptSever->failed.size()||attemptSever->failed[base+best])continue;
      const auto&station=s.stations[best];const Vec ab=pl.b-pl.a;const double L2=dot(ab,ab);
      const double along=L2>1e-12?std::clamp(dot(station.center-pl.a,ab)/L2,0.,1.):0.;if(length(station.center-(pl.a+ab*along))>s.radius+.04)continue;
      bool reached=false;for(int tx=station.tileLo.x;tx<=station.tileHi.x&&!reached;++tx)for(int ty=station.tileLo.y;ty<=station.tileHi.y&&!reached;++ty)
       for(int tz=station.tileLo.z;tz<=station.tileHi.z;++tz)if(blastDirty.count(Cell{tx,ty,tz})){reached=true;break;}
      if(!reached)continue;
      std::vector<const ResolvedTunnel*> tearCuts;
      {std::vector<const ResolvedCutHistory*> chain;for(auto p=historyBase.get();p;p=p->parent.get())chain.push_back(p);
       for(auto it=chain.rbegin();it!=chain.rend();++it)for(const auto&r:(*it)->appended)tearCuts.push_back(&r);}
      for(const auto&r:historyCuts)tearCuts.push_back(&r);
      const TearForce force{&s,&station,&pl};
      const Vec forceDir=length(blastMapping.direction)>1e-9?unit(blastMapping.direction):Vec{0,0,0};
      auto verdict=damageTear(grid,i.asset->sever_segments,*attemptSever,blastDirty,2.,tearCuts,i.asset->tissue_field.get(),&changedTiles,&touchedStations,&guide,&force,i.asset->closure_field.get(),&forceDir);
      tearCuts.clear();
      if(!verdict.records.empty()){
       for(auto&r:verdict.records){if(historyRoom())remember(std::move(r),blastSequence,blastMapping,ResolvedTunnel::Role::tear);else historyLimits|=resolved_cut_capacity;}
       if(verdict.edgeCells)historyLimits|=resolved_cut_edge_repairs;
       dirty.insert(verdict.dirty.begin(),verdict.dirty.end());blastDirty.insert(verdict.dirty.begin(),verdict.dirty.end());changed=true;}
      if(verdict.tear)for(auto&r:out->impacts)if(blastSequences.count(r.sequence)&&r.removed&&!r.sever){
       r.sever=true;r.sever_dangling=false;r.sever_bone=verdict.bone;r.sever_point=point(verdict.point);r.sever_axis=point(verdict.axis);
       r.sever_remaining_tissue=verdict.remainingTissue;r.sever_remaining_bone=verdict.remainingBone;r.sever_cells=verdict.cells;noteTornEnd(r,verdict);break;}}}
   }
   // R62 (BlastSevers.h): the sites the blast picked by force tear, each at the station nearest the point it hit hardest, the
   // tear plane leaning 10-35 degrees toward the push and the torn ends leaning with it (TissueTear.h). A site already torn there
   // (its nearest station failed) is left; a sever is reported on one of the batch's crater results, else on the request itself.
   if(!tearRequests.empty()&&historyBase&&!i.asset->sever_segments.empty()&&!damage_modes::noSever(dmode)){ // rpgblast: also after a body burst (the two run together)
    grid.journal=nullptr;std::size_t stationTotal=0;for(const auto&s:i.asset->sever_segments)stationTotal+=s.stations.size();
    if(attemptSever->failed.size()!=stationTotal)attemptSever->failed.assign(stationTotal,0);
    for(const auto&req:tearRequests){auto&own=out->impacts[req.result];
     const SeverSegment*seg=nullptr;const SeverStation*st=nullptr;double best=1e30;bool torn=false;std::size_t k0=0;
     for(const auto&s:i.asset->sever_segments){const std::size_t base=k0;k0+=s.stations.size();if(s.bone!=req.bone)continue;
      for(std::size_t q=0;q<s.stations.size();++q){const double d=length(s.stations[q].center-req.point);
       if(d<best){best=d;seg=&s;st=&s.stations[q];torn=attemptSever->failed[base+q]!=0;}}}
     // T1 (2026-10-29): pieces tear by force too (TissueTear works on a piece's owned grid; R62 refused: "tear_request_piece")
     if(!seg){own.rejection="tear_request_no_station";continue;}
     if(torn){own.rejection="tear_request_already_torn";continue;}
     const bool trunkTear=seg->radius>=.16;
     Vec n=seg->axis;if(!trunkTear){const Vec across=req.push-n*dot(req.push,n);
      if(length(across)>1e-6){const double th=(10.+25.*damage_tear::hash01(req.seed,91,1))*3.141592653589793/180;n=unit(seg->axis*std::cos(th)+unit(across)*std::sin(th));}}
     TearPlane plane;plane.point=st->center;plane.normal=n;plane.a=plane.b=st->center;TearForce force{seg,st,&plane};force.wholeSection=trunkTear;
     TearGuide forced;forced.tear_radius=.008;
     std::vector<const ResolvedTunnel*> tearCuts;
     {std::vector<const ResolvedCutHistory*> chain;for(auto p=historyBase.get();p;p=p->parent.get())chain.push_back(p);
      for(auto it=chain.rbegin();it!=chain.rend();++it)for(const auto&r:(*it)->appended)tearCuts.push_back(&r);}
     for(const auto&r:historyCuts)tearCuts.push_back(&r);
     // grenadeburst: a body with no earlier cut (a fresh NPC killed by the grenade: the game sends the tear requests before the
     // blast's craters) tears like a cut one. damageTear reads a forced tear's cuts for its seed only (it selects nothing and
     // returns at once on an empty list): the request stands in as the one cut, so every blast tears its own way. (Before: the
     // refusal "tear_request_no_cut" here - no limb tear and no torso split on a fresh body.)
     ResolvedTunnel seedCut;if(tearCuts.empty()){seedCut.role=ResolvedTunnel::Role::tear;seedCut.edge_seed=req.seed;seedCut.sequence=req.sequence;tearCuts.push_back(&seedCut);}
     const std::set<Cell> where{Grid::tileKey(piece_session::cellOf(grid,st->center))};const Vec push=req.push;
     auto verdict=damageTear(grid,i.asset->sever_segments,*attemptSever,blastDirty.empty()?where:blastDirty,2.,tearCuts,i.asset->tissue_field.get(),nullptr,nullptr,&forced,&force,i.asset->closure_field.get(),&push);
     tearCuts.clear();
     const Mapping mapping0{};const Mapping&recordAt=blastSequence?blastMapping:mapping0;
     if(!verdict.records.empty()){
      for(auto&r:verdict.records){if(historyRoom())remember(std::move(r),req.sequence,recordAt,ResolvedTunnel::Role::tear);else historyLimits|=resolved_cut_capacity;}
      if(verdict.edgeCells)historyLimits|=resolved_cut_edge_repairs;
      dirty.insert(verdict.dirty.begin(),verdict.dirty.end());blastDirty.insert(verdict.dirty.begin(),verdict.dirty.end());changed=true;}
     if(!verdict.tear){own.rejection=std::string("tear_request_")+(verdict.refusal?verdict.refusal:"refused");continue;}
     if(!verdict.remainingPixels&&!verdict.woundPixels){own.rejection="tear_request_no_tissue";continue;} // nothing there to tear
     ImpactResult*report=nullptr;for(auto&r:out->impacts)if(blastSequences.count(r.sequence)&&r.removed&&!r.sever){report=&r;break;}
     if(!report){double nearest=std::numeric_limits<double>::infinity();std::uint32_t face=0;Vec p;i.asset->source_tree.nearest(0,verdict.point,nearest,face,p);
      if(std::isfinite(nearest))own.entry_surface=woundSurface(i.asset->source,face,p);
      own.removed=own.entry_surface.valid;own.radius=own.exit_radius=.02;own.removed_solid_cells=verdict.cells;
      for(unsigned s=0;s<64;++s)own.removed_m3_by_slot[s]=double(verdict.bySlot[s])*grid.pitch*grid.pitch*grid.pitch;report=&own;}
     auto&r=*report;r.sever=true;r.sever_dangling=false;r.sever_bone=verdict.bone;r.sever_point=point(verdict.point);r.sever_axis=point(verdict.axis);
     r.sever_remaining_tissue=verdict.remainingTissue;r.sever_remaining_bone=verdict.remainingBone;r.sever_cells=verdict.cells;noteTornEnd(r,verdict);}
   }
   // rpgblast: a body burst in a job without a crater result (the game's batch) whose forced tears reported no sever: its own sever,
   // on the hardest request's result, as a forced tear reports its own - the game's sever hold, its split check and the held
   // craters wait for a sever of this job
   if(bodyBurst&&!burstReported){bool any=false;for(const auto&r:out->impacts)any=any||r.sever;
    if(!any){auto&own=out->impacts[burstOwn];double nearest=std::numeric_limits<double>::infinity();std::uint32_t face=0;Vec p;
     i.asset->source_tree.nearest(0,burstAt,nearest,face,p);if(std::isfinite(nearest))own.entry_surface=woundSurface(i.asset->source,face,p);
     own.rejection.clear();own.removed=own.entry_surface.valid;own.radius=own.exit_radius=.02;own.removed_solid_cells=burstCells;
     for(unsigned s=0;s<64;++s)own.removed_m3_by_slot[s]=double(burstBySlot[s])*grid.pitch*grid.pitch*grid.pitch;
     own.sever=true;own.sever_dangling=false;own.sever_bone=burstBone;own.sever_point=point(burstAt);own.sever_axis=point(burstAxis);
     own.sever_remaining_tissue=0;own.sever_remaining_bone=0;own.sever_cells=burstCells;}}
   // seam (value 3, SeamTear.h): after the job's damage (bullets, fragments, melee, blasts and their tears) the wound tracks of this
   // grid are searched for a line of hits; a weak enough seam splits the part along it (a sever, reported on the job's last wound) or
   // opens a deep split that later hits along the line extend and complete
   bool seamSevered=false;for(const auto&r:out->impacts)seamSevered=seamSevered||r.sever; // a job that tore a part leaves seams to the next one
   if(changed&&!seamSevered&&mask_only_mode_value()==3&&historyBase&&!i.asset->sever_segments.empty()&&!damage_modes::noSever(dmode)){
    std::set<std::uint64_t> jobSeq;for(const auto&r:historyCuts)if(r.role==ResolvedTunnel::Role::primary)jobSeq.insert(r.sequence);
    if(!jobSeq.empty()){grid.journal=nullptr;
     std::vector<const ResolvedTunnel*> seamCuts;
     {std::vector<const ResolvedCutHistory*> chain;for(auto p=historyBase.get();p;p=p->parent.get())chain.push_back(p);
      for(auto it=chain.rbegin();it!=chain.rend();++it)for(const auto&r:(*it)->appended)seamCuts.push_back(&r);}
     for(const auto&r:historyCuts)seamCuts.push_back(&r);
     auto so=seam_tear::check(grid,*attemptSever,i.asset->closure_field.get(),seamCuts,[&](std::uint64_t s){return jobSeq.count(s)!=0;},i.asset->sever_segments);
     seamCuts.clear();
     if(so.full||so.open){auto&verdict=so.v;const Mapping seamMapping{};const std::uint64_t seamSeq=*jobSeq.rbegin();
      for(auto&r:verdict.records){if(historyRoom())remember(std::move(r),seamSeq,seamMapping,ResolvedTunnel::Role::tear);else historyLimits|=resolved_cut_capacity;}
      if(verdict.edgeCells)historyLimits|=resolved_cut_edge_repairs;
      dirty.insert(verdict.dirty.begin(),verdict.dirty.end());changed=true;
      if(so.full){ImpactResult*report=nullptr;for(auto it=out->impacts.rbegin();it!=out->impacts.rend();++it)if(it->removed&&!it->sever){report=&*it;break;}
       if(!report)for(auto it=out->impacts.rbegin();it!=out->impacts.rend();++it)if(it->removed){report=&*it;break;}
       if(report){auto&r=*report;r.sever=true;r.sever_dangling=false;r.sever_bone=so.bone;r.sever_point=point(verdict.point);r.sever_axis=point(verdict.axis);
        r.sever_remaining_tissue=so.rt;r.sever_remaining_bone=so.rb;r.sever_cells=verdict.cells;noteTornEnd(r,verdict);}}}}}
   if(dentsAdded)changed=true; // melee: a dent publishes even when nothing was carved
   jobMark("blast");removalTime.finish();stages.mark("remove");out->no_geometry_change=!changed;
   out->irregular_edges=changed&&usedRough;
   // Allocate history before publication. A newly unsupported air-only mask
   // edit needs a node even when the existing geometry changed flag is false.
   // Actual no-mask-change jobs share the prior node.
   candidate_history=historyBase;
   if(historyBase&&(!historyCuts.empty()||historyLimits!=historyBase->limitations)){
    auto history=ResolvedCutHistory::create();history->parent=historyBase;history->appended=std::move(historyCuts);
    history->owner=historyBase->owner;history->damage_revision=task.stamp.revision;
    history->total_cuts=historyBase->total_cuts+history->appended.size();history->limitations=historyLimits;
    history->grid_origin=historyBase->grid_origin;history->grid_size=historyBase->grid_size;history->grid_pitch=historyBase->grid_pitch;
    candidate_history=std::move(history);
   }
   jobMark("history");
   fastwound::RegionList regionResults=i.region_cache;
   std::shared_ptr<JointMeasuresBeside> jointsBeside; // R68 D5
   const bool incremental=!i.asset->face_children.empty()&&!grid.ownership&&i.asset->fast&&i.asset->fast->valid;
   bool analyticBuilt=false;
   // Mask-only wounds: a sever (tear, head fracture, blast tear) in this job, or earlier on this body, keeps today's full path.
   {bool severNow=false;for(const auto&r:out->impacts)severNow=severNow||r.sever;
    if(candidate_history&&candidate_history!=historyBase)for(const auto&c:candidate_history->appended)severNow=severNow||c.role==ResolvedTunnel::Role::tear;
    if(severNow&&mask_only_mode.load(std::memory_order_relaxed)<2)i.mask_only_off=true;}
   const bool maskOnlyMode=mask_only_mode.load(std::memory_order_relaxed)!=0&&!i.piece&&!grid.ownership;
   const bool maskOnly=changed&&maskOnlyMode&&!i.mask_only_off;
   out->mask_only=maskOnly?1:(changed&&maskOnlyMode?2:0);
   if(changed&&i.piece){ // a separated piece publishes on the low-poly path only (PieceSession_impl.h)
    try{if(i.piece_published&&i.piece_published->mask_piece){PieceSessionOps::packMask(i,grid,out,workerPhases,candidate_history);out->mask_only=1;}
     else PieceSessionOps::pack(i,grid,candidate_history,candidate_channels,out,workerPhases);analyticBuilt=true;}
    catch(const std::bad_alloc&){throw;}catch(const MemoryShortage&){throw;}catch(const AdmissionCanceled&){throw;}
    catch(const std::exception&e){throw RecoverableImpactError(std::string("piece_publication: ")+e.what());}}
   else if(changed&&i.asset->analytic_channels&&i.analytic_disabled.load(std::memory_order_relaxed))out->analytic_refusal="renderer_refused_for_owner";
   else if(changed&&i.asset->analytic_channels){
    if(incremental&&candidate_history&&i.asset->closure_field&&!(candidate_history->limitations&~unsigned(resolved_cut_edge_repairs))){
     try{GFY_WORKER_TEST_GATE();admitJoin(i,memory,0,i.asset->source.f.size(),i.channel_cache?i.channel_cache->triangles:0,1);
      // R68 D5: the joint stage's measures beside the pack (both only read the grid); an exception leaving the pack closes the
      // claims and waits for the measures in progress before it travels on
      jointsBeside=JointMeasuresBeside::start(i,grid,dirty);
      try{if(maskOnly){packMaskOnlyCandidate(i,grid,out,workerPhases,&dirty);
        // slice 4: every decide-once tear of this body's history rides on the publication, unsplit (the separation splits them)
        {std::vector<const ResolvedCutHistory*> chain;for(const ResolvedCutHistory*h=candidate_history.get();h;h=h->parent.get())chain.push_back(h);
         for(auto h=chain.rbegin();h!=chain.rend();++h)for(const auto&r:(*h)->appended)if(r.surface&&r.surface->decide)out->mesh.tears.push_back({r.surface->decide,255});}}
       else packAnalyticCandidate(i,grid,candidate_history,candidate_channels,out,workerPhases,&dirty);}catch(...){if(jointsBeside)jointsBeside->finish(true);throw;}
      if(jointsBeside)jointsBeside->finish(false);analyticBuilt=true;
      boundary_cache.clear();regionResults.clear();i.boundary_valid=false;
     }catch(const std::bad_alloc&){throw;}catch(const MemoryShortage&){throw;}catch(const AdmissionCanceled&){throw;}
      catch(const std::exception&e){out->analytic_refusal=e.what();out->mesh={};out->analytic_diagnostic=false;}
    }else out->analytic_refusal=i.asset->closure_field?"unsupported_history_or_source_baseline_required":"closure_field_unavailable";
   }
   if(changed&&analyticBuilt){}
   else if(changed&&incremental){GFY_WORKER_TEST_GATE();packIncrementalCandidate(i,memory,grid,dirty,boundary_cache,regionResults,out,workerPhases,stages);}
   else if(changed){GFY_WORKER_TEST_GATE();PhaseTime boundaryTime(workerPhases.boundary_ms);need(grid.tiles.size()<=262144,"owner_sparse_tile_budget");for(auto tile:dirty){auto quads=core::dirtyBoundary(grid,{tile});if(quads.empty())boundary_cache.erase(tile);else boundary_cache[tile]=std::make_shared<const std::vector<core::Quad>>(std::move(quads));}std::vector<core::Quad>boundary;for(auto&item:boundary_cache)boundary.insert(boundary.end(),item.second->begin(),item.second->end());boundary=core::mergeBoundaryQuads(grid,boundary);boundaryTime.finish();stages.mark("boundary");regionResults.clear();
    admitJoin(i,memory,boundary.size(),std::min<std::size_t>(i.asset->source.f.size(),grid.tiles.size()*32),i.committed->impl_->geometry?i.committed->impl_->geometry->triangles:0,1);
    packWholeCandidate(i,memory,grid,boundary,out,workerPhases,candidate_memo,i.exterior_memo.get(),stages); }
   jobMark("pack");out->mesh.tissue_field=analyticBuilt?i.asset->closure_field:i.asset->tissue_field;
   if(changed&&i.job_dents_on&&out->mesh.mask_volume){out->mesh.dents=i.job_dents;out->mesh.slits=i.job_slits;} // melee: dents + slits ride on every value-3 publication
   if(changed&&!i.asset->accessory_surface_bytes.empty()){
    if(painter_ownersafe_fault("job_generic"))throw std::runtime_error("ownersafe_test_job_generic"); // ownersafe test switch
    out->accessory_skipped=clipOcularAccessories(*i.asset,grid,i.committed->impl_->geometry?&i.committed->impl_->geometry->mesh:i.piece_published.get(),out->mesh,bool(i.piece),&dirty);
    out->triangles=0;for(const auto&child:out->mesh.children)out->triangles+=child.indices.size()/3;}
   if(changed&&!i.piece){
    // a joint whose tile box holds no dirty tile reads exactly the committed tiles: keep its committed value.
    const std::vector<JointDamage>* prevJoints=nullptr;
    if(true&&!grid.ownership&&i.committed->impl_->geometry&&i.committed->impl_->geometry->joints.size()==i.asset->joints.size())prevJoints=&i.committed->impl_->geometry->joints;
    if(prevJoints&&i.spare_joint_tiles.size()!=i.asset->joints.size())JointMeasuresBeside::cacheJointTiles(i,grid);
    // R68: the touched joints' measures and the cranial measure side by side (each a pure function of the grid, stored at its
    // index; the joints keep their order). D5: measured beside the analytic pack when that ran them all for this touched set.
    const std::size_t J=i.asset->joints.size();std::vector<char> touched(J,1);
    for(std::size_t j=0;j<J;++j)if(prevJoints){const auto&b=i.spare_joint_tiles[j];bool t=false;
      for(auto it=dirty.lower_bound(Cell{b[0],INT_MIN,INT_MIN});it!=dirty.end()&&it->x<=b[3]&&!t;++it)
       t=it->y>=b[1]&&it->y<=b[4]&&it->z>=b[2]&&it->z<=b[5];touched[j]=t;}
    if(jointsBeside&&jointsBeside->done&&jointsBeside->touched==touched){jointsBeside->rethrow();
     for(std::size_t j=0;j<J;++j)out->joints.push_back(touched[j]?jointsBeside->values[j]:(*prevJoints)[j]);out->cranial=jointsBeside->cranial;}
    else{
    // the cranial measure's two halves (tissue J, brain J+1) are tasks of their own, first (the longest); combined below exactly
    // as CranialMeasure::evaluate combines them
    const auto&cm=i.asset->cranial_measure;std::vector<std::size_t> tasks;if(cm.valid){tasks.push_back(J);tasks.push_back(J+1);}
    for(std::size_t j=0;j<J;++j)if(touched[j])tasks.push_back(j);
    std::vector<JointDamage> values(J);JointDamage cranialTissue,cranialBrain;std::atomic<std::size_t> next{0};
    if(!tasks.empty())core::parallelChunks(std::min<std::size_t>(tasks.size(),core::parallelWidth()),1,[&](std::size_t,std::size_t,std::size_t){
     for(std::size_t k;(k=next.fetch_add(1,std::memory_order_relaxed))<tasks.size();){const auto j=tasks[k];
      if(j==J)cranialTissue=cm.tissue.evaluate(grid);else if(j==J+1)cranialBrain=cm.brain.evaluate(grid);else values[j]=i.asset->joints[j].evaluate(grid);}});
    for(std::size_t j=0;j<J;++j)out->joints.push_back(touched[j]?values[j]:(*prevJoints)[j]);
    CranialDamage cranial;
    if(cm.valid){const auto&a=cranialTissue;const auto&b=cranialBrain;cranial.valid=true;
     cranial.original_m3=a.original_m3;cranial.removed_m3=a.removed_m3;cranial.original_bone_m3=a.original_bone_m3;cranial.removed_bone_m3=a.removed_bone_m3;
     cranial.original_brain_m3=b.original_m3;cranial.removed_brain_m3=b.removed_m3;}
    out->cranial=cranial;}}
   jobMark("joints");out->tiles=grid.tiles.size();out->dirty_tiles=dirty.size();
   candidate_sever=std::move(attemptSever);
   if(true&&!i.piece){const auto serial=++i.spare_grid_serial;auto spare=i.spare_spare;
    candidate_grid=std::shared_ptr<const Grid>(new Grid(std::move(grid)),[spare,serial](const Grid*g){std::unique_ptr<Grid> own(const_cast<Grid*>(g)),old;
     {std::lock_guard<std::mutex> lock(spare->mutex);old=std::move(spare->grid);spare->grid=std::move(own);spare->serial=serial;}});
    candidate_spare_serial=serial;candidate_spare_dirty=std::make_shared<const std::vector<Cell>>(dirty.begin(),dirty.end());}
   else candidate_grid=std::make_shared<const Grid>(std::move(grid));
   candidate_boundary=std::move(boundary_cache);candidate_regions=std::move(regionResults);jobMark("finish");
   if(candidate_memo)candidate_memo->basis=candidate_grid; // Only a completed candidate owns a basis.
  }catch(const RecoverableImpactError&e){
    if(attempt==0&&usedRough&&!std::strncmp(e.what(),"joined_boundary_gate",20)){retryReason=e.what();retrySmooth=true;}
    else rejectGeometry(e.what());}
   catch(const core::CavityProvenanceError&e){
    if(attempt==0&&usedRough){retryReason=e.what();retrySmooth=true;}
    else rejectGeometry(e.what());}
   catch(const core::packing::PackingInfluenceError&e){rejectGeometry(e.what());}
   catch(const std::bad_alloc&){throw;} // resource failure: reported by run(), never a terminal error
   // ownersafe (2026-11-15): any other failure of the job is a geometry rejection of this hit too (the transactional no-op above:
   // committed state kept, a multi-hit batch isolated one hit at a time). As an output error it failed the owner for good.
   catch(const std::exception&e){rejectGeometry(e.what());}
   if(!retrySmooth)break;
   // The failed candidate never escaped this worker. Re-run the same events
   // once from committed occupancy and boundary, retaining no partial edits.
   *out=Output{};out->stamp=task.stamp;out->batch_events=activeEvents;
   candidate_channels.reset();candidate_history.reset();candidate_sever.reset();candidate_grid.reset();candidate_boundary.clear();candidate_regions.clear();candidate_memo.reset();
   candidate_spare_serial=0;candidate_spare_dirty.reset();
  }
  if(!geometryRejected||activeEvents==1||!out->error.empty())break;
  // A canceled owner is terminal and never publishes. Do not spend another
  // build (and its smooth retry) on a result that is dropped.
  {std::lock_guard<std::mutex>lock(i.mutex);if(i.canceled)break;}
  // Bounded failure isolation: a failed multi-event batch resolves ONLY its
  // earliest event from committed state. The untouched suffix becomes
  // isolated work drained one event per dispatch; it is never rebuilt as a
  // shrinking multi-event batch. This runs at most once per job.
  batchReason=geometryRejection;activeEvents=1;batchSplits=1;
  }
  out->geometry_retry_reason=std::move(retryReason);out->worker_phases=workerPhases;
  out->geometry_batch_splits=batchSplits;out->geometry_batch_reason=std::move(batchReason);out->isolated_event=task.isolated;
  out->join_estimate=memory.estimate;out->join_quads=memory.quads;out->join_faces=memory.faces;out->join_triangles=memory.triangles;out->join_body_triangles=memory.body;out->join_regions=memory.regions;out->heavy_join=memory.heavy;out->memory_deferrals=task.deferrals;
  std::vector<Impact>deferred;
  try{deferred.reserve(task.events.size()-activeEvents);for(std::size_t n=activeEvents;n<task.events.size();++n)deferred.push_back(std::move(task.events[n]));out->deferred_events=deferred.size();}
  catch(const std::bad_alloc&){throw;}catch(const std::exception&e){out->error=e.what();deferred.clear();}
  if(out->no_geometry_change)out->irregular_edges=false;
  // Allocate on the worker. Owner-side capture only shares this immutable
  // revision; it never copies or scans the full-body anatomy/removal map.
  if(out->error.empty())try{if(painter_ownersafe_fault("job_error"))throw std::runtime_error("ownersafe_test_job_error"); // ownersafe test switch
   auto state=std::make_shared<DamageSnapshot::Impl>();state->asset=i.asset;state->stamp=task.stamp;
   state->grid=candidate_grid?candidate_grid:i.committed->impl_->grid;
   state->history=candidate_grid?candidate_history:i.committed->impl_->history;
   state->geometry=out->no_geometry_change?i.committed->impl_->geometry:out;
   state->sever=candidate_grid&&candidate_sever?std::shared_ptr<const SeverState>(candidate_sever):i.committed->impl_->sever;
   state->piece=i.piece;state->piece_mesh=i.piece_published;
   state->gore=i.committed->impl_->gore;
   int eyeMode=state->gore.eye_remnants_mode;
   if(!out->no_geometry_change)for(const auto& impact:out->impacts)
    if(impact.removed&&impact.rejection.empty()){
     state->gore=impact.gore;
     bool cranial=impact.removed_m3_by_slot[3]>0.||impact.removed_m3_by_slot[14]>0.;
     for(const auto& event:task.events)if(event.sequence==impact.sequence&&event.cranial_bone!=65535&&event.hit_bone==event.cranial_bone)cranial=true;
     if(cranial)eyeMode=impact.gore.eye_remnants_mode;
    }
   state->gore.eye_remnants_mode=eyeMode;
   candidate_snapshot=std::shared_ptr<const DamageSnapshot>(new DamageSnapshot(std::move(state)));
  }catch(const std::bad_alloc&){throw;}catch(const std::exception&e){out->error=e.what();}
  jobMark("snapshot");
#ifdef GFY_OOM_VERIFY_REBUILD
  // Test builds only: the cache rebuilt after a governor eviction must equal
  // the incrementally maintained boundary cache of every candidate, bit for bit.
  if(candidate_grid){const auto&g=*candidate_grid;const bool inc=!i.asset->face_children.empty()&&!g.ownership&&i.asset->fast&&i.asset->fast->valid;
   const auto rebuilt=rebuildBoundaryCache(g,inc);bool same=rebuilt.size()==candidate_boundary.size();std::size_t quads=0;
   for(auto a=rebuilt.cbegin(),b=candidate_boundary.cbegin();same&&a!=rebuilt.end();++a,++b){quads+=a->second->size();
    same=a->first==b->first&&a->second->size()==b->second->size()&&(a->second->empty()||!std::memcmp(a->second->data(),b->second->data(),a->second->size()*sizeof(core::Quad)));}
   std::cerr<<"oom_verify_rebuild revision="<<task.stamp.revision<<" tiles="<<rebuilt.size()<<" quads="<<quads<<(same?" same":" MISMATCH")<<std::endl;
   if(!same)std::abort();}
#endif
  // Only a normal batch can defer, and dispatch runs normal batches only once
  // retry_pending is empty, so this never replaces isolated work. An isolated
  // job defers nothing and leaves the rest of the isolated suffix untouched.
  out->worker_ms=core::ms(start);{std::lock_guard<std::mutex>lock(i.mutex);if(!i.canceled){if(!deferred.empty())i.retry_pending=std::move(deferred);i.ready_channel_cache=std::move(candidate_channels);i.ready_sever=std::move(candidate_sever);i.ready_snapshot=std::move(candidate_snapshot);i.spare_ready_serial=candidate_grid?candidate_spare_serial:0;i.spare_ready_dirty=std::move(candidate_spare_dirty);i.ready_grid=std::move(candidate_grid);i.ready_boundary=std::move(candidate_boundary);i.ready_regions=std::move(candidate_regions);if(out->error.empty()&&candidate_memo&&candidate_memo->basis)i.ready_exterior_memo=std::move(candidate_memo);body_hit_outcomes::output("worker_result",nullptr,i.owner.spawn_cookie,*out);task.trace_transferred=true;i.done=std::move(out);}}
 }
};
std::shared_ptr<const painter_detail::TissueField> Session::tissue_field()const{return impl_->asset->tissue_field;}
bool Session::supports_bone(std::uint16_t bone)const {
 std::lock_guard<std::mutex>lock(impl_->mutex);
 if(bone>=64 || impl_->canceled || impl_->failed)return false;
 for(const auto&vertex:impl_->asset->source.v)for(unsigned k=0;k<4;++k)
  if(vertex.ids[k]==bone && vertex.w[k]!=0)return true;
 return false;
}
Pool::Pool(unsigned count,unsigned maximum):impl_(new Impl(count,maximum)){}Pool::~Pool()=default;
void Pool::quit()noexcept{try{impl_->quit();}catch(...){}}
void quit_workers()noexcept{core::worker_quit::stopAll();} // quitfix (core/join_cavity.hpp worker_quit)
std::unique_ptr<Pool>make_pool(unsigned count,unsigned maximum)noexcept{AllocationScope scope;try{return std::make_unique<Pool>(count,maximum);}catch(const std::exception&e){fail(e.what());return {};}catch(...){fail("unknown_pool_failure");return {};}}
std::shared_ptr<Session>Pool::attach_piece(std::shared_ptr<const PieceSeed>seed,Identity owner,std::shared_ptr<const gfy_packed::Mesh>published,std::uint64_t revision,std::uint64_t generation)noexcept{AllocationScope scope;try{
 // A whole part may separate without any wound-wall children. It has no birth channel field.
 const bool wallLess=published&&!published->children.empty()&&std::none_of(published->children.begin(),published->children.end(),
  [](const gfy_packed::Child&child){return child.material!=0;});
 need(seed&&seed->asset&&seed->grid&&seed->grid->ownership&&seed->history&&seed->topology&&published&&(published->analytic_channels||wallLess),"piece_seed");
 need(owner.spawn_cookie&&owner.visual_lineage&&owner.source==seed->asset->source_hash&&owner.exact_rig==seed->asset->rig_hash,"owner_asset_identity");
 auto&p=*impl_;std::lock_guard<std::mutex>lock(p.mutex);p.owners.erase(std::remove_if(p.owners.begin(),p.owners.end(),[](auto&w){return w.expired();}),p.owners.end());
 if(p.owners.size()>=p.max_owners){fail("owner_capacity");return {};}
 auto s=std::shared_ptr<Session>(new Session(std::make_unique<Session::Impl>(std::move(seed),owner,std::move(published),revision,generation)));p.owners.push_back(s);return s;
}catch(const std::bad_alloc&){fail(outOfMemoryText);return {};}catch(const std::exception&e){fail(e.what());return {};}catch(...){fail("unknown_piece_attach_failure");return {};}}
std::shared_ptr<Session>Pool::attach(std::shared_ptr<const Asset>asset,Identity owner)noexcept{AllocationScope scope;try{need(asset&&owner.spawn_cookie&&owner.visual_lineage&&owner.source==asset->source_hash&&owner.exact_rig==asset->rig_hash,"owner_asset_identity");auto&p=*impl_;std::lock_guard<std::mutex>lock(p.mutex);p.owners.erase(std::remove_if(p.owners.begin(),p.owners.end(),[](auto&w){return w.expired();}),p.owners.end());if(p.owners.size()>=p.max_owners){fail("owner_capacity");return {};}auto s=std::shared_ptr<Session>(new Session(std::make_unique<Session::Impl>(std::move(asset),owner)));p.owners.push_back(s);return s;}catch(const std::exception&e){fail(e.what());return {};}catch(...){fail("unknown_attach_failure");return {};}}
bool Pool::dispatch(const std::shared_ptr<Session>&session,std::uint64_t generation)noexcept{AllocationScope scope;try{
 if(!session)return false;auto&i=*session->impl_;auto&p=*impl_;std::lock_guard<std::mutex>poollock(p.mutex);std::lock_guard<std::mutex>lock(i.mutex);if(p.stop||i.canceled||i.failed||i.inflight||i.done||(i.pending.empty()&&i.retry_pending.empty())){
 auto trace_wait=[&](const auto&waiting){if(!waiting.empty()){const char*why=p.stop?"pool_stopped":i.canceled?"owner_canceled":i.failed?"owner_failed":i.done?"publication_pending":"worker_inflight";
  if(i.hit_wait.changed(why,waiting.front().sequence,waiting.back().sequence))body_hit_outcomes::report("dispatch_wait",why,i.owner.spawn_cookie,waiting.front().sequence,waiting.front().hit_trace,waiting.back().sequence,waiting.size(),unsigned(i.inflight));}};
 if(i.retry_pending.empty())trace_wait(i.pending);else trace_wait(i.retry_pending);
 return false;}
 // Memory admission (see MemoryPolicy): a deferred batch waits for the commit
 // it needs and no job starts below the dispatch floor, each for at most
 // max_wait_ms; then the job runs and reports its batch not applied if memory
 // is still short. Nothing is consumed while waiting.
 const auto policy=memory_policy();
 if(policy.enabled){const auto m=query_memory();
  if(m.valid&&m.avail_commit<std::max(i.need_commit,policy.dispatch_commit_floor)){
   if(!i.memory_waiting){i.memory_waiting=true;i.memory_wait_since=core::Clock::now();memoryCounters.dispatch_waits.fetch_add(1,std::memory_order_relaxed);}
   if(core::ms(i.memory_wait_since)<policy.max_wait_ms){auto trace_wait=[&](const auto&waiting){
    if(!waiting.empty()&&i.hit_wait.changed("memory_admission_wait",waiting.front().sequence,waiting.back().sequence))body_hit_outcomes::report("dispatch_wait","memory_admission_wait",i.owner.spawn_cookie,waiting.front().sequence,waiting.front().hit_trace,waiting.back().sequence,waiting.size());};
    if(i.retry_pending.empty())trace_wait(i.pending);else trace_wait(i.retry_pending);
    fail("memory_admission_wait");return false;}}}
 // Isolated work (the suffix of a failed batch) drains one event per job and
 // precedes fresh pending events; normal whole-batch dispatch resumes after it.
 Impl::Work task;task.session=session;task.isolated=!i.retry_pending.empty();if(task.isolated)task.events.assign(1,i.retry_pending.front());else{std::size_t take=i.pending.size();
  if(i.pending.front().blast){static const std::size_t chunk=[]{const char*e=std::getenv("GFY_BLAST_CHUNK");return e?std::size_t(std::max(0,std::atoi(e))):std::size_t(0);}();if(chunk)take=std::min(take,chunk);}
  task.events.assign(i.pending.begin(),i.pending.begin()+std::ptrdiff_t(take));}task.stamp={i.owner,generation,task.events.back().sequence};
 task.deferrals=i.memory_deferred==task.events.front().sequence?i.batch_deferrals:0u;
 // The job's failure report, preallocated here: see Pool::Impl::Work.
 task.failure=std::make_shared<Output>();{auto&f=*task.failure;f.stamp=task.stamp;f.batch_events=task.events.size();f.isolated_event=task.isolated;f.memory_deferrals=task.deferrals;
  f.impacts.resize(task.events.size());for(std::size_t n=0;n<task.events.size();++n){const auto&event=task.events[n];auto&result=f.impacts[n];
   result.hit_trace=event.hit_trace;result.sequence=event.sequence;result.gore=event.gore;result.explosion_world=event.explosion_world;result.explosion_cookie=event.explosion_cookie;
   result.explosion_target_birth=event.explosion_target_birth;result.explosion_member_token=event.explosion_member_token;}}
 // All allocations complete before consuming the owner queue; exceptions leave
 // pending events intact and never escape into exception-disabled xrGame.
 p.work.push_back(std::move(task));body_hit_outcomes::batch("dispatched",p.work.back().isolated?"isolated_retry":"worker_queue",i.owner.spawn_cookie,p.work.back().events);i.hit_wait.reset();if(p.work.back().isolated)i.retry_pending.erase(i.retry_pending.begin());else i.pending.erase(i.pending.begin(),i.pending.begin()+std::ptrdiff_t(p.work.back().events.size()));i.inflight=true;i.memory_waiting=false;i.need_commit=0;i.last_activity=core::Clock::now();p.wake.notify_one();return true;
}catch(const std::bad_alloc&){fail(outOfMemoryText);return false;}catch(const std::exception&e){fail(e.what());return false;}catch(...){fail("unknown_dispatch_failure");return false;}}
bool Pool::rebuild_completed(const std::shared_ptr<Session>&session,const std::shared_ptr<const Output>&expected,std::uint64_t generation)noexcept{AllocationScope scope;try{
 if(!session||!expected||session->impl_->piece)return false;auto&i=*session->impl_;auto&p=*impl_;
 std::lock_guard<std::mutex>poollock(p.mutex);std::lock_guard<std::mutex>lock(i.mutex);
 if(p.stop||!i.asset->allow_renderer_rebuild||i.canceled||i.failed||!i.inflight||i.done!=expected||
    !i.ready_snapshot||!i.ready_grid||!expected->error.empty()||!expected->resource_failure.empty()||expected->no_geometry_change||
    generation!=expected->stamp.expected_generation||!(expected->stamp.owner==i.owner)||
    i.renderer_rebuild==RendererRebuildState::queued||i.renderer_rebuild==RendererRebuildState::complete)return false;
 Impl::Work task;task.session=session;task.stamp=expected->stamp;task.renderer_source=expected;task.renderer_snapshot=i.ready_snapshot;
 // Allocate queue storage before changing the observable pending state. No
 // pending/retry event is copied, consumed, resolved or replayed by this job.
 p.work.push_back(std::move(task));i.renderer_rebuild=RendererRebuildState::queued;i.renderer_rebuild_reason="none";
 p.wake.notify_one();return true;
}catch(const std::bad_alloc&){fail(outOfMemoryText);return false;}catch(...){fail("renderer_rebuild_dispatch_failure");return false;}}
Pool::Eviction Pool::evict_idle_caches(std::size_t target,double min_idle_ms)noexcept{
 Eviction out;auto&p=*impl_;
 // Snapshot the owners under the pool lock (no allocation: fixed array), then
 // evict each under its own lock, least recently active first; canceled first.
 std::array<std::pair<double,std::shared_ptr<Session>>,pool_max_owners>order;std::size_t count=0;
 auto snapshot=p.ownerSnapshot();
 for(auto&s:snapshot){if(!s)continue;
  double idle;{std::lock_guard<std::mutex>own(s->impl_->mutex);idle=s->impl_->canceled?1e300:core::ms(s->impl_->last_activity);}
  if(idle<min_idle_ms)continue;order[count++]={idle,std::move(s)};}
 std::sort(order.begin(),order.begin()+std::ptrdiff_t(count),[](const auto&a,const auto&b){return a.first>b.first;});
 for(std::size_t n=0;n<count&&out.bytes<target;++n){const auto bytes=order[n].second->evict_caches();if(bytes){++out.sessions;out.bytes+=bytes;}}
 for(std::size_t n=0;n<count;++n)order[n].second.reset();
 memoryCounters.evicted_sessions.fetch_add(out.sessions,std::memory_order_relaxed);memoryCounters.evicted_bytes.fetch_add(out.bytes,std::memory_order_relaxed);
 return out;
}
bool Pool::submit_task(std::function<void()> task,bool urgent)noexcept{try{auto&p=*impl_;
  {std::lock_guard<std::mutex>lock(p.mutex);if(p.stop||!task)return false;
   while(p.task_threads.size()<p.task_workers)p.task_threads.emplace_back([&p]{p.run_tasks();}); // the lane starts with its first task
   if(urgent)p.tasks.push_front(std::move(task));else p.tasks.push_back(std::move(task));}
  p.task_wake.notify_one();return true;}catch(...){return false;}}
void Pool::configure_task_lane(unsigned workers)noexcept{try{auto&p=*impl_;std::lock_guard<std::mutex>lock(p.mutex);p.task_workers=workers<1?1:workers>4?4:workers;}catch(...){}}
bool Pool::any_memory_waiting()const noexcept{auto snapshot=impl_->ownerSnapshot();
 for(const auto&s:snapshot){if(!s)continue;std::lock_guard<std::mutex>own(s->impl_->mutex);if(!s->impl_->canceled&&s->impl_->memory_waiting)return true;}
 return false;}
std::size_t Pool::cache_bytes()const noexcept{
 auto snapshot=impl_->ownerSnapshot();std::size_t bytes=0;
 for(const auto&s:snapshot)if(s){std::lock_guard<std::mutex>own(s->impl_->mutex);if(!s->impl_->inflight&&!s->impl_->done)bytes+=Session::cache_bytes_locked(*s->impl_);}
 return bytes;
}

void run_directional_tests(){
 Grid g;g.size={16,16,16};g.origin={0,0,0};g.pitch=.002;g.base=std::make_shared<const std::vector<std::uint8_t>>(512,255);
 // Four incident cavity sheets from diagonally removed, otherwise solid
 // quarters must resolve by one intersected cell. Distant cutters and
 // non-checkerboard shapes must remain unchanged and retain mesh validation.
 for(double pitch:{.002,.001,.0005}){
  auto checker=g;checker.pitch=pitch;checker.erase({8,8,8});checker.erase({8,9,9});std::set<Cell>edgeTiles;
  need(resolveVoxelEdgeContacts(checker,{{8,9,9}},checker.center({8,8,9}),pitch*1.05,edgeTiles)==1&&checker.removed({8,8,9})&&!checker.removed({8,9,8}),"edge_contact_select_intersected_cell");
  auto distant=g;distant.pitch=pitch;distant.erase({8,8,8});distant.erase({8,9,9});edgeTiles.clear();
  need(resolveVoxelEdgeContacts(distant,{{8,9,9}},{-1,-1,-1},pitch*1.05,edgeTiles)==0&&!distant.removed({8,8,9})&&!distant.removed({8,9,8}),"edge_contact_refuse_distant_cutter");
  auto three=checker;edgeTiles.clear();need(resolveVoxelEdgeContacts(three,{{8,8,9}},three.center({8,9,8}),pitch*1.05,edgeTiles)==0,"edge_contact_not_checkerboard");
 }
 for(Vec d:std::vector<Vec>{{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1},{1,2,3},{-3,1,-2}}){auto u=unit(d),center=g.center({8,8,8}),origin=center-u*.1;auto r=firstSolid(g,origin,u);need(r.found&&g.valid(r.cell),"directional_solid_ray");Grid copy=g;auto a=eraseRay(copy,origin,u,.006,.002);auto b=eraseRay(copy,origin,u,.006,.002);need(a.freshSolid&&b.freshSolid&&dot(b.cutter-a.cutter,u)>.002,"same_radius_cumulative_depth");}
 need(!firstSolid(g,{-1,-1,-1},{1,0,0}).found,"outside_parallel_ray");
 for(int x=0;x<16;++x)g.erase({x,8,8});need(!firstSolid(g,{-.01,.017,.017},{1,0,0}).found,"fully_removed_ray_is_noop");
 // A side ray must see material in a different column, independently of -Z.
 need(firstSolid(g,{-.01,.019,.017},{1,0,0}).found,"side_survivor");
 Source s;s.bones=1;s.v.resize(3);s.f.push_back({0,1,2});s.v[0].p={0,0,0};s.v[1].p={1,0,0};s.v[2].p={0,1,0};for(auto&v:s.v){v.w[0]=1;v.normal={0,0,1};}
 Matrix m=Matrix::identity();m.row_major={0,0,1,.3,0,1,0,.4,-1,0,0,.5,0,0,0,1};Impact event;event.bone_model_render={m};event.accepted_pose_point=point(transform(m,{.2,.3,0}));event.accepted_pose_direction=point(transform(m,{0,0,-1},true));auto map=sourceRay(s,event);need(length(map.entry-Vec{.2,.3,0})<1e-12&&length(map.direction-Vec{0,0,-1})<1e-12,"posed_rigid_source_ray");
}
}

namespace gfy_live {
void run_tunnel_tests(){
 {
  Grid large;large.size={600,600,600};large.origin={0,0,0};large.pitch=.0005;
  large.base=std::make_shared<const std::vector<std::uint8_t>>(600*600*600/8,255);
  bool rejected=false;try {checkedCutterCells({0,0,0},{599,599,599});}
  catch(const RecoverableImpactError&e){rejected=std::string(e.what())=="impact_cell_work_budget";}
  need(rejected&&large.tiles.empty(),"cutter_budget_mutated_grid");
  need(eraseTunnel(large,{.064,.064,.2},{0,0,-1},.003,.01).freshSolid>0,"valid_after_work_budget");
 }
 for(double pitch:{.002,.001,.0005}){
  Grid g;g.size={32,32,32};g.origin={0,0,0};g.pitch=pitch;g.base=std::make_shared<const std::vector<std::uint8_t>>(4096,255);
  for(Vec direction:std::vector<Vec>{{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1},{1,2,3},{-3,1,-2}}){
   direction=unit(direction);Vec origin=g.center({16,16,16})-direction*.2;
   auto march=firstSolid(g,origin,direction);need(march.found,"tunnel_entry");
   for(double reach:{pitch*.6,pitch*2.5,pitch*9.5}){
    Grid copy=g;double radius=pitch*3.2;Tunnel volume{march.entry,direction,radius,reach,std::min(radius,reach)};
    need(volume.contains(march.entry),"connected_entry");
    need(!volume.contains(march.entry+direction*(reach+pitch*.01)),"no_radius_penetration_addition");
    need(!volume.contains(march.entry-direction*(pitch*.01)),"no_backward_penetration");
    volume.inlet=std::max(0.,dot(march.entry-origin,direction));
    auto cut=eraseTunnel(copy,origin,direction,radius,reach);
    for(int x=0;x<32;++x)for(int y=0;y<32;++y)for(int z=0;z<32;++z){Cell c{x,y,z};if(!copy.removed(c))continue;
     // Each ordinary center or repair witness must intersect exactly this cut.
     need(volume.contains(copy.center(c))||volume.intersectsCell(copy,c),"tunnel_removal_outside_volume");
    }
    for(int x=0;x<32;++x)for(int y=0;y<32;++y)for(int z=0;z<32;++z){Cell c{x,y,z};
     if(volume.contains(copy.center(c)))need(copy.removed(c),"tunnel_column_missed_cell");}
    if(reach>pitch*2){need(cut.freshSolid>0,"tunnel_expected_removal");auto first=firstSolid(copy,origin,direction);if(first.found)need(dot(first.entry-march.entry,direction)>=reach-pitch*1.75,"connected_axis_material_left_inside");}
   }
  }
 }
}
}

#include "LiveSeparationWorker_impl.h"
#include "OcularAccessoryDamage_impl.h"
#include "GearBody.h" // gearbody: the outfit's gear joins its body source (after the gear decoders)
#include "ComponentSession_impl.h"
namespace gfy_live {
// Engine governor hook (LiveDamageWorker.h): drops the separation region-join cache (separation/RegionJoin.h).
std::size_t clear_separation_region_cache()noexcept{try{auto&c=bound_surface::region_join::regionJoinCache();const auto bytes=c.bytes();c.clear();return bytes;}catch(...){return 0;}}
}
