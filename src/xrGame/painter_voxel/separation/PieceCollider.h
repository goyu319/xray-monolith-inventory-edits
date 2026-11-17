#pragma once
// CPU-only compound box proposal from exact remaining anatomical spans.
// Include native LiveDamageWorker/RemainingConnectivity definitions first.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <climits>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <thread>
#include <intrin.h>
#include <exception>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "../CollisionLimits.h"
#include "ThreadBudget.h"
#if defined(__AVX__)
#include <immintrin.h>
#endif
namespace gfy_piece_collider {
using gfy_live::Matrix;using gfy_live::Stamp;
using gfy_connectivity::Topology;
using V=std::array<double,3>;
inline void require(bool ok,const char*why){if(!ok)throw std::runtime_error(why);}
inline V add(V a,V b){return {a[0]+b[0],a[1]+b[1],a[2]+b[2]};}
inline V sub(V a,V b){return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};}
inline V mul(V a,double k){return {a[0]*k,a[1]*k,a[2]*k};}
inline double dot(V a,V b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
inline V cross(V a,V b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
inline double norm(V a){return std::sqrt(dot(a,a));}
inline V transform(const Matrix&m,V p){auto&a=m.row_major;return {
 a[0]*p[0]+a[1]*p[1]+a[2]*p[2]+a[3],
 a[4]*p[0]+a[5]*p[1]+a[6]*p[2]+a[7],
 a[8]*p[0]+a[9]*p[1]+a[10]*p[2]+a[11]};}
inline V rotate(const Matrix&m,V p){auto&a=m.row_major;return {
 a[0]*p[0]+a[1]*p[1]+a[2]*p[2],
 a[4]*p[0]+a[5]*p[1]+a[6]*p[2],
 a[8]*p[0]+a[9]*p[1]+a[10]*p[2]};}
inline void validateRigid(const Matrix&m){auto&a=m.row_major;
 for(double q:a)require(std::isfinite(q),"collider_pose_nonfinite");
 require(std::abs(a[12])<1e-9&&std::abs(a[13])<1e-9&&std::abs(a[14])<1e-9&&std::abs(a[15]-1)<1e-9,"collider_pose_affine");
 V c[3]={{a[0],a[4],a[8]},{a[1],a[5],a[9]},{a[2],a[6],a[10]}};
 for(int i=0;i<3;++i){require(std::abs(dot(c[i],c[i])-1)<1e-5,"collider_pose_scale");
  for(int j=0;j<i;++j)require(std::abs(dot(c[i],c[j]))<1e-5,"collider_pose_shear");}
 require(dot(c[0],cross(c[1],c[2]))>.99999,"collider_pose_reflection");
}
struct Capture {
 Stamp accepted;
 std::uint32_t component{};
 std::uint16_t anchor_bone{};
 // Original model rest -> captured anchor-bone model frame. Must be rigid.
 Matrix rest_to_anchor;
 Matrix anchor_to_world;
 bool rigid_pose_certified{}; // Caller reviewed all included skin/physics elements.
 double maximum_skin_deviation_m{};
 V parent_com_world{},parent_linear_world{},parent_angular_world{};
 double density_kg_m3{985};
};
struct Box {
 V center_anchor{};
 // Row-major, columns are the local box axes in anchor coordinates.
 std::array<double,9> orientation{};
 V half_extents_m{};
};
struct Prepared {
 Stamp accepted;
 std::uint32_t component{};
 std::uint16_t anchor_bone{};
 std::vector<Box> boxes;
 std::uint64_t owned_cells{};
 double anatomical_volume_m3{},mass_kg{};
 V com_anchor{};
 std::array<double,9> inertia_anchor_com{}; // kg m^2, symmetric positive definite.
 V linear_world{},angular_world{};
 double maximum_radius_m{},maximum_box_extent_m{};
 double proxy_to_anatomy_volume_ratio{},maximum_corner_gap_m{};
 std::uint64_t omitted_collision_cells{};
 double maximum_omitted_distance_m{};
};
struct Limits {
 std::size_t max_boxes{painter_collision_limits::boxes_per_element},max_spans{1500000};
 // Collision envelopes approximate major anatomy at ~8-15 mm. The original
 // 1 mm cell volume still supplies exact mass and inertia.
 double max_proxy_ratio{2.5},max_box_ratio{6.0},max_corner_gap_m{.025};
 double minimum_half_extent_m{.005}; // CPHGeometryOwner::add_Box clamp.
 double max_rigid_skin_deviation_m{.003};
 double max_omitted_cell_fraction{.02},max_omitted_distance_m{.025}; // .005 refused rough-cut limbs with a few 1-cell crumbs
};
namespace detail {
// Set while a caller already runs fits concurrently: prepare() then keeps its
// candidate evaluation serial, so total concurrency stays within that caller's
// thread budget (<= 4). Results are identical either way.
inline bool& serialOnly(){thread_local bool value=false;return value;}
struct SerialScope {bool previous;SerialScope():previous(serialOnly()){serialOnly()=true;}~SerialScope(){serialOnly()=previous;}
 SerialScope(const SerialScope&)=delete;SerialScope&operator=(const SerialScope&)=delete;};
// Separation-track P3 step B. prepare() fans out only when no enclosing parallel
// section exists: neither a SerialScope (outer prefit loop) nor a sep::parallel /
// sep::NestedScope token (ThreadBudget.h). Every parallel section below goes through
// sep::parallel with a cap of four threads (caller included) and installs a
// SerialScope in its bodies, so a job's collider work never exceeds four threads.
// All parallel work is pure per item and reduced in the serial order; outputs are
// identical to the serial computation.
constexpr unsigned max_threads=4;
inline bool mayFanOut(){return !serialOnly()&&!gfy_live::sep::nested();}
// speedfix: what the last prepare() on this thread did (GFY_SF_FITLOG read-out only).
struct FitTrace{double greedyMs{-1};std::size_t greedyLeaves{};int restarts{},speculated{};std::chrono::steady_clock::time_point start{};};
inline FitTrace& fitTrace(){thread_local FitTrace t;return t;}
#ifdef GFY_P3_CANCEL_TEST
// Test hook only: trips the installed cancel flag at the n-th cooperative check and
// records when (steady_clock ticks), for the cancel-to-exit latency.
struct CancelTest{std::atomic<unsigned long long>checks{0},at{0};std::atomic<std::atomic<bool>*>flag{nullptr};
 std::atomic<long long>tripped{0};
 void arm(std::atomic<bool>*f,unsigned long long n){at=n;checks=0;tripped=0;flag=f;}};
inline CancelTest& cancelTest(){static CancelTest t;return t;}
#endif
#if defined(GFY_P3_THREAD_AUDIT)||defined(GFY_P3_VERIFY)
// Audit of the collider's own parallel sections: every sep::parallel body below
// registers its thread once (thread_local), so `peak` is the largest number of
// threads that ever ran collider work at the same time (the caller runs chunk 0
// and is counted). Verify builds require it to stay within the four-thread budget.
struct ThreadAudit{std::atomic<int>active{0},peak{0};};
inline ThreadAudit& threadAudit(){static ThreadAudit a;return a;}
struct AuditScope{bool counted;AuditScope(){thread_local int depth=0;counted=depth++==0;dep=&depth;
  if(counted){auto&a=threadAudit();const int now=++a.active;int p=a.peak.load();while(now>p&&!a.peak.compare_exchange_weak(p,now)){}
#ifdef GFY_P3_VERIFY
   require(now<=int(max_threads),"p3_verify_thread_budget");
#endif
  }}
 ~AuditScope(){--*dep;if(counted)--threadAudit().active;}
 int*dep;AuditScope(const AuditScope&)=delete;AuditScope&operator=(const AuditScope&)=delete;};
#define P3_AUDIT const ::gfy_piece_collider::detail::AuditScope p3audit
#else
#define P3_AUDIT
#endif
// Separation-track P3 step C: cooperative cancellation. Aborts only: throws
// "separation_canceled" when the job's cancel flag (sep::CancelScope) is set; it
// never produces a partial result. No flag installed: a no-op.
// speedfix: set while this thread runs a speculative compact fit (Speculation): its stop flag.
inline std::atomic<bool>*& speculationStop(){thread_local std::atomic<bool>* s=nullptr;return s;}
// speedfix: set on the background prefit thread (LiveSeparationWorker PrefitService) to sep::activeJobs(): its fits wait at
// their cancel points while real separation work runs (its own cancel flag still stops them).
inline const std::atomic<int>*& pauseWhile(){thread_local const std::atomic<int>* p=nullptr;return p;}
inline void cancelPoint(){
#ifdef GFY_P3_CANCEL_TEST
 {auto&t=cancelTest();const auto n=++t.checks;if(auto*f=t.flag.load())if(t.at&&n>=t.at&&!f->load()){
   long long zero=0;t.tripped.compare_exchange_strong(zero,std::chrono::steady_clock::now().time_since_epoch().count());f->store(true);}}
#endif
 gfy_live::sep::throwIfCanceled();
 if(const auto*s=speculationStop();s&&s->load(std::memory_order_relaxed))throw std::runtime_error("collider_speculation_stopped");
 if(const auto*p=pauseWhile())while(p->load(std::memory_order_relaxed)>0){gfy_live::sep::throwIfCanceled();std::this_thread::sleep_for(std::chrono::milliseconds(1));}}
#ifdef GFY_RX_PROF
struct Prof{double evalMs=0,supportMs=0,splitMs=0,combineMs=0,axesMs=0;std::size_t evals=0,evalRuns=0,supports=0,splits=0,combines=0,blocks=0,cols=0;
 double specMs=0,summaryMs=0,scanMs=0,subdivMs=0,mergeMs=0,flushMs=0,commitMs=0,finalMs=0,omitMs=0;
 std::size_t iterations=0,omissions=0,summaryRuns=0,skippedBoxes=0,stoppedBoxes=0,flushes=0,flushItems=0,parallelFlushes=0,commits=0,commitRuns=0,momentFolds=0,momentFoldRuns=0,childPairs=0,chunkedPasses=0,omitFull=0,omitPoints=0;};
inline Prof& prof(){static Prof p;return p;}
struct ProfTimer{double&acc;std::chrono::steady_clock::time_point t=std::chrono::steady_clock::now();~ProfTimer(){acc+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t).count();}};
#define RXP(field) ::gfy_piece_collider::detail::ProfTimer rxp_##field{::gfy_piece_collider::detail::prof().field}
#define RXC(field,amount) (::gfy_piece_collider::detail::prof().field+=(amount))
#else
#define RXP(field)
#define RXC(field,amount) ((void)0)
#endif
struct Run {int x,y,lo,hi;};
struct FittedBox {V center_cells{};std::array<V,3> axes{{V{1,0,0},V{0,1,0},V{0,0,1}}};
 V half_cells{};double proxy_cells{},gap_cells{};bool minimum_size{};};
struct Moments {long double n{},s[3]{},q[3]{},xy{},xz{},yz{};};
// One run's contribution to moments(): the former loop body, verbatim. moments()
// and the fused per-append accumulation both call it, so they execute the identical
// operation sequence.
inline void addMoment(Moments&m,const Run&r,long double p,long double ox,long double oy,long double oz){
 const long double n=r.hi-r.lo,x=ox+(r.x+.5L)*p,y=oy+(r.y+.5L)*p;
 const long double low=r.lo,high=r.hi-1;
 const long double sumK=n*(low+high)/2;
 auto F=[](long double v){return v*(v+1)*(2*v+1)/6;};
 const long double sumK2=F(high)-F(low-1),z0=oz+p*.5L;
 const long double sz=n*z0+p*sumK,qz=n*z0*z0+2*z0*p*sumK+p*p*sumK2;
 m.n+=n;m.s[0]+=n*x;m.s[1]+=n*y;m.s[2]+=sz;
 m.q[0]+=n*x*x;m.q[1]+=n*y*y;m.q[2]+=qz;
 m.xy+=n*x*y;m.xz+=x*sz;m.yz+=y*sz;}
// principalAxes() evaluates moments() on a unit-pitch topology at the origin:
// p=1 and origin 0, exactly these long double values.
inline void addUnitMoment(Moments&m,const Run&r){addMoment(m,r,1.0L,0.0L,0.0L,0.0L);}
// One run's contribution to evaluate()'s bounds: the former loop body, verbatim.
inline void boundRun(const Run&s,const std::array<V,3>&axes,V&low,V&high){
 const V a{{double(s.x),double(s.y),double(s.lo)}},
         b{{double(s.x+1),double(s.y+1),double(s.hi)}};
 for(int i=0;i<3;++i){double mn=0,mx=0;for(int j=0;j<3;++j){const double u=axes[i][j];
   mn+=u*(u>=0?a[j]:b[j]);mx+=u*(u>=0?b[j]:a[j]);}
  low[i]=std::min(low[i],mn);high[i]=std::max(high[i],mx);}}
struct Region {
 std::vector<Run> runs;
 std::array<int,3> lo{{INT32_MAX,INT32_MAX,INT32_MAX}},hi{};
 std::uint64_t cells{},box_cells{};
 FittedBox fitted;
 bool evaluated{};
 // P3 step A summary, set by summarize() once the runs are final (or by
 // commitUnion): bone-axis bounds exactly as evaluate(runs,boneAxes) computes them,
 // and unit == moments(runs, unit-pitch topology), bit for bit.
 V boneLow{{1e30,1e30,1e30}},boneHigh{{-1e30,-1e30,-1e30}};
 Moments unit;
};
inline void append(Region&r,Run s){if(s.lo>=s.hi)return;
 r.runs.push_back(s);r.cells+=std::uint64_t(s.hi-s.lo);
 r.lo[0]=std::min(r.lo[0],s.x);r.lo[1]=std::min(r.lo[1],s.y);r.lo[2]=std::min(r.lo[2],s.lo);
 r.hi[0]=std::max(r.hi[0],s.x+1);r.hi[1]=std::max(r.hi[1],s.y+1);r.hi[2]=std::max(r.hi[2],s.hi);
}
inline void finish(Region&r){require(r.cells>0,"collider_empty_region");
 r.box_cells=1;for(int a=0;a<3;++a)r.box_cells*=std::uint64_t(r.hi[a]-r.lo[a]);}
// Exact moments. Every addMoment() term of a run is 1/2 or 1/4 of an integer
// polynomial in x,y,lo,hi; scaled they are:
//   n=hi-lo, 2*s0=n(2x+1), 2*s1=n(2y+1), 2*s2=2*sz=n(lo+hi), 4*q0=n(2x+1)^2,
//   4*q1=n(2y+1)^2, 4*q2=4*qz=sum_{k=lo}^{hi-1}(2k+1)^2=S(hi)-S(lo) with
//   S(m)=(4m^3-m)/3, 4*xy=n(2x+1)(2y+1), 4*xz=(2x+1)n(lo+hi), 4*yz=(2y+1)n(lo+hi).
// Certificate (exactMoments): non-negative coordinates below H<2^16 and
// cells*H^2 < 2^50. Then every addMoment() term is computed exactly (all
// intermediate products stay below 2^53) and every partial sum of every component,
// in any order, is a multiple of 1/4 below 2^51: the moments() fold equals the exact
// real sum. The scaled terms above are integers of at most 4*n*H^2 < 2^52 (S(m) <
// 2^50), computed exactly in double, and so is every partial sum of them in any
// order: ExactSums accumulated in any order or lane split and scaled back by 1/2 or
// 1/4 reproduce the fold bit for bit.
inline bool exactMoments(std::uint64_t cells,const std::array<int,3>&lo,const std::array<int,3>&hi){
 if(lo[0]<0||lo[1]<0||lo[2]<0)return false;
 const double H=double(std::max({hi[0],hi[1],hi[2]}));
 return H<65536&&double(cells)*H*H<0x1p50;}
struct ExactSums {double n{},s0{},s1{},s2{},q0{},q1{},q2{},xy{},xz{},yz{};};
inline void addExact(ExactSums&m,const Run&r){
 const double X=r.x,Y=r.y,L=r.lo,H=r.hi;
 const double n=H-L,a=2*X+1,b=2*Y+1,nc=n*(L+H),na=n*a,nb=n*b;
 m.n+=n;m.s0+=na;m.s1+=nb;m.s2+=nc;m.q0+=na*a;m.q1+=nb*b;
 m.q2+=(4*H*H*H-H)/3-(4*L*L*L-L)/3;m.xy+=na*b;m.xz+=a*nc;m.yz+=b*nc;}
inline void addSums(ExactSums&a,const ExactSums&b){a.n+=b.n;a.s0+=b.s0;a.s1+=b.s1;a.s2+=b.s2;a.q0+=b.q0;a.q1+=b.q1;a.q2+=b.q2;a.xy+=b.xy;a.xz+=b.xz;a.yz+=b.yz;}
inline Moments toMoments(const ExactSums&m){Moments o;
 o.n=(long double)m.n;o.s[0]=(long double)m.s0/2;o.s[1]=(long double)m.s1/2;o.s[2]=(long double)m.s2/2;
 o.q[0]=(long double)m.q0/4;o.q[1]=(long double)m.q1/4;o.q[2]=(long double)m.q2/4;
 o.xy=(long double)m.xy/4;o.xz=(long double)m.xz/4;o.yz=(long double)m.yz/4;return o;}
// Bounds of runs [first,first+count) under three axes, exactly as evaluate()'s loop
// computes them: per run and axis mn=((0+u0*c0)+u1*c1)+u2*c2 and likewise mx
// (boundRun), then min/max. The AVX kernel evaluates four runs per instruction with
// the same IEEE double multiplies and adds in the same order per lane (no FMA under
// /arch:AVX) and selects c by the sign of u exactly as boundRun does. min/max are
// exact, all values are finite, and no mn/mx is ever -0 (each starts from +0), so
// the result is independent of lane and visiting order. WithSums also accumulates
// the ExactSums of the same runs (valid only under exactMoments()).
template<bool WithSums>
inline void scanRuns(const Run*first,std::size_t count,const std::array<V,3>&axes,V&low,V&high,ExactSums&sums){
 std::size_t k=0;
#if defined(__AVX__)
 if(count>=8){
  __m256d U[3][3],M[3][3],L[3],H[3];
  for(int i=0;i<3;++i){L[i]=_mm256_set1_pd(1e30);H[i]=_mm256_set1_pd(-1e30);
   for(int j=0;j<3;++j){U[i][j]=_mm256_set1_pd(axes[i][j]);
    M[i][j]=_mm256_castsi256_pd(_mm256_set1_epi64x(axes[i][j]>=0?-1:0));}}
  const __m256d zero=_mm256_setzero_pd();const __m128i one=_mm_set1_epi32(1);
  const __m256d onev=_mm256_set1_pd(1),two=_mm256_set1_pd(2),three=_mm256_set1_pd(3),four=_mm256_set1_pd(4);
  __m256d acc[10];for(auto&v:acc)v=zero;
  for(;k+4<=count;k+=4){
   __m128 r0=_mm_castsi128_ps(_mm_loadu_si128(reinterpret_cast<const __m128i*>(first+k)));
   __m128 r1=_mm_castsi128_ps(_mm_loadu_si128(reinterpret_cast<const __m128i*>(first+k+1)));
   __m128 r2=_mm_castsi128_ps(_mm_loadu_si128(reinterpret_cast<const __m128i*>(first+k+2)));
   __m128 r3=_mm_castsi128_ps(_mm_loadu_si128(reinterpret_cast<const __m128i*>(first+k+3)));
   _MM_TRANSPOSE4_PS(r0,r1,r2,r3); // bit moves only: r0=x, r1=y, r2=lo, r3=hi of the four runs
   const __m128i X=_mm_castps_si128(r0),Y=_mm_castps_si128(r1);
   const __m256d a[3]={_mm256_cvtepi32_pd(X),_mm256_cvtepi32_pd(Y),_mm256_cvtepi32_pd(_mm_castps_si128(r2))};
   const __m256d b[3]={_mm256_cvtepi32_pd(_mm_add_epi32(X,one)),_mm256_cvtepi32_pd(_mm_add_epi32(Y,one)),
                       _mm256_cvtepi32_pd(_mm_castps_si128(r3))};
   for(int i=0;i<3;++i){
    __m256d mn=_mm256_add_pd(zero,_mm256_mul_pd(U[i][0],_mm256_blendv_pd(b[0],a[0],M[i][0])));
    __m256d mx=_mm256_add_pd(zero,_mm256_mul_pd(U[i][0],_mm256_blendv_pd(a[0],b[0],M[i][0])));
    mn=_mm256_add_pd(mn,_mm256_mul_pd(U[i][1],_mm256_blendv_pd(b[1],a[1],M[i][1])));
    mx=_mm256_add_pd(mx,_mm256_mul_pd(U[i][1],_mm256_blendv_pd(a[1],b[1],M[i][1])));
    mn=_mm256_add_pd(mn,_mm256_mul_pd(U[i][2],_mm256_blendv_pd(b[2],a[2],M[i][2])));
    mx=_mm256_add_pd(mx,_mm256_mul_pd(U[i][2],_mm256_blendv_pd(a[2],b[2],M[i][2])));
    L[i]=_mm256_min_pd(L[i],mn);H[i]=_mm256_max_pd(H[i],mx);}
   if constexpr(WithSums){ // addExact() on four runs; every operation is exact
    const __m256d lo=a[2],hi=b[2],n=_mm256_sub_pd(hi,lo);
    const __m256d ax=_mm256_add_pd(_mm256_mul_pd(two,a[0]),onev),by=_mm256_add_pd(_mm256_mul_pd(two,a[1]),onev);
    const __m256d nc=_mm256_mul_pd(n,_mm256_add_pd(lo,hi)),na=_mm256_mul_pd(n,ax),nb=_mm256_mul_pd(n,by);
    const __m256d sh=_mm256_div_pd(_mm256_sub_pd(_mm256_mul_pd(_mm256_mul_pd(_mm256_mul_pd(four,hi),hi),hi),hi),three);
    const __m256d sl=_mm256_div_pd(_mm256_sub_pd(_mm256_mul_pd(_mm256_mul_pd(_mm256_mul_pd(four,lo),lo),lo),lo),three);
    acc[0]=_mm256_add_pd(acc[0],n);acc[1]=_mm256_add_pd(acc[1],na);acc[2]=_mm256_add_pd(acc[2],nb);
    acc[3]=_mm256_add_pd(acc[3],nc);acc[4]=_mm256_add_pd(acc[4],_mm256_mul_pd(na,ax));acc[5]=_mm256_add_pd(acc[5],_mm256_mul_pd(nb,by));
    acc[6]=_mm256_add_pd(acc[6],_mm256_sub_pd(sh,sl));acc[7]=_mm256_add_pd(acc[7],_mm256_mul_pd(na,by));
    acc[8]=_mm256_add_pd(acc[8],_mm256_mul_pd(ax,nc));acc[9]=_mm256_add_pd(acc[9],_mm256_mul_pd(by,nc));}
  }
  alignas(32) double lanes[4];
  for(int i=0;i<3;++i){_mm256_store_pd(lanes,L[i]);for(double v:lanes)low[i]=std::min(low[i],v);
   _mm256_store_pd(lanes,H[i]);for(double v:lanes)high[i]=std::max(high[i],v);}
  if constexpr(WithSums){double*out[10]={&sums.n,&sums.s0,&sums.s1,&sums.s2,&sums.q0,&sums.q1,&sums.q2,&sums.xy,&sums.xz,&sums.yz};
   for(int c=0;c<10;++c){_mm256_store_pd(lanes,acc[c]);for(double v:lanes)*out[c]+=v;}}
 }
#endif
 for(;k<count;++k){boundRun(first[k],axes,low,high);if constexpr(WithSums)addExact(sums,first[k]);}}
inline void boundRunsSerial(const Run*first,std::size_t count,const std::array<V,3>&axes,V&low,V&high){
 ExactSums unused;scanRuns<false>(first,count,axes,low,high,unused);}
// scanRuns on up to four contiguous chunks when the range is large and no enclosing
// parallel section exists; chunk bounds merge with min/max and chunk sums add, both
// exactly.
template<bool WithSums>
inline void scanRunsMaybeParallel(const Run*first,std::size_t count,const std::array<V,3>&axes,V&low,V&high,ExactSums&sums){
 constexpr std::size_t grain=16384;
 if(count>=2*grain&&mayFanOut()){RXC(chunkedPasses,1);
  std::array<V,max_threads>lows,highs;std::array<ExactSums,max_threads>parts{};
  for(auto&v:lows)v=V{{1e30,1e30,1e30}};for(auto&v:highs)v=V{{-1e30,-1e30,-1e30}};
  gfy_live::sep::parallel(count,grain,max_threads,[&](std::size_t b,std::size_t e,std::size_t chunk){P3_AUDIT;const SerialScope serial;
   scanRuns<WithSums>(first+b,e-b,axes,lows[chunk],highs[chunk],parts[chunk]);});
  for(std::size_t c=0;c<max_threads;++c){for(int i=0;i<3;++i){low[i]=std::min(low[i],lows[c][i]);high[i]=std::max(high[i],highs[c][i]);}
   if constexpr(WithSums)addSums(sums,parts[c]);}
  return;}
 scanRuns<WithSums>(first,count,axes,low,high,sums);}
inline void boundRuns(const Run*first,std::size_t count,const std::array<V,3>&axes,V&low,V&high){
#ifdef GFY_RX_PROF
 RXC(evalRuns,count);
#endif
 ExactSums unused;scanRunsMaybeParallel<false>(first,count,axes,low,high,unused);}
// The P3 summary of a finished region (after finish()): bone bounds and unit
// moments in one pass under the exactMoments() certificate, else the bone pass and
// the moments() fold in run order.
inline void summarize(Region&r,const std::array<V,3>&bone){RXP(summaryMs);
#ifdef GFY_RX_PROF
 RXC(summaryRuns,r.runs.size());
#endif
 r.boneLow=V{{1e30,1e30,1e30}};r.boneHigh=V{{-1e30,-1e30,-1e30}};
 if(!exactMoments(r.cells,r.lo,r.hi)){RXC(momentFolds,1);RXC(momentFoldRuns,r.runs.size());
  ExactSums unused;scanRunsMaybeParallel<false>(r.runs.data(),r.runs.size(),bone,r.boneLow,r.boneHigh,unused);
  Moments m;for(const auto&s:r.runs)addUnitMoment(m,s);r.unit=m;return;}
 ExactSums sums;scanRunsMaybeParallel<true>(r.runs.data(),r.runs.size(),bone,r.boneLow,r.boneHigh,sums);
 r.unit=toMoments(sums);}
inline void reset(Region&r){r.runs.clear();r.lo={{INT32_MAX,INT32_MAX,INT32_MAX}};r.hi={};r.cells=0;r.box_cells=0;
 r.fitted=FittedBox{};r.evaluated=false;r.boneLow=V{{1e30,1e30,1e30}};r.boneHigh=V{{-1e30,-1e30,-1e30}};r.unit=Moments{};}
// A copy of r without its runs: bounds, cells, summary and fit.
inline Region shell(const Region&r){Region s;s.lo=r.lo;s.hi=r.hi;s.cells=r.cells;s.box_cells=r.box_cells;
 s.fitted=r.fitted;s.evaluated=r.evaluated;s.boneLow=r.boneLow;s.boneHigh=r.boneHigh;s.unit=r.unit;return s;}
// The former split() loop into a and b (reset first).
inline void splitInto(const Region&r,int axis,int plane,const std::array<V,3>&bone,Region&a,Region&b,bool summarizeParts=true){RXP(splitMs);
 reset(a);reset(b);
 for(const auto&s:r.runs){if(axis==0){append(s.x<plane?a:b,s);continue;}
  if(axis==1){append(s.y<plane?a:b,s);continue;}
  append(a,{s.x,s.y,s.lo,std::min(s.hi,plane)});
  append(b,{s.x,s.y,std::max(s.lo,plane),s.hi});}
 if(a.cells){finish(a);if(summarizeParts)summarize(a,bone);}if(b.cells){finish(b);if(summarizeParts)summarize(b,bone);}}
inline std::pair<Region,Region> split(const Region&r,int axis,int plane,const std::array<V,3>&bone){
 std::pair<Region,Region>out;splitInto(r,axis,plane,bone,out.first,out.second);return out;}
// Number of runs split(r,axis,plane) appends to one side (append skips empty parts).
inline std::size_t sideRuns(const Region&r,int axis,int plane,int side){std::size_t n=0;
 for(const auto&s:r.runs){if(axis==0)n+=(s.x<plane)==(side==0);else if(axis==1)n+=(s.y<plane)==(side==0);
  else n+=side==0?s.lo<std::min(s.hi,plane):std::max(s.lo,plane)<s.hi;}
 return n;}
// One side of split(r,axis,plane): exactly the runs split() appends to that side,
// in the same order, so the region (runs, bounds, summary) is identical.
inline void splitSide(const Region&r,int axis,int plane,int side,Region&out,const std::array<V,3>&bone){RXP(splitMs);
 out.runs.reserve(sideRuns(r,axis,plane,side));
 for(const auto&s:r.runs){if(axis==0){if((s.x<plane)==(side==0))append(out,s);continue;}
  if(axis==1){if((s.y<plane)==(side==0))append(out,s);continue;}
  if(side==0)append(out,{s.x,s.y,s.lo,std::min(s.hi,plane)});
  else append(out,{s.x,s.y,std::max(s.lo,plane),s.hi});}
 if(out.cells){finish(out);summarize(out,bone);}}
// The former splitOriented() loop into a and b (reset first).
inline void splitOrientedInto(const Region&r,V normal,double plane,const std::array<V,3>&bone,Region&a,Region&b,bool summarizeParts=true){RXP(splitMs);
 reset(a);reset(b);
 for(const auto&s:r.runs){const double xy=(s.x+.5)*normal[0]+(s.y+.5)*normal[1];
  if(std::abs(normal[2])<1e-12){append(xy<plane?a:b,s);continue;}
  const double z=(plane-xy)/normal[2]-.5;
  const auto split=normal[2]>0?std::ceil(z):std::floor(z)+1.;
  const int at=split<s.lo?s.lo:(split>s.hi?s.hi:int(split));
  if(normal[2]>0){append(a,{s.x,s.y,s.lo,at});append(b,{s.x,s.y,at,s.hi});}
  else{append(b,{s.x,s.y,s.lo,at});append(a,{s.x,s.y,at,s.hi});}
 }
 if(a.cells){finish(a);if(summarizeParts)summarize(a,bone);}if(b.cells){finish(b);if(summarizeParts)summarize(b,bone);}}
inline std::pair<Region,Region> splitOriented(const Region&r,V normal,double plane,const std::array<V,3>&bone){
 std::pair<Region,Region>out;splitOrientedInto(r,normal,plane,bone,out.first,out.second);return out;}
// Exact accelerated nearest-span query. Spans of one component are bucketed
// by 8x8 column blocks with the block's z range. Blocks are visited in
// Chebyshev rings around the query; a block is skipped only when its bound
// distance (computed with the same monotone operations, so it never exceeds
// any member span's distance in floating point) is > the best found, and the
// search stops when a whole ring's xy bound is > best. Every span that could
// attain the minimum is evaluated with the unchanged per-span expression, so
// the returned minimum is bit-identical to the reference search below.
struct SupportIndex {
 static constexpr int B=4; // 4x4-column blocks
 // == std::max({a,b,0.}) (max is exact); avoids initializer-list temporaries.
 static double gap(double a,double b){return std::max(std::max(a,b),0.);}
 struct Col{int x,y,zlo,zhi;std::uint32_t first,count;};struct Sp{int lo,hi;};
 const Topology* top{};std::uint32_t component{};int bx{},by{};
 std::vector<std::uint32_t>start;std::vector<Col>cols;std::vector<Sp>spans;std::vector<int>zlo,zhi;
 // P3 step D (GFY_P3_CERTIFIED_GAP only; -1 = exact): a query may stop at the first
 // span whose squared distance is <= stopBelow and return that value instead of the
 // minimum. Otherwise, and whenever the minimum exceeds stopBelow, it is exact.
 double stopBelow{-1};
 SupportIndex(const Topology&t,std::uint32_t c,double stop=-1):top(&t),component(c),stopBelow(stop){
  bx=(t.size[0]+B-1)/B;by=(t.size[1]+B-1)/B;const std::size_t blocks=std::size_t(bx)*std::size_t(by);
  start.assign(blocks+1,0);zlo.assign(blocks,INT_MAX);zhi.assign(blocks,INT_MIN);
  std::vector<Col>found;std::vector<Sp>compact;
  for(const auto&col:t.columns){Col out{col.x,col.y,INT_MAX,INT_MIN,std::uint32_t(compact.size()),0};
   for(auto k=col.first;k<col.first+col.count;++k){const auto&sp=t.spans[k];if(sp.component!=c)continue;
    compact.push_back({sp.lo,sp.hi});++out.count;out.zlo=std::min(out.zlo,sp.lo);out.zhi=std::max(out.zhi,sp.hi);}
   if(out.count){found.push_back(out);++start[block(col.x,col.y)+1];}}
  for(std::size_t i=1;i<start.size();++i)start[i]+=start[i-1];
  cols.resize(found.size());std::vector<std::uint32_t>fill(start.begin(),start.end()-1);
  // Block-major copies; spans of a column stay contiguous and in order.
  spans.reserve(compact.size());
  std::vector<std::size_t>slot(found.size());for(std::size_t i=0;i<found.size();++i)slot[i]=fill[block(found[i].x,found[i].y)]++;
  std::vector<std::size_t>order(found.size());for(std::size_t i=0;i<found.size();++i)order[slot[i]]=i;
  for(std::size_t j=0;j<order.size();++j){Col col=found[order[j]];const auto first=std::uint32_t(spans.size());
   for(std::uint32_t k=0;k<col.count;++k)spans.push_back(compact[col.first+k]);col.first=first;cols[j]=col;
   const auto bl=block(col.x,col.y);zlo[bl]=std::min(zlo[bl],col.zlo);zhi[bl]=std::max(zhi[bl],col.zhi);}
  // speedfix: the memo of this index's answers (see query()); sized by the component's spans.
  static const bool memoOn=[]{const char*e=std::getenv("GFY_SF_MEMO");return !e||std::atoi(e)!=0;}();
  if(memoOn){std::size_t n=1024;while(n<16384&&n<spans.size())n*=2;memo.reset(new MemoSlot[n]());memoMask=n-1;}
  // speedfix: the columns' coordinates side by side (the SIMD column tests), with room for a full last group
  colX.resize(cols.size()+4);colY.resize(cols.size()+4);colZlo.resize(cols.size()+4);colZhi.resize(cols.size()+4);
  for(std::size_t j=0;j<cols.size();++j){colX[j]=cols[j].x;colY[j]=cols[j].y;colZlo[j]=cols[j].zlo;colZhi[j]=cols[j].zhi;}
 }
 // speedfix: a fit asks the same corner point many times (the boxes of neighbouring candidate splits share corners, 30-40 %
 // of the queries). query() is a pure function of the point, so each answer is kept: a direct-mapped table keyed by the
 // point's exact bits, one entry per slot, shared by the crew's threads (a slot is a sequence lock: a reader takes an
 // entry only when its sequence is even and unchanged across the read and the key matches; a writer that finds the slot
 // busy simply does not store). A memo answer is the same double query() returns for that point.
 struct MemoSlot {std::atomic<std::uint64_t> seq,x,y,z,v;};
 mutable std::unique_ptr<MemoSlot[]> memo;std::size_t memoMask{};std::vector<int> colX,colY,colZlo,colZhi;
 static bool simdQuery(){static const bool on=[]{const char*e=std::getenv("GFY_SF_SIMD_QUERY");return !e||std::atoi(e)!=0;}();return on;}
 double query(V point)const{
  if(!memo)return queryExact(point);
  std::uint64_t key[3];std::memcpy(key,point.data(),sizeof key);
  std::uint64_t h=key[0]*0x9E3779B97F4A7C15ull^key[1]*0xC2B2AE3D27D4EB4Full^key[2]*0x165667B19E3779F9ull;h^=h>>31;h*=0xBF58476D1CE4E5B9ull;h^=h>>29;
  auto&slot=memo[h&memoMask];
  const std::uint64_t s1=slot.seq.load(std::memory_order_acquire);
  if(s1&&!(s1&1u)){const auto x=slot.x.load(std::memory_order_relaxed),y=slot.y.load(std::memory_order_relaxed),z=slot.z.load(std::memory_order_relaxed),
    v=slot.v.load(std::memory_order_relaxed);std::atomic_thread_fence(std::memory_order_acquire);
   if(slot.seq.load(std::memory_order_relaxed)==s1&&x==key[0]&&y==key[1]&&z==key[2]){double d;std::memcpy(&d,&v,sizeof d);return d;}}
  const double d=queryExact(point);
  std::uint64_t s0=slot.seq.load(std::memory_order_relaxed);
  if(!(s0&1u)&&slot.seq.compare_exchange_strong(s0,s0+1,std::memory_order_acq_rel,std::memory_order_relaxed)){
   std::atomic_thread_fence(std::memory_order_release);std::uint64_t bits;std::memcpy(&bits,&d,sizeof bits);
   slot.x.store(key[0],std::memory_order_relaxed);slot.y.store(key[1],std::memory_order_relaxed);slot.z.store(key[2],std::memory_order_relaxed);
   slot.v.store(bits,std::memory_order_relaxed);slot.seq.store(s0+2,std::memory_order_release);}
  return d;}
 std::size_t block(int x,int y)const{return std::size_t(x/B)*std::size_t(by)+std::size_t(y/B);}
 double queryExact(V point)const{
  double best=std::numeric_limits<double>::infinity();bool found=false;
  const int cx=std::clamp(int(std::floor(point[0]))/B,0,bx-1),cy=std::clamp(int(std::floor(point[1]))/B,0,by-1);
  const int rings=std::max(bx,by);
  auto visit=[&](std::size_t b)->bool{
   const double dx=gap(double(int(b/std::size_t(by))*B)-point[0],point[0]-(int(b/std::size_t(by))*B+double(B)));
   const double dy=gap(double(int(b%std::size_t(by))*B)-point[1],point[1]-(int(b%std::size_t(by))*B+double(B)));
   const double dz=gap(double(zlo[b])-point[2],point[2]-zhi[b]);
   if(dx*dx+dy*dy+dz*dz>best)return false;
#ifdef GFY_RX_PROF
   ++prof().blocks;prof().cols+=start[b+1]-start[b];
#endif
#if defined(__AVX__)
   // speedfix: the column tests four columns at a time (the same IEEE subtractions, maxima, products and sums per lane as
   // the scalar lines below; a lane is a column). A column is looked at, in column order, only when its bound
   // xy + cz * cz is at most the best at the start of its group of four - the scalar loop skips it then too (best only
   // falls) - and then with the scalar loop's own tests against the current best. GFY_SF_SIMD_QUERY=0: the scalar loop.
   if(simdQuery()){const auto e=start[b+1];auto k=start[b];
    const __m256d px=_mm256_set1_pd(point[0]),py=_mm256_set1_pd(point[1]),pz=_mm256_set1_pd(point[2]),one=_mm256_set1_pd(1.),zero=_mm256_setzero_pd();
    for(;k+4<=e;k+=4){
     const __m256d X=_mm256_cvtepi32_pd(_mm_loadu_si128(reinterpret_cast<const __m128i*>(colX.data()+k)));
     const __m256d Y=_mm256_cvtepi32_pd(_mm_loadu_si128(reinterpret_cast<const __m128i*>(colY.data()+k)));
     const __m256d ZL=_mm256_cvtepi32_pd(_mm_loadu_si128(reinterpret_cast<const __m128i*>(colZlo.data()+k)));
     const __m256d ZH=_mm256_cvtepi32_pd(_mm_loadu_si128(reinterpret_cast<const __m128i*>(colZhi.data()+k)));
     const __m256d cdx=_mm256_max_pd(_mm256_max_pd(_mm256_sub_pd(X,px),_mm256_sub_pd(_mm256_sub_pd(px,X),one)),zero);
     const __m256d cdy=_mm256_max_pd(_mm256_max_pd(_mm256_sub_pd(Y,py),_mm256_sub_pd(_mm256_sub_pd(py,Y),one)),zero);
     const __m256d xy=_mm256_add_pd(_mm256_mul_pd(cdx,cdx),_mm256_mul_pd(cdy,cdy));
     const __m256d cz=_mm256_max_pd(_mm256_max_pd(_mm256_sub_pd(ZL,pz),_mm256_sub_pd(pz,ZH)),zero);
     int mask=_mm256_movemask_pd(_mm256_cmp_pd(_mm256_add_pd(xy,_mm256_mul_pd(cz,cz)),_mm256_set1_pd(best),_CMP_LE_OQ));
     if(!mask)continue;
     alignas(32) double xyl[4],czl[4];_mm256_store_pd(xyl,xy);_mm256_store_pd(czl,cz);
     for(int l=0;l<4;++l){if(!((mask>>l)&1))continue;const auto&col=cols[k+std::uint32_t(l)];const double cxy=xyl[l];
      if(cxy>best)continue;if(cxy+czl[l]*czl[l]>best)continue;
      for(std::uint32_t i=0;i<col.count;++i){const auto&sp=spans[col.first+i];
       const double cdz=gap(double(sp.lo)-point[2],point[2]-sp.hi),d=cxy+cdz*cdz;
       if(d<=best){best=d;found=true;if(best==0||best<=stopBelow)return true;}}}}
    for(;k<e;++k){const auto&col=cols[k];
     const double cdx=gap(double(col.x)-point[0],point[0]-col.x-1.);
     const double cdy=gap(double(col.y)-point[1],point[1]-col.y-1.);
     const double xy=cdx*cdx+cdy*cdy;if(xy>best)continue;
     const double cz=gap(double(col.zlo)-point[2],point[2]-col.zhi);if(xy+cz*cz>best)continue;
     for(std::uint32_t i=0;i<col.count;++i){const auto&sp=spans[col.first+i];
      const double cdz=gap(double(sp.lo)-point[2],point[2]-sp.hi),d=xy+cdz*cdz;
      if(d<=best){best=d;found=true;if(best==0||best<=stopBelow)return true;}}}
    return false;}
#endif
   for(auto k=start[b];k<start[b+1];++k){const auto&col=cols[k];
    const double cdx=gap(double(col.x)-point[0],point[0]-col.x-1.);
    const double cdy=gap(double(col.y)-point[1],point[1]-col.y-1.);
    const double xy=cdx*cdx+cdy*cdy;if(xy>best)continue;
    const double cz=gap(double(col.zlo)-point[2],point[2]-col.zhi);if(xy+cz*cz>best)continue;
    for(std::uint32_t i=0;i<col.count;++i){const auto&sp=spans[col.first+i];
     const double cdz=gap(double(sp.lo)-point[2],point[2]-sp.hi),d=xy+cdz*cdz;
     if(d<=best){best=d;found=true;if(best==0||best<=stopBelow)return true;}}}
   return false;};
  for(int ring=0;ring<=rings;++ring){
   if(found&&ring>=2){const double gap=double((ring-1)*B);if(gap*gap>best)break;}
   // Border of the Chebyshev ring only (each block once).
   for(int X=cx-ring;X<=cx+ring;++X){const bool edgeX=X==cx-ring||X==cx+ring;
    for(int Y=cy-ring;Y<=cy+ring;Y+=(edgeX||ring==0)?1:2*ring){
     if(X<0||Y<0||X>=bx||Y>=by)continue;
     const std::size_t b=std::size_t(X)*std::size_t(by)+std::size_t(Y);if(start[b]==start[b+1])continue;
     if(visit(b))return best;}}
  }
  if(!found)throw std::runtime_error("collider_support_distance_range");
  return best;
 }
};
inline const SupportIndex*& activeSupport(){thread_local const SupportIndex* index=nullptr;return index;}
struct ActiveSupport {const SupportIndex* previous;explicit ActiveSupport(const SupportIndex*s):previous(activeSupport()){activeSupport()=s;}
 ~ActiveSupport(){activeSupport()=previous;}ActiveSupport(const ActiveSupport&)=delete;ActiveSupport&operator=(const ActiveSupport&)=delete;};
// speedfix (2026-11-12): the fit crew. The separation's fit threads (LiveSeparationWorker EarlyFits) form one crew; a
// member whose own fits are done helps the fits still running. An owner posts each batch of independent items of its
// prepare() - the candidate splits of one leaf, the two halves of a compact split, the union fits of a merge flush - in its
// own board slot and works through it with an atomic cursor; idle members take items from posted batches. An item writes
// only its own outputs (by index) and the owner waits for every item before it reduces them in index order as before, so a
// fit is the same as on one thread. Members run items under SerialScope with the owner's support index; no thread is
// created. GFY_SF_CREW=0: the former early fits (no crew).
struct CrewBatch {void(*run)(void*,std::size_t){};void*context{};std::size_t n{};std::atomic<std::size_t> next{0};const SupportIndex*support{};};
// speedfix: a greedy fit that nears its leaf cap posts the compact fit of the same part as a speculation; an idle member
// runs it (prepare(top, capture, limits, true): a pure function of its arguments). If the greedy fit reaches the cap (or
// an unsplittable leaf) the owner takes that result instead of running the same compact fit itself; if the greedy fit
// succeeds the speculation is stopped (its cooperative cancel points throw) and dropped. Either way the fit returns what
// it returned before. GFY_SF_SPEC=0: off; GFY_SF_SPEC_AT=<live + unacceptable leaves> when to post (default 56).
struct SpecTask {const Topology*top{};Capture capture;Limits limits;Prepared(*run)(const Topology&,const Capture&,Limits){};
 std::atomic<int> state{0}; // 0 posted, 1 running, 2 finished, 3 withdrawn before it started
 std::atomic<bool> stop{false};Prepared result;std::exception_ptr error;};
struct SpecStopScope {std::atomic<bool>*previous;explicit SpecStopScope(std::atomic<bool>*s):previous(speculationStop()){speculationStop()=s;}
 ~SpecStopScope(){speculationStop()=previous;}SpecStopScope(const SpecStopScope&)=delete;SpecStopScope&operator=(const SpecStopScope&)=delete;};
struct Crew {
 static constexpr unsigned slots=max_threads;
 std::array<std::atomic<CrewBatch*>,slots> board{};std::array<std::atomic<int>,slots> inside{};
 std::array<std::atomic<SpecTask*>,slots> specs{};std::array<std::atomic<int>,slots> specInside{};
 std::atomic<int> fitting{0};std::atomic<bool> stop{false};std::atomic<std::uint64_t> helped{0},speculated{0};
 bool(*extra)(void*){};void*extraCtx{}; // speedfix: the owner's own idle work (EarlyFits: the coarse-limit fits)
 // A member's own batch: posted in its slot, worked through by it and by whoever joins, closed when every item is done.
 template<class F> void run(unsigned own,std::size_t n,F&item,const SupportIndex*support){
  CrewBatch b;b.run=[](void*context,std::size_t k){(*static_cast<F*>(context))(k);};b.context=&item;b.n=n;b.support=support;
  board[own].store(&b,std::memory_order_seq_cst);
  for(std::size_t k;(k=b.next.fetch_add(1,std::memory_order_relaxed))<n;)item(k);
  board[own].store(nullptr,std::memory_order_seq_cst);
  while(inside[own].load(std::memory_order_seq_cst)!=0)_mm_pause();}
 // One helping round on another member's open batch with items left; false when there is none.
 bool helpOnce(unsigned own){
  for(unsigned s=0;s<slots;++s){if(s==own)continue;CrewBatch*b=board[s].load(std::memory_order_acquire);
   if(!b||b->next.load(std::memory_order_relaxed)>=b->n)continue;
   inside[s].fetch_add(1,std::memory_order_seq_cst);
   if(board[s].load(std::memory_order_seq_cst)!=b){inside[s].fetch_sub(1,std::memory_order_release);continue;}
   std::uint64_t done=0;
   {const SerialScope serial;const ActiveSupport guard(b->support);
    for(std::size_t k;(k=b->next.fetch_add(1,std::memory_order_relaxed))<b->n;++done)b->run(b->context,k);}
   inside[s].fetch_sub(1,std::memory_order_release);if(done)helped.fetch_add(done,std::memory_order_relaxed);return true;}
  return false;}
 // Runs another member's posted speculation that nobody has started; false when there is none.
 bool speculate(unsigned own){
  for(unsigned s=0;s<slots;++s){if(s==own)continue;SpecTask*t=specs[s].load(std::memory_order_acquire);
   if(!t||t->state.load(std::memory_order_relaxed)!=0)continue;
   specInside[s].fetch_add(1,std::memory_order_seq_cst);
   if(specs[s].load(std::memory_order_seq_cst)!=t){specInside[s].fetch_sub(1,std::memory_order_release);continue;}
   int expect=0;
   if(t->state.compare_exchange_strong(expect,1,std::memory_order_acq_rel)){
    {const SerialScope serial;const SpecStopScope scope(&t->stop);
     try{t->result=t->run(*t->top,t->capture,t->limits);}catch(...){t->error=std::current_exception();}}
    t->state.store(2,std::memory_order_release);speculated.fetch_add(1,std::memory_order_relaxed);}
   specInside[s].fetch_sub(1,std::memory_order_release);return true;}
  return false;}
 // A member without a fit of its own: runs speculations and helps until no fit of the crew is left (or the crew stops).
 void idle(unsigned own){unsigned spins=0;
  while(fitting.load(std::memory_order_acquire)>0&&!stop.load(std::memory_order_relaxed)){
   if(speculate(own)||helpOnce(own)||(extra&&extra(extraCtx))){spins=0;continue;}
   if(++spins<4096)_mm_pause();else{std::this_thread::yield();spins=2048;}}}
};
// The owner's side of a speculation (one per greedy prepare()).
struct Speculation {Crew*crew{};unsigned slot{};SpecTask task;bool posted{false};
 void post(Crew*c,unsigned s,const Topology&top,const Capture&capture,const Limits&limits,Prepared(*run)(const Topology&,const Capture&,Limits)){
  crew=c;slot=s;task.top=&top;task.capture=capture;task.limits=limits;task.run=run;posted=true;crew->specs[slot].store(&task,std::memory_order_seq_cst);}
 void drain(){while(crew->specInside[slot].load(std::memory_order_seq_cst)!=0)_mm_pause();}
 // The compact fit, when a member has run (or is running) it: true and `out` (or its exception rethrown). False when
 // nobody started it: the caller runs it.
 bool take(Prepared&out){if(!posted)return false;posted=false;crew->specs[slot].store(nullptr,std::memory_order_seq_cst);
  int expect=0;if(task.state.compare_exchange_strong(expect,3,std::memory_order_acq_rel)){drain();return false;}
  while(task.state.load(std::memory_order_acquire)!=2){if(!crew->helpOnce(slot))_mm_pause();}
  drain();if(task.error)std::rethrow_exception(task.error);out=std::move(task.result);return true;}
 ~Speculation(){if(!posted)return;crew->specs[slot].store(nullptr,std::memory_order_seq_cst);task.stop.store(true,std::memory_order_relaxed);
  int expect=0;if(!task.state.compare_exchange_strong(expect,3,std::memory_order_acq_rel))while(task.state.load(std::memory_order_acquire)!=2)_mm_pause();
  drain();}
 Speculation()=default;Speculation(const Speculation&)=delete;Speculation&operator=(const Speculation&)=delete;};
inline unsigned specAt(){static const unsigned v=[]{const char*e=std::getenv("GFY_SF_SPEC");if(e&&std::atoi(e)==0)return 100000u;
  const char*a=std::getenv("GFY_SF_SPEC_AT");return a?unsigned(std::atoi(a)):56u;}();return v;}
struct CrewMember {Crew*crew{};unsigned slot{};};
inline CrewMember& crewMember(){thread_local CrewMember member;return member;}
struct CrewScope {CrewMember previous;CrewScope(Crew*crew,unsigned slot):previous(crewMember()){crewMember()=CrewMember{crew,slot};}
 ~CrewScope(){crewMember()=previous;}CrewScope(const CrewScope&)=delete;CrewScope&operator=(const CrewScope&)=delete;};
// A member's split targets for crew items (capacity kept between items, released by releaseCrewScratch()).
inline std::pair<Region,Region>& crewScratch(){thread_local std::pair<Region,Region> scratch;return scratch;}
inline void releaseCrewScratch();
// Smallest leaf (runs) whose candidate splits are shared with the crew, and smallest merge flush (runs of its pairs).
inline std::size_t crewMinRuns(){static const std::size_t v=[]{const char*e=std::getenv("GFY_SF_CREW_MIN_RUNS");return e?std::size_t(std::atoll(e)):std::size_t(64);}();return v;}
inline std::size_t crewMinFlush(){static const std::size_t v=[]{const char*e=std::getenv("GFY_SF_CREW_MIN_FLUSH");return e?std::size_t(std::atoll(e)):std::size_t(256);}();return v;}
inline double supportDistanceSquared(V point,const Topology&top,std::uint32_t component){RXP(supportMs);
#ifdef GFY_RX_PROF
 ++prof().supports;
#endif
 if(const auto*index=activeSupport();index&&index->top==&top&&index->component==component)return index->query(point);
 // Sparse columns are sorted by y/x. Search bounded neighborhoods rather
 // than testing every anatomical run for every candidate corner. Once any
 // distance <= radius is found, the searched cube contains every nearer run.
 for(double radius=1;radius<131072;radius*=4){
  double best=radius*radius;bool found=false;
  const int y0=std::max(0,int(std::floor(point[1]-radius))-1);
  const int y1=std::min(top.size[1]-1,int(std::ceil(point[1]+radius)));
  const int x0=std::max(0,int(std::floor(point[0]-radius))-1);
  const int x1=std::min(top.size[0]-1,int(std::ceil(point[0]+radius)));
  for(int y=y0;y<=y1;++y){
   const double dy=std::max({double(y)-point[1],point[1]-y-1.,0.});if(dy*dy>best)continue;
   auto col=std::lower_bound(top.columns.begin(),top.columns.end(),std::array<int,2>{{x0,y}},
     [](const gfy_connectivity::Column&c,const std::array<int,2>&p){return gfy_connectivity::xyLess(c.x,c.y,p[0],p[1]);});
   for(;col!=top.columns.end()&&col->y==y&&col->x<=x1;++col){
    const double dx=std::max({double(col->x)-point[0],point[0]-col->x-1.,0.});
    const double xy=dx*dx+dy*dy;if(xy>best)continue;
    for(auto i=col->first;i<col->first+col->count;++i){const auto&s=top.spans[i];if(s.component!=component)continue;
     const double dz=std::max({double(s.lo)-point[2],point[2]-s.hi,0.}),d=xy+dz*dz;
     if(d<=best){best=d;found=true;if(best==0)return 0;}
    }
   }
  }
  if(found)return best;
 }
 throw std::runtime_error("collider_support_distance_range");
}
// Reference evaluation (the former fit path). With a support topology, fit() below
// produces bit-identical boxes without these run passes; this remains the path
// for a null support and the oracle for GFY_P3_VERIFY builds.
inline FittedBox evaluate(const Region&r,const std::array<V,3>&axes,double pitch,const Limits&limits,
                         const Topology*support=nullptr,std::uint32_t component=0){
 RXP(evalMs);
#ifdef GFY_RX_PROF
 ++prof().evals;prof().evalRuns+=r.runs.size();
#endif
 FittedBox out;out.axes=axes;V low{{1e30,1e30,1e30}},high{{-1e30,-1e30,-1e30}};
 for(const auto&s:r.runs){const V a{{double(s.x),double(s.y),double(s.lo)}},
                         b{{double(s.x+1),double(s.y+1),double(s.hi)}};
  for(int i=0;i<3;++i){double mn=0,mx=0;for(int j=0;j<3;++j){const double u=axes[i][j];
    mn+=u*(u>=0?a[j]:b[j]);mx+=u*(u>=0?b[j]:a[j]);}
   low[i]=std::min(low[i],mn);high[i]=std::max(high[i],mx);}}
 out.minimum_size=true;
 for(int i=0;i<3;++i){out.center_cells=add(out.center_cells,mul(axes[i],(low[i]+high[i])*.5));
  const double originalHalf=(high[i]-low[i])*.5;
  if(originalHalf*pitch+1e-12<.003)out.minimum_size=false;
  out.half_cells[i]=std::max(originalHalf,limits.minimum_half_extent_m/pitch);}
 out.proxy_cells=8*out.half_cells[0]*out.half_cells[1]*out.half_cells[2];
 double worst=0;
 for(int bx:{-1,1})for(int by:{-1,1})for(int bz:{-1,1}){
  V q=add(out.center_cells,add(mul(axes[0],bx*out.half_cells[0]),
            add(mul(axes[1],by*out.half_cells[1]),mul(axes[2],bz*out.half_cells[2]))));
  const double qx=q[0],qy=q[1],qz=q[2];
  double best=std::numeric_limits<double>::infinity();
  if(support)best=supportDistanceSquared(q,*support,component);
  else for(const auto&s:r.runs){const double dx=std::max({double(s.x)-qx,qx-double(s.x+1),0.});
   const double dy=std::max({double(s.y)-qy,qy-double(s.y+1),0.});
   const double dz=std::max({double(s.lo)-qz,qz-double(s.hi),0.});
   best=std::min(best,dx*dx+dy*dy+dz*dz);if(best==0)break;}
  worst=std::max(worst,std::sqrt(best));}
 out.gap_cells=worst;return out;}
// evaluate()'s box from already computed per-axis bounds, without its corner gap:
// the statements between its bounds loop and its corner loop, verbatim.
inline FittedBox boxShape(const std::array<V,3>&axes,const V&low,const V&high,double pitch,const Limits&limits){
 FittedBox out;out.axes=axes;
 out.minimum_size=true;
 for(int i=0;i<3;++i){out.center_cells=add(out.center_cells,mul(axes[i],(low[i]+high[i])*.5));
  const double originalHalf=(high[i]-low[i])*.5;
  if(originalHalf*pitch+1e-12<.003)out.minimum_size=false;
  out.half_cells[i]=std::max(originalHalf,limits.minimum_half_extent_m/pitch);}
 out.proxy_cells=8*out.half_cells[0]*out.half_cells[1]*out.half_cells[2];
 return out;}
// evaluate()'s corner loop with a support topology, verbatim: gap = the maximum over
// the eight corners of sqrt(squared support distance). stop(worstSoFar) may end the
// loop early (false is returned and out.gap_cells is left unset): worstSoFar never
// exceeds the full gap, so it is a lower bound the caller may decide with; a box whose
// loop stopped is discarded, never stored.
template<class Stop>
inline bool cornerGap(FittedBox&out,const Topology&support,std::uint32_t component,Stop&&stop){
 const auto&axes=out.axes;
 double worst=0;
 for(int bx:{-1,1})for(int by:{-1,1})for(int bz:{-1,1}){
  V q=add(out.center_cells,add(mul(axes[0],bx*out.half_cells[0]),
            add(mul(axes[1],by*out.half_cells[1]),mul(axes[2],bz*out.half_cells[2]))));
  const double best=supportDistanceSquared(q,support,component);
  worst=std::max(worst,std::sqrt(best));
  if(stop(worst))return false;}
 out.gap_cells=worst;return true;}
// evaluate()'s box from per-axis bounds with a support topology: boxShape + the full
// corner loop.
inline FittedBox boxFromBounds(const std::array<V,3>&axes,const V&low,const V&high,double pitch,const Limits&limits,
                               const Topology&support,std::uint32_t component){
 RXP(evalMs);
#ifdef GFY_RX_PROF
 ++prof().evals;
#endif
 FittedBox out=boxShape(axes,low,high,pitch,limits);
 cornerGap(out,support,component,[](double){return false;});
 return out;}
inline Moments moments(const std::vector<Run>&runs,const Topology&top){Moments m;
 const long double p=top.pitch,ox=top.origin[0],oy=top.origin[1],oz=top.origin[2];
 for(const auto&r:runs)addMoment(m,r,p,ox,oy,oz);
 return m;}
// Jacobi principal axes of unit-pitch moments (the former principalAxes body after
// its moments() call, verbatim).
inline std::array<V,3> principalAxes(const Moments&m){RXP(axesMs);
 const long double n=m.n;
 double a[3][3]={{double(m.q[0]/n-m.s[0]*m.s[0]/n/n),double(m.xy/n-m.s[0]*m.s[1]/n/n),double(m.xz/n-m.s[0]*m.s[2]/n/n)},
                 {0,double(m.q[1]/n-m.s[1]*m.s[1]/n/n),double(m.yz/n-m.s[1]*m.s[2]/n/n)},
                 {0,0,double(m.q[2]/n-m.s[2]*m.s[2]/n/n)}};
 a[1][0]=a[0][1];a[2][0]=a[0][2];a[2][1]=a[1][2];
 double v[3][3]={{1,0,0},{0,1,0},{0,0,1}};
 for(int iter=0;iter<24;++iter){int p=0,q=1;double largest=std::abs(a[0][1]);
  for(int i=0;i<3;++i)for(int j=i+1;j<3;++j)if(std::abs(a[i][j])>largest){largest=std::abs(a[i][j]);p=i;q=j;}
  if(largest<1e-9)break;
  const double angle=.5*std::atan2(2*a[p][q],a[q][q]-a[p][p]);
  const double c=std::cos(angle),s=std::sin(angle);
  for(int k=0;k<3;++k){const double u=a[k][p],w=a[k][q];a[k][p]=c*u-s*w;a[k][q]=s*u+c*w;}
  for(int k=0;k<3;++k){const double u=a[p][k],w=a[q][k];a[p][k]=c*u-s*w;a[q][k]=s*u+c*w;}
  for(int k=0;k<3;++k){const double u=v[k][p],w=v[k][q];v[k][p]=c*u-s*w;v[k][q]=s*u+c*w;}}
 std::array<int,3> order{{0,1,2}};
 std::sort(order.begin(),order.end(),[&](int i,int j){return a[i][i]>a[j][j];});
 std::array<V,3>axes{{V{v[0][order[0]],v[1][order[0]],v[2][order[0]]},
                       V{v[0][order[1]],v[1][order[1]],v[2][order[1]]},V{}}};
 axes[2]=cross(axes[0],axes[1]);return axes;}
inline std::array<V,3> principalAxes(const Region&r){
 Topology unit;unit.pitch=1;return principalAxes(moments(r.runs,unit));}
inline bool acceptableCells(const FittedBox&b,std::uint64_t cells,double pitch,const Limits&limits){
 return b.minimum_size&&b.proxy_cells/cells<=limits.max_box_ratio&&
        b.gap_cells*pitch<=limits.max_corner_gap_m;}
inline bool acceptable(const FittedBox&b,const Region&r,double pitch,const Limits&limits){
 return acceptableCells(b,r.cells,pitch,limits);}
inline double componentGap(const FittedBox&box,const Topology&support,std::uint32_t component){
 double worst=0;
 for(int x:{-1,1})for(int y:{-1,1})for(int z:{-1,1}){
  const auto p=add(box.center_cells,add(mul(box.axes[0],x*box.half_cells[0]),add(mul(box.axes[1],y*box.half_cells[1]),mul(box.axes[2],z*box.half_cells[2]))));
  double best=box.gap_cells*box.gap_cells;
  for(const auto&column:support.columns){const auto dx=std::max({double(column.x)-p[0],p[0]-column.x-1.,0.});
   const auto dy=std::max({double(column.y)-p[1],p[1]-column.y-1.,0.});const double xy=dx*dx+dy*dy;if(xy>=best)continue;
   for(auto i=column.first;i<column.first+column.count;++i){const auto&run=support.spans[i];if(run.component!=component)continue;
    const auto dz=std::max({double(run.lo)-p[2],p[2]-run.hi,0.});best=std::min(best,xy+dz*dz);}
   if(best==0)break;
  }
  worst=std::max(worst,std::sqrt(best));
 }
 return worst;
}
// The former fit(), verbatim (reference / null support / verify oracle).
inline bool fitReference(Region&r,double pitch,const Limits&limits,const std::array<V,3>&boneAxes,
                const Topology*support=nullptr,std::uint32_t component=0){
 if(r.evaluated)return acceptable(r.fitted,r,pitch,limits);
 const std::array<V,3>identity{{V{1,0,0},V{0,1,0},V{0,0,1}}};
 // A leaf boundary is an implementation detail. A proxy corner touching a
 // neighboring leaf of THIS SAME element is still supported anatomy. Looking
 // only inside the current leaf caused endless subdivisions of thin slices.
 auto error=[&](const FittedBox&b){return std::max({0.,
  b.proxy_cells/r.cells/limits.max_box_ratio-1.,
  b.gap_cells*pitch/std::max(limits.max_corner_gap_m,pitch)-1.,b.minimum_size?0.:1.});};
 r.fitted=evaluate(r,identity,pitch,limits,support,component);
 auto consider=[&](const std::array<V,3>&axes){
  auto candidate=evaluate(r,axes,pitch,limits,support,component);
  const double a=error(r.fitted),b=error(candidate);
  if(b<a-1e-9||(std::abs(a-b)<1e-9&&candidate.proxy_cells<r.fitted.proxy_cells))r.fitted=candidate;
 };
 if(!acceptable(r.fitted,r,pitch,limits)||r.box_cells>r.cells*1.08)consider(principalAxes(r));
 consider(boneAxes);
 r.evaluated=true;return acceptable(r.fitted,r,pitch,limits);}
// Fused fit (P3 step A): the same decisions and boxes as fitReference() from a
// region summary instead of run passes.
//  - Identity axes: evaluate()'s per-run mn/mx is ((0+u0*c0)+u1*c1)+u2*c2 with
//    u = e_i; 0*c = +0 for the non-negative cell coordinates, so mn == double(x|y|lo)
//    and mx == double(x+1|y+1|hi) exactly, and the bounds equal the region's integer
//    lo/hi, which append() maintains over the same runs.
//  - Bone axes: bounds from summarize() (the identical per-run expression, min/max
//    exact and order independent).
//  - Principal axes: from the summary's unit moments (bit-identical to
//    moments(runs)), followed by the one remaining run pass (principalPass).
// support must be non-null (prepare() always passes one).
template<class Pass>
inline FittedBox fitSummary(const std::array<int,3>&lo,const std::array<int,3>&hi,std::uint64_t cells,std::uint64_t boxCells,
   const V&boneLow,const V&boneHigh,const Moments&unit,Pass&&principalPass,double pitch,const Limits&limits,
   const std::array<V,3>&boneAxes,const Topology&support,std::uint32_t component,const std::array<V,3>*knownAxes=nullptr){
 const std::array<V,3>identity{{V{1,0,0},V{0,1,0},V{0,0,1}}};
 auto error=[&](const FittedBox&b){return std::max({0.,
  b.proxy_cells/cells/limits.max_box_ratio-1.,
  b.gap_cells*pitch/std::max(limits.max_corner_gap_m,pitch)-1.,b.minimum_size?0.:1.});};
 FittedBox fitted=boxFromBounds(identity,V{{double(lo[0]),double(lo[1]),double(lo[2])}},
                                V{{double(hi[0]),double(hi[1]),double(hi[2])}},pitch,limits,support,component);
 // consider() of fitReference(): replace iff b<a-1e-9 || (|a-b|<1e-9 && smaller
 // proxy), a=error(fitted), b=error(candidate). b == max(noGap, gap term) where
 // noGap is the max of the other terms, and the gap term is non-decreasing in the
 // gap (monotone rounding), which is the max over the corners. So noGap and the
 // running corner max give lower bounds of b. The corner queries stop as soon as the
 // bound proves that neither condition can hold for any b above it: the candidate
 // then loses exactly as in fitReference(), and its (unstored) gap is not needed.
 // A candidate that may win completes all eight corners, so every stored gap is exact.
 auto consider=[&](const std::array<V,3>&axes,const V&low,const V&high){
  RXP(evalMs);
#ifdef GFY_RX_PROF
  ++prof().evals;
#endif
  FittedBox candidate=boxShape(axes,low,high,pitch,limits);
  const double a=error(fitted);
  const double noGap=std::max({0.,candidate.proxy_cells/cells/limits.max_box_ratio-1.,candidate.minimum_size?0.:1.});
  auto cannotReplace=[&](double gapLower){
   const double lower=std::max(noGap,gapLower*pitch/std::max(limits.max_corner_gap_m,pitch)-1.);
   if(!(lower>=a-1e-9))return false;                          // b<a-1e-9 still possible
   if(!(candidate.proxy_cells<fitted.proxy_cells))return true; // tie-break cannot pick it
   return lower>=a&&std::abs(a-lower)>=1e-9;};                  // |a-b|>=1e-9 for every b>=lower
  if(cannotReplace(0.)){RXC(skippedBoxes,1);return;}
  if(!cornerGap(candidate,support,component,cannotReplace)){RXC(stoppedBoxes,1);return;}
  const double b=error(candidate);
  if(b<a-1e-9||(std::abs(a-b)<1e-9&&candidate.proxy_cells<fitted.proxy_cells))fitted=candidate;
 };
 if(!acceptableCells(fitted,cells,pitch,limits)||boxCells>cells*1.08){
  const auto axes=knownAxes?*knownAxes:principalAxes(unit);V low{{1e30,1e30,1e30}},high{{-1e30,-1e30,-1e30}}; // speedfix: knownAxes
  principalPass(axes,low,high);
  consider(axes,low,high);}
 consider(boneAxes,boneLow,boneHigh);
 return fitted;}
// speedfix: a split side's summary - everything fit() reads of a region except its runs, the principal pass already done
// (principalAxes(unit) and the bounds boundRuns gives under them, as fitSummary's pass computes them).
struct SideSummary {std::array<int,3> lo{},hi{};std::uint64_t cells{},box_cells{};V boneLow{},boneHigh{};Moments unit;std::array<V,3> paxes{};V plow{},phigh{};std::size_t runs{};};
inline void summarizeSide(const Region&r,SideSummary&s){s.lo=r.lo;s.hi=r.hi;s.cells=r.cells;s.box_cells=r.box_cells;s.boneLow=r.boneLow;s.boneHigh=r.boneHigh;
 s.unit=r.unit;s.runs=r.runs.size();s.paxes=principalAxes(r.unit);s.plow=V{{1e30,1e30,1e30}};s.phigh=V{{-1e30,-1e30,-1e30}};
 boundRuns(r.runs.data(),r.runs.size(),s.paxes,s.plow,s.phigh);}
// speedfix: stage 1 of the candidate scoring without Regions. splitRuns writes exactly the runs splitInto /
// splitOrientedInto append to each side, in the same order (the same per-run expressions; append skips empty parts, here a
// part is written always and counted only when not empty), into two thread-local buffers. summarizeRuns gives a side's
// summary as finish() + summarize() + the principal pass would from those runs (the same kernels on the same runs).
struct SplitBuffers {std::vector<Run> a,b;};
inline SplitBuffers& splitBuffers(){thread_local SplitBuffers buffers;return buffers;}
inline void releaseCrewScratch(){crewScratch()=std::pair<Region,Region>();splitBuffers()=SplitBuffers();}
inline void splitRuns(const Region&r,bool oriented,int axis,int plane,const V&normal,double at,SplitBuffers&buf,std::size_t&na,std::size_t&nb){
 const std::size_t n=r.runs.size();if(buf.a.size()<n+1)buf.a.resize(n+1);if(buf.b.size()<n+1)buf.b.resize(n+1);
 Run*A=buf.a.data();Run*B=buf.b.data();const Run*s=r.runs.data();std::size_t ia=0,ib=0;
 if(!oriented){
  if(axis==0)for(std::size_t i=0;i<n;++i){const Run q=s[i];const bool toA=q.x<plane;A[ia]=q;B[ib]=q;ia+=toA;ib+=!toA;}
  else if(axis==1)for(std::size_t i=0;i<n;++i){const Run q=s[i];const bool toA=q.y<plane;A[ia]=q;B[ib]=q;ia+=toA;ib+=!toA;}
  else for(std::size_t i=0;i<n;++i){const Run q=s[i];const int ha=std::min(q.hi,plane),lb=std::max(q.lo,plane);
   A[ia]=Run{q.x,q.y,q.lo,ha};B[ib]=Run{q.x,q.y,lb,q.hi};ia+=q.lo<ha;ib+=lb<q.hi;}}
 else if(std::abs(normal[2])<1e-12)for(std::size_t i=0;i<n;++i){const Run q=s[i];const double xy=(q.x+.5)*normal[0]+(q.y+.5)*normal[1];
  const bool toA=xy<at;A[ia]=q;B[ib]=q;ia+=toA;ib+=!toA;}
 else{const bool up=normal[2]>0;
  for(std::size_t i=0;i<n;++i){const Run q=s[i];const double xy=(q.x+.5)*normal[0]+(q.y+.5)*normal[1];
   const double z=(at-xy)/normal[2]-.5;const auto split=up?std::ceil(z):std::floor(z)+1.;
   const int cut=split<q.lo?q.lo:(split>q.hi?q.hi:int(split));
   if(up){A[ia]=Run{q.x,q.y,q.lo,cut};B[ib]=Run{q.x,q.y,cut,q.hi};ia+=q.lo<cut;ib+=cut<q.hi;}
   else{B[ib]=Run{q.x,q.y,q.lo,cut};A[ia]=Run{q.x,q.y,cut,q.hi};ib+=q.lo<cut;ia+=cut<q.hi;}}}
 na=ia;nb=ib;}
inline bool summarizeRuns(const Run*p,std::size_t m,const std::array<V,3>&bone,SideSummary&s){
 if(!m)return false;
 std::array<int,3> lo{{INT32_MAX,INT32_MAX,INT32_MAX}},hi{{0,0,0}};std::uint64_t cells=0;
 for(std::size_t i=0;i<m;++i){const Run&q=p[i];cells+=std::uint64_t(q.hi-q.lo);
  lo[0]=std::min(lo[0],q.x);lo[1]=std::min(lo[1],q.y);lo[2]=std::min(lo[2],q.lo);
  hi[0]=std::max(hi[0],q.x+1);hi[1]=std::max(hi[1],q.y+1);hi[2]=std::max(hi[2],q.hi);}
 if(!cells)return false;
 s.lo=lo;s.hi=hi;s.cells=cells;s.box_cells=1;for(int a=0;a<3;++a)s.box_cells*=std::uint64_t(hi[a]-lo[a]);s.runs=m;
 s.boneLow=V{{1e30,1e30,1e30}};s.boneHigh=V{{-1e30,-1e30,-1e30}};
 if(!exactMoments(cells,lo,hi)){ExactSums unused;scanRunsMaybeParallel<false>(p,m,bone,s.boneLow,s.boneHigh,unused);
  Moments mo;for(std::size_t i=0;i<m;++i)addUnitMoment(mo,p[i]);s.unit=mo;}
 else{ExactSums sums;scanRunsMaybeParallel<true>(p,m,bone,s.boneLow,s.boneHigh,sums);s.unit=toMoments(sums);}
 s.paxes=principalAxes(s.unit);s.plow=V{{1e30,1e30,1e30}};s.phigh=V{{-1e30,-1e30,-1e30}};boundRuns(p,m,s.paxes,s.plow,s.phigh);
 return true;}
// fit() of the region a summary was taken from: the same fitSummary call, its principal pass answered from the summary.
inline FittedBox fitSide(const SideSummary&s,double pitch,const Limits&limits,const std::array<V,3>&boneAxes,const Topology&support,std::uint32_t component){
 return fitSummary(s.lo,s.hi,s.cells,s.box_cells,s.boneLow,s.boneHigh,s.unit,[&](const std::array<V,3>&,V&low,V&high){low=s.plow;high=s.phigh;},
   pitch,limits,boneAxes,support,component,&s.paxes);}
// A shell (Region without runs) of a summarised side and its fit, as prepare() keeps for a scored candidate.
inline Region sideShell(const SideSummary&s,const FittedBox&fitted){Region r;r.lo=s.lo;r.hi=s.hi;r.cells=s.cells;r.box_cells=s.box_cells;
 r.fitted=fitted;r.evaluated=true;r.boneLow=s.boneLow;r.boneHigh=s.boneHigh;r.unit=s.unit;return r;}
#ifdef GFY_P3_VERIFY
inline bool sameBits(const void*a,const void*b,std::size_t n){return std::memcmp(a,b,n)==0;}
inline bool sameBox(const FittedBox&a,const FittedBox&b){
 return sameBits(&a.center_cells,&b.center_cells,sizeof(V))&&sameBits(&a.axes,&b.axes,sizeof(a.axes))&&
        sameBits(&a.half_cells,&b.half_cells,sizeof(V))&&sameBits(&a.proxy_cells,&b.proxy_cells,sizeof(double))&&
        sameBits(&a.gap_cells,&b.gap_cells,sizeof(double))&&a.minimum_size==b.minimum_size;}
inline bool sameMoments(const Moments&a,const Moments&b){
 return sameBits(&a.n,&b.n,sizeof(long double))&&sameBits(a.s,b.s,sizeof(a.s))&&sameBits(a.q,b.q,sizeof(a.q))&&
        sameBits(&a.xy,&b.xy,sizeof(long double))&&sameBits(&a.xz,&b.xz,sizeof(long double))&&sameBits(&a.yz,&b.yz,sizeof(long double));}
// Check a region's summary (bounds, fused bone bounds and unit moments) against
// direct recomputation from its runs, and its fused fit against fitReference().
inline void verifyRegion(const Region&r,const FittedBox&fitted,double pitch,const Limits&limits,const std::array<V,3>&boneAxes,
                         const Topology&support,std::uint32_t component){
 Region ref;ref.runs=r.runs;ref.lo=r.lo;ref.hi=r.hi;ref.cells=r.cells;ref.box_cells=r.box_cells;
 {std::array<int,3>lo{{INT32_MAX,INT32_MAX,INT32_MAX}},hi{};std::uint64_t cells=0;
  for(const auto&s:r.runs){cells+=std::uint64_t(s.hi-s.lo);lo[0]=std::min(lo[0],s.x);lo[1]=std::min(lo[1],s.y);lo[2]=std::min(lo[2],s.lo);
   hi[0]=std::max(hi[0],s.x+1);hi[1]=std::max(hi[1],s.y+1);hi[2]=std::max(hi[2],s.hi);}
  require(lo==r.lo&&hi==r.hi&&cells==r.cells,"p3_verify_region_bounds");}
 Topology unitTop;unitTop.pitch=1;require(sameMoments(moments(r.runs,unitTop),r.unit),"p3_verify_unit_moments");
 {V low{{1e30,1e30,1e30}},high{{-1e30,-1e30,-1e30}};for(const auto&s:r.runs)boundRun(s,boneAxes,low,high);
  require(sameBits(&low,&r.boneLow,sizeof(V))&&sameBits(&high,&r.boneHigh,sizeof(V)),"p3_verify_bone_bounds");}
 fitReference(ref,pitch,limits,boneAxes,&support,component);
 require(sameBox(ref.fitted,fitted),"p3_verify_fit");}
#endif
inline bool fit(Region&r,double pitch,const Limits&limits,const std::array<V,3>&boneAxes,
                const Topology*support=nullptr,std::uint32_t component=0){
 if(r.evaluated)return acceptable(r.fitted,r,pitch,limits);
 if(!support)return fitReference(r,pitch,limits,boneAxes,support,component);
 r.fitted=fitSummary(r.lo,r.hi,r.cells,r.box_cells,r.boneLow,r.boneHigh,r.unit,
   [&](const std::array<V,3>&axes,V&low,V&high){boundRuns(r.runs.data(),r.runs.size(),axes,low,high);},
   pitch,limits,boneAxes,*support,component);
#ifdef GFY_P3_VERIFY
 verifyRegion(r,r.fitted,pitch,limits,boneAxes,*support,component);
#endif
 r.evaluated=true;return acceptable(r.fitted,r,pitch,limits);}
// r (just split again from the same parent by the same deterministic split) takes
// the summary and fit of shell s, which were computed from identical runs.
inline void adopt(Region&r,const Region&s,double pitch,const Limits&limits,const std::array<V,3>&boneAxes,
                  const Topology&support,std::uint32_t component){
 r.fitted=s.fitted;r.evaluated=s.evaluated;r.boneLow=s.boneLow;r.boneHigh=s.boneHigh;r.unit=s.unit;
#ifdef GFY_P3_VERIFY
 require(r.lo==s.lo&&r.hi==s.hi&&r.cells==s.cells&&r.box_cells==s.box_cells&&r.evaluated,"p3_verify_adopt");
 verifyRegion(r,r.fitted,pitch,limits,boneAxes,support,component);
#else
 (void)pitch;(void)limits;(void)boneAxes;(void)support;(void)component;
#endif
}
// Moments of the concatenation a.runs ++ b.runs, i.e. the fold moments() performs.
// Under the exactMoments() certificate for the union every partial sum in any order
// is exact, so the fold equals the componentwise sum a.unit + b.unit (each of which
// is itself that exact sum). Otherwise continue the fold from a.unit over b.runs.
inline Moments unionMoments(const Region&a,const Region&b,std::uint64_t cells,const std::array<int,3>&lo,const std::array<int,3>&hi){
 Moments m=a.unit;
 if(exactMoments(cells,lo,hi)){const auto&u=b.unit;
  m.n+=u.n;for(int i=0;i<3;++i){m.s[i]+=u.s[i];m.q[i]+=u.q[i];}m.xy+=u.xy;m.xz+=u.xz;m.yz+=u.yz;return m;}
 RXC(momentFolds,1);RXC(momentFoldRuns,b.runs.size());
 for(const auto&r:b.runs)addUnitMoment(m,r);
 return m;}
struct UnionShape {std::array<int,3> lo,hi;std::uint64_t cells,box_cells;V boneLow,boneHigh;};
inline UnionShape unionShape(const Region&a,const Region&b){UnionShape u;
 for(int i=0;i<3;++i){u.lo[i]=std::min(a.lo[i],b.lo[i]);u.hi[i]=std::max(a.hi[i],b.hi[i]);
  u.boneLow[i]=std::min(a.boneLow[i],b.boneLow[i]);u.boneHigh[i]=std::max(a.boneHigh[i],b.boneHigh[i]);}
 u.cells=a.cells+b.cells;require(u.cells>0,"collider_empty_region");
 u.box_cells=1;for(int i=0;i<3;++i)u.box_cells*=std::uint64_t(u.hi[i]-u.lo[i]);return u;}
// fit() of the region a.runs ++ b.runs (the former combine()) without materializing
// it: bounds and bone bounds are the min/max of the parts (append() computes
// exactly these), moments via unionMoments, the principal pass visits a then b.
inline FittedBox fitUnion(const Region&a,const Region&b,double pitch,const Limits&limits,const std::array<V,3>&boneAxes,
                          const Topology&support,std::uint32_t component){RXP(combineMs);
 const auto u=unionShape(a,b);const auto unit=unionMoments(a,b,u.cells,u.lo,u.hi);
 auto fitted=fitSummary(u.lo,u.hi,u.cells,u.box_cells,u.boneLow,u.boneHigh,unit,
   [&](const std::array<V,3>&axes,V&low,V&high){boundRuns(a.runs.data(),a.runs.size(),axes,low,high);
                                                   boundRuns(b.runs.data(),b.runs.size(),axes,low,high);},
   pitch,limits,boneAxes,support,component);
#ifdef GFY_P3_VERIFY
 {Region ref;ref.runs=a.runs;ref.runs.insert(ref.runs.end(),b.runs.begin(),b.runs.end());
  ref.lo=u.lo;ref.hi=u.hi;ref.cells=u.cells;ref.box_cells=u.box_cells;ref.boneLow=u.boneLow;ref.boneHigh=u.boneHigh;ref.unit=unit;
  verifyRegion(ref,fitted,pitch,limits,boneAxes,support,component);}
#endif
 return fitted;}
// speedfix: true when fitUnion(a,b) is certain to give a box acceptableCells() refuses - then the merge flush never uses
// that fit (only acceptable unions are queued). fitUnion keeps one of three boxes (identity, principal, bone) of the
// union; each is tested as acceptableCells tests the kept one: too small, too much proxy, or one corner whose support
// distance alone exceeds the corner-gap limit (the box's gap is the largest corner distance). Only when all three fail
// is the union refused here; otherwise fitUnion runs as before. The boxes are built by the same calls fitUnion makes.
inline bool unionRefused(const Region&a,const Region&b,double pitch,const Limits&limits,const std::array<V,3>&boneAxes,
                         const Topology&support,std::uint32_t component){
 const auto u=unionShape(a,b);
 // The corners are asked farthest from the two leaves' boxes first (a refusing corner is usually found at once; the order
 // does not change the answer: refused as soon as any corner is too far, accepted only when none is).
 auto boxGap2=[](const V&q,const Region&r){double s=0;for(int a=0;a<3;++a){const double g=std::max(std::max(double(r.lo[a])-q[a],q[a]-double(r.hi[a])),0.);s+=g*g;}return s;};
 auto refused=[&](const FittedBox&box){if(!box.minimum_size||!(box.proxy_cells/u.cells<=limits.max_box_ratio))return true;
  const auto&ax=box.axes;V q[8];double key[8];int order[8];
  for(int k=0;k<8;++k){const int bx=(k&4)?1:-1,by=(k&2)?1:-1,bz=(k&1)?1:-1;
   q[k]=add(box.center_cells,add(mul(ax[0],bx*box.half_cells[0]),add(mul(ax[1],by*box.half_cells[1]),mul(ax[2],bz*box.half_cells[2]))));
   key[k]=std::min(boxGap2(q[k],a),boxGap2(q[k],b));order[k]=k;}
  std::stable_sort(order,order+8,[&](int x,int y){return key[x]>key[y];});
  for(const int k:order)if(!(std::sqrt(supportDistanceSquared(q[k],support,component))*pitch<=limits.max_corner_gap_m))return true;
  return false;};
 const std::array<V,3>identity{{V{1,0,0},V{0,1,0},V{0,0,1}}};
 if(!refused(boxShape(identity,V{{double(u.lo[0]),double(u.lo[1]),double(u.lo[2])}},V{{double(u.hi[0]),double(u.hi[1]),double(u.hi[2])}},pitch,limits)))return false;
 if(!refused(boxShape(boneAxes,u.boneLow,u.boneHigh,pitch,limits)))return false;
 const auto unit=unionMoments(a,b,u.cells,u.lo,u.hi);const auto axes=principalAxes(unit);V low{{1e30,1e30,1e30}},high{{-1e30,-1e30,-1e30}};
 boundRuns(a.runs.data(),a.runs.size(),axes,low,high);boundRuns(b.runs.data(),b.runs.size(),axes,low,high);
 return refused(boxShape(axes,low,high,pitch,limits));}
// Copy on commit: materialize a.runs ++ b.runs with its summary and the fit that
// fitUnion() computed for exactly this concatenation.
inline Region commitUnion(const Region&a,const Region&b,const FittedBox&fitted){RXP(commitMs);
 const auto u=unionShape(a,b);Region out;
 out.unit=unionMoments(a,b,u.cells,u.lo,u.hi);
 out.runs.reserve(a.runs.size()+b.runs.size());
 out.runs.insert(out.runs.end(),a.runs.begin(),a.runs.end());out.runs.insert(out.runs.end(),b.runs.begin(),b.runs.end());
 out.lo=u.lo;out.hi=u.hi;out.cells=u.cells;out.box_cells=u.box_cells;out.boneLow=u.boneLow;out.boneHigh=u.boneHigh;
 out.fitted=fitted;out.evaluated=true;
 RXC(commits,1);RXC(commitRuns,out.runs.size());
 return out;}
}
#ifdef GFY_P3_CERTIFIED_GAP
namespace detail {
// P3 step D, exact-decision form (NOT the default; needs sign-off because it turns
// Prepared::maximum_corner_gap_m into a certified bound). The largest squared cell
// distance t with fl(fl(sqrt(t))*pitch) <= maxGap: the predicate is monotone in t,
// so a binary search over the ordered bit patterns of non-negative doubles finds it.
// A corner whose returned value is <= t has a gap term that is <= maxGap both for the
// returned value and for the exact minimum (which is smaller), so acceptable(), the
// clamped error/violation terms (max(0,...)) and the final certificate decide exactly
// as with exact queries; a box whose largest corner exceeds t gets its exact gap.
inline double certifiedGap2(double pitch,double maxGap){
 auto ok=[&](double t){return std::sqrt(t)*pitch<=maxGap;};
 double hi=(maxGap/pitch+4)*(maxGap/pitch+4);if(!ok(0.))return -1;
 std::uint64_t lo=0,up;std::memcpy(&up,&hi,sizeof up);
 while(lo+1<up){const std::uint64_t mid=lo+(up-lo)/2;double t;std::memcpy(&t,&mid,sizeof t);if(ok(t))lo=mid;else up=mid;}
 double t;std::memcpy(&t,&lo,sizeof t);return t;}
}
#endif
// One minimum-size box for a tiny part (a sliver of a bone's region) that the box fitter cannot represent within the
// proxy/box ratios: mass, COM and inertia are the exact cell moments; the box is the cells' rest AABB clamped to the engine's
// minimum half extent, oriented like the anchor bone. Used only after prepare() refused such a part.
inline Prepared prepareTiny(const Topology&top,const Capture&capture,Limits limits={}){
 validateRigid(capture.rest_to_anchor);validateRigid(capture.anchor_to_world);
 std::vector<detail::Run>all;int lo[3]={1<<30,1<<30,1<<30},hi[3]={-(1<<30),-(1<<30),-(1<<30)};
 for(const auto&c:top.columns)for(auto i=c.first;i<c.first+c.count;++i){const auto&s=top.spans[i];
  if(s.component!=capture.component)continue;all.push_back({c.x,c.y,s.lo,s.hi});
  lo[0]=std::min(lo[0],int(c.x));hi[0]=std::max(hi[0],int(c.x)+1);lo[1]=std::min(lo[1],int(c.y));hi[1]=std::max(hi[1],int(c.y)+1);
  lo[2]=std::min(lo[2],int(s.lo));hi[2]=std::max(hi[2],int(s.hi));}
 require(!all.empty(),"collider_tiny_empty");
 auto moment=detail::moments(all,top);require(moment.n>0,"collider_tiny_cells");
 const long double cube=top.pitch*top.pitch*top.pitch,cellMass=capture.density_kg_m3*cube;
 V restCOM{};for(int a=0;a<3;++a)restCOM[a]=double(moment.s[a]/moment.n);
 Prepared out;out.accepted=capture.accepted;out.component=capture.component;out.anchor_bone=capture.anchor_bone;
 out.owned_cells=std::uint64_t(moment.n);out.anatomical_volume_m3=double(cube*moment.n);out.mass_kg=double(cellMass*moment.n);
 out.com_anchor=transform(capture.rest_to_anchor,restCOM);
 V half{};for(int a=0;a<3;++a)half[a]=std::max(limits.minimum_half_extent_m,.5*(hi[a]-lo[a])*top.pitch);
 // Solid-box inertia of the clamped box (always positive definite), rotated into the anchor frame.
 const double m=out.mass_kg;const double ib[3]={m*(4*(half[1]*half[1]+half[2]*half[2]))/12,m*(4*(half[0]*half[0]+half[2]*half[2]))/12,m*(4*(half[0]*half[0]+half[1]*half[1]))/12};
 const auto&rot=capture.rest_to_anchor.row_major;
 for(int i=0;i<3;++i)for(int j=0;j<3;++j){double v=0;for(int k=0;k<3;++k)v+=rot[i*4+k]*ib[k]*rot[j*4+k];out.inertia_anchor_com[i*3+j]=v;}
 V restCenter{};for(int a=0;a<3;++a)restCenter[a]=top.origin[a]+.5*(lo[a]+hi[a])*top.pitch;
 Box box;box.center_anchor=transform(capture.rest_to_anchor,restCenter);box.half_extents_m=half;
 for(int i=0;i<3;++i)for(int j=0;j<3;++j)box.orientation[i*3+j]=rot[i*4+j];
 out.boxes.push_back(box);out.maximum_box_extent_m=2*std::max({half[0],half[1],half[2]});
 out.maximum_radius_m=std::sqrt(half[0]*half[0]+half[1]*half[1]+half[2]*half[2]);
 const V worldCOM=transform(capture.anchor_to_world,out.com_anchor);
 out.angular_world=capture.parent_angular_world;
 out.linear_world=add(capture.parent_linear_world,cross(capture.parent_angular_world,sub(worldCOM,capture.parent_com_world)));
 out.proxy_to_anatomy_volume_ratio=8*half[0]*half[1]*half[2]/std::max(1e-12,out.anatomical_volume_m3);
 return out;
}
inline Prepared prepare(const Topology&top,const Capture&capture,Limits limits={},bool compactFallback=false){
 // Large torso elements were spending seconds exhausting the small-part
 // greedy strategy before retrying the bounded spatial strategy. Choose it
 // up front above 20 litres; both paths use the same final certificates.
 if(capture.component<top.components.size()&&
    top.components[capture.component].cells*top.pitch*top.pitch*top.pitch>.020)
  compactFallback=true;
 require(std::isfinite(top.pitch)&&top.pitch>=.0005&&top.pitch<=.01,"collider_pitch");
 require(capture.component>0&&capture.component<top.components.size(),"collider_component");
 require(capture.anchor_bone<64&&capture.accepted.owner.spawn_cookie&&capture.accepted.revision,
         "collider_identity");
 require(capture.rigid_pose_certified&&std::isfinite(capture.maximum_skin_deviation_m)&&
         capture.maximum_skin_deviation_m>=0&&capture.maximum_skin_deviation_m<=limits.max_rigid_skin_deviation_m,
         "collider_rigid_pose_unproven");
 require(std::isfinite(capture.density_kg_m3)&&capture.density_kg_m3>=100&&capture.density_kg_m3<=3000,
         "collider_density");
 require(limits.max_boxes>0&&limits.max_boxes<=painter_collision_limits::boxes_per_element&&limits.max_spans>0&&
         limits.max_proxy_ratio>=1&&limits.max_proxy_ratio<=6&&
         limits.max_box_ratio>=limits.max_proxy_ratio&&limits.max_box_ratio<=12&&
         limits.max_corner_gap_m>=0&&limits.max_corner_gap_m<=.05&&
         limits.max_omitted_cell_fraction>=0&&limits.max_omitted_cell_fraction<=.02&&
         limits.max_omitted_distance_m>=0&&limits.max_omitted_distance_m<=.05&&
         limits.minimum_half_extent_m>=.005,"collider_limits");
 validateRigid(capture.rest_to_anchor);validateRigid(capture.anchor_to_world);
 for(V v:{capture.parent_com_world,capture.parent_linear_world,capture.parent_angular_world})
  for(double q:v)require(std::isfinite(q),"collider_velocity_nonfinite");
#ifdef GFY_RX_PROF
 const auto p3Start=std::chrono::steady_clock::now();
#endif
#ifdef GFY_P3_CERTIFIED_GAP
 const detail::SupportIndex supportIndex(top,capture.component,detail::certifiedGap2(top.pitch,limits.max_corner_gap_m));
#else
 const detail::SupportIndex supportIndex(top,capture.component);
#endif
 const detail::ActiveSupport activeSupport(&supportIndex);detail::cancelPoint();
 if(!compactFallback)detail::fitTrace().start=std::chrono::steady_clock::now();
 struct ReleaseScratch{~ReleaseScratch(){detail::releaseCrewScratch();}} releaseScratch; // speedfix: this thread's split targets
 const auto&pose=capture.rest_to_anchor.row_major;
 const std::array<V,3> boneAxes{{V{pose[0],pose[1],pose[2]},
                                  V{pose[4],pose[5],pose[6]},
                                  V{pose[8],pose[9],pose[10]}}};
 detail::Region whole;
 for(const auto&c:top.columns)for(auto i=c.first;i<c.first+c.count;++i){const auto&s=top.spans[i];
  if(s.component!=capture.component)continue;
  require(whole.runs.size()<limits.max_spans,"collider_span_budget");
  detail::append(whole,{c.x,c.y,s.lo,s.hi});}
 detail::cancelPoint();
 detail::finish(whole);detail::summarize(whole,boneAxes);
 require(whole.cells==top.components[capture.component].cells,"collider_component_cells_mismatch");
 const auto originalCells=whole.cells;
 std::vector<detail::Region> leaves;leaves.push_back(std::move(whole));
 std::vector<bool> omitted(1,false);std::uint64_t omittedCells=0;
 // Indices of the leaves that are not omitted, ascending. Every loop that visited
 // leaves in index order skipping omitted ones visits exactly this sequence.
 std::vector<std::size_t> live{0};
 // Per-worker reusable split targets for scored candidates (capacity kept across
 // iterations; at most four pairs of the largest split leaf's run count).
 std::array<std::pair<detail::Region,detail::Region>,detail::max_threads>scratch;
 detail::Speculation speculation; // speedfix: the compact fit run ahead by an idle crew member
 // speedfix: a one-cell leaf's fit at this pitch - the identity box fitSummary keeps (a bone or principal box of one cube is
 // never smaller, and the principal one is not tried: box cells = cells) - passes every acceptance test wherever it lies
 // (its corners are at most sqrt(3) (half - .5) cells from the cell itself, which is anatomy of the part).
 static const bool certainOn=[]{const char*e=std::getenv("GFY_SF_CERTAIN");return !e||std::atoi(e)!=0;}();
 const std::size_t greedyCap=std::min<std::size_t>(64,limits.max_boxes);
 const bool certainFail=certainOn&&!compactFallback&&[&]{const std::array<V,3>unitAxes{{V{1,0,0},V{0,1,0},V{0,0,1}}};
  const auto b=detail::boxShape(unitAxes,V{{0.,0.,0.}},V{{1.,1.,1.}},top.pitch,limits);if(!b.minimum_size||!(b.proxy_cells<=limits.max_box_ratio))return false;
  const double h=std::max({b.half_cells[0],b.half_cells[1],b.half_cells[2]})-.5;return std::sqrt(3.)*h*top.pitch*(1+1e-6)+1e-9<=limits.max_corner_gap_m;}();
 {
#ifdef GFY_RX_PROF
 RXP(subdivMs);
#endif
 while(true){detail::cancelPoint();std::size_t sfBad=0; // speedfix: unacceptable leaves (speculation trigger)
  std::size_t chosen=leaves.size(),largestWaste=0;double badness=-1,waste=-1;
  std::uint64_t total=0;double proxy=0;RXC(iterations,1);
#ifdef GFY_RX_PROF
  const auto scanStart=std::chrono::steady_clock::now();
#endif
  for(const auto i:live){auto&r=leaves[i];total+=r.cells;
   const bool okay=detail::fit(r,top.pitch,limits,boneAxes,&top,capture.component);proxy+=r.fitted.proxy_cells;
   const double empty=r.fitted.proxy_cells-r.cells,ratio=r.fitted.proxy_cells/r.cells;
   if(empty>waste){waste=empty;largestWaste=i;}
   if(!okay){++sfBad;
    const double priority=ratio>limits.max_box_ratio?ratio*std::sqrt(double(r.cells)):empty;
    if(priority>badness){badness=priority;chosen=i;}}}
#ifdef GFY_RX_PROF
  detail::prof().scanMs+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-scanStart).count();
#endif
  require(total>0,"collider_all_collision_omitted");
  if(!compactFallback&&!speculation.posted&&live.size()+sfBad>=detail::specAt()&&!detail::mayFanOut()){const auto member=detail::crewMember(); // speedfix
   if(member.crew)speculation.post(member.crew,member.slot,top,capture,limits,[](const Topology&t,const Capture&c,Limits l){return prepare(t,c,l,true);});}
  // speedfix: the greedy fit is certain to reach its leaf cap - every unacceptable leaf is split before the fit can end
  // (a leaf's fit never changes; a leaf of two or more cells always has a valid axis split; a one-cell leaf, the only kind
  // that can be omitted instead, is acceptable at this pitch: singleCellsAcceptable), each split adds one leaf and none
  // is removed meanwhile, so live + unacceptable - 1 >= cap means the cap check fails before the last of them. That
  // failure restarts with the compact fit, so it is started now (or taken from the speculation). GFY_SF_CERTAIN=0: off.
  if(certainFail&&!compactFallback&&live.size()+sfBad>=greedyCap+1){auto&tr=detail::fitTrace();
   tr.greedyMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tr.start).count();tr.greedyLeaves=live.size();tr.restarts+=1000;
   {Prepared r;if(speculation.take(r)){tr.speculated=1;return r;}}return prepare(top,capture,limits,true);}
  if(chosen==leaves.size()){
   if(double(proxy)/total<=limits.max_proxy_ratio)break;
   chosen=largestWaste;
  }
  const auto active=live.size();
  if(active>=(compactFallback?256:std::min<std::size_t>(64,limits.max_boxes))){
   // A large torso can exhaust the ordinary greedy partition. Retry with a
   // bounded spatial subdivision and coalesce certified unions. This changes
   // neither anatomical ownership nor the volume/distance error limits.
   if(!compactFallback){auto&tr=detail::fitTrace();tr.greedyMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tr.start).count();
    tr.greedyLeaves=live.size();++tr.restarts;{Prepared r;if(speculation.take(r)){tr.speculated=1;return r;}}return prepare(top,capture,limits,true);}
   const auto&r=leaves[chosen];
#ifdef GFY_COLLIDER_DIAGNOSTICS
   for(std::size_t j=0;j<leaves.size();++j){const auto&v=leaves[j];
    std::cerr<<"collider_leaf="<<j<<" chosen="<<(j==chosen)<<" cells="<<v.cells
     <<" ratio="<<v.fitted.proxy_cells/v.cells<<" gap="<<v.fitted.gap_cells*top.pitch
     <<" minimum="<<v.fitted.minimum_size<<" accepted="<<detail::acceptable(v.fitted,v,top.pitch,limits)
     <<" dims="<<v.hi[0]-v.lo[0]<<','<<v.hi[1]-v.lo[1]<<','<<v.hi[2]-v.lo[2]<<'\n';}
#endif
   throw std::runtime_error("collider_box_capacity_refusal ratio="+
       std::to_string(r.fitted.proxy_cells/r.cells)+" gap="+
       std::to_string(r.fitted.gap_cells*top.pitch)+" dims="+
       std::to_string(r.fitted.half_cells[0]*top.pitch*2)+","+
       std::to_string(r.fitted.half_cells[1]*top.pitch*2)+","+
       std::to_string(r.fitted.half_cells[2]*top.pitch*2));}
  const auto&old=leaves[chosen];double bestScore=std::numeric_limits<double>::infinity();std::pair<detail::Region,detail::Region>best;
  static const bool pruneSpecs=[]{const char*e=std::getenv("GFY_SF_PRUNE");return !e||std::atoi(e)!=0;}(); // speedfix
  auto violation=[&](detail::Region&r){
   detail::fit(r,top.pitch,limits,boneAxes,&top,capture.component);
   const double ratio=r.fitted.proxy_cells/r.cells;
   const double gap=r.fitted.gap_cells*top.pitch/std::max(limits.max_corner_gap_m,top.pitch);
   return std::max({0.,ratio/limits.max_box_ratio-1.,gap-1.,r.fitted.minimum_size?0.:1.});
  };
  auto cost=[&](std::pair<detail::Region,detail::Region>&candidate){
   const double a=violation(candidate.first),b=violation(candidate.second);
   // Error is a distance limit, independent of leaf volume. Weighting error
   // by cells hid it in thin peripheral slices and exhausted the box budget.
   return 100.*std::max(a,b)+10.*(a+b)+
       (candidate.first.fitted.proxy_cells+candidate.second.fitted.proxy_cells)/old.cells;
  };
  // Candidate splits in the former order. Each is a pure function of `old`;
  // large leaves evaluate them on up to four threads (each thread activates
  // the exact support index), then the first strictly best score in that
  // order is chosen, exactly as the serial loop did.
  struct Spec{bool oriented;int axis,plane;V normal;double at;};std::vector<Spec>specs;
  for(int axis=0;axis<3;++axis){const int width=old.hi[axis]-old.lo[axis];if(width<2)continue;
   if(compactFallback&&width<std::max({old.hi[0]-old.lo[0],old.hi[1]-old.lo[1],old.hi[2]-old.lo[2]}))continue;
   for(int part=compactFallback?2:1;part<=(compactFallback?2:3);++part){const int plane=old.lo[axis]+width*part/4;
    if(plane<=old.lo[axis]||plane>=old.hi[axis])continue;
    specs.push_back({false,axis,plane,V{},0.});}
  }
  // Follow the fitted anatomy axes as well as the grid axes. A diagonal cut
  // or tapered pelvis otherwise produces many thin axis-aligned slivers.
  for(unsigned axis=0;!compactFallback&&axis<3;++axis){const auto normal=old.fitted.axes[axis];
   if(std::max({std::abs(normal[0]),std::abs(normal[1]),std::abs(normal[2])})>.9999)continue;
   for(double fraction:{-.5,0.,.5})specs.push_back({true,int(axis),0,normal,dot(old.fitted.center_cells,normal)+fraction*old.fitted.half_cells[axis]});
  }
  // A scored candidate keeps only its sides' shells (bounds, summary and fit,
  // no runs); each worker splits into its own reusable scratch pair. The winner
  // is split again below with the same deterministic split, so its runs are
  // identical and the stored summaries and fits are exactly theirs.
  std::vector<std::pair<detail::Region,detail::Region>>shells(specs.size());std::vector<std::array<std::size_t,2>>runCounts(specs.size());
  std::vector<double>scores(specs.size());std::vector<char>valid(specs.size(),0);
  auto evaluateSpec=[&](std::size_t i,std::pair<detail::Region,detail::Region>&candidate){detail::cancelPoint();const auto&sp=specs[i];
   for(auto*r:{&candidate.first,&candidate.second})if(r->runs.capacity()<old.runs.size())r->runs.reserve(old.runs.size());
   if(sp.oriented)detail::splitOrientedInto(old,sp.normal,sp.at,boneAxes,candidate.first,candidate.second);
   else detail::splitInto(old,sp.axis,sp.plane,boneAxes,candidate.first,candidate.second);
   if(!candidate.first.cells||!candidate.second.cells)return;
   // Optimize the actual oriented collision proxies and their error limits.
   // Axis-aligned empty volume alone kept splitting already thin patches,
   // exhausting the box budget without reducing the worst exterior gap.
   if(sp.oriented)require(candidate.first.cells+candidate.second.cells==old.cells,"collider_oriented_partition_conservation");
   scores[i]=cost(candidate);valid[i]=1;
   shells[i]={detail::shell(candidate.first),detail::shell(candidate.second)};
   runCounts[i]={candidate.first.runs.size(),candidate.second.runs.size()};};
  std::pair<detail::Region,detail::Region>direct;bool directUsed=false;
  const bool fan=detail::mayFanOut();
  const detail::CrewMember crewOn=fan?detail::CrewMember{}:detail::crewMember(); // speedfix
#ifdef GFY_RX_PROF
  RXP(specMs);
#endif
  if(pruneSpecs&&specs.size()>=2){
   // speedfix: the candidates by branch and bound - the same winner and the same adopted sides as scoring every candidate.
   // (1) every candidate is split and summarised (runs, bounds, moments, principal pass; side by side, no corner query);
   // (2) a lower bound of its score from the three boxes fitSummary may choose for each side (identity, bone, principal):
   //     a side's violation is the error of the box it keeps, at least min over the three of max(error without the gap,
   //     gap term of the corners queried so far), and its proxy at least their smallest proxy; the score is monotone in
   //     both; (3) the lowest bound is scored exactly (fitSummary itself), then every other candidate is queried corner
   //     by corner while its bound stays at or under the best exact score, and scored exactly only if it never rises
   //     above it. A candidate dropped is strictly worse than a scored one, so the first strictly best index among the
   //     scored ones is the former winner; its sides are the fitSummary results of the former code. GFY_SF_PRUNE=0: off.
   const auto n=specs.size();const std::array<V,3>identity{{V{1,0,0},V{0,1,0},V{0,0,1}}};
   static const bool fastSplit=[]{const char*e=std::getenv("GFY_SF_FAST_SPLIT");return !e||std::atoi(e)!=0;}();
   std::vector<std::array<detail::SideSummary,2>> sides(n);std::vector<std::exception_ptr>failed(n);std::vector<char>split(n,0);
   auto forEach=[&](std::size_t count,auto&&item){
    if(crewOn.crew&&count>=2&&old.runs.size()>=detail::crewMinRuns()){crewOn.crew->run(crewOn.slot,count,item,&supportIndex);return;}
    if(fan&&count>=2&&old.runs.size()>=4096){std::atomic<std::size_t>next{0};const auto workers=unsigned(std::min<std::size_t>(detail::max_threads,count));
     gfy_live::sep::parallel(workers,1,workers,[&](std::size_t,std::size_t,std::size_t){P3_AUDIT;const detail::SerialScope serial;const detail::ActiveSupport guard(&supportIndex);
      for(std::size_t k;(k=next.fetch_add(1))<count;)item(k);});return;}
    for(std::size_t k=0;k<count;++k)item(k);};
   auto stage1=[&](std::size_t k){const auto i=n-1-k; // heaviest kind (oriented) first
    try{detail::cancelPoint();const auto&sp=specs[i];auto&c=detail::crewScratch();
     if(fastSplit){auto&buf=detail::splitBuffers();std::size_t na=0,nb=0;
      detail::splitRuns(old,sp.oriented,sp.axis,sp.plane,sp.normal,sp.at,buf,na,nb);
      if(!detail::summarizeRuns(buf.a.data(),na,boneAxes,sides[i][0])||!detail::summarizeRuns(buf.b.data(),nb,boneAxes,sides[i][1]))return;
      if(sp.oriented)require(sides[i][0].cells+sides[i][1].cells==old.cells,"collider_oriented_partition_conservation");
      split[i]=1;return;}
     for(auto*r:{&c.first,&c.second})if(r->runs.capacity()<old.runs.size())r->runs.reserve(old.runs.size());
     if(sp.oriented)detail::splitOrientedInto(old,sp.normal,sp.at,boneAxes,c.first,c.second);
     else detail::splitInto(old,sp.axis,sp.plane,boneAxes,c.first,c.second);
     if(!c.first.cells||!c.second.cells)return;
     if(sp.oriented)require(c.first.cells+c.second.cells==old.cells,"collider_oriented_partition_conservation");
     detail::summarizeSide(c.first,sides[i][0]);detail::summarizeSide(c.second,sides[i][1]);split[i]=1;}
    catch(...){failed[i]=std::current_exception();}};
   forEach(n,stage1);
   for(auto&f:failed)if(f)std::rethrow_exception(f);
   // lower bounds without corner queries
   struct Probe {detail::FittedBox box;double noGap{},worst{};};
   const double gapScale=std::max(limits.max_corner_gap_m,top.pitch);
   auto probesOf=[&](std::size_t i,Probe(&p)[2][3],double(&proxyLb)[2]){
    for(int s=0;s<2;++s){const auto&d=sides[i][s];
     p[s][0].box=detail::boxShape(identity,V{{double(d.lo[0]),double(d.lo[1]),double(d.lo[2])}},V{{double(d.hi[0]),double(d.hi[1]),double(d.hi[2])}},top.pitch,limits);
     p[s][1].box=detail::boxShape(boneAxes,d.boneLow,d.boneHigh,top.pitch,limits);
     p[s][2].box=detail::boxShape(d.paxes,d.plow,d.phigh,top.pitch,limits);
     proxyLb[s]=std::numeric_limits<double>::infinity();
     for(auto&q:p[s]){q.noGap=std::max({0.,q.box.proxy_cells/d.cells/limits.max_box_ratio-1.,q.box.minimum_size?0.:1.});q.worst=0;
      proxyLb[s]=std::min(proxyLb[s],q.box.proxy_cells);}}};
   auto sideLb=[&](const Probe(&q)[3]){double lb=std::numeric_limits<double>::infinity();
    for(const auto&x:q)lb=std::min(lb,std::max(x.noGap,x.worst*top.pitch/gapScale-1.));return lb;};
   auto specLb=[&](const double(&lb)[2],const double(&proxyLb)[2]){return 100.*std::max(lb[0],lb[1])+10.*(lb[0]+lb[1])+(proxyLb[0]+proxyLb[1])/old.cells;};
   std::vector<double>bound(n,std::numeric_limits<double>::infinity());std::vector<std::size_t>order;
   for(std::size_t i=0;i<n;++i)if(split[i]){Probe p[2][3];double proxyLb[2];probesOf(i,p,proxyLb);const double lb[2]={sideLb(p[0]),sideLb(p[1])};
    bound[i]=specLb(lb,proxyLb);order.push_back(i);}
   std::stable_sort(order.begin(),order.end(),[&](std::size_t a,std::size_t b){return bound[a]<bound[b];});
   // exact scores (fitSummary of both sides, then cost() of the former code on them)
   std::vector<std::array<detail::FittedBox,2>> fits(n);std::atomic<double> bestExact{std::numeric_limits<double>::infinity()};std::mutex bestMutex;
   auto exact=[&](std::size_t i){
    for(int s=0;s<2;++s)fits[i][s]=detail::fitSide(sides[i][s],top.pitch,limits,boneAxes,top,capture.component);
    double v[2];for(int s=0;s<2;++s){const auto&f=fits[i][s];const double ratio=f.proxy_cells/sides[i][s].cells;
     const double gap=f.gap_cells*top.pitch/std::max(limits.max_corner_gap_m,top.pitch);
     v[s]=std::max({0.,ratio/limits.max_box_ratio-1.,gap-1.,f.minimum_size?0.:1.});}
    scores[i]=100.*std::max(v[0],v[1])+10.*(v[0]+v[1])+(fits[i][0].proxy_cells+fits[i][1].proxy_cells)/old.cells;valid[i]=1;
    {std::lock_guard<std::mutex> lock(bestMutex);if(scores[i]<bestExact.load(std::memory_order_relaxed))bestExact.store(scores[i],std::memory_order_relaxed);}};
   // true: candidate i is proven to score above the best exact score
   auto loses=[&](std::size_t i){if(bound[i]>bestExact.load(std::memory_order_relaxed))return true;
    Probe p[2][3];double proxyLb[2];probesOf(i,p,proxyLb);double lb[2]={sideLb(p[0]),sideLb(p[1])};int next[2][3]={{0,0,0},{0,0,0}};
    // Corners are queried where they can raise the bound: in the side whose bound is higher (one bad side decides the
    // score), on the box that holds that side's bound down (its smallest). A side whose smallest bound belongs to a box
    // with every corner queried cannot rise any more.
    auto pick=[&](int s){double low=std::numeric_limits<double>::infinity();for(const auto&x:p[s])low=std::min(low,std::max(x.noGap,x.worst*top.pitch/gapScale-1.));
     int c=-1;for(int j=0;j<3;++j)if(std::max(p[s][j].noGap,p[s][j].worst*top.pitch/gapScale-1.)==low){if(next[s][j]>=8)return -1;if(c<0)c=j;}
     return c;};
    while(true){const int c0=pick(0),c1=pick(1);if(c0<0&&c1<0)return false;
     const int s=(c0>=0&&(c1<0||lb[0]>=lb[1]))?0:1;const int c=s?c1:c0;auto&q=p[s][c];const int k=next[s][c]++;
     const int bx=(k&4)?1:-1,by=(k&2)?1:-1,bz=(k&1)?1:-1;const auto&b=q.box;const auto&ax=b.axes;
     const V corner=add(b.center_cells,add(mul(ax[0],bx*b.half_cells[0]),add(mul(ax[1],by*b.half_cells[1]),mul(ax[2],bz*b.half_cells[2]))));
     q.worst=std::max(q.worst,std::sqrt(detail::supportDistanceSquared(corner,top,capture.component)));
     lb[s]=sideLb(p[s]);if(specLb(lb,proxyLb)>bestExact.load(std::memory_order_relaxed))return true;}};
   if(!order.empty()){exact(order.front());
    std::vector<std::exception_ptr>failed2(n);
    auto stage2=[&](std::size_t k){const auto i=order[k+1];try{detail::cancelPoint();if(!loses(i))exact(i);}catch(...){failed2[i]=std::current_exception();}};
    forEach(order.size()-1,stage2);
    for(auto&f:failed2)if(f)std::rethrow_exception(f);
    for(std::size_t i=0;i<n;++i)if(valid[i]){shells[i]={detail::sideShell(sides[i][0],fits[i][0]),detail::sideShell(sides[i][1],fits[i][1])};
     runCounts[i]={sides[i][0].runs,sides[i][1].runs};}}
  }
  else if(specs.size()>=2&&old.runs.size()>=4096&&fan){
   // Up to four threads take specs dynamically, heaviest kind first (oriented
   // specs are last in the list and split every run in two). Results are stored
   // by index and reduced in index order below; the lowest failing index
   // rethrows, which is the exception the serial loop would have thrown.
   const auto n=specs.size();std::vector<std::exception_ptr>failed(n);std::atomic<std::size_t>next{0};
   const auto workers=unsigned(std::min<std::size_t>(detail::max_threads,n));
   gfy_live::sep::parallel(workers,1,workers,[&](std::size_t,std::size_t,std::size_t chunk){P3_AUDIT;
    const detail::SerialScope serial;const detail::ActiveSupport guard(&supportIndex);
    for(std::size_t k;(k=next.fetch_add(1))<n;){const auto i=n-1-k;try{evaluateSpec(i,scratch[chunk]);}catch(...){failed[i]=std::current_exception();}}});
   for(auto&f:failed)if(f)std::rethrow_exception(f);}
  else if(specs.size()==1&&!specs[0].oriented&&old.runs.size()>=8192&&fan){
   // Compact mode's single axis-aligned candidate: build and fit its two sides on
   // two threads. Both sides are non-empty by construction (lo < plane < hi, and
   // lo/hi are attained by runs: the run at lo[axis] contributes to side 0, the run
   // at hi[axis]-1, resp. ending at hi[2], to side 1), so evaluateSpec would fit
   // both, first then second; a failing side rethrows in that order
   // (sep::parallel rethrows the first failing chunk).
   RXC(childPairs,1);
   const auto&sp=specs[0];auto&candidate=direct;directUsed=true;
   gfy_live::sep::parallel(2,1,2,[&](std::size_t first,std::size_t last,std::size_t){P3_AUDIT;
    const detail::SerialScope serial;const detail::ActiveSupport guard(&supportIndex);
    for(std::size_t side=first;side<last;++side){detail::cancelPoint();auto&r=side?candidate.second:candidate.first;
     detail::splitSide(old,sp.axis,sp.plane,int(side),r,boneAxes);
     require(r.cells>0,"collider_empty_region");
     detail::fit(r,top.pitch,limits,boneAxes,&top,capture.component);}});
   scores[0]=cost(candidate);valid[0]=1;
  }
  else if(crewOn.crew&&specs.size()>=2&&old.runs.size()>=detail::crewMinRuns()){
   // speedfix: the candidates shared with the crew (heaviest first, as the fan-out above); the lowest failing index rethrows.
   const auto n=specs.size();std::vector<std::exception_ptr>failed(n);
   auto item=[&](std::size_t k){const auto i=n-1-k;try{evaluateSpec(i,detail::crewScratch());}catch(...){failed[i]=std::current_exception();}};
   crewOn.crew->run(crewOn.slot,n,item,&supportIndex);
   for(auto&f:failed)if(f)std::rethrow_exception(f);}
  else if(crewOn.crew&&specs.size()==1&&!specs[0].oriented&&old.runs.size()>=detail::crewMinRuns()){
   // speedfix: compact mode's single candidate, its two sides shared with the crew (the fan-out path above, item by item).
   RXC(childPairs,1);
   const auto&sp=specs[0];auto&candidate=direct;directUsed=true;std::exception_ptr failed[2];
   auto item=[&](std::size_t side){try{detail::cancelPoint();auto&r=side?candidate.second:candidate.first;
     detail::splitSide(old,sp.axis,sp.plane,int(side),r,boneAxes);
     require(r.cells>0,"collider_empty_region");
     detail::fit(r,top.pitch,limits,boneAxes,&top,capture.component);}catch(...){failed[side]=std::current_exception();}};
   crewOn.crew->run(crewOn.slot,2,item,&supportIndex);
   for(auto&f:failed)if(f)std::rethrow_exception(f);
   scores[0]=cost(candidate);valid[0]=1;
  }
  else for(std::size_t i=0;i<specs.size();++i)evaluateSpec(i,scratch[0]);
  std::size_t bestIndex=specs.size();
  for(std::size_t i=0;i<specs.size();++i)if(valid[i]&&scores[i]<bestScore){bestScore=scores[i];bestIndex=i;}
  if(bestIndex<specs.size()){
   if(directUsed)best=std::move(direct);
   else{const auto&sp=specs[bestIndex];
    best.first.runs.reserve(runCounts[bestIndex][0]);best.second.runs.reserve(runCounts[bestIndex][1]);
    if(sp.oriented)detail::splitOrientedInto(old,sp.normal,sp.at,boneAxes,best.first,best.second,false);
    else detail::splitInto(old,sp.axis,sp.plane,boneAxes,best.first,best.second,false);
    detail::adopt(best.first,shells[bestIndex].first,top.pitch,limits,boneAxes,top,capture.component);
    detail::adopt(best.second,shells[bestIndex].second,top.pitch,limits,boneAxes,top,capture.component);}}
  if(!std::isfinite(bestScore)&&old.cells<=32&&
     double(omittedCells+old.cells)/originalCells<=limits.max_omitted_cell_fraction){
    omitted[chosen]=true;omittedCells+=old.cells;live.erase(std::find(live.begin(),live.end(),chosen));RXC(omissions,1);continue;}
  if(!std::isfinite(bestScore)&&!compactFallback){auto&tr=detail::fitTrace();tr.greedyMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tr.start).count();
   tr.greedyLeaves=live.size();tr.restarts+=100;{Prepared r;if(speculation.take(r)){tr.speculated=1;return r;}}return prepare(top,capture,limits,true);}
  if(!std::isfinite(bestScore))throw std::runtime_error("collider_unsplittable_region cells="+
      std::to_string(old.cells)+" dims="+
      std::to_string(old.hi[0]-old.lo[0])+","+
      std::to_string(old.hi[1]-old.lo[1])+","+
      std::to_string(old.hi[2]-old.lo[2])+" proxy="+
      std::to_string(old.fitted.proxy_cells/old.cells)+" gap="+
      std::to_string(old.fitted.gap_cells*top.pitch));
  leaves[chosen]=std::move(best.first);leaves.push_back(std::move(best.second));
  omitted.push_back(false);live.push_back(leaves.size()-1);
 }
 }

 // Refit unions across subdivision boundaries. The temporary decomposition
 // need not be a minimal compound: neighboring leaves may share one OBB.
 if(compactFallback){
#ifdef GFY_RX_PROF
 RXP(mergeMs);
#endif
 struct Pair {double cost;std::size_t a,b,fit;bool operator<(const Pair&o)const{return cost>o.cost;}};
 std::priority_queue<Pair>pairs;std::vector<bool>alive(leaves.size(),true);
 std::size_t active=live.size();
 double currentProxy=0;for(const auto i:live)currentProxy+=leaves[i].fitted.proxy_cells;
 // Fitted union of each pushed pair (Pair::fit): a pure function of the two
 // unchanged leaves, reused when that pair is committed instead of refitting it.
 std::vector<detail::FittedBox>merges;
 std::vector<std::pair<std::size_t,std::size_t>>pending;
 auto candidate=[&](std::size_t a,std::size_t b){
  if(omitted[a]||omitted[b])return;
  const int reach=int(std::ceil(limits.max_corner_gap_m/top.pitch));
  for(unsigned axis=0;axis<3;++axis)
   if(leaves[a].lo[axis]>leaves[b].hi[axis]+reach||leaves[b].lo[axis]>leaves[a].hi[axis]+reach)return;
  pending.push_back({a,b});
 };
 // Merged fits of the queued pairs are pure functions of the (unchanged)
 // leaves: evaluate them on up to four threads (dynamic order, results by
 // index; the lowest failing index rethrows, as the serial loop would), then
 // push in queue order so the heap sees exactly the serial sequence.
 auto flush=[&](){RXP(flushMs);
  const auto n=pending.size();RXC(flushes,1);RXC(flushItems,n);
  std::vector<double>costs(n);std::vector<char>ok(n,0);std::vector<detail::FittedBox>fits(n);
  static const bool refuseUnions=[]{const char*e=std::getenv("GFY_SF_UNION_REFUSE");return !e||std::atoi(e)!=0;}(); // speedfix
  auto one=[&](std::size_t k){detail::cancelPoint();const auto a=pending[k].first,b=pending[k].second;
   if(refuseUnions&&detail::unionRefused(leaves[a],leaves[b],top.pitch,limits,boneAxes,top,capture.component))return; // ok[k] stays 0
   fits[k]=detail::fitUnion(leaves[a],leaves[b],top.pitch,limits,boneAxes,top,capture.component);
   if(detail::acceptableCells(fits[k],leaves[a].cells+leaves[b].cells,top.pitch,limits)){ok[k]=1;costs[k]=fits[k].proxy_cells-leaves[a].fitted.proxy_cells-leaves[b].fitted.proxy_cells;}};
  std::size_t work=0;for(const auto&p:pending)work+=leaves[p.first].runs.size()+leaves[p.second].runs.size();
  if(n>=2&&work>=16384&&detail::mayFanOut()){RXC(parallelFlushes,1);
   std::vector<std::exception_ptr>failed(n);std::atomic<std::size_t>next{0};
   const auto workers=unsigned(std::min<std::size_t>(detail::max_threads,n));
   gfy_live::sep::parallel(workers,1,workers,[&](std::size_t,std::size_t,std::size_t){P3_AUDIT;
    const detail::SerialScope serial;const detail::ActiveSupport guard(&supportIndex);
    for(std::size_t k;(k=next.fetch_add(1))<n;){try{one(k);}catch(...){failed[k]=std::current_exception();}}});
   for(auto&f:failed)if(f)std::rethrow_exception(f);}
  else if(n>=2&&work>=detail::crewMinFlush()&&!detail::mayFanOut()&&detail::crewMember().crew){
   // speedfix: the union fits shared with the crew; the lowest failing index rethrows, as the loop below would.
   const auto member=detail::crewMember();std::vector<std::exception_ptr>failed(n);
   auto item=[&](std::size_t k){try{one(k);}catch(...){failed[k]=std::current_exception();}};
   member.crew->run(member.slot,n,item,&supportIndex);
   for(auto&f:failed)if(f)std::rethrow_exception(f);}
  else for(std::size_t k=0;k<n;++k)one(k);
  for(std::size_t k=0;k<n;++k)if(ok[k]){pairs.push({costs[k],pending[k].first,pending[k].second,merges.size()});merges.push_back(fits[k]);}
  pending.clear();};
 for(std::size_t x=0;x<live.size();++x)for(std::size_t y=x+1;y<live.size();++y)candidate(live[x],live[y]);
 flush();
 while(active>std::min<std::size_t>(64,limits.max_boxes)&&!pairs.empty()){detail::cancelPoint();
  const auto p=pairs.top();pairs.pop();if(!alive[p.a]||!alive[p.b])continue;
  if((currentProxy+p.cost)/(originalCells-omittedCells)>limits.max_proxy_ratio)continue;
  auto merged=detail::commitUnion(leaves[p.a],leaves[p.b],merges[p.fit]);alive[p.a]=alive[p.b]=false;
  // Dead leaves are never read again (pairs naming them are skipped above).
  std::vector<detail::Run>().swap(leaves[p.a].runs);std::vector<detail::Run>().swap(leaves[p.b].runs);
  const auto id=leaves.size();leaves.push_back(std::move(merged));alive.push_back(true);omitted.push_back(false);--active;currentProxy+=p.cost;
  for(const auto i:live)if(alive[i])candidate(i,id);
  live.push_back(id);
  flush();
 }
 std::vector<detail::Region>compacted;std::vector<bool>keptOmissions;
 for(std::size_t i=0;i<leaves.size();++i)if(alive[i]){compacted.push_back(std::move(leaves[i]));keptOmissions.push_back(omitted[i]);}
 leaves=std::move(compacted);omitted=std::move(keptOmissions);
 }

 // Every surviving cell belongs to exactly one leaf; mass is from those
 // cells, never from the (necessarily approximate) collision proxy volume.
#ifdef GFY_RX_PROF
 const auto p3Final=std::chrono::steady_clock::now();
#endif
 detail::cancelPoint();
 std::vector<detail::Run>all;all.reserve(top.spans.size());
 std::uint64_t totalCells=0;double proxyCells=0,maxGap=0;
 for(std::size_t i=0;i<leaves.size();++i){const auto&r=leaves[i];totalCells+=r.cells;
  if(!omitted[i]){proxyCells+=r.fitted.proxy_cells;
   maxGap=std::max(maxGap,r.fitted.gap_cells*top.pitch);}
  all.insert(all.end(),r.runs.begin(),r.runs.end());}
 require(totalCells==top.components[capture.component].cells,"collider_partition_conservation");
#ifdef GFY_RX_PROF
 {auto&p=detail::prof();std::cerr<<"rxprof collider leaves="<<leaves.size()<<" compact="<<compactFallback<<" evals="<<p.evals<<" evalRuns="<<p.evalRuns<<" eval_ms(incl support)="<<p.evalMs<<" supports="<<p.supports<<" support_ms="<<p.supportMs<<" split_ms="<<p.splitMs<<" combine_ms(incl fit)="<<p.combineMs<<" axes_ms="<<p.axesMs<<" blocks="<<p.blocks<<" cols="<<p.cols<<std::endl;}
#endif
 auto moment=detail::moments(all,top);require(std::uint64_t(moment.n)==totalCells,"collider_moment_cells");
 const long double cube=top.pitch*top.pitch*top.pitch;
 const long double cellMass=capture.density_kg_m3*cube,mass=cellMass*moment.n;
 V restCOM{};for(int a=0;a<3;++a)restCOM[a]=double(moment.s[a]/moment.n);
 const long double vx=moment.q[0]-moment.s[0]*moment.s[0]/moment.n+moment.n*top.pitch*top.pitch/12;
 const long double vy=moment.q[1]-moment.s[1]*moment.s[1]/moment.n+moment.n*top.pitch*top.pitch/12;
 const long double vz=moment.q[2]-moment.s[2]*moment.s[2]/moment.n+moment.n*top.pitch*top.pitch/12;
 const long double cxy=moment.xy-moment.s[0]*moment.s[1]/moment.n;
 const long double cxz=moment.xz-moment.s[0]*moment.s[2]/moment.n;
 const long double cyz=moment.yz-moment.s[1]*moment.s[2]/moment.n;
 const long double ir[3][3]={{cellMass*(vy+vz),-cellMass*cxy,-cellMass*cxz},
                            {-cellMass*cxy,cellMass*(vx+vz),-cellMass*cyz},
                            {-cellMass*cxz,-cellMass*cyz,cellMass*(vx+vy)}};
 Prepared out;out.accepted=capture.accepted;out.component=capture.component;
 out.anchor_bone=capture.anchor_bone;out.owned_cells=totalCells;
 out.anatomical_volume_m3=double(cube*moment.n);out.mass_kg=double(mass);
 out.com_anchor=transform(capture.rest_to_anchor,restCOM);
 // Inertia transforms as R I R^T. Translation is already removed at COM.
 const auto&rot=capture.rest_to_anchor.row_major;
 for(int i=0;i<3;++i)for(int j=0;j<3;++j){long double value=0;
  for(int k=0;k<3;++k)for(int l=0;l<3;++l)value+=rot[i*4+k]*ir[k][l]*rot[j*4+l];
  out.inertia_anchor_com[i*3+j]=double(value);}
 const auto&I=out.inertia_anchor_com;
 const double det=I[0]*(I[4]*I[8]-I[5]*I[7])-I[1]*(I[3]*I[8]-I[5]*I[6])+I[2]*(I[3]*I[7]-I[4]*I[6]);
 require(out.mass_kg>0&&std::isfinite(out.mass_kg)&&I[0]>0&&I[0]*I[4]-I[1]*I[3]>0&&det>0,
         "collider_inertia_not_positive_definite");
 const V worldCOM=transform(capture.anchor_to_world,out.com_anchor);
 out.angular_world=capture.parent_angular_world;
 out.linear_world=add(capture.parent_linear_world,
                      cross(capture.parent_angular_world,sub(worldCOM,capture.parent_com_world)));
 for(std::size_t leaf=0;leaf<leaves.size();++leaf){if(omitted[leaf])continue;
  const auto&r=leaves[leaf];Box box;
  V restCenter{},half{};for(int a=0;a<3;++a){restCenter[a]=top.origin[a]+r.fitted.center_cells[a]*top.pitch;
   half[a]=r.fitted.half_cells[a]*top.pitch;
   require(half[a]+1e-12>=limits.minimum_half_extent_m,"collider_engine_box_clamp");}
  box.center_anchor=transform(capture.rest_to_anchor,restCenter);
  box.half_extents_m=half;
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)
   box.orientation[i*3+j]=rotate(capture.rest_to_anchor,r.fitted.axes[j])[i];
  out.maximum_box_extent_m=std::max({out.maximum_box_extent_m,half[0]*2,half[1]*2,half[2]*2});
  for(int dx:{-1,1})for(int dy:{-1,1})for(int dz:{-1,1}){
   V offset{};for(int a=0;a<3;++a)offset=add(offset,mul(r.fitted.axes[a],
                                                  (a==0?dx:(a==1?dy:dz))*half[a]));
   offset=rotate(capture.rest_to_anchor,offset);
   out.maximum_radius_m=std::max(out.maximum_radius_m,norm(sub(add(box.center_anchor,offset),out.com_anchor)));}
  out.boxes.push_back(box);}
 out.omitted_collision_cells=omittedCells;
 {
#ifdef GFY_RX_PROF
 RXP(omitMs);
#endif
 // maximum over omitted cells of the distance to the nearest box, computed per
 // cell exactly as before. A cell whose distance to one box (the previous cell's
 // nearest) is already <= the running maximum cannot raise it, because its
 // nearest distance is the minimum over all boxes including that one; such cells
 // skip the remaining boxes. The maximum is exact and unchanged.
 auto boxDistance=[&](const V&point,const Box&box){const V delta=sub(point,box.center_anchor);double squared=0;
  for(int axis=0;axis<3;++axis){V direction{{box.orientation[axis],
      box.orientation[3+axis],box.orientation[6+axis]}};
   const double gap=std::max(0.,std::abs(dot(delta,direction))-box.half_extents_m[axis]);
   squared+=gap*gap;}
  return std::sqrt(squared);};
 std::size_t hint=0;
 for(std::size_t leaf=0;leaf<leaves.size();++leaf)if(omitted[leaf]){
  for(const auto&r:leaves[leaf].runs)for(int z=r.lo;z<r.hi;++z){
   const V point=transform(capture.rest_to_anchor,{
      top.origin[0]+(r.x+.5)*top.pitch,
      top.origin[1]+(r.y+.5)*top.pitch,
      top.origin[2]+(z+.5)*top.pitch});
   RXC(omitPoints,1);
   if(hint<out.boxes.size()&&boxDistance(point,out.boxes[hint])<=out.maximum_omitted_distance_m)continue;
   RXC(omitFull,1);
   double nearest=std::numeric_limits<double>::infinity();
   for(std::size_t b=0;b<out.boxes.size();++b){const double d=boxDistance(point,out.boxes[b]);
    if(d<nearest)hint=b;
    nearest=std::min(nearest,d);}
   out.maximum_omitted_distance_m=std::max(out.maximum_omitted_distance_m,nearest);}}
 }
 require(double(omittedCells)/totalCells<=limits.max_omitted_cell_fraction+1e-12&&
         out.maximum_omitted_distance_m<=limits.max_omitted_distance_m+1e-12,
         "collider_omitted_anatomy_too_far");
 out.proxy_to_anatomy_volume_ratio=double(proxyCells)/(totalCells-omittedCells);
 out.maximum_corner_gap_m=maxGap;
 require(out.boxes.size()<=limits.max_boxes&&out.proxy_to_anatomy_volume_ratio<=limits.max_proxy_ratio+1e-12&&
         out.maximum_corner_gap_m<=limits.max_corner_gap_m+1e-12,"collider_fit_certificate");
#ifdef GFY_RX_PROF
 {auto&p=detail::prof();const auto now=std::chrono::steady_clock::now();
  std::cerr<<"p3prof total_ms="<<std::chrono::duration<double,std::milli>(now-p3Start).count()
   <<" iterations="<<p.iterations<<" omissions="<<p.omissions<<" scan_ms="<<p.scanMs<<" spec_ms="<<p.specMs<<" skipped_boxes="<<p.skippedBoxes<<" stopped_boxes="<<p.stoppedBoxes<<" summary_ms="<<p.summaryMs<<" summary_runs="<<p.summaryRuns<<" subdiv_ms="<<p.subdivMs<<" merge_ms="<<p.mergeMs<<" flush_ms="<<p.flushMs<<" commit_ms="<<p.commitMs
   <<" final_ms="<<std::chrono::duration<double,std::milli>(now-p3Final).count()<<" omit_ms="<<p.omitMs
   <<" flushes="<<p.flushes<<" flush_items="<<p.flushItems<<" parallel_flushes="<<p.parallelFlushes
   <<" commits="<<p.commits<<" commit_runs="<<p.commitRuns<<" moment_folds="<<p.momentFolds<<" moment_fold_runs="<<p.momentFoldRuns
   <<" child_pairs="<<p.childPairs<<" chunked_passes="<<p.chunkedPasses<<" omit_points="<<p.omitPoints<<" omit_full="<<p.omitFull<<std::endl;}
#endif
 return out;
}
}
