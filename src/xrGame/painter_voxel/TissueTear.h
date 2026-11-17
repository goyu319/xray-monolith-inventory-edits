#pragma once
#include <atomic>
int mask_only_mode_value() noexcept;
namespace tissue_tear {
// R64: the tear's parallel sections. Up to 4x the pool width of contiguous chunks, taken in turn by the caller and the pool
// threads as they free up (a limb's cross-section is uneven: the slabs through its middle hold far more tissue than those at its
// edges, and one chunk per thread left most threads idle behind the slowest). The thread budget is core::parallelChunks's (the
// caller plus parallelWidth()-1 pool threads); a section started on a pool thread runs inline; the first failing chunk in chunk
// order rethrows. Every section's result is independent of where its chunks begin and end, as with core::parallelChunks.
inline std::size_t ttWidth(){return 4*core::parallelWidth();}
template<class F>void ttParallel(std::size_t n,std::size_t grain,F&&body){
 const std::size_t chunks=n>=2*grain?std::min<std::size_t>(ttWidth(),n/grain):1;
 if(chunks<=1||core::painterInlineParallel){body(std::size_t(0),n,std::size_t(0));return;}
 using Body=std::remove_reference_t<F>;
 // shared with the pool tasks: a task that starts after every chunk was taken returns without touching the caller's frame
 // (target: the finished count the caller waits for - every chunk, or after a failed submit the chunks taken before the close)
 struct Shared {std::atomic<std::size_t> next{0};std::size_t n=0,chunks=0,finished=0,target=0;Body*body=nullptr;std::mutex m;std::condition_variable cv;std::vector<std::exception_ptr> failure;
  bool runOne(){const std::size_t c=next.fetch_add(1,std::memory_order_relaxed);if(c>=chunks)return false;
   try{(*body)(n*c/chunks,n*(c+1)/chunks,c);}catch(...){failure[c]=std::current_exception();}
   {std::lock_guard<std::mutex> l(m);if(++finished==target)cv.notify_all();}return true;}};
 const auto sh=std::make_shared<Shared>();sh->n=n;sh->chunks=chunks;sh->target=chunks;sh->body=&body;sh->failure.resize(chunks);
 const std::size_t helpers=std::min(chunks-1,core::parallelWidth()-1);auto&pool=core::workerPool();
 // R68: a submit that throws: the cursor is closed (no chunk starts any more), the chunks taken before the close finish, then the
 // submit's failure travels on
 try{for(std::size_t h=0;h<helpers;++h)pool.submit([sh]{while(sh->runOne()){}});}
 catch(...){const std::size_t taken=std::min(sh->next.exchange(chunks),chunks);
  {std::unique_lock<std::mutex> l(sh->m);sh->target=taken;sh->cv.wait(l,[&]{return sh->finished==sh->target;});}throw;}
 while(sh->runOne()){}
 {std::unique_lock<std::mutex> l(sh->m);sh->cv.wait(l,[&]{return sh->finished==sh->chunks;});}
 for(auto&f:sh->failure)if(f)std::rethrow_exception(f);
}
// Scratch buffers that are written in full before they are read: no zero fill on allocation (pages first touched by the writers)
template<class T> struct NoInit:std::allocator<T>{NoInit()=default;template<class U>NoInit(const NoInit<U>&)noexcept{}
 template<class U>struct rebind{using other=NoInit<U>;};
 template<class U>void construct(U*p)noexcept{::new(static_cast<void*>(p))U;}
 template<class U,class...A>void construct(U*p,A&&...a){::new(static_cast<void*>(p))U(std::forward<A>(a)...);}};
template<class T> using Raw=std::vector<T,NoInit<T>>;
template<class V,class T> inline void parallelFill(V&v,T value){ttParallel(v.size(),std::size_t(1)<<16,[&](std::size_t a,std::size_t b,std::size_t){std::fill(v.begin()+std::ptrdiff_t(a),v.begin()+std::ptrdiff_t(b),value);});}
enum:std::uint8_t{AIR=0,KEPT=1,CUT=2,TORN=3,LOOSE=4,STATE=7,REGION=8,FILL=16,CYL=32,MARK=64,RIM=128};
enum:std::uint8_t{FOREIGN=5,OLDCUT=6}; // T1 states on a piece's grid: another piece's tissue; a removed cell off the damage band
enum:std::uint8_t{PLUG=7}; // T2: a removed bone/marrow cell the bone's tear does not reach (limb profile): solid for the torn surface only (never erased or kept)
constexpr std::uint8_t EDGE=128; // lab bit: the cell lies on the cylinder boundary (tissue beyond it is the body)
constexpr std::uint8_t FAR=64; // lab bit: farther from the damage than any tear reaches (not read: kept, outside the limb region)
constexpr double maxDepth=.046,meshMargin=.008,sourceSlab=.025;
constexpr double limbDepth=.072; // T2: the limb profile's deepest tear (muscle groups 24-46 mm + bundles + the section's extra; the head profile keeps maxDepth) // only the damage band at the tear plane shapes the ends (not every crater of a grenade-hit section)
constexpr float INF=65535.f; // saturated squared distance (cells)
inline std::uint32_t h32(std::uint32_t x){x^=x>>16;x*=0x7feb352du;x^=x>>15;x*=0x846ca68bu;x^=x>>16;return x;}
inline double u01(std::uint32_t s,int a,int b,unsigned salt){return double(h32(s^h32(std::uint32_t(a)*0x9e3779b1u+h32(std::uint32_t(b)+salt))))*(1./4294967296.);}
inline double vnoise(std::uint32_t s,unsigned salt,double x,double y){const double x0=std::floor(x),y0=std::floor(y);const int xi=int(x0),yi=int(y0);
 const double fx=x-x0,fy=y-y0,sx=fx*fx*(3-2*fx),sy=fy*fy*(3-2*fy);auto v=[&](int a,int b){return 2*u01(s,a,b,salt)-1;};
 const double a=v(xi,yi)+(v(xi+1,yi)-v(xi,yi))*sx,b=v(xi,yi+1)+(v(xi+1,yi+1)-v(xi,yi+1))*sx;return a+(b-a)*sy;}
// One value in [0,1) per jittered cell of size `cell` (m) of the cross-section (Voronoi bundles).
inline double bundle(std::uint32_t s,unsigned salt,double x,double y,double cell){const double gx=x/cell,gy=y/cell;const int ix=int(std::floor(gx)),iy=int(std::floor(gy));
 double best=1e9;int bx=0,by=0;for(int a=-1;a<=1;++a)for(int b=-1;b<=1;++b){const int cx=ix+a,cy=iy+b;const double fx=cx+u01(s,cx,cy,salt),fy=cy+u01(s,cx,cy,salt+1);
  const double d=(gx-fx)*(gx-fx)+(gy-fy)*(gy-fy);if(d<best){best=d;bx=cx;by=cy;}}return u01(s,bx,by,salt+2);}
// R62 display values (baked render field): 2 cortex / 21 marrow, 5 generic muscle, 27-34 muscle variants (one per muscle, touching
// muscles never share one), 17 fascia, 18-20 vessel wall / arterial / venous blood, 22-24 airway / duct / valve, 25-26 brain.
enum Class:unsigned{bone,muscle,shell,small,organ,other,vessel,fascia,marrow,mv0,classes=mv0+8}; // T1: marrow (21) apart from the cortex
inline unsigned classOf(unsigned v,bool filler){
 if(v>=27&&v<=34)return mv0+(v-27); // also in the clothing gap: the muscle runs out to the fat layer and tears at one length
 if(v==17)return fascia;
 if(filler)return shell;
 switch(v){case 2:return bone;case 21:return marrow;case 5:return muscle;case 1:case 15:case 16:return shell;case 3:case 4:case 14:case 25:case 26:return small;
  case 18:case 19:case 20:return vessel;case 22:case 23:case 24:return organ;default:return v>=6&&v<=13?organ:other;}}
// The material slot of a display value (removed-tissue counts for the effects keep the 1-16 slot meanings)
inline unsigned baseSlot(unsigned v){if(v<17)return v;if(v>=27&&v<=34)return 5;
 switch(v){case 17:return 1;case 18:case 19:case 20:return 5;case 21:return 2;case 22:return 9;case 23:return 10;case 24:return 8;case 25:case 26:return 3;default:return 5;}}
// The shot: the torn ends lean across the limb toward its exit side (random amount and sign per sever), exit-side strands come
// out longer, and the push along the limb favours one end. Without a direction the lean axis is random.
// head (R62 HeadFracture.h): a burst skull's own profile - 1 the cap's fracture: the brain recedes 1-3 cm in lumps, the skull
// splinters 2-8 mm, the scalp pulls back ~1 cm; 2 the radial fractures between pieces: the brain 4-12 mm, the bone 2-5 mm, the
// scalp 5-9 mm, so the pieces stay whole (the limb profile would leave the brain face flat under the rips, or eat the pieces)
struct TearImpact {double a=0,b=0,along=0,tilt=0,Rl=.05;int head=0;double lim=maxDepth,mbase[2]={0,0};}; // T2: lim = the tear's depth limit; mbase = each end's mean muscle retraction (the shell pulls back past it)
// T1: a depth eased into its floor `lo` and the tear's limit (no plateau where a hard clamp would cut: a plateau is a flat shelf)
inline double ease(double d,double lo,double lim=maxDepth){const double hi=lim-.0015,kl=.003,kh=.006;
 if(d<lo+kl)d=lo+kl*std::exp((d-lo-kl)/kl);
 if(d>hi-kh)d=hi-kh*std::exp(-(d-hi+kh)/kh);
 return std::clamp(d,.002,lim-.001);}
// T1: one fascicle strand per jittered cell of size `cell` (m): its value in [0,1) and the distance (cells, <= 1) from its axis,
// so a torn strand tapers to a tip instead of ending in a flat top
inline double strand(std::uint32_t s,unsigned salt,double x,double y,double cell,double&dist){const double gx=x/cell,gy=y/cell;const int ix=int(std::floor(gx)),iy=int(std::floor(gy));
 double best=1e9;int bx=0,by=0;for(int a=-1;a<=1;++a)for(int b=-1;b<=1;++b){const int cx=ix+a,cy=iy+b;const double fx=cx+u01(s,cx,cy,salt),fy=cy+u01(s,cx,cy,salt+1);
  const double d=(gx-fx)*(gx-fx)+(gy-fy)*(gy-fy);if(d<best){best=d;bx=cx;by=cy;}}dist=std::min(1.,std::sqrt(best));return u01(s,bx,by,salt+2);}
// T1 (speed): a map row walks a in 1 mm steps, and a lattice's values change only every few steps: a row keeps the corners of its
// value-noise cell and the 3x3 jittered cells around its point until the point leaves that cell - the same numbers as the direct
// evaluation (GFY_TT_VERIFY compares every map point), a fraction of the hashing
struct NoiseRow {int xi=INT_MIN,yi=INT_MIN;double c00=0,c10=0,c01=0,c11=0;
 double at(std::uint32_t s,unsigned salt,double x,double y){const double x0=std::floor(x),y0=std::floor(y);const int X=int(x0),Y=int(y0);
  const double fx=x-x0,fy=y-y0,sx=fx*fx*(3-2*fx),sy=fy*fy*(3-2*fy);
  if(X!=xi||Y!=yi){xi=X;yi=Y;c00=2*u01(s,X,Y,salt)-1;c10=2*u01(s,X+1,Y,salt)-1;c01=2*u01(s,X,Y+1,salt)-1;c11=2*u01(s,X+1,Y+1,salt)-1;}
  const double a=c00+(c10-c00)*sx,b=c01+(c11-c01)*sx;return a+(b-a)*sy;}};
struct CellRow {int ix=INT_MIN,iy=INT_MIN;double fx[9]{},fy[9]{},v[9][3]{};
 int nearest(std::uint32_t s,unsigned salt,int nv,double gx,double gy,double&best){const int X=int(std::floor(gx)),Y=int(std::floor(gy));
  if(X!=ix||Y!=iy){ix=X;iy=Y;int k=0;for(int a=-1;a<=1;++a)for(int b=-1;b<=1;++b,++k){const int cx=X+a,cy=Y+b;fx[k]=cx+u01(s,cx,cy,salt);fy[k]=cy+u01(s,cx,cy,salt+1);
    for(int q=0;q<nv;++q)v[k][q]=u01(s,cx,cy,salt+2+unsigned(q));}}
  best=1e9;int bk=0;for(int k=0;k<9;++k){const double d=(gx-fx[k])*(gx-fx[k])+(gy-fy[k])*(gy-fy[k]);if(d<best){best=d;bk=k;}}return bk;}
 // T2: also the squared distance to the second nearest cell point (a bundle's border lies where the two are equal)
 int nearest2(std::uint32_t s,unsigned salt,int nv,double gx,double gy,double&best,double&second){const int bk=nearest(s,salt,nv,gx,gy,best);
  second=1e9;for(int k=0;k<9;++k){if(k==bk)continue;const double d=(gx-fx[k])*(gx-fx[k])+(gy-fy[k])*(gy-fy[k]);if(d<second)second=d;}return bk;}};
struct DepthRow {NoiseRow n[3];CellRow c[2];unsigned endSalt=~0u;double endValue=0;}; // one class per row: up to 3 noises, 2 cell grids, its end draw
// T1: how far a muscle tears back at (a, b) on one end: its own retraction `base`, torn chunks on ~14 mm cells (each its own offset
// 0-13 mm and tilt up to 4:5, so a chunk is a slanted facet, never a level), fascicle strands on ~7 mm cells (0-7 mm, tapering
// 5 mm to the strand's tip; longer on the exit side), a slow roll (+-3 mm) and half the shot's lean
inline double muscleEnd(std::uint32_t s,unsigned salt,double a,double b,double base,double down0,double lean,DepthRow*row=nullptr){
 if(row){const double cell=.014,wa=a+.004*row->n[0].at(s,salt,a/.02,b/.02),wb=b+.004*row->n[1].at(s,salt+1,a/.02,b/.02),gx=wa/cell,gy=wb/cell;
  double best=0;const int k=row->c[0].nearest(s,salt+2,3,gx,gy,best);const CellRow&C=row->c[0];
  const double off=C.v[k][0],sx=1.6*(C.v[k][1]-.5),sy=1.6*(C.v[k][2]-.5);const double chunk=.013*off+(sx*(gx-C.fx[k])+sy*(gy-C.fy[k]))*cell;
  double sb=0;const int q=row->c[1].nearest(s,salt+7,1,wa/.007,wb/.007,sb);const double e=std::min(1.,std::sqrt(sb)),t=row->c[1].v[q][0];
  return base+chunk+(1+.5*down0)*(.007*t+.005*e)+.003*row->n[2].at(s,salt+8,a/.02,b/.02)+.5*lean-.003*down0;}
 const double wa=a+.004*vnoise(s,salt,a/.02,b/.02),wb=b+.004*vnoise(s,salt+1,a/.02,b/.02);
 const double cell=.014,gx=wa/cell,gy=wb/cell;const int ix=int(std::floor(gx)),iy=int(std::floor(gy));double best=1e9,fx0=0,fy0=0;int bx=0,by=0;
 for(int i=-1;i<=1;++i)for(int j=-1;j<=1;++j){const int cx=ix+i,cy=iy+j;const double fx=cx+u01(s,cx,cy,salt+2),fy=cy+u01(s,cx,cy,salt+3);
  const double dd=(gx-fx)*(gx-fx)+(gy-fy)*(gy-fy);if(dd<best){best=dd;bx=cx;by=cy;fx0=fx;fy0=fy;}}
 const double off=u01(s,bx,by,salt+4),sx=1.6*(u01(s,bx,by,salt+5)-.5),sy=1.6*(u01(s,bx,by,salt+6)-.5);
 const double chunk=.013*off+(sx*(gx-fx0)+sy*(gy-fy0))*cell;
 double e=0;const double t=strand(s,salt+7,wa,wb,.007,e);
 return base+chunk+(1+.5*down0)*(.007*t+.005*e)+.003*vnoise(s,salt+8,a/.02,b/.02)+.5*lean-.003*down0;}
// T2 (tear2) the limb profile's constants (m unless noted), tuned on the harness's E3 renders of GFY 76d9244d torn ends
namespace limbK {
constexpr double chunkCell=.020,chunkOffset=.014,chunkSlant=1.0; // groups of fascicles: jittered cells, own offset, slant (ratio)
constexpr double bundleCell=.007,bundleReach=.015,reachPower=2,exitReach=.004; // fascicle bundles: cell, how far one reaches out (t^power), exit side
constexpr double tipTaper=.003,borderGroove=.003,fray=.003,frayCell=.003,warp=.003; // a bundle's rounded tip, the groove at its border, fibre fray
constexpr double sizeSpread=.5,reachSpread=.8; // per group and end: bundle size and reach vary +-25 / +-40 % (groups differ in texture)
constexpr double groupFloor=.024,groupSpan=.022; // a group's own retraction per end: 24-46 mm
constexpr double cortexFloor=.002,cortexSpan=.005,splinters=.014,marrowIn=.012; // the bone's break: 2-7 mm, splinters to 14 mm more, marrow 12 mm in
constexpr double shellPast=.006,shellLobes=.014,shellFlaps=.008; // skin/fat/filler: 6 mm past the end's mean muscle retraction, lobes round the limb, flaps
constexpr double boneSide=.014,boneSideFade=.030,dome=.006,domeFade=.012; // section: muscle next to the bone, a group's border (fading out)
}
// T2: a muscle's torn end on one end, at (x, y) ACROSS its fibres (a muscle group's map frame runs along its fibre direction, so a
// bundle is a column along the fibres): `base` = the group's own retraction; chunks of fascicles on ~20 mm jittered cells (own
// offset 0-14 mm, a slant); fascicle bundles on ~7 mm cells (size per group and end), each torn at its own length (few reach far:
// up to 15 mm out of the chunk, longer on the exit side), rounded to the bundle's tip, a groove at its border; fibre fray on 3 mm;
// half the shot's lean. Row caches as muscleEnd (the direct call evaluates through a fresh row: the same numbers).
inline double muscleEnd2(std::uint32_t s,unsigned salt,double x,double y,double base,double down0,double lean,DepthRow*row){
 if(!row){DepthRow tmp;return muscleEnd2(s,salt,x,y,base,down0,lean,&tmp);}
 using namespace limbK;
 const double wa=x+warp*row->n[0].at(s,salt,x/.018,y/.018),wb=y+warp*row->n[1].at(s,salt+1,x/.018,y/.018);
 const double cc=chunkCell;double cb=0;const int k=row->c[0].nearest(s,salt+2,3,wa/cc,wb/cc,cb);const CellRow&C=row->c[0];
 const double chunk=chunkOffset*C.v[k][0]+(chunkSlant*(C.v[k][1]-.5)*(wa/cc-C.fx[k])+chunkSlant*(C.v[k][2]-.5)*(wb/cc-C.fy[k]))*cc;
 const double gs=u01(s,int(salt+30),7,0x3C1u),go=u01(s,int(salt+31),7,0x3C1u); // this group's bundle size and reach on this end
 const double sc=bundleCell*(1+sizeSpread*(gs-.5));double f1=0,f2=0;const int q=row->c[1].nearest2(s,salt+7,1,wa/sc,wb/sc,f1,f2);const double t=row->c[1].v[q][0];
 const double r1=std::sqrt(f1),gap=std::sqrt(f2)-r1,rr=std::min(1.,r1/.7); // the bundle's axis distance and its border distance (cells)
 const double out=(bundleReach*(1+reachSpread*(go-.5))+exitReach*down0)*std::pow(t,reachPower);
 const double tip=tipTaper*rr*rr+borderGroove*(1-std::clamp(gap/.3,0.,1.));
 return base+chunk-out+tip+fray*row->n[2].at(s,salt+8,x/frayCell,y/frayCell)+.5*lean-.003*down0;}
inline double limbDepth2(unsigned c,std::uint32_t seed,int side,double a,double b,const TearImpact&I,DepthRow*row){
 using namespace limbK;
 const std::uint32_t s=seed^(side?0x51u:0xA3u);const double sg=side?1.:-1.;
 const double down=std::clamp((a*I.a+b*I.b)/I.Rl,-1.,1.),down0=std::max(0.,down);
 const double lean=I.tilt*down*sg-.006*I.along*sg,lim=I.lim;
 auto pair=[&](unsigned salt){const double u=u01(seed,int(salt),7,0x3C1u);return side?u:1-u;};
 auto perEnd=[&](unsigned salt){if(row&&row->endSalt==salt)return row->endValue;const double v=u01(s,int(salt),7,0x3C1u);if(row){row->endSalt=salt;row->endValue=v;}return v;};
 auto N=[&](int k,unsigned salt,double x,double y){return row?row->n[k].at(s,salt,x,y):vnoise(s,salt,x,y);};
 auto B=[&](int k,unsigned salt,double x,double y,double cell){if(!row)return bundle(s,salt,x,y,cell);double q=0;const int i=row->c[k].nearest(s,salt,1,x/cell,y/cell,q);return row->c[k].v[i][0];};
 double d;
 switch(c){
  case bone:{const double sp=B(0,82,a,b,.005);d=cortexFloor+cortexSpan*perEnd(81)+splinters*(1-sp*sp)+.002*(.5+.5*N(0,84,a/.012,b/.012))+.3*lean;return ease(d,.0015,lim);}
  case marrow:d=cortexFloor+marrowIn+cortexSpan*perEnd(81)+.004*(.5+.5*N(0,83,a/.004,b/.004))+.3*lean;return ease(d,.006,lim);
  case muscle:return ease(muscleEnd2(s,180,a,b,groupFloor+groupSpan*perEnd(10),down0,lean,row),.006,lim);
  case shell:{const double r=std::sqrt(a*a+b*b)+1e-9,ring=.5+.5*N(0,21,1.3*a/r+3.1,1.3*b/r+5.3);
   d=I.mbase[side?1:0]+shellPast+shellLobes*ring+shellFlaps*B(0,22,a,b,.012)+.004*(.5+.5*N(1,23,a/.025,b/.025))+.0012*N(2,24,a/.004,b/.004)+lean;return ease(d,.010,lim);}
  case small:d=.010+.016*(.5+.5*N(0,41,a/.014,b/.014))+.004*N(1,42,a/.005,b/.005)+.5*lean;return ease(d,.006,lim); // brain, eye, fluid where a tear crosses the head: torn back in lumps (a flat face before)
  case organ:d=.006+.014*(.5+.5*N(0,51,a/.02,b/.02))+.5*lean;break;
  case vessel:d=groupFloor+groupSpan*pair(91)+.004*(.5+.5*N(0,92,a/.01,b/.01))+.5*lean;return ease(d,.006,lim); // vessels retract with the muscles (no pole standing out)
  case fascia:d=.038+.010*perEnd(93)+.003*N(0,94,a/.01,b/.01)+lean;return ease(d,.014,lim);
  default:
   if(c>=mv0&&c<mv0+8){const unsigned v=c-mv0;return ease(muscleEnd2(s,200+20*v,a,b,groupFloor+groupSpan*perEnd(100+v),down0,lean,row),.006,lim);}
   d=.008+.012*(.5+.5*N(0,61,a/.02,b/.02));}
 return std::clamp(d,.002,lim-.001);}
// How far (m) a tissue tears back from the damage at cross-section point (a, b) on one side of the tear plane. Per-structure
// lengths are random per sever (seed) and complementary between the two ends: what tears deep on one end stands out of the other.
inline double depthOf(unsigned c,std::uint32_t seed,int side,double a,double b,const TearImpact&I,DepthRow*row=nullptr){ // T1: row = the map row's lattice caches
 if(!I.head)return limbDepth2(c,seed,side,a,b,I,row); // T2: the limb profile
 const std::uint32_t s=seed^(side?0x51u:0xA3u);const bool radial=I.head==2;
 auto pair=[&](unsigned salt){const double u=u01(seed,int(salt),7,0x3C1u);return side?u:1-u;};
 double d;
 if(c==small||c==organ)d=radial?.004+.006*(.5+.5*vnoise(s,43,a/.012,b/.012))+.003*vnoise(s,44,a/.005,b/.005)
                               :.010+.020*(.5+.5*vnoise(s,43,a/.014,b/.014))+.006*vnoise(s,44,a/.005,b/.005); // torn brain, lumpy
 else if(c==bone||c==marrow)d=radial?.002+.002*pair(81)+.002*bundle(s,82,a,b,.005):.002+.004*pair(81)+.003*bundle(s,82,a,b,.005); // splintered plates
 else if(c==shell)d=radial?.005+.004*(.5+.5*vnoise(s,21,a/.02,b/.02)):.012+.010*(.5+.5*vnoise(s,21,a/.02,b/.02)); // scalp torn back (T2: cap 12-22 mm, to the torn brain's level)
 else if(c==vessel)d=.004+.010*pair(91); // vessels
 else d=radial?.004+.006*(.5+.5*vnoise(s,47,a/.012,b/.012))+.002*vnoise(s,48,a/.005,b/.005) // T2: the head's muscles, fascia and the rest: torn like its brain
              :.010+.016*(.5+.5*vnoise(s,47,a/.014,b/.014))+.004*vnoise(s,48,a/.005,b/.005);
 return std::clamp(d,.002,maxDepth-.001);}
// A field cell of a grid cell: the same lattice (integer offset) or the field sampled at the cell centre.
struct Lattice {const painter_detail::TissueField*f{};int off[3]{};bool aligned{};
 void bind(const painter_detail::TissueField*field,const Grid&g){f=field&&field->available?field:nullptr;aligned=false;if(!f)return;
  if(std::abs(double(f->pitch)-g.pitch)>1e-6*g.pitch)return;
  for(int k=0;k<3;++k){const double o=(g.origin[k]-double(f->origin[k]))/g.pitch,r=std::round(o);if(std::abs(o-r)>1e-3)return;off[k]=int(r);}
  aligned=true;}
 unsigned at(const Grid&g,const Cell&c)const{if(!f)return 0;if(aligned)return f->at(c.x+off[0],c.y+off[1],c.z+off[2]);const Vec p=g.center(c);return f->sample(p.x,p.y,p.z);}
 // the same read with the brick's page entry cached (consecutive x cells of one brick share it)
 struct Cache {std::size_t page=~std::size_t(0);unsigned v=0;};
 unsigned at(const Grid&g,const Cell&c,Cache&k)const{if(!f)return 0;if(!aligned){const Vec p=g.center(c);return f->sample(p.x,p.y,p.z);}
  const int x=c.x+off[0],y=c.y+off[1],z=c.z+off[2];if(x<0||y<0||z<0||unsigned(x)>=f->size[0]||unsigned(y)>=f->size[1]||unsigned(z)>=f->size[2])return 0;
  const std::size_t pg=(std::size_t(z>>3)*f->pages[1]+unsigned(y>>3))*f->pages[0]+unsigned(x>>3)+painter_detail::TissueField::headerWords;
  if(pg!=k.page){k.page=pg;k.v=f->page[pg];}const unsigned v=k.v;if(v&painter_detail::TissueField::uniformBit)return v&127u; // R72: padding bit 64
  const unsigned local=((unsigned(z)&7u)*8+(unsigned(y)&7u))*8+(unsigned(x)&7u),bits=1u<<((v>>28)&3u),offset=v&0x0fffffffu;
  if(bits==8)return f->data[offset+local];const unsigned palette=1u<<bits,bit=local*bits;
  return f->data[offset+((f->data[offset+palette+(bit>>3)]>>(bit&7))&((1u<<bits)-1))];}};
struct Report {
 bool done{};const char* refusal{};std::size_t box{},sources{},region{},torn{},thin{},loose{},rim{},fresh{},solid{},rawQuads{},vertices{},triangles{};
 std::array<std::size_t,64> bySlot{};std::vector<Cell> tiles;std::shared_ptr<const ResolvedTearSurface> surface;
 double ms{},sourceMs{},fillMs{},regionMs{},edtMs{},tearMs{},thinMs{},islandMs{},writeMs{},blurMs{},netsMs{},simplifyMs{};
 double maxEdgeMm{};std::size_t longTris{}; // T1 (GFY_TISSUE_TEAR_LOG only): the torn surface's longest edge, triangles with an edge over 8 mm
 std::size_t clusterTriangles{},budget{},wholeTriangles{},outerTriangles{},patchTriangles{};double wholeMs{},wholeErrorMm{},patchMs{},patchOffset{};bool lowPoly{}; // speed: triangles after the cluster pass, the whole-surface pass's budget, result, time and reached error; the surface's own triangles and its backing patches
};
// Bit rows (x along the words) over the box, for the opening / dilation steps.
struct Bits {
 int nx{},ny{},nz{},W{};Raw<std::uint64_t> w;std::uint64_t lastMask{};
 // R64: shape() for an output written in full (step), init() = shape() + the zero fill side by side
 void shape(int x,int y,int z){nx=x;ny=y;nz=z;W=(nx+63)/64;w.resize(std::size_t(W)*ny*nz);const int vb=nx-64*(W-1);lastMask=vb==64?~std::uint64_t(0):((std::uint64_t(1)<<vb)-1);}
 void init(int x,int y,int z){shape(x,y,z);parallelFill(w,std::uint64_t(0));}
 std::uint64_t* row(int y,int z){return w.data()+(std::size_t(z)*ny+y)*W;}
 const std::uint64_t* row(int y,int z)const{return w.data()+(std::size_t(z)*ny+y)*W;}
 void set(int x,int y,int z){row(y,z)[x>>6]|=std::uint64_t(1)<<(x&63);}
 bool get(int x,int y,int z)const{return (row(y,z)[x>>6]>>(x&63))&1u;}
};
// One step of the 6-neighbourhood: erosion (cells outside the box count as set) or dilation (they count as clear); the
// result is ANDed with `mask` (when given). Rows run side by side.
inline void step(const Bits&a,Bits&out,bool erode,const Bits*mask){
 out.shape(a.nx,a.ny,a.nz);const int W=a.W;const std::uint64_t ones=~std::uint64_t(0);
 ttParallel(std::size_t(a.nz),2,[&](std::size_t z0,std::size_t z1,std::size_t){
  for(int z=int(z0);z<int(z1);++z)for(int y=0;y<a.ny;++y){const std::uint64_t*r=a.row(y,z);std::uint64_t*o=out.row(y,z);
   const std::uint64_t*nb[4]={y>0?a.row(y-1,z):nullptr,y+1<a.ny?a.row(y+1,z):nullptr,z>0?a.row(y,z-1):nullptr,z+1<a.nz?a.row(y,z+1):nullptr};
   const std::uint64_t*m=mask?mask->row(y,z):nullptr;
   for(int k=0;k<W;++k){const std::uint64_t c=r[k];
    std::uint64_t left=(c<<1)|(k?r[k-1]>>63:(erode?1u:0u)),right=(c>>1)|(k+1<W?r[k+1]<<63:0);
    if(erode&&k+1==W){const int vb=a.nx-64*(W-1);right|=std::uint64_t(1)<<(vb-1);}
    std::uint64_t v;
    if(erode){v=c&left&right;for(const auto*q:nb)v&=q?q[k]:ones;}
    else{v=c|left|right;for(const auto*q:nb)if(q)v|=q[k];}
    if(k+1==W)v&=a.lastMask;if(m)v&=m[k];o[k]=v;}}});
}
// R63: 6-connected components of x runs over a dense box of nx*ny*nz cells (index (z*ny+y)*nx+x) holding inside(i), labelled
// side by side with a lock-free union-find; every cell of a component holding a cell with seed(i,x,y,z) gets mark(i). The cells
// marked are exactly those a flood from the seed cells through inside cells reaches. inside/seed: read-only during the call.
template<class Inside,class Seed,class Mark> inline void runComponents(int nx,int ny,int nz,const Inside&inside,const Seed&seed,const Mark&mark){
 struct Run {std::int32_t x0,x1;std::uint32_t row;std::uint8_t seed;};
 const std::size_t R=std::size_t(ny)*std::size_t(nz);std::vector<std::vector<Run>> parts(ttWidth());std::vector<std::uint32_t> rowCount(R,0);
 ttParallel(std::size_t(nz),2,[&](std::size_t z0,std::size_t z1,std::size_t chunk){auto&P=parts[chunk];
  for(int z=int(z0);z<int(z1);++z)for(int y=0;y<ny;++y){const std::size_t r=std::size_t(z)*std::size_t(ny)+std::size_t(y),base=r*std::size_t(nx);std::uint32_t cnt=0;
   for(int x=0;x<nx;){if(!inside(base+std::size_t(x))){++x;continue;}Run run{x,x,std::uint32_t(r),0};
    while(x<nx&&inside(base+std::size_t(x))){if(!run.seed&&seed(base+std::size_t(x),x,y,z))run.seed=1;run.x1=x;++x;}P.push_back(run);++cnt;}
   rowCount[r]=cnt;}});
 std::vector<std::uint32_t> rowFirst(R+1,0);for(std::size_t r=0;r<R;++r)rowFirst[r+1]=rowFirst[r]+rowCount[r];
 std::vector<Run> runs;runs.reserve(rowFirst[R]);for(auto&P:parts){runs.insert(runs.end(),P.begin(),P.end());std::vector<Run>().swap(P);}
 if(runs.size()!=rowFirst[R])throw std::runtime_error("tissue_tear_runs");
 const std::uint32_t NR=std::uint32_t(runs.size());if(!NR)return;
 std::unique_ptr<std::atomic<std::uint32_t>[]> par(new std::atomic<std::uint32_t>[NR]);std::unique_ptr<std::atomic<std::uint8_t>[]> hit(new std::atomic<std::uint8_t>[NR]);
 ttParallel(NR,1u<<14,[&](std::size_t k0,std::size_t k1,std::size_t){for(std::size_t k=k0;k<k1;++k){par[k].store(std::uint32_t(k),std::memory_order_relaxed);hit[k].store(0,std::memory_order_relaxed);}});
 auto find=[&](std::uint32_t a){for(;;){std::uint32_t q=par[a].load(std::memory_order_relaxed);if(q==a)return a;const std::uint32_t g2=par[q].load(std::memory_order_relaxed);
   if(g2!=q)par[a].compare_exchange_weak(q,g2,std::memory_order_relaxed);a=g2;}};
 auto unite=[&](std::uint32_t a,std::uint32_t b){for(;;){a=find(a);b=find(b);if(a==b)return;if(a<b)std::swap(a,b);std::uint32_t e=a;
   if(par[a].compare_exchange_strong(e,b,std::memory_order_relaxed))return;}};
 auto link=[&](std::size_t r,std::size_t q){std::uint32_t a=rowFirst[r],b=rowFirst[q];const std::uint32_t ae=rowFirst[r+1],be=rowFirst[q+1];
  while(a<ae&&b<be){const Run&A=runs[a];const Run&B=runs[b];if(A.x1<B.x0){++a;continue;}if(B.x1<A.x0){++b;continue;}unite(a,b);if(A.x1<B.x1)++a;else ++b;}};
 ttParallel(std::size_t(nz),2,[&](std::size_t z0,std::size_t z1,std::size_t){
  for(int z=int(z0);z<int(z1);++z)for(int y=0;y<ny;++y){const std::size_t r=std::size_t(z)*std::size_t(ny)+std::size_t(y);if(rowFirst[r]==rowFirst[r+1])continue;
   if(y>0)link(r,r-1);if(z>0)link(r,r-std::size_t(ny));}});
 ttParallel(NR,4096,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t k=a;k<b;++k)if(runs[k].seed)hit[find(std::uint32_t(k))].store(1,std::memory_order_relaxed);});
 ttParallel(NR,4096,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t k=a;k<b;++k)if(hit[find(std::uint32_t(k))].load(std::memory_order_relaxed)){
  const std::size_t base=std::size_t(runs[k].row)*std::size_t(nx);for(int x=runs[k].x0;x<=runs[k].x1;++x)mark(base+std::size_t(x));}});
}
struct Work {
 Grid&g;const painter_detail::TissueField&T;const painter_detail::TissueField*A;
 Vec C,n,u,w;double Rl,H,pitch;std::uint32_t seed;Lattice TL,AL;Vec limbPoint{},limbAxis{};TearImpact imp;double depthScale=1.;bool keepIslands=false; // R62 head fractures: shallower ends; whole skull pieces are kept
 Vec fe1[classes]{},fe2[classes]{};TearImpact impC[classes];double maxD=maxDepth; // T2: each class's map frame (muscles: across their fibres) and impact; the depth limit
 // T2: the cross-section at the plane on 2 mm cells (u, w; origin secO): the classes there (region), the muscles' extra depth (section)
 int secN=0;double secO=0;std::unique_ptr<std::atomic<std::uint32_t>[]> secCls;std::vector<float> secExtra;std::vector<std::uint8_t> secFoot; // + the bone's footprint
 int lo[3]{},dim[3]{};std::size_t N{};
 // T1: a piece's grid: its ownership, and per box column (x, y) the piece's body / cloth spans clipped to the box's z range
 const core::PieceOwnership*own=nullptr;double ripR=0; // T1: ripR = the rip channels' radius (bone slab)
 double farReach=0; // speed: fill()'s FAR threshold (a 1 mm cell is FAR when its 2 mm cell's distance exceeds it)
 struct OwnIndex {std::vector<std::uint32_t> first;std::vector<std::int32_t> span;int x0=0,y0=0,nx=0,ny=0;
  void build(const core::PieceOwnership&o,int lx,int ly,int lz,int dx,int dy,int dz){x0=lx;y0=ly;nx=dx;ny=dy;const int z0=lz,z1=lz+dz;
   const std::size_t C=std::size_t(nx)*std::size_t(ny);first.assign(C+1,0);std::vector<std::uint32_t> at;
   for(int pass=0;pass<2;++pass){
    for(int list=0;list<2;++list){const auto&cols=list?o.cloth_columns:o.body_columns;const auto&sp=list?o.cloth_spans:o.body_spans;
     for(int y=y0;y<y0+ny;++y){auto it=std::lower_bound(cols.begin(),cols.end(),std::make_pair(y,x0),[](const core::OwnerColumn&a,const std::pair<int,int>&b){return a.y<b.first||(a.y==b.first&&a.x<b.second);});
      for(;it!=cols.end()&&it->y==y&&it->x<x0+nx;++it){const std::size_t col=std::size_t(y-y0)*std::size_t(nx)+std::size_t(it->x-x0);
       for(std::uint32_t k=it->first;k<it->first+it->count;++k){const int a=std::max(sp[k].lo,z0),b=std::min(sp[k].hi,z1);if(a>=b)continue;
        if(pass==0)++first[col+1];else{const std::uint32_t q=at[col]++;span[2*q]=a;span[2*q+1]=b;}}}}}
    if(pass==0){for(std::size_t c=0;c<C;++c)first[c+1]+=first[c];span.resize(2*std::size_t(first[C]));at.assign(first.begin(),first.end()-1);}}}
  bool has(int cx,int cy,int cz)const{const std::size_t col=std::size_t(cy-y0)*std::size_t(nx)+std::size_t(cx-x0);
   for(std::uint32_t k=first[col];k<first[col+1];++k)if(cz>=span[2*k]&&cz<span[2*k+1])return true;return false;}
 } ownIdx;
 // R63: per tissue class (classOf, filler as shell) the a/b extent in the cross-section plane of the cells region() saw
 double extLo[classes][2]{},extHi[classes][2]{};bool extAny[classes]{};int dbgCyl[3]{},dbgSrc[3]{};double dbgMs[16]{};std::uint64_t dbgMaskHash=1469598103934665603ull; // region: cf/flood/mark; write: steps/rim/radial/mask // R63 diagnostics: cylinder AABB and damage-source AABB sizes (cells)
 Raw<std::uint8_t> lab,f;bool present[classes]={};
 // 2 mm cells: the distance (m) from each to the nearest damaged 2 mm cell (exact Euclidean)
 int cdim[3]{};std::size_t NC{};Raw<float> cdist;
 std::size_t cidx(int x,int y,int z)const{return (std::size_t(z)*std::size_t(cdim[1])+std::size_t(y))*std::size_t(cdim[0])+std::size_t(x);}
 std::size_t cof(int x,int y,int z)const{return cidx(x/2,y/2,z/2);}
 Report rep;
 Work(Grid&grid,const painter_detail::TissueField&closure,const painter_detail::TissueField*anatomy):g(grid),T(closure),A(anatomy){}
 std::size_t idx(int x,int y,int z)const{return (std::size_t(z)*std::size_t(dim[1])+std::size_t(y))*std::size_t(dim[0])+std::size_t(x);}
 void xyz(std::size_t i,int&x,int&y,int&z)const{x=int(i%std::size_t(dim[0]));const std::size_t r=i/std::size_t(dim[0]);y=int(r%std::size_t(dim[1]));z=int(r/std::size_t(dim[1]));}
 Cell cell(int x,int y,int z)const{return Cell{lo[0]+x,lo[1]+y,lo[2]+z};}
 std::uint8_t state(std::size_t i)const{return f[i]&STATE;}
 bool kept(std::size_t i)const{return (f[i]&REGION)&&(f[i]&STATE)==KEPT;}
 bool cavity(std::size_t i)const{if(!(f[i]&REGION))return false;const auto s=f[i]&STATE;return s==CUT||s==TORN||s==LOOSE;}
 // the torn surface: kept limb tissue against the limb's cavity, and any other kept tissue (outside the limb or the cylinder)
 // against what the tear removed (not against the cuts: their own walls bound it there)
 bool solid(std::size_t i)const{const auto s=f[i]&STATE;return s==KEPT||s==PLUG;} // T2: + PLUG (a capped bore through the bone)
 bool tornAway(std::size_t i)const{if(!(f[i]&REGION))return false;const auto s=f[i]&STATE;return s==TORN||s==LOOSE;}
 bool face(std::size_t i,std::size_t j)const{return solid(i)&&cavity(j)&&((f[i]&REGION)||tornAway(j));}
 // per row (z * ny + y): the x range of tissue within the tear's reach (fill); grown by 3 cells in x, y and z (the rim's air);
 // the x range of the cells marked for the erase
 std::vector<std::int32_t> spanLo,spanHi,padLo,padHi,markLo,markHi;
 std::size_t rowOf(int y,int z)const{return std::size_t(z)*std::size_t(dim[1])+std::size_t(y);}
 void markRow(int x,int y,int z){const auto r=rowOf(y,z);markLo[r]=std::min(markLo[r],x);markHi[r]=std::max(markHi[r],x);}
 // body(x, y, z, i, chunk) over each row's tissue span (pad: the grown spans), z slabs side by side
 template<class F> void eachSpan(bool pad,F&&body){const auto&L=pad?padLo:spanLo;const auto&Hh=pad?padHi:spanHi;
  ttParallel(std::size_t(dim[2]),2,[&](std::size_t z0,std::size_t z1,std::size_t chunk){
   for(int z=int(z0);z<int(z1);++z)for(int y=0;y<dim[1];++y){const auto r=rowOf(y,z);if(L[r]>Hh[r])continue;std::size_t i=idx(L[r],y,z);
    for(int x=L[r];x<=Hh[r];++x,++i)body(x,y,z,i,chunk);}});}
 // body(x, y, z, i, chunk) over the box, z slabs side by side
 template<class F> void eachCell(F&&body){ttParallel(std::size_t(dim[2]),2,[&](std::size_t z0,std::size_t z1,std::size_t chunk){
  for(int z=int(z0);z<int(z1);++z)for(int y=0;y<dim[1];++y){std::size_t i=idx(0,y,z);for(int x=0;x<dim[0];++x,++i)body(x,y,z,i,chunk);}});}
 bool inCyl(const Vec&p,double&s,double&r2)const{const Vec d=p-C;s=dot(d,n);r2=dot(d,d)-s*s;return std::abs(s)<=H&&r2<=Rl*Rl;}
 // ---- 1. the damage in the cylinder (removed tissue cells) and the box around it
 bool box(){
  const auto t0=core::Clock::now();int clo[3],chi[3];
  for(int k=0;k<3;++k){const double e=H*std::abs(n[k])+Rl*std::sqrt(std::max(0.,1-n[k]*n[k]));
   clo[k]=std::max(0,int(std::floor((C[k]-e-g.origin[k])/pitch)));chi[k]=std::min((&g.size.x)[k]-1,int(std::floor((C[k]+e-g.origin[k])/pitch)));
   if(clo[k]>chi[k]){rep.refusal="tissue_tear_outside_grid";return false;}}
  const int tlo[3]={clo[0]>>3,clo[1]>>3,clo[2]>>3},thi[3]={chi[0]>>3,chi[1]>>3,chi[2]>>3};const int tnx=thi[0]-tlo[0]+1;
  struct Part {int lo[3]={INT_MAX,INT_MAX,INT_MAX},hi[3]={INT_MIN,INT_MIN,INT_MIN};std::size_t n=0;};std::vector<Part> parts(ttWidth());
  ttParallel(std::size_t(tnx),1,[&](std::size_t a,std::size_t b,std::size_t chunk){auto&P=parts[chunk];
   for(int tx=tlo[0]+int(a);tx<tlo[0]+int(b);++tx)for(int ty=tlo[1];ty<=thi[1];++ty)for(int tz=tlo[2];tz<=thi[2];++tz){const core::Tile*t=g.findTile(Cell{tx,ty,tz});if(!t)continue;
    for(unsigned q=0;q<8;++q){std::uint64_t bits=t->removed[q];while(bits){const unsigned l=q*64+core::lowestBit(bits);bits&=bits-1;
     const Cell c{tx*8+int(l>>6),ty*8+int((l>>3)&7),tz*8+int(l&7)};if(!g.valid(c))continue;double s,r2;if(!inCyl(g.center(c),s,r2))continue;
     if(std::abs(s)>sourceSlab)continue; // only damage that can shape the torn ends (grenades: not every crater of the section)
     if(!TL.at(g,c))continue;++P.n;const int v[3]={c.x,c.y,c.z};for(int k=0;k<3;++k){P.lo[k]=std::min(P.lo[k],v[k]);P.hi[k]=std::max(P.hi[k],v[k]);}}}}});
  int slo[3]={INT_MAX,INT_MAX,INT_MAX},shi[3]={INT_MIN,INT_MIN,INT_MIN};
  for(const auto&P:parts){rep.sources+=P.n;for(int k=0;k<3;++k){slo[k]=std::min(slo[k],P.lo[k]);shi[k]=std::max(shi[k],P.hi[k]);}}
  rep.sourceMs=core::ms(t0);for(int k=0;k<3;++k){dbgCyl[k]=chi[k]-clo[k]+1;dbgSrc[k]=shi[k]>=slo[k]?shi[k]-slo[k]+1:0;}
  if(!rep.sources){rep.refusal="tissue_tear_no_damage";return false;}
  // the box: the damage grown by the deepest tear (+ the mesh margin) along the plane normal, but only 25 mm across it - the tear
  // runs through the limb, never out past its side, and the damage of a tear spans the limb's section
  N=1;
  for(int k=0;k<3;++k){const int M=int(std::ceil(((maxD+meshMargin)*std::abs(n[k])+.025*std::sqrt(std::max(0.,1-n[k]*n[k])))/pitch)); // T2: maxD
   lo[k]=std::max(clo[k],slo[k]-M);const int hi=std::min(chi[k],shi[k]+M);dim[k]=hi-lo[k]+1;if(dim[k]<3){rep.refusal="tissue_tear_box_empty";return false;}N*=std::size_t(dim[k]);}
  rep.box=N;
  static const std::size_t budget=[]{const char*e=std::getenv("GFY_TISSUE_TEAR_CELLS_M");return std::size_t(e?std::max(1,std::atoi(e)):160)<<20;}(); // R62: oblique grenade blades box ~25-35 M cells (axis-aligned box of a tilted slab); row 7b: 160 M so trunk tears get torn ends (measured: waist 95.6 M, chest 140.6 M, ~0.3 GB for 0.56 s)
  if(N>budget){rep.refusal="tissue_tear_box_budget";return false;}
  // the damage on 2 mm cells (the same scan), then its exact distance transform there
  NC=1;for(int k=0;k<3;++k){cdim[k]=(dim[k]+1)/2;NC*=std::size_t(cdim[k]);}Raw<std::uint8_t> src(NC);parallelFill(src,std::uint8_t(0));
  ttParallel(std::size_t(tnx),1,[&](std::size_t a,std::size_t b,std::size_t){
   for(int tx=tlo[0]+int(a);tx<tlo[0]+int(b);++tx)for(int ty=tlo[1];ty<=thi[1];++ty)for(int tz=tlo[2];tz<=thi[2];++tz){const core::Tile*t=g.findTile(Cell{tx,ty,tz});if(!t)continue;
    for(unsigned q=0;q<8;++q){std::uint64_t bits=t->removed[q];while(bits){const unsigned l=q*64+core::lowestBit(bits);bits&=bits-1;
     const Cell c{tx*8+int(l>>6),ty*8+int((l>>3)&7),tz*8+int(l&7)};const int x=c.x-lo[0],y=c.y-lo[1],z=c.z-lo[2];if(x<0||y<0||z<0||x>=dim[0]||y>=dim[1]||z>=dim[2])continue;
     double s2,r2;if(!inCyl(g.center(c),s2,r2)||std::abs(s2)>sourceSlab||!TL.at(g,c))continue;src[cof(x,y,z)]=1;}}}});
  const auto e0=core::Clock::now();coarseDistance(src);rep.edtMs=core::ms(e0);
  return true;}
 // exact distance (m) from each 2 mm cell to the nearest damaged 2 mm cell (Felzenszwalb on squared cells, saturated)
 void coarseDistance(const Raw<std::uint8_t>&src){
  const int nx=cdim[0],ny=cdim[1],nz=cdim[2];Raw<std::uint16_t> cd2(NC); // R64: each row set to 65535 by its writer below
  ttParallel(std::size_t(nz),2,[&](std::size_t z0,std::size_t z1,std::size_t){
   for(int z=int(z0);z<int(z1);++z)for(int y=0;y<ny;++y){const std::size_t row=cidx(0,y,z);int last=-1000000;std::fill(cd2.data()+row,cd2.data()+row+std::size_t(nx),std::uint16_t(65535));
    for(int x=0;x<nx;++x){if(src[row+x])last=x;const int dd=x-last;if(dd<256)cd2[row+x]=std::uint16_t(dd*dd);}
    last=1000000;for(int x=nx-1;x>=0;--x){if(src[row+x])last=x;const int dd=last-x;if(dd<256){const auto v=std::uint16_t(dd*dd);if(v<cd2[row+x])cd2[row+x]=v;}}}});
  auto line=[](const float*in,int n,float*out,int*v,double*z){int k=-1;
   for(int q=0;q<n;++q){const double fq=in[q];if(fq>=INF)continue;double s=-1e30;
    while(k>=0){const int p=v[k];s=((fq+double(q)*q)-(double(in[p])+double(p)*p))/(2.*(q-p));if(s>z[k])break;--k;}
    ++k;v[k]=q;z[k]=k==0?-1e30:s;z[k+1]=1e30;}
   if(k<0){for(int q=0;q<n;++q)out[q]=INF;return;}
   int j=0;for(int q=0;q<n;++q){while(z[j+1]<q)++j;const int p=v[j];out[q]=float(std::min(double(INF),double(q-p)*(q-p)+in[p]));}};
  const std::size_t sx=std::size_t(nx),sxy=sx*std::size_t(ny);
  for(int pass=0;pass<2;++pass){const int n1=pass==0?ny:nz,outer=pass==0?nz:ny;const std::size_t stride=pass==0?sx:sxy;
   ttParallel(std::size_t(outer),1,[&](std::size_t o0,std::size_t o1,std::size_t){
    std::vector<float> in(static_cast<std::size_t>(n1)),out(static_cast<std::size_t>(n1));std::vector<int> v(std::size_t(n1)+1);std::vector<double> z(std::size_t(n1)+2);
    for(int o=int(o0);o<int(o1);++o)for(int x=0;x<nx;++x){const std::size_t base=pass==0?cidx(x,0,o):cidx(x,o,0);
     bool any=false;for(int q=0;q<n1;++q){const auto dd=cd2[base+std::size_t(q)*stride];in[std::size_t(q)]=float(dd);any=any||dd<65535;}
     if(!any)continue;line(in.data(),n1,out.data(),v.data(),z.data());
     for(int q=0;q<n1;++q)cd2[base+std::size_t(q)*stride]=std::uint16_t(std::min(65535.f,out[std::size_t(q)]));}});}
  cdist.resize(NC);const double unit2=2*pitch;
  ttParallel(NC,1u<<16,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t i=a;i<b;++i)cdist[i]=float(std::sqrt(double(cd2[i]))*unit2);});}
 // the distance (m) from a 1 mm cell's centre to the damage: trilinear between the 2 mm cells
 double distAt(int x,int y,int z)const{
  const double fx=(x+.5)*.5-.5,fy=(y+.5)*.5-.5,fz=(z+.5)*.5-.5;const int x0=std::max(0,int(std::floor(fx))),y0=std::max(0,int(std::floor(fy))),z0=std::max(0,int(std::floor(fz)));
  const int x1=std::min(cdim[0]-1,x0+1),y1=std::min(cdim[1]-1,y0+1),z1=std::min(cdim[2]-1,z0+1);
  const double tx=std::clamp(fx-x0,0.,1.),ty=std::clamp(fy-y0,0.,1.),tz=std::clamp(fz-z0,0.,1.);
  auto D=[&](int a,int b,int c){return double(cdist[cidx(a,b,c)]);};
  const double c00=D(x0,y0,z0)+(D(x1,y0,z0)-D(x0,y0,z0))*tx,c10=D(x0,y1,z0)+(D(x1,y1,z0)-D(x0,y1,z0))*tx;
  const double c01=D(x0,y0,z1)+(D(x1,y0,z1)-D(x0,y0,z1))*tx,c11=D(x0,y1,z1)+(D(x1,y1,z1)-D(x0,y1,z1))*tx;
  const double c0=c00+(c10-c00)*ty,c1=c01+(c11-c01)*ty;return c0+(c1-c0)*tz;}
 // ---- 2. labels and states: closure slot, removed (cut), clothing filler (closure 16 or no anatomy), in the cylinder
 void fill(){
  const auto t0=core::Clock::now();lab.resize(N);f.resize(N); // R64: every cell is written below (no zero fill)
  const std::size_t R=std::size_t(dim[1])*dim[2];spanLo.assign(R,INT_MAX);spanHi.assign(R,INT_MIN);markLo.assign(R,INT_MAX);markHi.assign(R,INT_MIN);
  const double edge=1.5*pitch,far=maxD+.012+std::sqrt(3.)*pitch; // T2: maxD
  farReach=far; // speed: region() skips the 2 mm cells whose eight cells are all FAR
  // T1: on a piece's grid, a tissue cell that is not stored removed but is neither in a pure tile, the piece's spans nor anatomical
  // air is another piece's (Grid::ownerCuttable false): FOREIGN; a removed cell farther off the plane than the damage band: OLDCUT
  if(own)ownIdx.build(*own,lo[0],lo[1],lo[2],dim[0],dim[1],dim[2]);
  const double oldSlab=sourceSlab+2*pitch;
  // R63: per row the centre's y/z terms, the 2 mm distance row, the field page rows and the grid tile row; per 8-cell brick the
  // page entry / tile; per cell the same expressions as Grid::center, inCyl, Lattice::at and tileBit (identical results)
  const bool rowFields=TL.f&&TL.aligned&&(!AL.f||AL.aligned);
  std::vector<double> dxs(static_cast<std::size_t>(dim[0]));for(int x=0;x<dim[0];++x)dxs[std::size_t(x)]=(g.origin.x+(double(lo[0]+x)+.5)*pitch)-C.x;
  const double H2=H-edge,R2e=(Rl-edge)*(Rl-edge),Rl2=Rl*Rl;
  // a field's value at field coordinates (x, y, z) from its page entry (TissueField::at after the page lookup)
  auto decode=[](const painter_detail::TissueField&F,unsigned v,unsigned local)->unsigned{if(v&painter_detail::TissueField::uniformBit)return v&127u; // R72: padding bit 64
   const unsigned bits=1u<<((v>>28)&3u),offset=v&0x0fffffffu;if(bits==8)return F.data[offset+local];const unsigned palette=1u<<bits,bit=local*bits;
   return F.data[offset+((F.data[offset+palette+(bit>>3)]>>(bit&7))&((1u<<bits)-1))];};
  ttParallel(std::size_t(dim[2]),2,[&](std::size_t z0,std::size_t z1,std::size_t){core::TileCache cache(g);Lattice::Cache tk,ak;
   for(int z=int(z0);z<int(z1);++z)for(int y=0;y<dim[1];++y){const std::size_t row0=idx(0,y,z);const int cy=lo[1]+y,cz=lo[2]+z;
    const double dy=(g.origin.y+(double(cy)+.5)*pitch)-C.y,dz=(g.origin.z+(double(cz)+.5)*pitch)-C.z;
    const float*crow=cdist.data()+cidx(0,y/2,z/2);int sLo=INT_MAX,sHi=INT_MIN;
    {bool reach=false;for(int q=0;q<cdim[0]&&!reach;++q)reach=!(double(crow[q])>far); // speed: every cell of the row tests crow[x/2] > far below
     if(!reach){std::fill(lab.data()+row0,lab.data()+row0+std::size_t(dim[0]),std::uint8_t(FAR));std::fill(f.data()+row0,f.data()+row0+std::size_t(dim[0]),std::uint8_t(KEPT));
      const auto rr=rowOf(y,z);spanLo[rr]=sLo;spanHi[rr]=sHi;continue;}}
    if(!rowFields){
     for(int x=0;x<dim[0];++x){const Cell c=cell(x,y,z);const std::size_t i=row0+std::size_t(x);
      if(double(crow[x/2])>far){lab[i]=FAR;f[i]=KEPT;continue;}
      double s,r2;const bool cyl=inCyl(g.center(c),s,r2);
      std::uint8_t flags=cyl?CYL:0;unsigned L=TL.at(g,c,tk); // T1: not const (another piece's cell reads 0)
      if(L){const core::Tile*t=cache.find(Grid::tileKey(c));const bool removed=t&&core::tileBit(*t,c);flags|=removed?CUT:KEPT;
       if((L&64u)||(L&63u)==16||(AL.f&&!AL.at(g,c,ak)))flags|=FILL;} // R72: the closure marks its padding (bit 64)
      if(own&&L){if((flags&STATE)==KEPT){if(!own->pureTile(c)&&!ownIdx.has(c.x,c.y,c.z)&&g.rawMaterialBase(c)){flags=std::uint8_t((flags&CYL)|FOREIGN);L=0;}}
       else if(std::abs(s)>oldSlab)flags=std::uint8_t((flags&~STATE)|OLDCUT);}
      if(!imp.head&&(flags&STATE)==CUT&&((L&63u)==2||(L&63u)==21))flags=std::uint8_t((flags&~STATE)|PLUG); // T2
      std::uint8_t l=std::uint8_t(L&63u);if(cyl&&(std::abs(s)>H2||r2>R2e))l|=EDGE;
      lab[i]=l;f[i]=flags;if(L){sLo=std::min(sLo,x);sHi=std::max(sHi,x);}}}
    else{
     const auto&TF=*TL.f;const int ty=cy+TL.off[1],tz=cz+TL.off[2];const bool tRow=ty>=0&&tz>=0&&unsigned(ty)<TF.size[1]&&unsigned(tz)<TF.size[2];
     const std::size_t tBase=tRow?(std::size_t(tz>>3)*TF.pages[1]+unsigned(ty>>3))*TF.pages[0]+painter_detail::TissueField::headerWords:0;const unsigned tYZ=((unsigned(tz)&7u)*8+(unsigned(ty)&7u))*8;
     const painter_detail::TissueField*AF=AL.f;const int ay=AF?cy+AL.off[1]:0,az=AF?cz+AL.off[2]:0;const bool aRow=AF&&ay>=0&&az>=0&&unsigned(ay)<AF->size[1]&&unsigned(az)<AF->size[2];
     const std::size_t aBase=aRow?(std::size_t(az>>3)*AF->pages[1]+unsigned(ay>>3))*AF->pages[0]+painter_detail::TissueField::headerWords:0;const unsigned aYZ=((unsigned(az)&7u)*8+(unsigned(ay)&7u))*8;
     const unsigned tileBitIndex=unsigned((cy&7)*8+(cz&7));const int tileY=cy/8,tileZ=cz/8;int tileX=INT_MIN;const core::Tile*tile=nullptr;
     std::size_t tPage=~std::size_t(0),aPage=~std::size_t(0);unsigned tv=0,av=0;int pureX=INT_MIN;bool pureNow=false;
     for(int x=0;x<dim[0];++x){const std::size_t i=row0+std::size_t(x);
      if(double(crow[x/2])>far){lab[i]=FAR;f[i]=KEPT;continue;}
      const double dx=dxs[std::size_t(x)],s=dx*n.x+dy*n.y+dz*n.z,r2=(dx*dx+dy*dy+dz*dz)-s*s;const bool cyl=std::abs(s)<=H&&r2<=Rl2;
      std::uint8_t flags=cyl?CYL:0;unsigned L=0;const int cx=lo[0]+x;
      {const int tx=cx+TL.off[0];if(tRow&&tx>=0&&unsigned(tx)<TF.size[0]){const std::size_t pg=tBase+unsigned(tx>>3);if(pg!=tPage){tPage=pg;tv=TF.page[pg];}L=decode(TF,tv,tYZ+(unsigned(tx)&7u));}}
      if(L){if(cx/8!=tileX){tileX=cx/8;tile=cache.find(Cell{tileX,tileY,tileZ});}const bool removed=tile&&((tile->removed[unsigned(cx&7)]>>tileBitIndex)&1u);flags|=removed?CUT:KEPT;
       if((L&64u)||(L&63u)==16)flags|=FILL; // R72: the closure marks its padding (bit 64)
       else if(AF){unsigned A=0;const int ax=cx+AL.off[0];if(aRow&&ax>=0&&unsigned(ax)<AF->size[0]){const std::size_t pg=aBase+unsigned(ax>>3);if(pg!=aPage){aPage=pg;av=AF->page[pg];}A=decode(*AF,av,aYZ+(unsigned(ax)&7u));}
        if(!A)flags|=FILL;}}
      if(own&&L){if((flags&STATE)==KEPT){if((cx>>3)!=pureX){pureX=cx>>3;pureNow=own->pureTile(Cell{cx,cy,cz});}
        if(!pureNow&&!ownIdx.has(cx,cy,cz)&&g.rawMaterialBase(Cell{cx,cy,cz})){flags=std::uint8_t((flags&CYL)|FOREIGN);L=0;}}
       else if(std::abs(s)>oldSlab)flags=std::uint8_t((flags&~STATE)|OLDCUT);}
      if(!imp.head&&(flags&STATE)==CUT&&((L&63u)==2||(L&63u)==21))flags=std::uint8_t((flags&~STATE)|PLUG); // T2
      std::uint8_t l=std::uint8_t(L&63u);if(cyl&&(std::abs(s)>H2||r2>R2e))l|=EDGE;
      lab[i]=l;f[i]=flags;if(L){sLo=std::min(sLo,x);sHi=std::max(sHi,x);}}}
    const auto rr=rowOf(y,z);spanLo[rr]=sLo;spanHi[rr]=sHi;}});
  // the spans grown by 3 cells in x and over the rows within 3 in y and z
  {const int P=3;padLo.assign(R,INT_MAX);padHi.assign(R,INT_MIN);std::vector<std::int32_t> tl(R,INT_MAX),th(R,INT_MIN);
   for(int z=0;z<dim[2];++z)for(int y=0;y<dim[1];++y){std::int32_t a=INT_MAX,b=INT_MIN;for(int q=std::max(0,y-P);q<=std::min(dim[1]-1,y+P);++q){const auto r=rowOf(q,z);a=std::min(a,spanLo[r]);b=std::max(b,spanHi[r]);}tl[rowOf(y,z)]=a;th[rowOf(y,z)]=b;}
   for(int z=0;z<dim[2];++z)for(int y=0;y<dim[1];++y){std::int32_t a=INT_MAX,b=INT_MIN;for(int q=std::max(0,z-P);q<=std::min(dim[2]-1,z+P);++q){const auto r=rowOf(y,q);a=std::min(a,tl[r]);b=std::max(b,th[r]);}
    const auto r=rowOf(y,z);if(a<=b){padLo[r]=std::max(0,a-P);padHi[r]=std::min(dim[0]-1,b+P);}}}
  rep.fillMs=core::ms(t0);}
 // T2: a removed bone cell stays a PLUG only inside a narrow gap of the bone: kept bone within 4 cells on both sides along some
 // axis (an older bore through the bone, up to ~8 mm wide); a wider cavity (a blast that took the bone) stays the cavity, never capped
 void plugs(){
  if(imp.head)return;const int nx=dim[0],ny=dim[1],nz=dim[2];const std::size_t sx=std::size_t(nx),sxy=sx*std::size_t(ny);
  auto keptBone=[&](std::size_t j){const unsigned L=lab[j]&63u;return (f[j]&STATE)==KEPT&&(L==2u||L==21u);};
  std::vector<std::vector<std::size_t>> back(ttWidth());
  ttParallel(std::size_t(nz),2,[&](std::size_t z0,std::size_t z1,std::size_t chunk){auto&B=back[chunk];
   for(int z=int(z0);z<int(z1);++z)for(int y=0;y<ny;++y){const auto r=rowOf(y,z);if(spanLo[r]>spanHi[r])continue;std::size_t i=idx(spanLo[r],y,z);
    for(int x=spanLo[r];x<=spanHi[r];++x,++i){if((f[i]&STATE)!=PLUG)continue;const int c[3]={x,y,z},n3[3]={nx,ny,nz};const std::size_t st[3]={1,sx,sxy};bool narrow=false;
     for(int ax=0;ax<3&&!narrow;++ax){bool lo=false,hi=false;
      for(int k=1;k<=4&&!lo&&c[ax]-k>=0;++k)lo=keptBone(i-std::size_t(k)*st[ax]);
      for(int k=1;k<=4&&!hi&&c[ax]+k<n3[ax];++k)hi=keptBone(i+std::size_t(k)*st[ax]);narrow=lo&&hi;}
     if(!narrow)B.push_back(i);}}});
  for(const auto&B:back)for(const auto i:B)f[i]=std::uint8_t((f[i]&~STATE)|CUT);}
 template<class Pass,class Enter> void flood(std::vector<std::size_t>&stack,Pass&&pass,Enter&&enter){
  const std::size_t sx=std::size_t(dim[0]),sxy=sx*std::size_t(dim[1]);
  while(!stack.empty()){const std::size_t i=stack.back();stack.pop_back();if(!pass(i))continue;int x,y,z;xyz(i,x,y,z);const std::size_t row=i-std::size_t(x);
   int a=x,b=x;while(a>0&&pass(row+std::size_t(a-1)))--a;while(b+1<dim[0]&&pass(row+std::size_t(b+1)))++b;
   for(int k=a;k<=b;++k)enter(row+std::size_t(k));
   auto scan=[&](std::size_t base){bool in=false;for(int k=a;k<=b;++k){const bool p=pass(base+std::size_t(k));if(p&&!in)stack.push_back(base+std::size_t(k));in=p;}};
   if(y>0)scan(row-sx);if(y+1<dim[1])scan(row+sx);if(z>0)scan(row-sxy);if(z+1<dim[2])scan(row+sxy);}}
 // ---- 3. the limb: tissue (not filler) in the cylinder connected to its core (<= 2.5 cm off the axis), then the filler
 // touching it. The filler is not structure: it never joins the two sides, it pulls back with the cloth. The flood runs on the
 // 2 mm cells within the tear's reach (a 2 mm cell is tissue when one of its cells is).
 void region(){
  const auto t0=core::Clock::now();const int nx=cdim[0],ny=cdim[1],nz=cdim[2];
  enum:std::uint8_t{T=1,FL=2,CORE=4,RT=8,RF=16};Raw<std::uint8_t> cf(NC); // R64: every 2 mm cell is written below
  struct Ext {double lo[classes][2],hi[classes][2];bool any[classes];};std::vector<Ext> exts(ttWidth());
  for(auto&E0:exts)for(unsigned c=0;c<classes;++c){E0.any[c]=false;E0.lo[c][0]=E0.lo[c][1]=1e30;E0.hi[c][0]=E0.hi[c][1]=-1e30;}
  const double coreReach=.025+.8660254037844387*pitch+1e-6; // no sub-cell of a 2 mm cell farther than this from the axis is CORE
  if(!imp.head){secO=-(Rl+2*pitch);secN=int(std::ceil(2*(Rl+2*pitch)/(2*pitch)))+1;const std::size_t NS=std::size_t(secN)*std::size_t(secN); // T2
   secCls.reset(new std::atomic<std::uint32_t>[NS]);for(std::size_t q=0;q<NS;++q)secCls[q].store(0,std::memory_order_relaxed);}
  ttParallel(std::size_t(nz),2,[&](std::size_t z0,std::size_t z1,std::size_t chunk){Ext E;
   for(unsigned c=0;c<classes;++c){E.any[c]=false;E.lo[c][0]=E.lo[c][1]=1e30;E.hi[c][0]=E.hi[c][1]=-1e30;}
   for(int cz=int(z0);cz<int(z1);++cz)for(int cy=0;cy<ny;++cy)for(int cx=0;cx<nx;++cx){std::uint8_t v=0;bool centre=false,nearAxis=true;Vec dc{0,0,0};std::uint64_t seenCls=0;double scen=1e9; // T2: the classes seen, the plane offset
    if(double(cdist[cidx(cx,cy,cz)])>farReach){cf[cidx(cx,cy,cz)]=0;continue;} // speed: all eight cells are FAR (skipped below one by one before)
    for(int q=0;q<8;++q){const int x=2*cx+(q&1),y=2*cy+((q>>1)&1),z=2*cz+((q>>2)&1);if(x>=dim[0]||y>=dim[1]||z>=dim[2])continue;const std::size_t i=idx(x,y,z);
     if((lab[i]&FAR)||!(f[i]&CYL)||(f[i]&STATE)==AIR||(f[i]&STATE)==FOREIGN)continue;
     if(!centre){centre=true;const Vec pc=g.center(cell(2*cx,2*cy,2*cz))+Vec{.5,.5,.5}*pitch;double sc,rc2;inCyl(pc,sc,rc2);
      nearAxis=std::sqrt(std::max(0.,rc2))<=coreReach;dc=pc-C;scen=sc;}
     seenCls|=std::uint64_t(1)<<classOf(lab[i]&63u,(f[i]&FILL)!=0);
     if(f[i]&FILL)v|=FL;else{v|=T;if(nearAxis){double s,r2;inCyl(g.center(cell(x,y,z)),s,r2);if(r2<=.025*.025)v|=CORE;}}}
    for(std::uint64_t m=seenCls;m;m&=m-1){const unsigned cls=core::lowestBit(m);E.any[cls]=true;const double ca=dot(dc,fe1[cls]),cb=dot(dc,fe2[cls]); // T2: in the class's frame
     E.lo[cls][0]=std::min(E.lo[cls][0],ca);E.hi[cls][0]=std::max(E.hi[cls][0],ca);E.lo[cls][1]=std::min(E.lo[cls][1],cb);E.hi[cls][1]=std::max(E.hi[cls][1],cb);}
    if(secCls&&seenCls&&std::abs(scen)<=.008){const int ia=int(std::floor((dot(dc,u)-secO)/(2*pitch))),jb=int(std::floor((dot(dc,w)-secO)/(2*pitch))); // T2: the section (8 mm slab)
     if(ia>=0&&jb>=0&&ia<secN&&jb<secN)secCls[std::size_t(jb)*std::size_t(secN)+std::size_t(ia)].fetch_or(std::uint32_t(seenCls),std::memory_order_relaxed);}
    cf[cidx(cx,cy,cz)]=v;}
   exts[chunk]=E;});
  for(unsigned c=0;c<classes;++c){extAny[c]=false;extLo[c][0]=extLo[c][1]=1e30;extHi[c][0]=extHi[c][1]=-1e30;
   for(std::size_t k=0;k<exts.size();++k){if(!exts[k].any[c])continue;extAny[c]=true;for(int a=0;a<2;++a){extLo[c][a]=std::min(extLo[c][a],exts[k].lo[c][a]);extHi[c][a]=std::max(extHi[c][a],exts[k].hi[c][a]);}}}
  // scan-line flood: a popped (marked) cell grows its run along x, then each passable run of the four neighbour rows under that
  // run gets its first cell marked and stacked (6-connected, the same cells as a cell-by-cell flood)
  auto flood=[&](std::uint8_t pass,std::uint8_t mark,std::vector<std::size_t>&stack){
   const std::size_t sx=std::size_t(nx),sxy=sx*std::size_t(ny);auto ok=[&](std::size_t j){return !(cf[j]&(RT|RF))&&(cf[j]&pass);};
   while(!stack.empty()){const std::size_t i=stack.back();stack.pop_back();const int cx=int(i%sx),cy=int((i/sx)%std::size_t(ny)),cz=int(i/sxy);const std::size_t row=i-std::size_t(cx);
    int a=cx,b=cx;while(a>0&&ok(row+std::size_t(a-1))){--a;cf[row+std::size_t(a)]|=mark;}while(b+1<nx&&ok(row+std::size_t(b+1))){++b;cf[row+std::size_t(b)]|=mark;}
    auto scan=[&](std::size_t base){bool in=false;for(int k=a;k<=b;++k){const std::size_t j=base+std::size_t(k);const bool q=ok(j);if(q&&!in){cf[j]|=mark;stack.push_back(j);}in=q;}};
    if(cy>0)scan(row-sx);if(cy+1<ny)scan(row+sx);if(cz>0)scan(row-sxy);if(cz+1<nz)scan(row+sxy);}};
  dbgMs[0]=core::ms(t0);const auto tf=core::Clock::now();
  (void)flood; // R63: the same marks, labelled side by side (runComponents)
  runComponents(nx,ny,nz,[&](std::size_t j){return (cf[j]&T)!=0;},[&](std::size_t j,int,int,int){return (cf[j]&CORE)!=0;},[&](std::size_t j){cf[j]|=RT;});
  {const std::size_t sx=std::size_t(nx),sxy=sx*std::size_t(ny);
   runComponents(nx,ny,nz,[&](std::size_t j){return (cf[j]&FL)&&!(cf[j]&RT);},
    [&](std::size_t j,int x,int y,int z){return (x>0&&(cf[j-1]&RT))||(x+1<nx&&(cf[j+1]&RT))||(y>0&&(cf[j-sx]&RT))||(y+1<ny&&(cf[j+sx]&RT))||(z>0&&(cf[j-sxy]&RT))||(z+1<nz&&(cf[j+sxy]&RT));},
    [&](std::size_t j){cf[j]|=RF;});}
  dbgMs[1]=core::ms(tf);
  std::vector<std::size_t> counts(ttWidth(),0);std::vector<std::array<bool,classes>> seen(ttWidth(),std::array<bool,classes>{});
  ttParallel(std::size_t(dim[2]),2,[&](std::size_t z0,std::size_t z1,std::size_t chunk){
   std::size_t nRegion=0;std::array<bool,classes> seenHere{};
   for(int z=int(z0);z<int(z1);++z)for(int y=0;y<dim[1];++y){const auto rr=rowOf(y,z);for(int x=spanLo[rr];x<=spanHi[rr];++x){const std::size_t i=idx(x,y,z);const auto v=f[i];
    if((lab[i]&FAR)||!(v&CYL)||(v&STATE)==AIR||(v&STATE)==FOREIGN)continue;const auto c=cf[cof(x,y,z)];
    if((v&FILL)?(c&(RT|RF)):(c&RT)){f[i]|=REGION;++nRegion;if((v&STATE)==KEPT||(v&STATE)==PLUG)seenHere[classOf(lab[i]&63u,(v&FILL)!=0)]=true;}}} // T2: + PLUG (its class needs a map)
   counts[chunk]=nRegion;seen[chunk]=seenHere;});
  for(auto c:counts)rep.region+=c;for(const auto&q:seen)for(unsigned k=0;k<classes;++k)present[k]=present[k]||q[k];
  rep.regionMs=core::ms(t0);}
 // ---- 4b. T2: the cross-section at the plane (2 mm cells): muscle next to the bone retracts more (the bone stands several cm out
 // of it: up to +14 mm, fading over 3 cm), and each muscle group domes (+6 mm at its border fading over 12 mm: the groups read as
 // separate masses with grooves between them, not panels with straight steps). Chamfer distances on the section; muscles only.
 void section(){
  if(!secCls)return;const int S=secN;const std::size_t NS=std::size_t(S)*std::size_t(S);const float cell=float(2*pitch);
  const std::uint32_t boneBits=(1u<<bone)|(1u<<marrow),musBits=(0xFFu<<mv0)|(1u<<muscle);
  std::vector<float> dB(NS,1e9f),dM(NS,1e9f);std::vector<signed char> grp(NS,-2); // -2 empty, -1 not one muscle group, else the group's class
  for(std::size_t q=0;q<NS;++q){const std::uint32_t m=secCls[q].load(std::memory_order_relaxed);if(!m)continue;if(m&boneBits)dB[q]=0;const std::uint32_t mm=m&musBits;
   grp[q]=mm&&!(mm&(mm-1))?static_cast<signed char>(core::lowestBit(mm)):static_cast<signed char>(-1);if(mm&&(mm&(mm-1)))dM[q]=0;}
  for(int y=0;y<S;++y)for(int x=0;x<S;++x){const std::size_t q=std::size_t(y)*S+std::size_t(x);if(grp[q]<0)continue;
   const int nb[4][2]={{x-1,y},{x+1,y},{x,y-1},{x,y+1}};for(const auto&e:nb){const signed char o=e[0]<0||e[1]<0||e[0]>=S||e[1]>=S?static_cast<signed char>(-2):grp[std::size_t(e[1])*S+std::size_t(e[0])];if(o!=grp[q]){dM[q]=0;break;}}}
  auto chamfer=[&](std::vector<float>&d){const float a=cell,b=cell*1.4142136f;
   for(int y=0;y<S;++y)for(int x=0;x<S;++x){float&v=d[std::size_t(y)*S+std::size_t(x)];if(x>0)v=std::min(v,d[std::size_t(y)*S+std::size_t(x-1)]+a);
    if(y>0){const std::size_t r=std::size_t(y-1)*S;v=std::min(v,d[r+std::size_t(x)]+a);if(x>0)v=std::min(v,d[r+std::size_t(x-1)]+b);if(x+1<S)v=std::min(v,d[r+std::size_t(x+1)]+b);}}
   for(int y=S-1;y>=0;--y)for(int x=S-1;x>=0;--x){float&v=d[std::size_t(y)*S+std::size_t(x)];if(x+1<S)v=std::min(v,d[std::size_t(y)*S+std::size_t(x+1)]+a);
    if(y+1<S){const std::size_t r=std::size_t(y+1)*S;v=std::min(v,d[r+std::size_t(x)]+a);if(x+1<S)v=std::min(v,d[r+std::size_t(x+1)]+b);if(x>0)v=std::min(v,d[r+std::size_t(x-1)]+b);}}};
  chamfer(dB);chamfer(dM);
  auto fade=[](double t){if(t>=1)return 0.;if(t<=0)return 1.;return 1-t*t*(3-2*t);};
  secExtra.assign(NS,0.f);for(std::size_t q=0;q<NS;++q)secExtra[q]=float(limbK::boneSide*fade(dB[q]/limbK::boneSideFade)+limbK::dome*fade(dM[q]/limbK::domeFade));
  secFoot.assign(NS,0);for(std::size_t q=0;q<NS;++q)secFoot[q]=dB[q]<=1.01f*cell?1:0;} // the bone's footprint in the section, one cell wide around it
 // ---- 5. the tear: limb tissue within its tissue's depth of the damage goes (depths are 2D maps over the cross-section, one
 // per tissue class and side of the plane)
 void tearByDepth(){
  const auto t0=core::Clock::now();
  // T2: one map frame per class: the cross-section plane (u, w) as before (same origin, windows and points), the muscle groups
  // across their own fibre direction (fe1, fe2), so a bundle runs along its fibres. Each map holds only its class's window.
  struct Frame {double amin=0,bmin=0;int ma=1,mb=1;std::vector<double> ax,bx;};std::vector<Frame> frm(classes);
  std::vector<double> dxs(static_cast<std::size_t>(dim[0]));for(int x=0;x<dim[0];++x)dxs[std::size_t(x)]=(g.origin.x+(double(lo[0]+x)+.5)*pitch)-C.x;
  for(unsigned c=0;c<classes;++c){auto&F=frm[c];const Vec e1=fe1[c],e2=fe2[c];const bool plane=e1.x==u.x&&e1.y==u.y&&e1.z==u.z&&e2.x==w.x&&e2.y==w.y&&e2.z==w.z;
   double amin=1e30,amax=-1e30,bmin=1e30,bmax=-1e30;
   for(int q=0;q<8;++q){const Vec p=g.center(cell((q&1)?dim[0]-1:0,(q&2)?dim[1]-1:0,(q&4)?dim[2]-1:0))-C;const double a=dot(p,e1),b=dot(p,e2);
    amin=std::min(amin,a);amax=std::max(amax,a);bmin=std::min(bmin,b);bmax=std::max(bmax,b);}
   if(plane){amin=std::max(amin-pitch,-Rl-pitch);amax=std::min(amax+pitch,Rl+pitch);bmin=std::max(bmin-pitch,-Rl-pitch);bmax=std::min(bmax+pitch,Rl+pitch);}
   else{amin-=pitch;amax+=pitch;bmin-=pitch;bmax+=pitch;}
   F.amin=amin;F.bmin=bmin;F.ma=std::max(1,int(std::ceil((amax-amin)/pitch))+1);F.mb=std::max(1,int(std::ceil((bmax-bmin)/pitch))+1);
   if(present[c]){F.ax.resize(dxs.size());F.bx.resize(dxs.size());for(std::size_t x=0;x<dxs.size();++x){F.ax[x]=dxs[x]*e1.x;F.bx[x]=dxs[x]*e2.x;}}}
  // R63: each class's map over the extent its cells span (+2 cells; a cell's nearest map point lies within one cell of its 2 mm
  // cell's centre point); a class region() never saw keeps the whole map
  std::vector<std::array<int,4>> win(classes);
  for(unsigned c=0;c<classes;++c){const auto&F=frm[c];win[c]={0,F.ma-1,0,F.mb-1};
   if(extAny[c])win[c]={std::clamp(int(std::floor((extLo[c][0]-F.amin)/pitch))-2,0,F.ma-1),std::clamp(int(std::ceil((extHi[c][0]-F.amin)/pitch))+2,0,F.ma-1),
                        std::clamp(int(std::floor((extLo[c][1]-F.bmin)/pitch))-2,0,F.mb-1),std::clamp(int(std::ceil((extHi[c][1]-F.bmin)/pitch))+2,0,F.mb-1)};}
  std::vector<Raw<float>> maps(classes*2);
  struct Job {unsigned c;int side;};std::vector<Job> jobs;for(unsigned c=0;c<classes;++c)if(present[c])for(int s=0;s<2;++s)jobs.push_back({c,s});
  std::vector<std::size_t> rowAt(jobs.size()+1,0);
  for(std::size_t q=0;q<jobs.size();++q){const auto&W=win[jobs[q].c];maps[jobs[q].c*2+unsigned(jobs[q].side)].resize(std::size_t(W[1]-W[0]+1)*std::size_t(W[3]-W[2]+1));rowAt[q+1]=rowAt[q]+std::size_t(W[3]-W[2]+1);}
  const double inv=1./(pitch*pitch);
  static const bool rowVerify=std::getenv("GFY_TT_VERIFY")!=nullptr;std::atomic<std::size_t> rowMismatch{0}; // T1 test: cached rows == direct
  ttParallel(rowAt.back(),8,[&](std::size_t r0,std::size_t r1,std::size_t){
   std::size_t q=std::size_t(std::upper_bound(rowAt.begin(),rowAt.end(),r0)-rowAt.begin())-1;
   for(std::size_t r=r0;r<r1;++r){while(r>=rowAt[q+1])++q;const auto&j=jobs[q];const auto&W=win[j.c];const auto&F=frm[j.c];const int jb=W[2]+int(r-rowAt[q]);
    float*mr=maps[j.c*2+unsigned(j.side)].data()+std::size_t(jb-W[2])*std::size_t(W[1]-W[0]+1);const double b=F.bmin+jb*pitch;
    DepthRow row; // T1: the row's lattice caches (identical depths)
    for(int ia=W[0];ia<=W[1];++ia){const double d=std::max(.002,depthOf(j.c,seed,j.side,F.amin+ia*pitch,b,impC[j.c],&row)*depthScale);mr[ia-W[0]]=float(d*d*inv);
     if(rowVerify&&std::max(.002,depthOf(j.c,seed,j.side,F.amin+ia*pitch,b,impC[j.c])*depthScale)!=d)rowMismatch.fetch_add(1,std::memory_order_relaxed);}}});
  if(rowVerify)std::cerr<<"GFY_TT_VERIFY depth_row_mismatches="<<rowMismatch.load()<<std::endl;
  dbgMs[6]=core::ms(t0);static const bool verifyWindows=std::getenv("GFY_TT_VERIFY")!=nullptr;std::atomic<std::size_t> outside{0};
  std::vector<std::size_t> counts(ttWidth(),0);
  std::vector<double> sxs(dxs.size()),pax(dxs.size()),pbx(dxs.size());for(std::size_t x=0;x<dxs.size();++x){sxs[x]=dxs[x]*n.x;pax[x]=dxs[x]*u.x;pbx[x]=dxs[x]*w.x;}
  // T2: a soft-tissue cell's extra depth from the section (bilinear on its 2 mm cells), eased into the limit like every depth; all but bone
  // and marrow: a vessel next to the bone retracts with the muscles round it (no pole standing out of the end), and the shell's inner edge
  // sinks with the outer muscles' border dome (no fat collar standing above the muscles)
  const bool sec=!secExtra.empty();const double secCell=2*pitch,capHi=maxD-.0015,capK=.006;
  auto extraAt=[&](double a,double b){const double fa=(a-secO)/secCell-.5,fb=(b-secO)/secCell-.5;const int ia=std::clamp(int(std::floor(fa)),0,secN-2),jb=std::clamp(int(std::floor(fb)),0,secN-2);
   const double ta=std::clamp(fa-ia,0.,1.),tb=std::clamp(fb-jb,0.,1.);const float*r0=secExtra.data()+std::size_t(jb)*std::size_t(secN)+std::size_t(ia),*r1=r0+secN;
   return (r0[0]+(r0[1]-r0[0])*ta)*(1-tb)+(r1[0]+(r1[1]-r1[0])*ta)*tb;};
  std::uint8_t clsTab[2][64];for(unsigned v=0;v<64;++v){clsTab[0][v]=std::uint8_t(classOf(v,false));clsTab[1][v]=std::uint8_t(classOf(v,true));}
  // R68: the per-x interpolation terms of the 2 mm distance row (the expressions the cell loop evaluated per cell)
  std::vector<int> xLo(dxs.size()),xHi(dxs.size());std::vector<double> xT(dxs.size());
  for(int x=0;x<dim[0];++x){const double fx=(x+.5)*.5-.5;const int x0=std::max(0,int(std::floor(fx))),x1=std::min(cdim[0]-1,x0+1);xLo[std::size_t(x)]=x0;xHi[std::size_t(x)]=x1;xT[std::size_t(x)]=std::clamp(fx-x0,0.,1.);}
  // std::lround for v >= 0 exactly (v - floor(v) is exact there); negative v clamps to 0 as before
  auto mapIndex=[](double v,int hi){if(!(v>=0))return 0;const double r=std::floor(v);const int k=int(r)+((v-r)>=.5?1:0);return k<hi?k:hi;};
  auto tearable=[&](std::size_t i){return kept(i)||((f[i]&REGION)&&(f[i]&STATE)==PLUG);}; // T2
  ttParallel(std::size_t(dim[2]),2,[&](std::size_t z0,std::size_t z1,std::size_t chunk){std::vector<double> row(static_cast<std::size_t>(cdim[0]));std::size_t nTorn=0;
   double ayc[classes],azc[classes],byc[classes],bzc[classes];
   for(int z=int(z0);z<int(z1);++z)for(int y=0;y<dim[1];++y){
    // the 2 mm distances blended across y and z for this row (the trilinear weights of distAt), then along x per cell
    const auto rr=rowOf(y,z);if(spanLo[rr]>spanHi[rr])continue;bool any=false;{const std::size_t r0=idx(0,y,z);for(int x=spanLo[rr];x<=spanHi[rr];++x)if(tearable(r0+std::size_t(x))){any=true;break;}}if(!any)continue;
    {const double fy=(y+.5)*.5-.5,fz=(z+.5)*.5-.5;const int y0=std::max(0,int(std::floor(fy))),z0c=std::max(0,int(std::floor(fz)));
     const int y1=std::min(cdim[1]-1,y0+1),z1c=std::min(cdim[2]-1,z0c+1);const double ty=std::clamp(fy-y0,0.,1.),tz=std::clamp(fz-z0c,0.,1.);
     for(int cx=0;cx<cdim[0];++cx){const double a00=cdist[cidx(cx,y0,z0c)],a10=cdist[cidx(cx,y1,z0c)],a01=cdist[cidx(cx,y0,z1c)],a11=cdist[cidx(cx,y1,z1c)];
      const double c0=a00+(a10-a00)*ty,c1=a01+(a11-a01)*ty;row[std::size_t(cx)]=c0+(c1-c0)*tz;}}
    const double dy=(g.origin.y+(double(lo[1]+y)+.5)*pitch)-C.y,dz=(g.origin.z+(double(lo[2]+z)+.5)*pitch)-C.z;
    const double sy=dy*n.y,sz=dz*n.z,pay=dy*u.y+dz*u.z,pby=dy*w.y+dz*w.z;
    for(unsigned c=0;c<classes;++c){ayc[c]=dy*fe1[c].y;azc[c]=dz*fe1[c].z;byc[c]=dy*fe2[c].y;bzc[c]=dz*fe2[c].z;}
    for(int x=spanLo[rr];x<=spanHi[rr];++x){const std::size_t i=idx(x,y,z);if(!tearable(i))continue;
     const int x0=xLo[std::size_t(x)],x1=xHi[std::size_t(x)];const double tx=xT[std::size_t(x)];
     const double dd=row[std::size_t(x0)]+(row[std::size_t(x1)]-row[std::size_t(x0)])*tx;if(dd>=maxD)continue;
    const unsigned cls=clsTab[(f[i]&FILL)?1:0][lab[i]&63u];const auto&F=frm[cls];
    const double a=(F.ax[std::size_t(x)]+ayc[cls])+azc[cls],b=(F.bx[std::size_t(x)]+byc[cls])+bzc[cls],s=sxs[std::size_t(x)]+sy+sz;
    const int ia=mapIndex((a-F.amin)/pitch,F.ma-1),jb=mapIndex((b-F.bmin)/pitch,F.mb-1);
    const auto&W=win[cls];const bool inW=ia>=W[0]&&ia<=W[1]&&jb>=W[2]&&jb<=W[3];
    if(verifyWindows&&!inW)outside.fetch_add(1,std::memory_order_relaxed);
    const auto&m=maps[cls*2+(s>=0?1u:0u)];
    const double mv=inW?double(m[std::size_t(jb-W[2])*std::size_t(W[1]-W[0]+1)+std::size_t(ia-W[0])]):0.;
    bool tear=dd*dd*inv<mv;
    if(!tear&&sec&&cls!=bone&&cls!=marrow){const double ex=extraAt(pax[std::size_t(x)]+pay,pbx[std::size_t(x)]+pby);if(ex>0){double e=std::sqrt(mv)*pitch+ex; // T2: the section's extra depth
     if(e>capHi-capK)e=capHi-capK*std::exp(-(e-capHi+capK)/capK);tear=dd<e;}}
    // T2: connective tissue in the bone's footprint (intervertebral discs, ligaments, the cord in its canal) breaks with the bone: the
    // spine stands out of a neck stump as one column (torn by its own depth it cut the vertebrae apart and islands() dropped them)
    if(ripR>0&&sec&&!secFoot.empty()&&present[bone]&&(cls==fascia||cls==other||cls==small||cls==vessel||cls==organ)){
     const double pa=pax[std::size_t(x)]+pay,pb=pbx[std::size_t(x)]+pby;const int qa=std::clamp(int(std::floor((pa-secO)/secCell)),0,secN-1),qb=std::clamp(int(std::floor((pb-secO)/secCell)),0,secN-1);
     if(secFoot[std::size_t(qb)*std::size_t(secN)+std::size_t(qa)]){const auto&FB=frm[bone];const auto&WB=win[bone];const int ia2=mapIndex((pa-FB.amin)/pitch,FB.ma-1),jb2=mapIndex((pb-FB.bmin)/pitch,FB.mb-1);
      if(ia2>=WB[0]&&ia2<=WB[1]&&jb2>=WB[2]&&jb2<=WB[3]){const auto&mb=maps[bone*2+(s>=0?1u:0u)];const double mvb=double(mb[std::size_t(jb2-WB[2])*std::size_t(WB[1]-WB[0]+1)+std::size_t(ia2-WB[0])]);
       const double ss=std::max(0.,std::abs(s)-ripR);tear=ss*ss*inv<mvb;}}}
    if(ripR>0&&!imp.head&&(cls==bone||cls==marrow)){const double ss=std::max(0.,std::abs(s)-ripR);tear=ss*ss*inv<mv;} // T2: the bone breaks across the rip slab only (T1: also by its distance to the damage): a cavity or bore that reaches along the bone does not sink it
    if(tear){if((f[i]&STATE)==PLUG)f[i]=std::uint8_t((f[i]&~STATE)|CUT);else{f[i]=std::uint8_t((f[i]&~STATE)|TORN);++nTorn;markRow(x,y,z);}}}} // T2: a reached plug is the cavity again
   counts[chunk]=nTorn;});
  if(verifyWindows&&outside.load())std::cerr<<"GFY_TT_VERIFY map_window_misses="<<outside.load()<<std::endl;
  for(auto c:counts)rep.torn+=c;
  rep.tearMs=core::ms(t0);}
 // ---- 6. no shreds: kept tissue thinner than ~5 mm (gone under an opening of radius 2) within 3 cells of the cavity
 void thin(){
  const auto t0=core::Clock::now();const int nx=dim[0],ny=dim[1],nz=dim[2];
  Bits K,Cv;K.init(nx,ny,nz);Cv.init(nx,ny,nz);
  ttParallel(std::size_t(nz),2,[&](std::size_t z0,std::size_t z1,std::size_t){
   for(int z=int(z0);z<int(z1);++z)for(int y=0;y<ny;++y){const auto r=rowOf(y,z);if(spanLo[r]>spanHi[r])continue;std::uint64_t*kr=K.row(y,z),*cr=Cv.row(y,z);
    std::size_t i=idx(spanLo[r],y,z);int word=spanLo[r]>>6;std::uint64_t kw=0,cw=0;
    for(int x=spanLo[r];x<=spanHi[r];++x,++i){if((x>>6)!=word){kr[word]|=kw;cr[word]|=cw;kw=cw=0;word=x>>6;}const auto s=f[i]&STATE;const std::uint64_t bit=std::uint64_t(1)<<(x&63);
     if(s==KEPT)kw|=bit;if((f[i]&REGION)&&(s==CUT||s==TORN))cw|=bit;}
    kr[word]|=kw;cr[word]|=cw;}});
  Bits a,b;step(K,a,true,nullptr);step(a,b,true,nullptr);step(b,a,false,&K);step(a,b,false,&K); // b = opened kept tissue
  Bits c1,c2;step(Cv,c1,false,nullptr);step(c1,c2,false,nullptr);step(c2,c1,false,nullptr); // c1 = within 3 cells of the cavity
  std::vector<std::size_t> counts(ttWidth(),0);
  ttParallel(std::size_t(dim[2]),2,[&](std::size_t z0,std::size_t z1,std::size_t chunk){std::size_t nThin=0;
   for(int z=int(z0);z<int(z1);++z)for(int y=0;y<dim[1];++y){const auto r=rowOf(y,z);if(spanLo[r]>spanHi[r])continue;std::size_t i=idx(spanLo[r],y,z);
    for(int x=spanLo[r];x<=spanHi[r];++x,++i){if(!kept(i)||b.get(x,y,z)||!c1.get(x,y,z))continue;
     if(!imp.head&&!(f[i]&FILL)&&((lab[i]&63u)==2||(lab[i]&63u)==21))continue; // T1: the bone's ring and splinters stay (thin by nature)
     f[i]=std::uint8_t((f[i]&~STATE)|TORN);++nThin;markRow(x,y,z);}}
   counts[chunk]=nThin;});
  for(auto c:counts)rep.thin+=c;rep.thinMs=core::ms(t0);}
 // ---- 7. nothing floats: kept limb tissue that reaches neither the box boundary nor the cylinder's (the body beyond) goes;
 // the filler stays only where it touches tissue that stays (or the boundary)
 void islands(){
  const auto t0=core::Clock::now();const int nx=dim[0],ny=dim[1],nz=dim[2];const std::size_t sx=std::size_t(nx),sxy=sx*std::size_t(ny);
  auto farAt=[&](std::size_t j){return (lab[j]&FAR)!=0;};
  auto boundary=[&](int x,int y,int z,std::size_t i){return x==0||y==0||z==0||x+1==nx||y+1==ny||z+1==nz||(lab[i]&EDGE)||
   farAt(i-1)||farAt(i+1)||farAt(i-sx)||farAt(i+sx)||farAt(i-sxy)||farAt(i+sxy);};
  // R63: the floods as connected components of x runs (6-connected, one domain each: kept tissue, kept filler), labelled side by
  // side with a lock-free union-find; a component is reached when one of its cells is a flood seed - exactly the flooded cells.
  struct Run {std::int32_t x0,x1;std::uint32_t row;std::uint8_t domain,seed;};
  const std::size_t R=std::size_t(ny)*std::size_t(nz);std::vector<std::vector<Run>> parts(ttWidth());std::vector<std::uint32_t> rowCount(R,0);
  ttParallel(std::size_t(nz),2,[&](std::size_t z0,std::size_t z1,std::size_t chunk){auto&P=parts[chunk];
   for(int z=int(z0);z<int(z1);++z)for(int y=0;y<ny;++y){const auto r=rowOf(y,z);if(spanLo[r]>spanHi[r])continue;std::uint32_t cnt=0;
    int x=spanLo[r];std::size_t i=idx(x,y,z);
    while(x<=spanHi[r]){if(!kept(i)){++x;++i;continue;}const std::uint8_t d=(f[i]&FILL)?1:0;Run run{x,x,std::uint32_t(r),d,0};
     while(x<=spanHi[r]&&kept(i)&&std::uint8_t((f[i]&FILL)?1:0)==d){if(!run.seed&&boundary(x,y,z,i))run.seed=1;run.x1=x;++x;++i;}
     P.push_back(run);++cnt;}
    rowCount[r]=cnt;}});
  std::vector<std::uint32_t> rowFirst(R+1,0);for(std::size_t r=0;r<R;++r)rowFirst[r+1]=rowFirst[r]+rowCount[r];
  std::vector<Run> runs;runs.reserve(rowFirst[R]);for(auto&P:parts){runs.insert(runs.end(),P.begin(),P.end());std::vector<Run>().swap(P);}
  if(runs.size()!=rowFirst[R])throw std::runtime_error("tissue_tear_island_runs");
  const std::uint32_t NR=std::uint32_t(runs.size());
  std::unique_ptr<std::atomic<std::uint32_t>[]> par(new std::atomic<std::uint32_t>[NR?NR:1]);std::unique_ptr<std::atomic<std::uint8_t>[]> hit(new std::atomic<std::uint8_t>[NR?NR:1]);
  ttParallel(NR,1u<<14,[&](std::size_t k0,std::size_t k1,std::size_t){for(std::size_t k=k0;k<k1;++k){par[k].store(std::uint32_t(k),std::memory_order_relaxed);hit[k].store(0,std::memory_order_relaxed);}});
  auto find=[&](std::uint32_t a){for(;;){std::uint32_t q=par[a].load(std::memory_order_relaxed);if(q==a)return a;const std::uint32_t g2=par[q].load(std::memory_order_relaxed);
    if(g2!=q)par[a].compare_exchange_weak(q,g2,std::memory_order_relaxed);a=g2;}};
  auto unite=[&](std::uint32_t a,std::uint32_t b){for(;;){a=find(a);b=find(b);if(a==b)return;if(a<b)std::swap(a,b);std::uint32_t e=a;
    if(par[a].compare_exchange_strong(e,b,std::memory_order_relaxed))return;}};
  auto link=[&](std::size_t r,std::size_t q){std::uint32_t a=rowFirst[r],b=rowFirst[q];const std::uint32_t ae=rowFirst[r+1],be=rowFirst[q+1];
   while(a<ae&&b<be){const Run&A=runs[a];const Run&B=runs[b];if(A.x1<B.x0){++a;continue;}if(B.x1<A.x0){++b;continue;}
    if(A.domain==B.domain)unite(a,b);if(A.x1<B.x1)++a;else ++b;}};
  ttParallel(std::size_t(nz),2,[&](std::size_t z0,std::size_t z1,std::size_t){
   for(int z=int(z0);z<int(z1);++z)for(int y=0;y<ny;++y){const auto r=rowOf(y,z);if(rowFirst[r]==rowFirst[r+1])continue;
    if(y>0)link(r,rowOf(y-1,z));if(z>0)link(r,rowOf(y,z-1));}});
  auto markRun=[&](const Run&run){const int y=int(run.row%std::uint32_t(ny)),z=int(run.row/std::uint32_t(ny));std::size_t i=idx(run.x0,y,z);
   for(int x=run.x0;x<=run.x1;++x,++i)f[i]|=MARK;};
  auto reach=[&](std::uint8_t d){
   ttParallel(NR,4096,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t k=a;k<b;++k)if(runs[k].domain==d&&runs[k].seed)hit[find(std::uint32_t(k))].store(1,std::memory_order_relaxed);});
   ttParallel(NR,4096,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t k=a;k<b;++k)if(runs[k].domain==d&&hit[find(std::uint32_t(k))].load(std::memory_order_relaxed))markRun(runs[k]);});};
  // T1: on a piece's grid a kept tissue part of 8 cm3 or more inside the box is the piece's own far side (a short piece torn
  // through), not a shred: it stays as if it reached the boundary. Head fractures (keepIslands: whole skull pieces stay) keep every
  // part of 1.5 cm3 or more; smaller crumbs go (the split drops them anyway, and their torn surfaces were left floating)
  const double bigM3=keepIslands?1.5e-6:(own?8e-6:0.);
  if(bigM3>0&&NR){const double bigCells=bigM3/(pitch*pitch*pitch);std::unique_ptr<std::atomic<std::uint64_t>[]> size(new std::atomic<std::uint64_t>[NR]);
   ttParallel(NR,1u<<14,[&](std::size_t k0,std::size_t k1,std::size_t){for(std::size_t k=k0;k<k1;++k)size[k].store(0,std::memory_order_relaxed);});
   ttParallel(NR,4096,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t k=a;k<b;++k)if(runs[k].domain==0)size[find(std::uint32_t(k))].fetch_add(std::uint64_t(runs[k].x1-runs[k].x0+1),std::memory_order_relaxed);});
   ttParallel(NR,4096,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t k=a;k<b;++k)if(runs[k].domain==0&&!runs[k].seed&&double(size[find(std::uint32_t(k))].load(std::memory_order_relaxed))>=bigCells)runs[k].seed=1;});}
  reach(0);
  // the filler is reached from the boundary or from kept tissue that was reached (one marked 6-neighbour)
  ttParallel(NR,1024,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t k=a;k<b;++k){auto&run=runs[k];if(run.domain!=1||run.seed)continue;
   const int y=int(run.row%std::uint32_t(ny)),z=int(run.row/std::uint32_t(ny));std::size_t i=idx(run.x0,y,z);
   for(int x=run.x0;x<=run.x1;++x,++i)if((x>0&&(f[i-1]&MARK))||(x+1<nx&&(f[i+1]&MARK))||(y>0&&(f[i-sx]&MARK))||(y+1<ny&&(f[i+sx]&MARK))||(z>0&&(f[i-sxy]&MARK))||(z+1<nz&&(f[i+sxy]&MARK))){run.seed=1;break;}}});
  reach(1);
  std::vector<std::size_t> counts(ttWidth(),0);
  ttParallel(std::size_t(dim[2]),2,[&](std::size_t z0,std::size_t z1,std::size_t chunk){std::size_t nLoose=0;
   for(int z=int(z0);z<int(z1);++z)for(int y=0;y<dim[1];++y){const auto r=rowOf(y,z);if(spanLo[r]>spanHi[r])continue;std::size_t i=idx(spanLo[r],y,z);
    for(int x=spanLo[r];x<=spanHi[r];++x,++i){if(kept(i)&&!(f[i]&MARK)){f[i]=std::uint8_t((f[i]&~STATE)|LOOSE);++nLoose;markRow(x,y,z);}f[i]&=std::uint8_t(~MARK);}}
   counts[chunk]=nLoose;});
  for(auto c:counts)rep.loose+=c;rep.islandMs=core::ms(t0);}
 // ---- 8. the air within 3 cells (through air) outside torn tissue, and the erase: torn + loose + that air rim
 void write(){
  const auto t0=core::Clock::now();const int nx=dim[0],ny=dim[1],nz=dim[2];
  {Bits Tn,Kp,Air;Tn.init(nx,ny,nz);Kp.init(nx,ny,nz);Air.init(nx,ny,nz);
   ttParallel(std::size_t(nz),2,[&](std::size_t z0,std::size_t z1,std::size_t){
    for(int z=int(z0);z<int(z1);++z)for(int y=0;y<ny;++y){const auto r=rowOf(y,z);if(padLo[r]>padHi[r])continue;std::uint64_t*tr=Tn.row(y,z),*kr=Kp.row(y,z),*ar=Air.row(y,z);
     std::size_t i=idx(padLo[r],y,z);int word=padLo[r]>>6;std::uint64_t tw=0,kw=0,aw=0;
     for(int x=padLo[r];x<=padHi[r];++x,++i){if((x>>6)!=word){tr[word]|=tw;kr[word]|=kw;ar[word]|=aw;tw=kw=aw=0;word=x>>6;}const auto s=f[i]&STATE;const std::uint64_t bit=std::uint64_t(1)<<(x&63);
      if((f[i]&REGION)&&(s==TORN||s==LOOSE))tw|=bit;else if(s==KEPT)kw|=bit;if(s==AIR&&(f[i]&CYL))aw|=bit;}
     tr[word]|=tw;kr[word]|=kw;ar[word]|=aw;}});
   // levels 1..3 through air from the torn and from the kept tissue; rim = reached from the torn at a level the kept has not
   Bits t[3],k[3],tmp;step(Tn,t[0],false,&Air);step(Kp,k[0],false,&Air);
   auto orInto=[](Bits&a,const Bits&b2){ttParallel(a.w.size(),1u<<14,[&](std::size_t q0,std::size_t q1,std::size_t){for(std::size_t q=q0;q<q1;++q)a.w[q]|=b2.w[q];});};
   for(int L=1;L<3;++L){step(t[L-1],t[L],false,&Air);orInto(t[L],t[L-1]);
    step(k[L-1],k[L],false,&Air);orInto(k[L],k[L-1]);}
   dbgMs[3]=core::ms(t0);
   tmp.shape(nx,ny,nz);ttParallel(tmp.w.size(),1u<<14,[&](std::size_t q0,std::size_t q1,std::size_t){for(std::size_t q=q0;q<q1;++q)tmp.w[q]=(t[0].w[q]&~k[0].w[q])|(t[1].w[q]&~k[1].w[q])|(t[2].w[q]&~k[2].w[q]);});
   {std::vector<std::size_t> rims(ttWidth(),0);
    ttParallel(std::size_t(dim[2]),2,[&](std::size_t z0,std::size_t z1,std::size_t chunk){std::size_t nRim=0;
     for(int z=int(z0);z<int(z1);++z)for(int y=0;y<dim[1];++y){const auto r=rowOf(y,z);if(padLo[r]>padHi[r])continue;std::size_t i=idx(padLo[r],y,z);
      for(int x=padLo[r];x<=padHi[r];++x,++i)if((f[i]&STATE)==AIR&&(f[i]&CYL)&&tmp.get(x,y,z)){f[i]|=RIM;++nRim;markRow(x,y,z);}}
     rims[chunk]=nRim;});
    for(auto c:rims)rep.rim+=c;}
   dbgMs[4]=core::ms(t0);
   // cloth standing off torn flesh: from each torn cell at the surface, the air radially outward from the limb axis, 25 mm at
   // most, up to the first tissue (marks gathered side by side, applied after)
   const std::size_t sx=std::size_t(nx),sxy=sx*std::size_t(ny);const int steps=int(std::ceil(.025/pitch));
   // speed: the marks are bits (one word row per box row), set side by side; then every row applies its own bits side by side.
   // The same cells as the former per-thread lists applied one after the other, each counted once.
   const std::size_t MW=std::size_t((nx+63)/64),MR=std::size_t(ny)*std::size_t(nz);std::unique_ptr<std::atomic<std::uint64_t>[]> markBits(new std::atomic<std::uint64_t>[MW*MR]);
   ttParallel(MW*MR,std::size_t(1)<<14,[&](std::size_t q0,std::size_t q1,std::size_t){for(std::size_t q=q0;q<q1;++q)markBits[q].store(0,std::memory_order_relaxed);});
   // R65: the start cells (torn cells with a 6-neighbour in the cylinder's air) = Tn & dilate(Air), visited in the same order
   Bits D;step(Air,D,false,nullptr);(void)sx;(void)sxy;
   ttParallel(std::size_t(nz),2,[&](std::size_t z0,std::size_t z1,std::size_t){
    for(int z=int(z0);z<int(z1);++z)for(int y=0;y<ny;++y){const std::uint64_t*tr=Tn.row(y,z),*dr=D.row(y,z);for(int w0=0;w0<Tn.W;++w0){std::uint64_t cand=tr[w0]&dr[w0];
     while(cand){const int x=w0*64+int(core::lowestBit(cand));cand&=cand-1;
     const Vec p=g.center(cell(x,y,z));Vec r=(p-limbPoint)-limbAxis*dot(p-limbPoint,limbAxis);const double l=length(r);if(!(l>1e-9))continue;r=r*(1./l);
     for(int k=1;k<=steps;++k){const Vec q=p+r*(k*pitch);const int qx=int(std::floor((q.x-g.origin.x)/pitch))-lo[0],qy=int(std::floor((q.y-g.origin.y)/pitch))-lo[1],qz=int(std::floor((q.z-g.origin.z)/pitch))-lo[2];
      if(qx<0||qy<0||qz<0||qx>=nx||qy>=ny||qz>=nz)break;const std::size_t j=idx(qx,qy,qz);if((f[j]&STATE)!=AIR)break;if(!(f[j]&CYL))break;
      if(!(f[j]&RIM))markBits[(std::size_t(qz)*std::size_t(ny)+std::size_t(qy))*MW+std::size_t(qx>>6)].fetch_or(std::uint64_t(1)<<(qx&63),std::memory_order_relaxed);}}}}});
   {std::vector<std::size_t> radial(ttWidth(),0);
    ttParallel(std::size_t(nz),2,[&](std::size_t z0,std::size_t z1,std::size_t chunk){std::size_t nRim=0;
     for(int z=int(z0);z<int(z1);++z)for(int y=0;y<ny;++y){const auto*row=markBits.get()+(std::size_t(z)*std::size_t(ny)+std::size_t(y))*MW;
      for(std::size_t w0=0;w0<MW;++w0){std::uint64_t bits=row[w0].load(std::memory_order_relaxed);
       while(bits){const int x=int(w0)*64+int(core::lowestBit(bits));bits&=bits-1;const std::size_t j=idx(x,y,z);if(!(f[j]&RIM)){f[j]|=RIM;++nRim;markRow(x,y,z);}}}}
     radial[chunk]=nRim;});
    for(auto c:radial)rep.rim+=c;}}
  dbgMs[5]=core::ms(t0);
  // the erase: tile masks from the marked rows, gathered in grid-tile z slabs side by side (each tile written by one thread), then
  // counted against the stored bits (reads only) and applied in one journaled pass
  const int tlo[3]={lo[0]>>3,lo[1]>>3,lo[2]>>3},thi[3]={(lo[0]+nx-1)>>3,(lo[1]+ny-1)>>3,(lo[2]+nz-1)>>3};
  const int tnx=thi[0]-tlo[0]+1,tny=thi[1]-tlo[1]+1,tnz=thi[2]-tlo[2]+1;const std::size_t TB=std::size_t(tnx)*tny*tnz;
  std::vector<core::Tile> mask(TB);std::vector<std::uint8_t> any(TB,0);
  auto tid=[&](int tx,int ty,int tz){return (std::size_t(tz-tlo[2])*std::size_t(tny)+std::size_t(ty-tlo[1]))*std::size_t(tnx)+std::size_t(tx-tlo[0]);};
  ttParallel(std::size_t(tnz),1,[&](std::size_t a2,std::size_t b2,std::size_t){
   for(int tz=tlo[2]+int(a2);tz<tlo[2]+int(b2);++tz)for(int z=std::max(0,tz*8-lo[2]);z<std::min(nz,tz*8+8-lo[2]);++z)for(int y=0;y<ny;++y){const auto rr=rowOf(y,z);
    for(int x=markLo[rr];x<=markHi[rr];++x){const std::size_t i=idx(x,y,z);const auto st=f[i]&STATE;if(!((f[i]&RIM)||((f[i]&REGION)&&(st==TORN||st==LOOSE))))continue;
     const Cell c=cell(x,y,z);if(!g.valid(c))continue;if(own&&st!=TORN&&st!=LOOSE&&!g.ownerCuttable(c))continue; // T1: a piece's rim: only air it may erase
     const auto t=tid(c.x>>3,c.y>>3,c.z>>3);const auto l=Grid::local(c);mask[t].removed[l/64]|=std::uint64_t(1)<<(l%64);any[t]=1;}}});
  std::vector<std::size_t> live;for(std::size_t t=0;t<TB;++t)if(any[t])live.push_back(t);
  struct Count {std::size_t fresh=0,solid=0;std::array<std::size_t,64> slot{};};std::vector<Count> counts(ttWidth());
  ttParallel(live.size(),64,[&](std::size_t a2,std::size_t b2,std::size_t chunk){auto&K=counts[chunk];
   for(std::size_t q=a2;q<b2;++q){const auto t=live[q];const int tx=int(t%std::size_t(tnx))+tlo[0],ty=int((t/std::size_t(tnx))%std::size_t(tny))+tlo[1],tz=int(t/(std::size_t(tnx)*tny))+tlo[2];
    const core::Tile*old=g.findTile(Cell{tx,ty,tz});
    for(unsigned w2=0;w2<8;++w2){std::uint64_t fresh=mask[t].removed[w2]&~(old?old->removed[w2]:0);K.fresh+=std::bitset<64>(fresh).count();
     while(fresh){const unsigned l=w2*64+core::lowestBit(fresh);fresh&=fresh-1;const Cell c{tx*8+int(l>>6),ty*8+int((l>>3)&7),tz*8+int(l&7)};
      const std::size_t i=idx(c.x-lo[0],c.y-lo[1],c.z-lo[2]);const unsigned L=lab[i]&63u;if(L&&!(f[i]&FILL)){++K.solid;++K.slot[baseSlot(L)];}}}}});
  for(const auto&K:counts){rep.fresh+=K.fresh;rep.solid+=K.solid;for(unsigned s2=0;s2<64;++s2)rep.bySlot[s2]+=K.slot[s2];}
  for(const auto t:live){const int tx=int(t%std::size_t(tnx))+tlo[0],ty=int((t/std::size_t(tnx))%std::size_t(tny))+tlo[1],tz=int(t/(std::size_t(tnx)*tny))+tlo[2];
   const Cell key{tx,ty,tz};auto&tt=core::journaledTile(g,key);for(unsigned q=0;q<8;++q)tt.removed[q]|=mask[t].removed[q];rep.tiles.push_back(key);}
  static const bool hashLog=std::getenv("GFY_TISSUE_TEAR_LOG")!=nullptr; // R63 diagnostics only
  if(hashLog){auto mix=[&](std::uint64_t v){for(int q=0;q<8;++q){dbgMaskHash^=(v>>(8*q))&255u;dbgMaskHash*=1099511628211ull;}};
   for(const auto t:live){const int tx=int(t%std::size_t(tnx))+tlo[0],ty=int((t/std::size_t(tnx))%std::size_t(tny))+tlo[1],tz=int(t/(std::size_t(tnx)*tny))+tlo[2];
    mix(std::uint64_t(std::uint32_t(tx))|(std::uint64_t(std::uint32_t(ty))<<32));mix(std::uint64_t(std::uint32_t(tz)));for(unsigned q=0;q<8;++q)mix(mask[t].removed[q]);}}
  rep.writeMs=core::ms(t0);}
 // ---- 9. the torn surface
 void mesh(){
  const int nx=dim[0],ny=dim[1],nz=dim[2];const int bx=(nx+7)/8,by=(ny+7)/8,bz=(nz+7)/8;const std::size_t NB=std::size_t(bx)*by*bz;
  auto bidx=[&](int x,int y,int z){return (std::size_t(z>>3)*std::size_t(by)+std::size_t(y>>3))*std::size_t(bx)+std::size_t(x>>3);};
  auto active=[&](std::size_t i,std::size_t j){return face(i,j)||face(j,i);};
  const std::size_t sx=std::size_t(nx),sxy=sx*std::size_t(ny);
  // the lattice step (cells): 2 by default (the softened field holds no finer detail), GFY_TT_MESH_STEP=1 for A/B renders
  static const int S=[]{const char*e=std::getenv("GFY_TT_MESH_STEP");return e&&std::atoi(e)==1?1:2;}();
  // active edges (cell, axis to +1) per z-slab
  auto t0=core::Clock::now();
  // R63: the cavity as a bitset (cavity cells are region cells: all within the spans)
  Bits Cav;Cav.init(nx,ny,nz);
  ttParallel(std::size_t(nz),2,[&](std::size_t z0,std::size_t z1,std::size_t){
   for(int z=int(z0);z<int(z1);++z)for(int y=0;y<ny;++y){const auto r=rowOf(y,z);if(spanLo[r]>spanHi[r])continue;std::size_t i=idx(spanLo[r],y,z);
    for(int x=spanLo[r];x<=spanHi[r];++x,++i)if(cavity(i))Cav.set(x,y,z);}});
  // a lattice row can hold an active edge only if it, the row S above in y or the row S above in z has a cavity cell
  std::vector<std::uint8_t> rowCav(std::size_t(ny)*std::size_t(nz),0);
  ttParallel(std::size_t(nz),2,[&](std::size_t z0,std::size_t z1,std::size_t){
   for(int z=int(z0);z<int(z1);++z)for(int y=0;y<ny;++y){const std::uint64_t*w=Cav.row(y,z);std::uint64_t any=0;for(int q=0;q<Cav.W;++q)any|=w[q];rowCav[rowOf(y,z)]=any?1:0;}});
  auto rowMay=[&](int y,int z){return rowCav[rowOf(y,z)]||(y+S<ny&&rowCav[rowOf(y+S,z)])||(z+S<nz&&rowCav[rowOf(y,z+S)]);};
  struct Edge {std::uint32_t x,y,z;std::uint8_t axis;};std::vector<std::vector<Edge>> edges(ttWidth());
  ttParallel(std::size_t(nz),2,[&](std::size_t z0,std::size_t z1,std::size_t chunk){auto&E=edges[chunk];
   for(int z=(int(z0)+S-1)/S*S;z<int(z1);z+=S)for(int y=0;y<ny;y+=S){if(!rowMay(y,z))continue;for(int x=0;x<nx;x+=S){const std::size_t i=idx(x,y,z);const bool k=solid(i),c=cavity(i);if(!k&&!c)continue;
    if(x+S<nx&&active(i,i+std::size_t(S)))E.push_back({std::uint32_t(x),std::uint32_t(y),std::uint32_t(z),0});
    if(y+S<ny&&active(i,i+std::size_t(S)*sx))E.push_back({std::uint32_t(x),std::uint32_t(y),std::uint32_t(z),1});
    if(z+S<nz&&active(i,i+std::size_t(S)*sxy))E.push_back({std::uint32_t(x),std::uint32_t(y),std::uint32_t(z),2});}}});
  dbgMs[8]=core::ms(t0);std::size_t edgeCount=0;for(const auto&E:edges)edgeCount+=E.size();if(!edgeCount)return;
  // blocks within one block of an active edge carry the softened field (Gaussian sigma 1.2 cells, radius 3; outside the box = 1)
  std::vector<std::int32_t> slot(NB,-1);
  {std::vector<std::uint8_t> centre(NB,0); // the blocks holding an edge (each edge's block once), then their 27-neighbourhoods
   for(const auto&E:edges)for(const auto&e:E)centre[bidx(int(e.x),int(e.y),int(e.z))]=1;
   for(int pz=0;pz<bz;++pz)for(int py=0;py<by;++py)for(int px=0;px<bx;++px){if(!centre[(std::size_t(pz)*std::size_t(by)+std::size_t(py))*std::size_t(bx)+std::size_t(px)])continue;
    for(int dz=-1;dz<=1;++dz)for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){const int qx=px+dx,qy=py+dy,qz=pz+dz;if(qx<0||qy<0||qz<0||qx>=bx||qy>=by||qz>=bz)continue;
     slot[(std::size_t(qz)*std::size_t(by)+std::size_t(qy))*std::size_t(bx)+std::size_t(qx)]=0;}}}
  std::vector<std::size_t> blocks;for(std::size_t b=0;b<NB;++b)if(slot[b]==0){slot[b]=std::int32_t(blocks.size());blocks.push_back(b);}
  dbgMs[9]=core::ms(t0);
  Raw<float> soft(blocks.size()*512); // R64: each block zeroes its own slice (below)
  float kw[7];{double s=0;for(int k=-3;k<=3;++k){kw[k+3]=float(std::exp(-.5*k*k/(1.2*1.2)));s+=kw[k+3];}for(auto&v:kw)v=float(v/s);}
  // R64: the x pass reads a 0/1 field (1 = not cavity, also outside the box), so each of its values is the float sum of the kernel
  // weights over the 1-cells of its 7-cell window, added in the same order: a 128-entry table indexed by the window's bits
  float lut[128];for(unsigned m=0;m<128;++m){float s=0;for(int t=0;t<7;++t)s+=kw[t]*float((m>>t)&1u);lut[m]=s;}
  auto window=[&](const std::uint64_t*cw,int X)->unsigned{const int a=X-3; // bit i: cell X-3+i is not cavity (cw null: row outside)
   if(cw&&a>=0&&X+3<nx){const int w0=a>>6,b0=a&63;std::uint64_t v=cw[w0]>>b0;if(b0>57)v|=cw[w0+1]<<(64-b0);return unsigned(~v)&127u;}
   unsigned m=0;for(int i=0;i<7;++i){const int x=a+i;if(!cw||x<0||x>=nx||!((cw[x>>6]>>(x&63))&1u))m|=1u<<i;}return m;};
  ttParallel(blocks.size(),16,[&](std::size_t b0,std::size_t b1,std::size_t){std::vector<float> Q(14*14*14),R(14*14*14);
   for(std::size_t k=b0;k<b1;++k){const std::size_t b=blocks[k];const int ox=int(b%std::size_t(bx))*8,oy=int((b/std::size_t(bx))%std::size_t(by))*8,oz=int(b/(std::size_t(bx)*by))*8;
    auto at=[](int x,int y,int z){return (z*14+y)*14+x;};
    for(int z=0;z<14;++z)for(int y=0;y<14;++y){const int Y=oy+y-3,Z=oz+z-3;const std::uint64_t*cw=(Y<0||Z<0||Y>=ny||Z>=nz)?nullptr:Cav.row(Y,Z);
     for(int x=3;x<11;x+=S)Q[std::size_t(at(x,y,z))]=lut[window(cw,ox+x-3)];}
    for(int z=0;z<14;++z)for(int y=3;y<11;y+=S)for(int x=3;x<11;x+=S){float s=0;for(int t=-3;t<=3;++t)s+=kw[t+3]*Q[std::size_t(at(x,y+t,z))];R[std::size_t(at(x,y,z))]=s;}
    float*out=soft.data()+k*512;std::fill(out,out+512,0.f);
    for(int z=3;z<11;z+=S)for(int y=3;y<11;y+=S)for(int x=3;x<11;x+=S){float s=0;for(int t=-3;t<=3;++t)s+=kw[t+3]*R[std::size_t(at(x,y,z+t))];out[((z-3)*8+(y-3))*8+(x-3)]=s;}}});
  auto G=[&](int x,int y,int z)->float{x=std::clamp(x,0,nx-1);y=std::clamp(y,0,ny-1);z=std::clamp(z,0,nz-1);const auto s=slot[bidx(x,y,z)];
   if(s<0)return cavity(idx(x,y,z))?0.f:1.f;return soft[std::size_t(s)*512+std::size_t(((z&7)*8+(y&7))*8+(x&7))];};
  rep.blurMs=core::ms(t0);dbgMs[15]=double(blocks.size());t0=core::Clock::now();
  // one vertex per dual cell (corners = cell centres x..x+1, ...) that holds an active edge: the mean of its edges' crossings
  // R64: the dual marks per active block (512 bits each); a mark outside every active block is never read (below: active blocks only)
  std::vector<std::uint64_t> dualBits(blocks.size()*8,0);
  auto local=[](int x,int y,int z){return unsigned(((z&7)*8+(y&7))*8+(x&7));};
  auto markDual=[&](int x,int y,int z){if(x<0||y<0||z<0||x+S>=nx||y+S>=ny||z+S>=nz)return;const auto sl=slot[bidx(x,y,z)];if(sl<0)return;
   const unsigned l=local(x,y,z);dualBits[std::size_t(sl)*8+(l>>6)]|=std::uint64_t(1)<<(l&63);};
  for(const auto&E:edges)for(const auto&e:E){const int x=int(e.x),y=int(e.y),z=int(e.z);
   if(e.axis==0){markDual(x,y-S,z-S);markDual(x,y,z-S);markDual(x,y,z);markDual(x,y-S,z);}
   else if(e.axis==1){markDual(x-S,y,z-S);markDual(x-S,y,z);markDual(x,y,z);markDual(x,y,z-S);}
   else{markDual(x-S,y-S,z);markDual(x,y-S,z);markDual(x,y,z);markDual(x-S,y,z);}}
  dbgMs[10]=core::ms(t0);std::vector<std::vector<std::size_t>> duals(blocks.size());std::vector<std::vector<std::array<float,6>>> dverts(blocks.size());
  ttParallel(blocks.size(),16,[&](std::size_t b0,std::size_t b1,std::size_t){
   for(std::size_t k=b0;k<b1;++k){const std::size_t b=blocks[k];const int ox=int(b%std::size_t(bx))*8,oy=int((b/std::size_t(bx))%std::size_t(by))*8,oz=int(b/(std::size_t(bx)*by))*8;
    for(int z=oz;z<std::min(oz+8,nz);z+=S)for(int y=oy;y<std::min(oy+8,ny);y+=S)for(int x=ox;x<std::min(ox+8,nx);x+=S){const std::size_t i=idx(x,y,z);{const unsigned l=local(x,y,z);if(!((dualBits[k*8+(l>>6)]>>(l&63))&1u))continue;}
     double sum[3]={0,0,0};int cnt=0;
     for(int e=0;e<12;++e){const int ax=e/4,a0=e&1,a1=(e>>1)&1;int p0[3];
      if(ax==0){p0[0]=0;p0[1]=a0*S;p0[2]=a1*S;}else if(ax==1){p0[0]=a0*S;p0[1]=0;p0[2]=a1*S;}else{p0[0]=a0*S;p0[1]=a1*S;p0[2]=0;}
      int p1[3]={p0[0],p0[1],p0[2]};p1[ax]+=S;
      const std::size_t i0=idx(x+p0[0],y+p0[1],z+p0[2]),i1=idx(x+p1[0],y+p1[1],z+p1[2]);if(!active(i0,i1))continue;
      const double g0=G(x+p0[0],y+p0[1],z+p0[2]),g1=G(x+p1[0],y+p1[1],z+p1[2]);double t=std::abs(g0-g1)>1e-6?(g0-.5)/(g0-g1):.5;t=std::clamp(t,.05,.95);
      for(int q=0;q<3;++q)sum[q]+=p0[q]+(p1[q]-p0[q])*t;++cnt;}
     if(!cnt)continue;
     // gradient of the softened field over the dual cell (points from the cavity into tissue): the vertex normal is its negative
     float c8[8];for(int q=0;q<8;++q)c8[q]=G(x+(q&1)*S,y+((q>>1)&1)*S,z+((q>>2)&1)*S);
     const double gx=(c8[1]+c8[3]+c8[5]+c8[7])-(c8[0]+c8[2]+c8[4]+c8[6]),gy=(c8[2]+c8[3]+c8[6]+c8[7])-(c8[0]+c8[1]+c8[4]+c8[5]),gz=(c8[4]+c8[5]+c8[6]+c8[7])-(c8[0]+c8[1]+c8[2]+c8[3]);
     const Vec p=g.center(cell(x,y,z))+Vec{sum[0]/cnt,sum[1]/cnt,sum[2]/cnt}*pitch;
     duals[k].push_back(i);dverts[k].push_back({float(p.x),float(p.y),float(p.z),float(gx),float(gy),float(gz)});}}});
  dbgMs[11]=core::ms(t0);std::vector<std::uint32_t> base(blocks.size()+1,0);for(std::size_t k=0;k<blocks.size();++k)base[k+1]=base[k]+std::uint32_t(duals[k].size());
  const std::uint32_t V=base.back();if(!V)return;
  std::vector<float> pos(std::size_t(V)*3),nrm(std::size_t(V)*3);
  // R68: each block writes its own rows (the same expressions as the former serial loop)
  ttParallel(blocks.size(),64,[&](std::size_t k0,std::size_t k1,std::size_t){for(std::size_t k=k0;k<k1;++k)for(std::size_t j=0;j<duals[k].size();++j){const auto&v=dverts[k][j];const std::size_t o=(base[k]+j)*3;
   pos[o]=v[0];pos[o+1]=v[1];pos[o+2]=v[2];const double l=std::sqrt(double(v[3])*v[3]+double(v[4])*v[4]+double(v[5])*v[5]);
   if(l>1e-9){nrm[o]=float(-v[3]/l);nrm[o+1]=float(-v[4]/l);nrm[o+2]=float(-v[5]/l);}else nrm[o]=nrm[o+1]=nrm[o+2]=0.f;}});
  auto vid=[&](int x,int y,int z)->std::int64_t{if(x<0||y<0||z<0||x+S>=nx||y+S>=ny||z+S>=nz)return -1;const auto s=slot[bidx(x,y,z)];if(s<0)return -1;
   const auto&D=duals[std::size_t(s)];const std::size_t i=idx(x,y,z);const auto it=std::lower_bound(D.begin(),D.end(),i);if(it==D.end()||*it!=i)return -1;
   return std::int64_t(base[std::size_t(s)])+std::int64_t(it-D.begin());};
  // quads around the active edges, wound so the face normal points from the cavity into the kept tissue (the cutter convention:
  // outward = into tissue; packWithField flips it for the cavity side)
  dbgMs[12]=core::ms(t0);std::vector<std::vector<std::uint32_t>> tris(edges.size());std::vector<std::vector<std::uint8_t>> labs(edges.size());
  ttParallel(edges.size(),1,[&](std::size_t c0,std::size_t c1,std::size_t){
   for(std::size_t c=c0;c<c1;++c)for(const auto&e:edges[c]){const int x=int(e.x),y=int(e.y),z=int(e.z),ax=e.axis;std::int64_t q[4];
    if(ax==0){q[0]=vid(x,y-S,z-S);q[1]=vid(x,y,z-S);q[2]=vid(x,y,z);q[3]=vid(x,y-S,z);}
    else if(ax==1){q[0]=vid(x-S,y,z-S);q[1]=vid(x-S,y,z);q[2]=vid(x,y,z);q[3]=vid(x,y,z-S);}
    else{q[0]=vid(x-S,y-S,z);q[1]=vid(x,y-S,z);q[2]=vid(x,y,z);q[3]=vid(x-S,y,z);}
    if(q[0]<0||q[1]<0||q[2]<0||q[3]<0)continue;
    const std::size_t i=idx(x,y,z),j=ax==0?i+std::size_t(S):ax==1?i+std::size_t(S)*sx:i+std::size_t(S)*sxy;const bool keptFirst=solid(i);const std::size_t kc=keptFirst?i:j;
    // the face normal must point INTO the kept cell: -axis when the first cell is kept
    const double want=keptFirst?-1.:1.;std::uint32_t v[4]={std::uint32_t(q[0]),std::uint32_t(q[1]),std::uint32_t(q[2]),std::uint32_t(q[3])};
    auto P=[&](std::uint32_t id){return Vec{pos[id*3],pos[id*3+1],pos[id*3+2]};};
    const Vec fn=cross(P(v[2])-P(v[0]),P(v[3])-P(v[1]));if(fn[ax]*want<0)std::swap(v[1],v[3]);
    auto&T=tris[c];T.insert(T.end(),{v[0],v[1],v[2],v[0],v[2],v[3]});labs[c].push_back(std::uint8_t(lab[kc]&63u));labs[c].push_back(std::uint8_t(lab[kc]&63u));}});
  // R68: the chunks' lists joined at prefix offsets (the serial concatenation)
  std::vector<std::size_t> triAt(tris.size()+1,0),labAt(tris.size()+1,0);for(std::size_t c=0;c<tris.size();++c){triAt[c+1]=triAt[c]+tris[c].size();labAt[c+1]=labAt[c]+labs[c].size();}
  std::vector<std::uint32_t> index(triAt.back());std::vector<std::uint8_t> label(labAt.back());
  ttParallel(tris.size(),1,[&](std::size_t c0,std::size_t c1,std::size_t){for(std::size_t c=c0;c<c1;++c){std::copy(tris[c].begin(),tris[c].end(),index.begin()+std::ptrdiff_t(triAt[c]));
   std::copy(labs[c].begin(),labs[c].end(),label.begin()+std::ptrdiff_t(labAt[c]));}});
  rep.rawQuads=label.size()/2;rep.netsMs=core::ms(t0);t0=core::Clock::now();
  if(index.empty())return;
  struct LowPoly {bool on;int cluster;double clusterError,wholeError,share,patch;std::size_t least,most;bool lockBorder;int normals,bulk,points;};
  static const LowPoly lp=[]{auto num=[](const char*name,double fallback){const char*e=std::getenv(name);return e?std::atof(e):fallback;};LowPoly q;
   q.on=num("GFY_TT_LOWPOLY",1)!=0;q.cluster=std::clamp(int(num("GFY_TT_CLUSTER_CELLS",32)),16,512);
   q.clusterError=num("GFY_TISSUE_TEAR_ERROR_MM",q.on?1.:.6)*.001;q.wholeError=q.on?num("GFY_TT_WHOLE_ERROR_MM",2.5)*.001:0.;
   q.share=std::clamp(num("GFY_TT_SHARE",.033),.001,1.);q.least=std::size_t(std::max(4.,num("GFY_TT_LEAST",1500)));q.most=std::size_t(std::max(4.,num("GFY_TT_MOST",20000)));
   q.lockBorder=num("GFY_TT_LOCK_BORDER",0)!=0;q.normals=q.on?std::clamp(int(num("GFY_TT_NORMALS",4)),0,4):0;
   q.patch=q.on&&q.normals>0?std::clamp(num("GFY_TT_PATCH_MM",3.5),0.,10.)*.001:0.;q.bulk=std::clamp(int(num("GFY_TT_BULK_CELLS",5)),2,12);q.points=num("GFY_TT_PATCH_POINTS",19)>7?19:7;return q;}();
  std::vector<std::uint32_t> keep;std::vector<std::uint8_t> keepLabel;
  {const int cs=lp.cluster;const int cx=(nx+cs-1)/cs,cy=(ny+cs-1)/cs,cz=(nz+cs-1)/cs;const std::size_t NC=std::size_t(cx)*cy*cz;
   std::vector<std::vector<std::uint32_t>> ctri(NC);
   // R68: each triangle's cluster side by side, then per-chunk counts and a fill over the same contiguous ranges: every cluster
   // list holds its triangles in ascending order, as the serial loop pushed them
   {const std::uint32_t T=std::uint32_t(index.size()/3);std::vector<std::uint32_t> clusterOf(T);
    ttParallel(T,1u<<12,[&](std::size_t a0,std::size_t a1,std::size_t){for(std::size_t t=a0;t<a1;++t){const std::uint32_t a=index[t*3];
     const int x=std::clamp(int((pos[a*3]-g.origin.x)/pitch)-lo[0],0,nx-1),y=std::clamp(int((pos[a*3+1]-g.origin.y)/pitch)-lo[1],0,ny-1),z=std::clamp(int((pos[a*3+2]-g.origin.z)/pitch)-lo[2],0,nz-1);
     clusterOf[t]=std::uint32_t((std::size_t(z/cs)*std::size_t(cy)+std::size_t(y/cs))*std::size_t(cx)+std::size_t(x/cs));}});
    const std::size_t W=ttWidth();std::vector<std::uint32_t> counts(W*NC,0);std::vector<std::size_t> chunkLo(W,0),chunkHi(W,0);std::vector<char> chunkUsed(W,0);
    ttParallel(T,1u<<12,[&](std::size_t a0,std::size_t a1,std::size_t chunk){chunkLo[chunk]=a0;chunkHi[chunk]=a1;chunkUsed[chunk]=1;auto*c=counts.data()+chunk*NC;for(std::size_t t=a0;t<a1;++t)++c[clusterOf[t]];});
    std::vector<std::uint32_t> at(W*NC,0);
    for(std::size_t k=0;k<NC;++k){std::uint32_t total=0;for(std::size_t w=0;w<W;++w){at[w*NC+k]=total;total+=counts[w*NC+k];}ctri[k].resize(total);}
    ttParallel(W,1,[&](std::size_t w0,std::size_t w1,std::size_t){for(std::size_t w=w0;w<w1;++w){if(!chunkUsed[w])continue;auto*a=at.data()+w*NC;
     for(std::size_t t=chunkLo[w];t<chunkHi[w];++t){const auto k=clusterOf[t];ctri[k][a[k]++]=std::uint32_t(t);}}});}
   std::vector<std::size_t> used;for(std::size_t c=0;c<NC;++c)if(!ctri[c].empty())used.push_back(c);
   std::vector<std::vector<std::uint32_t>> out(used.size());std::vector<std::vector<std::uint8_t>> outLabel(used.size());
   static const double target=lp.clusterError; // speed: 1 mm with the whole-surface pass after it (0.6 mm without)
   ttParallel(used.size(),1,[&](std::size_t u0,std::size_t u1,std::size_t){
    for(std::size_t k=u0;k<u1;++k){const auto&T=ctri[used[k]];std::unordered_map<std::uint32_t,std::uint32_t> local;std::vector<std::uint32_t> back,ix;ix.reserve(T.size()*3);
     for(const auto t:T)for(int q=0;q<3;++q){const std::uint32_t gv=index[t*3+q];auto it=local.find(gv);if(it==local.end()){it=local.emplace(gv,std::uint32_t(back.size())).first;back.push_back(gv);}ix.push_back(it->second);}
     std::vector<float> P(back.size()*3);for(std::size_t v=0;v<back.size();++v)for(int q=0;q<3;++q)P[v*3+q]=pos[back[v]*3+q];
     std::vector<std::uint32_t> dst(ix.size());float err=0;
     const std::size_t m=target>0?::meshopt_simplify(dst.data(),ix.data(),ix.size(),P.data(),back.size(),12,0,float(target),meshopt_SimplifyLockBorder|meshopt_SimplifyErrorAbsolute,&err):ix.size();
     if(target<=0)dst=ix;
     // a kept triangle's label (the shader reads the tissue per pixel; this is its fallback): the majority of its corners',
     // a corner's being the label of the first original triangle at it
     std::vector<std::uint8_t> cornerLabel(back.size(),0);for(std::size_t t=T.size();t-->0;)for(int q=0;q<3;++q)cornerLabel[ix[t*3+q]]=label[T[t]];
     auto&O=out[k];auto&L=outLabel[k];O.reserve(m);L.reserve(m/3);
     for(std::size_t t=0;t<m;t+=3){const std::uint32_t a=dst[t],b=dst[t+1],c=dst[t+2];O.insert(O.end(),{back[a],back[b],back[c]});
      const std::uint8_t la=cornerLabel[a],lb=cornerLabel[b],lc=cornerLabel[c];L.push_back(lb==lc?lb:la);}}});
   // R68: joined at prefix offsets in cluster order (the serial concatenation)
   {std::vector<std::size_t> keepAt(used.size()+1,0),labelAt(used.size()+1,0);for(std::size_t k=0;k<used.size();++k){keepAt[k+1]=keepAt[k]+out[k].size();labelAt[k+1]=labelAt[k]+outLabel[k].size();}
    keep.resize(keepAt.back());keepLabel.resize(labelAt.back());
    ttParallel(used.size(),1,[&](std::size_t k0,std::size_t k1,std::size_t){for(std::size_t k=k0;k<k1;++k){std::copy(out[k].begin(),out[k].end(),keep.begin()+std::ptrdiff_t(keepAt[k]));
     std::copy(outLabel[k].begin(),outLabel[k].end(),keepLabel.begin()+std::ptrdiff_t(labelAt[k]));}});}}
  // speed: the whole surface once more, the cluster seams included (they were locked above), down to the budget under the error cap
  rep.clusterTriangles=keep.size()/3;rep.lowPoly=lp.on;
  if(lp.on&&lp.wholeError>0){const std::size_t lattice=index.size()/3;
   rep.budget=std::min(lp.most,std::max(lp.least,std::size_t(double(lattice)*lp.share)));
   if(keep.size()/3>rep.budget){const auto w0=core::Clock::now();
    std::vector<std::int32_t> local(V,-1);std::vector<std::uint32_t> back,ix(keep.size());
    for(std::size_t q=0;q<keep.size();++q){auto&l=local[keep[q]];if(l<0){l=std::int32_t(back.size());back.push_back(keep[q]);}ix[q]=std::uint32_t(l);}
    std::vector<float> P(back.size()*3);for(std::size_t v=0;v<back.size();++v)for(int q=0;q<3;++q)P[v*3+q]=pos[back[v]*3+q];
    std::vector<std::uint32_t> dst(ix.size());float err=0;
    const std::size_t m=::meshopt_simplify(dst.data(),ix.data(),ix.size(),P.data(),back.size(),12,rep.budget*3,float(lp.wholeError),
     (lp.lockBorder?unsigned(meshopt_SimplifyLockBorder):0u)|unsigned(meshopt_SimplifyErrorAbsolute),&err);
    // a kept triangle's label as above: the majority of its corners', a corner's being its first triangle's
    std::vector<std::uint8_t> cornerLabel(back.size(),0);for(std::size_t t=keepLabel.size();t-->0;)for(int q=0;q<3;++q)cornerLabel[ix[t*3+q]]=keepLabel[t];
    std::vector<std::uint32_t> whole;std::vector<std::uint8_t> wholeLabel;whole.reserve(m);wholeLabel.reserve(m/3);
    for(std::size_t t=0;t+2<m;t+=3){const std::uint32_t a=dst[t],b=dst[t+1],c=dst[t+2];whole.insert(whole.end(),{back[a],back[b],back[c]});
     const std::uint8_t la=cornerLabel[a],lb=cornerLabel[b],lc=cornerLabel[c];wholeLabel.push_back(lb==lc?lb:la);}
    keep.swap(whole);keepLabel.swap(wholeLabel);rep.wholeMs=core::ms(w0);rep.wholeErrorMm=1000.*double(err);}}
  rep.wholeTriangles=keep.size()/3;
  // speed: what backs a point for the channel shader: 1 tissue (kept, or a plug the surface caps), -1 removed (cut, torn, loose, an
  // old cut, another piece's, the rim's air), 0 neither (plain air, outside the box)
  auto backingAt=[&](const Vec&p)->int{const int x=int(std::floor((p.x-g.origin.x)/pitch))-lo[0],y=int(std::floor((p.y-g.origin.y)/pitch))-lo[1],z=int(std::floor((p.z-g.origin.z)/pitch))-lo[2];
   if(x<0||y<0||z<0||x>=nx||y>=ny||z>=nz)return 0;const auto v=f[idx(x,y,z)];const auto st=v&STATE;if(st==KEPT||st==PLUG)return 1;if(st==AIR)return (v&RIM)?-1:0;return -1;};
  // the direction into the tissue at a point, from the cells around it: toward the mean of the tissue cells within lp.bulk cells
  // (the box's outside counts as tissue, as in the blur)
  auto bulkAt=[&](const Vec&p,Vec&d)->bool{const int R=lp.bulk,R2=R*R+R;
   const int px=int(std::floor((p.x-g.origin.x)/pitch))-lo[0],py=int(std::floor((p.y-g.origin.y)/pitch))-lo[1],pz=int(std::floor((p.z-g.origin.z)/pitch))-lo[2];double sx2=0,sy2=0,sz2=0;
   for(int dz=-R;dz<=R;++dz)for(int dy=-R;dy<=R;++dy){const int r2=dz*dz+dy*dy;if(r2>R2)continue;const int y=py+dy,z=pz+dz;const bool rowIn=y>=0&&z>=0&&y<ny&&z<nz;
    for(int dx=-R;dx<=R;++dx){if(r2+dx*dx>R2)continue;const int x=px+dx;bool tissue=true;if(rowIn&&x>=0&&x<nx){const auto st=f[idx(x,y,z)]&STATE;tissue=st==KEPT||st==PLUG;}
     if(tissue){sx2+=dx;sy2+=dy;sz2+=dz;}}}
   const double l=std::sqrt(sx2*sx2+sy2*sy2+sz2*sz2);if(!(l>1e-9))return false;d=Vec{sx2/l,sy2/l,sz2/l};return true;};
  // unit directions into the tissue per vertex of a triangle list (the negative of the render normal): its triangles' normal, area
  // weighted (1), the bulk direction (3) or the mean of both (4); zero where a vertex has none
  auto inwardOf=[&](const std::vector<std::uint32_t>&tri){const std::size_t n=pos.size()/3;std::vector<Vec> in(n,Vec{0,0,0});
   auto P=[&](std::uint32_t v){return Vec{double(pos[v*3]),double(pos[v*3+1]),double(pos[v*3+2])};};
   for(std::size_t t=0;t+2<tri.size();t+=3){const Vec fn=cross(P(tri[t+1])-P(tri[t]),P(tri[t+2])-P(tri[t]));for(int q=0;q<3;++q)in[tri[t+q]]=in[tri[t+q]]+fn;} // the stored winding points into tissue
   ttParallel(n,512,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t v=a;v<b;++v){const double l=length(in[v]);if(!(l>1e-20)){in[v]=Vec{0,0,0};continue;}in[v]=in[v]*(1./l);
    if(lp.normals>=3){Vec d;if(bulkAt(P(std::uint32_t(v)),d)){if(lp.normals==3)in[v]=d;else{const Vec s=in[v]+d;const double ls=length(s);if(ls>1e-6)in[v]=s*(1./ls);}}}}});
   return in;};
  // speed: a backing patch behind every low-poly triangle the channel shader would open. The shader discards a torn-surface pixel
  // whose backing sample (3 mm behind it along the normal) lies in removed cells. The lattice surface had such pixels too (thin
  // shreds, slits under the 2 mm lattice: the pinholes of every torn end); a low-poly triangle adds its own where it spans a gap
  // between two torn bundles, and with nothing drawn inside the gap a discarded pixel is a see-through hole. Every triangle with
  // such a point (7 points per triangle) gets a second triangle lp.patch behind it (its corners moved along their normals, which
  // the patch keeps): where the outer pixel is discarded the patch shows the tissue just under it, like the floor of the gap;
  // everywhere else it lies inside the tissue behind the outer surface and is never seen.
  std::vector<Vec> inward;const std::size_t outerCorners=keep.size();rep.patchOffset=lp.patch;
  if(lp.normals>0){inward=inwardOf(keep);
   if(lp.patch>0&&!keep.empty()){const auto p0=core::Clock::now();constexpr std::uint32_t none=~std::uint32_t(0);std::vector<std::uint32_t> twin(inward.size(),none);
    auto P=[&](std::uint32_t v){return Vec{double(pos[v*3]),double(pos[v*3+1]),double(pos[v*3+2])};};
    // 7 points inside the triangle, then 12 along its edges and corners (a gap under a low-poly edge opens a sliver along it)
    static const double W[19][3]={{1./3,1./3,1./3},{.6,.2,.2},{.2,.6,.2},{.2,.2,.6},{.45,.45,.1},{.1,.45,.45},{.45,.1,.45},
     {.49,.49,.02},{.02,.49,.49},{.49,.02,.49},{.74,.24,.02},{.24,.74,.02},{.02,.74,.24},{.02,.24,.74},{.24,.02,.74},{.74,.02,.24},{.9,.05,.05},{.05,.9,.05},{.05,.05,.9}};
    for(std::size_t t=0;t+2<outerCorners;t+=3){const std::uint32_t tv[3]={keep[t],keep[t+1],keep[t+2]};
     if(!(dot(inward[tv[0]],inward[tv[0]])>.25&&dot(inward[tv[1]],inward[tv[1]])>.25&&dot(inward[tv[2]],inward[tv[2]])>.25))continue;
     bool open=false;const Vec PA=P(tv[0]),PB=P(tv[1]),PC=P(tv[2]);
     for(int s=0;s<lp.points&&!open;++s){const double*bw=W[s];const Vec nn=inward[tv[0]]*bw[0]+inward[tv[1]]*bw[1]+inward[tv[2]]*bw[2];const double l=length(nn);if(!(l>1e-9))continue;
      if(backingAt(PA*bw[0]+PB*bw[1]+PC*bw[2]+nn*(.003/l))<0)open=true;}
     if(!open)continue;std::uint32_t q3[3];
     for(int q=0;q<3;++q){auto&tw=twin[tv[q]];if(tw==none){tw=std::uint32_t(pos.size()/3);const Vec at=P(tv[q])+inward[tv[q]]*lp.patch;
       pos.insert(pos.end(),{float(at.x),float(at.y),float(at.z)});nrm.insert(nrm.end(),{nrm[tv[q]*3],nrm[tv[q]*3+1],nrm[tv[q]*3+2]});inward.push_back(inward[tv[q]]);}q3[q]=tw;}
     keep.insert(keep.end(),{q3[0],q3[1],q3[2]});keepLabel.push_back(keepLabel[t/3]);++rep.patchTriangles;}
    rep.patchMs=core::ms(p0);}}
  // no zero-area triangle (the packer derives face normals; float positions as it reads them), then only the vertices the
  // kept triangles use; a vertex whose softened-field gradient vanished takes its triangles' normal (render side: into the cavity)
  auto surface=std::make_shared<ResolvedTearSurface>();std::vector<std::int32_t> remap(pos.size()/3,-1);std::vector<Vec> faceSum; // speed: + the patch vertices
  for(std::size_t t=0;t+2<keep.size();t+=3){const std::uint32_t a=keep[t],b=keep[t+1],c=keep[t+2];
   const Vec A0{double(pos[a*3]),double(pos[a*3+1]),double(pos[a*3+2])},B0{double(pos[b*3]),double(pos[b*3+1]),double(pos[b*3+2])},C0{double(pos[c*3]),double(pos[c*3+1]),double(pos[c*3+2])};
   const Vec fn=cross(B0-A0,C0-A0);if(!(length(fn)>1e-12))continue;std::array<std::uint32_t,3> tri{};const std::uint32_t abc[3]={a,b,c};
   for(int q=0;q<3;++q){const auto v=abc[q];if(remap[v]<0){remap[v]=std::int32_t(surface->points.size());surface->points.push_back({pos[v*3],pos[v*3+1],pos[v*3+2]});
     surface->normals.push_back({nrm[v*3],nrm[v*3+1],nrm[v*3+2]});faceSum.push_back(Vec{0,0,0});}
    tri[q]=std::uint32_t(remap[v]);faceSum[tri[q]]=faceSum[tri[q]]-fn;} // the stored winding points into tissue: the render normal is its negative
   surface->triangles.push_back(tri);surface->labels.push_back(keepLabel[t/3]);if(t<outerCorners)++rep.outerTriangles;}
  for(std::size_t v=0;v<surface->points.size();++v){auto&nv=surface->normals[v];if(double(nv[0])*nv[0]+double(nv[1])*nv[1]+double(nv[2])*nv[2]>.25)continue;
   const double l=length(faceSum[v]);if(l>1e-20)nv={float(faceSum[v].x/l),float(faceSum[v].y/l),float(faceSum[v].z/l)};}
  // speed: a low-poly vertex takes the normal of its own triangles (area weighted; GFY_TT_NORMALS: 0 the gradient as before, 2 the
  // mean of gradient and triangles, 3 the bulk direction, 4 the mean of triangles and bulk). The softened field's gradient at one
  // lattice vertex is the fine surface's normal there, not that of triangles which now span several of its bumps, and the
  // shader's backing sample 3 mm behind the surface follows the vertex normal.
  if(lp.normals>0){
   for(std::size_t v=0;v<inward.size();++v){if(remap[v]<0)continue;Vec d=inward[v];if(!(dot(d,d)>.25))continue;auto&nv=surface->normals[std::size_t(remap[v])];
    if(lp.normals==2&&double(nrm[v*3])*nrm[v*3]+double(nrm[v*3+1])*nrm[v*3+1]+double(nrm[v*3+2])*nrm[v*3+2]>.25){
     const Vec s=d-Vec{double(nrm[v*3]),double(nrm[v*3+1]),double(nrm[v*3+2])};const double ls=length(s);if(ls>1e-6)d=s*(1./ls);}
    nv={float(-d.x),float(-d.y),float(-d.z)};}
   // no vertex without a normal: the channel packer refuses a surface whose vertex has neither a normal nor a triangle normal sum
   // (channel_normal_degenerate: the damage job then publishes no wall geometry and the split cannot run). A vertex whose
   // gradient vanished and whose triangles' normals cancel takes the normal of one of its triangles (none is degenerate here).
   for(const auto&tri:surface->triangles){bool need=false;for(int q=0;q<3;++q){const auto&nv=surface->normals[tri[q]];need=need||!(double(nv[0])*nv[0]+double(nv[1])*nv[1]+double(nv[2])*nv[2]>.25);}
    if(!need)continue;Vec Q[3];for(int q=0;q<3;++q){const auto&a=surface->points[tri[q]];Q[q]=Vec{double(a[0]),double(a[1]),double(a[2])};}
    const Vec fn=cross(Q[1]-Q[0],Q[2]-Q[0]);const double l=length(fn);if(!(l>0))continue;
    for(int q=0;q<3;++q){auto&nv=surface->normals[tri[q]];if(!(double(nv[0])*nv[0]+double(nv[1])*nv[1]+double(nv[2])*nv[2]>.25))nv={float(-fn.x/l),float(-fn.y/l),float(-fn.z/l)};}}}
  rep.vertices=surface->points.size();rep.triangles=surface->triangles.size();rep.simplifyMs=core::ms(t0);
  static const bool edgeLog=std::getenv("GFY_TISSUE_TEAR_LOG")!=nullptr; // T1 diagnostics only
  if(edgeLog)for(const auto&tri:surface->triangles){double e=0;for(int q=0;q<3;++q){const auto&A=surface->points[tri[q]];const auto&B=surface->points[tri[(q+1)%3]];
    const double dx=double(A[0])-B[0],dy=double(A[1])-B[1],dz=double(A[2])-B[2];e=std::max(e,std::sqrt(dx*dx+dy*dy+dz*dz));}
   rep.maxEdgeMm=std::max(rep.maxEdgeMm,1000*e);if(e>.008)++rep.longTris;}
  if(rep.triangles)rep.surface=std::move(surface);}
};
// The whole tear. g: the body's damage grid (journaled by the caller like every cut). closure: the render closure field (slots
// 1..16, 16 = clothing-gap filler); anatomy: the anatomy field (0 where the closure only fills a clothing gap). center/axis: the
// tear plane (DamageTearVerdict point/axis); radius/half: the cylinder around it that holds the limb's torn ends.
inline Report tear(Grid&g,const painter_detail::TissueField&closure,const painter_detail::TissueField*anatomy,const Vec&center,const Vec&axis,
    double radius,double half,std::uint64_t seed,const Vec&limbPoint,const Vec&limbAxis,const Vec&impact=Vec{0,0,0},double depthScale=1.,bool keepIslands=false,int head=0,
    double ripRadius=0){ // T1: ripRadius = the rip channels' radius (DamageTear): the bone breaks across their slab
 const auto t0=core::Clock::now();Work k(g,closure,anatomy);auto&rep=k.rep;
 if(!closure.available||!(g.pitch>0)){rep.refusal="tissue_tear_unavailable";return rep;} // T1: piece grids too (Work::own)
 k.C=center;k.n=unit(axis);k.limbPoint=limbPoint;k.limbAxis=unit(limbAxis);k.u=unit(cross(k.n,std::abs(k.n.x)<.8?Vec{1,0,0}:Vec{0,1,0}));k.w=cross(k.n,k.u);
 k.Rl=radius;k.H=half;k.pitch=g.pitch;k.seed=std::uint32_t(seed^(seed>>32));k.TL.bind(&closure,g);k.AL.bind(anatomy,g);k.depthScale=std::clamp(depthScale,.1,1.);k.keepIslands=keepIslands;k.imp.head=head;k.own=g.ownership.get();k.ripR=std::max(0.,ripRadius);
 {double ia=0,ib=0;const double L=length(impact);
  if(L>1e-9){const Vec d=impact*(1./L);ia=dot(d,k.u);ib=dot(d,k.w);k.imp.along=dot(d,k.n);}
  double l2=std::sqrt(ia*ia+ib*ib);if(l2<.2){const double th=6.283185307179586*u01(k.seed,5,5,0x77u);ia=std::cos(th);ib=std::sin(th);l2=1;}
  k.imp.a=ia/l2;k.imp.b=ib/l2;k.imp.Rl=std::max(radius,.02);k.imp.tilt=(2*u01(k.seed,6,6,0x78u)-1)*.016;}
 // T2: the limb profile tears to limbDepth (the cylinder reaches past the deepest tear: no cap at its end); each muscle group's
 // map runs across its fibre direction: the limb axis tilted per group (half the E3 shader's tilts 5/35/15/70/50/25/85/10 deg at
 // its azimuths 0/40/200/110/300/150/250/80 deg, so the grain differs between neighbours and stays a limb's); the rest keep the
 // cross-section frame (u, w) and the shot
 if(!head){k.maxD=limbDepth;k.H=std::max(half,limbDepth+.03);}k.imp.lim=k.maxD;
 for(int sd=0;sd<2;++sd){const std::uint32_t s=k.seed^(sd?0x51u:0xA3u);double m=0;for(unsigned v=0;v<8;++v)m+=u01(s,int(100+v),7,0x3C1u);k.imp.mbase[sd]=limbK::groupFloor+limbK::groupSpan*m/8;}
 {Vec A0=unit(k.limbAxis);if(!(std::isfinite(A0.x)&&std::isfinite(A0.y)&&std::isfinite(A0.z)))A0=k.n;
  Vec p1=cross(A0,Vec{0,0,1});const double l1=dot(p1,p1);p1=l1>.04?p1*(1./std::sqrt(l1)):unit(cross(A0,Vec{1,0,0}));const Vec p2=cross(A0,p1);
  static const double tiltDeg[8]={2.5,17.5,7.5,35,25,12.5,42.5,5},azDeg[8]={0,40,200,110,300,150,250,80};const double rad=3.141592653589793/180;
  const Vec dimp=k.u*k.imp.a+k.w*k.imp.b;
  for(unsigned c=0;c<classes;++c){bool fib=false;Vec d=A0;
   if(!head&&c>=mv0&&c<mv0+8){const double t=tiltDeg[c-mv0]*rad,z=azDeg[c-mv0]*rad;d=unit(A0*std::cos(t)+p1*(std::sin(t)*std::cos(z))+p2*(std::sin(t)*std::sin(z)));fib=true;}
   else if(!head&&c==muscle)fib=true;
   if(fib){const Vec r=std::abs(d.x)<.6?Vec{1,0,0}:(std::abs(d.y)<.6?Vec{0,1,0}:Vec{0,0,1});k.fe1[c]=unit(cross(d,r));k.fe2[c]=cross(d,k.fe1[c]);
    k.impC[c]=k.imp;k.impC[c].a=dot(dimp,k.fe1[c]);k.impC[c].b=dot(dimp,k.fe2[c]);}
   else{k.fe1[c]=k.u;k.fe2[c]=k.w;k.impC[c]=k.imp;}}}
 if(!k.box()){rep.ms=core::ms(t0);return rep;}
 k.fill();k.plugs();k.region();k.section();k.tearByDepth();k.thin();k.islands();k.write();if(mask_only_mode_value()<2)k.mesh();else rep.surface=std::make_shared<const ResolvedTearSurface>(); /* mask-only wounds, mode 2: cells as today, no torn-end mesh; the (empty) surface keeps the tear record that later cuts read */
 rep.done=true;rep.ms=core::ms(t0);
 if(const char*e=std::getenv("GFY_TT_DUMP")){ // debug: states around a world point (yz plane at its x): A air, K kept, C cut, T torn,
  // L loose, lower case = not region, F filler kept, r rim; then d2 (cells^2) on the same grid
  char*q=nullptr;const double px=std::strtod(e,&q),py=q&&*q?std::strtod(q+1,&q):0.,pz=q&&*q?std::strtod(q+1,&q):0.;{const Cell c0{int(std::floor((px-g.origin.x)/g.pitch)),int(std::floor((py-g.origin.y)/g.pitch)),int(std::floor((pz-g.origin.z)/g.pitch))};
   const int x=c0.x-k.lo[0],y0=c0.y-k.lo[1],z0=c0.z-k.lo[2];std::cerr<<"TTDUMP box lo=("<<k.lo[0]<<","<<k.lo[1]<<","<<k.lo[2]<<") dim=("<<k.dim[0]<<","<<k.dim[1]<<","<<k.dim[2]<<") cell=("<<x<<","<<y0<<","<<z0<<")"<<std::endl;
   if(x>=0&&x<k.dim[0])for(int pass=0;pass<2;++pass){for(int z=z0+7;z>=z0-7;--z){std::cerr<<(pass?"  d2 ":"  st ");
     for(int y=y0-10;y<=y0+10;++y){if(y<0||z<0||y>=k.dim[1]||z>=k.dim[2]){std::cerr<<(pass?"    .":".");continue;}const std::size_t i=k.idx(x,y,z);const auto f=k.f[i];
      if(pass){std::cerr<<" "<<std::setw(4)<<int(k.distAt(x,y,z)*1000);continue;}
      const char*S="AKCTL???";char ch=S[f&STATE];if(!(f&REGION)&&ch!='A')ch=char(ch-'A'+'a');if((f&FILL)&&(f&STATE)==KEPT)ch='F';if(f&RIM)ch='r';if(y==y0&&z==z0)ch=ch=='A'?'@':ch;std::cerr<<ch;}
     std::cerr<<std::endl;}}}}
 static const bool log=std::getenv("GFY_TISSUE_TEAR_LOG")!=nullptr;
 if(log){std::uint64_t sh=1469598103934665603ull;auto mixb=[&](const void*d,std::size_t n){const auto*b=static_cast<const unsigned char*>(d);for(std::size_t q=0;q<n;++q){sh^=b[q];sh*=1099511628211ull;}};
  if(rep.surface){const auto&S=*rep.surface;mixb(S.points.data(),S.points.size()*12);mixb(S.normals.data(),S.normals.size()*12);mixb(S.triangles.data(),S.triangles.size()*12);mixb(S.labels.data(),S.labels.size());}
  std::cerr<<"tt_hash mask="<<std::hex<<k.dbgMaskHash<<" surf="<<sh<<std::dec<<" fresh="<<rep.fresh<<" solid="<<rep.solid<<" torn="<<rep.torn<<" thin="<<rep.thin<<" loose="<<rep.loose<<" rim="<<rep.rim<<" tris="<<rep.triangles<<std::endl;
  if(rep.surface){const auto&S=*rep.surface;std::size_t points=0,unbacked=0,open=0,floating=0,thinType=0,depthBin[6]={};double area=0,lost=0,lostOpen=0,edgeSum=0;
   auto removedAt=[&](const Vec&p){const int x=int(std::floor((p.x-g.origin.x)/g.pitch))-k.lo[0],y=int(std::floor((p.y-g.origin.y)/g.pitch))-k.lo[1],z=int(std::floor((p.z-g.origin.z)/g.pitch))-k.lo[2];
    if(x<0||y<0||z<0||x>=k.dim[0]||y>=k.dim[1]||z>=k.dim[2])return false;const auto v=k.f[k.idx(x,y,z)];const auto st=v&STATE;return st==AIR?(v&RIM)!=0:st!=KEPT;};
   static const double W[7][3]={{1./3,1./3,1./3},{.6,.2,.2},{.2,.6,.2},{.2,.2,.6},{.45,.45,.1},{.1,.45,.45},{.45,.1,.45}};
   const std::size_t own=rep.lowPoly?std::min(rep.outerTriangles,S.triangles.size()):S.triangles.size();
   for(std::size_t ti=0;ti<own;++ti){const auto&tri=S.triangles[ti];Vec P[3],Nn[3];for(int q=0;q<3;++q){const auto&a=S.points[tri[q]];const auto&b=S.normals[tri[q]];P[q]=Vec{double(a[0]),double(a[1]),double(a[2])};Nn[q]=Vec{double(b[0]),double(b[1]),double(b[2])};}
    const Vec fn=cross(P[1]-P[0],P[2]-P[0]);const double a2=.5*length(fn);area+=a2;unsigned bad=0,badOpen=0;
    for(int q=0;q<3;++q)edgeSum+=length(P[(q+1)%3]-P[q]);
    for(const auto&w:W){const Vec p=P[0]*w[0]+P[1]*w[1]+P[2]*w[2];const Vec m=Nn[0]*w[0]+Nn[1]*w[1]+Nn[2]*w[2];const double l=length(m);if(!(l>1e-9))continue;const Vec n=m*(1./l);
     ++points;if(removedAt(p-n*.003)){++unbacked;++bad;
      if(!(rep.patchOffset>0)||removedAt(p-m*rep.patchOffset-n*.003)){++open;++badOpen;}
      if(removedAt(p-n*.0005)){++floating;int d=1;for(;d<=40&&removedAt(p-n*(.001*d));++d){}++depthBin[d<=4?0:d<=6?1:d<=10?2:d<=20?3:d<=40?4:5];}else ++thinType;}}
    lost+=a2*double(bad)/7.;lostOpen+=a2*double(badOpen)/7.;}
   std::cerr<<"tt_lowpoly on="<<(rep.lowPoly?1:0)<<" lattice_tris="<<rep.rawQuads*2<<" cluster_tris="<<rep.clusterTriangles<<" budget="<<rep.budget<<" whole_tris="<<rep.wholeTriangles<<" own_tris="<<own<<" patch_tris="<<rep.patchTriangles<<" tris="<<rep.triangles<<" verts="<<rep.vertices
    <<" whole_ms="<<rep.wholeMs<<" patch_ms="<<rep.patchMs<<" whole_error_mm="<<rep.wholeErrorMm<<" area_cm2="<<area*1e4<<" mean_edge_mm="<<(own?1000.*edgeSum/(3.*double(own)):0.)
    <<" unbacked_points="<<unbacked<<"/"<<points<<" unbacked_area_pct="<<(area>0?100.*lost/area:0.)<<" open_points="<<open<<" open_area_pct="<<(area>0?100.*lostOpen/area:0.)
    <<" floating="<<floating<<" thin="<<thinType<<" float_depth_mm(<=4/<=6/<=10/<=20/<=40/more)="<<depthBin[0]<<"/"<<depthBin[1]<<"/"<<depthBin[2]<<"/"<<depthBin[3]<<"/"<<depthBin[4]<<"/"<<depthBin[5]<<std::endl;}
  std::size_t far=0,air=0,cyl=0,out=0,region=0,foreign=0,old=0;for(std::size_t i=0;i<k.N;++i){const auto v=k.f[i];if(k.lab[i]&FAR){++far;continue;}if((v&STATE)==AIR){++air;continue;}
   if((v&STATE)==5){++foreign;continue;}if((v&STATE)==6)++old;if(v&CYL)++cyl;else ++out;if(v&REGION)++region;}
  std::size_t rx[3]={0,0,0},torn3[3]={0,0,0};for(int z=0;z<k.dim[2];++z)for(int y=0;y<k.dim[1];++y)for(int x=0;x<k.dim[0];++x){const std::size_t i=k.idx(x,y,z);if(!(k.f[i]&REGION))continue;
   const double wx=g.center(k.cell(x,y,z)).x;const int b=wx<-.02?0:wx>.02?2:1;++rx[b];const auto st=k.f[i]&STATE;if(st==TORN||st==LOOSE)++torn3[b];}
  std::cerr<<"tissue_tear_census box="<<k.N<<" far="<<far<<" air="<<air<<" tissue_in_cylinder="<<cyl<<" tissue_outside="<<out<<" region="<<region
   <<" region_x(neg/mid/pos)="<<rx[0]<<"/"<<rx[1]<<"/"<<rx[2]<<" torn_x="<<torn3[0]<<"/"<<torn3[1]<<"/"<<torn3[2]<<" C=("<<k.C.x<<","<<k.C.y<<","<<k.C.z<<")"<<" owned="<<(g.ownership?1:0)<<" foreign="<<foreign<<" oldcut="<<old<<std::endl;
  std::size_t bs[8]={};for(std::size_t i=0;i<k.N;++i){if(k.lab[i]&FAR)continue;const unsigned L=k.lab[i]&63u;if((L==2||L==21)&&(k.f[i]&REGION))++bs[k.f[i]&STATE];} // T2
  std::cerr<<"tissue_tear_bone kept="<<bs[KEPT]<<" torn="<<bs[TORN]<<" loose="<<bs[LOOSE]<<" plug="<<bs[PLUG]<<" cut="<<bs[CUT]<<std::endl;}
 if(log)std::cerr<<"tissue_tear box="<<k.dim[0]<<"x"<<k.dim[1]<<"x"<<k.dim[2]<<" sources="<<rep.sources<<" region="<<rep.region<<" torn="<<rep.torn<<" thin="<<rep.thin
  <<" mesh(edges "<<k.dbgMs[8]<<" blocks "<<k.dbgMs[9]-k.dbgMs[8]<<" n="<<k.dbgMs[15]<<" | dualmark "<<k.dbgMs[10]<<" dualverts "<<k.dbgMs[11]-k.dbgMs[10]<<" vids "<<k.dbgMs[12]-k.dbgMs[11]<<")"<<" sub(cf "<<k.dbgMs[0]<<" flood "<<k.dbgMs[1]<<" maps "<<k.dbgMs[6]<<" | steps "<<k.dbgMs[3]<<" rim "<<k.dbgMs[4]-k.dbgMs[3]<<" radial "<<k.dbgMs[5]-k.dbgMs[4]<<")"<<" Rl="<<k.Rl<<" H="<<k.H<<" cyl="<<k.dbgCyl[0]<<"x"<<k.dbgCyl[1]<<"x"<<k.dbgCyl[2]<<" src="<<k.dbgSrc[0]<<"x"<<k.dbgSrc[1]<<"x"<<k.dbgSrc[2]<<" n=("<<k.n.x<<","<<k.n.y<<","<<k.n.z<<")"<<" loose="<<rep.loose<<" rim="<<rep.rim<<" fresh="<<rep.fresh<<" quads="<<rep.rawQuads<<" tris="<<rep.triangles<<" verts="<<rep.vertices<<" long_tris="<<rep.longTris<<" max_edge_mm="<<rep.maxEdgeMm
  <<" ms="<<rep.ms<<" (source "<<rep.sourceMs<<" fill "<<rep.fillMs<<" region "<<rep.regionMs<<" edt "<<rep.edtMs<<" tear "<<rep.tearMs<<" thin "<<rep.thinMs
  <<" islands "<<rep.islandMs<<" write "<<rep.writeMs<<" blur "<<rep.blurMs<<" nets "<<rep.netsMs<<" simplify "<<rep.simplifyMs<<")"<<std::endl;
 return rep;}
// T2 (tear2): the triangles of a torn surface that still have tissue behind them: kept, non-filler tissue (Grid::removed false, so
// on a piece grid only the piece's own) within 1-3 mm along the triangle's into-tissue normal (the stored winding) at its centroid
// or a corner. A later tear can erase what an earlier torn surface bounded; its triangles would be drawn with nothing behind them.
inline std::shared_ptr<ResolvedTearSurface> keepBacked(const Grid&g,const painter_detail::TissueField&closure,const ResolvedTearSurface&S,std::size_t*dropped=nullptr){
 auto solidAt=[&](const Vec&p){const Cell c{int(std::floor((p.x-g.origin.x)/g.pitch)),int(std::floor((p.y-g.origin.y)/g.pitch)),int(std::floor((p.z-g.origin.z)/g.pitch))};
  if(!g.valid(c)||g.removed(c))return false;const unsigned L=closure.sample(p.x,p.y,p.z)&63u;return L!=0u&&L!=16u;};
 auto out=std::make_shared<ResolvedTearSurface>();std::vector<std::int32_t> remap(S.points.size(),-1);std::size_t drop=0;
 for(std::size_t t=0;t<S.triangles.size();++t){const auto&T=S.triangles[t];Vec P[3];
  for(int q=0;q<3;++q)P[q]=Vec{double(S.points[T[q]][0]),double(S.points[T[q]][1]),double(S.points[T[q]][2])};
  const Vec fn=cross(P[1]-P[0],P[2]-P[0]);const double l=length(fn);bool ok=false;
  if(l>1e-12){const Vec in=fn*(1./l),c=(P[0]+P[1]+P[2])*(1./3);const Vec at[4]={c,P[0],P[1],P[2]};
   for(int k=0;k<4&&!ok;++k)for(double s:{.001,.002,.003})if(solidAt(at[k]+in*s)){ok=true;break;}}
  if(!ok){++drop;continue;}
  std::array<std::uint32_t,3> tri{};for(int q=0;q<3;++q){const auto v=T[q];if(remap[v]<0){remap[v]=std::int32_t(out->points.size());out->points.push_back(S.points[v]);out->normals.push_back(S.normals[v]);}tri[q]=std::uint32_t(remap[v]);}
  out->triangles.push_back(tri);out->labels.push_back(S.labels[t]);}
 if(dropped)*dropped=drop;return out;}
}
