#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <numeric>
#include <set>
#include <unordered_set>
#include <utility>
#include <vector>
#include "HeadBurstRecord.h"
namespace head_burst {
constexpr unsigned maxNear=6,nFarSites=9;
constexpr double eventScale=.05;
// ---- the round
struct Round {int cls=-1;double radius=0,penetration=0,burst=0,cavitation=0,size=1;}; // modes: size = the burst region's factor (Extreme 2)
inline int roundClass(const Round&r){return r.cls>=0?std::min(r.cls,2):(r.radius>=.011?2:(r.radius>=.0075?1:0));}
// 0..1: small arms ~0.15-0.25, rifle ~0.45-0.6, slug / heavy ~0.8-1 (the class, then the true hole size, exit burst, cavitation)
inline double strength(const Round&r){const int c=roundClass(r);
 double s=c==2?.82:(c==1?.50:.18);const double typical=c==2?.0085:(c==1?.0037:.0045);
 s+=std::clamp((r.radius-typical)/.004,-.1,.1)+.08*std::clamp(r.burst-.5,-.5,.5)+.05*std::clamp(r.cavitation-.8,-.8,.8);
 return std::clamp(s,.08,1.);}
// ---- the plan: sites (near first), their groups, the warp, the torn-end constants
struct Plan {
 Vec site[head_burst_record::maxSites];unsigned group[head_burst_record::maxSites]{};unsigned S=0,nNear=0,groups=1;Vec B{};
 double rf=0,sigma=0,Rcut=0,A1=0,L1=1,A2=0,L2=1,ripR=.0045,dscale=1;std::uint32_t seed=0;Vec exit{},entry{};bool cap=false;
 bool body=false, bulletBodySurface=false, ballisticBone=false;}; // body: a blast's body burst (bodyPlan; entry = the explosion, exit = the point it hit hardest)
// The bullet's way out of the head along d from P0 (last tissue before 2 cm of air, at most 20 cm on)
template<class Tissue>
inline Vec exitPoint(const Vec&P0,const Vec&d,const Tissue&tissueAt){double last=0;
 for(double t=0;t<=.20&&t-last<=.02;t+=.002)if(tissueAt(P0+d*t))last=t;return P0+d*last;}
template<class Tissue>
// headsize: where a round leaves the head or stops (X), and a bullet's OPENING: the point the event sits at (mouth) and the outward
// direction there (o) - its exit and its travel; when it stays inside the head, or leaves under the floor (down the neck, in the
// chest), the skin where it entered (the first tissue within 6 cm of its first solid cell) and against its travel. false: no
// opening above the floor (the plane through floorAt with the normal floorN; nullptr: no floor) - a hit through the neck. own (the
// Extreme damage mode, a crushing weapon's blow): the exit or the stop as it is, as before. (The template line above is this one's.)
inline bool opening(const Vec&entry,const Vec&P0,const Vec&d,double penetration,const Tissue&tissueAt,const Vec*floorAt,const Vec&floorN,bool own,Vec&X,Vec&mouth,Vec&o){
 auto over=[&](const Vec&q){return dot(q-*floorAt,floorN);};
 X=exitPoint(P0,d,tissueAt);bool leaves=!floorAt||over(X)>=0;
 if(penetration>0){const Vec stop=entry+d*penetration;if(dot(stop-P0,d)<dot(X-P0,d)){X=stop;leaves=own;}}
 mouth=X;o=d;if(leaves)return true;
 mouth=entry;o=d*-1.;for(double t=0;t<=.06;t+=.001)if(tissueAt(entry+d*t)){mouth=entry+d*t;break;}
 return !floorAt||over(mouth)>=0;}
// ownR: 0 = a bullet (the body event's scale and placement); > 0 = the head's own scale and placement (the Extreme damage mode, a
// crushing weapon's blow: as before). floorAt, floorN: a bullet's event stays above the plane through floorAt with the normal
// floorN (the head's floor; none: nullptr). A plan with S == 0: the round has no opening on the head - no event.
template<class Tissue>
inline Plan plan(const Vec&brainC,const Vec&entry,const Vec&P0,const Vec&d,std::uint64_t seed,bool capEvent,const Round&round,const Tissue&tissueAt,double ownR=0.,
 const Vec*floorAt=nullptr,const Vec&floorN=Vec{0,1,0}){
 if(const char*e=std::getenv("GFY_HEAD_SEED"))seed^=core::quantMix(std::strtoull(e,nullptr,10)*0x9E3779B97F4A7C15ull+1);
 auto R=[&](unsigned k){return damage_tear::hash01(seed,0xB0B6ull,k);};
 Plan p;p.seed=std::uint32_t(core::quantMix(seed^0x4EADB0B6ull));p.sigma=strength(round);p.cap=capEvent;p.entry=entry;
 if(const char*e=std::getenv("GFY_HEAD_STRENGTH"))p.sigma=std::clamp(std::atof(e),.05,1.); // probe test knob
 const double sg=p.sigma;
 const bool own=ownR>0,floored=!own&&floorAt!=nullptr;
 p.rf=(own?ownR:eventScale)*(.25+.90*sg)/1.3*(.85+.30*R(4))*(capEvent?1.15:1.)*(round.size>0?round.size:1.); // modes: Extreme doubles it
 auto over=[&](const Vec&q){return dot(q-*floorAt,floorN);}; // a point's height above the head's floor (floored only)
 // where the round leaves the head or stops, and a bullet's opening (opening() above). No opening on the head: no head event.
 Vec X,mouth,o;const bool open=opening(entry,P0,d,round.penetration,tissueAt,floored?floorAt:nullptr,floorN,own,X,mouth,o);
 p.exit=X;if(!open)return p;
 const Vec a0=unit(cross(o,std::abs(o.x)<.8?Vec{1,0,0}:Vec{0,1,0})),b0=cross(o,a0);
 if(own){ // (a small round's blast sits at its exit wound, a strong one's toward the middle of the head)
  const double f=std::min(1.,std::clamp(.9-.6*sg,.25,.9)+.25*std::max(0.,(.6-sg)/.6));
  p.B=P0+(X-P0)*f-d*(.30*p.rf*(1-f))+a0*(.5*p.rf*(R(2)-.5))+b0*(.5*p.rf*(R(3)-.5));}
 // a bullet's pieces: the blast point 0.45 rf inside the opening - the fragments are scalp, skull and what lies under them, open
 // through the skin. Its cap: 0.30 rf OUTSIDE the skin, the sites spread along it - a dish of scalp and skull (the body's
 // surface blowout)
 else p.B=mouth+o*(capEvent?.30*p.rf:-.45*p.rf)+a0*(.25*p.rf*(R(2)-.5))+b0*(.25*p.rf*(R(3)-.5));
 if(own||!capEvent)for(int k=1;k<=30&&!tissueAt(p.B);++k)p.B=p.B-o*.002; // the blast sits in tissue
 // headsize: the event stays on the head. The clip plane lies wob above the floor (the warp moves a boundary by up to A1 + A2 =
 // 0.27 rf); clip: the region can reach it (the near sites lie within 0.85 rf of the blast, a fragment reaches 1.7 rf from its
 // site at most). The blast point and every near site stay at least 0.1 rf above the plane (a site under it: its mirror image).
 const double wob=.27*p.rf;const bool clip=floored&&over(p.B)-wob<2.6*p.rf;
 if(clip&&over(p.B)-wob<.1*p.rf)p.B=p.B+floorN*(.1*p.rf-(over(p.B)-wob));
 auto sphere=[&](unsigned k){const double z=2*R(k)-1,t=6.283185307179586*R(k+1),r=std::sqrt(std::max(0.,1-z*z));return Vec{r*std::cos(t),r*std::sin(t),z};};
 Vec outward=o;if(own){outward=p.B-brainC;outward=length(outward)>1e-6?unit(outward):d;} // headsize: a bullet's sites lean out along o
 // near sites: pieces 1-6 from the round (pistol 1-2, rifle 3-4, slug 5-6); a cap event 2-3 sites joined into 1-2 plates
 const unsigned n0=capEvent?2u+unsigned(R(5)*1.999):std::min(maxNear,1u+unsigned(sg*5.*(.8+.4*R(5))));
 const unsigned n=clip?std::min(n0,4u):n0; // headsize: near the floor at most 4 (their mirror images need sites too)
 p.nNear=n;
 for(unsigned k=0;k<n;++k){
  if(k==0){p.site[k]=p.B+sphere(10)*(.15*p.rf*R(12));continue;}
  Vec s=sphere(20+3*k);const bool dish=capEvent&&!own;if(dish)s=s-o*(.6*dot(s,o)); // headsize: a bullet's cap, along the skin
  const Vec dir=unit(s*.6+outward*(dish?-.15:.4));p.site[k]=p.B+dir*(p.rf*(.35+.5*R(22+3*k)));}
 if(clip)for(unsigned k=0;k<n;++k){double h=over(p.site[k])-wob;if(h<0){p.site[k]=p.site[k]-floorN*(2*h);h=-h;}if(h<.1*p.rf)p.site[k]=p.site[k]+floorN*(.1*p.rf-h);}
 if(capEvent){const bool two=sg*R(6)>.45&&n>=2;for(unsigned k=0;k<n;++k)p.group[k]=two&&k==n-1?2u:1u;p.groups=two?3u:2u;}
 else{for(unsigned k=0;k<n;++k)p.group[k]=k+1;p.groups=n+1;}
 // far sites around them: the rest of the head (stays on the neck)
 {const double rot=6.283185307179586*R(60);
  for(unsigned k=0;k<nFarSites;++k){const double z=1-2*(k+.5)/nFarSites,t=2.399963229728653*k+rot,r=std::sqrt(std::max(0.,1-z*z));const Vec u{r*std::cos(t),r*std::sin(t),z};
   p.site[n+k]=p.B+unit(u+sphere(70+3*k)*.25)*(p.rf*(1.65+.35*R(72+3*k)));p.group[n+k]=0;}}
 p.S=n+nFarSites;
 // headsize: never under the head's floor. Every near site within 1.7 rf of the clip plane gets its mirror image under the
 // plane as a far site: the two are equally far from every point of the plane, so under it the mirror is the nearer one - nothing
 // there belongs to a fragment, and above it nothing changes. (The lowest far sites make room when the 15 sites are full: the
 // plane bounds the region where they stood.)
 if(clip){unsigned m=0;for(unsigned k=0;k<n;++k)if(over(p.site[k])-wob<1.7*p.rf)++m;
  while(p.S+m>head_burst_record::maxSites&&p.S>n){unsigned low=n;for(unsigned k=n+1;k<p.S;++k)if(over(p.site[k])<over(p.site[low]))low=k;
   p.site[low]=p.site[p.S-1];--p.S;}
  for(unsigned k=0;k<n&&p.S<head_burst_record::maxSites;++k){const double h=over(p.site[k])-wob;
   if(h<1.7*p.rf){p.site[p.S]=p.site[k]-floorN*(2*h);p.group[p.S]=0;++p.S;}}}
 // (a stronger warp folds space: thin slivers of another group would cross a fragment's hood as dark lines)
 p.A1=.22*p.rf;p.L1=.95*p.rf;p.A2=.05*p.rf;p.L2=.32*p.rf;p.ripR=.003;p.dscale=.75+.25*sg;
 p.Rcut=2.6*p.rf+2*(p.A1+p.A2); // provisional: the labelling pass sets the exact cut (every near cell inside)
 return p;}

// headsize: a finished plan kept above a level plane - a NECK hit's body event above the neck's base joint (LiveDamageWorker.cpp;
// the head's own events are clipped inside plan()). The same rules: the clip plane 0.27 rf above the floor, the blast point and
// the near sites at least 0.1 rf above it (a site under it: its mirror image), at most 4 near sites, and every near site within
// 1.7 rf of the plane gets its mirror image under it as a far site (the lowest far sites make room in the 15).
inline void clipAbove(Plan&p,const Vec&floorAt,const Vec&floorN){
 auto over=[&](const Vec&q){return dot(q-floorAt,floorN);};const double wob=.27*p.rf;
 if(!(p.rf>0)||!p.S||p.nNear>p.S||over(p.B)-wob>=2.6*p.rf)return;
 if(p.nNear>4){const unsigned n0=p.nNear,far=p.S-n0; // (a burst: each near site its own group 1..n; the last ones leave)
  for(unsigned k=0;k<far;++k){p.site[4+k]=p.site[n0+k];p.group[4+k]=p.group[n0+k];}
  p.S=4+far;p.nNear=4;if(p.groups>5)p.groups=5;}
 const unsigned n=p.nNear;
 if(over(p.B)-wob<.1*p.rf)p.B=p.B+floorN*(.1*p.rf-(over(p.B)-wob));
 for(unsigned k=0;k<n;++k){double h=over(p.site[k])-wob;if(h<0){p.site[k]=p.site[k]-floorN*(2*h);h=-h;}if(h<.1*p.rf)p.site[k]=p.site[k]+floorN*(.1*p.rf-h);}
 unsigned m=0;for(unsigned k=0;k<n;++k)if(over(p.site[k])-wob<1.7*p.rf)++m;
 while(p.S+m>head_burst_record::maxSites&&p.S>n){unsigned low=n;for(unsigned k=n+1;k<p.S;++k)if(over(p.site[k])<over(p.site[low]))low=k;
  p.site[low]=p.site[p.S-1];p.group[low]=p.group[p.S-1];--p.S;}
 for(unsigned k=0;k<n&&p.S<head_burst_record::maxSites;++k){const double h=over(p.site[k])-wob;
  if(h<1.7*p.rf){p.site[p.S]=p.site[k]-floorN*(2*h);p.group[p.S]=0;++p.S;}}}

template<class Tissue>
inline Plan bodyPlan(const Vec&E,const Vec&B0,double sigma,std::uint64_t seed,const Tissue&tissueAt,double size=1.){ // modes: size (Extreme 2)
 if(const char*e=std::getenv("GFY_BODY_SEED"))seed^=core::quantMix(std::strtoull(e,nullptr,10)*0x9E3779B97F4A7C15ull+1);
 auto R=[&](unsigned k){return damage_tear::hash01(seed,0xB0D7ull,k);};
 Plan p;p.body=true;p.seed=std::uint32_t(core::quantMix(seed^0x0B0D7B57ull));p.sigma=std::clamp(sigma,0.,1.);p.entry=E;p.exit=B0;
 if(const char*e=std::getenv("GFY_BODY_STRENGTH"))p.sigma=std::clamp(std::atof(e),.0,1.); // probe test knob
 const double sg=p.sigma;
 p.rf=(.08+.22*sg)*(.9+.2*R(4))*(size>0?size:1.); // modes: Extreme doubles it (the labelled box keeps its 0.6 m bound)
 Vec d=B0-E;d=length(d)>1e-6?unit(d):Vec{0,1,0};
 // the blast point a quarter of the region inside the body from where it hit hardest (back toward that point on a thin limb)
 p.B=B0+d*(.25*p.rf);for(int k=0;k<40&&!tissueAt(p.B);++k)p.B=p.B-d*(.25*p.rf/40.);
 auto sphere=[&](unsigned k){const double z=2*R(k)-1,tt=6.283185307179586*R(k+1),r=std::sqrt(std::max(0.,1-z*z));return Vec{r*std::cos(tt),r*std::sin(tt),z};};
 constexpr unsigned nFar=6;
 const unsigned n=std::min(head_burst_record::maxSites-nFar,2u+unsigned(std::lround(7*sg*(.85+.3*R(5)))));
 p.nNear=n;
 // near sites: dense at the blast, sparse farther (r = rf (0.2 + 0.8 u^1.5)), leaning into the body (away from the explosion)
 for(unsigned k=0;k<n;++k){if(k==0){p.site[k]=p.B;continue;}
  const Vec dir=unit(sphere(20+3*k)*.7+d*.3);p.site[k]=p.B+dir*(p.rf*(.2+.8*std::pow(R(22+3*k),1.5)));}
 for(unsigned k=0;k<n;++k)p.group[k]=k+1;p.groups=n+1;
 // far sites: the rest of the body (group 0), 1.5 - 1.9 rf out
 {const double rot=6.283185307179586*R(60);
  for(unsigned k=0;k<nFar;++k){const double z=1-2*(k+.5)/nFar,tt=2.399963229728653*k+rot,r=std::sqrt(std::max(0.,1-z*z));const Vec u{r*std::cos(tt),r*std::sin(tt),z};
   p.site[n+k]=p.B+unit(u+sphere(70+3*k)*.25)*(p.rf*(1.5+.4*R(72+3*k)));p.group[n+k]=0;}}
 p.S=n+nFar;
 p.A1=.22*p.rf;p.L1=.95*p.rf;p.A2=.05*p.rf;p.L2=.32*p.rf;p.ripR=.003;p.dscale=.75+.25*sg;
 p.Rcut=std::min(2.6*p.rf+2*(p.A1+p.A2),.6); // provisional (the labelling sets the exact cut); 0.6 m bounds the labelled box
 return p;}

// bonefix2 (2026-11-20): the bone switches (consoles painter_bone_resist 1/0 and painter_bone_strength 0..100, set by
// LiveDamageWorker.cpp), read by a crater (BulletBurst.h keepResistingBone) and by a body burst's record: its bone band (header
// word 51, site 1's unused w) = strength x 0.8 x the blast radius; a piece holding the far part keeps the bone of the fragments'
// region within it (shaders/r3/painter_vmask_head.h), so ribs, the spine and bone ends stand out of the torn body. A head burst
// (not body) carries none.
inline std::atomic<int>& boneResistOn(){static std::atomic<int> v{1};return v;}
inline std::atomic<int>& boneStrength(){static std::atomic<int> v{70};return v;}
inline double boneBand(const Plan&p){if(!p.ballisticBone||!boneResistOn().load(std::memory_order_relaxed))return 0;
 const double s=std::clamp(double(boneStrength().load(std::memory_order_relaxed))/100.,0.,.95);return s*.8*p.rf;}
// the record of a plan (HeadBurstRecord.h layout)
inline std::shared_ptr<gfy_packed::TearBlock> record(const Plan&p,double pitch){
 auto B=std::make_shared<gfy_packed::TearBlock>();B->params.assign(4*51,0u);B->params[29]=head_burst_record::MAGIC;
 {const float H=-1.f;std::memcpy(&B->params[7],&H,4);} // a decide-once reader (MaskPieces3 Region) finds no region and no section
 B->header.assign(104,0u);auto F=[&](int i,double v){const float f=float(v);std::memcpy(&B->header[std::size_t(i)],&f,4);};
 F(0,p.B.x);F(1,p.B.y);F(2,p.B.z);F(3,p.Rcut);F(4,p.A1);F(5,1./p.L1);F(6,p.A2);F(7,1./p.L2);F(8,p.ripR);F(9,p.dscale);
 B->header[10]=p.seed;B->header[11]=p.S;B->header[13]=p.groups;B->header[14]=p.body?2u:1u;B->header[15]=0u;B->header[27]=p.bulletBodySurface?0x424c4431u:0u;
 for(unsigned i=0;i<p.S;++i){F(44+4*int(i),p.site[i].x);F(45+4*int(i),p.site[i].y);F(46+4*int(i),p.site[i].z);B->header[24+i/8]|=(p.group[i]&15u)<<(4*(i%8));}
 if(p.ballisticBone&&boneBand(p)>0.)B->header[55]=0x425331u;
 F(51,boneBand(p)); // bonefix2: the bone band (site 1's w: no reader takes it as a site coordinate; the atlas writes 19, 27, 31)
 B->map.assign(20,0.f);B->map_dim[0]=B->map_dim[1]=1;B->map_pitch=1;B->bake.assign(1,255);B->bake_dim[0]=B->bake_dim[1]=B->bake_dim[2]=1;B->bake_pitch=1;
 for(int k=0;k<3;++k){B->centre[k]=p.B[k];B->normal[k]=k==1;B->u[k]=k==0;B->w[k]=k==2;}
 B->reach=p.Rcut;B->pitch=pitch;B->seed=p.seed;B->n=0;
 return B;}

// ---- the burst on the grid
struct Burst {bool done=false;std::size_t cells=0,mask=0,edgeCells=0,erased=0,labelled=0,tiles=0,samples=0;std::array<std::size_t,64> bySlot{};
 std::set<Cell> dirty;std::vector<ResolvedTunnel> records;const char*refusal=nullptr;double ms=0,labelMs=0,eraseMs=0,repairMs=0;
 std::vector<std::size_t> groupCells;std::size_t mixedTiles=0,crumbs=0,crumbCells=0,preCells=0;double crumbMs=0,preMs=0;Plan plan;
 std::size_t boneKept=0,boneShell=0;double boneMs=0;}; // bonefix2: bone the body keeps, near cells cut around it
// the lowest set bit of w != 0 (de Bruijn)
inline unsigned ctz64(std::uint64_t w){static const unsigned char tab[64]={0,1,56,2,57,49,28,3,61,58,42,50,38,29,17,4,62,47,59,36,45,43,51,22,53,39,33,30,24,18,12,5,
 63,55,48,27,60,41,37,16,46,35,44,21,52,32,23,11,54,26,40,15,34,20,31,10,25,14,19,9,13,8,7,6};return tab[((w&(~w+1))*0x03f79d71b4ca8b09ull)>>58];}
// Label the head's cells, erase the cracks, make the record. Journaled through Grid::erase.
inline Burst burst(Grid&g,const std::vector<SeverSegment>&segments,SeverState&state,const Plan&plan0){
 const auto t0=core::Clock::now();Burst out;out.plan=plan0;Plan&p=out.plan;const double pitch=g.pitch;
 auto block=record(p,pitch); // the record's own (float) numbers label the cells: the pixel programs read the same ones
 head_burst_record::Function fn;head_burst_record::decode(*block,fn);
 // The labelling, hierarchical: the function's group and margin (the warped distance to the nearest site of another group) at a
 // lattice of points; a block whose 8 corners share a group, each farther from any other group than the block's cells can be from
 // their nearest corner (x (1 + the warp's slope)), takes that group whole - tiles (8^3 cells) from a lattice every 8 cells, blocks
 // (4^3) every 4; a sub-block (2^3, a lattice every 2 cells = 4 mm) whose 8 corners share a group takes it, one whose corners differ
 // is the CRACK (its tissue erased): two sub-blocks of different groups never touch (they would share a corner), so every group
 // is cut from every other one, 4 mm wide, inside the torn gap the pixel programs draw around the exact boundary.
 // (1 + slope): the warp's slope was 0.39 median, 0.91 at the 99th percentile, 1.6 at most (measured, 200k points of a slug burst,
 // warp .30 / .07 rf; now .22 / .05 rf: x 0.73);
 // 1 is taken - a cell it misses lies within a few mm of a crack, inside the torn gap both pieces draw empty (painter_vmask_head.h)
 const double slope=2.0,M8=std::sqrt(3.)*4*pitch*slope,M4=std::sqrt(3.)*2*pitch*slope;
 const Vec lo=p.B-Vec{p.Rcut,p.Rcut,p.Rcut},hi=p.B+Vec{p.Rcut,p.Rcut,p.Rcut};
 int tlo[3],thi[3];for(int k=0;k<3;++k){tlo[k]=std::max(0,int(std::floor((lo[k]-g.origin[k])/pitch))/8);thi[k]=std::min(((&g.size.x)[k]-1)/8,int(std::floor((hi[k]-g.origin[k])/pitch))/8);}
 if(tlo[0]>thi[0]||tlo[1]>thi[1]||tlo[2]>thi[2]){out.refusal="head_burst_outside_grid";return out;}
 const int TX=thi[0]-tlo[0]+1,TY=thi[1]-tlo[1]+1,TZ=thi[2]-tlo[2]+1;
 auto tIdx=[&](int x,int y,int z){return (std::size_t(z-tlo[2])*TY+std::size_t(y-tlo[1]))*TX+std::size_t(x-tlo[0]);};
 const auto*table=state.table;const bool perCell=!table||g.ownership; // an owned grid (a piece's): its tissue cell by cell
 const std::size_t W=core::parallelWidth();std::vector<std::size_t> labelled(W,0);std::vector<double> rMaxBy(W,0.);std::vector<std::vector<Cell>> eraseBy(W),candBy(W);
 // the tiles' corner lattice over the cut's box
 const int KX=TX+1,KY=TY+1,KZ=TZ+1;std::vector<std::uint8_t> kG(std::size_t(KX)*KY*KZ,0);std::vector<float> kM(std::size_t(KX)*KY*KZ,0.f);
 core::parallelChunks(std::size_t(KZ),1,[&](std::size_t a,std::size_t b,std::size_t ch){for(std::size_t z=a;z<b;++z)for(int y=0;y<KY;++y)for(int x=0;x<KX;++x){
  const Vec q=g.origin+Vec{((tlo[0]+x)*8+.5)*pitch,((tlo[1]+y)*8+.5)*pitch,((tlo[2]+int(z))*8+.5)*pitch};double m;const unsigned gg=fn.labelMargin(q.x,q.y,q.z,m);
  const std::size_t i=(z*KY+std::size_t(y))*KX+std::size_t(x);kG[i]=std::uint8_t(gg);kM[i]=float(m);++labelled[ch];}});
 // tile classes: 254 no tissue, 255 mixed, else the group of every cell (beyond the cut: 0)
 std::vector<std::uint8_t> tileG(std::size_t(TX)*TY*TZ,254);
 core::parallelChunks(std::size_t(TZ),1,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t zz=a;zz<b;++zz){const int z=tlo[2]+int(zz);
  for(int y=tlo[1];y<=thi[1];++y)for(int x=tlo[0];x<=thi[0];++x){
   if(!perCell){const auto it=table->find(Cell{x,y,z});if(it==table->end())continue;std::uint64_t any=0;for(auto w:it->second.solid)any|=w;if(!any)continue;}
   const double bmin[3]={g.origin.x+(x*8+.5)*pitch,g.origin.y+(y*8+.5)*pitch,g.origin.z+(z*8+.5)*pitch},bmax[3]={bmin[0]+7*pitch,bmin[1]+7*pitch,bmin[2]+7*pitch};
   double c2=0;for(int a2=0;a2<3;++a2){const double d=std::max(bmin[a2]-fn.B[a2],0.)+std::max(fn.B[a2]-bmax[a2],0.);c2+=d*d;}
   if(c2>fn.Rcut*fn.Rcut){tileG[tIdx(x,y,z)]=0;continue;}
   unsigned gg=255;bool same=true;
   for(int c=0;c<8&&same;++c){const std::size_t i=(std::size_t(z-tlo[2]+(c>>2))*KY+std::size_t(y-tlo[1]+((c>>1)&1)))*KX+std::size_t(x-tlo[0]+(c&1));
    if(gg==255)gg=kG[i];same=kG[i]==gg&&kM[i]>M8;}
   tileG[tIdx(x,y,z)]=std::uint8_t(same?gg:255u);}}});
 auto tileClass=[&](int x,int y,int z)->unsigned{if(x<tlo[0]||y<tlo[1]||z<tlo[2]||x>thi[0]||y>thi[1]||z>thi[2])return 0u;return tileG[tIdx(x,y,z)];};
 const double tmClass=core::ms(t0);
 // the label box: tiles that can hold a near cell + one tile around them; labels 255 = no tissue (or removed), else the group
 int clo[3]={INT_MAX,INT_MAX,INT_MAX},chi[3]={INT_MIN,INT_MIN,INT_MIN};std::size_t mixed=0;
 for(int z=tlo[2];z<=thi[2];++z)for(int y=tlo[1];y<=thi[1];++y)for(int x=tlo[0];x<=thi[0];++x){const unsigned c=tileG[tIdx(x,y,z)];if(c==0||c==254)continue;++out.tiles;mixed+=c==255;
  const int t3[3]={x,y,z};for(int k=0;k<3;++k){clo[k]=std::min(clo[k],t3[k]-1);chi[k]=std::max(chi[k],t3[k]+1);}}
 if(!out.tiles){out.refusal="head_burst_no_tissue";return out;}
 for(int k=0;k<3;++k){clo[k]=std::max(clo[k],0);chi[k]=std::min(chi[k],((&g.size.x)[k]-1)/8);}
 const int NX=(chi[0]-clo[0]+1)*8,NY=(chi[1]-clo[1]+1)*8,NZ=(chi[2]-clo[2]+1)*8;const int ox=clo[0]*8,oy=clo[1]*8,oz=clo[2]*8;
 std::vector<std::uint8_t> lab(std::size_t(NX)*NY*NZ,255);
 auto idx=[&](int x,int y,int z){return (std::size_t(z-oz)*NY+std::size_t(y-oy))*NX+std::size_t(x-ox);};
 auto L=[&](int x,int y,int z)->std::uint8_t&{return lab[idx(x,y,z)];};
 auto inBox=[&](int x,int y,int z){return x>=ox&&y>=oy&&z>=oz&&x<ox+NX&&y<oy+NY&&z<oz+NZ;};
 const int BX=NX/4,BY=NY/4,BZ=NZ/4;std::vector<std::uint8_t> blockG(std::size_t(BX)*BY*BZ,254); // 4^3 blocks: 254 no tissue, 255 mixed, else the group
 auto bIdx=[&](int bx,int by,int bz){return (std::size_t(bz)*BY+std::size_t(by))*BX+std::size_t(bx);};
 // the tiles labelled: every tile that may hold a near cell and the far tiles around them (a crack on their faces); the far part
 // farther in the box is never read
 std::vector<Cell> boxTiles;for(int z=clo[2];z<=chi[2];++z)for(int y=clo[1];y<=chi[1];++y)for(int x=clo[0];x<=chi[0];++x){const unsigned c=tileClass(x,y,z);
  if(c==254u&&!perCell)continue;bool need=c!=0u&&c!=254u;
  for(int q=0;q<27&&!need;++q){if(q==13)continue;const unsigned n=tileClass(x+q%3-1,y+(q/3)%3-1,z+q/9-1);need=n!=0u&&n!=254u;}
  if(need)boxTiles.push_back(Cell{x,y,z});}
 {const std::size_t n=boxTiles.size();std::size_t stride=7919;while(n>1&&std::gcd(stride,n)!=1)stride+=2;std::vector<Cell> order(n);for(std::size_t i=0;i<n;++i)order[i]=boxTiles[(i*stride)%n];boxTiles.swap(order);}
 // a tile's kept tissue as bits (Grid::local order): the model's tile mask less the removed cells (an owned grid: cell by cell)
 auto tissueBits=[&](const Cell&t,std::uint64_t bits[8],TissueLookup&tl,core::TileCache&cache){for(int q=0;q<8;++q)bits[q]=0;
  // (TissueLookup's rule per tile: the model's mask where no ownership decides - no grid ownership, or a tile the owner holds whole)
  if(table&&(!g.ownership||g.ownership->pureTile(Cell{t.x*8,t.y*8,t.z*8}))){const auto it=table->find(t);if(it==table->end())return;const auto*tile=cache.find(t);
   for(int q=0;q<8;++q)bits[q]=it->second.solid[std::size_t(q)]&~(tile?tile->removed[std::size_t(q)]:0ull);return;}
  for(unsigned l=0;l<512;++l){const Cell c{t.x*8+int(l>>6),t.y*8+int((l>>3)&7u),t.z*8+int(l&7u)};if(tl.solid(c)&&!cache.removed(c))bits[l>>6]|=std::uint64_t(1)<<(l&63u);}};
 // per tile, no cell visited in a block of one group: the 8 blocks (4^3) - a tile of one group gives it to all of them; a mixed tile
 // tests each block's 3 x 3 x 3 corners (every 4 cells), and a block that fails, each 2^3 sub-block's 8 corners (every 2 cells):
 // one group -> its tissue cells take it, corners that differ -> its tissue cells are the crack. Labels are written only in mixed
 // blocks (the crumb flood reads them); sample points are the middle cells of blocks of one group.
 std::vector<std::vector<std::size_t>> groupBy(W,std::vector<std::size_t>(p.groups,0));std::vector<std::vector<float>> sampleBy(W);
 std::uint64_t blockMask[4];for(int k=0;k<4;++k){const int bv=(k&1)*4,bw=(k>>1)*4;std::uint64_t m=0;for(int v=bv;v<bv+4;++v)for(int w=bw;w<bw+4;++w)m|=std::uint64_t(1)<<(v*8+w);blockMask[k]=m;}
 const double tmBox=core::ms(t0);std::vector<double> chunkMs(W,0.);
 core::parallelChunks(boxTiles.size(),1,[&](std::size_t a,std::size_t b,std::size_t ch){TissueLookup tissue{g,table};core::TileCache cache(g);std::uint64_t bits[8];const auto ct0=core::Clock::now();
  // the tissue of one cell / of one block's 64 cells (the model's tile mask when the job has it: TissueLookup's rule)
  auto solidAt=[&](const Cell&c){return g.valid(c)&&tissue.solid(c)&&!cache.removed(c);};
  auto sampleAt=[&](const Cell&c,unsigned gg){const Vec q=g.center(c);sampleBy[ch].insert(sampleBy[ch].end(),{float(q.x),float(q.y),float(q.z),float(gg)});};
  for(std::size_t i=a;i<b;++i){const Cell t=boxTiles[i];const unsigned tc=tileClass(t.x,t.y,t.z);
   // a tile of the far part: nothing to label (a sample of the far part from every 7th such tile's middle cell)
   if(tc==0){if(((t.x*7+t.y*13+t.z*29)%7)==0){const Cell c{t.x*8+4,t.y*8+4,t.z*8+4};if(solidAt(c))sampleAt(c,0);}
    for(int k=0;k<8;++k){const int bx=(t.x*8-ox)/4+(k&1),by=(t.y*8-oy)/4+((k>>1)&1),bz=(t.z*8-oz)/4+(k>>2);if(bx<BX&&by<BY&&bz<BZ)blockG[bIdx(bx,by,bz)]=0;}continue;}
   bool nearSeen=false,bitsReady=false;
   auto tileBits=[&]{if(!bitsReady){tissueBits(t,bits,tissue,cache);bitsReady=true;}};
   unsigned lg[125];double lm[125];bool have[125]={};
   auto at=[&](int u,int v,int w,bool margin,double&m)->unsigned{const int k=(u>>1)+5*(v>>1)+25*(w>>1);
    if(!have[k]||(margin&&lm[k]<0)){const Vec q=g.center(Cell{t.x*8+u,t.y*8+v,t.z*8+w});if(margin)lg[k]=fn.labelMargin(q.x,q.y,q.z,lm[k]);else{lg[k]=fn.label(q.x,q.y,q.z);lm[k]=-1;}have[k]=true;++labelled[ch];}
    m=lm[k];return lg[k];};
   for(int k=0;k<8;++k){const int bu=(k&1)*4,bv=((k>>1)&1)*4,bw=(k>>2)*4;
    const int bx=(t.x*8-ox+bu)/4,by=(t.y*8-oy+bv)/4,bz=(t.z*8-oz+bw)/4;
    unsigned gg=tc;
    if(tc>=254){unsigned g4=255;bool same=true;for(int c=0;c<8&&same;++c){double m;const unsigned q=at(bu+(c&1)*4,bv+((c>>1)&1)*4,bw+(c>>2)*4,true,m);if(g4==255)g4=q;same=q==g4&&m>M4;}
     gg=same?g4:255u;}
    // a block of one group: no cell visited (its tissue is not needed: the crumb flood treats it as its group's, which is safe)
    if(gg<254){if(bx<BX&&by<BY&&bz<BZ)blockG[bIdx(bx,by,bz)]=std::uint8_t(gg);
     if(gg<p.groups){groupBy[ch][gg]+=64;const Cell c{t.x*8+bu+2,t.y*8+bv+2,t.z*8+bw+2};if((gg||((bx*7+by*13+bz*29)%5)==0)&&solidAt(c))sampleAt(c,gg);}
     if(gg)nearSeen=true;continue;}
    // a mixed block: its tissue, then its sub-blocks
    std::uint64_t mb[4]={0,0,0,0}; // rows u = bu..bu+3: bits v * 8 + w of this block
    if(table){tileBits();for(int u=0;u<4;++u)mb[u]=bits[bu+u]&blockMask[(bv>>2)+2*(bw>>2)];}
    else if(!g.ownership&&g.anatomy){ // the anatomy's z runs of the block's 16 columns (Grid::rawMaterialBase at the cells' centres)
     const auto&A=*g.anatomy;const core::Tile*tl=cache.find(t);
     for(int u=bu;u<bu+4;++u)for(int v=bv;v<bv+4;++v){const Cell c0{t.x*8+u,t.y*8+v,t.z*8+bw};if(!g.valid(c0))continue;const Vec q0=g.center(c0);
      const int ax=int(std::floor((q0.x-A.origin[0])/A.pitch)),ay=int(std::floor((q0.y-A.origin[1])/A.pitch));
      if(ax<0||ay<0||std::uint32_t(ax)>=A.size[0]||std::uint32_t(ay)>=A.size[1])continue;
      const auto col=std::size_t(ay)*A.size[0]+std::size_t(ax);const auto first=A.runs.begin()+A.offsets[col],last=A.runs.begin()+A.offsets[col+1];
      for(int w=bw;w<bw+4;++w){const Cell c{t.x*8+u,t.y*8+v,t.z*8+w};if(!g.valid(c))continue;const int az=int(std::floor((g.center(c).z-A.origin[2])/A.pitch));
       if(az<0||std::uint32_t(az)>=A.size[2])continue;auto it=std::upper_bound(first,last,az,[](int z,const auto&r){return z<int(r.lo);});
       if(it==first)continue;--it;if(!(az<int(it->hi)&&it->label))continue;
       if(tl&&((tl->removed[Grid::local(c)/64]>>(Grid::local(c)%64))&1u))continue;
       mb[u-bu]|=std::uint64_t(1)<<(v*8+w);}}}
    else for(int c=0;c<64;++c){const int u=bu+(c&3),v=bv+((c>>2)&3),w=bw+(c>>4);if(solidAt(Cell{t.x*8+u,t.y*8+v,t.z*8+w}))mb[u-bu]|=std::uint64_t(1)<<(v*8+w);}
    if(!(mb[0]|mb[1]|mb[2]|mb[3]))continue;
    if(bx<BX&&by<BY&&bz<BZ)blockG[bIdx(bx,by,bz)]=255;
    for(int s=0;s<8;++s){const int su=bu+(s&1)*2,sv=bv+((s>>1)&1)*2,sw=bw+(s>>2)*2;unsigned g2=255;bool s2=true;
     for(int c=0;c<8&&s2;++c){double m;const unsigned q=at(su+(c&1)*2,sv+((c>>1)&1)*2,sw+(c>>2)*2,false,m);if(g2==255)g2=q;s2=q==g2;}
     for(int c=0;c<8;++c){const int u=su+(c&1),v=sv+((c>>1)&1),w=sw+(c>>2);if(!((mb[u-bu]>>(v*8+w))&1u))continue;const Cell cc{t.x*8+u,t.y*8+v,t.z*8+w};
      if(!inBox(cc.x,cc.y,cc.z))continue;
      if(s2){L(cc.x,cc.y,cc.z)=std::uint8_t(g2);if(g2<p.groups)++groupBy[ch][g2];if(g2)nearSeen=true;}
      else if(p.body){L(cc.x,cc.y,cc.z)=253;eraseBy[ch].push_back(cc);nearSeen=true;} // a body burst: the mixed sub-block is crack whole
      else{const Vec q=g.center(cc);const unsigned ge=fn.label(q.x,q.y,q.z);++labelled[ch];L(cc.x,cc.y,cc.z)=std::uint8_t(ge);candBy[ch].push_back(cc);nearSeen=true;}}}}
   if(nearSeen){double r2=0;for(int c=0;c<8;++c){const Vec q=g.origin+Vec{(t.x*8+(c&1)*8)*pitch,(t.y*8+((c>>1)&1)*8)*pitch,(t.z*8+(c>>2)*8)*pitch};r2=std::max(r2,length(q-p.B));}
    if(r2>rMaxBy[ch])rMaxBy[ch]=r2;}}
  chunkMs[ch]+=core::ms(ct0);});
 double rMax=0;for(auto v:labelled)out.labelled+=v;for(auto v:rMaxBy)rMax=std::max(rMax,v);
 // the cut the pixel programs use: every near cell inside it (+ 2 cells), nothing beyond it can be near
 p.Rcut=std::min(p.Rcut,rMax+2*pitch);{const float f=float(p.Rcut);std::memcpy(&block->header[3],&f,4);}block->reach=p.Rcut;
 out.labelMs=core::ms(t0);const auto e0=core::Clock::now();
 if(std::getenv("GFY_HB_PROF"))std::cerr<<"HBPROF per_cell="<<perCell<<" ownership="<<(g.ownership?1:0)<<" table="<<(table?1:0)<<" class="<<tmClass<<" box="<<tmBox<<" chunks="<<chunkMs[0]<<","<<(W>1?chunkMs[1]:0)<<","<<(W>2?chunkMs[2]:0)<<","<<(W>3?chunkMs[3]:0)<<","<<(W>4?chunkMs[4]:0)<<" label="<<out.labelMs<<" box_tiles="<<boxTiles.size()<<" evals="<<out.labelled<<std::endl;
 out.groupCells.assign(p.groups,0);for(const auto&v:groupBy)for(unsigned k=0;k<p.groups;++k)out.groupCells[k]+=v[k];
 {std::vector<std::size_t> per(p.groups,0);auto&kept=block->section;
  for(const auto&v:sampleBy)for(std::size_t i=0;i+3<v.size();i+=4){const unsigned gr=unsigned(v[i+3]);if(gr<p.groups&&per[gr]++<512)kept.insert(kept.end(),v.begin()+std::ptrdiff_t(i),v.begin()+std::ptrdiff_t(i)+4);}}
 out.samples=block->section.size()/4;out.mixedTiles=mixed;
 // the crack: a cell of a sub-block whose corners differ (labelled exactly) that has a 6-neighbour of another group (a cell of a
 // one-group block counts as its group; one side of a contact is enough - two contacting cells of different groups never both stay)
 {auto groupAt=[&](int x,int y,int z)->unsigned{if(!inBox(x,y,z))return 0u;const auto l=lab[idx(x,y,z)];if(l<253)return l;if(l==253)return 253u;
   const unsigned bc=blockG[bIdx((x-ox)/4,(y-oy)/4,(z-oz)/4)];return bc<254?bc:255u;};
  core::parallelChunks(W,1,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t ch=a;ch<b;++ch)for(const Cell&c:candBy[ch]){const unsigned l=lab[idx(c.x,c.y,c.z)];
   const int nb[6][3]={{c.x-1,c.y,c.z},{c.x+1,c.y,c.z},{c.x,c.y-1,c.z},{c.x,c.y+1,c.z},{c.x,c.y,c.z-1},{c.x,c.y,c.z+1}};
   for(const auto&q:nb){const unsigned m=groupAt(q[0],q[1],q[2]);if(m<253&&m!=l){eraseBy[ch].push_back(c);break;}}}});
  for(const auto&v:eraseBy)for(const Cell&c:v)lab[idx(c.x,c.y,c.z)]=253;}
 std::vector<std::size_t> crackN(W);for(std::size_t ch=0;ch<W;++ch)crackN[ch]=eraseBy[ch].size(); // (the crumbs follow in eraseBy[0])
 // crumbs: a near group's pieces of tissue smaller than 6 cm3 once its cracks are out (a sliver the warp or a cavity cut off) are
 // blown away with the cracks - each fragment comes off as one chunk, no swarm of specks to fit and fly. A crumb lies in mixed blocks
 // only: a flood that reaches a block of its own group, the box's edge or 6 cm3 is a piece, not a crumb.
 // (off: the separation scraps every small fragment itself - HeadBurstPieces.h slivers() - and the pass cost ~8 ms; measured: the
 // same pieces. GFY_HB_CRUMBS=1 turns it on: test knob)
 static const bool crumbPass=std::getenv("GFY_HB_CRUMBS")&&std::atoi(std::getenv("GFY_HB_CRUMBS"))==1;
 if(crumbPass){const auto c0=core::Clock::now();
  const std::size_t crumbCells=std::size_t(std::ceil(6e-6/(pitch*pitch*pitch)));std::vector<std::uint8_t> seen(lab.size(),0);std::vector<Cell> stack,comp;
  for(int bz=0;bz<BZ;++bz)for(int by=0;by<BY;++by)for(int bx=0;bx<BX;++bx){if(blockG[bIdx(bx,by,bz)]!=255)continue;
   for(int w=0;w<64;++w){const int x=ox+bx*4+(w&3),y=oy+by*4+((w>>2)&3),z=oz+bz*4+(w>>4);const std::size_t i0=idx(x,y,z);const auto l=lab[i0];if(l==0||l>=253||seen[i0])continue;
    comp.clear();stack.assign(1,Cell{x,y,z});seen[i0]=1;bool big=false;std::size_t size=0;
    while(!stack.empty()&&!big){const Cell c=stack.back();stack.pop_back();++size;comp.push_back(c);if(size>crumbCells){big=true;break;}
     const int nb[6][3]={{c.x-1,c.y,c.z},{c.x+1,c.y,c.z},{c.x,c.y-1,c.z},{c.x,c.y+1,c.z},{c.x,c.y,c.z-1},{c.x,c.y,c.z+1}};
     for(const auto&q:nb){if(!inBox(q[0],q[1],q[2])){big=true;break;}const std::size_t j=idx(q[0],q[1],q[2]);const auto m=lab[j];
      if(m==l){if(seen[j]==2){big=true;break;}if(!seen[j]){seen[j]=1;stack.push_back(Cell{q[0],q[1],q[2]});}continue;}
      if(m==255&&blockG[bIdx((q[0]-ox)/4,(q[1]-oy)/4,(q[2]-oz)/4)]==l){big=true;break;}}}
    if(!big){for(const Cell&c:comp){lab[idx(c.x,c.y,c.z)]=253;eraseBy[0].push_back(c);}++out.crumbs;out.crumbCells+=comp.size();}
    else{for(const Cell&c:comp)seen[idx(c.x,c.y,c.z)]=2;for(const Cell&c:stack)seen[idx(c.x,c.y,c.z)]=2;}}}
  out.crumbMs=core::ms(c0);}
 {const float bandF=[&]{float f;std::memcpy(&f,&block->header[51],4);return f;}();
  if((p.ballisticBone||(p.body&&table&&!perCell))&&bandF>0.f&&bandF<1.f){const auto kb0=core::Clock::now();const double band=double(bandF);
   const std::uint32_t vs=fn.seed^0xB04Eu;const double vk=3.0/band;
   std::unordered_set<Cell,core::CellHash> keep,shell,broken;TissueLookup tissue{g,table};core::TileCache cache(g);
   auto kIdx=[&](int x,int y,int z){return (std::size_t(z-tlo[2])*KY+std::size_t(y-tlo[1]))*KX+std::size_t(x-tlo[0]);};
   auto protect=[&](const Cell&c){
    if(p.ballisticBone){
     if(g.bonePreviouslyHit(c)){broken.insert(c);return;}
     g.noteBoneStrike(c);
    }
    keep.insert(c);
   };
   std::size_t looked=0;
   for(const Cell&t:boxTiles){const unsigned tc=tileClass(t.x,t.y,t.z);if(tc==0u||tc==254u)continue;
    if(tc<254u){bool far=true; // a tile of one near group: skipped when every corner lies farther than the band from any other group
     for(int c=0;c<8&&far;++c){const int x=t.x+(c&1),y=t.y+((c>>1)&1),z=t.z+(c>>2);
      if(x<tlo[0]||y<tlo[1]||z<tlo[2]||x>thi[0]+1||y>thi[1]+1||z>thi[2]+1){far=false;break;}far=kM[kIdx(x,y,z)]>band+M8;}
     if(far)continue;}
    const auto*tile=cache.find(t);
    std::array<std::uint64_t,8> bone{};
    if(table){
     const auto it=table->find(t);
     if(it==table->end())continue;
     for(unsigned w=0;w<8;++w)bone[w]=it->second.bone[w]&it->second.solid[w];
    }else{
     // A continued piece can have no station table. Only its own, presently
     // solid skeletal material participates in the same partition.
     for(unsigned i=0;i<512;++i){
      const Cell c{t.x*8+int(i>>6),t.y*8+int((i>>3)&7),t.z*8+int(i&7)};
      if(g.valid(c)&&g.solidBase(c)&&g.renderMaterial(c)==2u)bone[i>>6]|=std::uint64_t(1)<<(i&63);
     }
    }
    const bool splitTile=table&&g.ownership&&!g.ownership->pureTile(Cell{t.x*8,t.y*8,t.z*8});
    for(int q=0;q<8;++q)for(std::uint64_t w=bone[std::size_t(q)]&~(tile?tile->removed[std::size_t(q)]:0ull);w;w&=w-1){
     const unsigned bit=ctz64(w);const Cell c{t.x*8+q,t.y*8+int(bit>>3),t.z*8+int(bit&7u)};if(splitTile&&!g.solidBase(c))continue;
     const Vec x=g.center(c);double gap;++looked;
     const unsigned gr=fn.zeroGap(x.x,x.y,x.z,gap);
     if(gr==0u){if(inBox(c.x,c.y,c.z)&&lab[idx(c.x,c.y,c.z)]==253)protect(c);continue;} // the far part's bone in a crack: not cut
     const double vary=.825+.175*head_burst_record::Function::vnoise(vs,x.x*vk,x.y*vk,x.z*vk);
     if(gap<=band*vary)protect(c);}}
   if(!keep.empty()||!broken.empty()){
    auto groupOf=[&](const Cell&c)->unsigned{if(inBox(c.x,c.y,c.z)){const auto l=lab[idx(c.x,c.y,c.z)];if(l<=253)return l;
      const unsigned bc=blockG[bIdx((c.x-ox)/4,(c.y-oy)/4,(c.z-oz)/4)];if(bc<254)return bc;}
     const Vec q=g.center(c);return fn.label(q.x,q.y,q.z);};
    static const int nb6[6][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    for(const Cell&c:keep)for(const auto&s:nb6){const Cell n{c.x+s[0],c.y+s[1],c.z+s[2]};
     if(keep.count(n)||broken.count(n)||!g.valid(n)||!tissue.solid(n)||cache.removed(n))continue;
     const unsigned gn=groupOf(n);if(gn!=0u&&gn!=253u)shell.insert(n);}
    // the crack lists without the kept bone, the shell after each list's cracks (before the crumbs)
    for(std::size_t ch=0;ch<W;++ch){std::vector<Cell> next;next.reserve(eraseBy[ch].size());std::size_t cracks=0;
     for(std::size_t i=0;i<eraseBy[ch].size();++i){const Cell&c=eraseBy[ch][i];if(keep.count(c)||broken.count(c))continue;next.push_back(c);if(i<crackN[ch])++cracks;}
     eraseBy[ch].swap(next);crackN[ch]=cracks;}
    std::vector<Cell> sh(shell.begin(),shell.end());std::sort(sh.begin(),sh.end(),[](const Cell&a,const Cell&b){return a.z!=b.z?a.z<b.z:(a.y!=b.y?a.y<b.y:a.x<b.x);});
    eraseBy[0].insert(eraseBy[0].begin()+std::ptrdiff_t(crackN[0]),sh.begin(),sh.end());crackN[0]+=sh.size();
    // A repeat removes this bone permanently. Put it outside the list of
    // restored crack-face samples, so the negative space survives the split.
    std::vector<Cell> chipped(broken.begin(),broken.end());
    std::sort(chipped.begin(),chipped.end(),[](const Cell&a,const Cell&b){
     return a.z!=b.z?a.z<b.z:(a.y!=b.y?a.y<b.y:a.x<b.x);
    });
    eraseBy[0].insert(eraseBy[0].end(),chipped.begin(),chipped.end());
    for(const Cell&c:chipped)if(inBox(c.x,c.y,c.z))lab[idx(c.x,c.y,c.z)]=253;
    for(const Cell&c:keep)if(inBox(c.x,c.y,c.z))lab[idx(c.x,c.y,c.z)]=0;
    for(const Cell&c:sh)if(inBox(c.x,c.y,c.z))lab[idx(c.x,c.y,c.z)]=253;
    // a fragment's sample point in kept bone would give the body that fragment's group: such samples go
    {auto&sec=block->section;std::vector<float> kept;kept.reserve(sec.size());const double inv=1./pitch;
     for(std::size_t i=0;i+3<sec.size();i+=4){const Cell c{int(std::floor((sec[i]-g.origin.x)*inv)),int(std::floor((sec[i+1]-g.origin.y)*inv)),int(std::floor((sec[i+2]-g.origin.z)*inv))};
      if(sec[i+3]!=0.f&&keep.count(c))continue;kept.insert(kept.end(),sec.begin()+std::ptrdiff_t(i),sec.begin()+std::ptrdiff_t(i)+4);}
     sec.swap(kept);}}
   out.boneKept=keep.size();out.boneShell=shell.size();out.boneMs=core::ms(kb0);
   if(std::getenv("GFY_M3_LOG"))std::cerr<<"bone_band band_mm="<<band*1000<<" bone_cells_looked="<<looked<<" kept="<<keep.size()<<" shell="<<shell.size()<<" repeat_broken="<<broken.size()<<" ms="<<out.boneMs<<std::endl;}}
 // the erase (journaled, in order), then the repair every cut gets
 std::vector<Cell> boundary;std::set<Cell> touched;std::size_t solid=0;
 // the crack cells erased here go into the record (after the group samples; header word 12 = the samples' float count, words 16-18
 // the box's first cell, 20-21 its size in x, y; one float per cell = its index in the box): the separation gives them back to each
 // piece's removal field, so the pixel programs draw the record's torn face itself, never the cells' 2 mm steps (HeadBurstPieces.h)
 auto&sec=block->section;block->header[12]=std::uint32_t(sec.size());
 // + the cells the shots removed INSIDE the fragments (a slug's channel and cavity: up to a litre): the head is blown apart around
 // them, each fragment a SOLID chunk torn by the record, never a hollow shell or a smooth tube of 2 mm steps; the cells within 15 mm
 // of this shot's entry and exit stay removed (the holes in the skin)
 std::vector<Cell> pre;
 // (a body burst lists none: its separation gives back every removed cell of the near groups itself, tile by tile - a point-blank
 // blast's craters are a million cells; HeadBurstPieces.h)
 if(!p.body){const auto q0=core::Clock::now();using TilePair=std::remove_reference_t<decltype(*g.tiles.cbegin())>;std::vector<const TilePair*> tl;
  for(const auto&t:g.tiles){const Cell&tk=t.first;if(tk.x<tlo[0]||tk.x>thi[0]||tk.y<tlo[1]||tk.y>thi[1]||tk.z<tlo[2]||tk.z>thi[2])continue;
   std::uint64_t any=0;for(auto w:t.second.removed)any|=w;if(any)tl.push_back(&t);}
  std::vector<std::vector<Cell>> preBy(W);const double hole2=.015*.015;
  core::parallelChunks(tl.size(),1,[&](std::size_t a,std::size_t b,std::size_t ch){for(std::size_t ti=a;ti<b;++ti){const Cell&tk=tl[ti]->first;const auto&tile=tl[ti]->second;
   for(unsigned w=0;w<8;++w)for(std::uint64_t bits=tile.removed[w];bits;bits&=bits-1){const unsigned l=w*64+ctz64(bits);
    const Cell c{tk.x*8+int(l>>6),tk.y*8+int((l>>3)&7u),tk.z*8+int(l&7u)};const Vec q=g.center(c);
    const double dx=q.x-fn.B[0],dy=q.y-fn.B[1],dz=q.z-fn.B[2];if(dx*dx+dy*dy+dz*dz>fn.Rcut*fn.Rcut)continue;
    const Vec e1=q-p.entry,e2=q-p.exit;if(!p.body&&(dot(e1,e1)<hole2||dot(e2,e2)<hole2))continue;
    const unsigned gl=fn.label(q.x,q.y,q.z);if(gl==0||gl>=253||!g.valid(c)||!g.materialBase(c))continue;preBy[ch].push_back(c);}}});
  for(auto&v:preBy)pre.insert(pre.end(),v.begin(),v.end());
  out.preMs=core::ms(q0);out.preCells=pre.size();}
 int cl[3]={INT_MAX,INT_MAX,INT_MAX},cm[3]={INT_MIN,INT_MIN,INT_MIN}; // the listed cells' own box (the label box can hold the shoulders)
 for(std::size_t ch=0;ch<W;++ch)for(std::size_t i=0;i<crackN[ch];++i){const Cell&c=eraseBy[ch][i];const int q[3]={c.x,c.y,c.z};for(int k=0;k<3;++k){cl[k]=std::min(cl[k],q[k]);cm[k]=std::max(cm[k],q[k]);}}
 for(const Cell&c:pre){const int q[3]={c.x,c.y,c.z};for(int k=0;k<3;++k){cl[k]=std::min(cl[k],q[k]);cm[k]=std::max(cm[k],q[k]);}}
 const long long cnx=cm[0]>=cl[0]?cm[0]-cl[0]+1:0,cny=cm[1]>=cl[1]?cm[1]-cl[1]+1:0,cnz=cm[2]>=cl[2]?cm[2]-cl[2]+1:0;
 const bool listCrack=cnx>0&&cny>0&&cnz>0&&cnx<65536&&cny<65536&&cnz<65536,wide=double(cnx)*double(cny)*double(cnz)>=16777216.; // wide: 2 floats per cell
 if(listCrack){block->header[16]=std::uint32_t(cl[0]);block->header[17]=std::uint32_t(cl[1]);block->header[18]=std::uint32_t(cl[2]);block->header[20]=std::uint32_t(cnx)|(wide?0x80000000u:0u);block->header[21]=std::uint32_t(cny);}
 auto putCell=[&](const Cell&c){const std::uint64_t v=(std::uint64_t(c.z-cl[2])*std::uint64_t(cny)+std::uint64_t(c.y-cl[1]))*std::uint64_t(cnx)+std::uint64_t(c.x-cl[0]);
  if(wide){sec.push_back(float(v>>12));sec.push_back(float(v&4095u));}else sec.push_back(float(v));};
 for(std::size_t ch=0;ch<W;++ch)for(std::size_t i=0;i<eraseBy[ch].size();++i){const Cell&c=eraseBy[ch][i];if(!g.valid(c)||!g.erase(c))continue;boundary.push_back(c);touched.insert(Grid::tileKey(c));++out.mask;
  if(listCrack&&i<crackN[ch])putCell(c);
  if(const auto label=g.materialBase(c)){++solid;const auto slot=g.material_slots&&label<g.material_slots->size()?unsigned((*g.material_slots)[label]&63u):1u;++out.bySlot[slot];}}
 if(listCrack)for(const Cell&c:pre)putCell(c);
 out.erased=boundary.size();out.eraseMs=core::ms(e0);const auto r0=core::Clock::now();
 std::size_t extraSolid=0;
 auto onCrack=[&](Cell c){if(!inBox(c.x,c.y,c.z))return false; // the crack and the cells beside it
  for(int dz=-1;dz<=1;++dz)for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx)if(inBox(c.x+dx,c.y+dy,c.z+dz)&&L(c.x+dx,c.y+dy,c.z+dz)==253)return true;return false;};
 if(!boundary.empty()){try{out.edgeCells=resolveEdgeContacts(g,boundary,p.B,onCrack,touched,&extraSolid);}catch(const RecoverableImpactError&){out.refusal="tear_repair_budget";}}
 out.cells=solid+extraSolid;out.mask+=out.edgeCells;out.repairMs=core::ms(r0);
 out.dirty=core::expandDirtyTiles(touched);
 // the stations whose tiles changed are measured again (as every tear does)
 std::size_t total=0;for(const auto&s2:segments)total+=s2.stations.size();
 if(!out.dirty.empty()&&state.measured.size()==total){std::size_t k2=0;
  for(const auto&s2:segments)for(const auto&station:s2.stations){const std::size_t kk=k2++;if(!state.measured[kk])continue;
   for(const Cell&t:out.dirty)if(t.x>=station.tileLo.x&&t.x<=station.tileHi.x&&t.y>=station.tileLo.y&&t.y<=station.tileHi.y&&t.z>=station.tileLo.z&&t.z<=station.tileHi.z){state.measured[kk]=0;break;}}}
 // the record (a repair refusal travels with it, as every tear's does)
 {ResolvedTunnel rec;const Vec n{0,1,0},a{1,0,0},b{0,0,1};const double reach=p.Rcut;const Vec e=p.B-n*reach;
  rec.entry={e.x,e.y,e.z};rec.direction={n.x,n.y,n.z};rec.edge_u={a.x,a.y,a.z};rec.edge_v={b.x,b.y,b.z};rec.radius=p.rf;rec.penetration=2*reach;rec.cap=2*reach;rec.inlet=0;
  rec.exit_distance=2*reach;rec.exit_mult=1;rec.edge_seed=p.seed;auto surf=std::make_shared<ResolvedTearSurface>();surf->decide=block;rec.surface=std::move(surf);
  out.records.push_back(std::move(rec));}
 out.done=out.erased>0;if(!out.done)out.refusal="head_burst_nothing_erased";
 out.ms=core::ms(t0);return out;}
} // namespace head_burst
