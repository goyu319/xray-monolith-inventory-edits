#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <unordered_set>
#include <vector>
int seam_rule_value() noexcept; // damagecut: console painter_seam_rule (LiveDamageWorker.cpp): 1 (default) on, 0 off
namespace seam_tear {
inline bool logOn(){static const bool on=std::getenv("GFY_SEAM_LOG")!=nullptr||std::getenv("GFY_M3_LOG")!=nullptr||std::getenv("GFY_TEAR_SELECT_LOG")!=nullptr;return on;}
inline bool off(){static const bool o=std::getenv("GFY_SEAM_OFF")!=nullptr;return o||seam_rule_value()==0;} // probe A/B knob
// the rule's numbers (see the header)
constexpr double parallelCos=.9659258262890683; // 15 degrees
constexpr double planeTol=.02,deepTol=.03,deepAt=.12,gapMax=.10,endReach=.08,spanMin=.15,tissueLeft=.45,boneLeft=.40,pixel=.004;
constexpr std::size_t chainMin=5;
constexpr double stretchLen=.12; // the seam's stretches (a station's measure along the line)
constexpr double openDepthScale=.2; // an open split: the torn ends pulled back a fifth of a separated tear's (the sides still hold together)
// seam_reach at the hit (Impact -> the primary record): the tissue a round wrecks around its track, m
inline float reachAt(const Impact&e){
 const double r=std::max(.0005,e.radius);
 if(e.melee){if(e.melee==4)return float(std::clamp(1.5*r,.004,.03));return float(r+.003);} // blunt bores; blades: their kerf
 if(e.slab)return float(r+.003);
 if(e.blast)return float(std::clamp(3.*r,.006,.02)); // grenade fragments / craters
 if(e.round_class==0)return float(std::clamp(3.5*r,.010,.018));
 if(e.round_class==1)return float(std::clamp(6.5*r,.016,.030));
 if(e.round_class>=2)return float(std::clamp(6.5*r,.022,.035));
 return float(std::clamp(6.*r,.010,.032));} // class not given: by the hole
inline double reachOf(const ResolvedTunnel&t){return t.seam_reach>0?double(t.seam_reach):std::clamp(6.*t.radius,.010,.032);}
// SeverState::seams entry (16 doubles): C(3) n(3) t(3) carved t0, t1 (frame coordinates), seed, full, spare
enum:int{S_C=0,S_N=3,S_T=6,S_T0=9,S_T1=10,S_SEED=11,S_FULL=12};
struct Chan {Vec a,d;double len,r,rw;bool fresh,slab;};
inline double segDist(const Vec&p,const Vec&a,const Vec&d,double len){const double s=std::clamp(dot(p-a,d),0.,len);return length(p-(a+d*s));}
// solid / bone of a cell from the model's tile table (TissueLookup's rule): 0 none, 1 tissue, 2 bone
struct MatLookup {
 const Grid&g;const std::unordered_map<Cell,SeverState::TileMaterial,core::CellHash>*table;Cell key{INT_MIN,INT_MIN,INT_MIN};const SeverState::TileMaterial*m=nullptr;
 int at(const Cell&c){if(!g.valid(c))return 0;
  if(!table||(g.ownership&&!g.ownership->pureTile(c))){if(!g.solidBase(c))return 0;const auto l=g.rawMaterialBase(c);
   return (g.material_slots&&l<g.material_slots->size()&&(*g.material_slots)[l]==2)?2:1;}
  const Cell k=Grid::tileKey(c);if(!(k==key)){key=k;const auto it=table->find(k);m=it==table->end()?nullptr:&it->second;}
  if(!m)return 0;const auto l=Grid::local(c);if(!((m->solid[l/64]>>(l%64))&1u))return 0;return ((m->bone[l/64]>>(l%64))&1u)?2:1;}
};
struct Outcome {bool full=false,open=false;DamageTearVerdict v;double rt=1,rb=1,ms=0;std::size_t chain=0;std::uint16_t bone=65535;};

// The decide-once tear along the seam (TearDecide.h's decision with this frame and footprint): full = sheet + band proof (the
// separation follows), else the open split's torn gap erased as cells (depth scale 0.2). inFoot(a, b): the footprint in the
// decision frame (C, u, w = n x u).
template<class InFoot,class InCarve>
inline bool decide(Grid&g,SeverState&state,const painter_detail::TissueField*closure,const Vec&C,const Vec&n,const Vec&u,double Rl,
    const InFoot&inFoot,const InCarve&inCarve,std::uint64_t seed64,const Vec&fire,bool full,const std::vector<SeverSegment>&segments,
    DamageTearVerdict&v,const char*&refusal){
 using namespace tear_decide;const auto t0=core::Clock::now();const double pitch=g.pitch;
 const Vec w=cross(n,u);const std::uint32_t seed=std::uint32_t(seed64^(seed64>>32));
 Decision D;D.pitch=pitch;D.seed=seed;double*P=D.P;
 const double maxD=tissue_tear::limbDepth,H=std::max(.07,maxD+.03),ripR=.005,ds=full?1.:openDepthScale;
 for(int k=0;k<3;++k){P[P_C+k]=C[k];P[P_N+k]=n[k];P[P_U+k]=u[k];P[P_W+k]=w[k];}
 P[P_RL]=Rl;P[P_H]=H;P[P_MAXD]=maxD;P[P_RIP]=ripR;P[P_DSCALE]=ds;
 double ia=dot(fire,u),ib=dot(fire,w);const double along=dot(fire,n);
 {double l2=std::sqrt(ia*ia+ib*ib);if(l2<.2){const double th=6.283185307179586*tissue_tear::u01(seed,5,5,0x77u);ia=std::cos(th);ib=std::sin(th);l2=1;}ia/=l2;ib/=l2;}
 const double tilt=(2*tissue_tear::u01(seed,6,6,0x78u)-1)*.016;
 double mbase[2];for(int sd=0;sd<2;++sd){const std::uint32_t s=seed^(sd?0x51u:0xA3u);double m=0;for(unsigned q=0;q<8;++q)m+=tissue_tear::u01(s,int(100+q),7,0x3C1u);mbase[sd]=tissue_tear::limbK::groupFloor+tissue_tear::limbK::groupSpan*m/8;}
 P[P_IMP]=ia;P[P_IMP+1]=ib;P[P_IMP+2]=along;P[P_IMP+3]=tilt;P[P_IMP+4]=.06;P[P_IMP+5]=maxD;P[P_IMP+6]=mbase[0];P[P_IMP+7]=mbase[1];
 // the muscle fibres: along the nearest segment's axis inside it, else the body's long axis (rest space: up)
 Vec A0{0,1,0};{double best=1e30;for(const auto&s:segments){const double tt=std::clamp(dot(C-s.pivot,s.axis),0.,s.length);const double dd=length(C-(s.pivot+s.axis*tt))-s.radius;
   if(dd<best){best=dd;if(dd<.02)A0=s.axis;}}}
 {Vec p1=cross(A0,Vec{0,0,1});const double l1=dot(p1,p1);p1=l1>.04?p1*(1./std::sqrt(l1)):unit(cross(A0,Vec{1,0,0}));const Vec p2=cross(A0,p1);
  static const double tiltDeg[8]={2.5,17.5,7.5,35,25,12.5,42.5,5},azDeg[8]={0,40,200,110,300,150,250,80};const double rad=3.141592653589793/180;
  const Vec dimp=u*ia+w*ib;
  for(unsigned c=0;c<NCLS;++c){bool fib=false;Vec d=A0;
   if(c>=MV0&&c<MV0+8){const double t=tiltDeg[c-MV0]*rad,z=azDeg[c-MV0]*rad;d=unit(A0*std::cos(t)+p1*(std::sin(t)*std::cos(z))+p2*(std::sin(t)*std::sin(z)));fib=true;}
   else if(c==MUSCLE)fib=true;
   Vec f1=u,f2=w;double ca=ia,cb=ib;
   if(fib){const Vec r=std::abs(d.x)<.6?Vec{1,0,0}:(std::abs(d.y)<.6?Vec{0,1,0}:Vec{0,0,1});f1=unit(cross(d,r));f2=cross(d,f1);ca=dot(dimp,f1);cb=dot(dimp,f2);}
   const int k=P_FE+8*int(c);for(int a=0;a<3;++a){P[k+a]=f1[a];P[k+3+a]=f2[a];}P[k+6]=ca;P[k+7]=cb;}}
 auto sample=[&](double x,double y,double z){return closure->sample(x,y,z);};
 auto parallel=[&](std::size_t count,const auto&body){core::parallelChunks(count,1,[&](std::size_t a,std::size_t b,std::size_t){body(a,b);});};
 auto allow=[&](double a,double b){return a*a+b*b<=Rl*Rl&&inFoot(a,b);};
 SectionCache cache;buildSectionFast(D,sample,parallel,allow,cache);
 for(int q=0;q<7;++q)P[P_SEEDC+q]=0.; // the seam's surface is its plane (the hits' line): no tilt, bow or wave
 const double sectionMs=core::ms(t0);
 if(D.limbCells==0){refusal="seam_empty_section";return false;}
 const double band=.014,h=1.25*pitch;
 const double proofPad=.20; // the proof and the contact walls reach 20 cm past the footprint (touching thighs run on below the crotch)
 auto offFoot=[&](double a,double b){if(D.inFoot(a,b))return false;const double e=.008;for(int q=0;q<9;++q)if(q!=4&&D.inFoot(a+e*(q%3-1),b+e*(q/3-1)))return false;return true;};
 // contact walls: off the footprint, where two limbs that are not neighbours along the skeleton touch across the plane (the inner
 // thighs under a sagittal split: skin against skin, joined only because the cells touch) a thin wall goes with the sheet
 std::vector<char> nbr(segments.size()*segments.size(),0);
 for(std::size_t a=0;a<segments.size();++a)for(std::size_t b=0;b<segments.size();++b){const auto&o=segments[a];const auto&q=segments[b];const Vec oe=o.pivot+o.axis*o.length,qe=q.pivot+q.axis*q.length;
  nbr[a*segments.size()+b]=a==b||length(qe-o.pivot)<.04||length(oe-q.pivot)<.04||length(q.pivot-o.pivot)<.02||length(qe-oe)<.02;}
 auto segAt=[&](const Vec&p,double&dist){int k=-1;dist=1e30;for(std::size_t i=0;i<segments.size();++i){const auto&sg=segments[i];const double tt=std::clamp(dot(p-sg.pivot,sg.axis),0.,sg.length);
   const double dd=length(p-(sg.pivot+sg.axis*tt))-sg.radius;if(dd<dist){dist=dd;k=int(i);}}return k;};
 auto wallAt=[&](const Vec&p){double da,db;const int a=segAt(p+n*.01,da),b=segAt(p-n*.01,db);
  return a>=0&&b>=0&&a!=b&&da<.03&&db<.03&&!nbr[std::size_t(a)*segments.size()+std::size_t(b)];};
 std::size_t wallCells=0;
 if(full&&!bridgeOff()){
  const double hB=.51*std::max(std::abs(n.x),std::max(std::abs(n.y),std::abs(n.z)))*pitch,hT=h; // the thinnest parting sheet; the old sheet's half thickness
  const double gl[2]={D.footLo[0]-proofPad,D.footLo[1]-proofPad},gh[2]={D.footHi[0]+proofPad,D.footHi[1]+proofPad};int lo[3],hi[3];
  for(int k=0;k<3;++k){double a=1e30,b=-1e30;for(int q=0;q<8;++q){const double A=(q&1)?gh[0]:gl[0],B=(q&2)?gh[1]:gl[1],S=(q&4)?band:-band;const double p=C[k]+A*u[k]+B*w[k]+S*n[k];a=std::min(a,p);b=std::max(b,p);}
   lo[k]=std::max(0,int(std::floor((a-g.origin[k])/pitch)));hi[k]=std::min((&g.size.x)[k]-1,int(std::floor((b-g.origin[k])/pitch)));if(lo[k]>hi[k]){refusal="seam_outside_grid";return false;}}
  const auto p0=core::Clock::now();
  const Across across{u,w,{gl[0],gh[0],gl[1],gh[1]}};const SlabColumns SC=slabColumns(g,lo,hi,C,n,-band,band,&across);const std::size_t NC=SC.size();
  const std::size_t KL=std::size_t(SC.KL),NJ=std::size_t(SC.nJ),rowSpan=std::max<std::size_t>(1,NJ*KL);
  // st: 0 no band tissue, 1 the sheet (a standing cell on the plane) or a contact wall, 2 removed, 3 standing, 4 standing inside the
  // old sheet's thickness; + 8: standing tissue past the sheet (the proof's goal). A row of columns is scanned by one thread, so a
  // row's lists keep the scan's order: the same cells in the same order every run, without a sort
  std::vector<std::uint8_t> st(NC,0);std::vector<std::vector<Cell>> rowBridge(std::size_t(SC.nI)),rowWall(std::size_t(SC.nI));
  std::vector<std::vector<std::size_t>> plus(core::parallelWidth());std::vector<std::size_t> woundBy(core::parallelWidth(),0);
  struct LkB{TissueLookup tissue;core::TileCache cache;};
  slabColumnsScan(SC,[&]{return LkB{TissueLookup{g,state.table},core::TileCache(g)};},[&](LkB&k,const Cell&c,std::size_t idx,std::size_t chunk){
   const Vec p=g.center(c);const Vec d=p-C;const double s=dot(d,n),as=std::abs(s);if(as>band)return; // the surface is the plane itself (no relief): s is the distance from it
   const double pa=dot(d,u),pb=dot(d,w);if(pa<gl[0]||pa>gh[0]||pb<gl[1]||pb>gh[1])return;
   if(!k.tissue.solid(c))return;if(k.cache.removed(c)){st[idx]=2;if(as<=hB&&D.inFoot(pa,pb))++woundBy[chunk];return;} // (counted: the sheet's cells the wounds already took)
   const bool foot=D.inFoot(pa,pb);
   if(foot&&as<=hB){st[idx]=1;rowBridge[idx/rowSpan].push_back(c);return;}
   if(!foot&&as<=hT+pitch&&wallAt(p)){st[idx]=1;rowWall[idx/rowSpan].push_back(c);return;}
   st[idx]=std::uint8_t(((foot&&as<=hT)?4u:3u)|(s<-(hT+pitch)?8u:0u));if(foot&&s>band-1.5*pitch)plus[chunk].push_back(idx);});
  const double scanMs=core::ms(p0);
  // the proof (the same band, faces, goal and contact walls as before): the flood from the + faces of the footprint through standing
  // tissue must not reach standing tissue past the sheet. The thin sheet first; when it leaves the sides joined, the old sheet's
  // thickness (so a split is never refused where the old sheet parts the sides: its standing cells are the bridge then)
  bool thick=false;std::size_t floodVisited=0;
  {std::size_t at=~std::size_t(0);
   auto flood=[&](bool old)->bool{std::vector<std::uint64_t> seen((NC+63)/64,0);std::vector<std::size_t> stack;
    auto push=[&](std::size_t q){const unsigned t=st[q]&7u;if(!(t==3u||(!old&&t==4u))||((seen[q>>6]>>(q&63))&1u))return;seen[q>>6]|=std::uint64_t(1)<<(q&63);stack.push_back(q);};
    for(const auto&F:plus)for(const auto j:F)push(j);
    while(!stack.empty()){const std::size_t j=stack.back();stack.pop_back();++floodVisited;if(st[j]&8u){at=j;return true;}
     const std::size_t col=j/KL;const int kk=int(j%KL),K=SC.k0[col]+kk,I=int(col/NJ),J=int(col%NJ);
     auto other=[&](std::size_t c2){const int k2=K-SC.k0[c2];if(k2>=0&&k2<SC.len[c2])push(c2*KL+std::size_t(k2));};
     if(kk>0)push(j-1);if(kk+1<SC.len[col])push(j+1);
     if(I>0)other(col-NJ);if(I+1<SC.nI)other(col+NJ);if(J>0)other(col-1);if(J+1<SC.nJ)other(col+1);}
    return false;};
   bool linked=flood(false);if(linked){linked=flood(true);thick=!linked;}
   if(linked){refusal="seam_not_through";
    if(logOn()){const Cell c=SC.cell(at);const Vec d=g.center(c)-C;const double pa=dot(d,u),pb=dot(d,w);
     std::cerr<<"seam_tear proof linked bridge=1 visited="<<floodVisited<<" reached pa="<<pa<<" pb="<<pb<<" s="<<dot(d,n)<<" foot="<<D.inFoot(pa,pb)
      <<" at=("<<g.center(c).x<<","<<g.center(c).y<<","<<g.center(c).z<<") section_ms="<<sectionMs<<" proof_ms="<<core::ms(p0)<<std::endl;}
    return false;}}
  const double proofMs=core::ms(p0);
  std::vector<Cell> bridgeCells,wallList;
  {std::size_t nb=0,nw=0;for(const auto&r:rowBridge)nb+=r.size();for(const auto&r:rowWall)nw+=r.size();bridgeCells.reserve(nb);wallList.reserve(nw);
   for(const auto&r:rowBridge)bridgeCells.insert(bridgeCells.end(),r.begin(),r.end());for(const auto&r:rowWall)wallList.insert(wallList.end(),r.begin(),r.end());}
  if(thick){for(std::size_t q=0;q<NC;++q)if((st[q]&7u)==4u)bridgeCells.push_back(SC.cell(q));std::sort(bridgeCells.begin(),bridgeCells.end(),cellLess);}
  const double hCut=thick?hT:hB;const std::size_t listed=bridgeCells.size()+wallList.size();
  std::size_t woundCells=0;for(const auto x:woundBy)woundCells+=x; // the log's read-out: the cut's cells that were already open (the damage) beside its bridge
  // the per-seed constants the sheet never read (finishSeeds) run beside the maps, the erase and its repair (decideOnce's way): they
  // write the decision's seed constants and shell floor only; joined before the GPU block and on every way out of this block
  std::shared_ptr<tear_bake::Pending> seedsJob;struct SeedsJoinB{std::shared_ptr<tear_bake::Pending>&p;~SeedsJoinB(){if(p)p->wait();}} seedsJoin{seedsJob};
  seedsJob=std::make_shared<tear_bake::Pending>();
  seedsJob->chunks.push_back([&D]{auto par=[](std::size_t count,const auto&body){core::parallelChunks(count,1,[&](std::size_t a,std::size_t b,std::size_t){body(a,b);});};finishSeeds(D,par);});
  tear_bake::service().submit(seedsJob);
  // the bridge's maps before its cells go: the drawn region, the cap per side, the taper at the wounds (HoldingCut.h)
  const auto m0=core::Clock::now();std::size_t woundPx=0;
  D.bridgeH=bridgeMaps(D,g,state,bridgeCells,bridgeRip(pitch),false,&woundPx);v.bridgeCells=bridgeCells.size();v.holding=bridgeCells.size();
  const double mapsMs=core::ms(m0);
  // the cells go into the grid (journaled erase). The edge repair every cut gets may take cells of the sheet itself only (the
  // station bridge's rule, decideOnce: the clothing's air at the sheet's rim) - never a standing cell beside the sheet, which a
  // thin sheet on a tilted plane touches edge to edge at every step (the old repair reaches one cell past its thick sheet)
  const auto e0=core::Clock::now();std::vector<Cell> boundary;boundary.reserve(listed);std::set<Cell> touched;std::size_t solid=0;Cell lastKey{INT_MIN,INT_MIN,INT_MIN};
  for(const std::vector<Cell>*list:{&bridgeCells,&wallList})for(const Cell&c:*list){if(!g.valid(c)||!g.erase(c))continue;boundary.push_back(c);
   const Cell key=Grid::tileKey(c);if(!(key==lastKey)){touched.insert(key);lastKey=key;}++v.mask;
   if(const auto label=g.materialBase(c)){++solid;const auto slot=g.material_slots&&label<g.material_slots->size()?unsigned((*g.material_slots)[label]&63u):1u;++v.bySlot[slot];}}
  const double eraseOnlyMs=core::ms(e0);std::size_t extraSolid=0;const auto r0=core::Clock::now();
  if(!boundary.empty()){
   try{v.edgeCells=resolveEdgeContacts(g,boundary,C,[&](Cell c){const Vec d=g.center(c)-C;return std::abs(dot(d,n))<=hCut&&D.inFoot(dot(d,u),dot(d,w));},touched,&extraSolid);}
   catch(const RecoverableImpactError&){v.refusal="tear_repair_budget";}}
  const double repairMs=core::ms(r0);
  v.cells=solid+extraSolid;v.mask+=v.edgeCells;v.dirty=core::expandDirtyTiles(touched);const double eraseMs=core::ms(e0);
  std::size_t stations=0;for(const auto&s2:segments)stations+=s2.stations.size();
  if(!v.dirty.empty()&&state.measured.size()==stations){std::size_t k2=0;
   for(const auto&s2:segments)for(const auto&station:s2.stations){const std::size_t kk=k2++;if(!state.measured[kk])continue;
    for(const Cell&t:v.dirty)if(t.x>=station.tileLo.x&&t.x<=station.tileHi.x&&t.y>=station.tileLo.y&&t.y<=station.tileHi.y&&t.z>=station.tileLo.z&&t.z<=station.tileHi.z){state.measured[kk]=0;break;}}}
  // the block (a bridge tear's rip gap and slab), its bake beside the separation when the job keeps the closure, the record - as below
  seedsJob->wait();
  D.P[P_RIP]=bridgeRip(pitch);D.P[P_H]=D.bridgeH;
  v.point=C;v.axis=n;v.tissueRan=true;v.tissueMs=core::ms(t0);v.tear=!v.refusal;
  auto block=gpuBlock(D);
  static const double bakeMmB=[]{const char*e=std::getenv("GFY_M3_BAKE_MM");const double x=e?std::atof(e):4.;return std::clamp(x,1.,8.);}();
  static const double mapMmB=[]{const char*e=std::getenv("GFY_M3_MAP_MM");const double x=e?std::atof(e):4.;return std::clamp(x,1.,8.);}();
  Bake2Stats bst;bool bakeAsync=false;const char*dumpDir=std::getenv("GFY_M3_DUMP");static std::atomic<unsigned> dumpCountB{3u<<20};const unsigned dumpIdx=dumpDir?dumpCountB.fetch_add(1):0u;
  {const auto&keep=tear_bake::jobKeep();
   if(!tear_bake::syncOnly()&&keep&&keep.get()==static_cast<const void*>(closure)){bakeAsync=true;const auto*cl=closure;
    auto sampleV=[cl](double x,double y,double z){return cl->sample(x,y,z);};const std::string dd=dumpDir?dumpDir:"";const bool log=logOn();const std::uint32_t sd=seed;
    auto pend=bakeTearChunks(std::make_shared<const Decision>(D),bakeMmB*.001,mapMmB*.001,sampleV,block,[block,dd,dumpIdx,log,sd](const Bake2Stats&s,double ms){
     if(!dd.empty())writeBakeDump(dd.c_str(),dumpIdx,*block);
     if(log)std::cerr<<"seam_bake_done seed="<<sd<<" ms="<<ms<<" bake="<<block->bake_dim[0]<<"x"<<block->bake_dim[1]<<"x"<<block->bake_dim[2]<<" evaluated="<<s.fieldEvaluated<<std::endl;});
    pend->keep=keep;bst.mapBytes=block->map.size()*4;bst.fieldBytes=block->bake.size();tear_bake::start(block.get(),pend);if(dumpDir)pend->wait();}
   else bakeTear(D,bakeMmB*.001,mapMmB*.001,sample,parallel,*block,bst);}
  {ResolvedTunnel rec;const double reach=H+.025;const Vec e=C-n*reach;
   rec.entry={e.x,e.y,e.z};rec.direction={n.x,n.y,n.z};rec.edge_u={u.x,u.y,u.z};rec.edge_v={w.x,w.y,w.z};rec.radius=Rl;rec.penetration=2*reach;rec.cap=2*reach;rec.inlet=0;
   rec.exit_distance=2*reach;rec.exit_mult=1;rec.edge_seed=seed64;auto surf=std::make_shared<ResolvedTearSurface>();surf->decide=block;rec.surface=std::move(surf);v.records.push_back(std::move(rec));}
  if(dumpDir){dump(D,dumpDir,dumpIdx);if(!bakeAsync)writeBakeDump(dumpDir,dumpIdx,*block);}
  v.tissueMs=core::ms(t0);
  if(v.tear){++state.decided;std::size_t k=0;const double doneR=.02;
   if(state.failed.size()==stations)for(const auto&s:segments)for(const auto&station:s.stations){const Vec d=station.center-C;
    if(std::abs(dot(d,n))<=doneR&&D.inFoot(dot(d,u),dot(d,w)))state.failed[k]=1;++k;}}
  if(logOn())std::cerr<<"seam_tear full ok="<<v.tear<<" seed="<<seed<<" N="<<D.N<<" limb_cells="<<D.limbCells<<" Rl="<<Rl<<" section_ms="<<sectionMs
   <<" sampled="<<cache.sampled<<" proof_ms="<<proofMs<<" flood_visited="<<floodVisited<<" erase_ms="<<eraseMs<<" (scan "<<scanMs<<" erase "<<eraseOnlyMs<<" repair "<<repairMs<<")"<<" cells="<<listed<<" erased="<<boundary.size()
   <<" edge_cells="<<v.edgeCells<<" bake_async="<<bakeAsync<<" bake_ms="<<bst.ms<<" map="<<block->map_dim[0]<<"x"<<block->map_dim[1]<<" bake="<<block->bake_dim[0]<<"x"<<block->bake_dim[1]<<"x"<<block->bake_dim[2]
   <<" bridge="<<bridgeCells.size()<<" sheet_wound="<<woundCells<<" walls="<<wallList.size()<<" solid="<<solid<<" extra_solid="<<extraSolid<<" thick="<<thick<<" wound_px="<<woundPx<<" maps_ms="<<mapsMs<<" emax_mm="<<(D.bridgeH-.03)*1e3
   <<" plane=("<<C.x<<","<<C.y<<","<<C.z<<";"<<n.x<<","<<n.y<<","<<n.z<<") h_mm="<<hCut*1e3<<" block_bytes="<<(block->params.size()*4+block->section.size()*4+block->map.size()*4+block->bake.size()+block->header.size()*4)
   <<" ms="<<core::ms(t0)<<std::endl;
  return v.tear;}
 // ---- full: the band proof: the flood from the +n face of the footprint through standing tissue of the band must not reach standing
 // tissue past the sheet (-n side) anywhere in the footprint's rectangle + 20 cm (around the sheet's edge, through touching parts)
 const auto p0=core::Clock::now();bool linked=false;std::size_t floodVisited=0;
 if(full){const double pad=proofPad;const double gl[2]={D.footLo[0]-pad,D.footLo[1]-pad},gh[2]={D.footHi[0]+pad,D.footHi[1]+pad};int lo[3],hi[3];
  for(int k=0;k<3;++k){double a=1e30,b=-1e30;for(int q=0;q<8;++q){const double A=(q&1)?gh[0]:gl[0],B=(q&2)?gh[1]:gl[1],S=(q&4)?band:-band;const double p=C[k]+A*u[k]+B*w[k]+S*n[k];a=std::min(a,p);b=std::max(b,p);}
   lo[k]=std::max(0,int(std::floor((a-g.origin[k])/pitch)));hi[k]=std::min((&g.size.x)[k]-1,int(std::floor((b-g.origin[k])/pitch)));if(lo[k]>hi[k]){refusal="seam_outside_grid";return false;}}
  const Across across{u,w,{gl[0],gh[0],gl[1],gh[1]}};const SlabColumns SC=slabColumns(g,lo,hi,C,n,-band,band,&across);const std::size_t NC=SC.size();
  std::vector<signed char> side(NC,-2);std::vector<std::uint8_t> st(NC,0);std::vector<std::uint64_t> seen((NC+63)/64,0);
  const std::size_t W=core::parallelWidth();std::vector<std::vector<std::size_t>> plus(W);
  struct Lk{TissueLookup tissue;core::TileCache cache;};
  slabColumnsScan(SC,[&]{return Lk{TissueLookup{g,state.table},core::TileCache(g)};},[&](Lk&k,const Cell&c,std::size_t idx,std::size_t chunk){
   const Vec d=g.center(c)-C;const double s0=dot(d,n);if(std::abs(s0)>band)return;const double pa=dot(d,u),pb=dot(d,w);
   if(pa<gl[0]||pa>gh[0]||pb<gl[1]||pb>gh[1])return;if(!k.tissue.solid(c))return;
   const double s=s0-D.offset(pa,pb);if(std::abs(s)>band)return;const bool foot=D.inFoot(pa,pb);
   if(foot&&std::abs(s)<=h){st[idx]=1;return;}
   if(k.cache.removed(c)){st[idx]=2;return;}
   if(!foot&&std::abs(s)<=h+pitch&&wallAt(g.center(c))){st[idx]=1;return;}
   st[idx]=3;const signed char sd=(foot&&s>band-1.5*pitch)?1:(s<-(h+pitch)?-1:0);side[idx]=sd;if(sd>0)plus[chunk].push_back(idx);});
  std::vector<std::size_t> stack;const std::size_t KL=std::size_t(SC.KL),NJ=std::size_t(SC.nJ);
  for(const auto&F:plus)for(const auto j:F)if(!((seen[j>>6]>>(j&63))&1u)){seen[j>>6]|=std::uint64_t(1)<<(j&63);stack.push_back(j);}
  std::size_t at=~std::size_t(0);
  while(!stack.empty()&&!linked){const std::size_t j=stack.back();stack.pop_back();++floodVisited;if(side[j]==-1){linked=true;at=j;break;}
   const std::size_t col=j/KL;const int kk=int(j%KL),K=SC.k0[col]+kk,I=int(col/NJ),J=int(col%NJ);
   auto visit=[&](std::size_t q){if(((seen[q>>6]>>(q&63))&1u)||st[q]!=3)return;seen[q>>6]|=std::uint64_t(1)<<(q&63);stack.push_back(q);};
   auto other=[&](std::size_t c2){const int k2=K-SC.k0[c2];if(k2>=0&&k2<SC.len[c2])visit(c2*KL+std::size_t(k2));};
   if(kk>0)visit(j-1);if(kk+1<SC.len[col])visit(j+1);
   if(I>0)other(col-NJ);if(I+1<SC.nI)other(col+NJ);if(J>0)other(col-1);if(J+1<SC.nJ)other(col+1);}
  if(linked){refusal="seam_not_through";
   if(logOn()){std::size_t nPlus=0;for(const auto&F:plus)nPlus+=F.size();const Cell c=SC.cell(at);const Vec d=g.center(c)-C;const double pa=dot(d,u),pb=dot(d,w);
    std::cerr<<"seam_tear proof linked visited="<<floodVisited<<" plus_faces="<<nPlus<<" reached pa="<<pa<<" pb="<<pb<<" s="<<dot(d,n)<<" foot="<<D.inFoot(pa,pb)
     <<" at=("<<g.center(c).x<<","<<g.center(c).y<<","<<g.center(c).z<<") section_ms="<<sectionMs<<" proof_ms="<<core::ms(p0)<<std::endl;}
   return false;}}
 const double proofMs=core::ms(p0);
 // ---- the cells: full = the sheet over the footprint; open = the torn gap (margin < 0 on either side) over the stretch
 const auto e0=core::Clock::now();if(!full)finishSeeds(D,parallel);
 std::vector<Cell> cells;
 {const double reach=full?h+pitch:(ripR+maxD*ds+.004),pad=full?proofPad:.016;const double gl0=D.footLo[0]-pad,gh0=D.footHi[0]+pad,gl1=D.footLo[1]-pad,gh1=D.footHi[1]+pad;int lo[3],hi[3];
  // open split: the torn depths read from a 4 mm map over the stretch (each class's E per side at the plane: the gash is drawn in
  // 2 mm cells, finer than the map; 10 classes x 2 sides per map cell, built side by side) instead of per cell
  const double em=.004;const int EA=full?0:std::max(1,int(std::ceil((gh0-gl0)/em))),EB=full?0:std::max(1,int(std::ceil((gh1-gl1)/em)));std::vector<float> Emap(std::size_t(EA)*EB*20,0.f);
  if(!full)parallel(std::size_t(EB),[&](std::size_t j0,std::size_t j1){for(std::size_t jb=j0;jb<j1;++jb)for(int ia=0;ia<EA;++ia){const double a=gl0+(ia+.5)*em,b=gl1+(double(jb)+.5)*em;
   if(!inCarve(a,b,true))continue;float*o=&Emap[(jb*std::size_t(EA)+std::size_t(ia))*20];const Vec dd0=u*a+w*b;
   const unsigned grp=[&]{const int nc=int(D.FOOT[std::size_t(D.cellIndex(a,b))]>>1)-1;return nc>=0?unsigned(nc):MUSCLE;}();
   for(int side=0;side<2;++side){for(int k=0;k<9;++k)o[2*k+side]=float(flatE(D,k,side,a,b));o[18+side]=float(muscleE(D,grp,side,dd0.x,dd0.y,dd0.z,a,b));}}});
  auto Eat=[&](double a,double b,int k,int side){const int ia=std::clamp(int((a-gl0)/em),0,EA-1),jb=std::clamp(int((b-gl1)/em),0,EB-1);return double(Emap[(std::size_t(jb)*EA+std::size_t(ia))*20+std::size_t(2*k+side)]);};
  for(int k=0;k<3;++k){double a=1e30,b=-1e30;for(int q=0;q<8;++q){const double A=(q&1)?gh0:gl0,B=(q&2)?gh1:gl1,S=(q&4)?reach:-reach;const double p=C[k]+A*u[k]+B*w[k]+S*n[k];a=std::min(a,p);b=std::max(b,p);}
   lo[k]=std::max(0,int(std::floor((a-g.origin[k])/pitch))-1);hi[k]=std::min((&g.size.x)[k]-1,int(std::floor((b-g.origin[k])/pitch))+1);}
  std::vector<std::vector<Cell>> parts(core::parallelWidth());const Across ac{u,w,{gl0,gh0,gl1,gh1}};
  struct Lk{TissueLookup tissue;};
  slabScan(g,lo,hi,C,n,-reach,reach,[&]{return Lk{TissueLookup{g,state.table}};},[&](Lk&k,const Cell&c,std::size_t chunk){
   const Vec p=g.center(c);const Vec d=p-C;const double s0=dot(d,n),pa=dot(d,u),pb=dot(d,w);const double s=s0-D.offset(pa,pb);
   if(full){if(!k.tissue.solid(c))return;const bool foot=D.inFoot(pa,pb);if((std::abs(s)<=h&&foot)||(!foot&&std::abs(s)<=h+pitch&&wallAt(p)))parts[chunk].push_back(c);return;}
   if(std::abs(s)>reach)return;
   const unsigned L=closure->sample(p.x,p.y,p.z);const unsigned lv=L&63u;const bool air=lv==0||lv==16;
   if(!inCarve(pa,pb,air&&!k.tissue.solid(c)))return; // tissue only on the measured part; the clothing's air 1.5 cm around it
     const int side=s>=0?1:0;const double dd=std::max(0.,std::abs(s)-ripR);double E;
   const unsigned cls=air?SHELL:classOf(lv,(L&64u)!=0);
   if(cls==MUSCLE||cls>=MV0)E=Eat(pa,pb,9,side);
   else{const int mk=mapClass(cls);E=Eat(pa,pb,mk<0?0:mk,side);}
   if(dd-E<0)parts[chunk].push_back(c);},&ac);
  for(auto&p:parts)cells.insert(cells.end(),p.begin(),p.end());std::sort(cells.begin(),cells.end(),cellLess);}
 const double scanMs=core::ms(e0);
 std::vector<Cell> boundary;std::set<Cell> touched;std::size_t solid=0;
 for(const Cell&c:cells){if(!g.valid(c)||!g.erase(c))continue;boundary.push_back(c);touched.insert(Grid::tileKey(c));++v.mask;
  if(const auto label=g.materialBase(c)){++solid;const auto slot=g.material_slots&&label<g.material_slots->size()?unsigned((*g.material_slots)[label]&63u):1u;++v.bySlot[slot];}}
 std::size_t extraSolid=0;const auto r0=core::Clock::now();const double eraseOnlyMs=core::ms(e0)-scanMs;
 if(!boundary.empty()){const double reach=full?h:(ripR+maxD*ds+.004);
  try{v.edgeCells=resolveEdgeContacts(g,boundary,C,[&](Cell c){const Vec d=g.center(c)-C;const double pa=dot(d,u),pb=dot(d,w);
    return std::abs(dot(d,n)-D.offset(pa,pb))<=reach+pitch&&(full?D.inFoot(pa,pb):inCarve(pa,pb,true));},touched,&extraSolid);}
  catch(const RecoverableImpactError&){v.refusal="tear_repair_budget";}}
 // shreds: islands of standing tissue the cut left loose next to it (under 40 cm3: torn scraps in the gash, a crumb of bone between the
 // sheet and a track) go with the cut - else each would fly off as its own piece at the next separation
 const double repairMs=core::ms(r0);std::size_t shredCells=0;
 // (an open split only; a split's thin sheet leaves none the separation's debris rule does not take). The standing tissue of the carved
 // slab + 6 cells, over the footprint's rectangle + 3 cm, in slab columns: components touching the slab's faces or the rectangle's
 // edge reach on beyond it; the others under 40 cm3 are shreds.
 if(!full&&!boundary.empty()){const double R2=ripR+maxD*ds+.004+6*pitch,pad2=.046;const double gl[2]={D.footLo[0]-pad2,D.footLo[1]-pad2},gh[2]={D.footHi[0]+pad2,D.footHi[1]+pad2};int lo[3],hi[3];
  for(int k=0;k<3;++k){double a=1e30,b=-1e30;for(int q=0;q<8;++q){const double A=(q&1)?gh[0]:gl[0],B=(q&2)?gh[1]:gl[1],S=(q&4)?R2:-R2;const double pp=C[k]+A*u[k]+B*w[k]+S*n[k];a=std::min(a,pp);b=std::max(b,pp);}
   lo[k]=std::max(0,int(std::floor((a-g.origin[k])/pitch)));hi[k]=std::min((&g.size.x)[k]-1,int(std::floor((b-g.origin[k])/pitch)));}
  const Across ac2{u,w,{gl[0],gh[0],gl[1],gh[1]}};const SlabColumns SC=slabColumns(g,lo,hi,C,n,-R2,R2,&ac2);const std::size_t NC=SC.size();
  std::vector<std::uint8_t> st(NC,0); // 1 standing, 2 standing at the region's edge, 4 seen
  struct Lk{TissueLookup tissue;core::TileCache cache;};
  slabColumnsScan(SC,[&]{return Lk{TissueLookup{g,state.table},core::TileCache(g)};},[&](Lk&k,const Cell&c,std::size_t idx,std::size_t){
   if(!k.tissue.solid(c)||k.cache.removed(c))return;const Vec d=g.center(c)-C;const double s0=dot(d,n),pa=dot(d,u),pb=dot(d,w);
   const bool edgeCell=std::abs(s0)>R2-2*pitch||pa<gl[0]+2*pitch||pa>gh[0]-2*pitch||pb<gl[1]+2*pitch||pb>gh[1]-2*pitch;st[idx]=edgeCell?2:1;});
  const std::size_t cap=std::size_t(4e-5/(pitch*pitch*pitch)),KL=std::size_t(SC.KL),NJ=std::size_t(SC.nJ);std::vector<std::size_t> comp,stk;
  for(std::size_t i0=0;i0<NC;++i0){if(!(st[i0]&3)||(st[i0]&4))continue;comp.clear();stk.assign(1,i0);st[i0]|=4;bool isBig=false;
   while(!stk.empty()){const std::size_t j=stk.back();stk.pop_back();comp.push_back(j);if((st[j]&2)||comp.size()>=cap)isBig=true;
    const std::size_t col=j/KL;const int kk=int(j%KL),K=SC.k0[col]+kk,I=int(col/NJ),J=int(col%NJ);
    if(kk==0||kk+1>=SC.len[col]||I==0||J==0||I+1>=SC.nI||J+1>=SC.nJ)isBig=true; // the slab's own faces
    auto visit=[&](std::size_t q){if(!(st[q]&3)||(st[q]&4))return;st[q]|=4;stk.push_back(q);};
    auto other=[&](std::size_t c2){const int k2=K-SC.k0[c2];if(k2>=0&&k2<SC.len[c2])visit(c2*KL+std::size_t(k2));};
    if(kk>0)visit(j-1);if(kk+1<SC.len[col])visit(j+1);if(I>0)other(col-NJ);if(I+1<SC.nI)other(col+NJ);if(J>0)other(col-1);if(J+1<SC.nJ)other(col+1);}
   if(isBig)continue;
   for(const auto j:comp){const Cell x=SC.cell(j);if(g.erase(x)){touched.insert(Grid::tileKey(x));++v.mask;++solid;++shredCells;}}}}
 v.cells=solid+extraSolid;v.mask+=v.edgeCells;v.dirty=core::expandDirtyTiles(touched);const double eraseMs=core::ms(e0);
 std::size_t total=0;for(const auto&s2:segments)total+=s2.stations.size();
 if(!v.dirty.empty()&&state.measured.size()==total){std::size_t k2=0;
  for(const auto&s2:segments)for(const auto&station:s2.stations){const std::size_t kk=k2++;if(!state.measured[kk])continue;
   for(const Cell&t:v.dirty)if(t.x>=station.tileLo.x&&t.x<=station.tileHi.x&&t.y>=station.tileLo.y&&t.y<=station.tileHi.y&&t.z>=station.tileLo.z&&t.z<=station.tileHi.z){state.measured[kk]=0;break;}}}
 // ---- the block, its bake (beside the separation when the job keeps the closure, as decideOnce) and the record: a split only (an open
 // split's drawing is its erased gap; its decision is kept as frame + seed on the lineage, SeverState::seams)
 finishSeeds(D,parallel);
 v.point=C;v.axis=n;v.tissueRan=true;v.tissueMs=core::ms(t0);v.tear=!v.refusal;
 auto block=gpuBlock(D);
 static const double bakeMm=[]{const char*e=std::getenv("GFY_M3_BAKE_MM");const double x=e?std::atof(e):4.;return std::clamp(x,1.,8.);}();
 static const double mapMm=[]{const char*e=std::getenv("GFY_M3_MAP_MM");const double x=e?std::atof(e):4.;return std::clamp(x,1.,8.);}();
 Bake2Stats bst;bool bakeAsync=false;const char*dumpDir=std::getenv("GFY_M3_DUMP");static std::atomic<unsigned> dumpCount{1u<<20};const unsigned dumpIdx=dumpDir?dumpCount.fetch_add(1):0u;
 {const auto&keep=tear_bake::jobKeep();
  if(!tear_bake::syncOnly()&&keep&&keep.get()==static_cast<const void*>(closure)){bakeAsync=true;const auto*cl=closure;
   auto sampleV=[cl](double x,double y,double z){return cl->sample(x,y,z);};const std::string dd=dumpDir?dumpDir:"";const bool log=logOn();const std::uint32_t sd=seed;
   auto p=bakeTearChunks(std::make_shared<const Decision>(D),bakeMm*.001,mapMm*.001,sampleV,block,[block,dd,dumpIdx,log,sd](const Bake2Stats&s,double ms){
    if(!dd.empty())writeBakeDump(dd.c_str(),dumpIdx,*block);
    if(log)std::cerr<<"seam_bake_done seed="<<sd<<" ms="<<ms<<" bake="<<block->bake_dim[0]<<"x"<<block->bake_dim[1]<<"x"<<block->bake_dim[2]<<" evaluated="<<s.fieldEvaluated<<std::endl;});
   p->keep=keep;bst.mapBytes=block->map.size()*4;bst.fieldBytes=block->bake.size();tear_bake::start(block.get(),p);if(dumpDir)p->wait();}
  else bakeTear(D,bakeMm*.001,mapMm*.001,sample,parallel,*block,bst);}
 {ResolvedTunnel rec;const double reach=H+.025;const Vec e=C-n*reach;
  rec.entry={e.x,e.y,e.z};rec.direction={n.x,n.y,n.z};rec.edge_u={u.x,u.y,u.z};rec.edge_v={w.x,w.y,w.z};rec.radius=Rl;rec.penetration=2*reach;rec.cap=2*reach;rec.inlet=0;
  rec.exit_distance=2*reach;rec.exit_mult=1;rec.edge_seed=seed64;auto surf=std::make_shared<ResolvedTearSurface>();surf->decide=block;rec.surface=std::move(surf);v.records.push_back(std::move(rec));}
 if(dumpDir){dump(D,dumpDir,dumpIdx);if(!bakeAsync)writeBakeDump(dumpDir,dumpIdx,*block);}
 v.tissueMs=core::ms(t0);
 if(v.tear&&full){++state.decided;std::size_t k=0;const double doneR=.02;
  if(state.failed.size()==total)for(const auto&s:segments)for(const auto&station:s.stations){const Vec d=station.center-C;
   if(std::abs(dot(d,n))<=doneR&&D.inFoot(dot(d,u),dot(d,w)))state.failed[k]=1;++k;}}
 if(logOn())std::cerr<<"seam_tear "<<(full?"full":"open")<<" ok="<<v.tear<<" seed="<<seed<<" N="<<D.N<<" limb_cells="<<D.limbCells<<" Rl="<<Rl<<" section_ms="<<sectionMs
  <<" sampled="<<cache.sampled<<" proof_ms="<<proofMs<<" flood_visited="<<floodVisited<<" erase_ms="<<eraseMs<<" (scan "<<scanMs<<" erase "<<eraseOnlyMs<<" repair "<<repairMs<<")"<<" cells="<<cells.size()<<" erased="<<boundary.size()
  <<" edge_cells="<<v.edgeCells<<" bake_async="<<bakeAsync<<" bake_ms="<<bst.ms<<" map="<<block->map_dim[0]<<"x"<<block->map_dim[1]<<" bake="<<block->bake_dim[0]<<"x"<<block->bake_dim[1]<<"x"<<block->bake_dim[2]
  <<" ms="<<core::ms(t0)<<std::endl;
 return v.tear;}

// The seam check after a job (the caller's gate: value 3, the job removed tissue). cuts: every record of this grid's history and of
// this job (oldest first); fresh(sequence): a hit of this job. Changes the grid only when a seam qualifies.
template<class Fresh>
inline Outcome check(Grid&g,SeverState&state,const painter_detail::TissueField*closure,const std::vector<const ResolvedTunnel*>&cuts,
    const Fresh&fresh,const std::vector<SeverSegment>&segments){
 Outcome out;const auto t0=core::Clock::now();if(off()||!closure||!closure->available)return out;
 // ---- the channels
 std::vector<Chan> ch;bool anyFresh=false;
 for(const ResolvedTunnel*r:cuts){if(!r||r->role!=ResolvedTunnel::Role::primary||r->surface||!(r->penetration>0)||!(r->radius>.0004))continue;
  const Vec a=vec(r->entry),d0=vec(r->direction);const double L=length(d0);if(!(L>1e-9))continue;const Vec d=d0*(1./L);
  const double len=std::min(.5,r->penetration);const bool f=fresh(r->sequence);anyFresh=anyFresh||f; // the cut runs to its reach (an "exit" may be an inner air space: lungs, airway, mouth)
  if(r->slab&&r->slab_half_width>.004){const Vec side=unit(vec(r->edge_v));const double hw=r->slab_half_width;const int k=std::max(1,int(std::ceil(hw/.008)));
   for(int q=-k;q<=k;++q)ch.push_back({a+side*(hw*q/k),d,len,r->radius,reachOf(*r),f,true});}
  else ch.push_back({a,d,len,r->radius,reachOf(*r),f,false});}
 if(!anyFresh||ch.size()<chainMin)return out;
 // ---- the chain: a hit of this job + a partner -> the plane they sweep; its inliers refitted once; the run along the line
 struct Cand {Vec n,t,dm;double c0=0;std::vector<std::size_t> run;double span=0,resid=0;};Cand best;
 const std::size_t N=ch.size();std::vector<double> key(N);std::vector<std::size_t> mem;
 auto refit=[&](const Vec&dm0,const Vec&n0,double c00,Vec&dm,Vec&t,Vec&n,double&c0,std::vector<std::size_t>&in){
  in.clear();Vec sd{0,0,0};
  for(std::size_t k=0;k<N;++k){if(dot(ch[k].d,dm0)<parallelCos)continue;const double e1=dot(ch[k].a,n0)-c00;const double e2=dot(ch[k].a+ch[k].d*std::min(ch[k].len,deepAt),n0)-c00;
   if(std::abs(e1)<=planeTol&&std::abs(e2)<=deepTol){in.push_back(k);sd=sd+ch[k].d;}}
  if(in.size()<2)return false;dm=unit(sd);
  // entries across the fire direction: their principal axis is the line; the plane holds the line and the fire direction
  double m[2]={0,0};const Vec e1=unit(cross(dm,std::abs(dm.y)<.9?Vec{0,1,0}:Vec{1,0,0})),e2=cross(dm,e1);
  for(auto k:in){m[0]+=dot(ch[k].a,e1);m[1]+=dot(ch[k].a,e2);}m[0]/=double(in.size());m[1]/=double(in.size());
  double sxx=0,sxy=0,syy=0;for(auto k:in){const double x=dot(ch[k].a,e1)-m[0],y=dot(ch[k].a,e2)-m[1];sxx+=x*x;sxy+=x*y;syy+=y*y;}
  const double th=.5*std::atan2(2*sxy,sxx-syy);t=unit(e1*std::cos(th)+e2*std::sin(th));n=unit(cross(t,dm));
  c0=0;for(auto k:in)c0+=dot(ch[k].a,n);c0/=double(in.size());return true;};
 for(std::size_t f=0;f<N;++f){if(!ch[f].fresh)continue;
  for(std::size_t q=0;q<N;++q){if(q==f)continue;const Vec dv=ch[q].a-ch[f].a;const double dist=length(dv);if(dist<.005||dist>.25)continue;
   if(dot(ch[f].d,ch[q].d)<parallelCos)continue;const Vec dm0=unit(ch[f].d+ch[q].d);const Vec tt=dv-dm0*dot(dv,dm0);if(length(tt)<.01)continue;
   const Vec n0=unit(cross(unit(tt),dm0));Vec dm,t,n;double c0;
   if(!refit(dm0,n0,dot(ch[f].a,n0),dm,t,n,c0,mem))continue;
   std::vector<std::size_t> in;Vec dm2,t2,n2;double c2;if(!refit(dm,n,c0,dm2,t2,n2,c2,in))continue;
   if(std::find(in.begin(),in.end(),f)==in.end())continue;
   for(auto k:in)key[k]=dot(ch[k].a,t2);std::sort(in.begin(),in.end(),[&](std::size_t x,std::size_t y){return key[x]<key[y];});
   std::size_t at=std::size_t(std::find(in.begin(),in.end(),f)-in.begin()),a=at,b=at;
   while(a>0&&key[in[a]]-key[in[a-1]]<=gapMax)--a;while(b+1<in.size()&&key[in[b+1]]-key[in[b]]<=gapMax)++b;
   std::vector<std::size_t> run(in.begin()+std::ptrdiff_t(a),in.begin()+std::ptrdiff_t(b)+1);
   // a chain counts each hit once (a blade's sub-tracks are one cut): distinct entries 5 mm apart
   std::size_t hits=0;{double last=-1e30;for(auto k:run){if(!ch[k].slab||key[k]-last>.02)++hits;last=key[k];}}
   const double span=key[run.back()]-key[run.front()];double resid=0;for(auto k:run)resid=std::max(resid,std::abs(dot(ch[k].a,n2)-c2));
   if(hits<chainMin||span<spanMin)continue; // a line: 5 hits over 15 cm or more (a cluster at one station is the stations' work)
   if(run.size()>best.run.size()||(run.size()==best.run.size()&&span>best.span+1e-9)){best.n=n2;best.t=t2;best.dm=dm2;best.c0=c2;best.run=std::move(run);best.span=span;best.resid=resid;}}}
 if(best.run.empty()){if(logOn()&&std::getenv("GFY_SEAM_LOG"))std::cerr<<"seam_check none channels="<<N<<" ms="<<core::ms(t0)<<std::endl;return out;}
 out.chain=best.run.size();
 // ---- the frame: the chain's own fit (a line that drifts as it grows is followed: the plane is refitted to every hit each time); a
 // seam this lineage already holds on the same plane (within 12 degrees and 2.5 cm) keeps its seed and the stretch it opened, and its
 // frame follows the fit
 Vec n=best.n,t=best.t;const Vec dm=best.dm;
 Vec O{0,0,0};for(auto k:best.run)O=O+ch[k].a+ch[k].d*(.5*std::min(ch[k].len,.3));O=O*(1./double(best.run.size()));O=O-n*(dot(O,n)-best.c0);
 int match=-1;
 for(std::size_t s=0;s<state.seams.size();++s){const auto&r=state.seams[s];const Vec rn{r[S_N],r[S_N+1],r[S_N+2]},rc{r[S_C],r[S_C+1],r[S_C+2]};
  if(std::abs(dot(rn,n))>=.978&&std::abs(dot(O-rc,rn))<=.025){match=int(s);break;}}
 if(match>=0){const auto&r=state.seams[std::size_t(match)];
  if(r[S_FULL]!=0){if(logOn())std::cerr<<"seam_check chain="<<out.chain<<" on_a_split_seam ms="<<core::ms(t0)<<std::endl;return out;}
  auto&w=state.seams[std::size_t(match)];for(int k=0;k<3;++k){w[S_C+k]=O[k];w[S_N+k]=n[k];w[S_T+k]=t[k];}} // the seam follows its line
 const Vec b=unit(cross(n,t));
 // ---- the section in the plane (4 mm pixels): anatomy joined to the chain's tracks; removed / bone / gone per pixel
 const double TW=.95,BW=.60;const int NT=int(2*TW/pixel),NB=int(2*BW/pixel);std::vector<std::uint8_t> F(std::size_t(NT)*NB,0); // 1 seen, 2 section, 4 bone, 8 removed, 16 gone
 auto pt=[&](int i,int j){return O+t*(-TW+(i+.5)*pixel)+b*(-BW+(j+.5)*pixel);};
 auto cellOf=[&](const Vec&p){return Cell{int(std::floor((p.x-g.origin.x)/g.pitch)),int(std::floor((p.y-g.origin.y)/g.pitch)),int(std::floor((p.z-g.origin.z)/g.pitch))};};
 MatLookup mat{g,state.table};core::TileCache rem(g);
 // (a piece's grid: only its own cells - another piece's tissue, stored removed there, is no part of this section)
 auto look=[&](int i,int j)->std::uint8_t{std::uint8_t&f=F[std::size_t(j)*NT+std::size_t(i)];if(f&1)return f;f|=1;const Cell c=cellOf(pt(i,j));
  if(g.ownership&&g.valid(c)&&!g.ownership->body(c))return f;const int m=mat.at(c);
  if(m){f|=2;if(m==2)f|=4;if(rem.removed(c))f|=8|16;}return f;};
 std::vector<int> stack;
 std::size_t seeding=0;
 for(auto k:best.run){const auto&c=ch[k];bool seeded=false;for(double s=0;s<=c.len;s+=pixel){const Vec p=c.a+c.d*s;const Vec q=p-n*dot(p-O,n);const Vec rel=q-O;
   const int i=int(std::floor((dot(rel,t)+TW)/pixel)),j=int(std::floor((dot(rel,b)+BW)/pixel));if(i<0||j<0||i>=NT||j>=NB)continue;
   const auto f0=F[std::size_t(j)*NT+std::size_t(i)];if(f0&1){seeded=seeded||(f0&2);continue;}if(look(i,j)&2){stack.push_back(j*NT+i);seeded=true;}}
  seeding+=seeded;}
 if(seeding<chainMin){if(logOn())std::cerr<<"seam_check chain="<<best.run.size()<<" tracks_on_this_part="<<seeding<<" (fewer than "<<chainMin<<": the line ran through other pieces) ms="<<core::ms(t0)<<std::endl;out.ms=core::ms(t0);return out;}
 std::size_t sectionPx=0;bool edge=false;
 while(!stack.empty()){const int q=stack.back();stack.pop_back();++sectionPx;const int i=q%NT,j=q/NT;if(i==0||j==0||i==NT-1||j==NB-1)edge=true;
  const int nb[4][2]={{i-1,j},{i+1,j},{i,j-1},{i,j+1}};
  for(const auto&e:nb){if(e[0]<0||e[1]<0||e[0]>=NT||e[1]>=NB)continue;if(F[std::size_t(e[1])*NT+std::size_t(e[0])]&1)continue;if(look(e[0],e[1])&2)stack.push_back(e[1]*NT+e[0]);}}
 // a pixel the flood never reached is no section (look() marks every pixel it read; only the flood's own are kept)
 {std::vector<std::uint8_t> keep(F.size(),0);std::vector<int> st2;
  for(auto k:best.run){const auto&c=ch[k];for(double s=0;s<=c.len;s+=pixel){const Vec p=c.a+c.d*s;const Vec q=p-n*dot(p-O,n);const Vec rel=q-O;
    const int i=int(std::floor((dot(rel,t)+TW)/pixel)),j=int(std::floor((dot(rel,b)+BW)/pixel));if(i<0||j<0||i>=NT||j>=NB)continue;
    const std::size_t x=std::size_t(j)*NT+std::size_t(i);if((F[x]&2)&&!keep[x]){keep[x]=1;st2.push_back(int(x));}}}
  while(!st2.empty()){const int q=st2.back();st2.pop_back();const int i=q%NT,j=q/NT;const int nb[4][2]={{i-1,j},{i+1,j},{i,j-1},{i,j+1}};
   for(const auto&e:nb){if(e[0]<0||e[1]<0||e[0]>=NT||e[1]>=NB)continue;const std::size_t x=std::size_t(e[1])*NT+std::size_t(e[0]);if((F[x]&2)&&!keep[x]){keep[x]=1;st2.push_back(int(x));}}}
  for(std::size_t x=0;x<F.size();++x)if(!keep[x])F[x]&=std::uint8_t(~(2|4|8|16));}
 // near the line: within 6 cm (in the plane) of a chain track (entry to its end) - the part the seam cuts and measures. The rest of
 // the section (a limb past the line's end, an arm across the plane, tissue a short track never reached) still joins the two sides
 auto isR=[](std::uint8_t f){return (f&2)&&(f&32);};
 {const double nearR=.06;
  for(auto k:best.run){const auto&c=ch[k];const Vec A=c.a-n*dot(c.a-O,n),B0=c.a+c.d*c.len;const Vec B=B0-n*dot(B0-O,n);
   const double ta=dot(A-O,t),tb=dot(B-O,t),ba=dot(A-O,b),bb=dot(B-O,b),dt=tb-ta,db=bb-ba,L2=dt*dt+db*db;
   const int i0=std::max(0,int(std::floor((std::min(ta,tb)-nearR+TW)/pixel))),i1=std::min(NT-1,int(std::floor((std::max(ta,tb)+nearR+TW)/pixel)));
   const int j0=std::max(0,int(std::floor((std::min(ba,bb)-nearR+BW)/pixel))),j1=std::min(NB-1,int(std::floor((std::max(ba,bb)+nearR+BW)/pixel)));
   for(int j=j0;j<=j1;++j)for(int i=i0;i<=i1;++i){auto&f=F[std::size_t(j)*NT+std::size_t(i)];if(!(f&2)||(f&32))continue;
    const double x=-TW+(i+.5)*pixel-ta,y=-BW+(j+.5)*pixel-ba;const double s=L2>1e-12?std::clamp((x*dt+y*db)/L2,0.,1.):0.;
    const double ex=x-dt*s,ey=y-db*s;if(ex*ex+ey*ey<=nearR*nearR)f|=32;}}}
 // gone: within a track's reach (every channel near the plane, the chain's or not)
 for(const auto&c:ch){const Vec A=c.a,B=c.a+c.d*c.len;const double sA=dot(A-O,n),sB=dot(B-O,n);if((sA>c.rw&&sB>c.rw)||(sA<-c.rw&&sB<-c.rw))continue;
  const double ta=dot(A-O,t),tb=dot(B-O,t),ba=dot(A-O,b),bb=dot(B-O,b);
  const int i0=std::max(0,int(std::floor((std::min(ta,tb)-c.rw+TW)/pixel))),i1=std::min(NT-1,int(std::floor((std::max(ta,tb)+c.rw+TW)/pixel)));
  const int j0=std::max(0,int(std::floor((std::min(ba,bb)-c.rw+BW)/pixel))),j1=std::min(NB-1,int(std::floor((std::max(ba,bb)+c.rw+BW)/pixel)));
  for(int j=j0;j<=j1;++j)for(int i=i0;i<=i1;++i){auto&f=F[std::size_t(j)*NT+std::size_t(i)];if(!(f&2)||(f&16))continue;if(segDist(pt(i,j),c.a,c.d,c.len)<=c.rw)f|=16;}}
 // damagecut3 (painter_sever_method 1): the stretch this seam's open splits parted holds nothing - its pixels near the line are gone.
 // The old open split says so through its gash (their tissue is removed); in method 1 an open split removes nothing (below), so the
 // remembered stretch says it. The rule's numbers are unchanged; the split comes at the hit it came at before.
 if(match>=0&&!tear_decide::bridgeOff()){const auto&sr=state.seams[std::size_t(match)];const double tO0=dot(O,t),c0=sr[S_T0]-tO0,c1=sr[S_T1]-tO0;
  if(sr[S_FULL]==0&&c1>=c0){const int i0=std::max(0,int(std::ceil((c0+TW)/pixel-.5))),i1=std::min(NT-1,int(std::floor((c1+TW)/pixel-.5)));
   for(int j=0;j<NB;++j)for(int i=i0;i<=i1;++i){auto&f=F[std::size_t(j)*NT+std::size_t(i)];if((f&2)&&(f&32))f|=16;}}}
 // ---- the measure over the part near the line (R); through = nothing of the section lies outside it
 double t0c=1e30,t1c=-1e30,rwSum=0;for(auto k:best.run){const double tk=dot(ch[k].a-O,t);t0c=std::min(t0c,tk);t1c=std::max(t1c,tk);rwSum+=ch[k].rw;}
 const double gapMean=best.run.size()>1?(t1c-t0c)/double(best.run.size()-1):.05;const double gEnd=std::clamp(std::max(rwSum/double(best.run.size()),.5*gapMean),.015,.05);
 struct Sum {std::size_t tissue=0,gone=0,bone=0,boneGone=0;double rt()const{return tissue?1-double(gone)/double(tissue):1;}double rb()const{return bone?1-double(boneGone)/double(bone):rt();}};
 // R: the part near the line (an open split is measured and drawn there); W: the whole section, R and what lies past it (the tear
 // runs through it to where the part ends: what the line did not reach counts as standing)
 Sum sR,sW;std::size_t beyond=0;
 for(const auto f:F){if(!(f&2))continue;auto add=[&](Sum&x){++x.tissue;if(f&16)++x.gone;if(f&4){++x.bone;if(f&16)++x.boneGone;}};add(sW);if(!(f&32)){++beyond;continue;}add(sR);}
 if(std::getenv("GFY_SEAM_CHANLOG"))for(auto k:best.run){const auto&c=ch[k];const Vec B=c.a+c.d*c.len;
  std::cerr<<"seam_chan t="<<dot(c.a-O,t)<<" b0="<<dot(c.a-O,b)<<" b1="<<dot(B-O,b)<<" s0="<<dot(c.a-O,n)<<" s1="<<dot(B-O,n)<<" len="<<c.len<<" rw="<<c.rw<<" fresh="<<c.fresh<<std::endl;}
 if(const char*md=std::getenv("GFY_SEAM_MAPDUMP")){static std::atomic<unsigned> mapCount{0};char path[1024];std::snprintf(path,sizeof(path),"%s/seammap_%03u.ppm",md,mapCount.fetch_add(1));
  if(std::FILE*o=std::fopen(path,"wb")){std::fprintf(o,"P6\n%d %d\n255\n",NT,NB);
   for(int j=NB-1;j>=0;--j)for(int i=0;i<NT;++i){const auto f=F[std::size_t(j)*NT+std::size_t(i)];unsigned char c[3]={20,20,20};
    if(f&2){if(!(f&32)){c[0]=60;c[1]=110;c[2]=230;}else if(f&8){c[0]=40;c[1]=40;c[2]=40;}else if(f&16){c[0]=200;c[1]=60;c[2]=60;}else{c[0]=230;c[1]=200;c[2]=170;}
     if(f&4){c[0]=std::uint8_t(std::min(255,c[0]+25));c[1]=std::uint8_t(std::min(255,c[1]+40));c[2]=std::uint8_t(std::min(255,c[2]+40));}}
    std::fwrite(c,1,3,o);}
   std::fclose(o);}}
 // STRETCHES: the whole section cut along the line into stretches of at most 12 cm (a station's measure along the seam). Across a limb
 // the stations hold in series (one failed station frees it); along a seam the stretches hold side by side: the halves come apart only
 // when EVERY stretch failed (tissue <= .45 and bone <= .40 left, or flesh <= .15); an open split spans the failed stretches the line
 // covers. A stretch with under 30 pixels (4.8 cm2) of tissue holds nothing.
 double tmin=1e30,tmax=-1e30;for(int j=0;j<NB;++j)for(int i=0;i<NT;++i)if(F[std::size_t(j)*NT+std::size_t(i)]&2){const double tp=-TW+(i+.5)*pixel;tmin=std::min(tmin,tp);tmax=std::max(tmax,tp);}
 const int NS=std::max(1,int(std::ceil((tmax-tmin)/stretchLen-1e-9)));const double strW=(tmax-tmin+pixel)/NS;
 std::vector<Sum> strs;strs.resize(static_cast<std::size_t>(NS));
 for(int j=0;j<NB;++j)for(int i=0;i<NT;++i){const auto f=F[std::size_t(j)*NT+std::size_t(i)];if(!(f&2))continue;const double tp=-TW+(i+.5)*pixel;
  auto&x=strs[std::size_t(std::clamp(int((tp-tmin+.5*pixel)/strW),0,NS-1))];++x.tissue;if(f&16)++x.gone;if(f&4){++x.bone;if(f&16)++x.boneGone;}}
 auto failed=[&](const Sum&x){return x.tissue<30||(x.rt()<=tissueLeft&&x.rb()<=boneLeft)||x.rt()<=.15;};
 bool allFailed=true;for(const auto&x:strs)allFailed=allFailed&&failed(x);
 const bool through=!edge&&allFailed;
 // the open stretch: the run of failed stretches holding the chain's middle, clipped to the chain's span + gEnd
 double lo=0,hi=-1;{const double tc=.5*(t0c+t1c);int k=std::clamp(int((tc-tmin+.5*pixel)/strW),0,NS-1);
  if(failed(strs[std::size_t(k)])){int a=k,b2=k;while(a>0&&failed(strs[std::size_t(a-1)]))--a;while(b2+1<NS&&failed(strs[std::size_t(b2+1)]))++b2;
   lo=std::max(t0c-gEnd,tmin-.5*pixel+a*strW);hi=std::min(t1c+gEnd,tmin-.5*pixel+(b2+1)*strW);}}
 const bool weak=hi-lo>=spanMin&&sR.rt()<=tissueLeft&&sR.rb()<=boneLeft;
 const double tO=dot(O,t); // stretches are kept along the line in absolute terms (t . point)
 const double carved0=match>=0?state.seams[std::size_t(match)][S_T0]-tO:1e30,carved1=match>=0?state.seams[std::size_t(match)][S_T1]-tO:-1e30;
 const bool grows=lo<carved0-.01||hi>carved1+.01;
 if(logOn()){std::cerr<<"seam_check chain="<<best.run.size()<<" channels="<<N<<" span="<<best.span<<" resid="<<best.resid<<" n="<<n.x<<","<<n.y<<","<<n.z<<" t="<<t.x<<","<<t.y<<","<<t.z
  <<" section_px="<<sectionPx<<" part_px="<<sR.tissue<<" tissue_left="<<sR.rt()<<" bone_left="<<sR.rb()<<" bone_px="<<sR.bone<<" beyond_px="<<beyond
  <<" whole_tissue_left="<<sW.rt()<<" whole_bone_left="<<sW.rb()<<" edge="<<edge<<" through="<<through<<" open="<<lo<<".."<<hi<<" stretches="<<NS;
  for(const auto&x:strs)std::cerr<<" "<<int(100*x.rt()+.5)<<"/"<<(x.bone?int(100*x.rb()+.5):-1)<<"("<<x.tissue<<")"<<(failed(x)?"x":"");std::cerr
  <<" match="<<match<<" grows="<<grows<<" g="<<gEnd<<" measure_ms="<<core::ms(t0)<<std::endl;}
 const bool tryFull=through,tryOpen=weak&&grows;
 if(!tryFull&&!tryOpen){out.ms=core::ms(t0);return out;}
 // the decision's centre: on the part nearest the frame origin (its footprint grows from there); a lineage's seam keeps its own
 Vec C=O;{double bestD=1e30;for(int j=0;j<NB;++j)for(int i=0;i<NT;++i){const auto f=F[std::size_t(j)*NT+std::size_t(i)];if(!isR(f))continue;
   const Vec p=pt(i,j);const double d2=dot(p-O,p-O);if(d2<bestD){bestD=d2;C=p;}}}
 const double cT=dot(C-O,t),cB=dot(C-O,b);
 auto pix=[&](double a,double bb2)->std::uint8_t{const int i=int(std::floor((a+cT+TW)/pixel)),j=int(std::floor((bb2+cB+BW)/pixel));if(i<0||j<0||i>=NT||j>=NB)return 0;return F[std::size_t(j)*NT+std::size_t(i)];};
 auto tOf=[&](double a){return a+cT;};
 std::uint64_t seed64=0x5EA11E55ull;if(match>=0)seed64=std::uint64_t(state.seams[std::size_t(match)][S_SEED]);
 else{std::vector<std::uint64_t> seqs;for(auto k:best.run)seqs.push_back(std::uint64_t(std::llround(dot(ch[k].a,t)*1e4))^std::uint64_t(std::llround(dot(ch[k].a,b)*1e4))*0x9E3779B97F4A7C15ull);
  std::sort(seqs.begin(),seqs.end());for(auto s:seqs)seed64=core::quantMix(seed64^s);seed64&=0xFFFFFFFFFFFFull;}
 out.rt=tryFull?sW.rt():sR.rt();out.rb=tryFull?sW.rb():sR.rb();
 {double bestD=1e30;for(const auto&s:segments){const double tt=std::clamp(dot(C-s.pivot,s.axis),0.,s.length);const double dd=length(C-(s.pivot+s.axis*tt))-s.radius;if(dd<bestD){bestD=dd;out.bone=s.bone;}}}
 const char*refusal=nullptr;
 auto inPart=[&](double a,double bb2){for(int q=0;q<9;++q)if(isR(pix(a+pixel*(q%3-1),bb2+pixel*(q/3-1))))return true;return false;}; // the part grown one pixel
 auto inWhole=[&](double a,double bb2){for(int q=0;q<9;++q)if(pix(a+pixel*(q%3-1),bb2+pixel*(q/3-1))&2)return true;return false;}; // the whole section grown one pixel
 if(tryFull){double Rl=0;for(int j=0;j<NB;++j)for(int i=0;i<NT;++i){if(!(F[std::size_t(j)*NT+std::size_t(i)]&2))continue;Rl=std::max(Rl,length(pt(i,j)-C));}
  Rl+=2*g.pitch+pixel;
  DamageTearVerdict v;
  auto inWhole3=[&](double a,double bb2,bool){return inWhole(a,bb2);};
  if(decide(g,state,closure,C,n,t,Rl,inWhole,inWhole3,seed64,dm,true,segments,v,refusal)){out.full=true;out.v=std::move(v);
   std::array<double,16> rec{};for(int k=0;k<3;++k){rec[S_C+k]=C[k];rec[S_N+k]=n[k];rec[S_T+k]=t[k];}rec[S_T0]=t0c-endReach;rec[S_T1]=t1c+endReach;rec[S_SEED]=double(seed64);rec[S_FULL]=1;
   if(match>=0)state.seams[std::size_t(match)]=rec;else state.seams.push_back(rec);}
  else if(logOn())std::cerr<<"seam_check full_refused why="<<(refusal?refusal:"")<<std::endl;}
 if(!out.full&&tryOpen&&!tear_decide::bridgeOff()){out.open=true;out.v.point=C;out.v.axis=n;
  std::array<double,16> rec{};for(int k=0;k<3;++k){rec[S_C+k]=O[k];rec[S_N+k]=n[k];rec[S_T+k]=t[k];}
  rec[S_T0]=std::min(lo,carved0)+tO;rec[S_T1]=std::max(hi,carved1)+tO;rec[S_SEED]=double(seed64);rec[S_FULL]=0;
  if(match>=0)state.seams[std::size_t(match)]=rec;else state.seams.push_back(rec);
  if(logOn())std::cerr<<"seam_open parted stretch="<<lo<<".."<<hi<<" kept="<<rec[S_T0]-tO<<".."<<rec[S_T1]-tO<<" (method 1: nothing removed)"<<std::endl;}
 else if(!out.full&&tryOpen){
  // the decision covers the stretch's NEW part (what an earlier open split of this seam carved keeps its cells and its block)
  double nlo=lo,nhi=hi;if(carved1>=carved0){if(lo<carved0-.01&&hi<=carved1+.01)nhi=std::min(hi,carved0+.02);else if(hi>carved1+.01&&lo>=carved0-.01)nlo=std::max(lo,carved1-.02);}
  Vec Cn=C;{double bestD=1e30;const Vec target=O+t*(.5*(nlo+nhi));for(int j=0;j<NB;++j)for(int i=0;i<NT;++i){const auto f=F[std::size_t(j)*NT+std::size_t(i)];if(!isR(f))continue;
    const double tp=-TW+(i+.5)*pixel;if(tp<nlo||tp>nhi)continue;const Vec p=pt(i,j);const double d2=dot(p-target,p-target);if(d2<bestD){bestD=d2;Cn=p;}}}
  const double cTn=dot(Cn-O,t),cBn=dot(Cn-O,b);
  auto pixn=[&](double a,double bb2)->std::uint8_t{const int i=int(std::floor((a+cTn+TW)/pixel)),j=int(std::floor((bb2+cBn+BW)/pixel));if(i<0||j<0||i>=NT||j>=NB)return 0;return F[std::size_t(j)*NT+std::size_t(i)];};
  auto inPartn=[&](double a,double bb2){for(int q=0;q<9;++q)if(isR(pixn(a+pixel*(q%3-1),bb2+pixel*(q/3-1))))return true;return false;};
  double Rl=0;
  for(int j=0;j<NB;++j)for(int i=0;i<NT;++i){if(!isR(F[std::size_t(j)*NT+std::size_t(i)]))continue;const double tp=-TW+(i+.5)*pixel;if(tp<nlo||tp>nhi)continue;Rl=std::max(Rl,length(pt(i,j)-Cn));}
  Rl+=2*g.pitch+pixel+.04;
  auto inFoot=[&](double a,double bb2){const double tp=a+cTn;if(tp<nlo||tp>nhi)return false;return inPartn(a,bb2);};
  // the carve: the new part only, tissue on the part, the clothing's air 1.5 cm around it
  auto inCarve=[&](double a,double bb2,bool airOnly){const double tp=a+cTn;if(tp<nlo||tp>nhi)return false;if(tp>=carved0&&tp<=carved1)return false;
   if(!airOnly)return inPartn(a,bb2);for(int q=0;q<25;++q)if(isR(pixn(a+.0075*(q%5-2),bb2+.0075*(q/5-2))))return true;return false;};
  DamageTearVerdict v;refusal=nullptr;
  if(decide(g,state,closure,Cn,n,t,Rl,inFoot,inCarve,seed64,dm,false,segments,v,refusal)){out.open=true;out.v=std::move(v);
   std::array<double,16> rec{};for(int k=0;k<3;++k){rec[S_C+k]=O[k];rec[S_N+k]=n[k];rec[S_T+k]=t[k];}
   rec[S_T0]=std::min(lo,carved0)+tO;rec[S_T1]=std::max(hi,carved1)+tO;rec[S_SEED]=double(seed64);rec[S_FULL]=0;
   if(match>=0)state.seams[std::size_t(match)]=rec;else state.seams.push_back(rec);}
  else if(logOn())std::cerr<<"seam_check open_refused why="<<(refusal?refusal:"")<<std::endl;}
 out.ms=core::ms(t0);
 if(logOn())std::cerr<<"seam_result full="<<out.full<<" open="<<out.open<<" chain="<<out.chain<<" tissue_left="<<out.rt<<" bone_left="<<out.rb<<" cells="<<out.v.cells<<" mask="<<out.v.mask<<" ms="<<out.ms<<std::endl;
 return out;}
} // namespace seam_tear
