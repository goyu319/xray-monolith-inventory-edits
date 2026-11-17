#pragma once
#include "TearDecide.h"
#include "TearSpeed.h" // speed step: the same numbers in less time
#include "HoldingCut.h" // damagecut: bridge tears, holding-cells severs in any direction
int mask_only_mode_value() noexcept; // gfy_live (LiveDamageWorker.cpp)
namespace tear_decide {
inline bool logOn(){static const bool log=std::getenv("GFY_TEAR_SELECT_LOG")!=nullptr||std::getenv("GFY_M3_LOG")!=nullptr;return log;}
// Every decide-once fallback (any build): one counted, logged line
inline std::atomic<unsigned long long>&fallbacks(){static std::atomic<unsigned long long> n{0};return n;}
inline void fallback(const char*why,const char*extra=""){const auto n=++fallbacks();
 if(logOn())std::cerr<<"tear_decide_once fallback="<<why<<" count="<<n<<" "<<extra<<std::endl;}
// keepOut(p): true where this tear may not reach (head fractures: outside its part of its plane); headTear: 1 the cap's / base
// fracture, 2 a radial or varied fracture (lighter torn ends); proof=false: no flood proof (head fractures: the separation decides)
template<class KeepOut>
inline bool decideOnce(Grid&g,SeverState&state,const painter_detail::TissueField*closure,const Vec&C0,const Vec&axisN,double R0,double half,std::uint64_t seed64,
    const SeverSegment&seg,const SeverStation&st,const Vec*impact,const TearForce*force,double ripRadius,bool standingInPlane,const std::vector<SeverSegment>&segments,
    DamageTearVerdict&v,const core::Clock::time_point&t0,const KeepOut&keepOut,bool keepActive){
 if(mask_only_mode_value()!=3)return false;
 if(!closure||!closure->available){fallback("no_closure_field");return false;}
 static const bool off=std::getenv("GFY_NO_TISSUE_TEAR")!=nullptr;if(off){fallback("GFY_NO_TISSUE_TEAR");return false;}
 const int headTear=force?force->headTear:0;const bool proofOn=headTear==0;
 const bool planeOnly=force&&force->planeOnly; // raggedsever: a killing shot's tear through the damage - its own plane only, no ring seam
 const bool wholeSection=force&&force->wholeSection; // rpgblast: a trunk tear - no foreign parts, the full cylinder at once (DamageTear.h TearForce)
 // damagecut: every damage sever is a bridge tear (HoldingCut.h); a melee clean cut, a head fracture and a keep-limited tear as before
 const bool bridgeMode=!bridgeOff()&&!(force&&force->clean)&&headTear==0&&!keepActive;
 const auto d0=core::Clock::now();const double pitch=g.pitch;
 // the tear's cylinder: the anatomy tear's (R0) widened to the reach of today's rip channels (2 cells + their radius)
 static const double rPad=[]{const char*e=std::getenv("GFY_M3_RPAD_MM");return e?std::atof(e)*.001:0.;}(); // probe test knob
 const double Rbase=R0+2*pitch+std::max(0.,ripRadius)+rPad;
 const Vec limbAxis=unit(seg.axis);
 Vec C=C0,n=unit(axisN),u{},w{};
 const std::uint32_t seed=std::uint32_t(seed64^(seed64>>32));
 // slice 4 (grenade thigh near the hip, a body piece's second thigh): when the sheet does not separate the limb - the plane runs on
 // into the buttock or groin, whose tissue on the limb's side stays joined to the trunk past the cylinder's edge (today's torn
 // ends erased it) - the tear is decided again square to the limb (a blast's tilt is what reached into the trunk), then square
 // 3 cm and 6 cm further out along the limb, where its section is the limb's own; only then today's tear runs (logged).
 const double R=Rbase,proofPad=.03,Rp=R+proofPad,Rl=R;
 Decision D;double*P=D.P;double decideMs=0;int N=0,lo[3]{},hi[3]{},nx=0,ny=0,nz=0;bool proof=false;int grown=0;
 std::vector<Cell> sheetList;bool linked=false;std::size_t sheetCells=0;core::Clock::time_point s0t=d0;
 const double band=.014,slab=band+offMax;double h=1.25*pitch; // melee: h per plane for a clean cut (below)
 const double coreR=seg.radius+.008; // the limb's core disk (today's Rb)
 // Which body part a point of the plane belongs to: the sever segment whose axis is nearest (distance minus its radius, within 3
 // cm). FOREIGN = another limb that is not this one's neighbour along the skeleton (sharing a joint): the other thigh a thigh
 // touches, the trunk's other limbs. Foreign tissue is never in this tear's footprint; in the flood it is a goal (reaching it from
 // the limb's core = still joined to the body); where the limb's band tissue touches it a thin wall is removed with the sheet (the
 // seam between two touching thighs). Head fractures (their segment is their own) have no foreign parts.
 int ownSeg=-1;for(std::size_t i=0;i<segments.size();++i)if(&segments[i]==&seg)ownSeg=int(i);
 std::vector<char> neighbour(segments.size(),0);
 if(ownSeg>=0){const auto&o=segments[std::size_t(ownSeg)];const Vec oe=o.pivot+o.axis*o.length;
  for(std::size_t i=0;i<segments.size();++i){const auto&s=segments[i];const Vec se=s.pivot+s.axis*s.length;
   neighbour[i]=int(i)==ownSeg||length(se-o.pivot)<.04||length(oe-s.pivot)<.04||length(s.pivot-o.pivot)<.02||length(se-oe)<.02;}}
 // (the foreign side is favoured by 15 mm: the skeleton's element regions claim the touching skin of the other limb a little
 // past the axes' midline - a seam on the midline left the other thigh's element in this limb's piece)
 auto foreignAt=[&](const Vec&p)->bool{if(ownSeg<0)return false;double bestF=1e30,bestO=1e30;
  for(std::size_t i=0;i<segments.size();++i){const auto&s=segments[i];const double t=std::clamp(dot(p-s.pivot,s.axis),0.,s.length);const double d=length(p-(s.pivot+s.axis*t))-s.radius;
   if(neighbour[i])bestO=std::min(bestO,d);else bestF=std::min(bestF,d);}
  return bestF<.03&&bestF<bestO+.015;};
 // the plane's foreign map: 4 mm cells over the proof's reach (rebuilt per plane)
 const double fmR=Rbase+.03+2*pitch,fmc=.004;const int FM=int(std::ceil(2*fmR/fmc))+1;std::vector<std::uint8_t> foreignMap;
 auto foreignIn=[&](double a,double b){if(foreignMap.empty())return false;const int i=int(std::floor((a+fmR)/fmc)),j=int(std::floor((b+fmR)/fmc));
  return i>=0&&j>=0&&i<FM&&j<FM&&foreignMap[std::size_t(j)*FM+i]==1;};
 auto foreignNear=[&](double a,double b){if(foreignMap.empty())return false;const int i=int(std::floor((a+fmR)/fmc)),j=int(std::floor((b+fmR)/fmc));
  return i>=0&&j>=0&&i<FM&&j<FM&&foreignMap[std::size_t(j)*FM+i]==2;};
 const bool squareDiffers=std::abs(dot(unit(axisN),limbAxis))<.996; // more than ~5 degrees off square
 auto limbAt=[&](double pa,double pb){const double O=P[P_SEC],cl=P[P_SEC+1];const int qa=int(std::floor((pa-O)/cl)),qb=int(std::floor((pb-O)/cl));
  return qa>=0&&qb>=0&&qa<N&&qb<N&&D.LIMB[std::size_t(qb)*N+std::size_t(qa)]!=0;};
 auto inSheetAt=[&](const Vec&p,double*sOut){const Vec d=p-C;const double s0=dot(d,n),pa=dot(d,u),pb=dot(d,w);if(std::abs(s0)>slab||pa*pa+pb*pb>Rl*Rl)return false;
  const int q=D.cellIndex(pa,pb);if(!sOut&&!D.LIMB[std::size_t(q)])return false; // speed step: the footprint before the offset (same answer)
  const double s=bridgeMode?s0:s0-D.offset(pa,pb);if(sOut)*sOut=s;return D.LIMB[std::size_t(q)]!=0&&std::abs(s)<=h;}; // damagecut3: a bridge tear's surface is the plane (no relief to evaluate)
 // attempts: plane 0 = as chosen, 1 = square (when it differs), 2 / 3 = square 3 / 6 cm out along the limb (its distal side:
 // +axis); for each plane the footprint first reaches the limb's own section (core + 3 cm), then core + 7 cm, then the cylinder
 // (a plane through the groin must not cut the other thigh's stump or the trunk it runs on into, unless the limb needs it)
 double Rf=R;int planeAt=-1;
 SectionCache secCache;int foreignPlane=-1;double tClass=0,tFlood=0,tSection=0,tAlloc=0,tList=0;std::size_t floodVisited=0;const auto attemptsStart=d0;const auto visits0=scanVisits().load();
 static const int ladder[12]={0,1,2,3,4,5,6,7,8,9,10,11},blastLadder[12]={0,9,10,11,1,2,3,4,5,6,7,8};
 const bool blastTear=half>.085;const int*const order=blastTear?blastLadder:ladder;
 // the loop's last step (its control below, step for step)
 int lastStep=0;{double rfv=R;for(int st=0;;++st){const int at=st<12?order[st]:12;int plane=at/3;const int rf=at%3;if(plane==1&&!squareDiffers)continue;if(plane>3)break;if(plane>0&&rf==0)continue;if(planeOnly&&at!=0)continue;
  const double prev=rfv;rfv=wholeSection?R:(rf==0?std::min(R,coreR+.03):(rf==1?std::min(R,coreR+.07):R)); /* rpgblast: rfv */ if(rf>0&&rfv<=prev+1e-9)continue;lastStep=st;if(st>=11)break;}}
 // cutfix: the ring seam (limb tears): every attempt of the ladder left the limb joined to tissue off its footprint (the top of the
 // thigh through the buttock): the ladder runs again with a seam 8 mm wide around the footprint through the band
 bool ringOn=false,ringRetry=false;
 for(int step=0;;++step){int attempt=step<12?order[step]:12; // speed step: the ladder's order (see above)
 {int plane=attempt/3;const int rf=attempt%3;if(plane==1&&!squareDiffers)continue; // (the former jump past plane 1)
  if(plane>3)break;
  if(planeOnly&&attempt!=0)continue; // raggedsever: its own plane, its first try only
  if(plane>0&&rf==0)continue; // a retried plane starts at core + 7 cm (the limb's own section already failed on plane 0)
  if(plane!=planeAt){planeAt=plane;if(plane==0){C=C0;n=unit(axisN);}else{n=dot(axisN,limbAxis)<0?limbAxis*-1.:limbAxis;C=C0+limbAxis*(.03*(plane-1>0?plane-1:0));}}
  const double prevRf=Rf;Rf=wholeSection?R:(rf==0?std::min(R,coreR+.03):(rf==1?std::min(R,coreR+.07):R)); // rpgblast: Rf
  if(rf>0&&Rf<=prevRf+1e-9&&!ringRetry)continue; // the cylinder is no wider than the last try on this plane: nothing new to try
  ringRetry=false;
 }
 u=unit(cross(n,std::abs(n.x)<.8?Vec{1,0,0}:Vec{0,1,0}));w=cross(n,u);
 if((force&&force->clean)||bridgeMode)h=.51*std::max(std::abs(n.x),std::max(std::abs(n.y),std::abs(n.z)))*pitch; // melee: the thinnest parting sheet (damagecut: a bridge tear's too)
 if(ownSeg>=0&&!keepActive&&!wholeSection/* rpgblast: no foreign map */&&foreignPlane!=planeAt){foreignPlane=planeAt;foreignMap.assign(std::size_t(FM)*FM,0); // speed step: once per plane
  core::parallelChunks(std::size_t(FM),4,[&](std::size_t j0,std::size_t j1,std::size_t){for(std::size_t j=j0;j<j1;++j)for(int i=0;i<FM;++i){
   const double a=-fmR+(i+.5)*fmc,b=-fmR+(double(j)+.5)*fmc;if(foreignAt(C+u*a+w*b))foreignMap[j*FM+std::size_t(i)]=1;}});
  for(int j=0;j<FM;++j)for(int i=0;i<FM;++i){auto&m=foreignMap[std::size_t(j)*FM+i];if(m)continue;bool near=false;
   for(int dj=-1;dj<=1&&!near;++dj)for(int di=-1;di<=1;++di){const int x=i+di,y=j+dj;if(x>=0&&y>=0&&x<FM&&y<FM&&foreignMap[std::size_t(y)*FM+x]==1){near=true;break;}}
   if(near)m=2;}}
 D=Decision{};D.pitch=pitch;D.seed=seed;linked=false;sheetList.clear();sheetCells=0;
 // ---- the decision: the anatomy tear's frame and impact terms (TissueTear.h tear(), limb profile), then the section map
 const double maxD=tissue_tear::limbDepth,H=std::max(half,tissue_tear::limbDepth+.03);
 double ia=0,ib=0,along=0;{const double L=impact?length(*impact):0.;if(L>1e-9){const Vec d=*impact*(1./L);ia=dot(d,u);ib=dot(d,w);along=dot(d,n);}
  double l2=std::sqrt(ia*ia+ib*ib);if(l2<.2){const double th=6.283185307179586*tissue_tear::u01(seed,5,5,0x77u);ia=std::cos(th);ib=std::sin(th);l2=1;}ia/=l2;ib/=l2;}
 const double tilt=(2*tissue_tear::u01(seed,6,6,0x78u)-1)*.016,impRl=std::max(R0,.02);
 double mbase[2];for(int sd=0;sd<2;++sd){const std::uint32_t s=seed^(sd?0x51u:0xA3u);double m=0;for(unsigned q=0;q<8;++q)m+=tissue_tear::u01(s,int(100+q),7,0x3C1u);mbase[sd]=tissue_tear::limbK::groupFloor+tissue_tear::limbK::groupSpan*m/8;}
 for(int k=0;k<3;++k){P[P_C+k]=C[k];P[P_N+k]=n[k];P[P_U+k]=u[k];P[P_W+k]=w[k];}
 // depth scale: a limb's as today (a forced tear's own scale); head fractures tear back less (the cap's 0.6, a radial crack's 0.3)
 const double dscale=headTear==1?.6:(headTear>=2?.3:std::clamp(force?force->tissueDepthScale:1.,.1,1.));
 P[P_RL]=Rl;P[P_H]=H;P[P_MAXD]=maxD;P[P_RIP]=std::max(0.,ripRadius);P[P_DSCALE]=dscale;
 if(force&&force->clean)P[P_RIP]=0.; // melee: a clean cut has no rip slab (its faces end on the sheet)
 P[P_IMP]=ia;P[P_IMP+1]=ib;P[P_IMP+2]=along;P[P_IMP+3]=tilt;P[P_IMP+4]=impRl;P[P_IMP+5]=maxD;P[P_IMP+6]=mbase[0];P[P_IMP+7]=mbase[1];
 {Vec A0=limbAxis;if(!(std::isfinite(A0.x)&&std::isfinite(A0.y)&&std::isfinite(A0.z)))A0=n;
  Vec p1=cross(A0,Vec{0,0,1});const double l1=dot(p1,p1);p1=l1>.04?p1*(1./std::sqrt(l1)):unit(cross(A0,Vec{1,0,0}));const Vec p2=cross(A0,p1);
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
 // the section's allowed part: a head fracture's own part of its plane (keepOut), everything for a limb
 // (a head fracture's part of its plane is widened 6 mm: neighbouring faces' sheets overlap along their shared edges, no bridge)
 auto allow=[&](double a,double b){if(a*a+b*b>Rf*Rf||foreignIn(a,b))return false;if(!keepActive)return true;
  static const double e=.006;const double o[5][2]={{0,0},{e,0},{-e,0},{0,e},{0,-e}};
  for(const auto&q:o)if(!keepOut(C+u*(a+q[0])+w*(b+q[1])))return true;return false;};
 buildSectionFast(D,sample,parallel,allow,secCache);tSection+=D.sectionMs; // speed step (TearSpeed.h)
 // melee (value 3): a clean blade cut - the tear surface is the plane itself (no tilt, bow, wave or impact lean): the sheet, its
 // proof and the drawing all read this flat surface
 if(force&&force->clean){for(int q=0;q<7;++q)D.P[P_SEEDC+q]=0.;D.P[P_IMP+3]=0.;}
 if(bridgeMode)for(int q=0;q<7;++q)D.P[P_SEEDC+q]=0.; // damagecut: a bridge tear parts on the chosen plane itself (a relief would cut tissue the plane missed)
 decideMs=core::ms(d0);s0t=core::Clock::now();
 if(D.limbCells==0){fallback("empty_section");return false;}
 // ---- the sheet and its proof over the tear band (|s - offset| <= band)
 // the proof's band reaches 30 mm past the cylinder and holds ANY tissue there (not only the limb's footprint): the flood starts
 // on the limb's footprint on one side and must not reach it on the other, also around the footprint's edge
 N=D.N;
 const double gl[2]={std::max(D.footLo[0]-proofPad,-Rp),std::max(D.footLo[1]-proofPad,-Rp)},gh[2]={std::min(D.footHi[0]+proofPad,Rp),std::min(D.footHi[1]+proofPad,Rp)};
 for(int k=0;k<3;++k){double a=1e30,b=-1e30;for(int q=0;q<8;++q){const double A=(q&1)?gh[0]:gl[0],B=(q&2)?gh[1]:gl[1],S=(q&4)?slab:-slab;const double p=C[k]+A*u[k]+B*w[k]+S*n[k];a=std::min(a,p);b=std::max(b,p);}
  lo[k]=std::max(0,int(std::floor((a-g.origin[k])/pitch)));hi[k]=std::min((&g.size.x)[k]-1,int(std::floor((b-g.origin[k])/pitch)));if(lo[k]>hi[k]){fallback("outside_grid");return false;}}
 nx=hi[0]-lo[0]+1;ny=hi[1]-lo[1]+1;nz=hi[2]-lo[2]+1;const std::size_t NB=std::size_t(nx)*ny*nz;
 static const std::size_t boxBudget=[]{const char*e=std::getenv("GFY_TEAR_BOX_CELLS_M");return std::size_t(e?std::max(1,std::atoi(e)):48)<<20;}();
 // the sheet's box: over the budget the sheet is still made (cells in the footprint only, no proof): never a fallback
 proof=proofOn&&standingInPlane&&NB<=boxBudget;
 static const double innerMargin=[]{const char*e=std::getenv("GFY_M3_INNER_MM");return e?std::atof(e)*.001:.04;}(); // probe test knob
 if(proof){
  // speed step: the band in slab-aligned columns (TearSpeed.h SlabColumns: clipped along the normal's largest axis and to the
  // proof's rectangle across; the former arrays spanned the whole box and its loop clipped z only). One cell's class is the
  // former per-cell code (classify). The flood classifies the cells it reaches when it reaches them (a refused attempt stops at
  // its first -1 cell having looked at a small part of the band); the faces come from a scan of the core's own box, the sheet
  // from scans of the sheet's own boxes once the sheet separates. GFY_MS_EAGER_PROOF=1: every band cell classified first.
  const auto a0t=core::Clock::now();
  const Across across{u,w,{gl[0],gh[0],gl[1],gh[1]}}; // the proof's rectangle across (a cell outside it is no band cell)
  const SlabColumns SC=slabColumns(g,lo,hi,C,n,-slab,slab,&across);const std::size_t NC=SC.size();
  if(std::getenv("GFY_MS_COLSDBG")){std::size_t sum=0;for(const int L:SC.len)sum+=std::size_t(L);std::cerr<<"M5COLS ka="<<SC.ka<<" ia="<<SC.ia<<" ja="<<SC.ja<<" nI="<<SC.nI<<" nJ="<<SC.nJ<<" KL="<<SC.KL<<" sum="<<sum<<" NC="<<NC
   <<" lo="<<lo[0]<<","<<lo[1]<<","<<lo[2]<<" hi="<<hi[0]<<","<<hi[1]<<","<<hi[2]<<" n="<<n.x<<","<<n.y<<","<<n.z<<" slab="<<slab<<std::endl;}
  static const bool eagerAll=std::getenv("GFY_MS_EAGER_PROOF")!=nullptr,lazyAll=std::getenv("GFY_MS_LAZY_PROOF")!=nullptr; // probe A/B knobs
  // the first try (a blast's first two) classifies every band cell side by side (most severs need one try); a later retry lets
  // its flood classify what it reaches
  const bool eagerProof=eagerAll||(!lazyAll&&(step==0||(blastTear&&step==1)));
  auto&scr=proofScratch();scr.side.assign(eagerProof?NC:0,-2);scr.sheet.assign(NC,0);scr.seen.assign(eagerProof?(NC+63)/64:0,0);
  signed char*const side=scr.side.data();std::uint8_t*const sheet=scr.sheet.data();std::uint64_t*const seen=scr.seen.data();
  tAlloc+=core::ms(a0t);const auto c0t=core::Clock::now();
  const double faceR=std::min(R-innerMargin,coreR);
  // (also any tissue 8 mm or more off the footprint: the sheet does not cut it - a trunk wedge the plane cut on the limb's side
  // stays joined to the trunk around the footprint's edge, the grenade's high thigh tear through the buttock)
  // (speed step: the centre point - q = 4, a + e * 0 is a itself - first; the same OR of pure lookups)
  auto offFoot=[&](double a,double b){if(limbAt(a,b))return false;const double e=.008;for(int q=0;q<9;++q)if(q!=4&&limbAt(a+e*(q%3-1),b+e*(q/3-1)))return false;return true;};
  // cutfix (the ring seam): within 8 mm of another limb's tissue, not in it
  auto foreignClose=[&](double a,double b){if(foreignMap.empty()||foreignIn(a,b))return false;const int i=int(std::floor((a+fmR)/fmc)),j=int(std::floor((b+fmR)/fmc));
   for(int dj=-2;dj<=2;++dj)for(int di=-2;di<=2;++di){const int x=i+di,y=j+dj;if(x>=0&&y>=0&&x<FM&&y<FM&&foreignMap[std::size_t(y)*FM+x]==1)return true;}return false;};
  // a cell's class: 0 not band tissue, 1 the sheet (+ the seam where it touches another limb), 2 removed, 3 standing; sd its face
  // side - the flood starts on the limb's own core (within its radius + 8 mm, today's core disk) on the limb's side (+1); it must
  // reach neither the core on the other side nor any foreign tissue in the band (-1: another limb, joined to the trunk)
  struct Lk{TissueLookup tissue;core::TileCache cache;};
  auto classify=[&](Lk&k,const Cell&c,signed char&sd)->int{
   const Vec d=g.center(c)-C;const double s0=dot(d,n);if(std::abs(s0)>slab)return 0;const double pa=dot(d,u),pb=dot(d,w);
   if(pa<gl[0]||pa>gh[0]||pb<gl[1]||pb>gh[1]||pa*pa+pb*pb>Rp*Rp)return 0;
   if(!k.tissue.solid(c))return 0; // (before the offset: the same AND of pure tests, the cheaper one first)
   const double s=bridgeMode?s0:s0-D.offset(pa,pb);if(std::abs(s)>band)return 0; // damagecut3: a bridge tear's surface is the plane (its relief is zero: not evaluated per cell)
   const double r2=pa*pa+pb*pb;const bool limb=limbAt(pa,pb)&&r2<=Rl*Rl,inner=limb&&r2<=faceR*faceR;
   sd=inner?(s>band-1.5*pitch?1:(s<-band+1.5*pitch?-1:0)):((foreignIn(pa,pb)||offFoot(pa,pb))?-1:0);
   if((limb&&std::abs(s)<=h)||(foreignNear(pa,pb)&&r2<=Rl*Rl)||(ringOn&&((limbAt(pa,pb)?(r2>Rl*Rl||foreignIn(pa,pb)):!offFoot(pa,pb))||foreignClose(pa,pb))))return 1; // cutfix: + the ring seam (the footprint edge, past the disk, along another limb)
   return k.cache.removed(c)?2:3;};
  auto jOf=[&](const Cell&c){return (std::size_t(c.x-lo[0])*ny+std::size_t(c.y-lo[1]))*nz+std::size_t(c.z-lo[2]);};
  const std::size_t W=core::parallelWidth();std::vector<std::vector<std::size_t>> plusF(W),sheetJ(W);
  auto makeLk=[&]{return Lk{TissueLookup{g,state.table},core::TileCache(g)};};
  bool sheetListed=false;
  // the sheet's cells (class 1) by scans of their own boxes: the limb's sheet (|s0| <= h + offMax, the footprint) and the seam (the
  // foreign-near cells' extent, the band); a cell found by both counted once
  auto listSheet=[&]{sheetListed=true;for(auto&v:sheetJ)v.clear();
   auto scanBox=[&](double a0,double a1,double b0,double b1,double sMin,double sMax){if(a0>a1||b0>b1)return;int blo[3],bhi[3];
    for(int k=0;k<3;++k){double a=1e30,b=-1e30;for(int q=0;q<8;++q){const double A=(q&1)?a1:a0,B=(q&2)?b1:b0,S=(q&4)?sMax:sMin;const double p=C[k]+A*u[k]+B*w[k]+S*n[k];a=std::min(a,p);b=std::max(b,p);}
     blo[k]=std::max(lo[k],int(std::floor((a-g.origin[k])/pitch))-1);bhi[k]=std::min(hi[k],int(std::floor((b-g.origin[k])/pitch))+1);}
    const Across ac{u,w,{a0,a1,b0,b1}};
    slabScan(g,blo,bhi,C,n,sMin,sMax,makeLk,[&](Lk&k,const Cell&c,std::size_t chunk){signed char sd=0;if(classify(k,c,sd)==1)sheetJ[chunk].push_back(jOf(c));},&ac);};
   scanBox(std::max(gl[0],D.footLo[0]),std::min(gh[0],D.footHi[0]),std::max(gl[1],D.footLo[1]),std::min(gh[1],D.footHi[1]),-(h+offMax),h+offMax);
   if(!foreignMap.empty()){int i0=FM,i1=-1,j0=FM,j1=-1;for(int j=0;j<FM;++j)for(int i=0;i<FM;++i)if(foreignMap[std::size_t(j)*FM+std::size_t(i)]==2){i0=std::min(i0,i);i1=std::max(i1,i);j0=std::min(j0,j);j1=std::max(j1,j);}
    if(i1>=0)scanBox(std::max(gl[0],-fmR+i0*fmc),std::min(gh[0],-fmR+(i1+1)*fmc),std::max(gl[1],-fmR+j0*fmc),std::min(gh[1],-fmR+(j1+1)*fmc),-slab,slab);}
   if(ringOn)scanBox(std::max(gl[0],D.footLo[0]-.009),std::min(gh[0],D.footHi[0]+.009),std::max(gl[1],D.footLo[1]-.009),std::min(gh[1],D.footHi[1]+.009),-slab,slab); // cutfix
   std::vector<std::size_t> all;for(const auto&v:sheetJ)all.insert(all.end(),v.begin(),v.end());std::sort(all.begin(),all.end());all.erase(std::unique(all.begin(),all.end()),all.end());
   sheetJ.assign(W,{});sheetJ[0]=std::move(all);sheetCells=sheetJ[0].size();};
  if(eagerProof){
   slabColumnsScan(SC,makeLk,[&](Lk&k,const Cell&c,std::size_t idx,std::size_t chunk){signed char sd=0;const int cl=classify(k,c,sd);if(!cl)return;
    side[idx]=sd;if(sd>0)plusF[chunk].push_back(idx);if(cl==1){sheet[idx]=1;sheetJ[chunk].push_back(jOf(c));}else if(cl==3)sheet[idx]=2;});
   for(const auto&v:sheetJ)sheetCells+=v.size();sheetListed=true;}
  else{ // the faces: the core's box (|a|, |b| <= faceR, s0 from band - 1.5 cells - offMax)
   int flo[3],fhi[3];const double a0=std::max(gl[0],-faceR),a1=std::min(gh[0],faceR),b0=std::max(gl[1],-faceR),b1=std::min(gh[1],faceR),sMin=band-1.5*pitch-offMax;
   for(int k=0;k<3;++k){double a=1e30,b=-1e30;for(int q=0;q<8;++q){const double A=(q&1)?a1:a0,B=(q&2)?b1:b0,S=(q&4)?slab:sMin;const double p=C[k]+A*u[k]+B*w[k]+S*n[k];a=std::min(a,p);b=std::max(b,p);}
    flo[k]=std::max(lo[k],int(std::floor((a-g.origin[k])/pitch))-1);fhi[k]=std::min(hi[k],int(std::floor((b-g.origin[k])/pitch))+1);}
   const Across fac{u,w,{a0,a1,b0,b1}};
   if(a0<=a1&&b0<=b1)slabScan(g,flo,fhi,C,n,sMin,slab,makeLk,[&](Lk&k,const Cell&c,std::size_t chunk){signed char sd=0;
    if(classify(k,c,sd)==3&&sd>0){const std::size_t idx=SC.index(c);if(idx!=SlabColumns::npos){sheet[idx]=3;plusF[chunk].push_back(idx);}}},&fac);}
  tClass+=core::ms(c0t);const auto f0t=core::Clock::now();
  // flood the standing tissue (not removed, not in the sheet) from the +1 faces; the sheet separates when no -1 face is reached
  {std::vector<std::size_t> stack;Lk lk=makeLk();
   const std::size_t KL=std::size_t(SC.KL),NJ=std::size_t(SC.nJ);
   if(eagerProof){
    auto standing=[&](std::size_t j){return sheet[j]==2;};
    for(const auto&F:plusF)for(const auto j:F)if(standing(j)&&!((seen[j>>6]>>(j&63))&1u)){seen[j>>6]|=std::uint64_t(1)<<(j&63);stack.push_back(j);}
    while(!stack.empty()&&!linked){const std::size_t j=stack.back();stack.pop_back();++floodVisited;if(side[j]==-1){linked=true;break;}
     const std::size_t col=j/KL;const int kk=int(j%KL),K=SC.k0[col]+kk,I=int(col/NJ),J=int(col%NJ);
     auto visit=[&](std::size_t k){if(((seen[k>>6]>>(k&63))&1u)||!standing(k))return;seen[k>>6]|=std::uint64_t(1)<<(k&63);stack.push_back(k);};
     auto other=[&](std::size_t c2){const int k2=K-SC.k0[c2];if(k2>=0&&k2<SC.len[c2])visit(c2*KL+std::size_t(k2));};
     if(kk>0)visit(j-1);if(kk+1<SC.len[col])visit(j+1);
     if(I>0)other(col-NJ);if(I+1<SC.nI)other(col+NJ);if(J>0)other(col-1);if(J+1<SC.nJ)other(col+1);}}
   else{ // sheet[] = the cell's state: 0 not looked at, 1 looked at (not standing), 3 standing (on the stack once)
    for(const auto&F:plusF)stack.insert(stack.end(),F.begin(),F.end());
    while(!stack.empty()&&!linked){const std::size_t j=stack.back();stack.pop_back();++floodVisited;
     const std::size_t col=j/KL;const int kk=int(j%KL),K=SC.k0[col]+kk,I=int(col/NJ),J=int(col%NJ);
     auto visit=[&](std::size_t k){if(sheet[k]||linked)return;signed char sd=0;const int cl=classify(lk,SC.cell(k),sd);
      if(cl!=3){sheet[k]=1;return;}sheet[k]=3;if(sd==-1){linked=true;return;}stack.push_back(k);};
     auto other=[&](std::size_t c2){const int k2=K-SC.k0[c2];if(k2>=0&&k2<SC.len[c2])visit(c2*KL+std::size_t(k2));};
     if(kk>0)visit(j-1);if(kk+1<SC.len[col])visit(j+1);
     if(I>0)other(col-NJ);if(I+1<SC.nI)other(col+NJ);if(J>0)other(col-1);if(J+1<SC.nJ)other(col+1);}}}
  tFlood+=core::ms(f0t);
  if(linked&&std::getenv("GFY_M3_LINKDBG"))std::cerr<<"M3LINK linked (attempt "<<attempt<<", visited "<<floodVisited<<")"<<std::endl;
  static const bool ignoreLink=std::getenv("GFY_M4_IGNORE_LINK")!=nullptr; // probe test knob: keep a sheet the proof refused
  if(linked&&ignoreLink&&attempt>=3){linked=false;std::cerr<<"tear_decide_once ignore_link=1"<<std::endl;}
  if(!linked){const auto l0t=core::Clock::now();if(!sheetListed)listSheet();
   std::vector<std::size_t> all;all.reserve(sheetCells);for(const auto&v:sheetJ)all.insert(all.end(),v.begin(),v.end());std::sort(all.begin(),all.end());
   for(const auto j:all){const int z=int(j%std::size_t(nz)),y=int((j/std::size_t(nz))%std::size_t(ny)),x=int(j/(std::size_t(nz)*ny));sheetList.push_back(Cell{lo[0]+x,lo[1]+y,lo[2]+z});}
   tList+=core::ms(l0t);}
  else if(!sheetListed&&step>=lastStep)listSheet(); // the last attempt's refusal line reports its sheet's size (the former count)
  scr.trim();
 }else if(standingInPlane){
  // no proof (head fractures, or a box over the flood budget): the sheet over the footprint, the separation decides
  // (speed step: slabScan over the sheet's own box - a sheet cell lies on the footprint (LIMB) and within h + offMax of the plane
  // (|offset| <= offMax) - inside the former box; the cells sorted to the former x, y, z order)
  const auto c0t=core::Clock::now();std::vector<std::vector<Cell>> parts(core::parallelWidth());
  int slo[3],shi[3];const double sh=h+offMax;
  for(int k=0;k<3;++k){double a=1e30,b=-1e30;for(int q=0;q<8;++q){const double A=(q&1)?D.footHi[0]:D.footLo[0],B=(q&2)?D.footHi[1]:D.footLo[1],S=(q&4)?sh:-sh;
    const double p=C[k]+A*u[k]+B*w[k]+S*n[k];a=std::min(a,p);b=std::max(b,p);}
   slo[k]=std::max(lo[k],int(std::floor((a-g.origin[k])/pitch))-1);shi[k]=std::min(hi[k],int(std::floor((b-g.origin[k])/pitch))+1);}
  struct Lk{TissueLookup tissue;};const Across footAcross{u,w,{D.footLo[0],D.footHi[0],D.footLo[1],D.footHi[1]}};
  slabScan(g,slo,shi,C,n,-sh,sh,[&]{return Lk{TissueLookup{g,state.table}};},[&](Lk&k,const Cell&c,std::size_t chunk){
   if(!inSheetAt(g.center(c),nullptr)||!k.tissue.solid(c))return;parts[chunk].push_back(c);},&footAcross);
  for(auto&p:parts){sheetCells+=p.size();sheetList.insert(sheetList.end(),p.begin(),p.end());}
  std::sort(sheetList.begin(),sheetList.end(),cellLess);tClass+=core::ms(c0t);
 }
 if(linked&&!ringOn&&proof&&headTear==0&&step>=lastStep&&!planeOnly){ringOn=true;step=-1;planeAt=-1;if(logOn())std::cerr<<"tear_decide_once ring_seam ladder_again"<<std::endl;continue;} // cutfix: the ladder again, with the seam
 if(linked&&step<11){++grown;if(logOn())std::cerr<<"tear_decide_once retry attempt="<<attempt<<" step="<<step<<" plane="<<planeAt<<" Rf="<<Rf<<" R="<<R
  <<" section_ms="<<D.sectionMs<<" class_ms="<<tClass<<" flood_ms="<<tFlood<<" flood_visited="<<floodVisited<<" sampled="<<secCache.sampled<<" at_ms="<<core::ms(attemptsStart)<<std::endl;continue;}
 break;}
 // speed step: the per-seed constants the sheet did not need (finishSeeds) run beside the seam, the erase and its repair - they
 // write the decision's seed constants and shell floor only, those read its offset and footprint; joined before the GPU block
 // (and on every way out of this function)
 std::shared_ptr<tear_bake::Pending> seedsJob;struct SeedsJoin{std::shared_ptr<tear_bake::Pending>&p;~SeedsJoin(){if(p)p->wait();}} seedsJoin{seedsJob};
 if(!linked){seedsJob=std::make_shared<tear_bake::Pending>();
  seedsJob->chunks.push_back([&D]{auto par=[](std::size_t count,const auto&body){core::parallelChunks(count,1,[&](std::size_t a,std::size_t b,std::size_t){body(a,b);});};finishSeeds(D,par);});
  tear_bake::service().submit(seedsJob);}
 // the seam where the limb touches a foreign limb (the inner thighs under the crotch) runs on below the band on the limb's side: its
 // wall goes on to 8 cm (the limb's own tissue next to the other limb; hidden between the two)
 std::size_t seamCells=0;
 if(!linked&&proof&&!foreignMap.empty()&&std::find(foreignMap.begin(),foreignMap.end(),std::uint8_t(2))!=foreignMap.end()){
  const double sLo=band,sHi=.08;int blo[3],bhi[3];
  for(int k=0;k<3;++k){double a=1e30,b=-1e30;for(int q=0;q<8;++q){const double A=(q&1)?Rl:-Rl,B=(q&2)?Rl:-Rl,S=(q&4)?sHi+offMax:sLo-offMax;const double p=C[k]+A*u[k]+B*w[k]+S*n[k];a=std::min(a,p);b=std::max(b,p);}
   blo[k]=std::max(0,int(std::floor((a-g.origin[k])/pitch)));bhi[k]=std::min((&g.size.x)[k]-1,int(std::floor((b-g.origin[k])/pitch)));}
  std::vector<std::vector<Cell>> parts(core::parallelWidth());
  // speed step: a seam cell lies on a foreign-near cell of the map (foreignNear) - the box shrinks to those cells' extent across
  Across seamAcross{u,w,{-Rl,Rl,-Rl,Rl}};
  {int i0=FM,i1=-1,j0=FM,j1=-1;for(int j=0;j<FM;++j)for(int i=0;i<FM;++i)if(foreignMap[std::size_t(j)*FM+std::size_t(i)]==2){i0=std::min(i0,i);i1=std::max(i1,i);j0=std::min(j0,j);j1=std::max(j1,j);}
   const double A0=std::max(-Rl,-fmR+i0*fmc),A1=std::min(Rl,-fmR+(i1+1)*fmc),B0=std::max(-Rl,-fmR+j0*fmc),B1=std::min(Rl,-fmR+(j1+1)*fmc);seamAcross.r[0]=A0;seamAcross.r[1]=A1;seamAcross.r[2]=B0;seamAcross.r[3]=B1;
   for(int k=0;k<3;++k){double a=1e30,b=-1e30;for(int q=0;q<8;++q){const double A=(q&1)?A1:A0,B=(q&2)?B1:B0,S=(q&4)?sHi+offMax:sLo-offMax;const double p=C[k]+A*u[k]+B*w[k]+S*n[k];a=std::min(a,p);b=std::max(b,p);}
    blo[k]=std::max(blo[k],int(std::floor((a-g.origin[k])/pitch))-1);bhi[k]=std::min(bhi[k],int(std::floor((b-g.origin[k])/pitch))+1);}}
  if(blo[0]<=bhi[0]&&blo[1]<=bhi[1]&&blo[2]<=bhi[2]){struct Lk{TissueLookup tissue;}; // speed step: slabScan over s0 in [sLo - offMax, sHi + offMax]
   slabScan(g,blo,bhi,C,n,sLo-offMax,sHi+offMax,[&]{return Lk{TissueLookup{g,state.table}};},[&](Lk&k,const Cell&c,std::size_t chunk){const Vec d=g.center(c)-C;
    const double pa=dot(d,u),pb=dot(d,w);if(pa*pa+pb*pb>Rl*Rl||!foreignNear(pa,pb))return;const double s=dot(d,n)-D.offset(pa,pb);if(s<=sLo||s>sHi)return;
    if(k.tissue.solid(c))parts[chunk].push_back(c);},&seamAcross);}
  {std::vector<Cell> seam;for(auto&p:parts)seam.insert(seam.end(),p.begin(),p.end());std::sort(seam.begin(),seam.end(),cellLess);
   seamCells+=seam.size();sheetList.insert(sheetList.end(),seam.begin(),seam.end());}}
 const double proofMs=core::ms(s0t);
 if(linked&&planeOnly){v.refusal="kill_plane_did_not_separate";v.ms=core::ms(t0);if(logOn())std::cerr<<"tear_decide_once kill_plane_did_not_separate grown="<<grown<<std::endl;return true;} // raggedsever: nothing changed
 if(linked){char e[160];std::snprintf(e,sizeof(e),"decide_ms=%.2f proof_ms=%.2f sheet_cells=%zu",decideMs,proofMs,sheetCells);fallback("sheet_did_not_separate",e);return false;}
 // damagecut: the bridge = the thin sheet's STANDING cells (its seam / ring walls are no bridge: hidden between the limbs); its maps
 // before they are erased (the cells it removes are exactly these, + the walls, + the checkerboard repair)
 if(bridgeMode){std::vector<Cell> bridgeCells;{TissueLookup tl{g,state.table};core::TileCache rc(g);
   for(const Cell&c:sheetList){const Vec d=g.center(c)-C;const double pa=dot(d,u),pb=dot(d,w);
    if(std::abs(dot(d,n)-D.offset(pa,pb))>h||!limbAt(pa,pb))continue;if(tl.solid(c)&&!rc.removed(c))bridgeCells.push_back(c);}}
  std::size_t woundPx=0;D.bridgeH=bridgeMaps(D,g,state,bridgeCells,bridgeRip(pitch),false,&woundPx);v.bridgeCells=bridgeCells.size();v.holding=bridgeCells.size();
  if(logOn())std::cerr<<"damagecut_bridge station cells="<<bridgeCells.size()<<" sheet_cells="<<sheetList.size()<<" wound_px="<<woundPx<<" emax_mm="<<(D.bridgeH-.03)*1e3<<std::endl;}
 // ---- the sheet goes into the grid (journaled erase, the same repair every cut gets)
 const auto e0=core::Clock::now();std::vector<Cell> boundary;std::set<Cell> touched;std::size_t solid=0;
 for(const Cell&c:sheetList){if(!g.valid(c)||!g.erase(c))continue;boundary.push_back(c);touched.insert(Grid::tileKey(c));++v.mask;
  if(const auto label=g.materialBase(c)){++solid;const auto slot=g.material_slots&&label<g.material_slots->size()?unsigned((*g.material_slots)[label]&63u):1u;++v.bySlot[slot];}}
 std::size_t extraSolid=0;
 if(!boundary.empty()){try{v.edgeCells=resolveEdgeContacts(g,boundary,C,[&](Cell c){return inSheetAt(g.center(c),nullptr);},touched,&extraSolid);}
  catch(const RecoverableImpactError&){v.refusal="tear_repair_budget";}}
 v.cells=solid+extraSolid;v.mask+=v.edgeCells;const double eraseMs=core::ms(e0);
 v.dirty=core::expandDirtyTiles(touched);
 std::size_t total=0;for(const auto&s2:segments)total+=s2.stations.size();
 if(!v.dirty.empty()&&state.measured.size()==total){std::size_t k2=0;
  for(const auto&s2:segments)for(const auto&station:s2.stations){const std::size_t kk=k2++;if(!state.measured[kk])continue;
   for(const Cell&t:v.dirty)if(t.x>=station.tileLo.x&&t.x<=station.tileHi.x&&t.y>=station.tileLo.y&&t.y<=station.tileHi.y&&t.z>=station.tileLo.z&&t.z<=station.tileHi.z){state.measured[kk]=0;break;}}}
 // the tear record: its cylinder (as today's tissue record) and the decision (no triangles: later cuts plan no walls for it)
 v.point=C;v.axis=n; // a retried tear reports where it was made
 if(seedsJob)seedsJob->wait(); // speed step: the per-seed constants (finishSeeds, started after the proof)
 if(D.bridge){D.P[P_RIP]=bridgeRip(pitch);D.P[P_H]=D.bridgeH;} // damagecut: a bridge tear's rip gap and slab (bridgeMaps)
 // melee (value 3): the clean cut's face - flat (the seeds may have been finished beside), no strands, no bone stand-off or muscle
 // dome, every tissue ending a few mm past the rip slab (depth scale 0.02-0.2, TearForce::tissueDepthScale)
 if(force&&force->clean){for(int q=0;q<7;++q)D.P[P_SEEDC+q]=0.;D.P[P_IMP+3]=0.;D.P[P_STR+4]=-1;D.P[P_STR+9]=-1;
  D.P[P_DSCALE]=std::clamp(force->tissueDepthScale,.02,.2);const std::size_t NN=std::size_t(D.N)*std::size_t(D.N);
  for(std::size_t q=0;q<NN&&q<D.EX.size();++q)D.EX[q]=0.f;}
 auto block=gpuBlock(D);
 // the bake (what the pixel programs read, TearDecide.h bakeTear, maskgame3): 2-D map + 3-D muscle field, GFY_M3_BAKE_MM / GFY_M3_MAP_MM
 static const double bakeMm=[]{const char*e=std::getenv("GFY_M3_BAKE_MM");const double v=e?std::atof(e):4.;return std::clamp(v,1.,8.);}();
 static const double mapMm=[]{const char*e=std::getenv("GFY_M3_MAP_MM");const double v=e?std::atof(e):4.;return std::clamp(v,1.,8.);}();
 // speed step: the bake runs beside the rest of the wound and the separation (TearBakeAsync.h; the drawing set waits for it), the
 // same code per cell in chunks; inside the sever as before when the job keeps no closure field for it (or GFY_MS_BAKE_SYNC)
 Bake2Stats bst;bool bakeAsync=false;const char*dumpDir=std::getenv("GFY_M3_DUMP");static std::atomic<unsigned> dumpCount{0};const unsigned dumpIdx=dumpDir?dumpCount.fetch_add(1):0u;
 {const auto&keep=tear_bake::jobKeep();
  if(!tear_bake::syncOnly()&&keep&&keep.get()==static_cast<const void*>(closure)){bakeAsync=true;const auto*cl=closure;
   auto sampleV=[cl](double x,double y,double z){return cl->sample(x,y,z);};const std::string dd=dumpDir?dumpDir:"";const bool log=logOn();const std::uint32_t sd=seed;
   auto p=bakeTearChunks(std::make_shared<const Decision>(D),bakeMm*.001,mapMm*.001,sampleV,block,[block,dd,dumpIdx,log,sd](const Bake2Stats&s,double ms){
    if(!dd.empty())writeBakeDump(dd.c_str(),dumpIdx,*block);
    if(log)std::cerr<<"tear_bake_done seed="<<sd<<" ms="<<ms<<" bake="<<block->bake_dim[0]<<"x"<<block->bake_dim[1]<<"x"<<block->bake_dim[2]<<" evaluated="<<s.fieldEvaluated<<std::endl;});
   p->keep=keep;bst.mapBytes=block->map.size()*4;bst.fieldBytes=block->bake.size();tear_bake::start(block.get(),p);
   if(dumpDir)p->wait();} // dump runs: the files are written before the sever returns (the probe may end before a bake would)
  else{auto sample=[&](double x,double y,double z){return closure->sample(x,y,z);};
   auto parallel=[&](std::size_t count,const auto&body){core::parallelChunks(count,1,[&](std::size_t a,std::size_t b,std::size_t){body(a,b);});};
   bakeTear(D,bakeMm*.001,mapMm*.001,sample,parallel,*block,bst);}}
 {ResolvedTunnel rec;const Vec a0=u,b0=w;const double reach=half+.025;const Vec e=C-n*reach;
  rec.entry={e.x,e.y,e.z};rec.direction={n.x,n.y,n.z};rec.edge_u={a0.x,a0.y,a0.z};rec.edge_v={b0.x,b0.y,b0.z};rec.radius=R;rec.penetration=2*reach;rec.cap=2*reach;rec.inlet=0;
  rec.exit_distance=2*reach;rec.exit_mult=1;rec.edge_seed=seed64;auto surf=std::make_shared<ResolvedTearSurface>();surf->decide=block;rec.surface=std::move(surf);v.records.push_back(std::move(rec));}
 if(dumpDir){dump(D,dumpDir,dumpIdx);if(!bakeAsync)writeBakeDump(dumpDir,dumpIdx,*block);} // speed step: an asynchronous bake writes its file when done
 v.tissueRan=true;v.tissueMs=core::ms(d0);v.tissueTorn=0;v.tissueFresh=0;v.tissueTriangles=0;v.crossing=0;v.rounds=0;v.carveMs=core::ms(s0t);v.tear=!v.refusal;
 if(v.tear){++state.decided;const double doneR=.02;std::size_t k=0;
  // slice 4: also any other segment's station whose centre lies in this tear's torn region (its footprint, within doneR of the
  // plane): those tissues are this tear's torn ends (a knee tear does not tear the shin's top station again in the same batch)
  if(state.failed.size()==total)for(const auto&s:segments)for(const auto&station:s.stations){bool done=&s==&seg&&std::abs(station.s-st.s)<=doneR;
   if(!done){const Vec d=station.center-C;const double sn=dot(d,n),pa=dot(d,u),pb=dot(d,w);done=std::abs(sn)<=doneR&&D.inFoot(pa,pb);}
   if(done)state.failed[k]=1;++k;}}
 else fallback("tear_repair_budget");
 v.ms=core::ms(t0);
 if(logOn())std::cerr<<"tear_decide_once ok seed="<<seed<<" N="<<N<<" limb_cells="<<D.limbCells<<" muscle_cells="<<D.muscleCells<<" section_ms="<<D.sectionMs<<" seed_ms="<<D.seedMs
  <<" decide_ms="<<decideMs<<" proof_ms="<<proofMs<<" erase_ms="<<eraseMs<<" sheet_cells="<<sheetCells<<" erased="<<boundary.size()<<" edge_cells="<<v.edgeCells
  <<" box="<<nx<<"x"<<ny<<"x"<<nz<<" standing="<<standingInPlane<<" proof="<<proof<<" head="<<headTear<<" owned="<<(g.ownership?1:0)<<" decided="<<state.decided<<" grown="<<grown<<" seam="<<seamCells
  <<" bytes="<<(block->params.size()*4+block->section.size()*4)<<" bake_ms="<<bst.ms<<" bake="<<block->bake_dim[0]<<"x"<<block->bake_dim[1]<<"x"<<block->bake_dim[2]
  <<" map="<<block->map_dim[0]<<"x"<<block->map_dim[1]<<" bake_map_ms="<<bst.mapMs<<" bake_field_ms="<<bst.fieldMs<<" bake_map_kb="<<bst.mapBytes/1024
  <<" bake_field_kb="<<bst.fieldBytes/1024<<" bake_evaluated="<<bst.fieldEvaluated<<" bake_async="<<bakeAsync
  <<" sections_ms="<<tSection<<" sampled="<<secCache.sampled<<" alloc_ms="<<tAlloc<<" class_ms="<<tClass<<" flood_ms="<<tFlood<<" list_ms="<<tList
  <<" flood_visited="<<floodVisited<<" scan_visits="<<(scanVisits().load()-visits0)<<" ms="<<v.ms<<std::endl;
 GFY_DAMAGE_TEAR_REPORT(v);
 if(std::getenv("GFY_M4_GLOBALDBG")){ // probe diagnosis: a whole-grid flood from the limb's core on its side, after the sheet is in
  TissueLookup tissue{g,state.table};core::TileCache cache(g);std::vector<std::uint64_t> seen((std::size_t(g.size.x)*g.size.y*g.size.z+63)/64,0);
  auto idx=[&](const Cell&c){return (std::size_t(c.x)*g.size.y+std::size_t(c.y))*g.size.z+std::size_t(c.z);};
  auto ok=[&](const Cell&c){return g.valid(c)&&tissue.solid(c)&&!cache.removed(c);};
  std::vector<Cell> q;std::vector<std::uint32_t> par;const Vec s0=C+n*.03;const Cell c0{int(std::floor((s0.x-g.origin.x)/pitch)),int(std::floor((s0.y-g.origin.y)/pitch)),int(std::floor((s0.z-g.origin.z)/pitch))};
  if(ok(c0)){q.push_back(c0);par.push_back(0);seen[idx(c0)>>6]|=std::uint64_t(1)<<(idx(c0)&63);}
  std::size_t head=0;bool firstOut=false,reached=false;
  while(head<q.size()&&q.size()<12000000&&!reached){const Cell c=q[head++];const Vec d=g.center(c)-C;const double s=dot(d,n),a=dot(d,u),b=dot(d,w);
   if(!firstOut&&s<-band){firstOut=true;std::cerr<<"M4GLOBAL first_minus s="<<s<<" pa="<<a<<" pb="<<b<<" r="<<std::sqrt(a*a+b*b)<<" label="<<(closure->sample(g.center(c).x,g.center(c).y,g.center(c).z)&63u)<<" visited="<<head<<std::endl;}
   if(s<-.04&&a*a+b*b<coreR*coreR){reached=true;std::size_t at=head-1;int shown=0;double prev=1;
    while(at&&shown<40){const Vec dd=g.center(q[at])-C;const double aa=dot(dd,u),bb=dot(dd,w),ss=dot(dd,n)-D.offset(aa,bb);
     if((ss>=0)!=(prev>=0)){++shown;std::cerr<<"M4PATH cross s="<<ss<<" pa="<<aa<<" pb="<<bb<<" r="<<std::sqrt(aa*aa+bb*bb)<<" foot="<<limbAt(aa,bb)<<" foreign="<<foreignIn(aa,bb)
      <<" label="<<(closure->sample(g.center(q[at]).x,g.center(q[at]).y,g.center(q[at]).z)&63u)<<" step="<<at<<std::endl;}prev=ss;at=par[at];}
    break;}
   const Cell nb[6]={{c.x-1,c.y,c.z},{c.x+1,c.y,c.z},{c.x,c.y-1,c.z},{c.x,c.y+1,c.z},{c.x,c.y,c.z-1},{c.x,c.y,c.z+1}};
   for(const auto&e:nb){if(!g.valid(e))continue;const auto i=idx(e);if((seen[i>>6]>>(i&63))&1u)continue;if(!ok(e))continue;seen[i>>6]|=std::uint64_t(1)<<(i&63);q.push_back(e);par.push_back(std::uint32_t(head-1));}}
  std::size_t minus=0,foreignCells=0,nearSeg[64]={};double sMin=1e9;
  for(std::size_t i=0;i<head;++i){const Vec p=g.center(q[i]);const Vec d=p-C;const double s=dot(d,n);sMin=std::min(sMin,s);minus+=s<0;
   double best=1e30;int k=-1;for(std::size_t j=0;j<segments.size();++j){const auto&sg=segments[j];const double t=std::clamp(dot(p-sg.pivot,sg.axis),0.,sg.length);const double dd=length(p-(sg.pivot+sg.axis*t))-sg.radius;if(dd<best){best=dd;k=int(j);}}
   if(k>=0&&k<64)++nearSeg[k];if(k>=0&&!neighbour.empty()&&!neighbour[std::size_t(k)])++foreignCells;}
  std::cerr<<"M4GLOBAL start_ok="<<ok(c0)<<" reached_minus_core="<<reached<<" visited="<<head<<" queued="<<q.size()<<" minus_side="<<minus<<" s_min="<<sMin<<" foreign_cells="<<foreignCells<<" by_segment_bone=";
  for(std::size_t j=0;j<segments.size()&&j<64;++j)if(nearSeg[j])std::cerr<<segments[j].bone<<":"<<nearSeg[j]<<",";std::cerr<<std::endl;}
 return true;} // the sheet is in the grid (journaled): a repair refusal travels on the verdict, as today's tear refusals do
} // namespace tear_decide
