#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <set>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>
namespace tear_decide {
// ---- tf_core round 2 constants
constexpr double warp=.003,warpCell=.018,chunkCell=.022,chunkOffset=.014,chunkSlant=1.0,chunkBlend=.30,moundAmp=.008,moundCell=.016;
constexpr double bundleCell=.011,bundleReach=.005,exitReach=.002,bundleBlend=.45,tipTaper=.003,borderGroove=.002,fray=.0015,frayCell=.0035;
constexpr double sizeSpread=.5,reachSpread=.8,groupFloor=.024,groupSpan=.022;
constexpr double boneStand=.022,boneStandSpan=.012,shellBelow=.004;
constexpr double boneSide=.014,boneSideFade=.030,dome=.006,domeFade=.012;
constexpr double tiltMax=.22,bowMax=.010,waveMax=.005,offMax=.024;
constexpr double strandR=.011,strandRSpan=.004,strandL=.020,strandLSpan=.014;
enum:unsigned{BONE=0,MUSCLE=1,SHELL=2,SMALL=3,ORGAN=4,OTHER=5,VESSEL=6,FASCIA=7,MARROW=8,MV0=9,NCLS=17};
// P layout (tf_core.py): the decision as one double array (the dump format; the GPU rows are packed from it)
constexpr int P_C=0,P_N=3,P_U=6,P_W=9,P_RL=12,P_H=13,P_MAXD=14,P_RIP=15,P_DSCALE=16,P_IMP=17,P_FE=25,P_SEC=P_FE+8*NCLS,P_LIMB=P_SEC+4,
 P_SEEDC=P_LIMB+5,P_STR=P_SEEDC+16,P_LEN=P_STR+10;
constexpr int GPU_ROWS=15+2*18; // TearParams (painter_tearfn.h): 15 float4 rows + fe[18][2]

inline std::uint32_t h32(std::uint32_t x){x^=x>>16;x*=0x7feb352du;x^=x>>15;x*=0x846ca68bu;x^=x>>16;return x;}
inline double u01(std::uint32_t s,int a,int b,std::uint32_t salt){return double(h32(s^h32(std::uint32_t(a)*0x9e3779b1u+h32(std::uint32_t(b)+salt))))*(1./4294967296.);}
inline double vnoise(std::uint32_t s,std::uint32_t salt,double x,double y){const double x0=std::floor(x),y0=std::floor(y);const int xi=int(x0),yi=int(y0);
 const double fx=x-x0,fy=y-y0,sx=fx*fx*(3-2*fx),sy=fy*fy*(3-2*fy);
 const double c00=2*u01(s,xi,yi,salt)-1,c10=2*u01(s,xi+1,yi,salt)-1,c01=2*u01(s,xi,yi+1,salt)-1,c11=2*u01(s,xi+1,yi+1,salt)-1;
 const double a=c00+(c10-c00)*sx,b=c01+(c11-c01)*sx;return a+(b-a)*sy;}
struct Near {double best,second;int bcx,bcy,scx,scy;};
inline Near nearest(std::uint32_t s,std::uint32_t salt,double gx,double gy){const int X=int(std::floor(gx)),Y=int(std::floor(gy));Near r{1e9,1e9,0,0,0,0};
 for(int a=-1;a<=1;++a)for(int b=-1;b<=1;++b){const int cx=X+a,cy=Y+b;const double fx=cx+u01(s,cx,cy,salt),fy=cy+u01(s,cx,cy,salt+1);
  const double d=(gx-fx)*(gx-fx)+(gy-fy)*(gy-fy);
  if(d<r.best){r.second=r.best;r.scx=r.bcx;r.scy=r.bcy;r.best=d;r.bcx=cx;r.bcy=cy;}else if(d<r.second){r.second=d;r.scx=cx;r.scy=cy;}}
 return r;}
// melee (2026-11-12): the torn ends' depth floor - 2 mm, a clean blade cut's (depth scale below 0.05, TearForce::clean) 0.05 mm
inline double eFloor(double ds){return ds<.05?.00005:.002;}
inline double sstep(double e0,double e1,double x){const double t=std::min(std::max((x-e0)/(e1-e0),0.),1.);return t*t*(3-2*t);}
inline double ease(double d,double lo,double lim){const double hi=lim-.0015,kl=.003,kh=.006;
 if(d<lo+kl)d=lo+kl*std::exp((d-lo-kl)/kl);if(d>hi-kh)d=hi-kh*std::exp(-(d-hi+kh)/kh);return std::min(std::max(d,.002),lim-.001);}
inline double chunkVal(std::uint32_t s,std::uint32_t salt,int cx,int cy,double gx,double gy){
 const double v0=u01(s,cx,cy,salt+4),v1=u01(s,cx,cy,salt+5),v2=u01(s,cx,cy,salt+6);
 return chunkOffset*v0+chunkSlant*((v1-.5)*(gx-cx-.5)+(v2-.5)*(gy-cy-.5))*chunkCell;}
inline double muscleLow(std::uint32_t s,std::uint32_t salt,double x,double y,double base,double down0,double lean){
 const double wa=x+warp*vnoise(s,salt,x/warpCell,y/warpCell),wb=y+warp*vnoise(s,salt+1,x/warpCell,y/warpCell);
 const double gx=wa/chunkCell,gy=wb/chunkCell;const Near t=nearest(s,salt+2,gx,gy);
 const double c1=chunkVal(s,salt,t.bcx,t.bcy,gx,gy),c2=chunkVal(s,salt,t.scx,t.scy,gx,gy);
 const double w=.5*(1-sstep(0.,chunkBlend,std::sqrt(t.second)-std::sqrt(t.best)));
 return base+c1+(c2-c1)*w+moundAmp*vnoise(s,salt+3,x/moundCell,y/moundCell)+.5*lean-.003*down0;}
inline double muscleDetail(std::uint32_t s,std::uint32_t salt,double x,double y,double down0){
 const double gs=u01(s,int(salt+30),7,0x3C1u),go=u01(s,int(salt+31),7,0x3C1u);const double sc=bundleCell*(1+sizeSpread*(gs-.5));
 const Near t=nearest(s,salt+7,x/sc,y/sc);const double t1=u01(s,t.bcx,t.bcy,salt+9),t2=u01(s,t.scx,t.scy,salt+9);
 const double r1=std::sqrt(t.best),r2=std::sqrt(t.second),gap=r2-r1,R=bundleReach*(1+reachSpread*(go-.5))+exitReach*down0;
 const double q1=std::min(1.,r1/.7),q2=std::min(1.,r2/.7),o1=R*t1*t1*(1-.6*q1*q1),o2=R*t2*t2*(1-.6*q2*q2);
 const double w=.5*(1-sstep(0.,bundleBlend,gap)),out=o1+(o2-o1)*w;
 const double groove=borderGroove*(1-std::min(std::max(gap/.25,0.),1.))+tipTaper*.5*(q1*q1+q2*q2)*.5;
 return -out+groove+fray*vnoise(s,salt+8,x/frayCell,y/frayCell);}
inline void muscleSalts(unsigned c,std::uint32_t&salt,std::uint32_t&bsalt){if(c==MUSCLE){salt=180;bsalt=10;return;}const unsigned v=c-MV0;salt=200+20*v;bsalt=100+v;}
inline unsigned classOf(unsigned v,bool filler){
 if(v>=27&&v<=34)return MV0+(v-27);if(v==17)return FASCIA;if(filler)return SHELL;if(v==2)return BONE;if(v==21)return MARROW;if(v==5)return MUSCLE;
 if(v==1||v==15||v==16)return SHELL;if(v==3||v==4||v==14||v==25||v==26)return SMALL;if(v>=18&&v<=20)return VESSEL;if(v>=22&&v<=24)return ORGAN;
 if(v>=6&&v<=13)return ORGAN;return OTHER;}
// class bit of a closure label (0: air, garment air 16)
inline std::uint32_t classBit(unsigned L){const unsigned v=L&63u;if(v==0||v==16)return 0;return 1u<<classOf(v,(L&64u)!=0);}

// ---- the decision
struct Decision {
 double P[P_LEN]{};std::uint32_t seed=0;double pitch=0;int N=0;
 std::vector<float> EX;std::vector<std::uint8_t> FOOT,LIMB; // EX: 3 N^2 (extra, shell floor A, B); FOOT bit 0 bone footprint; LIMB the limb footprint
 double footLo[2]{1e30,1e30},footHi[2]{-1e30,-1e30}; // the footprint's extent in the section frame (a, b)
 double sectionMs=0,seedMs=0;std::size_t limbCells=0,muscleCells=0;
 std::vector<int> wMcls;std::vector<std::uint32_t> wNear8;bool seedsPending=false; // speed step: the section's work for finishSeeds (TearSpeed.h)
 // tear surface offset at (pa, pb) (tf_core tear_offset / HLSL tf_offset)
 double offset(double pa,double pb)const{const double RL=P[P_LIMB+3],x=pa/RL,y=pb/RL;
  const double o=RL*(P[P_SEEDC]*x+P[P_SEEDC+1]*y)+P[P_SEEDC+2]*(x*x+y*y-.5)+P[P_SEEDC+3]*std::sin(P[P_SEEDC+4]*x+P[P_SEEDC+5]*y+P[P_SEEDC+6]);
  return std::min(std::max(o,-offMax),offMax);}
 int cellIndex(double pa,double pb)const{const double O=P[P_SEC],cl=P[P_SEC+1];
  const int qa=std::min(std::max(int(std::floor((pa-O)/cl)),0),N-1),qb=std::min(std::max(int(std::floor((pb-O)/cl)),0),N-1);return qb*N+qa;}
 bool inFoot(double pa,double pb)const{if(pa*pa+pb*pb>P[P_RL]*P[P_RL])return false;return LIMB[std::size_t(cellIndex(pa,pb))]!=0;}
 // damagecut (2026-11-13): a BRIDGE tear tears and draws only what still held the two sides (HoldingCut.h bridgeMaps): its region
 // is BRIDGE (N^2) instead of the footprint, its torn depths capped per side where the damage begins along the normal (CAP, 2 N^2)
 // and tapered to the floor at the wounds (TAPER, N^2); sectionBridge: the CPU footprint (gpuBlock's section flags) is BRIDGE too
 bool bridge=false,sectionBridge=false;double bridgeH=0;std::vector<std::uint8_t> BRIDGE;std::vector<float> TAPER,CAP;
 bool drawn(std::size_t q)const{return bridge?BRIDGE[q]!=0:LIMB[q]!=0;}
 double bridgeShare=1; // bridge tear: its section cells / the limb's (HoldingCut.h bridgeMaps)
 bool shellLevel(int side)const{return P[P_SEEDC+14+side]>0&&!(bridge&&bridgeShare<.75);}
 double sec2(const std::vector<float>&F,std::size_t base,double pa,double pb)const{const double O=P[P_SEC],cl=P[P_SEC+1];const double fa=(pa-O)/cl-.5,fb=(pb-O)/cl-.5;
  const int ia=std::min(std::max(int(std::floor(fa)),0),N-2),jb=std::min(std::max(int(std::floor(fb)),0),N-2);
  const double ta=std::min(std::max(fa-ia,0.),1.),tb=std::min(std::max(fb-jb,0.),1.);const std::size_t r0=base+std::size_t(jb)*std::size_t(N)+std::size_t(ia),r1=r0+std::size_t(N);
  return (F[r0]+(F[r0+1]-F[r0])*ta)*(1-tb)+(F[r1]+(F[r1+1]-F[r1])*ta)*tb;}
 double bridgeE(double depth,int side,double pa,double pb)const{const double fl=eFloor(P[P_DSCALE]);const std::size_t NN=std::size_t(N)*std::size_t(N);
  const double cap=std::max(fl,sec2(CAP,side?NN:0,pa,pb)),t=std::min(std::max(sec2(TAPER,0,pa,pb),0.),1.);const double e=std::min(depth,cap);
  return std::max(fl,fl+(e-fl)*t);}
 double extraAt(double pa,double pb,int ch)const{const double O=P[P_SEC],cl=P[P_SEC+1];const double fa=(pa-O)/cl-.5,fb=(pb-O)/cl-.5;
  const int ia=std::min(std::max(int(std::floor(fa)),0),N-2),jb=std::min(std::max(int(std::floor(fb)),0),N-2);
  const double ta=std::min(std::max(fa-ia,0.),1.),tb=std::min(std::max(fb-jb,0.),1.);const std::size_t r0=std::size_t(ch)*N*N+std::size_t(jb)*N+ia,r1=r0+N;
  return (EX[r0]+(EX[r0+1]-EX[r0])*ta)*(1-tb)+(EX[r1]+(EX[r1+1]-EX[r1])*ta)*tb;}
};
// a muscle group's depth at d (relative to the plane point) on one side, with the section extra (tf_core muscle_eff)
inline double muscleEff(const Decision&D,unsigned c,int side,double dx,double dy,double dz){
 const double*P=D.P;const double pa=dx*P[P_U]+dy*P[P_U+1]+dz*P[P_U+2],pb=dx*P[P_W]+dy*P[P_W+1]+dz*P[P_W+2];
 const int k=P_FE+8*int(c);const double a=dx*P[k]+dy*P[k+1]+dz*P[k+2],b=dx*P[k+3]+dy*P[k+4]+dz*P[k+5];const double sg=side?1.:-1.;
 const double down=std::min(std::max((a*P[k+6]+b*P[k+7])/P[P_IMP+4],-1.),1.),lean=P[P_IMP+3]*down*sg-.006*P[P_IMP+2]*sg,down0=std::max(0.,down);
 const std::uint32_t sd=D.seed^(side?0x51u:0xA3u);std::uint32_t salt,bsalt;muscleSalts(c,salt,bsalt);
 const double d=muscleLow(sd,salt,a,b,groupFloor+groupSpan*u01(sd,int(bsalt),7,0x3C1u),down0,lean)+muscleDetail(sd,salt,a,b,down0);
 return ease(d,.006,P[P_IMP+5])+D.extraAt(pa,pb,0);}

// ---- the whole function on the CPU (tf_core tear_value, round 2): margin (m, > 0 kept) and state 0 / 1 side kept, 2 torn, 3 out
constexpr double boneJag=.006,boneCrumb=.0015,marrowIn=.007,footBelow=.008,shellPast=.006,shellLobes=.014,shellFlaps=.004;
inline double vn(std::uint32_t s,std::uint32_t salt,double x,double y){return vnoise(s,salt,x,y);}
inline double boneDepth(const Decision&D,int side,double pa,double pb,double lean,bool marrow){const double*P=D.P;
 const std::uint32_t s=D.seed^(side?0x51u:0xA3u);const double sg=side?1.:-1.;
 const double ra=pa-P[P_LIMB],rb=pb-P[P_LIMB+1],rad=P[P_LIMB+2],r=std::sqrt(ra*ra+rb*rb)+1e-9;
 const double tip=P[P_SEEDC+8+side]-P[P_SEEDC+10],obl=sg*P[P_SEEDC+11]*(ra*P[P_SEEDC+12]+rb*P[P_SEEDC+13]);
 const double jag=boneJag*(.5+.5*vn(s,84,1.8*ra/r+5.,1.8*rb/r+5.))*std::min(r/rad,1.);
 double d=tip+obl+jag+boneCrumb*vn(s,85,pa/.004,pb/.004)+.3*lean;const double lim=P[P_IMP+5];
 if(marrow){d+=marrowIn+.002*(.5+.5*vn(s,83,pa/.005,pb/.005));return ease(d,.006,lim);}
 return ease(d,.0015,lim);}
inline double otherDepth(const Decision&D,unsigned c,int side,double a,double b,double lean,double lim){
 const std::uint32_t s=D.seed^(side?0x51u:0xA3u);double d;
 if(c==SHELL){const double r=std::sqrt(a*a+b*b)+1e-9,ring=.5+.5*vn(s,21,1.3*a/r+3.1,1.3*b/r+5.3),mb=D.P[P_IMP+6+side];
  d=mb+shellPast+shellLobes*ring+shellFlaps*(.5+.5*vn(s,22,a/.018,b/.018))+.004*(.5+.5*vn(s,23,a/.03,b/.03))+.0008*vn(s,24,a/.004,b/.004)+lean;return ease(d,.010,lim);}
 if(c==SMALL){d=.010+.016*(.5+.5*vn(s,41,a/.014,b/.014))+.002*vn(s,42,a/.006,b/.006)+.5*lean;return ease(d,.006,lim);}
 if(c==ORGAN){d=.006+.014*(.5+.5*vn(s,51,a/.02,b/.02))+.5*lean;return std::min(std::max(d,.002),lim-.001);}
 if(c==VESSEL){const double uu=u01(D.seed,91,7,0x3C1u),pr=side?uu:1-uu;d=groupFloor+groupSpan*pr+.004*(.5+.5*vn(s,92,a/.01,b/.01))+.5*lean;return ease(d,.006,lim);}
 if(c==FASCIA){d=.038+.010*u01(s,93,7,0x3C1u)+.003*vn(s,94,a/.01,b/.01)+lean;return ease(d,.014,lim);}
 d=.008+.012*(.5+.5*vn(s,61,a/.02,b/.02));return std::min(std::max(d,.002),lim-.001);}
inline double strandsAt(const Decision&D,int side,double pa,double pb){double d=0;const double*P=D.P;
 for(int k=0;k<2;++k){const int j=P_STR+5*k;if(P[j+4]<0)continue;const double ea=pa-P[j],eb=pb-P[j+1],R=P[j+2],r2=ea*ea+eb*eb;if(r2>=R*R)continue;
  const double h=P[j+3]*sstep(R,.5*R,std::sqrt(r2));if(int(P[j+4])==side)d-=h;else d+=.5*h;}return d;}
inline int tearValue(const Decision&D,double px,double py,double pz,unsigned lab,double&m){const double*P=D.P;
 const double dx=px-P[P_C],dy=py-P[P_C+1],dz=pz-P[P_C+2];const double s0=dx*P[P_N]+dy*P[P_N+1]+dz*P[P_N+2];
 const double pa=dx*P[P_U]+dy*P[P_U+1]+dz*P[P_U+2],pb=dx*P[P_W]+dy*P[P_W+1]+dz*P[P_W+2];m=1;
 if(std::abs(s0)>P[P_H]+offMax||pa*pa+pb*pb>P[P_RL]*P[P_RL])return 3;const int q=D.cellIndex(pa,pb);if(!D.LIMB[std::size_t(q)])return 3;
 const double s=s0-D.offset(pa,pb);const int side=s>=0?1:0;const unsigned v=lab&63u;const bool filler=(lab&64u)!=0||v==16;const unsigned c=classOf(v,filler);
 const double dd=std::max(0.,std::abs(s)-P[P_RIP]),lim=P[P_IMP+5],ds=P[P_DSCALE];const unsigned fl=D.FOOT[std::size_t(q)];
 const int k=P_FE+8*int(c);const double a=dx*P[k]+dy*P[k+1]+dz*P[k+2],b=dx*P[k+3]+dy*P[k+4]+dz*P[k+5],sg=side?1.:-1.;
 const double down=std::min(std::max((a*P[k+6]+b*P[k+7])/P[P_IMP+4],-1.),1.),lean=P[P_IMP+3]*down*sg-.006*P[P_IMP+2]*sg,down0=std::max(0.,down);
 if(c==BONE||c==MARROW)m=dd-std::max(eFloor(ds),boneDepth(D,side,pa,pb,lean,c==MARROW)*ds);
 else{double depth;
  if(c==MUSCLE||c>=MV0){const std::uint32_t sd=D.seed^(side?0x51u:0xA3u);std::uint32_t salt,bsalt;muscleSalts(c,salt,bsalt);
   depth=ease(muscleLow(sd,salt,a,b,groupFloor+groupSpan*u01(sd,int(bsalt),7,0x3C1u),down0,lean)+muscleDetail(sd,salt,a,b,down0)+strandsAt(D,side,pa,pb),.006,lim);}
  else depth=otherDepth(D,c,side,a,b,lean,lim);
  const double ex=D.extraAt(pa,pb,0);
  const bool lvl=c==SHELL&&D.shellLevel(side);
  if(lvl)depth=std::min(P[P_SEEDC+14+side],lim-.001);else if(c==SHELL)depth=std::max(depth,std::min(std::max(D.extraAt(pa,pb,1+side),P[P_SEEDC+14+side]),lim-.001));
  depth=std::max(eFloor(ds),depth*ds);m=dd-depth;
  if(ex>0&&!lvl){double e=depth+ex;const double capHi=P[P_MAXD]-.0015,capK=.006;if(e>capHi-capK)e=capHi-capK*std::exp(-(e-capHi+capK)/capK);m=std::min(m,dd-e);}
  if((fl&1u)&&(c==FASCIA||c==OTHER||c==SMALL||c==VESSEL||c==ORGAN))m=dd-std::max(eFloor(ds),(boneDepth(D,side,pa,pb,lean,false)+footBelow)*ds);}
 return m>=0?side:2;}

// ---- the bake: the function sampled once over the tear's slab (tear frame a = u, b = w, s = n; cell centres), one byte per cell:
// bit 7 = the side of the tear surface; bits 0-6 = q: 0..124 the margin m = (q - 62) * 0.1 mm (clamped to +-6.2 mm; < 0 = torn),
// 127 = outside the tear's region (the skeleton decides). Tissue cells carry their own label's margin; air cells the shell's
// (the skin's), so the exterior and open air read the same field. Cells farther from the tear surface than the deepest tear
// are kept (q 124) and cells within the rip slab + 2 mm are torn (q 0) without evaluating the function.
struct BakeStats {double ms=0;std::size_t cells=0,evaluated=0,bytes=0;};
template<class Sample,class Parallel>
inline void bake(const Decision&D,double bp,const Sample&sample,const Parallel&parallel,gfy_packed::TearBlock&B,BakeStats&st){
 const auto t0=std::chrono::steady_clock::now();const double*P=D.P;
 const double reach=P[P_RIP]+P[P_MAXD]+footBelow+offMax+2*D.pitch;
 const double a0=D.footLo[0]-2*D.pitch,a1=D.footHi[0]+2*D.pitch,b0=D.footLo[1]-2*D.pitch,b1=D.footHi[1]+2*D.pitch;
 const int nx=std::max(1,int(std::ceil((a1-a0)/bp))),ny=std::max(1,int(std::ceil((b1-b0)/bp))),nz=std::max(1,int(std::ceil(2*reach/bp)));
 B.bake_dim[0]=std::uint32_t(nx);B.bake_dim[1]=std::uint32_t(ny);B.bake_dim[2]=std::uint32_t(nz);B.bake_origin[0]=a0;B.bake_origin[1]=b0;B.bake_origin[2]=-reach;B.bake_pitch=bp;
 B.bake.assign(std::size_t(nx)*ny*nz,0);std::atomic<std::size_t> evaluated{0};
 const double inner=P[P_RIP]+.002,outer=P[P_RIP]+P[P_MAXD]+footBelow+.001;
 parallel(std::size_t(nz),[&](std::size_t z0,std::size_t z1){std::size_t ev=0;
  for(std::size_t z=z0;z<z1;++z)for(int y=0;y<ny;++y)for(int x=0;x<nx;++x){
   const double a=a0+(x+.5)*bp,b=b0+(y+.5)*bp,s=-reach+(double(z)+.5)*bp;
   double p[3];for(int k=0;k<3;++k)p[k]=P[P_C+k]+a*P[P_U+k]+b*P[P_W+k]+s*P[P_N+k];
   std::uint8_t code=127;
   if(std::abs(s)<=P[P_H]+offMax&&a*a+b*b<=P[P_RL]*P[P_RL]&&D.LIMB[std::size_t(D.cellIndex(a,b))]){
    const double sp=s-D.offset(a,b);const unsigned side=sp>=0?1u:0u;double m;
    if(std::abs(sp)>outer)m=.0062;else if(std::abs(sp)<inner)m=-.0062;
    else{unsigned lab=sample(p[0],p[1],p[2]);if((lab&63u)==0||(lab&63u)==16)lab=1;tearValue(D,p[0],p[1],p[2],lab,m);++ev;}
    const int q=std::min(124,std::max(0,int(std::lround(m*1e4))+62));code=std::uint8_t(q|(side<<7));}
   B.bake[(z*std::size_t(ny)+std::size_t(y))*std::size_t(nx)+std::size_t(x)]=code;}
  evaluated+=ev;});
 st.cells=B.bake.size();st.evaluated=evaluated.load();st.bytes=B.bake.size();
 st.ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count();}

// ---- the bake, version 2 (2026-11-12, label-aware): the function split where it is exact.
// For every class but the muscles the tear depth is a function of the position across the tear only (their map frame is the
// section's u, w; lean and the impact are 2-D), so the margin at ANY point is m = |s - offset(a, b)| - ripR - E_class,side(a, b):
// a 2-D map holds offset, the region / bone-footprint flags and E for 9 classes x 2 sides (the 9th: soft tissue on the bone's
// footprint). The muscles' depth runs in their fibre frames: a 3-D field holds E of the cell's muscle group (its label, else the
// section's nearest group), WITHOUT the fibre fray, which the pixel program adds (one value noise, 3.5 mm cells: finer than any
// bake cell); the bone's crumbs (4 mm) likewise. The program reads the label at the sample point, so the cortex ring, marrow and
// the muscle boundaries stay exactly the anatomy's (the version-1 margin bake mixed them away).
constexpr int MAP_CH=20; // offset, flags (1 region, 2 bone footprint), then E for k = 0..8 x side A/B at 2 + 2k + side
inline int mapClass(unsigned c){switch(c){case SHELL:return 0;case SMALL:return 1;case ORGAN:return 2;case VESSEL:return 3;case FASCIA:return 4;
 case OTHER:return 5;case BONE:return 6;case MARROW:return 7;default:return -1;}}
// a non-muscle class's E (margin = dd - E) at (pa, pb) on one side, without the bone's crumbs
inline double flatE0(const Decision&D,int k,int side,double pa,double pb){const double*P=D.P; // damagecut: flatE (below) adds a bridge tear's cap and taper
 const double lim=P[P_IMP+5],ds=P[P_DSCALE],sg=side?1.:-1.;
 const double down=std::min(std::max((pa*P[P_IMP]+pb*P[P_IMP+1])/P[P_IMP+4],-1.),1.),lean=P[P_IMP+3]*down*sg-.006*P[P_IMP+2]*sg;
 auto bone=[&](bool marrow){const std::uint32_t s=D.seed^(side?0x51u:0xA3u);const double ra=pa-P[P_LIMB],rb=pb-P[P_LIMB+1],rad=P[P_LIMB+2],r=std::sqrt(ra*ra+rb*rb)+1e-9;
  double d=P[P_SEEDC+8+side]-P[P_SEEDC+10]+sg*P[P_SEEDC+11]*(ra*P[P_SEEDC+12]+rb*P[P_SEEDC+13])+boneJag*(.5+.5*vn(s,84,1.8*ra/r+5.,1.8*rb/r+5.))*std::min(r/rad,1.)+.3*lean;
  if(marrow){d+=marrowIn+.002*(.5+.5*vn(s,83,pa/.005,pb/.005));return ease(d,.006,lim);}return ease(d,.0015,lim);};
 if(k==6||k==7)return std::max(eFloor(ds),bone(k==7)*ds);
 if(k==8)return std::max(eFloor(ds),(bone(false)+footBelow)*ds);
 static const unsigned cls[6]={SHELL,SMALL,ORGAN,VESSEL,FASCIA,OTHER};const unsigned c=cls[k];
 double depth=otherDepth(D,c,side,pa,pb,lean,lim);
 const bool lvl=c==SHELL&&D.shellLevel(side);
 if(lvl)depth=std::min(P[P_SEEDC+14+side],lim-.001);else if(c==SHELL)depth=std::max(depth,std::min(std::max(D.extraAt(pa,pb,1+side),P[P_SEEDC+14+side]),lim-.001));
 depth=std::max(eFloor(ds),depth*ds);const double ex=D.extraAt(pa,pb,0);
 if(ex>0&&!lvl){double e=depth+ex;const double capHi=P[P_MAXD]-.0015,capK=.006;if(e>capHi-capK)e=capHi-capK*std::exp(-(e-capHi+capK)/capK);depth=std::max(depth,e);}
 return depth;}
inline double flatE(const Decision&D,int k,int side,double pa,double pb){const double e=flatE0(D,k,side,pa,pb);
 if(k==0&&D.shellLevel(side))return e;
 if(D.bridge&&k==0){const double fl=eFloor(D.P[P_DSCALE]);const double t=std::min(std::max(D.sec2(D.TAPER,0,pa,pb),0.),1.);return std::max(fl,fl+(e-fl)*t);}
 return D.bridge?D.bridgeE(e,side,pa,pb):e;} // damagecut
// a muscle group's E at a point (fibre frame), without the fray
inline double muscleE0(const Decision&D,unsigned c,int side,double dx,double dy,double dz,double pa,double pb){const double*P=D.P; // damagecut: muscleE (below)
 const int k=P_FE+8*int(c);const double a=dx*P[k]+dy*P[k+1]+dz*P[k+2],b=dx*P[k+3]+dy*P[k+4]+dz*P[k+5],sg=side?1.:-1.;
 const double down=std::min(std::max((a*P[k+6]+b*P[k+7])/P[P_IMP+4],-1.),1.),lean=P[P_IMP+3]*down*sg-.006*P[P_IMP+2]*sg,down0=std::max(0.,down);
 const std::uint32_t sd=D.seed^(side?0x51u:0xA3u);std::uint32_t salt,bsalt;muscleSalts(c,salt,bsalt);
 const double det=muscleDetail(sd,salt,a,b,down0)-fray*vnoise(sd,salt+8,a/frayCell,b/frayCell);
 double depth=ease(muscleLow(sd,salt,a,b,groupFloor+groupSpan*u01(sd,int(bsalt),7,0x3C1u),down0,lean)+det+strandsAt(D,side,pa,pb),.006,P[P_IMP+5]);
 depth=std::max(eFloor(P[P_DSCALE]),depth*P[P_DSCALE]);const double ex=D.extraAt(pa,pb,0);
 if(ex>0){double e=depth+ex;const double capHi=P[P_MAXD]-.0015,capK=.006;if(e>capHi-capK)e=capHi-capK*std::exp(-(e-capHi+capK)/capK);depth=std::max(depth,e);}
 return depth;}
inline double muscleE(const Decision&D,unsigned c,int side,double dx,double dy,double dz,double pa,double pb){ // damagecut: + a bridge tear's cap and taper
 const double e=muscleE0(D,c,side,dx,dy,dz,pa,pb);return D.bridge?D.bridgeE(e,side,pa,pb):e;}
template<class Sample>
inline bool bakeMuscle(const Decision&D,const Sample&sample,double h3,double a,double b,double s,int q,int side,double&E){const double*P=D.P;
 auto rel=[&](double A,double B,double S,double&x,double&y,double&z){x=A*P[P_U]+B*P[P_W]+S*P[P_N];y=A*P[P_U+1]+B*P[P_W+1]+S*P[P_N+1];z=A*P[P_U+2]+B*P[P_W+2]+S*P[P_N+2];};
 double dx,dy,dz;rel(a,b,s,dx,dy,dz);
 const unsigned c=classOf(sample(P[P_C]+dx,P[P_C+1]+dy,P[P_C+2]+dz)&63u,false);
 if(c==MUSCLE||c>=MV0){E=muscleE(D,c,side,dx,dy,dz,a,b);return true;}
 static const bool old=std::getenv("GFY_CE_BAKE_OLD")!=nullptr;
 if(!old){std::uint32_t seen=0;double best=-1;
  for(int o=0;o<8;++o){double ex,ey,ez;rel(a+((o&1)?.25:-.25)*h3,b+((o&2)?.25:-.25)*h3,s+((o&4)?.25:-.25)*h3,ex,ey,ez);
   const unsigned co=classOf(sample(P[P_C]+ex,P[P_C+1]+ey,P[P_C+2]+ez)&63u,false);if(!(co==MUSCLE||co>=MV0)||((seen>>co)&1u))continue;seen|=1u<<co;
   best=std::max(best,muscleE(D,co,side,dx,dy,dz,a,b));}
  if(best>=0){E=best;return true;}}
 const int nc=int(D.FOOT[std::size_t(q)]>>1)-1;if(nc<0)return false;E=muscleE(D,unsigned(nc),side,dx,dy,dz,a,b);return true;}
struct Bake2Stats {double ms=0,mapMs=0,fieldMs=0;std::size_t mapBytes=0,fieldBytes=0,fieldEvaluated=0;};
// The box (tear frame): a in [a0, a0 + na h3), b likewise, s (from the plane) in [s0, s0 + ns h3); the 2-D map covers a, b at h2.
struct BakeBox {double a0,b0,s0,a1,b1,s1;};
template<class Sample,class Parallel>
inline void bakeBox(const Decision&D,const BakeBox&box,double h3,double h2,const Sample&sample,const Parallel&parallel,gfy_packed::TearBlock&B,Bake2Stats&st){
 const auto t0=std::chrono::steady_clock::now();const double*P=D.P;
 const int mx=std::max(1,int(std::ceil((box.a1-box.a0)/h2))),my=std::max(1,int(std::ceil((box.b1-box.b0)/h2)));
 B.map_dim[0]=std::uint32_t(mx);B.map_dim[1]=std::uint32_t(my);B.map_origin[0]=box.a0;B.map_origin[1]=box.b0;B.map_pitch=h2;
 B.map.assign(std::size_t(mx)*my*MAP_CH,0.f);
 parallel(std::size_t(my),[&](std::size_t y0,std::size_t y1){for(std::size_t y=y0;y<y1;++y)for(int x=0;x<mx;++x){
  const double a=box.a0+(x+.5)*h2,b=box.b0+(double(y)+.5)*h2;float*o=&B.map[(y*std::size_t(mx)+std::size_t(x))*MAP_CH];
  const bool in=a*a+b*b<=P[P_RL]*P[P_RL]&&D.drawn(std::size_t(D.cellIndex(a,b)));o[0]=float(D.offset(a,b)); // damagecut: a bridge tear's region is its bridge
  o[1]=float((in?1:0)|(in&&(D.FOOT[std::size_t(D.cellIndex(a,b))]&1u)?2:0)|(D.bridge&&a*a+b*b<=P[P_RL]*P[P_RL]&&(D.sectionBridge?D.BRIDGE:D.LIMB)[std::size_t(D.cellIndex(a,b))]?4:0)); // 4: the footprint
  for(int k=0;k<9;++k)for(int side=0;side<2;++side)o[2+2*k+side]=float(flatE(D,k,side,a,b));}});
 st.mapBytes=B.map.size()*4;st.mapMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count();
 const auto t1=std::chrono::steady_clock::now();
 const int nx=std::max(1,int(std::ceil((box.a1-box.a0)/h3))),ny=std::max(1,int(std::ceil((box.b1-box.b0)/h3))),nz=std::max(1,int(std::ceil((box.s1-box.s0)/h3)));
 B.bake_dim[0]=std::uint32_t(nx);B.bake_dim[1]=std::uint32_t(ny);B.bake_dim[2]=std::uint32_t(nz);B.bake_origin[0]=box.a0;B.bake_origin[1]=box.b0;B.bake_origin[2]=box.s0;B.bake_pitch=h3;
 B.bake.assign(std::size_t(nx)*ny*nz,255);std::atomic<std::size_t> evaluated{0};
 const double inner=P[P_RIP]+.0015,outer=P[P_RIP]+P[P_MAXD]+.004;
 parallel(std::size_t(nz),[&](std::size_t z0,std::size_t z1){std::size_t ev=0;
  for(std::size_t z=z0;z<z1;++z)for(int y=0;y<ny;++y)for(int x=0;x<nx;++x){
   const double a=box.a0+(x+.5)*h3,b=box.b0+(y+.5)*h3,s=box.s0+(double(z)+.5)*h3;
   if(a*a+b*b>P[P_RL]*P[P_RL])continue;const int q=D.cellIndex(a,b);if(!D.drawn(std::size_t(q)))continue; // damagecut: the region
   const double sp=s-D.offset(a,b);if(std::abs(sp)<inner-h3||std::abs(sp)>outer+h3)continue;
   double E=0;if(!bakeMuscle(D,sample,h3,a,b,s,q,sp>=0?1:0,E))continue;++ev; // cutends v2: a cell centred off the muscle takes its groups' deepest
   B.bake[(z*std::size_t(ny)+std::size_t(y))*std::size_t(nx)+std::size_t(x)]=std::uint8_t(std::min(254,std::max(0,int(std::lround(E/.00032)))));}
  evaluated+=ev;});
 st.fieldBytes=B.bake.size();st.fieldEvaluated=evaluated.load();st.fieldMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t1).count();
 // the header the pixel programs read (26 rows of 4 words; word 19 = the piece's side, set by the renderer)
 B.header.assign(4*26,0u);auto F=[&](int i,double v){const float f=float(v);std::memcpy(&B.header[std::size_t(i)],&f,4);};
 for(int k=0;k<3;++k){F(k,P[P_C+k]);F(4+k,P[P_U+k]);F(8+k,P[P_W+k]);F(12+k,P[P_N+k]);}
 F(3,h3);F(7,box.a0);F(11,box.b0);F(15,box.s0);B.header[16]=std::uint32_t(nx);B.header[17]=std::uint32_t(ny);B.header[18]=std::uint32_t(nz);
 F(20,box.a0);F(21,box.b0);F(22,h2);F(23,P[P_RIP]);B.header[24]=std::uint32_t(mx);B.header[25]=std::uint32_t(my);B.header[26]=D.seed;
 F(28,P[P_H]);F(29,P[P_RL]);F(30,P[P_DSCALE]);
 for(int k=0;k<9;++k){const unsigned c=k==0?MUSCLE:MV0+unsigned(k-1);const int j=P_FE+8*int(c);for(int a=0;a<3;++a){F(4*(8+2*k)+a,P[j+a]);F(4*(9+2*k)+a,P[j+3+a]);}}
 st.ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count();}
// the tear's own box: the footprint's extent across, the tear's reach along the normal (the coverage box)
template<class Sample,class Parallel>
inline void bakeTear(const Decision&D,double h3,double h2,const Sample&sample,const Parallel&parallel,gfy_packed::TearBlock&B,Bake2Stats&st){
 const double reach=D.P[P_RIP]+D.P[P_MAXD]+footBelow+offMax+2*D.pitch;
 bakeBox(D,BakeBox{D.footLo[0]-2*D.pitch,D.footLo[1]-2*D.pitch,-reach,D.footHi[0]+2*D.pitch,D.footHi[1]+2*D.pitch,reach},h3,h2,sample,parallel,B,st);}

// 2-D exact squared distance transform (Felzenszwalb) on N x N, seeds where f==0
inline void edt2(std::vector<double>&f,int N){
 const double INF=1e20;std::vector<double> fin(static_cast<std::size_t>(N)),fout(static_cast<std::size_t>(N)),zb(static_cast<std::size_t>(N)+1);std::vector<int> vb(static_cast<std::size_t>(N));
 auto line=[&](){int k=-1;
  for(int q=0;q<N;++q){if(!(fin[std::size_t(q)]<INF))continue;double s=-1e30;
   while(k>=0){const int pp=vb[std::size_t(k)];s=((fin[std::size_t(q)]+double(q)*q)-(fin[std::size_t(pp)]+double(pp)*pp))/(2.*(q-pp));if(s>zb[std::size_t(k)])break;--k;}
   ++k;vb[std::size_t(k)]=q;zb[std::size_t(k)]=k==0?-1e30:s;zb[std::size_t(k)+1]=1e30;}
  if(k<0){for(int q=0;q<N;++q)fout[std::size_t(q)]=INF;return;}
  int j=0;for(int q=0;q<N;++q){while(zb[std::size_t(j)+1]<q)++j;const int pp=vb[std::size_t(j)];fout[std::size_t(q)]=double(q-pp)*(q-pp)+fin[std::size_t(pp)];}};
 for(int y=0;y<N;++y){for(int x=0;x<N;++x)fin[std::size_t(x)]=f[std::size_t(y)*N+x];line();for(int x=0;x<N;++x)f[std::size_t(y)*N+x]=fout[std::size_t(x)];}
 for(int x=0;x<N;++x){for(int y=0;y<N;++y)fin[std::size_t(y)]=f[std::size_t(y)*N+x];line();for(int y=0;y<N;++y)f[std::size_t(y)*N+x]=fout[std::size_t(y)];}}
// 4-connected labels of mask (0 = background), returns the label count
inline int label4(const std::vector<std::uint8_t>&m,int N,std::vector<int>&lab){lab.assign(m.size(),0);int next=0;std::vector<int> st;
 for(int q0=0;q0<N*N;++q0){if(!m[std::size_t(q0)]||lab[std::size_t(q0)])continue;++next;lab[std::size_t(q0)]=next;st.push_back(q0);
  while(!st.empty()){const int q=st.back();st.pop_back();const int x=q%N,y=q/N;const int nb[4]={x>0?q-1:-1,x+1<N?q+1:-1,y>0?q-N:-1,y+1<N?q+N:-1};
   for(int j:nb)if(j>=0&&m[std::size_t(j)]&&!lab[std::size_t(j)]){lab[std::size_t(j)]=next;st.push_back(j);}}}
 return next;}

// The section map (tf_core build_section, engine sampling: the closure field at 2 x 2 points per section cell; class bits over the
// 8 mm slab on 2 mm levels; the limb's footprint = the plane's tissue connected to the core (25 mm) plus, at levels every 6 mm
// across the tear's reach, the tissue connected to it there; grown by two cells) and the per-seed constants (seed_consts).
// slice 4: allow(a, b) = false where this tear may not reach (a head fracture's own part of its plane): no footprint there
template<class Sample,class Parallel,class Allow>
inline void buildSection(Decision&D,const Sample&sample,const Parallel&parallel,const Allow&allow){
 const auto t0=std::chrono::steady_clock::now();double*P=D.P;
 const double Rl=P[P_RL],cell=2*D.pitch,secO=-(Rl+2*D.pitch);const int N=int(std::ceil(2*(Rl+2*D.pitch)/cell))+1;D.N=N;
 P[P_SEC]=secO;P[P_SEC+1]=cell;P[P_SEC+2]=N;const std::size_t NN=std::size_t(N)*N;
 const double C[3]={P[P_C],P[P_C+1],P[P_C+2]},n[3]={P[P_N],P[P_N+1],P[P_N+2]},u[3]={P[P_U],P[P_U+1],P[P_U+2]},w[3]={P[P_W],P[P_W+1],P[P_W+2]};
 auto at=[&](double a,double b,double s){double p[3];for(int k=0;k<3;++k)p[k]=C[k]+a*u[k]+b*w[k]+s*n[k];return sample(p[0],p[1],p[2]);};
 const double sub[2]={-cell/4,cell/4};
 std::vector<std::uint32_t> bits(NN,0);std::vector<std::uint8_t> tissue0(NN,0);
 parallel(std::size_t(N),[&](std::size_t r0,std::size_t r1){for(std::size_t jb=r0;jb<r1;++jb)for(int ia=0;ia<N;++ia){const std::size_t q=jb*N+std::size_t(ia);
  const double A=secO+(ia+.5)*cell,B=secO+(double(jb)+.5)*cell;std::uint32_t m=0;bool t0v=false;
  for(int L=-4;L<=4;++L){const double s=L*.002;for(double da:sub)for(double db:sub){const unsigned lab=at(A+da,B+db,s);m|=classBit(lab);if(L==0&&(lab&63u)!=0&&(lab&63u)!=16)t0v=true;}}
  bits[q]=m;tissue0[q]=t0v&&allow(A,B)?1:0;}});
 auto r2At=[&](std::size_t q){const double A=secO+(double(q%N)+.5)*cell,B=secO+(double(q/N)+.5)*cell;return A*A+B*B;};
 std::vector<int> lab;label4(tissue0,N,lab);std::vector<char> keepL(NN+1,0);
 for(std::size_t q=0;q<NN;++q)if(tissue0[q]&&r2At(q)<=.025*.025)keepL[std::size_t(lab[q])]=1;
 std::vector<std::uint8_t> limb(NN,0);for(std::size_t q=0;q<NN;++q)limb[q]=(lab[q]&&keepL[std::size_t(lab[q])]&&r2At(q)<=Rl*Rl)?1:0;
 const std::vector<std::uint8_t> limb0=limb;
 const double reach=P[P_RIP]+P[P_MAXD]+offMax;std::vector<double> levels;for(double s=-reach;s<=reach+1e-6;s+=.006)levels.push_back(s);
 std::vector<std::vector<std::uint8_t>> levTissue(levels.size());
 parallel(levels.size(),[&](std::size_t l0,std::size_t l1){for(std::size_t li=l0;li<l1;++li){auto&lev=levTissue[li];lev.assign(NN,0);
  for(std::size_t q=0;q<NN;++q){const double A=secO+(double(q%N)+.5)*cell,B=secO+(double(q/N)+.5)*cell;bool t=false;
   for(double da:sub)for(double db:sub){const unsigned lb=at(A+da,B+db,levels[li]);t=t||((lb&63u)!=0&&(lb&63u)!=16);}lev[q]=t&&allow(A,B)?1:0;}}});
 for(const auto&lev:levTissue){std::vector<int> ll;label4(lev,N,ll);std::vector<char> kk(NN+1,0);
  for(std::size_t q=0;q<NN;++q)if(lev[q]&&limb0[q])kk[std::size_t(ll[q])]=1;
  for(std::size_t q=0;q<NN;++q)if(ll[q]&&kk[std::size_t(ll[q])]&&r2At(q)<=Rl*Rl)limb[q]=1;}
 for(int it=0;it<2;++it){std::vector<std::uint8_t> d=limb;for(int y=0;y<N;++y)for(int x=0;x<N;++x){const std::size_t q=std::size_t(y)*N+x;if(limb[q])continue;
  if((x>0&&limb[q-1])||(x+1<N&&limb[q+1])||(y>0&&limb[q-N])||(y+1<N&&limb[q+N]))d[q]=1;}limb.swap(d);}
 for(std::size_t q=0;q<NN;++q)if(r2At(q)>Rl*Rl||!allow(secO+(double(q%N)+.5)*cell,secO+(double(q/N)+.5)*cell))limb[q]=0; // engine: the footprint ends at the cylinder (the function's region) and where it may not reach
 for(std::size_t q=0;q<NN;++q)if(!limb[q])bits[q]=0;
 const std::uint32_t boneBits=(1u<<BONE)|(1u<<MARROW),musBits=(0xFFu<<MV0)|(1u<<MUSCLE);
 std::vector<double> dB(NN),dM(NN);std::vector<int> grp(NN,-2);
 for(std::size_t q=0;q<NN;++q){const std::uint32_t mm=bits[q]&musBits;dB[q]=(bits[q]&boneBits)?0.:1e20;
  if(!bits[q]){grp[q]=-2;continue;}grp[q]=mm==0?-2:((mm&(mm-1))==0?int(std::log2(double(mm))+.5):-1);}
 for(int y=0;y<N;++y)for(int x=0;x<N;++x){const std::size_t q=std::size_t(y)*N+x;const std::uint32_t mm=bits[q]&musBits;bool border=mm&&(mm&(mm-1));
  if(grp[q]>=0){const int nb[4][2]={{x,y-1},{x,y+1},{x-1,y},{x+1,y}};for(const auto&e:nb){const int o=e[0]<0||e[1]<0||e[0]>=N||e[1]>=N?-2:grp[std::size_t(e[1])*N+std::size_t(e[0])];if(o!=grp[q])border=true;}}
  dM[q]=border?0.:1e20;}
 edt2(dB,N);edt2(dM,N);
 auto fade=[](double t){t=std::min(std::max(t,0.),1.);return t>=1?0.:1-t*t*(3-2*t);};
 D.EX.assign(3*NN,0.f);D.FOOT.assign(NN,0);D.LIMB=limb;
 for(std::size_t q=0;q<NN;++q){const double b=std::sqrt(dB[q])*cell,m=std::sqrt(dM[q])*cell;D.EX[q]=float(boneSide*fade(b/boneSideFade)+dome*fade(m/domeFade));D.FOOT[q]=b<=1.01*cell?1:0;}
 // nearest muscle group per cell (multi-source BFS, 8-neighbour, approximate nearest)
 std::vector<int> mcls(NN,-1);
 for(std::size_t q=0;q<NN;++q){const std::uint32_t mm=bits[q]&musBits;if(!mm)continue;if(mm&(1u<<MUSCLE)){mcls[q]=MUSCLE;continue;}for(unsigned c=MV0;c<MV0+8;++c)if(mm&(1u<<c)){mcls[q]=int(c);break;}}
 std::vector<int> nearCls=mcls;{std::vector<std::size_t> cur,nxt;for(std::size_t q=0;q<NN;++q)if(mcls[q]>=0)cur.push_back(q);
  while(!cur.empty()){nxt.clear();for(const auto q:cur){const int x=int(q%N),y=int(q/N);for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){const int X=x+dx,Y=y+dy;
    if(X<0||Y<0||X>=N||Y>=N)continue;const std::size_t j=std::size_t(Y)*N+X;if(nearCls[j]>=0)continue;nearCls[j]=nearCls[q];nxt.push_back(j);}}cur.swap(nxt);}}
 for(std::size_t q=0;q<NN;++q)if(nearCls[q]>=0)D.FOOT[q]=std::uint8_t(D.FOOT[q]|((nearCls[q]+1)<<1)); // tf_core: FOOT bits 1-5 = nearest muscle group + 1
 // bone centre / radius, limb radius
 {double sa=0,sb=0;std::size_t nb=0,nl=0;for(std::size_t q=0;q<NN;++q){if(limb0[q])++nl;if(!(bits[q]&boneBits))continue;sa+=secO+(double(q%N)+.5)*cell;sb+=secO+(double(q/N)+.5)*cell;++nb;}
  if(nb){P[P_LIMB]=sa/double(nb);P[P_LIMB+1]=sb/double(nb);P[P_LIMB+2]=std::max(.006,std::sqrt(double(nb)*cell*cell/3.141592653589793));}else{P[P_LIMB]=0;P[P_LIMB+1]=0;P[P_LIMB+2]=.012;}
  P[P_LIMB+3]=std::max(.03,std::sqrt(double(nl)*cell*cell/3.141592653589793));}
 for(std::size_t q=0;q<NN;++q){if(!limb[q])continue;++D.limbCells;const double A=secO+(double(q%N)+.5)*cell,B=secO+(double(q/N)+.5)*cell;
  D.footLo[0]=std::min(D.footLo[0],A-cell/2);D.footHi[0]=std::max(D.footHi[0],A+cell/2);D.footLo[1]=std::min(D.footLo[1],B-cell/2);D.footHi[1]=std::max(D.footHi[1],B+cell/2);}
 // the muscle groups within 8 mm (5 x 5 cells) of each cell; else the nearest group
 std::vector<std::uint32_t> near8(NN,0);
 for(int y=0;y<N;++y)for(int x=0;x<N;++x){const std::size_t q=std::size_t(y)*N+x;if(!limb[q])continue;std::uint32_t m=0;
  for(int dy=-2;dy<=2;++dy)for(int dx=-2;dx<=2;++dx){const int X=x+dx,Y=y+dy;if(X<0||Y<0||X>=N||Y>=N)continue;m|=bits[std::size_t(Y)*N+X]&musBits;}
  if(!m&&nearCls[q]>=0)m=1u<<unsigned(nearCls[q]);near8[q]=m;}
 D.sectionMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count();
 const auto t1=std::chrono::steady_clock::now();
 // ---- per-seed constants (seed_consts)
 const std::uint32_t seed=D.seed;auto U=[&](int k){return u01(seed,300+k,11,0x5EEDu);};const int j=P_SEEDC;
 {const double ang=2*3.141592653589793*U(0),mag=tiltMax*(.35+.65*U(1));P[j]=mag*std::cos(ang);P[j+1]=mag*std::sin(ang);
  P[j+2]=bowMax*(2*U(2)-1);P[j+3]=waveMax*(.3+.7*U(3));const double wa=2*3.141592653589793*U(4),wk=2.0+3.0*U(5);
  P[j+4]=wk*std::cos(wa);P[j+5]=wk*std::sin(wa);P[j+6]=2*3.141592653589793*U(6);}
 std::vector<std::size_t> mus;for(std::size_t q=0;q<NN;++q)if(mcls[q]>=0&&limb[q])mus.push_back(q);D.muscleCells=mus.size();
 auto rel=[&](std::size_t q,double&dx,double&dy,double&dz){const double A=secO+(double(q%N)+.5)*cell,B=secO+(double(q/N)+.5)*cell;dx=A*u[0]+B*w[0];dy=A*u[1]+B*w[1];dz=A*u[2]+B*w[2];};
 for(int side=0;side<2;++side){double m;
  if(!mus.empty()){std::vector<double> vals(mus.size());
   parallel(mus.size(),[&](std::size_t a,std::size_t b){for(std::size_t i=a;i<b;++i){double dx,dy,dz;rel(mus[i],dx,dy,dz);vals[i]=muscleEff(D,unsigned(mcls[mus[i]]),side,dx,dy,dz);}});
   std::sort(vals.begin(),vals.end());const double pos=.08*double(vals.size()-1);const std::size_t lo=std::size_t(pos);const double fr=pos-double(lo);
   m=vals[lo]+(lo+1<vals.size()?(vals[lo+1]-vals[lo])*fr:0.);}
  else m=groupFloor+.5*groupSpan;
  P[j+8+side]=m;P[j+14+side]=0;}
 // shell floor (section channels 1 / 2)
 parallel(NN,[&](std::size_t a,std::size_t b){for(std::size_t q=a;q<b;++q){const std::uint32_t m=near8[q];if(!m)continue;double dx,dy,dz;rel(q,dx,dy,dz);
  for(int side=0;side<2;++side){double best=0;for(unsigned c=0;c<NCLS;++c)if((m>>c)&1u)best=std::max(best,muscleEff(D,c,side,dx,dy,dz));D.EX[NN*std::size_t(1+side)+q]=float(best+shellBelow);}}});
 {static const bool lvlOff=std::getenv("GFY_SHELL_LEVEL_OFF")!=nullptr;
  for(int side=0;side<2;++side){double top=0;for(std::size_t q=0;q<NN;++q)if(limb[q])top=std::max(top,double(D.EX[NN*std::size_t(1+side)+q]));P[j+14+side]=lvlOff?0.:top;}}
 P[j+10]=boneStand+boneStandSpan*U(7);P[j+11]=.20+.25*U(8);{const double oa=2*3.141592653589793*U(9);P[j+12]=std::cos(oa);P[j+13]=std::sin(oa);}
 for(int k=0;k<2;++k){const int jj=P_STR+5*k;P[jj+4]=-1;if(mus.empty()||(k==1&&U(20)<.35))continue;double a_=0,b_=0;
  for(int tries=0;tries<8;++tries){const std::size_t idx=std::min(std::size_t(U(30+8*k+tries)*double(mus.size())),mus.size()-1);const std::size_t q=mus[idx];
   a_=secO+(double(q%N)+.5)*cell;b_=secO+(double(q/N)+.5)*cell;
   if(std::hypot(a_-P[P_LIMB],b_-P[P_LIMB+1])>P[P_LIMB+2]+.018&&std::hypot(a_,b_)<.55*P[P_LIMB+3])break;}
  P[jj]=a_;P[jj+1]=b_;P[jj+2]=strandR+strandRSpan*U(40+k);P[jj+3]=strandL+strandLSpan*U(42+k);P[jj+4]=U(44+k)<.5?1:0;}
 D.seedMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t1).count();}

// the GPU form (painter_tearfn.h TearParams rows; section map float4 = extra, flags (1 limb, 2 bone footprint), shell floor A, B)
inline std::shared_ptr<gfy_packed::TearBlock> gpuBlock(const Decision&D){
 auto B=std::make_shared<gfy_packed::TearBlock>();const double*P=D.P;B->params.assign(4*GPU_ROWS,0u);
 auto F=[&](int row,int c,double v){const float f=float(v);std::uint32_t u;std::memcpy(&u,&f,4);B->params[std::size_t(4*row+c)]=u;};
 auto row=[&](int r,double a,double b,double c,double d){F(r,0,a);F(r,1,b);F(r,2,c);F(r,3,d);};
 row(0,P[P_C],P[P_C+1],P[P_C+2],P[P_RL]);row(1,P[P_N],P[P_N+1],P[P_N+2],P[P_H]);row(2,P[P_U],P[P_U+1],P[P_U+2],P[P_MAXD]);row(3,P[P_W],P[P_W+1],P[P_W+2],P[P_RIP]);
 row(4,P[P_IMP],P[P_IMP+1],P[P_IMP+2],P[P_IMP+3]);row(5,P[P_IMP+4],P[P_IMP+5],P[P_IMP+6],P[P_IMP+7]);row(6,P[P_SEC],P[P_SEC+1],P[P_SEC+2],P[P_DSCALE]);
 B->params[28]=D.seed;B->params[29]=0;B->params[30]=P[P_STR+4]<0?0xFFFFFFFFu:std::uint32_t(P[P_STR+4]);B->params[31]=P[P_STR+9]<0?0xFFFFFFFFu:std::uint32_t(P[P_STR+9]);
 row(8,P[P_LIMB],P[P_LIMB+1],P[P_LIMB+2],P[P_LIMB+3]);row(9,P[P_SEEDC],P[P_SEEDC+1],P[P_SEEDC+2],P[P_SEEDC+3]);row(10,P[P_SEEDC+4],P[P_SEEDC+5],P[P_SEEDC+6],0);
 row(11,P[P_SEEDC+8],P[P_SEEDC+9],P[P_SEEDC+10],P[P_SEEDC+11]);row(12,P[P_SEEDC+12],P[P_SEEDC+13],0,0);
 row(13,P[P_STR],P[P_STR+1],P[P_STR+2],P[P_STR+3]);row(14,P[P_STR+5],P[P_STR+6],P[P_STR+7],P[P_STR+8]);
 for(unsigned c=0;c<NCLS;++c){const int k=P_FE+8*int(c);row(15+2*int(c),P[k],P[k+1],P[k+2],P[k+6]);row(16+2*int(c),P[k+3],P[k+4],P[k+5],P[k+7]);}
 const std::size_t NN=std::size_t(D.N)*D.N;B->n=std::uint32_t(D.N);B->section.assign(4*NN,0.f);
 for(std::size_t q=0;q<NN;++q){B->section[4*q]=D.EX[q];B->section[4*q+1]=float(((D.sectionBridge?D.BRIDGE[q]!=0:D.LIMB[q]!=0)?1:0)|(D.FOOT[q]&1u?2:0)); /* damagecut: a holding-cells sever's footprint is its bridge */B->section[4*q+2]=D.EX[NN+q];B->section[4*q+3]=D.EX[2*NN+q];}
 for(int k=0;k<3;++k){B->centre[k]=P[P_C+k];B->normal[k]=P[P_N+k];B->u[k]=P[P_U+k];B->w[k]=P[P_W+k];}
 B->footLo[0]=D.footLo[0];B->footLo[1]=D.footLo[1];B->footHi[0]=D.footHi[0];B->footHi[1]=D.footHi[1];
 B->reach=P[P_RIP]+P[P_MAXD]+footBelow+offMax+2*D.pitch;B->pitch=D.pitch;B->seed=D.seed;
 return B;}

// The dump (probe / picture): the decision in tf_core's P layout plus the section map, one binary file per tear
inline void dump(const Decision&D,const char*dir,unsigned index){char path[1024];std::snprintf(path,sizeof(path),"%s/m3tear_%u.bin",dir,index);
 if(std::FILE*o=std::fopen(path,"wb")){const char magic[8]={'M','3','T','E','A','R','0','1'};std::fwrite(magic,1,8,o);const std::int32_t hdr[3]={P_LEN,D.N,std::int32_t(D.seed)};std::fwrite(hdr,4,3,o);
  std::fwrite(&D.pitch,8,1,o);std::fwrite(D.P,8,P_LEN,o);std::fwrite(D.EX.data(),4,D.EX.size(),o);std::fwrite(D.FOOT.data(),1,D.FOOT.size(),o);std::fwrite(D.LIMB.data(),1,D.LIMB.size(),o);std::fclose(o);}}
} // namespace tear_decide
