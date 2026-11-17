#pragma once
// Mask-only wounds, speed step (painter_voxel_mask_only 3, 2026-11-12): the decide-once tear's section, seeds and slab scans with
// the same numbers in less time. Included by TearDecideTear.h right after TearDecide.h.
//  * buildSectionFast = TearDecide.h buildSection (slice 4 form, with the allow mask), same results:
//    - the closure is sampled only at cells the tear may reach (allow at the cell centre): every sample-derived value of the
//      former function (class bits, the plane's tissue, the levels' tissue) is used only where allow holds there;
//    - one plane's samples serve all of decideOnce's attempts on that plane (SectionCache: a retry with a wider footprint
//      samples only its new ring);
//    - a level's four sub-samples stop at the first tissue hit (pure lookups; the former loop took all four);
//    - the levels are labelled side by side (each level's growth reads the plane's own footprint only; the union is order-free);
//    - the per-seed constants the sheet and its proof never read (muscle medians, shell floor, bone, strands) wait until the sheet
//      separates (finishSeeds, before the GPU block); the offset constants come first, as before. A refused attempt skips them.
//  * slabScan: the cells of a box whose distance along the normal lies in a range, clipped along the box axis on which the normal
//    is largest (the former loops clipped z only: a thigh's normal along y scanned its whole box), rows taken dynamically.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>
#include "TearBakeAsync.h"
namespace tear_decide {
// the probe's bake dump (TearDecideTear.h's former block): magic, header words (104), map floats, field bytes
inline void writeBakeDump(const char*dir,unsigned idx,const gfy_packed::TearBlock&b){char path[1024];std::snprintf(path,sizeof(path),"%s/m3bake_%u.bin",dir,idx);
 if(std::FILE*o=std::fopen(path,"wb")){std::fwrite("M3BAKE02",1,8,o);std::fwrite(b.header.data(),4,b.header.size(),o);
  std::fwrite(b.map.data(),4,b.map.size(),o);std::fwrite(b.bake.data(),1,b.bake.size(),o);std::fclose(o);}}
// one plane's section samples (pure functions of the plane, the cell layout and the closure field)
struct SectionCache {bool on=false;double key[16]{};int N=0;std::size_t L=0;std::vector<std::uint32_t> bits;std::vector<std::uint8_t> t0,lev,done;std::size_t sampled=0;};
template<class Sample,class Parallel,class Allow>
inline void buildSectionFast(Decision&D,const Sample&sample,const Parallel&parallel,const Allow&allow,SectionCache&S){
 const auto t0=std::chrono::steady_clock::now();double*P=D.P;
 const double Rl=P[P_RL],cell=2*D.pitch,secO=-(Rl+2*D.pitch);const int N=int(std::ceil(2*(Rl+2*D.pitch)/cell))+1;D.N=N;
 P[P_SEC]=secO;P[P_SEC+1]=cell;P[P_SEC+2]=N;const std::size_t NN=std::size_t(N)*N;
 const double C[3]={P[P_C],P[P_C+1],P[P_C+2]},n[3]={P[P_N],P[P_N+1],P[P_N+2]},u[3]={P[P_U],P[P_U+1],P[P_U+2]},w[3]={P[P_W],P[P_W+1],P[P_W+2]};
 auto at=[&](double a,double b,double s){double p[3];for(int k=0;k<3;++k)p[k]=C[k]+a*u[k]+b*w[k]+s*n[k];return sample(p[0],p[1],p[2]);};
 const double sub[2]={-cell/4,cell/4};
 const double reach=P[P_RIP]+P[P_MAXD]+offMax;std::vector<double> levels;for(double s=-reach;s<=reach+1e-6;s+=.006)levels.push_back(s);
 const std::size_t NL=levels.size();
 auto cA=[&](std::size_t q){return secO+(double(q%std::size_t(N))+.5)*cell;};auto cB=[&](std::size_t q){return secO+(double(q/std::size_t(N))+.5)*cell;};
 // where the tear may reach (the former function's allow(A, B) at the cell centres, evaluated once)
 std::vector<std::uint8_t> allowed(NN,0);
 parallel(NN,[&](std::size_t a,std::size_t b){for(std::size_t q=a;q<b;++q)allowed[q]=allow(cA(q),cB(q))?1:0;});
 {const double key[16]={C[0],C[1],C[2],n[0],n[1],n[2],u[0],u[1],u[2],w[0],w[1],w[2],secO,cell,reach,double(N)};
  bool same=S.on&&S.N==N&&S.L==NL;for(int k=0;same&&k<16;++k)same=S.key[k]==key[k];
  if(!same){S.on=true;std::copy(key,key+16,S.key);S.N=N;S.L=NL;S.bits.assign(NN,0);S.t0.assign(NN,0);S.lev.assign(NN*NL,0);S.done.assign(NN,0);}}
 // the samples (rows taken dynamically: the allowed cells lie in the middle rows)
 {std::atomic<std::size_t> nextRow{0};std::atomic<std::size_t> count{0};
  parallel(16,[&](std::size_t,std::size_t){std::size_t mine=0;
   for(std::size_t jb;(jb=nextRow.fetch_add(1,std::memory_order_relaxed))<std::size_t(N);)for(std::size_t ia=0;ia<std::size_t(N);++ia){const std::size_t q=jb*N+ia;
    if(!allowed[q]||S.done[q])continue;++mine;
    const double A=cA(q),B=cB(q);std::uint32_t m=0;bool t0v=false;
    for(int L=-4;L<=4;++L){const double s=L*.002;for(double da:sub)for(double db:sub){const unsigned lab=at(A+da,B+db,s);m|=classBit(lab);if(L==0&&(lab&63u)!=0&&(lab&63u)!=16)t0v=true;}}
    S.bits[q]=m;S.t0[q]=t0v?1:0;
    for(std::size_t li=0;li<NL;++li){bool t=false;
     for(int k=0;k<4&&!t;++k){const unsigned lb=at(A+sub[k>>1],B+sub[k&1],levels[li]);t=(lb&63u)!=0&&(lb&63u)!=16;}
     S.lev[li*NN+q]=t?1:0;}
    S.done[q]=1;}
   count+=mine;});
  S.sampled+=count.load();}
 std::vector<std::uint32_t> bits(NN,0);std::vector<std::uint8_t> tissue0(NN,0);
 for(std::size_t q=0;q<NN;++q)if(allowed[q]){bits[q]=S.bits[q];tissue0[q]=S.t0[q];}
 auto r2At=[&](std::size_t q){const double A=cA(q),B=cB(q);return A*A+B*B;};
 std::vector<int> lab;label4(tissue0,N,lab);std::vector<char> keepL(NN+1,0);
 for(std::size_t q=0;q<NN;++q)if(tissue0[q]&&r2At(q)<=.025*.025)keepL[std::size_t(lab[q])]=1;
 std::vector<std::uint8_t> limb(NN,0);for(std::size_t q=0;q<NN;++q)limb[q]=(lab[q]&&keepL[std::size_t(lab[q])]&&r2At(q)<=Rl*Rl)?1:0;
 const std::vector<std::uint8_t> limb0=limb;
 // the levels, labelled side by side; each one's growth reads limb0 only, the union below is the former loop's
 std::vector<std::uint8_t> grow(NN*NL,0);
 parallel(NL,[&](std::size_t l0,std::size_t l1){std::vector<std::uint8_t> lev(NN,0);std::vector<int> ll;
  for(std::size_t li=l0;li<l1;++li){for(std::size_t q=0;q<NN;++q)lev[q]=S.lev[li*NN+q]&&allowed[q]?1:0;
   label4(lev,N,ll);std::vector<char> kk(NN+1,0);
   for(std::size_t q=0;q<NN;++q)if(lev[q]&&limb0[q])kk[std::size_t(ll[q])]=1;
   for(std::size_t q=0;q<NN;++q)if(ll[q]&&kk[std::size_t(ll[q])]&&r2At(q)<=Rl*Rl)grow[li*NN+q]=1;}});
 for(std::size_t li=0;li<NL;++li)for(std::size_t q=0;q<NN;++q)if(grow[li*NN+q])limb[q]=1;
 for(int it=0;it<2;++it){std::vector<std::uint8_t> d=limb;for(int y=0;y<N;++y)for(int x=0;x<N;++x){const std::size_t q=std::size_t(y)*N+x;if(limb[q])continue;
  if((x>0&&limb[q-1])||(x+1<N&&limb[q+1])||(y>0&&limb[q-N])||(y+1<N&&limb[q+N]))d[q]=1;}limb.swap(d);}
 for(std::size_t q=0;q<NN;++q)if(r2At(q)>Rl*Rl||!allowed[q])limb[q]=0; // the footprint ends at the cylinder and where it may not reach
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
 std::vector<int> mcls(NN,-1);
 for(std::size_t q=0;q<NN;++q){const std::uint32_t mm=bits[q]&musBits;if(!mm)continue;if(mm&(1u<<MUSCLE)){mcls[q]=MUSCLE;continue;}for(unsigned c=MV0;c<MV0+8;++c)if(mm&(1u<<c)){mcls[q]=int(c);break;}}
 std::vector<int> nearCls=mcls;{std::vector<std::size_t> cur,nxt;for(std::size_t q=0;q<NN;++q)if(mcls[q]>=0)cur.push_back(q);
  while(!cur.empty()){nxt.clear();for(const auto q:cur){const int x=int(q%N),y=int(q/N);for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){const int X=x+dx,Y=y+dy;
    if(X<0||Y<0||X>=N||Y>=N)continue;const std::size_t j=std::size_t(Y)*N+X;if(nearCls[j]>=0)continue;nearCls[j]=nearCls[q];nxt.push_back(j);}}cur.swap(nxt);}}
 for(std::size_t q=0;q<NN;++q)if(nearCls[q]>=0)D.FOOT[q]=std::uint8_t(D.FOOT[q]|((nearCls[q]+1)<<1));
 {double sa=0,sb=0;std::size_t nb=0,nl=0;for(std::size_t q=0;q<NN;++q){if(limb0[q])++nl;if(!(bits[q]&boneBits))continue;sa+=secO+(double(q%N)+.5)*cell;sb+=secO+(double(q/N)+.5)*cell;++nb;}
  if(nb){P[P_LIMB]=sa/double(nb);P[P_LIMB+1]=sb/double(nb);P[P_LIMB+2]=std::max(.006,std::sqrt(double(nb)*cell*cell/3.141592653589793));}else{P[P_LIMB]=0;P[P_LIMB+1]=0;P[P_LIMB+2]=.012;}
  P[P_LIMB+3]=std::max(.03,std::sqrt(double(nl)*cell*cell/3.141592653589793));}
 for(std::size_t q=0;q<NN;++q){if(!limb[q])continue;++D.limbCells;const double A=secO+(double(q%N)+.5)*cell,B=secO+(double(q/N)+.5)*cell;
  D.footLo[0]=std::min(D.footLo[0],A-cell/2);D.footHi[0]=std::max(D.footHi[0],A+cell/2);D.footLo[1]=std::min(D.footLo[1],B-cell/2);D.footHi[1]=std::max(D.footHi[1],B+cell/2);}
 std::vector<std::uint32_t> near8(NN,0);
 for(int y=0;y<N;++y)for(int x=0;x<N;++x){const std::size_t q=std::size_t(y)*N+x;if(!limb[q])continue;std::uint32_t m=0;
  for(int dy=-2;dy<=2;++dy)for(int dx=-2;dx<=2;++dx){const int X=x+dx,Y=y+dy;if(X<0||Y<0||X>=N||Y>=N)continue;m|=bits[std::size_t(Y)*N+X]&musBits;}
  if(!m&&nearCls[q]>=0)m=1u<<unsigned(nearCls[q]);near8[q]=m;}
 D.sectionMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count();
 const auto t1=std::chrono::steady_clock::now();
 // ---- per-seed constants: the tear surface's offset now (the sheet reads it), the rest in finishSeeds
 const std::uint32_t seed=D.seed;auto U=[&](int k){return u01(seed,300+k,11,0x5EEDu);};const int j=P_SEEDC;
 {const double ang=2*3.141592653589793*U(0),mag=tiltMax*(.35+.65*U(1));P[j]=mag*std::cos(ang);P[j+1]=mag*std::sin(ang);
  P[j+2]=bowMax*(2*U(2)-1);P[j+3]=waveMax*(.3+.7*U(3));const double wa=2*3.141592653589793*U(4),wk=2.0+3.0*U(5);
  P[j+4]=wk*std::cos(wa);P[j+5]=wk*std::sin(wa);P[j+6]=2*3.141592653589793*U(6);}
 D.wMcls=std::move(mcls);D.wNear8=std::move(near8);D.seedsPending=true;
 D.seedMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t1).count();}
// the rest of the per-seed constants (TearDecide.h buildSection's seed_consts after the offset), same order and numbers
template<class Parallel>
inline void finishSeeds(Decision&D,const Parallel&parallel){
 if(!D.seedsPending)return;D.seedsPending=false;const auto t1=std::chrono::steady_clock::now();double*P=D.P;
 const int N=D.N;const std::size_t NN=std::size_t(N)*N;const double secO=P[P_SEC],cell=P[P_SEC+1];
 const double u[3]={P[P_U],P[P_U+1],P[P_U+2]},w[3]={P[P_W],P[P_W+1],P[P_W+2]};
 const std::vector<int>&mcls=D.wMcls;const std::vector<std::uint32_t>&near8=D.wNear8;const std::vector<std::uint8_t>&limb=D.LIMB;
 const std::uint32_t seed=D.seed;auto U=[&](int k){return u01(seed,300+k,11,0x5EEDu);};const int j=P_SEEDC;
 std::vector<std::size_t> mus;for(std::size_t q=0;q<NN;++q)if(mcls[q]>=0&&limb[q])mus.push_back(q);D.muscleCells=mus.size();
 auto rel=[&](std::size_t q,double&dx,double&dy,double&dz){const double A=secO+(double(q%N)+.5)*cell,B=secO+(double(q/N)+.5)*cell;dx=A*u[0]+B*w[0];dy=A*u[1]+B*w[1];dz=A*u[2]+B*w[2];};
 for(int side=0;side<2;++side){double m;
  if(!mus.empty()){std::vector<double> vals(mus.size());
   parallel(mus.size(),[&](std::size_t a,std::size_t b){for(std::size_t i=a;i<b;++i){double dx,dy,dz;rel(mus[i],dx,dy,dz);vals[i]=muscleEff(D,unsigned(mcls[mus[i]]),side,dx,dy,dz);}});
   std::sort(vals.begin(),vals.end());const double pos=.08*double(vals.size()-1);const std::size_t lo=std::size_t(pos);const double fr=pos-double(lo);
   m=vals[lo]+(lo+1<vals.size()?(vals[lo+1]-vals[lo])*fr:0.);}
  else m=groupFloor+.5*groupSpan;
  P[j+8+side]=m;P[j+14+side]=0;}
 parallel(NN,[&](std::size_t a,std::size_t b){for(std::size_t q=a;q<b;++q){const std::uint32_t m=near8[q];if(!m)continue;double dx,dy,dz;rel(q,dx,dy,dz);
  for(int side=0;side<2;++side){double best=0;for(unsigned c=0;c<NCLS;++c)if((m>>c)&1u)best=std::max(best,muscleEff(D,c,side,dx,dy,dz));D.EX[NN*std::size_t(1+side)+q]=float(best+shellBelow);}}});
 P[j+10]=boneStand+boneStandSpan*U(7);P[j+11]=.20+.25*U(8);{const double oa=2*3.141592653589793*U(9);P[j+12]=std::cos(oa);P[j+13]=std::sin(oa);}
 for(int k=0;k<2;++k){const int jj=P_STR+5*k;P[jj+4]=-1;if(mus.empty()||(k==1&&U(20)<.35))continue;double a_=0,b_=0;
  for(int tries=0;tries<8;++tries){const std::size_t idx=std::min(std::size_t(U(30+8*k+tries)*double(mus.size())),mus.size()-1);const std::size_t q=mus[idx];
   a_=secO+(double(q%N)+.5)*cell;b_=secO+(double(q/N)+.5)*cell;
   if(std::hypot(a_-P[P_LIMB],b_-P[P_LIMB+1])>P[P_LIMB+2]+.018&&std::hypot(a_,b_)<.55*P[P_LIMB+3])break;}
  P[jj]=a_;P[jj+1]=b_;P[jj+2]=strandR+strandRSpan*U(40+k);P[jj+3]=strandL+strandLSpan*U(42+k);P[jj+4]=U(44+k)<.5?1:0;}
 D.wMcls.clear();D.wMcls.shrink_to_fit();D.wNear8.clear();D.wNear8.shrink_to_fit();
 D.seedMs+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t1).count();}
// Box cells (lo..hi inclusive) whose distance along n from C lies in [sMin, sMax], with one cell of margin (the body tests
// exactly), clipped along the box axis on which |n| is largest; rows taken dynamically over the parallel width. make() builds
// one chunk's lookups, body(lookups, cell, chunk).
inline std::atomic<unsigned long long>&scanVisits(){static std::atomic<unsigned long long> n{0};return n;} // probe read-out
// One column's cells K (from base = the box's first cell along it) whose linear value f0 + (K - base) * df lies in [lo, hi], one
// cell of margin each way (conservative: the caller tests every cell exactly)
inline void clipColumn(double f0,double df,double lo,double hi,int base,double pitch,int&k0,int&k1){
 if(std::abs(df)>1e-12){double a=(lo-f0)/df,e=(hi-f0)/df;if(a>e)std::swap(a,e);
  k0=std::max(k0,base+int(std::floor(std::max(a,-1e9)))-1);k1=std::min(k1,base+int(std::ceil(std::min(e,1e9)))+1);}
 else if(f0<lo-pitch||f0>hi+pitch)k1=k0-1;}
// the plane's frame across (u, w) and a rectangle there the cells must lie in (aLo, aHi, bLo, bHi), or none
struct Across {Vec u,w;double r[4];};
template<class Make,class Body>
inline void slabScan(const Grid&g,const int lo[3],const int hi[3],const Vec&C,const Vec&n,double sMin,double sMax,const Make&make,const Body&body,const Across*across=nullptr){
 const double an[3]={std::abs(n.x),std::abs(n.y),std::abs(n.z)};
 static const bool oldOrder=std::getenv("GFY_MS_OLDSCAN")!=nullptr; // probe A/B knob: the former x / y / z-clipped loop order
 const int ka=oldOrder?2:(an[0]>=an[1]?(an[0]>=an[2]?0:2):(an[1]>=an[2]?1:2)),ia=ka==0?1:0,ja=ka==2?1:2;
 if(hi[ia]<lo[ia]||hi[ja]<lo[ja]||hi[ka]<lo[ka])return;
 const int di=hi[ia]-lo[ia]+1;const double ds=n[ka]*g.pitch;std::atomic<int> next{0};std::atomic<unsigned long long> visits{0};
 core::parallelChunks(core::parallelWidth(),1,[&](std::size_t,std::size_t,std::size_t chunk){auto lk=make();unsigned long long mine=0;
  for(int r;(r=next.fetch_add(1,std::memory_order_relaxed))<di;){const int I=lo[ia]+r;
   for(int J=lo[ja];J<=hi[ja];++J){Cell b{0,0,0};(&b.x)[ia]=I;(&b.x)[ja]=J;(&b.x)[ka]=lo[ka];const Vec d0=g.center(b)-C;
    int k0=lo[ka],k1=hi[ka];clipColumn(dot(d0,n),ds,sMin,sMax,lo[ka],g.pitch,k0,k1);
    if(across&&k0<=k1){clipColumn(dot(d0,across->u),across->u[ka]*g.pitch,across->r[0],across->r[1],lo[ka],g.pitch,k0,k1);
     clipColumn(dot(d0,across->w),across->w[ka]*g.pitch,across->r[2],across->r[3],lo[ka],g.pitch,k0,k1);}
    if(k0>k1)continue;
    mine+=std::size_t(k1-k0+1);
    for(int K=k0;K<=k1;++K){Cell c=b;(&c.x)[ka]=K;body(lk,c,chunk);}}}
  visits+=mine;});
 scanVisits()+=visits.load();}
inline bool cellLess(const Cell&a,const Cell&b){return a.x!=b.x?a.x<b.x:(a.y!=b.y?a.y<b.y:a.z<b.z);}
// The proof's band in slab-aligned columns: every column (I, J) of the box's two minor axes holds the cells k0 .. k0 + len - 1 along
// the normal's largest axis (slabScan's clip: every cell of the box within [sMin, sMax] along the normal, one cell of margin);
// index = column * KL + (K - k0). A cell outside its column's range lies outside the slab (the former arrays spanned the box).
struct SlabColumns {
 static constexpr std::size_t npos=~std::size_t(0);
 int ka=2,ia=0,ja=1,lo[3]{},hi[3]{},nI=0,nJ=0,KL=0;std::vector<int> k0,len;
 std::size_t size()const{return std::size_t(nI)*std::size_t(nJ)*std::size_t(KL);}
 std::size_t index(int I,int J,int K)const{if(I<lo[ia]||I>hi[ia]||J<lo[ja]||J>hi[ja])return npos;const std::size_t col=std::size_t(I-lo[ia])*std::size_t(nJ)+std::size_t(J-lo[ja]);
  const int kk=K-k0[col];if(kk<0||kk>=len[col])return npos;return col*std::size_t(KL)+std::size_t(kk);}
 std::size_t index(const Cell&c)const{return index((&c.x)[ia],(&c.x)[ja],(&c.x)[ka]);}
 Cell cell(std::size_t idx)const{const std::size_t col=idx/std::size_t(KL);const int kk=int(idx%std::size_t(KL));Cell c{0,0,0};
  (&c.x)[ia]=lo[ia]+int(col/std::size_t(nJ));(&c.x)[ja]=lo[ja]+int(col%std::size_t(nJ));(&c.x)[ka]=k0[col]+kk;return c;}
};
inline SlabColumns slabColumns(const Grid&g,const int lo[3],const int hi[3],const Vec&C,const Vec&n,double sMin,double sMax,const Across*across=nullptr){
 SlabColumns S;const double an[3]={std::abs(n.x),std::abs(n.y),std::abs(n.z)};
 S.ka=an[0]>=an[1]?(an[0]>=an[2]?0:2):(an[1]>=an[2]?1:2);S.ia=S.ka==0?1:0;S.ja=S.ka==2?1:2;
 for(int k=0;k<3;++k){S.lo[k]=lo[k];S.hi[k]=hi[k];}
 if(hi[0]<lo[0]||hi[1]<lo[1]||hi[2]<lo[2])return S;
 S.nI=hi[S.ia]-lo[S.ia]+1;S.nJ=hi[S.ja]-lo[S.ja]+1;S.k0.assign(std::size_t(S.nI)*S.nJ,0);S.len.assign(std::size_t(S.nI)*S.nJ,0);
 const double ds=n[S.ka]*g.pitch;int KL=0;
 for(int I=0;I<S.nI;++I)for(int J=0;J<S.nJ;++J){Cell b{0,0,0};(&b.x)[S.ia]=lo[S.ia]+I;(&b.x)[S.ja]=lo[S.ja]+J;(&b.x)[S.ka]=lo[S.ka];const Vec d0=g.center(b)-C;
  int k0=lo[S.ka],k1=hi[S.ka];clipColumn(dot(d0,n),ds,sMin,sMax,lo[S.ka],g.pitch,k0,k1);
  if(across&&k0<=k1){clipColumn(dot(d0,across->u),across->u[S.ka]*g.pitch,across->r[0],across->r[1],lo[S.ka],g.pitch,k0,k1);
   clipColumn(dot(d0,across->w),across->w[S.ka]*g.pitch,across->r[2],across->r[3],lo[S.ka],g.pitch,k0,k1);}
  const std::size_t col=std::size_t(I)*S.nJ+std::size_t(J);S.k0[col]=k0;S.len[col]=std::max(0,k1-k0+1);KL=std::max(KL,S.len[col]);}
 S.KL=std::max(KL,1);return S;}
// every cell of the columns, rows (I) taken dynamically: body(lookups, cell, index, chunk)
template<class Make,class Body>
inline void slabColumnsScan(const SlabColumns&S,const Make&make,const Body&body){
 if(!S.nI||!S.nJ)return;std::atomic<int> next{0};std::atomic<unsigned long long> visits{0};
 core::parallelChunks(core::parallelWidth(),1,[&](std::size_t,std::size_t,std::size_t chunk){auto lk=make();unsigned long long mine=0;
  for(int I;(I=next.fetch_add(1,std::memory_order_relaxed))<S.nI;)for(int J=0;J<S.nJ;++J){const std::size_t col=std::size_t(I)*S.nJ+std::size_t(J);const int k0=S.k0[col],L=S.len[col];
   Cell c{0,0,0};(&c.x)[S.ia]=S.lo[S.ia]+I;(&c.x)[S.ja]=S.lo[S.ja]+J;mine+=std::size_t(L);
   for(int kk=0;kk<L;++kk){(&c.x)[S.ka]=k0+kk;body(lk,c,col*std::size_t(S.KL)+std::size_t(kk),chunk);}}
  visits+=mine;});
 scanVisits()+=visits.load();}
// The bake of TearDecide.h bakeTear (bakeBox, version 2) cut into chunks for TearBakeAsync.h: the sizes, arrays and header made
// here (before the sever returns), the map's rows and the field's slices filled by the chunks with bakeBox's own per-cell code.
// done(stats, ms) runs once after the last chunk (stats: sizes, evaluated cells).
template<class Sample>
inline std::shared_ptr<tear_bake::Pending> bakeTearChunks(const std::shared_ptr<const Decision>&Dp,double h3,double h2,const Sample&sample,
    const std::shared_ptr<gfy_packed::TearBlock>&Bp,std::function<void(const Bake2Stats&,double)> done){
 const Decision&D=*Dp;gfy_packed::TearBlock&B=*Bp;const double*P=D.P;
 const double reach=D.P[P_RIP]+D.P[P_MAXD]+footBelow+offMax+2*D.pitch;
 const BakeBox box{D.footLo[0]-2*D.pitch,D.footLo[1]-2*D.pitch,-reach,D.footHi[0]+2*D.pitch,D.footHi[1]+2*D.pitch,reach};
 const int mx=std::max(1,int(std::ceil((box.a1-box.a0)/h2))),my=std::max(1,int(std::ceil((box.b1-box.b0)/h2)));
 B.map_dim[0]=std::uint32_t(mx);B.map_dim[1]=std::uint32_t(my);B.map_origin[0]=box.a0;B.map_origin[1]=box.b0;B.map_pitch=h2;
 B.map.assign(std::size_t(mx)*my*MAP_CH,0.f);
 const int nx=std::max(1,int(std::ceil((box.a1-box.a0)/h3))),ny=std::max(1,int(std::ceil((box.b1-box.b0)/h3))),nz=std::max(1,int(std::ceil((box.s1-box.s0)/h3)));
 B.bake_dim[0]=std::uint32_t(nx);B.bake_dim[1]=std::uint32_t(ny);B.bake_dim[2]=std::uint32_t(nz);B.bake_origin[0]=box.a0;B.bake_origin[1]=box.b0;B.bake_origin[2]=box.s0;B.bake_pitch=h3;
 B.bake.assign(std::size_t(nx)*ny*nz,255);
 B.header.assign(4*26,0u);auto F=[&](int i,double v){const float f=float(v);std::memcpy(&B.header[std::size_t(i)],&f,4);};
 for(int k=0;k<3;++k){F(k,P[P_C+k]);F(4+k,P[P_U+k]);F(8+k,P[P_W+k]);F(12+k,P[P_N+k]);}
 F(3,h3);F(7,box.a0);F(11,box.b0);F(15,box.s0);B.header[16]=std::uint32_t(nx);B.header[17]=std::uint32_t(ny);B.header[18]=std::uint32_t(nz);
 F(20,box.a0);F(21,box.b0);F(22,h2);F(23,P[P_RIP]);B.header[24]=std::uint32_t(mx);B.header[25]=std::uint32_t(my);B.header[26]=D.seed;
 F(28,P[P_H]);F(29,P[P_RL]);F(30,P[P_DSCALE]);
 for(int k=0;k<9;++k){const unsigned c=k==0?MUSCLE:MV0+unsigned(k-1);const int j=P_FE+8*int(c);for(int a=0;a<3;++a){F(4*(8+2*k)+a,P[j+a]);F(4*(9+2*k)+a,P[j+3+a]);}}
 auto p=std::make_shared<tear_bake::Pending>();auto evaluated=std::make_shared<std::atomic<std::size_t>>(0);
 // the map: rows (bakeBox's map loop body)
 const int rowStep=std::max(1,my/8);
 for(int y0=0;y0<my;y0+=rowStep){const int y1=std::min(my,y0+rowStep);
  p->chunks.push_back([Dp,Bp,box,h2,mx,y0,y1]{const Decision&D=*Dp;gfy_packed::TearBlock&B=*Bp;const double*P=D.P;
   for(std::size_t y=std::size_t(y0);y<std::size_t(y1);++y)for(int x=0;x<mx;++x){
    const double a=box.a0+(x+.5)*h2,b=box.b0+(double(y)+.5)*h2;float*o=&B.map[(y*std::size_t(mx)+std::size_t(x))*MAP_CH];
    const bool in=a*a+b*b<=P[P_RL]*P[P_RL]&&D.drawn(std::size_t(D.cellIndex(a,b)));o[0]=float(D.offset(a,b)); // damagecut: a bridge tear's region is its bridge
    o[1]=float((in?1:0)|(in&&(D.FOOT[std::size_t(D.cellIndex(a,b))]&1u)?2:0)|(D.bridge&&a*a+b*b<=P[P_RL]*P[P_RL]&&(D.sectionBridge?D.BRIDGE:D.LIMB)[std::size_t(D.cellIndex(a,b))]?4:0)); // 4: the footprint
    for(int k=0;k<9;++k)for(int side=0;side<2;++side)o[2+2*k+side]=float(flatE(D,k,side,a,b));}});}
 // the field: slices (bakeBox's field loop body)
 const double inner=P[P_RIP]+.0015,outer=P[P_RIP]+P[P_MAXD]+.004;
 for(int z0=0;z0<nz;z0+=2){const int z1=std::min(nz,z0+2);
  p->chunks.push_back([Dp,Bp,box,h3,nx,ny,z0,z1,inner,outer,sample,evaluated]{const Decision&D=*Dp;gfy_packed::TearBlock&B=*Bp;const double*P=D.P;std::size_t ev=0;
   for(std::size_t z=std::size_t(z0);z<std::size_t(z1);++z)for(int y=0;y<ny;++y)for(int x=0;x<nx;++x){
    const double a=box.a0+(x+.5)*h3,b=box.b0+(y+.5)*h3,s=box.s0+(double(z)+.5)*h3;
    if(a*a+b*b>P[P_RL]*P[P_RL])continue;const int q=D.cellIndex(a,b);if(!D.drawn(std::size_t(q)))continue; // damagecut: the region
    const double sp=s-D.offset(a,b);if(std::abs(sp)<inner-h3||std::abs(sp)>outer+h3)continue;
    double E=0;if(!bakeMuscle(D,sample,h3,a,b,s,q,sp>=0?1:0,E))continue;++ev; // cutends v2: a cell centred off the muscle takes its groups' deepest
    B.bake[(z*std::size_t(ny)+std::size_t(y))*std::size_t(nx)+std::size_t(x)]=std::uint8_t(std::min(254,std::max(0,int(std::lround(E/.00032)))));}
   *evaluated+=ev;});}
 Bake2Stats st;st.mapBytes=B.map.size()*4;st.fieldBytes=B.bake.size();
 p->finish=[st,evaluated,done](double ms){Bake2Stats s=st;s.fieldEvaluated=evaluated->load();s.ms=ms;if(done)done(s,ms);};
 return p;}
// the proof's arrays, kept by each thread (no fresh pages after its first tear; released when one held more than 64 MB)
struct ProofScratch {std::vector<signed char> side;std::vector<std::uint8_t> sheet;std::vector<std::uint64_t> seen;
 void trim(){if(side.capacity()+sheet.capacity()+8*seen.capacity()>(std::size_t(64)<<20)){std::vector<signed char>().swap(side);std::vector<std::uint8_t>().swap(sheet);std::vector<std::uint64_t>().swap(seen);}}};
inline ProofScratch&proofScratch(){thread_local ProofScratch s;return s;}
} // namespace tear_decide
