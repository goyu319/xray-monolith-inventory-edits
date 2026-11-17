#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <vector>
#include "TearBakeAsync.h" // speed step: a tear's bake may still be running beside the separation
namespace tear_atlas {
struct Stats {std::size_t tears{},bytes{},indexCells{},overlap3{},listEntries{},maxList{},pruned{},sets{};double ms{};};
struct Box {double lo[3]{1e30,1e30,1e30},hi[3]{-1e30,-1e30,-1e30};
 bool valid()const{return lo[0]<=hi[0]&&lo[1]<=hi[1]&&lo[2]<=hi[2];}
 void add(const double*p){for(int a=0;a<3;++a){lo[a]=std::min(lo[a],p[a]);hi[a]=std::max(hi[a],p[a]);}}
 void add(const Box&b){if(!b.valid())return;add(b.lo);add(b.hi);}
 bool meets(const Box&b,double pad=0)const{for(int a=0;a<3;++a)if(lo[a]>b.hi[a]+pad||hi[a]<b.lo[a]-pad)return false;return true;}
 Box padded(double d)const{Box b=*this;for(int a=0;a<3;++a){b.lo[a]-=d;b.hi[a]+=d;}return b;}};
inline float f32(const std::vector<std::uint32_t>&h,std::size_t i){float v;std::memcpy(&v,&h[i],4);return v;}
// headfix: a head burst's record (HeadBurst.h MAGIC in params word 29; a decide-once tear writes 0 there): kept with side 3 (its
// header holds the piece's groups; its sites from word 44 on, clear of the words written below), no torn zone (left out of the
// index and the plane tests), named in row 1 texel 1 (x = id + 1) for painter_vmask_head.h
inline bool burstRecord(const gfy_packed::TearBlock&b){return b.params.size()>29&&b.params[29]==0x50415254u;}
inline bool usable(const gfy_packed::TearBlock&b){
 return b.header.size()==4*26&&b.map_dim[0]&&b.map_dim[1]&&b.map.size()==std::size_t(b.map_dim[0])*b.map_dim[1]*20&&
  b.bake.size()==std::size_t(b.bake_dim[0])*b.bake_dim[1]*b.bake_dim[2]&&b.bake_dim[0]<=2048&&b.bake_dim[1]<=2048&&b.bake_dim[2]<=2048&&5*b.map_dim[0]<=16384;}
// One tear's torn zone in its own frame (the numbers the programs read): s0 = plane distance, (pa, pb) across
struct Zone {double C[3]{},n[3]{},u[3]{},w[3]{},Rl=0,H=0,Z=0,sLo=0,sHi=0,a0=0,a1=0,b0=0,b1=0,eMax=0,scale=1,slab=0,a2=0,b2=0,h2=0;std::uint32_t mx=0,my=0;
 Box box,region;bool ok=false;}; // box: the torn zone; region: the slab |s0| <= H + 24 mm over the footprint (the tear says anything only there)
// scale: this piece's depth factor for the tear (depthScale below); 1 = the baked depths
inline Zone zone(const gfy_packed::TearBlock&b,double scale=1){Zone z;if(!usable(b))return z;const auto&h=b.header;z.scale=std::min(std::max(scale,0.),1.);
 for(int a=0;a<3;++a){z.C[a]=f32(h,a);z.u[a]=f32(h,4+a);z.w[a]=f32(h,8+a);z.n[a]=f32(h,12+a);}
 const double a2=f32(h,20),b2=f32(h,21),h2=f32(h,22),ripR=f32(h,23),ds=f32(h,30);z.H=f32(h,28);z.Rl=f32(h,29);
 const std::uint32_t mx=b.map_dim[0],my=b.map_dim[1];
 // the programs: m = max(0, |s - offset| - ripR) - E. Every baked E (map and field, TearDecide.h flatE / muscleE) is eased to the
 // decision's deepest tear maxD and scaled by ds <= 1; H = max(half, maxD + 30 mm) >= maxD + 30 mm; the fray / crumbs add at most
 // 1.5 mm * ds; |offset| <= offMax (24 mm). A margin >= 4 mm changes no result (the own margin starts at 4 mm); beyond the region
 // slab (|s0| > H + 24 mm) the tear says nothing. No scan of the baked data: the bound holds by construction.
 const double eMax=std::max(0.,z.H-.03),oLo=-tear_decide::offMax,oHi=tear_decide::offMax;
 // cutfix: the rip gap scales with the depths (a thin piece keeps its middle); the muscle / bone floor (2 mm) and the fray on top
 z.eMax=eMax;z.Z=(ripR+eMax)*z.scale+.002+.0015*std::max(ds,1.)+.004+1e-4;const double slab=z.H+.024;
 z.sLo=std::max(oLo-z.Z,-slab);z.sHi=std::min(oHi+z.Z,slab);
 z.a0=std::max(a2,-z.Rl);z.a1=std::min(a2+mx*h2,z.Rl);z.b0=std::max(b2,-z.Rl);z.b1=std::min(b2+my*h2,z.Rl);z.slab=slab;z.a2=a2;z.b2=b2;z.h2=h2;z.mx=mx;z.my=my;
 if(z.a0>z.a1||z.b0>z.b1||z.sLo>z.sHi)return z;
 for(int q=0;q<8;++q){const double A=(q&1)?z.a1:z.a0,B=(q&2)?z.b1:z.b0,S=(q&4)?z.sHi:z.sLo,S2=(q&4)?slab:-slab;double p[3],r[3];
  for(int a=0;a<3;++a){p[a]=z.C[a]+A*z.u[a]+B*z.w[a]+S*z.n[a];r[a]=z.C[a]+A*z.u[a]+B*z.w[a]+S2*z.n[a];}z.box.add(p);z.region.add(r);}
 z.ok=true;return z;}
// does the zone meet the axis-aligned cell [lo, lo + d)? (exact for the frame box: separating axes of the two boxes)
inline bool zoneMeets(const Zone&z,const double*lo,double d){
 const double c[3]={lo[0]+.5*d,lo[1]+.5*d,lo[2]+.5*d},r=.5*d;
 const double ax[3][3]={{z.u[0],z.u[1],z.u[2]},{z.w[0],z.w[1],z.w[2]},{z.n[0],z.n[1],z.n[2]}};
 const double mid[3]={.5*(z.a0+z.a1),.5*(z.b0+z.b1),.5*(z.sLo+z.sHi)},half[3]={.5*(z.a1-z.a0),.5*(z.b1-z.b0),.5*(z.sHi-z.sLo)};
 double cen[3];for(int a=0;a<3;++a)cen[a]=z.C[a]+mid[0]*ax[0][a]+mid[1]*ax[1][a]+mid[2]*ax[2][a];
 const double t[3]={c[0]-cen[0],c[1]-cen[1],c[2]-cen[2]};
 for(int i=0;i<3;++i){const double proj=std::abs(t[i]),rz=half[0]*std::abs(ax[0][i])+half[1]*std::abs(ax[1][i])+half[2]*std::abs(ax[2][i]);if(proj>r+rz)return false;}
 for(int i=0;i<3;++i){const double proj=std::abs(t[0]*ax[i][0]+t[1]*ax[i][1]+t[2]*ax[i][2]),rc=r*(std::abs(ax[i][0])+std::abs(ax[i][1])+std::abs(ax[i][2]));if(proj>rc+half[i])return false;}
 for(int i=0;i<3;++i)for(int j=0;j<3;++j){const double e[3]={i==0?1.:0.,i==1?1.:0.,i==2?1.:0.};
  const double L[3]={e[1]*ax[j][2]-e[2]*ax[j][1],e[2]*ax[j][0]-e[0]*ax[j][2],e[0]*ax[j][1]-e[1]*ax[j][0]};const double l2=L[0]*L[0]+L[1]*L[1]+L[2]*L[2];if(l2<1e-12)continue;
  const double proj=std::abs(t[0]*L[0]+t[1]*L[1]+t[2]*L[2]),rc=r*(std::abs(L[0])+std::abs(L[1])+std::abs(L[2]));
  double rz=0;for(int k=0;k<3;++k)rz+=half[k]*std::abs(ax[k][0]*L[0]+ax[k][1]*L[1]+ax[k][2]*L[2]);if(proj>rc+rz)return false;}
 return true;}
// The set of a piece. own: the box of the piece's own cells (rest space; the programs search the volume only inside it, padded by
// 2 cells); draw: the box of everything the piece draws (its cells and its exterior triangles). A tear whose zone misses `draw` is
// left out. Returns null only for an empty set.
// a point inside the tear's region (painter_vt_value's test: the slab, the disk, the map's footprint flag); s0 out = its plane distance
inline bool inRegion(const Zone&z,const gfy_packed::TearBlock&b,const double*p,double&s0){const double d[3]={p[0]-z.C[0],p[1]-z.C[1],p[2]-z.C[2]};
 s0=d[0]*z.n[0]+d[1]*z.n[1]+d[2]*z.n[2];const double pa=d[0]*z.u[0]+d[1]*z.u[1]+d[2]*z.u[2],pb=d[0]*z.w[0]+d[1]*z.w[1]+d[2]*z.w[2];
 if(std::abs(s0)>z.slab||pa*pa+pb*pb>z.Rl*z.Rl)return false;
 const int i=std::min(std::max(int(std::floor((pa-z.a2)/z.h2)),0),int(z.mx)-1),j=std::min(std::max(int(std::floor((pb-z.b2)/z.h2)),0),int(z.my)-1);
 return (int(b.map[(std::size_t(j)*z.mx+std::size_t(i))*20+1]+.5f)&1)!=0;}
// a point inside the tear's torn zone (|s0 - offset| can reach Z, inside the region; `margin` widens the zone along the normal and
// narrows the disk): the tear decides a cell there on its own (its torn face); everywhere else what another piece holds is marked
inline bool inZone(const Zone&z,const gfy_packed::TearBlock&b,const double*p,double margin){const double d[3]={p[0]-z.C[0],p[1]-z.C[1],p[2]-z.C[2]};
 const double s0=d[0]*z.n[0]+d[1]*z.n[1]+d[2]*z.n[2];if(s0<z.sLo-margin||s0>z.sHi+margin||std::abs(s0)>z.slab)return false;
 const double pa=d[0]*z.u[0]+d[1]*z.u[1]+d[2]*z.u[2],pb=d[0]*z.w[0]+d[1]*z.w[1]+d[2]*z.w[2],r=std::max(0.,z.Rl-margin);if(pa*pa+pb*pb>r*r)return false;
 const int i=std::min(std::max(int(std::floor((pa-z.a2)/z.h2)),0),int(z.mx)-1),j=std::min(std::max(int(std::floor((pb-z.b2)/z.h2)),0),int(z.my)-1);
 return (int(b.map[(std::size_t(j)*z.mx+std::size_t(i))*20+1]+.5f)&1)!=0;}
// the whole cell [lo, lo + d) on the tear's far side (the side this piece does not hold), beyond its torn zone, inside its region (at
// all 8 corners): every point of it is hidden by this tear alone
inline bool covers(const Zone&z,const gfy_packed::TearBlock&b,const double*lo,double d,unsigned side){if(side>1)return false;
 for(int q=0;q<8;++q){const double p[3]={lo[0]+((q&1)?d:0.),lo[1]+((q&2)?d:0.),lo[2]+((q&4)?d:0.)};double s0;
  if(!inRegion(z,b,p,s0))return false;if(side==0?!(s0>z.sHi):!(s0<z.sLo))return false;}
 return true;}
// cutfix: a torn end never tears more than half of the piece's own thickness along the tear's normal (the rip gap and the depths
// together: ripR + the deepest E); a piece at least twice that thick (about 15 cm for a limb tear) is unchanged: thickness = the
// piece's own cells' farthest plane distance on its side (MaskPieces3.h). A torn end FACING another torn end of the piece (a slice
// between two cuts) is limited further per pixel (painter_vmask_tear.h: 30 % of the local distance between the two torn surfaces,
// its partner below), so the middle of a slice stays whole: skin around it, meat across it.
inline double depthScale(const gfy_packed::TearBlock&b,double thickness){const Zone z=zone(b);if(!z.ok||!(thickness>0))return 1.;
 const double ripR=f32(b.header,23);return std::min(1.,.5*thickness/std::max(1e-6,ripR+z.eMax));}
// the partner of tear k in a set: the nearest other tear of the piece whose kept side FACES k's (the two kept directions within 45
// degrees of opposite) in front of it (its centre on k's kept side), across the same section (centres within the larger disk), near
// enough to matter (30 % of the gap can be less than k's deepest tear, with 48 mm for the two surfaces' offsets); -1 none
inline int partnerOf(const std::vector<const gfy_packed::TearRef*>&use,const std::vector<Zone>&Zs,std::size_t k){
 if(use[k]->side>1)return -1;const auto&a=Zs[k];const double sa=use[k]->side?1.:-1.,da[3]={sa*a.n[0],sa*a.n[1],sa*a.n[2]};
 const double ripR=f32(use[k]->block->header,23),reach=((ripR+a.eMax)*a.scale)/.3+.048;int best=-1;double bestL=1e30;
 for(std::size_t j=0;j<use.size();++j){if(j==k||use[j]->side>1)continue;const auto&b=Zs[j];const double sb=use[j]->side?1.:-1.;
  const double dot=da[0]*sb*b.n[0]+da[1]*sb*b.n[1]+da[2]*sb*b.n[2];if(dot>-.7)continue;
  const double c[3]={b.C[0]-a.C[0],b.C[1]-a.C[1],b.C[2]-a.C[2]},L=c[0]*da[0]+c[1]*da[1]+c[2]*da[2];if(L<=-.048||L>reach)continue;
  const double q[3]={c[0]-L*da[0],c[1]-L*da[1],c[2]-L*da[2]};if(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]>std::pow(std::max(a.Rl,b.Rl),2))continue;
  if(L<bestL){bestL=L;best=int(j);}}
 return best;}
// thickness: per entry of the tears the piece's thickness on its side (null or <= 0: unknown, the baked depths)
// gearback: exterior = the box an EXTERIOR pixel's ray searches (row 0 texels 4, 5: the box of everything the piece draws, its
// gear with its filled inside too). None given: the texels stay 0 and the programs use `own`, as for a set made before this.
inline std::shared_ptr<const gfy_packed::TearAtlas> build(const std::vector<gfy_packed::TearRef>&tears,const Box&own,const Box&draw,Stats*st=nullptr,
    const std::vector<double>*thickness=nullptr,const Box*exterior=nullptr){
 const auto t0=std::chrono::steady_clock::now();
 for(const auto&t:tears)if(t.block)tear_bake::await(t.block.get()); // speed step: the bakes these tears started (TearBakeAsync.h)
 std::vector<const gfy_packed::TearRef*> use;std::vector<Zone> Zs;std::size_t pruned=0;
 for(std::size_t ti=0;ti<tears.size();++ti){const auto&t=tears[ti];
  if(t.block&&t.side==3&&burstRecord(*t.block)&&usable(*t.block)){if(use.size()<32767){use.push_back(&t);Zs.push_back(Zone{});}continue;} // headfix: a burst record (no zone)
  if(!t.block||t.side>2||!usable(*t.block))continue;
  const double th=thickness&&ti<thickness->size()?(*thickness)[ti]:-1.;Zone z=zone(*t.block,th>0?depthScale(*t.block,th):1.);if(!z.ok)continue;
  if(draw.valid()&&!z.box.meets(draw)){++pruned;continue;}
  if(use.size()>=32767){++pruned;continue;} // 15-bit tear ids: 32767 tears on ONE piece is far beyond memory (~200 KB each)
  use.push_back(&t);Zs.push_back(z);}
 if(use.empty())return nullptr;
 auto A=std::make_shared<gfy_packed::TearAtlas>();const std::uint32_t K=std::uint32_t(use.size());A->K=K;
 // ---- map atlas: shelves of at most 16384 texels
 std::vector<std::uint32_t> mapX(K),mapY(K);std::uint32_t mw=1,mh=0;
 {std::uint32_t x=0,shelf=0,shelfH=0;
  for(std::uint32_t k=0;k<K;++k){const auto&b=*use[k]->block;const std::uint32_t w=5*b.map_dim[0],h=b.map_dim[1];
   if(x+w>16384){shelf+=shelfH;x=0;shelfH=0;}mapX[k]=x;mapY[k]=shelf;x+=w;shelfH=std::max(shelfH,h);mw=std::max(mw,x);}
  mh=shelf+shelfH;}
 // ---- field atlas: bricks in stacks (z <= 2048); stacks side by side in x, then rows in y (each <= 2048)
 std::uint32_t bw=1,bh=1,bd=1;std::uint64_t sumD=0;
 for(std::uint32_t k=0;k<K;++k){const auto&b=*use[k]->block;bw=std::max(bw,b.bake_dim[0]);bh=std::max(bh,b.bake_dim[1]);bd=std::max(bd,b.bake_dim[2]);sumD+=b.bake_dim[2];}
 const std::uint64_t stacks0=std::max<std::uint64_t>(1,(sumD+2047)/2048);
 const std::uint32_t D=std::uint32_t(std::min<std::uint64_t>(2048,std::max<std::uint64_t>(bd,(sumD+stacks0-1)/stacks0+bd)));
 std::vector<std::uint32_t> fx(K),fy(K),fz(K);std::uint32_t stacks=1;
 {std::uint32_t s=0,z=0;for(std::uint32_t k=0;k<K;++k){const auto nz=use[k]->block->bake_dim[2];if(z+nz>D){++s;z=0;}fz[k]=z;z+=nz;fx[k]=s;}stacks=s+1;}
 const std::uint32_t perRow=std::max<std::uint32_t>(1,std::min<std::uint32_t>(stacks,2048/bw)),stackRows=(stacks+perRow-1)/perRow;
 for(std::uint32_t k=0;k<K;++k){const auto s=fx[k];fx[k]=(s%perRow)*bw;fy[k]=(s/perRow)*bh;}
 const std::uint32_t FW=perRow*bw,FH=stackRows*bh,FD=D;
 bool fits=mh<=16384&&FW<=2048&&FH<=2048;
 if(!fits){A->K=0;return nullptr;} // unreachable for any set D3D11 memory could hold (see the header)
 // cutfix (packing by reference): no tear's map or field is copied here: the set lists each tear's block and its place in the two
 // textures (map column, map row, field x, y, slice); the renderer uploads every block into its place (RuntimeRequiredFields.h
 // PainterTearGpu). A separation's cost no longer grows with the tears a piece holds.
 A->mapW=mw;A->mapH=std::max(mh,1u);A->fieldDim[0]=FW;A->fieldDim[1]=FH;A->fieldDim[2]=FD;std::size_t blockBytes=0;
 A->blocks.reserve(K);A->place.reserve(K);
 for(std::uint32_t k=0;k<K;++k){A->blocks.push_back(use[k]->block);A->place.push_back({mapX[k],mapY[k],fx[k],fy[k],fz[k]});
  blockBytes+=use[k]->block->map.size()*4+use[k]->block->bake.size();}
 // ---- the index: the union of the regions (inside `draw`), cells of 4 cm or more (at most 262144 cells). A cell lists every tear
 // whose torn zone meets it; a cell some tear covers whole (covers()) lists that tear alone, flagged (bit 15): hidden at once
 Box frame;for(const auto&z:Zs)frame.add(z.region);
 if(draw.valid())for(int a=0;a<3;++a){frame.lo[a]=std::max(frame.lo[a],draw.lo[a]-.01);frame.hi[a]=std::min(frame.hi[a],draw.hi[a]+.01);}
 if(!frame.valid())frame=Zs[0].region;
 if(!frame.valid())for(int a=0;a<3;++a){frame.lo[a]=0;frame.hi[a]=0;} // headfix: burst records only: a 1-cell index, no entry
 double cell=.04;std::uint32_t d[3];
 for(;;){for(int a=0;a<3;++a)d[a]=std::max(1u,std::uint32_t(std::ceil((frame.hi[a]-frame.lo[a])/cell)));if(std::size_t(d[0])*d[1]*d[2]<=262144)break;cell*=1.25;}
 const std::size_t NC=std::size_t(d[0])*d[1]*d[2];std::vector<std::vector<std::uint16_t>> lists(NC);std::vector<std::int32_t> cover(NC,-1);
 for(std::uint32_t k=0;k<K;++k){const auto&z=Zs[k];int i0[3],i1[3];bool any=true;if(!z.ok)continue; // headfix: a burst record
  for(int a=0;a<3;++a){i0[a]=std::max(0,int(std::floor((z.region.lo[a]-frame.lo[a])/cell)));i1[a]=std::min(int(d[a])-1,int(std::floor((z.region.hi[a]-frame.lo[a])/cell)));any=any&&i0[a]<=i1[a];}
  if(!any)continue;
  for(int iz=i0[2];iz<=i1[2];++iz)for(int iy=i0[1];iy<=i1[1];++iy)for(int ix=i0[0];ix<=i1[0];++ix){
   const double lo[3]={frame.lo[0]+ix*cell,frame.lo[1]+iy*cell,frame.lo[2]+iz*cell};
   const std::size_t c=(std::size_t(iz)*d[1]+iy)*d[0]+ix;
   if(zoneMeets(z,lo,cell))lists[c].push_back(std::uint16_t(k));
   else if(use[k]->side<2){if(cover[c]<0&&covers(z,*use[k]->block,lo,cell,use[k]->side))cover[c]=std::int32_t(k);
    else{Zone far=z;if(use[k]->side==0){far.sLo=z.sHi;far.sHi=z.slab;}else{far.sLo=-z.slab;far.sHi=z.sLo;} // the far side beyond the zone, in part
     if(far.sLo<far.sHi&&zoneMeets(far,lo,cell))lists[c].push_back(std::uint16_t(k));}}}}
 for(std::size_t c=0;c<NC;++c)if(cover[c]>=0)lists[c].assign(1,std::uint16_t(0x8000u|unsigned(cover[c])));
 std::size_t entries=0,maxList=0,over3=0;for(const auto&l:lists){entries+=std::min<std::size_t>(l.size(),1023);maxList=std::max(maxList,l.size());over3+=l.size()>2;}
 // ---- params rows: frame, headers, plane table, cell table, lists
 const std::size_t planeTexels=2*std::size_t(K),tableTexels=(NC+3)/4,listTexels=(entries+7)/8;
 const std::size_t planeRow=2+K,tableRow=planeRow+(planeTexels+25)/26,listRow=tableRow+(tableTexels+25)/26,rows=listRow+std::max<std::size_t>(1,(listTexels+25)/26);
 if(rows>16384||entries>=(1u<<22)){A->K=0;return nullptr;} // unreachable: 262144 cells x 1023 tears would be needed
 A->paramsRows=std::uint32_t(rows);A->params.assign(rows*104,0u);auto P=[&](std::size_t row,std::size_t word){return &A->params[row*104+word];};
 auto F=[&](std::size_t row,std::size_t word,double v){const float f=float(v);std::memcpy(P(row,word),&f,4);};
 for(int a=0;a<3;++a){F(0,a,frame.lo[a]);F(0,4+a,own.valid()?own.lo[a]:-1e9);F(0,8+a,own.valid()?own.hi[a]:1e9);}F(0,3,cell);
 P(0,12)[0]=std::uint32_t(tableRow);P(0,13)[0]=std::uint32_t(listRow);P(0,14)[0]=std::uint32_t(planeRow);P(0,15)[0]=std::uint32_t(NC);
 if(exterior&&exterior->valid())for(int a=0;a<3;++a){F(0,16+a,exterior->lo[a]);F(0,20+a,exterior->hi[a]);} // gearback: the exterior rays' box
 P(1,0)[0]=d[0];P(1,1)[0]=d[1];P(1,2)[0]=d[2];P(1,3)[0]=K;
 for(std::uint32_t k=0;k<K;++k)if(!Zs[k].ok){P(1,4)[0]=k+1;break;} // headfix: row 1 texel 1 x = the burst record's id + 1
 for(std::uint32_t k=0;k<K;++k){std::memcpy(P(2+k,0),use[k]->block->header.data(),104*4);P(2+k,19)[0]=use[k]->side;P(2+k,27)[0]=mapY[k];P(2+k,31)[0]=fz[k];
  P(2+k,35)[0]=mapX[k];P(2+k,39)[0]=fx[k];P(2+k,43)[0]=fy[k];{const int pj=partnerOf(use,Zs,k);P(2+k,47)[0]=std::uint32_t(pj+1);}
  const auto&z=Zs[k];const std::size_t w0=planeRow*104+std::size_t(2*k)*4;const float pl[6]={float(z.n[0]),float(z.n[1]),float(z.n[2]),
   float(z.n[0]*z.C[0]+z.n[1]*z.C[1]+z.n[2]*z.C[2]),float(z.sLo),float(z.sHi)};std::memcpy(&A->params[w0],pl,24);
  A->params[w0+6]=mapX[k]|(fx[k]<<16);A->params[w0+7]=fy[k]|(std::uint32_t(use[k]->side&3u)<<12)|(std::uint32_t(std::lround(z.scale*65535.))<<16);}
 {std::size_t at=0;for(std::size_t c=0;c<NC;++c){const auto&l=lists[c];const std::size_t n=std::min<std::size_t>(l.size(),1023);
   A->params[tableRow*104+c]=std::uint32_t(at<<10|n);
   for(std::size_t j=0;j<n;++j,++at){std::uint32_t&wd=A->params[listRow*104+at/2];wd|=std::uint32_t(l[j])<<(16*(at&1));}}}
 A->bytes=A->params.size()*4+blockBytes;
 if(st){++st->sets;st->tears+=K;st->bytes+=A->bytes;st->indexCells+=NC;st->overlap3+=over3;st->listEntries+=entries;st->maxList=std::max(st->maxList,maxList);st->pruned+=pruned;
  st->ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count();}
 return A;}
}
