#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>
namespace crevice_padding {
constexpr double reachIn=.08,reachOut=.08; // m: how far behind / in front of a triangle is looked
using TF=painter_detail::TissueField;
// the cells of the body grid's lattice that get no padding
struct Plan {Vec origin;double pitch{};int sx{},sy{},sz{},tx{},ty{},tz{};std::vector<std::uint64_t> drop;std::vector<std::uint8_t> tile; // tile: 8^3 cells, 1 = holds a dropped cell
 std::size_t padding{},kept{},cleared{},dropped{},votesOut{},votesIn{},against{};double ms{};
 bool cell(int x,int y,int z)const{if(x<0||y<0||z<0||x>=sx||y>=sy||z>=sz)return false;const std::size_t i=(std::size_t(z)*sy+std::size_t(y))*sx+std::size_t(x);return ((drop[i>>6]>>(i&63))&1u)!=0;}
 bool tileAny(int a,int b,int c)const{return a>=0&&b>=0&&c>=0&&a<tx&&b<ty&&c<tz&&tile[(std::size_t(c)*ty+std::size_t(b))*tx+std::size_t(a)]!=0;}};
// markClosurePadding's rule (closure brick / closure cell coordinates)
struct Drop {const Plan*plan;const TF*closure;
 bool brick(unsigned bx,unsigned by,unsigned bz)const{int lo[3],hi[3];const unsigned b[3]={bx,by,bz};const double cp=closure->pitch;
  for(int a=0;a<3;++a){const double w0=double(closure->origin[std::size_t(a)])+b[a]*8.*cp,w1=w0+8.*cp;
   lo[a]=int(std::floor((w0-plan->origin[a])/plan->pitch+1e-9))>>3;hi[a]=int(std::floor((w1-plan->origin[a])/plan->pitch-1e-9))>>3;}
  for(int z=lo[2];z<=hi[2];++z)for(int y=lo[1];y<=hi[1];++y)for(int x=lo[0];x<=hi[0];++x)if(plan->tileAny(x,y,z))return true;
  return false;}
 bool cell(int x,int y,int z)const{const double cp=closure->pitch,p=plan->pitch;
  return plan->cell(int(std::floor((double(closure->origin[0])+(x+.5)*cp-plan->origin.x)/p)),int(std::floor((double(closure->origin[1])+(y+.5)*cp-plan->origin.y)/p)),
   int(std::floor((double(closure->origin[2])+(z+.5)*cp-plan->origin.z)/p)));}};
namespace detail {
// one bit per cell of the lattice, set from any thread
struct Bits {std::unique_ptr<std::atomic<std::uint64_t>[]> w;std::size_t words=0;int sx=0,sy=0,sz=0;
 void init(int x,int y,int z){sx=x;sy=y;sz=z;words=(std::size_t(x)*y*z+63)/64;w.reset(new std::atomic<std::uint64_t>[words]());}
 bool inside(int x,int y,int z)const{return x>=0&&y>=0&&z>=0&&x<sx&&y<sy&&z<sz;}
 bool in(int x,int y,int z)const{if(!inside(x,y,z))return false;const std::size_t i=(std::size_t(z)*sy+std::size_t(y))*sx+std::size_t(x);return ((w[i>>6].load(std::memory_order_relaxed)>>(i&63))&1u)!=0;}
 void set(int x,int y,int z){const std::size_t i=(std::size_t(z)*sy+std::size_t(y))*sx+std::size_t(x);w[i>>6].fetch_or(std::uint64_t(1)<<(i&63),std::memory_order_relaxed);}
 std::size_t count()const{std::size_t n=0;for(std::size_t i=0;i<words;++i){std::uint64_t v=w[i].load(std::memory_order_relaxed);while(v){v&=v-1;++n;}}return n;}
 // every set cell of the words [first, last)
 template<class F> void each(std::size_t first,std::size_t last,F&&f)const{const std::size_t row=std::size_t(sx),slab=row*std::size_t(sy);
  for(std::size_t i=first;i<last;++i){std::uint64_t v=w[i].load(std::memory_order_relaxed);
   for(unsigned b=0;v;++b,v>>=1)if(v&1u){const std::size_t c=i*64+b;f(int(c%row),int((c/row)%std::size_t(sy)),int(c/slab));}}}};
// (a tile map's bits with the last tile remembered: not used since the plan has its own lattice)
struct Band {const std::unordered_map<Cell,core::Tile,core::CellHash>*tiles;const core::Tile*last=nullptr;Cell lastKey{INT_MIN,INT_MIN,INT_MIN};
 bool in(int x,int y,int z){const Cell key{x>>3,y>>3,z>>3};if(!(key==lastKey)){const auto it=tiles->find(key);last=it==tiles->end()?nullptr:&it->second;lastKey=key;}
  return last&&((last->removed[std::size_t(x&7)]>>((y&7)*8+(z&7)))&1u)!=0;}};
}
// the plan of an outfit (its baked closure, its anatomy field, its triangles, the body grid's box)
inline std::shared_ptr<const Plan> plan(const TF&closure,const TF&anatomy,const Source&s,const Grid&g){
 const auto t0=std::chrono::steady_clock::now();auto out=std::make_shared<Plan>();
 auto finish=[&](){out->ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count();return std::shared_ptr<const Plan>(out);};
 if(!closure.available||!anatomy.available||!(g.pitch>0)||!(closure.pitch>0)||s.v.empty()||s.f.empty())return finish();
 // the plan's own lattice: the body grid's box in cells of 2 mm at least (the padding's shape does not need the wound cells' size;
 // at 1 mm wound cells the grid's own lattice would be 1.9 G cells)
 const double p=std::max(double(g.pitch),.002);
 const int sx=std::max(1,int(std::ceil(g.size.x*double(g.pitch)/p-1e-9))),sy=std::max(1,int(std::ceil(g.size.y*double(g.pitch)/p-1e-9))),sz=std::max(1,int(std::ceil(g.size.z*double(g.pitch)/p-1e-9)));
 const int sdim[3]={sx,sy,sz};
 const std::size_t cellsAll=std::size_t(sx)*std::size_t(sy)*std::size_t(sz);
 if(!cellsAll||cellsAll>(std::size_t(1)<<30))return finish(); // beyond 128 MiB a bit set: not this outfit
 const unsigned px=closure.pages[0],py=closure.pages[1],pz=closure.pages[2];const std::size_t bricks=std::size_t(px)*py*pz;
 if(!bricks||bricks+TF::headerWords>closure.page.size())return finish();
 out->origin=g.origin;out->pitch=p;out->sx=sx;out->sy=sy;out->sz=sz;
 const double cp=closure.pitch,q=.25*cp,inv=1./p; // a cell's label: read a quarter closure cell off its centre (never on a closure cell's face)
 auto closureAt=[&](int x,int y,int z)->unsigned{return closure.sample(g.origin.x+(x+.5)*p+q,g.origin.y+(y+.5)*p+q,g.origin.z+(z+.5)*p+q)&63u;};
 auto anatomyAt=[&](int x,int y,int z)->unsigned{return anatomy.sample(g.origin.x+(x+.5)*p+q,g.origin.y+(y+.5)*p+q,g.origin.z+(z+.5)*p+q);};
 const std::size_t width=std::max<std::size_t>(1,core::parallelWidth());
 detail::Bits pad,keep,clear,grow,band;pad.init(sx,sy,sz);keep.init(sx,sy,sz);clear.init(sx,sy,sz);grow.init(sx,sy,sz);band.init(sx,sy,sz);
 // 1. the cells the envelope holds without anatomy (the padding of before): the closure's bricks with a label, but not those the
 //    anatomy fills whole (its brick under a closure brick when the two lattices differ by whole bricks, as markClosurePadding)
 int ak[3]={0,0,0};bool aligned=std::abs(double(closure.pitch)-double(anatomy.pitch))<1e-9*double(closure.pitch);
 for(int a=0;a<3&&aligned;++a){const double shift=(double(anatomy.origin[std::size_t(a)])-double(closure.origin[std::size_t(a)]))/(cp*8);const double r=std::round(shift);aligned=std::abs(shift-r)<1e-3;ak[a]=int(r);}
 {std::atomic<std::size_t> n{0};
  core::parallelChunks(bricks,std::max<std::size_t>(256,bricks/(width*8)),[&](std::size_t first,std::size_t last,std::size_t){std::size_t local=0;
   for(std::size_t i=first;i<last;++i){const std::uint32_t v=closure.page[i+TF::headerWords];if((v&TF::uniformBit)&&!(v&63u))continue;
    const unsigned bb[3]={unsigned(i%px),unsigned((i/px)%py),unsigned(i/(std::size_t(px)*py))};int lo[3],hi[3];bool some=true;
    if(aligned){const int ax=int(bb[0])-ak[0],ay=int(bb[1])-ak[1],az=int(bb[2])-ak[2];
     if(ax>=0&&ay>=0&&az>=0&&unsigned(ax)<anatomy.pages[0]&&unsigned(ay)<anatomy.pages[1]&&unsigned(az)<anatomy.pages[2]){
      const std::uint32_t ap=anatomy.page[(std::size_t(az)*anatomy.pages[1]+unsigned(ay))*anatomy.pages[0]+unsigned(ax)+TF::headerWords];
      if((ap&TF::uniformBit)&&(ap&127u))continue;}} // flesh throughout: no padding in this brick
    for(int a=0;a<3;++a){const double w0=double(closure.origin[std::size_t(a)])+bb[a]*8.*cp,w1=w0+8.*cp; // the cells read inside [w0, w1)
     lo[a]=std::max(0,int(std::ceil((w0-q-g.origin[a])/p-.5-1e-9)));hi[a]=std::min(sdim[a]-1,int(std::ceil((w1-q-g.origin[a])/p-.5-1e-9))-1);some=some&&lo[a]<=hi[a];}
    if(!some)continue;
    for(int x=lo[0];x<=hi[0];++x)for(int y=lo[1];y<=hi[1];++y)for(int z=lo[2];z<=hi[2];++z)if(closureAt(x,y,z)!=0&&anatomyAt(x,y,z)==0){pad.set(x,y,z);++local;}}
   n.fetch_add(local,std::memory_order_relaxed);});
  out->padding=n.load();}
 if(!out->padding)return finish();
 // 2. the outfit's majority: open air (no closure label) lies on the outer side of a surface
 std::size_t vo=0,vi=0;
 for(const auto&f:s.f){if(f[0]>=s.v.size()||f[1]>=s.v.size()||f[2]>=s.v.size())continue;
  const Vec A=s.v[f[0]].p,B=s.v[f[1]].p,C=s.v[f[2]].p;Vec n=cross(B-A,C-A);const double ln=length(n);if(!(ln>1e-14))continue;n=n*(1./ln);
  const Vec c=(A+B+C)*(1./3.);
  for(int k=1;k<=2;++k){const Vec a=c+n*(.004*k),b=c-n*(.004*k);const unsigned la=closure.sample(a.x,a.y,a.z),lb=closure.sample(b.x,b.y,b.z);
   if(la==0&&lb!=0)++vo;else if(la!=0&&lb==0)++vi;}}
 out->votesOut=vo;out->votesIn=vi;
 const bool decided=vo+vi>=64&&std::max(vo,vi)>=4*std::min(vo,vi);const double major=vo>=vi?1.:-1.;
 // every triangle's surface sampled at 0.7 cell (as GearCells): body(point, unit normal toward the outside); a triangle whose
 // own shading normal does not say which side is outside takes the outfit's majority (none either: left out)
 // (the samples of one triangle that start in a cell another sample of it started in are left out: their rays run side by side)
 const double step=.7*p,along=.5*p;std::atomic<std::size_t> against{0};
 auto cell=[&](const Vec&w,int&x,int&y,int&z){x=int(std::floor((w.x-g.origin.x)*inv));y=int(std::floor((w.y-g.origin.y)*inv));z=int(std::floor((w.z-g.origin.z)*inv));
  return x>=0&&y>=0&&z>=0&&x<sx&&y<sy&&z<sz;};
 struct Seen {std::array<std::array<int,4>,64> e{};}; // one per thread: the start cells of the triangle in hand (cell, triangle + 1)
 auto faces=[&](std::size_t first,std::size_t last,bool count,Seen&seen,auto&&body){std::size_t flipped=0;
  for(std::size_t fi=first;fi<last;++fi){const auto&f=s.f[fi];if(f[0]>=s.v.size()||f[1]>=s.v.size()||f[2]>=s.v.size())continue;
   const Vec A=s.v[f[0]].p,B=s.v[f[1]].p,C=s.v[f[2]].p;const Vec e1=B-A,e2=C-A;Vec n=cross(e1,e2);const double ln=length(n);if(!(ln>1e-14))continue;n=n*(1./ln);
   const double sh=dot(n,s.v[f[0]].normal+s.v[f[1]].normal+s.v[f[2]].normal);double side=sh>.3?1.:sh<-.3?-1.:0.;
   if(side==0.){if(!decided)continue;side=major;}
   if(decided&&side!=major)++flipped;
   n=n*side;
   const double L=std::max({length(e1),length(e2),length(C-B)});const int N=std::max(1,int(std::ceil(L/step)));if(N>4096)continue;const int stamp=int(fi)+1;
   for(int i=0;i<=N;++i)for(int j=0;i+j<=N;++j){const Vec P=A+e1*(double(i)/N)+e2*(double(j)/N);int cx,cy,cz;if(!cell(P,cx,cy,cz))continue;
    auto&e=seen.e[(unsigned(cx)*73856093u^unsigned(cy)*19349663u^unsigned(cz)*83492791u)&63u];
    if(e[3]==stamp&&e[0]==cx&&e[1]==cy&&e[2]==cz)continue;e={cx,cy,cz,stamp};
    body(P,n);}}
  if(count)against.fetch_add(flipped,std::memory_order_relaxed);};
 const std::size_t grainF=std::max<std::size_t>(64,s.f.size()/(width*16));
 // the band of the outfit's triangles on this lattice (as GearCells: the surface and one cell to either side along the normal)
 core::parallelChunks(s.f.size(),grainF,[&](std::size_t first,std::size_t last,std::size_t){
  for(std::size_t fi=first;fi<last;++fi){const auto&f=s.f[fi];if(f[0]>=s.v.size()||f[1]>=s.v.size()||f[2]>=s.v.size())continue;
   const Vec A=s.v[f[0]].p,B=s.v[f[1]].p,C=s.v[f[2]].p;const Vec e1=B-A,e2=C-A;Vec n=cross(e1,e2);const double ln=length(n);if(!(ln>1e-14))continue;n=n*(p/ln);
   const double L=std::max({length(e1),length(e2),length(C-B)});const int N=std::max(1,int(std::ceil(L/step)));if(N>4096)continue;
   for(int i=0;i<=N;++i)for(int j=0;i+j<=N;++j){const Vec P=A+e1*(double(i)/N)+e2*(double(j)/N);int x,y,z;
    if(cell(P,x,y,z))band.set(x,y,z);if(cell(P+n,x,y,z))band.set(x,y,z);if(cell(P-n,x,y,z))band.set(x,y,z);}}});
 // 3. KEEP: behind every triangle the envelope's cells without anatomy down to the flesh, or to the next surface
 core::parallelChunks(s.f.size(),grainF,[&](std::size_t first,std::size_t last,std::size_t){Seen seen;
  faces(first,last,true,seen,[&](const Vec&P,const Vec&n){int lx=INT_MIN,ly=0,lz=0,zeros=0,cells=0;bool left=false;
   for(double t=0;t<=reachIn;t+=along){int x,y,z;if(!cell(P-n*t,x,y,z))break;if(x==lx&&y==ly&&z==lz)continue;lx=x;ly=y;lz=z;++cells;
    if(pad.in(x,y,z)){keep.set(x,y,z);zeros=0;}
    else{if(closureAt(x,y,z)!=0)break; /* flesh: the backing ends */ if(++zeros>=2)break; /* open air */}
    if(!band.in(x,y,z))left=true;else if(left||cells>4)break;}});}); // the next surface: its own triangles keep what lies behind it
 static const int d6[6][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
 const std::size_t grainW=std::max<std::size_t>(1024,keep.words/(width*8));
 // (the rays the start-cell rule keeps apart leave single cells out: a cell beside a kept one, off every surface's band, is kept too -
 //  so no ray of pass 4 slips through the inside of gear)
 core::parallelChunks(keep.words,grainW,[&](std::size_t first,std::size_t last,std::size_t){
  keep.each(first,last,[&](int x,int y,int z){for(const auto&d:d6){const int ax=x+d[0],ay=y+d[1],az=z+d[2];if(pad.in(ax,ay,az)&&!keep.in(ax,ay,az)&&!band.in(ax,ay,az))grow.set(ax,ay,az);}});});
 for(std::size_t i=0;i<keep.words;++i)if(const std::uint64_t v=grow.w[i].load(std::memory_order_relaxed)){keep.w[i].fetch_or(v,std::memory_order_relaxed);grow.w[i].store(0,std::memory_order_relaxed);}
 // 4. CLEAR: in front of every triangle those cells out to open air, another part's flesh or the next surface; never KEEP
 core::parallelChunks(s.f.size(),grainF,[&](std::size_t first,std::size_t last,std::size_t){Seen seen;
  faces(first,last,false,seen,[&](const Vec&P,const Vec&n){int lx,ly,lz;if(!cell(P,lx,ly,lz))return;int zeros=0,cells=0;bool left=!band.in(lx,ly,lz),past=false;
   for(double t=along;t<=reachOut;t+=along){int x,y,z;if(!cell(P+n*t,x,y,z))break;if(x==lx&&y==ly&&z==lz)continue;lx=x;ly=y;lz=z;++cells;
    if(!band.in(x,y,z))left=true;else if(left)break; // the next surface: its own triangles clear their own outer side
    if(keep.in(x,y,z))break; // behind another triangle (gear worn over this surface)
    if(pad.in(x,y,z)){clear.set(x,y,z);zeros=0;past=true;continue;}
    if(closureAt(x,y,z)!=0){if(past||cells>3)break;continue;} // flesh: this part's own just past its clothing (the first cells), else another part
    past=true;if(++zeros>=2)break;}});}); // open air
 out->against=against.load();
 // 5. the rays of a convex surface spread: a cell beside a cleared one, off every surface's band and not KEEP, goes with it
 core::parallelChunks(clear.words,grainW,[&](std::size_t first,std::size_t last,std::size_t){
  clear.each(first,last,[&](int x,int y,int z){grow.set(x,y,z);
   for(const auto&d:d6){const int ax=x+d[0],ay=y+d[1],az=z+d[2];if(pad.in(ax,ay,az)&&!keep.in(ax,ay,az)&&!band.in(ax,ay,az))grow.set(ax,ay,az);}});});
 // 6. and the cells beside those that the lattice does not read as padding (a cell read as flesh or air can hold a 1 mm sheet of
 //    it): the sheet gets no padding either. `grow` = the cleared cells; `clear` takes the cells beside them.
 core::parallelChunks(grow.words,grainW,[&](std::size_t first,std::size_t last,std::size_t){
  grow.each(first,last,[&](int x,int y,int z){for(const auto&d:d6){const int ax=x+d[0],ay=y+d[1],az=z+d[2];if(clear.inside(ax,ay,az)&&!pad.in(ax,ay,az)&&!keep.in(ax,ay,az))clear.set(ax,ay,az);}});});
 out->kept=keep.count();out->cleared=grow.count();
 out->drop.resize(grow.words);for(std::size_t i=0;i<grow.words;++i)out->drop[i]=grow.w[i].load(std::memory_order_relaxed)|clear.w[i].load(std::memory_order_relaxed);
 out->tx=(sx+7)/8;out->ty=(sy+7)/8;out->tz=(sz+7)/8;out->tile.assign(std::size_t(out->tx)*out->ty*out->tz,0);
 {const std::size_t row=std::size_t(sx),slab=row*std::size_t(sy);
  for(std::size_t i=0;i<out->drop.size();++i){std::uint64_t v=out->drop[i];for(unsigned b=0;v;++b,v>>=1)if(v&1u){++out->dropped;const std::size_t c=i*64+b;
   out->tile[(std::size_t(int(c/slab)>>3)*out->ty+std::size_t(int((c/row)%std::size_t(sy))>>3))*out->tx+std::size_t(int(c%row)>>3)]=1;}}}
 return finish();}
} // namespace crevice_padding
