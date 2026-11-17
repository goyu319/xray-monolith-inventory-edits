#pragma once
#include <chrono>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <vector>
namespace gear_cells {
struct GearCells {
 Vec origin;double pitch{};Cell size;
 std::unordered_map<Cell,core::Tile,core::CellHash> tiles; // bit order of a Tile: removed[x & 7] bit (y & 7) * 8 + (z & 7)
 std::vector<std::uint8_t> tileFlag; // per tile of the lattice ((tz * tny + ty) * tnx + tx): 1 = holds gear cells
 std::size_t cells{},bytes{};double ms{};
 bool in(int x,int y,int z)const{auto it=tiles.find(Cell{x>>3,y>>3,z>>3});if(it==tiles.end())return false;
  return ((it->second.removed[std::size_t(x&7)]>>((y&7)*8+(z&7)))&1u)!=0;}
 const core::Tile* tile(int tx,int ty,int tz)const{auto it=tiles.find(Cell{tx,ty,tz});return it==tiles.end()?nullptr:&it->second;}
 // the same lattice as grid g (origin, pitch, size)
 bool fits(const Vec&o,double p,const Cell&s)const{return std::abs(o.x-origin.x)<1e-7&&std::abs(o.y-origin.y)<1e-7&&std::abs(o.z-origin.z)<1e-7&&
  std::abs(p-pitch)<1e-9&&s.x==size.x&&s.y==size.y&&s.z==size.z;}
};
// the tiles (grid lattice) whose tissue-field bricks may hold clothing padding (anatomy bit 64): once per outfit field and lattice
// (warmed at the asset's preparation, LiveDamageWorker.cpp; MaskPieces3.h owners() reads it at every separation)
inline std::shared_ptr<const std::vector<std::uint32_t>> padTiles(const std::shared_ptr<const painter_detail::TissueField>&tf,const Grid&grid){
 struct Entry {std::shared_ptr<const painter_detail::TissueField> tf;Vec origin;double pitch;Cell size;std::shared_ptr<const std::vector<std::uint32_t>> tiles;};
 static std::mutex mutex;static std::vector<Entry> cache;
 {std::lock_guard<std::mutex> lock(mutex);for(const auto&e:cache)if(e.tf==tf&&e.pitch==grid.pitch&&e.size==grid.size&&e.origin.x==grid.origin.x&&e.origin.y==grid.origin.y&&e.origin.z==grid.origin.z)return e.tiles;}
 auto out=std::make_shared<std::vector<std::uint32_t>>();const int tnx=(grid.size.x+7)/8,tny=(grid.size.y+7)/8,tnz=(grid.size.z+7)/8;
 std::vector<std::uint8_t> seen(std::size_t(tnx)*tny*tnz,0);
 for(std::uint32_t bz=0;bz<tf->pages[2];++bz)for(std::uint32_t by=0;by<tf->pages[1];++by)for(std::uint32_t bx=0;bx<tf->pages[0];++bx){
  const auto v=tf->page[(std::size_t(bz)*tf->pages[1]+by)*tf->pages[0]+bx+painter_detail::TissueField::headerWords];
  bool pad;if(v&painter_detail::TissueField::uniformBit)pad=(v&64u)!=0;else{const unsigned bits=1u<<((v>>28)&3u);pad=bits==8;
   if(!pad){const std::size_t o=v&0x0fffffffu;const unsigned n=1u<<bits;for(unsigned e=0;e<n&&!pad;++e)pad=o+e<tf->data.size()&&(tf->data[o+e]&64u);}}
  if(!pad)continue;int lo[3],hi[3];const std::uint32_t bb[3]={bx,by,bz};const int tn[3]={tnx,tny,tnz};bool any=true;
  for(int a=0;a<3;++a){const double w0=double(tf->origin[std::size_t(a)])+bb[a]*8.*tf->pitch,w1=w0+8.*tf->pitch;
   lo[a]=std::max(0,int(std::floor((w0-grid.origin[a])/grid.pitch))>>3);hi[a]=std::min(tn[a]-1,int(std::floor((w1-grid.origin[a])/grid.pitch-1e-9))>>3);any=any&&lo[a]<=hi[a];}
  if(!any)continue;
  for(int z=lo[2];z<=hi[2];++z)for(int y=lo[1];y<=hi[1];++y)for(int x=lo[0];x<=hi[0];++x){const std::size_t i=(std::size_t(z)*tny+std::size_t(y))*tnx+std::size_t(x);
   if(!seen[i]){seen[i]=1;out->push_back(std::uint32_t(i));}}}
 std::lock_guard<std::mutex> lock(mutex);if(cache.size()>=16)cache.erase(cache.begin());cache.push_back({tf,grid.origin,grid.pitch,grid.size,out});return out;}
// the same as dense flags per tile of the lattice (1 = may hold padding), cached alike
inline std::shared_ptr<const std::vector<std::uint8_t>> padTileFlags(const std::shared_ptr<const painter_detail::TissueField>&tf,const Grid&grid){
 struct Entry {std::shared_ptr<const std::vector<std::uint32_t>> list;std::shared_ptr<const std::vector<std::uint8_t>> flags;};static std::mutex mutex;static std::vector<Entry> cache;
 const auto list=padTiles(tf,grid);
 {std::lock_guard<std::mutex> lock(mutex);for(const auto&e:cache)if(e.list==list)return e.flags;}
 auto f=std::make_shared<std::vector<std::uint8_t>>(std::size_t((grid.size.x+7)/8)*std::size_t((grid.size.y+7)/8)*std::size_t((grid.size.z+7)/8),0);
 for(const auto i:*list)if(i<f->size())(*f)[i]=1;
 std::lock_guard<std::mutex> lock(mutex);if(cache.size()>=16)cache.erase(cache.begin());cache.push_back({list,f});return f;}
inline std::shared_ptr<const GearCells> build(const Source&s,const Grid&g){
 const auto t0=std::chrono::steady_clock::now();
 auto out=std::make_shared<GearCells>();out->origin=g.origin;out->pitch=g.pitch;out->size=g.size;
 if(!(g.pitch>0)||s.v.empty()||s.f.empty())return out;
 const double p=g.pitch,step=.7*p,inv=1./p;core::Tile*last=nullptr;Cell lastKey{INT_MIN,INT_MIN,INT_MIN};
 auto mark=[&](double x,double y,double z){const int cx=int(std::floor((x-g.origin.x)*inv)),cy=int(std::floor((y-g.origin.y)*inv)),cz=int(std::floor((z-g.origin.z)*inv));
  if(cx<0||cy<0||cz<0||cx>=g.size.x||cy>=g.size.y||cz>=g.size.z)return;const Cell key{cx>>3,cy>>3,cz>>3};
  if(!last||!(key==lastKey)){last=&out->tiles[key];lastKey=key;}last->removed[std::size_t(cx&7)]|=std::uint64_t(1)<<((cy&7)*8+(cz&7));};
 for(const auto&f:s.f){if(f[0]>=s.v.size()||f[1]>=s.v.size()||f[2]>=s.v.size())continue;
  const Vec A=s.v[f[0]].p,B=s.v[f[1]].p,C=s.v[f[2]].p;const Vec e1=B-A,e2=C-A;
  Vec n{e1.y*e2.z-e1.z*e2.y,e1.z*e2.x-e1.x*e2.z,e1.x*e2.y-e1.y*e2.x};const double ln=std::sqrt(n.x*n.x+n.y*n.y+n.z*n.z);if(!(ln>1e-14))continue;n=n*(1./ln);
  const double L=std::max({std::sqrt(e1.x*e1.x+e1.y*e1.y+e1.z*e1.z),std::sqrt(e2.x*e2.x+e2.y*e2.y+e2.z*e2.z),std::sqrt((C-B).x*(C-B).x+(C-B).y*(C-B).y+(C-B).z*(C-B).z)});
  const int N=std::max(1,int(std::ceil(L/step)));if(N>4096)continue; // a degenerate sliver longer than 5.7 m at 2 mm: not an outfit triangle
  for(int i=0;i<=N;++i)for(int j=0;i+j<=N;++j){const double a=double(i)/N,b=double(j)/N;const Vec P=A+e1*a+e2*b;
   mark(P.x,P.y,P.z);mark(P.x+n.x*p,P.y+n.y*p,P.z+n.z*p);mark(P.x-n.x*p,P.y-n.y*p,P.z-n.z*p);}}
 for(const auto&t:out->tiles)for(const auto w:t.second.removed){std::uint64_t v=w;while(v){v&=v-1;++out->cells;}}
 {const int tnx=(g.size.x+7)/8,tny=(g.size.y+7)/8,tnz=(g.size.z+7)/8;out->tileFlag.assign(std::size_t(tnx)*tny*tnz,0);
  for(const auto&t:out->tiles){const Cell&c=t.first;if(c.x>=0&&c.y>=0&&c.z>=0&&c.x<tnx&&c.y<tny&&c.z<tnz)out->tileFlag[(std::size_t(c.z)*tny+std::size_t(c.y))*tnx+std::size_t(c.x)]=1;}}
 out->bytes=out->tiles.size()*(sizeof(core::Tile)+sizeof(Cell)+32)+out->tiles.bucket_count()*sizeof(void*)+out->tileFlag.size();
 out->ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count();
 return out;}
}
