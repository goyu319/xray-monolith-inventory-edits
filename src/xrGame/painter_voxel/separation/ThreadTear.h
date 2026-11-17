#pragma once
namespace thread_tear {
constexpr int radius=2;
constexpr double minimum_piece_m3=5e-5;
struct Result {std::shared_ptr<const Grid> grid;std::size_t removed{},tiles{};double ms{};};
// Reference (per-cell block fill, byte passes): GFY_THREAD_TEAR_VERIFY=1 compares open() against it.
inline Result openReference(const Grid&g){
 const auto t0=core::Clock::now();Result out;
 std::vector<Cell> keys;keys.reserve(g.tiles.size());
 for(const auto&it:g.tiles){std::uint64_t any=0,all=~std::uint64_t(0);for(auto w:it.second.removed){any|=w;all&=w;}if(any&&all!=~std::uint64_t(0))keys.push_back(it.first);}
 out.tiles=keys.size();
 if(keys.empty()||keys.size()>60000){out.ms=core::ms(t0);return out;}
 std::vector<std::pair<Cell,core::Tile>> adds;std::mutex addMutex;std::atomic<std::size_t> total{0};
 core::parallelChunks(keys.size(),32,[&](std::size_t first,std::size_t last,std::size_t){
  constexpr int R=radius,H=2*R,W=8+2*H;
  std::vector<std::uint8_t> solid(W*W*W),a(W*W*W),b(W*W*W);std::vector<std::pair<Cell,core::Tile>> local;
  auto at=[&](int x,int y,int z){return (x*W+y)*W+z;};
  // Separable min (erode) or max (dilate) over +-R along one axis; out of range counts as air.
  auto pass=[&](const std::vector<std::uint8_t>&in,std::vector<std::uint8_t>&res,int axis,bool erode){
   for(int x=0;x<W;++x)for(int y=0;y<W;++y)for(int z=0;z<W;++z){std::uint8_t v=erode?1:0;
    for(int d=-R;d<=R;++d){const int q[3]={x+(axis==0?d:0),y+(axis==1?d:0),z+(axis==2?d:0)};
     const std::uint8_t s=q[0]<0||q[1]<0||q[2]<0||q[0]>=W||q[1]>=W||q[2]>=W?0:in[at(q[0],q[1],q[2])];
     if(erode){if(!s){v=0;break;}}else if(s){v=1;break;}}
    res[at(x,y,z)]=v;}};
  for(std::size_t i=first;i<last;++i){const Cell key=keys[i];const Cell base{key.x*8-H,key.y*8-H,key.z*8-H};
   for(int x=0;x<W;++x)for(int y=0;y<W;++y)for(int z=0;z<W;++z){const Cell c{base.x+x,base.y+y,base.z+z};
    solid[at(x,y,z)]=g.valid(c)&&g.rawMaterialBase(c)&&!g.removed(c)?1:0;}
   pass(solid,a,2,true);pass(a,b,1,true);pass(b,a,0,true);   // a = eroded (valid in [R,W-R))
   pass(a,b,2,false);pass(b,a,1,false);pass(a,b,0,false);    // b = opened (valid in [H,W-H) = the tile)
   const auto found=g.tiles.find(key);core::Tile t=found!=g.tiles.end()?found->second:core::Tile{};std::size_t n=0;
   for(int x=0;x<8;++x)for(int y=0;y<8;++y)for(int z=0;z<8;++z){const int k=at(x+H,y+H,z+H);if(!solid[k]||b[k])continue;
    const Cell c{key.x*8+x,key.y*8+y,key.z*8+z};const auto bit=Grid::local(c);t.removed[bit/64]|=std::uint64_t(1)<<(bit%64);++n;}
   if(n){local.emplace_back(key,t);total+=n;}}
  std::lock_guard<std::mutex> lock(addMutex);adds.insert(adds.end(),local.begin(),local.end());});
 out.removed=total;
 if(!adds.empty()){auto opened=std::make_shared<Grid>(g);opened->journal=nullptr;for(auto&a:adds)opened->tiles[a.first]=a.second;out.grid=std::move(opened);}
 out.ms=core::ms(t0);return out;
}
// table: the model's static tissue masks (Asset::tile_material, complete; bit = rawMaterialBase(c)!=0 on this lattice).
// split (2026-11-11): the former open(), kept for A/B runs (GFY_THREAD_TEAR_FAST=0); open() below runs openFast().
inline Result openFormer(const Grid&g,const std::unordered_map<Cell,SeverState::TileMaterial,core::CellHash>*table){
 const auto t0=core::Clock::now();Result out;
 std::vector<Cell> keys;keys.reserve(g.tiles.size());
 for(const auto&it:g.tiles){std::uint64_t any=0,all=~std::uint64_t(0);for(auto w:it.second.removed){any|=w;all&=w;}if(any&&all!=~std::uint64_t(0))keys.push_back(it.first);}
 out.tiles=keys.size();
 if(keys.empty()||keys.size()>60000){out.ms=core::ms(t0);return out;}
 static_assert(radius==2,"block layout below assumes the 16^3 block of radius 2 (a tile and half of each neighbour)");
 // Tiles the blocks read: each key's 3x3x3 neighbourhood, indexed densely over the keys' box (+1).
 Cell lo=keys[0],hi=keys[0];
 for(const auto&k:keys){lo={std::min(lo.x,k.x),std::min(lo.y,k.y),std::min(lo.z,k.z)};hi={std::max(hi.x,k.x),std::max(hi.y,k.y),std::max(hi.z,k.z)};}
 lo={lo.x-1,lo.y-1,lo.z-1};hi={hi.x+1,hi.y+1,hi.z+1};
 const std::size_t ny=std::size_t(hi.y-lo.y+1),nz=std::size_t(hi.z-lo.z+1);
 std::vector<std::uint32_t> slot(std::size_t(hi.x-lo.x+1)*ny*nz,0);
 auto slotOf=[&](const Cell&t)->std::uint32_t&{return slot[(std::size_t(t.x-lo.x)*ny+std::size_t(t.y-lo.y))*nz+std::size_t(t.z-lo.z)];};
 std::vector<Cell> wanted;wanted.reserve(keys.size()*4);
 for(const auto&k:keys)for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)for(int dz=-1;dz<=1;++dz){const Cell t{k.x+dx,k.y+dy,k.z+dz};
  auto&s=slotOf(t);if(!s){wanted.push_back(t);s=std::uint32_t(wanted.size());}}
 std::vector<std::array<std::uint64_t,8>> mask(wanted.size());
 core::parallelChunks(wanted.size(),64,[&](std::size_t first,std::size_t last,std::size_t){
  for(std::size_t i=first;i<last;++i){const Cell t=wanted[i];std::array<std::uint64_t,8> m{};
   if(t.x>=0&&t.y>=0&&t.z>=0&&t.x*8<g.size.x&&t.y*8<g.size.y&&t.z*8<g.size.z){ // else every cell is outside the grid
    const core::Tile* stored=g.findTile(t);
    const bool inside=(t.x+1)*8<=g.size.x&&(t.y+1)*8<=g.size.y&&(t.z+1)*8<=g.size.z; // every cell valid
    if(table&&inside){
     const auto it=table->find(t);
     if(it!=table->end()){m=it->second.solid;if(stored)for(int w=0;w<8;++w)m[w]&=~stored->removed[w];
      if(g.ownership&&!g.ownership->pureTile(Cell{t.x*8,t.y*8,t.z*8}))
       for(int w=0;w<8;++w)for(std::uint64_t bits=m[w];bits;bits&=bits-1){const unsigned b=core::lowestBit(bits);
        if(!g.ownerCuttable(Cell{t.x*8+w,t.y*8+int(b>>3),t.z*8+int(b&7)}))m[w]&=~(std::uint64_t(1)<<b);}}
    }else
    for(int x=0;x<8;++x)for(int y=0;y<8;++y)for(int z=0;z<8;++z){const Cell c{t.x*8+x,t.y*8+y,t.z*8+z};
     if(!g.valid(c)||!g.rawMaterialBase(c))continue;
     const auto bit=Grid::local(c);
     if(stored&&((stored->removed[bit/64]>>(bit%64))&1u))continue;
     if(g.ownership&&!g.ownerCuttable(c))continue;
     m[bit/64]|=std::uint64_t(1)<<(bit%64);}}
   mask[i]=m;}});
 std::vector<std::pair<Cell,core::Tile>> adds;std::mutex addMutex;std::atomic<std::size_t> total{0};
 core::parallelChunks(keys.size(),32,[&](std::size_t first,std::size_t last,std::size_t){
  // Block rows: index x*16+y (block coordinates, block origin = key*8-4), bit z.
  using Rows=std::array<std::uint16_t,256>;Rows solid,a,b;std::vector<std::pair<Cell,core::Tile>> local;
  auto zPass=[](const Rows&in,Rows&res,bool erode){
   for(int i=0;i<256;++i){const unsigned s=in[i];
    res[i]=std::uint16_t(erode?(s&(s>>1)&(s>>2)&(s<<1)&(s<<2)):((s|(s>>1)|(s>>2)|(s<<1)|(s<<2))&0xFFFFu));}};
  auto rowPass=[](const Rows&in,Rows&res,int axis,bool erode){
   for(int x=0;x<16;++x)for(int y=0;y<16;++y){unsigned v=erode?0xFFFFu:0u;
    for(int d=-radius;d<=radius;++d){const int qx=x+(axis==0?d:0),qy=y+(axis==1?d:0);
     const unsigned s=qx<0||qy<0||qx>=16||qy>=16?0u:in[qx*16+qy];v=erode?(v&s):(v|s);}
    res[x*16+y]=std::uint16_t(v);}};
  for(std::size_t i=first;i<last;++i){const Cell key=keys[i];
   const std::array<std::uint64_t,8>* nb[3][3][3];
   for(int dx=0;dx<3;++dx)for(int dy=0;dy<3;++dy)for(int dz=0;dz<3;++dz)nb[dx][dy][dz]=&mask[slotOf(Cell{key.x+dx-1,key.y+dy-1,key.z+dz-1})-1];
   for(int x=0;x<16;++x){const int tx=x<4?0:x<12?1:2,lx=x<4?x+4:x<12?x-4:x-12;
    for(int y=0;y<16;++y){const int ty=y<4?0:y<12?1:2,ly=y<4?y+4:y<12?y-4:y-12;
     auto row=[&](int tz){return unsigned(((*nb[tx][ty][tz])[lx]>>(ly*8))&255u);};
     solid[x*16+y]=std::uint16_t((row(0)>>4)|(row(1)<<4)|((row(2)&15u)<<12));}}
   zPass(solid,a,true);rowPass(a,b,1,true);rowPass(b,a,0,true);    // a = eroded
   zPass(a,b,false);rowPass(b,a,1,false);rowPass(a,b,0,false);     // b = opened
   const auto found=g.tiles.find(key);core::Tile t=found!=g.tiles.end()?found->second:core::Tile{};std::size_t n=0;
   for(int x=0;x<8;++x)for(int y=0;y<8;++y){const unsigned k=(x+4)*16+(y+4);
    const unsigned fresh=(unsigned(solid[k])&~unsigned(b[k])&0xFFFFu)>>4&255u;if(!fresh)continue;
    t.removed[x]|=std::uint64_t(fresh)<<(y*8);for(unsigned f=fresh;f;f&=f-1)++n;}
   if(n){local.emplace_back(key,t);total+=n;}}
  std::lock_guard<std::mutex> lock(addMutex);adds.insert(adds.end(),local.begin(),local.end());});
 out.removed=total;
 if(!adds.empty()){auto opened=std::make_shared<Grid>(g);opened->journal=nullptr;for(auto&a:adds)opened->tiles[a.first]=a.second;out.grid=std::move(opened);}
 out.ms=core::ms(t0);return out;
}
inline void ownedRuns(const std::vector<core::OwnerColumn>&columns,const std::vector<core::OwnerSpan>&spans,const Cell&t,std::array<std::uint64_t,8>&bits){
 if(columns.empty())return;
 const int x0=t.x*8,z0=t.z*8;
 for(int y=0;y<8;++y){const int gy=t.y*8+y;
  auto col=std::lower_bound(columns.begin(),columns.end(),x0,[gy](const core::OwnerColumn&a,int x){return a.y<gy||(a.y==gy&&a.x<x);});
  for(;col!=columns.end()&&col->y==gy&&col->x<x0+8;++col){
   const auto first=spans.begin()+col->first,last=first+col->count;
   auto s=std::lower_bound(first,last,z0,[](const core::OwnerSpan&span,int z){return span.hi<=z;});
   for(;s!=last&&s->lo<z0+8;++s){const int lo=std::max(s->lo,z0)-z0,hi=std::min(s->hi,z0+8)-z0;if(lo>=hi)continue;
    bits[std::size_t(col->x-x0)]|=(((std::uint64_t(1)<<unsigned(hi-lo))-1)<<unsigned(lo))<<unsigned(y*8);}}}
}
inline Result openFast(const Grid&g,const std::unordered_map<Cell,SeverState::TileMaterial,core::CellHash>*table){
 const auto t0=core::Clock::now();Result out;
 static const bool prof=std::getenv("GFY_THREAD_TEAR_PROF")!=nullptr;auto pt=t0;double pms[8]={};int pat=0; // log only
 const auto lap=[&]{if(!prof)return;const auto now=core::Clock::now();if(pat<8)pms[pat++]=std::chrono::duration<double,std::milli>(now-pt).count();pt=now;};
 // keys: the partly removed tiles, as before; damaged: every tile with a removed bit (the stored masks of the neighbours)
 std::vector<Cell> keys;keys.reserve(g.tiles.size());std::vector<const core::Tile*> keyTile;keyTile.reserve(g.tiles.size());
 std::vector<std::pair<Cell,const core::Tile*>> damaged;damaged.reserve(g.tiles.size());
 for(const auto&it:g.tiles){std::uint64_t any=0,all=~std::uint64_t(0);for(auto w:it.second.removed){any|=w;all&=w;}
  if(!any)continue;damaged.push_back({it.first,&it.second});if(all!=~std::uint64_t(0)){keys.push_back(it.first);keyTile.push_back(&it.second);}}
 out.tiles=keys.size();lap();
 if(keys.empty()||keys.size()>60000){out.ms=core::ms(t0);return out;}
 static_assert(radius==2,"block layout below assumes the 16^3 block of radius 2 (a tile and half of each neighbour)");
 // Tiles the blocks read, indexed densely over the keys' box (+1): the keys first, then the neighbours of the keys that hold tissue.
 Cell lo=keys[0],hi=keys[0];
 for(const auto&k:keys){lo={std::min(lo.x,k.x),std::min(lo.y,k.y),std::min(lo.z,k.z)};hi={std::max(hi.x,k.x),std::max(hi.y,k.y),std::max(hi.z,k.z)};}
 lo={lo.x-1,lo.y-1,lo.z-1};hi={hi.x+1,hi.y+1,hi.z+1};
 const std::size_t ny=std::size_t(hi.y-lo.y+1),nz=std::size_t(hi.z-lo.z+1);
 std::vector<std::uint32_t> slot(std::size_t(hi.x-lo.x+1)*ny*nz,0);
 auto slotOf=[&](const Cell&t)->std::uint32_t&{return slot[(std::size_t(t.x-lo.x)*ny+std::size_t(t.y-lo.y))*nz+std::size_t(t.z-lo.z)];};
 std::vector<Cell> wanted;wanted.reserve(keys.size()*4);
 for(const auto&k:keys){wanted.push_back(k);slotOf(k)=std::uint32_t(wanted.size());}
 const std::size_t K=keys.size();
 const auto*own=g.ownership.get();
 const auto solidMask=[&](const Cell&t,const core::Tile*stored)->std::array<std::uint64_t,8>{std::array<std::uint64_t,8> m{};
  if(!(t.x>=0&&t.y>=0&&t.z>=0&&t.x*8<g.size.x&&t.y*8<g.size.y&&t.z*8<g.size.z))return m; // every cell is outside the grid
  const bool inside=(t.x+1)*8<=g.size.x&&(t.y+1)*8<=g.size.y&&(t.z+1)*8<=g.size.z; // every cell valid
  if(table&&inside){
   const auto it=table->find(t);if(it==table->end())return m;
   m=it->second.solid;if(stored)for(int w=0;w<8;++w)m[w]&=~stored->removed[w];
   if(own&&!own->pureTile(Cell{t.x*8,t.y*8,t.z*8})){std::array<std::uint64_t,8> mine{};
    ownedRuns(own->body_columns,own->body_spans,t,mine);ownedRuns(own->cloth_columns,own->cloth_spans,t,mine);
    for(int w=0;w<8;++w)m[w]&=mine[w];}
   return m;}
  for(int x=0;x<8;++x)for(int y=0;y<8;++y)for(int z=0;z<8;++z){const Cell c{t.x*8+x,t.y*8+y,t.z*8+z};
   if(!g.valid(c)||!g.rawMaterialBase(c))continue;
   const auto bit=Grid::local(c);
   if(stored&&((stored->removed[bit/64]>>(bit%64))&1u))continue;
   if(g.ownership&&!g.ownerCuttable(c))continue;
   m[bit/64]|=std::uint64_t(1)<<(bit%64);}
  return m;};
 lap();std::vector<std::array<std::uint64_t,8>> mask(K);
 core::parallelChunks(K,64,[&](std::size_t first,std::size_t last,std::size_t){
  for(std::size_t i=first;i<last;++i)mask[i]=solidMask(keys[i],g.overlay_base?g.findTile(keys[i]):keyTile[i]);});
 lap();
 // Neighbours of the keys that hold tissue (a key without tissue has nothing to open).
 std::vector<std::uint32_t> live;live.reserve(K);
 for(std::size_t i=0;i<K;++i){const auto&m=mask[i];if(!(m[0]|m[1]|m[2]|m[3]|m[4]|m[5]|m[6]|m[7]))continue;live.push_back(std::uint32_t(i));
  const Cell&k=keys[i];
  for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)for(int dz=-1;dz<=1;++dz){const Cell t{k.x+dx,k.y+dy,k.z+dz};
   auto&s=slotOf(t);if(!s){wanted.push_back(t);s=std::uint32_t(wanted.size());}}}
 if(live.empty()){out.ms=core::ms(t0);return out;}
 std::vector<const core::Tile*> stored(wanted.size(),nullptr);
 if(g.overlay_base){for(std::size_t i=K;i<wanted.size();++i)stored[i]=g.findTile(wanted[i]);} // an overlay grid: its base's tiles count too
 else for(const auto&d:damaged){const Cell&t=d.first;if(t.x<lo.x||t.y<lo.y||t.z<lo.z||t.x>hi.x||t.y>hi.y||t.z>hi.z)continue;
  const auto s=slotOf(t);if(s)stored[s-1]=d.second;}
 lap();mask.resize(wanted.size());
 core::parallelChunks(wanted.size()-K,64,[&](std::size_t first,std::size_t last,std::size_t){
  for(std::size_t i=K+first;i<K+last;++i)mask[i]=solidMask(wanted[i],stored[i]);});
 lap();std::vector<std::pair<Cell,core::Tile>> adds;std::mutex addMutex;std::atomic<std::size_t> total{0};
 core::parallelChunks(live.size(),32,[&](std::size_t first,std::size_t last,std::size_t){
  // Block rows (block origin = key*8-4): 16 x 16 rows of 16 z bits, row (x,y) at (x+2)*S+(y+2), inside a zero margin of two
  // rows in x and y. A pass writes only the 16 x 16 rows, so the margins of all three arrays stay zero.
  constexpr int S=20;std::uint16_t solid[S*S]={},a[S*S]={},b[S*S]={};std::vector<std::pair<Cell,core::Tile>> local;
  for(std::size_t q=first;q<last;++q){const std::size_t i=live[q];const Cell key=keys[i];
   const std::array<std::uint64_t,8>* nb[3][3][3];
   for(int dx=0;dx<3;++dx)for(int dy=0;dy<3;++dy)for(int dz=0;dz<3;++dz)nb[dx][dy][dz]=&mask[slotOf(Cell{key.x+dx-1,key.y+dy-1,key.z+dz-1})-1];
   for(int x=0;x<16;++x){const int tx=x<4?0:x<12?1:2,lx=x<4?x+4:x<12?x-4:x-12;std::uint16_t*row=solid+(x+2)*S+2;
    for(int ty=0;ty<3;++ty){const std::uint64_t w0=(*nb[tx][ty][0])[std::size_t(lx)],w1=(*nb[tx][ty][1])[std::size_t(lx)],w2=(*nb[tx][ty][2])[std::size_t(lx)];
     const int y0=ty==0?0:ty==1?4:12,y1=ty==0?4:ty==1?12:16,ly0=ty==0?4:0;
     for(int y=y0;y<y1;++y){const unsigned sh=unsigned(ly0+(y-y0))*8u;
      const unsigned r0=unsigned((w0>>sh)&255u),r1=unsigned((w1>>sh)&255u),r2=unsigned((w2>>sh)&255u);
      row[y]=std::uint16_t((r0>>4)|(r1<<4)|((r2&15u)<<12));}}}
   // a = eroded: z, y, x
   for(int x=0;x<16;++x){const std::uint16_t*s=solid+(x+2)*S+2;std::uint16_t*d=a+(x+2)*S+2;
    for(int y=0;y<16;++y){const unsigned v=s[y];d[y]=std::uint16_t(v&(v>>1)&(v>>2)&(v<<1)&(v<<2));}}
   for(int x=0;x<16;++x){const std::uint16_t*s=a+(x+2)*S+2;std::uint16_t*d=b+(x+2)*S+2;
    for(int y=0;y<16;++y)d[y]=std::uint16_t(s[y-2]&s[y-1]&s[y]&s[y+1]&s[y+2]);}
   for(int x=0;x<16;++x){const std::uint16_t*s=b+(x+2)*S+2;std::uint16_t*d=a+(x+2)*S+2;
    for(int y=0;y<16;++y)d[y]=std::uint16_t(s[y-2*S]&s[y-S]&s[y]&s[y+S]&s[y+2*S]);}
   // b = opened: z, y, x
   for(int x=0;x<16;++x){const std::uint16_t*s=a+(x+2)*S+2;std::uint16_t*d=b+(x+2)*S+2;
    for(int y=0;y<16;++y){const unsigned v=s[y];d[y]=std::uint16_t((v|(v>>1)|(v>>2)|(v<<1)|(v<<2))&0xFFFFu);}}
   for(int x=0;x<16;++x){const std::uint16_t*s=b+(x+2)*S+2;std::uint16_t*d=a+(x+2)*S+2;
    for(int y=0;y<16;++y)d[y]=std::uint16_t(s[y-2]|s[y-1]|s[y]|s[y+1]|s[y+2]);}
   for(int x=0;x<16;++x){const std::uint16_t*s=a+(x+2)*S+2;std::uint16_t*d=b+(x+2)*S+2;
    for(int y=0;y<16;++y)d[y]=std::uint16_t(s[y-2*S]|s[y-S]|s[y]|s[y+S]|s[y+2*S]);}
   core::Tile fresh{};std::size_t n=0;
   for(int x=0;x<8;++x){const std::uint16_t*s=solid+(x+6)*S+6,*o=b+(x+6)*S+6;
    for(int y=0;y<8;++y){const unsigned f=((unsigned(s[y])&~unsigned(o[y])&0xFFFFu)>>4)&255u;if(!f)continue;
     fresh.removed[std::size_t(x)]|=std::uint64_t(f)<<(y*8);for(unsigned r=f;r;r&=r-1)++n;}}
   if(n){const auto found=g.tiles.find(key);core::Tile t=found!=g.tiles.end()?found->second:core::Tile{};
    for(int w=0;w<8;++w)t.removed[std::size_t(w)]|=fresh.removed[std::size_t(w)];local.emplace_back(key,t);total+=n;}}
  std::lock_guard<std::mutex> lock(addMutex);adds.insert(adds.end(),local.begin(),local.end());});
 out.removed=total;lap();
 if(!adds.empty()){auto opened=std::make_shared<Grid>(g);opened->journal=nullptr;for(auto&add:adds)opened->tiles[add.first]=add.second;out.grid=std::move(opened);}
 out.ms=core::ms(t0);lap();
 if(prof)std::cerr<<"thread_tear_prof tiles="<<g.tiles.size()<<" keys="<<K<<" live="<<live.size()<<" wanted="<<wanted.size()<<" changed="<<adds.size()<<" owned="<<(own?1:0)<<" table="<<(table?1:0)<<" buckets="<<(table?table->bucket_count():0)
  <<" scan_ms="<<pms[0]<<" slots_ms="<<pms[1]<<" key_masks_ms="<<pms[2]<<" neighbours_ms="<<pms[3]<<" masks_ms="<<pms[4]<<" blocks_ms="<<pms[5]<<" grid_copy_ms="<<pms[6]<<" ms="<<out.ms<<std::endl;
 return out;
}
inline Result open(const Grid&g,const std::unordered_map<Cell,SeverState::TileMaterial,core::CellHash>*table=nullptr){
 static const bool former=[]{const char*e=std::getenv("GFY_THREAD_TEAR_FAST");return e&&std::atoi(e)==0;}();
 Result out=former?openFormer(g,table):openFast(g,table);
 static const bool verify=std::getenv("GFY_THREAD_TEAR_VERIFY")!=nullptr;
 if(verify){const auto ref=openReference(g);bool same=ref.removed==out.removed&&ref.tiles==out.tiles&&bool(ref.grid)==bool(out.grid);
  if(same&&ref.grid){same=ref.grid->tiles.size()==out.grid->tiles.size();
   for(const auto&it:ref.grid->tiles){if(!same)break;const auto o=out.grid->tiles.find(it.first);same=o!=out.grid->tiles.end()&&o->second.removed==it.second.removed;}}
  need(same,"thread_tear_mismatch");
  std::cerr<<"thread_tear_verify ok tiles="<<out.tiles<<" removed="<<out.removed<<" ms="<<out.ms<<" reference_ms="<<ref.ms<<std::endl;}
 return out;
}
}
