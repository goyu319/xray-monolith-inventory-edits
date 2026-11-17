#pragma once
// Included by LiveDamageWorker.cpp after the native Grid and connectivity
// builder. This continuation owns CPU occupancy only; it has no render mesh.
#include "RemainingConnectivity.h"

namespace gfy_live {
namespace piece_session {
// Finite, conservative domain of every piece-cell lookup issued by the
// analytic wall/cap shader on this exact published mesh. Unknown forms use
// the original full-volume producer, never a guessed crop.
struct SupportQueryBounds {
 bool available=false,empty=true;
 Vec lo{},hi{};
};
inline SupportQueryBounds supportQueryBounds(const gfy_packed::Mesh&mesh){
 SupportQueryBounds out;if(!mesh.analytic_channels)return out;
 const auto&f=*mesh.analytic_channels;
 if(f.words.size()<1+std::size_t(f.cuts)*9)return out;
 auto add=[&](const double*lo,const double*hi){
  for(unsigned a=0;a<3;++a)if(!std::isfinite(lo[a])||!std::isfinite(hi[a])||lo[a]>hi[a]||std::abs(lo[a])>100||std::abs(hi[a])>100)return false;
  if(out.empty){out.lo={lo[0],lo[1],lo[2]};out.hi={hi[0],hi[1],hi[2]};out.empty=false;}
  else{out.lo={std::min(out.lo.x,lo[0]),std::min(out.lo.y,lo[1]),std::min(out.lo.z,lo[2])};out.hi={std::max(out.hi.x,hi[0]),std::max(out.hi.y,hi[1]),std::max(out.hi.z,hi[2])};}return true;
 };
 unsigned cachedWall=~0u;double cachedLo[3]{},cachedHi[3]{};
 for(const auto&child:mesh.children){const auto*owners=f.child(child.child_index);if(!owners)continue;
  if(child.indices.size()%3||owners->count!=child.indices.size()/3||owners->data.size()<owners->count)return SupportQueryBounds{};
  for(std::size_t t=0;t<owners->count;++t){const auto owner=owners->data[t][0],kind=owners->data[t][1];
   if(owner>=f.cuts)return SupportQueryBounds{};if(kind>=3)continue;
   const auto*w=&f.words[1+std::size_t(owner)*9];const double mode=w[6][3];
   if(mode>=3)continue;if(mode!=0&&mode!=1&&mode!=2)return SupportQueryBounds{};
   if(kind==0&&owner==cachedWall){if(!add(cachedLo,cachedHi))return SupportQueryBounds{};continue;}
   for(unsigned j=0;j<9;++j)for(unsigned a=0;a<4;++a)if(!std::isfinite(w[j][a])||std::abs(w[j][a])>1e6f)return SupportQueryBounds{};
   const double e[3]={w[0][0],w[0][1],w[0][2]},d[3]={w[1][0],w[1][1],w[1][2]},u[3]={w[2][0],w[2][1],w[2][2]},v[3]={w[3][0],w[3][1],w[3][2]};
   double dn=0,un=0,vn=0;for(unsigned a=0;a<3;++a){dn+=d[a]*d[a];un+=u[a]*u[a];vn+=v[a]*v[a];}
   if(dn<.81||dn>1.21||un<.81||un>1.21||vn<.81||vn>1.21||w[1][3]<=0||w[3][3]<0)return SupportQueryBounds{};
   double lo[3]={1e300,1e300,1e300},hi[3]={-1e300,-1e300,-1e300};
   if(kind){
    // Cap reconstruction is affine in interpolatedRest, so triangle corner
    // extrema enclose every fragment, including caps outside the bore disc.
    const double end=kind==1?-double(w[3][3]):double(w[1][3]);
    for(unsigned j=0;j<3;++j){const auto ix=child.indices[t*3+j];if(ix>=child.vertices.size())return SupportQueryBounds{};const auto&p=child.vertices[ix].surface;
     double dotq=0;for(unsigned a=0;a<3;++a){if(!std::isfinite(p[a]))return SupportQueryBounds{};dotq+=(double(p[a])-e[a])*d[a];}
     for(unsigned a=0;a<3;++a){const double q=double(p[a])+d[a]*(end-dotq);lo[a]=std::min(lo[a],q);hi[a]=std::max(hi[a],q);}}
   }else{
    double radius=0;
    const double off=std::floor(double(w[6][2]));if(off<0||off>double(f.profiles.size()))return SupportQueryBounds{};
    auto knot=[&](unsigned i,double&dest){if(off+i>=double(f.profiles.size()))return false;const double z=f.profiles[std::size_t(off)+i];if(!std::isfinite(z)||std::abs(z)>1e6)return false;dest=std::max(dest,std::abs(z));return true;};
    if(mode==0){
     // Saturated flare, squared sine swell, crack ramp and all linearly
     // interpolated profile knots. This bounds absolute radius, not merely
     // the removed volume, using the stored frame without renormalization.
     const double flare=w[4][0]>=0?std::abs(double(w[4][1])*(1+double(w[5][1]))-1):0;
     radius=std::abs(double(w[0][3]))*(1+flare+std::abs(double(w[4][2])));
     if(w[6][0]!=0)radius*=1+std::abs(double(w[5][1]));
     if(w[6][1]!=0){double profile=0;for(unsigned i=0;i<256;++i)if(!knot(i,profile))return SupportQueryBounds{};radius*=profile;}
     if(w[4][0]<0&&!(w[2][3]>0))return SupportQueryBounds{};
    }else if(mode==1)radius=std::hypot(double(w[0][3]),double(w[5][3]));
    else{double face=0,shear=0;for(unsigned j=0;j<9;++j){if(!knot(3*j,face)||!knot(3*j+1,face)||!knot(3*j+2,shear))return SupportQueryBounds{};}const double width=std::abs(double(w[5][3]));radius=std::hypot(face+shear*width,width);}
    if(!std::isfinite(radius)||radius>10)return SupportQueryBounds{};
    for(unsigned a=0;a<3;++a){const double x=e[a]-d[a]*double(w[3][3]),y=e[a]+d[a]*double(w[1][3]);const double r=radius*std::hypot(u[a],v[a]);lo[a]=std::min(x,y)-r;hi[a]=std::max(x,y)+r;}
   }
   // All ten R72 backing sites are at most 6.6mm away (stored cap normal
   // may have length 1.1). A 2mm + relative guard covers float reconstruction
   // and address conversion. The producer separately includes neighbor cells.
   for(unsigned a=0;a<3;++a){const double pad=.0086+1e-4*std::max(std::abs(lo[a]),std::abs(hi[a]));lo[a]-=pad;hi[a]+=pad;}
   if(!add(lo,hi))return SupportQueryBounds{};
   if(kind==0){cachedWall=owner;for(unsigned a=0;a<3;++a){cachedLo[a]=lo[a];cachedHi[a]=hi[a];}}
  }
 }
 out.available=true;return out;
}

inline std::vector<painter_detail::PieceCellSupport> keptCellSupport(const gfy_connectivity::Topology&top,const Grid&grid,
    const std::vector<int>*pieceOf,std::size_t pieces,const gfy_packed::Mesh*queryMesh=nullptr){
 try{
  const auto begin=core::Clock::now();
  const auto* const cancelFlag=sep::cancelFlag();
  const auto cancel=[cancelFlag](){if(cancelFlag&&cancelFlag->load(std::memory_order_relaxed))throw std::runtime_error("separation_canceled");};
  const bool serial=std::getenv("GFY_CELL_SUPPORT_SERIAL")!=nullptr;
  const bool profile=std::getenv("GFY_CELL_SUPPORT_PROFILE")!=nullptr;
  if(!pieces||pieces>12||!(grid.pitch>0))return {};
  const unsigned TX=unsigned((grid.size.x+7)/8),TY=unsigned((grid.size.y+7)/8),TZ=unsigned((grid.size.z+7)/8);
  const std::uint64_t slots64=std::uint64_t(pieces)*TX*TZ;if(!TX||!TY||!TZ||slots64>(1u<<22))return {};
  const bool full=serial||!queryMesh||std::getenv("GFY_CELL_SUPPORT_FULL");
  const auto query=full?SupportQueryBounds{}:supportQueryBounds(*queryMesh);
  int cropLo[3]={0,0,0},cropHi[3]={grid.size.x-1,grid.size.y-1,grid.size.z-1};
  if(query.available){
   if(query.empty){std::vector<painter_detail::PieceCellSupport> none(pieces);for(auto&p:none)p.admitted=p.dilated=true;return none;}
   const double qlo[3]={query.lo.x,query.lo.y,query.lo.z},qhi[3]={query.hi.x,query.hi.y,query.hi.z},origin[3]={grid.origin.x,grid.origin.y,grid.origin.z};
   for(unsigned a=0;a<3;++a){cropLo[a]=std::max(0,int(std::floor((qlo[a]-origin[a])/grid.pitch))-8)&~7;cropHi[a]=std::min(cropHi[a],(int(std::floor((qhi[a]-origin[a])/grid.pitch))+8)|7);}
  }
  struct BandBrick {painter_detail::PieceCellBrick value;std::size_t piece;const core::Tile*removed;};
  // The former scan: one step per column and per 8-cell tile segment of every span (18 M steps for a whole body, 44 ms).
  // Kept as the reference (GFY_CELL_SUPPORT_SCAN=0 runs it alone, GFY_CELL_SUPPORT_SCAN_VERIFY=1 runs both and compares
  // every brick) and as the path for a topology that is not in lattice order.
  const auto scanFormer=[&](std::vector<painter_detail::PieceCellSupport>&result)->bool{
   std::vector<int> slots(std::size_t(slots64),-1);std::vector<std::size_t> touched;
   std::vector<BandBrick> band;
   std::uint64_t work=0;std::size_t logicalBytes=0;constexpr std::uint64_t workLimit=64000000;
   auto flush=[&](){
    for(const auto&b:band){bool all=true,any=false;for(auto word:b.value.bits){all&=word==~std::uint64_t(0);any|=word!=0;}if(!any)continue;
     const auto bytes=all?sizeof(std::array<int,3>):sizeof(painter_detail::PieceCellBrick);if(bytes>(32u<<20)-logicalBytes)return false;logicalBytes+=bytes;
     if(all)result[b.piece].full.push_back(b.value.key);else result[b.piece].mixed.push_back(b.value);}
    for(auto at:touched)slots[at]=-1;touched.clear();band.clear();return true;
   };
   int currentBand=-1;
   for(const auto&column:top.columns){
    if(work++>=workLimit)return false;if((work&8191u)==0)cancel();
    if(column.x<0||column.y<0||column.x>=grid.size.x||column.y>=grid.size.y)return false;
    if(column.x<cropLo[0]||column.x>cropHi[0]||column.y<cropLo[1]||column.y>cropHi[1])continue;
    const int ty=column.y>>3;if(ty!=currentBand){cancel();if(!flush())return false;currentBand=ty;}
    for(std::size_t s=column.first;s<std::size_t(column.first)+column.count;++s){if(work++>=workLimit||s>=top.spans.size())return false;const auto&span=top.spans[s];
     const int piece=pieceOf?(span.component<pieceOf->size()?(*pieceOf)[span.component]:-1):(span.component?0:-1);if(piece<0)continue;if(std::size_t(piece)>=pieces)return false;
     if(span.lo<0||span.hi>grid.size.z||span.hi<span.lo)return false;
     for(int z=std::max(span.lo,cropLo[2]),stop=std::min(span.hi,cropHi[2]+1);z<stop;){if(work++>=workLimit)return false;if((work&8191u)==0)cancel();
      const int end=std::min(stop,((z>>3)+1)*8);const unsigned localZ=unsigned(z&7),count=unsigned(end-z);
      const std::size_t at=(std::size_t(piece)*TX+unsigned(column.x>>3))*TZ+unsigned(z>>3);
      auto index=slots[at];
      if(index<0){if(band.size()>=(16u<<20)/sizeof(BandBrick))return false;
       const Cell key{column.x>>3,currentBand,z>>3};const auto it=grid.tiles.find(key);
       BandBrick b{};b.piece=std::size_t(piece);b.value.key={key.x,key.y,key.z};b.removed=it==grid.tiles.end()?nullptr:&it->second;
       index=int(band.size());band.push_back(b);touched.push_back(at);slots[at]=index;}
      auto&b=band[std::size_t(index)];const unsigned x=unsigned(column.x&7),shift=unsigned(column.y&7)*8+localZ;
      const std::uint64_t mask=((std::uint64_t(1)<<count)-1)<<shift;
      b.value.bits[x]|=mask&~(b.removed?b.removed->removed[x]:std::uint64_t(0));z=end;
     }
    }
   }
   if(!flush())return false;
   return true;};
  // split2 (2026-11-11): the same bricks from tile RANGES. A span covers the tiles first..last of its tile column wholly
  // and at most two tiles partly. Per band of 8 grid rows: the partly covered tiles get their bits as before; the wholly
  // covered ranges are counted per (piece, tile x) row with a difference array (+1 at first, -1 after last). A tile that
  // 64 columns cover wholly is a full brick without touching its bits (most bricks of a body); a tile covered wholly by
  // fewer columns is mixed, and only those tiles get the covering columns' bits (the ranges are walked again, jumping
  // from mixed tile to mixed tile). The removed masks come from the grid's tiles bucketed by band (the former scan
  // looked one up per brick) and are applied to the finished bits: (a|b)&~r == (a&~r)|(b&~r).
  // Same bricks: under the topology's order (columns ascending by (y,x), each once; a column's spans ascending and
  // disjoint - checked here, else the former scan runs) a column covers a tile wholly with at most one span and then has
  // no other cell in it, so 64 wholly covering columns are exactly the 512 cells, and every other tile's bits are the
  // OR of the same masks as before. Same refusals: the work units are counted as before (one per column, per span and per
  // tile segment = tiles of the span), the band's brick count and the result's byte budget are the same sums. The order
  // of the bricks inside the result differs; the dilation addresses them by key (pages) and emits in page order.
  // Returns 1 scanned, 0 refused, -1 not applicable (run the former scan).
  const auto scanFast=[&](std::vector<painter_detail::PieceCellSupport>&result)->int{
   constexpr std::uint64_t workLimit=64000000;constexpr std::size_t npos=~std::size_t(0);
   if(TZ>65535u)return -1;
   const std::size_t rows=std::size_t(pieces)*TX,rowWidth=std::size_t(TZ)+1;
   struct RemovedTile{std::uint32_t tx,tz;const core::Tile*tile;};
   std::vector<std::uint32_t> removedStart(std::size_t(TY)+1,0);std::vector<RemovedTile> removedTiles;
   {const auto inside=[&](const Cell&k){return k.x>=0&&k.y>=0&&k.z>=0&&unsigned(k.x)<TX&&unsigned(k.y)<TY&&unsigned(k.z)<TZ;};
    for(const auto&it:grid.tiles)if(inside(it.first))++removedStart[std::size_t(it.first.y)+1];
    for(std::size_t y=0;y<TY;++y)removedStart[y+1]+=removedStart[y];
    removedTiles.resize(removedStart[TY]);std::vector<std::uint32_t> cursor(removedStart.begin(),removedStart.end()-1);
    for(const auto&it:grid.tiles)if(inside(it.first))removedTiles[cursor[std::size_t(it.first.y)]++]={std::uint32_t(it.first.x),std::uint32_t(it.first.z),&it.second};}
   std::vector<int> slots(std::size_t(slots64),-1);std::vector<std::size_t> touched;std::vector<std::uint8_t> cover(std::size_t(slots64),0);
   std::vector<std::int32_t> diff(rows*rowWidth,0);std::vector<std::uint16_t> nextMixed(rows*rowWidth,0);std::vector<char> rowUsed(rows,0);std::vector<std::uint32_t> usedRows;
   struct FullRange{std::uint32_t row;std::uint16_t first,last;std::uint8_t x,shift;};std::vector<FullRange> fullRanges;
   std::vector<BandBrick> band;const std::size_t bandLimit=(16u<<20)/sizeof(BandBrick);
   std::uint64_t work=0;std::size_t logicalBytes=0;int currentBand=-1;
   // The band's brick of a slot (made on first use; npos: the band holds as many bricks as the former scan allows).
   const auto brick=[&](std::size_t at,std::size_t piece,int tx,int tz)->std::size_t{auto index=slots[at];
    if(index<0){if(band.size()>=bandLimit)return npos;
     BandBrick b{};b.piece=piece;b.value.key={tx,currentBand,tz};b.removed=nullptr;index=int(band.size());band.push_back(b);touched.push_back(at);slots[at]=index;}
    return std::size_t(index);};
   const auto emit=[&](std::size_t piece,const painter_detail::PieceCellBrick&value,bool all)->bool{
    const auto bytes=all?sizeof(std::array<int,3>):sizeof(painter_detail::PieceCellBrick);if(bytes>(32u<<20)-logicalBytes)return false;logicalBytes+=bytes;
    if(all)result[piece].full.push_back(value.key);else result[piece].mixed.push_back(value);return true;};
   const auto flush=[&]()->int{
    std::size_t distinct=band.size();bool anyMixed=false;
    for(const auto row:usedRows){const std::size_t d=std::size_t(row)*rowWidth,s=std::size_t(row)*TZ;std::int32_t run=0;
     for(unsigned tz=0;tz<TZ;++tz){run+=diff[d+tz];if(run<=0)continue;if(run>64||(run==64&&slots[s+tz]>=0))return -1;
      cover[s+tz]=std::uint8_t(run);if(slots[s+tz]<0)++distinct;anyMixed=anyMixed||run<64;}
     std::uint16_t next=std::uint16_t(TZ);nextMixed[d+TZ]=next;
     for(unsigned tz=TZ;tz-->0;){const auto n=cover[s+tz];if(n>0&&n<64)next=std::uint16_t(tz);nextMixed[d+tz]=next;}}
    if(distinct>bandLimit)return 0;
    if(anyMixed)for(const auto&f:fullRanges){const std::size_t d=std::size_t(f.row)*rowWidth,s=std::size_t(f.row)*TZ;
     for(unsigned tz=nextMixed[d+f.first];tz<=f.last;tz=nextMixed[d+tz+1]){
      const auto bi=brick(s+tz,std::size_t(f.row)/TX,int(f.row%TX),int(tz));if(bi==npos)return 0;band[bi].value.bits[f.x]|=std::uint64_t(0xff)<<f.shift;}}
    if(currentBand>=0)for(std::uint32_t r=removedStart[std::size_t(currentBand)];r<removedStart[std::size_t(currentBand)+1];++r){const auto&rt=removedTiles[r];
     for(std::size_t piece=0;piece<pieces;++piece){const std::size_t s=(piece*TX+rt.tx)*TZ+rt.tz;
      if(slots[s]>=0){auto&b=band[std::size_t(slots[s])];for(unsigned x=0;x<8;++x)b.value.bits[x]&=~rt.tile->removed[x];}
      else if(cover[s]==64){const auto bi=brick(s,piece,int(rt.tx),int(rt.tz));if(bi==npos)return 0;for(unsigned x=0;x<8;++x)band[bi].value.bits[x]=~rt.tile->removed[x];}}}
    for(const auto&b:band){bool all=true,any=false;for(auto word:b.value.bits){all&=word==~std::uint64_t(0);any|=word!=0;}if(!any)continue;if(!emit(b.piece,b.value,all))return 0;}
    for(const auto row:usedRows){const std::size_t s=std::size_t(row)*TZ;painter_detail::PieceCellBrick whole;
     for(unsigned tz=0;tz<TZ;++tz)if(cover[s+tz]==64&&slots[s+tz]<0){whole.key={int(row%TX),currentBand,int(tz)};if(!emit(std::size_t(row)/TX,whole,true))return 0;}}
    for(const auto row:usedRows){std::fill(diff.begin()+std::ptrdiff_t(std::size_t(row)*rowWidth),diff.begin()+std::ptrdiff_t((std::size_t(row)+1)*rowWidth),0);
     std::fill(cover.begin()+std::ptrdiff_t(std::size_t(row)*TZ),cover.begin()+std::ptrdiff_t((std::size_t(row)+1)*TZ),std::uint8_t(0));rowUsed[row]=0;}
    usedRows.clear();for(auto at:touched)slots[at]=-1;touched.clear();band.clear();fullRanges.clear();return 1;};
   int previousX=-1,previousY=-1;
   for(const auto&column:top.columns){
    if(work++>=workLimit)return 0;if((work&8191u)==0)cancel();
    if(column.x<0||column.y<0||column.x>=grid.size.x||column.y>=grid.size.y)return 0;
    if(column.y<previousY||(column.y==previousY&&column.x<=previousX))return -1; // not the lattice order: the former scan
    previousY=column.y;previousX=column.x;
    if(column.x<cropLo[0]||column.x>cropHi[0]||column.y<cropLo[1]||column.y>cropHi[1])continue;
    const int ty=column.y>>3;if(ty!=currentBand){cancel();const int flushed=flush();if(flushed<=0)return flushed;currentBand=ty;}
    const int tx=column.x>>3;const std::uint8_t lx=std::uint8_t(column.x&7),yshift=std::uint8_t((column.y&7)*8);int previousHi=INT_MIN;
    for(std::size_t s=column.first;s<std::size_t(column.first)+column.count;++s){if(work++>=workLimit||s>=top.spans.size())return 0;const auto&span=top.spans[s];
     if(span.lo<previousHi)return -1;previousHi=std::max(previousHi,span.hi); // spans not ascending and disjoint: the former scan
     const int piece=pieceOf?(span.component<pieceOf->size()?(*pieceOf)[span.component]:-1):(span.component?0:-1);if(piece<0)continue;if(std::size_t(piece)>=pieces)return 0;
     if(span.lo<0||span.hi>grid.size.z||span.hi<span.lo)return 0;
     const int z0=std::max(span.lo,cropLo[2]),stop=std::min(span.hi,cropHi[2]+1);if(z0>=stop)continue;
     const int tzA=z0>>3,tzB=(stop-1)>>3;
     work+=std::uint64_t(tzB-tzA+1);if(work>workLimit)return 0; // one unit per tile segment, as the former loop counted
     const std::uint32_t row=std::uint32_t(std::size_t(piece)*TX+unsigned(tx));const std::size_t slotBase=std::size_t(row)*TZ;
     int first=tzA,last=tzB;
     if(z0&7){const int end=std::min(stop,(tzA+1)*8);const unsigned count=unsigned(end-z0); // partly covered first tile
      const auto bi=brick(slotBase+unsigned(tzA),std::size_t(piece),tx,tzA);if(bi==npos)return 0;
      band[bi].value.bits[lx]|=((std::uint64_t(1)<<count)-1)<<(unsigned(yshift)+unsigned(z0&7));first=tzA+1;}
     if((stop&7)&&last>=first){const unsigned count=unsigned(stop-tzB*8); // partly covered last tile
      const auto bi=brick(slotBase+unsigned(tzB),std::size_t(piece),tx,tzB);if(bi==npos)return 0;
      band[bi].value.bits[lx]|=((std::uint64_t(1)<<count)-1)<<unsigned(yshift);last=tzB-1;}
     if(first<=last){diff[std::size_t(row)*rowWidth+unsigned(first)]+=1;diff[std::size_t(row)*rowWidth+unsigned(last)+1]-=1;
      if(!rowUsed[row]){rowUsed[row]=1;usedRows.push_back(row);}
      fullRanges.push_back({row,std::uint16_t(first),std::uint16_t(last),lx,yshift});}
    }
   }
   {const int flushed=flush();if(flushed<=0)return flushed;}
   return 1;};
  std::vector<painter_detail::PieceCellSupport> result(pieces);
  {static const bool formerOnly=[]{const char*e=std::getenv("GFY_CELL_SUPPORT_SCAN");return e&&std::atoi(e)==0;}();
   static const bool verifyScan=std::getenv("GFY_CELL_SUPPORT_SCAN_VERIFY")!=nullptr;
   const int fast=formerOnly?-1:scanFast(result);
   if(fast<0)result.assign(pieces,painter_detail::PieceCellSupport{});
   const bool scanned=fast<0?scanFormer(result):fast>0;
   if(verifyScan&&fast<0&&!formerOnly)std::cerr<<"cell_support_scan_verify former_scan_used"<<std::endl;
   if(verifyScan&&fast>=0){std::vector<painter_detail::PieceCellSupport> reference(pieces);const bool referenceScanned=scanFormer(reference);
    const auto keyLess=[](const std::array<int,3>&a,const std::array<int,3>&b){return a[0]!=b[0]?a[0]<b[0]:(a[1]!=b[1]?a[1]<b[1]:a[2]<b[2]);};
    const auto brickLess=[&](const painter_detail::PieceCellBrick&a,const painter_detail::PieceCellBrick&b){return keyLess(a.key,b.key);};
    bool same=referenceScanned==scanned;std::size_t fullCount=0,mixedCount=0;
    if(same&&scanned)for(std::size_t k=0;k<pieces&&same;++k){auto a=reference[k],b=result[k];
     std::sort(a.full.begin(),a.full.end(),keyLess);std::sort(b.full.begin(),b.full.end(),keyLess);
     std::sort(a.mixed.begin(),a.mixed.end(),brickLess);std::sort(b.mixed.begin(),b.mixed.end(),brickLess);
     same=a.full.size()==b.full.size()&&a.mixed.size()==b.mixed.size();
     for(std::size_t q=0;same&&q<a.full.size();++q)same=a.full[q][0]==b.full[q][0]&&a.full[q][1]==b.full[q][1]&&a.full[q][2]==b.full[q][2];
     for(std::size_t q=0;same&&q<a.mixed.size();++q){same=a.mixed[q].key[0]==b.mixed[q].key[0]&&a.mixed[q].key[1]==b.mixed[q].key[1]&&a.mixed[q].key[2]==b.mixed[q].key[2];
      for(unsigned x=0;same&&x<8;++x)same=a.mixed[q].bits[x]==b.mixed[q].bits[x];}
     fullCount+=a.full.size();mixedCount+=a.mixed.size();}
    std::cerr<<"cell_support_scan_verify "<<(same?"ok":"MISMATCH")<<" pieces="<<pieces<<" scanned="<<scanned<<" full="<<fullCount<<" mixed="<<mixedCount<<std::endl;
    if(!same)throw std::runtime_error("piece_cell_scan_mismatch");}
   if(!scanned)return {};}
  // Serial per piece: fixed dilation scratch is never multiplied by LP's
  // parallel assembly jobs. A refused piece stays explicitly unadmitted.
  const auto dilationBegin=core::Clock::now();const double rawMs=core::ms(begin);
  auto execute=[cancelFlag](std::size_t n,auto&&body){sep::parallel(n,2048,sep::max_job_threads,[&](std::size_t first,std::size_t last,std::size_t){
   core::enterPainterThread();const sep::CancelScope scope(cancelFlag);body(first,last);});};
  std::size_t rawFull=0,rawMixed=0;
  for(auto&piece:result){rawFull+=piece.full.size();rawMixed+=piece.mixed.size();piece.admitted=true;
   painter_detail::PieceCellDilationStats stats;
   piece=serial?painter_detail::dilatePieceCellSupport(piece,cancel):painter_detail::dilatePieceCellSupport(piece,execute,cancel,&stats);
   if(profile&&!serial)std::cerr<<"cell_support_dilation piece="<<(&piece-result.data())<<" typed=1 pages="<<stats.pages<<" passes="<<stats.passes<<" logical_work="<<stats.logicalWork<<" admitted="<<piece.admitted<<std::endl;}
  if(profile)std::cerr<<"cell_support_profile serial="<<serial<<" pieces="<<pieces<<" raw_full="<<rawFull<<" raw_mixed="<<rawMixed
   <<" locality="<<query.available<<" raw_ms="<<rawMs<<" dilation_ms="<<core::ms(dilationBegin)<<" total_ms="<<core::ms(begin)<<std::endl;
#if defined(GFY_VOXEL_WORKER_TESTS)
  if(query.available&&!query.empty&&std::getenv("GFY_CELL_SUPPORT_VERIFY")){
   const auto fullResult=keptCellSupport(top,grid,pieceOf,pieces,nullptr);
   if(fullResult.size()!=result.size())throw std::runtime_error("piece_cell_locality_verify_admission");
   using Key=std::array<int,3>;using Bits=std::array<std::uint64_t,8>;
   const double qlo[3]={query.lo.x,query.lo.y,query.lo.z},qhi[3]={query.hi.x,query.hi.y,query.hi.z},origin[3]={grid.origin.x,grid.origin.y,grid.origin.z};
   int low[3],high[3];for(unsigned a=0;a<3;++a){low[a]=int(std::floor((qlo[a]-origin[a])/grid.pitch/8));high[a]=int(std::floor((qhi[a]-origin[a])/grid.pitch/8));}
   auto relevant=[&](const Key&key){for(unsigned a=0;a<3;++a)if(key[a]<low[a]||key[a]>high[a])return false;return true;};
   std::size_t checked=0;
   for(std::size_t k=0;k<pieces;++k){if(!fullResult[k].admitted||!result[k].admitted)throw std::runtime_error("piece_cell_locality_verify_refused");
    auto rows=[&](const painter_detail::PieceCellSupport&p){std::map<Key,Bits> m;Bits full;full.fill(~std::uint64_t(0));for(const auto&key:p.full)if(relevant(key))m.emplace(key,full);for(const auto&b:p.mixed)if(relevant(b.key))m.emplace(b.key,b.bits);return m;};
    const auto a=rows(fullResult[k]),b=rows(result[k]);if(a!=b)throw std::runtime_error("piece_cell_locality_verify_bits");checked+=a.size();
   }
   std::cerr<<"cell_support_query_verify pieces="<<pieces<<" nonzero_bricks="<<checked<<" result=exact"<<std::endl;
  }
#endif
  return result;
 }catch(const std::bad_alloc&){return {};}
}
}

namespace component_detail {
inline std::shared_ptr<const core::PieceOwnership> ownerFor(
    const gfy_connectivity::Topology&top,std::uint32_t id){
 need(id>0&&id<top.components.size(),"piece_component_id");
 auto owner=std::make_shared<core::PieceOwnership>();
 std::uint64_t cells=0;
 for(const auto&c:top.columns){core::OwnerColumn out{c.x,c.y,std::uint32_t(owner->body_spans.size()),0};
  for(auto i=c.first;i<c.first+c.count;++i){const auto&s=top.spans[i];if(s.component!=id)continue;
   owner->body_spans.push_back({s.lo,s.hi});cells+=std::uint64_t(s.hi-s.lo);++out.count;}
  if(out.count)owner->body_columns.push_back(out);
 }
 need(cells&&cells==top.components[id].cells,"piece_component_mismatch");
 return owner;
}
// ownerFor(top,id) and the topology of the owned grid in ONE pass (P1.2).
// For top==build(accepted), build(owned) with owned=accepted+ownerFor(top,id)
// equals `second` in every field except Topology::work:
//  * the owner body spans are post-removal spans of the same tiles, so the
//    removal clip (RemainingConnectivity.h:110-113) intersects nothing;
//  * spans of a topology never touch in a column, so appendBase merges nothing;
//  * the owner columns are visited in the same (y,x) order;
//  * the spans form one face-connected set, so every span gets label 1, with
//    the cells and bounds of top.components[id].
inline std::pair<std::shared_ptr<const core::PieceOwnership>,gfy_connectivity::Topology> ownerAndTopology(
    const gfy_connectivity::Topology&top,std::uint32_t id){
 need(id>0&&id<top.components.size(),"piece_component_id");
 auto owner=std::make_shared<core::PieceOwnership>();
 gfy_connectivity::Topology owned;owned.size=top.size;owned.origin=top.origin;owned.pitch=top.pitch;
 owned.components.push_back(top.components[id]);
 // Exact reservation (a read-only counting pass): no reallocation copies.
 {std::size_t columns=0,spans=0;
  for(const auto&c:top.columns){std::size_t n=0;for(auto i=c.first;i<c.first+c.count;++i)n+=top.spans[i].component==id;spans+=n;columns+=n!=0;}
  owner->body_columns.reserve(columns);owner->body_spans.reserve(spans);owned.columns.reserve(columns);owned.spans.reserve(spans);}
 std::uint64_t cells=0;
 for(const auto&c:top.columns){core::OwnerColumn out{c.x,c.y,std::uint32_t(owner->body_spans.size()),0};
  for(auto i=c.first;i<c.first+c.count;++i){const auto&s=top.spans[i];if(s.component!=id)continue;
   owner->body_spans.push_back({s.lo,s.hi});owned.spans.push_back({s.lo,s.hi,1});cells+=std::uint64_t(s.hi-s.lo);++out.count;}
  if(out.count){owner->body_columns.push_back(out);owned.columns.push_back({c.x,c.y,out.first,out.count});}
 }
 need(cells&&cells==top.components[id].cells,"piece_component_mismatch");
 owned.cells=cells;
 return {std::move(owner),std::move(owned)};
}
// A composite semantic owner still has the exact disconnected raw topology.
inline std::pair<std::shared_ptr<const core::PieceOwnership>,gfy_connectivity::Topology> ownerAndTopology(
 const gfy_connectivity::Topology&top,const std::vector<std::uint32_t>&ids){
 need(ids.size()>1&&std::is_sorted(ids.begin(),ids.end())&&std::adjacent_find(ids.begin(),ids.end())==ids.end(),"island_owner_ids");
 auto owner=std::make_shared<core::PieceOwnership>();gfy_connectivity::Topology owned;owned.size=top.size;owned.origin=top.origin;owned.pitch=top.pitch;
 std::vector<std::uint32_t> label(top.components.size(),0);std::uint64_t expected=0;
 for(const auto id:ids){need(id&&id<top.components.size()&&top.components[id].cells,"island_owner_component");label[id]=std::uint32_t(owned.components.size());owned.components.push_back(top.components[id]);expected+=top.components[id].cells;}
 for(const auto&col:top.columns){core::OwnerColumn row{col.x,col.y,std::uint32_t(owner->body_spans.size()),0};
  for(auto n=col.first;n<col.first+col.count;++n){const auto&sp=top.spans[n];if(!label[sp.component])continue;
   owner->body_spans.push_back({sp.lo,sp.hi});owned.spans.push_back({sp.lo,sp.hi,label[sp.component]});owned.cells+=std::uint64_t(sp.hi-sp.lo);++row.count;}
  if(row.count){owner->body_columns.push_back(row);owned.columns.push_back({row.x,row.y,row.first,row.count});}}
 need(owned.cells==expected,"island_owner_cell_conservation");return {std::move(owner),std::move(owned)};
}
inline std::shared_ptr<const Source> visibleSource(const gfy_packed::Mesh&mesh,
    std::uint32_t bones){
 need(bones&&bones<=64,"piece_visible_bone_count");
 auto source=std::make_shared<Source>();source->bones=bones;
 for(const auto&child:mesh.children){
  need(child.indices.size()%3==0,"piece_visible_indices");
  const auto offset=source->v.size();
  need(offset+child.vertices.size()<=300000,"piece_visible_vertex_budget");
  for(const auto&packed:child.vertices){
   core::SourceVertex v;const auto&s=packed.surface;
   v.p={s[0],s[1],s[2]};v.normal={s[3],s[4],s[5]};
   v.tangent={s[6],s[7],s[8]};v.binormal={s[9],s[10],s[11]};
   v.uv={s[12],s[13]};v.ids=packed.bone;v.w=packed.weight;v.authored_hardware=gfy_signed_hw::valid4(v.w,true);
   double total=0;for(unsigned k=0;k<4;++k){
    need(v.ids[k]<bones&&std::isfinite(v.w[k])&&(v.w[k]>=0||v.authored_hardware),"piece_visible_skin");
    total+=v.w[k];}
   need(std::abs(total-1)<2e-6,"piece_visible_skin_sum");
   source->v.push_back(v);
  }
  for(std::size_t i=0;i<child.indices.size();i+=3){
   std::array<std::uint32_t,3>f{};
   for(unsigned k=0;k<3;++k){need(child.indices[i+k]<child.vertices.size(),"piece_visible_face_range");
    f[k]=std::uint32_t(offset+child.indices[i+k]);}
   source->f.push_back(f);
  }
 }
 need(!source->f.empty()&&source->f.size()<=600000,"piece_visible_surface_missing");
 return source;
}
inline gfy_packed::Mesh combineMeshes(const std::vector<SeparationPiece>&pieces,
    std::uint16_t originalChildren){
 using Group=std::pair<unsigned,std::uint16_t>;
 std::map<Group,std::vector<const gfy_packed::Child*>>groups;
 std::map<const gfy_packed::Child*,const painter_detail::SkinOwnershipField*>ownershipSources;
 painter_detail::ChannelIdentity ownershipIdentity;
 for(const auto&piece:pieces)if(piece.mesh.skin_ownership){ownershipIdentity=piece.mesh.skin_ownership->identity;break;}
 painter_detail::SkinOwnershipBuilder ownership(ownershipIdentity);
 gfy_packed::Mesh out;
 for(const auto&piece:pieces){
  gfy_packed::validate_skin_ownership(piece.mesh);
  if(!out.tissue_field)out.tissue_field=piece.mesh.tissue_field;
  out.reduced_skin_vertices+=piece.mesh.reduced_skin_vertices;
  out.max_discarded_skin_weight=std::max(out.max_discarded_skin_weight,
      piece.mesh.max_discarded_skin_weight);
  for(const auto&child:piece.mesh.children){
   groups[{child.material,child.material_source_child}].push_back(&child);ownershipSources[&child]=piece.mesh.skin_ownership.get();}
 }
 unsigned next=originalChildren;std::size_t inputTriangles=0,outputTriangles=0;
 auto appended=[&](){need(next<gfy_packed::max_child_slots,"piece_combined_child_slots");
  return std::uint16_t(next++);};
 for(const auto&entry:groups){
  const auto material=entry.first.first;
  const auto sourceChild=entry.first.second;
  need(sourceChild<originalChildren,"piece_combined_source_child");
  gfy_packed::Child child;child.material=material;child.material_source_child=sourceChild;
  child.child_index=material?appended():sourceChild;
  using Key=std::array<std::uint8_t,sizeof(gfy_packed::Vertex)>;
  std::map<Key,std::uint16_t>vertices;
  auto flush=[&](){outputTriangles+=child.original_faces.size();
   out.children.push_back(std::move(child));child={};
   child.material=material;child.material_source_child=sourceChild;
   child.child_index=appended();vertices.clear();};
  for(const auto*source:entry.second){
   need(source->indices.size()==source->original_faces.size()*3,
        "piece_combined_face_provenance");
   for(std::size_t t=0;t<source->original_faces.size();++t){
    ++inputTriangles;std::array<Key,3>keys;std::array<gfy_packed::Vertex,3>rows;
    unsigned fresh=0;
    for(unsigned k=0;k<3;++k){const auto id=source->indices[3*t+k];
     need(id<source->vertices.size(),"piece_combined_index");
     rows[k]=source->vertices[id];std::memcpy(keys[k].data(),&rows[k],sizeof(rows[k]));
     if(!vertices.count(keys[k]))++fresh;}
    if(child.vertices.size()+fresh>65535||child.original_faces.size()>=65535)flush();
    for(unsigned k=0;k<3;++k){
     auto [it,inserted]=vertices.emplace(keys[k],std::uint16_t(child.vertices.size()));
     if(inserted)child.vertices.push_back(rows[k]);child.indices.push_back(it->second);}
    child.original_faces.push_back(source->original_faces[t]);if(!material)ownership.append(child.child_index,ownershipSources.at(source),source->child_index,t);
   }
  }
  outputTriangles+=child.original_faces.size();out.children.push_back(std::move(child));
 }
 gfy_packed::set_skin_ownership(out,ownership.finish());
 need(outputTriangles==inputTriangles,"piece_combined_triangle_conservation");
 return out;
}
inline void combineElement(SeparationElement&dst,const SeparationElement&src){
 need(dst.bone==src.bone&&!dst.retain_original_shape&&!src.retain_original_shape&&
      dst.mass_kg>0&&src.mass_kg>0,"piece_combined_duplicate_element_shape");
 const double mass=dst.mass_kg+src.mass_kg;
 Point center{(dst.com.x*dst.mass_kg+src.com.x*src.mass_kg)/mass,
              (dst.com.y*dst.mass_kg+src.com.y*src.mass_kg)/mass,
              (dst.com.z*dst.mass_kg+src.com.z*src.mass_kg)/mass};
 const auto shift=[&](const SeparationElement&e,unsigned row,unsigned col){
  const double d[3]={e.com.x-center.x,e.com.y-center.y,e.com.z-center.z};
  const double norm=d[0]*d[0]+d[1]*d[1]+d[2]*d[2];
  return e.mass_kg*((row==col?norm:0)-d[row]*d[col]);};
 for(unsigned row=0;row<3;++row)for(unsigned col=0;col<3;++col){const auto i=row*3+col;
  dst.inertia[i]+=src.inertia[i]+shift(dst,row,col)+shift(src,row,col);}
 dst.com=center;dst.mass_kg=mass;dst.cells+=src.cells;
 dst.boxes.insert(dst.boxes.end(),src.boxes.begin(),src.boxes.end());
 need(dst.boxes.size()<=64,"piece_combined_element_box_budget");
}
inline SeparationPiece combinePieces(std::vector<SeparationPiece>&pieces,
    std::uint32_t sourceComponent,std::uint16_t originalChildren){
 need(pieces.size()>1,"piece_combined_split_required");
 SeparationPiece out;out.component=sourceComponent;
 const auto selected=std::find_if(pieces.begin(),pieces.end(),[&](const auto&p){
  return p.component==sourceComponent;});
 need(selected!=pieces.end(),"piece_combined_source_component");
 out.root_bone=selected->root_bone;
 out.mesh=combineMeshes(pieces,originalChildren);
 std::map<std::uint16_t,SeparationElement>elements;
 std::set<std::uint16_t>accessories;
 for(const auto&piece:pieces){out.cells+=piece.cells;
  for(const auto&element:piece.elements){auto [it,inserted]=elements.emplace(element.bone,element);
   if(!inserted)combineElement(it->second,element);}
  if(out.pose_anchors.empty())out.pose_anchors=piece.pose_anchors;
  else{need(out.pose_anchors.size()==piece.pose_anchors.size(),"piece_combined_pose_palette");
   for(std::size_t i=0;i<piece.pose_anchors.size();++i){const auto anchor=piece.pose_anchors[i];
    if(anchor==65535)continue;
    need(out.pose_anchors[i]==65535||out.pose_anchors[i]==anchor,
         "piece_combined_pose_anchor_conflict");
    out.pose_anchors[i]=anchor;}}
  accessories.insert(piece.retained_accessories.begin(),piece.retained_accessories.end());
 }
 for(auto&entry:elements)out.elements.push_back(std::move(entry.second));
 out.retained_accessories.assign(accessories.begin(),accessories.end());
 return out;
}
}

struct ComponentSession::Impl {
 mutable std::mutex mutex;
 const Stamp source_stamp;
 const std::uint32_t source_component;
 const std::shared_ptr<const Asset> asset;
 const std::shared_ptr<const Grid> accepted;
 Grid current;
 gfy_connectivity::Topology topology;
 std::uint64_t revision{};
 Identity owner;std::uint64_t generation{},sequence{};
 bool bound{},pending{},canceled{};
 std::shared_ptr<const gfy_packed::Mesh>published_mesh;
 std::shared_ptr<const Source>published_source;
 std::shared_ptr<const gfy_packed::SkinOwnershipQuery>published_ownership;
 // build(*accepted) when known (P1.2/P1.7; shared by every piece of one
 // separation). Null: prepare_owned_separation builds it as before.
 std::shared_ptr<const gfy_connectivity::Topology>accepted_topology;
 struct Ready {
  Grid grid;gfy_connectivity::Topology topology;
  std::shared_ptr<const gfy_packed::Mesh>mesh;
  std::shared_ptr<const Source>source;
  std::shared_ptr<const gfy_packed::SkinOwnershipQuery>ownership;
  ComponentReceipt receipt;
  bool changed{},failed{};
 };
 std::shared_ptr<Ready>ready;
 Impl(Stamp stamp,std::uint32_t root,std::shared_ptr<const Asset>source,
      std::shared_ptr<const Grid>snapshot,Grid owned)
  :source_stamp(std::move(stamp)),source_component(root),asset(std::move(source)),accepted(std::move(snapshot)),
   current(std::move(owned)),topology(gfy_connectivity::build(current)){}
 // `known` must equal build(owned) in every field except work (P1.2/P1.7).
 Impl(Stamp stamp,std::uint32_t root,std::shared_ptr<const Asset>source,
      std::shared_ptr<const Grid>snapshot,Grid owned,gfy_connectivity::Topology known,
      std::shared_ptr<const gfy_connectivity::Topology>acceptedTopology={})
  :source_stamp(std::move(stamp)),source_component(root),asset(std::move(source)),accepted(std::move(snapshot)),
   current(std::move(owned)),topology(std::move(known)),accepted_topology(std::move(acceptedTopology)){
#ifdef GFY_SEP_VERIFY
  need(separation_detail::sameTopology(gfy_connectivity::build(current),topology),"piece_known_topology_mismatch");
  need(!accepted_topology||(accepted&&separation_detail::sameTopology(gfy_connectivity::build(*accepted),*accepted_topology)),"piece_accepted_topology_mismatch");
#endif
 }
};
namespace component_detail {
// Also used by the direct CPU regression, which constructs an accepted grid
// after exact native test cuts without asking the renderer to publish it.
inline std::shared_ptr<ComponentSession> make(std::shared_ptr<const Grid>accepted,
    const Stamp&stamp,std::uint32_t component,std::uint32_t root=0,
    std::shared_ptr<const Asset>asset={}){
 need(bool(accepted),"piece_snapshot_required");
 const auto top=gfy_connectivity::build(*accepted);
 auto owner=ownerFor(top,component);
 Grid grid=*accepted;grid.ownership=std::move(owner);
 auto state=std::make_shared<ComponentSession::Impl>(stamp,root?root:component,std::move(asset),
    std::move(accepted),std::move(grid));
 need(state->topology.cells==top.components[component].cells,"piece_initial_topology_mismatch");
 return std::make_shared<ComponentSession>(std::move(state));
}
// make() with the accepted grid's topology already known (P1.2): acceptedTop
// must equal build(*accepted) except work. No connectivity is rebuilt: the
// owner and the owned topology come from one pass over acceptedTop. The
// statement order, checks and messages are those of make().
inline std::shared_ptr<ComponentSession> make(std::shared_ptr<const Grid>accepted,
    const Stamp&stamp,std::uint32_t component,std::uint32_t root,
    std::shared_ptr<const Asset>asset,std::shared_ptr<const gfy_connectivity::Topology>acceptedTop,
    bool keepAcceptedTopology=true){
 if(!acceptedTop)return make(std::move(accepted),stamp,component,root,std::move(asset));
 need(bool(accepted),"piece_snapshot_required");
#ifdef GFY_RX_PROF
 auto rxT=core::Clock::now();auto rxMark=[&](const char*n){std::cerr<<"rxprof make."<<n<<' '<<core::ms(rxT)<<std::endl;rxT=core::Clock::now();};
#else
 auto rxMark=[](const char*){};
#endif
 const auto&top=*acceptedTop;
 auto owned=ownerAndTopology(top,component);rxMark("owner");
 Grid grid=*accepted;grid.ownership=std::move(owned.first);rxMark("grid_copy");
#ifndef GFY_SEP_DERIVED_CONNECTIVITY_BUDGETS
 // The Impl constructor of make() would build(owned grid); when that build
 // would fail a budget, run make() so it fails exactly as it does in base.
 // The owner is a component of acceptedTop == build(*accepted) and grid has
 // *accepted's tiles, so no owner span holds a removed cell (budget shortcut).
 if(!gfy_connectivity::referenceOwnedBuildBudget(grid,owned.second,gfy_connectivity::unknownUnionLabelWork,false,{},true).accepted)
  return make(std::move(accepted),stamp,component,root,std::move(asset));
#endif
 if(!keepAcceptedTopology)acceptedTop.reset();
 auto state=std::make_shared<ComponentSession::Impl>(stamp,root?root:component,std::move(asset),
    std::move(accepted),std::move(grid),std::move(owned.second),std::move(acceptedTop));rxMark("impl");
 need(state->topology.cells==top.components[component].cells,"piece_initial_topology_mismatch");
 return std::make_shared<ComponentSession>(std::move(state));
}
}
namespace component_detail {
// build(g) in every field but work, with build(g)'s failures, from parent =
// build(g0) of a grid g0 with the same base (anatomy object, or ownership) and
// lattice whose removed cells are a subset of g's (P1.7: piece hits, virtual
// parents). derived=true when parentSpans (optional) name each span's parent
// span; otherwise build(g) itself ran. Both owned and verified full-body
// grids charge derive()'s actual coalesced work against the unchanged limits.
// Failure behavior can differ when the old builder overcounts adjacent
// removal runs; topology does not change.
inline gfy_connectivity::Topology exactBuild(const gfy_connectivity::Topology&parent,const Grid&g,
    const Asset*asset,const SeparationInput*input,std::vector<std::uint32_t>*parentSpans,bool&derived){
 derived=false;
 const bool lattice=parent.pitch==g.pitch&&parent.size==std::array<int,3>{g.size.x,g.size.y,g.size.z}&&
  parent.origin==std::array<double,3>{g.origin.x,g.origin.y,g.origin.z};
 if(lattice&&(g.ownership||(g.anatomy&&asset&&input))){
  std::uint64_t unionLabel=0;bool accept=false;gfy_connectivity::Topology result;
#ifdef GFY_SEP_DERIVED_CONNECTIVITY_BUDGETS
  result=gfy_connectivity::derive(parent,g,parentSpans,{},&unionLabel);accept=true;
#else
  if(g.ownership){
   // Use the exact coalesced algorithm's own unchanged work/span/run caps.
   // Counting the old builder's unmerged 8-cell runs rejects valid results
   // after repeated wounds even when this algorithm is within every cap.
   result=gfy_connectivity::derive(parent,g,parentSpans,{},&unionLabel);
   accept=true;
  }else{
   separation_detail::PristineRef reference;bool have=true;
   try{reference=separation_detail::pristineReference(*asset,*input);}catch(...){have=false;}
   if(have&&separation_detail::derivableFromPristine(reference,g)){
    result=gfy_connectivity::derive(parent,g,parentSpans,{},&unionLabel);
    accept=true;}
  }
#endif
  if(accept){derived=true;
#ifdef GFY_SEP_VERIFY
   // Verification only; production derivation remains bounded above.
   need(separation_detail::sameTopology(gfy_connectivity::build(g,gfy_connectivity::unlimitedLimits()),result),"piece_derived_topology_mismatch");
#endif
   return result;}
 }
 if(parentSpans)parentSpans->clear();
 return gfy_connectivity::build(g);
}
}
ComponentSession::ComponentSession(std::shared_ptr<Impl>state):impl_(std::move(state)){}
const Stamp&ComponentSession::source_stamp()const noexcept{return impl_->source_stamp;}
std::uint32_t ComponentSession::source_component()const noexcept{return impl_->source_component;}
std::uint64_t ComponentSession::local_revision()const noexcept{
 std::lock_guard<std::mutex>lock(impl_->mutex);return impl_->revision;
}
DamageSample ComponentSession::sample_rest(Point p)const noexcept{
 std::lock_guard<std::mutex>lock(impl_->mutex);const auto&g=impl_->current;const Vec v=vec(p);Cell c;
 for(int a=0;a<3;++a){double value=(v[a]-g.origin[a])/g.pitch;
  if(!std::isfinite(value)||value<0||value>=g.size[a])return {};
  (&c.x)[a]=int(std::floor(value));}
 auto label=g.materialBase(c);return {label,std::uint16_t(label?g.renderMaterial(c):0),g.removed(c)};
}
std::size_t ComponentSession::sparse_tiles()const noexcept{
 std::lock_guard<std::mutex>lock(impl_->mutex);return impl_->current.tiles.size();
}
bool ComponentSession::initialize_published_surface(const gfy_packed::Mesh&mesh)noexcept{
 AllocationScope scope;
 try{need(bool(impl_->asset),"piece_visible_asset_required");
  auto copy=std::make_shared<const gfy_packed::Mesh>(mesh);
  auto source=component_detail::visibleSource(*copy,impl_->asset->source.bones);
  auto ownership=gfy_packed::skin_ownership_query(*copy,false);
  std::lock_guard<std::mutex>lock(impl_->mutex);
  need(!impl_->bound&&!impl_->pending&&!impl_->published_source,"piece_visible_initialization_once");
  impl_->published_mesh=std::move(copy);impl_->published_source=std::move(source);impl_->published_ownership=std::move(ownership);
  return true;
 }catch(const std::exception&e){fail(e.what());return false;}
 catch(...){fail("unknown_piece_visible_initialization_failure");return false;}
}
bool ComponentSession::can_bind_owner(Identity owner,std::uint64_t generation)const noexcept{
 std::lock_guard<std::mutex>lock(impl_->mutex);
 return !impl_->bound&&!impl_->canceled&&!impl_->pending&&!impl_->ready&&
    bool(impl_->asset)&&bool(impl_->published_mesh)&&bool(impl_->published_source)&&
    owner.spawn_cookie&&owner.visual_lineage&&generation>0&&
    owner.source==impl_->source_stamp.owner.source&&
    owner.exact_rig==impl_->source_stamp.owner.exact_rig;
}
bool ComponentSession::bind_owner(Identity owner,std::uint64_t generation)noexcept{
 try{std::lock_guard<std::mutex>lock(impl_->mutex);
  need(!impl_->bound&&!impl_->canceled&&!impl_->pending&&!impl_->ready&&
       bool(impl_->asset)&&bool(impl_->published_mesh)&&bool(impl_->published_source),
       "piece_bind_requires_published_surface");
  need(owner.spawn_cookie&&owner.visual_lineage&&generation>0&&
       owner.source==impl_->source_stamp.owner.source&&
       owner.exact_rig==impl_->source_stamp.owner.exact_rig,
       "piece_bound_owner_identity");
  impl_->owner=std::move(owner);impl_->generation=generation;impl_->bound=true;
  return true;
 }catch(const std::exception&e){fail(e.what());return false;}
 catch(...){fail("unknown_piece_bind_failure");return false;}
}
void ComponentSession::cancel()noexcept{
 std::lock_guard<std::mutex>lock(impl_->mutex);
 impl_->canceled=true;impl_->pending=false;impl_->ready.reset();
}
std::vector<ComponentRemaining> ComponentSession::remaining_components()const{
 std::lock_guard<std::mutex>lock(impl_->mutex);std::vector<ComponentRemaining>out;
 for(std::uint32_t id=1;id<impl_->topology.components.size();++id)
  out.push_back({id,impl_->topology.components[id].cells});
 return out;
}
ComponentHit ComponentSession::hit_rest(Point origin,Point direction,double radius,double penetration)noexcept{
 AllocationScope scope;
 std::lock_guard<std::mutex>lock(impl_->mutex);ComponentHit out;out.source_stamp=impl_->source_stamp;
 out.local_revision=impl_->revision;
 try{
  need(!impl_->bound&&!impl_->pending&&!impl_->canceled,
       "piece_direct_hit_after_owner_binding");
  Grid candidate=impl_->current;std::size_t edge=0;
  auto hit=eraseTunnel(candidate,vec(origin),vec(direction),radius,penetration,&edge);
  if(!hit.freshSolid)return out;
  need(candidate.tiles.size()<=262144,"owner_sparse_tile_budget");
  // P1.7: derived from the session topology (= build(current)); compare()
  // cannot fail for a derived result (every span lies in its parent span).
  bool derived=false;
  auto after=component_detail::exactBuild(impl_->topology,candidate,impl_->asset.get(),nullptr,nullptr,derived);
  if(!derived)(void)gfy_connectivity::compare(impl_->topology,after);
  need(after.cells<=impl_->topology.cells&&
      impl_->topology.cells-after.cells==hit.freshSolid,"piece_exact_removed_volume");
  impl_->current=std::move(candidate);impl_->topology=std::move(after);
  out.changed=true;out.removed_solid_cells=hit.freshSolid;
  out.removed_mask_cells=hit.freshMask;out.edge_contact_cells=edge;
  out.local_revision=++impl_->revision;
 }catch(const std::exception&e){out.error=e.what();}catch(...){out.error="unknown_component_hit_failure";}
 return out;
}
std::shared_ptr<ComponentSession>ComponentSession::prepare_descendant(std::uint32_t component)const noexcept{
 AllocationScope scope;
 try{std::lock_guard<std::mutex>lock(impl_->mutex);
  // P1.7: impl_->topology is build(current); the owned grid's accepted
  // topology is not retained (an owned accepted grid cannot re-separate).
  return component_detail::make(std::make_shared<const Grid>(impl_->current),
      impl_->source_stamp,component,impl_->source_component,impl_->asset,
      std::make_shared<const gfy_connectivity::Topology>(impl_->topology),false);
 }catch(const std::exception&e){fail(e.what());return {};}catch(...){fail("unknown_component_descendant_failure");return {};}
}
namespace component_detail {
// DamageSnapshot::prepare_component_session for an explicit snapshot state.
inline std::shared_ptr<ComponentSession> snapshotSession(const std::shared_ptr<const Grid>&grid,const Stamp&stamp,
    std::uint32_t component,const std::shared_ptr<const Asset>&asset,
    const std::shared_ptr<const gfy_connectivity::Topology>&gridTopology={})noexcept{
 try{return make(grid,stamp,component,0,asset,gridTopology);}
 catch(const std::exception&e){fail(e.what());return {};}
 catch(...){fail("unknown_component_session_failure");return {};}
}
}
std::shared_ptr<ComponentSession>DamageSnapshot::prepare_component_session(std::uint32_t component)const noexcept{
 AllocationScope scope;
 return component_detail::snapshotSession(impl_->grid,impl_->stamp,component,impl_->asset);
}
// speedfix: set only by the GFY_SF_PURE_VERIFY check (the former per-tile loop for its reference walk)
inline bool& sfPureRangesOff(){thread_local bool off=false;return off;}
inline std::shared_ptr<SeparationOutput> separation_detail::prepareSnapshot(const std::shared_ptr<const Asset>&asset,
    const std::shared_ptr<const Grid>&grid,const gfy_packed::Mesh*geometry,const Stamp&stamp,
    const SeparationInput&input,const SeparationCheckpoint&checkpoint,SnapshotTiming*timing,const PieceContext*context){
 PrepareExtras extras;extras.published=input.published_surface?geometry:nullptr;extras.lowpoly=geometry;
 extras.piece=context?context->piece:nullptr;
 extras.gore=context?context->gore:GoreControls{};
 std::shared_ptr<const Grid> built;std::shared_ptr<const gfy_connectivity::Topology> builtVolume;extras.grid_out=&built;extras.pieces_volume_out=&builtVolume;
 // P1.2: keep the remaining-volume topology prepare() used; it is build(*grid),
 // so every piece continuation reuses it instead of rebuilding it twice.
 // split2 part 11 (2026-11-12): only the component-session continuation below (GFY_COMPONENT_SESSIONS) reads `volume`; the
 // low-poly continuation cuts its seeds from builtVolume. Without this second reference prepare() releases the topology it
 // derived before the debris filter (about 21 MB for a body) when the filtered one replaces it, not at the end of the job.
 // GFY_SEP_KEEP_VOLUME=1: keep it as before.
 const bool keepVolume=std::getenv("GFY_COMPONENT_SESSIONS")!=nullptr||std::getenv("GFY_SEP_KEEP_VOLUME")!=nullptr;
 std::shared_ptr<const gfy_connectivity::Topology>volume;if(keepVolume)extras.volume_out=&volume;
 // split2 (2026-11-12): every piece's seed needs component_detail::ownerAndTopology(piece volume, component) - two passes
 // over every span of the remaining volume, 13-14 ms for a body piece, formerly after prepare(). It is a pure function of
 // the volume and the component: prepare() reports them as soon as they are final (piece_volume_ready) and the owners are
 // computed on up to two helper threads beside the surface partition. The seed loop below takes an owner only when it was
 // computed from the very volume object the seeds are cut from; anything else is computed there as before.
 // GFY_SEP_EARLY_OWNERS=0 switches this off.
 struct EarlyOwners {
  using Owned=std::pair<std::shared_ptr<const core::PieceOwnership>,gfy_connectivity::Topology>;
  struct Task {std::uint32_t component{};Owned value;bool done{},taken{};};
  std::shared_ptr<const gfy_connectivity::Topology> volume;std::vector<Task> tasks;std::atomic<std::size_t> cursor{0};std::atomic<bool> stop{false};std::vector<std::thread> workers;
  // A failed owner stays not done: the seed loop computes it and reports the failure where it always did.
  void work(){for(std::size_t k;!stop.load(std::memory_order_relaxed)&&(k=cursor.fetch_add(1))<tasks.size();){auto&t=tasks[k];
    try{t.value=component_detail::ownerAndTopology(*volume,t.component);t.done=true;}catch(...){}}
   // speedfix: then the columns of every component (built once, by whichever thread gets here first)
   bool expected=false;if(!stop.load(std::memory_order_relaxed)&&indexClaimed.compare_exchange_strong(expected,true)){
    try{columnsOf.assign(volume->components.size(),{});
     for(std::size_t c=0;c<volume->columns.size();++c){const auto&col=volume->columns[c];std::uint32_t last=0;
      for(auto s=col.first;s<col.first+col.count;++s){const auto id=volume->spans[s].component;
       if(id&&id<columnsOf.size()&&id!=last){auto&list=columnsOf[id];if(list.empty()||list.back()!=std::uint32_t(c))list.push_back(std::uint32_t(c));last=id;}}}
     indexDone=true;}catch(...){columnsOf.clear();}}}
  // speedfix: per component, the indices of the volume columns holding a span of it (ascending; see the seed loop)
  std::vector<std::vector<std::uint32_t>> columnsOf;std::atomic<bool> indexClaimed{false};bool indexDone{false};
  void join(){for(auto&w:workers)if(w.joinable())w.join();}
  ~EarlyOwners(){stop.store(true);join();}};
 std::unique_ptr<EarlyOwners> earlyOwners;
 static const bool earlyOwnersOn=[]{const char*e=std::getenv("GFY_SEP_EARLY_OWNERS");return !e||std::atoi(e)!=0;}();
 if(earlyOwnersOn&&!std::getenv("GFY_COMPONENT_SESSIONS"))extras.piece_volume_ready=[&earlyOwners](const std::shared_ptr<const gfy_connectivity::Topology>&volume,const std::vector<std::uint32_t>&components){
  earlyOwners.reset(); // a second prepare() (the thread-tear grid) has its own volume
  if(!volume||components.empty())return;
  auto early=std::make_unique<EarlyOwners>();early->volume=volume;
  for(const auto id:components)if(id>0&&id<volume->components.size()){EarlyOwners::Task task;task.component=id;early->tasks.push_back(std::move(task));}
  // larger pieces first; two helpers (the surface partition and the collider fits run beside them)
  std::stable_sort(early->tasks.begin(),early->tasks.end(),[&](const EarlyOwners::Task&a,const EarlyOwners::Task&b){return volume->components[a.component].cells>volume->components[b.component].cells;});
  const std::size_t helpers=std::min<std::size_t>(early->tasks.size(),2);early->workers.reserve(helpers);
  earlyOwners=std::move(early);auto*e=earlyOwners.get();
  try{for(std::size_t h=0;h<helpers;++h)e->workers.emplace_back([e]{core::enterPainterThread();e->work();});}catch(...){}};
 const auto started=core::Clock::now();
 auto out=prepare(*asset,*grid,stamp,input,nullptr,checkpoint,extras);
 // Thin bridges tear (ThreadTear.h): a body still joined only by tissue threads thinner than ~5 mm inside the
 // damage splits there, when at least two of the pieces are large.
 std::shared_ptr<const Grid> used=grid;
 if(out->no_separation&&asset&&geometry&&!std::getenv("GFY_NO_THREAD_TEAR")){const auto opened=thread_tear::open(*grid,asset->tile_material.get());
  if(opened.grid){std::shared_ptr<const gfy_connectivity::Topology> openedVolume;PrepareExtras again=extras;again.thread_tear_retry_m3=thread_tear::minimum_piece_m3; /* gfyspeed3 */if(keepVolume)again.volume_out=&openedVolume;
   std::shared_ptr<const Grid> openedBuilt;std::shared_ptr<const gfy_connectivity::Topology> openedPieces;again.grid_out=&openedBuilt;again.pieces_volume_out=&openedPieces;
   std::shared_ptr<SeparationOutput> split;
   try{split=prepare(*asset,*opened.grid,stamp,input,nullptr,checkpoint,again);}
   catch(...){if(sep::cancelRequested())throw;split.reset();}
   std::size_t large=0;
   if(split&&!split->no_separation&&split->error.empty())for(const auto&p:split->pieces)
    large+=double(p.cells)*grid->pitch*grid->pitch*grid->pitch>=thread_tear::minimum_piece_m3?1:0;
   if(large>=2){out=split;volume=openedVolume;used=opened.grid;built=openedBuilt;builtVolume=openedPieces;}
   out->lowpoly_summary+=" thread_tear_cells="+std::to_string(opened.removed)+" thread_tear_tiles="+std::to_string(opened.tiles)+
    " thread_tear_ms="+std::to_string(int(opened.ms))+(large>=2?" thread_tear=split":" thread_tear=held");}}
 // split (2026-11-11): the next separation checks of a piece found still connected use the tile verdict
 // (LiveSeparationWorker_impl.h PieceTileBases); its graph is built on the service's thread, asked for here, after the check.
 if(out->no_separation&&extras.piece&&grid&&pieceTileVerdictApplies(extras.piece,*grid)){try{pieceTileBases().request(extras.piece->topology);}catch(...){}}
 if(!out->no_separation&&!extras.piece)prefitDrop(stamp.owner); // speedfix: the body separates; its fits ahead are not wanted any more
 if(timing){timing->prepare_ms=core::ms(started);timing->output=out;}
 const auto continued=core::Clock::now();
 if(!out->no_separation&&!std::getenv("GFY_COMPONENT_SESSIONS")){
  // Low-poly continuation (PieceSeed.h): each piece's seed gets the grid its mesh was cut from, restricted to the
  // piece (ownership) and cropped to its surroundings, the piece's topology, the cut history and the torn stations.
  if(timing)timing->piece_ms.assign(out->pieces.size(),0.);
  const Grid&base=built?*built:*used;need(bool(builtVolume),"separation_piece_volume_missing");
  std::vector<std::exception_ptr> failed(out->pieces.size());
  // split2 part 6: the owners started beside the surface partition, when they are of this volume object. What is left of
  // them is finished here; each piece gets its own (one task per component, handed out once, before the threads start).
  std::vector<EarlyOwners::Task*> earlyOwner(out->pieces.size(),nullptr);
  const std::vector<std::vector<std::uint32_t>>* foreignColumns=nullptr; // speedfix: EarlyOwners::columnsOf of this volume
  static const bool sfProf=std::getenv("GFY_SEP_PROF")!=nullptr; // speedfix read-out: the continuation's parts
  const auto ownersWait=core::Clock::now();std::vector<double> sfPieceMs(out->pieces.size(),0.);
  if(earlyOwners&&earlyOwners->volume.get()==builtVolume.get()){earlyOwners->work();earlyOwners->join();
   if(earlyOwners->indexDone&&earlyOwners->columnsOf.size()==builtVolume->components.size())foreignColumns=&earlyOwners->columnsOf;
   for(std::size_t i=0;i<out->pieces.size();++i){const auto&piece=out->pieces[i];if(!piece.seed||!piece.seed->components.empty())continue;
    for(auto&task:earlyOwners->tasks)if(task.component==piece.component&&task.done&&!task.taken){task.taken=true;earlyOwner[i]=&task;break;}}}
  sep::parallel(out->pieces.size(),1,sep::max_job_threads,[&](std::size_t first,std::size_t last,std::size_t){
   for(std::size_t i=first;i<last;++i)try{auto&piece=out->pieces[i];const auto pieceStarted=core::Clock::now();
    auto seed=std::const_pointer_cast<PieceSeed>(piece.seed);need(bool(seed),"separation_piece_seed_missing");
    auto owned=earlyOwner[i]?std::move(earlyOwner[i]->value):(seed->components.empty()?component_detail::ownerAndTopology(*builtVolume,piece.component):component_detail::ownerAndTopology(*builtVolume,seed->components));
    Grid g;g.size=base.size;g.origin=base.origin;g.pitch=base.pitch;g.base=base.base;g.anatomy=base.anatomy;g.material_slots=base.material_slots;g.ownership=std::move(owned.first);
    const auto&comp=builtVolume->components[piece.component];Cell lo{comp.lo[0]>>3,comp.lo[1]>>3,comp.lo[2]>>3},hi{comp.hi[0]>>3,comp.hi[1]>>3,comp.hi[2]>>3};
    if(!seed->components.empty())for(const auto id:seed->components){const auto&member=builtVolume->components[id];for(unsigned a=0;a<3;++a){(&lo.x)[a]=std::min((&lo.x)[a],member.lo[a]>>3);(&hi.x)[a]=std::max((&hi.x)[a],member.hi[a]>>3);}}
    for(const auto&ch:piece.mesh.children)for(const auto&v:ch.vertices){const Cell c{int(std::floor((v.surface[0]-base.origin.x)/base.pitch))>>3,int(std::floor((v.surface[1]-base.origin.y)/base.pitch))>>3,int(std::floor((v.surface[2]-base.origin.z)/base.pitch))>>3};
     lo={std::min(lo.x,c.x),std::min(lo.y,c.y),std::min(lo.z,c.z)};hi={std::max(hi.x,c.x),std::max(hi.y,c.y),std::max(hi.z,c.z)};}
    lo={lo.x-2,lo.y-2,lo.z-2};hi={hi.x+2,hi.y+2,hi.z+2};
    {auto&own=const_cast<core::PieceOwnership&>(*g.ownership); // made above, not shared yet
     own.pure_lo=lo;own.pure_size={hi.x-lo.x+1,hi.y-lo.y+1,hi.z-lo.z+1};
     const std::size_t n=std::size_t(own.pure_size.x)*std::size_t(own.pure_size.y)*std::size_t(own.pure_size.z);own.pure.assign((n+63)/64,~std::uint64_t(0));
     const auto&V=*builtVolume;
     // The tiles of one column that hold cells of another component are not pure (the former statements).
     const auto clearForeign=[&](const gfy_connectivity::Column&column,std::vector<std::uint64_t>&pure){
      for(auto s=column.first;s<column.first+column.count;++s){const auto&sp=V.spans[s];if(!sp.component||(seed->components.empty()?sp.component==piece.component:std::binary_search(seed->components.begin(),seed->components.end(),sp.component)))continue;
       // speedfix: the bits of tz = z0..z1 are consecutive (i grows by one with tz): cleared a word at a time, the same
       // bits the former per-tile loop cleared (GFY_SF_PURE_RANGES=0: that loop)
       static const bool pureRanges=[]{const char*e=std::getenv("GFY_SF_PURE_RANGES");return !e||std::atoi(e)!=0;}();
       const int z0=std::max(lo.z,sp.lo>>3),z1=std::min(hi.z,(sp.hi-1)>>3);
       if(pureRanges&&!sfPureRangesOff()){if(z0>z1)continue;
        const std::size_t first=(std::size_t((column.x>>3)-lo.x)*std::size_t(own.pure_size.y)+std::size_t((column.y>>3)-lo.y))*std::size_t(own.pure_size.z)+std::size_t(z0-lo.z);
        const std::size_t last=first+std::size_t(z1-z0);
        for(std::size_t w=first>>6;w<=(last>>6);++w){const std::size_t b0=w==(first>>6)?(first&63):0,b1=w==(last>>6)?(last&63):63;
         const std::uint64_t mask=(b1-b0==63?~std::uint64_t(0):((std::uint64_t(1)<<(b1-b0+1))-1))<<b0;pure[w]&=~mask;}
        continue;}
       for(int tz=z0;tz<=z1;++tz){
        const std::size_t i=(std::size_t((column.x>>3)-lo.x)*std::size_t(own.pure_size.y)+std::size_t((column.y>>3)-lo.y))*std::size_t(own.pure_size.z)+std::size_t(tz-lo.z);
        pure[i>>6]&=~(std::uint64_t(1)<<(i&63));}}};
     const auto formerWalk=[&](std::vector<std::uint64_t>&pure){
      auto col=std::lower_bound(V.columns.begin(),V.columns.end(),lo.y*8,[](const auto&c,int y){return c.y<y;});
      for(;col!=V.columns.end()&&col->y<=hi.y*8+7;++col){if(col->x<lo.x*8||col->x>hi.x*8+7)continue;clearForeign(*col,pure);}};
     // split2 part 9 (2026-11-12): the former walk visits every column of the piece's box to find the spans of other
     // components (5-6 ms for a body piece that has one severed limb beside it). Every span of a component lies in that
     // component's bounding box: only the other components' boxes (cut to the piece's box) are walked, when together they
     // are at most a quarter of the columns. A column in several boxes is visited again; clearing is idempotent.
     // GFY_SEP_PURE_BOXES=0: the former walk alone. GFY_SEP_PURE_BOXES_VERIFY=1: both, the masks compared (piece_pure_mask_mismatch).
     static const bool pureBoxes=[]{const char*e=std::getenv("GFY_SEP_PURE_BOXES");return !e||std::atoi(e)!=0;}();
     static const bool pureBoxesVerify=std::getenv("GFY_SEP_PURE_BOXES_VERIFY")!=nullptr;
     const int boxX0=lo.x*8,boxX1=hi.x*8+7,boxY0=lo.y*8,boxY1=hi.y*8+7;
     bool boxes=pureBoxes&&seed->components.empty();std::uint64_t boxColumns=0;
     for(std::uint32_t id=1;boxes&&id<V.components.size();++id){const auto&other=V.components[id];if(id==piece.component||!other.cells)continue;
      const int ax=std::max(other.lo[0],boxX0),bx=std::min(other.hi[0]-1,boxX1),ay=std::max(other.lo[1],boxY0),by=std::min(other.hi[1]-1,boxY1);
      if(ax<=bx&&ay<=by)boxColumns+=std::uint64_t(bx-ax+1)*std::uint64_t(by-ay+1);
      boxes=boxColumns*4<=V.columns.size();}
     // speedfix: formerWalk visits every column of the rows the box spans (all of a long limb's rows: most of the body's
     // 200,000 columns at 2 mm) and clears for the foreign spans only. Here either the other components' columns inside
     // the box are visited (their column lists, EarlyOwners, built beside the surface partition; a column holding several
     // foreign components is visited once per component - clearing is idempotent) or, when that is more, each row of the
     // box from its first column inside the box (a binary search) - the same columns formerWalk clears for, so the mask is
     // the same. GFY_SF_PURE_INDEX=0: the former walks.
     static const bool pureIndex=[]{const char*e=std::getenv("GFY_SF_PURE_INDEX");return !e||std::atoi(e)!=0;}();
     std::uint64_t indexCost=~std::uint64_t(0);
     if(foreignColumns&&seed->components.empty()){indexCost=0;for(std::uint32_t id=1;id<foreignColumns->size();++id)if(id!=piece.component)indexCost+=(*foreignColumns)[id].size();}
     const std::uint64_t rowCost=std::uint64_t(std::max(0,boxY1-boxY0+1))*std::uint64_t(std::max(0,boxX1-boxX0+1));
     if(pureIndex&&!pureBoxesVerify&&indexCost<rowCost){
      for(std::uint32_t id=1;id<V.components.size()&&id<foreignColumns->size();++id){if(id==piece.component)continue;
       for(const auto c:(*foreignColumns)[id]){const auto&col=V.columns[c];
        if(col.y<boxY0||col.y>boxY1||col.x<boxX0||col.x>boxX1)continue;clearForeign(col,own.pure);}}}
     else if(pureIndex&&!pureBoxesVerify){
      for(int y=boxY0;y<=boxY1;++y){
       auto col=std::lower_bound(V.columns.begin(),V.columns.end(),std::array<int,2>{{boxX0,y}},[](const gfy_connectivity::Column&a,const std::array<int,2>&q){return gfy_connectivity::xyLess(a.x,a.y,q[0],q[1]);});
       for(;col!=V.columns.end()&&col->y==y&&col->x<=boxX1;++col)clearForeign(*col,own.pure);}}
     else if(boxes){
      for(std::uint32_t id=1;id<V.components.size();++id){const auto&other=V.components[id];if(id==piece.component||!other.cells)continue;
       const int ax=std::max(other.lo[0],boxX0),bx=std::min(other.hi[0]-1,boxX1),ay=std::max(other.lo[1],boxY0),by=std::min(other.hi[1]-1,boxY1);
       for(int y=ay;y<=by&&ax<=bx;++y){
        auto col=std::lower_bound(V.columns.begin(),V.columns.end(),std::array<int,2>{{ax,y}},[](const gfy_connectivity::Column&a,const std::array<int,2>&q){return gfy_connectivity::xyLess(a.x,a.y,q[0],q[1]);});
        for(;col!=V.columns.end()&&col->y==y&&col->x<=bx;++col)clearForeign(*col,own.pure);}}
      if(pureBoxesVerify){std::vector<std::uint64_t> former(own.pure.size(),~std::uint64_t(0));formerWalk(former);const bool same=former==own.pure;
       std::cerr<<(same?"pure_boxes_verify ok":"pure_boxes_verify MISMATCH")<<" component="<<piece.component<<" words="<<own.pure.size()<<" box_columns="<<boxColumns<<std::endl;
       need(same,"piece_pure_mask_mismatch");}
     }else formerWalk(own.pure);
     if(std::getenv("GFY_SF_PURE_VERIFY")){std::vector<std::uint64_t> former(own.pure.size(),~std::uint64_t(0));
      {auto&slowLoop=sfPureRangesOff();const bool kept=slowLoop;slowLoop=true;formerWalk(former);slowLoop=kept;}
      const bool same=former==own.pure;std::cerr<<"sf_pure_verify "<<(same?"ok":"MISMATCH")<<" component="<<piece.component<<std::endl;need(same,"sf_pure_verify_mismatch");}}
    for(const auto&t:base.tiles)if(t.first.x>=lo.x&&t.first.y>=lo.y&&t.first.z>=lo.z&&t.first.x<=hi.x&&t.first.y<=hi.y&&t.first.z<=hi.z)g.tiles.emplace(t.first,t.second);
    if(!seed->components.empty()){
     need(owned.second.cells==piece.cells,"island_seed_cell_conservation");
#ifdef GFY_SEP_VERIFY
     const auto rebuilt=gfy_connectivity::build(g);need(separation_detail::sameTopology(rebuilt,owned.second)&&rebuilt.cells==piece.cells,"island_seed_exact_rebuilt_topology");
     std::cerr<<"ISLAND_CONSERVATION owner="<<piece.component<<" cells="<<piece.cells<<" actual_components="<<rebuilt.components.size()-1<<" topology_exact=1 members=";
     for(const auto id:seed->components)std::cerr<<id<<":"<<builtVolume->components[id].cells<<",";std::cerr<<std::endl;
#endif
    }
    seed->asset=asset;seed->parent=stamp;seed->grid=std::make_shared<const Grid>(std::move(g));
    seed->topology=std::make_shared<const gfy_connectivity::Topology>(std::move(owned.second));
    seed->history=context?context->history:std::shared_ptr<const ResolvedCutHistory>();seed->sever=context?context->sever:std::shared_ptr<const SeverState>();
    if(timing)timing->piece_ms[i]=core::ms(pieceStarted);sfPieceMs[i]=core::ms(pieceStarted);}catch(...){failed[i]=std::current_exception();}});
  if(sfProf){std::cerr<<"sep_prof_continuation owners_wait_ms="<<core::ms(ownersWait)<<" pieces_ms=";for(const double m:sfPieceMs)std::cerr<<m<<",";std::cerr<<std::endl;}
  for(const auto&e:failed)if(e)std::rethrow_exception(e);
  for(const auto&piece:out->pieces)if(checkpoint)checkpoint("component_continuation",piece.component,65535);
 }else if(!out->no_separation){
  if(timing)timing->piece_ms.assign(out->pieces.size(),0.);
  // P1.2: the pieces' continuations are prepared side by side (<= 4 threads;
  // each make() and surface copy is serial inside), then committed in piece
  // order exactly as the serial loop did: checkpoint, then the session or its
  // failure. A piece's failure is replayed on this thread with the message the
  // serial loop recorded (fail() is thread-local), and later pieces are
  // discarded, as the serial loop never built them.
  // A canceled job (P1.5) skips the pieces not started yet and throws
  // "separation_canceled" at the first skipped piece.
  struct Continuation {std::shared_ptr<ComponentSession>session;bool made{},initialized{},skipped{};std::string failure;double ms{};};
  const std::unique_ptr<Continuation[]>ready(new Continuation[out->pieces.size()]);
  sep::parallel(out->pieces.size(),1,sep::max_job_threads,[&](std::size_t first,std::size_t last,std::size_t){
   for(std::size_t i=first;i<last;++i){auto&c=ready[i];const auto pieceStarted=core::Clock::now();
    if(sep::cancelRequested()){c.skipped=true;continue;}
    c.session=component_detail::snapshotSession(used,stamp,out->pieces[i].component,asset,volume);c.made=bool(c.session);
    if(!c.made)c.failure=last_failure();
    else{c.initialized=c.session->initialize_published_surface(out->pieces[i].mesh);if(!c.initialized)c.failure=last_failure();}
    c.ms=core::ms(pieceStarted);}});
  for(std::size_t i=0;i<out->pieces.size();++i){auto&piece=out->pieces[i];auto&c=ready[i];
   if(checkpoint)checkpoint("component_continuation",piece.component,65535);
   if(c.skipped)sep::throwIfCanceled();
   // P1.10: a failing piece still reports the time it took.
   if(timing)timing->piece_ms[i]=c.ms;
   if(!c.made||!c.initialized)fail(c.failure.c_str());
   piece.continuation=c.made?std::move(c.session):std::shared_ptr<ComponentSession>{};
   need(c.made&&c.initialized,"separation_component_session_preparation");
  }
 }else if(asset){
  // P1.3 (v): after a no-split verdict, prewarm this key's physics map on the
  // warm thread (a no-op when it is cached or being built), so a later split of
  // the same corpse does not pay the cold map build.
  (void)warm_separation_async(asset,input);
  if(!extras.piece)(void)prefitQueue(asset,grid,nullptr,&input,stamp,true); // speedfix: this body's damaged elements fitted ahead
 }
 out->continuation_ms=core::ms(continued);if(timing)timing->continuation_ms=out->continuation_ms;
 if(std::getenv("GFY_SEP_PROF"))std::cerr<<"sep_prof_continuation total_ms="<<out->continuation_ms<<" prepare_ms="<<core::ms(started)-out->continuation_ms<<std::endl; // speedfix
 return out;
}
std::shared_ptr<const SeparationOutput>ComponentSession::prepare_owned_separation(
    const SeparationInput&input,bool allow_separation)const noexcept{
 AllocationScope scope;const auto started=core::Clock::now();const sep::ActiveJob activeJob;
 std::shared_ptr<SeparationOutput>out;try{out=std::make_shared<SeparationOutput>();}catch(...){return {};}
 try{
  std::lock_guard<std::mutex>lock(impl_->mutex);
  out->stamp=impl_->source_stamp;out->owned_continuation=true;
  out->original_component=impl_->source_component;out->local_revision=impl_->revision;
  need(bool(impl_->asset)&&bool(impl_->accepted),"piece_original_asset_required");
  need(bool(impl_->current.ownership),"piece_ownership_required");
  // Never run parent-source joining against the owned Grid. The virtual grid
  // preserves the accepted whole-parent context and overlays only newly
  // removed cells authorized by the original component's immutable spans.
  Grid virtualParent=*impl_->accepted;
  need(!virtualParent.ownership,"piece_accepted_parent_must_be_whole");
  for(const auto&entry:impl_->accepted->tiles){
   auto current=impl_->current.tiles.find(entry.first);
   need(current!=impl_->current.tiles.end(),"piece_accepted_tile_lost");
   for(unsigned word=0;word<8;++word)
    need((entry.second.removed[word]&~current->second.removed[word])==0,
         "piece_accepted_mask_regrowth");
  }
  for(const auto&entry:impl_->current.tiles){
   auto old=impl_->accepted->tiles.find(entry.first);
   for(unsigned word=0;word<8;++word){
    const auto oldBits=old==impl_->accepted->tiles.end()?0:old->second.removed[word];
    const auto now=entry.second.removed[word];
    need((oldBits&~now)==0,"piece_accepted_mask_regrowth");
    const auto added=now&~oldBits;
    if(!added)continue;
    for(unsigned bit=0;bit<64;++bit)if((added>>bit)&1){
     Cell cell{entry.first.x*8+int(word),entry.first.y*8+int(bit/8),entry.first.z*8+int(bit%8)};
     need(impl_->current.valid(cell)&&
          (impl_->current.ownership->cuttable(cell)||!impl_->accepted->rawMaterialBase(cell)),
          "piece_foreign_mask_bit");
    }
    virtualParent.tiles[entry.first].removed[word]|=added;
   }
  }
  need(virtualParent.tiles.size()<=262144,"piece_virtual_parent_tile_budget");
  // P1.7: `before` is the stored build(*accepted) when known; `after` is
  // derived from it (the virtual parent only adds removed cells), with
  // build()'s exact budgets. compare() and the ancestor checks cannot fail for
  // a derived result: every span lies inside one parent span.
  std::shared_ptr<const gfy_connectivity::Topology>beforeShared=impl_->accepted_topology;
  if(!beforeShared)beforeShared=std::make_shared<const gfy_connectivity::Topology>(gfy_connectivity::build(*impl_->accepted));
  const auto&before=*beforeShared;
  std::vector<std::uint32_t>parentSpans;bool derived=false;
  const auto afterShared=std::make_shared<const gfy_connectivity::Topology>(
      component_detail::exactBuild(before,virtualParent,impl_->asset.get(),&input,&parentSpans,derived));
  const auto&after=*afterShared;
  if(!derived)(void)gfy_connectivity::compare(before,after);
  need(impl_->source_component>0&&impl_->source_component<before.components.size(),
       "piece_original_component_lost");
  std::map<std::uint32_t,std::uint32_t>ancestor;
  if(derived){const auto origin=gfy_connectivity::derivedAncestors(before,after,parentSpans);
   for(std::uint32_t c=1;c<origin.size();++c)ancestor.emplace(c,origin[c]);
   std::vector<std::uint32_t>().swap(parentSpans);}
  else for(const auto&column:after.columns)for(auto i=column.first;i<column.first+column.count;++i){
   const auto&span=after.spans[i];
   const auto original=before.at(column.x,column.y,span.lo);
   need(original&&before.at(column.x,column.y,span.hi-1)==original,
        "piece_descendant_spans_multiple_ancestors");
   auto [it,inserted]=ancestor.emplace(span.component,original);
   need(inserted||it->second==original,"piece_descendant_merges_ancestors");
  }
  std::set<std::uint32_t>wanted;
  for(const auto&entry:ancestor)if(entry.second==impl_->source_component)
   wanted.insert(entry.first);
  need(!wanted.empty(),"piece_no_retained_descendant");
  separation_detail::PrepareExtras extras;extras.precomputed_volume=afterShared; // == build(virtualParent)
  auto full=separation_detail::prepare(*impl_->asset,virtualParent,impl_->source_stamp,input,&wanted,{},extras);
  need(!full->no_separation&&full->error.empty(),"piece_parent_partition_unavailable");
  out->original_surface_children=std::move(full->original_surface_children);
  out->phase_ms=full->phase_ms;
  auto nextParent=std::make_shared<const Grid>(std::move(virtualParent));
  std::uint64_t largest=0;
  for(auto&piece:full->pieces){
   auto it=ancestor.find(piece.component);
   need(it!=ancestor.end(),"piece_partition_without_volume_component");
   if(it->second!=impl_->source_component)continue;
   need(piece.component<after.components.size()&&piece.cells==after.components[piece.component].cells,
        "piece_partition_volume_mismatch");
   if(piece.cells>largest){largest=piece.cells;out->source_component=piece.component;}
   out->pieces.push_back(std::move(piece));
  }
  need(!out->pieces.empty()&&out->source_component,"piece_no_retained_descendant");
  const auto continued=core::Clock::now();
  if(!allow_separation&&out->pieces.size()>1){
   auto merged=component_detail::combinePieces(out->pieces,out->source_component,
       impl_->asset->original_children);
   out->pieces.clear();out->pieces.push_back(std::move(merged));
   out->combined_owner=true;
  }else for(auto&piece:out->pieces){
   piece.continuation=component_detail::make(nextParent,impl_->source_stamp,
       piece.component,0,impl_->asset,afterShared);
   need(piece.continuation->initialize_published_surface(piece.mesh),
        "piece_descendant_visible_surface");
  }
  out->continuation_ms=core::ms(continued);
  // A continuing wound need not split again. One updated owner mesh is still
  // a publication candidate; siblings never appear in this output.
  out->no_separation=false;
 }catch(const std::bad_alloc&){out->pieces.clear();out->source_component=0;out->error=outOfMemoryText;}
 catch(const std::exception&e){out->pieces.clear();out->source_component=0;try{out->error=e.what();}catch(...){out->error=outOfMemoryText;}}
 catch(...){out->pieces.clear();out->source_component=0;out->error="unknown_owned_separation_failure";}
 out->worker_ms=core::ms(started);return out;
}
struct ComponentPreparation::Impl {
 mutable std::mutex mutex;
 std::shared_ptr<const ComponentOutput>output;
 // Preallocated at submission on the engine thread: the worker reports any
 // failure, including running out of memory, without allocating.
 std::shared_ptr<ComponentOutput>reserve;std::shared_ptr<ComponentSession::Impl::Ready>reserve_ready;
};
ComponentPreparation::ComponentPreparation(std::shared_ptr<Impl>state):impl_(std::move(state)){}
std::shared_ptr<const ComponentOutput>ComponentPreparation::completed()const{
 std::lock_guard<std::mutex>lock(impl_->mutex);return impl_->output;
}
namespace component_detail {
// One bounded CPU service shared by every fragment. Jobs may be canceled by
// their session, but never spawn one operating-system thread per hit.
struct WorkService {
 std::mutex mutex;std::condition_variable wake;
 std::deque<std::function<void()>>jobs;bool stop{};std::thread worker;
 WorkService():worker([this]{run();}){}
 void quit()noexcept{{std::lock_guard<std::mutex>lock(mutex);stop=true;jobs.clear();}
  wake.notify_one();if(worker.joinable())worker.join();}
 ~WorkService(){quit();}
 void run(){painterThreadEntry();for(;;){std::function<void()>job;
   {std::unique_lock<std::mutex>lock(mutex);
    wake.wait(lock,[&]{return stop||!jobs.empty();});if(stop)return;
    job=std::move(jobs.front());jobs.pop_front();}
   painterThreadEntry(); // the engine may install its policy after this thread started
   job();
  }}
};
inline WorkService&service(){static WorkService& value=::gfy_live::core::worker_quit::immortal<WorkService>(0);return value;} // quitfix: never destroyed
inline bool sameStamp(const Stamp&a,const Stamp&b){
 return a.owner==b.owner&&a.expected_generation==b.expected_generation&&a.revision==b.revision;
}
}
std::shared_ptr<ComponentPreparation>ComponentSession::submit_async(
    Impact event,SeparationInput input,std::uint64_t current_generation,
    bool allow_separation)noexcept{
 AllocationScope scope;
 try{
  auto state=std::make_shared<ComponentPreparation::Impl>();
  state->reserve=std::make_shared<ComponentOutput>();state->reserve_ready=std::make_shared<Impl::Ready>();
  auto handle=std::make_shared<ComponentPreparation>(std::move(state));
  auto self=shared_from_this();auto&service=component_detail::service();
  std::lock_guard<std::mutex>queueLock(service.mutex);
  need(service.jobs.size()<2,"piece_worker_queue_full");
  {std::lock_guard<std::mutex>lock(impl_->mutex);
   need(impl_->bound&&!impl_->canceled&&!impl_->pending&&bool(impl_->published_source),
        "piece_owner_not_ready");
   need(current_generation==impl_->generation,"piece_generation_mismatch");
   need(event.sequence==impl_->sequence+1,"piece_event_sequence");
   need(event.penetration_m>0&&std::isfinite(event.radius)&&
        event.radius>=.0005&&event.radius<=.08,"piece_impact_size");
   impl_->pending=true;impl_->sequence=event.sequence;
  }
  try{service.jobs.push_back([self,event=std::move(event),input=std::move(input),
                              current_generation,allow_separation,handle]()mutable{
    self->run_candidate(std::move(event),std::move(input),current_generation,
        allow_separation,handle);
   });}
  catch(...){std::lock_guard<std::mutex>lock(impl_->mutex);
   impl_->pending=false;--impl_->sequence;throw;}
  service.wake.notify_one();return handle;
 }catch(const std::bad_alloc&){fail(outOfMemoryText);return {};}
 catch(const std::exception&e){fail(e.what());return {};}
 catch(...){fail("unknown_piece_submit_failure");return {};}
}
void ComponentSession::run_candidate(Impact event,SeparationInput input,
    std::uint64_t expected_generation,bool allow_separation,
    std::shared_ptr<ComponentPreparation>handle)noexcept{
 const sep::ActiveJob activeJob; // P1.3: warm builds yield while real separation work runs
 // Preallocated at submission, like the failed receipt below: any failure,
 // including running out of memory, is reported without allocating.
 auto out=std::move(handle->impl_->reserve);
 out->separation_allowed=allow_separation;
 auto failCandidate=[&]{std::lock_guard<std::mutex>lock(impl_->mutex);
  if(!impl_->canceled&&impl_->pending&&impl_->sequence==event.sequence){
   auto ready=std::move(handle->impl_->reserve_ready);ready->receipt=out->receipt;
   ready->failed=true;impl_->ready=std::move(ready);}};
 {std::lock_guard<std::mutex>lock(impl_->mutex);
  out->receipt.owner=impl_->owner;out->receipt.parent_stamp=impl_->source_stamp;
  out->receipt.expected_generation=expected_generation;
  out->receipt.expected_local_revision=impl_->revision;
  out->receipt.next_local_revision=impl_->revision;
  out->receipt.sequence=event.sequence;
 }
 try{
  Grid candidate;std::shared_ptr<const Source>visible;std::shared_ptr<const gfy_packed::SkinOwnershipQuery>ownership;
  std::shared_ptr<const Grid>accepted;std::shared_ptr<const Asset>asset;
  Stamp stamp;Identity owner;std::uint32_t root=0;std::uint64_t oldRevision=0;
  gfy_connectivity::Topology before;std::shared_ptr<const gfy_connectivity::Topology>acceptedTopology;
  {std::lock_guard<std::mutex>lock(impl_->mutex);
   need(impl_->bound&&!impl_->canceled&&impl_->pending&&!impl_->ready&&
        expected_generation==impl_->generation&&event.sequence==impl_->sequence,
        "piece_candidate_stale_owner");
   candidate=impl_->current;before=impl_->topology;visible=impl_->published_source;ownership=impl_->published_ownership;
   accepted=impl_->accepted;asset=impl_->asset;stamp=impl_->source_stamp;acceptedTopology=impl_->accepted_topology;
   owner=impl_->owner;root=impl_->source_component;oldRevision=impl_->revision;
  }
  auto&receipt=out->receipt;receipt.owner=owner;receipt.parent_stamp=stamp;
  receipt.expected_generation=expected_generation;receipt.expected_local_revision=oldRevision;
  receipt.next_local_revision=oldRevision;receipt.sequence=event.sequence;
  auto&result=out->impact;result.sequence=event.sequence;
  result.ammo_section=event.ammo_section;result.hit_bone=event.hit_bone;
  result.hit_power=event.hit_power;result.radius=event.radius;
  Mapping mapping;
  try{mapping=sourceRay(*visible,event,nullptr,nullptr,ownership.get());
   const auto near=std::abs(mapping.posed_distance-event.source_search_distance);
   need(near<=std::max(.03,4*candidate.pitch),"piece_hit_off_current_surface");
   need(!occluded(*visible,event,mapping.posed_distance,nullptr,ownership.get()),"piece_hit_occluded_by_current_surface");
  }catch(const std::bad_alloc&){throw;}catch(const std::exception&e){result.rejection=e.what();out->no_geometry_change=true;}
  std::shared_ptr<Impl::Ready>ready=std::make_shared<Impl::Ready>();
  ready->receipt=receipt;
  if(!out->no_geometry_change){
   result.rest_entry=point(mapping.entry);result.rest_direction=point(mapping.direction);
   result.source_face=mapping.face;result.barycentric=mapping.bary;
   result.entry_surface=woundSurface(*visible,mapping.face,mapping.entry);
   // Match the source worker's exterior entry convention. eraseTunnel finds
   // the first solid cell from here, then measures penetration from that cell;
   // the outside inlet does not shorten the specified distal reach.
   Vec start=mapping.entry-mapping.direction*(candidate.pitch*2+event.radius);
   std::size_t edge=0;
   const auto march=firstSolid(candidate,start,mapping.direction);
   result.rest_entry=point(march.found?march.entry:mapping.entry);
   // The immutable whole-parent envelope could report an exit through an
   // absent sibling. Certify the far side against this published piece only.
   core::BodyBVH pieceEnvelope(*visible);
   TunnelExit exit;
   auto hit=eraseTunnel(candidate,start,mapping.direction,event.radius,event.penetration_m,
       &edge,&pieceEnvelope,event.exit_diameter_mult,&exit);
   if(!hit.freshSolid){result.miss=true;out->no_geometry_change=true;}
   else{
    need(candidate.tiles.size()<=262144,"piece_sparse_tile_budget");
    bool derived=false; // P1.7, as in hit_rest
    auto after=component_detail::exactBuild(before,candidate,asset.get(),nullptr,nullptr,derived);
    if(!derived)(void)gfy_connectivity::compare(before,after);
    need(after.cells<=before.cells&&before.cells-after.cells==hit.freshSolid,
         "piece_candidate_exact_removed_volume");
    result.removed=true;result.removed_solid_cells=hit.freshSolid;
    result.edge_contact_cells=edge;result.cutter=point(hit.cutter);
    result.exit_radius=event.radius*event.exit_diameter_mult;
    if(exit.reached){
     double nearest=std::numeric_limits<double>::infinity();
     std::uint32_t face=0;Vec p;
     pieceEnvelope.nearest(0,exit.point,nearest,face,p);
     if(nearest<1e-10){
      result.exit_surface=woundSurface(*visible,face,p);
      result.penetrated=result.exit_surface.valid;
     }
    }
    // P1.7: the preview's topology is build(candidate) == after; its accepted
    // grid is this session's, so is its accepted topology.
    auto preview=std::make_shared<ComponentSession>(
        std::make_shared<Impl>(stamp,root,asset,accepted,candidate,after,acceptedTopology));
    preview->impl_->revision=oldRevision+1;
    auto separation=preview->prepare_owned_separation(input,allow_separation);
    if(!separation||separation->error==outOfMemoryText)throw std::bad_alloc();
    need(bool(separation)&&separation->error.empty()&&!separation->pieces.empty(),
         separation?separation->error.c_str():"piece_candidate_separation_missing");
    out->separation=std::move(separation);
    receipt.next_local_revision=oldRevision+1;ready->receipt=receipt;
    ready->changed=true;ready->grid=std::move(candidate);
    ready->topology=std::move(after);
    if(out->separation->pieces.size()==1){
     ready->mesh=std::make_shared<const gfy_packed::Mesh>(out->separation->pieces[0].mesh);
     ready->source=component_detail::visibleSource(*ready->mesh,asset->source.bones);
     ready->ownership=gfy_packed::skin_ownership_query(*ready->mesh,false);
    }
   }
  }
  {std::lock_guard<std::mutex>lock(impl_->mutex);
   need(!impl_->canceled&&impl_->pending&&impl_->generation==expected_generation&&
        impl_->revision==oldRevision&&impl_->sequence==event.sequence,
        "piece_candidate_canceled_or_superseded");
   impl_->ready=std::move(ready);
  }
 }catch(const std::bad_alloc&){out->separation.reset();out->error=outOfMemoryText;memoryCounters.piece_oom.fetch_add(1,std::memory_order_relaxed);failCandidate();
 }catch(const std::exception&e){out->separation.reset();try{out->error=e.what();}catch(...){out->error=outOfMemoryText;}failCandidate();
 }catch(...){out->separation.reset();try{out->error="unknown_piece_candidate_failure";}catch(...){out->error=outOfMemoryText;}failCandidate();
 }
 std::lock_guard<std::mutex>lock(handle->impl_->mutex);
 handle->impl_->output=std::move(out);
}
namespace component_detail {
inline bool admitReceipt(const ComponentSession::Impl&state,
    const ComponentReceipt&receipt,bool accepted,std::uint64_t published_generation){
 const auto&ready=state.ready;
 return !state.canceled&&state.bound&&state.pending&&bool(ready)&&
    receipt.owner==state.owner&&sameStamp(receipt.parent_stamp,state.source_stamp)&&
    receipt.expected_generation==state.generation&&
    receipt.expected_local_revision==state.revision&&
    receipt.sequence==state.sequence&&
    receipt.next_local_revision==ready->receipt.next_local_revision&&
    (!accepted||!ready->failed)&&
    (!accepted||published_generation>=receipt.expected_generation)&&
    (!accepted||!ready->changed||published_generation>receipt.expected_generation);
}
}
bool ComponentSession::can_acknowledge(const ComponentReceipt&receipt,
    std::uint64_t published_generation)const noexcept{
 std::lock_guard<std::mutex>lock(impl_->mutex);
 return component_detail::admitReceipt(*impl_,receipt,true,published_generation);
}
bool ComponentSession::acknowledge(const ComponentReceipt&receipt,bool accepted,
    std::uint64_t published_generation)noexcept{
 std::lock_guard<std::mutex>lock(impl_->mutex);
 if(!component_detail::admitReceipt(*impl_,receipt,accepted,published_generation))return false;
 const auto&ready=impl_->ready;
 if(accepted&&ready->changed){
  if(!ready->mesh){
   // A new separation replaced this owner with precomputed descendant
   // sessions. Native binds them only after its visual/collider commit.
   impl_->canceled=true;
  }else{
   impl_->current=std::move(ready->grid);impl_->topology=std::move(ready->topology);
   impl_->published_mesh=std::move(ready->mesh);impl_->published_source=std::move(ready->source);impl_->published_ownership=std::move(ready->ownership);
   impl_->revision=receipt.next_local_revision;
   impl_->generation=published_generation;
  }
 }else if(accepted)impl_->generation=published_generation;
 impl_->ready.reset();impl_->pending=false;return true;
}
}
