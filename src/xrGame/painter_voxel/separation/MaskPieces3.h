#pragma once
// Mask-only wounds, slices 3 + 4 (console painter_voxel_mask_only 3): separation pieces of decide-once tears (TearDecide.h).
// Included by LiveSeparationWorker_impl.h after MaskPieces.h. Slice 4 (2026-11-12): ANY number of decide-once tears, on the body
// or on a piece, in one separation or one after another.
// The published mesh lists its tears (gfy_packed::Mesh::tears): side 255 = decided on this grid since its birth (not split yet),
// 0 / 1 = a tear its parent's separation gave it (this side), 2 = a tear that did not come apart here (a crack: both sides kept).
// When the published mesh holds at least one unsplit tear, build() runs instead of slice 2's mask_pieces::build:
//  * each piece's side of every tear: a vote over the tear's footprint, cells 2.5 / 4.5 / 7.5 cells off the tear surface on each
//    side (next to its thin sheet) and at the tear's reach: the volume's component there. A piece on both sides of a tear holds it
//    as a crack (side 2);
//  * each piece's triangles: the ORIGINAL exterior triangles (a parent piece's coverage boxes are left out and made again); a
//    sample point inside unsplit tears' regions belongs to the pieces on its side of all of them; any other point (or one the tears
//    leave to several pieces) to the piece whose bones carry most of the triangle's skin weight; a tie (pieces posed by the same
//    bones: head pieces) goes to the piece whose volume lies under the triangle (stepped inward along its normal);
//    posing rules as slice 2 (own bones, or a foreign bone frozen to its unique retained frontier);
//  * each piece keeps the unsplit tears it lies on (its side) and the inherited tears it borders (its triangles or its volume next
//    to that tear's sheet on the parent's side), any number (no limit); one coverage box per kept tear (the tear slab, 12 triangles
//    skinned to one bone of the piece, u = -1000: slice 2's box sentinel);
//  * the removal field: the piece grid's removed cells + the inherited marks (a piece of a piece keeps hiding what its parent hid);
//    the torn ends, the gaps and the other sides are the tear function's (painter_vmaskt_* programs).
// Otherwise (no unsplit tear) slice 2's build runs and attach() gives every piece the tears it borders.
// cutfix (2026-11-12, repeated cuts and pieces with many torn ends):
//  * a tear acts only inside its torn zone (TearAtlas.h). Everything of another piece is hidden by this piece's removal field
//    (outside its kept tears' torn zones): the other pieces' cells inside the box of its own cells, and every gear / padding cell
//    another piece OWNS (owners(): the gear cells of the outfit, GearCells.h, and the clothing padding, each given to one piece at
//    the separation - the fresh tear's side near its plane, else connectivity to the pieces' tissue); these marks are inherited, so
//    a piece of a piece keeps hiding what its parent's neighbours were. The programs search the volume only inside the box of the
//    piece's own cells (TearAtlas row 0).
//  * a triangle is drawn by the pieces owning its samples' cells (each pixel by its cell's owner alone); where no cell is owned,
//    by the selection's rule (a skin tie to ONE piece); a piece the owners leave without a triangle takes the former rule's.
//  * a piece keeps only the tears whose torn zone meets what it draws; a coverage box only for a tear whose zone meets its cells.
#include <climits>
#include <unordered_set>
#include "../TearAtlas.h" // slice 4: one drawing set per piece for all of its torn ends
#include "../HeadBurstRecord.h" // headfix: a head burst's record (its function: the group of a point)
namespace mask_pieces3 {
constexpr std::size_t maxPieces=32; // pieces of one separation (bit masks are 32 wide)
struct Stats {
 std::size_t selected{},dropped{},unposable{},frozen{},boxes{},boxTriangles{},fieldBytes{},inRegion{},bySkeleton{},byVolume{},ties{};
 std::size_t tears{},fresh{},kept{},cracks{};tear_atlas::Stats atlas;bool used{},attached{};double ms{},voteMs{},selectMs{},packMs{};const char*why="";
 std::size_t ghosts{},markTiles{},ghostTiles{},prunedTears{},prunedBoxes{},padTiles{},padCells{},padMarks{},gearCells{},ownedCells{},seededCells{},altPieces{};double marksMs{},padMs{}; // cutfix
 std::size_t gfBars{},gfExtTiles{},gfLeftTiles{},gfLeftCells{},gfFaces{},gfRects{},gfChildren{},gfTris{};double gfMs{}; // gearfine
 std::vector<std::array<signed char,maxPieces>> sig;   // per published tear: each piece's side (-1 not on it, 0 / 1, 2 both)
 std::vector<std::array<long long,2*maxPieces>> votes; // per published tear: [side * maxPieces + piece]
 std::string sides() const{std::string s;for(std::size_t t=0;t<sig.size();++t){if(t)s+=';';for(std::size_t k=0;k<maxPieces;++k){const int v=sig[t][k];if(v<0)continue;
   s+=std::to_string(k)+(v==0?"A":(v==1?"B":"C"));}}return s;}
};
// One tear's region and side on the CPU, from the GPU block (the same numbers the shader reads)
struct Region {
 double C[3],n[3],u[3],w[3],Rl,H,O,cell,RL,s0,s1,s2,s3,k0,k1,ph;int N;const gfy_packed::TearBlock*T;
 explicit Region(const gfy_packed::TearBlock&b):T(&b){auto f=[&](int r,int c){float v;std::memcpy(&v,&b.params[std::size_t(4*r+c)],4);return double(v);};
  for(int k=0;k<3;++k){C[k]=b.centre[k];n[k]=b.normal[k];u[k]=b.u[k];w[k]=b.w[k];}
  Rl=f(0,3);H=f(1,3);O=f(6,0);cell=f(6,1);N=int(f(6,2)+.5);RL=f(8,3);s0=f(9,0);s1=f(9,1);s2=f(9,2);s3=f(9,3);k0=f(10,0);k1=f(10,1);ph=f(10,2);}
 double offset(double pa,double pb)const{const double x=pa/RL,y=pb/RL;const double o=s0*pa+s1*pb+s2*(x*x+y*y-.5)+s3*std::sin(k0*x+k1*y+ph);
  return std::min(std::max(o,-tear_decide::offMax),tear_decide::offMax);}
 bool foot(double pa,double pb)const{const int qa=std::min(std::max(int(std::floor((pa-O)/cell)),0),N-1),qb=std::min(std::max(int(std::floor((pb-O)/cell)),0),N-1);
  return (int(T->section[4*(std::size_t(qb)*N+std::size_t(qa))+1]+.5f)&1)!=0;}
 // -1 outside the region (the skeleton decides), else 0 / 1 (side A / B of the tear surface)
 int side(double x,double y,double z)const{const double d[3]={x-C[0],y-C[1],z-C[2]};const double s=d[0]*n[0]+d[1]*n[1]+d[2]*n[2],pa=d[0]*u[0]+d[1]*u[1]+d[2]*u[2],pb=d[0]*w[0]+d[1]*w[1]+d[2]*w[2];
  if(std::abs(s)>H+tear_decide::offMax||pa*pa+pb*pb>Rl*Rl||!foot(pa,pb))return -1;return s-offset(pa,pb)>=0?1:0;}
 // cutfix: the signed distance to the tear surface (s - offset) inside the region; false outside it
 bool dist(double x,double y,double z,double&out)const{const double d[3]={x-C[0],y-C[1],z-C[2]};const double s=d[0]*n[0]+d[1]*n[1]+d[2]*n[2],pa=d[0]*u[0]+d[1]*u[1]+d[2]*u[2],pb=d[0]*w[0]+d[1]*w[1]+d[2]*w[2];
  if(std::abs(s)>H+tear_decide::offMax||pa*pa+pb*pb>Rl*Rl||!foot(pa,pb))return false;out=s-offset(pa,pb);return true;}
};
inline bool fresh(const gfy_packed::TearRef&t){return t.block&&t.side==255;}
inline std::size_t freshCount(const gfy_packed::Mesh&m){std::size_t n=0;for(const auto&t:m.tears)n+=fresh(t)?1:0;return n;}
// The volume's piece (index into components, -1 none) at a point
struct PieceAt {
 const Grid&grid;const gfy_connectivity::Topology&volume;std::vector<int> pieceOf;std::vector<std::int32_t> columnOf;
 PieceAt(const Grid&g,const gfy_connectivity::Topology&v,const std::vector<std::uint32_t>&components):grid(g),volume(v),pieceOf(v.components.size(),-1),
  columnOf(std::size_t(g.size.x)*std::size_t(g.size.y),-1){
  for(std::size_t k=0;k<components.size()&&k<maxPieces;++k)if(components[k]<pieceOf.size())pieceOf[components[k]]=int(k);
  for(std::size_t ci=0;ci<v.columns.size();++ci){const auto&c=v.columns[ci];if(c.x>=0&&c.y>=0&&c.x<g.size.x&&c.y<g.size.y)columnOf[std::size_t(c.y)*std::size_t(g.size.x)+std::size_t(c.x)]=std::int32_t(ci);}}
 int operator()(double px,double py,double pz)const{const int x=int(std::floor((px-grid.origin.x)/grid.pitch)),y=int(std::floor((py-grid.origin.y)/grid.pitch)),z=int(std::floor((pz-grid.origin.z)/grid.pitch));
  if(x<0||y<0||z<0||x>=grid.size.x||y>=grid.size.y||z>=grid.size.z)return -1;const auto ci=columnOf[std::size_t(y)*std::size_t(grid.size.x)+std::size_t(x)];if(ci<0)return -1;
  const auto&c=volume.columns[std::size_t(ci)];for(auto s=c.first;s<c.first+c.count;++s){const auto&sp=volume.spans[s];if(z>=sp.lo&&z<sp.hi)return sp.component<pieceOf.size()?pieceOf[sp.component]:-1;}return -1;}
};
// cutfix: a box of cells [lo, hi)
struct CellBox {int lo[3]{INT_MAX,INT_MAX,INT_MAX},hi[3]{INT_MIN,INT_MIN,INT_MIN};
 bool valid()const{return lo[0]<hi[0]&&lo[1]<hi[1]&&lo[2]<hi[2];}
 void add(int x,int y,int z0,int z1){lo[0]=std::min(lo[0],x);hi[0]=std::max(hi[0],x+1);lo[1]=std::min(lo[1],y);hi[1]=std::max(hi[1],y+1);lo[2]=std::min(lo[2],z0);hi[2]=std::max(hi[2],z1);}
 void add(const CellBox&b){if(!b.valid())return;for(int a=0;a<3;++a){lo[a]=std::min(lo[a],b.lo[a]);hi[a]=std::max(hi[a],b.hi[a]);}}};
// each piece's own cells' box (from the volume's spans)
inline std::vector<CellBox> ownBoxes(const gfy_connectivity::Topology&volume,const std::vector<std::uint32_t>&components){
 std::vector<int> pieceOf(volume.components.size(),-1);for(std::size_t k=0;k<components.size();++k)if(components[k]<pieceOf.size())pieceOf[components[k]]=int(k);
 std::vector<CellBox> out(components.size());
 for(const auto&c:volume.columns)for(auto s=c.first;s<c.first+c.count;++s){const auto&sp=volume.spans[s];if(sp.component>=pieceOf.size())continue;const int k=pieceOf[sp.component];if(k<0)continue;
  out[std::size_t(k)].add(c.x,c.y,sp.lo,sp.hi);}
 return out;}
inline tear_atlas::Box worldBox(const Grid&g,const CellBox&b,double padCells){tear_atlas::Box w;if(!b.valid())return w;
 for(int a=0;a<3;++a){w.lo[a]=g.origin[a]+(b.lo[a]-padCells)*g.pitch;w.hi[a]=g.origin[a]+(b.hi[a]+padCells)*g.pitch;}return w;}
// cutfix: one bit per 8^3 tile of the grid, rows along tile z (words), row index ty * tnx + tx
struct TileSet {
 int tnx,tny,tnz,wz;std::vector<std::uint64_t> w;
 explicit TileSet(const Grid&g):tnx((g.size.x+7)/8),tny((g.size.y+7)/8),tnz((g.size.z+7)/8),wz(((g.size.z+7)/8+63)/64),
  w(std::size_t((g.size.x+7)/8)*std::size_t((g.size.y+7)/8)*std::size_t(((g.size.z+7)/8+63)/64),0){}
 bool in(int tx,int ty,int tz)const{if(tx<0||ty<0||tz<0||tx>=tnx||ty>=tny||tz>=tnz)return false;
  return ((w[(std::size_t(ty)*tnx+std::size_t(tx))*wz+std::size_t(tz>>6)]>>(tz&63))&1u)!=0;}
 void setZ(int tx,int ty,int tz0,int tz1){if(tx<0||ty<0||tx>=tnx||ty>=tny)return;tz0=std::max(tz0,0);tz1=std::min(tz1,tnz-1); // inclusive
  std::uint64_t*r=&w[(std::size_t(ty)*tnx+std::size_t(tx))*wz];
  for(int z=tz0;z<=tz1;){const int i=z>>6,b=z&63,n=std::min(64-b,tz1-z+1);r[i]|=(n==64?~std::uint64_t(0):((std::uint64_t(1)<<n)-1))<<b;z+=n;}}
 // box (Chebyshev) dilation by r tiles, separable: z inside rows, then x, then y
 TileSet dilated(int r)const{TileSet a=*this;const std::size_t rows=std::size_t(tnx)*tny;
  for(std::size_t q=0;q<rows;++q){const std::uint64_t*s=&w[q*wz];std::uint64_t*d=&a.w[q*wz];
   for(int i=0;i<wz;++i){std::uint64_t v=s[i];for(int k=1;k<=r;++k){v|=s[i]<<k;if(i>0)v|=s[i-1]>>(64-k);v|=s[i]>>k;if(i+1<wz)v|=s[i+1]<<(64-k);}d[i]=v;}}
  TileSet b=a;for(int y=0;y<tny;++y)for(int x=0;x<tnx;++x){std::uint64_t*d=&b.w[(std::size_t(y)*tnx+x)*wz];
   for(int k=-r;k<=r;++k){const int xx=x+k;if(!k||xx<0||xx>=tnx)continue;const std::uint64_t*s=&a.w[(std::size_t(y)*tnx+xx)*wz];for(int i=0;i<wz;++i)d[i]|=s[i];}}
  TileSet c=b;for(int y=0;y<tny;++y)for(int x=0;x<tnx;++x){std::uint64_t*d=&c.w[(std::size_t(y)*tnx+x)*wz];
   for(int k=-r;k<=r;++k){const int yy=y+k;if(!k||yy<0||yy>=tny)continue;const std::uint64_t*s=&b.w[(std::size_t(yy)*tnx+x)*wz];for(int i=0;i<wz;++i)d[i]|=s[i];}}
  return c;}
};
// cutfix: one walk of the volume's spans: each piece's own cells' box and tile set, every component's box
struct SpanPass {std::vector<CellBox> own,comp;std::vector<TileSet> tiles;};
inline SpanPass spanPass(const Grid&grid,const gfy_connectivity::Topology&volume,const std::vector<std::uint32_t>&components){
 SpanPass out;const std::size_t P=components.size();out.own.resize(P);out.comp.resize(volume.components.size());out.tiles.assign(P,TileSet(grid));
 std::vector<int> pieceOf(volume.components.size(),-1);for(std::size_t k=0;k<P;++k)if(components[k]<pieceOf.size())pieceOf[components[k]]=int(k);
 for(const auto&c:volume.columns)for(auto s=c.first;s<c.first+c.count;++s){const auto&sp=volume.spans[s];if(sp.component>=pieceOf.size()||sp.hi<=sp.lo)continue;
  if(sp.component)out.comp[sp.component].add(c.x,c.y,sp.lo,sp.hi);const int k=pieceOf[sp.component];if(k<0)continue;
  out.own[std::size_t(k)].add(c.x,c.y,sp.lo,sp.hi);out.tiles[std::size_t(k)].setZ(c.x>>3,c.y>>3,sp.lo>>3,(sp.hi-1)>>3);}
 return out;}
// cutfix (the owner's design: the cell grid extended to the gear): every gear cell (GearCells.h: a band around every exterior
// triangle) and every clothing-padding cell (anatomy bit 64: solid for the programs, no piece's cell) has ONE owning piece, decided
// at the separation like the tissue: by connectivity to the pieces' tissue, and across a fresh decide-once tear by the tear's side.
//  * tiles (8^3): breadth first (26 neighbours) over the tiles holding gear, padding or a piece's tissue, from each piece's tissue
//    tiles: a tile reached first by one piece is that piece's;
//  * cells, only in the tiles where different owners meet (and one tile around them): breadth first (6 neighbours) through gear and
//    padding cells, from the tissue they lie on, from the owned tiles around, and - inside a fresh tear's region - from the one piece
//    on the cell's side of the tear; the grid's removed cells (the tear's sheet, shot channels) are not crossed.
// Every other piece hides a cell it does not own (build: removal field marks, outside its kept tears' torn zones); a triangle is
// drawn by the pieces owning its samples: each exterior pixel is drawn by exactly one piece.
struct Owners {
 // per-tile scratch of the calling thread, valid until its next owners() call (generation stamps: no clearing per call)
 struct Scratch {std::vector<std::uint32_t> stamp,mask;std::vector<std::int32_t> slot;std::vector<std::uint16_t> dist;std::vector<std::uint8_t> flag;std::uint32_t gen=0;
  std::vector<std::uint32_t> gfc;}; // gearfine: per tile, 2 bits a barrier (0 free, 1 wholly side 0, 2 wholly side 1, 3 across)
 const Scratch*sc=nullptr;std::uint32_t gen=0;
 std::vector<std::uint64_t> roiW;int roiNx=0,roiNy=0,roiWz=0;int L=-1; // the region decided here; outside it the largest piece L owns
 int tnx=0,tny=0,tnz=0;std::size_t P=0;
 struct PT {Cell c;std::array<std::uint8_t,512> own;};std::vector<PT> tiles; // own: piece index, 255 none (tissue: 255, its topology decides)
 std::size_t uTiles=0,bTiles=0,gearCells=0,padCells=0,seeded=0,owned=0;double ms=0;
 std::size_t gfBars=0,gfExtTiles=0,gfLeftTiles=0,gfLeftCells=0; // gearfine: barriers, tiles the barriers added, tiles / cells no piece reached without crossing
 std::vector<std::uint32_t> ulist; // the tiles of the decided region holding gear, padding or tissue (tile index)
 std::size_t idx(int tx,int ty,int tz)const{return (std::size_t(tz)*tny+std::size_t(ty))*tnx+std::size_t(tx);}
 bool inside(int tx,int ty,int tz)const{return tx>=0&&ty>=0&&tz>=0&&tx<tnx&&ty<tny&&tz<tnz;}
 bool valid()const{return sc!=nullptr;}
 std::uint32_t maskAt(std::size_t i)const{return sc->stamp[i]==gen?sc->mask[i]:0u;}
 std::int32_t slotAt(std::size_t i)const{return sc->stamp[i]==gen?sc->slot[i]:-1;}
 static int first(std::uint32_t m){unsigned long k=0;
#ifdef _MSC_VER
  _BitScanForward(&k,m);
#else
  k=unsigned(__builtin_ctz(m));
#endif
  return int(k);}
 // the owner of cell (x, y, z): 255 none
 bool inRoi(int tx,int ty,int tz)const{return ((roiW[(std::size_t(ty)*roiNx+std::size_t(tx))*roiWz+std::size_t(tz>>6)]>>(tz&63))&1u)!=0;}
 std::uint8_t owner(int x,int y,int z)const{const int tx=x>>3,ty=y>>3,tz=z>>3;if(!sc||!inside(tx,ty,tz))return 255;if(L>=0&&!inRoi(tx,ty,tz))return std::uint8_t(L);const std::size_t i=idx(tx,ty,tz);
  const std::int32_t s=slotAt(i);if(s>=0)return tiles[std::size_t(s)].own[std::size_t(((x&7)*8+(y&7))*8+(z&7))];
  const std::uint32_t m=maskAt(i);return (m&&!(m&(m-1)))?std::uint8_t(first(m)):std::uint8_t(255);}
};
inline Owners owners(const Grid&grid,const gfy_connectivity::Topology&volume,const std::vector<std::uint32_t>&components,
    const std::shared_ptr<const painter_detail::TissueField>&tfp,const gear_cells::GearCells*gear,const std::vector<Region>&R,const std::vector<std::size_t>&freshIdx,
    const std::vector<std::array<std::uint32_t,2>>&sideMask,const std::vector<TileSet>&ownT){
 Owners out;const std::size_t P=components.size();out.P=P;if(P<2||P>32||ownT.size()!=P)return out;
 if(gear&&!gear->fits(grid.origin,grid.pitch,grid.size))gear=nullptr;
 const painter_detail::TissueField*tf=tfp&&tfp->available?tfp.get():nullptr;
 if(!tf&&!gear)return out;
 const auto t0=core::Clock::now();
 static const bool prof=std::getenv("GFY_CF_OWNPROF")!=nullptr;double tA=0,tB=0,tC=0,tD=0,tE=0;auto tp=core::Clock::now();
 auto lap=[&](double&acc){const auto n=core::Clock::now();acc+=std::chrono::duration<double,std::milli>(n-tp).count();tp=n;};
 out.tnx=(grid.size.x+7)/8;out.tny=(grid.size.y+7)/8;out.tnz=(grid.size.z+7)/8;const std::size_t T=std::size_t(out.tnx)*out.tny*out.tnz;
 thread_local Owners::Scratch S;
 if(S.stamp.size()!=T){S.stamp.assign(T,0);S.mask.assign(T,0);S.slot.assign(T,-1);S.dist.assign(T,65535);S.flag.assign(T,0);S.gen=0;}
 if(S.gfc.size()!=T)S.gfc.assign(T,0); // gearfine
 if(++S.gen==0){std::fill(S.stamp.begin(),S.stamp.end(),0u);S.gen=1;}
 out.sc=&S;out.gen=S.gen;const std::uint32_t G=S.gen;
 auto touch=[&](std::size_t i){if(S.stamp[i]!=G){S.stamp[i]=G;S.mask[i]=0;S.slot[i]=-1;S.dist[i]=65535;S.flag[i]=0;S.gfc[i]=0;}}; // gearfine: S.gfc
 auto flagAt=[&](std::size_t i)->std::uint8_t{return S.stamp[i]==G?S.flag[i]:std::uint8_t(0);};
 std::vector<int> pieceOf(volume.components.size(),-1);for(std::size_t k=0;k<P;++k)if(components[k]<pieceOf.size())pieceOf[components[k]]=int(k);
 std::vector<std::uint32_t> ulist,frontier,next;
 // the region decided here: the tiles within 8 tiles (128 mm) of a smaller piece's tissue (every piece but the largest); outside
 // it only the largest piece's tissue is near, and the triangle selection's own rule stands
 std::size_t L=0;for(std::size_t k=1;k<P;++k)if(volume.components[components[k]].cells>volume.components[components[L]].cells)L=k;
 TileSet roi(grid);for(std::size_t k=0;k<P;++k){if(k==L)continue;for(std::size_t i=0;i<roi.w.size();++i)roi.w[i]|=ownT[k].w[i];}
 roi=roi.dilated(8);
 // gearfine (2026-11-15): THE CUT THROUGH GEAR FOLLOWS THE TEAR. A fresh decide-once tear that parts two sets of pieces is a
 // BARRIER for the gear and the padding: its surface - the plane and the offset its side uses (Region::side) - extended over the
 // gear out to the tear's region radius + 25 cm. No search below crosses it: a piece on one side does not enter a tile wholly on the
 // other side unless it comes from that side (a limb of the piece lying past the plane keeps its sleeve: its own tissue there
 // seeds it), and no cell step crosses it. So the gear is parted on the tear's surface cell by cell (2 mm), as the tissue is, not
 // on the 16 mm tiles where two searches met (that cut wandered 10-19 cm off the tear and showed tile steps, and gave a pack's
 // outer tile layer to one part and the fill behind it to the other). Gear or padding no piece reaches without crossing goes to
 // the piece it touches (tiles, then cells). The barrier zone's gear and padding tiles on the side the largest piece is not on
 // (|s| <= 30 cm) join the decided region: gear standing off the body (a pack) is parted there too, not left to the largest piece.
 struct GfBar {const Region*R;std::uint32_t m0,m1;double rb,thr;};std::vector<GfBar> gfBar;
 static const bool gfNoBarrier=std::getenv("GFY_GF_NO_BARRIER")!=nullptr; // probe comparisons: owners as before
 const double gfHd=.87*8*grid.pitch;
 if(!gfNoBarrier)for(const auto t:freshIdx){if(gfBar.size()>=16)break;const std::uint32_t m0=sideMask[t][0],m1=sideMask[t][1];if(!m0||!m1||(m0&m1))continue;
  const Region&Rg=R[t];if(!(Rg.Rl>0)||!(Rg.RL>0))continue;
  // thr: |s - offset| at a tile's centre beyond which the whole tile lies on one side (its half diagonal times 1 + the offset's slope)
  const double slope=std::abs(Rg.s0)+std::abs(Rg.s1)+2.*std::abs(Rg.s2)*(Rg.Rl+.25)/(Rg.RL*Rg.RL)+std::abs(Rg.s3)*(std::abs(Rg.k0)+std::abs(Rg.k1))/Rg.RL;
  gfBar.push_back(GfBar{&Rg,m0,m1,Rg.Rl+.25,gfHd*(1.+slope)+1e-4});}
 out.gfBars=gfBar.size();
 // the value of barrier b's surface at a point (s - offset: >= 0 is side 1), false past its reach
 auto gfV=[&](const GfBar&b,double x,double y,double z,double&v)->bool{const Region&Rg=*b.R;const double d0=x-Rg.C[0],d1=y-Rg.C[1],d2=z-Rg.C[2];
  const double pa=d0*Rg.u[0]+d1*Rg.u[1]+d2*Rg.u[2],pb=d0*Rg.w[0]+d1*Rg.w[1]+d2*Rg.w[2];if(pa*pa+pb*pb>b.rb*b.rb)return false;
  v=d0*Rg.n[0]+d1*Rg.n[1]+d2*Rg.n[2]-Rg.offset(pa,pb);return true;};
 // a tile's code for barrier b: 0 free (its centre or a corner past the reach), 1 wholly side 0, 2 wholly side 1, 3 across
 auto gfTile=[&](const GfBar&b,int tx,int ty,int tz)->unsigned{double v=0;
  if(!gfV(b,grid.origin.x+(tx*8+4)*grid.pitch,grid.origin.y+(ty*8+4)*grid.pitch,grid.origin.z+(tz*8+4)*grid.pitch,v))return 0u;
  if(v>b.thr)return 2u;if(v<-b.thr)return 1u;unsigned pos=0,neg=0;
  for(int q=0;q<8;++q){double w=0;if(!gfV(b,grid.origin.x+(tx*8+((q&1)?8:0))*grid.pitch,grid.origin.y+(ty*8+((q&2)?8:0))*grid.pitch,grid.origin.z+(tz*8+((q&4)?8:0))*grid.pitch,w))return 0u;
   if(w>=0)++pos;else ++neg;}
  return pos&&neg?3u:(pos?2u:1u);};
 if(!gfBar.empty()){const auto gfPadE=tf?gear_cells::padTileFlags(tfp,grid):std::shared_ptr<const std::vector<std::uint8_t>>();const std::vector<std::uint8_t>*gfGearE=gear?&gear->tileFlag:nullptr;
  const int tmax[3]={out.tnx-1,out.tny-1,out.tnz-1};
  for(const auto&b:gfBar){const bool L0=((b.m0>>L)&1u)!=0,L1=((b.m1>>L)&1u)!=0;if(L0==L1)continue;const unsigned far=L0?2u:1u;const Region&Rg=*b.R;
   int lo[3],hi[3];for(int a=0;a<3;++a){const double e=b.rb*std::sqrt(Rg.u[a]*Rg.u[a]+Rg.w[a]*Rg.w[a])+.30*std::abs(Rg.n[a]);
    lo[a]=std::max(0,int(std::floor((Rg.C[a]-e-grid.origin[a])/grid.pitch))>>3);hi[a]=std::min(tmax[a],int(std::floor((Rg.C[a]+e-grid.origin[a])/grid.pitch))>>3);}
   for(int tz=lo[2];tz<=hi[2];++tz)for(int ty=lo[1];ty<=hi[1];++ty)for(int tx=lo[0];tx<=hi[0];++tx){if(roi.in(tx,ty,tz))continue;const std::size_t i=out.idx(tx,ty,tz);
    if(!((gfGearE&&i<gfGearE->size()&&(*gfGearE)[i])||(gfPadE&&i<gfPadE->size()&&(*gfPadE)[i])))continue;
    const double cx=grid.origin.x+(tx*8+4)*grid.pitch-Rg.C[0],cy=grid.origin.y+(ty*8+4)*grid.pitch-Rg.C[1],cz=grid.origin.z+(tz*8+4)*grid.pitch-Rg.C[2];
    if(std::abs(cx*Rg.n[0]+cy*Rg.n[1]+cz*Rg.n[2])>.30)continue;
    const unsigned c=gfTile(b,tx,ty,tz);if(c!=far&&c!=3u)continue;roi.setZ(tx,ty,tz,tz);++out.gfExtTiles;}}}
 out.roiW=roi.w;out.roiNx=roi.tnx;out.roiNy=roi.tny;out.roiWz=roi.wz;out.L=int(L);
 const std::vector<std::uint8_t>*padF=tf?gear_cells::padTileFlags(tfp,grid).get():nullptr;const std::vector<std::uint8_t>*gearF=gear?&gear->tileFlag:nullptr;
 lap(tA);
 // the region's tiles: tissue (distance 0, its pieces), gear, padding; flag 1 = in U
 for(int ty=0;ty<roi.tny;++ty)for(int tx=0;tx<roi.tnx;++tx){const std::uint64_t*r=&roi.w[(std::size_t(ty)*roi.tnx+std::size_t(tx))*roi.wz];
  for(int wi=0;wi<roi.wz;++wi)for(std::uint64_t b=r[wi];b;b&=b-1){unsigned long bit=0;
#ifdef _MSC_VER
   _BitScanForward64(&bit,b);
#else
   bit=unsigned(__builtin_ctzll(b));
#endif
   const int tz=wi*64+int(bit);if(tz>=out.tnz)continue;const std::size_t i=out.idx(tx,ty,tz);std::uint32_t m=0;for(std::size_t k=0;k<P;++k)if(ownT[k].in(tx,ty,tz))m|=1u<<k;
   const bool g=gearF&&i<gearF->size()&&(*gearF)[i],pd=padF&&i<padF->size()&&(*padF)[i];if(!m&&!g&&!pd)continue;
   touch(i);S.flag[i]=1;ulist.push_back(std::uint32_t(i));if(m){S.mask[i]=m;S.dist[i]=0;frontier.push_back(std::uint32_t(i));}
   if(!gfBar.empty()){std::uint32_t gc=0;for(std::size_t b=0;b<gfBar.size();++b)gc|=gfTile(gfBar[b],tx,ty,tz)<<(2*b);S.gfc[i]=gc;}}} // gearfine: the tile's barrier codes
 out.uTiles=ulist.size();out.ulist=ulist;lap(tB); // tiles: breadth first (26 neighbours); a tile two pieces reach at the same step keeps both
 for(std::uint16_t d=0;!frontier.empty()&&d<65534;++d){next.clear();
  for(const auto i:frontier){const int tx=int(i%std::size_t(out.tnx)),ty=int((i/std::size_t(out.tnx))%std::size_t(out.tny)),tz=int(i/(std::size_t(out.tnx)*out.tny));
   const std::uint32_t m=S.mask[i];
   for(int dz=-1;dz<=1;++dz)for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){if(!dx&&!dy&&!dz)continue;const int x=tx+dx,y=ty+dy,z=tz+dz;if(!out.inside(x,y,z))continue;
    const std::size_t j=out.idx(x,y,z);if(!(flagAt(j)&1u))continue;
    std::uint32_t mm=m; // gearfine: a piece on one side of a barrier does not enter a tile wholly on its other side from a tile that is not
    if(!gfBar.empty()){const std::uint32_t ci=S.gfc[i],cj=S.gfc[j];if(ci!=cj)for(std::size_t b=0;b<gfBar.size();++b){const unsigned a=(ci>>(2*b))&3u,c=(cj>>(2*b))&3u;
      if(c==2u&&a!=2u)mm&=~gfBar[b].m0;else if(c==1u&&a!=1u)mm&=~gfBar[b].m1;}if(!mm)continue;}
    if(S.dist[j]==65535){S.dist[j]=std::uint16_t(d+1);S.mask[j]=mm;next.push_back(std::uint32_t(j));}else if(S.dist[j]==d+1)S.mask[j]|=mm;}}
  frontier.swap(next);}
 // gearfine: tiles no piece reached without crossing a barrier (gear hanging past the surface that only the far side's gear
 // holds): the pieces they touch, 26 neighbours from every reached tile, as before the barriers
 if(!gfBar.empty()){std::size_t left=0;for(const auto ui:ulist)left+=S.dist[ui]==65535?1u:0u;out.gfLeftTiles=left;
  if(left){frontier.clear();for(const auto ui:ulist)if(S.dist[ui]!=65535&&S.mask[ui])frontier.push_back(ui);
   for(std::uint16_t d=60000;!frontier.empty()&&d<65533;++d){next.clear();
    for(const auto i:frontier){const int tx=int(i%std::size_t(out.tnx)),ty=int((i/std::size_t(out.tnx))%std::size_t(out.tny)),tz=int(i/(std::size_t(out.tnx)*out.tny));
     const std::uint32_t m=S.mask[i];
     for(int dz=-1;dz<=1;++dz)for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){if(!dx&&!dy&&!dz)continue;const int x=tx+dx,y=ty+dy,z=tz+dz;if(!out.inside(x,y,z))continue;
      const std::size_t j=out.idx(x,y,z);if(!(flagAt(j)&1u))continue;
      if(S.dist[j]==65535){S.dist[j]=std::uint16_t(d+1);S.mask[j]=m;next.push_back(std::uint32_t(j));}else if(S.dist[j]==d+1)S.mask[j]|=m;}}
    frontier.swap(next);}}}
 lap(tC);
 // the tiles where owners meet: two owners, or a neighbour owned by another piece (flag 2)
 std::vector<std::uint32_t> sl;
 for(const auto ui:ulist){const std::size_t i=ui;const std::uint32_t m=S.mask[i];if(!m)continue;bool b=(m&(m-1))!=0;
  if(!b){const int tx=int(i%std::size_t(out.tnx)),ty=int((i/std::size_t(out.tnx))%std::size_t(out.tny)),tz=int(i/(std::size_t(out.tnx)*out.tny));
   static const int f6[6][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
   for(int d=0;d<6&&!b;++d){const int x=tx+f6[d][0],y=ty+f6[d][1],z=tz+f6[d][2];if(!out.inside(x,y,z))continue;
    const std::uint32_t n=out.maskAt(out.idx(x,y,z));b=n&&n!=m;}}
  if(b){S.flag[i]|=2u;sl.push_back(std::uint32_t(i));}}
 std::sort(sl.begin(),sl.end());lap(tD);
 // cells of the meeting tiles: 0 other, 1 gear or padding, 2 + k tissue of piece k
 int off[3]={0,0,0};bool aligned=tf&&std::abs(double(tf->pitch)-grid.pitch)<1e-6*grid.pitch;
 if(tf)for(int a=0;a<3;++a){const double o=(grid.origin[a]-double(tf->origin[std::size_t(a)]))/grid.pitch;off[a]=int(std::lround(o));aligned=aligned&&std::abs(o-off[a])<1e-3;}
 auto padAt=[&](int x,int y,int z)->bool{if(!tf)return false;
  if(!aligned)return (tf->sample(grid.origin.x+(x+.5)*grid.pitch,grid.origin.y+(y+.5)*grid.pitch,grid.origin.z+(z+.5)*grid.pitch)&64u)!=0;
  const int X=x+off[0],Y=y+off[1],Z=z+off[2];if(X<0||Y<0||Z<0||unsigned(X)>=tf->size[0]||unsigned(Y)>=tf->size[1]||unsigned(Z)>=tf->size[2])return false;
  const auto v=tf->page[(std::size_t(Z>>3)*tf->pages[1]+unsigned(Y>>3))*tf->pages[0]+unsigned(X>>3)+painter_detail::TissueField::headerWords];
  if(v&painter_detail::TissueField::uniformBit)return (v&64u)!=0;return (tf->at(X,Y,Z)&64u)!=0;};
 const std::size_t NS=sl.size();std::vector<std::array<std::uint8_t,512>> code(NS);out.tiles.resize(NS);std::vector<std::uint32_t> q,nq; // q packed: slot << 9 | cell
 for(std::size_t s=0;s<NS;++s){const std::size_t i=sl[s];S.slot[i]=std::int32_t(s);
  out.tiles[s].c=Cell{int(i%std::size_t(out.tnx)),int((i/std::size_t(out.tnx))%std::size_t(out.tny)),int(i/(std::size_t(out.tnx)*out.tny))};out.tiles[s].own.fill(255);}
 // gearfine: each split tile across a barrier holds its cells' sides (bit set = side 1), from the surface's values at the tile's
 // corners (trilinear: exact for the plane; the offset's bend inside 16 mm is far below a cell). gfSide: 0 free, 1 side 0, 2 side 1
 const std::vector<std::uint32_t>&gfc=S.gfc; // (the calling thread's scratch, read by the workers through this reference)
 std::vector<std::int8_t> gfT(NS,-1);std::vector<std::array<std::uint64_t,8>> gfB(gfBar.empty()?std::size_t(0):NS);
 if(!gfBar.empty())lowpoly_pieces::eachBlock(NS,16,[&](std::size_t first,std::size_t last,std::size_t){for(std::size_t s=first;s<last;++s){const std::uint32_t gc=gfc[sl[s]];
  for(std::size_t b=0;b<gfBar.size();++b){if(((gc>>(2*b))&3u)!=3u)continue;const Cell tc=out.tiles[s].c;double cv[8];bool okc=true;
   for(int q=0;q<8&&okc;++q)okc=gfV(gfBar[b],grid.origin.x+(tc.x*8+((q&1)?8:0))*grid.pitch,grid.origin.y+(tc.y*8+((q&2)?8:0))*grid.pitch,grid.origin.z+(tc.z*8+((q&4)?8:0))*grid.pitch,cv[q]);
   if(!okc)continue;gfT[s]=std::int8_t(b);auto&bits=gfB[s];bits.fill(0);
   for(int x=0;x<8;++x){const double fx=(x+.5)/8;for(int y=0;y<8;++y){const double fy=(y+.5)/8;for(int z=0;z<8;++z){const double fz=(z+.5)/8;
    const double v=((cv[0]*(1-fx)+cv[1]*fx)*(1-fy)+(cv[2]*(1-fx)+cv[3]*fx)*fy)*(1-fz)+((cv[4]*(1-fx)+cv[5]*fx)*(1-fy)+(cv[6]*(1-fx)+cv[7]*fx)*fy)*fz;
    if(v>=0)bits[std::size_t(x)]|=std::uint64_t(1)<<(y*8+z);}}}
   break;}}});
 auto gfSide=[&](std::size_t s,int i,std::size_t b)->unsigned{const unsigned c=(gfc[sl[s]]>>(2*b))&3u;if(c!=3u)return c;if(gfT[s]!=int(b))return 0u;
  return ((gfB[s][std::size_t(i>>6)]>>(i&63))&1u)?2u:1u;};
 auto gfCross=[&](std::size_t s,int i,std::size_t s2,int i2)->bool{for(std::size_t b=0;b<gfBar.size();++b){const unsigned a=gfSide(s,i,b),c=gfSide(s2,i2,b);if(a&&c&&a!=c)return true;}return false;};
 std::vector<tear_atlas::Box> rbox(freshIdx.size());for(std::size_t f=0;f<freshIdx.size();++f){const auto z=tear_atlas::zone(*R[freshIdx[f]].T);if(z.ok)rbox[f]=z.region;}
 const double halfDiag=.87*8*grid.pitch;std::atomic<std::size_t> gearC{0},padC{0},seedC{0};
 // classification (0 other, 1 gear or padding, 2 + k tissue of piece k) and the tear-side seeds (inside a fresh tear's region within
 // 44 mm of its plane: the one piece on the cell's side), tile by tile in parallel
 lowpoly_pieces::eachBlock(NS,8,[&](std::size_t first,std::size_t last,std::size_t){std::size_t gc=0,pc=0,sc2=0;
  for(std::size_t s=first;s<last;++s){const Cell tc=out.tiles[s].c;const int tx=tc.x,ty=tc.y,tz=tc.z;auto&c=code[s];c.fill(0);
   for(int y=ty*8;y<ty*8+8;++y){auto col=std::lower_bound(volume.columns.begin(),volume.columns.end(),std::array<int,2>{{tx*8,y}},
     [](const gfy_connectivity::Column&a,const std::array<int,2>&qq){return gfy_connectivity::xyLess(a.x,a.y,qq[0],qq[1]);});
    for(;col!=volume.columns.end()&&col->y==y&&col->x<tx*8+8;++col)for(auto sp0=col->first;sp0<col->first+col->count;++sp0){const auto&sp=volume.spans[sp0];
     if(sp.component>=pieceOf.size()||pieceOf[sp.component]<0)continue;const int lo=std::max(sp.lo,tz*8),hi=std::min(sp.hi,tz*8+8);
     for(int z=lo;z<hi;++z)c[std::size_t(((col->x&7)*8+(y&7))*8+(z&7))]=std::uint8_t(2+pieceOf[sp.component]);}}
   const core::Tile*gt=gear?gear->tile(tx,ty,tz):nullptr;const auto gr=grid.tiles.find(Cell{tx,ty,tz});const core::Tile*rm=gr==grid.tiles.end()?nullptr:&gr->second;
   for(int x=0;x<8;++x)for(int y=0;y<8;++y)for(int z=0;z<8;++z){const std::size_t qq=std::size_t((x*8+y)*8+z);auto&v=c[qq];if(v)continue;
    if(rm&&((rm->removed[std::size_t(x)]>>(y*8+z))&1u))continue;
    const bool g=gt&&((gt->removed[std::size_t(x)]>>(y*8+z))&1u);const bool pd=!g&&padAt(tx*8+x,ty*8+y,tz*8+z);
    if(g||pd){v=1;if(g)++gc;else ++pc;}}
   const tear_atlas::Box tb{{grid.origin.x+tx*8*grid.pitch,grid.origin.y+ty*8*grid.pitch,grid.origin.z+tz*8*grid.pitch},
    {grid.origin.x+(tx*8+8)*grid.pitch,grid.origin.y+(ty*8+8)*grid.pitch,grid.origin.z+(tz*8+8)*grid.pitch}};
   const double cxw=.5*(tb.lo[0]+tb.hi[0]),cyw=.5*(tb.lo[1]+tb.hi[1]),czw=.5*(tb.lo[2]+tb.hi[2]);
   std::uint32_t fsm=0;for(std::size_t f=0;f<freshIdx.size()&&f<32;++f){if(!rbox[f].valid()||!rbox[f].meets(tb))continue;const auto&Rg=R[freshIdx[f]];
    const double s0=(cxw-Rg.C[0])*Rg.n[0]+(cyw-Rg.C[1])*Rg.n[1]+(czw-Rg.C[2])*Rg.n[2];if(std::abs(s0)>.044+halfDiag)continue;fsm|=1u<<f;}
   if(!fsm)continue;
   for(int i=0;i<512;++i){if(c[std::size_t(i)]!=1)continue;const double x=grid.origin.x+(tx*8+(i>>6)+.5)*grid.pitch,y=grid.origin.y+(ty*8+((i>>3)&7)+.5)*grid.pitch,
     z=grid.origin.z+(tz*8+(i&7)+.5)*grid.pitch;std::uint32_t cand=P>=32?~0u:((1u<<P)-1u);bool constrained=false;
    for(std::uint32_t m=fsm;m;m&=m-1){const int f=Owners::first(m);const auto&Rg=R[freshIdx[std::size_t(f)]];
     const double s0=(x-Rg.C[0])*Rg.n[0]+(y-Rg.C[1])*Rg.n[1]+(z-Rg.C[2])*Rg.n[2];if(std::abs(s0)>.044)continue;
     const int sd=Rg.side(x,y,z);if(sd<0)continue;constrained=true;cand&=sideMask[freshIdx[std::size_t(f)]][std::size_t(sd)];}
    if(!constrained||!cand||(cand&(cand-1)))continue;
    out.tiles[s].own[std::size_t(i)]=std::uint8_t(Owners::first(cand));++sc2;}}
  gearC+=gc;padC+=pc;seedC+=sc2;});
 out.gearCells=gearC;out.padCells=padC;out.seeded=seedC;out.bTiles=NS;
 // seeds across a tile face from a tile one piece owns whole (where no tear-side seed is)
 for(std::size_t s=0;s<NS;++s){const Cell&tc=out.tiles[s].c;
  for(int d=0;d<6;++d){const int dx=d==0?1:(d==1?-1:0),dy=d==2?1:(d==3?-1:0),dz=d==4?1:(d==5?-1:0);const int x=tc.x+dx,y=tc.y+dy,z=tc.z+dz;if(!out.inside(x,y,z))continue;
   const std::size_t j=out.idx(x,y,z);if(out.slotAt(j)>=0)continue;const std::uint32_t m=out.maskAt(j);if(!m||(m&(m-1)))continue;const auto k=std::uint8_t(Owners::first(m));
   for(int u=0;u<8;++u)for(int w=0;w<8;++w){
    const int ix=dx?(dx>0?7:0):u,iy=dy?(dy>0?7:0):(dx?u:w),iz=dz?(dz>0?7:0):w;
    const std::size_t qq=std::size_t((ix*8+iy)*8+iz);if(code[s][qq]!=1||out.tiles[s].own[qq]!=255)continue;
    if(!gfBar.empty()){bool cr=false;const std::uint32_t gj=gfc[j];for(std::size_t b=0;b<gfBar.size()&&!cr;++b){const unsigned cj=(gj>>(2*b))&3u,cs=gfSide(s,int(qq),b);cr=(cj==1u||cj==2u)&&cs&&cs!=cj;}if(cr)continue;} // gearfine: not across a barrier
    out.tiles[s].own[qq]=k;}}}
 auto nb=[&](std::int32_t s,int i,int d,std::int32_t&s2,int&i2)->bool{static const int d6[6][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
  const Cell&c=out.tiles[std::size_t(s)].c;int x=(i>>6)+d6[d][0],y=((i>>3)&7)+d6[d][1],z=(i&7)+d6[d][2];int tx=c.x,ty=c.y,tz=c.z;
  if(x<0){x=7;--tx;}else if(x>7){x=0;++tx;}if(y<0){y=7;--ty;}else if(y>7){y=0;++ty;}if(z<0){z=7;--tz;}else if(z>7){z=0;++tz;}
  if(tx==c.x&&ty==c.y&&tz==c.z)s2=s;else{if(!out.inside(tx,ty,tz))return false;s2=out.slotAt(out.idx(tx,ty,tz));}i2=(x*8+y)*8+z;return s2>=0;};
 // seeds 3: next to the tissue
 const auto c0=core::Clock::now();
 lowpoly_pieces::eachBlock(NS,16,[&](std::size_t first,std::size_t last,std::size_t){for(std::size_t s=first;s<last;++s)for(int i=0;i<512;++i){
  if(code[s][std::size_t(i)]!=1||out.tiles[s].own[std::size_t(i)]!=255)continue;
  for(int d=0;d<6;++d){std::int32_t s2;int i2;if(!nb(std::int32_t(s),i,d,s2,i2))continue;const auto v=code[std::size_t(s2)][std::size_t(i2)];
   if(v>=2&&!(!gfBar.empty()&&gfCross(s,i,std::size_t(s2),i2))){out.tiles[s].own[std::size_t(i)]=std::uint8_t(v-2);break;}}}}); // gearfine: not across a barrier (tissue seeds)
 q.clear();for(std::size_t s=0;s<NS;++s)for(int i=0;i<512;++i)if(code[s][std::size_t(i)]==1&&out.tiles[s].own[std::size_t(i)]!=255)q.push_back(std::uint32_t(s)<<9|std::uint32_t(i));
 const double tSeeds=core::ms(c0);const auto c1=core::Clock::now();
 // the meeting tiles in connected groups (face neighbours): a group's search never leaves it, so the groups run side by side
 std::vector<std::int32_t> grp(NS,-1);std::vector<std::vector<std::uint32_t>> grpSeeds;
 {std::vector<std::int32_t> st;static const int f6[6][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
  for(std::size_t s0=0;s0<NS;++s0){if(grp[s0]>=0)continue;const std::int32_t g=std::int32_t(grpSeeds.size());grpSeeds.emplace_back();grp[s0]=g;st.assign(1,std::int32_t(s0));
   while(!st.empty()){const std::int32_t s=st.back();st.pop_back();const Cell&c=out.tiles[std::size_t(s)].c;
    for(const auto&f:f6){const int x=c.x+f[0],y=c.y+f[1],z=c.z+f[2];if(!out.inside(x,y,z))continue;const std::int32_t s2=out.slotAt(out.idx(x,y,z));
     if(s2>=0&&grp[std::size_t(s2)]<0){grp[std::size_t(s2)]=g;st.push_back(s2);}}}}}
 for(const auto e:q)grpSeeds[std::size_t(grp[e>>9])].push_back(e);
 std::vector<std::vector<std::int32_t>> gfSlots(gfBar.empty()?std::size_t(0):grpSeeds.size()); // gearfine: each group's split tiles
 if(!gfBar.empty())for(std::size_t s=0;s<NS;++s)gfSlots[std::size_t(grp[s])].push_back(std::int32_t(s));
 lowpoly_pieces::eachBlock(grpSeeds.size(),1,[&](std::size_t first,std::size_t last,std::size_t){std::vector<std::uint32_t> a,b;
  for(std::size_t g=first;g<last;++g){a=grpSeeds[g];
   while(!a.empty()){b.clear();
    for(const auto e:a){const std::int32_t s=std::int32_t(e>>9);const int i=int(e&511u);const auto own=out.tiles[std::size_t(s)].own[std::size_t(i)];
     for(int d=0;d<6;++d){std::int32_t s2;int i2;if(!nb(s,i,d,s2,i2))continue;
      if(code[std::size_t(s2)][std::size_t(i2)]!=1||out.tiles[std::size_t(s2)].own[std::size_t(i2)]!=255)continue;
      if(!gfBar.empty()&&gfCross(std::size_t(s),i,std::size_t(s2),i2))continue; // gearfine: no step across a barrier
      out.tiles[std::size_t(s2)].own[std::size_t(i2)]=own;
      b.push_back(std::uint32_t(s2)<<9|std::uint32_t(i2));}}
    a.swap(b);}
   // gearfine: the group's gear / padding cells no piece reached without crossing: the piece they touch (crossing allowed)
   if(!gfBar.empty()){a.clear();
    for(const auto s:gfSlots[g])for(int i=0;i<512;++i){if(code[std::size_t(s)][std::size_t(i)]!=1||out.tiles[std::size_t(s)].own[std::size_t(i)]==255)continue;bool open=false;
     for(int d=0;d<6&&!open;++d){std::int32_t s2;int i2;if(!nb(s,i,d,s2,i2))continue;open=code[std::size_t(s2)][std::size_t(i2)]==1&&out.tiles[std::size_t(s2)].own[std::size_t(i2)]==255;}
     if(open)a.push_back(std::uint32_t(s)<<9|std::uint32_t(i));}
    while(!a.empty()){b.clear();
     for(const auto e:a){const std::int32_t s=std::int32_t(e>>9);const int i=int(e&511u);const auto own=out.tiles[std::size_t(s)].own[std::size_t(i)];
      for(int d=0;d<6;++d){std::int32_t s2;int i2;if(!nb(s,i,d,s2,i2))continue;
       if(code[std::size_t(s2)][std::size_t(i2)]!=1||out.tiles[std::size_t(s2)].own[std::size_t(i2)]!=255)continue;out.tiles[std::size_t(s2)].own[std::size_t(i2)]=own;
       b.push_back(std::uint32_t(s2)<<9|std::uint32_t(i2));}}
     a.swap(b);}}}});
 if(!gfBar.empty()){std::size_t leftC=0; // gearfine: a split tile's gear / padding cell still without a piece takes the tile's first piece
  for(std::size_t s=0;s<NS;++s){const std::uint32_t m=S.mask[sl[s]];for(int i=0;i<512;++i){if(code[s][std::size_t(i)]!=1||out.tiles[s].own[std::size_t(i)]!=255)continue;++leftC;
    if(m)out.tiles[s].own[std::size_t(i)]=std::uint8_t(Owners::first(m));}}
  out.gfLeftCells=leftC;}
 const double tBfs=core::ms(c1);std::atomic<std::size_t> ownedC{0};
 lowpoly_pieces::eachBlock(NS,16,[&](std::size_t first,std::size_t last,std::size_t){std::size_t n=0;for(std::size_t s=first;s<last;++s)for(int i=0;i<512;++i){const auto v=code[s][std::size_t(i)];
  if(v>=2)out.tiles[s].own[std::size_t(i)]=std::uint8_t(v-2);else if(v!=1)out.tiles[s].own[std::size_t(i)]=255;else n+=out.tiles[s].own[std::size_t(i)]!=255;}ownedC+=n;});
 out.owned=ownedC;
 lap(tE);out.ms=core::ms(t0);
 if(prof)std::fprintf(stderr,"CFOWNPROF tissue=%.2f gear+pad=%.2f tile_bfs=%.2f split=%.2f cells=%.2f (seeds %.2f bfs %.2f) total=%.2f u=%zu split=%zu\n",tA,tB,tC,tD,tE,tSeeds,tBfs,out.ms,out.uTiles,sl.size());
 return out;}
// cutfix: the piece's thickness on its side of each tear (TearAtlas.h depthScale): the farthest plane distance of its own cells.
// -1 (the baked depths) for a crack, a block without a header, or a piece of more than 200k cells (1.6 litres at 2 mm: no tear zone
// reaches across it)
inline std::vector<double> thickness(const Grid&grid,const gfy_connectivity::Topology&volume,std::uint32_t component,const CellBox&box,
    const std::vector<gfy_packed::TearRef>&tears){
 std::vector<double> out(tears.size(),-1.);
 if(!box.valid()||component>=volume.components.size()||volume.components[component].cells>200000)return out;
 struct Pl {double C[3],n[3];double sign;std::size_t i;};std::vector<Pl> pl;
 for(std::size_t i=0;i<tears.size();++i){const auto&tr=tears[i];if(!tr.block||tr.side>1||tr.block->header.size()!=104)continue;const auto&h=tr.block->header;
  Pl p;for(int a=0;a<3;++a){p.C[a]=tear_atlas::f32(h,std::size_t(a));p.n[a]=tear_atlas::f32(h,std::size_t(12+a));}p.sign=tr.side?1.:-1.;p.i=i;pl.push_back(p);out[i]=0;}
 if(pl.empty())return out;
 for(int y=box.lo[1];y<box.hi[1];++y){
  auto col=std::lower_bound(volume.columns.begin(),volume.columns.end(),std::array<int,2>{{box.lo[0],y}},
   [](const gfy_connectivity::Column&a,const std::array<int,2>&q){return gfy_connectivity::xyLess(a.x,a.y,q[0],q[1]);});
  for(;col!=volume.columns.end()&&col->y==y&&col->x<box.hi[0];++col)for(auto s=col->first;s<col->first+col->count;++s){const auto&sp=volume.spans[s];
   if(sp.component!=component||sp.hi<=sp.lo)continue;
   const double px=grid.origin.x+(col->x+.5)*grid.pitch,py=grid.origin.y+(y+.5)*grid.pitch,z0=grid.origin.z+(sp.lo+.5)*grid.pitch,z1=grid.origin.z+(sp.hi-.5)*grid.pitch;
   for(const auto&p:pl){const double b=(px-p.C[0])*p.n[0]+(py-p.C[1])*p.n[1];
    const double s0=p.sign*(b+(z0-p.C[2])*p.n[2]),s1=p.sign*(b+(z1-p.C[2])*p.n[2]);out[p.i]=std::max(out[p.i],std::max(s0,s1));}}}
 return out;}
// cutfix: the other pieces' cells inside a box of cells (the piece's own cells' box): partial-tile marks
// (a tile `inRegion` says lies in a tear's region is left to that tear: inside its region the tear decides the sides, and the march
// must meet the other side's tissue there as the tear's - its torn face - not as removed cells)
// tileClass(tile): 0 wholly outside every region (mark), 1 wholly inside one (skip), 2 across a region's boundary: cellIn(x, y, z)
// decides per cell
template<class TileClass,class CellIn>
// (boxes: every component's cell box, by component id: only the other components' boxes inside b are scanned)
inline void otherCells(const gfy_connectivity::Topology&volume,std::uint32_t mine,const CellBox&b0,const std::vector<CellBox>&boxes,
    std::unordered_map<Cell,core::Tile,core::CellHash>&out,TileClass&&tileClass,CellIn&&cellIn){
 if(!b0.valid())return;std::unordered_map<Cell,char,core::CellHash> regionTile;
 auto cls=[&](const Cell&t){auto it=regionTile.find(t);if(it!=regionTile.end())return int(it->second);const char v=char(tileClass(t));regionTile.emplace(t,v);return int(v);};
 for(std::uint32_t id=1;id<boxes.size();++id){if(id==mine||!boxes[id].valid())continue;CellBox b;
  for(int a=0;a<3;++a){b.lo[a]=std::max(b0.lo[a],boxes[id].lo[a]);b.hi[a]=std::min(b0.hi[a],boxes[id].hi[a]);}if(!b.valid())continue;
  const int t0[3]={b.lo[0]>>3,b.lo[1]>>3,b.lo[2]>>3},tn[3]={((b.hi[0]-1)>>3)-t0[0]+1,((b.hi[1]-1)>>3)-t0[1]+1,((b.hi[2]-1)>>3)-t0[2]+1};
  std::vector<signed char> dense(std::size_t(tn[0])*tn[1]*tn[2],-1);
  auto clsD=[&](const Cell&t){auto&v=dense[(std::size_t(t.z-t0[2])*tn[1]+std::size_t(t.y-t0[1]))*tn[0]+std::size_t(t.x-t0[0])];if(v<0)v=signed char(cls(t));return int(v);};
 for(int y=b.lo[1];y<b.hi[1];++y){
  auto col=std::lower_bound(volume.columns.begin(),volume.columns.end(),std::array<int,2>{{b.lo[0],y}},
   [](const gfy_connectivity::Column&a,const std::array<int,2>&q){return gfy_connectivity::xyLess(a.x,a.y,q[0],q[1]);});
  for(;col!=volume.columns.end()&&col->y==y&&col->x<b.hi[0];++col)for(auto s=col->first;s<col->first+col->count;++s){const auto&sp=volume.spans[s];
   if(sp.component!=id)continue;const int lo=std::max(sp.lo,b.lo[2]),hi=std::min(sp.hi,b.hi[2]);
   for(int z=lo;z<hi;){const int n=std::min(8-(z&7),hi-z);const std::uint64_t run=((std::uint64_t(1)<<n)-1)<<(z&7);const Cell t{col->x>>3,y>>3,z>>3};
    const int c=clsD(t);
    if(c==0)out[t].removed[std::size_t(col->x&7)]|=run<<((y&7)*8);
    else if(c==2)for(int zz=z;zz<z+n;++zz)if(!cellIn(col->x,y,zz))out[t].removed[std::size_t(col->x&7)]|=std::uint64_t(1)<<((y&7)*8+(zz&7));
    z+=n;}}}}}
// every component's cell box, by component id
inline std::vector<CellBox> componentBoxes(const gfy_connectivity::Topology&volume){std::vector<CellBox> out(volume.components.size());
 for(const auto&c:volume.columns)for(auto s=c.first;s<c.first+c.count;++s){const auto&sp=volume.spans[s];if(sp.component&&sp.component<out.size())out[sp.component].add(c.x,c.y,sp.lo,sp.hi);}
 return out;}
// cutfix (every cut that leaves a slice at or above the piece minimum takes it off): LiveSeparationWorker_impl.h erases a small
// boneless component (below the boneless-scrap volume) that touches the cut as debris. A slice cut off between this cut and an
// earlier one is no scrap even when the shots took its bone and most of its core (SA: a 2 cm forearm slice of 25 cm3): a small
// component lying next to the sheet of an EARLIER cut the mesh holds (side 0 / 1: a tear its parent's separation gave it, on that
// side) AND next to a fresh decide-once tear's sheet (8 or more samples 2.5 / 4.5 / 7.5 cells off a sheet over its footprint) is
// marked. A flap blown loose beside one cut lies next to one sheet only; the fragments of one event (a head burst's, a blast's)
// lie next to that event's fresh sheets only: they stay debris as before.
// (speed: the earlier cuts' sheets are sampled only where a small component's box is, the fresh sheets only for the components
// found next to an earlier sheet)
inline void tearSlices(const gfy_packed::Mesh*published,const Grid&grid,const gfy_connectivity::Topology&volume,const std::vector<char>&small,std::vector<char>&slice){
 slice.assign(volume.components.size(),0);if(!published)return;
 bool anySmall=false,anyFresh=false,anyOld=false;for(const auto s:small)anySmall=anySmall||s!=0;
 for(const auto&tr:published->tears){if(!tr.block||tr.block->section.empty())continue;if(fresh(tr))anyFresh=true;else if(tr.side<=1)anyOld=true;}
 if(!anySmall||!anyFresh||!anyOld)return;
 static const bool log=std::getenv("GFY_M3_LOG")!=nullptr||std::getenv("GFY_PIECE_LOG")!=nullptr;
 const double p=grid.pitch,ex[3]={2.5*p,4.5*p,7.5*p};const std::size_t K=published->tears.size(),NCp=volume.components.size();
 const auto boxes=componentBoxes(volume);
 // does any wanted component's box meet the box of a tear's samples (|offset| <= offMax, 7.5 cells off it)?
 auto meets=[&](const Region&R,const std::vector<std::uint32_t>&want){
  const double a0=R.O+.5*R.cell,a1=R.O+(R.N-.5)*R.cell,sm=tear_decide::offMax+7.5*p,org[3]={grid.origin.x,grid.origin.y,grid.origin.z};int lo[3],hi[3];
  for(int k=0;k<3;++k){double mn=1e30,mx=-1e30;for(int q=0;q<8;++q){const double A=(q&1)?a1:a0,B=(q&2)?a1:a0,S=(q&4)?sm:-sm;const double v=R.C[k]+A*R.u[k]+B*R.w[k]+S*R.n[k];
    mn=std::min(mn,v);mx=std::max(mx,v);}lo[k]=int(std::floor((mn-org[k])/p))-1;hi[k]=int(std::floor((mx-org[k])/p))+2;}
  for(const auto id:want){const auto&c=boxes[id];if(c.valid()&&c.lo[0]<hi[0]&&lo[0]<c.hi[0]&&c.lo[1]<hi[1]&&lo[1]<c.hi[1]&&c.lo[2]<hi[2]&&lo[2]<c.hi[2])return true;}
  return false;};
 // the samples next to tear t's sheet (an earlier cut's: on the side the mesh holds), per wanted component
 auto sample=[&](std::size_t t,const std::vector<char>&wanted,std::map<std::uint32_t,long long>&next){const auto&tr=published->tears[t];const Region R(*tr.block);
  for(int qb=0;qb<R.N;++qb)for(int qa=0;qa<R.N;++qa){const std::size_t q=std::size_t(qb)*R.N+std::size_t(qa);if(!(int(tr.block->section[4*q+1]+.5f)&1))continue;
   const double a=R.O+(qa+.5)*R.cell,b=R.O+(qb+.5)*R.cell;const double o=R.offset(a,b);
   for(int sd=0;sd<2;++sd){if(!fresh(tr)&&sd!=int(tr.side))continue;
    for(const double e:ex){const double s=o+(sd?1:-1)*e;double x[3];for(int k=0;k<3;++k)x[k]=R.C[k]+a*R.u[k]+b*R.w[k]+s*R.n[k];
     const int cx=int(std::floor((x[0]-grid.origin.x)/p)),cy=int(std::floor((x[1]-grid.origin.y)/p)),cz=int(std::floor((x[2]-grid.origin.z)/p));
     if(cx<0||cy<0||cz<0||cx>=grid.size.x||cy>=grid.size.y||cz>=grid.size.z)continue;const auto id=volume.at(cx,cy,cz);
     if(id&&id<wanted.size()&&wanted[id])++next[id];}}}};
 std::vector<std::uint32_t> smallIds;for(std::uint32_t id=1;id<small.size()&&id<NCp;++id)if(small[id])smallIds.push_back(id);
 std::vector<char> wanted(NCp,0);for(const auto id:smallIds)wanted[id]=1;
 // 1: the earlier cuts' sheets
 std::map<std::uint32_t,std::pair<int,int>> sheets; // component -> (fresh sheets, earlier sheets) it lies next to
 for(std::size_t t=0;t<K;++t){const auto&tr=published->tears[t];if(!tr.block||tr.block->section.empty()||fresh(tr)||tr.side>1)continue;
  if(!meets(Region(*tr.block),smallIds))continue;std::map<std::uint32_t,long long> next;sample(t,wanted,next);
  for(const auto&v:next)if(v.second>=8)++sheets[v.first].second;}
 if(sheets.empty())return;
 // 2: the fresh sheets, for the components next to an earlier sheet
 std::vector<std::uint32_t> candIds;std::vector<char> cand(NCp,0);for(const auto&c:sheets){candIds.push_back(c.first);cand[c.first]=1;}
 for(std::size_t t=0;t<K;++t){const auto&tr=published->tears[t];if(!tr.block||tr.block->section.empty()||!fresh(tr))continue;
  if(!meets(Region(*tr.block),candIds))continue;std::map<std::uint32_t,long long> next;sample(t,cand,next);
  for(const auto&v:next)if(v.second>=8)++sheets[v.first].first;}
 for(const auto&c:sheets){const bool cut=c.second.first>=1;if(cut)slice[c.first]=1;
  if(log)std::fprintf(stderr,"tear_slice component=%u cells=%llu fresh_sheets=%d earlier_sheets=%d piece=%d\n",c.first,(unsigned long long)volume.components[c.first].cells,
   c.second.first,c.second.second,int(cut));}}
// headfix: piece k's copy of a head burst record - inherited (side 3): kept as it is (the groups it holds are in it); fresh: a copy
// holding the groups whose sample points (the record's section: x, y, z, group) lie in the piece (none: not attached)
inline std::shared_ptr<const gfy_packed::TearBlock> burstFor(const gfy_packed::TearRef&tr,const PieceAt&pieceAt,std::size_t k){
 if(!fresh(tr))return tr.side==3?tr.block:nullptr;
 const auto&b=*tr.block;std::uint32_t count[32]={};
 const std::size_t ns=head_burst_record::sampleFloats(b);for(std::size_t i=0;i+3<ns;i+=4)if(pieceAt(b.section[i],b.section[i+1],b.section[i+2])==int(k))++count[std::min(31u,unsigned(b.section[i+3]))];
 std::uint32_t mask=0;for(unsigned g=0;g<32;++g)if(count[g]>=2)mask|=1u<<g;
 if(!mask||b.header.size()<16)return nullptr;
 auto c=std::make_shared<gfy_packed::TearBlock>(b);c->header[15]=mask;return c;}
// Each piece's side of every published tear: the volume's component next to the tear's sheet and at its reach, over its footprint.
inline void vote(const gfy_packed::Mesh&published,const Grid&grid,const PieceAt&pieceAt,std::size_t P,Stats&st){
 const auto t0=core::Clock::now();const std::size_t K=published.tears.size();st.sig.assign(K,{});st.votes.assign(K,{});
 for(std::size_t t=0;t<K;++t){st.sig[t].fill(-1);if(!published.tears[t].block||tear_atlas::burstRecord(*published.tears[t].block))continue; /* headfix: records apart */const auto&T=*published.tears[t].block;const Region R(T);
  const double p=grid.pitch,ex[4]={2.5*p,4.5*p,7.5*p,T.reach};
  for(int qb=0;qb<R.N;++qb)for(int qa=0;qa<R.N;++qa){const std::size_t q=std::size_t(qb)*R.N+std::size_t(qa);if(!(int(T.section[4*q+1]+.5f)&1))continue;
   const double a=R.O+(qa+.5)*R.cell,b=R.O+(qb+.5)*R.cell;const double o=R.offset(a,b);
   for(int sd=0;sd<2;++sd)for(int ei=0;ei<(fresh(published.tears[t])?4:3);++ei){const double e=ex[ei]; // an inherited tear: next to its sheet only (its reach spans the neighbouring slices)
   const double s=o+(sd?1:-1)*e;double x[3];for(int k=0;k<3;++k)x[k]=R.C[k]+a*R.u[k]+b*R.w[k]+s*R.n[k];
    const int k=pieceAt(x[0],x[1],x[2]);if(k>=0&&std::size_t(k)<P)++st.votes[t][std::size_t(sd)*maxPieces+std::size_t(k)];}}
  for(std::size_t k=0;k<P;++k){const long long A=st.votes[t][k],B=st.votes[t][maxPieces+k],hi=std::max(A,B),lo=std::min(A,B);
   if(hi<4)continue;st.sig[t][k]=signed char((lo>20&&lo*5>hi)?2:(B>A?1:0));}}
 st.voteMs=core::ms(t0);}
inline bool eligible(const gfy_packed::Mesh&published,const Grid&grid,const gfy_connectivity::Topology&volume,const std::vector<std::uint32_t>&components,Stats&st){
 st.tears=published.tears.size();st.fresh=freshCount(published);
 if(published.tears.empty()){st.why="no_tear";return false;} // slice 4: any piece with tears (a split by bullets alone too: no slice-2 field)
 if(components.size()<1||components.size()>maxPieces){st.why="piece_count";return false;}
 vote(published,grid,PieceAt(grid,volume,components),components.size(),st);
 return true;}
// slice 2's pieces: each piece gets the tears it lies on (unsplit: its voted side; inherited: the parent's side when its volume
// lies next to that tear's sheet on that side or any of its triangles lies in the tear's region), no limit
inline void attach(std::vector<bound_surface::Piece>&pieces,const std::vector<std::uint32_t>&components,const gfy_packed::Mesh&published,
    const Grid&grid,const gfy_connectivity::Topology&volume,Stats&st){
 if(published.tears.empty())return;
 if(st.sig.size()!=published.tears.size())vote(published,grid,PieceAt(grid,volume,components),components.size(),st);
 std::vector<Region> R;for(const auto&t:published.tears)R.emplace_back(*t.block);
 const PieceAt burstAt(grid,volume,components); // headfix
 const auto boxes=ownBoxes(volume,components); // cutfix: slice 2's field marks every other cell; the atlas gets the own box
 for(auto&piece:pieces){int k=-1;for(std::size_t j=0;j<components.size();++j)if(components[j]==piece.volume_component)k=int(j);if(k<0||std::size_t(k)>=maxPieces)continue;
  for(std::size_t t=0;t<published.tears.size();++t){const auto&tr=published.tears[t];int side=-1;
   if(tear_atlas::burstRecord(*tr.block)){if(auto c=burstFor(tr,burstAt,std::size_t(k)))piece.mesh.tears.push_back({c,3});continue;} // headfix
   if(fresh(tr))side=st.sig[t][std::size_t(k)];
   else{const bool voted=tr.side<2?st.votes[t][std::size_t(tr.side)*maxPieces+std::size_t(k)]>0:(st.votes[t][std::size_t(k)]+st.votes[t][maxPieces+std::size_t(k)])>0;
    bool tri=false;for(const auto&c:piece.mesh.children){for(const auto&v:c.vertices)if(R[t].side(v.surface[0],v.surface[1],v.surface[2])>=0){tri=true;break;}if(tri)break;}
    if(voted||tri)side=tr.side;}
   if(side<0)continue;
   piece.mesh.tears.push_back({tr.block,std::uint8_t(side)});st.attached=true;}
  const auto own=worldBox(grid,boxes[std::size_t(k)],0);
  const auto th=thickness(grid,volume,components[std::size_t(k)],boxes[std::size_t(k)],piece.mesh.tears);
  piece.mesh.tear_atlas=tear_atlas::build(piece.mesh.tears,own,own.padded(.08),&st.atlas,&th);}}

#include "../HeadBurstPieces.h"
inline std::vector<std::uint8_t> regionDepth(const tear_atlas::Zone&z,const gfy_packed::TearBlock&b){
 const int MX=int(z.mx),MY=int(z.my);std::vector<std::uint8_t> d;if(MX<=0||MY<=0||b.map.size()<std::size_t(MX)*std::size_t(MY)*20)return d;
 d.resize(std::size_t(MX)*std::size_t(MY));
 for(std::size_t mc=0;mc<d.size();++mc)d[mc]=(int(b.map[mc*20+1]+.5f)&1)?std::uint8_t(8):std::uint8_t(0);
 auto at=[&](int i,int j)->int{return int(d[std::size_t(j)*std::size_t(MX)+std::size_t(i)]);};
 for(int j=0;j<MY;++j)for(int i=0;i<MX;++i){int v=at(i,j);if(!v)continue;
  if(i>0)v=std::min(v,at(i-1,j)+1);
  if(j>0){v=std::min(v,at(i,j-1)+1);if(i>0)v=std::min(v,at(i-1,j-1)+1);if(i<MX-1)v=std::min(v,at(i+1,j-1)+1);}
  d[std::size_t(j)*std::size_t(MX)+std::size_t(i)]=std::uint8_t(v);}
 for(int j=MY-1;j>=0;--j)for(int i=MX-1;i>=0;--i){int v=at(i,j);if(!v)continue;
  if(i<MX-1)v=std::min(v,at(i+1,j)+1);
  if(j<MY-1){v=std::min(v,at(i,j+1)+1);if(i<MX-1)v=std::min(v,at(i+1,j+1)+1);if(i>0)v=std::min(v,at(i-1,j+1)+1);}
  d[std::size_t(j)*std::size_t(MX)+std::size_t(i)]=std::uint8_t(v);}
 return d;}
inline bool regionWhole(const tear_atlas::Zone&z,const gfy_packed::TearBlock&b,const double*p,double half,const std::vector<std::uint8_t>*depth=nullptr){
 static const bool off=std::getenv("GFY_RW_OFF")!=nullptr;if(off||!z.mx||!z.my||!(z.h2>0))return true;
 const double d[3]={p[0]-z.C[0],p[1]-z.C[1],p[2]-z.C[2]};
 const double pa=d[0]*z.u[0]+d[1]*z.u[1]+d[2]*z.u[2],pb=d[0]*z.w[0]+d[1]*z.w[1]+d[2]*z.w[2];
 const double ea=half*(std::abs(z.u[0])+std::abs(z.u[1])+std::abs(z.u[2])),eb=half*(std::abs(z.w[0])+std::abs(z.w[1])+std::abs(z.w[2]));
 const int MX=int(z.mx),MY=int(z.my);
 if(depth&&depth->size()==std::size_t(MX)*std::size_t(MY)){
  const int i=std::min(std::max(int(std::floor((pa-z.a2)/z.h2)),0),MX-1),j=std::min(std::max(int(std::floor((pb-z.b2)/z.h2)),0),MY-1);
  const int dv=int((*depth)[std::size_t(j)*std::size_t(MX)+std::size_t(i)]);
  if(!dv)return false;if(dv>int(std::ceil(std::max(ea,eb)/z.h2)))return true;} // every map cell the footprint can reach is the region's
 const int i0=std::min(std::max(int(std::floor((pa-ea-z.a2)/z.h2)),0),MX-1),i1=std::min(std::max(int(std::floor((pa+ea-z.a2)/z.h2)),0),MX-1);
 const int j0=std::min(std::max(int(std::floor((pb-eb-z.b2)/z.h2)),0),MY-1),j1=std::min(std::max(int(std::floor((pb+eb-z.b2)/z.h2)),0),MY-1);
 for(int j=j0;j<=j1;++j)for(int i=i0;i<=i1;++i)if(!(int(b.map[(std::size_t(j)*z.mx+std::size_t(i))*20+1]+.5f)&1))return false;
 return true;}
struct LevelStats {std::size_t tears=0,looked=0,marked=0,far=0,band=0;double ms=0,level=0;};
template<class Mine>
inline void levelCut(const Grid&grid,const painter_detail::TissueField&tf,const gear_cells::GearCells*gear,const std::vector<std::uint8_t>*padF,const Owners&owns,std::size_t self,
    const std::vector<gfy_packed::TearRef>&tears,const std::vector<double>&th,const std::vector<char>&freshT,const CellBox&drawn,Mine&mine,LevelStats&st,bool smaller,const TileSet&ownTissue){
 if(!smaller)return; // the largest piece is not touched outside the tear's region: the rest of the body lies beside the cut there
 static const bool off=std::getenv("GFY_LEVEL_OFF")!=nullptr;if(off||!drawn.valid()||!tf.available||!owns.valid()||self>=maxPieces)return;const auto t0=core::Clock::now();
 constexpr double reachS=.08;const double p=grid.pitch;const double org[3]={grid.origin.x,grid.origin.y,grid.origin.z};
 const std::vector<std::uint8_t>*gearF=gear?&gear->tileFlag:nullptr;
 const std::uint8_t me=std::uint8_t(self);const std::uint32_t meBit=1u<<self;
 const Mine&cm=mine;std::vector<Cell> cut;
 for(std::size_t ti=0;ti<tears.size();++ti){const auto&tr=tears[ti];if(!tr.block||tr.side>1||tear_atlas::burstRecord(*tr.block)||!(ti<freshT.size()&&freshT[ti]))continue;
  const double thk=ti<th.size()?th[ti]:-1.;const auto z=tear_atlas::zone(*tr.block,thk>0?tear_atlas::depthScale(*tr.block,thk):1.);
  const auto&b=*tr.block;if(!z.ok||!(z.h2>0)||!z.mx||!z.my||b.map.size()<std::size_t(z.mx)*z.my*20||b.header.size()<104)continue;
  const int MX=int(z.mx),MY=int(z.my),sideI=tr.side?1:0;const double sg=tr.side?1.:-1.,f=z.scale,ripR=tear_atlas::f32(b.header,23);
  // each map cell's distance (map cells, 8 neighbours) to the limb's section: the region (flag 1) or the footprint (flag 4)
  const double lat=smaller?.06:.024;const int dmax=std::max(1,int(std::ceil(lat/z.h2)));
  std::vector<std::uint16_t> dist(std::size_t(MX)*std::size_t(MY),65535);std::vector<std::size_t> cur,nxt;
  auto flags=[&](std::size_t mc){return unsigned(b.map[mc*20+1]+.5f);};
  // a bridge tear of less than three quarters of the limb's section (flag 1 the bridge, flag 4 the limb's footprint): a limb the
  // owner's damage cut - the wounds made the ends, nothing is levelled (measured before this line: 105,988 cells on a KVMA leg shot off)
  {std::size_t n1=0,n4=0;for(std::size_t mc=0;mc<std::size_t(MX)*std::size_t(MY);++mc){const unsigned fl=flags(mc);n1+=fl&1u;n4+=(fl>>2)&1u;}if(n4&&double(n1)<.75*double(n4))continue;}
  for(std::size_t mc=0;mc<dist.size();++mc)if(flags(mc)&5u){dist[mc]=0;cur.push_back(mc);}
  if(cur.empty())continue;
  for(int s=1;s<=dmax&&!cur.empty();++s){nxt.clear();
   for(const auto mc:cur){const int i=int(mc%std::size_t(MX)),j=int(mc/std::size_t(MX));
    for(int dj=-1;dj<=1;++dj)for(int di=-1;di<=1;++di){const int I=i+di,J=j+dj;if(I<0||J<0||I>=MX||J>=MY)continue;const std::size_t q=std::size_t(J)*std::size_t(MX)+std::size_t(I);
     if(dist[q]!=65535)continue;dist[q]=std::uint16_t(s);nxt.push_back(q);}}
   cur.swap(nxt);}
  ++st.tears;
  const std::vector<std::uint8_t> rdep=regionDepth(z,b);
  const int rneed=1+int(std::ceil(.5*p*std::max(std::abs(z.u[0])+std::abs(z.u[1])+std::abs(z.u[2]),std::abs(z.w[0])+std::abs(z.w[1])+std::abs(z.w[2]))/z.h2));
  // the map cells the rule can cut in (within reach of the section, outside the region), spread by a tile's half diagonal: a tile
  // whose centre's map cell is not among them holds no cell to cut. The level: the map's deepest skin-class depth on this side.
  const int cr=int(std::ceil(.87*8*p/z.h2))+1;std::vector<std::uint8_t> cand(dist.size(),0),candX(dist.size(),0),candD(dist.size(),0);float eTop=0.f;
  for(std::size_t mc=0;mc<dist.size();++mc){cand[mc]=std::uint8_t(dist[mc]!=65535&&!(flags(mc)&1u));eTop=std::max(eTop,b.map[mc*20+2+std::size_t(sideI)]);}
  for(int j=0;j<MY;++j)for(int i=0;i<MX;++i){std::uint8_t v=0;for(int di=-cr;di<=cr&&!v;++di){const int I=i+di;if(I>=0&&I<MX)v=cand[std::size_t(j)*std::size_t(MX)+std::size_t(I)];}candX[std::size_t(j)*std::size_t(MX)+std::size_t(i)]=v;}
  for(int j=0;j<MY;++j)for(int i=0;i<MX;++i){std::uint8_t v=0;for(int dj=-cr;dj<=cr&&!v;++dj){const int J=j+dj;if(J>=0&&J<MY)v=candX[std::size_t(J)*std::size_t(MX)+std::size_t(i)];}candD[std::size_t(j)*std::size_t(MX)+std::size_t(i)]=v;}
  const double sFar=-.03,sDeep=(double(eTop)+ripR)*f+p; // the slab on this piece's side: 3 cm past the surface to the level
  const double rl2=z.Rl*z.Rl,latR=z.Rl+lat,a3=z.a2+MX*z.h2,b3=z.b2+MY*z.h2;
  const double reachN=reachS+tear_decide::offMax+p;int lo[3],hi[3];bool okb=true;
  // the box of the map cells within reach (past the map's border: + the reach), over the slab the rule looks at
  int i0=MX,i1=-1,j0=MY,j1=-1;for(std::size_t mc=0;mc<dist.size();++mc)if(dist[mc]!=65535){const int i=int(mc%std::size_t(MX)),j=int(mc/std::size_t(MX));i0=std::min(i0,i);i1=std::max(i1,i);j0=std::min(j0,j);j1=std::max(j1,j);}
  if(i1<i0)continue;
  {const double A0=z.a2+i0*z.h2-(i0==0?lat:0.),A1=z.a2+(i1+1)*z.h2+(i1==MX-1?lat:0.),B0=z.b2+j0*z.h2-(j0==0?lat:0.),B1=z.b2+(j1+1)*z.h2+(j1==MY-1?lat:0.);
   const double S0=sFar-tear_decide::offMax,S1=sDeep+tear_decide::offMax;double wlo[3]={1e30,1e30,1e30},whi[3]={-1e30,-1e30,-1e30};
   for(int q=0;q<8;++q){const double A=(q&1)?A1:A0,B=(q&2)?B1:B0,S=sg*((q&4)?S1:S0);for(int a=0;a<3;++a){const double v=z.C[a]+A*z.u[a]+B*z.w[a]+S*z.n[a];wlo[a]=std::min(wlo[a],v);whi[a]=std::max(whi[a],v);}}
   for(int a=0;a<3;++a){lo[a]=std::max(drawn.lo[a],int(std::floor((wlo[a]-org[a])/p)));hi[a]=std::min(drawn.hi[a],int(std::floor((whi[a]-org[a])/p))+1);okb=okb&&lo[a]<hi[a];}}
  if(!okb)continue;
  const int tx0=lo[0]>>3,tx1=(hi[0]-1)>>3,ty0=lo[1]>>3,ty1=(hi[1]-1)>>3,tz0=lo[2]>>3,tz1=(hi[2]-1)>>3;
  std::vector<std::vector<Cell>> part(core::parallelWidth()+1);std::atomic<std::size_t> looked{0},nFar{0},nBand{0};std::atomic<int> lvl{0};
  lowpoly_pieces::eachBlock(std::size_t(ty1-ty0+1),1,[&](std::size_t f0,std::size_t f1,std::size_t chunk){auto&out=part[std::min(chunk,part.size()-1)];std::size_t nl=0,nf=0,nb=0;
   for(std::size_t yi=f0;yi<f1;++yi){const int ty=ty0+int(yi);
    for(int tz=tz0;tz<=tz1;++tz)for(int tx=tx0;tx<=tx1;++tx){if(!owns.inside(tx,ty,tz))continue;const std::size_t oi=owns.idx(tx,ty,tz);
     if(!((gearF&&oi<gearF->size()&&(*gearF)[oi])||(padF&&oi<padF->size()&&(*padF)[oi])||ownTissue.in(tx,ty,tz)))continue; // no gear, padding or own tissue in the tile
     const bool roi=owns.L<0||owns.inRoi(tx,ty,tz);const std::int32_t slot=roi?owns.slotAt(oi):-1;
     if(slot<0&&(roi?owns.maskAt(oi):(1u<<owns.L))!=meBit)continue; // this piece's cells: a split tile's table, or the whole tile
     const std::array<std::uint8_t,512>*own=slot>=0?&owns.tiles[std::size_t(slot)].own:nullptr;
     {const double d[3]={org[0]+(tx*8+4)*p-z.C[0],org[1]+(ty*8+4)*p-z.C[1],org[2]+(tz*8+4)*p-z.C[2]};
      const double sc=(d[0]*z.n[0]+d[1]*z.n[1]+d[2]*z.n[2])*sg;if(sc<sFar-tear_decide::offMax-7*p||sc>sDeep+tear_decide::offMax+7*p)continue; // outside the slab
      const double ca=d[0]*z.u[0]+d[1]*z.u[1]+d[2]*z.u[2],cb=d[0]*z.w[0]+d[1]*z.w[1]+d[2]*z.w[2];
      const int CI=std::min(std::max(int(std::floor((ca-z.a2)/z.h2)),0),MX-1),CJ=std::min(std::max(int(std::floor((cb-z.b2)/z.h2)),0),MY-1);
      if(!candD[std::size_t(CJ)*std::size_t(MX)+std::size_t(CI)])continue;} // no map cell under the tile to cut in
     const Cell tc{tx,ty,tz};const auto gi=grid.tiles.find(tc);const core::Tile*gt=gi==grid.tiles.end()?nullptr:&gi->second;
     const auto mi=cm.find(tc);const core::Tile*mt=mi==cm.end()?nullptr:&mi->second;const core::Tile*ge=gear?gear->tile(tx,ty,tz):nullptr;
     for(int lx=0;lx<8;++lx){const int x=tx*8+lx;if(x<lo[0]||x>=hi[0])continue;const std::uint64_t rm=(gt?gt->removed[std::size_t(lx)]:0u)|(mt?mt->removed[std::size_t(lx)]:0u);
      if(rm==~std::uint64_t(0))continue;const std::uint64_t gw=ge?ge->removed[std::size_t(lx)]:0u;
      for(int ly=0;ly<8;++ly){const int y=ty*8+ly;if(y<lo[1]||y>=hi[1])continue;
       for(int lz=0;lz<8;++lz){const int zc=tz*8+lz;if(zc<lo[2]||zc>=hi[2])continue;const unsigned bit=unsigned(ly*8+lz);if((rm>>bit)&1u)continue;
        if(own&&(*own)[std::size_t((lx*8+ly)*8+lz)]!=me)continue;
        const double wx=org[0]+(x+.5)*p,wy=org[1]+(y+.5)*p,wz=org[2]+(zc+.5)*p;const double d[3]={wx-z.C[0],wy-z.C[1],wz-z.C[2]};
        const double s0=d[0]*z.n[0]+d[1]*z.n[1]+d[2]*z.n[2];if(std::abs(s0)>reachN)continue;
        const double pa=d[0]*z.u[0]+d[1]*z.u[1]+d[2]*z.u[2],pb=d[0]*z.w[0]+d[1]*z.w[1]+d[2]*z.w[2],r2=pa*pa+pb*pb;if(r2>latR*latR)continue;
        const int I=std::min(std::max(int(std::floor((pa-z.a2)/z.h2)),0),MX-1),J=std::min(std::max(int(std::floor((pb-z.b2)/z.h2)),0),MY-1);
        const std::size_t mc=std::size_t(J)*std::size_t(MX)+std::size_t(I);const unsigned fl=flags(mc);
        if((fl&1u)&&r2<=rl2){if(mc<rdep.size()&&int(rdep[mc])>=rneed)continue; // in the region with its whole footprint (the programs ask every pixel): the tear's
         const double pc[3]={wx,wy,wz};if(regionWhole(z,b,pc,.5*p))continue;}
        if(dist[mc]==65535)continue;
        {const double oa=pa<z.a2?z.a2-pa:(pa>a3?pa-a3:0.),ob=pb<z.b2?z.b2-pb:(pb>b3?pb-b3:0.);if(double(dist[mc])*z.h2+std::sqrt(oa*oa+ob*ob)>lat+z.h2)continue;} // (past the map: the border cell's distance + the way to it)
        const double so=(s0-double(b.map[mc*20]))*sg,E=double(b.map[mc*20+2+std::size_t(sideI)])*f; // so > 0: this piece's side of the surface
        if(so<sFar)continue;const bool farC=so<0;
        if(!farC&&!(std::max(0.,so-ripR*f)<E))continue;
        const bool gc=((gw>>bit)&1u)!=0;
        auto cls=[&](unsigned l){return (!l||l==16u)?0:(tear_decide::classOf(l&63u,(l&64u)!=0)==tear_decide::SHELL?1:2);}; // 0 air, 1 skin class, 2 flesh
        const unsigned l0=tf.sample(wx,wy,wz);const int c0=cls(l0);if(c0==2||(c0==0&&!gc))continue;
        if(!smaller&&c0==1&&!(l0&64u))continue; // the largest piece keeps its skin and fat outside the region (the body beside the cut): cloth and padding only
        ++nl;bool thin=c0==1,flesh=false; // (the cell's eight other anatomy samples: the programs read 1 mm labels)
        for(int q=0;q<8&&!flesh;++q){const int c=cls(tf.sample(wx+((q&1)?.25:-.25)*p,wy+((q&2)?.25:-.25)*p,wz+((q&4)?.25:-.25)*p));thin=thin||c==1;flesh=c==2;}
        if(flesh||(!thin&&!gc))continue;
        if(farC)++nf;else{++nb;lvl.store(int(E*1e6),std::memory_order_relaxed);}
        out.push_back(Cell{x,y,zc});}}}}}
   looked+=nl;nFar+=nf;nBand+=nb;});
  st.looked+=looked.load();st.far+=nFar.load();st.band+=nBand.load();if(lvl.load())st.level=lvl.load()*1e-6;for(const auto&v:part)cut.insert(cut.end(),v.begin(),v.end());}
 for(const Cell&c:cut){auto&tt=mine[Cell{c.x>>3,c.y>>3,c.z>>3}];tt.removed[std::size_t(c.x&7)]|=std::uint64_t(1)<<((c.y&7)*8+(c.z&7));++st.marked;}
 st.ms+=core::ms(t0);}
struct FloatPadArea {int lo[3]{0,0,0},n[3]{0,0,0};std::vector<Cell> tiles;std::vector<std::array<std::uint64_t,8>> pad;std::vector<std::int32_t> at;
 std::int32_t find(int tx,int ty,int tz)const{tx-=lo[0];ty-=lo[1];tz-=lo[2];if(tx<0||ty<0||tz<0||tx>=n[0]||ty>=n[1]||tz>=n[2])return -1;
  return at[(std::size_t(tz)*std::size_t(n[1])+std::size_t(ty))*std::size_t(n[0])+std::size_t(tx)];}
 bool inBox(int tx,int ty,int tz)const{tx-=lo[0];ty-=lo[1];tz-=lo[2];return tx>=0&&ty>=0&&tz>=0&&tx<n[0]&&ty<n[1]&&tz<n[2];}};
struct FloatPadStats {std::size_t nodes=0,groups=0,floating=0,cells=0,foreign=0,skin=0,film=0;double ms=0;};
inline FloatPadArea floatPadArea(const Grid&grid,const std::shared_ptr<const painter_detail::TissueField>&tfp,const std::vector<std::uint8_t>*padF,const std::vector<Region>&R,
    const std::vector<std::size_t>&freshIdx,const std::vector<CellBox>&own,std::size_t big){
 FloatPadArea A;static const bool off=std::getenv("GFY_FP_OFF")!=nullptr;
 const painter_detail::TissueField*tf=tfp&&tfp->available?tfp.get():nullptr;if(off||!tf||!padF||freshIdx.empty()||own.size()<2)return A;
 CellBox b;for(std::size_t k=0;k<own.size();++k)if(k!=big)b.add(own[k]);if(!b.valid())return A;
 const int tn[3]={(grid.size.x+7)/8,(grid.size.y+7)/8,(grid.size.z+7)/8};int hi[3];
 for(int a=0;a<3;++a){A.lo[a]=std::max(0,(b.lo[a]-25)>>3);hi[a]=std::min(tn[a]-1,(b.hi[a]+24)>>3);A.n[a]=hi[a]-A.lo[a]+1;if(A.n[a]<=0){A.n[0]=A.n[1]=A.n[2]=0;return A;}}
 if(std::size_t(A.n[0])*std::size_t(A.n[1])*std::size_t(A.n[2])>4000000){A.n[0]=A.n[1]=A.n[2]=0;return A;}
 A.at.assign(std::size_t(A.n[0])*std::size_t(A.n[1])*std::size_t(A.n[2]),-1);
 const double p=grid.pitch,reach=.06+.87*8*p;
 for(int tz=A.lo[2];tz<=hi[2];++tz)for(int ty=A.lo[1];ty<=hi[1];++ty)for(int tx=A.lo[0];tx<=hi[0];++tx){
  const std::size_t i=(std::size_t(tz)*std::size_t(tn[1])+std::size_t(ty))*std::size_t(tn[0])+std::size_t(tx);if(i>=padF->size()||!(*padF)[i])continue;
  const double c[3]={grid.origin.x+(tx*8+4)*p,grid.origin.y+(ty*8+4)*p,grid.origin.z+(tz*8+4)*p};bool nearT=false;
  for(const auto f:freshIdx){const Region&g=R[f];if(std::abs((c[0]-g.C[0])*g.n[0]+(c[1]-g.C[1])*g.n[1]+(c[2]-g.C[2])*g.n[2])<=reach){nearT=true;break;}}
  if(!nearT)continue;
  A.at[(std::size_t(tz-A.lo[2])*std::size_t(A.n[1])+std::size_t(ty-A.lo[1]))*std::size_t(A.n[0])+std::size_t(tx-A.lo[0])]=std::int32_t(A.tiles.size());A.tiles.push_back(Cell{tx,ty,tz});}
 A.pad.resize(A.tiles.size());if(A.tiles.empty())return A;
 int o3[3]={0,0,0};bool aligned=std::abs(double(tf->pitch)-p)<1e-6*p;
 for(int a=0;a<3;++a){const double o=(grid.origin[a]-double(tf->origin[std::size_t(a)]))/p;o3[a]=int(std::lround(o));aligned=aligned&&std::abs(o-o3[a])<1e-3;}
 lowpoly_pieces::eachBlock(A.tiles.size(),16,[&](std::size_t first,std::size_t last,std::size_t){for(std::size_t q=first;q<last;++q){const Cell tc=A.tiles[q];auto&w=A.pad[q];w.fill(0);
  for(int x=0;x<8;++x)for(int y=0;y<8;++y)for(int z=0;z<8;++z){bool pd=false;
   if(!aligned)pd=(tf->sample(grid.origin.x+(tc.x*8+x+.5)*p,grid.origin.y+(tc.y*8+y+.5)*p,grid.origin.z+(tc.z*8+z+.5)*p)&64u)!=0;
   else{const int X=tc.x*8+x+o3[0],Y=tc.y*8+y+o3[1],Z=tc.z*8+z+o3[2];
    if(!(X<0||Y<0||Z<0||unsigned(X)>=tf->size[0]||unsigned(Y)>=tf->size[1]||unsigned(Z)>=tf->size[2])){
     const auto v=tf->page[(std::size_t(Z>>3)*tf->pages[1]+unsigned(Y>>3))*tf->pages[0]+unsigned(X>>3)+painter_detail::TissueField::headerWords];
     pd=(v&painter_detail::TissueField::uniformBit)?(v&64u)!=0:(tf->at(X,Y,Z)&64u)!=0;}}
   if(pd)w[std::size_t(x)]|=std::uint64_t(1)<<(y*8+z);}}});
 return A;}
template<class Mine>
inline void floatPad(const FloatPadArea&A,const Grid&grid,const std::vector<Region>&R,const std::vector<std::size_t>&freshIdx,const std::vector<std::array<signed char,maxPieces>>&sig,
    std::size_t k,const CellBox&drawn,Mine&mine,FloatPadStats&st,const Owners&owns,bool smaller,const painter_detail::TissueField*tf){
 if(A.tiles.empty()||!drawn.valid()||k>=maxPieces)return;const auto t0=core::Clock::now();
 const double p=grid.pitch,hd=.87*8*p;
 auto inDrawn=[&](int tx,int ty,int tz){return tx*8>=drawn.lo[0]&&tx*8<drawn.hi[0]&&ty*8>=drawn.lo[1]&&ty*8<drawn.hi[1]&&tz*8>=drawn.lo[2]&&tz*8<drawn.hi[2];};
 auto count=[](std::uint64_t v){v=v-((v>>1)&0x5555555555555555ull);v=(v&0x3333333333333333ull)+((v>>2)&0x3333333333333333ull);
  return std::size_t((((v+(v>>4))&0x0f0f0f0f0f0f0f0full)*0x0101010101010101ull)>>56);};
 std::vector<tear_atlas::Zone> fpZ;if(smaller)for(const auto f:freshIdx)fpZ.push_back(tear_atlas::zone(*R[f].T)); // the fresh tears' zones (the programs' numbers)
 struct Node {std::int32_t tile;std::array<std::uint64_t,8> kept;std::uint32_t cells;bool anchor,sideA;const Region*g;double sgn,so;};
 std::vector<Node> nodes;std::vector<std::int32_t> nodeOf(A.tiles.size(),-1);const Mine&cm=mine;std::vector<char> slab(A.tiles.size(),0); // slab: the area's tiles in this piece's slab
 for(std::size_t q=0;q<A.tiles.size();++q){const Cell&tc=A.tiles[q];if(!inDrawn(tc.x,tc.y,tc.z))continue;
  const double c[3]={grid.origin.x+(tc.x*8+4)*p,grid.origin.y+(tc.y*8+4)*p,grid.origin.z+(tc.z*8+4)*p};bool in=false,deep=false;const Region*ng=nullptr;double nsg=1.,nso=0.;
  for(const auto f:freshIdx){const int sd=f<sig.size()?int(sig[f][k]):-1;if(sd!=0&&sd!=1)continue;const Region&g=R[f];
   const double so=((c[0]-g.C[0])*g.n[0]+(c[1]-g.C[1])*g.n[1]+(c[2]-g.C[2])*g.n[2])*(sd?1.:-1.);if(so<-.03-hd||so>.06+hd)continue;in=true;deep=so>=.06-hd;ng=&g;nsg=sd?1.:-1.;nso=so;break;}
  if(!in)continue;slab[q]=1;
  Node nd;nd.tile=std::int32_t(q);nd.kept=A.pad[q];nd.cells=0;nd.anchor=deep;nd.sideA=false;nd.g=ng;nd.sgn=nsg;nd.so=nso;
  const auto gi=grid.tiles.find(tc);if(gi!=grid.tiles.end())for(unsigned x=0;x<8;++x)nd.kept[x]&=~gi->second.removed[x];
  {const auto mi=cm.find(tc);if(mi!=cm.end())for(unsigned x=0;x<8;++x)nd.kept[x]&=~mi->second.removed[x];}
  // a smaller piece: padding the owners' table does not give it (another piece's cell its marks missed, or nobody's - measured: the
  // rest of the crotch pocket belonged to no piece, so the body AND the leg drew it) is not this piece's to draw: marked
  if(smaller&&owns.valid()){std::array<std::uint64_t,8> fo;fo.fill(0);bool any=false;
   for(unsigned x=0;x<8;++x)for(std::uint64_t w=nd.kept[x];w;w&=w-1){unsigned long bq=0;
#ifdef _MSC_VER
    _BitScanForward64(&bq,w);
#else
    bq=unsigned(__builtin_ctzll(w));
#endif
    const int cx=tc.x*8+int(x),cy=tc.y*8+int(bq>>3),cz=tc.z*8+int(bq&7u);if(owns.owner(cx,cy,cz)==std::uint8_t(k))continue;
    const double wx=grid.origin.x+(cx+.5)*p,wy=grid.origin.y+(cy+.5)*p,wz=grid.origin.z+(cz+.5)*p;bool inReg=false; // (inside a tear's region the tear decides)
    {const double pq[3]={wx,wy,wz};for(std::size_t zi=0;zi<fpZ.size()&&!inReg;++zi){double s0q=0;inReg=tear_atlas::inRegion(fpZ[zi],*R[freshIdx[zi]].T,pq,s0q)&&regionWhole(fpZ[zi],*R[freshIdx[zi]].T,pq,.5*p);}} // (the programs' own region test: the map's flag)
    if(!inReg){fo[x]|=std::uint64_t(1)<<bq;any=true;}}
   if(any){auto&tt=mine[tc];for(unsigned x=0;x<8;++x){tt.removed[x]|=fo[x];nd.kept[x]&=~fo[x];st.foreign+=count(fo[x]);}}}
  for(unsigned x=0;x<8;++x)nd.cells+=std::uint32_t(count(nd.kept[x]));
  if(!nd.cells)continue;
  // the area's side where the piece draws on: the padding may carry on there
  if(!nd.anchor){static const int f6[6][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
   for(const auto&d:f6){const int x=tc.x+d[0],y=tc.y+d[1],z=tc.z+d[2];if(!A.inBox(x,y,z)&&inDrawn(x,y,z)){nd.anchor=true;nd.sideA=true;break;}}}
  nodeOf[q]=std::int32_t(nodes.size());nodes.push_back(nd);}
 st.nodes+=nodes.size();
 std::vector<char> seen(nodes.size(),0);std::vector<std::int32_t> stack,group;
 std::vector<std::uint64_t> dropped; // the removed padding cells (tile index in A << 9 | cell), for the skin pass below
 std::vector<std::array<std::uint64_t,8>> vis(nodes.size());for(auto&v:vis)v.fill(0);std::vector<std::uint32_t> cstack,comp; // cell level: node << 9 | cell
 for(std::size_t s0=0;s0<nodes.size();++s0){if(seen[s0])continue;seen[s0]=1;stack.assign(1,std::int32_t(s0));group.clear();bool anchor=false;std::size_t cells=0;
  while(!stack.empty()){const std::int32_t s=stack.back();stack.pop_back();group.push_back(s);const Node&nd=nodes[std::size_t(s)];anchor=anchor||nd.anchor;cells+=nd.cells;
   const Cell&tc=A.tiles[std::size_t(nd.tile)];
   for(int dz=-1;dz<=1;++dz)for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){if(!dx&&!dy&&!dz)continue;const std::int32_t q=A.find(tc.x+dx,tc.y+dy,tc.z+dz);if(q<0)continue;
    const std::int32_t s2=nodeOf[std::size_t(q)];if(s2<0||seen[std::size_t(s2)])continue;seen[std::size_t(s2)]=1;stack.push_back(s2);}}
  ++st.groups;
  if(!anchor&&cells<=4000){++st.floating;st.cells+=cells; // no tile of the group carries on: all of it
   for(const auto s:group){const Node&nd=nodes[std::size_t(s)];auto&tt=mine[A.tiles[std::size_t(nd.tile)]];for(unsigned x=0;x<8;++x)tt.removed[x]|=nd.kept[x];
    for(int c=0;c<512;++c)if((nd.kept[std::size_t(c>>6)]>>(c&63))&1u)dropped.push_back(std::uint64_t(nd.tile)<<9|std::uint64_t(c));}continue;}
  if(cells>8000)continue; // a large body of padding: not searched cell by cell
  // cell by cell (26 neighbours) inside the group: a part that reaches neither the slab's deep end (6 cm) nor the area's side
  for(const auto s:group)for(int c=0;c<512;++c){const std::uint64_t b0=std::uint64_t(1)<<(c&63);
   if(!(nodes[std::size_t(s)].kept[std::size_t(c>>6)]&b0)||(vis[std::size_t(s)][std::size_t(c>>6)]&b0))continue;
   vis[std::size_t(s)][std::size_t(c>>6)]|=b0;comp.clear();cstack.assign(1,std::uint32_t(s)<<9|std::uint32_t(c));bool anc=false;
   while(!cstack.empty()){const std::uint32_t e=cstack.back();cstack.pop_back();comp.push_back(e);const std::int32_t s1=std::int32_t(e>>9);const int c1=int(e&511u),x=c1>>6,y=(c1>>3)&7,z=c1&7;
    const Node&n1=nodes[std::size_t(s1)];const Cell t1=A.tiles[std::size_t(n1.tile)];
    if(!anc){if(n1.sideA)anc=true;else if(n1.so>=.06-2*hd){const Region&g=*n1.g;const double wx=grid.origin.x+(t1.x*8+x+.5)*p,wy=grid.origin.y+(t1.y*8+y+.5)*p,wz=grid.origin.z+(t1.z*8+z+.5)*p;
      anc=((wx-g.C[0])*g.n[0]+(wy-g.C[1])*g.n[1]+(wz-g.C[2])*g.n[2])*n1.sgn>=.06-2*p;}}
    for(int dz=-1;dz<=1;++dz)for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){if(!dx&&!dy&&!dz)continue;int X=x+dx,Y=y+dy,Z=z+dz;std::int32_t s2=s1;
     if(X<0||X>7||Y<0||Y>7||Z<0||Z>7){const std::int32_t q=A.find(t1.x+(X<0?-1:(X>7?1:0)),t1.y+(Y<0?-1:(Y>7?1:0)),t1.z+(Z<0?-1:(Z>7?1:0)));if(q<0)continue;
      s2=nodeOf[std::size_t(q)];if(s2<0)continue;X&=7;Y&=7;Z&=7;}
     const std::uint64_t bm=std::uint64_t(1)<<(Y*8+Z);if(!(nodes[std::size_t(s2)].kept[std::size_t(X)]&bm)||(vis[std::size_t(s2)][std::size_t(X)]&bm))continue;
     vis[std::size_t(s2)][std::size_t(X)]|=bm;cstack.push_back(std::uint32_t(s2)<<9|std::uint32_t((X*8+Y)*8+Z));}}
   if(anc||comp.size()>4000)continue;
   ++st.floating;st.cells+=comp.size();
   for(const auto e:comp){const Node&n1=nodes[std::size_t(e>>9)];const int c1=int(e&511u);auto&tt=mine[A.tiles[std::size_t(n1.tile)]];tt.removed[std::size_t(c1>>6)]|=std::uint64_t(1)<<(c1&63);
    dropped.push_back(std::uint64_t(n1.tile)<<9|std::uint64_t(c1));}}}
 // the skin pass: the anatomy is read at 1 mm and a 2 mm cell counted as padding by its centre alone, so round a removed pocket
 // a 1 mm skin of the same padding stayed in the neighbouring cells (centre: garment air) - still drawn, as a sheet. Each
 // 26-neighbour cell of a removed cell that holds padding at any of its eight 1 mm samples and no tissue is marked too.
 if(tf&&tf->available&&!dropped.empty()){std::unordered_set<std::uint64_t> done;done.reserve(dropped.size()*4);std::vector<Cell> more;
  for(const auto e:dropped){const Cell&tc=A.tiles[std::size_t(e>>9)];const int c=int(e&511u),gx=tc.x*8+(c>>6),gy=tc.y*8+((c>>3)&7),gz=tc.z*8+(c&7);
   for(int dz=-1;dz<=1;++dz)for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){if(!dx&&!dy&&!dz)continue;const int x=gx+dx,y=gy+dy,z=gz+dz;if(x<0||y<0||z<0)continue;
    const std::uint64_t key=(std::uint64_t(std::uint32_t(x))<<42)|(std::uint64_t(std::uint32_t(y))<<21)|std::uint64_t(std::uint32_t(z));if(!done.insert(key).second)continue;
    const Cell nt{x>>3,y>>3,z>>3};const unsigned bx=unsigned(x&7),bb=unsigned((y&7)*8+(z&7));
    {const auto gi=grid.tiles.find(nt);if(gi!=grid.tiles.end()&&((gi->second.removed[bx]>>bb)&1u))continue;}
    {const auto mi=cm.find(nt);if(mi!=cm.end()&&((mi->second.removed[bx]>>bb)&1u))continue;}
    const double wx=grid.origin.x+(x+.5)*p,wy=grid.origin.y+(y+.5)*p,wz=grid.origin.z+(z+.5)*p;bool pad=false,tissue=false;
    for(int q=0;q<8&&!tissue;++q){const unsigned l=tf->sample(wx+((q&1)?.25:-.25)*p,wy+((q&2)?.25:-.25)*p,wz+((q&4)?.25:-.25)*p);
     if(l&64u)pad=true;else if(l!=0u&&l!=16u)tissue=true;}
    if(pad&&!tissue)more.push_back(Cell{x,y,z});}}
  for(const Cell&c:more){auto&tt=mine[Cell{c.x>>3,c.y>>3,c.z>>3}];tt.removed[std::size_t(c.x&7)]|=std::uint64_t(1)<<((c.y&7)*8+(c.z&7));}
  st.skin+=more.size();}
 // the film pass (measured on KVMA: the floating fat bits): a cell is hidden by its centre, the anatomy is read at 1 mm - beside a
 // hidden fat / skin / padding cell, a cell whose centre is air still holds a 1 mm film of that layer, is no rule's cell and was
 // drawn alone. In the slab's tiles: an unmarked cell with air at its centre, a skin-class sample among its eight 1 mm samples and no
 // flesh, and a face neighbour that a rule hides (marked, not a wound's removed cell) whose centre is skin class, is marked too.
 if(tf&&tf->available){std::vector<std::size_t> list;for(std::size_t q=0;q<A.tiles.size();++q)if(slab[q])list.push_back(q);
  std::vector<std::vector<Cell>> part(core::parallelWidth()+1);
  auto marksOf=[&](const Cell&c)->const core::Tile*{const auto it=cm.find(c);return it==cm.end()?nullptr:&it->second;};
  auto cls=[](unsigned l){return (!l||l==16u)?0:(tear_decide::classOf(l&63u,(l&64u)!=0)==tear_decide::SHELL?1:2);}; // 0 air, 1 skin class, 2 flesh
  lowpoly_pieces::eachBlock(list.size(),8,[&](std::size_t first,std::size_t last,std::size_t th){auto&out=part[std::min(th,part.size()-1)];
   for(std::size_t li=first;li<last;++li){const Cell tc=A.tiles[list[li]];const core::Tile*mt=marksOf(tc);const auto gi=grid.tiles.find(tc);const core::Tile*gt=gi==grid.tiles.end()?nullptr:&gi->second;
    const core::Tile*nb[6]={marksOf(Cell{tc.x-1,tc.y,tc.z}),marksOf(Cell{tc.x+1,tc.y,tc.z}),marksOf(Cell{tc.x,tc.y-1,tc.z}),marksOf(Cell{tc.x,tc.y+1,tc.z}),marksOf(Cell{tc.x,tc.y,tc.z-1}),marksOf(Cell{tc.x,tc.y,tc.z+1})};
    if(!mt&&!nb[0]&&!nb[1]&&!nb[2]&&!nb[3]&&!nb[4]&&!nb[5])continue;
    std::uint64_t M[8],G[8],H[8];for(int x=0;x<8;++x){M[x]=mt?mt->removed[std::size_t(x)]:0;G[x]=gt?gt->removed[std::size_t(x)]:0;H[x]=M[x]&~G[x];}
    // hidden by a rule at local (x, y, z); one coordinate may lie one cell outside the tile
    auto hid=[&](int x,int y,int z)->bool{const core::Tile*t=mt;bool own=true;
     if(x<0){t=nb[0];x=7;own=false;}else if(x>7){t=nb[1];x=0;own=false;}else if(y<0){t=nb[2];y=7;own=false;}else if(y>7){t=nb[3];y=0;own=false;}
     else if(z<0){t=nb[4];z=7;own=false;}else if(z>7){t=nb[5];z=0;own=false;}
     if(!t||!((t->removed[std::size_t(x)]>>(y*8+z))&1u))return false;return !(own&&((G[x]>>(y*8+z))&1u));};
    for(int x=0;x<8;++x){const std::uint64_t h=H[x];
     std::uint64_t D=((h<<1)&0xfefefefefefefefeull)|((h>>1)&0x7f7f7f7f7f7f7f7full)|(h<<8)|(h>>8); // a hidden neighbour along z, y
     D|=x>0?H[x-1]:(nb[0]?nb[0]->removed[7]:0);D|=x<7?H[x+1]:(nb[1]?nb[1]->removed[0]:0);
     if(nb[2])D|=(nb[2]->removed[std::size_t(x)]>>56)&0xffull;if(nb[3])D|=(nb[3]->removed[std::size_t(x)]&0xffull)<<56;
     if(nb[4])D|=(nb[4]->removed[std::size_t(x)]>>7)&0x0101010101010101ull;if(nb[5])D|=(nb[5]->removed[std::size_t(x)]&0x0101010101010101ull)<<7;
     for(std::uint64_t cand=D&~M[x]&~G[x];cand;cand&=cand-1){unsigned long bq=0;
#ifdef _MSC_VER
      _BitScanForward64(&bq,cand);
#else
      bq=unsigned(__builtin_ctzll(cand));
#endif
      const int y=int(bq>>3),z=int(bq&7u);const double wx=grid.origin.x+(tc.x*8+x+.5)*p,wy=grid.origin.y+(tc.y*8+y+.5)*p,wz=grid.origin.z+(tc.z*8+z+.5)*p;
      if(cls(tf->sample(wx,wy,wz))!=0)continue; // air at its centre: no rule's cell
      bool thin=false,flesh=false;for(int q=0;q<8&&!flesh;++q){const int c=cls(tf->sample(wx+((q&1)?.25:-.25)*p,wy+((q&2)?.25:-.25)*p,wz+((q&4)?.25:-.25)*p));thin=thin||c==1;flesh=c==2;}
      if(!thin||flesh)continue;
      static const int f6[6][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};bool layer=false;
      for(int d=0;d<6&&!layer;++d)if(hid(x+f6[d][0],y+f6[d][1],z+f6[d][2]))layer=cls(tf->sample(wx+f6[d][0]*p,wy+f6[d][1]*p,wz+f6[d][2]*p))==1;
      if(layer)out.push_back(Cell{tc.x*8+x,tc.y*8+y,tc.z*8+z});}}}});
  for(const auto&v:part){for(const Cell&c:v){auto&tt=mine[Cell{c.x>>3,c.y>>3,c.z>>3}];tt.removed[std::size_t(c.x&7)]|=std::uint64_t(1)<<((c.y&7)*8+(c.z&7));}st.film+=v.size();}}
 st.ms+=core::ms(t0);}
template<class Anchorable,class RegionFrontier>
inline std::vector<bound_surface::Piece> build(const gfy_packed::Mesh&published,const Grid&grid,const gfy_connectivity::Topology&volume,
    const std::vector<std::uint32_t>&components,const std::set<std::uint16_t>&wholeChildren,std::uint16_t originalChildren,
    Anchorable&&anchorable,RegionFrontier&&regionFrontier,Stats&stats,
    const std::vector<std::pair<Cell,core::Tile>>*inherited,std::vector<std::vector<std::pair<Cell,core::Tile>>>*marksOut,
    const gear_cells::GearCells*gear=nullptr){
 if(head_burst_pieces::only(published))return head_burst_pieces::build(published,grid,volume,components,wholeChildren,originalChildren,anchorable,regionFrontier,stats,inherited,marksOut,gear); // headfix (gear: cutfix's gear cells, owned per cell by the record)
 const auto t0=core::Clock::now();const std::size_t P=components.size(),K=published.tears.size();
 need(P>=1&&P<=maxPieces&&stats.sig.size()==K,"mask_piece3_input");
 need(published.mask_volume&&published.removal_field&&published.removal_field->available&&!published.children.empty(),"mask_piece3_published_mesh");
 stats.used=true;const PieceAt pieceAt(grid,volume,components);
 // Assign a relief brick only when the source topology in its one-brick
 // neighbourhood belongs to exactly one published piece. Ambiguous bricks
 // at a split retain the released torn surface of each piece.
 std::unordered_map<Cell,int,core::CellHash> floorOwner;
 if(grid.visibleFloor)for(const auto& relief:*grid.visibleFloor){
  const Cell k=relief.first;int owner=-1;bool ambiguous=false;
  const int z0=std::max(0,8*k.z-8),z1=std::min(grid.size.z,8*k.z+16);
  for(int x=std::max(0,8*k.x-8);x<std::min(grid.size.x,8*k.x+16)&&!ambiguous;++x)
   for(int y=std::max(0,8*k.y-8);y<std::min(grid.size.y,8*k.y+16)&&!ambiguous;++y){
    const auto ci=pieceAt.columnOf[std::size_t(y)*grid.size.x+x];if(ci<0)continue;
    const auto& c=volume.columns[std::size_t(ci)];
    for(auto it=c.first;it<c.first+c.count;++it){
     const auto& span=volume.spans[it];if(span.hi<=z0||span.lo>=z1)continue;
     int part=span.component<pieceAt.pieceOf.size()?pieceAt.pieceOf[span.component]:-1;
     if(part<0||(owner>=0&&part!=owner)){ambiguous=true;break;}owner=part;
    }
   }
  if(!ambiguous&&owner>=0)floorOwner.emplace(k,owner);
 }
 std::vector<Region> R;R.reserve(K);for(const auto&t:published.tears)R.emplace_back(*t.block);
 std::vector<std::size_t> freshIdx;for(std::size_t t=0;t<K;++t)if(fresh(published.tears[t]))freshIdx.push_back(t);
 // the pieces a sample on side s of fresh tear t may belong to
 std::vector<std::array<std::uint32_t,2>> sideMask(K,{0u,0u});
 for(const auto t:freshIdx)for(std::size_t k=0;k<P;++k){const int v=stats.sig[t][k];if(v==0||v==2)sideMask[t][0]|=1u<<k;if(v==1||v==2)sideMask[t][1]|=1u<<k;}
 std::vector<std::array<char,64>> ok(P);std::vector<std::array<std::uint16_t,64>> front(P);
 for(std::size_t k=0;k<P;++k)for(unsigned b=0;b<64;++b){ok[k][b]=anchorable(k,std::uint16_t(b))?1:0;front[k][b]=ok[k][b]?std::uint16_t(65535):regionFrontier(k,std::uint16_t(b));}
 auto anchored=[&](std::size_t k,const gfy_packed::Vertex&v){
  for(unsigned w=0;w<4;++w)if(gfy_signed_hw::dependency(v.weight,w,negligible_skin_weight)&&(v.bone[w]>=64||!ok[k][v.bone[w]]))return false;return true;};
 auto frozenPosable=[&](std::size_t k,const gfy_packed::Vertex&v){
  for(unsigned w=0;w<4;++w)if(v.weight[w]<0.f&&!(w==3&&gfy_signed_hw::valid4(v.weight,true)&&
     gfy_signed_hw::quantize(v.weight[0])+gfy_signed_hw::quantize(v.weight[1])+gfy_signed_hw::quantize(v.weight[2])<=256u))return false;
  bool own=false;
  for(unsigned w=0;w<4;++w){if(!gfy_signed_hw::dependency(v.weight,w,negligible_skin_weight))continue;const auto b=v.bone[w];
   if(b>=64)return false;if(ok[k][b]){own=own||v.weight[w]>0.f;continue;}if(front[k][b]==65535)return false;}
  return own;};
 auto position=[](const gfy_packed::Vertex&v){return Vec{v.surface[0],v.surface[1],v.surface[2]};};
 // cutfix: every gear / padding cell's owner (owners above): the exterior is drawn by the piece owning its cells
 auto tileOf=[&](const Vec&q,int&tx,int&ty,int&tz){tx=int(std::floor((q.x-grid.origin.x)/grid.pitch))>>3;ty=int(std::floor((q.y-grid.origin.y)/grid.pitch))>>3;tz=int(std::floor((q.z-grid.origin.z)/grid.pitch))>>3;};
 const SpanPass spans=spanPass(grid,volume,components);const auto&own=spans.own; // each piece's box and tile set, every component's box
 const Owners owns=owners(grid,volume,components,published.tissue_field,gear,R,freshIdx,sideMask,spans.tiles);
 stats.padTiles+=owns.tiles.size();stats.padCells+=owns.padCells;stats.gearCells+=owns.gearCells;stats.ownedCells+=owns.owned;stats.seededCells+=owns.seeded;stats.padMs+=owns.ms;
 stats.gfBars+=owns.gfBars;stats.gfExtTiles+=owns.gfExtTiles;stats.gfLeftTiles+=owns.gfLeftTiles;stats.gfLeftCells+=owns.gfLeftCells; // gearfine
 auto sampleOwner=[&](const Vec&q)->std::uint8_t{const int x=int(std::floor((q.x-grid.origin.x)/grid.pitch)),y=int(std::floor((q.y-grid.origin.y)/grid.pitch)),
  z=int(std::floor((q.z-grid.origin.z)/grid.pitch));return owns.owner(x,y,z);};
 // the pieces (of `among`) whose bones carry most of the triangle's skin weight
 auto skinOwners=[&](const gfy_packed::Vertex*const*v,std::uint32_t among)->std::uint32_t{double best=0;std::uint32_t set=0;
  for(std::size_t k=0;k<P;++k){if(!((among>>k)&1u))continue;double s=0;for(int j=0;j<3;++j)for(unsigned w=0;w<4;++w){const auto b=v[j]->bone[w];if(b<64&&ok[k][b]&&v[j]->weight[w]>0.f)s+=v[j]->weight[w];}
   if(s>best+1e-6){best=s;set=1u<<k;}else if(s>0&&std::abs(s-best)<=1e-6)set|=1u<<k;}return set;};
 // a tie: the piece (of `set`) whose volume lies under the triangle's centre, stepped inward along its normal (18 mm); else (cutfix:
 // a tie never goes to every tied piece - gear standing off the anatomy, a hood, a mask - it was drawn twice) the tied piece whose
 // tissue is nearest along the normal (both ways) and the six axes, up to 96 mm; else the tied piece whose cells are near the
 // centre's tile (64 mm); else the first. `deep` = found beyond 18 mm (not a border).
 auto underneath=[&](const gfy_packed::Vertex*const*v,std::uint32_t set,bool&found,bool&deep)->int{found=false;deep=false;const Vec c=(position(*v[0])+position(*v[1])+position(*v[2]))*(1./3);
  Vec nn{0,0,0};for(int j=0;j<3;++j)nn=nn+Vec{v[j]->surface[3],v[j]->surface[4],v[j]->surface[5]};const double ln=length(nn);
  if(ln>1e-9){nn=nn*(1./ln);for(int s=0;s<=8;++s){const Vec q=c-nn*(grid.pitch*(.5+s));const int k=pieceAt(q.x,q.y,q.z);if(k>=0&&((set>>k)&1u)){found=true;return k;}}}
  found=true;deep=true;
  {const Vec dirs[8]={ln>1e-9?nn*-1.:Vec{0,-1,0},ln>1e-9?nn:Vec{0,1,0},Vec{1,0,0},Vec{-1,0,0},Vec{0,1,0},Vec{0,-1,0},Vec{0,0,1},Vec{0,0,-1}};
   for(int s=0;s<48;++s)for(const auto&dv:dirs){const Vec q=c+dv*(grid.pitch*(.5+s));const int k=pieceAt(q.x,q.y,q.z);if(k>=0&&((set>>k)&1u))return k;}}
  {const auto o=sampleOwner(c);if(o<P&&((set>>o)&1u))return int(o);}
  for(std::size_t k=0;k<P;++k)if((set>>k)&1u)return int(k);found=false;deep=false;return -1;};
 auto isBox=[](const gfy_packed::Child&c){if(c.vertices.empty())return false;for(const auto&v:c.vertices)if(!(v.surface[12]<=-999.f))return false;return true;};
 const auto s0=core::Clock::now();const auto&children=published.children;
 struct Sel {std::vector<std::vector<std::uint32_t>> tris,alt,altFar;std::vector<std::map<std::uint16_t,std::uint16_t>> anchors,altAnchors;std::vector<std::vector<char>> touch;
  std::size_t dropped{},unposable{},frozen{},inRegion{},bySkeleton{},byVolume{},ties{},ghosts{};};
 std::vector<Sel> sel(children.size());const std::uint32_t all=P>=32?~0u:((1u<<P)-1u);
 // piecesregional (2026-11-16): the selection's cost follows the blowout, not the model. The same per-triangle rule, decisions and
 // counters, in the same triangle order:
 //  - every child in chunks of 2048 triangles, the chunks in parallel; each chunk's lists, anchors, touches and counters are
 //    appended / merged into its child's in chunk order (a decision reads nothing another triangle wrote);
 //  - a triangle none of whose 7 samples lies in any tear's region: every sample's region test answers -1, so each sample takes the
 //    skin rule (bySkin(all), as before, 7 times) and touches nothing - its samples' loop runs without the region work. A tear is
 //    tested at the samples only where a sample can lie in its region (a ball around the region, 2 % slack on an orthonormal frame:
 //    a 64-triangle block's box first, then the triangle's own ball); any sample in a region: the per-sample loop, unchanged.
 //  (GFY_PR_SLOW_SELECT: every triangle by the per-sample loop, one child a job - probe comparisons.)
 static const bool prSlowSelect=std::getenv("GFY_PR_SLOW_SELECT")!=nullptr;
 struct PrBall {double c[3]={0,0,0},reach=0;bool use=false;};std::vector<PrBall> prBall(K);
 for(std::size_t i=0;i<K;++i){const Region&g=R[i];auto&b=prBall[i];const double*f[3]={g.n,g.u,g.w};bool ortho=true;
  for(int x=0;x<3;++x)for(int y=0;y<3;++y){const double dt=f[x][0]*f[y][0]+f[x][1]*f[y][1]+f[x][2]*f[y][2];ortho=ortho&&std::abs(dt-(x==y?1.:0.))<1e-3;}
  const double hh=g.H+tear_decide::offMax;b.reach=std::sqrt((hh*hh+g.Rl*g.Rl)/.98)+1e-6;for(int a=0;a<3;++a)b.c[a]=g.C[a];b.use=ortho&&std::isfinite(b.reach);}
 // a point farther than `reach` from the region's centre is outside it: |s|^2+pa^2+pb^2 >= .98 |d|^2 on a frame within 1e-3 of orthonormal
 struct PrChunk {std::size_t c,t0,t1;};std::vector<PrChunk> prChunks;const std::size_t prChunkTris=prSlowSelect?std::size_t(1)<<40:2048;
 for(std::size_t c=0;c<children.size();++c){auto&s=sel[c];s.tris.resize(P);s.anchors.resize(P);s.alt.resize(P);s.altFar.resize(P);s.altAnchors.resize(P);
  s.touch.assign(P,std::vector<char>(K,0));
  const auto&src=children[c];if(src.material||src.indices.empty()||wholeChildren.count(src.material_source_child)||isBox(src))continue;
  const std::size_t T=src.indices.size()/3;for(std::size_t a=0;a<T;a+=prChunkTris)prChunks.push_back({c,a,std::min(T,a+prChunkTris)});}
 std::vector<Sel> prPart(prChunks.size());
 lowpoly_pieces::eachBlock(prChunks.size(),1,[&](std::size_t first,std::size_t last,std::size_t){for(std::size_t ci=first;ci<last;++ci){auto&s=prPart[ci];s.tris.resize(P);s.anchors.resize(P);s.alt.resize(P);s.altFar.resize(P);s.altAnchors.resize(P);
  s.touch.assign(P,std::vector<char>(K,0));
  const auto&src=children[prChunks[ci].c];const std::size_t prEnd=prChunks[ci].t1;std::vector<int> inR(K);
  std::size_t prBlockEnd=prChunks[ci].t0;std::vector<char> prBlockTear(K,1);
  for(std::size_t t=prChunks[ci].t0;t<prEnd;++t){const gfy_packed::Vertex*v[3]={&src.vertices[src.indices[3*t]],&src.vertices[src.indices[3*t+1]],&src.vertices[src.indices[3*t+2]]};
   const Vec a=position(*v[0]),b=position(*v[1]),d=position(*v[2]);const Vec samples[7]={a,b,d,(a+b+d)*(1./3),(a+b)*.5,(b+d)*.5,(a+d)*.5};
   bool prFast=false;
   if(!prSlowSelect){
    if(t>=prBlockEnd){prBlockEnd=std::min(prEnd,t+64);double lo[3]={1e300,1e300,1e300},hi[3]={-1e300,-1e300,-1e300};bool fin=true;
     for(std::size_t u=t;u<prBlockEnd;++u)for(int j=0;j<3;++j){const auto&row=src.vertices[src.indices[3*u+j]];for(int ax=0;ax<3;++ax){const double q=row.surface[ax];fin=fin&&std::isfinite(q);lo[ax]=std::min(lo[ax],q);hi[ax]=std::max(hi[ax],q);}}
     for(std::size_t i=0;i<K;++i){double s2=0;for(int ax=0;ax<3;++ax){const double c=prBall[i].c[ax],e=c<lo[ax]?lo[ax]-c:c>hi[ax]?c-hi[ax]:0.;s2+=e*e;}
      prBlockTear[i]=char(!fin||!prBall[i].use||!(std::sqrt(s2)>prBall[i].reach));}}
    prFast=true;Vec cc{0,0,0};double rr=-1;
    for(std::size_t i=0;i<K&&prFast;++i){if(!prBlockTear[i])continue;const auto&g=prBall[i];
     if(g.use){if(rr<0){cc=(a+b+d)*(1./3);rr=std::max({length(a-cc),length(b-cc),length(d-cc)});}const double dx=cc.x-g.c[0],dy=cc.y-g.c[1],dz=cc.z-g.c[2];
      if(std::sqrt(dx*dx+dy*dy+dz*dz)>g.reach+rr)continue;}
     for(const auto&q:samples)if(R[i].side(q.x,q.y,q.z)>=0){prFast=false;break;}}}
   std::uint32_t want=0,skin=0,tieAll=0;bool skinDone=false,region=false;int how=0;
   std::uint32_t owned=0;for(const auto&q:samples){const auto o=sampleOwner(q);if(o<P)owned|=1u<<o;} // cutfix: the cells' owners first
   auto bySkin=[&](std::uint32_t among)->std::uint32_t{std::uint32_t o;if(among==all){if(!skinDone){skin=skinOwners(v,all);skinDone=true;}o=skin;}else o=skinOwners(v,among);
    if(!o)o=among==all?0u:among;if(o&&(o&(o-1))&&owned){++s.ties;tieAll|=o;how=8;return o;} // the cells decide: no tie search
    if(o&&(o&(o-1))){++s.ties;tieAll|=o;bool f=false,deep=false;const int k=underneath(v,o,f,deep);if(f)++s.byVolume;how=f&&!deep?4:8;return f?1u<<k:o;}how=2;return o;};
   std::uint32_t wantIn=0,wantFree=0;std::vector<char> sidesSeen(freshIdx.size(),0); // cutfix
   if(prFast)for(int q7=0;q7<7;++q7){const std::uint32_t got=bySkin(all);if(got)++s.bySkeleton;wantFree|=got;want|=got;} // piecesregional: no sample in a region
   else for(const auto&q:samples){std::uint32_t cand=all;bool constrained=false;
    for(std::size_t i=0;i<K;++i){inR[i]=R[i].side(q.x,q.y,q.z);}
    for(std::size_t f=0;f<freshIdx.size();++f){const auto t=freshIdx[f];const int sd=inR[t];if(sd<0)continue;constrained=true;cand&=sideMask[t][std::size_t(sd)];sidesSeen[f]|=char(1<<sd);}
    std::uint32_t got=0;
    how=1;if(constrained&&cand){region=true;got=(cand&(cand-1))?bySkin(cand):cand;wantIn|=got;}
    else{got=bySkin(all);if(got)++s.bySkeleton;wantFree|=got;}
    want|=got;
    for(std::size_t k=0;k<P;++k)if((got>>k)&1u)for(std::size_t i=0;i<K;++i)if(inR[i]>=0)s.touch[k][i]|=char(how);}
   if(region)++s.inRegion;
   // cutfix: one piece per triangle unless a fresh tear's surface crosses it (its samples on both sides: the programs split it per
   // pixel by that tear): a triangle partly in a region and partly outside goes to the region's piece only (outside every region
   // no program hides anything, so a second piece drew that part twice)
   if(wantIn){bool crossing=false;for(const auto sd:sidesSeen)crossing=crossing||sd==3;want=crossing?(wantIn|wantFree):wantIn;}
   // the owner's design: the pieces owning the triangle's sample cells draw it (each pixel then by its cell's owner alone)
   const std::uint32_t wantRule=want|tieAll|wantIn|wantFree; // the former rule (ties to every tied piece): a piece left empty takes these
   if(owned)want=owned;
   bool any=false;
   for(std::size_t k=0;k<P;++k){const bool main=((want>>k)&1u)!=0;if(!main&&!((wantRule>>k)&1u))continue;if(main)any=true;
    if(!main){const auto&b=own[k];if(!b.valid())continue;bool nearOwn=false;for(const auto&q:samples){const double cx=(q.x-grid.origin.x)/grid.pitch,cy=(q.y-grid.origin.y)/grid.pitch,cz=(q.z-grid.origin.z)/grid.pitch;
      nearOwn=nearOwn||(cx>=b.lo[0]-8&&cx<b.hi[0]+8&&cy>=b.lo[1]-8&&cy<b.hi[1]+8&&cz>=b.lo[2]-8&&cz<b.hi[2]+8);}if(!nearOwn){s.altFar[k].push_back(std::uint32_t(t));continue;}}
    bool fine=anchored(k,*v[0])&&anchored(k,*v[1])&&anchored(k,*v[2]);auto&anc=main?s.anchors[k]:s.altAnchors[k];
    if(!fine){bool freeze=frozenPosable(k,*v[0])&&frozenPosable(k,*v[1])&&frozenPosable(k,*v[2]);if(!freeze){if(main)++s.unposable;continue;}
     for(const auto*row:v)for(unsigned w=0;w<4;++w){const auto bb=row->bone[w];if(bb<64&&!ok[k][bb]&&gfy_signed_hw::dependency(row->weight,w,negligible_skin_weight))anc.emplace(bb,front[k][bb]);}
     if(main)++s.frozen;}
    (main?s.tris[k]:s.alt[k]).push_back(std::uint32_t(t));}
   if(!any)++s.dropped;}}});
 for(std::size_t ci=0;ci<prChunks.size();++ci){auto&dst=sel[prChunks[ci].c];auto&s=prPart[ci]; // piecesregional: chunk order = triangle order
  for(std::size_t k=0;k<P;++k){dst.tris[k].insert(dst.tris[k].end(),s.tris[k].begin(),s.tris[k].end());dst.alt[k].insert(dst.alt[k].end(),s.alt[k].begin(),s.alt[k].end());
   dst.altFar[k].insert(dst.altFar[k].end(),s.altFar[k].begin(),s.altFar[k].end());dst.anchors[k].insert(s.anchors[k].begin(),s.anchors[k].end());
   dst.altAnchors[k].insert(s.altAnchors[k].begin(),s.altAnchors[k].end());for(std::size_t i=0;i<K;++i)dst.touch[k][i]|=s.touch[k][i];}
  dst.dropped+=s.dropped;dst.unposable+=s.unposable;dst.frozen+=s.frozen;dst.inRegion+=s.inRegion;dst.bySkeleton+=s.bySkeleton;dst.byVolume+=s.byVolume;dst.ties+=s.ties;dst.ghosts+=s.ghosts;}
 // cutfix: a piece the cell owners left without a triangle takes those of the selection's own rule (never an empty piece)
 for(std::size_t k=0;k<P;++k){std::size_t n=0,na=0;for(const auto&s:sel)if(!s.tris.empty()){n+=s.tris[k].size();na+=s.alt[k].size();}if(n)continue;
  for(auto&s:sel){if(s.tris.empty())continue;s.tris[k].insert(s.tris[k].end(),s.alt[k].begin(),s.alt[k].end());for(const auto&a:s.altAnchors[k])s.anchors[k].insert(a);++stats.altPieces;}
  if(!na)for(auto&s:sel){if(s.tris.empty())continue;const auto&src=children[std::size_t(&s-sel.data())]; // nothing near: any of the former rule's (posable)
   for(const auto tt:s.altFar[k]){const gfy_packed::Vertex*v[3]={&src.vertices[src.indices[3*tt]],&src.vertices[src.indices[3*tt+1]],&src.vertices[src.indices[3*tt+2]]};
    if(anchored(k,*v[0])&&anchored(k,*v[1])&&anchored(k,*v[2])){s.tris[k].push_back(tt);continue;}
    if(!(frozenPosable(k,*v[0])&&frozenPosable(k,*v[1])&&frozenPosable(k,*v[2])))continue;
    for(const auto*row:v)for(unsigned w=0;w<4;++w){const auto bb=row->bone[w];if(bb<64&&!ok[k][bb]&&gfy_signed_hw::dependency(row->weight,w,negligible_skin_weight))s.anchors[k].emplace(bb,front[k][bb]);}
    s.tris[k].push_back(tt);}}}
 stats.selectMs=core::ms(s0);const auto p0=core::Clock::now();
 for(const auto&s:sel){stats.dropped+=s.dropped;stats.unposable+=s.unposable;stats.frozen+=s.frozen;stats.inRegion+=s.inRegion;stats.bySkeleton+=s.bySkeleton;stats.byVolume+=s.byVolume;stats.ties+=s.ties;
  stats.ghosts+=s.ghosts;}
 long long windingVote=0;
 for(const auto&src:children){if(src.material||isBox(src))continue;for(std::size_t t=0;t<src.indices.size()/3&&t<4096;++t){
   const auto&A=src.vertices[src.indices[3*t]];const auto&B=src.vertices[src.indices[3*t+1]];const auto&Cc=src.vertices[src.indices[3*t+2]];
   const Vec e1=position(B)-position(A),e2=position(Cc)-position(A);const Vec g{e1.y*e2.z-e1.z*e2.y,e1.z*e2.x-e1.x*e2.z,e1.x*e2.y-e1.y*e2.x};
   const Vec nn{A.surface[3]+B.surface[3]+Cc.surface[3],A.surface[4]+B.surface[4]+Cc.surface[4],A.surface[5]+B.surface[5]+Cc.surface[5]};
   const double s=dot(g,nn);windingVote+=s>0?1:(s<0?-1:0);}}
 const bool ccwOutward=windingVote>=0;
 // A split can contain both a fresh joint tear and a fresh burst. Its
 // preceding publication has no burst proxy yet. Locate the retained cells
 // inside each new burst using the final connectivity spans once per record.
 std::vector<std::vector<head_burst_pieces::CarryBox>> burstStarts(P);
 std::vector<std::vector<CellBox>> burstBounds(K);
 for(std::size_t t=0;t<K;++t){
  const auto&tr=published.tears[t];
  if(!fresh(tr)||!tear_atlas::burstRecord(*tr.block))continue;
  const auto& h=tr.block->header;
  if(h.size()!=104||h[14]!=2u||h[27]!=0u)continue;
  const double radius=tear_atlas::f32(h,3);
  const std::array<double,3> centre={tear_atlas::f32(h,0),tear_atlas::f32(h,1),tear_atlas::f32(h,2)};
  if(!(radius>0))continue;
  auto&bounds=burstBounds[t];bounds.resize(P);
  const auto lower=[&](int a,double d){return std::max(0,int(std::ceil((centre[a]-d-grid.origin[a])/grid.pitch-.5)));};
  const auto upper=[&](int a,double d){return std::min((&grid.size.x)[a],1+int(std::floor((centre[a]+d-grid.origin[a])/grid.pitch-.5)));};
  const int x0=lower(0,radius),x1=upper(0,radius),y0=lower(1,radius),y1=upper(1,radius);
  for(int y=y0;y<y1;++y)for(int x=x0;x<x1;++x){
   const auto ci=pieceAt.columnOf[std::size_t(y)*grid.size.x+x];
   if(ci<0)continue;
   const double dx=grid.origin.x+(x+.5)*grid.pitch-centre[0],dy=grid.origin.y+(y+.5)*grid.pitch-centre[1];
   const double zz=radius*radius-dx*dx-dy*dy;
   if(zz<=0.)continue;
   const double dz=std::sqrt(zz);
   const int z0=lower(2,dz),z1=upper(2,dz);
   const auto&col=volume.columns[std::size_t(ci)];
   for(auto i=col.first;i<col.first+col.count;++i){
    const auto&sp=volume.spans[i];
    if(sp.component>=pieceAt.pieceOf.size())continue;
    const int k=pieceAt.pieceOf[sp.component],lo=std::max(z0,sp.lo),hi=std::min(z1,sp.hi);
    if(k>=0&&lo<hi)bounds[std::size_t(k)].add(x,y,lo,hi);
   }
  }
 }
 // the tears each piece keeps: unsplit ones it lies on (its voted side), inherited ones it borders (the parent's side)
 std::vector<std::vector<gfy_packed::TearRef>> keep(P);
 for(std::size_t k=0;k<P;++k)for(std::size_t t=0;t<K;++t){const auto&tr=published.tears[t];int side=-1;
  if(tear_atlas::burstRecord(*tr.block)){
   if(auto c=burstFor(tr,pieceAt,k)){
    keep[k].push_back({c,3});
    if(!burstBounds[t].empty()&&burstBounds[t][k].valid()){
     const auto& bounds=burstBounds[t][k];
     head_burst_pieces::CarryBox box;
     for(int a=0;a<3;++a){
      box.b[a]=grid.origin[a]+bounds.lo[a]*grid.pitch-.035;
      box.b[a+3]=grid.origin[a]+bounds.hi[a]*grid.pitch+.035;
     }
     burstStarts[k].push_back(box);
    }
   }
   continue;
  }
  if(fresh(tr))side=stats.sig[t][k];
  else{int touched=0;for(const auto&s:sel)if(!s.touch.empty())touched|=s.touch[k][t];
   const bool voted=tr.side<2?stats.votes[t][std::size_t(tr.side)*maxPieces+k]>0:(stats.votes[t][k]+stats.votes[t][maxPieces+k])>0;
   if(voted||(touched&1))side=tr.side; // 8 = only by an unresolved tie (the triangle went to every tied piece): not a border
   if(std::getenv("GFY_M3_PIECEDBG"))std::fprintf(stderr,"M3KEEP piece=%zu tear=%zu side=%d touched=%d voted=%d votesA=%lld votesB=%lld\n",k,t,int(tr.side),touched,int(voted),
    stats.votes[t][k],stats.votes[t][maxPieces+k]);}
  if(side<0)continue;keep[k].push_back({tr.block,std::uint8_t(side)});if(side==2)++stats.cracks;}
 // cutfix: each piece's own cells' box, and the box of everything it draws (its cells and its selected exterior triangles)
 const auto&compBoxes=spans.comp;
 std::vector<CellBox> drawn(P);
 for(std::size_t k=0;k<P;++k){drawn[k]=own[k];
  for(std::size_t c=0;c<children.size();++c){if(sel[c].tris.empty())continue;const auto&src=children[c];
   for(const auto t:sel[c].tris[k])for(int j=0;j<3;++j){const auto&vv=src.vertices[src.indices[3*t+j]];
    const int x=int(std::floor((vv.surface[0]-grid.origin.x)/grid.pitch)),y=int(std::floor((vv.surface[1]-grid.origin.y)/grid.pitch)),z=int(std::floor((vv.surface[2]-grid.origin.z)/grid.pitch));
    drawn[k].add(x,y,z,z+1);}}
  if(drawn[k].valid()){const int sz[3]={grid.size.x,grid.size.y,grid.size.z}; // + 3 cells, tile aligned, inside the grid
   for(int a=0;a<3;++a){drawn[k].lo[a]=std::max(0,drawn[k].lo[a]-3)&~7;drawn[k].hi[a]=std::min((sz[a]+7)&~7,((drawn[k].hi[a]+3)+7)&~7);}}}
 // cutfix: a piece keeps the tears whose torn zone meets what it draws; a coverage box for those whose zone meets its own cells
 std::vector<std::vector<char>> boxFor(P);
 for(std::size_t k=0;k<P;++k){const auto ownW=worldBox(grid,own[k],2),drawW=worldBox(grid,drawn[k],0);std::vector<gfy_packed::TearRef> kept;
  for(const auto&tr:keep[k]){if(tr.block&&tear_atlas::burstRecord(*tr.block)){kept.push_back(tr);boxFor[k].push_back(0);continue;} // headfix
   if(tr.block)tear_bake::await(tr.block.get());const auto z=tear_atlas::zone(*tr.block);
   if(z.ok&&drawW.valid()&&!z.box.meets(drawW)){++stats.prunedTears;continue;}
   kept.push_back(tr);boxFor[k].push_back(char(!z.ok||!ownW.valid()||z.box.meets(ownW)));if(!boxFor[k].back())++stats.prunedBoxes;}
  if(kept.empty()&&!keep[k].empty()){kept.push_back(keep[k].front());boxFor[k].assign(1,1);} // never without a set: the first kept as before
  keep[k]=std::move(kept);}
 // seethrough: the burst boxes the published mesh has (a tear split selected none and made none for records), each to the piece
 // that keeps its crater (HeadBurstPieces.h carryBoxes)
 const auto stCarry=head_burst_pieces::carryBoxes(children,pieceAt,P,ok);
 // cutends (2026-11-16): the published mesh's cut-end children of the tears no piece here keeps (a mask cut's box: a head shot off
 // has no tear) go, unchanged, to the piece that keeps that end (HeadBurstPieces.h cutEndCarry); the kept tears' are made again below
 const auto ceCut=head_burst_pieces::cutEndCarry(published,children,pieceAt,P,ok,[&](std::size_t k,const gfy_packed::TearBlock*T){for(const auto&tr:keep[k])if(tr.block.get()==T)return true;return false;},
  [&](const gfy_packed::TearBlock*T){for(std::size_t k=0;k<P;++k)for(const auto&tr:keep[k])if(tr.block.get()==T)return true;return false;});
 if((ceCut.boxes||ceCut.starters||ceCut.dropped)&&(std::getenv("GFY_M3_LOG")||std::getenv("GFY_PIECE_LOG")))
  std::fprintf(stderr,"cutends tear_split boxes=%zu starters=%zu remade=%zu dropped=%zu\n",ceCut.boxes,ceCut.starters,ceCut.remade,ceCut.dropped);
 std::vector<bound_surface::Piece> out(P);std::vector<std::uint32_t> empty;
 static const bool gfCheck=std::getenv("GFY_GF_CHECK")!=nullptr;std::vector<std::unordered_map<Cell,core::Tile,core::CellHash>> gfChk(gfCheck?P:std::size_t(0)),gfChkO(gfChk.size()),gfChkR(gfChk.size()); // gearfine: probes, the marks of each piece
 if(marksOut)marksOut->assign(P,{});
 // headfix: clothing just outside a fresh tear's footprint takes the tear's side (HeadBurstPieces.h tearRing / tearRingMarks)
 const auto hbT0=core::Clock::now();double hbWaitMs=0,hbMarkMs=0;
 const head_burst_pieces::TearNear hbNear=head_burst_pieces::tearNear(published,grid,volume,pieceAt);std::vector<head_burst_pieces::TearRing> hbRings;std::size_t hbRingCells=0;
 if(hbNear.ok)for(const auto&tr:published.tears)if(fresh(tr)&&!tear_atlas::burstRecord(*tr.block)){const auto w0=core::Clock::now();tear_bake::await(tr.block.get());hbWaitMs+=core::ms(w0);hbRings.push_back(head_burst_pieces::tearRing(*tr.block));}
 const double hbPrepMs=core::ms(hbT0);
 // headmask2: the boolean remove (HeadBurstPieces.h unattached) - every piece of every decide-once separation, in the cut's box
 head_burst_pieces::AttachStats hmStats;std::size_t hmBig=0;for(std::size_t k=1;k<P;++k)if(volume.components[components[k]].cells>volume.components[components[hmBig]].cells)hmBig=k;
 const CellBox hmCut=head_burst_pieces::cutBox(own,hmBig);const auto hmCuts=head_burst_pieces::cutBoxes(own,hmBig);const gear_cells::GearCells*hmGear=gear&&gear->fits(grid.origin,grid.pitch,grid.size)?gear:nullptr;
 const painter_detail::TissueField*hmTf=published.tissue_field&&published.tissue_field->available?published.tissue_field.get():nullptr;
 const auto hmPad=hmTf?gear_cells::padTileFlags(published.tissue_field,grid):std::shared_ptr<const std::vector<std::uint8_t>>();
 std::vector<Cell> hmGone=P>=2?head_burst_pieces::goneBlocks(grid,hmTf,hmCut):std::vector<Cell>();head_burst_pieces::PadCache hmPads;
 // gearfine: the scrap rule's gone anatomy leaves out the fresh tears' SHEETS (the removed cells within 2 cells of a fresh tear's
 // surface inside its region): the sheet is the cut, not a wound - judged as gone anatomy it cut the gear and padding along the cut
 // line from both parts (a strip no part drew); which part a cell there belongs to is owners()' (the barrier above)
 {static const bool gfSheetGone=std::getenv("GFY_GF_SHEET_GONE")!=nullptr; // probe comparisons: the sheets stay gone anatomy
  if(!gfSheetGone&&!hmGone.empty()&&!freshIdx.empty()){const double lim=2.*grid.pitch;std::size_t w=0;
   for(std::size_t i=0;i<hmGone.size();++i){const Cell&c=hmGone[i];const double x=grid.origin.x+(c.x+.5)*grid.pitch,y=grid.origin.y+(c.y+.5)*grid.pitch,z=grid.origin.z+(c.z+.5)*grid.pitch;
    bool sheet=false;for(const auto t:freshIdx){const Region&Rg=R[t];const double s0=(x-Rg.C[0])*Rg.n[0]+(y-Rg.C[1])*Rg.n[1]+(z-Rg.C[2])*Rg.n[2];
     if(std::abs(s0)>tear_decide::offMax+lim)continue;double dd=0;if(Rg.dist(x,y,z,dd)&&std::abs(dd)<=lim){sheet=true;break;}}
    if(!sheet)hmGone[w++]=c;}
   hmGone.resize(w);}}
 const FloatPadArea fpArea=P>=2?floatPadArea(grid,published.tissue_field,hmPad.get(),R,freshIdx,own,hmBig):FloatPadArea();
 for(std::size_t k=0;k<P;++k){auto&piece=out[k];piece.volume_component=components[k];auto&mesh=piece.mesh;
  unsigned next=originalChildren; // splitfix: appended slots start right after the original children in every piece; numbered above the source's own appended slots, a cut part's pieces left a gap the native packet check refuses (no re-split was ever staged)
  for(std::size_t c=0;c<children.size();++c){const auto&src=children[c];const auto&list=sel[c].tris.empty()?std::vector<std::uint32_t>():sel[c].tris[k];if(list.empty())continue;
   for(const auto&a:sel[c].anchors[k]){auto [entry,freshA]=piece.constrained_anchors.emplace(a);need(freshA||entry->second==a.second,"mask_piece3_frozen_anchor_conflict");}
   gfy_packed::Child child;child.material=0;child.material_source_child=src.material_source_child;child.skin_clip=true;
   if(src.child_index<originalChildren)child.child_index=src.child_index;else{need(next<gfy_packed::max_child_slots,"mask_piece3_child_slots");child.child_index=std::uint16_t(next++);}
   std::vector<std::int32_t> map(src.vertices.size(),-1);const bool faces=src.original_faces.size()*3==src.indices.size();
   for(const auto t:list){for(int j=0;j<3;++j){const auto v=src.indices[3*t+j];
     if(map[v]<0){map[v]=std::int32_t(child.vertices.size());child.vertices.push_back(src.vertices[v]);}
     child.indices.push_back(std::uint16_t(map[v]));}
    child.original_faces.push_back(faces?src.original_faces[t]:~std::uint32_t(0));}
   need(child.vertices.size()<=65535,"mask_piece3_vertex_budget");
   piece.triangles+=list.size();stats.selected+=list.size();mesh.children.push_back(std::move(child));}
  // one coverage box per kept tear: the tear slab, skinned to the bone of this piece's kept vertex nearest the box centre
  for(std::size_t q=0;q<keep[k].size();++q){if(!boxFor[k][q])continue;const auto&tr=keep[k][q];const auto&T=*tr.block;painter_mask_pieces::Box box;const double ma=.5*(T.footLo[0]+T.footHi[0]),mb=.5*(T.footLo[1]+T.footHi[1]);
   for(int a=0;a<3;++a){box.centre[std::size_t(a)]=T.centre[a]+ma*T.u[a]+mb*T.w[a];box.axis[0][std::size_t(a)]=T.u[a];box.axis[1][std::size_t(a)]=T.w[a];box.axis[2][std::size_t(a)]=T.normal[a];}
   const double pad=2*T.pitch;box.lo={-(.5*(T.footHi[0]-T.footLo[0])+pad),-(.5*(T.footHi[1]-T.footLo[1])+pad),-T.reach};box.hi={-box.lo[0],-box.lo[1],T.reach};box.valid=T.footHi[0]>T.footLo[0];
   if(!box.valid||next>=gfy_packed::max_child_slots)continue;
   const Vec boxCentre{box.centre[0],box.centre[1],box.centre[2]};double bestD=1e300;std::uint16_t boxBone=65535,boxMaterialSource=0;
   for(const auto&ch:mesh.children)for(const auto&row:ch.vertices){unsigned best=0;for(unsigned w=1;w<4;++w)if(row.weight[w]>row.weight[best])best=w;
    const double dd=length(position(row)-boxCentre);if(dd<bestD&&row.bone[best]<64&&ok[k][row.bone[best]]){bestD=dd;boxBone=row.bone[best];boxMaterialSource=ch.material_source_child;}}
   if(boxBone==65535)continue;
   const auto corner=painter_mask_pieces::corners(box);const auto tri=painter_mask_pieces::triangles(box);
   gfy_packed::Child child;child.material=0;child.material_source_child=boxMaterialSource;child.skin_clip=true;child.child_index=std::uint16_t(next++);
   for(std::size_t f=0;f<12;++f){const std::uint16_t qv[3]={tri[3*f],tri[3*f+1],tri[3*f+2]};
    const Vec A{corner[qv[0]][0],corner[qv[0]][1],corner[qv[0]][2]},B{corner[qv[1]][0],corner[qv[1]][1],corner[qv[1]][2]},Cq{corner[qv[2]][0],corner[qv[2]][1],corner[qv[2]][2]};
    Vec nn{(B-A).y*(Cq-A).z-(B-A).z*(Cq-A).y,(B-A).z*(Cq-A).x-(B-A).x*(Cq-A).z,(B-A).x*(Cq-A).y-(B-A).y*(Cq-A).x};const double ln=length(nn);if(!(ln>1e-12))continue;nn=nn*(1./ln);
    const bool outward=dot(nn,A-boxCentre)>0;
    const std::uint16_t order[3]={qv[0],outward==ccwOutward?qv[1]:qv[2],outward==ccwOutward?qv[2]:qv[1]};
    const Vec no=outward?nn:nn*-1.;const Vec tg=std::abs(no.y)<.9?Vec{no.z,0,-no.x}:Vec{0,-no.z,no.y};const Vec tn=tg*(1./length(tg));
    const Vec bn{no.y*tn.z-no.z*tn.y,no.z*tn.x-no.x*tn.z,no.x*tn.y-no.y*tn.x};
    for(const auto qi:order){gfy_packed::Vertex row;
     const double*p=corner[qi].data();const double sv[14]={p[0],p[1],p[2],no.x,no.y,no.z,tn.x,tn.y,tn.z,bn.x,bn.y,bn.z,-1000.,0.};
     for(unsigned a=0;a<14;++a)row.surface[a]=float(sv[a]);
     row.bone={boxBone,boxBone,boxBone,boxBone};row.weight={1.f,0.f,0.f,0.f};gfy_packed::canonical_weights(row);
     child.indices.push_back(std::uint16_t(child.vertices.size()));child.vertices.push_back(row);}
    child.original_faces.push_back(~std::uint32_t(0));}
   stats.boxTriangles+=child.indices.size()/3;++stats.boxes;mesh.children.push_back(std::move(child));}
  if(!stCarry[k].empty()||!burstStarts[k].empty()){ // inherited and newly visible burst faces
   std::vector<head_burst_pieces::CarryBox> L;
   auto boxes=stCarry[k];
   boxes.insert(boxes.end(),burstStarts[k].begin(),burstStarts[k].end());
   for(auto x:boxes){if(x.bone>=64||!ok[k][x.bone]){const Vec cc{.5*(x.b[0]+x.b[3]),.5*(x.b[1]+x.b[4]),.5*(x.b[2]+x.b[5])};double bestD=1e300;x.bone=65535;
     for(const auto&ch:mesh.children)for(const auto&row:ch.vertices){unsigned best=0;for(unsigned w=1;w<4;++w)if(row.weight[w]>row.weight[best])best=w;
      const double dd=length(position(row)-cc);if(dd<bestD&&row.bone[best]<64&&ok[k][row.bone[best]]){bestD=dd;x.bone=row.bone[best];if(!x.kept)x.src=ch.material_source_child;}}
     if(x.bone==65535)for(unsigned bb=0;bb<64;++bb)if(ok[k][bb]){x.bone=std::uint16_t(bb);break;}
     if(x.bone==65535)continue;}
    L.push_back(x);}
   head_burst_pieces::mergeCarryBoxes(L,next<gfy_packed::max_child_slots?std::size_t(gfy_packed::max_child_slots-next):std::size_t(0));
   for(const auto&x:L){std::size_t bt=0;head_burst_pieces::emitCarryBox(mesh,x,next,ccwOutward,bt);stats.boxTriangles+=bt;++stats.boxes;}
   if(std::getenv("GFY_M3_LOG")||std::getenv("GFY_PIECE_LOG"))std::fprintf(stderr,"seethrough_boxes tear piece=%zu kept=%zu drawn=%zu\n",k,stCarry[k].size(),L.size());}
  // cutends: the published mesh's cut-end children this piece keeps (the ones no kept tear makes again), as they were
  for(const auto c:ceCut.of[k]){const std::size_t t=head_burst_pieces::emitCutChild(mesh,children[c],next,ok[k]);stats.boxTriangles+=t;if(t)++stats.boxes;}
  if(!piece.triangles){empty.push_back(components[k]);continue;}
  // the removal field: the grid's removed tiles + the inherited marks (a piece of a piece keeps hiding what its parent hid)
  // cutfix: + the other pieces' cells inside this piece's own cells' box (+ 2 cells: the volume the programs search), + every tile
  // the piece's triangles reach that lies near another piece and not near this one (whole tiles; clothing padding is no piece's cell);
  // both only outside every kept tear's torn zone (there the tear decides, and the march must meet the other side as its torn face);
  // inherited marks kept inside the
  // box of what the piece draws only (its children lie inside it)
  struct FieldTiles {std::unordered_map<Cell,core::Tile,core::CellHash> tiles;const void* ownership=nullptr;double pitch;Vec origin;decltype(grid.size) size;};
  FieldTiles field{{},nullptr,grid.pitch,grid.origin,grid.size};
  for(const auto&t:grid.tiles){std::uint64_t any=0;for(auto wd:t.second.removed)any|=wd;if(any){
   auto v=t.second;
   const auto who=floorOwner.find(t.first);
   if(who!=floorOwner.end()&&who->second==int(k)){
    const auto& relief=grid.visibleFloor->at(t.first);
    for(unsigned j=0;j<8;++j)v.removed[j]&=~relief.removed[j];
   }
   field.tiles.emplace(t.first,v);}}
  const CellBox&mb=drawn[k];auto inBox=[&](const Cell&t){return mb.valid()&&t.x*8>=mb.lo[0]&&t.x*8<mb.hi[0]&&t.y*8>=mb.lo[1]&&t.y*8<mb.hi[1]&&t.z*8>=mb.lo[2]&&t.z*8<mb.hi[2];};
  std::unordered_map<Cell,core::Tile,core::CellHash> mine;
  {const auto m0=core::Clock::now();CellBox ob=own[k];if(ob.valid())for(int a=0;a<3;++a){ob.lo[a]-=2;ob.hi[a]+=2;}
   std::vector<std::pair<tear_atlas::Zone,const gfy_packed::TearBlock*>> regs;
   for(const auto&tr:keep[k])if(tr.block&&!tear_atlas::burstRecord(*tr.block)){const auto z=tear_atlas::zone(*tr.block);if(z.ok)regs.push_back({z,tr.block.get()});} // headfix: records apart
   std::vector<std::vector<std::uint8_t>> regDepth;regDepth.reserve(regs.size());for(const auto&r:regs)regDepth.push_back(regionDepth(r.first,*r.second));
   std::unordered_map<Cell,char,core::CellHash> regionCache;
   // a tear's torn zone (TearAtlas.h inZone): a cell whose centre lies within half a cell diagonal of it is the tear's to decide
   const double half=.87*grid.pitch,tileHalf=8*.87*grid.pitch;
   // gearfine: a kept tear's zone is the tear's to decide only inside the box of the piece's cells (+ 2 cells: the box the coverage
   // rays search, where they draw its torn faces). Past it (gear: a pack's inside where the footprint's map reaches it) no ray of this
   // piece draws the tear's decision, and leaving the cells unmarked there left the cut gear open (square holes from below): the
   // owners' marks hold there. GFY_GF_ZONE_EVERYWHERE=1 (probes): zones everywhere, as before
   static const bool gfZoneAll=std::getenv("GFY_GF_ZONE_EVERYWHERE")!=nullptr;
   auto pointIn=[&](const double*pq,double margin){
    if(!gfZoneAll&&ob.valid()){const double cx=(pq[0]-grid.origin.x)/grid.pitch,cy=(pq[1]-grid.origin.y)/grid.pitch,cz=(pq[2]-grid.origin.z)/grid.pitch,mc=margin/grid.pitch;
     if(cx<ob.lo[0]-mc||cx>ob.hi[0]+mc||cy<ob.lo[1]-mc||cy>ob.hi[1]+mc||cz<ob.lo[2]-mc||cz>ob.hi[2]+mc)return false;}
    for(const auto&r:regs)if(tear_atlas::inZone(r.first,*r.second,pq,margin))return true;return false;};
   auto tileClassRaw=[&](const Cell&tq){const double c[3]={grid.origin.x+(tq.x*8+4)*grid.pitch,grid.origin.y+(tq.y*8+4)*grid.pitch,grid.origin.z+(tq.z*8+4)*grid.pitch};
    if(!pointIn(c,tileHalf+half))return 0; // no zone within the tile's reach: every other piece's cell in it is marked
    int in=0;for(int q=0;q<8;++q){const double pq[3]={grid.origin.x+(tq.x*8+((q&1)?8:0))*grid.pitch,grid.origin.y+(tq.y*8+((q&2)?8:0))*grid.pitch,grid.origin.z+(tq.z*8+((q&4)?8:0))*grid.pitch};
     in+=pointIn(pq,-half)?1:0;}
    if(in!=8)return 2;
    for(std::size_t ri=0;ri<regs.size();++ri)if(tear_atlas::inZone(regs[ri].first,*regs[ri].second,c,tileHalf+half)&&regionWhole(regs[ri].first,*regs[ri].second,c,4*grid.pitch,&regDepth[ri]))return 1;
    return 2;};
   auto tileClass=[&](const Cell&tq){auto it=regionCache.find(tq);if(it!=regionCache.end())return int(it->second);const int v=tileClassRaw(tq);regionCache.emplace(tq,char(v));return v;};
   auto tileInRegion=[&](const Cell&tq){return tileClass(tq)!=0;};
   auto cellIn=[&](int x,int y,int z){const double pq[3]={grid.origin.x+(x+.5)*grid.pitch,grid.origin.y+(y+.5)*grid.pitch,grid.origin.z+(z+.5)*grid.pitch};
    if(!gfZoneAll&&ob.valid()){const double cx=x+.5,cy=y+.5,cz=z+.5,mc=half/grid.pitch;
     if(cx<ob.lo[0]-mc||cx>ob.hi[0]+mc||cy<ob.lo[1]-mc||cy>ob.hi[1]+mc||cz<ob.lo[2]-mc||cz>ob.hi[2]+mc)return false;}
    for(std::size_t ri=0;ri<regs.size();++ri)if(tear_atlas::inZone(regs[ri].first,*regs[ri].second,pq,half)&&regionWhole(regs[ri].first,*regs[ri].second,pq,.5*grid.pitch,&regDepth[ri]))return true;
    return false;};
   if(!(owns.valid()&&int(k)==owns.L))otherCells(volume,components[k],ob,compBoxes,mine,tileClass,cellIn);
   // the gear and padding (and tissue) another piece owns, inside the box of what this piece draws, outside its kept tears' zones:
   // a tile one other piece owns whole is hidden whole; where owners meet, cell by cell
   if(owns.valid()){
    // the tiles to look at: the largest piece - the decided region's tiles inside what it draws (another piece owns tiles only
    // there); a smaller piece - its own cells' box (+2 cells: the programs' search) and the tiles its triangles' surfaces cross
    std::vector<std::uint32_t> look;const CellBox&db=drawn[k];
    auto inDrawn=[&](int tx,int ty,int tz){return db.valid()&&tx*8+8>db.lo[0]&&tx*8<db.hi[0]&&ty*8+8>db.lo[1]&&ty*8<db.hi[1]&&tz*8+8>db.lo[2]&&tz*8<db.hi[2];};
    if(int(k)==owns.L){for(const auto u:owns.ulist){const int tx=int(u%std::size_t(owns.tnx)),ty=int((u/std::size_t(owns.tnx))%std::size_t(owns.tny)),tz=int(u/(std::size_t(owns.tnx)*owns.tny));
      if(inDrawn(tx,ty,tz))look.push_back(u);}}
    // gearback: an exterior pixel's ray searches the box of everything the piece draws (the atlas below), so a smaller piece hides
    // what the others own in ALL of that box: every tile of the decided region inside it (as the largest piece), and outside the
    // region (the largest piece's by rule) the tiles holding its tissue, gear or padding - and, as before, every tile of its own
    // cells' box there (what the coverage box rays search keeps its marks to the cell). Needs the outfit's gear tiles (their band
    // holds every triangle's surface); without them the former tiles (own box, the triangles' surfaces) are looked at.
    else if(gear&&gear->fits(grid.origin,grid.pitch,grid.size)&&db.valid()){
     for(const auto u:owns.ulist){const int tx=int(u%std::size_t(owns.tnx)),ty=int((u/std::size_t(owns.tnx))%std::size_t(owns.tny)),tz=int(u/(std::size_t(owns.tnx)*owns.tny));
      if(inDrawn(tx,ty,tz))look.push_back(u);}
     if(owns.L>=0){const auto padF=(published.tissue_field&&published.tissue_field->available)?gear_cells::padTileFlags(published.tissue_field,grid):std::shared_ptr<const std::vector<std::uint8_t>>();
      const auto&gearF=gear->tileFlag;const TileSet&big=spans.tiles[std::size_t(owns.L)];
      auto inOwn=[&](int tx,int ty,int tz){return ob.valid()&&tx>=(ob.lo[0]>>3)&&tx<=((ob.hi[0]-1)>>3)&&ty>=(ob.lo[1]>>3)&&ty<=((ob.hi[1]-1)>>3)&&tz>=(ob.lo[2]>>3)&&tz<=((ob.hi[2]-1)>>3);};
      for(int tz=std::max(0,db.lo[2]>>3);tz<=std::min(owns.tnz-1,(db.hi[2]-1)>>3);++tz)for(int ty=std::max(0,db.lo[1]>>3);ty<=std::min(owns.tny-1,(db.hi[1]-1)>>3);++ty)
       for(int tx=std::max(0,db.lo[0]>>3);tx<=std::min(owns.tnx-1,(db.hi[0]-1)>>3);++tx){if(owns.inRoi(tx,ty,tz))continue;const std::size_t i=owns.idx(tx,ty,tz);
        if(inOwn(tx,ty,tz)||big.in(tx,ty,tz)||(i<gearF.size()&&gearF[i])||(padF&&i<padF->size()&&(*padF)[i]))look.push_back(std::uint32_t(i));}}}
    else{std::unordered_set<std::uint32_t> seen;auto once=[&](int tx,int ty,int tz){if(!owns.inside(tx,ty,tz))return;const auto u=std::uint32_t(owns.idx(tx,ty,tz));if(seen.insert(u).second)look.push_back(u);};
     if(ob.valid())for(int tz=std::max(0,ob.lo[2]>>3);tz<=(ob.hi[2]-1)>>3;++tz)for(int ty=std::max(0,ob.lo[1]>>3);ty<=(ob.hi[1]-1)>>3;++ty)
      for(int tx=std::max(0,ob.lo[0]>>3);tx<=(ob.hi[0]-1)>>3;++tx)once(tx,ty,tz);
     const double step=4*grid.pitch;
     for(std::size_t c=0;c<children.size();++c){if(sel[c].tris.empty())continue;const auto&src=children[c];
      for(const auto tt:sel[c].tris[k]){const Vec A=position(src.vertices[src.indices[3*tt]]),B=position(src.vertices[src.indices[3*tt+1]]),C3=position(src.vertices[src.indices[3*tt+2]]);
       const double L0=std::max(length(B-A),std::max(length(C3-A),length(C3-B)));const int N=std::max(1,std::min(256,int(std::ceil(L0/step))));
       for(int i=0;i<=N;++i)for(int j=0;i+j<=N;++j){const Vec q=A+(B-A)*(double(i)/N)+(C3-A)*(double(j)/N);int tq[3];tileOf(q,tq[0],tq[1],tq[2]);once(tq[0],tq[1],tq[2]);}}}}
    // each tile: a tile one other piece owns whole is hidden whole (outside the kept zones); where owners meet, cell by cell
    std::vector<std::vector<std::pair<Cell,core::Tile>>> part(core::parallelWidth()+1);
    lowpoly_pieces::eachBlock(look.size(),32,[&](std::size_t first,std::size_t last,std::size_t thread){auto&out=part[std::min(thread,part.size()-1)];
     for(std::size_t li=first;li<last;++li){const std::size_t i=look[li];const int tx=int(i%std::size_t(owns.tnx)),ty=int((i/std::size_t(owns.tnx))%std::size_t(owns.tny)),
       tz=int(i/(std::size_t(owns.tnx)*owns.tny));const Cell tc{tx,ty,tz};core::Tile tile{};bool any=false;
      if(owns.slotAt(i)>=0){const auto&e=owns.tiles[std::size_t(owns.slotAt(i))];int cls=-1;
       for(int c=0;c<512;++c){const auto o=e.own[std::size_t(c)];if(o==255||o==std::uint8_t(k))continue;if(cls<0)cls=tileClassRaw(tc);if(cls==1)break;
        const int x=tx*8+(c>>6),y=ty*8+((c>>3)&7),z=tz*8+(c&7);if(cls==2&&cellIn(x,y,z))continue;tile.removed[std::size_t(x&7)]|=std::uint64_t(1)<<((y&7)*8+(z&7));any=true;}}
      else{const std::uint32_t m=(owns.L>=0&&!owns.inRoi(tx,ty,tz))?(1u<<owns.L):owns.maskAt(i);if(!m||(m&(m-1))||(m&(1u<<k)))continue;const int cls=tileClassRaw(tc);if(cls==1)continue;
       if(cls==0){for(auto&wd:tile.removed)wd=~std::uint64_t(0);any=true;}
       else for(int c=0;c<512;++c){const int x=tx*8+(c>>6),y=ty*8+((c>>3)&7),z=tz*8+(c&7);if(cellIn(x,y,z))continue;tile.removed[std::size_t(x&7)]|=std::uint64_t(1)<<((y&7)*8+(z&7));any=true;}}
      if(any)out.emplace_back(tc,tile);}});
    for(const auto&pp:part)for(const auto&e:pp){auto&tile=mine[e.first];for(unsigned q=0;q<8;++q)tile.removed[q]|=e.second.removed[q];++stats.padMarks;}}
   if(gfCheck)gfChkO[k]=mine; // gearfine: probes, the marks before the ring rule
   // headfix: the ring rule - no sliver of clothing standing over a split's other face
   if(hbNear.ok){std::vector<const head_burst_pieces::TearRing*> hbMine;std::vector<unsigned> hbSide;
    for(const auto&tr:keep[k]){if(!tr.block||tr.side>1)continue;for(const auto&rg:hbRings)if(rg.b==tr.block.get()){hbMine.push_back(&rg);hbSide.push_back(tr.side);}}
    const auto r0=core::Clock::now();for(std::size_t q=0;q<hbMine.size();++q)hbRingCells+=head_burst_pieces::tearRingMarks(*hbMine[q],hbSide[q],grid,drawn[k],hbNear,mine,hbMine);hbMarkMs+=core::ms(r0);}
   if(gfCheck)gfChkR[k]=mine; // gearfine: probes, the marks before the scrap rule
   // headmask2: the boolean remove - what is not attached to this piece's anatomy is cut away (a kept tear's zone is the tear's);
   // a piece's box: its cells' box 16 cells out (what it draws round them), the body's: each cluster of the smaller pieces' boxes inside that
   if(P>=2&&k!=hmBig){const CellBox pb=head_burst_pieces::attachBox(own[k],16);
    head_burst_pieces::unattached(grid,volume,pieceAt.pieceOf,k,pb,hmGear,hmPad.get(),hmTf,hmGone,mine,[&](const double*pq){return pointIn(pq,3.5*grid.pitch);},hmStats,hmGear?std::vector<Cell>():head_burst_pieces::shellCells(grid,children,sel,k,head_burst_pieces::attachBox(pb,4)),hmPads,inherited,nullptr,false,hmGear!=nullptr);} // gearfine: partsDecided (owners() decides between parts)
   else if(P>=2)for(const CellBox&cb:hmCuts){const CellBox pb=head_burst_pieces::attachClip(cb,own[k],16);
    head_burst_pieces::unattached(grid,volume,pieceAt.pieceOf,k,pb,hmGear,hmPad.get(),hmTf,hmGone,mine,[&](const double*pq){return pointIn(pq,3.5*grid.pitch);},hmStats,hmGear?std::vector<Cell>():head_burst_pieces::shellCells(grid,children,sel,k,head_burst_pieces::attachBox(pb,4)),hmPads,inherited,nullptr,false,hmGear!=nullptr);} // gearfine: partsDecided (owners() decides between parts)
   stats.markTiles+=mine.size();stats.marksMs+=core::ms(m0);}
  std::vector<std::pair<Cell,core::Tile>> marks;
  if(inherited)for(const auto&m:*inherited)if(inBox(m.first)){auto&t=mine[m.first];for(unsigned q=0;q<8;++q)t.removed[q]|=m.second.removed[q];}
  if(P>=2&&hmTf&&owns.valid()){std::vector<char> lcFresh(keep[k].size(),0);
   for(std::size_t i=0;i<keep[k].size();++i)for(const auto&t:published.tears)if(t.block==keep[k][i].block&&fresh(t)){lcFresh[i]=1;break;}
   LevelStats ls;levelCut(grid,*hmTf,hmGear,hmPad.get(),owns,k,keep[k],thickness(grid,volume,components[k],own[k],keep[k]),lcFresh,drawn[k],mine,ls,k!=hmBig,spans.tiles[k]);
   if(ls.tears&&(std::getenv("GFY_M3_LOG")||std::getenv("GFY_PIECE_LOG")))
    std::fprintf(stderr,"cutends level_cut piece=%zu tears=%zu looked=%zu marked=%zu far=%zu band=%zu level_mm=%.1f ms=%.2f\n",k,ls.tears,ls.looked,ls.marked,ls.far,ls.band,ls.level*1000.,ls.ms);}
  if(P>=2){FloatPadStats fs;floatPad(fpArea,grid,R,freshIdx,stats.sig,k,drawn[k],mine,fs,owns,k!=hmBig,hmTf);
   if(fs.nodes&&(std::getenv("GFY_M3_LOG")||std::getenv("GFY_PIECE_LOG")))
    std::fprintf(stderr,"cutends float_pad piece=%zu area_tiles=%zu nodes=%zu groups=%zu floating=%zu cells=%zu not_owned=%zu skin=%zu film=%zu ms=%.2f\n",k,fpArea.tiles.size(),fs.nodes,fs.groups,fs.floating,fs.cells,fs.foreign,fs.skin,fs.film,fs.ms);}
  if(gfCheck)gfChk[k]=mine; // gearfine: probes, keep the marks
  marks.reserve(mine.size());for(const auto&m:mine)marks.push_back(m);
  std::sort(marks.begin(),marks.end(),[](const std::pair<Cell,core::Tile>&a,const std::pair<Cell,core::Tile>&b){return a.first.x!=b.first.x?a.first.x<b.first.x:(a.first.y!=b.first.y?a.first.y<b.first.y:a.first.z<b.first.z);});
  for(const auto&m:marks){auto&t=field.tiles[m.first];for(unsigned q=0;q<8;++q)t.removed[q]|=m.second.removed[q];}
  // Only carry actual visible differences from this piece's published birth mask.
  // floorOwner excludes shared, ownerless and otherwise ambiguous bricks. Marks
  // added by this piece's tear, gear and ancestry take precedence.
  if(grid.visibleFloor){
   auto carried=std::make_shared<Grid::FloorTiles>();
   for(const auto& assignment:floorOwner){
    if(assignment.second!=int(k))continue;
    const auto raw=grid.findTile(assignment.first);
    const auto shown=field.tiles.find(assignment.first);
    if(!raw||shown==field.tiles.end())continue;
    const auto& floor=grid.visibleFloor->at(assignment.first);
    core::Tile kept{};bool any=false;
    for(unsigned j=0;j<8;++j){
     kept.removed[j]=floor.removed[j]&raw->removed[j]&~shown->second.removed[j];
     any|=kept.removed[j]!=0;
    }
    if(any)carried->emplace(assignment.first,kept);
   }
   if(!carried->empty())piece.grenadeFloor=std::move(carried);
  }
  auto removal=painter_detail::buildDamageField(field);need(removal&&removal->available,"mask_piece3_removal_field");
  stats.fieldBytes+=removal->bytes();
  if(marksOut)(*marksOut)[k]=std::move(marks);
  mesh.removal_field=std::move(removal);mesh.tissue_field=published.tissue_field;mesh.mask_volume=true;mesh.mask_piece=true;
  mesh.dents=published.dents;mesh.slits=published.slits; // melee (value 3): a piece keeps the dents and slits of the body it came from
  mesh.tears=keep[k];stats.kept+=keep[k].size();const auto th=thickness(grid,volume,components[k],own[k],mesh.tears);
  // gearfine (2026-11-15): GEAR STARTERS ON THE CUT ITSELF. gearback2's starters were the outer faces of 16 mm tiles whose pixels
  // searched 5 cm and drew any gear fill met after a hidden sample: every tile-shaped ownership error showed (a frame on a pack,
  // spikes and stripes under a cut, a square chunk off a limb end). Now the starters ARE the cut: every 2 mm cell face between gear
  // fill (the garment fill, label 80) this piece owns and keeps and a cell its marks hide (another piece's cells, a scrap rule's,
  // its parent's; not the grid's removed cells - a wound's wall is drawn through its own hole), merged greedily into rectangles per
  // facing (across depths where only hidden cells lie between), a quarter cell out on the hidden side and a quarter cell wider (no
  // crack at a step). A starter pixel crosses only hidden cells to the fill (painter_vmask_tear.h's starter search, unchanged), so only a real cut face is drawn, on the cells. Not in the
  // kept tears' zones (the tear draws there); gearback2 left out the whole cells' box (+ 2): a pack's front beside the torn section
  // and the gear between a torn section and a pack stayed open. Texture u = -2000
  // (a coverage box child: the interior pass only), one child per kept tear with a side (the rectangles nearest its centre; another
  // child when one is full), rigid with the bone of the piece's kept vertex nearest the group, as the tear slab.
  if(gear&&gear->fits(grid.origin,grid.pitch,grid.size)&&owns.valid()&&!mine.empty()&&drawn[k].valid()&&published.tissue_field&&published.tissue_field->available&&next<gfy_packed::max_child_slots){
   const auto gf0=core::Clock::now();const painter_detail::TissueField&gtf=*published.tissue_field;
   CellBox gbOwn=own[k];if(gbOwn.valid())for(int a=0;a<3;++a){gbOwn.lo[a]-=2;gbOwn.hi[a]+=2;}const CellBox&gbDrawn=drawn[k];
   // the drawn box in tiles (tile aligned by its making)
   const int gbLo[3]={gbDrawn.lo[0]>>3,gbDrawn.lo[1]>>3,gbDrawn.lo[2]>>3},gbN[3]={(gbDrawn.hi[0]-gbDrawn.lo[0]+7)>>3,(gbDrawn.hi[1]-gbDrawn.lo[1]+7)>>3,(gbDrawn.hi[2]-gbDrawn.lo[2]+7)>>3};
   if(gbN[0]>0&&gbN[1]>0&&gbN[2]>0){
    const std::size_t gbCount=std::size_t(gbN[0])*std::size_t(gbN[1])*std::size_t(gbN[2]);
    auto gbIndex=[&](int x,int y,int z)->std::ptrdiff_t{x-=gbLo[0];y-=gbLo[1];z-=gbLo[2];if(x<0||y<0||z<0||x>=gbN[0]||y>=gbN[1]||z>=gbN[2])return -1;
     return std::ptrdiff_t((std::size_t(z)*std::size_t(gbN[1])+std::size_t(y))*std::size_t(gbN[0])+std::size_t(x));};
    static const int gf6[6][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    // by tile (word x & 7, bit (y & 7) * 8 + (z & 7)): the cells the marks hide, the grid's removed cells
    std::vector<const core::Tile*> gfMark(gbCount,nullptr),gfGrid(gbCount,nullptr);
    for(const auto&m:mine){const auto i=gbIndex(m.first.x,m.first.y,m.first.z);if(i>=0)gfMark[std::size_t(i)]=&m.second;}
    for(const auto&t:grid.tiles){const auto i=gbIndex(t.first.x,t.first.y,t.first.z);if(i>=0)gfGrid[std::size_t(i)]=&t.second;}
    // hidden: what the marks hide, not the grid's removed cells (a wound's wall is drawn through its own hole). A blast crater's void
    // in a pack's fill that opens to the cut stays open from the cut's side: its walls as starters measured +44,000 triangles and
    // +6.6 MB on a trunk part (T2). GFY_GF_CRATER_WALLS=1 (probes): the grid's removed cells too
    static const bool gfMarksOnly=std::getenv("GFY_GF_CRATER_WALLS")==nullptr;
    auto gfHid=[&](std::ptrdiff_t i,unsigned q)->std::uint64_t{if(i<0)return 0;const core::Tile*m=gfMark[std::size_t(i)];const core::Tile*g=gfGrid[std::size_t(i)];
     if(gfMarksOnly){if(!m)return 0;return m->removed[q]&~(g?g->removed[q]:std::uint64_t(0));}
     return (m?m->removed[q]:std::uint64_t(0))|(g?g->removed[q]:std::uint64_t(0));};
    auto gfGone=[&](std::ptrdiff_t i,unsigned q)->std::uint64_t{if(i<0)return 0;const core::Tile*m=gfMark[std::size_t(i)];const core::Tile*g=gfGrid[std::size_t(i)];
     return (m?m->removed[q]:std::uint64_t(0))|(g?g->removed[q]:std::uint64_t(0));};
    // the tiles to look at: a tile holding a hidden cell (not all hidden) and the six beside every such tile
    std::vector<std::uint32_t> gfList;std::vector<std::uint8_t> gfSeen(gbCount,0);{std::vector<std::uint8_t>&seen=gfSeen;
     for(std::size_t i=0;i<gbCount;++i){if(!gfMark[i]&&(gfMarksOnly||!gfGrid[i]))continue;bool any=false,full=true;
      for(unsigned q=0;q<8;++q){const std::uint64_t h=gfHid(std::ptrdiff_t(i),q);any=any||h!=0;full=full&&h==~std::uint64_t(0);}
      if(!any)continue;const int x=int(i%std::size_t(gbN[0]))+gbLo[0],y=int((i/std::size_t(gbN[0]))%std::size_t(gbN[1]))+gbLo[1],z=int(i/(std::size_t(gbN[0])*std::size_t(gbN[1])))+gbLo[2];
      if(!full&&!seen[i]){seen[i]=1;gfList.push_back(std::uint32_t(i));}
      for(const auto&f:gf6){const auto j=gbIndex(x+f[0],y+f[1],z+f[2]);if(j<0||seen[std::size_t(j)])continue;seen[std::size_t(j)]=1;gfList.push_back(std::uint32_t(j));}}}
    // GFY_GF_NEAR_CRATERS=1 (probes): near the cut (the tiles holding marks and the six beside each) a crater's or a channel's removed
    // cells count as hidden too (measured on T2: +826 triangles, the blast voids in the pack's fill that open to the cut get bars of
    // wall near the cut and stay open deeper in - not taken)
    static const bool gfNearCraters=std::getenv("GFY_GF_NEAR_CRATERS")!=nullptr;
    auto gfHidN=[&](std::ptrdiff_t i,unsigned q)->std::uint64_t{if(i<0)return 0;const std::uint64_t h=gfHid(i,q);if(!gfNearCraters||!gfSeen[std::size_t(i)])return h;
     const core::Tile*g=gfGrid[std::size_t(i)];return h|(g?g->removed[q]:std::uint64_t(0));};
    // the kept tears' zones (the tear draws there), the tissue field's alignment, the padding tiles
    std::vector<std::pair<tear_atlas::Zone,const gfy_packed::TearBlock*>> gfRegs;
    for(const auto&tr:keep[k])if(tr.block&&!tear_atlas::burstRecord(*tr.block)){const auto z=tear_atlas::zone(*tr.block);if(z.ok)gfRegs.push_back({z,tr.block.get()});}
    int gfOff[3]={0,0,0};bool gfAl=std::abs(double(gtf.pitch)-grid.pitch)<1e-6*grid.pitch;
    for(int a=0;a<3;++a){const double o=(grid.origin[a]-double(gtf.origin[std::size_t(a)]))/grid.pitch;gfOff[a]=int(std::lround(o));gfAl=gfAl&&std::abs(o-gfOff[a])<1e-3;}
    auto gfLab=[&](int x,int y,int z)->unsigned{if(!gfAl)return gtf.sample(grid.origin.x+(x+.5)*grid.pitch,grid.origin.y+(y+.5)*grid.pitch,grid.origin.z+(z+.5)*grid.pitch);
     return gtf.at(x+gfOff[0],y+gfOff[1],z+gfOff[2]);};
    // every cut face of gear outside the kept tears' zones gets starters, also where the coverage box rays could reach it (inside the
    // cells' box + 2 and a tear slab: their rules leave such faces open in places - square holes between a torn section and a pack -
    // and a face both draw lands on the same point). GFY_GF_SKIP_SLABBED=1 (probes): leave those to the coverage box rays
    static const bool gfSkipSlabbed=std::getenv("GFY_GF_SKIP_SLABBED")!=nullptr,gfZoneAll=std::getenv("GFY_GF_ZONE_EVERYWHERE")!=nullptr;
    struct GfSlab {double c[3],u[3],w[3],n[3],hu,hw,hn;};std::vector<GfSlab> gfSlabs;
    for(std::size_t q=0;q<keep[k].size();++q){if(q>=boxFor[k].size()||!boxFor[k][q]||!keep[k][q].block||tear_atlas::burstRecord(*keep[k][q].block))continue;const auto&T=*keep[k][q].block;
     if(!(T.footHi[0]>T.footLo[0]))continue;GfSlab sb;const double ma=.5*(T.footLo[0]+T.footHi[0]),mb=.5*(T.footLo[1]+T.footHi[1]);
     for(int a=0;a<3;++a){sb.c[a]=T.centre[a]+ma*T.u[a]+mb*T.w[a];sb.u[a]=T.u[a];sb.w[a]=T.w[a];sb.n[a]=T.normal[a];}
     sb.hu=.5*(T.footHi[0]-T.footLo[0])+2*T.pitch;sb.hw=.5*(T.footHi[1]-T.footLo[1])+2*T.pitch;sb.hn=T.reach;gfSlabs.push_back(sb);}
    auto gfSlabbed=[&](int x,int y,int z){const double p[3]={grid.origin.x+(x+.5)*grid.pitch,grid.origin.y+(y+.5)*grid.pitch,grid.origin.z+(z+.5)*grid.pitch};
     for(const auto&sb:gfSlabs){const double d[3]={p[0]-sb.c[0],p[1]-sb.c[1],p[2]-sb.c[2]};
      if(std::abs(d[0]*sb.u[0]+d[1]*sb.u[1]+d[2]*sb.u[2])<=sb.hu&&std::abs(d[0]*sb.w[0]+d[1]*sb.w[1]+d[2]*sb.w[2])<=sb.hw&&std::abs(d[0]*sb.n[0]+d[1]*sb.n[1]+d[2]*sb.n[2])<=sb.hn)return true;}
     return false;};
    const auto gfPadF=gear_cells::padTileFlags(published.tissue_field,grid);const std::size_t gfTnx=std::size_t((grid.size.x+7)/8),gfTny=std::size_t((grid.size.y+7)/8);
    const std::uint64_t gfZ0=0x0101010101010101ull,gfZ7=0x8080808080808080ull;
    // the faces: a kept cell of gear fill the piece owns with a hidden neighbour (bit operations per tile, the tests per cell after)
    struct GfFace {std::uint8_t a,s;int p,u,v,hd;};std::vector<std::vector<GfFace>> gfPart(core::parallelWidth()+1);
    const int gfD=16; // the hidden cells counted outward from a face (a rectangle may stand that far out in front of a face)
    lowpoly_pieces::eachBlock(gfList.size(),8,[&](std::size_t first,std::size_t last,std::size_t thread){auto&outF=gfPart[std::min(thread,gfPart.size()-1)];
     for(std::size_t li=first;li<last;++li){const std::size_t i=gfList[li];const int tx=int(i%std::size_t(gbN[0]))+gbLo[0],ty=int((i/std::size_t(gbN[0]))%std::size_t(gbN[1]))+gbLo[1],
       tz=int(i/(std::size_t(gbN[0])*std::size_t(gbN[1])))+gbLo[2];
      const std::size_t pi=(std::size_t(tz)*gfTny+std::size_t(ty))*gfTnx+std::size_t(tx);if(!gfPadF||pi>=gfPadF->size()||!(*gfPadF)[pi])continue; // no padding: no gear fill
      std::ptrdiff_t nbI[6];for(int f=0;f<6;++f)nbI[f]=gbIndex(tx+gf6[f][0],ty+gf6[f][1],tz+gf6[f][2]);
      std::uint64_t K[8],Hs[8],face[6][8];bool anyK=false;
      for(unsigned q=0;q<8;++q){K[q]=~gfGone(std::ptrdiff_t(i),q);Hs[q]=gfHidN(std::ptrdiff_t(i),q);anyK=anyK||K[q]!=0;}
      if(!anyK)continue;
      for(unsigned q=0;q<8;++q){
       face[0][q]=K[q]&(q<7?Hs[q+1]:gfHidN(nbI[0],0));                                   // +x
       face[1][q]=K[q]&(q>0?Hs[q-1]:gfHidN(nbI[1],7));                                   // -x
       face[2][q]=K[q]&((Hs[q]>>8)|((gfHidN(nbI[2],q)&0xffull)<<56));                    // +y
       face[3][q]=K[q]&((Hs[q]<<8)|((gfHidN(nbI[3],q)>>56)&0xffull));                    // -y
       face[4][q]=K[q]&(((Hs[q]>>1)&~gfZ7)|((gfHidN(nbI[4],q)&gfZ0)<<7));                // +z
       face[5][q]=K[q]&(((Hs[q]<<1)&~gfZ0)|((gfHidN(nbI[5],q)&gfZ7)>>7));}               // -z
      for(unsigned q=0;q<8;++q){std::uint64_t any=0;for(int f=0;f<6;++f)any|=face[f][q];
       for(std::uint64_t m=any;m;m&=m-1){unsigned long bit=0;
#ifdef _MSC_VER
        _BitScanForward64(&bit,m);
#else
        bit=unsigned(__builtin_ctzll(m));
#endif
        const int x=tx*8+int(q),y=ty*8+int(bit>>3),z=tz*8+int(bit&7u);
        if(gfSkipSlabbed&&gbOwn.valid()&&x>=gbOwn.lo[0]&&x<gbOwn.hi[0]&&y>=gbOwn.lo[1]&&y<gbOwn.hi[1]&&z>=gbOwn.lo[2]&&z<gbOwn.hi[2]&&gfSlabbed(x,y,z))continue; // (probes: the coverage box rays' own)
        if((gfLab(x,y,z)&127u)!=80u)continue; // gear fill only (garment air under padding: what the starter search draws)
        if(owns.owner(x,y,z)!=std::uint8_t(k))continue; // fill this piece owns
        if(gfZoneAll||(gbOwn.valid()&&x>=gbOwn.lo[0]-2&&x<gbOwn.hi[0]+2&&y>=gbOwn.lo[1]-2&&y<gbOwn.hi[1]+2&&z>=gbOwn.lo[2]-2&&z<gbOwn.hi[2]+2)){ // (the zones count inside the cells' box only, as the marks)
         const double pq[3]={grid.origin.x+(x+.5)*grid.pitch,grid.origin.y+(y+.5)*grid.pitch,grid.origin.z+(z+.5)*grid.pitch};bool inZ=false;
         for(const auto&r:gfRegs)if(tear_atlas::inZone(r.first,*r.second,pq,1.5*grid.pitch)){inZ=true;break;}if(inZ)continue;}
        const int cc[3]={x,y,z};
        for(int f=0;f<6;++f)if((face[f][q]>>bit)&1u){const int a=f>>1;const bool pos=(f&1)==0;
         // hd: the hidden cells (marks or the grid's) outward from the face, up to gfD
         int hd=0;for(int s2=1;s2<=gfD;++s2){const int nx=x+gf6[f][0]*s2,ny=y+gf6[f][1]*s2,nz=z+gf6[f][2]*s2;const auto j=gbIndex(nx>>3,ny>>3,nz>>3);
          if(j<0||!((gfGone(j,unsigned(nx&7))>>((ny&7)*8+(nz&7)))&1u))break;++hd;}
         outF.push_back(GfFace{std::uint8_t(a),std::uint8_t(pos?1:0),cc[a]+(pos?1:0),cc[(a+1)%3],cc[(a+2)%3],std::max(hd,1)});}}}}});
    std::vector<GfFace> gfFaces;{std::size_t n=0;for(const auto&pp:gfPart)n+=pp.size();gfFaces.reserve(n);for(const auto&pp:gfPart)gfFaces.insert(gfFaces.end(),pp.begin(),pp.end());}
    stats.gfFaces+=gfFaces.size();
    // greedy rectangles per facing (axis, sign) over the faces' (u, v), ACROSS DEPTHS: a rectangle stands in the plane of its
    // outermost face (a quarter cell out) and may take faces deeper in where every cell between that plane and the face is hidden
    // (marks or the grid's: a starter pixel's ray then crosses only hidden cells to its face - a staircase cut through a pack is a
    // few rectangles, not one a step). Per (u, v) the outermost face counts (a face behind it lies behind its fill). In q = p
    // (facing +) or -p (facing -): a face's cap = q + hidden - 1; a rectangle holds faces while max q <= min cap.
    std::sort(gfFaces.begin(),gfFaces.end(),[](const GfFace&l,const GfFace&r){return l.a!=r.a?l.a<r.a:(l.s!=r.s?l.s<r.s:(l.v!=r.v?l.v<r.v:(l.u!=r.u?l.u<r.u:(l.s?l.p>r.p:l.p<r.p))));});
    struct GfRect {std::uint8_t a,s;int p,u0,v0,u1,v1;};std::vector<GfRect> gfRects;struct GfCell {int q,cap;std::uint8_t st;};std::vector<GfCell> bm;
    for(std::size_t g0=0;g0<gfFaces.size();){std::size_t g1=g0;while(g1<gfFaces.size()&&gfFaces[g1].a==gfFaces[g0].a&&gfFaces[g1].s==gfFaces[g0].s)++g1;
     const bool sp=gfFaces[g0].s!=0;
     int u0=INT_MAX,u1=INT_MIN,v0=INT_MAX,v1=INT_MIN;for(std::size_t i=g0;i<g1;++i){u0=std::min(u0,gfFaces[i].u);u1=std::max(u1,gfFaces[i].u);v0=std::min(v0,gfFaces[i].v);v1=std::max(v1,gfFaces[i].v);}
     const int W=u1-u0+1,Hh=v1-v0+1;bm.assign(std::size_t(W)*std::size_t(Hh),GfCell{0,0,0});
     for(std::size_t i=g0;i<g1;++i){auto&e=bm[std::size_t(gfFaces[i].v-v0)*std::size_t(W)+std::size_t(gfFaces[i].u-u0)];if(e.st)continue; // (sorted: the outermost first)
      const int q=sp?gfFaces[i].p:-gfFaces[i].p;e=GfCell{q,q+gfFaces[i].hd-1,1};}
     auto at=[&](int u,int v)->GfCell&{return bm[std::size_t(v)*std::size_t(W)+std::size_t(u)];};
     for(int v=0;v<Hh;++v)for(int u=0;u<W;++u){if(at(u,v).st!=1)continue;int mq=at(u,v).q,mc=at(u,v).cap,w=1;
      while(u+w<W){const GfCell&e=at(u+w,v);if(e.st!=1||std::max(mq,e.q)>std::min(mc,e.cap))break;mq=std::max(mq,e.q);mc=std::min(mc,e.cap);++w;}
      int h=1;for(;v+h<Hh;++h){int rq=mq,rc=mc;bool row=true;for(int c2=0;c2<w&&row;++c2){const GfCell&e=at(u+c2,v+h);row=e.st==1;if(row){rq=std::max(rq,e.q);rc=std::min(rc,e.cap);}}
       if(!row||rq>rc)break;mq=rq;mc=rc;}
      for(int r=0;r<h;++r)for(int c2=0;c2<w;++c2)at(u+c2,v+r).st=2;
      gfRects.push_back(GfRect{gfFaces[g0].a,gfFaces[g0].s,sp?mq:-mq,u0+u,v0+v,u0+u+w,v0+v+h});}
     g0=g1;}
    stats.gfRects+=gfRects.size();
    // one child per kept tear with a side (the rectangles nearest its centre)
    std::vector<const Region*> gbTears;
    for(const auto&gbRef:keep[k]){if(!gbRef.block||gbRef.side>1||tear_atlas::burstRecord(*gbRef.block))continue;for(const auto&Rg:R)if(Rg.T==gbRef.block.get()){gbTears.push_back(&Rg);break;}}
    auto gfCentre=[&](const GfRect&r,double*c){double q[3];q[r.a]=r.p;q[(r.a+1)%3]=.5*(r.u0+r.u1);q[(r.a+2)%3]=.5*(r.v0+r.v1);
     c[0]=grid.origin.x+q[0]*grid.pitch;c[1]=grid.origin.y+q[1]*grid.pitch;c[2]=grid.origin.z+q[2]*grid.pitch;};
    std::vector<std::vector<std::uint32_t>> gbGroups(gbTears.empty()?std::size_t(1):gbTears.size());
    for(std::size_t ri=0;ri<gfRects.size();++ri){std::size_t g=0;double best=1e300,cc[3];gfCentre(gfRects[ri],cc);
     for(std::size_t i=0;i<gbTears.size();++i){const double dx=cc[0]-gbTears[i]->C[0],dy=cc[1]-gbTears[i]->C[1],dz=cc[2]-gbTears[i]->C[2],dd=dx*dx+dy*dy+dz*dz;if(dd<best){best=dd;g=i;}}
     gbGroups[g].push_back(std::uint32_t(ri));}
    for(const auto&list:gbGroups){if(list.empty()||next>=gfy_packed::max_child_slots)continue;
     double sx=0,sy=0,sz=0;for(const auto ri:list){double cc[3];gfCentre(gfRects[ri],cc);sx+=cc[0];sy+=cc[1];sz+=cc[2];}
     const Vec gbCentre{sx/double(list.size()),sy/double(list.size()),sz/double(list.size())};double bestD=1e300;std::uint16_t gbBone=65535,gbMaterialSource=0;
     for(const auto&ch:mesh.children)for(const auto&row:ch.vertices){if(row.surface[12]<=-999.f)continue;unsigned bw=0;for(unsigned w=1;w<4;++w)if(row.weight[w]>row.weight[bw])bw=w;
      const double dd=length(position(row)-gbCentre);if(dd<bestD&&row.bone[bw]<64&&ok[k][row.bone[bw]]){bestD=dd;gbBone=row.bone[bw];gbMaterialSource=ch.material_source_child;}}
     if(gbBone==65535)continue;
     for(std::size_t r0=0;r0<list.size()&&next<gfy_packed::max_child_slots;){
      gfy_packed::Child child;child.material=0;child.material_source_child=gbMaterialSource;child.skin_clip=true;child.child_index=std::uint16_t(next);
      std::size_t r1=r0;
      for(;r1<list.size()&&child.vertices.size()+4<=65532;++r1){const GfRect&r=gfRects[list[r1]];const int a=r.a,b=(a+1)%3,c2=(a+2)%3;const bool outward=r.s!=0;
       // a quarter cell out on the hidden side, a quarter cell wider on each side
       const double P=r.p+(outward?.25:-.25),B0=r.u0-.25,B1=r.u1+.25,C0=r.v0-.25,C1=r.v1+.25;
       double corner[4][3];for(int i=0;i<4;++i){corner[i][a]=P;corner[i][b]=(i==1||i==2)?B1:B0;corner[i][c2]=(i>=2)?C1:C0;}
       double nrm[3]={0,0,0},tg[3]={0,0,0},bn[3]={0,0,0};nrm[a]=outward?1.:-1.;tg[b]=1.;bn[c2]=outward?1.:-1.;
       std::uint16_t id[4];
       for(int i=0;i<4;++i){gfy_packed::Vertex row;
        const double sv[14]={grid.origin.x+corner[i][0]*grid.pitch,grid.origin.y+corner[i][1]*grid.pitch,grid.origin.z+corner[i][2]*grid.pitch,nrm[0],nrm[1],nrm[2],tg[0],tg[1],tg[2],bn[0],bn[1],bn[2],-2000.,0.};
        for(unsigned q=0;q<14;++q)row.surface[q]=float(sv[q]);
        row.bone={gbBone,gbBone,gbBone,gbBone};row.weight={1.f,0.f,0.f,0.f};gfy_packed::canonical_weights(row);
        id[i]=std::uint16_t(child.vertices.size());child.vertices.push_back(row);}
       // (p0, p1, p2) turns about +axis a: front faces toward the hidden side
       const std::uint16_t tri[6]={id[0],outward==ccwOutward?id[1]:id[2],outward==ccwOutward?id[2]:id[1],id[0],outward==ccwOutward?id[2]:id[3],outward==ccwOutward?id[3]:id[2]};
       for(const auto v:tri)child.indices.push_back(v);child.original_faces.push_back(~std::uint32_t(0));child.original_faces.push_back(~std::uint32_t(0));}
      r0=r1;if(child.indices.empty())break;
      ++next;stats.boxTriangles+=child.indices.size()/3;stats.gfTris+=child.indices.size()/3;++stats.boxes;++stats.gfChildren;mesh.children.push_back(std::move(child));}}}
   stats.gfMs+=core::ms(gf0);}
  // gearback: the exterior rays' box (row 0 texels 4, 5) = the box of everything the piece draws - its cells AND its exterior
  // triangles - 2 cells inside the box its field hides the others in (the programs pad it by 2): gear standing outside the tissue
  // box (a pack, pouches, a holster, a hat) is filled in the volume (the garment fill, padding bit 64) but was never searched,
  // so a hidden gear pixel showed the world through the hole. A coverage box pixel's ray keeps the cells' box.
  const tear_atlas::Box exteriorBox=drawn[k].valid()?worldBox(grid,drawn[k],-2):tear_atlas::Box();
  mesh.tear_atlas=tear_atlas::build(mesh.tears,worldBox(grid,own[k],0),worldBox(grid,drawn[k],0),&stats.atlas,&th,&exteriorBox);}
 // gearfine (probes, GFY_GF_CHECK): near every barrier (|s - offset| <= 4 cm, within its reach), the gear / padding cells hidden on
 // every piece that draws there (its drawn box) by the marks (not the grid's removed cells), and those owned by no piece
 if(gfCheck){std::size_t hidAll=0,noOwner=0,looked=0,hidBy[4]={0,0,0,0},hidD[4]={0,0,0,0};const auto padC=published.tissue_field&&published.tissue_field->available?gear_cells::padTileFlags(published.tissue_field,grid):std::shared_ptr<const std::vector<std::uint8_t>>();
  const painter_detail::TissueField*ctf=published.tissue_field&&published.tissue_field->available?published.tissue_field.get():nullptr;
  for(const auto t:freshIdx){const Region&Rg=R[t];if(!(Rg.Rl>0))continue;const double rb=Rg.Rl+.25;
   int lo[3],hi[3];for(int a=0;a<3;++a){const double e=rb*std::sqrt(Rg.u[a]*Rg.u[a]+Rg.w[a]*Rg.w[a])+.05*std::abs(Rg.n[a]);
    lo[a]=std::max(0,int(std::floor((Rg.C[a]-e-grid.origin[a])/grid.pitch)));hi[a]=std::min(a==0?grid.size.x-1:(a==1?grid.size.y-1:grid.size.z-1),int(std::floor((Rg.C[a]+e-grid.origin[a])/grid.pitch)));}
   for(int tz=lo[2]>>3;tz<=hi[2]>>3;++tz)for(int ty=lo[1]>>3;ty<=hi[1]>>3;++ty)for(int tx=lo[0]>>3;tx<=hi[0]>>3;++tx){const Cell tc{tx,ty,tz};
    const std::size_t pi=(std::size_t(tc.z)*std::size_t((grid.size.y+7)/8)+std::size_t(tc.y))*std::size_t((grid.size.x+7)/8)+std::size_t(tc.x);
    const core::Tile*gt=gear?gear->tile(tc.x,tc.y,tc.z):nullptr;const bool padT=padC&&pi<padC->size()&&(*padC)[pi];if(!gt&&!padT)continue;
   for(int z=std::max(lo[2],tz*8);z<=std::min(hi[2],tz*8+7);++z)for(int y=std::max(lo[1],ty*8);y<=std::min(hi[1],ty*8+7);++y)for(int x=std::max(lo[0],tx*8);x<=std::min(hi[0],tx*8+7);++x){
    const bool g=gt&&((gt->removed[std::size_t(x&7)]>>((y&7)*8+(z&7)))&1u);
    bool pd=false;if(!g&&ctf&&padT)pd=(ctf->sample(grid.origin.x+(x+.5)*grid.pitch,grid.origin.y+(y+.5)*grid.pitch,grid.origin.z+(z+.5)*grid.pitch)&64u)!=0;
    if(!g&&!pd)continue;
    const double px=grid.origin.x+(x+.5)*grid.pitch-Rg.C[0],py=grid.origin.y+(y+.5)*grid.pitch-Rg.C[1],pz=grid.origin.z+(z+.5)*grid.pitch-Rg.C[2];
    const double pa=px*Rg.u[0]+py*Rg.u[1]+pz*Rg.u[2],pb=px*Rg.w[0]+py*Rg.w[1]+pz*Rg.w[2];if(pa*pa+pb*pb>rb*rb)continue;
    const double vS=px*Rg.n[0]+py*Rg.n[1]+pz*Rg.n[2]-Rg.offset(pa,pb);if(std::abs(vS)>.04)continue;
    {const auto gr=grid.tiles.find(tc);if(gr!=grid.tiles.end()&&((gr->second.removed[std::size_t(x&7)]>>((y&7)*8+(z&7)))&1u))continue;}
    ++looked;bool all=true;std::size_t drawers=0;
    auto hidIn=[&](const std::vector<std::unordered_map<Cell,core::Tile,core::CellHash>>&v,std::size_t k){const auto it=v[k].find(tc);
     return it!=v[k].end()&&((it->second.removed[std::size_t(x&7)]>>((y&7)*8+(z&7)))&1u)!=0;};
    for(std::size_t k=0;k<P&&all;++k){if(!drawn[k].valid()||!out[k].triangles)continue;const CellBox&d=drawn[k];if(x<d.lo[0]||x>=d.hi[0]||y<d.lo[1]||y>=d.hi[1]||z<d.lo[2]||z>=d.hi[2])continue;++drawers;
     all=hidIn(gfChk,k);}
    const std::uint8_t ow=owns.valid()?owns.owner(x,y,z):std::uint8_t(255);
    if(all&&drawers){++hidAll;const double av=std::abs(vS);++hidD[av<.002?0:(av<.006?1:(av<.02?2:3))];
     if(ow>=P)++hidBy[3];else if(hidIn(gfChkO,ow))++hidBy[0];else if(hidIn(gfChkR,ow))++hidBy[1];else ++hidBy[2];}
    if(ow==255)++noOwner;}}}
  std::fprintf(stderr,"gearfine_check pieces=%zu looked=%zu hidden_on_every_piece=%zu (on its owner by: owners %zu ring %zu scrap/inherited %zu, no owner %zu; |s-offset| <2mm %zu <6mm %zu <20mm %zu more %zu) owned_by_none=%zu\n",
   P,looked,hidAll,hidBy[0],hidBy[1],hidBy[2],hidBy[3],hidD[0],hidD[1],hidD[2],hidD[3],noOwner);}
 if(!empty.empty())throw lowpoly_pieces::EmptyPieces(std::move(empty));
 stats.packMs=core::ms(p0);stats.ms=core::ms(t0);
 static const bool log=std::getenv("GFY_M3_LOG")!=nullptr||std::getenv("GFY_PIECE_LOG")!=nullptr;
 if(log)std::fprintf(stderr,"boolean_remove tear pieces=%zu blocks=%zu skipped=%zu gone_blocks=%zu ms=%.3f islands=%zu medium=%zu shell=%zu pad_tiles=%zu island_ms=%.3f\n",hmStats.pieces,hmStats.marked,hmStats.skipped,hmGone.size(),hmStats.ms,
   hmStats.islands,hmStats.medium,hmStats.shell,hmStats.padTiles,hmStats.islandMs); // headmask2
 if(log)std::fprintf(stderr,"tear_pieces_ring ok=%d rings=%zu ring_cells=%zu near_ms=%.2f prep_ms=%.2f (bake wait %.2f) marks_ms=%.2f\n",hbNear.ok?1:0,hbRings.size(),hbRingCells,hbNear.ms,hbPrepMs,hbWaitMs,hbMarkMs); // headfix
 if(log)std::fprintf(stderr,"tear_pieces_cutfix pieces=%zu ghost_tris=%zu ghost_tiles=%zu mark_tiles=%zu marks_ms=%.2f pruned_tears=%zu pruned_boxes=%zu atlas_sets=%zu atlas_tears=%zu atlas_kb=%zu atlas_ms=%.2f index_cells=%zu list_entries=%zu max_list=%zu field_kb=%zu own_tiles=%zu gear_cells=%zu pad_cells=%zu owned=%zu seeded=%zu own_marks=%zu own_ms=%.2f outfit_gear_cells=%zu outfit_gear_tiles=%zu outfit_gear_kb=%zu outfit_gear_build_ms=%.1f\n",
  P,stats.ghosts,stats.ghostTiles,stats.markTiles,stats.marksMs,stats.prunedTears,stats.prunedBoxes,stats.atlas.sets,stats.atlas.tears,stats.atlas.bytes/1024,stats.atlas.ms,
  stats.atlas.indexCells,stats.atlas.listEntries,stats.atlas.maxList,stats.fieldBytes/1024,stats.padTiles,stats.gearCells,stats.padCells,stats.ownedCells,stats.seededCells,stats.padMarks,stats.padMs,
  gear?gear->cells:std::size_t(0),gear?gear->tiles.size():std::size_t(0),gear?gear->bytes/1024:std::size_t(0),gear?gear->ms:0.);
 if(log)std::fprintf(stderr,"tear_pieces_gearfine bars=%zu ext_tiles=%zu left_tiles=%zu left_cells=%zu starter_faces=%zu starter_rects=%zu starter_children=%zu starter_tris=%zu starter_ms=%.2f own_ms=%.2f\n",
  stats.gfBars,stats.gfExtTiles,stats.gfLeftTiles,stats.gfLeftCells,stats.gfFaces,stats.gfRects,stats.gfChildren,stats.gfTris,stats.gfMs,stats.padMs); // gearfine
 return out;
}
} // namespace mask_pieces3
