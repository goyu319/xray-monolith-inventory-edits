// Mask-only wounds, value 3: the pieces of a separation whose fresh tears are head burst records only (headfix, 2026-11-12).
// Included by separation/MaskPieces3.h inside namespace mask_pieces3 (after PieceAt and Region; mask_pieces3::build calls build()
// below first thing when only() holds). No standard header here (the including file has them).
// What each piece draws:
//  * its copy of every record with the groups it holds (header word 15): a group belongs to the pieces holding >= 2 of the record's
//    sample points of that group; the pixel programs then keep exactly that part of the head (painter_vmask_head.h);
//  * the ORIGINAL exterior triangles, by the record's own function: a sample point (corners, centre, edge midpoints) inside a
//    record's cut belongs to the pieces holding its group; several (one group in two pieces: a far chunk the fragments cut off)
//    or a point outside every cut: the piece whose bones carry most of the triangle's skin weight, a tie the piece whose volume
//    lies under it (as MaskPieces3); a triangle goes to every piece one of its samples gives it (a crack never leaves a strip of
//    skin to nobody); posing rules as MaskPieces3;
//  * one coverage box per piece and record: the axis box of its part of the burst (its samples; the far part: the whole broken
//    region), 2-3 cm out, skinned to one bone of the piece;
//  * its removal field: the grid's removed cells + the inherited marks + the cells of every OTHER piece that holds one of its
//    groups (one group in two pieces: each hides the other's cells, at cell size).
// Inherited (not fresh) decide-once tears: kept as MaskPieces3 keeps them (voted, or a triangle of the piece in its region).
namespace head_burst_pieces {
struct FlightPass {
 using Surfaces=std::vector<bound_surface::Piece>;
 using Marks=std::vector<std::vector<std::pair<Cell,core::Tile>>>;
 using Ids=std::vector<std::uint32_t>;
 std::set<std::uint32_t> fitted;
 const Ids* ids=nullptr;
 const Surfaces* surfaces=nullptr;
 const Marks* marks=nullptr;
 bool consumed=false;
 std::function<void()> admit;
 static FlightPass*& current(){static thread_local FlightPass* pass=nullptr;return pass;}
};

inline unsigned lowBit64(std::uint64_t b){
#ifdef _MSC_VER
 unsigned long i=0;_BitScanForward64(&i,b);return unsigned(i);
#else
 return unsigned(__builtin_ctzll(b));
#endif
}
#include "ChunkHaloFrontier.h"
inline unsigned highBit64(std::uint64_t b){ // blockfine
#ifdef _MSC_VER
 unsigned long i=0;_BitScanReverse64(&i,b);return unsigned(i);
#else
 return 63u-unsigned(__builtin_clzll(b));
#endif
}
// A fragment far smaller than the burst's largest (under a quarter of it, at most 50 cm3: a sliver a shot's cavity or a crack cut
// off a group) is blown away with the cracks - a scrap of the separation (LiveSeparationWorker_impl.h prepare), never a ragged sheet
// flying off. The body (the largest component) and the fragments of normal size are untouched; keep[id] (a detached eye) stays.
inline void slivers(const gfy_packed::Mesh*published,const Grid&grid,const gfy_connectivity::Topology&v,const std::vector<char>&keep,std::vector<char>&scrap){
 if(!published||scrap.size()<v.components.size()||v.components.size()<3)return;
 for(const auto&tr:published->tears){if(!fresh(tr)||!head_burst_record::is(*tr.block))continue;head_burst_record::Function f;if(!head_burst_record::decode(*tr.block,f))continue;
  std::vector<char> in(v.components.size(),0);int lo[2],hi[2];
  for(int a=0;a<2;++a){lo[a]=int(std::floor((f.B[a]-f.Rcut-grid.origin[a])/grid.pitch));hi[a]=int(std::floor((f.B[a]+f.Rcut-grid.origin[a])/grid.pitch));}
  for(const auto&col:v.columns){if(col.x<lo[0]||col.x>hi[0]||col.y<lo[1]||col.y>hi[1])continue;
   for(auto s=col.first;s<col.first+col.count;++s){const auto&sp=v.spans[s];if(!sp.component||sp.component>=in.size()||in[sp.component])continue;
    const Vec q=grid.center(Cell{col.x,col.y,sp.lo});const double dx=q.x-f.B[0],dy=q.y-f.B[1],dz=q.z-f.B[2];if(dx*dx+dy*dy+dz*dz<=f.Rcut*f.Rcut)in[sp.component]=1;}}
  std::uint32_t body=1;for(std::uint32_t id=1;id<v.components.size();++id)if(v.components[id].cells>v.components[body].cells)body=id;
  std::uint64_t big=0;for(std::uint32_t id=1;id<v.components.size();++id)if(in[id]&&id!=body)big=std::max<std::uint64_t>(big,v.components[id].cells);
  const bool bodyRec=tr.block->header.size()>14&&tr.block->header[14]==2u; // a blast's body burst: chunks of a torso, 150 cm3
  const std::uint64_t lim=std::min<std::uint64_t>(std::uint64_t((bodyRec?150e-6:50e-6)/(grid.pitch*grid.pitch*grid.pitch)),big/4);
  for(std::uint32_t id=1;id<v.components.size();++id)if(in[id]&&id!=body&&v.components[id].cells<lim&&!(id<keep.size()&&keep[id]))scrap[id]=1;}}
inline void chunks(const gfy_packed::Mesh*published,const Grid&grid,const gfy_connectivity::Topology&v,const std::vector<char>&small,std::vector<char>&chunk){
 chunk.assign(v.components.size(),0);if(!published||small.size()<v.components.size())return;
 bool any=false;for(const auto s:small)any=any||s!=0;if(!any)return;
 for(const auto&tr:published->tears){if(!fresh(tr)||!head_burst_record::is(*tr.block))continue;
  if(tr.block->header.size()>14&&tr.block->header[14]==2u)continue;
  head_burst_record::Function f;if(!head_burst_record::decode(*tr.block,f))continue;int lo[2],hi[2];
  for(int a=0;a<2;++a){lo[a]=int(std::floor((f.B[a]-f.Rcut-grid.origin[a])/grid.pitch));hi[a]=int(std::floor((f.B[a]+f.Rcut-grid.origin[a])/grid.pitch));}
  for(const auto&col:v.columns){if(col.x<lo[0]||col.x>hi[0]||col.y<lo[1]||col.y>hi[1])continue;
   for(auto s=col.first;s<col.first+col.count;++s){const auto&sp=v.spans[s];if(!sp.component||sp.component>=chunk.size()||!small[sp.component]||chunk[sp.component])continue;
    const Vec q=grid.center(Cell{col.x,col.y,sp.lo});if(f.label(q.x,q.y,q.z)!=0)chunk[sp.component]=1;}}}}
inline bool only(const gfy_packed::Mesh&published){bool burst=false,other=false;
 for(const auto&t:published.tears)if(fresh(t))(head_burst_record::is(*t.block)?burst:other)=true;return burst&&!other;}

struct NoZone {static constexpr bool never=true;bool operator()(const double*)const{return false;}};
template<class Z,class=void> struct ZoneNever {static constexpr bool value=false;};
template<class Z> struct ZoneNever<Z,decltype(void(Z::never))> {static constexpr bool value=Z::never;};
template<class Z> constexpr bool noZone(){return ZoneNever<typename std::decay<Z>::type>::value;}
struct AttachStats {std::size_t pieces=0,marked=0,skipped=0,gone=0,islands=0,medium=0,shell=0,padTiles=0;double ms=0,islandMs=0;};
using PadCache=std::unordered_map<Cell,core::Tile,core::CellHash>; // a tile's clothing padding cells, sampled once per separation
// the cells under piece k's exterior triangles inside a box (one sample a cell pitch; a coverage box's faces are no surface).
// Asked for only where the outfit has no gear cells: their band holds every triangle's surface (measured on 4 outfits: the triangles
// added 0 to 19 cells to a medium of 35,000 to 600,000 and no island) and the samples cost more than the whole fill.
template<class Children,class Sel>
inline std::vector<Cell> shellCells(const Grid&grid,const Children&children,const Sel&sel,std::size_t k,const CellBox&box){std::vector<Cell> out;if(!box.valid())return out;
 const double p=grid.pitch;const double lo[3]={grid.origin.x+box.lo[0]*p,grid.origin.y+box.lo[1]*p,grid.origin.z+box.lo[2]*p},hi[3]={grid.origin.x+box.hi[0]*p,grid.origin.y+box.hi[1]*p,grid.origin.z+box.hi[2]*p};
 for(std::size_t c=0;c<children.size();++c){if(sel[c].tris.size()<=k)continue;const auto&src=children[c];
  for(const auto e:sel[c].tris[k]){const std::size_t t=std::size_t(e)&0x7fffffffu;if(3*t+2>=src.indices.size())continue;const auto&v0=src.vertices[src.indices[3*t]];if(v0.surface[12]<=-999.f)continue;
   const auto&v1=src.vertices[src.indices[3*t+1]];const auto&v2=src.vertices[src.indices[3*t+2]];
   const Vec A{v0.surface[0],v0.surface[1],v0.surface[2]},B{v1.surface[0],v1.surface[1],v1.surface[2]},C{v2.surface[0],v2.surface[1],v2.surface[2]};
   if(std::max(A.x,std::max(B.x,C.x))<lo[0]||std::min(A.x,std::min(B.x,C.x))>=hi[0]||std::max(A.y,std::max(B.y,C.y))<lo[1]||std::min(A.y,std::min(B.y,C.y))>=hi[1]
     ||std::max(A.z,std::max(B.z,C.z))<lo[2]||std::min(A.z,std::min(B.z,C.z))>=hi[2])continue;
   const double L=std::max(length(B-A),std::max(length(C-B),length(A-C)));const int n=std::max(1,std::min(512,int(std::ceil(L/p))));Cell last{INT_MIN,INT_MIN,INT_MIN};
   for(int i=0;i<=n;++i)for(int j=0;i+j<=n;++j){const Vec q=A+(B-A)*(double(i)/n)+(C-A)*(double(j)/n);
    const Cell cc{int(std::floor((q.x-grid.origin.x)/p)),int(std::floor((q.y-grid.origin.y)/p)),int(std::floor((q.z-grid.origin.z)/p))};
    if(cc.x<box.lo[0]||cc.x>=box.hi[0]||cc.y<box.lo[1]||cc.y>=box.hi[1]||cc.z<box.lo[2]||cc.z>=box.hi[2]||cc==last)continue;last=cc;out.push_back(cc);}}}
 return out;}
// the box of a cut: every piece's own cells' box but the largest's, 16 cells out (the cut borders the smaller pieces)
inline CellBox cutBox(const std::vector<CellBox>&own,std::size_t largest){CellBox b;
 for(std::size_t j=0;j<own.size();++j)if(j!=largest&&own[j].valid()){CellBox q=own[j];for(int a=0;a<3;++a){q.lo[a]-=16;q.hi[a]+=16;}b.add(q);}
 return b;}
// the same boxes apart: boxes that meet are one (a head's fragments); a forearm and a calf torn by one blast stay two
inline void addCluster(std::vector<CellBox>&out,CellBox q){if(!q.valid())return;
 for(bool merged=true;merged;){merged=false;for(std::size_t i=0;i<out.size();++i){const CellBox o=out[i];
   if(o.lo[0]<q.hi[0]&&q.lo[0]<o.hi[0]&&o.lo[1]<q.hi[1]&&q.lo[1]<o.hi[1]&&o.lo[2]<q.hi[2]&&q.lo[2]<o.hi[2]){q.add(o);out.erase(out.begin()+std::ptrdiff_t(i));merged=true;break;}}}
 out.push_back(q);}
inline std::vector<CellBox> cutBoxes(const std::vector<CellBox>&own,std::size_t largest){std::vector<CellBox> out;
 for(std::size_t j=0;j<own.size();++j){if(j==largest||!own[j].valid())continue;CellBox q=own[j];for(int a=0;a<3;++a){q.lo[a]-=16;q.hi[a]+=16;}addCluster(out,q);}
 return out;}
// the box of a piece's own pass: its cells' box, grown where the caller says so (a severed limb, the body: 0 - grown 16 cells it
// was 3 x the marks and half the time more for 2 of 216 loose samples); the largest piece's: a cluster of the cut's boxes inside that
inline CellBox attachBox(CellBox b,int grow=0){if(b.valid())for(int a=0;a<3;++a){b.lo[a]-=grow;b.hi[a]+=grow;}return b;}
inline CellBox attachClip(CellBox b,const CellBox&own,int grow=0){const CellBox o=attachBox(own,grow);for(int a=0;a<3;++a){b.lo[a]=std::max(b.lo[a],o.lo[a]);b.hi[a]=std::min(b.hi[a],o.hi[a]);}return b;}
// the blocks (cell of a block's low corner) where this cut or an earlier wound removed anatomy (8 or more of 64 cells gone, tissue in
// the model at the block's centre), inside the box: once per separation
inline std::vector<Cell> goneBlocks(const Grid&grid,const painter_detail::TissueField*tf,const CellBox&box){std::vector<Cell> out;if(!tf||!box.valid())return out;
 // blockfine: every such CELL instead (removed in the grid, anatomy in the model at its centre), the box 64 cells out
 static const bool blockRule=std::getenv("GFY_HB_BLOCK_RULE")!=nullptr;
 if(!blockRule){const auto g0=core::Clock::now();CellBox b=box;for(int a=0;a<3;++a){b.lo[a]-=64;b.hi[a]+=64;}
  std::vector<const std::pair<const Cell,core::Tile>*> ts;
  for(const auto&gt:grid.tiles){const Cell&tk=gt.first;if(tk.x*8+8<=b.lo[0]||tk.x*8>=b.hi[0]||tk.y*8+8<=b.lo[1]||tk.y*8>=b.hi[1]||tk.z*8+8<=b.lo[2]||tk.z*8>=b.hi[2])continue;ts.push_back(&gt);}
  // (a tile's cells block by block: a block's cells stay together in the list, which the passes read once a block; tiles on the workers)
  std::vector<std::vector<Cell>> part(core::parallelWidth()+1);
  lowpoly_pieces::eachBlock(ts.size(),16,[&](std::size_t first,std::size_t last,std::size_t chunk){auto&o=part[std::min(chunk,part.size()-1)];
   for(std::size_t i=first;i<last;++i){const Cell&tk=ts[i]->first;const core::Tile&t=ts[i]->second;
    for(int q=0;q<8;++q){const int ox=(q&1)*4,oy=((q>>1)&1)*4,oz=(q>>2)*4;std::uint64_t mq=0;for(int ly=0;ly<4;++ly)mq|=std::uint64_t(0xF)<<((oy+ly)*8+oz);
     for(int lx=0;lx<4;++lx)for(std::uint64_t m=t.removed[std::size_t(ox+lx)]&mq;m;m&=m-1){const unsigned bq=lowBit64(m);const int x=tk.x*8+ox+lx,y=tk.y*8+int(bq>>3),z=tk.z*8+int(bq&7u);
      const unsigned an=tf->sample(grid.origin.x+(x+.5)*grid.pitch,grid.origin.y+(y+.5)*grid.pitch,grid.origin.z+(z+.5)*grid.pitch);
      if(!(an&64u)&&(an&63u)!=0&&(an&63u)!=16)o.push_back(Cell{x,y,z});}}}});
  for(const auto&o:part)out.insert(out.end(),o.begin(),o.end());
  if(std::getenv("GFY_M3_LOG")||std::getenv("GFY_PIECE_LOG"))std::cerr<<"blockfine gone cells="<<out.size()<<" ms="<<core::ms(g0)<<std::endl;
  return out;}
 for(const auto&gt:grid.tiles){const Cell&tk=gt.first;
  if(tk.x*8+8<=box.lo[0]||tk.x*8>=box.hi[0]||tk.y*8+8<=box.lo[1]||tk.y*8>=box.hi[1]||tk.z*8+8<=box.lo[2]||tk.z*8>=box.hi[2])continue;
  for(int q=0;q<8;++q){const int ox=(q&1)*4,oy=((q>>1)&1)*4,oz=(q>>2)*4;std::uint64_t m=0;for(int ly=0;ly<4;++ly)m|=std::uint64_t(0xF)<<((oy+ly)*8+oz);
   int n=0;for(int lx=0;lx<4;++lx)for(std::uint64_t v=gt.second.removed[std::size_t(ox+lx)]&m;v;v&=v-1)++n;if(n<8)continue;
   const unsigned an=tf->sample(grid.origin.x+(tk.x*8+ox+2)*grid.pitch,grid.origin.y+(tk.y*8+oy+2)*grid.pitch,grid.origin.z+(tk.z*8+oz+2)*grid.pitch);
   if(!(an&64u)&&(an&63u)!=0&&(an&63u)!=16)out.push_back(Cell{tk.x*8+ox,tk.y*8+oy,tk.z*8+oz});}}
 return out;}
// piece k's pass over `box` (cells). pieceOf: the volume's component -> piece (-1 none). inZone(world point of a block's centre): the
// block is a kept tear's to decide (left alone). Marks: tile -> bits, OR-ed. shell: shellCells of the piece in the box. inherited:
// the marks the piece's parent left it (tile, bits), merged into its field by the caller.
template<class Marks,class InZone,class Inh>
inline void unattached(const Grid&grid,const gfy_connectivity::Topology&volume,const std::vector<int>&pieceOf,std::size_t k,CellBox box,
    const gear_cells::GearCells*gear,const std::vector<std::uint8_t>*padF,const painter_detail::TissueField*tf,const std::vector<Cell>&gone,Marks&marks,InZone&&inZone,AttachStats&st,
    const std::vector<Cell>&shell,PadCache&padCache,const Inh*inherited,const PadCache*given,bool air=false,bool partsDecided=false){ // (given: the flesh a burst gave back - bonefill; partsDecided: the caller
    // decided which part each cell belongs to (owners(), MaskPieces3.h) - B is then the gone cells alone, not other parts' anatomy)
 static const bool off=std::getenv("GFY_HB_NO_ATTACH")!=nullptr;if(off)return;
 const auto t0=core::Clock::now();const int R=8,PAD=R+2;const int sz[3]={grid.size.x,grid.size.y,grid.size.z};
 for(int a=0;a<3;++a){box.lo[a]=std::max(box.lo[a],0)&~3;box.hi[a]=std::min((box.hi[a]+3)&~3,(sz[a]+3)&~3);}
 if(!box.valid())return;
 int b0[3],nb[3],c0[3],c1[3];
 for(int a=0;a<3;++a){b0[a]=(box.lo[a]>>2)-PAD;nb[a]=((box.hi[a]-box.lo[a])>>2)+2*PAD;c0[a]=std::max(b0[a]*4,0);c1[a]=std::min((b0[a]+nb[a])*4,sz[a]);}
 const std::size_t wpr=std::size_t(nb[0]+63)/64,rows=std::size_t(nb[1])*std::size_t(nb[2]),NW=rows*wpr;
 if(!NW||NW>(std::size_t(1)<<16)){++st.skipped;return;}
 std::vector<std::uint64_t> O(NW,0),U(NW,0),Ot(NW,0); // (bonefill: Ot = the blocks of other pieces' anatomy and of removed tissue)
 auto word=[&](int bx,int by,int bz){return (std::size_t(bz-b0[2])*std::size_t(nb[1])+std::size_t(by-b0[1]))*wpr+std::size_t((bx-b0[0])>>6);};
 auto bitOf=[&](int bx){return std::uint64_t(1)<<((bx-b0[0])&63);};
 auto inW=[&](int bx,int by,int bz){return bx>=b0[0]&&by>=b0[1]&&bz>=b0[2]&&bx<b0[0]+nb[0]&&by<b0[1]+nb[1]&&bz<b0[2]+nb[2];};
 std::size_t ownSpans=0;
 const std::size_t gzw=std::size_t(nb[2]+63)/64;std::vector<std::uint64_t> gO(std::size_t(nb[0])*std::size_t(nb[1])*gzw,0),gOt(gO.size(),0);
 auto setRun=[&](std::vector<std::uint64_t>&v,std::size_t g,int a,int b){for(int z=a;z<=b;){const int s=z&63,n=std::min(64-s,b-z+1);
   v[g+std::size_t(z>>6)]|=(n==64?~std::uint64_t(0):((std::uint64_t(1)<<n)-1))<<s;z+=n;}};
 for(int y=c0[1];y<c1[1];++y){auto col=std::lower_bound(volume.columns.begin(),volume.columns.end(),std::array<int,2>{{c0[0],y}},
   [](const gfy_connectivity::Column&a,const std::array<int,2>&q){return gfy_connectivity::xyLess(a.x,a.y,q[0],q[1]);});
  for(;col!=volume.columns.end()&&col->y==y&&col->x<c1[0];++col)for(auto s=col->first;s<col->first+col->count;++s){const auto&sp=volume.spans[s];
   if(sp.component>=pieceOf.size())continue;const int j=pieceOf[sp.component];if(j<0)continue;const int lo=std::max<int>(sp.lo,c0[2]),hi=std::min<int>(sp.hi,c1[2]);if(lo>=hi)continue;
   const bool mine=std::size_t(j)==k;ownSpans+=mine?1u:0u;
   // (blockfine: a span's blocks as a bit range of its column group's z mask, scattered below once a block)
   const std::size_t g=(std::size_t((y>>2)-b0[1])*std::size_t(nb[0])+std::size_t((col->x>>2)-b0[0]))*gzw;
   setRun(mine?gO:gOt,g,(lo>>2)-b0[2],((hi-1)>>2)-b0[2]);}}
 if(!ownSpans)return; // none of the piece's anatomy near this box: not this pass's case
 for(int by=0;by<nb[1];++by)for(int bx=0;bx<nb[0];++bx){const std::size_t g=(std::size_t(by)*std::size_t(nb[0])+std::size_t(bx))*gzw;
  for(std::size_t q=0;q<gzw;++q){const std::uint64_t mo=gO[g+q],mt=gOt[g+q];
   for(std::uint64_t m=mo|mt;m;m&=m-1){const unsigned zb=unsigned(q*64+lowBit64(m));const std::uint64_t bit=std::uint64_t(1)<<zb%64;
    const std::size_t w=word(b0[0]+bx,b0[1]+by,b0[2]+int(zb));const std::uint64_t b=bitOf(b0[0]+bx);U[w]|=b;if(mo&bit)O[w]|=b;if(mt&bit)Ot[w]|=b;}}}
 const std::vector<std::uint64_t> OaB=Ot; // (blockfine: the blocks of the other pieces' anatomy alone)
 {Cell lastG{INT_MIN,INT_MIN,INT_MIN};for(const Cell&g:gone){const Cell gb{g.x>>2,g.y>>2,g.z>>2};if(gb==lastG)continue;lastG=gb; // (blockfine: once a block)
   if(inW(gb.x,gb.y,gb.z)){U[word(gb.x,gb.y,gb.z)]|=bitOf(gb.x);Ot[word(gb.x,gb.y,gb.z)]|=bitOf(gb.x);}}}
 const std::vector<std::uint64_t> U0=U,O0=O;std::vector<std::uint64_t> T(NW),Un(NW),Up;
 const std::uint64_t lastMask=(nb[0]&63)?((std::uint64_t(1)<<(nb[0]&63))-1):~std::uint64_t(0);const std::size_t n1=std::size_t(nb[1]),n2=std::size_t(nb[2]);
 auto dilate=[&](std::vector<std::uint64_t>&v){ // one block, 26 neighbours: x in the words, then the rows beside, then the slabs
  for(std::size_t r=0;r<rows;++r){std::uint64_t carry=0;for(std::size_t i=0;i<wpr;++i){const std::uint64_t cur=v[r*wpr+i],nx=i+1<wpr?(v[r*wpr+i+1]&1u):0u;
    T[r*wpr+i]=cur|(cur<<1)|carry|(cur>>1)|(nx<<63);carry=cur>>63;}T[r*wpr+wpr-1]&=lastMask;}
  for(std::size_t z=0;z<n2;++z)for(std::size_t y=0;y<n1;++y){const std::size_t r=(z*n1+y)*wpr;for(std::size_t i=0;i<wpr;++i)v[r+i]=T[r+i]|(y?T[r-wpr+i]:0u)|(y+1<n1?T[r+wpr+i]:0u);}
  const std::size_t slab=n1*wpr;
  for(std::size_t z=0;z<n2;++z)for(std::size_t q=0;q<slab;++q){const std::size_t r=z*slab+q;T[r]=v[r]|(z?v[r-slab]:0u)|(z+1<n2?v[r+slab]:0u);}
  v.swap(T);};
 static const bool blockRule=std::getenv("GFY_HB_BLOCK_RULE")!=nullptr; // blockfine: headmask2's 8 mm blocks (probe comparisons)
 std::vector<std::uint64_t> SB(NW,0);for(const Cell&c:shell)if(inW(c.x>>2,c.y>>2,c.z>>2))SB[word(c.x>>2,c.y>>2,c.z>>2)]|=bitOf(c.x>>2);
 const int tnx=(sz[0]+7)/8,tny=(sz[1]+7)/8;std::size_t made=0;
 if(blockRule){
 for(std::size_t i=0;i<NW;++i)Un[i]=U[i]&~O[i];
 for(int r=1;r<=R;++r){Up=U;dilate(U);dilate(O);for(std::size_t i=0;i<NW;++i)Un[i]|=(U[i]&~Up[i])&~O[i];}
 // the marks: an unattached block inside the box that holds gear, may hold padding, or holds another piece's anatomy
 // (or one of the piece's triangles: SB)
 Cell lastT{INT_MIN,INT_MIN,INT_MIN};const core::Tile*gt=nullptr;
 for(int bz=box.lo[2]>>2;bz<(box.hi[2]>>2);++bz)for(int by=box.lo[1]>>2;by<(box.hi[1]>>2);++by){const std::size_t r=(std::size_t(bz-b0[2])*n1+std::size_t(by-b0[1]))*wpr;
  for(std::size_t i=0;i<wpr;++i)for(std::uint64_t m=Un[r+i];m;m&=m-1){const int bx=b0[0]+int(i*64+lowBit64(m));if(bx<(box.lo[0]>>2)||bx>=(box.hi[0]>>2))continue;
   const int x0=bx*4,y0=by*4,z0=bz*4;if(x0<0||y0<0||z0<0||x0+4>sz[0]||y0+4>sz[1]||z0+4>sz[2])continue;const Cell tc{x0>>3,y0>>3,z0>>3};
   std::uint64_t cm=0;for(int ly=0;ly<4;++ly)cm|=std::uint64_t(0xF)<<(((y0&7)+ly)*8+(z0&7));
   const std::size_t w=word(bx,by,bz);const std::uint64_t b=bitOf(bx);bool any=air||((U0[w]&b)&&!(O0[w]&b))||(SB[w]&b)!=0; // (air: whatever the block holds)
   if(!any&&padF){const std::size_t ti=(std::size_t(tc.z)*std::size_t(tny)+std::size_t(tc.y))*std::size_t(tnx)+std::size_t(tc.x);if(tf&&ti<padF->size()&&(*padF)[ti]!=0)for(int q=0;q<16&&!any;++q){const int a=q&3,b2=q>>2,c=(a+b2)&3; // 16 of the 64 cells: every layer of every axis four times
     any=(tf->sample(grid.origin.x+(x0+a+.5)*grid.pitch,grid.origin.y+(y0+b2+.5)*grid.pitch,grid.origin.z+(z0+c+.5)*grid.pitch)&64u)!=0;}}
   if(!any&&gear){if(!(tc==lastT)){lastT=tc;gt=gear->tile(tc.x,tc.y,tc.z);}if(gt)for(int lx=0;lx<4&&!any;++lx)any=(gt->removed[std::size_t((x0&7)+lx)]&cm)!=0;}
   if(!any)continue;
   const double pq[3]={grid.origin.x+(x0+2)*grid.pitch,grid.origin.y+(y0+2)*grid.pitch,grid.origin.z+(z0+2)*grid.pitch};if(inZone(pq))continue;
   auto&tile=marks[tc];for(int lx=0;lx<4;++lx)tile.removed[std::size_t((x0&7)+lx)]|=cm;++made;}}
 }else{ // ---- blockfine: THE RULE PER CELL (the head of the namespace)
  const auto f0=core::Clock::now();static const bool check=std::getenv("GFY_HB_CHECK")!=nullptr;
  const double msBefore=core::ms(t0)-core::ms(f0); // (the pass's work before this branch: its window, spans, removed blocks)
  // the cut's neighbourhood: a piece that meets another piece's anatomy is judged within 36 cells of where it meets it (the half of
  // a body a waist cut leaves would judge every crater of the half, in its whole box); a piece that meets none (a fragment broken
  // off across removed tissue) in its whole box
  CellBox rb=box;
  {std::vector<std::uint64_t> D=OaB;dilate(D);CellBox ib;
   for(int bz=box.lo[2]>>2;bz<(box.hi[2]>>2);++bz)for(int by=box.lo[1]>>2;by<(box.hi[1]>>2);++by){const std::size_t r=(std::size_t(bz-b0[2])*n1+std::size_t(by-b0[1]))*wpr;
    for(std::size_t i=0;i<wpr;++i)for(std::uint64_t m=D[r+i]&O0[r+i];m;m&=m-1){const int bx=b0[0]+int(i*64+lowBit64(m));if(bx<(box.lo[0]>>2)||bx>=(box.hi[0]>>2))continue;
     ib.add(bx*4,by*4,bz*4,bz*4+4);ib.add(bx*4+3,by*4+3,bz*4,bz*4+4);}}
   if(ib.valid())for(int a=0;a<3;++a){rb.lo[a]=std::max(box.lo[a],(ib.lo[a]-36)&~3);rb.hi[a]=std::min(box.hi[a],(ib.hi[a]+36+3)&~3);}}
  // the anatomy that is gone, cell by cell (goneBlocks: the grid's removed cells where the model holds anatomy, once per separation)
  // partsDecided: B = the gone cells alone - another part's anatomy is no B (owners() decided between the parts; its cells are still
  // cut, they are hidden on this part anyway): Gt = the gone cells' blocks
  const std::vector<Cell>&goneC=gone;std::vector<std::uint64_t> Gt;
  if(partsDecided){Gt.assign(NW,0);Cell lastG{INT_MIN,INT_MIN,INT_MIN};for(const Cell&g:gone){const Cell gb{g.x>>2,g.y>>2,g.z>>2};if(gb==lastG)continue;lastG=gb;
    if(inW(gb.x,gb.y,gb.z))Gt[word(gb.x,gb.y,gb.z)]|=bitOf(gb.x);}}
  const std::vector<std::uint64_t>&Bt=partsDecided?Gt:Ot; // (Ot: the other pieces' anatomy and the gone cells' blocks)
  // block distances over the box (Chebyshev, in blocks): kB to the blocks holding B, kA to the blocks holding the piece's anatomy
  const int q0[3]={rb.lo[0]>>2,rb.lo[1]>>2,rb.lo[2]>>2},qn[3]={std::max(0,(rb.hi[0]-rb.lo[0])>>2),std::max(0,(rb.hi[1]-rb.lo[1])>>2),std::max(0,(rb.hi[2]-rb.lo[2])>>2)};
  const std::size_t NQ=std::size_t(qn[0])*std::size_t(qn[1])*std::size_t(qn[2]);
  std::vector<std::uint8_t> kB(NQ,255),kA(NQ,255);
  auto qi=[&](int bx,int by,int bz){return (std::size_t(bz-q0[2])*std::size_t(qn[1])+std::size_t(by-q0[1]))*std::size_t(qn[0])+std::size_t(bx-q0[0]);};
  auto rank=[&](std::vector<std::uint64_t> D,std::vector<std::uint8_t>&kv,int steps){std::vector<std::uint64_t> seen(NW,0);
   for(int r=0;r<=steps;++r){if(r)dilate(D);
    for(int bz=q0[2];bz<q0[2]+qn[2];++bz)for(int by=q0[1];by<q0[1]+qn[1];++by){const std::size_t row=(std::size_t(bz-b0[2])*n1+std::size_t(by-b0[1]))*wpr;
     for(std::size_t i=0;i<wpr;++i){const std::uint64_t nw=D[row+i]&~seen[row+i];seen[row+i]|=nw;
      for(std::uint64_t m=nw;m;m&=m-1){const int bx=b0[0]+int(i*64+lowBit64(m));if(bx>=q0[0]&&bx<q0[0]+qn[0])kv[qi(bx,by,bz)]=std::uint8_t(r);}}}}};
  rank(Bt,kB,8);rank(O0,kA,9);
  // the blocks that COULD hold a cell to cut: B within 32 cells of a cell of theirs (the cells of blocks kB apart are 4 kB - 3 apart
  // or more), not every cell nearer its own anatomy (own anatomy kA blocks away lies within sqrt(3) (4 kA + 3) of each cell), and
  // material in them (another piece's anatomy, triangles' cells, a padding tile, gear)
  const bool fineShell=!shell.empty();std::vector<std::uint64_t> SBd=SB;if(fineShell)dilate(SBd); // (a triangle's cell and the cells round it)
  // farBlock: no cell of a block kB / kA blocks from B / its own anatomy can be cut
  auto farBlock=[&](int b,int a){return b>8||(b>=1&&a<=9&&double(4*b-3)>std::sqrt(3.0)*double(4*a+3));};
  struct QB {int x,y,z;};std::vector<QB> cand;int lo3[3]={INT_MAX,INT_MAX,INT_MAX},hi3[3]={INT_MIN,INT_MIN,INT_MIN};
  {Cell lastT{INT_MIN,INT_MIN,INT_MIN};const core::Tile*gt=nullptr;
   for(int bz=q0[2];bz<q0[2]+qn[2];++bz)for(int by=q0[1];by<q0[1]+qn[1];++by)for(int bx=q0[0];bx<q0[0]+qn[0];++bx){
    const std::size_t ix=qi(bx,by,bz);const int b=kB[ix],a=kA[ix];
    if(farBlock(b,a)&&!(partsDecided&&(OaB[word(bx,by,bz)]&bitOf(bx))))continue; // (partsDecided: another part's anatomy is cut where it lies)
    const int x0=bx*4,y0=by*4,z0=bz*4;if(x0<0||y0<0||z0<0||x0>=sz[0]||y0>=sz[1]||z0>=sz[2])continue;
    const std::size_t w=word(bx,by,bz);const std::uint64_t bb=bitOf(bx);const Cell tc{x0>>3,y0>>3,z0>>3};
    bool any=air||(OaB[w]&bb)!=0||(SBd[w]&bb)!=0;
    if(!any&&padF&&tf){const std::size_t ti=(std::size_t(tc.z)*std::size_t(tny)+std::size_t(tc.y))*std::size_t(tnx)+std::size_t(tc.x);any=ti<padF->size()&&(*padF)[ti]!=0;}
    if(!any&&gear){if(!(tc==lastT)){lastT=tc;gt=gear->tile(tc.x,tc.y,tc.z);}
     if(gt){std::uint64_t cm=0;for(int ly=0;ly<4;++ly)cm|=std::uint64_t(0xF)<<(((y0&7)+ly)*8+(z0&7));for(int lx=0;lx<4&&!any;++lx)any=(gt->removed[std::size_t((x0&7)+lx)]&cm)!=0;}}
    if(!any)continue;
    cand.push_back(QB{bx,by,bz});lo3[0]=std::min(lo3[0],x0);lo3[1]=std::min(lo3[1],y0);lo3[2]=std::min(lo3[2],z0);
    hi3[0]=std::max(hi3[0],x0+4);hi3[1]=std::max(hi3[1],y0+4);hi3[2]=std::max(hi3[2],z0+4);}}
  // the cells: bit columns along z over the candidates' box 37 cells out (every cell a decision can need lies inside)
  struct CV {int w0[3]{},w1[3]{},cn[3]{};std::size_t nzw=0,words=0;std::vector<std::uint64_t> A,B,O,S;std::vector<std::uint8_t> col; // (col: 1 A in the column, 2 B)
   std::size_t at(int x,int y)const{return (std::size_t(y-w0[1])*std::size_t(cn[0])+std::size_t(x-w0[0]))*nzw;}
   bool in(int x,int y,int z)const{return x>=w0[0]&&y>=w0[1]&&z>=w0[2]&&x<w1[0]&&y<w1[1]&&z<w1[2];}
   static bool bit(const std::vector<std::uint64_t>&v,std::size_t r,int b){return ((v[r+std::size_t(b>>6)]>>(b&63))&1u)!=0;}
   bool has(const std::vector<std::uint64_t>&v,int x,int y,int z)const{return !v.empty()&&in(x,y,z)&&bit(v,at(x,y),z-w0[2]);}};
  auto build=[&](CV&V,const int lo[3],const int hi[3],int H)->bool{
   for(int a=0;a<3;++a){V.w0[a]=std::max(lo[a]-H,0);V.w1[a]=std::min(hi[a]+H,sz[a]);V.cn[a]=std::max(V.w1[a]-V.w0[a],0);}
   V.nzw=std::size_t(V.cn[2]+63)/64;V.words=std::size_t(V.cn[0])*std::size_t(V.cn[1])*V.nzw;if(!V.words||V.words>(std::size_t(1)<<21))return false;
   V.A.assign(V.words,0);V.B.assign(V.words,0);V.O.assign(V.words,0);if(fineShell)V.S.assign(V.words,0);V.col.assign(std::size_t(V.cn[0])*std::size_t(V.cn[1]),0);
   auto setZ=[&](std::vector<std::uint64_t>&v,std::size_t r,int z0,int z1){for(int a=z0-V.w0[2],b=z1-V.w0[2];a<b;){const int s=a&63,n=std::min(64-s,b-a);
     v[r+std::size_t(a>>6)]|=(n==64?~std::uint64_t(0):((std::uint64_t(1)<<n)-1))<<s;a+=n;}};
   auto setBit=[&](std::vector<std::uint64_t>&v,int x,int y,int z){if(!V.in(x,y,z))return;const int b=z-V.w0[2];v[V.at(x,y)+std::size_t(b>>6)]|=std::uint64_t(1)<<(b&63);};
   for(int y=V.w0[1];y<V.w1[1];++y){auto col=std::lower_bound(volume.columns.begin(),volume.columns.end(),std::array<int,2>{{V.w0[0],y}},
     [](const gfy_connectivity::Column&a,const std::array<int,2>&q){return gfy_connectivity::xyLess(a.x,a.y,q[0],q[1]);});
    for(;col!=volume.columns.end()&&col->y==y&&col->x<V.w1[0];++col){const std::size_t r=V.at(col->x,y);
     for(auto s=col->first;s<col->first+col->count;++s){const auto&sp=volume.spans[s];if(sp.component>=pieceOf.size())continue;const int j=pieceOf[sp.component];if(j<0)continue;
      const int lo2=std::max<int>(sp.lo,V.w0[2]),hi2=std::min<int>(sp.hi,V.w1[2]);if(lo2>=hi2)continue;
      const std::size_t ci=std::size_t(y-V.w0[1])*std::size_t(V.cn[0])+std::size_t(col->x-V.w0[0]);
      if(std::size_t(j)==k){setZ(V.A,r,lo2,hi2);V.col[ci]|=1;}else{if(!partsDecided){setZ(V.B,r,lo2,hi2);V.col[ci]|=2;}setZ(V.O,r,lo2,hi2);}}}} // (partsDecided: no B)
   for(const Cell&g:goneC)if(V.in(g.x,g.y,g.z)){setBit(V.B,g.x,g.y,g.z);V.col[std::size_t(g.y-V.w0[1])*std::size_t(V.cn[0])+std::size_t(g.x-V.w0[0])]|=2;}
   if(fineShell)for(const Cell&c:shell)for(int dz=-1;dz<=1;++dz)for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx)setBit(V.S,c.x+dx,c.y+dy,c.z+dz);
   return true;};
  // a cell's material besides another piece's anatomy (the brute-force check's; the pass reads the same by blocks): a triangle's cell
  // (or one round it), gear, padding (the model's flag), anything (air)
  auto material=[&](const CV&V,int x,int y,int z)->bool{if(air||V.has(V.S,x,y,z))return true;const Cell tc{x>>3,y>>3,z>>3};
   if(gear){const core::Tile*g=gear->tile(tc.x,tc.y,tc.z);if(g&&((g->removed[std::size_t(x&7)]>>((y&7)*8+(z&7)))&1u))return true;}
   if(padF&&tf){const std::size_t ti=(std::size_t(tc.z)*std::size_t(tny)+std::size_t(tc.y))*std::size_t(tnx)+std::size_t(tc.x);
    if(ti<padF->size()&&(*padF)[ti]!=0&&(tf->sample(grid.origin.x+(x+.5)*grid.pitch,grid.origin.y+(y+.5)*grid.pitch,grid.origin.z+(z+.5)*grid.pitch)&64u))return true;}
   return false;};
  std::size_t exact=0,words=0;std::vector<Cell> mineC;
  CV V;
  const double msPre=core::ms(f0);double msBuild=0,msJoint=0;std::size_t joint=0,visJ=0,tilesJ=0;
  if(!cand.empty()&&!build(V,lo3,hi3,40))++st.skipped; // (40: the window's z starts on a block, so a block's 4 cells sit in one word)
  else if(!cand.empty()){words=V.words;msBuild=core::ms(f0)-msPre;
   auto colFlag=[&](int x,int y)->int{if(x<V.w0[0]||x>=V.w1[0]||y<V.w0[1]||y>=V.w1[1])return 0;return V.col[std::size_t(y-V.w0[1])*std::size_t(V.cn[0])+std::size_t(x-V.w0[0])];};
   // the nearest set cell along a column from each of the 4 cells z0 .. z0+3 of a block (z0 - the window's z a multiple of 4: one
   // word holds them; 99: none within 60 cells)
   static const std::array<std::array<std::uint8_t,4>,16> nibD=[]{std::array<std::array<std::uint8_t,4>,16> t{};for(int n=0;n<16;++n)for(int lz=0;lz<4;++lz){int b=99;
     for(int j=0;j<4;++j)if((n>>j)&1)b=std::min(b,std::abs(j-lz));t[std::size_t(n)][std::size_t(lz)]=std::uint8_t(b);}return t;}();
   auto near4p=[&](const std::uint64_t*c,int zl0,int g[4]){
    const std::size_t w=std::size_t(zl0>>6);const int s=zl0&63;const unsigned n=unsigned((c[w]>>s)&0xFu);
    int up=99,dn=99; // above the 4 cells: from z0+4; below: from z0-1
    {std::uint64_t m=s+4<64?(c[w]>>(s+4)):0;if(m)up=int(lowBit64(m));else if(w+1<V.nzw&&c[w+1])up=60-s+int(lowBit64(c[w+1]));}
    {std::uint64_t m=s>0?(c[w]<<(64-s)):0;if(m)dn=63-int(highBit64(m));else if(w>0&&c[w-1])dn=s+63-int(highBit64(c[w-1]));}
    for(int lz=0;lz<4;++lz)g[lz]=std::min(int(nibD[n][std::size_t(lz)]),std::min(up<99?up+4-lz:99,dn<99?dn+1+lz:99));};
   auto near4=[&](const std::vector<std::uint64_t>&v,int x,int y,int zl0,int g[4]){near4p(v.data()+V.at(x,y),zl0,g);};
   auto tileBlock=[](const core::Tile&t,int x0,int y0,int z0)->std::uint64_t{std::uint64_t m=0;
    for(int lx=0;lx<4;++lx)for(int ly=0;ly<4;++ly)m|=((t.removed[std::size_t((x0&7)+lx)]>>(((y0&7)+ly)*8+(z0&7)))&0xFu)<<(lx*16+ly*4);return m;};
   // tiles of 4 x 4 columns (the window's x and y start on a block): the OR of their columns, A and B, and which they hold
   const int ntx=(V.cn[0]+3)/4,nty=(V.cn[1]+3)/4;std::vector<std::uint64_t> TA(std::size_t(ntx)*std::size_t(nty)*V.nzw,0),TB(TA.size(),0);std::vector<std::uint8_t> tflag(std::size_t(ntx)*std::size_t(nty),0);
   for(int y=V.w0[1];y<V.w1[1];++y)for(int x=V.w0[0];x<V.w1[0];++x){const int f=colFlag(x,y);if(!f)continue;const std::size_t ti=std::size_t((y-V.w0[1])>>2)*std::size_t(ntx)+std::size_t((x-V.w0[0])>>2),r=V.at(x,y);
    tflag[ti]|=std::uint8_t(f);for(std::size_t w=0;w<V.nzw;++w){if(f&1)TA[ti*V.nzw+w]|=V.A[r+w];if(f&2)TB[ti*V.nzw+w]|=V.B[r+w];}}
   // the tiles round a block in order of their least distance to its columns (4 |d| - 3 a side)
   struct TO {int dx,dy,g2;};
   static const std::vector<TO> tiles=[]{std::vector<TO> v;for(int dy=-9;dy<=9;++dy)for(int dx=-9;dx<=9;++dx){const int gx=std::max(0,4*std::abs(dx)-3),gy=std::max(0,4*std::abs(dy)-3);v.push_back(TO{dx,dy,gx*gx+gy*gy});}
    std::stable_sort(v.begin(),v.end(),[](const TO&p,const TO&q){return p.g2<q.g2;});return v;}();
   // scan one set (1 A, 2 B) round a block, tile by tile in that order and column by column, while a tile may hold a cell within
   // need() (squared) of an open cell (the tile's OR column bounds its columns' cells); visit(X, Y, g): a column's distances along z
   struct FS {std::size_t exact=0,joint=0,cols=0,tiles=0;}; // (counts, one per worker)
   auto scan=[&](FS&f,int set,int x0,int y0,int z0,unsigned lzm,auto&&need,auto&&visit){const int zl0=z0-V.w0[2],BX=(x0-V.w0[0])>>2,BY=(y0-V.w0[1])>>2;
    const std::vector<std::uint64_t>&TV=set==1?TA:TB;const std::vector<std::uint64_t>&CVv=set==1?V.A:V.B;std::size_t n=0;int nd=need();
    for(const TO&t:tiles){if((n++&3)==0)nd=need();if(nd<0||t.g2>nd)break;
     const int TX=BX+t.dx,TY=BY+t.dy;if(TX<0||TY<0||TX>=ntx||TY>=nty)continue;const std::size_t ti=std::size_t(TY)*std::size_t(ntx)+std::size_t(TX);if(!(tflag[ti]&set))continue;
     int g[4];near4p(TV.data()+ti*V.nzw,zl0,g);bool use=false;for(int lz=0;lz<4;++lz)if(((lzm>>lz)&1u)&&g[lz]<99&&t.g2+g[lz]*g[lz]<=nd)use=true;if(!use)continue;++f.tiles;
     for(int cy=0;cy<4;++cy)for(int cx=0;cx<4;++cx){const int X=V.w0[0]+TX*4+cx,Y=V.w0[1]+TY*4+cy;if(!(colFlag(X,Y)&set))continue;int gc[4];near4(CVv,X,Y,zl0,gc);++f.cols;visit(X,Y,gc);}}};
   // the open cells of one block, exactly, in three steps: (1) an upper bound a on each cell's distance to its own anatomy (the
   // columns round the block; else the block distance: A within sqrt(3) (4 kA + 3)); (2) B within a (and 32 cells), nearest first:
   // none there - the cell keeps (B is no nearer than its own anatomy); a B cell nearer than the block's least distance to its own
   // anatomy (4 kA - 3) - cut; (3) a cell with B at b within a: its own anatomy within b keeps it, none - cut. Every distance squared,
   // in whole cells: a tie keeps. A decided cell no longer widens a scan.
   // upd: one column's distances into all 64 cells of the block at once (v: squared distance so far, c = 16 x + 4 y + z)
   auto upd=[](int*v,int X,int Y,int x0,int y0,const int*g){int dx[4],dy[4],gz[4];
    for(int l=0;l<4;++l){const int p=X-(x0+l),q=Y-(y0+l);dx[l]=p*p;dy[l]=q*q;gz[l]=g[l]<99?g[l]*g[l]:(1<<28);}
    for(int lx=0;lx<4;++lx)for(int ly=0;ly<4;++ly){int*r=v+lx*16+ly*4;const int h=dx[lx]+dy[ly];for(int lz=0;lz<4;++lz){const int d=h+gz[lz];r[lz]=d<r[lz]?d:r[lz];}}};
   auto decideBlock=[&](FS&f,int x0,int y0,int z0,std::uint64_t Um,int ka)->std::uint64_t{alignas(16) int a2[64],bB[64],nA[64];
    const int ach=ka>9?(1<<29):3*(4*ka+3)*(4*ka+3),la=ka>9?37:(ka>=1?4*ka-3:0),la2=la*la;unsigned lzm=0;
    for(int c=0;c<64;++c){a2[c]=ach;bB[c]=1<<29;nA[c]=1<<29;}
    for(std::uint64_t m=Um;m;m&=m-1){lzm|=1u<<(lowBit64(m)&3u);++f.exact;}++f.joint;
    const int zl0=z0-V.w0[2];
    for(int Y=y0-1;Y<=y0+4;++Y)for(int X=x0-1;X<=x0+4;++X){if(!(colFlag(X,Y)&1))continue;int g[4];near4(V.A,X,Y,zl0,g);upd(a2,X,Y,x0,y0,g);}
    std::uint64_t act=Um,cut=0;std::size_t nv=0;
    auto settle=[&]{for(std::uint64_t m=act;m;m&=m-1){const int c=int(lowBit64(m));if(bB[c]<la2&&bB[c]<=1024){cut|=std::uint64_t(1)<<c;act&=~(std::uint64_t(1)<<c);}}};
    scan(f,2,x0,y0,z0,lzm,[&]{settle();int q=-1;for(std::uint64_t m=act;m;m&=m-1){const int c=int(lowBit64(m));q=std::max(q,std::min(std::min(a2[c],1024),bB[c]));}return q;},
     [&](int X,int Y,const int*g){upd(bB,X,Y,x0,y0,g);++nv;});
    settle();
    std::uint64_t open=0;unsigned lz3=0;for(std::uint64_t m=act;m;m&=m-1){const int c=int(lowBit64(m));if(bB[c]<=std::min(a2[c],1024)){open|=std::uint64_t(1)<<c;lz3|=1u<<(c&3);}}
    std::uint64_t found=0;
    auto foundNow=[&]{for(std::uint64_t m=open&~found;m;m&=m-1){const int c=int(lowBit64(m));if(nA[c]<=bB[c])found|=std::uint64_t(1)<<c;}};
    if(open)scan(f,1,x0,y0,z0,lz3,[&]{foundNow();int q=-1;for(std::uint64_t m=open&~found;m;m&=m-1)q=std::max(q,bB[lowBit64(m)]);return q;},
     [&](int X,int Y,const int*g){upd(nA,X,Y,x0,y0,g);});
    foundNow();
    return cut|(open&~found);};
   // the candidates' padding tiles first (the separation's cache is written here only; the workers read it)
   if(padF&&tf){std::vector<Cell> need;{Cell last{INT_MIN,INT_MIN,INT_MIN};for(const QB&q:cand){const Cell tc{q.x>>1,q.y>>1,q.z>>1};if(tc==last)continue;last=tc;
      const std::size_t ti=(std::size_t(tc.z)*std::size_t(tny)+std::size_t(tc.y))*std::size_t(tnx)+std::size_t(tc.x);if(ti<padF->size()&&(*padF)[ti]&&!padCache.count(tc))need.push_back(tc);}}
    std::sort(need.begin(),need.end(),[](const Cell&a,const Cell&b){return a.x!=b.x?a.x<b.x:(a.y!=b.y?a.y<b.y:a.z<b.z);});need.erase(std::unique(need.begin(),need.end()),need.end());
    std::vector<core::Tile> pts(need.size());
    lowpoly_pieces::eachBlock(need.size(),8,[&](std::size_t first,std::size_t last,std::size_t){for(std::size_t i=first;i<last;++i){const Cell&tk=need[i];core::Tile pt{};
      for(int c=0;c<512;++c){const int lx=c>>6,ly=(c>>3)&7,lz=c&7;
       if(tf->sample(grid.origin.x+(tk.x*8+lx+.5)*grid.pitch,grid.origin.y+(tk.y*8+ly+.5)*grid.pitch,grid.origin.z+(tk.z*8+lz+.5)*grid.pitch)&64u)pt.removed[std::size_t(lx)]|=std::uint64_t(1)<<(ly*8+lz);}
      pts[i]=pt;}});
    for(std::size_t i=0;i<need.size();++i){padCache.emplace(need[i],pts[i]);++st.padTiles;}}
   auto padFind=[&](const Cell&tk)->const core::Tile*{if(!padF||!tf)return nullptr;const auto it=padCache.find(tk);return it==padCache.end()?nullptr:&it->second;};
   // each block's cells to cut, on the workers (each writes its own slot); the marks after, in order
   std::vector<std::uint64_t> cutOf(cand.size(),0),openOf(cand.size(),0);std::vector<std::uint8_t> kaOf(cand.size(),0);std::vector<FS> fs(core::parallelWidth()+1);const auto p0=core::Clock::now();
   static const double edtRatio=std::getenv("GFY_HB_EDT_RATIO")?std::atof(std::getenv("GFY_HB_EDT_RATIO")):133.;
   lowpoly_pieces::eachBlock(cand.size(),16,[&](std::size_t first,std::size_t last,std::size_t chunk){FS&f=fs[std::min(chunk,fs.size()-1)];
    Cell lastG{INT_MIN,INT_MIN,INT_MIN};const core::Tile*gTile=nullptr;
   for(std::size_t ci=first;ci<last;++ci){const QB&q=cand[ci];const int x0=q.x*4,y0=q.y*4,z0=q.z*4;
    std::uint64_t valid=~std::uint64_t(0);if(x0+4>sz[0]||y0+4>sz[1]||z0+4>sz[2]){valid=0;for(int c=0;c<64;++c)if(x0+(c>>4)<sz[0]&&y0+((c>>2)&3)<sz[1]&&z0+(c&3)<sz[2])valid|=std::uint64_t(1)<<c;}
    std::uint64_t mA=0,mO=0,mB=0,mS=0;{const int zl=z0-V.w0[2];const std::size_t zw=std::size_t(zl>>6);const int zs=zl&63;
     for(int lx=0;lx<4;++lx){const int x=x0+lx;if(x>=V.w1[0])continue;for(int ly=0;ly<4;++ly){const int y=y0+ly;if(y>=V.w1[1])continue;const std::size_t r=V.at(x,y)+zw;const int sh=lx*16+ly*4;
       mA|=((V.A[r]>>zs)&0xFu)<<sh;mO|=((V.O[r]>>zs)&0xFu)<<sh;mB|=((V.B[r]>>zs)&0xFu)<<sh;if(fineShell)mS|=((V.S[r]>>zs)&0xFu)<<sh;}}}
    const Cell tc{x0>>3,y0>>3,z0>>3};
    if(!(tc==lastG)){lastG=tc;const auto it=grid.tiles.find(tc);gTile=it==grid.tiles.end()?nullptr:&it->second;}
    const std::uint64_t mR=gTile?tileBlock(*gTile,x0,y0,z0):0; // removed in the grid: hidden on every piece already
    std::uint64_t cut=mO&valid; // another piece's anatomy: B at 0 - cut
    // the cells to judge: not anatomy, not removed, holding material, no own anatomy beside them (A at 1: B is no nearer)
    std::uint64_t J=valid&~(mA|mB|mO|mR); // (mO: cut above - within mB unless partsDecided)
    if(J&&!air){std::uint64_t M=mS;
     if(gear)if(const core::Tile*g=gear->tile(tc.x,tc.y,tc.z))M|=tileBlock(*g,x0,y0,z0);
     if(J&~M)if(const core::Tile*pt=padFind(tc))M|=tileBlock(*pt,x0,y0,z0);
     J&=M;}
    for(std::uint64_t m=J;m;m&=m-1){const int c=int(lowBit64(m)),x=x0+(c>>4),y=y0+((c>>2)&3),z=z0+(c&3);
     if(V.has(V.A,x+1,y,z)||V.has(V.A,x-1,y,z)||V.has(V.A,x,y+1,z)||V.has(V.A,x,y-1,z)||V.has(V.A,x,y,z+1)||V.has(V.A,x,y,z-1))J&=~(std::uint64_t(1)<<c);}
    if(J){const std::size_t ix=qi(q.x,q.y,q.z);const int kb=kB[ix],ka=kA[ix];
     // the block distances alone: own anatomy 4 kA - 3 cells away or more (kA past 9: 37), B within sqrt(3) (4 kB + 3)
     const double chLA=ka>9?37.:(ka>=1?4.*ka-3.:0.),chUB=std::sqrt(3.)*(4.*kb+3.);
     if(chUB<chLA-1e-9&&chUB<32-1e-9)cut|=J;
     else if(!farBlock(kb,ka)){openOf[ci]=J;kaOf[ci]=std::uint8_t(std::min(ka,255));}} // (far: a block kept for another part's anatomy alone)
    cutOf[ci]=cut;}});
   // the open cells, exactly: per layer of blocks (dense layers; a sparse layer's blocks each by its search, above) (4 cells of z; a block lies in one layer, so a layer's worker writes only its
   // blocks' slots), the squared Euclidean distance to A and to B, separable: along z from the bit columns (each column's nearest set
   // cell, 4 planes at once), then along y and along x, each within 32 cells - only in runs of 16 columns within 32 cells of open
   // cells. Exact for every distance up to 32 cells; a farther one reads as more than 32 (a column's cell farther than 32 as 33),
   // which decides nothing differently: B must lie within 32 cells to cut, and A beyond 32 loses to any B within 32
   std::vector<std::vector<std::uint32_t>> layer;
   {int zlo=INT_MAX,zhi=INT_MIN;for(std::size_t ci=0;ci<cand.size();++ci)if(openOf[ci]){zlo=std::min(zlo,cand[ci].z);zhi=std::max(zhi,cand[ci].z);}
    if(zlo<=zhi){layer.resize(std::size_t(zhi-zlo+1));for(std::size_t ci=0;ci<cand.size();++ci)if(openOf[ci])layer[std::size_t(cand[ci].z-zlo)].push_back(std::uint32_t(ci));}}
   lowpoly_pieces::eachBlock(layer.size(),1,[&](std::size_t first,std::size_t last,std::size_t chunk){FS&f=fs[std::min(chunk,fs.size()-1)];
    std::vector<float> GA,GB,HA,HB,RA,RB;std::vector<std::uint64_t> om,hm,gm;const float BIG=1e9f;
    for(std::size_t li=first;li<last;++li){const auto&L=layer[li];if(L.empty())continue;++f.joint;
     int x0=INT_MAX,x1=INT_MIN,y0=INT_MAX,y1=INT_MIN;const int z0=cand[L[0]].z*4;
     for(const auto ci:L){x0=std::min(x0,cand[ci].x*4);x1=std::max(x1,cand[ci].x*4+4);y0=std::min(y0,cand[ci].y*4);y1=std::max(y1,cand[ci].y*4+4);}
     const int X0=std::max(x0-32,V.w0[0]),X1=std::min(x1+32,V.w1[0]),Y0=std::max(y0-32,V.w0[1]),Y1=std::min(y1+32,V.w1[1]);
     const int nx=X1-X0,ny=Y1-Y0;const std::size_t P=std::size_t(nx)*std::size_t(ny);
     int CWd=16;while((nx+CWd-1)/CWd>64)CWd*=2;const int nc=(nx+CWd-1)/CWd,reach=(32+CWd-1)/CWd; // runs of CWd columns, a 64-bit mask a row
     auto dil=[&](std::uint64_t m){std::uint64_t r=m;for(int s=1;s<=reach;++s)r|=(m<<s)|(m>>s);return nc<64?r&((std::uint64_t(1)<<nc)-1):r;};
     // the runs holding open cells (om, row = plane and y in the window), the runs read along x (hm), the runs read along y (gm, per y)
     om.assign(std::size_t(4*ny),0);hm.assign(std::size_t(4*ny),0);gm.assign(std::size_t(ny),0);
     for(const auto ci:L)for(std::uint64_t m=openOf[ci];m;m&=m-1){const int c=int(lowBit64(m)),x=cand[ci].x*4+(c>>4),y=cand[ci].y*4+((c>>2)&3),l=c&3;
      om[std::size_t(l*ny+(y-Y0))]|=std::uint64_t(1)<<((x-X0)/CWd);}
     for(int l=0;l<4;++l)for(int y=0;y<ny;++y){const std::uint64_t o=om[std::size_t(l*ny+y)];if(!o)continue;const std::uint64_t h=dil(o);hm[std::size_t(l*ny+y)]=h;
      for(int yy=std::max(0,y-32);yy<=std::min(ny-1,y+32);++yy)gm[std::size_t(yy)]|=h;}
     // which way, by an estimate of each (measured: a block's search about 12 us, a column of the transform about 90 ns)
     {std::size_t gcols=0;for(int y=0;y<ny;++y)gcols+=std::bitset<64>(gm[std::size_t(y)]).count();gcols*=std::size_t(CWd);
      if(double(gcols)>edtRatio*double(L.size())){for(const auto ci:L)cutOf[ci]|=decideBlock(f,cand[ci].x*4,cand[ci].y*4,cand[ci].z*4,openOf[ci],kaOf[ci]);continue;}
      for(const auto ci:L)f.exact+=std::bitset<64>(openOf[ci]).count();}
     // along z: G[l][y][x] = (the column's nearest cell of the set along z from z0 + l, 33 at most)^2
     GA.resize(4*P+16);GB.resize(4*P+16);RA.resize(4*P+16);RB.resize(4*P+16);const int zl0=z0-V.w0[2]; // (+16: a step of 8 past a row's end)
     for(int y=0;y<ny;++y)for(std::uint64_t m=gm[std::size_t(y)];m;m&=m-1){const int k0=int(lowBit64(m))*CWd;
      for(int xi=k0;xi<std::min(nx,k0+CWd);++xi){const int x=X0+xi,yw=Y0+y;const std::size_t o=std::size_t(y)*std::size_t(nx)+std::size_t(xi);const int fl=colFlag(x,yw);int g[4];++f.cols;
       if(fl&1)near4(V.A,x,yw,zl0,g);else g[0]=g[1]=g[2]=g[3]=99;for(int l=0;l<4;++l){const int v=std::min(g[l],33);GA[std::size_t(l)*P+o]=float(v*v);}
       if(fl&2)near4(V.B,x,yw,zl0,g);else g[0]=g[1]=g[2]=g[3]=99;for(int l=0;l<4;++l){const int v=std::min(g[l],33);GB[std::size_t(l)*P+o]=float(v*v);}}}
     // along y (the runs hm of rows with open cells), then along x (the runs of their open cells): 8 cells a step (AVX), the minimum
     // kept in registers over the 65 offsets; H rows carry 32 cells of BIG at each end (a run never reads past them)
     const int hw=nx+64+16;HA.resize(std::size_t(4*ny)*std::size_t(hw));HB.resize(std::size_t(4*ny)*std::size_t(hw));
     const __m256 vBig=_mm256_set1_ps(BIG);
     for(int l=0;l<4;++l)for(int y=0;y<ny;++y){const std::size_t r=std::size_t(l*ny+y);const std::uint64_t h=hm[r];if(!h)continue;++f.tiles;
      float*ha=&HA[r*std::size_t(hw)]+32,*hb=&HB[r*std::size_t(hw)]+32;
      for(std::uint64_t m=h;m;m&=m-1){const int k0=int(lowBit64(m))*CWd,k1=std::min(nx,k0+CWd);
       for(int x=k0;x<k1;x+=8){__m256 ma=vBig,mb=vBig;
        for(int dy=std::max(-32,-y);dy<=std::min(32,ny-1-y);++dy){const __m256 add=_mm256_set1_ps(float(dy*dy));const std::size_t g=std::size_t(l)*P+std::size_t(y+dy)*std::size_t(nx)+std::size_t(x);
         ma=_mm256_min_ps(ma,_mm256_add_ps(_mm256_loadu_ps(&GA[g]),add));mb=_mm256_min_ps(mb,_mm256_add_ps(_mm256_loadu_ps(&GB[g]),add));}
        _mm256_storeu_ps(ha+x,ma);_mm256_storeu_ps(hb+x,mb);}}
      for(int x=-32;x<0;++x){ha[x]=BIG;hb[x]=BIG;}for(int x=nx;x<nx+32+16;++x){ha[x]=BIG;hb[x]=BIG;} // (after the runs: a last run's step past nx)
      float*ra=&RA[std::size_t(l)*P+std::size_t(y)*std::size_t(nx)],*rb2=&RB[std::size_t(l)*P+std::size_t(y)*std::size_t(nx)];
      for(std::uint64_t m=om[r];m;m&=m-1){const int k0=int(lowBit64(m))*CWd,k1=std::min(nx,k0+CWd);
       for(int x=k0;x<k1;x+=8){__m256 ma=vBig,mb=vBig;
        for(int dx=-32;dx<=32;++dx){const __m256 add=_mm256_set1_ps(float(dx*dx));
         ma=_mm256_min_ps(ma,_mm256_add_ps(_mm256_loadu_ps(ha+x+dx),add));mb=_mm256_min_ps(mb,_mm256_add_ps(_mm256_loadu_ps(hb+x+dx),add));}
        _mm256_storeu_ps(ra+x,ma);_mm256_storeu_ps(rb2+x,mb);}}}
     // one compare a cell: B strictly nearer than A and within 32 cells
     for(const auto ci:L){std::uint64_t cut=0;
      for(std::uint64_t m=openOf[ci];m;m&=m-1){const int c=int(lowBit64(m)),x=cand[ci].x*4+(c>>4),y=cand[ci].y*4+((c>>2)&3),l=c&3;
       const std::size_t o=std::size_t(l)*P+std::size_t(y-Y0)*std::size_t(nx)+std::size_t(x-X0);if(RB[o]<RA[o]&&RB[o]<=1024.f)cut|=std::uint64_t(1)<<c;}
      cutOf[ci]|=cut;}}});
   msJoint=core::ms(p0);for(const FS&f:fs){exact+=f.exact;joint+=f.joint;visJ+=f.cols;tilesJ+=f.tiles;}
   for(std::size_t ci=0;ci<cand.size();++ci){std::uint64_t cut=cutOf[ci];if(!cut)continue;const int x0=cand[ci].x*4,y0=cand[ci].y*4,z0=cand[ci].z*4;const Cell tc{x0>>3,y0>>3,z0>>3};
    if(!check){auto it=marks.find(tc);if(it!=marks.end())cut&=~tileBlock(it->second,x0,y0,z0);if(!cut)continue;} // (hidden there already: nothing to add)
    if(!noZone<InZone>())for(std::uint64_t m=cut;m;m&=m-1){const int c=int(lowBit64(m));const double pq[3]={grid.origin.x+(x0+(c>>4)+.5)*grid.pitch,grid.origin.y+(y0+((c>>2)&3)+.5)*grid.pitch,grid.origin.z+(z0+(c&3)+.5)*grid.pitch};
     if(inZone(pq))cut&=~(std::uint64_t(1)<<c);}
    if(!cut)continue;
    auto&tile=marks[tc];for(int lx=0;lx<4;++lx)for(int ly=0;ly<4;++ly){const std::uint64_t nib=(cut>>(lx*16+ly*4))&0xFu;if(nib)tile.removed[std::size_t((x0&7)+lx)]|=nib<<(((y0&7)+ly)*8+(z0&7));}
    made+=std::bitset<64>(cut).count();if(check)for(std::uint64_t m=cut;m;m&=m-1){const int c=int(lowBit64(m));mineC.push_back(Cell{x0+(c>>4),y0+((c>>2)&3),z0+(c&3)});}}}
  const double fineMs=core::ms(f0);
  if(std::getenv("GFY_M3_LOG")||std::getenv("GFY_PIECE_LOG"))std::cerr<<"blockfine piece="<<k<<" box=("<<rb.lo[0]<<","<<rb.lo[1]<<","<<rb.lo[2]<<")-("<<rb.hi[0]<<","<<rb.hi[1]<<","<<rb.hi[2]
   <<") cand_blocks="<<cand.size()<<" window_words="<<words<<" cells="<<made<<" exact="<<exact<<" blocks_exact="<<joint<<" gone_cells="<<goneC.size()<<" ms="<<fineMs
   <<" (before "<<msBefore<<" pre "<<msPre<<" build "<<msBuild<<" decide "<<msJoint<<" columns "<<visJ<<" tiles "<<tilesJ<<")"<<std::endl;
  // GFY_HB_CHECK (probes): every cell of the box again, by brute force - no blocks, no bounds: 3D offsets in order of distance, the
  // first distance with any anatomy decides (A there: keep; B alone, within 32 cells: cut)
  if(check){const auto k0=core::Clock::now();CV W;const int blo[3]={rb.lo[0],rb.lo[1],rb.lo[2]},bhi[3]={rb.hi[0],rb.hi[1],rb.hi[2]};
   if(build(W,blo,bhi,33)){
    static const std::vector<std::array<int,4>> offs=[]{std::vector<std::array<int,4>> v;for(int dz=-32;dz<=32;++dz)for(int dy=-32;dy<=32;++dy)for(int dx=-32;dx<=32;++dx){
      const int d2=dx*dx+dy*dy+dz*dz;if(d2<=1024)v.push_back({{dx,dy,dz,d2}});}std::stable_sort(v.begin(),v.end(),[](const std::array<int,4>&a,const std::array<int,4>&b){return a[3]<b[3];});return v;}();
    std::vector<Cell> brute;std::size_t judged=0;Cell bT{INT_MIN,INT_MIN,INT_MIN};const core::Tile*bTile=nullptr;
    for(int z=std::max(rb.lo[2],0);z<std::min(rb.hi[2],sz[2]);++z)for(int y=std::max(rb.lo[1],0);y<std::min(rb.hi[1],sz[1]);++y)for(int x=std::max(rb.lo[0],0);x<std::min(rb.hi[0],sz[0]);++x){
     if(W.has(W.A,x,y,z))continue;bool cut=false;
     if(W.has(W.O,x,y,z))cut=true;
     else{if(W.has(W.B,x,y,z))continue;
      {const Cell tk{x>>3,y>>3,z>>3};if(!(tk==bT)){bT=tk;const auto it=grid.tiles.find(tk);bTile=it==grid.tiles.end()?nullptr:&it->second;}
       if(bTile&&((bTile->removed[std::size_t(x&7)]>>((y&7)*8+(z&7)))&1u))continue;}
      if(!material(W,x,y,z))continue;++judged;
      bool hA=false,hB=false;int d=-1;
      for(const auto&o:offs){if(o[3]!=d){if(hA||hB)break;d=o[3];}if(W.has(W.A,x+o[0],y+o[1],z+o[2]))hA=true;if(W.has(W.B,x+o[0],y+o[1],z+o[2]))hB=true;}
      cut=hB&&!hA;}
     if(!cut)continue;const double pq[3]={grid.origin.x+(x+.5)*grid.pitch,grid.origin.y+(y+.5)*grid.pitch,grid.origin.z+(z+.5)*grid.pitch};if(inZone(pq))continue;brute.push_back(Cell{x,y,z});}
    auto less=[](const Cell&a,const Cell&b){return a.x!=b.x?a.x<b.x:(a.y!=b.y?a.y<b.y:a.z<b.z);};std::sort(brute.begin(),brute.end(),less);std::sort(mineC.begin(),mineC.end(),less);
    std::size_t onlyFast=0,onlyBrute=0;{std::size_t i=0,j=0;while(i<mineC.size()||j<brute.size()){if(j>=brute.size()||(i<mineC.size()&&less(mineC[i],brute[j]))){++onlyFast;++i;}
      else if(i>=mineC.size()||less(brute[j],mineC[i])){++onlyBrute;++j;}else{++i;++j;}}}
    std::cerr<<"blockfine_check piece="<<k<<" judged="<<judged<<" cut_fast="<<mineC.size()<<" cut_brute="<<brute.size()<<" only_fast="<<onlyFast<<" only_brute="<<onlyBrute<<" differ="<<(onlyFast+onlyBrute)
     <<" ms="<<core::ms(k0)<<std::endl;}
   else std::cerr<<"blockfine_check piece="<<k<<" skipped (window)"<<std::endl;}
 }
 // ---- THE FILL (bonefill; console painter_island_fill - the worker's copy, LiveDamageWorker.cpp): rows (x,y) with one row of
 // margin, z along the bits; a worklist of 64-cell words
 if(::gfy_live::island_fill_value()!=0){const auto i0=core::Clock::now();
  // the sever on this piece: its blocks within one block of another piece's anatomy or of removed tissue; the fill's box: 16 cells out
  CellBox fb;{std::vector<std::uint64_t> D=Ot;dilate(D);
   for(int bz=box.lo[2]>>2;bz<(box.hi[2]>>2);++bz)for(int by=box.lo[1]>>2;by<(box.hi[1]>>2);++by){const std::size_t r=(std::size_t(bz-b0[2])*n1+std::size_t(by-b0[1]))*wpr;
    for(std::size_t i=0;i<wpr;++i)for(std::uint64_t m=D[r+i]&O0[r+i];m;m&=m-1){const int bx=b0[0]+int(i*64+lowBit64(m));if(bx<(box.lo[0]>>2)||bx>=(box.hi[0]>>2))continue;
     fb.add(bx*4,by*4,bz*4,bz*4+4);fb.add(bx*4+3,by*4+3,bz*4,bz*4+4);}}}
  if(fb.valid())for(int a=0;a<3;++a){fb.lo[a]=std::max(fb.lo[a]-16,box.lo[a]);fb.hi[a]=std::min(std::min(fb.hi[a]+16,box.hi[a]),sz[a]);}
  const bool fok=fb.valid();const int fx=fok?fb.hi[0]-fb.lo[0]+2:0,fy=fok?fb.hi[1]-fb.lo[1]+2:0,fz=fok?fb.hi[2]-fb.lo[2]:0;
  const std::size_t fw=std::size_t(fz+63)/64,FW=std::size_t(fx)*std::size_t(fy)*fw;
  if(FW&&FW<=(std::size_t(1)<<17)){
   // A the piece's own anatomy, Oc the other pieces', Fl flesh that may float (no piece's; given back by a burst), F what the walk may enter
   std::vector<std::uint64_t> A(FW,0),Oc(FW,0),Fl(FW,0),F(FW,0),Rr(FW,0),P(FW,0),Tm(FW,0);
   auto rowOf=[&](int x,int y){return (std::size_t(y-fb.lo[1]+1)*std::size_t(fx)+std::size_t(x-fb.lo[0]+1))*fw;};
   auto setRange=[&](std::vector<std::uint64_t>&v,std::size_t r,int z0,int z1){for(int a=z0-fb.lo[2],b=z1-fb.lo[2];a<b;){const int s=a&63,n=std::min(64-s,b-a);
     v[r+std::size_t(a>>6)]|=(n==64?~std::uint64_t(0):((std::uint64_t(1)<<n)-1))<<s;a+=n;}};
   for(int y=fb.lo[1];y<fb.hi[1];++y){auto col=std::lower_bound(volume.columns.begin(),volume.columns.end(),std::array<int,2>{{fb.lo[0],y}},
     [](const gfy_connectivity::Column&a,const std::array<int,2>&q){return gfy_connectivity::xyLess(a.x,a.y,q[0],q[1]);});
    for(;col!=volume.columns.end()&&col->y==y&&col->x<fb.hi[0];++col){const std::size_t r=rowOf(col->x,y);
     for(auto s=col->first;s<col->first+col->count;++s){const auto&sp=volume.spans[s];const int lo=std::max<int>(sp.lo,fb.lo[2]),hi=std::min<int>(sp.hi,fb.hi[2]);if(lo>=hi)continue;
      const int j=sp.component<pieceOf.size()?pieceOf[sp.component]:-1;setRange(j<0?Fl:(std::size_t(j)==k?A:Oc),r,lo,hi);}}}
   auto tileBits=[&](std::vector<std::uint64_t>&v,const core::Tile&t,const Cell&tk,bool clear){const int zb=tk.z*8-fb.lo[2]; // the tile's first z as a bit; 8 cells of z a step
    for(int lx=0;lx<8;++lx){const std::uint64_t wd=t.removed[std::size_t(lx)];const int x=tk.x*8+lx;if(!wd||x<fb.lo[0]||x>=fb.hi[0])continue;
     for(int ly=0;ly<8;++ly){std::uint64_t by8=(wd>>(ly*8))&0xFFu;const int y=tk.y*8+ly;if(!by8||y<fb.lo[1]||y>=fb.hi[1])continue;const std::size_t r=rowOf(x,y);
      int z0=zb;if(z0<0){by8>>=(-z0);z0=0;}if(z0+8>fz)by8&=(std::uint64_t(1)<<std::max(0,fz-z0))-1; // the bits below and above the box
      if(!by8)continue;const std::size_t w=std::size_t(z0>>6);const int s=z0&63;
      if(clear){v[r+w]&=~(by8<<s);if(s>56&&w+1<fw)v[r+w+1]&=~(by8>>(64-s));}else{v[r+w]|=by8<<s;if(s>56&&w+1<fw)v[r+w+1]|=by8>>(64-s);}}}};
   auto meets=[&](const Cell&tk){return tk.x*8+8>fb.lo[0]&&tk.x*8<fb.hi[0]&&tk.y*8+8>fb.lo[1]&&tk.y*8<fb.hi[1]&&tk.z*8+8>fb.lo[2]&&tk.z*8<fb.hi[2];};
   // the material: gear cells, clothing padding (blockfine: every cell's flag, once per tile of the separation), the cells under the piece's triangles
   for(int tz=fb.lo[2]>>3;tz<=((fb.hi[2]-1)>>3);++tz)for(int ty=fb.lo[1]>>3;ty<=((fb.hi[1]-1)>>3);++ty)for(int tx=fb.lo[0]>>3;tx<=((fb.hi[0]-1)>>3);++tx){const Cell tk{tx,ty,tz};
    const std::size_t ti=(std::size_t(tz)*std::size_t(tny)+std::size_t(ty))*std::size_t(tnx)+std::size_t(tx);
    if(gear&&(ti>=gear->tileFlag.size()||gear->tileFlag[ti]))if(const core::Tile*g=gear->tile(tx,ty,tz))tileBits(F,*g,tk,false);
    if(padF&&tf&&ti<padF->size()&&(*padF)[ti]!=0){auto it=padCache.find(tk);
     if(it==padCache.end()){core::Tile pt{};
      if(!blockRule){for(int c=0;c<512;++c){const int lx=c>>6,ly=(c>>3)&7,lz=c&7; // (blockfine: the model's padding flag, cell by cell)
        if(tf->sample(grid.origin.x+(tx*8+lx+.5)*grid.pitch,grid.origin.y+(ty*8+ly+.5)*grid.pitch,grid.origin.z+(tz*8+lz+.5)*grid.pitch)&64u)pt.removed[std::size_t(lx)]|=std::uint64_t(1)<<(ly*8+lz);}}
      else for(int q=0;q<8;++q){const int ox=(q&1)*4,oy=((q>>1)&1)*4,oz=(q>>2)*4;
       if(!(tf->sample(grid.origin.x+(tx*8+ox+2)*grid.pitch,grid.origin.y+(ty*8+oy+2)*grid.pitch,grid.origin.z+(tz*8+oz+2)*grid.pitch)&64u))continue;
       std::uint64_t m=0;for(int ly=0;ly<4;++ly)m|=std::uint64_t(0xF)<<((oy+ly)*8+oz);for(int lx=0;lx<4;++lx)pt.removed[std::size_t(ox+lx)]|=m;}
      it=padCache.emplace(tk,pt).first;++st.padTiles;}
     tileBits(F,it->second,tk,false);}}
   for(const Cell&c:shell){if(c.x<fb.lo[0]||c.x>=fb.hi[0]||c.y<fb.lo[1]||c.y>=fb.hi[1]||c.z<fb.lo[2]||c.z>=fb.hi[2])continue;const int bit=c.z-fb.lo[2];F[rowOf(c.x,c.y)+std::size_t(bit>>6)]|=std::uint64_t(1)<<(bit&63);}
   // ... no piece's anatomy; flesh that is no piece's; not removed in the grid - but the flesh a burst gave back; not marked on this
   // piece (the block rule's marks, its parent's)
   for(std::size_t i=0;i<FW;++i)F[i]=(F[i]&~(A[i]|Oc[i]))|Fl[i];
   for(const auto&g:grid.tiles)if(meets(g.first))tileBits(F,g.second,g.first,true);
   if(given)for(const auto&g:*given)if(meets(g.first)){tileBits(F,g.second,g.first,false);tileBits(Fl,g.second,g.first,false);}
   for(const auto&m:marks)if(meets(m.first))tileBits(F,m.second,m.first,true);
   if(inherited)for(const auto&m:*inherited)if(meets(m.first))tileBits(F,m.second,m.first,true);
   for(std::size_t i=0;i<FW;++i)F[i]&=~(A[i]|Oc[i]);
   // the piece's own anatomy grown by 2 cells (26 neighbours a step: along z in the words, then the rows beside in x, then in y; the
   // margin rows stay empty)
   auto grow=[&](std::vector<std::uint64_t>&v){
    for(int yi=1;yi<fy-1;++yi)for(int xi=1;xi<fx-1;++xi){const std::size_t r=(std::size_t(yi)*std::size_t(fx)+std::size_t(xi))*fw;
     for(std::size_t w=0;w<fw;++w){const std::uint64_t c=v[r+w];std::uint64_t o=c|(c<<1)|(c>>1);if(w)o|=v[r+w-1]>>63;if(w+1<fw)o|=v[r+w+1]<<63;Tm[r+w]=o;}}
    for(int yi=1;yi<fy-1;++yi)for(int xi=1;xi<fx-1;++xi){const std::size_t r=(std::size_t(yi)*std::size_t(fx)+std::size_t(xi))*fw;for(std::size_t w=0;w<fw;++w)v[r+w]=Tm[r+w]|Tm[r-fw+w]|Tm[r+fw+w];}
    const std::size_t up=std::size_t(fx)*fw;
    for(int yi=1;yi<fy-1;++yi)for(int xi=1;xi<fx-1;++xi){const std::size_t r=(std::size_t(yi)*std::size_t(fx)+std::size_t(xi))*fw;for(std::size_t w=0;w<fw;++w)Tm[r+w]=v[r+w]|v[r-up+w]|v[r+up+w];}
    v.swap(Tm);};
   std::vector<std::uint64_t> DA=A;grow(DA);grow(DA);
   // a word's new cells: the run along z filled inside the word, the word queued
   std::vector<std::uint32_t> wq;
   auto reach=[&](std::size_t i,std::uint64_t add){const std::uint64_t m=F[i]&~Rr[i];for(;;){const std::uint64_t y=add|(((add<<1)|(add>>1))&m);if(y==add)break;add=y;}
    Rr[i]|=add;if(!P[i])wq.push_back(std::uint32_t(i));P[i]|=add;};
   // the seeds: material within 2 cells of the piece's own anatomy; material on the box's faces whose block is not another's
   std::vector<std::uint64_t> zm(fw,0);int zmx=INT_MIN,zmy=INT_MIN;const std::size_t topW=std::size_t((fz-1)>>6);const std::uint64_t topBit=std::uint64_t(1)<<((fz-1)&63);
   for(int yi=1;yi<fy-1;++yi)for(int xi=1;xi<fx-1;++xi){const std::size_t r=(std::size_t(yi)*std::size_t(fx)+std::size_t(xi))*fw;const bool edgeRow=xi==1||xi==fx-2||yi==1||yi==fy-2;
    const int x=fb.lo[0]+xi-1,y=fb.lo[1]+yi-1;
    for(std::size_t w=0;w<fw;++w){const std::uint64_t f=F[r+w];if(!f)continue;std::uint64_t s=f&DA[r+w],e=edgeRow?f:std::uint64_t(0);
     if(w==0)e|=f&1u;if(w==topW)e|=f&topBit;e&=~s;
     if(e){if((x>>2)!=zmx||(y>>2)!=zmy){zmx=x>>2;zmy=y>>2;std::fill(zm.begin(),zm.end(),std::uint64_t(0));
       for(int z=fb.lo[2];z<fb.hi[2];z+=4){const int bz=z>>2;if(inW(zmx,zmy,bz)&&(Un[word(zmx,zmy,bz)]&bitOf(zmx))!=0)continue;const int b=z-fb.lo[2];zm[std::size_t(b>>6)]|=std::uint64_t(0xF)<<(b&63);}}
      s|=e&zm[w];}
     if(s)reach(r+w,s);}}
   // the walk: a word's new cells go to the 9 rows round it (their cells at the same z and one beside) and to the words beside along z
   for(std::size_t h=0;h<wq.size();++h){const std::size_t i=wq[h];const std::uint64_t N=P[i];P[i]=0;if(!N)continue;const std::size_t ri=i/fw,w=i%fw;
    const std::uint64_t S=N|(N<<1)|(N>>1);const bool dn=(N&1u)!=0&&w>0,upw=(N>>63)!=0&&w+1<fw;
    for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){const std::size_t j=std::size_t(std::ptrdiff_t(ri)+dy*fx+dx)*fw+w;
     std::uint64_t a=S&F[j]&~Rr[j];if(a)reach(j,a);
     if(dn){a=(std::uint64_t(1)<<63)&F[j-1]&~Rr[j-1];if(a)reach(j-1,a);}
     if(upw){a=std::uint64_t(1)&F[j+1]&~Rr[j+1];if(a)reach(j+1,a);}}}
   // what the walk did not reach: each such cell and the cells round it (26) that are not the piece's own anatomy and not reached
   // are marked (a triangle crossing a cell's corner is not left as a dot)
   std::size_t isl=0,flesh=0;Cell lastB{INT_MIN,INT_MIN,INT_MIN};bool lastIn=false;
   for(int yi=1;yi<fy-1;++yi)for(int xi=1;xi<fx-1;++xi){const std::size_t r=(std::size_t(yi)*std::size_t(fx)+std::size_t(xi))*fw;
    for(std::size_t w=0;w<fw;++w)for(std::uint64_t m=F[r+w]&~Rr[r+w];m;m&=m-1){const int b=int(w*64+lowBit64(m));if(b>=fz)continue;++isl;if((Fl[r+w]>>(b&63))&1u)++flesh;
     for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){const int xi2=xi+dx,yi2=yi+dy;if(xi2<1||xi2>fx-2||yi2<1||yi2>fy-2)continue;const std::size_t r2=(std::size_t(yi2)*std::size_t(fx)+std::size_t(xi2))*fw;
      for(int dz=-1;dz<=1;++dz){const int b2=b+dz;if(b2<0||b2>=fz)continue;const std::size_t i2=r2+std::size_t(b2>>6);if(((Rr[i2]|A[i2])>>(b2&63))&1u)continue;
       const int x=fb.lo[0]+xi2-1,y=fb.lo[1]+yi2-1,z=fb.lo[2]+b2;const Cell bk=blockRule?Cell{x>>2,y>>2,z>>2}:Cell{x,y,z};const double bs=blockRule?4.:1.,bh=blockRule?2.:.5; // (blockfine: a kept tear's zone cell by cell)
       if(!(bk==lastB)){lastB=bk;const double pq[3]={grid.origin.x+(bk.x*bs+bh)*grid.pitch,grid.origin.y+(bk.y*bs+bh)*grid.pitch,grid.origin.z+(bk.z*bs+bh)*grid.pitch};lastIn=inZone(pq);}
       if(lastIn)continue;auto&tile=marks[Cell{x>>3,y>>3,z>>3}];tile.removed[std::size_t(x&7)]|=std::uint64_t(1)<<((y&7)*8+(z&7));}}}}
   st.islands+=isl;st.medium+=wq.size();st.shell+=shell.size();
   if(isl&&(std::getenv("GFY_M3_LOG")||std::getenv("GFY_PIECE_LOG")))std::cerr<<"bone_fill piece="<<k<<" unreached="<<isl<<" flesh="<<flesh<<" words="<<wq.size()
     <<" box=("<<fb.lo[0]<<","<<fb.lo[1]<<","<<fb.lo[2]<<")-("<<fb.hi[0]<<","<<fb.hi[1]<<","<<fb.hi[2]<<")"<<std::endl;}
  else if(FW)++st.skipped;
  st.islandMs+=core::ms(i0);}
 ++st.pieces;st.marked+=made;st.ms+=core::ms(t0);}
// headfix (2026-11-13, the owner at the seam pictures: thin slivers of clothing standing over a split's face): CLOTHING JUST
// OUTSIDE A TEAR'S FOOTPRINT TAKES THE TEAR'S SIDE. A decide-once tear acts only inside its footprint (the region flag of its baked
// map: the body's section in the tear plane); a jacket's hem a few millimetres outside that outline is no cell of the section, the
// tear says nothing there, and cutfix's owners() hands it to the piece whose tissue reaches it first through the clothing - so a
// strip of it stood on the wrong piece, past the torn face (measured on the waist split SW: the sliver pixels lay 0-13 mm outside
// the footprint and up to 41 mm past the tear surface on the far side; a trace of one such cell: its CENTRE lay in the footprint's
// edge map cell - the marks left it to the tear - while the pixel, one 4 mm map cell out, found no footprint and was kept).
// Rule, decided once at the cut: a cell on the footprint's EDGE (a footprint map cell with a neighbour outside it) or outside the
// footprint within 12 mm of it (the map's cells, dilated), within 64 mm of the plane, on the FAR side of the tear surface (the
// nearest footprint cell's offset), whose nearest tissue lies inside the footprint (so it hangs on the torn section, not on
// another limb passing by: tearNear), is hidden on the piece - a mark in its removal field (all cells there: solid). Only a
// cell safely inside a fresh tear's footprint (no edge cell) is left to the tear, as before.
// each tile's nearest tissue tile around the fresh tears (their region boxes + 6 cm): two raster passes of vector propagation
struct TearNear {bool ok=false;int lo[3]{},n[3]{};std::vector<std::int32_t> nearS;double ms=0;
 bool at(int tx,int ty,int tz,int s[3])const{if(!ok)return false;tx-=lo[0];ty-=lo[1];tz-=lo[2];if(tx<0||ty<0||tz<0||tx>=n[0]||ty>=n[1]||tz>=n[2])return false;
  const std::int32_t v=nearS[(std::size_t(tz)*std::size_t(n[1])+std::size_t(ty))*std::size_t(n[0])+std::size_t(tx)];if(v<0)return false;
  s[0]=(v&1023)+lo[0];s[1]=((v>>10)&1023)+lo[1];s[2]=(v>>20)+lo[2];return true;}};
inline TearNear tearNear(const gfy_packed::Mesh&published,const Grid&grid,const gfy_connectivity::Topology&volume,const PieceAt&pieceAt){
 TearNear out;if(std::getenv("GFY_HB_NO_TEAR_RING"))return out;const auto t0=core::Clock::now();
 const int tn[3]={(grid.size.x+7)/8,(grid.size.y+7)/8,(grid.size.z+7)/8};
 int lo[3]={INT_MAX,INT_MAX,INT_MAX},hi[3]={INT_MIN,INT_MIN,INT_MIN};bool any=false;
 for(const auto&tr:published.tears){if(!fresh(tr)||tear_atlas::burstRecord(*tr.block))continue;const auto z=tear_atlas::zone(*tr.block);if(!z.ok||!z.region.valid())continue;any=true;
  for(int a=0;a<3;++a){lo[a]=std::min(lo[a],int(std::floor((z.region.lo[a]-.06-grid.origin[a])/grid.pitch))>>3);hi[a]=std::max(hi[a],int(std::floor((z.region.hi[a]+.06-grid.origin[a])/grid.pitch))>>3);}}
 if(!any)return out;
 for(int a=0;a<3;++a){lo[a]=std::max(lo[a],0);hi[a]=std::min(hi[a],tn[a]-1);any=any&&lo[a]<=hi[a]&&hi[a]-lo[a]<1023;}
 if(!any)return out;
 const std::size_t n0=std::size_t(hi[0]-lo[0]+1),n1=std::size_t(hi[1]-lo[1]+1),n2=std::size_t(hi[2]-lo[2]+1),N=n0*n1*n2;if(!N||N>=(std::size_t(1)<<23))return out;
 auto rIdx=[&](int x,int y,int z){return (std::size_t(z)*n1+std::size_t(y))*n0+std::size_t(x);};
 out.nearS.assign(N,-1);std::vector<std::int32_t> best(N,INT_MAX);std::size_t seeds=0;
 for(const auto&col:volume.columns){const int tx=(col.x>>3)-lo[0],ty=(col.y>>3)-lo[1];if(col.x<0||col.y<0||tx<0||ty<0||tx>=int(n0)||ty>=int(n1))continue;
  for(auto si=col.first;si<col.first+col.count;++si){const auto&sp=volume.spans[si];if(sp.component>=pieceAt.pieceOf.size()||sp.hi<=sp.lo||pieceAt.pieceOf[sp.component]<0)continue;
   const int z0t=std::max((sp.lo>>3)-lo[2],0),z1t=std::min(((sp.hi-1)>>3)-lo[2],int(n2)-1);
   for(int tz=z0t;tz<=z1t;++tz){const std::size_t i=rIdx(tx,ty,tz);if(best[i]){best[i]=0;out.nearS[i]=std::int32_t(tx|(ty<<10)|(tz<<20));++seeds;}}}}
 if(!seeds)return out;
 static const int fw[13][3]={{-1,-1,-1},{0,-1,-1},{1,-1,-1},{-1,0,-1},{0,0,-1},{1,0,-1},{-1,1,-1},{0,1,-1},{1,1,-1},{-1,-1,0},{0,-1,0},{1,-1,0},{-1,0,0}};
 for(int pass=0;pass<2;++pass){const int sgn=pass?-1:1;
  for(int zz=0;zz<int(n2);++zz){const int z=pass?int(n2)-1-zz:zz;for(int yy=0;yy<int(n1);++yy){const int y=pass?int(n1)-1-yy:yy;for(int xx=0;xx<int(n0);++xx){const int x=pass?int(n0)-1-xx:xx;
     const std::size_t i=rIdx(x,y,z);if(!best[i])continue;
     for(const auto&d:fw){const int X=x+sgn*d[0],Y=y+sgn*d[1],Z=z+sgn*d[2];if(X<0||Y<0||Z<0||X>=int(n0)||Y>=int(n1)||Z>=int(n2))continue;
      const std::int32_t s=out.nearS[rIdx(X,Y,Z)];if(s<0)continue;const int ax=x-(s&1023),ay=y-((s>>10)&1023),az=z-(s>>20);const int dd=ax*ax+ay*ay+az*az;
      if(dd<best[i]){best[i]=dd;out.nearS[i]=s;}}}}}}
 for(int a=0;a<3;++a)out.lo[a]=lo[a];out.n[0]=int(n0);out.n[1]=int(n1);out.n[2]=int(n2);out.ok=true;out.ms=core::ms(t0);return out;}
// one fresh tear's ring: the map cells outside its footprint within 2 cm of it, each with the offset of the footprint cell it grew from
struct TearRing {const gfy_packed::TearBlock*b=nullptr;tear_atlas::Zone z;int mx=0,my=0;std::vector<std::uint8_t> foot,ring;std::vector<float> off;
 // (pa, pb) of the tear's frame: 1 = safely in the footprint, 3 = on its edge, 2 = in the ring outside it (off: the tear surface's
 // offset there), 0 = neither
 int at(double pa,double pb,double&o)const{const double fa=(pa-z.a2)/z.h2,fb=(pb-z.b2)/z.h2;const int i=int(std::floor(fa)),j=int(std::floor(fb));
  if(i>=0&&j>=0&&i<mx&&j<my){const std::size_t q=std::size_t(j)*mx+i;o=off[q];return foot[q]?int(foot[q]):(ring[q]?2:0);}
  // beyond the map's edge: the ring of a footprint that reaches the edge
  const int ic=std::min(std::max(i,0),mx-1),jc=std::min(std::max(j,0),my-1);const std::size_t q=std::size_t(jc)*mx+ic;if(!foot[q])return 0;
  const double da=fa<0?-fa:(fa>mx?fa-mx:0.),db=fb<0?-fb:(fb>my?fb-my:0.);if((da*da+db*db)*z.h2*z.h2>.012*.012)return 0;o=off[q];return 2;}};
inline TearRing tearRing(const gfy_packed::TearBlock&b){TearRing r;r.b=&b;r.z=tear_atlas::zone(b);if(!r.z.ok||!(r.z.h2>0))return r;r.mx=int(r.z.mx);r.my=int(r.z.my);
 const std::size_t n=std::size_t(r.mx)*r.my;r.foot.assign(n,0);r.ring.assign(n,0);r.off.assign(n,0.f);
 for(std::size_t q=0;q<n;++q){r.foot[q]=(int(b.map[q*20+1]+.5f)&5)?1:0; /* damagecut: 4 = a bridge tear's footprint */r.off[q]=b.map[q*20];}
 std::vector<std::uint8_t> cur=r.foot,next;const int steps=std::max(1,int(std::ceil(.012/r.z.h2)));static const int o8[8][2]={{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
 for(int s=0;s<steps;++s){next=cur;
  for(int j=0;j<r.my;++j)for(int i=0;i<r.mx;++i){const std::size_t q=std::size_t(j)*r.mx+i;if(cur[q])continue;
   for(const auto&o:o8){const int ii=i+o[0],jj=j+o[1];if(ii<0||jj<0||ii>=r.mx||jj>=r.my)continue;const std::size_t qq=std::size_t(jj)*r.mx+ii;
    if(cur[qq]){next[q]=1;r.ring[q]=1;r.off[q]=r.off[qq];break;}}}
  cur.swap(next);}
 // the footprint's edge cells (a neighbour inside the map that is no footprint): 3
 for(int j=0;j<r.my;++j)for(int i=0;i<r.mx;++i){const std::size_t q=std::size_t(j)*r.mx+i;if(!r.foot[q])continue;
  for(const auto&o:o8){const int ii=i+o[0],jj=j+o[1];if(ii<0||jj<0||ii>=r.mx||jj>=r.my)continue;if(!r.foot[std::size_t(jj)*r.mx+ii]){r.foot[q]=3;break;}}}
 return r;}
// the cells the piece (on side `side` of the tear) hides by the rule, OR-ed into `mine` (tile -> bits); all: the rings of the
// piece's fresh tears (a cell safely inside one of their footprints is that tear's to decide). Returns the cells marked.
template<class Mine>
inline std::size_t tearRingMarks(const TearRing&r,unsigned side,const Grid&grid,const CellBox&drawn,const TearNear&nz,Mine&mine,const std::vector<const TearRing*>&all){
 const auto&z=r.z;if(!z.ok||side>1||r.ring.empty()||!nz.ok||!drawn.valid()||!z.region.valid())return 0;
 const double p=grid.pitch,hd=.87*8*p,reach=std::min(z.slab,.064),oMax=tear_decide::offMax;std::size_t marked=0;
 int t0[3],t1[3];for(int a=0;a<3;++a){const int c0=std::max(drawn.lo[a],int(std::floor((z.region.lo[a]-.02-grid.origin[a])/p))),c1=std::min(drawn.hi[a]-1,int(std::floor((z.region.hi[a]+.02-grid.origin[a])/p)));
  if(c0>c1)return 0;t0[a]=std::max(c0,0)>>3;t1[a]=c1>>3;}
 auto frame=[&](double x,double y,double zc,double&s0,double&pa,double&pb){const double d[3]={x-z.C[0],y-z.C[1],zc-z.C[2]};
  s0=d[0]*z.n[0]+d[1]*z.n[1]+d[2]*z.n[2];pa=d[0]*z.u[0]+d[1]*z.u[1]+d[2]*z.u[2];pb=d[0]*z.w[0]+d[1]*z.w[1]+d[2]*z.w[2];};
 // (parallel over the tile slabs: each thread its own list, merged below)
 std::vector<std::vector<std::pair<Cell,core::Tile>>> part(core::parallelWidth()+1);std::vector<std::size_t> count(part.size(),0);
 lowpoly_pieces::eachBlock(std::size_t(t1[2]-t0[2]+1),1,[&](std::size_t zf,std::size_t zl,std::size_t thread){const std::size_t th=std::min(thread,part.size()-1);auto&list=part[th];std::size_t made=0;
 for(std::size_t zi=zf;zi<zl;++zi){const int tz=t0[2]+int(zi);for(int ty=t0[1];ty<=t1[1];++ty)for(int tx=t0[0];tx<=t1[0];++tx){
  double s0,pa,pb;frame(grid.origin.x+(tx*8+4)*p,grid.origin.y+(ty*8+4)*p,grid.origin.z+(tz*8+4)*p,s0,pa,pb);
  if(std::abs(s0)>reach+hd||pa*pa+pb*pb>(z.Rl+hd)*(z.Rl+hd))continue;
  if(side==0?s0<-oMax-hd:s0>oMax+hd)continue; // the whole tile on the piece's own side of the tear surface
  // any ring cell under the tile?
  {bool any=false;const int i0=int(std::floor((pa-hd-z.a2)/z.h2)),i1=int(std::floor((pa+hd-z.a2)/z.h2)),j0=int(std::floor((pb-hd-z.b2)/z.h2)),j1=int(std::floor((pb+hd-z.b2)/z.h2));
   for(int j=j0;j<=j1&&!any;++j)for(int i=i0;i<=i1&&!any;++i){double o;any=r.at(z.a2+(i+.5)*z.h2,z.b2+(j+.5)*z.h2,o)>=2;}
   if(!any)continue;}
  // the tile's nearest tissue tile lies in the footprint (its centre or a corner): the clothing here hangs on the torn section
  {int s[3];if(!nz.at(tx,ty,tz,s))continue;bool in=false;
   for(int q=0;q<9&&!in;++q){const double cx=grid.origin.x+(s[0]*8+(q==8?4:((q&1)?8:0)))*p,cy=grid.origin.y+(s[1]*8+(q==8?4:((q&2)?8:0)))*p,cz=grid.origin.z+(s[2]*8+(q==8?4:((q&4)?8:0)))*p;
    double a0,a1,a2;frame(cx,cy,cz,a0,a1,a2);double o;const int st=r.at(a1,a2,o);in=std::abs(a0)<=z.slab&&(st==1||st==3);}
   if(!in)continue;}
  core::Tile bits{};bool any=false;
  for(int c=0;c<512;++c){const int x=tx*8+(c>>6),y=ty*8+((c>>3)&7),zc=tz*8+(c&7);
   if(x<drawn.lo[0]||x>=drawn.hi[0]||y<drawn.lo[1]||y>=drawn.hi[1]||zc<drawn.lo[2]||zc>=drawn.hi[2])continue;
   frame(grid.origin.x+(x+.5)*p,grid.origin.y+(y+.5)*p,grid.origin.z+(zc+.5)*p,s0,pa,pb);if(std::abs(s0)>reach||pa*pa+pb*pb>z.Rl*z.Rl)continue;
   double o=0;if(r.at(pa,pb,o)<2)continue;const double sd=s0-double(o);if(side==0?!(sd>0):!(sd<0))continue;
   {bool safe=false;const double wx=grid.origin.x+(x+.5)*p,wy=grid.origin.y+(y+.5)*p,wz=grid.origin.z+(zc+.5)*p;
    for(const TearRing*q:all){if(!q||!q->z.ok)continue;const auto&zq=q->z;const double d[3]={wx-zq.C[0],wy-zq.C[1],wz-zq.C[2]};
     const double q0=d[0]*zq.n[0]+d[1]*zq.n[1]+d[2]*zq.n[2],qa=d[0]*zq.u[0]+d[1]*zq.u[1]+d[2]*zq.u[2],qb=d[0]*zq.w[0]+d[1]*zq.w[1]+d[2]*zq.w[2];
     if(std::abs(q0)>zq.slab||qa*qa+qb*qb>zq.Rl*zq.Rl)continue;double oq;if(q->at(qa,qb,oq)==1){safe=true;break;}}
    if(safe)continue;}
   bits.removed[std::size_t(x&7)]|=std::uint64_t(1)<<((y&7)*8+(zc&7));any=true;++made;}
  if(any)list.emplace_back(Cell{tx,ty,tz},bits);}}
 count[th]+=made;});
 for(const auto&list:part)for(const auto&e:list){auto&tile=mine[e.first];for(unsigned q=0;q<8;++q)tile.removed[q]|=e.second.removed[q];}
 for(const auto c:count)marked+=c;
 return marked;}

struct CarryBox {std::array<double,6> b{};std::uint16_t bone=65535,src=0;bool kept=false;};
inline bool carryBoxOf(const gfy_packed::Child&c,CarryBox&out){
 if(c.material||c.vertices.empty()||c.indices.size()!=36)return false;
 std::array<double,6> b{1e30,1e30,1e30,-1e30,-1e30,-1e30};const std::uint16_t bone=c.vertices[0].bone[0];const float mark=c.vertices[0].surface[13];
 for(const auto&v:c.vertices){if(!(v.surface[12]<=-999.f)||v.bone[0]!=bone||v.surface[13]!=mark)return false;
  int unit=0;for(std::size_t a=0;a<3;++a){const float n=v.surface[3+a];if(n==1.f||n==-1.f)++unit;else if(n!=0.f)return false;}if(unit!=1)return false; // (a tear's slab box: not axis-aligned)
  for(std::size_t a=0;a<3;++a){b[a]=std::min(b[a],double(v.surface[a]));b[3+a]=std::max(b[3+a],double(v.surface[a]));}}
 if(!(b[3]>b[0]&&b[4]>b[1]&&b[5]>b[2]))return false;
 if(!(mark>.5f))for(std::size_t a=0;a<3;++a){const double cc=.5*(b[a]+b[3+a]),h=std::max(.005,.5*(b[3+a]-b[a])-.025);b[a]=cc-h;b[3+a]=cc+h;} // (made by a split: 2.5 cm in, once)
 out.b=b;out.bone=bone;out.src=c.material_source_child;out.kept=true;return true;}
template<class PieceAtT,class Ok>
inline std::vector<std::vector<CarryBox>> carryBoxes(const std::vector<gfy_packed::Child>&children,const PieceAtT&pieceAt,std::size_t P,const Ok&ok){
 std::vector<std::vector<CarryBox>> out(P);
 for(const auto&c:children){CarryBox x;if(!carryBoxOf(c,x))continue;
  int n[3];for(std::size_t a=0;a<3;++a)n[a]=std::max(2,std::min(12,int(std::ceil((x.b[3+a]-x.b[a])/.004)))); // (every 4 mm, at most 12 a side)
  std::vector<std::size_t> cnt(P,0);
  for(int i=0;i<n[0];++i)for(int j=0;j<n[1];++j)for(int l=0;l<n[2];++l){
   const int k=pieceAt(x.b[0]+(x.b[3]-x.b[0])*(i+.5)/n[0],x.b[1]+(x.b[4]-x.b[1])*(j+.5)/n[1],x.b[2]+(x.b[5]-x.b[2])*(l+.5)/n[2]);
   if(k>=0&&std::size_t(k)<P)++cnt[std::size_t(k)];}
  int best=-1;
  for(std::size_t k=0;k<P;++k)if(cnt[k]&&x.bone<64&&ok[k][x.bone]&&(best<0||cnt[k]>cnt[std::size_t(best)]))best=int(k);
  if(best<0)for(std::size_t k=0;k<P;++k)if(cnt[k]&&(best<0||cnt[k]>cnt[std::size_t(best)]))best=int(k);
  if(best>=0)out[std::size_t(best)].push_back(x);}
 return out;}
inline void mergeCarryBoxes(std::vector<CarryBox>&L,std::size_t limit){
 auto meets=[](const CarryBox&a,const CarryBox&b){for(std::size_t q=0;q<3;++q)if(a.b[q]>b.b[3+q]||b.b[q]>a.b[3+q])return false;return true;};
 auto inside=[](const CarryBox&a,const CarryBox&b){for(std::size_t q=0;q<3;++q)if(a.b[q]<b.b[q]||a.b[3+q]>b.b[3+q])return false;return true;};
 auto unite=[](CarryBox&a,const CarryBox&b){for(std::size_t q=0;q<3;++q){a.b[q]=std::min(a.b[q],b.b[q]);a.b[3+q]=std::max(a.b[3+q],b.b[3+q]);}};
 auto vol=[](const CarryBox&a){return (a.b[3]-a.b[0])*(a.b[4]-a.b[1])*(a.b[5]-a.b[2]);};
 for(;;){bool again=false;
  for(std::size_t i=0;i<L.size()&&!again;++i)for(std::size_t j=0;j<L.size()&&!again;++j){if(i==j)continue;
   if(inside(L[j],L[i])){L.erase(L.begin()+std::ptrdiff_t(j));again=true;}
   else if(L[i].kept&&L[j].kept&&L[i].bone==L[j].bone&&meets(L[i],L[j])){CarryBox u=L[i];unite(u,L[j]);
    if(vol(u)<=vol(L[i])+vol(L[j])){L[i]=u;L.erase(L.begin()+std::ptrdiff_t(j));again=true;}}}
  if(again)continue;
  if(L.size()<=limit||L.size()<2)break;
  std::size_t bi=0,bj=1;double bd=1e300; // the child slots: the two nearest boxes become one (the larger keeps its bone)
  for(std::size_t i=0;i<L.size();++i)for(std::size_t j=i+1;j<L.size();++j){double d=0;
   for(std::size_t q=0;q<3;++q){const double e=.5*(L[i].b[q]+L[i].b[3+q]-L[j].b[q]-L[j].b[3+q]);d+=e*e;}if(d<bd){bd=d;bi=i;bj=j;}}
  if(vol(L[bj])>vol(L[bi]))std::swap(L[bi],L[bj]);
  L[bi].kept=L[bi].kept||L[bj].kept;unite(L[bi],L[bj]);L.erase(L.begin()+std::ptrdiff_t(bj));}
 if(limit==0)L.clear();}
// one box child, as the fresh boxes below are made
inline void emitCarryBox(gfy_packed::Mesh&mesh,const CarryBox&x,unsigned&next,bool ccwOutward,std::size_t&boxTris){
 painter_mask_pieces::Box box;for(std::size_t a=0;a<3;++a){box.centre[a]=.5*(x.b[a]+x.b[3+a]);box.axis[0][a]=a==0;box.axis[1][a]=a==2;box.axis[2][a]=a==1;}
 box.lo={-.5*(x.b[3]-x.b[0]),-.5*(x.b[5]-x.b[2]),-.5*(x.b[4]-x.b[1])};box.hi={-box.lo[0],-box.lo[1],-box.lo[2]};box.valid=true;
 const Vec boxCentre{box.centre[0],box.centre[1],box.centre[2]};const std::uint16_t boxBone=x.bone;
 const auto corner=painter_mask_pieces::corners(box);const auto tri=painter_mask_pieces::triangles(box);
 gfy_packed::Child child;child.material=0;child.material_source_child=x.src;child.skin_clip=true;child.child_index=std::uint16_t(next++);
 for(std::size_t f=0;f<12;++f){const std::uint16_t q[3]={tri[3*f],tri[3*f+1],tri[3*f+2]};
  const Vec A{corner[q[0]][0],corner[q[0]][1],corner[q[0]][2]},Bq{corner[q[1]][0],corner[q[1]][1],corner[q[1]][2]},Cq{corner[q[2]][0],corner[q[2]][1],corner[q[2]][2]};
  Vec nn{(Bq-A).y*(Cq-A).z-(Bq-A).z*(Cq-A).y,(Bq-A).z*(Cq-A).x-(Bq-A).x*(Cq-A).z,(Bq-A).x*(Cq-A).y-(Bq-A).y*(Cq-A).x};const double ln=length(nn);if(!(ln>1e-12))continue;nn=nn*(1./ln);
  const bool outward=dot(nn,A-boxCentre)>0;
  const std::uint16_t order[3]={q[0],outward==ccwOutward?q[1]:q[2],outward==ccwOutward?q[2]:q[1]};
  const Vec no=outward?nn:nn*-1.;const Vec tg=std::abs(no.y)<.9?Vec{no.z,0,-no.x}:Vec{0,-no.z,no.y};const Vec tn=tg*(1./length(tg));
  const Vec bn{no.y*tn.z-no.z*tn.y,no.z*tn.x-no.x*tn.z,no.x*tn.y-no.y*tn.x};
  for(const auto qi:order){gfy_packed::Vertex row;
   const double*pp=corner[qi].data();const double sv[14]={pp[0],pp[1],pp[2],no.x,no.y,no.z,tn.x,tn.y,tn.z,bn.x,bn.y,bn.z,-1000.,x.kept?1.:0.};
   for(unsigned a=0;a<14;++a)row.surface[a]=float(sv[a]);
   row.bone={boxBone,boxBone,boxBone,boxBone};row.weight={1.f,0.f,0.f,0.f};gfy_packed::canonical_weights(row);
   child.indices.push_back(std::uint16_t(child.vertices.size()));child.vertices.push_back(row);}
  child.original_faces.push_back(~std::uint32_t(0));}
 boxTris+=child.indices.size()/3;mesh.children.push_back(std::move(child));}

// cutends v2 (2026-11-16, the bench BENCH_cutends.md R1): the host's publication BEFORE this burst's damage - a separated piece's
// birth mesh, what it drew (its torn faces given back at its split included). Set by the separation's caller for one prepare on
// its thread (LiveSeparationWorker_impl.h prepare_separation); null: a whole body (no give-backs ever). The published mesh a split
// reads is the host's publication AFTER the damage: a piece's republished field is its grid's removed cells and marks, so the
// earlier bursts' crack cells it drew read removed there.
inline const gfy_packed::Mesh*&hostBefore(){thread_local const gfy_packed::Mesh*m=nullptr;return m;}
struct CutCarry {std::vector<std::vector<std::size_t>> of;std::size_t boxes=0,starters=0,remade=0,dropped=0;};
template<class PieceAtT,class Ok,class Keeps,class Remade>
inline CutCarry cutEndCarry(const gfy_packed::Mesh&published,const std::vector<gfy_packed::Child>&children,const PieceAtT&pieceAt,std::size_t P,const Ok&ok,Keeps&&keeps,Remade&&remade){
 CutCarry R;R.of.assign(P,{});
 auto pos=[](const gfy_packed::Vertex&v){return Vec{v.surface[0],v.surface[1],v.surface[2]};};
 std::vector<const gfy_packed::TearBlock*> tears;for(const auto&tr:published.tears)if(tr.block&&!head_burst_record::is(*tr.block))tears.push_back(tr.block.get());
 for(std::size_t c=0;c<children.size();++c){const auto&ch=children[c];if(ch.material||ch.vertices.empty()||ch.indices.empty())continue;
  bool slab=true,starter=true;for(const auto&v:ch.vertices){slab=slab&&v.surface[12]<=-999.f&&v.surface[12]>-1500.f;starter=starter&&v.surface[12]<=-1999.f;}
  CarryBox axisBox;if(!(starter||(slab&&!carryBoxOf(ch,axisBox))))continue; // (an axis-aligned box is a burst's: carryBoxes)
  Vec ctr{0,0,0};for(const auto&v:ch.vertices)ctr=ctr+pos(v);ctr=ctr*(1./double(ch.vertices.size()));
  const gfy_packed::TearBlock*tear=nullptr;double bestD=.05*.05;
  for(const auto*T:tears){const Vec d{ctr.x-T->centre[0],ctr.y-T->centre[1],ctr.z-T->centre[2]};
   const double a=d.x*T->u[0]+d.y*T->u[1]+d.z*T->u[2],b=d.x*T->w[0]+d.y*T->w[1]+d.z*T->w[2],n=d.x*T->normal[0]+d.y*T->normal[1]+d.z*T->normal[2];
   const double ea=std::max(0.,std::max(T->footLo[0]-a,a-T->footHi[0])),eb=std::max(0.,std::max(T->footLo[1]-b,b-T->footHi[1])),en=std::max(0.,std::abs(n)-T->reach);
   const double dd=ea*ea+eb*eb+en*en;if(dd<=bestD){bestD=dd;tear=T;}}
  if(tear&&remade(tear)){++R.remade;continue;}
  std::vector<std::size_t> cnt(P,0);
  auto vote=[&](const Vec&q,std::size_t w){const int k=pieceAt(q.x,q.y,q.z);if(k>=0&&std::size_t(k)<P)cnt[std::size_t(k)]+=w;};
  const std::size_t stride=std::max<std::size_t>(1,ch.vertices.size()/64);
  for(std::size_t i=0;i<ch.vertices.size();i+=stride){const Vec q=pos(ch.vertices[i]);vote(q,1);if(slab)for(int f=1;f<4;++f)vote(ctr+(q-ctr)*(.25*f),1);}
  vote(ctr,2);
  const std::uint16_t bone=ch.vertices[0].bone[0];int pick=-1;
  for(std::size_t k=0;k<P;++k){if(tear&&!keeps(k,tear))continue;if(!tear&&!cnt[k])continue;
   if(pick<0){pick=int(k);continue;}const std::size_t pk=std::size_t(pick);
   if(cnt[k]>cnt[pk]||(cnt[k]==cnt[pk]&&bone<64&&ok[k][bone]&&!ok[pk][bone]))pick=int(k);}
  if(pick<0&&tear)for(std::size_t k=0;k<P;++k){if(!cnt[k])continue;if(pick<0||cnt[k]>cnt[std::size_t(pick)])pick=int(k);} // its tear kept by no piece
  if(pick<0){++R.dropped;continue;}
  R.of[std::size_t(pick)].push_back(c);if(starter)++R.starters;else ++R.boxes;}
 return R;}
// one carried cut-end child into a piece's mesh, unchanged (its bone rebound to the piece's nearest kept vertex's when the piece
// cannot pose it): the triangles added, 0 when no child slot is left or the piece poses no bone
template<class OkK>
inline std::size_t emitCutChild(gfy_packed::Mesh&mesh,const gfy_packed::Child&src,unsigned&next,const OkK&okk){
 if(next>=gfy_packed::max_child_slots||src.vertices.empty())return 0;
 auto pos=[](const gfy_packed::Vertex&v){return Vec{v.surface[0],v.surface[1],v.surface[2]};};
 gfy_packed::Child child=src;const std::uint16_t bone=src.vertices[0].bone[0];
 if(bone>=64||!okk[bone]){std::uint16_t nb=65535;double bestD=1e300;Vec ctr{0,0,0};for(const auto&v:src.vertices)ctr=ctr+pos(v);ctr=ctr*(1./double(src.vertices.size()));
  for(const auto&ch:mesh.children)for(const auto&row:ch.vertices){if(row.surface[12]<=-999.f)continue;unsigned best=0;for(unsigned w=1;w<4;++w)if(row.weight[w]>row.weight[best])best=w;
   const double dd=length(pos(row)-ctr);if(dd<bestD&&row.bone[best]<64&&okk[row.bone[best]]){bestD=dd;nb=row.bone[best];}}
  if(nb==65535)for(unsigned b=0;b<64;++b)if(okk[b]){nb=std::uint16_t(b);break;}
  if(nb==65535)return 0;
  for(auto&row:child.vertices){row.bone={nb,nb,nb,nb};row.weight={1.f,0.f,0.f,0.f};gfy_packed::canonical_weights(row);}}
 child.child_index=std::uint16_t(next++);const std::size_t t=child.indices.size()/3;mesh.children.push_back(std::move(child));return t;}

template<class Anchorable,class RegionFrontier>
inline std::vector<bound_surface::Piece> build(const gfy_packed::Mesh&published,const Grid&grid,const gfy_connectivity::Topology&volume,
    const std::vector<std::uint32_t>&components,const std::set<std::uint16_t>&wholeChildren,std::uint16_t originalChildren,
    Anchorable&&anchorable,RegionFrontier&&regionFrontier,Stats&stats,
    const std::vector<std::pair<Cell,core::Tile>>*inherited,std::vector<std::vector<std::pair<Cell,core::Tile>>>*marksOut,
    const gear_cells::GearCells*gear=nullptr){
 const auto t0=core::Clock::now();const std::size_t P=components.size(),K=published.tears.size();
 need(P>=1&&P<=maxPieces&&stats.sig.size()==K,"head_burst_pieces_input");
 need(published.mask_volume&&published.removal_field&&published.removal_field->available&&!published.children.empty(),"head_burst_pieces_published_mesh");
 auto* pass=FlightPass::current();
 if(pass && pass->surfaces){
  need(!pass->consumed && pass->ids && *pass->ids==components && pass->marks && marksOut,"flight_surface_epoch");
  *marksOut=*pass->marks;pass->consumed=true;stats.used=true;return *pass->surfaces;
 }
 std::vector<char> deferred(P,0);
 if(pass && marksOut && P>=2 && P<=13){
  std::size_t stump=0;for(std::size_t k=1;k<P;++k)if(volume.components[components[k]].cells>volume.components[components[stump]].cells)stump=k;
  for(std::size_t k=0;k<P;++k)deferred[k]=k!=stump && pass->fitted.count(components[k])!=0;
 }
 const bool flight=std::find(deferred.begin(),deferred.end(),char(1))!=deferred.end();
 stats.used=true;const PieceAt pieceAt(grid,volume,components);const std::uint32_t all=P>=32?~0u:((1u<<P)-1u);
 // ---- the records: their functions, each piece's groups, each group's pieces
 struct Rec {std::size_t t;head_burst_record::Function f;std::vector<std::uint32_t> mask,owners;std::vector<std::array<double,6>> box,cellBox;int main=-1;};
 std::vector<Rec> recs;
 for(std::size_t t=0;t<K;++t){const auto&tr=published.tears[t];if(!fresh(tr)||!head_burst_record::is(*tr.block))continue;
  Rec r;r.t=t;if(!head_burst_record::decode(*tr.block,r.f))continue;const unsigned G=std::min(32u,std::max(1u,r.f.groups));
  std::vector<std::uint32_t> count(std::size_t(G)*P,0);const auto&s=tr.block->section;const std::size_t ns=head_burst_record::sampleFloats(*tr.block);
  r.box.assign(P,{1e30,1e30,1e30,-1e30,-1e30,-1e30});std::vector<std::array<double,6>> groupBox(G,{1e30,1e30,1e30,-1e30,-1e30,-1e30});
  for(std::size_t i=0;i+3<ns;i+=4){const unsigned g=std::min(G-1,unsigned(s[i+3]));const int k=pieceAt(s[i],s[i+1],s[i+2]);
   for(int a=0;a<3;++a){groupBox[g][a]=std::min(groupBox[g][a],double(s[i+a]));groupBox[g][3+a]=std::max(groupBox[g][3+a],double(s[i+a]));}
   if(k<0||std::size_t(k)>=P)continue;++count[std::size_t(g)*P+std::size_t(k)];
   auto&b=r.box[std::size_t(k)];for(int a=0;a<3;++a){b[a]=std::min(b[a],double(s[i+a]));b[3+a]=std::max(b[3+a],double(s[i+a]));}}
  // a piece the samples missed (a small chunk; a part cut off between fragments): its own cells inside the cut, labelled, and their box
  for(std::size_t k=0;k<P;++k){std::uint32_t got=0;for(unsigned g=0;g<G;++g)got+=count[std::size_t(g)*P+k];if(got>=2)continue;
   std::size_t seen=0;const std::uint32_t comp=components[k];
   for(const auto&col:volume.columns)for(auto si=col.first;si<col.first+col.count;++si){const auto&sp=volume.spans[si];if(sp.component!=comp)continue;
    for(int z=sp.lo;z<sp.hi;++z){if((seen++)%5)continue;const Vec q=grid.center(Cell{col.x,col.y,z});
     const double dx=q.x-r.f.B[0],dy=q.y-r.f.B[1],dz=q.z-r.f.B[2];if(dx*dx+dy*dy+dz*dz>r.f.Rcut*r.f.Rcut)continue;
     const unsigned g=std::min(G-1,r.f.label(q.x,q.y,q.z));++count[std::size_t(g)*P+k];
     auto&b=r.box[k];const double c[3]={q.x,q.y,q.z};for(int a=0;a<3;++a){b[a]=std::min(b[a],c[a]);b[3+a]=std::max(b[3+a],c[a]);}}}}
  r.mask.assign(P,0u);r.owners.assign(G,0u);std::uint32_t best=0;
  for(unsigned g=0;g<G;++g)for(std::size_t k=0;k<P;++k)if(count[std::size_t(g)*P+k]>=2){r.mask[k]|=1u<<g;r.owners[g]|=1u<<k;}
  // the main far piece (the neck's: the body): of the pieces holding the far part, the one with the most cells
  {std::uint64_t most=0;for(std::size_t k=0;k<P;++k){if(count[k]<2)continue;const std::uint64_t cc=components[k]<volume.components.size()?volume.components[components[k]].cells:0;
    if(r.main<0||cc>most){most=cc;r.main=int(k);}}
   if(r.main<0)for(std::size_t k=0;k<P;++k)if(count[k]>best){best=count[k];r.main=int(k);}}
  // a piece holding no group of the record (not part of the burst) owns the far part as far as the record goes
  for(std::size_t k=0;k<P;++k)if(!r.mask[k])r.owners[0]|=1u<<k;
  // the far part's pieces: their box takes the whole broken region (every near group's samples)
  for(std::size_t k=0;k<P;++k)if(r.mask[k]&1u)for(unsigned g=1;g<G;++g){auto&b=r.box[k];for(int a=0;a<3;++a){b[a]=std::min(b[a],groupBox[g][a]);b[3+a]=std::max(b[3+a],groupBox[g][3+a]);}}
  recs.push_back(std::move(r));}
 need(!recs.empty(),"head_burst_pieces_no_record");
 // each piece's cells inside each record's cut: their box (the coverage box of what it holds)
 {std::vector<int> pieceOf(volume.components.size(),-1);for(std::size_t k=0;k<P;++k)if(components[k]<pieceOf.size())pieceOf[components[k]]=int(k);
  for(auto&r:recs){r.cellBox.assign(P,{1e30,1e30,1e30,-1e30,-1e30,-1e30});int lo[3],hi[3];
   for(int a=0;a<3;++a){lo[a]=std::max(0,int(std::floor((r.f.B[a]-r.f.Rcut-grid.origin[a])/grid.pitch)));hi[a]=std::min((&grid.size.x)[a]-1,int(std::floor((r.f.B[a]+r.f.Rcut-grid.origin[a])/grid.pitch)));}
   for(const auto&col:volume.columns){if(col.x<lo[0]||col.x>hi[0]||col.y<lo[1]||col.y>hi[1])continue;
    for(auto si=col.first;si<col.first+col.count;++si){const auto&sp=volume.spans[si];if(sp.component>=pieceOf.size())continue;const int k=pieceOf[sp.component];if(k<0)continue;
     const int z0=std::max(sp.lo,lo[2]),z1=std::min(sp.hi-1,hi[2]);if(z0>z1)continue;auto&b=r.cellBox[std::size_t(k)];
     const double x=grid.origin.x+(col.x+.5)*grid.pitch,y=grid.origin.y+(col.y+.5)*grid.pitch,za=grid.origin.z+(z0+.5)*grid.pitch,zb=grid.origin.z+(z1+.5)*grid.pitch;
     b[0]=std::min(b[0],x);b[3]=std::max(b[3],x);b[1]=std::min(b[1],y);b[4]=std::max(b[4],y);b[2]=std::min(b[2],za);b[5]=std::max(b[5],zb);}}}}
 std::vector<Region> R;R.reserve(K);for(const auto&t:published.tears)R.emplace_back(*t.block);
 // the records' crack cells by tile (bits to clear in each piece's removal field)
 std::unordered_map<Cell,core::Tile,core::CellHash> crack;std::size_t crackCells=0;
 // (into a dense tile array over the cells' box first - a body burst lists hundreds of thousands - then the map, tile by tile)
 for(const auto&r:recs){int lo[3]={INT_MAX,INT_MAX,INT_MAX},hi[3]={INT_MIN,INT_MIN,INT_MIN};
  head_burst_record::eachCrackCell(*published.tears[r.t].block,[&](int x,int y,int z){const int q[3]={x>>3,y>>3,z>>3};for(int a=0;a<3;++a){lo[a]=std::min(lo[a],q[a]);hi[a]=std::max(hi[a],q[a]);}});
  if(lo[0]>hi[0])continue;const int nx=hi[0]-lo[0]+1,ny=hi[1]-lo[1]+1,nz=hi[2]-lo[2]+1;
  std::vector<core::Tile> dense(std::size_t(nx)*ny*nz);std::vector<char> used(dense.size(),0);
  head_burst_record::eachCrackCell(*published.tears[r.t].block,[&](int x,int y,int z){const Cell c{x,y,z};if(!grid.valid(c))return;
   const std::size_t i=(std::size_t((z>>3)-lo[2])*ny+std::size_t((y>>3)-lo[1]))*nx+std::size_t((x>>3)-lo[0]);const auto l=Grid::local(c);
   dense[i].removed[l/64]|=std::uint64_t(1)<<(l%64);used[i]=1;++crackCells;});
  for(int tz=0;tz<nz;++tz)for(int ty=0;ty<ny;++ty)for(int tx=0;tx<nx;++tx){const std::size_t i=(std::size_t(tz)*ny+std::size_t(ty))*nx+std::size_t(tx);if(!used[i])continue;
   auto&tile=crack[Cell{lo[0]+tx,lo[1]+ty,lo[2]+tz}];for(unsigned w=0;w<8;++w)tile.removed[w]|=dense[i].removed[w];}}
 // a body burst: every removed cell inside its blown-apart region (the near groups) goes back - the blast's craters there, a sliver
 // the separation scrapped, the repair's edge cells (else the chunks show those cell walls as 2 mm-stepped stripes); the far body
 // (group 0) keeps its craters. Tile by tile: one labelMargin at a tile's centre proves its group for all its cells (margin over
 // the half diagonal x 2), else cell by cell; parallel over the tiles
 std::size_t restoreExtra=0;
 for(const auto&r:recs){const auto&h=published.tears[r.t].block->header;if(!(h.size()>14&&h[14]==2u))continue;
  int tlo[3],thi[3];for(int a=0;a<3;++a){tlo[a]=int(std::floor((r.f.B[a]-r.f.Rcut-grid.origin[a])/grid.pitch))>>3;thi[a]=int(std::floor((r.f.B[a]+r.f.Rcut-grid.origin[a])/grid.pitch))>>3;}
  std::vector<std::pair<Cell,const core::Tile*>> tl;
  for(const auto&gt:grid.tiles){const Cell&tk=gt.first;if(tk.x<tlo[0]||tk.x>thi[0]||tk.y<tlo[1]||tk.y>thi[1]||tk.z<tlo[2]||tk.z>thi[2])continue;tl.push_back({tk,&gt.second});}
  const double proof=std::sqrt(3.)*4*grid.pitch*2.;const std::size_t Wn=std::max<std::size_t>(1,core::parallelWidth());
  std::vector<std::vector<std::pair<Cell,core::Tile>>> addBy(Wn);std::vector<std::size_t> nBy(Wn,0);
  core::parallelChunks(tl.size(),1,[&](std::size_t a,std::size_t b,std::size_t ch){for(std::size_t i=a;i<b;++i){const Cell tk=tl[i].first;const core::Tile&src=*tl[i].second;
    const auto ci=crack.find(tk);core::Tile rest=src;if(ci!=crack.end())for(unsigned w=0;w<8;++w)rest.removed[w]&=~ci->second.removed[w]; // (listed: back already)
    std::uint64_t anyBits=0;for(auto wd:rest.removed)anyBits|=wd;if(!anyBits)continue;
    const Vec c0=grid.center(Cell{tk.x*8+4,tk.y*8+4,tk.z*8+4})-Vec{.5*grid.pitch,.5*grid.pitch,.5*grid.pitch};double m=0;const unsigned g0=r.f.labelMargin(c0.x,c0.y,c0.z,m);
    core::Tile add{};bool any=false;
    if(m>proof){if(g0==0||g0>=253)continue;add=rest;any=true;for(auto wd:rest.removed)for(;wd;wd&=wd-1)++nBy[ch];}
    else for(unsigned w=0;w<8;++w)for(std::uint64_t bits=rest.removed[w];bits;bits&=bits-1){const std::uint64_t low=bits&(~bits+1);unsigned bit=0;while(!((low>>bit)&1u))++bit;
     const Cell c{tk.x*8+int(w),tk.y*8+int(bit>>3),tk.z*8+int(bit&7u)};const Vec q=grid.center(c);const unsigned gl=r.f.label(q.x,q.y,q.z);
     if(gl==0||gl>=253)continue;add.removed[w]|=low;any=true;++nBy[ch];}
    if(any)addBy[ch].push_back({tk,add});}});
  for(std::size_t ch=0;ch<Wn;++ch){restoreExtra+=nBy[ch];for(const auto&ad:addBy[ch]){auto&tile=crack[ad.first];for(unsigned w=0;w<8;++w)tile.removed[w]|=ad.second.removed[w];}}}
 std::size_t stNotHeld=0;double stHeldMs=0;
 {const auto h0=core::Clock::now();
  struct HostRec {head_burst_record::Function f;std::uint32_t mask=0;bool shareFar=false;double rad=0,c[3]{};};
  std::vector<HostRec> hr;
  for(const auto&tr:published.tears){if(!tr.block||fresh(tr)||!head_burst_record::is(*tr.block))continue;HostRec h;if(!head_burst_record::decode(*tr.block,h.f))continue;
   const auto&hh=tr.block->header;h.mask=hh[15];float rad=0;std::memcpy(&rad,&hh[22],4);h.rad=rad;h.shareFar=(hh[26]&255u)==1u&&(hh[23]&1u)==0u&&rad>0;
   if(h.shareFar)for(int a=0;a<3;++a){float v=0;std::memcpy(&v,&hh[std::size_t(28+a)],4);h.c[a]=v;}
   hr.push_back(h);}
  auto groupHeld=[&](const HostRec&h,unsigned g,const Vec&q)->bool{if(!((h.mask>>std::min(g,31u))&1u))return false;if(g!=0||!h.shareFar)return true;
   const double dx=q.x-h.f.B[0],dy=q.y-h.f.B[1],dz=q.z-h.f.B[2];if(dx*dx+dy*dy+dz*dz<=h.f.Rcut*h.f.Rcut)return true;
   const double ex=q.x-h.c[0],ey=q.y-h.c[1],ez=q.z-h.c[2];return ex*ex+ey*ey+ez*ez<=h.rad*h.rad;};
  std::unordered_map<Cell,core::Tile,core::CellHash> hMarks;if(inherited)for(const auto&m:*inherited){auto&d=hMarks[m.first];for(unsigned q=0;q<8;++q)d.removed[q]|=m.second.removed[q];}
  std::unordered_map<Cell,core::Tile,core::CellHash> hOldCrack;
  // cutends v2 (bench R1): with the host's publication before this damage (a separated piece) the test is that field: what it did
  // not draw before this burst never comes back (an earlier burst's erased cells, an earlier wound's crater), what it drew does (its
  // torn faces given back at its split - the crack lists' rule above refused those whenever this burst's crack ran through them)
  const auto*ceBefore=(hostBefore()&&hostBefore()->removal_field&&hostBefore()->removal_field->available)?hostBefore()->removal_field.get():decltype(published.removal_field.get())(nullptr);
  for(const auto&tr:published.tears)if(tr.block&&!fresh(tr)&&head_burst_record::is(*tr.block))
   head_burst_record::eachCrackCell(*tr.block,[&](int x,int y,int z){const Cell c{x,y,z};const Cell tk=Grid::tileKey(c);if(!crack.count(tk))return;
    auto&tt=hOldCrack[tk];const auto l=Grid::local(c);tt.removed[l/64]|=std::uint64_t(1)<<(l%64);});
  std::vector<std::pair<Cell,core::Tile*>> cl;cl.reserve(crack.size());for(auto&c:crack)cl.push_back({c.first,&c.second});
  const double hd=std::sqrt(3.)*4*grid.pitch;std::atomic<std::size_t> dropped{0};
  lowpoly_pieces::eachBlock(cl.size(),16,[&](std::size_t a,std::size_t b,std::size_t){std::size_t dn=0;std::vector<char> perCell(hr.size(),0);
   for(std::size_t s=a;s<b;++s){core::Tile&t=*cl[s].second;const Cell tk=cl[s].first;
    // Crack-face smoothing may restore cells of a new fracture. A previously
    // contacted bone cell which the current field removes must stay removed.
    for(unsigned w=0;w<8;++w)if(t.removed[w])t.removed[w]&=~grid.boneStrikeWord(tk,w);
    // per record: the whole tile held (1), none of it (0: every cell dropped), or cell by cell (-1)
    bool none=false;const Vec c0=grid.center(Cell{tk.x*8+4,tk.y*8+4,tk.z*8+4})-Vec{.5*grid.pitch,.5*grid.pitch,.5*grid.pitch};
    for(std::size_t i=0;i<hr.size()&&!none;++i){const auto&h=hr[i];perCell[i]=1;if(h.shareFar){perCell[i]=-1;continue;}
     const double dx=c0.x-h.f.B[0],dy=c0.y-h.f.B[1],dz=c0.z-h.f.B[2],dd=std::sqrt(dx*dx+dy*dy+dz*dz);int g=-1;
     if(dd-hd>h.f.Rcut)g=0;else if(dd+hd<=h.f.Rcut){double m=0;const unsigned gg=h.f.labelMargin(c0.x,c0.y,c0.z,m);if(m>2*hd)g=int(gg);}
     if(g<0)perCell[i]=-1;else if(!((h.mask>>std::min(unsigned(g),31u))&1u))none=true;}
    const auto mi=hMarks.find(tk);const core::Tile*mk=mi==hMarks.end()?nullptr:&mi->second;
    const auto oci=hOldCrack.find(tk);const core::Tile*oc=oci==hOldCrack.end()?nullptr:&oci->second; // cutends
    for(unsigned w=0;w<8;++w)for(std::uint64_t bits=t.removed[w];bits;bits&=bits-1){const unsigned bit=lowBit64(bits);
     bool keep=!none&&!(mk&&((mk->removed[w]>>bit)&1u));
     if(keep){const Vec q=grid.center(Cell{tk.x*8+int(w),tk.y*8+int(bit>>3),tk.z*8+int(bit&7u)});
      for(std::size_t i=0;i<hr.size()&&keep;++i)if(perCell[i]<0)keep=groupHeld(hr[i],hr[i].f.label(q.x,q.y,q.z),q);
      if(keep&&ceBefore)keep=!ceBefore->sample(q.x,q.y,q.z); // cutends v2: the host did not draw it before this burst
      else if(keep&&oc&&((oc->removed[w]>>bit)&1u)&&published.removal_field->sample(q.x,q.y,q.z))keep=false;} // cutends: an earlier burst erased it
     if(!keep){t.removed[w]&=~(std::uint64_t(1)<<bit);++dn;}}}
   dropped+=dn;});
  stNotHeld=dropped.load();stHeldMs=core::ms(h0);
  if(std::getenv("GFY_M3_LOG")||std::getenv("GFY_PIECE_LOG"))std::cerr<<"seethrough2 host_held records="<<hr.size()<<" listed_not_held="<<stNotHeld<<" ms="<<stHeldMs<<" host_before="<<(ceBefore?1:0)<<std::endl;}
 // ---- one group in several pieces (a fragment the shot channel or a cavity cut in two; a far chunk sharing the far part): the
 // group is cut again between them - on the plane between their sample points' centroids, torn like every crack (the pixel programs:
 // painter_vmask_head.h, record words 26-42 of each sharer's copy); triangles over the group go to the sharer whose centroid is
 // nearest (the same rule), so nothing of the group is drawn twice and nothing steps along cells
 const auto ownB=ownBoxes(volume,components);
 // a fragment's drawing box (TearAtlas row 0): its cells + 3 cm - the clothing padding of its groups (the space under a hood, no
 // piece's cells) is part of the chunk; the programs search only there (+ 2 cells), its marks and restored cells reach 2 cells more
 const int padCells=int(std::ceil(.03/grid.pitch));
 std::vector<CellBox> lim(P);for(std::size_t k=0;k<P;++k){lim[k]=ownB[k];if(lim[k].valid())for(int a=0;a<3;++a){lim[k].lo[a]-=padCells+2;lim[k].hi[a]+=padCells+2;}}
 const int tnx=(grid.size.x+7)/8,tny=(grid.size.y+7)/8,tnz=(grid.size.z+7)/8;
 auto tIdx=[&](int tx,int ty,int tz){return (std::size_t(tz)*std::size_t(tny)+std::size_t(ty))*std::size_t(tnx)+std::size_t(tx);};
 std::vector<char> isMainPiece(P,0);for(const auto&r:recs)if(r.main>=0&&std::size_t(r.main)<P)isMainPiece[std::size_t(r.main)]=1;
 std::vector<std::vector<std::pair<Cell,core::Tile>>> floodMarks(P);std::vector<std::vector<Cell>> reachedTiles(P),zoneMarks(P);
 std::size_t islandCells=0,zoneTiles=0,zoneSeeds=0,zoneNobody=0;double floodMs=0,zoneMs=0;
 // each tile's pieces by their own cells (one walk of the spans): the zones' seeds; and a given-back cell can touch a piece's
 // cells only in a tile that holds them or in one beside it
 std::vector<std::uint32_t> tisT(std::size_t(tnx)*tny*tnz,0);
 for(const auto&col:volume.columns){const int tx=col.x>>3,ty=col.y>>3;if(col.x<0||col.y<0||tx>=tnx||ty>=tny)continue;
  for(auto si=col.first;si<col.first+col.count;++si){const auto&sp=volume.spans[si];if(sp.component>=pieceAt.pieceOf.size()||sp.hi<=sp.lo)continue;const int k=pieceAt.pieceOf[sp.component];if(k<0)continue;
   const int z0t=std::max(sp.lo>>3,0),z1t=std::min((sp.hi-1)>>3,tnz-1);for(int tz=z0t;tz<=z1t;++tz)tisT[tIdx(tx,ty,tz)]|=1u<<k;}}
 const auto*tfF=published.tissue_field&&published.tissue_field->available?published.tissue_field.get():nullptr;
 {const auto f0=core::Clock::now();
  std::vector<std::pair<Cell,const core::Tile*>> ct;ct.reserve(crack.size());
  for(const auto&c:crack){const Cell&tk=c.first;if(tk.x<0||tk.y<0||tk.z<0||tk.x>=tnx||tk.y>=tny||tk.z>=tnz)continue;ct.push_back({tk,&c.second});}
  const std::size_t NS=ct.size();std::vector<std::int32_t> slotOf(std::size_t(tnx)*tny*tnz,-1);
  for(std::size_t s=0;s<NS;++s)slotOf[tIdx(ct[s].first.x,ct[s].first.y,ct[s].first.z)]=std::int32_t(s);
  // each given-back cell's record and group (record << 4 | group; 255 no cell): one proof per tile where a single record's
  // function is constant over it, else cell by cell; parallel over the tiles
  std::vector<std::array<std::uint8_t,512>> lab(NS);const double proofT=std::sqrt(3.)*4*grid.pitch*2.;
  lowpoly_pieces::eachBlock(NS,16,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t s=a;s<b;++s){auto&L=lab[s];L.fill(255);const Cell tk=ct[s].first;const core::Tile&bits=*ct[s].second;
    if(recs.size()==1){const Vec c0=grid.center(Cell{tk.x*8+4,tk.y*8+4,tk.z*8+4})-Vec{.5*grid.pitch,.5*grid.pitch,.5*grid.pitch};double m=0;
     const unsigned g0=recs[0].f.labelMargin(c0.x,c0.y,c0.z,m);
     if(m>proofT){for(unsigned w=0;w<8;++w)for(std::uint64_t bb=bits.removed[w];bb;bb&=bb-1)L[w*64+lowBit64(bb)]=std::uint8_t(g0&15u);continue;}}
    for(unsigned w=0;w<8;++w)for(std::uint64_t bb=bits.removed[w];bb;bb&=bb-1){const unsigned bit=lowBit64(bb);
     const Vec q=grid.center(Cell{tk.x*8+int(w),tk.y*8+int(bit>>3),tk.z*8+int(bit&7u)});std::size_t ri=0;
     for(std::size_t j=0;j<recs.size()&&j<15;++j){const auto&f=recs[j].f;const double dx=q.x-f.B[0],dy=q.y-f.B[1],dz=q.z-f.B[2];if(dx*dx+dy*dy+dz*dz<=f.Rcut*f.Rcut){ri=j;break;}}
     L[w*64+bit]=std::uint8_t((ri<<4)|(recs[ri].f.label(q.x,q.y,q.z)&15u));}}});
  const int sx=grid.size.x,sy=grid.size.y,sz=grid.size.z;
  auto cellPiece=[&](int x,int y,int z)->int{if(x<0||y<0||z<0||x>=sx||y>=sy||z>=sz)return -1;const auto ci=pieceAt.columnOf[std::size_t(y)*std::size_t(sx)+std::size_t(x)];if(ci<0)return -1;
   const auto&c=volume.columns[std::size_t(ci)];for(auto s=c.first;s<c.first+c.count;++s){const auto&sp=volume.spans[s];if(z>=sp.lo&&z<sp.hi)return sp.component<pieceAt.pieceOf.size()?pieceAt.pieceOf[sp.component]:-1;}return -1;};
  std::vector<std::size_t> islandBy(P,0);
  lowpoly_pieces::eachBlock(P,1,[&](std::size_t kf,std::size_t kl,std::size_t){for(std::size_t k=kf;k<kl;++k){
    std::array<char,256> mineLab{};for(std::size_t ri=0;ri<recs.size()&&ri<15;++ri)for(unsigned g=0;g<16&&g<recs[ri].owners.size();++g)if((recs[ri].owners[g]>>k)&1u)mineLab[(ri<<4)|g]=1;
    const CellBox&Lb=lim[k];const bool allT=isMainPiece[k]!=0;std::vector<std::uint32_t> mine;std::vector<std::int32_t> local(NS,-1);
    for(std::size_t s=0;s<NS;++s){const Cell&tk=ct[s].first;
     if(!allT&&(!Lb.valid()||tk.x*8+8<=Lb.lo[0]||tk.x*8>=Lb.hi[0]||tk.y*8+8<=Lb.lo[1]||tk.y*8>=Lb.hi[1]||tk.z*8+8<=Lb.lo[2]||tk.z*8>=Lb.hi[2]))continue;
     local[s]=std::int32_t(mine.size());mine.push_back(std::uint32_t(s));}
    std::vector<core::Tile> cand(mine.size()),reach(mine.size());std::vector<std::uint32_t> q,nq; // q: local slot << 9 | cell
    for(std::size_t i=0;i<mine.size();++i){const std::size_t s=mine[i];const auto&L=lab[s];const core::Tile&bits=*ct[s].second;const Cell tk=ct[s].first;
     const std::uint32_t kb=1u<<k;const bool nearT=((tisT[tIdx(tk.x,tk.y,tk.z)]|(tk.x>0?tisT[tIdx(tk.x-1,tk.y,tk.z)]:0u)|(tk.x+1<tnx?tisT[tIdx(tk.x+1,tk.y,tk.z)]:0u)|(tk.y>0?tisT[tIdx(tk.x,tk.y-1,tk.z)]:0u)|(tk.y+1<tny?tisT[tIdx(tk.x,tk.y+1,tk.z)]:0u)|(tk.z>0?tisT[tIdx(tk.x,tk.y,tk.z-1)]:0u)|(tk.z+1<tnz?tisT[tIdx(tk.x,tk.y,tk.z+1)]:0u))&kb)!=0;
     for(unsigned w=0;w<8;++w)for(std::uint64_t bb=bits.removed[w];bb;bb&=bb-1){const unsigned bit=lowBit64(bb);if(!mineLab[L[w*64+bit]])continue;
      cand[i].removed[w]|=std::uint64_t(1)<<bit;const int x=tk.x*8+int(w),y=tk.y*8+int(bit>>3),z=tk.z*8+int(bit&7u);const int kk=int(k);
      if(nearT&&(cellPiece(x,y,z-1)==kk||cellPiece(x,y,z+1)==kk||cellPiece(x-1,y,z)==kk||cellPiece(x+1,y,z)==kk||cellPiece(x,y-1,z)==kk||cellPiece(x,y+1,z)==kk)){
       reach[i].removed[w]|=std::uint64_t(1)<<bit;q.push_back(std::uint32_t(i<<9)|(w*64+bit));}}}
    static const int d6[6][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    while(!q.empty()){nq.clear();
     for(const auto e:q){const std::size_t i=e>>9;const unsigned l=e&511u;const Cell tk=ct[mine[i]].first;const int x=tk.x*8+int(l>>6),y=tk.y*8+int((l>>3)&7u),z=tk.z*8+int(l&7u);
      for(const auto&d:d6){const int X=x+d[0],Y=y+d[1],Z=z+d[2];if(X<0||Y<0||Z<0||X>=sx||Y>=sy||Z>=sz)continue;std::size_t j=i;
       if((X>>3)!=tk.x||(Y>>3)!=tk.y||(Z>>3)!=tk.z){const std::int32_t s2=slotOf[tIdx(X>>3,Y>>3,Z>>3)];if(s2<0)continue;const std::int32_t j2=local[std::size_t(s2)];if(j2<0)continue;j=std::size_t(j2);}
       const unsigned w=unsigned(X&7),bit=unsigned((Y&7)*8+(Z&7));const std::uint64_t m=std::uint64_t(1)<<bit;
       if(!(cand[j].removed[w]&m)||(reach[j].removed[w]&m))continue;reach[j].removed[w]|=m;nq.push_back(std::uint32_t(j<<9)|(w*64+bit));}}
     q.swap(nq);}
    // (only TISSUE stays removed: an island of flesh. Open air and padding the blast removed go back as before - a mark there
    // would hide single cells of the clothing over it)
    for(std::size_t i=0;i<mine.size();++i){core::Tile t{};bool any=false,got=false;const Cell tk=ct[mine[i]].first;
     for(unsigned w=0;w<8;++w){std::uint64_t un=cand[i].removed[w]&~reach[i].removed[w];got=got||reach[i].removed[w]!=0;
      if(tfF)for(std::uint64_t v=un;v;v&=v-1){const unsigned bit=lowBit64(v);const Vec qc=grid.center(Cell{tk.x*8+int(w),tk.y*8+int(bit>>3),tk.z*8+int(bit&7u)});
       const unsigned an=tfF->sample(qc.x,qc.y,qc.z);if((an&64u)||(an&63u)==0||(an&63u)==16)un&=~(std::uint64_t(1)<<bit);}
      t.removed[w]=un;any=any||un!=0;for(std::uint64_t v=un;v;v&=v-1)++islandBy[k];}
     if(any)floodMarks[k].push_back({ct[mine[i]].first,t});if(got)reachedTiles[k].push_back(ct[mine[i]].first);}}});
  for(std::size_t k=0;k<P;++k)islandCells+=islandBy[k];
  floodMs=core::ms(f0);}
 struct Zones {bool ok=false;int lo[3]{},n[3]{};std::vector<std::uint32_t> zone;
  std::uint32_t at(int tx,int ty,int tz)const{if(!ok)return ~0u;tx-=lo[0];ty-=lo[1];tz-=lo[2];if(tx<0||ty<0||tz<0||tx>=n[0]||ty>=n[1]||tz>=n[2])return ~0u;
   return zone[(std::size_t(tz)*std::size_t(n[1])+std::size_t(ty))*std::size_t(n[0])+std::size_t(tx)];}};
 Zones zones;
 std::vector<char> lostMain; // splitfix2: the far tiles the main piece's zone lost
 if(P>=2&&!std::getenv("GFY_HB_NO_ZONES")){const auto z0=core::Clock::now();
  // the region: every record's cut + 20 cm (a hood's peak, a pack's top beyond the cut), in tiles
  int lo[3]={INT_MAX,INT_MAX,INT_MAX},hi[3]={INT_MIN,INT_MIN,INT_MIN};const int tn[3]={tnx,tny,tnz};
  for(const auto&r:recs)for(int a=0;a<3;++a){const double Rr=r.f.Rcut+.20;
   lo[a]=std::min(lo[a],int(std::floor((r.f.B[a]-Rr-grid.origin[a])/grid.pitch))>>3);hi[a]=std::max(hi[a],int(std::floor((r.f.B[a]+Rr-grid.origin[a])/grid.pitch))>>3);}
  bool any=true;for(int a=0;a<3;++a){lo[a]=std::max(lo[a],0);hi[a]=std::min(hi[a],tn[a]-1);any=any&&lo[a]<=hi[a]&&hi[a]-lo[a]<1023;}
  const std::size_t n0=any?std::size_t(hi[0]-lo[0]+1):0,n1=any?std::size_t(hi[1]-lo[1]+1):0,n2=any?std::size_t(hi[2]-lo[2]+1):0,N=n0*n1*n2;
  if(any&&N&&N<(std::size_t(1)<<23)){
   auto rIdx=[&](int x,int y,int z){return (std::size_t(z)*n1+std::size_t(y))*n0+std::size_t(x);};
   // seeds: each tile's pieces (their cells; their connected given-back cells)
   std::vector<std::uint32_t> seed(N,0);
   for(int z=0;z<int(n2);++z)for(int y=0;y<int(n1);++y){const std::uint32_t*from=&tisT[tIdx(lo[0],y+lo[1],z+lo[2])];std::uint32_t*to=&seed[rIdx(0,y,z)];for(std::size_t x=0;x<n0;++x)to[x]=from[x];}
   for(std::size_t k=0;k<P;++k)for(const Cell&c:reachedTiles[k]){const int tx=c.x-lo[0],ty=c.y-lo[1],tz=c.z-lo[2];if(tx<0||ty<0||tz<0||tx>=int(n0)||ty>=int(n1)||tz>=int(n2))continue;seed[rIdx(tx,ty,tz)]|=1u<<k;}
   // nearest seed per tile (its packed tile, -1 none): two raster passes of vector propagation over the 26 neighbours
   std::vector<std::int32_t> nearS(N,-1),best(N,INT_MAX);
   {std::size_t i=0;for(int z=0;z<int(n2);++z)for(int y=0;y<int(n1);++y)for(int x=0;x<int(n0);++x,++i)if(seed[i]){nearS[i]=std::int32_t(x|(y<<10)|(z<<20));best[i]=0;++zoneSeeds;}}
   // nobody's seeds: a tile with no piece in it where solid tissue was removed (3 of 8 probe cells: removed and tissue in the anatomy)
   {const auto*tf=published.tissue_field&&published.tissue_field->available?published.tissue_field.get():nullptr;
    if(tf)for(const auto&gt:grid.tiles){const Cell&tk=gt.first;const int tx=tk.x-lo[0],ty=tk.y-lo[1],tz=tk.z-lo[2];if(tx<0||ty<0||tz<0||tx>=int(n0)||ty>=int(n1)||tz>=int(n2))continue;
     const std::size_t i=rIdx(tx,ty,tz);if(seed[i])continue;int gone=0;
     for(int q=0;q<8;++q){const int x=(q&1)?5:2,y=(q&2)?5:2,z=(q&4)?5:2;if(!((gt.second.removed[std::size_t(x)]>>(y*8+z))&1u))continue;
      const Vec w=grid.center(Cell{tk.x*8+x,tk.y*8+y,tk.z*8+z});const unsigned v=tf->sample(w.x,w.y,w.z);if(!(v&64u)&&(v&63u)!=0&&(v&63u)!=16)++gone;}
     if(gone>=3){nearS[i]=std::int32_t(tx|(ty<<10)|(tz<<20));best[i]=0;++zoneNobody;}}}
   if(zoneSeeds){
    static const int fw[13][3]={{-1,-1,-1},{0,-1,-1},{1,-1,-1},{-1,0,-1},{0,0,-1},{1,0,-1},{-1,1,-1},{0,1,-1},{1,1,-1},{-1,-1,0},{0,-1,0},{1,-1,0},{-1,0,0}};
    for(int pass=0;pass<2;++pass){const int sgn=pass?-1:1;
     for(int zz=0;zz<int(n2);++zz){const int z=pass?int(n2)-1-zz:zz;for(int yy=0;yy<int(n1);++yy){const int y=pass?int(n1)-1-yy:yy;for(int xx=0;xx<int(n0);++xx){const int x=pass?int(n0)-1-xx:xx;const std::size_t i=rIdx(x,y,z);if(!best[i])continue;
      for(const auto&d:fw){const int X=x+sgn*d[0],Y=y+sgn*d[1],Z=z+sgn*d[2];if(X<0||Y<0||Z<0||X>=int(n0)||Y>=int(n1)||Z>=int(n2))continue;
       const std::int32_t s=nearS[rIdx(X,Y,Z)];if(s<0)continue;const int ax=x-(s&1023),ay=y-((s>>10)&1023),az=z-(s>>20);const int dd=ax*ax+ay*ay+az*az;
       if(dd<best[i]){best[i]=dd;nearS[i]=s;}}}}}}
    // each tile's pieces: the pieces of its nearest seed, one tile of slack (the 27 tiles around)
    std::vector<std::uint32_t> a(N),b(N);
    for(std::size_t i=0;i<N;++i){const std::int32_t s=nearS[i];a[i]=s<0?~0u:seed[rIdx(s&1023,(s>>10)&1023,s>>20)];}
    {std::size_t i=0;for(std::size_t z=0;z<n2;++z)for(std::size_t y=0;y<n1;++y)for(std::size_t x=0;x<n0;++x,++i)b[i]=a[i]|(x?a[i-1]:0u)|(x+1<n0?a[i+1]:0u);}
    {std::size_t i=0;for(std::size_t z=0;z<n2;++z)for(std::size_t y=0;y<n1;++y)for(std::size_t x=0;x<n0;++x,++i)a[i]=b[i]|(y?b[i-n0]:0u)|(y+1<n1?b[i+n0]:0u);}
    zones.zone.resize(N);{std::size_t i=0;const std::size_t n01=n0*n1;for(std::size_t z=0;z<n2;++z)for(std::size_t r=0;r<n01;++r,++i)zones.zone[i]=a[i]|(z?a[i-n01]:0u)|(z+1<n2?a[i+n01]:0u);}
    std::size_t farTiles=0;
    bool bodyBurst=false;for(const auto&r:recs){const auto&h=published.tears[r.t].block->header;bodyBurst=bodyBurst||(h.size()>14&&h[14]==2u);}
    if(!bodyBurst&&!std::getenv("GFY_HB_NO_FARZONE")){const auto fz0=core::Clock::now();std::uint32_t mainBits=0;for(std::size_t k=0;k<P;++k)if(isMainPiece[k])mainBits|=1u<<k;std::unordered_map<std::int32_t,std::array<double,4*maxPieces>> cent;const int sx=grid.size.x,sy=grid.size.y;
     for(std::size_t i=0;i<N;++i){if(best[i]<9||best[i]==INT_MAX)continue;const std::int32_t s=nearS[i];if(s<0)continue;
      const int tx=s&1023,ty=(s>>10)&1023,tz=s>>20;std::uint32_t bits=seed[rIdx(tx,ty,tz)];
      if(bits&(bits-1)){auto it=cent.find(s);
       if(it==cent.end()){std::array<double,4*maxPieces> c{};const int x0=(tx+lo[0])*8,y0=(ty+lo[1])*8,z0=(tz+lo[2])*8;
        for(int y=y0;y<y0+8;++y)for(int x=x0;x<x0+8;++x){if(x<0||y<0||x>=sx||y>=sy)continue;const auto ci=pieceAt.columnOf[std::size_t(y)*std::size_t(sx)+std::size_t(x)];if(ci<0)continue;
         const auto&col=volume.columns[std::size_t(ci)];for(auto si=col.first;si<col.first+col.count;++si){const auto&sp=volume.spans[si];if(sp.component>=pieceAt.pieceOf.size())continue;
          const int k=pieceAt.pieceOf[sp.component];if(k<0||k>=int(maxPieces))continue;const int a=std::max<int>(sp.lo,z0),b=std::min<int>(sp.hi,z0+8);if(a>=b)continue;
          const double n=b-a;c[4*k]+=n;c[4*k+1]+=n*x;c[4*k+2]+=n*y;c[4*k+3]+=.5*(double(a)+double(b)-1)*n;}}
        it=cent.emplace(s,c).first;}
       const std::size_t x=i%n0,y=(i/n0)%n1,z=i/(n0*n1);const double qx=(double(x)+lo[0])*8+4,qy=(double(y)+lo[1])*8+4,qz=(double(z)+lo[2])*8+4;
       double bd=1e30;int bk=-1;for(unsigned k=0;k<maxPieces;++k){if(!((bits>>k)&1u))continue;const auto&c=it->second;
        const double cx=c[4*k]>0?c[4*k+1]/c[4*k]:(tx+lo[0])*8+4,cy=c[4*k]>0?c[4*k+2]/c[4*k]:(ty+lo[1])*8+4,cz=c[4*k]>0?c[4*k+3]/c[4*k]:(tz+lo[2])*8+4;
        const double d=(cx-qx)*(cx-qx)+(cy-qy)*(cy-qy)+(cz-qz)*(cz-qz);if(d<bd){bd=d;bk=int(k);}}
       if(bk>=0)bits=1u<<bk;}
      const std::uint32_t old=zones.zone[i],now=(old&~mainBits)|(bits&mainBits);if(now!=old){zones.zone[i]=now;++farTiles;if(lostMain.empty())lostMain.assign(N,0);lostMain[i]=1;}}
     if(std::getenv("GFY_M3_LOG")||std::getenv("GFY_PIECE_LOG"))std::cerr<<"head_burst_far_zone tiles="<<farTiles<<" mixed_seeds="<<cent.size()<<" region_tiles="<<N<<" far_ms="<<core::ms(fz0)<<std::endl;}
    for(int q=0;q<3;++q){zones.lo[q]=lo[q];}zones.n[0]=int(n0);zones.n[1]=int(n1);zones.n[2]=int(n2);zones.ok=true;
    // the marks: every tile holding gear cells or padding outside a piece's zone (the body: the whole region; a fragment: its
    // drawing box + 8 cm, its triangles reach no farther)
    const gear_cells::GearCells*gz=gear&&gear->fits(grid.origin,grid.pitch,grid.size)?gear:nullptr;
    const auto padKeep=(published.tissue_field&&published.tissue_field->available)?gear_cells::padTileFlags(published.tissue_field,grid):std::shared_ptr<const std::vector<std::uint8_t>>();
    struct UT {std::uint32_t i;Cell c;};std::vector<UT> U;
    {std::size_t i=0;for(int z=0;z<int(n2);++z)for(int y=0;y<int(n1);++y){std::size_t g=tIdx(lo[0],y+lo[1],z+lo[2]);for(int x=0;x<int(n0);++x,++i,++g)
      if((gz&&g<gz->tileFlag.size()&&gz->tileFlag[g])||(padKeep&&g<padKeep->size()&&(*padKeep)[g]))U.push_back({std::uint32_t(i),Cell{x+lo[0],y+lo[1],z+lo[2]}});}}
    std::vector<std::size_t> zt(P,0);const int reachC=int(std::ceil(.08/grid.pitch));
    lowpoly_pieces::eachBlock(P,1,[&](std::size_t kf,std::size_t kl,std::size_t){for(std::size_t k=kf;k<kl;++k){const CellBox&Lb=lim[k];const bool allT=isMainPiece[k]!=0;if(!allT&&!Lb.valid())continue;
      for(const auto&u:U){if((zones.zone[u.i]>>k)&1u)continue;const int tx=u.c.x,ty=u.c.y,tz=u.c.z;if(allT&&!lostMain.empty()&&lostMain[u.i])continue; /* splitfix2: no mark - the selection keeps the main piece off these tiles */
       if(!allT&&(tx*8+8<=Lb.lo[0]-reachC||tx*8>=Lb.hi[0]+reachC||ty*8+8<=Lb.lo[1]-reachC||ty*8>=Lb.hi[1]+reachC||tz*8+8<=Lb.lo[2]-reachC||tz*8>=Lb.hi[2]+reachC))continue;
       zoneMarks[k].push_back(Cell{tx,ty,tz});++zt[k];}}});
    for(std::size_t k=0;k<P;++k)zoneTiles+=zt[k];}}
  zoneMs=core::ms(z0);}
 auto zoneAt=[&](const Vec&q)->std::uint32_t{if(!zones.ok)return ~0u;
  return zones.at(int(std::floor((q.x-grid.origin.x)/grid.pitch))>>3,int(std::floor((q.y-grid.origin.y)/grid.pitch))>>3,int(std::floor((q.z-grid.origin.z)/grid.pitch))>>3);};
 struct Split {std::size_t rec=0;unsigned g=0;std::uint32_t sharers=0;std::array<Vec,maxPieces> c{};std::array<double,maxPieces> rad{};int main=-1;};
 std::vector<Split> splits;double splitMs=0;
 {const auto q0=core::Clock::now();
  for(std::size_t ri=0;ri<recs.size();++ri){const auto&r=recs[ri];std::uint32_t notIn=0;for(std::size_t k=0;k<P;++k)if(!r.mask[k])notIn|=1u<<k;
   const auto&s=published.tears[r.t].block->section;const std::size_t ns=head_burst_record::sampleFloats(*published.tears[r.t].block);
   for(std::size_t g=0;g<r.owners.size();++g){const std::uint32_t o=r.owners[g]&~notIn;if(!(o&(o-1)))continue;
    Split sp;sp.rec=ri;sp.g=unsigned(g);sp.sharers=o;std::array<double,maxPieces> n{};
    for(std::size_t i=0;i+3<ns;i+=4){if(unsigned(s[i+3])!=g)continue;const int k=pieceAt(s[i],s[i+1],s[i+2]);if(k<0||!((o>>k)&1u))continue;
     sp.c[std::size_t(k)]=sp.c[std::size_t(k)]+Vec{s[i],s[i+1],s[i+2]};n[std::size_t(k)]+=1;}
    for(std::size_t k=0;k<P;++k)if((o>>k)&1u){if(n[k]>0)sp.c[k]=sp.c[k]*(1./n[k]);
     else{const auto&b=r.box[k];sp.c[k]=Vec{.5*(b[0]+b[3]),.5*(b[1]+b[4]),.5*(b[2]+b[5])};}
     // the sphere of what it holds: its centroid to the farthest corner of its cells in the cut, + 3 cm (its padding)
     const auto&cb=r.cellBox[k].at(3)>=r.cellBox[k].at(0)?r.cellBox[k]:r.box[k];double m=0;
     for(int q=0;q<8;++q){const Vec cr{(q&1)?cb[3]:cb[0],(q&2)?cb[4]:cb[1],(q&4)?cb[5]:cb[2]};m=std::max(m,length(cr-sp.c[k]));}
     sp.rad[k]=m+.03;}
    if(r.main>=0&&((o>>r.main)&1u))sp.main=r.main;
    splits.push_back(sp);}}
  splitMs=core::ms(q0);}
 auto splitAt=[&](const Vec&q,std::uint32_t cand)->std::uint32_t{ // of one split's sharers in cand, only the one with the nearest centroid stays
  for(const auto&sp:splits){const std::uint32_t sh=cand&sp.sharers;if(!(sh&(sh-1)))continue;
   const unsigned gq=recs[sp.rec].f.label(q.x,q.y,q.z);if(gq!=sp.g)continue;
   double best=1e300;int bk=-1;
   if(sp.main>=0&&((sh>>sp.main)&1u)){bk=sp.main;best=length(q-sp.c[std::size_t(sp.main)]); // the body, unless inside a chunk's sphere and nearer it
    for(std::size_t k=0;k<P;++k)if(((sh>>k)&1u)&&int(k)!=sp.main){const double d=length(q-sp.c[k]);if(d<sp.rad[k]&&d<best){best=d;bk=int(k);}}}
   else for(std::size_t k=0;k<P;++k)if((sh>>k)&1u){const double d=length(q-sp.c[k]);if(d<best){best=d;bk=int(k);}}
   if(bk>=0)cand=(cand&~sp.sharers)|(1u<<bk);}
  return cand;};
 bool anyBody=false;int mainPiece=-1;for(const auto&r:recs){const auto&h=published.tears[r.t].block->header;anyBody=anyBody||(h.size()>14&&h[14]==2u);if(mainPiece<0)mainPiece=r.main;}
 // ---- posing rules (MaskPieces3's)
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
 auto skinOwners=[&](const gfy_packed::Vertex*const*v,std::uint32_t among)->std::uint32_t{double best=0;std::uint32_t set=0;
  for(std::size_t k=0;k<P;++k){if(!((among>>k)&1u))continue;double s=0;for(int j=0;j<3;++j)for(unsigned w=0;w<4;++w){const auto b=v[j]->bone[w];if(b<64&&ok[k][b]&&v[j]->weight[w]>0.f)s+=v[j]->weight[w];}
   if(s>best+1e-6){best=s;set=1u<<k;}else if(s>0&&std::abs(s-best)<=1e-6)set|=1u<<k;}return set;};
 auto underneath=[&](const gfy_packed::Vertex*const*v,std::uint32_t set,bool&found)->int{found=false;const Vec c=(position(*v[0])+position(*v[1])+position(*v[2]))*(1./3);
  Vec nn{0,0,0};for(int j=0;j<3;++j)nn=nn+Vec{v[j]->surface[3],v[j]->surface[4],v[j]->surface[5]};const double ln=length(nn);
  if(ln>1e-9){nn=nn*(1./ln);for(int s=0;s<=8;++s){const Vec q=c-nn*(grid.pitch*(.5+s));const int k=pieceAt(q.x,q.y,q.z);if(k>=0&&((set>>k)&1u)){found=true;return k;}}}
  for(std::size_t k=0;k<P;++k)if((set>>k)&1u)return int(k);return -1;};
 auto isBox=[](const gfy_packed::Child&c){if(c.vertices.empty())return false;for(const auto&v:c.vertices)if(!(v.surface[12]<=-999.f))return false;return true;};
 // ---- triangle selection (parallel over children)
 const auto s0=core::Clock::now();const auto&children=published.children;
 struct Sel {std::vector<std::vector<std::uint32_t>> tris;std::vector<std::map<std::uint16_t,std::uint16_t>> anchors;std::vector<std::vector<char>> touch;
  std::size_t dropped{},unposable{},frozen{},inRegion{},bySkeleton{},byVolume{},ties{};};
 std::vector<Sel> sel(children.size());
 // piecesregional (2026-11-16): the selection's cost follows the blowout, not the model. The same per-triangle rule, decisions and
 // counters, in the same triangle order:
 //  - every child in chunks of 2048 triangles, the chunks in parallel; each chunk's lists, anchors, touches and counters are
 //    appended / merged into its child's in chunk order (a decision reads nothing another triangle wrote);
 //  - a triangle whose 7 samples all lie outside every record's cut (its centre farther than the cut + its own radius) gets group
 //    0's owners from every record, the same for every sample; when its samples' zone is one value too (all outside the zones' box,
 //    or one tile, or all equal), every sample has the same candidate set: worked out once with the same skin / tie rule, the
 //    per-sample counters x 7. A tear region is asked only where a sample can lie in it (a ball around the region, 2 % slack on an
 //    orthonormal frame) and only for a touch not yet set. Blocks of 64 triangles answer those three questions once from their
 //    box (+ its diagonal for the cut, + 1 um for the tiles). Anything else: the per-sample rule below, unchanged.
 //  (GFY_PR_SLOW_SELECT: every triangle by the per-sample rule - probe comparisons.)
 static const bool prSlowSelect=std::getenv("GFY_PR_SLOW_SELECT")!=nullptr;
 std::uint32_t prOut=all;for(const auto&r:recs)prOut&=0u<r.owners.size()?r.owners[0]:0u;
 struct PrBall {double c[3]={0,0,0},reach=0;bool use=false;};std::vector<PrBall> prBall(K);
 for(std::size_t i=0;i<K;++i){const Region&g=R[i];auto&b=prBall[i];const double*f[3]={g.n,g.u,g.w};bool ortho=true;
  for(int x=0;x<3;++x)for(int y=0;y<3;++y){const double dt=f[x][0]*f[y][0]+f[x][1]*f[y][1]+f[x][2]*f[y][2];ortho=ortho&&std::abs(dt-(x==y?1.:0.))<1e-3;}
  const double hh=g.H+tear_decide::offMax;b.reach=std::sqrt((hh*hh+g.Rl*g.Rl)/.98)+1e-6;for(int a=0;a<3;++a)b.c[a]=g.C[a];b.use=ortho&&std::isfinite(b.reach);}
 // a point farther than `reach` from the region's centre is outside it: |s|^2+pa^2+pb^2 >= .98 |d|^2 on a frame within 1e-3 of orthonormal
 auto prTile=[&](double q,double o){return int(std::floor((q-o)/grid.pitch))>>3;};
 const double prOrigin[3]={grid.origin.x,grid.origin.y,grid.origin.z};
 struct PrChunk {std::size_t c,t0,t1;};std::vector<PrChunk> prChunks;const std::size_t prChunkTris=prSlowSelect?std::size_t(1)<<40:2048;
 for(std::size_t c=0;c<children.size();++c){auto&s=sel[c];s.tris.resize(P);s.anchors.resize(P);s.touch.assign(P,std::vector<char>(K,0));
  const auto&src=children[c];if(src.material||src.indices.empty()||wholeChildren.count(src.material_source_child)||isBox(src))continue;
  const std::size_t T=src.indices.size()/3;for(std::size_t a=0;a<T;a+=prChunkTris)prChunks.push_back({c,a,std::min(T,a+prChunkTris)});}
 std::vector<Sel> prPart(prChunks.size());
 lowpoly_pieces::eachBlock(prChunks.size(),1,[&](std::size_t first,std::size_t last,std::size_t){for(std::size_t ci=first;ci<last;++ci){auto&s=prPart[ci];s.tris.resize(P);s.anchors.resize(P);
  s.touch.assign(P,std::vector<char>(K,0));
  const auto&src=children[prChunks[ci].c];const std::size_t prEnd=prChunks[ci].t1;
  std::size_t prBlockEnd=prChunks[ci].t0;bool prBlockFar=false,prBlockOut=false;std::vector<char> prBlockTear(K,1);
  for(std::size_t t=prChunks[ci].t0;t<prEnd;++t){const gfy_packed::Vertex*v[3]={&src.vertices[src.indices[3*t]],&src.vertices[src.indices[3*t+1]],&src.vertices[src.indices[3*t+2]]};
   const Vec a=position(*v[0]),b=position(*v[1]),d=position(*v[2]);
   if(!prSlowSelect){
    if(t>=prBlockEnd){prBlockEnd=std::min(prEnd,t+64);double lo[3]={1e300,1e300,1e300},hi[3]={-1e300,-1e300,-1e300};bool fin=true;
     for(std::size_t u=t;u<prBlockEnd;++u)for(int j=0;j<3;++j){const auto&row=src.vertices[src.indices[3*u+j]];for(int ax=0;ax<3;++ax){const double q=row.surface[ax];fin=fin&&std::isfinite(q);lo[ax]=std::min(lo[ax],q);hi[ax]=std::max(hi[ax],q);}}
     double diag=0;for(int ax=0;ax<3;++ax)diag+=(hi[ax]-lo[ax])*(hi[ax]-lo[ax]);diag=std::sqrt(diag);
     auto boxDist=[&](const double*c){double s2=0;for(int ax=0;ax<3;++ax){const double e=c[ax]<lo[ax]?lo[ax]-c[ax]:c[ax]>hi[ax]?c[ax]-hi[ax]:0.;s2+=e*e;}return std::sqrt(s2);};
     prBlockFar=fin;for(const auto&r:recs)prBlockFar=prBlockFar&&boxDist(r.f.B)>r.f.Rcut+diag+1e-6;
     prBlockOut=fin;if(prBlockOut&&zones.ok){bool outside=false;
      for(int ax=0;ax<3;++ax){const int tl=prTile(lo[ax]-1e-6,prOrigin[ax]),th=prTile(hi[ax]+1e-6,prOrigin[ax]);outside=outside||th<zones.lo[ax]||tl>=zones.lo[ax]+zones.n[ax];}
      prBlockOut=outside;}
     for(std::size_t i=0;i<K;++i)prBlockTear[i]=char(!fin||!prBall[i].use||!(boxDist(prBall[i].c)>prBall[i].reach));}
    Vec cc{0,0,0};double rr=0;bool ccDone=false;auto centre=[&]{if(!ccDone){cc=(a+b+d)*(1./3);rr=std::max({length(a-cc),length(b-cc),length(d-cc)});ccDone=true;}};
    bool far=prBlockFar;
    if(!far){centre();far=true;for(const auto&r:recs){const double dx=cc.x-r.f.B[0],dy=cc.y-r.f.B[1],dz=cc.z-r.f.B[2];if(!(std::sqrt(dx*dx+dy*dy+dz*dz)>r.f.Rcut+rr+1e-6)){far=false;break;}}}
    if(far){Vec q7[7];bool qDone=false;auto samples7=[&]{if(!qDone){q7[0]=a;q7[1]=b;q7[2]=d;q7[3]=(a+b+d)*(1./3);q7[4]=(a+b)*.5;q7[5]=(b+d)*.5;q7[6]=(a+d)*.5;qDone=true;}}; // (not near any cut: the rule's 7 samples)
     std::uint32_t Z=~0u;bool same=true;
     if(zones.ok&&!prBlockOut){samples7();int lo[3],hi[3];for(int ax=0;ax<3;++ax){double mn=1e300,mx=-1e300;for(const auto&q:q7){const double cq=ax==0?q.x:ax==1?q.y:q.z;mn=std::min(mn,cq);mx=std::max(mx,cq);}
       lo[ax]=prTile(mn,prOrigin[ax]);hi[ax]=prTile(mx,prOrigin[ax]);}
      bool outside=false;for(int ax=0;ax<3;++ax)outside=outside||hi[ax]<zones.lo[ax]||lo[ax]>=zones.lo[ax]+zones.n[ax];
      if(outside)Z=~0u;
      else if(lo[0]==hi[0]&&lo[1]==hi[1]&&lo[2]==hi[2])Z=zones.at(lo[0],lo[1],lo[2]);
      else{Z=zoneAt(q7[0]);for(int i=1;i<7&&same;++i)same=zoneAt(q7[i])==Z;}}
     if(same){std::uint32_t cand=prOut&Z;
      if(!cand){++s.dropped;continue;}
      if(cand&(cand-1)){std::uint32_t o=skinOwners(v,cand);if(!o)o=cand;
       if(o&(o-1)){s.ties+=7;bool f=false;const int k=underneath(v,o,f);if(f)s.byVolume+=7;cand=k>=0?1u<<k:o;}else cand=o;
       s.bySkeleton+=7;}
      for(std::size_t i=0;i<K;++i){if(!prBlockTear[i])continue;bool ask=false;for(std::size_t k=0;k<P&&!ask;++k)ask=((cand>>k)&1u)&&!s.touch[k][i];if(!ask)continue;
       const auto&g=prBall[i];if(g.use){centre();const double dx=cc.x-g.c[0],dy=cc.y-g.c[1],dz=cc.z-g.c[2];if(std::sqrt(dx*dx+dy*dy+dz*dz)>g.reach+rr)continue;}
       samples7();bool in=false;for(const auto&q:q7)if(R[i].side(q.x,q.y,q.z)>=0){in=true;break;}
       if(in)for(std::size_t k=0;k<P;++k)if((cand>>k)&1u)s.touch[k][i]=1;}
      for(std::size_t k=0;k<P;++k){if(!((cand>>k)&1u))continue;
       bool fine=anchored(k,*v[0])&&anchored(k,*v[1])&&anchored(k,*v[2]);
       if(!fine){bool freeze=frozenPosable(k,*v[0])&&frozenPosable(k,*v[1])&&frozenPosable(k,*v[2]);
        if(!freeze){++s.unposable;s.tris[k].push_back(std::uint32_t(t)|0x80000000u);continue;}
        for(const auto*row:v)for(unsigned w=0;w<4;++w){const auto bb=row->bone[w];if(bb<64&&!ok[k][bb]&&gfy_signed_hw::dependency(row->weight,w,negligible_skin_weight))s.anchors[k].emplace(bb,front[k][bb]);}
        ++s.frozen;}
       s.tris[k].push_back(std::uint32_t(t));}
      continue;}}}
   // the samples: corners, centre, edge midpoints; inside a record's cut a barycentric grid every 8 mm or finer (a fragment's patch
   // of skin smaller than the triangle is never missed between samples)
   std::vector<Vec> samples={a,b,d,(a+b+d)*(1./3),(a+b)*.5,(b+d)*.5,(a+d)*.5};
   {bool near=false;const Vec cc=(a+b+d)*(1./3);const double rr=std::max({length(a-cc),length(b-cc),length(d-cc)});
    for(const auto&r:recs){const double dx=cc.x-r.f.B[0],dy=cc.y-r.f.B[1],dz=cc.z-r.f.B[2];if(std::sqrt(dx*dx+dy*dy+dz*dz)<=r.f.Rcut+rr)near=true;}
    const double e=std::max({length(b-a),length(d-b),length(a-d)});const int m=near?int(std::ceil(e/(anyBody?.012:.008))):1; /* a body burst: 12 mm (its fragments are large) */
    if(m>2)for(int i=0;i<=m;++i)for(int j=0;j<=m-i;++j){if((i==0&&j==0)||(i==m&&j==0)||(i==0&&j==m))continue;samples.push_back(a+(b-a)*(double(i)/m)+(d-a)*(double(j)/m));}}
   std::uint32_t want=0,skin=0;bool skinDone=false,region=false;
   Vec triN{0,0,0};for(int j=0;j<3;++j)triN=triN+Vec{v[j]->surface[3],v[j]->surface[4],v[j]->surface[5]};{const double ln=length(triN);triN=ln>1e-9?triN*(1./ln):Vec{0,0,0};}
   for(const auto&q:samples){std::uint32_t cand=all;bool inCut=false;
    for(const auto&r:recs){const double dx=q.x-r.f.B[0],dy=q.y-r.f.B[1],dz=q.z-r.f.B[2];const bool in=dx*dx+dy*dy+dz*dz<=r.f.Rcut*r.f.Rcut;inCut=inCut||in;
     // the group at the skin and 2 / 4 mm under it: the pieces holding the tissue right under a patch of skin get its triangle (a
     // crack between the skin and the first tissue cell never leaves a piece's tissue without its skin)
     static const double depth[4]={0.,.004,.010,.018};
     std::uint32_t o=0;for(int s=0;s<4;++s){const Vec qq=q-triN*depth[s];const unsigned g=in?r.f.label(qq.x,qq.y,qq.z):0u;o|=g<r.owners.size()?r.owners[g]:0u;if(!in)break;}
     cand&=o;}
    if(inCut)region=true;
    cand&=zoneAt(q); // a piece takes a sample only in its zone (its own tissue the nearest, one tile of slack)
    if(!cand)continue;
    // several pieces: of one group's sharers only the one whose centroid is nearest (the programs cut the group there the same way);
    // pieces of different groups ALL take the triangle - the record splits it per pixel exactly (a seam strip of the hood, its
    // normal turned, never goes to the wrong piece alone and leaves a dark line)
    if(inCut&&(cand&(cand-1)))cand=splitAt(q,cand);
    // a body burst's fragment takes the triangle only inside its drawing box (no gear sliver far from its chunk; the body keeps all)
    // (a head burst's fragment: its drawing box + 3 cm - the zone marks reach 8 cm beyond the box)
    if(inCut)for(std::size_t k=0;k<P;++k){if(!((cand>>k)&1u)||isMainPiece[k])continue;const auto&L=lim[k];const int e=anyBody?-2:int(std::ceil(.03/grid.pitch));
     const int cx=int(std::floor((q.x-grid.origin.x)/grid.pitch)),cy=int(std::floor((q.y-grid.origin.y)/grid.pitch)),cz=int(std::floor((q.z-grid.origin.z)/grid.pitch));
     if(!L.valid()||cx<L.lo[0]-e||cx>=L.hi[0]+e||cy<L.lo[1]-e||cy>=L.hi[1]+e||cz<L.lo[2]-e||cz>=L.hi[2]+e)cand&=~(1u<<k);}
    // inside a cut, several pieces holding the sample's group (one group in two pieces: a hood shell and the skull under it) ALL take
    // the triangle - each hides the other's cells and the 2 cells around them (below), so each draws only its own part of it
    if((cand&(cand-1))&&!inCut){std::uint32_t o;if(cand==all){if(!skinDone){skin=skinOwners(v,all);skinDone=true;}o=skin;}else o=skinOwners(v,cand);
     if(!o)o=cand;if(o&(o-1)){++s.ties;bool f=false;const int k=underneath(v,o,f);if(f)++s.byVolume;cand=k>=0?1u<<k:o;}else cand=o;
     ++s.bySkeleton;}
    want|=cand;
    for(std::size_t k=0;k<P;++k)if((cand>>k)&1u)for(std::size_t i=0;i<K;++i)if(R[i].side(q.x,q.y,q.z)>=0)s.touch[k][i]=1;}
   if(region)++s.inRegion;
   bool any=false;
   for(std::size_t k=0;k<P;++k){if(!((want>>k)&1u))continue;any=true;
    bool fine=anchored(k,*v[0])&&anchored(k,*v[1])&&anchored(k,*v[2]);
    if(!fine){bool freeze=frozenPosable(k,*v[0])&&frozenPosable(k,*v[1])&&frozenPosable(k,*v[2]);
     if(!freeze){++s.unposable;s.tris[k].push_back(std::uint32_t(t)|0x80000000u);continue;} // rebound to one bone of the piece (below)
     for(const auto*row:v)for(unsigned w=0;w<4;++w){const auto bb=row->bone[w];if(bb<64&&!ok[k][bb]&&gfy_signed_hw::dependency(row->weight,w,negligible_skin_weight))s.anchors[k].emplace(bb,front[k][bb]);}
     ++s.frozen;}
    s.tris[k].push_back(std::uint32_t(t));}
   if(!any)++s.dropped;}}});
 for(std::size_t ci=0;ci<prChunks.size();++ci){auto&dst=sel[prChunks[ci].c];auto&s=prPart[ci]; // piecesregional: chunk order = triangle order
  for(std::size_t k=0;k<P;++k){dst.tris[k].insert(dst.tris[k].end(),s.tris[k].begin(),s.tris[k].end());dst.anchors[k].insert(s.anchors[k].begin(),s.anchors[k].end());
   for(std::size_t i=0;i<K;++i)dst.touch[k][i]|=s.touch[k][i];}
  dst.dropped+=s.dropped;dst.unposable+=s.unposable;dst.frozen+=s.frozen;dst.inRegion+=s.inRegion;dst.bySkeleton+=s.bySkeleton;dst.byVolume+=s.byVolume;dst.ties+=s.ties;}
 stats.selectMs=core::ms(s0);const auto p0=core::Clock::now();
 for(const auto&s:sel){stats.dropped+=s.dropped;stats.unposable+=s.unposable;stats.frozen+=s.frozen;stats.inRegion+=s.inRegion;stats.bySkeleton+=s.bySkeleton;stats.byVolume+=s.byVolume;stats.ties+=s.ties;}
 long long windingVote=0;
 for(const auto&src:children){if(src.material||isBox(src))continue;for(std::size_t t=0;t<src.indices.size()/3&&t<4096;++t){
   const auto&A=src.vertices[src.indices[3*t]];const auto&Bv=src.vertices[src.indices[3*t+1]];const auto&Cc=src.vertices[src.indices[3*t+2]];
   const Vec e1=position(Bv)-position(A),e2=position(Cc)-position(A);const Vec g{e1.y*e2.z-e1.z*e2.y,e1.z*e2.x-e1.x*e2.z,e1.x*e2.y-e1.y*e2.x};
   const Vec nn{A.surface[3]+Bv.surface[3]+Cc.surface[3],A.surface[4]+Bv.surface[4]+Cc.surface[4],A.surface[5]+Bv.surface[5]+Cc.surface[5]};
   const double sw=dot(g,nn);windingVote+=sw>0?1:(sw<0?-1:0);}}
 const bool ccwOutward=windingVote>=0;
 // ---- the tears each piece keeps: its copy of every record (the groups it holds), the inherited tears it borders
 std::vector<std::vector<gfy_packed::TearRef>> keep(P);
 for(std::size_t k=0;k<P;++k)for(std::size_t t=0;t<K;++t){const auto&tr=published.tears[t];
  if(head_burst_record::is(*tr.block)){
   if(fresh(tr)){for(std::size_t ri=0;ri<recs.size();++ri){const auto&r=recs[ri];if(r.t!=t||!r.mask[k])continue;
     auto c=std::make_shared<gfy_packed::TearBlock>(*tr.block);c->header[15]=r.mask[k];
     auto F=[&](std::size_t w,double v){const float f=float(v);std::memcpy(&c->header[w],&f,4);};
     for(const auto&sp:splits){if(sp.rec!=ri||!((sp.sharers>>k)&1u))continue; // one split per copy (the first)
      unsigned n=0;F(28,sp.c[k].x);F(29,sp.c[k].y);F(30,sp.c[k].z);
      for(std::size_t j=0;j<P&&n<3;++j)if(j!=k&&((sp.sharers>>j)&1u)){F(32+4*n,sp.c[j].x);F(33+4*n,sp.c[j].y);F(34+4*n,sp.c[j].z);++n;}
      c->header[26]=(sp.g+1u)|(n<<8);
      // a split with the main far piece: the chunk's sphere (on the chunk: its own; on the body: the first other's), bit 0 = main
      if(sp.main>=0){const bool isM=int(k)==sp.main;double rr=0;
       if(isM){for(std::size_t j=0;j<P;++j)if(j!=k&&((sp.sharers>>j)&1u)){rr=sp.rad[j];break;}}else rr=sp.rad[k];
       F(22,rr);c->header[23]=isM?1u:0u;}
      break;}
     if(int(k)==r.main)c->header[23]|=2u; // bit 1: this piece is the record's main far piece (the body)
     // seethrough2 (2026-11-16): the pixel programs evaluate ONE burst record of a piece - the first of its drawing set (TearAtlas.h,
     // row 1 texel 1). A FRAGMENT of this split (not the record's main far piece) asks the record that made it: its copy goes first
     // among its burst records (its groups, torn faces, sphere and fragment flags; on a later burst it asked the body's old copy and
     // drew its whole box of the body's tissue as a cell grid). The main far piece keeps its older record first (below: stOld).
     if(int(k)!=r.main){std::size_t at=keep[k].size();for(std::size_t i=0;i<keep[k].size();++i)if(keep[k][i].block&&head_burst_record::is(*keep[k][i].block)){at=i;break;}
      keep[k].insert(keep[k].begin()+std::ptrdiff_t(at),gfy_packed::TearRef{c,3});}else
     keep[k].push_back({c,3});}}
   else if(tr.side==3)keep[k].push_back({tr.block,3});
   continue;}
  if(fresh(tr))continue; // (only() holds: no other fresh tear)
  int touched=0;for(const auto&s:sel)if(!s.touch.empty())touched|=s.touch[k][t];
  const bool voted=tr.side<2?stats.votes[t][std::size_t(tr.side)*maxPieces+k]>0:(stats.votes[t][k]+stats.votes[t][maxPieces+k])>0;
  if(voted||touched)keep[k].push_back({tr.block,tr.side});}
 // ---- one group in two pieces: each hides the other's cells (the topology's spans in the records' boxes)
 std::vector<std::uint32_t> hideFrom(P,0u); // pieces whose cells piece k hides
 for(const auto&r:recs)for(std::size_t g=0;g<r.owners.size();++g){const std::uint32_t o=r.owners[g]&~[&]{std::uint32_t m=0;for(std::size_t k=0;k<P;++k)if(!r.mask[k])m|=1u<<k;return m;}();
  if(o&(o-1))for(std::size_t k=0;k<P;++k)if((o>>k)&1u)hideFrom[k]|=o&~(1u<<k);}
 // seethrough2 (2026-11-16): a piece whose first burst record is still an older one (the body of a later burst: stOld) does not
 // evaluate this split's record. Its crack cells (erased only to part the pieces, given back for the record to tear) and the other
 // pieces' cells would count as its own tissue: its box ray met the cells a fragment took (their skin went with it) from the open
 // air, nothing drew them and the floor showed through the fresh crater. Such a piece gets no crack cells back and marks the other
 // pieces' cells in the records' cut boxes (below): its rays pass them and meet its own crater wall.
 std::uint32_t stOld=0;
 for(std::size_t k=0;k<P;++k){const gfy_packed::TearBlock*first=nullptr;
  for(const auto&tr:keep[k])if(tr.block&&head_burst_record::is(*tr.block)){first=tr.block.get();break;}
  if(!first)continue;
  for(const auto&tr:published.tears)if(!fresh(tr)&&tr.block.get()==first){stOld|=1u<<k;break;}}
 std::vector<std::unordered_map<Cell,core::Tile,core::CellHash>> extra(P);
 auto markShared=[&](bool finishDetached){bool any=false;for(auto h:hideFrom)any=any||h;
  if(any){std::vector<int> pieceOf(volume.components.size(),-1);for(std::size_t k=0;k<P&&k<maxPieces;++k)if(components[k]<pieceOf.size())pieceOf[components[k]]=int(k);
   // only inside the piece's own cells' box + 4 cells: the programs search the volume there alone (TearAtlas row 0, + 2 cells), and
   // its triangles lie over its own cells (the selection above) - a small chunk sharing the far group never marks the whole neck
   // (lim: above)
   for(const auto&r:recs){int lo[3],hi[3];for(int a=0;a<3;++a){lo[a]=std::max(0,int(std::floor((r.f.B[a]-r.f.Rcut-grid.origin[a])/grid.pitch)));
     hi[a]=std::min((&grid.size.x)[a]-1,int(std::floor((r.f.B[a]+r.f.Rcut-grid.origin[a])/grid.pitch)));}
    for(const auto&col:volume.columns){if(col.x<lo[0]||col.x>hi[0]||col.y<lo[1]||col.y>hi[1])continue;
     for(auto si=col.first;si<col.first+col.count;++si){const auto&sp=volume.spans[si];if(sp.component>=pieceOf.size())continue;const int j=pieceOf[sp.component];if(j<0)continue;
      for(std::size_t k=0;k<P;++k){if(bool(deferred[k])!=finishDetached || !((hideFrom[k]>>j)&1u))continue;const auto&L=lim[k];
       if(!L.valid()||col.x<L.lo[0]||col.x>=L.hi[0]||col.y<L.lo[1]||col.y>=L.hi[1])continue;
       for(int z=std::max({sp.lo,lo[2],L.lo[2]});z<std::min({sp.hi,hi[2]+1,L.hi[2]});++z){const Cell c{col.x,col.y,z};auto&tile=extra[k][Grid::tileKey(c)];const auto l=Grid::local(c);tile.removed[l/64]|=std::uint64_t(1)<<(l%64);}}}}}
   // and 2 cells around them that are not the piece's own (the air at the other piece's skin: its exterior there is not this piece's)
   for(std::size_t k=0;k<P;++k){if(bool(deferred[k])!=finishDetached || !hideFrom[k])continue;std::vector<Cell> cells;
    gatherHaloFrontier(extra[k],cells);
    for(int it=0;it<2;++it){std::vector<Cell> next;
     for(const Cell&c:cells){const Cell nb[6]={{c.x-1,c.y,c.z},{c.x+1,c.y,c.z},{c.x,c.y-1,c.z},{c.x,c.y+1,c.z},{c.x,c.y,c.z-1},{c.x,c.y,c.z+1}};
      for(const Cell&n:nb){if(!grid.valid(n))continue;auto&tile=extra[k][Grid::tileKey(n)];const auto l=Grid::local(n);if((tile.removed[l/64]>>(l%64))&1u)continue;
       const Vec q=grid.center(n);if(pieceAt(q.x,q.y,q.z)==int(k))continue;tile.removed[l/64]|=std::uint64_t(1)<<(l%64);next.push_back(n);}}
     cells.swap(next);}}}
 // seethrough2: a piece whose first burst record is an older one marks the other pieces' cells inside this split's cut boxes (within
 // its own cells' box + 4 cells, lim) - no air ring (that is for a shared group's skin)
 if(stOld){std::vector<int> pieceOf2(volume.components.size(),-1);for(std::size_t k=0;k<P&&k<maxPieces;++k)if(components[k]<pieceOf2.size())pieceOf2[components[k]]=int(k);
  std::size_t stMarked=0;
  for(const auto&r:recs){int lo[3],hi[3];for(int a=0;a<3;++a){lo[a]=std::max(0,int(std::floor((r.f.B[a]-r.f.Rcut-grid.origin[a])/grid.pitch)));
    hi[a]=std::min((&grid.size.x)[a]-1,int(std::floor((r.f.B[a]+r.f.Rcut-grid.origin[a])/grid.pitch)));}
   for(const auto&col:volume.columns){if(col.x<lo[0]||col.x>hi[0]||col.y<lo[1]||col.y>hi[1])continue;
    for(auto si=col.first;si<col.first+col.count;++si){const auto&sp=volume.spans[si];if(sp.component>=pieceOf2.size())continue;const int j=pieceOf2[sp.component];if(j<0)continue;
     for(std::size_t k=0;k<P;++k){if(bool(deferred[k])!=finishDetached || !((stOld>>k)&1u)||int(k)==j)continue;const auto&L=lim[k];
      if(!L.valid()||col.x<L.lo[0]||col.x>=L.hi[0]||col.y<L.lo[1]||col.y>=L.hi[1])continue;
      for(int z=std::max({sp.lo,lo[2],L.lo[2]});z<std::min({sp.hi,hi[2]+1,L.hi[2]});++z){const Cell c{col.x,col.y,z};auto&tile=extra[k][Grid::tileKey(c)];const auto l=Grid::local(c);
       tile.removed[l/64]|=std::uint64_t(1)<<(l%64);++stMarked;}}}}}
  if(std::getenv("GFY_M3_LOG")||std::getenv("GFY_PIECE_LOG"))std::cerr<<"seethrough2 older_record_pieces="<<std::hex<<stOld<<std::dec<<" marked="<<stMarked<<std::endl;}};
 markShared(false);
 std::size_t farTriTiles=0,farTris=0;double farTriMs=0;bool olderRecord=false;
 for(std::size_t t=0;t<K;++t){const auto&tr=published.tears[t];if(!tr.block||!head_burst_record::is(*tr.block))continue;if(fresh(tr))break;if(tr.side==3){olderRecord=true;break;}}
 auto markFar=[&](bool finishDetached){if(olderRecord&&zones.ok&&!std::getenv("GFY_HB_NO_FARTRI")){const auto f0=core::Clock::now();const int reachF=int(std::ceil(.08/grid.pitch));
  std::vector<std::size_t> farN(P,0);
  lowpoly_pieces::eachBlock(P,1,[&](std::size_t kf,std::size_t kl,std::size_t){for(std::size_t k=kf;k<kl;++k){if(bool(deferred[k])!=finishDetached || isMainPiece[k])continue;const CellBox&Lb=lim[k];if(!Lb.valid())continue;
    std::vector<std::uint64_t> tiles; // packed tiles under the far-reaching triangles, outside the zone
    auto cellOf=[&](const Vec&q,int c[3]){c[0]=int(std::floor((q.x-grid.origin.x)/grid.pitch));c[1]=int(std::floor((q.y-grid.origin.y)/grid.pitch));c[2]=int(std::floor((q.z-grid.origin.z)/grid.pitch));};
    for(std::size_t c=0;c<children.size();++c){if(sel[c].tris.size()<=k)continue;const auto&src=children[c];
     for(const std::uint32_t e:sel[c].tris[k]){const std::size_t tt=e&0x7fffffffu;if(3*tt+2>=src.indices.size())continue;
      const Vec a=position(src.vertices[src.indices[3*tt]]),b=position(src.vertices[src.indices[3*tt+1]]),d=position(src.vertices[src.indices[3*tt+2]]);
      bool beyond=false;for(const Vec*q:{&a,&b,&d}){int cc[3];cellOf(*q,cc);for(int ax=0;ax<3;++ax)beyond=beyond||cc[ax]<Lb.lo[ax]-reachF||cc[ax]>=Lb.hi[ax]+reachF;}
      if(!beyond)continue;++farN[k];
      const double len=std::max({length(b-a),length(d-b),length(a-d)});const int m=std::max(1,std::min(512,int(std::ceil(len/.008))));
      for(int i=0;i<=m;++i)for(int j=0;i+j<=m;++j){const Vec q=a+(b-a)*(double(i)/m)+(d-a)*(double(j)/m);int cc[3];cellOf(q,cc);
       if(cc[0]<0||cc[1]<0||cc[2]<0||cc[0]>=grid.size.x||cc[1]>=grid.size.y||cc[2]>=grid.size.z)continue;const int tx=cc[0]>>3,ty=cc[1]>>3,tz=cc[2]>>3;
       if((zones.at(tx,ty,tz)>>k)&1u)continue;const std::uint64_t key=(std::uint64_t(tx)<<42)|(std::uint64_t(ty)<<21)|std::uint64_t(tz);
       if(tiles.empty()||tiles.back()!=key)tiles.push_back(key);}}}
    std::sort(tiles.begin(),tiles.end());tiles.erase(std::unique(tiles.begin(),tiles.end()),tiles.end());
    for(const auto key:tiles)zoneMarks[k].push_back(Cell{int(key>>42),int((key>>21)&0x1fffffu),int(key&0x1fffffu)});
    farN[k]|=tiles.size()<<32;}});
  for(std::size_t k=0;k<P;++k){farTriTiles+=farN[k]>>32;farTris+=farN[k]&0xffffffffu;}
  farTriMs=core::ms(f0);
  if(std::getenv("GFY_M3_LOG")||std::getenv("GFY_PIECE_LOG"))std::cerr<<"head_burst_far_triangles triangles="<<farTris<<" tiles="<<farTriTiles<<" ms="<<farTriMs<<std::endl;}};
 markFar(false);
 // ---- the marks of the two rules above (connected tissue; zones): into each piece's removal field
 std::size_t gearSeen=islandCells,gearMarks=zoneTiles;double gearMs=floodMs+zoneMs;
 auto mergeMarks=[&](bool finishDetached){for(std::size_t k=0;k<P;++k){
  if(bool(deferred[k])!=finishDetached)continue;
  for(const auto&m:floodMarks[k]){auto&tile=extra[k][m.first];for(unsigned q=0;q<8;++q)tile.removed[q]|=m.second.removed[q];}
  for(const Cell&c:zoneMarks[k]){auto&tile=extra[k][c];for(auto&wd:tile.removed)wd=~std::uint64_t(0);}}};
 mergeMarks(false);
 // headmask2: the boolean remove (unattached, above) - every piece of the burst inside the cut's box
 auto attach=[&](bool finishDetached){AttachStats ast;std::size_t big=0;for(std::size_t k=1;k<P;++k)if(volume.components[components[k]].cells>volume.components[components[big]].cells)big=k;
  // a burst's cut is more than its fragments: the box of the cells it broke out (the crack tiles, 16 cells out) is part of the
  // cut's box - a face blown away is no piece's, and the mask over it stood 2 to 3 cm off, outside the fragments' boxes; a
  // fragment is handed triangles up to 4 to 5 cm from its cells (the nearest centroid), so its own pass reaches 32 cells
  const int fragGrow=32,bodyGrow=16;const bool crackBox=true,burstAir=false;
  CellBox crackB;if(crackBox)for(const auto&ck:crack){const Cell&tk=ck.first;crackB.add(tk.x*8,tk.y*8,tk.z*8,tk.z*8+8);crackB.add(tk.x*8+7,tk.y*8+7,tk.z*8,tk.z*8+8);}
  if(crackB.valid())for(int a=0;a<3;++a){crackB.lo[a]-=16;crackB.hi[a]+=16;}
  CellBox cutB=cutBox(ownB,big);cutB.add(crackB);const gear_cells::GearCells*ag=gear&&gear->fits(grid.origin,grid.pitch,grid.size)?gear:nullptr;
  const auto apad=tfF?gear_cells::padTileFlags(published.tissue_field,grid):std::shared_ptr<const std::vector<std::uint8_t>>();
  const std::vector<Cell> agone=P>=2?goneBlocks(grid,tfF,cutB):std::vector<Cell>();
  PadCache apads;
  if(P>=2){auto cutBs=cutBoxes(ownB,big);addCluster(cutBs,crackB);for(std::size_t k=0;k<P;++k){ // the body: each cluster of the cut's boxes
   if(bool(deferred[k])!=finishDetached)continue;
   if(k!=big){const CellBox pb=attachBox(ownB[k],fragGrow);unattached(grid,volume,pieceAt.pieceOf,k,pb,ag,apad.get(),tfF,agone,extra[k],NoZone{},ast,ag?std::vector<Cell>():shellCells(grid,children,sel,k,attachBox(pb,4)),apads,inherited,&crack,burstAir);continue;}
   for(const CellBox&cb:cutBs){const CellBox pb=attachClip(cb,ownB[k],bodyGrow);unattached(grid,volume,pieceAt.pieceOf,k,pb,ag,apad.get(),tfF,agone,extra[k],NoZone{},ast,ag?std::vector<Cell>():shellCells(grid,children,sel,k,attachBox(pb,4)),apads,inherited,&crack,burstAir);}}}
  if(std::getenv("GFY_M3_LOG")||std::getenv("GFY_PIECE_LOG"))std::cerr<<"boolean_remove burst pieces="<<ast.pieces<<" blocks="<<ast.marked<<" skipped="<<ast.skipped<<" gone_blocks="<<agone.size()<<" ms="<<ast.ms
   <<" islands="<<ast.islands<<" medium="<<ast.medium<<" shell="<<ast.shell<<" pad_tiles="<<ast.padTiles<<" island_ms="<<ast.islandMs<<std::endl;};
 attach(false);
 // seethrough: the burst boxes the body already has, each to the piece that keeps its crater (carryBoxes, above build())
 const auto stCarry=carryBoxes(children,pieceAt,P,ok);
 // hollowleg / cutends (2026-11-16): the body's cut-end children (a severed limb's or a shot-off head's end: no skin over the cut)
 // go, unchanged, to the piece that keeps that end (cutEndCarry above); a burst split makes none of them again
 const auto hlCut=cutEndCarry(published,children,pieceAt,P,ok,[&](std::size_t k,const gfy_packed::TearBlock*T){for(const auto&tr:keep[k])if(tr.block.get()==T)return true;return false;},
  [](const gfy_packed::TearBlock*){return false;});
 if((hlCut.boxes||hlCut.starters||hlCut.dropped)&&(std::getenv("GFY_M3_LOG")||std::getenv("GFY_PIECE_LOG")))
  std::cerr<<"cutends burst_split boxes="<<hlCut.boxes<<" starters="<<hlCut.starters<<" dropped="<<hlCut.dropped<<std::endl;
 // ---- assemble
 std::vector<bound_surface::Piece> out(P);std::vector<std::uint32_t> empty;
 if(marksOut)marksOut->assign(P,{});
 // (parallel over the pieces: each writes only its own piece; counters and atlas stats merged after, in piece order)
 struct PieceStats {std::size_t selected=0,boxTriangles=0,boxes=0,fieldBytes=0,kept=0;bool empty=false;tear_atlas::Stats atlas;};std::vector<PieceStats> ps(P);
 auto assemble=[&](bool finishDetached){
 lowpoly_pieces::eachBlock(P,1,[&](std::size_t kf,std::size_t kl,std::size_t){for(std::size_t k=kf;k<kl;++k){
  if(finishDetached && !deferred[k])continue;
  auto&S=ps[k];auto&piece=out[k];piece.volume_component=components[k];auto&mesh=piece.mesh;
  if(!finishDetached){
  unsigned next=originalChildren; // splitfix: appended slots start right after the original children in every piece; numbered above the source's own appended slots, a cut part's pieces left a gap the native packet check refuses (no re-split was ever staged)
  for(std::size_t c=0;c<children.size();++c){const auto&src=children[c];const auto&list=sel[c].tris.empty()?std::vector<std::uint32_t>():sel[c].tris[k];if(list.empty())continue;
   for(const auto&a:sel[c].anchors[k]){auto [entry,freshA]=piece.constrained_anchors.emplace(a);need(freshA||entry->second==a.second,"head_burst_pieces_frozen_anchor_conflict");}
   gfy_packed::Child child;child.material=0;child.material_source_child=src.material_source_child;child.skin_clip=true;
   if(src.child_index<originalChildren)child.child_index=src.child_index;else{need(next<gfy_packed::max_child_slots,"head_burst_pieces_child_slots");child.child_index=std::uint16_t(next++);}
   std::vector<std::int32_t> map(src.vertices.size(),-1);const bool faces=src.original_faces.size()*3==src.indices.size();
   // (a triangle marked 0x80000000: the piece cannot pose it - skin weighted to bones it does not hold, a hood vertex on the neck: its
   // corners follow ONE bone of the piece rigidly, the anchorable bone with the most weight on the vertex, else the piece's first)
   std::vector<std::int32_t> mapR(src.vertices.size(),-1);
   auto rebound=[&](gfy_packed::Vertex row){unsigned best=64;float bw=-1;for(unsigned w=0;w<4;++w){const auto b=row.bone[w];if(b<64&&ok[k][b]&&row.weight[w]>bw){bw=row.weight[w];best=b;}}
    if(best==64)for(unsigned b=0;b<64;++b)if(ok[k][b]){best=b;break;}if(best==64)return row;
    row.bone={std::uint16_t(best),std::uint16_t(best),std::uint16_t(best),std::uint16_t(best)};row.weight={1.f,0.f,0.f,0.f};gfy_packed::canonical_weights(row);return row;};
   for(const auto tt:list){const bool rb=(tt&0x80000000u)!=0;const auto t=tt&0x7fffffffu;for(int j=0;j<3;++j){const auto v=src.indices[3*t+j];auto&mp=rb?mapR:map;
     if(mp[v]<0){mp[v]=std::int32_t(child.vertices.size());child.vertices.push_back(rb?rebound(src.vertices[v]):src.vertices[v]);}
     child.indices.push_back(std::uint16_t(mp[v]));}
    child.original_faces.push_back(faces?src.original_faces[t]:~std::uint32_t(0));}
   need(child.vertices.size()<=65535,"head_burst_pieces_vertex_budget");
   piece.triangles+=list.size();S.selected+=list.size();mesh.children.push_back(std::move(child));}
  std::size_t boxTris=0;
  // coverage boxes: per record the piece's part of the burst, axis-aligned in rest space: a fragment's own cells (3.5 cm out); the
  // main far piece the cells of every piece without the far part (the broken region, 3.5 cm out); a far chunk its own cells
  if(!stCarry[k].empty()){ // seethrough: the fresh boxes (as below) and the boxes this piece keeps, merged (above build())
   std::vector<CarryBox> L;
   auto nearestBone=[&](double cx,double cy,double cz,std::uint16_t&bone,std::uint16_t&src){const Vec cc{cx,cy,cz};double bestD=1e300;bone=65535;
    for(const auto&ch:mesh.children)for(const auto&row:ch.vertices){unsigned best=0;for(unsigned w=1;w<4;++w)if(row.weight[w]>row.weight[best])best=w;
     const double dd=length(position(row)-cc);if(dd<bestD&&row.bone[best]<64&&ok[k][row.bone[best]]){bestD=dd;bone=row.bone[best];src=ch.material_source_child;}}
    if(bone==65535)for(unsigned bb=0;bb<64;++bb)if(ok[k][bb]){bone=std::uint16_t(bb);break;}};
   for(const auto&r:recs){if(!r.mask[k])continue;std::array<double,6> b=r.cellBox[k];double pad=.035;
    if((r.mask[k]&1u)&&int(k)==r.main){b={1e30,1e30,1e30,-1e30,-1e30,-1e30};for(std::size_t j=0;j<P;++j)if(r.mask[j]&&!(r.mask[j]&1u))for(int q=0;q<3;++q){b[q]=std::min(b[q],r.cellBox[j][q]);b[3+q]=std::max(b[3+q],r.cellBox[j][3+q]);}pad=.035;}
    if(!(b[3]>=b[0]))b=r.box[k];if(!(b[3]>=b[0]))continue;
    CarryBox x;for(std::size_t a=0;a<3;++a){x.b[a]=b[a]-pad;x.b[3+a]=b[3+a]+pad;}
    nearestBone(.5*(b[0]+b[3]),.5*(b[1]+b[4]),.5*(b[2]+b[5]),x.bone,x.src);if(x.bone==65535)continue;L.push_back(x);}
   const std::size_t freshBoxes=L.size();
   for(auto x:stCarry[k]){if(x.bone>=64||!ok[k][x.bone]){std::uint16_t s2=x.src;nearestBone(.5*(x.b[0]+x.b[3]),.5*(x.b[1]+x.b[4]),.5*(x.b[2]+x.b[5]),x.bone,s2);if(x.bone==65535)continue;}
    L.push_back(x);}
   const std::size_t keptBoxes=L.size()-freshBoxes;
   mergeCarryBoxes(L,next<gfy_packed::max_child_slots?std::size_t(gfy_packed::max_child_slots-next):std::size_t(0));
   for(const auto&x:L){const std::size_t before=boxTris;emitCarryBox(mesh,x,next,ccwOutward,boxTris);S.boxTriangles+=boxTris-before;++S.boxes;}
   if(std::getenv("GFY_M3_LOG")||std::getenv("GFY_PIECE_LOG"))std::cerr<<"seethrough_boxes burst piece="<<k<<" fresh="<<freshBoxes<<" kept="<<keptBoxes<<" drawn="<<L.size()<<std::endl;}
  else
  for(const auto&r:recs){if(!r.mask[k])continue;std::array<double,6> b=r.cellBox[k];double pad=.035; // (beyond the drawing box: a box face never lies inside the chunk's padding)
   if((r.mask[k]&1u)&&int(k)==r.main){b={1e30,1e30,1e30,-1e30,-1e30,-1e30};for(std::size_t j=0;j<P;++j)if(r.mask[j]&&!(r.mask[j]&1u))for(int q=0;q<3;++q){b[q]=std::min(b[q],r.cellBox[j][q]);b[3+q]=std::max(b[3+q],r.cellBox[j][3+q]);}pad=.035;}
   if(!(b[3]>=b[0]))b=r.box[k];if(!(b[3]>=b[0]))continue;
   painter_mask_pieces::Box box;for(int a=0;a<3;++a){box.centre[std::size_t(a)]=.5*(b[a]+b[3+a]);box.axis[0][std::size_t(a)]=a==0;box.axis[1][std::size_t(a)]=a==2;box.axis[2][std::size_t(a)]=a==1;}
   box.lo={-(.5*(b[3]-b[0])+pad),-(.5*(b[5]-b[2])+pad),-(.5*(b[4]-b[1])+pad)};box.hi={-box.lo[0],-box.lo[1],-box.lo[2]};box.valid=true;
   if(next>=gfy_packed::max_child_slots)continue;
   const Vec boxCentre{box.centre[0],box.centre[1],box.centre[2]};double bestD=1e300;std::uint16_t boxBone=65535,boxMaterialSource=0;
   for(const auto&ch:mesh.children)for(const auto&row:ch.vertices){unsigned best=0;for(unsigned w=1;w<4;++w)if(row.weight[w]>row.weight[best])best=w;
    const double dd=length(position(row)-boxCentre);if(dd<bestD&&row.bone[best]<64&&ok[k][row.bone[best]]){bestD=dd;boxBone=row.bone[best];boxMaterialSource=ch.material_source_child;}}
   if(boxBone==65535)for(unsigned b=0;b<64;++b)if(ok[k][b]){boxBone=std::uint16_t(b);break;} // a fragment with no skin (inside the head): one of its bones
   if(boxBone==65535)continue;
   const auto corner=painter_mask_pieces::corners(box);const auto tri=painter_mask_pieces::triangles(box);
   gfy_packed::Child child;child.material=0;child.material_source_child=boxMaterialSource;child.skin_clip=true;child.child_index=std::uint16_t(next++);
   for(std::size_t f=0;f<12;++f){const std::uint16_t q[3]={tri[3*f],tri[3*f+1],tri[3*f+2]};
    const Vec A{corner[q[0]][0],corner[q[0]][1],corner[q[0]][2]},Bq{corner[q[1]][0],corner[q[1]][1],corner[q[1]][2]},Cq{corner[q[2]][0],corner[q[2]][1],corner[q[2]][2]};
    Vec nn{(Bq-A).y*(Cq-A).z-(Bq-A).z*(Cq-A).y,(Bq-A).z*(Cq-A).x-(Bq-A).x*(Cq-A).z,(Bq-A).x*(Cq-A).y-(Bq-A).y*(Cq-A).x};const double ln=length(nn);if(!(ln>1e-12))continue;nn=nn*(1./ln);
    const bool outward=dot(nn,A-boxCentre)>0;
    const std::uint16_t order[3]={q[0],outward==ccwOutward?q[1]:q[2],outward==ccwOutward?q[2]:q[1]};
    const Vec no=outward?nn:nn*-1.;const Vec tg=std::abs(no.y)<.9?Vec{no.z,0,-no.x}:Vec{0,-no.z,no.y};const Vec tn=tg*(1./length(tg));
    const Vec bn{no.y*tn.z-no.z*tn.y,no.z*tn.x-no.x*tn.z,no.x*tn.y-no.y*tn.x};
    for(const auto qi:order){gfy_packed::Vertex row;
     const double*pp=corner[qi].data();const double sv[14]={pp[0],pp[1],pp[2],no.x,no.y,no.z,tn.x,tn.y,tn.z,bn.x,bn.y,bn.z,-1000.,0.};
     for(unsigned a=0;a<14;++a)row.surface[a]=float(sv[a]);
     row.bone={boxBone,boxBone,boxBone,boxBone};row.weight={1.f,0.f,0.f,0.f};gfy_packed::canonical_weights(row);
     child.indices.push_back(std::uint16_t(child.vertices.size()));child.vertices.push_back(row);}
    child.original_faces.push_back(~std::uint32_t(0));}
   S.boxTriangles+=child.indices.size()/3;++S.boxes;boxTris+=child.indices.size()/3;mesh.children.push_back(std::move(child));}
  // hollowleg / cutends: the body's cut-end children this piece keeps, as the body had them
  for(const auto c:hlCut.of[k]){const std::size_t t=emitCutChild(mesh,children[c],next,ok[k]);boxTris+=t;S.boxTriangles+=t;if(t)++S.boxes;}
  if(!piece.triangles)piece.triangles=boxTris; } // a fragment inside the head is drawn by its box alone
  if(!piece.triangles){S.empty=true;continue;}
  if(!finishDetached && deferred[k]){
   mesh.removal_field=published.removal_field;mesh.tissue_field=published.tissue_field;
   mesh.mask_volume=true;mesh.mask_piece=true;mesh.tears=keep[k];
   if(inherited)(*marksOut)[k]=*inherited;
   continue;
  }
  // the removal field: the grid's removed tiles + the inherited marks + the cells of the pieces it hides (one group in two pieces)
  struct FieldTiles {std::unordered_map<Cell,core::Tile,core::CellHash> tiles;const void* ownership=nullptr;double pitch;Vec origin;decltype(grid.size) size;};
  FieldTiles field{{},nullptr,grid.pitch,grid.origin,grid.size};
  // (a fragment: only the tiles of its drawing box + 2 cells - the programs never read its field farther out; the body: all)
  bool isMainK=false;for(const auto&r:recs)isMainK=isMainK||int(k)==r.main;
  auto inLim=[&](const Cell&tk){const auto&L=lim[k];return isMainK||(L.valid()&&tk.x*8+8>L.lo[0]&&tk.x*8<L.hi[0]&&tk.y*8+8>L.lo[1]&&tk.y*8<L.hi[1]&&tk.z*8+8>L.lo[2]&&tk.z*8<L.hi[2]);};
  for(const auto&t:grid.tiles){if(!inLim(t.first))continue;std::uint64_t any=0;for(auto wd:t.second.removed)any|=wd;if(any)field.tiles.emplace(t.first,t.second);}
  // the crack cells back (inside the piece's own cells' box + 4 cells; the main far piece: all): the record's function draws the torn
  // faces there (the cells were erased only to part the pieces)
  if(!((stOld>>k)&1u)){const CellBox&lb=lim[k];bool all=false;for(const auto&r:recs)all=all||int(k)==r.main; // seethrough2: not on an stOld piece
   for(const auto&ct:crack){const Cell&tk=ct.first;if(!all&&(!lb.valid()||tk.x*8+8<=lb.lo[0]||tk.x*8>=lb.hi[0]||tk.y*8+8<=lb.lo[1]||tk.y*8>=lb.hi[1]||tk.z*8+8<=lb.lo[2]||tk.z*8>=lb.hi[2]))continue;
    const auto it=field.tiles.find(tk);if(it==field.tiles.end())continue;for(unsigned q=0;q<8;++q)it->second.removed[q]&=~ct.second.removed[q];}}
  if(inherited)for(const auto&m:*inherited){if(!inLim(m.first))continue;auto&t=field.tiles[m.first];for(unsigned q=0;q<8;++q)t.removed[q]|=m.second.removed[q];}
  for(const auto&m:extra[k]){auto&t=field.tiles[m.first];for(unsigned q=0;q<8;++q)t.removed[q]|=m.second.removed[q];}
  auto removal=painter_detail::buildDamageField(field);need(removal&&removal->available,"head_burst_pieces_removal_field");
  S.fieldBytes+=removal->bytes();
  if(marksOut){if(inherited)(*marksOut)[k]=*inherited;for(const auto&m:extra[k])(*marksOut)[k].emplace_back(m.first,m.second);}
  mesh.removal_field=std::move(removal);mesh.tissue_field=published.tissue_field;mesh.mask_volume=true;mesh.mask_piece=true;
  // the drawing set (TearAtlas.h): the piece's own cells' box bounds every search into the volume (the record hides the other
  // pieces' groups everywhere, so no other piece's cell needs marking); inherited tears' depths scaled by the piece's thickness
  mesh.tears=keep[k];S.kept+=keep[k].size();{bool isMain=false;for(const auto&r:recs)isMain=isMain||int(k)==r.main;const auto ownW=worldBox(grid,ownB[k],isMain?0:padCells);const auto th=thickness(grid,volume,components[k],ownB[k],mesh.tears);
   // gearback: the main far piece (the body) draws the outfit: its exterior pixels' rays search the box of everything it draws (its
   // exterior triangles too: the pack, pouches - filled in the volume, outside the tissue box); its field holds every tile (above).
   // A fragment searches its drawing box as before.
   tear_atlas::Box exteriorBox;
   if(isMain&&ownW.valid()){exteriorBox=ownW;for(const auto&ch:mesh.children)for(const auto&v:ch.vertices){if(v.surface[12]<=-999.f)continue;const double q[3]={v.surface[0],v.surface[1],v.surface[2]};exteriorBox.add(q);}
    exteriorBox=exteriorBox.padded(grid.pitch);}
   mesh.tear_atlas=tear_atlas::build(mesh.tears,ownW,ownW.padded(.08),&S.atlas,&th,exteriorBox.valid()?&exteriorBox:nullptr);}}});};
 assemble(false);
 if(flight){
  const bool complete=std::none_of(ps.begin(),ps.end(),[](const PieceStats& p){return p.empty;});
  if(complete && !sep::cancelRequested()){
   struct Reset {FlightPass* p;~Reset(){p->ids=nullptr;p->surfaces=nullptr;p->marks=nullptr;p->consumed=false;}} reset{pass};
   pass->ids=&components;pass->surfaces=&out;pass->marks=marksOut;pass->consumed=false;
   try{pass->admit();}catch(...){/* A preview allocation failure leaves the exact worker available. */}
  }
  markShared(true);markFar(true);mergeMarks(true);attach(true);assemble(true);
 }
 for(std::size_t k=0;k<P;++k){const auto&S=ps[k];stats.selected+=S.selected;stats.boxTriangles+=S.boxTriangles;stats.boxes+=S.boxes;stats.fieldBytes+=S.fieldBytes;stats.kept+=S.kept;
  stats.atlas.tears+=S.atlas.tears;stats.atlas.bytes+=S.atlas.bytes;stats.atlas.indexCells+=S.atlas.indexCells;stats.atlas.overlap3+=S.atlas.overlap3;
  stats.atlas.listEntries+=S.atlas.listEntries;stats.atlas.maxList=std::max(stats.atlas.maxList,S.atlas.maxList);stats.atlas.pruned+=S.atlas.pruned;
  stats.atlas.sets+=S.atlas.sets;stats.atlas.ms+=S.atlas.ms;if(S.empty)empty.push_back(components[k]);}
 stats.packMs=core::ms(p0);stats.ms=core::ms(t0);
 if(std::getenv("GFY_M3_LOG")||std::getenv("GFY_PIECE_LOG")){
  std::cerr<<"head_burst_pieces pieces="<<P<<" empty="<<empty.size()<<" crack_cells="<<crackCells<<" restore_extra="<<restoreExtra<<" crack_tiles="<<crack.size()<<" splits="<<splits.size()<<" split_ms="<<splitMs<<" island_cells="<<gearSeen<<" zone_tiles="<<gearMarks<<" zone_seeds="<<zoneSeeds<<" zone_nobody="<<zoneNobody<<" flood_ms="<<floodMs<<" zone_ms="<<zoneMs<<" marks_ms="<<gearMs<<" triangles=";for(std::size_t k=0;k<P;++k)std::cerr<<(k?",":"")<<out[k].triangles;
  std::cerr<<" cells=";for(std::size_t k=0;k<P;++k)std::cerr<<(k?",":"")<<(components[k]<volume.components.size()?volume.components[components[k]].cells:0);std::cerr<<std::endl;
  for(const auto&r:recs){std::cerr<<"head_burst_pieces record="<<r.t<<" main="<<r.main<<" masks=";for(std::size_t k=0;k<P;++k)std::cerr<<(k?",":"")<<r.mask[k];
   std::cerr<<" owners=";for(std::size_t g=0;g<r.owners.size();++g)std::cerr<<(g?",":"")<<r.owners[g];std::cerr<<" hide=";for(std::size_t k=0;k<P;++k)std::cerr<<(k?",":"")<<hideFrom[k];
   std::cerr<<" select_ms="<<stats.selectMs<<" ms="<<stats.ms<<std::endl;}}
 if(!empty.empty())throw lowpoly_pieces::EmptyPieces(std::move(empty));
 return out;
}
} // namespace head_burst_pieces
