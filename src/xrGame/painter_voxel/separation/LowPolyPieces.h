#pragma once
namespace lowpoly_pieces {
struct EmptyPieces:std::runtime_error{std::vector<std::uint32_t> components;explicit EmptyPieces(std::vector<std::uint32_t> ids):std::runtime_error("lowpoly_piece_empty"),components(std::move(ids)){}};
// Jobs of very uneven cost (a few long wound-wall ranges among many cheap skin ranges; mask tiles at a break among
// tiles with one piece around): every thread takes the next block of jobs when it finishes one, so no thread is left
// holding all the expensive ones. body(first,last,chunk) as core::parallelChunks; each job writes only its own slots.
template<class F>void eachBlock(std::size_t n,std::size_t block,F&&body){const std::size_t blocks=(n+block-1)/block;std::atomic<std::size_t> next{0};
 core::parallelChunks(std::min<std::size_t>(blocks,core::parallelWidth()),1,[&](std::size_t,std::size_t,std::size_t chunk){
  for(std::size_t b;(b=next.fetch_add(1,std::memory_order_relaxed))<blocks;)body(b*block,std::min(n,(b+1)*block),chunk);});}
struct Stats {std::size_t cellSupportAdmitted{},cellSupportRefused{},cellSupportBytes{};std::size_t skin{},channel{},dropped{},unanchored{},boundary{},tiles{},foreign{},marched{},byBone{},unowned{},nearest{},channelDropped{},frozen{};
 std::size_t keptShell{};std::string shellByPiece;/* E5: kept skin triangles with no tissue under any corner; per piece kept/shell/visible/emptied/cm2@centre */double ownerMs{},maskMs{},ms{},vertexMs{},ringMs{},triangleMs{};};
inline bool available(const gfy_packed::Mesh*m){
 return m&&m->analytic_channels&&m->removal_field&&m->removal_field->available&&m->tissue_field&&!m->children.empty();
}
struct Lookup {
 const Grid&g;const gfy_connectivity::Topology&t;const std::vector<int>&piece;const std::vector<std::int32_t>*columnOf=nullptr;
 // R70: the builder's 8^3 tile map (0 = no piece tissue in the tile) and its size; null: no early answers
 const std::vector<std::uint16_t>*tiles=nullptr;int TX=0,TY=0,TZ=0;
 // Exactly t.at(x,y,z): the column found by index instead of a search.
 std::uint32_t component(int x,int y,int z)const{
  if(!columnOf)return t.at(x,y,z);
  if(x<0||y<0||x>=g.size.x||y>=g.size.y)return 0;const auto ci=(*columnOf)[std::size_t(y)*std::size_t(g.size.x)+std::size_t(x)];if(ci<0)return 0;
  const auto&c=t.columns[std::size_t(ci)];const auto first=t.spans.begin()+c.first,last=first+c.count;
  auto i=std::upper_bound(first,last,z,[](int zz,const gfy_connectivity::Span&s){return zz<s.lo;});if(i==first)return 0;--i;return z<i->hi?i->component:0;}
 int at(const Vec&q)const{
  const double fx=(q.x-g.origin.x)/g.pitch,fy=(q.y-g.origin.y)/g.pitch,fz=(q.z-g.origin.z)/g.pitch;
  if(!(fx>=0&&fy>=0&&fz>=0&&fx<g.size.x&&fy<g.size.y&&fz<g.size.z))return -1;
  const int x=int(fx),y=int(fy),z=int(fz);
  if(tiles&&!(*tiles)[(std::size_t(x>>3)*std::size_t(TY)+std::size_t(y>>3))*std::size_t(TZ)+std::size_t(z>>3)])return -1; // no piece tissue in the tile
  const auto c=component(x,y,z);return c<piece.size()?piece[c]:-1;}
 // The substantial component of the first remaining tissue along dir from p (half-cell steps).
 int along(Vec p,Vec dir,double reach,double*found=nullptr)const{
  for(double s=0;s<=reach;s+=g.pitch*.5){const int k=at(p+dir*s);if(k>=0){if(found)*found=s;return k;}}return -1;}
 // The substantial component of the nearest remaining tissue within reach, searched along 26 directions (5 mm steps):
 // garments standing far off the body (a hood's shoulder skirt in the T pose) belong to the body part they cover.
 int nearest(Vec p,double reach,double*found=nullptr)const{
  static const double r2=0.7071067811865476,r3=0.5773502691896258;
  static const Vec dirs[26]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1},
   {r2,r2,0},{r2,-r2,0},{-r2,r2,0},{-r2,-r2,0},{r2,0,r2},{r2,0,-r2},{-r2,0,r2},{-r2,0,-r2},{0,r2,r2},{0,r2,-r2},{0,-r2,r2},{0,-r2,-r2},
   {r3,r3,r3},{r3,r3,-r3},{r3,-r3,r3},{r3,-r3,-r3},{-r3,r3,r3},{-r3,r3,-r3},{-r3,-r3,r3},{-r3,-r3,-r3}};
  for(double s=.005;s<=reach;s+=.005)for(const auto&d:dirs){const int k=at(p+d*s);if(k>=0){if(found)*found=s;return k;}}
  return -1;}
};
inline Vec position(const gfy_packed::Vertex&v){return {v.surface[0],v.surface[1],v.surface[2]};}
inline bool direction(const gfy_packed::Vertex&v,Vec&n){n={v.surface[3],v.surface[4],v.surface[5]};const double l=length(n);
 if(!(l>1e-6)||!std::isfinite(l))return false;n=n*(1/l);return true;}

// A face can have more than four CPU influences across its corners while all
// corners render with exactly the same four-link hardware map. In that case
// one affine skin transform acts on the entire rendered triangle. Subdivision
// may retain that existing map without approximating its visible posed shape.
// This does not claim equality with unquantized CPU picking/decal weights.
inline bool contourHardwareRow(const gfy_packed::Vertex&in,gfy_packed::Vertex&out,std::array<double,64>&map){
 const bool signed4=in.weight[3]<0.f&&gfy_signed_hw::valid4(in.weight,true);float sum=0;
 for(unsigned j=0;j<4;++j){if(in.bone[j]>=64||!std::isfinite(in.weight[j])||in.weight[j]>1.f||(in.weight[j]<0.f&&!signed4))return false;sum+=in.weight[j];}
 if(std::abs(sum-1.f)>1e-5f)return false;
 const float left=gfy_signed_hw::residual(in.weight),grouped=1.f-((in.weight[0]+in.weight[1])+in.weight[2]);
 if((!signed4&&(left<0.f||grouped<0.f))||std::abs(left-in.weight[3])>1e-5f)return false;
 out=in;unsigned bytes=0;
 for(unsigned j=0;j<3;++j){const auto q=gfy_signed_hw::quantize(in.weight[j]);bytes+=q;out.weight[j]=gfy_signed_hw::decoded(q);}
 if(!signed4&&bytes>255)return false;
 out.weight[3]=gfy_signed_hw::residual(out.weight);
 if(!gfy_signed_hw::valid4(out.weight,true))return false;
 map=gfy_signed_hw::full(out.bone,out.weight);return true;
}
inline bool commonContourHardware(const gfy_packed::Child&src,std::size_t t,gfy_packed::Vertex&common){
 std::array<double,64>want{};
 for(unsigned q=0;q<3;++q){gfy_packed::Vertex row;std::array<double,64>map{};
  if(!contourHardwareRow(src.vertices[src.indices[3*t+q]],row,map))return false;
  if(q==0){common=row;want=map;}else if(map!=want)return false;
 }
 // Original corners pass through the existing final publication prune, while
 // generated hardware rows are preserved. Refuse a contour if that publication
 // would move any original corner across an encoding boundary.
 gfy_packed::Mesh publication;publication.children.resize(1);
 for(unsigned q=0;q<3;++q)publication.children[0].vertices.push_back(src.vertices[src.indices[3*t+q]]);
 pruneNegligibleSkinWeights(publication);
 for(const auto&corner:publication.children[0].vertices){gfy_packed::Vertex encoded;std::array<double,64>map{};
  if(!contourHardwareRow(corner,encoded,map)||map!=want)return false;}
 gfy_packed::Vertex check;std::array<double,64>again{};
 return contourHardwareRow(common,check,again)&&again==want;
}

// Only the encoder's representable signed branch may round generated contour rows.
// Its approximate strongest-positive branch changes dependencies, not just bytes.
inline bool contourWeightsFit(const std::array<double,64>&want,const gfy_packed::Vertex&row,double discarded){
 bool signedSkin=false;
 for(double w:want){if(!std::isfinite(w))return false;signedSkin|=w<-1e-12;}
 for(unsigned k=0;k<4;++k)if(row.bone[k]>=64||!std::isfinite(row.weight[k]))return false;
 if(!std::isfinite(discarded))return false;
 if(!signedSkin)return discarded<1e-12; // Preserve the unsigned contour rule.
 std::array<std::uint16_t,4>bones{};std::array<float,4>weights{};
 const auto error=gfy_signed_hw::pack(want,bones,weights);
 if(error.approximated||row.bone!=bones||row.weight!=weights)return false;
 const auto have=gfy_signed_hw::full(row.bone,row.weight);
 for(unsigned b=0;b<64;++b){
  if(!std::isfinite(have[b]))return false;
  if(std::abs(want[b])<=1e-12){if(std::abs(have[b])>1e-12)return false;}
  else if(want[b]>0?!(have[b]>0):!(have[b]<0))return false;
 }
 return true;
}

// components: the substantial component IDs (one piece each, in this order). anchorable(k,bone): a weight on bone
// can be represented on piece k (the bone's element is on it or is its unique adjacent element).
// boneOwner(bone): the one piece holding that bone's whole physical element, or -1 when the element is split/absent.
// splitOwner(bone,p): for a bone whose element the break splits, the piece whose part of that element lies nearest p.
template<class Anchorable,class BoneOwner,class SplitOwner>
inline std::vector<bound_surface::Piece> build(const gfy_packed::Mesh&published,const Grid&grid,const gfy_connectivity::Topology&volume,
    const std::vector<std::uint32_t>&components,const std::set<std::uint16_t>&wholeChildren,std::uint16_t originalChildren,
    Anchorable&&anchorable,BoneOwner&&boneOwner,SplitOwner&&splitOwner,Stats&stats,
    const std::vector<std::pair<Cell,core::Tile>>*inherited=nullptr,std::vector<std::vector<std::pair<Cell,core::Tile>>>*marksOut=nullptr,
    const std::unordered_map<Cell,SeverState::TileMaterial,core::CellHash>*tissueTable=nullptr,
    const std::set<std::uint32_t>*deferredSourceComponents=nullptr,
    const std::set<std::pair<std::uint16_t,std::uint32_t>>*attachedFaces=nullptr,
    const std::function<std::uint16_t(std::size_t,std::uint16_t)>&regionFrontier={},
    const std::function<bool(const gfy_packed::Child&,std::size_t,std::vector<std::pair<std::size_t,core::Poly>>&)>&bindingRegions={}){
 const auto t0=core::Clock::now();
 need(available(&published),"lowpoly_piece_published_mesh");gfy_packed::validate_skin_ownership(published);
 // An owned grid is a piece session's (PieceSeed.h): other pieces' cells read as removed, and the marks it was born
 // with (`inherited`) stay in every new piece's field within its bounds.
 const std::size_t P=components.size();need(P>=1&&P<=12,"lowpoly_piece_count");
 std::vector<int> pieceOf(volume.components.size(),-1);
 for(std::size_t k=0;k<P;++k){need(components[k]>0&&components[k]<pieceOf.size(),"lowpoly_piece_component");pieceOf[components[k]]=int(k);}
 std::vector<std::int32_t> columnOf(std::size_t(grid.size.x)*std::size_t(grid.size.y),-1);
 for(std::size_t ci=0;ci<volume.columns.size();++ci){const auto&c=volume.columns[ci];
  if(c.x>=0&&c.y>=0&&c.x<grid.size.x&&c.y<grid.size.y)columnOf[std::size_t(c.y)*std::size_t(grid.size.x)+std::size_t(c.x)]=std::int32_t(ci);}
 // R70: built here (was: before the triangle owners) so every lookup can skip empty tiles.
 // Pieces per 8^3 tile of the remaining volume: 0 none, k+1 piece k only, 0xFFFF several.
 const int TX=(grid.size.x+7)/8,TY=(grid.size.y+7)/8,TZ=(grid.size.z+7)/8;
 std::vector<std::uint16_t> tilePiece(std::size_t(TX)*std::size_t(TY)*std::size_t(TZ),0);
 std::vector<std::size_t> band(std::size_t(TY)+1);
 for(int ty=0;ty<=TY;++ty)band[std::size_t(ty)]=std::size_t(std::lower_bound(volume.columns.begin(),volume.columns.end(),ty*8,
  [](const gfy_connectivity::Column&c,int y){return c.y<y;})-volume.columns.begin());
 const auto formerTileMap=[&](std::vector<std::uint16_t>&tilePiece){
 eachBlock(std::size_t(TY),1,[&](std::size_t first,std::size_t last,std::size_t){
  for(std::size_t ci=band[first];ci<band[last];++ci){const auto&col=volume.columns[ci];for(auto s=col.first;s<col.first+col.count;++s){const auto&sp=volume.spans[s];
   if(sp.component>=pieceOf.size()||pieceOf[sp.component]<0)continue;const std::uint16_t id=std::uint16_t(pieceOf[sp.component]+1);
   const int tx=col.x>>3,ty=col.y>>3;if(tx<0||ty<0||tx>=TX||ty>=TY)continue;
   for(int tz=std::max(0,sp.lo>>3);tz<=std::min(TZ-1,(sp.hi-1)>>3);++tz){auto&v=tilePiece[(std::size_t(tx)*TY+std::size_t(ty))*TZ+std::size_t(tz)];if(!v)v=id;else if(v!=id)v=0xFFFF;}}}});};
 // split2 part 8 (2026-11-12): the former loop below visits every tile of every span (one step per column and tile). A span
 // covers a range of tiles of its tile column: per band (one tile y) the ranges are counted per (piece, tile x) row with a
 // difference array (+1 at the first tile, -1 after the last) and each row is written once. A tile's value depends only on
 // the set of pieces that reach it (none 0, one piece k+1, several 0xFFFF), so the map is the same. Scratch per pool chunk.
 // GFY_LP_TILE_RANGES=0: the former loop alone. GFY_LP_TILE_RANGES_VERIFY=1: both, every tile compared (lowpoly_tile_map_mismatch).
 static const bool tileRanges=[]{const char*e=std::getenv("GFY_LP_TILE_RANGES");return !e||std::atoi(e)!=0;}();
 static const bool tileRangesVerify=std::getenv("GFY_LP_TILE_RANGES_VERIFY")!=nullptr;
 if(!tileRanges)formerTileMap(tilePiece);
 else{
  struct RangeScratch {std::vector<std::int32_t> diff;std::vector<char> flag;std::vector<std::uint32_t> used;};
  std::vector<RangeScratch> rangeScratch(core::parallelWidth());const std::size_t rangeRows=P*std::size_t(TX),rangeWidth=std::size_t(TZ)+1;
  eachBlock(std::size_t(TY),1,[&](std::size_t first,std::size_t last,std::size_t chunk){auto&scratch=rangeScratch[chunk<rangeScratch.size()?chunk:0];
   if(scratch.diff.empty()){scratch.diff.assign(rangeRows*rangeWidth,0);scratch.flag.assign(rangeRows,0);}
   for(std::size_t tileY=first;tileY<last;++tileY){
    for(std::size_t ci=band[tileY];ci<band[tileY+1];++ci){const auto&col=volume.columns[ci];
     const int tx=col.x>>3,ty=col.y>>3;if(tx<0||ty<0||tx>=TX||ty>=TY)continue;
     for(auto s=col.first;s<col.first+col.count;++s){const auto&sp=volume.spans[s];
      if(sp.component>=pieceOf.size()||pieceOf[sp.component]<0)continue;
      const int tzA=std::max(0,sp.lo>>3),tzB=std::min(TZ-1,(sp.hi-1)>>3);if(tzA>tzB)continue;
      const std::size_t row=std::size_t(pieceOf[sp.component])*std::size_t(TX)+std::size_t(tx);
      scratch.diff[row*rangeWidth+std::size_t(tzA)]+=1;scratch.diff[row*rangeWidth+std::size_t(tzB)+1]-=1;
      if(!scratch.flag[row]){scratch.flag[row]=1;scratch.used.push_back(std::uint32_t(row));}}}
    for(const auto row:scratch.used){const std::uint16_t id=std::uint16_t(row/std::size_t(TX)+1);const std::size_t tx=row%std::size_t(TX);std::int32_t run=0;
     auto*d=scratch.diff.data()+std::size_t(row)*rangeWidth;
     for(int tz=0;tz<TZ;++tz){run+=d[tz];d[tz]=0;if(run<=0)continue;auto&v=tilePiece[(tx*std::size_t(TY)+tileY)*std::size_t(TZ)+std::size_t(tz)];if(!v)v=id;else if(v!=id)v=0xFFFF;}
     d[TZ]=0;scratch.flag[row]=0;}
    scratch.used.clear();}});
  if(tileRangesVerify){std::vector<std::uint16_t> former(tilePiece.size(),0);formerTileMap(former);const bool same=former==tilePiece;
   std::size_t one=0,several=0;for(const auto v:tilePiece){one+=v&&v!=0xFFFF;several+=v==0xFFFF;}
   std::cerr<<(same?"tile_ranges_verify ok":"tile_ranges_verify MISMATCH")<<" tiles="<<tilePiece.size()<<" one_piece="<<one<<" several="<<several<<std::endl;
   need(same,"lowpoly_tile_map_mismatch");}}
 const Lookup L{grid,volume,pieceOf,&columnOf,&tilePiece,TX,TY,TZ};const double pitch=grid.pitch;
 const auto&field=*published.analytic_channels;
 // Exact standing component cells, shared by all siblings, no garment padding.
 const auto cellSupport=::gfy_live::piece_session::keptCellSupport(volume,grid,&pieceOf,P,&published);
 // Per piece and bone: may a weight on this bone stay on the piece?
 std::vector<std::vector<char>> ok(P);
 for(std::size_t k=0;k<P;++k){ok[k].resize(64);for(unsigned b=0;b<64;++b)ok[k][b]=anchorable(k,std::uint16_t(b))?1:0;}
 auto anchored=[&](std::size_t k,const gfy_packed::Vertex&v){
  for(unsigned w=0;w<4;++w)if(gfy_signed_hw::dependency(v.weight,w,negligible_skin_weight)&&(v.bone[w]>=64||!ok[k][v.bone[w]]))return false;return true;};
 // Generated wound walls are not authored skin: a corner may keep only the influences this piece can pose (wallRow
 // below drops the rest). A wall corner by the other thigh otherwise loses the whole cap triangle on the severed leg.
 auto wallAnchored=[&](std::size_t k,const gfy_packed::Vertex&v){
  if(gfy_signed_hw::negative(v.weight))return anchored(k,v);
  for(unsigned w=0;w<4;++w)if(v.weight[w]>0.f&&v.bone[w]<64&&ok[k][v.bone[w]])return true;return false;};
 auto wallRow=[&](std::size_t k,gfy_packed::Vertex row){
  if(gfy_signed_hw::negative(row.weight))return row;bool foreign=false;
  for(unsigned w=0;w<4;++w)if(gfy_signed_hw::dependency(row.weight,w,negligible_skin_weight)&&(row.bone[w]>=64||!ok[k][row.bone[w]]))foreign=true;
  if(!foreign)return row;std::array<double,64> keep{};double sum=0;
  for(unsigned w=0;w<4;++w){const auto b=row.bone[w];const double x=double(row.weight[w]);if(b<64&&ok[k][b]&&x>0){keep[b]+=x;sum+=x;}}
  need(sum>0,"lowpoly_wall_anchor");std::array<std::uint16_t,4> bones{};std::array<double,4> wts{};unsigned used=0;
  for(unsigned b=0;b<64&&used<4;++b)if(keep[b]>0){bones[used]=std::uint16_t(b);wts[used]=keep[b]/sum;++used;}
  for(unsigned q=0;q<4;++q){row.bone[q]=q<used?bones[q]:bones[0];row.weight[q]=q<used?float(wts[q]):0.f;}
  gfy_packed::canonical_weights(row);return row;};
 // R54: a detached thigh's trousers carry a few percent of the OTHER thigh near the crotch. cutSkinAnchor accepts only the
 // element next to the piece, so those faces were left off both pieces: holes just below the cap. The rule certified
 // clipped regions already use (regionFrontier: the single retained element bounding the bone's original subtree)
 // freezes such an influence with its captured relative pose, so the face stays exactly where it was. It applies only
 // when every foreign influence has that unique frontier and the corner keeps a positive influence the piece poses.
 std::vector<std::array<std::uint16_t,64>> front(P);
 for(std::size_t k=0;k<P;++k)for(unsigned b=0;b<64;++b)front[k][b]=(!ok[k][b]&&regionFrontier)?regionFrontier(k,std::uint16_t(b)):std::uint16_t(65535);
 auto frozenPosable=[&](std::size_t k,const gfy_packed::Vertex&v){
  // R94: a stock signed row (three UNORM8 links + a signed implicit fourth, RuntimeSignedSkin.h) whose three links quantize
  // to a sum of at most 256 carries only a one-unit quantization residue in its fourth slot (-1/255, or a float residue at
  // 255). That residue no longer refuses the freeze (KVMA 0055496d thigh cut: 98 trouser faces 5-21 cm below the cap were
  // on no piece, a see-through leg). Every negative slot still counts as a dependency below, so a residue on a bone the
  // piece cannot pose still needs that bone's unique retained frontier; weights and bones are unchanged. Any other
  // negative weight refuses the freeze as before.
  for(unsigned w=0;w<4;++w)if(v.weight[w]<0.f&&!(w==3&&gfy_signed_hw::valid4(v.weight,true)&&
     gfy_signed_hw::quantize(v.weight[0])+gfy_signed_hw::quantize(v.weight[1])+gfy_signed_hw::quantize(v.weight[2])<=256u))return false;
  bool own=false;
  for(unsigned w=0;w<4;++w){if(!gfy_signed_hw::dependency(v.weight,w,negligible_skin_weight))continue;const auto b=v.bone[w];
   if(b>=64)return false;if(ok[k][b]){own=own||v.weight[w]>0.f;continue;}if(front[k][b]==65535)return false;}
  return own;};
 auto freezeInto=[&](std::map<std::uint16_t,std::uint16_t>&anchors,std::size_t k,const gfy_packed::Vertex&v){
  for(unsigned w=0;w<4;++w){const auto b=v.bone[w];if(b<64&&!ok[k][b]&&gfy_signed_hw::dependency(v.weight,w,negligible_skin_weight))anchors.emplace(b,front[k][b]);}};
 std::vector<int> bonePiece(64,-1);for(unsigned b=0;b<64;++b)bonePiece[b]=boneOwner(std::uint16_t(b));
 // Far from the break the dominant skin bone decides (its element lies wholly on one piece); near it, the tissue under
 // the vertex decides (inward first, then outward for surfaces whose normal faces in).
 // Where the damage is: 32 mm blocks holding removed cells, grown by 12 cm (the reach of every tissue search below).
 // A vertex outside it cannot lie on a break: its whole-element skin bone decides without searching (in-game limb
 // splits spent ~55 ms marching all ~35k skin vertices, a third of them through the 12 cm nearest-tissue search).
 constexpr int blockCells=32;const int bx=(grid.size.x+blockCells-1)/blockCells,by=(grid.size.y+blockCells-1)/blockCells,bz=(grid.size.z+blockCells-1)/blockCells;
 std::vector<char> nearDamage(std::size_t(bx)*by*bz,0);
 {const int grow=int(std::ceil(.12/(blockCells*pitch)))+1;std::vector<char> seed(nearDamage.size(),0);
  for(const auto&it:grid.tiles){std::uint64_t any=0;for(auto w:it.second.removed)any|=w;if(!any)continue;
   const int x=it.first.x*8/blockCells,y=it.first.y*8/blockCells,z=it.first.z*8/blockCells;
   if(x>=0&&y>=0&&z>=0&&x<bx&&y<by&&z<bz)seed[(std::size_t(x)*by+y)*bz+z]=1;}
  for(int x=0;x<bx;++x)for(int y=0;y<by;++y)for(int z=0;z<bz;++z)if(seed[(std::size_t(x)*by+y)*bz+z])
   for(int a=std::max(0,x-grow);a<=std::min(bx-1,x+grow);++a)for(int b=std::max(0,y-grow);b<=std::min(by-1,y+grow);++b)for(int c=std::max(0,z-grow);c<=std::min(bz-1,z+grow);++c)
    nearDamage[(std::size_t(a)*by+b)*bz+c]=1;}
 auto nearWound=[&](const Vec&p){const int x=int(std::floor((p.x-grid.origin.x)/pitch))/blockCells,y=int(std::floor((p.y-grid.origin.y)/pitch))/blockCells,z=int(std::floor((p.z-grid.origin.z)/pitch))/blockCells;
  return x<0||y<0||z<0||x>=bx||y>=by||z>=bz||nearDamage[(std::size_t(x)*by+y)*bz+z]!=0;};
 // R62: torn tissue ends (channel field mode 4: the record's cylinder about the tear plane). Marching in from skin over a torn
 // end crosses the torn-away flesh and can reach the other side's tissue 4-6 cm down (trouser bands under the stump went to
 // the other piece: see-through slits under the torn rim). Each side's piece: the first remaining tissue along the axis past
 // the torn band.
 struct TornEnd {Vec c,n;double R,H;int side[2];};std::vector<TornEnd> tornEnds;
 for(unsigned i=0;i<field.cuts;++i){const auto&md=field.words[1+std::size_t(i)*9+6];if(md[3]!=4.f)continue;
  const auto&er=field.words[1+std::size_t(i)*9],&dp=field.words[1+std::size_t(i)*9+1];TornEnd e;e.H=.5*double(dp[3]);e.R=er[3];
  e.n=Vec{dp[0],dp[1],dp[2]};e.c=Vec{er[0],er[1],er[2]}+e.n*e.H;
  for(int sd=0;sd<2;++sd){const Vec dir=e.n*(sd?1.:-1.);e.side[sd]=L.along(e.c+dir*(.4*e.H),dir,.6*e.H+.03);}
  if(e.side[0]>=0&&e.side[1]>=0&&e.side[0]!=e.side[1])tornEnds.push_back(e);}
 // T2 (tear2): a point inside several torn ends whose sides disagree (the fractures of a burst head overlap the whole head) has
 // no side (-2): the first cylinder's side sent every scalp vertex over the blown vault to one fragment (black scalp sheets
 // floating off it) or to the neck (scalp crumbs above the stump)
 auto tornSide=[&](const Vec&p)->int{int found=-1;for(const auto&e:tornEnds){const Vec d=p-e.c;const double s=dot(d,e.n);if(std::abs(s)>e.H||dot(d,d)-s*s>e.R*e.R)continue;
   const int k=e.side[s>=0?1:0];if(found<0)found=k;else if(found!=k)return -2;}return found;};
 // E5: backed (optional) = remaining tissue lies under the point (met along its normal), or its whole-element bone decides far from damage
 auto skinPointOwner=[&](const Vec&p,const Vec&n,bool hasNormal,unsigned dominant,int&how,double&depth,bool*backed=nullptr){
  const int byBone=dominant<64?bonePiece[dominant]:-1;int k=-1;depth=1;if(backed)*backed=false;
  if(byBone>=0&&!nearWound(p)){how=2;if(backed)*backed=true;return byBone;}
  if(hasNormal){k=L.along(p+n*pitch,n*-1.,.06,&depth);if(k<0)k=L.along(p-n*pitch,n,.02,&depth);}else{k=L.at(p);depth=0;}
  if(backed)*backed=k>=0;
  if(!tornEnds.empty()){const int side=tornSide(p);if(side>=0&&(k<0||(k!=side&&depth>.012))){how=1;depth=0;return side;}
   if(side==-2&&(k<0||depth>.012)){double dn=1;const int q=L.nearest(p,.12,&dn);if(q>=0){how=3;depth=dn;return q;}}} // T2: no agreed side: the nearest remaining tissue (not one found deep across the blown part)
  if(k>=0){how=1;return k;}
  // Nothing under it within 6 cm: the nearest tissue within 12 cm, before the skin weights (a hood weighted to the
  // head whose skirt drapes over the shoulders stays on the shoulders when the head comes off).
  k=L.nearest(p,.12,&depth);if(k>=0){how=3;return k;}
  depth=1;if(byBone>=0){how=2;return byBone;}
  // Skin with no tissue under it on a split element (collars, hoods, masks at a severed neck): the side of the split
  // its position belongs to, never the owner of whatever mesh it happens to connect to (floating head-piece triangles).
  if(dominant<64){k=splitOwner(dominant,p);if(k>=0){how=2;return k;}}
  how=0;return -1;};
 auto skinOwner=[&](const gfy_packed::Vertex&v,int&how,double&depth,bool*backed=nullptr){
  unsigned best=0;for(unsigned w=1;w<4;++w)if(v.weight[w]>v.weight[best])best=w;
  Vec n;const bool hasNormal=direction(v,n);return skinPointOwner(position(v),n,hasNormal,v.bone[best],how,depth,backed);};
 struct ClippedTriangle {std::array<gfy_packed::Vertex,3> v;std::uint32_t face,source;};
 struct OwnedTriangle {std::uint32_t source;std::vector<std::array<float,4>> planes;std::map<std::uint16_t,std::uint16_t> anchors;};
 struct Work {std::vector<std::vector<OwnedTriangle>> owned;std::vector<std::vector<ClippedTriangle>> clipped;std::vector<std::vector<std::uint32_t>> tris;std::vector<std::vector<Cell>> boxes;/* per piece: tiles around its boundary triangles surfaces */std::vector<std::map<std::uint16_t,std::uint16_t>> anchors;/* per piece: frozen bone -> frontier */std::size_t dropped{},unanchored{},boundary{},marched{},byBone{},unowned{},nearest{},channelDropped{},frozen{};};
 const auto&children=published.children;std::vector<Work> work(children.size());
 auto cellOf=[&](const Vec&p){return Cell{int(std::floor((p.x-grid.origin.x)/pitch)),int(std::floor((p.y-grid.origin.y)/pitch)),int(std::floor((p.z-grid.origin.z)/pitch))};};
 // Nearest remaining tissue of a substantial component, shell by shell from `from` to `reach` cells.
 auto nearestShell=[&](const Vec&p,int from,int reach){const Cell c0=cellOf(p);int best=1<<30,found=-1;
  for(int r=from;r<=reach&&found<0;++r)for(int dx=-r;dx<=r;++dx)for(int dy=-r;dy<=r;++dy)for(int dz=-r;dz<=r;++dz){
   if(painter_short_minmax::maxOf(std::abs(dx),std::abs(dy),std::abs(dz))!=r)continue;const int d2=dx*dx+dy*dy+dz*dz;if(d2>=best)continue;
   const Cell c{c0.x+dx,c0.y+dy,c0.z+dz};if(!grid.valid(c))continue;
   if(!tilePiece[(std::size_t(c.x>>3)*std::size_t(TY)+std::size_t(c.y>>3))*std::size_t(TZ)+std::size_t(c.z>>3)])continue; // R70: no piece tissue in the tile
   const auto comp=L.component(c.x,c.y,c.z);
   if(comp<pieceOf.size()&&pieceOf[comp]>=0){best=d2;found=pieceOf[comp];}}
  return found;};
 // The work is spread over vertex and triangle ranges of all children (a few large wound-wall children used to leave
 // most threads idle: 11-piece gib 493 ms); every range writes its own slots and the results merge in child/range
 // order, so the pieces are identical to the per-child loop.
 enum:char{kind_skip,kind_skin,kind_channel};std::vector<char> kind(children.size(),kind_skip);
 for(std::size_t c=0;c<children.size();++c){const auto&src=children[c];if(src.indices.empty()||src.vertices.empty())continue;
  if(!src.material){if(!wholeChildren.count(src.material_source_child))kind[c]=kind_skin;}
  else{const auto*prim=field.child(src.child_index);need(prim&&prim->count==src.indices.size()/3,"lowpoly_piece_channel_manifest");kind[c]=kind_channel;}}
 std::vector<std::vector<char>> bare(children.size()),howOf(children.size()); // E5: skin vertex with no tissue under it; placement code
 std::vector<std::vector<int>> own(children.size());std::vector<std::vector<float>> depth(children.size());std::vector<std::vector<char>> spread(children.size());
 // Walls in the anatomy-to-clothing gap (R53). The renderer draws walls over the closure field: anatomy plus the air
 // inside the closed body envelope, where the true-anatomy reach of 8 mm finds nothing (median gap 2.5-4 cm in loose
 // trousers). Such a vertex takes the piece whose remaining tissue is nearest along a straight path that stays inside
 // the envelope and never enters removed volume, so it never crosses the cut to the other piece.
 std::vector<std::vector<char>> envOk(children.size());
 const auto*closureField=published.tissue_field.get();const auto*removalField=published.removal_field.get();
 auto inEnvelope=[&](const Vec&q){return closureField->sample(q.x,q.y,q.z)!=0u;};
 auto cutAt=[&](const Vec&q){return removalField->sample(q.x,q.y,q.z);};
 auto gapOwner=[&](const Vec&p,const Vec&n,bool hasNormal)->int{
  static const double r2=0.7071067811865476,r3=0.5773502691896258;
  static const Vec dirs[26]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1},
   {r2,r2,0},{r2,-r2,0},{-r2,r2,0},{-r2,-r2,0},{r2,0,r2},{r2,0,-r2},{-r2,0,r2},{-r2,0,-r2},{0,r2,r2},{0,r2,-r2},{0,-r2,r2},{0,-r2,-r2},
   {r3,r3,r3},{r3,r3,-r3},{r3,-r3,r3},{r3,-r3,-r3},{-r3,r3,r3},{-r3,r3,-r3},{-r3,-r3,r3},{-r3,-r3,-r3}};
  const Vec s0=hasNormal?p-n*.003:p;if(!inEnvelope(s0)||cutAt(s0))return -1;int best=-1;double reach=.12;
  for(const auto&d:dirs){if(hasNormal&&dot(d,n)>.3)continue;
   for(double s=.0025;s<=reach;s+=.0025){const Vec q=s0+d*s;const int k=L.at(q);
    if(k>=0){best=k;reach=s-.0025;break;}if(cutAt(q)||!inEnvelope(q))break;}}
  return best;};
 // R67: small jobs (vertices 256, skin triangles 32, walls 32): boundary faces cost up to ~0.4 ms each and cluster, and one large
 // job of them left a single thread working long after the others ran out of jobs (results do not depend on the job size)
 struct Range {std::uint32_t c,lo,hi;};constexpr std::uint32_t chunk=256,triangleChunk=32;std::vector<Range> vjobs,tjobs;
 for(std::size_t c=0;c<children.size();++c){if(kind[c]==kind_skip)continue;const auto&src=children[c];
  const std::uint32_t V=std::uint32_t(src.vertices.size()),T=std::uint32_t(src.indices.size()/3);
  own[c].assign(V,-1);if(kind[c]==kind_skin){depth[c].assign(V,1.f);spread[c].assign(V,0);bare[c].assign(V,1);howOf[c].assign(V,0);}else envOk[c].assign(V,0);
  for(std::uint32_t lo=0;lo<V;lo+=chunk)vjobs.push_back({std::uint32_t(c),lo,std::min(V,lo+chunk)});
  const std::uint32_t step=kind[c]==kind_channel?32u:triangleChunk;for(std::uint32_t lo=0;lo<T;lo+=step)tjobs.push_back({std::uint32_t(c),lo,std::min(T,lo+step)});}
 // ---- vertex owners
 const auto vt0=core::Clock::now();
 std::vector<std::array<std::size_t,4>> howCount(vjobs.size(),std::array<std::size_t,4>{}); // marched, by bone, nearest, unowned
 eachBlock(vjobs.size(),1,[&](std::size_t first,std::size_t last,std::size_t){
  for(std::size_t j=first;j<last;++j){const Range r=vjobs[j];const auto&src=children[r.c];auto&o=own[r.c];
   if(kind[r.c]==kind_skin)for(std::uint32_t v=r.lo;v<r.hi;++v){int how=0;double dv=1;bool bk=false;o[v]=skinOwner(src.vertices[v],how,dv,&bk);depth[r.c][v]=float(dv);
     if(o[v]<0){o[v]=nearestShell(position(src.vertices[v]),1,6);if(o[v]>=0)how=3;} // nearest remaining tissue (6 mm)
     bare[r.c][v]=bk?0:1;howOf[r.c][v]=char(how);
     ++howCount[j][how==1?0:how==2?1:how==3?2:3];}
   else for(std::uint32_t v=r.lo;v<r.hi;++v){const Vec p=position(src.vertices[v]);Vec n;
     // Wound walls are drawn wherever the closure field (anatomy + closed body envelope) holds tissue, which includes
     // anatomy gaps with no solid voxel behind the wall. Owner per vertex: the tissue behind it (either normal side,
     // 8 mm); else the connected wall around it; else the nearest remaining tissue (6 mm). A wall with no tissue
     // anywhere near cannot belong to either side and is left off (in-game sever18: 90 % of the torn-end walls were
     // dropped by a backing-only rule, leaving see-through ends).
     const bool hasNormal=direction(src.vertices[v],n);
     if(hasNormal){o[v]=L.along(p+n*(.5*pitch),n*-1.,.008);if(o[v]<0)o[v]=L.along(p-n*(.5*pitch),n,.008);}
     else o[v]=L.at(p);
     envOk[r.c][v]=char(inEnvelope(hasNormal?p-n*.003:p)?1:0);
     if(o[v]<0&&envOk[r.c][v])o[v]=gapOwner(p,n,hasNormal);}}});
 stats.vertexMs=core::ms(vt0);const auto rt0=core::Clock::now();
 for(std::size_t j=0;j<vjobs.size();++j){auto&w=work[vjobs[j].c];w.marched+=howCount[j][0];w.byBone+=howCount[j][1];w.nearest+=howCount[j][2];w.unowned+=howCount[j][3];}
 // ---- owners spread over the connected surface, ring by ring (per child)
 core::parallelChunks(children.size(),1,[&](std::size_t first,std::size_t last,std::size_t){
  for(std::size_t c=first;c<last;++c){if(kind[c]==kind_skip)continue;const auto&src=children[c];const std::size_t T=src.indices.size()/3;auto&o=own[c];
   if(kind[c]==kind_skin){
    // Skin with no tissue under it and a split element (inside or beside a wound): take the owner of the connected
    // surface around it, ring by ring. Such triangles count as boundary so the removal field decides per pixel.
    auto&sp=spread[c];
    for(int ring=0;ring<3;++ring){bool changed=false;
     for(std::size_t t=0;t<T;++t){int known=-1;for(int j=0;j<3;++j)if(o[src.indices[3*t+j]]>=0&&!sp[src.indices[3*t+j]]){known=o[src.indices[3*t+j]];break;}
      if(known<0)for(int j=0;j<3;++j)if(o[src.indices[3*t+j]]>=0){known=o[src.indices[3*t+j]];break;}
      if(known<0)continue;
      for(int j=0;j<3;++j){const auto v=src.indices[3*t+j];if(o[v]<0){o[v]=known;sp[v]=1;changed=true;}}}
     if(!changed)break;}}
   else for(int ring=0;ring<64;++ring){bool changed=false;
     for(std::size_t t=0;t<T;++t){int known=-1;for(int j=0;j<3&&known<0;++j)known=o[src.indices[3*t+j]];if(known<0)continue;
      for(int j=0;j<3;++j){const auto v=src.indices[3*t+j];if(o[v]<0&&envOk[c][v]){o[v]=known;changed=true;}}}
     if(!changed)break;}}});
 stats.ringMs=core::ms(rt0);const auto tt0=core::Clock::now();
 auto tileAt=[&](int x,int y,int z)->std::uint16_t{if(x<0||y<0||z<0||x>=TX||y>=TY||z>=TZ)return 0;return tilePiece[(std::size_t(x)*TY+std::size_t(y))*TZ+std::size_t(z)];};
 // Only piece k's tissue (or none) within a tile of the triangle's box.
 auto singleAround=[&](const Vec&a,const Vec&b,const Vec&d,int k){const std::uint16_t id=std::uint16_t(k+1);
  Cell lo=cellOf(a),hi=lo;for(const Vec*p:{&b,&d}){const Cell c=cellOf(*p);lo={std::min(lo.x,c.x),std::min(lo.y,c.y),std::min(lo.z,c.z)};hi={std::max(hi.x,c.x),std::max(hi.y,c.y),std::max(hi.z,c.z)};}
  for(int x=(lo.x>>3)-1;x<=(hi.x>>3)+1;++x)for(int y=(lo.y>>3)-1;y<=(hi.y>>3)+1;++y)for(int z=(lo.z>>3)-1;z<=(hi.z>>3)+1;++z){const auto v=tileAt(x,y,z);if(v&&v!=id)return false;}
  return true;};
 // One interpolation/classification path for contour tracing and region certificates.
 auto surfaceOwner=[&](const gfy_packed::Child&src,std::size_t t,const std::array<double,3>&b){
  Vec p{},n{};std::array<double,64>weights{};unsigned bone=64;
  for(unsigned q=0;q<3;++q){const auto&row=src.vertices[src.indices[3*t+q]];Vec normal;if(!direction(row,normal))return -1;
   p=p+position(row)*b[q];n=n+normal*b[q];if(b[q]==1.){unsigned best=0;for(unsigned w=1;w<4;++w)if(row.weight[w]>row.weight[best])best=w;bone=row.bone[best];}
   for(unsigned w=0;w<4;++w)if(row.bone[w]<64)weights[row.bone[w]]+=b[q]*row.weight[w];}
  if(bone==64){bone=0;for(unsigned q=1;q<64;++q)if(weights[q]>weights[bone])bone=q;}
  const double len=length(n);if(!(len>1e-9))return -1;n=n*(1./len);int how=0;double depth=1;
  int k=skinPointOwner(p,n,true,bone,how,depth);if(k<0)k=nearestShell(p,1,6);return k;};
 auto effectiveRegion=[&](const gfy_packed::Child&src,std::size_t t,core::Poly poly){
  const auto*table=published.skin_ownership?published.skin_ownership->child(src.child_index):nullptr;
  if(!table)return poly;const auto&r=table->data[t];
  for(unsigned plane=0;plane<r[1]&&!poly.empty();++plane){const auto&p=published.skin_ownership->planes[r[0]+plane];core::Poly next;
   auto distance=[&](const core::V&v){return p[0]*v.p.x+p[1]*v.p.y+p[2]*v.p.z+p[3];};
   for(std::size_t q=0;q<poly.size();++q){const auto&a=poly[q];const auto&b=poly[(q+1)%poly.size()];const double da=distance(a),db=distance(b);
    if(da>=0)next.push_back(a);if((da>=0)!=(db>=0)){const double f=da/(da-db);core::V v;v.p=a.p*(1-f)+b.p*f;for(unsigned j=0;j<3;++j)v.b[j]=a.b[j]*(1-f)+b.b[j]*f;next.push_back(v);}}
   poly=std::move(next);}
  return poly;};
 // Convert an immutable binding-space partition to a convex predicate on
 // the untouched current authored triangle. No generated skin weights.
 auto polygonPlanes=[&](const gfy_packed::Child&src,std::size_t t,const core::Poly&poly){
  need(poly.size()>=3,"binding_region_empty");std::array<Vec,3> original;
  for(unsigned q=0;q<3;++q)original[q]=position(src.vertices[src.indices[3*t+q]]);
  auto cross=[](const Vec&a,const Vec&b){return Vec{a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};};
  const Vec normal=cross(original[1]-original[0],original[2]-original[0]);need(length(normal)>1e-20,"binding_region_degenerate_source");
  Vec centre{};for(const auto&v:poly){double sum=0;Vec reconstructed{};for(unsigned q=0;q<3;++q){need(std::isfinite(v.b[q])&&v.b[q]>=-1e-6&&v.b[q]<=1.000001,"binding_region_barycentric");sum+=v.b[q];reconstructed=reconstructed+original[q]*v.b[q];}
   need(std::abs(sum-1.)<1e-6&&length(reconstructed-v.p)<2e-6,"binding_region_source_plane");centre=centre+v.p*(1./poly.size());}
  std::vector<std::array<float,4>>planes;
  for(std::size_t q=0;q<poly.size();++q){const Vec start=poly[q].p,edge=poly[(q+1)%poly.size()].p-start;Vec n=cross(normal,edge);const double len=length(n);if(!(len>1e-24))continue;n=n*(1./len);double w=-dot(n,start);
   if(dot(n,centre)+w<0){n=n*-1.;w=-w;}for(const auto&v:poly)need(dot(n,v.p)+w>=-2e-9,"binding_region_nonconvex");
   std::array<float,4>plane{{float(n.x),float(n.y),float(n.z),float(w)}};for(float v:plane)need(std::isfinite(v),"binding_region_nonfinite");planes.push_back(plane);}
  need(planes.size()>=3,"binding_region_no_predicate");return planes;};
 auto constrainedAnchors=[&](const gfy_packed::Child&src,std::size_t t,std::size_t k,const int owners[3],const core::Poly&region,const std::vector<std::array<float,4>>&planes,std::map<std::uint16_t,std::uint16_t>&anchors,bool bindingCertified=false){
  for(unsigned q=0;q<3;++q){const auto&v=src.vertices[src.indices[3*t+q]];for(unsigned w=0;w<4;++w)if(gfy_signed_hw::dependency(v.weight,w,0.)){
   if(v.bone[w]>=64)return false;if(ok[k][v.bone[w]])continue;
   if(!bindingCertified||!regionFrontier)return false;const auto anchor=regionFrontier(k,v.bone[w]);if(anchor==65535)return false;anchors[v.bone[w]]=anchor;}}
  if(anchors.empty())return true;const auto effective=effectiveRegion(src,t,region);return effective.size()>=3&&bound_surface::fraction(effective)>1e-14;};
 // Trace the owner transition along the authored garment edge. A whole
 // triangle vote leaves its third corner sticking out of the detached piece.
 // Keep both original-surface regions, separated by that measured transition;
 // do not infer a world-axis plane or repartition unrelated collars/gear.
 auto contourRegions=[&](const gfy_packed::Child&src,std::size_t t,const int owners[3],std::vector<core::Poly>&regions,bool&commonHardware,gfy_packed::Vertex&commonRow,bool&predicate,std::vector<std::vector<std::array<float,4>>>&planes){
  std::array<Vec,3> pos,norm;std::array<unsigned,3> dominant{};
  for(unsigned q=0;q<3;++q){const auto&row=src.vertices[src.indices[3*t+q]];pos[q]=position(row);
   if(!direction(row,norm[q]))return false;unsigned best=0;for(unsigned w=1;w<4;++w)if(row.weight[w]>row.weight[best])best=w;dominant[q]=row.bone[best];}
  auto locate=[&](const std::array<double,3>&b){return surfaceOwner(src,t,b);};
  for(unsigned q=0;q<3;++q){std::array<double,3>b{};b[q]=1.;if(owners[q]<0||locate(b)!=owners[q])return false;}
  // Unknown/third-owner/non-monotonic edges are explicit refusals, never a
  // guessed contour. First bracket the only transition, then refine it.
  auto transition=[&](unsigned a,unsigned b,core::V&point){
   constexpr unsigned steps=16;unsigned switchAt=0;bool switched=false;
   for(unsigned q=1;q<=steps;++q){std::array<double,3>bar{};bar[a]=1.-double(q)/steps;bar[b]=double(q)/steps;const int k=locate(bar);
    if(k==owners[b]){if(!switched){switched=true;switchAt=q;}}
    else if(k!=owners[a]||switched)return false;}
   if(!switched)return false;double lo=double(switchAt-1)/steps,hi=double(switchAt)/steps;
   for(unsigned step=0;step<12;++step){const double u=(lo+hi)*.5;std::array<double,3>bar{};bar[a]=1-u;bar[b]=u;const int k=locate(bar);
    if(k==owners[a])lo=u;else if(k==owners[b])hi=u;else return false;}
   const double u=(lo+hi)*.5;point.p=pos[a]*(1-u)+pos[b]*u;point.b[a]=1-u;point.b[b]=u;return true;};
  core::Poly original;for(unsigned q=0;q<3;++q){core::V v;v.p=pos[q];v.b[q]=1.;original.push_back(v);}
  auto cross=[](const Vec&a,const Vec&b){return Vec{a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};};
  const Vec triNormal=cross(pos[1]-pos[0],pos[2]-pos[0]);if(!(length(triNormal)>1e-12))return false;
  auto planeThrough=[&](const Vec&a,const Vec&b,const Vec&keep,std::array<float,4>&out){
   Vec n=cross(triNormal,b-a);const double len=length(n);if(!(len>1e-12)||!std::isfinite(len))return false;n=n*(1./len);
   double w=-dot(n,a);if(dot(n,keep)+w<0){n=n*-1.;w=-w;}out={float(n.x),float(n.y),float(n.z),float(w)};
   for(float q:out)if(!std::isfinite(q))return false;return true;};
  regions.assign(P,{});planes.assign(P,{});
  int lonely=-1;for(int q=0;q<3;++q)if(owners[q]!=owners[(q+1)%3]&&owners[q]!=owners[(q+2)%3]&&owners[(q+1)%3]==owners[(q+2)%3])lonely=q;
  if(lonely>=0){
   const unsigned a=unsigned(lonely),b=(a+1)%3,c=(a+2)%3;core::V ab,ac;
   if(!transition(a,b,ab)||!transition(a,c,ac))return false;
   regions[owners[a]]={original[a],ab,ac};regions[owners[b]]={ab,original[b],original[c],ac};
   std::array<float,4>plane;if(!planeThrough(ab.p,ac.p,pos[a],plane))return false;
   planes[owners[a]].push_back(plane);for(auto&q:plane)q=-q;planes[owners[b]].push_back(plane);
  }else{
   if(owners[0]==owners[1]||owners[0]==owners[2]||owners[1]==owners[2])return false;
   std::array<core::V,3>edges;for(unsigned q=0;q<3;++q)if(!transition(q,(q+1)%3,edges[q]))return false;
   // Three measured edge crossings bound a central triangle. Its barycentre
   // joins the three region borders; this planar partition exactly covers the
   // original face without overlaps or gaps, including grenade three-way cuts.
   core::V centre;for(const auto&e:edges){centre.p=centre.p+e.p*(1./3);for(unsigned q=0;q<3;++q)centre.b[q]+=e.b[q]/3;}
   for(unsigned q=0;q<3;++q){const unsigned prev=(q+2)%3;regions[owners[q]]={original[q],edges[q],centre,edges[prev]};
    std::array<float,4>plane;if(!planeThrough(edges[q].p,centre.p,pos[q],plane))return false;
    planes[owners[q]].push_back(plane);for(auto&v:plane)v=-v;planes[owners[(q+1)%3]].push_back(plane);}
  }
  double area=0;for(const auto&poly:regions)area+=bound_surface::fraction(poly);
  need(std::abs(area-1.)<1e-10,"skin_contour_area");
  // Do not add another skin-weight reduction at a cut. The render ABI has
  // four influences; unsupported edges keep the existing masked path.
  for(const auto&poly:regions)for(const auto&v:poly){std::array<double,64> weights{};
   for(unsigned q=0;q<3;++q){const auto&row=src.vertices[src.indices[3*t+q]];
    for(unsigned j=0;j<4;++j)weights[row.bone[j]]+=row.weight[j]*v.b[q];}
   unsigned used=0;for(auto w:weights)used+=std::abs(w)>1e-12;if(used>4&&!commonHardware){if(commonContourHardware(src,t,commonRow))commonHardware=true;else predicate=true;}
  }
  return true;
 };
 const double tileBuildMs=core::ms(tt0);std::atomic<long long> skinNs{0},wallNs{0};
 // ---- triangles to pieces
 std::vector<Work> tw(tjobs.size());
 // A wall triangle is long (a tunnel's wall runs its whole length): one crossing the break goes on every piece
 // whose tissue backs part of it, and the channel shader drops the pixels whose backing tissue is foreign on that
 // piece (painter_channel_tissue + the piece's removal field) - no red slab of tunnel hanging off a severed head.
 auto backing=[&](const Vec&p,const Vec&n){for(double s=.001;s<=.008;s+=.002){int k=L.at(p-n*s);if(k>=0)return k;k=L.at(p+n*s);if(k>=0)return k;}return -1;};
 eachBlock(tjobs.size(),1,[&](std::size_t first,std::size_t last,std::size_t){
  for(std::size_t j=first;j<last;++j){const auto jobStart=core::Clock::now();struct JobTime{std::atomic<long long>&n;core::Clock::time_point s;~JobTime(){n+=std::chrono::duration_cast<std::chrono::nanoseconds>(core::Clock::now()-s).count();}};
   const Range r=tjobs[j];JobTime jobTime{kind[r.c]==kind_skin?skinNs:wallNs,jobStart};const auto&src=children[r.c];auto&w=tw[j];w.tris.resize(P);w.boxes.resize(P);w.clipped.resize(P);w.owned.resize(P);w.anchors.resize(P);const auto&o=own[r.c];
   if(kind[r.c]==kind_skin){const auto&dp=depth[r.c];const auto&sp=spread[r.c];
    for(std::size_t t=r.lo;t<r.hi;++t){
     if(attachedFaces&&!attachedFaces->empty()){
      need(src.original_faces.size()*3==src.indices.size(),"mixed_accessory_source_provenance");
      if(attachedFaces->count({src.material_source_child,src.original_faces[t]}))continue;
     }
     int ow[3];int K[3];int nk=0;bool unknown=false;
     for(int q=0;q<3;++q){ow[q]=o[src.indices[3*t+q]];if(ow[q]<0){unknown=true;continue;}bool seen=false;for(int s=0;s<nk;++s)seen|=K[s]==ow[q];if(!seen)K[nk++]=ow[q];}
     for(int q=0;q<3;++q)unknown|=sp[src.indices[3*t+q]]!=0;
     // Immutable original attachment takes precedence on ambiguous boundary
     // faces and on inherited clipped faces. A certified region never falls
     // back to a whole-face vote or alternate host.
     const auto*inheritedTable=published.skin_ownership?published.skin_ownership->child(src.child_index):nullptr;
     if(bindingRegions&&(nk>1||unknown||(inheritedTable&&inheritedTable->data[t][1]))){
      std::vector<std::pair<std::size_t,core::Poly>>bound;
      if(bindingRegions(src,t,bound)){
       std::vector<std::pair<std::size_t,OwnedTriangle>>pending;double total=0;
       for(const auto&region:bound){const auto k=region.first;need(k<P,"binding_region_owner");auto poly=effectiveRegion(src,t,region.second);
        const double area=bound_surface::fraction(poly);if(poly.size()<3||area<1e-14)continue;total+=area;
        auto planes=polygonPlanes(src,t,poly);std::map<std::uint16_t,std::uint16_t>anchors;
        need(constrainedAnchors(src,t,k,ow,poly,planes,anchors,true),"binding_region_without_unique_frontier");
        pending.push_back({k,{std::uint32_t(t),std::move(planes),std::move(anchors)}});}
       need(total<=1.00001,"binding_region_overlap");
       for(auto&region:pending)w.owned[region.first].push_back(std::move(region.second));
       if(pending.empty())++w.dropped;else ++w.boundary;continue;
      }
     }
     if(!nk){++w.dropped;continue;}
     std::vector<core::Poly> regions;std::vector<std::vector<std::array<float,4>>> regionPlanes;bool contourFallback=false,commonHardware=false,predicate=false;gfy_packed::Vertex commonRow;
     if(nk>=2&&!unknown&&contourRegions(src,t,ow,regions,commonHardware,commonRow,predicate,regionPlanes)){
      need(src.original_faces.size()*3==src.indices.size(),"skin_contour_source_faces");
      core::JoinedTriangle original;
      for(unsigned q=0;q<3;++q){std::array<double,3>b{};b[q]=1;original.v[q]=original_accessory::unpack(src.vertices[src.indices[3*t+q]],src.original_faces[t],b);}
      // Buffer the entire contour: all rows must be faithful AND poseable before any polygon is appended.
      // Otherwise keep the original authored triangle on an existing valid whole-face host below.
      std::vector<std::pair<std::size_t,ClippedTriangle>> rows;bool exact=!predicate;
      for(std::size_t k=0;k<P&&exact;++k){const auto&poly=regions[k];if(poly.empty())continue;
       for(std::size_t q=1;q+1<poly.size()&&exact;++q){ClippedTriangle row;row.face=src.original_faces[t];row.source=std::uint32_t(t);
        const core::V*corners[3]={&poly[0],&poly[q],&poly[q+1]};
        for(unsigned v=0;v<3&&exact;++v){bool corner=false;
         for(unsigned q=0;q<3;++q)if(corners[v]->b[q]==1.){row.v[v]=src.vertices[src.indices[3*t+q]];corner=true;break;}
         if(!corner){auto generated=bound_surface::interpolate(original,corners[v]->b);
          if(commonHardware){generated.hardware_corner=true;generated.hardware_ids=commonRow.bone;generated.hardware_w=commonRow.weight;generated.w=gfy_signed_hw::full(commonRow.bone,commonRow.weight);}
          double lost=0;row.v[v]=core::packing::vertex(generated,&lost);
          exact=commonHardware?(row.v[v].bone==commonRow.bone&&row.v[v].weight==commonRow.weight):contourWeightsFit(generated.w,row.v[v],lost);}
         if(exact)exact=anchored(k,row.v[v]);}
        if(exact)rows.emplace_back(k,std::move(row));}}
      if(exact){for(auto&r:rows)w.clipped[r.first].push_back(std::move(r.second));++w.boundary;continue;}
      // All original corners must remain poseable on every receiving piece.
      // Buffer this decision before publishing either side, just like the
      // retessellated path above. The material clips only this source triangle.
      bool poseable=true;std::vector<std::map<std::uint16_t,std::uint16_t>> certified(P);
      for(std::size_t k=0;k<P&&poseable;++k)if(!regions[k].empty())poseable=constrainedAnchors(src,t,k,ow,regions[k],regionPlanes[k],certified[k]);
      if(poseable){for(std::size_t k=0;k<P;++k)if(!regions[k].empty())w.owned[k].push_back({std::uint32_t(t),regionPlanes[k],std::move(certified[k])});++w.boundary;continue;}
      contourFallback=true;
     }
     if(nk==1&&!unknown&&published.skin_ownership){const auto*tab=published.skin_ownership->child(src.child_index);
      if(tab&&tab->data[t][1]){core::Poly original;for(unsigned q=0;q<3;++q){core::V v;v.p=position(src.vertices[src.indices[3*t+q]]);v.b[q]=1.;original.push_back(v);}
       std::map<std::uint16_t,std::uint16_t> certified;const auto k=std::size_t(K[0]);
       if(constrainedAnchors(src,t,k,ow,original,{},certified)){w.owned[k].push_back({std::uint32_t(t),{},std::move(certified)});continue;}}}
     // The per-pixel split (foreign marks) reaches 3 cells off the tissue. A straddling triangle standing farther off
     // (garment over the break) would be drawn whole on BOTH pieces - the ghost shell that flew off with severed
     // heads - so it goes whole to the piece holding most of it (ties: the vertex closest to its tissue).
     int alt[3];int na=0;
     if(nk>1&&!unknown){float close=1e9f;int pick=-1,votes[3]={0,0,0};
      for(int q=0;q<3;++q){const auto v=src.indices[3*t+q];for(int s=0;s<nk;++s)if(K[s]==ow[q])++votes[s];if(dp[v]<close){close=dp[v];pick=ow[q];}}
      if(close>float(3*pitch)){int top=0;for(int s=1;s<nk;++s)if(votes[s]>votes[top])top=s;
       if(votes[top]>1)pick=K[top];for(int s=0;s<nk;++s)if(K[s]!=pick)alt[na++]=K[s];K[0]=pick;nk=1;}}
     bool boundary=nk>1||unknown;std::vector<Cell> surfaceTiles;
     auto surfaceTilesOf=[&](){if(!surfaceTiles.empty())return;
      // Tiles within 3 cells of the triangle's SURFACE (4 mm samples), not of its bounding box: a large straddling
      // garment triangle touches few tiles but boxes many (in-game head split: 80k tiles, 1.2 s).
      const Vec a=position(src.vertices[src.indices[3*t]]),b=position(src.vertices[src.indices[3*t+1]]),d=position(src.vertices[src.indices[3*t+2]]);
      const double edge=painter_short_minmax::maxOf(length(b-a),length(d-a),length(d-b));const int steps=std::max(1,int(std::ceil(edge/.004)));
      for(int i=0;i<=steps;++i)for(int jj=0;jj<=steps-i;++jj){const Cell q=cellOf(a+(b-a)*(double(i)/steps)+(d-a)*(double(jj)/steps));
       for(int dx=-3;dx<=3;dx+=6)for(int dy=-3;dy<=3;dy+=6)for(int dz=-3;dz<=3;dz+=6)
        surfaceTiles.push_back(Cell{(q.x+dx)>>3,(q.y+dy)>>3,(q.z+dz)>>3});}
      core::sortUniqueCells(surfaceTiles);};
     if(boundary){++w.boundary;surfaceTilesOf();}
     bool placed=false;
     // Row 1 (2026-10-23): a straddling face with no tissue under any corner (a collar or hood panel standing off the body)
     // that one of its owners poses outright is not also frozen onto an owner that cannot pose it. The R62 torn-band side rule
     // gives such corners depth 0, which skipped the stand-off vote above, so Dux collar panels were drawn whole on the torso
     // and, frozen, on the severed head (the flaps). Faces over tissue and single-owner faces are unchanged.
     bool noFrozenCopy=false;
     if(nk>1){bool standoff=true;
      for(int q=0;q<3&&standoff;++q){const auto&v=src.vertices[src.indices[3*t+q]];const Vec p=position(v);Vec n;int km;double dm=1;
       if(direction(v,n)){km=L.along(p+n*pitch,n*-1.,.06,&dm);if(km<0)km=L.along(p-n*pitch,n,.02,&dm);}else km=L.at(p);standoff=km<0;}
      for(int s=0;s<nk&&standoff&&!noFrozenCopy;++s){bool all=true;for(int q=0;q<3&&all;++q)all=anchored(std::size_t(K[s]),src.vertices[src.indices[3*t+q]]);noFrozenCopy=all;}}
     for(int s=0;s<nk;++s){const std::size_t k=std::size_t(K[s]);bool fine=true;
      for(int q=0;q<3&&fine;++q)fine=anchored(k,src.vertices[src.indices[3*t+q]]);
      if(!fine&&noFrozenCopy)continue;
      if(!fine){bool freeze=true;for(int q=0;q<3&&freeze;++q)freeze=frozenPosable(k,src.vertices[src.indices[3*t+q]]);
       if(!freeze){++w.unanchored;continue;}
       for(int q=0;q<3;++q)freezeInto(w.anchors[k],k,src.vertices[src.indices[3*t+q]]);++w.frozen;}
      placed=true;w.tris[k].push_back(std::uint32_t(t));if(boundary)w.boxes[k].insert(w.boxes[k].end(),surfaceTiles.begin(),surfaceTiles.end());}
     // The voted host cannot pose one of the corners: another corner owner that can keeps the face visible, as a
     // boundary triangle (the removal field decides per pixel); none -> left off, as before.
     for(int s=0;s<na&&!placed;++s){const std::size_t k=std::size_t(alt[s]);bool fine=true;
      for(int q=0;q<3&&fine;++q)fine=anchored(k,src.vertices[src.indices[3*t+q]]);
      if(!fine)continue;placed=true;if(!boundary){boundary=true;++w.boundary;}surfaceTilesOf();
      w.tris[k].push_back(std::uint32_t(t));w.boxes[k].insert(w.boxes[k].end(),surfaceTiles.begin(),surfaceTiles.end());}
     // Row 2 (2026-10-23): a face no voted owner can hold (left off: a hole) goes to the one piece that poses every corner
     // outright. Dux 0011 chin faces skinned to the jaw, over the head's tissue 4-6 cm in, were given to the torso by the R62
     // torn-band side rule; the torso cannot pose the jaw, so they were dropped (a head-side hole). Placed faces are unchanged.
     if(!placed){int to=-1;bool unique=true;
      for(std::size_t k2=0;k2<P&&unique;++k2){bool all=true;for(int q=0;q<3&&all;++q)all=anchored(k2,src.vertices[src.indices[3*t+q]]);
       if(all){if(to>=0)unique=false;else to=int(k2);}}
      if(unique&&to>=0){placed=true;if(!boundary){boundary=true;++w.boundary;}surfaceTilesOf();
       w.tris[std::size_t(to)].push_back(std::uint32_t(t));w.boxes[std::size_t(to)].insert(w.boxes[std::size_t(to)].end(),surfaceTiles.begin(),surfaceTiles.end());}}
     // A newly unsupported contour must never silently lose its original face.
     need(!contourFallback||placed,"skin_contour_influences");}
   }else for(std::size_t t=r.lo;t<r.hi;++t){const auto&a=src.vertices[src.indices[3*t]];const auto&b=src.vertices[src.indices[3*t+1]];const auto&d=src.vertices[src.indices[3*t+2]];
     int ow[3]={o[src.indices[3*t]],o[src.indices[3*t+1]],o[src.indices[3*t+2]]};
     int k=ow[0]>=0&&(ow[0]==ow[1]||ow[0]==ow[2]||(ow[1]<0&&ow[2]<0))?ow[0]:(ow[1]>=0&&(ow[1]==ow[2]||ow[2]<0)?ow[1]:(ow[2]>=0?ow[2]:ow[0]));
     const Vec pa=position(a),pb=position(b),pd=position(d);
     if(k<0){Vec na,nb,nd;const bool hn=direction(a,na)&&direction(b,nb)&&direction(d,nd);Vec nm=hn?na+nb+nd:Vec{0,0,0};const double lm=length(nm);
      const bool use=hn&&lm>1e-9;if(use)nm=nm*(1./lm);
      for(const Vec&q:{(pa+pb+pd)*(1./3),(pa+pb)*.5,(pb+pd)*.5,(pa+pd)*.5}){if(k>=0)break;if(inEnvelope(use?q-nm*.003:q))k=gapOwner(q,nm,use);}}
     if(k<0)k=nearestShell((pa+pb+pd)*(1./3),0,6);
     if(k<0){++w.channelDropped;continue;}
     if((ow[0]<0||ow[0]==k)&&(ow[1]<0||ow[1]==k)&&(ow[2]<0||ow[2]==k)&&singleAround(pa,pb,pd,k)){
      if(wallAnchored(std::size_t(k),a)&&wallAnchored(std::size_t(k),b)&&wallAnchored(std::size_t(k),d))w.tris[std::size_t(k)].push_back(std::uint32_t(t));else ++w.unanchored;
      continue;}
     int K[12];int nk=0;auto add=[&](int q){if(q<0)return;for(int i=0;i<nk;++i)if(K[i]==q)return;if(nk<12)K[nk++]=q;};
     add(k);for(int q=0;q<3;++q)add(ow[q]);
     {Vec na,nb,nd;if(direction(a,na)&&direction(b,nb)&&direction(d,nd)){
       const double edge=painter_short_minmax::maxOf(length(pb-pa),length(pd-pa),length(pd-pb));const int steps=std::min(10,std::max(1,int(std::ceil(edge/.02))));
       for(int i=0;i<=steps;++i)for(int jj=0;jj<=steps-i;++jj){const double u=double(i)/steps,v=double(jj)/steps;
        Vec n=na*(1-u-v)+nb*u+nd*v;const double l=length(n);if(!(l>1e-9))continue;add(backing(pa+(pb-pa)*u+(pd-pa)*v,n*(1/l)));}}}
     std::vector<Cell> surfaceTiles;
     if(nk>0){const double edge=painter_short_minmax::maxOf(length(pb-pa),length(pd-pa),length(pd-pb));const int steps=std::max(1,int(std::ceil(edge/.004)));
      for(int i=0;i<=steps;++i)for(int jj=0;jj<=steps-i;++jj){const Cell q=cellOf(pa+(pb-pa)*(double(i)/steps)+(pd-pa)*(double(jj)/steps));
       for(int dx=-3;dx<=3;dx+=6)for(int dy=-3;dy<=3;dy+=6)for(int dz=-3;dz<=3;dz+=6)surfaceTiles.push_back(Cell{(q.x+dx)>>3,(q.y+dy)>>3,(q.z+dz)>>3});}
      core::sortUniqueCells(surfaceTiles);}
     for(int q=0;q<nk;++q){const std::size_t kk=std::size_t(K[q]);
      if(!wallAnchored(kk,a)||!wallAnchored(kk,b)||!wallAnchored(kk,d)){++w.unanchored;continue;}
      w.tris[kk].push_back(std::uint32_t(t));if(nk>0)w.boxes[kk].insert(w.boxes[kk].end(),surfaceTiles.begin(),surfaceTiles.end());}}}});
 stats.triangleMs=core::ms(tt0);
 static const bool lowpolyProf=std::getenv("GFY_LOWPOLY_PROF")!=nullptr;
 if(lowpolyProf)std::cerr<<"lowpoly_prof tile_map_ms="<<tileBuildMs<<" skin_thread_ms="<<skinNs/1e6<<" wall_thread_ms="<<wallNs/1e6<<" triangle_ms="<<stats.triangleMs<<" skin_jobs+wall_jobs="<<tjobs.size()<<std::endl;
 // Each range's tile lists hold every boundary triangle's tiles, mostly shared with its neighbours: deduplicate in
 // parallel before the serial merge (the merge copied millions of repeated keys).
 core::parallelChunks(tw.size(),1,[&](std::size_t first,std::size_t last,std::size_t){
  for(std::size_t j=first;j<last;++j)for(auto&list:tw[j].boxes){std::sort(list.begin(),list.end());list.erase(std::unique(list.begin(),list.end()),list.end());}});
 for(std::size_t c=0;c<children.size();++c){work[c].tris.resize(P);work[c].boxes.resize(P);work[c].clipped.resize(P);work[c].owned.resize(P);work[c].anchors.resize(P);}
 for(std::size_t j=0;j<tjobs.size();++j){auto&w=work[tjobs[j].c];auto&s=tw[j];
  for(std::size_t k=0;k<P;++k){w.anchors[k].insert(s.anchors[k].begin(),s.anchors[k].end());w.owned[k].insert(w.owned[k].end(),std::make_move_iterator(s.owned[k].begin()),std::make_move_iterator(s.owned[k].end()));w.clipped[k].insert(w.clipped[k].end(),std::make_move_iterator(s.clipped[k].begin()),std::make_move_iterator(s.clipped[k].end()));w.tris[k].insert(w.tris[k].end(),s.tris[k].begin(),s.tris[k].end());w.boxes[k].insert(w.boxes[k].end(),s.boxes[k].begin(),s.boxes[k].end());}
  w.dropped+=s.dropped;w.unanchored+=s.unanchored;w.boundary+=s.boundary;w.channelDropped+=s.channelDropped;w.frozen+=s.frozen;}
 // E5 (logging): per piece the kept skin triangles and the SHELL triangles (no corner backed by tissue) with centroid, area and
 // whether the model had anatomy under it before any damage (6 cm along the inward normal: cloth emptied by damage)
 struct ShellTri{Vec c;double area;bool emptied;};std::vector<std::vector<ShellTri>> shellOf(P);std::vector<std::size_t> keptOf(P,0);
 {static const char*dumpPath=std::getenv("GFY_LP_SHELL_DUMP");std::unique_ptr<std::ofstream> dump;if(dumpPath)dump=std::make_unique<std::ofstream>(dumpPath,std::ios::app);
  // split2 (2026-11-11): the tissue marches of this read-out (up to 280 lookups per triangle; 27 ms for 5,700 shell
  // triangles of a shot-up body) ran on the job's thread alone. A triangle's answer is a pure function of read-only data:
  // the candidates are listed in the former order, answered on the pool, and collected in the former order (the sums
  // made over each piece's list later run in list order).
  struct ShellCandidate{std::uint32_t c,k,t;};std::vector<ShellCandidate> candidates;
  for(std::size_t c=0;c<children.size();++c){if(kind[c]!=kind_skin)continue;const auto&src=children[c];
   for(std::size_t k=0;k<P;++k){keptOf[k]+=work[c].tris[k].size()+work[c].clipped[k].size()+work[c].owned[k].size();
    for(const auto t:work[c].tris[k]){const auto i0=src.indices[3*t],i1=src.indices[3*t+1],i2=src.indices[3*t+2];
     if(!(bare[c][i0]||bare[c][i1]||bare[c][i2]))continue; // a corner with no tissue under it
     candidates.push_back({std::uint32_t(c),std::uint32_t(k),std::uint32_t(t)});}}}
  struct ShellAnswer{ShellTri tri;Vec n;char keep;};std::vector<ShellAnswer> answers(candidates.size());
  eachBlock(candidates.size(),64,[&](std::size_t first,std::size_t last,std::size_t){
   for(std::size_t i=first;i<last;++i){const auto&cd=candidates[i];auto&out=answers[i];out.keep=0;const auto&src=children[cd.c];const std::size_t k=cd.k,t=cd.t;
     const auto i0=src.indices[3*t],i1=src.indices[3*t+1],i2=src.indices[3*t+2];
     const Vec a=position(src.vertices[i0]),b=position(src.vertices[i1]),d=position(src.vertices[i2]);const Vec m=(a+b+d)*(1./3);
     Vec n0,n1,n2;Vec nn=(direction(src.vertices[i0],n0)?n0:Vec{0,0,0})+(direction(src.vertices[i1],n1)?n1:Vec{0,0,0})+(direction(src.vertices[i2],n2)?n2:Vec{0,0,0});
     const double ln=length(nn);bool emptied=false;if(!(ln>1e-9))continue;nn=nn*(1/ln);
     if(L.along(m+nn*pitch,nn*-1.,.06)==int(k)||L.along(m-nn*pitch,nn,.02)==int(k))continue; // the centre lies over this piece's tissue
     for(double s=pitch*.5;s<=.06&&!emptied;s+=pitch*.5){const Cell q=cellOf(m-nn*s);emptied=grid.valid(q)&&grid.rawMaterialBase(q)!=0;}
     out.tri={m,.5*length(cross(b-a,d-a)),emptied};out.n=nn;out.keep=1;}});
  for(std::size_t i=0;i<candidates.size();++i){if(!answers[i].keep)continue;const auto&cd=candidates[i];const auto&src=children[cd.c];const std::size_t c=cd.c,k=cd.k,t=cd.t;
     const auto i0=src.indices[3*t],i1=src.indices[3*t+1],i2=src.indices[3*t+2];const Vec m=answers[i].tri.c,nn=answers[i].n;const bool emptied=answers[i].tri.emptied;
     shellOf[k].push_back(answers[i].tri);
     if(dump)*dump<<"SHELLTRI piece="<<k<<" component="<<components[k]<<" child="<<src.child_index<<" source_child="<<src.material_source_child<<" tri="<<t<<" c="<<m.x<<","<<m.y<<","<<m.z
      <<" n="<<nn.x<<","<<nn.y<<","<<nn.z<<" how="<<int(howOf[c][i0])<<int(howOf[c][i1])<<int(howOf[c][i2])<<" emptied="<<emptied<<"\n";}}
 std::vector<std::uint32_t> empty;
 for(std::size_t k=0;k<P;++k){bool any=false;for(const auto&w:work)any|=!w.tris[k].empty()||!w.clipped[k].empty()||!w.owned[k].empty();
  if(!any&&(!deferredSourceComponents||!deferredSourceComponents->count(components[k])))empty.push_back(components[k]);}
 if(!empty.empty())throw EmptyPieces(std::move(empty));
 stats.ownerMs=core::ms(t0);
 // ---- foreign marks: per 8^3 tile around boundary triangles, label solid cells by piece and grow the labels 3 cells
 // into air; every cell labelled with ANOTHER piece is removed from this piece's field.
 const auto m0=core::Clock::now();
 // Per-piece tile lists as sorted unique vectors (tree sets of every boundary triangle's tiles, duplicates included,
 // took most of the mask time: jaw split 130 ms).
 std::vector<std::vector<Cell>> tilesOf(P);std::vector<Cell> tiles;
 const int tilesX=(grid.size.x+7)/8,tilesY=(grid.size.y+7)/8,tilesZ=(grid.size.z+7)/8;
 core::parallelChunks(P,1,[&](std::size_t first,std::size_t last,std::size_t){
  for(std::size_t k=first;k<last;++k){auto&list=tilesOf[k];
   for(const auto&w:work)for(const auto&key:w.boxes[k])
    if(key.x>=0&&key.y>=0&&key.z>=0&&key.x<tilesX&&key.y<tilesY&&key.z<tilesZ)list.push_back(key);
   std::sort(list.begin(),list.end());list.erase(std::unique(list.begin(),list.end()),list.end());}});
 const double tilesOfMs=core::ms(m0);std::atomic<long long> labelNs{0};std::atomic<std::size_t> labelled{0};
 for(const auto&list:tilesOf)tiles.insert(tiles.end(),list.begin(),list.end());
 std::sort(tiles.begin(),tiles.end());tiles.erase(std::unique(tiles.begin(),tiles.end()),tiles.end());stats.tiles=tiles.size();
 std::vector<std::vector<std::pair<Cell,core::Tile>>> marks(P);std::mutex markMutex;std::atomic<std::size_t> foreign{0};
 const double tileListMs=core::ms(m0);
 eachBlock(tiles.size(),32,[&](std::size_t first,std::size_t last,std::size_t){const auto blockStart=core::Clock::now();
  core::TileCache cache(grid);constexpr int H=3,W=8+2*H;std::vector<int> label(W*W*W),next(W*W*W);
  std::vector<std::vector<std::pair<Cell,core::Tile>>> local(P);
  for(std::size_t i=first;i<last;++i){const Cell key=tiles[i];const Cell base{key.x*8-H,key.y*8-H,key.z*8-H};
   {std::uint16_t seen=0;bool mixed=false; // one piece around and only its triangles here: nothing foreign to mark
    for(int dx=-1;dx<=1&&!mixed;++dx)for(int dy=-1;dy<=1&&!mixed;++dy)for(int dz=-1;dz<=1;++dz){const auto v=tileAt(key.x+dx,key.y+dy,key.z+dz);
     if(!v)continue;if(v==0xFFFF||(seen&&v!=seen)){mixed=true;break;}seen=v;}
    if(!mixed){bool other=false;for(std::size_t k=0;k<P&&!other;++k)if(std::uint16_t(k+1)!=seen&&std::binary_search(tilesOf[k].begin(),tilesOf[k].end(),key))other=true;
     if(!seen||!other)continue;}}
   auto at=[&](int x,int y,int z){return (x*W+y)*W+z;};++labelled;
   const SeverState::TileMaterial*near[27]={};
   if(tissueTable)for(int i=0;i<27;++i){const Cell k{key.x+i/9-1,key.y+(i/3)%3-1,key.z+i%3-1};const auto it=tissueTable->find(k);near[i]=it==tissueTable->end()?nullptr:&it->second;}
   auto tissueNear=[&](const Cell&c)->bool{if(!tissueTable)return grid.rawMaterialBase(c)!=0;const Cell k=Grid::tileKey(c);
    const auto*t=near[((k.x-key.x+1)*3+(k.y-key.y+1))*3+(k.z-key.z+1)];if(!t)return false;const auto l=Grid::local(c);return ((t->solid[l/64]>>(l%64))&1u)!=0;};
   const core::Tile*removedNear[27]={};const bool columnBits=!grid.ownership&&tissueTable;
   if(columnBits)for(int i=0;i<27;++i){const Cell k{key.x+i/9-1,key.y+(i/3)%3-1,key.z+i%3-1};const auto it=grid.tiles.find(k);removedNear[i]=it==grid.tiles.end()?nullptr:&it->second;}
   for(int x=0;x<W;++x)for(int y=0;y<W;++y){const gfy_connectivity::Span*span=nullptr,*spanEnd=nullptr;std::uint32_t removedBits=0,tissueBits=0;
    {const int cx=base.x+x,cy=base.y+y;if(cx>=0&&cy>=0&&cx<grid.size.x&&cy<grid.size.y){const auto ci=columnOf[std::size_t(cy)*std::size_t(grid.size.x)+std::size_t(cx)];
      if(ci>=0){const auto&col=volume.columns[std::size_t(ci)];span=volume.spans.data()+col.first;spanEnd=span+col.count;}
      if(columnBits){const int tx=(cx>>3)-key.x+1,ty=(cy>>3)-key.y+1,lx=cx&7,ly=cy&7; // window z: the top 8-H cells of the tile below, 8, H
       for(int dz=0;dz<3;++dz){const int i=(tx*3+ty)*3+dz,shift=dz*8-8+H;
        const std::uint32_t r=removedNear[i]?std::uint32_t((removedNear[i]->removed[std::size_t(lx)]>>(ly*8))&255u):0u,t=near[i]?std::uint32_t((near[i]->solid[std::size_t(lx)]>>(ly*8))&255u):0u;
        if(shift>=0){removedBits|=r<<shift;tissueBits|=t<<shift;}else{removedBits|=r>>-shift;tissueBits|=t>>-shift;}}}}}
    for(int z=0;z<W;++z){const Cell c{base.x+x,base.y+y,base.z+z};int&l=label[at(x,y,z)];
     if(!grid.valid(c)){l=-1;continue;}if(columnBits?((removedBits>>z)&1u)!=0:cache.removed(c)){l=-3;continue;}
     if(columnBits?((tissueBits>>z)&1u)!=0:tissueNear(c)){while(span!=spanEnd&&span->hi<=c.z)++span;const std::uint32_t comp=span!=spanEnd&&span->lo<=c.z?span->component:0u;
      l=comp<pieceOf.size()&&pieceOf[comp]>=0?pieceOf[comp]:-2;}else l=-1;}}
   for(int step=0;step<H;++step){next=label;const int lo=step+1,hi=W-1-step;
    for(int x=lo;x<hi;++x)for(int y=lo;y<hi;++y)for(int z=lo;z<hi;++z){if(label[at(x,y,z)]!=-1)continue;
     const int q[6][3]={{x-1,y,z},{x+1,y,z},{x,y-1,z},{x,y+1,z},{x,y,z-1},{x,y,z+1}};
     for(const auto&p:q){if(p[0]<0||p[1]<0||p[2]<0||p[0]>=W||p[1]>=W||p[2]>=W)continue;const int v=label[at(p[0],p[1],p[2])];if(v>=0){next[at(x,y,z)]=v;break;}}}
    label.swap(next);}
   for(std::size_t k=0;k<P;++k){if(!std::binary_search(tilesOf[k].begin(),tilesOf[k].end(),key))continue;core::Tile t{};std::size_t n=0;
    for(int x=0;x<8;++x)for(int y=0;y<8;++y)for(int z=0;z<8;++z){const int v=label[at(x+H,y+H,z+H)];if(v<0||v==int(k))continue;
     const Cell c{key.x*8+x,key.y*8+y,key.z*8+z};const auto bit=Grid::local(c);t.removed[bit/64]|=std::uint64_t(1)<<(bit%64);++n;}
    if(n){local[k].emplace_back(key,t);foreign+=n;}}}
  std::lock_guard<std::mutex> lock(markMutex);for(std::size_t k=0;k<P;++k)marks[k].insert(marks[k].end(),local[k].begin(),local[k].end());
  labelNs+=std::chrono::duration_cast<std::chrono::nanoseconds>(core::Clock::now()-blockStart).count();});
 stats.foreign=foreign;stats.maskMs=core::ms(m0);
 if(lowpolyProf)std::cerr<<"lowpoly_prof mask tiles_of_ms="<<tilesOfMs<<" tile_list_ms="<<tileListMs<<" label_thread_ms="<<labelNs/1e6<<" labelled="<<labelled<<"/"<<tiles.size()<<" mask_ms="<<stats.maskMs<<std::endl;
 // ---- assemble
 struct ShellSeen{std::size_t visible{},emptied{};double area{};Vec at{0,0,0};};std::vector<ShellSeen> shellSeen(P); // E5
 std::vector<unsigned char> supportState(P,0);std::vector<std::size_t> supportBytes(P,0);
 std::vector<bound_surface::Piece> out(P);std::vector<std::exception_ptr> failed(P);std::vector<std::array<std::size_t,2>> counted(P,std::array<std::size_t,2>{});
 if(marksOut)marksOut->resize(P);
 eachBlock(P,1,[&](std::size_t first,std::size_t last,std::size_t){for(std::size_t k=first;k<last;++k)try{auto&piece=out[k];piece.volume_component=components[k];auto&mesh=piece.mesh;
  painter_detail::SkinOwnershipBuilder ownership(field.identity);
  mesh.tissue_field=published.tissue_field;mesh.analytic_coverage_admitted=published.analytic_coverage_admitted;
  auto channels=std::make_shared<painter_detail::AnalyticChannelField>();
  channels->identity=field.identity;channels->cuts=field.cuts;channels->limitations=field.limitations;channels->words=field.words;channels->profiles=field.profiles;
  unsigned next=originalChildren;
  for(std::size_t c=0;c<children.size();++c){const auto&src=children[c];const auto&list=work[c].tris[k];const auto&clipped=work[c].clipped[k];const auto&owned=work[c].owned[k];if(list.empty()&&clipped.empty()&&owned.empty())continue;
   for(const auto&a:work[c].anchors[k]){auto [entry,fresh]=piece.constrained_anchors.emplace(a);need(fresh||entry->second==a.second,"skin_frozen_anchor_conflict");}
   gfy_packed::Child child;child.material=src.material;child.material_source_child=src.material_source_child;
   child.skin_clip=!src.material&&(src.skin_clip||!work[c].boxes[k].empty());
   if(!src.material&&src.child_index<originalChildren)child.child_index=src.child_index;
   else{need(next<gfy_packed::max_child_slots,"lowpoly_piece_child_slots");child.child_index=std::uint16_t(next++);}
   std::vector<std::int32_t> map(src.vertices.size(),-1);const bool faces=src.original_faces.size()*3==src.indices.size();
   for(const auto t:list){for(int j=0;j<3;++j){const auto v=src.indices[3*t+j];
     if(map[v]<0){map[v]=std::int32_t(child.vertices.size());auto row=src.vertices[v];double sum=0;for(auto x:row.weight)sum+=x;
      const auto*prior=published.skin_ownership?published.skin_ownership->child(src.child_index):nullptr;
      if(!(prior&&prior->data[t][1])&&sum>0)for(auto&x:row.weight)x=decltype(+x)(x/sum);
      if(src.material){if(frozenPosable(k,row))freezeInto(piece.constrained_anchors,k,row);else row=wallRow(k,row);}child.vertices.push_back(row);}
     child.indices.push_back(std::uint16_t(map[v]));}
    child.original_faces.push_back(faces?src.original_faces[t]:~std::uint32_t(0));if(!src.material)ownership.append(child.child_index,published.skin_ownership.get(),src.child_index,t);}
   if(!clipped.empty()){
    core::packing::VertexIndex vi;vi.reset(child.vertices.size()+3*clipped.size());
    for(const auto&t:clipped){need(child.vertices.size()+3<=65535,"skin_contour_vertex_budget");
     for(const auto&v:t.v){const auto index=vi.insert(child.vertices,v,core::packing::vertexHash(v));child.indices.push_back(index.first);}
     child.original_faces.push_back(t.face);ownership.append(child.child_index,published.skin_ownership.get(),src.child_index,t.source);}
   }
   for(const auto&t:owned){for(const auto&a:t.anchors){auto [entry,fresh]=piece.constrained_anchors.emplace(a);need(fresh||entry->second==a.second,"skin_owner_anchor_conflict");}
    need(!src.material,"skin_owner_material");need(child.vertices.size()+3<=65535,"skin_owner_vertex_budget");
    for(unsigned q=0;q<3;++q){child.indices.push_back(std::uint16_t(child.vertices.size()));child.vertices.push_back(src.vertices[src.indices[3*t.source+q]]);}
    child.original_faces.push_back(src.original_faces[t.source]);ownership.append(child.child_index,published.skin_ownership.get(),src.child_index,t.source,t.planes);
   }
   counted[k][src.material?1:0]+=list.size()+clipped.size()+owned.size();piece.triangles+=list.size()+clipped.size()+owned.size();
   if(src.material){const auto*prim=field.child(src.child_index);painter_detail::ChannelPrimitiveField pf;pf.child=child.child_index;pf.count=unsigned(list.size());
    pf.data.assign(((list.size()+pf.width-1)/pf.width)*pf.width,std::array<std::uint32_t,2>{});
    for(std::size_t i=0;i<list.size();++i)pf.data[i]=prim->data[list[i]];channels->children.push_back(std::move(pf));}
   mesh.children.push_back(std::move(child));}
  gfy_packed::set_skin_ownership(mesh,ownership.finish());
  if(!channels->children.empty()){
   const painter_detail::PieceCellSupport missing;const auto&support=cellSupport.size()==P?cellSupport[k]:missing;
   const bool supportOK=painter_detail::appendPieceCellSupport(*channels,{grid.origin.x,grid.origin.y,grid.origin.z},grid.pitch,support,[](){sep::throwIfCanceled();});
   supportState[k]=supportOK?1u:2u;supportBytes[k]=supportOK?channels->bytes():0;
   mesh.analytic_channels=std::move(channels);
  }else mesh.analytic_coverage_admitted=false;
  if(!piece.triangles){
   // The caller has already proved an exact original accessory surface for
   // this ocular component. It appends that source before final admission.
   need(deferredSourceComponents&&deferredSourceComponents->count(piece.volume_component),"lowpoly_piece_empty");
   mesh.removal_field=published.removal_field;
   if(inherited)marks[k]=*inherited;
   if(marksOut)(*marksOut)[k]=marks[k];
   continue;
  }
  // The piece's removal field spans only its own bounds (+2 tiles): a hand's field is small instead of the whole
  // body's damage box (ten blasted bodies filled the 64 MB GPU field cache: child_render_prepare refusals).
  struct FieldTiles {std::unordered_map<Cell,core::Tile,core::CellHash> tiles;const void* ownership=nullptr;double pitch;Vec origin;decltype(grid.size) size;};
  FieldTiles masked{{},nullptr,grid.pitch,grid.origin,grid.size};
  {Cell lo{INT_MAX,INT_MAX,INT_MAX},hi{INT_MIN,INT_MIN,INT_MIN};
   for(const auto&ch:mesh.children)for(const auto&v:ch.vertices){const Cell c=cellOf(position(v));
    lo={std::min(lo.x,c.x>>3),std::min(lo.y,c.y>>3),std::min(lo.z,c.z>>3)};hi={std::max(hi.x,c.x>>3),std::max(hi.y,c.y>>3),std::max(hi.z,c.z>>3)};}
   lo={lo.x-2,lo.y-2,lo.z-2};hi={hi.x+2,hi.y+2,hi.z+2};
   for(const auto&it:grid.tiles){const Cell&t=it.first;if(t.x<lo.x||t.y<lo.y||t.z<lo.z||t.x>hi.x||t.y>hi.y||t.z>hi.z)continue;
    std::uint64_t any=0;for(auto w:it.second.removed)any|=w;if(any)masked.tiles.emplace(t,it.second);}
   if(inherited)for(const auto&m:*inherited){const Cell&t=m.first;if(t.x<lo.x||t.y<lo.y||t.z<lo.z||t.x>hi.x||t.y>hi.y||t.z>hi.z)continue;marks[k].push_back(m);}}
  for(const auto&m:marks[k]){auto&t=masked.tiles[m.first];for(unsigned q=0;q<8;++q)t.removed[q]|=m.second.removed[q];}
  {auto&sv=shellSeen[k];for(const auto&s:shellOf[k]){const Cell q=cellOf(s.c);const auto it=masked.tiles.find(Grid::tileKey(q));const auto bit=Grid::local(q); // E5: drawn = not in the removal field
    if(it!=masked.tiles.end()&&((it->second.removed[bit/64]>>(bit%64))&1u))continue;++sv.visible;if(!s.emptied)continue;++sv.emptied;sv.area+=s.area;sv.at=sv.at+s.c*s.area;}}
  if(marksOut)(*marksOut)[k]=marks[k];
  auto removal=painter_detail::buildDamageField(masked);need(removal&&removal->available,"lowpoly_piece_removal_field");
  mesh.removal_field=std::move(removal);
  need(piece.triangles>0,"lowpoly_piece_empty");}catch(...){failed[k]=std::current_exception();}});
 for(const auto&f:failed)if(f)std::rethrow_exception(f);
 for(std::size_t k=0;k<P;++k){stats.cellSupportAdmitted+=supportState[k]==1;stats.cellSupportRefused+=supportState[k]==2;stats.cellSupportBytes+=supportBytes[k];}
 for(const auto&c:counted){stats.skin+=c[0];stats.channel+=c[1];}
 for(std::size_t k=0;k<P;++k){const auto&sv=shellSeen[k];stats.keptShell+=shellOf[k].size();const Vec at=sv.area>0?sv.at*(1./sv.area):Vec{0,0,0}; // E5
  char item[160];std::snprintf(item,sizeof(item),"%s%zu:%zu/%zu/%zu/%zu/%.1f@%d,%d,%d",k?";":"",k,keptOf[k],shellOf[k].size(),sv.visible,sv.emptied,sv.area*1e4,int(std::lround(at.x*1000)),int(std::lround(at.y*1000)),int(std::lround(at.z*1000)));
  stats.shellByPiece+=item;}
 for(const auto&w:work){stats.dropped+=w.dropped;stats.unanchored+=w.unanchored;stats.boundary+=w.boundary;stats.marched+=w.marched;stats.byBone+=w.byBone;stats.unowned+=w.unowned;stats.nearest+=w.nearest;stats.channelDropped+=w.channelDropped;stats.frozen+=w.frozen;}
 stats.ms=core::ms(t0);return out;
}
}
