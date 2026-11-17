#pragma once
// Mask-only wounds, slice 2 (console painter_voxel_mask_only 2): separation pieces without a surface partition.
// Included by LiveSeparationWorker_impl.h inside gfy_live::separation_detail, after LowPolyPieces.h, and used instead of
// lowpoly_pieces::build when the published body is a mask-only publication (mesh.mask_volume: the original exterior children
// and a removal field, no wound geometry). The separation's decisions (remaining volume, pieces, connectivity, physics
// partitions, colliders, pose) are untouched; only what each piece DRAWS is decided here:
//  * the piece's MARKED field (MaskPieceField.h): removed cells, the other pieces' cells, and free cells whose nearest piece cell
//    (60 mm) is another piece's, in today's removal-field format. The renderer binds it as the piece's removal field; the
//    exterior is hidden where marked and the view ray steps to the first kept tissue, which is this piece's (slice 1's shader).
//  * the ORIGINAL exterior triangles of the published children that matter for the piece: a triangle is kept when any of 7
//    samples (corners, centre, edge midpoints) lies in a kept cell of the piece and every corner can be posed by the piece
//    (its bones, or a foreign bone frozen to its unique retained frontier exactly as the low-poly path freezes them).
//    No clipping at cell planes, no rebuilt surface, no ownership table.
//  * the COVERAGE BOX: an oriented box around the piece's torn cells, 24 vertices / 12 triangles skinned 100 % to one bone of the
//    piece (the dominant bone of the kept skin vertex nearest the box centre), texture u = -1000 (the shader's box sentinel:
//    never drawn as the exterior). Wound the same way as the model's own triangles (measured on the published skin).
//  * marks for the piece's seed: the marked tiles (a later hit on the piece rebuilds its field from them and its grid).
#include "MaskPieceField.h"
namespace mask_pieces {
struct Stats {
 painter_mask_pieces::Stats field;
 std::size_t selected{},dropped{},unposable{},frozen{},boxes{},boxTriangles{},fieldBytes{};
 double ms{},fieldMs{},selectMs{},packMs{};
};
inline double clockMs(){return std::chrono::duration<double,std::milli>(core::Clock::now().time_since_epoch()).count();}

template<class Anchorable,class RegionFrontier>
inline std::vector<bound_surface::Piece> build(const gfy_packed::Mesh&published,const Grid&grid,const gfy_connectivity::Topology&volume,
    const std::vector<std::uint32_t>&components,const std::set<std::uint16_t>&wholeChildren,std::uint16_t originalChildren,
    Anchorable&&anchorable,RegionFrontier&&regionFrontier,Stats&stats,
    const std::vector<std::pair<Cell,core::Tile>>*inherited,std::vector<std::vector<std::pair<Cell,core::Tile>>>*marksOut){
 const auto t0=core::Clock::now();
 const std::size_t P=components.size();need(P>=1&&P<=12,"mask_piece_count");
 need(published.mask_volume&&published.removal_field&&published.removal_field->available&&!published.children.empty(),"mask_piece_published_mesh");
 std::vector<int> pieceOf(volume.components.size(),-1);
 for(std::size_t k=0;k<P;++k){need(components[k]>0&&components[k]<pieceOf.size(),"mask_piece_component");pieceOf[components[k]]=int(k);}
 std::vector<std::int32_t> columnOf(std::size_t(grid.size.x)*std::size_t(grid.size.y),-1);
 for(std::size_t ci=0;ci<volume.columns.size();++ci){const auto&c=volume.columns[ci];
  if(c.x>=0&&c.y>=0&&c.x<grid.size.x&&c.y<grid.size.y)columnOf[std::size_t(c.y)*std::size_t(grid.size.x)+std::size_t(c.x)]=std::int32_t(ci);}
 // A piece separated again: its seed's marked tiles (the earlier separation's foreign cells) count as removed.
 std::unordered_map<Cell,core::Tile,core::CellHash> inheritedTiles;
 if(inherited)for(const auto&m:*inherited){auto&t=inheritedTiles[m.first];for(unsigned q=0;q<8;++q)t.removed[q]|=m.second.removed[q];}
 // One tile's cells: the piece of every cell (the topology's spans of the tile's 64 columns) and its removed non-piece cells
 // (the grid's tile bits, the inherited marks; on an owned grid also the cells its ownership excludes, read per cell).
 auto tileCells=[&](const painter_mask_pieces::Key&k,painter_mask_pieces::Cells&pc,painter_mask_pieces::Bits&rm){
  pc.fill(-1);rm.fill(0);const int z0=k.z*8;
  for(int u=0;u<8;++u)for(int v=0;v<8;++v){const int x=k.x*8+u,y=k.y*8+v;
   if(x<0||y<0||x>=grid.size.x||y>=grid.size.y)continue;const auto ci=columnOf[std::size_t(y)*std::size_t(grid.size.x)+std::size_t(x)];if(ci<0)continue;
   const auto&c=volume.columns[std::size_t(ci)];const auto first=volume.spans.begin()+c.first,last=first+c.count;
   auto i=std::upper_bound(first,last,z0,[](int zz,const gfy_connectivity::Span&s){return zz<s.lo;});if(i!=first)--i;
   for(;i!=last&&i->lo<z0+8;++i){if(i->hi<=z0)continue;const int p=i->component<pieceOf.size()?pieceOf[i->component]:-1;if(p<0)continue;
    for(int z=std::max(i->lo,z0);z<std::min(i->hi,z0+8);++z)pc[std::size_t((u*8+v)*8+(z-z0))]=static_cast<signed char>(p);}}
  {const auto it=grid.tiles.find(Cell{k.x,k.y,k.z});if(it!=grid.tiles.end())for(unsigned q=0;q<8;++q)rm[q]|=it->second.removed[q];}
  if(!inheritedTiles.empty()){const auto it=inheritedTiles.find(Cell{k.x,k.y,k.z});if(it!=inheritedTiles.end())for(unsigned q=0;q<8;++q)rm[q]|=it->second.removed[q];}
  if(grid.ownership)for(unsigned b=0;b<512;++b){if(pc[b]>=0)continue;const Cell c{k.x*8+int(b>>6),k.y*8+int((b>>3)&7u),z0+int(b&7u)};if(grid.removed(c))rm[b>>6]|=std::uint64_t(1)<<(b&63u);}
  for(unsigned b=0;b<512;++b)if(pc[b]>=0)rm[b>>6]&=~(std::uint64_t(1)<<(b&63u));};
 // kept tissue for the torn-cell test: the shader's SOLID class (anatomy label not 0 and not garment air 16; padding counts)
 auto tissueAt=[&](int x,int y,int z){const auto m=grid.rawMaterialBase(Cell{x,y,z});return m!=0&&m!=16;};
 auto parallel=[&](std::size_t n,const auto&body){lowpoly_pieces::eachBlock(n,1,[&](std::size_t f,std::size_t l,std::size_t){body(f,l);});};
 // tile summary from the spans and the grid's tiles (no per-cell pass over the whole grid)
 const std::function<void(std::vector<std::uint16_t>&,std::vector<char>&,const std::array<int,3>&,const std::array<int,3>&)> summary=
  [&](std::vector<std::uint16_t>&present,std::vector<char>&anyRemoved,const std::array<int,3>&tlo,const std::array<int,3>&tn){
   auto ti=[&](int x,int y,int z){return (std::size_t(x-tlo[0])*std::size_t(tn[1])+std::size_t(y-tlo[1]))*std::size_t(tn[2])+std::size_t(z-tlo[2]);};
   auto inside=[&](int x,int y,int z){return x>=tlo[0]&&y>=tlo[1]&&z>=tlo[2]&&x<tlo[0]+tn[0]&&y<tlo[1]+tn[1]&&z<tlo[2]+tn[2];};
   for(const auto&col:volume.columns){const int tx=col.x>>3,ty=col.y>>3;
    for(auto s=col.first;s<col.first+col.count;++s){const auto&sp=volume.spans[s];if(sp.component>=pieceOf.size()||pieceOf[sp.component]<0)continue;
     const auto bit=std::uint16_t(1u<<unsigned(pieceOf[sp.component]));
     for(int tz=sp.lo>>3;tz<=((sp.hi-1)>>3);++tz)if(inside(tx,ty,tz))present[ti(tx,ty,tz)]|=bit;}}
   for(const auto&t:grid.tiles){std::uint64_t any=0;for(auto w:t.second.removed)any|=w;if(any&&inside(t.first.x,t.first.y,t.first.z))anyRemoved[ti(t.first.x,t.first.y,t.first.z)]=1;}
   for(const auto&t:inheritedTiles)if(inside(t.first.x,t.first.y,t.first.z))anyRemoved[ti(t.first.x,t.first.y,t.first.z)]=1;
   // an owned grid (a piece's): the cells its ownership excludes read as removed - mark every tile it does not hold purely
   if(grid.ownership)for(int x=0;x<tn[0];++x)for(int y=0;y<tn[1];++y)for(int z=0;z<tn[2];++z){const Cell k{tlo[0]+x,tlo[1]+y,tlo[2]+z};
    if(!grid.ownership->pureTile(Cell{k.x*8,k.y*8,k.z*8}))anyRemoved[ti(k.x,k.y,k.z)]=1;}
  };
 // distance grid: blocks of 2^3 cells (4 mm at 2 mm cells); GFY_MASK_EDT_BLOCK=1 exact cell distances, 4 = 8 mm blocks
 static const int edtBlock=[]{const char*e=std::getenv("GFY_MASK_EDT_BLOCK");const int b=e?std::atoi(e):2;return b<1?1:(b>4?4:b);}();
 const auto f0=core::Clock::now();
 auto fields=painter_mask_pieces::build({0,0,0},{grid.size.x,grid.size.y,grid.size.z},unsigned(P),{grid.origin.x,grid.origin.y,grid.origin.z},grid.pitch,
  0.06,2.0,tileCells,tissueAt,parallel,stats.field,&clockMs,edtBlock,&summary);
 stats.fieldMs=core::ms(f0);
 auto markedAt=[&](std::size_t k,const Vec&p){
  const double fx=(p.x-grid.origin.x)/grid.pitch,fy=(p.y-grid.origin.y)/grid.pitch,fz=(p.z-grid.origin.z)/grid.pitch;
  if(!(fx>=0&&fy>=0&&fz>=0&&fx<grid.size.x&&fy<grid.size.y&&fz<grid.size.z))return true; // outside the grid: never this piece's
  const int x=int(fx),y=int(fy),z=int(fz);const auto it=fields[k].marked.find(painter_mask_pieces::Key{x>>3,y>>3,z>>3});
  if(it==fields[k].marked.end())return false;const unsigned b=painter_mask_pieces::bitOf(x,y,z);return ((it->second[b>>6]>>(b&63u))&1u)!=0;};
 // ---- posing rules (the low-poly path's: anchorable bones; a foreign bone frozen to its unique retained frontier)
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
 // ---- triangle selection per piece and child (parallel over children; each child writes its own slots)
 const auto s0=core::Clock::now();const auto&children=published.children;
 auto position=[](const gfy_packed::Vertex&v){return Vec{v.surface[0],v.surface[1],v.surface[2]};};
 struct Sel {std::vector<std::vector<std::uint32_t>> tris;std::vector<std::map<std::uint16_t,std::uint16_t>> anchors;std::size_t dropped{},unposable{},frozen{};};
 std::vector<Sel> sel(children.size());
 parallel(children.size(),[&](std::size_t first,std::size_t last){for(std::size_t c=first;c<last;++c){auto&s=sel[c];s.tris.resize(P);s.anchors.resize(P);
  const auto&src=children[c];if(src.material||src.indices.empty()||wholeChildren.count(src.material_source_child))continue;
  const std::size_t T=src.indices.size()/3;
  for(std::size_t t=0;t<T;++t){const gfy_packed::Vertex*v[3]={&src.vertices[src.indices[3*t]],&src.vertices[src.indices[3*t+1]],&src.vertices[src.indices[3*t+2]]};
   const Vec a=position(*v[0]),b=position(*v[1]),d=position(*v[2]);const Vec samples[7]={a,b,d,(a+b+d)*(1./3),(a+b)*.5,(b+d)*.5,(a+d)*.5};
   bool any=false;
   for(std::size_t k=0;k<P;++k){bool kept=false;for(const auto&q:samples)if(!markedAt(k,q)){kept=true;break;}if(!kept)continue;any=true;
    bool fine=anchored(k,*v[0])&&anchored(k,*v[1])&&anchored(k,*v[2]);
    if(!fine){bool freeze=frozenPosable(k,*v[0])&&frozenPosable(k,*v[1])&&frozenPosable(k,*v[2]);if(!freeze){++s.unposable;continue;}
     for(const auto*row:v)for(unsigned w=0;w<4;++w){const auto bb=row->bone[w];if(bb<64&&!ok[k][bb]&&gfy_signed_hw::dependency(row->weight,w,negligible_skin_weight))s.anchors[k].emplace(bb,front[k][bb]);}
     ++s.frozen;}
    s.tris[k].push_back(std::uint32_t(t));}
   if(!any)++s.dropped;}}});
 stats.selectMs=core::ms(s0);const auto p0=core::Clock::now();
 for(const auto&s:sel){stats.dropped+=s.dropped;stats.unposable+=s.unposable;stats.frozen+=s.frozen;}
 // winding of the model's own triangles relative to their outward normals (majority over the published skin)
 long long windingVote=0;
 for(const auto&src:children){if(src.material)continue;for(std::size_t t=0;t<src.indices.size()/3&&t<4096;++t){
   const auto&A=src.vertices[src.indices[3*t]];const auto&B=src.vertices[src.indices[3*t+1]];const auto&C=src.vertices[src.indices[3*t+2]];
   const Vec e1=position(B)-position(A),e2=position(C)-position(A);const Vec g{e1.y*e2.z-e1.z*e2.y,e1.z*e2.x-e1.x*e2.z,e1.x*e2.y-e1.y*e2.x};
   const Vec n{A.surface[3]+B.surface[3]+C.surface[3],A.surface[4]+B.surface[4]+C.surface[4],A.surface[5]+B.surface[5]+C.surface[5]};
   const double s=dot(g,n);windingVote+=s>0?1:(s<0?-1:0);}}
 const bool ccwOutward=windingVote>=0; // the box is wound like the model: (b-a)x(c-a) along the outward normal when true
 // ---- assemble
 std::vector<bound_surface::Piece> out(P);std::vector<std::uint32_t> empty;
 if(marksOut)marksOut->assign(P,{});
 for(std::size_t k=0;k<P;++k){auto&piece=out[k];piece.volume_component=components[k];auto&mesh=piece.mesh;
  unsigned next=originalChildren; // splitfix: appended slots start right after the original children in every piece; numbered above the source's own appended slots, a cut part's pieces left a gap the native packet check refuses (no re-split was ever staged)
  double bestD=1e300;std::uint16_t boxBone=65535,boxMaterialSource=0;
  const auto&box=fields[k].box;const Vec boxCentre{box.centre[0],box.centre[1],box.centre[2]};
  for(std::size_t c=0;c<children.size();++c){const auto&src=children[c];const auto&list=sel[c].tris.empty()?std::vector<std::uint32_t>():sel[c].tris[k];if(list.empty())continue;
   for(const auto&a:sel[c].anchors[k]){auto [entry,fresh]=piece.constrained_anchors.emplace(a);need(fresh||entry->second==a.second,"mask_piece_frozen_anchor_conflict");}
   gfy_packed::Child child;child.material=0;child.material_source_child=src.material_source_child;child.skin_clip=true;
   if(src.child_index<originalChildren)child.child_index=src.child_index;else{need(next<gfy_packed::max_child_slots,"mask_piece_child_slots");child.child_index=std::uint16_t(next++);}
   std::vector<std::int32_t> map(src.vertices.size(),-1);const bool faces=src.original_faces.size()*3==src.indices.size();
   for(const auto t:list){for(int j=0;j<3;++j){const auto v=src.indices[3*t+j];
     if(map[v]<0){map[v]=std::int32_t(child.vertices.size());auto row=src.vertices[v];child.vertices.push_back(row);
      if(box.valid){unsigned best=0;for(unsigned w=1;w<4;++w)if(row.weight[w]>row.weight[best])best=w;
       const double dd=length(position(row)-boxCentre);if(dd<bestD&&row.bone[best]<64&&ok[k][row.bone[best]]){bestD=dd;boxBone=row.bone[best];boxMaterialSource=src.material_source_child;}}}
     child.indices.push_back(std::uint16_t(map[v]));}
    child.original_faces.push_back(faces?src.original_faces[t]:~std::uint32_t(0));}
   need(child.vertices.size()<=65535,"mask_piece_vertex_budget");
   piece.triangles+=list.size();stats.selected+=list.size();mesh.children.push_back(std::move(child));}
  if(box.valid&&boxBone!=65535&&next<gfy_packed::max_child_slots){
   const auto corner=painter_mask_pieces::corners(box);const auto tri=painter_mask_pieces::triangles(box);
   gfy_packed::Child child;child.material=0;child.material_source_child=boxMaterialSource;child.skin_clip=true;child.child_index=std::uint16_t(next++);
   for(std::size_t f=0;f<12;++f){const std::uint16_t q[3]={tri[3*f],tri[3*f+1],tri[3*f+2]};
    const Vec A{corner[q[0]][0],corner[q[0]][1],corner[q[0]][2]},B{corner[q[1]][0],corner[q[1]][1],corner[q[1]][2]},C{corner[q[2]][0],corner[q[2]][1],corner[q[2]][2]};
    Vec n{(B-A).y*(C-A).z-(B-A).z*(C-A).y,(B-A).z*(C-A).x-(B-A).x*(C-A).z,(B-A).x*(C-A).y-(B-A).y*(C-A).x};const double ln=length(n);if(!(ln>1e-12))continue;n=n*(1./ln);
    const Vec centre{box.centre[0],box.centre[1],box.centre[2]};const bool outward=dot(n,A-centre)>0;
    const std::uint16_t order[3]={q[0],outward==ccwOutward?q[1]:q[2],outward==ccwOutward?q[2]:q[1]};
    const Vec no=outward?n:n*-1.;const Vec tg=std::abs(no.y)<.9?Vec{no.z,0,-no.x}:Vec{0,-no.z,no.y};const Vec tn=tg*(1./length(tg));
    const Vec bn{no.y*tn.z-no.z*tn.y,no.z*tn.x-no.x*tn.z,no.x*tn.y-no.y*tn.x};
    for(const auto qi:order){gfy_packed::Vertex row;
     const double*p=corner[qi].data();const double s[14]={p[0],p[1],p[2],no.x,no.y,no.z,tn.x,tn.y,tn.z,bn.x,bn.y,bn.z,-1000.,0.};
     for(unsigned a=0;a<14;++a)row.surface[a]=float(s[a]);
     row.bone={boxBone,boxBone,boxBone,boxBone};row.weight={1.f,0.f,0.f,0.f};gfy_packed::canonical_weights(row);
     child.indices.push_back(std::uint16_t(child.vertices.size()));child.vertices.push_back(row);}
    child.original_faces.push_back(~std::uint32_t(0));}
   stats.boxTriangles+=child.indices.size()/3;++stats.boxes;mesh.children.push_back(std::move(child));}
  if(!piece.triangles){empty.push_back(components[k]);continue;}
  struct FieldTiles {std::vector<std::pair<Cell,core::Tile>> tiles; /* a list: every key once (the marked map) */const void* ownership=nullptr;double pitch;Vec origin;decltype(grid.size) size;};
  FieldTiles field{{},nullptr,grid.pitch,grid.origin,grid.size};
  std::vector<std::pair<Cell,core::Tile>> marks;marks.reserve(fields[k].marked.size());
  for(const auto&t:fields[k].marked){core::Tile tile;tile.removed=t.second;const Cell key{t.first.x,t.first.y,t.first.z};marks.emplace_back(key,tile);}
  field.tiles=marks;
  auto removal=painter_detail::buildDamageField(field);need(removal&&removal->available,"mask_piece_removal_field");
  stats.fieldBytes+=removal->bytes();
  mesh.removal_field=std::move(removal);mesh.tissue_field=published.tissue_field;mesh.mask_volume=true;mesh.mask_piece=true;
  if(marksOut)(*marksOut)[k]=std::move(marks);}
 if(!empty.empty())throw lowpoly_pieces::EmptyPieces(std::move(empty));
 stats.packMs=core::ms(p0);stats.ms=core::ms(t0);
 return out;
}
} // namespace mask_pieces
