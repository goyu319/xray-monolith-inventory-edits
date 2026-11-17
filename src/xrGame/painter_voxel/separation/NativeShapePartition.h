#pragma once
// Immutable original-rig collision regions. This labels physics ELEMENTS only;
// exact remaining anatomical connectivity still decides which PIECE owns cells.
// Adaptive z spans use a Lipschitz bound, not an unchecked midpoint shortcut.
#include <atomic>
#include <memory>
#include "PhysicalElementPartition.h"
#include "OriginalBoneFrames.h"
#include "ThreadBudget.h"
namespace gfy_piece_collider { namespace native_shape {
struct Shape {
 std::uint16_t bone{},element{},type{};Matrix inverse;
 V center{},extent{},direction{};std::array<V,3>axes{};
 double along_z_lipschitz{};
 double score(V p)const{
  const auto q=sub(transform(inverse,p),center);
  if(type==1){V d{{std::abs(dot(q,axes[0]))-extent[0],std::abs(dot(q,axes[1]))-extent[1],std::abs(dot(q,axes[2]))-extent[2]}};
   // max(max(a,b),c) == std::max({a,b,c}) (exact); avoids an initializer-list temporary.
   return norm({{std::max(0.,d[0]),std::max(0.,d[1]),std::max(0.,d[2])}})+std::min(0.,std::max(std::max(d[0],d[1]),d[2]));}
  if(type==2)return norm(q)-extent[0];
  const auto axial=dot(q,direction);const auto radial=norm(sub(q,mul(direction,axial)));
  const auto a=std::abs(axial)-extent[0],r=radial-extent[1];
  return std::hypot(std::max(0.,a),std::max(0.,r))+std::min(0.,std::max(a,r));
 }
};
struct Catalog {std::vector<Shape>shapes;std::array<double,64>lipschitz{};std::set<std::uint16_t>elements;};
inline Catalog decode(const std::vector<std::uint8_t>&bytes,const original_frames::Frames&frames){
 require(bytes.size()>=80&&std::memcmp(bytes.data(),"PHYSRAW1",8)==0,"native_shape_header");
 require(std::memcmp(bytes.data()+8,frames.source_sha256.data(),32)==0&&std::memcmp(bytes.data()+40,frames.fit_sha256.data(),32)==0,"native_shape_identity");
 std::uint32_t bones=0,count=0;std::memcpy(&bones,bytes.data()+72,4);std::memcpy(&count,bytes.data()+76,4);
 require(bones==frames.bones.size()&&bones<=64&&count>0&&count<=bones&&bytes.size()==80ull+116ull*count,"native_shape_size");
 Catalog out;
 for(std::size_t n=0;n<count;++n){const auto*raw=bytes.data()+80+n*116;Shape shape;std::uint16_t flags=0;
  std::memcpy(&shape.bone,raw,2);std::memcpy(&shape.element,raw+2,2);std::memcpy(&shape.type,raw+4,2);std::memcpy(&flags,raw+6,2);
  require(shape.bone<bones&&shape.element<bones&&frames.bones[shape.bone].physical_element==shape.element&&shape.type>=1&&shape.type<=3&&!(flags&4),"native_shape_bone");
  shape.inverse=frames.bones[shape.bone].rest_to_anchor;
  auto value=[&](unsigned at){float f=0;std::memcpy(&f,raw+4+at,4);require(std::isfinite(f),"native_shape_nonfinite");return double(f);};
  auto vector=[&](unsigned at){return V{{value(at),value(at+4),value(at+8)}};};
  const auto z=rotate(shape.inverse,{{0,0,1}});
  if(shape.type==1){
   for(unsigned a=0;a<3;++a)shape.axes[a]=vector(4+a*12);
   shape.center=vector(40);shape.extent=vector(52);
   for(unsigned a=0;a<3;++a){require(shape.extent[a]>.000001&&std::abs(norm(shape.axes[a])-1)<1e-4,"native_box_frame");
    for(unsigned b=0;b<a;++b)require(std::abs(dot(shape.axes[a],shape.axes[b]))<1e-4,"native_box_orthogonality");
   }
  }else if(shape.type==2){shape.center=vector(64);shape.extent[0]=value(76);require(shape.extent[0]>.000001,"native_sphere_radius");}
  else{shape.center=vector(80);shape.direction=vector(92);require(norm(shape.direction)>.000001,"native_cylinder_axis");shape.direction=mul(shape.direction,1./norm(shape.direction));
   shape.extent={value(104)*.5,value(108),0};require(shape.extent[0]>.000001&&shape.extent[1]>.000001,"native_cylinder_extent");
  }
  // Signed Euclidean distance is comparable across differently-sized shapes.
  // A normalized radius score incorrectly lets a distant calf claim a hand.
  // All three distance functions are 1-Lipschitz in rigid local coordinates.
  shape.along_z_lipschitz=norm(z)*1.0002;
  out.lipschitz[shape.element]=std::max(out.lipschitz[shape.element],shape.along_z_lipschitz);out.elements.insert(shape.element);out.shapes.push_back(shape);
 }
 return out;
}
inline std::uint16_t classify(const Catalog&catalog,V p,std::array<double,64>&scores){
 scores.fill(std::numeric_limits<double>::infinity());
 for(const auto&s:catalog.shapes)scores[s.element]=std::min(scores[s.element],s.score(p));
 std::uint16_t winner=65535;double best=std::numeric_limits<double>::infinity();
 for(auto element:catalog.elements)if(scores[element]<best){best=scores[element];winner=element;}
 require(winner!=65535,"native_shape_no_owner");return winner;
}
// The int14t algorithm, kept verbatim as the reference of build() (P1.9).
inline element_partition::Map buildReference(const Catalog&catalog,const gfy_connectivity::Topology&original,
                                    const gfy_live::Hash&source,const gfy_live::Hash&volume){
 const auto&a=original;
 element_partition::Map out;out.source_sha256=source;out.volume_sha256=volume;
 for(unsigned axis=0;axis<3;++axis){require(a.size[axis]>0&&a.size[axis]<=65535,"native_shape_extent");out.size[axis]=std::uint32_t(a.size[axis]);}
 out.origin=a.origin;out.pitch=a.pitch;
 require(std::isfinite(a.pitch)&&a.pitch>=.0005&&a.pitch<=.01,"native_shape_pitch");
 const auto columns=std::uint64_t(a.size[0])*a.size[1];require(columns<=30000000,"native_shape_column_budget");out.offsets.resize(std::size_t(columns)+1);
 // Columns are independent: contiguous column ranges run on up to four
 // threads (gfy_live::core::parallelChunks), each into its own run list and
 // cell counts; they are concatenated in column order. Runs, offsets and
 // counts equal the serial loop; the work/run budgets are checked on totals
 // (same messages; only the point of failure inside the loop differs).
 struct Part{std::vector<element_partition::Run>runs;std::vector<std::uint32_t>perColumn;std::map<std::uint16_t,std::uint64_t>cells;std::uint64_t work{};std::size_t sparse{};};
 // Blocks of 4096 columns pulled dynamically (column cost is very uneven).
 constexpr std::size_t blockColumns=4096;const std::size_t blockCount=(std::size_t(columns)+blockColumns-1)/blockColumns;
 std::vector<Part>parts(blockCount);std::vector<std::size_t>bounds(blockCount+1);
 for(std::size_t b=0;b<=blockCount;++b)bounds[b]=std::min<std::size_t>(std::size_t(columns),b*blockColumns);
 std::atomic<std::size_t>cursor{0};
 gfy_live::core::parallelChunks(blockCount>=2?4:1,1,[&](std::size_t,std::size_t,std::size_t){for(std::size_t chunk;(chunk=cursor.fetch_add(1))<blockCount;){
 const std::size_t first=bounds[chunk],last=bounds[chunk+1];
 auto&part=parts[chunk];part.perColumn.assign(last-first,0);
 std::uint64_t work=0;
 std::size_t sparseColumn=std::size_t(std::lower_bound(a.columns.begin(),a.columns.end(),first,[&](const auto&c,std::size_t value){return std::uint64_t(c.y)*a.size[0]+c.x<value;})-a.columns.begin());
 const std::size_t sparseFirst=sparseColumn;
 for(std::uint32_t column=std::uint32_t(first);column<last;++column){const std::size_t before=part.runs.size();
  const V position{{a.origin[0]+(column%a.size[0]+.5)*a.pitch,a.origin[1]+(column/a.size[0]+.5)*a.pitch,0}};
  auto append=[&](int lo,int hi,std::uint16_t element){
   if(part.runs.size()>before&&part.runs.back().hi==lo&&part.runs.back().element==element)part.runs.back().hi=std::uint16_t(hi);
   else{require(part.runs.size()<64000000,"native_shape_span_budget");part.runs.push_back({column,std::uint16_t(lo),std::uint16_t(hi),element});}
   part.cells[element]+=std::uint64_t(hi-lo);
  };
  auto divide=[&](auto&&self,int lo,int hi)->void{
   require(++work<400000000,"native_shape_work_budget");const int middle=lo+(hi-lo-1)/2;auto p=position;p[2]=a.origin[2]+(middle+.5)*a.pitch;
   std::array<double,64>scores;const auto winner=classify(catalog,p,scores);
   const auto reach=std::max(middle-lo,hi-1-middle)*a.pitch;bool uniform=true;
   for(auto other:catalog.elements)if(other!=winner&&scores[other]-scores[winner]<=(catalog.lipschitz[other]+catalog.lipschitz[winner])*reach+1e-12){uniform=false;break;}
   if(uniform||hi-lo==1){append(lo,hi,winner);return;}
   const int split=lo+(hi-lo)/2;self(self,lo,split);self(self,split,hi);
  };
  // Labels change at organ boundaries; merge occupied runs before classifying
  // physics so the number of organs cannot multiply this workload.
  int lo=-1,hi=-1;
  if(sparseColumn<a.columns.size()&&std::uint64_t(a.columns[sparseColumn].y)*a.size[0]+a.columns[sparseColumn].x==column){
   const auto&occupied=a.columns[sparseColumn++];
   for(auto r=occupied.first;r<occupied.first+occupied.count;++r){const auto&run=a.spans[r];
    if(lo>=0&&run.lo==hi){hi=run.hi;continue;}
    if(lo>=0)divide(divide,lo,hi);lo=run.lo;hi=run.hi;}
   if(lo>=0)divide(divide,lo,hi);}
  part.perColumn[column-first]=std::uint32_t(part.runs.size()-before);
 }
 part.work=work;part.sparse=sparseColumn-sparseFirst;}});
 std::uint64_t work=0;std::size_t sparseColumn=0,total=0;for(const auto&part:parts){work+=part.work;sparseColumn+=part.sparse;total+=part.runs.size();}
 require(work<400000000,"native_shape_work_budget");require(total<=64000000,"native_shape_span_budget");
 out.runs.reserve(total);
 for(std::size_t c=0;c<blockCount;++c){const auto&part=parts[c];std::size_t at=0;
  for(std::size_t i=0;i<part.perColumn.size();++i){out.offsets[bounds[c]+i]=std::uint32_t(out.runs.size()+at);at+=part.perColumn[i];}
  out.runs.insert(out.runs.end(),part.runs.begin(),part.runs.end());
  for(const auto&e:part.cells)out.original_cells[e.first]+=e.second;}
 out.offsets.back()=std::uint32_t(out.runs.size());
 std::uint64_t cells=0;for(const auto&entry:out.original_cells)cells+=entry.second;
 require(sparseColumn==a.columns.size()&&cells==a.cells,"native_shape_cell_conservation");return out;
}
// Rigorous 3D Lipschitz bound of a shape's score (used by the exact build's bounds).
namespace exact {
inline double spectralBound(const std::array<V,3>&rows){ // >= largest singular value (Gershgorin on X^T X)
 double b=0;
 for(int i=0;i<3;++i){double s=0;for(int j=0;j<3;++j)s+=std::abs(rows[0][i]*rows[0][j]+rows[1][i]*rows[1][j]+rows[2][i]*rows[2][j]);b=std::max(b,s);}
 return std::sqrt(b);
}
// Every shape score is 1-Lipschitz in its local coordinates (box, sphere and
// capsule distance functions); local coordinates are the linear part of
// rest_to_anchor applied to p (times the box axes for boxes).
inline double spatialLipschitz(const Shape&s){
 const auto&a=s.inverse.row_major;
 const std::array<V,3>linear{{{{a[0],a[1],a[2]}},{{a[4],a[5],a[6]}},{{a[8],a[9],a[10]}}}};
 double l=spectralBound(linear);
 if(s.type==1)l*=spectralBound(s.axes);
 return l*1.000001;
}
}
// ---- Exact accelerated reference (separation-track P1.9, the default build) --
// ExactMapBuild runs buildReference()'s recursion node for node (same merged
// occupied spans, same midpoints, reaches, splits and work count) and makes the
// same decisions; it only avoids evaluating shape scores whose comparison
// outcome is already PROVEN by rigorous bounds.
//  * Bounds: every lattice block of 16^3 cells gets every element's exact score
//    S_e at its centre pc (classify's per-element evaluation, cached per band).
//    For a midpoint p at distance d=|p-pc| the true score lies in S_e -+ L3_e*d,
//    where L3_e is a rigorous 3D Lipschitz bound of the element's score (a
//    spectral bound of each shape's linear map, times the box-axes bound,
//    *1.000001: exact::spatialLipschitz). Floating-point evaluation of any
//    score is within ~1e-13 m of the true value for |S|,|p| < 100 m (checked;
//    otherwise the node evaluates every score as the reference does), and every
//    bound decision keeps a 1e-9 m margin on top.
//  * Winner: elements whose lower bound exceeds the smallest upper bound by
//    more than the margin can neither win nor tie; the rest are evaluated
//    exactly (classify's per-element min over the element's shapes in catalog
//    order) and the winner is the first ascending id with the smallest score,
//    as in classify().
//  * Uniform test: for each other element the reference's own comparison
//    "scores[o]-scores[w] <= (L_o+L_w)*reach+1e-12" (same fp expression for the
//    threshold) is decided by the bounds when they clear it by the margin in
//    either direction, and otherwise by the exact scores. The evaluation order
//    does not matter: the result is the conjunction over all elements.
// Hence runs, offsets, cell counts and the work total equal buildReference().
// Failure paths are the reference's own: a total work >= 400M, more than 64M
// runs, a node without a finite owner or any exception inside a band makes
// finish() run buildReference() itself, which throws (or not) exactly as before.
// Bands (16 grid rows each) may run on any threads in any order (the separation
// cache's joinable task); the build owns the catalog and topology it reads.
namespace exact {
constexpr int block=16;
constexpr double margin=1e-9,sane=100;
struct Element {std::uint16_t id{};double lz{},l3{};std::vector<const Shape*>shapes;};
// classify()'s score of one element: min over its shapes in catalog order.
inline double score(const Element&e,const V&p){double v=std::numeric_limits<double>::infinity();for(const auto*s:e.shapes)v=std::min(v,s->score(p));return v;}
struct Stats {std::uint64_t nodes{},exactScores{},fullNodes{},blocks{},fastNodes{};};
}
struct ExactMapBuild {
 std::shared_ptr<const Catalog>catalog;std::shared_ptr<const gfy_connectivity::Topology>original;gfy_live::Hash source{},volume{};
 std::vector<exact::Element>elements;double l3max{},lzmax{};
 std::size_t sx{},bandCount{},nbx{},nbz{};std::uint64_t columns{};
 struct Part{std::vector<element_partition::Run>runs;std::vector<std::uint32_t>perColumn;std::map<std::uint16_t,std::uint64_t>cells;std::uint64_t work{};std::size_t sparse{};exact::Stats stats;};
 std::vector<Part>parts;std::vector<std::size_t>bounds;
 std::atomic<bool>failed{false};
 ExactMapBuild(std::shared_ptr<const Catalog>c,std::shared_ptr<const gfy_connectivity::Topology>t,const gfy_live::Hash&s,const gfy_live::Hash&v)
  :catalog(std::move(c)),original(std::move(t)),source(s),volume(v){
  const auto&a=*original;
  // The reference's preconditions, in its order and with its messages.
  for(unsigned axis=0;axis<3;++axis)require(a.size[axis]>0&&a.size[axis]<=65535,"native_shape_extent");
  require(std::isfinite(a.pitch)&&a.pitch>=.0005&&a.pitch<=.01,"native_shape_pitch");
  columns=std::uint64_t(a.size[0])*a.size[1];require(columns<=30000000,"native_shape_column_budget");
  std::array<int,64>index;index.fill(-1);
  for(auto e:catalog->elements){require(e<64,"native_shape_bone");index[e]=int(elements.size());exact::Element el;el.id=e;el.lz=catalog->lipschitz[e];elements.push_back(el);}
  for(const auto&shape:catalog->shapes){auto&el=elements[std::size_t(index[shape.element])];el.shapes.push_back(&shape);
   el.l3=std::max(el.l3,exact::spatialLipschitz(shape));}
  for(const auto&el:elements){l3max=std::max(l3max,el.l3);lzmax=std::max(lzmax,el.lz);}
  sx=std::size_t(a.size[0]);bandCount=(std::size_t(a.size[1])+exact::block-1)/exact::block;
  nbx=(sx+exact::block-1)/exact::block;nbz=(std::size_t(a.size[2])+exact::block-1)/exact::block;
  parts.resize(bandCount);bounds.resize(bandCount+1);
  for(std::size_t b=0;b<=bandCount;++b)bounds[b]=std::min<std::size_t>(std::size_t(columns),b*exact::block*sx);
 }
 void band(std::size_t band){try{run(band);}catch(...){failed=true;}}
 void run(std::size_t band){
  using exact::block;const auto&a=*original;const auto&cat=*catalog;const auto&E=elements;const std::size_t ne=E.size();
  std::vector<double>blockScores(nbx*nbz*ne);std::vector<std::uint8_t>blockState(nbx*nbz); // 0 unknown, 1 bounds usable, 2 reference nodes
  std::vector<std::uint8_t>blockOrder(nbx*nbz*ne); // per block: element indices by ascending S (ties by index)
  // Per block, when one element wins at every cell of the block: its index, and block-wide
  // bounds G <= min(s_e-s_w) and H >= min(s_e-s_w) (element hElem) over the block's cells.
  std::vector<std::uint8_t>blockWinner(nbx*nbz),blockHElem(nbx*nbz);std::vector<double>blockG(nbx*nbz),blockH(nbx*nbz);
  const double radius=7.5*std::sqrt(3.)*a.pitch*(1+1e-9); // cell centres of a block lie within this of its centre
  std::vector<double>lb(ne),ub(ne),exactScore(ne);std::vector<std::uint8_t>known(ne);
  std::array<double,64>fullScores;
  const std::size_t first=bounds[band],last=bounds[band+1];
  auto&part=parts[band];part.perColumn.assign(last-first,0);
  std::uint64_t work=0;
  const int y0=int(band)*block,y1=std::min(int(a.size[1]),y0+block);
  auto columnLess=[](const gfy_connectivity::Column&c,int y){return c.y<y;};
  const auto cBegin=std::size_t(std::lower_bound(a.columns.begin(),a.columns.end(),y0,columnLess)-a.columns.begin());
  const auto cEnd=std::size_t(std::lower_bound(a.columns.begin(),a.columns.end(),y1,columnLess)-a.columns.begin());
  const double pcy=a.origin[1]+(double(band)*block+block*.5)*a.pitch;
  for(auto c=cBegin;c<cEnd;++c){const auto&occupied=a.columns[c];
   const std::uint32_t column=std::uint32_t(std::uint64_t(occupied.y)*sx+std::uint64_t(occupied.x));const std::size_t before=part.runs.size();
   const V position{{a.origin[0]+(column%a.size[0]+.5)*a.pitch,a.origin[1]+(column/a.size[0]+.5)*a.pitch,0}};
   const std::size_t bx=std::size_t(occupied.x)/block;
   const double pcx=a.origin[0]+(double(bx)*block+block*.5)*a.pitch;
   auto append=[&](int lo,int hi,std::uint16_t element){
    if(part.runs.size()>before&&part.runs.back().hi==lo&&part.runs.back().element==element)part.runs.back().hi=std::uint16_t(hi);
    else{if(part.runs.size()>=64000000)throw std::runtime_error("native_shape_span_budget");part.runs.push_back({column,std::uint16_t(lo),std::uint16_t(hi),element});}
    part.cells[element]+=std::uint64_t(hi-lo);
   };
   // The reference's own node (every score evaluated): classify + its uniform loop.
   auto referenceNode=[&](const V&p,double reach,bool&uniform)->std::uint16_t{
    ++part.stats.fullNodes;const auto winner=classify(cat,p,fullScores);uniform=true;
    for(auto other:cat.elements)if(other!=winner&&fullScores[other]-fullScores[winner]<=(cat.lipschitz[other]+cat.lipschitz[winner])*reach+1e-12){uniform=false;break;}
    return winner;};
   auto divide=[&](auto&&self,int lo,int hi)->void{
    // A band past either reference budget stops early (the reference would
    // throw; finish() then runs it, so the failure is its own).
    if(++work>=400000000)throw std::runtime_error("native_shape_work_budget");const int middle=lo+(hi-lo-1)/2;auto p=position;p[2]=a.origin[2]+(middle+.5)*a.pitch;
    const auto reach=std::max(middle-lo,hi-1-middle)*a.pitch;
    const std::size_t bz=std::size_t(middle)/block,slot=bx*nbz+bz;
    double*S=blockScores.data()+slot*ne;
    const double pcz=a.origin[2]+(double(bz)*block+block*.5)*a.pitch;
    if(!blockState[slot]){++part.stats.blocks;
     const V pc{{pcx,pcy,pcz}};bool usable=std::abs(pc[0])<exact::sane&&std::abs(pc[1])<exact::sane&&std::abs(pc[2])<exact::sane;
     for(std::size_t i=0;i<ne;++i){S[i]=exact::score(E[i],pc);usable=usable&&std::isfinite(S[i])&&std::abs(S[i])<exact::sane&&std::isfinite(E[i].l3);}
     std::uint8_t*order=blockOrder.data()+slot*ne;for(std::size_t i=0;i<ne;++i)order[i]=std::uint8_t(i);
     if(usable)std::sort(order,order+ne,[&](std::uint8_t x,std::uint8_t y){return S[x]!=S[y]?S[x]<S[y]:x<y;});
     blockWinner[slot]=255;
     if(usable){double ubmin=std::numeric_limits<double>::infinity();
      for(std::size_t i=0;i<ne;++i)ubmin=std::min(ubmin,S[i]+E[i].l3*radius);
      std::size_t candidates=0,w=0;for(std::size_t i=0;i<ne;++i)if(S[i]-E[i].l3*radius<=ubmin+exact::margin){++candidates;w=i;}
      if(candidates==1){double g=std::numeric_limits<double>::infinity(),h=g;std::size_t he=w;
       const double lbw=S[w]-E[w].l3*radius,ubw=S[w]+E[w].l3*radius;
       for(std::size_t i=0;i<ne;++i)if(i!=w){g=std::min(g,S[i]-E[i].l3*radius-ubw);const double hi_=S[i]+E[i].l3*radius-lbw;if(hi_<h){h=hi_;he=i;}}
       blockWinner[slot]=std::uint8_t(w);blockG[slot]=g;blockH[slot]=h;blockHElem[slot]=std::uint8_t(he);}}
     blockState[slot]=usable?1:2;}
    std::uint16_t winner=65535;bool uniform=true;
    const bool bounded=blockState[slot]==1&&std::abs(p[0])<exact::sane&&std::abs(p[1])<exact::sane&&std::abs(p[2])<exact::sane;
    // Fast path: the block has a single winner at every cell, and the uniform
    // test is settled by the block-wide gap bounds (or the node is one cell).
    bool settled=false;
    if(bounded&&blockWinner[slot]!=255){const std::size_t w=blockWinner[slot];
     if(hi-lo==1)settled=true;
     else if(blockG[slot]>(lzmax+E[w].lz)*reach+1e-12+exact::margin)settled=true;
     else if(blockH[slot]<(E[blockHElem[slot]].lz+E[w].lz)*reach+1e-12-exact::margin){settled=true;uniform=false;}
     if(settled){winner=E[w].id;++part.stats.fastNodes;}}
    if(settled){}
    else if(!bounded)winner=referenceNode(p,reach,uniform);
    else{
     // Bounds, visited in ascending S order: an element whose S exceeds the
     // running bound by the largest Lipschitz term can neither be a candidate
     // nor fail the uniform test, and neither can any element after it.
     const std::uint8_t*order=blockOrder.data()+slot*ne;
     const double dx=p[0]-pcx,dy=p[1]-pcy,dz=p[2]-pcz,d=std::sqrt(dx*dx+dy*dy+dz*dz),spread=l3max*d;
     double ubmin=std::numeric_limits<double>::infinity();std::size_t prefix=0;
     for(;prefix<ne;++prefix){const auto i=order[prefix];if(S[i]-spread>ubmin+exact::margin)break;
      lb[i]=S[i]-E[i].l3*d;ub[i]=S[i]+E[i].l3*d;ubmin=std::min(ubmin,ub[i]);known[i]=0;}
     for(std::size_t k=prefix;k<ne;++k)known[order[k]]=0;
     std::size_t candidates=0,w=order[0];
     for(std::size_t k=0;k<prefix;++k){const auto i=order[k];if(lb[i]<=ubmin+exact::margin){++candidates;w=i;}}
     bool full=false;
     if(candidates!=1){double best=std::numeric_limits<double>::infinity();bool found=false;
      // Ascending element index among the candidates: classify()'s tie order.
      for(std::size_t i=0;i<ne;++i)if(known[i]==0&&std::find(order,order+prefix,std::uint8_t(i))!=order+prefix&&lb[i]<=ubmin+exact::margin){
       exactScore[i]=exact::score(E[i],p);known[i]=1;++part.stats.exactScores;
       if(!std::isfinite(exactScore[i]))full=true;
       if(exactScore[i]<best){best=exactScore[i];w=i;found=true;}}
      full=full||!found;}
     if(full)winner=referenceNode(p,reach,uniform);
     else{
      winner=E[w].id;
      if(hi-lo>1){ // a one-cell node is appended whatever the test says
       auto exactOf=[&](std::size_t i){if(!known[i]){exactScore[i]=exact::score(E[i],p);known[i]=1;++part.stats.exactScores;}return exactScore[i];};
       const double hi_w0=known[w]?exactScore[w]:S[w]+E[w].l3*d;
       const double stop=(lzmax+E[w].lz)*reach+1e-12+exact::margin; // >= every element's threshold+margin
       for(std::size_t k=0;k<ne;++k){const auto i=order[k];if(i==w)continue;
        if(S[i]-spread-hi_w0>stop)break; // this and every later element pass
        const double threshold=(E[i].lz+E[w].lz)*reach+1e-12;
        const double lo_i=known[i]?exactScore[i]:S[i]-E[i].l3*d,hi_i=known[i]?exactScore[i]:S[i]+E[i].l3*d;
        const double lo_w=known[w]?exactScore[w]:S[w]-E[w].l3*d,hi_w=known[w]?exactScore[w]:S[w]+E[w].l3*d;
        if(lo_i-hi_w>threshold+exact::margin)continue;
        if(hi_i-lo_w<threshold-exact::margin){uniform=false;break;}
        const double si=exactOf(i),sw=exactOf(w);
        if(!std::isfinite(si)||!std::isfinite(sw)){winner=referenceNode(p,reach,uniform);break;} // not expected in a bounded block
        if(si-sw<=threshold){uniform=false;break;}}
      }
     }
    }
    if(uniform||hi-lo==1){append(lo,hi,winner);return;}
    const int split=lo+(hi-lo)/2;self(self,lo,split);self(self,split,hi);
   };
   // Labels change at organ boundaries; merge occupied runs before classifying
   // physics so the number of organs cannot multiply this workload.
   int lo=-1,hi=-1;
   for(auto r=occupied.first;r<occupied.first+occupied.count;++r){const auto&run=a.spans[r];
    if(lo>=0&&run.lo==hi){hi=run.hi;continue;}
    if(lo>=0)divide(divide,lo,hi);lo=run.lo;hi=run.hi;}
   if(lo>=0)divide(divide,lo,hi);
   part.perColumn[column-first]=std::uint32_t(part.runs.size()-before);
  }
  part.work=work;part.sparse=cEnd-cBegin;part.stats.nodes=work;
 }
 exact::Stats stats()const{exact::Stats all;for(const auto&p:parts){all.nodes+=p.stats.nodes;all.exactScores+=p.stats.exactScores;all.fullNodes+=p.stats.fullNodes;all.blocks+=p.stats.blocks;all.fastNodes+=p.stats.fastNodes;}return all;}
 element_partition::Map finish(){
  const auto&a=*original;
  std::uint64_t work=0;std::size_t sparseColumn=0,total=0;
  for(const auto&part:parts){work+=part.work;sparseColumn+=part.sparse;total+=part.runs.size();}
  if(failed||work>=400000000||total>64000000)return buildReference(*catalog,a,source,volume);
  element_partition::Map out;out.source_sha256=source;out.volume_sha256=volume;
  for(unsigned axis=0;axis<3;++axis)out.size[axis]=std::uint32_t(a.size[axis]);
  out.origin=a.origin;out.pitch=a.pitch;out.offsets.resize(std::size_t(columns)+1);
  out.runs.reserve(total);
  for(std::size_t c=0;c<bandCount;++c){auto&part=parts[c];std::size_t at=0;
   for(std::size_t i=0;i<part.perColumn.size();++i){out.offsets[bounds[c]+i]=std::uint32_t(out.runs.size()+at);at+=part.perColumn[i];}
   out.runs.insert(out.runs.end(),part.runs.begin(),part.runs.end());
   for(const auto&e:part.cells)out.original_cells[e.first]+=e.second;
   std::vector<element_partition::Run>().swap(part.runs);std::vector<std::uint32_t>().swap(part.perColumn);}
  out.offsets.back()=std::uint32_t(out.runs.size());
  std::uint64_t cells=0;for(const auto&entry:out.original_cells)cells+=entry.second;
  require(sparseColumn==a.columns.size()&&cells==a.cells,"native_shape_cell_conservation");
#ifdef GFY_SEP_VERIFY
  {const auto reference=buildReference(*catalog,a,source,volume);
   bool same=reference.runs.size()==out.runs.size()&&reference.offsets==out.offsets&&reference.original_cells==out.original_cells;
   for(std::size_t i=0;same&&i<out.runs.size();++i)same=std::memcmp(&reference.runs[i],&out.runs[i],sizeof(out.runs[i]))==0;
   require(same,"native_shape_exact_map_mismatch");}
#endif
  return out;
 }
};
// Solo form: the bands on <= 4 threads, then finish(). stats (optional) reports
// the accelerated work (nodes = the reference's work units).
inline element_partition::Map buildExact(std::shared_ptr<const Catalog>catalog,std::shared_ptr<const gfy_connectivity::Topology>original,
                                         const gfy_live::Hash&source,const gfy_live::Hash&volume,exact::Stats*stats=nullptr){
 ExactMapBuild job(std::move(catalog),std::move(original),source,volume);std::atomic<std::size_t>cursor{0};
 gfy_live::sep::parallel(job.bandCount>=2?4:1,1,4,[&](std::size_t,std::size_t,std::size_t){
  for(std::size_t b;(b=cursor.fetch_add(1))<job.bandCount;)job.band(b);});
 if(stats)*stats=job.stats();
 return job.finish();
}
// The separation physics map: buildReference()'s result (or failure), computed by
// the exact accelerated recursion. Copies the catalog and the topology it reads.
inline element_partition::Map build(const Catalog&catalog,const gfy_connectivity::Topology&original,
                                    const gfy_live::Hash&source,const gfy_live::Hash&volume){
 return buildExact(std::make_shared<const Catalog>(catalog),std::make_shared<const gfy_connectivity::Topology>(original),source,volume);
}
}}
