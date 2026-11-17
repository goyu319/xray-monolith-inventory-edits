#pragma once
// Exact positive-area support against adjacent surviving voxel faces.
// No ray search, nearest-body association or change to render triangulation.
namespace gfy_live { namespace surface_support {
struct Budget {std::uint64_t visited{},limit{8000000};};
// Exterior contact is evaluated at the actual surface, against surviving spans
// of ORIGINAL anatomy. It does not search along normals or cross an air gap.
// This only certifies contact/owner candidates, never a garment attachment.
std::map<std::uint32_t,double> exterior(const core::JoinedTriangle&t,const Grid&grid,
                                       const gfy_connectivity::Topology&volume,Budget&budget){
 need(!t.material,"exterior_support_requires_source");
 need(volume.pitch==grid.pitch,"exterior_support_lattice");
 for(int a=0;a<3;++a)need(volume.origin[a]==grid.origin[a]&&volume.size[a]==grid.size[a],"exterior_support_domain");
 core::Poly polygon;core::Bounds bounds;
 for(const auto&v:t.v){for(int a=0;a<3;++a)need(std::isfinite(v.p[a]),"exterior_support_nonfinite");
  core::V point;point.p=v.p;polygon.push_back(point);bounds.add(v.p);}
 int lo[2],hi[2];
 for(int a=0;a<2;++a){
  const auto lower=(bounds.lo[a]-grid.origin[a])/grid.pitch;
  const auto upper=(bounds.hi[a]-grid.origin[a])/grid.pitch;
  if(upper<0||lower>grid.size[a])return {};
  lo[a]=int(std::max(0.,std::floor(std::nextafter(lower,-std::numeric_limits<double>::infinity()))));
  hi[a]=int(std::min(double(grid.size[a]-1),std::floor(upper)));
 }
 std::map<std::uint32_t,double> support;
 for(int y=lo[1];y<=hi[1];++y){
  need(++budget.visited<=budget.limit,"exterior_support_budget");
  auto row=core::clip(polygon,1,grid.origin[1]+y*grid.pitch,1);
  row=core::clip(row,1,grid.origin[1]+(y+1)*grid.pitch,-1);if(row.empty())continue;
  auto column=std::lower_bound(volume.columns.begin(),volume.columns.end(),std::array<int,2>{{lo[0],y}},
   [](const gfy_connectivity::Column&c,const std::array<int,2>&p){return gfy_connectivity::xyLess(c.x,c.y,p[0],p[1]);});
  for(;column!=volume.columns.end()&&column->y==y&&column->x<=hi[0];++column){
   need(++budget.visited<=budget.limit,"exterior_support_budget");
   auto slab=core::clip(row,0,grid.origin[0]+column->x*grid.pitch,1);
   slab=core::clip(slab,0,grid.origin[0]+(column->x+1)*grid.pitch,-1);if(slab.empty())continue;
   for(auto index=column->first;index<column->first+column->count;++index){
    need(++budget.visited<=budget.limit,"exterior_support_budget");
    const auto&span=volume.spans[index];need(span.component&&span.component<volume.components.size(),"exterior_support_component");
    auto cut=core::clip(slab,2,grid.origin[2]+span.lo*grid.pitch,1);
    cut=core::clip(cut,2,grid.origin[2]+span.hi*grid.pitch,-1);
    if(!cut.empty()){double area=core::area(cut);if(area>1e-16)support[span.component]+=area;}
   }
  }
 }
 return support;
}
std::map<std::uint32_t,double> cavity(const core::JoinedTriangle&t,const Grid&grid,
                                     const gfy_connectivity::Topology&volume,Budget&budget){
 need(t.material!=0,"support_requires_cavity");
 Vec normal=t.v[0].normal+t.v[1].normal+t.v[2].normal;
 int axis=0;for(int a=1;a<3;++a)if(std::abs(normal[a])>std::abs(normal[axis]))axis=a;
 need(length(normal)>1e-10,"support_missing_authored_normal");
 for(int a=0;a<3;++a)if(a!=axis)need(std::abs(normal[a])<1e-6*std::abs(normal[axis]),"support_non_axial_cavity");
 double position=0;for(const auto&v:t.v)position+=v.p[axis]/3;
 const int plane=int(std::llround((position-grid.origin[axis])/grid.pitch));
 const double exact=grid.origin[axis]+plane*grid.pitch;
 core::Poly polygon;core::Bounds bounds;
 for(const auto&v:t.v){need(std::abs(v.p[axis]-exact)<=1e-6,"support_off_lattice_plane");
  core::V p;p.p=v.p;p.p[axis]=exact;polygon.push_back(p);bounds.add(p.p);}
 const int u=(axis+1)%3,v=(axis+2)%3;
 int lo[2],hi[2];
 for(int i=0;i<2;++i){int a=i?v:u;
  lo[i]=std::max(0,int(std::floor((bounds.lo[a]-grid.origin[a])/grid.pitch)));
  hi[i]=std::min(grid.size[a]-1,int(std::floor((bounds.hi[a]-grid.origin[a])/grid.pitch)));}
 std::map<std::uint32_t,double> supported;
 for(int i=lo[0];i<=hi[0];++i)for(int j=lo[1];j<=hi[1];++j){
  need(++budget.visited<=budget.limit,"support_face_budget");
  // cavityPolygons reverses the removed-cell quad winding: rendered normals
  // point into the wound, so surviving tissue is on the opposite side.
  Cell n;(&n.x)[axis]=plane-(normal[axis]>0?1:0);(&n.x)[u]=i;(&n.x)[v]=j;
  const auto owner=volume.at(n.x,n.y,n.z);if(!owner)continue;
  need(grid.solidBase(n)&&!grid.removed(n),"support_topology_material_mismatch");
  auto clipped=core::clip(polygon,u,grid.origin[u]+i*grid.pitch,1);
  clipped=core::clip(clipped,u,grid.origin[u]+(i+1)*grid.pitch,-1);
  clipped=core::clip(clipped,v,grid.origin[v]+j*grid.pitch,1);
  clipped=core::clip(clipped,v,grid.origin[v]+(j+1)*grid.pitch,-1);
  if(clipped.empty())continue;
  const double area=core::area(clipped);if(area>1e-16)supported[owner]+=area;
 }
 return supported;
}
// ---- Separation-track P2 (S2/S3): exact lookup accelerators ---------------
// Row index over Topology::columns (sorted by (y,x)). lower(x,y) returns
// exactly std::lower_bound(columns,{x,y},xyLess); column() and at() return
// exactly Topology::column() and Topology::at(). Only the global binary
// search is replaced, by a search inside row y (rows outside [0,size.y) use
// the global search).
struct ColumnRows {
 const gfy_connectivity::Topology*topology{};std::vector<std::uint32_t> start;int rows{};
 explicit ColumnRows(const gfy_connectivity::Topology&t):topology(&t),rows(std::max(0,t.size[1])){
  const auto&c=t.columns;start.resize(std::size_t(rows)+1);
  for(int y=0;y<=rows;++y)start[std::size_t(y)]=std::uint32_t(std::lower_bound(c.begin(),c.end(),y,[](const gfy_connectivity::Column&col,int row){return col.y<row;})-c.begin());
 }
 const gfy_connectivity::Column*end()const{return topology->columns.data()+topology->columns.size();}
 const gfy_connectivity::Column*lower(int x,int y)const{
  const auto&c=topology->columns;
  if(y<0||y>=rows)return c.data()+(std::lower_bound(c.begin(),c.end(),std::array<int,2>{{x,y}},
   [](const gfy_connectivity::Column&col,const std::array<int,2>&p){return gfy_connectivity::xyLess(col.x,col.y,p[0],p[1]);})-c.begin());
  const auto first=c.begin()+start[std::size_t(y)],last=c.begin()+start[std::size_t(y)+1];
  return c.data()+(std::lower_bound(first,last,x,[](const gfy_connectivity::Column&col,int at){return col.x<at;})-c.begin());
 }
 const gfy_connectivity::Column*column(int x,int y)const{const auto*i=lower(x,y);return i!=end()&&i->x==x&&i->y==y?i:nullptr;}
 // The span part of Topology::at for one column.
 std::uint32_t spanAt(const gfy_connectivity::Column&c,int z)const{
  auto first=topology->spans.begin()+c.first,last=first+c.count;
  auto i=std::upper_bound(first,last,z,[](int at,const gfy_connectivity::Span&s){return at<s.lo;});
  if(i==first)return 0;--i;return z<i->hi?i->component:0;
 }
 std::uint32_t at(int x,int y,int z)const{const auto*c=column(x,y);return c?spanAt(*c,z):0;}
};
// Existence of Grid::tiles keys as a bitset over the key bounding box:
// find(k) returns exactly (tiles.find(k)==end ? null : &tile). Without the
// bitset (key box above 2^27 tiles) every query hashes.
struct TileIndex {
 const Grid*grid{};std::array<std::int64_t,3> lo{},n{};std::vector<std::uint64_t> bits;bool dense{};
 explicit TileIndex(const Grid&g):grid(&g){
  if(g.tiles.empty())return;
  std::array<std::int64_t,3> a{{INT64_MAX,INT64_MAX,INT64_MAX}},b{{INT64_MIN,INT64_MIN,INT64_MIN}};
  for(const auto&t:g.tiles)for(int i=0;i<3;++i){a[std::size_t(i)]=std::min<std::int64_t>(a[std::size_t(i)],t.first[i]);b[std::size_t(i)]=std::max<std::int64_t>(b[std::size_t(i)],t.first[i]);}
  std::uint64_t count=1;
  for(std::size_t i=0;i<3;++i){n[i]=b[i]-a[i]+1;count*=std::uint64_t(n[i]);if(count>(std::uint64_t(1)<<27))return;}
  lo=a;bits.assign(std::size_t((count+63)/64),0);dense=true;
  for(const auto&t:g.tiles){const auto i=index(t.first);bits[std::size_t(i>>6)]|=std::uint64_t(1)<<(i&63);}
 }
 std::uint64_t index(Cell k)const{return (std::uint64_t(k.x-lo[0])*std::uint64_t(n[1])+std::uint64_t(k.y-lo[1]))*std::uint64_t(n[2])+std::uint64_t(k.z-lo[2]);}
 const core::Tile*find(Cell k)const{
  if(dense){for(std::size_t i=0;i<3;++i)if(std::int64_t(k[int(i)])<lo[i]||std::int64_t(k[int(i)])-lo[i]>=n[i])return nullptr;
   const auto i=index(k);if(!((bits[std::size_t(i>>6)]>>(i&63))&1u))return nullptr;}
  const auto it=grid->tiles.find(k);return it==grid->tiles.end()?nullptr:&it->second;
 }
};
struct CavityScratch {core::Poly polygon,a,b;std::vector<std::array<int,2>> cells;};
inline bool cavityOwner(unsigned material,const core::JoinedVertex*const*t,const Grid&grid,const ColumnRows&rows,core::TileCache&tiles,
                        const std::vector<std::uint8_t>&retained,Budget&budget,std::uint32_t&owner,CavityScratch&s){
 need(material!=0,"support_requires_cavity");
 Vec normal=t[0]->normal+t[1]->normal+t[2]->normal;
 int axis=0;for(int a=1;a<3;++a)if(std::abs(normal[a])>std::abs(normal[axis]))axis=a;
 need(length(normal)>1e-10,"support_missing_authored_normal");
 for(int a=0;a<3;++a)if(a!=axis)need(std::abs(normal[a])<1e-6*std::abs(normal[axis]),"support_non_axial_cavity");
 double position=0;for(int k=0;k<3;++k)position+=t[k]->p[axis]/3;
 const int plane=int(std::llround((position-grid.origin[axis])/grid.pitch));
 const double exact=grid.origin[axis]+plane*grid.pitch;
 auto&polygon=s.polygon;polygon.clear();core::Bounds bounds;
 for(int k=0;k<3;++k){need(std::abs(t[k]->p[axis]-exact)<=1e-6,"support_off_lattice_plane");
  core::V p;p.p=t[k]->p;p.p[axis]=exact;polygon.push_back(p);bounds.add(p.p);}
 const int u=(axis+1)%3,v=(axis+2)%3;
 int lo[2],hi[2];
 for(int i=0;i<2;++i){int a=i?v:u;
  lo[i]=std::max(0,int(std::floor((bounds.lo[a]-grid.origin[a])/grid.pitch)));
  hi[i]=std::min(grid.size[a]-1,int(std::floor((bounds.hi[a]-grid.origin[a])/grid.pitch)));}
 const std::uint64_t cells=hi[0]>=lo[0]&&hi[1]>=lo[1]?std::uint64_t(std::int64_t(hi[0])-lo[0]+1)*std::uint64_t(std::int64_t(hi[1])-lo[1]+1):0;
 if(budget.visited>budget.limit||cells>budget.limit-budget.visited)return false;
 // cavity() visits n[axis]=fixed, n[u]=i, n[v]=j; here grouped by column.
 const int fixed=plane-(normal[axis]>0?1:0);
 std::uint32_t candidate=0;s.cells.clear();bool valid=true;
 auto visit=[&](int i,int j,std::uint32_t id){
  if(!id)return true;
  Cell n;(&n.x)[axis]=fixed;(&n.x)[u]=i;(&n.x)[v]=j;
  if(!(grid.solidBase(n)&&!tiles.removed(n)))return false; // cavity() throws support_topology_material_mismatch here
  if(id<retained.size()&&retained[id]){if(!candidate)candidate=id;else if(id!=candidate)return false;s.cells.push_back({{i,j}});}
  return true;};
 if(axis==0){ // x fixed; i=y, j=z: one column per i
  for(int i=lo[0];i<=hi[0]&&valid;++i){const auto*c=rows.column(fixed,i);
   for(int j=lo[1];j<=hi[1]&&valid;++j)valid=visit(i,j,c?rows.spanAt(*c,j):0);}
 }else if(axis==1){ // y fixed; i=z, j=x: walk row y=fixed
  const auto*end=rows.end();const auto*c=rows.lower(lo[1],fixed);
  for(int j=lo[1];j<=hi[1]&&valid;++j){while(c!=end&&c->y==fixed&&c->x<j)++c;
   const bool has=c!=end&&c->y==fixed&&c->x==j;
   for(int i=lo[0];i<=hi[0]&&valid;++i)valid=visit(i,j,has?rows.spanAt(*c,i):0);}
 }else{ // z fixed; i=x, j=y: walk row y=j
  const auto*end=rows.end();
  for(int j=lo[1];j<=hi[1]&&valid;++j){const auto*c=rows.lower(lo[0],j);
   for(int i=lo[0];i<=hi[0]&&valid;++i){while(c!=end&&c->y==j&&c->x<i)++c;
    valid=visit(i,j,c!=end&&c->y==j&&c->x==i?rows.spanAt(*c,fixed):0);}}
 }
 if(!valid)return false;
 bool positive=false;
 for(const auto&cell:s.cells){const int i=cell[0],j=cell[1];
  core::clipInto(polygon,u,grid.origin[u]+i*grid.pitch,1,s.a);
  core::clipInto(s.a,u,grid.origin[u]+(i+1)*grid.pitch,-1,s.b);
  core::clipInto(s.b,v,grid.origin[v]+j*grid.pitch,1,s.a);
  core::clipInto(s.a,v,grid.origin[v]+(j+1)*grid.pitch,-1,s.b);
  if(s.b.empty())continue;const double area=core::area(s.b);if(area>1e-16){positive=true;break;}
 }
 budget.visited+=cells;owner=positive?candidate:0;return true;
}
}}
