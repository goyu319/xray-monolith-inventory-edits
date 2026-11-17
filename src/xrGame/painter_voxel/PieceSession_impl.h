#pragma once
// Piece sessions (PieceSeed.h): hit mapping and publication. Included by LiveDamageWorker.cpp after Session::Impl.
namespace sep { inline void throwIfCanceled(); }
namespace piece_session {
inline std::vector<painter_detail::PieceCellSupport> keptCellSupport(const gfy_connectivity::Topology&,const Grid&,const std::vector<int>*,std::size_t,const gfy_packed::Mesh*);
// No skin of this piece under the shot (a cut face, a torn end): the hit bone's rest frame maps it instead.
inline Mapping boneRay(const Impact&event){
 need(event.hit_bone<event.bone_model_render.size(),"piece_hit_bone");
 const Matrix&m=event.bone_model_render[event.hit_bone];const auto&a=m.row_major;
 Mapping r;r.face=~std::uint32_t(0);r.bary={0,0,0};r.posed_distance=event.source_search_distance;
 r.entry=inverseDirection(m,vec(event.accepted_pose_point)-Vec{a[3],a[7],a[11]});
 r.direction=unit(inverseDirection(m,unit(vec(event.accepted_pose_direction))));
 return r;}
inline Cell cellOf(const Grid&g,const Vec&p){return Cell{int(std::floor((p.x-g.origin.x)/g.pitch)),int(std::floor((p.y-g.origin.y)/g.pitch)),int(std::floor((p.z-g.origin.z)/g.pitch))};}
inline bool stored(const Grid&g,const Cell&c){const auto it=g.tiles.find(Grid::tileKey(c));return it!=g.tiles.end()&&core::tileBit(it->second,c);}
// Remaining tissue at c: 1 this piece's, 2 another piece's, 0 none (air or removed).
inline int tissueAt(const Grid&g,const Cell&c){
 if(!g.valid(c)||!g.rawMaterialBase(c)||stored(g,c))return 0;return g.ownership->body(c)?1:2;}
// Other pieces' cells in the given tiles and the air within 3 cells of them nearer to them than to this piece
// (LowPolyPieces.h foreign marks with the ownership predicate).
inline std::vector<std::pair<Cell,core::Tile>> foreignMarks(const Grid&g,const std::vector<Cell>&tiles){
 std::vector<std::vector<std::pair<Cell,core::Tile>>> parts(core::parallelWidth());
 core::parallelChunks(tiles.size(),16,[&](std::size_t first,std::size_t last,std::size_t chunk){
  constexpr int H=3,W=8+2*H;std::vector<int> label(W*W*W),next(W*W*W);auto at=[&](int x,int y,int z){return (x*W+y)*W+z;};
  for(std::size_t i=first;i<last;++i){const Cell key=tiles[i];const Cell base{key.x*8-H,key.y*8-H,key.z*8-H};bool any=false;
   for(int x=0;x<W;++x)for(int y=0;y<W;++y)for(int z=0;z<W;++z){const Cell c{base.x+x,base.y+y,base.z+z};int&l=label[at(x,y,z)];
    if(!g.valid(c)){l=-1;continue;}if(stored(g,c)){l=-3;continue;}
    if(g.rawMaterialBase(c)){l=g.ownership->body(c)?0:1;any=any||l==1;}else l=-1;}
   if(!any)continue;
   for(int step=0;step<H;++step){next=label;
    for(int x=0;x<W;++x)for(int y=0;y<W;++y)for(int z=0;z<W;++z){if(label[at(x,y,z)]!=-1)continue;
     const int q[6][3]={{x-1,y,z},{x+1,y,z},{x,y-1,z},{x,y+1,z},{x,y,z-1},{x,y,z+1}};
     for(const auto&p:q){if(p[0]<0||p[1]<0||p[2]<0||p[0]>=W||p[1]>=W||p[2]>=W)continue;const int v=label[at(p[0],p[1],p[2])];if(v>=0){next[at(x,y,z)]=v;break;}}}
    label.swap(next);}
   core::Tile t{};std::size_t n=0;
   for(int x=0;x<8;++x)for(int y=0;y<8;++y)for(int z=0;z<8;++z){if(label[at(x+H,y+H,z+H)]!=1)continue;
    const auto bit=Grid::local(Cell{key.x*8+x,key.y*8+y,key.z*8+z});t.removed[bit/64]|=std::uint64_t(1)<<(bit%64);++n;}
   if(n)parts[chunk].emplace_back(key,t);}});
 std::vector<std::pair<Cell,core::Tile>> out;for(auto&p:parts)out.insert(out.end(),p.begin(),p.end());return out;}
// Tiles within 3 cells of a triangle's surface (4 mm samples), as LowPolyPieces.h does for boundary triangles.
inline void surfaceTiles(const Grid&g,const Vec&a,const Vec&b,const Vec&d,std::vector<Cell>&out){
 const double edge=std::max({length(b-a),length(d-a),length(d-b)});const int steps=std::max(1,int(std::ceil(edge/.004)));
 for(int i=0;i<=steps;++i)for(int j=0;j<=steps-i;++j){const Cell q=cellOf(g,a+(b-a)*(double(i)/steps)+(d-a)*(double(j)/steps));
  for(int dx=-3;dx<=3;dx+=6)for(int dy=-3;dy<=3;dy+=6)for(int dz=-3;dz<=3;dz+=6)out.push_back(Cell{(q.x+dx)>>3,(q.y+dy)>>3,(q.z+dz)>>3});}}
struct Stats {bool supportAdmitted=false;std::size_t supportBytes=0;std::size_t cuts{},triangles{},kept{},dropped{},decided{},boundary{},markTiles{},marks{};double channelMs{},filterMs{},markMs{},fieldMs{};};
}
struct PieceSessionOps { // friend of Session: its Impl
// Cuts at birth are drawn by the birth mesh: the channel cache starts there, so only later cuts get walls planned.
static std::shared_ptr<const channelworker::State> channelPrior(const Session::Impl&i){
 auto s=std::make_shared<channelworker::State>();s->asset_pin=i.asset;s->history=i.piece->history;s->tolerance=.009;
 s->total_cuts=i.piece->history->total_cuts;return s;}
// The piece's publication: the mesh its owner committed at birth (skin, walls, removed original children), the walls
// of the cuts made since that lie on this piece's tissue, the cumulative analytic field and the piece's removal field.
static void pack(Session::Impl&i,const Grid&grid,const std::shared_ptr<const ResolvedCutHistory>&history,
    std::shared_ptr<const channelworker::State>&candidate,const std::shared_ptr<Output>&out,WorkerPhaseTimes&phases){
 using namespace piece_session;
 PhaseTime whole(phases.pack_ms);const PieceSeed&seed=*i.piece;const gfy_packed::Mesh&birth=*i.piece_published;
 need(grid.ownership&&history&&seed.history,"piece_publication_inputs"); // birth may have no wound walls
 Stats st;auto t0=core::Clock::now();
 fastwound::WallWeights weights(*i.asset);
 packedchannel::WeightExecutor executor=[](unsigned jobs,const std::function<void(unsigned)>&work){
  core::parallelChunks(jobs,1,[&](std::size_t first,std::size_t last,std::size_t){for(auto n=first;n<last;++n)work(unsigned(n));});};
 auto prepared=channelworker::prepareChannels(i.channel_cache?i.channel_cache:channelPrior(i),history,i.asset,i.lineage,weights,executor);
 st.channelMs=core::ms(t0);t0=core::Clock::now();
 const std::uint32_t baseCuts=std::uint32_t(seed.history->total_cuts);st.cuts=history->total_cuts-baseCuts;
 // Which new wall triangles lie on this piece: the tissue behind them (either side, 8 mm), else the nearest (6 mm).
 // A cut's walls never change once packed, so each triangle is decided once: verdicts are kept per cut (checked
 // against the cut's record) by the triangle's ordinal within its cut in child order; a publication decides only the
 // triangles of cuts it has not seen (speed: the filter was re-run over every wall since birth).
 const auto&packets=prepared.state->children;
 std::vector<const ResolvedTunnel*> records;records.reserve(history->total_cuts);
 {std::vector<const ResolvedCutHistory*>chain;for(auto p=history;p;p=p->parent)chain.push_back(p.get());
  for(auto it=chain.rbegin();it!=chain.rend();++it)for(const auto&r:(*it)->appended)records.push_back(&r);}
 need(records.size()==history->total_cuts,"piece_history_count");
 auto&memo=i.piece_walls;if(memo.size()>st.cuts)memo.resize(st.cuts);
 for(std::size_t k=0;k<memo.size();++k){const auto&r=*records[baseCuts+k];
  if(memo[k].sequence!=r.sequence||memo[k].role!=unsigned(r.role)||memo[k].entry!=std::array<double,3>{r.entry.x,r.entry.y,r.entry.z}){memo.resize(k);break;}}
 const std::size_t known=memo.size();memo.resize(st.cuts);
 for(std::size_t k=known;k<st.cuts;++k){const auto&r=*records[baseCuts+k];memo[k].sequence=r.sequence;memo[k].role=unsigned(r.role);memo[k].entry={r.entry.x,r.entry.y,r.entry.z};memo[k].verdict.clear();}
 std::vector<std::vector<signed char>> verdict(packets.size());std::vector<std::uint32_t> ordinal(st.cuts,0);
 struct Todo {std::uint32_t c,t;};std::vector<Todo> todo;
 for(std::uint32_t c=0;c<packets.size();++c){const auto&packet=*packets[c];const auto T=std::uint32_t(packet.geometry.indices.size()/3);auto&v=verdict[c];v.assign(T,-1);
  for(std::uint32_t t=0;t<T;++t){const auto owner=packet.owners[t].owner;if(owner<baseCuts)continue;const auto k=owner-baseCuts;const auto ord=ordinal[k]++;
   if(ord<memo[k].verdict.size())v[t]=memo[k].verdict[ord];else{v[t]=-2;todo.push_back({c,t});}}}
 auto position=[](const gfy_packed::Vertex&v){return Vec{v.surface[0],v.surface[1],v.surface[2]};};
 auto normalOf=[](const gfy_packed::Vertex&v,Vec&n){n={v.surface[3],v.surface[4],v.surface[5]};const double l=length(n);if(!(l>1e-6)||!std::isfinite(l))return false;n=n*(1/l);return true;};
 core::parallelChunks(todo.size(),64,[&](std::size_t first,std::size_t last,std::size_t){
  // speed: the stored bit through a tile-pointer cache (an overlay grid keeps the direct lookup: its base is not stored()'s), and
  // before the anatomy read. storedAt(q) == stored(grid,q) and tissueOf(q) == tissueAt(grid,q) for every cell.
  core::TileCache tileCache(grid);const bool direct=grid.overlay_base!=nullptr;
  auto storedAt=[&](const Cell&q){return direct?stored(grid,q):tileCache.bit(q);};
  auto tissueOf=[&](const Cell&q)->int{if(!grid.valid(q)||storedAt(q)||!grid.rawMaterialBase(q))return 0;return grid.ownership->body(q)?1:2;};
  for(std::size_t j=first;j<last;++j){const auto c=todo[j].c,t=todo[j].t;const auto&ch=packets[c]->geometry;
   const gfy_packed::Vertex*v[3]={&ch.vertices[ch.indices[3*t]],&ch.vertices[ch.indices[3*t+1]],&ch.vertices[ch.indices[3*t+2]]};
   bool own=false,other=false;
   auto sample=[&](const Vec&p,Vec n){for(double s=.001;s<=.008;s+=.002)for(int sg:{-1,1}){const int k=tissueOf(cellOf(grid,p+n*(s*sg)));if(k){(k==1?own:other)=true;return;}}};
   Vec n[3];bool normals=true;for(int q=0;q<3;++q)normals=normalOf(*v[q],n[q])&&normals;
   if(normals){for(int q=0;q<3;++q)sample(position(*v[q]),n[q]);
    const Vec cc=(position(*v[0])+position(*v[1])+position(*v[2]))*(1./3),nc=n[0]+n[1]+n[2];if(length(nc)>1e-9)sample(cc,nc*(1/length(nc)));}
   if(!own&&!other){const Cell c0=cellOf(grid,(position(*v[0])+position(*v[1])+position(*v[2]))*(1./3));
    for(int rr=0;rr<=6&&!own&&!other;++rr)for(int dx=-rr;dx<=rr&&!own&&!other;++dx)for(int dy=-rr;dy<=rr&&!own&&!other;++dy)for(int dz=-rr;dz<=rr;++dz){
     if(std::max({std::abs(dx),std::abs(dy),std::abs(dz)})!=rr)continue;const int k=tissueOf(Cell{c0.x+dx,c0.y+dy,c0.z+dz});if(k){(k==1?own:other)=true;break;}}}
   // R53: a wall in the anatomy-to-clothing gap (no tissue within 8 mm / 6 cells) belongs to the side whose tissue is
   // nearest along straight paths inside the closed body envelope that never enter removed volume (LowPolyPieces.h).
   if(!own&&!other&&i.asset->closure_field){const auto&cf=*i.asset->closure_field;auto inside=[&](const Vec&q){return cf.sample(q.x,q.y,q.z)!=0u;};
    static const double r2=0.7071067811865476,r3=0.5773502691896258;
  static const Vec dirs[26]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1},
   {r2,r2,0},{r2,-r2,0},{-r2,r2,0},{-r2,-r2,0},{r2,0,r2},{r2,0,-r2},{-r2,0,r2},{-r2,0,-r2},{0,r2,r2},{0,r2,-r2},{0,-r2,r2},{0,-r2,-r2},
   {r3,r3,r3},{r3,r3,-r3},{r3,-r3,r3},{r3,-r3,-r3},{-r3,r3,r3},{-r3,r3,-r3},{-r3,-r3,r3},{-r3,-r3,-r3}};
    double reach=.12;int found=0;
    for(int q=0;q<3;++q){Vec nq;const bool hn=normalOf(*v[q],nq);const Vec s0=hn?position(*v[q])-nq*.003:position(*v[q]);
     if(!inside(s0)||storedAt(cellOf(grid,s0)))continue;
     for(const auto&d:dirs){if(hn&&dot(d,nq)>.3)continue;
      for(double s=.0025;s<=reach;s+=.0025){const Vec p=s0+d*s;const Cell cc=cellOf(grid,p);const int k=tissueOf(cc);
       if(k){found=k;reach=s-.0025;break;}if(storedAt(cc)||!inside(p))break;}}}
    if(found)(found==1?own:other)=true;}
   verdict[c][t]=signed char(own?(other?2:1):0);}});
 for(const auto&d:todo){const auto k=packets[d.c]->owners[d.t].owner-baseCuts;memo[k].verdict.push_back(verdict[d.c][d.t]);}
 struct Keep {std::vector<std::uint32_t> tris;std::vector<char> boundary;};std::vector<Keep> keep(packets.size());
 for(std::size_t c=0;c<packets.size();++c){const auto&v=verdict[c];
  for(std::uint32_t t=0;t<v.size();++t){if(v[t]<0)continue;if(!v[t]){++st.dropped;continue;}keep[c].tris.push_back(t);keep[c].boundary.push_back(v[t]==2?1:0);}}
 st.decided=todo.size();
 st.filterMs=core::ms(t0);t0=core::Clock::now();
 gfy_packed::validate_skin_ownership(birth);
 gfy_packed::Mesh mesh;mesh.reduced_skin_vertices=birth.reduced_skin_vertices;mesh.max_discarded_skin_weight=birth.max_discarded_skin_weight;
 // Removed tiles as boxes: a skin child near one is clipped per pixel.
 std::vector<std::array<double,6>> removedTiles;removedTiles.reserve(grid.tiles.size());
 for(const auto&t:grid.tiles){bool any=false;for(auto w:t.second.removed)any|=w!=0;if(!any)continue;std::array<double,6>b;
  for(int a=0;a<3;++a){b[a]=grid.origin[a]+double((&t.first.x)[a])*8*grid.pitch;b[3+a]=b[a]+8*grid.pitch;}removedTiles.push_back(b);}
 auto removedNear=[&](const gfy_packed::Child&c){double lo[3]={1e300,1e300,1e300},hi[3]={-1e300,-1e300,-1e300};
  for(const auto&v:c.vertices)for(int a=0;a<3;++a){lo[a]=std::min(lo[a],double(v.surface[a]));hi[a]=std::max(hi[a],double(v.surface[a]));}
  for(const auto&b:removedTiles){bool overlap=true;for(int a=0;a<3&&overlap;++a)overlap=b[a]<=hi[a]+grid.pitch&&b[3+a]>=lo[a]-grid.pitch;if(overlap)return true;}return false;};
 unsigned next=i.asset->original_children;
 for(const auto&c:birth.children){next=std::max(next,unsigned(c.child_index)+1);auto child=c;
  if(!child.material&&!child.vertices.empty()&&!child.skin_clip)child.skin_clip=removedNear(child);mesh.children.push_back(std::move(child));}
 // New walls: this piece's weights (a bone the piece does not pose moves with its anchor), then their tiles for marks.
 std::vector<Cell> markTiles;std::vector<std::pair<unsigned,std::vector<packedchannel::OwnerTriangle>>> fresh;
 for(std::size_t c=0;c<packets.size();++c){const auto&k=keep[c];if(k.tris.empty())continue;const auto&src=packets[c]->geometry;
  need(next<gfy_packed::max_child_slots,"piece_child_slots");
  gfy_packed::Child child;child.material=src.material;child.material_source_child=src.material_source_child;child.child_index=std::uint16_t(next++);
  std::vector<std::int32_t> map(src.vertices.size(),-1);const bool faces=src.original_faces.size()*3==src.indices.size();
  std::vector<packedchannel::OwnerTriangle> owners;owners.reserve(k.tris.size());
  for(std::size_t n=0;n<k.tris.size();++n){const auto t=k.tris[n];
   for(int j=0;j<3;++j){const auto v=src.indices[3*t+j];
    if(map[v]<0){map[v]=std::int32_t(child.vertices.size());auto row=src.vertices[v];std::array<float,4> w{};std::array<std::uint16_t,4> b{};unsigned used=0;
     if(!gfy_signed_hw::remap_hardware(row.bone,row.weight,seed.render_bone)){
     for(unsigned q=0;q<4;++q){if(row.weight[q]==0)continue;const auto bone=row.bone[q]<64?seed.render_bone[row.bone[q]]:row.bone[q];
      unsigned s=0;while(s<used&&b[s]!=bone)++s;if(s==used){b[used]=bone;w[used]=0;++used;}w[s]+=row.weight[q];}
     float sum=0;for(unsigned q=0;q<used;++q)sum+=w[q];
     if(sum>0){for(unsigned q=0;q<4;++q){row.bone[q]=q<used?b[q]:b[0];row.weight[q]=q<used?w[q]/sum:0.f;}gfy_packed::canonical_weights(row);}}
     child.vertices.push_back(row);}
    child.indices.push_back(std::uint16_t(map[v]));}
   child.original_faces.push_back(faces?src.original_faces[t]:~std::uint32_t(0));owners.push_back(packets[c]->owners[t]);
   if(k.boundary[n]){++st.boundary;surfaceTiles(grid,position(src.vertices[src.indices[3*t]]),position(src.vertices[src.indices[3*t+1]]),position(src.vertices[src.indices[3*t+2]]),markTiles);}}
  st.kept+=k.tris.size();fresh.emplace_back(child.child_index,std::move(owners));mesh.children.push_back(std::move(child));}
 // Cumulative analytic field: every cut in order, the birth walls' primitive rows, then the new walls'.
 painter_detail::ChannelIdentity identity;identity.source=i.asset->source_hash;identity.rig=i.asset->rig_hash;identity.cookie=i.owner.spawn_cookie;identity.lineage=i.owner.visual_lineage;identity.topology=i.owner.topology;
 identity.generation=out->stamp.expected_generation;identity.revision=out->stamp.revision;identity.snapshot=history;
 if(birth.skin_ownership){auto own=std::make_shared<painter_detail::SkinOwnershipField>(*birth.skin_ownership);own->identity=identity;own->validate();gfy_packed::set_skin_ownership(mesh,std::move(own));}
 painter_detail::ChannelFieldBuilder builder(identity,history->limitations);std::vector<const ResolvedCutHistory*>nodes;
 for(auto p=history;p;p=p->parent)nodes.push_back(p.get());std::reverse(nodes.begin(),nodes.end());
 for(auto node:nodes)for(const auto&record:node->appended){auto resolved=record;
  if(!resolved.edge&&!resolved.cracks){channelworker::ResolvedView view(resolved);resolved.edge_u={view.edge_u.x,view.edge_u.y,view.edge_u.z};resolved.edge_v={view.edge_v.x,view.edge_v.y,view.edge_v.z};}builder.append(resolved);}
 if(birth.analytic_channels)for(const auto&pf:birth.analytic_channels->children){std::vector<packedchannel::OwnerTriangle> rows(pf.count);
  for(unsigned k=0;k<pf.count;++k){rows[k].owner=pf.data[k][0];rows[k].kind=static_cast<decltype(rows[k].kind)>(pf.data[k][1]);}builder.addChild(pf.child,rows);}
 for(const auto&f:fresh)builder.addChild(f.first,f.second);
 // Every piece publication refreshes membership from exact birth topology
 // minus all current removals, for BOTH inherited walls and newly carved cuts.
 const auto baseline=builder.finish();mesh.analytic_channels=baseline;
 const auto cellSupport=keptCellSupport(*seed.topology,grid,nullptr,1,&mesh);
 const painter_detail::PieceCellSupport missing;const auto&support=cellSupport.size()==1?cellSupport[0]:missing;
 try{auto supported=std::make_shared<painter_detail::AnalyticChannelField>(*baseline);
  st.supportAdmitted=painter_detail::appendPieceCellSupport(*supported,{grid.origin.x,grid.origin.y,grid.origin.z},grid.pitch,support,[](){sep::throwIfCanceled();});
  if(st.supportAdmitted){st.supportBytes=supported->bytes();mesh.analytic_channels=std::move(supported);}
 }catch(const std::bad_alloc&){/* keep the baseline publication; no stale parent support */}
 // Removal field: the piece's damage, the other pieces' cells beside its birth triangles, and beside its new walls.
 const auto m0=core::Clock::now();
 std::sort(markTiles.begin(),markTiles.end());markTiles.erase(std::unique(markTiles.begin(),markTiles.end()),markTiles.end());
 {const int tx=(grid.size.x+7)/8,ty=(grid.size.y+7)/8,tz=(grid.size.z+7)/8;
  markTiles.erase(std::remove_if(markTiles.begin(),markTiles.end(),[&](const Cell&k){return k.x<0||k.y<0||k.z<0||k.x>=tx||k.y>=ty||k.z>=tz;}),markTiles.end());}
 {std::vector<Cell> fresh;for(const auto&k:markTiles)if(!i.piece_mark_tiles.count(k))fresh.push_back(k);st.markTiles=fresh.size();
  for(auto&m:foreignMarks(grid,fresh))i.piece_marks.emplace_back(std::move(m));i.piece_mark_tiles.insert(fresh.begin(),fresh.end());}
 const auto&marks=i.piece_marks;st.markMs=core::ms(m0);
 struct FieldTiles {std::unordered_map<Cell,core::Tile,core::CellHash> tiles;const void* ownership=nullptr;double pitch;Vec origin;decltype(grid.size) size;};
 FieldTiles field{{},nullptr,grid.pitch,grid.origin,grid.size};
 for(const auto&t:grid.tiles){std::uint64_t any=0;for(auto w:t.second.removed)any|=w;if(any)field.tiles.emplace(t.first,t.second);}
 for(const auto*list:{&seed.marks,&marks})for(const auto&m:*list){auto&t=field.tiles[m.first];for(unsigned q=0;q<8;++q)t.removed[q]|=m.second.removed[q];st.marks+=1;}
 mesh.removal_field=painter_detail::buildDamageField(field);need(mesh.removal_field&&mesh.removal_field->available,"piece_removal_field");
 // Without birth walls, new walls use the same closure-field coverage proof as whole-body publication.
 mesh.tissue_field=i.asset->closure_field;mesh.analytic_coverage_admitted=(birth.analytic_channels?birth.analytic_coverage_admitted:true)&&bool(mesh.tissue_field);
 st.fieldMs=core::ms(m0);
 out->mesh=std::move(mesh);out->triangles=0;for(const auto&c:out->mesh.children)out->triangles+=c.indices.size()/3;
 out->analytic_diagnostic=true;out->analytic_coverage_requirements=prepared.state->coverage_requirements;
 out->analytic_new_cuts=prepared.stats.planned_cuts;out->analytic_new_triangles=prepared.stats.planned_triangles;out->analytic_cavity_triangles=st.kept;
 out->analytic_plan_ms=prepared.stats.plan_ms;out->analytic_weight_ms=prepared.stats.weight_ms;out->analytic_pack_ms=prepared.stats.pack_ms;out->analytic_append_ms=prepared.stats.append_ms;out->analytic_mask_ms=st.markMs;
 if(std::getenv("GFY_PIECE_LOG"))std::cerr<<"piece_publication cookie="<<i.owner.spawn_cookie<<" revision="<<out->stamp.revision<<" new_cuts="<<st.cuts<<" walls_kept="<<st.kept<<" dropped="<<st.dropped<<" decided="<<st.decided
  <<" cell_support="<<(st.supportAdmitted?"admitted":"refused")<<" cell_support_bytes="<<st.supportBytes<<" boundary="<<st.boundary<<" mark_tiles="<<st.markTiles<<" channel_ms="<<st.channelMs<<" filter_ms="<<st.filterMs<<" mark_ms="<<st.markMs<<" field_ms="<<st.fieldMs<<" triangles="<<out->triangles<<std::endl;
 candidate=prepared.state;
}
// Mask-only wounds, slice 2 (painter_voxel_mask_only 2): a mask-only piece keeps its birth children (the original exterior
// triangles chosen at the separation and its coverage box) unchanged; a hit only rebuilds its removal field = the piece grid's
// removed cells OR the separation's marked tiles (the seed's marks: other pieces' cells, foreign free cells, removed cells).
static void packMask(Session::Impl&i,const Grid&grid,const std::shared_ptr<Output>&out,WorkerPhaseTimes&phases,
    const std::shared_ptr<const ResolvedCutHistory>&history=nullptr){ // slice 4: the decide-once tears made on this piece since its birth
 PhaseTime whole(phases.pack_ms);const PieceSeed&seed=*i.piece;const gfy_packed::Mesh&birth=*i.piece_published;
 need(birth.mask_piece,"mask_piece_birth");const auto m0=core::Clock::now();
 gfy_packed::Mesh mesh;mesh.reduced_skin_vertices=birth.reduced_skin_vertices;mesh.max_discarded_skin_weight=birth.max_discarded_skin_weight;
 mesh.children=birth.children;
 struct FieldTiles {std::unordered_map<Cell,core::Tile,core::CellHash> tiles;const void* ownership=nullptr;double pitch;Vec origin;decltype(grid.size) size;};
 FieldTiles field{{},nullptr,grid.pitch,grid.origin,grid.size};
 for(const auto&t:grid.tiles){std::uint64_t any=0;for(auto w:t.second.removed)any|=w;if(any)field.tiles.emplace(t.first,t.second);}
 for(const auto&m:seed.marks){auto&t=field.tiles[m.first];for(unsigned q=0;q<8;++q)t.removed[q]|=m.second.removed[q];}
 mesh.removal_field=painter_detail::buildDamageField(field);need(mesh.removal_field&&mesh.removal_field->available,"mask_piece_removal_field");
 mesh.tissue_field=birth.tissue_field;mesh.mask_volume=true;mesh.mask_piece=true;
 mesh.tears=birth.tears;mesh.tear_atlas=birth.tear_atlas; // slices 3 + 4: the piece keeps its tears and their drawing set; only the removal field changes
 if(history&&seed.history){std::vector<const ResolvedCutHistory*> chain;for(auto p=history.get();p;p=p->parent.get())chain.push_back(p);
  std::size_t at=0;const std::size_t base=seed.history->total_cuts; // tears decided on this piece since its birth ride unsplit
  for(auto h=chain.rbegin();h!=chain.rend();++h)for(const auto&r:(*h)->appended){if(at++<base)continue;if(r.surface&&r.surface->decide)mesh.tears.push_back({r.surface->decide,255});}}
 out->mesh=std::move(mesh);out->triangles=0;for(const auto&c:out->mesh.children)out->triangles+=c.indices.size()/3;
 out->analytic_mask_ms=core::ms(m0);
 if(std::getenv("GFY_PIECE_LOG"))std::cerr<<"piece_publication mask_piece=1 cookie="<<i.owner.spawn_cookie<<" revision="<<out->stamp.revision<<" field_tiles="<<field.tiles.size()
  <<" field_bytes="<<out->mesh.removal_field->bytes()<<" field_ms="<<out->analytic_mask_ms<<" triangles="<<out->triangles<<std::endl;
}
};
