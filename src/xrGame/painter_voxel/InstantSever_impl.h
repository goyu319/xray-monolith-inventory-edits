#pragma once
namespace gfy_live {
namespace instant_detail {
constexpr double tau=6.283185307179586;
// Sub-phase times on stderr (GFY_INSTANT_PROF, measurement only).
inline bool profiling(){static const bool on=std::getenv("GFY_INSTANT_PROF")!=nullptr;return on;}
// Profile lines are collected per job and written once (unbuffered stderr writes would land inside the timings).
inline std::ostringstream& profileText(){thread_local std::ostringstream text;return text;}
// Thread cycles in millions (QueryThreadCycleTime: preemption by other load is not counted), wall ms elsewhere.
#if defined(_WIN32)
extern "C" __declspec(dllimport) int __stdcall QueryThreadCycleTime(void*,unsigned long long*);
extern "C" __declspec(dllimport) int __stdcall GetThreadPriority(void*);
#endif
inline double threadMegacycles(){
#if defined(_WIN32)
 unsigned long long cycles=0;QueryThreadCycleTime(GetCurrentThread(),&cycles);return double(cycles)*1e-6;
#else
 return core::ms(core::Clock::time_point{});
#endif
}
struct ProfClock {double t=threadMegacycles();double lap(){const double now=threadMegacycles();const double v=now-t;t=now;return v;}};
// ---- anatomy lookups ---------------------------------------------------------------------------------------------
// Material slot (0 none, 2 bone, 5 muscle, ...) of the immutable anatomy at p: the brick tissue field (O(1)) when the
// asset has one, else the anatomy runs. closure(): the render closure field (anatomy + air inside the closed body
// envelope, what the channel shader draws), else the anatomy.
struct Slots {
 const Asset&asset;
 unsigned at(const Vec&p)const{
  if(asset.tissue_field&&asset.tissue_field->available)return asset.tissue_field->sample(p.x,p.y,p.z);
  const auto&g=asset.base;if(!g.anatomy||!g.material_slots)return 0;const auto label=g.anatomy->point(p.x,p.y,p.z);
  return label&&label<g.material_slots->size()?unsigned((*g.material_slots)[label]&63u):0u;}
 unsigned closure(const Vec&p)const{
  if(asset.closure_field&&asset.closure_field->available)return asset.closure_field->sample(p.x,p.y,p.z);return at(p);}
};
// ---- station sample maps (cached per asset) ----------------------------------------------------------------------
// A station slab (12 mm across the segment, radius R) sampled on a square grid of pitch h in its plane and four layers
// 3 mm apart, grouped in blocks of 4 x 4 pixels x 4 layers (bit = layer*16 + y*4 + x). Only blocks holding tissue are
// kept, with their tissue and bone bits.
inline double pitchFor(double radius){return radius<.03?.001:.002;}
struct StationMap {
 double h{},R{};int n{},bn{};Vec C,a,u,v;std::array<double,4> layer{{-.0045,-.0015,.0015,.0045}};
 struct Block {std::uint64_t tissue{},bone{};std::uint16_t tissueCount{},boneCount{};std::int16_t bx{},by{};};
 std::vector<Block> blocks;std::uint32_t tissue{},bone{};
 // 2 x 2 blocks (4h x 4h... 8h x 8h x 12 mm) classified before their blocks: first block, count, tissue and bone.
 struct Group {std::uint32_t first{},count{};std::uint32_t tissueCount{},boneCount{};std::int16_t gx{},gy{};};std::vector<Group> groups;
 Vec groupCenter(const Group&g)const{return C+u*((g.gx*8+4-n*.5)*h)+v*((g.gy*8+4-n*.5)*h);}
 double groupRadius()const{return std::sqrt(2*(3.5*h)*(3.5*h)+.0045*.0045)+1e-9;}
 Vec sample(const Block&b,unsigned bit)const{const int l=int(bit>>4),y=int((bit>>2)&3u),x=int(bit&3u);
  return C+u*((b.bx*4+x-n*.5+.5)*h)+v*((b.by*4+y-n*.5+.5)*h)+a*layer[std::size_t(l)];}
 Vec center(const Block&b)const{return C+u*((b.bx*4+2-n*.5)*h)+v*((b.by*4+2-n*.5)*h);}
 double radius()const{return std::sqrt(2*(1.5*h)*(1.5*h)+.0045*.0045)+1e-9;} // reaches every sample of a block
 // One layer's 2 x 2 quarter (qx,qy in 0..1) of a block: its centre, bits and reach.
 Vec quarter(const Block&b,unsigned l,unsigned qx,unsigned qy)const{
  return C+u*((b.bx*4+2*int(qx)+1-n*.5)*h)+v*((b.by*4+2*int(qy)+1-n*.5)*h)+a*layer[l];}
 static std::uint64_t quarterBits(unsigned l,unsigned qx,unsigned qy){std::uint64_t m=0;
  for(unsigned y=2*qy;y<2*qy+2;++y)for(unsigned x=2*qx;x<2*qx+2;++x)m|=std::uint64_t(1)<<(l*16+y*4+x);return m;}
 double quarterRadius()const{return .7071067811865476*h+1e-9;}
};
inline std::shared_ptr<const StationMap> buildStationMap(const SeverSegment&seg,const SeverStation&st,const Slots&slots){
 auto m=std::make_shared<StationMap>();m->R=seg.radius;m->h=pitchFor(seg.radius);
 m->n=4*int(std::ceil(2*seg.radius/m->h/4));m->bn=m->n/4;m->C=st.center;m->a=seg.axis;m->u=seg.u;m->v=seg.v;
 const double R2=seg.radius*seg.radius;
 for(int gy=0;gy<(m->bn+1)/2;++gy)for(int gx=0;gx<(m->bn+1)/2;++gx){StationMap::Group group;group.gx=std::int16_t(gx);group.gy=std::int16_t(gy);group.first=std::uint32_t(m->blocks.size());
 for(int by=2*gy;by<std::min(m->bn,2*gy+2);++by)for(int bx=2*gx;bx<std::min(m->bn,2*gx+2);++bx){StationMap::Block b;b.bx=std::int16_t(bx);b.by=std::int16_t(by);
  for(unsigned bit=0;bit<64;++bit){const int y=int((bit>>2)&3u),x=int(bit&3u);
   const double px=(bx*4+x-m->n*.5+.5)*m->h,py=(by*4+y-m->n*.5+.5)*m->h;if(px*px+py*py>R2)continue;
   const unsigned slot=slots.at(m->sample(b,bit));if(!slot)continue;
   b.tissue|=std::uint64_t(1)<<bit;++b.tissueCount;if(slot==2){b.bone|=std::uint64_t(1)<<bit;++b.boneCount;}}
  if(b.tissue){m->tissue+=b.tissueCount;m->bone+=b.boneCount;group.tissueCount+=b.tissueCount;group.boneCount+=b.boneCount;m->blocks.push_back(b);}}
  group.count=std::uint32_t(m->blocks.size())-group.first;if(group.count)m->groups.push_back(group);}
 return m;
}
// ---- the physics element map, indexed by element (cached per map) --------------------------------------------------
struct ElementRun {std::int32_t x{},y{};std::uint16_t lo{},hi{};};
// Moment sums of one element's cells inside a 16 mm cube (rest space) with their bounds: a plane divides an element
// by whole cubes (the side of each cube's centroid), so mass, centre, inertia and box of a part are sums of these.
struct Coarse {double n{},s[3]{},q[3]{},xy{},xz{},yz{};Vec lo{1e300,1e300,1e300},hi{-1e300,-1e300,-1e300};Vec centroid()const{return Vec{s[0]/n,s[1]/n,s[2]/n};}};
struct ElementIndex {
 std::shared_ptr<const gfy_piece_collider::element_partition::Map> map;
 std::array<std::vector<ElementRun>,64> runs;std::array<std::uint64_t,64> cells{};std::array<Vec,64> centroid{};
 std::array<std::vector<Coarse>,64> coarse;
 Vec cellCenter(int x,int y,double z)const{return Vec{map->origin[0]+(x+.5)*map->pitch,map->origin[1]+(y+.5)*map->pitch,map->origin[2]+(z+.5)*map->pitch};}
 // Element of the original anatomy cell at p (65535: none).
 std::uint16_t at(const Vec&p)const{const auto&m=*map;
  const double fx=std::floor((p.x-m.origin[0])/m.pitch),fy=std::floor((p.y-m.origin[1])/m.pitch),fz=std::floor((p.z-m.origin[2])/m.pitch);
  if(!(fx>=0&&fy>=0&&fz>=0&&fx<m.size[0]&&fy<m.size[1]&&fz<m.size[2]))return 65535;
  const auto col=std::size_t(fy)*m.size[0]+std::size_t(fx);const int z=int(fz);
  for(auto r=m.offsets[col];r<m.offsets[col+1];++r)if(m.runs[r].lo<=z&&z<m.runs[r].hi)return m.runs[r].element;
  return 65535;}
};
inline std::shared_ptr<const ElementIndex> buildElementIndex(std::shared_ptr<const gfy_piece_collider::element_partition::Map> map){
 auto out=std::make_shared<ElementIndex>();out->map=map;const auto&m=*map;std::array<long double,64> sx{},sy{},sz{};
 const std::size_t columns=std::size_t(m.size[0])*m.size[1];
 for(std::size_t col=0;col<columns;++col)for(auto r=m.offsets[col];r<m.offsets[col+1];++r){const auto&run=m.runs[r];if(run.element>=64)continue;
  const int x=int(col%m.size[0]),y=int(col/m.size[0]);out->runs[run.element].push_back({x,y,run.lo,run.hi});
  const double n=double(run.hi-run.lo);out->cells[run.element]+=std::uint64_t(run.hi-run.lo);
  sx[run.element]+=n*(x+.5);sy[run.element]+=n*(y+.5);sz[run.element]+=n*(.5*(double(run.lo)+double(run.hi)));}
 for(unsigned e=0;e<64;++e)if(out->cells[e])out->centroid[e]=out->cellCenter(0,0,0)+Vec{double(sx[e]/out->cells[e])-.5,double(sy[e]/out->cells[e])-.5,double(sz[e]/out->cells[e])-.5}*m.pitch;
 constexpr int cube=16;const double h=m.pitch,half=.5*h;
 for(unsigned e=0;e<64;++e){if(out->runs[e].empty())continue;std::unordered_map<std::uint64_t,std::uint32_t> at;auto&list=out->coarse[e];
  for(const auto&run:out->runs[e])for(int z0=run.lo;z0<run.hi;){const int z1=std::min<int>(run.hi,(z0/cube+1)*cube);
   const std::uint64_t key=(std::uint64_t(std::uint32_t(run.x/cube))<<42)|(std::uint64_t(std::uint32_t(run.y/cube))<<21)|std::uint64_t(std::uint32_t(z0/cube));
   auto it=at.find(key);if(it==at.end()){it=at.emplace(key,std::uint32_t(list.size())).first;list.emplace_back();}
   auto&c=list[it->second];const Vec base=out->cellCenter(run.x,run.y,0);const double n=z1-z0;
   const double zs=(double(z0)+double(z1-1))*n*.5,zq=(double(z1-1)*z1*(2.*z1-1)-double(z0-1)*z0*(2.*z0-1))/6.;
   const double X=base.x,Y=base.y,Sz=n*base.z+zs*h,Szz=n*base.z*base.z+2*base.z*zs*h+zq*h*h;
   c.n+=n;c.s[0]+=n*X;c.s[1]+=n*Y;c.s[2]+=Sz;c.q[0]+=n*X*X;c.q[1]+=n*Y*Y;c.q[2]+=Szz;c.xy+=n*X*Y;c.xz+=X*Sz;c.yz+=Y*Sz;
   c.lo={std::min(c.lo.x,X-half),std::min(c.lo.y,Y-half),std::min(c.lo.z,base.z+z0*h-half)};c.hi={std::max(c.hi.x,X+half),std::max(c.hi.y,Y+half),std::max(c.hi.z,base.z+(z1-1)*h+half)};
   z0=z1;}}
 return out;
}
}
struct Asset::InstantCache {
 std::mutex mutex; // guards every member below
 std::vector<std::shared_ptr<const instant_detail::StationMap>> stations; // global station order (SeverState::failed)
 std::shared_ptr<const gfy_packed::Mesh> birth; // the intact body's low-poly publication (static skin, no walls)
 std::shared_ptr<const instant_detail::ElementIndex> elements;
 Hash framesKey{};std::shared_ptr<const gfy_piece_collider::original_frames::Frames> frames;
 Hash accessoryKey{};bool accessoryRead{};std::vector<std::uint16_t> surfaceChildren;
 std::shared_ptr<const sep::SourceFaceIndex> sourceFaces; // (child, face) -> source face, for the seeds
 std::shared_ptr<const std::vector<std::vector<std::uint32_t>>> faceOfChild; // [child][face in child] -> source face (~0: none)
 std::shared_ptr<const std::unordered_set<std::uint64_t>> superTiles; // 64-cell blocks holding tissue (march skipping)
};
namespace instant_detail {
inline Asset::InstantCache& cacheOf(const Asset&asset){
 std::lock_guard<std::mutex> lock(asset.instant_mutex);if(!asset.instant_cache)asset.instant_cache=std::make_shared<Asset::InstantCache>();return *asset.instant_cache;}
inline std::size_t stationCount(const Asset&asset){std::size_t n=0;for(const auto&s:asset.sever_segments)n+=s.stations.size();return n;}
inline std::shared_ptr<const StationMap> stationMap(const Asset&asset,std::size_t k,const SeverSegment&seg,const SeverStation&st){
 auto&c=cacheOf(asset);
 {std::lock_guard<std::mutex> lock(c.mutex);if(c.stations.size()!=stationCount(asset))c.stations.assign(stationCount(asset),{});if(c.stations[k])return c.stations[k];}
 auto built=buildStationMap(seg,st,Slots{asset});
 std::lock_guard<std::mutex> lock(c.mutex);if(!c.stations[k])c.stations[k]=built;return c.stations[k];
}
// Every station map of the asset (warm path; the first blast on a model otherwise builds the ones it touches).
inline void warmStations(const Asset&asset){std::size_t k=0;
 for(const auto&seg:asset.sever_segments)for(const auto&st:seg.stations)(void)stationMap(asset,k++,seg,st);}
// The intact body's publication: the static skin of every source child (fast->static_child, exactly the skin children
// packAnalyticCandidate publishes), no wound walls, no removal.
inline std::shared_ptr<const gfy_packed::Mesh> birthMesh(const Asset&asset){
 auto&c=cacheOf(asset);{std::lock_guard<std::mutex> lock(c.mutex);if(c.birth)return c.birth;}
 need(asset.fast&&asset.fast->valid,"instant_asset_static_skin");
 auto mesh=std::make_shared<gfy_packed::Mesh>();unsigned next=asset.original_children;
 for(const auto&source:asset.fast->static_child){mesh->reduced_skin_vertices+=source.reduced;mesh->max_discarded_skin_weight=std::max(mesh->max_discarded_skin_weight,source.max_discarded);
  for(std::size_t part=0;part<source.chunks.size();++part){auto child=source.chunks[part];
   if(part){need(next<gfy_packed::max_child_slots,"instant_child_capacity");child.child_index=std::uint16_t(next++);}
   child.skin_clip=false;mesh->children.push_back(std::move(child));}}
 mesh->tissue_field=asset.closure_field?asset.closure_field:asset.tissue_field;mesh->analytic_coverage_admitted=bool(asset.closure_field);
 separation_detail::pruneNegligibleSkinWeights(*mesh); // once: every piece copies these rows as they are
 std::lock_guard<std::mutex> lock(c.mutex);if(!c.birth)c.birth=std::move(mesh);return c.birth;
}
inline std::shared_ptr<const gfy_piece_collider::original_frames::Frames> framesOf(const Asset&asset,const SeparationInput&input){
 auto&c=cacheOf(asset);{std::lock_guard<std::mutex> lock(c.mutex);if(c.frames&&c.framesKey==input.bone_frames.expected)return c.frames;}
 auto frames=std::make_shared<const gfy_piece_collider::original_frames::Frames>(
  gfy_piece_collider::original_frames::decode(separation_detail::read(input.bone_frames,input.hash),asset.source_hash,input.fit_hash));
 need(frames->bones.size()==asset.source.bones,"separation_original_bone_count");
 std::lock_guard<std::mutex> lock(c.mutex);c.frames=frames;c.framesKey=input.bone_frames.expected;return frames;
}
inline std::shared_ptr<const ElementIndex> elementsOf(const Asset&asset,const SeparationInput&input,const gfy_piece_collider::original_frames::Frames&frames){
 const auto pristine=separation_detail::pristineReference(asset,input).topology;need(bool(pristine),"instant_pristine_topology");
 const auto map=separation_detail::physicsMap(asset,pristine,input,frames);need(bool(map),"instant_physics_map");
 auto&c=cacheOf(asset);{std::lock_guard<std::mutex> lock(c.mutex);if(c.elements&&c.elements->map==map)return c.elements;}
 auto index=buildElementIndex(map);std::lock_guard<std::mutex> lock(c.mutex);c.elements=index;return index;
}
// Original children with a partitioned surface (source children + accessory surfaces), as prepare() reports them.
inline std::vector<std::uint16_t> surfaceChildrenOf(const Asset&asset,const SeparationInput&input){
 auto&c=cacheOf(asset);{std::lock_guard<std::mutex> lock(c.mutex);if(c.accessoryRead&&c.accessoryKey==input.accessory_surfaces.expected)return c.surfaceChildren;}
 std::set<std::uint16_t> children(asset.face_children.begin(),asset.face_children.end());
 if(!input.accessory_surfaces.path.empty()){const auto extra=separation_detail::readAccessorySurfaces(asset,separation_detail::read(input.accessory_surfaces,input.hash));
  children.insert(extra.face_children.begin(),extra.face_children.end());}
 std::vector<std::uint16_t> out(children.begin(),children.end());
 std::lock_guard<std::mutex> lock(c.mutex);c.surfaceChildren=out;c.accessoryKey=input.accessory_surfaces.expected;c.accessoryRead=true;return out;
}
inline std::shared_ptr<const std::vector<std::vector<std::uint32_t>>> faceTableOf(const Asset&asset){
 auto&c=cacheOf(asset);{std::lock_guard<std::mutex> lock(c.mutex);if(c.faceOfChild)return c.faceOfChild;}
 auto table=std::make_shared<std::vector<std::vector<std::uint32_t>>>(asset.original_children);
 for(std::size_t f=0;f<asset.face_children.size()&&f<asset.face_indices.size();++f){const auto child=asset.face_children[f];const auto local=asset.face_indices[f];if(child>=table->size())continue;
  auto&list=(*table)[child];if(list.size()<=local)list.resize(std::size_t(local)+1,~std::uint32_t(0));list[local]=std::uint32_t(f);}
 std::lock_guard<std::mutex> lock(c.mutex);if(!c.faceOfChild)c.faceOfChild=table;return c.faceOfChild;
}
inline std::uint64_t superKey(const Cell&tile){return (std::uint64_t(std::uint32_t(tile.x>>3))<<42)|(std::uint64_t(std::uint32_t(tile.y>>3))<<21)|std::uint64_t(std::uint32_t(tile.z>>3));}
inline std::shared_ptr<const std::unordered_set<std::uint64_t>> superTilesOf(const Asset&asset){
 if(!asset.tile_material)return {};auto&c=cacheOf(asset);{std::lock_guard<std::mutex> lock(c.mutex);if(c.superTiles)return c.superTiles;}
 auto set=std::make_shared<std::unordered_set<std::uint64_t>>();for(const auto&t:*asset.tile_material)set->insert(superKey(t.first));
 std::lock_guard<std::mutex> lock(c.mutex);if(!c.superTiles)c.superTiles=set;return c.superTiles;
}
inline std::shared_ptr<const sep::SourceFaceIndex> sourceFacesOf(const Asset&asset){
 auto&c=cacheOf(asset);{std::lock_guard<std::mutex> lock(c.mutex);if(c.sourceFaces)return c.sourceFaces;}
 auto index=std::make_shared<const sep::SourceFaceIndex>(asset.face_children,asset.face_indices,asset.source.f.size());
 std::lock_guard<std::mutex> lock(c.mutex);if(!c.sourceFaces)c.sourceFaces=index;return c.sourceFaces;
}
// Everything an instant split of this model reads besides the snapshot, built once (the warm path at bind; otherwise
// the first blast on the model pays it): station maps, bone frames, element index, surface children, the intact
// publication and the source face index.
inline void warmInstant(const Asset&asset,const SeparationInput&input){
 if(asset.sever_segments.empty())return;
 warmStations(asset);(void)birthMesh(asset);(void)faceTableOf(asset);(void)superTilesOf(asset);
 if(input.physical_parents.size()==asset.source.bones&&!input.bone_frames.path.empty()){const auto frames=framesOf(asset,input);(void)elementsOf(asset,input,*frames);(void)surfaceChildrenOf(asset,input);}
}
// ---- impacts -> the worker's rest-space cutters -------------------------------------------------------------------
struct Crater {Tunnel cut;Vec a,b,lo,hi;double maxR{};std::uint64_t sequence{},edgeSeed{};std::uint32_t blade{};Vec mapped;bool blast{};std::uint16_t bone{};double radius{},penetration{};};
inline bool identityMatrix(const Matrix&m){for(unsigned i=0;i<16;++i)if(m.row_major[i]!=((i%5)==0?1.:0.))return false;return true;}
inline bool rayBox(const core::Bounds&b,const Vec&p,const Vec&d,double tmax){double lo=0,hi=tmax;
 for(int a=0;a<3;++a){if(std::abs(d[a])<1e-14){if(p[a]<b.lo[a]||p[a]>b.hi[a])return false;continue;}
  double t0=(b.lo[a]-p[a])/d[a],t1=(b.hi[a]-p[a])/d[a];if(t0>t1)std::swap(t0,t1);lo=std::max(lo,t0);hi=std::min(hi,t1);if(hi<lo)return false;}return true;}
// sourceRay's rule in rest space: among the front-facing source faces the ray start+t*d crosses for t in [0, 2*search],
// the one nearest the accepted point (t = search); ties go to the lowest face. The BVH only culls.
inline bool sourceHit(const Asset&asset,const Vec&start,const Vec&d,double search,double&tHit){
 const auto&tree=asset.source_tree;const auto&s=asset.source;if(tree.nodes.empty())return false;
 double best=1e100;std::uint32_t bestFace=~0u;double bestT=0;int stack[128];int top=0;stack[top++]=0;
 while(top){const auto&node=tree.nodes[std::size_t(stack[--top])];if(!rayBox(node.bounds,start,d,2*search))continue;
  if(node.left>=0){if(top+2>128)return false;stack[top++]=node.left;stack[top++]=node.right;continue;}
  for(std::size_t i=node.first;i<node.first+node.count;++i){const std::uint32_t f=tree.order[i];const auto ids=s.f[f];
   const Vec a=s.v[ids[0]].p,e1=s.v[ids[1]].p-a,e2=s.v[ids[2]].p-a,h=cross(d,e2);const double det=dot(e1,h);if(std::abs(det)<1e-14)continue;
   const Vec ss=start-a;const double u=dot(ss,h)/det;const Vec q=cross(ss,e1);const double v=dot(d,q)/det,t=dot(e2,q)/det;
   if(u< -1e-8||v< -1e-8||u+v>1+1e-8||t<0||t>2*search)continue;
   const Vec restN=cross(e1,e2),normal=s.v[ids[0]].normal+s.v[ids[1]].normal+s.v[ids[2]].normal;
   const double orientation=dot(restN,normal)>=0?1:-1;if(dot(restN,d)*orientation>=0)continue;
   const double distance=std::abs(t-search);if(distance<best||(distance==best&&f<bestFace)){best=distance;bestFace=f;bestT=t;}}}
 if(bestFace==~0u)return false;tHit=bestT;return true;
}
// firstSolid / firstSolidCylinder with the cells this batch's earlier cutters removed read as removed (the worker carves
// the craters one after another, so a later crater starts below an earlier one). Solid cells come from the model's
// tile masks (SeverState::table, exactly solidBase on an unowned grid) and a tile holding no tissue is crossed in one
// step; the cell found is the one firstSolid finds.
struct MarchStats {std::size_t steps{},jumps{},skips{},fallbacks{},rays{};};inline MarchStats& marchStats(){thread_local MarchStats s;return s;}
struct SolidTiles {
 const Grid&g;const std::unordered_map<Cell,SeverState::TileMaterial,core::CellHash>*table;Cell key{INT_MIN,INT_MIN,INT_MIN};const SeverState::TileMaterial*m=nullptr;
 // 0: the tile holds no tissue; 1: c is solid; 2: c is not solid
 int at(const Cell&c){if(!table)return g.solidBase(c)?1:2;const Cell k=Grid::tileKey(c);
  if(!(k==key)){key=k;const auto it=table->find(k);m=it==table->end()?nullptr:&it->second;}
  if(!m)return 0;const auto l=Grid::local(c);return ((m->solid[l/64]>>(l%64))&1u)?1:2;}
};
struct Crater; // below
// Where the ray o + s d leaves the core of an earlier cutter k that holds p (radius x the edge minimum, inlet to the
// tip cap, shrunk by 1.5 cells, so every cell centre the ray crosses until then is inside k): the march resumes there.
// s0 when p is inside k but not in that core.
inline double coreExit(const Tunnel&k,const Vec&p,const Vec&o,const Vec&d,double pitch){
 const double margin=1.5*pitch;const double s0=dot(p-o,d);
 const double inner=k.radius*(k.edge.enabled()?WoundEdgeProfile::minimum_radius:1.)-margin;if(!(inner>0))return s0;
 const double tLo=-k.inlet+margin,tHi=(k.exit_distance<0?k.penetration-k.cap:k.penetration)-margin;if(!(tHi>tLo))return s0;
 const Vec q0=o-k.entry;const double a0=dot(q0,k.direction),a1=dot(d,k.direction);const Vec w0=q0-k.direction*a0,w1=d-k.direction*a1;
 double lo=-1e300,hi=1e300;
 if(std::abs(a1)>1e-12){double sa=(tLo-a0)/a1,sb=(tHi-a0)/a1;if(sa>sb)std::swap(sa,sb);lo=sa;hi=sb;}else if(a0<=tLo||a0>=tHi)return s0;
 const double A=dot(w1,w1),B=2*dot(w0,w1),C=dot(w0,w0)-inner*inner;
 if(A>1e-18){const double disc=B*B-4*A*C;if(disc<=0)return s0;const double r=std::sqrt(disc);lo=std::max(lo,(-B-r)/(2*A));hi=std::min(hi,(-B+r)/(2*A));}else if(C>=0)return s0;
 if(!(s0>=lo&&s0<hi))return s0;return std::max(s0,hi);
}
// Earlier cutters of the batch: each tunnel with its bounds (they are read as removed while marching).
struct Prior {const Tunnel*cut;Vec lo,hi;};
// firstSolid along o + s d up to sMax, with the earlier cutters whose bounds the ray meets read as removed.
inline March firstSolidMasked(const Grid&g,const std::unordered_map<Cell,SeverState::TileMaterial,core::CellHash>*table,Vec origin,Vec direction,
    const std::vector<Prior>&prior,const std::unordered_set<std::uint64_t>*super,double sMax=1e300){
 direction=unit(direction);double enter=0,leave=1e6;
 for(int a=0;a<3;++a){double low=g.origin[a],high=low+g.size[a]*g.pitch;if(std::abs(direction[a])<1e-14){if(origin[a]<low||origin[a]>=high)return {};continue;}double t0=(low-origin[a])/direction[a],t1=(high-origin[a])/direction[a];if(t0>t1)std::swap(t0,t1);enter=std::max(enter,t0);leave=std::min(leave,t1);}
 leave=std::min(leave,sMax);if(enter>=leave)return {};
 // the cutters this ray can meet (slab test of their bounds)
 const Tunnel*near[64];std::size_t nearCount=0;std::vector<const Tunnel*> more;
 for(const auto&c:prior){double lo=enter,hi=leave;bool hit=true;
  for(int a=0;a<3&&hit;++a){if(std::abs(direction[a])<1e-14){hit=origin[a]>=c.lo[a]&&origin[a]<=c.hi[a];continue;}
   double t0=(c.lo[a]-origin[a])/direction[a],t1=(c.hi[a]-origin[a])/direction[a];if(t0>t1)std::swap(t0,t1);lo=std::max(lo,t0);hi=std::min(hi,t1);hit=lo<=hi;}
  if(!hit)continue;if(nearCount<64)near[nearCount++]=c.cut;else more.push_back(c.cut);}
 SolidTiles solid{g,table};core::TileCache removed(g);
 double t=enter;
 for(int restart=0;restart<100000;++restart){
  Vec p=origin+direction*(t+g.pitch*1e-7);Cell c;std::array<int,3> step{};std::array<double,3> next{},delta{};
  for(int a=0;a<3;++a){(&c.x)[a]=std::clamp(int(std::floor((p[a]-g.origin[a])/g.pitch)),0,g.size[a]-1);step[a]=direction[a]>0?1:-1;if(std::abs(direction[a])<1e-14){next[a]=delta[a]=1e100;}else{double plane=g.origin[a]+(c[a]+(step[a]>0?1:0))*g.pitch;next[a]=(plane-origin[a])/direction[a];delta[a]=g.pitch/std::abs(direction[a]);}}
  const int maxsteps=g.size.x+g.size.y+g.size.z+6;bool skipped=false;
  ++marchStats().rays;
  for(int n=0;n<maxsteps&&g.valid(c)&&t<leave;++n){const int s=solid.at(c);++marchStats().steps;
   if(s==0){ // an empty tile: leave it at once (its exit face, parametric along the ray)
    double exit=1e300;const Cell k=Grid::tileKey(c);const bool wide=super&&!super->count(superKey(k)); // a whole empty 64-cell block
    for(int a=0;a<3;++a){if(std::abs(direction[a])<1e-14)continue;const int first=wide?((&k.x)[a]>>3)*64:(&k.x)[a]*8,span=wide?64:8;
     const double face=g.origin[a]+(first+(direction[a]>0?span:0))*g.pitch;exit=std::min(exit,(face-origin[a])/direction[a]);}
    if(exit>=leave)return {};t=std::max(t,exit);skipped=true;++marchStats().skips;break;}
   if(s==1&&!removed.removed(c)){const Vec centre=g.center(c);double resume=-1;
    for(std::size_t q=0;q<nearCount+more.size()&&resume<0;++q){const Tunnel*k=q<nearCount?near[q]:more[q-nearCount];if(k->contains(centre))resume=coreExit(*k,centre,origin,direction,g.pitch);}
    if(resume<0)return {true,origin+direction*t,c};
    if(resume>t+g.pitch){t=resume;skipped=true;++marchStats().jumps;break;}}
   double target=std::min({next[0],next[1],next[2]});if(target>=leave)return {};for(int a=0;a<3;++a)if(next[a]<=target+1e-11){(&c.x)[a]+=step[a];next[a]+=delta[a];}t=target;}
  if(!skipped)return {};}
 return {};
}
// firstSolidCylinder: the axis first, else the nearest hit of 36 parallel rays at .35/.7/1 radius (the entry projected
// on the axis). A ring ray stops where it can no longer beat the nearest hit so far.
inline March firstSolidCylinderMasked(const Grid&g,const std::unordered_map<Cell,SeverState::TileMaterial,core::CellHash>*table,Vec origin,Vec direction,double radius,
    const std::vector<Prior>&prior,const std::unordered_set<std::uint64_t>*super){
 direction=unit(direction);auto axis=firstSolidMasked(g,table,origin,direction,prior,super);if(axis.found||radius<=0)return axis;
 ++marchStats().fallbacks;
 const Vec ref=std::abs(direction.x)<.8?Vec{1,0,0}:Vec{0,1,0};const Vec u=unit(cross(direction,ref)),v=cross(direction,u);
 double best=std::numeric_limits<double>::infinity();March out;
 for(double f:{.35,.7,1.})for(int k=0;k<12;++k){const double a=tau*k/12;const Vec off=(u*std::cos(a)+v*std::sin(a))*(radius*f);
  auto m=firstSolidMasked(g,table,origin+off,direction,prior,super,best+g.pitch);if(!m.found)continue;const double t=dot(m.entry-(origin+off),direction);if(t<best){best=t;out=m;}}
 if(!std::isfinite(best))return axis;
 out.entry=origin+direction*best;return out;
}
// The cutters the worker carves for these impacts (blast craters and bullet bores: the primary channel; a legacy
// sphere, a bore that maps to no surface or finds no tissue carves nothing and is skipped, as there).
inline std::vector<Crater> resolveCraters(const Asset&asset,const Grid&grid,const std::vector<Impact>&impacts){
 std::vector<Crater> out;out.reserve(impacts.size());double tSource=0,tMarch=0,tEnvelope=0,tProfile=0;ProfClock pc;marchStats()=MarchStats{};
 std::vector<Prior> prior;prior.reserve(impacts.size());const auto superTiles=superTilesOf(asset);
 for(const auto&e:impacts){
  if(!(e.penetration_m>0&&e.penetration_m<=.5&&e.radius>=.0005&&e.radius<=.08&&e.exit_diameter_mult>=1&&e.exit_diameter_mult<=2))continue;
  const Vec anchorPosed=vec(e.accepted_pose_point),dirPosed=vec(e.accepted_pose_direction);
  if(!std::isfinite(anchorPosed.x)||!std::isfinite(anchorPosed.y)||!std::isfinite(anchorPosed.z)||!(length(dirPosed)>1e-9))continue;
  Vec anchor=anchorPosed,d=unit(dirPosed);
  // Posed -> rest through the hit bone's render transform (the crater's zone bone; identity in the T pose). The worker
  // maps through the blended matrix of the face it hits, which is this bone's on the zone it aims at.
  if(e.hit_bone<e.bone_model_render.size()&&!identityMatrix(e.bone_model_render[e.hit_bone])){const auto&m=e.bone_model_render[e.hit_bone];const auto&r=m.row_major;
   try{anchor=inverseDirection(m,anchorPosed-Vec{r[3],r[7],r[11]});d=unit(inverseDirection(m,d));}catch(const std::bad_alloc&){throw;}catch(...){continue;}}
  const double search=e.source_search_distance;if(!(std::isfinite(search)&&search>0&&search<=.5))continue;
  const Vec start=anchor-d*search;double t=0;pc.lap();const bool hit=sourceHit(asset,start,d,search,t);tSource+=pc.lap();if(!hit)continue;
  const Vec entry=start+d*t;bool inside=true;
  for(int a=0;a<3;++a)inside=inside&&entry[a]>=grid.origin[a]-grid.pitch&&entry[a]<=grid.origin[a]+grid.size[a]*grid.pitch+grid.pitch;
  if(!inside)continue;
  const Vec origin=entry-d*(grid.pitch*2+e.radius);
  const auto march=firstSolidCylinderMasked(grid,asset.tile_material.get(),origin,d,e.radius,prior,superTiles.get());tMarch+=pc.lap();if(!march.found)continue;
  Crater c;c.sequence=e.sequence;c.blade=e.blade;c.blast=e.blast;c.bone=e.hit_bone;c.mapped=entry;c.radius=e.radius;c.penetration=e.penetration_m;
  c.edgeSeed=e.penetration_m>0&&!asset.face_children.empty()&&!e.ammo_section.empty()?(e.ammo_seed?e.ammo_seed:e.sequence):0;
  Tunnel cut{march.entry,d,e.radius,e.penetration_m,std::min(e.radius,e.penetration_m)};cut.setEdgeProfile(c.edgeSeed);tProfile+=pc.lap();
  const auto hits=asset.envelope_tree.crossingDistances(march.entry,d);tEnvelope+=pc.lap();const std::size_t exitIndex=hits.size()%2?0:1;
  if(exitIndex<hits.size()&&e.penetration_m>=hits[exitIndex]){cut.exit_distance=hits[exitIndex];cut.exit_mult=e.exit_diameter_mult;}
  cut.inlet=std::max(0.,dot(march.entry-origin,d));cut.setForensics(e.cavitation,e.burst,c.edgeSeed?c.edgeSeed:0x5EED1234ull);
  c.cut=cut;c.maxR=cut.maxRadius();c.a=cut.entry-d*cut.inlet;c.b=cut.entry+d*cut.penetration;
  for(int a=0;a<3;++a){(&c.lo.x)[a]=std::min(c.a[a],c.b[a])-c.maxR;(&c.hi.x)[a]=std::max(c.a[a],c.b[a])+c.maxR;}
  out.push_back(std::move(c));prior.push_back({&out.back().cut,out.back().lo,out.back().hi});} // out never reallocates (reserved)
 if(profiling())profileText()<<"instant_prof march_counts rays="<<marchStats().rays<<" steps="<<marchStats().steps<<" jumps="<<marchStats().jumps<<" skips="<<marchStats().skips<<" fallbacks="<<marchStats().fallbacks<<'\n';
 if(profiling())profileText()<<"instant_prof resolve source="<<tSource<<" march="<<tMarch<<" profile="<<tProfile<<" envelope="<<tEnvelope<<" craters="<<out.size()<<'\n';
 return out;
}
// Tunnel::classifyBox for the ball (centre c, radius r): 1 every point inside the cutter, -1 none, 0 undecided.
inline int classifyBall(const Tunnel&t,const Vec&c,double r){
 const Vec q=c-t.entry;const double tc=dot(q,t.direction);const double te=r+1e-9;const double tlo=tc-te,thi=tc+te;
 if(thi< -t.inlet||tlo>t.penetration)return -1;
 const double e=r+1e-9;const Vec radial=q-t.direction*tc;const double rc=length(radial);const double nearD=std::max(0.,rc-e),farD=rc+e;
 const double coreEnd=t.exit_distance<0?t.penetration-t.cap:t.penetration;const bool inCore=tlo> -t.inlet&&thi<coreEnd;
 double rlo=t.radiusMinOver(tlo,thi),rhi=t.radiusMaxOver(tlo,thi);const double m=t.edge.enabled()?WoundEdgeProfile::minimum_radius:1.;
 if(inCore&&farD<rlo*m*(1-1e-9))return 1;
 double cl=1,ch=1;const bool angular=rc>e*1.5;Vec qa,qb;
 if(angular){const Vec tangent=cross(t.direction,radial)*(e/std::sqrt(rc*rc-e*e));qa=radial-tangent;qb=radial+tangent;}
 if(t.cracks>0&&thi>=t.exit_distance-t.burst_len){
  if(angular)ch=t.crackBound(qa,qb,thi,tlo,cl);
  else{const double ramp=std::clamp((thi-(t.exit_distance-t.burst_len))/t.burst_len,0.,1.);ch=1+t.burst_amp*ramp*ramp;}}
 rlo*=cl;rhi*=ch;
 if(nearD>rhi*(1+1e-9))return -1;
 if(inCore&&farD<rlo*m*(1-1e-9))return 1;
 if(t.edge.enabled()&&angular){double el,eh;
  if(t.edge.arcDepthBounds(dot(qa,t.edge_u),dot(qa,t.edge_v),dot(qb,t.edge_u),dot(qb,t.edge_v),
    std::clamp(tlo/t.penetration-1e-9,0.,1.),std::clamp(thi/t.penetration+1e-9,0.,1.),el,eh)){
   if(nearD>rhi*eh*(1+1e-9))return -1;
   if(inCore&&farD<rlo*el*(1-1e-9))return 1;}}
 return 0;
}
// A station's removed tissue/bone counted at the damage lattice, exactly as the voxel path measures it after the carve
// (measureStationTile: anatomy cells of the slab, removed = committed damage or the centre inside a cutter), for a
// station whose sampled values lie next to a threshold. Tiles a cutter certainly contains or misses are decided whole.
template<class CutterList>
inline void exactStation(const Asset&asset,const Grid&g,const SeverSegment&seg,const SeverStation&st,const CutterList&cutters,double&tissueLeft,double&boneLeft){
 const auto*table=asset.tile_material.get();core::TileCache removed(g);const double cell=g.pitch*g.pitch*g.pitch;
 double cutT=0,cutB=0;const double reach=std::sqrt(3.)*4*g.pitch;
 for(int tx=st.tileLo.x;tx<=st.tileHi.x;++tx)for(int ty=st.tileLo.y;ty<=st.tileHi.y;++ty)for(int tz=st.tileLo.z;tz<=st.tileHi.z;++tz){const Cell key{tx,ty,tz};
  const Vec centre=g.origin+Vec{(tx*8+4)*g.pitch,(ty*8+4)*g.pitch,(tz*8+4)*g.pitch};const Vec d0=centre-st.center;const double a0=dot(d0,seg.axis);
  if(std::abs(a0)>seg.halfWidth+reach||dot(d0,d0)-a0*a0>(seg.radius+reach)*(seg.radius+reach))continue;
  SeverState::TileMaterial local{};const SeverState::TileMaterial*m=nullptr;
  if(table){const auto it=table->find(key);if(it==table->end())continue;m=&it->second;}
  else{for(unsigned l=0;l<512;++l){const Cell c{tx*8+int(l>>6),ty*8+int((l>>3)&7),tz*8+int(l&7)};const auto label=g.rawMaterialBase(c);if(!label)continue;
    local.solid[l/64]|=std::uint64_t(1)<<(l%64);if(g.material_slots&&label<g.material_slots->size()&&(*g.material_slots)[label]==2)local.bone[l/64]|=std::uint64_t(1)<<(l%64);}m=&local;}
  int cls=-1;const Crater*open[16];std::size_t openCount=0;
  for(const auto*cr:cutters){const int r=classifyBall(cr->cut,centre,reach);if(r==1){cls=1;break;}if(r==0){cls=0;if(openCount<16)open[openCount++]=cr;else{openCount=17;break;}}}
  const core::Tile*done=removed.find(key); // committed damage of the tile
  if(cls==-1&&!done)continue;
  // the undecided cutters of each 4^3 sub-cube (index (x/4)*4+(y/4)*2+z/4): 1 all inside, -1 none, 0 test cells
  int sub[8];const Crater*subOpen[8][16];std::size_t subCount[8]={};
  if(cls==0&&openCount<=16)for(int q=0;q<8;++q){const Vec sc=g.origin+Vec{(tx*8+(q>>2)*4+2)*g.pitch,(ty*8+((q>>1)&1)*4+2)*g.pitch,(tz*8+(q&1)*4+2)*g.pitch};sub[q]=-1;
   for(std::size_t o=0;o<openCount;++o){const int r=classifyBall(open[o]->cut,sc,std::sqrt(3.)*2*g.pitch);if(r==1){sub[q]=1;break;}if(r==0){sub[q]=0;subOpen[q][subCount[q]++]=open[o];}}}
  // Per column (x,y) of the tile the slab's cells form one z interval: axial |s| <= halfWidth is linear in z and the
  // radial bound quadratic. Cells in it: bit masks; only a tile a cutter may partly hold tests centres.
  for(int x=0;x<8;++x)for(int y=0;y<8;++y){const unsigned shift=unsigned(y*8);const unsigned column=unsigned((m->solid[std::size_t(x)]>>shift)&255u);if(!column)continue;
   const Vec p0=g.center(Cell{tx*8+x,ty*8+y,tz*8})-st.center;const double a0z=dot(p0,seg.axis),az=seg.axis.z*g.pitch;
   int z0=0,z1=7;
   if(std::abs(az)>1e-12){double lo=(-seg.halfWidth-a0z)/az,hi=(seg.halfWidth-a0z)/az;if(lo>hi)std::swap(lo,hi);z0=std::max(z0,int(std::ceil(lo-1e-9)));z1=std::min(z1,int(std::floor(hi+1e-9)));}
   else if(std::abs(a0z)>seg.halfWidth)continue;
   if(z0>z1)continue;
   // radial^2(z) = |p0 + e_z h z|^2 - (a0z + az z)^2: a quadratic A z^2 + B z + C <= R^2
   {const double A=g.pitch*g.pitch-az*az,B=2*(p0.z*g.pitch-a0z*az),C=dot(p0,p0)-a0z*a0z-seg.radius*seg.radius;
    if(A>1e-18){const double disc=B*B-4*A*C;if(disc<0)continue;const double r=std::sqrt(disc);z0=std::max(z0,int(std::ceil((-B-r)/(2*A)-1e-9)));z1=std::min(z1,int(std::floor((-B+r)/(2*A)+1e-9)));}
    else if(C>0)continue;}
   if(z0>z1)continue;const unsigned range=((1u<<unsigned(z1+1))-1u)&~((1u<<unsigned(z0))-1u);
   const unsigned tissue=column&range,bone=unsigned((m->bone[std::size_t(x)]>>shift)&255u)&tissue;if(!tissue)continue;
   unsigned gone=done?unsigned((done->removed[std::size_t(x)]>>shift)&255u)&tissue:0u;
   if(cls==1)gone=tissue;
   else if(cls==0)for(unsigned rest=tissue&~gone;rest;rest&=rest-1){const unsigned z=core::lowestBit(rest);const Vec p=g.center(Cell{tx*8+x,ty*8+y,tz*8+int(z)});
    if(openCount>16){for(std::size_t q=0;q<cutters.size();++q)if(cutters[q]->cut.contains(p)){gone|=1u<<z;break;}continue;}
    const int q=(x>>2)*4+(y>>2)*2+int(z>>2);if(sub[q]==1){gone|=1u<<z;continue;}if(sub[q]==-1)continue;
    for(std::size_t o=0;o<subCount[q];++o)if(subOpen[q][o]->cut.contains(p)){gone|=1u<<z;break;}}
   cutT+=cell*double(std::bitset<8>(gone).count());cutB+=cell*double(std::bitset<8>(gone&bone).count());}}
 tissueLeft=std::clamp(1-cutT/st.intactTissue,0.,1.);boneLeft=st.intactBone>1e-7?std::clamp(1-cutB/st.intactBone,0.,1.):tissueLeft;
}
inline double pointSegment(const Vec&p,const Vec&a,const Vec&b){const Vec ab=b-a;const double L2=dot(ab,ab);const double t=L2>1e-18?std::clamp(dot(p-a,ab)/L2,0.,1.):0.;return length(p-(a+ab*t));}
// ---- the station verdict ------------------------------------------------------------------------------------------
struct Failing {const SeverSegment*seg{};const SeverStation*st{};std::size_t k{};double tissue{1},bone{1};bool dangling{},guided{};Vec guideNormal;};
struct Verdict {std::vector<Failing> failing,picks;std::size_t measured{},exact{},samples{},blocks{},mixed{},quarters{},cells{};};
// Removed tissue/bone of every station slab a cutter reaches (committed damage included), measured on the station's
// sample map in two passes. Bounds: whole 16 mm groups and 8 mm blocks a cutter certainly contains or misses are
// decided at once; DamageTear's classes are boxes in (tissue, bone) left, so when both bounds fall in one class the
// station is decided. Exact: a station still undecided is sampled per quarter and sample, and one next to a class
// boundary is counted at the damage cells as the voxel path counts it - unless a decided failing station of its segment
// lies within one limb radius, where its value cannot change the verdict (that cluster tears either way; the middle of
// its bounds orders the picks). Then blade guidance and the one-limb-radius clustering in the blast lanes' order.
inline Verdict stationVerdict(const Asset&asset,const Grid&grid,const SeverState*state,const std::vector<Crater>&craters,const TearGuide&guide,
    double severScale,bool requireFail){
 Verdict v;if(asset.sever_segments.empty()||craters.empty())return v;
 const double T=std::clamp(.45-.1*(severScale-1),.25,.6);
 auto classOf=[&](double t,double b){return (t<=T&&b<=.4)?2:((b<=.25&&t<=T+.15)?1:0);};
 // Tiles of the committed grid holding removed cells (a station with any of them in its box counts that damage too).
 std::vector<Cell> damaged;for(const auto&t:grid.tiles){std::uint64_t any=0;for(auto w:t.second.removed)any|=w;if(any)damaged.push_back(t.first);}
 core::TileCache removed(grid);double cycBlocks=0,cycSamples=0,cycCells=0;ProfClock vclock;
 auto cellOf=[&](const Vec&p){return Cell{int(std::floor((p.x-grid.origin.x)/grid.pitch)),int(std::floor((p.y-grid.origin.y)/grid.pitch)),int(std::floor((p.z-grid.origin.z)/grid.pitch))};};
 struct Mixed {const StationMap::Block*b;std::array<const Crater*,8> open;unsigned count;};
 struct Eval {const SeverSegment*seg;const SeverStation*st;std::size_t k;std::shared_ptr<const StationMap> map;std::vector<const Crater*> nearby;std::vector<Mixed> mixed;
  bool prior;std::size_t cutT,cutB;double tlMin,blMin,tlMax,blMax;int lowClass,highClass;double tissue{1},bone{1};};
 std::vector<Eval> evals;
 // ---- pass 1: bounds
 std::size_t k=0;
 for(const auto&seg:asset.sever_segments){
  std::vector<const Crater*> segCraters;const Vec s0=seg.pivot+seg.axis*seg.stations.front().s,s1=seg.pivot+seg.axis*seg.stations.back().s;
  for(const auto&c:craters){const double reach=seg.radius+c.maxR+.02;
   double dmin=1e30;for(int i=0;i<=8;++i){const Vec p=s0+(s1-s0)*(i/8.);dmin=std::min(dmin,pointSegment(p,c.a,c.b));}
   if(dmin<=reach+length(s1-s0)/16+1e-9)segCraters.push_back(&c);}
  for(const auto&st:seg.stations){const std::size_t index=k++;
   if(segCraters.empty())continue;if(state&&index<state->failed.size()&&state->failed[index])continue;
   Eval e;e.seg=&seg;e.st=&st;e.k=index;
   for(const auto*c:segCraters)if(pointSegment(st.center,c->a,c->b)<=seg.radius+c->maxR+.02)e.nearby.push_back(c);
   if(e.nearby.empty())continue;
   e.prior=false;for(const Cell&t:damaged)if(t.x>=st.tileLo.x&&t.x<=st.tileHi.x&&t.y>=st.tileLo.y&&t.y<=st.tileHi.y&&t.z>=st.tileLo.z&&t.z<=st.tileHi.z){e.prior=true;break;}
   e.map=stationMap(asset,index,seg,st);const auto&m=*e.map;if(!m.tissue)continue;++v.measured;v.blocks+=m.blocks.size();
   std::size_t cutT=0,cutB=0,openT=0,openB=0;
   vclock.lap();const double rb=m.radius(),rg=m.groupRadius();std::vector<const Crater*> groupOpen;
   for(const auto&g:m.groups){int gcls=-1;const Vec gc=m.groupCenter(g);groupOpen.clear();
    for(const auto*cr:e.nearby){const int r=classifyBall(cr->cut,gc,rg);if(r==1){gcls=1;break;}if(r==0){gcls=0;groupOpen.push_back(cr);}}
    if(gcls==1){cutT+=g.tissueCount;cutB+=g.boneCount;continue;}
    if(gcls==-1&&!e.prior)continue;
    const auto&candidates=e.prior?e.nearby:groupOpen;
    for(std::uint32_t bi=g.first;bi<g.first+g.count;++bi){const auto&b=m.blocks[bi];int cls=-1;const Vec c=m.center(b);Mixed mx{&b,{},0};
     for(const auto*cr:candidates){const int r=classifyBall(cr->cut,c,rb);if(r==1){cls=1;break;}if(r==0){cls=0;if(mx.count<8)mx.open[mx.count]=cr;++mx.count;}}
     if(cls==1){cutT+=b.tissueCount;cutB+=b.boneCount;continue;}
     if(cls==-1&&!e.prior)continue;
     if(mx.count>8||e.prior){mx.count=0;for(const auto*cr:e.nearby){if(mx.count==8){mx.count=9;break;}mx.open[mx.count++]=cr;}} // test every nearby cutter
     e.mixed.push_back(mx);openT+=b.tissueCount;openB+=b.boneCount;}}
   cycBlocks+=vclock.lap();
   auto left=[&](std::size_t cutTissue,std::size_t cutBone,double&tl,double&bl){tl=1-double(cutTissue)/m.tissue;bl=m.bone?1-double(cutBone)/m.bone:tl;};
   left(cutT+openT,cutB+openB,e.tlMin,e.blMin);left(cutT,cutB,e.tlMax,e.blMax);
   e.lowClass=classOf(e.tlMin,e.blMin);e.highClass=classOf(e.tlMax,e.blMax);
   if(requireFail&&e.lowClass==0)continue; // cannot fail whatever the undecided samples hold
   e.cutT=cutT;e.cutB=cutB;evals.push_back(std::move(e));}}
 // ---- pass 2: exact where it can change the verdict
 for(auto&e:evals){const auto&m=*e.map;
  e.tissue=.5*(e.tlMin+e.tlMax);e.bone=.5*(e.blMin+e.blMax);
  if(requireFail&&e.lowClass==e.highClass)continue; // decided
  bool settled=false;
  if(requireFail)for(const auto&f:evals)if(&f!=&e&&f.seg==e.seg&&f.lowClass==f.highClass&&f.lowClass>0&&std::abs(f.st->s-e.st->s)<=e.seg->radius){settled=true;break;}
  if(settled)continue;
  vclock.lap();std::size_t cutT=e.cutT,cutB=e.cutB;v.mixed+=e.mixed.size();if(!e.mixed.empty())++v.exact;const double rq=m.quarterRadius();
  // Sampling stops once the cut so far plus the blocks still unsampled decide the class either way.
  std::size_t restT=0,restB=0;for(const auto&mx:e.mixed){restT+=mx.b->tissueCount;restB+=mx.b->boneCount;}bool decided=false;
  if(requireFail)std::stable_sort(e.mixed.begin(),e.mixed.end(),[](const Mixed&a,const Mixed&b){return a.b->tissueCount>b.b->tissueCount;}); // big blocks decide soonest
  for(const auto&mx:e.mixed){const auto*b=mx.b;
   if(requireFail){double tl,bl,th,bh;const double mt=double(m.tissue),mb=double(m.bone);
    tl=1-double(cutT+restT)/mt;bl=m.bone?1-double(cutB+restB)/mb:tl;th=1-double(cutT)/mt;bh=m.bone?1-double(cutB)/mb:th;
    // decided only with the bounds clear of every threshold by the cell-count margin (inside it the slab's cells decide)
    if(classOf(tl-.01,bl-.01)==classOf(th+.01,bh+.01)){e.tissue=.5*(tl+th);e.bone=.5*(bl+bh);decided=true;break;}}
   restT-=b->tissueCount;restB-=b->boneCount;
   auto all=[&](std::size_t q)->const Crater*{return mx.count>8?e.nearby[q]:mx.open[q];};const std::size_t candidates=mx.count>8?e.nearby.size():std::min<std::size_t>(mx.count,8);
   for(unsigned l=0;l<4;++l)for(unsigned qy=0;qy<2;++qy)for(unsigned qx=0;qx<2;++qx){
    const std::uint64_t bits=StationMap::quarterBits(l,qx,qy)&b->tissue;if(!bits)continue;
    int cls=-1;const Vec c=m.quarter(*b,l,qx,qy);++v.quarters;
    for(std::size_t q=0;q<candidates;++q){const int r=classifyBall(all(q)->cut,c,rq);if(r==1){cls=1;break;}if(r==0)cls=0;}
    if(cls==1){cutT+=std::size_t(std::bitset<64>(bits).count());cutB+=std::size_t(std::bitset<64>(bits&b->bone).count());continue;}
    if(cls==-1&&!e.prior)continue;
    for(std::uint64_t w=bits;w;w&=w-1){const unsigned bit=core::lowestBit(w);const Vec p=m.sample(*b,bit);++v.samples;
     bool gone=e.prior&&grid.valid(cellOf(p))&&removed.removed(cellOf(p));
     for(std::size_t q=0;q<candidates&&!gone;++q)gone=all(q)->cut.contains(p);
     if(gone){++cutT;if((b->bone>>bit)&1u)++cutB;}}}}
  if(!decided){e.tissue=1-double(cutT)/m.tissue;e.bone=m.bone?1-double(cutB)/m.bone:e.tissue;}cycSamples+=vclock.lap();
  if(decided)continue;
  // Next to a class boundary the sampling cannot decide: count the slab's cells as the voxel path will.
  auto near=[](double value,double threshold){return std::abs(value-threshold)<.01;};
  if(requireFail&&(near(e.tissue,T)||near(e.tissue,T+.15)||near(e.bone,.4)||near(e.bone,.25))){const double st=e.tissue,sb=e.bone;
   exactStation(asset,grid,*e.seg,*e.st,e.nearby,e.tissue,e.bone);++v.cells;cycCells+=vclock.lap();
   if(profiling())profileText()<<"instant_exact bone="<<e.seg->bone<<" s="<<e.st->s<<" sampled_t="<<st<<" sampled_b="<<sb<<" cells_t="<<e.tissue<<" cells_b="<<e.bone<<'\n';}}
 for(const auto&e:evals){
  const bool tear=e.tissue<=T&&e.bone<=.4,dangling=!tear&&e.bone<=.25&&e.tissue<=T+.15;
  if(requireFail&&!(tear||dangling))continue;
  Failing f;f.seg=e.seg;f.st=e.st;f.k=e.k;f.tissue=e.tissue;f.bone=e.bone;f.dangling=dangling;double best=1e30;
  for(const auto&pl:guide.planes){const double off=std::abs(dot(e.st->center-pl.point,pl.normal));if(off>e.seg->radius)continue;
   const Vec ab=pl.b-pl.a;const double L2=dot(ab,ab);const double t=L2>1e-12?std::clamp(dot(e.st->center-pl.a,ab)/L2,0.,1.):0.;
   if(length(e.st->center-(pl.a+ab*t))>e.seg->radius+.04)continue;if(off<best){best=off;f.guided=true;f.guideNormal=pl.normal;}}
  v.failing.push_back(f);}
 if(profiling())profileText()<<"instant_prof verdict_mcycles blocks="<<cycBlocks<<" samples="<<cycSamples<<" cells="<<cycCells<<'\n';
 // The blast lanes' order: blade-guided first, then the weakest; one tear per limb radius; at most 12.
 auto order=v.failing;std::stable_sort(order.begin(),order.end(),[](const Failing&a,const Failing&b){if(a.guided!=b.guided)return a.guided;return a.tissue+a.bone<b.tissue+b.bone;});
 for(const auto&f:order){bool close=false;for(const auto&p:v.picks)if(p.seg==f.seg&&std::abs(p.st->s-f.st->s)<=f.seg->radius){close=true;break;}
  if(!close&&v.picks.size()<12)v.picks.push_back(f);}
 return v;
}
// ---- rows of blast cuts across the torso ---------------------------------------------------------------------------
// PainterVoxelBlast cuts a point-blank torso with a row of 14 overlapping bores (same direction, radius and depth, their
// entries on one line across the body). Nothing in the joint profile crosses the torso: the voxel path separates it
// through connectivity once the row's bores have removed the whole cross section. A row is the longest run of such
// parallel bores whose axes lie in one plane at gaps under their diameter.
struct Row {Vec point,normal,side;double halfSpan{},radius{};std::vector<std::size_t> craters;};
inline std::vector<Row> detectRows(const std::vector<Crater>&craters){
 std::vector<Row> rows;std::vector<char> used(craters.size(),0);
 for(std::size_t i=0;i<craters.size();++i){if(used[i]||!craters[i].blast)continue;
  std::vector<std::size_t> group;
  for(std::size_t j=i;j<craters.size();++j){if(used[j]||!craters[j].blast)continue;
   if(dot(craters[j].cut.direction,craters[i].cut.direction)>1-1e-9&&std::abs(craters[j].radius-craters[i].radius)<1e-9&&std::abs(craters[j].penetration-craters[i].penetration)<1e-9)group.push_back(j);}
  if(group.size()<6)continue;
  const Vec d=craters[i].cut.direction;
  // The spread direction of the axes (their mapped entries, projected off the bore direction).
  std::size_t fa=group.front(),fb=group.front();double widest=-1;
  for(auto a:group)for(auto b:group){Vec w=craters[b].mapped-craters[a].mapped;w=w-d*dot(w,d);const double l=length(w);if(l>widest){widest=l;fa=a;fb=b;}}
  if(widest<.05)continue;Vec side=craters[fb].mapped-craters[fa].mapped;side=unit(side-d*dot(side,d));
  const Vec normal=unit(cross(side,d));
  std::vector<std::pair<double,std::size_t>> along;bool planar=true;
  for(auto j:group){const Vec w=craters[j].mapped-craters[fa].mapped;if(std::abs(dot(w,normal))>.25*craters[i].radius)planar=false;along.push_back({dot(w,side),j});}
  if(!planar)continue;std::sort(along.begin(),along.end());
  // longest run of gaps under one bore diameter
  std::size_t bestFirst=0,bestLast=0;for(std::size_t a=0;a<along.size();){std::size_t b=a;while(b+1<along.size()&&along[b+1].first-along[b].first<=2*craters[i].radius)++b;
   if(b-a>bestLast-bestFirst){bestFirst=a;bestLast=b;}a=b+1;}
  if(bestLast-bestFirst+1<6)continue;
  Row r;r.side=side;r.normal=normal;r.radius=craters[i].radius;Vec sum{};
  for(std::size_t q=bestFirst;q<=bestLast;++q){r.craters.push_back(along[q].second);used[along[q].second]=1;sum=sum+craters[along[q].second].cut.entry;}
  r.point=sum*(1./double(r.craters.size()));r.point=r.point-normal*dot(r.point-craters[fa].mapped,normal);
  r.halfSpan=.5*(along[bestLast].first-along[bestFirst].first);rows.push_back(std::move(r));}
 return rows;
}
// ---- the cut section on a tear plane --------------------------------------------------------------------------------
// Pixels of the plane (pitch h, radius Rs about the plane point): 1 the section - the tissue connected in the plane to
// the seeds (DamageTear.h's cross-section: the limb's whole cross-section, not a fixed disk); 2 the cap region - the
// closure field connected to the section within 2.5 cm (garment gaps and clothing the cap must seal), grown two pixels;
// 4 the footprint - the section grown by 2.5 cm (vertices near the plane there go by their side of it).
struct Section {
 Vec point,normal,u,v;double h{},Rs{};int M{},W{};bool closed{};std::size_t tissuePixels{};
 std::vector<std::uint8_t> px;
 bool inside(int i,int j)const{return i>=0&&j>=0&&i<W&&j<W;}
 std::uint8_t at(double x,double y)const{const int i=int(std::lround(x/h))+M,j=int(std::lround(y/h))+M;return inside(i,j)?px[std::size_t(i)*std::size_t(W)+std::size_t(j)]:std::uint8_t(0);}
 Vec pos(double i,double j)const{return point+u*((i-M)*h)+v*((j-M)*h);}
 // Footprint test of p projected along the plane normal.
 bool footprint(const Vec&p)const{const Vec d=p-point;return (at(dot(d,u),dot(d,v))&4)!=0;}
};
template<class Seed,class Accept>
inline Section buildSection(const Slots&slots,const Vec&point,const Vec&normal,double Rs,double h,const Seed&seed,const Accept&accept){
 Section s;s.point=point;s.normal=normal;s.h=h;s.Rs=Rs;s.M=int(std::ceil(Rs/h));s.W=2*s.M+1;
 s.u=unit(cross(normal,std::abs(normal.x)<.8?Vec{1,0,0}:Vec{0,1,0}));s.v=cross(normal,s.u);
 const std::size_t N=std::size_t(s.W)*std::size_t(s.W);s.px.assign(N,0);std::vector<std::uint8_t> raw(N,0);
 const double R2=Rs*Rs;std::vector<std::size_t> stack;
 for(int i=0;i<s.W;++i)for(int j=0;j<s.W;++j){const double x=(i-s.M)*h,y=(j-s.M)*h;if(x*x+y*y>R2)continue;
  const std::size_t q=std::size_t(i)*s.W+std::size_t(j);if(accept(s.pos(i,j)))raw[q]=1;
  if(raw[q]&&seed(x,y)){s.px[q]=1;stack.push_back(q);}}
 while(!stack.empty()){const std::size_t q=stack.back();stack.pop_back();const int i=int(q/s.W),j=int(q%s.W);
  const int nb[4][2]={{i-1,j},{i+1,j},{i,j-1},{i,j+1}};
  for(const auto&m:nb){if(!s.inside(m[0],m[1]))continue;const std::size_t r=std::size_t(m[0])*s.W+std::size_t(m[1]);if(!raw[r]||(s.px[r]&1))continue;s.px[r]|=1;stack.push_back(r);}}
 // footprint: the section grown by 2.5 cm (separable square growth)
 {const int g=int(std::ceil(.025/h));std::vector<std::uint8_t> rowsGrown(N,0);std::vector<int> pre(std::size_t(s.W)+1);
  for(int i=0;i<s.W;++i){pre[0]=0;for(int j=0;j<s.W;++j)pre[std::size_t(j)+1]=pre[std::size_t(j)]+((s.px[std::size_t(i)*s.W+j]&1)?1:0);
   for(int j=0;j<s.W;++j){const int lo=std::max(0,j-g),hi=std::min(s.W-1,j+g);if(pre[std::size_t(hi)+1]-pre[std::size_t(lo)])rowsGrown[std::size_t(i)*s.W+j]=1;}}
  for(int j=0;j<s.W;++j){pre[0]=0;for(int i=0;i<s.W;++i)pre[std::size_t(i)+1]=pre[std::size_t(i)]+rowsGrown[std::size_t(i)*s.W+j];
   for(int i=0;i<s.W;++i){const int lo=std::max(0,i-g),hi=std::min(s.W-1,i+g);if(pre[std::size_t(hi)+1]-pre[std::size_t(lo)])s.px[std::size_t(i)*s.W+j]|=4;}}}
 // cap region: the closure field connected to the section inside the footprint, grown two pixels
 {std::vector<std::uint8_t> cap(N,0);for(std::size_t q=0;q<N;++q)if(s.px[q]&1){cap[q]=1;stack.push_back(q);}
  while(!stack.empty()){const std::size_t q=stack.back();stack.pop_back();const int i=int(q/s.W),j=int(q%s.W);
   const int nb[4][2]={{i-1,j},{i+1,j},{i,j-1},{i,j+1}};
   for(const auto&m:nb){if(!s.inside(m[0],m[1]))continue;const std::size_t r=std::size_t(m[0])*s.W+std::size_t(m[1]);if(cap[r]||!(s.px[r]&4))continue;
    const double x=(m[0]-s.M)*h,y=(m[1]-s.M)*h;if(x*x+y*y>R2)continue;if(!raw[r]&&!slots.closure(s.pos(m[0],m[1])))continue;cap[r]=1;stack.push_back(r);}}
  for(int i=0;i<s.W;++i)for(int j=0;j<s.W;++j)if(cap[std::size_t(i)*s.W+j])for(int a=std::max(0,i-2);a<=std::min(s.W-1,i+2);++a)for(int b=std::max(0,j-2);b<=std::min(s.W-1,j+2);++b)s.px[std::size_t(a)*s.W+b]|=2;}
 // closed: the section stays inside its disk (it does not run on through tissue past Rs)
 s.closed=true;const double edge=(Rs-2.5*h)*(Rs-2.5*h);
 for(int i=0;i<s.W;++i)for(int j=0;j<s.W;++j){const std::size_t q=std::size_t(i)*s.W+j;if(!(s.px[q]&1))continue;++s.tissuePixels;
  const double x=(i-s.M)*h,y=(j-s.M)*h;if(x*x+y*y>edge)s.closed=false;}
 return s;
}
// ---- the cap outline and its triangles ------------------------------------------------------------------------------
// Boundary of the cap region traced along pixel edges (region on the left), simplified within 1.5 pixels (the region
// was grown by two pixels, so the simplified outline still covers every cap pixel) and triangulated. The shader discards
// the cap's pixels outside tissue/closure, so the rim is exact per pixel whatever the polygon. Holes are filled.
inline void simplifyLoop(const std::vector<polytri::P2>&in,double tol,std::vector<polytri::P2>&out){
 out.clear();const std::size_t n=in.size();if(n<4){out=in;return;}
 std::size_t opposite=0;double best=-1;for(std::size_t i=1;i<n;++i){const double d=polytri::dist(in[0],in[i]);if(d>best){best=d;opposite=i;}}
 std::vector<char> keep(n,0);keep[0]=keep[opposite]=1;
 auto rdp=[&](auto&&self,std::size_t a,std::size_t b)->void{ // indices mod n, a<b (b may be n meaning 0)
  const polytri::P2 A=in[a%n],B=in[b%n];double worst=-1;std::size_t at=a;
  for(std::size_t i=a+1;i<b;++i){const double d=std::abs(polytri::detail::height(A,B,in[i%n]));if(d>worst){worst=d;at=i;}}
  if(worst>tol){keep[at%n]=1;self(self,a,at);self(self,at,b);}};
 rdp(rdp,0,opposite);rdp(rdp,opposite,n);
 for(std::size_t i=0;i<n;++i)if(keep[i])out.push_back(in[i]);
}
// HOOK(track D): the cap outline is this section's boundary; D's PieceRimProfile rim (seed, angle, depth in the cap
// cutter's u/v frame) replaces it here once its header is in this tree.
inline std::vector<std::array<polytri::P2,3>> capTriangles(const Section&s){
 std::vector<std::array<polytri::P2,3>> tris;const int W=s.W,C=W+1;
 auto inCap=[&](int i,int j){return s.inside(i,j)&&(s.px[std::size_t(i)*W+j]&2);};
 // directed pixel-edge boundary, corners (a,b) = pixel corner lattice
 std::vector<std::array<std::int32_t,2>> edges;std::vector<std::int32_t> out1(std::size_t(C)*C,-1),out2(std::size_t(C)*C,-1);
 auto add=[&](int a0,int b0,int a1,int b1){const std::int32_t from=a0*C+b0,to=a1*C+b1;const auto id=std::int32_t(edges.size());edges.push_back({from,to});
  if(out1[std::size_t(from)]<0)out1[std::size_t(from)]=id;else out2[std::size_t(from)]=id;};
 for(int i=0;i<W;++i)for(int j=0;j<W;++j){if(!inCap(i,j))continue;
  if(!inCap(i,j-1))add(i,j,i+1,j);if(!inCap(i+1,j))add(i+1,j,i+1,j+1);if(!inCap(i,j+1))add(i+1,j+1,i,j+1);if(!inCap(i-1,j))add(i,j+1,i,j);}
 if(edges.empty())return tris;
 auto P=[&](std::int32_t corner){const int a=corner/C,b=corner%C;return polytri::P2{(a-s.M-.5)*s.h,(b-s.M-.5)*s.h};};
 std::vector<char> used(edges.size(),0);bool failed=false;
 for(std::size_t e0=0;e0<edges.size()&&!failed;++e0){if(used[e0])continue;
  std::vector<polytri::P2> loop;std::size_t e=e0;
  for(std::size_t guard=0;guard<=edges.size();++guard){used[e]=1;loop.push_back(P(edges[e][0]));const auto end=edges[e][1];
   std::int32_t next=out1[std::size_t(end)];if(next<0||used[std::size_t(next)])next=out2[std::size_t(end)];
   if(next<0||used[std::size_t(next)])break;e=std::size_t(next);}
  double area=0;for(std::size_t q=0;q<loop.size();++q){const auto a=loop[q],b=loop[(q+1)%loop.size()];area+=a.x*b.y-b.x*a.y;}
  if(area<=0||loop.size()<3)continue; // a hole (filled by its outer loop) or a pinch fragment
  std::vector<polytri::P2> simple;simplifyLoop(loop,1.5*s.h,simple);
  std::vector<polytri::P2> pts=simple;std::vector<polytri::DEdge> dedges;
  for(std::uint32_t q=0;q<pts.size();++q)dedges.push_back({q,std::uint32_t((q+1)%pts.size())});
  std::vector<polytri::Tri> out;std::string error;
  if(pts.size()>=3&&polytri::triangulate(pts,dedges,1e-7,out,error)){for(const auto&t:out)tris.push_back({pts[t[0]],pts[t[1]],pts[t[2]]});}
  else failed=true;}
 if(failed){ // robust cover: 4 x 4 pixel cells holding any cap pixel, two triangles each
  tris.clear();for(int i=0;i<W;i+=4)for(int j=0;j<W;j+=4){bool any=false;for(int a=i;a<std::min(W,i+4)&&!any;++a)for(int b=j;b<std::min(W,j+4);++b)if(inCap(a,b)){any=true;break;}
   if(!any)continue;const polytri::P2 p0{(i-s.M-.5)*s.h,(j-s.M-.5)*s.h},p1{(i+4-s.M-.5)*s.h,(j-s.M-.5)*s.h},p2{(i+4-s.M-.5)*s.h,(j+4-s.M-.5)*s.h},p3{(i-s.M-.5)*s.h,(j+4-s.M-.5)*s.h};
   tris.push_back({p0,p1,p2});tris.push_back({p0,p2,p3});}}
 return tris;
}
// ---- cuts and piece ownership ---------------------------------------------------------------------------------------
struct Cut {
 Vec point,normal;double R{};const SeverSegment*seg{};std::size_t k{};bool row{},dangling{},guided{};double tissue{1},bone{1};
 Section section;std::array<char,64> split{},distal{}; // elements the plane divides; elements wholly beyond it
 int piece{-1},depth{};double s{};std::uint16_t element{65535}; // distal piece, order key, dominant element of the section
 std::size_t open{}; // section tissue outside the tear's disc the damage leaves (it keeps the sides joined)
};
struct Ownership {
 const std::vector<Cut>&cuts;const std::vector<std::size_t>&order;
 // Fast path of of(p,e,true): an element no plane divides, away from every plane's footprint, is wholly on one piece.
 std::array<int,64> elementPiece{};std::array<char,64> divided{};std::vector<double> reach2;
 Ownership(const std::vector<Cut>&c,const std::vector<std::size_t>&o):cuts(c),order(o){
  for(const auto&cut:cuts)for(unsigned e=0;e<64;++e)divided[e]|=cut.split[e];
  for(unsigned e=0;e<64;++e)elementPiece[e]=divided[e]?-1:of(Vec{},std::uint16_t(e),false);
  for(const auto&cut:cuts){const double r=cut.section.Rs+.03;reach2.push_back(r*r+.03*.03);}}
 int fast(const Vec&p,std::uint16_t e)const{
  if(e>=64||divided[e])return of(p,e,true);
  for(std::size_t i=0;i<cuts.size();++i){const Vec d=p-cuts[i].point;if(dot(d,d)<=reach2[i])return of(p,e,true);}
  return elementPiece[e];}
 // The piece of a point of physical element e: every cut (outer before nested) it lies beyond moves it to that cut's
 // piece. Near a plane inside its footprint the side alone decides (skin weighted across a joint, garments), else the
 // element: an element the plane divides by its side, an element hanging below one of those by the cut's flag.
 int of(const Vec&p,std::uint16_t e,bool surface)const{int piece=0;
  for(const auto i:order){const Cut&c=cuts[i];const double sd=dot(p-c.point,c.normal);bool beyond;
   if(surface&&std::abs(sd)<.03&&c.section.footprint(p))beyond=sd>0;
   else if(e<64&&c.split[e])beyond=sd>0;
   else beyond=e<64&&c.distal[e];
   if(beyond)piece=c.piece;}
  return piece;}
};
// ---- mass properties of element parts --------------------------------------------------------------------------------
struct Moments {long double n{},s[3]{},q[3]{},xy{},xz{},yz{};double lo[3]{1e300,1e300,1e300},hi[3]{-1e300,-1e300,-1e300};};
// Cells (x,y,z0..z1-1) of the map, in rest space, with the anchor-frame projections of their extent.
inline void addRun(Moments&m,const ElementIndex&ix,int x,int y,int z0,int z1,const Matrix&restToAnchor){if(z1<=z0)return;
 const double h=ix.map->pitch;const Vec base=ix.cellCenter(x,y,0);const long double n=z1-z0;
 const long double zs=((long double)z0+z1-1)*n*.5L; // sum of z over the run
 const long double zq=((long double)(z1-1)*z1*(2*(long double)z1-1)-(long double)(z0-1)*z0*(2*(long double)z0-1))/6.L; // sum of z^2
 const long double X=base.x,Y=base.y,Z0=base.z;
 // cell centre (X, Y, Z0 + z h)
 const long double Sz=n*Z0+zs*h,Szz=n*Z0*Z0+2*Z0*zs*h+zq*h*h;
 m.n+=n;m.s[0]+=n*X;m.s[1]+=n*Y;m.s[2]+=Sz;m.q[0]+=n*X*X;m.q[1]+=n*Y*Y;m.q[2]+=Szz;m.xy+=n*X*Y;m.xz+=X*Sz;m.yz+=Y*Sz;
 const auto&r=restToAnchor.row_major;const double half=.5*h;
 for(int a=0;a<3;++a){const double ax=r[a*4],ay=r[a*4+1],az=r[a*4+2],t=r[a*4+3];const double ext=half*(std::abs(ax)+std::abs(ay)+std::abs(az));
  const double p0=ax*base.x+ay*base.y+az*(base.z+z0*h)+t,p1=ax*base.x+ay*base.y+az*(base.z+(z1-1)*h)+t;
  m.lo[a]=std::min(m.lo[a],std::min(p0,p1)-ext);m.hi[a]=std::max(m.hi[a],std::max(p0,p1)+ext);}
}
// A 16 mm cube of moment sums, with its bounds projected on the anchor frame's axes.
inline void addCoarse(Moments&m,const Coarse&c,const Matrix&restToAnchor){
 m.n+=c.n;for(int a=0;a<3;++a){m.s[a]+=c.s[a];m.q[a]+=c.q[a];}m.xy+=c.xy;m.xz+=c.xz;m.yz+=c.yz;
 const auto&r=restToAnchor.row_major;const Vec center=(c.lo+c.hi)*.5,half=(c.hi-c.lo)*.5;
 for(int a=0;a<3;++a){const double ax=r[a*4],ay=r[a*4+1],az=r[a*4+2];const double mid=ax*center.x+ay*center.y+az*center.z+r[a*4+3];
  const double ext=std::abs(ax)*half.x+std::abs(ay)*half.y+std::abs(az)*half.z;m.lo[a]=std::min(m.lo[a],mid-ext);m.hi[a]=std::max(m.hi[a],mid+ext);}}
// A fitted element from its cells: one box = the cells' bounds in the element's bone frame, exact mass/COM/inertia
// (the sums fitCollider uses), density 985 kg/m3.
inline SeparationElement fittedElement(std::uint16_t bone,const Moments&m,double pitch,const Matrix&restToAnchor){
 SeparationElement e;e.bone=bone;e.cells=std::uint64_t(m.n);if(!(m.n>0))return e;
 const long double cube=pitch*pitch*pitch,cellMass=985.L*cube,n=m.n;
 const long double vx=m.q[0]-m.s[0]*m.s[0]/n+n*pitch*pitch/12,vy=m.q[1]-m.s[1]*m.s[1]/n+n*pitch*pitch/12,vz=m.q[2]-m.s[2]*m.s[2]/n+n*pitch*pitch/12;
 const long double cxy=m.xy-m.s[0]*m.s[1]/n,cxz=m.xz-m.s[0]*m.s[2]/n,cyz=m.yz-m.s[1]*m.s[2]/n;
 const long double ir[3][3]={{cellMass*(vy+vz),-cellMass*cxy,-cellMass*cxz},{-cellMass*cxy,cellMass*(vx+vz),-cellMass*cyz},{-cellMass*cxz,-cellMass*cyz,cellMass*(vx+vy)}};
 const auto&rot=restToAnchor.row_major;
 for(int i=0;i<3;++i)for(int j=0;j<3;++j){long double value=0;for(int k=0;k<3;++k)for(int l=0;l<3;++l)value+=rot[i*4+k]*ir[k][l]*rot[j*4+l];e.inertia[std::size_t(i*3+j)]=double(value);}
 const Vec com{double(m.s[0]/n),double(m.s[1]/n),double(m.s[2]/n)};
 e.com={rot[0]*com.x+rot[1]*com.y+rot[2]*com.z+rot[3],rot[4]*com.x+rot[5]*com.y+rot[6]*com.z+rot[7],rot[8]*com.x+rot[9]*com.y+rot[10]*com.z+rot[11]};
 e.mass_kg=double(cellMass*n);
 SeparationBox box;box.center={.5*(m.lo[0]+m.hi[0]),.5*(m.lo[1]+m.hi[1]),.5*(m.lo[2]+m.hi[2])};
 box.half_extents={std::clamp(.5*(m.hi[0]-m.lo[0]),.005,.75),std::clamp(.5*(m.hi[1]-m.lo[1]),.005,.75),std::clamp(.5*(m.hi[2]-m.lo[2]),.005,.75)};
 box.orientation={1,0,0,0,1,0,0,0,1};e.boxes.push_back(box);
 return e;
}
// ---- the synthetic tear cutter of a cap ------------------------------------------------------------------------------
// HOOK(track D): the cutter gets D's 128-knot edge table (WoundEdgeProfile importer) here, so the shader's cap and the
// CPU predicate share one rim.
inline ResolvedTunnel capCutter(const Vec&planePoint,const Vec&d,double radius,std::uint64_t sequence){
 constexpr double inlet=.006,reach=.06; // 6 mm behind the cap plane; hides this piece's walls up to 6 cm past the break
 ResolvedTunnel r;r.role=ResolvedTunnel::Role::tear;r.sequence=sequence;r.edge_seed=0;
 const Vec entry=planePoint+d*inlet;const Vec eu=unit(cross(d,std::abs(d.x)<.8?Vec{1,0,0}:Vec{0,1,0})),ev=cross(d,eu);
 r.entry=point(entry);r.direction=point(d);r.edge_u=point(eu);r.edge_v=point(ev);
 r.radius=radius;r.penetration=reach;r.cap=.001;r.inlet=inlet;r.exit_distance=reach;r.exit_mult=1;
 return r;
}
// ---- one split row of a triangle cut at a plane (LowPolyPieces.h's sec63 rows: P/N/T/B/UV interpolated, frame
// renormalized, the skin weights of the corner on this side) ---------------------------------------------------------
inline gfy_packed::Vertex along(const gfy_packed::Vertex&a,const gfy_packed::Vertex&b,double s,const gfy_packed::Vertex&weights){
 gfy_packed::Vertex r=weights;for(unsigned q=0;q<14;++q)r.surface[q]=float(double(a.surface[q])*(1-s)+double(b.surface[q])*s);
 for(unsigned base:{3u,6u,9u}){double l=0;for(unsigned q=0;q<3;++q)l+=double(r.surface[base+q])*r.surface[base+q];l=std::sqrt(l);if(l>1e-9)for(unsigned q=0;q<3;++q)r.surface[base+q]=float(r.surface[base+q]/l);}
 return r;
}
inline Vec position(const gfy_packed::Vertex&v){return {v.surface[0],v.surface[1],v.surface[2]};}
// separation_detail::cutSkinAnchor without the exception: the element posing a skin weight of `original` on a piece
// holding `elements` (itself, else its one adjacent element there), 65535 when there is none or more than one.
inline std::uint16_t anchorOf(std::uint16_t original,const std::set<std::uint16_t>&elements,const std::vector<std::uint16_t>&parents){
 if(elements.count(original))return original;std::uint16_t found=65535;unsigned count=0;
 if(original<parents.size()&&elements.count(parents[original])){found=parents[original];++count;}
 for(auto bone:elements)if(bone<parents.size()&&parents[bone]==original&&bone!=found){found=bone;++count;}
 return count==1?found:std::uint16_t(65535);
}
}
// How an instant split's pieces own the snapshot's tissue: the cell rule of instant_detail::Ownership (element, then
// each cut in order: a divided element by its side of the plane, an element hanging below one by the cut's flag).
struct InstantPlan {
 std::shared_ptr<const Asset> asset;std::shared_ptr<const Grid> grid;std::shared_ptr<const instant_detail::ElementIndex> elements;
 struct Plane {Vec point,normal;std::array<char,64> split{},distal{};int piece{};};
 std::vector<Plane> planes;std::size_t pieces{}; // planes outer before nested
 std::vector<std::array<int,6>> mesh_tiles; // per piece: tile bounds (lo, hi inclusive) of its birth mesh's vertices
 struct Band {std::size_t piece{};Vec point,d;double radius{};}; // a piece's cap: plane point, axis out of the piece, reach
 std::vector<Band> bands;
 int of(const Vec&p,std::uint16_t e)const{int piece=0;for(const auto&c:planes){const bool beyond=e<64&&(c.split[e]?dot(p-c.point,c.normal)>0:c.distal[e]!=0);if(beyond)piece=c.piece;}return piece;}
};
namespace instant_detail {
// ---- the job ----------------------------------------------------------------------------------------------------------
struct Job {
 std::shared_ptr<const Asset> asset;std::shared_ptr<const Grid> grid;std::shared_ptr<const ResolvedCutHistory> history;
 std::shared_ptr<const Output> geometry;std::vector<char> failed; // committed publication; stations already torn
 InstantSeverInput input;Stamp stamp;
};
struct SplitTri {std::array<gfy_packed::Vertex,3> v;std::uint32_t source{};};
inline std::shared_ptr<SeparationOutput> run(const Job&job){
 namespace pc=gfy_piece_collider;
 const auto t0=core::Clock::now();auto phase=t0;auto lap=[&](double&slot){slot+=core::ms(phase);phase=core::Clock::now();};
 const Asset&asset=*job.asset;const Grid&grid=*job.grid;const auto&input=job.input;const auto&sep=input.separation;
 auto out=std::make_shared<SeparationOutput>();out->stamp=job.stamp;auto&rep=out->instant;rep.instant=true;
 need(sep.physical_parents.size()==asset.source.bones,"separation_original_physics_tree");
 need(bool(job.history),"instant_requires_cut_history");
 need(std::isfinite(sep.minimum_piece_m3)&&sep.minimum_piece_m3>=.0000001&&sep.minimum_piece_m3<=.001,"separation_piece_volume_limit");
 struct FlushProfile{~FlushProfile(){if(profiling()){auto&text=profileText();std::cerr<<text.str()<<std::flush;text.str("");text.clear();}}} flushProfile; // every return path
 const Slots slots{asset};ProfClock prof;auto mark=[&](const char*what){if(profiling()){const double v=prof.lap();profileText()<<"instant_prof "<<what<<' '<<v<<'\n';}else prof.lap();};
 // ---- 1. the cutters the worker will carve
 const auto craters=resolveCraters(asset,grid,input.impacts);rep.craters=input.impacts.size();rep.resolved=craters.size();
 bool blast=false;for(const auto&e:input.impacts)blast=blast||e.blast;
 // DamageTear's threshold scale: 2 for a blast batch, the ammo's (or its radius) for bullets.
 double severScale=2;
 if(!blast){severScale=0;for(const auto&e:input.impacts)severScale=std::max(severScale,e.sever_scale>0?e.sever_scale:std::clamp(e.radius/.008,.3,2.));if(!(severScale>0))severScale=1;}
 TearGuide guide;guide.tilt=blast;
 {struct Slot{Vec a,b,dir;unsigned n=0;};std::map<std::uint32_t,Slot> bladeSlots; // grenade blade slots (rest space), as the worker collects them
  for(const auto&c:craters)if(c.blast&&c.blade){auto&s=bladeSlots[c.blade];if(!s.n)s.a=c.mapped;s.b=c.mapped;s.dir=s.dir+c.cut.direction;++s.n;}
  for(const auto&e:bladeSlots){const auto&s=e.second;if(s.n<2)continue;const Vec along=s.b-s.a;if(length(along)<.02||length(s.dir)<1e-9)continue;
   const Vec normal=cross(unit(along),unit(s.dir));if(length(normal)<.2)continue;guide.planes.push_back({(s.a+s.b)*.5,unit(normal),s.a,s.b});}}
 lap(rep.resolve_ms);mark("resolve");
 // ---- 2. which stations fail, and torso rows
 SeverState committed;committed.failed=job.failed;
 const auto verdict=stationVerdict(asset,grid,job.failed.empty()?nullptr:&committed,craters,guide,severScale,input.require_station_fail);
 rep.stations_measured=verdict.measured;rep.exact_stations=verdict.exact;rep.samples=verdict.samples;
 if(profiling())profileText()<<"instant_prof verdict_counts measured="<<verdict.measured<<" exact="<<verdict.exact<<" cell_exact="<<verdict.cells<<" blocks="<<verdict.blocks<<" mixed="<<verdict.mixed<<" quarters="<<verdict.quarters<<" samples="<<verdict.samples<<'\n';
 mark("verdict");
 auto report=[](const Failing&f){InstantSplitStation s;s.bone=f.seg->bone;s.s=f.st->s;s.tissue=f.tissue;s.bone_left=f.bone;s.dangling=f.dangling;s.guided=f.guided;
  s.point=point(f.st->center);s.normal=point(f.seg->axis);return s;};
 for(const auto&f:verdict.failing)rep.failing.push_back(report(f));
 for(const auto&f:verdict.picks)rep.picked.push_back(report(f));
 const auto frames=framesOf(asset,sep);const auto index=elementsOf(asset,sep,*frames);const auto&ix=*index;const auto&parents=sep.physical_parents;mark("frames_elements");
 const std::size_t bones=frames->bones.size();
 auto elementOfBone=[&](std::uint16_t b)->std::uint16_t{if(b>=bones)return 65535;const int e=frames->bones[b].physical_element;return e<0||e>=64?std::uint16_t(65535):std::uint16_t(e);};
 std::array<char,64> isElement{};for(std::size_t b=0;b<bones&&b<64;++b)if(frames->bones[b].physical_element==int(b)&&ix.cells[b])isElement[b]=1;
 std::array<std::vector<std::uint16_t>,64> childrenOf;
 for(unsigned e=0;e<64;++e)if(isElement[e]&&parents[e]<64&&isElement[parents[e]])childrenOf[parents[e]].push_back(std::uint16_t(e));
 auto depthOf=[&](std::uint16_t e){int d=0;for(std::uint16_t p=e;p<64&&parents[p]<64&&d<64;p=parents[p])++d;return d;};
 std::vector<Cut> cuts;
 for(const auto&f:verdict.picks){Cut c;c.seg=f.seg;c.k=f.k;c.point=f.st->center;c.normal=f.seg->axis;c.R=f.seg->radius;c.s=f.st->s;
  c.dangling=f.dangling;c.guided=f.guided;c.tissue=f.tissue;c.bone=f.bone;cuts.push_back(std::move(c));}
 std::vector<Row> rows=blast?detectRows(craters):std::vector<Row>{};
 for(auto&r:rows){
  // Orientation: the element the row crosses keeps its parent on the near side (the part that comes off is beyond).
  std::array<std::size_t,64> hits{};for(int q=-4;q<=4;++q){const auto e=ix.at(r.point+r.side*(r.halfSpan*q/4.));if(e<64)++hits[e];}
  std::uint16_t e=65535;std::size_t most=0;for(unsigned q=0;q<64;++q)if(hits[q]>most){most=hits[q];e=std::uint16_t(q);}
  if(e<64&&parents[e]<64&&ix.cells[parents[e]]&&dot(ix.centroid[parents[e]]-r.point,r.normal)>0)r.normal=r.normal*-1.;
  const double Rs=r.halfSpan+r.radius+.06;const Vec side=r.side;
  auto seed=[&](const Section&s){return [&s,&r,side](double x,double y){const Vec p=s.point+s.u*x+s.v*y;const double t=std::clamp(dot(p-r.point,side),-r.halfSpan,r.halfSpan);return length(p-(r.point+side*t))<=r.radius;};};
  Section probe;probe.point=r.point;probe.normal=r.normal;probe.u=unit(cross(r.normal,std::abs(r.normal.x)<.8?Vec{1,0,0}:Vec{0,1,0}));probe.v=cross(r.normal,probe.u);
  // Only the crossed element's tissue counts (arms lying in the plane stay attached above the row either way).
  Section section=buildSection(slots,r.point,r.normal,Rs,.004,seed(probe),[&](const Vec&p){return slots.at(p)&&(e>=64||ix.at(p)==e);});
  // The row severs the torso when its bores (with the committed damage) cover the whole section.
  core::TileCache removed(grid);std::size_t open=0;std::vector<std::uint8_t> debugImage(std::getenv("GFY_INSTANT_DEBUG_DIR")?std::size_t(section.W)*section.W:0,0);
  const double blockReach=std::sqrt(2.)*1.5*section.h+1e-9;std::vector<const Crater*> blockOpen;
  // counting stops once more stands than the row may leave (the count is then a lower bound)
  const std::size_t openLimit=std::size_t(.002*double(section.tissuePixels));auto counting=[&]{return open<=openLimit||!debugImage.empty();};
  for(int bi=0;bi<section.W&&counting();bi+=4)for(int bj=0;bj<section.W&&counting();bj+=4){
   bool any=false;for(int i=bi;i<std::min(section.W,bi+4)&&!any;++i)for(int j=bj;j<std::min(section.W,bj+4);++j)if(section.px[std::size_t(i)*section.W+j]&1){any=true;break;}
   if(!any&&debugImage.empty())continue;
   const Vec centre=section.pos(bi+1.5,bj+1.5);int bcls=-1;blockOpen.clear();
   for(const auto&cr:craters){if(centre.x+blockReach<cr.lo.x||centre.y+blockReach<cr.lo.y||centre.z+blockReach<cr.lo.z||centre.x-blockReach>cr.hi.x||centre.y-blockReach>cr.hi.y||centre.z-blockReach>cr.hi.z)continue;
    const int r=classifyBall(cr.cut,centre,blockReach);if(r==1){bcls=1;break;}if(r==0){bcls=0;blockOpen.push_back(&cr);}}
  for(int i=bi;i<std::min(section.W,bi+4);++i)for(int j=bj;j<std::min(section.W,bj+4);++j){if(!debugImage.empty()&&slots.at(section.pos(i,j)))debugImage[std::size_t(j)*section.W+i]=40;if(!(section.px[std::size_t(i)*section.W+j]&1))continue;const Vec p=section.pos(i,j);
   const Cell cell{int(std::floor((p.x-grid.origin.x)/grid.pitch)),int(std::floor((p.y-grid.origin.y)/grid.pitch)),int(std::floor((p.z-grid.origin.z)/grid.pitch))};
   bool gone=bcls==1||(grid.valid(cell)&&removed.removed(cell));
   for(std::size_t q=0;q<blockOpen.size()&&!gone;++q)gone=blockOpen[q]->cut.contains(p);
   if(!debugImage.empty())debugImage[std::size_t(j)*section.W+i]=gone?110:255;
   if(!gone)++open;}}
  if(!debugImage.empty()){static std::atomic<int> serial{0};const std::string path=std::string(std::getenv("GFY_INSTANT_DEBUG_DIR"))+"/row_"+std::to_string(++serial)+".pgm";
   std::ofstream f(path,std::ios::binary);f<<"P5\n"<<section.W<<' '<<section.W<<"\n255\n";f.write(reinterpret_cast<const char*>(debugImage.data()),std::streamsize(debugImage.size()));}
  const double left=section.tissuePixels?double(open)/double(section.tissuePixels):1.;
  if(profiling())profileText()<<"instant_row craters="<<r.craters.size()<<" half_span="<<r.halfSpan<<" radius="<<r.radius<<" section_px="<<section.tissuePixels<<" open="<<open<<" closed="<<section.closed<<'\n';
  InstantSplitStation s;s.bone=65535;s.row=true;s.tissue=left;s.bone_left=left;s.point=point(r.point);s.normal=point(r.normal);rep.failing.push_back(s);
  // No station tears a torso: only connectivity separates it, so the bores must cover the whole section (0.2 %: pixel
  // sampling at the rims). blast_chest_035 leaves 1.2 % (lateral rims 4-12 mm thick, above the 5 mm thread tear) and
  // the voxel path keeps that torso whole.
  if(section.tissuePixels&&left<=.002){Cut c;c.row=true;c.point=r.point;c.normal=r.normal;c.R=r.halfSpan+r.radius;c.tissue=left;c.bone=left;
   c.section=std::move(section);cuts.push_back(std::move(c));}}
 lap(rep.verdict_ms);mark("rows");
 if(cuts.empty()){out->no_separation=true;out->original_components=1;out->remaining_components=1;rep.total_ms=core::ms(t0);return out;}
 if(cuts.size()>12)cuts.resize(12);
 // ---- 3. sections, split elements, piece order
 // Section tissue outside a disc of radius Rt that neither the craters nor the committed damage removed.
 core::TileCache removedCells(grid);std::vector<const Crater*> blockCrossed;
 auto openOutside=[&](const Section&section,double Rt)->std::size_t{std::size_t open=0;const double reach=std::sqrt(2.)*1.5*section.h+1e-9,R2=Rt*Rt;
  auto outside=[&](int i,int j){const double x=(i-section.M)*section.h,y=(j-section.M)*section.h;return x*x+y*y>R2;};
  for(int bi=0;bi<section.W;bi+=4)for(int bj=0;bj<section.W;bj+=4){bool any=false;
   for(int i=bi;i<std::min(section.W,bi+4)&&!any;++i)for(int j=bj;j<std::min(section.W,bj+4);++j)if((section.px[std::size_t(i)*section.W+j]&1)&&outside(i,j)){any=true;break;}
   if(!any)continue;const Vec centre=section.pos(bi+1.5,bj+1.5);int cls=-1;blockCrossed.clear();
   for(const auto&cr:craters){if(centre.x+reach<cr.lo.x||centre.y+reach<cr.lo.y||centre.z+reach<cr.lo.z||centre.x-reach>cr.hi.x||centre.y-reach>cr.hi.y||centre.z-reach>cr.hi.z)continue;
    const int r=classifyBall(cr.cut,centre,reach);if(r==1){cls=1;break;}if(r==0){cls=0;blockCrossed.push_back(&cr);}}
   if(cls==1)continue;
   for(int i=bi;i<std::min(section.W,bi+4);++i)for(int j=bj;j<std::min(section.W,bj+4);++j){if(!(section.px[std::size_t(i)*section.W+j]&1)||!outside(i,j))continue;
    const Vec p=section.pos(i,j);const Cell cell{int(std::floor((p.x-grid.origin.x)/grid.pitch)),int(std::floor((p.y-grid.origin.y)/grid.pitch)),int(std::floor((p.z-grid.origin.z)/grid.pitch))};
    bool gone=grid.valid(cell)&&removedCells.removed(cell);for(std::size_t q=0;q<blockCrossed.size()&&!gone;++q)gone=blockCrossed[q]->cut.contains(p);
    if(!gone)++open;}}
  return open;};
 for(auto&c:cuts){
  if(!c.row){const double Rb=c.R+.008,Rs=std::max(Rb+.03,2.2*c.R+.008),h=c.R<.03?.0015:(c.R<.06?.003:.004);
   c.section=buildSection(slots,c.point,c.normal,Rs,h,[Rb](double x,double y){return x*x+y*y<=Rb*Rb;},[&](const Vec&p){return slots.at(p)!=0;});
   // DamageTear tears the limb's cross-section connected to its disc out to Rs (DamageTear.h): tissue running on past
   // Rs that the damage leaves keeps the two sides joined, and the voxel path finds no separation there (a foot or a
   // hand longer than the section). The outer ring of the section is where it runs on.
   c.open=openOutside(c.section,Rs-2.5*h);}
  // The plane divides the elements of the section that belong to the segment's own element or hang below it (a jaw or
  // shoulder station's section also holds the neck or chest, which stay whole on their side).
  std::array<char,64> allowed{};
  if(c.seg){const auto root=elementOfBone(c.seg->bone);
   if(root<64){std::vector<std::uint16_t> stack{root};while(!stack.empty()){const auto q=stack.back();stack.pop_back();if(allowed[q])continue;allowed[q]=1;for(auto g:childrenOf[q])stack.push_back(g);}}}
  else allowed.fill(1);
  std::array<std::size_t,64> count{};std::size_t total=0;const auto&s=c.section;
  for(int i=0;i<s.W;i+=2)for(int j=0;j<s.W;j+=2)if(s.px[std::size_t(i)*s.W+j]&1){const auto e=ix.at(s.pos(i,j));if(e<64&&isElement[e]&&allowed[e]){++count[e];++total;}}
  std::size_t most=0;for(unsigned e=0;e<64;++e){if(count[e]>most){most=count[e];c.element=std::uint16_t(e);}if(count[e]&&count[e]*20>=total)c.split[e]=1;}
  if(c.element>=64)continue;
  // The part that comes off is the side away from the body: the divided element's parent lies on the near side (a
  // segment's axis need not point away from the body: the foot segment runs up from the ankle, the jaw's down).
  {const auto parent=parents[c.element];Vec toward{};bool known=false;
   if(parent<64&&ix.cells[parent]){toward=ix.centroid[parent]-c.point;known=true;}
   else{std::int64_t balance=0;const Vec base=c.point; // the root element: the smaller side comes off
    for(const auto&run:ix.runs[c.element]){const Vec p=ix.cellCenter(run.x,run.y,.5*(run.lo+run.hi-1));balance+=(dot(p-base,c.normal)>0?1:-1)*std::int64_t(run.hi-run.lo);}
    toward=c.normal*(balance>0?1.:-1.);known=true;}
   if(known&&dot(toward,c.normal)>0)c.normal=c.normal*-1.;}
  // Elements hanging below a divided one on the far side of the plane come off whole.
  for(unsigned e=0;e<64;++e)if(c.split[e])for(auto ch:childrenOf[e]){if(c.split[ch]||dot(ix.centroid[ch]-c.point,c.normal)<=0)continue;
   std::vector<std::uint16_t> stack{ch};while(!stack.empty()){const auto q=stack.back();stack.pop_back();if(c.split[q])continue;c.distal[q]=1;for(auto g:childrenOf[q])stack.push_back(g);}}
  c.depth=depthOf(c.element);}
 if(profiling())for(const auto&c:cuts){auto&o=profileText();o<<"instant_cut bone="<<(c.seg?int(c.seg->bone):-1)<<" row="<<c.row<<" s="<<c.s<<" element="<<c.element<<" px="<<c.section.tissuePixels<<" open="<<c.open<<" split=";
  for(unsigned e=0;e<64;++e)if(c.split[e])o<<e<<',';o<<" distal=";for(unsigned e=0;e<64;++e)if(c.distal[e])o<<e<<',';o<<'\n';}
 cuts.erase(std::remove_if(cuts.begin(),cuts.end(),[](const Cut&c){return c.element>=64||!c.section.tissuePixels||double(c.open)>.002*double(c.section.tissuePixels);}),cuts.end());mark("sections");
 if(cuts.empty()){out->no_separation=true;out->original_components=1;out->remaining_components=1;rep.total_ms=core::ms(t0);return out;}
 lap(rep.cap_ms);
 // ---- 4. element parts per piece (a cut too small for a piece is dropped and the parts recounted)
 std::vector<std::size_t> order;std::size_t P=0;const double pitch=ix.map->pitch,cell=pitch*pitch*pitch;
 std::vector<std::array<Moments,64>> parts;std::vector<std::array<std::uint64_t,64>> cellsOf;std::array<char,64> divided{};
 // What the craters leave of an element, per 16 mm cube: 0 inside a crater, 1 clear of all, crossed cubes by 8 samples.
 std::array<std::vector<float>,64> keepCache;std::array<char,64> keepDone{};std::vector<const Crater*> nearCraters,crossed;
 std::array<std::vector<const Crater*>,64> nearOf;
 auto keepCube=[&](unsigned e,std::size_t q)->float{auto&k=keepCache[e];const auto&cubes=ix.coarse[e];
  if(!keepDone[e]){keepDone[e]=1;k.assign(cubes.size(),-1.f);Vec elo{1e300,1e300,1e300},ehi{-1e300,-1e300,-1e300};
   for(const auto&cc:cubes){elo={std::min(elo.x,cc.lo.x),std::min(elo.y,cc.lo.y),std::min(elo.z,cc.lo.z)};ehi={std::max(ehi.x,cc.hi.x),std::max(ehi.y,cc.hi.y),std::max(ehi.z,cc.hi.z)};}
   for(const auto&cr:craters)if(!(cr.hi.x<elo.x||cr.hi.y<elo.y||cr.hi.z<elo.z||cr.lo.x>ehi.x||cr.lo.y>ehi.y||cr.lo.z>ehi.z))nearOf[e].push_back(&cr);}
  if(k[q]>=0)return k[q];k[q]=1.f;const auto&nearCraters=nearOf[e];if(nearCraters.empty())return 1.f;
  {const auto&cc=cubes[q];const Vec c=(cc.lo+cc.hi)*.5;const double r=.5*length(cc.hi-cc.lo)+1e-9;crossed.clear();bool gone=false;
   for(const auto*cr:nearCraters){if(c.x+r<cr->lo.x||c.y+r<cr->lo.y||c.z+r<cr->lo.z||c.x-r>cr->hi.x||c.y-r>cr->hi.y||c.z-r>cr->hi.z)continue;
    const int v=classifyBall(cr->cut,c,r);if(v==1){gone=true;break;}if(v==0)crossed.push_back(cr);}
   if(gone){k[q]=0;return 0.f;}if(crossed.empty())return 1.f;
   const Vec h=(cc.hi-cc.lo)*.25;int kept=0;
   for(int s=0;s<8;++s){const Vec p{c.x+((s&1)?h.x:-h.x),c.y+((s&2)?h.y:-h.y),c.z+((s&4)?h.z:-h.z)};bool in=false;for(const auto*cr:crossed)if(cr->cut.contains(p)){in=true;break;}kept+=!in;}
   k[q]=float(kept)/8.f;}
  return k[q];};
 // Bounds of every element's cubes (a piece's box is the union of its cubes' or whole elements' boxes).
 std::array<std::array<double,6>,64> elementBox;for(auto&b:elementBox)b={1e300,1e300,1e300,-1e300,-1e300,-1e300};
 for(unsigned e=0;e<64;++e)for(const auto&cc:ix.coarse[e]){auto&b=elementBox[e];b={std::min(b[0],cc.lo.x),std::min(b[1],cc.lo.y),std::min(b[2],cc.lo.z),std::max(b[3],cc.hi.x),std::max(b[4],cc.hi.y),std::max(b[5],cc.hi.z)};}
 auto grow=[](std::array<double,6>&b,const std::array<double,6>&o){for(int a=0;a<3;++a){b[a]=std::min(b[a],o[a]);b[a+3]=std::max(b[a+3],o[a+3]);}};
 std::vector<std::array<double,6>> pieceBox;
 for(;;){order.resize(cuts.size());for(std::size_t i=0;i<cuts.size();++i)order[i]=i;
  // Cuts dividing one element nest along the limb: a cut whose point lies beyond another's plane comes after it (the
  // stations' s are positions along different segments - calf, foot, the knee end of the thigh - and cannot order them).
  std::vector<int> nest(cuts.size(),0);
  for(std::size_t i=0;i<cuts.size();++i)for(std::size_t j=0;j<cuts.size();++j)
   if(i!=j&&cuts[i].element<64&&cuts[j].split[cuts[i].element]&&dot(cuts[i].point-cuts[j].point,cuts[j].normal)>0)++nest[i];
  std::stable_sort(order.begin(),order.end(),[&](std::size_t a,std::size_t b){if(cuts[a].depth!=cuts[b].depth)return cuts[a].depth<cuts[b].depth;
   if(nest[a]!=nest[b])return nest[a]<nest[b];return cuts[a].s<cuts[b].s;});
  for(std::size_t q=0;q<order.size();++q)cuts[order[q]].piece=int(q+1);
  P=cuts.size()+1;const Ownership own{cuts,order};parts.assign(P,{});cellsOf.assign(P,{});divided.fill(0);
  pieceBox.assign(P,{1e300,1e300,1e300,-1e300,-1e300,-1e300});
  for(const auto&c:cuts)for(unsigned e=0;e<64;++e)divided[e]|=c.split[e];
  for(unsigned e=0;e<64;++e){if(!isElement[e])continue;
   if(!divided[e]){const int piece=own.of(ix.centroid[e],std::uint16_t(e),false);cellsOf[std::size_t(piece)][e]=ix.cells[e];grow(pieceBox[std::size_t(piece)],elementBox[e]);continue;}
   const Matrix&toAnchor=frames->bones[e].rest_to_anchor; // a divided element: its 16 mm cubes by the side of their centroid
   for(std::size_t q=0;q<ix.coarse[e].size();++q){const auto&cc=ix.coarse[e][q];const int piece=own.of(cc.centroid(),std::uint16_t(e),false);
    addCoarse(parts[std::size_t(piece)][e],cc,toAnchor);cellsOf[std::size_t(piece)][e]+=std::uint64_t(cc.n);grow(pieceBox[std::size_t(piece)],{cc.lo.x,cc.lo.y,cc.lo.z,cc.hi.x,cc.hi.y,cc.hi.z});}}
  // What the craters leave of a piece: only when the craters near it could remove three quarters of it (their capsule
  // volumes) are its cubes counted.
  auto survivingOf=[&](std::size_t p,std::uint64_t n)->double{const auto&b=pieceBox[p];double bound=0;
   for(const auto&cr:craters){if(cr.hi.x<b[0]||cr.hi.y<b[1]||cr.hi.z<b[2]||cr.lo.x>b[3]||cr.lo.y>b[4]||cr.lo.z>b[5])continue;
    // the part of the axis within the box grown by the radius (slab clipping), plus the end caps
    const Vec d=cr.b-cr.a;double t0=0,t1=1;
    for(int a=0;a<3&&t0<=t1;++a){const double lo=b[a]-cr.maxR,hi=b[a+3]+cr.maxR,o=cr.a[a],v=d[a];
     if(std::abs(v)<1e-15){if(o<lo||o>hi)t1=-1;continue;}double s0=(lo-o)/v,s1=(hi-o)/v;if(s0>s1)std::swap(s0,s1);t0=std::max(t0,s0);t1=std::min(t1,s1);}
    if(t1<t0)continue;bound+=3.141592653589793*cr.maxR*cr.maxR*((t1-t0)*length(d)+2*cr.maxR);}
   if(bound<.75*double(n)*cell)return double(n);
   double kept=0;
   for(unsigned e=0;e<64;++e){if(!cellsOf[p][e])continue;
    for(std::size_t q=0;q<ix.coarse[e].size();++q){const auto&cc=ix.coarse[e][q];if(divided[e]&&std::size_t(own.of(cc.centroid(),std::uint16_t(e),false))!=p)continue;kept+=cc.n*keepCube(e,q);}}
   return kept;};
  std::vector<double> surviving(P,-1.);
  // A cut whose own piece is below the piece minimum (or holds nothing) is not a separation; neither is one whose piece
  // the craters leave less than a quarter of (a stretch between two tears): that stays on the near side and is carved
  // there by the catch-up instead of flying off intact and vanishing.
  std::size_t drop=cuts.size();
  for(std::size_t i=0;i<cuts.size()&&drop==cuts.size();++i){std::uint64_t n=0;for(unsigned e=0;e<64;++e)n+=cellsOf[std::size_t(cuts[i].piece)][e];
   if(double(n)*cell<sep.minimum_piece_m3){drop=i;break;}
   const double kept=surviving[std::size_t(cuts[i].piece)]=survivingOf(std::size_t(cuts[i].piece),n);
   if(kept*cell<sep.minimum_piece_m3||kept<.25*double(n))drop=i;}
  if(drop==cuts.size())break;
  if(profiling()){std::uint64_t n=0;for(unsigned e=0;e<64;++e)n+=cellsOf[std::size_t(cuts[drop].piece)][e];profileText()<<"instant_drop bone="<<(cuts[drop].seg?int(cuts[drop].seg->bone):-1)<<" row="<<cuts[drop].row<<" piece="<<cuts[drop].piece<<" cells="<<n<<" surviving="<<surviving[std::size_t(cuts[drop].piece)]<<'\n';}
  cuts.erase(cuts.begin()+std::ptrdiff_t(drop));
  if(cuts.empty()){out->no_separation=true;out->original_components=1;out->remaining_components=1;rep.total_ms=core::ms(t0);return out;}}
 mark("element_parts");const Ownership own{cuts,order};
 std::vector<std::set<std::uint16_t>> elementsOf(P);std::map<std::uint16_t,unsigned> pieceOwners;
 for(std::size_t k=0;k<P;++k)for(unsigned e=0;e<64;++e)if(cellsOf[k][e]){elementsOf[k].insert(std::uint16_t(e));++pieceOwners[std::uint16_t(e)];}
 for(std::size_t k=0;k<P;++k)need(!elementsOf[k].empty(),"instant_piece_without_elements");
 // ---- 5. the published mesh split between the pieces
 const gfy_packed::Mesh*published=job.geometry&&!job.geometry->mesh.children.empty()?&job.geometry->mesh:nullptr;
 std::shared_ptr<const gfy_packed::Mesh> birth;if(!published){birth=birthMesh(asset);published=birth.get();}
 need(published->tissue_field!=nullptr,"instant_publication_tissue_field");
 gfy_packed::validate_skin_ownership(*published);
 const painter_detail::AnalyticChannelField*field=published->analytic_channels.get();
 const auto&children=published->children;
 // anchorable(k,bone): a skin weight on bone can be represented on piece k (its element is there or is the unique neighbour)
 std::vector<std::array<signed char,64>> anchorOk(P);for(auto&a:anchorOk)a.fill(-1);
 auto anchorable=[&](std::size_t k,std::uint16_t bone)->bool{if(bone>=64)return false;auto&cached=anchorOk[k][bone];
  if(cached<0){const auto e=elementOfBone(bone);cached=e<64&&anchorOf(e,elementsOf[k],parents)!=65535?1:0;}
  return cached==1;};
 std::vector<std::uint64_t> anchorMask(P,0);for(std::size_t k=0;k<P;++k)for(unsigned b=0;b<64;++b)if(anchorable(k,std::uint16_t(b)))anchorMask[k]|=std::uint64_t(1)<<b;
 auto anchored=[&](std::size_t k,const gfy_packed::Vertex&v){for(unsigned w=0;w<4;++w)if(v.weight[w]>separation_detail::negligible_skin_weight&&(v.bone[w]>=64||!((anchorMask[k]>>v.bone[w])&1u)))return false;return true;};
 auto dominantElement=[&](const gfy_packed::Vertex&v){unsigned best=0;for(unsigned w=1;w<4;++w)if(v.weight[w]>v.weight[best])best=w;return elementOfBone(v.bone[best]);};
 auto bonesOf=[&](const gfy_packed::Vertex&v){std::uint64_t m=0;for(unsigned w=0;w<4;++w)if(v.weight[w]>separation_detail::negligible_skin_weight)m|=v.bone[w]<64?std::uint64_t(1)<<v.bone[w]:~std::uint64_t(0);return m;};
 // A skin triangle's crossing of a cut plane: its two points, with the weights of the near (below) and far side.
 struct RimSeg {std::size_t cut;gfy_packed::Vertex a0,b0,a1,b1;};
 struct Work {std::vector<std::vector<std::uint32_t>> tris;std::vector<std::vector<SplitTri>> extra;std::vector<RimSeg> rims;};
 std::vector<Work> work(children.size());std::size_t unanchored=0,splitCount=0,dropped=0;
 for(std::size_t ci=0;ci<children.size();++ci){const auto&src=children[ci];auto&w=work[ci];w.tris.resize(P);w.extra.resize(P);
  if(src.indices.empty()||src.vertices.empty())continue;
  if(src.material){const auto*prim=field?field->child(src.child_index):nullptr;need(prim&&prim->count==src.indices.size()/3,"instant_channel_manifest");}
  std::vector<std::int8_t> ow(src.vertices.size());std::vector<std::uint64_t> used(src.vertices.size());
  for(std::size_t q=0;q<src.vertices.size();++q){ow[q]=std::int8_t(own.fast(position(src.vertices[q]),dominantElement(src.vertices[q])));used[q]=bonesOf(src.vertices[q]);}
  const std::size_t T=src.indices.size()/3;
  for(std::size_t t=0;t<T;++t){const std::uint16_t id[3]={src.indices[3*t],src.indices[3*t+1],src.indices[3*t+2]};
   const int o0=ow[id[0]],o1=ow[id[1]],o2=ow[id[2]];
   if(o0==o1&&o1==o2){if(!((used[id[0]]|used[id[1]]|used[id[2]])&~anchorMask[std::size_t(o0)]))w.tris[std::size_t(o0)].push_back(std::uint32_t(t));else ++unanchored;continue;}
   // Straddles: cut at every plane its corners lie on both sides of; each part goes to the piece its centre belongs to.
   std::vector<std::vector<gfy_packed::Vertex>> polys{{src.vertices[id[0]],src.vertices[id[1]],src.vertices[id[2]]}};bool crossed=false;
   for(const auto i:order){const Cut&c=cuts[i];double sd[3];int pos=0,neg=0;
    for(int q=0;q<3;++q){sd[q]=dot(position(src.vertices[id[q]])-c.point,c.normal);(sd[q]>0?pos:neg)++;}
    if(!pos||!neg)continue;crossed=true;std::vector<std::vector<gfy_packed::Vertex>> next;
    for(const auto&poly:polys){std::vector<gfy_packed::Vertex> above,below;const std::size_t n=poly.size();gfy_packed::Vertex hit[4];int hits=0;
     for(std::size_t q=0;q<n;++q){const auto&a=poly[q];const auto&b=poly[(q+1)%n];const double da=dot(position(a)-c.point,c.normal),db=dot(position(b)-c.point,c.normal);
      (da>0?above:below).push_back(a);
      if((da>0)!=(db>0)){const double s=da/(da-db);above.push_back(along(a,b,s,da>0?a:b));below.push_back(along(a,b,s,da>0?b:a));if(hits<4){hit[hits++]=below.back();hit[hits++]=above.back();}}}
     if(!src.material&&hits==4&&above.size()>=3&&below.size()>=3)w.rims.push_back({i,hit[0],hit[2],hit[1],hit[3]});
     if(above.size()>=3)next.push_back(std::move(above));if(below.size()>=3)next.push_back(std::move(below));}
    polys.swap(next);}
   if(!crossed){const int k=o0==o1||o0==o2?o0:(o1==o2?o1:o0);
    if(anchored(std::size_t(k),src.vertices[id[0]])&&anchored(std::size_t(k),src.vertices[id[1]])&&anchored(std::size_t(k),src.vertices[id[2]]))w.tris[std::size_t(k)].push_back(std::uint32_t(t));else ++unanchored;continue;}
   ++splitCount;
   for(const auto&poly:polys){Vec centre{};for(const auto&v:poly)centre=centre+position(v);centre=centre*(1./double(poly.size()));
    const int k=own.of(centre,dominantElement(poly[0]),true);bool fine=true;for(const auto&v:poly)fine=fine&&anchored(std::size_t(k),v);if(!fine){++unanchored;continue;}
    for(std::size_t q=1;q+1<poly.size();++q){const Vec a=position(poly[0]),b=position(poly[q]),d=position(poly[q+1]);if(length(cross(b-a,d-a))<2e-9){++dropped;continue;}
     w.extra[std::size_t(k)].push_back({{poly[0],poly[q],poly[q+1]},std::uint32_t(t)});}}}}
 rep.split_triangles=splitCount;lap(rep.geometry_ms);mark("triangles");
 // ---- skin rims: each cut's crossing segments (skin and garments) form a planar graph (nodes: 10 um keys); a cap's
 // outline is the face of that graph holding the cut's centre - the innermost boundary around it (the neck's skin and
 // the inner wall of a collar that meets it; not the gap out to a sleeve or a hood) - walked with the face on the left:
 // from the first edge a ray from the centre meets, at every node the edge first clockwise from the way back. Open ends
 // (seams, small holes) are bridged to the nearest node within 3 mm. The cap's rim is then the skin's cut edge with the
 // skin's weights (sealed); without such a face (or past twice the section's reach) the section outline is the cap.
 struct RimPoint {Vec p;gfy_packed::Vertex nearRow,farRow;};
 std::vector<std::vector<RimPoint>> rimLoop(cuts.size());
 {std::vector<std::vector<const RimSeg*>> byCut(cuts.size());for(const auto&w:work)for(const auto&r:w.rims)byCut[r.cut].push_back(&r);
  using Key=std::array<long long,3>;auto key=[](const Vec&p){return Key{{std::llround(p.x*1e5),std::llround(p.y*1e5),std::llround(p.z*1e5)}};};
  for(std::size_t ci=0;ci<cuts.size();++ci){const auto&segs=byCut[ci];if(segs.size()<3)continue;const Cut&c=cuts[ci];
   if(const char*dir=std::getenv("GFY_INSTANT_DEBUG_DIR")){ // the cut's rim segments in the section frame (debug)
    static std::atomic<int> serial{0};std::ofstream f(std::string(dir)+"/rim_"+std::to_string(c.seg?int(c.seg->bone):-1)+"_"+std::to_string(++serial)+".txt");
    for(const auto*r:segs){const Vec a=position(r->a0)-c.point,b=position(r->b0)-c.point;f<<dot(a,c.section.u)<<' '<<dot(a,c.section.v)<<' '<<dot(b,c.section.u)<<' '<<dot(b,c.section.v)<<'\n';}}
   std::map<Key,std::uint32_t> nodeOf;std::vector<RimPoint> node;std::vector<std::array<double,2>> xy;
   auto nodeFor=[&](const gfy_packed::Vertex&vn,const gfy_packed::Vertex&vf){const Vec p=position(vn);const Key k=key(p);const auto it=nodeOf.find(k);if(it!=nodeOf.end())return it->second;
    const auto id=std::uint32_t(node.size());nodeOf.emplace(k,id);node.push_back({p,vn,vf});const Vec d=p-c.point;xy.push_back({dot(d,c.section.u),dot(d,c.section.v)});return id;};
   std::vector<std::vector<std::uint32_t>> adj;
   auto link=[&](std::uint32_t a,std::uint32_t b){if(a==b)return;if(adj.size()<node.size())adj.resize(node.size());
    if(std::find(adj[a].begin(),adj[a].end(),b)==adj[a].end()){adj[a].push_back(b);adj[b].push_back(a);}};
   for(const auto*r:segs){const auto a=nodeFor(r->a0,r->a1),b=nodeFor(r->b0,r->b1);link(a,b);}
   adj.resize(node.size());std::size_t bridges=0;
   // An open end (a seam, a small hole, skin ending under a collar) joins the nearest point of another edge within 3 mm;
   // that edge is split there (a T-junction), so the face walk can turn onto it.
   const std::size_t original=node.size();
   for(std::uint32_t n=0;n<original;++n){if(adj[n].size()!=1)continue;double best=.003;std::uint32_t ba=~0u,bb=~0u;double bt=0;
    for(std::uint32_t a=0;a<node.size();++a)for(const auto b:adj[a]){if(b<a||a==n||b==n)continue;const Vec ab=node[b].p-node[a].p;const double L2=dot(ab,ab);
     const double s=L2>1e-18?std::clamp(dot(node[n].p-node[a].p,ab)/L2,0.,1.):0.;const double d=length(node[a].p+ab*s-node[n].p);if(d<best){best=d;ba=a;bb=b;bt=s;}}
    if(ba==~0u)continue;std::uint32_t to;
    if(bt<.02)to=ba;else if(bt>.98)to=bb;
    else{to=std::uint32_t(node.size());const auto&A=node[ba];const auto&B=node[bb];
     node.push_back({A.p+(B.p-A.p)*bt,along(A.nearRow,B.nearRow,bt,bt<.5?A.nearRow:B.nearRow),along(A.farRow,B.farRow,bt,bt<.5?A.farRow:B.farRow)});
     const Vec d=node.back().p-c.point;xy.push_back({dot(d,c.section.u),dot(d,c.section.v)});adj.emplace_back();
     adj[ba].erase(std::find(adj[ba].begin(),adj[ba].end(),bb));adj[bb].erase(std::find(adj[bb].begin(),adj[bb].end(),ba));link(ba,to);link(to,bb);}
    if(to!=adj[n][0]){link(n,to);++bridges;}}
   // Planar: edges that cross (a skin line running over a collar wall) are split at the crossing.
   {std::vector<std::array<std::uint32_t,2>> E;for(std::uint32_t a=0;a<node.size();++a)for(const auto b:adj[a])if(a<b)E.push_back({a,b});
    std::vector<std::vector<std::pair<double,std::uint32_t>>> splits(E.size());bool any=false;
    for(std::size_t i=0;i<E.size();++i)for(std::size_t j=i+1;j<E.size();++j){const auto a=E[i][0],b=E[i][1],c2=E[j][0],d2=E[j][1];
     if(a==c2||a==d2||b==c2||b==d2)continue;
     const double rx=xy[b][0]-xy[a][0],ry=xy[b][1]-xy[a][1],sx=xy[d2][0]-xy[c2][0],sy=xy[d2][1]-xy[c2][1],den=rx*sy-ry*sx;if(std::abs(den)<1e-18)continue;
     const double qx=xy[c2][0]-xy[a][0],qy=xy[c2][1]-xy[a][1],s=(qx*sy-qy*sx)/den,r=(qx*ry-qy*rx)/den;
     if(s<=1e-6||s>=1-1e-6||r<=1e-6||r>=1-1e-6)continue;
     const auto m=std::uint32_t(node.size());const RimPoint A=node[a],B=node[b];
     node.push_back({A.p+(B.p-A.p)*s,along(A.nearRow,B.nearRow,s,s<.5?A.nearRow:B.nearRow),along(A.farRow,B.farRow,s,s<.5?A.farRow:B.farRow)});
     const Vec dd=node.back().p-c.point;xy.push_back({dot(dd,c.section.u),dot(dd,c.section.v)});splits[i].push_back({s,m});splits[j].push_back({r,m});any=true;}
    if(any){adj.assign(node.size(),{});
     for(std::size_t i=0;i<E.size();++i){auto&sp=splits[i];std::sort(sp.begin(),sp.end());std::uint32_t prev=E[i][0];for(const auto&x:sp){link(prev,x.second);prev=x.second;}link(prev,E[i][1]);}}}
   // the first edge a ray from the centre along +u meets, directed with the centre on its left
   double bestX=1e30;std::uint32_t ea=~0u,eb=~0u;
   for(std::uint32_t a=0;a<node.size();++a)for(const auto b:adj[a]){if(b<a)continue;const auto&A=xy[a];const auto&B=xy[b];if((A[1]>0)==(B[1]>0))continue;
    const double x=A[0]+(B[0]-A[0])*(0-A[1])/(B[1]-A[1]);if(x>0&&x<bestX){bestX=x;ea=a;eb=b;}}
   std::size_t faceSize=0;bool enclosed=false,inReach=false;
   if(ea!=~0u){if((xy[eb][0]-xy[ea][0])*(0-xy[ea][1])-(xy[eb][1]-xy[ea][1])*(0-xy[ea][0])<0)std::swap(ea,eb);
    std::vector<std::uint32_t> face;std::uint32_t u=ea,v=eb;bool closed=false;
    for(std::size_t guard=0;guard<4*node.size()+8;++guard){face.push_back(u);
     const double bx=xy[u][0]-xy[v][0],by=xy[u][1]-xy[v][1];double bestAngle=1e30;std::uint32_t w=u;
     for(const auto cand:adj[v]){const double dx=xy[cand][0]-xy[v][0],dy=xy[cand][1]-xy[v][1];
      double a=-std::atan2(bx*dy-by*dx,bx*dx+by*dy);if(a<=0)a+=tau;if(a<bestAngle){bestAngle=a;w=cand;}}
     u=v;v=w;if(u==ea&&v==eb){closed=true;break;}}
    if(closed&&face.size()>=3){double area=0;bool inside=false;double far=0;
     for(std::size_t q=0;q<face.size();++q){const auto&a=xy[face[q]];const auto&b=xy[face[(q+1)%face.size()]];area+=a[0]*b[1]-b[0]*a[1];
      if((a[1]>0)!=(b[1]>0)&&a[0]+(b[0]-a[0])*(0-a[1])/(b[1]-a[1])>0)inside=!inside;far=std::max(far,std::hypot(a[0],a[1]));}
     faceSize=face.size();enclosed=inside&&area>0;inReach=far<=2*c.section.Rs;
     if(const char*dir=std::getenv("GFY_INSTANT_DEBUG_DIR")){static std::atomic<int> serial{0};
      std::ofstream f(std::string(dir)+"/face_"+std::to_string(c.seg?int(c.seg->bone):-1)+"_"+std::to_string(++serial)+".txt");
      for(std::size_t q=0;q<face.size();++q){const auto&a=xy[face[q]];const auto&b=xy[face[(q+1)%face.size()]];f<<a[0]<<' '<<a[1]<<' '<<b[0]<<' '<<b[1]<<'\n';}}
     if(enclosed&&inReach){auto&loop=rimLoop[ci];for(const auto n:face)loop.push_back(node[n]);}}}
   if(profiling())profileText()<<"instant_rim bone="<<(c.seg?int(c.seg->bone):-1)<<" segments="<<segs.size()<<" nodes="<<node.size()<<" bridges="<<bridges
    <<" face="<<faceSize<<" enclosed="<<enclosed<<" in_reach="<<inReach<<" chosen="<<rimLoop[ci].size()<<'\n';}}
 // ---- 6. caps
 struct Cap {std::size_t piece{};Vec planePoint,d;std::vector<std::array<Vec,3>> tris;std::uint16_t bone{65535};ResolvedTunnel cutter;
  bool sealed{};std::vector<std::array<gfy_packed::Vertex,3>> rimTris;}; // sealed: triangles of the skin rim loop, the side's rows
 std::vector<Cap> caps;std::uint64_t capSerial=0;
 for(const auto i:order){const Cut&c=cuts[i];ProfClock capClock;const auto&loop=rimLoop[i];std::vector<polytri::Tri> loopTris;
  if(!loop.empty()){std::vector<polytri::P2> pts;std::vector<polytri::DEdge> edges;std::string error;
   for(const auto&rp:loop){const Vec d=rp.p-c.point;pts.push_back({dot(d,c.section.u),dot(d,c.section.v)});}
   double signedArea=0;for(std::size_t q=0;q<pts.size();++q){const auto&a=pts[q];const auto&b=pts[(q+1)%pts.size()];signedArea+=a.x*b.y-b.x*a.y;}
   // Star-shaped around the cut's centre (a limb's section): a fan from the centre (index = the loop size), O(n);
   // otherwise the loop is triangulated as a polygon.
   bool star=std::abs(signedArea)>0;
   for(std::size_t q=0;q<pts.size()&&star;++q){const auto&a=pts[q];const auto&b=pts[(q+1)%pts.size()];const double o=a.x*b.y-b.x*a.y;if(!(signedArea>0?o>0:o<0))star=false;}
   if(star)for(std::uint32_t q=0;q<pts.size();++q)loopTris.push_back({std::uint32_t(pts.size()),q,std::uint32_t((q+1)%pts.size())});
   else{for(std::uint32_t q=0;q<pts.size();++q){const std::uint32_t r=std::uint32_t((q+1)%pts.size());edges.push_back(signedArea>0?polytri::DEdge{q,r}:polytri::DEdge{r,q});} // region on the left
    if(!polytri::triangulate(pts,edges,1e-7,loopTris,error))loopTris.clear();}}
  const auto tri2=loopTris.empty()?capTriangles(c.section):std::vector<std::array<polytri::P2,3>>{};
  if(profiling())profileText()<<"instant_cap bone="<<(c.seg?int(c.seg->bone):-1)<<" W="<<c.section.W<<" rim_points="<<loop.size()<<" rim_triangles="<<loopTris.size()<<" section_triangles="<<tri2.size()<<" ms="<<capClock.lap()<<'\n';
  if(tri2.empty()&&loopTris.empty())continue;
  double reach=0;for(const auto&t:tri2)for(const auto&p:t)reach=std::max(reach,std::hypot(p.x,p.y));
  if(!loopTris.empty())for(const auto&rp:loop)reach=std::max(reach,length(rp.p-c.point));
  for(int sideSign:{-1,1}){Cap cap;cap.planePoint=c.point;cap.d=c.normal*double(sideSign); // axis out of the piece: +n for the near piece, -n for the far one
   cap.piece=std::size_t(own.of(c.point-cap.d*.004,c.element,false));
   cap.bone=elementsOf[cap.piece].count(c.element)?c.element:*elementsOf[cap.piece].begin();
   if(!loopTris.empty()){cap.sealed=true; // the near piece's rim carries the near side's weights, the far piece's the far side's
    gfy_packed::Vertex centre=loop.front().nearRow; // the fan's centre: on the plane at the cut point, rigid to the cap's bone
    centre.surface[0]=float(c.point.x);centre.surface[1]=float(c.point.y);centre.surface[2]=float(c.point.z);
    for(unsigned w=0;w<4;++w){centre.bone[w]=0;centre.weight[w]=0;}centre.bone[0]=cap.bone;centre.weight[0]=1;
    for(const auto&t:loopTris){std::array<gfy_packed::Vertex,3> r;
     for(int q=0;q<3;++q){const auto ix=t[std::size_t(q)];r[std::size_t(q)]=ix>=loop.size()?centre:(sideSign>0?loop[ix].nearRow:loop[ix].farRow);}
     if(dot(cross(position(r[1])-position(r[0]),position(r[2])-position(r[0])),cap.d)<0)std::swap(r[1],r[2]); // face normal = the cutter axis
     cap.rimTris.push_back(r);}}
   const bool frameAlong=dot(cross(c.section.u,c.section.v),cap.d)>0; // outline triangles are counter-clockwise in (u,v)
   for(const auto&t:tri2){std::array<Vec,3> p;for(int q=0;q<3;++q)p[std::size_t(q)]=c.section.point+c.section.u*t[std::size_t(q)].x+c.section.v*t[std::size_t(q)].y;
    if(!frameAlong)std::swap(p[1],p[2]); // emitted face normal = the cutter axis (the visible side)
    cap.tris.push_back(p);}
   cap.cutter=capCutter(c.point,cap.d,reach+.01,(std::uint64_t(1)<<63)|(++capSerial));
   caps.push_back(std::move(cap));}}
 rep.caps=caps.size();mark("cap_outlines");
 // ---- 7. assembly: children, analytic field (history + this piece's caps), removal field
 const unsigned materialSource=asset.face_children.empty()?asset.source_child:asset.face_children.front();
 std::vector<const ResolvedTunnel*> records;
 if(job.history){std::vector<const ResolvedCutHistory*> chain;for(auto p=job.history.get();p;p=p->parent.get())chain.push_back(p);
  for(auto it=chain.rbegin();it!=chain.rend();++it)for(const auto&r:(*it)->appended)records.push_back(&r);}
 need(!field||field->cuts==records.size(),"instant_field_history_mismatch");
 painter_detail::ChannelIdentity identity;
 if(field)identity=field->identity;
 else{identity.source=asset.source_hash;identity.rig=asset.rig_hash;identity.cookie=job.stamp.owner.spawn_cookie;identity.lineage=job.stamp.owner.visual_lineage;identity.topology=job.stamp.owner.topology;
  identity.generation=job.stamp.expected_generation;identity.revision=job.stamp.revision;}
 if(!identity.snapshot){if(job.history)identity.snapshot=job.history;else identity.snapshot=std::make_shared<const int>(0);}
 const unsigned limitations=field?field->limitations:(job.history?job.history->limitations:0u);
 std::vector<std::shared_ptr<const ResolvedCutHistory>> pieceHistory(P);
 std::vector<SeparationPiece> pieces(P);double cycChildren=0,cycCaps=0,cycField=0,cycRemoval=0;ProfClock aclock;
 for(std::size_t k=0;k<P;++k){auto&piece=pieces[k];piece.component=std::uint32_t(k+1);auto&mesh=piece.mesh;aclock.lap();
  mesh.tissue_field=published->tissue_field;mesh.analytic_coverage_admitted=published->analytic_coverage_admitted;
  mesh.reduced_skin_vertices=published->reduced_skin_vertices;mesh.max_discarded_skin_weight=published->max_discarded_skin_weight;
  // The piece's history: the committed cuts, then its caps' cutters (a piece session rebuilds its field from its seed's
  // history, so the caps the birth mesh draws must be cuts of it).
  {auto node=ResolvedCutHistory::create();node->parent=job.history;node->owner=job.history->owner;node->damage_revision=job.stamp.revision;
   for(const auto&cap:caps)if(cap.piece==k)node->appended.push_back(cap.cutter);
   node->total_cuts=job.history->total_cuts+node->appended.size();node->limitations=job.history->limitations;
   node->grid_origin=job.history->grid_origin;node->grid_size=job.history->grid_size;node->grid_pitch=job.history->grid_pitch;
   pieceHistory[k]=std::move(node);}
  identity.snapshot=pieceHistory[k];
  painter_detail::ChannelFieldBuilder builder(identity,limitations);
  painter_detail::SkinOwnershipBuilder ownership(identity);
  for(const auto*r:records){auto resolved=*r;
   if(!resolved.edge&&!resolved.cracks){channelworker::ResolvedView view(resolved);resolved.edge_u={view.edge_u.x,view.edge_u.y,view.edge_u.z};resolved.edge_v={view.edge_v.x,view.edge_v.y,view.edge_v.z};}
   builder.append(resolved);}
  unsigned next=asset.original_children;bool anyChannel=false;
  for(std::size_t ci=0;ci<children.size();++ci){const auto&src=children[ci];const auto&list=work[ci].tris[k];const auto&extra=work[ci].extra[k];if(list.empty()&&extra.empty())continue;
   gfy_packed::Child child;child.material=src.material;child.material_source_child=src.material_source_child;child.skin_clip=!src.material&&src.skin_clip;
   if(!src.material&&src.child_index<asset.original_children)child.child_index=src.child_index;
   else{need(next<gfy_packed::max_child_slots,"lowpoly_piece_child_slots");child.child_index=std::uint16_t(next++);}
   const bool faces=src.original_faces.size()*3==src.indices.size();
   std::vector<packedchannel::OwnerTriangle> rows;
   const auto*prim=src.material&&field?field->child(src.child_index):nullptr;
   if(extra.empty()&&list.size()*3==src.indices.size()){ // the whole child: its rows as they are
    child.vertices=src.vertices;child.indices=src.indices;child.original_faces=src.original_faces;
    if(!src.material)for(std::size_t t=0;t<src.indices.size()/3;++t)ownership.append(child.child_index,published->skin_ownership.get(),src.child_index,t);
    if(prim){rows.resize(list.size());for(std::size_t t=0;t<list.size();++t)rows[t]={prim->data[t][0],static_cast<packedchannel::PrimitiveKind>(prim->data[t][1])};}
    if(src.material){builder.addChild(child.child_index,rows);anyChannel=true;}
    mesh.children.push_back(std::move(child));continue;}
   std::vector<std::int32_t> map(src.vertices.size(),-1);
   for(const auto t:list){for(int j=0;j<3;++j){const auto v=src.indices[3*t+j];
     if(map[v]<0){map[v]=std::int32_t(child.vertices.size());child.vertices.push_back(src.vertices[v]);}
     child.indices.push_back(std::uint16_t(map[v]));}
    child.original_faces.push_back(faces?src.original_faces[t]:~std::uint32_t(0));if(!src.material)ownership.append(child.child_index,published->skin_ownership.get(),src.child_index,t);
    if(prim)rows.push_back({prim->data[t][0],static_cast<packedchannel::PrimitiveKind>(prim->data[t][1])});}
   for(const auto&x:extra){if(child.vertices.size()+3>65535)break;
    for(int j=0;j<3;++j){auto row=x.v[std::size_t(j)];double sum=0;for(auto q:row.weight)sum+=q;if(sum>0)for(auto&q:row.weight)q=decltype(+q)(q/sum);
     child.indices.push_back(std::uint16_t(child.vertices.size()));child.vertices.push_back(row);}
    child.original_faces.push_back(faces?src.original_faces[x.source]:~std::uint32_t(0));if(!src.material)ownership.append(child.child_index,published->skin_ownership.get(),src.child_index,x.source);
    if(prim)rows.push_back({prim->data[x.source][0],static_cast<packedchannel::PrimitiveKind>(prim->data[x.source][1])});}
   if(child.indices.empty())continue;
   if(src.material){builder.addChild(child.child_index,rows);anyChannel=true;}
   mesh.children.push_back(std::move(child));}
  cycChildren+=aclock.lap();
  // caps of this piece: one synthetic tear cutter each, after the history's cuts
  unsigned owner=unsigned(records.size());
  for(auto&cap:caps){if(cap.piece!=k)continue;builder.append(cap.cutter);
   gfy_packed::Child child;child.material=1;child.material_source_child=std::uint16_t(materialSource);need(next<gfy_packed::max_child_slots,"lowpoly_piece_child_slots");child.child_index=std::uint16_t(next++);
   std::vector<packedchannel::OwnerTriangle> rows;
   for(const auto&t:cap.rimTris){for(int q=0;q<3;++q){const auto&row=t[std::size_t(q)];core::JoinedVertex j;j.p=position(row);j.normal=cap.d;j.tangent=std::abs(cap.d.x)>.9?Vec{0,0,1}:Vec{1,0,0};j.binormal=cross(j.normal,j.tangent);
     j.uv={dot(j.p,j.tangent)/.08,dot(j.p,j.binormal)/.08};bool posed=true;
     for(unsigned w=0;w<4;++w)if(row.weight[w]>separation_detail::negligible_skin_weight&&(row.bone[w]>=64||!((anchorMask[k]>>row.bone[w])&1u)))posed=false;
     if(posed){for(unsigned w=0;w<4;++w)if(row.weight[w]>0&&row.bone[w]<64)j.w[row.bone[w]]+=row.weight[w];}else j.w[cap.bone]=1;
     child.indices.push_back(std::uint16_t(child.vertices.size()));child.vertices.push_back(core::packing::vertex(j));}
    child.original_faces.push_back(~std::uint32_t(0));rows.push_back({owner,packedchannel::PrimitiveKind::entry_cap});
    if(child.vertices.size()+3>65535)break;}
   if(!cap.sealed)for(const auto&t:cap.tris){for(int q=0;q<3;++q){core::JoinedVertex j;j.p=t[std::size_t(q)];j.normal=cap.d;j.tangent=std::abs(cap.d.x)>.9?Vec{0,0,1}:Vec{1,0,0};j.binormal=cross(j.normal,j.tangent);
     j.uv={dot(j.p,j.tangent)/.08,dot(j.p,j.binormal)/.08};j.w[cap.bone]=1;
     child.indices.push_back(std::uint16_t(child.vertices.size()));child.vertices.push_back(core::packing::vertex(j));}
    child.original_faces.push_back(~std::uint32_t(0));rows.push_back({owner,packedchannel::PrimitiveKind::entry_cap});
    if(child.vertices.size()+3>65535)break;}
   ++owner;builder.addChild(child.child_index,rows);anyChannel=true;mesh.children.push_back(std::move(child));}
  gfy_packed::set_skin_ownership(mesh,ownership.finish());
  cycCaps+=aclock.lap();if(anyChannel)mesh.analytic_channels=builder.finish();else mesh.analytic_coverage_admitted=false;cycField+=aclock.lap();
  // removal field: the committed damage within the piece's bounds (+2 tiles)
  struct FieldTiles {std::unordered_map<Cell,core::Tile,core::CellHash> tiles;const void* ownership=nullptr;double pitch;Vec origin;decltype(grid.size) size;};
  FieldTiles masked{{},nullptr,grid.pitch,grid.origin,grid.size};
  if(!grid.tiles.empty()){Cell lo{INT_MAX,INT_MAX,INT_MAX},hi{INT_MIN,INT_MIN,INT_MIN};
   for(const auto&ch:mesh.children)for(const auto&v:ch.vertices){const Cell c{int(std::floor((v.surface[0]-grid.origin.x)/grid.pitch))>>3,int(std::floor((v.surface[1]-grid.origin.y)/grid.pitch))>>3,int(std::floor((v.surface[2]-grid.origin.z)/grid.pitch))>>3};
    lo={std::min(lo.x,c.x),std::min(lo.y,c.y),std::min(lo.z,c.z)};hi={std::max(hi.x,c.x),std::max(hi.y,c.y),std::max(hi.z,c.z)};}
   lo={lo.x-2,lo.y-2,lo.z-2};hi={hi.x+2,hi.y+2,hi.z+2};
   for(const auto&it:grid.tiles){const Cell&t=it.first;if(t.x<lo.x||t.y<lo.y||t.z<lo.z||t.x>hi.x||t.y>hi.y||t.z>hi.z)continue;
    std::uint64_t any=0;for(auto w:it.second.removed)any|=w;if(any)masked.tiles.emplace(t,it.second);}}
  mesh.removal_field=painter_detail::buildDamageField(masked);need(mesh.removal_field&&mesh.removal_field->available,"lowpoly_piece_removal_field");
  if(!birth)separation_detail::pruneNegligibleSkinWeights(mesh);cycRemoval+=aclock.lap();} // the intact publication was pruned when cached; split and cap rows carry pruned weights
 lap(rep.cap_ms);if(profiling())profileText()<<"instant_prof assembly_mcycles children="<<cycChildren<<" caps="<<cycCaps<<" field="<<cycField<<" removal="<<cycRemoval<<'\n';mark("assembly");
 // ---- 8. colliders, pose anchors, accessories
 std::map<std::uint16_t,std::uint64_t> remainingByElement,originalCells;
 for(unsigned e=0;e<64;++e)if(isElement[e]){remainingByElement[std::uint16_t(e)]=ix.cells[e];originalCells[std::uint16_t(e)]=ix.cells[e];}
 for(std::size_t k=0;k<P;++k){auto&piece=pieces[k];const auto&elements=elementsOf[k];
  for(const auto e:elements){SeparationElement element;const Matrix&toAnchor=frames->bones[e].rest_to_anchor;
   if(divided[e]&&pieceOwners[e]>1)element=fittedElement(e,parts[k][e],pitch,toAnchor);
   else if(elements.size()>1){element.bone=e;element.retain_original_shape=true;element.cells=cellsOf[k][e];}
   else{Moments m;for(const auto&cc:ix.coarse[e])addCoarse(m,cc,toAnchor);element=fittedElement(e,m,pitch,toAnchor);}
   piece.cells+=cellsOf[k][e];piece.elements.push_back(std::move(element));}
  piece.root_bone=separation_detail::root(elements,parents);
  piece.pose_anchors.assign(bones,65535);std::vector<std::uint16_t> resolved(bones,65535);
  for(const auto&child:piece.mesh.children)for(const auto&vertex:child.vertices)for(unsigned w=0;w<4;++w)if(vertex.weight[w]>separation_detail::negligible_skin_weight){
   const auto bone=vertex.bone[w];need(bone<bones,"separation_vertex_bone");if(resolved[bone]!=65535){if(!elements.count(bone))piece.pose_anchors[bone]=resolved[bone];continue;}
   const auto e=elementOfBone(bone);need(e<64,"separation_weighted_unphysical_root");
   const auto anchor=anchorOf(e,elements,parents);need(anchor!=65535,"separation_skin_without_unique_original_neighbor");resolved[bone]=anchor;if(!elements.count(bone))piece.pose_anchors[bone]=anchor;}
  for(const auto&a:sep.accessories){const auto e=a.physical_element;if(e>=64||!elements.count(e))continue;
   if(own.of(vec(a.original_anchor),e,false)==int(k))piece.retained_accessories.push_back(a.child);}
  separation_detail::admitNativeShell(piece,pieceOwners,remainingByElement,originalCells,parents);
  gearBodyFlesh(asset,piece,*frames);} // gearbody: the merged gear this part holds, as flesh for the bullet test
 lap(rep.colliders_ms);mark("colliders");
 // Cut receipts: the cut that made each non-stump piece (normal into the piece) and the bone its near side keeps there.
 for(std::size_t q=0;q<order.size();++q){const Cut&c=cuts[order[q]];auto&piece=pieces[std::size_t(c.piece)];
  piece.cut_point=point(c.point);piece.cut_normal=point(c.normal);
  const auto near=std::size_t(own.of(c.point-c.normal*.004,c.element,false));std::uint16_t keep=c.element;unsigned guard=0;
  while(keep<64&&!elementsOf[near].count(keep)&&guard++<64)keep=keep<parents.size()?parents[keep]:std::uint16_t(65535);
  piece.stump_bone=keep<64?keep:pieces[near].root_bone;
  // The batch's impact that severed it: the latest crater reaching the tear's disc (a row: its span).
  std::uint64_t seq=0;const double reach=c.R+.008;
  for(const auto&cr:craters){if(cr.sequence<=seq||c.point.x+reach<cr.lo.x||c.point.y+reach<cr.lo.y||c.point.z+reach<cr.lo.z||c.point.x-reach>cr.hi.x||c.point.y-reach>cr.hi.y||c.point.z-reach>cr.hi.z)continue;
   if(classifyBall(cr.cut,c.point,reach)!=-1)seq=cr.sequence;}
  if(!seq)for(const auto&e:input.impacts)seq=std::max(seq,e.sequence);
  piece.sever_sequence=seq;}
 // ---- 9. seeds (completed by the catch-up: grid restricted to the piece and its topology)
 const auto faceTable=faceTableOf(asset);const auto&faceOf=*faceTable;
 auto torn=std::make_shared<SeverState>();torn->failed=job.failed;torn->failed.resize(stationCount(asset),0);torn->table=asset.tile_material.get();
 {std::size_t kk=0;for(const auto&s:asset.sever_segments)for(const auto&st:s.stations){for(const auto&c:cuts)if(c.seg==&s&&std::abs(st.s-c.s)<=s.radius)torn->failed[kk]=1;++kk;}}
 auto plan=std::make_shared<InstantPlan>();plan->asset=job.asset;plan->grid=job.grid;plan->elements=index;plan->pieces=P;
 for(const auto i:order){InstantPlan::Plane pl;pl.point=cuts[i].point;pl.normal=cuts[i].normal;pl.split=cuts[i].split;pl.distal=cuts[i].distal;pl.piece=cuts[i].piece;plan->planes.push_back(pl);}
 for(const auto&cap:caps)plan->bands.push_back({cap.piece,cap.planePoint,cap.d,cap.cutter.radius});
 plan->mesh_tiles.assign(P,{{INT_MAX,INT_MAX,INT_MAX,INT_MIN,INT_MIN,INT_MIN}});
 for(std::size_t k=0;k<P;++k){auto&b=plan->mesh_tiles[k];const double inv=1./grid.pitch;
  for(const auto&ch:pieces[k].mesh.children)for(const auto&v:ch.vertices){const int c[3]={int(std::floor((v.surface[0]-grid.origin.x)*inv))>>3,int(std::floor((v.surface[1]-grid.origin.y)*inv))>>3,int(std::floor((v.surface[2]-grid.origin.z)*inv))>>3};
   for(int q=0;q<3;++q){b[std::size_t(q)]=std::min(b[std::size_t(q)],c[q]);b[std::size_t(q)+3]=std::max(b[std::size_t(q)+3],c[q]);}}}
 for(std::size_t k=0;k<P;++k){auto&piece=pieces[k];auto seed=std::make_shared<PieceSeed>();seed->asset=job.asset;seed->parent=job.stamp;seed->history=pieceHistory[k];seed->sever=torn;seed->root_bone=piece.root_bone;
  seed->instant=plan;seed->instant_piece=std::uint32_t(k);
  seed->faces.assign(asset.source.f.size(),0);std::array<char,64> used{};
  for(const auto&child:piece.mesh.children){
   for(const auto&v:child.vertices)for(unsigned w=0;w<4;++w)if(v.weight[w]>separation_detail::negligible_skin_weight&&v.bone[w]<64)used[v.bone[w]]=1;
   if(child.material||child.original_faces.size()*3!=child.indices.size())continue;
   if(child.material_source_child>=faceOf.size())continue;const auto&table=faceOf[child.material_source_child];
   for(const auto f:child.original_faces)if(f<table.size()&&table[f]!=~std::uint32_t(0))seed->faces[table[f]]=1;}
  std::set<std::uint16_t> held;for(const auto&e:piece.elements)held.insert(e.bone);
  for(unsigned b=0;b<64;++b){std::uint16_t r=std::uint16_t(b);
   if(!held.count(r)&&!used[b]){r=b<piece.pose_anchors.size()?piece.pose_anchors[b]:std::uint16_t(65535);
    if(r==65535&&b<bones&&frames->bones[b].physical_element>=0){
     r=anchorOf(std::uint16_t(frames->bones[b].physical_element),held,parents);}
    if(r==65535||!held.count(r))r=piece.root_bone;}
   seed->render_bone[b]=r;}
  piece.seed=std::move(seed);}
 // ---- output
 std::uint64_t largest=0;for(const auto&p:pieces)if(p.cells>largest){largest=p.cells;out->source_component=p.component;}
 out->original_surface_children=surfaceChildrenOf(asset,sep);
 {std::set<std::uint16_t> accessoryChildren;for(const auto&a:sep.accessories)accessoryChildren.insert(a.child);
  need(out->original_surface_children.size()+accessoryChildren.size()==asset.original_children,"separation_missing_original_child_ownership");}
 for(auto&p:pieces){need(!p.mesh.children.empty(),"lowpoly_piece_empty");out->pieces.push_back(std::move(p));}
 out->original_components=1;out->remaining_components=std::uint32_t(out->pieces.size());out->largest_substantial_split=std::uint32_t(out->pieces.size());
 for(const auto i:order){const Cut&c=cuts[i];InstantSplitStation s;s.bone=c.seg?c.seg->bone:std::uint16_t(65535);s.s=c.s;s.tissue=c.tissue;s.bone_left=c.bone;s.dangling=c.dangling;s.guided=c.guided;s.row=c.row;
  s.point=point(c.point);s.normal=point(c.normal);rep.torn.push_back(s);}
 lap(rep.seeds_ms);rep.total_ms=core::ms(t0);mark("seeds_output");
 if(profiling()){auto&text=profileText();std::cerr<<text.str()<<std::flush;text.str("");text.clear();}
 out->phase_ms[separation_phase_remaining_connectivity]=rep.resolve_ms+rep.verdict_ms;out->phase_ms[separation_phase_surface_partition]=rep.geometry_ms+rep.cap_ms;
 out->phase_ms[separation_phase_colliders_and_pose]=rep.colliders_ms+rep.seeds_ms;
 char text[400];std::snprintf(text,sizeof(text),"instant pieces=%zu cuts=%zu caps=%zu split=%zu unanchored=%zu dropped=%zu craters=%zu/%zu stations=%zu exact=%zu samples=%zu resolve_ms=%.2f verdict_ms=%.2f geometry_ms=%.2f cap_ms=%.2f colliders_ms=%.2f seeds_ms=%.2f",
  out->pieces.size(),cuts.size(),caps.size(),splitCount,unanchored,dropped,rep.resolved,rep.craters,rep.stations_measured,rep.exact_stations,rep.samples,rep.resolve_ms,rep.verdict_ms,rep.geometry_ms,rep.cap_ms,rep.colliders_ms,rep.seeds_ms);
 out->lowpoly_summary=text;
 return out;
}
// ---- the catch-up: every piece's seed completed from the snapshot the split was made from ---------------------------
// One pass over the anatomy columns (damage lattice) in bands of 8 rows side by side: each column's solid runs are cut
// at the physics element runs, at the planes of the cuts dividing that element and at the committed removed cells; the
// kept spans go to the owning piece (PieceOwnership and topology, one component, as ownerAndTopology builds them).
// Then per piece, side by side: the snapshot's damage cropped to the piece and its mesh (+2 tiles) with that ownership
// (tiles holding no other piece's tissue are pure), and a marks band - the cells just beyond each of its cut planes,
// within the cap, read as removed in its render field.
inline std::shared_ptr<SeparationOutput> completeSeeds(const InstantPlan&plan,const SeparationOutput&committed){
 const auto t0=core::Clock::now();const Grid&g=*plan.grid;need(g.anatomy&&!g.ownership,"instant_seed_grid");
 const auto&a=*g.anatomy;const auto&ix=*plan.elements;const auto&map=*ix.map;
 need(std::abs(a.pitch-g.pitch)<1e-12&&std::abs(map.pitch-g.pitch)<1e-12,"instant_seed_pitch");
 int off[3],moff[3];
 for(int k=0;k<3;++k){const double o=(a.origin[k]-g.origin[k])/g.pitch,m=(map.origin[k]-g.origin[k])/g.pitch;off[k]=int(std::lround(o));moff[k]=int(std::lround(m));
  need(std::abs(o-off[k])<1e-6&&std::abs(m-moff[k])<1e-6,"instant_seed_lattice");}
 const std::size_t P=plan.pieces;need(P>=2&&P<=16&&committed.pieces.size()==P&&plan.mesh_tiles.size()==P,"instant_seed_pieces");
 for(std::size_t p=0;p<P;++p){const auto&s=committed.pieces[p].seed;need(s&&s->instant.get()==&plan&&s->instant_piece==p&&committed.pieces[p].component==p+1,"instant_seed_piece_order");}
 const int TX=(g.size.x+7)/8,TY=(g.size.y+7)/8,TZ=(g.size.z+7)/8;std::vector<std::uint16_t> tileMask(std::size_t(TX)*TY*TZ,0);
 auto tileIndex=[&](int tx,int ty,int tz){return (std::size_t(tx)*TY+std::size_t(ty))*TZ+std::size_t(tz);};
 std::unordered_map<std::uint64_t,std::vector<std::pair<int,const core::Tile*>>> damage; // committed removed cells per tile column
 for(const auto&t:g.tiles){std::uint64_t any=0;for(auto w:t.second.removed)any|=w;if(!any)continue;
  damage[(std::uint64_t(std::uint32_t(t.first.x))<<32)|std::uint32_t(t.first.y)].push_back({t.first.z,&t.second});}
 for(auto&d:damage)std::sort(d.second.begin(),d.second.end(),[](const auto&l,const auto&r){return l.first<r.first;});
 struct Part {std::vector<core::OwnerColumn> cols;std::vector<core::OwnerSpan> spans;std::uint64_t cells{};
  std::array<int,3> lo{{INT_MAX,INT_MAX,INT_MAX}},hi{{INT_MIN,INT_MIN,INT_MIN}};int lastX=INT_MIN,lastY=INT_MIN;};
 const std::size_t bands=std::size_t(TY);std::vector<std::vector<Part>> parts(bands,std::vector<Part>(P));
 std::vector<std::uint64_t> bandAnatomy(bands,0),bandCovered(bands,0);
 const double scanStart=core::ms(t0);
 // A band is one row of tiles (8 damage rows): its tile mask entries are its own.
 sep::parallel(bands,2,sep::max_job_threads,[&](std::size_t first,std::size_t last,std::size_t){
  std::vector<std::pair<int,int>> solid;const std::size_t words=(std::size_t(TZ)+63)/64;std::vector<std::uint64_t> cover(P*std::size_t(TX)*words);
  for(std::size_t band=first;band<last;++band){auto&part=parts[band];std::uint64_t anatomyCells=0,coveredCells=0;std::fill(cover.begin(),cover.end(),0);
   auto append=[&](std::size_t p,int x,int y,int z0,int z1){if(z1<=z0)return;auto&B=part[p];
    if(B.lastX!=x||B.lastY!=y){B.cols.push_back({x,y,std::uint32_t(B.spans.size()),0});B.lastX=x;B.lastY=y;}
    if(B.cols.back().count&&B.spans.back().hi==z0)B.spans.back().hi=z1;else{B.spans.push_back({z0,z1});++B.cols.back().count;}
    B.cells+=std::uint64_t(z1-z0);B.lo={std::min(B.lo[0],x),std::min(B.lo[1],y),std::min(B.lo[2],z0)};B.hi={std::max(B.hi[0],x+1),std::max(B.hi[1],y+1),std::max(B.hi[2],z1)};
    const int t0=std::max(0,z0>>3),t1=std::min(TZ-1,(z1-1)>>3);if(t1<t0)return;std::uint64_t*w=&cover[(p*std::size_t(TX)+std::size_t(x>>3))*words];
    for(int i=t0>>6;i<=t1>>6;++i){const int a0=std::max(t0,i*64)-i*64,a1=std::min(t1,i*64+63)-i*64;w[i]|=(a1==63?~std::uint64_t(0):((std::uint64_t(1)<<(a1+1))-1))&~((std::uint64_t(1)<<a0)-1);}};
   for(int y=int(band)*8;y<std::min(g.size.y,int(band)*8+8);++y){const int ay=y-off[1],my=y-moff[1];if(ay<0||ay>=int(a.size[1])||my<0||my>=int(map.size[1]))continue;
    for(int ax=0;ax<int(a.size[0]);++ax){const auto col=std::size_t(ay)*a.size[0]+std::size_t(ax);if(a.offsets[col]==a.offsets[col+1])continue;
     const int x=ax+off[0],mx=x-moff[0];if(x<0||x>=g.size.x||mx<0||mx>=int(map.size[0]))continue;
     const auto mcol=std::size_t(my)*map.size[0]+std::size_t(mx);const auto mfirst=map.offsets[mcol],mlast=map.offsets[mcol+1];
     const auto dit=damage.find((std::uint64_t(std::uint32_t(x>>3))<<32)|std::uint32_t(y>>3));const std::vector<std::pair<int,const core::Tile*>>*dlist=dit==damage.end()?nullptr:&dit->second;
     const unsigned shift=unsigned((y&7)*8);
     auto removedAt=[&](int z){for(const auto&d:*dlist)if(d.first==(z>>3))return ((d.second->removed[std::size_t(x&7)]>>(shift+unsigned(z&7)))&1u)!=0;return false;};
     auto emit=[&](std::size_t p,int z0,int z1){ // the span minus committed removed cells
      if(!dlist){append(p,x,y,z0,z1);return;}int start=z0;for(int z=z0;z<z1;++z)if(removedAt(z)){append(p,x,y,start,z);start=z+1;}append(p,x,y,start,z1);};
     solid.clear(); // the column's tissue as solid runs (labels do not matter for ownership)
     for(auto r=a.offsets[col];r<a.offsets[col+1];++r){if(!a.runs[r].label)continue;const int z0=int(a.runs[r].lo)+off[2],z1=int(a.runs[r].hi)+off[2];
      if(!solid.empty()&&solid.back().second==z0)solid.back().second=z1;else solid.push_back({z0,z1});}
     const Vec base=g.center(Cell{x,y,0});
     for(const auto&run:solid){const int z0=run.first,z1=run.second;anatomyCells+=std::uint64_t(z1-z0);
      for(auto m=mfirst;m<mlast;++m){const int e0=std::max(z0,int(map.runs[m].lo)+moff[2]),e1=std::min(z1,int(map.runs[m].hi)+moff[2]);if(e1<=e0)continue;coveredCells+=std::uint64_t(e1-e0);
       const std::uint16_t e=map.runs[m].element;int zs[18];int nz=0;zs[nz++]=e0;zs[nz++]=e1;
       if(e<64)for(const auto&c:plan.planes){if(!c.split[e]||nz>=18)continue;const double A=dot(base-c.point,c.normal),B=c.normal.z*g.pitch;if(std::abs(B)<1e-15)continue;
        const int z=int(std::ceil(-A/B));if(z>e0&&z<e1)zs[nz++]=z;}
       if(nz>2)std::sort(zs,zs+nz);
       for(int q=0;q+1<nz;++q){if(zs[q+1]<=zs[q])continue;const int p=plan.of(g.center(Cell{x,y,(zs[q]+zs[q+1])/2}),e);emit(std::size_t(p),zs[q],zs[q+1]);}}}}}
   for(std::size_t p=0;p<P;++p)for(int tx=0;tx<TX;++tx){const std::uint64_t*w=&cover[(p*std::size_t(TX)+std::size_t(tx))*words];
    for(std::size_t i=0;i<words;++i)for(std::uint64_t bits=w[i];bits;bits&=bits-1){unsigned long b=0;
#ifdef _MSC_VER
     _BitScanForward64(&b,bits);
#else
     b=static_cast<unsigned long>(__builtin_ctzll(bits));
#endif
     tileMask[tileIndex(tx,int(band),int(i*64+b))]|=std::uint16_t(1u<<p);}}
   bandAnatomy[band]=anatomyCells;bandCovered[band]=coveredCells;}});
 const double scanMs=core::ms(t0)-scanStart;
 std::uint64_t anatomyCells=0,coveredCells=0;for(std::size_t b=0;b<bands;++b){anatomyCells+=bandAnatomy[b];coveredCells+=bandCovered[b];}
 std::vector<std::shared_ptr<PieceSeed>> seeds(P);std::vector<std::uint64_t> cellsOf(P,0);std::vector<std::size_t> markTiles(P,0);std::vector<std::exception_ptr> failed(P);
 sep::parallel(P,1,sep::max_job_threads,[&](std::size_t first,std::size_t last,std::size_t){
  for(std::size_t p=first;p<last;++p)try{
   std::size_t nc=0,ns=0;for(const auto&band:parts){nc+=band[p].cols.size();ns+=band[p].spans.size();}
   auto owner=std::make_shared<core::PieceOwnership>();owner->body_columns.reserve(nc);owner->body_spans.reserve(ns);
   gfy_connectivity::Topology top;top.columns.reserve(nc);top.spans.reserve(ns);
   std::uint64_t cells=0;std::array<int,3> lo{{INT_MAX,INT_MAX,INT_MAX}},hi{{INT_MIN,INT_MIN,INT_MIN}};
   for(const auto&band:parts){const auto&B=band[p];const auto base=std::uint32_t(owner->body_spans.size());
    for(const auto&c:B.cols){owner->body_columns.push_back({c.x,c.y,c.first+base,c.count});top.columns.push_back({c.x,c.y,c.first+base,c.count});}
    for(const auto&s:B.spans){owner->body_spans.push_back(s);top.spans.push_back({s.lo,s.hi,1});}
    cells+=B.cells;for(int q=0;q<3;++q){lo[std::size_t(q)]=std::min(lo[std::size_t(q)],B.lo[std::size_t(q)]);hi[std::size_t(q)]=std::max(hi[std::size_t(q)],B.hi[std::size_t(q)]);}}
   need(cells>0,"instant_seed_piece_without_tissue");cellsOf[p]=cells;
   const auto&mt=plan.mesh_tiles[p]; // the piece's tissue and its mesh, +2 tiles (as the ordinary seeds are cropped)
   const Cell tlo{std::min(lo[0]>>3,mt[0])-2,std::min(lo[1]>>3,mt[1])-2,std::min(lo[2]>>3,mt[2])-2},thi{std::max((hi[0]-1)>>3,mt[3])+2,std::max((hi[1]-1)>>3,mt[4])+2,std::max((hi[2]-1)>>3,mt[5])+2};
   owner->pure_lo=tlo;owner->pure_size={thi.x-tlo.x+1,thi.y-tlo.y+1,thi.z-tlo.z+1};
   const std::size_t n=std::size_t(owner->pure_size.x)*std::size_t(owner->pure_size.y)*std::size_t(owner->pure_size.z);owner->pure.assign((n+63)/64,0);
   for(int tx=tlo.x;tx<=thi.x;++tx)for(int ty=tlo.y;ty<=thi.y;++ty)for(int tz=tlo.z;tz<=thi.z;++tz){
    const bool mine=tx<0||ty<0||tz<0||tx>=TX||ty>=TY||tz>=TZ||(tileMask[tileIndex(tx,ty,tz)]&~std::uint16_t(1u<<p))==0;if(!mine)continue;
    const std::size_t i=(std::size_t(tx-tlo.x)*std::size_t(owner->pure_size.y)+std::size_t(ty-tlo.y))*std::size_t(owner->pure_size.z)+std::size_t(tz-tlo.z);owner->pure[i>>6]|=std::uint64_t(1)<<(i&63);}
   Grid seedGrid;seedGrid.size=g.size;seedGrid.origin=g.origin;seedGrid.pitch=g.pitch;seedGrid.base=g.base;seedGrid.anatomy=g.anatomy;seedGrid.material_slots=g.material_slots;seedGrid.ownership=std::move(owner);
   for(const auto&t:g.tiles)if(t.first.x>=tlo.x&&t.first.y>=tlo.y&&t.first.z>=tlo.z&&t.first.x<=thi.x&&t.first.y<=thi.y&&t.first.z<=thi.z)seedGrid.tiles.emplace(t.first,t.second);
   top.size={g.size.x,g.size.y,g.size.z};top.origin={g.origin.x,g.origin.y,g.origin.z};top.pitch=g.pitch;top.cells=cells;
   top.components.resize(2);top.components[1].cells=cells;top.components[1].lo=lo;top.components[1].hi=hi;
   auto seed=std::make_shared<PieceSeed>(*committed.pieces[p].seed);
   // Marks band: beyond each of this piece's cut planes (d points out of the piece), 4 mm deep, within the cap.
   for(const auto&band:plan.bands){if(band.piece!=p)continue;const double R=band.radius,depth=.004,h=g.pitch;
    const Vec u=unit(cross(band.d,std::abs(band.d.x)<.8?Vec{1,0,0}:Vec{0,1,0})),v=cross(band.d,u);
    Vec blo{1e300,1e300,1e300},bhi{-1e300,-1e300,-1e300};
    for(int cu=-1;cu<=1;cu+=2)for(int cv=-1;cv<=1;cv+=2)for(double s:{0.,depth}){const Vec q=band.point+u*(cu*R)+v*(cv*R)+band.d*s;
     blo={std::min(blo.x,q.x),std::min(blo.y,q.y),std::min(blo.z,q.z)};bhi={std::max(bhi.x,q.x),std::max(bhi.y,q.y),std::max(bhi.z,q.z)};}
    const Cell klo{int(std::floor((blo.x-g.origin.x)/h))>>3,int(std::floor((blo.y-g.origin.y)/h))>>3,int(std::floor((blo.z-g.origin.z)/h))>>3};
    const Cell khi{int(std::floor((bhi.x-g.origin.x)/h))>>3,int(std::floor((bhi.y-g.origin.y)/h))>>3,int(std::floor((bhi.z-g.origin.z)/h))>>3};
    const double reach=std::sqrt(3.)*4*h; // a tile's half diagonal
    for(int kx=std::max(0,klo.x);kx<=std::min(TX-1,khi.x);++kx)for(int ky=std::max(0,klo.y);ky<=std::min(TY-1,khi.y);++ky)for(int kz=std::max(0,klo.z);kz<=std::min(TZ-1,khi.z);++kz){
     const Vec c0=g.center(Cell{kx*8,ky*8,kz*8}),mid=c0+Vec{3.5*h,3.5*h,3.5*h};const double sm=dot(mid-band.point,band.d);
     if(sm<-reach||sm>depth+reach||length((mid-band.point)-band.d*sm)>R+reach)continue;
     core::Tile tile{};bool any=false;const bool rim=length((mid-band.point)-band.d*sm)+reach>R;
     const double s0=dot(c0-band.point,band.d),sx=band.d.x*h,sy=band.d.y*h,sz=band.d.z*h;
     for(int x=0;x<8;++x)for(int y=0;y<8;++y){const double sxy=s0+x*sx+y*sy;
      for(int z=0;z<8;++z){const double s=sxy+z*sz;if(!(s>0&&s<=depth))continue;
       if(rim){const Vec q=c0+Vec{x*h,y*h,z*h};if(length((q-band.point)-band.d*s)>R)continue;}
       const auto bit=Grid::local(Cell{x,y,z});tile.removed[bit/64]|=std::uint64_t(1)<<(bit%64);any=true;}}
     if(any)seed->marks.emplace_back(Cell{kx,ky,kz},tile);}}
   markTiles[p]=seed->marks.size();
   seed->grid=std::make_shared<const Grid>(std::move(seedGrid));seed->topology=std::make_shared<const gfy_connectivity::Topology>(std::move(top));
   seeds[p]=std::move(seed);}catch(...){failed[p]=std::current_exception();}});
 for(const auto&f:failed)if(f)std::rethrow_exception(f);
 auto out=std::make_shared<SeparationOutput>();out->stamp=committed.stamp;out->source_component=committed.source_component;
 out->original_components=committed.original_components;out->remaining_components=committed.remaining_components;out->largest_substantial_split=committed.largest_substantial_split;
 std::size_t marks=0;
 for(std::size_t p=0;p<P;++p){const auto&c=committed.pieces[p];SeparationPiece piece;piece.component=c.component;piece.cells=cellsOf[p];piece.root_bone=c.root_bone;
  piece.cut_point=c.cut_point;piece.cut_normal=c.cut_normal;piece.stump_bone=c.stump_bone;piece.sever_sequence=c.sever_sequence;piece.seed=std::move(seeds[p]);
  marks+=markTiles[p];out->pieces.push_back(std::move(piece));}
 out->worker_ms=core::ms(t0);out->instant.instant=true;
 char text[240];std::snprintf(text,sizeof(text),"instant catch_up pieces=%zu anatomy_cells=%llu outside_elements=%llu mark_tiles=%zu scan_ms=%.2f ms=%.2f",
  P,(unsigned long long)anatomyCells,(unsigned long long)(anatomyCells-coveredCells),marks,scanMs,out->worker_ms);
 out->lowpoly_summary=text;
 return out;
}
// ---- the instant-sever service: its own threads, so a blast's bodies never queue behind ordinary separations ------------
struct Service {
 std::mutex mutex;std::condition_variable wake;std::deque<std::function<void()>> jobs,later;bool stop{};std::vector<std::thread> workers;
 // One job per body on its own thread: 10 bodies of one blast run side by side (player floor 4C/8T -> 6, 6C/12T -> 8).
 static unsigned width(){if(const char*e=std::getenv("GFY_INSTANT_THREADS"))return unsigned(std::clamp(std::atoi(e),1,16));
  const unsigned hw=std::max(2u,std::thread::hardware_concurrency());return std::clamp(hw>4?hw-2:2u,2u,8u);}
 Service(){const unsigned n=width();for(unsigned i=0;i<n;++i)workers.emplace_back([this]{loop();});}
 void quit()noexcept{{std::lock_guard<std::mutex> l(mutex);stop=true;jobs.clear();later.clear();}wake.notify_all();for(auto&w:workers)if(w.joinable())w.join();} // quitfix
 ~Service(){quit();}
 void loop(){painterThreadEntry();core::painterInlineParallel=true; // a job is one thread: nested sections run inline
  for(;;){std::function<void()> job;bool urgent=false;{std::unique_lock<std::mutex> l(mutex);wake.wait(l,[&]{return stop||!jobs.empty()||!later.empty();});if(stop)return;
    urgent=!jobs.empty();auto&from=urgent?jobs:later;job=std::move(from.front());from.pop_front();}
   painterThreadEntry();
#if defined(_WIN32)
   // A split lands within frames: normal priority for its duration (worker threads may default below normal).
   const int previous=GetThreadPriority(GetCurrentThread());if(urgent&&previous<0)SetThreadPriority(GetCurrentThread(),0);
   try{job();}catch(...){}
   if(urgent&&previous<0)SetThreadPriority(GetCurrentThread(),previous);}}
#else
   try{job();}catch(...){}}}
#endif
 // urgent: a split (ahead of everything); otherwise a catch-up (behind every pending split)
 bool submit(std::function<void()> job,bool urgent=true){{std::lock_guard<std::mutex> l(mutex);auto&q=urgent?jobs:later;if(stop||q.size()>=64)return false;q.push_back(std::move(job));}wake.notify_one();return true;}
};
// Caches first, so they outlive the service threads at shutdown.
inline Service& service(){(void)separation_detail::pristineCache();(void)separation_detail::mapCache();static Service& s=::gfy_live::core::worker_quit::immortal<Service>(0);return s;} // quitfix: never destroyed
}
std::shared_ptr<SeparationPreparation>Session::instant_sever(InstantSeverInput input)noexcept{
 AllocationScope scope;
 try{auto&i=*impl_;auto job=std::make_shared<instant_detail::Job>();
  {std::lock_guard<std::mutex> lock(i.mutex);need(!i.canceled&&!i.failed,"instant_owner_state");need(!i.piece,"instant_piece_session");need(bool(i.committed),"instant_committed_snapshot");
   const auto&s=*i.committed->impl_;job->asset=i.asset;job->grid=s.grid;job->history=s.history;job->geometry=s.geometry;job->stamp=s.stamp;
   if(s.sever)job->failed=s.sever->failed;else if(i.sever_state)job->failed=i.sever_state->failed;}
  need(job->grid&&job->asset,"instant_snapshot_state");need(bool(job->history),"instant_requires_cut_history"); // the caps are cuts of the pieces' histories
  job->stamp.owner=impl_->owner;
  if(input.current_generation)job->stamp.expected_generation=input.current_generation;
  // The output's revision follows the committed one (the pieces' cap cuts are a history node after it, and a piece
  // session's own cuts follow from this revision on), or the impacts' when they were queued on this owner.
  {std::uint64_t last=0;for(const auto&e:input.impacts)last=std::max(last,e.sequence);job->stamp.revision=std::max<std::uint64_t>({job->stamp.revision+1,last,1});}
  job->input=std::move(input);
  auto state=std::make_shared<SeparationPreparation::Impl>();state->stamp=job->stamp;
  auto handle=std::shared_ptr<SeparationPreparation>(new SeparationPreparation(state));
  need(instant_detail::service().submit([state,job]{
   if(state->progress.canceled())return;
   const sep::CancelScope cancelScope(state->progress.cancel_flag());const sep::ActiveJob activeJob;
   const auto started=core::Clock::now();std::shared_ptr<SeparationOutput> out;
   try{state->progress.checkpoint("instant_sever");out=instant_detail::run(*job);}
   catch(const std::bad_alloc&){out.reset();memoryCounters.separation_oom.fetch_add(1,std::memory_order_relaxed);try{out=std::make_shared<SeparationOutput>();out->error=outOfMemoryText;}catch(...){}}
   catch(const std::exception&e){out.reset();try{out=std::make_shared<SeparationOutput>();out->error=e.what();}catch(...){}}
   catch(...){out.reset();try{out=std::make_shared<SeparationOutput>();out->error="unknown_instant_failure";}catch(...){}}
   if(out){out->stamp=state->stamp;out->worker_ms=core::ms(started);out->instant.instant=true;}
   state->progress.finish();std::lock_guard<std::mutex> lock(state->mutex);state->output=std::move(out);}),"instant_queue_full");
  return handle;
 }catch(const std::bad_alloc&){fail(outOfMemoryText);return {};}catch(const std::exception&e){fail(e.what());return {};}catch(...){fail("unknown_instant_sever_failure");return {};}
}
std::shared_ptr<SeparationPreparation>Session::instant_catch_up(std::shared_ptr<const SeparationOutput> committed)noexcept{
 AllocationScope scope;
 try{need(committed&&committed->error.empty()&&!committed->no_separation&&!committed->pieces.empty(),"instant_catch_up_output");
  const auto&front=committed->pieces.front();need(front.seed&&front.seed->instant,"instant_catch_up_plan");const auto plan=front.seed->instant;
  for(const auto&p:committed->pieces)need(p.seed&&p.seed->instant==plan,"instant_catch_up_plan");
  need(plan->asset==impl_->asset,"instant_catch_up_asset");
  auto state=std::make_shared<SeparationPreparation::Impl>();state->stamp=committed->stamp;
  auto handle=std::shared_ptr<SeparationPreparation>(new SeparationPreparation(state));
  // Behind every pending split on the instant-sever service; the pieces are side by side inside the job.
  need(instant_detail::service().submit([state,plan,committed]{
   if(state->progress.canceled())return;
   const sep::CancelScope cancelScope(state->progress.cancel_flag());const sep::ActiveJob activeJob;std::shared_ptr<SeparationOutput> out;
   try{state->progress.checkpoint("instant_catch_up");out=instant_detail::completeSeeds(*plan,*committed);}
   catch(const std::bad_alloc&){out.reset();memoryCounters.separation_oom.fetch_add(1,std::memory_order_relaxed);try{out=std::make_shared<SeparationOutput>();out->error=outOfMemoryText;}catch(...){}}
   catch(const std::exception&e){out.reset();try{out=std::make_shared<SeparationOutput>();out->error=e.what();}catch(...){}}
   catch(...){out.reset();try{out=std::make_shared<SeparationOutput>();out->error="unknown_instant_catch_up_failure";}catch(...){}}
   if(out){out->stamp=state->stamp;out->instant.instant=true;}
   state->progress.finish();std::lock_guard<std::mutex> lock(state->mutex);state->output=std::move(out);},false),"instant_queue_full");
  return handle;
 }catch(const std::bad_alloc&){fail(outOfMemoryText);return {};}catch(const std::exception&e){fail(e.what());return {};}catch(...){fail("unknown_instant_catch_up_failure");return {};}
}
}
