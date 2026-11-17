#pragma once
// Included after LiveDamageWorker.cpp's private immutable Asset/Grid types.
// All substantial work stays on this bounded CPU worker. No engine objects,
// Lua calls, physics shells or renderer allocations are accessed here.
#include <mutex>
#include "RemainingConnectivity.h"
#include "PartOwnerDecision.h"
#include "ShortMinMax.h"
#include <atomic>
#include "separation/BoundSurfacePartition.h"
#include "separation/SurfaceOptions.h"
#include "separation/ThreadBudget.h"
#include "separation/PieceCollider.h"
#include "separation/PhysicalElementPartition.h"
#include "separation/OriginalBoneFrames.h"
#include "separation/NativeShapePartition.h"
#include "separation/SeparationCache.h"
#include "separation/OriginalAccessorySurface.h"
#include "separation/OcularSourceSupport.h"
#include "EyeRemnantPolicy.h"
#include "separation/OcularCollider.h"

// Offline-harness hooks of the separation track (harness/sep_probe_rx.cpp):
// separation_detail::warm() and separation_detail::prepareSnapshot().
#define GFY_SEP_HARNESS_HOOKS 1
namespace gfy_live {
// gfyspeed3 (OcularAccessoryDamage_impl.h): the asset's decoded gear for a separation's gear surface (empty: not the same / none)
struct GearAccess {std::shared_ptr<const original_accessory::CollectMemo> collect;std::shared_ptr<const ocular_source::Map> ocular;};
inline GearAccess accessoryGear(const Asset&asset,const bound_surface::ExtraSurface&extra);
inline bool gs3On(){static const bool on=[]{const char*e=std::getenv("GFY_GS3");
#ifdef GFY_OWNERSAFE_FAULT
 if(std::getenv("GFY_OWNERSAFE_FAULT"))return false; // a fault-injection probe tests ownersafe's own statements
#endif
 return !e||std::atoi(e)!=0;}();return on;}
namespace separation_detail {
inline std::vector<std::uint8_t> read(const AssetFile&spec,AssetHashFunction hash){
 need(hash&& !spec.path.empty()&&spec.max_bytes>0&&spec.max_bytes<=640u*1024u*1024u,"separation_file_descriptor");
 std::ifstream f(spec.path,std::ios::binary|std::ios::ate);need(bool(f),"separation_file_open");
 const auto length=f.tellg();need(length>0&&std::uint64_t(length)<=spec.max_bytes,"separation_file_size");
 std::vector<std::uint8_t>bytes(static_cast<std::size_t>(length));f.seekg(0);
 f.read(reinterpret_cast<char*>(bytes.data()),std::streamsize(bytes.size()));need(bool(f),"separation_file_read");
 need(hash(bytes.data(),bytes.size())==spec.expected,"separation_file_hash");return bytes;
}
inline void installLimbRegions(const Asset&asset,const SeparationInput&input){
 if(asset.limbRegions())return;
 namespace pc=gfy_piece_collider;
 const auto shapes=read(input.physics_elements,input.hash);
 if(shapes.size()<8||std::memcmp(shapes.data(),"PHYSRAW1",8)!=0)return;
 const auto frames=pc::original_frames::decode(read(input.bone_frames,input.hash),asset.source_hash,input.fit_hash);
 const std::size_t nb=frames.bones.size();need(nb==asset.source.bones&&nb<=64&&input.physical_parents.size()==nb,"limb_regions_bones");
 auto catalog=std::make_shared<const pc::native_shape::Catalog>(pc::native_shape::decode(shapes,frames));
 std::map<std::uint16_t,std::vector<std::uint16_t>> kids;std::uint16_t top=65535;
 for(auto e:catalog->elements){need(e<nb,"limb_regions_element");const auto p=input.physical_parents[e];
  if(p<nb&&catalog->elements.count(p))kids[p].push_back(e);else top=e;}
 auto pathLength=[&](std::uint16_t e)->int{int n=1;for(;;){const auto it=kids.find(e);if(it==kids.end()||it->second.empty())return n;if(it->second.size()!=1)return -1;e=it->second[0];++n;}};
 std::array<std::uint8_t,64> chainOf{};std::uint8_t chains=0;
 for(const auto&k:kids){std::map<int,std::vector<std::uint16_t>> byLength;for(auto c:k.second){const int n=pathLength(c);if(n>=2)byLength[n].push_back(c);}
  for(const auto&g:byLength)if(g.second.size()>=2)for(auto r:g.second){if(chains>=30)break;const std::uint8_t id=++chains;
   for(std::uint16_t e=r;;){chainOf[e]=id;const auto it=kids.find(e);if(it==kids.end()||it->second.empty())break;e=it->second[0];}}}
 (void)top;if(chains<2)return;
 auto out=std::make_shared<fastwound::LimbRegions>();out->chains=chains;
 for(std::size_t b=0;b<nb;++b){const int e=frames.bones[b].physical_element;out->boneChain[b]=e>=0&&e<64?chainOf[std::size_t(e)]:0;}
 out->faces=asset.source.f.size();out->faceChain.resize(out->faces);
 for(std::size_t f=0;f<out->faces;++f){std::array<double,64> sum{};for(auto id:asset.source.f[f]){const auto&v=asset.source.v[id];for(unsigned k=0;k<4;++k)if(v.ids[k]<64&&v.w[k]>0)sum[v.ids[k]]+=v.w[k];}
  unsigned best=0;for(unsigned b=1;b<64;++b)if(sum[b]>sum[best])best=b;out->faceChain[f]=out->boneChain[best];}
 const auto limbChain=chainOf;
 out->classify=[catalog,limbChain](const Vec&p,std::uint8_t&own,std::uint8_t&ownChain,unsigned&near,double&gap){std::array<double,64> sc{};
  const auto e=pc::native_shape::classify(*catalog,pc::V{{p.x,p.y,p.z}},sc);own=e<64?std::uint8_t(e):255;ownChain=e<64?limbChain[e]:0;near=0;
  double m0=std::numeric_limits<double>::infinity();for(auto x:catalog->elements)if(x<64&&limbChain[x])m0=std::min(m0,sc[x]);
  for(auto x:catalog->elements)if(x<64&&limbChain[x]&&sc[x]<=m0+fastwound::limbChainMargin())near|=1u<<limbChain[x];gap=e<64?m0-sc[e]:1e300;};
 // skin2: per-chain scores (side, exact tear line), bone anchors and physical parents (refinement error bound), sibling
 // chains (the other chain hanging from the same element with the same length) and each limb bone's match in its sibling
 out->chainScores=[catalog,limbChain](const Vec&p,std::array<double,32>&best,std::uint8_t&own,std::uint8_t&ownChain){std::array<double,64> sc{};
  const auto e=pc::native_shape::classify(*catalog,pc::V{{p.x,p.y,p.z}},sc);own=e<64?std::uint8_t(e):255;ownChain=e<64?limbChain[e]:0;
  best.fill(std::numeric_limits<double>::infinity());for(auto x:catalog->elements)if(x<64){auto&b=best[limbChain[x]&31u];b=std::min(b,sc[x]);}};
 for(std::size_t b=0;b<nb;++b){const auto&m=frames.bones[b].anchor_to_rest.row_major;out->pivot[b]=Vec{m[3],m[7],m[11]};
  const auto q=input.physical_parents[b];out->parent[b]=q<nb?std::uint8_t(q):std::uint8_t(255);}
 {std::array<std::vector<std::uint16_t>,32> path;std::array<std::uint16_t,32> rootParent;rootParent.fill(65535);
  for(auto e:catalog->elements){if(e>=64||!chainOf[e])continue;const auto pe=input.physical_parents[e];if(pe<64&&chainOf[pe]==chainOf[e])continue;
   rootParent[chainOf[e]]=pe;for(std::uint16_t x=e;;){path[chainOf[e]].push_back(x);const auto it=kids.find(x);if(it==kids.end()||it->second.size()!=1)break;x=it->second[0];}}
  for(unsigned a=1;a<=chains&&a<32;++a){unsigned match=0,count=0;for(unsigned c=1;c<=chains&&c<32;++c)if(c!=a&&rootParent[c]==rootParent[a]&&path[c].size()==path[a].size()){match=c;++count;}
   if(count==1)out->sibling[a]=std::uint8_t(match);out->fromTop[a]=rootParent[a]==top?1:0;}
  out->mirrorBone.fill(255);
  for(std::size_t b=0;b<nb;++b){const int e=frames.bones[b].physical_element;if(e<0||e>=64)continue;const auto c=chainOf[std::size_t(e)];if(!c||!out->sibling[c])continue;
   const auto&pa=path[c];const auto&pb=path[out->sibling[c]];for(std::size_t i=0;i<pa.size()&&i<pb.size();++i)if(pa[i]==e){out->mirrorBone[b]=std::uint8_t(pb[i]);break;}}}
 std::lock_guard<std::mutex> lock(asset.limb_mutex);if(!asset.limb_regions)asset.limb_regions=std::move(out);
}
inline std::uint16_t root(const std::set<std::uint16_t>&elements,const std::vector<std::uint16_t>&parents){
 std::uint16_t found=65535;
 for(auto bone:elements){need(bone<parents.size(),"separation_physical_parent_count");
  if(!elements.count(parents[bone])){if(found!=65535&&std::getenv("GFY_M4_ROOTDBG")){std::fprintf(stderr,"M4ROOTS elements=");for(auto e:elements)std::fprintf(stderr,"%u,",unsigned(e));std::fprintf(stderr,"\n");} // probe read-out
   need(found==65535,"separation_multiple_physical_roots");found=bone;}}
 need(found!=65535,"separation_cyclic_physical_tree");return found;
}
// ---- Native admission contract of a separation piece ----
// The owner-barrier producer/submit (PainterDamagedFragmentProducer_impl.h fitted(), PainterDamagedFragment_impl.h
// submit()/valid_piece()/pose_covered()) refuses the WHOLE sever when any one piece violates these, so the CPU shapes
// each piece to satisfy them without weakening them.
// Skin weights at or below this are numerically void; the pose-anchor proof ignores them, so the packed piece must not
// carry them either (pose_covered requires every non-zero weight to be represented or anchored). Renormalized away.
constexpr double negligible_skin_weight=1e-6;
static_assert(negligible_skin_weight==original_accessory::publicationPruneThreshold,"Accessory repeat corner pruning must match publication");
inline void pruneNegligibleSkinWeights(gfy_packed::Mesh&mesh){
 const double prior=mesh.max_discarded_skin_weight;double largest=0;
 gfy_packed::validate_skin_ownership(mesh);
 for(auto&child:mesh.children){
  // Predicate carriers retain the authored row: pruning can move a GPU byte
  // across its rounding boundary and change the rendered cut-time surface.
  std::vector<bool> protectedRows(child.vertices.size(),false);
  const auto*table=mesh.skin_ownership?mesh.skin_ownership->child(child.child_index):nullptr;
  if(table)for(std::size_t t=0;t<child.indices.size()/3;++t)if(table->data[t][1])
   for(unsigned q=0;q<3;++q){const auto v=child.indices[3*t+q];need(v<protectedRows.size(),"skin_owner_vertex_index");protectedRows[v]=true;}
  for(std::size_t vi=0;vi<child.vertices.size();++vi){auto&v=child.vertices[vi];if(protectedRows[vi]||gfy_signed_hw::valid4(v.weight,true))continue;double kept=0,pruned=0;
  for(unsigned w=0;w<4;++w){if(v.weight[w]>0&&v.weight[w]<=negligible_skin_weight){pruned+=double(v.weight[w]);v.weight[w]=0;}kept+=v.weight[w];}
  if(pruned==0)continue;need(kept>.5,"separation_skin_weight_prune");
  for(unsigned w=0;w<4;++w)v.weight[w]=decltype(+v.weight[w])(v.weight[w]/kept);
  gfy_packed::canonical_weights(v);
  largest=std::max(largest,pruned);}}
 using Discarded=std::decay_t<decltype(mesh.max_discarded_skin_weight)>;
 mesh.max_discarded_skin_weight=Discarded(std::min(1.,prior+(1.-prior)*largest));
}
// valid_piece() rigid-body limits in the engine's float precision (inertia about the fitted COM, box half extents).
inline bool nativeElementAdmissible(const SeparationElement&e){
 if(e.retain_original_shape)return true;
 const float mass=float(e.mass_kg),s=.00001f;float i[9];for(unsigned k=0;k<9;++k)i[k]=float(e.inertia[k]);
 if(!std::isfinite(mass)||mass<=0.f)return false;
 for(float n:i)if(!std::isfinite(n))return false;
 if(i[0]<=s||i[4]<=s||i[8]<=s||std::abs(i[1]-i[3])>s||std::abs(i[2]-i[6])>s||std::abs(i[5]-i[7])>s||i[0]*i[4]-i[1]*i[3]<=s*s)return false;
 const double d=double(i[0])*(double(i[4])*i[8]-double(i[5])*i[7])-double(i[1])*(double(i[3])*i[8]-double(i[5])*i[6])+double(i[2])*(double(i[3])*i[7]-double(i[4])*i[6]);
 if(d<=1e-15)return false;
 for(const auto&b:e.boxes){const float h[3]={float(b.half_extents.x),float(b.half_extents.y),float(b.half_extents.z)};
  for(float v:h)if(!std::isfinite(v)||v<.005f||v>.75f)return false;}
 return true;
}
template<class Partitions>
inline std::optional<std::array<double,4>> bodyCutPlane(const gfy_connectivity::Topology&body,const Partitions&partitions,
                                                       std::uint32_t bodyComponent,std::uint16_t bone,const Matrix&restToAnchor){
 namespace pc=gfy_piece_collider;
 struct Sum{long double n{},s[3]{};};
 const auto add=[](Sum&m,const gfy_connectivity::Topology&t){const long double p=t.pitch;
  for(const auto&col:t.columns){const long double x=t.origin[0]+(col.x+.5L)*p,y=t.origin[1]+(col.y+.5L)*p;
   for(auto i=col.first;i<col.first+col.count;++i){const auto&s=t.spans[i];if(s.component!=1)continue;const long double n=s.hi-s.lo;
    m.n+=n;m.s[0]+=n*x;m.s[1]+=n*y;m.s[2]+=n*(t.origin[2]+p*(s.lo+s.hi)*.5L);}}};
 Sum kept,cut;add(kept,body);
 for(const auto&entry:partitions)if(entry.first!=bodyComponent)for(const auto&part:entry.second)if(part.bone==bone)add(cut,part.volume);
 if(kept.n<1||cut.n<1)return std::nullopt;
 pc::V a{},b{};for(int q=0;q<3;++q){a[q]=double(kept.s[q]/kept.n);b[q]=double(cut.s[q]/cut.n);}
 pc::V n=pc::sub(pc::transform(restToAnchor,b),pc::transform(restToAnchor,a));const double length=pc::norm(n);
 if(!(length>1e-4))return std::nullopt;
 n=pc::mul(n,1./length);
 const auto&m=restToAnchor.row_major; // n.x of a cell centre in the bone frame = w.rest + o
 const pc::V w{m[0]*n[0]+m[4]*n[1]+m[8]*n[2],m[1]*n[0]+m[5]*n[1]+m[9]*n[2],m[2]*n[0]+m[6]*n[1]+m[10]*n[2]};
 const double o=m[3]*n[0]+m[7]*n[1]+m[11]*n[2],p=body.pitch,step=w[2]*p;
 const auto base=[&](const gfy_connectivity::Column&col){return w[0]*(body.origin[0]+(col.x+.5)*p)+w[1]*(body.origin[1]+(col.y+.5)*p)+w[2]*(body.origin[2]+.5*p)+o;};
 double top=-1e300;
 for(const auto&col:body.columns){const double v0=base(col);
  for(auto i=col.first;i<col.first+col.count;++i){const auto&s=body.spans[i];if(s.component!=1)continue;
   top=std::max(top,std::max(v0+step*s.lo,v0+step*(s.hi-1)));}}
 if(!(top>-1e299))return std::nullopt;
 // d: the lowest level (within 5 cm of the top) with at most 1 cm3 of the body's cells above it - the spans reaching into those
 // 5 cm counted in closed form per level, 24 halvings
 const double band=.05;struct Run{double v0;int lo,hi;};std::vector<Run> topRuns;
 for(const auto&col:body.columns){const double v0=base(col);
  for(auto i=col.first;i<col.first+col.count;++i){const auto&s=body.spans[i];if(s.component!=1)continue;
   if(std::max(v0+step*s.lo,v0+step*(s.hi-1))>=top-band)topRuns.push_back({v0,s.lo,s.hi});}}
 const auto above=[&](double t){double cells=0; // cells of those spans with v > t
  for(const auto&r:topRuns){if(step>0){const double k=std::floor((t-r.v0)/step)+1;cells+=std::max(0.,double(r.hi)-std::max(double(r.lo),k));}
   else if(step<0){const double k=std::ceil((t-r.v0)/step);cells+=std::max(0.,std::min(double(r.hi),k)-double(r.lo));}
   else if(r.v0>t)cells+=double(r.hi-r.lo);}
  return cells;};
 const double allowed=1e-6/(p*p*p);double lo=top-band,hi=top;
 if(above(lo)>allowed)for(int q=0;q<24;++q){const double mid=.5*(lo+hi);if(above(mid)>allowed)lo=mid;else hi=mid;}
 else hi=lo;
 return std::array<double,4>{n[0],n[1],n[2],hi};
}
template<class Owners,class Remaining,class Original>
inline void admitNativeShell(SeparationPiece&piece,const Owners&owners,const Remaining&remaining,const Original&original,
                             const std::vector<std::uint16_t>&parents,const std::set<std::uint16_t>*adopters=nullptr){
 // 1. A divided element's gram-scale remnant beside a cut gets no rigid body of its own jointed to a kilogram
 //    neighbour; its skin is posed rigidly by the adjacent element (nearest physical ancestor in the piece, else its
 //    largest physical child). Only when the remaining elements still form one physical tree.
 for(bool changed=true;changed&&piece.elements.size()>1;){changed=false;
  for(std::size_t i=0;i<piece.elements.size()&&!changed;++i){const auto dropped=piece.elements[i].bone;
   if(nativeElementAdmissible(piece.elements[i]))continue;
   std::set<std::uint16_t>rest;for(const auto&o:piece.elements)if(o.bone!=dropped)rest.insert(o.bone);
   std::size_t roots=0;for(auto b:rest)roots+=!rest.count(parents.at(b));
   if(roots!=1)continue;
   std::uint16_t into=65535;
   for(auto p=parents.at(dropped);p<parents.size();p=parents[p])if(rest.count(p)){into=p;break;}
   if(into==65535){std::uint64_t best=0;for(const auto&o:piece.elements)if(o.bone!=dropped&&parents.at(o.bone)==dropped&&o.cells>best){best=o.cells;into=o.bone;}}
   if(into==65535)continue;
   for(auto&a:piece.pose_anchors)if(a==dropped)a=into;
   piece.pose_anchors.at(dropped)=into;
   piece.elements.erase(piece.elements.begin()+std::ptrdiff_t(i));piece.root_bone=root(rest,parents);changed=true;}}
 // 2. A lone crumb-sized piece keeps its true mass, centre and box shape; its rotational inertia is raised isotropically
 //    to the native rigid-body floor (tensor stays symmetric positive definite) and a sub-5 mm box extent to 5 mm.
 if(piece.elements.size()==1&&!nativeElementAdmissible(piece.elements[0])){auto&e=piece.elements[0];
  if(!e.retain_original_shape&&e.mass_kg>0&&std::isfinite(e.mass_kg)){const double floor=2e-5;e.inertia[0]+=floor;e.inertia[4]+=floor;e.inertia[8]+=floor;
   for(auto&b:e.boxes){b.half_extents.x=std::max(b.half_extents.x,.005);b.half_extents.y=std::max(b.half_extents.y,.005);b.half_extents.z=std::max(b.half_extents.z,.005);}}}
 // 3. One native shell holds boxes_per_shell boxes. When the piece's refits exceed it, an element wholly owned by
 //    this piece (wounded, not divided by the cut) uses its original native (ragdoll) shape instead, least damaged
 //    first. Divided elements always keep their cut fit; single-element pieces are never given the original shape.
 auto boxes=[&]{std::size_t n=0;for(const auto&e:piece.elements)n+=e.boxes.size();return n;};
 while(piece.elements.size()>1&&boxes()>painter_collision_limits::boxes_per_shell){SeparationElement*pick=nullptr;double least=2;
  for(auto&e:piece.elements){if(e.retain_original_shape||e.boxes.empty()||(adopters&&adopters->count(e.bone)))continue; // an adopter's original shape does not cover its folded remnant
   const auto o=owners.find(e.bone);const auto r=remaining.find(e.bone);const auto c=original.find(e.bone);
   if(o==owners.end()||o->second!=1||r==remaining.end()||c==original.end()||!c->second)continue;
   const double damage=1.-double(r->second)/double(c->second);if(damage<least){least=damage;pick=&e;}}
  if(!pick)break;
  pick->retain_original_shape=true;pick->boxes.clear();pick->mass_kg=0;pick->com={};pick->inertia={};}
}
// ---- Per-(model, quality) caches and the warm path (separation-track P1.3) ----
// Both caches hold pure functions of immutable asset data, keyed exactly as the
// int14t caches (same key fields, same byte budgets: 256 MB topology, 128 MB
// map). Changes: at most 4 ready entries each (was 2), single-flight (a request
// joins a build already in flight instead of starting a second one) and
// joinable bands (a real job joins a warm build with its own threads). A value
// is exactly what the uncached build returns; a failed build is not cached, so
// the next request fails the same way (see sep::FlightCache).
// Pristine (undamaged) connectivity of the asset's base volume. The entry also
// remembers WHICH anatomy object it was built from (weak, so the cache never
// extends an anatomy's lifetime). Derived connectivity (P1.4) uses the pristine
// spans as the base B of a grid only when that grid shares this exact anatomy
// object; any other grid falls back to build() (exact either way).
using AnatomyHandle=decltype(Grid::anatomy);
// scan_work: build()'s work units that depend only on anatomy and lattice
// (pristine work minus its union and label work), for referenceBuildBudget().
struct PristineRef {std::shared_ptr<const gfy_connectivity::Topology>topology;AnatomyHandle built_from;std::uint64_t scan_work{};};
struct PristineEntry {gfy_connectivity::Topology topology;std::weak_ptr<typename AnatomyHandle::element_type>anatomy;std::uint64_t scan_work{};};
inline std::size_t topologyBytes(const gfy_connectivity::Topology&t){
 return t.columns.size()*sizeof(t.columns[0])+t.spans.size()*sizeof(t.spans[0])+t.components.size()*sizeof(t.components[0]);}
inline sep::FlightCache&pristineCache(){static sep::FlightCache cache(12,256u*1024u*1024u);return cache;}
inline sep::FlightCache&mapCache(){static sep::FlightCache cache(12,192u*1024u*1024u);return cache;}
// PristineBuild == build(asset.base) in every field including work; the job owns
// a copy of the base grid (sharing the immutable anatomy).
struct PristineJob final:sep::BandJob {
 gfy_connectivity::PristineBuild<Grid> build;std::weak_ptr<typename AnatomyHandle::element_type>anatomy;
 explicit PristineJob(const Grid&base):build(base),anatomy(base.anatomy){}
 std::size_t bands()const override{return build.bands;}
 void band(std::size_t b)override{build.band(b);}
 std::shared_ptr<const void> finish(std::size_t&bytes)override{
  auto entry=std::make_shared<PristineEntry>();entry->topology=build.finish(&entry->scan_work);entry->anatomy=anatomy;
  bytes=topologyBytes(entry->topology);return entry;}
};
// Null topology only for a warm request whose key is already being built elsewhere.
inline PristineRef pristineReference(const Asset&asset,const SeparationInput&input,sep::Request request=sep::Request::job){
 const Grid&g=asset.base;sep::KeyBytes key;
 key.add(asset.source_hash).add(input.anatomy_hash).add(g.pitch).add(g.origin.x).add(g.origin.y).add(g.origin.z).add(g.size.x).add(g.size.y).add(g.size.z);
 const auto value=pristineCache().get(key.str(),[](const void*context)->std::shared_ptr<sep::BandJob>{
  return std::make_shared<PristineJob>(static_cast<const Asset*>(context)->base);},&asset,request);
 if(!value)return {};
 const auto entry=std::static_pointer_cast<const PristineEntry>(value);
 return {std::shared_ptr<const gfy_connectivity::Topology>(entry,&entry->topology),entry->anatomy.lock(),entry->scan_work};
}
inline std::shared_ptr<const gfy_connectivity::Topology> pristineTopology(const Asset&asset,const SeparationInput&input){
 return pristineReference(asset,input).topology;
}
// True when `pristine` equals build() of `grid` without its removed cells:
// same anatomy object (pristine built from it), no ownership, same lattice.
inline bool derivableFromPristine(const PristineRef&pristine,const Grid&grid){
 const auto&t=*pristine.topology;
 return pristine.built_from&&grid.anatomy.get()==pristine.built_from.get()&&!grid.ownership&&
  t.pitch==grid.pitch&&t.size==std::array<int,3>{grid.size.x,grid.size.y,grid.size.z}&&
  t.origin==std::array<double,3>{grid.origin.x,grid.origin.y,grid.origin.z};
}
inline std::size_t mapBytes(const gfy_piece_collider::element_partition::Map&m){
 return m.runs.size()*sizeof(gfy_piece_collider::element_partition::Run)+m.offsets.size()*sizeof(std::uint32_t);}
// Physics map build job: the exact accelerated reference (native_shape::ExactMapBuild)
// for PHYSRAW1 shapes, or the decoded PHYSCOL1 map. Owns the catalog and topology.
struct MapJob final:sep::BandJob {
 std::unique_ptr<gfy_piece_collider::native_shape::ExactMapBuild>exact;
 std::shared_ptr<const gfy_piece_collider::element_partition::Map>decoded;
 std::size_t bands()const override{return exact?exact->bandCount:0;}
 void band(std::size_t b)override{exact->band(b);}
 std::shared_ptr<const void> finish(std::size_t&bytes)override{
  auto map=decoded?decoded:std::make_shared<const gfy_piece_collider::element_partition::Map>(exact->finish());
  bytes=mapBytes(*map);return map;}
};
struct MapContext {const Asset*asset;const std::shared_ptr<const gfy_connectivity::Topology>*pristine;const SeparationInput*input;const gfy_piece_collider::original_frames::Frames*frames;};
// Original cell ownership is immutable and quality-specific. In particular,
// 0.5 mm pieces must not be compared with counts from a 1 mm physics map.
// Null only for a warm request whose key is already being built elsewhere.
inline std::shared_ptr<const gfy_piece_collider::element_partition::Map> physicsMap(
 const Asset&asset,const std::shared_ptr<const gfy_connectivity::Topology>&pristine,const SeparationInput&input,
 const gfy_piece_collider::original_frames::Frames&frames,sep::Request request=sep::Request::job){
 sep::KeyBytes key;
 key.add(asset.source_hash).add(input.anatomy_hash).add(input.fit_hash).add(input.physics_elements.expected).add(input.bone_frames.expected)
    .add(pristine->pitch).add(pristine->origin).add(pristine->size);
 const MapContext context{&asset,&pristine,&input,&frames};
 const auto value=mapCache().get(key.str(),[](const void*raw)->std::shared_ptr<sep::BandJob>{
  // Same statements, order and failures as the int14t miss path: read (hash
  // checked), decode, then build.
  namespace pc=gfy_piece_collider;const auto&c=*static_cast<const MapContext*>(raw);
  const auto bytes=read(c.input->physics_elements,c.input->hash);auto job=std::make_shared<MapJob>();
  if(bytes.size()>=8&&std::memcmp(bytes.data(),"PHYSRAW1",8)==0)
   job->exact=std::make_unique<pc::native_shape::ExactMapBuild>(std::make_shared<const pc::native_shape::Catalog>(pc::native_shape::decode(bytes,*c.frames)),
                                                               *c.pristine,c.asset->source_hash,c.input->anatomy_hash);
  else job->decoded=std::make_shared<const pc::element_partition::Map>(pc::element_partition::decode(bytes,c.asset->source_hash,c.input->anatomy_hash));
  return job;},&context,request);
 return std::static_pointer_cast<const gfy_piece_collider::element_partition::Map>(value);
}
// Harness form (int14t signature). The topology must outlive the call; a job
// request finishes its build inside the call.
inline std::shared_ptr<const gfy_piece_collider::element_partition::Map> physicsMap(
 const Asset&asset,const gfy_connectivity::Topology&pristine,const SeparationInput&input,
 const gfy_piece_collider::original_frames::Frames&frames){
 return physicsMap(asset,std::shared_ptr<const gfy_connectivity::Topology>(std::shared_ptr<const gfy_connectivity::Topology>{},&pristine),input,frames);
}
// ---- Folded physical ownership (PieceSeed::folded) --------------------------
// admitNativeShell step 1 gives a gram-scale remnant of a divided element no rigid body of its own: its tissue and skin
// stay, posed by an adjacent element (the adopter, recorded in pose_anchors). The engine's shell of the piece then has
// no body for that element, and its parent table (PainterPieceContinuation physical_parents: nearest kinematic
// ancestor WITH a shell element) cannot place it or its former children. A later separation of the piece has to see
// the remnant as the adopter's: otherwise the original element partition brings the element back as a part with no
// parent (its former children with none), root() refuses with separation_multiple_physical_roots, its skin weights
// cannot be anchored (lowpoly_piece_empty), and a reintroduced body would fail the engine's resample_shell.
// Flattened adopter of a bone along the fold chain (2->12 then 12->15 resolves 2->15); 65535 = not folded. A malformed
// record (target out of range, self fold, any cycle, also a chain ending in a cycle) is rejected as a whole: 65535.
inline std::uint16_t foldedAdopter(const std::array<std::uint16_t,64>&folded,std::uint16_t bone){
 if(bone>=64||folded[bone]==65535)return 65535;
 std::uint64_t seen=std::uint64_t(1)<<bone;std::uint16_t a=bone;
 for(unsigned steps=0;steps<64;++steps){const auto next=folded[a];if(next==65535)return a==bone?65535:a;
  if(next>=64||((seen>>next)&1u))return 65535;seen|=std::uint64_t(1)<<next;a=next;}
 return 65535;
}
inline bool anyFolded(const std::array<std::uint16_t,64>&folded){for(auto f:folded)if(f!=65535)return true;return false;}
// Private copy of the shared physics map with every folded element's runs relabelled to its adopter. An adopter's
// original_cells sums the folded originals (remaining <= original keeps holding); the adopters are reported so that
// the caller never certifies one as whole: its native shape does not cover the folded remnant, it must be refitted.
inline std::shared_ptr<const gfy_piece_collider::element_partition::Map> foldedMap(const gfy_piece_collider::element_partition::Map&map,
    const std::array<std::uint16_t,64>&folded,std::set<std::uint16_t>&adopters){
 auto out=std::make_shared<gfy_piece_collider::element_partition::Map>(map);
 for(auto&r:out->runs){const auto a=foldedAdopter(folded,r.element);if(a!=65535){r.element=a;adopters.insert(a);}}
 out->original_cells.clear();for(const auto&r:out->runs)out->original_cells[r.element]+=std::uint64_t(r.hi-r.lo);
 return out;
}
inline gfy_piece_collider::original_frames::Frames foldedFrames(const gfy_piece_collider::original_frames::Frames&frames,const std::array<std::uint16_t,64>&folded){
 auto out=frames;for(auto&b:out.bones)if(b.physical_element>=0){const auto a=foldedAdopter(folded,std::uint16_t(b.physical_element));if(a!=65535)b.physical_element=std::int16_t(a);}
 return out;
}
inline std::vector<std::uint16_t> foldedBindings(const std::vector<std::uint16_t>&elements,const std::array<std::uint16_t,64>&folded){
 auto out=elements;for(auto&e:out){const auto a=foldedAdopter(folded,e);if(a!=65535)e=a;}return out;
}
// ---- Tile-graph connectivity base (separation-track A) ----------------------
// The pristine tile graph of an asset (RemainingConnectivity.h tiles::Base): a
// pure function of the pristine topology, cached like it (4 entries, 128 MB)
// and built by the warm path. Null only for a warm request of a key in flight.
struct TileBaseJob final:sep::BandJob {
 std::shared_ptr<const gfy_connectivity::Topology> pristine;std::shared_ptr<const gfy_connectivity::tiles::Base> base;
 explicit TileBaseJob(std::shared_ptr<const gfy_connectivity::Topology> p):pristine(std::move(p)){}
 std::size_t bands()const override{return 1;}
 void band(std::size_t)override{base=std::make_shared<const gfy_connectivity::tiles::Base>(gfy_connectivity::tiles::buildBase(*pristine));}
 std::shared_ptr<const void> finish(std::size_t&bytes)override{need(bool(base),"tile_base_unbuilt");bytes=base->bytes();return base;}
};
inline sep::FlightCache&tileBaseCache(){static sep::FlightCache cache(12,160u*1024u*1024u);return cache;}
inline bool tileVerdictEnabled(){static const bool on=[]{const char*e=std::getenv("GFY_SEP_TILE_VERDICT");return !e||std::atoi(e)!=0;}();return on;}
// split (2026-11-11): the tile graph of a separated piece's birth topology (RemainingConnectivity.h verdictOwned). A pure
// function of the topology. It is built BESIDE the jobs, on this service's own thread, after the first separation check
// that found the piece still connected (prepareSnapshot asks for it then): a cut that splits the piece runs the span
// path as before and pays nothing, and no graph is built for a piece that is about to be replaced by its parts; the
// checks after that find the graph ready. Kept while the piece's seed lives, for the four pieces asked for last. A
// graph that could not be built is remembered as absent (the piece keeps the span path).
// GFY_SEP_PIECE_TILE_VERDICT=0 switches the piece verdict off (the span path alone, as before).
inline bool pieceTileVerdictEnabled(){static const bool on=[]{const char*e=std::getenv("GFY_SEP_PIECE_TILE_VERDICT");return !e||std::atoi(e)!=0;}();return on;}
struct PieceTileBases {
 using Topology=gfy_connectivity::Topology;using Base=gfy_connectivity::tiles::Base;
 struct Entry {const Topology*key{};std::weak_ptr<const Topology>alive;std::shared_ptr<const Base>base;std::uint64_t used{};};
 static constexpr std::size_t capacity=4;
 std::mutex mutex;std::condition_variable wake;std::vector<Entry>entries;std::deque<std::weak_ptr<const Topology>>queue;
 std::uint64_t clock{};bool stop{};std::atomic<bool>cancel{false};std::thread worker;
 PieceTileBases():worker(&PieceTileBases::run,this){}
 void quit()noexcept{{std::lock_guard<std::mutex>lock(mutex);stop=true;}cancel.store(true);wake.notify_all();if(worker.joinable())worker.join();} // quitfix
 ~PieceTileBases(){quit();}
 void store(const std::shared_ptr<const Topology>&topology,std::shared_ptr<const Base>base){ // mutex held
  for(auto&e:entries)if(e.key==topology.get()){e.used=++clock;if(base)e.base=std::move(base);return;}
  if(entries.size()>=capacity){auto oldest=entries.begin();for(auto it=entries.begin();it!=entries.end();++it)if(it->used<oldest->used)oldest=it;entries.erase(oldest);}
  Entry entry;entry.key=topology.get();entry.alive=topology;entry.base=std::move(base);entry.used=++clock;entries.push_back(std::move(entry));}
 void run(){core::enterPainterThread();const sep::CancelScope scope(&cancel);
  for(;;){std::shared_ptr<const Topology>topology;
   {std::unique_lock<std::mutex>lock(mutex);while(!stop&&queue.empty())wake.wait(lock);if(stop)return;
    topology=queue.front().lock();queue.pop_front();}
   if(!topology)continue; // the piece is gone
   std::shared_ptr<const Base>base;
   try{base=std::make_shared<const Base>(gfy_connectivity::tiles::buildBase(*topology,2));}catch(...){base.reset();}
   std::lock_guard<std::mutex>lock(mutex);store(topology,std::move(base));}}
 // The graph when it is ready, else null.
 std::shared_ptr<const Base> ready(const std::shared_ptr<const Topology>&topology){
  std::lock_guard<std::mutex>lock(mutex);
  for(auto&e:entries)if(e.key==topology.get()){e.used=++clock;return e.base;}
  return {};}
 // Queues the build once (nothing when the graph is there, known absent or already queued).
 void request(const std::shared_ptr<const Topology>&topology){
  std::lock_guard<std::mutex>lock(mutex);if(stop)return;
  entries.erase(std::remove_if(entries.begin(),entries.end(),[](const Entry&e){return e.alive.expired();}),entries.end());
  for(const auto&e:entries)if(e.key==topology.get())return;
  for(const auto&q:queue)if(q.lock().get()==topology.get())return;
  if(queue.size()<16){queue.push_back(topology);wake.notify_one();}}
 // Built on the calling thread (GFY_SEP_PIECE_TILE_VERIFY: every check compares both paths, the first one too).
 std::shared_ptr<const Base> now(const std::shared_ptr<const Topology>&topology){
  {std::lock_guard<std::mutex>lock(mutex);for(auto&e:entries)if(e.key==topology.get()&&e.base){e.used=++clock;return e.base;}}
  auto base=std::make_shared<const Base>(gfy_connectivity::tiles::buildBase(*topology));
  std::lock_guard<std::mutex>lock(mutex);store(topology,base);return base;}
};
// Constructed at the first piece check, so it is destroyed (its thread joined) before every static made earlier.
inline PieceTileBases&pieceTileBases(){static PieceTileBases& service=::gfy_live::core::worker_quit::immortal<PieceTileBases>(0);return service;} // quitfix: never destroyed
// The grids the piece verdict serves: a piece's own grid (ownership, no overlay) on the lattice of its birth topology.
inline bool pieceTileVerdictApplies(const PieceSeed*piece,const Grid&grid){
 return pieceTileVerdictEnabled()&&piece&&piece->topology&&grid.ownership&&!grid.overlay_base&&piece->topology->pitch==grid.pitch&&
  piece->topology->size==std::array<int,3>{grid.size.x,grid.size.y,grid.size.z}&&
  piece->topology->origin==std::array<double,3>{grid.origin.x,grid.origin.y,grid.origin.z};
}
inline std::shared_ptr<const gfy_connectivity::tiles::Base> tileBase(const Asset&asset,const SeparationInput&input,
 const std::shared_ptr<const gfy_connectivity::Topology>&pristine,sep::Request request=sep::Request::job){
 sep::KeyBytes key;key.add(asset.source_hash).add(input.anatomy_hash).add(pristine->pitch).add(pristine->origin).add(pristine->size).add(std::uint32_t(0x54494C45u));
 const auto value=tileBaseCache().get(key.str(),[](const void*context)->std::shared_ptr<sep::BandJob>{
  return std::make_shared<TileBaseJob>(*static_cast<const std::shared_ptr<const gfy_connectivity::Topology>*>(context));},&pristine,request);
 return std::static_pointer_cast<const gfy_connectivity::tiles::Base>(value);
}
// Cold-cost hook (separation-track P1.1c; P1.3 extends it). Builds every
// process-wide cache that a later separation of the same (model, quality) key
// reuses, never anything keyed by damage, and reports each stage's cold cost.
// Exact: the caches only return what prepare() would otherwise compute. A warm
// request (the warm thread) builds only missing keys, on its own thread, and
// yields to real jobs between bands.
// The model's station masks (SeverCheck.h StationMasks): for every station and every tile of its box whose static
// tissue holds the 48 cells a measurement would need before caching, the same per-cell slab test measureStationTile
// runs, inserted ahead of time (up to half the cache, leaving room for what play adds). Masks are pure functions of
// the station and the lattice, so a measurement that finds one computes exactly what it would have computed itself.
inline void warmStationMasks(const Asset&asset){
 if(!asset.station_masks||!asset.tile_material||asset.sever_segments.empty())return;
 auto&masks=*asset.station_masks;const Grid&g=asset.base;const double cell=g.pitch*g.pitch*g.pitch;if(masks.cell!=cell)return;
 const auto&table=*asset.tile_material;const std::size_t limit=StationMasks::maxEntries/2;std::size_t k=0;
 for(const auto&seg:asset.sever_segments)for(const auto&st:seg.stations){const std::size_t index=k++;
  if(sep::warmStop())return;
  for(int tx=st.tileLo.x;tx<=st.tileHi.x;++tx)for(int ty=st.tileLo.y;ty<=st.tileHi.y;++ty)for(int tz=st.tileLo.z;tz<=st.tileHi.z;++tz){
   const Cell tile{tx,ty,tz};const auto it=table.find(tile);if(it==table.end())continue;
   unsigned solid=0;for(unsigned w=0;w<8;++w)solid+=::painter_detail::tile_tunnel_detail::popcount(it->second.solid[w]);if(solid<48)continue;
   if(masks.entries.load(std::memory_order_relaxed)>=limit)return;
   const auto key=StationMasks::key(index,tile);std::array<std::uint64_t,8> probe;if(masks.find(key,probe))continue;
   const auto inside=stationSlabMask(g,seg,st,tile); // speedfix: the same bits, four cells at a time
   masks.insert(key,inside);}}
}
struct WarmReport {std::vector<std::pair<std::string,double>>stages_ms;std::size_t bytes{};bool complete{};};
struct DecodedInputs;
struct DecodedCache {
 std::mutex mutex;std::vector<std::pair<std::string,std::shared_ptr<const DecodedInputs>>> entries;
};
inline DecodedCache& decodedCache(){static DecodedCache cache;return cache;}
inline std::shared_ptr<const DecodedInputs> decodedInputs(const Asset&,const SeparationInput&,
 const std::shared_ptr<const gfy_connectivity::Topology>&);
inline WarmReport warm(const Asset&asset,const SeparationInput&input,sep::Request request=sep::Request::job){
 WarmReport report;auto t=core::Clock::now();
 const auto pristine=pristineReference(asset,input,request).topology;report.stages_ms.emplace_back("pristine_topology",core::ms(t));
 if(!pristine)return report;
 t=core::Clock::now();
 const auto frames=gfy_piece_collider::original_frames::decode(read(input.bone_frames,input.hash),asset.source_hash,input.fit_hash);
 need(frames.bones.size()==asset.source.bones,"separation_original_bone_count");
 report.stages_ms.emplace_back("bone_frames_decode_uncached",core::ms(t));
 t=core::Clock::now();const auto map=physicsMap(asset,pristine,input,frames,request);report.stages_ms.emplace_back("physics_map",core::ms(t));
 if(!map)return report;
 t=core::Clock::now();const auto base=tileVerdictEnabled()?tileBase(asset,input,pristine,request):nullptr;report.stages_ms.emplace_back("tile_base",core::ms(t));
 if(tileVerdictEnabled()&&!base)return report;
 t=core::Clock::now();warmStationMasks(asset);report.stages_ms.emplace_back("station_masks",core::ms(t));
 // Bindings and accessory ownership are also pure model inputs. Reading and
 // decoding them for the first time after death needlessly delays the cut.
 t=core::Clock::now();(void)decodedInputs(asset,input,pristine);report.stages_ms.emplace_back("decoded_inputs",core::ms(t));
 report.bytes=topologyBytes(*pristine)+map->runs.size()*sizeof(map->runs[0])+map->offsets.size()*sizeof(map->offsets[0])+(base?base->bytes():0);
 report.complete=true;return report;
}
// The warm thread (P1.3 iv): one low-priority thread, jobs served in order,
// each building only missing keys and pausing between bands while a real
// separation job runs (unless that job waits for the same key).
inline void lowerCurrentThreadPriority()noexcept{
#if defined(_WIN32)
 (void)SetThreadPriority(GetCurrentThread(),-2); // THREAD_PRIORITY_LOWEST
#endif
}
// speedfix: the separation input each model was warmed with (warm_separation_async: the game's bind call), for the prefit
// after a hit (PrefitService). Up to 32 models; entries of models that are gone are dropped.
struct PrefitInputs {std::mutex mutex;std::vector<std::pair<std::weak_ptr<const Asset>,std::shared_ptr<const SeparationInput>>> entries;};
inline PrefitInputs& prefitInputs(){static PrefitInputs registry;return registry;}
inline bool& sfPrefitThread(){thread_local bool on=false;return on;} // speedfix: this thread is the prefit thread
inline void prefitRecordInput(const std::shared_ptr<const Asset>&asset,const SeparationInput&input)noexcept{
 try{if(!asset)return;auto&r=prefitInputs();
  {std::lock_guard<std::mutex>lock(r.mutex);for(const auto&e:r.entries)if(e.first.lock()==asset)return;} // the first input of a model stays
  auto copy=std::make_shared<const SeparationInput>(input);
  std::lock_guard<std::mutex>lock(r.mutex);
  r.entries.erase(std::remove_if(r.entries.begin(),r.entries.end(),[](const std::pair<std::weak_ptr<const Asset>,std::shared_ptr<const SeparationInput>>&e){return e.first.expired();}),r.entries.end());
  for(const auto&e:r.entries)if(e.first.lock()==asset)return;
  if(r.entries.size()>=32)r.entries.erase(r.entries.begin());
  r.entries.emplace_back(asset,std::move(copy));}catch(...){}}
inline std::shared_ptr<const SeparationInput> prefitInputOf(const std::shared_ptr<const Asset>&asset)noexcept{
 try{auto&r=prefitInputs();std::lock_guard<std::mutex>lock(r.mutex);for(const auto&e:r.entries)if(e.first.lock()==asset)return e.second;}catch(...){}
 return {};}
struct WarmService {
 struct Job {std::shared_ptr<const Asset>asset;SeparationInput input;};
 static constexpr std::size_t capacity=32; // one pending entry per model
 std::mutex mutex;std::condition_variable wake;std::deque<Job>jobs;bool stop{};
 std::uint64_t served{},failed{};std::thread worker;
 WarmService():worker(&WarmService::run,this){sep::warmThreadHandle().store(worker.native_handle());}
 void quit()noexcept{{std::lock_guard<std::mutex>lock(mutex);stop=true;}sep::warmStop()=true;wake.notify_all();sep::warmThreadHandle().store(nullptr);if(worker.joinable())worker.join();} // quitfix
 ~WarmService(){quit();}
 void run(){lowerCurrentThreadPriority();
  for(;;){Job job;{std::unique_lock<std::mutex>lock(mutex);while(!stop&&jobs.empty())wake.wait(lock);if(stop)return;
    job=std::move(jobs.front());jobs.pop_front();}
   lowerCurrentThreadPriority(); // raised to normal while a job waited for the previous task
   bool ok=true;try{(void)warm(*job.asset,job.input,sep::Request::warm);}catch(...){ok=false;}
   std::lock_guard<std::mutex>lock(mutex);++served;failed+=ok?0:1;}
 }
};
// Caches are constructed before the service, so they outlive its thread.
inline WarmService&warmService(){
 // Every cache touched by the warm thread must outlive its joined shutdown.
 // Constructing one for the first time inside run() would destroy it before
 // the service at process exit while that thread may still be using it.
 (void)pristineCache();(void)mapCache();(void)tileBaseCache();(void)decodedCache();
 static WarmService& service=::gfy_live::core::worker_quit::immortal<WarmService>(0);return service; // quitfix: never destroyed
}
}
bool warm_separation_async(std::shared_ptr<const Asset>asset,SeparationInput input)noexcept{
 if(asset)try{separation_detail::installLimbRegions(*asset,input);}catch(...){}
 if(asset)separation_detail::prefitRecordInput(asset,input); // speedfix: the input of the prefit after this model's hits
 try{if(!asset)return false;auto&service=separation_detail::warmService();
  {std::lock_guard<std::mutex>lock(service.mutex);if(service.stop)return false;
   for(const auto&j:service.jobs)if(j.asset.get()==asset.get())return true; // already queued for this model
   if(service.jobs.size()>=service.capacity)return false;
   service.jobs.push_back({std::move(asset),std::move(input)});}
  service.wake.notify_one();return true;
 }catch(...){return false;}
}
namespace separation_detail {
// `v` without the components marked in `drop` (whole components, touching no kept one): build()/derive() of the grid
// with their cells erased. parentSpans (derive()'s per-span parent map) is filtered alongside.
inline gfy_connectivity::Topology withoutComponents(const gfy_connectivity::Topology&v,const std::vector<char>&drop,std::vector<std::uint32_t>*parentSpans){
 gfy_connectivity::Topology out;out.size=v.size;out.origin=v.origin;out.pitch=v.pitch;out.work=v.work;
 out.columns.reserve(v.columns.size());out.spans.reserve(v.spans.size());
 std::vector<std::uint32_t> id(v.components.size(),0),parents;if(parentSpans)parents.reserve(parentSpans->size());
 for(const auto&c:v.columns){gfy_connectivity::Column column{c.x,c.y,std::uint32_t(out.spans.size()),0};
  for(auto s=c.first;s<c.first+c.count;++s){const auto&sp=v.spans[s];if(drop[sp.component])continue;
   auto&n=id[sp.component];if(!n){n=std::uint32_t(out.components.size());out.components.push_back(v.components[sp.component]);}
   out.spans.push_back({sp.lo,sp.hi,n});out.cells+=std::uint64_t(sp.hi-sp.lo);if(parentSpans)parents.push_back((*parentSpans)[s]);}
  column.count=std::uint32_t(out.spans.size()-column.first);if(column.count)out.columns.push_back(column);}
 if(parentSpans)parentSpans->swap(parents);
 return out;
}
// split2 part 7 (2026-11-12): withoutComponents() on up to four threads. Pass 1 counts, per contiguous column range, the
// kept spans, the columns that keep a span, the kept cells and the components in order of their first kept span. The
// former code numbers a component when its first kept span comes by in column order: merging the ranges' lists in range
// order gives the same numbers. Pass 2 writes every range at its place (span and column offsets = prefix sums).
// GFY_SEP_DEBRIS_FILTER_VERIFY=1: the former code too, every field and the parent-span map compared.
inline gfy_connectivity::Topology withoutComponentsSideBySide(const gfy_connectivity::Topology&v,const std::vector<char>&drop,std::vector<std::uint32_t>*parentSpans){
 static const bool verify=std::getenv("GFY_SEP_DEBRIS_FILTER_VERIFY")!=nullptr;
 const std::size_t NC=v.columns.size();constexpr std::size_t grain=std::size_t(1)<<14;
 const std::size_t chunks=std::max<std::size_t>(1,sep::chunkCount(NC,grain,sep::max_job_threads));
 if(chunks<2&&!verify)return withoutComponents(v,drop,parentSpans); // nothing to run side by side
 std::vector<std::uint32_t> formerParents;gfy_connectivity::Topology former;
 if(verify){if(parentSpans)formerParents=*parentSpans;former=withoutComponents(v,drop,parentSpans?&formerParents:nullptr);}
 struct Part {std::size_t columns{},spans{};std::uint64_t cells{};std::vector<std::uint32_t> first;};
 std::vector<Part> parts(chunks);
 sep::parallel(NC,grain,sep::max_job_threads,[&](std::size_t k0,std::size_t k1,std::size_t chunk){auto&part=parts[chunk];std::vector<char> seen(v.components.size(),0);
  for(std::size_t k=k0;k<k1;++k){const auto&c=v.columns[k];std::size_t n=0;
   for(auto s=c.first;s<c.first+c.count;++s){const auto&sp=v.spans[s];if(drop[sp.component])continue;++n;part.cells+=std::uint64_t(sp.hi-sp.lo);
    if(!seen[sp.component]){seen[sp.component]=1;part.first.push_back(sp.component);}}
   part.spans+=n;part.columns+=n!=0;}});
 gfy_connectivity::Topology out;out.size=v.size;out.origin=v.origin;out.pitch=v.pitch;out.work=v.work;
 std::vector<std::uint32_t> id(v.components.size(),0);std::vector<std::size_t> spanAt(chunks+1,0),columnAt(chunks+1,0);
 for(std::size_t c=0;c<chunks;++c){const auto&part=parts[c];spanAt[c+1]=spanAt[c]+part.spans;columnAt[c+1]=columnAt[c]+part.columns;out.cells+=part.cells;
  for(const auto component:part.first){auto&n=id[component];if(!n){n=std::uint32_t(out.components.size());out.components.push_back(v.components[component]);}}}
 out.spans.resize(spanAt[chunks]);out.columns.resize(columnAt[chunks]);std::vector<std::uint32_t> parents;if(parentSpans)parents.resize(spanAt[chunks]);
 sep::parallel(NC,grain,sep::max_job_threads,[&](std::size_t k0,std::size_t k1,std::size_t chunk){std::size_t at=spanAt[chunk],columnCursor=columnAt[chunk];
  for(std::size_t k=k0;k<k1;++k){const auto&c=v.columns[k];gfy_connectivity::Column column{c.x,c.y,std::uint32_t(at),0};
   for(auto s=c.first;s<c.first+c.count;++s){const auto&sp=v.spans[s];if(drop[sp.component])continue;
    out.spans[at]={sp.lo,sp.hi,id[sp.component]};if(parentSpans)parents[at]=(*parentSpans)[s];++at;}
   column.count=std::uint32_t(at-column.first);if(column.count)out.columns[columnCursor++]=column;}});
 if(parentSpans)parentSpans->swap(parents);
 if(verify){
  bool same=former.size==out.size&&former.origin==out.origin&&former.pitch==out.pitch&&former.work==out.work&&former.cells==out.cells&&former.columns.size()==out.columns.size()&&
   former.spans.size()==out.spans.size()&&former.components.size()==out.components.size()&&(!parentSpans||formerParents==*parentSpans);
  for(std::size_t i=0;same&&i<out.columns.size();++i)same=former.columns[i].x==out.columns[i].x&&former.columns[i].y==out.columns[i].y&&former.columns[i].first==out.columns[i].first&&former.columns[i].count==out.columns[i].count;
  for(std::size_t i=0;same&&i<out.spans.size();++i)same=former.spans[i].lo==out.spans[i].lo&&former.spans[i].hi==out.spans[i].hi&&former.spans[i].component==out.spans[i].component;
  for(std::size_t i=0;same&&i<out.components.size();++i)same=former.components[i].cells==out.components[i].cells&&former.components[i].lo==out.components[i].lo&&former.components[i].hi==out.components[i].hi;
  std::cerr<<(same?"debris_filter_verify ok":"debris_filter_verify MISMATCH")<<" chunks="<<chunks<<" spans="<<out.spans.size()<<" columns="<<out.columns.size()<<" components="<<out.components.size()<<std::endl;
  need(same,"separation_debris_filter_mismatch");}
 return out;
}
// Field-by-field connectivity equality (Topology::work excluded: it is a budget counter).
inline bool sameTopology(const gfy_connectivity::Topology&a,const gfy_connectivity::Topology&b){
 if(a.size!=b.size||a.origin!=b.origin||a.pitch!=b.pitch||a.cells!=b.cells||a.columns.size()!=b.columns.size()||
    a.spans.size()!=b.spans.size()||a.components.size()!=b.components.size())return false;
 for(std::size_t i=0;i<a.columns.size();++i){const auto&x=a.columns[i];const auto&y=b.columns[i];
  if(x.x!=y.x||x.y!=y.y||x.first!=y.first||x.count!=y.count)return false;}
 for(std::size_t i=0;i<a.spans.size();++i){const auto&x=a.spans[i];const auto&y=b.spans[i];
  if(x.lo!=y.lo||x.hi!=y.hi||x.component!=y.component)return false;}
 for(std::size_t i=0;i<a.components.size();++i){const auto&x=a.components[i];const auto&y=b.components[i];
  if(x.cells!=y.cells||x.lo!=y.lo||x.hi!=y.hi)return false;}
 return true;
}
// Frozen interface hook of separation_detail::prepare (separation-track P1.0d).
// Every field defaults to "absent"; with the defaults prepare() is the exact
// int14t separation.
// gfyspeed3: thrown by prepare() in the thread-tear retry (ComponentSession_impl.h) when no second large piece can come out
struct ThreadTearHeld {};
struct PrepareExtras {
 GoreControls gore;
 double thread_tear_retry_m3{}; // gfyspeed3: > 0 in the thread-tear retry: the volume each of its two large pieces needs
 std::shared_ptr<const std::map<std::uint32_t,std::uint32_t>> semantic_groups;
 // Snapshot's published wound mesh, forwarded to SurfaceOptions::published.
 // Non-null only when SeparationInput::published_surface selects it (P2 S4).
 const gfy_packed::Mesh* published{};
 // The snapshot's published low-poly wound mesh: the only source of separated piece surfaces (LowPolyPieces.h).
 const gfy_packed::Mesh* lowpoly{};
 // Connectivity of `grid` already known to the caller (P1.7). It must equal
 // gfy_connectivity::build(grid) in every field except Topology::work
 // (checked in GFY_SEP_VERIFY builds). Null: prepare() builds it.
 std::shared_ptr<const gfy_connectivity::Topology> precomputed_volume;
 // Receives the remaining-volume topology prepare() used (P1.2/P1.4b), set
 // before the separation verdict, so also on no_separation.
 std::shared_ptr<const gfy_connectivity::Topology>* volume_out{};
 // Parent topology plus the removal z-runs added since it (P1.4 derived
 // connectivity; runs in (y,x,lo) order). Not consumed yet: prepare() still
 // builds from `grid`, which derive() must equal.
 struct Derivation {
  std::shared_ptr<const gfy_connectivity::Topology> parent;
  std::shared_ptr<const Grid> parent_grid;
  std::vector<gfy_connectivity::detail::Removed> added;
 };
 const Derivation* derived_from{};
 // Separating a piece session's snapshot (PieceSeed.h): the split is compared with the piece at birth, and the new
 // pieces inherit its foreign marks.
 const PieceSeed* piece{};
 // The grid (debris erased; null = the input grid) and remaining topology the pieces were built from: seeds are cut
 // from them.
 std::shared_ptr<const Grid>* grid_out{};std::shared_ptr<const gfy_connectivity::Topology>* pieces_volume_out{};
 // split2 part 6: called by prepare() on its own thread as soon as the pieces' volume (the object pieces_volume_out will
 // receive) and the piece components are final - long before prepare() returns. Not called for grouped island pieces.
 std::function<void(const std::shared_ptr<const gfy_connectivity::Topology>&,const std::vector<std::uint32_t>&)> piece_volume_ready;
};
// Snapshot state a separation hands to its pieces' seeds.
struct PieceContext {const PieceSeed* piece{};std::shared_ptr<const ResolvedCutHistory> history;std::shared_ptr<const SeverState> sever;GoreControls gore;};
inline bound_surface::ExtraSurface readAccessorySurfaces(const Asset&asset,const std::vector<std::uint8_t>&raw){
 Cursor c{raw};c.magic("VXGEOM01");const auto source=c.take<Hash>();need(source==asset.source_hash,"accessory_surface_identity");
 auto lengths=c.take<std::array<std::uint32_t,4>>();std::array<std::vector<std::uint8_t>,4>blob;
 for(unsigned i=0;i<4;++i){need(lengths[i]<=128u*1024u*1024u&&c.at+lengths[i]<=raw.size(),"accessory_surface_blob_size");
  blob[i].assign(raw.begin()+c.at,raw.begin()+c.at+lengths[i]);c.at+=lengths[i];}c.done();
 bound_surface::ExtraSurface out;out.source=sourceBytes(blob[0],true);need(out.source.bones==asset.source.bones,"accessory_surface_rig");
 Cursor shade{blob[1]};shade.magic("VXSHAD01");need(shade.take<std::uint32_t>()==out.source.v.size(),"accessory_surface_shading");
 for(auto&v:out.source.v){v.normal={shade.take<float>(),shade.take<float>(),shade.take<float>()};v.tangent={shade.take<float>(),shade.take<float>(),shade.take<float>()};v.binormal={shade.take<float>(),shade.take<float>(),shade.take<float>()};}shade.done();
 Cursor children{blob[2]};children.magic("VXCHLD01");need(children.take<std::uint32_t>()==out.source.f.size(),"accessory_surface_children");
 for(std::size_t f=0;f<out.source.f.size();++f){const auto child=children.take<std::uint16_t>();const auto face=children.take<std::uint32_t>();
  need(child<asset.original_children,"accessory_surface_child_range");out.face_children.push_back(child);out.face_indices.push_back(face);}children.done();
 out.bindings=bound_surface::read(blob[3],asset.source_hash,out.source,asset.base,2.0);return out;
}
inline std::uint16_t cutSkinAnchor(std::uint16_t original,const std::set<std::uint16_t>&elements,
                                  const std::vector<std::uint16_t>&parents){
 if(elements.count(original))return original;
 // Skin blends across a cut can refer to the immediately adjacent original
 // physical element. Keep that captured relative pose on the ONE retained
 // neighbor. Never walk to a distant surviving body part. Coat panels with
 // missing leg influences have already been removed by the waist-root pass.
 std::set<std::uint16_t> adjacent;
 if(original<parents.size()&&elements.count(parents[original]))adjacent.insert(parents[original]);
 for(auto bone:elements)if(bone<parents.size()&&parents[bone]==original)adjacent.insert(bone);
 if(adjacent.size()!=1){std::string detail="separation_skin_without_unique_original_neighbor original="+std::to_string(original)+" retained=";
  for(auto b:elements)detail+=std::to_string(b)+",";
  detail+=" adjacent=";for(auto b:adjacent)detail+=std::to_string(b)+",";
  throw std::runtime_error(detail);
 }
 return *adjacent.begin();
}
// Missing influences on a certified clipped skin region may freeze to the
// single retained boundary of their original physical subtree. No spatial
// nearest-bone choice, graph cycles or multiple retained frontiers are allowed.
inline std::uint16_t regionSkinFrontier(std::uint16_t original,const std::set<std::uint16_t>&retained,const std::vector<std::uint16_t>&parents){
 if(original>=parents.size()||retained.count(original))return 65535;
 std::vector<char>seen(parents.size(),0);std::set<std::uint16_t>frontier;bool malformed=false;
 std::function<void(std::uint16_t,std::uint16_t)>walk=[&](std::uint16_t at,std::uint16_t from){
  if(retained.count(at)){frontier.insert(at);return;}if(at>=parents.size()||seen[at]){malformed=true;return;}seen[at]=1;
  const auto parent=parents[at];if(parent!=65535&&parent!=from)walk(parent,at);
  for(std::size_t q=0;q<parents.size();++q)if(q!=from&&parents[q]==at)walk(std::uint16_t(q),at);};
 walk(original,65535);return !malformed&&frontier.size()==1?*frontier.begin():65535;
}
inline bool constrainedSkinUses(const gfy_packed::Mesh&mesh,std::uint16_t bone){
 bool used=false;for(const auto&child:mesh.children){const auto*table=mesh.skin_ownership?mesh.skin_ownership->child(child.child_index):nullptr;
  for(std::size_t t=0;t<child.indices.size()/3;++t){bool weighted=false;
   for(unsigned q=0;q<3;++q){const auto&v=child.vertices[child.indices[3*t+q]];for(unsigned w=0;w<4;++w)weighted|=v.bone[w]==bone&&gfy_signed_hw::dependency(v.weight,w,0.);}
   if(weighted){used=true;if(child.material||!child.skin_owned||!table||!table->data[t][1])return false;}}}
 return used;
}
// Only an immutable, material-backed source binding can certify a foreign
// authored skin influence on a surviving part. A render vertex alone, or a
// nearby surviving bone, is not attachment evidence.
inline std::uint16_t originalBindingElement(const gfy_piece_collider::element_partition::Map&map,Vec point){
 std::array<int,3>cell{};
 for(unsigned a=0;a<3;++a){const double coordinate=(point[a]-map.origin[a])/map.pitch;
  if(!std::isfinite(coordinate)||coordinate<0||coordinate>=map.size[a]||
     std::abs(coordinate-std::nearbyint(coordinate))<=1e-8)return 65535;
  cell[a]=int(std::floor(coordinate));}
 const auto column=std::size_t(cell[1])*map.size[0]+cell[0];
 for(auto i=map.offsets[column];i<map.offsets[column+1];++i)
  if(map.runs[i].lo<=cell[2]&&cell[2]<map.runs[i].hi)return map.runs[i].element;
 return 65535;
}
inline std::uint16_t sourceSkinAnchor(const Asset&asset,const bound_surface::Bindings&bindings,
 const gfy_piece_collider::original_frames::Frames&frames,const std::vector<std::uint16_t>&bindingElements,
 const sep::SourceFaceIndex&sourceFaces,
 const gfy_packed::Mesh&mesh,const std::set<std::uint16_t>&retained,std::uint16_t weightedBone){
 std::uint16_t owner=65535;bool seen=false;
 for(const auto&child:mesh.children){
  if(child.indices.size()%3||child.original_faces.size()!=child.indices.size()/3)return 65535;
  for(std::size_t t=0;t<child.indices.size()/3;++t){bool used=false;
   for(unsigned corner=0;corner<3;++corner){const auto index=child.indices[3*t+corner];
    if(index>=child.vertices.size())return 65535;
    const auto&v=child.vertices[index];
    for(unsigned w=0;w<4;++w)used=used||(v.bone[w]==weightedBone&&gfy_signed_hw::dependency(v.weight,w,1e-6));}
   if(!used)continue;
   // Cavity, loose extra surfaces and coat tails require their own admission.
   if(child.material||child.original_faces[t]==~std::uint32_t(0))return 65535;
   const auto found=sourceFaces.find(child.child_index,child.original_faces[t]);
   if(!found)return 65535;
   const auto face=*found;
   if(face>=asset.source.f.size()||(!bindings.coat_faces.empty()&&bindings.coat_faces[face]))return 65535;
   std::uint16_t faceOwner=65535;bool authoredWeight=false;
   for(auto vertex:asset.source.f[face]){
    if(vertex>=bindingElements.size())return 65535;
    const auto element=bindingElements[vertex];
    if(element==65535||!retained.count(element)||(faceOwner!=65535&&faceOwner!=element))return 65535;
    faceOwner=element;const auto&source=asset.source.v[vertex];double localSkin=0;
    for(unsigned w=0;w<4;++w){const auto bone=source.ids[w];
     if(bone>=frames.bones.size())return 65535;
     if(frames.bones[bone].physical_element==element)localSkin+=source.w[w];
     authoredWeight=authoredWeight||(bone==weightedBone&&gfy_signed_hw::dependency(source.w,w,1e-6));}
    // The source face itself must be predominantly skinned to the exact
    // physical element identified by its original anatomical binding.
    if(localSkin<=.5)return 65535;
   }
   if(!authoredWeight||(owner!=65535&&owner!=faceOwner))return 65535;
   owner=faceOwner;seen=true;
  }
 }
 return seen?owner:65535;
}
#include "separation/LowPolyPieces.h"
#include "separation/MaskPieces.h" // mask-only wounds, slice 2
#include "separation/MaskPieces3.h" // mask-only wounds, slice 3
#include "separation/PristineIslandGrouping.h"
#include "separation/ThreadTear.h"
#include "separation/BurstChunk.h" // chunknosplash
// The part's cells at f x the pitch: a coarse cell holds tissue when any of its f^3 cells does.
inline gfy_connectivity::Topology coarseTopologyFormer(const gfy_connectivity::Topology&top,std::uint32_t component,int f){
 gfy_connectivity::Topology c;c.pitch=top.pitch*f;c.origin=top.origin;for(int a=0;a<3;++a)c.size[a]=(top.size[a]+f-1)/f;
 std::map<std::pair<int,int>,std::vector<std::pair<int,int>>> columns; // (y,x): the order Topology keeps
 for(const auto&col:top.columns)for(auto i=col.first;i<col.first+col.count;++i){const auto&s=top.spans[i];if(s.component!=component)continue;
  columns[{col.y/f,col.x/f}].push_back({s.lo/f,(s.hi-1)/f+1});}
 c.components.resize(2);auto&comp=c.components[1];
 for(auto&entry:columns){auto&runs=entry.second;std::sort(runs.begin(),runs.end());
  gfy_connectivity::Column out;out.x=entry.first.second;out.y=entry.first.first;out.first=std::uint32_t(c.spans.size());out.count=0;
  int lo=runs[0].first,hi=runs[0].second;
  auto flush=[&]{gfy_connectivity::Span s;s.lo=lo;s.hi=hi;s.component=1;c.spans.push_back(s);++out.count;comp.cells+=std::uint64_t(hi-lo);
   comp.lo[0]=std::min(comp.lo[0],out.x);comp.hi[0]=std::max(comp.hi[0],out.x+1);comp.lo[1]=std::min(comp.lo[1],out.y);comp.hi[1]=std::max(comp.hi[1],out.y+1);
   comp.lo[2]=std::min(comp.lo[2],lo);comp.hi[2]=std::max(comp.hi[2],hi);};
  for(std::size_t k=1;k<runs.size();++k){if(runs[k].first<=hi)hi=std::max(hi,runs[k].second);else{flush();lo=runs[k].first;hi=runs[k].second;}}
  flush();c.columns.push_back(out);}
 c.cells=comp.cells;return c;
}
// split2 (2026-11-11): the same coarse part without a tree. The former code keeps one std::map entry and one vector per
// coarse column and finds the entry once per 1 mm span (4-16 ms per fit; 70 ms of the fit work of a shot-up body). A
// topology's columns are in lattice order ((y,x) ascending), so the coarse rows (y/f) arrive one after another: one coarse
// row's runs are collected in one vector, sorted by (coarse x, lo, hi) and merged. Per coarse column that is the former
// sorted run list (same runs, same order) and the former merge; the columns come out in (y,x) order as the map gave them.
// Columns that are not in that order: the former code. GFY_COARSE_TOPOLOGY=0: the former code alone.
// GFY_COARSE_TOPOLOGY_VERIFY=1: both, every field compared (coarse_topology_mismatch).
inline gfy_connectivity::Topology coarseTopology(const gfy_connectivity::Topology&top,std::uint32_t component,int f){
 static const bool formerOnly=[]{const char*e=std::getenv("GFY_COARSE_TOPOLOGY");return e&&std::atoi(e)==0;}();
 static const bool verify=std::getenv("GFY_COARSE_TOPOLOGY_VERIFY")!=nullptr;
 if(formerOnly)return coarseTopologyFormer(top,component,f);
 gfy_connectivity::Topology c;c.pitch=top.pitch*f;c.origin=top.origin;for(int a=0;a<3;++a)c.size[a]=(top.size[a]+f-1)/f;
 c.components.resize(2);auto&comp=c.components[1];
 struct Run{int x,lo,hi;};std::vector<Run> row;
 const auto flushRow=[&](int y){if(row.empty())return;
  std::sort(row.begin(),row.end(),[](const Run&a,const Run&b){return a.x!=b.x?a.x<b.x:(a.lo!=b.lo?a.lo<b.lo:a.hi<b.hi);});
  for(std::size_t i=0;i<row.size();){gfy_connectivity::Column out;out.x=row[i].x;out.y=y;out.first=std::uint32_t(c.spans.size());out.count=0;
   int lo=row[i].lo,hi=row[i].hi;
   auto flush=[&]{gfy_connectivity::Span s;s.lo=lo;s.hi=hi;s.component=1;c.spans.push_back(s);++out.count;comp.cells+=std::uint64_t(hi-lo);
    comp.lo[0]=std::min(comp.lo[0],out.x);comp.hi[0]=std::max(comp.hi[0],out.x+1);comp.lo[1]=std::min(comp.lo[1],out.y);comp.hi[1]=std::max(comp.hi[1],out.y+1);
    comp.lo[2]=std::min(comp.lo[2],lo);comp.hi[2]=std::max(comp.hi[2],hi);};
   std::size_t k=i+1;
   for(;k<row.size()&&row[k].x==out.x;++k){if(row[k].lo<=hi)hi=std::max(hi,row[k].hi);else{flush();lo=row[k].lo;hi=row[k].hi;}}
   flush();c.columns.push_back(out);i=k;}
  row.clear();};
 bool started=false;int rowY=0;
 for(const auto&col:top.columns){const int y=col.y/f;
  if(started&&y<rowY)return coarseTopologyFormer(top,component,f); // not in lattice order: the former code takes any order
  if(started&&y!=rowY)flushRow(rowY);
  started=true;rowY=y;const int x=col.x/f;
  for(auto i=col.first;i<col.first+col.count;++i){const auto&s=top.spans[i];if(s.component!=component)continue;row.push_back({x,s.lo/f,(s.hi-1)/f+1});}}
 if(started)flushRow(rowY);
 c.cells=comp.cells;
 if(verify){const auto r=coarseTopologyFormer(top,component,f);
  bool same=r.size==c.size&&r.origin==c.origin&&r.pitch==c.pitch&&r.cells==c.cells&&r.work==c.work&&r.columns.size()==c.columns.size()&&r.spans.size()==c.spans.size()&&r.components.size()==c.components.size();
  for(std::size_t i=0;same&&i<c.columns.size();++i)same=r.columns[i].x==c.columns[i].x&&r.columns[i].y==c.columns[i].y&&r.columns[i].first==c.columns[i].first&&r.columns[i].count==c.columns[i].count;
  for(std::size_t i=0;same&&i<c.spans.size();++i)same=r.spans[i].lo==c.spans[i].lo&&r.spans[i].hi==c.spans[i].hi&&r.spans[i].component==c.spans[i].component;
  for(std::size_t i=0;same&&i<c.components.size();++i)same=r.components[i].cells==c.components[i].cells&&r.components[i].lo==c.components[i].lo&&r.components[i].hi==c.components[i].hi;
  std::cerr<<(same?"coarse_topology_verify ok":"coarse_topology_verify MISMATCH")<<" f="<<f<<" columns="<<c.columns.size()<<" spans="<<c.spans.size()<<" cells="<<c.cells<<std::endl;
  need(same,"coarse_topology_mismatch");}
 return c;
}
// Collider fit of one part (speed, see sec36): large parts are fitted at 2-4 mm, mass properties exact at 1 mm.
// split2 part 13 (2026-11-12): the coarse parts of one prepare(). coarseTopology() is a pure function of the part volume,
// the component and the coarsening; a fit that asks for a part an earlier fit of this prepare() already coarsened (the
// shell-budget refits: the same element again with a smaller box limit) takes that one. Keyed by the part volume's
// address: the part volumes of one prepare() stay where they are until it ends.
struct CoarsePartCache {std::mutex mutex;
 std::map<std::pair<const gfy_connectivity::Topology*,std::pair<std::uint32_t,int>>,std::shared_ptr<const gfy_connectivity::Topology>> parts;};
inline gfy_piece_collider::Prepared fitColliderCore(const gfy_connectivity::Topology&top,const gfy_piece_collider::Capture&capture,
                                                gfy_piece_collider::Limits limits={},CoarsePartCache*coarseCache=nullptr){
 namespace pc=gfy_piece_collider;
 static const int forced=[]{const char*e=std::getenv("GFY_COLLIDER_COARSE");return e?std::atoi(e):0;}();
 need(capture.component>0&&capture.component<top.components.size(),"collider_component");
 const auto cells=top.components[capture.component].cells;
 const int f=forced>0?forced:(cells>4000000?4:(cells>400000?3:(cells>60000?2:1)));
 if(f<=1||top.pitch*f>.01)return pc::prepare(top,capture,limits);
 std::shared_ptr<const gfy_connectivity::Topology> keptCoarse; // split2 part 13
 if(coarseCache){std::lock_guard<std::mutex> lock(coarseCache->mutex);const auto found=coarseCache->parts.find({&top,{capture.component,f}});if(found!=coarseCache->parts.end())keptCoarse=found->second;}
 if(!keptCoarse){keptCoarse=std::make_shared<const gfy_connectivity::Topology>(coarseTopology(top,capture.component,f));
  if(coarseCache){std::lock_guard<std::mutex> lock(coarseCache->mutex);coarseCache->parts.emplace(std::make_pair(&top,std::make_pair(capture.component,f)),keptCoarse);}}
 const auto&coarse=*keptCoarse;
 pc::Capture c=capture;c.component=1;
 auto out=pc::prepare(coarse,c,limits);
 // Exact mass properties from the 1 mm cells (the same sums pc::prepare uses).
 std::vector<pc::detail::Run> all;
 for(const auto&col:top.columns)for(auto i=col.first;i<col.first+col.count;++i){const auto&s=top.spans[i];if(s.component==capture.component)all.push_back({col.x,col.y,s.lo,s.hi});}
 const auto moment=pc::detail::moments(all,top);need(moment.n>0,"collider_moment_cells");
 const long double cube=top.pitch*top.pitch*top.pitch,cellMass=capture.density_kg_m3*cube;
 pc::V restCOM{};for(int a=0;a<3;++a)restCOM[a]=double(moment.s[a]/moment.n);
 const long double vx=moment.q[0]-moment.s[0]*moment.s[0]/moment.n+moment.n*top.pitch*top.pitch/12;
 const long double vy=moment.q[1]-moment.s[1]*moment.s[1]/moment.n+moment.n*top.pitch*top.pitch/12;
 const long double vz=moment.q[2]-moment.s[2]*moment.s[2]/moment.n+moment.n*top.pitch*top.pitch/12;
 const long double cxy=moment.xy-moment.s[0]*moment.s[1]/moment.n,cxz=moment.xz-moment.s[0]*moment.s[2]/moment.n,cyz=moment.yz-moment.s[1]*moment.s[2]/moment.n;
 const long double ir[3][3]={{cellMass*(vy+vz),-cellMass*cxy,-cellMass*cxz},{-cellMass*cxy,cellMass*(vx+vz),-cellMass*cyz},{-cellMass*cxz,-cellMass*cyz,cellMass*(vx+vy)}};
 out.owned_cells=std::uint64_t(moment.n);out.anatomical_volume_m3=double(cube*moment.n);out.mass_kg=double(cellMass*moment.n);
 out.com_anchor=pc::transform(capture.rest_to_anchor,restCOM);
 const auto&rot=capture.rest_to_anchor.row_major;
 for(int i=0;i<3;++i)for(int j=0;j<3;++j){long double value=0;for(int k=0;k<3;++k)for(int l=0;l<3;++l)value+=rot[i*4+k]*ir[k][l]*rot[j*4+l];out.inertia_anchor_com[i*3+j]=double(value);}
 const auto&I=out.inertia_anchor_com;const double det=I[0]*(I[4]*I[8]-I[5]*I[7])-I[1]*(I[3]*I[8]-I[5]*I[6])+I[2]*(I[3]*I[7]-I[4]*I[6]);
 need(out.mass_kg>0&&std::isfinite(out.mass_kg)&&I[0]>0&&I[0]*I[4]-I[1]*I[3]>0&&det>0,"collider_inertia_not_positive_definite");
 const pc::V worldCOM=pc::transform(capture.anchor_to_world,out.com_anchor);
 out.angular_world=capture.parent_angular_world;
 out.linear_world=pc::add(capture.parent_linear_world,pc::cross(capture.parent_angular_world,pc::sub(worldCOM,capture.parent_com_world)));
 out.maximum_radius_m=0;
 for(const auto&box:out.boxes)for(int dx:{-1,1})for(int dy:{-1,1})for(int dz:{-1,1}){pc::V p=box.center_anchor;
  for(int a=0;a<3;++a){const pc::V axis{box.orientation[a],box.orientation[3+a],box.orientation[6+a]};p=pc::add(p,pc::mul(axis,(a==0?dx:(a==1?dy:dz))*box.half_extents_m[a]));}
  out.maximum_radius_m=std::max(out.maximum_radius_m,pc::norm(pc::sub(p,out.com_anchor)));}
 out.proxy_to_anatomy_volume_ratio*=double(coarse.cells)*double(coarse.pitch*coarse.pitch*coarse.pitch)/std::max(1e-12,out.anatomical_volume_m3);
 return out;
}
// speedfix: GFY_SF_FITLOG=1 prints one line per collider fit (element, cells, boxes, the greedy try a compact fit restarted
// from, time, thread).
inline gfy_piece_collider::Prepared fitColliderLogged(const gfy_connectivity::Topology&top,const gfy_piece_collider::Capture&capture,
                                                gfy_piece_collider::Limits limits={},CoarsePartCache*coarseCache=nullptr){
 static const bool fitLog=std::getenv("GFY_SF_FITLOG")!=nullptr;
 if(!fitLog)return fitColliderCore(top,capture,limits,coarseCache);
 auto&tr=gfy_piece_collider::detail::fitTrace();tr=gfy_piece_collider::detail::FitTrace{};const auto t=core::Clock::now();
 const auto cells=capture.component<top.components.size()?top.components[capture.component].cells:0;
 gfy_piece_collider::Prepared out;
 const auto print=[&](const char*failed){std::uint64_t h=1469598103934665603ull; // FNV-1a over the fit's boxes and mass properties
  const auto mix=[&](const void*p,std::size_t n){for(std::size_t i=0;i<n;++i){h^=static_cast<const unsigned char*>(p)[i];h*=1099511628211ull;}};
  for(const auto&b:out.boxes){mix(b.center_anchor.data(),sizeof b.center_anchor);mix(b.orientation.data(),sizeof b.orientation);mix(b.half_extents_m.data(),sizeof b.half_extents_m);}
  mix(&out.mass_kg,sizeof out.mass_kg);mix(out.com_anchor.data(),sizeof out.com_anchor);mix(out.inertia_anchor_com.data(),sizeof out.inertia_anchor_com);
  mix(&out.maximum_corner_gap_m,sizeof out.maximum_corner_gap_m);mix(&out.proxy_to_anatomy_volume_ratio,sizeof out.proxy_to_anatomy_volume_ratio);
  std::ostringstream line;line<<"sf_fit bone="<<capture.anchor_bone<<" cells="<<cells<<" spans="<<top.spans.size()<<" columns="<<top.columns.size()<<" boxes="<<out.boxes.size()
  <<" max_boxes="<<limits.max_boxes<<" fithash="<<std::hex<<h<<std::dec<<" restarts="<<tr.restarts<<" speculated="<<tr.speculated<<" greedy_ms="<<tr.greedyMs<<" greedy_leaves="<<tr.greedyLeaves
  <<" mass="<<out.mass_kg<<" ms="<<core::ms(t)<<" thread="<<std::this_thread::get_id()<<(sfPrefitThread()?" prefit=1":"")<<(failed?" error=":"")<<(failed?failed:"")<<'\n';std::cerr<<line.str()<<std::flush;};
 try{out=fitColliderCore(top,capture,limits,coarseCache);}catch(const std::exception&e){print(e.what());throw;}
 print(nullptr);return out;}
// ---- speedfix: the fit cache -----------------------------------------------------------------------------------------------
// fitColliderCore() reads the part volume (lattice, columns, spans, components: not its work counter), the capture and the
// limits, and nothing else: the result's stamp is the capture's, and the coarse-part cache only keeps coarseTopology() of the
// same part (a pure function of it). The cache keeps the last 256 fits by those inputs (the capture without its stamp); a fit
// of the same inputs gets a copy with its own stamp: the boxes, masses, COM, inertia and velocities a fresh fit computes.
// Key: two independent 64-bit hashes over every byte of those inputs, plus the cell / column / span counts and the bone (all
// compared). Failed fits are not kept (they fail again where they failed); a capture without a valid stamp is not answered
// (prepare() refuses it first). The Limits / Capture layouts are asserted: a field
// added there must be added to the key. GFY_SF_FIT_CACHE=0: off. GFY_SF_FIT_CACHE_VERIFY=1: every hit is fitted fresh too and
// compared field by field (sf_fitcache_verify lines).
static_assert(sizeof(gfy_piece_collider::Limits)==2*sizeof(std::size_t)+7*sizeof(double),"speedfix fit key: Limits changed");
static_assert(sizeof(gfy_piece_collider::Capture)==sizeof(Stamp)+360,"speedfix fit key: Capture changed");
static_assert(sizeof(gfy_connectivity::Column)==16&&sizeof(gfy_connectivity::Span)==12&&sizeof(gfy_connectivity::Component)==32,"speedfix fit key: topology layout");
struct FitKey {std::uint64_t a{},b{},cells{};std::uint64_t columns{},spans{};std::uint16_t bone{};
 bool operator==(const FitKey&o)const{return a==o.a&&b==o.b&&cells==o.cells&&columns==o.columns&&spans==o.spans&&bone==o.bone;}};
struct FitKeyHash {std::uint64_t a=0x9E3779B97F4A7C15ull,b=0xD6E8FEB86659FD93ull;
 static std::uint64_t rotl(std::uint64_t x,int r){return (x<<r)|(x>>(64-r));}
 static std::uint64_t fmix(std::uint64_t k){k^=k>>33;k*=0xFF51AFD7ED558CCDull;k^=k>>33;k*=0xC4CEB9FE1A85EC53ull;k^=k>>33;return k;}
 void word(std::uint64_t w){a=rotl(a^(w*0x87C37B91114253D5ull),31)*0x4CF5AD432745937Full;b=rotl(b+(w^0xA0761D6478BD642Full)*0xE7037ED1A0B428DBull,27)*0x9E3779B97F4A7C15ull+0x52DCE729ull;}
 void bytes(const void*p,std::size_t n){const auto*c=static_cast<const unsigned char*>(p);std::size_t i=0;
  for(;i+8<=n;i+=8){std::uint64_t w;std::memcpy(&w,c+i,8);word(w);}
  if(i<n){std::uint64_t w=0;std::memcpy(&w,c+i,n-i);word(w^(std::uint64_t(n-i)<<56));}
  word(std::uint64_t(n));}
 template<class T> void value(const T&v){bytes(&v,sizeof v);}};
inline FitKey fitKey(const gfy_connectivity::Topology&top,const gfy_piece_collider::Capture&c,const gfy_piece_collider::Limits&l){
 FitKeyHash h;h.value(top.size);h.value(top.origin);h.value(top.pitch);h.value(top.cells);
 h.bytes(top.columns.data(),top.columns.size()*sizeof(gfy_connectivity::Column));h.bytes(top.spans.data(),top.spans.size()*sizeof(gfy_connectivity::Span));
 h.bytes(top.components.data(),top.components.size()*sizeof(gfy_connectivity::Component));
 h.value(c.component);h.value(c.anchor_bone);h.value(c.rest_to_anchor.row_major);h.value(c.anchor_to_world.row_major);h.value(c.rigid_pose_certified);
 h.value(c.maximum_skin_deviation_m);h.value(c.parent_com_world);h.value(c.parent_linear_world);h.value(c.parent_angular_world);h.value(c.density_kg_m3);
 h.value(l.max_boxes);h.value(l.max_spans);h.value(l.max_proxy_ratio);h.value(l.max_box_ratio);h.value(l.max_corner_gap_m);h.value(l.minimum_half_extent_m);
 h.value(l.max_rigid_skin_deviation_m);h.value(l.max_omitted_cell_fraction);h.value(l.max_omitted_distance_m);
 FitKey k;k.a=FitKeyHash::fmix(h.a);k.b=FitKeyHash::fmix(h.b^k.a);k.cells=top.cells;k.columns=top.columns.size();k.spans=top.spans.size();k.bone=c.anchor_bone;return k;}
struct FitCache {
 static constexpr std::size_t capacity=256;
 std::mutex mutex;std::vector<std::pair<FitKey,std::shared_ptr<const gfy_piece_collider::Prepared>>> entries; // least recently used first
 std::atomic<std::uint64_t> hits{0},misses{0}; // lookups of the separation jobs (not the prefit thread's)
 std::shared_ptr<const gfy_piece_collider::Prepared> find(const FitKey&k){std::lock_guard<std::mutex> lock(mutex);
  for(std::size_t i=entries.size();i-->0;)if(entries[i].first==k){auto e=std::move(entries[i]);entries.erase(entries.begin()+std::ptrdiff_t(i));entries.push_back(std::move(e));return entries.back().second;}
  return {};}
 void insert(const FitKey&k,const gfy_piece_collider::Prepared&p){auto v=std::make_shared<const gfy_piece_collider::Prepared>(p);
  std::lock_guard<std::mutex> lock(mutex);for(const auto&e:entries)if(e.first==k)return;
  if(entries.size()>=capacity)entries.erase(entries.begin());entries.emplace_back(k,std::move(v));}};
inline FitCache& fitCache(){static FitCache cache;return cache;}
inline std::uint64_t sfFitHash(const gfy_piece_collider::Prepared&out){std::uint64_t h=1469598103934665603ull; // the GFY_SF_FITLOG fithash
 const auto mix=[&](const void*p,std::size_t n){for(std::size_t i=0;i<n;++i){h^=static_cast<const unsigned char*>(p)[i];h*=1099511628211ull;}};
 for(const auto&b:out.boxes){mix(b.center_anchor.data(),sizeof b.center_anchor);mix(b.orientation.data(),sizeof b.orientation);mix(b.half_extents_m.data(),sizeof b.half_extents_m);}
 mix(&out.mass_kg,sizeof out.mass_kg);mix(out.com_anchor.data(),sizeof out.com_anchor);mix(out.inertia_anchor_com.data(),sizeof out.inertia_anchor_com);
 mix(&out.maximum_corner_gap_m,sizeof out.maximum_corner_gap_m);mix(&out.proxy_to_anatomy_volume_ratio,sizeof out.proxy_to_anatomy_volume_ratio);return h;}
inline bool samePrepared(const gfy_piece_collider::Prepared&a,const gfy_piece_collider::Prepared&b){ // every field but the stamp, bit for bit
 const auto eq=[](const auto&x,const auto&y){return std::memcmp(&x,&y,sizeof x)==0;};
 if(a.boxes.size()!=b.boxes.size())return false;
 for(std::size_t i=0;i<a.boxes.size();++i)if(!eq(a.boxes[i].center_anchor,b.boxes[i].center_anchor)||!eq(a.boxes[i].orientation,b.boxes[i].orientation)||!eq(a.boxes[i].half_extents_m,b.boxes[i].half_extents_m))return false;
 return a.component==b.component&&a.anchor_bone==b.anchor_bone&&a.owned_cells==b.owned_cells&&eq(a.anatomical_volume_m3,b.anatomical_volume_m3)&&eq(a.mass_kg,b.mass_kg)&&
  eq(a.com_anchor,b.com_anchor)&&eq(a.inertia_anchor_com,b.inertia_anchor_com)&&eq(a.linear_world,b.linear_world)&&eq(a.angular_world,b.angular_world)&&
  eq(a.maximum_radius_m,b.maximum_radius_m)&&eq(a.maximum_box_extent_m,b.maximum_box_extent_m)&&eq(a.proxy_to_anatomy_volume_ratio,b.proxy_to_anatomy_volume_ratio)&&
  eq(a.maximum_corner_gap_m,b.maximum_corner_gap_m)&&a.omitted_collision_cells==b.omitted_collision_cells&&eq(a.maximum_omitted_distance_m,b.maximum_omitted_distance_m);}
inline gfy_piece_collider::Prepared fitCollider(const gfy_connectivity::Topology&top,const gfy_piece_collider::Capture&capture,
                                                gfy_piece_collider::Limits limits={},CoarsePartCache*coarseCache=nullptr){
 static const bool cacheOn=[]{const char*e=std::getenv("GFY_SF_FIT_CACHE");return !e||std::atoi(e)!=0;}();
 if(!cacheOn)return fitColliderLogged(top,capture,limits,coarseCache);
 static const bool verify=std::getenv("GFY_SF_FIT_CACHE_VERIFY")!=nullptr,fitLog=std::getenv("GFY_SF_FITLOG")!=nullptr;
 if(!(capture.accepted.owner.spawn_cookie&&capture.accepted.revision))return fitColliderLogged(top,capture,limits,coarseCache); // fails as before (collider_identity)
 const auto t=core::Clock::now();const FitKey key=fitKey(top,capture,limits);auto&cache=fitCache();const bool job=!sfPrefitThread();
 if(const auto hit=cache.find(key)){if(job)cache.hits.fetch_add(1,std::memory_order_relaxed);
  gfy_piece_collider::Prepared out=*hit;out.accepted=capture.accepted;
  if(fitLog){const auto cells=capture.component<top.components.size()?top.components[capture.component].cells:0;std::ostringstream line;
   line<<"sf_fit bone="<<capture.anchor_bone<<" cells="<<cells<<" spans="<<top.spans.size()<<" columns="<<top.columns.size()<<" boxes="<<out.boxes.size()
    <<" max_boxes="<<limits.max_boxes<<" fithash="<<std::hex<<sfFitHash(out)<<std::dec<<" cached=1"<<(job?"":" prefit=1")<<" ms="<<core::ms(t)<<'\n';std::cerr<<line.str()<<std::flush;}
  if(verify){const auto fresh=fitColliderCore(top,capture,limits,coarseCache);const bool same=samePrepared(fresh,out);std::ostringstream line;
   line<<(same?"sf_fitcache_verify ok":"sf_fitcache_verify MISMATCH")<<" bone="<<capture.anchor_bone<<" cells="<<top.cells<<" boxes="<<out.boxes.size()<<" max_boxes="<<limits.max_boxes<<'\n';std::cerr<<line.str()<<std::flush;}
  return out;}
 if(job)cache.misses.fetch_add(1,std::memory_order_relaxed);
 auto out=fitColliderLogged(top,capture,limits,coarseCache);
 cache.insert(key,out);return out;}
// Per-model decoded separation inputs: a pure function of the model and its files (keyed by their expected hashes).
struct DecodedInputs {bound_surface::Bindings bindings;gfy_piece_collider::original_frames::Frames frames;std::vector<std::uint16_t> bindingElements;
 std::unique_ptr<const sep::SourceFaceIndex> sourceFaces;std::unique_ptr<const bound_surface::ExtraSurface> accessorySurface;};
inline std::shared_ptr<const DecodedInputs> decodedInputs(const Asset&asset,const SeparationInput&input,const std::shared_ptr<const gfy_connectivity::Topology>&pristine){
 sep::KeyBytes key;key.add(asset.source_hash).add(input.anatomy_hash).add(input.fit_hash).add(input.surface_bindings.expected).add(input.coat_roots.expected)
  .add(input.bone_frames.expected).add(input.accessory_surfaces.expected).add(input.physics_elements.expected)
  .add(pristine->pitch).add(pristine->origin).add(pristine->size).add(std::uint32_t(0x44454331u))
  .add(std::uint64_t(asset.gear_body.merged_faces)); // gearbody
 const std::string k=key.str();
 auto&cache=decodedCache();auto&mutex=cache.mutex;auto&entries=cache.entries; // least recently used first
 {std::lock_guard<std::mutex> lock(mutex);
  for(std::size_t i=0;i<entries.size();++i)if(entries[i].first==k){auto e=entries[i];entries.erase(entries.begin()+std::ptrdiff_t(i));entries.push_back(e);return e.second;}}
 auto d=std::make_shared<DecodedInputs>();
 // gearbody: the merged gear's binding rows follow the body's; the coat roots get a 0 flag per gear face
 d->bindings=bound_surface::read(gearBodyBindings(asset,read(input.surface_bindings,input.hash),
  asset.gear_body.merged_faces?read(input.accessory_surfaces,input.hash):std::vector<std::uint8_t>{}),asset);
 if(!input.coat_roots.path.empty())bound_surface::applyCoatRoots(d->bindings,gearBodyCoatRoots(asset,read(input.coat_roots,input.hash)),asset);
 d->frames=gfy_piece_collider::original_frames::decode(read(input.bone_frames,input.hash),asset.source_hash,input.fit_hash);
 need(d->frames.bones.size()==asset.source.bones,"separation_original_bone_count");
 const auto map=physicsMap(asset,pristine,input,d->frames);
 d->bindingElements.reserve(d->bindings.vertices.size());
 for(const auto&binding:d->bindings.vertices)d->bindingElements.push_back(originalBindingElement(*map,binding.point));
 d->sourceFaces=std::make_unique<const sep::SourceFaceIndex>(asset.face_children,asset.face_indices,asset.source.f.size());
 // gearbody: with merged gear, the gear surface is the one the asset kept (eyes, out-of-reach gear; none: no gear stage)
 if(!input.accessory_surfaces.path.empty()&&(!asset.gear_body.merged_faces||!asset.accessory_surface_bytes.empty()))
  d->accessorySurface=std::make_unique<const bound_surface::ExtraSurface>(readAccessorySurfaces(asset,asset.gear_body.merged_faces?asset.accessory_surface_bytes:read(input.accessory_surfaces,input.hash)));
 {std::lock_guard<std::mutex> lock(mutex);entries.emplace_back(k,d);while(entries.size()>12)entries.erase(entries.begin());}
 return d;
}
// ---- speedfix: the prefit (fits of damaged elements made ahead) -----------------------------------------------------------
// After a hit on a whole body (its acknowledge, with the input the model was warmed with) and after a no-split check
// (prepareSnapshot), one low-priority thread takes that snapshot: derive() of its remaining volume (as the separation
// derives it), the physical partition of every substantial component (partitionFused / intersect, as the separation), and
// fitCollider() of every element part whose shape is not the original one (a damaged element), with the capture the
// separation's piece loop gives it. The fits land in the fit cache. A later sever of the body asks the same fits for the
// element parts it leaves as they were (same cells: same part volume; same element: same capture), so it fits only what
// the sever changed. Nothing else is shared: the results are the fit cache's (see there).
// Scheduling: one queued job per body (a newer one replaces it; a running older one stops after its current fit); a hit's
// job starts GFY_SF_PREFIT_DELAY ms (default 50) after the hit unless a no-split check of that snapshot confirms it first;
// a separation of the body drops its jobs; while real separation work runs (sep::activeJobs) the thread waits (its fits at
// their cancel points); every parallel section runs on this one thread. GFY_SF_PREFIT=0: off.
#define GFY_SF_PREFIT_API 1
struct PrefitStats {std::uint64_t served{},fitted{},failed{},hits{},misses{};double lastMs{};};
struct PrefitService {
 struct Job {std::shared_ptr<const Asset>asset;std::shared_ptr<const Grid>grid;std::shared_ptr<const SeparationInput>input;Stamp stamp;
  core::Clock::time_point due;bool confirmed{};};
 static constexpr std::size_t capacity=8;
 std::mutex mutex;std::condition_variable wake,idle;std::deque<Job>jobs;bool stop{},running{};Identity runningOwner;std::uint64_t runningRevision{};
 std::atomic<bool> cancel{false},superseded{false};std::uint64_t served{},fitted{},failed{};double lastMs{};std::thread worker;
 PrefitService():worker(&PrefitService::run,this){}
 void quit()noexcept{{std::lock_guard<std::mutex>lock(mutex);stop=true;}cancel.store(true);wake.notify_all();if(worker.joinable())worker.join();} // quitfix
 ~PrefitService(){quit();}
 void run();};
inline std::size_t prefitBody(const Asset&asset,std::shared_ptr<const Grid>&grid,const SeparationInput&input,const Stamp&stamp,PrefitService&service){
 namespace pc=gfy_piece_collider;
 const sep::NestedScope one; // every sep::parallel section below stays on this thread
 const auto waitJobs=[]{while(sep::activeJobs().load()>0){sep::throwIfCanceled();std::this_thread::sleep_for(std::chrono::milliseconds(1));}sep::throwIfCanceled();};
 waitJobs();
 const auto reference=pristineReference(asset,input,sep::Request::warm);
 if(!reference.topology||!derivableFromPristine(reference,*grid))return 0;
 auto volume=gfy_connectivity::derive(*reference.topology,*grid);
 grid.reset(); // from here on only the derived volume (the session may recycle its grid)
 waitJobs();
 const auto decoded=decodedInputs(asset,input,reference.topology);const auto&frames=decoded->frames;
 const auto map=physicsMap(asset,reference.topology,input,frames,sep::Request::warm);if(!map)return 0;
 std::vector<std::uint32_t> ids;const double cell=volume.pitch*volume.pitch*volume.pitch;
 for(std::uint32_t id=1;id<volume.components.size();++id)if(double(volume.components[id].cells)*cell>=input.minimum_piece_m3)ids.push_back(id);
 if(ids.empty())return 0;
 std::map<std::uint16_t,std::uint64_t> remaining;std::map<std::uint32_t,std::vector<pc::element_partition::Element>> parts;
 if(!pc::element_partition::partitionFused(*map,volume,ids,remaining,parts)){parts.clear();for(const auto id:ids)parts.emplace(id,pc::element_partition::intersect(*map,volume,id));}
 volume=gfy_connectivity::Topology{};
 std::vector<const pc::element_partition::Element*> list;
 for(const auto&entry:parts)for(const auto&p:entry.second)
  if(!p.original_shape_complete&&p.bone<frames.bones.size()&&frames.bones[p.bone].physical_element==p.bone)list.push_back(&p);
 std::stable_sort(list.begin(),list.end(),[](const pc::element_partition::Element*a,const pc::element_partition::Element*b){return a->volume.cells>b->volume.cells;});
 std::size_t fitted=0;
 for(const auto*p:list){if(service.superseded.load())break;waitJobs();
  pc::Capture capture;capture.accepted=stamp;capture.component=1;capture.anchor_bone=p->bone; // as the piece loop's (selectPrefits)
  capture.rest_to_anchor=frames.bones[p->bone].rest_to_anchor;capture.anchor_to_world=frames.bones[p->bone].anchor_to_rest;capture.rigid_pose_certified=true;
  CoarsePartCache coarse;const pc::detail::SerialScope serial;
  try{(void)fitCollider(p->volume,capture,{},&coarse);++fitted;}
  catch(const std::bad_alloc&){throw;}
  catch(const std::exception&){sep::throwIfCanceled();} // a part that does not fit: the separation meets that itself, as before
 }
 return fitted;}
inline void PrefitService::run(){
 core::enterPainterThread();lowerCurrentThreadPriority();sfPrefitThread()=true;gfy_piece_collider::detail::pauseWhile()=&sep::activeJobs();
 const sep::CancelScope scope(&cancel);
 for(;;){Job job;
  {std::unique_lock<std::mutex>lock(mutex);
   for(;;){if(stop)return;
    if(jobs.empty()){idle.notify_all();wake.wait(lock);continue;}
    const auto now=core::Clock::now();auto pick=jobs.end();auto first=jobs.front().due;
    for(auto it=jobs.begin();it!=jobs.end();++it){if(it->confirmed||it->due<=now){pick=it;break;}if(it->due<first)first=it->due;}
    if(pick==jobs.end()){wake.wait_until(lock,first);continue;}
    job=std::move(*pick);jobs.erase(pick);break;}
   running=true;runningOwner=job.stamp.owner;runningRevision=job.stamp.revision;cancel.store(false);superseded.store(false);}
  const auto t=core::Clock::now();std::size_t n=0;bool ok=true;
  try{n=prefitBody(*job.asset,job.grid,*job.input,job.stamp,*this);}catch(...){ok=false;}
  job=Job{}; // its grid, model pin and input go here, off the lock
  {std::lock_guard<std::mutex>lock(mutex);++served;fitted+=n;failed+=ok?0:1;lastMs=core::ms(t);running=false;}
  idle.notify_all();}}
inline std::atomic<bool>& prefitStarted(){static std::atomic<bool> started{false};return started;}
inline PrefitService& prefitService(){
 (void)pristineCache();(void)mapCache();(void)tileBaseCache();(void)decodedCache();(void)fitCache();(void)prefitInputs(); // outlive its thread
 static PrefitService& service=::gfy_live::core::worker_quit::immortal<PrefitService>(0);prefitStarted().store(true);return service;} // quitfix: never destroyed
inline bool prefitOn(){static const bool on=[]{const char*e=std::getenv("GFY_SF_PREFIT");return !e||std::atoi(e)!=0;}();return on;}
// Queues the prefit of this snapshot (input: shared, or copied from `raw` when a job is made). confirmed: a no-split check.
inline bool prefitQueue(const std::shared_ptr<const Asset>&asset,const std::shared_ptr<const Grid>&grid,std::shared_ptr<const SeparationInput>input,
                        const SeparationInput*raw,const Stamp&stamp,bool confirmed)noexcept{
 static const int delayMs=[]{const char*e=std::getenv("GFY_SF_PREFIT_DELAY");const int v=e?std::atoi(e):50;return v>=0&&v<=2000?v:50;}();
 if(!prefitOn()||!asset||!grid||(!input&&!raw)||!stamp.revision)return false;
 try{auto&s=prefitService();std::vector<PrefitService::Job> dropped;
  {std::lock_guard<std::mutex>lock(s.mutex);if(s.stop)return false;
   if(s.running&&s.runningOwner==stamp.owner){if(s.runningRevision>=stamp.revision)return true;s.superseded.store(true);}
   for(auto it=s.jobs.begin();it!=s.jobs.end();++it){if(!(it->stamp.owner==stamp.owner))continue;
    if(it->stamp.revision==stamp.revision){if(confirmed&&!it->confirmed){it->confirmed=true;s.wake.notify_all();}return true;}
    if(it->stamp.revision>stamp.revision)return true; // a newer state of this body is queued
    dropped.push_back(std::move(*it));s.jobs.erase(it);break;}
   if(s.jobs.size()>=PrefitService::capacity){dropped.push_back(std::move(s.jobs.front()));s.jobs.pop_front();}
   PrefitService::Job job;job.asset=asset;job.grid=grid;job.input=input?std::move(input):std::make_shared<const SeparationInput>(*raw);job.stamp=stamp;
   job.due=core::Clock::now()+std::chrono::milliseconds(confirmed?0:delayMs);job.confirmed=confirmed;s.jobs.push_back(std::move(job));}
  s.wake.notify_all();return true;}catch(...){return false;}}
// A separation of this body: its queued jobs go, a running one stops.
inline void prefitDrop(const Identity&owner)noexcept{
 if(!prefitStarted().load())return;
 try{auto&s=prefitService();std::vector<PrefitService::Job> dropped;
  {std::lock_guard<std::mutex>lock(s.mutex);
   for(auto it=s.jobs.begin();it!=s.jobs.end();)if(it->stamp.owner==owner){dropped.push_back(std::move(*it));it=s.jobs.erase(it);}else ++it;
   if(s.running&&s.runningOwner==owner)s.cancel.store(true);}
  s.idle.notify_all();}catch(...){}}
// Read-outs (probe): wait until no job is queued or running; the counters.
inline bool prefitWaitIdle(double timeoutMs){auto&s=prefitService();std::unique_lock<std::mutex>lock(s.mutex);
 return s.idle.wait_for(lock,std::chrono::duration<double,std::milli>(timeoutMs),[&]{return s.jobs.empty()&&!s.running;});}
inline PrefitStats prefitStats(){PrefitStats st;auto&s=prefitService();{std::lock_guard<std::mutex>lock(s.mutex);st.served=s.served;st.fitted=s.fitted;st.failed=s.failed;st.lastMs=s.lastMs;}
 st.hits=fitCache().hits.load();st.misses=fitCache().misses.load();return st;}
}
// speedfix: after a hit is acknowledged (LiveDamageWorker.cpp Session::acknowledge): the prefit of that snapshot, with the
// input the model was warmed with (none recorded: nothing).
bool prefit_after_commit(const std::shared_ptr<const Asset>&asset,const std::shared_ptr<const Grid>&grid,const Stamp&stamp)noexcept{
 try{if(!separation_detail::prefitOn()||!asset||!grid)return false;auto input=separation_detail::prefitInputOf(asset);if(!input)return false;
  return separation_detail::prefitQueue(asset,grid,std::move(input),nullptr,stamp,false);}catch(...){return false;}}
namespace separation_detail {
inline std::shared_ptr<SeparationOutput> prepare(const Asset&asset,const Grid&grid,const Stamp&stamp,const SeparationInput&input,
    const std::set<std::uint32_t>*wantedComponents=nullptr,const SeparationCheckpoint& checkpoint={},
    const PrepareExtras& extras={}){
 namespace pc=gfy_piece_collider;
 const auto progress=[&](const char* name,std::uint32_t component=0,std::uint16_t bone=65535){
  if(checkpoint)checkpoint(name,component,bone);};
#ifdef GFY_COLLIDER_DIAGNOSTICS
 const auto separationStart=core::Clock::now();
 auto phase=[&](const char*name){std::cerr<<"separation_phase="<<name<<" elapsed_ms="<<core::ms(separationStart)<<'\n';};
#endif
 auto out=std::make_shared<SeparationOutput>();out->stamp=stamp;out->gore=extras.gore;
 need(stamp.owner.source==asset.source_hash&&stamp.owner.exact_rig==asset.rig_hash,"separation_snapshot_identity");
 need(stamp.revision&&stamp.owner.spawn_cookie,"separation_snapshot_unmodified");
 need(std::isfinite(input.minimum_piece_m3)&&input.minimum_piece_m3>=.0000001&&input.minimum_piece_m3<=.001,"separation_piece_volume_limit");
 need(input.physical_parents.size()==asset.source.bones,"separation_original_physics_tree");
 auto phaseStart=core::Clock::now();
 const auto lap=[&](SeparationPhase phase){const auto now=core::Clock::now();
  out->phase_ms[phase]=std::chrono::duration<double,std::milli>(now-phaseStart).count();phaseStart=now;};
 progress("remaining_connectivity");
 static const bool sepProf=std::getenv("GFY_SEP_PROF")!=nullptr;auto profStart=core::Clock::now();
 const auto profMark=[&](const char*step){if(!sepProf)return;std::cerr<<"sep_prof "<<step<<' '<<core::ms(profStart)<<std::endl;profStart=core::Clock::now();};
 std::shared_ptr<const gfy_connectivity::Topology>volumeShared=extras.precomputed_volume,pristineShared;
 PristineRef reference;
 // P1.4: parent (pristine) span of every remaining span when the volume is derived.
 std::vector<std::uint32_t>pristineSpans;bool derived=false;
 // 62p: a part's volume derived from its birth topology (PieceSeed::topology), and each span's parent span there.
 std::vector<std::uint32_t>pieceSpans;bool pieceDerived=false;
 static const bool verifyPiece=std::getenv("GFY_SEP_PIECE_VERIFY")!=nullptr;
 // A: tile-graph verdict (RemainingConnectivity.h tiles::). For a grid derivable
 // from the cached pristine topology, "still connected" is decided from the
 // pristine tile graph and the grid's removed tiles alone (no span topology,
 // no debris scan); a split runs the span path below unchanged. Under
 // GFY_SEP_VERIFY_TILES both run and the verdict and the full cell partition are
 // compared.
 std::optional<gfy_connectivity::tiles::Verdict> tileVerdict;
 struct EarlyDerive {std::atomic<bool> stop{false};std::thread worker;gfy_connectivity::Topology volume;std::vector<std::uint32_t> spans;
  std::uint64_t unionLabel=0;std::exception_ptr error;
  void join(){if(worker.joinable())worker.join();}
  ~EarlyDerive(){stop=true;join();}};
 std::unique_ptr<EarlyDerive> early;
 // split2 part 7: the working copy of the grid for the debris erase below, made on its own thread while this thread waits
 // for the derive, when the tile verdict has already seen debris. GFY_SEP_DEBRIS_FAST=0: the former debris path.
 struct EarlyGridCopy {Grid copy;std::exception_ptr error;std::thread worker;~EarlyGridCopy(){if(worker.joinable())worker.join();}};
 std::unique_ptr<EarlyGridCopy> earlyGrid;
 static const bool debrisFast=[]{const char*e=std::getenv("GFY_SEP_DEBRIS_FAST");return !e||std::atoi(e)!=0;}();
#ifdef GFY_SEP_VERIFY_TILES
 gfy_connectivity::tiles::Labeling<Grid> tileLabels;constexpr bool verifyTiles=true;
#else
 constexpr bool verifyTiles=false;
#endif
 if(!volumeShared&&tileVerdictEnabled()){
  try{reference=pristineReference(asset,input);}
  catch(const std::exception&e){if(std::strcmp(e.what(),"separation_canceled")!=0)(void)gfy_connectivity::build(grid);throw;}
  catch(...){(void)gfy_connectivity::build(grid);throw;}
  pristineShared=reference.topology;profMark("reference");
  if(derivableFromPristine(reference,grid)){const auto base=tileBase(asset,input,pristineShared);profMark("tile_base");
#ifndef GFY_SEP_DERIVED_CONNECTIVITY_BUDGETS
   static const bool serialVerdict=std::getenv("GFY_SEP_SERIAL_VERDICT")!=nullptr;
   if(base&&!serialVerdict&&!verifyTiles&&!extras.precomputed_volume){early=std::make_unique<EarlyDerive>();
    try{early->worker=std::thread([e=early.get(),&grid,pristine=pristineShared]{core::enterPainterThread();const sep::CancelScope scope(&e->stop);
      // Charge the coalesced operation against the unchanged limits, as
      // owned-piece continuation does. No legacy raw-run fallback is needed.
      try{e->volume=gfy_connectivity::derive(*pristine,grid,&e->spans,{},&e->unionLabel);}
      catch(...){e->error=std::current_exception();}});}
    catch(...){early.reset();}}
#endif
   if(base){
#ifdef GFY_SEP_VERIFY_TILES
    tileVerdict=gfy_connectivity::tiles::verdict(*base,grid,input.minimum_piece_m3,&tileLabels);
#else
    tileVerdict=gfy_connectivity::tiles::verdict(*base,grid,input.minimum_piece_m3);
#endif
#ifdef GFY_COLLIDER_DIAGNOSTICS
    std::cerr<<"tile_verdict separated="<<tileVerdict->separated<<" components="<<tileVerdict->components<<" remaining="<<tileVerdict->remaining_components
             <<" largest_split="<<tileVerdict->largest_substantial_split<<" debris="<<tileVerdict->debris_components<<"/"<<tileVerdict->debris_cells
             <<" touched_tiles="<<tileVerdict->touched_tiles<<" touched_nodes="<<tileVerdict->touched_nodes<<" air_tiles="<<tileVerdict->air_tiles
             <<" base_tiles="<<base->keys.size()<<" base_nodes="<<base->nodes.size()<<" base_mb="<<double(base->bytes())/1048576.
             <<" ms_touch="<<tileVerdict->ms_touch<<" ms_union="<<tileVerdict->ms_union<<" ms_verdict="<<tileVerdict->ms_verdict<<std::endl;
#endif
   }}
  profMark("tile_verdict");
  if(tileVerdict&&!tileVerdict->separated&&!verifyTiles){
   lap(separation_phase_remaining_connectivity);progress("original_connectivity");
   lap(separation_phase_original_connectivity);progress("compare_connectivity");
   out->original_components=std::uint32_t(pristineShared->components.size()-1);
   out->remaining_components=tileVerdict->remaining_components;
   out->largest_substantial_split=tileVerdict->largest_substantial_split;
   lap(separation_phase_compare_connectivity);
   out->no_separation=true;return out;
  }
 }
 if(debrisFast&&tileVerdict&&tileVerdict->separated&&tileVerdict->debris_components&&!verifyTiles){
  try{auto copy=std::make_unique<EarlyGridCopy>();auto*e=copy.get();
   e->worker=std::thread([e,&grid]{core::enterPainterThread();try{e->copy=grid;e->copy.journal=nullptr;}catch(...){e->error=std::current_exception();}});
   earlyGrid=std::move(copy);}
  catch(...){earlyGrid.reset();}}
 // split (2026-11-11): a separated piece's still-connected verdict from the tile graph of its birth topology
 // (RemainingConnectivity.h verdictOwned): no span topology, no debris erase, no rebuilt topology. Used only for
 // "not split" with a usable result; a split, an unusable result or a failed base build runs the span path below unchanged.
 std::optional<gfy_connectivity::tiles::OwnedVerdict> pieceVerdict;
 static const bool pieceTileVerify=std::getenv("GFY_SEP_PIECE_TILE_VERIFY")!=nullptr;
 if(!volumeShared&&!tileVerdict&&pieceTileVerdictApplies(extras.piece,grid)){
  std::shared_ptr<const gfy_connectivity::tiles::Base> pieceBase;
  try{pieceBase=pieceTileVerify?pieceTileBases().now(extras.piece->topology):pieceTileBases().ready(extras.piece->topology);}
  catch(const std::bad_alloc&){pieceBase.reset();}
  catch(const std::exception&e){if(sep::cancelRequested()||std::strcmp(e.what(),"separation_canceled")==0)throw;pieceBase.reset();}
  profMark("piece_tile_base");
  if(pieceBase){
   const double boneless=std::max(input.minimum_piece_m3,input.boneless_piece_m3);
   pieceVerdict=gfy_connectivity::tiles::verdictOwned(*pieceBase,grid,input.minimum_piece_m3,grid.material_slots?boneless:input.minimum_piece_m3);
   if(sepProf)std::cerr<<"sep_prof_piece_verdict usable="<<pieceVerdict->usable<<" refusal="<<pieceVerdict->refusal<<" separated="<<pieceVerdict->separated
    <<" components="<<pieceVerdict->components<<" debris="<<pieceVerdict->debris_components<<" touched_tiles="<<pieceVerdict->touched_tiles<<" touched_nodes="<<pieceVerdict->touched_nodes
    <<" base_tiles="<<pieceBase->keys.size()<<" base_nodes="<<pieceBase->nodes.size()<<" ms_touch="<<pieceVerdict->ms_touch<<" ms_union="<<pieceVerdict->ms_union<<" ms_verdict="<<pieceVerdict->ms_verdict<<std::endl;
   profMark("piece_tile_verdict");
   if(pieceVerdict->usable&&!pieceVerdict->separated&&!pieceTileVerify){
    lap(separation_phase_remaining_connectivity);progress("original_connectivity");
    if(!pristineShared)pristineShared=pristineTopology(asset,input);
    lap(separation_phase_original_connectivity);progress("compare_connectivity");
    out->original_components=std::uint32_t(pristineShared->components.size()-1);
    out->remaining_components=pieceVerdict->remaining_components;
    out->largest_substantial_split=pieceVerdict->largest_substantial_split;
    lap(separation_phase_compare_connectivity);
    out->no_separation=true;return out;
   }
  }
 }
 if(volumeShared){
  need(volumeShared->pitch==grid.pitch&&volumeShared->size==std::array<int,3>{grid.size.x,grid.size.y,grid.size.z}&&
       volumeShared->origin==std::array<double,3>{grid.origin.x,grid.origin.y,grid.origin.z},"separation_precomputed_volume_domain");
#ifdef GFY_SEP_VERIFY
  need(sameTopology(gfy_connectivity::build(grid),*volumeShared),"separation_precomputed_volume_mismatch");
#endif
 }else{
  // Derived connectivity (P1.4): the remaining spans are the cached pristine
  // spans minus the grid's removed cells; exact (see gfy_connectivity::derive).
  // Base order: build(grid) ran before the pristine build, so a failure of
  // both reports build(grid)'s error.
  // Reuse the complete successful reference from the tile verdict above.
  // A cancel (P1.5) is rethrown at once; any other failure first runs build(grid).
  if(!reference.topology){
   try{reference=pristineReference(asset,input);}
   catch(const std::exception&e){if(std::strcmp(e.what(),"separation_canceled")!=0)(void)gfy_connectivity::build(grid);throw;}
   catch(...){(void)gfy_connectivity::build(grid);throw;}
  }
  pristineShared=reference.topology;
  if(derivableFromPristine(reference,grid)){
   std::uint64_t unionLabel=0;
#ifdef GFY_SEP_DERIVED_CONNECTIVITY_BUDGETS
   // The serial configuration uses the same bounded exact derivation.
   volumeShared=std::make_shared<const gfy_connectivity::Topology>(gfy_connectivity::derive(*pristineShared,grid,&pristineSpans,{},&unionLabel));derived=true;
#else
   // Apply the same work/span/removal-run limits to the exact coalesced
   // derivation. The old builder counts adjacent 8-cell fragments separately
   // and can reject this identical topology despite its bounded useful work.
   gfy_connectivity::Topology volume;profMark("reference2");
   if(early){early->join();if(early->error)std::rethrow_exception(early->error);
    volume=std::move(early->volume);pristineSpans=std::move(early->spans);unionLabel=early->unionLabel;early.reset();profMark("derive_joined");}
   else{volume=gfy_connectivity::derive(*pristineShared,grid,&pristineSpans,{},&unionLabel);profMark("derive");}
   volumeShared=std::make_shared<const gfy_connectivity::Topology>(std::move(volume));derived=true;
#endif
#ifdef GFY_SEP_VERIFY
   // Verification only: the legacy oracle must compare the full result,
   // including inputs its obsolete raw-fragment budget would reject.
   need(sameTopology(gfy_connectivity::build(grid,gfy_connectivity::unlimitedLimits()),*volumeShared),"separation_derived_volume_mismatch");
#endif
  }else if(extras.piece&&extras.piece->topology&&extras.piece->topology->pitch==grid.pitch&&
           extras.piece->topology->size==std::array<int,3>{grid.size.x,grid.size.y,grid.size.z}&&
           extras.piece->topology->origin==std::array<double,3>{grid.origin.x,grid.origin.y,grid.origin.z}){
   // As component_detail::exactBuild: use bounded exact derivation.
   std::uint64_t unionLabel=0;
   // Derivation coalesces adjacent removals before charging the same run
   // limit. Do not reject it using a different, unmerged builder's estimate.
   auto volume=gfy_connectivity::derive(*extras.piece->topology,grid,&pieceSpans,{},&unionLabel);profMark("piece_derive");
   volumeShared=std::make_shared<const gfy_connectivity::Topology>(std::move(volume));pieceDerived=true;
   if(verifyPiece){const auto rebuilt=gfy_connectivity::build(grid);
    need(sameTopology(rebuilt,*volumeShared)&&rebuilt.cells==volumeShared->cells,"separation_piece_derive_mismatch");
    std::cerr<<"piece_derive_verify ok spans="<<rebuilt.spans.size()<<" components="<<rebuilt.components.size()<<std::endl;profMark("piece_verify");}
  }else volumeShared=std::make_shared<const gfy_connectivity::Topology>(gfy_connectivity::build(grid));
 }
 if(extras.volume_out)*extras.volume_out=volumeShared;
 double minimumPiece=input.minimum_piece_m3;
 std::vector<char> scrap(volumeShared->components.size(),0),detachedEyes(volumeShared->components.size(),0);bool anyDetachedEye=false;
 {const double cell=grid.pitch*grid.pitch*grid.pitch;const double boneless=std::max(minimumPiece,input.boneless_piece_m3);
  const auto&v0=*volumeShared;
  if(boneless>minimumPiece&&grid.material_slots){const auto&slots=*grid.material_slots;
   std::vector<char> small(v0.components.size(),0),bone(v0.components.size(),0);bool any=false;
   std::vector<std::uint64_t> ocular(v0.components.size(),0);
   for(std::uint32_t id=1;id<v0.components.size();++id){const double m3=double(v0.components[id].cells)*cell;if(m3>=minimumPiece&&m3<boneless){small[id]=1;any=true;}}
   if(any){for(const auto&col:v0.columns)for(auto s=col.first;s<col.first+col.count;++s){const auto&span=v0.spans[s];
     if(!span.component||!small[span.component]||bone[span.component])continue;
     for(int z=span.lo;z<span.hi;++z){const auto label=grid.materialBase(Cell{col.x,col.y,z});
      if(label<slots.size()&&slots[label]==2){bone[span.component]=1;break;}
      if(label<slots.size()&&slots[label]==14)++ocular[span.component];}}
    bool hasEye=false;for(std::uint32_t id=1;id<v0.components.size();++id)
     hasEye=hasEye||eye_remnants::ocular(v0.components[id].cells,ocular[id],bone[id]);
    if(hasEye){
     if(!pristineShared)pristineShared=pristineTopology(asset,input);
     const auto&reference=extras.piece&&extras.piece->topology?*extras.piece->topology:*pristineShared;
     // A pre-existing isolated atlas eye must keep its authored attachment.
     // Require a genuine one-to-many split of its original tissue component.
     const auto changes=gfy_connectivity::compare(reference,v0);
     for(const auto&change:changes)if(change.pieces.size()>1)
      for(const auto&piece:change.pieces)if(piece.component<detachedEyes.size()&&
        eye_remnants::ocular(v0.components[piece.component].cells,ocular[piece.component],bone[piece.component]))
       {detachedEyes[piece.component]=1;anyDetachedEye=true;}
    }
    // cutfix: a slice cut off between a fresh decide-once tear and an earlier cut is a piece, not a scrap (MaskPieces3.h tearSlices; value 3 only)
    std::vector<char> tearSlice;mask_pieces3::tearSlices(extras.lowpoly,grid,v0,small,tearSlice);
    // headsize: a head event's small fragment is a chunk that flies, not a scrap (HeadBurstPieces.h chunks; value 3 only)
    std::vector<char> headChunk;mask_pieces3::head_burst_pieces::chunks(extras.lowpoly,grid,v0,small,headChunk);
    for(std::uint32_t id=1;id<v0.components.size();++id)
     // Both modes require an exact surface ownership packet. Mode 0 commits
     // the body without spawning its partitioned eye, rather than returning
     // no_separation and leaving the old authored eye behind on the body.
     scrap[id]=small[id]&&!bone[id]&&!detachedEyes[id]&&!tearSlice[id]&&!headChunk[id];
   }}}
 mask_pieces3::head_burst_pieces::slivers(extras.lowpoly,grid,*volumeShared,detachedEyes,scrap); // headfix: a head burst's slivers are scraps (value 3)
 profMark("scrap_scan");
 std::vector<Cell> eyeWitness;
 if(anyDetachedEye){std::vector<char> seen(detachedEyes.size(),0);
  for(const auto&col:volumeShared->columns)for(auto s=col.first;s<col.first+col.count;++s){const auto&span=volumeShared->spans[s];
   if(span.component<detachedEyes.size()&&detachedEyes[span.component]&&!seen[span.component]){
    seen[span.component]=1;eyeWitness.push_back({col.x,col.y,span.lo});}}}
 {constexpr std::size_t maxPieces=12;const double cell=grid.pitch*grid.pitch*grid.pitch;std::vector<std::uint64_t> sizes;
  for(std::uint32_t id=1;id<volumeShared->components.size();++id)if(!scrap[id]&&double(volumeShared->components[id].cells)*cell>=minimumPiece)sizes.push_back(volumeShared->components[id].cells);
  if(sizes.size()>maxPieces){std::nth_element(sizes.begin(),sizes.begin()+(maxPieces-1),sizes.end(),std::greater<std::uint64_t>());
   minimumPiece=std::max(minimumPiece,double(sizes[maxPieces-1])*cell);}}
 Grid working;const Grid*activeGrid=&grid;
 {const auto minimal=[&](std::uint32_t id){return scrap[id]||volumeShared->components[id].cells*grid.pitch*grid.pitch*grid.pitch<minimumPiece;};
  const auto&v0=*volumeShared;std::vector<char> touchesCut(v0.components.size(),0);std::uint64_t debris=0;
  // R66: an unowned topology span contains only valid, wholly unremoved cells.
  // Its internal z-neighbors therefore cannot touch a cut; only its two z ends
  // and the four side-column intervals can. Owned grids retain the scalar path
  // because Grid::removed also represents implicit foreign-owner removals.
  const auto touchesRemovedSpan=[&](int x,int y,int lo,int hi){
   if(!grid.ownership){
    const auto rawInterval=[&](int qx,int qy,int first,int last){
     if(qx<0||qy<0||qx>=grid.size.x||qy>=grid.size.y)return false;
     first=std::max(first,0);last=std::min(last,grid.size.z);
     for(int z=first;z<last;){const int end=z+std::min(last-z,8-(z&7));const Cell q{qx,qy,z};
      if(const auto*t=grid.findTile(Grid::tileKey(q))){const auto bit=Grid::local(q);
       const auto mask=((std::uint64_t(1)<<unsigned(end-z))-1)<<(bit%64);
       if(t->removed[bit/64]&mask)return true;}
      z=end;
     }
     return false;
    };
    const Cell below{x,y,lo-1},above{x,y,hi};
    return rawInterval(x-1,y,lo,hi)||rawInterval(x+1,y,lo,hi)||
           rawInterval(x,y-1,lo,hi)||rawInterval(x,y+1,lo,hi)||
           (grid.valid(below)&&grid.removed(below))||(grid.valid(above)&&grid.removed(above));
   }
   for(int z=lo;z<hi;++z){const Cell c{x,y,z};
    for(int a=0;a<3;++a)for(int sgn:{-1,1}){Cell q=c;(&q.x)[a]+=sgn;if(grid.valid(q)&&grid.removed(q))return true;}}
   return false;
  };
  // split2 part 7: only a component below the minimum can be debris, and all its spans lie in its bounding box. When those
  // boxes are a small part of the columns, the spans are found through them (the same test per span, the same marks).
  std::vector<std::uint32_t> minimalIds;std::uint64_t boxColumns=0;
  if(debrisFast)for(std::uint32_t id=1;id<v0.components.size();++id){const auto&c=v0.components[id];if(!c.cells||!minimal(id))continue;minimalIds.push_back(id);
   boxColumns+=std::uint64_t(std::max(0,c.hi[0]-c.lo[0]))*std::uint64_t(std::max(0,c.hi[1]-c.lo[1]));}
  const bool boxScan=debrisFast&&boxColumns*4<=v0.columns.size();
  // visit(column, span index) for every span of one component, in column order; false stops
  const auto spansOf=[&](std::uint32_t id,auto&&visit){const auto&c=v0.components[id];
   for(int y=c.lo[1];y<c.hi[1];++y){
    auto col=std::lower_bound(v0.columns.begin(),v0.columns.end(),std::array<int,2>{{c.lo[0],y}},[](const gfy_connectivity::Column&a,const std::array<int,2>&q){return gfy_connectivity::xyLess(a.x,a.y,q[0],q[1]);});
    for(;col!=v0.columns.end()&&col->y==y&&col->x<c.hi[0];++col)for(auto s=col->first;s<col->first+col->count;++s)if(v0.spans[s].component==id&&!visit(*col,s))return;}};
  if(boxScan){for(const auto id:minimalIds)spansOf(id,[&](const gfy_connectivity::Column&col,std::uint32_t s){const auto&span=v0.spans[s];
    if(!touchesRemovedSpan(col.x,col.y,span.lo,span.hi))return true;touchesCut[id]=1;return false;});}
  else
  for(const auto&col:v0.columns)for(auto s=col.first;s<col.first+col.count;++s){const auto&span=v0.spans[s];if(!span.component||!minimal(span.component)||touchesCut[span.component])continue;
   if(touchesRemovedSpan(col.x,col.y,span.lo,span.hi))touchesCut[span.component]=1;}
  for(std::uint32_t id=1;id<v0.components.size();++id)if(touchesCut[id])debris+=v0.components[id].cells;
  profMark("debris_scan");
  if(debris){bool copied=false; // split2 part 7: the copy started beside the derive, else made here as before
   if(earlyGrid){if(earlyGrid->worker.joinable())earlyGrid->worker.join();if(!earlyGrid->error){working=std::move(earlyGrid->copy);copied=true;}earlyGrid.reset();}
   if(!copied)working=grid;
   working.journal=nullptr;std::size_t erased=0;profMark("grid_copy");
   if(boxScan){std::vector<std::pair<std::uint32_t,std::uint32_t>> doomed; // (column, span) of every debris span; erased in the former order
    for(const auto id:minimalIds)if(touchesCut[id])spansOf(id,[&](const gfy_connectivity::Column&col,std::uint32_t s){doomed.push_back({std::uint32_t(&col-v0.columns.data()),s});return true;});
    std::sort(doomed.begin(),doomed.end());
    for(const auto&d:doomed){const auto&col=v0.columns[d.first];const auto&span=v0.spans[d.second];for(int z=span.lo;z<span.hi;++z)if(working.erase(Cell{col.x,col.y,z}))++erased;}}
   else
   for(const auto&col:v0.columns)for(auto s=col.first;s<col.first+col.count;++s){const auto&span=v0.spans[s];if(!span.component||!touchesCut[span.component])continue;
    for(int z=span.lo;z<span.hi;++z)if(working.erase(Cell{col.x,col.y,z}))++erased;}
   activeGrid=&working;profMark("debris_erase");
   static const bool rederive=std::getenv("GFY_SEP_DEBRIS_REDERIVE")!=nullptr,verifyDebris=std::getenv("GFY_SEP_DEBRIS_VERIFY")!=nullptr;
   if(rederive||verifyDebris){if(pieceDerived){pieceDerived=false;std::vector<std::uint32_t>().swap(pieceSpans);}auto spans=pristineSpans;auto filtered=withoutComponents(v0,touchesCut,derived?&spans:nullptr);
    if(derived){std::vector<std::uint32_t>().swap(pristineSpans);std::uint64_t unionLabel=0;
     volumeShared=std::make_shared<const gfy_connectivity::Topology>(gfy_connectivity::derive(*pristineShared,working,&pristineSpans,gfy_connectivity::unlimitedLimits(),&unionLabel));}
    else volumeShared=std::make_shared<const gfy_connectivity::Topology>(gfy_connectivity::build(working));
    if(verifyDebris){need(sameTopology(filtered,*volumeShared)&&filtered.cells==volumeShared->cells&&(!derived||spans==pristineSpans),"separation_debris_topology_mismatch");
     std::cerr<<"debris_verify ok components="<<filtered.components.size()<<" spans="<<filtered.spans.size()<<std::endl;}}
   else{std::vector<std::uint32_t>*const parentMap=derived?&pristineSpans:(pieceDerived?&pieceSpans:nullptr); // split2 part 7: side by side
    volumeShared=std::make_shared<const gfy_connectivity::Topology>(debrisFast?withoutComponentsSideBySide(v0,touchesCut,parentMap):withoutComponents(v0,touchesCut,parentMap));}
#ifdef GFY_COLLIDER_DIAGNOSTICS
   std::cerr<<"debris_islands cells="<<debris<<" erased="<<erased<<std::endl;
#endif
  }}
 profMark("debris_topology");
 // Debris filtering renumbers component IDs. Carry eye identity by its exact
 // surviving cell, never by the obsolete pre-filter component number.
 if(!eyeWitness.empty()){
  detachedEyes.assign(volumeShared->components.size(),0);
  for(const auto&cell:eyeWitness){const auto id=volumeShared->at(cell.x,cell.y,cell.z);if(id)detachedEyes[id]=1;}
 }
 const Grid&sgrid=*activeGrid;
 const auto&volume=*volumeShared;
 lap(separation_phase_remaining_connectivity);progress("original_connectivity");
 if(!pristineShared)pristineShared=pristineTopology(asset,input);
 const auto&pristine=*pristineShared;
 lap(separation_phase_original_connectivity);progress("compare_connectivity");
 const auto changes=derived?gfy_connectivity::compareDerived(pristine,volume,pristineSpans):
  pieceDerived?gfy_connectivity::compareDerived(*extras.piece->topology,volume,pieceSpans):
  gfy_connectivity::compare(extras.piece&&extras.piece->topology?*extras.piece->topology:pristine,volume);
 if(pieceDerived&&verifyPiece){const auto reference=gfy_connectivity::compare(*extras.piece->topology,volume);bool same=reference.size()==changes.size();
  for(std::size_t i=0;same&&i<changes.size();++i){same=changes[i].original==reference[i].original&&changes[i].pieces.size()==reference[i].pieces.size();
   for(std::size_t k=0;same&&k<changes[i].pieces.size();++k)same=changes[i].pieces[k].component==reference[i].pieces[k].component&&changes[i].pieces[k].cells==reference[i].pieces[k].cells;}
  need(same,"separation_piece_compare_mismatch");std::cerr<<"piece_compare_verify ok splits="<<changes.size()<<std::endl;}
 std::vector<std::uint32_t>().swap(pieceSpans);
#ifdef GFY_SEP_VERIFY
 if(derived){const auto reference=gfy_connectivity::compare(pristine,volume);bool same=reference.size()==changes.size();
  for(std::size_t i=0;same&&i<changes.size();++i){same=changes[i].original==reference[i].original&&changes[i].pieces.size()==reference[i].pieces.size();
   for(std::size_t k=0;same&&k<changes[i].pieces.size();++k)same=changes[i].pieces[k].component==reference[i].pieces[k].component&&changes[i].pieces[k].cells==reference[i].pieces[k].cells;}
  need(same,"separation_derived_compare_mismatch");}
#endif
 std::vector<std::uint32_t>().swap(pristineSpans);
 out->original_components=std::uint32_t(pristine.components.size()-1);
 out->remaining_components=std::uint32_t(volume.components.size()-1);
 const auto substantial=[&](std::uint64_t cells){return cells*grid.pitch*grid.pitch*grid.pitch>=minimumPiece;};
 bool separated=false;
 for(const auto&change:changes){unsigned count=0;for(const auto&p:change.pieces)if(substantial(p.cells))++count;
  out->largest_substantial_split=std::max(out->largest_substantial_split,count);if(count>=2)separated=true;}
 // gfyspeed3: the thread-tear retry keeps its result only when two of its pieces hold thread_tear_retry_m3 each (the caller's
 // count: double(cells)*pitch^3 per piece). A piece is one component of this volume or a union of them (semantic groups), cells
 // only shrink after here, and two pieces share no component - so when all components but the largest hold less than that
 // together, no second large piece can come out whatever follows: the retry ends here and its caller holds, as it would.
 if(extras.thread_tear_retry_m3>0&&separated&&gs3On()){std::uint64_t total=0,largest=0;
  for(std::uint32_t id=1;id<volume.components.size();++id){total+=volume.components[id].cells;largest=std::max<std::uint64_t>(largest,volume.components[id].cells);}
  if(double(total-largest)*grid.pitch*grid.pitch*grid.pitch<extras.thread_tear_retry_m3)throw ThreadTearHeld{};}
 lap(separation_phase_compare_connectivity);
#ifdef GFY_SEP_VERIFY_TILES
 if(tileVerdict){const auto&tv=*tileVerdict;
  if(tv.separated!=separated||tv.remaining_components!=out->remaining_components||tv.largest_substantial_split!=out->largest_substantial_split){
   std::cerr<<"GFY_SEP_VERIFY_TILES MISMATCH tile separated="<<tv.separated<<" remaining="<<tv.remaining_components<<" largest="<<tv.largest_substantial_split
            <<" span separated="<<separated<<" remaining="<<out->remaining_components<<" largest="<<out->largest_substantial_split<<std::endl;
   need(false,"tile_verdict_mismatch");}
  // Full partition: every span of the (post-debris) volume maps to one live tile component,
  // the map is a bijection between span and tile components, and the cell counts agree.
  std::vector<std::uint32_t> spanToTile(volume.components.size(),0),tileToSpan(tv.componentCells.size(),0);std::vector<std::uint64_t> cellsOf(volume.components.size(),0);
  for(const auto&c:volume.columns)for(auto s=c.first;s<c.first+c.count;++s){const auto&span=volume.spans[s];const auto tc=tileLabels.at(c.x,c.y,span.lo);
   need(tc&&!tv.componentDebris[tc],"tile_partition_cell_unlabeled");
   need((!spanToTile[span.component]||spanToTile[span.component]==tc)&&(!tileToSpan[tc]||tileToSpan[tc]==span.component),"tile_partition_mismatch");
   spanToTile[span.component]=tc;tileToSpan[tc]=span.component;cellsOf[span.component]+=std::uint64_t(span.hi-span.lo);}
  for(std::uint32_t c=1;c<volume.components.size();++c)need(spanToTile[c]&&cellsOf[c]==tv.componentCells[spanToTile[c]]&&cellsOf[c]==volume.components[c].cells,"tile_partition_cells");
  std::size_t live=0;for(std::uint32_t c=1;c<tv.componentCells.size();++c)live+=!tv.componentDebris[c];need(live+1==volume.components.size(),"tile_partition_count");
  std::cerr<<"GFY_SEP_VERIFY_TILES identical separated="<<separated<<" remaining="<<out->remaining_components<<" largest="<<out->largest_substantial_split<<" components="<<live<<" debris="<<tv.debris_components<<std::endl;}
#endif
 if(pieceVerdict&&pieceTileVerify){const auto&pv=*pieceVerdict; // split: both paths ran; every field must agree
  if(pv.usable)need(pv.separated==separated&&pv.remaining_components==out->remaining_components&&pv.largest_substantial_split==out->largest_substantial_split,"piece_tile_verdict_mismatch");
  std::cerr<<"piece_tile_verify "<<(pv.usable?"ok":"unusable")<<" refusal="<<pv.refusal<<" separated="<<separated<<" remaining="<<out->remaining_components
   <<" largest="<<out->largest_substantial_split<<" components="<<pv.components<<" debris="<<pv.debris_components<<std::endl;}
 if(!separated){out->no_separation=true;return out;}
#ifdef GFY_COLLIDER_DIAGNOSTICS
 phase("connectivity");
#endif
 const auto groupedVolume=extras.semantic_groups?std::make_shared<const gfy_connectivity::Topology>(semanticIslandVolume(volume,*extras.semantic_groups)):std::shared_ptr<const gfy_connectivity::Topology>();
 const auto&pieceVolume=groupedVolume?*groupedVolume:volume;
 progress("surface_bindings");
 const auto decoded=decodedInputs(asset,input,pristineShared);
 const auto&bindings=decoded->bindings;
 // Folded physical ownership of a piece lineage (PieceSeed::folded): the folded elements' cells and bone bindings are
 // the adopters' for this separation, on private copies; the shared caches stay untouched. No folds: the shared objects.
 std::set<std::uint16_t> adopters;const bool folds=extras.piece&&anyFolded(extras.piece->folded);
 const auto framesFolded=folds?std::make_shared<const gfy_piece_collider::original_frames::Frames>(foldedFrames(decoded->frames,extras.piece->folded)):std::shared_ptr<const gfy_piece_collider::original_frames::Frames>();
 const auto&frames=folds?*framesFolded:decoded->frames;
 lap(separation_phase_surface_bindings);progress("original_physics_map");
 const auto mapShared=physicsMap(asset,pristineShared,input,decoded->frames);
 const auto map=folds?foldedMap(*mapShared,extras.piece->folded,adopters):mapShared;
 lap(separation_phase_original_physics_map);progress("physical_partitions");
 const auto bindingFolded=folds?std::make_shared<const std::vector<std::uint16_t>>(foldedBindings(decoded->bindingElements,extras.piece->folded)):std::shared_ptr<const std::vector<std::uint16_t>>();
 const auto&bindingElements=folds?*bindingFolded:decoded->bindingElements;
 const sep::SourceFaceIndex&sourceFaces=*decoded->sourceFaces;
 // P1.6: remainingCounts() and intersect() of every substantial component in
 // one pass on <= 4 threads (exact; any failure re-runs the originals below,
 // which fail with their own message in their own order).
 // R54: an island that was already separate in the reference anatomy (a KVMA / original GFY thumb block, 17 cm3, bone
 // inside) keeps its authored attachment like a pre-existing eye: it is a piece only when a split produced it. It used
 // to leave as its own hollow piece on EVERY separation (any limb or the neck). Its skin follows its bone's piece.
 std::vector<char> splitPiece(volume.components.size(),0);
 for(const auto&change:changes)if(change.pieces.size()>1)for(const auto&p:change.pieces)if(p.component<splitPiece.size())splitPiece[p.component]=1;
 std::vector<std::uint32_t>substantialIds;
 for(std::uint32_t id=1;id<pieceVolume.components.size();++id)if(substantial(pieceVolume.components[id].cells)&&(groupedVolume||splitPiece[id]))substantialIds.push_back(id);
 std::map<std::uint16_t,std::uint64_t>remainingByElement;std::map<std::uint32_t,std::vector<pc::element_partition::Element>>fusedParts;
 const bool fusedPartition=pc::element_partition::partitionFused(*map,pieceVolume,substantialIds,remainingByElement,fusedParts);
 if(!fusedPartition)remainingByElement=pc::element_partition::remainingCounts(*map,pieceVolume);
#ifdef GFY_SEP_VERIFY
 if(fusedPartition){need(remainingByElement==pc::element_partition::remainingCounts(*map,pieceVolume),"separation_fused_remaining_mismatch");
  for(const auto id:substantialIds){const auto reference=pc::element_partition::intersect(*map,pieceVolume,id);const auto&fused=fusedParts.at(id);
   bool same=reference.size()==fused.size();
   for(std::size_t i=0;same&&i<fused.size();++i)same=reference[i].bone==fused[i].bone&&reference[i].original_shape_complete==fused[i].original_shape_complete&&
    sameTopology(reference[i].volume,fused[i].volume)&&reference[i].volume.work==fused[i].volume.work;
   need(same,"separation_fused_partition_mismatch");}}
#endif
 std::map<std::uint16_t,unsigned>pieceOwners;
 bound_surface::GarmentAdmission garment;
 for(std::size_t b=0;b<frames.bones.size();++b)garment.bone_to_element[b]=frames.bones[b].physical_element<0?65535:std::uint16_t(frames.bones[b].physical_element);
 // split2 part 13: the coarse parts of this prepare()'s collider fits (before the part volumes: it outlives every fit)
 static const bool keepCoarse=[]{const char*e=std::getenv("GFY_SEP_COARSE_KEEP");return !e||std::atoi(e)!=0;}();
 CoarsePartCache coarseParts;CoarsePartCache*const coarseCache=keepCoarse?&coarseParts:nullptr;
 std::map<std::uint32_t,std::vector<pc::element_partition::Element>> partitions;
 for(const auto id:substantialIds){
  progress("physical_partition",id);
  auto parts=fusedPartition?std::move(fusedParts.at(id)):pc::element_partition::intersect(*map,pieceVolume,id);
  if(!adopters.empty())for(auto&p:parts)if(adopters.count(p.bone))p.original_shape_complete=false; // folded remnant inside: refit
  for(const auto&p:parts){garment.component_elements[id].insert(p.bone);++pieceOwners[p.bone];}
  partitions.emplace(id,std::move(parts));
 }
 std::uint32_t bodyComponent=0;
 {std::uint64_t most=0;for(const auto id:substantialIds)if((!wantedComponents||wantedComponents->count(id))&&pieceVolume.components[id].cells>most){most=pieceVolume.components[id].cells;bodyComponent=id;}}
 const std::uint64_t bodyFitted=extras.piece?extras.piece->fitted_elements:0;
 const auto keepsWhole=[&](std::uint32_t component,std::uint16_t bone,std::size_t parts){
  // stumpshell: every piece, not only the remaining body (a severed limb keeps its native shapes, also a single element)
  if(bone>=64||((bodyFitted>>bone)&1u))return false;
  const auto o=pieceOwners.find(bone);return o!=pieceOwners.end()&&o->second==1;};
 // R94: a forest component - a divided element's gram-scale remnant whose parent left with another piece (the jaw crumb
 // a neck cut leaves on the torso while head 15 goes with the head piece). admitNativeShell step 1 folds such a remnant
 // into its nearest retained ancestor, but only after the low-poly surfaces were owned and posed. Until then the remnant's
 // bone is a second retained neighbour of every missing head/neck influence (cutSkinAnchor and regionSkinFrontier see
 // {12,19}), so the chest skin round the lower neck could not be posed on the torso: left off (an open collar under a
 // floating cap) or hosted on the head (strips), and root() refused the rest (separation_multiple_physical_roots).
 // The same fold is decided here with the same rule - single-element extra root, natively inadmissible fit, the rest one
 // physical tree, nearest retained ancestor - and the remnant leaves the element set that skin ownership, pose anchors
 // and root() use. Its part stays: it is fitted and folded by admitNativeShell exactly as before. One-tree components
 // are untouched.
 std::map<std::uint32_t,std::map<std::uint16_t,std::uint16_t>> preFolded; // component -> remnant element -> adopter
 for(const auto id:substantialIds){auto&E=garment.component_elements[id];if(E.size()<2)continue;
  const auto&parents=input.physical_parents;
  const auto under=[&](std::uint16_t b,std::uint16_t r){for(std::size_t s=0;s<=parents.size()&&b<parents.size();++s){if(b==r)return true;b=parents[b];}return false;};
  std::vector<std::uint16_t> roots;for(auto b:E){need(b<parents.size(),"separation_physical_parent_count");if(!E.count(parents[b]))roots.push_back(b);}
  if(roots.size()<2)continue;
  const auto&parts=partitions.at(id);
  std::uint16_t body=roots[0];std::uint64_t most=0;
  for(auto r:roots){std::uint64_t n=0;for(const auto&p:parts)if(under(p.bone,r))n+=p.volume.cells;if(n>most){most=n;body=r;}}
  std::set<std::uint16_t> rest=E;bool fold=true;std::map<std::uint16_t,std::uint16_t> adopted;
  for(auto r:roots){if(r==body)continue;
   bool single=true;for(auto b:E)if(b!=r&&under(b,r))single=false;
   std::uint16_t into=65535;
   if(single){std::uint16_t p=parents[r];for(std::size_t s=0;s<parents.size()&&p<parents.size();++s,p=parents[p])if(rest.count(p)){into=p;break;}}
   const auto part=std::find_if(parts.begin(),parts.end(),[&](const auto&q){return q.bone==r;});
   if(!single||into==65535||part==parts.end()||!(r<frames.bones.size()&&frames.bones[r].physical_element==r)){fold=false;break;}
   const auto o=pieceOwners.find(r);const auto rem=remainingByElement.find(r);const auto c=map->original_cells.find(r);
   if((o!=pieceOwners.end()&&o->second==1&&rem!=remainingByElement.end()&&c!=map->original_cells.end()&&rem->second==c->second&&!adopters.count(r))||
      keepsWhole(id,r,parts.size())){fold=false;break;} // whole: keeps its original shape (admissible); bodyjitter3: or kept as it is
   SeparationElement e;e.bone=r;
   pc::Capture capture;capture.accepted=stamp;capture.component=1;capture.anchor_bone=r;
   capture.rest_to_anchor=frames.bones[r].rest_to_anchor;capture.anchor_to_world=frames.bones[r].anchor_to_rest;capture.rigid_pose_certified=true;
   try{pc::Prepared fitted;
    try{fitted=fitCollider(part->volume,capture,{},coarseCache);}
    catch(const std::bad_alloc&){throw;}
    catch(const std::exception&x){if(std::string(x.what()).find("collider_")==std::string::npos)throw;fitted=pc::prepareTiny(part->volume,capture);}
    e.mass_kg=fitted.mass_kg;e.com={fitted.com_anchor[0],fitted.com_anchor[1],fitted.com_anchor[2]};e.inertia=fitted.inertia_anchor_com;
    for(const auto&box:fitted.boxes)e.boxes.push_back({{box.center_anchor[0],box.center_anchor[1],box.center_anchor[2]},
      {box.half_extents_m[0],box.half_extents_m[1],box.half_extents_m[2]},box.orientation});}
   catch(const std::bad_alloc&){throw;}
   catch(const std::exception&x){ // cancellation propagates; any other failure: the loop below fails the same way, as before
    if(std::strcmp(x.what(),"separation_canceled")==0)throw;sep::throwIfCanceled();fold=false;break;}
   if(nativeElementAdmissible(e)){fold=false;break;}
   rest.erase(r);adopted[r]=into;}
  if(!fold)continue;
  std::size_t restRoots=0;for(auto b:rest)restRoots+=!rest.count(parents[b]);
  if(restRoots==1){E=std::move(rest);preFolded[id]=std::move(adopted);}
 }
 const bound_surface::ExtraSurface* accessorySurface=decoded->accessorySurface.get();
#ifdef GFY_COLLIDER_DIAGNOSTICS
 phase("bindings_and_partitions");
#endif
 lap(separation_phase_physical_partitions);progress("surface_partition");
 bound_surface::SurfaceOptions surfaceOptions;surfaceOptions.wanted=wantedComponents;surfaceOptions.published=extras.published;
 surfaceOptions.source_faces=&sourceFaces;surfaceOptions.cancel=sep::cancelFlag();
 if(checkpoint)surfaceOptions.checkpoint=[&](const char*name){progress(name);};
 (void)surfaceOptions;
 need(lowpoly_pieces::available(extras.lowpoly)||(extras.lowpoly&&extras.lowpoly->mask_volume&&!extras.lowpoly->analytic_channels),"separation_requires_lowpoly_geometry");
 bound_surface::Packed surfaces;std::vector<std::vector<std::pair<Cell,core::Tile>>> pieceMarks;
 std::map<std::pair<std::uint32_t,std::uint16_t>,std::array<double,4>> bodyCut; // (component, bone) -> keep n.x <= d (bone frame)
 for(const auto&bp:partitions){const auto component=bp.first;
  if(wantedComponents&&!wantedComponents->count(component))continue;
  const auto pf=preFolded.find(component); // stumpshell: single-element pieces too
  for(const auto&part:bp.second){const auto bone=part.bone;
   if(bone>=64||((bodyFitted>>bone)&1u)||!(bone<frames.bones.size()&&frames.bones[bone].physical_element==bone))continue;
   const auto o=pieceOwners.find(bone);if(o==pieceOwners.end()||o->second<2)continue;
   if(pf!=preFolded.end()&&pf->second.count(bone))continue;
   if(double(part.volume.cells)*part.volume.pitch*part.volume.pitch*part.volume.pitch<150e-6)continue; // a crumb: fitted, folded
   if(component!=bodyComponent){const auto all=remainingByElement.find(bone);if(all==remainingByElement.end()||4*part.volume.cells<all->second)continue;}
   if(const auto plane=bodyCutPlane(part.volume,partitions,component,bone,frames.bones[bone].rest_to_anchor))bodyCut[{component,bone}]=*plane;}}
 const auto keepsShape=[&](std::uint32_t component,std::uint16_t bone,std::size_t parts){
  return keepsWhole(component,bone,parts)||bodyCut.count({component,bone})!=0;}; // stumpshell: any piece, any element count
 struct Prefit{std::uint32_t component;std::uint16_t bone;const gfy_connectivity::Topology*volume;pc::Capture capture;pc::Prepared fitted;std::exception_ptr error;double ms{};};
 // The fits the piece loop asks for, in its order, for these piece components. Selection mirrors the loop; anything
 // whose selection would throw is left to the serial path, which throws the same error first.
 const auto selectPrefits=[&](const std::vector<std::uint32_t>&pieceComponents){std::vector<Prefit> list;
  for(const auto component:pieceComponents){
   const auto p=partitions.find(component);const auto e=garment.component_elements.find(component);
   if(p==partitions.end()||e==garment.component_elements.end())break;
   bool stop=false;
   for(const auto&part:p->second){
    if(!(part.bone<frames.bones.size()&&frames.bones[part.bone].physical_element==part.bone)){stop=true;break;}
    const auto owners=pieceOwners.find(part.bone);const auto remaining=remainingByElement.find(part.bone);const auto original=map->original_cells.find(part.bone);
    // R94: physical shape policy counts the component's parts (the element set before the early remnant fold), as before.
    if(p->second.size()>1&&(owners==pieceOwners.end()||remaining==remainingByElement.end()||original==map->original_cells.end())){stop=true;break;}
    const bool retain=(p->second.size()>1&&owners->second==1&&remaining->second==original->second&&!adopters.count(part.bone))||
                      keepsShape(component,part.bone,p->second.size()); // bodyjitter3: the piece loop's rule
    if(retain)continue;
    Prefit f;f.component=component;f.bone=part.bone;f.volume=&part.volume;
    f.capture.accepted=stamp;f.capture.component=1;f.capture.anchor_bone=part.bone;
    f.capture.rest_to_anchor=frames.bones[part.bone].rest_to_anchor;f.capture.anchor_to_world=frames.bones[part.bone].anchor_to_rest;
    f.capture.rigid_pose_certified=true;list.push_back(std::move(f));}
   if(stop)break;}
  return list;};
 static const bool outerFits=[]{const char*e=std::getenv("GFY_SEP_COLLIDER_OUTER");return !e||std::atoi(e)!=0;}();
 // One element far larger than every other (a torso cut: measured faster with the fitter's own fan-out
 // than beside serial small fits) is fitted by the loop with that fan-out. R69: the other elements are still fitted side by
 // side first (a head shot's skull fragments were fitted one after another), larger elements first. Returns the fit order.
 const auto planPrefits=[&](std::vector<Prefit>&list){
  {std::uint64_t largest=0,second=0;std::size_t largestAt=0;
   for(std::size_t k=0;k<list.size();++k){const auto cells=list[k].volume->cells;if(cells>largest){second=largest;largest=cells;largestAt=k;}else if(cells>second)second=cells;}
   if(!outerFits||list.size()<2)list.clear();
   else if(largest>4*second){list.erase(list.begin()+std::ptrdiff_t(largestAt));if(list.size()<2)list.clear();}}
  std::vector<std::size_t> order(list.size());for(std::size_t k=0;k<order.size();++k)order[k]=k;
  std::stable_sort(order.begin(),order.end(),[&](std::size_t a,std::size_t b){return list[a].volume->cells>list[b].volume->cells;});
  return order;};
 // The side-by-side subset of a fit list by that rule (indices into the list, larger elements first): the fits of
 // planPrefits, in its order. The rest of the list is the piece loop's own (fitted with the fitter's fan-out).
 const auto sideBySide=[&](const std::vector<Prefit>&list){std::vector<std::size_t> side(list.size());for(std::size_t k=0;k<side.size();++k)side[k]=k;
  std::uint64_t largest=0,second=0;std::size_t largestAt=0;
  for(std::size_t k=0;k<list.size();++k){const auto cells=list[k].volume->cells;if(cells>largest){second=largest;largest=cells;largestAt=k;}else if(cells>second)second=cells;}
  if(!outerFits||side.size()<2)side.clear();
  else if(largest>4*second){side.erase(side.begin()+std::ptrdiff_t(largestAt));if(side.size()<2)side.clear();}
  std::stable_sort(side.begin(),side.end(),[&](std::size_t a,std::size_t b){return list[a].volume->cells>list[b].volume->cells;});
  return side;};
 // prefits: every fit of the piece loop, in loop order. order: the side-by-side ones (each under the nesting token and
 // SerialScope of the former sep::parallel chunk). tail: the loop's own fits (the one far larger element, a single fit):
 // they run after the side-by-side ones, on the first helper, one after another in loop order with the fitter's fan-out,
 // while no other thread is fitting - what the loop did, but beside the surface partition. Collider work never uses more
 // than max_job_threads threads: `fitting` counts the threads inside the side-by-side fits, and the tail waits for zero.
 struct EarlyFits {std::vector<Prefit> prefits;std::vector<std::size_t> order,tail;std::atomic<std::size_t> cursor{0},tailCursor{0};std::atomic<bool> stop{false};
  std::mutex mutex;std::condition_variable idle;unsigned fitting{0};bool jobHelps{true};std::vector<std::thread> workers;
  CoarsePartCache*coarse{}; // split2 part 13: prepare()'s coarse parts (null: every fit builds its own)
  // speedfix: crew mode (GFY_SF_CREW, default on): every fit is in `order` (larger first, no tail) and every thread that has
  // no fit left helps the running ones (PieceCollider.h Crew) until the last one is done. slot: the thread's board slot
  // (helpers 0..2, the job's thread 3).
  bool crewMode{false};gfy_piece_collider::detail::Crew crew;
  // speedfix: a fit with more than 64 boxes may be refitted for the shell budget (shellBudgetRefit); its attempt under the
  // coarse limits with a share of 64 or more gives the same leaves as the same limits with 96 boxes (the greedy cap and the
  // merge target are min(64, max_boxes) either way) and only its box count decides the certificate. Idle crew members fit
  // that once, while other fits still run; stopped (cooperative cancel points) when the last fit is done.
  // coarseState: 0 none, 1 queued, 2 running, 3 done, 4 failed, 5 stopped. GFY_SF_COARSE_AHEAD=0: off.
  std::vector<std::atomic<int>> coarseState;std::vector<gfy_piece_collider::Prepared> coarseFit;std::vector<std::exception_ptr> coarseError;
  std::atomic<bool> coarseStop{false};
  static gfy_piece_collider::Limits coarseLimits(){gfy_piece_collider::Limits l;l.max_corner_gap_m=.05;l.max_omitted_distance_m=.05;l.max_proxy_ratio=6.;l.max_box_ratio=12.;return l;}
  static bool coarseTask(void*context){auto*e=static_cast<EarlyFits*>(context);
   for(std::size_t k=0;k<e->coarseState.size();++k){int queued=1;if(!e->coarseState[k].compare_exchange_strong(queued,2))continue;
    auto&f=e->prefits[k];int state=3;
    {const gfy_piece_collider::detail::SerialScope serial;const gfy_piece_collider::detail::SpecStopScope stopScope(&e->coarseStop);
     try{e->coarseFit[k]=fitCollider(*f.volume,f.capture,coarseLimits(),e->coarse);}
     catch(...){state=e->coarseStop.load()?5:4;if(state==4)e->coarseError[k]=std::current_exception();}}
    e->coarseState[k].store(state,std::memory_order_release);return true;}
   return false;}
  // Every side-by-side fit exactly once: each thread takes the next one in the order until none is left (or the job gave up).
  void work(unsigned slot=0){const sep::NestedScope token;
   const gfy_piece_collider::detail::CrewScope member(crewMode?&crew:nullptr,slot);
   for(std::size_t k;!stop.load(std::memory_order_relaxed)&&(k=cursor.fetch_add(1))<order.size();){auto&f=prefits[order[k]];
    {const gfy_piece_collider::detail::SerialScope serial;const auto t=core::Clock::now();
     try{f.fitted=fitCollider(*f.volume,f.capture,{},coarse);}catch(...){f.error=std::current_exception();}f.ms=core::ms(t);}
    if(crewMode){static const bool ahead=[]{const char*v=std::getenv("GFY_SF_COARSE_AHEAD");return !v||std::atoi(v)!=0;}();
     if(ahead&&!f.error&&f.fitted.boxes.size()>64&&order[k]<coarseState.size())coarseState[order[k]].store(1,std::memory_order_release);
     if(crew.fitting.fetch_sub(1,std::memory_order_acq_rel)==1)coarseStop.store(true);}}
   if(crewMode){crew.idle(slot);gfy_piece_collider::detail::releaseCrewScratch();}}
  void tailWork(){for(std::size_t k;!stop.load(std::memory_order_relaxed)&&(k=tailCursor.fetch_add(1))<tail.size();){auto&f=prefits[tail[k]];const auto t=core::Clock::now();
    try{f.fitted=fitCollider(*f.volume,f.capture,{},coarse);}catch(...){f.error=std::current_exception();}f.ms=core::ms(t);}}
  void enter(){std::lock_guard<std::mutex> lock(mutex);++fitting;}
  void leave(){{std::lock_guard<std::mutex> lock(mutex);--fitting;}idle.notify_all();}
  // A helper (entered by the job's thread before it starts): its share of the side-by-side fits; the first helper then
  // runs the tail once no thread is inside a side-by-side fit (none is left to take by then).
  void helper(bool first,unsigned slot){work(slot);leave();if(!first||crewMode)return;
   {std::unique_lock<std::mutex> lock(mutex);idle.wait(lock,[&]{return fitting==0;});}
   tailWork();}
  // The job's thread, when the surface partition is done: takes part in the side-by-side fits (if the helpers are fewer
  // than the budget), waits for the helpers, and runs what is left of the tail itself (no first helper could be started).
  void finish(){if(jobHelps){enter();work(gfy_piece_collider::detail::Crew::slots-1);leave();}join();tailWork();}
  void join(){for(auto&w:workers)if(w.joinable())w.join();}
  ~EarlyFits(){stop.store(true);crew.stop.store(true);coarseStop.store(true);join();}};
 std::unique_ptr<EarlyFits> earlyFits;
 static const bool earlyFitsOn=[]{const char*e=std::getenv("GFY_SEP_EARLY_FITS");return !e||std::atoi(e)!=0;}();
 // Helper threads of the side-by-side fits beside the surface partition: the collider crew's members (Crew::slots, 4) or the
 // job's cap when smaller, less the job's thread (3 at a cap of 4 or more; the job's thread, slot Crew::slots-1, is the last when it
 // gets there). GFY_SEP_EARLY_FIT_THREADS=1..that for measurements (all of them: the job's thread only waits).
 static const int earlyThreadsSet=[]{const char*e=std::getenv("GFY_SEP_EARLY_FIT_THREADS");return e?std::atoi(e):0;}(); // gearpar
 const unsigned crewCap=std::min<unsigned>(sep::jobThreads(),gfy_piece_collider::detail::Crew::slots);
 const std::size_t earlyThreads=std::size_t(earlyThreadsSet>=1&&earlyThreadsSet<=int(crewCap)?earlyThreadsSet:int(crewCap)-1);
 if(earlyFitsOn){
  std::vector<std::uint32_t> pieceComponents;for(const auto id:substantialIds)if(!wantedComponents||wantedComponents->count(id))pieceComponents.push_back(id);
  auto list=selectPrefits(pieceComponents);
  if(!list.empty()){std::size_t helpers=0;
   try{auto early=std::make_unique<EarlyFits>();early->coarse=coarseCache;
    static const bool crewFits=[]{const char*e=std::getenv("GFY_SF_CREW");return !e||std::atoi(e)!=0;}(); // speedfix
    if(crewFits){early->crewMode=true;early->order.resize(list.size());for(std::size_t k=0;k<list.size();++k)early->order[k]=k;
     std::stable_sort(early->order.begin(),early->order.end(),[&](std::size_t a,std::size_t b){return list[a].volume->cells>list[b].volume->cells;});
     early->crew.fitting.store(int(list.size()));}
    else{early->order=sideBySide(list);
    {std::vector<char> side(list.size(),0);for(const auto k:early->order)side[k]=1;for(std::size_t k=0;k<list.size();++k)if(!side[k])early->tail.push_back(k);}}
    early->prefits=std::move(list);
    if(early->crewMode){early->coarseState=std::vector<std::atomic<int>>(early->prefits.size());early->coarseFit.resize(early->prefits.size());
     early->coarseError.resize(early->prefits.size());early->crew.extra=&EarlyFits::coarseTask;early->crew.extraCtx=early.get();}
    helpers=early->crewMode?earlyThreads:std::max<std::size_t>(early->tail.empty()?0:1,std::min<std::size_t>(early->order.size(),earlyThreads));early->workers.reserve(helpers);
    earlyFits=std::move(early);}
   catch(const std::bad_alloc&){earlyFits.reset();} // no early fits: the piece loop's own path fits (and reports) as before
   if(earlyFits){const auto cancel=sep::cancelFlag();auto*e=earlyFits.get();
    // A helper that cannot be started is simply missing: the remaining threads and the job's own thread do its fits.
    try{for(std::size_t h=0;h<helpers;++h){e->enter();
      try{e->workers.emplace_back([e,cancel,h]{core::enterPainterThread();const sep::CancelScope scope(cancel);e->helper(h==0,unsigned(h));});}
      catch(...){e->leave();throw;}}}
    catch(...){}
    e->jobHelps=e->workers.size()<crewCap;}}}
 // split2 part 6: the caller's work that needs only the piece volume starts now too (prepareSnapshot: the owner spans and
 // topology of every piece's seed). The volume object and the piece components are final from here on.
 if(extras.piece_volume_ready&&!groupedVolume&&!wantedComponents){try{extras.piece_volume_ready(volumeShared,substantialIds);}catch(...){}}
 {std::vector<std::uint32_t> ids;for(const auto id:substantialIds)if(!wantedComponents||wantedComponents->count(id))ids.push_back(id);
  std::set<std::uint16_t> whole;for(const auto&a:input.accessories)whole.insert(a.child);
  // Published body skin has no source-excluded exterior attachments on the
  // first cut. On later cuts their original fixed anchors, not skin proximity,
  // still decide ownership. Keep both cases out of the body classifier.
  std::set<std::pair<std::uint16_t,std::uint32_t>> attachedFaces;
  if(accessorySurface){
   const std::set<std::uint16_t> body(asset.face_children.begin(),asset.face_children.end());
   for(std::size_t f=0;f<accessorySurface->face_children.size();++f){
    const auto child=accessorySurface->face_children[f];
    attachedFaces.emplace(child,accessorySurface->face_indices[f]);
    if(!body.count(child))whole.insert(child);
   }
  }
  auto anchorable=[&](std::size_t k,std::uint16_t bone){
   if(bone>=frames.bones.size()||frames.bones[bone].physical_element<0)return false;
   const auto e=garment.component_elements.find(ids[k]);if(e==garment.component_elements.end())return false;
   try{(void)cutSkinAnchor(std::uint16_t(frames.bones[bone].physical_element),e->second,input.physical_parents);return true;}
   catch(const std::bad_alloc&){throw;}catch(const std::exception&){return false;}};
  auto regionFrontier=[&](std::size_t k,std::uint16_t bone){
   if(bone>=frames.bones.size()||frames.bones[bone].physical_element<0)return std::uint16_t(65535);
   const auto e=garment.component_elements.find(ids[k]);if(e==garment.component_elements.end())return std::uint16_t(65535);
   return regionSkinFrontier(std::uint16_t(frames.bones[bone].physical_element),e->second,input.physical_parents);};
  // Immutable anatomical attachments decide ambiguous skin regions. Skin
  // weights control motion, not ownership: a hood can follow the head while
  // its lower corners remain attached to the shoulders. Only matched body
  // source faces use this certificate; existing coat/accessory rules remain.
  const std::set<std::uint32_t> bindingRetained(ids.begin(),ids.end());
  const auto partitionCertificate=bound_surface::buildPartitionCertificate(sgrid,pieceVolume,bindingRetained);
  // R62: torn tissue ends (the published channel field's mode-4 records: a cylinder about each tear plane).
  struct TornEnd {Vec c,n;double R,H;};std::vector<TornEnd> tornEnds;
  if(extras.lowpoly&&extras.lowpoly->analytic_channels){const auto&F=*extras.lowpoly->analytic_channels;
   for(unsigned i=0;i<F.cuts;++i){const auto&md=F.words[1+std::size_t(i)*9+6];if(md[3]!=4.f)continue;
    const auto&er=F.words[1+std::size_t(i)*9];const auto&dp=F.words[1+std::size_t(i)*9+1];TornEnd e;e.H=.5*double(dp[3]);e.R=er[3];
    e.n=Vec{dp[0],dp[1],dp[2]};e.c=Vec{er[0],er[1],er[2]}+e.n*e.H;tornEnds.push_back(e);}}
  auto tornAnchor=[&](const Vec&a){for(const auto&e:tornEnds){const Vec d=a-e.c;const double s=dot(d,e.n);if(std::abs(s)>e.H||dot(d,d)-s*s>e.R*e.R)continue;
    const Cell c{int(std::floor((a.x-sgrid.origin.x)/sgrid.pitch)),int(std::floor((a.y-sgrid.origin.y)/sgrid.pitch)),int(std::floor((a.z-sgrid.origin.z)/sgrid.pitch))};
    if(sgrid.valid(c)&&sgrid.removed(c))return true;}return false;};
  auto bindingRegions=[&](const gfy_packed::Child&child,std::size_t triangle,
                         std::vector<std::pair<std::size_t,core::Poly>>&regions)->bool{
   regions.clear();
   if(child.material||child.indices.size()%3||child.original_faces.size()!=child.indices.size()/3||triangle>=child.original_faces.size())return false;
   const auto sourceFace=sourceFaces.find(child.material_source_child,child.original_faces[triangle]);
   if(!sourceFace||*sourceFace>=asset.source.f.size())return false;
   const auto face=*sourceFace;
   if(!bindings.coat_faces.empty()&&bindings.coat_faces[face])return false;
   std::array<Vec,3>anchors{};core::JoinedTriangle surface;
   for(unsigned q=0;q<3;++q){
    const auto index=child.indices[triangle*3+q];if(index>=child.vertices.size())return false;
    const auto&row=child.vertices[index];std::array<double,3>bary;
    try{bary=original_accessory::barycentric(asset.source,face,row);}
    catch(const std::bad_alloc&){throw;}
    catch(const std::runtime_error&){return false;}
    for(unsigned j=0;j<3;++j){const auto vertex=asset.source.f[face][j];if(vertex>=bindings.vertices.size())return false;
     anchors[q]=anchors[q]+bindings.vertices[vertex].point*bary[j];}
    // The partition carries barycentrics in THIS published triangle. The
    // original source barycentrics above are used only to recover attachments.
    std::array<double,3>local{};local[q]=1.;surface.v[q]=original_accessory::unpack(row,face,local);
   }
   if(!tornEnds.empty())for(unsigned q=0;q<3;++q)if(tornAnchor(anchors[q]))return false; // R62: attached to torn-away tissue
   bound_surface::Result partition;
   bound_surface::partitionTriangle(surface,anchors,sgrid,pieceVolume,bindingRetained,partition,partitionCertificate.get());
   for(auto&entry:partition.meshes){
    const auto owner=std::find(ids.begin(),ids.end(),entry.first);
    need(owner!=ids.end(),"skin_binding_owner_range");std::vector<core::Poly>polygons;
    polygons.reserve(entry.second.triangles.size());
    for(const auto&t:entry.second.triangles){core::Poly poly;
     for(const auto&corner:t.v){core::V v;v.p=corner.p;v.b=corner.sourceBary;poly.push_back(v);}
     polygons.push_back(std::move(poly));}
    // Merge exact neighbouring pieces to avoid paying for voxel subdivisions
    // in the renderer. Vertices and skin weights are never repacked here.
    thread_local bound_surface::MergeScratch mergeScratch; // R66: mergeConvexFast = the same merges in the same order
    for(auto&poly:bound_surface::mergeConvexFast(std::move(polygons),mergeScratch))
     regions.emplace_back(std::size_t(owner-ids.begin()),std::move(poly));
   }
   // Empty is authoritative too: these attachment regions were removed.
   // Inherited piece predicates are intersected by the receiving low-poly path.
   return true;
  };
  auto boneOwner=[&](std::uint16_t bone)->int{
   if(bone>=frames.bones.size()||frames.bones[bone].physical_element<0)return -1;const auto e=std::uint16_t(frames.bones[bone].physical_element);
   const auto n=pieceOwners.find(e);
   if(n!=pieceOwners.end()&&n->second==1){
    for(std::size_t k=0;k<ids.size();++k){const auto c=garment.component_elements.find(ids[k]);if(c!=garment.component_elements.end()&&c->second.count(e))return int(k);}
    return -1;}
   if(bone==e)return -1; // skin of the split element's own bone: the tissue under it decides
   // A bone inside a split element (hand/finger bones of a severed forearm): the nearest remaining tissue to its joint.
   const auto&m=frames.bones[bone].anchor_to_rest.row_major;const Vec o{m[3],m[7],m[11]};
   const Cell c0{int(std::floor((o.x-sgrid.origin.x)/sgrid.pitch)),int(std::floor((o.y-sgrid.origin.y)/sgrid.pitch)),int(std::floor((o.z-sgrid.origin.z)/sgrid.pitch))};
   int found=-1;int best=1<<30;const int R=12;
   for(int dx=-R;dx<=R;++dx)for(int dy=-R;dy<=R;++dy)for(int dz=-R;dz<=R;++dz){const int d2=dx*dx+dy*dy+dz*dz;if(d2>=best)continue;
    const Cell c{c0.x+dx,c0.y+dy,c0.z+dz};if(!sgrid.valid(c))continue;const auto comp=pieceVolume.at(c.x,c.y,c.z);if(!comp)continue;
    for(std::size_t k=0;k<ids.size();++k)if(ids[k]==comp){best=d2;found=int(k);}}
   return found;};
  // Centre of each piece's part of every physical element (a split element has one part per side of the break).
  std::map<std::pair<std::uint32_t,std::uint16_t>,Vec> partCentre;
  for(const auto id:ids){const auto pt=partitions.find(id);if(pt==partitions.end())continue;
   for(const auto&part:pt->second){const auto&tv=part.volume;double sx=0,sy=0,sz=0,count=0;
    for(const auto&c:tv.columns)for(auto s=c.first;s<c.first+c.count;++s){const auto&sp=tv.spans[s];const double m=double(sp.hi-sp.lo);sx+=m*c.x;sy+=m*c.y;sz+=m*.5*(sp.lo+sp.hi-1);count+=m;}
    if(count>0)partCentre[{id,part.bone}]=Vec{tv.origin[0]+(sx/count+.5)*tv.pitch,tv.origin[1]+(sy/count+.5)*tv.pitch,tv.origin[2]+(sz/count+.5)*tv.pitch};}}
  // Exact nearest remaining tissue of each piece's part of one physical element (this separation's partition, the
  // part volume's own origin/pitch): skin with no tissue under it on a split element goes to the side whose tissue is
  // nearest, not to the side whose centroid is nearest (a hip chunk's centroid sits under a pouch whose thigh is 10 cm
  // away; the 26-ray search has already failed before this point; in-game KVMA grenade: pouch flaps standing in the
  // air on gram-scale chunks). Columns bucketed on a 1 cm xy lattice once; a query walks bucket rings outward and
  // stops when the ring's lower bound exceeds the best distance. Ties: ids order.
  struct PartIndex{const gfy_connectivity::Topology*vol;double ox,oy,oz,pitch;int C,F,cx0,cy0,cnx,cny;int lo[3],hi[3];
   std::vector<std::uint32_t> order,start,fineStart,occupiedLeaves;std::vector<int> zlo,zhi,fzlo,fzhi;};
  auto floorDiv=[](int a,int b){return a>=0?a/b:-((-a+b-1)/b);};
  // R66: the index of a part is built on its first use (std::call_once; most severs never use one).
  struct LazyPartIndex {std::once_flag once;const gfy_connectivity::Topology*volume{};PartIndex ix;};
  std::map<std::pair<std::uint32_t,std::uint16_t>,std::unique_ptr<LazyPartIndex>> partIndex;
  for(const auto id:ids){const auto pt=partitions.find(id);if(pt==partitions.end())continue;
   for(const auto&part:pt->second){const auto&tv=part.volume;if(tv.columns.empty()||!(tv.pitch>0))continue;
    auto slot=std::make_unique<LazyPartIndex>();slot->volume=&tv;partIndex.emplace(std::make_pair(id,part.bone),std::move(slot));}}
  auto buildPartIndex=[&](const gfy_connectivity::Topology&tv,PartIndex&ix){ix.vol=&tv;ix.ox=tv.origin[0];ix.oy=tv.origin[1];ix.oz=tv.origin[2];ix.pitch=tv.pitch;
    ix.F=std::max(1,int(std::lround(.01/tv.pitch)));ix.C=4*ix.F; // 1 cm leaf cells under 4 cm coarse cells
    int xmin=INT_MAX,ymin=INT_MAX,xmax=INT_MIN,ymax=INT_MIN,zmin=INT_MAX,zmax=INT_MIN;
    for(const auto&c:tv.columns){xmin=std::min(xmin,c.x);xmax=std::max(xmax,c.x);ymin=std::min(ymin,c.y);ymax=std::max(ymax,c.y);
     for(auto s=c.first;s<c.first+c.count;++s){zmin=std::min(zmin,tv.spans[s].lo);zmax=std::max(zmax,tv.spans[s].hi);}}
    ix.lo[0]=xmin;ix.lo[1]=ymin;ix.lo[2]=zmin;ix.hi[0]=xmax+1;ix.hi[1]=ymax+1;ix.hi[2]=zmax;
    ix.cx0=floorDiv(xmin,ix.C);ix.cy0=floorDiv(ymin,ix.C);ix.cnx=floorDiv(xmax,ix.C)-ix.cx0+1;ix.cny=floorDiv(ymax,ix.C)-ix.cy0+1;
    const std::size_t cells=std::size_t(ix.cnx)*std::size_t(ix.cny);
    ix.start.assign(cells+1,0);ix.zlo.assign(cells,INT_MAX);ix.zhi.assign(cells,INT_MIN);
    ix.fineStart.assign(cells*16+1,0);ix.fzlo.assign(cells*16,INT_MAX);ix.fzhi.assign(cells*16,INT_MIN);
    auto cellOf=[&](const gfy_connectivity::Column&c){return std::size_t(floorDiv(c.x,ix.C)-ix.cx0)*std::size_t(ix.cny)+std::size_t(floorDiv(c.y,ix.C)-ix.cy0);};
    auto fineOf=[&](const gfy_connectivity::Column&c){const int fx=floorDiv(c.x,ix.F)-4*floorDiv(c.x,ix.C),fy=floorDiv(c.y,ix.F)-4*floorDiv(c.y,ix.C);return std::size_t(fx*4+fy);};
    for(const auto&c:tv.columns){const auto i=cellOf(c),f=i*16+fineOf(c);++ix.fineStart[f+1];
     for(auto s=c.first;s<c.first+c.count;++s){ix.zlo[i]=std::min(ix.zlo[i],tv.spans[s].lo);ix.zhi[i]=std::max(ix.zhi[i],tv.spans[s].hi);ix.fzlo[f]=std::min(ix.fzlo[f],tv.spans[s].lo);ix.fzhi[f]=std::max(ix.fzhi[f],tv.spans[s].hi);}}
    for(std::size_t f=0;f<cells*16;++f)ix.fineStart[f+1]+=ix.fineStart[f];
    for(std::size_t i=0;i<cells;++i)ix.start[i]=ix.fineStart[i*16];ix.start[cells]=ix.fineStart[cells*16];
    ix.order.resize(tv.columns.size());{std::vector<std::uint32_t> fill(ix.fineStart.begin(),ix.fineStart.end()-1);
     for(std::uint32_t ci=0;ci<tv.columns.size();++ci){const auto&c=tv.columns[ci];ix.order[fill[cellOf(c)*16+fineOf(c)]++]=ci;}}
    for(std::uint32_t f=0;f<cells*16;++f)if(ix.fineStart[f]!=ix.fineStart[f+1])ix.occupiedLeaves.push_back(f);
   };
  // A build that throws leaves the slot untouched (call_once retries from a fresh index, never from a partial one).
  auto readyPartIndex=[&](LazyPartIndex&slot)->const PartIndex&{std::call_once(slot.once,[&]{PartIndex fresh;buildPartIndex(*slot.volume,fresh);slot.ix=std::move(fresh);});return slot.ix;};
  auto partIndexOf=[&](std::uint32_t id,std::uint16_t bone)->const PartIndex*{const auto found=partIndex.find({id,bone});
   return found==partIndex.end()?nullptr:&readyPartIndex(*found->second);};
  // Squared distance from p to the nearest occupied cell box of the part, or bound2 when nothing is nearer: the part's
  // box first, then 4 cm coarse cells walked in rings from the query cell (xy range x z range as the lower bound),
  // then the 1 cm leaf cells of a promising coarse cell, then the columns of a promising leaf cell. Exact: a box lower
  // bound only skips what cannot beat the best so far.
  auto partDistance2=[&](const PartIndex&ix,const Vec&p,double bound2)->double{
   const double fx=(p.x-ix.ox)/ix.pitch,fy=(p.y-ix.oy)/ix.pitch,fz=(p.z-ix.oz)/ix.pitch,pitch2=ix.pitch*ix.pitch;
   double best=bound2;
   {const double dx=painter_short_minmax::maxOf(double(ix.lo[0])-fx,fx-double(ix.hi[0]),0.),dy=painter_short_minmax::maxOf(double(ix.lo[1])-fy,fy-double(ix.hi[1]),0.),dz=painter_short_minmax::maxOf(double(ix.lo[2])-fz,fz-double(ix.hi[2]),0.);
    if((dx*dx+dy*dy+dz*dz)*pitch2>=best)return best;}
   const int qx=floorDiv(int(std::floor(fx)),ix.C)-ix.cx0,qy=floorDiv(int(std::floor(fy)),ix.C)-ix.cy0;
   const int rmax=painter_short_minmax::maxOf(qx,ix.cnx-1-qx,qy,ix.cny-1-qy,0)+1;
   for(int r=0;r<=rmax;++r){const double lb=r>0?double((r-1)*ix.C)*ix.pitch:0.;if(lb*lb>=best)break;
    for(int cx=qx-r;cx<=qx+r;++cx)for(int cy=qy-r;cy<=qy+r;++cy){if(std::max(std::abs(cx-qx),std::abs(cy-qy))!=r)continue;if(cx<0||cy<0||cx>=ix.cnx||cy>=ix.cny)continue;
     const std::size_t i=std::size_t(cx)*std::size_t(ix.cny)+std::size_t(cy);if(ix.start[i]==ix.start[i+1])continue;
     const double x0=double((ix.cx0+cx)*ix.C),y0=double((ix.cy0+cy)*ix.C);
     {const double dx=painter_short_minmax::maxOf(x0-fx,fx-(x0+ix.C),0.),dy=painter_short_minmax::maxOf(y0-fy,fy-(y0+ix.C),0.),dz=painter_short_minmax::maxOf(double(ix.zlo[i])-fz,fz-double(ix.zhi[i]),0.);
      if((dx*dx+dy*dy+dz*dz)*pitch2>=best)continue;}
     for(std::size_t f=i*16;f<i*16+16;++f){if(ix.fineStart[f]==ix.fineStart[f+1])continue;
      const double fx0=x0+double(int((f-i*16)/4)*ix.F),fy0=y0+double(int((f-i*16)%4)*ix.F);
      {const double dx=painter_short_minmax::maxOf(fx0-fx,fx-(fx0+ix.F),0.),dy=painter_short_minmax::maxOf(fy0-fy,fy-(fy0+ix.F),0.),dz=painter_short_minmax::maxOf(double(ix.fzlo[f])-fz,fz-double(ix.fzhi[f]),0.);
       if((dx*dx+dy*dy+dz*dz)*pitch2>=best)continue;}
      for(auto o=ix.fineStart[f];o<ix.fineStart[f+1];++o){const auto&c=ix.vol->columns[ix.order[o]];
       const double dx=painter_short_minmax::maxOf(double(c.x)-fx,fx-double(c.x+1),0.),dy=painter_short_minmax::maxOf(double(c.y)-fy,fy-double(c.y+1),0.);const double dxy2=(dx*dx+dy*dy)*pitch2;if(dxy2>=best)continue;
       for(auto s=c.first;s<c.first+c.count;++s){const auto&sp=ix.vol->spans[s];const double dz=painter_short_minmax::maxOf(double(sp.lo)-fz,fz-double(sp.hi),0.);const double d2=dxy2+dz*dz*pitch2;if(d2<best)best=d2;}}}}}
   return best;};
  // Immutable per-element entries retain ids order; the legacy fallback below still performs its original map walk.
  std::array<std::vector<std::pair<int,const PartIndex*>>,64> decisionParts;std::array<std::vector<LazyPartIndex*>,64> decisionSlots;
  for(std::size_t k=0;k<ids.size();++k)for(unsigned e=0;e<64;++e){const auto it=partIndex.find({ids[k],std::uint16_t(e)});
   if(it!=partIndex.end()){decisionParts[e].push_back({int(k),&it->second->ix});decisionSlots[e].push_back(it->second.get());}}
  static const bool decisionDisabled=std::getenv("GFY_NO_PART_OWNER_DECISION")!=nullptr;
  static const bool decisionStats=std::getenv("GFY_PART_OWNER_DECISION_STATS")!=nullptr;
  std::atomic<std::uint64_t> decisionCalls{0},decisionFast{0};
  auto splitOwner=[&](std::uint16_t bone,const Vec&p)->int{
   if(bone>=frames.bones.size()||frames.bones[bone].physical_element<0)return -1;const auto e=std::uint16_t(frames.bones[bone].physical_element);
   if(decisionStats)decisionCalls.fetch_add(1,std::memory_order_relaxed);
   if(!decisionDisabled&&e<decisionParts.size()){
    for(auto*slot:decisionSlots[e])(void)readyPartIndex(*slot); // R66: the element's indexes, built once, before the decision reads them
    const int proven=gfy_part_owner_decision::decide(decisionParts[e],p,floorDiv);
    if(proven>=0){if(decisionStats)decisionFast.fetch_add(1,std::memory_order_relaxed);return proven;}}
   int found=-1;double best=std::numeric_limits<double>::infinity();
   for(std::size_t k=0;k<ids.size();++k){const auto*ix=partIndexOf(ids[k],e);if(!ix)continue;const double d2=partDistance2(*ix,p,best);if(d2<best){best=d2;found=int(k);}}
   if(found>=0)return found;
   best=1e30;
   for(std::size_t k=0;k<ids.size();++k){const auto c=partCentre.find({ids[k],e});if(c==partCentre.end())continue;const Vec d=c->second-p;const double d2=dot(d,d);if(d2<best){best=d2;found=int(k);}}
   return found;};
  // Qualify original accessory ownership before the body-only lowpoly pass.
  // An ocular island may have no body triangles because its authored surface
  // is a separate original child. Only exact partitioned source proves it.
  std::array<bound_surface::Result,2> accessoryPartition;original_accessory::CollectDrops accessoryDrops{}; // gfyrefuse safety net: what collect() left out
  std::array<std::map<std::uint32_t,original_accessory::GearMesh>,2> gearMeshes;std::vector<original_accessory::LightTriangle> light; // gfyspeed3
  std::set<std::uint32_t> deferredEyeSurfaces;
  // ownersafe (2026-11-15): no gear (accessory) step may refuse a separation (f13ab: cpu_refused reason=accessory_repeat_barycentric_range,
  // the blowout's chunks never came off). A gear triangle that cannot be read back or partitioned is left out (counted, the first
  // reason kept); when the gear partition as a whole cannot be done the pieces are cut without gear surfaces; a piece whose gear cannot
  // be appended keeps its body surface. The summary ends with " accessory_skipped ..." when anything was skipped.
  std::size_t partitionDropped=0,appendSkipped=0;std::string partitionWhy,appendWhy,gearSkipped;
  if(accessorySurface&&!accessorySurface->source.f.empty())try{
   // gfyspeed3: with the asset's gear memo (the same gear surface) the triangles are collected light when every row reads back
   // exactly; collect() itself (ownersafe's rules) otherwise.
   const auto gsGear=gs3On()?accessoryGear(asset,*accessorySurface):GearAccess{};std::vector<original_accessory::Triangle> attachedStore;
   const bool gsLight=gsGear.collect&&original_accessory::collectLight(asset,*accessorySurface,*extras.lowpoly,extras.piece!=nullptr,*gsGear.collect,light);
   if(!gsLight){light.clear();attachedStore=original_accessory::collect(asset,*accessorySurface,*extras.lowpoly,extras.piece!=nullptr);}
   accessoryDrops=original_accessory::lastCollectDrops;const auto&attached=attachedStore;
   if(painter_ownersafe_fault("sep_block"))throw std::runtime_error("ocular_source_face_range"); // ownersafe test switch
   // gfyspeed3: the asset's ocular map - qualify() of the same asset and surface - when it is the same gear surface
   const auto gsOcular=gsGear.collect?gsGear.ocular:std::make_shared<const ocular_source::Map>(ocular_source::qualify(asset,*accessorySurface));
   const auto&ocular=*gsOcular;
   std::vector<std::uint32_t> ocularOwners(ocular.islands.size(),0);
   for(std::size_t q=0;q<ocular.islands.size();++q)if(ocular.islands[q].qualified)
    ocularOwners[q]=ocular_source::uniqueOwner(asset,ocular.islands[q],sgrid,pieceVolume);
   const std::set<std::uint32_t> retained(ids.begin(),ids.end());std::uint64_t partitionWork=0;
   // R66: the partitions on worker threads (see patch_accpar66): routes in order, private results, the serial loop's
   // budgets checked on the sums, triangles appended in triangle order; anything else runs the serial loop below.
   bool parallelDone=false;
   static const bool serialAccessory=std::getenv("GFY_SEP_SERIAL_ACCESSORY")!=nullptr;
   if(gsLight&&!serialAccessory&&light.size()>=8){
    // gfyspeed3: the R66 parallel partition on the light triangles. Each is partitioned first by its corner positions alone with a
    // deferred keep: until it keeps the triangle whole, partitionTriangle() reads no other field of it (area() reads the positions,
    // keep() records the owner). Any other outcome runs again on collect()'s triangle (materialize()) in a fresh result: the
    // reference outcome, stats and errors. A kept triangle's rows are packed beside it (lightPack). survivingSupport() - a pure
    // function of the island and the grid - is asked once per island.
    // blowoutrest (2026-11-16): a triangle kept whole by the early rule records its owner, work and output count (a thread's scratch
    // result) instead of a whole partition result of its own; any other outcome keeps its full result as before.
    // Its clip is read once (the loops after the partition do not walk the light triangles again).
    struct Routed {std::uint8_t route{},clip{};bool compact{},grBound{};std::uint32_t owner{},cOwner{};std::uint64_t cWork{},cOutput{};std::unique_ptr<bound_surface::Result> full;std::uint64_t peak{};std::exception_ptr error;
     std::uint64_t work()const{return compact?cWork:full->stats.work;}std::uint64_t output()const{return compact?cOutput:full->stats.output;}};
    std::vector<Routed> routed(light.size());bool clean=true;std::vector<signed char> support(ocular.islands.size(),-1);
    for(std::size_t j=0;j<light.size();++j)routed[j].clip=light[j].skin_clip?1:0;
    static const bool grFull=std::getenv("GFY_GR_FULL")!=nullptr,grVerify=std::getenv("GFY_GR_VERIFY")!=nullptr;std::atomic<std::size_t> grChecked{0},grDiffer{0};
    try{for(std::size_t j=0;j<light.size()&&clean;++j){const auto f=light[j].f;
      if(!(f<ocular.face_island.size())){clean=false;break;}
      const auto island=ocular.face_island[f];const auto owner=ocularOwners[island];
      if(ocular.islands[island].qualified&&owner&&retained.count(owner)){routed[j].route=1;routed[j].owner=owner;continue;}
      if(ocular.islands[island].qualified){auto&s=support[island];if(s<0)s=ocular_source::survivingSupport(asset,ocular.islands[island],sgrid)?1:0;if(!s)continue;}
      routed[j].route=2;}}
    catch(...){clean=false;}
    if(clean){
     {std::atomic<std::size_t> cursor{0};const std::size_t n=light.size();
      sep::parallel(n,1,sep::max_job_threads,[&](std::size_t,std::size_t,std::size_t){
       for(std::size_t j;(j=cursor.fetch_add(1))<n;){auto&r=routed[j];if(r.route!=2)continue;auto&l=light[j];
        bool kept=false;
        // only a triangle whose snapped corners read one retained owner can be kept by the early rule: the others go straight to
        // collect()'s triangle (either order gives the reference result; this one avoids running them twice)
        const std::uint32_t o0=bound_surface::at(bound_surface::snapToCellPlanes(l.anchors[0],sgrid),sgrid,pieceVolume);
        const bool shapeFirst=o0==bound_surface::at(bound_surface::snapToCellPlanes(l.anchors[1],sgrid),sgrid,pieceVolume)&&
         o0==bound_surface::at(bound_surface::snapToCellPlanes(l.anchors[2],sgrid),sgrid,pieceVolume)&&retained.count(o0);
        if(shapeFirst){thread_local core::JoinedTriangle shape{};for(unsigned k=0;k<3;++k)shape.v[k].p=l.p[k]; // only the positions are ever set
         thread_local bound_surface::Result scratch;scratch.meshes.clear();scratch.stats=bound_surface::Stats{};scratch.deferred=false;scratch.deferredOwner=0;
         scratch.deferKeep=true;bound_surface::partitionWorkPeak=&r.peak;
         try{bound_surface::partitionTriangle(shape,l.anchors,sgrid,pieceVolume,retained,scratch,partitionCertificate.get());kept=scratch.deferred;}catch(...){kept=false;}
         bound_surface::partitionWorkPeak=nullptr;
         if(kept){if(scratch.meshes.empty()){r.compact=true;r.cOwner=scratch.deferredOwner;r.cWork=scratch.stats.work;r.cOutput=scratch.stats.output;}
          else r.full=std::make_unique<bound_surface::Result>(std::move(scratch));}}
        if(kept){original_accessory::lightPack(*accessorySurface,l);continue;}
        // gearregion: a triangle whose corners the early rule cannot read (a fragment of an earlier cut) and that lies whole over one
        // retained component's spans is kept whole for it - the partition's own rule after its clipping (intactOverOneOwner)
        // (GFY_GR_FULL: every such triangle by the partition; GFY_GR_VERIFY: both, the partition's result is used and differences are
        // counted on the summary's line)
        if(!shapeFirst&&!grFull){std::uint32_t owner=0;std::uint64_t bound=0;
         if(bound_surface::intactOverOneOwner(l.anchors,sgrid,pieceVolume,retained,owner,bound)){
          if(grVerify){bound_surface::Result ref;std::uint64_t refPeak=0;bound_surface::partitionWorkPeak=&refPeak;bool same=false;
           try{const auto mt=original_accessory::materialize(*accessorySurface,l);bound_surface::partitionTriangle(mt.surface,mt.anchors,sgrid,pieceVolume,retained,ref,partitionCertificate.get());
            same=!ref.deferred&&ref.stats.kept==1&&ref.stats.output==1&&ref.meshes.size()==1&&ref.meshes.begin()->first==owner&&ref.meshes.begin()->second.triangles.size()==1&&
             ref.stats.work<=bound&&refPeak<=bound;} // kept==1: the partition's keep() of the intact triangle (a split counts split)
           catch(...){same=false;}
           bound_surface::partitionWorkPeak=nullptr;grChecked.fetch_add(1);if(!same){grDiffer.fetch_add(1);goto grReference;}}
          r.compact=true;r.cOwner=owner;r.cWork=bound;r.cOutput=1;r.peak=bound;r.grBound=true;original_accessory::lightPack(*accessorySurface,l);continue;}}
        grReference:
        r.full=std::make_unique<bound_surface::Result>();r.peak=0;const auto full=original_accessory::materialize(*accessorySurface,l);
        bound_surface::partitionWorkPeak=&r.peak;
        try{bound_surface::partitionTriangle(full.surface,full.anchors,sgrid,pieceVolume,retained,*r.full,partitionCertificate.get());}
        catch(...){r.error=std::current_exception();}
        bound_surface::partitionWorkPeak=nullptr;}});}
     std::uint64_t peaks=0;std::size_t outputs[2]{};
     for(std::size_t j=0;j<light.size()&&clean;++j){auto&r=routed[j];if(r.route!=2)continue;
      if(r.error){ // ownersafe's rule (as in the statements below): a gear triangle that cannot be partitioned is left out
       try{std::rethrow_exception(r.error);}catch(const std::bad_alloc&){throw;}catch(const std::exception&e){if(!partitionDropped)partitionWhy=e.what();}catch(...){if(!partitionDropped)partitionWhy="unknown";}
       ++partitionDropped;r.route=0;r.error=nullptr;r.full.reset();r.compact=false;continue;}
      peaks+=std::max(r.peak,r.work());outputs[r.clip]+=r.output();}
     if(clean&&peaks>160000000ull){peaks=0;outputs[0]=outputs[1]=0; // gearregion: never decide the budget on a bound
      for(std::size_t j=0;j<light.size();++j){auto&r=routed[j];if(r.route!=2)continue;
       if(r.grBound){r.compact=false;r.grBound=false;r.full=std::make_unique<bound_surface::Result>();r.peak=0;const auto full=original_accessory::materialize(*accessorySurface,light[j]);
        bound_surface::partitionWorkPeak=&r.peak;try{bound_surface::partitionTriangle(full.surface,full.anchors,sgrid,pieceVolume,retained,*r.full,partitionCertificate.get());}catch(...){r.error=std::current_exception();}
        bound_surface::partitionWorkPeak=nullptr;
        if(r.error){try{std::rethrow_exception(r.error);}catch(const std::bad_alloc&){throw;}catch(const std::exception&e){if(!partitionDropped)partitionWhy=e.what();}catch(...){if(!partitionDropped)partitionWhy="unknown";}
         ++partitionDropped;r.route=0;r.error=nullptr;r.full.reset();continue;}}
       peaks+=std::max(r.peak,r.work());outputs[r.clip]+=r.output();}}
     if(grVerify)std::fprintf(stderr,"gearregion_verify checked=%zu differ=%zu\n",grChecked.load(),grDiffer.load());
     if(clean&&peaks<=160000000ull&&outputs[0]<4000000&&outputs[1]<4000000){original_accessory::GearMesh*lastGear=nullptr;unsigned lastClip=0;std::uint32_t lastOwner=0;
      // each piece's list of whole (cut) gear triangles is reserved at its final size first: a growing list copied every 2 KB
      // triangle again on each growth (16-33k of them a blowout on GFY 1.06); the same entries, created in the same loop below
      {std::array<std::map<std::uint32_t,std::size_t>,2> count;
       for(std::size_t j=0;j<light.size();++j){const auto&r=routed[j];
        if(r.route==1){++count[1][r.owner];continue;}
        if(r.route!=2||r.compact||!r.full)continue;for(const auto&m:r.full->meshes)count[r.clip][m.first]+=m.second.triangles.size();}
       for(unsigned c=0;c<2;++c)for(const auto&e:count[c])if(e.second)gearMeshes[c][e.first].tris.reserve(e.second);}
      for(std::size_t j=0;j<light.size();++j){auto&r=routed[j];
       if(r.route==1){auto&g=gearMeshes[1][r.owner];g.order.emplace_back(~std::uint32_t(0),std::uint32_t(g.tris.size()));g.tris.push_back(original_accessory::materialize(*accessorySurface,light[j]).surface);continue;}
       if(r.route!=2)continue;const unsigned clip=r.clip;auto&result=accessoryPartition[clip];
       if(r.compact){ // the same map entry as gearMeshes[clip][owner], looked up once per run of one owner
        if(!lastGear||lastClip!=clip||lastOwner!=r.cOwner){lastGear=&gearMeshes[clip][r.cOwner];lastClip=clip;lastOwner=r.cOwner;}
        lastGear->order.emplace_back(std::uint32_t(j),0u);partitionWork+=r.cWork;result.stats.work=partitionWork;continue;}
       if(r.full->deferred)gearMeshes[clip][r.full->deferredOwner].order.emplace_back(std::uint32_t(j),0u);
       for(auto&m:r.full->meshes){auto&g=gearMeshes[clip][m.first];
        for(auto&tri:m.second.triangles){g.order.emplace_back(~std::uint32_t(0),std::uint32_t(g.tris.size()));g.tris.push_back(std::move(tri));}}
       partitionWork+=r.full->stats.work;result.stats.work=partitionWork;}
      parallelDone=true;}}
   }
   // gfyspeed3: anything else runs the former statements on collect()'s triangles
   if(gsLight&&!parallelDone){attachedStore.reserve(light.size());for(const auto&l:light)attachedStore.push_back(original_accessory::materialize(*accessorySurface,l));}
   if(!gsLight&&!serialAccessory&&attached.size()>=8){
    struct Routed {std::uint8_t route{};std::uint32_t owner{};bound_surface::Result result;std::uint64_t peak{};std::exception_ptr error;};
    std::vector<Routed> routed(attached.size());bool clean=true;
    try{for(std::size_t j=0;j<attached.size()&&clean;++j){const auto&t=attached[j];const auto f=t.surface.v[0].sourceFace;
      if(!(f<ocular.face_island.size())){clean=false;break;}
      const auto island=ocular.face_island[f];const auto owner=ocularOwners[island];
      if(ocular.islands[island].qualified&&owner&&retained.count(owner)){routed[j].route=1;routed[j].owner=owner;continue;}
      if(ocular.islands[island].qualified&&!ocular_source::survivingSupport(asset,ocular.islands[island],sgrid))continue;
      routed[j].route=2;}}
    catch(...){clean=false;}
    if(clean){
     // The separation's thread budget (sep::max_job_threads, as the collider prefits): each thread takes the next triangle.
     {std::atomic<std::size_t> cursor{0};const std::size_t n=attached.size();
      sep::parallel(n,1,sep::max_job_threads,[&](std::size_t,std::size_t,std::size_t){
       for(std::size_t j;(j=cursor.fetch_add(1))<n;){auto&r=routed[j];if(r.route!=2)continue;const auto&t=attached[j];
        bound_surface::partitionWorkPeak=&r.peak;
        try{if(painter_ownersafe_fault("sep_partition"))throw std::runtime_error("source_partition_work_budget"); /* ownersafe test switch */
         bound_surface::partitionTriangle(t.surface,t.anchors,sgrid,pieceVolume,retained,r.result,partitionCertificate.get());}
        catch(...){r.error=std::current_exception();}
        bound_surface::partitionWorkPeak=nullptr;}});}
     std::uint64_t peaks=0;std::size_t outputs[2]{};
     for(std::size_t j=0;j<attached.size()&&clean;++j){auto&r=routed[j];if(r.route!=2)continue;
      if(r.error){ // ownersafe: a gear triangle that cannot be partitioned is left out (it refused the whole separation)
       try{std::rethrow_exception(r.error);}catch(const std::bad_alloc&){throw;}catch(const std::exception&e){if(!partitionDropped)partitionWhy=e.what();}catch(...){if(!partitionDropped)partitionWhy="unknown";}
       ++partitionDropped;r.route=0;r.error=nullptr;r.result=bound_surface::Result{};continue;}
      peaks+=std::max(r.peak,r.result.stats.work);outputs[attached[j].skin_clip?1:0]+=r.result.stats.output;}
     if(clean&&peaks<=160000000ull&&outputs[0]<4000000&&outputs[1]<4000000){
      for(std::size_t j=0;j<attached.size();++j){auto&r=routed[j];
       if(r.route==1){accessoryPartition[1].meshes[r.owner].triangles.push_back(attached[j].surface);continue;}
       if(r.route!=2)continue;auto&result=accessoryPartition[attached[j].skin_clip?1:0];
       for(auto&m:r.result.meshes){auto&dst=result.meshes[m.first].triangles;
        dst.insert(dst.end(),std::make_move_iterator(m.second.triangles.begin()),std::make_move_iterator(m.second.triangles.end()));}
       partitionWork+=r.result.stats.work;result.stats.work=partitionWork;}
      parallelDone=true;}}
   }
   if(!parallelDone)
   for(const auto&t:attached)try{const auto f=t.surface.v[0].sourceFace;need(f<ocular.face_island.size(),"ocular_source_face_range"); // ownersafe: per triangle
    const auto island=ocular.face_island[f];const auto owner=ocularOwners[island];
    // The fixed garment anchors of real eyeballs can lie on nearby brain.
    // A UNIQUE surviving original slot-14 overlap proves this ocular island's
    // owner directly. Ambiguous fragments keep the exact fixed-anchor path.
    if(ocular.islands[island].qualified&&owner&&retained.count(owner)){
     accessoryPartition[1].meshes[owner].triangles.push_back(t.surface);continue;}
    if(ocular.islands[island].qualified&&!ocular_source::survivingSupport(asset,ocular.islands[island],sgrid))continue;
    auto&result=accessoryPartition[t.skin_clip?1:0];
    // ownersafe: partitioned into its own result first, so a triangle that cannot be partitioned leaves nothing behind
    bound_surface::Result one;one.stats=result.stats;one.stats.work=partitionWork;
    if(painter_ownersafe_fault("sep_partition"))throw std::runtime_error("source_partition_work_budget"); // ownersafe test switch
    bound_surface::partitionTriangle(t.surface,t.anchors,sgrid,pieceVolume,retained,one,partitionCertificate.get());
    for(auto&m:one.meshes){auto&dst=result.meshes[m.first].triangles;
     dst.insert(dst.end(),std::make_move_iterator(m.second.triangles.begin()),std::make_move_iterator(m.second.triangles.end()));}
    result.stats=one.stats;partitionWork=result.stats.work;}
   catch(const std::bad_alloc&){throw;}
   catch(const std::exception&e){if(!partitionDropped++)partitionWhy=e.what();} // ownersafe: this gear triangle is left out
   for(const auto&result:accessoryPartition)for(const auto&owner:result.meshes)
    if(owner.first<detachedEyes.size()&&detachedEyes[owner.first]&&!owner.second.triangles.empty())deferredEyeSurfaces.insert(owner.first);
   for(const auto&clip:gearMeshes)for(const auto&owner:clip) // gfyspeed3
    if(owner.first<detachedEyes.size()&&detachedEyes[owner.first]&&!owner.second.order.empty())deferredEyeSurfaces.insert(owner.first);
  }catch(const std::bad_alloc&){throw;}
  catch(const std::exception&e){accessoryPartition={};gearMeshes={};deferredEyeSurfaces.clear();gearSkipped=e.what();} // ownersafe: pieces without gear surfaces
  lowpoly_pieces::Stats lp;mask_pieces::Stats mp;const bool maskPieces=extras.lowpoly->mask_volume&&!extras.lowpoly->analytic_channels;
  mask_pieces3::Stats m3;bool tearPieces=maskPieces&&mask_pieces3::eligible(*extras.lowpoly,sgrid,pieceVolume,ids,m3);
  try{if(tearPieces){try{surfaces.pieces=mask_pieces3::build(*extras.lowpoly,sgrid,pieceVolume,ids,whole,asset.original_children,anchorable,regionFrontier,m3,
    extras.piece?&extras.piece->marks:nullptr,&pieceMarks,asset.gear_cells.get());} // slices 3 + 4: the tears' sides, no ownership field; cutfix: gear cells
    catch(const lowpoly_pieces::EmptyPieces&){throw;}catch(const std::bad_alloc&){throw;}
    catch(const std::exception&e){tearPieces=false;surfaces.pieces.clear();pieceMarks.clear();static thread_local std::string why;why=e.what();m3.why=why.c_str(); // logged below
     std::fprintf(stderr,"tear_pieces_fallback why=%s\n",e.what());}}
   if(tearPieces){}
   else if(maskPieces){surfaces.pieces=mask_pieces::build(*extras.lowpoly,sgrid,pieceVolume,ids,whole,asset.original_children,anchorable,regionFrontier,mp,
    extras.piece?&extras.piece->marks:nullptr,&pieceMarks); // mask-only wounds, slice 2: no surface partition
    mask_pieces3::attach(surfaces.pieces,ids,*extras.lowpoly,sgrid,pieceVolume,m3);} // slices 3 + 4: each piece keeps the tears it borders
   else surfaces.pieces=lowpoly_pieces::build(*extras.lowpoly,sgrid,pieceVolume,ids,whole,asset.original_children,anchorable,boneOwner,splitOwner,lp,
   extras.piece?&extras.piece->marks:nullptr,&pieceMarks,asset.tile_material.get(),&deferredEyeSurfaces,&attachedFaces,regionFrontier,bindingRegions);
   if(decisionStats)std::fprintf(stderr,"part_decision calls=%llu fast=%llu fallback=%llu\n",
    (unsigned long long)decisionCalls.load(),(unsigned long long)decisionFast.load(),
    (unsigned long long)(decisionCalls.load()-decisionFast.load()));}
  catch(const lowpoly_pieces::EmptyPieces&empty){
   earlyFits.reset(); // split2: these fits belong to the partition that is given up here
   // An eye may only become an owner of its exact authored/published surface.
   // Never merge an eye with no qualified surface back into a nearby head.
   for(const auto id:empty.components)if(id<detachedEyes.size()&&detachedEyes[id])
    throw std::runtime_error("separation_eye_surface_unavailable");
   // Retry only this formerly rejected case. Existing renderable islands and
   // every ordinary split retain their original ownership and output bytes.
   if(extras.semantic_groups||wantedComponents||std::getenv("GFY_COMPONENT_SESSIONS"))throw;
   IslandGroups groups;
   if(!chooseIslandGroups(sgrid,volume,pristine,ids,empty.components,garment.component_elements,groups))throw;
   for(const auto&group:groups)if((group.first<detachedEyes.size()&&detachedEyes[group.first])||
      (group.second<detachedEyes.size()&&detachedEyes[group.second]))
    throw std::runtime_error("separation_eye_surface_cannot_regroup");
   PrepareExtras retry=extras;retry.semantic_groups=std::make_shared<const IslandGroups>(std::move(groups));retry.precomputed_volume=volumeShared;
   const auto preservedGrid=std::make_shared<const Grid>(sgrid);std::shared_ptr<const Grid> retryGrid;retry.grid_out=&retryGrid;
   auto result=prepare(asset,*preservedGrid,stamp,input,wantedComponents,checkpoint,retry);
   if(extras.grid_out)*extras.grid_out=retryGrid?retryGrid:preservedGrid;
   return result;
  }

  if(accessorySurface&&!accessorySurface->source.f.empty()){
   for(std::size_t k=0;k<surfaces.pieces.size();++k){auto&piece=surfaces.pieces[k];
    // ownersafe: a piece whose gear cannot be appended (or its clip mask carried) keeps its body surface, without that gear
    const std::size_t keptChildren=piece.mesh.children.size(),keptTriangles=piece.triangles;std::vector<std::pair<Cell,core::Tile>> keptMarks;if(!maskPieces)keptMarks=pieceMarks[k];
    try{
    for(unsigned clip=0;clip<2;++clip){const auto found=accessoryPartition[clip].meshes.find(piece.volume_component);
     if(found!=accessoryPartition[clip].meshes.end()){if(painter_ownersafe_fault("sep_append"))throw std::runtime_error("accessory_output_triangle_conservation"); // ownersafe test switch
      original_accessory::append(piece.mesh,found->second,*accessorySurface,asset.original_children,clip!=0);
      piece.triangles+=found->second.triangles.size();}
     const auto gear=gearMeshes[clip].find(piece.volume_component); // gfyspeed3: the light path's triangles of this piece
     if(gear!=gearMeshes[clip].end()&&!gear->second.order.empty()){original_accessory::appendGear(piece.mesh,gear->second,light,*accessorySurface,asset.original_children,clip!=0);
      piece.triangles+=gear->second.order.size();}}
    if(!maskPieces)original_accessory::preserveClipMask(piece.mesh,*extras.lowpoly,*accessorySurface,sgrid,extras.piece?&extras.piece->marks:nullptr,pieceMarks[k]);
    }catch(const std::bad_alloc&){throw;}
    catch(const std::exception&e){piece.mesh.children.erase(piece.mesh.children.begin()+std::ptrdiff_t(keptChildren),piece.mesh.children.end());piece.triangles=keptTriangles;
     if(!maskPieces)pieceMarks[k]=std::move(keptMarks);if(!appendSkipped++)appendWhy=e.what();}
    if(deferredEyeSurfaces.count(piece.volume_component))need(piece.triangles>0&&!piece.mesh.children.empty(),"separation_eye_source_append_empty");}
  }

  char text[1024];std::snprintf(text,sizeof(text),"pieces=%zu skin_triangles=%zu channel_triangles=%zu dropped=%zu channel_dropped=%zu unanchored=%zu boundary=%zu tiles=%zu foreign_cells=%zu vertices_marched=%zu by_bone=%zu nearest=%zu unowned=%zu owner_ms=%.1f (vertex %.1f ring %.1f triangle %.1f) mask_ms=%.1f ms=%.1f frozen=%zu",
   surfaces.pieces.size(),lp.skin,lp.channel,lp.dropped,lp.channelDropped,lp.unanchored,lp.boundary,lp.tiles,lp.foreign,lp.marched,lp.byBone,lp.nearest,lp.unowned,lp.ownerMs,lp.vertexMs,lp.ringMs,lp.triangleMs,lp.maskMs,lp.ms,lp.frozen);
  out->lowpoly_summary=text;
  if(accessoryDrops.repeats||accessoryDrops.overCovered){char drops[96];std::snprintf(drops,sizeof(drops)," accessory_dropped repeats=%zu over_covered=%zu",
   accessoryDrops.repeats,accessoryDrops.overCovered);out->lowpoly_summary+=drops;} // gfyrefuse safety net: a refusal it replaced
  if(accessoryDrops.unread||accessoryDrops.clamped||partitionDropped||appendSkipped||!gearSkipped.empty()){char skipped[192]; // ownersafe
   std::snprintf(skipped,sizeof(skipped)," accessory_skipped unread=%zu reread_nearest=%zu partition_dropped=%zu piece_gear_dropped=%zu",
    accessoryDrops.unread,accessoryDrops.clamped,partitionDropped,appendSkipped);out->lowpoly_summary+=skipped;
   if(accessoryDrops.unread)out->lowpoly_summary+=" unread_why="+accessoryDrops.why;
   if(partitionDropped)out->lowpoly_summary+=" partition_why="+partitionWhy;
   if(appendSkipped)out->lowpoly_summary+=" piece_gear_why="+appendWhy;
   if(!gearSkipped.empty())out->lowpoly_summary+=" gear_partition=skipped why="+gearSkipped;}
  if(tearPieces){char ttext[768];std::snprintf(ttext,sizeof(ttext),"tear_pieces=1 tears=%zu unsplit=%zu sides=%s kept=%zu cracks=%zu selected=%zu in_region=%zu by_skeleton=%zu ties=%zu by_volume=%zu dropped=%zu "
    "unposable=%zu frozen=%zu boxes=%zu box_triangles=%zu field_bytes=%zu atlas_tears=%zu atlas_kb=%zu atlas_ms=%.2f atlas_overlap3=%zu vote_ms=%.2f select_ms=%.1f pack_ms=%.1f ms=%.1f | ",m3.tears,m3.fresh,m3.sides().c_str(),m3.kept,m3.cracks,
    m3.selected,m3.inRegion,m3.bySkeleton,m3.ties,m3.byVolume,m3.dropped,m3.unposable,m3.frozen,m3.boxes,m3.boxTriangles,
    m3.fieldBytes,m3.atlas.tears,m3.atlas.bytes/1024,m3.atlas.ms,m3.atlas.overlap3,m3.voteMs,m3.selectMs,m3.packMs,m3.ms);out->lowpoly_summary=std::string(ttext)+out->lowpoly_summary;}
  else if(maskPieces&&!extras.lowpoly->tears.empty()){out->lowpoly_summary=std::string("tear_pieces=0 why=")+m3.why+(m3.attached?" attached=1":" attached=0")+" | "+out->lowpoly_summary;}
  if(maskPieces&&!tearPieces){char mtext[768];std::snprintf(mtext,sizeof(mtext),"mask_pieces=1 selected=%zu dropped=%zu unposable=%zu frozen=%zu boxes=%zu box_triangles=%zu field_bytes=%zu "
    "field_ms=%.1f (tiles %.1f edt %.1f mark %.1f box %.1f contested_tiles=%zu edt_cells=%zu) select_ms=%.1f pack_ms=%.1f mask_ms=%.1f | ",
    mp.selected,mp.dropped,mp.unposable,mp.frozen,mp.boxes,mp.boxTriangles,mp.fieldBytes,mp.fieldMs,mp.field.tileMs,mp.field.edtMs,mp.field.markMs,mp.field.boxMs,
    mp.field.contestedTiles,mp.field.edtCells,mp.selectMs,mp.packMs,mp.ms);out->lowpoly_summary=std::string(mtext)+out->lowpoly_summary;}
  out->lowpoly_summary+=" kept_shell="+std::to_string(lp.keptShell)+" shell_by_piece="+lp.shellByPiece;} // E5: kept/shell/visible/emptied/cm2@centre per piece
 lap(separation_phase_surface_partition);progress("surface_ownership");
#ifdef GFY_COLLIDER_DIAGNOSTICS
 phase("surfaces");
#endif
 std::set<std::uint16_t> surfaceChildren(asset.face_children.begin(),asset.face_children.end());
 if(accessorySurface)surfaceChildren.insert(accessorySurface->face_children.begin(),accessorySurface->face_children.end());
 out->original_surface_children.assign(surfaceChildren.begin(),surfaceChildren.end());
 std::set<std::uint16_t> accessoryChildren;
 for(const auto&a:input.accessories){need(a.child<asset.original_children&&!surfaceChildren.count(a.child)&&accessoryChildren.insert(a.child).second,"separation_accessory_child_identity");
  need(a.physical_element<asset.source.bones&&decoded->frames.bones[a.physical_element].physical_element==a.physical_element,"separation_accessory_physical_element"); // original labels: original frames
  need(asset.base.anatomy&&asset.base.anatomy->point(a.original_anchor.x,a.original_anchor.y,a.original_anchor.z),"separation_accessory_not_original_anatomy");}
 need(surfaceChildren.size()+accessoryChildren.size()==asset.original_children,"separation_missing_original_child_ownership");
 std::uint64_t largest=0;
 // Collider fits are pure functions of (part pieceVolume, original frame). Fit
 // every one the loop below will request on up to four threads first; the
 // loop then consumes them in its order, rethrowing a stored failure exactly
 // where (and with the same message as) the serial fit would have thrown.
 // Selection mirrors the loop; anything whose selection would throw is left
 // to the serial path, which throws the same error first.
 std::vector<Prefit>prefits; // split2: selected by selectPrefits (defined before the surface partition), the same rule
 {std::vector<std::uint32_t> pieceComponents;
  for(const auto&surface:surfaces.pieces){if(wantedComponents&&!wantedComponents->count(surface.volume_component))continue;pieceComponents.push_back(surface.volume_component);}
  prefits=selectPrefits(pieceComponents);}
 // C2 (separation-track): the collider fits of every side and the low-poly
 // reductions of the piece surfaces are pure functions of their inputs (part
 // pieceVolume + original frame; the packed mesh). They run side by side on up to
 // four threads (sep::parallel: each fit keeps its internals serial under
 // SerialScope, each reduction is single-threaded), reductions first (the
 // largest tasks); the loop below consumes them in its order and rethrows a
 // stored failure exactly where, and with the message, the serial code threw.
 // GFY_SEP_COLLIDER_OUTER=0 restores the former serial fits with the fitter's
 // own internal fan-out (measured faster for one very large torso element);
 // the reductions still run first, side by side.
 struct Reduction{std::size_t surface;gfy_packed::Mesh mesh;std::exception_ptr error;double ms{};};
 std::vector<Reduction>reductions;
 for(std::size_t i=0;i<surfaces.pieces.size();++i){auto&surface=surfaces.pieces[i];if(wantedComponents&&!wantedComponents->count(surface.volume_component))continue;
  Reduction r;r.surface=i;r.mesh=std::move(surface.mesh);reductions.push_back(std::move(r));}
 // split2: the fits started before the surface partition are the piece loop's fits when the two selections agree
 // (every fit of the loop, in loop order): finish them and take them all - the loop then finds a result for every fit.
 // Otherwise stop them, and reduce, order and fit here as before (planPrefits = the former reduction and order).
 // speedfix: coarse-limit fits made ahead by the crew (component, bone) -> result or its failure (EarlyFits::coarseTask)
 std::map<std::pair<std::uint32_t,std::uint16_t>,std::pair<pc::Prepared,std::exception_ptr>> coarseAhead;
 bool earlyTaken=false;
 if(earlyFits){bool same=earlyFits->prefits.size()==prefits.size();
  for(std::size_t k=0;same&&k<prefits.size();++k){const auto&a=earlyFits->prefits[k];const auto&b=prefits[k];same=a.component==b.component&&a.bone==b.bone&&a.volume==b.volume;}
  if(same){profMark("sp_end");earlyFits->finish();
   for(std::size_t k=0;k<earlyFits->coarseState.size();++k){const int s=earlyFits->coarseState[k].load(); // speedfix: the coarse-limit fits
    const auto&f=earlyFits->prefits[k];if(s==3)coarseAhead[{f.component,f.bone}]={earlyFits->coarseFit[k],nullptr};
    else if(s==4)coarseAhead[{f.component,f.bone}]={gfy_piece_collider::Prepared{},earlyFits->coarseError[k]};}
   prefits=std::move(earlyFits->prefits);earlyTaken=true;profMark("fits_wait");}
  earlyFits.reset();}
 const std::vector<std::size_t> prefitOrder=earlyTaken?std::vector<std::size_t>():planPrefits(prefits);
 if(earlyTaken)for(auto&r:reductions){const auto t=core::Clock::now();r.ms=core::ms(t);} // pieces are already low-poly (LowPolyPieces.h): nothing to reduce
 else{const std::size_t tasks=reductions.size()+prefits.size();std::atomic<std::size_t>cursor{0};
  sep::parallel(tasks,1,sep::max_job_threads,[&](std::size_t,std::size_t,std::size_t){
   for(std::size_t k;(k=cursor.fetch_add(1))<tasks;){
    if(k<reductions.size()){auto&r=reductions[k];const auto t=core::Clock::now();
     r.ms=core::ms(t);} // pieces are already low-poly (LowPolyPieces.h): nothing to reduce
    else{auto&f=prefits[prefitOrder[k-reductions.size()]];const pc::detail::SerialScope serial;const auto t=core::Clock::now();
     try{f.fitted=fitCollider(*f.volume,f.capture,{},coarseCache);}catch(...){f.error=std::current_exception();}f.ms=core::ms(t);}}});}
 std::size_t prefitAt=0,reductionAt=0;
 // R34 (2026-10-17, body 19878: a grenade with 16 blades cut the body into 12 pieces; one piece's cut elements alone
 // needed more boxes than one native shell holds, the producer refused the piece ("boxes_per_shell") and the body never
 // separated). admitNativeShell can only hand whole, undivided elements their original shape. Here every element that
 // keeps a cut fit and lies over its share of the shell is fitted again with that share as its box limit: under the
 // standard certificates first, then the coarser ones the fitter accepts (50 mm corner gap, proxy ratio 6, box ratio
 // 12), and a part no certificate represents in its share gets the one-box fit (prepareTiny). The limits themselves do
 // not move: no element above boxes_per_element, no shell above boxes_per_shell. Returns a note for the summary.
 // speedfix: the box count of each element's standard fit in the piece loop (piece component, bone), for shellBudgetRefit.
 std::map<std::pair<std::uint32_t,std::uint16_t>,std::size_t> firstFitBoxes;
 const auto shellBudgetRefit=[&](SeparationPiece&piece,const auto&parts)->std::string{
  const auto boxes=[&]{std::size_t n=0;for(const auto&e:piece.elements)n+=e.boxes.size();return n;};
  const std::size_t budget=painter_collision_limits::boxes_per_shell;
  if(boxes()<=budget)return std::string();
  std::vector<SeparationElement*>cut;for(auto&e:piece.elements)if(!e.retain_original_shape&&!e.boxes.empty())cut.push_back(&e);
  if(cut.empty())return std::string();
  // The largest share a cut element may keep so that the sum fits the shell (elements under the share keep their fit).
  std::size_t share=budget;
  for(;share>1;--share){std::size_t total=0;for(const auto*e:cut)total+=std::min(e->boxes.size(),share);if(total<=budget)break;}
  std::string note=" shell_refit=";
  for(auto*e:cut){if(e->boxes.size()<=share)continue;
   const auto part=std::find_if(parts.begin(),parts.end(),[&](const auto&p){return p.bone==e->bone;});
   if(part==parts.end()||!(e->bone<frames.bones.size()))continue;
   pc::Capture capture;capture.accepted=stamp;capture.component=1;capture.anchor_bone=e->bone;
   capture.rest_to_anchor=frames.bones[e->bone].rest_to_anchor;capture.anchor_to_world=frames.bones[e->bone].anchor_to_rest;
   capture.rigid_pose_certified=true;
   pc::Limits limits;limits.max_boxes=std::min(share,limits.max_boxes);
   pc::Prepared fitted;const char*how="";
   // speedfix: with a share of 64 boxes or more the standard attempt makes the same leaves as the element's first fit
   // (the greedy cap and the merge target are min(64, max_boxes) either way); it fails, with collider_fit_certificate,
   // exactly when that fit has more boxes than the share - always so here (only such elements are refitted). Then the
   // coarse limits are tried at once, as the failure did. GFY_SF_REFIT_SKIP=0: the attempt runs.
   static const bool refitSkip=[]{const char*e=std::getenv("GFY_SF_REFIT_SKIP");return !e||std::atoi(e)!=0;}();
   const auto first=firstFitBoxes.find({piece.component,e->bone});
   const bool decided=refitSkip&&limits.max_boxes>=64&&first!=firstFitBoxes.end()&&first->second>limits.max_boxes;
   try{if(decided)throw std::runtime_error("collider_fit_certificate");fitted=fitCollider(part->volume,capture,limits,coarseCache);how="s";}
   catch(const std::bad_alloc&){throw;}
   catch(const std::exception&){
    limits.max_corner_gap_m=.05;limits.max_omitted_distance_m=.05;limits.max_proxy_ratio=6.;limits.max_box_ratio=12.;
    // speedfix: with a share of 64 or more the coarse attempt is the crew's coarse-limit fit (same leaves): accepted when its
    // box count is within the share, else the same certificate failure; its own failure is this attempt's failure
    const auto ahead=coarseAhead.find({piece.component,e->bone});
    try{if(refitSkip&&limits.max_boxes>=64&&ahead!=coarseAhead.end()){if(ahead->second.second)std::rethrow_exception(ahead->second.second);
      if(ahead->second.first.boxes.size()>limits.max_boxes)throw std::runtime_error("collider_fit_certificate");fitted=ahead->second.first;how="c";}
     else{fitted=fitCollider(part->volume,capture,limits,coarseCache);how="c";}}
    catch(const std::bad_alloc&){throw;}
    catch(const std::exception&){
     try{fitted=pc::prepareTiny(part->volume,capture);how="b";}
     catch(const std::bad_alloc&){throw;}
     catch(const std::exception&){how="";}}}
   if(!*how||fitted.boxes.empty()||fitted.boxes.size()>e->boxes.size()){note+=std::to_string(e->bone)+":"+std::to_string(e->boxes.size())+"kept,";continue;}
   note+=std::to_string(e->bone)+":"+std::to_string(e->boxes.size())+">"+std::to_string(fitted.boxes.size())+how+",";
   e->boxes.clear();e->mass_kg=fitted.mass_kg;e->com={fitted.com_anchor[0],fitted.com_anchor[1],fitted.com_anchor[2]};e->inertia=fitted.inertia_anchor_com;
   for(const auto&box:fitted.boxes)e->boxes.push_back({{box.center_anchor[0],box.center_anchor[1],box.center_anchor[2]},
     {box.half_extents_m[0],box.half_extents_m[1],box.half_extents_m[2]},box.orientation});
  }
  note+="total="+std::to_string(boxes());
  return note;};
 for(std::size_t surfaceIndex=0;surfaceIndex<surfaces.pieces.size();++surfaceIndex){auto&surface=surfaces.pieces[surfaceIndex];
  // A detached owner may update only its descendants. The complete virtual
  // parent was still joined and partitioned above to preserve skin ownership;
  // unrelated siblings need no new collision fitting or pose packet.
  if(wantedComponents&&!wantedComponents->count(surface.volume_component))continue;
  SeparationPiece piece;piece.component=surface.volume_component;piece.cells=pieceVolume.components[piece.component].cells;
  piece.ocular_remnant=piece.component<detachedEyes.size()&&detachedEyes[piece.component]!=0;
  piece.burst_chunk=burst_chunk::of(extras.lowpoly,pieceVolume,piece.component); // chunknosplash
  auto&parts=partitions.at(piece.component);const auto&elements=garment.component_elements.at(piece.component);
  piece.root_bone=root(elements,input.physical_parents);
  {need(reductionAt<reductions.size()&&reductions[reductionAt].surface==surfaceIndex,"separation_reduction_order");auto&r=reductions[reductionAt++];
   if(r.error)std::rethrow_exception(r.error);piece.mesh=std::move(r.mesh);pruneNegligibleSkinWeights(piece.mesh);
#ifdef GFY_COLLIDER_DIAGNOSTICS
   std::cerr<<"reduce_end component="<<piece.component<<" ms="<<r.ms<<" (parallel)"<<'\n';
#endif
  }
  for(const auto&part:parts){
   progress("collider_fit",piece.component,part.bone);
   SeparationElement element;element.bone=part.bone;element.cells=part.volume.cells;
   need(element.bone<frames.bones.size()&&frames.bones[element.bone].physical_element==element.bone,"separation_element_original_frame");
   // A detached head/hand is fitted even when its original element was not
   // itself damaged; do not give a severed head its old whole-body sphere.
   element.retain_original_shape=(parts.size()>1&&pieceOwners.at(part.bone)==1&&remainingByElement.at(part.bone)==map->original_cells.at(part.bone)&&!adopters.count(part.bone))|| // R94: parts = pre-fold element count
    keepsShape(piece.component,part.bone,parts.size()); // bodyjitter3: the remaining body keeps its elements, cut ones cut back
   if(element.retain_original_shape){ // bodyjitter3: the cut-backs it carries, and this cut's on the body's divided element
    if(extras.piece)for(const auto&t:extras.piece->trims)if(t.first==part.bone)element.trims.push_back(t.second);
    const auto cut=bodyCut.find({piece.component,part.bone}); // stumpshell: the cut-back of this piece's part
    if(cut!=bodyCut.end()){if(element.trims.size()>=4)element.trims.erase(element.trims.begin());element.trims.push_back(cut->second);
     const auto all=remainingByElement.find(part.bone);
     if(all!=remainingByElement.end()&&all->second)element.mass_scale=std::min(1.,double(part.volume.cells)/double(all->second));}}
   if(!element.retain_original_shape){
    pc::Capture capture;capture.accepted=stamp;capture.component=1;capture.anchor_bone=part.bone;
    capture.rest_to_anchor=frames.bones[part.bone].rest_to_anchor;capture.anchor_to_world=frames.bones[part.bone].anchor_to_rest;
    capture.rigid_pose_certified=true; // One original native physical element; articulated pieces retain separate elements.
    pc::Prepared fitted;
#ifdef GFY_COLLIDER_DIAGNOSTICS
    const auto fitStart=core::Clock::now();
    std::cerr<<"collider_start component="<<piece.component<<" bone="<<part.bone<<" cells="<<part.volume.cells<<'\n';
    if(const char* folder=std::getenv("GFY_COLLIDER_EXPORT_PATH")){
     std::ofstream dump(std::string(folder)+"/"+std::to_string(piece.component)+"_"+std::to_string(part.bone)+".bin",std::ios::binary);
     auto write=[&](const auto&v){dump.write(reinterpret_cast<const char*>(&v),sizeof(v));};
     write(part.volume.pitch);write(part.volume.origin);write(part.volume.size);write(capture.rest_to_anchor);
     write(std::uint64_t(part.volume.columns.size()));for(const auto&v:part.volume.columns)write(v);
     write(std::uint64_t(part.volume.spans.size()));for(const auto&v:part.volume.spans)write(v);
     write(part.volume.components[1]);
    }
#endif
    const Prefit*pre=nullptr;
    if(prefitAt<prefits.size()&&prefits[prefitAt].component==piece.component&&prefits[prefitAt].bone==part.bone&&prefits[prefitAt].volume==&part.volume)pre=&prefits[prefitAt++];
    try{if(pre){if(pre->error)std::rethrow_exception(pre->error);fitted=pre->fitted;}else fitted=fitCollider(part.volume,capture,{},coarseCache);
     firstFitBoxes[{piece.component,part.bone}]=fitted.boxes.size();} // speedfix: a standard fit (not the one-box fit below)
    catch(const std::bad_alloc&){throw;}
    catch(const std::exception&e){
     // A tiny sliver of a bone's region (<= 4096 cells) the box fitter cannot represent gets one minimum box.
     bool tiny=false;if(std::string(e.what()).find("collider_")!=std::string::npos){try{fitted=pc::prepareTiny(part.volume,capture);tiny=true;}catch(const std::bad_alloc&){throw;}catch(...){}}
     if(!tiny)throw std::runtime_error("separation_component="+std::to_string(piece.component)+" bone="+std::to_string(part.bone)+" cells="+std::to_string(part.volume.cells)+" "+e.what());}
#ifdef GFY_COLLIDER_DIAGNOSTICS
    std::cerr<<"collider_end boxes="<<fitted.boxes.size()<<" ms="<<(pre?pre->ms:core::ms(fitStart))<<(pre?" (parallel prefit)":"")<<'\n';
#endif
    element.mass_kg=fitted.mass_kg;element.com={fitted.com_anchor[0],fitted.com_anchor[1],fitted.com_anchor[2]};element.inertia=fitted.inertia_anchor_com;
    for(const auto&box:fitted.boxes)element.boxes.push_back({{box.center_anchor[0],box.center_anchor[1],box.center_anchor[2]},
      {box.half_extents_m[0],box.half_extents_m[1],box.half_extents_m[2]},box.orientation});
   }
   piece.elements.push_back(std::move(element));
  }
  fitOcularSurfaceCollider(piece,frames);
  piece.pose_anchors.assign(frames.bones.size(),65535);
  progress("pose_anchors",piece.component);
  std::vector<std::uint16_t> resolvedAnchors(frames.bones.size(),65535);
  std::map<std::uint16_t,std::uint16_t>certifiedSourceSkinAnchors;
  for(const auto&child:piece.mesh.children)for(const auto&vertex:child.vertices)for(unsigned w=0;w<4;++w)if(gfy_signed_hw::dependency(vertex.weight,w,child.skin_owned?0.:negligible_skin_weight)){
   const auto bone=vertex.bone[w];need(bone<frames.bones.size(),"separation_vertex_bone");
   const auto original=frames.bones[bone].physical_element;need(original>=0,"separation_weighted_unphysical_root");
   // The binding proof depends on the piece and original bone, not the
   // individual vertex. Reuse it across all vertices of the packed surface.
   if(resolvedAnchors[bone]!=65535){
    if(!elements.count(bone))piece.pose_anchors[bone]=resolvedAnchors[bone];continue;
   }
   std::uint16_t anchor;
   try{anchor=cutSkinAnchor(std::uint16_t(original),elements,input.physical_parents);}
   catch(const std::bad_alloc&){throw;}
   catch(const std::exception&e){
    const auto region=surface.constrained_anchors.find(bone);
    if(region!=surface.constrained_anchors.end()&&elements.count(region->second)&&constrainedSkinUses(piece.mesh,bone)){
     const auto certified=regionSkinFrontier(std::uint16_t(original),elements,input.physical_parents);
     need(certified==region->second,"separation_region_anchor_stale");anchor=certified;
     resolvedAnchors[bone]=anchor;if(!elements.count(bone))piece.pose_anchors[bone]=anchor;continue;
    }
    // R94: skin of a remnant folded before ownership is posed by its adopter, as admitNativeShell poses it.
    {const auto pf=preFolded.find(piece.component);if(pf!=preFolded.end()){const auto f=pf->second.find(std::uint16_t(original));
      if(f!=pf->second.end()&&elements.count(f->second)){anchor=f->second;resolvedAnchors[bone]=anchor;if(!elements.count(bone))piece.pose_anchors[bone]=anchor;continue;}}}
    auto [it,fresh]=certifiedSourceSkinAnchors.emplace(bone,65535);
    if(fresh)it->second=sourceSkinAnchor(asset,bindings,frames,bindingElements,sourceFaces,piece.mesh,elements,bone);
    anchor=it->second;
    if(anchor!=65535){resolvedAnchors[bone]=anchor;if(!elements.count(bone))piece.pose_anchors[bone]=anchor;continue;}
    // R54: a whole face or wound wall LowPolyPieces kept with this influence frozen (a detached thigh's trousers
    // weighted a few percent to the other thigh) poses it from the bone's single retained frontier, captured pose kept.
    if(region!=surface.constrained_anchors.end()){const auto frontier=regionSkinFrontier(std::uint16_t(original),elements,input.physical_parents);
     if(frontier!=65535&&elements.count(frontier)){anchor=frontier;resolvedAnchors[bone]=anchor;if(!elements.count(bone))piece.pose_anchors[bone]=anchor;continue;}}
    std::string why=e.what();why+=" skin_bone="+std::to_string(bone)+" child="+std::to_string(child.child_index)+" material="+std::to_string(child.material)+" point=";
    for(unsigned a=0;a<3;++a)why+=std::to_string(vertex.surface[a])+",";
    why+=" weights=";for(unsigned a=0;a<4;++a)why+=std::to_string(vertex.bone[a])+":"+std::to_string(vertex.weight[a])+",";
    const auto v=std::size_t(&vertex-child.vertices.data());
    for(std::size_t t=0;t<child.indices.size()/3;++t)if(child.indices[t*3]==v||child.indices[t*3+1]==v||child.indices[t*3+2]==v){why+=" face="+std::to_string(child.original_faces[t]);break;}
    throw std::runtime_error(why);
   }
   resolvedAnchors[bone]=anchor;if(!elements.count(bone))piece.pose_anchors[bone]=anchor;
  }
  for(const auto&a:input.accessories)if(bound_surface::at(vec(a.original_anchor),sgrid,pieceVolume)==piece.component){
   // An attachment bound to a folded element belongs to the element that carries its tissue now.
   std::uint16_t owner=a.physical_element;if(folds){const auto f=foldedAdopter(extras.piece->folded,owner);if(f!=65535)owner=f;}
   {const auto pf=preFolded.find(piece.component);if(pf!=preFolded.end()){const auto f=pf->second.find(owner);if(f!=pf->second.end())owner=f->second;}} // R94
   need(elements.count(owner),"separation_accessory_missing_physics_owner");piece.retained_accessories.push_back(a.child);
  }
  admitNativeShell(piece,pieceOwners,remainingByElement,map->original_cells,input.physical_parents,&adopters);
  {const auto refit=shellBudgetRefit(piece,parts);if(!refit.empty())out->lowpoly_summary+=refit;}
  if(piece.cells>largest){largest=piece.cells;out->source_component=piece.component;}
  gearBodyFlesh(asset,piece,frames); // gearbody: the merged gear this part holds, as flesh for the bullet test
  profMark("piece"); // speedfix read-out (GFY_SEP_PROF)
  if(!piece.mesh.tissue_field)piece.mesh.tissue_field=asset.tissue_field; // low-poly pieces keep the closure field
  // Continuation seed (PieceSeed.h; prepareSnapshot adds the grid, topology, history and torn stations): the piece's
  // foreign marks, the source faces its skin shows, and the bone each skin weight of a later wound uses on it.
  {auto seed=std::make_shared<PieceSeed>();seed->root_bone=piece.root_bone;seed->gore=extras.gore;
   if(extras.semantic_groups){seed->components.push_back(piece.component);for(const auto&g:*extras.semantic_groups)if(g.second==piece.component)seed->components.push_back(g.first);
    if(seed->components.size()==1)seed->components.clear();else std::sort(seed->components.begin(),seed->components.end());}
   if(surfaceIndex<pieceMarks.size())seed->marks=std::move(pieceMarks[surfaceIndex]);
   seed->faces.assign(asset.source.f.size(),0);std::array<char,64> used{};
   for(const auto&child:piece.mesh.children){
    for(const auto&v:child.vertices)for(unsigned w=0;w<4;++w)if(gfy_signed_hw::dependency(v.weight,w,child.skin_owned?0.:negligible_skin_weight)&&v.bone[w]<64)used[v.bone[w]]=1;
    if(child.material||child.original_faces.size()*3!=child.indices.size())continue;
    for(const auto f:child.original_faces)if(f!=~std::uint32_t(0))if(const auto*g=sourceFaces.find(child.material_source_child,f))seed->faces[*g]=1;}
   std::set<std::uint16_t> held;for(const auto&e:piece.elements)held.insert(e.bone);
   for(const auto&e:piece.elements){if(!e.retain_original_shape&&e.bone<64)seed->fitted_elements|=std::uint64_t(1)<<e.bone; // bodyjitter3
    if(e.retain_original_shape)for(const auto&t:e.trims)seed->trims.emplace_back(e.bone,t);}
   // Folded physical ownership: the parent seed's folds, plus every element of this component that admission gave no
   // rigid body (its pose anchor names the adopter). Stored flattened along the chain.
   if(extras.piece)seed->folded=extras.piece->folded;
   for(const auto&part:parts)if(part.bone<64&&!held.count(part.bone)){const auto a=part.bone<piece.pose_anchors.size()?piece.pose_anchors[part.bone]:std::uint16_t(65535);
    if(a!=65535&&a<64&&held.count(a))seed->folded[part.bone]=a;}
   // Flatten, and keep only folds whose adopter is an element this piece holds: a fold cannot point at a body that
   // left with another piece or at a malformed chain (foldedAdopter rejects those).
   {const auto raw=seed->folded;for(unsigned b=0;b<64;++b){const auto a=foldedAdopter(raw,std::uint16_t(b));seed->folded[b]=(a!=65535&&held.count(a))?a:std::uint16_t(65535);}}
   for(unsigned b=0;b<64;++b){std::uint16_t r=std::uint16_t(b);
    if(!held.count(r)&&!used[b]){r=b<piece.pose_anchors.size()?piece.pose_anchors[b]:std::uint16_t(65535);
     if(r==65535&&b<frames.bones.size()&&frames.bones[b].physical_element>=0){
      try{r=cutSkinAnchor(std::uint16_t(frames.bones[b].physical_element),held,input.physical_parents);}catch(const std::bad_alloc&){throw;}catch(const std::exception&){r=65535;}}
     if(r==65535||!held.count(r))r=piece.root_bone;}
    seed->render_bone[b]=r;}
   piece.seed=std::move(seed);}
  out->pieces.push_back(std::move(piece));
 }
#ifdef GFY_COLLIDER_DIAGNOSTICS
 phase("physics_and_pose");
#endif
 lap(separation_phase_colliders_and_pose);
 // R34: prepare() phase times for the log (remaining connectivity / original connectivity / compare / surface bindings /
 // original physics map / physical partitions / surface partition / colliders and pose).
 {char text[200];std::snprintf(text,sizeof(text)," phase_ms=%.0f/%.0f/%.0f/%.0f/%.0f/%.0f/%.0f/%.0f",
   out->phase_ms[separation_phase_remaining_connectivity],out->phase_ms[separation_phase_original_connectivity],
   out->phase_ms[separation_phase_compare_connectivity],out->phase_ms[separation_phase_surface_bindings],
   out->phase_ms[separation_phase_original_physics_map],out->phase_ms[separation_phase_physical_partitions],
   out->phase_ms[separation_phase_surface_partition],out->phase_ms[separation_phase_colliders_and_pose]);
  out->lowpoly_summary+=text;}
 if(extras.pieces_volume_out)*extras.pieces_volume_out=volumeShared;
 if(extras.grid_out)*extras.grid_out=activeGrid==&working?std::make_shared<const Grid>(std::move(working)):std::shared_ptr<const Grid>();
 need(out->pieces.size()>=(wantedComponents?1u:2u)&&out->source_component,"separation_missing_body_pieces");return out;
}
// Timing of one production separation job (filled only when requested). `output`
// is the prepare() result, kept so a failing continuation can still be inspected.
struct SnapshotTiming {double prepare_ms{},continuation_ms{};std::vector<double>piece_ms;std::shared_ptr<SeparationOutput>output;};
// The complete body of DamageSnapshot::prepare_separation: prepare() and then one
// component_continuation per piece. The offline harness calls this same function,
// so it always measures the production continuation loop. `geometry` is the
// snapshot's published wound mesh (null when unmodified). Defined in
// ComponentSession_impl.h, after component_detail::make.
inline std::shared_ptr<SeparationOutput> prepareSnapshot(const std::shared_ptr<const Asset>&asset,
    const std::shared_ptr<const Grid>&grid,const gfy_packed::Mesh*geometry,const Stamp&stamp,
    const SeparationInput&input,const SeparationCheckpoint&checkpoint,SnapshotTiming*timing=nullptr,const PieceContext*context=nullptr);
}
// gearpar: console painter_sep_threads (0 automatic, 1..16 fixed; separation/ThreadBudget.h).
void set_sep_threads(int threads)noexcept{sep::setJobThreadsOverride(threads);}
std::shared_ptr<const SeparationOutput>DamageSnapshot::prepare_separation(const SeparationInput&input,const SeparationCheckpoint&checkpoint)const noexcept{
 const sep::JobThreads jobCap; // gearpar: the job's thread cap (the offline harness calls this function directly)
 AllocationScope scope;const auto started=core::Clock::now();std::shared_ptr<SeparationOutput>out;
 // The failure report exists before any work, so reporting never allocates
 // (short reasons). Null only when even this could not be allocated.
 std::shared_ptr<SeparationOutput>failure;
 try{failure=std::make_shared<SeparationOutput>();failure->stamp=impl_->stamp;}catch(...){return {};}
 // Memory admission: a separation starts only with commit for it; the owner
 // retries later (the accepted wound itself is unaffected).
 {const auto policy=memory_policy();const auto m=query_memory();
  if(policy.enabled&&m.valid&&m.avail_commit<policy.separation_bytes+policy.commit_margin){
   memoryCounters.separation_refused.fetch_add(1,std::memory_order_relaxed);failure->error=lowMemoryText;failure->worker_ms=core::ms(started);return failure;}}
 const separation_detail::PieceContext context{impl_->piece.get(),impl_->history,impl_->sever,impl_->gore};
 // cutends v2 (2026-11-16): a separated piece's birth mesh is its publication before this damage - the burst split's give-back reads
 // it (HeadBurstPieces.h hostBefore), on this thread, for this prepare only
 struct CeHostBefore {explicit CeHostBefore(const gfy_packed::Mesh*m){separation_detail::mask_pieces3::head_burst_pieces::hostBefore()=m;}~CeHostBefore(){separation_detail::mask_pieces3::head_burst_pieces::hostBefore()=nullptr;}} ceHostBefore(impl_->piece_mesh.get());
 try{out=separation_detail::prepareSnapshot(impl_->asset,impl_->grid,impl_->geometry?&impl_->geometry->mesh:impl_->piece_mesh.get(),
                                            impl_->stamp,input,checkpoint,nullptr,&context);}
 catch(const std::bad_alloc&){out.reset();memoryCounters.separation_oom.fetch_add(1,std::memory_order_relaxed);failure->error=outOfMemoryText;out=failure;}
 catch(const std::exception&e){out.reset();try{failure->error=e.what();}catch(...){failure->error=outOfMemoryText;}out=failure;}
 catch(...){out.reset();try{failure->error="unknown_separation_failure";}catch(...){failure->error=outOfMemoryText;}out=failure;}
 out->worker_ms=core::ms(started);return out;
}
struct SeparationPreparation::Impl {Stamp stamp;SeparationJobProgress progress;mutable std::mutex mutex;std::shared_ptr<const SeparationOutput>output;};
SeparationPreparation::SeparationPreparation(std::shared_ptr<Impl>state):impl_(std::move(state)){}
SeparationPreparation::~SeparationPreparation(){cancel();}
const Stamp&SeparationPreparation::stamp()const noexcept{return impl_->stamp;}
SeparationProgress SeparationPreparation::progress()const{return impl_->progress.progress();}
void SeparationPreparation::cancel(){impl_->progress.cancel();}
std::shared_ptr<const SeparationOutput>SeparationPreparation::completed()const{
 std::lock_guard<std::mutex>lock(impl_->mutex);return impl_->output;
}
std::shared_ptr<SeparationPreparation>prepare_separation_async(std::shared_ptr<const DamageSnapshot>snapshot,SeparationInput input)noexcept{
 AllocationScope scope;
 try{need(bool(snapshot),"separation_snapshot_missing");
  struct Service {
   // failure: preallocated at submission, used if the worker cannot even
   // allocate its own report (see DamageSnapshot::prepare_separation).
   struct Job {std::shared_ptr<const DamageSnapshot>snapshot;SeparationInput input;std::shared_ptr<SeparationPreparation::Impl>state;std::shared_ptr<SeparationOutput>failure;};
   // A blast splits many bodies at once (grenade into 10 NPCs: 21 of 27 splits refused by a 2-deep, single-thread
   // queue): three workers (each still bounded by sep::max_job_threads helpers), 32 queued (ten bodies can queue an
   // early split and a later one each). GFY_SEP_SERVICE_THREADS overrides the worker count (1-16, measurements).
   const unsigned threads=[]{const char*e=std::getenv("GFY_SEP_SERVICE_THREADS");const int n=e?std::atoi(e):0;return n>=1&&n<=16?unsigned(n):3u;}();
   const std::size_t depth=32;
   std::mutex mutex;std::condition_variable wake;std::deque<Job>jobs;bool stop{};
   std::vector<std::shared_ptr<SeparationPreparation::Impl>>active;std::vector<std::thread>workers;
   Service(){active.resize(threads);for(unsigned i=0;i<threads;++i)workers.emplace_back([this,i]{run(i);});}
   void quit()noexcept{{std::lock_guard<std::mutex>lock(mutex);stop=true;for(auto&a:active)if(a)a->progress.cancel();jobs.clear();}wake.notify_all();for(auto&w:workers)if(w.joinable())w.join();} // quitfix: running separations are cancelled, queued ones dropped
   ~Service(){quit();}
   void run(unsigned slot){painterThreadEntry();for(;;){Job job;{std::unique_lock<std::mutex>lock(mutex);wake.wait(lock,[&]{return stop||!jobs.empty();});if(stop)return;job=std::move(jobs.front());jobs.pop_front();}
    painterThreadEntry(); // the engine may install its policy after this thread started
    {std::lock_guard<std::mutex>lock(mutex);active[slot]=job.state;if(stop)active[slot]->progress.cancel();}
    if(!job.state->progress.canceled()){
     // P1.5: the job's cancel flag is visible to every stage (and to the helper
     // threads of sep::parallel); P1.3: warm builds yield while it runs.
     const sep::CancelScope cancelScope(job.state->progress.cancel_flag());const sep::ActiveJob activeJob;
     std::shared_ptr<const SeparationOutput>result=job.snapshot->prepare_separation(job.input,[&](const char*phase,std::uint32_t component,std::uint16_t bone){job.state->progress.checkpoint(phase,component,bone);});
     if(!result){job.failure->error=outOfMemoryText;result=job.failure;}
     job.state->progress.finish();
     std::lock_guard<std::mutex>lock(job.state->mutex);job.state->output=std::move(result);
    }
    {std::lock_guard<std::mutex>lock(mutex);active[slot].reset();}}}
  };
  // Caches first, so they outlive the service thread at shutdown.
  (void)separation_detail::pristineCache();(void)separation_detail::mapCache();
  (void)separation_detail::tileBaseCache();(void)separation_detail::decodedCache();
  static Service& service=::gfy_live::core::worker_quit::immortal<Service>(0);auto state=std::make_shared<SeparationPreparation::Impl>();state->stamp=snapshot->stamp(); // quitfix: never destroyed
  auto failure=std::make_shared<SeparationOutput>();failure->stamp=state->stamp;
  auto handle=std::shared_ptr<SeparationPreparation>(new SeparationPreparation(state));
  {std::lock_guard<std::mutex>lock(service.mutex);
   service.jobs.erase(std::remove_if(service.jobs.begin(),service.jobs.end(),[](const auto&job){return job.state->progress.canceled();}),service.jobs.end());
   need(service.jobs.size()<service.depth,"separation_queue_full");service.jobs.push_back({std::move(snapshot),std::move(input),state,std::move(failure)});}
  service.wake.notify_one();return handle;
 }catch(const std::bad_alloc&){fail(outOfMemoryText);return {};}catch(const std::exception&e){fail(e.what());return {};}catch(...){fail("unknown_separation_queue_failure");return {};}
}
}
