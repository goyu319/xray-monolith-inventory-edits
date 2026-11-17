#pragma once
#define GFY_ORIGINAL_ACCESSORY_SURFACE_V1 1
#include <unordered_set> // gfyspeed2
// Decoded original exterior attachments are independent of body wound skin.
// Never reconstruct an owned piece from the whole source: its published faces
// are the only permitted inputs on subsequent cuts.
namespace gfy_live { namespace original_accessory {
constexpr double publicationPruneThreshold=1e-6;
struct Triangle {core::JoinedTriangle surface;std::array<Vec,3> anchors;bool skin_clip{};};
inline std::set<std::uint16_t> children(const bound_surface::ExtraSurface&extra){
 return {extra.face_children.begin(),extra.face_children.end()};
}
inline Triangle original(const bound_surface::ExtraSurface&extra,std::uint32_t f){
 Triangle out;
 for(unsigned k=0;k<3;++k){const auto index=extra.source.f[f][k];core::V v;v.p=extra.source.v[index].p;v.b[k]=1;
  out.surface.v[k]=core::interpolateVertex(extra.source,f,v);out.anchors[k]=extra.bindings.vertices[index].point;}
 return out;
}
// Prove that a source-edge point can serialize to this exact packed position.
// Bounds are IEEE float rounding cells, not an enlarged geometric tolerance.
inline bool publishedEdgeParameter(const Vec&a,const Vec&b,const Vec&p,double&parameter){
 double lo=0.,hi=1.;const Vec edge=b-a;
 for(unsigned q=0;q<3;++q){
  const double target=(&p.x)[q],start=(&a.x)[q],step=(&edge.x)[q];
  const float f=float(target);if(!std::isfinite(f)||double(f)!=target)return false;
  const float prev=std::nextafter(f,-std::numeric_limits<float>::infinity());
  const float next=std::nextafter(f, std::numeric_limits<float>::infinity());
  if(!std::isfinite(prev)||!std::isfinite(next))return false;
  const double low=(double(prev)+target)*.5,high=(double(next)+target)*.5;
  if(step==0.){if(float(start)!=f)return false;continue;}
  const double t0=(low-start)/step,t1=(high-start)/step;
  lo=std::max(lo,std::min(t0,t1));hi=std::min(hi,std::max(t0,t1));if(lo>hi)return false;
 }
 const double length2=dot(edge,edge);if(!(length2>0.)||!std::isfinite(length2))return false;
 const double closest=std::clamp(dot(p-a,edge)/length2,lo,hi);
 for(const double t:{closest,lo+(hi-lo)*.5}){
  const Vec point=a+edge*t;bool exact=true;
  for(unsigned q=0;q<3;++q){const float actual=float((&point.x)[q]),expected=float((&p.x)[q]);exact&=std::memcmp(&actual,&expected,sizeof(float))==0;}
  if(exact){parameter=t;return true;}
 }
 return false;
}
inline std::array<double,3> barycentric(const Source&source,std::uint32_t f,const gfy_packed::Vertex&row){
 need(f<source.f.size(),"surface_binding_source_face");Triangle ref;
 for(unsigned k=0;k<3;++k){const auto index=source.f[f][k];need(index<source.v.size(),"surface_binding_source_vertex");
  core::V v;v.p=source.v[index].p;v.b[k]=1.;ref.surface.v[k]=core::interpolateVertex(source,f,v);}
 // Exact original corners also support authored zero-area triangles and UV seams.
 for(unsigned k=0;k<3;++k){auto packed=core::packing::vertex(ref.surface.v[k]);
  bool same=!std::memcmp(&packed,&row,sizeof(row));
  // Publications apply the existing negligible-weight pruning. Recognize its
  // exact packed corner too, including authored zero-area faces.
  const bool preserveHW=gfy_signed_hw::valid4(packed.weight,true);double kept=0,pruned=0;for(unsigned j=0;j<4;++j){if(!preserveHW&&packed.weight[j]>0&&packed.weight[j]<=publicationPruneThreshold){pruned+=packed.weight[j];packed.weight[j]=0;}kept+=packed.weight[j];}
  if(pruned&&kept>.5){for(auto&w:packed.weight)w=float(w/kept);gfy_packed::canonical_weights(packed);same|=!std::memcmp(&packed,&row,sizeof(row));}
  if(same){std::array<double,3>b{};b[k]=1;return b;}}
 const auto&a=ref.surface.v[0].p;const Vec u=ref.surface.v[1].p-a,v=ref.surface.v[2].p-a;
 const Vec p{row.surface[0],row.surface[1],row.surface[2]},d=p-a;
 const double uu=dot(u,u),uv=dot(u,v),vv=dot(v,v),du=dot(d,u),dv=dot(d,v),den=uu*vv-uv*uv;
 need(std::isfinite(den)&&den>1e-30,"accessory_repeat_degenerate_provenance");
 const double b1=(du*vv-dv*uv)/den,b2=(dv*uu-du*uv)/den;
 std::array<double,3>b{{1-b1-b2,b1,b2}};bool outside=false,rangeValid=true;
 for(const auto x:b){need(std::isfinite(x),"accessory_repeat_barycentric_range");rangeValid&=x>=-1e-4&&x<=1.0001;outside|=x<0||x>1;}
 if(outside){
  // Float publication can move an edge point outside a skinny source triangle.
  // Keep the existing range rule; an otherwise rejected point is admitted only
  // with an exact float round-trip witness on this immutable source boundary.
  // The original packed row and the spatial check below remain unchanged.
  double closest=std::numeric_limits<double>::infinity();std::array<double,3> edgeBary{};
  for(unsigned i=0;i<3;++i){const unsigned j=(i+1)%3;
   const auto&start=ref.surface.v[i].p;const Vec edge=ref.surface.v[j].p-start;
   const double size2=dot(edge,edge);if(size2==0)continue;
   double t=std::clamp(dot(p-start,edge)/size2,0.,1.);
   if(!rangeValid&&!publishedEdgeParameter(start,ref.surface.v[j].p,p,t))continue;
   const Vec delta=start+edge*t-p;
   const double distance2=dot(delta,delta);if(distance2<closest){closest=distance2;edgeBary={};edgeBary[i]=1-t;edgeBary[j]=t;}}
  need(rangeValid||std::isfinite(closest),"accessory_repeat_barycentric_range");
  b=edgeBary;
 }
 need(length(a*b[0]+ref.surface.v[1].p*b[1]+ref.surface.v[2].p*b[2]-p)<2e-6,"accessory_repeat_source_plane");
 return b;
}
inline std::array<double,3> barycentric(const bound_surface::ExtraSurface&extra,std::uint32_t f,const gfy_packed::Vertex&row){
 return barycentric(extra.source,f,row);
}
inline core::JoinedVertex unpack(const gfy_packed::Vertex&row,std::uint32_t face,const std::array<double,3>&bary){
 core::JoinedVertex v;v.p={row.surface[0],row.surface[1],row.surface[2]};v.normal={row.surface[3],row.surface[4],row.surface[5]};
 v.tangent={row.surface[6],row.surface[7],row.surface[8]};v.binormal={row.surface[9],row.surface[10],row.surface[11]};
 v.uv={row.surface[12],row.surface[13]};v.sourceFace=face;v.sourceBary=bary;
 double sum=0;for(unsigned k=0;k<4;++k){need(row.bone[k]<64&&std::isfinite(row.weight[k])&&(row.weight[k]>=0||gfy_signed_hw::valid4(row.weight,true)),"accessory_repeat_skin");v.w[row.bone[k]]+=row.weight[k];sum+=row.weight[k];}
 need(std::abs(sum-1)<2e-6,"accessory_repeat_skin_sum");if(gfy_signed_hw::valid4(row.weight,true)){v.hardware_corner=true;v.hardware_ids=row.bone;v.hardware_w=row.weight;}else {const auto largest=std::max_element(v.w.begin(),v.w.end());*largest+=1-sum;}
 return v;
}
// gfyrefuse safety net (2026-11-15): what the last collect() on this thread left out instead of refusing (LiveSeparationWorker
// reports it in its summary)
struct CollectDrops {std::size_t repeats{},overCovered{};
 // ownersafe (2026-11-15): published gear triangles that could not be read back (left out; `why` = the first reason) and gear
 // corners read on their source triangle's nearest point (logged as unread / reread_nearest)
 std::size_t unread{},clamped{};std::string why;
 void bad(const char*reason){if(!unread++)why=reason?reason:"unknown";}};
inline thread_local CollectDrops lastCollectDrops{};
inline bool nearestBary(const std::array<Vec,3>&P,const Vec&p,std::array<double,3>&out){
 if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return false;
 double best=std::numeric_limits<double>::infinity();
 auto take=[&](const std::array<double,3>&b){const Vec q=P[0]*b[0]+P[1]*b[1]+P[2]*b[2];const double d=length(q-p);if(std::isfinite(d)&&d<best){best=d;out=b;}};
 const Vec u=P[1]-P[0],v=P[2]-P[0],d=p-P[0];const double uu=dot(u,u),uv=dot(u,v),vv=dot(v,v),du=dot(d,u),dv=dot(d,v),den=uu*vv-uv*uv;
 if(std::isfinite(den)&&den>1e-30){const double b1=(du*vv-dv*uv)/den,b2=(dv*uu-du*uv)/den;if(b1>=0&&b2>=0&&b1+b2<=1)take({1-b1-b2,b1,b2});}
 for(unsigned i=0;i<3;++i){const unsigned j=(i+1)%3;const Vec e=P[j]-P[i];const double s=dot(e,e);std::array<double,3> b{};
  if(s>0&&std::isfinite(s)){const double t=std::clamp(dot(p-P[i],e)/s,0.,1.);b[i]=1-t;b[j]=t;}else b[i]=1;take(b);}
 return best<2e-6;}
template<class Exact> inline bool rereadBary(Exact&&exact,const Source&source,std::uint32_t f,const gfy_packed::Vertex&row,std::array<double,3>&b){
 try{if(painter_ownersafe_fault("collect_bary"))throw std::runtime_error("accessory_repeat_barycentric_range");b=exact();return true;}
 catch(const std::bad_alloc&){throw;}
 catch(const std::exception&e){
  if(f<source.f.size()){const auto&F=source.f[f];
   if(F[0]<source.v.size()&&F[1]<source.v.size()&&F[2]<source.v.size()&&
      nearestBary({source.v[F[0]].p,source.v[F[1]].p,source.v[F[2]].p},Vec{row.surface[0],row.surface[1],row.surface[2]},b)){++lastCollectDrops.clamped;return true;}}
  lastCollectDrops.bad(e.what());return false;}}
inline bool rereadSkin(const gfy_packed::Vertex&row,std::uint32_t f,const std::array<double,3>&bary,core::JoinedVertex&v){
 try{if(painter_ownersafe_fault("collect_skin"))throw std::runtime_error("accessory_repeat_skin");v=unpack(row,f,bary);return true;}
 catch(const std::bad_alloc&){throw;}
 catch(const std::exception&e){lastCollectDrops.bad(e.what());return false;}}
inline std::vector<Triangle> collect(const Asset&asset,const bound_surface::ExtraSurface&extra,
                                    const gfy_packed::Mesh&published,bool owned){
 need(extra.bindings.source==asset.source_hash&&extra.source.bones==asset.source.bones&&
      extra.bindings.vertices.size()==extra.source.v.size()&&extra.face_children.size()==extra.source.f.size()&&
      extra.face_indices.size()==extra.source.f.size(),"accessory_collect_identity");
 std::vector<Triangle> result;lastCollectDrops={}; /* gfyrefuse safety net */if(extra.source.f.empty())return result;
 const auto slots=children(extra);std::set<std::uint16_t> body(asset.face_children.begin(),asset.face_children.end());
 for(auto child:slots)need(child<asset.original_children,"accessory_body_child_range");
 std::set<std::pair<std::uint16_t,std::uint32_t>> bodyFaces;
 for(std::size_t f=0;f<asset.face_children.size();++f)bodyFaces.emplace(asset.face_children[f],asset.face_indices[f]);
 for(std::size_t f=0;f<extra.face_children.size();++f)
  need(!bodyFaces.count({extra.face_children[f],extra.face_indices[f]}),"accessory_body_face_overlap");
 std::map<std::pair<std::uint16_t,std::uint32_t>,std::uint32_t> faces;
 for(std::uint32_t f=0;f<extra.source.f.size();++f)
  need(faces.emplace(std::make_pair(extra.face_children[f],extra.face_indices[f]),f).second,"accessory_duplicate_source_face");
 std::set<std::uint16_t> existingChildren;std::vector<double> covered(extra.source.f.size(),0.);
 using Row=std::array<unsigned char,sizeof(gfy_packed::Vertex)>;using TriangleRows=std::array<Row,3>;
 std::set<std::pair<std::uint32_t,TriangleRows>> seen;
 for(const auto&child:published.children){if(child.material||!slots.count(child.material_source_child))continue;
  if(child.indices.empty()&&!body.count(child.material_source_child))existingChildren.insert(child.material_source_child);
  // ownersafe: a child whose rows do not match its faces is left out whole; its slot counts as published (never regrown from the source)
  if(!(child.indices.size()%3==0&&child.original_faces.size()*3==child.indices.size())||painter_ownersafe_fault("collect_child")){
   existingChildren.insert(child.material_source_child);lastCollectDrops.bad("accessory_repeat_provenance");continue;}
  for(std::size_t t=0;t<child.original_faces.size();++t){
   const auto key=std::make_pair(child.material_source_child,child.original_faces[t]);
   const auto found=faces.find(key);
   if(found==faces.end()){if(!bodyFaces.count(key))lastCollectDrops.bad("accessory_repeat_unknown_face");continue;} // ownersafe: left out
   existingChildren.insert(child.material_source_child);
   Triangle triangle;triangle.skin_clip=child.skin_clip;const auto f=found->second;TriangleRows rows;bool readable=true;
   for(unsigned k=0;k<3&&readable;++k){if(!(child.indices[3*t+k]<child.vertices.size())){lastCollectDrops.bad("accessory_repeat_vertex_index");readable=false;break;}
    const auto&row=child.vertices[child.indices[3*t+k]];std::memcpy(rows[k].data(),&row,sizeof(row));std::array<double,3> bary{};
    if(!rereadBary([&]{return barycentric(extra,f,row);},extra.source,f,row,bary)||!rereadSkin(row,f,bary,triangle.surface.v[k])){readable=false;break;}
    Vec anchor{};
    for(unsigned q=0;q<3;++q)anchor=anchor+extra.bindings.vertices[extra.source.f[f][q]].point*bary[q];
    triangle.anchors[k]=anchor;}
   if(!readable)continue; // ownersafe: this published gear triangle is left out (counted; the first reason is logged)
   std::sort(rows.begin(),rows.end());
   // gfyrefuse safety net: a repeat of an earlier triangle of this source face (identical rows) draws the same pixels again - it is
   // left out; a triangle that would cover its source face more than once is left out (that face is drawn already). Both used to
   // refuse the whole publication, and every later hit of the body with it.
   if(!seen.emplace(f,rows).second){++lastCollectDrops.repeats;continue;}
   const auto&a=triangle.surface.v[0].sourceBary;const auto&b=triangle.surface.v[1].sourceBary;const auto&c=triangle.surface.v[2].sourceBary;
   const double area=std::abs((b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1]));
   if(covered[f]+area>1.0001){++lastCollectDrops.overCovered;continue;}
   covered[f]+=area;
   result.push_back(std::move(triangle));}
 }
 // A whole-body wound may publish only the newly clipped ocular child while
 // untouched hats/gear remain native originals. Missing children can use
 // their exact source; an explicit empty child is a permanent removal marker.
 // Owned fragments NEVER recover absent geometry from the whole source.
 if(!owned){result.reserve(result.size()+extra.source.f.size());for(std::uint32_t f=0;f<extra.source.f.size();++f)
  if(!existingChildren.count(extra.face_children[f]))result.push_back(original(extra,f));}
 return result;
}
// ---- gfyspeed2 (2026-11-15): collect() without its per-call rebuilds ----------------------------------------------------------
// collect() rebuilt, on every hit and split, the 88k body-face set and the accessory face map, and for each published gear vertex
// barycentric() re-interpolated and re-packed the three source corners of its face. CollectMemo keeps exactly those values for one
// (asset, accessory surface): the reference's tables (the same contents as sorted vectors) and every source face's packed corners
// (packing::vertex of interpolateVertex of the corner, as barycentric() computes them; a face with an index out of range or a
// corner whose packing throws keeps the reference statements). It is built only when the reference's table checks pass.
// collectFast() runs collect()'s statements in collect()'s order with those values: the same triangles, decisions and errors.
// `wanted` (clipOcularAccessories): a triangle of another child is only checked (vertex index, barycentric, skin: the same need()
// calls in the same order) and not built; repeat / coverage decisions are per source face and a face belongs to one child, so the
// wanted children's decisions are unchanged; originals are made for wanted children only.
struct CollectMemo {
 std::set<std::uint16_t> slots;std::vector<std::uint8_t> bodyChild; // children(extra); the body's children by id
 std::vector<std::uint64_t> bodyKeys; // sorted (child<<32|index) of the body faces (collect()'s bodyFaces)
 std::vector<std::pair<std::uint64_t,std::uint32_t>> faces; // sorted (child<<32|index) -> accessory source face (collect()'s faces)
 std::vector<gfy_packed::Vertex> corners;std::vector<std::uint8_t> cornerOk; // per source face: its 3 packed exact corners
 std::size_t bytes()const{return sizeof(*this)+slots.size()*40+bodyChild.size()+bodyKeys.size()*8+faces.size()*16+corners.size()*sizeof(gfy_packed::Vertex)+cornerOk.size();}};
inline std::uint64_t faceKey(std::uint16_t child,std::uint32_t index){return (std::uint64_t(child)<<32)|index;}
inline std::shared_ptr<const CollectMemo> buildCollectMemo(const Asset&asset,const bound_surface::ExtraSurface&extra){
 if(extra.source.f.empty()||extra.face_children.size()!=extra.source.f.size()||extra.face_indices.size()!=extra.source.f.size()||
    asset.face_children.size()!=asset.face_indices.size())return {};
 auto m=std::make_shared<CollectMemo>();m->slots=children(extra);
 for(auto child:m->slots)if(!(child<asset.original_children))return {}; // the reference refuses: no memo
 m->bodyChild.assign(65536,0);for(const auto c:asset.face_children)m->bodyChild[c]=1;
 m->bodyKeys.reserve(asset.face_children.size());
 for(std::size_t f=0;f<asset.face_children.size();++f)m->bodyKeys.push_back(faceKey(asset.face_children[f],asset.face_indices[f]));
 std::sort(m->bodyKeys.begin(),m->bodyKeys.end());m->bodyKeys.erase(std::unique(m->bodyKeys.begin(),m->bodyKeys.end()),m->bodyKeys.end());
 m->faces.reserve(extra.source.f.size());
 for(std::uint32_t f=0;f<extra.source.f.size();++f){const auto key=faceKey(extra.face_children[f],extra.face_indices[f]);
  if(std::binary_search(m->bodyKeys.begin(),m->bodyKeys.end(),key))return {}; // a body face: the reference refuses
  m->faces.emplace_back(key,f);}
 std::sort(m->faces.begin(),m->faces.end());
 for(std::size_t k=1;k<m->faces.size();++k)if(m->faces[k].first==m->faces[k-1].first)return {}; // a duplicate: the reference refuses
 const auto&source=extra.source;m->corners.resize(source.f.size()*3);m->cornerOk.assign(source.f.size(),0);
 for(std::uint32_t f=0;f<source.f.size();++f){bool ok=true;
  for(unsigned k=0;k<3&&ok;++k){const auto index=source.f[f][k];if(!(index<source.v.size())){ok=false;break;}
   try{core::V v;v.p=source.v[index].p;v.b[k]=1.;m->corners[3*f+k]=core::packing::vertex(core::interpolateVertex(source,f,v));}catch(...){ok=false;}}
  m->cornerOk[f]=ok?1:0;}
 return m;}
// barycentric() with the memo: the reference's corner loop on the memo's packed corners (ref.surface.v[k].p is the corner's
// source position: interpolateVertex returns v.p), then the reference's statements.
inline std::array<double,3> barycentric(const Source&source,std::uint32_t f,const gfy_packed::Vertex&row,const CollectMemo*memo){
 if(!memo||!(f<memo->cornerOk.size())||!memo->cornerOk[f]||!(f<source.f.size()))return barycentric(source,f,row);
 for(unsigned k=0;k<3;++k){auto packed=memo->corners[3*f+k];
  bool same=!std::memcmp(&packed,&row,sizeof(row));
  const bool preserveHW=gfy_signed_hw::valid4(packed.weight,true);double kept=0,pruned=0;for(unsigned j=0;j<4;++j){if(!preserveHW&&packed.weight[j]>0&&packed.weight[j]<=publicationPruneThreshold){pruned+=packed.weight[j];packed.weight[j]=0;}kept+=packed.weight[j];}
  if(pruned&&kept>.5){for(auto&w:packed.weight)w=float(w/kept);gfy_packed::canonical_weights(packed);same|=!std::memcmp(&packed,&row,sizeof(row));}
  if(same){std::array<double,3>b{};b[k]=1;return b;}}
 const Vec P[3]={source.v[source.f[f][0]].p,source.v[source.f[f][1]].p,source.v[source.f[f][2]].p};
 const auto&a=P[0];const Vec u=P[1]-a,v=P[2]-a;
 const Vec p{row.surface[0],row.surface[1],row.surface[2]},d=p-a;
 const double uu=dot(u,u),uv=dot(u,v),vv=dot(v,v),du=dot(d,u),dv=dot(d,v),den=uu*vv-uv*uv;
 need(std::isfinite(den)&&den>1e-30,"accessory_repeat_degenerate_provenance");
 const double b1=(du*vv-dv*uv)/den,b2=(dv*uu-du*uv)/den;
 std::array<double,3>b{{1-b1-b2,b1,b2}};bool outside=false,rangeValid=true;
 for(const auto x:b){need(std::isfinite(x),"accessory_repeat_barycentric_range");rangeValid&=x>=-1e-4&&x<=1.0001;outside|=x<0||x>1;}
 if(outside){
  double closest=std::numeric_limits<double>::infinity();std::array<double,3> edgeBary{};
  for(unsigned i=0;i<3;++i){const unsigned j=(i+1)%3;
   const auto&start=P[i];const Vec edge=P[j]-start;
   const double size2=dot(edge,edge);if(size2==0)continue;
   double t=std::clamp(dot(p-start,edge)/size2,0.,1.);
   if(!rangeValid&&!publishedEdgeParameter(start,P[j],p,t))continue;
   const Vec delta=start+edge*t-p;
   const double distance2=dot(delta,delta);if(distance2<closest){closest=distance2;edgeBary={};edgeBary[i]=1-t;edgeBary[j]=t;}}
  need(rangeValid||std::isfinite(closest),"accessory_repeat_barycentric_range");
  b=edgeBary;
 }
 need(length(a*b[0]+P[1]*b[1]+P[2]*b[2]-p)<2e-6,"accessory_repeat_source_plane");
 return b;
}
// unpack()'s checks only (the same need() calls, the same weight sum)
inline void unpackCheck(const gfy_packed::Vertex&row){double sum=0;
 for(unsigned k=0;k<4;++k){need(row.bone[k]<64&&std::isfinite(row.weight[k])&&(row.weight[k]>=0||gfy_signed_hw::valid4(row.weight,true)),"accessory_repeat_skin");sum+=row.weight[k];}
 need(std::abs(sum-1)<2e-6,"accessory_repeat_skin_sum");}
// collect()'s repeat key (source face, sorted vertex rows) in a hash set: the same membership as its std::set.
struct CollectSeenKey {std::uint32_t f{};std::array<std::array<unsigned char,sizeof(gfy_packed::Vertex)>,3> rows{};
 bool operator==(const CollectSeenKey&o)const{return f==o.f&&rows==o.rows;}};
struct CollectSeenHash {std::size_t operator()(const CollectSeenKey&k)const{std::uint64_t h=0x9E3779B97F4A7C15ull^k.f;
 for(const auto&r:k.rows)for(std::size_t i=0;i+8<=r.size();i+=8){std::uint64_t w;std::memcpy(&w,r.data()+i,8);h=(h^w)*0x100000001B3ull;h^=h>>29;}
 return std::size_t(h);}};
inline std::vector<Triangle> collectFast(const Asset&asset,const bound_surface::ExtraSurface&extra,const gfy_packed::Mesh&published,bool owned,
                                        const CollectMemo&memo,const std::set<std::uint16_t>*wanted=nullptr){
 need(extra.bindings.source==asset.source_hash&&extra.source.bones==asset.source.bones&&
      extra.bindings.vertices.size()==extra.source.v.size()&&extra.face_children.size()==extra.source.f.size()&&
      extra.face_indices.size()==extra.source.f.size(),"accessory_collect_identity");
 std::vector<Triangle> result;lastCollectDrops={};if(extra.source.f.empty())return result;
 const auto&slots=memo.slots;
 std::set<std::uint16_t> existingChildren;std::vector<double> covered(extra.source.f.size(),0.);
 std::unordered_set<CollectSeenKey,CollectSeenHash> seen;
 for(const auto&child:published.children){if(child.material||!slots.count(child.material_source_child))continue;
  if(child.indices.empty()&&!memo.bodyChild[child.material_source_child])existingChildren.insert(child.material_source_child);
  if(!(child.indices.size()%3==0&&child.original_faces.size()*3==child.indices.size())||painter_ownersafe_fault("collect_child")){ // ownersafe: as collect()
   existingChildren.insert(child.material_source_child);lastCollectDrops.bad("accessory_repeat_provenance");continue;}
  const bool build=!wanted||wanted->count(child.material_source_child);
  for(std::size_t t=0;t<child.original_faces.size();++t){
   const auto key=faceKey(child.material_source_child,child.original_faces[t]);
   const auto found=std::lower_bound(memo.faces.begin(),memo.faces.end(),std::make_pair(key,std::uint32_t(0)));
   if(found==memo.faces.end()||found->first!=key){if(!std::binary_search(memo.bodyKeys.begin(),memo.bodyKeys.end(),key))lastCollectDrops.bad("accessory_repeat_unknown_face");continue;}
   existingChildren.insert(child.material_source_child);const auto f=found->second;
   // ownersafe: a child the eye re-clip does not use is no longer read back (helmet, backpack: those reads decided nothing; they
   // could only refuse the hit, and cost a source-triangle solve per corner on every hit of a continued part)
   if(!build)continue;
   Triangle triangle;triangle.skin_clip=child.skin_clip;CollectSeenKey rows;rows.f=f;bool readable=true;
   for(unsigned k=0;k<3&&readable;++k){if(!(child.indices[3*t+k]<child.vertices.size())){lastCollectDrops.bad("accessory_repeat_vertex_index");readable=false;break;}
    const auto&row=child.vertices[child.indices[3*t+k]];std::memcpy(rows.rows[k].data(),&row,sizeof(row));std::array<double,3> bary{};
    if(!rereadBary([&]{return barycentric(extra.source,f,row,&memo);},extra.source,f,row,bary)||!rereadSkin(row,f,bary,triangle.surface.v[k])){readable=false;break;}
    Vec anchor{};
    for(unsigned q=0;q<3;++q)anchor=anchor+extra.bindings.vertices[extra.source.f[f][q]].point*bary[q];
    triangle.anchors[k]=anchor;}
   if(!readable)continue; // ownersafe: left out (counted)
   std::sort(rows.rows.begin(),rows.rows.end());
   if(!seen.insert(rows).second){++lastCollectDrops.repeats;continue;}
   const auto&a=triangle.surface.v[0].sourceBary;const auto&b=triangle.surface.v[1].sourceBary;const auto&c=triangle.surface.v[2].sourceBary;
   const double area=std::abs((b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1]));
   if(covered[f]+area>1.0001){++lastCollectDrops.overCovered;continue;}
   covered[f]+=area;
   result.push_back(std::move(triangle));}
 }
 if(!owned){result.reserve(result.size()+extra.source.f.size());for(std::uint32_t f=0;f<extra.source.f.size();++f)
  if(!existingChildren.count(extra.face_children[f])&&(!wanted||wanted->count(extra.face_children[f])))result.push_back(original(extra,f));}
 return result;
}
inline void append(gfy_packed::Mesh&mesh,const core::JoinResult&joined,const bound_surface::ExtraSurface&extra,
                   std::uint16_t originalChildren,bool skinClip=false){
 if(joined.triangles.empty())return;
 auto packed=core::packing::packMulti(joined,extra.face_children,extra.face_indices,originalChildren);
 unsigned next=originalChildren;std::set<std::uint16_t> used;
 for(const auto&child:mesh.children){need(used.insert(child.child_index).second,"accessory_existing_duplicate_slot");next=std::max(next,unsigned(child.child_index)+1);}
 std::size_t count=0;
 for(auto&child:packed.children){if(child.indices.empty())continue;
  need(!child.material,"accessory_unexpected_cavity");
  if(child.child_index<originalChildren&&used.count(child.child_index)){
   const auto existing=std::find_if(mesh.children.begin(),mesh.children.end(),[&](const auto&c){return c.child_index==child.child_index;});
   need(existing!=mesh.children.end()&&!existing->material&&existing->material_source_child==child.material_source_child,"accessory_original_slot_collision");
   // Body and gear can share an authored material child, but never a face.
   if(existing->skin_clip==skinClip){
    const std::set<std::uint32_t> already(existing->original_faces.begin(),existing->original_faces.end());
    for(auto face:child.original_faces)need(!already.count(face),"accessory_duplicate_output_face");
   }
   need(next<gfy_packed::max_child_slots,"accessory_appended_slot_limit");child.child_index=std::uint16_t(next++);
  }else if(child.child_index>=originalChildren){need(next<gfy_packed::max_child_slots,"accessory_appended_slot_limit");child.child_index=std::uint16_t(next++);}
  child.skin_clip=skinClip;
  need(used.insert(child.child_index).second,"accessory_duplicate_output_child");
  count+=child.indices.size()/3;mesh.children.push_back(std::move(child));}
 need(count==joined.triangles.size(),"accessory_output_triangle_conservation");
 mesh.reduced_skin_vertices+=packed.reduced_skin_vertices;
 mesh.max_discarded_skin_weight=std::max(mesh.max_discarded_skin_weight,packed.max_discarded_skin_weight);
}
// ---- gfyspeed3 (2026-11-15): the separation's gear triangles without materializing the unchanged ones ------------------------
// collect() turned every published gear triangle (~47k on a GFY body) into a 2 KB JoinedTriangle; the partition copied the intact
// ones into its result, the result was copied again into the per-piece lists and append() packed them back into rows. A
// LightTriangle keeps what collect() derives for a triangle (face, rows, barycentrics, anchors, corner positions);
// materialize() builds collect()'s Triangle from it by the same unpack() / original() calls. A kept triangle is packed from its
// rows: packMulti() computes packing::vertex() of collect()'s vertex, i.e. of unpack(row) - a function of the row bytes.
struct LightTriangle {std::uint32_t f{};bool skin_clip{},original{};std::array<const gfy_packed::Vertex*,3> row{};
 std::array<std::array<double,3>,3> bary{};std::array<Vec,3> anchors{},p{};
 std::array<gfy_packed::Vertex,3> packed{};std::array<double,3> discarded{};bool packOk{};};
inline Triangle materialize(const bound_surface::ExtraSurface&extra,const LightTriangle&l){
 if(l.original)return original(extra,l.f);
 Triangle triangle;triangle.skin_clip=l.skin_clip;
 for(unsigned k=0;k<3;++k){triangle.surface.v[k]=unpack(*l.row[k],l.f,l.bary[k]);triangle.anchors[k]=l.anchors[k];}
 return triangle;}
// collect() on a published surface whose every gear row reads back exactly (the case where collect()'s ownersafe rules - a row
// read on the nearest point of its source triangle, a triangle / child left out - never act): its statements in its order, the
// same triangles, repeat / coverage decisions and drop counts; the triangles are not built. Returns false - nothing decided - as
// soon as any of those rules would act (a child whose rows do not match its faces, an unknown face, a vertex index out of range,
// a barycentric or skin read that does not succeed exactly): the caller then runs collect() itself.
// Three passes: (1) collect()'s walk over the children and faces, serial; (2) each walked triangle's vertex work - index check,
// barycentric, unpack's checks, anchors: a pure function of the triangle - side by side (sep::parallel); (3) the walk again in its
// order with the repeat / coverage decisions.
inline bool collectLight(const Asset&asset,const bound_surface::ExtraSurface&extra,const gfy_packed::Mesh&published,bool owned,
                         const CollectMemo&memo,std::vector<LightTriangle>&result){
 need(extra.bindings.source==asset.source_hash&&extra.source.bones==asset.source.bones&&
      extra.bindings.vertices.size()==extra.source.v.size()&&extra.face_children.size()==extra.source.f.size()&&
      extra.face_indices.size()==extra.source.f.size(),"accessory_collect_identity");
 result.clear();lastCollectDrops={};if(extra.source.f.empty())return true;
 const auto&slots=memo.slots;
 std::set<std::uint16_t> existingChildren;std::vector<double> covered(extra.source.f.size(),0.);
 // blowoutrest (2026-11-16): a corner the exact rule cannot place is read as collect() reads it (rereadBary: its source triangle's
 // nearest point within 2e-6 m, counted as clamped; else the triangle is left out, counted, the first reason kept) - the light
 // path no longer gives up for one such row (GFY V6, every blowout of a continued body: 1 row of 35,880 sent it to collect(),
 // 110-160 ms, and its partition, 75-95 ms). The items are reserved; the repeat test (same source face and sorted rows as an
 // earlier triangle) is decided on a hash and an exact comparison instead of a node set: the same repeats.
 // GFY_OWNERSAFE_FAULT (test switches, which this path does not call): every such row still sends the call to collect().
 struct Item {const gfy_packed::Child*child{};std::size_t t{};std::exception_ptr error;bool unread{},odd{};unsigned clamped{};std::string why;LightTriangle l;};
 std::vector<Item> items;
 {std::size_t total=0;for(const auto&child:published.children)if(!child.material&&slots.count(child.material_source_child))total+=child.original_faces.size();items.reserve(total);}
 const bool walked=[&]{for(const auto&child:published.children){if(child.material||!slots.count(child.material_source_child))continue;
  if(child.indices.empty()&&!memo.bodyChild[child.material_source_child])existingChildren.insert(child.material_source_child);
  if(!(child.indices.size()%3==0&&child.original_faces.size()*3==child.indices.size()))return false; // collect() leaves the child out
  for(std::size_t t=0;t<child.original_faces.size();++t){
   const auto key=faceKey(child.material_source_child,child.original_faces[t]);
   const auto found=std::lower_bound(memo.faces.begin(),memo.faces.end(),std::make_pair(key,std::uint32_t(0)));
   if(found==memo.faces.end()||found->first!=key){
    if(!std::binary_search(memo.bodyKeys.begin(),memo.bodyKeys.end(),key))return false; // collect() counts an unread face
    continue;}
   existingChildren.insert(child.material_source_child);
   items.emplace_back();auto&item=items.back();item.child=&child;item.t=t;item.l.f=found->second;item.l.skin_clip=child.skin_clip;}}
  return true;}();
 if(!walked){result.clear();return false;}
 {std::atomic<std::size_t> cursor{0};const std::size_t n=items.size();
  sep::parallel(n,256,sep::max_job_threads,[&](std::size_t,std::size_t,std::size_t){
   for(std::size_t j;(j=cursor.fetch_add(1))<n;){auto&item=items[j];const auto&child=*item.child;const auto t=item.t;auto&l=item.l;const auto f=l.f;
    try{for(unsigned k=0;k<3;++k){if(!(child.indices[3*t+k]<child.vertices.size())){item.unread=true;item.why="accessory_repeat_vertex_index";break;}
      const auto&row=child.vertices[child.indices[3*t+k]];std::array<double,3> bary{};
      try{bary=barycentric(extra.source,f,row,&memo);}
      catch(const std::bad_alloc&){throw;}
      catch(const std::exception&e){bool nearest=false; // collect()'s rereadBary
       if(f<extra.source.f.size()){const auto&F=extra.source.f[f];
        if(F[0]<extra.source.v.size()&&F[1]<extra.source.v.size()&&F[2]<extra.source.v.size())
         nearest=nearestBary({extra.source.v[F[0]].p,extra.source.v[F[1]].p,extra.source.v[F[2]].p},Vec{row.surface[0],row.surface[1],row.surface[2]},bary);}
       if(!nearest){item.unread=true;item.why=e.what();break;}
       ++item.clamped;}
      try{unpackCheck(row);}catch(const std::bad_alloc&){throw;}catch(const std::exception&e){item.unread=true;item.why=e.what();break;} // collect()'s rereadSkin
      l.row[k]=&row;l.bary[k]=bary;l.p[k]=Vec{row.surface[0],row.surface[1],row.surface[2]};Vec anchor{};
      for(unsigned q=0;q<3;++q)anchor=anchor+extra.bindings.vertices[extra.source.f[f][q]].point*bary[q];
      l.anchors[k]=anchor;}}
    catch(const std::bad_alloc&){item.error=std::current_exception();}
    catch(...){item.odd=true;}}});} // a read the rules above do not describe: collect() decides
 for(const auto&item:items)if(item.error)std::rethrow_exception(item.error);
 static const bool faultSwitches=std::getenv("GFY_OWNERSAFE_FAULT")!=nullptr;
 for(const auto&item:items)if(item.odd||(faultSwitches&&(item.unread||item.clamped))){result.clear();return false;}
 // repeats: a triangle whose (source face, sorted rows) an earlier read triangle has (collect()'s seen set, the same answers)
 std::vector<char> repeat(items.size(),0);
 // gearpar (2026-11-16): the keys are built on the separation's threads (sep::parallel, at most 4 with the job's own; each key is a
 // function of its own triangle) and the repeats found in walk order in an open-addressed table of the first triangle of each
 // distinct key (full key comparison on every equal hash): a triangle is a repeat exactly when an earlier read triangle has its key.
 // GFY_GP_SERIAL=1: the former serial build and sort.
 static const bool gpSerial=std::getenv("GFY_GP_SERIAL")!=nullptr;
 if(!gpSerial){std::vector<CollectSeenKey> keys(items.size());std::vector<std::uint64_t> hashes(items.size(),0);const CollectSeenHash hash;
  sep::parallel(items.size(),1024,sep::max_job_threads,[&](std::size_t first,std::size_t last,std::size_t){
   for(std::size_t j=first;j<last;++j){const auto&item=items[j];if(item.unread)continue;auto&rows=keys[j];rows.f=item.l.f;
    for(unsigned k=0;k<3;++k)std::memcpy(rows.rows[k].data(),item.l.row[k],sizeof(gfy_packed::Vertex));
    std::sort(rows.rows.begin(),rows.rows.end());hashes[j]=std::uint64_t(hash(rows));}});
  std::size_t size=16;while(size<2*items.size())size<<=1;std::vector<std::uint32_t> table(size,0);const std::size_t mask=size-1;
  for(std::size_t j=0;j<items.size();++j){if(items[j].unread)continue;const std::uint64_t h=hashes[j];
   for(std::size_t at=std::size_t(h^(h>>32))&mask;;at=(at+1)&mask){const std::uint32_t s=table[at];
    if(!s){table[at]=std::uint32_t(j+1);break;}
    if(hashes[s-1]==h&&keys[s-1]==keys[j]){repeat[j]=1;break;}}}}
 else{std::vector<CollectSeenKey> keys(items.size());std::vector<std::pair<std::uint64_t,std::uint32_t>> order;order.reserve(items.size());const CollectSeenHash hash;
  for(std::size_t j=0;j<items.size();++j){const auto&item=items[j];if(item.unread)continue;auto&rows=keys[j];rows.f=item.l.f;
   for(unsigned k=0;k<3;++k)std::memcpy(rows.rows[k].data(),item.l.row[k],sizeof(gfy_packed::Vertex));
   std::sort(rows.rows.begin(),rows.rows.end());order.emplace_back(std::uint64_t(hash(rows)),std::uint32_t(j));}
  std::sort(order.begin(),order.end());
  for(std::size_t a=0;a<order.size();){std::size_t b=a;while(b<order.size()&&order[b].first==order[a].first)++b;
   for(std::size_t i=a+1;i<b;++i)for(std::size_t e=a;e<i;++e)if(keys[order[e].second]==keys[order[i].second]){repeat[order[i].second]=1;break;}
   a=b;}}
 result.reserve(items.size());
 for(std::size_t j=0;j<items.size();++j){auto&item=items[j];
  lastCollectDrops.clamped+=item.clamped;
  if(item.unread){lastCollectDrops.bad(item.why.c_str());continue;} // collect(): left out, counted
  const auto&l=item.l;const auto f=l.f;
  if(repeat[j]){++lastCollectDrops.repeats;continue;}
  const auto&a=l.bary[0];const auto&b=l.bary[1];const auto&c=l.bary[2];
  const double area=std::abs((b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1]));
  if(covered[f]+area>1.0001){++lastCollectDrops.overCovered;continue;}
  covered[f]+=area;
  result.push_back(l);}
 if(!owned){result.reserve(result.size()+extra.source.f.size());for(std::uint32_t f=0;f<extra.source.f.size();++f)
  if(!existingChildren.count(extra.face_children[f])){LightTriangle l;l.f=f;l.original=true;
   for(unsigned k=0;k<3;++k){const auto index=extra.source.f[f][k];l.p[k]=extra.source.v[index].p;l.anchors[k]=extra.bindings.vertices[index].point;}
   result.push_back(l);}}
 return true;
}
// packMulti()'s rows of a kept triangle: packing::vertex() of collect()'s vertices (unpack(row) / the original corner). A row whose
// packing throws is left to packGear(), which materializes the triangle and packs it there, as packMulti() did.
inline void lightPack(const bound_surface::ExtraSurface&extra,LightTriangle&l){
 try{for(unsigned k=0;k<3;++k){l.discarded[k]=0;
   if(l.original){core::V v;v.p=extra.source.v[extra.source.f[l.f][k]].p;v.b[k]=1.;l.packed[k]=core::packing::vertex(core::interpolateVertex(extra.source,l.f,v),&l.discarded[k]);}
   else l.packed[k]=core::packing::vertex(unpack(*l.row[k],l.f,l.bary[k]),&l.discarded[k]);}
  l.packOk=true;}catch(...){l.packOk=false;}}
// The triangles of one piece and clip, in the reference's order: (light index, ~0u) = a kept light triangle; (~0u, i) = tris[i].
struct GearMesh {std::vector<std::pair<std::uint32_t,std::uint32_t>> order;std::vector<core::JoinedTriangle> tris;};
// core::packing::packMulti() over a GearMesh: a kept light triangle is collect()'s triangle (material 0, every corner on face f)
// with its packed rows; everything else is packMulti()'s statements.
inline gfy_packed::Mesh packGear(const GearMesh&gear,const std::vector<LightTriangle>&light,const bound_surface::ExtraSurface&extra,
    const std::vector<std::uint16_t>& faceChildren,const std::vector<std::uint32_t>& faceIndices,std::uint16_t originalChildren){
    using namespace core::packing;
    packingRequire(faceChildren.size()==faceIndices.size()&&!faceChildren.empty(),"Source child map");
    packingRequire(originalChildren>0&&originalChildren<gfy_packed::max_child_slots,"Original child slots");
    using Group=std::pair<unsigned,std::uint16_t>;
    std::map<Group,std::vector<std::size_t>> groups;
    for(auto child:faceChildren){packingRequire(child<originalChildren,"Source child outside original");groups[{0,child}];}
    const auto fallback=faceChildren.front();
    for(std::size_t e=0;e<gear.order.size();++e){const auto&o=gear.order[e];unsigned material=0;std::uint16_t source=fallback;
        if(o.first!=~std::uint32_t(0)){const auto face=light[o.first].f;packingRequire(face<faceChildren.size(),"Exterior source face");source=faceChildren[face];}
        else{const auto&triangle=gear.tris[o.second];material=triangle.material;
            if(!triangle.material){const auto face=triangle.v[0].sourceFace;
                packingRequire(face<faceChildren.size()&&triangle.v[1].sourceFace==face&&triangle.v[2].sourceFace==face,"Exterior source face");
                source=faceChildren[face];}}
        packingRequire(material<64,"Interior material slot limit");groups[{material,source}].push_back(e);
    }
    gfy_packed::Mesh out;unsigned next=originalChildren;
    auto appendSlot=[&]{packingRequire(next<gfy_packed::max_child_slots,"Runtime child slots exhausted");return std::uint16_t(next++);};
    VertexIndex vertices;
    for(const auto& group:groups){
        const auto& entries=group.second;
        gfy_packed::Child child;child.material=group.first.first;child.material_source_child=group.first.second;
        child.child_index=child.material?appendSlot():group.first.second;
        vertices.reset(std::min<std::size_t>(3*entries.size(),65535));
        auto reserve=[&](std::size_t remaining){remaining=std::min<std::size_t>(remaining,65535);child.indices.reserve(3*remaining);child.original_faces.reserve(remaining);};
        reserve(entries.size());
        auto flush=[&]{packingRequire(child.child_index<gfy_packed::max_child_slots,"Runtime child slots exhausted");out.children.push_back(std::move(child));};
        for(std::size_t t=0;t<entries.size();++t){const auto&o=gear.order[entries[t]];
            const LightTriangle*l=o.first!=~std::uint32_t(0)?&light[o.first]:nullptr;
            core::JoinedTriangle made;const core::JoinedTriangle*triangle=nullptr;
            if(!l)triangle=&gear.tris[o.second];
            else if(!l->packOk){made=materialize(extra,*l).surface;triangle=&made;} // packs - and refuses - as packMulti() did
            std::array<gfy_packed::Vertex,3> rows;std::array<std::uint64_t,3> hashes;std::array<long,3> found;std::array<double,3> discarded{};unsigned fresh=0;
            for(unsigned k=0;k<3;++k){
                if(triangle)rows[k]=vertex(triangle->v[k],&discarded[k]);else{rows[k]=l->packed[k];discarded[k]=l->discarded[k];}
                hashes[k]=vertexHash(rows[k]);found[k]=vertices.find(child.vertices,rows[k],hashes[k]);if(found[k]<0)++fresh;}
            if(child.vertices.size()+fresh>65535||child.indices.size()/3>=65535){flush();child={};child.material=group.first.first;child.material_source_child=group.first.second;child.child_index=appendSlot();vertices.clear();found.fill(-1);reserve(entries.size()-t);}
            for(unsigned k=0;k<3;++k){
                if(found[k]>=0){child.indices.push_back(std::uint16_t(found[k]));continue;}
                const auto [index,inserted]=vertices.insert(child.vertices,rows[k],hashes[k]);
                if(inserted){out.reduced_skin_vertices+=discarded[k]>1e-12;out.max_discarded_skin_weight=std::max(out.max_discarded_skin_weight,discarded[k]);}
                child.indices.push_back(index);
            }
            child.original_faces.push_back(child.material?~std::uint32_t(0):faceIndices[l?l->f:triangle->v[0].sourceFace]);
        }
        flush();
    }
    return out;
}
// gearpar (2026-11-16): packGear() with its groups packed side by side. Each group (material, source child) is packed into its own
// children by packGear's own statements (its own vertex index, as packGear resets its index per group); what depends on the
// groups before it - the appended slot numbers (appendSlot) and the first failure - is replayed afterwards in packGear's order: per
// group its slot requests, flushes and a failure as they happened, then the next group. The same children, slots, counts and the
// same first error. On the separation's threads (sep::parallel, at most 4 with the job's own). GFY_GP_SERIAL=1: packGear().
inline gfy_packed::Mesh packGearPar(const GearMesh&gear,const std::vector<LightTriangle>&light,const bound_surface::ExtraSurface&extra,
    const std::vector<std::uint16_t>& faceChildren,const std::vector<std::uint32_t>& faceIndices,std::uint16_t originalChildren){
    using namespace core::packing;
    packingRequire(faceChildren.size()==faceIndices.size()&&!faceChildren.empty(),"Source child map");
    packingRequire(originalChildren>0&&originalChildren<gfy_packed::max_child_slots,"Original child slots");
    using Group=std::pair<unsigned,std::uint16_t>;
    std::map<Group,std::vector<std::size_t>> groups;
    for(auto child:faceChildren){packingRequire(child<originalChildren,"Source child outside original");groups[{0,child}];}
    const auto fallback=faceChildren.front();
    for(std::size_t e=0;e<gear.order.size();++e){const auto&o=gear.order[e];unsigned material=0;std::uint16_t source=fallback;
        if(o.first!=~std::uint32_t(0)){const auto face=light[o.first].f;packingRequire(face<faceChildren.size(),"Exterior source face");source=faceChildren[face];}
        else{const auto&triangle=gear.tris[o.second];material=triangle.material;
            if(!triangle.material){const auto face=triangle.v[0].sourceFace;
                packingRequire(face<faceChildren.size()&&triangle.v[1].sourceFace==face&&triangle.v[2].sourceFace==face,"Exterior source face");
                source=faceChildren[face];}}
        packingRequire(material<64,"Interior material slot limit");groups[{material,source}].push_back(e);
    }
    enum : std::uint8_t {Slot,Flush,Fail};
    struct Out {std::vector<gfy_packed::Child> children;std::vector<std::pair<std::uint8_t,std::uint32_t>> events;std::size_t reduced{};double maxDiscarded{};std::exception_ptr error;};
    std::vector<const std::pair<const Group,std::vector<std::size_t>>*> list;list.reserve(groups.size());for(const auto&g:groups)list.push_back(&g);
    std::vector<Out> outs(list.size());std::atomic<std::size_t> cursor{0};
    sep::parallel(list.size(),1,sep::max_job_threads,[&](std::size_t,std::size_t,std::size_t){
     for(std::size_t gi;(gi=cursor.fetch_add(1))<list.size();){const auto&group=*list[gi];auto&go=outs[gi];
      try{
        const auto& entries=group.second;
        gfy_packed::Child child;child.material=group.first.first;child.material_source_child=group.first.second;
        if(child.material){go.events.push_back({Slot,0u});child.child_index=0;}else child.child_index=group.first.second;
        VertexIndex vertices;
        vertices.reset(std::min<std::size_t>(3*entries.size(),65535));
        auto reserve=[&](std::size_t remaining){remaining=std::min<std::size_t>(remaining,65535);child.indices.reserve(3*remaining);child.original_faces.reserve(remaining);};
        reserve(entries.size());
        auto flush=[&]{go.events.push_back({Flush,std::uint32_t(go.children.size())});go.children.push_back(std::move(child));};
        for(std::size_t t=0;t<entries.size();++t){const auto&o=gear.order[entries[t]];
            const LightTriangle*l=o.first!=~std::uint32_t(0)?&light[o.first]:nullptr;
            core::JoinedTriangle made;const core::JoinedTriangle*triangle=nullptr;
            if(!l)triangle=&gear.tris[o.second];
            else if(!l->packOk){made=materialize(extra,*l).surface;triangle=&made;} // packs - and refuses - as packMulti() did
            std::array<gfy_packed::Vertex,3> rows;std::array<std::uint64_t,3> hashes;std::array<long,3> found;std::array<double,3> discarded{};unsigned fresh=0;
            for(unsigned k=0;k<3;++k){
                if(triangle)rows[k]=vertex(triangle->v[k],&discarded[k]);else{rows[k]=l->packed[k];discarded[k]=l->discarded[k];}
                hashes[k]=vertexHash(rows[k]);found[k]=vertices.find(child.vertices,rows[k],hashes[k]);if(found[k]<0)++fresh;}
            if(child.vertices.size()+fresh>65535||child.indices.size()/3>=65535){flush();child={};child.material=group.first.first;child.material_source_child=group.first.second;
                go.events.push_back({Slot,std::uint32_t(go.children.size())});child.child_index=0;vertices.clear();found.fill(-1);reserve(entries.size()-t);}
            for(unsigned k=0;k<3;++k){
                if(found[k]>=0){child.indices.push_back(std::uint16_t(found[k]));continue;}
                const auto [index,inserted]=vertices.insert(child.vertices,rows[k],hashes[k]);
                if(inserted){go.reduced+=discarded[k]>1e-12;go.maxDiscarded=std::max(go.maxDiscarded,discarded[k]);}
                child.indices.push_back(index);
            }
            child.original_faces.push_back(child.material?~std::uint32_t(0):faceIndices[l?l->f:triangle->v[0].sourceFace]);
        }
        flush();}
      catch(...){go.error=std::current_exception();go.events.push_back({Fail,0u});}}});
    gfy_packed::Mesh out;unsigned next=originalChildren;
    auto appendSlot=[&]{packingRequire(next<gfy_packed::max_child_slots,"Runtime child slots exhausted");return std::uint16_t(next++);};
    for(auto&go:outs){
        for(const auto&ev:go.events){
            if(ev.first==Slot){const auto slot=appendSlot();if(ev.second<go.children.size())go.children[ev.second].child_index=slot;} // (a group that failed before its flush)
            else if(ev.first==Flush){auto&child=go.children[ev.second];packingRequire(child.child_index<gfy_packed::max_child_slots,"Runtime child slots exhausted");out.children.push_back(std::move(child));}
            else std::rethrow_exception(go.error);}
        out.reduced_skin_vertices+=go.reduced;out.max_discarded_skin_weight=std::max(out.max_discarded_skin_weight,go.maxDiscarded);
    }
    return out;
}
// append() with packGear()
inline void appendGear(gfy_packed::Mesh&mesh,const GearMesh&gear,const std::vector<LightTriangle>&light,const bound_surface::ExtraSurface&extra,
                       std::uint16_t originalChildren,bool skinClip=false){
 if(gear.order.empty())return;
 static const bool gpSerialPack=std::getenv("GFY_GP_SERIAL")!=nullptr; // gearpar
 auto packed=gpSerialPack?packGear(gear,light,extra,extra.face_children,extra.face_indices,originalChildren):
  packGearPar(gear,light,extra,extra.face_children,extra.face_indices,originalChildren);
 unsigned next=originalChildren;std::set<std::uint16_t> used;
 for(const auto&child:mesh.children){need(used.insert(child.child_index).second,"accessory_existing_duplicate_slot");next=std::max(next,unsigned(child.child_index)+1);}
 std::size_t count=0;
 for(auto&child:packed.children){if(child.indices.empty())continue;
  need(!child.material,"accessory_unexpected_cavity");
  if(child.child_index<originalChildren&&used.count(child.child_index)){
   const auto existing=std::find_if(mesh.children.begin(),mesh.children.end(),[&](const auto&c){return c.child_index==child.child_index;});
   need(existing!=mesh.children.end()&&!existing->material&&existing->material_source_child==child.material_source_child,"accessory_original_slot_collision");
   if(existing->skin_clip==skinClip){
    const std::set<std::uint32_t> already(existing->original_faces.begin(),existing->original_faces.end());
    for(auto face:child.original_faces)need(!already.count(face),"accessory_duplicate_output_face");
   }
   need(next<gfy_packed::max_child_slots,"accessory_appended_slot_limit");child.child_index=std::uint16_t(next++);
  }else if(child.child_index>=originalChildren){need(next<gfy_packed::max_child_slots,"accessory_appended_slot_limit");child.child_index=std::uint16_t(next++);}
  child.skin_clip=skinClip;
  need(used.insert(child.child_index).second,"accessory_duplicate_output_child");
  count+=child.indices.size()/3;mesh.children.push_back(std::move(child));}
 need(count==gear.order.size(),"accessory_output_triangle_conservation");
 mesh.reduced_skin_vertices+=packed.reduced_skin_vertices;
 mesh.max_discarded_skin_weight=std::max(mesh.max_discarded_skin_weight,packed.max_discarded_skin_weight);
}
// Decode only sparse non-kept pages overlapping a clipped attachment's bounds.
// This carries shader removal from the published piece, including marks added by
// previous piece wounds that are not part of the original birth seed's marks.
inline void publishedMarks(const painter_detail::DamageField&field,const Grid&grid,const Cell&lo,const Cell&hi,
                           std::map<Cell,core::Tile>&out){
 need(field.available&&std::abs(double(field.pitch)-grid.pitch)<=grid.pitch*1e-6,"accessory_clip_field_lattice");
 if(!field.size[0]||!field.size[1]||!field.size[2])return;
 Cell base;for(unsigned a=0;a<3;++a){const double index=(double(field.origin[a])-grid.origin[a])/(8*grid.pitch);
  (&base.x)[a]=int(std::llround(index));need(std::abs(index-double((&base.x)[a]))<1e-3,"accessory_clip_field_origin");}
 const auto pages=std::uint64_t(field.pages[0])*field.pages[1]*field.pages[2];
 need(pages<=field.page.size()&&pages+painter_detail::DamageField::headerWords<=field.page.size(),"accessory_clip_field_pages");
 const int lower[3]={std::max(0,lo.x-base.x),std::max(0,lo.y-base.y),std::max(0,lo.z-base.z)};
 const int upper[3]={std::min(int(field.pages[0])-1,hi.x-base.x),std::min(int(field.pages[1])-1,hi.y-base.y),std::min(int(field.pages[2])-1,hi.z-base.z)};
 for(int z=lower[2];z<=upper[2];++z)for(int y=lower[1];y<=upper[1];++y)for(int x=lower[0];x<=upper[0];++x){
  const auto offset=(std::size_t(z)*field.pages[1]+unsigned(y))*field.pages[0]+unsigned(x)+painter_detail::DamageField::headerWords;
  const auto page=field.page[offset];if(page==painter_detail::DamageField::keptPage)continue;
  const Cell key{base.x+x,base.y+y,base.z+z};need(key.x>=0&&key.y>=0&&key.z>=0&&key.x<(grid.size.x+7)/8&&key.y<(grid.size.y+7)/8&&key.z<(grid.size.z+7)/8,"accessory_clip_field_extent");
  auto&tile=out[key];
  if(page==painter_detail::DamageField::removedPage){for(auto&w:tile.removed)w=~std::uint64_t(0);continue;}
  need(std::uint64_t(page)-1+16<=field.data.size(),"accessory_clip_field_data");
  for(unsigned q=0;q<8;++q)tile.removed[q]|=std::uint64_t(field.data[page-1+q*2])|(std::uint64_t(field.data[page+q*2])<<32);
 }
}
inline void preserveClipMask(gfy_packed::Mesh&mesh,const gfy_packed::Mesh&published,const bound_surface::ExtraSurface&extra,
                             const Grid&grid,const std::vector<std::pair<Cell,core::Tile>>*inherited,
                             std::vector<std::pair<Cell,core::Tile>>&marks){
 const auto slots=children(extra);Cell lo{INT_MAX,INT_MAX,INT_MAX},hi{INT_MIN,INT_MIN,INT_MIN};
 Cell clipLo=lo,clipHi=hi;bool clipped=false;
 auto bounds=[&](const gfy_packed::Vertex&v,Cell&low,Cell&high){for(unsigned a=0;a<3;++a){
  const int tile=int(std::floor((double(v.surface[a])-grid.origin[a])/(grid.pitch*8)));
  (&low.x)[a]=std::min((&low.x)[a],tile);(&high.x)[a]=std::max((&high.x)[a],tile);}};
 for(const auto&child:mesh.children)for(const auto&v:child.vertices){bounds(v,lo,hi);
  if(!child.material&&child.skin_clip&&slots.count(child.material_source_child)){bounds(v,clipLo,clipHi);clipped=true;}}
 if(!clipped)return;
 need(bool(published.removal_field)&&published.removal_field->available,"accessory_clip_missing_published_field");
 for(unsigned a=0;a<3;++a){(&lo.x)[a]-=2;(&hi.x)[a]+=2;(&clipLo.x)[a]-=2;(&clipHi.x)[a]+=2;}
 auto inside=[&](Cell t){return t.x>=lo.x&&t.y>=lo.y&&t.z>=lo.z&&t.x<=hi.x&&t.y<=hi.y&&t.z<=hi.z;};
 std::map<Cell,core::Tile> carried;
 auto merge=[&](const auto&entry){if(!inside(entry.first))return;auto&t=carried[entry.first];for(unsigned q=0;q<8;++q)t.removed[q]|=entry.second.removed[q];};
 for(const auto&m:marks)merge(m);if(inherited)for(const auto&m:*inherited)merge(m);
 publishedMarks(*published.removal_field,grid,clipLo,clipHi,carried);
 marks.assign(carried.begin(),carried.end());
 struct Fields {std::unordered_map<Cell,core::Tile,core::CellHash> tiles;const void*ownership=nullptr;double pitch;Vec origin;decltype(grid.size) size;};
 Fields fields{{},nullptr,grid.pitch,grid.origin,grid.size};
 for(const auto&entry:grid.tiles)if(inside(entry.first))fields.tiles.emplace(entry.first,entry.second);
 for(const auto&entry:carried){auto&t=fields.tiles[entry.first];for(unsigned q=0;q<8;++q)t.removed[q]|=entry.second.removed[q];}
 auto field=painter_detail::buildDamageField(fields);need(field&&field->available,"accessory_clip_field_capacity");mesh.removal_field=std::move(field);
}
} }
