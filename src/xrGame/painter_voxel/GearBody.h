#pragma once
namespace gfy_live {
namespace {std::atomic<int> gearBodyOn{-1};}
void set_gear_body(int on)noexcept{gearBodyOn.store(on?1:0,std::memory_order_relaxed);}
int gear_body_value()noexcept{int v=gearBodyOn.load(std::memory_order_relaxed);
 if(v<0){const char*e=std::getenv("GFY_GEAR_BODY");v=e&&*e?(std::atoi(e)!=0?1:0):1;}return v;}
namespace gear_body_detail {
struct Blobs {bool ok{};std::array<std::size_t,4> at{},size{};};
inline Blobs blobs(const std::vector<std::uint8_t>&raw){Blobs b;if(raw.size()<56||std::memcmp(raw.data(),"VXGEOM01",8))return b;
 std::size_t at=56;for(unsigned i=0;i<4;++i){std::uint32_t n;std::memcpy(&n,raw.data()+40+4*i,4);if(n>raw.size()-at)return b;b.at[i]=at;b.size[i]=n;at+=n;}
 b.ok=at==raw.size();return b;}
inline std::uint32_t u32(const std::vector<std::uint8_t>&raw,std::size_t at){std::uint32_t v;std::memcpy(&v,raw.data()+at,4);return v;}
inline void put32(std::vector<std::uint8_t>&raw,std::size_t at,std::uint32_t v){std::memcpy(raw.data()+at,&v,4);}
// The gear file with only the faces keepFace marks: every row copied byte for byte, same hash and blob formats.
inline std::vector<std::uint8_t> keepOnly(const std::vector<std::uint8_t>&raw,const std::vector<char>&keepFace){
 const auto b=blobs(raw);need(b.ok&&b.size[0]>=20&&b.size[1]>=12&&b.size[2]>=12&&b.size[3]>=52,"gear_body_blobs");
 const std::size_t h0=b.at[0],h1=b.at[1],h2=b.at[2],h3=b.at[3];const std::uint64_t nv=u32(raw,h0+8),nf=u32(raw,h0+12);
 need(b.size[0]==20+44*nv+12*nf&&keepFace.size()==nf&&b.size[1]==12+36*nv&&b.size[2]==12+6*nf&&b.size[3]==52+32*nv,"gear_body_rows");
 std::vector<std::uint32_t> remap(std::size_t(nv),~0u),order,faceOf;std::vector<std::array<std::uint32_t,3>> faces;
 for(std::uint32_t f=0;f<nf;++f){if(!keepFace[f])continue;std::array<std::uint32_t,3> q;
  for(unsigned k=0;k<3;++k){const auto v=u32(raw,std::size_t(h0+20+44*nv+12ull*f+4*k));need(v<nv,"gear_body_index");
   if(remap[v]==~0u){remap[v]=std::uint32_t(order.size());order.push_back(v);}q[k]=remap[v];}
  faces.push_back(q);faceOf.push_back(f);}
 const auto kv=std::uint32_t(order.size()),kf=std::uint32_t(faces.size());
 auto rows=[&](std::size_t head,std::size_t first,std::size_t width,const std::vector<std::uint32_t>&which){
  std::vector<std::uint8_t> o(raw.begin()+std::ptrdiff_t(head),raw.begin()+std::ptrdiff_t(first));o.reserve(o.size()+width*which.size());
  for(auto w:which)o.insert(o.end(),raw.begin()+std::ptrdiff_t(first+width*w),raw.begin()+std::ptrdiff_t(first+width*w+width));return o;};
 auto g=rows(h0,h0+20,44,order);put32(g,8,kv);put32(g,12,kf);
 for(const auto&q:faces){const auto at=g.size();g.resize(at+12);std::memcpy(g.data()+at,q.data(),12);}
 auto s=rows(h1,h1+12,36,order);put32(s,8,kv);
 auto c=rows(h2,h2+12,6,faceOf);put32(c,8,kf);
 auto a=rows(h3,h3+52,32,order);put32(a,40,kv);put32(a,44,kf);
 std::vector<std::uint8_t> out(raw.begin(),raw.begin()+40);
 for(const auto*p:{&g,&s,&c,&a}){const auto n=std::uint32_t(p->size());const auto at=out.size();out.resize(at+4);std::memcpy(out.data()+at,&n,4);}
 for(const auto*p:{&g,&s,&c,&a})out.insert(out.end(),p->begin(),p->end());return out;}
}
GearBodyState gearBodyStart(const AssetInput&input){
 GearBodyState s;if(input.accessory_surfaces.empty()||input.source_children.empty())return s;
 const auto b=gear_body_detail::blobs(input.accessory_surfaces);if(!b.ok||b.size[0]<20)return s;
 s.gear_faces=gear_body_detail::u32(input.accessory_surfaces,b.at[0]+12);if(!s.gear_faces)return s;
 if(!gear_body_value()){s.note="switch=0 gear_triangles="+std::to_string(s.gear_faces)+" (the gear path)";return s;}
 s.deferred=true;return s;}
void gearBodyMerge(Asset&a,const AssetInput&in){
 auto&g=a.gear_body;const auto start=core::Clock::now();
 g.body_vertices=std::uint32_t(a.source.v.size());g.body_faces=std::uint32_t(a.source.f.size());
 auto buildTree=[&]{auto&t=a.source_tree;t.order.clear();t.nodes.clear();t.order.reserve(a.source.f.size());
  for(std::uint32_t i=0;i<a.source.f.size();++i)t.order.push_back(i);t.build(0,t.order.size());};
 std::string why;
 try{
  const auto extra=separation_detail::readAccessorySurfaces(a,in.accessory_surfaces);
  std::map<std::uint16_t,const char*> stay;
  for(const auto c:ocular_source::qualify(a,extra).children)stay.emplace(c,"eye");
  for(std::size_t f=0;f<extra.source.f.size();++f){const auto c=extra.face_children[f];if(stay.count(c))continue;
   for(const auto v:extra.source.f[f]){const double d=extra.bindings.vertices[v].distance;if(!(d<=.35)){stay.emplace(c,"bound_beyond_35cm");break;}}}
  if(a.analytic_channels&&!in.render_field.empty()){
   g.baked=renderFieldBytes(in.render_field); // read once: the analytic block below takes it
   const auto&r=*g.baked;
   for(std::size_t f=0;f<extra.source.f.size();++f){const auto c=extra.face_children[f];if(stay.count(c))continue;
    for(const auto v:extra.source.f[f]){const auto&p=extra.source.v[v].p;bool inside=true;
     for(int ax=0;ax<3;++ax){const double o=double(r.origin[ax]),e=o+double(r.size[ax])*double(r.pitch);inside=inside&&p[ax]>=o&&p[ax]<=e;}
     if(!inside){stay.emplace(c,"outside_render_field");break;}}}}
  std::vector<char> keepFace(extra.source.f.size(),0);std::map<std::uint16_t,std::size_t> mergedTris,keptTris;
  for(std::size_t f=0;f<extra.source.f.size();++f){const auto c=extra.face_children[f];if(stay.count(c)){keepFace[f]=1;++keptTris[c];}else ++mergedTris[c];}
  std::vector<std::uint8_t> remaining;
  if(!mergedTris.empty()&&!keptTris.empty())remaining=gear_body_detail::keepOnly(in.accessory_surfaces,keepFace);
  // append (nothing above changed the asset; only an allocation can fail from here on)
  if(!mergedTris.empty()){
   std::vector<std::uint32_t> remap(extra.source.v.size(),~0u);
   a.source.v.reserve(a.source.v.size()+extra.source.v.size());a.source.f.reserve(a.source.f.size()+extra.source.f.size());
   for(std::size_t f=0;f<extra.source.f.size();++f){if(keepFace[f])continue;std::array<std::uint32_t,3> face;
    for(unsigned k=0;k<3;++k){const auto v=extra.source.f[f][k];
     if(remap[v]==~0u){remap[v]=std::uint32_t(a.source.v.size());a.source.v.push_back(extra.source.v[v]);g.vertices.push_back(v);}face[k]=remap[v];}
    a.source.f.push_back(face);a.face_children.push_back(extra.face_children[f]);a.face_indices.push_back(extra.face_indices[f]);++g.merged_faces;}
   a.accessory_surface_bytes=std::move(remaining);
   for(const auto&m:mergedTris)g.merged.push_back(m.first);}
  for(const auto&k:keptTris)g.kept_faces+=k.second;
  std::string merged,stayed;
  for(const auto&m:mergedTris)merged+=(merged.empty()?"":",")+std::to_string(m.first)+":"+std::to_string(m.second);
  for(const auto&k:keptTris)stayed+=(stayed.empty()?"":",")+std::to_string(k.first)+":"+std::to_string(k.second)+":"+stay.at(k.first);
  g.note="merged_children="+(merged.empty()?std::string("none"):merged)+" stayed_children="+(stayed.empty()?std::string("none"):stayed)+
   " merged_triangles="+std::to_string(g.merged_faces)+" stayed_triangles="+std::to_string(g.kept_faces);
 }catch(const std::bad_alloc&){throw;}
 catch(const std::exception&e){why=e.what();}
 if(!why.empty())g.note="merged_children=none skipped="+why+" gear_triangles="+std::to_string(g.gear_faces)+" (the gear path)";
 buildTree();
 char ms[48];std::snprintf(ms,sizeof ms," prep_ms=%.1f",core::ms(start));g.note+=ms;
}
std::string gear_body_note(const std::shared_ptr<const Asset>&asset)noexcept{try{return asset?asset->gear_body.note:std::string();}catch(...){return {};}}
// The body's binding rows + the merged gear vertices' rows (the gear file's VXATTCH1 blob), in the order they joined the source.
std::vector<std::uint8_t> gearBodyBindings(const Asset&a,std::vector<std::uint8_t> body,const std::vector<std::uint8_t>&gear){
 const auto&g=a.gear_body;if(!g.merged_faces)return body;
 need(body.size()>=52&&!std::memcmp(body.data(),"VXATTCH1",8)&&gear_body_detail::u32(body,40)==g.body_vertices&&gear_body_detail::u32(body,44)==g.body_faces,"gear_body_binding_rows");
 const auto b=gear_body_detail::blobs(gear);need(b.ok&&b.size[3]>=52,"gear_body_binding_blob");
 const std::uint64_t nv=gear_body_detail::u32(gear,b.at[3]+40);need(b.size[3]==52+32*nv,"gear_body_binding_count");
 body.reserve(body.size()+32*g.vertices.size());
 for(const auto v:g.vertices){need(v<nv,"gear_body_binding_vertex");const auto at=std::ptrdiff_t(b.at[3]+52+32ull*v);body.insert(body.end(),gear.begin()+at,gear.begin()+at+32);}
 gear_body_detail::put32(body,40,std::uint32_t(g.body_vertices+g.vertices.size()));gear_body_detail::put32(body,44,std::uint32_t(g.body_faces+g.merged_faces));
 return body;}
// The coat roots of the body source: same rooted vertices, a 0 face flag (no rooted corner) for each merged gear face.
std::vector<std::uint8_t> gearBodyCoatRoots(const Asset&a,std::vector<std::uint8_t> roots){
 const auto&g=a.gear_body;if(!g.merged_faces)return roots;
 need(roots.size()>=52&&!std::memcmp(roots.data(),"VXROOT01",8)&&gear_body_detail::u32(roots,40)==g.body_vertices&&gear_body_detail::u32(roots,44)==g.body_faces,"gear_body_coat_roots");
 gear_body_detail::put32(roots,40,std::uint32_t(g.body_vertices+g.vertices.size()));gear_body_detail::put32(roots,44,std::uint32_t(g.body_faces+g.merged_faces));
 roots.insert(roots.end(),g.merged_faces,std::uint8_t(0));return roots;}
// Where a traced hit was mapped: onto merged gear (gear_child), or onto the body with merged gear crossed in front of that point
// (gear_on_ray, gear_on_ray_m before it). Only for events the owner traces (Impact::trace_gear: the verbose log).
void gearBodyTrace(const Asset&a,const Impact&event,const Mapping&mapping,PoseCache&cache,ImpactResult&result){
 const auto&g=a.gear_body;if(!g.merged_faces||mapping.face>=a.source.f.size())return;
 if(mapping.face>=g.body_faces)result.gear_child=a.face_children[mapping.face];
 const auto&posed=cache.get(a.source,event.bone_model_render,false);if(posed.size()!=a.source.v.size())return;
 const Vec d=unit(vec(event.accepted_pose_direction)),start=vec(event.accepted_pose_point)-d*event.source_search_distance;
 double best=mapping.posed_distance-1e-6;std::uint32_t found=~0u;
 for(std::uint32_t f=g.body_faces;f<a.source.f.size();++f){if(f==mapping.face)continue;const auto&ids=a.source.f[f];
  const Vec p0=posed[ids[0]],e1=posed[ids[1]]-p0,e2=posed[ids[2]]-p0,h=cross(d,e2);const double det=dot(e1,h);if(std::abs(det)<1e-14)continue;
  const Vec s=start-p0;const double u=dot(s,h)/det;if(u<0||u>1)continue;const Vec q=cross(s,e1);const double v=dot(d,q)/det;if(v<0||u+v>1)continue;
  const double t=dot(e2,q)/det;if(t>0&&t<best){best=t;found=f;}}
 if(found!=~0u){result.gear_on_ray=a.face_children[found];result.gear_on_ray_m=mapping.posed_distance-best;}
}
// The merged gear a part holds, as flesh for the bullet test (never physics): per element of the part one box (its original bone
// frame, axis-aligned there) around the gear vertices that follow it (the dominant bone's element in the part, or its pose anchor).
template<class Frames> void gearBodyFlesh(const Asset&a,SeparationPiece&piece,const Frames&frames){
 piece.gear_flesh.clear();const auto&g=a.gear_body;if(!g.merged_faces||piece.elements.empty())return;
 std::set<std::uint16_t> own;for(const auto&e:piece.elements)own.insert(e.bone);
 std::map<std::uint16_t,std::array<double,6>> box;
 for(const auto&child:piece.mesh.children){if(child.material||!std::binary_search(g.merged.begin(),g.merged.end(),child.material_source_child))continue;
  for(const auto&vx:child.vertices){if(vx.surface[12]< -500.f)continue; // a coverage box row
   const auto w=gfy_signed_hw::full(vx.bone,vx.weight);std::size_t b=0;for(std::size_t k=1;k<w.size();++k)if(w[k]>w[b])b=k;
   if(b>=frames.bones.size())continue;
   std::uint16_t e=std::uint16_t(b);
   if(!own.count(e)){if(b<piece.pose_anchors.size()&&piece.pose_anchors[b]!=65535&&own.count(piece.pose_anchors[b]))e=piece.pose_anchors[b];
    else{const auto pe=frames.bones[b].physical_element;if(pe>=0&&own.count(std::uint16_t(pe)))e=std::uint16_t(pe);else continue;}}
   const auto&m=frames.bones[e].rest_to_anchor.row_major;const double x=vx.surface[0],y=vx.surface[1],z=vx.surface[2];
   const double p[3]={m[0]*x+m[1]*y+m[2]*z+m[3],m[4]*x+m[5]*y+m[6]*z+m[7],m[8]*x+m[9]*y+m[10]*z+m[11]};
   auto it=box.find(e);if(it==box.end())box.emplace(e,std::array<double,6>{p[0],p[1],p[2],p[0],p[1],p[2]});
   else for(int k=0;k<3;++k){it->second[k]=std::min(it->second[k],p[k]);it->second[3+k]=std::max(it->second[3+k],p[k]);}}}
 for(const auto&kv:box){SeparationElement el;el.bone=kv.first;SeparationBox sb;const auto&r=kv.second;
  sb.center={.5*(r[0]+r[3]),.5*(r[1]+r[4]),.5*(r[2]+r[5])};sb.half_extents={.5*(r[3]-r[0]),.5*(r[4]-r[1]),.5*(r[5]-r[2])};
  sb.orientation={1,0,0,0,1,0,0,0,1};el.boxes.push_back(sb);piece.gear_flesh.push_back(std::move(el));}
}
}
