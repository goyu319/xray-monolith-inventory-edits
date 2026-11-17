#pragma once
// Included after the exact source/accessory decoders in LiveSeparationWorker.
namespace gfy_live {
struct OcularAccessoryData {bound_surface::ExtraSurface extra;ocular_source::Map ocular;std::size_t bytes{};std::set<std::pair<std::uint16_t,std::uint32_t>> body_faces;
 mutable std::mutex collectMutex;mutable bool collectTried{};mutable std::shared_ptr<const original_accessory::CollectMemo> collectMemo;}; // gfyspeed2
inline std::shared_ptr<const OcularAccessoryData> ocularAccessories(const Asset&asset){
 if(asset.accessory_surface_bytes.empty())return {};
 std::lock_guard<std::mutex>lock(asset.ocular_mutex);if(asset.ocular_accessories)return asset.ocular_accessories;
 auto data=std::make_shared<OcularAccessoryData>();data->extra=separation_detail::readAccessorySurfaces(asset,asset.accessory_surface_bytes);
 data->ocular=ocular_source::qualify(asset,data->extra);
 for(std::size_t f=0;f<asset.face_children.size();++f)data->body_faces.emplace(asset.face_children[f],asset.face_indices[f]);
 data->bytes=sizeof(OcularAccessoryData)+data->extra.source.v.capacity()*sizeof(core::SourceVertex)+
  data->extra.source.f.capacity()*sizeof(data->extra.source.f[0])+data->extra.bindings.vertices.capacity()*sizeof(bound_surface::Binding)+
  data->extra.face_children.capacity()*2+data->extra.face_indices.capacity()*4+data->ocular.face_island.capacity()*4+
  data->body_faces.size()*(sizeof(std::pair<std::uint16_t,std::uint32_t>)+4*sizeof(void*));
 for(const auto&i:data->ocular.islands)data->bytes+=sizeof(i)+i.faces.capacity()*4+i.support.capacity()*sizeof(ocular_source::SupportSpan)+i.labels.size()*40;
 asset.ocular_accessories=data;return data;
}
// gfyspeed2: the asset's gear collect memo (original_accessory::CollectMemo), built once beside its decoded gear surface.
inline std::shared_ptr<const original_accessory::CollectMemo> ocularCollectMemo(const Asset&asset,const OcularAccessoryData&data){
 static const bool on=[]{const char*e=std::getenv("GFY_GS2_COLLECT");return !e||std::atoi(e)!=0;}();if(!on)return {};
 std::lock_guard<std::mutex>lock(data.collectMutex);
 if(!data.collectTried){data.collectTried=true;try{data.collectMemo=original_accessory::buildCollectMemo(asset,data.extra);}catch(...){data.collectMemo.reset();}}
 return data.collectMemo;}
// gfyspeed3 (LiveSeparationWorker_impl.h): the asset's decoded gear when it is this gear surface - every field collect(),
// qualify() and the corner memo read, compared exactly (nothing is decoded here: null when the asset has none yet).
inline bool sameGearSource(const bound_surface::ExtraSurface&a,const bound_surface::ExtraSurface&b){
 if(&a==&b)return true;
 if(!(a.source.bones==b.source.bones)||a.source.v.size()!=b.source.v.size()||a.source.f.size()!=b.source.f.size()||a.face_children!=b.face_children||
    a.face_indices!=b.face_indices||!(a.bindings.source==b.bindings.source)||a.bindings.vertices.size()!=b.bindings.vertices.size())return false;
 if(!a.source.f.empty()&&std::memcmp(a.source.f.data(),b.source.f.data(),a.source.f.size()*sizeof(a.source.f[0]))!=0)return false;
 for(std::size_t i=0;i<a.source.v.size();++i){const auto&x=a.source.v[i];const auto&y=b.source.v[i];
  if(std::memcmp(&x.p,&y.p,sizeof x.p)||std::memcmp(&x.normal,&y.normal,sizeof x.normal)||std::memcmp(&x.tangent,&y.tangent,sizeof x.tangent)||
     std::memcmp(&x.binormal,&y.binormal,sizeof x.binormal)||std::memcmp(&x.uv,&y.uv,sizeof x.uv)||std::memcmp(&x.ids,&y.ids,sizeof x.ids)||
     std::memcmp(&x.w,&y.w,sizeof x.w)||x.authored_hardware!=y.authored_hardware)return false;}
 for(std::size_t i=0;i<a.bindings.vertices.size();++i)if(std::memcmp(&a.bindings.vertices[i].point,&b.bindings.vertices[i].point,sizeof(Vec)))return false;
 return true;}
inline GearAccess accessoryGear(const Asset&asset,const bound_surface::ExtraSurface&extra){
 std::shared_ptr<const OcularAccessoryData> data;{std::lock_guard<std::mutex>lock(asset.ocular_mutex);data=asset.ocular_accessories;}
 GearAccess out;if(!data||!sameGearSource(data->extra,extra))return out;
 out.collect=ocularCollectMemo(asset,*data);if(out.collect)out.ocular=std::shared_ptr<const ocular_source::Map>(data,&data->ocular);
 return out;}
std::size_t ocularAccessoryBytes(const Asset&asset)noexcept{
 std::shared_ptr<const OcularAccessoryData> data;{std::lock_guard<std::mutex>lock(asset.ocular_mutex);data=asset.ocular_accessories;}
 if(!data)return 0;std::size_t memo=0;{std::lock_guard<std::mutex>lock(data->collectMutex);if(data->collectMemo)memo=data->collectMemo->bytes();} // gfyspeed2: + the memo
 return data->bytes+memo;
}
inline core::JoinedVertex ocularInterpolate(const core::JoinedTriangle&t,const std::array<double,3>&b){
 for(unsigned k=0;k<3;++k)if(b[k]==1.&&b[(k+1)%3]==0.&&b[(k+2)%3]==0.)return t.v[k];
 return bound_surface::interpolate(t,b);
}
// ownersafe: the re-clip; clipOcularAccessories() at the end runs it and keeps the published gear when it fails. `dirty`: the
// tiles this job changed (null: every eye triangle is re-clipped, as before)
inline void clipOcularAccessoriesExact(const Asset&asset,const Grid&grid,const gfy_packed::Mesh*published,gfy_packed::Mesh&mesh,bool owned,
                                       const std::set<Cell>*dirty){
 const auto decoded=ocularAccessories(asset);if(!decoded)return;
 const auto&extra=decoded->extra;const auto&ocular=decoded->ocular;
 const std::set<std::uint16_t> body(asset.face_children.begin(),asset.face_children.end());
 std::set<std::uint16_t> mixed;
 std::set<std::pair<std::uint16_t,std::uint32_t>> selectedFaces;
 for(std::size_t f=0;f<extra.face_children.size();++f){
  const auto child=extra.face_children[f];if(body.count(child))mixed.insert(child);
  if(body.count(child)||ocular.children.count(child))selectedFaces.emplace(child,extra.face_indices[f]);
 }
 // Body damage must not turn original external gear into clipped skin.
 // PieceSession carries the birth mesh and may mark nearby children; only
 // exact body faces (or explicitly managed ocular faces) may use that mask.
 for(auto&child:mesh.children)if(!child.material&&!ocular.children.count(child.material_source_child)){
  bool bodyFace=false;for(auto f:child.original_faces)if(decoded->body_faces.count({child.material_source_child,f})){bodyFace=true;break;}
  bool coverageBox=!child.vertices.empty();for(const auto&v:child.vertices)if(!(v.surface[12]< -500.f)){coverageBox=false;break;}
  if(!bodyFace&&!coverageBox)child.skin_clip=false;
 }
 if(ocular.children.empty()&&mixed.empty())return;
 const gfy_packed::Mesh empty;
 // gfyspeed2: collect with the asset's memo, building only the ocular / mixed children used below (the others are checked);
 // every source face has an island here (face_island covers the surface), so the loop below sees the same triangles it uses.
 std::set<std::uint16_t> wanted(ocular.children.begin(),ocular.children.end());wanted.insert(mixed.begin(),mixed.end());
 const auto collectMemo=ocular.face_island.size()>=extra.source.f.size()?ocularCollectMemo(asset,*decoded):nullptr;
 const auto attached=collectMemo?original_accessory::collectFast(asset,extra,published?*published:empty,owned,*collectMemo,&wanted):
  original_accessory::collect(asset,extra,published?*published:empty,owned);
 std::vector<core::JoinedTriangle> clip;core::JoinResult untouched;
 // ownersafe (2026-11-15): an eye triangle that was clipped at an earlier publication and lies away from every tile this job changed
 // is kept as published; only the ones near the new damage are clipped again. Re-clipping all of them on every hit (core::exterior
 // conforms every piece to its neighbours' split vertices) grew the eye surface on every publication of a continued GFY body - on the
 // probe chain of soldier 20530 from 515 to 84,836 triangles in 28 hits, the worker from 18 ms to 1,085 ms a hit - and its slivers
 // are the rows the gear re-read finally refused (f13ab, accessory_repeat_barycentric_range). A hit only touches what it changes.
 core::JoinResult kept;
 auto nearChange=[&](const core::JoinedTriangle&t){if(!dirty)return true;double lo[3],hi[3];
  for(unsigned a=0;a<3;++a){lo[a]=hi[a]=t.v[0].p[a];for(unsigned k=1;k<3;++k){lo[a]=std::min(lo[a],t.v[k].p[a]);hi[a]=std::max(hi[a],t.v[k].p[a]);}}
  int tl[3],th[3];const double tile=grid.pitch*8;
  for(unsigned a=0;a<3;++a){const double o=grid.origin[a],l=std::floor((lo[a]-o-grid.pitch)/tile),h=std::floor((hi[a]-o+grid.pitch)/tile);
   if(!std::isfinite(l)||!std::isfinite(h))return true;tl[a]=int(std::clamp(l,-1e6,1e6));th[a]=int(std::clamp(h,-1e6,1e6));}
  for(auto it=dirty->lower_bound(Cell{tl[0],INT_MIN,INT_MIN});it!=dirty->end()&&it->x<=th[0];++it)
   if(it->y>=tl[1]&&it->y<=th[1]&&it->z>=tl[2]&&it->z<=th[2])return true;
  return false;};
 std::vector<char> supported(ocular.islands.size(),0);
 for(std::size_t i=0;i<ocular.islands.size();++i)if(ocular.islands[i].qualified)
  supported[i]=ocular_source::survivingSupport(asset,ocular.islands[i],grid);
 for(const auto&t:attached){const auto f=t.surface.v[0].sourceFace;need(f<ocular.face_island.size(),"ocular_clip_source_face");
  if(!ocular.children.count(extra.face_children[f])){
   if(mixed.count(extra.face_children[f]))untouched.triangles.push_back(t.surface);
   continue;
  }
  const auto id=ocular.face_island[f];
  if(!ocular.islands[id].qualified)untouched.triangles.push_back(t.surface);
  else if(supported[id]){if(t.skin_clip&&!nearChange(t.surface))kept.triangles.push_back(t.surface);else clip.push_back(t.surface);} // ownersafe
 }
 // Clip physical original coordinates by the same cumulative removal field
 // as body skin. Polygon barycentrics interpolate the already-published exact
 // face provenance, UVs, rig and weights; repeat wounds cannot restore faces.
 Source source;source.bones=asset.source.bones;source.v.reserve(clip.size()*3);source.f.reserve(clip.size());
 for(const auto&t:clip){std::array<std::uint32_t,3>face;for(unsigned k=0;k<3;++k){core::SourceVertex v{};v.p=t.v[k].p;face[k]=std::uint32_t(source.v.size());source.v.push_back(v);}source.f.push_back(face);}
 core::JoinResult clipped;
 if(!source.f.empty()){if(painter_ownersafe_fault("clip_exterior"))throw std::runtime_error("ocular_clip_face_range"); /* ownersafe test switch */
  const auto outside=core::exterior(source,grid);
  for(const auto&poly:outside.polygons){need(poly.sourceFace<clip.size(),"ocular_clip_face_range");
   for(std::size_t k=1;k+1<poly.polygon.size();++k){core::JoinedTriangle t;const core::V*rows[3]={&poly.polygon[0],&poly.polygon[k],&poly.polygon[k+1]};
    for(unsigned q=0;q<3;++q)t.v[q]=ocularInterpolate(clip[poly.sourceFace],rows[q]->b);clipped.triangles.push_back(std::move(t));}}
 }
 clipped.triangles.insert(clipped.triangles.end(),kept.triangles.begin(),kept.triangles.end()); // ownersafe: kept as published
 // gfyrefuse (2026-11-15): a degenerate clip polygon (repeated vertices: a tiny eye triangle at the removal field's edge) fans into
 // the same triangle more than once; one copy per source face is kept (corners identical as packed floats - what collect()'s
 // rows check compares - draw the same surface again and nothing else). The next collect() refused such a mesh for good: every
 // later hit of the body was rejected. No repeat: nothing changes.
 {std::set<std::pair<std::uint32_t,std::array<std::array<float,3>,3>>> clipSeen;std::size_t clipKept=0;
  // gfyreject2 (2026-11-15): a corner of a degenerate eye sliver can come out of core::exterior with NaN barycentrics; the packing
  // then refused the publication ("Invalid skin influence") and the hit was thrown away - on every later hit, as the sliver stayed.
  // Only triangles with finite corners are kept (a non-finite one cannot be drawn); no such triangle: nothing changes.
  auto clipFinite=[](const core::JoinedTriangle&t){for(const auto&v:t.v){for(const Vec*e:{&v.p,&v.normal,&v.tangent,&v.binormal})
    if(!std::isfinite(e->x)||!std::isfinite(e->y)||!std::isfinite(e->z))return false;
   for(const double b:v.sourceBary)if(!std::isfinite(b))return false;for(const double u:v.uv)if(!std::isfinite(u))return false;
   for(const double w:v.w)if(!std::isfinite(w))return false;}return true;};
  for(std::size_t i=0;i<clipped.triangles.size();++i){const auto&tr=clipped.triangles[i];if(!clipFinite(tr))continue;std::array<std::array<float,3>,3> q;
   for(unsigned k=0;k<3;++k)q[k]={float(tr.v[k].p.x),float(tr.v[k].p.y),float(tr.v[k].p.z)};std::sort(q.begin(),q.end());
   if(!clipSeen.emplace(tr.v[0].sourceFace,q).second)continue;
   if(clipKept!=i)clipped.triangles[clipKept]=std::move(clipped.triangles[i]);++clipKept;}
  clipped.triangles.resize(clipKept);}
 // Remove only the selected accessory faces. A body child may share the
 // same material slot and must remain intact, on initial and repeat wounds.
 for(auto&child:mesh.children){if(child.material||(!mixed.count(child.material_source_child)&&!ocular.children.count(child.material_source_child)))continue;
  need(child.original_faces.size()*3==child.indices.size(),"mixed_accessory_output_provenance");
  std::size_t keep=0;for(std::size_t f=0;f<child.original_faces.size();++f){
   if(selectedFaces.count({child.material_source_child,child.original_faces[f]}))continue;
   for(unsigned q=0;q<3;++q)child.indices[3*keep+q]=child.indices[3*f+q];child.original_faces[keep++]=child.original_faces[f];
  }
  child.indices.resize(keep*3);child.original_faces.resize(keep);
  if(!keep){child.vertices.clear();child.skin_clip=false;}

 }
 mesh.children.erase(std::remove_if(mesh.children.begin(),mesh.children.end(),[&](const auto&child){return !child.material&&child.indices.empty()&&(child.child_index>=asset.original_children||(ocular.children.count(child.material_source_child)&&!mixed.count(child.material_source_child)));}),mesh.children.end());
 original_accessory::append(mesh,untouched,extra,asset.original_children,false);
 if(painter_ownersafe_fault("clip_append"))throw std::runtime_error("accessory_output_triangle_conservation"); // ownersafe test switch
 original_accessory::append(mesh,clipped,extra,asset.original_children,true);
 for(auto child:ocular.children){bool present=false;for(const auto&row:mesh.children)if(!row.material&&row.material_source_child==child)present=true;
  if(!present){gfy_packed::Child marker;marker.child_index=child;marker.material_source_child=child;mesh.children.push_back(std::move(marker));}}
 // Explicit empty original children have skin_clip=false: the native bridge
 // removes their original draw surface and retains this marker in snapshots.
 if(!clipped.triangles.empty()&&(!mesh.removal_field||!mesh.removal_field->available)){
  mesh.removal_field=painter_detail::buildDamageField(grid);need(mesh.removal_field&&mesh.removal_field->available,"ocular_damage_field_capacity");}
 }
inline void ownersafeKeepPublishedGear(const Asset&asset,const Grid&grid,const gfy_packed::Mesh*published,gfy_packed::Mesh&mesh){
 const auto decoded=ocularAccessories(asset);if(!decoded)return;
 const auto&extra=decoded->extra;const auto&ocular=decoded->ocular;
 const std::set<std::uint16_t> body(asset.face_children.begin(),asset.face_children.end());
 std::set<std::uint16_t> managed(ocular.children.begin(),ocular.children.end());
 for(const auto child:extra.face_children)if(body.count(child))managed.insert(child);
 auto gear=[&](const gfy_packed::Child&c){if(c.material||!managed.count(c.material_source_child))return false;
  if(c.original_faces.empty())return !body.count(c.material_source_child);
  return !decoded->body_faces.count({c.material_source_child,c.original_faces[0]});};
 mesh.children.erase(std::remove_if(mesh.children.begin(),mesh.children.end(),gear),mesh.children.end());
 if(!published)return;
 std::set<std::uint16_t> used;unsigned next=asset.original_children;
 for(const auto&c:mesh.children){used.insert(c.child_index);next=std::max(next,unsigned(c.child_index)+1);}
 bool masked=false;
 for(const auto&c:published->children){if(!gear(c))continue;const bool move=c.child_index>=asset.original_children||used.count(c.child_index);
  if(move&&next>=gfy_packed::max_child_slots)continue;
  auto copy=c;if(move)copy.child_index=std::uint16_t(next++);
  used.insert(copy.child_index);masked|=copy.skin_clip;mesh.children.push_back(std::move(copy));}
 if(masked&&(!mesh.removal_field||!mesh.removal_field->available)){mesh.removal_field=painter_detail::buildDamageField(grid);
  if(!mesh.removal_field||!mesh.removal_field->available)for(auto&c:mesh.children)if(gear(c))c.skin_clip=false;}
}
std::string clipOcularAccessories(const Asset&asset,const Grid&grid,const gfy_packed::Mesh*published,gfy_packed::Mesh&mesh,bool owned,
                                  const std::set<Cell>*dirty){
 original_accessory::lastCollectDrops={};std::string note;
 try{clipOcularAccessoriesExact(asset,grid,published,mesh,owned,dirty);}
 catch(const std::bad_alloc&){throw;}
 catch(const std::exception&e){note=std::string("gear_reclip=skipped gear=as_published why=")+e.what();
  try{ownersafeKeepPublishedGear(asset,grid,published,mesh);}
  catch(const std::bad_alloc&){throw;}
  catch(const std::exception&k){note+=" keep_failed=";note+=k.what();}}
 const auto&d=original_accessory::lastCollectDrops;
 if(d.unread||d.clamped){char text[128];std::snprintf(text,sizeof(text),"%sgear_rows unread=%zu reread_nearest=%zu",note.empty()?"":" ",d.unread,d.clamped);
  note+=text;if(d.unread){note+=" why=";note+=d.why;}}
 return note;}
}
