// Included inside namespace gfy_live after DamageSnapshot::Impl has been defined.
// Pose once for this complete event. Refit existing leaves using the original triangle
// ordering; later rays traverse these bounds without rebuilding or scanning the mesh.
// The legacy accessory/occlusion file overlaps source after GearBodyMerge. Its
// surviving payload has already been filtered by the engine. Decode that once,
// lazily on first fire, rather than leaving inaccessible gear as a second skin.
struct BurnBlockers {
 Source storage;
 const Source& source;
 const core::BodyBVH tree;
 std::set<unsigned> children;
 bool allUnknown{};
 explicit BurnBlockers(Source&& geometry):storage(std::move(geometry)),source(storage),tree(source){}
 explicit BurnBlockers(const Source& original):source(original),tree(source),allUnknown(true){}
};
namespace burn_kernel {
inline std::shared_ptr<const BurnBlockers> assetBlockers(const Asset& a){
 std::call_once(a.burn_blocker_once,[&]{
    if(a.occlusion.f.empty())return;
    auto legacy=[&]{a.burn_blockers=std::make_shared<BurnBlockers>(a.occlusion);};
    if(!a.burn_accessory_matches_occlusion){legacy();return;}
    const auto& raw=a.accessory_surface_bytes;
    if(raw.empty())return; // All accessory triangles joined the primary cuttable source.
    std::array<std::uint32_t,4> lengths{};
    if(raw.size()<56){legacy();return;}
    std::memcpy(lengths.data(),raw.data()+40,16);
    const std::size_t map=56ull+lengths[0]+lengths[1];
    if(map+lengths[2]+lengths[3]!=raw.size()||lengths[0]<20||lengths[2]<12){legacy();return;}
    const auto count=[&](std::size_t at){unsigned n=0;std::memcpy(&n,raw.data()+at,4);return n;};
    const auto faces=count(56+12);
    if(!faces)return;
    if(std::memcmp(raw.data()+map,"VXCHLD01",8)||count(map+8)!=faces||lengths[2]!=12ull+6ull*faces){legacy();return;}
    const std::vector<std::uint8_t> bytes(raw.begin()+56,raw.begin()+56+lengths[0]);
    auto b=std::make_shared<BurnBlockers>(sourceBytes(bytes,true));
    for(unsigned i=0;i<faces;++i){std::uint16_t child{};std::memcpy(&child,raw.data()+map+12+6ull*i,2);b->children.insert(child);}
    a.burn_blockers=std::move(b);
 });
 return a.burn_blockers;
}
// A separated part owns its published accessory triangles, including any CPU
// clipping the separation already performed. Never pose the original actor's
// leftover glasses on each fragment.
inline std::shared_ptr<const BurnBlockers> pieceBlockers(const Asset& a,const BurnBlockers& original,const gfy_packed::Mesh& mesh){
 const auto faces=original.allUnknown?a.burnFaces():nullptr;
 Source owned;owned.bones=a.source.bones;
 for(const auto& child:mesh.children){
    if(child.material||child.material_source_child>=a.original_children||child.indices.empty())continue;
    if(!original.allUnknown&&!original.children.count(child.material_source_child))continue;
    const auto base=std::uint32_t(owned.v.size());bool added=false;
    for(std::size_t i=0;i+2<child.indices.size();i+=3){
       const auto ordinal=i/3<child.original_faces.size()?child.original_faces[i/3]:~0u;
       if(faces&&child.material_source_child<faces->byChild.size()){
          const auto& lookup=faces->byChild[child.material_source_child];
          if(ordinal<lookup.size()&&lookup[ordinal])continue;
       }
       if(!added){for(const auto& v:child.vertices){
          core::SourceVertex p{};p.p={v.surface[0],v.surface[1],v.surface[2]};p.ids=v.bone;p.w=v.weight;owned.v.push_back(p);
       }added=true;}
       owned.f.push_back({base+child.indices[i],base+child.indices[i+1],base+child.indices[i+2]});
    }
 }
 if(owned.f.empty())return {};
 return std::make_shared<BurnBlockers>(std::move(owned));
}
inline std::uint64_t authoredFace(const Asset& a,unsigned f){
 return (std::uint64_t(a.face_children[f])<<32)|a.face_indices[f];
}
struct Posed {
 const Source& source;const core::BodyBVH& tree;
 std::vector<Vec> points;
 std::vector<core::Bounds> boxes;
 Posed(const Source& input,const core::BodyBVH& b,const std::vector<Matrix>& palette):
   source(input),tree(b),points(posedVertices(input,palette)),boxes(b.nodes.size()) {
    for(size_t i=boxes.size();i--;){
      const auto& n=b.nodes[i];auto& box=boxes[i];
      if(n.left>=0){for(int child:{n.left,n.right}) {box.add(boxes[child].lo);box.add(boxes[child].hi);}}
      else for(size_t j=n.first;j<n.first+n.count;++j)for(auto v:input.f[b.order[j]])box.add(points[v]);
    }
 }
 template<class F> void walk(size_t node,Vec o,Vec dir,double& limit,F& visit)const {
    const auto& b=boxes[node];double near_t=0,far_t=limit;
    for(int ax=0;ax<3;++ax){
      if(std::abs(dir[ax])<1.e-14){if(o[ax]<b.lo[ax]||o[ax]>b.hi[ax])return;continue;}
      double a=(b.lo[ax]-o[ax])/dir[ax],z=(b.hi[ax]-o[ax])/dir[ax];
      if(z<a)std::swap(a,z);near_t=std::max(a,near_t);far_t=std::min(far_t,z);
      if(far_t<near_t)return;
    }
    const auto& n=tree.nodes[node];
    if(n.left>=0){walk(n.left,o,dir,limit,visit);walk(n.right,o,dir,limit,visit);return;}
    for(size_t j=n.first;j<n.first+n.count;++j){
      auto face=tree.order[j];auto v=source.f[face];Vec a=points[v[0]],e=points[v[1]]-a,g=points[v[2]]-a;
      Vec h=cross(dir,g);double det=dot(e,h);if(std::abs(det)<1.e-14)continue;
      Vec s=o-a,q=cross(s,e);double u=dot(s,h)/det,w=dot(q,dir)/det,t=dot(g,q)/det;
      if(u<0.||w<0.||u+w>1.||t<0.||t>=limit)continue;
      visit(face,t,std::array<double,3>{1.-u-w,u,w});
    }
 }
};
inline Point point3(Vec p){return{p.x,p.y,p.z};}
}
std::shared_ptr<const painter_burn::SourceFaces> Session::burn_faces()const {
 return impl_->asset->burnFaces();
}
bool DamageSnapshot::owns_burn(unsigned token,unsigned domain,Point p)const {
 if(!token || !impl_->asset)return false;
 const auto& s=*impl_;const auto& grid=*s.grid;Cell c{int(std::floor((p.x-grid.origin.x)/grid.pitch)),int(std::floor((p.y-grid.origin.y)/grid.pitch)),int(std::floor((p.z-grid.origin.z)/grid.pitch))};
 if(domain==1){
    if(token>s.asset->source.f.size())return false;
    if(!s.piece)return !grid.removed(c);
    const auto f=token-1;
    return f<s.piece->faces.size()&&s.piece->faces[f]&&!grid.removed(c)&&
         (!s.burn_owner_query||s.burn_owner_query->accepts(burn_kernel::authoredFace(*s.asset,f),{float(p.x),float(p.y),float(p.z)}));
 }
 return grid.valid(c)&&!grid.removed(c)&&(!grid.ownership||grid.ownerCuttable(c));
}
// Select actual surviving surface triangles on the receiver that the simulation
// has already ignited. All coordinates are in the original mesh rest space.
std::shared_ptr<const painter_fire::Shape> DamageSnapshot::burn_spread_shape()const{
 if(!impl_->asset)return {};
 const auto& asset=*impl_->asset;const auto& geom=asset.source;if(geom.v.empty())return {};
 auto result=std::make_shared<painter_fire::Shape>();
 using P=painter_fire::V;
 auto pos=[](Vec p)->P{return {float(p.x),float(p.y),float(p.z)};};
 P low=pos(geom.v[0].p),high=low;
 for(const auto& v:geom.v){auto p=pos(v.p);for(int a=0;a<3;a++){low[a]=std::min(low[a],p[a]);high[a]=std::max(high[a],p[a]);}}
 result->bounds(low,high);
 const bool pristine=!impl_->piece&&!stamp().revision;
 for(unsigned i=0;i<geom.f.size();++i){
    if(impl_->piece&&(i>=impl_->piece->faces.size()||!impl_->piece->faces[i]))continue;
    const auto ix=geom.f[i];const auto& a=geom.v[ix[0]];const auto& b=geom.v[ix[1]];const auto& c=geom.v[ix[2]];
    if(!pristine){auto centre=(a.p+b.p+c.p)*(1./3.);
      if(!owns_burn(i+1,1,{centre.x,centre.y,centre.z}))continue;}
    const auto bone=a.ids[unsigned(std::max_element(a.w.begin(),a.w.end())-a.w.begin())];
    result->triangle(pos(a.p),pos(b.p),pos(c.p),bone);
 }
 if(result->active.empty()&&impl_->piece&&mesh()){
   for(const auto& ch:mesh()->children){
     if(!ch.material)continue;
     for(std::size_t n=2;n<ch.indices.size();n+=3){
        P p[3];bool valid=true;
        for(unsigned k=0;k<3;k++){
          const auto ix=ch.indices[n-2+k];if(ix>=ch.vertices.size()){valid=false;break;}
          const auto& v=ch.vertices[ix].surface;p[k]={v[0],v[1],v[2]};
        }
        if(valid)result->triangle(p[0],p[1],p[2],65535);
     }
   }
 }
 if(result->active.empty())return {};
 result->finish([&](P p){const auto sample=sample_rest({p[0],p[1],p[2]});
   return sample.anatomy_label!=0&&sample.anatomy_label!=16&&!sample.removed;});
 return result;
}
std::vector<BurnIntersection> DamageSnapshot::burn_sites(std::uint64_t salt,unsigned limit)const {
 std::vector<BurnIntersection> result;
 if(!impl_->asset||!limit)return result;
 limit=std::min(96u,limit);result.reserve(limit);
 const auto& a=*impl_->asset;const auto& geom=a.source;const auto n=unsigned(geom.f.size());
 if(!n)return result;
 auto face=[&](unsigned f){
   if(f>=a.face_children.size()||f>=a.face_indices.size()||
      (impl_->piece&&(f>=impl_->piece->faces.size()||!impl_->piece->faces[f])))return;
   const auto ix=geom.f[f];const auto p=geom.v[ix[0]].p,u=geom.v[ix[1]].p-p,v=geom.v[ix[2]].p-p;
   const auto normal=cross(u,v);if(dot(normal,normal)<1.e-20)return;
   const auto rest=burn_kernel::point3(p+(u+v)*(1./3.));
   if(!owns_burn(f+1,1,rest))return;
   if(impl_->burn_owner_query&&!impl_->burn_owner_query->accepts(burn_kernel::authoredFace(a,f),{float(rest.x),float(rest.y),float(rest.z)}))return;
   result.push_back({0,rest,burn_kernel::point3(unit(normal)),f+1,unsigned(a.face_children[f]),1});
 };
 const unsigned start=unsigned(salt%n),trials=std::min(n,limit*4u);
 for(unsigned shift=0;shift<4 && result.size()<limit;++shift)
   for(unsigned j=shift;j<trials&&result.size()<limit;j+=4)
     face((start+unsigned((std::uint64_t(j)*n)/trials))%n);
 // A small detached hand can own only a few scattered source faces. The compact
 // ownership flags are already resident; inspect them once when the sparse
 // direct sample finds too few. Never pose or build a BVH for an ignited body.
 if(impl_->piece && result.size()<6){
   result.clear();const auto& mask=impl_->piece->faces;
   const unsigned ownedSize=std::min<unsigned>(n,unsigned(mask.size()));
   for(unsigned j=0;j<ownedSize && result.size()<limit;++j){
     const auto f=(start+j)%ownedSize;if(mask[f])face(f);
   }
 }
 // A tissue-only chunk has no exterior source faces. Select measured tissue
 // at vertices of its own published surface, with its live ownership mask.
 if(result.empty()&&mesh()&&a.closure_field){
   for(const auto& child:mesh()->children){
     if(!child.material||child.vertices.empty()||child.indices.empty())continue;
     const unsigned samples=std::min<unsigned>(unsigned(child.indices.size()),limit);
     for(unsigned j=0;j<samples&&result.size()<limit;++j){
       unsigned idx=child.indices[(start+std::size_t(j)*child.indices.size()/samples)%child.indices.size()];
       if(idx>=child.vertices.size())continue;
       const auto& v=child.vertices[idx].surface;Point p{v[0],v[1],v[2]};Vec nrm{v[3],v[4],v[5]};
       if(dot(nrm,nrm)<1.e-12||!owns_burn(262144,2,p))continue;
       const auto material=a.closure_field->sample(p.x,p.y,p.z);
       const auto label=material&63u;if(!label||label==16u||(material&64u))continue;
       result.push_back({0,p,burn_kernel::point3(unit(nrm)),0,unsigned(label),2});
     }
     if(result.size()>=limit)break;
   }
 }
 return result;
}
std::shared_ptr<const BurnTrace> DamageSnapshot::burn_trace(const std::vector<Matrix>& worldBones)const {
 const auto state=impl_;const auto& asset=*state->asset;
 if(worldBones.size()!=asset.source.bones||asset.source.f.empty())return {};
 auto posed=std::make_shared<burn_kernel::Posed>(asset.source,asset.source_tree,worldBones);
 auto blockers=burn_kernel::assetBlockers(asset);
 if(state->piece&&blockers){
    const auto* published=mesh();
    blockers=published?burn_kernel::pieceBlockers(asset,*blockers,*published):nullptr;
 }
 auto shield=blockers?std::make_shared<burn_kernel::Posed>(blockers->source,blockers->tree,worldBones):nullptr;
 const auto self=std::shared_ptr<const DamageSnapshot>(new DamageSnapshot(state));
 return std::make_shared<const BurnTrace>([self,posed,blockers,shield,worldBones,state](Point origin,Point direction,double reach,const BurnBreach& breached,BurnIntersection& best){
    using burn_kernel::point3;
    const auto& a=*state->asset;const auto& s=a.source;
    Vec o=vec(origin),d=vec(direction);double limit=std::min(reach,best.distance);
    auto visit=[&](unsigned face,double dist,const std::array<double,3>& bary){
      if(state->piece&&(face>=state->piece->faces.size()||!state->piece->faces[face]))return;
      auto ids=s.f[face];auto base=s.v[ids[0]].p;
      Vec rest=base+(s.v[ids[1]].p-base)*bary[1]+(s.v[ids[2]].p-base)*bary[2];
      if(face>=a.face_children.size()||face>=a.face_indices.size())return;
      if(state->burn_owner_query&&!state->burn_owner_query->accepts(burn_kernel::authoredFace(a,face),{float(rest.x),float(rest.y),float(rest.z)}))return;
      Vec n=cross(s.v[ids[1]].p-base,s.v[ids[2]].p-base);if(dot(n,n)<1.e-20)return;n=unit(n);
      BurnIntersection hit{dist,point3(rest),point3(n),face+1,unsigned(a.face_children[face]),1};
      const Vec wn=cross(posed->points[ids[1]]-posed->points[ids[0]],posed->points[ids[2]]-posed->points[ids[0]]);
      if(dot(wn,wn)>1.e-18)hit.posedNormal=point3(unit(wn));
      auto field=a.closure_field?a.closure_field:a.tissue_field;
      bool removed=self->sample_rest(hit.rest).removed;
      const bool gap=!removed&&breached&&breached(hit);
      if(!removed&&!gap){best=hit;limit=dist;return;}
      if(!field||!field->available)return;
      Matrix blend{};
      for(unsigned j=0;j<3;++j){const auto& v=s.v[ids[j]];
        for(unsigned k=0;k<4;++k)for(unsigned n=0;n<16;++n)blend.row_major[n]+=worldBones[v.ids[k]].row_major[n]*v.w[k]*bary[j];}
      Vec inv=inverseDirection(blend,d);const double scale=length(inv);
      if(!std::isfinite(scale)||scale<1.e-7)return;
      Vec ray=inv*(1./scale);
      // A breached garment may reveal only the same short first-underlayer
      // interval actually sampled by pb_first_under_cloth.
      const double step=gap?.0005:std::min(.0005,.5*field->pitch);
      const double start=gap?.00025:0.,end=std::min(gap?.0105:.30,(limit-dist)*scale);
      if(!(step>.00001))return;
      auto solid=[&](Vec p){if(self->sample_rest(point3(p)).removed)return 0u;return field->sample(p.x,p.y,p.z);};
      for(double advance=start;advance<end;advance+=step){
         const Vec p=rest+ray*advance;
         if(gap&&self->sample_rest(point3(p)).removed)return;
         const unsigned raw=solid(p);if(!raw||raw==16u)continue;
         const unsigned slot=raw&63u;const double t=dist+advance/scale;
         if((raw&64u)||!slot||slot==16u){
             // Synthetic closure is visible backing. It cannot certify a new
             // exposed anatomical surface and must not admit fire through it.
             best={t,point3(p),{},0,0,0};limit=t;return;
         }
         
         Vec grad{};for(int ax=0;ax<3;++ax){Vec offset{};offset[ax]=field->pitch;
             auto inside=[](unsigned m){return !(m&64u)&&(m&63u)!=0u&&(m&63u)!=16u;};
             grad[ax]=double(inside(solid(p-offset)))-double(inside(solid(p+offset)));}
         if(dot(grad,grad)<1e-15)grad=ray*(-1.);else grad=unit(grad);
         if(dot(grad,ray)>0.)grad=grad*(-1.);
         best={t,point3(p),point3(grad),0,slot,2};limit=t;return;
      }
    };
    posed->walk(0,o,d,limit,visit);
    if(shield){
       auto block=[&](unsigned face,double t,const std::array<double,3>& bary){
          const auto& src=blockers->source;const auto ids=src.f[face];
          const Vec rest=src.v[ids[0]].p*bary[0]+src.v[ids[1]].p*bary[1]+src.v[ids[2]].p*bary[2];
          if(self->sample_rest(point3(rest)).removed)return;
          // An unmatched secondary shell can stop light/heat but carries no
          // certified drawable token. Equal-depth primary triangles win.
          if(t+1.e-8>=limit)return;
          limit=t;best={t,point3(rest),{},0,0,0};
       };
       shield->walk(0,o,d,limit,block);
    }
 });
}
