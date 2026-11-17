#pragma once
// CPU geometry cache only. Never publishes, acknowledges, or alters logical
// damage. Includes must provide actual core, history, planner and packing types.
namespace channelworker {
inline bool sameOwner(const gfy_live::Identity&a,const gfy_live::Identity&b){return a.spawn_cookie==b.spawn_cookie&&a.visual_lineage==b.visual_lineage&&a.topology==b.topology&&a.source==b.source&&a.exact_rig==b.exact_rig;}
enum CoverageRequirement:unsigned {analytic_voxel_union_unproven=1,skin_cavity_rim_unproven=2,edge_repairs_unrepresented=4,body_visibility_required=8};
struct ChildPacket {gfy_packed::Child geometry;std::vector<packedchannel::OwnerTriangle> owners;};
struct State {
 std::shared_ptr<const void> asset_pin;
 std::shared_ptr<const gfy_live::ResolvedCutHistory> history;
 std::vector<std::shared_ptr<const ChildPacket>> children;
 double tolerance{};std::size_t total_cuts{},triangles{};unsigned coverage_requirements{};
 // No strong DamageSnapshot pin: it could own Output.mesh.field and cycle.
};
struct Stats {std::size_t planned_cuts{},planned_triangles{},packed_triangles{},reused_sealed_children{},copied_tail_rows{};double plan_ms{},weight_ms{},pack_ms{},append_ms{},total_ms{};packedchannel::WeightBatchStats weights;std::size_t split_added{},split_cut_edges{},split_by_ends{},split_by_mid{},split_rounds{},split_deep{},split_hidden{};double split_ms{},split_sample_ms{},split_cut_ms{},split_eval_ms{};}; // skin2: wall refinement statistics
struct Prepared {std::shared_ptr<const State> state;Stats stats;bool deployment_eligible()const noexcept{return false;}};
// R62: a torn tissue surface's walls are its stored triangles (TissueTear.h), not a cutter's planned wall.
inline channelbirth::Plan surfacePlan(const gfy_live::ResolvedTearSurface&s){channelbirth::Plan p;p.vertices.reserve(s.points.size());
 if(gfy_live::mask_only_mode_value()>=2&&s.triangles.empty()&&s.points.empty()){p.admitted=true;return p;} /* mask-only wounds, mode 2: a tear with no torn-end mesh plans no walls */
 for(const auto&q:s.points)p.vertices.push_back({double(q[0]),double(q[1]),double(q[2])});p.triangles=s.triangles;
 if(s.labels.size()!=s.triangles.size()||s.normals.size()!=s.points.size()||s.triangles.empty())throw std::runtime_error("channel_tissue_surface");
 for(const auto&t:s.triangles)for(auto id:t)if(id>=s.points.size())throw std::runtime_error("channel_tissue_index");
 p.stats.triangles=p.triangles.size();p.admitted=true;return p;}

inline double wallSplitWeightDelta(){static const double v=[]{const char* e=std::getenv("GFY_WALL_SPLIT_DELTA");return e?std::atof(e):.4;}();return v;}
// edge refined across a limb contact (GFY_WALL_LIMB_EDGE_MM, default 6: 80 % of the 3 mm gain at half its extra triangles; 0 = off)
inline double wallLimbSplitEdge(){static const double v=[]{const char* e=std::getenv("GFY_WALL_LIMB_EDGE_MM");return (e?std::atof(e):6.)*.001;}();return v;}
inline double wallEdgeLimit(){static const double v=[]{const char* e=std::getenv("GFY_WALL_EDGE_MM");const double mm=e?std::atof(e):30.;return mm>0?mm*.001:0.;}();return v;}
template<class WeightField> inline void splitLongEdges(packedchannel::Input& in,double maxEdge,WeightField& field){
 if(!(maxEdge>0)||in.triangles.empty())return;
 auto rounded=[](const core::Vec& q){return core::Vec{double(float(q.x)),double(float(q.y)),double(float(q.z))};};
 const double limit2=maxEdge*maxEdge;
 std::vector<std::array<double,64>> skin;std::vector<char> known;
 auto ensure=[&](std::uint32_t i){if(skin.size()<in.p.size()){skin.resize(in.p.size());known.resize(in.p.size(),0);}
  if(!known[i]){bool cached=true;if(!field.sampleCached(rounded(in.p[i]),skin[i],cached)&&(cached||!field.sample(rounded(in.p[i]),skin[i])))skin[i].fill(0.);known[i]=1;}};
 auto len2=[&](std::uint32_t a,std::uint32_t b){const core::Vec d=rounded(in.p[b])-rounded(in.p[a]);return core::dot(d,d);};
 auto skinChanges=[&](std::uint32_t a,std::uint32_t b){ensure(a);ensure(b);double l1=0;for(unsigned k=0;k<64;++k)l1+=std::abs(skin[a][k]-skin[b][k]);return l1>wallSplitWeightDelta();};
 auto key=[](std::uint32_t a,std::uint32_t b){return a<b?(std::uint64_t(a)<<32)|b:(std::uint64_t(b)<<32)|a;};
 // Longest-edge bisection (Rivara) with conforming propagation: a triangle with a long edge joining different skin marks
 // its LONGEST edge; any triangle with a marked edge marks its longest edge too; every touched triangle is then split
 // around the midpoint m of its longest edge a-b (c opposite): (a,m,c),(m,b,c), a marked b-c (midpoint n) splits (m,b,c)
 // into (m,b,n),(m,n,c), a marked c-a (midpoint k) splits (a,m,c) into (a,m,k),(k,m,c). Every new edge ends at m and is
 // shorter than a-b, so the longest refined edge falls every round and the triangles keep their shape (no slivers).
 // Splitting only the edges whose ends differ never converges here (fans of float-degenerate slivers, measured).
 auto longest=[&](const std::array<std::uint32_t,3>& t){int best=0;double bl=-1;std::uint64_t bk=0;
  for(int e=0;e<3;++e){const double l=len2(t[e],t[(e+1)%3]);const auto k=key(t[e],t[(e+1)%3]);if(l>bl||(l==bl&&k>bk)){bl=l;bk=k;best=e;}}return best;};
 // An edge whose two ends lean to different limbs (the tear's test, fastwound::limbTearShare) is refined down to
 // wallLimbSplitEdge: tearLimbContacts gives each such triangle to one limb, and a torn triangle then stands off its limb
 // by at most that much (the bores through the touching thighs of d280403f hung 1-3 cm strips into the parted crotch).
 const auto* limbs=field.limbs.get();std::vector<std::uint8_t> dom;
 auto domChain=[&](std::uint32_t i){if(dom.size()<in.p.size())dom.resize(in.p.size(),254);if(dom[i]==254){ensure(i);double share[32]{};
   for(unsigned b=0;b<64;++b)if(skin[i][b]>0)share[limbs->boneChain[b]&31u]+=skin[i][b];unsigned best=0;double bs=0;
   for(unsigned c=1;c<32;++c)if(share[c]>bs){bs=share[c];best=c;}dom[i]=bs>fastwound::limbTearShare?std::uint8_t(best):0;}return dom[i];};
 auto limbCross=[&](std::uint32_t a,std::uint32_t b){if(!limbs||fastwound::wallSkinLegacy())return false;const auto da=domChain(a),db=domChain(b);return da&&db&&da!=db;};
 const double limbLimit2=wallLimbSplitEdge()>0?wallLimbSplitEdge()*wallLimbSplitEdge():1e300;
 auto needs=[&](const std::array<std::uint32_t,3>& t){for(int e=0;e<3;++e){const auto a=t[e],b=t[(e+1)%3];const double l=len2(a,b);
   if((l>limit2&&skinChanges(a,b))||(l>limbLimit2&&limbCross(a,b)))return true;}return false;};
 bool any=false;for(const auto& t:in.triangles)if(needs(t)){any=true;break;}
 if(!any)return;
 if(in.normals.empty()){std::vector<core::Vec> sum(in.p.size());
  for(const auto& t:in.triangles){const core::Vec a=rounded(in.p[t[0]]),b=rounded(in.p[t[1]]),c=rounded(in.p[t[2]]);const core::Vec n=core::cross(c-a,b-a);for(auto id:t)sum[id]=sum[id]+n;}
  in.normals.resize(in.p.size());for(std::size_t i=0;i<sum.size();++i)in.normals[i]=packedchannel::unit(sum[i]);}
 const bool kinds=!in.kinds.empty();
 auto touches=[&](const std::array<std::uint32_t,3>& t,const std::unordered_set<std::uint64_t>& marked){for(int e=0;e<3;++e)if(marked.count(key(t[e],t[(e+1)%3])))return true;return false;};
 for(int round=0;round<64;++round){std::unordered_set<std::uint64_t> marked;
  for(const auto& t:in.triangles)if(needs(t)){const int e=longest(t);marked.insert(key(t[e],t[(e+1)%3]));}
  for(bool grew=!marked.empty();grew;){grew=false;
   for(const auto& t:in.triangles)if(touches(t,marked)){const int e=longest(t);grew|=marked.insert(key(t[e],t[(e+1)%3])).second;}}
  std::vector<std::uint64_t> order(marked.begin(),marked.end());std::sort(order.begin(),order.end());
  std::unordered_map<std::uint64_t,std::uint32_t> mid;mid.reserve(order.size());
  for(const auto k:order){const auto a=std::uint32_t(k>>32),b=std::uint32_t(k&0xffffffffull);
   const auto id=std::uint32_t(in.p.size());in.p.push_back((in.p[a]+in.p[b])*.5);const core::Vec n=in.normals[a]+in.normals[b];const double l=core::length(n);
   in.normals.push_back(l>1e-9?n*(1/l):in.normals[a]);mid.emplace(k,id);}
  if(mid.empty())break;
  if(in.p.size()>=std::size_t(1)<<31)throw std::runtime_error("channel_split_capacity");
  std::vector<std::array<std::uint32_t,3>> next;std::vector<std::uint32_t> nextKinds;next.reserve(in.triangles.size()+3*mid.size());
  for(std::size_t i=0;i<in.triangles.size();++i){const auto t=in.triangles[i];
   auto emit=[&](std::uint32_t a,std::uint32_t b,std::uint32_t c){next.push_back({a,b,c});if(kinds)nextKinds.push_back(in.kinds[i]);};
   if(!touches(t,marked)){emit(t[0],t[1],t[2]);continue;}
   const int e=longest(t);const std::uint32_t a=t[e],b=t[(e+1)%3],c=t[(e+2)%3],m=mid.at(key(a,b));
   auto bc=mid.find(key(b,c));if(bc==mid.end())emit(m,b,c);else{emit(m,b,bc->second);emit(m,bc->second,c);}
   auto ca=mid.find(key(c,a));if(ca==mid.end())emit(a,m,c);else{emit(a,m,ca->second);emit(ca->second,m,c);}}
  in.triangles=std::move(next);if(kinds)in.kinds=std::move(nextKinds);}
}

// skin2 (2026-10-29): wound-wall refinement from the posed error, replacing round 1's splitLongEdges (kept for
// GFY_SKIN2_LEGACY=1). Two steps on each cut's planned / torn-surface triangles, after the weight prefetch:
// 1. Exact tear line (W2). Where the corners' limb side changes along an edge (both ends lean to a limb chain, > 5 % after the
//    side gate, and the two chains differ: thighs touching at the crotch, the crotch midline), the edge is cut where the two
//    chains' physics-shape scores tie (14 bisection steps of native_shape::classify) and its triangles are split along the cuts
//    (1 cut: 2 triangles, 2: 3, 3: 4). A cut within 3 % of a corner is not made (that corner is the boundary). The tear
//    (fastwound::tearLimbSides) gives each side its own limb, so the torn edge is the side boundary. Round 1 refined every
//    limb-crossing edge down to 6 mm instead (about half of its extra triangles) and the torn triangles stood up to 6 mm off.
// 2. Posed-error refinement (W3). Skinned at its corners only, a wall triangle is posed flat; the true place of a wall point is
//    the blend of its own skin. For an edge a-b with midpoint m the posed midpoint misses the mean of the posed corners by
//    E = sum_k D_k [delta_k m - Delta_k (b-a)/4] (delta = w(m)-(w(a)+w(b))/2, Delta = w(b)-w(a), D_k = bone k's motion less
//    the identity; the weights sum to 1). Over every pose whose joints j turn up to theta_j:
//      E <= |delta|_1/2 * max sum_j theta_j |m - anchor_j|  +  |Delta|_1/8 * |b-a| * max sum_j theta_j
//    (sums over the joints between a bone gaining and a bone losing weight in the physical parent tree; rest anchors; theta:
//    limb bones GFY_WALL_THETA_DEG, trunk bones GFY_WALL_THETA_TRUNK_DEG). An edge is bisected (Rivara longest-edge bisection
//    with conforming closure, as round 1) while E > tolerance and it is longer than the minimum edge; the midpoint skin is
//    sampled only for edges longer than one lattice cell (the 2 cm trilinear field is close to linear inside one). Walls
//    whose skin does not change are never split, however long; long walls across a slow change are (round 1 kept a 49 cm
//    waist bore wall whose end skins differed by .37 < .4 in one piece: a flat panel through the bending waist).
//    Rounds work on a worklist (only triangles made in the last round are tested again; decisions are kept per edge).
// The rest surface is unchanged: coplanar parts, kinds kept, corner normals as pack() gives them (computed here on the unsplit
// mesh the same way, only once something is split), a new point takes the normalized blend of its edge's end normals.
// weights (out) = every corner's sampled skin (the field's own sample at the float-rounded point, as packWithField takes it).
struct WallRefineStats {std::size_t cut_edges{},evaluated{},rounds{},by_ends{},by_mid{},deep{},hidden{};double ms_sample{},ms_cut{},ms_eval{},ms_prefetch{},ms_worklist{},ms_leb{},ms_map{},ms_bfs{},ms_mid{},ms_emit{},ms_final{};std::size_t interpolated{};};
inline double wallSampleEdge(){static const double v=[]{const char* e=std::getenv("GFY_WALL_SAMPLE_EDGE_MM");return (e?std::atof(e):0.)*.001;}();return v;}
inline bool wallPrefetch(){static const bool v=[]{const char* e=std::getenv("GFY_WALL_PREFETCH");return e&&std::atoi(e)!=0;}();return v;}
inline double wallMidDepth(){static const double v=[]{const char* e=std::getenv("GFY_WALL_MID_DEPTH_MM");return (e?std::atof(e):16.)*.001;}();return v;}
inline bool wallProfile(){static const bool v=std::getenv("GFY_WALL_PROFILE")!=nullptr;return v;} // development read-out only
inline double wallTolerance(){static const double v=[]{const char* e=std::getenv("GFY_WALL_TOL_MM");return (e?std::atof(e):2.)*.001;}();return v;}
inline double wallThetaLimb(){static const double v=[]{const char* e=std::getenv("GFY_WALL_THETA_DEG");return (e?std::atof(e):110.)*0.017453292519943295;}();return v;}
inline double wallThetaTrunk(){static const double v=[]{const char* e=std::getenv("GFY_WALL_THETA_TRUNK_DEG");return (e?std::atof(e):35.)*0.017453292519943295;}();return v;}
inline double wallMinEdge(){static const double v=[]{const char* e=std::getenv("GFY_WALL_MIN_EDGE_MM");return (e?std::atof(e):20.)*.001;}();return v;}
inline double wallDepthShare(){static const double v=[]{const char* e=std::getenv("GFY_WALL_DEPTH_SHARE");return e?std::atof(e):.5;}();return v;}
inline int wallSplitLog(){static const int v=[]{const char* e=std::getenv("GFY_WALL_SPLIT_LOG");return e?std::atoi(e):0;}();return v;} // development read-out only
inline double wallMidFloor(){static const double v=[]{const char* e=std::getenv("GFY_WALL_MID_FLOOR");return e?std::atof(e):.05;}();return v;}
inline bool wallExactCut(){static const bool v=[]{const char* e=std::getenv("GFY_WALL_EXACT_CUT");return !e||std::atoi(e)!=0;}();return v;}
template<class WeightField> inline void refineWalls(packedchannel::Input& in,WeightField& field,WallRefineStats& st,std::vector<std::array<double,64>>& weights,const packedchannel::WeightExecutor& executor={}){
 weights.clear();if(in.triangles.empty())return;
 const auto* limbs=field.limbs.get();
 auto rounded=[](const core::Vec& q){return core::Vec{double(float(q.x)),double(float(q.y)),double(float(q.z))};};
 bool sampleFailed=false;
 auto sampleAt=[&](const core::Vec& q,std::array<double,64>& w){bool cached=true;if(!field.sampleCached(rounded(q),w,cached)&&(cached||!field.sample(rounded(q),w))){w.fill(0.);return false;}return true;};
 std::vector<std::array<double,64>>& skin=weights;std::vector<char> known;
 auto ensure=[&](std::uint32_t i){if(skin.size()<in.p.size()){skin.resize(in.p.size());known.resize(in.p.size(),0);}if(!known[i]){if(!sampleAt(in.p[i],skin[i]))sampleFailed=true;known[i]=1;}};
 auto key=[](std::uint32_t a,std::uint32_t b){return a<b?(std::uint64_t(a)<<32)|b:(std::uint64_t(b)<<32)|a;};
 const bool kinds=!in.kinds.empty();bool normalsReady=!in.normals.empty();
 auto ensureNormals=[&]{if(normalsReady)return;normalsReady=true;std::vector<core::Vec> sum(in.p.size());
  for(const auto& t:in.triangles){const core::Vec a=rounded(in.p[t[0]]),b=rounded(in.p[t[1]]),c=rounded(in.p[t[2]]);const core::Vec n=core::cross(c-a,b-a);for(auto id:t)sum[id]=sum[id]+n;}
  in.normals.resize(in.p.size());for(std::size_t i=0;i<sum.size();++i)in.normals[i]=packedchannel::unit(sum[i]);};
 auto addPoint=[&](const core::Vec& q,std::uint32_t a,std::uint32_t b,double s){const auto id=std::uint32_t(in.p.size());in.p.push_back(q);
  const core::Vec n=in.normals[a]*(1-s)+in.normals[b]*s;const double l=core::length(n);in.normals.push_back(l>1e-9?n*(1/l):in.normals[a]);
  if(in.p.size()>=std::size_t(1)<<31)throw std::runtime_error("channel_split_capacity");return id;};
 auto phase0=std::chrono::steady_clock::now();
 for(std::uint32_t i=0;i<in.p.size();++i)ensure(i);
 st.ms_sample+=packedchannel::batchMs(phase0);phase0=std::chrono::steady_clock::now();
 // the limb a corner leans to (> limbTearShare after the side gate; 0: none, or no limb data): the tear's test
 std::vector<std::uint8_t> domCache;
 auto domOf=[&](std::uint32_t i)->std::uint8_t{if(!limbs)return 0;if(domCache.size()<in.p.size())domCache.resize(in.p.size(),254);if(domCache[i]!=254)return domCache[i];ensure(i);
  return domCache[i]=fastwound::leanSide(*limbs,skin[i]);};
 // 1. exact tear line where the corners' limb side changes
 if(limbs&&limbs->chainScores&&wallExactCut()){
  std::vector<std::uint8_t> side(in.p.size());for(std::uint32_t i=0;i<in.p.size();++i)side[i]=domOf(i);
  std::unordered_map<std::uint64_t,std::uint32_t> cut;bool any=false;
  for(std::size_t ti=0;ti<in.triangles.size();++ti){const auto t=in.triangles[ti];for(int e=0;e<3;++e){
   const std::uint32_t a=std::min(t[e],t[(e+1)%3]),b=std::max(t[e],t[(e+1)%3]);const auto ca=side[a],cb=side[b];
   if(!ca||!cb||ca==cb)continue;const auto k=key(a,b);if(cut.count(k))continue;
   const core::Vec A=in.p[a],B=in.p[b];double lo=0,hi=1;
   for(int it=0;it<14;++it){const double mid=.5*(lo+hi);std::array<double,32> sc;std::uint8_t e2=255,c2=0;limbs->chainScores(rounded(A+(B-A)*mid),sc,e2,c2);if(sc[ca]<=sc[cb])lo=mid;else hi=mid;}
   const double s=.5*(lo+hi);if(s<.03||s>.97){cut.emplace(k,~0u);continue;}
   ensureNormals();any=true;const auto id=addPoint(A+(B-A)*s,a,b,s);side.push_back(0);ensure(id);cut.emplace(k,id);++st.cut_edges;}}
  if(any){std::vector<std::array<std::uint32_t,3>> next;std::vector<std::uint32_t> nextKinds;next.reserve(in.triangles.size()+2*cut.size());
   for(std::size_t i=0;i<in.triangles.size();++i){const auto t=in.triangles[i];
    auto emit=[&](std::uint32_t a,std::uint32_t b,std::uint32_t c){next.push_back({a,b,c});if(kinds)nextKinds.push_back(in.kinds[i]);};
    std::uint32_t m[3];int n=0;for(int e=0;e<3;++e){const auto it=cut.find(key(t[e],t[(e+1)%3]));m[e]=it==cut.end()?~0u:it->second;n+=m[e]!=~0u;}
    if(n==0){emit(t[0],t[1],t[2]);continue;}
    if(n==1){int e=0;while(m[e]==~0u)++e;emit(t[e],m[e],t[(e+2)%3]);emit(m[e],t[(e+1)%3],t[(e+2)%3]);continue;}
    if(n==2){int u=0;while(m[u]!=~0u)++u; // edge t[u]-t[u+1] uncut: corner t[u+2] is cut off, the rest is a quad
     const auto A=t[u],B=t[(u+1)%3],C=t[(u+2)%3],m1=m[(u+1)%3],m2=m[(u+2)%3];emit(m1,C,m2);
     if(core::length(in.p[A]-in.p[m1])<=core::length(in.p[B]-in.p[m2])){emit(A,B,m1);emit(A,m1,m2);}else{emit(A,B,m2);emit(B,m1,m2);}continue;}
    emit(t[0],m[0],m[2]);emit(m[0],t[1],m[1]);emit(m[1],t[2],m[2]);emit(m[0],m[1],m[2]);}
   in.triangles=std::move(next);if(kinds)in.kinds=std::move(nextKinds);}}
 st.ms_cut+=packedchannel::batchMs(phase0);
 // 2. posed-error refinement
 const double tol=wallTolerance(),thL=wallThetaLimb(),thT=wallThetaTrunk(),minEdge=wallMinEdge(),cell=fastwound::WallWeights::h;
 const double sampleEdge2=wallSampleEdge()>0?wallSampleEdge()*wallSampleEdge():-1.;
 auto thetaOf=[&](unsigned j){return limbs&&limbs->boneChain[j]?thL:thT;};
 auto pathOf=[&](unsigned k,unsigned l,const core::Vec* m,double& lever,double& turn){lever=0;turn=0;
  if(!limbs){turn=2*thL;if(m)lever=.3*thL;return;}
  std::array<unsigned char,64> ak{};for(unsigned x=k,s=0;s<64&&x<64;++s){ak[x]=1;x=limbs->parent[x];}
  unsigned y=l;for(unsigned s=0;s<64&&y<64&&!ak[y];++s){turn+=thetaOf(y);if(m)lever+=thetaOf(y)*core::length(*m-limbs->pivot[y]);y=limbs->parent[y];}
  for(unsigned x=k,s=0;s<64&&x<64&&x!=y;++s){turn+=thetaOf(x);if(m)lever+=thetaOf(x)*core::length(*m-limbs->pivot[x]);x=limbs->parent[x];}};
 auto maxPair=[&](const std::array<double,64>& v,const core::Vec* m,double& lever,double& turn){lever=0;turn=0;unsigned pos[64],neg[64];int np=0,nn=0;
  for(unsigned b=0;b<64;++b){if(v[b]>1e-4)pos[np++]=b;else if(v[b]<-1e-4)neg[nn++]=b;}
  for(int i=0;i<np;++i)for(int j=0;j<nn;++j){double r=0,q=0;pathOf(pos[i],neg[j],m,r,q);lever=std::max(lever,r);turn=std::max(turn,q);}};
 // a wall deep inside may bow more than one near the surface: coarse depth of the midpoint = the largest r of 4/8/16/32 mm
 // with the point and its six axis neighbours at r all inside the closure (the render field, body to the clothing); an edge
 // whose bound is within GFY_WALL_DEPTH_SHARE of that depth is not split
 const auto* closure=field.asset.closure_field.get();const double depthShare=wallDepthShare();
 auto depthAt=[&](const core::Vec& m)->double{if(!closure||closure->sample(m.x,m.y,m.z)==0u)return 0.;double d=0;
  for(const double r:{.004,.008,.016,.032}){bool all=true;
   for(int ax=0;ax<3&&all;++ax)for(int sg=-1;sg<=1&&all;sg+=2){core::Vec q=m;if(ax==0)q.x+=sg*r;else if(ax==1)q.y+=sg*r;else q.z+=sg*r;all=closure->sample(q.x,q.y,q.z)!=0u;}
   if(!all)break;d=r;}return d;};
 std::unordered_map<std::uint64_t,char> decided;
 auto edgeNeeds=[&](std::uint32_t a,std::uint32_t b)->bool{const auto k=key(a,b);const auto it=decided.find(k);if(it!=decided.end())return it->second!=0;
  const auto tick=std::chrono::steady_clock::now();
  const core::Vec A=rounded(in.p[a]),B=rounded(in.p[b]);const double L=core::length(B-A);bool split=false;
  const auto da=domOf(a),db=domOf(b);const bool torn=da&&db&&da!=db; // the tear re-skins one end for the other's side: no split
  // an edge outside the body everywhere (closure label 0 at both ends and at its quarter points: the bore's air part before
  // the entry and after the exit) is discarded by the shader in every pose: never split
  const bool hidden=L>minEdge&&!torn&&closure&&[&]{for(const double s:{0.,.25,.5,.75,1.}){const core::Vec q=A+(B-A)*s;if(closure->sample(q.x,q.y,q.z)!=0u)return false;}return true;}();
  if(hidden)++st.hidden;
  if(L>minEdge&&!torn&&!hidden){ensure(a);ensure(b);std::array<double,64> D;double dl1=0;for(unsigned q=0;q<64;++q){D[q]=skin[b][q]-skin[a][q];dl1+=std::abs(D[q]);}
   double E=0;if(dl1>1e-6){double lever=0,turn=0;maxPair(D,nullptr,lever,turn);E+=.125*dl1*turn*L;}const bool byEnds=E>tol;
   const double depth=depthShare>0?depthAt((A+B)*.5):0.,allow=std::max(tol,depthShare*depth);
   // the midpoint's own skin (a new lattice node to compute, the costly part) only for walls near the surface
   if(E<=allow&&L>cell&&depth<wallMidDepth()){const core::Vec M=(A+B)*.5;std::array<double,64> wm;sampleAt(M,wm);double ml1=0;
    for(unsigned q=0;q<64;++q){wm[q]-=.5*(skin[a][q]+skin[b][q]);ml1+=std::abs(wm[q]);}
    if(ml1>wallMidFloor()){double lever=0,turn=0;maxPair(wm,&M,lever,turn);E+=.5*(ml1-wallMidFloor())*lever;}}
   split=E>allow;if(!split&&E>tol)++st.deep;
   ++st.evaluated;if(split){if(byEnds)++st.by_ends;else ++st.by_mid;}
   if(wallSplitLog()){static std::atomic<int> logged{0};if(logged.fetch_add(1)<wallSplitLog()){std::array<double,64> wm{};sampleAt((A+B)*.5,wm);unsigned ba=0,bb=0;for(unsigned q=1;q<64;++q){if(skin[a][q]>skin[a][ba])ba=q;if(skin[b][q]>skin[b][bb])bb=q;}
     std::cerr<<"WALLSPLIT owner="<<in.owner<<" split="<<split<<" ends="<<byEnds<<" L_mm="<<L*1000<<" dl1="<<dl1<<" E_mm="<<E*1000<<" depth_mm="<<depthAt((A+B)*.5)*1000<<" a=("<<A.x<<","<<A.y<<","<<A.z<<") b=("<<B.x<<","<<B.y<<","<<B.z<<") top_a="<<ba<<":"<<skin[a][ba]<<" top_b="<<bb<<":"<<skin[b][bb]<<std::endl;}}}
  decided.emplace(k,char(split));st.ms_eval+=packedchannel::batchMs(tick);return split;};
 auto len2=[&](std::uint32_t a,std::uint32_t b){const core::Vec d=rounded(in.p[b])-rounded(in.p[a]);return core::dot(d,d);};
 auto longest=[&](const std::array<std::uint32_t,3>& t){int best=0;double bl=-1;std::uint64_t bk=0;
  for(int e=0;e<3;++e){const double l=len2(t[e],t[(e+1)%3]);const auto k=key(t[e],t[(e+1)%3]);if(l>bl||(l==bl&&k>bk)){bl=l;bk=k;best=e;}}return best;};
 // Rivara rounds in place: a split triangle keeps its index for its first part, the other parts are appended; the edge map
 // (edge -> its two triangles, for the conforming closure) is built once and updated with the splits; only the parts made
 // in a round are tested in the next one.
 auto& T=in.triangles;
 std::unordered_map<std::uint64_t,std::array<std::uint32_t,2>> edgeTris;bool haveEdges=false;
 auto addEdges=[&](std::uint32_t i){const auto& t=T[i];for(int e=0;e<3;++e){auto& s=edgeTris.try_emplace(key(t[e],t[(e+1)%3]),std::array<std::uint32_t,2>{~0u,~0u}).first->second;if(s[0]==~0u)s[0]=i;else if(s[1]==~0u)s[1]=i;}};
 auto dropEdges=[&](std::uint32_t i){const auto& t=T[i];for(int e=0;e<3;++e){const auto it=edgeTris.find(key(t[e],t[(e+1)%3]));if(it==edgeTris.end())continue;auto& s=it->second;
   if(s[0]==i)s[0]=~0u;else if(s[1]==i)s[1]=~0u;if(s[0]==~0u&&s[1]==~0u)edgeTris.erase(it);}};
 std::vector<std::uint32_t> work(T.size());for(std::uint32_t i=0;i<T.size();++i)work[i]=i;
 for(int round=0;round<64&&!work.empty();++round){std::unordered_set<std::uint64_t> marked;std::vector<std::uint64_t> queue;
  auto tr0=std::chrono::steady_clock::now();
  if(wallPrefetch()){// the lattice nodes of this round's candidate midpoints, computed side by side on the production executor (the prefetch)
   std::vector<core::Vec> pts;for(const auto i:work){const auto& t=T[i];for(int e=0;e<3;++e){const auto a=t[e],b=t[(e+1)%3];if(decided.count(key(a,b)))continue;
     const core::Vec A=rounded(in.p[a]),B=rounded(in.p[b]);if(core::length(B-A)<=minEdge)continue;const core::Vec M=rounded((A+B)*.5);
     if(closure&&closure->sample(M.x,M.y,M.z)==0u&&closure->sample(A.x,A.y,A.z)==0u&&closure->sample(B.x,B.y,B.z)==0u)continue;pts.push_back(M);}}
   if(pts.size()>8){packedchannel::WeightBatchStats ps;(void)packedchannel::prefetchWithField(field,pts,ps,true,executor);}}
  st.ms_prefetch+=packedchannel::batchMs(tr0);tr0=std::chrono::steady_clock::now();
  for(const auto i:work){const auto& t=T[i];bool need=false;for(int e=0;e<3&&!need;++e)need=edgeNeeds(t[e],t[(e+1)%3]);
   if(need){const int e=longest(t);const auto k=key(t[e],t[(e+1)%3]);if(marked.insert(k).second)queue.push_back(k);}}
  st.ms_worklist+=packedchannel::batchMs(tr0);tr0=std::chrono::steady_clock::now();
  work.clear();if(marked.empty())break;
  auto tq=std::chrono::steady_clock::now();
  if(!haveEdges){edgeTris.reserve(T.size()*2);for(std::uint32_t i=0;i<T.size();++i)addEdges(i);haveEdges=true;}
  st.ms_map+=packedchannel::batchMs(tq);tq=std::chrono::steady_clock::now();
  std::vector<std::uint32_t> touched;std::unordered_set<std::uint32_t> seen;
  while(!queue.empty()){const auto k=queue.back();queue.pop_back();const auto it=edgeTris.find(k);if(it==edgeTris.end())continue;
   for(const auto ti:it->second){if(ti==~0u)continue;if(seen.insert(ti).second)touched.push_back(ti);const auto& t=T[ti];const int e=longest(t);const auto k2=key(t[e],t[(e+1)%3]);if(marked.insert(k2).second)queue.push_back(k2);}}
  st.ms_bfs+=packedchannel::batchMs(tq);tq=std::chrono::steady_clock::now();
  ensureNormals();++st.rounds;
  std::vector<std::uint64_t> order(marked.begin(),marked.end());std::sort(order.begin(),order.end());
  std::unordered_map<std::uint64_t,std::uint32_t> mid;mid.reserve(order.size());
  if(executor&&order.size()>16&&sampleEdge2<1e300){// the new midpoints' lattice nodes side by side on the production executor (exactly the nodes the
   // next ensure() calls need; one by one they were most of the refinement's time)
   std::vector<core::Vec> pts;pts.reserve(order.size());for(const auto k:order){const auto a=std::uint32_t(k>>32),b=std::uint32_t(k&0xffffffffull);if(len2(a,b)>sampleEdge2)pts.push_back(rounded((in.p[a]+in.p[b])*.5));}
   const auto tp=std::chrono::steady_clock::now();packedchannel::WeightBatchStats ps;(void)packedchannel::prefetchWithField(field,pts,ps,true,executor,32);st.ms_prefetch+=packedchannel::batchMs(tp);}
  // a new point on an edge of at most GFY_WALL_SAMPLE_EDGE_MM (default 0 = never; 40 measured: no time saved, most split edges are
  // longer, and the arms-up armpit got worse) would take the mean of its ends' skins - (two lattice cells, where the field is close to
  // linear) takes the mean of its ends' skins - what the flat triangle carried there, posed now as a point of its own, which
  // is what removes the bow; on longer edges the field is sampled (a new lattice node, the costly part)
  for(const auto k:order){const auto a=std::uint32_t(k>>32),b=std::uint32_t(k&0xffffffffull);const bool interp=len2(a,b)<=sampleEdge2;const auto id=addPoint((in.p[a]+in.p[b])*.5,a,b,.5);
   if(interp){ensure(a);ensure(b);if(skin.size()<in.p.size()){skin.resize(in.p.size());known.resize(in.p.size(),0);}for(unsigned q=0;q<64;++q)skin[id][q]=.5*(skin[a][q]+skin[b][q]);known[id]=1;++st.interpolated;}
   else ensure(id);mid.emplace(k,id);}
  st.ms_mid+=packedchannel::batchMs(tq);tq=std::chrono::steady_clock::now();
  std::sort(touched.begin(),touched.end());
  for(const auto i:touched){const auto t=T[i];dropEdges(i);const int e=longest(t);const std::uint32_t a=t[e],b=t[(e+1)%3],c=t[(e+2)%3],m=mid.at(key(a,b));
   std::array<std::array<std::uint32_t,3>,4> parts;int np=0;
   const auto bc=mid.find(key(b,c));if(bc==mid.end())parts[std::size_t(np++)]={m,b,c};else{parts[std::size_t(np++)]={m,b,bc->second};parts[std::size_t(np++)]={m,bc->second,c};}
   const auto ca=mid.find(key(c,a));if(ca==mid.end())parts[std::size_t(np++)]={a,m,c};else{parts[std::size_t(np++)]={a,m,ca->second};parts[std::size_t(np++)]={ca->second,m,c};}
   T[i]=parts[0];addEdges(i);work.push_back(i);
   for(int q=1;q<np;++q){T.push_back(parts[std::size_t(q)]);if(kinds){const auto kd=in.kinds[i];in.kinds.push_back(kd);}const auto ni=std::uint32_t(T.size()-1);addEdges(ni);work.push_back(ni);}}
  st.ms_emit+=packedchannel::batchMs(tq);st.ms_leb+=packedchannel::batchMs(tr0);}
 {const auto tf=std::chrono::steady_clock::now();for(std::uint32_t i=0;i<in.p.size();++i)ensure(i);st.ms_final+=packedchannel::batchMs(tf);}
 if(sampleFailed)weights.clear(); // packWithField samples again and reports the failure exactly as before
}

template<class Asset,class WeightField> Prepared prepareChannels(const std::shared_ptr<const State>&prior,
 const std::shared_ptr<const gfy_live::ResolvedCutHistory>&history,const std::shared_ptr<const Asset>&asset,
 const gfy_live::Identity&expectedOwner,WeightField&field,const packedchannel::WeightExecutor&executor={},double tolerance=.009){
 const auto begin=std::chrono::steady_clock::now();Prepared out;
 if(!asset||!history||&field.asset!=static_cast<const void*>(asset.get()))throw std::runtime_error("channel_asset_field_identity");
 if(!sameOwner(history->owner,expectedOwner)||expectedOwner.source!=asset->source_hash||expectedOwner.exact_rig!=asset->rig_hash)throw std::runtime_error("channel_asset_owner_identity");
 if(!std::isfinite(tolerance)||tolerance<=0||asset->source.bones==0||asset->source.bones>64||!asset->original_children||asset->original_children>=1024||asset->source_child>=asset->original_children)throw std::runtime_error("channel_asset_quality_range");
 if(history->grid_pitch!=asset->base.pitch||history->grid_origin.x!=asset->base.origin.x||history->grid_origin.y!=asset->base.origin.y||history->grid_origin.z!=asset->base.origin.z||history->grid_size!=std::array<std::uint32_t,3>{std::uint32_t(asset->base.size.x),std::uint32_t(asset->base.size.y),std::uint32_t(asset->base.size.z)})throw std::runtime_error("channel_history_grid_identity");
 const unsigned unsupported=history->limitations&~unsigned(gfy_live::resolved_cut_edge_repairs);if(unsupported)throw std::runtime_error("channel_unsupported_history_baseline_required");
 bool reuse=prior&&prior->asset_pin.get()==asset.get()&&prior->tolerance==tolerance;std::vector<std::shared_ptr<const gfy_live::ResolvedCutHistory>> suffix;
 for(auto p=history;p&&(!reuse||p!=prior->history);p=p->parent){suffix.push_back(p);const auto previous=p->parent?p->parent->total_cuts:0;
  if(!sameOwner(p->owner,history->owner)||previous>gfy_live::ResolvedCutHistory::max_cuts||p->appended.size()>gfy_live::ResolvedCutHistory::max_cuts-previous||p->total_cuts!=previous+p->appended.size()||p->total_cuts>gfy_live::ResolvedCutHistory::max_cuts||(p->parent&&((p->parent->limitations&~p->limitations)!=0||p->parent->damage_revision>=p->damage_revision)))throw std::runtime_error("channel_history_chain");
  if(p->grid_pitch!=history->grid_pitch||p->grid_size!=history->grid_size||p->grid_origin.x!=history->grid_origin.x||p->grid_origin.y!=history->grid_origin.y||p->grid_origin.z!=history->grid_origin.z)throw std::runtime_error("channel_history_grid_changed");
  if(!p->parent&&reuse){reuse=false;break;}}
 if(reuse&&prior->total_cuts!=prior->history->total_cuts)throw std::runtime_error("channel_prefix_count");
 std::reverse(suffix.begin(),suffix.end());auto next=std::make_shared<State>();next->asset_pin=asset;next->history=history;next->tolerance=tolerance;next->total_cuts=history->total_cuts;
 next->coverage_requirements=analytic_voxel_union_unproven|skin_cavity_rim_unproven|body_visibility_required|(history->limitations&gfy_live::resolved_cut_edge_repairs?edge_repairs_unrepresented:0);
 if(reuse){next->children=prior->children;next->triangles=prior->triangles;}
 std::vector<packedchannel::Input>inputs;std::vector<core::Vec>points;std::size_t owner=reuse?prior->total_cuts:0;auto phase=std::chrono::steady_clock::now();
 std::vector<const gfy_live::ResolvedTunnel*> planned;for(const auto&node:suffix)for(const auto&resolved:node->appended)planned.push_back(&resolved);
 // The new cuts' local plans are pure functions of their cutters: planned side by side on the executor, then merged in
 // history order with the same owner numbers, point list and failure order as the former serial loop.
 std::vector<channelbirth::Plan> plans(planned.size());std::vector<std::exception_ptr> planErrors(planned.size());
 auto planOne=[&](std::size_t k){try{if(planned[k]->surface){plans[k]=surfacePlan(*planned[k]->surface);return;}ResolvedView cut(*planned[k]);plans[k]=channelbirth::planLocal(cut,tolerance);}catch(...){planErrors[k]=std::current_exception();}};
 if(executor&&planned.size()>1)executor(unsigned(planned.size()),[&](unsigned k){planOne(k);});else for(std::size_t k=0;k<planned.size();++k)planOne(k);
 for(std::size_t k=0;k<planned.size();++k){if(planErrors[k])std::rethrow_exception(planErrors[k]);ResolvedView cut(*planned[k]);auto&plan=plans[k];if(!plan.admitted)throw std::runtime_error("channel_plan_refused");packedchannel::Input input;input.owner=std::uint32_t(owner++);input.entry=cut.entry;input.direction=cut.direction;input.inlet=cut.inlet;input.penetration=cut.penetration;input.exitDistance=cut.exit_distance;input.p=std::move(plan.vertices);input.triangles=std::move(plan.triangles);
  if(const auto*surface=planned[k]->surface.get()){input.kinds.resize(surface->labels.size());for(std::size_t t=0;t<surface->labels.size();++t)input.kinds[t]=3u+(surface->labels[t]&63u);
   input.normals.reserve(surface->normals.size());for(const auto&q:surface->normals)input.normals.push_back({double(q[0]),double(q[1]),double(q[2])});}
  out.stats.planned_triangles+=input.triangles.size();++out.stats.planned_cuts;for(auto p:input.p)points.push_back({double(float(p.x)),double(float(p.y)),double(float(p.z))});inputs.push_back(std::move(input));}
 if(owner!=history->total_cuts)throw std::runtime_error("channel_final_owner_count");out.stats.plan_ms=packedchannel::batchMs(phase);phase=std::chrono::steady_clock::now();
 if(!inputs.empty())packedchannel::prefetchWithField(field,points,out.stats.weights,true,executor);
 out.stats.weight_ms=packedchannel::batchMs(phase);phase=std::chrono::steady_clock::now(); // skin2: weight time again (the E2 comment had swallowed it)
 std::vector<std::vector<std::array<double,64>>> inputWeights(inputs.size()); // skin2: corner weights sampled by refineWalls, reused by packWithField
 {std::size_t before=0,after=0;for(const auto&input:inputs)before+=input.triangles.size();
  if(fastwound::skin2Legacy()){const double limit=wallEdgeLimit();if(limit>0)for(auto&input:inputs)splitLongEdges(input,limit,field);}
  else{WallRefineStats rs;for(std::size_t k=0;k<inputs.size();++k)refineWalls(inputs[k],field,rs,inputWeights[k],executor);out.stats.split_cut_edges=rs.cut_edges;out.stats.split_by_ends=rs.by_ends;out.stats.split_by_mid=rs.by_mid;out.stats.split_rounds=rs.rounds;out.stats.split_deep=rs.deep;out.stats.split_hidden=rs.hidden;out.stats.split_sample_ms=rs.ms_sample;out.stats.split_cut_ms=rs.ms_cut;out.stats.split_eval_ms=rs.ms_eval;if(wallProfile())std::cerr<<"rxrefine prefetch_ms="<<rs.ms_prefetch<<" worklist_ms="<<rs.ms_worklist<<" leb_ms="<<rs.ms_leb<<" eval_ms="<<rs.ms_eval<<" sample_ms="<<rs.ms_sample<<" cut_ms="<<rs.ms_cut<<" map_ms="<<rs.ms_map<<" bfs_ms="<<rs.ms_bfs<<" mid_ms="<<rs.ms_mid<<" emit_ms="<<rs.ms_emit<<" final_ms="<<rs.ms_final<<" interpolated="<<rs.interpolated<<std::endl;} // skin2 W2/W3
  for(const auto&input:inputs)after+=input.triangles.size();out.stats.split_added=after-before;}
 out.stats.split_ms=packedchannel::batchMs(phase);phase=std::chrono::steady_clock::now();
 std::vector<packedchannel::Output>append;
 // Only the last child can accept more triangles. Earlier immutable children
 // remain shared; copying/reindexing the tail is bounded by native capacities.
 unsigned firstChild=asset->original_children;bool tailFirst=false;
 if(!inputs.empty()&&!next->children.empty()){const auto tail=next->children.back();next->children.pop_back();out.stats.copied_tail_rows=tail->geometry.vertices.size();packedchannel::Output copy;copy.mesh.children.push_back(tail->geometry);copy.triangleOwners=tail->owners;append.push_back(std::move(copy));firstChild=tail->geometry.child_index;tailFirst=true;}
 out.stats.reused_sealed_children=next->children.size();const auto material=asset->face_children.empty()?asset->source_child:asset->face_children.front();
 // R68: the small cuts packed side by side with the weight cache read-only; then, in input order, the packets not made yet
 // (large torn surfaces with their own fan-out, cuts needing a cache fill) exactly as before, the first failure rethrown in input
 // order, every packet appended in input order.
 {const std::size_t n=inputs.size();std::vector<packedchannel::Output> packed(n);std::vector<std::exception_ptr> failed(n);std::vector<char> made(n,0);
  std::vector<std::size_t> small;for(std::size_t k=0;k<n;++k)if(inputs[k].triangles.size()<4096)small.push_back(k);
  if(executor&&small.size()>1){std::atomic<std::size_t> nextSmall{0};
   executor(unsigned(std::min<std::size_t>(small.size(),core::parallelWidth())),[&](unsigned){
    struct InlineOnly{bool previous;InlineOnly():previous(core::painterInlineParallel){core::painterInlineParallel=true;}~InlineOnly(){core::painterInlineParallel=previous;}} inlineOnly;
    for(std::size_t q;(q=nextSmall.fetch_add(1,std::memory_order_relaxed))<small.size();){const auto k=small[q];
     try{packed[k]=packedchannel::packWithField(inputs[k],field,asset->source.bones,asset->original_children,material,false,&inputWeights[k]);made[k]=1;}
     catch(const packedchannel::NeedsCacheFill&){}catch(...){failed[k]=std::current_exception();}}});}
  for(std::size_t k=0;k<n;++k){const auto&input=inputs[k];if(failed[k])std::rethrow_exception(failed[k]);
   if(!made[k])packed[k]=packedchannel::packWithField(input,field,asset->source.bones,asset->original_children,material,true,&inputWeights[k]);
   append.push_back(std::move(packed[k]));out.stats.packed_triangles+=input.triangles.size();next->triangles+=input.triangles.size();}}
 out.stats.pack_ms=packedchannel::batchMs(phase);phase=std::chrono::steady_clock::now();
 if(!append.empty()){auto merged=packedchannel::batchPacked(append,std::uint16_t(firstChild),tailFirst&&true);for(std::size_t i=0;i<merged.mesh.children.size();++i){auto child=std::make_shared<ChildPacket>();child->geometry=std::move(merged.mesh.children[i]);child->owners=std::move(merged.owners[i]);next->children.push_back(std::move(child));}}
 out.stats.append_ms=packedchannel::batchMs(phase);out.stats.total_ms=packedchannel::batchMs(begin);out.state=std::move(next);return out;
}
inline gfy_packed::Mesh materialize(const State&state){gfy_packed::Mesh mesh;for(const auto&child:state.children)mesh.children.push_back(child->geometry);return mesh;}
}
