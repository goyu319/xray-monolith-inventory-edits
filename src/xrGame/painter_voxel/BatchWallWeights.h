#pragma once
// Optional exact prefetch for the existing WallWeights sampler. The immutable
// source BVH and existing bounded shared node cache remain authoritative.
// Spatial coherence supplies ONLY an initial exact source-face distance bound;
// every BVH node that can beat or tie it is still searched with the native rule.
namespace packedchannel {
struct WeightBatchStats {
 std::size_t inputPositions{},requestedNodes{},uniqueNodes{},localHits{},sharedHits{},nearestCalls{},triangleTests{},seedTests{},treeNodes{};
 double gatherMs{},cacheMs{},nearestMs{},normalizeMs{},computeWallMs{},publishMs{},totalMs{};unsigned jobs{1};
};
// Executor must invoke each job exactly once and return only after all jobs
// finish. At most four independent jobs; no threads are created here. The
// production caller supplies its existing global CPU-budget executor.
using WeightExecutor=std::function<void(unsigned,const std::function<void(unsigned)>&)>;
inline double batchMs(std::chrono::steady_clock::time_point begin){return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();}
inline bool coherentNearest(const core::BodyBVH& tree,Vec p,std::array<double,64>& w,std::uint32_t& seed,WeightBatchStats& stats){
 if(tree.nodes.empty())return false;
 double best=1e300;std::uint32_t bestFace=~0u;Vec bestPoint;
 auto face=[&](std::uint32_t f){const auto&ids=tree.source.f[f];const Vec q=core::closestTriangle(p,tree.source.v[ids[0]].p,tree.source.v[ids[1]].p,tree.source.v[ids[2]].p);const double d=dot(q-p,q-p);if(d<best||(d==best&&f<bestFace)){best=d;bestFace=f;bestPoint=q;}};
 if(seed<tree.source.f.size()){face(seed);++stats.seedTests;}
 auto boxDist=[&](const core::Bounds&b){double d=0;for(int a=0;a<3;++a){const double v=std::max({b.lo[a]-p[a],0.,p[a]-b.hi[a]});d+=v*v;}return d;};
 std::vector<int> stack;stack.reserve(64);stack.push_back(0);
 while(!stack.empty()){const int n=stack.back();stack.pop_back();++stats.treeNodes;const auto&node=tree.nodes[std::size_t(n)];if(boxDist(node.bounds)>best)continue;
  if(node.left<0){for(std::size_t i=node.first;i<node.first+node.count;++i){face(tree.order[i]);++stats.triangleTests;}}
  else{const double dl=boxDist(tree.nodes[std::size_t(node.left)].bounds),dr=boxDist(tree.nodes[std::size_t(node.right)].bounds);if(dl<dr){stack.push_back(node.right);stack.push_back(node.left);}else{stack.push_back(node.left);stack.push_back(node.right);}}}
 if(bestFace==~0u)return false;seed=bestFace;
 core::V v;v.p=bestPoint;v.b=core::faceBary(tree.source,bestFace,bestPoint);
 for(auto& b:v.b)b=std::clamp(b,0.,1.);{double sum=v.b[0]+v.b[1]+v.b[2];if(!(sum>0))return false;for(auto&b:v.b)b/=sum;}
 w=core::weights(tree.source,bestFace,v);double total=0;for(double x:w)total+=x;if(!(total>0))return false;for(double&x:w)x/=total;return true;
}
// points must be canonical packed-float positions, as pack() samples them.
// Returns false without prefetch above the existing 200k shared-cache bound;
// caller then uses the unchanged lazy native field. It never changes geometry.
inline bool prefetch(WallWeights& field,const std::vector<Vec>& points,WeightBatchStats& stats,bool coherent=true,const WeightExecutor& executor={}){
 auto begin=std::chrono::steady_clock::now(),phase=begin;stats.inputPositions=points.size();
 struct Node {int x,y,z;std::uint64_t key;};std::unordered_map<std::uint64_t,Node> unique;unique.reserve(std::min<std::size_t>(points.size()*3,200000));
 for(const auto&p:points){const double fx=p.x/WallWeights::h,fy=p.y/WallWeights::h,fz=p.z/WallWeights::h;const int x=int(std::floor(fx)),y=int(std::floor(fy)),z=int(std::floor(fz));const double tx=fx-x,ty=fy-y,tz=fz-z;
  for(int c=0;c<8;++c){const int dx=c&1,dy=(c>>1)&1,dz=(c>>2)&1;const double k=(dx?tx:1-tx)*(dy?ty:1-ty)*(dz?tz:1-tz);if(k<=0)continue;++stats.requestedNodes;
   const auto key=WallWeights::key(x+dx,y+dy,z+dz);unique.try_emplace(key,Node{x+dx,y+dy,z+dz,key});if(unique.size()>200000){stats.totalMs=batchMs(begin);return false;}}}
 std::vector<Node> ordered;ordered.reserve(unique.size());for(const auto&x:unique)ordered.push_back(x.second);std::sort(ordered.begin(),ordered.end(),[](const Node&a,const Node&b){return a.key<b.key;});
 std::size_t added=0;for(const auto&node:ordered)added+=!field.local.count(node.key);if(field.local.size()+added>200000){stats.totalMs=batchMs(begin);return false;}
 stats.uniqueNodes=ordered.size();stats.gatherMs=batchMs(phase);phase=std::chrono::steady_clock::now();
 std::vector<Node> missing;missing.reserve(ordered.size());
 {std::lock_guard<std::mutex> lock(field.asset.skin_nodes.mutex);for(const auto&node:ordered){if(field.local.count(node.key)){++stats.localHits;continue;}const auto found=field.asset.skin_nodes.map.find(node.key);if(found!=field.asset.skin_nodes.map.end()){field.local.emplace(node.key,found->second);++stats.sharedHits;}else missing.push_back(node);}}
 stats.cacheMs=batchMs(phase);
 using Top=std::array<std::pair<std::uint16_t,float>,6>;std::vector<std::pair<std::uint64_t,Top>> fresh(missing.size());
 stats.jobs=executor?unsigned(std::max<std::size_t>(1,std::min<std::size_t>(4,(missing.size()+255)/256))):1;std::array<WeightBatchStats,4> partial{};
 auto compute=[&](unsigned job){auto&part=partial[job];std::uint32_t seed=~0u;for(std::size_t i=missing.size()*job/stats.jobs;i<missing.size()*(job+1)/stats.jobs;++i){const auto&node=missing[i];Top top{};std::array<double,64> full{};auto tick=std::chrono::steady_clock::now();++part.nearestCalls;if(!coherent)seed=~0u;
  const bool found=fastwound::wallNodeSkin(field.asset,field.limbs.get(),Vec{node.x*WallWeights::h,node.y*WallWeights::h,node.z*WallWeights::h},full,&seed,&part.triangleTests,&part.treeNodes,&part.seedTests);
  part.nearestMs+=batchMs(tick);tick=std::chrono::steady_clock::now(); // skin2: timing again (the E2 comment had swallowed it)
  if(found){std::array<unsigned,64> order;for(unsigned b=0;b<64;++b)order[b]=b;const bool signedSkin=gfy_signed_hw::negative(full);std::stable_sort(order.begin(),order.end(),[&](unsigned a,unsigned b){return signedSkin?std::abs(full[a])>std::abs(full[b]):full[a]>full[b];});
   double sum=0;for(int j=0;j<6;++j){top[std::size_t(j)]={std::uint16_t(order[std::size_t(j)]),float(full[order[std::size_t(j)]])};sum+=top[std::size_t(j)].second;}if(sum>0)for(auto&e:top)e.second=float(e.second/sum);}
  fresh[i]={node.key,top};part.normalizeMs+=batchMs(tick);}};
 phase=std::chrono::steady_clock::now();if(stats.jobs>1)executor(stats.jobs,compute);else compute(0);stats.computeWallMs=batchMs(phase);
 for(unsigned i=0;i<stats.jobs;++i){const auto&p=partial[i];stats.nearestCalls+=p.nearestCalls;stats.triangleTests+=p.triangleTests;stats.seedTests+=p.seedTests;stats.treeNodes+=p.treeNodes;stats.nearestMs+=p.nearestMs;stats.normalizeMs+=p.normalizeMs;}
 phase=std::chrono::steady_clock::now();{std::lock_guard<std::mutex> lock(field.asset.skin_nodes.mutex);for(auto&node:fresh){if(field.asset.skin_nodes.map.size()>=200000)field.asset.skin_nodes.map.clear();const auto&top=field.asset.skin_nodes.map.try_emplace(node.first,node.second).first->second;field.local.emplace(node.first,top);}}
 stats.publishMs=batchMs(phase);stats.totalMs=batchMs(begin);return true;
}
}
