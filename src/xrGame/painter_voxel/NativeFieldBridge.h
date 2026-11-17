#pragma once
// Exact frozen algorithm bodies, parameterized only on the existing native field type.
namespace packedchannel {
// R68: packWithField with the weight cache read-only (mayFillCache=false) stops here at the first uncached node.
struct NeedsCacheFill {};
// skin2: preWeights = the corner weights refineWalls already sampled with this field at the same float-rounded points (used
// as they are when complete; the checks below still run)
template<class WeightField> inline Output packWithField(const Input& in,WeightField& field,std::uint32_t boneCount,std::uint16_t firstChild,std::uint16_t materialSource,bool mayFillCache=true,
                                                        const std::vector<std::array<double,64>>* preWeights=nullptr){
 Output out;out.inputPositions=in.p.size();out.inputTriangles=in.triangles.size();
 if(boneCount==0||boneCount>64)throw std::runtime_error("channel_source_bone_limit");
 if(!(in.inlet>=0&&in.penetration>0)||std::abs(core::dot(in.direction,in.direction)-1)>1e-10)throw std::runtime_error("channel_axis_domain");
 if(!in.kinds.empty()&&in.kinds.size()!=in.triangles.size())throw std::runtime_error("channel_kind_count");
 if(!in.normals.empty()&&in.normals.size()!=in.p.size())throw std::runtime_error("channel_normal_count");
 for(std::size_t t=0;t<in.triangles.size();++t)out.triangleOwners.push_back({in.owner,in.kinds.empty()?primitiveKind(in,in.triangles[t]):PrimitiveKind(in.kinds[t])});
 std::vector<core::Vec> p(in.p.size()),normal(in.p.size()),faceNormal(in.triangles.size()),faceRaw(in.triangles.size());out.fullWeights.resize(in.p.size());
 // R64: the per-vertex weights side by side from the prefetched cache (a vertex with an uncached node is sampled serially below:
 // same values); failures throw the same message as the serial loop
 std::vector<char> uncached(in.p.size(),0);
 core::parallelChunks(in.p.size(),2048,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t i=a;i<b;++i){
  p[i]={double(float(in.p[i].x)),double(float(in.p[i].y)),double(float(in.p[i].z))};bool cached=true;
  if(preWeights&&preWeights->size()==in.p.size()){out.fullWeights[i]=(*preWeights)[i];double s=0;for(double x:out.fullWeights[i])s+=x;if(!(s>0))throw std::runtime_error("channel_wall_skin_unavailable");}else
  if(!field.sampleCached(p[i],out.fullWeights[i],cached)){if(!cached){uncached[i]=1;continue;}throw std::runtime_error("channel_wall_skin_unavailable");}
  for(unsigned bb=boneCount;bb<64;++bb)if(out.fullWeights[i][bb]!=0)throw std::runtime_error("channel_bone_outside_source");}});
 for(std::size_t i=0;i<in.p.size();++i)if(uncached[i]){if(!mayFillCache)throw NeedsCacheFill{};
  if(!field.sample(p[i],out.fullWeights[i]))throw std::runtime_error("channel_wall_skin_unavailable");
  for(unsigned b=boneCount;b<64;++b)if(out.fullWeights[i][b]!=0)throw std::runtime_error("channel_bone_outside_source");}
 for(std::size_t k=0;k<in.triangles.size();++k){const auto&t=in.triangles[k];for(auto id:t)if(id>=p.size())throw std::runtime_error("channel_index");}
 // Input cutter points outward. Cavity rendering points INTO removed volume. Face normals side by side; the vertex sums in triangle order.
 core::parallelChunks(in.triangles.size(),4096,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t k=a;k<b;++k){const auto&t=in.triangles[k];
  faceRaw[k]=core::cross(p[t[2]]-p[t[0]],p[t[1]]-p[t[0]]);faceNormal[k]=unit(faceRaw[k]);}});
 for(std::size_t k=0;k<in.triangles.size();++k){const auto&t=in.triangles[k];for(auto id:t)normal[id]=normal[id]+faceRaw[k];}
 core::parallelChunks(normal.size(),4096,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t i=a;i<b;++i)
  normal[i]=!in.normals.empty()&&core::length(in.normals[i])>.5?unit(in.normals[i]):unit(normal[i]);}); // R62: a torn surface's own normals
 gfy_packed::Child child;std::vector<std::uint32_t> positionOfRow;core::packing::VertexIndex index;
 unsigned next=firstChild;std::uint32_t firstTriangle=0;
 auto begin=[&]{if(next>=gfy_packed::max_child_slots)throw std::runtime_error("channel_child_slots");child={};child.child_index=std::uint16_t(next++);child.material_source_child=materialSource;child.material=1;positionOfRow.clear();index.reset(std::min<std::size_t>(3*in.triangles.size(),65535));};
 auto flush=[&]{out.owners.push_back({child.child_index,in.owner,firstTriangle,std::uint32_t(child.indices.size()/3)});firstTriangle+=std::uint32_t(child.indices.size()/3);out.mesh.children.push_back(std::move(child));out.positionOfRow.push_back(std::move(positionOfRow));};
 begin();
 std::unordered_map<std::size_t,std::array<std::array<double,64>,3>> torn;
 if(field.limbs){if(fastwound::skin2Legacy()||!field.limbs->chainScores)fastwound::tearLimbContacts(*field.limbs,p,in.triangles,out.fullWeights,torn);
  else{const auto*limbs=field.limbs.get(); // skin2 W2: tear by side; a corner of the other side re-skinned from its own nodes for this side
   auto regate=[&](const core::Vec&q,std::uint8_t side,std::array<double,64>&o){std::array<double,64> raw;bool cached=true;if(!field.sampleCachedSide(q,raw,cached,side))return false;
    double inSide=0;for(unsigned b=0;b<64;++b)if(limbs->boneChain[b]==side)inSide+=raw[b];if(!(inSide>0))return false;o=raw;return true;};
   fastwound::tearLimbSides(*limbs,p,in.triangles,out.fullWeights,torn,regate);}}
 // R64: every corner's packed row and hash side by side (pure functions of the corner); the dedupe below runs in triangle order
 struct Corner {gfy_packed::Vertex row;std::uint64_t hash;double discarded;bool crease;};std::vector<Corner> corners(in.triangles.size()*3);
 core::parallelChunks(in.triangles.size(),1024,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t t=a;t<b;++t){const auto& tri=in.triangles[t];const std::uint32_t ids[3]={tri[0],tri[2],tri[1]};
  for(unsigned c=0;c<3;++c){auto id=ids[c];core::JoinedVertex j;j.p=p[id];j.w=out.fullWeights[id];if(!torn.empty()){const auto it=torn.find(t);if(it!=torn.end())j.w=it->second[c==0?0u:c==1?2u:1u];}j.sourceFace=~std::uint32_t(0);auto&o=corners[t*3+c];o.discarded=0;
   j.normal=normal[id];o.crease=core::dot(j.normal,faceNormal[t])<.2;if(o.crease)j.normal=faceNormal[t];
   // Native wall tangent choice and 8 cm wall-texture UV convention.
   j.tangent=std::abs(j.normal.x)>.9?core::Vec{0,0,1}:core::Vec{1,0,0};j.binormal=core::cross(j.normal,j.tangent);
   j.uv={core::dot(j.p,j.tangent)/.08,core::dot(j.p,j.binormal)/.08};
   o.row=core::packing::vertex(j,&o.discarded);o.hash=core::packing::vertexHash(o.row);}}});
 for(std::size_t t=0;t<in.triangles.size();++t){const auto& tri=in.triangles[t];const std::uint32_t ids[3]={tri[0],tri[2],tri[1]};
  std::array<gfy_packed::Vertex,3>rows;std::array<std::uint64_t,3>hash;std::array<long,3>found;std::array<double,3>discarded{};unsigned fresh=0;
  for(unsigned c=0;c<3;++c){const auto&o=corners[t*3+c];out.creaseRows+=o.crease;rows[c]=o.row;hash[c]=o.hash;discarded[c]=o.discarded;
   found[c]=index.find(child.vertices,rows[c],hash[c]);if(found[c]<0)++fresh;}
  if(child.vertices.size()+fresh>65535||child.indices.size()/3>=65535){flush();begin();found.fill(-1);}
  for(unsigned c=0;c<3;++c){if(found[c]>=0){child.indices.push_back(std::uint16_t(found[c]));continue;}
   auto inserted=index.insert(child.vertices,rows[c],hash[c]);child.indices.push_back(inserted.first);
   if(inserted.second){positionOfRow.push_back(ids[c]);out.mesh.reduced_skin_vertices+=discarded[c]>1e-12;out.mesh.max_discarded_skin_weight=std::max(out.mesh.max_discarded_skin_weight,discarded[c]);}}
  child.original_faces.push_back(~std::uint32_t(0));}
 flush();std::size_t emitted=0;for(const auto&c:out.mesh.children)emitted+=c.indices.size()/3;
 if(emitted!=in.triangles.size())throw std::runtime_error("channel_triangle_count_changed");return out;
}
template<class WeightField> inline bool prefetchWithField(WeightField& field,const std::vector<Vec>& points,WeightBatchStats& stats,bool coherent=true,const WeightExecutor& executor={},std::size_t perJob=256){
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
 stats.jobs=executor?unsigned(std::max<std::size_t>(1,std::min<std::size_t>(4,(missing.size()+perJob-1)/perJob))):1;/* skin2: perJob, 256 as before unless the caller asks */std::array<WeightBatchStats,4> partial{};
 auto compute=[&](unsigned job){auto&part=partial[job];std::uint32_t seed=~0u;for(std::size_t i=missing.size()*job/stats.jobs;i<missing.size()*(job+1)/stats.jobs;++i){const auto&node=missing[i];Top top{};std::array<double,64> full{};auto tick=std::chrono::steady_clock::now();++part.nearestCalls;if(!coherent)seed=~0u;
  const bool found=fastwound::wallNodeSkin(field.asset,field.limbs.get(),Vec{node.x*WallWeights::h,node.y*WallWeights::h,node.z*WallWeights::h},full,&seed,&part.triangleTests,&part.treeNodes,&part.seedTests);
  part.nearestMs+=batchMs(tick);tick=std::chrono::steady_clock::now(); // skin2: timing again (the E2 comment had swallowed it)
  if(found){std::array<unsigned,64> order;for(unsigned b=0;b<64;++b)order[b]=b;std::stable_sort(order.begin(),order.end(),[&](unsigned a,unsigned b){return full[a]>full[b];});
   double sum=0;for(int j=0;j<6;++j){top[std::size_t(j)]={std::uint16_t(order[std::size_t(j)]),float(full[order[std::size_t(j)]])};sum+=top[std::size_t(j)].second;}if(sum>0)for(auto&e:top)e.second=float(e.second/sum);}
  fresh[i]={node.key,top};part.normalizeMs+=batchMs(tick);}};
 phase=std::chrono::steady_clock::now();if(stats.jobs>1)executor(stats.jobs,compute);else compute(0);stats.computeWallMs=batchMs(phase);
 for(unsigned i=0;i<stats.jobs;++i){const auto&p=partial[i];stats.nearestCalls+=p.nearestCalls;stats.triangleTests+=p.triangleTests;stats.seedTests+=p.seedTests;stats.treeNodes+=p.treeNodes;stats.nearestMs+=p.nearestMs;stats.normalizeMs+=p.normalizeMs;}
 phase=std::chrono::steady_clock::now();{std::lock_guard<std::mutex> lock(field.asset.skin_nodes.mutex);for(auto&node:fresh){if(field.asset.skin_nodes.map.size()>=200000)field.asset.skin_nodes.map.clear();const auto&top=field.asset.skin_nodes.map.try_emplace(node.first,node.second).first->second;field.local.emplace(node.first,top);}}
 stats.publishMs=batchMs(phase);stats.totalMs=batchMs(begin);return true;
}
}
