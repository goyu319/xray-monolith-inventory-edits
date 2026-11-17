#pragma once
#include "../Include/xrRender/OriginalSkinProxySource.h"
#include "../Include/xrRender/Kinematics.h"
#include "../xrPhysics/PHElement.h"
#include "../xrPhysics/PhysicsShell.h"
#include <mutex>
#include <cstring>
#include <memory>
#include <string>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>
namespace painter_skin_proxy {
struct AppendStats {bool cache_hit=false;unsigned source_vertices=0,added=0;double preparation_ms=0;};
struct CachedGroup {u16 anchor=65535;std::vector<ModelSphere> spheres;std::vector<ModelBox> boxes;unsigned original_geoms=0;};
struct Cached {std::shared_ptr<const Source> source;std::string rig;std::vector<CachedGroup> groups;u64 access=0;};
// Owner barrier, strictly after genuine preBuild and BEFORE native Build.
// Fitted callers pass the represented-bone map and retain-original payload;
// they must additionally certify that the original skin source is still kept.
inline bool append_impl(CPhysicsShell& shell,IKinematics& k,const BONE_P_MAP& owners,
 const std::vector<PHFittedBone>* fitted=nullptr,const std::vector<u16>* certified_groups=nullptr,AppendStats* stats=nullptr){
 const auto start=std::chrono::steady_clock::now();
 auto source=k.LL_OriginalSkinProxySource();if(!source||source->vertices.empty()||(fitted&&(!certified_groups||certified_groups->empty())))return false;
 if(source->vertices.size()>16384||source->rig.size()!=k.LL_BoneCount()||k.LL_BoneCount()>64||owners.size()!=k.LL_BoneCount()||shell.isActive())return false;
 std::string rig;auto bytes=[&](const void* data,size_t size){rig.append(static_cast<const char*>(data),size);};
 for(u16 b=0;b<k.LL_BoneCount();++b){if(!owners.count(b))return false;const auto& bd=k.LL_GetData(b);const auto& original=source->rig[b];if(std::memcmp(original.model_to_bone.data(),&bd.m2b_transform,64)||std::memcmp(original.shape.data(),&bd.shape,112)||original.joint_type!=bd.IK_data.type||original.parent!=bd.GetParentID())return false;bytes(&bd.m2b_transform,sizeof(bd.m2b_transform));bytes(&bd.shape,sizeof(bd.shape));const u16 owner=owners.at(b).element?owners.at(b).element->m_SelfID:u16(-1);bytes(&owner,sizeof(owner));}
 static std::mutex mutex;static std::vector<Cached> cache;static u64 clock=0;std::lock_guard<std::mutex> lock(mutex);Cached* entry=nullptr;
 for(auto& item:cache)if(item.source==source&&item.rig==rig){entry=&item;break;}
 if(stats){stats->cache_hit=entry!=nullptr;stats->source_vertices=unsigned(source->vertices.size());}
 if(!entry){Cached pending;pending.source=source;pending.rig=std::move(rig);
  for(const auto& group:source->prepared){auto* e=shell.get_Element(group.anchor);if(!e||(group.spheres.empty()&&group.boxes.empty())||(!group.spheres.empty()&&!group.boxes.empty())||group.spheres.size()+group.boxes.size()>40)continue;bool same_owner=true;unsigned authored_geoms=0;for(u16 b=0;b<source->rig.size();++b)if(source->rig[b].owner==group.anchor){if(!owners.count(b)||owners.at(b).element!=e){same_owner=false;break;}const auto& shape=k.LL_GetData(b).shape;if(shape.type>0&&shape.type<=SBoneShape::stCylinder&&!shape.flags.test(SBoneShape::sfNoPhysics))++authored_geoms;}if(same_owner&&e->numberOfGeoms()==authored_geoms)pending.groups.push_back({group.anchor,group.spheres,group.boxes,authored_geoms});}
  // Strong immutable-source ownership prevents pointer reuse after model pool
  // eviction. Exact original bind/shape/physical-owner bytes qualify the rig.
  // Bounded cache; entries contain no IKinematics/shell/native-element pointer.
  if(cache.size()>=32){auto old=std::min_element(cache.begin(),cache.end(),[](const Cached&a,const Cached&b){return a.access<b.access;});cache.erase(old);}cache.push_back(std::move(pending));entry=&cache.back();
 }
 entry->access=++clock;unsigned geoms=0;for(u16 i=0;i<shell.get_ElementsNumber();++i)geoms+=shell.get_ElementByStoreOrder(i)->numberOfGeoms();
 std::vector<std::pair<CPhysicsElement*,const CachedGroup*>> admitted;unsigned additions=0;
 if(stats)stats->preparation_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
 std::vector<const CachedGroup*> ordered;for(const auto& g:entry->groups)ordered.push_back(&g);const u16 head=k.LL_BoneID("bip01_head");std::stable_sort(ordered.begin(),ordered.end(),[&](const CachedGroup* a,const CachedGroup* b){return a->anchor==head&&b->anchor!=head;});
 for(const auto* item:ordered){const auto& group=*item;auto* e=shell.get_Element(group.anchor);if(!e||e->numberOfGeoms()!=group.original_geoms)continue;if(fitted){if(std::find(certified_groups->begin(),certified_groups->end(),group.anchor)==certified_groups->end())continue;auto found=std::find_if(fitted->begin(),fitted->end(),[&](const PHFittedBone& b){return b.bone==group.anchor;});if(found==fitted->end()||!found->retain_original_shape)continue;}if(geoms+additions+group.spheres.size()+group.boxes.size()>128)continue;additions+=unsigned(group.spheres.size()+group.boxes.size());admitted.emplace_back(e,&group);}
 if(!additions||geoms+additions>128)return false;
 struct NativeGroup {CPhysicsElement* element;u16 bone;Flags16 flags;std::vector<Fsphere> spheres;std::vector<Fobb> boxes;};
 std::vector<NativeGroup> prepared;prepared.reserve(admitted.size());
 for(const auto& group:admitted){const auto& bd=k.LL_GetData(group.second->anchor);NativeGroup n{group.first,group.second->anchor,bd.shape.flags,{}};n.spheres.reserve(group.second->spheres.size());for(const auto& ball:group.second->spheres){Fvector model;model.set(float(ball.center[0]),float(ball.center[1]),float(ball.center[2]));Fsphere s;bd.m2b_transform.transform_tiny(s.P,model);s.R=std::nextafter(float(ball.radius+2e-7),std::numeric_limits<float>::infinity());n.spheres.push_back(s);}n.boxes.reserve(group.second->boxes.size());for(const auto& box:group.second->boxes){Fobb model;model.m_rotate.i.set(float(box.axes[0]),float(box.axes[1]),float(box.axes[2]));model.m_rotate.j.set(float(box.axes[3]),float(box.axes[4]),float(box.axes[5]));model.m_rotate.k.set(float(box.axes[6]),float(box.axes[7]),float(box.axes[8]));model.m_translate.set(float(box.center[0]),float(box.center[1]),float(box.center[2]));model.m_halfsize.set(std::nextafter(float(box.halfsize[0]+2e-7),std::numeric_limits<float>::infinity()),std::nextafter(float(box.halfsize[1]+2e-7),std::numeric_limits<float>::infinity()),std::nextafter(float(box.halfsize[2]+2e-7),std::numeric_limits<float>::infinity()));Fobb local;local.transform(model,bd.m2b_transform);n.boxes.push_back(local);}prepared.push_back(std::move(n));}
 bool applied=false;
 for(const auto& group:prepared){auto* e=group.element;const dMass before=*e->getMassTensor();if(group.boxes.empty()?static_cast<CPHElement*>(e)->try_append_Spheres_prebuild(group.spheres,group.bone,group.flags):static_cast<CPHElement*>(e)->try_append_Boxes_prebuild(group.boxes,group.bone,group.flags)){applied=true;if(stats)stats->added+=unsigned(group.spheres.size()+group.boxes.size());}R_ASSERT(!std::memcmp(&before,e->getMassTensor(),sizeof(before)));}
 return applied;
}
inline bool append(CPhysicsShell& shell,IKinematics& k,const BONE_P_MAP& owners,
 const std::vector<PHFittedBone>* fitted=nullptr,const std::vector<u16>* certified_groups=nullptr,AppendStats* stats=nullptr) noexcept {
 try{return append_impl(shell,k,owners,fitted,certified_groups,stats);}catch(...){return false;}
}
}

