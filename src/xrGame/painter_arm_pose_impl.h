// Included once by PhysicsSkeletonObject.cpp; no new spawn class or backend.
#include "painter_arm_pose.h"
#include "painter_arm_pose_math.h"
#include "script_game_object.h"
#include "ai/stalker/ai_stalker.h"
#include "CharacterPhysicsSupport.h"
#include "inventory.h"
#include "weapon.h"
#include "level.h"
#include "ai_space.h"
#include "alife_simulator.h"
#include "alife_object_registry.h"
#include "../xrPhysics/IPHWorld.h"
#include "../xrServerEntities/clsid_game.h"
#include "../xrEngine/bone.h"

namespace painter_arm_pose_detail {
constexpr u32 count=12, capacity=8;
constexpr LPCSTR sources[2]={
 "painter\\butcher_review\\gfy_sources\\4be67c0445cef074a61fabfabc76db1c5d0cd73f75486a09e15b197f3e714276",
 "painter\\butcher_review\\gfy_sources\\99474d54c1dd24767d057f71e95ac6bbdcb2dae63fb04b0120708addf8c75aa8"};
constexpr LPCSTR sections[2][2]={{"painter_butcher_arm_neutral_left","painter_butcher_arm_neutral_right"},
 {"painter_butcher_arm_coat_left","painter_butcher_arm_coat_right"}};
constexpr LPCSTR visuals[2][2]={{"painter\\detached_arms_v1\\neutral_left_arm","painter\\detached_arms_v1\\neutral_right_arm"},
 {"painter\\detached_arms_v1\\coat_left_arm","painter\\detached_arms_v1\\coat_right_arm"}};
struct Snapshot { Fmatrix world[count]; u32 time=0,frame=0; };
struct Ticket {
 u32 token=0,side=0,model=0,state=0,thread=0;
 CGameObject* owner=nullptr;
 CCharacterPhysicsSupport* support=nullptr;
 u16 owner_id=BI_NONE,fragment_id=BI_NONE;
 shared_str owner_name,owner_section,owner_visual,fragment_name;
 CSE_Abstract* fragment_server=nullptr; // compared only; never dereferenced later
 shared_str names[count];u16 parents[count]={};Fmatrix binds[count];float masses[count]={};
 Snapshot previous,current,sealed;bool has_previous=false;
};
Ticket tickets[capacity];u32 next_token=0;

bool reject(LPCSTR stage,LPCSTR reason,u32 owner=BI_NONE,u32 bone=BI_NONE)
{
 Msg("[butcher_arm_pose] phase=rejected stage=%s reason=%s owner=%u bone=%u time_ms=%u frame=%u",
     stage,reason,owner,bone,Device.dwTimeGlobal,Device.dwFrame);
 return false;
}
bool environment()
{
 if (!g_pGameLevel) return reject("environment","no_level");
 if (!physics_world()) return reject("environment","no_physics_world");
 if (physics_world()->Processing()) return reject("environment","physics_processing");
 if (Device.Paused()) return reject("environment","device_paused");
 string_path app;FS.update_path(app,"$app_data_root$","");
 // FS may preserve the drive slash from fsgame.ltx and append backslashes.
 // Normalize separators only; retain the exact directory and map restriction.
 for(char* p=app;*p;++p) if(*p=='/') *p='\\';
 if (_stricmp(app,"a:\\anomaly_mt_blood_test\\appdata\\")) {
  Msg("[butcher_arm_pose] environment_app=%s",app);
  return reject("environment","private_app_data_root");
 }
 if (xr_strcmp(Level().name().c_str(),"fake_start")) return reject("environment","private_map");
 return true;
}
CAI_Stalker* stalker(CScriptGameObject* object)
{
 if (!object || !object->is_valid()) return nullptr;
 return smart_cast<CAI_Stalker*>(&object->object());
}
bool identity(const Ticket& t, CAI_Stalker* owner)
{
 return owner && t.owner==owner && !owner->getDestroy() && owner->ID()==t.owner_id &&
  owner->cName()==t.owner_name && owner->cNameSect()==t.owner_section && owner->cNameVisual()==t.owner_visual &&
  owner->character_physics_support()==t.support && Level().Objects.net_Find(t.owner_id)==owner;
}
void purge()
{
 for (auto& t:tickets) if (t.token && u32(Device.dwTimeGlobal-t.current.time)>2000) t=Ticket{};
}
Ticket* find(CAI_Stalker* owner,u32 token)
{
 if (!environment()) return nullptr;
 purge();
 for (auto& t:tickets) if (t.token==token && token && t.thread==GetCurrentThreadId() && identity(t,owner)) return &t;
 return nullptr;
}
bool take(CAI_Stalker& owner,u32 side,Snapshot& out)
{
 if (owner.PPhysicsShell()) return reject("take","source_has_shell",owner.ID());
 if (owner.getDestroy()) return reject("take","source_destroying",owner.ID());
 auto* k=smart_cast<IKinematics*>(owner.Visual());
 if (!k) return reject("take","source_not_kinematics",owner.ID());
 if (k->LL_BoneCount()!=47 || k->LL_BoneID("bip01_l_upperarm")!=21 ||
     k->LL_BoneID("bip01_r_upperarm")!=34) return reject("take","source_rig",owner.ID());
 auto* weapon=smart_cast<CWeapon*>(owner.inventory().ActiveItem());
 if (!weapon) return reject("take","active_item_not_weapon",owner.ID());
 if (weapon->H_Parent()!=&owner) return reject("take","weapon_parent",owner.ID());
 if (weapon->strapped_mode()) return reject("take","weapon_strapped",owner.ID());
 if (xr_strcmp(weapon->cNameSect_str(),"wpn_ak74")) return reject("take","weapon_section",owner.ID());
 const u16 root=side ? 34:21;
 // Calculate first, then copy a coherent numeric pose under the same renderer lock.
 xrCriticalSectionGuard guard(&k->UCalc_Mutex);
 k->CalculateBones_Invalidate();k->CalculateBones(TRUE);
 if (!painter_arm_pose_math::rigid(owner.XFORM())) return reject("take","nonrigid_owner_world",owner.ID());
 for (u16 i=0;i<count;++i) {
  if (!k->LL_GetBoneVisible(root+i)) return reject("take","hidden_branch_bone",owner.ID(),root+i);
  out.world[i].mul_43(owner.XFORM(),k->LL_GetBoneInstance(root+i).mTransform);
  if (!painter_arm_pose_math::rigid(out.world[i])) return reject("take","nonrigid_bone_world",owner.ID(),root+i);
 }
 out.time=Device.dwTimeGlobal;out.frame=Device.dwFrame;return true;
}
bool same_matrix(const Fmatrix& a,const Fmatrix& b)
{
 for (u32 i=0;i<4;++i) for(u32 j=0;j<4;++j) if(_abs(a.m[i][j]-b.m[i][j])>1e-5f) return false;
 return true;
}
bool hidden(const Ticket& t,CAI_Stalker& owner)
{
 auto* k=smart_cast<IKinematics*>(owner.Visual());if(!k||k->LL_BoneCount()!=47)return false;
 xrCriticalSectionGuard guard(&k->UCalc_Mutex);
 for(u16 i=0;i<count;++i) if(k->LL_GetBoneVisible((t.side?34:21)+i))return false;
 return !owner.g_Alive();
}
}

struct SPainterArmPoseData {
 struct Child { IKinematics* k=nullptr;u16 parent=BI_NONE;Fmatrix local; } children[12];
 IKinematics* owned_visual=nullptr;
 painter_arm_pose_detail::Ticket snapshot;
 u32 result=0;
};

u32 painter_arm_pose_sample(CScriptGameObject* wrapper,u32 side)
{
 using namespace painter_arm_pose_detail;
 auto* owner=stalker(wrapper);
 if (!environment()) return 0;
 if (!owner) { reject("sample","owner_not_stalker");return 0; }
 if (!owner->g_Alive()) { reject("sample","source_not_alive",owner->ID());return 0; }
 if (side>1) { reject("sample","invalid_side",owner->ID());return 0; }
 purge();
 u32 model=2;for(u32 m=0;m<2;++m)if(!xr_strcmp(owner->cNameVisual().c_str(),sources[m]))model=m;
 if(model==2) {
  Msg("[butcher_arm_pose] sample_source_visual=%s",owner->cNameVisual().c_str());
  reject("sample","source_visual",owner->ID());return 0;
 }
 Snapshot sample;if(!take(*owner,side,sample))return 0;
 Ticket* target=nullptr;
 for(auto& t:tickets) if(t.token&&identity(t,owner)&&t.side==side) {if(t.state!=1){reject("sample","ticket_not_sampling",owner->ID());return 0;}target=&t;break;}
 if(!target) {
  for(auto& t:tickets) if(!t.token){target=&t;break;}
  if(!target){reject("sample","ticket_capacity",owner->ID());return 0;}
  if(next_token==u32(-1)){reject("sample","ticket_serial_exhausted",owner->ID());return 0;} // Never reuse a ticket serial.
  auto& t=*target;t=Ticket{};t.token=++next_token;t.state=1;t.side=side;t.model=model;t.thread=GetCurrentThreadId();
  t.owner=owner;t.support=owner->character_physics_support();t.owner_id=owner->ID();
  t.owner_name=owner->cName();t.owner_section=owner->cNameSect();t.owner_visual=owner->cNameVisual();
  auto* k=smart_cast<IKinematics*>(owner->Visual());const u16 root=side?34:21;
  for(u16 i=0;i<count;++i){const auto& b=k->GetBoneData(root+i);t.names[i]=k->LL_BoneName_dbg(root+i);
   t.parents[i]=i?u16(b.GetParentID()-root):BI_NONE;t.binds[i]=b.get_bind_transform();t.masses[i]=b.get_mass();
   if(i&&t.parents[i]>=i){reject("sample","branch_parent_order",owner->ID(),root+i);t=Ticket{};return 0;}}
  t.current=sample;
 } else if(target->thread!=GetCurrentThreadId()){reject("sample","ticket_thread",owner->ID());return 0;}
 else if(u32(sample.time-target->current.time)>=4) {target->previous=target->current;target->has_previous=true;target->current=sample;}
 return target->token;
}

bool painter_arm_pose_seal(CScriptGameObject* wrapper,u32 token)
{
 using namespace painter_arm_pose_detail;
 auto* owner=stalker(wrapper);auto* t=find(owner,token);if(!t||t->state!=1)return false;
 Snapshot current;if(!take(*owner,t->side,current))return false;
 Snapshot prior=t->current;
 if(u32(current.time-prior.time)<4){if(!t->has_previous)return false;prior=t->previous;}
 const u32 dt=current.time-prior.time;if(dt<4||dt>250)return false;
 t->previous=prior;t->sealed=current;t->state=2;
 Msg("[butcher_arm_pose] owner=%u token=%u side=%u phase=sealed dt_ms=%u frame=%u animated_velocity=true inertia_transfer=false",
  t->owner_id,t->token,t->side,dt,current.frame);return true;
}

bool painter_arm_pose_bind(CScriptGameObject* wrapper,u32 token,u32 fragment_id)
{
 using namespace painter_arm_pose_detail;
 auto* owner=stalker(wrapper);auto* t=find(owner,token);
 if(!t||t->state!=2||fragment_id>=65535||u32(Device.dwTimeGlobal-t->sealed.time)>1000||
    !hidden(*t,*owner)||Level().Objects.net_Find(u16(fragment_id)))return false;
 auto* se=ai().alife().objects().object(u16(fragment_id),true);
 if(!se||se->m_tClassID!=CLSID_PH_SKELETON_OBJECT||se->ID_Parent!=BI_NONE||
    xr_strcmp(se->name(),sections[t->model][t->side]))return false;
 for(const auto& other:tickets)if(other.token&&other.fragment_id==fragment_id)return false;
 t->fragment_id=u16(fragment_id);t->fragment_server=se;t->fragment_name=se->name_replace();t->state=3;
 Msg("[butcher_arm_pose] owner=%u token=%u piece=%u phase=bound_pending_client",t->owner_id,t->token,t->fragment_id);return true;
}

bool painter_arm_pose_cancel(CScriptGameObject* wrapper,u32 token)
{
 auto* t=painter_arm_pose_detail::find(painter_arm_pose_detail::stalker(wrapper),token);
 if(!t)return false;*t=painter_arm_pose_detail::Ticket{};return true;
}
void painter_arm_pose_owner_gone(CGameObject& owner)
{
 for(auto& t:painter_arm_pose_detail::tickets)if(t.token&&t.owner==&owner)t=painter_arm_pose_detail::Ticket{};
}

int painter_arm_pose_begin(CPhysicsSkeletonObject& piece,CSE_Abstract* se,SPainterArmPoseData*& data)
{
 using namespace painter_arm_pose_detail;
 if(!pSettings->line_exist(piece.cNameSect_str(),"painter_arm_pose_required")||
    !pSettings->r_bool(piece.cNameSect_str(),"painter_arm_pose_required"))return 0;
 if(data||!environment())return -1;
 Ticket* match=nullptr;
 for(auto& t:tickets)if(t.token&&t.state==3&&t.fragment_id==piece.ID()&&t.fragment_server==se&&
  t.fragment_name==se->name_replace()&&t.thread==GetCurrentThreadId()) {match=&t;break;}
 if(!match)return -1;
 Ticket t=*match;*match=Ticket{}; // one-shot consumption even on later rejection
 auto* owner=smart_cast<CAI_Stalker*>(Level().Objects.net_Find(t.owner_id));
 auto* po=smart_cast<CSE_PHSkeleton*>(se);auto* k=smart_cast<IKinematics*>(piece.Visual());
 if(!identity(t,owner)||!hidden(t,*owner)||u32(Device.dwTimeGlobal-t.sealed.time)>1000||!po||
    po->_flags.test(CSE_PHSkeleton::flSavedData|CSE_PHSkeleton::flSpawnCopy)||
    xr_strcmp(piece.cNameSect_str(),sections[t.model][t.side])||
    xr_strcmp(piece.cNameVisual().c_str(),visuals[t.model][t.side])||!k||k->LL_BoneCount()!=count)return -1;
 for(u16 i=0;i<count;++i){const auto& b=k->GetBoneData(i);
  if(xr_strcmp(k->LL_BoneName_dbg(i),t.names[i].c_str())||b.GetParentID()!=t.parents[i]||
     _abs(b.get_mass()-t.masses[i])>1e-5f||(i&&!same_matrix(b.get_bind_transform(),t.binds[i])))return -1;
 }
 if(po->saved_bones.root_bone!=0||po->saved_bones.bones_mask!=u64(-1))return -1;
 data=xr_new<SPainterArmPoseData>();data->snapshot=t;data->owned_visual=k;
 data->snapshot.owner=nullptr;data->snapshot.support=nullptr;data->snapshot.fragment_server=nullptr;
 piece.XFORM()=t.sealed.world[0];
 return 1;
}

namespace {
void _BCL painter_arm_frozen_child(CBoneInstance* bone)
{
 // Preserve the native bctPhysics callback parameter: shell hit dispatch casts
 // it to CPhysicsElement. A custom payload here would corrupt hit routing.
 auto* element=static_cast<CPhysicsElement*>(bone->callback_param());
 auto* piece=element?smart_cast<CPhysicsSkeletonObject*>(element->PhysicsRefObject()):nullptr;
 if(piece&&piece->PPhysicsShell()==element->PhysicsShell())piece->PainterArmPoseChild(bone);
}
}

void painter_arm_pose_child(SPainterArmPoseData* data,CBoneInstance* bone)
{
 if(!data||!data->owned_visual)return;
 for(u16 i=2;i<12;++i)if(&data->owned_visual->LL_GetBoneInstance(i)==bone){
  const auto& child=data->children[i];
  bone->mTransform.mul_43(child.k->LL_GetBoneInstance(child.parent).mTransform,child.local);return;
 }
 data->result=2;
}

bool painter_arm_pose_apply(CPhysicsSkeletonObject& piece,SPainterArmPoseData* data)
{
 using namespace painter_arm_pose_detail;
 if(!data)return true;data->result=2;
 auto* shell=piece.PPhysicsShell();auto* k=smart_cast<IKinematics*>(piece.Visual());const auto& t=data->snapshot;
 if(!environment()||GetCurrentThreadId()!=t.thread||!shell||!k||k->LL_BoneCount()!=count||
    !shell->isActive()||shell->isFullActive()||shell->get_ElementsNumber()!=2||shell->get_JointsNumber()!=1)return false;
 CPhysicsElement* elements[2]={shell->get_Element(u16(0)),shell->get_Element(u16(1))};
 Fvector linear[2],angular[2];const float dt=float(t.sealed.time-t.previous.time)/1000.f;
 for(u16 i=0;i<2;++i){auto* e=elements[i];
  // For CPHElement, active && !full-active means flActivating is still set.
  // Refuse an already-consumed callback; never repair a live shell by force.
  if(!e||e->m_SelfID!=i||!e->isActive()||e->isFullActive()||!std::isfinite(e->getMass())||e->getMass()<=0.f||
     !painter_arm_pose_math::velocity(t.previous.world[i],t.sealed.world[i],e->local_mass_Center(),dt,linear[i],angular[i]))return false;
 }
 if(elements[0]==elements[1]||_abs(shell->getMass()-16.f)>.01f)return false;
 xrCriticalSectionGuard guard(&k->UCalc_Mutex);
 Fmatrix inverse_root;inverse_root.invert(t.sealed.world[0]);
 for(u16 i=0;i<count;++i){auto& b=k->LL_GetBoneInstance(i);
  b.mTransform.mul_43(inverse_root,t.sealed.world[i]);
  if(i<2){if(!b.callback()||b.callback_type()!=bctPhysics||b.callback_param()!=elements[i])return false;}
 }
 // The normal shell activation callback seeds each body from its OWN named
 // bone, updates COM/interpolation, clears activating and retains bind-defined
 // joint references. This is before the client's first physics step/render.
 for(u16 i=0;i<2;++i){auto& b=k->LL_GetBoneInstance(i);b.callback()(&b);}
 if(!shell->isFullActive()||!elements[0]->isFullActive()||!elements[1]->isFullActive())return false;
 for(u16 i=2;i<count;++i){auto& child=data->children[i];child.k=k;child.parent=t.parents[i];
  Fmatrix inverse;inverse.invert(t.sealed.world[child.parent]);child.local.mul_43(inverse,t.sealed.world[i]);
  auto& bone=k->LL_GetBoneInstance(i);
  if(bone.callback_type()!=bctPhysics||!bone.callback_param())return false;
  bone.set_callback(bctPhysics,painter_arm_frozen_child,bone.callback_param(),TRUE);
 }
 for(u16 i=0;i<2;++i){elements[i]->set_LinearVel(linear[i]);elements[i]->set_AngularVel(angular[i]);}
 k->CalculateBones_Invalidate();k->CalculateBones(TRUE);
 // Native readback checks every visual bone, both ODE frames and velocities.
 float maximum_error=0.f;
 for(u16 i=0;i<count;++i){Fmatrix world;world.mul_43(piece.XFORM(),k->LL_GetBoneInstance(i).mTransform);
  for(u16 r=0;r<4;++r)for(u16 c=0;c<4;++c)maximum_error=_max(maximum_error,_abs(world.m[r][c]-t.sealed.world[i].m[r][c]));}
 for(u16 i=0;i<2;++i){Fmatrix world;elements[i]->GetGlobalTransformDynamic(&world);Fvector lv,av;
  elements[i]->get_LinearVel(lv);elements[i]->get_AngularVel(av);
  if(!same_matrix(world,t.sealed.world[i])||!lv.similar(linear[i],1e-4f)||!av.similar(angular[i],1e-4f))return false;
 }
 if(!std::isfinite(maximum_error)||maximum_error>1e-4f)return false;
 data->result=1;
 Msg("[butcher_arm_pose] owner=%u piece=%u token=%u side=%u phase=adopted bodies=2 joints=1 mass=%g pose_max_error=%g sample_dt_ms=%u animated_velocity=true exact_momentum_transfer=false",
  t.owner_id,piece.ID(),t.token,t.side,shell->getMass(),maximum_error,t.sealed.time-t.previous.time);
 return true;
}

void painter_arm_pose_release(CPhysicsSkeletonObject& piece,SPainterArmPoseData*& data)
{
 if(!data)return;
 auto* k=data->owned_visual;
 if(k){xrCriticalSectionGuard guard(&k->UCalc_Mutex);
  for(u16 i=2;i<12&&i<k->LL_BoneCount();++i){auto& bone=k->LL_GetBoneInstance(i);
   if(bone.callback()==painter_arm_frozen_child)bone.reset_callback();}}
 xr_delete(data);
}
u32 painter_arm_pose_result(const SPainterArmPoseData* data){return data?data->result:0;}
u32 painter_arm_pose_status(CScriptGameObject* object)
{
 if(!object||!object->is_valid())return 0;
 auto* piece=smart_cast<CPhysicsSkeletonObject*>(&object->object());return piece?piece->PainterArmPoseStatus():0;
}
