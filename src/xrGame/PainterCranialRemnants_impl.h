#pragma once
// Publication-side physics for the ACTUAL damaged head. No fixed replacement,
// rig mutation, bone hiding, or geometry removal. Called at the native publish
// barrier only after the worker output and its wound history were accepted.
#include "painter_voxel/LiveDamageWorker.h"
#include "painter_voxel/CranialSupportPolicy.h"
#include "../xrphysics/PhysicsShell.h"
#include <map>
#include <array>
#include <cmath>
namespace painter_cranial_remnants {
struct JointState {
    CPhysicsShellHolder* owner=nullptr;
    IKinematics* visual=nullptr;
    CPhysicsShell* shell=nullptr;
    CPhysicsJoint* joint=nullptr;
    u64 revision=0;
    float scale=1.f,joint_spring=1.f,joint_damping=1.f;
    u16 axes=0;
    std::array<float,3> low{},high{},spring{},damping{};
    painter_cranial_support::Decision support;
    u16 measured=BI_NONE;
    u64 pending_revision=0;
};
// Function-local static of an external inline function is shared across the
// corpse and fragment translation units. forget() removes the owner on release.
inline std::map<u16,JointState>& states(){static std::map<u16,JointState> value;return value;}
inline void forget(CPhysicsShellHolder& object){
    auto& map=states();auto it=map.find(object.ID());
    if(it!=map.end()&&it->second.owner==&object)map.erase(it);
}
inline bool finite(float x){return std::isfinite(x);}
inline bool capture(JointState& out,CPhysicsShellHolder& owner,CPhysicsShell& shell,CPhysicsJoint& joint){
    const u16 axes=joint.GetAxesNumber();
    if(!axes||axes>3||!joint.PFirst_element()||!joint.PSecond_element())return false;
    JointState state;state.owner=&owner;state.shell=&shell;state.joint=&joint;state.axes=axes;
    joint.GetJointSDfactors(state.joint_spring,state.joint_damping);
    if(!finite(state.joint_spring)||!finite(state.joint_damping)||state.joint_spring<0.f||state.joint_damping<0.f)return false;
    for(u16 a=0;a<axes;++a){
        joint.GetLimits(state.low[a],state.high[a],a);
        joint.GetAxisSDfactors(state.spring[a],state.damping[a],a);
        if(!finite(state.low[a])||!finite(state.high[a])||state.low[a]>state.high[a]||
           !finite(state.spring[a])||!finite(state.damping[a])||state.spring[a]<0.f||state.damping[a]<0.f)return false;
    }
    state.visual=out.visual;state.support=out.support;state.measured=out.measured;state.pending_revision=out.pending_revision;
    // A shell refit must not heal the previously committed support loss.
    state.scale=out.scale;
    out=state;return true;
}
inline bool has_pending(CPhysicsShellHolder& object){
    const auto it=states().find(object.ID());return it!=states().end()&&it->second.owner==&object&&
        (it->second.pending_revision>it->second.revision||object.PPhysicsShell()!=it->second.shell);
}
inline bool poll(CPhysicsShellHolder& object){
    if(object.getDestroy()||!Device.IsGeometryPublishBarrier()||!physics_world()||physics_world()->Processing())return false;
    auto it=states().find(object.ID());if(it==states().end())return false;
    auto& state=it->second;
    if(state.owner!=&object||Level().Objects.net_Find(object.ID())!=&object){states().erase(it);return false;}
    auto* shell=object.PPhysicsShell();auto* k=smart_cast<IKinematics*>(object.Visual());
    if(k!=state.visual){states().erase(it);return false;}
    if(auto* npc=smart_cast<CAI_Stalker*>(&object))if(npc->g_Alive())return false;
    if(!shell||!shell->isFullActive()||!k)return false;
    if(state.pending_revision<=state.revision&&state.shell==shell)return false;
    const u16 head=k->LL_BoneID("bip01_head"),neck=k->LL_BoneID("bip01_neck");
    auto* joint=head!=BI_NONE?shell->get_Joint(head):nullptr;
    if(!joint&&neck!=BI_NONE)joint=shell->get_Joint(neck);
    if(!joint){
        state.shell=shell;state.joint=nullptr;state.revision=state.pending_revision;
        if(painter_log_verbose())Msg("[painter_cranial] id=%u phase=joint_unavailable revision=%llu",object.ID(),state.revision);
        return false;
    }
    const bool new_shell=state.shell!=shell||state.joint!=joint;
    if(new_shell&&!capture(state,object,*shell,*joint)){
        state.shell=shell;state.joint=nullptr;state.revision=state.pending_revision;
        if(painter_log_verbose())Msg("[painter_cranial] id=%u phase=joint_contract_unavailable revision=%llu",object.ID(),state.revision);
        return false;
    }
    state.revision=state.pending_revision;
    const float scale=_min(state.scale,float(state.support.stiffness_scale));
    state.scale=scale;
    constexpr float hang_angle=1.10f;
    joint->SetJointSDfactors(state.joint_spring*scale,state.joint_damping);
    for(u16 a=0;a<state.axes;++a){
        joint->SetAxisSDfactors(state.spring[a]*_max(scale,.35f),state.damping[a],a); // 2026-11-11: stops no softer than 35 %
        joint->SetLimits(_min(state.low[a],-hang_angle),_max(state.high[a],hang_angle),a);
        // 2026-11-11: the corpse's own joint resistance stays (it was zeroed here, so the remnant swung undamped)
    }
    shell->Enable();
    if(painter_log_verbose())Msg("[painter_cranial] id=%u revision=%llu phase=original_joint_hanging measured_bone=%u tissue_left=%.3f bone_left=%.3f stiffness=%.3f",
        object.ID(),state.revision,unsigned(state.measured),state.support.tissue_fraction,state.support.bone_fraction,scale);
    return true;
}
inline bool committed(CPhysicsShellHolder& object,const std::vector<gfy_live::JointDamage>& joints,
    const std::vector<gfy_live::ImpactResult>& impacts,const gfy_live::CranialDamage& cranial,u64 revision){
    if(object.getDestroy())return false;
    auto* k=smart_cast<IKinematics*>(object.Visual());if(!k)return false;
    const u16 head=k->LL_BoneID("bip01_head"),neck=k->LL_BoneID("bip01_neck");
    if(head==BI_NONE||head>=k->LL_BoneCount())return false;
    const gfy_live::ImpactResult* latest=nullptr;
    for(const auto& impact:impacts)if(impact.removed&&impact.rejection.empty()&&
        (impact.hit_bone==head||impact.hit_bone==neck||impact.removed_m3_by_slot[3]>0.)&&
        (!latest||impact.sequence>latest->sequence))latest=&impact;
    if(!latest||!latest->gore.head_remnants_enabled)return false;
    painter_cranial_support::Decision chosen;
    u16 measured=BI_NONE;
    for(const auto& joint:joints)if(joint.bone==head||joint.bone==neck){
        const auto decision=cranial.valid?painter_cranial_support::evaluateCranial(joint.original_m3,joint.removed_m3,
            joint.original_bone_m3,joint.removed_bone_m3,cranial.original_m3,cranial.removed_m3,
            cranial.original_bone_m3,cranial.removed_bone_m3,cranial.original_brain_m3,cranial.removed_brain_m3,
            latest->gore.head_tether_strength_mult):painter_cranial_support::evaluate(joint.original_m3,joint.removed_m3,
            joint.original_bone_m3,joint.removed_bone_m3,latest->gore.head_tether_strength_mult);
        if(decision.state==painter_cranial_support::State::hanging&&
           (measured==BI_NONE||decision.stiffness_scale<chosen.stiffness_scale)){chosen=decision;measured=joint.bone;}
    }
    if(measured==BI_NONE)return false;
    // Resolve the original articulation by actual shell membership. A fitted
    // head fragment without that joint already moves as its own physical owner.
    auto& map=states();auto it=map.find(object.ID());
    if(it!=map.end()&&it->second.owner!=&object){map.erase(it);it=map.end();}
    if(it==map.end()){
        // Bounded bookkeeping; do not silently stop a scene after the first
        // 256 damaged heads. Only retired identities are pruned.
        if(map.size()>=256)for(auto p=map.begin();p!=map.end();){
            if(Level().Objects.net_Find(p->first)!=p->second.owner)p=map.erase(p);else ++p;}
        if(map.size()>=256)return false;
        it=map.emplace(object.ID(),JointState{}).first;
    }
    auto& state=it->second;
    if(revision<=state.pending_revision)return false;
    state.owner=&object;state.visual=k;state.pending_revision=revision;
    state.support=chosen;state.measured=measured;
    return poll(object);
}
}
