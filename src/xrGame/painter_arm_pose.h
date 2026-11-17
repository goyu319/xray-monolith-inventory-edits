#pragma once
class CScriptGameObject;
class CGameObject;
class CPhysicsSkeletonObject;
class CSE_Abstract;
class CBoneInstance;
struct SPainterArmPoseData;

// Bounded, opt-in diagnostic. This adopts sampled animation, not an existing
// body's mass/inertia or solver state. Side 0=left, 1=right.
u32 painter_arm_pose_sample(CScriptGameObject* owner, u32 side);
bool painter_arm_pose_seal(CScriptGameObject* owner, u32 token);
bool painter_arm_pose_bind(CScriptGameObject* owner, u32 token, u32 fragment_id);
bool painter_arm_pose_cancel(CScriptGameObject* owner, u32 token);
u32 painter_arm_pose_status(CScriptGameObject* fragment);
void painter_arm_pose_owner_gone(CGameObject& owner);

// net_Spawn owns the resulting data until net_Destroy/destruction. No borrowed
// NPC bone, shell, or Lua wrapper survives in the fragment's callback data.
int painter_arm_pose_begin(CPhysicsSkeletonObject&, CSE_Abstract*, SPainterArmPoseData*&);
bool painter_arm_pose_apply(CPhysicsSkeletonObject&, SPainterArmPoseData*);
void painter_arm_pose_release(CPhysicsSkeletonObject&, SPainterArmPoseData*&);
u32 painter_arm_pose_result(const SPainterArmPoseData*);
void painter_arm_pose_child(SPainterArmPoseData*, CBoneInstance*);
