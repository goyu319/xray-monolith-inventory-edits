////////////////////////////////////////////////////////////////////////////
//	Module 		: stalker_animation_callbacks.cpp
//	Created 	: 25.02.2003
//  Modified 	: 19.11.2004
//	Author		: Dmitriy Iassenev
//	Description : Stalker animation manager : bone callbacks
////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "stalker_animation_manager.h"
#include "../Include/xrRender/Kinematics.h"
#include "ai/stalker/ai_stalker.h"
#include "sight_manager.h"
#include "stalker_movement_manager_smart_cover.h"
#include "game_object_space.h"
#include "effectorshot.h"

typedef CStalkerAnimationManager::callback_params callback_params;

// Procedural lean (script: set_painter_lean -1..1 / set_painter_lean_level -3..3): a roll about the forward axis, applied
// with the look/aim rotation and the bone origin kept, distributed over the spine (hip), the shoulder bone and the head
// (counter-roll) so the trunk bends in a curve on top of whatever animation plays. The pack has no lean clip; this is it.
static void painter_apply_lean(CBoneInstance* const bone, callback_params const* const parameter)
{
	if (fis_zero(parameter->m_lean_share, EPS))
		return;
	float const lean = parameter->m_object->painter_lean() * parameter->m_lean_share;
	if (fis_zero(lean, EPS))
		return;
	// Same space and operator as the look rotation above (setXYZ(pitch, yaw, roll) + mulA_43), so the roll channel is the
	// lean; a world-space axis here would turn into a pitch as soon as the NPC faces away from world +Z.
	Fmatrix lean_transform;
	lean_transform.setXYZ(0.f, 0.f, lean * CAI_Stalker::painter_lean_max_rad());
	bone->mTransform.mulA_43(lean_transform);
}

// Spine compensation share at this bone: rotate from the legs clip's actual bip01_spine to the spine pose the current
// torso clip was authored on (CStalkerAnimationManager::painter_update_torso_spine), bone origin kept.
static void painter_apply_spine_comp(CBoneInstance* const bone, callback_params const* const parameter)
{
	if (fis_zero(parameter->m_spine_comp, EPS))
		return;
	CStalkerAnimationManager const& animation = parameter->m_object->animation();
	Fquaternion authored;
	if (!animation.painter_torso_spine(authored))
		return;
	u16 const spine = animation.painter_spine_bone();
	IKinematics* const kinematics = smart_cast<IKinematics*>(parameter->m_object->Visual());
	if (spine == BI_NONE || !kinematics)
		return;
	Fmatrix actual = kinematics->LL_GetTransform(spine);
	actual.c.set(0.f, 0.f, 0.f);
	Fmatrix actual_inverse;
	actual_inverse.invert(actual);
	Fmatrix target;
	target.rotation(authored);
	Fmatrix full;
	full.mul_43(target, actual_inverse);
	Fquaternion q;
	q.set(full);
	Fquaternion identity;
	identity.set(Fidentity);
	Fquaternion share;
	share.slerp(identity, q, parameter->m_spine_comp);
	Fmatrix rotation;
	rotation.rotation(share);
	if (!_valid(rotation))
		return;
	Fvector const position = bone->mTransform.c;
	bone->mTransform.mulA_43(rotation);
	bone->mTransform.c = position;
}


static void _BCL callback_rotation(CBoneInstance* bone)
{
	if (!_valid( bone->mTransform )) return;
	R_ASSERT(_valid( bone->mTransform ));
	callback_params* parameter = static_cast<callback_params*>(bone->callback_param());
	VERIFY(parameter);
	VERIFY(parameter->m_rotation);
	VERIFY(parameter->m_object);

	if (!parameter || !_valid(*parameter->m_rotation))
	{
		return;
	}

	CAI_Stalker const* object = parameter->m_object;
	painter_apply_spine_comp(bone, parameter);
	if (!object->sight().enabled())
		return;

	Fvector position = bone->mTransform.c;
	R_ASSERT(_valid( *parameter->m_rotation ));
	bone->mTransform.mulA_43(*parameter->m_rotation);
	painter_apply_lean(bone, parameter);
	CWeaponShotEffector& effector = object->weapon_shot_effector();
	if (!effector.IsActive())
	{
		bone->mTransform.c = position;
		R_ASSERT(_valid( bone->mTransform ));
		return;
	}

	Fvector angles;
	effector.GetDeltaAngle(angles);
	angles.x = angle_normalize_signed(angles.x);
	angles.y = angle_normalize_signed(angles.y);
	angles.z = angle_normalize_signed(angles.z);

	if (object->movement().current_params().cover())
		angles.mul(0.f);
	else
		angles.mul(.1f);

	Fmatrix effector_transform;
	effector_transform.setXYZ(angles);
	R_ASSERT(_valid( effector_transform ));
	bone->mTransform.mulA_43(effector_transform);
	bone->mTransform.c = position;
	R_ASSERT(_valid( bone->mTransform ));
}

static void _BCL callback_rotation_blend(CBoneInstance* const bone)
{
	R_ASSERT(_valid( bone->mTransform ));

	callback_params* parameter = static_cast<callback_params*>(bone->callback_param());
	VERIFY(parameter);
	VERIFY(parameter->m_rotation);
	VERIFY(parameter->m_object);
	VERIFY(parameter->m_blend);
	//	VERIFY2							( *parameter->m_blend, make_string( "%d %s[%s]", Device.dwTimeGlobal, parameter->m_object->cName().c_str(), parameter->m_object->g_Alive() ? "+" : "-") );

	float multiplier = 1.f;
	if (*parameter->m_blend)
	{
		CBlend const& blend = **parameter->m_blend;
		multiplier = blend.timeCurrent / blend.timeTotal;
	}

	VERIFY(multiplier >= 0.f);
	VERIFY(multiplier <= 1.f);
	multiplier = parameter->m_forward ? multiplier : (1.f - multiplier);

#if 0
	Fmatrix rotation				= *parameter->m_rotation;
	Fvector							angles;
	rotation.getXYZ					(angles);
	angles.mul						(multiplier);
	rotation.setXYZ					(angles);
#else // #if 0
	Fquaternion left;
	left.set(Fidentity);

	Fquaternion right;
	right.set(*parameter->m_rotation);

	Fquaternion result;
	result.slerp(left, right, multiplier);

	Fmatrix rotation;
	rotation.rotation(result);
#endif // #if 0

	painter_apply_spine_comp(bone, parameter);
	Fvector position = bone->mTransform.c;
	R_ASSERT(_valid( rotation ));
	bone->mTransform.mulA_43(rotation);
	painter_apply_lean(bone, parameter);
	bone->mTransform.c = position;
	R_ASSERT(_valid( bone->mTransform ));
}

// callback_rotation / callback_rotation_blend only read per-stalker state (sight
// rotations, shot effector deltas, cover flag and the stalker's own blend) and write
// the bone they are called for. All writers of that state run on the main thread,
// which is blocked at the render-preparation barrier while workers evaluate bones, so
// these two callbacks may be evaluated by the skeleton job workers. Registration is
// idempotent and always happens before either callback is installed on a bone.
static void register_worker_safe_bone_callbacks()
{
	bone_callback_mark_worker_safe(&callback_rotation);
	bone_callback_mark_worker_safe(&callback_rotation_blend);
}

void CStalkerAnimationManager::assign_bone_callbacks()
{
	register_worker_safe_bone_callbacks();

	IKinematics* kinematics = smart_cast<IKinematics*>(m_visual);
	VERIFY(kinematics);

#ifdef DEBUG
	m_head_params.invalidate		();
	m_shoulder_params.invalidate	();
	m_spine_params.invalidate		();
#endif // #ifdef DEBUG

	LPCSTR section = *object().cNameSect();

	m_head_params.m_rotation = &object().sight().current_head_rotation();
	m_head_params.m_object = &object();
	m_head_params.m_blend = 0;
	m_head_params.m_forward = true;
	m_head_params.m_spine_comp = 0.f;
	m_head_params.m_lean_share = CAI_Stalker::painter_lean_share_head();

	int head_bone = kinematics->LL_BoneID(pSettings->r_string(section, "bone_head"));
	kinematics->LL_GetBoneInstance(u16(head_bone)).set_callback(bctCustom, &callback_rotation, &m_head_params);

	m_shoulder_params.m_rotation = &object().sight().current_shoulder_rotation();
	m_shoulder_params.m_object = &object();
	m_shoulder_params.m_blend = 0;
	m_shoulder_params.m_forward = true;
	m_shoulder_params.m_spine_comp = 0.5f;
	m_shoulder_params.m_lean_share = CAI_Stalker::painter_lean_share_shoulder();

	int shoulder_bone = kinematics->LL_BoneID(pSettings->r_string(section, "bone_shoulder"));
	kinematics->LL_GetBoneInstance(u16(shoulder_bone)).set_callback(bctCustom, &callback_rotation, &m_shoulder_params);

	m_spine_params.m_rotation = &object().sight().current_spine_rotation();
	m_spine_params.m_object = &object();
	m_spine_params.m_blend = 0;
	m_spine_params.m_forward = true;
	m_spine_params.m_spine_comp = 0.5f;
	m_spine_params.m_lean_share = CAI_Stalker::painter_lean_share_spine();

	int spine_bone = kinematics->LL_BoneID(pSettings->r_string(section, "bone_spin"));
	kinematics->LL_GetBoneInstance(u16(spine_bone)).set_callback(bctCustom, &callback_rotation, &m_spine_params);

	//	remove_bone_callbacks	();
}

void CStalkerAnimationManager::assign_bone_blend_callbacks(bool const& forward_direction)
{
	register_worker_safe_bone_callbacks();

	IKinematics* kinematics = smart_cast<IKinematics*>(m_visual);
	VERIFY(kinematics);

#ifdef DEBUG
	m_head_params.invalidate		();
	m_shoulder_params.invalidate	();
	m_spine_params.invalidate		();
#endif // #ifdef DEBUG

	LPCSTR section = *object().cNameSect();

	m_head_params.m_rotation = &object().sight().current_head_rotation();
	m_head_params.m_object = &object();
	m_head_params.m_blend = &global().blend();
	m_head_params.m_forward = forward_direction;
	m_head_params.m_spine_comp = 0.f;
	m_head_params.m_lean_share = CAI_Stalker::painter_lean_share_head();

	int head_bone = kinematics->LL_BoneID(pSettings->r_string(section, "bone_head"));
	kinematics->LL_GetBoneInstance(u16(head_bone)).set_callback(bctCustom, &callback_rotation_blend, &m_head_params);

	m_shoulder_params.m_rotation = &object().sight().current_shoulder_rotation();
	m_shoulder_params.m_object = &object();
	m_shoulder_params.m_blend = &global().blend();
	m_shoulder_params.m_forward = forward_direction;
	m_shoulder_params.m_spine_comp = 0.5f;
	m_shoulder_params.m_lean_share = CAI_Stalker::painter_lean_share_shoulder();

	int shoulder_bone = kinematics->LL_BoneID(pSettings->r_string(section, "bone_shoulder"));
	kinematics->LL_GetBoneInstance(u16(shoulder_bone)).set_callback(bctCustom, &callback_rotation_blend,
	                                                                &m_shoulder_params);

	m_spine_params.m_rotation = &object().sight().current_spine_rotation();
	m_spine_params.m_object = &object();
	m_spine_params.m_blend = &global().blend();
	m_spine_params.m_forward = forward_direction;
	m_spine_params.m_spine_comp = 0.5f;
	m_spine_params.m_lean_share = CAI_Stalker::painter_lean_share_spine();

	int spine_bone = kinematics->LL_BoneID(pSettings->r_string(section, "bone_spin"));
	kinematics->LL_GetBoneInstance(u16(spine_bone)).set_callback(bctCustom, &callback_rotation_blend, &m_spine_params);

	//	remove_bone_callbacks	();
}

void CStalkerAnimationManager::remove_bone_callbacks()
{
	IKinematics* kinematics = smart_cast<IKinematics*>(m_visual);
	VERIFY(kinematics);

#ifdef DEBUG
	m_head_params.invalidate		();
	m_shoulder_params.invalidate	();
	m_spine_params.invalidate		();
#endif // #ifdef DEBUG

	LPCSTR section = *object().cNameSect();

	int head_bone = kinematics->LL_BoneID(pSettings->r_string(section, "bone_head"));
	kinematics->LL_GetBoneInstance(u16(head_bone)).set_callback(bctCustom, 0, 0);

	int shoulder_bone = kinematics->LL_BoneID(pSettings->r_string(section, "bone_shoulder"));
	kinematics->LL_GetBoneInstance(u16(shoulder_bone)).set_callback(bctCustom, 0, 0);

	int spin_bone = kinematics->LL_BoneID(pSettings->r_string(section, "bone_spin"));
	kinematics->LL_GetBoneInstance(u16(spin_bone)).set_callback(bctCustom, 0, 0);
}

bool CStalkerAnimationManager::forward_blend_callbacks() const
{
	if (!m_head_params.m_blend)
		return (false);

	return (m_head_params.m_forward);
}

bool CStalkerAnimationManager::backward_blend_callbacks() const
{
	if (!m_head_params.m_blend)
		return (false);

	return (!m_head_params.m_forward);
}

void CStalkerAnimationManager::clear_unsafe_callbacks()
{
	if (!m_head_params.m_blend)
		return;

	assign_bone_callbacks();
}
