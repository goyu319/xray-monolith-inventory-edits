////////////////////////////////////////////////////////////////////////////
//	Module 		: stalker_animation_manager.cpp
//	Created 	: 25.02.2003
//  Modified 	: 19.11.2004
//	Author		: Dmitriy Iassenev
//	Description : Stalker animation manager
////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "stalker_animation_manager.h"
#include "ai/stalker/ai_stalker.h"
#include "stalker_animation_data_storage.h"
#include "stalker_animation_data.h"
#include "stalker_movement_manager_smart_cover.h"

#include "CharacterPhysicsSupport.h"
#include "weapon.h"
#include "../xrEngine/SkeletonMotions.h"
#include "../Include/xrRender/animation_blend.h"

// TODO:
// stalker animation manager consists of 5 independent managers,
// they should be represented with the different classes:
//    * head
//    * torso
//    * legs
//    * globals
//    * script

CStalkerAnimationManager::CStalkerAnimationManager(CAI_Stalker* object) :
	m_object(object),
	m_global(object),
	m_head(object),
	m_torso(object),
	m_legs(object),
	m_script(object),
	m_start_new_script_animation(false),
	m_painter_torso_mirrored(false),
	m_painter_grip_active(false),
	m_painter_grip_blend(0.f),
	m_painter_grip_from(0.f),
	m_painter_torso_spine_valid(false),
	m_painter_spine_chain_count(0)
{
}

void CStalkerAnimationManager::reinit()
{
	m_direction_start = 0;
	m_current_direction = eMovementDirectionForward;
	m_target_direction = eMovementDirectionForward;

	m_change_direction_time = 0;
	m_looking_back = 0;

	m_no_move_actual = false;
	m_painter_torso_mirrored = false;
	m_painter_grip_active = false;
	m_painter_grip_blend = 0.f;
	m_painter_grip_from = 0.f;
	m_painter_mirror_cache.clear();

	m_script_animations.clear();

	m_global.reset();
	m_head.reset();
	m_torso.reset();
	m_legs.reset();
	m_script.reset();

	m_legs.step_dependence(true);
	m_global.step_dependence(true);
	m_script.step_dependence(true);

	m_global.global_animation(true);
	m_script.global_animation(true);

	m_call_script_callback = false;

	m_previous_speed = 0.f;
	m_target_speed = 0.f;
	m_last_non_zero_speed = m_target_speed;

	m_special_danger_move = false;
}

void CStalkerAnimationManager::reload()
{
	m_visual = object().Visual();
	m_painter_mirror_cache.clear(); // motion ids belong to the visual's banks
	m_painter_spine_chain_count = 0; // bone ids belong to the visual
	m_painter_torso_spine_valid = false;

	m_crouch_state_config = object().SpecificCharacter().crouch_type();
	VERIFY((m_crouch_state_config == 0) || (m_crouch_state_config == 1) || (m_crouch_state_config == -1));
	m_crouch_state = m_crouch_state_config;

	if (object().already_dead())
		return;

	m_skeleton_animated = smart_cast<IKinematicsAnimated*>(m_visual);
	VERIFY(m_skeleton_animated);

	m_data_storage = stalker_animation_data_storage().object(m_skeleton_animated);
	VERIFY(m_data_storage);

	if (!object().g_Alive())
		return;

#ifdef USE_HEAD_BONE_PART_FAKE
	VERIFY(!m_data_storage->m_head_animations.A.empty());
	u16 bone_part = m_skeleton_animated->LL_GetMotionDef(m_data_storage->m_head_animations.A.front())->bone_or_part;
	VERIFY(bone_part != BI_NONE);
	m_script_bone_part_mask = CStalkerAnimationPair::all_bone_parts ^ (1 << bone_part);
#endif

	assign_bone_callbacks();

#ifdef DEBUG
	global().set_dbg_info		(*object().cName(),"Global");
	head().set_dbg_info			(*object().cName(),"Head  ");
	torso().set_dbg_info		(*object().cName(),"Torso ");
	legs().set_dbg_info			(*object().cName(),"Legs  ");
	script().set_dbg_info		(*object().cName(),"Script");
#endif

	m_global.reset();
	m_head.reset();
	m_torso.reset();
	m_legs.reset();
	m_script.reset();
};

void CStalkerAnimationManager::play_fx(float power_factor, int fx_index)
{
	VERIFY(fx_index >= 0);
	VERIFY(fx_index < (int)m_data_storage->m_part_animations.A[object().movement().body_state()].m_global.A[0].A.size())
	;
#ifdef DEBUG
	if (psAI_Flags.is(aiAnimation)) {
		LPCSTR					name = m_skeleton_animated->LL_MotionDefName_dbg(m_data_storage->m_part_animations.A[object().movement().body_state()].m_global.A[0].A[fx_index]).first;
		Msg						("%6d [%s][%s][%s][%f]",Device.dwTimeGlobal,*object().cName(),"FX",name,power_factor);
	}
#endif
	m_skeleton_animated->PlayFX(
		m_data_storage->m_part_animations.A[object().movement().body_state()].m_global.A[0].A[fx_index], power_factor);
}

bool CStalkerAnimationManager::standing() const
{
	CAI_Stalker& obj = object();
	stalker_movement_manager_smart_cover& movement = obj.movement();

	if (movement.speed(obj.character_physics_support()->movement()) < EPS_L)
		return (true);

	if (eMovementTypeStand == movement.movement_type())
		return (true);

	return (false);
}

// Name of a motion on this model (reverse lookup through the public motion slots; the debug-only name accessor is not
// built in release). Once per torso change / per logged second, so the linear scan is fine.
LPCSTR painter_motion_name(IKinematicsAnimated* const animated, MotionID const& id)
{
	if (!animated || !id.valid() || id.slot >= animated->LL_MotionsSlotCount())
		return "-";
	shared_motions& motions = const_cast<shared_motions&>(animated->LL_MotionsSlot(id.slot));
	accel_map const* const map = motions.motion_map();
	if (!map)
		return "-";
	for (accel_map::const_iterator i = map->begin(), e = map->end(); i != e; ++i)
		if (i->second == id.idx)
			return *i->first;
	return "?";
}

// Left-shoulder stance: swap a torso clip for its mirrored "<name>_ls" twin when the bank has one. Only standing and only
// while the NPC asks for the left shoulder; otherwise the stock clip plays and the grip stays right.
MotionID CStalkerAnimationManager::painter_mirrored_torso(MotionID const& animation)
{
	m_painter_torso_mirrored = false;
	if (!animation.valid() || !m_skeleton_animated)
		return animation;
	if (!object().painter_shoulder_left())
		return animation;
	if (object().movement().body_state() != MonsterSpace::eBodyStateStand)
		return animation;
	// No hand-dependence gate: GAMMA declares every weapon hand_dependence = 1, rifles included. Only the rifle-type
	// torso clips have "_ls" twins in the bank, so the lookup below is the weapon-class gate.
	u32 const key = (u32(animation.slot) << 16) | u32(animation.idx);
	xr_map<u32, MotionID>::const_iterator const found = m_painter_mirror_cache.find(key);
	MotionID mirrored;
	if (found != m_painter_mirror_cache.end())
		mirrored = found->second;
	else
	{
		LPCSTR const name = painter_motion_name(m_skeleton_animated, animation);
		if (name && name[0] != '-' && name[0] != '?')
		{
			string256 mirrored_name;
			strconcat(sizeof(mirrored_name), mirrored_name, name, "_ls");
			mirrored = m_skeleton_animated->ID_Cycle_Safe(mirrored_name);
		}
		m_painter_mirror_cache[key] = mirrored;
	}
	if (!mirrored.valid())
		return animation;
	m_painter_torso_mirrored = true;
	return mirrored;
}

// Grip blend for the weapon placement: when the torso clip changes, start from the current grip value and move toward the
// new clip's side (1 = mirrored) by the new blend's weight, which is exactly how much of the new torso pose is visible.
void CStalkerAnimationManager::painter_update_grip(bool const torso_changed)
{
	if (torso_changed)
		m_painter_grip_from = m_painter_grip_blend;
	float weight = 1.f;
	if (CBlend const* const blend = torso().blend())
		if (blend->blendPower > EPS_L)
			weight = clampr(blend->blendAmount / blend->blendPower, 0.f, 1.f);
	float const target = m_painter_torso_mirrored ? 1.f : 0.f;
	m_painter_grip_blend = m_painter_grip_from + (target - m_painter_grip_from) * weight;
	m_painter_grip_active = true;
}

// Console "painter_spine_comp 0|1" (default 1): rotate the upper body onto the spine pose its torso clip was authored on.
int g_painter_spine_comp = 1;

// Key of one bone of a motion at a time (frames at SAMPLE_FPS, looping), as the skeleton's own key sampling does.
static void painter_motion_key(CMotion const& motion, float const time, Fquaternion& rotation, Fvector& translation)
{
	u32 const count = motion.get_count();
	float const frames = time * float(SAMPLE_FPS);
	u32 const frame = u32(iFloor(frames < 0.f ? 0.f : frames));
	float const delta = clampr(frames - float(frame), 0.f, 1.f);
	if (motion.test_flag(flRKeyAbsent) || count < 2)
	{
		CKeyQR const& k = motion._keysR[0];
		rotation.x = float(k.x) * KEY_QuantI; rotation.y = float(k.y) * KEY_QuantI;
		rotation.z = float(k.z) * KEY_QuantI; rotation.w = float(k.w) * KEY_QuantI;
	}
	else
	{
		CKeyQR const& k0 = motion._keysR[frame % count];
		CKeyQR const& k1 = motion._keysR[(frame + 1) % count];
		Fquaternion q0, q1;
		q0.x = float(k0.x) * KEY_QuantI; q0.y = float(k0.y) * KEY_QuantI; q0.z = float(k0.z) * KEY_QuantI; q0.w = float(k0.w) * KEY_QuantI;
		q1.x = float(k1.x) * KEY_QuantI; q1.y = float(k1.y) * KEY_QuantI; q1.z = float(k1.z) * KEY_QuantI; q1.w = float(k1.w) * KEY_QuantI;
		rotation.slerp(q0, q1, delta);
	}
	if (motion.test_flag(flTKeyPresent) && count)
	{
		Fvector t0, t1;
		if (motion.test_flag(flTKey16IsBit))
		{
			CKeyQT16 const& a = motion._keysT16[frame % count];
			CKeyQT16 const& b = motion._keysT16[(frame + 1) % count];
			t0.set(float(a.x1) * motion._sizeT.x + motion._initT.x, float(a.y1) * motion._sizeT.y + motion._initT.y, float(a.z1) * motion._sizeT.z + motion._initT.z);
			t1.set(float(b.x1) * motion._sizeT.x + motion._initT.x, float(b.y1) * motion._sizeT.y + motion._initT.y, float(b.z1) * motion._sizeT.z + motion._initT.z);
		}
		else
		{
			CKeyQT8 const& a = motion._keysT8[frame % count];
			CKeyQT8 const& b = motion._keysT8[(frame + 1) % count];
			t0.set(float(a.x1) * motion._sizeT.x + motion._initT.x, float(a.y1) * motion._sizeT.y + motion._initT.y, float(a.z1) * motion._sizeT.z + motion._initT.z);
			t1.set(float(b.x1) * motion._sizeT.x + motion._initT.x, float(b.y1) * motion._sizeT.y + motion._initT.y, float(b.z1) * motion._sizeT.z + motion._initT.z);
		}
		translation.lerp(t0, t1, delta);
	}
	else
		translation.set(motion._initT);
}

void CStalkerAnimationManager::painter_update_torso_spine()
{
	m_painter_torso_spine_valid = false;
	if (!g_painter_spine_comp || !m_skeleton_animated)
		return;
	CBlend const* const blend = torso().blend();
	MotionID const id = torso().animation();
	if (!blend || !id.valid())
		return;
	IKinematics* const kinematics = smart_cast<IKinematics*>(m_visual);
	if (!kinematics)
		return;
	if (!m_painter_spine_chain_count)
	{
		// chain root .. bip01_spine, root first
		u16 chain[8];
		u16 count = 0;
		u16 bone = kinematics->LL_BoneID("bip01_spine");
		while (bone != BI_NONE && count < 8)
		{
			chain[count++] = bone;
			u16 const parent = kinematics->LL_GetData(bone).GetParentID();
			if (parent == bone)
				break;
			bone = parent;
		}
		if (!count || bone != BI_NONE)
			return; // no spine bone, or a chain deeper than expected: no compensation for this visual
		for (u16 i = 0; i < count; ++i)
			m_painter_spine_chain[i] = chain[count - 1 - i];
		m_painter_spine_chain_count = count;
	}
	Fmatrix model;
	model.identity();
	for (u16 i = 0; i < m_painter_spine_chain_count; ++i)
	{
		CMotion* const motion = m_skeleton_animated->LL_GetMotion(id, m_painter_spine_chain[i]);
		if (!motion || !motion->get_count())
			return;
		Fquaternion rotation;
		Fvector translation;
		painter_motion_key(*motion, blend->timeCurrent, rotation, translation);
		Fmatrix local;
		local.mk_xform(rotation, translation);
		model.mulB_43(local);
	}
	m_painter_torso_spine.set(model);
	m_painter_torso_spine_valid = _valid(m_painter_torso_spine.x) && _valid(m_painter_torso_spine.w);
}

