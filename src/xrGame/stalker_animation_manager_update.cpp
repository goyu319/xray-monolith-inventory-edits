////////////////////////////////////////////////////////////////////////////
//	Module 		: stalker_animation_manager_update.cpp
//	Created 	: 25.02.2003
//  Modified 	: 13.12.2006
//	Author		: Dmitriy Iassenev
//	Description : Stalker animation manager update cycle
////////////////////////////////////////////////////////////////////////////

#include "pch_script.h"
#include "painter_ai/EngineAttainedPose.h"         // after the precompiled header: anything above it is skipped
#include "stalker_animation_manager.h"
#include "ai/stalker/ai_stalker.h"
#include "game_object_space.h"
#include "script_callback_ex.h"
#include "profiler.h"
#include "stalker_movement_manager_smart_cover.h"
#include "Inventory.h"

void CStalkerAnimationManager::play_delayed_callbacks()
{
	if (m_call_script_callback)
	{
		m_call_script_callback = false;
		object().callback(GameObject::eScriptAnimation)();
		return;
	}

	if (m_call_global_callback)
	{
		m_call_global_callback = false;
		if (m_global_callback)
			m_global_callback();
		return;
	}
}

IC bool CStalkerAnimationManager::script_callback() const
{
	if (script_animations().empty())
		return (false);

	return (object().callback(GameObject::eScriptAnimation));
}

IC bool CStalkerAnimationManager::need_update() const
{
	if (script_callback())
		return (true);

	return (non_script_need_update());
}

IC void CStalkerAnimationManager::update_tracks()
{
	if (!need_update())
		return;

	m_skeleton_animated->UpdateTracks();
}

#ifdef USE_HEAD_BONE_PART_FAKE
IC void CStalkerAnimationManager::play_script_impl()
{
	clear_unsafe_callbacks();
	global().reset();
	torso().reset();
	legs().reset();

	const CStalkerAnimationScript& selected = assign_script_animation();
	script().animation(selected.animation());
	if (selected.use_movement_controller())
	{
		script().target_matrix(selected.transform(object()));
		if (m_start_new_script_animation)
		{
			m_start_new_script_animation = false;
			if (selected.has_transform() && object().animation_movement())
				object().destroy_anim_mov_ctrl();
		}
	}

	script().play(
		m_skeleton_animated,
		script_play_callback,
		selected.use_movement_controller(),
		selected.local_animation(),
		false,
		m_script_bone_part_mask
	);

	head().animation(assign_head_animation());
	head().play(m_skeleton_animated, head_play_callback, false, false);
}
#else // USE_HEAD_BONE_PART_FAKE
IC	void CStalkerAnimationManager::play_script_impl			()
{
    clear_unsafe_callbacks	();
    global().reset			();
    head().reset			();
    torso().reset			();
    legs().reset			();

    const CStalkerAnimationScript	&selected = assign_script_animation();
    script().animation		(selected.animation());
    script().play			(
        m_skeleton_animated,
        script_play_callback,
        selected.use_movement_controller(),
        selected.local_animation(),
        false,
        m_script_bone_part_mask
        );
}
#endif // USE_HEAD_BONE_PART_FAKE

bool CStalkerAnimationManager::play_script()
{
	if (script_animations().empty())
	{
		m_start_new_script_animation = false;
		script().reset();
		return (false);
	}

	play_script_impl();

	return (true);
}

#ifdef USE_HEAD_BONE_PART_FAKE
IC void CStalkerAnimationManager::play_global_impl(const MotionID& animation, bool const& animation_movement_controller)
{
	torso().reset();
	legs().reset();

	global().animation(animation);
	global().play(
		m_skeleton_animated,
		global_play_callback,
		animation_movement_controller,
		true,
		false,
		m_script_bone_part_mask,
		true
	);

	if (m_global_modifier)
		m_global_modifier(global().blend());

	head().animation(assign_head_animation());
	head().play(m_skeleton_animated, head_play_callback, false, false);
}
#else // USE_HEAD_BONE_PART_FAKE
IC	void CStalkerAnimationManager::play_global_impl			(const MotionID &animation, bool const &animation_movement_controller)
{
    head().reset			();
    torso().reset			();
    legs().reset			();

    global().animation		(animation);
    global().play			(m_skeleton_animated,global_play_callback,false,false,false);
}
#endif // USE_HEAD_BONE_PART_FAKE

bool CStalkerAnimationManager::play_global()
{
	bool animation_movement_controller = false;
	const MotionID& global_animation = assign_global_animation(animation_movement_controller);
	if (!global_animation)
	{
		clear_unsafe_callbacks();
		global().reset();
		return (false);
	}

	play_global_impl(global_animation, animation_movement_controller);

	return (true);
}

IC void CStalkerAnimationManager::play_head()
{
	head().animation(assign_head_animation());
	head().play(m_skeleton_animated, head_play_callback, false, false);
}

IC void CStalkerAnimationManager::play_torso()
{
	bool const unchanged = torso().animation(painter_mirrored_torso(assign_torso_animation()));
	torso().play(m_skeleton_animated, torso_play_callback, false, false);
	painter_update_grip(!unchanged);
	painter_update_torso_spine();
}

void CStalkerAnimationManager::play_legs()
{
	float speed = 0.f;
	bool first_time = !legs().animation();
	bool result = legs().animation(assign_legs_animation());

	if (!first_time && !result && legs().blend())
	{
		float amount = legs().blend()->blendAmount;
		m_previous_speed = (m_target_speed - m_previous_speed) * amount + m_previous_speed;
	}

	legs().play(m_skeleton_animated, legs_play_callback, false, false, !fis_zero(m_target_speed));

	// painter_speed_scale: the movement velocity is scaled in stalker_movement_manager_base::speed(); scale the leg
	// cycle (and, while moving, the torso cycle) by the same factor so the stride length stays the clip's own.
	// Written every frame (at scale 1 this equals what PlayCycle set), so a scale change in either direction takes
	// effect on the running cycle; the torso rate returns to the clip's own when the NPC stops moving.
	{
		const float scale = object().painter_speed_scale();
		if (CBlend* blend = legs().blend())
		{
			CMotionDef* def = m_skeleton_animated->LL_GetMotionDef(legs().animation());
			blend->speed = (def ? def->Speed() : 1.f) * scale;
		}
		if (CBlend* blend = torso().blend())
		{
			CMotionDef* def = m_skeleton_animated->LL_GetMotionDef(torso().animation());
			blend->speed = (def ? def->Speed() : 1.f) * (fis_zero(m_target_speed) ? 1.f : scale);
		}
	}

	if (result && legs().blend())
	{
		float amount = legs().blend()->blendAmount;
		speed = (m_target_speed - m_previous_speed) * amount + m_previous_speed;
	}

	if (fis_zero(speed))
		return;

	if (!legs().blend())
		return;

	object().movement().setup_speed_from_animation(speed);
}

void CStalkerAnimationManager::update_impl()
{
	m_painter_pose_valid = false;
	// painter grip: a frame without a torso clip (global/script animation, dead, vehicle) holds the weapon in the right hand
	if (!m_painter_grip_active)
		m_painter_grip_blend = 0.f;
	m_painter_grip_active = false;
	m_painter_torso_spine_valid = false; // set again by play_torso when a torso clip plays this frame

	if (!object().g_Alive())
		return;

	update_tracks();
	play_delayed_callbacks();
#ifdef HOLDERCUSTOM_NEW
	if (object().Holder())
		return;
#endif

	if (play_script())
		return;

	if (play_global())
		return;

	play_head();
	play_torso();
	play_legs();

	torso().synchronize(m_skeleton_animated, m_legs);
	// Record the body category of the clips selected and played this frame.
	m_painter_pose_body = u8(object().movement().body_state());
	m_painter_pose_frame = Device.dwFrame;
	m_painter_pose_valid = true;
}

void CStalkerAnimationManager::update()
{
	START_PROFILE("stalker/client_update/animations")
		try
		{
			update_impl();
		}
		catch (...)
		{
			Msg("! error in stalker [%s], profile [%s] with visual [%s]",
				object().cNameSect().c_str(),
				object().cast_inventory_owner()->CharacterInfo().GetSpecificCharacterId().c_str(),
				object().cNameVisual().c_str());

			for (auto& invItem : m_object->cast_inventory_owner()->m_inventory->m_all)
			{
				Msg("# Inventory item: [%s] - %s", invItem->m_section_id.c_str(), invItem->m_name.c_str());
			}

			/* avo: prevent game from crashing */
			global().reset();
			head().reset();
			torso().reset();
			legs().reset();
			return;
			//throw;
			/* avo: end */
		}
	STOP_PROFILE
}

// A target movement body state alone is not attained stance. Both current
// animation cycles must have reached their full contribution this frame.
bool CStalkerAnimationManager::painter_attained_pose(u8& body) const
{
    const CBlend* torso = m_torso.blend(); const CBlend* legs = m_legs.blend();
    if (!torso || !legs) return false;
    using painter::combat::engine::AppliedPoseChannel;
    const AppliedPoseChannel t{m_torso.actual(),m_torso.animation()==torso->motionID,
        torso->blend_state()==CBlend::eAccrue,torso->blendAmount,torso->blendPower};
    const AppliedPoseChannel l{m_legs.actual(),m_legs.animation()==legs->motionID,
        legs->blend_state()==CBlend::eAccrue,legs->blendAmount,legs->blendPower};
    if (!painter::combat::engine::attained_animation_pose(m_painter_pose_valid,
        m_painter_pose_frame,Device.dwFrame,t,l)) return false;
    body = m_painter_pose_body; return true;
}
