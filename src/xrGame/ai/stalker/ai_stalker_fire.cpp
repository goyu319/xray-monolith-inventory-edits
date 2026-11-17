////////////////////////////////////////////////////////////////////////////
//	Module 		: ai_stalker_fire.cpp
//	Created 	: 25.02.2003
//  Modified 	: 25.02.2003
//	Author		: Dmitriy Iassenev
//	Description : Fire and enemy parameters for monster "Stalker"
////////////////////////////////////////////////////////////////////////////

#include "pch_script.h"
#include "../../../Layers/xrRender/BloodNativeProfile.h"
#include "../../../Layers/xrRender/PainterHitMain.h" // triggerhitch
#include "ai_stalker.h"
#include "ai_stalker_impl.h"
#include "../../script_entity_action.h"
#include "../../inventory.h"
#include "../../ef_storage.h"
#include "../../stalker_decision_space.h"
#include "../../script_game_object.h"
#include "../../customzone.h"
#include "../../../Include/xrRender/KinematicsAnimated.h"
#include "../../agent_manager.h"
#include "../../stalker_animation_manager.h"
#include "../../stalker_planner.h"
#include "../../ef_pattern.h"
#include "../../memory_manager.h"
#include "../../hit_memory_manager.h"
#include "../../enemy_manager.h"
#include "../../item_manager.h"
#include "../../stalker_movement_manager_smart_cover.h"
#include "../../entitycondition.h"
#include "../../sound_player.h"
#include "../../cover_point.h"
#include "../../agent_member_manager.h"
#include "../../agent_location_manager.h"
#include "../../danger_cover_location.h"
#include "../../object_handler_planner.h"
#include "../../object_handler_space.h"
#include "../../visual_memory_manager.h"
#include "../../weapon.h"
#include "ai_stalker_space.h"
#include "../../effectorshot.h"
#include "../../BoneProtections.h"
#include "../../RadioactiveZone.h"
#include "../../restricted_object.h"
#include "../../ai_object_location.h"
#include "../../missile.h"
#include "../../../xrphysics/iphworld.h"
#include "../../stalker_animation_names.h"
#include "../../agent_corpse_manager.h"
#include "../../CharacterPhysicsSupport.h"
#include "../../stalker_planner.h"
#include "../../stalker_decision_space.h"
#include "../../script_game_object.h"
#include "../../inventory.h"

#include "../../trajectories.h"
#include "script_hit.h"
#include "../../xrServerEntities/script_engine.h"

using namespace StalkerSpace;

float g_ai_reload_threshold = 0.5f;

float g_ai_cover_danger_radius      = 3.f;
u32   g_ai_cover_danger_time         = 120000;

float g_ai_fire_range_extension        = 2.5f;
float g_ai_fire_max_height_diff          = 2.f;
float g_ai_fire_min_dist           = 2.5f;
u32   g_ai_fire_make_sense_interval = 10000;

static float const min_throw_distance = 10.f;

float g_dispersion_base = 1.0f;
float g_dispersion_factor = 1.0f;

namespace
{
const ::luabind::functor<float>* stalker_weapon_accuracy_callback()
{
	static cached_script_functor<float> callback_cache("_g.CAI_Stalker__GetWeaponAccuracy");
	return callback_cache.get(ai().script_engine());
}

const ::luabind::functor<CScriptGameObject*>* update_best_weapon_callback()
{
	static cached_script_functor<CScriptGameObject*> callback_cache("_g.update_best_weapon");
	return callback_cache.get(ai().script_engine());
}
}

float CAI_Stalker::GetWeaponAccuracy() const
{
	float base = PI / 180.f;
	base *= m_fRankDisperison;
	CWeapon* W = smart_cast<CWeapon*>(inventory().ActiveItem());

	if (!movement().path_completed())
	{
		if (movement().movement_type() == eMovementTypeWalk)
		{
			if (movement().body_state() == eBodyStateStand)
			{
				base *= (m_disp_walk_stand * g_dispersion_factor + g_dispersion_base);
			}
			else
			{
				base *= (m_disp_walk_crouch * g_dispersion_factor + g_dispersion_base);
			}
		}
		else if (movement().movement_type() == eMovementTypeRun)
		{
			if (movement().body_state() == eBodyStateStand)
			{
				base *= (m_disp_run_stand * g_dispersion_factor + g_dispersion_base);
			}
			else
			{
				base *= (m_disp_run_crouch * g_dispersion_factor + g_dispersion_base);
			}
		}
	}
	else
	{
		bool hasScope = W && W->IsScopeAttached();

		if (movement().body_state() == eBodyStateStand)
		{
			if (zoom_state() && hasScope)
			{
				base *= (m_disp_stand_stand_zoom * g_dispersion_factor + g_dispersion_base);
			}
			else
			{
				base *= (m_disp_stand_stand * g_dispersion_factor + g_dispersion_base);
			}
		}
		else if (movement().body_state() == eBodyStateCrouch)
		{
			if (zoom_state() && hasScope)
			{
				base *= (m_disp_stand_crouch_zoom * g_dispersion_factor + g_dispersion_base);
			}
			else
			{
				base *= (m_disp_stand_crouch * g_dispersion_factor + g_dispersion_base);
			}
		}
		else
			base *= (m_disp_run_stand * g_dispersion_factor + g_dispersion_base); // fallback to worst aim if state could not determined, this should never happen (tm)
	}

	if (const auto* func = stalker_weapon_accuracy_callback())
	{
		base = (*func)(lua_game_object(), W ? W->lua_game_object() : nullptr, base, movement().body_state(), movement().movement_type());
	}

	return base;
}

void CAI_Stalker::g_fireParams(const CHudItem* pHudItem, Fvector& P, Fvector& D)
{
	//.	VERIFY				(inventory().ActiveItem());
	if (!inventory().ActiveItem())
	{
#ifdef DEBUG
		Msg				("! CAI_Stalker::g_fireParams() : VERIFY(inventory().ActiveItem())");
#endif // DEBUG
		P = Position();
		D = Fvector().set(0.f, 0.f, 1.f);
		return;
	}

	CWeapon* weapon = smart_cast<CWeapon*>(inventory().ActiveItem());
	if (!weapon)
	{
		CMissile* missile = smart_cast<CMissile*>(inventory().ActiveItem());
		if (missile)
		{
			update_throw_params();
			P = m_throw_position;
			D = Fvector().set(m_throw_velocity).normalize();
			VERIFY(!fis_zero(D.square_magnitude()));
			return;
		}

		P = eye_matrix.c;
		D = eye_matrix.k;

		if (weapon_shot_effector().IsActive())
			D = weapon_shot_effector_direction(D);

		VERIFY(!fis_zero(D.square_magnitude()));
		return;
	}

	if (!g_Alive())
	{
		P = weapon->get_LastFP();
		D = weapon->get_LastFD();
		VERIFY(!fis_zero(D.square_magnitude()));
		return;
	}

	if (!animation().script_animations().empty() || animation().global_selector())
	{
		P = weapon->get_LastFP();

		if (sniper_fire_mode())
			D.setHP(-movement().m_head.target.yaw, -movement().m_head.target.pitch);
		else
			D = weapon->get_LastFD();
		VERIFY(!fis_zero(D.square_magnitude()));

		return;
	}

	switch (movement().body_state())
	{
	case eBodyStateStand:
		{
			// The measured lean eye is the firing origin while moving too.
			// Use actual smoothed lean, never a merely requested lean target.
			if (movement().movement_type() == eMovementTypeStand || !fis_zero(painter_lean(), EPS))
			{
				P = eye_matrix.c;
				D = eye_matrix.k;
				if (weapon_shot_effector().IsActive())
					D = weapon_shot_effector_direction(D);
				VERIFY(!fis_zero(D.square_magnitude()));
			}
			else
			{
				D.setHP(-movement().m_head.current.yaw, -movement().m_head.current.pitch);
				if (weapon_shot_effector().IsActive())
					D = weapon_shot_effector_direction(D);
				Center(P);
				P.mad(D, .5f);
				P.y += .50f;
				VERIFY(!fis_zero(D.square_magnitude()));
			}

			if (sniper_fire_mode())
				D.setHP(-movement().m_head.target.yaw, -movement().m_head.target.pitch);

			return;
		}
	case eBodyStateCrouch:
		{
			P = eye_matrix.c;
			D = eye_matrix.k;
			if (weapon_shot_effector().IsActive())
				D = weapon_shot_effector_direction(D);

			VERIFY(!fis_zero(D.square_magnitude()));

			if (sniper_fire_mode())
				D.setHP(-movement().m_head.target.yaw, -movement().m_head.target.pitch);

			return;
		}
	default: NODEFAULT;
	}

#ifdef DEBUG
	P					= weapon->get_LastFP();
	D					= weapon->get_LastFD();
	VERIFY				(!fis_zero(D.square_magnitude()));
#endif
}

void CAI_Stalker::g_WeaponBones(int& L, int& R1, int& R2)
{
	int r_hand, r_finger2, l_finger1;
	CObjectHandler::weapon_bones(r_hand, r_finger2, l_finger1);
	R1 = r_hand;
	R2 = r_finger2;
	if (!animation().script_animations().empty() && animation().script_animations().front().hand_usage())
		L = R2;
	else
		L = l_finger1;
}

void CAI_Stalker::Hit(SHit* pHDS)
{
	BloodNativeProfile::Scope bloodHit(BloodNativeProfile::Stage::StalkerHit);
	//хит может меняться в зависимости от ранга (новички получают больше хита, чем ветераны)
	SHit HDS = *pHDS;
	HDS.painter_armor_penetration=-1; // fresh recipient: do not inherit another NPC armor decision
	HDS.painter_alive_before=g_Alive()?1:0; // R62: this hit's own death transition, stamped before any before-hit Lua (GBO) runs
	HDS.add_wound = true;

	//AVO: get bone names from IDs
	//if (HDS.whoID == 0) // if shot by actor
	//{
	//    LPCSTR bone_name = smart_cast<IKinematics*>(this->Visual())->LL_BoneName_dbg(HDS.boneID);
	//    Msg("Bone [%d]->[%s]", HDS.boneID, bone_name);
	//}
	//-AVO

	float hit_power = HDS.power * m_fRankImmunity;

	if (Core.ParamsData.test(ECoreParams::dbgbullet))
		Msg("CAI_Stalker::Hit hit_type=%d | hit_power(%f)*m_fRankImmunity(%f) = %f", (u32)HDS.hit_type, HDS.power,
		    m_fRankImmunity, hit_power);

	if (m_boneHitProtection && HDS.hit_type == ALife::eHitTypeFireWound)
	{
		float BoneArmor = m_boneHitProtection->getBoneArmor(HDS.bone());
		float ap = HDS.armor_piercing;
		if (!fis_zero(BoneArmor, EPS))
		{
			if (ap > BoneArmor)
			{
				float d_hit_power = (ap - BoneArmor) / (ap * m_boneHitProtection->APScale);
				clamp(d_hit_power, m_boneHitProtection->m_fHitFracNpc, 1.0f);

				hit_power *= d_hit_power;
				VERIFY(hit_power>=0.0f);

				if (Core.ParamsData.test(ECoreParams::dbgbullet))
					Msg("CAI_Stalker::Hit AP(%f) > BoneArmor(%f) [HitFracNpc=%f] modified hit_power=%f", ap, BoneArmor,
					    m_boneHitProtection->m_fHitFracNpc, hit_power);
			}
			else
			{
				hit_power *= m_boneHitProtection->m_fHitFracNpc;
				//HDS.add_wound = false;

				if (Core.ParamsData.test(ECoreParams::dbgbullet))
					Msg("CAI_Stalker::Hit AP(%f) > BoneArmor(%f) [HitFracNpc=%f] modified hit_power=%f", ap, BoneArmor,
					    m_boneHitProtection->m_fHitFracNpc, hit_power);
			}
		}

		if (wounded()) //уже лежит => добивание
		{
			hit_power = 1000.f;
		}
	}

	HDS.power = hit_power;

	if (g_Alive())
	{

		{
			const CEntityAlive *entity_alive = smart_cast<const CEntityAlive*>(pHDS->initiator());
			if(entity_alive && entity_alive->g_Alive() && tfGetRelationType(entity_alive) == ALife::eRelationTypeEnemy)
			{
				movement().set_mental_state(eMentalStateDanger);
				if (!memory().visual().visible_now(entity_alive))
				{
					CNotYetVisibleObject		new_object;
					new_object.m_object			= smart_cast<const CGameObject*>(pHDS->initiator());
					new_object.m_value			= 1.0f;
					new_object.m_prev_time		= Device.dwTimeGlobal-1;
					new_object.m_update_time	= Device.dwTimeGlobal;
					memory().visual().add_not_yet_visible_object(new_object);
					memory().visual().add_visible_object(pHDS->initiator(), Device.fTimeDelta);
				}
			}
		}

		bool already_critically_wounded = critically_wounded();

		if (!already_critically_wounded)
		{
			const CCoverPoint* cover = agent_manager().member().member(this).cover();
			if (!invulnerable() && cover && HDS.initiator() &&
				(HDS.initiator()->ID() != ID()) && !fis_zero(HDS.damage()) && brain().affect_cover())
			{
				agent_manager().location().add(
					xr_new<CDangerCoverLocation>(cover, Device.dwTimeGlobal, g_ai_cover_danger_time,
					                             g_ai_cover_danger_radius));
			}
		}

		const CEntityAlive* entity_alive = smart_cast<const CEntityAlive*>(HDS.initiator());
		if (entity_alive && !wounded())
		{
			if (is_relation_enemy(entity_alive))
				sound().play(eStalkerSoundInjuring);
			//			else
			//				sound().play		(eStalkerSoundInjuringByFriend);
		}

		int weapon_type = -1;
		if (best_weapon())
			weapon_type = best_weapon()->object().ef_weapon_type();

		if (
			!wounded() &&
			!already_critically_wounded)
		{
			bool became_critically_wounded = update_critical_wounded(HDS.boneID, HDS.power);
			if (
				!became_critically_wounded &&
				animation().script_animations().empty() &&
				(HDS.bone() != BI_NONE)
			)
			{
				Fvector D;
				float yaw, pitch;
				D.getHP(yaw, pitch);

#pragma todo("Dima to Dima : forward-back bone impulse direction has been determined incorrectly!")
				float power_factor = m_power_fx_factor * HDS.damage() / 100.f;
				clamp(power_factor, 0.f, 1.f);

				//IKinematicsAnimated		*tpKinematics = smart_cast<IKinematicsAnimated*>(Visual());
				IKinematics* tpKinematics = smart_cast<IKinematics*>(Visual());
#ifdef DEBUG
				tpKinematics->LL_GetBoneInstance	(HDS.bone());
				if (HDS.bone() >= tpKinematics->LL_BoneCount()) {
					Msg					("tpKinematics has no bone_id %d",HDS.bone());
					HDS._dump			();
				}
#endif
				//				int						fx_index = iFloor(tpKinematics->LL_GetBoneInstance(HDS.bone()).get_param(1) + (angle_difference(movement().m_body.current.yaw,-yaw) <= PI_DIV_2 ? 0 : 1));
				//				if (fx_index != -1)
				//					animation().play_fx	(power_factor,fx_index);
			}
			else
			{
				if (!already_critically_wounded && became_critically_wounded)
				{
					if (HDS.who)
					{
						CAI_Stalker* stalker = smart_cast<CAI_Stalker*>(HDS.who);
						if (stalker && stalker->g_Alive())
							stalker->on_critical_wound_initiator(this);
					}

					{
						::luabind::functor<void> funct;
						if (ai().script_engine().functor("_G.CAI_Stalker__OnCriticallyWounded", funct))
						{
							CScriptHit tLuaHit(&HDS);
							funct(this->lua_game_object(), &tLuaHit, HDS.boneID);
						}
					}
				}
			}
		}
	}

	if (g_Alive() && (!m_hit_callback || m_hit_callback(&HDS)))
	{
		CScriptHit tLuaHit(&HDS);

		::luabind::functor<bool>	funct;
		if (ai().script_engine().functor("_G.CAI_Stalker__BeforeHitCallback", funct))
		{
			BloodNativeProfile::Scope bloodCallback(BloodNativeProfile::Stage::ScriptHitCallback);
			painter_hit_main::PartScope hit_main_script(painter_hit_main::Script);
			if (!funct(this->lua_game_object(), &tLuaHit, HDS.boneID))
				return;
		}

		HDS.ApplyScriptHit(&tLuaHit);

		float const damage_factor = invulnerable() ? 0.f : 100.f;
		memory().hit().add(damage_factor * HDS.damage(), HDS.direction(), HDS.who, HDS.boneID);
	}

	//conditions().health()			= 1.f;

	inherited::Hit(&HDS);
}

void CAI_Stalker::HitSignal(float amount, Fvector& vLocalDir, CObject* who, s16 element)
{
}

void CAI_Stalker::OnItemTake(CInventoryItem* inventory_item)
{
	CObjectHandler::OnItemTake(inventory_item);
	m_item_actuality = false;
	m_sell_info_actuality = false;
}

void CAI_Stalker::OnItemDrop(CInventoryItem* inventory_item, bool just_before_destroy)
{
	CObjectHandler::OnItemDrop(inventory_item, just_before_destroy);

	m_item_actuality = false;
	m_sell_info_actuality = false;

	if (!g_Alive())
		return;

	if (!critically_wounded())
		return;

	//	VERIFY						(inventory().ActiveItem());

	if (inventory().ActiveItem() && (inventory().ActiveItem() != inventory_item))
		return;

	brain().CStalkerPlanner::m_storage.set_property(StalkerDecisionSpace::eWorldPropertyCriticallyWounded, false);
}

void CAI_Stalker::update_best_item_info()
{
	if (m_bTrading || !m_can_select_weapon)
		return;

	if (m_item_actuality && (m_best_item_info_cached_update_id == m_best_item_info_update_id))
		return;

	update_best_item_info_impl();
	m_best_item_info_cached_update_id = m_best_item_info_update_id;
}

void CAI_Stalker::update_best_item_info_impl()
{
	if (const auto* funct = update_best_weapon_callback())
	{
		CGameObject* cur_itm = smart_cast<CGameObject*>(m_best_item_to_kill);
		CScriptGameObject* GO = (*funct)(this->lua_game_object(), cur_itm ? cur_itm->lua_game_object() : NULL);
		CInventoryItem* bw = GO ? smart_cast<CInventoryItem*>(&GO->object()) : NULL;
		if (bw)
		{
			m_item_actuality = true;
			m_best_item_to_kill = bw;
			m_best_item_value = 0.f;
			m_best_ammo = bw;
			m_best_found_item_to_kill = 0;
			m_best_found_ammo = 0;
			return;
		}
	}

	ai().ef_storage().alife_evaluation(false);
	/* Alundaio: This is what causes stalkers to switch weapons during combat; It's stupid
	if	(
			m_item_actuality &&
			m_best_item_to_kill &&
			m_best_item_to_kill->can_kill()
		) {
		
		if (!memory().enemy().selected()) 
			return;

		ai().ef_storage().non_alife().member()	= this;
		ai().ef_storage().non_alife().enemy()	= memory().enemy().selected() ? memory().enemy().selected() : this;
		ai().ef_storage().non_alife().member_item()	= &m_best_item_to_kill->object();
		float									value;
		value									= ai().ef_storage().m_pfWeaponEffectiveness->ffGetValue();
		if (fsimilar(value,m_best_item_value))
			return;
	}
	*/

	// initialize parameters
	m_item_actuality = true;
	ai().ef_storage().non_alife().member() = this;
	ai().ef_storage().non_alife().enemy() = memory().enemy().selected() ? memory().enemy().selected() : this;
	m_best_item_to_kill = 0;
	m_best_ammo = 0;
	m_best_found_item_to_kill = 0;
	m_best_found_ammo = 0;
	m_best_item_value = 0.f;

	// try to find the best item which can kill
	{
		TIItemContainer::iterator I = inventory().m_all.begin();
		TIItemContainer::iterator E = inventory().m_all.end();
		for (; I != E; ++I)
		{
			if ((*I)->can_kill())
			{
				ai().ef_storage().non_alife().member_item() = &(*I)->object();
				float value;
				if (memory().enemy().selected())
					value = ai().ef_storage().m_pfWeaponEffectiveness->ffGetValue();
				else
					value = (float)(*I)->Cost();

				if (!fsimilar(value, m_best_item_value) && (value < m_best_item_value))
					continue;

				if (!fsimilar(value, m_best_item_value) && (value > m_best_item_value))
				{
					m_best_item_value = value;
					m_best_item_to_kill = *I;
					continue;
				}

				VERIFY(fsimilar(value,m_best_item_value));
				if (m_best_item_to_kill && ((*I)->Cost() <= m_best_item_to_kill->Cost()))
					continue;

				m_best_item_value = value;
				m_best_item_to_kill = *I;
			}
		}
	}

	// check if we found
	if (m_best_item_to_kill)
	{
		m_best_ammo = m_best_item_to_kill;
		return;
	}

	// so we do not have such an item
	// check if we remember we saw item which can kill
	// or items which can make my item killing
	{
		xr_vector<const CGameObject*>::const_iterator I = memory().item().objects().begin();
		xr_vector<const CGameObject*>::const_iterator E = memory().item().objects().end();
		for (; I != E; ++I)
		{
			const CInventoryItem* inventory_item = smart_cast<const CInventoryItem*>(*I);
			if (!inventory_item || !memory().item().useful(&inventory_item->object()))
				continue;
			CInventoryItem* item = inventory_item->can_kill(&inventory());
			if (item)
			{
				ai().ef_storage().non_alife().member_item() = &inventory_item->object();
				float value = ai().ef_storage().m_pfWeaponEffectiveness->ffGetValue();
				if (value > m_best_item_value)
				{
					m_best_item_value = value;
					m_best_found_item_to_kill = inventory_item;
					m_best_found_ammo = 0;
					m_best_ammo = item;
				}
			}
			else
			{
				item = inventory_item->can_make_killing(&inventory());
				if (!item)
					continue;

				ai().ef_storage().non_alife().member_item() = &item->object();
				float value = ai().ef_storage().m_pfWeaponEffectiveness->ffGetValue();
				if (value > m_best_item_value)
				{
					m_best_item_value = value;
					m_best_item_to_kill = item;
					m_best_found_item_to_kill = 0;
					m_best_found_ammo = inventory_item;
				}
			}
		}
	}

	// check if we found such an item
	if (m_best_found_item_to_kill || m_best_found_ammo)
		return;

	// check if we remember we saw item to kill
	// and item which can make this item killing
	xr_vector<const CGameObject*>::const_iterator I = memory().item().objects().begin();
	xr_vector<const CGameObject*>::const_iterator E = memory().item().objects().end();
	for (; I != E; ++I)
	{
		const CInventoryItem* inventory_item = smart_cast<const CInventoryItem*>(*I);
		if (!inventory_item || !memory().item().useful(&inventory_item->object()))
			continue;
		const CInventoryItem* item = inventory_item->can_kill(memory().item().objects());
		if (item)
		{
			ai().ef_storage().non_alife().member_item() = &inventory_item->object();
			float value = ai().ef_storage().m_pfWeaponEffectiveness->ffGetValue();
			if (value > m_best_item_value)
			{
				m_best_item_value = value;
				m_best_found_item_to_kill = inventory_item;
				m_best_found_ammo = item;
			}
		}
	}
}

bool CAI_Stalker::item_to_kill()
{
	update_best_item_info();
	return (!!m_best_item_to_kill);
}

bool CAI_Stalker::item_can_kill()
{
	update_best_item_info();
	return (!!m_best_ammo);
}

bool CAI_Stalker::remember_item_to_kill()
{
	update_best_item_info();
	return (!!m_best_found_item_to_kill);
}

bool CAI_Stalker::remember_ammo()
{
	update_best_item_info();
	return (!!m_best_found_ammo);
}

bool CAI_Stalker::ready_to_kill()
{
	return (
		m_best_item_to_kill &&
		inventory().ActiveItem() &&
		(inventory().ActiveItem()->object().ID() == m_best_item_to_kill->object().ID()) &&
		m_best_item_to_kill->ready_to_kill()
	);
}

bool CAI_Stalker::ready_to_detour()
{
	if (!ready_to_kill())
		return (false);

	CWeapon* weapon = smart_cast<CWeapon*>(m_best_item_to_kill);
	if (!weapon)
		return (false);

	return ((float)weapon->GetAmmoElapsed() > (float)weapon->GetAmmoMagSize() * g_ai_reload_threshold);
}

class ray_query_param
{
public:
	CAI_Stalker* m_holder;
	float m_power;
	float m_power_threshold;
	bool m_can_kill_enemy;
	bool m_can_kill_member;
	float m_pick_distance;

public:
	IC ray_query_param(const CAI_Stalker* holder, float power_threshold, float distance)
	{
		m_holder = const_cast<CAI_Stalker*>(holder);
		m_power_threshold = power_threshold;
		m_power = 1.f;
		m_can_kill_enemy = false;
		m_can_kill_member = false;
		m_pick_distance = distance;
	}
};

IC BOOL ray_query_callback(collide::rq_result& result, LPVOID params)
{
	ray_query_param* param = (ray_query_param*)params;
	float power = param->m_holder->feel_vision_mtl_transp(result.O, result.element);
	param->m_power *= power;

	//	if (power >= .05f) {
	//		param->m_pick_distance			= result.range;
	//		return							(true);
	//	}

	if (!result.O)
	{
		if (param->m_power > param->m_power_threshold)
			return (true);

		param->m_pick_distance = result.range;
		return (false);
	}

	CEntityAlive* entity_alive = smart_cast<CEntityAlive*>(result.O);
	if (!entity_alive)
	{
		if (param->m_power > param->m_power_threshold)
			return (true);

		param->m_pick_distance = result.range;
		return (false);
	}

	if (param->m_holder->is_relation_enemy(entity_alive))
		param->m_can_kill_enemy = true;
	else
		param->m_can_kill_member = true;

	param->m_pick_distance = result.range;
	return (false);
}

void CAI_Stalker::can_kill_entity(const Fvector& position, const Fvector& direction, float distance,
                                  collide::rq_results& rq_storage)
{
	VERIFY(!fis_zero(direction.square_magnitude()));

	collide::ray_defs ray_defs(position, direction, distance, CDB::OPT_CULL, collide::rqtBoth);
	VERIFY(!fis_zero(ray_defs.dir.square_magnitude()));

	ray_query_param params(this, memory().visual().transparency_threshold(), distance);

	Level().ObjectSpace.RayQuery(rq_storage, ray_defs, ray_query_callback, &params,NULL, this);
	m_can_kill_enemy = m_can_kill_enemy || params.m_can_kill_enemy;
	m_can_kill_member = m_can_kill_member || params.m_can_kill_member;
	m_pick_distance = _max(m_pick_distance, params.m_pick_distance);
}

void CAI_Stalker::can_kill_entity_from(const Fvector& position, Fvector direction, float distance)
{
	m_pick_distance = 0.f;
	rq_storage.r_clear();
	can_kill_entity(position, direction, distance, rq_storage);
	if (m_can_kill_member && m_can_kill_enemy)
		return;

	float yaw, pitch, safety_fire_angle = PI_DIV_8 * .125f;
	direction.getHP(yaw, pitch);


	direction.setHP(yaw, pitch - safety_fire_angle);
	can_kill_entity(position, direction, distance, rq_storage);
	if (m_can_kill_member && m_can_kill_enemy)
		return;

	direction.setHP(yaw, pitch + safety_fire_angle);
	can_kill_entity(position, direction, distance, rq_storage);
	if (m_can_kill_member)
		return;

	bool can_kill_enemy = m_can_kill_enemy;

	direction.setHP(yaw - safety_fire_angle, pitch);
	can_kill_entity(position, direction, distance, rq_storage);
	m_can_kill_enemy = can_kill_enemy;
	if (m_can_kill_member)
		return;

	direction.setHP(yaw + safety_fire_angle, pitch);
	can_kill_entity(position, direction, distance, rq_storage);
	m_can_kill_enemy = can_kill_enemy;
}

IC float CAI_Stalker::start_pick_distance() const
{
	const float result = 50.f;
	const CEntityAlive* enemy = memory().enemy().selected();
	if (!enemy)
		return (result);

	return (_max(result, enemy->Position().distance_to(Position()) + 1.f));
}

float CAI_Stalker::pick_distance()
{
	if (!inventory().ActiveItem())
		return (start_pick_distance());

	update_can_kill_info();
	return (m_pick_distance);
}

void CAI_Stalker::update_can_kill_info()
{
	if (m_pick_frame_id == Device.dwFrame)
		return;

	if (!inventory().ActiveItem())
		return;

	PROF_EVENT("update_can_kill_info");

	m_pick_frame_id = Device.dwFrame;
	m_can_kill_member = false;
	m_can_kill_enemy = false;

	Fvector position, direction;
	g_fireParams(0, position, direction);
	can_kill_entity_from(position, direction, start_pick_distance());
}

bool CAI_Stalker::undetected_anomaly()
{
	return (inside_anomaly() || brain().CStalkerPlanner::m_storage.property(StalkerDecisionSpace::eWorldPropertyAnomaly)
	);
}

bool CAI_Stalker::inside_anomaly()
{
	xr_vector<CObject*>::const_iterator I = feel_touch.begin();
	xr_vector<CObject*>::const_iterator E = feel_touch.end();
	for (; I != E; ++I)
	{
		CCustomZone* zone = smart_cast<CCustomZone*>(*I);
		if (zone && (zone->restrictor_type() != RestrictionSpace::eRestrictorTypeNone))
		{
			if (smart_cast<CRadioactiveZone*>(zone))
				continue;

			return (true);
		}
	}
	return (false);
}

bool CAI_Stalker::zoom_state() const
{
	if (!inventory().ActiveItem())
		return (false);

	if ((movement().movement_type() != eMovementTypeStand) && (movement().body_state() != eBodyStateCrouch) && !
		movement().path_completed())
		return (false);

	switch (CObjectHandler::planner().current_action_state_id())
	{
	case ObjectHandlerSpace::eWorldOperatorAim1:
	case ObjectHandlerSpace::eWorldOperatorAim2:
	case ObjectHandlerSpace::eWorldOperatorAimingReady1:
	case ObjectHandlerSpace::eWorldOperatorAimingReady2:
	case ObjectHandlerSpace::eWorldOperatorQueueWait1:
	case ObjectHandlerSpace::eWorldOperatorQueueWait2:
	case ObjectHandlerSpace::eWorldOperatorFire1:
		// we need this 2 operators to prevent fov/range switching
	case ObjectHandlerSpace::eWorldOperatorReload1:
	case ObjectHandlerSpace::eWorldOperatorReload2:
	case ObjectHandlerSpace::eWorldOperatorForceReload1:
	case ObjectHandlerSpace::eWorldOperatorForceReload2:
		return (true);
	}

	return (false);
}

void CAI_Stalker::update_range_fov(float& new_range, float& new_fov, float start_range, float start_fov)
{
	float range = start_range, fov = start_fov;

	if (zoom_state())
		inventory().ActiveItem()->modify_holder_params(range, fov);

	VERIFY2(start_fov < 180.f, make_string("[%s] %f", cName().c_str(), start_fov));
	VERIFY2(fov < 180.f,
	        make_string("fix addon multiplier for weapon or scope %s (fov=%f)", inventory().ActiveItem()->object().cName
		        ().c_str(), fov));

	return (inherited::update_range_fov(new_range, new_fov, range, fov));
}

bool CAI_Stalker::fire_make_sense()
{
	// if we do not have an enemy
	const CEntityAlive* enemy = memory().enemy().selected();
	if (!enemy)
		return (false);

	const Fvector& object_position = Position();
	const Fvector& enemy_position = enemy->Position();
	const float current_pick_distance = pick_distance();

	if ((current_pick_distance + g_ai_fire_range_extension) < object_position.distance_to(enemy_position))
		return (false);

	if (_abs(object_position.y - enemy_position.y) > g_ai_fire_max_height_diff)
		return (false);

	if (current_pick_distance < g_ai_fire_min_dist)
		return (false);

	if (memory().visual().visible_right_now(enemy))
		return (true);

	u32 last_time_seen = memory().visual().visible_object_time_last_seen(enemy);
	if (last_time_seen == u32(-1))
		return (false);

	if (Device.dwTimeGlobal > last_time_seen + g_ai_fire_make_sense_interval)
		return (false);

	// if we do not have a weapon
	if (!best_weapon())
		return (false);

	// if we do not have automatic weapon
	switch (best_weapon()->object().ef_weapon_type())
	{
	case 6:
	case 7:
	case 8:
	case 10:
		break;
	default:
		return (false);
	}

	return (true);
}

// shot effector stuff
void CAI_Stalker::on_weapon_shot_start(CWeapon* weapon)
{
	weapon_shot_effector().SetRndSeed(m_weapon_shot_random_seed);
	weapon_shot_effector().Shot(weapon);

	{
		::luabind::functor<void> funct;
		if (ai().script_engine().functor("_G.CAI_Stalker__OnWeaponShotStart", funct))
			funct(lua_game_object(), weapon ? weapon->lua_game_object() : nullptr);
	}
}

void CAI_Stalker::on_weapon_shot_update()
{
}

void CAI_Stalker::on_weapon_shot_stop(CWeapon*)
{
	{
		::luabind::functor<void> funct;
		if (ai().script_engine().functor("_G.CAI_Stalker__OnWeaponShotStop", funct))
			funct(lua_game_object());
	}
}

void CAI_Stalker::on_weapon_shot_remove(CWeapon* weapon)
{
}

void CAI_Stalker::on_weapon_hide(CWeapon* weapon)
{
}

void CAI_Stalker::notify_on_wounded_or_killed(CObject* object)
{
	CAI_Stalker* stalker = smart_cast<CAI_Stalker*>(object);
	if (!stalker)
		return;

	if (!stalker->g_Alive())
		return;

	stalker->on_enemy_wounded_or_killed(this);

	typedef CAgentCorpseManager::MEMBER_CORPSES MEMBER_CORPSES;

	const MEMBER_CORPSES& corpses = agent_manager().corpse().corpses();
	if (std::find(corpses.begin(), corpses.end(), this) != corpses.end())
		return;

	agent_manager().corpse().register_corpse(this);
}

void CAI_Stalker::notify_on_wounded_or_killed()
{
	ALife::_OBJECT_ID last_hit_object_id = memory().hit().last_hit_object_id();
	if (last_hit_object_id == ALife::_OBJECT_ID(-1))
		return;

	CObject* object = Level().Objects.net_Find(last_hit_object_id);
	if (!object)
		return;

	notify_on_wounded_or_killed(object);
}

void CAI_Stalker::wounded(bool value)
{
	if (m_wounded == value)
		return;

	if (!g_Alive())
		return;

	if (value)
		notify_on_wounded_or_killed();

	m_wounded = value;

	if (!m_wounded && g_Alive())
		character_physics_support()->CreateCharacterSafe();

	if (!m_wounded)
		return;

	character_physics_support()->movement()->DestroyCharacter();

	if (!agent_manager().member().registered_in_combat(this))
		return;

	agent_manager().member().unregister_in_combat(this);
}

bool CAI_Stalker::wounded(const CRestrictedObject* object) const
{
	if (!wounded())
		return (false);

	VERIFY(object);
	if (!object->accessible(Position()))
		return (false);

	if (!object->accessible(ai_location().level_vertex_id()))
		return (false);

	return (true);
}

// demonized: toggle movement collision for stalker NPCs
void CAI_Stalker::set_enable_movement_collision(bool value) {
	character_physics_support()->movement()->DestroyCharacter();
	if (value)
		character_physics_support()->CreateCharacterSafe();
}

bool CAI_Stalker::use_default_throw_force()
{
	return (false);
}

bool CAI_Stalker::use_throw_randomness()
{
	return (false);
}

float CAI_Stalker::missile_throw_force()
{
	update_throw_params();
	float force = m_throw_velocity.magnitude();
	if (m_painter_throw)
		return force; // a script-aimed arc only lands where it was aimed at exactly this speed

	::luabind::functor<float> funct;
	if (ai().script_engine().functor("_G.CAI_Stalker__GetMissileThrowForce", funct))
	{
		float result = funct(lua_game_object(), force);
		if (_valid(result) && result > 0.f)
			force = result;
		else
			ai().script_engine().script_log(ScriptStorage::eLuaMessageTypeError,
			                                "CAI_Stalker [%s]: _g.CAI_Stalker__GetMissileThrowForce returned invalid value %f (must be > 0), ignoring",
			                                *cName(), result);
	}

	return force;
}

void CAI_Stalker::compute_throw_miss(u32 const vertex_id)
{
	float const throw_miss_radius = ::Random.randF(2.f, 5.f);
	u32 const try_count = 6;

	CLevelGraph const& level_graph = ai().level_graph();
	for (u32 i = 0; i < try_count; ++i)
	{
		Fvector const direction = Fvector().random_dir();
		Fvector const check_position = Fvector().mad(m_throw_target_position, direction, throw_miss_radius);
		u32 const check_vertex_id = level_graph.check_position_in_direction(
			vertex_id, m_throw_target_position, check_position);
		if (!level_graph.valid_vertex_id(check_vertex_id))
			continue;

		m_throw_target_position = check_position;
		return;
	}
}

void CAI_Stalker::throw_target_impl(const Fvector& position, CObject* throw_ignore_object)
{
	float distance_to_sqr = position.distance_to_sqr(m_throw_target_position);
	m_throw_actual = m_throw_actual && (distance_to_sqr < _sqr(.1f));
	m_throw_target_position = position;
	m_throw_ignore_object = throw_ignore_object;
}

void CAI_Stalker::throw_target(const Fvector& position, CObject* throw_ignore_object)
{
	if (m_painter_throw)
	{
		m_painter_throw = m_painter_throw_via_set = false; // the native AI aims again: stock arc and spread
		m_throw_actual = false;
	}
	throw_target_impl(position, throw_ignore_object);
}

void CAI_Stalker::throw_target(const Fvector& position, u32 const vertex_id, CObject* throw_ignore_object)
{
	if (m_painter_throw)
	{
		m_painter_throw = m_painter_throw_via_set = false;
		m_throw_actual = false;
	}
	throw_target_impl(position, throw_ignore_object);
	compute_throw_miss(vertex_id);
}

void CAI_Stalker::painter_throw(const Fvector& target, const Fvector* via)
{
	throw_target_impl(target, nullptr);
	m_painter_throw_requested = target;
	m_painter_throw_reason = "no_target";
	m_painter_throw = true;
	m_painter_throw_via_set = via != nullptr;
	if (via)
		m_painter_throw_via = *via;
	m_throw_actual = false;
}

// Door throw candidates: arcs from start to target, scanned by flight time, that cross the doorway plane within 45 cm of
// the door point (the centre of the opening) at a hand-throw speed (16 m/s or less), gentlest first, as (time, velocity).
// lateral = how far the door point lies off the vertical plane through start and target.
static void painter_door_throw_candidates(const Fvector& start, const Fvector& door, const Fvector& target, float gravity,
                                          xr_vector<std::pair<float, Fvector>>& out, float& lateral)
{
	out.clear();
	lateral = flt_max;
	Fvector const delta = Fvector().sub(target, start);
	Fvector h;
	h.set(delta.x, 0.f, delta.z);
	float const x_target = h.magnitude();
	if (x_target < .5f || gravity <= 0.f)
		return;
	h.div(x_target);
	Fvector const to_door = Fvector().set(door.x - start.x, 0.f, door.z - start.z);
	float const x_door = to_door.dotproduct(h);
	lateral = _abs(to_door.x * h.z - to_door.z * h.x);
	if (x_door < .2f || x_door > x_target - .2f) // the doorway must lie between thrower and target
		return;
	float const t_min = _sqrt(2.f * delta.magnitude() / gravity);
	for (int i = 1; i <= 60; ++i)
	{
		float const time = t_min * (.15f + .04f * float(i));
		float const vh = x_target / time;
		float const vy = delta.y / time + .5f * gravity * time;
		Fvector const velocity = Fvector().set(h.x * vh, vy, h.z * vh);
		float const t_door = x_door / vh;
		float const y_door = start.y + vy * t_door - .5f * gravity * t_door * t_door;
		if (velocity.magnitude() <= 16.f && _abs(y_door - door.y) <= .45f)
			out.push_back(std::make_pair(time, velocity));
	}
	std::sort(out.begin(), out.end(), [](std::pair<float, Fvector> const& a, std::pair<float, Fvector> const& b) {
		return a.second.square_magnitude() < b.second.square_magnitude();
	});
}

// delete this function!!!!
bool CAI_Stalker::throw_check_error(
	float low,
	float high,
	const Fvector& position,
	const Fvector& velocity,
	const Fvector& gravity
)
{
	return true;
}

void CAI_Stalker::check_throw_trajectory(const float& throw_time)
{
	m_throw_enabled = false;

	xr_vector<trajectory_pick>* trajectory_picks = NULL;
	xr_vector<Fvector>* collide_tris = NULL;
#ifdef DEBUG
	trajectory_picks				=	& m_throw_picks;
	collide_tris					=	& m_throw_collide_tris;
#endif // #ifdef DEBUG

	Fvector box_size = {0.f, 0.f, 0.f};
	//Fvector box_size				=	{ 0.1f, 0.1f , 0.1f };

	if (!trajectory_intersects_geometry(throw_time,
	                                    m_throw_position,
	                                    m_throw_target_position,
	                                    m_throw_velocity,
	                                    m_throw_collide_position,
	                                    this,
	                                    m_throw_ignore_object,
	                                    rq_storage,
	                                    trajectory_picks,
	                                    collide_tris,
	                                    box_size))
	{
		m_throw_collide_position = Fvector().set(flt_max,flt_max,flt_max);
		m_throw_enabled = true;
	}
}

void CAI_Stalker::update_throw_params()
{
	if (m_throw_actual)
	{
		if (m_computed_object_position.similar(Position()))
		{
			if (m_computed_object_direction.similar(Direction()))
			{
				VERIFY(_valid(m_throw_velocity));
				return;
			}
		}
	}

	m_throw_actual = true;
	m_computed_object_position = Position();
	m_computed_object_direction = Direction();

#if 0
	m_throw_position		= eye_matrix.c;
#else
	m_throw_position = Position();

	CMissile* const pMissile = fast_dynamic_cast<CMissile*>(inventory().ActiveItem());
	if (pMissile)
	{
		static const LPCSTR third_person_offset_id = "third_person_throw_point_offset";

		if (!pSettings->line_exist(pMissile->cNameSect(), third_person_offset_id))
		{
			m_throw_position.y += 2.f;

			if (pMissile)
			{
				Fvector offset;
				XFORM().transform_dir(offset, pMissile->throw_point_offset());
				m_throw_position.add(offset);
			}
		}
		else
			m_throw_position.add(pSettings->r_fvector3(pMissile->cNameSect(), third_person_offset_id));
	}
#endif

	static float const distances[] = {
		30.f, 40.f, 50.f, 60.f
	};
	VERIFY(g_SingleGameDifficulty < sizeof(distances)/sizeof(distances[0]));
	float const max_distance = distances[g_SingleGameDifficulty];

	// computing velocity with minimum magnitude
	m_throw_velocity.sub(m_throw_target_position, m_throw_position);

	if (m_throw_velocity.magnitude() > max_distance)
	{
		m_throw_enabled = false;
		if (m_painter_throw)
			m_painter_throw_reason = "out_of_range";

#ifdef DEBUG
		m_throw_picks.clear	();
#endif // DEBUG
		return;
	}

	if (m_painter_throw)
	{
		float const gravity = physics_world()->Gravity();
		float time = 0.f;
		if (m_painter_throw_via_set)
		{
			// released at hand height: the native 2 m release point is already at lintel height
			m_throw_position.y = Position().y + 1.6f;
			m_throw_enabled = false;
			if (!pMissile)
			{
				m_painter_throw_reason = "no_grenade";
				return;
			}
			// the target is re-aimed onto the line from the release point through the door point, as deep past the door as
			// asked (the caller cannot see the release point, so any lateral offset clips the frame); when that arc hits
			// something, shallower targets are tried
			Fvector const& door = m_painter_throw_via;
			Fvector line = Fvector().set(door.x - m_throw_position.x, 0.f, door.z - m_throw_position.z);
			float const to_door = line.magnitude();
			if (to_door < .3f)
			{
				m_painter_throw_reason = "door_behind";
				return;
			}
			line.div(to_door);
			Fvector const asked = m_painter_throw_requested;
			float depth = Fvector().set(asked.x - door.x, 0.f, asked.z - door.z).dotproduct(line);
			if (depth < 1.f)
				depth = _max(1.f, Fvector().set(asked.x - door.x, 0.f, asked.z - door.z).magnitude());
			float const depths[] = {depth, depth * .75f, depth * .5f, 1.f};
			Fvector first_target{}, first_velocity{};
			bool have_first = false;
			bool any_arc = false;
			xr_vector<std::pair<float, Fvector>> candidates;
			for (int d = 0; d < 4; ++d)
			{
				if (d && depths[d] >= depths[d - 1] - .1f)
					continue;
				Fvector target = Fvector().mad(door, line, depths[d]);
				target.y = asked.y;
				m_throw_target_position = target;
				float lateral = flt_max;
				painter_door_throw_candidates(m_throw_position, door, target, gravity, candidates, lateral);
				if (candidates.empty())
					continue;
				any_arc = true;
				if (!have_first)
				{
					have_first = true;
					first_target = target;
					first_velocity = candidates.front().second;
				}
				for (u32 i = 0; i < candidates.size() && i < 8; ++i)
				{
					m_throw_velocity = candidates[i].second;
					check_throw_trajectory(candidates[i].first);
					if (m_throw_enabled)
					{
						m_painter_throw_reason = "ok";
						return; // exact speed: no random spread
					}
				}
			}
			m_painter_throw_reason = any_arc ? "blocked" : "no_arc";
			if (have_first)
			{
				m_throw_target_position = first_target; // a valid aim even while blocked
				m_throw_velocity = first_velocity;
			}
			return;
		}
		else
		{
			time = ThrowMinVelTime(m_throw_velocity, gravity);
			TransferenceToThrowVel(m_throw_velocity, time, gravity);
		}
		check_throw_trajectory(time);
		m_painter_throw_reason = !pMissile ? "no_grenade" : (m_throw_enabled ? "ok" : "blocked");
		return; // exact speed: no random spread
	}

	float time = ThrowMinVelTime(m_throw_velocity, physics_world()->Gravity());
	TransferenceToThrowVel(m_throw_velocity, time, physics_world()->Gravity());

	check_throw_trajectory(time);

	m_throw_velocity.mul(::Random.randF(.75f, 1.25f));
}

void CAI_Stalker::on_throw_completed()
{
	agent_manager().member().on_throw_completed();
	m_last_throw_time = Device.dwTimeGlobal;
}

bool CAI_Stalker::critically_wounded()
{
	if (critical_wound_type() == critical_wound_type_dummy)
		return (false);

	if (!brain().CStalkerPlanner::m_storage.property(StalkerDecisionSpace::eWorldPropertyCriticallyWounded))
	{
		critical_wounded_state_stop();
		return (false);
	}

	return (true);
}

bool CAI_Stalker::critical_wound_external_conditions_suitable()
{
	if (movement().body_state() != eBodyStateStand)
		return (false);

	if (animation().non_script_need_update())
		return (false);

	CWeapon* active_weapon = smart_cast<CWeapon*>(inventory().ActiveItem());
	if (!active_weapon)
		return (false);

	if (movement().in_smart_cover())
		return (false);

	switch (active_weapon->animation_slot())
	{
	case 1: // pistols
	case 2: // automatic weapon
	case 3: // rifles
		break;
	default:
		return (false);
	}

	if (!agent_manager().member().registered_in_combat(this))
		return (false);

	//	Msg								("%6d executing critical hit",Device.dwTimeGlobal);
	animation().global().make_inactual();
	return (true);
}

void CAI_Stalker::remove_critical_hit()
{
	brain().CStalkerPlanner::m_storage.set_property(
		StalkerDecisionSpace::eWorldPropertyCriticallyWounded,
		false
	);

	animation().global().remove_callback(
		CStalkerAnimationPair::CALLBACK_ID(
			this,
			&CAI_Stalker::remove_critical_hit
		)
	);
}

void CAI_Stalker::critical_wounded_state_start()
{
	brain().CStalkerPlanner::m_storage.set_property(
		StalkerDecisionSpace::eWorldPropertyCriticallyWounded,
		true
	);

	animation().global().add_callback(
		CStalkerAnimationPair::CALLBACK_ID(
			this,
			&CAI_Stalker::remove_critical_hit
		)
	);
}

bool CAI_Stalker::can_cry_enemy_is_wounded() const
{
	if (!brain().initialized())
		return (false);

	if (brain().current_action_id() != StalkerDecisionSpace::eWorldOperatorCombatPlanner)
		return (false);

	if (!agent_manager().member().group_behaviour())
		return (false);

	typedef CActionPlannerActionScript<CAI_Stalker> planner_type;
	planner_type* planner = smart_cast<planner_type*>(&brain().current_action());
	VERIFY(planner);

	switch (planner->current_action_id())
	{
	case StalkerDecisionSpace::eWorldOperatorGetReadyToKill:
	case StalkerDecisionSpace::eWorldOperatorGetReadyToDetour:
	case StalkerDecisionSpace::eWorldOperatorKillEnemy:
	case StalkerDecisionSpace::eWorldOperatorTakeCover:
	case StalkerDecisionSpace::eWorldOperatorLookOut:
	case StalkerDecisionSpace::eWorldOperatorHoldPosition:
	case StalkerDecisionSpace::eWorldOperatorDetourEnemy:
	case StalkerDecisionSpace::eWorldOperatorSearchEnemy:
	case StalkerDecisionSpace::eWorldOperatorKillEnemyIfNotVisible:
	case StalkerDecisionSpace::eWorldOperatorKillEnemyIfCriticallyWounded:
	case StalkerDecisionSpace::eWorldOperatorThrowGrenade:
	case StalkerDecisionSpace::eWorldOperatorGetDistance:
	case StalkerDecisionSpace::eWorldOperatorLowCover:
	case StalkerDecisionSpace::eWorldOperatorInSmartCover:
		return (true);
	case StalkerDecisionSpace::eWorldOperatorGetItemToKill:
	case StalkerDecisionSpace::eWorldOperatorRetreatFromEnemy:
	case StalkerDecisionSpace::eWorldOperatorPostCombatWait:
	case StalkerDecisionSpace::eWorldOperatorHideFromGrenade:
	case StalkerDecisionSpace::eWorldOperatorSuddenAttack:
	case StalkerDecisionSpace::eWorldOperatorKillEnemyIfPlayerOnThePath:
	case StalkerDecisionSpace::eWorldOperatorKillWoundedEnemy:
	case StalkerDecisionSpace::eWorldOperatorCriticallyWounded:
		return (false);
	default: NODEFAULT;
	}
#ifdef DEBUG
	return							(false);
#endif // DEBUG
}

void CAI_Stalker::on_critical_wound_initiator(const CAI_Stalker* critically_wounded)
{
	if (!g_Alive())
		return;

	if (!can_cry_enemy_is_wounded())
		return;

	sound().play(eStalkerSoundEnemyCriticallyWounded);
}

void CAI_Stalker::on_enemy_wounded_or_killed(const CAI_Stalker* wounded_or_killed)
{
	if (!g_Alive())
		return;

	if (!can_cry_enemy_is_wounded())
		return;

	sound().play(eStalkerSoundEnemyKilledOrWounded);

	{
		::luabind::functor<void> funct;
		if (ai().script_engine().functor("_G.CAI_Stalker__OnEnemyWoundedOrKilled", funct))
		{
			if (wounded_or_killed)
				funct(lua_game_object(), wounded_or_killed->lua_game_object());
			else
				ai().script_engine().script_log(ScriptStorage::eLuaMessageTypeError,
				                                "CAI_Stalker [%s]: OnEnemyWoundedOrKilled called with null victim, skipping Lua hook",
				                                *cName());
		}
	}
}

bool CAI_Stalker::can_kill_member()
{
	if (!animation().script_animations().empty())
		return (false);

	update_can_kill_info();
	return (m_can_kill_member);
}

bool CAI_Stalker::can_kill_enemy()
{
	VERIFY(inventory().ActiveItem());
	update_can_kill_info();
	return (m_can_kill_enemy);
}

bool CAI_Stalker::too_far_to_kill_enemy(const Fvector& position)
{
#if 1
	return (false);
#else
	VERIFY					(memory().enemy().selected());
	VERIFY					(best_weapon());

	int						weapon_type = best_weapon()->object().ef_weapon_type();
	float					distance = position.distance_to(Position());
	switch (weapon_type) {
		// pistols
		case 5 : return		(distance > 10.f);
		// shotguns
		case 9 : return		(distance > 5.f);
		// sniper rifles
		case 11 :
		case 12 :
			return			(distance > 70.f);
		default: return		(distance > 5.f);
	}
#endif
}

// Console "painter_fire_log 1": one [painter_fire] line per stalker per second while it has a selected enemy, with the
// native fire checks the AI layer reads through painter_native_fire_flags (log-only, no behaviour change).
int g_painter_fire_log = 0;

u32 CAI_Stalker::painter_native_fire_flags()
{
	if (!g_Alive() || !inventory().ActiveItem())
		return 0;
	u32 flags = smart_cast<CWeapon*>(inventory().ActiveItem()) ? 8 : 0;
	if (fire_make_sense())
		flags |= 1;
	if (can_kill_enemy())
		flags |= 2;
	if (can_kill_member())
		flags |= 4;
	return flags;
}

// The tactical target may be a different personally visible enemy from
// native memory's selected one. Keep the native muzzle cone/material/friend
// checks, but evaluate range and visibility for that exact target. This does
// not alter native enemy selection or its cached firing verdict.
u32 CAI_Stalker::painter_target_fire_flags(const CEntityAlive* target)
{
    if(!target||target->getDestroy()||!target->g_Alive()||!is_relation_enemy(target)||
        !memory().visual().visible_now(target)||!g_Alive()||!smart_cast<CWeapon*>(inventory().ActiveItem()))return 0;
    // Direct native-combat sight is not the legacy floor-gap heuristic.
    // Keep actual weapon range, material/cone obstruction and friendly checks.
    auto* weapon=smart_cast<CWeapon*>(inventory().ActiveItem());
    const float maximum=weapon->GetFireDistance();
    const float target_distance=Position().distance_to(target->Position());
    if(!_valid(maximum)||maximum<=0||!_valid(target_distance)||target_distance>maximum)return 8u;
    if(target==memory().enemy().selected()){
        // Preserve the stock per-frame cone/material/friend cache. Only the
        // direct-visible usefulness rule differs from fire_make_sense().
        const float pick=pick_distance();
        const bool useful=pick>=g_ai_fire_min_dist&&pick+g_ai_fire_range_extension>=target_distance;
        return 8u|(useful?1u:0u)|(can_kill_enemy()?2u:0u)|(can_kill_member()?4u:0u);
    }
    Fvector position,direction;g_fireParams(nullptr,position,direction);
    if(!_valid(position)||!_valid(direction)||direction.square_magnitude()<=EPS)return 0;
    direction.normalize();
    const float distance=_min(maximum,_max(50.f,position.distance_to(target->Position())+target->Radius()+1.f));
    float yaw,pitch;direction.getHP(yaw,pitch);
    const float cone=PI_DIV_8*.125f;
    bool enemy=false,friend_in_cone=false;float pick=0;
    collide::rq_results storage;
    for(int n=0;n<5;++n){
        Fvector ray=direction;
        if(n)ray.setHP(yaw+(n==3?-cone:n==4?cone:0.f),pitch+(n==1?-cone:n==2?cone:0.f));
        collide::ray_defs def(position,ray,distance,CDB::OPT_CULL,collide::rqtBoth);
        ray_query_param result(this,memory().visual().transparency_threshold(),distance);
        storage.r_clear();Level().ObjectSpace.RayQuery(storage,def,ray_query_callback,&result,nullptr,this);
        if(n<3)enemy=enemy||result.m_can_kill_enemy;
        friend_in_cone=friend_in_cone||result.m_can_kill_member;pick=_max(pick,result.m_pick_distance);
        if(friend_in_cone)break;
    }
    const bool useful=pick>=g_ai_fire_min_dist&&pick+g_ai_fire_range_extension>=target_distance;
    return 8u|(useful?1u:0u)|(enemy?2u:0u)|(friend_in_cone?4u:0u);
}

void CAI_Stalker::painter_fire_log()
{
	if (!g_Alive() || Device.dwTimeGlobal - m_painter_fire_logged_at < 1000)
		return;
	CEntityAlive const* const enemy = memory().enemy().selected();
	if (!enemy)
		return;
	m_painter_fire_logged_at = Device.dwTimeGlobal;
	u32 const flags = painter_native_fire_flags();
	Fvector fp, fd;
	fp.set(Position());
	fd.set(0.f, 0.f, 1.f);
	if (inventory().ActiveItem())
		g_fireParams(0, fp, fd);
	Msg("[painter_fire] t=%u id=%u name=%s enemy=%u dist=%.1f visible=%u flags=%u verdict=%u fp=%.2f,%.2f,%.2f fd=%.2f,%.2f,%.2f pick=%.1f",
		Device.dwTimeGlobal, ID(), cName().c_str(), enemy->ID(), Position().distance_to(enemy->Position()),
		unsigned(memory().visual().visible_right_now(enemy)), flags, unsigned((flags & 3) == 3 && !(flags & 4)),
		VPUSH(fp), VPUSH(fd), pick_distance());
}

