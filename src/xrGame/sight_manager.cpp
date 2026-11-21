////////////////////////////////////////////////////////////////////////////
//	Module 		: sight_manager.cpp
//	Created 	: 27.12.2003
//  Modified 	: 27.12.2003
//	Author		: Dmitriy Iassenev
//	Description : Sight manager
////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "sight_manager.h"
#include "ai/stalker/ai_stalker.h"
#include "stalker_movement_manager_smart_cover.h"
#include "profiler.h"
#include "aimers_weapon.h"
#include "aimers_bone.h"
#include "stalker_animation_manager.h"
#include "detail_path_manager.h"
#include "weapon.h"
#include "../Include/xrRender/Kinematics.h"
#include "../Include/xrRender/KinematicsAnimated.h"
#include "../xrEngine/SkeletonMotions.h"

using MonsterSpace::SBoneRotation;

// Console "painter_pose_log 1": one [painter_pose] line per stalker per second with rendered-skeleton sanity metrics
// (world space, from the posed bones): head height over the pelvis, each hand's height relative to the head, the hand
// line (right index -> left index = the engine's rifle axis) pitch, torso lean from vertical, and flags: 1 = a hand above
// the head, 2 = rifle line steeper than 45 deg while both hands hold it, 4 = standing but bent over 60 deg, 8 = head
// collapsed toward the pelvis. This is what catches an animation that the sight angles cannot see.
int g_painter_pose_log = 0;

LPCSTR painter_motion_name(IKinematicsAnimated* const animated, MotionID const& id); // stalker_animation_manager.cpp
// Console "painter_lean_test N" (-100..100): forces every stalker's lean target to N/100 so the lean can be exercised and
// measured (pose log lean= / head_dx=) without script support. 0 = scripts control the lean. Diagnostics only.
int g_painter_lean_test = 0;
extern int g_painter_fire_log; // ai_stalker_fire.cpp
float painter_support_hand_weight(u16 id); // IKLimbsController.cpp: left hand on the gun's grip
float painter_support_hand_error(u16 id);
float painter_support_hand_short(u16 id);
float painter_support_hand_elbow(u16 id);
float painter_support_hand_fade(u16 id);
int g_painter_shoulder_test = -1; // -1 off; 0/1/2 force every stalker's shoulder mode (diagnostics)

// Console "painter_look_log 1": one [painter_look] line per NPC per second with the selected sight action, the
// head/body yaw+pitch (degrees, positive pitch = up, as Fvector::setHP), the look point/object and the pitch the
// target would imply from the eye. Off by default; read-only diagnostics for the AI test ground.
int g_painter_look_log = 0;
static LPCSTR painter_sight_type_name(u32 type)
{
	static LPCSTR names[] = {"current_direction", "path_direction", "direction", "position", "object", "cover", "search",
	                         "look_over", "cover_look_over", "fire_object", "fire_position", "animation_direction"};
	return type < sizeof(names) / sizeof(names[0]) ? names[type] : "dummy";
}

//												head  shoulder  spine
static Fvector const s_danger_factors = {.0f, .50f, .50f};
static Fvector const s_free_factors = {.25f, .25f, .50f};
static float const s_factor_lerp_speed = 1.f;

//#define SIGHT_DEBUG

CSightManager::CSightManager(CAI_Stalker* object) :
	inherited(object),
	m_enabled(true),
	m_turning_in_place(false),
	m_painter_look_logged_at(0),
	m_painter_pose_logged_at(0),
	m_aiming_type(aiming_none)
{
}

void CSightManager::Load(LPCSTR section)
{
}

void CSightManager::reinit()
{
	inherited::reinit();
	m_enabled = true;
	m_turning_in_place = false;

	VERIFY(_valid(s_free_factors));
	m_current.m_head.m_factor = s_free_factors.x;
	m_current.m_shoulder.m_factor = s_free_factors.y;
	m_current.m_spine.m_factor = s_free_factors.z;
}

void CSightManager::reload(LPCSTR section)
{
	m_max_left_angle = deg2rad(READ_IF_EXISTS(pSettings, r_float, section, "max_left_torso_angle", 90.f));
	m_max_right_angle = deg2rad(READ_IF_EXISTS(pSettings, r_float, section, "max_right_torso_angle", 60.f));
}

void CSightManager::vfValidateAngleDependency(float x1, float& x2, float x3)
{
	float const _x2 = angle_normalize_signed(x2 - x1);
	float const _x3 = angle_normalize_signed(x3 - x1);
	if (_x2 * _x3 >= 0.f)
		return;

	if (_abs(_x2) + _abs(_x3) <= PI)
		return;

	x2 = x3;
}

#ifdef DEBUG
BOOL g_ai_dbg_sight = 0;
#endif // #ifdef DEBUG

float g_ai_aim_min_speed = 0.24f; //PI_DIV_8/2.f; //Alundaio
float g_ai_aim_min_angle = PI_DIV_8 / 2.f;
float g_ai_aim_max_angle = PI_DIV_4;
BOOL g_ai_aim_use_smooth_aim = 1;

static inline float select_speed(float const distance, float const speed, float const min_speed,
                                 float const min_distance, float const max_distance)
{
	VERIFY(max_distance > min_distance);

#ifdef DEBUG
	if ( !g_ai_aim_use_smooth_aim )
		return speed;
#endif// #ifdef DEBUG

	if (speed <= min_speed)
		return speed;

	if (distance < min_distance)
		return min_speed;

	if (distance >= max_distance)
		return speed;

	float const factor = (distance - min_distance) / (max_distance - min_distance);
	return min_speed + factor * (speed - min_speed);
}

void CSightManager::Exec_Look(float time_delta)
{
	START_PROFILE("Sight Manager")
		extern int g_goyu_painter_look;
		if (g_goyu_painter_look) {
		if (g_painter_lean_test) m_object->set_painter_lean(float(g_painter_lean_test) / 100.f);
		if (g_painter_shoulder_test >= 0) m_object->set_painter_shoulder(g_painter_shoulder_test);
		m_object->painter_lean_update(time_delta); // procedural lean target -> current, consumed by the spine bone callback
		if (g_painter_fire_log)
			m_object->painter_fire_log();
		}
		SBoneRotation& body = object().movement().m_body;
		SBoneRotation& head = object().movement().m_head;

		if (object().animation_movement_controlled())
			body.target = body.current;

		// normalizing torso angles
		body.current.yaw = angle_normalize_signed(body.current.yaw);
		body.current.pitch = angle_normalize_signed(body.current.pitch);
		body.target.yaw = angle_normalize_signed(body.target.yaw);
		body.target.pitch = angle_normalize_signed(body.target.pitch);

		// normalizing head angles
		head.current.yaw = angle_normalize_signed(head.current.yaw);
		head.current.pitch = angle_normalize_signed(head.current.pitch);
		head.target.yaw = angle_normalize_signed(head.target.yaw);
		head.target.pitch = angle_normalize_signed(head.target.pitch);

		float body_speed = body.speed;
		if (current_action().change_body_speed())
			body_speed = current_action().body_speed();

		float head_speed = head.speed;
		if (current_action().change_head_speed())
			head_speed = current_action().head_speed();

		extern float g_painter_look_head_rate_deg;
		extern float g_painter_look_body_rate_deg;
		extern float g_painter_look_dead_zone_deg;
		bool head_in_dead_zone = false;
		if (g_goyu_painter_look)
		{
		body_speed = _min(body_speed, deg2rad(g_painter_look_body_rate_deg));
		head_speed = _min(head_speed, deg2rad(g_painter_look_head_rate_deg));
		}
		if (g_goyu_painter_look)
		{
			const u32 sight_type = u32(current_action().sight_type());
			const bool on_enemy = sight_type == SightManager::eSightTypeObject || sight_type == SightManager::eSightTypeFireObject;
			const float dead = deg2rad(g_painter_look_dead_zone_deg);
			head_in_dead_zone = !on_enemy && dead > 0.f &&
				_abs(angle_difference(head.current.yaw, head.target.yaw)) < dead &&
				_abs(angle_difference(head.current.pitch, head.target.pitch)) < dead;
		}

#ifdef SIGHT_DEBUG
	if ( object().cName() == "level_prefix_stalker" ) {
		Msg				("[%6d][%s] BEFORE BODY [%f] -> [%f]",Device.dwTimeGlobal, object().cName().c_str(), object().movement().m_body.current.yaw,object().movement().m_body.target.yaw);
		Msg				("[%6d][%s] BEFORE HEAD [%f] -> [%f]",Device.dwTimeGlobal, object().cName().c_str(), object().movement().m_head.current.yaw,object().movement().m_head.target.yaw);
	}
#endif // #ifdef SIGHT_DEBUG

		//static CStatGraph* s_stats_graph	= 0;
		//if ( !s_stats_graph ) {
		//	s_stats_graph					= xr_new<CStatGraph>();
		//	s_stats_graph->SetRect			(0, 1024-68, 1280, 68, 0xff000000, 0xff000000);
		//	s_stats_graph->SetMinMax		(-PI, PI, 1000);
		//	s_stats_graph->SetStyle			(CStatGraph::stBarLine);
		//	s_stats_graph->AppendSubGraph	(CStatGraph::stCurve);
		//	s_stats_graph->AppendSubGraph	(CStatGraph::stCurve);
		//}

		//s_stats_graph->AppendItem			( angle_normalize_signed(head.current.yaw),   0xff00ff00, 0 );
		//s_stats_graph->AppendItem			( angle_normalize_signed(head.current.pitch), 0xffff0000, 1 );

#ifdef DEBUG
	if ( g_ai_dbg_sight )
		Msg							( "%6d [%s] before body[%f]->[%f], head[%f]->[%f]", Device.dwTimeGlobal, object().cName().c_str(), body.current.yaw, body.target.yaw, head.current.yaw, head.target.yaw );
#endif // #ifdef DEBUG
		vfValidateAngleDependency(body.current.yaw, body.target.yaw, head.current.yaw);
#ifdef DEBUG
	if ( g_ai_dbg_sight )
		Msg							( "%6d [%s] after  body[%f]->[%f], head[%f]->[%f]", Device.dwTimeGlobal, object().cName().c_str(), body.current.yaw, body.target.yaw, head.current.yaw, head.target.yaw );
#endif // #ifdef DEBUG

		m_object->angle_lerp_bounds(body.current.yaw, body.target.yaw,
		                            select_speed(angle_difference(body.current.yaw, body.target.yaw), body_speed,
		                                         m_object->aim_min_speed(), m_object->aim_min_angle(), m_object->aim_max_angle()),
		                            time_delta);
		m_object->angle_lerp_bounds(body.current.pitch, body.target.pitch,
		                            select_speed(angle_difference(body.current.pitch, body.target.pitch), body_speed,
		                                         m_object->aim_min_speed(), m_object->aim_min_angle(), m_object->aim_max_angle()),
		                            time_delta);

		if (!head_in_dead_zone)
		{
			m_object->angle_lerp_bounds(head.current.yaw, head.target.yaw,
			                            select_speed(angle_difference(head.current.yaw, head.target.yaw), head_speed,
			                                         m_object->aim_min_speed(), m_object->aim_min_angle(), m_object->aim_max_angle()),
			                            time_delta);
			m_object->angle_lerp_bounds(head.current.pitch, head.target.pitch,
			                            select_speed(angle_difference(head.current.pitch, head.target.pitch), head_speed,
			                                         m_object->aim_min_speed(), m_object->aim_min_angle(), m_object->aim_max_angle()),
			                            time_delta);
		}

#ifdef DEBUG
	if ( g_ai_dbg_sight )
		Msg							( "%6d [%s] after2 body[%f]->[%f], head[%f]->[%f]", Device.dwTimeGlobal, object().cName().c_str(), body.current.yaw, body.target.yaw, head.current.yaw, head.target.yaw );
#endif // #ifdef DEBUG

#ifdef SIGHT_DEBUG
	// normalizing torso angles
	body.current.yaw	= angle_normalize_signed	(body.current.yaw);
	body.current.pitch	= angle_normalize_signed	(body.current.pitch);

	// normalizing head angles
	head.current.yaw	= angle_normalize_signed	(head.current.yaw);
	head.current.pitch	= angle_normalize_signed	(head.current.pitch);

	if ( object().cName() == "level_prefix_stalker" ) {
		Msg				("[%6d][%s] AFTER  BODY [%f] -> [%f]",			Device.dwTimeGlobal, object().cName().c_str(),object().movement().m_body.current.yaw,object().movement().m_body.target.yaw);
		Msg				("[%6d][%s] AFTER  HEAD [%f][%f] -> [%f][%f]",	Device.dwTimeGlobal, object().cName().c_str(), object().movement().m_head.current.yaw,object().movement().m_head.current.pitch,object().movement().m_head.target.yaw,object().movement().m_head.target.pitch);
	}
#endif // #ifdef SIGHT_DEBUG

		if (enabled())
		{
			compute_aiming(time_delta, head_speed);
			current_action().on_frame();
		}

		if (g_painter_look_log && enabled() && Device.dwTimeGlobal - m_painter_look_logged_at >= 1000)
		{
			m_painter_look_logged_at = Device.dwTimeGlobal;
			const CSightAction& action = current_action();
			const u32 type = u32(action.sight_type());
			const Fvector eye = m_object->eye_matrix.c;
			// Internal pitch is positive DOWN; the public head_orientation() export uses setHP(-yaw, -pitch). Log world
			// directions with the same convention so dir.y > 0 means looking up, and world pitch = -internal (degrees).
			Fvector head_dir; head_dir.setHP(-head.current.yaw, -head.current.pitch);
			Fvector target_dir; target_dir.setHP(-head.target.yaw, -head.target.pitch);
			Fvector point = action.vector3d();
			const bool aim_solution_valid = type == SightManager::eSightTypeFireObject || type == SightManager::eSightTypePosition || type == SightManager::eSightTypeFirePosition;
			Fvector aim_solution; aim_solution.set(0.f, 0.f, 0.f);
			if (aim_solution_valid) aim_solution = action.vector3d();
			const int fire_mode = type == SightManager::eSightTypeFireObject ? int(action.state_fire_object()) : -1;
			const CGameObject* look_object = action.object_to_look();
			u32 object_id = u32(-1);
			bool point_is_position = (type == SightManager::eSightTypePosition) || (type == SightManager::eSightTypeFirePosition);
			if (look_object)
			{
				object_id = look_object->ID();
				point = look_object->Position();
				point_is_position = true;
			}
			float implied_pitch = 0.f;
			if (point_is_position)
			{
				const Fvector delta = Fvector().sub(point, eye);
				implied_pitch = atan2f(delta.y, _sqrt(delta.x * delta.x + delta.z * delta.z));
			}
			// B2 (AI session): the yaw the legs use (path segment, else the movement history), the movement-history yaw, the
			// selected legs direction, turn-in-place, and the clips on the legs and torso. Same yaw convention as head_yaw.
			bool const on_path = !object().movement().path().empty() &&
				(m_object->movement().detail().curr_travel_point_index() + 1 < m_object->movement().detail().path().size());
			float path_h = 0.f, path_p = 0.f, vel_h = 0.f, vel_p = 0.f;
			bool const path_ok = GetDirectionAngles(path_h, path_p);
			bool const vel_ok = GetDirectionAnglesByPrevPositions(vel_h, vel_p);
			float const path_yaw = path_ok ? rad2deg(angle_normalize_signed(-path_h)) : 999.f;
			float const vel_yaw = vel_ok ? rad2deg(angle_normalize_signed(-vel_h)) : 999.f;
			LPCSTR const path_src = on_path ? "path" : (path_ok ? "history" : "none");
			LPCSTR const legs_dirs[4] = {"forward", "backward", "left", "right"};
			u32 const legs_dir = m_object->animation().painter_legs_direction();
			IKinematicsAnimated* const look_animated = smart_cast<IKinematicsAnimated*>(m_object->Visual());
			LPCSTR const look_legs = painter_motion_name(look_animated, m_object->animation().legs().animation());
			LPCSTR const look_torso = painter_motion_name(look_animated, m_object->animation().torso().animation());

			Msg("[painter_look] t=%u id=%u name=%s type=%s torso_look=%u head_yaw=%.1f head_pitch_world=%.1f head_pitch_internal=%.1f head_dir_y=%.2f target_yaw=%.1f target_pitch_world=%.1f target_pitch_internal=%.1f target_dir_y=%.2f body_yaw=%.1f body_pitch_world=%.1f eye=%.2f,%.2f,%.2f point=%.2f,%.2f,%.2f point_is_position=%u object=%u implied_pitch_world=%.1f factors=%.2f/%.2f/%.2f path_yaw=%.1f path_src=%s vel_yaw=%.1f legs_dir=%s turning=%u legs=%s torso=%s point_source=%s aim_solution_valid=%u aim_solution=%.2f,%.2f,%.2f fire_mode=%d",
			    Device.dwTimeGlobal, object().ID(), object().cName().c_str(), painter_sight_type_name(type), unsigned(use_torso_look()),
			    rad2deg(head.current.yaw), -rad2deg(head.current.pitch), rad2deg(head.current.pitch), head_dir.y, rad2deg(head.target.yaw), -rad2deg(head.target.pitch), rad2deg(head.target.pitch), target_dir.y,
			    rad2deg(body.current.yaw), -rad2deg(body.current.pitch), eye.x, eye.y, eye.z, point.x, point.y, point.z, unsigned(point_is_position), object_id,
			    rad2deg(implied_pitch), m_current.m_head.m_factor, m_current.m_shoulder.m_factor, m_current.m_spine.m_factor,
			    path_yaw, path_src, vel_yaw, legs_dirs[legs_dir < 4 ? legs_dir : 0], unsigned(turning_in_place()), look_legs, look_torso, look_object ? "target_origin" : "sight_intent", unsigned(aim_solution_valid),
			    aim_solution.x, aim_solution.y, aim_solution.z, fire_mode);
		}

		if (g_painter_pose_log && Device.dwTimeGlobal - m_painter_pose_logged_at >= 1000)
		{
			m_painter_pose_logged_at = Device.dwTimeGlobal;
			IKinematics* kinematics = smart_cast<IKinematics*>(m_object->Visual());
			if (kinematics)
			{
				LPCSTR names[] = {"bip01_head", "bip01_pelvis", "bip01_spine2", "bip01_r_finger1", "bip01_l_finger1", "bip01_r_hand", "bip01_l_hand", "bip01_r_finger11"};
				Fvector p[8]; u16 ids[8]; bool ok = true;
				for (u32 i = 0; i < 8; ++i)
				{
					u16 const id = kinematics->LL_BoneID(names[i]);
					ids[i] = id;
					if (id == BI_NONE) { ok = false; break; }
					m_object->XFORM().transform_tiny(p[i], kinematics->LL_GetTransform(id).c);
				}
				if (ok)
				{
					Fvector const& head_p = p[0]; Fvector const& pelvis_p = p[1];
					Fvector barrel = Fvector().sub(p[4], p[3]); float const hands_dist = barrel.magnitude();
					if (hands_dist > EPS) barrel.div(hands_dist);
					float const barrel_pitch = asinf(clampr(barrel.y, -1.f, 1.f));
					Fvector trunk = Fvector().sub(head_p, pelvis_p); float const trunk_len = trunk.magnitude();
					if (trunk_len > EPS) trunk.div(trunk_len);
					float const torso_lean = acosf(clampr(trunk.y, -1.f, 1.f));
					bool const standing = m_object->movement().body_state() == MonsterSpace::eBodyStateStand;
					u32 flags = 0;
					if (p[5].y > head_p.y + .05f || p[6].y > head_p.y + .05f) flags |= 1;
					if (_abs(barrel_pitch) > deg2rad(45.f) && hands_dist > .15f) flags |= 2;
					if (standing && torso_lean > deg2rad(60.f)) flags |= 4;
					if (standing && head_p.y - pelvis_p.y < .35f) flags |= 8;
					// Motion currently selected on each partition, so a log reader can tell which clip an order produced.
					IKinematicsAnimated* animated = smart_cast<IKinematicsAnimated*>(m_object->Visual());
					LPCSTR const legs_name = painter_motion_name(animated, m_object->animation().legs().animation());
					LPCSTR const torso_name = painter_motion_name(animated, m_object->animation().torso().animation());
					float wpn_up = 0.f, wpn_fwd = 0.f, wpn_dx = 0.f;
					LPCSTR wpn_section = "-";
					Fvector sup_w, sup_h;
					sup_w.set(0.f, 0.f, 0.f);
					sup_h.set(0.f, 0.f, 0.f);
					{
						// stock one-hand weapon frame from the right index finger (model space), as CWeapon::UpdateXForm builds it
						Fmatrix const& finger = kinematics->LL_GetTransform(ids[3]);
						Fvector D = Fvector().sub(kinematics->LL_GetTransform(ids[7]).c, finger.c);
						if (D.square_magnitude() > EPS_L)
						{
							D.normalize();
							Fvector R = Fvector().crossproduct(finger.j, D);
							if (R.square_magnitude() > EPS_L)
							{
								R.normalize();
								Fvector const N = Fvector().crossproduct(D, R);
								Fvector const v = Fvector().sub(kinematics->LL_GetTransform(ids[4]).c, finger.c);
								sup_h.set(v.dotproduct(R), v.dotproduct(N), v.dotproduct(D));
							}
						}
					}
					if (CWeapon const* const weapon = smart_cast<CWeapon const*>(m_object->best_weapon()))
					{
						wpn_section = weapon->cNameSect().c_str();
						wpn_up = weapon->XFORM().j.y;
						wpn_fwd = weapon->XFORM().k.dotproduct(m_object->XFORM().k);
						wpn_dx = Fvector().sub(weapon->Position(), m_object->Position()).dotproduct(m_object->XFORM().i);
						Fmatrix weapon_inverse;
						weapon_inverse.invert(weapon->XFORM());
						weapon_inverse.transform_tiny(sup_w, m_object->painter_grip_blend() >= .5f ? p[3] : p[4]);
					}
					Msg("[painter_pose] t=%u id=%u name=%s alive=%u body=%u move=%u speed=%.2f scale=%.2f head_h=%.2f rhand_dh=%.2f lhand_dh=%.2f hands_dist=%.2f barrel_pitch=%.1f torso_lean=%.1f lean=%.2f head_dx=%.2f flags=%u legs=%s torso=%s shoulder=%u mirror=%u grip=%.2f wpn_up=%.2f wpn_fwd=%.2f wpn_dx=%.2f sup_w=%.3f,%.3f,%.3f sup_h=%.3f,%.3f,%.3f lh=%.2f lh_err=%.3f lh_short=%.3f lh_elbow=%.0f lh_fade=%.2f gait=%u/%u wpn=%s",
					    Device.dwTimeGlobal, m_object->ID(), m_object->cName().c_str(), unsigned(m_object->g_Alive()), unsigned(m_object->movement().body_state()),
					    unsigned(m_object->movement().movement_type()), m_object->m_fCurSpeed, m_object->painter_speed_scale(), head_p.y - pelvis_p.y,
					    p[5].y - head_p.y, p[6].y - head_p.y, hands_dist, rad2deg(barrel_pitch), rad2deg(torso_lean),
					    m_object->painter_lean(), Fvector().sub(head_p, pelvis_p).dotproduct(m_object->XFORM().i), flags, legs_name, torso_name,
						unsigned(m_object->painter_shoulder_left()), unsigned(m_object->animation().painter_torso_mirrored()), m_object->painter_grip_blend(), wpn_up, wpn_fwd, wpn_dx, sup_w.x, sup_w.y, sup_w.z, sup_h.x, sup_h.y, sup_h.z,
						painter_support_hand_weight(m_object->ID()), painter_support_hand_error(m_object->ID()), painter_support_hand_short(m_object->ID()),
							painter_support_hand_elbow(m_object->ID()), painter_support_hand_fade(m_object->ID()),
							unsigned(m_object->painter_gait()), m_object->painter_gait_effective(), wpn_section);
				}
			}
		}

#ifdef DEBUG
	if ( g_ai_dbg_sight )
		Msg							( "%6d [%s] after3 body[%f]->[%f], head[%f]->[%f]", Device.dwTimeGlobal, object().cName().c_str(), body.current.yaw, body.target.yaw, head.current.yaw, head.target.yaw );
#endif // #ifdef DEBUG

		if (object().animation_movement_controlled())
			return;

#ifdef HOLDERCUSTOM_NEW
		/* Don't update m_object->XFORM() here. Holder will update. */
		if (m_object->cast_stalker() && m_object->cast_stalker()->Holder())
			return;
#endif

		Fmatrix& m = m_object->XFORM();
		float h = -body.current.yaw;
		float _sh = _sin(h), _ch = _cos(h);
		m.i.set(_ch, 0.f, _sh);
		m._14_ = 0.f;
		m.j.set(0.f, 1.f, 0.f);
		m._24_ = 0.f;
		m.k.set(-_sh, 0.f, _ch);
		m._34_ = 0.f;

	STOP_PROFILE
}

void CSightManager::setup(const CSightAction& sight_action)
{
	if (m_actions.size() > 1)
		clear();

	if (!m_actions.empty() && (*(*m_actions.begin()).second == sight_action))
		return;

	clear();
	add_action(0, xr_new<CSightControlAction>(1.f, u32(-1), sight_action));
}

void CSightManager::update()
{
	START_PROFILE("Sight Manager")
		if (!enabled())
			return;

		if (!fis_zero(object().movement().speed()))
		{
			m_turning_in_place = false;
			inherited::update();
			return;
		}

		if (!m_turning_in_place)
		{
			if (angle_difference(object().movement().m_body.current.yaw, object().movement().m_head.current.yaw) > (
				left_angle(-object().movement().m_head.current.yaw, -object().movement().m_body.current.yaw)
					? m_max_left_angle
					: m_max_right_angle))
			{
				m_turning_in_place = true;
				//			Msg				("%6d started turning in place",Device.dwTimeGlobal);
				object().movement().m_body.target.yaw = object().movement().m_head.current.yaw;
			}
			else
				object().movement().m_body.target.yaw = object().movement().m_body.current.yaw;

			inherited::update();
			return;
		}

		if (angle_difference(object().movement().m_body.current.yaw, object().movement().m_head.target.yaw) > EPS_L)
		{
			//		object().movement().m_body.target.yaw	= object().movement().m_head.current.yaw;
			object().movement().m_body.target.yaw = object().movement().m_head.target.yaw;
		}
		else
		{
			m_turning_in_place = false;
			//		Msg					("%6d stopped turning in place",Device.dwTimeGlobal);
			object().movement().m_body.target.yaw = object().movement().m_body.current.yaw;
		}

		inherited::update();

	STOP_PROFILE
}

void CSightManager::remove_links(CObject* object)
{
	setup_actions::iterator I = actions().begin();
	setup_actions::iterator E = actions().end();
	for (; I != E; ++I)
		(*I).second->remove_links(object);
}

Fvector CSightManager::object_position() const
{
	CGameObject const* object = current_action().object_to_look();
	if (!object)
		return m_object->eye_matrix.c;
	Fvector look_pos;

	if (const_cast<CGameObject*>(object)->cast_actor())
		object->Visual()->dcast_PKinematics()->CalculateBBox(FALSE);

	object->Center(look_pos);

	const CEntityAlive* entity_alive = smart_cast<const CEntityAlive*>(object);
	if (!entity_alive || entity_alive->g_Alive())
	{
		look_pos.x = object->Position().x;
		look_pos.z = object->Position().z;
	}

	Fvector my_position = m_object->eye_matrix.c;
	Fvector target = look_pos;
	if (!aim_target(my_position, target, object))
		target = look_pos;

	return (target);
}

//#include "actor.h"
//CActor*			Actor()	;

Fvector CSightManager::aiming_position() const
{
	Fvector result;

#if 0
	Fmatrix								player_head;
	IKinematics* actor_kinematics		= smart_cast<IKinematics*>(Actor()->Visual());
	actor_kinematics->Bone_GetAnimPos	(player_head, actor_kinematics->LL_BoneID("bip01_head"), 1, false);
	player_head.mulA_43					(Actor()->XFORM());
	return								( player_head.c );
#endif // #if 0

#ifdef DEBUG
	result.set					(flt_max, flt_max, flt_max);
#endif // #ifdef DEBUG

	float const fake_distance = 10000.f;

	using namespace SightManager;

	switch (current_action().sight_type())
	{
	case eSightTypeCurrentDirection:
		{
			VERIFY2(_valid(object().Position()), make_string("[%f][%f][%f]", VPUSH(object().Position())));
			VERIFY2(_valid(-object().movement().m_head.current.yaw),
			        make_string("%f", -object().movement().m_head.current.yaw));
			VERIFY2(_valid(-object().movement().m_head.current.yaw),
			        make_string("%f", -object().movement().m_head.current.pitch));
			VERIFY(
				_valid(
					Fvector().setHP(
						-object().movement().m_head.current.yaw,
						-object().movement().m_head.current.pitch
					)
				)
			);
			result.mad(
				object().Position(),
				Fvector().setHP(
					-object().movement().m_head.current.yaw,
					-object().movement().m_head.current.pitch
				),
				fake_distance
			);

			VERIFY2(
				result.magnitude() < 100000.f,
				make_string(
					"[%f][%f][%f] [%f][%f] [%f]",
					VPUSH(object().Position()),
					-object().movement().m_head.current.yaw,
					-object().movement().m_head.current.pitch,
					fake_distance
				)
			);
			VERIFY(_valid(result));
			break;
		}
	case eSightTypePathDirection:
		{
			result.mad(
				object().Position(),
				Fvector().setHP(
					-object().movement().m_head.target.yaw,
					-object().movement().m_head.target.pitch
				),
				fake_distance
			);
			VERIFY2(
				result.magnitude() < 100000.f,
				make_string(
					"[%f][%f][%f] [%f][%f] [%f]",
					VPUSH(object().Position()),
					-object().movement().m_head.target.yaw,
					-object().movement().m_head.target.pitch,
					fake_distance
				)
			);
			VERIFY(_valid(result));
			break;
		}
	case eSightTypeDirection:
		{
			VERIFY(_valid(current_action().vector3d()));
			result.mad(
				object().Position(),
				current_action().vector3d(),
				fake_distance
			);
			VERIFY2(
				result.magnitude() < 100000.f,
				make_string(
					"[%f][%f][%f] [%f][%f][%f] [%f]",
					VPUSH(object().Position()),
					VPUSH(current_action().vector3d()),
					fake_distance
				)
			);
			VERIFY(_valid(result));
			break;
		}
	case eSightTypePosition:
	case eSightTypeFirePosition:
		{
			result = current_action().vector3d();
			VERIFY2(
				result.magnitude() < 100000.f,
				make_string(
					"[%f][%f][%f]",
					VPUSH(current_action().vector3d())
				)
			);
			VERIFY(_valid(current_action().vector3d()));
			break;
		}
	case eSightTypeObject:
		{
			result = object_position();
			VERIFY2(
				result.magnitude() < 100000.f,
				make_string(
					"[%f][%f][%f]",
					VPUSH(result)
				)
			);
			VERIFY(_valid(result));
			break;
		}
	case eSightTypeFireObject:
		{
			switch (current_action().state_fire_object())
			{
			case 0:
				{
					result = current_action().vector3d(); //object_position();
					VERIFY2(
						result.magnitude() < 100000.f,
						make_string(
							"[%f][%f][%f]",
							VPUSH(result)
						)
					);
					VERIFY(_valid(result));
					break;
				}
			case 1:
				{
					result = current_action().vector3d();
					VERIFY2(
						result.magnitude() < 100000.f,
						make_string(
							"[%f][%f][%f]",
							VPUSH(result)
						)
					);
					VERIFY(_valid(result));
					break;
				}
			default: NODEFAULT;
			}
			break;
		}
	case eSightTypeCover:
		{
			result.mad(
				object().Position(),
				Fvector().setHP(
					-object().movement().m_head.current.yaw,
					-object().movement().m_head.current.pitch
				),
				fake_distance
			);
			VERIFY2(
				result.magnitude() < 100000.f,
				make_string(
					"[%f][%f][%f] [%f][%f] [%f]",
					VPUSH(object().Position()),
					-object().movement().m_head.current.yaw,
					-object().movement().m_head.current.pitch,
					fake_distance
				)
			);
			VERIFY(_valid(result));
			break;
		}
	case eSightTypeSearch:
		{
			result.mad(
				object().Position(),
				Fvector().setHP(
					-object().movement().m_head.current.yaw,
					-object().movement().m_head.current.pitch
				),
				fake_distance
			);
			VERIFY2(
				result.magnitude() < 100000.f,
				make_string(
					"[%f][%f][%f] [%f][%f] [%f]",
					VPUSH(object().Position()),
					-object().movement().m_head.current.yaw,
					-object().movement().m_head.current.pitch,
					fake_distance
				)
			);
			VERIFY(_valid(result));
			break;
		}
	case eSightTypeLookOver:
		{
			result.mad(
				object().Position(),
				Fvector().setHP(
					-object().movement().m_head.current.yaw,
					-object().movement().m_head.current.pitch
				),
				fake_distance
			);
			VERIFY2(
				result.magnitude() < 100000.f,
				make_string(
					"[%f][%f][%f] [%f][%f] [%f]",
					VPUSH(object().Position()),
					-object().movement().m_head.current.yaw,
					-object().movement().m_head.current.pitch,
					fake_distance
				)
			);
			VERIFY(_valid(result));
			break;
		}
	case eSightTypeCoverLookOver:
		{
			result.mad(
				object().Position(),
				Fvector().setHP(
					-object().movement().m_head.current.yaw,
					-object().movement().m_head.current.pitch
				),
				fake_distance
			);
			VERIFY2(
				result.magnitude() < 100000.f,
				make_string(
					"[%f][%f][%f] [%f][%f] [%f]",
					VPUSH(object().Position()),
					-object().movement().m_head.current.yaw,
					-object().movement().m_head.current.pitch,
					fake_distance
				)
			);
			VERIFY(_valid(result));
			break;
		}
	case eSightTypeAnimationDirection:
		{
			result.mad(
				object().Position(),
				Fvector().setHP(
					-object().movement().m_body.current.yaw,
					-object().movement().m_body.current.pitch
				),
				fake_distance
			);
			VERIFY2(
				result.magnitude() < 100000.f,
				make_string(
					"[%f][%f][%f] [%f][%f] [%f]",
					VPUSH(object().Position()),
					-object().movement().m_head.current.yaw,
					-object().movement().m_head.current.pitch,
					fake_distance
				)
			);
			VERIFY(_valid(result));
			break;
		}
	default: NODEFAULT;
	}

	VERIFY2(
		result.magnitude() < 100000.f,
		make_string(
			"[%f][%f][%f] [%f][%f] [%f]",
			VPUSH(result)
		)
	);
	VERIFY(_valid(result));
	return (result);
}

static inline float lerp(float low, float high, float value)
{
	float result;
	if (low > high)
	{
		result = low - value;
		std::swap(low, high);
	}
	else
		result = low + value;;

	if (result >= high)
		return (high);

	if (result <= low)
		return (low);

	return (result);
}

void CSightManager::process_action(float const time_delta)
{
	VERIFY(_valid(time_delta));
	VERIFY(_valid(s_factor_lerp_speed));

	//	if ( current_action().sight_type() == SightManager::eSightTypeAnimationDirection ) {
	//		m_current.m_spine.m_rotation			= Fidentity;
	//		m_current.m_shoulder.m_rotation			= Fidentity;
	//		m_current.m_head.m_rotation				= Fidentity;
	//		return;
	//	}

	SBoneRotation const& head = object().movement().m_head;
	SBoneRotation const& body = object().movement().m_body;

	Fvector const& factors = current_action().use_torso_look() ? s_danger_factors : s_free_factors;
	VERIFY(_valid(factors));
	//	if ( object().cName() == "level_prefix_stalker" ) {
	//		Msg							("[%6d][%6d] [%f] + [%f] = [%f] ([%f])",  Device.dwFrame, Device.dwTimeGlobal, m_current.m_head.m_factor,		s_factor_lerp_speed*time_delta,		lerp ( m_current.m_head.m_factor,		factors.x, s_factor_lerp_speed*time_delta ), factors.x );
	//		Msg							("[%6d][%6d] [%f] + [%f] = [%f] ([%f])",  Device.dwFrame, Device.dwTimeGlobal, m_current.m_shoulder.m_factor,	s_factor_lerp_speed*time_delta,		lerp ( m_current.m_shoulder.m_factor,	factors.y, s_factor_lerp_speed*time_delta ), factors.y );
	//		Msg							("[%6d][%6d] [%f] + [%f] = [%f] ([%f])",  Device.dwFrame, Device.dwTimeGlobal, m_current.m_spine.m_factor,	s_factor_lerp_speed*time_delta,		lerp ( m_current.m_spine.m_factor,		factors.z, s_factor_lerp_speed*time_delta ), factors.z );
	//	}

	VERIFY(_valid(m_current.m_head.m_factor));
	m_current.m_head.m_factor = lerp(m_current.m_head.m_factor, factors.x, s_factor_lerp_speed * time_delta);
	VERIFY(_valid(m_current.m_head.m_factor));

	VERIFY(_valid(m_current.m_shoulder.m_factor));
	m_current.m_shoulder.m_factor = lerp(m_current.m_shoulder.m_factor, factors.y, s_factor_lerp_speed * time_delta);
	VERIFY(_valid(m_current.m_shoulder.m_factor));

	VERIFY(_valid(m_current.m_spine.m_factor));
	m_current.m_spine.m_factor = lerp(m_current.m_spine.m_factor, factors.z, s_factor_lerp_speed * time_delta);
	VERIFY(_valid(m_current.m_spine.m_factor));

	Fvector const angles =
		Fvector().set(
			angle_normalize_signed(-(head.current.pitch - body.current.pitch)),
			angle_normalize_signed(-(head.current.yaw - body.current.yaw)),
			angle_normalize_signed(head.current.roll - body.current.roll)
		);

	m_current.m_head.m_rotation.setXYZ(Fvector(angles).mul(m_current.m_head.m_factor));
	m_current.m_shoulder.m_rotation.setXYZ(Fvector(angles).mul(m_current.m_shoulder.m_factor));
	m_current.m_spine.m_rotation.setXYZ(Fvector(angles).mul(m_current.m_spine.m_factor));
}

// native17 (v2.24 Order::gait): the gait the legs, torso and speed use = the native action's request after the engine's
// guard. Aimed (0) whenever: no request; crouched; not running; the weapon is firing; the sight is on an enemy object;
// or the body faces more than 30 deg away from the path direction (the rifle-down cycles are forward-only).
u32 CAI_Stalker::painter_gait_effective() const
{
	if (!m_painter_gait)
		return 0;
	stalker_movement_manager_smart_cover const& move = movement();
	if (move.body_state() == MonsterSpace::eBodyStateCrouch)
		return 0;
	if (move.movement_type() != MonsterSpace::eMovementTypeRun)
		return 0;
	if (CWeapon const* const weapon = smart_cast<CWeapon const*>(inventory().ActiveItem()))
		if (weapon->GetState() == CWeapon::eFire || weapon->GetState() == CWeapon::eFire2 ||
			weapon->GetState() == CWeapon::eReload)
			return 0;
	const u32 sight_type = u32(sight().current_action().sight_type());
	if (sight_type == SightManager::eSightTypeObject || sight_type == SightManager::eSightTypeFireObject)
		return 0;
	float path_h = 0.f, path_p = 0.f;
	if (sight().GetDirectionAngles(path_h, path_p))
	{
		float const path_yaw = angle_normalize_signed(-path_h);
		if (angle_difference(path_yaw, move.body_orientation().current.yaw) > deg2rad(30.f))
			return 0;
	}
	return m_painter_gait;
}

void CSightManager::compute_aiming(float const time_delta, float const angular_speed)
{
	switch (m_aiming_type)
	{
	case aiming_none:
		{
			process_action(time_delta);
			return;
		}
	case aiming_weapon:
		{
			if (!enabled())
				return;

			if (current_action().sight_type() == SightManager::eSightTypeAnimationDirection)
			{
				m_target.m_spine.m_rotation = Fidentity;
				m_target.m_shoulder.m_rotation = Fidentity;
				m_target.m_head.m_rotation = Fidentity;
				break;
			}

			if (!object().best_weapon())
			{
				m_target.m_spine.m_rotation = Fidentity;
				m_target.m_shoulder.m_rotation = Fidentity;
				m_target.m_head.m_rotation = Fidentity;
				break;
			}

			VERIFY(m_animation_id.size());
			VERIFY(m_animation_frame != animation_frame_none);

			bool forward_blend_callbacks = object().animation().forward_blend_callbacks();
			bool backward_blend_callbacks = object().animation().backward_blend_callbacks();
			object().animation().remove_bone_callbacks();
			VERIFY(object().best_weapon());
			VERIFY(smart_cast<CWeapon const*>(object().best_weapon()));
			VERIFY(_valid(aiming_position()));
			aimers::weapon aimer(
				&object(),
				m_animation_id.c_str(),
				m_animation_frame == animation_frame_start,
				aiming_position(),
				pSettings->r_string(object().cNameSect().c_str(), "bone_spin"),
				pSettings->r_string(object().cNameSect().c_str(), "bone_shoulder"),
				pSettings->r_string(object().cNameSect().c_str(), "weapon_bone0"),
				pSettings->r_string(object().cNameSect().c_str(), "weapon_bone2"),
				*smart_cast<CWeapon const*>(object().best_weapon())
			);
			if (forward_blend_callbacks)
				object().animation().assign_bone_blend_callbacks(true);
			else
			{
				if (backward_blend_callbacks)
					object().animation().assign_bone_blend_callbacks(false);
				else
					object().animation().assign_bone_callbacks();
			}

			m_target.m_spine.m_rotation = aimer.get_bone(0);
			m_target.m_shoulder.m_rotation = aimer.get_bone(1);
			m_target.m_head.m_rotation = Fidentity;

			if (!forward_blend_callbacks && !backward_blend_callbacks)
			{
				if (!fis_zero(time_delta))
					slerp_rotations(time_delta, angular_speed);

				break;
			}

			m_current.m_head.m_rotation = m_target.m_head.m_rotation;
			m_current.m_shoulder.m_rotation = m_target.m_shoulder.m_rotation;
			m_current.m_spine.m_rotation = m_target.m_spine.m_rotation;

			break;
		}
	case aiming_head:
		{
			if (!enabled())
				return;

			if (current_action().sight_type() == SightManager::eSightTypeAnimationDirection)
			{
				m_target.m_spine.m_rotation = Fidentity;
				m_target.m_shoulder.m_rotation = Fidentity;
				m_target.m_head.m_rotation = Fidentity;
				break;
			}

			VERIFY(m_animation_id.size());
			VERIFY(m_animation_frame != animation_frame_none);
			LPCSTR bones[] =
			{
				pSettings->r_string(object().cNameSect().c_str(), "bone_spin"),
				pSettings->r_string(object().cNameSect().c_str(), "bone_shoulder"),
				pSettings->r_string(object().cNameSect().c_str(), "bone_head"),
			};

			bool forward_blend_callbacks = object().animation().forward_blend_callbacks();
			bool backward_blend_callbacks = object().animation().backward_blend_callbacks();
			object().animation().remove_bone_callbacks();
			VERIFY(_valid(aiming_position()));
			aimers::bone<3> aimer(
				&object(),
				m_animation_id.c_str(),
				m_animation_frame == animation_frame_start,
				aiming_position(),
				bones
			);
			if (forward_blend_callbacks)
				object().animation().assign_bone_blend_callbacks(true);
			else
			{
				if (backward_blend_callbacks)
					object().animation().assign_bone_blend_callbacks(false);
				else
					object().animation().assign_bone_callbacks();
			}

			m_target.m_spine.m_rotation = aimer.get_bone(0);
			m_target.m_shoulder.m_rotation = aimer.get_bone(1);
			m_target.m_head.m_rotation = aimer.get_bone(2);

			if (!forward_blend_callbacks && !backward_blend_callbacks)
			{
				if (!fis_zero(time_delta))
				{
#ifdef DEBUG
					Msg						( "!animation movement controller wasn't created" );
#endif // #ifdef DEBUG
					if (m_object->animation_movement())
						slerp_rotations(time_delta, m_object->animation_movement()->IsBlending() ? .1f : angular_speed);
				}

				break;
			}

			m_current.m_head.m_rotation = m_target.m_head.m_rotation;
			m_current.m_shoulder.m_rotation = m_target.m_shoulder.m_rotation;
			m_current.m_spine.m_rotation = m_target.m_spine.m_rotation;

			break;
		}
	default: NODEFAULT;
	}
}

static void slerp_rotations(float const time_delta, float const angular_speed, Fmatrix& current, Fmatrix const& target)
{
	VERIFY(!fis_zero(time_delta));
	VERIFY(!fis_zero(angular_speed));

	Fquaternion left;
	left.set(current);

	Fquaternion right;
	right.set(target);

	Fquaternion left_inversed;
	left_inversed.inverse(left);

	Fquaternion difference;
	difference.mul(right, left_inversed);

	Fvector axe;
	float angle;
	difference.get_axis_angle(axe, angle);

	if (fis_zero(angle))
	{
		current = target;
		return;
	}

	float const test_angle = clampr(_abs(angle), EPS_L, PI);
	float speed = angular_speed;
	float const test_speed = PI / 36.f;
	float const min_speed = PI / 180.f;
	if (angular_speed > test_speed)
	{
		if (test_angle < test_speed)
		{
			speed = test_angle;
			if ((speed < angular_speed) && (speed < min_speed))
				speed = min_speed;
		}
	}

	float const factor = clampr(time_delta / (angle / speed), 0.f, 1.f);
	if (fsimilar(1.f, factor))
	{
		current = target;
		return;
	}

	Fquaternion result;
	result.slerp(left, right, factor);

	current.rotation(result);
}

void CSightManager::slerp_rotations(float const time_delta, float const angular_speed)
{
	::slerp_rotations(time_delta, angular_speed, m_current.m_spine.m_rotation, m_target.m_spine.m_rotation);
	::slerp_rotations(time_delta, angular_speed, m_current.m_shoulder.m_rotation, m_target.m_shoulder.m_rotation);
	::slerp_rotations(time_delta, angular_speed, m_current.m_head.m_rotation, m_target.m_head.m_rotation);
}

void CSightManager::adjust_orientation()
{
	m_current.m_spine.m_rotation = Fidentity;
	m_current.m_shoulder.m_rotation = Fidentity;
	m_current.m_head.m_rotation = Fidentity;

	m_target.m_spine.m_rotation = Fidentity;
	m_target.m_shoulder.m_rotation = Fidentity;
	m_target.m_head.m_rotation = Fidentity;

	SBoneRotation& body = object().movement().m_body;
	object().XFORM().getXYZ(body.current.pitch, body.current.yaw, body.current.roll);
	body.current.pitch *= -1.f;
	body.current.yaw *= -1.f;
	body.current.roll = 0.f;
	body.target = body.current;

	SBoneRotation& head = object().movement().m_head;
	head.current = body.current;
	head.target = head.current;
}

void CSightManager::enable(bool const value)
{
	if (value == m_enabled)
		return;

	m_enabled = value;
	//	Msg					("[%d][%s] sight_enabled[%c]", Device.dwTimeGlobal, object().cName().c_str(), value ? '+' : '-');

	if (!m_enabled)
		return;

	if (!object().animation_movement())
		return;

	adjust_orientation();
}
