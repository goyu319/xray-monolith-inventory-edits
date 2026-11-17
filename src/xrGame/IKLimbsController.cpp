#include "stdafx.h"

#include "IKLimbsController.h"

#include "IK/IKLimb.h"
#include "physicsshellholder.h"

#include "ik_anim_state.h"
#include "../xrphysics/mathutils.h"
#include "../Include/xrRender/RenderVisual.h"
#include "../Include/xrRender/Kinematics.h"
//#include "ode_include.h"
#include "characterphysicssupport.h"
#include "../xrEngine/motion.h"
#ifdef DEBUG
#	include "PHDebug.h"
#endif // DEBUG

#include "Actor.h"

CIKLimbsController::CIKLimbsController(): m_object(0), m_legs_blend(0)
{
}

void CIKLimbsController::Create(CGameObject* O)
{
	VERIFY(O);
	m_legs_blend = 0;

	IKinematics* K = smart_cast<IKinematics*>(O->Visual());
	m_object = O;
	VERIFY(K);
	u16 sz = 2;
	if (K->LL_UserData() && K->LL_UserData()->section_exist("ik"))
		sz = K->LL_UserData()->r_u16("ik", "num_limbs");
	VERIFY(sz <= max_size);

	_bone_chains.reserve(sz);
	for (u16 i = 0; sz > i; ++i)
		LimbSetup();

	bool already_has_callbacks = !O->visual_callbacks().empty();
	O->add_visual_callback(IKVisualCallback);
	if (already_has_callbacks)
		std::swap(*(O->visual_callbacks().begin()), *(O->visual_callbacks().end() - 1));
	_pose_extrapolation.init(O->XFORM());
}


void CIKLimbsController::LimbSetup()
{
	_bone_chains.push_back(CIKLimb());

	IKinematicsAnimated* skeleton_animated = m_object->Visual()->dcast_PKinematicsAnimated();

	_bone_chains.back().Create((u16)_bone_chains.size() - 1, skeleton_animated, true);
}

void CIKLimbsController::LimbCalculate(SCalculateData& cd)
{
	cd.do_collide = m_legs_blend && !cd.m_limb->KinematicsAnimated()->LL_GetMotionDef(m_legs_blend->motionID)->marks.
	                                    empty(); //m_legs_blend->;
	cd.m_limb->ApplyState(cd);
}

void CIKLimbsController::LimbUpdate(CIKLimb& L)
{
	IKinematicsAnimated* skeleton_animated = m_object->Visual()->dcast_PKinematicsAnimated();
	VERIFY(skeleton_animated);
	L.Update(m_object, m_legs_blend, _pose_extrapolation);
}

IC void update_blend(CBlend* & b)
{
	if (b && CBlend::eFREE_SLOT == b->blend_state())
		b = 0;
}

IC float lerp(float t, float a, float b)
{
	return (a + t * (b - a));
}

void y_shift_bones(IKinematics* K, float shift)
{
	u16 bc = K->LL_BoneCount();
	for (u16 i = 0; bc > i; ++i)
		K->LL_GetTransform(i).c.y += shift;
}

float CIKLimbsController::LegLengthShiftLimit(float current_shift, const SCalculateData cd[max_size])
{
	float shift_down = -phInfinity;
	const u16 sz = (u16)_bone_chains.size();
	for (u16 j = 0; sz > j; ++j)
		if (cd[j].state.foot_step)
		{
			float s_down = cd[j].m_limb->ObjShiftDown(current_shift, cd[j]);
			if (shift_down < s_down)
				shift_down = s_down;
		}

	if (Actor() == m_object) {
		if (Actor()->active_cam() == EActorCameras::eacFirstEye) {
			clamp(shift_down, -0.1f, 0.1f);
		}
	}

	return shift_down;
}

static const float static_shift_object_speed = .2f;

float CIKLimbsController::StaticObjectShift(const SCalculateData cd[max_size])
{
	const float current_shift = _object_shift.shift();

	u16 cnt = 0;
	float shift_up = 0;
	const u16 sz = (u16)_bone_chains.size();
	for (u16 j = 0; sz > j; ++j)
		if (cd[j].state.foot_step)
		{
			float s_up = cd[j].cl_shift.y + current_shift;
			if (0.f < s_up)
			{
				shift_up += s_up;
				++cnt;
			}
		}
	if (0 < cnt)
		shift_up /= cnt;
	float shift_down = LegLengthShiftLimit(current_shift, cd);
	float shift = 0;
	if (shift_down > 0.f)
		shift = -shift_down;
	else if (-shift_down < shift_up)
		shift = -shift_down;
	else
		shift = shift_up;
	VERIFY(_valid( shift ));
	_object_shift.set_taget(shift, _abs(current_shift - shift) / static_shift_object_speed);
	return shift;
}

static float doun_shift_to_correct = 0.3f;
static float doun_shift_correct = 0.1f;

bool CIKLimbsController::PredictObjectShift(const SCalculateData cd[max_size])
{
	float predict_time_shift_down = FLT_MAX;
	float predict_time_shift_up = FLT_MAX;
	float predict_shift_down = 0.f;
	//float predict_shift_up = 0.f;
	bool shift_down = false;
	bool shift_up = false;
	const u16 sz = (u16)_bone_chains.size();
	float current_shift = _object_shift.shift();
	for (u16 j = 0; sz > j; ++j)
		if (!cd[j].state.foot_step)
		{
			float time = cd[j].m_limb->time_to_footstep();

			float lshift = cd[j].m_limb->footstep_shift();
			if (lshift < 0.f)
			{
				if (time < predict_time_shift_down)
				{
					predict_time_shift_down = time;
					predict_shift_down = lshift;
					shift_down = true;
				}
			}
			else if (current_shift < -doun_shift_to_correct && time < predict_time_shift_up)
			{
				predict_time_shift_up = time;

				shift_up = true;
			}
		}
	float predict_shift = 0;
	float predict_time_shift = FLT_MAX;

	if (shift_down)
	{
		predict_shift = predict_shift_down;
		predict_time_shift = predict_time_shift_down;
	}
	else if (shift_up)
	{
		predict_shift = 0;
		predict_time_shift = predict_time_shift_up;
	}
	else
		return false;
	//{
	//	predict_shift = 0;
	//	predict_time_shift = Device.fTimeDelta;
	//}


	if (predict_time_shift < EPS_S)
		predict_time_shift = Device.fTimeDelta;

	if (Actor() == m_object) {
		if (Actor()->active_cam() == EActorCameras::eacFirstEye) {
			clamp(predict_shift, -0.1f, 0.1f);
		}
	}

	_object_shift.set_taget(predict_shift, predict_time_shift);
	return true;
}


void CIKLimbsController::ObjectShift(float static_shift, const SCalculateData cd[max_size])
{
	u16 cnt_in_step = 0;
	const u16 sz = (u16)_bone_chains.size();
	for (u16 j = 0; sz > j; ++j)
		if (cd[j].m_limb->foot_step())
			++cnt_in_step;

	CPhysicsShellHolder* sh = smart_cast<CPhysicsShellHolder*>(m_object);
	VERIFY(sh);
	//CCharacterPhysicsSupport *ch = sh->character_physics_support();	
	_object_shift.freeze(!!Device.Paused()); //ch->interactive_motion() ||

	if (cnt_in_step != sz && PredictObjectShift(cd)) //cnt_in_step > 0 &&
		return;
	StaticObjectShift(cd);
}

void CIKLimbsController::ShiftObject(const SCalculateData cd[max_size])
{
	IKinematics* skeleton_animated = m_object->Visual()->dcast_PKinematics();
	VERIFY(skeleton_animated);
	//	u16 root = skeleton_animated->LL_GetBoneRoot( ) ;

	//CBoneData &BD=skeleton_animated->LL_GetData(root);

	const float y_shift = _object_shift.shift();
	const u16 bones_count = skeleton_animated->LL_BoneCount();
	for (u16 i = 0; i < bones_count; ++i)
		skeleton_animated->LL_GetTransform(i).c.y += y_shift;

	for (u16 i = 0; i < bones_count; ++i)
	{
		CBoneInstance& bi = skeleton_animated->LL_GetBoneInstance(i);
		if (bi.callback())
			bi.callback()(&bi);
		skeleton_animated->LL_GetTransform_R(i).c.y += y_shift;
	}
	//	skeleton_animated->LL_GetTransform(root).c.y += _object_shift.shift();
	//	skeleton_animated->Bone_Calculate(&BD, &Fidentity );
}

int ik_shift_object = 1;

void CIKLimbsController::Calculate()
{
	update_blend(m_legs_blend);

	Fmatrix& obj = m_object->XFORM();
#ifdef	DEBUG
	if( ph_dbg_draw_mask1.test( phDbgDrawIKSHiftObject ) )
		_object_shift.dbg_draw( obj, _pose_extrapolation, Fvector().set( 0,2.5f,0));
#endif

	SCalculateData cd[max_size];


	xr_vector<CIKLimb>::iterator i, b = _bone_chains.begin(), e = _bone_chains.end();
	for (i = b; e != i; ++i)
	{
		cd[i - b] = SCalculateData(*i, obj);
		LimbCalculate(cd[i - b]);
	}

	IKinematics* K = m_object->Visual()->dcast_PKinematics();
	u16 root = K->LL_GetBoneRoot();
	CBoneInstance& root_bi = K->LL_GetBoneInstance(root);

	BOOL sv_root_cb_ovwr = root_bi.callback_overwrite();
	BoneCallback sv_root_cb = root_bi.callback();

	root_bi.set_callback(root_bi.callback_type(), 0, root_bi.callback_param(), TRUE);


	if (ik_shift_object) //&& ! m_object->animation_movement_controlled( )
	{
		ShiftObject(cd);
	}

	const u16 sz = (u16)_bone_chains.size();
	for (u16 j = 0; sz > j; ++j)
		cd[j].m_limb->SetGoal(cd[j]);

	for (u16 j = 0; sz > j; ++j)
	{
		cd[j].m_limb->SolveBones(cd[j]);

#ifdef	DEBUG
	if( ph_dbg_draw_mask1.test( phDbgDrawIKPredict ) )
	{
		//IKinematics *K = m_object->Visual()->dcast_PKinematics( );
		u16 ref_bone_id = cd[j].m_limb->dbg_ref_bone_id();
		Fmatrix m =Fmatrix().mul( obj, K->LL_GetTransform( ref_bone_id ) );
		Fvector toe;
		cd[j].m_limb->dbg_ik_foot().ToePosition( toe );
		m.transform_tiny( toe );
		DBG_DrawLine( toe, Fvector().add( toe, Fvector().set( 0, -_object_shift.shift(), 0 ) ), D3DCOLOR_XRGB( 255, 0, 0 )  );
	}
#endif
	}
	ObjectShift(0, cd);


	root_bi.set_callback(root_bi.callback_type(), sv_root_cb, root_bi.callback_param(), sv_root_cb_ovwr);
}

void CIKLimbsController::Destroy(CGameObject* O)
{
#ifdef _DEBUG
	CPhysicsShellHolder*	Sh = smart_cast<CPhysicsShellHolder*>(O);
	VERIFY(Sh);
	CIKLimbsController* ik = Sh->character_ik_controller();
	VERIFY( ik );
	VERIFY( ik == this );
#endif

	O->remove_visual_callback(IKVisualCallback);
	xr_vector<CIKLimb>::iterator i = _bone_chains.begin(), e = _bone_chains.end();
	for (; e != i; ++i)
		i->Destroy();
	_bone_chains.clear();
}

// ---------------------------------------------------------------------------------------------------------------------
// painter: NPC support hand on the weapon.
// GAMMA places every NPC gun from the right hand (hand_dependence 1: bip01_r_finger1 -> second grip bone, plus the
// section's position / orientation offsets) and the left hand comes from the weapon slot's torso clip - one spot per
// slot, so short guns get the hand on the barrel. [npc_left_hand_grips] gives, per weapon section, the bip01_l_hand
// model-space matrix in the weapon's model-space frame (fitted offline on each visible gun: grip = inverse(W) * H, with
// W = right-hand frame * m_Offset). Every frame, after the animation, the aim rotation and the leg IK, the left arm is
// moved so the hand lands on weapon * grip: clavicle reach about the gun's up axis (smallest angle up to 60 deg, 95% arm
// reach) - or, for a target so close the elbow would fold under 70 deg (short guns in aimed poses), up to 25 deg the
// other way - two-bone arm with the elbow plane biased 70% down (and out to the side while folded), eased out while the
// elbow would still be under 40 deg (no collapsed arm), forearm roll that keeps the clip's wrist bend in the
// forearm, fingers re-evaluated from the clip under the new hand. Blended in/out over ~0.17 s; off while reloading,
// drawing, holstering, strapped, in a script or global animation, on the left shoulder, or without a table entry.
// Console: painter_lh_ik 0/1 (default 1); painter_lh_ik_test N slides the clip's own grip N mm along the barrel for
// guns without an entry (diagnostics only).
// ---------------------------------------------------------------------------------------------------------------------
#include "ai/stalker/ai_stalker.h"
#include "Weapon.h"
#include "Inventory.h"
#include "stalker_animation_manager.h"
#include "object_handler_space.h"
#include "object_handler_planner.h"

int g_painter_lh_ik = 1;
int g_painter_lh_ik_test = 0;

namespace
{
struct SPainterSupportHand
{
	IRenderVisual* visual = nullptr;
	u16 bones[4] = {BI_NONE, BI_NONE, BI_NONE, BI_NONE}; // clavicle, upper arm, forearm, hand
	float weight = 0.f;
	u32 time = 0;
	Fmatrix grip = Fidentity;
	bool grip_valid = false;
	float error = 0.f; // clip wrist to target distance before the correction, metres
	float short_of = 0.f; // remaining distance when the target is out of reach, metres
	float elbow = 0.f; // the solved elbow angle, degrees (0 while the IK is off)
	float fold = 1.f; // 1 = applied; eases to 0 while the arm would still fold shut (hand left on the clip's spot)
};

xr_map<u16, SPainterSupportHand> painter_support_hands;
xr_map<shared_str, std::pair<bool, Fmatrix>> painter_grip_table;

bool painter_grip_entry(shared_str const& section, Fmatrix& grip)
{
	auto it = painter_grip_table.find(section);
	if (it == painter_grip_table.end())
	{
		std::pair<bool, Fmatrix> entry(false, Fidentity);
		LPCSTR const table = "npc_left_hand_grips";
		if (section.size() && pSettings->section_exist(table) && pSettings->line_exist(table, section))
		{
			float v[12];
			if (12 == sscanf(pSettings->r_string(table, section.c_str()), "%f ,%f ,%f ,%f ,%f ,%f ,%f ,%f ,%f ,%f ,%f ,%f",
			                 &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8], &v[9], &v[10], &v[11]))
			{
				Fmatrix& m = entry.second;
				m.identity();
				m.i.set(v[0], v[1], v[2]);
				m.j.set(v[3], v[4], v[5]);
				m.k.set(v[6], v[7], v[8]);
				m.c.set(v[9], v[10], v[11]);
				entry.first = _valid(m) && !fis_zero(m.i.square_magnitude()) && !fis_zero(m.j.square_magnitude()) &&
					!fis_zero(m.k.square_magnitude());
			}
			if (!entry.first)
				Msg("! [painter_support_hand] bad [%s] entry for %s (12 numbers expected)", table, section.c_str());
		}
		it = painter_grip_table.insert(mk_pair(section, entry)).first;
	}
	grip = it->second.second;
	return it->second.first;
}

// rotation taking direction a onto direction b (no change when either is degenerate or they are parallel)
Fmatrix painter_rotation_between(Fvector a, Fvector b)
{
	Fmatrix r;
	r.identity();
	if (fis_zero(a.square_magnitude()) || fis_zero(b.square_magnitude()))
		return r;
	a.normalize();
	b.normalize();
	Fvector axis;
	axis.crossproduct(a, b);
	float const s = axis.magnitude();
	if (s < 1e-6f)
		return r;
	axis.div(s);
	float const angle = atan2f(s, a.dotproduct(b));
	r.rotation(axis, angle);
	Fvector t;
	r.transform_dir(t, a);
	if (t.dotproduct(b) < .999f)
		r.rotation(axis, -angle);
	return r;
}

// rotate a model-space bone matrix by rot about pivot
void painter_rotate_about(Fmatrix& m, Fmatrix const& rot, Fvector const& pivot)
{
	Fvector d;
	d.sub(m.c, pivot);
	Fmatrix r;
	r.mul_43(rot, m);
	Fvector rd;
	rot.transform_dir(rd, d);
	r.c.add(pivot, rd);
	m.set(r);
}

// signed rotation about axis that best turns matrix a's frame onto matrix b's (using the axis least aligned with it)
float painter_roll_about(Fvector const& axis, Fmatrix const& a, Fmatrix const& b)
{
	Fvector const* const av[3] = {&a.i, &a.j, &a.k};
	Fvector const* const bv[3] = {&b.i, &b.j, &b.k};
	int pick = 0;
	float least = flt_max;
	for (int n = 0; n < 3; ++n)
	{
		float const d = _abs(av[n]->dotproduct(axis)) / _max(EPS_S, av[n]->magnitude());
		if (d < least)
		{
			least = d;
			pick = n;
		}
	}
	Fvector u = *av[pick], w = *bv[pick];
	u.mad(axis, -u.dotproduct(axis));
	w.mad(axis, -w.dotproduct(axis));
	if (fis_zero(u.square_magnitude()) || fis_zero(w.square_magnitude()))
		return 0.f;
	u.normalize();
	w.normalize();
	Fvector c;
	c.crossproduct(u, w);
	float angle = atan2f(c.dotproduct(axis), u.dotproduct(w));
	Fmatrix r;
	r.rotation(axis, angle);
	Fvector t;
	r.transform_dir(t, u);
	if (t.dotproduct(w) < .999f)
		angle = -angle;
	return angle;
}

void __stdcall painter_support_bone_callback(CBoneInstance* B)
{
	B->mTransform.set(*static_cast<Fmatrix*>(B->callback_param()));
}
} // namespace

float painter_support_hand_weight(u16 id)
{
	auto const it = painter_support_hands.find(id);
	return it == painter_support_hands.end() ? 0.f : it->second.weight;
}

float painter_support_hand_error(u16 id)
{
	auto const it = painter_support_hands.find(id);
	return it == painter_support_hands.end() ? 0.f : it->second.error;
}

float painter_support_hand_short(u16 id)
{
	auto const it = painter_support_hands.find(id);
	return it == painter_support_hands.end() ? 0.f : it->second.short_of;
}

float painter_support_hand_elbow(u16 id)
{
	auto const it = painter_support_hands.find(id);
	return it == painter_support_hands.end() ? 0.f : it->second.elbow;
}

float painter_support_hand_fade(u16 id)
{
	auto const it = painter_support_hands.find(id);
	return it == painter_support_hands.end() ? 0.f : 1.f - it->second.fold;
}

static void painter_support_hand_update(CGameObject* O, IKinematics* K)
{
	CAI_Stalker* const stalker = smart_cast<CAI_Stalker*>(O);
	if (!stalker || !K)
		return;
	SPainterSupportHand& st = painter_support_hands[stalker->ID()];
	if (st.visual != O->Visual())
	{
		st = SPainterSupportHand();
		st.visual = O->Visual();
		LPCSTR const names[4] = {"bip01_l_clavicle", "bip01_l_upperarm", "bip01_l_forearm", "bip01_l_hand"};
		for (int b = 0; b < 4; ++b)
			st.bones[b] = K->LL_BoneID(names[b]);
	}
	for (u16 const b : st.bones)
		if (b == BI_NONE)
			return;
	float const dt = st.time ? _min(.1f, float(Device.dwTimeGlobal - st.time) * .001f) : 0.f;
	st.time = Device.dwTimeGlobal;

	CWeapon* const weapon = smart_cast<CWeapon*>(stalker->inventory().ActiveItem());
	int bone_l = -1, bone_r = -1, bone_r2 = -1;
	if (weapon)
		stalker->g_WeaponBones(bone_l, bone_r, bone_r2);
	bool const frame_ok = weapon && bone_r >= 0 && bone_r2 >= 0;
	u32 const state = weapon ? weapon->GetState() : 0;
	// slinging / unslinging are object-handler operators, not weapon states (the torso plays strap_* / unstrap_*)
	u32 const operation = stalker->CObjectHandler::planner().current_action_state_id();
	bool const slinging = operation == ObjectHandlerSpace::eWorldOperatorStrapping ||
		operation == ObjectHandlerSpace::eWorldOperatorUnstrapping ||
		operation == ObjectHandlerSpace::eWorldOperatorStrapping2Idle ||
		operation == ObjectHandlerSpace::eWorldOperatorUnstrapping2Idle;
	bool want = g_painter_lh_ik && frame_ok && stalker->g_Alive() && !weapon->strapped_mode() && !slinging &&
		weapon->HandDependence() == hd1Hand && state != CWeapon::eReload && state != CHUDState::eShowing &&
		state != CHUDState::eHiding && state != CHUDState::eHidden && stalker->animation().script_animations().empty() &&
		stalker->painter_grip_blend() <= 0.f;
	Fmatrix grip;
	bool const entry = want && painter_grip_entry(weapon->cNameSect(), grip);
	if (want && !entry && !g_painter_lh_ik_test)
		want = false;
	float const rate = 6.f;
	st.weight = want ? _min(1.f, st.weight + rate * dt) : _max(0.f, st.weight - rate * dt);
	if (!frame_ok || st.weight <= 0.f)
	{
		st.error = st.short_of = st.elbow = 0.f;
		st.fold = 1.f;
		return;
	}

	// weapon frame in the NPC's model space, exactly as CWeapon::UpdateXForm builds it (without the NPC's world xform)
	Fmatrix const& mR = K->LL_GetTransform(u16(bone_r));
	Fmatrix const& mL = K->LL_GetTransform(u16(bone_r2));
	Fvector D;
	D.sub(mL.c, mR.c);
	if (fis_zero(D.square_magnitude()))
		return;
	D.normalize();
	Fvector R;
	R.crossproduct(mR.j, D);
	Fvector N;
	N.crossproduct(D, R);
	if (!_valid(N) || fis_zero(N.square_magnitude()))
		return;
	N.normalize();
	Fmatrix frame;
	frame.identity();
	frame.i.set(R);
	frame.j.set(N);
	frame.k.set(D);
	frame.c.set(mR.c);
	Fmatrix W;
	W.mul_43(frame, weapon->get_mOffset());

	if (want)
	{
		if (!entry)
		{
			// diagnostics: the clip's own grip, slid along the barrel
			Fmatrix W_inverse;
			W_inverse.invert(W);
			grip.mul_43(W_inverse, K->LL_GetTransform(st.bones[3]));
			grip.c.z += float(g_painter_lh_ik_test) * .001f;
		}
		st.grip = grip;
		st.grip_valid = true;
	}
	if (!st.grip_valid)
		return;
	Fmatrix T;
	T.mul_43(W, st.grip);

	Fmatrix const C0 = K->LL_GetTransform(st.bones[0]), U0 = K->LL_GetTransform(st.bones[1]),
		F0 = K->LL_GetTransform(st.bones[2]), H0 = K->LL_GetTransform(st.bones[3]);
	Fmatrix C = C0, U = U0, F = F0, H = H0;
	st.error = H0.c.distance_to(T.c);
	float const l1 = U0.c.distance_to(F0.c), l2 = F0.c.distance_to(H0.c);
	if (l1 < EPS_L || l2 < EPS_L)
		return;
	// 95% of the arm (the offline fit's limit, so the fitted clips themselves sit exactly on it) plus 2 mm of rounding
	float const reach = (l1 + l2) * .95f + .002f;

	// 1. clavicle reach about the gun's up axis: the smallest rotation (either way, up to 60 deg) that brings the
	//    shoulder within 95% of the arm's length of the target
	Fvector up = W.j;
	up.normalize_safe();
	auto shoulder_at = [&](float degrees) {
		Fmatrix r;
		r.rotation(up, deg2rad(degrees));
		Fvector d;
		d.sub(U0.c, C0.c);
		Fvector rd;
		r.transform_dir(rd, d);
		return Fvector().add(C0.c, rd);
	};
	// the elbow's inner angle for a shoulder-to-hand distance d (180 = straight, 0 = folded shut)
	auto elbow_deg = [&](float d) {
		float const c = (l1 * l1 + l2 * l2 - d * d) / (2.f * l1 * l2);
		return rad2deg(acosf(_max(-1.f, _min(1.f, c))));
	};
	auto swing_clavicle = [&](float degrees) {
		Fmatrix r;
		r.rotation(up, deg2rad(degrees));
		Fvector const pivot = C0.c;
		painter_rotate_about(C, r, pivot);
		painter_rotate_about(U, r, pivot);
		painter_rotate_about(F, r, pivot);
		painter_rotate_about(H, r, pivot);
	};
	float const open_deg = 70.f;
	if (shoulder_at(0.f).distance_to(T.c) > reach)
	{
		float const sign = shoulder_at(10.f).distance_to(T.c) < shoulder_at(-10.f).distance_to(T.c) ? 1.f : -1.f;
		float lo = 0.f, hi = 60.f;
		if (shoulder_at(sign * hi).distance_to(T.c) <= reach)
			for (int n = 0; n < 24; ++n)
			{
				float const mid = .5f * (lo + hi);
				if (shoulder_at(sign * mid).distance_to(T.c) > reach)
					lo = mid;
				else
					hi = mid;
			}
		swing_clavicle(sign * hi);
	}
	else if (elbow_deg(shoulder_at(0.f).distance_to(T.c)) < open_deg)
	{
		float const sign = shoulder_at(10.f).distance_to(T.c) > shoulder_at(-10.f).distance_to(T.c) ? 1.f : -1.f;
		float lo = 0.f, hi = 25.f;
		if (elbow_deg(shoulder_at(sign * hi).distance_to(T.c)) >= open_deg)
			for (int n = 0; n < 20; ++n)
			{
				float const mid = .5f * (lo + hi);
				if (elbow_deg(shoulder_at(sign * mid).distance_to(T.c)) < open_deg)
					lo = mid;
				else
					hi = mid;
			}
		swing_clavicle(sign * hi);
	}

	// 2. two-bone arm, fixed segment lengths; an unreachable target leaves the hand at full reach toward it
	Fvector const S = U.c;
	Fvector target = T.c;
	Fvector span;
	span.sub(target, S);
	float dist = span.magnitude();
	float const max_dist = (l1 + l2) * .999f;
	st.short_of = 0.f;
	if (dist > max_dist)
	{
		st.short_of = dist - max_dist;
		span.mul(max_dist / dist);
		target.add(S, span);
		dist = max_dist;
	}
	if (dist < _abs(l1 - l2) + 1e-4f)
		return;
	float const theta = elbow_deg(dist);
	st.elbow = theta;
	// still folded shut after the swing: ease the correction out (the hand stays on the clip's own spot) rather than show a
	// collapsed arm; back in above 45 deg
	float const fold_target = theta < 40.f ? 0.f : theta >= 45.f ? 1.f : st.fold;
	st.fold = fold_target > st.fold ? _min(fold_target, st.fold + rate * dt) : _max(fold_target, st.fold - rate * dt);
	Fvector dir;
	dir.div(span, dist);
	Fvector bend;
	bend.sub(F.c, S);
	bend.mad(dir, -bend.dotproduct(dir));
	if (fis_zero(bend.square_magnitude()))
		bend.set(0.f, -1.f, 0.f);
	bend.normalize();
	Fvector down;
	down.set(0.f, -1.f, 0.f);
	down.mad(dir, -down.dotproduct(dir));
	if (!fis_zero(down.square_magnitude()))
	{
		down.normalize();
		bend.lerp(bend, down, .7f);
		bend.normalize_safe();
	}
	// a folded arm points its elbow down AND out (toward this shoulder's side of the body), not into the ribs
	if (theta < 90.f)
	{
		Fvector out;
		out.sub(S, C0.c);
		out.mad(dir, -out.dotproduct(dir));
		if (!fis_zero(out.square_magnitude()))
		{
			out.normalize();
			bend.lerp(bend, out, .6f * _min(1.f, (90.f - theta) / 50.f));
			bend.normalize_safe();
		}
	}
	float const x = (l1 * l1 - l2 * l2 + dist * dist) / (2.f * dist);
	float const y = _sqrt(_max(0.f, l1 * l1 - x * x));
	Fvector E;
	E.mad(S, dir, x);
	E.mad(bend, y);
	Fmatrix const swing_upper = painter_rotation_between(Fvector().sub(F.c, S), Fvector().sub(E, S));
	painter_rotate_about(U, swing_upper, S);
	painter_rotate_about(F, swing_upper, S);
	painter_rotate_about(H, swing_upper, S);
	Fmatrix const swing_fore = painter_rotation_between(Fvector().sub(H.c, F.c), Fvector().sub(target, F.c));
	painter_rotate_about(F, swing_fore, F.c);
	painter_rotate_about(H, swing_fore, F.c);

	// 3. hand at the grip orientation; forearm rolled about its own axis so it carries the clip's forearm-to-hand bend
	Fmatrix hand = T;
	hand.c.set(H.c);
	Fmatrix forearm_to_hand;
	forearm_to_hand.mul_43(Fmatrix().invert(F0), H0);
	Fmatrix forearm_wanted;
	forearm_wanted.mul_43(hand, Fmatrix().invert(forearm_to_hand));
	Fvector axis;
	axis.sub(hand.c, F.c);
	axis.normalize_safe();
	float const roll = painter_roll_about(axis, F, forearm_wanted);
	if (!fis_zero(roll))
	{
		Fmatrix r;
		r.rotation(axis, roll);
		painter_rotate_about(F, r, F.c);
	}

	// 4. blend with the clip by the weight (times the fold fade), positions rebuilt down the chain from the clip's bone offsets
	float const w = st.weight * st.fold;
	if (w <= 0.f)
		return;
	auto blend = [&](Fmatrix const& a, Fmatrix const& b, Fmatrix& out) {
		Fquaternion qa, qb, q;
		qa.set(a);
		qb.set(b);
		q.slerp(qa, qb, w);
		out.rotation(q);
	};
	Fmatrix out[4];
	blend(C0, C, out[0]);
	out[0].c.set(C0.c);
	Fmatrix const* const clip[4] = {&C0, &U0, &F0, &H0};
	Fmatrix const* const solved[4] = {&C, &U, &F, &hand};
	for (int b = 1; b < 4; ++b)
	{
		blend(*clip[b], *solved[b], out[b]);
		Fvector local;
		Fmatrix().invert(*clip[b - 1]).transform_tiny(local, clip[b]->c);
		out[b - 1].transform_tiny(out[b].c, local);
	}
	for (Fmatrix const& m : out)
		if (!_valid(m))
			return;

	// 5. write the four bones and re-evaluate their subtree (fingers from the clip, render matrices)
	struct SSaved
	{
		u32 type;
		BoneCallback callback;
		void* param;
		BOOL overwrite;
	} saved[4];
	for (int b = 0; b < 4; ++b)
	{
		CBoneInstance& bi = K->LL_GetBoneInstance(st.bones[b]);
		saved[b] = {bi.callback_type(), bi.callback(), bi.callback_param(), bi.callback_overwrite()};
		bi.set_callback(bctCustom, painter_support_bone_callback, &out[b], TRUE);
	}
	CBoneData& clavicle = K->LL_GetData(st.bones[0]);
	K->Bone_Calculate(&clavicle, &K->LL_GetTransform(clavicle.GetParentID()));
	for (int b = 0; b < 4; ++b)
		K->LL_GetBoneInstance(st.bones[b]).set_callback(saved[b].type, saved[b].callback, saved[b].param, saved[b].overwrite);
}

void _stdcall CIKLimbsController::IKVisualCallback(IKinematics* K)
{
	//if (Device.Paused())
	//	return;

#ifdef DEBUG
	if( ph_dbg_draw_mask1.test( phDbgIKOff ) )
		return;
#endif

	if(CGameObject* O = (CGameObject*)K->GetUpdateCallbackParam())
	{
		if(CPhysicsShellHolder* Sh = O->cast_physics_shell_holder())
		{
			if(CIKLimbsController* ik = Sh->character_ik_controller())
			{
			    if(!Sh->m_pPhysicsShell)
			    {
				    Fvector ce;
				    O->Center(ce);
				    if (Render->ViewBase.testSphere_dirty(ce, O->Radius()))
				    {
					    if (Device.vCameraPosition.distance_to_sqr(O->Position()) > 3000.f)
					    {
						    //if (Device.dwFrame < ik->optimize_frame)
						    IKinematics* K = O->Visual()->dcast_PKinematics();
						    u16 root = K->LL_GetBoneRoot();
						    CBoneInstance& root_bi = K->LL_GetBoneInstance(root);
						    root_bi.reset_callback();
						    return;
						    //ik->optimize_frame = Device.dwFrame + Random.randI(8);
					    }

                        {
                            PROF_EVENT("IK_UPDATE_CALCULATE");
                            ik->_pose_extrapolation.update(O->XFORM());
                            for (CIKLimb& limb : ik->_bone_chains)
                                ik->LimbUpdate(limb);

                            ik->Calculate();
                            painter_support_hand_update(O, K); // after the leg IK may have shifted the body
                        }
					    
				    }
				    else
				    {
					    IKinematics* K = O->Visual()->dcast_PKinematics();
					    u16 root = K->LL_GetBoneRoot();
					    CBoneInstance& root_bi = K->LL_GetBoneInstance(root);
					    root_bi.reset_callback();
				    }
			    }
			}
		}
	}
}

void CIKLimbsController::PlayLegs(CBlend* b)
{
	m_legs_blend = b;
#ifdef DEBUG
	IKinematicsAnimated *skeleton_animated = m_object->Visual( )->dcast_PKinematicsAnimated( );
	VERIFY( skeleton_animated );
	anim_name = skeleton_animated->LL_MotionDefName_dbg( b->motionID ).first;
	anim_set_name = skeleton_animated->LL_MotionDefName_dbg( b->motionID ).second;

	CMotionDef	&MD = *skeleton_animated->LL_GetMotionDef( b->motionID );
	if( MD.marks.empty() && ( MD.flags & esmUseFootSteps ) )
		Msg( "! No foot stseps for animation: animation name: %s, animation set: %s ", anim_name, anim_set_name );
#endif
}

void CIKLimbsController::Update()
{
#ifdef DEBUG
	if( ph_dbg_draw_mask1.test( phDbgIKOff ) )
		return;
#endif
	IKinematicsAnimated* skeleton_animated = m_object->Visual()->dcast_PKinematicsAnimated();
	VERIFY(skeleton_animated);

	skeleton_animated->UpdateTracks();
	update_blend(m_legs_blend);

	/*
	Fmatrix predict;
	_pose_extrapolation.extrapolate( predict, Device.fTimeGlobal  ); 




	DBG_DrawMatrix( m_object->XFORM(), 1 );
	DBG_DrawMatrix( predict, 1 );
	
	_pose_extrapolation.extrapolate( predict, Device.fTimeGlobal + 1  ); 
	DBG_DrawMatrix( predict, 1 );
	*/
}
