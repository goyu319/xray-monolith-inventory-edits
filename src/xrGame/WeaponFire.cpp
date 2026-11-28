// WeaponFire.cpp: implementation of the CWeapon class.
// function responsible for firing with CWeapon
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Weapon.h"
#include "entity.h"
#include "actor.h"

#include "actoreffector.h"
#include "effectorshot.h"

#include "level_bullet_manager.h"

#include "game_cl_mp.h"
#include "reward_event_generator.h"
#include "../Layers/xrRender/xrRender_console.h"

#include "PainterKillcamAim.h"
#include "painter_hit_ammo.h"
#include "level.h"
#include "HUDManager.h"
#include "inventory.h"
#include "../xrEngine/CameraBase.h"
#define FLAME_TIME 0.05f
// Qualified, short-lived player aim authority; never stores a target point or
// rewrites generated bullets. GameObject retirement invalidates all raw keys.
namespace painter_killcam_aim {
namespace {
struct AimLease {
    const CActor* actor{};
    const CWeapon* weapon{};
    const CGameObject* target{};
    const CLevel* world{};
    u32 time{};
};
xrCriticalSection aim_lock;
AimLease aim_lease;
bool current(CActor& actor, CWeapon& weapon) {
    return g_pGameLevel && Level().CurrentControlEntity() == &actor && actor.g_Alive()
        && weapon.H_Parent() == &actor
        && smart_cast<CWeapon*>(actor.inventory().ActiveItem()) == &weapon;
}
}
bool origin(CActor& actor, CWeapon& weapon, Fvector& point) {
    if (!current(actor, weapon)) return false;
    // The physical world weapon frame is independent of HUD/Device projection.
    static_cast<CHudItem&>(weapon).UpdateXForm();
    const Fvector& local = weapon.SilencerAttachable() && weapon.IsSilencerAttached()
        ? weapon.vLoadedFirePointSilencer : weapon.vLoadedFirePoint;
    weapon.XFORM().transform_tiny(point, local);
    return _valid(point);
}
bool renew(CActor& actor, CWeapon& weapon, CGameObject& target) {
    if (!current(actor, weapon) || Level().Objects.net_Find(target.ID()) != &target
        || &target == &actor || &target == &weapon) return false;
    xrCriticalSectionGuard guard(&aim_lock);
    aim_lease.actor=&actor; aim_lease.weapon=&weapon; aim_lease.target=&target;
    aim_lease.world=&Level(); aim_lease.time=Device.dwTimeContinual;
    return true;
}
void clear(CActor& actor) {
    xrCriticalSectionGuard guard(&aim_lock);
    if (aim_lease.actor == &actor) aim_lease=AimLease{};
}
void retire(const CGameObject& object) {
    xrCriticalSectionGuard guard(&aim_lock);
    if (aim_lease.actor == &object || aim_lease.weapon == &object || aim_lease.target == &object)
        aim_lease=AimLease{};
}
bool apply(CWeapon& weapon, Fvector& point, Fvector& direction) {
    CActor* actor=smart_cast<CActor*>(weapon.H_Parent());
    if (!actor || !current(*actor, weapon)) return false;
    {
        xrCriticalSectionGuard guard(&aim_lock);
        if (aim_lease.actor != actor || aim_lease.weapon != &weapon
            || aim_lease.world != &Level() || u32(Device.dwTimeContinual-aim_lease.time)>100)
            return false;
    }
    CCameraBase* camera=actor->cam_Active();
    Fvector muzzle;
    if (!camera || !origin(*actor, weapon, muzzle) || !_valid(camera->vPosition)
        || !_valid(camera->yaw) || !_valid(camera->pitch)) return false;
    Fvector base;
    base.setHP(-camera->yaw, -camera->pitch);
    // Match stock CHudItem::Ray's first-person eye-to-barrel obstruction
    // treatment, with the underlying physical eye instead of cinematic Device.
    SPickParam clearance(CDB::OPT_CULL | CDB::OPT_ONLYFIRST);
    clearance.defs.start=camera->vPosition;
    clearance.defs.dir.sub(muzzle, camera->vPosition);
    clearance.defs.range=clearance.defs.dir.magnitude();
    if (clearance.defs.range>EPS_L) {
        clearance.defs.dir.div(clearance.defs.range);
        if (HUD().DoPick(clearance)) {
            point=clearance.defs.start;
            direction=clearance.defs.dir;
            return true;
        }
    }
    point=muzzle; direction=base;
    return true;
}
} // namespace painter_killcam_aim



float _nrand(float sigma)
{
#define ONE_OVER_SIGMA_EXP (1.0f / 0.7975f)

	if (sigma == 0) return 0;

	float y;
	do
	{
		y = -logf(Random.randF());
	}
	while (Random.randF() > expf(-_sqr(y - 1.0f) * 0.5f));
	if (rand() & 0x1) return y * sigma * ONE_OVER_SIGMA_EXP;
	else return -y * sigma * ONE_OVER_SIGMA_EXP;
}

void random_dir(Fvector& tgt_dir, const Fvector& src_dir, float dispersion)
{
	float sigma = dispersion / 3.f;
	float alpha = clampr(_nrand(sigma), -dispersion, dispersion);
	float theta = Random.randF(0, PI);
	float r = tan(alpha);
	Fvector U, V, T;
	Fvector::generate_orthonormal_basis(src_dir, U, V);
	U.mul(r * _sin(theta));
	V.mul(r * _cos(theta));
	T.add(U, V);
	tgt_dir.add(src_dir, T).normalize();
}

float CWeapon::GetWeaponDeterioration()
{
	return conditionDecreasePerShot * f_weapon_deterioration;
}

void CWeapon::FireStart()
{
	if (ParentIsActor())
		Actor()->StopSprint();

	CShootingObject::FireStart();
}

void CWeapon::FireTrace(const Fvector& native_point, const Fvector& native_direction)
{
	Fvector P = native_point, D = native_direction;
	painter_killcam_aim::apply(*this, P, D);
	VERIFY(m_magazine.size());

	CCartridge& l_cartridge = m_magazine.back();
	//	Msg("ammo - %s", l_cartridge.m_ammoSect.c_str());
	VERIFY(u16(-1) != l_cartridge.bullet_material_idx);
	//-------------------------------------------------------------	
	bool is_tracer = m_bHasTracers && !!l_cartridge.m_flags.test(CCartridge::cfTracer);
	if (is_tracer && !IsGameTypeSingle())
		is_tracer = is_tracer /*&& (m_magazine.size() % 3 == 0)*/ && !IsSilencerAttached();

	l_cartridge.m_flags.set(CCartridge::cfTracer, is_tracer);
	if (m_u8TracerColorID != u8(-1))
		l_cartridge.param_s.u8ColorID = m_u8TracerColorID;
	//-------------------------------------------------------------
	//повысить изношенность оружия с учетом влияния конкретного патрона
	//	float Deterioration = GetWeaponDeterioration();
	//	Msg("Deterioration = %f", Deterioration);
	ChangeCondition(-GetWeaponDeterioration() * l_cartridge.param_s.impair * cur_silencer_koef.condition_shot_dec);


	float fire_disp = 0.f;
	CActor* tmp_actor = NULL;
	if (!IsGameTypeSingle())
	{
		tmp_actor = smart_cast<CActor*>(Level().CurrentControlEntity());
		if (tmp_actor)
		{
			CEntity::SEntityState state;
			tmp_actor->g_State(state);
			if (m_first_bullet_controller.is_bullet_first(state.fVelocity))
			{
				fire_disp = m_first_bullet_controller.get_fire_dispertion();
				m_first_bullet_controller.make_shot();
			}
		}
		game_cl_mp* tmp_mp_game = smart_cast<game_cl_mp*>(&Game());
		VERIFY(tmp_mp_game);
		if (tmp_mp_game->get_reward_generator())
			tmp_mp_game->get_reward_generator()->OnWeapon_Fire(H_Parent()->ID(), ID());
	}
	if (fsimilar(fire_disp, 0.f))
	{
		//CActor* tmp_actor = smart_cast<CActor*>(Level().CurrentControlEntity());
		if (H_Parent() && (H_Parent() == tmp_actor))
		{
			fire_disp = tmp_actor->GetFireDispertion();
		}
		else
		{
			fire_disp = GetFireDispersion(true);
		}
	}


	bool SendHit = SendHitAllowed(H_Parent());
	//выстерлить пулю (с учетом возможной стрельбы дробью)
	{
		const painter_cartridge::Scope fired(&l_cartridge);
		for (int i = 0; i < l_cartridge.param_s.buckShot; ++i)
			FireBullet(P, D, fire_disp, l_cartridge, H_Parent()->ID(), ID(), SendHit, iAmmoElapsed);
	}

	StartShotParticles();

	if (m_bLightShotEnabled)
		Light_Start();

	// Interactive Grass FX
	extern Fvector4 ps_ssfx_int_grass_params_2;
	Fvector ShotPos = Fvector().mad(P, D, 1.5f);
	g_pGamePersistent->GrassBendersAddShot(cast_game_object()->ID(), ShotPos, D, 3.0f, 20.0f, ps_ssfx_int_grass_params_2.z, ps_ssfx_int_grass_params_2.w);

	// Ammo
	m_lastCartridge = l_cartridge;
	m_magazine.pop_back();
	--iAmmoElapsed;

	VERIFY((u32)iAmmoElapsed == m_magazine.size());
}

void CWeapon::StopShooting()
{
	//	SetPending			(TRUE);

	//принудительно останавливать зацикленные партиклы
	if (m_pFlameParticles && m_pFlameParticles->IsLooped())
		StopFlameParticles();

	if (!m_current_motion_def || !m_playFullShotAnim)
		SwitchState(eIdle);

	FireEnd();
}

void CWeapon::FireEnd()
{
	CShootingObject::FireEnd();
	StopShotEffector();
}

//--DSR-- SilencerOverheat_start
void CWeapon::FireBullet(const Fvector& pos,
	const Fvector& shot_dir,
	float fire_disp,
	const CCartridge& cartridge,
	u16 parent_id,
	u16 weapon_id,
	bool send_hit, int iShotNum)
{
	CShootingObject::FireBullet(pos, shot_dir, fire_disp, cartridge, parent_id, weapon_id, send_hit, iShotNum);
	
	temperature += sil_glow_shot_temp;
	if (temperature > sil_glow_max_temp)
		temperature = sil_glow_max_temp;
}

float CWeapon::GetGlowing()
{
	return temperature;
}
//--DSR-- SilencerOverheat_end

void CWeapon::StartFlameParticles2()
{
	CShootingObject::StartParticles(m_pFlameParticles2, *m_sFlameParticles2, get_LastFP2());
}

void CWeapon::StopFlameParticles2()
{
	CShootingObject::StopParticles(m_pFlameParticles2);
}

void CWeapon::UpdateFlameParticles2()
{
	if (m_pFlameParticles2) CShootingObject::UpdateParticles(m_pFlameParticles2, get_LastFP2());
}
