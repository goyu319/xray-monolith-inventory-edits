#include "stdafx.h"
#include "active_ragdoll_controller.h"

#include "Entity_Alive.h"
#include "Hit.h"
#include "../Include/xrRender/Kinematics.h"
#include "../xrPhysics/PhysicsShell.h"

namespace
{
    // This is deliberately a low authority controller.  The shell remains the
    // source of truth and the controller only shapes the first few seconds of
    // a stalker death before giving the body back to native physics.
    static const float kImpactDuration = 0.12f;
    static const float kRecoverDuration = 0.80f;
    static const float kReleaseDuration = 1.50f;
    static const float kSpring = 90.f;
    static const float kDamping = 16.f;
    static const float kForceLimit = 300.f;
    static const float kMaxVelocity = 2.8f;
    static const float kMaxReferenceAge = 30.f;

    static const char* const kRequiredStalkerBones[] = {
        "bip01_pelvis", "bip01_spine", "bip01_spine1", "bip01_spine2",
        "bip01_neck", "bip01_head",
        "bip01_l_upperarm", "bip01_l_forearm",
        "bip01_r_upperarm", "bip01_r_forearm",
        "bip01_l_thigh", "bip01_l_calf",
        "bip01_r_thigh", "bip01_r_calf",
    };

    static float clamp_value(float value, float low, float high)
    {
        return value < low ? low : value > high ? high : value;
    }

    static bool finite_value(float value)
    {
        return value == value && value > -100000.f && value < 100000.f;
    }
}

CActiveRagdollController::CActiveRagdollController()
    : m_shell(nullptr), m_owner(nullptr), m_enabled(false), m_supported(false),
      m_phase(phase_unready), m_phase_age(0.f), m_reference_age(0.f),
      m_joints{
          joint_state("bip01_spine1"),
          joint_state("bip01_head"),
          joint_state("bip01_l_upperarm"),
          joint_state("bip01_l_forearm"),
          joint_state("bip01_r_upperarm"),
          joint_state("bip01_r_forearm"),
          joint_state("bip01_l_thigh"),
          joint_state("bip01_l_calf"),
          joint_state("bip01_r_thigh"),
          joint_state("bip01_r_calf")
      }
{
}

CActiveRagdollController::~CActiveRagdollController()
{
    detach();
}

void CActiveRagdollController::attach(CPhysicsShell* shell, CEntityAlive* owner)
{
    if (m_shell == shell && m_owner == owner)
        return;

    detach();
    m_shell = shell;
    m_owner = owner;
    m_balance_blocked=false;
    m_supported = is_stalker_profile();
    m_phase = m_supported ? phase_impact : phase_unready;
    m_phase_age = 0.f;
    m_reference_age = 0.f;
    if (m_supported)
        rebuild_joints();
}

void CActiveRagdollController::detach()
{
    m_balance.stop();
    Deactivate();
    for (joint_state& state : m_joints)
        release_joint(state);
    m_shell = nullptr;
    m_owner = nullptr;
    m_reference_age = 0.f;
    m_phase_age = 0.f;
    m_phase = phase_unready;
    m_supported = false;
    m_enabled = false;
}

void CActiveRagdollController::set_enabled(bool value)
{
    if (!value) m_balance.stop();
    if (value && painter_balance_test_enabled) {
        // A test rejection stays passive; never disguise the old scripted
        // controller as the learned model. No motor writes from both owners.
        Deactivate(); m_enabled=false;
        for (joint_state& state : m_joints) release_joint(state);
        if(!m_balance_blocked)m_balance.start(m_shell,m_owner);
        return;
    }
    m_enabled = value && m_shell != nullptr && m_supported;
    if (m_enabled)
    {
        // CharacterPhysicsSupport enables us after the corpse transform and
        // velocity are installed. ODE refreshes Euler angles on demand, so
        // capture the actual death pose immediately.
        capture_reference_pose();
        m_phase = phase_impact;
        m_phase_age = 0.f;
        Activate();
    }
    else
    {
        Deactivate();
        for (joint_state& state : m_joints)
            release_joint(state);
    }
}

bool CActiveRagdollController::is_stalker_profile() const
{
    // CharacterPhysicsSupport is shared by actors and monsters.  The explicit
    // runtime type gate is intentional: a compatible looking mutant must not
    // accidentally receive the human stalker motor map.
    if (!m_owner || !m_owner->cast_stalker() || !m_shell || !m_owner->Visual())
        return false;

    IKinematics* kinematics = smart_cast<IKinematics*>(m_owner->Visual());
    if (!kinematics)
        return false;

    for (const char* bone_name : kRequiredStalkerBones)
    {
        if (kinematics->LL_BoneID(bone_name) == BI_NONE)
            return false;
    }

    // The stalker shell normally contains six independent joints here.  Keep
    // the gate tolerant of cosmetic skeleton variants, but never start on a
    // shell with fewer than four usable motors.
    unsigned int usable_joints = 0;
    for (const joint_state& state : m_joints)
    {
        CPhysicsJoint* joint = m_shell->get_Joint(state.name);
        if (joint && joint->GetAxesNumber() > 0)
            ++usable_joints;
    }
    return usable_joints >= 4;
}

void CActiveRagdollController::rebuild_joints()
{
    for (joint_state& state : m_joints)
    {
        state.joint = nullptr;
        state.axes = 0;
        state.has_target = false;
        state.previous_valid = false;
        if (!m_shell)
            continue;

        state.joint = m_shell->get_Joint(state.name);
        if (!state.joint)
            continue;

        const u16 axes = state.joint->GetAxesNumber();
        state.axes = static_cast<unsigned char>(axes > 3 ? 3 : axes);
    }
}

void CActiveRagdollController::capture_reference_pose()
{
    if (!m_shell || !m_supported)
        return;

    rebuild_joints();
    for (joint_state& state : m_joints)
    {
        if (!state.joint || !state.joint->bActive || !state.axes)
            continue;

        state.has_target = true;
        state.previous_valid = false;
        for (unsigned char axis = 0; axis < state.axes; ++axis)
        {
            const float angle = state.joint->GetAxisAngle(axis);
            state.target[axis] = finite_value(angle) ? angle : 0.f;
            state.previous_angle[axis] = state.target[axis];
        }
    }
    m_reference_age = 0.f;
}

float CActiveRagdollController::joint_gain(const joint_state& state) const
{
    // Keep the torso and hips coherent while allowing hands, forearms and the
    // head to react naturally to impact.  These are multipliers on the motor
    // budget, not changes to the OGF joint limits.
    if (!state.name)
        return 0.5f;
    if (strstr(state.name, "spine") || strstr(state.name, "thigh") ||
        strstr(state.name, "calf"))
        return 0.36f;
    if (strstr(state.name, "head"))
        return 0.20f;
    return 0.16f;
}

bool CActiveRagdollController::bone_is_visible(const joint_state& state) const
{
    if (!m_owner || !m_owner->Visual())
        return true;

    IKinematics* kinematics = smart_cast<IKinematics*>(m_owner->Visual());
    if (!kinematics)
        return true;

    const u16 bone = kinematics->LL_BoneID(state.name);
    if (bone == BI_NONE)
        return false;
    return !!kinematics->LL_GetBoneVisible(bone);
}

void CActiveRagdollController::release_joint(joint_state& state)
{
    // CPhysicsJoint is owned by the shell.  Clear our pointer after disabling
    // it so detach/re-attach and NetRelcase cannot issue commands to a dead
    // joint.
    if (state.joint)
    {
        for (unsigned char axis = 0; axis < state.axes; ++axis)
            state.joint->SetForceAndVelocity(0.f, 0.f, axis);
    }
    state.joint = nullptr;
    state.axes = 0;
    state.has_target = false;
    state.previous_valid = false;
}

void CActiveRagdollController::PhTune(float step)
{
    if (!m_enabled || !m_supported || !m_shell || !m_shell->isFullActive())
        return;

    if (step <= 0.f || step > 1.f)
        return;

    m_phase_age += step;
    m_reference_age += step;
    if (m_reference_age > kMaxReferenceAge)
        capture_reference_pose();

    // CreateShell can attach us one frame before ODE has activated every
    // joint.  Defer the first reference capture instead of turning an
    // unavailable angle into a false zero pose.
    bool have_reference = false;
    for (const joint_state& state : m_joints)
    {
        if (state.joint && state.joint->bActive && state.has_target)
        {
            have_reference = true;
            break;
        }
    }
    if (!have_reference)
    {
        capture_reference_pose();
        return;
    }

    // The native shell owns the fall.  Motor authority fades through impact,
    // recovery and release so a failed catch turns into an ordinary ragdoll
    // instead of a body that is held in its death pose forever.
    if (m_phase == phase_impact && m_phase_age >= kImpactDuration)
        m_phase = phase_recover;
    if (m_phase == phase_recover && m_phase_age >= kImpactDuration + kRecoverDuration)
        m_phase = phase_release;
    if (m_phase == phase_release &&
        m_phase_age >= kImpactDuration + kRecoverDuration + kReleaseDuration)
    {
        set_enabled(false);
        return;
    }

    float phase_gain = 1.f;
    if (m_phase == phase_recover)
    {
        const float t = (m_phase_age - kImpactDuration) / kRecoverDuration;
        phase_gain = 0.82f - clamp_value(t, 0.f, 1.f) * 0.42f;
    }
    else if (m_phase == phase_release)
    {
        const float t = (m_phase_age - kImpactDuration - kRecoverDuration) /
                        kReleaseDuration;
        phase_gain = 0.40f - clamp_value(t, 0.f, 1.f) * 0.32f;
    }

    for (joint_state& state : m_joints)
    {
        if (!state.joint || !state.axes || !state.has_target)
            continue;

        if (!bone_is_visible(state))
        {
            release_joint(state);
            continue;
        }

        for (unsigned char axis = 0; axis < state.axes; ++axis)
        {
            const float angle = state.joint->GetAxisAngle(axis);
            // This ODE build does not implement dJointGetAMotorAngleRate.
            // Estimate angular rate from successive supported angle reads.
            float rate = 0.f;
            if (state.previous_valid && step > 0.f)
            {
                float delta = angle - state.previous_angle[axis];
                if (delta > 3.14159265f) delta -= 6.28318530f;
                if (delta < -3.14159265f) delta += 6.28318530f;
                rate = delta / step;
            }
            state.previous_angle[axis] = angle;
            if (!finite_value(angle) || !finite_value(rate))
            {
                state.joint->SetForceAndVelocity(0.f, 0.f, axis);
                continue;
            }

            const float error = clamp_value(state.target[axis] - angle, -3.14159265f, 3.14159265f);
            const float gain = phase_gain * joint_gain(state);
            const float desired_velocity = clamp_value(error * (2.2f + 0.7f * gain) -
                                                           rate * 0.45f,
                                                       -kMaxVelocity, kMaxVelocity);
            // ODE's FMax is a magnitude; direction is supplied by Vel.
            const float force = clamp_value(std::fabs(error * kSpring * gain -
                                                      rate * kDamping * gain),
                                            0.f, kForceLimit);
            state.joint->SetForceAndVelocity(force, desired_velocity, axis);
        }
        state.previous_valid = true;
    }
}

void CActiveRagdollController::PhDataUpdate(float step)
{
    // The force commands are installed in PhTune before IslandStep. The data
    // phase is deliberately empty so it never mutates a shell while ODE is
    // integrating it.
    (void)step;
}

void CActiveRagdollController::NetRelcase(CPhysicsShell* shell)
{
    if (shell == m_shell)
        detach();
}

void CActiveRagdollController::hit(const SHit& hit,bool fatal)
{
    m_balance.hit(hit);
    if(!m_owner||(!fatal&&m_owner->g_Alive()))return;
    auto*k=smart_cast<IKinematics*>(m_owner->Visual());
    if(!k||hit.bone()>=k->LL_BoneCount())return;
    const char*name=k->LL_BoneName_dbg(hit.bone());
    if(strstr(name,"head")||strstr(name,"neck")||strstr(name,"jaw")||hit.type()==ALife::eHitTypeExplosion){
        m_balance_blocked=true;m_balance.stop();
    }
}
