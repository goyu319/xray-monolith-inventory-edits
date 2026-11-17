#pragma once
#include "painter_balance_test.h"

#include "../xrphysics/PHUpdateObject.h"

class CEntityAlive;
class CPhysicsShell;
class CPhysicsJoint;

// Fixed-step native controller for Painter's stalker-only active-ragdoll port.
// CharacterPhysicsSupport still applies the runtime stalker profile gate, so
// unsupported visuals remain on the stock ragdoll path.
class CActiveRagdollController final : public CPHUpdateObject
{
public:
    CActiveRagdollController();
    ~CActiveRagdollController() override;

    void attach(CPhysicsShell* shell, CEntityAlive* owner);
    void detach();
    void set_enabled(bool value);
    void hit(const SHit& hit, bool fatal=false);
    bool enabled() const { return m_enabled; }
    bool supported() const { return m_supported; }

    // Capture the current joint angles as the controller's reference pose.
    // The native limits are never changed by this controller.
    void capture_reference_pose();

    void PhDataUpdate(float step) override;
    void PhTune(float step) override;
    void NetRelcase(CPhysicsShell* shell) override;

private:
    enum controller_phase
    {
        phase_unready,
        phase_impact,
        phase_recover,
        phase_release,
    };

    struct joint_state
    {
        const char* name;
        CPhysicsJoint* joint;
        float target[3];
        float previous_angle[3];
        unsigned char axes;
        bool has_target;
        bool previous_valid;

        joint_state(const char* n)
            : name(n), joint(nullptr), axes(0), has_target(false), previous_valid(false)
        {
            target[0] = target[1] = target[2] = 0.f;
            previous_angle[0] = previous_angle[1] = previous_angle[2] = 0.f;
        }
    };

    PainterBalanceTest m_balance;
    bool m_balance_blocked=false;
    CPhysicsShell* m_shell;
    CEntityAlive* m_owner;
    bool m_enabled;
    bool m_supported;
    controller_phase m_phase;
    float m_phase_age;
    float m_reference_age;
    joint_state m_joints[10];

    bool is_stalker_profile() const;
    void rebuild_joints();
    void release_joint(joint_state& state);
    bool bone_is_visible(const joint_state& state) const;
    float joint_gain(const joint_state& state) const;
};
