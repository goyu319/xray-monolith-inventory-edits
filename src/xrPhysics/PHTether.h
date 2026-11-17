#pragma once
#include "phupdateobject.h"
#include "phisland.h"
#include "../3rd party/ode/include/ode/common.h"
#include "IPHTether.h"
class CPhysicsShell;
class CPhysicsElement;
// See IPHTether.h. Modelled on CPHCapture: the joint lives in its own island, which both shells' islands are
// merged into before every step (PhTune) and unmerged after it (PhDataUpdate).
class CPHTether : public CPHUpdateObject, public IPHTether
{
public:
	CPHTether(CPhysicsShell* a, u16 bone_a, CPhysicsShell* b, u16 bone_b, const Fvector& anchor, float break_force, u32 life_ms,
              bool exact_elements = false);
	virtual ~CPHTether();
	virtual bool Alive() const { return m_joint != NULL; }
	virtual void Release();
	void ShellGone(CPhysicsShell* s);
protected:
	CPhysicsShell* m_a;
	CPhysicsShell* m_b;
	CPhysicsElement* m_ea;
	CPhysicsElement* m_eb;
	dJointID m_joint;
	dJointFeedback m_feedback;
	CPHIsland m_island;
	float m_break_force;
	u32 m_born_at;
	u32 m_life_ms;
	virtual void PhDataUpdate(dReal step);
	virtual void PhTune(dReal step);
	virtual void NetRelcase(CPhysicsShell* s);
};
// Called by CPHShell before it is deactivated or destroyed: every tether on that shell lets go first.
void phtether_shell_gone(CPhysicsShell* s);
