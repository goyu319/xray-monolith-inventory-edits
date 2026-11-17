#include "stdafx.h"
#include "PHIsland.h"
#include "physics.h"
#include "ph_valid_ode.h"

PHIslandStepCaptureCallback g_ph_island_step_capture = nullptr;
PHIslandStepCaptureCallback g_ph_island_step_policy = nullptr;

bool PHIslandContactIsSpecial(const dxJoint* joint)
{
	return joint && joint->vtable == &__dcontact_special_vtable;
}

void CPHIsland::Step(dReal step)
{
	if (!m_flags.is_active()) return;
	//dWorldStepFast1	(DWorld(),	fixed_step,	phIterations/*+Random.randI(0,phIterationCycle)*/);
	const bool exact = (m_flags.is_exact_integration_forced() && nj < max_joint_allowed_for_forced_exact_integration)
		|| (m_flags.is_exact_integration_prefeared() && nj < max_joint_allowed_for_exeact_integration);
    if (g_ph_island_step_policy) g_ph_island_step_policy(this, false, exact);
	if (g_ph_island_step_capture) g_ph_island_step_capture(this, false, exact);
	if (exact)
		dWorldStep(DWorld(), fixed_step);
	else
		dWorldQuickStep(DWorld(), fixed_step);
	if (g_ph_island_step_capture) g_ph_island_step_capture(this, true, exact);
	//dWorldStep(DWorld(),fixed_step);
}

void CPHIsland::Enable()
{
	if (!m_flags.is_active()) return;

	for (dxBody* body = DWorld()->firstbody; body; body = (dxBody *)body->next)body->flags &= ~dxBodyDisabled;
}

void CPHIsland::Repair()
{
	if (!m_flags.is_active()) return;
	dBodyID body;
	for (body = firstbody; body; body = (dxBody *)body->next)
	{
		if (!dV_valid(dBodyGetAngularVel(body)))
			dBodySetAngularVel(body, 0.f, 0.f, 0.f);
		if (!dV_valid(dBodyGetLinearVel(body)))
			dBodySetLinearVel(body, 0.f, 0.f, 0.f);
		if (!dV_valid(dBodyGetPosition(body)))
			dBodySetPosition(body, 0.f, 0.f, 0.f);
		if (!dQ_valid(dBodyGetQuaternion(body)))
		{
			dQuaternion q = {1.f, 0.f, 0.f, 0.f}; //dQSetIdentity(q);
			dBodySetQuaternion(body, q);
		}
	}
}
