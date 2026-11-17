#pragma once
#include "../xrphysics/iphysics_scripted.h"
#include "../xrphysics/physicsshell.h"
#include "script_export_space.h"
#include <cmath>

class CPhysicsElement;

class cphysics_element_scripted :
	public cphysics_game_scripted<CPhysicsElement>
{
public:
	cphysics_element_scripted(CPhysicsElement* imp): cphysics_game_scripted<CPhysicsElement>(imp)
	{
	}

	void applyForce(float x, float y, float z) { physics_impl().applyForce(x, y, z); }

	// demonized: apply_torque
	void applyTorque(float x, float y, float z) { physics_impl().applyTorque(x, y, z); }

	bool isBreakable() { return physics_impl().isBreakable(); }
	void get_LinearVel(Fvector& velocity) const { physics_impl().get_LinearVel(velocity); }
	void get_AngularVel(Fvector& velocity) const { physics_impl().get_AngularVel(velocity); }


	// Exact ODE COM; rendered bone origin includes a different local frame.
	bool get_mass_center(Fvector& result) const
	{
		if (!physics_impl().isFullActive()) return false;
		result.set(physics_impl().mass_Center());
		return true;
	}
	// Read-only projected collider support, relative to the requested world-space bone pivot.
	// A negative result means unavailable; Lua retains its legacy target on older executables.
	float get_contact_clearance(const Fvector& axis, const Fvector& pivot) const
	{
		if (!physics_impl().isFullActive() || !_valid(axis) || !_valid(pivot)) return -1.f;
		const float m2 = axis.square_magnitude();
		if (!std::isfinite(m2) || m2 <= 1e-12f) return -1.f;
		Fvector normal; normal.mul(axis, 1.f / std::sqrt(m2));
		const float center = normal.dotproduct(pivot);
		if (!std::isfinite(center)) return -1.f;
		float lo=0.f, hi=0.f;
		physics_impl().get_Extensions(normal, center, lo, hi);
		if (!std::isfinite(lo) || !std::isfinite(hi) || lo > hi) return -1.f;
		return _max(0.f, -lo);
	}
	float getMass() { return physics_impl().getMass(); }
	float getDensity() { return physics_impl().getDensity(); }
	float getVolume() { return physics_impl().getVolume(); }

	void Fix() { physics_impl().Fix(); }
	void ReleaseFixed() { physics_impl().ReleaseFixed(); }
	bool isFixed() { return physics_impl().isFixed(); }
	void GetGlobalTransformDynamic(Fmatrix* m) const { physics_impl().GetGlobalTransformDynamic(m); }

DECLARE_SCRIPT_REGISTER_FUNCTION
};
