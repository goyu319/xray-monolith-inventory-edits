#pragma once
// A tissue tether: a breakable ball joint between an element of one physics shell and an element of another,
// anchored at a world point (a limb torn nearly free hangs from its stump by it). It breaks when the joint
// force exceeds break_force, after life_ms, or when either shell goes away; the pieces then part.
#include "xrPhysics.h"
class CPhysicsShell;
class IPHTether
{
public:
	virtual bool Alive() const = 0;
	virtual void Release() = 0;
protected:
	virtual ~IPHTether() = 0
	{
	}
};
XRPHYSICS_API IPHTether* phtether_create(CPhysicsShell* a, u16 bone_a, CPhysicsShell* b, u16 bone_b, const Fvector& anchor,
                                          float break_force, u32 life_ms);
// Attachment-critical callers require these exact physical elements. Missing
// bones fail instead of silently attaching to element0 of another body part.
XRPHYSICS_API IPHTether* phtether_create_exact(CPhysicsShell* a, u16 bone_a, CPhysicsShell* b, u16 bone_b, const Fvector& anchor,
                                            float break_force, u32 life_ms);
XRPHYSICS_API void phtether_destroy(IPHTether*& t);
