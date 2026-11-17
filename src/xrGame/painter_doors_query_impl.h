#pragma once
// level.painter_doors_near(pos, radius): the physical doors (ph_door) the engine knows near a point, nearest first, so a
// room-clearing AI can match them to the building scan's doorways and treat a closed one as a wall until it is opened:
//   { {id, name, hinge, centre (the leaf's centre when closed), normal (flat, across the closed leaf), half_width,
//      open (0..1, measured from the leaf's current swing), state ("open" | "closed" | "opening" | "closing"),
//      locked (closed for NPCs: they wait at it), held (stalkers walking through are holding it)}, ... }
// Stalkers open a door their path crosses by themselves (doors::actor asks it to open ~1.4 s ahead and the ph_door
// script swings it); a locked one makes them wait in front of it.

#include <algorithm>
#include "doors_manager.h"
#include "doors_door.h"
#include "PhysicObject.h"
#include "../Include/xrRender/Kinematics.h"

namespace painter_doors
{
inline ::luabind::object near_doors(const Fvector& position, float radius)
{
	lua_State* const L = ai().script_engine().lua();
	::luabind::object result = ::luabind::newtable(L);
	if (!_valid(position))
		return result;
	radius = clampr(_valid(radius) ? radius : 30.f, 1.f, 100.f);
	doors::doors_type found;
	ai().doors().nearest(position, radius, found);
	std::sort(found.begin(), found.end(), [&](doors::door* a, doors::door* b) {
		return a->position().distance_to_sqr(position) < b->position().distance_to_sqr(position);
	});
	int count = 0;
	for (doors::door* const door : found)
	{
		CPhysicObject& object = door->object();
		Fmatrix const& xform = door->get_matrix();
		Fvector closed, open;
		xform.transform_dir(closed, door->get_vector(doors::door_state_closed));
		xform.transform_dir(open, door->get_vector(doors::door_state_open));
		closed.y = 0.f;
		open.y = 0.f;
		if (closed.square_magnitude() < EPS_S)
			continue;
		closed.normalize();
		bool const has_open = open.square_magnitude() > EPS_S;
		if (has_open)
			open.normalize();
		// the leaf: the "door" bone's box gives the real width, the hinge (bone origin) and the current swing
		float width = 1.1f; // the engine's door length when the leaf cannot be read
		float mid_height = 1.f;
		Fvector hinge = xform.c;
		float swing = door->state() == doors::door_state_open ? 1.f : 0.f;
		IKinematics* const kinematics = object.Visual() ? object.Visual()->dcast_PKinematics() : nullptr;
		u16 const bone = kinematics ? kinematics->LL_BoneID("door") : BI_NONE;
		if (bone != BI_NONE && kinematics->LL_GetData(bone).shape.type == SBoneShape::stBox)
		{
			SBoneShape const& shape = kinematics->LL_GetData(bone).shape;
			Fmatrix const leaf = Fmatrix().mul_43(xform, kinematics->LL_GetTransform(bone));
			Fvector leaf_centre;
			leaf.transform_tiny(leaf_centre, shape.box.m_translate);
			hinge = leaf.c;
			width = 2.f * shape.box.m_halfsize.x;
			mid_height = leaf_centre.y - hinge.y;
			Fvector leaf_dir = Fvector().sub(leaf_centre, hinge);
			leaf_dir.y = 0.f;
			if (leaf_dir.square_magnitude() > EPS_S && has_open)
			{
				leaf_dir.normalize();
				float const full = acosf(clampr(open.dotproduct(closed), -1.f, 1.f));
				float const now = acosf(clampr(leaf_dir.dotproduct(closed), -1.f, 1.f));
				if (full > EPS_L)
					swing = clampr(now / full, 0.f, 1.f);
			}
		}
		Fvector centre = Fvector().mad(hinge, closed, .5f * width);
		centre.y = hinge.y + mid_height;
		LPCSTR state;
		if (door->state() == doors::door_state_open)
			state = door->target_state() == doors::door_state_open ? "open" : "closing";
		else
			state = door->target_state() == doors::door_state_closed ? "closed" : "opening";
		::luabind::object entry = ::luabind::newtable(L);
		entry["id"] = object.ID();
		entry["name"] = object.cName().c_str();
		entry["hinge"] = hinge;
		entry["centre"] = centre;
		entry["normal"] = Fvector().set(closed.z, 0.f, -closed.x);
		entry["half_width"] = .5f * width;
		entry["open"] = swing;
		entry["state"] = state;
		entry["locked"] = ai().doors().is_door_locked(door);
		entry["held"] = ai().doors().is_door_blocked(door);
		result[++count] = entry;
	}
	return result;
}
} // namespace painter_doors

inline ::luabind::object painter_doors_near(const Fvector& position, float radius)
{
	return painter_doors::near_doors(position, radius);
}
