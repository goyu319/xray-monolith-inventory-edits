#pragma once
// Batched AI queries: one call for what a squad's decision pass used to ask hundreds of times, with the rays on worker
// threads. Rays are static geometry only: the ray picker keeps its collider state per thread and the static model is
// read-only, while the dynamic object database is not safe off the calling thread (use level.ray_pick for objects).
//   level.painter_rays(list) -> {d1, d2, ...}
//     list[i] = {ax,ay,az, bx,by,bz}; d = distance along a->b to the first static hit, -1 when the line is clear to b,
//     -2 when the entry is not six numbers.
//   level.painter_cover_query(q) -> nearest-first array of the cover manager's own cover points (as level.get_nearby_covers)
//     q = {centre, radius (default 10), threats = {pos, ...} (up to 8), npc_id (his movement restrictions apply),
//          min_enemy_m (drop spots nearer to any threat), max_lo (keep a spot only if, toward some threat, its low or
//          high cover value is <= max_lo), max_n (default 24), heights (default {0.7, 1.1, 1.7})}
//     positions may be {x,y,z}, {x=,y=,z=} or vectors. Each entry: {x,y,z, vid, hi, lo (cover values toward threats[1],
//     interpolated between the four quadrant values as vertex_info gives them), b = {blocked per height: a ray from the
//     spot + h to the eye (y + 1.7) of threat t hits static geometry}, t (the nearest threat, 1-based), nb (a nav node on
//     that threat's side of the spot), dc, dt (flat distances to the centre and to threat t)}.

#include <algorithm>
#include "cover_manager.h"
#include "cover_point.h"
#include "ai/stalker/ai_stalker.h"
#include "stalker_movement_manager_smart_cover.h"
#include "restricted_object.h"

namespace painter_batch
{
inline bool lua_number(::luabind::object const& o, float& out)
{
	if (o.type() != LUA_TNUMBER)
		return false;
	auto const value = ::luabind::object_cast_nothrow<float>(o);
	if (!value)
		return false;
	out = *value;
	return _valid(out);
}

// {x,y,z}, {x=,y=,z=} or a vector
inline bool lua_vector(::luabind::object const& o, Fvector& out)
{
	if (o.type() == LUA_TTABLE)
	{
		if (lua_number(o[1], out.x) && lua_number(o[2], out.y) && lua_number(o[3], out.z))
			return true;
		return lua_number(o["x"], out.x) && lua_number(o["y"], out.y) && lua_number(o["z"], out.z);
	}
	if (o.type() == LUA_TUSERDATA)
	{
		auto const value = ::luabind::object_cast_nothrow<Fvector*>(o);
		if (value && *value)
		{
			out = **value;
			return _valid(out);
		}
	}
	return false;
}

// distance along a->b to the first static hit, -1 when clear (safe on any thread)
inline float static_hit(Fvector const& a, Fvector const& b)
{
	Fvector direction = Fvector().sub(b, a);
	float const length = direction.magnitude();
	if (length < EPS_L)
		return -1.f;
	direction.div(length);
	collide::rq_result result;
	if (!Level().ObjectSpace.RayPick(a, direction, length, collide::rqtStatic, result, nullptr))
		return -1.f;
	return result.range;
}

inline void run_rays(xr_vector<std::pair<Fvector, Fvector>> const& rays, xr_vector<float>& hits)
{
	hits.resize(rays.size());
	if (rays.size() < 24)
	{
		for (u32 i = 0; i < rays.size(); ++i)
			hits[i] = static_hit(rays[i].first, rays[i].second);
		return;
	}
	xr_parallel_for(0u, u32(rays.size()), [&](u32 i) { hits[i] = static_hit(rays[i].first, rays[i].second); });
}

inline ::luabind::object rays(::luabind::object const& list)
{
	lua_State* const L = ai().script_engine().lua();
	::luabind::object result = ::luabind::newtable(L);
	if (list.type() != LUA_TTABLE)
		return result;
	xr_vector<std::pair<Fvector, Fvector>> work;
	xr_vector<int> slot;
	for (int i = 1; i <= 4096; ++i)
	{
		::luabind::object const item = list[i];
		if (item.type() == LUA_TNIL)
			break;
		Fvector a, b;
		bool const ok = item.type() == LUA_TTABLE && lua_number(item[1], a.x) && lua_number(item[2], a.y) &&
			lua_number(item[3], a.z) && lua_number(item[4], b.x) && lua_number(item[5], b.y) && lua_number(item[6], b.z);
		slot.push_back(ok ? int(work.size()) : -1);
		if (ok)
			work.push_back(std::make_pair(a, b));
	}
	xr_vector<float> hits;
	run_rays(work, hits);
	for (u32 i = 0; i < slot.size(); ++i)
		result[i + 1] = slot[i] < 0 ? -2.f : hits[slot[i]];
	return result;
}

// the cover values toward (dx, dz): the four quadrant values (+x, +z, -x, -z, as level.vertex_info gives them)
// interpolated linearly by the heading, as the AI's own cover port does
inline void cover_toward(float const (&high)[4], float const (&low)[4], float dx, float dz, float& hi, float& lo)
{
	float a = atan2f(dz, dx);
	if (a < 0.f)
		a += PI_MUL_2;
	float const q = a / PI_DIV_2;
	int const i = int(floorf(q)) % 4;
	float const f = q - floorf(q);
	int const j = (i + 1) % 4;
	hi = high[i] * (1.f - f) + high[j] * f;
	lo = low[i] * (1.f - f) + low[j] * f;
}

inline ::luabind::object cover_query(::luabind::object const& q)
{
	lua_State* const L = ai().script_engine().lua();
	::luabind::object result = ::luabind::newtable(L);
	Fvector centre;
	if (q.type() != LUA_TTABLE || !lua_vector(q["centre"], centre))
		return result;
	float radius = 10.f, min_enemy = 0.f, max_lo = 1.f, max_n_value = 24.f, npc = -1.f;
	lua_number(q["radius"], radius);
	lua_number(q["min_enemy_m"], min_enemy);
	lua_number(q["max_lo"], max_lo);
	lua_number(q["max_n"], max_n_value);
	radius = clampr(radius, 1.f, 60.f);
	u32 const max_n = u32(clampr(max_n_value, 1.f, 256.f));
	xr_vector<Fvector> threats;
	::luabind::object const threat_list = q["threats"];
	if (threat_list.type() == LUA_TTABLE)
		for (int i = 1; i <= 8; ++i)
		{
			Fvector t;
			::luabind::object const item = threat_list[i];
			if (item.type() == LUA_TNIL)
				break;
			if (lua_vector(item, t))
				threats.push_back(t);
		}
	xr_vector<float> heights;
	::luabind::object const height_list = q["heights"];
	if (height_list.type() == LUA_TTABLE)
		for (int i = 1; i <= 8; ++i)
		{
			float h;
			if (!lua_number(height_list[i], h))
				break;
			heights.push_back(h);
		}
	if (heights.empty())
	{
		heights.push_back(.7f);
		heights.push_back(1.1f);
		heights.push_back(1.7f);
	}
	CRestrictedObject const* restrictions = nullptr;
	if (lua_number(q["npc_id"], npc) && npc >= 0.f && npc < 65535.f)
		if (CAI_Stalker* const stalker = smart_cast<CAI_Stalker*>(Level().Objects.net_Find(u16(npc))))
			restrictions = &stalker->movement().restrictions();

	CLevelGraph const& graph = ai().level_graph();
	Fvector const quadrant[4] = {{1.f, 0.f, 0.f}, {0.f, 0.f, 1.f}, {-1.f, 0.f, 0.f}, {0.f, 0.f, -1.f}};
	float quadrant_yaw[4];
	for (int k = 0; k < 4; ++k)
	{
		float y, p;
		quadrant[k].getHP(y, p);
		quadrant_yaw[k] = angle_normalize(y);
	}
	Fvector const slot_direction[4] = {{-1.f, 0.f, 0.f}, {0.f, 0.f, 1.f}, {1.f, 0.f, 0.f}, {0.f, 0.f, -1.f}}; // link slots

	struct Spot
	{
		CCoverPoint const* cover;
		float dc, dt, hi, lo;
		int t;
	};
	xr_vector<CCoverPoint*> covers;
	ai().cover_manager().covers().nearest(centre, radius, covers);
	xr_vector<Spot> kept;
	for (CCoverPoint* const cover : covers)
	{
		u32 const vid = cover->level_vertex_id();
		if (!graph.valid_vertex_id(vid) || (restrictions && !restrictions->accessible(vid)))
			continue;
		Fvector const& p = cover->position();
		float const dc = p.distance_to_xz(centre);
		if (dc > radius)
			continue;
		float high[4], low[4];
		for (int k = 0; k < 4; ++k)
		{
			high[k] = graph.high_cover_in_direction(quadrant_yaw[k], vid);
			low[k] = graph.low_cover_in_direction(quadrant_yaw[k], vid);
		}
		Spot spot{cover, dc, flt_max, 1.f, 1.f, -1};
		bool too_close = false, covered = threats.empty();
		for (u32 k = 0; k < threats.size(); ++k)
		{
			float const d = p.distance_to_xz(threats[k]);
			if (d < min_enemy)
				too_close = true;
			if (d < spot.dt)
			{
				spot.dt = d;
				spot.t = int(k);
			}
			float hi, lo;
			cover_toward(high, low, threats[k].x - p.x, threats[k].z - p.z, hi, lo);
			if (!k)
			{
				spot.hi = hi;
				spot.lo = lo;
			}
			covered = covered || lo <= max_lo || hi <= max_lo;
		}
		if (!too_close && covered)
			kept.push_back(spot);
	}
	std::sort(kept.begin(), kept.end(), [](Spot const& a, Spot const& b) { return a.dc < b.dc; });
	if (kept.size() > max_n)
		kept.resize(max_n);
	// line of fire: from the spot at each height to the nearest threat's eye
	xr_vector<std::pair<Fvector, Fvector>> work;
	for (Spot const& spot : kept)
		if (spot.t >= 0)
		{
			Fvector const& p = spot.cover->position();
			Fvector const eye = Fvector().set(threats[spot.t].x, threats[spot.t].y + 1.7f, threats[spot.t].z);
			for (float const h : heights)
				work.push_back(std::make_pair(Fvector().set(p.x, p.y + h, p.z), eye));
		}
	xr_vector<float> hits;
	run_rays(work, hits);
	u32 ray = 0;
	int count = 0;
	for (Spot const& spot : kept)
	{
		Fvector const& p = spot.cover->position();
		u32 const vid = spot.cover->level_vertex_id();
		::luabind::object entry = ::luabind::newtable(L);
		entry["x"] = p.x;
		entry["y"] = p.y;
		entry["z"] = p.z;
		entry["vid"] = vid;
		entry["hi"] = spot.hi;
		entry["lo"] = spot.lo;
		entry["dc"] = spot.dc;
		if (spot.t >= 0)
		{
			::luabind::object blocked = ::luabind::newtable(L);
			for (u32 h = 0; h < heights.size(); ++h)
				blocked[h + 1] = hits[ray++] >= 0.f;
			entry["b"] = blocked;
			entry["t"] = spot.t + 1;
			entry["dt"] = spot.dt;
			// a nav node on the threat's side: the link slot that points most toward it
			Fvector to_threat = Fvector().set(threats[spot.t].x - p.x, 0.f, threats[spot.t].z - p.z);
			int best = 0;
			for (int k = 1; k < 4; ++k)
				if (slot_direction[k].dotproduct(to_threat) > slot_direction[best].dotproduct(to_threat))
					best = k;
			entry["nb"] = graph.valid_vertex_id(graph.vertex(vid)->link(u8(best)));
		}
		result[++count] = entry;
	}
	return result;
}
} // namespace painter_batch

inline ::luabind::object painter_rays(::luabind::object const& list)
{
	return painter_batch::rays(list);
}

inline ::luabind::object painter_cover_query(::luabind::object const& query)
{
	return painter_batch::cover_query(query);
}
