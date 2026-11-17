#pragma once

#include <array>
#include <map>
#include <string>
#include <unordered_map>
#include "Level.h"
#include "level_graph.h"

namespace painter_building
{
float const roof_range = 12.f;        // a roof or ceiling over the node within this height
int const strong_walls = 6;           // roofless mode: chest rays (of 8) that hit walls
int const weak_walls = 4;
int const slope_walls = 2;            // a stair node needs only 2 walls
float const wall_range = 15.f;        // roofless mode
float const roofed_wall_range = 10.f; // roofed mode: walled-in nodes next to the roofed space
int const weak_reach = 4;             // weak nodes count only within this many links of a strong one
u32 const seed_budget = 400;
u32 const max_building = 4000;
float const jamb_range = 1.6f;
float const jamb_height = 1.1f;
float const jamb_min = .5f;
float const cut_low = 1.1f;  // a door jamb is a wall at chest and head height; cover is not that tall
float const cut_high = 1.9f;
int const door_end_steps = 3; // the narrow part of a doorway ends within 3 x 0.35 m along the link
int const lateral_k = 4;
u32 const min_room_nodes = 4;
float const floor_gap = 2.f;
u32 const landing_max = 150;
float const window_far = 4.6f;
float const window_join = 2.2f;
int const start_search = 3;
int const across_slot[4][2] = {{1, 3}, {0, 2}, {1, 3}, {0, 2}}; // link slots: 0 -x, 1 +z, 2 +x, 3 -z
int const reverse_slot[4] = {2, 3, 0, 1};
Fvector const link_dir[4] = {{-1.f, 0.f, 0.f}, {0.f, 0.f, 1.f}, {1.f, 0.f, 0.f}, {0.f, 0.f, -1.f}};

inline bool static_ray(const Fvector& start, const Fvector& direction, float range, float& hit)
{
	collide::rq_result result;
	if (!Level().ObjectSpace.RayPick(start, direction, range, collide::rqtStatic, result, nullptr))
		return false;
	hit = result.range;
	return true;
}

// Walls left and right of the travel direction d at height h over p, together at least jamb_min apart.
inline bool narrow_at(const Fvector& p, const Fvector& d, float h, float* width = nullptr)
{
	Fvector const across = Fvector().set(-d.z, 0.f, d.x);
	Fvector const chest = Fvector().set(p.x, p.y + h, p.z);
	float left, right;
	if (!static_ray(chest, Fvector().set(-across.x, 0.f, -across.z), jamb_range, left) ||
		!static_ray(chest, across, jamb_range, right) || left + right < jamb_min)
		return false;
	if (width)
		*width = left + right;
	return true;
}

// Wall jambs either side of an opening at chest height. The check slides along the passage (+-1 m) and keeps the
// narrowest place with walls on both sides within 1.6 m: that is the door plane. False: no walls on both sides anywhere.
inline bool jambs(const Fvector& centre, const Fvector& normal, float floor_y, Fvector& door_centre, float& half_width)
{
	Fvector const across = Fvector().set(-normal.z, 0.f, normal.x);
	Fvector const back = Fvector().set(-across.x, 0.f, -across.z);
	float best = flt_max;
	for (int step = -4; step <= 4; ++step)
	{
		Fvector const chest = Fvector().set(centre.x + normal.x * .25f * float(step), floor_y + jamb_height, centre.z + normal.z * .25f * float(step));
		float left, right;
		if (!static_ray(chest, back, jamb_range, left) || !static_ray(chest, across, jamb_range, right) || left + right < jamb_min || left + right >= best)
			continue;
		best = left + right;
		half_width = .5f * best;
		door_centre = Fvector().mad(chest, across, .5f * (right - left));
		door_centre.y = centre.y;
	}
	return best < flt_max;
}

// Walking from v across the passage (slot j) for up to k nodes: a node blocked in the passage direction (no link in
// slot i), or the walk itself blocked (a wall beside the passage).
inline bool lateral_blocked(CLevelGraph const& graph, u32 v, int i, int j, int k)
{
	u32 c = v;
	for (int step = 0; step <= k; ++step)
	{
		if (step)
		{
			c = graph.vertex(c)->link(u8(j));
			if (!graph.valid_vertex_id(c))
				return true;
		}
		if (!graph.valid_vertex_id(graph.vertex(c)->link(u8(i))))
			return true;
	}
	return false;
}

// The link from node va in slot i passes through a doorway.
inline bool door_link(CLevelGraph const& graph, u32 va, int i)
{
	u32 const vb = graph.vertex(va)->link(u8(i));
	for (int const j : across_slot[i])
		if (!lateral_blocked(graph, va, i, j, lateral_k) && !lateral_blocked(graph, vb, reverse_slot[i], j, lateral_k))
			return false; // open beside the passage on this side: not a gap in a wall line
	Fvector const a = graph.vertex_position(va), b = graph.vertex_position(vb);
	Fvector d = Fvector().set(b.x - a.x, 0.f, b.z - a.z);
	float const m = d.magnitude();
	if (m < 1e-6f)
		return false;
	d.div(m);
	// walls at both heights somewhere along the link (5 samples: a thin wall between two node rows is not missed)
	float best = flt_max;
	Fvector at{};
	for (int t = 0; t <= 4; ++t)
	{
		Fvector const p = Fvector().lerp(a, b, .25f * float(t));
		float width;
		if (!narrow_at(p, d, cut_low, &width) || width >= best || !narrow_at(p, d, cut_high))
			continue;
		best = width;
		at = p;
	}
	if (best == flt_max)
		return false;
	// the narrow part ends within ~1 m on one side, before any wall ahead: a gap in a wall, not a corridor
	float const rise = (b.y - a.y) / m;
	for (int side = 0; side < 2; ++side)
	{
		float const sign = side ? -1.f : 1.f;
		Fvector const dd = Fvector().set(d.x * sign, 0.f, d.z * sign);
		float ahead;
		if (!static_ray(Fvector().set(at.x, at.y + cut_high, at.z), dd, .35f * float(door_end_steps) + .1f, ahead))
			ahead = flt_max;
		for (int k = 1; k <= door_end_steps; ++k)
		{
			float const s = .35f * float(k);
			if (s >= ahead - .1f)
				break; // a wall ahead: the passage is closed this way
			if (!narrow_at(Fvector().set(at.x + dd.x * s, at.y + rise * s * sign, at.z + dd.z * s), d, cut_high))
				return true;
		}
	}
	return false;
}

// The node in the position's own cell, else the nearest node in the cells around it (no full-level search).
inline u32 start_node(CLevelGraph const& graph, const Fvector& position)
{
	if (graph.valid_vertex_position(position))
	{
		u32 const v = graph.vertex_id(position);
		if (graph.valid_vertex_id(v))
			return v;
	}
	float const cell = graph.header().cell_size();
	u32 best = u32(-1);
	float best_distance = flt_max;
	for (int dx = -start_search; dx <= start_search; ++dx)
		for (int dz = -start_search; dz <= start_search; ++dz)
		{
			Fvector const p = Fvector().set(position.x + float(dx) * cell, position.y, position.z + float(dz) * cell);
			if (!graph.valid_vertex_position(p))
				continue;
			u32 const c = graph.vertex_id(p);
			if (!graph.valid_vertex_id(c))
				continue;
			float const distance = graph.vertex_position(c).distance_to_sqr(position);
			if (distance < best_distance)
			{
				best_distance = distance;
				best = c;
			}
		}
	return best;
}

// The node in the cell of p (the nearest one under it, else over it), or u32(-1).
inline u32 cell_node(CLevelGraph const& graph, const Fvector& p)
{
	return graph.valid_vertex_position(p) ? graph.vertex_id(p) : u32(-1);
}

struct SOpening
{
	int room_a = -1, room_b = -1; // room_b -1 = outside
	bool window = false;
	bool stair = false;
	Fvector direction{}; // windows: the wall's outward direction
	float y0 = 0.f, y1 = 0.f;
	u32 key = u32(-1);
	xr_vector<std::pair<Fvector, Fvector>> sides; // per link: point on room_a's side, point on room_b's side
};

inline Fvector side_middle(const std::pair<Fvector, Fvector>& s)
{
	return Fvector().add(s.first, s.second).mul(.5f);
}

// Centre, flat normal (room_a -> room_b) and floor height of an opening; false when it has no direction.
inline bool opening_frame(const SOpening& o, Fvector& centre, Fvector& normal, float& floor_y)
{
	centre.set(0.f, 0.f, 0.f);
	normal.set(0.f, 0.f, 0.f);
	floor_y = flt_max;
	for (auto const& s : o.sides)
	{
		centre.add(side_middle(s));
		normal.add(Fvector().sub(s.second, s.first));
		floor_y = _min(floor_y, _min(s.first.y, s.second.y));
	}
	centre.div(float(o.sides.size()));
	normal.y = 0.f;
	if (normal.square_magnitude() < EPS_S) // sides in opposite directions (a node between two of the other room): the first side's
		normal.set(o.sides[0].second.x - o.sides[0].first.x, 0.f, o.sides[0].second.z - o.sides[0].first.z);
	if (normal.square_magnitude() < EPS_S)
		return false;
	normal.normalize();
	return true;
}

inline ::luabind::object scan(const Fvector& position, float radius)
{
	lua_State* const L = ai().script_engine().lua();
	CLevelGraph const& graph = ai().level_graph();
	radius = clampr(_valid(radius) ? radius : 30.f, 3.f, 60.f);
	if (!_valid(position))
		return ::luabind::object();
	u32 const start = start_node(graph, position);
	if (!graph.valid_vertex_id(start))
		return ::luabind::object();
	float const cell = graph.header().cell_size();
	auto in_range = [&](u32 v) {
		Fvector const p = graph.vertex_position(v);
		return _sqr(p.x - position.x) + _sqr(p.z - position.z) <= radius * radius && _abs(p.y - position.y) <= radius;
	};

	// 1. node tests, cached: roof, stair (a link with a 20 cm+ step), walls in 8 directions at chest height
	std::unordered_map<u32, u8> roofs, slopes;
	std::unordered_map<u32, u8> walls_cache[2];
	Fvector around[8];
	for (int a = 0; a < 8; ++a)
		around[a].set(_cos(float(a) * PI_DIV_4), 0.f, _sin(float(a) * PI_DIV_4));
	auto roofed = [&](u32 v) -> bool {
		auto const it = roofs.find(v);
		if (it != roofs.end())
			return it->second != 0;
		Fvector const p = graph.vertex_position(v);
		float hit;
		bool const r = static_ray(Fvector().set(p.x, p.y + .25f, p.z), Fvector().set(0.f, 1.f, 0.f), roof_range, hit);
		roofs[v] = r;
		return r;
	};
	auto sloped = [&](u32 v) -> bool {
		auto const it = slopes.find(v);
		if (it != slopes.end())
			return it->second != 0;
		Fvector const p = graph.vertex_position(v);
		bool s = false;
		for (u8 i = 0; i < 4 && !s; ++i)
		{
			u32 const n = graph.vertex(v)->link(i);
			s = graph.valid_vertex_id(n) && _abs(graph.vertex_position(n).y - p.y) >= .2f;
		}
		slopes[v] = s;
		return s;
	};
	// chest-height rays that hit a wall within range; stops once `enough` hits are certain or impossible
	auto walls = [&](u32 v, int which, int enough) -> int {
		auto const it = walls_cache[which].find(v);
		if (it != walls_cache[which].end())
			return it->second;
		float const range = which ? roofed_wall_range : wall_range;
		Fvector const p = graph.vertex_position(v);
		int w = 0;
		for (int a = 0; a < 8; ++a)
		{
			if (w >= enough || w + (8 - a) < _min(enough, slope_walls))
				break;
			float hit;
			if (static_ray(Fvector().set(p.x, p.y + 1.3f, p.z), around[a], range, hit))
				++w;
		}
		walls_cache[which][v] = u8(w);
		return w;
	};

	// 2. mode and seed: roofed when the start has a roof over it, roofless when it is walled in; a start outdoors takes
	//    the nearest roofed node by links, else the nearest walled-in one
	u32 seed = u32(-1);
	bool roofed_mode = true;
	if (roofed(start))
		seed = start;
	else if (walls(start, 0, strong_walls) >= strong_walls)
	{
		seed = start;
		roofed_mode = false;
	}
	else
		for (int pass = 0; pass < 2 && seed == u32(-1); ++pass)
		{
			std::unordered_map<u32, u8> seen;
			xr_deque<u32> wave;
			seen[start] = 1;
			wave.push_back(start);
			while (!wave.empty() && seen.size() < seed_budget)
			{
				u32 const v = wave.front();
				wave.pop_front();
				if (pass == 0 ? roofed(v) : walls(v, 0, strong_walls) >= strong_walls)
				{
					seed = v;
					roofed_mode = pass == 0;
					break;
				}
				for (u8 i = 0; i < 4; ++i)
				{
					u32 const n = graph.vertex(v)->link(i);
					if (graph.valid_vertex_id(n) && !seen.count(n) && in_range(n))
					{
						seen[n] = 1;
						wave.push_back(n);
					}
				}
			}
		}
	if (seed == u32(-1))
		return ::luabind::object(); // no building here
	// 2 strong, 1 weak (counts only within weak_reach links of a strong node), 0 out
	auto classify = [&](u32 v) -> int {
		if (roofed(v))
			return 2;
		int const need = sloped(v) ? slope_walls : weak_walls;
		if (roofed_mode)
			return walls(v, 1, need) >= need ? 1 : 0;
		int const w = walls(v, 0, strong_walls);
		return w >= strong_walls ? 2 : (w >= need ? 1 : 0);
	};

	// 3. the building: strong nodes, and weak ones within weak_reach links of a strong one. In roofless structures stair
	//    runs above low walls join the floors at no cost; in roofed buildings the stairs are under the roof, and a free
	//    run would carry the flood over open hillsides.
	std::unordered_map<u32, int> reach;
	xr_vector<u32> building;
	xr_vector<std::pair<u32, u8>> edge_links; // links to indoor nodes beyond the radius or the size cap
	bool capped = false;
	reach[seed] = 0;
	building.push_back(seed);
	{
		xr_deque<u32> work;
		work.push_back(seed);
		while (!work.empty())
		{
			u32 const v = work.front();
			work.pop_front();
			int const dv = reach[v];
			for (u8 slot = 0; slot < 4; ++slot)
			{
				u32 const n = graph.vertex(v)->link(slot);
				if (!graph.valid_vertex_id(n))
					continue;
				bool const known = reach.count(n) != 0;
				bool const inside = in_range(n);
				if (!inside || (!known && building.size() >= max_building))
				{
					if (!known && classify(n))
					{
						edge_links.push_back(std::make_pair(v, slot));
						capped = capped || inside;
					}
					continue;
				}
				int const indoor = classify(n);
				if (!indoor)
					continue;
				int const nd = indoor == 2 ? 0 : dv + (sloped(n) ? (roofed_mode ? 1 : 0) : 1);
				if (nd > weak_reach)
					continue;
				if (known && reach[n] <= nd)
					continue;
				if (!known)
					building.push_back(n);
				reach[n] = nd;
				if (nd <= dv)
					work.push_front(n);
				else
					work.push_back(n);
			}
		}
	}
	u32 const count = u32(building.size());
	std::unordered_map<u32, int> local;
	for (u32 b = 0; b < count; ++b)
		local[building[b]] = int(b);
	xr_vector<std::array<int, 4>> neighbour(count);
	xr_vector<Fvector> node_position(count);
	xr_vector<u8> strong(count), slope(count), edge(count, 0);
	for (u32 b = 0; b < count; ++b)
	{
		node_position[b] = graph.vertex_position(building[b]);
		strong[b] = classify(building[b]) == 2;
		slope[b] = sloped(building[b]);
		for (u8 i = 0; i < 4; ++i)
		{
			u32 const n = graph.vertex(building[b])->link(i);
			auto const it = graph.valid_vertex_id(n) ? local.find(n) : local.end();
			neighbour[b][i] = it == local.end() ? -1 : it->second;
		}
	}
	for (auto const& e : edge_links)
		edge[local[e.first]] |= u8(1 << e.second);

	// 4. rooms: cut the door links (and the stair runs between floors); the parts are rooms
	xr_vector<u8> cut(count, 0), stair_cut(count, 0);
	auto cut_link = [&](u32 b, int i, bool stair) {
		int const n = neighbour[b][i];
		cut[b] |= u8(1 << i);
		if (stair)
			stair_cut[b] |= u8(1 << i);
		for (int j = 0; j < 4; ++j)
			if (neighbour[n][j] == int(b))
			{
				cut[n] |= u8(1 << j);
				if (stair)
					stair_cut[n] |= u8(1 << j);
			}
	};
	for (u32 b = 0; b < count; ++b)
		for (int i = 0; i < 4; ++i)
			if (neighbour[b][i] > int(b) && door_link(graph, building[b], i))
				cut_link(b, i, false);
	{
		xr_vector<int> run_of(count, -1);
		int runs = 0;
		for (u32 b = 0; b < count; ++b)
		{
			if (!slope[b] || run_of[b] >= 0)
				continue;
			xr_vector<u32> group;
			group.push_back(b);
			run_of[b] = runs;
			for (u32 q = 0; q < group.size(); ++q)
				for (int i = 0; i < 4; ++i)
				{
					int const n = neighbour[group[q]][i];
					if (n >= 0 && slope[n] && run_of[n] < 0 && !(cut[group[q]] & (1 << i)))
					{
						run_of[n] = runs;
						group.push_back(u32(n));
					}
				}
			++runs;
			float lo = flt_max, hi = -flt_max;
			xr_vector<std::pair<u32, int>> ends;
			for (u32 const m : group)
				for (int i = 0; i < 4; ++i)
				{
					int const n = neighbour[m][i];
					if (n >= 0 && !slope[n] && !(cut[m] & (1 << i)))
					{
						ends.push_back(std::make_pair(m, i));
						lo = _min(lo, node_position[n].y);
						hi = _max(hi, node_position[n].y);
					}
				}
			if (!ends.empty() && hi - lo >= floor_gap)
				for (auto const& e : ends)
					cut_link(e.first, e.second, true);
		}
	}
	xr_vector<int> part(count, -1);
	int parts = 0;
	for (u32 b = 0; b < count; ++b)
	{
		if (part[b] >= 0)
			continue;
		xr_vector<u32> group;
		group.push_back(b);
		part[b] = parts;
		for (u32 q = 0; q < group.size(); ++q)
			for (int i = 0; i < 4; ++i)
			{
				int const n = neighbour[group[q]][i];
				if (n >= 0 && part[n] < 0 && !(cut[group[q]] & (1 << i)))
				{
					part[n] = parts;
					group.push_back(u32(n));
				}
			}
		++parts;
	}
	xr_vector<int> parent(parts);
	for (int r = 0; r < parts; ++r)
		parent[r] = r;
	auto find = [&](int r) {
		while (parent[r] != r)
			r = parent[r] = parent[parent[r]];
		return r;
	};
	// groups by root, in the order of their first node; lowest level vertex id per root
	xr_vector<int> roots;
	xr_vector<xr_vector<u32>> members(parts);
	xr_vector<u32> lowest(parts, u32(-1));
	auto regroup = [&]() {
		roots.clear();
		for (auto& m : members)
			m.clear();
		for (u32 b = 0; b < count; ++b)
		{
			int const r = find(part[b]);
			if (members[r].empty())
				roots.push_back(r);
			members[r].push_back(b);
		}
		for (int const r : roots)
		{
			lowest[r] = u32(-1);
			for (u32 const b : members[r])
				lowest[r] = _min(lowest[r], building[b]);
		}
	};
	// door links from a root to every other root
	auto contacts = [&](int r) {
		std::map<int, int> out;
		for (u32 const b : members[r])
			for (int i = 0; i < 4; ++i)
			{
				int const n = neighbour[b][i];
				if (n >= 0 && (cut[b] & (1 << i)) && find(part[n]) != r)
					++out[find(part[n])];
			}
		return out;
	};
	// small parts (door thresholds, a node in a thick wall) join the part they share most door links with; order and
	// ties by the lowest level vertex id, so the result does not depend on where the scan started
	for (;;)
	{
		regroup();
		xr_vector<int> tiny;
		for (int const r : roots)
			if (members[r].size() < min_room_nodes)
				tiny.push_back(r);
		std::sort(tiny.begin(), tiny.end(), [&](int x, int y) {
			return members[x].size() != members[y].size() ? members[x].size() < members[y].size() : lowest[x] < lowest[y];
		});
		bool joined = false;
		for (int const r : tiny)
		{
			auto const c = contacts(r);
			if (c.empty())
				continue;
			int best = -1;
			for (auto const& t : c)
				if (best < 0 || t.second > c.at(best) || (t.second == c.at(best) && lowest[t.first] < lowest[best]))
					best = t.first;
			parent[r] = best;
			joined = true;
			break;
		}
		if (!joined)
			break;
	}
	// adjoining stair parts are one stair (a stairwell whose walls end part of the way up); a small flat part that only
	// touches stair parts (a landing) belongs to them
	auto is_stairs = [&](int r) {
		u32 s = 0;
		for (u32 const b : members[r])
			s += slope[b];
		return float(s) >= _max(3.f, .4f * float(members[r].size()));
	};
	for (;;)
	{
		regroup();
		xr_vector<int> ordered = roots;
		std::sort(ordered.begin(), ordered.end(), [&](int x, int y) { return lowest[x] < lowest[y]; });
		int keep = -1, join = -1;
		for (int const r : ordered)
		{
			auto const c = contacts(r);
			if (is_stairs(r))
			{
				int other = -1;
				for (auto const& t : c)
					if (is_stairs(t.first) && (other < 0 || lowest[t.first] < lowest[other]))
						other = t.first;
				if (other >= 0)
				{
					keep = r;
					join = other;
				}
			}
			else if (members[r].size() <= landing_max && c.size() >= 2)
			{
				bool all_stairs = true;
				int first = -1;
				for (auto const& t : c)
				{
					all_stairs = all_stairs && is_stairs(t.first);
					if (first < 0 || lowest[t.first] < lowest[first])
						first = t.first;
				}
				if (all_stairs)
				{
					keep = first;
					join = r;
				}
			}
			if (keep >= 0)
				break;
		}
		if (keep < 0)
			break;
		parent[find(join)] = find(keep);
	}
	regroup();
	xr_vector<int> room(count);
	for (u32 b = 0; b < count; ++b)
		room[b] = find(part[b]);
	// weak dead ends (porches, steps outside a door): no strong node and a single neighbouring room -> outside
	{
		u32 dropped = 0;
		xr_vector<int> drop;
		for (int const r : roots)
		{
			bool any_strong = false;
			for (u32 const b : members[r])
				any_strong = any_strong || strong[b];
			if (!any_strong && contacts(r).size() <= 1 && roots.size() - dropped > 1)
			{
				drop.push_back(r);
				++dropped;
			}
		}
		for (u32 b = 0; b < count; ++b)
			if (std::find(drop.begin(), drop.end(), room[b]) != drop.end())
				room[b] = -1;
	}

	// 5. openings: links between rooms, and from rooms to walkable nodes outside
	xr_vector<SOpening> openings;
	auto collect_openings = [&]() {
		openings.clear();
		auto add_side = [&](int a, int b, const Fvector& pa, const Fvector& pb, u32 key, bool stair) {
			Fvector const mid = Fvector().add(pa, pb).mul(.5f);
			for (SOpening& o : openings)
			{
				if (o.window || o.room_a != a || o.room_b != b)
					continue;
				for (auto const& s : o.sides)
					if (side_middle(s).distance_to(mid) <= 1.5f * cell)
					{
						o.sides.push_back(std::make_pair(pa, pb));
						o.key = _min(o.key, key);
						o.stair = o.stair || stair;
						return;
					}
			}
			SOpening o;
			o.room_a = a;
			o.room_b = b;
			o.key = key;
			o.stair = stair;
			o.sides.push_back(std::make_pair(pa, pb));
			openings.push_back(o);
		};
		for (u32 b = 0; b < count; ++b)
		{
			if (room[b] < 0)
				continue;
			u32 const v = building[b];
			for (int i = 0; i < 4; ++i)
			{
				int const n = neighbour[b][i];
				if (n >= 0 && room[n] >= 0)
				{
					if (room[b] < room[n])
						add_side(room[b], room[n], node_position[b], node_position[n],
						         _min(v * 4 + u32(i), building[n] * 4 + u32(reverse_slot[i])), (stair_cut[b] & (1 << i)) != 0);
					continue;
				}
				u32 const out = graph.vertex(v)->link(u8(i));
				if (graph.valid_vertex_id(out) && !(edge[b] & (1 << i))) // a walkable node outside (or in a dropped part)
					add_side(room[b], -1, node_position[b], graph.vertex_position(out), v * 4 + u32(i), false);
			}
		}
		// merge openings that chained into each other after their first side
		for (u32 i = 0; i < openings.size(); ++i)
			for (u32 j = i + 1; j < openings.size(); ++j)
			{
				if (openings[i].room_a != openings[j].room_a || openings[i].room_b != openings[j].room_b)
					continue;
				bool touch = false;
				for (auto const& s : openings[i].sides)
					for (auto const& t : openings[j].sides)
						touch = touch || side_middle(s).distance_to(side_middle(t)) <= 1.5f * cell;
				if (!touch)
					continue;
				openings[i].sides.insert(openings[i].sides.end(), openings[j].sides.begin(), openings[j].sides.end());
				openings[i].key = _min(openings[i].key, openings[j].key);
				openings[i].stair = openings[i].stair || openings[j].stair;
				openings.erase(openings.begin() + j);
				j = i;
			}
	};
	collect_openings();
	// rooms joined by an opening with no wall jambs are one room (a safety net; door links have jambs by construction,
	// stair mouths have none by design)
	{
		std::map<int, int> par;
		for (u32 b = 0; b < count; ++b)
			if (room[b] >= 0)
				par[room[b]] = room[b];
		auto top = [&](int r) {
			while (par[r] != r)
				r = par[r] = par[par[r]];
			return r;
		};
		bool merged = false;
		for (SOpening const& o : openings)
		{
			if (o.room_b < 0 || o.stair)
				continue;
			Fvector centre, normal, door_centre;
			float floor_y, half_width;
			if (!opening_frame(o, centre, normal, floor_y) || jambs(centre, normal, floor_y, door_centre, half_width))
				continue;
			int const a = top(o.room_a), b = top(o.room_b);
			if (a != b)
			{
				par[b] = a;
				merged = true;
			}
		}
		if (merged)
		{
			for (u32 b = 0; b < count; ++b)
				if (room[b] >= 0)
					room[b] = top(room[b]);
			collect_openings();
		}
	}
	auto room_of = [&](u32 v) {
		auto const it = local.find(v);
		return it == local.end() ? -1 : room[it->second];
	};

	// 6. windows: a wall side open at chest height and solid at knee height, with another room or outside beyond it
	for (u32 b = 0; b < count; ++b)
	{
		if (room[b] < 0)
			continue;
		u32 const v = building[b];
		for (int i = 0; i < 4; ++i)
		{
			if (graph.valid_vertex_id(graph.vertex(v)->link(u8(i))))
				continue; // only wall sides (no link at all)
			Fvector const& d = link_dir[i];
			Fvector const base = node_position[b];
			float hit;
			if (static_ray(Fvector().set(base.x, base.y + 1.3f, base.z), d, 1.5f, hit))
				continue; // solid at chest height
			if (!static_ray(Fvector().set(base.x, base.y + .45f, base.z), d, 1.f, hit))
				continue; // open at knee height too: a drop or a gap, not a window
			float far_wall;
			bool const wall_behind = static_ray(Fvector().set(base.x, base.y + 1.3f, base.z), d, window_far, far_wall);
			float const limit = wall_behind ? far_wall - .1f : window_far;
			u32 level_node = u32(-1), below = u32(-1);
			for (float s = 1.2f; s <= limit + 1e-4f; s += .7f)
			{
				u32 const fv = cell_node(graph, Fvector().set(base.x + d.x * s, base.y + .3f, base.z + d.z * s));
				if (!graph.valid_vertex_id(fv))
					continue;
				if (_abs(graph.vertex_position(fv).y - base.y) < .6f)
				{
					level_node = fv;
					break;
				}
				if (below == u32(-1))
					below = fv;
			}
			int other;
			if (level_node != u32(-1))
			{
				other = room_of(level_node);
				if (other == room[b])
					continue; // the same room carries on behind it: low cover, a counter
			}
			else if (below != u32(-1))
			{
				other = room_of(below);
				if (other == room[b])
					continue; // a drop inside the same room: the side of a stair, a gallery edge
			}
			else if (wall_behind)
				continue; // nothing walkable before the wall behind it: a ledge or a shelf, not an opening
			else
				other = -1;
			// sill and lintel: the open band at 0.15 m steps, then each edge refined by halving (~2 cm)
			auto open_at = [&](float h) {
				float t;
				return !static_ray(Fvector().set(base.x, base.y + h, base.z), d, 1.5f, t);
			};
			int k0 = -1, k1 = -1;
			for (int k = 0; k < 17; ++k)
				if (open_at(.3f + .15f * float(k)))
				{
					if (k0 < 0)
						k0 = k;
					k1 = k;
				}
			if (k0 < 0)
				continue;
			float lo_h = .3f + .15f * float(k0), hi_h = .3f + .15f * float(k1);
			if (k0 > 0)
			{
				float closed = lo_h - .15f, open = lo_h;
				for (int it = 0; it < 3; ++it)
				{
					float const m = .5f * (closed + open);
					(open_at(m) ? open : closed) = m;
				}
				lo_h = open;
			}
			if (k1 < 16)
			{
				float open = hi_h, closed = hi_h + .15f;
				for (int it = 0; it < 3; ++it)
				{
					float const m = .5f * (open + closed);
					(open_at(m) ? open : closed) = m;
				}
				hi_h = open;
			}
			Fvector const sill = Fvector().set(base.x + d.x * .5f * cell, base.y, base.z + d.z * .5f * cell);
			auto const side = std::make_pair(sill, Fvector().mad(sill, d, .5f));
			u32 const key = v * 4 + u32(i);
			bool joined = false;
			for (SOpening& o : openings)
			{
				if (!o.window || o.room_a != room[b] || o.room_b != other || o.direction.dotproduct(d) <= .9f)
					continue;
				for (auto const& s : o.sides)
					if (s.first.distance_to(sill) <= window_join * cell)
					{
						o.sides.push_back(side);
						o.y0 = _min(o.y0, base.y + lo_h);
						o.y1 = _max(o.y1, base.y + hi_h);
						o.key = _min(o.key, key);
						joined = true;
						break;
					}
				if (joined)
					break;
			}
			if (joined)
				continue;
			SOpening o;
			o.room_a = room[b];
			o.room_b = other;
			o.window = true;
			o.direction = d;
			o.y0 = base.y + lo_h;
			o.y1 = base.y + hi_h;
			o.key = key;
			o.sides.push_back(side);
			openings.push_back(o);
		}
	}
	// windows that chained into each other after their first side (same rooms, same facing): one window
	for (u32 i = 0; i < openings.size(); ++i)
		for (u32 j = i + 1; j < openings.size(); ++j)
		{
			SOpening& oi = openings[i];
			SOpening const& oj = openings[j];
			if (!oi.window || !oj.window || oi.room_a != oj.room_a || oi.room_b != oj.room_b || oi.direction.dotproduct(oj.direction) <= .9f)
				continue;
			bool touch = false;
			for (auto const& s : oi.sides)
				for (auto const& t : oj.sides)
					touch = touch || s.first.distance_to(t.first) <= window_join * cell;
			if (!touch)
				continue;
			oi.sides.insert(oi.sides.end(), oj.sides.begin(), oj.sides.end());
			oi.y0 = _min(oi.y0, oj.y0);
			oi.y1 = _max(oi.y1, oj.y1);
			oi.key = _min(oi.key, oj.key);
			openings.erase(openings.begin() + j);
			j = i;
		}

	// 7. the Lua table
	std::map<int, u32> room_lowest; // room -> lowest level vertex id (the stable id)
	for (u32 b = 0; b < count; ++b)
		if (room[b] >= 0)
		{
			auto const it = room_lowest.find(room[b]);
			if (it == room_lowest.end() || building[b] < it->second)
				room_lowest[room[b]] = building[b];
		}
	auto room_id = [&](int r) {
		return r < 0 ? xr_string("outside") : xr_string("r") + xr_string(std::to_string(room_lowest[r]).c_str());
	};
	::luabind::object result = ::luabind::newtable(L);
	::luabind::object room_list = ::luabind::newtable(L);
	::luabind::object node_room = ::luabind::newtable(L);
	int rooms_out = 0;
	for (auto const& rl : room_lowest)
	{
		int const r = rl.first;
		xr_string const id = room_id(r);
		::luabind::object entry = ::luabind::newtable(L);
		::luabind::object nodes = ::luabind::newtable(L);
		Fvector centre{}, lo{flt_max, flt_max, flt_max}, hi{-flt_max, -flt_max, -flt_max};
		u32 n_nodes = 0, n_slope = 0;
		xr_vector<u32> mine;
		for (u32 b = 0; b < count; ++b)
		{
			if (room[b] != r)
				continue;
			nodes[++n_nodes] = building[b];
			node_room[building[b]] = id.c_str();
			centre.add(node_position[b]);
			lo.min(node_position[b]);
			hi.max(node_position[b]);
			n_slope += slope[b];
			mine.push_back(b);
		}
		centre.div(float(n_nodes));
		LPCSTR kind = "room";
		if (float(n_slope) >= _max(3.f, .4f * float(n_nodes)))
			kind = "stairs";
		else if (n_nodes >= 6)
		{
			// corridor: long and at most 3 nodes (~2.1 m) wide across its long axis, from the room's own links
			float const span_x = hi.x - lo.x, span_z = hi.z - lo.z;
			int const ax0 = span_z >= span_x ? 0 : 1, ax1 = span_z >= span_x ? 2 : 3;
			u32 narrow = 0;
			for (u32 const b0 : mine)
			{
				int w = 1;
				for (int const j : {ax0, ax1})
				{
					int c = int(b0);
					for (int step = 0; step < 3; ++step)
					{
						c = neighbour[c][j];
						if (c < 0 || room[c] != r)
							break;
						++w;
					}
				}
				narrow += w <= 3;
			}
			if (_max(span_x, span_z) >= 3.f * _min(span_x, span_z) + cell && float(narrow) >= .8f * float(n_nodes))
				kind = "corridor";
		}
		entry["id"] = id.c_str();
		entry["floor_y"] = lo.y;
		entry["nodes"] = nodes;
		entry["centre"] = centre;
		entry["x0"] = lo.x;
		entry["z0"] = lo.z;
		entry["x1"] = hi.x;
		entry["z1"] = hi.z;
		entry["kind"] = kind;
		entry["transit"] = xr_strcmp(kind, "room") != 0;
		room_list[++rooms_out] = entry;
	}
	::luabind::object portal_list = ::luabind::newtable(L);
	int portals = 0;
	Fvector const up = Fvector().set(0.f, 1.f, 0.f);
	for (SOpening const& o : openings)
	{
		Fvector centre, normal;
		float floor_y;
		if (!opening_frame(o, centre, normal, floor_y))
			continue;
		if (o.window)
			normal = o.direction;
		Fvector const across = Fvector().set(-normal.z, 0.f, normal.x);
		float lo = flt_max, hi = -flt_max;
		for (auto const& s : o.sides)
		{
			float const t = side_middle(s).sub(centre).dotproduct(across);
			lo = _min(lo, t);
			hi = _max(hi, t);
		}
		float y0 = o.y0, y1 = o.y1;
		float half_width = .5f * (hi - lo) + .5f * cell;
		::luabind::object entry = ::luabind::newtable(L);
		if (!o.window)
		{
			y0 = flt_max;
			for (auto const& s : o.sides)
				y0 = _min(y0, _min(s.first.y, s.second.y));
			float hit;
			y1 = static_ray(Fvector().set(centre.x, y0 + .3f, centre.z), up, 3.f, hit) ? y0 + .3f + hit : y0 + 2.2f;
			// the jambs give the real width, the door plane, the frame points and the wall depth
			Fvector door_centre;
			float jamb_half;
			if (jambs(centre, normal, y0, door_centre, jamb_half))
			{
				half_width = jamb_half;
				centre = door_centre;
				float const ch = y0 + jamb_height;
				entry["jamb_a"] = Fvector().set(centre.x - across.x * half_width, ch, centre.z - across.z * half_width);
				entry["jamb_b"] = Fvector().set(centre.x + across.x * half_width, ch, centre.z + across.z * half_width);
				float depth = flt_max;
				for (int side = 0; side < 2; ++side)
				{
					float const off = (half_width + .2f) * (side ? 1.f : -1.f);
					float const sx = centre.x + across.x * off, sz = centre.z + across.z * off;
					float t1, t2;
					if (static_ray(Fvector().set(sx - normal.x, ch, sz - normal.z), normal, 1.2f, t1) &&
						static_ray(Fvector().set(sx + normal.x, ch, sz + normal.z), Fvector().set(-normal.x, 0.f, -normal.z), 1.2f, t2) &&
						2.f - t1 - t2 > .02f)
						depth = _min(depth, 2.f - t1 - t2);
				}
				if (depth < flt_max)
					entry["wall_depth"] = depth;
			}
			centre.y = y0 + .5f * (y1 - y0);
		}
		else
			centre.y = .5f * (y0 + y1);
		entry["id"] = (xr_string(o.window ? "w" : "d") + xr_string(std::to_string(o.key).c_str())).c_str();
		entry["type"] = o.window ? "window" : "doorway";
		entry["room_a"] = room_id(o.room_a).c_str();
		entry["room_b"] = room_id(o.room_b).c_str();
		entry["centre"] = centre;
		entry["normal"] = normal;
		entry["half_width"] = half_width;
		entry["y0"] = y0;
		entry["y1"] = y1;
		entry["traversable"] = !o.window;
		if (o.stair)
			entry["stair"] = true;
		portal_list[++portals] = entry;
	}
	result["rooms"] = room_list;
	result["portals"] = portal_list;
	result["node_room"] = node_room;
	result["cell"] = cell;
	result["nodes_scanned"] = u32(roofs.size());
	result["mode"] = roofed_mode ? "roofed" : "walled";
	result["capped"] = capped;
	result["truncated"] = !edge_links.empty();
	return result;
}
} // namespace painter_building

inline ::luabind::object painter_building_scan(const Fvector& position, float radius)
{
	return painter_building::scan(position, radius);
}

inline LPCSTR painter_map_hash()
{
	static shared_str level_name;
	static string128 hash = "";
	if (hash[0] && level_name == Level().name())
		return hash;
	u64 const offset = 0xcbf29ce484222325ull, prime = 0x100000001b3ull;
	CLevelGraph const& graph = ai().level_graph();
	u32 const nodes = graph.header().vertex_count();
	u8 const* const first = reinterpret_cast<u8 const*>(graph.vertex(u32(0)));
	size_t const bytes = size_t(reinterpret_cast<u8 const*>(graph.vertex(nodes - 1)) - first) + sizeof(CLevelGraph::CVertex);
	u64 node_sum = offset;
	for (size_t i = 0; i + 8 <= bytes; i += 8)
	{
		u64 word;
		CopyMemory(&word, first + i, 8);
		node_sum = (node_sum ^ word) * prime;
	}
	for (size_t i = bytes & ~size_t(7); i < bytes; ++i)
		node_sum = (node_sum ^ first[i]) * prime;
	CDB::MODEL* const model = Level().ObjectSpace.GetStaticModel();
	int const verts = model->get_verts_count(), tris = model->get_tris_count();
	u64 vert_sum = offset;
	for (int v = 0; v < verts; v += 4096)
	{
		u8 const* const p = reinterpret_cast<u8 const*>(model->get_verts() + v);
		for (int k = 0; k < 12; ++k)
			vert_sum = (vert_sum ^ p[k]) * prime;
	}
	xr_sprintf(hash, "%x-%016llx-%x-%x-%016llx", nodes, node_sum, verts, tris, vert_sum);
	level_name = Level().name();
	return hash;
}
