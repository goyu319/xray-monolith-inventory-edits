#pragma once
// AI-map queries for the room-clearing AI, so its movement works wherever there is an AI map:
//   level.painter_path_corners(from_vid, to_vid): the level path between two nodes (the A* the stalkers use), pulled
//     taut to its turning points with the engine's straight-walk test -> { corners = {vector...}, vids = {vertex ids},
//     length, nodes = path node count } or nil (no path). The corners include the start and the end.
//   level.painter_nav_ray(from, to): whether a stalker can walk straight from one point to the other on the AI map ->
//     { ok, stop = the farthest point reached along the line, vid = the node there, distance } or nil when the start
//     is off the AI map.
// Both are called from scripts (the scheduler task, or the frame update): the same thread as the stalkers' own path
// building, never at the same time.

#include "level_graph.h"
#include "graph_engine.h"

namespace painter_path
{
// the node under p, else the nearest node in the cells around it (no full-level search)
inline u32 node_near(CLevelGraph const& graph, const Fvector& p)
{
	if (graph.valid_vertex_position(p))
	{
		u32 const v = graph.vertex_id(p);
		if (graph.valid_vertex_id(v))
			return v;
	}
	float const cell = graph.header().cell_size();
	u32 best = u32(-1);
	float best_distance = flt_max;
	for (int dx = -2; dx <= 2; ++dx)
		for (int dz = -2; dz <= 2; ++dz)
		{
			Fvector const q = Fvector().set(p.x + float(dx) * cell, p.y, p.z + float(dz) * cell);
			if (!graph.valid_vertex_position(q))
				continue;
			u32 const c = graph.vertex_id(q);
			if (!graph.valid_vertex_id(c))
				continue;
			float const distance = graph.vertex_position(c).distance_to_sqr(p);
			if (distance < best_distance)
			{
				best_distance = distance;
				best = c;
			}
		}
	return best;
}

inline ::luabind::object corners(u32 from, u32 to)
{
	CLevelGraph const& graph = ai().level_graph();
	if (!graph.valid_vertex_id(from) || !graph.valid_vertex_id(to))
		return ::luabind::object();
	xr_vector<u32> path;
	GraphEngineSpace::CBaseParameters parameters;
	parameters.max_visited_node_count = 30000;
	if (!ai().graph_engine().search(graph, from, to, &path, parameters) || path.empty())
		return ::luabind::object();
	// from each corner, the farthest node along the path (up to 200 nodes on) reachable in a straight walk; a grid path
	// zigzags, so a blocked node does not mean the ones after it are
	xr_vector<u32> turn;
	turn.push_back(0);
	for (u32 i = 0; i + 1 < path.size();)
	{
		Fvector const start = graph.vertex_position(path[i]);
		u32 j = i + 1;
		u32 const last = _min(u32(path.size()) - 1, i + 200);
		for (u32 k = last; k > i + 1; --k)
			if (graph.check_vertex_in_direction(path[i], start, path[k]))
			{
				j = k;
				break;
			}
		turn.push_back(j);
		i = j;
	}
	lua_State* const L = ai().script_engine().lua();
	::luabind::object result = ::luabind::newtable(L);
	::luabind::object points = ::luabind::newtable(L);
	::luabind::object vids = ::luabind::newtable(L);
	float length = 0.f;
	for (u32 k = 0; k < turn.size(); ++k)
	{
		Fvector const p = graph.vertex_position(path[turn[k]]);
		points[k + 1] = p;
		vids[k + 1] = path[turn[k]];
		if (k)
			length += p.distance_to(graph.vertex_position(path[turn[k - 1]]));
	}
	result["corners"] = points;
	result["vids"] = vids;
	result["length"] = length;
	result["nodes"] = u32(path.size());
	return result;
}

inline ::luabind::object nav_ray(const Fvector& from, const Fvector& to)
{
	CLevelGraph const& graph = ai().level_graph();
	if (!_valid(from) || !_valid(to))
		return ::luabind::object();
	u32 const start = node_near(graph, from);
	if (!graph.valid_vertex_id(start))
		return ::luabind::object();
	Fvector const origin = graph.vertex_position(start);
	auto reach = [&](const Fvector& p) {
		return graph.valid_vertex_position(p) ? graph.check_position_in_direction(start, origin, p) : u32(-1);
	};
	u32 vid = reach(to);
	bool const ok = graph.valid_vertex_id(vid);
	Fvector stop = to;
	if (!ok)
	{
		// the farthest point along the line still reachable in a straight walk (halving, ~1/1000 of the length)
		float lo = 0.f, hi = 1.f;
		vid = start;
		for (int it = 0; it < 10; ++it)
		{
			float const mid = .5f * (lo + hi);
			u32 const v = reach(Fvector().lerp(origin, to, mid));
			if (graph.valid_vertex_id(v))
			{
				lo = mid;
				vid = v;
			}
			else
				hi = mid;
		}
		stop.lerp(origin, to, lo);
	}
	stop.y = graph.vertex_plane_y(vid, stop.x, stop.z);
	lua_State* const L = ai().script_engine().lua();
	::luabind::object result = ::luabind::newtable(L);
	result["ok"] = ok;
	result["stop"] = stop;
	result["vid"] = vid;
	result["distance"] = origin.distance_to_xz(stop);
	return result;
}
} // namespace painter_path

inline ::luabind::object painter_path_corners(u32 from, u32 to)
{
	return painter_path::corners(from, to);
}

inline ::luabind::object painter_nav_ray(const Fvector& from, const Fvector& to)
{
	return painter_path::nav_ray(from, to);
}
