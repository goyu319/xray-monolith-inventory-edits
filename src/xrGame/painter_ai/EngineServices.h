#pragma once
#include "NodeRoute.h"
// painter_ai/EngineServices.h - what the decision core may call (contract v2.2 EngineServices): the worker-safe cover
// query and static ray batch, the per-level building cache, and the work queue on the Painter's worker pool.
#include "CombatContract.h"

namespace painter { namespace combat { namespace engine {

// Any thread (values in, values out): the cover points' quadtree and the AI map's cover values are read-only after the
// level loads, and the ray picker keeps its collider per thread. No restrictions (the publish task re-checks them):
// CoverSpot::accessible / reachable stay Evidence::unknown; protection_known is yes when threats were evaluated.
std::uint8_t cover_query(const CoverQueryInput& in, CoverSpot* out, std::uint8_t capacity);
void ray_batch(const Segment* in, float* hit, std::uint32_t count);

// Any thread (contract v2.6): both read only the level graph's vertex array and header (the mapped level.ai, immutable
// after load) - never the restriction mask, never a log line. project: the nearest node to p by 3D distance among the
// nodes whose centre is within max_horizontal_m flat (searched up to 32 m) and max_vertical_m in height, ties to the
// lower id; invalid_node when none, so a point on an upper floor never snaps to the floor below.
std::uint32_t project(const Vec3& p, float max_horizontal_m, float max_vertical_m, Vec3* snapped);
bool node_position(std::uint32_t node, Vec3* out);
// Any thread (contract v2.13): metres along node centres between two nodes, -1 when the goal is not reached within
// max_nodes expansions (<= 65536). Its own bounded A* (NodePath.h) with this thread's scratch: the engine's CGraphEngine
// keeps one search state for every stalker's path building and is not used. No restrictions (publish re-checks).
float path_length(std::uint32_t from_node, std::uint32_t to_node, std::uint32_t max_nodes);

// The building cache of the current level, loaded on the authority thread from the installed building data
// (painter_ai_buildings_<level>.get(), whose hash must equal the running map's) once per level_epoch. Null until loaded
// or when the map has no data / a different hash. Readers keep it pinned by keeping EngineServices for one epoch.
// Data v9 adds each portal side's packed red-light table (LitPack.h); a table that fails to decode is left empty (the
// core's own rays) and counted in the load line. lit_complete (v2.7) when the file is v9, says lit_skipped = 0 and every
// table decoded.
const BuildingCache* buildings();
void load_buildings();                              // authority thread: (re)load when the level epoch changed

// The job queue on the Painter's pool: its own lane of threads there (job_workers), never the wound workers, so neither
// holds the other back (contract v2.8); a per-frame budget of lane time, cancellation (per ticket and on a level change),
// completion age (submit -> done), wait (submit -> start) and `late` (waits over job_max_wait_ms, a watchdog only).
JobQueue& jobs();
void configure_jobs(float frame_budget_ms, std::uint32_t max_depth, std::uint32_t workers, float max_wait_ms);

// Everything at once, for the core (once per level_epoch).
EngineServices services();

// Door <-> portal, one match used both ways: Portal::door is set at load_buildings for the doors online then (the pinned
// cache never changes afterwards); Door::portal comes from the same per-door cache in capture_doors, matching doors that
// came online later on first sight. A door matches the nearest doorway portal whose centre is within half_width + 0.6 m;
// a door near no portal, or a door two portals claim, is logged once. 0 = no portal.
std::uint32_t door_portal(std::uint16_t door, const Vec3& at);

std::uint32_t portal_named(std::uint32_t building, const char* data_id);
std::uint32_t room_named(const char* data_id);

// [painter_combat] in the game's settings (mod_system_painter_combat.ltx), read at every level load: enabled (the master
// switch, default false: the owning action stays inert and the Lua adapter keeps its men), job_budget_ms (4),
// job_max_depth (256), job_workers (1), job_max_wait_ms (8), and every Config field under its struct name (defaults:
// Config's own).
void configure_from_settings();
bool combat_enabled();
const Config& combat_config();

// The per-frame entry: CLevel::OnFrame pushes it onto the AI thread (Device.seqParallel, after the scheduler) or calls it
// directly when mt_Scheduler is off. On a new level instance it reads the settings and loads the buildings (only while a
// tick is registered), then runs the tick: the adapter's capture, decide and publish step. Should the master switch go
// off after the tick ran enabled on this level, the tick runs once more in the disabled state (its master-off branch
// releases the men it owns). Today the switch is read only at level start, so that happens only once a toggle exists.
struct FrameTask { void run(); };
FrameTask& frame_task();
void level_created();                               // CLevel's constructor, before the level's scripts: new epoch
void level_destroyed();                             // CLevel's destructor, first: cancel and drain every job, then
                                                    // drop the building cache (the lifetime guarantee)
void set_tick(void (*tick)(TimeMs now));

}}} // namespace painter::combat::engine
