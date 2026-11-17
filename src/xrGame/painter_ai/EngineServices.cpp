#include "pch_script.h"
#include "EngineServices.h"
#include "EngineCapture.h"
#include "EngineNearMiss.h"
#include "EngineCompat.h"
#include "FactionStyleParse.h"
#include "OutdoorSettingsRead.h"
#include "LitPack.h"
#include "NodePath.h"
#include "NodeRoute.h"
#include "character_community.h"
#include "cover_manager.h"
#include "cover_point.h"
#include "level_graph.h"
#include "Level.h"
#include "ai_space.h"
#include "../xrServerEntities/script_engine.h"
#include "painter_voxel/LiveDamageWorker.h"
#include "doors_manager.h"
#include "doors_door.h"
#include "PhysicObject.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include "EngineRecorder.h"
#include "EngineTuning.h"
#include <cstdio>
#include <ctime>

float g_painter_look_head_rate_deg = 150.f;
float g_painter_look_body_rate_deg = 120.f;
float g_painter_look_dead_zone_deg = 2.f;

// ---------------------------------------------------------------------------------------------------------------------
// native17: publication reject counters and the live snapshot recorder (see EngineRecorder.h)
namespace painter { namespace combat { namespace engine {
namespace {
const char* const k_reject_names[13] = {"none", "epoch", "geometry", "older_snapshot", "generation", "ownership", "goal_invalid",
                                        "goal_inaccessible", "order_seq", "issued_in_future", "unsupported", "fire_point", "order_invalid"};
struct RejectCounters { std::uint64_t by_reason[13] = {}; std::uint64_t envelopes = 0; bool sampled[13] = {}; } rejects;

struct Recorder
{
    bool on = false, capped = false, header_written = false;
    std::uint64_t max_bytes = 200ull << 20, bytes = 0, records = 0;
    FILE* file = nullptr;
    std::string line, handbacks, compat;       // the tick's record under construction; events buffered between ticks
    float max_ms = 0.f;
} rec;

// ---- a small JSON writer (append-only, no allocation beyond the line) ----
void j_raw(std::string& s, const char* text) { s += text; }
void j_sep(std::string& s) { if (!s.empty() && s.back() != '{' && s.back() != '[' && s.back() != ':') s += ','; }
void j_key(std::string& s, const char* key) { j_sep(s); s += '"'; s += key; s += "\":"; }
void j_u(std::string& s, std::uint64_t v) { char b[32]; std::snprintf(b, sizeof b, "%llu", (unsigned long long)v); s += b; }
void j_i(std::string& s, long long v) { char b[32]; std::snprintf(b, sizeof b, "%lld", v); s += b; }
void j_f(std::string& s, float v) { char b[32]; std::snprintf(b, sizeof b, "%.9g", double(v)); s += b; }
void j_b(std::string& s, bool v) { s += v ? "true" : "false"; }
void j_str(std::string& s, const char* v) { s += '"'; for (const char* c = v; c && *c; ++c) { if (*c == '"' || *c == '\\') s += '\\'; if (*c == '\n') { s += "\\n"; continue; } s += *c; } s += '"'; }
void j_vec(std::string& s, const Vec3& v) { s += '['; j_f(s, v.x); s += ','; j_f(s, v.y); s += ','; j_f(s, v.z); s += ']'; }
void j_actor(std::string& s, const ActorKey& k) { s += "{\"id\":"; j_u(s, k.id); s += ",\"gen\":"; j_u(s, k.generation); s += '}'; }
void j_ku(std::string& s, const char* key, std::uint64_t v) { j_key(s, key); j_u(s, v); }
void j_ki(std::string& s, const char* key, long long v) { j_key(s, key); j_i(s, v); }
void j_kf(std::string& s, const char* key, float v) { j_key(s, key); j_f(s, v); }
void j_kb(std::string& s, const char* key, bool v) { j_key(s, key); j_b(s, v); }
void j_kv(std::string& s, const char* key, const Vec3& v) { j_key(s, key); j_vec(s, v); }
void j_ka(std::string& s, const char* key, const ActorKey& k) { j_key(s, key); j_actor(s, k); }
void j_open(std::string& s, const char* key) { j_key(s, key); s += '{'; }
void j_close(std::string& s) { s += '}'; }
void j_array(std::string& s, const char* key) { j_key(s, key); s += '['; }
void j_end_array(std::string& s) { s += ']'; }

void j_contact(std::string& s, const Contact& c)
{
    s += '{'; j_ka(s, "key", c.key); j_kv(s, "position", c.position); j_kv(s, "facing", c.facing); j_kf(s, "distance", c.distance);
    j_ku(s, "observed_at_ms", c.observed_at_ms); j_ku(s, "last_seen_ms", c.last_seen_ms); j_ku(s, "perception_seq", c.perception_seq);
    j_kb(s, "visible", c.visible); j_kb(s, "hostile", c.hostile); j_kb(s, "wounded", c.wounded); j_kb(s, "facing_me", c.facing_me);
    j_kb(s, "alive_when_observed", c.alive_when_observed); j_kb(s, "finish_allowed", c.finish_allowed); s += '}';
}
void j_actor_snapshot(std::string& s, const ActorSnapshot& a)
{
    s += '{'; j_ka(s, "key", a.key); j_ku(s, "level_epoch", a.level_epoch); j_ku(s, "snapshot_seq", a.snapshot_seq);
    j_ku(s, "ownership_generation", a.ownership_generation); j_ku(s, "restrictions_revision", a.restrictions_revision);
    j_ku(s, "observed_at_ms", a.observed_at_ms); j_kb(s, "alive", a.alive); j_kb(s, "wounded", a.wounded); j_kb(s, "reloading", a.reloading);
    j_kb(s, "current_position_accessible", a.current_position_accessible); j_kb(s, "accessible_current_order", a.accessible_current_order);
    j_kv(s, "position", a.position); j_kv(s, "forward", a.forward); j_kv(s, "head_dir", a.head_dir); j_kf(s, "head_pitch_rad", a.head_pitch_rad);
    j_kf(s, "health", a.health); j_ku(s, "node_id", a.node_id); j_ku(s, "room_id", a.room_id); j_ku(s, "building_id", a.building_id);
    j_ku(s, "faction", a.faction); j_ku(s, "native_squad", a.native_squad);
    j_open(s, "weapon"); j_ku(s, "id", a.weapon.id); j_ku(s, "kind", std::uint8_t(a.weapon.kind)); j_ku(s, "state", std::uint8_t(a.weapon.state));
    j_ku(s, "ammo_in_mag", a.weapon.ammo_in_mag); j_ku(s, "mag_size", a.weapon.mag_size); j_ku(s, "ammo_reserve", a.weapon.ammo_reserve); j_close(s);
    j_open(s, "grenades"); j_ku(s, "count", a.grenades.count); j_ku(s, "slot_item", a.grenades.slot_item); j_close(s);
    j_key(s, "enemy"); j_contact(s, a.enemy);
    j_array(s, "visible_enemies"); for (std::size_t i = 0; i < a.visible_enemies.count && i < a.visible_enemies.items.size(); ++i) { j_sep(s); j_contact(s, a.visible_enemies.items[i]); } j_end_array(s);
    j_array(s, "seen_dead"); for (std::size_t i = 0; i < a.seen_dead.count && i < a.seen_dead.items.size(); ++i) { j_sep(s); j_actor(s, a.seen_dead.items[i]); } j_end_array(s);
    j_array(s, "heard"); for (std::size_t i = 0; i < a.heard.count && i < a.heard.items.size(); ++i) { const Heard& h = a.heard.items[i]; j_sep(s); s += '{';
        j_ku(s, "kind", std::uint8_t(h.kind)); j_kv(s, "position", h.position); j_kf(s, "power", h.power); j_ku(s, "at_ms", h.at_ms); j_ka(s, "source", h.source); s += '}'; } j_end_array(s);
    j_array(s, "hits"); for (std::size_t i = 0; i < a.hits.count && i < a.hits.items.size(); ++i) { const Hit& h = a.hits.items[i]; j_sep(s); s += '{';
        j_ku(s, "at_ms", h.at_ms); j_kv(s, "dir", h.dir); j_ka(s, "from", h.from); s += '}'; } j_end_array(s);
    j_array(s, "near_misses"); for (std::size_t i = 0; i < a.near_misses.count && i < a.near_misses.items.size(); ++i) { const NearMiss& n = a.near_misses.items[i]; j_sep(s); s += '{';
        j_ku(s, "at_ms", n.at_ms); j_kf(s, "miss_m", n.miss_m); j_kv(s, "origin", n.origin); j_kv(s, "dir", n.dir); j_ka(s, "from", n.from); s += '}'; } j_end_array(s);
    j_open(s, "danger"); j_ku(s, "kind", std::uint8_t(a.danger.kind)); j_ku(s, "perceive", a.danger.perceive); j_ku(s, "at_ms", a.danger.at_ms); j_kv(s, "position", a.danger.position); j_close(s);
    j_open(s, "native_properties"); j_kb(s, "enemy", a.native_properties.enemy); j_kb(s, "danger", a.native_properties.danger); j_kb(s, "anomaly", a.native_properties.anomaly); j_close(s);
    j_ku(s, "flags", a.flags); j_ku(s, "yield_reasons", a.yield_reasons);
    j_open(s, "last_rejected"); j_ku(s, "order_seq", a.last_rejected.order_seq); j_ku(s, "node", a.last_rejected.node); j_ku(s, "reason", a.last_rejected.reason); j_ku(s, "restrictions_revision", a.last_rejected.restrictions_revision); j_close(s);
    j_open(s, "last_applied"); j_ku(s, "order_seq", a.last_applied.order_seq); j_ku(s, "at_ms", a.last_applied.at_ms); j_ku(s, "weapon", a.last_applied.weapon); j_kb(s, "fired_since_previous", a.last_applied.fired_since_previous); j_close(s);
    j_open(s, "throw_state"); j_ku(s, "serial", a.throw_state.serial); j_ku(s, "phase", a.throw_state.phase); j_ku(s, "reason", a.throw_state.reason); j_ku(s, "at_ms", a.throw_state.at_ms);
    j_ku(s, "releases", a.throw_state.releases); j_ku(s, "projectile", a.throw_state.projectile); j_ku(s, "source_item", a.throw_state.source_item); j_ku(s, "released_at_ms", a.throw_state.released_at_ms); j_close(s);
    j_open(s, "aid_state"); j_ka(s, "target", a.aid_state.target); j_ku(s, "phase", a.aid_state.phase); j_ku(s, "at_ms", a.aid_state.at_ms); j_close(s);
    j_open(s, "aid_eligibility"); j_ku(s, "allowed", std::uint8_t(a.aid_eligibility.allowed)); j_kb(s, "claimed", a.aid_eligibility.claimed); j_ka(s, "claimant", a.aid_eligibility.claimant); j_ku(s, "observed_at_ms", a.aid_eligibility.observed_at_ms); j_close(s);
    j_open(s, "compat"); j_ku(s, "token", a.compat.token); j_ku(s, "pass_revision", a.compat.pass_revision); j_ku(s, "sealed", std::uint8_t(a.compat.sealed)); j_ku(s, "veto", std::uint8_t(a.compat.veto)); j_ku(s, "at_ms", a.compat.at_ms); j_close(s);
    s += '}';
}
void j_side(std::string& s, const SideSnapshot& d)
{
    s += '{'; j_ku(s, "side_id", d.side_id); j_ku(s, "faction", d.faction); j_ku(s, "group", d.group);
    j_array(s, "members"); for (const auto& m : d.members) { j_sep(s); j_actor(s, m); } j_end_array(s);
    j_open(s, "mission"); j_ku(s, "revision", d.mission.revision); j_ku(s, "kind", std::uint8_t(d.mission.kind)); j_ku(s, "building", d.mission.building); j_ku(s, "entry_portal", d.mission.entry_portal);
    j_array(s, "rooms"); for (std::size_t i = 0; i < d.mission.rooms.count && i < d.mission.rooms.items.size(); ++i) { j_sep(s); j_u(s, d.mission.rooms.items[i]); } j_end_array(s); j_close(s);
    j_array(s, "known_enemies"); for (const auto& k : d.known_enemies) { j_sep(s); s += '{'; j_ka(s, "key", k.key); j_kv(s, "position", k.position); j_ku(s, "latest_ms", k.latest_ms); j_kb(s, "visible", k.visible); j_ku(s, "member_mask", k.member_mask);
        j_array(s, "reporters"); for (std::size_t i = 0; i < k.reporters.count && i < k.reporters.items.size(); ++i) { j_sep(s); j_actor(s, k.reporters.items[i]); } j_end_array(s); s += '}'; } j_end_array(s);
    j_ku(s, "building_id", d.building_id); j_ku(s, "mission_state", d.mission_state); j_kv(s, "link_up", d.link_up); s += '}';
}
void j_order(std::string& s, const Order& o)
{
    s += '{'; j_ka(s, "actor", o.actor); j_ku(s, "level_epoch", o.level_epoch); j_ku(s, "snapshot_seq", o.snapshot_seq); j_ku(s, "order_seq", o.order_seq);
    j_ku(s, "ownership_generation", o.ownership_generation); j_ku(s, "geometry_revision", o.geometry_revision); j_ku(s, "restrictions_revision", o.restrictions_revision);
    j_ku(s, "issued_at_ms", o.issued_at_ms); j_ku(s, "expires_at_ms", o.expires_at_ms); j_ku(s, "kind", std::uint8_t(o.kind)); j_ku(s, "role", o.role);
    j_ku(s, "goal_node", o.goal_node); j_kv(s, "goal_position", o.goal_position); j_kf(s, "arrival_radius_m", o.arrival_radius_m);
    j_ku(s, "pace", std::uint8_t(o.pace)); j_ku(s, "stance", std::uint8_t(o.stance));
    j_open(s, "look"); j_ku(s, "kind", std::uint8_t(o.look.kind)); j_kv(s, "value", o.look.value); j_ka(s, "target", o.look.target); j_close(s);
    j_open(s, "fire"); j_kb(s, "allowed", o.fire.allowed); j_kv(s, "at", o.fire.at); j_kb(s, "at_valid", o.fire.at_valid); j_ku(s, "until_ms", o.fire.until_ms);
    j_ku(s, "queue", o.fire.queue); j_ku(s, "interval_ms", o.fire.interval_ms); j_kf(s, "ammo_floor", o.fire.ammo_floor); j_close(s);
    j_kf(s, "lean", o.lean); j_kv(s, "lean_edge", o.lean_edge);
    j_open(s, "expose"); j_kb(s, "stand", o.expose.stand); j_kb(s, "lean", o.expose.lean); j_close(s);
    j_ku(s, "flags", o.flags);
    j_open(s, "portal"); j_kv(s, "centre", o.portal.centre); j_kv(s, "tangent", o.portal.tangent); j_kv(s, "normal", o.portal.normal); j_kf(s, "half_width", o.portal.half_width); j_kf(s, "thickness", o.portal.thickness); j_kb(s, "valid", o.portal.valid); j_close(s);
    j_open(s, "throw_order"); j_ku(s, "door", o.throw_order.door); j_ku(s, "item", o.throw_order.item); j_kv(s, "target", o.throw_order.target); j_kb(s, "valid", o.throw_order.valid); j_ku(s, "serial", o.throw_order.serial); j_ku(s, "portal", o.throw_order.portal); j_close(s);
    j_ka(s, "aid", o.aid); j_ka(s, "finish", o.finish); j_ku(s, "abort_vertex", o.abort_vertex); j_kf(s, "speed_scale", o.speed_scale); j_ku(s, "gait", std::uint8_t(o.gait));
    j_open(s, "pie"); j_kv(s, "pivot", o.pie.pivot); j_kb(s, "valid", o.pie.valid); j_close(s);
    j_open(s, "corner"); j_kv(s, "point", o.corner.point); j_kb(s, "valid", o.corner.valid); j_close(s);
    j_ki(s, "side", o.side);
    j_open(s, "assignment"); j_ku(s, "sector", o.assignment.sector); j_ku(s, "portal", o.assignment.portal); j_ku(s, "slot", o.assignment.slot); j_ku(s, "generation", o.assignment.generation); j_close(s);
    s += '}';
}

bool rec_open()
{
    if (rec.file) return true;
    string_path path; string64 stamp; time_t t = time(nullptr); tm lt; localtime_s(&lt, &t);
    strftime(stamp, sizeof stamp, "%Y%m%d_%H%M%S", &lt);
    string256 name; xr_sprintf(name, "painter_replay_%s_%s.jsonl", g_pGameLevel ? *Level().name() : "nolevel", stamp);
    FS.update_path(path, "$logs$", name);
    rec.file = fopen(path, "wb");
    if (!rec.file) { Msg("! [painter_combat] phase=replay_record error=open path=%s", path); rec.on = false; return false; }
    Msg("[painter_combat] phase=replay_record state=open path=%s max_mb=%llu", path, (unsigned long long)(rec.max_bytes >> 20));
    return true;
}
void rec_write(const std::string& s)
{
    if (!rec.file) return;
    if (rec.bytes + s.size() > rec.max_bytes) {
        if (!rec.capped) { rec.capped = true; Msg("[painter_combat] phase=replay_record state=capped bytes=%llu records=%llu", (unsigned long long)rec.bytes, (unsigned long long)rec.records); }
        return;
    }
    fwrite(s.data(), 1, s.size(), rec.file); rec.bytes += s.size(); ++rec.records;
}
void rec_header()
{
    if (rec.header_written) return; rec.header_written = true;
    std::string h = "{\"header\":true,\"format\":\"native-replay-v1\",";
    h += "\"level\":"; j_str(h, g_pGameLevel ? *Level().name() : "nolevel"); h += ",\"build\":\"native27 " __DATE__ " " __TIME__ "\",\"contract\":\"v2.24\"";
    const Config& c = combat_config();
    j_open(h, "config"); j_ku(h, "first_order_deadline_ms", c.first_order_deadline_ms); j_ku(h, "order_lifetime_ms", c.order_lifetime_ms); j_ku(h, "stale_release_ms", c.stale_release_ms);
    j_ku(h, "snapshot_max_age_ms", c.snapshot_max_age_ms); j_ku(h, "memory_max_age_ms", c.memory_max_age_ms); j_ku(h, "posture_dwell_ms", c.posture_dwell_ms);
    j_ku(h, "unseen_reaction_source_ms", c.unseen_reaction_source_ms); j_ku(h, "unseen_reaction_hold_ms", c.unseen_reaction_hold_ms);
    j_kf(h, "lifecycle_release_range_m", c.lifecycle_release_range_m); j_kf(h, "lifecycle_distance_cost_per_m", c.lifecycle_distance_cost_per_m); j_ku(h, "lifecycle_required_door_stuck_ms", c.lifecycle_required_door_stuck_ms); j_close(h);
    h += "}\n"; rec_write(h);
}
}

void recorder_configure(bool on, std::uint32_t max_mb)
{
    rec.on = on; rec.max_bytes = std::uint64_t(max_mb ? max_mb : 1) << 20;
}
void recorder_snapshot(const WorldSnapshot& w)
{
    if (!rec.on || rec.capped) return;
    const auto started = std::chrono::steady_clock::now();
    if (!rec_open()) return;
    rec_header();
    std::string& s = rec.line; s.clear(); s.reserve(1 << 16);
    s += '{'; j_ku(s, "t_ms", w.now_ms); j_ku(s, "seq", w.sequence);
    j_open(s, "snapshot"); j_ku(s, "level_epoch", w.level_epoch); j_ku(s, "sequence", w.sequence); j_ku(s, "geometry_revision", w.geometry_revision); j_ku(s, "map_hash", w.map_hash); j_ku(s, "now_ms", w.now_ms);
    j_array(s, "actors"); for (const auto& a : w.actors) { j_sep(s); j_actor_snapshot(s, a); } j_end_array(s);
    j_array(s, "sides"); for (const auto& d : w.sides) { j_sep(s); j_side(s, d); } j_end_array(s);
    j_array(s, "doors"); for (const auto& d : w.doors) { j_sep(s); s += '{'; j_ku(s, "id", d.id); j_ku(s, "portal", d.portal); j_kf(s, "open", d.open); j_ku(s, "state", std::uint8_t(d.state)); j_kb(s, "locked", d.locked); j_kb(s, "held", d.held); s += '}'; } j_end_array(s);
    j_close(s);
    j_array(s, "decisions");   // closed by recorder_tick_end
    rec.max_ms = std::max(rec.max_ms, std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - started).count());
}
void recorder_publish(const Decision& d, const PublishResult& r)
{
    if (!rec.on || rec.capped || rec.line.empty()) return;
    const auto started = std::chrono::steady_clock::now();
    std::string& s = rec.line; j_sep(s); s += '{';
    j_open(s, "decision"); j_ku(s, "side_id", d.side_id); j_ku(s, "level_epoch", d.level_epoch); j_ku(s, "snapshot_seq", d.snapshot_seq); j_ku(s, "geometry_revision", d.geometry_revision);
    j_array(s, "orders"); for (const auto& o : d.orders) { j_sep(s); j_order(s, o); } j_end_array(s);
    j_array(s, "releases"); for (const auto& k : d.releases) { j_sep(s); j_actor(s, k); } j_end_array(s);
    j_array(s, "events"); for (const auto& e : d.events) { j_sep(s); s += '{'; j_ku(s, "kind", e.kind); j_ku(s, "room", e.room); j_ku(s, "detail", e.detail); s += '}'; } j_end_array(s);
    j_ku(s, "cost_us", d.cost_us); j_close(s);
    j_open(s, "result"); j_ku(s, "envelope", r.envelope);
    j_array(s, "accepted_orders"); for (const auto& k : r.accepted_orders) { j_sep(s); j_actor(s, k); } j_end_array(s);
    j_array(s, "accepted_releases"); for (const auto& k : r.accepted_releases) { j_sep(s); j_actor(s, k); } j_end_array(s);
    j_array(s, "rejected"); for (const auto& f : r.rejected) { j_sep(s); s += '{'; j_ka(s, "actor", f.actor); j_ku(s, "reason", f.reason); j_ku(s, "yields", f.yields); s += '}'; } j_end_array(s);
    j_close(s); s += '}';
    rec.max_ms = std::max(rec.max_ms, std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - started).count());
}
void recorder_handback(std::uint16_t actor, std::uint64_t generation, std::uint64_t ownership, std::uint64_t order, const char* reason, std::uint32_t yield_bits, TimeMs at)
{
    if (!rec.on || rec.capped) return;
    std::string& s = rec.handbacks; j_sep(s); s += '{'; j_ku(s, "actor", actor); j_ku(s, "generation", generation); j_ku(s, "ownership", ownership); j_ku(s, "order", order);
    j_key(s, "reason"); j_str(s, reason ? reason : ""); j_ku(s, "yield_bits", yield_bits); j_ku(s, "at_ms", at); s += '}';
}
void recorder_compat(std::uint16_t actor, std::uint8_t verdict)
{
    if (!rec.on || rec.capped) return;
    std::string& s = rec.compat; j_sep(s); s += '{'; j_ku(s, "actor", actor); j_ku(s, "verdict", verdict); s += '}';
}
void recorder_tick_end(TimeMs now)
{
    if (!rec.on || rec.capped || rec.line.empty()) return;
    const auto started = std::chrono::steady_clock::now();
    std::string& s = rec.line; j_end_array(s);
    j_array(s, "handbacks"); s += rec.handbacks; j_end_array(s); rec.handbacks.clear();
    j_array(s, "compat"); s += rec.compat; j_end_array(s); rec.compat.clear();
    j_ku(s, "end_ms", now); s += "}\n"; rec_write(s); s.clear();
    rec.max_ms = std::max(rec.max_ms, std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - started).count());
}
void recorder_level_end()
{
    if (rec.file) { fclose(rec.file); rec.file = nullptr; Msg("[painter_combat] phase=replay_record state=closed bytes=%llu records=%llu max_ms=%.2f", (unsigned long long)rec.bytes, (unsigned long long)rec.records, rec.max_ms); }
    rec.line.clear(); rec.handbacks.clear(); rec.compat.clear(); rec.header_written = false; rec.capped = false; rec.bytes = 0; rec.records = 0; rec.max_ms = 0.f;
}
float recorder_max_ms() { return rec.max_ms; }
std::uint64_t recorder_bytes() { return rec.bytes; }

void note_publish_rejects(const Decision& d, const PublishResult& r)
{
    if (r.envelope != reject_none) {
        ++rejects.envelopes; const std::uint8_t e = r.envelope < 13 ? r.envelope : 0; ++rejects.by_reason[e];
        if (!rejects.sampled[e]) { rejects.sampled[e] = true;
            Msg("[painter_combat] phase=reject_sample reason=%s envelope=1 side=%u orders=%u snapshot_seq=%llu", k_reject_names[e], d.side_id, unsigned(d.orders.size()), (unsigned long long)d.snapshot_seq); }
    }
    for (const auto& f : r.rejected) {
        const std::uint8_t e = f.reason < 13 ? f.reason : 0; ++rejects.by_reason[e];
        if (rejects.sampled[e]) continue; rejects.sampled[e] = true;
        const Order* o = nullptr; for (const auto& x : d.orders) if (x.actor == f.actor) { o = &x; break; }
        Msg("[painter_combat] phase=reject_sample reason=%s actor=%u order=%llu kind=%u goal=%u at=%llu yields=%u", k_reject_names[e], f.actor.id,
            (unsigned long long)(o ? o->order_seq : 0), unsigned(o ? std::uint8_t(o->kind) : 255), unsigned(o ? o->goal_node : invalid_node),
            (unsigned long long)(o ? o->issued_at_ms : 0), f.yields);
    }
}
const char* reject_reason_name(std::uint8_t reason) { return k_reject_names[reason < 13 ? reason : 0]; }
std::string reject_summary()
{
    std::string s = "rejects=";
    for (int i = 1; i < 13; ++i) { char b[64]; std::snprintf(b, sizeof b, "%s%s:%llu", i > 1 ? "," : "", k_reject_names[i], (unsigned long long)rejects.by_reason[i]); s += b; }
    char b[64]; std::snprintf(b, sizeof b, " envelopes=%llu", (unsigned long long)rejects.envelopes); s += b; return s;
}
void reset_reject_counters() { rejects = RejectCounters{}; }
}}}

gfy_live::Pool* painter_voxel_live_pool(); // PainterVoxelLive_impl.h: the Painter's worker pool (made on first use)

namespace painter { namespace combat { namespace engine {
namespace {
Fvector native(const Vec3& v) { return Fvector().set(v.x, v.y, v.z); }

float static_hit(const Fvector& a, const Fvector& b)
{
    Fvector direction = Fvector().sub(b, a);
    const float length = direction.magnitude();
    if (length < EPS_L) return -1.f;
    direction.div(length);
    collide::rq_result result;
    if (!Level().ObjectSpace.RayPick(a, direction, length, collide::rqtStatic, result, nullptr)) return -1.f;
    return result.range;
}

// cover values toward (dx, dz) from the four quadrant values (+x, +z, -x, -z), as level.vertex_info gives them
void cover_toward(const float (&high)[4], const float (&low)[4], float dx, float dz, float& hi, float& lo)
{
    float a = atan2f(dz, dx);
    if (a < 0.f) a += PI_MUL_2;
    const float q = a / PI_DIV_2;
    const int i = int(floorf(q)) % 4, j = (i + 1) % 4;
    const float f = q - floorf(q);
    hi = high[i] * (1.f - f) + high[j] * f;
    lo = low[i] * (1.f - f) + low[j] * f;
}

// ---- the building cache ----
std::unique_ptr<BuildingCache> cache;
std::unordered_map<std::uint32_t, std::pair<std::uint32_t, std::uint32_t>> node_index;
std::unordered_map<std::string, std::uint32_t> portal_names;         // "<building>:<data id>" -> the cache's portal id
std::uint64_t cache_epoch = 0;
std::unordered_map<std::uint16_t, std::uint32_t> door_to_portal;     // door id -> portal id (0 = none), per level
std::uint64_t level_serial = 0, seen_serial = 0;
void (*tick_fn)(TimeMs) = nullptr;
bool combat_on = false;                                               // [painter_combat] enabled
bool ticked_on = false;                                               // the tick ran enabled on this level
Config combat_cfg;

// The nearest doorway portal of the cache within half_width + 0.6 m (flat) of `at`, with the vertical span around it;
// `claimants` counts the portals in reach (two or more: logged by the caller).
std::uint32_t match_portal(const BuildingCache& c, const Fvector& at, unsigned& claimants)
{
    std::uint32_t best = 0;
    float best_d = flt_max;
    claimants = 0;
    for (const auto& b : c.buildings)
        for (const auto& p : b.portals)
        {
            if (!p.doorway) continue;
            const float d = _sqrt(_sqr(at.x - p.centre.x) + _sqr(at.z - p.centre.z));
            if (d > p.half_width + .6f || at.y < p.y0 - 1.f || at.y > p.y1 + 1.f) continue;
            ++claimants;
            if (d < best_d) { best_d = d; best = p.id; }
        }
    return best;
}

std::uint64_t fnv(const char* text)
{
    std::uint64_t h = 0xcbf29ce484222325ull;
    for (; text && *text; ++text) h = (h ^ std::uint8_t(*text)) * 0x100000001b3ull;
    return h;
}
std::uint32_t numeric_id(const char* text, std::uint32_t fallback)
{
    if (!text || !*text) return fallback;
    const char* digits = text;
    while (*digits && (*digits < '0' || *digits > '9')) ++digits;
    return *digits ? std::uint32_t(std::strtoul(digits, nullptr, 10)) : fallback;
}
bool number(const ::luabind::object& o, float& out)
{
    if (o.type() != LUA_TNUMBER) return false;
    out = ::luabind::object_cast<float>(o);
    return _valid(out);
}
bool vec(const ::luabind::object& o, Vec3& out)
{
    if (o.type() != LUA_TUSERDATA) return false;
    const auto value = ::luabind::object_cast_nothrow<Fvector*>(o);
    if (!value || !*value) return false;
    out = {(*value)->x, (*value)->y, (*value)->z};
    return true;
}
const char* text(const ::luabind::object& o) { return o.type() == LUA_TSTRING ? ::luabind::object_cast<LPCSTR>(o) : nullptr; }
// fn(key, value) for every entry of a table field (nothing when it is not a table)
template <class Fn> void each(::luabind::object table, Fn&& fn)
{
    if (table.type() != LUA_TTABLE) return;
    for (::luabind::object::iterator i = table.begin(), e = table.end(); i != e; ++i) fn(::luabind::object(i.key()), ::luabind::object(*i));
}

// ---- the job queue ----
class PoolJobs final : public JobQueue {
public:
    struct Shared {
        std::mutex mutex;
        std::uint64_t submitted = 0, completed = 0, cancelled = 0, rejected = 0;
        std::uint32_t depth = 0;
        std::array<float, 256> ages{};              // recent completion ages (ms), ring
        std::uint32_t ages_n = 0, ages_at = 0;
        float age_max = 0;
        std::array<float, 256> waits{};             // recent waits submit -> start (ms) of admitted jobs, ring
        std::uint32_t waits_n = 0, waits_at = 0;
        float wait_max = 0;
        float max_wait_ms = 8.f;                    // [painter_combat] job_max_wait_ms: the watchdog behind `late`
        std::uint64_t late = 0;
        std::uint64_t summary_at[5] = {};           // submitted, completed, cancelled, rejected, late at the last summary
        std::uint32_t frame = 0;
        float frame_used_ms = 0;
        std::unordered_map<JobTicket, std::shared_ptr<std::atomic<bool>>> live;
        std::atomic<std::uint64_t> epoch{0};
        std::uint32_t running = 0;                  // admitted jobs inside fn (the drain waits for these)
        std::condition_variable idle;
    };
    std::shared_ptr<Shared> shared = std::make_shared<Shared>();
    float budget_ms = 4.f;
    std::uint32_t max_depth = 256;
    std::uint32_t lane_workers = 1;                 // [painter_combat] job_workers: the pool's task lane threads
    std::atomic<std::uint32_t> lane_applied{0};
    std::atomic<JobTicket> next{1};

    JobTicket submit(std::function<void(const std::function<bool()>&)> fn, std::uint8_t priority) override
    {
        gfy_live::Pool* pool = painter_voxel_live_pool();
        if (!pool || !fn) return reject();
        if (lane_applied.load() != lane_workers) { pool->configure_task_lane(lane_workers); lane_applied.store(lane_workers); }
        auto flag = std::make_shared<std::atomic<bool>>(false);
        const JobTicket ticket = next.fetch_add(1);
        const std::uint64_t epoch = shared->epoch.load();
        {
            std::lock_guard<std::mutex> lock(shared->mutex);
            roll(*shared);
            if (shared->depth >= max_depth || shared->frame_used_ms >= budget_ms) { ++shared->rejected; return 0; }
            ++shared->submitted;
            ++shared->depth;
            shared->live.emplace(ticket, flag);
        }
        auto s = shared;
        const auto queued = std::chrono::steady_clock::now();
        const bool sent = pool->submit_task([s, flag, fn = std::move(fn), ticket, epoch, queued]() {
            // Admission and cancellation under one lock: either the drain sees this job running and waits for it, or
            // the job sees the drain's cancellation and never touches the level.
            bool admitted = false;
            {
                std::lock_guard<std::mutex> lock(s->mutex);
                admitted = !flag->load() && s->epoch.load() == epoch;
                if (admitted) ++s->running;
            }
            const auto start = std::chrono::steady_clock::now();
            if (admitted)
            {
                const std::function<bool()> cancelled = [&] { return flag->load() || s->epoch.load() != epoch; };
                fn(cancelled);                      // the core's TU catches its own errors: this one has no unwinding
            }
            const auto done = std::chrono::steady_clock::now();
            const float run_ms = std::chrono::duration<float, std::milli>(done - start).count();
            const float age_ms = std::chrono::duration<float, std::milli>(done - queued).count();
            const float wait_ms = std::chrono::duration<float, std::milli>(start - queued).count();
            {
                std::lock_guard<std::mutex> lock(s->mutex);
                roll(*s);
                s->frame_used_ms += run_ms;
                if (s->depth) --s->depth;
                s->live.erase(ticket);
                if (!admitted) { ++s->cancelled; return; }
                --s->running;
                ++s->completed;
                s->ages[s->ages_at] = age_ms;
                s->ages_at = (s->ages_at + 1) % s->ages.size();
                s->ages_n = std::min<std::uint32_t>(s->ages_n + 1, std::uint32_t(s->ages.size()));
                s->age_max = std::max(s->age_max, age_ms);
                s->waits[s->waits_at] = wait_ms;
                s->waits_at = (s->waits_at + 1) % s->waits.size();
                s->waits_n = std::min<std::uint32_t>(s->waits_n + 1, std::uint32_t(s->waits.size()));
                s->wait_max = std::max(s->wait_max, wait_ms);
                if (wait_ms > s->max_wait_ms) ++s->late;
            }
            s->idle.notify_all();
        }, priority > 0);
        if (!sent)
        {
            std::lock_guard<std::mutex> lock(shared->mutex);
            if (shared->depth) --shared->depth;
            shared->live.erase(ticket);
            --shared->submitted;
            ++shared->rejected;
            return 0;
        }
        return ticket;
    }
    void cancel(JobTicket ticket) override
    {
        std::lock_guard<std::mutex> lock(shared->mutex);
        const auto found = shared->live.find(ticket);
        if (found != shared->live.end()) found->second->store(true);
    }
    JobStats stats() const override
    {
        JobStats out;
        std::lock_guard<std::mutex> lock(shared->mutex);
        out.submitted = shared->submitted;
        out.completed = shared->completed;
        out.cancelled = shared->cancelled;
        out.rejected = shared->rejected;
        out.depth = shared->depth;
        out.late = shared->late;
        out.age_ms_p50 = quantile(shared->ages, shared->ages_n, 5);
        out.age_ms_p90 = quantile(shared->ages, shared->ages_n, 9);
        out.age_ms_max = shared->age_max;
        out.wait_ms_p90 = quantile(shared->waits, shared->waits_n, 9);
        out.wait_ms_max = shared->wait_max;
        out.frame_budget_ms = budget_ms;
        out.frame_used_ms = shared->frame_used_ms;
        return out;
    }
    // One line per level (at teardown, after the drain): this level's counts and the waits / ages of its recent jobs;
    // the windows start again for the next level. All zeros when nothing ran (a disabled run shows the path).
    void summary(const char* level)
    {
        std::lock_guard<std::mutex> lock(shared->mutex);
        Shared& s = *shared;
        const std::uint64_t now[5] = {s.submitted, s.completed, s.cancelled, s.rejected, s.late};
        Msg("[painter_combat] phase=jobs_summary level=%s submitted=%llu completed=%llu cancelled=%llu rejected=%llu "
            "wait_ms_p90=%.1f wait_ms_max=%.1f late=%llu max_wait_ms=%.0f age_ms_p90=%.1f age_ms_max=%.1f workers=%u",
            level, now[0] - s.summary_at[0], now[1] - s.summary_at[1], now[2] - s.summary_at[2], now[3] - s.summary_at[3],
            quantile(s.waits, s.waits_n, 9), s.wait_max, now[4] - s.summary_at[4], s.max_wait_ms,
            quantile(s.ages, s.ages_n, 9), s.age_max, lane_workers);
        std::copy(std::begin(now), std::end(now), s.summary_at);
        s.ages_n = s.ages_at = s.waits_n = s.waits_at = 0;
        s.age_max = s.wait_max = 0;
    }
    void cancel_all()
    {
        std::lock_guard<std::mutex> lock(shared->mutex);
        shared->epoch.fetch_add(1);
        for (auto& job : shared->live) job.second->store(true);
    }
    // Level teardown: cancel every ticket and wait until no admitted job is inside fn. Queued jobs that have not been
    // admitted stay queued and return without touching anything (their admission check fails).
    void drain()
    {
        std::unique_lock<std::mutex> lock(shared->mutex);
        shared->epoch.fetch_add(1);
        for (auto& job : shared->live) job.second->store(true);
        shared->idle.wait(lock, [&] { return shared->running == 0; });
    }

private:
    JobTicket reject()
    {
        std::lock_guard<std::mutex> lock(shared->mutex);
        ++shared->rejected;
        return 0;
    }
    static void roll(Shared& s) // a new engine frame starts a new budget
    {
        const std::uint32_t frame = Device.dwFrame;
        if (frame != s.frame) { s.frame = frame; s.frame_used_ms = 0; }
    }
    static float quantile(const std::array<float, 256>& ring, std::uint32_t n, std::uint32_t tenths)
    {
        if (!n) return 0.f;
        std::array<float, 256> sorted = ring;
        std::sort(sorted.begin(), sorted.begin() + n);
        return sorted[std::min<std::uint32_t>(n - 1, (n * tenths) / 10)];
    }
};
PoolJobs pool_jobs;
std::uint64_t jobs_epoch = 0;
} // namespace

std::uint8_t cover_query(const CoverQueryInput& in, CoverSpot* out, std::uint8_t capacity)
{
    if (!out || !capacity || !g_pGameLevel || !ai().get_level_graph()) return 0;
    const float radius = clampr(in.radius, 1.f, 60.f);
    const Fvector centre = native(in.centre);
    const CLevelGraph& graph = ai().level_graph();
    float quadrant_yaw[4];
    {
        const Fvector quadrant[4] = {{1.f, 0.f, 0.f}, {0.f, 0.f, 1.f}, {-1.f, 0.f, 0.f}, {0.f, 0.f, -1.f}};
        for (int k = 0; k < 4; ++k) { float y, p; quadrant[k].getHP(y, p); quadrant_yaw[k] = angle_normalize(y); }
    }
    const Fvector slot_direction[4] = {{-1.f, 0.f, 0.f}, {0.f, 0.f, 1.f}, {1.f, 0.f, 0.f}, {0.f, 0.f, -1.f}}; // link slots
    struct Candidate { const CCoverPoint* cover; float dc, dt, hi, lo; std::uint8_t t; float high[4], low[4]; };
    xr_vector<CCoverPoint*> covers;
    ai().cover_manager().covers().nearest(centre, radius, covers);
    xr_vector<Candidate> kept;
    kept.reserve(covers.size());
    for (const CCoverPoint* cover : covers)
    {
        const u32 vid = cover->level_vertex_id();
        if (!graph.valid_vertex_id(vid)) continue;
        const Fvector& p = cover->position();
        Candidate c{cover, p.distance_to_xz(centre), flt_max, 1.f, 1.f, 0, {}, {}};
        if (c.dc > radius) continue;
        for (int k = 0; k < 4; ++k)
        {
            c.high[k] = graph.high_cover_in_direction(quadrant_yaw[k], vid);
            c.low[k] = graph.low_cover_in_direction(quadrant_yaw[k], vid);
        }
        bool too_close = false, covered = in.threats.count == 0;
        for (std::uint8_t k = 0; k < in.threats.count; ++k)
        {
            const Vec3& t = in.threats.items[k];
            const float d = p.distance_to_xz(native(t));
            if (d < in.min_enemy_m) too_close = true;
            if (d < c.dt) { c.dt = d; c.t = k; }
            float hi, lo;
            cover_toward(c.high, c.low, t.x - p.x, t.z - p.z, hi, lo);
            covered = covered || lo <= in.max_lo || hi <= in.max_lo;
        }
        if (in.threats.count)
        {
            const Vec3& t = in.threats.items[c.t];
            cover_toward(c.high, c.low, t.x - p.x, t.z - p.z, c.hi, c.lo);
        }
        if (!too_close && covered) kept.push_back(c);
    }
    std::sort(kept.begin(), kept.end(), [](const Candidate& a, const Candidate& b) { return a.dc < b.dc; });
    const std::size_t n = std::min<std::size_t>({kept.size(), std::size_t(in.max_n), std::size_t(capacity)});
    const std::uint64_t revision = level_epoch();
    for (std::size_t i = 0; i < n; ++i)
    {
        const Candidate& c = kept[i];
        CoverSpot& s = out[i];
        s = CoverSpot{};
        const u32 vid = c.cover->level_vertex_id();
        const Fvector& p = c.cover->position();
        s.node = vid;
        s.position = {p.x, p.y, p.z};
        s.hi = c.hi;
        s.lo = c.lo;
        s.dc = c.dc;
        s.dt = in.threats.count ? c.dt : 0.f;
        s.threat = c.t;
        for (int k = 0; k < 4; ++k) { s.nibbles[k] = c.high[k]; s.nibbles[4 + k] = c.low[k]; }
        s.geometry_revision = revision;
        if (!in.threats.count) continue;
        // line of fire: from the spot at each height to the nearest threat's eye (static geometry), serial on this thread
        const Vec3& t = in.threats.items[c.t];
        const Fvector eye = Fvector().set(t.x, t.y + 1.7f, t.z);
        const std::uint8_t heights = in.heights.count ? std::min<std::uint8_t>(in.heights.count, 4) : 3;
        static const float defaults[3] = {.7f, 1.1f, 1.7f};
        for (std::uint8_t h = 0; h < heights; ++h)
        {
            const float y = in.heights.count ? in.heights.items[h] : defaults[h];
            s.blocked[h] = static_hit(Fvector().set(p.x, p.y + y, p.z), eye) >= 0.f;
        }
        s.protection_known = Evidence::yes;
        Fvector to_threat = Fvector().set(t.x - p.x, 0.f, t.z - p.z);
        int best = 0;
        for (int k = 1; k < 4; ++k)
            if (slot_direction[k].dotproduct(to_threat) > slot_direction[best].dotproduct(to_threat)) best = k;
        const u32 link = graph.vertex(vid)->link(u8(best));
        s.neighbour_toward_threat = graph.valid_vertex_id(link) ? link : invalid_node;
    }
    return std::uint8_t(n);
}

void ray_batch(const Segment* in, float* hit, std::uint32_t count)
{
    if (!in || !hit) return;
    if (!g_pGameLevel) { for (std::uint32_t i = 0; i < count; ++i) hit[i] = -1.f; return; }
    for (std::uint32_t i = 0; i < count; ++i) hit[i] = static_hit(native(in[i].a), native(in[i].b));
}

std::uint32_t project(const Vec3& p, float max_horizontal_m, float max_vertical_m, Vec3* snapped)
{
    const CLevelGraph* graph = g_pGameLevel ? ai().get_level_graph() : nullptr;
    if (!graph || !_valid(p.x) || !_valid(p.y) || !_valid(p.z) || !(max_horizontal_m >= 0.f) || !(max_vertical_m >= 0.f))
        return invalid_node;
    // a node's centre is its cell's centre (index * cell + box min): the cells within reach, one cell of slack, clipped
    const float cell = graph->header().cell_size(), reach = std::min(max_horizontal_m, 32.f), span = reach / cell + 1.f;
    const Fvector& low = graph->header().box().min;
    const float fx = (p.x - low.x) / cell, fz = (p.z - low.z) / cell;
    const int max_x = int(graph->max_x()), max_z = int(graph->max_z());
    if (fx + span < 0.f || fz + span < 0.f || fx - span > float(max_x) || fz - span > float(max_z)) return invalid_node;
    const int x0 = std::max(0, iCeil(fx - span)), x1 = std::min(max_x, iFloor(fx + span));
    const int z0 = std::max(0, iCeil(fz - span)), z1 = std::min(max_z, iFloor(fz + span));
    const u32 row = graph->row_length();
    const CLevelGraph::const_vertex_iterator B = graph->begin(), E = graph->end();
    std::uint32_t best = invalid_node;
    float best_d = flt_max;
    // the vertex array is sorted by packed xz (x * row + z), the floors of one cell next to each other: one run per x
    for (int x = x0; x <= x1 && z0 <= z1; ++x)
    {
        const u32 last = u32(x) * row + u32(z1);
        for (CLevelGraph::const_vertex_iterator I = std::lower_bound(B, E, u32(x) * row + u32(z0)); I != E && I->position().xz() <= last; ++I)
        {
            const Fvector c = graph->vertex_position(*I);
            const float dx = c.x - p.x, dy = c.y - p.y, dz = c.z - p.z, flat = dx * dx + dz * dz;
            if (flat > reach * reach || _abs(dy) > max_vertical_m) continue;
            const float d = flat + dy * dy;
            if (d < best_d) { best_d = d; best = std::uint32_t(I - B); }
        }
    }
    if (best != invalid_node && snapped) node_position(best, snapped);
    return best;
}

namespace {
// The pinned level graph as NodePath.h reads it: the mapped vertex array and header, nothing mutable
struct LevelGraphView {
    const CLevelGraph& graph;
    bool valid(std::uint32_t id) const { return graph.valid_vertex_id(id); }
    void centre(std::uint32_t id, float (&out)[3]) const
    {
        const Fvector c = graph.vertex_position(id);
        out[0] = c.x;
        out[1] = c.y;
        out[2] = c.z;
    }
    std::uint32_t link(std::uint32_t id, int k) const { return graph.vertex(id)->link(k); }
};
} // namespace

float path_length(std::uint32_t from_node, std::uint32_t to_node, std::uint32_t max_nodes)
{
    const CLevelGraph* graph = g_pGameLevel ? ai().get_level_graph() : nullptr;
    if (!graph) return -1.f;
    thread_local PathScratch scratch;               // per thread: the lane's workers and the authority thread never share it
    return node_path_length(LevelGraphView{*graph}, from_node, to_node, max_nodes, scratch);
}

StaticRouteResult query_static_route(const StaticRouteRequest& request, StaticRouteQuota& quota,
    const std::function<bool()>& cancelled)
{
    StaticRouteResult invalid; invalid.source = request;
    // Same immutable level graph used by path_length. The caller's job epoch
    // cancels lifetime changes; this function never obtains a live actor.
    const auto epoch = level_epoch();
    const CLevelGraph* graph = g_pGameLevel ? ai().get_level_graph() : nullptr;
    if (!graph || request.level_epoch != epoch || request.geometry_revision != epoch) return invalid;
    const LevelGraphView view{*graph};
    StaticRouteGraph callbacks; callbacks.context = &view;
    callbacks.valid = [](const void* context, std::uint32_t id) { return static_cast<const LevelGraphView*>(context)->valid(id); };
    callbacks.centre = [](const void* context, std::uint32_t id, float (&p)[3]) { static_cast<const LevelGraphView*>(context)->centre(id, p); };
    callbacks.link = [](const void* context, std::uint32_t id, int k) { return static_cast<const LevelGraphView*>(context)->link(id, k); };
    return build_static_route_query(callbacks, request, quota, cancelled);
}

bool node_position(std::uint32_t node, Vec3* out)
{
    const CLevelGraph* graph = g_pGameLevel ? ai().get_level_graph() : nullptr;
    if (!graph || !graph->valid_vertex_id(node)) return false;
    if (out)
    {
        const Fvector c = graph->vertex_position(node);
        *out = {c.x, c.y, c.z};
    }
    return true;
}

const BuildingCache* buildings() { return cache && cache->level_epoch == level_epoch() ? cache.get() : nullptr; }

void load_buildings()
{
    if (cache_epoch == level_epoch()) return;
    cache_epoch = level_epoch();
    cache.reset();
    node_index.clear();
    portal_names.clear();
    if (!g_pGameLevel) return;
    string256 name;
    xr_sprintf(name, "painter_ai_buildings_%s.get", *Level().name());
    ::luabind::functor<::luabind::object> get;
    ::luabind::functor<LPCSTR> map_hash;
    if (!ai().script_engine().functor(name, get) || !ai().script_engine().functor("level.painter_map_hash", map_hash)) return;
    ::luabind::object data = get();
    if (!data || data.type() != LUA_TTABLE) return;
    const char* file_hash = text(data["hash"]);
    const LPCSTR running = map_hash();
    if (!file_hash || !running || xr_strcmp(file_hash, running))
    {
        Msg("[painter_combat] phase=buildings_skipped level=%s reason=hash file=%s running=%s", *Level().name(),
            file_hash ? file_hash : "-", running ? running : "-");
        return;
    }
    CTimer timer;
    timer.Start();
    auto built = std::make_unique<BuildingCache>();
    built->map_hash = fnv(running);
    built->level_epoch = level_epoch();
    std::uint32_t building_id = 0, portal_id = 0, rooms_total = 0;
    // data v9: each portal side's packed red-light table; another version's tables are not read (counted as ignored).
    // lit_skipped: the sides the generator's cap left without a table (absent: unknown)
    float version = 0.f, skipped = -1.f;
    number(data["version"], version);
    number(data["lit_skipped"], skipped);
    built->data_version = version > 0.f ? std::uint32_t(version) : 0;
    const std::uint32_t node_count = ai().level_graph().header().vertex_count();
    std::uint32_t lit_sides = 0, lit_nodes = 0, lit_bad = 0, lit_ignored = 0;
    const auto lit = [&](const ::luabind::object& o, std::vector<Lit>& out) {
        const char* packed = text(o);
        if (!packed) return;
        if (version != 9.f) { ++lit_ignored; return; }
        if (!unpack_lit(packed, xr_strlen(packed), node_count, out)) { ++lit_bad; return; }
        ++lit_sides;
        lit_nodes += std::uint32_t(out.size());
    };
    each(data["buildings"], [&](const ::luabind::object&, const ::luabind::object& source) {
        if (source.type() != LUA_TTABLE) return;
        Building building;
        building.id = ++building_id;
        std::unordered_map<std::string, std::uint32_t> room_of;   // the data's room id -> the cache's
        each(source["rooms"], [&](const ::luabind::object&, const ::luabind::object& row) {
            if (row.type() != LUA_TTABLE) return;
            Room room;
            const char* id = text(row["id"]);
            room.id = numeric_id(id, rooms_total + 1);
            ++rooms_total;
            vec(row["centre"], room.centre);
            number(row["floor_y"], room.floor_y);
            each(row["nodes"], [&](const ::luabind::object&, const ::luabind::object& node) {
                ++room.nodes;
                if (node.type() == LUA_TNUMBER) room.node_ids.push_back(::luabind::object_cast<std::uint32_t>(node));
            });
            std::sort(room.node_ids.begin(), room.node_ids.end());
            // v2.9: the flat footprint (x0,z0)-(x1,z1), at the floor height; sectors and approach stay empty (no data yet)
            float x0 = 0, z0 = 0, x1 = 0, z1 = 0;
            if (number(row["x0"], x0) && number(row["z0"], z0) && number(row["x1"], x1) && number(row["z1"], z1))
            {
                room.min = {std::min(x0, x1), room.floor_y, std::min(z0, z1)};
                room.max = {std::max(x0, x1), room.floor_y, std::max(z0, z1)};
            }
            if (id) room_of[id] = room.id;
            building.rooms.push_back(std::move(room));
        });
        each(source["portals"], [&](const ::luabind::object&, const ::luabind::object& row) {
            if (row.type() != LUA_TTABLE) return;
            Portal portal;
            portal.id = ++portal_id;
            if (const char* data_id = text(row["id"]))
                portal_names[std::to_string(building.id) + ":" + data_id] = portal.id;
            const char* type = text(row["type"]);
            portal.doorway = !type || !xr_strcmp(type, "doorway");
            const char* a = text(row["room_a"]);
            const char* bb = text(row["room_b"]);
            const auto ra = a ? room_of.find(a) : room_of.end(), rb = bb ? room_of.find(bb) : room_of.end();
            portal.room_a = ra != room_of.end() ? ra->second : invalid_room;
            portal.room_b = rb != room_of.end() ? rb->second : invalid_room;   // "outside" -> invalid_room
            const ::luabind::object traversable = row["traversable"];
            portal.traversable = traversable.type() != LUA_TBOOLEAN || ::luabind::object_cast<bool>(traversable);
            const ::luabind::object stair = row["stair"];
            portal.stair = stair.type() == LUA_TBOOLEAN && ::luabind::object_cast<bool>(stair);
            vec(row["centre"], portal.centre);
            vec(row["normal"], portal.normal);
            vec(row["jamb_a"], portal.jamb_a);
            vec(row["jamb_b"], portal.jamb_b);
            number(row["half_width"], portal.half_width);
            number(row["wall_depth"], portal.wall_depth);
            number(row["y0"], portal.y0);
            number(row["y1"], portal.y1);
            lit(row["lit_a"], portal.lit_a);
            lit(row["lit_b"], portal.lit_b);
            for (auto& room : building.rooms)
                if (room.id == portal.room_a || room.id == portal.room_b) room.portals.push_back(portal.id);
            building.portals.push_back(std::move(portal));
        });
        each(source["node_room"], [&](const ::luabind::object& key, const ::luabind::object& value) {
            const char* id = text(value);
            if (key.type() != LUA_TNUMBER || !id) return;
            const auto room = room_of.find(id);
            if (room != room_of.end())
                node_index.emplace(::luabind::object_cast<std::uint32_t>(key), std::make_pair(building.id, room->second));
        });
        built->buildings.push_back(std::move(building));
    });
    // doors online now: each doorway portal takes the nearest door within its reach; one door, one portal
    std::unordered_map<std::uint16_t, std::pair<std::uint32_t, float>> claimed;
    for (auto& b : built->buildings)
        for (auto& p : b.portals)
        {
            if (!p.doorway) continue;
            doors::doors_type found;
            ai().doors().nearest(native(p.centre), p.half_width + .6f, found);
            float best_d = flt_max;
            const doors::door* best = nullptr;
            for (const doors::door* d : found)
            {
                const Fvector& at = d->object().Position();
                const float dist = _sqrt(_sqr(at.x - p.centre.x) + _sqr(at.z - p.centre.z));
                if (dist <= p.half_width + .6f && dist < best_d) { best_d = dist; best = d; }
            }
            if (!best) continue;
            const std::uint16_t id = best->object().ID();
            const auto other = claimed.find(id);
            if (other != claimed.end())
            {
                Msg("[painter_combat] phase=door_portal_conflict door=%u portals=%u,%u kept=%u", id, other->second.first, p.id,
                    best_d < other->second.second ? p.id : other->second.first);
                if (best_d >= other->second.second) continue;
                for (auto& bb : built->buildings)
                    for (auto& pp : bb.portals)
                        if (pp.id == other->second.first) pp.door = invalid_id;
            }
            claimed[id] = {p.id, best_d};
            p.door = id;
        }
    door_to_portal.clear();
    for (const auto& c : claimed) door_to_portal[c.first] = c.second.first;
    built->locate = [](std::uint32_t node) {
        const auto found = node_index.find(node);
        return found == node_index.end() ? std::make_pair(invalid_room, invalid_room) : found->second;
    };
    // every side decoded and none skipped: an empty table is dark from every far-side eye (the core skips its rays)
    built->lit_complete = version == 9.f && skipped == 0.f && !lit_bad;
    Msg("[painter_combat] phase=buildings_loaded level=%s version=%d buildings=%u rooms=%u portals=%u nodes=%u doors=%u "
        "lit_sides=%u lit_nodes=%u lit_bad=%u lit_ignored=%u lit_complete=%u ms=%.0f", *Level().name(), int(version), building_id,
        rooms_total, portal_id, unsigned(node_index.size()), unsigned(door_to_portal.size()), lit_sides, lit_nodes, lit_bad,
        lit_ignored, unsigned(built->lit_complete), timer.GetElapsed_sec() * 1000.f);
    cache = std::move(built);
}

JobQueue& jobs()
{
    if (jobs_epoch != level_epoch()) { jobs_epoch = level_epoch(); pool_jobs.cancel_all(); }
    return pool_jobs;
}
void configure_jobs(float frame_budget_ms, std::uint32_t max_depth, std::uint32_t workers, float max_wait_ms)
{
    pool_jobs.budget_ms = clampr(frame_budget_ms, .1f, 50.f);
    pool_jobs.max_depth = std::max<std::uint32_t>(1, max_depth);
    pool_jobs.lane_workers = clampr<std::uint32_t>(workers, 1, 4);
    std::lock_guard<std::mutex> lock(pool_jobs.shared->mutex);
    pool_jobs.shared->max_wait_ms = clampr(max_wait_ms, .1f, 10000.f);
}

EngineServices services()
{
    EngineServices s;
    s.jobs = &jobs();
    s.cover_query = &cover_query;
    s.rays = &ray_batch;
    s.buildings = buildings();
    s.project = &project;
    s.node_position = &node_position;
    s.path_length = &path_length;
    return s;
}


std::uint32_t portal_named(std::uint32_t building, const char* data_id)
{
    if (!data_id) return 0;
    const auto found = portal_names.find(std::to_string(building) + ":" + data_id);
    return found == portal_names.end() ? 0 : found->second;
}
std::uint32_t room_named(const char* data_id) { return numeric_id(data_id, invalid_room); }

std::uint32_t door_portal(std::uint16_t door, const Vec3& at)
{
    const auto known = door_to_portal.find(door);
    if (known != door_to_portal.end()) return known->second;
    const BuildingCache* c = buildings();
    if (!c) return 0;                               // no building data: not cached, matched once data exists
    unsigned claimants = 0;
    const std::uint32_t portal = match_portal(*c, native(at), claimants);
    if (!portal) Msg("[painter_combat] phase=door_without_portal door=%u at=%.1f,%.1f,%.1f", door, at.x, at.y, at.z);
    else if (claimants > 1) Msg("[painter_combat] phase=door_portal_ambiguous door=%u portals=%u kept=%u", door, claimants, portal);
    door_to_portal[door] = portal;
    return portal;
}

namespace {
// v2.14 [painter_combat_factions]: <community> = <profile> <doctrine> <spread_m> <morale> <leave_wounded 0|1> <comms>
// (FactionStyleParse.h); the community string becomes the index capture writes into ActorSnapshot::faction. A bad line or
// an unknown community is named and skipped.
void read_factions(Config& c)
{
    LPCSTR const section = "painter_combat_factions";
    if (!pSettings->section_exist(section)) return;
    for (const auto& line : pSettings->r_section(section).Data)
    {
        LPCSTR community = *line.first;
        LPCSTR text = line.second.size() ? *line.second : "";
        if (c.factions.count >= c.factions.items.size())
        {
            Msg("! [painter_combat] phase=settings_faction community=%s skipped=full (%u kept)", community,
                unsigned(c.factions.items.size()));
            continue;
        }
        const CHARACTER_COMMUNITY_INDEX index = CHARACTER_COMMUNITY::IdToIndex(line.first, NO_COMMUNITY_INDEX, true);
        if (index == NO_COMMUNITY_INDEX)
        {
            Msg("! [painter_combat] phase=settings_faction community=%s skipped=unknown_community", community);
            continue;
        }
        FactionStyle style;
        if (!parse_faction_style(text, style))
        {
            Msg("! [painter_combat] phase=settings_faction community=%s skipped=bad_line line=\"%s\"", community, text);
            continue;
        }
        style.faction = std::uint32_t(index);
        c.factions.items[c.factions.count++] = style;
    }
}
} // namespace

void configure_from_settings()
{
    LPCSTR const section = "painter_combat";
    const bool present = !!pSettings->section_exist(section);
    const auto has = [&](LPCSTR key) { return present && pSettings->line_exist(section, key); };
    combat_on = has("enabled") && !!pSettings->r_bool(section, "enabled");
    Config c;                                        // the struct's defaults unless the file names the field
    const auto ms = [&](LPCSTR key, TimeMs& v) { if (!has(key)) return false; v = TimeMs(pSettings->r_u32(section, key)); return true; };
    const auto real = [&](LPCSTR key, float& v) { if (!has(key)) return false; v = pSettings->r_float(section, key); return true; };
    ms("first_order_deadline_ms", c.first_order_deadline_ms);
    ms("order_lifetime_ms", c.order_lifetime_ms);
    ms("stale_release_ms", c.stale_release_ms);
    ms("snapshot_max_age_ms", c.snapshot_max_age_ms);
    ms("memory_max_age_ms", c.memory_max_age_ms);
    ms("refresh_ms", c.refresh_ms);                  // v2.10 pacing
    ms("cover_cache_ms", c.cover_cache_ms);
    ms("plan_interval_ms", c.plan_interval_ms);
    // v2.24 (2026-10-04): the six named constants from the first live runs, keys of the same names (standard reader)
    ms("posture_dwell_ms", c.posture_dwell_ms);
    ms("unseen_reaction_source_ms", c.unseen_reaction_source_ms);
    ms("unseen_reaction_hold_ms", c.unseen_reaction_hold_ms);
    real("lifecycle_release_range_m", c.lifecycle_release_range_m);
    real("lifecycle_distance_cost_per_m", c.lifecycle_distance_cost_per_m);
    ms("lifecycle_required_door_stuck_ms", c.lifecycle_required_door_stuck_ms);
    // native17: engine-local keys (not Config): the live recorder and the look smoothing (EngineTuning.h)
    {
        const bool record = has("replay_record") && !!pSettings->r_bool(section, "replay_record");
        const u32 max_mb = has("replay_record_max_mb") ? pSettings->r_u32(section, "replay_record_max_mb") : 200;
        recorder_configure(record, max_mb);
        real("painter_look_head_rate_deg", g_painter_look_head_rate_deg);
        real("painter_look_body_rate_deg", g_painter_look_body_rate_deg);
        real("painter_look_dead_zone_deg", g_painter_look_dead_zone_deg);
        g_painter_look_head_rate_deg = clampr(g_painter_look_head_rate_deg, 30.f, 720.f);
        g_painter_look_body_rate_deg = clampr(g_painter_look_body_rate_deg, 30.f, 720.f);
        g_painter_look_dead_zone_deg = clampr(g_painter_look_dead_zone_deg, 0.f, 10.f);
        Msg("[painter_combat] phase=engine_tuning replay_record=%u replay_record_max_mb=%u look_head_rate=%.0f look_body_rate=%.0f look_dead_zone=%.1f",
            unsigned(record), max_mb, g_painter_look_head_rate_deg, g_painter_look_body_rate_deg, g_painter_look_dead_zone_deg);
    }
    const auto count16 = [&](LPCSTR key, std::uint16_t& v) { if (!has(key)) return false; v = std::uint16_t(std::min<u32>(65535, pSettings->r_u32(section, key))); return true; };
    count16("bootstrap_work_per_tick", c.bootstrap_work_per_tick);
    count16("cover_queries_per_slice", c.cover_queries_per_slice);
    ms("fire_window_ms", c.fire_window_ms);          // v2.11 timings and cover cost weights
    ms("compat_poll_ms", c.compat_poll_ms);
    if (has("bootstrap_deadline_us")) c.bootstrap_deadline_us = pSettings->r_u32(section, "bootstrap_deadline_us");
    real("cover_exposure_cost_m", c.cover_exposure_cost_m);
    real("cover_unknown_cost_m", c.cover_unknown_cost_m);
    real("cover_move_hysteresis_m", c.cover_move_hysteresis_m);
    real("suppressed_exposure_cost_m", c.suppressed_exposure_cost_m);   // v2.12
    real("cover_far_cost_m_per_m", c.cover_far_cost_m_per_m);
    ms("readmit_delay_ms", c.readmit_delay_ms);
    real("lean_eye_offset_m_stand", c.lean_eye_offset_m_stand);     // v2.13, the measured engine lean
    real("lean_eye_offset_m_crouch", c.lean_eye_offset_m_crouch);
    real("nearby_group_distance", c.nearby_group_distance);
    real("regroup_separation_distance", c.regroup_separation_distance);
    real("link_up_max_distance", c.link_up_max_distance);
    real("minimum_goal_separation", c.minimum_goal_separation);
    real("immediate_cover_distance", c.immediate_cover_distance);
    real("cover_radius", c.cover_radius);
    if (has("cover_candidates")) c.cover_candidates = std::uint8_t(std::min<u32>(255, pSettings->r_u32(section, "cover_candidates")));
    if (has("seed")) c.seed = pSettings->r_u32(section, "seed");
    real("near_miss_m", c.near_miss_m);              // v2.14: near misses and the side's comms
    real("voice_m", c.voice_m);
    real("voice_p", c.voice_p);
    read_factions(c);

    // v2.15: the knowledge layer and the room doctrine, read as knowledge_<field> / room_<field>: the key is the field's
    // own name (stringized below), so a renamed field can never be read under an old key
    const auto count32 = [&](LPCSTR key, std::uint32_t& v) { if (!has(key)) return false; v = pSettings->r_u32(section, key); return true; };
    const auto count8 = [&](LPCSTR key, std::uint8_t& v) { if (!has(key)) return false; v = std::uint8_t(std::min<u32>(255, pSettings->r_u32(section, key))); return true; };
    const auto list = [&](LPCSTR key, auto& v) {     // a std::array<float, N> as a comma list
        if (!has(key)) return false;
        LPCSTR text = pSettings->r_string(section, key);
        const int items = _GetItemCount(text);
        if (items != int(v.size()))
            Msg("! [painter_combat] phase=settings_list key=%s items=%d expected=%u (the defaults kept past the list)", key,
                items, unsigned(v.size()));
        string64 item;
        for (int i = 0; i < items && i < int(v.size()); ++i) v[i] = float(atof(_GetItem(text, i, item)));
        return true;
    };
    xr_vector<shared_str> known;                     // every knowledge_ / room_ key this build reads
    u32 knowledge_keys = 0, room_keys = 0;
    const auto note = [&](LPCSTR key, bool found) { known.push_back(key); return found ? 1u : 0u; };
#define PAINTER_KNOWLEDGE(read, field) knowledge_keys += note("knowledge_" #field, read("knowledge_" #field, c.knowledge.field))
#define PAINTER_ROOM(read, field) room_keys += note("room_" #field, read("room_" #field, c.room.field))
    PAINTER_KNOWLEDGE(ms, forget_ms);
    PAINTER_KNOWLEDGE(ms, search_forget_ms);
    PAINTER_KNOWLEDGE(ms, contact_keep_ms);
    PAINTER_KNOWLEDGE(ms, shot_at_me_ms);
    PAINTER_KNOWLEDGE(ms, shot_me_ms);
    PAINTER_KNOWLEDGE(ms, reloading_ms);
    PAINTER_KNOWLEDGE(ms, suppressed_ms);
    PAINTER_KNOWLEDGE(ms, ghost_keep_ms);
    PAINTER_KNOWLEDGE(ms, ghost_dying_ms);
    PAINTER_KNOWLEDGE(ms, uf_dir_ms);
    PAINTER_KNOWLEDGE(ms, uf_escape_ms);
    PAINTER_KNOWLEDGE(ms, uf_run_ms);
    PAINTER_KNOWLEDGE(ms, heard_report_ms);
    PAINTER_KNOWLEDGE(ms, shot_me_pick_ms);
    PAINTER_KNOWLEDGE(ms, squad_seen_ms);
    PAINTER_KNOWLEDGE(ms, assess_foe_ms);
    PAINTER_KNOWLEDGE(ms, dog_seen_ms);
    PAINTER_KNOWLEDGE(ms, dog_shotme_ms);
    PAINTER_KNOWLEDGE(ms, dog_unseen_ms);
    PAINTER_KNOWLEDGE(ms, dog_nosol_cool_ms);
    PAINTER_KNOWLEDGE(ms, peace_ms);
    PAINTER_KNOWLEDGE(ms, input_age_ms);
    PAINTER_KNOWLEDGE(count32, latency_ticks);
    PAINTER_KNOWLEDGE(real, jitter_min);
    PAINTER_KNOWLEDGE(real, jitter_max);
    PAINTER_KNOWLEDGE(real, hit_cone_deg);
    PAINTER_KNOWLEDGE(real, hit_cone_close_deg);
    PAINTER_KNOWLEDGE(real, hit_cone_close_m);       // v2.19
    PAINTER_KNOWLEDGE(real, ghost_cone_deg);
    PAINTER_KNOWLEDGE(real, ghost_merge_deg);
    PAINTER_KNOWLEDGE(real, ghost_merge_min_m);
    PAINTER_KNOWLEDGE(real, ghost_merge_range_k);
    PAINTER_KNOWLEDGE(real, hit_default_m);
    PAINTER_KNOWLEDGE(real, hit_min_m);
    PAINTER_KNOWLEDGE(real, hit_back_m);
    PAINTER_KNOWLEDGE(real, probe_fraction);
    PAINTER_KNOWLEDGE(real, probe_min_m);
    PAINTER_KNOWLEDGE(list, ghost_probe_m);
    PAINTER_KNOWLEDGE(real, red_source_cone_deg);
    PAINTER_KNOWLEDGE(real, red_source_min_m);
    PAINTER_KNOWLEDGE(real, red_source_max_m);
    PAINTER_KNOWLEDGE(real, red_floor_m);
    PAINTER_KNOWLEDGE(real, red_angle_cost);
    PAINTER_KNOWLEDGE(real, red_distance_cost);
    PAINTER_KNOWLEDGE(real, eye_m);
    PAINTER_KNOWLEDGE(real, disp_shot);
    PAINTER_KNOWLEDGE(real, disp_other);
    PAINTER_KNOWLEDGE(real, disp_cap_m);
    PAINTER_KNOWLEDGE(real, heard_near_m);
    PAINTER_KNOWLEDGE(real, clear_heard_m);
    PAINTER_KNOWLEDGE(real, gunfire_max_m);
    PAINTER_KNOWLEDGE(real, meter_max);
    PAINTER_KNOWLEDGE(real, meter_decay_per_second);
    PAINTER_KNOWLEDGE(real, meter_hit);
    PAINTER_KNOWLEDGE(real, caliber);
    PAINTER_KNOWLEDGE(real, proxy_shot);
    PAINTER_KNOWLEDGE(list, meter_thresholds);
    PAINTER_KNOWLEDGE(real, miss_near_m);
    PAINTER_KNOWLEDGE(real, miss_full_m);
    PAINTER_KNOWLEDGE(real, miss_end_m);
    PAINTER_KNOWLEDGE(real, miss_near_factor);
    PAINTER_KNOWLEDGE(real, keep_target_k);
    PAINTER_KNOWLEDGE(real, engaged_near_m);
    PAINTER_KNOWLEDGE(real, dog_start_m);
    PAINTER_KNOWLEDGE(real, dog_end_m);
    PAINTER_KNOWLEDGE(real, dog_ammo);
    PAINTER_KNOWLEDGE(real, project_horizontal_m);
    PAINTER_KNOWLEDGE(real, project_vertical_m);
    PAINTER_KNOWLEDGE(count16, contacts_cap);
    PAINTER_KNOWLEDGE(count16, event_history_cap);
    PAINTER_KNOWLEDGE(count16, hit_log_cap);
    PAINTER_KNOWLEDGE(count16, outbox_cap);
    PAINTER_ROOM(real, pitch_m);
    PAINTER_ROOM(real, spacing_m);
    PAINTER_ROOM(real, jamb_offset_m);
    PAINTER_ROOM(real, wall_offset_m);
    PAINTER_ROOM(real, arrival_m);
    PAINTER_ROOM(real, projection_m);
    PAINTER_ROOM(real, floor_tolerance_m);
    PAINTER_ROOM(real, exposure_cost_m);
    PAINTER_ROOM(real, unknown_cost_m);
    PAINTER_ROOM(real, funnel_cost_m);
    PAINTER_ROOM(real, path_unknown_cost_m);
    PAINTER_ROOM(real, approach_weight);
    PAINTER_ROOM(real, alternate_door_cost_m);
    PAINTER_ROOM(real, straggler_cost_per_second);
    PAINTER_ROOM(real, readiness_missing_cost);
    PAINTER_ROOM(real, two_door_gain_m);
    PAINTER_ROOM(real, post_distance_m);
    PAINTER_ROOM(ms, straggler_ms);
    PAINTER_ROOM(ms, swap_ms);
    PAINTER_ROOM(real, look_height_m);
    PAINTER_ROOM(count32, path_nodes);
    PAINTER_ROOM(count16, alternative_slots);
    PAINTER_ROOM(count8, minimum_door_members);
    // v2.17: the R2 room doctrine (pie, entry, search, assess, casualty, frag, defend, withdraw)
    PAINTER_ROOM(real, pie_radius_m);
    PAINTER_ROOM(real, pie_advance_m);
    PAINTER_ROOM(real, pie_lean);
    PAINTER_ROOM(real, edge_lean);
    PAINTER_ROOM(ms, pie_dwell_ms);
    PAINTER_ROOM(ms, slice_dwell_ms);
    PAINTER_ROOM(ms, pie_cap_ms);
    PAINTER_ROOM(real, form_tolerance_m);
    PAINTER_ROOM(ms, form_dwell_ms);
    PAINTER_ROOM(ms, leader_idle_ms);
    PAINTER_ROOM(ms, leader_no_shot_ms);
    PAINTER_ROOM(ms, leader_go_wait_ms);
    PAINTER_ROOM(ms, leader_up_wait_ms);
    PAINTER_ROOM(real, leader_progress_m);
    PAINTER_ROOM(real, leader_heel_m);
    PAINTER_ROOM(real, leader_point_m);
    PAINTER_ROOM(real, leader_set_m);
    PAINTER_ROOM(real, pie_engage_hysteresis);
    PAINTER_ROOM(real, pie_unaware_deg);
    PAINTER_ROOM(real, pie_partly_deg);
    PAINTER_ROOM(real, pie_near_m);
    PAINTER_ROOM(real, pie_mid_m);
    PAINTER_ROOM(real, pie_engage_unaware);
    PAINTER_ROOM(real, pie_engage_partly);
    PAINTER_ROOM(real, pie_engage_aware);
    PAINTER_ROOM(real, pie_engage_solo);
    PAINTER_ROOM(real, pie_engage_duo);
    PAINTER_ROOM(real, pie_engage_team);
    PAINTER_ROOM(real, pie_engage_near);
    PAINTER_ROOM(real, pie_engage_mid);
    PAINTER_ROOM(real, pie_engage_far);
    PAINTER_ROOM(real, pie_engage_unseen);
    PAINTER_ROOM(real, pie_call_hysteresis);
    PAINTER_ROOM(ms, pie_overwatch_cap_ms);
    PAINTER_ROOM(real, pie_call_men);
    PAINTER_ROOM(real, pie_call_near);
    PAINTER_ROOM(real, pie_call_mid);
    PAINTER_ROOM(real, pie_call_far);
    PAINTER_ROOM(real, pie_call_surprise);
    PAINTER_ROOM(real, pie_call_alert);
    PAINTER_ROOM(real, pie_call_unseen);
    PAINTER_ROOM(real, pie_call_unseen_over);
    PAINTER_ROOM(real, pie_call_door);
    PAINTER_ROOM(real, pie_call_casualty);
    PAINTER_ROOM(real, pie_call_half_line);
    PAINTER_ROOM(real, pie_call_line);
    PAINTER_ROOM(ms, pie_call_door_ms);
    PAINTER_ROOM(ms, release_gap_ms);
    PAINTER_ROOM(real, entry_arrival_m);
    PAINTER_ROOM(real, entry_follow_m);
    PAINTER_ROOM(real, entry_heel_m);
    PAINTER_ROOM(real, entry_behind_m);
    PAINTER_ROOM(real, entry_third_m);
    PAINTER_ROOM(real, entry_ready_m);
    PAINTER_ROOM(real, entry_settle_m);
    PAINTER_ROOM(ms, entry_stall_ms);
    PAINTER_ROOM(ms, entry_reload_cap_ms);
    PAINTER_ROOM(ms, entry_flow_cap_ms);
    PAINTER_ROOM(real, entry_zone_half_m);
    PAINTER_ROOM(real, entry_zone_depth_m);
    PAINTER_ROOM(real, entry_lane_half_m);
    PAINTER_ROOM(real, entry_lane_depth_m);
    PAINTER_ROOM(count8, entry_pace);
    PAINTER_ROOM(real, entry_walk_scale);
    PAINTER_ROOM(real, entry_run_scale);
    PAINTER_ROOM(real, jamb_final_m);
    PAINTER_ROOM(real, jamb_speed_scale);
    PAINTER_ROOM(ms, door_hot_ms);
    PAINTER_ROOM(real, door_hot_cost);
    PAINTER_ROOM(ms, crossing_gap_ms);
    PAINTER_ROOM(ms, crossing_block_cap_ms);
    PAINTER_ROOM(real, denial_step_m);
    PAINTER_ROOM(real, exposed_correction_max_m);
    PAINTER_ROOM(ms, return_fire_ms);
    PAINTER_ROOM(ms, return_fire_overlap_ms);
    PAINTER_ROOM(ms, return_fire_recent_shot_ms);
    PAINTER_ROOM(count16, suppression_queue);
    PAINTER_ROOM(ms, suppression_interval_ms);
    PAINTER_ROOM(real, suppression_ammo_floor);
    PAINTER_ROOM(real, see_range_m);
    PAINTER_ROOM(real, check_half_deg);
    PAINTER_ROOM(ms, seen_ms);
    PAINTER_ROOM(ms, seen_red_ms);
    PAINTER_ROOM(ms, sweep_step_ms);
    PAINTER_ROOM(ms, search_after_ms);
    PAINTER_ROOM(real, known_enemy_exclusion_m);
    PAINTER_ROOM(real, known_enemy_search_m);
    PAINTER_ROOM(ms, known_enemy_ms);
    PAINTER_ROOM(list, finisher_rings_m);
    PAINTER_ROOM(count8, finisher_tries);
    PAINTER_ROOM(ms, finisher_try_ms);
    PAINTER_ROOM(ms, finisher_cap_ms);
    PAINTER_ROOM(ms, finisher_skip_ms);
    PAINTER_ROOM(ms, finisher_quiet_ms);
    PAINTER_ROOM(ms, finisher_keep_ms);
    PAINTER_ROOM(ms, assess_cap_ms);
    PAINTER_ROOM(ms, assess_crossing_grace_ms);
    PAINTER_ROOM(ms, assess_fresh_hit_ms);
    PAINTER_ROOM(ms, assess_slice_ms);
    PAINTER_ROOM(real, assess_radius_m);
    PAINTER_ROOM(list, assess_angles_deg);
    PAINTER_ROOM(list, assess_radii_m);
    PAINTER_ROOM(ms, aid_hit_quiet_ms);
    PAINTER_ROOM(ms, aid_beam_ok_ms);
    PAINTER_ROOM(ms, aid_cap_ms);
    PAINTER_ROOM(ms, aid_work_ms);
    PAINTER_ROOM(real, aid_reach_m);
    PAINTER_ROOM(real, aid_arrival_m);
    PAINTER_ROOM(real, aid_min_body_m);
    PAINTER_ROOM(list, aid_placement_radii_m);
    PAINTER_ROOM(list, aid_placement_angles_rad);
    PAINTER_ROOM(ms, casualty_lane_block_cap_ms);
    PAINTER_ROOM(real, casualty_lane_block_cost);
    PAINTER_ROOM(real, casualty_lane_width_margin_m);
    PAINTER_ROOM(real, casualty_lane_depth_margin_m);
    PAINTER_ROOM(real, casualty_lane_vertical_m);
    PAINTER_ROOM(ms, buddy_answer_ms);
    PAINTER_ROOM(ms, frag_fuse_ms);
    PAINTER_ROOM(ms, frag_prefire_cap_ms);
    PAINTER_ROOM(ms, frag_hand_cap_ms);
    PAINTER_ROOM(ms, frag_arc_wait_ms);
    PAINTER_ROOM(ms, frag_release_cap_ms);
    PAINTER_ROOM(ms, frag_receipt_ms);
    PAINTER_ROOM(list, frag_inside_m);
    PAINTER_ROOM(real, frag_min_m);
    PAINTER_ROOM(real, frag_max_m);
    PAINTER_ROOM(real, frag_door_height_m);
    PAINTER_ROOM(real, frag_reposition_step_m);
    PAINTER_ROOM(ms, frag_reposition_cap_ms);
    PAINTER_ROOM(ms, frag_arc_poll_ms);
    PAINTER_ROOM(ms, defend_quiet_ms);
    PAINTER_ROOM(ms, withdraw_cap_ms);
    PAINTER_ROOM(ms, withdraw_quiet_ms);
    PAINTER_ROOM(ms, lone_hold_ms);
    PAINTER_ROOM(count8, withdraw_break_hits);
    PAINTER_ROOM(ms, withdraw_break_window_ms);
    PAINTER_ROOM(ms, withdraw_foe_ms);
    PAINTER_ROOM(ms, withdraw_kill_ms);
    PAINTER_ROOM(ms, withdraw_trend_ms);
    PAINTER_ROOM(ms, withdraw_persist_ms);
    PAINTER_ROOM(ms, withdraw_cooldown_ms);
    PAINTER_ROOM(ms, withdraw_threat_ms);
    PAINTER_ROOM(ms, withdraw_resume_quiet_ms);
    PAINTER_ROOM(real, withdraw_unsafe_m);
    PAINTER_ROOM(real, withdraw_health_mean);
    PAINTER_ROOM(real, withdraw_able_fraction);
    PAINTER_ROOM(real, withdraw_casualty_rally_m);
    PAINTER_ROOM(ms, withdraw_peel_inside_ms);
    PAINTER_ROOM(ms, withdraw_peel_door_ms);
    PAINTER_ROOM(real, backout_min_m);
    PAINTER_ROOM(real, backout_max_m);
    PAINTER_ROOM(real, backstep_enemy_m);
    PAINTER_ROOM(real, backstep_m);
    PAINTER_ROOM(ms, backout_cap_ms);
    PAINTER_ROOM(ms, backout_unseen_ms);
    PAINTER_ROOM(ms, backstep_ms);
    // v2.20: the stack's last compiled-in numbers as data
    PAINTER_ROOM(count8, rear_from_slot);
    PAINTER_ROOM(count8, watch_post_min_men);
    PAINTER_ROOM(real, door_locked_cost_k);
    PAINTER_ROOM(real, door_held_cost_k);
#undef PAINTER_KNOWLEDGE
#undef PAINTER_ROOM
    count8("compat_mode", c.compat_mode);            // v2.21: 0 bridge (poll + pushes, the default), 1 push-only
    // v2.23: the outdoor layers' tunables into Config::outdoor (OutdoorSettingsRead.h; the runtime objects are made from it by
    // make_outdoor_policies); missing / malformed keys keep the OutdoorConfig values and are named, never all-or-nothing
    OutdoorIntake outdoor;
    read_outdoor_config([&](const char* key) -> const char* {
        if (!has(key)) return nullptr;
        LPCSTR text = pSettings->r_string(section, key);
        return text ? text : "";
    }, c.outdoor, outdoor);
    Msg("[painter_combat] phase=outdoor_settings keys_read=%u missing=%u bad=%u layers_off=%s reverted=%s", outdoor.read,
        outdoor.missing, outdoor.bad, outdoor.layers_off.empty() ? "none" : outdoor.layers_off.c_str(),
        outdoor.reverted.empty() ? "none" : outdoor.reverted.c_str());
    if (outdoor.missing)
        Msg("! [painter_combat] phase=outdoor_settings_missing count=%u keys=%s (the OutdoorConfig values kept)", outdoor.missing,
            outdoor.missing_keys.c_str());
    if (outdoor.bad)
        Msg("! [painter_combat] phase=outdoor_settings_bad count=%u keys=%s (malformed or out of range; the OutdoorConfig values "
            "kept)", outdoor.bad, outdoor.bad_keys.c_str());
    if (pSettings->section_exist("painter_combat_outdoor"))
        Msg("! [painter_combat] phase=outdoor_settings retired_section=painter_combat_outdoor (ignored: the keys are "
            "outdoor_<name> in [painter_combat])");
    for (const char* name : outdoor_setting_names)
        known.push_back(name);
    // a knowledge_ / room_ / outdoor_ key this build does not know (a typo, or a field of a later contract) is named, never silent
    if (present)
        for (const auto& line : pSettings->r_section(section).Data)
        {
            LPCSTR key = *line.first;
            if (!key || (strncmp(key, "knowledge_", 10) && strncmp(key, "room_", 5) && strncmp(key, "outdoor_", 8))) continue;
            if (std::find(known.begin(), known.end(), line.first) == known.end())
                Msg("! [painter_combat] phase=settings_unknown_key key=%s (not a field of this build's contract)", key);
        }
    combat_cfg = c;
    float budget = 4.f, max_wait = 8.f;
    u32 depth = 256, workers = 1;
    if (has("job_budget_ms")) budget = pSettings->r_float(section, "job_budget_ms");
    if (has("job_max_depth")) depth = pSettings->r_u32(section, "job_max_depth");
    if (has("job_workers")) workers = pSettings->r_u32(section, "job_workers");
    if (has("job_max_wait_ms")) max_wait = pSettings->r_float(section, "job_max_wait_ms");
    configure_jobs(budget, depth, workers, max_wait);
    Msg("[painter_combat] phase=settings enabled=%u job_budget_ms=%.1f job_max_depth=%u job_workers=%u job_max_wait_ms=%.1f "
        "cover_radius=%.0f cover_candidates=%u near_miss_m=%.1f factions=%u knowledge_keys=%u room_keys=%u compat_mode=%u",
        unsigned(combat_on), budget, depth, workers, max_wait, c.cover_radius, unsigned(c.cover_candidates), c.near_miss_m,
        unsigned(c.factions.count), knowledge_keys, room_keys, unsigned(c.compat_mode));
}
bool combat_enabled() { return combat_on; }
const Config& combat_config() { return combat_cfg; }

void FrameTask::run()
{
    if (!g_pGameLevel || !tick_fn) return;
    if (seen_serial != level_serial)
    {
        seen_serial = level_serial;
        configure_from_settings();
        ticked_on = false;
    }
    if (combat_on)
    {
        load_buildings();                           // once per level epoch (returns at once afterwards)
        tick_fn(now_ms());
        ticked_on = true;
    }
    else if (ticked_on)
    {
        // the master switch went off in play: one tick in the disabled state, whose master-off branch releases the
        // men the runtime owns; inert afterwards (the Lua adapter keeps its men)
        ticked_on = false;
        tick_fn(now_ms());
    }
}
FrameTask& frame_task()
{
    static FrameTask task;
    return task;
}
void level_created()
{
    ++level_serial;
    door_to_portal.clear();
    on_level_load();                                // new epoch, script flags dropped (before the level's scripts run)
}
void set_tick(void (*tick)(TimeMs now)) { tick_fn = tick; }
void level_destroyed()
{
    // Lifetime guarantee (CombatContract.h): no job outlives its epoch; the cache goes only after the drain.
    pool_jobs.drain();
    pool_jobs.summary(g_pGameLevel ? *Level().name() : "-");
    near_miss_summary(g_pGameLevel ? *Level().name() : "-");
    compat_summary(g_pGameLevel ? *Level().name() : "-");
    jobs_epoch = 0;
    cache.reset();
    node_index.clear();
    portal_names.clear();
    cache_epoch = 0;
    door_to_portal.clear();
}

}}} // namespace painter::combat::engine
