#pragma once
#include <array>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>
#include "CombatIds.h"
#include "OutdoorConfig.h"

namespace painter { namespace combat {

using TimeMs = std::uint64_t;                    // level time (Device.dwTimeGlobal widened; wrap tracked by the engine)
constexpr std::uint32_t invalid_node = 0xffffffffu;
constexpr std::uint32_t invalid_room = 0xffffffffu;
constexpr std::uint16_t invalid_id = 0xffffu;

struct Vec3 { float x = 0, y = 0, z = 0; };
struct ActorKey {                                // object id + spawn generation: an id is never trusted alone
    std::uint32_t id = invalid_id;
    std::uint64_t generation = 0;
    bool operator==(const ActorKey& b) const { return id == b.id && generation == b.generation; }
    bool operator!=(const ActorKey& b) const { return !(*this == b); }
};
template <class T, std::size_t N> struct Bounded { std::array<T, N> items{}; std::uint8_t count = 0; };

// ---- snapshot (engine -> core) ----
enum class WeaponKind : std::uint8_t { none, pistol, smg, rifle, shotgun, sniper, mg, launcher, melee };
enum class WeaponState : std::uint8_t { idle, firing, reloading, switching, misfire, empty };
enum class SoundKind : std::uint8_t { gunshot, explosion, footstep, weapon, item, voice, injury, other };
// v2.14 note (the capture's mapping, EngineCapture.cpp): hit = the engine's bullet_ricochet, Danger::position = the IMPACT
// point of a round (where it struck the world); attack = entity_attacked / attacked; enemy_sound = attack_sound /
// enemy_sound; corpse = entity_death / fresh_corpse. The impact point is not a miss distance: the measured one is NearMiss.
enum class DangerKind : std::uint8_t { none, grenade, enemy_sound, attack, corpse, hit, anomaly, other };
enum class DoorState : std::uint8_t { open, closed, opening, closing };

// Yields carried by the owning action's same-id evaluator (a set bit = the man leaves our ownership this tick and the
// game's own logic resumes; the engine fires painter_combat.on_release). Merged from both proposals.
enum Yield : std::uint32_t {
    yield_none = 0,
    yield_dead = 1u << 0, yield_wounded = 1u << 1, yield_grenade = 1u << 2, yield_anomaly = 1u << 3,
    yield_scheme_block = 1u << 4,            // a script scheme holds him (meet, abuse, help_wounded, corpse, ...)
    yield_story = 1u << 5, yield_companion = 1u << 6, yield_smart_cover = 1u << 7,
    yield_script_combat = 1u << 8,           // script combat type set
    yield_script_control = 1u << 9,          // CScriptEntity control / scene script
    yield_released = 1u << 10,               // the side handed him back (Decision::releases)
    yield_foreign_owner = 1u << 11,          // another owner (the Lua adapter during migration)
    yield_disabled = 1u << 12,               // the system is off
    yield_no_weapon = 1u << 13, yield_invalid_navigation = 1u << 14,
    yield_stale = 1u << 15,                  // no fresh order within stale_release_ms (see Order comment)
    yield_offline = 1u << 16, yield_invalid_input = 1u << 17,
    yield_no_first_order = 1u << 18,         // v2.12: the provisional hold outlived first_order_deadline_ms with no accepted
                                             // order (distinct from yield_stale, which is a lost refresh of a real order)
    yield_compat_unknown = 1u << 19,         // v2.21: his compatibility manifest is not sealed / a required evaluator is
                                             // missing / his token is stale -> he stays with the game's own logic (push mode)
};
// Script-only states arrive through level.painter_combat_set_flag(npc, name, bool); the name table is the engine's.
enum Flag : std::uint32_t {
    flag_story = 1u << 0, flag_companion = 1u << 1, flag_scheme_block = 1u << 2, flag_script_combat = 1u << 3,
    flag_dont_shoot = 1u << 4, flag_help_wounded = 1u << 5, flag_wounded_scheme = 1u << 6,
    flag_requested_control = 1u << 7,        // a mission / test asked for ownership of this man (explicit request only)
    flag_admitted = 1u << 8,                 // v2.10: the engine side admitted him this snapshot (owned, requested, or in
                                             // contact); the core plans only for admitted men. Never a Yield bit.
};

enum class Evidence : std::uint8_t { unknown, yes, no };   // evidence state; unknown is never treated as yes

struct CompatState {                             // v2.21: the man's compatibility receipt as the capture copies it
    std::uint32_t token = 0;                     // his current registration revision (0 = none)
    std::uint32_t pass_revision = 0;             // the forced pass whose results `veto` reflects (0 = no complete pass yet)
    Evidence sealed = Evidence::unknown;         // the manifest is committed
    Evidence veto = Evidence::unknown;           // the aggregate of his required evaluators and known facts in the last complete
                                                 // pass: yes = a script holds him; no = free; unknown = not evaluated / not sealed
    TimeMs at_ms = 0;
};

// v2.13: the engine's aimed-throw reasons as codes (ActorSnapshot::throw_state.reason). The executor calls the engine's
// begin_throw(npc, serial) when it hands a throw order to the aimed throw and end_throw(npc, cancelled|timeout|0) when it
// gives up or clears; capture derives the phase (prepared / in_hand / thrown / failed) and copies the reason. Aid: the
// executor calls note_aid(npc, target, phase); capture copies it into aid_state (phase 0 clears).
enum ThrowReason : std::uint8_t {
    throw_ok = 0, throw_no_target = 1, throw_blocked = 2, throw_no_arc = 3, throw_out_of_range = 4, throw_no_grenade = 5,
    throw_door_behind = 6, throw_cancelled = 7, throw_timeout = 8,
};

struct Contact {                                 // one perceived enemy, from the man's own memory (provenance kept)
    ActorKey key;
    Vec3 position;                               // seen now, or last seen
    Vec3 facing;                                 // unit while visible; (0,0,0) = unknown (this build has no stored orientation)
    float distance = 0;
    TimeMs observed_at_ms = 0;                   // when this memory record was last updated (any sense)
    TimeMs last_seen_ms = 0;                     // when he was last SEEN (0 = never seen, heard/hit only)
    std::uint64_t perception_seq = 0;            // changes whenever the fact is refreshed; a change detector, not a
                                                 // counter (this build fills it from the record's level time)
    bool visible = false;                        // seen THIS snapshot; a selected enemy alone never implies visible
    bool hostile = false;                        // relation at observation
    bool wounded = false, facing_me = false, alive_when_observed = true;
    bool finish_allowed = false;                 // engine fact: wounded, can_kill_enemy, no able foe seen <= 3 s
};
struct Heard { SoundKind kind = SoundKind::other; Vec3 position; float power = 0; TimeMs at_ms = 0; ActorKey source; };
struct Hit { TimeMs at_ms = 0; Vec3 dir; ActorKey from; };     // dir: from the man toward the hit's origin, flat
struct Danger { DangerKind kind = DangerKind::none; std::uint8_t perceive = 0; TimeMs at_ms = 0; Vec3 position; };
// v2.14: a round that passed the man without hitting him, MEASURED by the engine side in the bullet manager (xrGame
// Level_Bullet_Manager: CBulletManager::UpdateWorkload advances every live bullet one flight segment per frame, on the
// Device.seqParallel task under its own lock; the engine side takes the closest approach of that segment to the man's head
// point - the eye point the lean measurement used - and records ONE NearMiss per bullet and man at the moment the bullet
// passed its closest point or was removed, when that distance is within Config::near_miss_m). A bullet that hits the man
// is a Hit, never a NearMiss; the man's own bullets are excluded; a teammate's are not (the core attributes, the knowledge
// spec section 3). The core never invents a miss distance: with no NearMiss records the meter has only hits and the impact
// path (Danger::hit, head-to-impact distance). An engine build without the measurement leaves the array empty - no default.
struct NearMiss {
    TimeMs at_ms = 0;
    float miss_m = 0;                            // closest approach to the head point, metres, measured
    Vec3 origin;                                 // the shot's MUZZLE (the engine's SBullet::shot_origin, kept for the whole
                                                 // flight; NOT start_position, which the engine rewrites at every ricochet
                                                 // and exit point - erratum to v2.16's wording, native4): the crack's
                                                 // apparent source; the core treats it as a heard shot (dispersed, 4)
    Vec3 dir;                                    // from the man toward the origin, flat (as Hit::dir)
    ActorKey from;                               // the engine's shooter id; attribution per the knowledge spec (a visible
                                                 // foe inside the cone claims it, else a ghost on the bearing) - never a
                                                 // position source
};

struct ActorSnapshot {
    ActorKey key;
    std::uint64_t level_epoch = 0, snapshot_seq = 0, ownership_generation = 0, restrictions_revision = 0;
    TimeMs observed_at_ms = 0;
    bool alive = true, wounded = false, reloading = false;
    bool current_position_accessible = false, accessible_current_order = false;
    Vec3 position, forward, head_dir;
    float head_pitch_rad = 0, health = 1.f;
    std::uint32_t node_id = invalid_node, room_id = invalid_room, building_id = invalid_room;
    std::uint32_t faction = 0;                   // the engine's community index (actor.Community()); FactionStyle::faction
    std::uint64_t native_squad = 0;              // stock team/squad/group packed; the side id is in SideSnapshot
    struct { std::uint16_t id = invalid_id; WeaponKind kind = WeaponKind::none; WeaponState state = WeaponState::idle;
             std::uint16_t ammo_in_mag = 0, mag_size = 0;
             std::uint16_t ammo_reserve = 0;     // v2.10: rounds outside the magazine; 0 with an empty magazine = he cannot
                                                 // reload (the engine side yields no_weapon; the core never orders reload)
    } weapon;
    struct { std::uint8_t count = 0; std::uint16_t slot_item = invalid_id; } grenades;
    Contact enemy;                               // memory().enemy().selected(); key.id == invalid_id when none
    Bounded<Contact, 8> visible_enemies;         // nearest first
    Bounded<ActorKey, 8> visible_friends;         // physically visible, same-community stalkers; no implied side membership
    struct WitnessedSever {ActorKey subject;std::uint64_t revision=0,observer_owner=0;std::uint32_t component=0;TimeMs at_ms=0;};
    Bounded<WitnessedSever, 4> witnessed_severs;  // true committed sever, seen by THIS actor at the commit; never a death fact
    Bounded<ActorKey, 8> seen_dead;
    Bounded<Heard, 8> heard;                     // CSoundMemoryManager with the adapter's filters, 2 s window
    Bounded<Hit, 4> hits;                        // since the previous snapshot
    Bounded<NearMiss, 4> near_misses;            // v2.14: since the previous snapshot, newest kept (measured, see NearMiss)
    Danger danger;
    struct { bool enemy = false, danger = false, anomaly = false; } native_properties;
    std::uint32_t flags = 0;                     // Flag bits
    std::uint32_t yield_reasons = yield_disabled;// Yield bits the engine side already knows this tick
    // Feedback from the publish task: the last Order for this man it dropped, so the core learns that a goal was not
    // accessible (or stale) under the current revisions instead of proposing it again. Cleared when an order is accepted.
    struct { std::uint64_t order_seq = 0; std::uint32_t node = invalid_node; std::uint8_t reason = 0;
             std::uint64_t restrictions_revision = 0; } last_rejected;   // reason: RejectReason
    // v2.9 execution facts the core cannot derive (engine side fills from the owning action): the last order applied
    // and what the weapon did since the previous snapshot; throw and aid progress for the orders that asked for them.
    struct { std::uint64_t order_seq = 0; TimeMs at_ms = 0; std::uint8_t weapon = 0;   // weapon: 0 none, 1 aim, 2 fire, 3 reload
             bool fired_since_previous = false; } last_applied;
    struct { std::uint32_t serial = 0; std::uint8_t phase = 0;   // phase: 0 none, 1 prepared, 2 in_hand, 3 thrown, 4 failed
             std::uint8_t reason = 0; TimeMs at_ms = 0;          // reason: ThrowReason (v2.13)
             // v2.16: the engine's REAL release receipt (the detach of the live projectile at CGrenade::Throw's motion mark,
             // GE_OWNERSHIP_REJECT processed on the main thread). phase 3 (thrown) is set only when `releases` advanced
             // since begin_throw; never from an inventory change, the throwing animation or last_throw_time (the stock
             // action calls on_throw_completed on a slot change, which is the inventory inference the core refuses).
             std::uint32_t releases = 0;                         // his release count on this level, +1 per projectile that left his hand
             std::uint32_t projectile = invalid_id;              // object id of the last launched projectile
             std::uint32_t source_item = invalid_id;             // the hand grenade item that launched it (= grenades.slot_item at
                                                                 // submission when it was the submitted grenade)
             TimeMs released_at_ms = 0;                          // when it left his hand
    } throw_state;
    struct { ActorKey target; std::uint8_t phase = 0;            // phase: 0 none, 1 reaching, 2 working, 3 done, 4 failed
             TimeMs at_ms = 0; } aid_state;
    // v2.18: whether THIS man, when he is the wounded one, may be helped, and the stock scheme's claim on him - his own
    // side's script state (xr_wounded not_for_help; the help-wounded claim wounded_already_selected), pushed by the Lua
    // side through the engine export level.painter_combat_set_aid(npc, allowed: bool, claimed: bool, claimant_id: number)
    // whenever it changes (painter_ai_native_flags.script: with the stable flags at spawn / scheme switch, and a poll of
    // the currently wounded men at compat_poll_ms for the claim; claimant_id 65535 = none). The engine side stores it per
    // object id with the time and the capture copies it. The core's rule: a casualty with allowed != yes, or claimed by
    // someone other than the helper it would send, is never dispatched to; claimed stays true even when the claimant id
    // cannot be resolved to a live key (claimant.id = the raw id, generation 0). Friendly script state, never an enemy fact.
    struct { Evidence allowed = Evidence::unknown; bool claimed = false; ActorKey claimant; TimeMs observed_at_ms = 0; } aid_eligibility;
    CompatState compat;                          // v2.21: the push model's receipt (push-only mode admits only sealed && veto == no)
};
enum RejectReason : std::uint8_t {
    reject_none = 0, reject_epoch = 1, reject_geometry = 2, reject_older_snapshot = 3, reject_generation = 4,
    reject_ownership = 5, reject_goal_invalid = 6, reject_goal_inaccessible = 7, reject_order_seq = 8,
    reject_issued_in_future = 9,
    // v2.10: reasons that are NOT about the goal node; the core must not mark the goal inaccessible on them
    reject_unsupported = 10,                 // the executor does not implement this order's throw / aid in this slice
    reject_fire_point = 11,                  // fire.at is not a point the side knows
    reject_order_invalid = 12,               // an order field out of range / expired at publication (publishable() false)
};
// last_rejected.node is filled only for reject_goal_invalid / reject_goal_inaccessible; invalid_node otherwise.

// v2.9: the side's mission as an explicit request (authored by a test or a mission hook through the engine side, or
// left kind = none so the core decides from its own knowledge and the BuildingCache). Geometry to inspect, never
// enemy knowledge: a room to clear is a place, not an occupied hostile position.
enum class MissionKind : std::uint8_t { none, clear_building, defend, attack, hold };
struct Mission {
    std::uint32_t revision = 0;                  // bumped on every change; the core re-plans on a new revision
    MissionKind kind = MissionKind::none;
    std::uint32_t building = invalid_room, entry_portal = 0;   // entry_portal 0 = the core chooses
    Bounded<std::uint32_t, 32> rooms;            // ordered room ids allowed / to clear; empty = all of the building
};
struct SideSnapshot {
    std::uint32_t side_id = 0, faction = 0;
    std::uint64_t group = 0;                     // native squad id, or the ad-hoc group id
    std::vector<ActorKey> members;               // slot order (the roster: slot i = members[i])
    Mission mission;                             // v2.9
    struct Known {                               // one enemy the side knows about, merged from the members' own facts
        ActorKey key; Vec3 position; TimeMs latest_ms = 0; bool visible = false;
        std::uint64_t member_mask = 0;           // members (slot bit) with a fact about him; slots >= 64 are not in it
        Bounded<ActorKey, 8> reporters;          // the members whose facts were merged, newest first (no member cap)
    };
    std::vector<Known> known_enemies;            // shared knowledge; the engine's own squads cap at 64 members anyway
    std::uint32_t building_id = invalid_room;
    std::uint32_t mission_state = 0;             // deprecated since v2.9 (use mission); kept so v2.8 objects still compile
    Vec3 link_up;
    std::uint32_t cooperation_side = 0;          // worker-confirmed survivor link; native squad identities remain distinct
};

struct Door {                                    // dynamic door state (ai().doors()); geometry is in the BuildingCache
    std::uint16_t id = invalid_id;
    std::uint32_t portal = 0;                    // Portal::id it closes, 0 when none
    float open = 0;                              // 0 closed .. 1 open
    DoorState state = DoorState::closed;
    bool locked = false, held = false;
};

struct WorldSnapshot {
    std::uint64_t level_epoch = 0, sequence = 0, geometry_revision = 0, map_hash = 0;
    TimeMs now_ms = 0;
    std::vector<ActorSnapshot> actors;           // every online combatant eligible or owned; capacity reused
    std::vector<SideSnapshot> sides;
    std::vector<Door> doors;                     // within the sides' reach
};

// ---- decision (core -> engine) ----
enum class OrderKind : std::uint8_t {
    hold, advance, search, regroup, withdraw, recover, press, stack, peek, entry, hold_door, cover, bound, rush, follow,
    reload, yield,
};
enum class Pace : std::uint8_t { stand, walk, run };
enum class Stance : std::uint8_t { stand, crouch };
enum class LookKind : std::uint8_t { enemy, point, direction, path };
enum OrderFlag : std::uint32_t {
    order_committed_entry = 1u << 0, order_face_movement = 1u << 1, order_urgent = 1u << 2, order_sprint = 1u << 3,
    order_move_crouch = 1u << 4, order_cover_fire = 1u << 5, order_side_peek = 1u << 6, order_fan_peek = 1u << 7,
    order_rear_ok = 1u << 8, order_jamb = 1u << 9,
    order_first_response = 1u << 10,         // the cheap pass's order; replaced by the full plan
    order_fire_lane_low = 1u << 11,          // transient paired low stance; revalidated each authored update
    order_fire_lane_split = 1u << 12,        // same-target lateral step; aborts if pair/target dissolves
    order_fire_lane_high = 1u << 13,         // paired rear standing shooter; live native friend ray still gates fire
    order_fire_lane_low_was_crouch = 1u << 15,   // restore the authored front posture when the pair dissolves
    order_fire_lane_high_was_crouch = 1u << 16,  // restore the authored rear posture when the pair dissolves
    order_survivor_meet = 1u << 14,          // exact worker pending-pair regroup with route proof; publisher narrows spacing only for that pair
};

// An order is PERSISTENT: executed until superseded (higher order_seq) or the man is released. expires_at_ms is a
// staleness mark, not an end: past it the engine keeps position and look, closes the fire window, and after
// Config::stale_release_ms without a fresh order sets yield_stale and hands the man back with a logged reason. A man is
// never left idle in the stock planner while owned.
// v2.24: the executor's gait for a moving order (the animation banks: aimed = the danger walk / jog with the rifle up;
// lowered = the rifle-down jog, W2_Jog_F 3.19 m/s; sprint = the rifle-down full run, W2_Run_F 4.97 m/s). The core sets
// sprint for an order_sprint run with no fire / look target, lowered for a run whose look is LookKind::path, aimed for
// anything with a fire or look target. The executor calls set_painter_gait(gait); speed_scale is 1.0 for lowered / sprint
// (the clip's own speed; Config::outdoor sprint_speed becomes a cap). Never sprint or lowered while aiming: the engine falls
// back to aimed while the weapon fires, the sight is on an enemy object, the body is > 30 deg off the path, or crouched.
enum class Gait : std::uint8_t { aimed = 0, lowered = 1, sprint = 2 };

struct Order {
    ActorKey actor;
    std::uint64_t level_epoch = 0, snapshot_seq = 0, order_seq = 0, ownership_generation = 0, geometry_revision = 0,
                  restrictions_revision = 0;
    TimeMs issued_at_ms = 0, expires_at_ms = 0;
    OrderKind kind = OrderKind::hold;
    std::uint16_t role = roles::none;            // roles:: in CombatIds.h
    std::uint32_t goal_node = invalid_node;
    Vec3 goal_position;                          // node centre
    float arrival_radius_m = .45f;
    Pace pace = Pace::stand;
    Stance stance = Stance::stand;
    struct { LookKind kind = LookKind::path; Vec3 value; ActorKey target; } look;
    // fire.allowed is INTENT; the engine side applies the live gates (friend ray, enemy ray, aim angle, reload, wounded
    // enemy). fire.at with at_valid = suppression at a point (sight-to-point + fire), no visible target needed.
    struct { bool allowed = false; Vec3 at; bool at_valid = false; TimeMs until_ms = 0;
             std::uint8_t queue = 1; std::uint16_t interval_ms = 300; float ammo_floor = .33f; } fire;
    float lean = 0;                              // -1..1
    Vec3 lean_edge;
    struct { bool stand = false, lean = false; } expose;
    std::uint32_t flags = 0;                     // OrderFlag bits
    struct { Vec3 centre, tangent, normal; float half_width = 0, thickness = 0; bool valid = false; } portal;
    struct { std::uint16_t door = invalid_id, item = invalid_id; Vec3 target; bool valid = false;
             std::uint32_t serial = 0, portal = 0; } throw_order;   // v2.9: serial echoed in throw_state; portal for jambs/normal
    ActorKey aid, finish;
    std::uint32_t abort_vertex = invalid_node;
    float speed_scale = 1.f;                     // 0.3..1.5
    Gait gait = Gait::aimed;                     // v2.24: see enum Gait above; order_sprint stays as the intent flag
    // v2.9 doorway aiming (the meaning the core emits, the executor applies while the man moves): the executor's look
    // priority is  visible enemy > pie.pivot (valid) > corner.point (valid, once past the door line) > look  ; side is
    // the signed stack side of the portal (-1 / +1 along portal.tangent), 0 = none. Never inferred from role or lean.
    struct { Vec3 pivot; bool valid = false; } pie;
    struct { Vec3 point; bool valid = false; } corner;
    std::int8_t side = 0;
    // v2.9 persistent assignment identity (the core's typed replacement for the Lua role string's qualifier): the
    // executor keys its own persistence (earned shot, retreat anchor, fire key) on {actor, assignment}, NOT on
    // order_seq, which changes on every refresh; generation changes only when the assignment itself changes.
    struct { std::uint32_t sector = 0, portal = 0; std::uint8_t slot = 0; std::uint64_t generation = 0; } assignment;
};

struct Event { std::uint16_t kind = events::none; std::uint32_t room = invalid_room; std::uint32_t detail = 0; };
struct Decision {
    std::uint32_t side_id = 0;
    std::uint64_t level_epoch = 0, snapshot_seq = 0, geometry_revision = 0;
    std::vector<Order> orders;
    std::vector<ActorKey> releases;              // hand-backs to the game's own logic
    std::vector<Event> events;                   // diagnostics only
    std::uint32_t cost_us = 0;
};
// VALIDATION at publication (engine, publish task):
// - whole Decision dropped when level_epoch or geometry_revision differ from the current ones, or snapshot_seq is
//   older than the side's last applied decision;
// - per Order dropped (the others stand) when the actor's generation or ownership_generation changed, the goal node is
//   invalid or not accessible under the man's CURRENT restrictions, order_seq is not greater than his current one, or
//   issued_at_ms is later than now; the engine's current order for him stands;
// - until the core's first order for a man arrives (admission + 500 ms at most) the engine side holds him crouched
//   facing the threat (hit direction, else selected enemy, else forward) - never idle in the stock planner;
// - a dropped Order is reported back in the man's next ActorSnapshot::last_rejected (node, reason, revisions) so the
//   core marks that goal accessible = no for those revisions and proposes another; the worker cover query's
//   Evidence::unknown is never turned into yes by anyone but this publish check.

// v2.14: a faction's fighting style (the Lua reference's FD.FACTION, from the lore; OUTDOOR_LAYERS_SPEC 5). Data, not
// code: the posture layer reads it as costs. profile 0 normal (patient) / 1 chad (short hold-ground window, quick to search
// and to rush a vulnerable enemy); doctrine 0 loose (cover to cover together) / 1 drilled (bounding overwatch, leapfrog,
// push on a suppressed enemy, assault); morale 0 steady (breaks with half the team down or badly hurt) / 1 brittle (a
// third) / 2 fanatic (never withdraws); comms 0 voice (Config voice_m / voice_p) / 1 radio (every report delivered).
// Read by the engine side from [painter_combat_factions], one line per engine community string:
//   <community> = <profile> <doctrine> <spread_m> <morale> <leave_wounded 0|1> <comms>
// and matched by the community index the capture writes into ActorSnapshot / SideSnapshot::faction.
struct FactionStyle {
    std::uint32_t faction = 0;                   // the engine's community index
    std::uint8_t profile = 0, doctrine = 0, morale = 0, comms = 0;
    bool leave_wounded = false;                  // does not hold a withdrawal for its own casualty
    float spread_m = 5.f;                        // spacing moving up on open ground before contact (4 for the loose mobs)
};

struct KnowledgeConfig {
    TimeMs forget_ms = 240000, search_forget_ms = 400000, contact_keep_ms = 30000;
    TimeMs shot_at_me_ms = 5000, shot_me_ms = 15000, reloading_ms = 4000, suppressed_ms = 4000;
    TimeMs ghost_keep_ms = 30000, ghost_dying_ms = 2500, uf_dir_ms = 1500, uf_escape_ms = 2000, uf_run_ms = 1500;
    TimeMs heard_report_ms = 1000, shot_me_pick_ms = 2000, squad_seen_ms = 10000, assess_foe_ms = 60000;
    TimeMs dog_seen_ms = 1000, dog_shotme_ms = 15000, dog_unseen_ms = 8000, dog_nosol_cool_ms = 15000;
    TimeMs peace_ms = 60000, input_age_ms = 5000;
    std::uint32_t latency_ticks = 1;                 // reports queued one tick reach their receivers the next
    float jitter_min = .5f, jitter_max = 1.5f, hit_cone_deg = 20, hit_cone_close_deg = 45;
    float hit_cone_close_m = 10;                     // v2.19: hit_cone_close_deg applies to a visible foe within this (spec 3;
                                                     // was borrowed from dog_start_m in K1 - its own field now)
    float ghost_cone_deg = 25, ghost_merge_deg = 20, ghost_merge_min_m = 3, ghost_merge_range_k = .3f;
    float hit_default_m = 25, hit_min_m = 2, hit_back_m = .4f, probe_fraction = .6f, probe_min_m = 3;
    std::array<float, 2> ghost_probe_m{{12, 24}};    // ray probes along the round's line when nothing else names the source
    float red_source_cone_deg = 30, red_source_min_m = 1, red_source_max_m = 25, red_floor_m = 4.5f;
    float red_angle_cost = 40, red_distance_cost = .05f, eye_m = 1.6f;
    float disp_shot = 17.5f, disp_other = 12.5f, disp_cap_m = 50, heard_near_m = 3;
    float clear_heard_m = 60, gunfire_max_m = 80;
    float meter_max = 30, meter_decay_per_second = 1, meter_hit = 3, caliber = 2, proxy_shot = 2;
    std::array<float, 4> meter_thresholds{{1, 6, 15, 25}};   // light / medium / heavy / extreme
    float miss_near_m = .5f, miss_full_m = 4, miss_end_m = 10, miss_near_factor = 1.5f;   // the meter's miss factor shape:
                                                     // miss_near_factor within miss_near_m of the head, 1.0 to miss_full_m,
                                                     // falling to 0 at miss_end_m (fed by NearMiss::miss_m, v2.14; never a default)
    float keep_target_k = .75f, engaged_near_m = 50, dog_start_m = 10, dog_end_m = 15, dog_ammo = .3f;
    float project_horizontal_m = 2, project_vertical_m = 2;   // NodeProjectFn bounds for perceived points
    std::uint16_t contacts_cap = 16, event_history_cap = 128, hit_log_cap = 64, outbox_cap = 200;   // bounded storage
};

struct RoomConfig {
    float pitch_m = .7f, spacing_m = .8f, jamb_offset_m = .45f, wall_offset_m = .55f;   // the stack: shoulders touch
    float arrival_m = .45f, projection_m = .8f, floor_tolerance_m = .45f;
    float exposure_cost_m = 20.f, unknown_cost_m = 6.f, funnel_cost_m = 20.f;           // slot costs (metres equivalent)
    float path_unknown_cost_m = 12.f, approach_weight = 1.f, alternate_door_cost_m = 2.f;
    float straggler_cost_per_second = 1.f, readiness_missing_cost = 8.f;
    float two_door_gain_m = 8.f, post_distance_m = 1.5f;
    TimeMs straggler_ms = 8000, swap_ms = 3000;      // a straggler never holds the team past this; a #2 / #3 swap at most this
    float look_height_m = 1.3f;
    std::uint32_t path_nodes = 2048;                 // the bound of a stack path search
    std::uint16_t alternative_slots = 8;
    std::uint8_t minimum_door_members = 2;
    float pie_radius_m = 2.f, pie_advance_m = .9f, pie_lean = .8f, edge_lean = .7f;   // a standing moving pie
    TimeMs pie_dwell_ms = 0, slice_dwell_ms = 0, pie_cap_ms = 20000;                // caps, never minimum waits
    float form_tolerance_m = .6f; TimeMs form_dwell_ms = 0;                         // distinct from exact slot arrival_m
    TimeMs leader_idle_ms = 1200, leader_no_shot_ms = 3000, leader_go_wait_ms = 2000, leader_up_wait_ms = 3000;
    float leader_progress_m = .15f, leader_heel_m = 2.5f, leader_point_m = 1.2f, leader_set_m = .6f;
    float pie_engage_hysteresis = 1, pie_unaware_deg = 45, pie_partly_deg = 100, pie_near_m = 3.5f, pie_mid_m = 6;
    float pie_engage_unaware = 3, pie_engage_partly = 1, pie_engage_aware = 1.5f, pie_engage_solo = 2.5f,
          pie_engage_duo = 1.25f, pie_engage_team = .5f, pie_engage_near = 1.5f, pie_engage_mid = .5f,
          pie_engage_far = .5f, pie_engage_unseen = .5f;                            // score terms, signs as in the reference
    float pie_call_hysteresis = 1; TimeMs pie_overwatch_cap_ms = 8000;
    float pie_call_men = 1, pie_call_near = 2, pie_call_mid = .5f, pie_call_far = 1.5f, pie_call_surprise = 1.5f,
          pie_call_alert = 1, pie_call_unseen = 1.5f, pie_call_unseen_over = 1, pie_call_door = 2,
          pie_call_casualty = 1.5f, pie_call_half_line = .5f, pie_call_line = 2.5f;
    TimeMs pie_call_door_ms = 5000;
    // entry / crossing / local correction
    TimeMs release_gap_ms = 450; float entry_arrival_m = .7f;                        // distinct from stack arrival_m
    float entry_follow_m = 1.f, entry_heel_m = 1.5f, entry_behind_m = .5f, entry_third_m = 4, entry_ready_m = .6f,
          entry_settle_m = 1.2f;
    TimeMs entry_stall_ms = 2000, entry_reload_cap_ms = 6000, entry_flow_cap_ms = 8000;   // the active release fallback
    float entry_zone_half_m = .35f, entry_zone_depth_m = .8f, entry_lane_half_m = .7f, entry_lane_depth_m = 1.6f;
    std::uint8_t entry_pace = 0;                     // 0 walk (ruling), 1 run
    float entry_walk_scale = 1.45f, entry_run_scale = .8f, jamb_final_m = 1.f, jamb_speed_scale = .5f;
    TimeMs door_hot_ms = 3000; float door_hot_cost = 40;                             // 40 -> 20 -> 0 at 0 / 1500 / 3000 ms
    TimeMs crossing_gap_ms = 1500, crossing_block_cap_ms = 1500;                      // bounded once, not re-armed
    float denial_step_m = .35f, exposed_correction_max_m = 1.5f;                    // tests the CURRENT body location
    TimeMs return_fire_ms = 2000, return_fire_overlap_ms = 1000, return_fire_recent_shot_ms = 3000;
    std::uint16_t suppression_queue = 3; TimeMs suppression_interval_ms = 600; float suppression_ammo_floor = .33f;
    // search / known positions / finisher
    float see_range_m = 15, check_half_deg = 35;
    TimeMs seen_ms = 20000, seen_red_ms = 8000, sweep_step_ms = 1000, search_after_ms = 2000;   // coverage, never a census
    float known_enemy_exclusion_m = 1.8f, known_enemy_search_m = 3.5f; TimeMs known_enemy_ms = 60000;   // per-man provenance kept
    std::array<float, 5> finisher_rings_m{{2.1f, 2.8f, 3.5f, 4.2f, 4.9f}};
    std::uint8_t finisher_tries = 3;
    TimeMs finisher_try_ms = 6000, finisher_cap_ms = 30000, finisher_skip_ms = 30000, finisher_quiet_ms = 4000,
           finisher_keep_ms = 120000;                // one eligible finisher; the rest keep their sectors
    // post-room assess (the room's own; the strategic withdrawal assessment is the outdoor layers')
    TimeMs assess_cap_ms = 4000, assess_crossing_grace_ms = 1000, assess_fresh_hit_ms = 2000, assess_slice_ms = 900;
    float assess_radius_m = 2.8f;
    std::array<float, 3> assess_angles_deg{{20, 40, 60}};
    std::array<float, 3> assess_radii_m{{2.8f, 2.1f, 3.5f}};
    // casualty / aid (the executor's aid transaction reads these; O1 reuses them outdoors)
    TimeMs aid_hit_quiet_ms = 2500, aid_beam_ok_ms = 8000, aid_cap_ms = 45000, aid_work_ms = 4000;
    float aid_reach_m = 1.6f, aid_arrival_m = .5f, aid_min_body_m = .45f;
    std::array<float, 3> aid_placement_radii_m{{.8f, .65f, 1.f}};
    std::array<float, 8> aid_placement_angles_rad{{0, .6f, -.6f, 1.2f, -1.2f, 1.9f, -1.9f, 3.14159265f}};
    TimeMs casualty_lane_block_cap_ms = 1500; float casualty_lane_block_cost = 60;
    float casualty_lane_width_margin_m = .3f, casualty_lane_depth_margin_m = .7f, casualty_lane_vertical_m = 1;
    TimeMs buddy_answer_ms = 3000;                   // the teammate-hit answer window (O1 reuses it)
    // frag / executor timing (the atomic throw transaction, ruling above)
    TimeMs frag_fuse_ms = 3500, frag_prefire_cap_ms = 4000, frag_hand_cap_ms = 2500, frag_arc_wait_ms = 1500,
           frag_release_cap_ms = 3500, frag_receipt_ms = 250;   // fuse and release cap share 3500 by coincidence
    std::array<float, 3> frag_inside_m{{2.5f, 3.5f, 4.5f}};
    float frag_min_m = .8f, frag_max_m = 4.5f, frag_door_height_m = 1.1f, frag_reposition_step_m = .7f;
    TimeMs frag_reposition_cap_ms = 2000;
    TimeMs frag_arc_poll_ms = 100;                   // approved: bounded 10 Hz native arc work; Config never replaces the
                                                     // engine's throwability / trajectory / team-safety answer or the release proof
    // defend / withdraw (the room's)
    TimeMs defend_quiet_ms = 2500, withdraw_cap_ms = 30000, withdraw_quiet_ms = 3000, lone_hold_ms = 20000;
    std::uint8_t withdraw_break_hits = 2; TimeMs withdraw_break_window_ms = 3000;   // observed hits, never a danger count
    TimeMs withdraw_foe_ms = 60000, withdraw_kill_ms = 2000, withdraw_trend_ms = 15000, withdraw_persist_ms = 1500,
           withdraw_cooldown_ms = 60000, withdraw_threat_ms = 20000, withdraw_resume_quiet_ms = 6000;
    float withdraw_unsafe_m = 6, withdraw_health_mean = .35f, withdraw_able_fraction = .5f, withdraw_casualty_rally_m = 3;
    TimeMs withdraw_peel_inside_ms = 2500, withdraw_peel_door_ms = 6000;
    float backout_min_m = 1, backout_max_m = 1.5f, backstep_enemy_m = 2, backstep_m = 1.5f;   // short back-out, then turn
    TimeMs backout_cap_ms = 2500, backout_unseen_ms = 300, backstep_ms = 900;
    std::uint8_t rear_from_slot = 3;                 // from this slot index the look is rear security (back along the stack's
                                                     // wall or the nearest other doorway of the stack's room), role rear_security
    std::uint8_t watch_post_min_men = 3;             // a far-door post is considered from this many men (was a literal)
    float door_locked_cost_k = 1.f, door_held_cost_k = .5f;   // door-state weights on funnel_cost_m (were literals)
};

// ---- core settings (read by the engine from [painter_combat], passed to the core; costs and timings are data) ----
// v2.14 convention, honoured in v2.15: the knowledge / room phases' settings are Config::knowledge (KnowledgeConfig) and
// Config::room (RoomConfig) below, read as knowledge_<field> / room_<field>; later slices add fields to those structs.
struct Config {
    TimeMs first_order_deadline_ms = 500;
    TimeMs order_lifetime_ms = 3000;             // expires_at_ms - issued_at_ms
    TimeMs stale_release_ms = 10000;             // no fresh order for this long -> yield_stale
    TimeMs snapshot_max_age_ms = 500;
    TimeMs memory_max_age_ms = 5000;
    float nearby_group_distance = 20.f;          // ad-hoc squads: same faction within this
    float regroup_separation_distance = 40.f;
    float link_up_max_distance = 150.f;          // only a quiet friendly group within this can anchor a withdrawal
    float minimum_goal_separation = 2.5f;        // one man per spot
    float immediate_cover_distance = 10.f;
    float cover_radius = 35.f;
    std::uint8_t cover_candidates = 32;
    std::uint32_t seed = 1;
    // v2.10: the dispatcher's pacing, data like everything else (were compiled-in constants)
    TimeMs refresh_ms = 250;                     // first-response re-decision interval while no fresh plan covers the man
    TimeMs cover_cache_ms = 250;                 // a cover query result stays usable this long
    TimeMs plan_interval_ms = 250;               // between full-plan jobs for a side
    std::uint16_t bootstrap_work_per_tick = 32;  // first-response decisions per authority tick
    std::uint16_t cover_queries_per_slice = 32;
    TimeMs fire_window_ms = 500;                 // how long a fire intent stays open from its evidence, independent of
                                                 // snapshot_max_age_ms; the executor still gates live visibility
    std::uint32_t bootstrap_deadline_us = 500;   // the authority tick's first-response time box
    TimeMs compat_poll_ms = 250;                 // the compatibility shim's poll cadence while the push model is not in
    // Costs are metres of travel equivalent (a 20 m walk and a fully exposed spot weigh the same): costs, never vetoes
    float cover_exposure_cost_m = 20.f;          // per unit of exposure (0..1) toward the threats
    float cover_unknown_cost_m = 6.f;            // for Evidence::unknown protection / reachability
    float cover_move_hysteresis_m = 2.f;         // a new spot must beat the held one by this to move
    // v2.12
    float suppressed_exposure_cost_m = 8.f;      // surcharge on a spot's exposure while the man is suppressed
    float cover_far_cost_m_per_m = 1.f;          // cost per metre beyond immediate_cover_distance (a cost, not a cut-off)
    TimeMs readmit_delay_ms = 250;               // after a hand-back, no re-admission of the same man for this long
    // v2.13: the engine lean's real sideways eye shift at full lean (measured on the real skeleton and aim clips,
    // 2026-10-04: standing 0.149-0.171 m, crouched 0.056-0.099 m; the Lua assumed 0.35 m). The core prices a lean's
    // sight gain with these, nothing else.
    float lean_eye_offset_m_stand = .15f;
    float lean_eye_offset_m_crouch = .06f;
    // v2.14: near misses and the side's comms (the knowledge layer's values from the Lua reference; data)
    float near_miss_m = 10.f;                    // the engine side records a NearMiss within this of the head point (the
                                                 // meter's miss factor reaches 0 at 10 m)
    float voice_m = 35.f;                        // a report reaches a teammate by voice within this ...
    float voice_p = .7f;                         // ... with this probability (seeded per sender / receiver / serial); a
                                                 // side with comms = radio always delivers
    // v2.14: the factions' styles (above); the engine side fills them from [painter_combat_factions]
    Bounded<FactionStyle, 24> factions;
    FactionStyle faction_default;                // for a community not listed: normal, loose, 5 m, steady, keeps wounded, voice
    // v2.15: the phases' own settings (above)
    KnowledgeConfig knowledge;
    RoomConfig room;
    // v2.21: the compatibility model as data: 0 = bridge (the cached poll at compat_poll_ms + the pushes; default until the
    // push acceptance passes), 1 = push-only (no poll; an unsealed / unknown man yields yield_compat_unknown)
    std::uint8_t compat_mode = 0;
    // v2.24 ERRATA to v2.21, from the live runs of 2026-10-04 (all live in the installed Lua registrar):
    //  (a) the veto EVALUATOR set is ONLY the evaluators that are preconditions of the stock COMBAT action - today the man's
    //      own wound (sidor_wounded_base). meet / abuse / wounded_exist / corpse_exist and the scheme modules' evaids are
    //      preconditions of the ALIFE idle action; registered as vetoes they refused 98 % of admissions in a fight.
    //  (b) compat::dont_shoot means a SCRIPT forbids shooting (story scenes). rx_ff's per-tick friendly-fire flag is never
    //      this fact (the native fire gate has its own friendly ray); the registrar pushes it as a known false.
    //  (c) registration is retried on the man's updates until his planner is initialized - a man spawned into a loaded level
    //      is not registrable at his bind events.
    //  (d) aid eligibility is pushed after every successful registration; an unset not_for_help is 'may be helped'.
    //  (e) writers push only a CHANGED value; no invalidate-before-write (per-tick writers churned 129k seals).
    // v2.23: the outdoor layers' tunables (painter_ai/OutdoorConfig.h, a contract header since v2.23; keys outdoor_<name>)
    OutdoorConfig outdoor;
    TimeMs posture_dwell_ms = 1500;              // a side's posture is a commitment: it does not change for this long unless
                                                 // the reason is the casualty / hit exchange or the mission ended
    TimeMs unseen_reaction_source_ms = 600;      // an unseen hit's K1 point stays a reaction source this long
    TimeMs unseen_reaction_hold_ms = 4000;       // the responder watches an unseen source this long
    float lifecycle_release_range_m = 200.f;     // a side whose nearest man is farther than this from the fight's reference
                                                 // point is released out_of_range
    float lifecycle_distance_cost_per_m = 1.f;   // the distance cost per metre in that release
    TimeMs lifecycle_required_door_stuck_ms = 3000;
};

// ---- the work queue the core uses (Painter's pool, shared budget) ----
struct JobStats {
    std::uint64_t submitted = 0, completed = 0, cancelled = 0, rejected = 0;
    std::uint64_t late = 0;                      // v2.8: jobs whose wait (submit -> start) exceeded job_max_wait_ms
    std::uint32_t depth = 0;
    float age_ms_p50 = 0, age_ms_p90 = 0, age_ms_max = 0;   // submit -> completion
    float wait_ms_p90 = 0, wait_ms_max = 0;      // v2.8: submit -> start; the Painter-starvation measure
    float frame_budget_ms = 0, frame_used_ms = 0;
};
// SCHEDULING GUARANTEE (v2.8, engine owner): AI jobs run on a reserved lane of their own threads (job_workers, data),
// never on the Painter's wound workers, so a wound backlog cannot delay an AI job and an AI slice never sits in front
// of a wound burst; the drain waits only for admitted lane jobs. job_max_wait_ms (data) is a watchdog that counts
// `late`, not a scheduling input.
using JobTicket = std::uint64_t;                 // 0 = rejected (queue full or over budget)
// LIFETIME GUARANTEE (engine owner): EngineServices (the JobQueue, the cover and ray callbacks, the BuildingCache) are
// valid for exactly one level_epoch. At level teardown the owner cancels every ticket and DRAINS the queue (waits for
// the running slices to return) before the level, its static data or the cache are freed; a slice is bounded (a job
// polls cancelled() at least every 2 ms of work) so the drain is short. A job therefore never outlives its epoch and
// the raw pointers in EngineServices are safe for the job that received them. The core never caches EngineServices
// across epochs and never dereferences them outside a job or the publish path.
class JobQueue {
public:
    virtual ~JobQueue() = default;
    // fn runs on a worker; it must touch values only. cancelled() turns true when the ticket is cancelled or the level
    // epoch changes; long jobs poll it between slices (at least every 2 ms of work).
    // fn MUST NOT THROW: it is called from engine-header code built with exceptions off (no unwind tables between the
    // kernel lane and fn), so the core catches everything inside its own translation units and returns normally.
    virtual JobTicket submit(std::function<void(const std::function<bool()>& cancelled)> fn, std::uint8_t priority) = 0;
    virtual void cancel(JobTicket ticket) = 0;
    virtual JobStats stats() const = 0;
};

// ---- static data the core may call from a worker (engine implements; values in, values out) ----
struct CoverQueryInput {
    Vec3 centre; float radius = 35.f;
    Bounded<Vec3, 8> threats;
    float min_enemy_m = 3.f, max_lo = .4f;
    std::uint8_t max_n = 32;
    Bounded<float, 4> heights;                   // default {.7, 1.1, 1.7}
};
struct CoverSpot {                               // a candidate WITH its evidence; unknown is never good cover
    std::uint32_t node = invalid_node, neighbour_toward_threat = invalid_node;
    Vec3 position;
    float hi = 1, lo = 1, dc = 0, dt = 0;        // cover toward threats[threat]; flat distances to centre / that threat
    std::array<float, 8> nibbles{};              // high0..3, low0..3 (the nav cover values)
    std::uint8_t threat = 0;                     // index into CoverQueryInput::threats of the nearest threat
    std::array<bool, 4> blocked{};               // static ray from spot + heights[i] to that threat's eye hits geometry
    // provenance (the decision core's CoverCandidate evidence, kept): what was checked, against which revisions
    Evidence protection_known = Evidence::unknown;   // nibbles and rays evaluated for this threat set
    Evidence accessible = Evidence::unknown;         // under the man's restrictions (the worker query leaves it
                                                     // unknown; the publish task sets yes/no on the chosen goal)
    Evidence reachable = Evidence::unknown;          // a path exists (unknown unless a path check ran)
    std::uint64_t geometry_revision = 0, restrictions_revision = 0;   // 0 = not checked against any
    ActorKey threat_key;                         // the threat the evidence refers to (core fills from its Contact)
    TimeMs threat_observed_at_ms = 0;            // that Contact's observed_at_ms at query time
};
// Same selection as level.painter_cover_query, without restrictions (the publish task re-checks accessibility).
// Returns the number written. Thread: any (cover quadtree and nav values are read-only after load). The core's own
// scoring (exposure, travel, teammate safety from the side's positions) lives in painter::combat::core on top of this.
using CoverQueryFn = std::uint8_t (*)(const CoverQueryInput& in, CoverSpot* out, std::uint8_t capacity);
struct Segment { Vec3 a, b; };
// Distance along a->b to the first static hit, -1 when clear. Thread-local collider: any thread.
using RayBatchFn = void (*)(const Segment* in, float* hit, std::uint32_t count);

// Static red light through an opening (v2.4): for a node on one side of a portal, the share of sample points on the
// OTHER side (within 12 m of the portal, standing eye 1.6 m) with a clear static line to the node, at a standing
// target (1.1 m) and a crouched one (0.5 m); 0..255. Built offline into the per-map building data and loaded with the
// cache; an opening with no table has empty vectors and the core falls back to its own rays. The core reads a node's
// exposure through an opening in O(log n) here and spends rays only to confirm a chosen spot.
struct Lit { std::uint32_t node = invalid_node; std::uint8_t standing = 0, crouched = 0; };

struct Portal {
    std::uint32_t id = 0, room_a = invalid_room, room_b = invalid_room;   // room_b invalid_room = outside
    bool doorway = true, traversable = true, stair = false;
    Vec3 centre, normal, jamb_a, jamb_b;         // flat normal a -> b; no axis-aligned assumption
    float half_width = 0, wall_depth = 0, y0 = 0, y1 = 0;
    std::uint16_t door = invalid_id;             // Door::id when a door object closes it
    std::vector<Lit> lit_a, lit_b;               // nodes in room_a lit from room_b's side, and the reverse; sorted by node
};
struct Room {
    std::uint32_t id = 0; Vec3 centre; float floor_y = 0;
    std::uint32_t nodes = 0;                     // node count (kept from v2.4)
    std::vector<std::uint32_t> portals;
    // v2.9, from the per-map data: the room's nav nodes (sorted), its flat footprint (x0,z0)-(x1,z1) as min/max, and the
    // ordered check points ("sectors") when the data carries them (empty = the core derives them by its own rule and
    // says so in its receipt); approach = an authored approach point or (0,0,0)
    std::vector<std::uint32_t> node_ids;
    Vec3 min, max;
    std::vector<Vec3> sectors;
    Vec3 approach;
};
struct Building { std::uint32_t id = 0; std::vector<Room> rooms; std::vector<Portal> portals; };
struct BuildingCache {                           // pinned for one level_epoch
    std::uint64_t map_hash = 0, level_epoch = 0;
    std::uint32_t data_version = 0;              // the per-map data file's version (9 = with lit tables)
    bool lit_complete = false;                   // v2.7: true when every portal side's lit table decoded (lit_bad 0) and no
                                                 // side was skipped: then an EMPTY lit vector means genuinely dark from every
                                                 // far-side eye and the core skips its own rays there; false = fall back to rays
    std::vector<Building> buildings;
    // node -> (building, room); invalid_room when outside every building
    std::function<std::pair<std::uint32_t, std::uint32_t>(std::uint32_t node)> locate;
};

// Point -> nav node (v2.6): the nearest nav node to p within max_horizontal_m (flat) and max_vertical_m (so a point on
// an upper floor never snaps to the floor below), on the pinned level graph (read-only after load: any thread); returns
// invalid_node when none is in reach and never invents an id. snapped (optional) receives the node's centre. The core
// uses it for every target it computes as a point (stack slots, pie places, entry spots, posts); accessibility is still
// the publish task's check. node_position: a node's centre, false when the id is out of range.
using NodeProjectFn = std::uint32_t (*)(const Vec3& p, float max_horizontal_m, float max_vertical_m, Vec3* snapped);
using NodePositionFn = bool (*)(std::uint32_t node, Vec3* out);
// v2.13: path length in metres between two nav nodes on the pinned graph (the engine owner's own bounded A* over vertex
// links and centres, straight-line heuristic, per-thread scratch; the engine's CGraphEngine is NOT reentrant and is not
// used), restrictions ignored (publish re-checks), -1 when the goal is not reached within max_nodes (<= 65536).
// Lane-safe: callable from a job.
using NodePathLengthFn = float (*)(std::uint32_t from_node, std::uint32_t to_node, std::uint32_t max_nodes);

// Everything the core needs from the engine, handed once per level_epoch.
struct EngineServices {
    JobQueue* jobs = nullptr;
    CoverQueryFn cover_query = nullptr;
    RayBatchFn rays = nullptr;
    NodeProjectFn project = nullptr;
    NodePositionFn node_position = nullptr;
    NodePathLengthFn path_length = nullptr;      // v2.13
    const BuildingCache* buildings = nullptr;
};

}} // namespace painter::combat
