#pragma once
// painter_ai/EngineCapture.h - the engine side's capture for the native combat AI (contract v2.2, CombatContract.h).
// Authority thread only: the publish task (THREAD MODEL in CombatContract.h). Perceived facts only - everything comes
// from the man's own memory managers; nothing here enumerates hostiles. No pointer leaves these functions.
#include "CombatContract.h"
#include <functional>

class CAI_Stalker;
class CEntityAlive;

namespace painter { namespace combat { namespace engine {

struct CaptureContext {
    ActorKey key;                                   // the registry's key for this man
    std::uint64_t level_epoch = 0, snapshot_seq = 0, ownership_generation = 0, restrictions_revision = 0;
    TimeMs now_ms = 0;
    TimeMs previous_snapshot_ms = 0;                // his previous snapshot: hits newer than this are reported
    std::uint32_t current_goal_node = invalid_node; // his current order's goal (accessible_current_order)
    bool requested_control = false;
    std::function<ActorKey(const CEntityAlive&)> key_of;   // id + generation of a perceived entity (the registry's)
    const BuildingCache* buildings = nullptr;       // room and building of his node, once built
    decltype(ActorSnapshot::last_rejected) last_rejected{};   // the publish task's last dropped Order for him
};

// Fills every ActorSnapshot field from `actor`. yield_reasons carries what the engine knows here (dead, wounded,
// grenade, anomaly, smart cover, script control, no weapon, and the yields of the script flags); the registry ORs in
// its own (disabled, released, foreign owner, offline, stale).
void capture_actor(CAI_Stalker& actor, const CaptureContext& context, ActorSnapshot& out);
// Same personal sound / measured bullet selectors used by admission and capture.
// peek does not consume bullet receipts. No contact/type/visibility is invented.
void capture_personal_events(CAI_Stalker&,const CaptureContext&,ActorSnapshot&,bool consume_misses);


// Dynamic door state within `radius` of `centre` (ai().doors()); Door::portal from the door <-> portal match
// (EngineServices door_portal, the same match as Portal::door).
void capture_doors(const Vec3& centre, float radius, std::vector<Door>& out);
// At most max_rows serialized. The native nearest() lookup itself is indivisible.
void capture_doors_bounded(const Vec3& centre, float radius, std::size_t max_rows, std::vector<Door>& out);

// Level time widened to 64 bits (Device.dwTimeGlobal wraps tracked) and the level epoch, which changes on every level
// load; on_level_load also drops the script flags.
TimeMs now_ms();
std::uint64_t level_epoch();
void on_level_load();

// Script-only states, from level.painter_combat_set_flag(npc, name, bool). Names: story, companion, scheme_block,
// script_combat, dont_shoot, help_wounded, wounded_scheme, requested_control. False for an unknown name.
bool set_flag(std::uint16_t id, const char* name, bool value);
std::uint32_t flags_of(std::uint16_t id);
void forget(std::uint16_t id);                      // net_Destroy of the object

// painter_combat.on_release(npc): the owning action's finalize calls this, so the script state manager re-syncs.
void notify_release(CAI_Stalker& actor);

// Side missions (v2.9 SideSnapshot::Mission), authored by a test or a mission hook through
// level.painter_combat_set_mission / painter_combat_clear_mission and read by the registry when it builds a side's
// snapshot. side = the key ActorSnapshot::native_squad carries (level.painter_combat_side_of(npc)) or the registry's own
// group id. Every set and every clear bumps that side's revision (a cleared side reads back kind none with the new
// revision, so the core sees the change); missions go with the level.
std::uint64_t side_key(const CEntityAlive& entity);           // team << 32 | squad << 16 | group (native_squad)
void set_mission(std::uint64_t side, const Mission& mission);   // the stored revision is the side's next, not mission's
void clear_mission(std::uint64_t side);
bool mission_of(std::uint64_t side, Mission& out);              // false when the side never had one on this level

// Throw and aid receipts (v2.9 ActorSnapshot::throw_state / aid_state): the executor reports what it started, capture
// reports the progress. begin_throw when the executor hands a throw order to the engine's aimed throw
// (CAI_Stalker::painter_throw), with the order's serial (its at_ms is the submission mark): capture then reports
// prepared -> in_hand (the grenade armed, ready or being thrown) -> thrown only on the engine's own release: a live
// projectile leaving his hand after begin (note_release, from the hand grenade's ownership reject of its projectile,
// which the throw's motion mark sends). Never from a grenade count drop, the throwing animation or the stock
// on_throw_completed (the stock throw action calls that on an inventory change); failed only through the executor's
// end_throw. The reason is the engine's (painter_throw_state) or the executor's. Aid has no engine state machine:
// note_aid carries the executor's phases as they are. A new begin / note replaces the receipt; 0 clears.
// Reasons: ThrowReason (CombatContract.h, v2.13).
void begin_throw(std::uint16_t id, std::uint32_t serial);
void end_throw(std::uint16_t id, std::uint8_t reason);          // the executor gave up: failed with reason; 0 clears
void note_aid(std::uint16_t id, ActorKey target, std::uint8_t phase);   // phase 0 clears
// CMissile::OnEvent(GE_OWNERSHIP_REJECT) of the live projectile (main thread, game events): `thrower` let go of
// `projectile` from the hand item `item`. Counted per thrower for the level.
void note_release(std::uint16_t thrower, std::uint16_t projectile, std::uint16_t item);

// v2.18 ActorSnapshot::aid_eligibility: his own side's script state as the wounded one, from
// level.painter_combat_set_aid(npc, allowed, claimed, claimant_id) (the Lua producer pushes on change): may he be helped
// (xr_wounded not_for_help) and the stock helper's claim on him (claimant 65535 = none). Kept per object id with the time;
// capture copies it (allowed is unknown until the first push); dropped at level load and net_Destroy.
void set_aid(std::uint16_t id, bool allowed, bool claimed, std::uint16_t claimant);
// Read-only script permission receipt, original change time retained; no admission.
decltype(ActorSnapshot{}.aid_eligibility) aid_eligibility_of(std::uint16_t id,TimeMs now,
    const std::function<ActorKey(const CEntityAlive&)>& key_of);

}}} // namespace painter::combat::engine
