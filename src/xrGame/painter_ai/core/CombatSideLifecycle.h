#pragma once
#include "CombatSideRegistry.h"
#include "CombatKnowledge.h"
#include "PerceivedSpecies.h"
#include <memory>

namespace painter { namespace combat { namespace core {

// Caller-supplied PROPOSAL until canonical controller allocation. No hidden
// policy defaults and no engine admission, gameplay orders or scheduler here.
struct SideLifecyclePolicy {
    TimeMs quiet_field_ms,quiet_building_ms,stand_down_after_ms,side_cooldown_ms,error_cooldown_ms;
    bool retain_authored_mission_without_evidence;
    SideLifecyclePolicy()=delete;
    SideLifecyclePolicy(TimeMs field,TimeMs building,TimeMs relax,TimeMs cooldown,TimeMs error,
        bool retain_authored):quiet_field_ms(field),quiet_building_ms(building),stand_down_after_ms(relax),
        side_cooldown_ms(cooldown),error_cooldown_ms(error),retain_authored_mission_without_evidence(retain_authored){}
};
enum class SideLifecycleStatus : std::uint8_t { invalid,accepted,stale,cancelled,failed };
enum class SideLifecyclePhase : std::uint8_t { engaged,stand_down,released,cooldown,awaiting_admission };
enum class SideReleaseReason : std::uint8_t { none,fight_over,building_cleared,no_living_men,error,identity_reset,monster_fight,out_of_range,door_stuck };
// Current explicit mission authority requires matching captured living
// membership and requested control from every living admitted member.
// An absent roster key grants no control lease or proof of death.
bool requested_authored_mission(const WorldSnapshot& world,const SideSnapshot& side);
enum class LifecycleEvidenceKind : std::uint8_t { none,contact,hit,heard,near_miss,own_seen,own_heard,reported_seen,reported_heard };
struct LifecycleEvidence {
    bool valid=false;LifecycleEvidenceKind kind=LifecycleEvidenceKind::none;
    ActorKey observer,reporter;std::uint64_t ownership_generation=0;FoeId foe;
    TimeMs at_ms=0; // original source time, NEVER lifecycle intake/delivery time
    bool inferred_attack=false; // ghost bearing is not proof of a live hostile's identity/species
};
struct LifecycleIdentity {
    std::uint64_t level_epoch=0,map_hash=0,geometry_revision=0,generation=0,group=0;
    std::uint32_t side=0,faction=0,mission_revision=0;
};
struct LifecycleLease {ActorKey actor;std::uint64_t ownership_generation=0;};
struct LifecycleReceiptStamp {
    LifecycleIdentity identity;std::uint64_t source_snapshot=0;TimeMs at_ms=0;
};
struct BuildingCompletionReceipt {
    LifecycleReceiptStamp stamp;std::uint32_t building=invalid_room;
    bool whole_building=false; // one RoomDoctrine ready_for_next is NOT this receipt
};
struct LifecycleRangeReceipt {
    LifecycleReceiptStamp stamp;Vec3 reference_position;
};
struct LifecycleDoorReceipt {
    LifecycleReceiptStamp stamp;std::uint32_t portal=0;
    // Non-entry portals are supplied by the native accepted-order ledger,
    // never a planner's still-unaccepted desired assignment.
    ActorKey actor;std::uint64_t ownership_generation=0,applied_order_seq=0;
};
struct LifecycleSignals {
    std::vector<BuildingCompletionReceipt> completed;
    std::vector<LifecycleReceiptStamp> errors,identity_resets;
    PerceivedSpeciesEvidence perceived_species;
    std::vector<LifecycleRangeReceipt> ranges;
    std::vector<LifecycleDoorReceipt> required_doors;
};
struct RetainedControlLease {
    LifecycleIdentity identity;LifecycleLease member;std::uint64_t source_snapshot=0;
    TimeMs captured_at_ms=0,quiet_deadline_ms=0;
    bool authored_mission_retention=false; // no expiry fabrication or fake Contact
};
struct LifecycleRelease {
    LifecycleIdentity identity;LifecycleLease member;SideReleaseReason reason=SideReleaseReason::none;
    std::uint64_t source_snapshot=0;TimeMs released_at_ms=0,cooldown_until_ms=0;
};
struct SideLifecycleView {
    LifecycleIdentity identity;SideLifecyclePhase phase=SideLifecyclePhase::awaiting_admission;
    LifecycleEvidence evidence;TimeMs activated_at_ms=0,quiet_origin_ms=0,quiet_deadline_ms=0,cooldown_until_ms=0;
    SideReleaseReason release_reason=SideReleaseReason::none;
    bool building_context=false,roster_complete=false,has_living_member=false;
    bool distance_known=false;float distance_m=0,distance_cost=0;
};
// Immutable capture-to-admission sidecar. Native reads a copied committed
// result before taking ownership; it never reads the worker's mutable state.
struct LifecycleCooldown {
    LifecycleIdentity identity;std::uint64_t native_group=0;
    TimeMs until_ms=0;SideReleaseReason reason=SideReleaseReason::none;
    std::vector<ActorKey> members;
};
struct SideLifecycleResult {
    SideLifecycleStatus status=SideLifecycleStatus::invalid;bool accepted=false,cancelled=false;
    std::uint64_t level_epoch=0,map_hash=0,snapshot_seq=0;TimeMs captured_at_ms=0;
    std::vector<SideLifecycleView> sides;
    std::vector<RetainedControlLease> retained;
    std::vector<LifecycleRelease> releases;
    std::size_t ignored_receipts=0,ignored_knowledge_rows=0;
    std::vector<LifecycleCooldown> cooldowns;
};
bool lifecycle_admission_blocked(const std::vector<LifecycleCooldown>&,
    const ActorSnapshot&,std::uint64_t epoch,std::uint64_t map,TimeMs now);
const char* lifecycle_release_reason(SideReleaseReason);

// Atomic/cancellable, snapshot-only controller foundation. Registry and optional
// K1 frames must be accepted and from THIS capture. No hidden hostile census.
// Exact current ownership/membership/mission/geometry changes invalidate older
// completion/error receipts. A release is an idempotent intent for its exact
// lease, not an assertion that native publication accepted it. Released leases
// are never re-retained; a genuinely new admitted lease is required after cooldown.
class CombatSideLifecycle {
public:
    explicit CombatSideLifecycle(SideLifecyclePolicy,const Config& config=Config{});
    ~CombatSideLifecycle();
    CombatSideLifecycle(const CombatSideLifecycle&)=delete;
    CombatSideLifecycle& operator=(const CombatSideLifecycle&)=delete;
    std::unique_ptr<CombatSideLifecycle> clone(const std::function<bool()>& cancelled={}) const;
    SideLifecycleResult update(const WorldSnapshot&,const SideRegistryResult&,const KnowledgeFrame*,
        const LifecycleSignals& signals={},const std::function<bool()>& cancelled={});
    void reset();
private:
    struct Impl;SideLifecyclePolicy policy_;std::unique_ptr<Impl> impl_;
    float release_range_m_,distance_cost_per_m_;
    TimeMs required_door_stuck_ms_;
};
}}}
