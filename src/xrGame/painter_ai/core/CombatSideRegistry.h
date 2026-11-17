#pragma once
#include "CombatCore.h"
#include "FriendlyCasualty.h"
#include <functional>
#include <memory>
#include <vector>
namespace painter { namespace combat { namespace core {
enum class SideRegistryStatus : std::uint8_t { accepted,invalid_input,stale,cancelled,identity_exhausted };
enum class SideResetReason : std::uint8_t { epoch_or_map,expired,explicit_identity_reuse,explicit_generated_collision,disjoint_native_roster };
struct SideRegistryReset {
    std::uint32_t side=0,previous_faction=0,current_faction=0;
    std::uint64_t previous_group=0,current_group=0;
    SideResetReason reason=SideResetReason::expired;
};
struct SideLeaseChange {ActorKey actor;std::uint32_t side=0;std::uint64_t previous_owner=0,current_owner=0;};
enum class SideConflictReason : std::uint8_t { incomplete_native_roster,split_explicit_native_squad };
struct SideRegistryConflict {
    SideConflictReason reason=SideConflictReason::incomplete_native_roster;
    std::uint32_t faction=0;std::uint64_t native_squad=0;
    std::vector<std::uint32_t> explicit_sides;
    std::vector<ActorKey> omitted; // these captured keys get no auto side; caller retains safe individual fallback
};
struct FriendlySideMembership {std::uint32_t side_id=0,faction=0,mission_revision=0;std::uint64_t group=0;ActorKey member;TimeMs known_at_ms=0;};
struct SideRegistryResult {
    SideRegistryStatus status=SideRegistryStatus::invalid_input;bool accepted=false,cancelled=false;
    std::uint64_t level_epoch=0,map_hash=0,snapshot_seq=0;TimeMs captured_at_ms=0;
    std::vector<SideSnapshot> sides;
    std::vector<FriendlySideMembership> friendly_memberships; // retained identity only: NOT current life, health or position
    std::vector<FriendlyCasualtyFact> friendly_casualties; // actual perceived passive members only
    std::vector<SideRegistryReset> resets; // consumer must invalidate these IDs before K1 consumes the result
    std::vector<SideLeaseChange> lease_changes; // assignment authority only; ActorKey's personal memory survives a lease change
    std::vector<SideRegistryConflict> conflicts;
    std::size_t generated_sides=0,generated_members=0,unassigned_conflicts=0,pair_checks=0;
};
// Shared identity only: no admission, species inference, contacts, mission
// creation, posture, side quiet/cooldown or independent scheduler. The engine
// supplies admitted captures. Explicit rosters may retain absent/dead members.
// Generated grouping uses finite current admitted living snapshots and existing
// Config near/regroup distances, with deterministic complete-link proximity.
class CombatSideRegistry {
public:
    explicit CombatSideRegistry(Config);
    ~CombatSideRegistry();
    CombatSideRegistry(const CombatSideRegistry&)=delete;
    CombatSideRegistry& operator=(const CombatSideRegistry&)=delete;
    std::unique_ptr<CombatSideRegistry> clone(const std::function<bool()>& cancelled={}) const;
    SideRegistryResult update(const WorldSnapshot&,TimeMs retain_ms,const std::function<bool()>& cancelled={},
        const FriendlyCasualtyEvidence& passive={});
    void reset();
private:
    struct Impl;
    Config config_;
    std::unique_ptr<Impl> impl_;
};
}}}
