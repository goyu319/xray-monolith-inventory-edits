#pragma once
#include "CombatOutdoorExecution.h"
#include "CombatOutdoorBounds.h"
#include "CombatRouteProof.h"

namespace painter { namespace combat { namespace core {
// Explicit PRIVATE policy: zero distances/durations are invalid, never live defaults.
// Missing canonical outdoor allocations are listed in the accompanying receipt.
// OutdoorPersonalPolicy is allocated in ../OutdoorConfig.h.
enum class OutdoorPersonalStatus : std::uint8_t { invalid,disabled,ready,cancelled,failed };
enum class OutdoorPersonalReason : std::uint8_t {
    unchanged,native_yield,committed_room,committed_bound,hands,reload,
    unseen_escape,escape_unavailable,dogfight,continue_cover,personal_contact,squad,committed_withdrawal,grenade_escape,grenade_wait
};
struct OutdoorPersonalReservation {ActorKey actor;std::uint32_t node=invalid_node;Vec3 position;};
struct OutdoorPersonalReservations {
    std::uint64_t level_epoch=0,snapshot_seq=0;
    std::vector<OutdoorPersonalReservation> members;
};
// A side-local receipt from acknowledged bound execution. Borrowed mover
// capture/order are ordinary immutable values, not engine objects. The producer
// must select a member of the same current side; the local reservation roster
// and exact native application are checked again here.
struct OutdoorBaseCommitment {
    std::uint64_t level_epoch=0,snapshot_seq=0,geometry_revision=0;
    std::uint32_t side_id=0;
    ActorKey base;
    std::uint64_t ownership_generation=0,restrictions_revision=0,order_seq=0,assignment_generation=0;
    const ActorSnapshot* mover=nullptr;
    const Order* mover_applied=nullptr;
};
// Source-qualified acknowledgment from the current outdoor execution batch.
// Outdoor bounds intentionally have assignment.generation==0. A receipt is
// NOT inferred from a role, proposed goal or a fabricated assignment token.
struct OutdoorBoundCommitment {
    std::uint64_t level_epoch=0,snapshot_seq=0,geometry_revision=0;
    std::uint32_t side_id=0;
    ActorKey actor;
    std::uint64_t ownership_generation=0,restrictions_revision=0,applied_order_seq=0;
};
// An assessed field withdrawal is not a room assignment. Its producer validates
// the safe candidate/episode; continuation additionally names actual execution.
struct OutdoorWithdrawalCommitment {
    std::uint64_t level_epoch=0,snapshot_seq=0,geometry_revision=0,assessment_serial=0;
    std::uint32_t side_id=0;ActorKey actor;
    std::uint64_t ownership_generation=0,restrictions_revision=0,applied_order_seq=0;
    std::uint32_t goal_node=invalid_node;Vec3 goal_position;bool continuing=false;
};
// Optional current immutable geometry and route receipts. Work/ray counters
// belong to the whole Director update; no actor or destination resets them.
struct OutdoorGrenadeFuse {
    ActorKey actor;std::uint64_t epoch=0,sequence=0,owner=0,restrictions=0;
    TimeMs observed_at=0,perceived_at=0;Vec3 position;TimeMs until=0;
};
struct OutdoorGrenadeMemory {
    ActorKey actor;std::uint64_t epoch=0,geometry=0,owner=0,restrictions=0;
    Danger perceived;TimeMs until=0;bool fuse_measured=false;std::uint32_t goal_node=invalid_node;Vec3 goal;
};
struct OutdoorEscapeContext {
    const EngineServices* services=nullptr;
    OutdoorStamp source;
    const std::vector<OutdoorBoundPath>* paths=nullptr;
    const RouteProofService* route_service=nullptr;RouteProofBudget* route_budget=nullptr;
    std::size_t* work_left=nullptr;std::size_t* queries_left=nullptr;
    std::size_t* work_used=nullptr;std::size_t* queries_used=nullptr;
};
struct OutdoorPersonalInput {
    ProposedOrder base;
    // Exact native-accepted ledger row selected by last_applied.order_seq, not
    // simply its newest row. Borrowed immutable values live only for this call.
    const Order* applied=nullptr;
    const OutdoorBaseCommitment* base_commitment=nullptr;
    const OutdoorBoundCommitment* bound_commitment=nullptr;
    const ActorCoverCache* cover=nullptr;
    const OutdoorCoverProfile* cover_profile=nullptr;
    const KnowledgeFrame* knowledge=nullptr;
    std::size_t knowledge_index=static_cast<std::size_t>(-1);
    std::uint32_t side_id=0;
    const OutdoorPersonalReservations* reservations=nullptr;
    const OutdoorWithdrawalCommitment* withdrawal_commitment=nullptr;
    // Authored Self intent, not a claim that native weapon state is reloading.
    // Commitments/native hands still win; only actual safe cover motion may
    // continue, otherwise this selects a stationary reload request.
    bool tactical_reload=false;
    const OutdoorEscapeContext* escape_context=nullptr;
    OutdoorGrenadeMemory* grenade_memory=nullptr;
    const OutdoorGrenadeFuse* grenade_fuse=nullptr;
};
struct OutdoorPersonalResult {
    OutdoorPersonalStatus status=OutdoorPersonalStatus::invalid;
    OutdoorPersonalReason reason=OutdoorPersonalReason::unchanged;
    OutdoorGrenadeMemory grenade;
    ProposedOrder proposal;
    bool personal_contact=false,acknowledged_motion=false;
    // Blocked chest ray is a fixed query profile, NOT measured current pose/FOV.
    bool escaped_line_proven=false;
    std::size_t candidates=0,reservations=0;
    // Existing exact base/mover receipt passed validation. This is commitment
    // evidence only; it does not claim the base fired or block native reload.
    bool acknowledged_base=false;
};
// Per-man, stateless decorator. Caller supplies an O(1) K1 index and a side-local
// current reservation slice; no whole-world scan, engine object or synchronous
// nav/ray call. Unknown navigation stays pending for native publication.
// Neither this selector nor its execution intents certify arrival/fire/handback.
// Prefer this exact urgent hit source before the ordinary tactical query focus
// when Personal policy is enabled. It performs no query. The batch caller first
// validates accepted current K1 frame/side membership and indexes this member;
// this function checks actor source, identity, lease and the measured hit facts.
bool outdoor_personal_cover_focus(const Config&,const WorldSnapshot&,const ActorSnapshot&,
    const MemberKnowledge&,KnowledgeTrace&);
OutdoorPersonalResult outdoor_personal_order(const Config&,const OutdoorPersonalPolicy&,
    const WorldSnapshot&,const ActorSnapshot&,const OutdoorPersonalInput&,
    const std::function<bool()>& cancelled={});
}}}
