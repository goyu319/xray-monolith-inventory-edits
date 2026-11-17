#pragma once
#include "CombatOutdoorExecution.h"
#include "SquadTactics.h"

namespace painter { namespace combat { namespace core {

// Explicit private proposal data, pending canonical allocation. Zero values
// are invalid; there are no runtime defaults or faction-specific constants.
// OutdoorBoundPolicy is allocated in ../OutdoorConfig.h.
// Optional full route, obtained from a real path producer. A length, an
// endpoint ray or the direct actor->goal segment is NOT this receipt.
struct OutdoorBoundPath {
    ActorKey actor;std::uint64_t owner=0,restrictions_revision=0;
    std::uint32_t origin_node=invalid_node,goal_node=invalid_node;
    Vec3 goal_position;Evidence complete=Evidence::unknown;
    std::vector<Vec3> points;
};
struct OutdoorBoundsRequest {
    const WorldSnapshot* world=nullptr;const SideSnapshot* side=nullptr;
    const KnowledgeFrame* knowledge=nullptr;const OutdoorPostureResult* posture=nullptr;
    const OutdoorExecutionFacts* execution=nullptr;const SquadView* elements=nullptr;
    const std::vector<Order>* accepted_orders=nullptr;
    const std::vector<ActorCoverCache>* cover=nullptr;
    // Exact query producer's profile; blocked[] alone does not identify its heights.
    const OutdoorExecutionEvidence* query_evidence=nullptr;
    // This stamp authenticates every optional path row against THIS capture.
    OutdoorStamp path_stamp;const std::vector<OutdoorBoundPath>* paths=nullptr;
};
enum class OutdoorBoundStatus : std::uint8_t { invalid,ready,cancelled,failed };
struct OutdoorBoundEndpoint {
    ProposedOrder proposal;OutdoorCover cover_class=OutdoorCover::unknown;
    Evidence restricted_route=Evidence::unknown,full_route_fire_clearance=Evidence::unknown;
    bool continuing=false,door_objective=false;
    // Still required for unknown paths and at execution time for ALL paths.
    bool live_friendly_fire_guard_required=true;
    FoeId focus;TimeMs focus_at_ms=0;
};
struct OutdoorBoundsResult {
    OutdoorBoundStatus status=OutdoorBoundStatus::invalid;OutdoorStamp stamp;
    const char* reason="invalid";
    std::vector<ActorKey> ready_base;
    // Formation proposals are not readiness. Movers still require the actual
    // arrived, firing base certified above; these orders establish one first.
    std::vector<ProposedOrder> base_positions;
    std::vector<OutdoorBoundEndpoint> endpoints;
    OutdoorSupportFailure support_failure;
    // Independent approach, never a bound and never a claim of covering fire.
    std::vector<ProposedOrder> support_reposition;
    const char* support_reason="not_requested";
    std::size_t base_evaluated=0,base_invalid=0,base_protection=0,base_restricted=0;
    std::size_t recovery_pending=0,recovery_unsafe=0,recovery_occupied=0;
    std::size_t pending_navigation=0,unknown_route_clearance=0,rejected_candidates=0,examined_candidates=0;
};

// One authorized ATTACK bound slice. No posture selection, admission, role
// assignment, assault, withdrawal, base-fire creation or native acceptance.
// Unhandled/blocked men keep their existing solo/cover orders. Actual accepted
// bounds retain their destination through contact and reload. Cancellation
// never commits the small recent-fired ledger or any partial endpoint list.
class CombatOutdoorBounds {
public:
    CombatOutdoorBounds(Config,OutdoorBoundPolicy);
    OutdoorBoundsResult update(const OutdoorBoundsRequest&,const std::function<bool()>& cancelled={});
    void reset();
private:
    struct Fired {ActorKey actor;std::uint64_t owner=0;FoeId focus;TimeMs at_ms=0;};
    Config config_;OutdoorBoundPolicy policy_;OutdoorStamp stamp_;std::vector<Fired> fired_;
    // Authorship alone grants no continuation: the next capture must match the
    // actually applied goal/lease and certify its current restricted route.
    Order recovery_;OutdoorStamp recovery_stamp_;FoeId recovery_focus_;Vec3 recovery_focus_position_;
};
}}}
