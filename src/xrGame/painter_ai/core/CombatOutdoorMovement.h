#pragma once
#include "CombatOutdoorSolo.h"

namespace painter { namespace combat { namespace core {
// Private caller data until allocated by the engine owner. No live defaults.
// OutdoorMovementPolicy is allocated in ../OutdoorConfig.h.
struct OutdoorMovementQuota {std::size_t work_left=0,queries_left=0,work_used=0,queries_used=0;};
struct OutdoorMovementRequest {
    const WorldSnapshot* world=nullptr;const SideSnapshot* side=nullptr;
    const KnowledgeFrame* knowledge=nullptr;const CombatMissionIntel* mission=nullptr;
    const OutdoorPostureResult* posture=nullptr;const SquadView* elements=nullptr;
    const std::vector<ProposedOrder>* baseline=nullptr;
    const std::vector<Order>* applied=nullptr;
    // Whole actor-restricted routes, when provided, retain their exact query
    // source stamp. Static path length alone never becomes that evidence.
    const std::vector<OutdoorSoloRoute>* routes=nullptr;
    const RouteProofService* route_service=nullptr;RouteProofBudget* route_budget=nullptr;
};
struct OutdoorMovementResult {
    OutdoorStatus status=OutdoorStatus::invalid;OutdoorStamp stamp;
    const char* reason="invalid";
    std::vector<ProposedOrder> proposals;
    std::size_t work=0,queries=0,pending_navigation=0,continued=0,arrived=0;
    bool exhausted=false,investigating=false;
};
// Search, personally-sourced assault and mission approach destinations. This
// does not clear locations merely by arriving, invent routes through buildings,
// or grant enemy firing from shared knowledge. Native publication remains the
// final actor-restriction gate. State is copied with the outer Work transaction.
class CombatOutdoorMovement {
public:
    CombatOutdoorMovement(Config,OutdoorMovementPolicy);
    OutdoorMovementResult update(const OutdoorMovementRequest&,const EngineServices&,
        OutdoorMovementQuota&,const std::function<bool()>& cancelled={});
    void reset();
private:
    struct Issued {
        ActorKey actor;std::uint64_t owner=0,restrictions=0,previous=0;
        TimeMs at_ms=0;std::uint32_t node=invalid_node;Vec3 point;
        OrderKind kind=OrderKind::hold;FoeId focus;TimeMs focus_at=0;
        Vec3 objective;std::uint32_t objective_portal=0;ActorKey sound_reporter;bool sound_report=false;
    };
    Config config_;OutdoorMovementPolicy policy_;OutdoorStamp stamp_,cursor_stamp_;
    struct Investigated {ActorKey actor;std::uint64_t owner=0;TimeMs at=0;Vec3 point;};
    struct PendingSound {ActorKey actor;std::uint64_t owner=0,restrictions=0;LooseContact sound;bool blocked=false;};
    ActorKey after_;std::vector<Issued> issued_;std::vector<Investigated> investigated_;std::vector<PendingSound> pending_sounds_;
};
}}}
