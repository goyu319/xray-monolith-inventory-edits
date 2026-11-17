#pragma once
#include "CombatOutdoorBounds.h"
#include "CombatOutdoorSensors.h"
#include "CombatOutdoorPersonal.h"
#include "CombatOutdoorFire.h"
#include "CombatOutdoorSolo.h"
#include "CombatOutdoorAssessment.h"
#include "CombatOutdoorMovement.h"
#include "CombatOutdoorWithdrawal.h"
#include "CombatOutdoorSelf.h"
#include "CombatRouteProof.h"
#include <map>

namespace painter { namespace combat { namespace core {
// Explicit private opt-in; engine settings allocation belongs to the owner.
// OutdoorDispatchPolicy is allocated in ../OutdoorConfig.h.
struct OutdoorCaptureEvidence {
    // World portion authenticates the optional full-route rows. Line rows carry
    // their own actor/source stamps. Query profiles are produced by our worker.
    OutdoorStamp source;
    std::vector<OutdoorMeasuredLine> lines;
    std::vector<OutdoorSearchObservation> search_observations;
    std::vector<OutdoorBoundPath> paths;
    std::vector<OutdoorCapturedEye> eyes;
    std::vector<OutdoorGrenadeFuse> grenade_fuses;
    std::vector<OutdoorBuddyStep> buddy_steps;
    std::vector<OutdoorFallbackEvidence> fallbacks;
    std::vector<OutdoorSoloRoute> solo_routes;
    std::vector<OutdoorSoloThrow> solo_throws;
    std::vector<OutdoorSoloGrenadeEscape> grenade_escapes;
    std::vector<OutdoorWithdrawalCandidate> withdrawal_candidates;
    // Optional native capture receipts. Absence/duplicates supply no proof;
    // the Solo consumer revalidates exact actor/focus/lease and applied order.
    std::vector<OutdoorSoloAimEvidence> solo_aim;
    std::vector<OutdoorSoloLeanCapture> solo_lean;
    RouteProofService route_service; // optional value-queue bridge; unknown until authority-certified
};
struct OutdoorDispatchReceipt {
    OutdoorPostureResult posture;
    OutdoorExecutionResult execution;
    OutdoorBoundsResult bounds;
    std::vector<OutdoorPersonalResult> personal;
    OutdoorFireResult fire;
    std::vector<OutdoorSoloResult> solo;
    OutdoorAssessmentResult assessment;
    OutdoorMovementResult movement;
    OutdoorWithdrawalExecution withdrawal_execution;
    OutdoorWithdrawalResult withdrawal;
    std::vector<OutdoorSelfResult> self;
    SquadView effective_elements;
    bool elements_swapped=false;
};
struct OutdoorDirectorResult {
    OutdoorStatus status=OutdoorStatus::invalid;
    std::vector<ProposedOrder> overrides;
    std::vector<OutdoorDispatchReceipt> sides;
    OutdoorExecutionCosts index_cost;
    std::vector<SuppressedFoeFact> executed_suppression;
    std::size_t route_work=0,route_completed=0;
};
// Value-only composition. Work owns a staged copy, which may commit only after
// current capture/authority/freshness validation. No native acceptance is inferred.
class CombatOutdoorDirector {
public:
    CombatOutdoorDirector(Config,OutdoorDispatchPolicy);
    OutdoorDirectorResult update(const WorldSnapshot&,const KnowledgeFrame&,
        const std::vector<CombatMissionIntel>&,const SquadResult&,
        const std::vector<Order>& current,const std::vector<Order>& executed,
        const std::vector<ActorCoverCache>&,const OutdoorExecutionEvidence&,
        const OutdoorCaptureEvidence&,const std::function<bool()>& cancelled={},const EngineServices& services={},
        // Null is an explicit standalone/full-publication call. Dispatcher
        // supplies its exact cohort; empty means no personal work may publish.
        const std::vector<ActorKey>* publication_cohort=nullptr);
    void reset_side(std::uint32_t);
private:
    struct SideState {
        SideState(Config c,OutdoorDispatchPolicy p):posture(c,p.posture),bounds(c,p.bounds),movement(c,p.movement),withdrawal(c,p.withdrawal){}
        CombatOutdoorPosture posture;CombatOutdoorBounds bounds;
        CombatOutdoorMovement movement;
        CombatOutdoorWithdrawal withdrawal;
        // Shared immutable checkpoint. Each update mutates a deep clone.
        std::shared_ptr<const CombatOutdoorFire> fire;
        std::shared_ptr<const CombatOutdoorAssessment> assessment;
        std::map<std::pair<std::uint32_t,std::uint64_t>,std::shared_ptr<const CombatOutdoorSolo>> solo;
        std::map<std::pair<std::uint32_t,std::uint64_t>,std::shared_ptr<const CombatOutdoorSelf>> self;
        std::map<std::pair<std::uint32_t,std::uint64_t>,OutdoorGrenadeMemory> grenades;
        ActorKey solo_after,route_after,push_actor;
        OutdoorStamp element_source;
        SquadView elements;
        std::vector<std::pair<ActorKey,std::uint64_t>> element_owners;
        struct BoundLeg {ActorKey actor;std::uint64_t owner=0,restrictions=0;
            std::uint32_t node=invalid_node;Vec3 goal;};
        std::vector<BoundLeg> bound_legs;
        OutdoorSupportFailure support_failure;
    };
    Config config_;OutdoorDispatchPolicy policy_;
    std::map<std::uint32_t,SideState> states_;
    std::uint32_t after_side_=0;
};
}}}
