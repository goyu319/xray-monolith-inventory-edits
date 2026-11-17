#pragma once
#include "CombatOutdoorPosture.h"
#include "CombatRouteProof.h"
#include <array>
#include <memory>

namespace painter { namespace combat { namespace core {
enum class OutdoorExecutionStatus : std::uint8_t { invalid,ready,cancelled,failed };
struct OutdoorExecutionCosts {
    std::size_t actor_rows=0,order_rows=0,cache_rows=0,profile_rows=0,line_rows=0,members=0,spots=0;
};
// Private proof from the producer of a real query, not an engine allocation or
// configuration default. Coordinates are captured values, never target lookups.
struct OutdoorQuerySource {
    ActorKey actor;
    std::uint64_t level_epoch=0,snapshot_seq=0,geometry_revision=0,ownership_generation=0,restrictions_revision=0;
    TimeMs at_ms=0;
    std::uint32_t origin_node=invalid_node;
    Vec3 actor_position;
    FoeId focus;TimeMs focus_at_ms=0;Vec3 focus_position;
};
struct OutdoorCoverProfile {
    OutdoorQuerySource source;
    std::array<float,3> heights{};
};
struct OutdoorMeasuredLine {
    OutdoorQuerySource source;
    Vec3 measured_eye;
    Evidence clear=Evidence::unknown;
};
struct OutdoorSearchObservation {
    OutdoorQuerySource source;
    std::uint64_t applied_order_seq=0;
    Vec3 measured_eye,observed_point,head;
    // Authority-thread ray includes dynamic objects. Static worker LOS alone
    // cannot establish visibility of an old point through a closed door/prop.
    Evidence clear=Evidence::unknown;
};
struct OutdoorExecutionEvidence {
    std::vector<OutdoorCoverProfile> profiles;
    std::vector<OutdoorMeasuredLine> lines;
    std::vector<OutdoorSearchObservation> search_observations;
};
struct OutdoorExecutionResult {
    OutdoorExecutionStatus status=OutdoorExecutionStatus::invalid;
    OutdoorExecutionFacts facts;
    std::size_t applied=0,unconfirmed=0,cover_unknown=0;
    OutdoorExecutionCosts costs;
};
// Attach a complete current authority route to an exact actually applied goal.
// A candidate's route, old origin, proposal, or pending native query is no proof.
bool observe_outdoor_route(const Config&,const WorldSnapshot&,const ActorSnapshot&,
    const RouteProofReceipt&,OutdoorActorExecution&);
// A borrowed, immutable value-batch index. The caller must keep WorldSnapshot,
// orders, caches and evidence alive and UNMODIFIED until every side call ends.
// No engine object or live manager pointer is accepted. One index per captured
// batch avoids rebuilding whole-world/ledger maps for every side.
class OutdoorExecutionIndex {
public:
    OutdoorExecutionStatus status() const{return status_;}
    const OutdoorExecutionCosts& costs()const{return costs_;}
private:
    struct Data;std::shared_ptr<const Data> data_;
    OutdoorExecutionStatus status_=OutdoorExecutionStatus::invalid;OutdoorExecutionCosts costs_;
    friend OutdoorExecutionIndex index_outdoor_execution(const WorldSnapshot&,const std::vector<Order>&,
        const std::vector<ActorCoverCache>&,const OutdoorExecutionEvidence&,const std::function<bool()>&);
    friend OutdoorExecutionResult outdoor_execution_facts(const Config&,const OutdoorExecutionIndex&,
        const SideSnapshot&,const SquadView&,const OutdoorPostureResult&,const std::function<bool()>&);
};
OutdoorExecutionIndex index_outdoor_execution(const WorldSnapshot&,const std::vector<Order>&,
    const std::vector<ActorCoverCache>&,const OutdoorExecutionEvidence&,
    const std::function<bool()>& cancelled={});
OutdoorExecutionResult outdoor_execution_facts(const Config&,const OutdoorExecutionIndex&,
    const SideSnapshot&,const SquadView&,const OutdoorPostureResult&,
    const std::function<bool()>& cancelled={});
// Convenience compatibility entry. Without explicit query-profile and measured
// current-eye receipts, static cover and current line stay UNKNOWN. Applied
// Order.stance is a request, not proof that the body reached that posture.
// Route, searched-place, endpoint, assessment and door stay unknown in all APIs.
OutdoorExecutionResult outdoor_execution_facts(const Config&,const WorldSnapshot&,
    const SideSnapshot&,const SquadView&,const OutdoorPostureResult&,
    const std::vector<Order>&,const std::vector<ActorCoverCache>&,
    const std::function<bool()>& cancelled={});
}}}
