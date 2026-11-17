#pragma once
#include "CombatOutdoorExecution.h"
namespace painter { namespace combat { namespace core {
// Explicit caller policy from the outdoor doctrine; zero initialization is
// invalid. Faction morale / loyalty come only from canonical Config data.
// OutdoorAssessmentPolicy is allocated in ../OutdoorConfig.h.
struct OutdoorFallbackRoute {
    ActorKey actor;std::uint64_t owner=0,restrictions_revision=0;
    std::uint32_t origin_node=invalid_node,goal_node=invalid_node;Vec3 goal_position;
    Evidence route=Evidence::unknown,cover=Evidence::unknown;
};
struct OutdoorFallbackLine {
    FoeId foe;SourceFact source;
    // A real static query from the exact known source position to the explicit
    // fallback point. 'no' means blocked; absence/unknown never proves safety.
    Vec3 ray_from,ray_to;
    Evidence visible=Evidence::unknown;
};
struct OutdoorFallbackEvidence {
    OutdoorStamp stamp;std::uint64_t revision=0;
    std::uint32_t node=invalid_node;Vec3 point;
    std::vector<OutdoorFallbackRoute> routes;
    std::vector<OutdoorFallbackLine> lines;
};
// Caller linkage from the actual accepted episode ledger. A posture/serial or
// unrelated native withdrawal action alone cannot spend this assessment.
struct OutdoorWithdrawalLegReceipt {
    OutdoorStamp stamp;std::uint64_t assessment_serial=0;
    ActorKey actor;std::uint64_t owner=0,restrictions_revision=0,order_seq=0;
    std::uint32_t goal_node=invalid_node;Vec3 goal_position;
};
struct OutdoorAssessmentRequest {
    OutdoorPosture posture=OutdoorPosture::quiet;ActorKey leader;
    const OutdoorFallbackEvidence* fallback=nullptr;
    // Only actual accepted+applied withdraw orders of this issued assessment
    // episode may start cooldown or earn completion. Bare posture is no proof.
    std::uint64_t executing_serial=0;
    const OutdoorExecutionFacts* execution=nullptr;
    std::vector<OutdoorWithdrawalLegReceipt> execution_legs;
};
struct OutdoorAssessmentResult {
    OutdoorStatus status=OutdoorStatus::invalid;OutdoorStamp stamp;
    OutdoorAssessment verdict=OutdoorAssessment::unknown;
    const char* reason="invalid";std::uint64_t serial=0;
    // The withdrawal episode remains identifiable on its completing verdict,
    // whose ordinary serial advances when withdraw changes to proceed.
    std::uint64_t withdrawal_episode_serial=0;
    std::size_t roster=0,observed=0,able=0,hurt=0,dead=0,unknown=0,known=0,unseen=0,ours=0,theirs=0,downs=0;
    float mean_health=0;
    std::size_t hits_taken=0,foe_wounded=0,witnessed_severs=0;
    float advance_cost=0,hold_cost=0;
    bool exchange_hold=false;
    TimeMs exchange_at_ms=0,exchange_until_ms=0;
    bool broken=false,outmatched=false,losing=false,pressure=false;
    Evidence safe_fallback=Evidence::unknown,casualty_obligation_clear=Evidence::unknown;
    std::uint32_t fallback_node=invalid_node;Vec3 fallback;
    TimeMs pending_since_ms=0,cooldown_remaining_ms=0;
    bool reset=false,execution_started=false,withdrawing=false,completed=false,capped=false;
};
// A value-only assessment producer for OutdoorExecutionFacts. No placement,
// route, ray, native acceptance, hidden census, or withdrawal leg is fabricated.
// Unknown safety leaves the ordinary per-man/defend pipeline free to operate.
class CombatOutdoorAssessment {
public:
    CombatOutdoorAssessment(Config,OutdoorAssessmentPolicy);
    OutdoorAssessmentResult update(const WorldSnapshot&,const SideSnapshot&,const KnowledgeFrame&,
        const OutdoorAssessmentRequest&,const std::function<bool()>& cancelled={});
    void reset();
    const OutdoorAssessmentResult& state()const{return result_;}
private:
    struct Member {ActorKey actor;std::uint64_t owner=0;bool alive=true,wounded=false;TimeMs observed_at=0;};
    struct Loss {ActorKey actor;std::uint64_t owner=0;TimeMs at_ms=0;};
    struct Down {FoeId foe;TimeMs at_ms=0;};
    struct ObservedDown {FoeId foe;ActorKey reporter;TimeMs at_ms=0;};
    struct TakenHit {ActorKey actor;TimeMs at_ms=0;};
    struct SeverShock {ActorKey observer,subject;std::uint64_t owner=0,revision=0;std::uint32_t component=0;TimeMs at_ms=0;};
    struct Relation {ActorKey observer,target;TimeMs at_ms=0;};
    Config config_;OutdoorAssessmentPolicy policy_;OutdoorAssessmentResult result_;Mission mission_;
    std::vector<Member> members_;std::vector<Loss> losses_;std::vector<TakenHit> hits_;std::vector<SeverShock> sever_shocks_;std::vector<Down> downs_;std::vector<ObservedDown> observed_downs_;std::vector<Relation> relations_;
    OutdoorAssessment pending_=OutdoorAssessment::unknown;TimeMs pending_at_=0,withdraw_at_=0;
    bool initialized_=false,has_withdrawn_=false,executing_=false;
    TimeMs exchange_hold_at_=0;
    std::uint64_t serial_=0,active_serial_=0;std::uint32_t active_node_=invalid_node;Vec3 active_point_;
};
}}}
