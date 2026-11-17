#pragma once
#include "CombatOutdoorAssessment.h"
#include "CombatOutdoorBounds.h"
namespace painter { namespace combat { namespace core {
// OutdoorWithdrawalPolicy is allocated in ../OutdoorConfig.h.
// Exact current, query-produced candidate. Profile owns provenance; every
// leader-known source must also have a blocked static line at the candidate.
// No native query runs in this module and unknown is never certified safe.
struct OutdoorWithdrawalCandidate {
    OutdoorStamp stamp;OutdoorCoverProfile profile;CoverSpot spot;
    OutdoorBoundPath route;std::vector<OutdoorFallbackLine> lines;
};
struct OutdoorWithdrawalIntent {
    OutdoorStamp stamp;std::uint64_t episode=0,leg=0;
    ActorKey actor;std::uint64_t owner=0,restrictions_revision=0,previous_order_seq=0;
    std::uint32_t node=invalid_node;Vec3 position;
    TimeMs issued_at_ms=0,expires_at_ms=0;
};
// Match the actual final publication; never a proposed/local order generation.
bool outdoor_withdrawal_accepts(const OutdoorWithdrawalIntent&,const Order&);
struct OutdoorWithdrawalExecution {
    OutdoorStatus status=OutdoorStatus::invalid;OutdoorStamp stamp;
    std::vector<OutdoorWithdrawalLegReceipt> legs;
    std::vector<ActorKey> arrived;
    std::size_t work=0;
};
struct OutdoorWithdrawalRequest {
    const WorldSnapshot* world=nullptr;const SideSnapshot* side=nullptr;
    const KnowledgeFrame* knowledge=nullptr;const SquadView* elements=nullptr;
    const OutdoorAssessmentResult* assessment=nullptr;
    const OutdoorExecutionFacts* execution=nullptr;
    const OutdoorWithdrawalExecution* withdrawal_execution=nullptr;
    const std::vector<Order>* accepted_orders=nullptr;
    const std::vector<ProposedOrder>* base_orders=nullptr;
    const std::vector<OutdoorWithdrawalCandidate>* candidates=nullptr;
};
struct OutdoorWithdrawalOrder {ProposedOrder proposal;OutdoorWithdrawalIntent intent;bool continuing=false;};
struct OutdoorWithdrawalResult {
    OutdoorStatus status=OutdoorStatus::invalid;OutdoorStamp stamp;const char* reason="invalid";
    std::uint64_t episode=0;bool reset=false,swapped=false,done=false;
    std::vector<ActorKey> base,movers;
    std::vector<OutdoorWithdrawalOrder> orders;
    std::size_t work=0,examined=0,rejected=0;bool exhausted=false;
};
// Value-only proposal producer. Native acceptance/last_applied are authoritative.
// The caller supplies a SHARED remaining work allowance; consumed validation
// work is never refunded, including cancellation. No actor-count admission cap.
class CombatOutdoorWithdrawal {
public:
    CombatOutdoorWithdrawal(Config,OutdoorWithdrawalPolicy);
    void reset();
    OutdoorWithdrawalExecution capture_execution(const WorldSnapshot&,const SideSnapshot&,
        const std::vector<Order>&,const OutdoorExecutionFacts&,std::size_t& work_remaining,
        const std::function<bool()>& cancelled={})const;
    OutdoorWithdrawalResult update(const OutdoorWithdrawalRequest&,std::size_t& work_remaining,
        const std::function<bool()>& cancelled={});
private:
    Config config_;OutdoorWithdrawalPolicy policy_;OutdoorStamp stamp_;
    std::uint64_t episode_=0,next_leg_=0;std::size_t cursor_=0;
    std::vector<ActorKey> base_,movers_;
    std::vector<OutdoorWithdrawalIntent> intents_;
};
}}}
