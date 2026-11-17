#pragma once
#include "PointFireAuthority.h"
#include "CombatCore.h"
#include "CombatKnowledge.h"
#include <memory>

namespace painter { namespace combat { namespace core {

// The authority supplies an actual delivered communication event. The layer
// also verifies its original observation against its own snapshot history.
// Absence of a receipt never implies radio, voice delivery or shared vision.
struct ReportReceipt {
    ActorKey source, receiver, target;
    std::uint64_t level_epoch = 0, source_snapshot_seq = 0;
    TimeMs observed_at_ms = 0, delivered_at_ms = 0;
    Vec3 observed_position;
};
struct SuppressionFact {
    ActorKey observer;
    std::uint64_t level_epoch = 0, snapshot_seq = 0;
    TimeMs observed_at_ms = 0;
    float intensity = 0; // Authority-observed own suppression, 0..1.
};
struct CasualtyFact {
    ActorKey observer, casualty;
    std::uint64_t level_epoch = 0, snapshot_seq = 0;
    TimeMs observed_at_ms = 0;
};
struct ActorCoverCache {
    ActorKey actor;
    std::uint64_t level_epoch = 0, snapshot_seq = 0, ownership_generation = 0;
    // Revision this proposal was requested for, NOT accessibility evidence.
    // CoverSpot::restrictions_revision stays 0 when access was not checked.
    std::uint64_t restrictions_revision = 0;
    std::uint32_t origin_node = invalid_node;
    TimeMs observed_at_ms = 0;
    std::vector<CoverSpot> spots;
    // Exact knowledge query identity. Ghosts remain side-local values; they
    // are never encoded into an engine ActorKey. Old self-contact callers may
    // omit this and use CoverSpot's real threat provenance only.
    FoeId knowledge_threat;
    TimeMs knowledge_observed_at_ms = 0;
    Vec3 knowledge_threat_position;
    bool knowledge_unidentified = false;
};
struct SquadContext {
    std::vector<ActorCoverCache> cover;
    std::vector<ReportReceipt> delivered_reports;
    std::vector<SuppressionFact> suppression;
    std::vector<CasualtyFact> seen_casualties;
    std::shared_ptr<const KnowledgeFrame> knowledge;
};
struct SquadTuning {
    // (the suppression surcharge is data now: Config::suppressed_exposure_cost_m, v2.12)
    // Evidence capacity is per observer/target pair, never an admission cap.
    std::size_t observation_history_per_pair = 32;
    // Worker queries cannot resolve live navigation. Opt-in proposals preserve
    // unknown navigation/protection evidence with configured costs; bootstrap stays strict.
    bool allow_pending_navigation = false;
    // Production registry/K1 supplies the authoritative side roster. Opt-in
    // preserves those identities/slots and performs no automatic regrouping.
    // Default false retains the standalone legacy grouping contract.
    // Shared knowledge then comes only from SquadContext::knowledge; legacy
    // delivered_reports are rejected (personal captures still work without K1).
    bool use_side_roster = false;
};
struct KnowledgeTrace {
    ActorKey receiver, target, observer;
    Vec3 position;
    std::uint64_t source_snapshot_seq = 0;
    TimeMs observed_at_ms = 0;
    bool personally_visible = false;
    bool shared = false;
    std::uint64_t receiver_ownership_generation = 0;
    std::uint32_t receiver_faction = 0;
    FoeId foe;
    bool unidentified = false;
};
struct SquadView {
    std::uint64_t group_id = 0, native_squad = 0;
    std::uint32_t faction = 0;
    std::uint32_t side_id = 0; // supplied-side mode: group_id == side_id; otherwise 0
    bool ad_hoc = false;
    ActorKey leader;
    std::vector<ActorKey> members, base, movers;
    std::vector<KnowledgeTrace> known;
    std::vector<ActorKey> known_casualties;
    std::vector<CasualtyFact> casualty_observations;
};
struct ProposedOrder {
    PointFireAuthority point_fire;
    // The integrator must allocate a NEW order generation and publish only if
    // expected_previous_order_seq still matches. This is not a second owner.
    Order order;
    std::uint64_t expected_previous_order_seq = 0, group_id = 0;
    float cover_cost = 0;
    bool cover_conflict = false;
    bool has_knowledge = false;
    KnowledgeTrace knowledge;
    bool has_cover_evidence = false;
    bool navigation_pending = false;
    CoverSpot cover_evidence;
};
struct SquadResult {
    bool accepted = false;
    bool cancelled = false;
    std::uint64_t level_epoch = 0, snapshot_seq = 0;
    std::vector<SquadView> groups;
    std::vector<ProposedOrder> proposals;
    std::size_t verified_reports = 0, rejected_reports = 0;
    std::size_t actors_without_current_order = 0;
    std::size_t preserved_planned_orders = 0;
};

// The same selection is used for worker cover queries and order proposals.
// Frame sight is revalidated against the actor's CURRENT personal snapshot;
// reports and ghosts can orient or inform cover, never grant enemy fire.
bool tactical_knowledge_focus(const ActorSnapshot&,const WorldSnapshot&,
    const KnowledgeFrame&,KnowledgeTrace&);
// Indexed callers must first validate the containing frame and exact side.
// This overload retains actor/lease checks and current personal sight checks.
bool tactical_knowledge_focus(const ActorSnapshot&,const WorldSnapshot&,
    const MemberKnowledge&,KnowledgeTrace&);

// Snapshot-only coordination. Existing bootstrap orders remain untouched until
// the integrator validates/applies these proposals. No engine object, query,
// wall clock, implicit communication or full room planner is used here.
class SquadTactics {
public:
    explicit SquadTactics(Config config = {}, SquadTuning tuning = {});
    ~SquadTactics();
    SquadTactics(SquadTactics&&) noexcept;
    SquadTactics& operator=(SquadTactics&&) noexcept;
    SquadTactics(const SquadTactics&) = delete;
    SquadTactics& operator=(const SquadTactics&) = delete;
    SquadResult update(const WorldSnapshot&, const std::vector<Order>& current,
        const SquadContext& context = {}, const std::function<bool()>& cancelled = {});
    void reset();
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}}} // namespace painter::combat::core
