#pragma once
#include "CombatCore.h"
#include "CombatSideLifecycle.h"
#include <unordered_map>

namespace painter { namespace combat { namespace core {
struct RuntimeStats {
    std::uint64_t admitted=0,decisions=0,releases=0,stale_inputs=0,missed_first_deadline=0;
    // Nonexclusive causes of emitted first-response episodes, not refreshes.
    std::uint64_t episode_admission=0,episode_contact=0,episode_hit=0;
    TimeMs max_first_order_ms=0;
    std::size_t waiting_first_order=0;
};
struct BootstrapCoverCache {
    ActorKey actor;
    std::uint64_t level_epoch=0,geometry_revision=0,restrictions_revision=0;
    std::uint32_t origin_node=invalid_node;
    TimeMs at_ms=0;
    std::vector<CoverSpot> candidates;
};
// Optional worker evidence, installed only with its native-accepted order.
// It is never inferred from OrderKind::cover or from a goal being proposed.
struct AcceptedCoverProof {
    CoverSpot spot;
    Vec3 threat_position;
    std::uint64_t snapshot_seq=0;
    TimeMs observed_at_ms=0;
    // Original own hit sensor backing an inferred-shooter escape. It can
    // preserve accepted movement during reload; it never authorizes firing.
    bool has_attributed_hit=false;
    Hit attributed_hit;
};

// Serialized core state owned by the dispatcher. A worker may construct an
// BootstrapCoverCache on values; only the dispatcher installs it. No engine call.
class Runtime {
public:
    explicit Runtime(Config config={});
    bool ingest(WorldSnapshot snapshot);
    // Main-thread current capture guard for a queued absence-based handback.
    // It never creates perception or reclaims an actually accepted release.
    void observe_species(const PerceivedSpeciesEvidence&);
    bool install_cover(BootstrapCoverCache cache);
    // The dispatcher alone assigns generations across bootstrap and squad work.
    std::uint64_t allocate_order_sequence();
    // Call only after native publication accepted this actor's order. A fresh
    // plan suppresses ordinary bootstrap refreshes, never urgent yield checks.
    // Authored room look is separate from the exact published order's current
    // sight. It is accepted only for this exact assignment and never inherited.
    bool accepted_plan(const Order& order,const Order* authored_room=nullptr,
        const AcceptedCoverProof* cover=nullptr);
    // Install only after the matching accepted plan, from the worker's exact
    // captured visual grenade/retained episode; never inferred from urgent flags.
    bool accepted_grenade_episode(const Order&,const ActorSnapshot& source,const Danger& perceived,TimeMs until);
    // Explicit side/mission authority change, NOT a handback. The dispatcher
    // first cancels superseded work. Keep admission, personal evidence and the
    // current order accounting while scheduling an independent replacement.
    // Returns false for a missing/reused/stale or already releasing lease.
    bool invalidate_plan(ActorKey actor,std::uint64_t ownership_generation);
    // Admission without an observed native execution reached its configured deadline.
    // A failed lease cannot publish more plans; only a new lease can retry.
    bool fail_first_order(ActorKey actor,std::uint64_t ownership_generation);
    // Opt-in retention/release only. source is the exact Registry-enriched
    // capture consumed by lifecycle; raw Runtime capture remains unchanged.
    // The serialized integrator cancels superseded work and invalidates these
    // intents on side/mission/roster changes before attempting publication.
    bool install_lifecycle(const WorldSnapshot& source,const SideLifecycleResult& result);
    void invalidate_lifecycle(ActorKey actor,std::uint64_t ownership_generation);
    bool accepts_orders(ActorKey actor,std::uint64_t ownership_generation) const;
    // Release acknowledgments use the source ownership lease. A confirmed
    // handback cannot reacquire that same lease from repeated stale admission.
    void accepted_release(ActorKey actor,std::uint64_t ownership_generation);
    void rejected_release(ActorKey actor);
    void rejected_proposal(ActorKey actor,std::uint64_t order_seq);
    bool defer_refresh(ActorKey actor,std::uint64_t expected_order_seq,TimeMs until);
    std::size_t step(TimeMs now,std::size_t max_steps,std::vector<Decision>& output,
        bool (*deadline_reached)(void*)=nullptr,void* context=nullptr);
    // A new hit or a selected-target flip under an accepted plan: the plan keeps its movement (look and fire are
    // refreshed at once) and asks for a squad re-plan in the background. Reading clears the request.
    bool take_replan_request() { const bool due=replan_requested_;replan_requested_=false;return due; }
    void reset(std::uint64_t epoch=0);
    const RuntimeStats& stats() const { return stats_; }
    const WorldSnapshot& snapshot() const { return snapshot_; }
private:
    struct Record {
        ActorKey key,plan_target,episode_target;
        TimeMs episode_hit_at=0,pending_episode_hit_at=0;bool episode_started=false,first_failed=false,episode_contact_pending=false;
        struct EpisodeContact {ActorKey key;TimeMs at=0;};
        // Personal contacts already present in this combat episode. A sight
        // blink may select another known foe without starting combat anew.
        std::vector<EpisodeContact> episode_contacts;
        std::uint64_t owner_generation=0;
        TimeMs admitted_at=0,next_refresh=0,deferred_until=0,last_evidence_at=0,plan_hit_at=0;
        bool first_pending=true,has_evidence=false,released=false,release_pending=false,has_plan=false;
        Order order,plan;
        AcceptedCoverProof cover;
        bool has_cover=false;
        Danger grenade;TimeMs grenade_until=0;std::uint64_t grenade_plan_seq=0;bool has_grenade=false;
        // An accepted reload request is pending until this exact weapon is
        // observed working, then idle, under an applied matching command.
        // Cheap refreshes retain plan.expires_at_ms; they do not rearm it.
        std::uint16_t reload_weapon=invalid_id;
        bool reload_requested=false,reload_seen=false,reload_complete=false;
        bool lifecycle_installed=false,lifecycle_forced_release=false,lifecycle_authored=false;
        bool species_release=false;
        bool has_group_evidence=false;
        std::uint64_t evidence_native_group=0;std::uint32_t evidence_faction=0;
        TimeMs group_evidence_at=0;
        std::uint64_t lifecycle_invalidated_through=0;
        LifecycleIdentity lifecycle_identity;
        std::uint64_t lifecycle_native_squad=0;
        TimeMs lifecycle_deadline=0;
    };
    Config config_;
    WorldSnapshot snapshot_;
    PerceivedSpeciesEvidence current_species_;
    std::unordered_map<std::uint32_t,SideSnapshot> species_release_sides_;
    std::unordered_map<std::uint32_t,Record> records_;
    std::unordered_map<std::uint32_t,BootstrapCoverCache> covers_;
    RuntimeStats stats_;
    std::size_t first_cursor_=0,refresh_cursor_=0;
    std::uint64_t next_order_=1;
    bool replan_requested_=false;
    std::uint64_t lifecycle_source_sequence_=0;
    // Exact value signature (not a collision-prone hash) for idempotent replay.
    std::vector<std::uint64_t> lifecycle_signature_;
    // Only used after explicit lifecycle installation. These are stripped
    // authority rows, not a copy of side enemy knowledge or malformed input.
    std::unordered_map<std::uint32_t,SideSnapshot> lifecycle_last_valid_sides_;
    void decide(const ActorSnapshot& actor,Record& record,TimeMs now,std::vector<Decision>& output);
    bool handback_due(const ActorSnapshot& actor,const Record& record,TimeMs now) const;
    bool observe_episode_contacts(const ActorSnapshot&,Record&,TimeMs,const Contact* selected);
    TimeMs retention_deadline(const ActorSnapshot& actor,const Record& record,TimeMs now) const;
    // The plan's movement is still valid (its goal, lease, revisions and the man's own state)
    bool plan_compatible(const ActorSnapshot& actor,const Record& record,TimeMs now) const;
    bool renew_ordinary_plan(const ActorSnapshot&,Record&,TimeMs);
    bool grenade_movement(const ActorSnapshot&,const Record&,TimeMs) const;
    bool reload_cover_movement(const ActorSnapshot& actor,const Record& record,TimeMs now) const;
    // New facts under the plan that change aim, not movement: a hit newer than the plan's, a different selected target
    bool plan_disturbed(const ActorSnapshot& actor,const Record& record,TimeMs now) const;
};
}}}
