#pragma once
#include "CombatCore.h"
#include "CombatKnowledge.h"
#include "PointFireAuthority.h"
#include "CombatRouteProof.h"
#include <array>
#include <memory>

namespace painter { namespace combat { namespace core {
// Required caller data pending canonical allocation. No runtime defaults.
// OutdoorFirePolicy is allocated in ../OutdoorConfig.h.
struct OutdoorFireQuota {std::size_t work_left=0,queries_left=0,work_used=0,queries_used=0;};
struct OutdoorFireSource {
    ActorKey actor;std::uint64_t level_epoch=0,snapshot_seq=0,geometry_revision=0,owner=0,restrictions=0;
    TimeMs at_ms=0;std::uint32_t origin_node=invalid_node;Vec3 position;
};
struct OutdoorFireEye {OutdoorFireSource source;Vec3 measured_eye;};
// Optional measured/validated local-step proof, not an invented path. Source,
// exact projected goal, route and doorway/red safety must all be certified by
// the producer for this captured actor. Native path length is also checked.
// sample_eye is an explicit geometry sample supplied by that producer; it is
// NEVER evidence of a current eye, current visibility or permission to fire.
struct OutdoorBuddyStep {
    OutdoorFireSource source;ActorKey buddy;TimeMs hit_at_ms=0;
    FoeId target;Vec3 known_point; // exact reported source for the doorway/red/route assessment
    Vec3 requested,goal,sample_eye;std::uint32_t node=invalid_node;
    Evidence route_safe=Evidence::unknown,doorway_safe=Evidence::unknown,red_safe=Evidence::unknown;
    std::uint64_t assignment_generation=0; // caller allocates its final namespace
};
struct OutdoorFireRequest {
    std::vector<Order> base_orders;     // current proposed orders, sequence already allocated
    // Actual accepted native ledger, AFTER final sequence allocation/sanitizing.
    // Capture must confirm its final sequence. Source-stamped suppression intent
    // is matched independently of the earlier proposal's provisional sequence.
    std::vector<Order> applied_orders;
    std::vector<OutdoorFireEye> eyes;
    std::vector<OutdoorBuddyStep> steps;
    const RouteProofService* route_service=nullptr;RouteProofBudget* route_budget=nullptr;
};
enum class OutdoorFireStatus : std::uint8_t {invalid,ready,pending,cancelled,failed};
enum class OutdoorFireReason : std::uint8_t {
    unchanged,suppress,buddy_here,buddy_step,buddy_watch,cooldown,protected_role,
    missing_eye,no_line,unknown_line,point_authority_missing,teammate_line,budget,weapon,own_fight
};
struct OutdoorFireProposal {
    Order order;OutdoorFireReason reason=OutdoorFireReason::unchanged;
    FoeId target;ActorKey buddy;TimeMs hit_at_ms=0,source_at_ms=0,window_until_ms=0;
    Vec3 known_position,selected_point,ray_from,ray_to;
    Evidence line=Evidence::unknown;
    bool changed=false,movement_changed=false,point_authorized=false;
    PointFireAuthority point_fire;
};
struct OutdoorFireResult {
    OutdoorFireStatus status=OutdoorFireStatus::invalid;
    std::uint64_t epoch=0,sequence=0;std::uint32_t side=0;
    std::vector<OutdoorFireProposal> proposals;
    std::vector<SuppressedFoeFact> executed_suppression; // actual applied fire observation, never merely a proposal
    std::size_t work=0,queries=0,cache_hits=0,unknown_lines=0;
};
// One side, value-only, transactional state. All faction IDs including zero are
// treated alike. One caller quota must be shared across every side/member.
// Unknown geometry cannot authorize point fire. Native publication and final
// live teammate/aim/weapon vetoes remain mandatory.
class CombatOutdoorFire {
public:
    CombatOutdoorFire(Config,OutdoorFirePolicy);
    ~CombatOutdoorFire();
    CombatOutdoorFire(CombatOutdoorFire&&) noexcept;
    CombatOutdoorFire& operator=(CombatOutdoorFire&&) noexcept;
    CombatOutdoorFire(const CombatOutdoorFire&)=delete;
    CombatOutdoorFire& operator=(const CombatOutdoorFire&)=delete;
    std::unique_ptr<CombatOutdoorFire> clone(const std::function<bool()>& cancelled={}) const;
    OutdoorFireResult update(const WorldSnapshot&,const SideSnapshot&,const KnowledgeFrame&,
        const OutdoorFireRequest&,const EngineServices&,OutdoorFireQuota&,
        const std::function<bool()>& cancelled={});
    void reset();
private:
    struct Impl;std::unique_ptr<Impl> impl_;
};
}}}
