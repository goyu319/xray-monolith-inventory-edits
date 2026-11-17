#pragma once
#include "CombatMissionIntel.h"

namespace painter { namespace combat { namespace core {

// REQUIRED caller data. Zero initialization is deliberately invalid, not a set
// of runtime defaults. Allocation in canonical Config and engine settings is pending.
// OutdoorPolicy is allocated in ../OutdoorConfig.h.
enum class OutdoorPosture : std::uint8_t { quiet,contact,defend,attack,assault,clear,search,withdraw };
enum class OutdoorStatus : std::uint8_t { invalid,ready,stale,cancelled,failed };
enum class OutdoorElement : std::uint8_t { none,base,mover };
enum class OutdoorCover : std::uint8_t { unknown,open,low,chest,full };
enum class OutdoorAssessment : std::uint8_t { unknown,proceed,hold,withdraw };
enum class OutdoorEndpoint : std::uint8_t { unknown,feasible,failed };
enum class OutdoorHandover : std::uint8_t { none,inside,arrived,at_door,building_fight,solo,stalled,time };

struct OutdoorStamp {
    std::uint64_t level_epoch=0,map_hash=0,geometry_revision=0,snapshot_seq=0,group=0;
    TimeMs at_ms=0; std::uint32_t side=0,faction=0,mission_revision=0;
};
// These are EXISTING native-applied orders and query receipts, never proposals.
// Every row is checked against the captured actor's lease, restrictions and
// last_applied. Evidence fields belong to that exact order/goal and focus fact.
struct OutdoorActorExecution {
    OutdoorElement element=OutdoorElement::none;
    Order applied;
    OutdoorCover cover=OutdoorCover::unknown;
    Evidence cover_verified=Evidence::unknown,fire_line=Evidence::unknown;
    Evidence movement_route=Evidence::unknown,line_to_focus=Evidence::unknown;
    Evidence searched=Evidence::unknown;
    FoeId focus; TimeMs focus_at_ms=0;
    // Search proof means observed coverage of this exact source location, not
    // just arriving at it. Unknown never resolves a last-known.
};
// A completed, authenticated base-placement evaluation. Missing query work is
// not failure. The original observation clock is never renewed by consumption.
struct OutdoorSupportFailure {
    OutdoorStamp stamp;ActorKey actor;std::uint64_t owner=0,restrictions=0;
    FoeId focus;TimeMs focus_at_ms=0;Vec3 focus_position;
    bool completed=false;
};
struct OutdoorExecutionFacts {
    OutdoorStamp stamp;
    OutdoorSupportFailure support_failure;
    std::vector<OutdoorActorExecution> actors;
    std::uint64_t endpoint_serial=0;
    OutdoorEndpoint endpoint=OutdoorEndpoint::unknown;
    FoeId endpoint_focus; TimeMs endpoint_focus_at_ms=0;
    ActorKey endpoint_actor;std::uint64_t endpoint_owner=0,endpoint_restrictions=0;
    std::uint32_t endpoint_node=invalid_node;Vec3 endpoint_position;
    Evidence endpoint_route=Evidence::unknown,endpoint_cover=Evidence::unknown;
    OutdoorAssessment assessment=OutdoorAssessment::unknown;
    std::uint64_t assessment_serial=0;
    Evidence safe_fallback=Evidence::unknown,casualty_obligation_clear=Evidence::unknown;
    Vec3 fallback;
    // New assessment producer owns actual execution/cooldown. A verdict or
    // posture alone cannot spend its episode. Legacy direct callers may omit it.
    bool assessment_managed=false,assessment_executing=false;
    bool assessment_exchange_hold=false;
    TimeMs assessment_exchange_at_ms=0,assessment_exchange_until_ms=0;
    float assessment_advance_cost=0,assessment_hold_cost=0;
    bool assessment_completed=false,assessment_capped=false;
    std::uint64_t assessment_withdrawal_episode_serial=0;
    TimeMs assessment_cooldown_remaining_ms=0;
    // The objective is selected from authored geometry with a validated route.
    // A building in mission intelligence alone is not a reachable doorway.
    Evidence door_validated=Evidence::unknown;
    std::uint32_t door_building=invalid_room,door_portal=0,door_node=invalid_node;
    Vec3 door_point;
};
struct OutdoorPostureResult {
    OutdoorStatus status=OutdoorStatus::invalid;
    OutdoorStamp stamp;
    OutdoorPosture posture=OutdoorPosture::quiet,previous=OutdoorPosture::quiet;
    const char* reason="uninitialized";
    bool transitioned=false,reset=false,lease_changed=false;
    TimeMs phase_at_ms=0,transition_source_ms=0;
    ActorKey leader; std::size_t able=0,known=0,unseen=0,unresolved=0;
    bool focus_valid=false; FoeId focus; SourceFact focus_fact;
    Evidence base_set=Evidence::unknown,base_pinned=Evidence::unknown;
    bool bound_in_progress=false,allow_bound=false,allow_assault=false;
    bool support_recovery=false; // permits one proven protective approach; never support readiness
    bool base_formation_progress=false; // Measured travel of an accepted base goal, not proposed movement.
    std::uint32_t endpoint_failures=0;
    OutdoorHandover handover=OutdoorHandover::none;
    std::uint32_t handover_building=invalid_room,handover_portal=0;
    // Only a movement destination intention. No Order, new fire permission,
    // hidden target position, or side release is emitted by this module.
    bool door_bound=false; Vec3 movement_focus;
    bool exchange_hold=false;
    std::uint32_t defended_building=invalid_room;
    bool loose_advance=false; // All eligible men progress by cover; no invented set base.
    FactionStyle resolved_style;bool style_matched=false;
    // Original adverse assessment provenance, never the time of a later pass.
    TimeMs exchange_source_at_ms=0,exchange_source_until_ms=0,exchange_favourable_since_ms=0;
};

class CombatOutdoorPosture {
public:
    CombatOutdoorPosture(Config config,OutdoorPolicy policy);
    OutdoorPostureResult update(const WorldSnapshot&,const SideSnapshot&,
        const KnowledgeFrame&,const CombatMissionIntel&,const OutdoorExecutionFacts&,
        const std::function<bool()>& cancelled={});
    const OutdoorPostureResult& state() const {return state_;}
    void reset();
private:
    struct MemberState {ActorKey key;std::uint64_t owner=0;bool fired=false;TimeMs fired_at=0;FoeId fired_focus;
        std::uint32_t base_node=invalid_node;Vec3 base_goal;float base_remaining=0;TimeMs base_progress_at=0;};
    struct Searched {FoeId foe;TimeMs source_at=0;};
    struct RelationState {ActorKey observer,target;TimeMs nonhostile_at=0;};
    Config config_; OutdoorPolicy policy_; bool initialized_=false;
    OutdoorPostureResult state_; Mission mission_;
    std::vector<MemberState> members_; std::vector<Searched> searched_;
    std::vector<RelationState> relations_;
    bool has_info_=false,has_seen_=false,has_hit_=false,has_heard_=false,has_supp_=false;
    TimeMs info_at_=0,seen_at_=0,hit_at_=0,heard_at_=0,supp_at_=0;
    bool stalled_=false,defend_ok_=false,withdrew_=false;
    TimeMs stalled_at_=0,defend_ok_at_=0,withdraw_at_=0,mission_at_=0;
    std::uint64_t endpoint_serial_=0,assessment_serial_=0;
    bool door_progress_=false;float door_best_m_=0;TimeMs door_progress_at_=0;
    Vec3 door_point_;std::uint32_t door_portal_=0;
    bool cleared_activity_=false;TimeMs cleared_activity_ms_=0;
    bool withdrawal_goal_=false;Vec3 withdrawal_point_;std::uint64_t withdrawal_serial_=0;
    bool exchange_latched_=false,exchange_favourable_=false;
    TimeMs exchange_at_=0,exchange_until_=0,exchange_favourable_at_=0;
};
}}}
