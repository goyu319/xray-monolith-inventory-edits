#pragma once
#include "RoomStack.h"
#include "CombatKnowledge.h"
#include "RoomGeometry.h"
#include "RoomLaneCost.h"
#include "RoomLeader.h"
#include <vector>
#include <set>

namespace painter { namespace combat { namespace core {
enum class RoomPhase : std::uint8_t { invalid, form, pie, entry, search, assess, complete };
enum class RoomReason : std::uint8_t { none, invalid_input, no_geometry, no_projection, forming, arrived,
    pie_progress, pie_cap, engage_cost, go_cost, entry_flow, coverage, known_foe, assess_cap,
    fresh_contact, frag_requested, frag_fuse, frag_failed, cancelled, sight_clear, assess_observed, casualty_cost, leader_call };
struct RoomDoctrineRequest {
    RoomStackRequest stack;
    // Current admitted cohort is physically inside this exact authored room.
    // Start local SEARCH without claiming an entry crossing or prior clearance.
    bool inspect_current_room=false;
    // Atomic optional throw. No prepare-only command or inferred release.
    bool permit_frag=false;
    ActorKey preferred_thrower;
    // Accepted native ledger, never the previous worker's raw proposals.
    std::vector<Order> applied_orders;
    // Scheduling work budget, not a doctrine tuning constant. Zero disables
    // optional full-nav sight-only proof while ordinary entry/search continues.
    std::size_t sight_clear_nodes_per_tick=0;
    // Static room-node intake work; zero selects Config.cover_queries_per_slice.
    std::size_t geometry_nodes_per_update=0;
    const RoomExposureCostView* exposure=nullptr; // Borrowed; overrides stack.exposure.
};
struct RoomCoverage {
    Vec3 point; std::uint32_t sector=0;
    TimeMs seen_at_ms=0; ActorKey observer; std::uint64_t observer_owner=0;
    bool derived=false;
};
struct RoomDoctrineSlot {
    ActorKey actor; Order order;
    bool released=false,crossed=false,arrived=false;
    TimeMs released_at_ms=0;
    bool execution_confirmed=false;
    std::uint64_t applied_order_seq=0;
    decltype(Order{}.look) authored_look;
    bool authored_look_valid=false;
    // Actual room membership, separate from doorway-plane and issued movement.
    Evidence room_membership=Evidence::unknown;
    RoomExposureCostReceipt exposure;
};
struct RoomDoctrineCosts {
    float go=0,wait=0,engage=0,door_hot=0,back_out=0,overwatch=0,destination_exposure=0,casualty_lane=0,leader=0;
    std::size_t known_foes=0,visible_foes=0,known_wounded=0,covered=0,total=0,unprojected=0;
    std::size_t sight_nodes=0,sight_seen=0,sight_tested=0;
    std::size_t crossing_waiters=0,crossing_cap_releases=0;
    TimeMs crossing_wait_ms=0;
    float held_exposure=0; // Diagnostic refreshed price; never a movement veto.
};
enum class PieAwareness : std::uint8_t { unknown,unaware,partly,aware };
struct RoomPieChoice {
    ActorKey actor,enemy;PieAwareness awareness=PieAwareness::unknown;
    float engage=0,back_out=0,enter=0,overwatch=0;
    bool engage_here=false,enter_now=false;
};
struct RoomPhaseMember {ActorKey actor;Vec3 position;TimeMs observed_at_ms=0;};
struct RoomTransition {RoomPhase from=RoomPhase::invalid,to=RoomPhase::invalid;RoomReason reason=RoomReason::none;TimeMs at_ms=0;
    std::vector<RoomPhaseMember> members;};
struct RoomCrossingWait {
    ActorKey actor;std::uint32_t portal=0;
    TimeMs last_hit_ms=0,started_ms=0,waited_ms=0,gap_remaining_ms=0,cap_remaining_ms=0;
    bool waiting=false,cap_released=false;
};
struct RoomThreatAttention {
    ActorKey observer;FoeId foe;Vec3 point;
    TimeMs source_at_ms=0,assigned_at_ms=0,inspected_at_ms=0;
    bool repositioning=false;
};
struct RoomDoctrineResult {
    bool accepted=false,cancelled=false,ready_for_next=false;
    bool interior_start=false;
    RoomPhase phase=RoomPhase::invalid;RoomReason reason=RoomReason::none;
    std::uint64_t level_epoch=0,snapshot_seq=0,knowledge_seq=0,generation=0,map_hash=0,geometry_revision=0,mission_revision=0;
    std::uint32_t building=invalid_room,room=invalid_room;
    TimeMs captured_at_ms=0,phase_started_ms=0;
    RoomDoctrineCosts costs;
    bool geometry_pending=false,geometry_prepared=false,coverage_is_sampled=true;
    std::size_t geometry_nodes_total=0,geometry_nodes_processed=0,geometry_nodes_valid=0;
    RoomStackResult formation;
    std::uint64_t formation_snapshot_seq=0;
    TimeMs formation_at_ms=0;
    std::vector<RoomDoctrineSlot> slots;
    std::vector<RoomCoverage> coverage;
    std::vector<RoomCellFact> room_cells; // current genuine contributors, never synthetic clearance
    std::vector<RoomCellFact> clear_proof; // exact cells of the verified physical clear
    std::vector<std::uint32_t> cleared_rooms;
    std::vector<Event> events;
    std::vector<RoomTransition> transitions;
    std::vector<RoomPieChoice> pie_choices;
    // The request identity, including zero for geometry-selected entry. An
    // observed-clear room can still be assessing through this same entrance.
    std::uint32_t requested_entry_portal=0;
    std::vector<RoomCrossingWait> crossings;
    RoomLaneResult lane;
    RoomLeaderResult leader;
    // Inspection is a measured look at one sourced position, not an enemy
    // death/disproof, room clearance or permission to shoot a shared report.
    std::vector<RoomThreatAttention> threat_attention;
    RoomExposureCostStats exposure;
};

// Primary per-room progression only. Caller/controller selects the next room.
// Update is transactional on cancellation; static queries are worker-callable.
// It never examines an enemy census, claims current access, or allocates native
// order sequences. Outputs remain proposals until the native publisher accepts.
class RoomDoctrine {
public:
    explicit RoomDoctrine(Config={});
    RoomDoctrineResult update(const WorldSnapshot&,const SideSnapshot&,const RoomDoctrineRequest&,
        const KnowledgeFrame*,const EngineServices&,const std::function<bool()>& cancelled={});
    void reset();
    const RoomDoctrineResult& state() const {return result_;}
private:
    struct Man {
        ActorKey actor;std::uint64_t owner=0,restrictions=0;
        Order order;std::uint32_t portal=0;std::size_t rank=0,pie_step=0;
        bool watcher=false,released=false,crossed=false,arrived=false,goal_ready=false,entry_line_prepared=false;
        bool interior_bootstrap=false;
        bool entry_tracking=false;float entry_best_m=0;TimeMs entry_progress_ms=0;
        Evidence room_membership=Evidence::unknown;
        TimeMs released_ms=0;
        bool crossing_wait_started=false;TimeMs crossing_wait_ms=0;
        Vec3 stack_goal;std::uint32_t stack_node=invalid_node;
        ActorKey pie_enemy;TimeMs contact_ms=0,assess_arrived_ms=0;
        std::uint8_t engage_choice=0,leader_choice=0;
        std::uint32_t assess_portal=0;std::size_t assess_step=0;
        std::vector<std::uint32_t> rejected;
        TimeMs search_hazard_retry_ms=0;std::size_t search_hazard_cursor=0;
    };
    struct Throw {
        ActorKey actor;std::uint64_t owner=0;std::uint32_t serial=0,baseline_releases=0;
        std::uint16_t item=invalid_id;Vec3 target;TimeMs requested_ms=0,go_at_ms=0,armed_ms=0;
        bool attempted=false,finished=false;
    };
    RoomDoctrineResult update_impl(const WorldSnapshot&,const SideSnapshot&,const RoomDoctrineRequest&,
        const KnowledgeFrame*,const EngineServices&,const std::function<bool()>&);
    Config config_;RoomStack stack_;RoomGeometry room_geometry_;RoomLaneCost lane_cost_;RoomLeader leader_;RoomDoctrineResult result_;
    std::vector<Man> men_;Throw throw_;std::set<std::uint32_t> released_portals_;
    struct DoorHeat {std::uint32_t portal=0;TimeMs at_ms=0;Vec3 origin,point;};
    struct SightCell {std::uint32_t node=invalid_node;TimeMs seen_ms=0;ActorKey observer;std::uint64_t observer_owner=0;};
    struct Inspection {RoomThreatAttention receipt;std::uint64_t owner=0;};
    std::vector<Inspection> inspections_;
    std::vector<DoorHeat> heat_;std::vector<SightCell> sight_cells_;
    std::size_t sight_cursor_=0;bool sight_geometry_complete_=false;
    bool interior_start_=false;
    std::vector<ActorKey> assess_down_;
    std::uint64_t epoch_=0,map_=0,geometry_=0,building_data_=0,sequence_=0,generation_=0;
    std::uint32_t side_=0,mission_=0,building_=0,target_=invalid_room,entry_=0,throw_serial_=0;
    TimeMs phase_at_=0,go_at_=0,last_shot_=0,last_contact_=0,pair_ready_at_=0;
    RoomPhase phase_=RoomPhase::invalid;
};
}}}
