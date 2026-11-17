#pragma once
#include "RoomDoctrine.h"

namespace painter { namespace combat { namespace core {
enum class DefenseMode : std::uint8_t { hold, defend, assessed_withdraw };
enum class DefensePhase : std::uint8_t { inactive, holding, defending, assessing, withdrawing, regroup };
enum class DefenseReason : std::uint8_t { none, invalid_input, geometry, placing, hold, defend, continue_task,
    assessment, no_cleared_trail, unsafe_fallback, casualty_at_risk, cooldown, committed_entry,
    peeling, quiet, capped, resume, native_veto, cancelled };
// Caller records ACTUAL own-side locations, never desired order destinations.
// Trail points are replayed newest to oldest; a distant rally is not a path.
struct RoomPositionCrumb {
    ActorKey actor;TimeMs at_ms=0;std::uint64_t level_epoch=0,geometry_revision=0;
    std::uint32_t node=invalid_node,building=invalid_room,room=invalid_room;
};
struct RoomDefenseRequest {
    DefenseMode mode=DefenseMode::hold;std::uint32_t revision=0,building=invalid_room,room=invalid_room;
    std::vector<SearchedRoom> cleared;
    std::vector<RoomPositionCrumb> crumbs;
    std::vector<Order> base_orders;
    // Scheduling budget, supplied by the caller, not an alternative Config.
    // One quantum = one cover query OR one placement candidate; a withdrawal quantum
    // probes at most three short legs (direct plus two grid-axis alternatives).
    std::size_t max_work=0;
    const RoomExposureCostView* exposure=nullptr; // Borrowed for this update only.
    // Placement scope only. SideSnapshot remains the complete identity and
    // exposure authority; a building may have several independent room posts.
    bool restrict_participants=false;
    std::vector<ActorKey> participants;
};
struct DefenseCost {
    float travel=0,lit=0,door=0,unknown=0,reserve=0,reload=0,total=0;
    Evidence path=Evidence::unknown,lane=Evidence::unknown,protection=Evidence::unknown;
};
// A local witness is not a native path/restriction certificate. Publication must
// still validate the actor; a future route-node seam can supply stronger proof.
enum class DefenseRoute : std::uint8_t { none, unknown, blocked, uncleared, detour, local_witness };
struct DefenseAssignment {
    ActorKey actor;std::uint32_t portal=0,node=invalid_node;std::int8_t side=0;
    std::uint64_t intent_revision=0;DefenseCost cost;
    bool pending=false,released=false,backing_out=false,historical_trail=false,reserve=false;
    DefenseRoute route=DefenseRoute::none;
    std::uint64_t route_snapshot_seq=0;TimeMs route_at_ms=0,crumb_at_ms=0;std::uint32_t route_from_node=invalid_node;
    bool fallback_look_valid=false;decltype(Order{}.look) fallback_look{};
    RoomExposureCostReceipt exposure;
    std::uint32_t room=invalid_room;
    std::uint64_t element=0; // Local defense element, never a native side identity.
};
struct WithdrawalAssessment {
    std::size_t roster=0,observed=0,able=0,wounded=0,dead=0,unknown=0,known=0,unseen=0,our_losses=0,their_losses=0,hits=0;
    float mean_health=0;bool broken=false,outmatched=false,losing=false,pressure=false,no_firing_solution=true;
    bool requested=false,allowed=false,fallback_safe=false;DefenseReason reason=DefenseReason::none;
};
struct DefenseTransition {DefensePhase from=DefensePhase::inactive,to=DefensePhase::inactive;DefenseReason reason=DefenseReason::none;TimeMs at_ms=0;};
struct RoomDefenseResult {
    bool accepted=false,cancelled=false,partial=false,resume=false;
    DefensePhase phase=DefensePhase::inactive;DefenseReason reason=DefenseReason::none;
    std::uint64_t level_epoch=0,snapshot_seq=0,geometry_revision=0;TimeMs at_ms=0;
    WithdrawalAssessment assessment;std::vector<Order> orders;std::vector<DefenseAssignment> assignments;
    std::vector<DefenseTransition> transitions;std::size_t work_used=0;
    RoomExposureCostStats exposure;
};
// Value-only worker state. No mission inference, hidden census, health writes,
// current-restriction claim, or native actor access. Cancelled slices commit none.
class RoomDefense {
public:
    explicit RoomDefense(Config={});
    RoomDefenseResult update(const WorldSnapshot&,const SideSnapshot&,const RoomDefenseRequest&,
        const KnowledgeFrame*,const EngineServices&,const std::function<bool()>& cancelled={});
    void reset();
private:
    struct Candidate {std::uint32_t node=invalid_node;Vec3 position;Evidence protection=Evidence::unknown;};
    struct Man {
        ActorKey actor;std::uint64_t owner=0,restrictions=0,intent=0;
        RoomExposureCostReceipt exposure;float priced_lit=0,priced_unknown=0;
        std::uint32_t portal=0,node=invalid_node;std::int8_t side=0;Vec3 goal;DefenseCost cost;
        std::vector<Candidate> candidates;std::vector<std::uint32_t> rejected;std::size_t next=0,authored_next=0,native_next=0;
        bool initialized=false,complete=false,released=false,reserve=false;TimeMs release_ms=0,replan_hit=0;
        std::uint64_t element=0;std::size_t element_size=0;
        DefenseRoute route=DefenseRoute::none;std::uint32_t route_from=invalid_node;std::size_t trail_candidate=0;
        std::uint64_t witness_snapshot=0;TimeMs witness_at=0,witness_crumb=0;std::uint32_t witness_from=invalid_node;
        std::uint32_t backout=invalid_node;Vec3 backout_point;TimeMs trail_before=0;bool backout_attempted=false;
    };
    struct Own {ActorKey actor;bool alive=true,wounded=false;};
    struct Loss {ActorKey actor;TimeMs at_ms=0;};
    struct HitFact {ActorKey actor;TimeMs at_ms=0;};
    struct EnemyLoss {FoeId foe;TimeMs at_ms=0;};
    RoomDefenseResult update_impl(const WorldSnapshot&,const SideSnapshot&,const RoomDefenseRequest&,
        const KnowledgeFrame*,const EngineServices&,const std::function<bool()>&);
    Config config_;std::vector<Man> men_;std::vector<Own> own_;std::vector<Loss> losses_;std::vector<HitFact> hits_;std::vector<EnemyLoss> enemy_losses_;
    std::uint64_t epoch_=0,map_=0,geometry_=0,sequence_=0,intent_=0;std::uint32_t side_=0,revision_=0,building_=invalid_room,room_=invalid_room;
    DefenseMode mode_=DefenseMode::hold;DefensePhase phase_=DefensePhase::inactive;
    TimeMs now_=0,phase_at_=0,assessment_at_=0,last_withdraw_=0,last_evidence_=0,resumed_at_=0;
    DefenseReason assessment_reason_=DefenseReason::none;std::size_t cursor_=0;
};
}}}
