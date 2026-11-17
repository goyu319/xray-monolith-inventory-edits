#pragma once
#include "../CombatContract.h"
namespace painter { namespace combat { namespace core {
enum class LeaderPhase : std::uint8_t { inactive,form,pie,entry };
enum class LeaderCall : std::uint8_t { none,up,pie,entry,go };
// A measured planner milestone, not an issued goal. phase_ready means the
// caller already proved this phase complete from physical/execution evidence.
struct RoomLeaderFact {
    bool valid=false;ActorKey observer;std::uint64_t ownership_generation=0;
    std::uint64_t level_epoch=0,snapshot_seq=0,geometry_revision=0,attempt=0;
    TimeMs at_ms=0;
};
struct RoomLeaderRequest {
    std::uint64_t attempt=0;std::uint32_t room=invalid_room;
    LeaderPhase phase=LeaderPhase::inactive;ActorKey point;
    std::vector<ActorKey> members; // ordered plan members, a subset of supplied side
    std::vector<Order> accepted_orders; // actual publication ledger, not proposals
    RoomLeaderFact phase_ready,progress,threat_hold;
};
struct RoomLeaderResult {
    bool accepted=false,cancelled=false,released=false,stacked=false,line_set=false,ready_now=false;
    bool seeing=false,fight=false,stuck_fight=false,progressed=false;
    LeaderCall suggestion=LeaderCall::none;ActorKey leader,point;
    std::uint64_t level_epoch=0,snapshot_seq=0,attempt=0;
    TimeMs idle_ms=0,no_shot_ms=0,up_remaining_ms=0,go_remaining_ms=0,no_shot_remaining_ms=0;
    float up_cost_fraction=0,go_cost_fraction=0,no_shot_cost_fraction=0;
    std::size_t able=0,execution_confirmed=0,unknown_members=0;
};
// Advisory receipts only. No Orders, speed changes, native calls, clearance or
// safety overrides. The ordinary planner may proceed as soon as ready; these
// timers bound an escalation/cost and never impose an additional minimum dwell.
// A confirmed committed release latches off all further calls for the attempt.
class RoomLeader {
public:
    explicit RoomLeader(Config={});
    RoomLeaderResult update(const WorldSnapshot&,const SideSnapshot&,const RoomLeaderRequest&,
        const std::function<bool()>& cancelled={});
    void reset();
private:
    struct Track {ActorKey actor;std::uint64_t owner=0,restrictions=0,assignment=0;
        std::uint32_t node=invalid_node;Vec3 goal;float best_distance=0;};
    RoomLeaderResult update_impl(const WorldSnapshot&,const SideSnapshot&,const RoomLeaderRequest&,
        const std::function<bool()>&);
    Config config_;std::vector<Track> tracks_;
    bool initialized_=false,released_=false,up_started_=false,go_started_=false,seeing_=false,shot_=false;
    std::uint64_t epoch_=0,map_=0,geometry_=0,sequence_=0,attempt_=0,group_=0;
    std::uint32_t side_=0,mission_=0,faction_=0,room_=invalid_room;
    TimeMs now_=0,last_progress_=0,last_shot_=0,phase_since_=0,seen_since_=0,up_since_=0,go_since_=0,last_fact_=0;
    LeaderPhase phase_=LeaderPhase::inactive;
};
}}}
