#pragma once
#include "RoomDoctrine.h"
#include "FriendlyCasualty.h"

namespace painter { namespace combat { namespace core {
enum class CasualtyPhase : std::uint8_t { none, reaching, guard, working, done, failed };
enum class CasualtyReason : std::uint8_t { none, invalid_input, no_helper, no_projection, travelling,
    visible_threat, hit_quiet, native_work, native_done, native_failed, cap, target_lost,
    helper_lost, eligibility, cancelled };
struct RoomCasualtyCosts {
    float path_m=0,exposure_m=0,unknown_m=0,total_m=0;
    std::size_t projected_candidates=0,path_queries=0,ray_queries=0,unknown_rays=0;
    Evidence reach_line=Evidence::unknown;
};
struct RoomCasualtyResult {
    bool accepted=false,cancelled=false;
    CasualtyPhase phase=CasualtyPhase::none;CasualtyReason reason=CasualtyReason::none;
    ActorKey helper,casualty,security;std::uint64_t generation=0,intent_revision=0;
    bool stay_with_casualty=false;
    TimeMs started_ms=0,work_elapsed_ms=0;
    RoomCasualtyCosts placement;
    std::vector<RoomDoctrineSlot> slots;
    std::vector<Event> events;
};

// One side's optional care team: one helper and one independent security man.
// No health,
// native claim writes, enemy census or assumed accessibility. The executor owns
// the stock claim and treatment transaction. Cancelled work commits no state.
class RoomCasualty {
public:
    explicit RoomCasualty(Config={});
    RoomCasualtyResult update(const WorldSnapshot&,const SideSnapshot&,const RoomDoctrineResult&,
        const EngineServices&,const std::function<bool()>& cancelled={},
        const std::vector<FriendlyCasualtyFact>& observed_targets={},
        const std::vector<SearchedRoom>& communicated_rooms={});
    void reset();
private:
    struct Attempt {
        ActorKey helper,target;std::uint64_t owner=0,restrictions=0,generation=0,intent=0;
        Vec3 target_position,goal;std::uint32_t node=invalid_node;
        TimeMs started=0,requested=0,last_hit=0;
        bool aid_requested=false;
        RoomCasualtyCosts placement;
        std::uint8_t baseline_phase=0;TimeMs baseline_at=0;
        CasualtyPhase phase=CasualtyPhase::none;
    };
    RoomCasualtyResult update_impl(const WorldSnapshot&,const SideSnapshot&,const RoomDoctrineResult&,
        const EngineServices&,const std::function<bool()>&,const std::vector<FriendlyCasualtyFact>&,
        const std::vector<SearchedRoom>&);
    Config config_;Attempt attempt_;
    ActorKey stay_target_,security_;std::uint32_t stay_room_=invalid_room;
    CasualtyReason stay_reason_=CasualtyReason::none;
    std::vector<ActorKey> finished_;
    std::uint64_t epoch_=0,map_=0,geometry_=0,sequence_=0,generation_=0,intent_revision_=0;
    std::uint32_t side_=0,mission_=0;
    TimeMs now_=0;
};
}}}
