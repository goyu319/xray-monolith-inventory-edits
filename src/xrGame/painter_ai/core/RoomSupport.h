#pragma once
#include "RoomDoctrine.h"
#include "CombatKnowledge.h"

namespace painter { namespace combat { namespace core {
enum class SupportPhase : std::uint8_t { none, answering, seeking, reaching, finishing, paused, done, skipped };
enum class SupportReason : std::uint8_t { none, invalid_input, cancelled, no_free_member, quiet,
    hit_answer, wounded_known, pending_geometry, no_place, unknown_line, target_lost,
    member_lost, tries, cap, observed_down };
struct RoomSupportCosts {
    std::size_t candidate_work=0,project_queries=0,path_queries=0,ray_queries=0,unknown_rays=0;
    float path_m=0,exposure_m=0,unknown_m=0;
};
struct RoomSupportResult {
    bool accepted=false,cancelled=false,pending=false;
    SupportPhase phase=SupportPhase::none;SupportReason reason=SupportReason::none;
    ActorKey actor,target;std::uint64_t intent_revision=0;
    std::uint64_t level_epoch=0,snapshot_seq=0,knowledge_seq=0;
    std::uint8_t tries=0;RoomSupportCosts costs;
    std::vector<RoomDoctrineSlot> slots;
};

// One optional responder in one side. Value-only, transactional on cancellation.
// Raw base assignment generations stay unchanged; the publisher stamps the
// selected actor/target/intent_revision. No native objects or enemy world lookup.
class RoomSupport {
public:
    explicit RoomSupport(Config={});
    RoomSupportResult update(const WorldSnapshot&,const SideSnapshot&,const RoomDoctrineResult&,
        const KnowledgeFrame*,const EngineServices&,std::size_t work_quantum,
        const std::function<bool()>& cancelled={},const std::vector<std::uint32_t>* defended_rooms=nullptr);
    void reset();
private:
    struct Finish {
        ActorKey actor,target;std::uint64_t owner=0,restrictions=0,assignment=0,intent=0;
        Vec3 target_position,goal,best;std::uint32_t node=invalid_node,best_node=invalid_node;
        float best_cost=0;RoomSupportCosts best_costs;
        TimeMs started=0,arrived=0;bool has_arrived=false;
        std::uint8_t tries=0;std::size_t cursor=0;
        std::vector<std::uint32_t> tried,scanned;
    } finish_;
    struct Answer {ActorKey actor,victim,target;std::uint64_t owner=0,assignment=0,intent=0;
        Vec3 origin;TimeMs at=0;bool unseen=false;TimeMs until=0;} answer_;
    struct Skip {ActorKey target;TimeMs until=0;};std::vector<Skip> skipped_;
    RoomSupportResult update_impl(const WorldSnapshot&,const SideSnapshot&,const RoomDoctrineResult&,
        const KnowledgeFrame*,const EngineServices&,std::size_t,const std::function<bool()>&,
        const std::vector<std::uint32_t>*);
    Config config_;
    std::uint64_t epoch_=0,map_=0,geometry_=0,sequence_=0,intent_=0;
    std::uint32_t side_=0,mission_=0;TimeMs now_=0,last_fight_=0;bool fight_seen_=false;
};
}}}
