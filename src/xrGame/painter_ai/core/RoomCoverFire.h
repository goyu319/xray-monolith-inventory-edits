#pragma once
#include "RoomDoctrine.h"

namespace painter { namespace combat { namespace core {
enum class CoverFirePhase : std::uint8_t { none, seeking, looking, returning, crossing_overlap };
enum class CoverFireReason : std::uint8_t { none, invalid_input, cancelled, no_crossing, no_hot_line,
    no_responder, pending_line, no_line, own_knowledge, reported_direction, expired };
struct RoomCoverFireCosts {
    std::size_t candidates=0,ray_queries=0,cached_rays=0,unknown_rays=0;
    float jamb_distance_m=0;
};
struct RoomCoverFireResult {
    bool accepted=false,cancelled=false,pending=false;
    CoverFirePhase phase=CoverFirePhase::none;CoverFireReason reason=CoverFireReason::none;
    std::uint64_t level_epoch=0,snapshot_seq=0,knowledge_seq=0;
    ActorKey actor,crossing,observer;FoeId target;std::uint32_t portal=0;
    Vec3 aim;TimeMs evidence_at_ms=0,until_ms=0;
    RoomCoverFireCosts costs;std::vector<RoomDoctrineSlot> slots;
};
struct RoomCoverFireRequest {
    std::vector<Order> applied_orders; // accepted native ledger, never proposals
    std::size_t query_quantum=0;
};

// A look/fire-only decorator. Movement and assignment identity are unchanged.
// Caller supplies its query scheduling budget, never an actor admission cap.
// Static line receipts allow nearest-first work to continue across small slices;
// unknown results authorize nothing. State is disposable until update succeeds.
class RoomCoverFire {
public:
    explicit RoomCoverFire(Config={});
    RoomCoverFireResult update(const WorldSnapshot&,const SideSnapshot&,const RoomDoctrineResult&,
        const KnowledgeFrame*,const EngineServices&,const RoomCoverFireRequest&,
        const std::function<bool()>& cancelled={});
    void reset();
private:
    struct Line {ActorKey actor;std::uint64_t owner=0,restrictions=0;Vec3 from,to;Evidence clear=Evidence::unknown;};
    RoomCoverFireResult update_impl(const WorldSnapshot&,const SideSnapshot&,const RoomDoctrineResult&,
        const KnowledgeFrame*,const EngineServices&,const RoomCoverFireRequest&,const std::function<bool()>&);
    Config config_;std::vector<Line> lines_;
    std::uint64_t epoch_=0,map_=0,geometry_=0,sequence_=0;
    std::uint32_t side_=0,mission_=0;TimeMs now_=0;
};
}}}
