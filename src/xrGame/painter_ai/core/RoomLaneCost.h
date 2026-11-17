#pragma once
#include "../CombatContract.h"

namespace painter { namespace combat { namespace core {

struct RoomLaneRequest {
    // Stable for one entrance attempt; refreshing an order is not a new attempt.
    std::uint64_t attempt=0;
    std::uint32_t building=invalid_room,target_room=invalid_room;
    std::vector<std::uint32_t> portals;
};
enum class RoomLaneStatus : std::uint8_t { ready,invalid_input,older_snapshot,invalid_geometry,cancelled };
struct RoomLaneBody {
    ActorKey actor;
    std::uint64_t ownership_generation=0,restrictions_revision=0;
    TimeMs observed_at_ms=0;
    Vec3 position;
    bool dead=false,wounded=false;
};
struct RoomLaneReceipt {
    std::uint32_t portal=0;
    float cost=0;
    bool started=false,capped=false;
    TimeMs started_ms=0,elapsed_ms=0,remaining_ms=0;
    std::vector<RoomLaneBody> bodies; // only CURRENT measured occupants; never historical positions
};
struct RoomLaneResult {
    bool accepted=false,cancelled=false;
    RoomLaneStatus status=RoomLaneStatus::invalid_input;
    std::uint64_t level_epoch=0,snapshot_seq=0,map_hash=0,geometry_revision=0,attempt=0;
    TimeMs observed_at_ms=0;
    std::uint32_t side=0,mission_revision=0;
    std::vector<RoomLaneReceipt> portals;
    // Missing/old/invalid member capture is unknown, not a proof of clear space.
    std::size_t unknown_members=0,validated_members=0,body_portal_tests=0;
};

// Positive entrance cost only. No Order, care veto, path claim, dragging or
// enemy query. Caller applies the receipt before NEW releases, never stopping
// an already committed entrant. One portal clock per attempt, independent of
// body identity/lease changes. A clear current lane immediately costs zero.
// Linear snapshot indexing + O(current casualties * selected portals), with
// explicit work counters and cancellation; no fixed NPC count or native calls.
class RoomLaneCost {
public:
    explicit RoomLaneCost(Config={});
    RoomLaneResult update(const WorldSnapshot&,const SideSnapshot&,const RoomLaneRequest&,
        const BuildingCache&,const std::function<bool()>& cancelled={});
    void reset();
private:
    struct Lane {
        std::uint32_t portal=0;Vec3 centre,normal,tangent;
        float half_width=0,depth=0,floor=0;
        bool started=false;TimeMs started_ms=0;
    };
    RoomLaneResult update_impl(const WorldSnapshot&,const SideSnapshot&,const RoomLaneRequest&,
        const BuildingCache&,const std::function<bool()>&);
    Config config_;
    bool initialized_=false;
    std::uint64_t epoch_=0,map_=0,geometry_=0,sequence_=0,attempt_=0,group_=0;
    TimeMs now_=0;
    std::uint32_t side_=0,faction_=0,mission_=0,building_=invalid_room,room_=invalid_room;
    MissionKind mission_kind_=MissionKind::none;
    std::vector<Lane> lanes_;
};
}}}
