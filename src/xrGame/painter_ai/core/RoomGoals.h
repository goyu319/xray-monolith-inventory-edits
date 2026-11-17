#pragma once
#include "../CombatContract.h"
#include <cstddef>
#include <memory>
#include <vector>

namespace painter { namespace combat { namespace core {

// Geometry and projection only. No phase selection, NPC/hostile queries, fire
// permission, accessibility claim, or published Order is produced here.
struct RoomGoalStamp {
    std::uint64_t level_epoch=0, map_hash=0, geometry_revision=0;
    bool operator==(const RoomGoalStamp& b) const {
        return level_epoch==b.level_epoch && map_hash==b.map_hash && geometry_revision==b.geometry_revision;
    }
    bool operator!=(const RoomGoalStamp& b) const { return !(*this==b); }
};
struct RoomDoorFrame {
    Vec3 centre, tangent, normal; // tangent/normal are the Lua flat X/Z basis
    float half_width=0, thickness=0;
};
Vec3 room_frame_world(const RoomDoorFrame&, float u, float v, float y);
Vec3 room_frame_local(const RoomDoorFrame&, const Vec3&); // x=u,z=v; y=0

struct AuthoredRoomGoal {
    OrderKind kind=OrderKind::hold;
    std::uint16_t role=roles::none;
    std::uint32_t room=invalid_room;
    Vec3 position, look;
    bool has_look=false;
    bool door_bound=false;
    bool geometry_enabled=false;
    bool has_portal=false;
    RoomDoorFrame portal;
    // Optional world(frame,u,v,room.floor_y) input, matching ground_point.
    bool from_portal_uv=false;
    float u=0, v=0, floor_y=0;
    // Contract project bounds are explicit. Caller chooses horizontal reach;
    // ground_point additionally requires strict |snapped.y-floor_y| < .5.
    float max_horizontal_m=.8f, max_vertical_m=.5f;
};
struct RoomGoalValue {
    OrderKind kind=OrderKind::hold;
    std::uint16_t role=roles::none;
    std::uint32_t room=invalid_room, node=invalid_node;
    Vec3 authored_position, position, look;
    bool has_look=false, door_bound=false, face_movement=false, has_portal=false;
    RoomDoorFrame portal;
    float radius_m=.65f, speed_scale=1.f;
    Pace pace=Pace::walk;
};
enum class RoomGoalStatus : std::uint8_t { Pending, Ready, NoNode, InvalidInput, ServiceFault };
struct RoomGoalResult {
    RoomGoalStatus status=RoomGoalStatus::Pending;
    RoomGoalStamp stamp;
    RoomGoalValue value;
    bool cache_hit=false;
    // Ready says only a static node was projected. The engine still checks
    // current restrictions, actors, doors, ownership and route traversal.
};
enum class RoomGoalJobStatus : std::uint8_t { Pending, Complete, Cancelled, Invalidated };

class RoomGoalCache {
public:
    explicit RoomGoalCache(std::size_t max_entries=4096);
    ~RoomGoalCache();
    RoomGoalCache(RoomGoalCache&&) noexcept;
    RoomGoalCache& operator=(RoomGoalCache&&) noexcept;
    RoomGoalCache(const RoomGoalCache&)=delete;
    RoomGoalCache& operator=(const RoomGoalCache&)=delete;
    void clear();
    std::size_t size() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    friend class RoomGoalJob;
};

// One caller owns cache/job writes. Never stores EngineServices or callbacks.
// Request copies/allocation occur at construction; step performs at most one
// projection (plus optional node centre validation) per work unit. A native
// callback itself is not preemptible and no wall-time guarantee is claimed.
class RoomGoalJob {
public:
    RoomGoalJob(RoomGoalStamp, std::vector<AuthoredRoomGoal>, RoomConfig=RoomConfig{});
    std::size_t step(std::size_t max_work, const RoomGoalStamp& current,
        RoomGoalCache&, NodeProjectFn, NodePositionFn=nullptr,
        bool (*deadline_reached)(void*)=nullptr, void* clock_context=nullptr);
    void cancel();
    RoomGoalJobStatus status() const { return status_; }
    const std::vector<RoomGoalResult>& results() const { return results_; }
    std::size_t completed() const { return next_; }
private:
    RoomConfig config_;
    RoomGoalStamp stamp_;
    std::vector<AuthoredRoomGoal> requests_;
    std::vector<RoomGoalResult> results_;
    std::size_t next_=0;
    RoomGoalJobStatus status_=RoomGoalJobStatus::Pending;
};

}}}
