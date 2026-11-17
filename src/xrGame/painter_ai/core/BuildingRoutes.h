#pragma once
#include "../CombatContract.h"

// Portable building topology only. This module creates no gameplay orders and
// never reads an NPC, an engine object, Lua, or hidden enemy state.
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace painter { namespace combat { namespace core {

constexpr std::uint32_t no_node = 0xffffffffu;
struct GraphStamp {
    std::uint64_t level_epoch = 0;
    std::uint64_t map_hash = 0;
    std::uint64_t geometry_revision = 0;
    std::uint32_t building_id = 0;
    bool operator==(const GraphStamp&) const;
    bool operator!=(const GraphStamp& b) const { return !(*this==b); }
    bool operator<(const GraphStamp&) const;
};
struct QueryStamp {
    GraphStamp graph;
    // Caller increments when its searched rooms or any side evidence changes.
    // Each query owns its evidence snapshot; none is cached into static data.
    std::uint64_t knowledge_revision = 0;
    bool operator==(const QueryStamp&) const;
    bool operator!=(const QueryStamp& b) const { return !(*this==b); }
};
struct BuildingNode { std::uint32_t id=no_node; std::uint32_t room=invalid_room; };
struct PortalSide {
    std::uint32_t id=0, from_room=invalid_room, to_room=invalid_room;
    std::uint32_t from_node=no_node, to_node=no_node;
    bool traversable=true;
};
struct BuildingInput {
    GraphStamp stamp;
    Building building; // canonical contract Room::portals order is preserved
    std::vector<BuildingNode> nodes;
};

class BuildingGraph {
public:
    const GraphStamp& stamp() const;
    std::size_t room_count() const;
    std::size_t portal_count() const;
    std::uint32_t room_for_node(std::uint32_t node) const;
private:
    struct Data;
    std::shared_ptr<const Data> data_;
    explicit BuildingGraph(std::shared_ptr<const Data> data);
    friend class BuildingRouteCache;
    friend class RouteQuery;
};
struct CacheResult {
    std::shared_ptr<const BuildingGraph> graph;
    std::string error;
    bool reused=false;
};
// Prepare once at the level/building scan boundary. Writes are caller-owned;
// shared graphs are immutable and can safely outlive cache eviction.
class BuildingRouteCache {
public:
    CacheResult insert(BuildingInput input);
    void erase_epoch(std::uint64_t epoch);
    void clear();
    std::size_t size() const { return graphs_.size(); }
private:
    std::map<GraphStamp,std::shared_ptr<const BuildingGraph>> graphs_;
};

enum class SideState : std::uint8_t { Unknown, DynamicUnknown, Open, ClosedOpenable, Blocked };
struct SideEvidence {
    std::uint32_t portal_id=0, from_room=invalid_room;
    SideState state=SideState::Unknown;
};
struct RouteRequest {
    QueryStamp stamp;
    std::uint32_t from_room=invalid_room, to_room=invalid_room, avoid_room=invalid_room;
    std::vector<std::uint32_t> searched_rooms;
    std::vector<SideEvidence> evidence;
    // Zero retains frozen Lua ordered BFS: one cost per portal. Nonzero
    // penalties explicitly opt into weighted route selection (not Lua parity).
    double unknown_cost=0;
    double closed_cost=0;
};
enum class RouteStatus : std::uint8_t { Searching, Found, Unreachable, Cancelled, Invalidated, InvalidInput };
struct RouteStep {
    std::uint32_t from_room=invalid_room, room=invalid_room, portal=invalid_room;
    std::uint32_t from_node=no_node, to_node=no_node;
    SideState evidence=SideState::Unknown;
};
struct RouteResult {
    RouteStatus status=RouteStatus::Searching;
    QueryStamp stamp;
    std::vector<RouteStep> steps;
    std::uint32_t next_room=invalid_room, next_portal=invalid_room;
    std::string error;
    double cost=0;
    bool requires_investigation=false;
    bool requires_open=false;
    bool blocked_sides_encountered=false;
    // True only if every side in this candidate was observed Open. This is
    // evidence, never live movement authorization or an obstruction recheck.
    bool all_sides_known_open=false;
    std::size_t work_done=0;
    bool current(const QueryStamp& value) const { return stamp==value; }
};

class RouteQuery {
public:
    RouteQuery(std::shared_ptr<const BuildingGraph> graph, RouteRequest request);
    ~RouteQuery();
    RouteQuery(RouteQuery&&) noexcept;
    RouteQuery& operator=(RouteQuery&&) noexcept;
    RouteQuery(const RouteQuery&)=delete;
    RouteQuery& operator=(const RouteQuery&)=delete;
    // Each unit examines a frontier item, one portal side, or one path step.
    // Deadline/cancellation is checked between units; no per-NPC global cap.
    std::size_t step(std::size_t max_work, const QueryStamp& current,
        bool (*deadline_reached)(void*)=nullptr, void* clock_context=nullptr);
    void cancel();
    const RouteResult& result() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}}}
