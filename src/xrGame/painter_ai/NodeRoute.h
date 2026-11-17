#pragma once
// Static graph route with the SAME four-link geometry as NodePath. A complete
// centre polyline is not an actor-restriction proof, a nav-cell inset proof, or
// an exact foot/goal attachment. No partial route is labelled complete.
#include "CombatContract.h"
#include <functional>
namespace painter { namespace combat { namespace engine {
enum class StaticRouteStatus {invalid,complete,unreachable,exhausted,cancelled,failed};
struct StaticRouteRequest {
    ActorKey actor;std::uint64_t level_epoch=0,snapshot_seq=0,geometry_revision=0,ownership_generation=0,restrictions_revision=0;
    TimeMs captured_at_ms=0;std::uint32_t origin_node=invalid_node,goal_node=invalid_node;
    std::uint32_t expansion_limit=0,output_limit=0;
};
struct StaticRouteQuota {std::size_t work_left=0,work_used=0;};
struct StaticRouteResult {
    StaticRouteRequest source;StaticRouteStatus status=StaticRouteStatus::invalid;
    Evidence restricted_route=Evidence::unknown; // ALWAYS unknown in this static-only producer
    std::vector<std::uint32_t> nodes;std::vector<Vec3> centres;
    float length=0;std::size_t expansions=0,work=0;
};
// Synchronous erased view: caller retains the pinned graph for this call only.
// No context pointer is retained in a result or queued request.
struct StaticRouteGraph {
    const void* context=nullptr;
    bool(*valid)(const void*,std::uint32_t)=nullptr;
    void(*centre)(const void*,std::uint32_t,float(&)[3])=nullptr;
    std::uint32_t(*link)(const void*,std::uint32_t,int)=nullptr;
};
StaticRouteResult build_static_route_query(const StaticRouteGraph&,const StaticRouteRequest&,StaticRouteQuota&,const std::function<bool()>&cancelled={});
// Current owner integration exports this private callback. A future canonical
// callback must retain these source stamps and distinguish static vs restricted.
StaticRouteResult query_static_route(const StaticRouteRequest&,StaticRouteQuota&,const std::function<bool()>&cancelled={});
}}}
