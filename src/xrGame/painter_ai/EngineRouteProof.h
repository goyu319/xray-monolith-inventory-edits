#pragma once
#include "core/CombatRouteProof.h"
#include "NodeRoute.h"
#include <memory>
namespace painter { namespace combat { namespace engine {
// Explicit private review policy. No enabled/default tuning is invented.
struct RouteProofPolicy {
 std::size_t queue_bytes=0,job_overhead_bytes=0,worker_work=0,authority_work=0;
 std::uint32_t expansion_limit=0,output_limit=0;TimeMs source_age_ms=0;
 float endpoint_height_m=0;
};
struct RouteAuthority {
 void* context=nullptr;
 bool(*identity)(void*,const core::RouteProofSource&,const core::RouteProofSource&)=nullptr;
 Evidence(*node)(void*,const core::RouteProofSource&,const core::RouteProofSource&,std::uint32_t,std::uint32_t,Vec3)=nullptr;
 Evidence(*attachment)(void*,const core::RouteProofSource&,const core::RouteProofSource&,bool,Vec3,float)=nullptr;
};
// Adapter supplies these callbacks on its authority thread. They obtain the
// current registered object for each call; no pointer enters this queue.
RouteAuthority route_authority();
struct RouteAdvance {std::size_t work=0,queued_bytes=0,completed=0,rejected=0;};
class EngineRouteProof:public std::enable_shared_from_this<EngineRouteProof> {
public:
 using Search=StaticRouteResult(*)(const StaticRouteRequest&,StaticRouteQuota&,const std::function<bool()>&);
 EngineRouteProof(RouteProofPolicy,Search);
 ~EngineRouteProof();
 core::RouteProofService service();
 RouteAdvance advance(const WorldSnapshot&,const RouteAuthority&);
private:
 struct Impl;std::unique_ptr<Impl> impl_;
 core::RouteProofReceipt query(const core::RouteProofSource&,core::RouteProofBudget&,const std::function<bool()>&);
};
}}}
