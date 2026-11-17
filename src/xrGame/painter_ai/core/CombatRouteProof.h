#pragma once
#include "CombatCore.h"
#include <functional>
#include <memory>
namespace painter { namespace combat { namespace core {
struct RouteProofSource {
 ActorKey actor;std::uint64_t owner=0,restrictions=0,epoch=0,map=0,geometry=0,sequence=0;TimeMs at=0;
 std::uint32_t origin_node=invalid_node,goal_node=invalid_node;Vec3 origin,goal;
};
enum class RouteProofStatus {unknown,pending,complete,blocked,stale,cancelled,budget,failed};
struct RouteProofReceipt {
 // queried preserves the original request; validated is fresh AUTHORITY work,
 // not a retimestamped sight/knowledge fact. The validated current origin
 // attachment is exact; the original request may start elsewhere in that cell.
 RouteProofSource queried,validated;RouteProofStatus status=RouteProofStatus::unknown;
 std::vector<std::uint32_t> nodes;std::vector<Vec3> points;
};
struct RouteProofBudget {std::size_t work_left=0,work_used=0;};
struct RouteProofService {
 // Lifetime is value-only queue ownership; no engine object pointer is retained.
 std::shared_ptr<void> lifetime;void* context=nullptr;std::size_t work_per_update=0;
 RouteProofReceipt(*query)(void*,const RouteProofSource&,RouteProofBudget&,const std::function<bool()>&)=nullptr;
};
RouteProofSource route_proof_source(const WorldSnapshot&,const ActorSnapshot&,std::uint32_t,Vec3);
// Static corridor identity permits a new exact attachment inside the same origin node.
bool same_route_corridor(const RouteProofSource&,const RouteProofSource&);
bool same_route_identity(const RouteProofSource&,const RouteProofSource&);
bool same_route_capture(const RouteProofSource&,const RouteProofSource&);
bool route_proof_complete(const RouteProofReceipt&,const RouteProofSource&);
RouteProofReceipt request_route_proof(const RouteProofService*,const RouteProofSource&,RouteProofBudget&,const std::function<bool()>& cancelled={});
}}}

