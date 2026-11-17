#include "CombatRouteProof.h"
#include <algorithm>
namespace painter { namespace combat { namespace core {
namespace {bool point(Vec3 a,Vec3 b){return finite(a)&&finite(b)&&a.x==b.x&&a.y==b.y&&a.z==b.z;}}
RouteProofSource route_proof_source(const WorldSnapshot&w,const ActorSnapshot&a,std::uint32_t node,Vec3 goal){
 RouteProofSource q;q.actor=a.key;q.owner=a.ownership_generation;q.restrictions=a.restrictions_revision;q.epoch=w.level_epoch;q.map=w.map_hash;q.geometry=w.geometry_revision;q.sequence=w.sequence;q.at=w.now_ms;q.origin_node=a.node_id;q.origin=a.position;q.goal_node=node;q.goal=goal;return q;
}
bool same_route_corridor(const RouteProofSource&a,const RouteProofSource&b){return a.actor==b.actor&&a.owner==b.owner&&a.restrictions==b.restrictions&&a.epoch==b.epoch&&a.map==b.map&&a.geometry==b.geometry&&a.origin_node==b.origin_node&&a.goal_node==b.goal_node&&finite(a.origin)&&finite(b.origin)&&point(a.goal,b.goal);}
bool same_route_identity(const RouteProofSource&a,const RouteProofSource&b){return same_route_corridor(a,b)&&point(a.origin,b.origin);}
bool same_route_capture(const RouteProofSource&a,const RouteProofSource&b){return same_route_identity(a,b)&&a.sequence==b.sequence&&a.at==b.at;}
bool route_proof_complete(const RouteProofReceipt&r,const RouteProofSource&q){return r.status==RouteProofStatus::complete&&same_route_capture(r.validated,q)&&same_route_corridor(r.queried,q)&&r.queried.sequence<=q.sequence&&r.queried.at<=q.at&&!r.nodes.empty()&&r.nodes.front()==q.origin_node&&r.nodes.back()==q.goal_node&&r.points.size()==r.nodes.size()+2&&std::all_of(r.points.begin(),r.points.end(),[](Vec3 p){return finite(p);})&&point(r.points.front(),q.origin)&&point(r.points.back(),q.goal);}
RouteProofReceipt request_route_proof(const RouteProofService*s,const RouteProofSource&q,RouteProofBudget&budget,const std::function<bool()>&stop){
 RouteProofReceipt out;out.queried=q;if(stop&&stop()){out.status=RouteProofStatus::cancelled;return out;}
 if(!s||!s->lifetime||!s->context||!s->query||!budget.work_left)return out;
 if(!valid_key(q.actor)||!q.owner||!q.epoch||!q.geometry||!q.sequence||q.origin_node==invalid_node||q.goal_node==invalid_node||!finite(q.origin)||!finite(q.goal))return out;
 out=s->query(s->context,q,budget,stop);
 if(out.status==RouteProofStatus::complete&&!route_proof_complete(out,q)){out={};out.queried=q;}
 return out;
}
}}}
