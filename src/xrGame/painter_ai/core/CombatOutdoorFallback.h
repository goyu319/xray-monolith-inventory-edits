#pragma once
#include "CombatOutdoorWithdrawal.h"
#include "CombatRouteProof.h"
#include "../CombatIds.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <limits>

namespace painter { namespace combat { namespace core {
struct OutdoorFallbackProduction {
 bool fallback_valid=false,link_up_valid=false,cancelled=false;
 OutdoorFallbackEvidence fallback;
 std::vector<OutdoorWithdrawalCandidate> withdrawal_candidates;
 Vec3 link_up;std::uint32_t link_side=0;ActorKey link_member;
 std::size_t work=0,rays=0;
};
namespace outdoor_fallback_detail {
using Key=std::pair<std::uint32_t,std::uint64_t>;
inline Key key(ActorKey a){return {a.id,a.generation};}
inline bool current(const Config&c,const WorldSnapshot&w,const ActorSnapshot&a){return valid_key(a.key)&&a.level_epoch==w.level_epoch&&
 a.snapshot_seq==w.sequence&&fresh(a.observed_at_ms,w.now_ms,c.snapshot_max_age_ms)&&finite(a.position)&&a.node_id!=invalid_node;}
inline bool same(Vec3 a,Vec3 b){return distance_squared(a,b)==0;}
inline float flat(Vec3 a,Vec3 b){return std::hypot(a.x-b.x,a.z-b.z);}
inline OutdoorStamp stamp(const WorldSnapshot&w,const SideSnapshot&s){return {w.level_epoch,w.map_hash,w.geometry_revision,w.sequence,s.group,w.now_ms,s.side_id,s.faction,s.mission.revision};}
inline bool cache(const WorldSnapshot&w,const ActorSnapshot&a,const ActorCoverCache&c){return c.actor==a.key&&c.level_epoch==w.level_epoch&&
 c.snapshot_seq==w.sequence&&c.ownership_generation==a.ownership_generation&&c.restrictions_revision==a.restrictions_revision&&
 c.origin_node==a.node_id&&c.observed_at_ms==w.now_ms;}
inline bool profile(const WorldSnapshot&w,const ActorSnapshot&a,const OutdoorCoverProfile&p){const auto&s=p.source;return s.actor==a.key&&
  s.level_epoch==w.level_epoch&&s.snapshot_seq==w.sequence&&s.geometry_revision==w.geometry_revision&&s.ownership_generation==a.ownership_generation&&
  s.restrictions_revision==a.restrictions_revision&&s.origin_node==a.node_id&&s.at_ms==a.observed_at_ms&&same(s.actor_position,a.position);}
// A quiet group is affirmative current evidence about every member, rather
// than the absence of one selected enemy on a convenient nearby member.
inline bool quiet_group(const Config&c,const WorldSnapshot&w,const SideSnapshot&side,const KnowledgeFrame&k,
 const std::function<const ActorSnapshot*(ActorKey)>&get){
 if(side.members.empty()||side.mission.kind==MissionKind::attack||side.mission.kind==MissionKind::clear_building||
  !side.known_enemies.empty())return false;
 const SideKnowledge* group_knowledge=nullptr;
 for(const auto&row:k.sides)if(row.side==side.side_id){if(group_knowledge)return false;group_knowledge=&row;}
 if(!group_knowledge||group_knowledge->known_count||group_knowledge->unseen_count||!group_knowledge->hits.empty())return false;
 for(auto member:side.members){const auto*a=get(member);
  if(!a||!current(c,w,*a)||!a->alive||a->wounded||a->faction!=side.faction||
   (a->native_squad&&a->native_squad!=side.group)||
   a->native_properties.enemy||(a->flags&flag_requested_control)||(a->enemy.hostile&&valid_key(a->enemy.key))||
   a->hits.count||a->near_misses.count||a->danger.kind==DangerKind::hit||a->danger.kind==DangerKind::grenade)return false;
  for(std::size_t n=0;n<a->visible_enemies.count&&n<a->visible_enemies.items.size();++n)
   if(a->visible_enemies.items[n].hostile&&a->visible_enemies.items[n].alive_when_observed)return false;
  const MemberKnowledge* mind=nullptr;
  for(const auto&row:k.members)if(row.actor==member&&row.side==side.side_id&&row.ownership_generation==a->ownership_generation){
   if(mind)return false;mind=&row;}
  if(!mind||mind->in_contact||mind->threat_valid||mind->target.valid()||mind->best_visible.valid()||
   mind->dogfight.valid()||mind->fresh_fire.valid||mind->hit_from.valid)return false;
  for(const auto&foe:mind->foes)if(!foe.down)return false;
 }
 return true;
}
}
// Only supplied current combatant captures may provide friendly group anchors.
// A fallback is an actual cover cell, not that occupied friendly anchor. Every
// able member's complete restricted route and every leader-known source line
// must be proved. All primitive queries consume the caller's shared allowance.
inline OutdoorFallbackProduction outdoor_fallback_candidates(const Config&c,const WorldSnapshot&w,const SideSnapshot&s,
 const KnowledgeFrame&k,ActorKey leader,const std::vector<ActorCoverCache>&covers,const OutdoorExecutionEvidence&profiles,
 const RouteProofService&routes,RouteProofBudget&budget,const EngineServices&services,const std::function<bool()>&stop={}) {
 using namespace outdoor_fallback_detail;OutdoorFallbackProduction out;const auto start=budget.work_used;
 auto cancelled=[&]{if(stop&&stop()){out.cancelled=true;return true;}return false;};
 auto finish=[&]{out.work=budget.work_used-start;if(out.cancelled){out.fallback_valid=false;out.link_up_valid=false;out.withdrawal_candidates.clear();}return out;};
 auto charge=[&]{if(cancelled()||!budget.work_left)return false;--budget.work_left;++budget.work_used;return true;};
 if(!w.level_epoch||!w.sequence||!k.accepted||k.cancelled||k.level_epoch!=w.level_epoch||k.sequence!=w.sequence)return finish();
 std::map<Key,const ActorSnapshot*> actors;for(const auto&a:w.actors)if(current(c,w,a)){if(!actors.emplace(key(a.key),&a).second)return finish();}
 auto get=[&](ActorKey a)->const ActorSnapshot*{auto i=actors.find(key(a));return i==actors.end()?nullptr:i->second;};
 const auto*lead=get(leader);if(!lead||!lead->alive||lead->wounded||lead->faction!=s.faction||
  std::find(s.members.begin(),s.members.end(),leader)==s.members.end())return finish();
 const MemberKnowledge*mind=nullptr;for(const auto&m:k.members)if(m.actor==leader&&m.side==s.side_id&&m.ownership_generation==lead->ownership_generation){if(mind)return finish();mind=&m;}
 if(!mind)return finish();
  float nearest=c.link_up_max_distance;
  if(std::isfinite(nearest)&&nearest>0)for(const auto&side:w.sides){if(cancelled())return finish();if(side.side_id==s.side_id||side.group==s.group||side.faction!=s.faction||
   (s.cooperation_side&&side.side_id!=s.cooperation_side)||
   !quiet_group(c,w,side,k,get))continue;
   for(auto member:side.members){const auto*a=get(member);if(!a||!a->alive||a->wounded||a->faction!=s.faction||
    (a->native_squad&&a->native_squad!=side.group))continue;
    const float distance=flat(lead->position,a->position);if(s.cooperation_side&&flat(s.link_up,a->position)>3.f)continue;
    if(distance<=nearest){nearest=distance;out.link_up_valid=true;out.link_up=a->position;out.link_member=a->key;out.link_side=side.side_id;}}}
 struct Known{FoeId id;SourceFact fact;};std::vector<Known> known;
 for(const auto&foe:mind->foes){if(cancelled())return finish();if(!foe.id.valid()||foe.down||
  (!foe.id.ghost()&&std::find(s.members.begin(),s.members.end(),foe.id.actor)!=s.members.end()))continue;
  const SourceFact*fact=nullptr;for(const auto&source:foe.sources)if(source.valid&&valid_key(source.reporter)&&finite(source.position)&&
   source.at_ms<=lead->observed_at_ms&&fresh(source.at_ms,w.now_ms,c.knowledge.assess_foe_ms)&&(!fact||source.at_ms>fact->at_ms))fact=&source;
  if(fact){auto contradicted=[&](const Contact&contact){return !foe.id.ghost()&&contact.key==foe.id.actor&&contact.observed_at_ms>=fact->at_ms&&
    contact.observed_at_ms<=lead->observed_at_ms&&(!contact.hostile||!contact.alive_when_observed||contact.wounded);};
   bool rejected=contradicted(lead->enemy);for(std::size_t i=0;i<lead->visible_enemies.count&&i<lead->visible_enemies.items.size();++i)rejected=rejected||contradicted(lead->visible_enemies.items[i]);
   if(!rejected)known.push_back({foe.id,*fact});}}
 if(known.empty())return finish();
 std::vector<const ActorSnapshot*> able;for(auto member:s.members){const auto*a=get(member);if(a&&a->alive&&!a->wounded&&a->faction==s.faction)able.push_back(a);}
 if(able.size()<2)return finish();
 const auto st=stamp(w,s);const auto&p=c.outdoor.p.assessment;
 float lead_threat=(std::numeric_limits<float>::max)();for(const auto&foe:known)lead_threat=(std::min)(lead_threat,flat(lead->position,foe.fact.position));
 const auto per_actor=(std::max)(std::size_t(1),budget.work_left/(std::max)(std::size_t(1),covers.size())/(known.size()+able.size()+2));
 for(const auto&ca:covers){const auto*a=get(ca.actor);if(!a||!cache(w,*a,ca)||std::find(s.members.begin(),s.members.end(),a->key)==s.members.end())continue;
  const OutdoorCoverProfile*profile_row=nullptr;for(const auto&row:profiles.profiles)if(row.source.actor==a->key&&profile(w,*a,row)){if(profile_row){profile_row=nullptr;break;}profile_row=&row;}
  if(!profile_row||profile_row->source.focus!=ca.knowledge_threat||profile_row->source.focus_at_ms!=ca.knowledge_observed_at_ms||
   !same(profile_row->source.focus_position,ca.knowledge_threat_position))continue;
  std::vector<const CoverSpot*> ordered;for(const auto&spot:ca.spots)if(finite(spot.position))ordered.push_back(&spot);
  auto cost=[&](const CoverSpot&spot){float near=(std::numeric_limits<float>::max)();for(const auto&foe:known)near=(std::min)(near,flat(foe.fact.position,spot.position));
   return out.link_up_valid?flat(spot.position,out.link_up):flat(lead->position,spot.position)-near;};
  std::stable_sort(ordered.begin(),ordered.end(),[&](const CoverSpot*x,const CoverSpot*y){return cost(*x)<cost(*y);});
  std::size_t considered=0;for(const auto*row:ordered){if(considered++>=per_actor)break;const auto&spot=*row;if(!charge())return finish();if(spot.node==invalid_node||!finite(spot.position)||spot.geometry_revision!=w.geometry_revision||
   spot.accessible==Evidence::no||spot.reachable==Evidence::no||spot.protection_known!=Evidence::yes||(!spot.blocked[0]&&!spot.blocked[1]&&!spot.blocked[2])||
   (spot.restrictions_revision&&spot.restrictions_revision!=a->restrictions_revision))continue;
   OutdoorWithdrawalCandidate candidate;candidate.stamp=st;candidate.profile=*profile_row;candidate.spot=spot;bool safe=true;
   for(const auto&foe:known){OutdoorFallbackLine line;line.foe=foe.id;line.source=foe.fact;line.ray_from=foe.fact.position;line.ray_to=spot.position;
    line.ray_from.y+=p.safety_height_m;line.ray_to.y+=p.safety_height_m;
    if(flat(foe.fact.position,spot.position)<=p.unsafe_m)safe=false;
    if(services.rays&&charge()){Segment ray{line.ray_from,line.ray_to};float hit=-2;services.rays(&ray,&hit,1);++out.rays;
     if(cancelled())return finish();if(hit==-1)line.visible=Evidence::yes;else if(std::isfinite(hit)&&hit>=0)line.visible=Evidence::no;}
    if(line.visible!=Evidence::no)safe=false;candidate.lines.push_back(line);
   }
   const auto request=route_proof_source(w,*a,spot.node,spot.position);auto proof=request_route_proof(&routes,request,budget,stop);
   if(proof.status==RouteProofStatus::cancelled){out.cancelled=true;return finish();}
   if(route_proof_complete(proof,request)){auto&path=candidate.route;path.actor=a->key;path.owner=a->ownership_generation;path.restrictions_revision=a->restrictions_revision;
    path.origin_node=a->node_id;path.goal_node=spot.node;path.goal_position=spot.position;path.complete=Evidence::yes;path.points=std::move(proof.points);
    candidate.spot.accessible=candidate.spot.reachable=Evidence::yes;candidate.spot.restrictions_revision=a->restrictions_revision;}
   if(safe&&!out.fallback_valid){float near=(std::numeric_limits<float>::max)();for(const auto&foe:known)near=(std::min)(near,flat(foe.fact.position,spot.position));
    if(near>lead_threat&&flat(lead->position,spot.position)>p.arrived_m){
     OutdoorFallbackEvidence fallback;fallback.stamp=st;fallback.revision=w.sequence;fallback.node=spot.node;fallback.point=spot.position;fallback.lines=candidate.lines;bool valid=true;
     for(const auto*member:able){const auto requested=route_proof_source(w,*member,fallback.node,fallback.point);auto proved=request_route_proof(&routes,requested,budget,stop);
      if(proved.status==RouteProofStatus::cancelled){out.cancelled=true;return finish();}
      if(!route_proof_complete(proved,requested)){valid=false;break;}
      fallback.routes.push_back({member->key,member->ownership_generation,member->restrictions_revision,member->node_id,fallback.node,fallback.point,Evidence::yes,Evidence::yes});}
     if(valid){out.fallback=std::move(fallback);out.fallback_valid=true;}
    }
   }
   out.withdrawal_candidates.push_back(std::move(candidate));
  }
 }
 return finish();
}

// A loose side moves by individually proved cover, without inventing a firing
// base or bound acknowledgement. Regroup uses its own captured leader and the
// existing separation distance; the withdrawal link-up is a separate intent.
// Returned proposals only replace
// ordinary field work; room, hands, reload and accepted bounds retain priority.
inline std::vector<ProposedOrder> outdoor_cover_progress(const Config&c,const WorldSnapshot&w,const SideSnapshot&s,
  const KnowledgeFrame&k,const OutdoorPostureResult&p,ActorKey leader,const std::vector<ActorCoverCache>&covers,
 const std::vector<ProposedOrder>&base,const RouteProofService&routes,RouteProofBudget&budget,const std::function<bool()>&stop={}) {
 using namespace outdoor_fallback_detail;std::vector<ProposedOrder> out;
 if(!k.accepted||k.cancelled||k.level_epoch!=w.level_epoch||k.sequence!=w.sequence||p.status!=OutdoorStatus::ready||
  p.stamp.level_epoch!=w.level_epoch||p.stamp.snapshot_seq!=w.sequence||p.stamp.side!=s.side_id||p.stamp.map_hash!=w.map_hash||
  p.stamp.geometry_revision!=w.geometry_revision||p.stamp.at_ms!=w.now_ms||p.stamp.group!=s.group||p.stamp.faction!=s.faction||
  p.stamp.mission_revision!=s.mission.revision)return out;
  std::map<Key,const ActorSnapshot*> actors;std::map<Key,const ProposedOrder*> bases;
  for(const auto&a:w.actors)if(!actors.emplace(key(a.key),&a).second)return {};
  for(const auto&b:base)if(!bases.emplace(key(b.order.actor),&b).second)return {};
  const auto lead=actors.find(key(leader));
  const ActorSnapshot* own_anchor=lead!=actors.end()&&std::find(s.members.begin(),s.members.end(),leader)!=s.members.end()&&
   current(c,w,*lead->second)&&lead->second->alive&&!lead->second->wounded?lead->second:nullptr;
 std::vector<Vec3> selected;
 for(auto id:s.members){if(stop&&stop())return {};auto ai=actors.find(key(id));auto bi=bases.find(key(id));if(ai==actors.end()||bi==bases.end())continue;
  const auto&a=*ai->second;const auto&b=*bi->second;const auto&o=b.order;
  if(!current(c,w,a)||!a.alive||a.wounded||a.faction!=s.faction||a.yield_reasons||!(a.flags&flag_admitted)||
   a.reloading||a.weapon.state==WeaponState::reloading||!a.weapon.ammo_in_mag||o.assignment.generation||o.portal.valid||o.throw_order.valid||
   valid_key(o.aid)||valid_key(o.finish)||(o.flags&order_committed_entry)||o.kind==OrderKind::bound||o.kind==OrderKind::withdraw||
   o.kind==OrderKind::reload||o.kind==OrderKind::recover||o.kind==OrderKind::yield)continue;
   const bool regroup=own_anchor&&a.key!=leader&&flat(a.position,own_anchor->position)>c.regroup_separation_distance;
   if(!regroup&&!p.loose_advance)continue;
   const Vec3 destination=regroup?own_anchor->position:p.focus_fact.position;if(!finite(destination))continue;
  const MemberKnowledge*mind=nullptr;for(const auto&m:k.members)if(m.actor==a.key&&m.side==s.side_id&&m.ownership_generation==a.ownership_generation){if(mind){mind=nullptr;break;}mind=&m;}
  if(!mind||!mind->alive||!mind->able)continue;
  const ActorCoverCache*ca=nullptr;for(const auto&row:covers)if(row.actor==id&&cache(w,a,row)){if(ca){ca=nullptr;break;}ca=&row;}if(!ca)continue;
  KnowledgeTrace focus;if(!tactical_knowledge_focus(a,w,*mind,focus)||focus.foe!=ca->knowledge_threat||
   focus.observed_at_ms!=ca->knowledge_observed_at_ms||!same(focus.position,ca->knowledge_threat_position))continue;
  std::vector<std::pair<float,const CoverSpot*>> ranked;
  for(const auto&spot:ca->spots){if(!budget.work_left)break;--budget.work_left;++budget.work_used;
   if(spot.node==invalid_node||spot.node==a.node_id||!finite(spot.position)||spot.geometry_revision!=w.geometry_revision||spot.accessible==Evidence::no||spot.reachable==Evidence::no||
    (spot.restrictions_revision&&spot.restrictions_revision!=a.restrictions_revision)||flat(spot.position,destination)>=flat(a.position,destination))continue;
   bool occupied=false;float crowd=0;for(const auto&peer:w.actors)if(peer.key!=id&&peer.alive&&peer.node_id==spot.node){occupied=true;break;}if(occupied)continue;
   for(Vec3 pos:selected){const float d=flat(pos,spot.position);if(d<c.minimum_goal_separation)crowd+=c.minimum_goal_separation-d;}
   const float exposure=spot.protection_known==Evidence::yes?(std::max)(0.f,(std::min)(1.f,spot.lo))*c.cover_exposure_cost_m:c.cover_unknown_cost_m;
   ranked.push_back({flat(spot.position,destination)+flat(a.position,spot.position)+exposure+crowd*c.cover_exposure_cost_m,&spot});
  }
  std::stable_sort(ranked.begin(),ranked.end(),[](const auto&x,const auto&y){return x.first<y.first;});
  for(const auto&candidate:ranked){const auto&spot=*candidate.second;const auto request=route_proof_source(w,a,spot.node,spot.position);
   auto proof=request_route_proof(&routes,request,budget,stop);if(proof.status==RouteProofStatus::cancelled)return {};
   if(!route_proof_complete(proof,request))continue;
   auto proposal=b;proposal.order=first_response(a,c,w.now_ms,w.level_epoch,w.geometry_revision,o.order_seq);
   if(proposal.order.kind==OrderKind::yield||proposal.order.kind==OrderKind::reload)break;
   proposal.order.snapshot_seq=w.sequence;proposal.order.kind=regroup?OrderKind::regroup:OrderKind::advance;
   proposal.order.role=roles::travel;proposal.order.goal_node=spot.node;proposal.order.goal_position=spot.position;proposal.order.pace=Pace::walk;
   proposal.has_cover_evidence=true;proposal.cover_evidence=spot;proposal.cover_evidence.accessible=proposal.cover_evidence.reachable=Evidence::yes;
   proposal.cover_evidence.restrictions_revision=a.restrictions_revision;proposal.navigation_pending=false;proposal.group_id=s.side_id;
   proposal.cover_cost=candidate.first;proposal.has_knowledge=true;proposal.knowledge=focus;out.push_back(std::move(proposal));selected.push_back(spot.position);break;
  }
 }
 return out;
}
}}}
