#include "CombatOutdoorWithdrawal.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <limits>
namespace painter { namespace combat { namespace core {
namespace {
struct Cancelled{};struct Exhausted{};
struct Budget {std::size_t&left;const std::function<bool()>&cancel;void step(){if(cancel&&cancel())throw Cancelled{};if(!left)throw Exhausted{};--left;}};
using Key=std::pair<std::uint32_t,std::uint64_t>;Key key(ActorKey a){return {a.id,a.generation};}
bool pos(float x){return std::isfinite(x)&&x>0;}
float flat(Vec3 a,Vec3 b){return std::hypot(a.x-b.x,a.z-b.z);}
bool equal(Vec3 a,Vec3 b){return finite(a)&&finite(b)&&distance_squared(a,b)==0;}
OutdoorStamp stamp(const WorldSnapshot&w,const SideSnapshot&s){return {w.level_epoch,w.map_hash,w.geometry_revision,w.sequence,s.group,w.now_ms,s.side_id,s.faction,s.mission.revision};}
bool same(OutdoorStamp a,OutdoorStamp b){return a.level_epoch==b.level_epoch&&a.map_hash==b.map_hash&&a.geometry_revision==b.geometry_revision&&a.snapshot_seq==b.snapshot_seq&&
 a.group==b.group&&a.at_ms==b.at_ms&&a.side==b.side&&a.faction==b.faction&&a.mission_revision==b.mission_revision;}
bool context(OutdoorStamp a,OutdoorStamp b){a.snapshot_seq=b.snapshot_seq=0;a.at_ms=b.at_ms=0;return same(a,b);}
bool policy(const OutdoorWithdrawalPolicy&p){return pos(p.min_leg_m)&&p.max_leg_m>=p.min_leg_m&&std::isfinite(p.max_leg_m)&&p.pair_leg_m>=p.min_leg_m&&p.pair_leg_m<=p.max_leg_m&&
 pos(p.arrival_m)&&p.final_walk_m>=p.arrival_m&&std::isfinite(p.final_walk_m)&&pos(p.safety_height_m)&&p.aimed_speed>=.3f&&p.aimed_speed<=1.5f&&p.run_speed>=.3f&&p.run_speed<=1.5f;}
bool live(const ActorSnapshot&a,const WorldSnapshot&w,const Config&c){return a.level_epoch==w.level_epoch&&a.snapshot_seq==w.sequence&&a.observed_at_ms==w.now_ms&&
 fresh(a.observed_at_ms,w.now_ms,c.snapshot_max_age_ms)&&a.alive&&!a.wounded&&a.ownership_generation&&(a.flags&flag_admitted)&&!a.yield_reasons&&!a.native_properties.anomaly&&!(a.native_properties.danger&&a.danger.kind==DangerKind::grenade)&&finite(a.position)&&
 a.node_id!=invalid_node&&a.current_position_accessible&&a.weapon.id!=invalid_id&&(a.weapon.ammo_in_mag||a.weapon.ammo_reserve);}
bool applied(const Order&o,const ActorSnapshot&a,const WorldSnapshot&w){return o.actor==a.key&&o.level_epoch==w.level_epoch&&o.geometry_revision==w.geometry_revision&&
 o.ownership_generation==a.ownership_generation&&o.restrictions_revision==a.restrictions_revision&&o.snapshot_seq<=w.sequence&&o.order_seq&&o.order_seq==a.last_applied.order_seq&&
 o.issued_at_ms<=a.last_applied.at_ms&&a.last_applied.at_ms<=a.observed_at_ms&&o.expires_at_ms>w.now_ms&&a.accessible_current_order&&
 !(a.last_rejected.order_seq==o.order_seq&&a.last_rejected.reason!=reject_none)&&o.goal_node!=invalid_node&&finite(o.goal_position);}
bool same_order(const Order&a,const Order&b){return a.actor==b.actor&&a.order_seq==b.order_seq&&a.level_epoch==b.level_epoch&&a.snapshot_seq==b.snapshot_seq&&
 a.geometry_revision==b.geometry_revision&&a.ownership_generation==b.ownership_generation&&a.restrictions_revision==b.restrictions_revision&&a.kind==b.kind&&
 a.goal_node==b.goal_node&&equal(a.goal_position,b.goal_position)&&a.issued_at_ms==b.issued_at_ms&&a.expires_at_ms==b.expires_at_ms;}
bool hands(const Order&o){return o.throw_order.valid||valid_key(o.aid)||valid_key(o.finish)||o.assignment.portal||o.kind==OrderKind::entry||o.kind==OrderKind::stack||o.kind==OrderKind::peek||o.kind==OrderKind::recover;}
bool fact_equal(const SourceFact&a,const SourceFact&b){return a.valid&&b.valid&&a.at_ms==b.at_ms&&a.reporter==b.reporter&&a.sound==b.sound&&equal(a.position,b.position);}
struct Known {FoeId id;SourceFact fact;};
}
bool outdoor_withdrawal_accepts(const OutdoorWithdrawalIntent&i,const Order&o){return i.episode&&i.leg&&o.actor==i.actor&&o.level_epoch==i.stamp.level_epoch&&
 o.geometry_revision==i.stamp.geometry_revision&&o.snapshot_seq==i.stamp.snapshot_seq&&o.ownership_generation==i.owner&&o.restrictions_revision==i.restrictions_revision&&
 o.order_seq>i.previous_order_seq&&o.kind==OrderKind::withdraw&&!hands(o)&&o.goal_node==i.node&&equal(o.goal_position,i.position)&&
 o.issued_at_ms==i.issued_at_ms&&o.expires_at_ms<=i.expires_at_ms&&o.expires_at_ms>o.issued_at_ms;}
CombatOutdoorWithdrawal::CombatOutdoorWithdrawal(Config c,OutdoorWithdrawalPolicy p):config_(std::move(c)),policy_(p){}
void CombatOutdoorWithdrawal::reset(){stamp_={};episode_=next_leg_=0;cursor_=0;base_.clear();movers_.clear();intents_.clear();}
OutdoorWithdrawalExecution CombatOutdoorWithdrawal::capture_execution(const WorldSnapshot&w,const SideSnapshot&s,const std::vector<Order>&accepted,
 const OutdoorExecutionFacts&e,std::size_t&remaining,const std::function<bool()>&cancel)const{
 OutdoorWithdrawalExecution out;out.stamp=stamp(w,s);const auto before=remaining;Budget b{remaining,cancel};
 auto finish=[&](){out.work=before-remaining;return out;};
 try{b.step();if(!episode_||!context(stamp_,out.stamp)||!same(e.stamp,out.stamp)||!w.sequence)return finish();
  std::map<Key,const ActorSnapshot*>actors;std::map<Key,const Order*>orders;std::map<Key,const OutdoorActorExecution*>rows;std::set<Key>roster;
  for(const auto&a:w.actors){b.step();if(!actors.emplace(key(a.key),&a).second)return finish();}
  for(auto k:s.members){b.step();if(!roster.insert(key(k)).second)return finish();}
  for(const auto&o:accepted){b.step();if(!orders.emplace(key(o.actor),&o).second)return finish();}
  for(const auto&r:e.actors){b.step();if(!rows.emplace(key(r.applied.actor),&r).second)return finish();}
  for(const auto&i:intents_){b.step();auto a=actors.find(key(i.actor));auto o=orders.find(key(i.actor));auto row=rows.find(key(i.actor));
   if(!roster.count(key(i.actor))||i.episode!=episode_||a==actors.end()||o==orders.end()||row==rows.end()||!live(*a->second,w,config_)||
    !outdoor_withdrawal_accepts(i,*o->second)||!same_order(row->second->applied,*o->second)||!applied(*o->second,*a->second,w))continue;
   out.legs.push_back({out.stamp,episode_,i.actor,i.owner,i.restrictions_revision,o->second->order_seq,i.node,i.position});
   if(flat(a->second->position,i.position)<=policy_.arrival_m)out.arrived.push_back(i.actor);
  }out.status=OutdoorStatus::ready;
 }catch(const Cancelled&){out.legs.clear();out.arrived.clear();out.status=OutdoorStatus::cancelled;}
 catch(const Exhausted&){out.legs.clear();out.arrived.clear();out.status=OutdoorStatus::invalid;}
 catch(...){out.legs.clear();out.arrived.clear();out.status=OutdoorStatus::failed;}
 out.work=before-remaining;return finish();
}
OutdoorWithdrawalResult CombatOutdoorWithdrawal::update(const OutdoorWithdrawalRequest&q,std::size_t&remaining,const std::function<bool()>&cancel){
 OutdoorWithdrawalResult out;const auto before=remaining;Budget b{remaining,cancel};
 auto finish=[&](){out.work=before-remaining;return out;};
 try{b.step();if(!q.world||!q.side||!q.knowledge||!q.elements||!q.assessment||!q.execution||!q.accepted_orders||!q.base_orders||!q.candidates||!policy(policy_)||!config_.order_lifetime_ms||!std::isfinite(config_.minimum_goal_separation)||config_.minimum_goal_separation<0||
   !std::isfinite(config_.cover_exposure_cost_m)||config_.cover_exposure_cost_m<0)return finish();
  const auto&w=*q.world;const auto&s=*q.side;const auto&k=*q.knowledge;const auto&assess=*q.assessment;const auto&g=*q.elements;out.stamp=stamp(w,s);
  if(!w.level_epoch||!w.sequence||!s.side_id||!k.accepted||k.cancelled||k.level_epoch!=w.level_epoch||k.sequence!=w.sequence||
   assess.status!=OutdoorStatus::ready||!same(assess.stamp,out.stamp)||!same(q.execution->stamp,out.stamp)||g.side_id!=s.side_id||g.faction!=s.faction||
   (stamp_.level_epoch&&(w.level_epoch<stamp_.level_epoch||(w.level_epoch==stamp_.level_epoch&&(w.sequence<=stamp_.snapshot_seq||w.now_ms<stamp_.at_ms)))))return finish();
  auto n=*this;out.reset=!context(stamp_,out.stamp)||episode_!=assess.serial;if(out.reset)n.reset();
  if(assess.verdict!=OutdoorAssessment::withdraw||!assess.serial||assess.safe_fallback!=Evidence::yes||assess.casualty_obligation_clear!=Evidence::yes||
   assess.completed||assess.capped){out.status=OutdoorStatus::ready;out.reason=assess.completed?"fallback_arrival":assess.capped?"withdraw_cap":"assessment_unavailable";
   out.done=assess.completed||assess.capped;
   // A safety gap blocks new orders, but does not revoke an already issued
   // same-episode intent whose real native acknowledgment may arrive later.
   // Explicit episode/context replacement and completion still discard it.
   if(assess.verdict!=OutdoorAssessment::withdraw||!assess.serial||out.done||out.reset)n.reset();
   else {out.episode=n.episode_;out.base=n.base_;out.movers=n.movers_;}
   n.stamp_=out.stamp;out.work=before-remaining;*this=std::move(n);return finish();}
  n.episode_=assess.serial;out.episode=assess.serial;
  std::map<Key,const ActorSnapshot*>actors;std::map<Key,const MemberKnowledge*>minds;std::map<Key,const Order*>accepted;
  std::map<Key,const ProposedOrder*>bases;std::map<Key,const OutdoorActorExecution*>rows;std::set<Key>roster;
  for(const auto&a:w.actors){b.step();if(!valid_key(a.key)||!actors.emplace(key(a.key),&a).second)return finish();}
  for(const auto&m:k.members){b.step();if(!minds.emplace(key(m.actor),&m).second)return finish();}
  for(auto a:s.members){b.step();if(!valid_key(a)||!roster.insert(key(a)).second)return finish();}
  for(const auto&o:*q.accepted_orders){b.step();if(!accepted.emplace(key(o.actor),&o).second)return finish();}
  for(const auto&p:*q.base_orders){b.step();if(!bases.emplace(key(p.order.actor),&p).second)return finish();}
  for(const auto&r:q.execution->actors){b.step();if(!rows.emplace(key(r.applied.actor),&r).second)return finish();}
  auto get=[&](ActorKey x)->const ActorSnapshot*{auto i=actors.find(key(x));return i==actors.end()||i->second->key!=x?nullptr:i->second;};
  auto eligible=[&](ActorKey x){const auto*a=get(x);return roster.count(key(x))&&a&&a->faction==s.faction&&live(*a,w,config_);};
  auto mind=[&](ActorKey x)->const MemberKnowledge*{auto i=minds.find(key(x));const auto*a=get(x);return a&&i!=minds.end()&&i->second->side==s.side_id&&
   i->second->ownership_generation==a->ownership_generation?i->second:nullptr;};
  if(out.reset){std::set<Key>assigned;for(auto x:g.base){b.step();if(!roster.count(key(x))||!assigned.insert(key(x)).second)return finish();n.base_.push_back(x);}
   for(auto x:g.movers){b.step();if(!roster.count(key(x))||!assigned.insert(key(x)).second)return finish();n.movers_.push_back(x);}}
  auto prune=[&](std::vector<ActorKey>&v){v.erase(std::remove_if(v.begin(),v.end(),[&](ActorKey x){b.step();const auto*a=get(x);return !roster.count(key(x))||(a&&a->level_epoch==w.level_epoch&&a->snapshot_seq==w.sequence&&fresh(a->observed_at_ms,w.now_ms,config_.snapshot_max_age_ms)&&(!a->alive||a->wounded));}),v.end());};
  prune(n.base_);prune(n.movers_);
  // A casualty may leave an empty element; split only actually present able men.
  if(n.base_.empty()||n.movers_.empty()){std::vector<ActorKey>available;for(auto x:s.members){b.step();if(eligible(x))available.push_back(x);}if(available.size()>=2){
    n.base_.assign(available.begin(),available.begin()+available.size()/2);n.movers_.assign(available.begin()+available.size()/2,available.end());}}
  const auto*leader=mind(g.leader);const auto*la=get(g.leader);std::vector<Known>known;
  if(leader&&la)for(const auto&f:leader->foes){b.step();if((f.down&&valid_key(f.status_reporter)&&f.down_ms<=la->observed_at_ms)||!f.id.valid()||(!f.id.ghost()&&roster.count(key(f.id.actor))))continue;
   const SourceFact*sf=nullptr;for(const auto&candidate:f.sources){b.step();if(candidate.valid&&finite(candidate.position)&&valid_key(candidate.reporter)&&candidate.at_ms<=la->observed_at_ms&&
    fresh(candidate.at_ms,w.now_ms,config_.knowledge.assess_foe_ms)&&(!sf||candidate.at_ms>sf->at_ms))sf=&candidate;}if(sf)known.push_back({f.id,*sf});}
  if(!leader||known.empty()){out.status=OutdoorStatus::ready;out.reason="known_source_unavailable";out.base=n.base_;out.movers=n.movers_;out.work=before-remaining;return finish();}
  const bool executing=q.withdrawal_execution&&q.withdrawal_execution->status==OutdoorStatus::ready&&same(q.withdrawal_execution->stamp,out.stamp);
  std::set<Key>arrived,executed;if(executing){for(const auto&x:q.withdrawal_execution->legs){b.step();const auto*a=get(x.actor);auto o=accepted.find(key(x.actor));auto row=rows.find(key(x.actor));
    if(!a||o==accepted.end()||row==rows.end()||x.assessment_serial!=n.episode_||!same(x.stamp,out.stamp)||x.owner!=a->ownership_generation||
      x.restrictions_revision!=a->restrictions_revision||x.order_seq!=o->second->order_seq||x.goal_node!=o->second->goal_node||!equal(x.goal_position,o->second->goal_position)||
      !applied(*o->second,*a,w)||!same_order(*o->second,row->second->applied))continue;
    bool staged=false;for(const auto&i:n.intents_){b.step();staged|=i.actor==x.actor&&i.episode==n.episode_&&outdoor_withdrawal_accepts(i,*o->second);}
    if(!staged)continue;executed.insert(key(x.actor));if(flat(a->position,x.goal_position)<=policy_.arrival_m)arrived.insert(key(x.actor));}}
  bool wave_complete=!n.movers_.empty();for(auto x:n.movers_){b.step();wave_complete=wave_complete&&executed.count(key(x))&&arrived.count(key(x));}
  if(wave_complete){std::swap(n.base_,n.movers_);out.swapped=true;}
  out.base=n.base_;out.movers=n.movers_;
  // A real held cover row suffices; this layer never claims a shot occurred.
  bool covered_base=false;for(auto x:n.base_){b.step();auto o=accepted.find(key(x));auto row=rows.find(key(x));const auto*a=get(x);
   if(!eligible(x)||!a||o==accepted.end()||row==rows.end()||!applied(*o->second,*a,w)||!same_order(*o->second,row->second->applied)||hands(*o->second))continue;
   const auto&r=*row->second;if(flat(a->position,o->second->goal_position)<=policy_.arrival_m&&r.cover_verified==Evidence::yes&&
    r.cover!=OutdoorCover::unknown&&r.cover!=OutdoorCover::open)covered_base=true;}
  auto refresh=[&](const ActorSnapshot&a,const Order&old,const ProposedOrder&base,const OutdoorWithdrawalIntent&prior){
   b.step();const auto applied_intent=prior;OutdoorWithdrawalOrder r;r.intent=prior;r.continuing=true;r.proposal=base;r.proposal.order=old;
   const auto reflex=first_response(a,config_,w.now_ms,w.level_epoch,w.geometry_revision,old.order_seq);
   auto&o=r.proposal.order;o.snapshot_seq=w.sequence;o.issued_at_ms=w.now_ms;o.expires_at_ms=saturated_after(w.now_ms,config_.order_lifetime_ms);
   o.look=reflex.look;o.fire=reflex.fire;if(o.fire.until_ms>o.expires_at_ms)o.fire.until_ms=o.expires_at_ms;
   if(a.reloading||a.weapon.state==WeaponState::reloading||!a.weapon.ammo_in_mag){o.fire={};o.pace=Pace::run;o.speed_scale=policy_.run_speed;o.flags|=order_face_movement;}
   else if(reflex.look.kind==LookKind::enemy){o.pace=Pace::walk;o.speed_scale=policy_.aimed_speed;o.flags&=~order_face_movement;}
   r.proposal.expected_previous_order_seq=old.order_seq;r.proposal.group_id=g.group_id;
   r.intent.stamp=out.stamp;r.intent.previous_order_seq=old.order_seq;r.intent.issued_at_ms=o.issued_at_ms;r.intent.expires_at_ms=o.expires_at_ms;
   // Keep the actually-applied generation until the refresh itself is accepted.
   n.intents_.erase(std::remove_if(n.intents_.begin(),n.intents_.end(),[&](const OutdoorWithdrawalIntent&i){b.step();return i.actor==a.key;}),n.intents_.end());
   n.intents_.push_back(applied_intent);n.intents_.push_back(r.intent);out.orders.push_back(r);
  };
  // Arrived members remain on their real withdrawal leg while serving as base;
  // an unrelated ordinary hold would erase the final all-member arrival proof.
  for(auto x:n.base_){b.step();const auto*a=get(x);auto old=accepted.find(key(x));auto bp=bases.find(key(x));
   if(!eligible(x)||!a||old==accepted.end()||bp==bases.end()||hands(bp->second->order)||!applied(*old->second,*a,w))continue;
   auto i=std::find_if(n.intents_.begin(),n.intents_.end(),[&](const OutdoorWithdrawalIntent&i){b.step();return i.actor==x&&outdoor_withdrawal_accepts(i,*old->second);});
   if(i!=n.intents_.end()&&flat(a->position,old->second->goal_position)<=policy_.arrival_m)refresh(*a,*old->second,*bp->second,*i);
  }
  std::vector<std::pair<ActorKey,Vec3>>occupied;for(auto x:s.members){b.step();const auto*a=get(x);if(a&&a->alive){occupied.push_back({x,a->position});auto o=accepted.find(key(x));
   if(o!=accepted.end()&&applied(*o->second,*a,w))occupied.push_back({x,o->second->goal_position});}}
  std::size_t able_count=0;for(auto x:s.members){b.step();able_count+=eligible(x)?1:0;}
  const float max_leg=able_count==2?policy_.pair_leg_m:policy_.max_leg_m;
  for(std::size_t offset=0;offset<n.movers_.size();++offset){b.step();const auto mi=(n.cursor_+offset)%n.movers_.size();const auto x=n.movers_[mi];const auto*a=get(x);
   auto bp=bases.find(key(x));auto old=accepted.find(key(x));const auto*km=mind(x);
   if(!eligible(x)||!a||!km||bp==bases.end()||hands(bp->second->order)||(old!=accepted.end()&&hands(*old->second)))continue;
   auto staged=n.intents_.end();for(auto i=n.intents_.begin();i!=n.intents_.end();++i){b.step();if(i->actor==x&&(staged==n.intents_.end()||
     (old!=accepted.end()&&outdoor_withdrawal_accepts(*i,*old->second))))staged=i;}
   const bool moving=executing&&executed.count(key(x))&&!arrived.count(key(x));
   if(moving&&staged!=n.intents_.end()&&old!=accepted.end()&&applied(*old->second,*a,w)&&outdoor_withdrawal_accepts(*staged,*old->second)){
    refresh(*a,*old->second,*bp->second,*staged);continue;}
   // A published but not yet applied leg remains a stable proposal; it earns
   // neither arrival nor element swap and does not create another local leg.
   if(staged!=n.intents_.end()&&staged->owner==a->ownership_generation&&staged->restrictions_revision==a->restrictions_revision&&staged->expires_at_ms>w.now_ms&&!out.swapped)continue;
   if(!covered_base||a->reloading||a->weapon.state==WeaponState::reloading||!a->weapon.ammo_in_mag||able_count<2)continue;
   const OutdoorWithdrawalCandidate*best=nullptr;float best_cost=(std::numeric_limits<float>::max)();KnowledgeTrace best_trace;
   for(const auto&c:*q.candidates){b.step();if(c.profile.source.actor!=x)continue;++out.examined;bool valid=true;const auto&src=c.profile.source;const auto&spot=c.spot;
    const auto reject=[&](){++out.rejected;};
    if(!same(c.stamp,out.stamp)||src.level_epoch!=w.level_epoch||src.snapshot_seq!=w.sequence||src.geometry_revision!=w.geometry_revision||
     src.ownership_generation!=a->ownership_generation||src.restrictions_revision!=a->restrictions_revision||src.at_ms!=w.now_ms||src.origin_node!=a->node_id||
     !equal(src.actor_position,a->position)||c.profile.heights!=std::array<float,3>{{.7f,1.1f,1.7f}}||spot.node==invalid_node||!finite(spot.position)||
     spot.geometry_revision!=w.geometry_revision||spot.restrictions_revision!=a->restrictions_revision||spot.accessible!=Evidence::yes||spot.reachable!=Evidence::yes||
     spot.protection_known!=Evidence::yes||(!spot.blocked[0]&&!spot.blocked[1]&&!spot.blocked[2])||!std::isfinite(spot.lo)||spot.lo<0||spot.lo>1||
     !src.focus.valid()||spot.threat_key!=src.focus.actor||spot.threat_observed_at_ms!=src.focus_at_ms){reject();continue;}
    const SourceFact*source=nullptr;for(const auto&f:km->foes){b.step();if(f.id==src.focus&&!f.down)for(const auto&sf:f.sources){b.step();if(sf.valid&&sf.at_ms==src.focus_at_ms&&
      valid_key(sf.reporter)&&equal(sf.position,src.focus_position)&&fresh(sf.at_ms,w.now_ms,config_.knowledge.assess_foe_ms))source=&sf;}}
    if(!source){reject();continue;}
    const float travel=flat(a->position,spot.position),progress=flat(a->position,assess.fallback)-flat(spot.position,assess.fallback);
    if(travel<=policy_.arrival_m||travel>max_leg||progress<=0){reject();continue;}
    const auto&path=c.route;if(path.actor!=x||path.owner!=a->ownership_generation||path.restrictions_revision!=a->restrictions_revision||path.origin_node!=a->node_id||
     path.goal_node!=spot.node||!equal(path.goal_position,spot.position)||path.complete!=Evidence::yes||path.points.size()<2||!equal(path.points.front(),a->position)||!equal(path.points.back(),spot.position)){reject();continue;}
    for(auto p:path.points){b.step();if(!finite(p))valid=false;}
    for(const auto&f:known){b.step();const OutdoorFallbackLine*line=nullptr;bool duplicate=false;for(const auto&l:c.lines){b.step();if(l.foe==f.id&&fact_equal(l.source,f.fact)){duplicate|=line!=nullptr;line=&l;}}
     Vec3 from=f.fact.position,to=spot.position;from.y+=policy_.safety_height_m;to.y+=policy_.safety_height_m;
     if(!line||duplicate||line->visible!=Evidence::no||!equal(line->ray_from,from)||!equal(line->ray_to,to))valid=false;}
    for(const auto&place:occupied){b.step();if(place.first!=x&&flat(place.second,spot.position)<config_.minimum_goal_separation)valid=false;}
    if(!valid){reject();continue;}const float cost=travel+flat(spot.position,assess.fallback)+std::max(0.f,policy_.min_leg_m-travel)+spot.lo*config_.cover_exposure_cost_m;
    if(std::isfinite(cost)&&(!best||cost<best_cost||(cost==best_cost&&spot.node<best->spot.node))){best=&c;best_cost=cost;best_trace={};best_trace.receiver=x;
      best_trace.receiver_ownership_generation=a->ownership_generation;best_trace.receiver_faction=a->faction;best_trace.foe=src.focus;best_trace.target=src.focus.actor;
      best_trace.observer=source->reporter;best_trace.observed_at_ms=source->at_ms;best_trace.position=source->position;best_trace.shared=source->reporter!=x;}
   }
   if(!best)continue;b.step();if(n.next_leg_==(std::numeric_limits<std::uint64_t>::max)())return finish();
   OutdoorWithdrawalOrder r;r.proposal=*bp->second;r.proposal.order=first_response(*a,config_,w.now_ms,w.level_epoch,w.geometry_revision,a->last_applied.order_seq);
   auto&o=r.proposal.order;o.kind=OrderKind::withdraw;o.role=roles::mover;o.assignment={};o.goal_node=best->spot.node;o.goal_position=best->spot.position;o.arrival_radius_m=policy_.arrival_m;
   o.flags&=~order_first_response;o.stance=Stance::stand;const bool visible=o.look.kind==LookKind::enemy;
   o.pace=visible||flat(a->position,o.goal_position)<=policy_.final_walk_m?Pace::walk:Pace::run;o.speed_scale=o.pace==Pace::walk?policy_.aimed_speed:policy_.run_speed;
   if(!visible){o.flags|=order_face_movement;o.look={LookKind::point,best_trace.position,{}};o.fire={};}
   r.proposal.expected_previous_order_seq=old==accepted.end()?a->last_applied.order_seq:old->second->order_seq;r.proposal.group_id=g.group_id;
   r.proposal.has_cover_evidence=true;r.proposal.cover_evidence=best->spot;r.proposal.cover_cost=best_cost;r.proposal.has_knowledge=true;r.proposal.knowledge=best_trace;
   r.intent={out.stamp,n.episode_,++n.next_leg_,x,a->ownership_generation,a->restrictions_revision,r.proposal.expected_previous_order_seq,o.goal_node,o.goal_position,o.issued_at_ms,o.expires_at_ms};
   n.intents_.erase(std::remove_if(n.intents_.begin(),n.intents_.end(),[&](const OutdoorWithdrawalIntent&i){b.step();return i.actor==x;}),n.intents_.end());
   n.intents_.push_back(r.intent);occupied.push_back({x,o.goal_position});out.orders.push_back(r);
  }
  if(!n.movers_.empty())n.cursor_=(n.cursor_+1)%n.movers_.size();b.step();out.status=OutdoorStatus::ready;out.reason=out.orders.empty()?(covered_base?"no_safe_leg":"base_cover_unverified"):"withdrawal_proposals";
  n.stamp_=out.stamp;out.work=before-remaining;*this=std::move(n);return finish();
 }catch(const Cancelled&){out.orders.clear();out.status=OutdoorStatus::cancelled;out.reason="cancelled";}
 catch(const Exhausted&){out.orders.clear();out.status=OutdoorStatus::ready;out.exhausted=true;out.reason="work_exhausted";out.base=base_;out.movers=movers_;out.swapped=false;out.reset=false;out.episode=episode_;}
 catch(...){out.orders.clear();out.status=OutdoorStatus::failed;out.reason="exception";}
 out.work=before-remaining;return finish();
}
}}}
