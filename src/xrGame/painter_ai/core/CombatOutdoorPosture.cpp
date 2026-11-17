#include "CombatOutdoorPosture.h"
#include "CombatCore.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <tuple>

namespace painter { namespace combat { namespace core {
namespace {
using Key=std::pair<std::uint32_t,std::uint64_t>;
Key key(ActorKey k){return {k.id,k.generation};}
bool positive(float x){return std::isfinite(x)&&x>0;}
float flat(Vec3 a,Vec3 b){return std::hypot(a.x-b.x,a.z-b.z);}
bool policy_valid(const OutdoorPolicy& p){
    return p.clear_seen_ms&&p.clear_hit_ms&&p.clear_heard_ms&&p.fired_recent_ms&&
        p.attack_stall_ms&&p.defend_attack_ms&&p.lk_empty_ms&&p.assess_cooldown_ms&&
        p.handover_cap_ms&&p.handover_stall_ms&&p.endpoint_fail_max&&positive(p.near_m)&&
        positive(p.assault_m)&&positive(p.search_reach_m)&&positive(p.arrived_m)&&
        positive(p.door_m)&&positive(p.fight_m)&&positive(p.stall_near_m)&&positive(p.stall_gain_m);
}
bool mission_equal(const Mission&a,const Mission&b){
    if(a.revision!=b.revision||a.kind!=b.kind||a.building!=b.building||a.entry_portal!=b.entry_portal||a.rooms.count!=b.rooms.count)return false;
    for(std::size_t i=0;i<a.rooms.count;++i)if(a.rooms.items[i]!=b.rooms.items[i])return false;
    return true;
}
OutdoorStamp stamp(const WorldSnapshot&w,const SideSnapshot&s){
    OutdoorStamp x;x.level_epoch=w.level_epoch;x.map_hash=w.map_hash;x.geometry_revision=w.geometry_revision;
    x.snapshot_seq=w.sequence;x.group=s.group;x.at_ms=w.now_ms;x.side=s.side_id;x.faction=s.faction;x.mission_revision=s.mission.revision;return x;
}
bool same_stamp(const OutdoorStamp&a,const OutdoorStamp&b){
    return a.level_epoch==b.level_epoch&&a.map_hash==b.map_hash&&a.geometry_revision==b.geometry_revision&&
        a.snapshot_seq==b.snapshot_seq&&a.group==b.group&&a.at_ms==b.at_ms&&a.side==b.side&&
        a.faction==b.faction&&a.mission_revision==b.mission_revision;
}
bool identity(const OutdoorStamp&a,const OutdoorStamp&b){
    return a.level_epoch==b.level_epoch&&a.map_hash==b.map_hash&&a.geometry_revision==b.geometry_revision&&
        a.group==b.group&&a.side==b.side&&a.faction==b.faction;
}
bool able(const ActorSnapshot&a,const WorldSnapshot&w,const Config&c){
    return a.level_epoch==w.level_epoch&&a.snapshot_seq==w.sequence&&a.ownership_generation&&
        fresh(a.observed_at_ms,w.now_ms,c.snapshot_max_age_ms)&&a.alive&&!a.wounded&&!a.yield_reasons&&
        (a.flags&flag_admitted)&&finite(a.position)&&a.current_position_accessible&&a.node_id!=invalid_node&&
        a.weapon.kind!=WeaponKind::none&&(a.weapon.ammo_in_mag||a.weapon.ammo_reserve);
}
const SourceFact* source(const FoeKnowledge&f,TimeMs now){
    const SourceFact* best=nullptr;
    for(const auto& x:f.sources)if(x.valid&&finite(x.position)&&x.at_ms<=now&&(!best||x.at_ms>best->at_ms))best=&x;
    return best;
}
bool foe_less(const FoeId&a,const FoeId&b){return std::tie(a.actor.id,a.actor.generation,a.side,a.ghost_serial)<std::tie(b.actor.id,b.actor.generation,b.side,b.ghost_serial);}
bool applied(const Order&o,const ActorSnapshot&a,const WorldSnapshot&w){
    return o.actor==a.key&&o.level_epoch==w.level_epoch&&o.geometry_revision==w.geometry_revision&&
        o.ownership_generation==a.ownership_generation&&o.restrictions_revision==a.restrictions_revision&&
        o.order_seq&&o.order_seq==a.last_applied.order_seq&&a.accessible_current_order&&o.snapshot_seq<=w.sequence&&
        o.issued_at_ms<=a.last_applied.at_ms&&a.last_applied.at_ms<=a.observed_at_ms&&
        o.expires_at_ms>w.now_ms&&o.goal_node!=invalid_node&&finite(o.goal_position)&&positive(o.arrival_radius_m);
}
bool visible(const ActorSnapshot&a,const FoeId&f){
    if(f.ghost()||!f.valid())return false;
    auto match=[&](const Contact&c){return c.key==f.actor&&c.hostile&&c.alive_when_observed&&personally_visible(c,a);};
    if(match(a.enemy))return true;
    for(std::size_t i=0;i<std::min<std::size_t>(a.visible_enemies.count,a.visible_enemies.items.size());++i)if(match(a.visible_enemies.items[i]))return true;
    return false;
}
bool combat(OutdoorPosture p){return p==OutdoorPosture::contact||p==OutdoorPosture::defend||p==OutdoorPosture::attack||p==OutdoorPosture::assault||p==OutdoorPosture::withdraw;}
}

CombatOutdoorPosture::CombatOutdoorPosture(Config c,OutdoorPolicy p):config_(std::move(c)),policy_(p){}
void CombatOutdoorPosture::reset(){CombatOutdoorPosture fresh_state(config_,policy_);*this=std::move(fresh_state);}

OutdoorPostureResult CombatOutdoorPosture::update(const WorldSnapshot&w,const SideSnapshot&s,
    const KnowledgeFrame&k,const CombatMissionIntel&i,const OutdoorExecutionFacts&e,
    const std::function<bool()>&cancelled){
    OutdoorPostureResult fail;fail.stamp=stamp(w,s);
    auto stop=[&](){return cancelled&&cancelled();};
    auto cancelled_result=[&](){fail.status=OutdoorStatus::cancelled;fail.reason="cancelled";return fail;};
    try {
        if(stop())return cancelled_result();
        if(!policy_valid(policy_)||config_.factions.count>config_.factions.items.size()||
            !w.level_epoch||!w.sequence||!s.side_id||s.members.empty()||s.mission.rooms.count>s.mission.rooms.items.size()||
            !k.accepted||k.cancelled||k.level_epoch!=w.level_epoch||k.sequence!=w.sequence||
            i.status!=MissionIntelStatus::ready||i.level_epoch!=w.level_epoch||i.map_hash!=w.map_hash||
            i.geometry_revision!=w.geometry_revision||i.snapshot_seq!=w.sequence||i.side!=s.side_id||
            i.group!=s.group||i.faction!=s.faction||!mission_equal(i.requested,s.mission)){
            fail.reason="invalid_policy_or_source";return fail;
        }
        if(initialized_&&(w.level_epoch<state_.stamp.level_epoch||
            (w.level_epoch==state_.stamp.level_epoch&&(w.sequence<=state_.stamp.snapshot_seq||w.now_ms<state_.stamp.at_ms)))){
            fail.status=OutdoorStatus::stale;fail.reason="stale_capture";return fail;
        }
        std::map<Key,const ActorSnapshot*> actors;std::map<Key,const MemberKnowledge*> knowledge;
        std::set<Key> roster;std::map<Key,const MemberState*> previous;
        for(const auto&a:w.actors){if(stop())return cancelled_result();if(!valid_key(a.key)||!actors.emplace(key(a.key),&a).second){fail.reason="invalid_actor_keys";return fail;}}
        for(const auto&m:k.members){if(stop())return cancelled_result();if(!valid_key(m.actor)||!knowledge.emplace(key(m.actor),&m).second){fail.reason="invalid_knowledge_keys";return fail;}}
        for(const auto&m:members_){if(stop())return cancelled_result();previous.emplace(key(m.key),&m);}
        bool continuity=false;
        for(const auto&a:s.members){
            if(stop())return cancelled_result();if(!valid_key(a)||!roster.emplace(key(a)).second){fail.reason="invalid_side_keys";return fail;}
            continuity=continuity||previous.count(key(a))!=0;
            auto f=actors.find(key(a));if(f!=actors.end()&&f->second->faction!=s.faction){fail.reason="side_faction_conflict";return fail;}
        }
        const bool reset_state=!initialized_||!identity(state_.stamp,fail.stamp)||!mission_equal(mission_,s.mission)||!continuity;
        CombatOutdoorPosture n(config_,policy_); // vectors are copied below with cancellation polling
        if(!reset_state){
            n.state_=state_;n.has_info_=has_info_;n.has_seen_=has_seen_;n.has_hit_=has_hit_;n.has_heard_=has_heard_;n.has_supp_=has_supp_;
            n.info_at_=info_at_;n.seen_at_=seen_at_;n.hit_at_=hit_at_;n.heard_at_=heard_at_;n.supp_at_=supp_at_;
            n.stalled_=stalled_;n.stalled_at_=stalled_at_;n.defend_ok_=defend_ok_;n.defend_ok_at_=defend_ok_at_;
            n.withdrew_=withdrew_;n.withdraw_at_=withdraw_at_;n.mission_at_=mission_at_;
            n.endpoint_serial_=endpoint_serial_;n.assessment_serial_=assessment_serial_;
            n.door_progress_=door_progress_;n.door_best_m_=door_best_m_;n.door_progress_at_=door_progress_at_;
            n.door_point_=door_point_;n.door_portal_=door_portal_;
            n.withdrawal_goal_=withdrawal_goal_;n.withdrawal_point_=withdrawal_point_;n.withdrawal_serial_=withdrawal_serial_;
            n.cleared_activity_=cleared_activity_;n.cleared_activity_ms_=cleared_activity_ms_;
            n.exchange_latched_=exchange_latched_;n.exchange_favourable_=exchange_favourable_;
            n.exchange_at_=exchange_at_;n.exchange_until_=exchange_until_;n.exchange_favourable_at_=exchange_favourable_at_;
        }else n.mission_at_=w.now_ms;
        n.initialized_=true;n.mission_=s.mission;
        auto&r=n.state_;r.stamp=fail.stamp;r.status=OutdoorStatus::ready;r.reset=reset_state;r.lease_changed=false;
        r.previous=r.posture;r.transitioned=false;r.transition_source_ms=0;
        r.able=r.known=r.unseen=r.unresolved=0;r.leader={};r.base_set=r.base_pinned=Evidence::unknown;
        r.bound_in_progress=r.base_formation_progress=r.allow_bound=r.allow_assault=r.door_bound=false;r.handover=OutdoorHandover::none;
        r.handover_building=invalid_room;r.handover_portal=0;r.exchange_hold=false;r.defended_building=invalid_room;
        const FoeId old_focus=r.focus;r.focus_valid=false;r.focus={};r.focus_fact={};
        std::vector<const ActorSnapshot*> men;std::map<Key,const MemberKnowledge*> ours;
        const MemberKnowledge* leader=nullptr;const ActorSnapshot* lead_actor=nullptr;
        auto time_fact=[&](bool exists,TimeMs at,bool&has,TimeMs&stored){if(exists&&at<=w.now_ms&&(!has||at>stored)){has=true;stored=at;}};
        for(const auto&ak:s.members){
            if(stop())return cancelled_result();auto ai=actors.find(key(ak));
            const ActorSnapshot*a=ai==actors.end()?nullptr:ai->second;
            MemberState ms;ms.key=ak;ms.owner=a?a->ownership_generation:0;
            auto old=previous.find(key(ak));
            if(!reset_state&&old!=previous.end()){
                if(a&&old->second->owner==ms.owner){ms=*old->second;}
                else if(a&&old->second->owner!=ms.owner)r.lease_changed=true;
            }
            n.members_.push_back(ms);
            if(!a||!able(*a,w,config_))continue;
            ++r.able;men.push_back(a);
            if(!lead_actor){lead_actor=a;r.leader=ak;}
            auto ki=knowledge.find(key(ak));
            if(ki==knowledge.end()||ki->second->side!=s.side_id||ki->second->ownership_generation!=a->ownership_generation||!ki->second->alive||!ki->second->able)continue;
            const auto&m=*ki->second;ours.emplace(key(ak),&m);
            if(a==lead_actor)leader=&m;
            time_fact(m.has_seen,m.last_seen_ms,n.has_seen_,n.seen_at_);
            time_fact(m.has_hit,m.last_hit_ms,n.has_hit_,n.hit_at_);
            time_fact(m.has_heard,m.last_heard_ms,n.has_heard_,n.heard_at_);
            if(a->near_misses.count>a->near_misses.items.size()){fail.reason="invalid_near_miss_count";return fail;}
            for(std::size_t j=0;j<a->near_misses.count;++j){
                const auto&f=a->near_misses.items[j];time_fact(std::isfinite(f.miss_m)&&f.miss_m>=0&&finite(f.origin)&&finite(f.dir)&&f.at_ms<=a->observed_at_ms,f.at_ms,n.has_supp_,n.supp_at_);
            }
            time_fact(a->danger.kind==DangerKind::hit&&finite(a->danger.position)&&a->danger.at_ms<=a->observed_at_ms,a->danger.at_ms,n.has_supp_,n.supp_at_);
        }
        if(r.lease_changed){n.stalled_=n.defend_ok_=false;n.endpoint_serial_=0;r.endpoint_failures=0;n.withdrawal_goal_=false;n.withdrawal_serial_=0;}
        const bool had_info=n.has_info_;const TimeMs last_info=n.info_at_;
        const FoeKnowledge* focus=nullptr;const SourceFact* focus_source=nullptr;float closest=0;
        std::map<Key,TimeMs> relation_times;
        if(leader&&lead_actor){
            std::map<Key,TimeMs> prior_relations;
            if(!reset_state)for(const auto&old:relations_){
                if(stop())return cancelled_result();
                if(old.observer==lead_actor->key)prior_relations.emplace(key(old.target),old.nonhostile_at);
            }
            for(const auto&f:leader->foes){
                if(stop())return cancelled_result();if(!f.id.valid()||f.id.ghost())continue;
                TimeMs at=0;auto prior=prior_relations.find(key(f.id.actor));if(prior!=prior_relations.end())at=prior->second;
                auto capture_relation=[&](const Contact&contact){
                    if(contact.key==f.id.actor&&!contact.hostile&&contact.observed_at_ms<=lead_actor->observed_at_ms&&finite(contact.position))
                        at=std::max(at,contact.observed_at_ms);
                };
                capture_relation(lead_actor->enemy);
                for(std::size_t ci=0;ci<std::min<std::size_t>(lead_actor->visible_enemies.count,lead_actor->visible_enemies.items.size());++ci)
                    capture_relation(lead_actor->visible_enemies.items[ci]);
                if(at&&relation_times.emplace(key(f.id.actor),at).second)n.relations_.push_back({lead_actor->key,f.id.actor,at});
            }
        }
        // K1 deliberately retains historical hostile observations. A current
        // exact friendly relation or supplied own-side identity is stronger
        // attack-selection evidence; it does not delete/rewrite that history.
        auto current_friendly=[&](const FoeKnowledge&f,const SourceFact&sf){
            if(f.id.ghost())return false;
            if(roster.count(key(f.id.actor)))return true;
            auto relation=relation_times.find(key(f.id.actor));
            return relation!=relation_times.end()&&relation->second>=sf.at_ms;
        };
        if(leader){
            time_fact(leader->has_seen,leader->last_seen_ms,n.has_info_,n.info_at_);
            time_fact(leader->has_hit,leader->last_hit_ms,n.has_info_,n.info_at_);
            time_fact(leader->has_heard,leader->last_heard_ms,n.has_info_,n.info_at_);
            for(const auto&f:leader->foes){
                if(stop())return cancelled_result();const auto*sf=source(f,w.now_ms);
                if(!f.id.valid()||f.down||!sf||current_friendly(f,*sf))continue;
                time_fact(true,sf->at_ms,n.has_info_,n.info_at_);
                for(std::size_t source_index=0;source_index<f.sources.size();++source_index){
                    const auto& fact=f.sources[source_index];
                    if(!fact.valid||!finite(fact.position)||fact.at_ms>w.now_ms)continue;
                    if(source_index==0||source_index==2)time_fact(true,fact.at_ms,n.has_seen_,n.seen_at_);
                    else time_fact(true,fact.at_ms,n.has_heard_,n.heard_at_);
                }
                if(!reset_state)for(const auto&searched:searched_){if(stop())return cancelled_result();if(searched.foe==f.id){n.searched_.push_back(searched);break;}}
                const bool done=std::any_of(n.searched_.begin(),n.searched_.end(),[&](const Searched&x){return x.foe==f.id&&x.source_at>=sf->at_ms;});
                if(!done)++r.unresolved;
                const auto focus_age=r.posture==OutdoorPosture::search?config_.knowledge.search_forget_ms:config_.knowledge.assess_foe_ms;
                if(!fresh(sf->at_ms,w.now_ms,focus_age))continue;
                if(f.id.ghost()){++r.unseen;continue;}++r.known;
                float d=flat(lead_actor->position,sf->position);
                if(!focus||(f.id==old_focus)||(focus->id!=old_focus&&(d<closest||(d==closest&&foe_less(f.id,focus->id))))){focus=&f;focus_source=sf;closest=d;}
            }
        }
        if(focus){r.focus_valid=true;r.focus=focus->id;r.focus_fact=*focus_source;r.movement_focus=focus_source->position;}
        const bool new_info=n.has_info_&&fresh(n.info_at_,w.now_ms,config_.knowledge.assess_foe_ms)&&(!had_info||n.info_at_>last_info);
        if(old_focus!=r.focus){n.endpoint_serial_=0;r.endpoint_failures=0;n.stalled_=false;}
        const bool exec=same_stamp(e.stamp,r.stamp);
        std::map<Key,const OutdoorActorExecution*> proofs;
        if(exec)for(const auto&p:e.actors){
            if(stop())return cancelled_result();if(!roster.count(key(p.applied.actor))||!proofs.emplace(key(p.applied.actor),&p).second){fail.reason="invalid_execution_roster";return fail;}
        }
        auto mark_searched=[&](const FoeId&foe,TimeMs at){
            for(auto&old:n.searched_){if(stop())return false;if(old.foe==foe){old.source_at=std::max(old.source_at,at);return true;}}
            n.searched_.push_back({foe,at});return true;
        };
        std::size_t bases=0,pinned_bases=0,movers=0,withdrawers=0,withdraw_arrived=0;
        TimeMs base_progress_time=0;
        bool base_known=false,base_set=false,mover_line=false,all_search=true,elements_complete=!men.empty();
        for(std::size_t j=0;j<n.members_.size();++j){
            if(stop())return cancelled_result();auto&ms=n.members_[j];auto ai=actors.find(key(ms.key));
            if(ai==actors.end()||!able(*ai->second,w,config_))continue;const auto&a=*ai->second;
            auto pi=proofs.find(key(ms.key));if(pi==proofs.end()||!applied(pi->second->applied,a,w)){elements_complete=false;continue;}
            const auto&p=*pi->second;const auto&o=p.applied;const bool focus_match=r.focus_valid&&p.focus==r.focus&&p.focus_at_ms==r.focus_fact.at_ms;
            const bool arrived=distance_squared(a.position,o.goal_position)<=o.arrival_radius_m*o.arrival_radius_m;
            const bool target_matches=focus_match&&((o.look.kind==LookKind::enemy&&o.look.target==r.focus.actor)||
                (o.fire.at_valid&&finite(o.fire.at)&&distance_squared(o.fire.at,r.focus_fact.position)==0));
            if(a.last_applied.fired_since_previous&&target_matches){ms.fired=true;ms.fired_at=a.observed_at_ms;ms.fired_focus=r.focus;}
            const bool gun_ready=!a.reloading&&a.weapon.state!=WeaponState::reloading&&a.weapon.ammo_in_mag>0;
            if(p.element==OutdoorElement::base){
                ++bases;auto km=ours.find(key(a.key));const bool pinned=!gun_ready||(km!=ours.end()&&km->second->meter_level>=3);
                if(pinned)++pinned_bases;
                if(p.cover_verified!=Evidence::unknown)base_known=true;
                const bool protection=p.cover_verified==Evidence::yes&&p.cover!=OutdoorCover::unknown&&p.cover!=OutdoorCover::open&&(p.cover!=OutdoorCover::full||o.expose.lean);
                // A verified, accepted base leg that is visibly making ground
                // must finish before attack is labelled stalled. Goal refreshes
                // do not restart this clock; only measured travel does.
                if(o.kind==OrderKind::cover&&o.role==roles::base_of_fire&&!arrived&&!pinned&&focus_match&&protection){
                    const float remaining=flat(a.position,o.goal_position);
                    if(ms.base_node!=o.goal_node||distance_squared(ms.base_goal,o.goal_position)!=0){
                        ms.base_node=o.goal_node;ms.base_goal=o.goal_position;ms.base_remaining=remaining;
                        // A new goal is intention, not observed progress.
                        ms.base_progress_at=0;
                    }else if(ms.base_remaining-remaining>=policy_.stall_gain_m){
                        ms.base_remaining=remaining;ms.base_progress_at=w.now_ms;
                    }
                    if(ms.base_progress_at&&w.now_ms-ms.base_progress_at<policy_.attack_stall_ms){
                        r.base_formation_progress=true;base_progress_time=(std::max)(base_progress_time,ms.base_progress_at);
                    }
                }else{ms.base_node=invalid_node;ms.base_progress_at=0;}
                // A settled base may cover the next move between bursts. A live,
                // applied point-fire command is readiness, not an invented shot.
                // Its source, cover and clear line are still verified below.
                const bool point_support=o.fire.at_valid&&finite(o.fire.at)&&distance_squared(o.fire.at,r.focus_fact.position)==0;
                const bool firing=target_matches&&o.fire.allowed&&o.fire.until_ms>w.now_ms&&!(a.flags&flag_dont_shoot)&&
                    (point_support||(ms.fired&&ms.fired_focus==r.focus&&fresh(ms.fired_at,w.now_ms,policy_.fired_recent_ms)));
                // Arrived field cover is refreshed as HOLD by SquadTactics.
                // The verified occupied cover and current fire, not the moving
                // verb, certify this base. A bare hold still proves nothing.
                if((o.kind==OrderKind::cover||o.kind==OrderKind::hold)&&!valid_key(o.aid)&&focus_match&&!pinned&&arrived&&protection&&p.fire_line==Evidence::yes&&firing)base_set=true;
            }
            if(p.element==OutdoorElement::mover){
                ++movers;
                if(focus_match&&flat(a.position,r.focus_fact.position)<=policy_.assault_m&&(visible(a,r.focus)||p.line_to_focus==Evidence::yes)&&p.movement_route==Evidence::yes)mover_line=true;
                if(o.kind==OrderKind::bound&&!arrived&&p.movement_route==Evidence::yes&&focus_match)r.bound_in_progress=true;
                if(!focus_match||!arrived||p.searched!=Evidence::yes||flat(a.position,r.focus_fact.position)>2*policy_.search_reach_m||
                    (focus&&focus->visible)||visible(a,r.focus))all_search=false;
            }
            if(r.posture==OutdoorPosture::search&&leader&&o.kind==OrderKind::search&&p.movement_route==Evidence::yes&&p.searched==Evidence::yes&&arrived){
                for(const auto&f:leader->foes){
                    if(stop())return cancelled_result();const auto*sf=source(f,w.now_ms);
                    if(f.id!=p.focus||f.down||f.visible||visible(a,f.id)||!sf||sf->at_ms!=p.focus_at_ms||flat(a.position,sf->position)>2*policy_.search_reach_m)continue;
                    if(!mark_searched(f.id,sf->at_ms))return cancelled_result();
                }
            }
            if(o.kind==OrderKind::withdraw&&n.withdrawal_goal_&&p.movement_route==Evidence::yes){
                ++withdrawers;if(arrived&&distance_squared(o.goal_position,n.withdrawal_point_)<=o.arrival_radius_m*o.arrival_radius_m)++withdraw_arrived;
            }
        }
        // Count unresolved locations after accepted search coverage. No enemy
        // lookup or synthetic dead fact is used, and a newer source reopens it.
        r.unresolved=0;
        if(leader)for(const auto&f:leader->foes){
            if(stop())return cancelled_result();const auto*sf=source(f,w.now_ms);
            if(!f.id.valid()||f.down||!sf||current_friendly(f,*sf))continue;bool searched=false;
            for(const auto&old:n.searched_){if(stop())return cancelled_result();if(old.foe==f.id&&old.source_at>=sf->at_ms){searched=true;break;}}
            if(!searched)++r.unresolved;
        }
        r.base_pinned=bases?(pinned_bases==bases?Evidence::yes:Evidence::no):Evidence::unknown;
        r.base_set=base_set?Evidence::yes:(base_known?Evidence::no:Evidence::unknown);
        const bool endpoint_match=exec&&r.focus_valid&&e.endpoint_focus==r.focus&&e.endpoint_focus_at_ms==r.focus_fact.at_ms;
        auto endpoint_member=actors.find(key(e.endpoint_actor));
        const bool endpoint_identity=roster.count(key(e.endpoint_actor))&&endpoint_member!=actors.end()&&
            able(*endpoint_member->second,w,config_)&&e.endpoint_owner==endpoint_member->second->ownership_generation&&
            e.endpoint_restrictions==endpoint_member->second->restrictions_revision;
        const bool endpoint_feasible=endpoint_match&&endpoint_identity&&e.endpoint_serial&&e.endpoint==OutdoorEndpoint::feasible&&
            e.endpoint_node!=invalid_node&&finite(e.endpoint_position)&&e.endpoint_route==Evidence::yes&&e.endpoint_cover==Evidence::yes;
        if(endpoint_match&&endpoint_identity&&e.endpoint_serial>n.endpoint_serial_&&(e.endpoint==OutdoorEndpoint::failed||endpoint_feasible)){
            n.endpoint_serial_=e.endpoint_serial;
            if(e.endpoint==OutdoorEndpoint::failed){if(r.endpoint_failures<UINT32_MAX)++r.endpoint_failures;}
            else if(endpoint_feasible)r.endpoint_failures=0;
        }
        const auto& failure=e.support_failure;const auto failed_actor=actors.find(key(failure.actor));
        // A newer observation of the same foe does not erase learning. A
        // materially displaced source, changed context/lease or expired original
        // failure does: the producer must then evaluate the new geometry.
        const bool support_failed=exec&&failure.completed&&r.focus_valid&&failure.focus==r.focus&&
            failure.focus_at_ms<=r.focus_fact.at_ms&&finite(failure.focus_position)&&
            distance_squared(failure.focus_position,r.focus_fact.position)<=policy_.arrived_m*policy_.arrived_m&&
            failure.stamp.level_epoch==w.level_epoch&&failure.stamp.map_hash==w.map_hash&&failure.stamp.geometry_revision==w.geometry_revision&&
            failure.stamp.group==s.group&&failure.stamp.side==s.side_id&&failure.stamp.faction==s.faction&&failure.stamp.mission_revision==s.mission.revision&&
            failure.stamp.snapshot_seq<w.sequence&&fresh(failure.stamp.at_ms,w.now_ms,config_.order_lifetime_ms)&&
            roster.count(key(failure.actor))&&failed_actor!=actors.end()&&able(*failed_actor->second,w,config_)&&
            failed_actor->second->ownership_generation==failure.owner&&failed_actor->second->restrictions_revision==failure.restrictions;
        const FactionStyle*style=&config_.faction_default;
        r.style_matched=false;
        for(std::size_t j=0;j<config_.factions.count;++j)if(config_.factions.items[j].faction==s.faction){style=&config_.factions.items[j];r.style_matched=true;break;}
        r.resolved_style=*style;
        const bool drilled=style->doctrine==1;
        auto transition=[&](OutdoorPosture p,const char*why,TimeMs source_ms){
            if(p==r.posture)return;
            r.previous=r.posture;r.posture=p;r.reason=why;r.transitioned=true;r.phase_at_ms=w.now_ms;r.transition_source_ms=source_ms;
            n.stalled_=n.defend_ok_=false;r.endpoint_failures=0;
        };
        const bool cooling=exec&&e.assessment_managed?e.assessment_cooldown_remaining_ms!=0:
            (n.withdrew_&&w.now_ms-n.withdraw_at_<policy_.assess_cooldown_ms);
        const bool assessment=exec&&e.assessment_serial>n.assessment_serial_;
        if(assessment)n.assessment_serial_=e.assessment_serial;
        const bool current_exchange=exec&&e.assessment_managed&&e.assessment_serial&&e.assessment_exchange_hold&&
            e.assessment_exchange_at_ms&&e.assessment_exchange_at_ms<=w.now_ms&&e.assessment_exchange_until_ms>w.now_ms&&
            std::isfinite(e.assessment_advance_cost)&&std::isfinite(e.assessment_hold_cost)&&
            e.assessment_advance_cost>e.assessment_hold_cost&&e.assessment_hold_cost>=0;
        if(current_exchange){
            n.exchange_latched_=true;n.exchange_favourable_=false;n.exchange_favourable_at_=0;
            if(e.assessment_exchange_at_ms>=n.exchange_at_){
                n.exchange_at_=e.assessment_exchange_at_ms;n.exchange_until_=e.assessment_exchange_until_ms;
            }
        }else if(n.exchange_latched_){
            // Absence, an expired receipt or a failed preview is not a good
            // exchange. Leave only after continuously favourable assessments;
            // the original hit/casualty window belongs to Assessment and is
            // never renewed by a posture pass or an empty transient hit ring.
            const bool favourable=exec&&e.assessment_managed&&e.assessment_serial&&
                e.assessment==OutdoorAssessment::proceed&&!e.assessment_exchange_hold&&
                std::isfinite(e.assessment_advance_cost)&&std::isfinite(e.assessment_hold_cost)&&
                e.assessment_advance_cost>=0&&e.assessment_hold_cost>=e.assessment_advance_cost;
            if(favourable){
                if(!n.exchange_favourable_){n.exchange_favourable_=true;n.exchange_favourable_at_=w.now_ms;}
                if(w.now_ms-n.exchange_favourable_at_>=config_.posture_dwell_ms&&
                    w.now_ms-r.phase_at_ms>=config_.posture_dwell_ms){
                    n.exchange_latched_=false;n.exchange_favourable_=false;n.exchange_favourable_at_=0;
                }
            }else{n.exchange_favourable_=false;n.exchange_favourable_at_=0;}
        }
        const bool current_hold=(assessment&&e.assessment==OutdoorAssessment::hold)||
            (exec&&e.assessment_managed&&e.assessment_serial&&(e.assessment==OutdoorAssessment::hold||current_exchange))||n.exchange_latched_;
        // Equal strength has no numerical penalty for assembling a bound.
        // Positive own casualties/hits are assessed above; an actually pinned
        // base without a certified covered destination makes waiting cheaper.
        // This prices posture intent only: firing-base, endpoint, route and
        // teammate gates still authorize each individual bound separately.
        const auto opposition=r.known+r.unseen;
        const float numerical_cost=opposition>r.able?float(opposition-r.able):0.f;
        const float pinned_cost=r.base_pinned==Evidence::yes&&!endpoint_feasible?float((std::max)(std::size_t(1),r.able)):0.f;
        const float objective_credit=s.mission.kind==MissionKind::attack?numerical_cost:0.f;
        const bool advance_affordable=numerical_cost+pinned_cost<=objective_credit;
        const bool clearing_mission=s.mission.kind==MissionKind::clear_building&&s.mission.building!=invalid_room;
        const bool owns_building=i.own_place.located&&i.own_place.building!=invalid_room&&i.pending==MissionKind::none&&
            !(clearing_mission&&i.own_place.building==s.mission.building);
        const auto activity=std::max({n.info_at_,n.seen_at_,n.hit_at_,n.supp_at_,n.heard_at_});
        const bool unconsumed_activity=(n.has_info_||n.has_seen_||n.has_hit_||n.has_supp_||n.has_heard_)&&
            (!n.cleared_activity_||activity>n.cleared_activity_ms_);
        const bool clear_gate=!clearing_mission&&unconsumed_activity&&(!n.has_seen_||w.now_ms-n.seen_at_>=policy_.clear_seen_ms)&&
            (!n.has_hit_||w.now_ms-n.hit_at_>=policy_.clear_hit_ms)&&(!n.has_supp_||w.now_ms-n.supp_at_>=policy_.clear_hit_ms)&&
            (!n.has_heard_||w.now_ms-n.heard_at_>=policy_.clear_heard_ms);
        if(exec&&e.assessment_managed&&e.assessment_serial&&e.assessment_serial==n.assessment_serial_&&r.posture==OutdoorPosture::withdraw&&
            n.withdrawal_serial_&&e.assessment_withdrawal_episode_serial==n.withdrawal_serial_&&
            (e.assessment_completed||e.assessment_capped)){
            transition(OutdoorPosture::defend,e.assessment_completed?"withdraw_complete":"withdraw_cap",w.now_ms);
            n.withdrawal_goal_=false;n.withdrawal_serial_=0;
        }
        else if(owns_building&&r.posture==OutdoorPosture::quiet&&!new_info&&!assessment&&
            (reset_state||w.now_ms-r.phase_at_ms>=config_.posture_dwell_ms)){transition(OutdoorPosture::defend,"own_building",w.now_ms);}
        else if(assessment&&e.assessment==OutdoorAssessment::withdraw&&r.posture!=OutdoorPosture::withdraw&&r.able>=2&&
            style->morale!=2&&!cooling&&e.safe_fallback==Evidence::yes&&finite(e.fallback)&&
            (!e.assessment_managed||(e.assessment_withdrawal_episode_serial&&e.assessment_withdrawal_episode_serial==e.assessment_serial))&&
            (style->leave_wounded||e.casualty_obligation_clear==Evidence::yes)){
            transition(OutdoorPosture::withdraw,"assessment",e.stamp.at_ms);
            if(!e.assessment_managed){n.withdrew_=true;n.withdraw_at_=w.now_ms;}
            n.withdrawal_goal_=true;n.withdrawal_point_=e.fallback;
            n.withdrawal_serial_=e.assessment_managed?e.assessment_withdrawal_episode_serial:0;
        }else if(current_hold&&r.posture!=OutdoorPosture::withdraw){
            transition(OutdoorPosture::defend,n.exchange_latched_?"exchange_hold":"assessment_hold",
                n.exchange_latched_?n.exchange_at_:e.stamp.at_ms);
            r.reason=n.exchange_latched_?"exchange_hold":"assessment_hold";n.defend_ok_=false;
        }
        else if(clearing_mission&&(r.posture==OutdoorPosture::quiet||r.posture==OutdoorPosture::clear||r.posture==OutdoorPosture::search)){
            transition(OutdoorPosture::attack,"mission_clear",w.now_ms);
        }
        else if(!reset_state&&w.now_ms-r.phase_at_ms<config_.posture_dwell_ms&&
            !(new_info&&(r.posture==OutdoorPosture::quiet||r.posture==OutdoorPosture::clear||r.posture==OutdoorPosture::search))){
            // Retain the existing commitment; observations and native orders
            // continue updating above. This is not a stop-movement timer.
        }
        else if(r.posture==OutdoorPosture::quiet||r.posture==OutdoorPosture::clear||r.posture==OutdoorPosture::search){
            if(new_info)transition(OutdoorPosture::contact,"first_knowledge",n.info_at_);
            else if(r.posture==OutdoorPosture::clear)transition(r.unresolved?OutdoorPosture::search:OutdoorPosture::quiet,r.unresolved?"unresolved_lk":"mission_resume",w.now_ms);
            else if(r.posture==OutdoorPosture::search&&!r.unresolved)transition(OutdoorPosture::quiet,"search_done",w.now_ms);
        }else if(r.posture!=OutdoorPosture::contact&&combat(r.posture)&&clear_gate&&
            !(r.posture==OutdoorPosture::withdraw&&exec&&e.assessment_managed)){
            transition(OutdoorPosture::clear,"clear_gate",std::max({n.seen_at_,n.hit_at_,n.supp_at_,n.heard_at_}));
            n.cleared_activity_=true;n.cleared_activity_ms_=activity;
        }
        else if(r.posture==OutdoorPosture::contact){
            if(!r.focus_valid)transition(OutdoorPosture::defend,"no_known_foe",n.info_at_);
            else if(r.able<2)transition(OutdoorPosture::defend,"solo",w.now_ms);
            else if(!drilled&&(s.mission.kind==MissionKind::attack||r.known<=r.able))transition(OutdoorPosture::attack,"loose_cover_advance",r.focus_fact.at_ms);
            else if(closest<=policy_.near_m&&r.known<=r.able)transition(drilled?OutdoorPosture::assault:OutdoorPosture::defend,drilled?"near_contact":"near_contact_loose",r.focus_fact.at_ms);
            else transition(drilled&&advance_affordable?OutdoorPosture::attack:OutdoorPosture::defend,
                drilled&&advance_affordable?"far_contact_advance":pinned_cost>0?"base_pinned":"far_contact",r.focus_fact.at_ms);
        }else if(r.posture==OutdoorPosture::withdraw){
            if(!e.assessment_managed&&withdrawers==r.able&&withdrawers&&withdraw_arrived==withdrawers)transition(OutdoorPosture::defend,"withdraw_complete",w.now_ms);
        }else if(r.posture==OutdoorPosture::attack){
            if(r.able<2&&!clearing_mission)transition(OutdoorPosture::defend,"solo",w.now_ms);
            else if(!r.focus_valid&&!clearing_mission)transition(OutdoorPosture::defend,"no_focus",w.now_ms);
            else if(drilled&&mover_line)transition(OutdoorPosture::assault,"movers_close",w.now_ms);
            else if(r.endpoint_failures>=policy_.endpoint_fail_max)transition(OutdoorPosture::defend,"no_endpoint",e.stamp.at_ms);
            else if(drilled&&!clearing_mission&&!r.bound_in_progress&&(r.base_pinned==Evidence::yes||r.base_set!=Evidence::yes)){
                if(r.base_formation_progress){n.stalled_=true;n.stalled_at_=base_progress_time;}
                else if(!n.stalled_){n.stalled_=true;n.stalled_at_=w.now_ms;}
                if(w.now_ms-n.stalled_at_>=policy_.attack_stall_ms)transition(OutdoorPosture::defend,r.base_pinned==Evidence::yes?"base_pinned":"base_not_set",w.now_ms);
            }else n.stalled_=false;
        }else if(r.posture==OutdoorPosture::assault){
            if(r.able<2)transition(OutdoorPosture::defend,"solo",w.now_ms);
            else if(!r.focus_valid)transition(OutdoorPosture::defend,"focus_resolved",w.now_ms);
            else if(elements_complete&&movers&&all_search&&w.now_ms-r.focus_fact.at_ms>=policy_.lk_empty_ms){
                if(!mark_searched(r.focus,r.focus_fact.at_ms))return cancelled_result();
                if(r.unresolved)--r.unresolved;transition(OutdoorPosture::defend,"lk_empty",r.focus_fact.at_ms);
            }
        }else if(r.posture==OutdoorPosture::defend){
            const bool retry=(std::string(r.reason)=="base_pinned"&&r.base_pinned==Evidence::no)||
                ((std::string(r.reason)=="base_not_set"||std::string(r.reason)=="no_endpoint")&&w.now_ms-r.phase_at_ms>=policy_.defend_attack_ms);
            if(!drilled&&r.able>=2&&r.focus_valid&&r.known<=r.able&&!cooling)transition(OutdoorPosture::attack,"loose_cover_advance",w.now_ms);
            else if(retry&&drilled&&r.able>=2&&r.focus_valid&&!cooling)transition(OutdoorPosture::attack,"retry",w.now_ms);
            else if(drilled&&r.able>=2&&r.focus_valid&&advance_affordable&&!cooling&&
                std::string(r.reason)!="base_pinned"&&std::string(r.reason)!="base_not_set"&&std::string(r.reason)!="no_endpoint"){
                transition(OutdoorPosture::attack,"far_contact_advance",r.focus_fact.at_ms);
            }
            else if(drilled&&r.able>=2&&r.focus_valid&&r.known<=r.able/2){
                if(!n.defend_ok_){n.defend_ok_=true;n.defend_ok_at_=w.now_ms;}
                if(w.now_ms-n.defend_ok_at_>=policy_.defend_attack_ms&&!cooling)transition(OutdoorPosture::attack,"outnumber_them",w.now_ms);
            }else n.defend_ok_=false;
        }
        r.exchange_hold=n.exchange_latched_&&r.posture==OutdoorPosture::defend;
        r.exchange_source_at_ms=n.exchange_at_;r.exchange_source_until_ms=n.exchange_until_;
        r.exchange_favourable_since_ms=n.exchange_favourable_?n.exchange_favourable_at_:0;
        if(r.exchange_hold&&i.own_place.located&&i.own_place.building!=invalid_room&&i.threat_valid&&
            !i.unidentified_location&&i.threat_place.projected&&i.threat_place.node!=invalid_node&&
            (!i.threat_place.located||i.threat_place.building!=i.own_place.building))
            r.defended_building=i.own_place.building;
        r.allow_bound=r.posture==OutdoorPosture::attack&&r.focus_valid&&r.base_set==Evidence::yes&&
            r.base_pinned==Evidence::no&&endpoint_feasible;
        r.allow_assault=r.posture==OutdoorPosture::assault&&r.focus_valid&&mover_line;
        r.support_recovery=drilled&&support_failed&&r.able>=2&&r.focus_valid&&r.base_set!=Evidence::yes&&
            !r.bound_in_progress&&!r.base_formation_progress&&!cooling&&!current_hold&&
            (r.posture==OutdoorPosture::attack||r.posture==OutdoorPosture::defend)&&!clearing_mission;
        r.loose_advance=!drilled&&r.posture==OutdoorPosture::attack&&r.focus_valid&&r.able>=2&&!cooling;
        const bool pending=i.pending==MissionKind::clear_building&&i.pending_building!=invalid_room;
        const bool attacking=r.posture==OutdoorPosture::attack||r.posture==OutdoorPosture::assault;
        const bool door=pending&&exec&&e.door_validated==Evidence::yes&&e.door_building==i.pending_building&&
            e.door_portal&&e.door_node!=invalid_node&&finite(e.door_point);
        if(pending&&!r.exchange_hold){
            for(const auto*a:men){if(stop())return cancelled_result();if(a->building_id==i.pending_building&&a->room_id!=invalid_room){r.handover=OutdoorHandover::inside;break;}}
        }
        if(door&&attacking&&!men.empty()){
            Vec3 centre;for(const auto*a:men){if(stop())return cancelled_result();const float count=static_cast<float>(men.size());centre.x+=a->position.x/count;centre.y+=a->position.y/count;centre.z+=a->position.z/count;}
            const float distance=flat(centre,e.door_point);
            if(!n.door_progress_||n.door_portal_!=e.door_portal||distance_squared(n.door_point_,e.door_point)>0||n.door_best_m_-distance>=policy_.stall_gain_m){
                n.door_progress_=true;n.door_best_m_=distance;n.door_progress_at_=w.now_ms;n.door_point_=e.door_point;n.door_portal_=e.door_portal;
            }
            if(r.focus_valid&&i.threat_valid&&i.threat.foe==r.focus&&i.threat.receiver==r.leader&&
                i.threat.observed_at_ms==r.focus_fact.at_ms&&finite(i.threat.position)&&distance_squared(i.threat.position,r.focus_fact.position)==0&&
                i.threat_place.located&&i.threat_place.building==i.pending_building){r.door_bound=true;r.movement_focus=e.door_point;}
            if(r.handover==OutdoorHandover::none){
                if(distance<=policy_.arrived_m)r.handover=OutdoorHandover::arrived;
                else if(std::any_of(men.begin(),men.end(),[&](const ActorSnapshot*a){return flat(a->position,e.door_point)<=policy_.door_m;}))r.handover=OutdoorHandover::at_door;
                else if(lead_actor&&flat(lead_actor->position,e.door_point)<=policy_.fight_m&&r.door_bound)r.handover=OutdoorHandover::building_fight;
                else if(r.able==1&&distance<=policy_.fight_m)r.handover=OutdoorHandover::solo;
                else if(distance<=policy_.stall_near_m&&w.now_ms-n.door_progress_at_>=policy_.handover_stall_ms)r.handover=OutdoorHandover::stalled;
                else if(w.now_ms-n.mission_at_>=policy_.handover_cap_ms)r.handover=OutdoorHandover::time;
            }
        }else n.door_progress_=false;
        if(r.handover!=OutdoorHandover::none){r.handover_building=i.pending_building;r.handover_portal=door?e.door_portal:0;}
        if(stop())return cancelled_result();*this=std::move(n);return state_;
    }catch(...){fail.status=OutdoorStatus::failed;fail.reason="exception";return fail;}
}
}}}
