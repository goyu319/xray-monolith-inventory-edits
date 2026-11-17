#include "CombatOutdoorExecution.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>

namespace painter { namespace combat { namespace core {
namespace {
using Key=std::pair<std::uint32_t,std::uint64_t>;
Key key(ActorKey k){return {k.id,k.generation};}
bool stamp_matches(const OutdoorStamp& st,const WorldSnapshot&w,const SideSnapshot&s){
    return st.level_epoch==w.level_epoch&&st.map_hash==w.map_hash&&st.geometry_revision==w.geometry_revision&&
        st.snapshot_seq==w.sequence&&st.at_ms==w.now_ms&&st.side==s.side_id&&st.group==s.group&&
        st.faction==s.faction&&st.mission_revision==s.mission.revision;
}
bool applied(const Order&o,const ActorSnapshot&a,const WorldSnapshot&w,const Config&c){
    return a.alive&&!a.wounded&&!a.yield_reasons&&(a.flags&flag_admitted)&&a.current_position_accessible&&a.ownership_generation&&
        a.level_epoch==w.level_epoch&&a.snapshot_seq==w.sequence&&fresh(a.observed_at_ms,w.now_ms,c.snapshot_max_age_ms)&&
        o.actor==a.key&&o.order_seq&&a.last_applied.order_seq==o.order_seq&&a.accessible_current_order&&
        !(a.last_rejected.order_seq==o.order_seq&&a.last_rejected.reason!=reject_none)&&o.kind!=OrderKind::yield&&
        o.level_epoch==w.level_epoch&&o.geometry_revision==w.geometry_revision&&o.snapshot_seq&&o.snapshot_seq<=w.sequence&&
        o.ownership_generation==a.ownership_generation&&o.restrictions_revision==a.restrictions_revision&&
        o.issued_at_ms<=a.last_applied.at_ms&&a.last_applied.at_ms<=a.observed_at_ms&&o.expires_at_ms>w.now_ms&&
        finite(a.position)&&finite(o.goal_position)&&o.goal_node!=invalid_node&&
        std::isfinite(o.arrival_radius_m)&&o.arrival_radius_m>0;
}
bool query_matches(const OutdoorQuerySource&q,const ActorSnapshot&a,const WorldSnapshot&w,const OutdoorPostureResult&f){
    return q.actor==a.key&&q.level_epoch==w.level_epoch&&q.snapshot_seq==w.sequence&&q.geometry_revision==w.geometry_revision&&
        q.ownership_generation==a.ownership_generation&&q.restrictions_revision==a.restrictions_revision&&
        q.at_ms==w.now_ms&&q.origin_node==a.node_id&&finite(q.actor_position)&&distance_squared(q.actor_position,a.position)==0&&
        f.focus_valid&&q.focus==f.focus&&q.focus_at_ms==f.focus_fact.at_ms&&q.focus_at_ms<=a.observed_at_ms&&
        finite(q.focus_position)&&distance_squared(q.focus_position,f.focus_fact.position)==0;
}
bool own_visible(const ActorSnapshot&a,const OutdoorPostureResult&f){
    auto matches=[&](const Contact&c){return c.key==f.focus.actor&&c.hostile&&c.alive_when_observed&&personally_visible(c,a)&&
        finite(c.position)&&distance_squared(c.position,f.focus_fact.position)==0;};
    if(matches(a.enemy))return true;
    for(std::size_t i=0;i<std::min<std::size_t>(a.visible_enemies.count,a.visible_enemies.items.size());++i)if(matches(a.visible_enemies.items[i]))return true;
    return false;
}
}
bool observe_outdoor_route(const Config&c,const WorldSnapshot&w,const ActorSnapshot&a,
    const RouteProofReceipt&r,OutdoorActorExecution&proof){
    if(!applied(proof.applied,a,w,c))return false;
    const auto request=route_proof_source(w,a,proof.applied.goal_node,proof.applied.goal_position);
    if(!route_proof_complete(r,request))return false;
    proof.movement_route=Evidence::yes;return true;
}
struct OutdoorExecutionIndex::Data {
    const WorldSnapshot* world=nullptr;std::uint64_t epoch=0,sequence=0,map=0,geometry=0;TimeMs at=0;
    std::map<Key,const ActorSnapshot*> actors;std::map<Key,const Order*> accepted;
    std::map<Key,const ActorCoverCache*> cover;std::map<Key,const OutdoorCoverProfile*> profiles;
    std::map<Key,const OutdoorMeasuredLine*> lines;
    std::map<Key,const OutdoorSearchObservation*> search_observations;
};
OutdoorExecutionIndex index_outdoor_execution(const WorldSnapshot&w,const std::vector<Order>&orders,
    const std::vector<ActorCoverCache>&covers,const OutdoorExecutionEvidence&e,const std::function<bool()>&cancelled){
    OutdoorExecutionIndex out;auto stop=[&](){return cancelled&&cancelled();};
    auto abort=[&](){OutdoorExecutionIndex n;n.status_=OutdoorExecutionStatus::cancelled;return n;};
    try{
        if(stop())return abort();if(!w.level_epoch||!w.sequence)return out;
        auto n=std::make_shared<OutdoorExecutionIndex::Data>();n->world=&w;n->epoch=w.level_epoch;n->sequence=w.sequence;n->map=w.map_hash;n->geometry=w.geometry_revision;n->at=w.now_ms;
        std::set<std::uint32_t> ids;
        for(const auto&a:w.actors){if(stop())return abort();++out.costs_.actor_rows;
            if(!valid_key(a.key)||!ids.insert(a.key.id).second||!n->actors.emplace(key(a.key),&a).second)return out;}
        for(const auto&o:orders){if(stop())return abort();++out.costs_.order_rows;if(!valid_key(o.actor))return out;
            auto a=n->actors.find(key(o.actor));if(a==n->actors.end()||o.order_seq!=a->second->last_applied.order_seq)continue;
            if(!n->accepted.emplace(key(o.actor),&o).second)return out;
        }
        for(const auto&cache:covers){if(stop())return abort();++out.costs_.cache_rows;if(!valid_key(cache.actor)||!n->cover.emplace(key(cache.actor),&cache).second)return out;}
        for(const auto&p:e.profiles){if(stop())return abort();++out.costs_.profile_rows;if(!valid_key(p.source.actor)||!n->profiles.emplace(key(p.source.actor),&p).second)return out;}
        for(const auto&l:e.lines){if(stop())return abort();++out.costs_.line_rows;if(!valid_key(l.source.actor)||!n->lines.emplace(key(l.source.actor),&l).second)return out;}
        for(const auto&observation:e.search_observations){if(stop())return abort();if(!valid_key(observation.source.actor)||!n->search_observations.emplace(key(observation.source.actor),&observation).second)return out;}
        if(stop())return abort();out.data_=std::move(n);out.status_=OutdoorExecutionStatus::ready;return out;
    }catch(...){OutdoorExecutionIndex n;n.status_=OutdoorExecutionStatus::failed;return n;}
}
OutdoorExecutionResult outdoor_execution_facts(const Config&c,const OutdoorExecutionIndex&index,
    const SideSnapshot&s,const SquadView&group,const OutdoorPostureResult&focus,const std::function<bool()>&cancelled){
    OutdoorExecutionResult out;auto stop=[&](){return cancelled&&cancelled();};
    auto abort=[&](){OutdoorExecutionResult n;n.status=OutdoorExecutionStatus::cancelled;return n;};
    try{
        if(stop())return abort();if(index.status_!=OutdoorExecutionStatus::ready||!index.data_){out.status=index.status_;return out;}
        const auto&data=*index.data_;const auto&w=*data.world;
        if(w.level_epoch!=data.epoch||w.sequence!=data.sequence||w.map_hash!=data.map||w.geometry_revision!=data.geometry||w.now_ms!=data.at)return out;
        if(!s.side_id||s.members.empty()||focus.status!=OutdoorStatus::ready||!stamp_matches(focus.stamp,w,s)||
            group.side_id!=s.side_id||group.group_id!=s.side_id||group.faction!=s.faction)return out;
        if(focus.focus_valid&&(!focus.focus.valid()||focus.focus.ghost()||!focus.focus_fact.valid||
            !finite(focus.focus_fact.position)||!fresh(focus.focus_fact.at_ms,w.now_ms,
                focus.posture==OutdoorPosture::search?c.knowledge.search_forget_ms:c.knowledge.assess_foe_ms)))return out;
        std::set<Key> roster,active,base,movers;
        for(auto member:s.members){if(stop())return abort();if(!valid_key(member)||!roster.insert(key(member)).second)return out;}
        for(auto member:group.members){if(stop())return abort();if(!roster.count(key(member))||!active.insert(key(member)).second)return out;}
        for(auto member:group.base){if(stop())return abort();if(!active.count(key(member))||!base.insert(key(member)).second)return out;}
        for(auto member:group.movers){if(stop())return abort();if(!active.count(key(member))||base.count(key(member))||!movers.insert(key(member)).second)return out;}
        out.facts.stamp=focus.stamp;
        for(auto member:group.members){
            if(stop())return abort();++out.costs.members;auto ai=data.actors.find(key(member));auto oi=data.accepted.find(key(member));
            if(ai==data.actors.end()||oi==data.accepted.end()){++out.unconfirmed;continue;}
            const auto&a=*ai->second;const auto&o=*oi->second;
            if(a.faction!=s.faction||!applied(o,a,w,c)){++out.unconfirmed;continue;}
            OutdoorActorExecution proof;proof.applied=o;
            proof.element=base.count(key(member))?OutdoorElement::base:(movers.count(key(member))?OutdoorElement::mover:OutdoorElement::none);
            if(focus.focus_valid){proof.focus=focus.focus;proof.focus_at_ms=focus.focus_fact.at_ms;}
            auto ci=data.cover.find(key(member));auto pi=data.profiles.find(key(member));
            if(ci!=data.cover.end()&&pi!=data.profiles.end()&&focus.focus_valid){
                const auto&cache=*ci->second;const auto&profile=*pi->second;
                const bool source=query_matches(profile.source,a,w,focus)&&profile.heights==std::array<float,3>{{.7f,1.1f,1.7f}}&&
                    cache.level_epoch==w.level_epoch&&cache.snapshot_seq==w.sequence&&cache.ownership_generation==a.ownership_generation&&
                    cache.restrictions_revision==a.restrictions_revision&&cache.origin_node==a.node_id&&cache.observed_at_ms==w.now_ms&&
                    cache.knowledge_threat==focus.focus&&cache.knowledge_observed_at_ms==focus.focus_fact.at_ms&&
                    finite(cache.knowledge_threat_position)&&distance_squared(cache.knowledge_threat_position,focus.focus_fact.position)==0;
                if(source)for(const auto&spot:cache.spots){
                    if(stop())return abort();++out.costs.spots;
                    if(spot.node!=o.goal_node||!finite(spot.position)||distance_squared(spot.position,o.goal_position)!=0||
                        spot.geometry_revision!=w.geometry_revision||spot.threat!=0||spot.protection_known!=Evidence::yes||
                        spot.threat_key!=focus.focus.actor||spot.threat_observed_at_ms!=focus.focus_fact.at_ms)continue;
                    if(spot.accessible==Evidence::no||spot.reachable==Evidence::no||
                        (spot.restrictions_revision&&spot.restrictions_revision!=a.restrictions_revision))continue;
                    proof.cover_verified=Evidence::yes;
                    proof.cover=spot.blocked[2]?OutdoorCover::full:(spot.blocked[1]?OutdoorCover::chest:(spot.blocked[0]?OutdoorCover::low:OutdoorCover::open));break;
                }
            }
            // An ordered stance and a goal-height ray are not an actual body
            // pose or current eye ray. Only the separately measured receipt can
            // establish that line; route/endpoint/search remain unproven.
            auto li=data.lines.find(key(member));
            if(li!=data.lines.end()&&distance_squared(a.position,o.goal_position)<=o.arrival_radius_m*o.arrival_radius_m){
                const auto&line=*li->second;
                if(query_matches(line.source,a,w,focus)&&finite(line.measured_eye)&&
                    distance_squared(line.measured_eye,a.position)>0&&(line.clear==Evidence::yes||line.clear==Evidence::no)){
                    proof.fire_line=line.clear;
                    // Geometry to another man's LK is not this man's field of
                    // view. Only his own current sight can certify mover line.
                    if(own_visible(a,focus))proof.line_to_focus=line.clear;
                    // A current looked-at, visible location can retire its old
                    // search source. This is neither a dead-foe fact nor room clearance.
                    const auto& policy=c.outdoor.p.posture;
                    Vec3 point=focus.focus_fact.position;point.y+=c.outdoor.p.solo.combat.target_height_m;
                    const Vec3 delta{point.x-line.measured_eye.x,point.y-line.measured_eye.y,point.z-line.measured_eye.z};
                    const float head=std::sqrt(distance_squared(a.head_dir,Vec3{}));
                    const float sight=std::sqrt(distance_squared(delta,Vec3{}));
                    const float half=c.room.check_half_deg;
                    auto seen_target=[&](const Contact& contact){return contact.key==focus.focus.actor&&contact.hostile&&contact.alive_when_observed&&personally_visible(contact,a);};
                    bool target_visible=seen_target(a.enemy);
                    for(std::size_t n=0;n<a.visible_enemies.count;++n)target_visible=target_visible||seen_target(a.visible_enemies.items[n]);
                    const auto observed=data.search_observations.find(key(member));
                    const auto* observation=observed==data.search_observations.end()?nullptr:observed->second;
                    if(o.kind==OrderKind::search&&line.clear==Evidence::yes&&!target_visible&&observation&&
                        observation->clear==Evidence::yes&&query_matches(observation->source,a,w,focus)&&observation->applied_order_seq==o.order_seq&&
                        finite(observation->measured_eye)&&distance_squared(observation->measured_eye,line.measured_eye)==0&&
                        finite(observation->observed_point)&&distance_squared(observation->observed_point,point)==0&&
                        finite(observation->head)&&distance_squared(observation->head,a.head_dir)==0&&
                        std::isfinite(policy.search_reach_m)&&policy.search_reach_m>0&&
                        std::hypot(a.position.x-focus.focus_fact.position.x,a.position.z-focus.focus_fact.position.z)<=policy.search_reach_m&&
                        finite(a.head_dir)&&head>.5f&&std::isfinite(sight)&&sight>.001f&&
                        std::isfinite(half)&&half>0&&half<90&&
                        (a.head_dir.x*delta.x+a.head_dir.y*delta.y+a.head_dir.z*delta.z)/(head*sight)>=std::cos(half*3.14159265358979323846f/180.f))
                        proof.searched=Evidence::yes;
                }
            }
            if(proof.cover_verified==Evidence::unknown)++out.cover_unknown;
            out.facts.actors.push_back(std::move(proof));++out.applied;
        }
        if(stop())return abort();out.status=OutdoorExecutionStatus::ready;return out;
    }catch(...){OutdoorExecutionResult n;n.status=OutdoorExecutionStatus::failed;return n;}
}
OutdoorExecutionResult outdoor_execution_facts(const Config&c,const WorldSnapshot&w,const SideSnapshot&s,
    const SquadView&g,const OutdoorPostureResult&f,const std::vector<Order>&orders,const std::vector<ActorCoverCache>&cover,
    const std::function<bool()>&cancelled){
    const OutdoorExecutionEvidence evidence;const auto index=index_outdoor_execution(w,orders,cover,evidence,cancelled);
    auto out=outdoor_execution_facts(c,index,s,g,f,cancelled);const auto members=out.costs.members,spots=out.costs.spots;
    out.costs=index.costs();out.costs.members=members;out.costs.spots=spots;return out;
}
}}}
