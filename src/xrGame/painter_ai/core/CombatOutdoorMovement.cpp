#include "CombatOutdoorMovement.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <limits>
#include <string>

namespace painter { namespace combat { namespace core {
namespace {
using Key=std::pair<std::uint32_t,std::uint64_t>;
Key key(ActorKey a){return {a.id,a.generation};}
bool pos(float x){return std::isfinite(x)&&x>0;}
bool speed(float x){return std::isfinite(x)&&x>=.3f&&x<=1.5f;}
float flat(Vec3 a,Vec3 b){return std::hypot(a.x-b.x,a.z-b.z);}
bool same(Vec3 a,Vec3 b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
OutdoorStamp stamp(const WorldSnapshot&w,const SideSnapshot&s){return {w.level_epoch,w.map_hash,w.geometry_revision,w.sequence,s.group,w.now_ms,s.side_id,s.faction,s.mission.revision};}
bool match(const OutdoorStamp&a,const OutdoorStamp&b,bool frame=true){return a.level_epoch==b.level_epoch&&a.map_hash==b.map_hash&&a.geometry_revision==b.geometry_revision&&
    a.side==b.side&&a.faction==b.faction&&a.group==b.group&&a.mission_revision==b.mission_revision&&(!frame||(a.snapshot_seq==b.snapshot_seq&&a.at_ms==b.at_ms));}
bool policy_ok(const OutdoorMovementPolicy&p){return pos(p.project_horizontal_m)&&pos(p.project_vertical_m)&&pos(p.door_offset_m)&&
    pos(p.spread_m)&&pos(p.close_spread_m)&&p.close_spread_m<=p.spread_m&&pos(p.close_m)&&pos(p.step_m)&&pos(p.arrival_m)&&
    speed(p.aimed_speed)&&speed(p.run_speed)&&speed(p.sprint_speed)&&speed(p.search_speed)&&pos(p.sprint_m)&&p.path_nodes&&p.path_nodes<=65536;}
bool protected_order(const Order&o){return o.assignment.generation||o.portal.valid||o.throw_order.valid||valid_key(o.aid)||valid_key(o.finish)||
    (o.flags&order_committed_entry)||o.kind==OrderKind::bound||o.kind==OrderKind::withdraw||o.kind==OrderKind::recover||o.kind==OrderKind::reload||o.kind==OrderKind::yield;}
bool actual(const Order&o,const ActorSnapshot&a,const WorldSnapshot&w,const Config&config){auto check=o;if(check.fire.until_ms<=w.now_ms)check.fire={};
    return publishable(check,a,w.now_ms,w.level_epoch,w.geometry_revision)&&o.snapshot_seq<=w.sequence&&a.accessible_current_order&&
        o.order_seq==a.last_applied.order_seq&&a.last_applied.at_ms>=o.issued_at_ms&&a.last_applied.at_ms<=a.observed_at_ms&&
        fresh(a.last_applied.at_ms,w.now_ms,config.snapshot_max_age_ms)&&
        !(a.last_rejected.order_seq==o.order_seq&&a.last_rejected.reason!=reject_none);}
bool source(const SourceFact&s,const ActorSnapshot&a,const WorldSnapshot&w,const Config&c){return s.valid&&valid_key(s.reporter)&&finite(s.position)&&
    s.at_ms<=a.observed_at_ms&&fresh(s.at_ms,w.now_ms,c.knowledge.search_forget_ms);}
bool friendly(const ActorSnapshot&a,FoeId foe,TimeMs at){if(foe.ghost())return false;
    auto test=[&](const Contact&c){return c.key==foe.actor&&c.observed_at_ms>=at&&c.observed_at_ms<=a.observed_at_ms&&(!c.hostile||!c.alive_when_observed||c.wounded);};
    if(test(a.enemy))return true;for(std::size_t n=0;n<a.visible_enemies.count;++n)if(test(a.visible_enemies.items[n]))return true;return false;}
}
CombatOutdoorMovement::CombatOutdoorMovement(Config c,OutdoorMovementPolicy p):config_(std::move(c)),policy_(p){}
void CombatOutdoorMovement::reset(){stamp_={};cursor_stamp_={};after_={};issued_.clear();investigated_.clear();pending_sounds_.clear();}
OutdoorMovementResult CombatOutdoorMovement::update(const OutdoorMovementRequest&q,const EngineServices&services,
    OutdoorMovementQuota&quota,const std::function<bool()>&cancelled){
    OutdoorMovementResult out;const auto start_work=quota.work_used,start_queries=quota.queries_used;
    struct Cancelled{};struct Exhausted{};
    auto poll=[&](){if(cancelled&&cancelled())throw Cancelled{};};
    auto work=[&](){poll();if(!quota.work_left)throw Exhausted{};--quota.work_left;++quota.work_used;};
    auto query=[&](){poll();if(!quota.queries_left)throw Exhausted{};--quota.queries_left;++quota.queries_used;};
    auto measured=[&](){out.work=quota.work_used-start_work;out.queries=quota.queries_used-start_queries;return out;};
    std::vector<Issued> next;std::vector<Investigated> checked;std::vector<PendingSound> sounds;
    ActorKey advanced=after_;bool advanced_once=false,validated=false;
    try{
        poll();if(!q.world||!q.side||!q.knowledge||!q.mission||!q.posture||!q.elements||!q.baseline||!q.applied)return out;
        const auto&w=*q.world;const auto&s=*q.side;const auto&k=*q.knowledge;const auto&p=*q.posture;const auto&mission=*q.mission;
        out.stamp=stamp(w,s);
        if(!policy_.enabled){out.status=OutdoorStatus::ready;out.reason="disabled";return out;}
        if(!policy_ok(policy_)||!w.level_epoch||!w.sequence||!s.side_id||!match(p.stamp,out.stamp)||p.status!=OutdoorStatus::ready||
            !k.accepted||k.cancelled||k.level_epoch!=w.level_epoch||k.sequence!=w.sequence||q.elements->side_id!=s.side_id||q.elements->faction!=s.faction||
            mission.status!=MissionIntelStatus::ready||mission.side!=s.side_id||mission.faction!=s.faction||mission.group!=s.group||
            mission.level_epoch!=w.level_epoch||mission.map_hash!=w.map_hash||mission.geometry_revision!=w.geometry_revision||mission.snapshot_seq!=w.sequence||
            mission.requested.revision!=s.mission.revision||mission.requested.kind!=s.mission.kind||s.mission.rooms.count>s.mission.rooms.items.size())return out;
        if(stamp_.level_epoch&&(w.level_epoch<stamp_.level_epoch||(w.level_epoch==stamp_.level_epoch&&(w.sequence<=stamp_.snapshot_seq||w.now_ms<stamp_.at_ms)))){
            out.status=OutdoorStatus::stale;out.reason="stale_capture";return out;}
        const bool continuity=match(stamp_,out.stamp,false);
        std::map<Key,const ActorSnapshot*> actors;std::map<Key,const MemberKnowledge*> known;
        std::map<Key,const ProposedOrder*> bases;std::map<Key,const Order*> applied;std::set<Key> roster,movers;
        for(const auto&a:w.actors){work();if(!actors.emplace(key(a.key),&a).second)return measured();}
        for(const auto&m:k.members){work();if(!known.emplace(key(m.actor),&m).second)return measured();}
        for(const auto&b:*q.baseline){work();if(!bases.emplace(key(b.order.actor),&b).second)return measured();}
        for(const auto&o:*q.applied){work();if(!applied.emplace(key(o.actor),&o).second)return measured();}
        for(auto actor:s.members){work();if(!valid_key(actor)||!roster.insert(key(actor)).second)return measured();}
        for(auto actor:q.elements->movers){work();if(!roster.count(key(actor))||!movers.insert(key(actor)).second)return measured();}
        std::vector<const ActorSnapshot*> men;std::map<Key,std::size_t> slots;
        for(auto id:roster){work();auto ai=actors.find(id);auto ki=known.find(id);if(ai==actors.end()||ki==known.end())continue;
            const auto&a=*ai->second;const auto&m=*ki->second;
            if(a.faction!=s.faction||m.side!=s.side_id||m.ownership_generation!=a.ownership_generation)return measured();
            if(a.level_epoch!=w.level_epoch||a.snapshot_seq!=w.sequence||a.observed_at_ms!=w.now_ms||!a.alive||a.wounded||
                !(a.flags&flag_admitted)||a.yield_reasons||!a.ownership_generation||!finite(a.position)||!a.current_position_accessible||
                a.node_id==invalid_node||!std::isfinite(a.health)||a.health<=0||!m.alive||!m.able||a.visible_enemies.count>a.visible_enemies.items.size())continue;
            slots.emplace(id,men.size());men.push_back(&a);
        }
        // Preserve off-cohort requests with unchanged authority. They are only
        // execution evidence after the exact native applied row is observed.
        if(continuity)for(const auto&x:issued_){work();auto a=actors.find(key(x.actor));if(a==actors.end()||!roster.count(key(x.actor)))continue;
            if(x.owner==a->second->ownership_generation&&x.restrictions==a->second->restrictions_revision)next.push_back(x);}
        if(continuity)for(const auto&x:investigated_){work();auto a=actors.find(key(x.actor));
            if(a!=actors.end()&&roster.count(key(x.actor))&&x.owner==a->second->ownership_generation&&
                fresh(x.at,w.now_ms,config_.knowledge.contact_keep_ms))checked.push_back(x);}
        std::vector<const ActorSnapshot*> order;
        const ActorKey cursor=match(cursor_stamp_,out.stamp,false)?after_:ActorKey{};
        for(const auto*a:men){work();if(key(a->key)>key(cursor))order.push_back(a);}
        for(const auto*a:men){work();if(key(a->key)<=key(cursor))order.push_back(a);}
        std::vector<std::pair<ActorKey,Vec3>> occupied;
        for(const auto*a:men){work();occupied.push_back({a->key,a->position});auto old=applied.find(key(a->key));
            if(old!=applied.end()&&actual(*old->second,*a,w,config_)&&old->second->pace!=Pace::stand)
                occupied.push_back({a->key,old->second->goal_position});}
        const bool search=p.posture==OutdoorPosture::search;
        const bool assault=p.posture==OutdoorPosture::assault&&p.allow_assault;
        const bool approach=(p.posture==OutdoorPosture::quiet||p.posture==OutdoorPosture::clear||p.posture==OutdoorPosture::contact)&&
            (s.mission.kind==MissionKind::clear_building||mission.pending==MissionKind::clear_building);
        // Unidentified gunfire is an investigation location, never an enemy
        // identity or permission to shoot. Explicit objectives and adverse
        // exchange assessments keep ownership of their current jobs.
        const bool investigate=!p.focus_valid&&!p.unseen&&!p.exchange_hold&&std::string(p.reason)!="assessment_hold"&&s.mission.kind==MissionKind::none&&
            mission.pending==MissionKind::none&&w.now_ms>=p.phase_at_ms&&w.now_ms-p.phase_at_ms>=config_.posture_dwell_ms&&
            (p.posture==OutdoorPosture::contact||p.posture==OutdoorPosture::defend||p.posture==OutdoorPosture::quiet||p.posture==OutdoorPosture::clear);
        out.investigating=investigate;
        if(investigate&&match(cursor_stamp_,out.stamp,false))for(const auto&x:pending_sounds_){work();auto a=actors.find(key(x.actor));
            if(a!=actors.end()&&roster.count(key(x.actor))&&x.owner==a->second->ownership_generation&&
                x.restrictions==a->second->restrictions_revision&&fresh(x.sound.at_ms,w.now_ms,config_.knowledge.contact_keep_ms))sounds.push_back(x);}
        validated=true;
        for(std::size_t rank=0;rank<order.size();++rank){const auto&a=*order[rank];work();
            if(!advanced_once){advanced=a.key;advanced_once=true;}
            auto bi=bases.find(key(a.key));
            if(bi==bases.end()||protected_order(bi->second->order)||
                a.reloading||a.weapon.state==WeaponState::reloading||!a.weapon.ammo_in_mag)continue;
            if(!search&&!assault&&!approach&&!investigate)continue;
            if(assault&&!movers.count(key(a.key)))continue;
            const auto&m=*known.at(key(a.key));const FoeKnowledge*focus=nullptr;const SourceFact*fact=nullptr;
            Vec3 goal;std::uint32_t objective_portal=0;bool has_goal=false;const LooseContact*sound=nullptr;LooseContact retained_sound;
            if(investigate){
                const auto slot=slots.at(key(a.key));
                if((!movers.empty()&&!movers.count(key(a.key)))||
                    (movers.empty()&&slot>=(std::max)(std::size_t(1),men.size()/2))||m.meter_level>=2||
                    (m.has_hit&&fresh(m.last_hit_ms,w.now_ms,config_.outdoor.p.personal.personal_hit_ms))||
                    (m.fresh_fire.valid&&fresh(m.fresh_fire.at_ms,w.now_ms,config_.outdoor.p.personal.personal_hit_ms)))continue;
                auto seeing=[&](const Contact&c){return c.hostile&&c.alive_when_observed&&personally_visible(c,a);};
                bool sees=seeing(a.enemy);for(std::size_t n=0;n<a.visible_enemies.count;++n){work();sees=sees||seeing(a.visible_enemies.items[n]);}
                if(sees)continue;
                for(const auto&v:m.contacts){work();
                    if(v.kind!=SoundKind::gunshot||!finite(v.position)||!valid_key(v.reporter)||
                        (!v.from_report&&v.reporter!=a.key)||v.at_ms>a.observed_at_ms||
                        !fresh(v.at_ms,w.now_ms,config_.knowledge.contact_keep_ms))continue;
                    bool done=false;for(const auto&x:checked){work();if(x.actor==a.key&&x.at>=v.at_ms&&flat(x.point,v.position)<=policy_.arrival_m*2){done=true;break;}}
                    for(const auto&x:sounds){work();if(x.actor==a.key&&x.blocked&&x.sound.at_ms==v.at_ms&&same(x.sound.position,v.position))done=true;}
                    if(!done&&(!sound||v.at_ms>sound->at_ms))sound=&v;
                }
                // Latch a route request before it is a movement order. New
                // gunshots cannot move its destination while the worker is
                // still preparing the first route. This is planning intent,
                // never a fabricated native applied-order receipt.
                for(const auto&x:sounds)if(x.actor==a.key&&!x.blocked){retained_sound=x.sound;sound=&retained_sound;break;}
                // Keep a granted leg's original observation while that same
                // sourced episode remains fresh. Rapid gunshots must not cause
                // continuous destination changes or restart a settle timer.
                for(const auto&x:next)if(x.actor==a.key&&x.kind==OrderKind::advance&&!x.focus.valid()&&!x.objective_portal){
                    const auto current=applied.find(key(a.key));
                    if(current!=applied.end()&&actual(*current->second,a,w,config_)&&current->second->kind==x.kind&&
                        current->second->goal_node==x.node&&same(current->second->goal_position,x.point)&&
                        current->second->order_seq>x.previous&&valid_key(x.sound_reporter)&&
                        fresh(x.focus_at,w.now_ms,config_.knowledge.contact_keep_ms)){
                        retained_sound={x.objective,x.focus_at,SoundKind::gunshot,x.sound_report,false,x.sound_reporter};sound=&retained_sound;
                    }
                    break;
                }
                if(!sound)continue;goal=sound->position;has_goal=true;
                const PendingSound intent{a.key,a.ownership_generation,a.restrictions_revision,*sound,false};
                auto pi=std::find_if(sounds.begin(),sounds.end(),[&](const PendingSound&x){return x.actor==a.key;});
                if(pi==sounds.end())sounds.push_back(intent);else *pi=intent;
            }else if(search||assault){
                for(const auto&f:m.foes){work();const auto*sf=f.last_known();
                    if(!f.id.valid()||f.down||f.wounded||!sf||!source(*sf,a,w,config_)||friendly(a,f.id,sf->at_ms)||
                        (f.id.ghost()?f.id.side!=s.side_id:roster.count(key(f.id.actor))!=0))continue;
                    if(assault&&(f.id!=p.focus||sf->at_ms!=p.focus_fact.at_ms||!same(sf->position,p.focus_fact.position)))continue;
                    if(!focus||flat(a.position,sf->position)<flat(a.position,fact->position)){focus=&f;fact=sf;}
                }
                if(!focus)continue;goal=fact->position;has_goal=true;
                // Assault never chases a shared-only point as if it were sight.
                if(assault){
                    auto sight=[&](const Contact&c){return valid_key(c.key)&&c.key==focus->id.actor&&
                        personally_visible(c,a)&&c.hostile&&c.alive_when_observed&&!c.wounded;};
                    bool visible=sight(a.enemy);
                    for(std::size_t n=0;!visible&&n<a.visible_enemies.count;++n){work();visible=sight(a.visible_enemies.items[n]);}
                    if(!visible)continue;
                }
            }else if(approach&&services.buildings&&services.buildings->level_epoch==w.level_epoch&&services.buildings->map_hash==w.map_hash){
                const auto building=s.mission.building!=invalid_room?s.mission.building:mission.pending_building;
                const Portal*best=nullptr;float distance=(std::numeric_limits<float>::max)(),floor=a.position.y;
                for(const auto&b:services.buildings->buildings){work();if(b.id!=building)continue;
                    for(const auto&door:b.portals){work();
                        if(door.room_b!=invalid_room||!door.doorway||!door.traversable||!finite(door.centre)||!finite(door.normal)||
                            !pos(door.half_width)||!std::isfinite(door.wall_depth)||door.wall_depth<0||
                            (s.mission.entry_portal&&door.id!=s.mission.entry_portal))continue;
                        const float normal=std::hypot(door.normal.x,door.normal.z);if(normal<.99f||normal>1.01f)continue;
                        bool blocked=false;for(const auto&d:w.doors){work();if(d.portal==door.id&&(d.locked||d.held))blocked=true;}
                        if(blocked)continue;const float d=flat(a.position,door.centre);if(!best||d<distance){best=&door;distance=d;floor=a.position.y;
                            for(const auto&room:b.rooms){work();if(room.id==door.room_a&&std::isfinite(room.floor_y)){floor=room.floor_y;break;}}}
                    }
                }
                if(best){goal=best->centre;goal.y=floor;goal.x+=best->normal.x*(policy_.door_offset_m+best->wall_depth*.5f);
                    goal.z+=best->normal.z*(policy_.door_offset_m+best->wall_depth*.5f);objective_portal=best->id;has_goal=true;}
            }
            if(!has_goal||!finite(goal))continue;
            const auto old=applied.find(key(a.key));const Issued*held=nullptr;
            if(continuity&&old!=applied.end()&&actual(*old->second,a,w,config_))for(const auto&x:issued_){work();
                if(x.actor==a.key&&x.owner==a.ownership_generation&&x.restrictions==a.restrictions_revision&&x.previous<old->second->order_seq&&
                    x.at_ms<=old->second->issued_at_ms&&old->second->kind==x.kind&&old->second->goal_node==x.node&&same(old->second->goal_position,x.point)&&
                    x.objective_portal==objective_portal&&same(x.objective,goal)&&
                    ((!focus&&!x.focus.valid())||(focus&&x.focus==focus->id&&x.focus_at<=fact->at_ms))){held=&x;break;}}
            ProposedOrder proposal=*bi->second;const auto seq=proposal.expected_previous_order_seq;
            auto response=first_response(a,config_,w.now_ms,w.level_epoch,w.geometry_revision,seq);
            if(response.kind==OrderKind::yield||response.kind==OrderKind::reload)continue;
            std::uint32_t node=invalid_node;Vec3 snapped;bool pending=true,continued=false;
            // Arrival starts an observed point check; it does not itself clear the source.
            if(search&&flat(a.position,goal)<=policy_.arrival_m){
                node=a.node_id;snapped=a.position;pending=false;++out.arrived;
            }
            else if(held&&flat(a.position,held->point)>policy_.arrival_m){node=held->node;snapped=held->point;pending=false;continued=true;}
            else {
                if(!investigate&&flat(a.position,goal)<=policy_.arrival_m){++out.arrived;continue;}
                if(!services.project||(!investigate&&!services.path_length))continue;
                const float dx=goal.x-a.position.x,dz=goal.z-a.position.z,d=std::hypot(dx,dz);
                if(!pos(d)&&!investigate)continue;const float step=std::min(d,policy_.step_m);
                const float divisor=pos(d)?d:1.f;
                Vec3 wanted{a.position.x+dx/divisor*step,a.position.y,a.position.z+dz/divisor*step};
                const FactionStyle* style=&config_.faction_default;
                for(std::size_t i=0;i<config_.factions.count;++i)
                    if(config_.factions.items[i].faction==a.faction){style=&config_.factions.items[i];break;}
                const float spacing=d<=policy_.close_m?policy_.close_spread_m:style->spread_m;
                // Stable roster rank keeps side offsets from changing when the
                // fairness cursor rotates. There is no fixed three-metre cap.
                const std::size_t slot=slots.at(key(a.key));
                const float offset=slot?float((slot+1)/2)*(slot%2?1.f:-1.f)*spacing:0;
                // Project the heard destination itself; stepping straight
                // toward it can repeatedly stop against the inside of a hall.
                // The native route gate finds the actual way around its walls.
                if(investigate)wanted=goal;
                if(!investigate){wanted.x+=-dz/d*offset;wanted.z+=dx/d*offset;}
                if(objective_portal&&d<=policy_.step_m){wanted=goal;wanted.x+=-dz/d*offset;wanted.z+=dx/d*offset;}
                query();node=services.project(wanted,policy_.project_horizontal_m,policy_.project_vertical_m,&snapped);
                if(node==invalid_node||!finite(snapped)||flat(wanted,snapped)>policy_.project_horizontal_m||std::fabs(snapped.y-wanted.y)>policy_.project_vertical_m)continue;
                if(a.last_rejected.node==node&&a.last_rejected.restrictions_revision==a.restrictions_revision&&
                    (a.last_rejected.reason==reject_goal_invalid||a.last_rejected.reason==reject_goal_inaccessible))continue;
                if(investigate){
                    if(!q.route_service||!q.route_budget)continue;
                    const auto request=route_proof_source(w,a,node,snapped);query();
                    const auto route=request_route_proof(q.route_service,request,*q.route_budget,cancelled);
                    if(!route_proof_complete(route,request)){
                        if(route.status==RouteProofStatus::blocked)for(auto&x:sounds)if(x.actor==a.key)x.blocked=true;
                        ++out.pending_navigation;continue;
                    }
                    float total=0;for(std::size_t n=1;n<route.points.size();++n){work();total+=std::sqrt(distance_squared(route.points[n-1],route.points[n]));}
                    const float reach=(std::max)(policy_.arrival_m*2,config_.outdoor.p.posture.search_reach_m);
                    if(total<=reach&&std::fabs(a.position.y-goal.y)<=policy_.project_vertical_m){
                        checked.erase(std::remove_if(checked.begin(),checked.end(),[&](const Investigated&x){return x.actor==a.key&&flat(x.point,goal)<=reach;}),checked.end());
                        checked.push_back({a.key,a.ownership_generation,sound->at_ms,goal});
                        next.erase(std::remove_if(next.begin(),next.end(),[&](const Issued&x){return x.actor==a.key;}),next.end());
                        sounds.erase(std::remove_if(sounds.begin(),sounds.end(),[&](const PendingSound&x){return x.actor==a.key;}),sounds.end());
                        ++out.arrived;continue;
                    }
                    // Follow a bounded leg of the proven polyline. A necessary
                    // detour away from the sound is valid progress, not a stall.
                    float walked=0;Vec3 prior=a.position;std::size_t chosen=0;
                    for(std::size_t n=0;n<route.points.size();++n){work();
                        walked+=std::sqrt(distance_squared(prior,route.points[n]));prior=route.points[n];
                        if(walked<=policy_.step_m)chosen=n;else break;
                    }
                    node=chosen==0?a.node_id:route.nodes[(std::min)(chosen-1,route.nodes.size()-1)];snapped=route.points[chosen];
                    if(flat(a.position,snapped)<=policy_.arrival_m)continue;
                    pending=false;
                }
                if(!investigate){query();const float length=services.path_length(a.node_id,node,policy_.path_nodes);
                    if(!std::isfinite(length)||length<0)continue;}
                if(q.routes){const OutdoorSoloRoute*route=nullptr;bool duplicate=false;
                    for(const auto&r:*q.routes){work();if(r.source.actor==a.key&&r.goal_node==node&&same(r.goal,snapped)){if(route)duplicate=true;route=&r;}}
                    if(route&&!duplicate){const auto&x=route->source;
                        const bool exact=x.level_epoch==w.level_epoch&&x.snapshot_seq==w.sequence&&x.geometry_revision==w.geometry_revision&&x.ownership_generation==a.ownership_generation&&
                            x.restrictions_revision==a.restrictions_revision&&x.at_ms==a.observed_at_ms&&x.origin_node==a.node_id&&same(x.actor_position,a.position);
                        if(exact&&route->restricted_route==Evidence::no)continue;
                        if(exact&&route->restricted_route==Evidence::yes)pending=false;
                    }
                }
                if(pending&&!policy_.allow_pending_navigation)continue;
            }
            if(investigate&&continued){
                if(!q.route_service||!q.route_budget)continue;
                const auto request=route_proof_source(w,a,node,snapped);query();
                if(!route_proof_complete(request_route_proof(q.route_service,request,*q.route_budget,cancelled),request)){
                    ++out.pending_navigation;continue;
                }
            }
            bool clash=false;for(const auto&x:occupied){work();if(x.first!=a.key&&flat(x.second,snapped)<config_.minimum_goal_separation){clash=true;break;}}
            if(clash)continue;
            proposal.order=response;auto&o=proposal.order;
            o.kind=assault?OrderKind::press:(search?OrderKind::search:OrderKind::advance);o.role=search?roles::search:roles::travel;
            o.flags&=~order_first_response;o.goal_node=node;o.goal_position=snapped;o.arrival_radius_m=policy_.arrival_m;o.assignment={};
            const bool sight=o.look.kind==LookKind::enemy;
            if(search||investigate){o.pace=Pace::walk;o.speed_scale=policy_.search_speed;o.stance=m.meter_level?Stance::crouch:Stance::stand;}
            else if(sight||flat(a.position,snapped)<=2*policy_.arrival_m){o.pace=Pace::walk;o.speed_scale=policy_.aimed_speed;o.stance=Stance::stand;}
            else {o.pace=Pace::run;o.speed_scale=flat(a.position,snapped)>=policy_.sprint_m?policy_.sprint_speed:policy_.run_speed;o.stance=Stance::stand;
                o.flags|=order_face_movement;o.fire={};if(o.speed_scale==policy_.sprint_speed)o.flags|=order_sprint;}
            if(!sight){auto aim=goal;if(search)aim.y+=config_.outdoor.p.solo.combat.target_height_m;
                o.look={LookKind::point,aim,{}};o.fire={};
                if(investigate){o.look={LookKind::path,{},{}};o.flags|=order_face_movement;}
            }
            proposal.has_cover_evidence=false;proposal.navigation_pending=pending;proposal.has_knowledge=false;
            if(focus){auto&t=proposal.knowledge;t={};t.receiver=a.key;t.receiver_ownership_generation=a.ownership_generation;t.receiver_faction=a.faction;
                t.foe=focus->id;t.target=focus->id.actor;t.observer=fact->reporter;t.observed_at_ms=fact->at_ms;t.position=fact->position;
                t.shared=fact->reporter!=a.key;t.unidentified=focus->id.ghost();t.personally_visible=sight&&a.enemy.key==focus->id.actor;
                t.source_snapshot_seq=t.personally_visible?w.sequence:0;proposal.has_knowledge=true;}
            std::size_t previous=next.size();for(std::size_t n=0;n<next.size();++n){work();if(next[n].actor==a.key){previous=n;break;}}
            // An unacknowledged refresh must not erase the older, still-applied
            // origin proof for this identical leg.
            Issued row{a.key,a.ownership_generation,a.restrictions_revision,continued?held->previous:seq,continued?held->at_ms:w.now_ms,
                node,snapped,o.kind,focus?focus->id:FoeId{},continued?held->focus_at:(fact?fact->at_ms:(sound?sound->at_ms:0)),goal,objective_portal,
                sound?sound->reporter:ActorKey{},sound?sound->from_report:false};
            if(previous==next.size())next.push_back(row);else next[previous]=row;
            occupied.push_back({a.key,snapped});out.pending_navigation+=pending;out.continued+=continued;out.proposals.push_back(std::move(proposal));
        }
        poll();issued_=std::move(next);investigated_=std::move(checked);pending_sounds_=std::move(sounds);stamp_=out.stamp;after_=advanced;cursor_stamp_=out.stamp;out.status=OutdoorStatus::ready;
        out.reason=out.proposals.empty()?"no_movement_candidate":(investigate?"investigating_sound":"movement_candidates");return measured();
    }catch(const Cancelled&){out.proposals.clear();out.status=OutdoorStatus::cancelled;out.reason="cancelled";return measured();}
    catch(const Exhausted&){
        // Budget exhaustion is a completed prefix, not cancellation. Retain only
        // finished per-actor proposals and rotate a source-qualified fair cursor.
        // With no completed proposal, the same capture can retry (stamp_ stays).
        if(validated&&advanced_once){after_=advanced;cursor_stamp_=out.stamp;}
        if(validated){pending_sounds_=std::move(sounds);}
        if(!out.proposals.empty()){issued_=std::move(next);investigated_=std::move(checked);stamp_=out.stamp;}
        out.exhausted=true;out.status=OutdoorStatus::ready;out.reason="budget";return measured();}
    catch(...){out.proposals.clear();out.status=OutdoorStatus::failed;out.reason="exception";return measured();}
}
}}}
