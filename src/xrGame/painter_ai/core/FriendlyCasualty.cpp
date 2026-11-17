#include "FriendlyCasualty.h"
#include <algorithm>
#include <map>
#include <set>
#include <tuple>
#include <cmath>
#include <limits>
namespace painter { namespace combat { namespace core {
namespace {
struct Interrupted {};
void poll(const std::function<bool()>& stop){if(stop&&stop())throw Interrupted{};}
struct Less {bool operator()(ActorKey a,ActorKey b)const{return std::tie(a.id,a.generation)<std::tie(b.id,b.generation);}};
bool contains(const SideSnapshot& s,ActorKey key){return std::find(s.members.begin(),s.members.end(),key)!=s.members.end();}
}
bool friendly_casualty_member(const FriendlyCasualtyFact& f,const SideSnapshot& s){
    return f.side_id==s.side_id&&f.faction==s.faction&&f.group==s.group&&f.mission_revision==s.mission.revision
        &&contains(s,f.observer)&&contains(s,f.target)
        &&(!f.target_native_squad||f.target_native_squad==s.group);
}
void complete_friendly_casualty_rosters(WorldSnapshot& w,const std::vector<SideSnapshot>& known,
    const FriendlyCasualtyEvidence& input,const Config& cfg){
    const auto checked=validate_friendly_casualties(w,input,cfg);
    if(!checked.accepted)return;
    for(const auto& f:checked.facts){if(f.alive&&!f.wounded)continue;
        auto current=std::find_if(w.sides.begin(),w.sides.end(),[&](const SideSnapshot& s){return s.side_id==f.side_id;});
        if(current==w.sides.end()||current->group!=f.group||current->faction!=f.faction
            ||current->mission.revision!=f.mission_revision||!contains(*current,f.observer))continue;
        bool witnessed=false;for(const auto& old:known)if(friendly_casualty_member(f,old))witnessed=true;
        if(!witnessed)continue;
        bool elsewhere=false;for(const auto& other:w.sides)if(other.side_id!=current->side_id&&contains(other,f.target))elsewhere=true;
        if(!elsewhere&&!contains(*current,f.target))current->members.push_back(f.target);
    }
}
bool friendly_casualty_aid_allowed(const FriendlyCasualtyFact& f,ActorKey helper,TimeMs now,TimeMs max_age){
    const auto& e=f.aid_eligibility;
    return valid_key(helper)&&f.visible_now&&f.friendly==Evidence::yes&&f.alive&&f.wounded
        &&fresh(f.observed_at_ms,now,max_age)&&e.allowed==Evidence::yes&&e.observed_at_ms<=f.observed_at_ms
        &&(!e.claimed||e.claimant==helper);
}
FriendlyCasualtyValidation validate_friendly_casualties(const WorldSnapshot& w,const FriendlyCasualtyEvidence& e,
    const Config& cfg,const std::function<bool()>& stop){
    FriendlyCasualtyValidation out;
    try {
        poll(stop);
        if(e.facts.empty()){out.accepted=true;return out;}
        if(e.level_epoch!=w.level_epoch||e.snapshot_seq!=w.sequence||e.map_hash!=w.map_hash
            ||e.geometry_revision!=w.geometry_revision||e.captured_at_ms!=w.now_ms)return out;
        std::map<std::uint32_t,const ActorSnapshot*> actors;
        for(const auto& a:w.actors){poll(stop);if(!actors.emplace(a.key.id,&a).second)return out;}
        std::map<std::pair<std::uint32_t,std::uint32_t>,std::size_t> rows;
        std::set<std::pair<std::uint32_t,std::uint32_t>> duplicates;
        std::map<std::uint32_t,const FriendlyCasualtyFact*> target_facts;std::set<std::uint32_t> conflicts;
        for(const auto& f:e.facts){poll(stop);const auto pair=std::make_pair(f.observer.id,f.target.id);
            if(!rows.emplace(pair,rows.size()).second)duplicates.insert(pair);
            auto prior=target_facts.emplace(f.target.id,&f);if(!prior.second){const auto& old=*prior.first->second;
                if(old.target!=f.target||old.alive!=f.alive||old.wounded!=f.wounded||old.faction!=f.faction
                    ||old.side_id!=f.side_id||old.group!=f.group||distance_squared(old.position,f.position)>.0001f)conflicts.insert(f.target.id);}}
        for(const auto& f:e.facts){poll(stop);const auto observer=actors.find(f.observer.id);
            bool valid=valid_key(f.observer)&&valid_key(f.target)&&f.observer!=f.target&&f.side_id
                &&f.group&&finite(f.position)&&f.visible_now&&f.friendly==Evidence::yes
                &&fresh(f.observed_at_ms,w.now_ms,cfg.snapshot_max_age_ms)
                &&f.aid_eligibility.observed_at_ms<=f.observed_at_ms
                &&!conflicts.count(f.target.id)&&!duplicates.count({f.observer.id,f.target.id})&&observer!=actors.end();
            if(valid){const auto& a=*observer->second;
                valid=a.key==f.observer&&a.level_epoch==w.level_epoch&&a.snapshot_seq==w.sequence
                    &&a.observed_at_ms==f.observed_at_ms&&a.alive&&!a.wounded&&(a.flags&flag_admitted)
                    &&a.ownership_generation&&a.ownership_generation==f.observer_owner
                    &&a.restrictions_revision==f.observer_restrictions&&a.faction==f.faction
                    &&(!a.native_squad||a.native_squad==f.group);
            }
            const auto target=actors.find(f.target.id);
            if(valid&&target!=actors.end()){const auto& a=*target->second;
                valid=a.key==f.target&&a.faction==f.faction&&a.native_squad==f.target_native_squad
                    &&a.alive==f.alive&&a.wounded==f.wounded;}
            if(valid)out.facts.push_back(f);else ++out.rejected;
        }
        poll(stop);out.accepted=true;return out;
    } catch(const Interrupted&) {out={};out.cancelled=true;return out;}
}

CombatAidResult combat_aid_orders(const WorldSnapshot& w,const SideSnapshot& side,const KnowledgeFrame& knowledge,
    const std::vector<FriendlyCasualtyFact>& facts,const std::vector<ProposedOrder>& base,const std::vector<Order>& accepted,
    const Config& cfg,const EngineServices& services,const RouteProofService* routes,CombatAidBudget& budget,std::uint64_t& sequence,
    const std::function<bool()>& stop){
    CombatAidResult out;auto token=sequence;
    auto work=[&](){poll(stop);if(!budget.work_left)return false;--budget.work_left;++budget.work_used;return true;};
    auto query=[&](){if(!work()||!budget.queries_left)return false;--budget.queries_left;++budget.queries_used;return true;};
    auto actor=[&](ActorKey k)->const ActorSnapshot*{for(const auto&a:w.actors)if(a.key==k)return &a;return nullptr;};
    auto held=[&](ActorKey k)->const Order*{for(const auto&o:accepted)if(o.actor==k)return &o;return nullptr;};
    std::vector<ProposedOrder> available=base;
    // The tactics layer intentionally omits actors whose accepted aid owns
    // their hands. Carry that exact accepted intent into fresh safety checks,
    // otherwise another free man could be sent to the same patient each tick.
    for(const auto& old:accepted)if(valid_key(old.aid)&&contains(side,old.actor)&&
        std::none_of(available.begin(),available.end(),[&](const ProposedOrder& p){return p.order.actor==old.actor;})){
        ProposedOrder p;p.order=old;p.expected_previous_order_seq=old.order_seq;p.group_id=side.side_id;
        available.push_back(p);
    }
    auto proposed=[&](ActorKey k)->const ProposedOrder*{for(const auto&p:available)if(p.order.actor==k)return &p;return nullptr;};
    const auto& r=cfg.room;
    if(!w.level_epoch||!w.sequence||!knowledge.accepted||knowledge.level_epoch!=w.level_epoch||knowledge.sequence!=w.sequence||
        !services.project||!services.rays||!routes||!routes->query||!(r.aid_min_body_m>0)||!(r.aid_reach_m>r.aid_min_body_m)||!r.aid_cap_ms)return out;
    try{
        std::vector<Vec3> shooters;
        for(const auto& m:knowledge.members){if(!work())return out;if(m.side!=side.side_id||!contains(side,m.actor))continue;
            for(const auto& f:m.foes){if(!work())return out;if(f.down||f.wounded)continue;const auto* fact=f.last_known();
                if(!fact||!fact->valid||!finite(fact->position)||!fresh(fact->at_ms,w.now_ms,cfg.memory_max_age_ms))continue;
                if(std::none_of(shooters.begin(),shooters.end(),[&](Vec3 p){return distance_squared(p,fact->position)<.0001f;}))shooters.push_back(fact->position);}}
        std::vector<const ActorSnapshot*> helpers;std::size_t rifles=0,covering_rifles=0;
        auto covering=[&](const ActorSnapshot& a){const auto* t=select_contact(a,cfg,w.now_ms);
            return a.weapon.kind==WeaponKind::rifle&&!(a.flags&flag_dont_shoot)&&t&&personally_visible(*t,a);};
        for(auto key:side.members){if(!work())return out;const auto*a=actor(key);const auto* old=held(key);const auto*p=proposed(key);
            if(!a||!old||!p||!a->alive||a->wounded||!(a->flags&flag_admitted)||a->yield_reasons||a->reloading||a->weapon.state==WeaponState::reloading||
                !a->weapon.ammo_in_mag||!a->current_position_accessible||a->observed_at_ms!=w.now_ms||a->snapshot_seq!=w.sequence||
                old->ownership_generation!=a->ownership_generation||old->restrictions_revision!=a->restrictions_revision||
                old->geometry_revision!=w.geometry_revision||old->expires_at_ms<=w.now_ms||p->order.throw_order.valid||old->throw_order.valid||
                p->order.flags&(order_committed_entry|order_urgent)||valid_key(p->order.finish)||valid_key(old->finish)||
                p->order.role==roles::care_security||p->order.role==roles::casualty_guard||
                (a->throw_state.phase==1||a->throw_state.phase==2))continue;
            const bool rifle=a->weapon.kind==WeaponKind::rifle;
            if(rifle&&!valid_key(old->aid)){++rifles;if(covering(*a))++covering_rifles;}
            helpers.push_back(a);
        }
        std::vector<const FriendlyCasualtyFact*> patients;std::set<std::uint32_t> seen;
        for(const auto& f:facts){if(!work())return out;if(!friendly_casualty_member(f,side)||!seen.insert(f.target.id).second||
            !f.alive||!f.wounded||!f.visible_now||f.friendly!=Evidence::yes||!fresh(f.observed_at_ms,w.now_ms,cfg.snapshot_max_age_ms))continue;patients.push_back(&f);}
        std::stable_sort(patients.begin(),patients.end(),[](const auto*a,const auto*b){return a->target.id<b->target.id;});
        std::set<std::uint32_t> used;
        auto ray=[&](Vec3 a,Vec3 b){if(!query())return Evidence::unknown;Segment seg{a,b};float distance=-2;services.rays(&seg,&distance,1);poll(stop);
            if(!std::isfinite(distance)||distance< -1)return Evidence::unknown;
            return distance==-1||distance>=std::sqrt(distance_squared(a,b))?Evidence::yes:Evidence::no;};
        for(const auto* patient:patients){if(!work())break;
            auto ordered=helpers;std::stable_sort(ordered.begin(),ordered.end(),[&](const auto*a,const auto*b){const auto* aa=held(a->key);const auto* bb=held(b->key);
                const bool ac=aa->aid==patient->target,bc=bb->aid==patient->target;if(ac!=bc)return ac;
                const auto da=distance_squared(a->position,patient->position),db=distance_squared(b->position,patient->position);return da!=db?da<db:a->key.id<b->key.id;});
            ActorKey existing_helper;
            for(const auto& original:available)if(original.order.aid==patient->target)existing_helper=original.order.actor;
            for(const auto& original:accepted)if(original.aid==patient->target&&contains(side,original.actor)&&original.expires_at_ms>w.now_ms)
                existing_helper=original.actor;
            bool found=false;
            for(const auto* h:ordered){if(!work())break;const auto* old=held(h->key);const auto* p=proposed(h->key);const bool continuing=old->aid==patient->target;
                if((valid_key(existing_helper)&&existing_helper!=h->key)||used.count(h->key.id)||!friendly_casualty_aid_allowed(*patient,h->key,w.now_ms,cfg.snapshot_max_age_ms)||
                    (valid_key(old->aid)&&!continuing)||(valid_key(p->order.aid)&&p->order.aid!=patient->target)||
                    (!continuing&&!shooters.empty()&&h->weapon.kind==WeaponKind::rifle&&(rifles<=1||(covering(*h)&&covering_rifles<=1)))||
                    (h->aid_state.target==patient->target&&(h->aid_state.phase==3||h->aid_state.phase==4)))continue;
                // This command is freshly route/eligibility-validated below.
                // Renew its execution freshness, not the native aid episode:
                // the unchanged assignment token preserves AidExecution.started
                // and its original aid_cap_ms through approach and treatment.
                const auto original_deadline=saturated_after(w.now_ms,cfg.order_lifetime_ms);
                if(original_deadline<=w.now_ms)continue;
                std::vector<Vec3> desired;if(continuing)desired.push_back(old->goal_position);
                const float heading=std::atan2(h->position.z-patient->position.z,h->position.x-patient->position.x);
                for(float radius:r.aid_placement_radii_m)for(float angle:r.aid_placement_angles_rad)
                    desired.push_back({patient->position.x+radius*std::cos(heading+angle),patient->position.y,patient->position.z+radius*std::sin(heading+angle)});
                for(const auto point:desired){if(!query())break;Vec3 goal;const auto node=services.project(point,r.projection_m,r.floor_tolerance_m,&goal);poll(stop);
                    if(node==invalid_node||!finite(goal)||std::abs(goal.y-point.y)>r.floor_tolerance_m||distance_squared(goal,point)>r.projection_m*r.projection_m||
                        distance_squared(goal,patient->position)<r.aid_min_body_m*r.aid_min_body_m||distance_squared(goal,patient->position)>r.aid_reach_m*r.aid_reach_m)continue;
                    if(h->last_rejected.node==node&&h->last_rejected.restrictions_revision==h->restrictions_revision&&
                       (h->last_rejected.reason==reject_goal_invalid||h->last_rejected.reason==reject_goal_inaccessible))continue;
                    bool occupied=false;for(const auto& order:accepted)if(order.actor!=h->key&&order.actor!=patient->target&&
                        (order.goal_node==node||distance_squared(goal,order.goal_position)<r.spacing_m*r.spacing_m))occupied=true;
                    for(const auto& order:out.proposals)if(order.order.goal_node==node||distance_squared(goal,order.order.goal_position)<r.spacing_m*r.spacing_m)occupied=true;
                    if(occupied)continue;
                    const auto request=route_proof_source(w,*h,node,goal);const auto route=request_route_proof(routes,request,budget.routes,stop);poll(stop);
                    if(!route_proof_complete(route,request)){++out.pending;continue;}
                    bool protected_path=true;
                    for(std::size_t i=1;i<route.points.size()&&protected_path;++i){const auto from=route.points[i-1],to=route.points[i];
                        const float samples=std::ceil(std::sqrt(distance_squared(from,to))/r.aid_min_body_m);
                        if(!std::isfinite(samples)||samples>float(budget.work_left)){protected_path=false;break;}
                        const auto count=std::max<std::size_t>(1,std::size_t(samples));
                        for(std::size_t k=0;k<=count&&protected_path;++k){if(!work()){protected_path=false;break;}const float t=float(k)/float(count);
                            Vec3 sample{from.x+(to.x-from.x)*t,from.y+(to.y-from.y)*t+r.look_height_m,from.z+(to.z-from.z)*t};
                            for(auto shooter:shooters){shooter.y+=r.look_height_m;if(ray(shooter,sample)!=Evidence::no){protected_path=false;break;}}}}
                    if(!protected_path)continue;
                    Vec3 reach=goal,target=patient->position;reach.y+=r.look_height_m;target.y+=r.look_height_m;
                    if(ray(reach,target)!=Evidence::yes)continue;
                    constexpr auto domain=std::uint64_t(1)<<63;
                    if(!continuing&&token>=domain-1)return out;
                    ProposedOrder result=*p;auto& order=result.order;
                    order.actor=h->key;order.level_epoch=w.level_epoch;order.snapshot_seq=w.sequence;order.geometry_revision=w.geometry_revision;
                    order.ownership_generation=h->ownership_generation;order.restrictions_revision=h->restrictions_revision;
                    order.kind=OrderKind::recover;order.role=roles::aid_casualty;order.goal_node=node;order.goal_position=goal;
                    order.aid=patient->target;order.finish={};order.throw_order={};order.portal={};order.pie={};order.corner={};order.expose={};
                    order.assignment={};order.assignment.generation=continuing?old->assignment.generation:domain|++token;
                    if(!order.assignment.generation)order.assignment.generation=domain|++token;
                    order.issued_at_ms=w.now_ms;order.expires_at_ms=original_deadline;order.order_seq=0;order.pace=Pace::walk;
                    order.stance=Stance::crouch;order.arrival_radius_m=r.aid_arrival_m;order.lean=0;order.side=0;order.flags=0;
                    order.look={LookKind::point,patient->position,{}};order.fire={};result.expected_previous_order_seq=old->order_seq;
                    result.point_fire={};result.has_cover_evidence=false;result.has_knowledge=false;result.navigation_pending=false;
                    out.proposals.push_back(result);used.insert(h->key.id);if(!continuing&&h->weapon.kind==WeaponKind::rifle){--rifles;if(covering(*h))--covering_rifles;}found=true;break;
                }
                if(found)break;
            }
        }
        poll(stop);sequence=token;return out;
    }catch(const Interrupted&){out={};out.cancelled=true;return out;}
}
}}}
