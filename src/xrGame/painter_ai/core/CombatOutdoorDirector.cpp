#include "CombatOutdoorDirector.h"
#include "CombatMissionDoor.h"
#include "CombatOutdoorFallback.h"
#include "CombatWoundedFinisher.h"
#include "CombatBuildingHold.h"
#include <algorithm>
#include <set>

namespace painter { namespace combat { namespace core {
namespace {
using Key=std::pair<std::uint32_t,std::uint64_t>;
Key key(ActorKey a){return {a.id,a.generation};}
bool source_matches(const OutdoorStamp&s,const WorldSnapshot&w){return s.level_epoch==w.level_epoch&&s.map_hash==w.map_hash&&
    s.geometry_revision==w.geometry_revision&&s.snapshot_seq==w.sequence&&s.at_ms==w.now_ms;}
bool protected_order(const Order&o){return o.assignment.generation||o.portal.valid||o.throw_order.valid||valid_key(o.aid)||
    valid_key(o.finish)||o.kind==OrderKind::entry||o.kind==OrderKind::withdraw||o.kind==OrderKind::recover||o.kind==OrderKind::reload||
    (o.flags&order_committed_entry);}
}
CombatOutdoorDirector::CombatOutdoorDirector(Config c,OutdoorDispatchPolicy p):config_(std::move(c)),policy_(p){}
void CombatOutdoorDirector::reset_side(std::uint32_t side){states_.erase(side);}
OutdoorDirectorResult CombatOutdoorDirector::update(const WorldSnapshot&w,const KnowledgeFrame&k,
    const std::vector<CombatMissionIntel>&intel,const SquadResult&plans,const std::vector<Order>&current,
    const std::vector<Order>&executed,const std::vector<ActorCoverCache>&cover,const OutdoorExecutionEvidence&e,
    const OutdoorCaptureEvidence&capture,const std::function<bool()>&cancelled,const EngineServices&services,
    const std::vector<ActorKey>* publication_cohort){
    OutdoorDirectorResult out;auto stop=[&](){return cancelled&&cancelled();};
    auto abort=[&](OutdoorStatus status){OutdoorDirectorResult empty;empty.status=status;return empty;};
    try{
        if(stop())return abort(OutdoorStatus::cancelled);
        if(!plans.accepted||plans.cancelled||plans.level_epoch!=w.level_epoch||plans.snapshot_seq!=w.sequence||
            !k.accepted||k.cancelled||k.level_epoch!=w.level_epoch||k.sequence!=w.sequence)return out;
        if((policy_.solo.enabled||policy_.self.enabled)&&!policy_.personal.enabled)return out;
        if(policy_.withdrawal_enable&&!policy_.assessment_enable)return out;
        const auto index=index_outdoor_execution(w,executed,cover,e,cancelled);
        if(index.status()==OutdoorExecutionStatus::cancelled)return abort(OutdoorStatus::cancelled);
        if(index.status()!=OutdoorExecutionStatus::ready)return out;
        out.index_cost=index.costs();
        std::set<Key> personal_cohort;
        if(publication_cohort)for(auto actor:*publication_cohort){
            if(stop())return abort(OutdoorStatus::cancelled);
            if(!valid_key(actor)||!personal_cohort.insert(key(actor)).second)return out;
        }
        std::map<Key,const ActorSnapshot*> actors;std::map<Key,const MemberKnowledge*> members;
        std::map<Key,const Order*> previous,applied;std::map<Key,const ActorCoverCache*> covers;
        std::map<Key,const OutdoorCoverProfile*> profiles;std::map<Key,const ProposedOrder*> baseline;
        std::map<std::uint32_t,const CombatMissionIntel*> missions;std::map<std::uint32_t,const SquadView*> groups;
        std::map<std::uint32_t,const OutdoorFallbackEvidence*> fallbacks;
        std::map<Key,std::vector<OutdoorSoloRoute>> solo_routes;
        std::map<Key,const OutdoorSoloThrow*> solo_throws;
        std::map<Key,const OutdoorSoloGrenadeEscape*> grenade_escapes;
        std::map<Key,const OutdoorSoloAimEvidence*> solo_aim;
        std::map<Key,const OutdoorSoloLeanCapture*> solo_lean;
        std::map<std::uint32_t,std::vector<OutdoorWithdrawalCandidate>> withdrawal_candidates;
        for(const auto&a:w.actors){if(stop())return abort(OutdoorStatus::cancelled);if(!actors.emplace(key(a.key),&a).second)return out;}
        for(const auto&m:k.members){if(stop())return abort(OutdoorStatus::cancelled);if(!members.emplace(key(m.actor),&m).second)return out;}
        for(const auto&o:current){if(stop())return abort(OutdoorStatus::cancelled);if(!previous.emplace(key(o.actor),&o).second)return out;}
        for(const auto&o:executed){if(stop())return abort(OutdoorStatus::cancelled);if(!applied.emplace(key(o.actor),&o).second)return out;}
        for(const auto&c:cover){if(stop())return abort(OutdoorStatus::cancelled);if(!covers.emplace(key(c.actor),&c).second)return out;}
        for(const auto&p:e.profiles){if(stop())return abort(OutdoorStatus::cancelled);if(!profiles.emplace(key(p.source.actor),&p).second)return out;}
        for(const auto&p:plans.proposals){if(stop())return abort(OutdoorStatus::cancelled);if(!baseline.emplace(key(p.order.actor),&p).second)return out;}
        for(const auto&i:intel){if(stop())return abort(OutdoorStatus::cancelled);if(!missions.emplace(i.side,&i).second)return out;}
        for(const auto&g:plans.groups){if(stop())return abort(OutdoorStatus::cancelled);if(!groups.emplace(g.side_id,&g).second)return out;}
        if(source_matches(capture.source,w)){
            for(const auto&f:capture.fallbacks){if(stop())return abort(OutdoorStatus::cancelled);
                const auto row=fallbacks.emplace(f.stamp.side,&f);if(!row.second)row.first->second=nullptr;}
            for(const auto&r:capture.solo_routes){if(stop())return abort(OutdoorStatus::cancelled);solo_routes[key(r.source.actor)].push_back(r);}
            for(const auto&t:capture.solo_throws){if(stop())return abort(OutdoorStatus::cancelled);
                const auto row=solo_throws.emplace(key(t.source.actor),&t);if(!row.second)row.first->second=nullptr;}
            for(const auto&g:capture.grenade_escapes){if(stop())return abort(OutdoorStatus::cancelled);
                const auto row=grenade_escapes.emplace(key(g.source.actor),&g);if(!row.second)row.first->second=nullptr;}
            for(const auto&c:capture.withdrawal_candidates){if(stop())return abort(OutdoorStatus::cancelled);withdrawal_candidates[c.stamp.side].push_back(c);}
            for(const auto&a:capture.solo_aim){if(stop())return abort(OutdoorStatus::cancelled);
                const auto row=solo_aim.emplace(key(a.source.actor),&a);if(!row.second)row.first->second=nullptr;}
            for(const auto&l:capture.solo_lean){if(stop())return abort(OutdoorStatus::cancelled);
                const auto row=solo_lean.emplace(key(l.source.actor),&l);if(!row.second)row.first->second=nullptr;}
        }
        // Transactional module state. Cancellation must not advance posture or
        // consume an actual-shot receipt in a discarded worker result.
        auto next=states_;std::set<std::uint32_t> active;
        OutdoorFireQuota fire_quota{policy_.fire_work_per_update,policy_.fire_queries_per_update,0,0};
        OutdoorSoloQuota solo_quota{policy_.solo_work_per_update,policy_.solo_queries_per_update,0,0};
        OutdoorSelfQuota self_quota{policy_.self_work_per_update,0};
        OutdoorMovementQuota movement_quota{policy_.movement_work_per_update,policy_.movement_queries_per_update,0,0};
        std::size_t withdrawal_work=policy_.withdrawal_work_per_update;
        RouteProofBudget route_budget{source_matches(capture.source,w)?capture.route_service.work_per_update:0,0};
        // Reserve part of the SAME allowance for urgent local reactions. General
        // cover/fallback searches must not starve grenade/reload/buddy queries.
        RouteProofBudget local_route_budget{route_budget.work_left/3,0};route_budget.work_left-=local_route_budget.work_left;
        std::map<std::uint32_t,const SideSnapshot*> ordered;
        for(const auto&s:w.sides){if(stop())return abort(OutdoorStatus::cancelled);
            if(!active.insert(s.side_id).second)return out;ordered.emplace(s.side_id,&s);}
        std::vector<const SideSnapshot*> side_order;side_order.reserve(ordered.size());
        const auto split=ordered.upper_bound(after_side_);
        for(auto i=split;i!=ordered.end();++i)side_order.push_back(i->second);
        for(auto i=ordered.begin();i!=split;++i)side_order.push_back(i->second);
        std::uint32_t next_after=after_side_;bool fair_advanced=false;
        for(const auto*side:side_order){
            if(stop())return abort(OutdoorStatus::cancelled);auto s=*side;
            const auto mi=missions.find(s.side_id);const auto gi=groups.find(s.side_id);
            if(mi==missions.end()||gi==groups.end()||mi->second->status!=MissionIntelStatus::ready)continue;
            auto found=next.find(s.side_id);if(found==next.end())found=next.emplace(std::piecewise_construct,
                std::forward_as_tuple(s.side_id),std::forward_as_tuple(config_,policy_)).first;
            auto&state=found->second;
            // Every side consumes indexed local rows, including assessment.
            // Unseen enemies in the global actor vector supply no facts here.
            WorldSnapshot local;local.level_epoch=w.level_epoch;local.map_hash=w.map_hash;local.geometry_revision=w.geometry_revision;
            local.sequence=w.sequence;local.now_ms=w.now_ms;local.sides={s};local.doors=w.doors;
            KnowledgeFrame known;known.accepted=true;known.level_epoch=k.level_epoch;known.sequence=k.sequence;
            std::vector<Order> own_orders;std::vector<ActorCoverCache> own_cover;OutdoorExecutionEvidence own_evidence;
            std::set<Key> roster;
            for(auto actor:s.members){if(stop())return abort(OutdoorStatus::cancelled);const auto id=key(actor);roster.insert(id);
                auto a=actors.find(id);if(a!=actors.end())local.actors.push_back(*a->second);
                auto m=members.find(id);if(m!=members.end())known.members.push_back(*m->second);
                auto o=applied.find(id);if(o!=applied.end())own_orders.push_back(*o->second);
                auto c=covers.find(id);if(c!=covers.end())own_cover.push_back(*c->second);
                auto p=profiles.find(id);if(p!=profiles.end())own_evidence.profiles.push_back(*p->second);
            }
            // Dead contacts are intentionally absent from per-man foe rows.
            // Preserve this side's original K1 casualty/report provenance too.
            for(const auto& side_knowledge:k.sides){if(stop())return abort(OutdoorStatus::cancelled);
                if(side_knowledge.side==s.side_id)known.sides.push_back(side_knowledge);}
            for(const auto& event:k.events){if(stop())return abort(OutdoorStatus::cancelled);
                if(event.side==s.side_id)known.events.push_back(event);}
            // Keep the side's effective elements across passes. Their first
            // allocation is SquadTactics'; subsequent leapfrogs require actual
            // arrived/firing receipts below, not a cadence or proposed goal.
            std::vector<std::pair<ActorKey,std::uint64_t>> element_owners;
            for(auto actor:gi->second->members){auto row=actors.find(key(actor));
                element_owners.push_back({actor,row==actors.end()?0:row->second->ownership_generation});}
            const auto& es=state.element_source;
            const bool same_elements=es.level_epoch==w.level_epoch&&es.map_hash==w.map_hash&&es.geometry_revision==w.geometry_revision&&
                es.group==s.group&&es.side==s.side_id&&es.faction==s.faction&&es.mission_revision==s.mission.revision&&
                state.elements.members==gi->second->members&&state.element_owners==element_owners;
            if(!same_elements){state.elements=*gi->second;state.element_owners=element_owners;state.bound_legs.clear();state.support_failure={};}
            const auto* element_group=&state.elements;
            // Posture owns its focus selection. Preview on a disposable copy,
            // then commit exactly one update using actual execution evidence.
            auto preview=state.posture;OutdoorExecutionFacts unknown;
            const auto focus=preview.update(local,s,known,*mi->second,unknown,cancelled);
            if(focus.status==OutdoorStatus::cancelled)return abort(OutdoorStatus::cancelled);
            if(focus.status!=OutdoorStatus::ready)continue;
            OutdoorDispatchReceipt receipt;
            receipt.execution=outdoor_execution_facts(config_,index,s,*element_group,focus,cancelled);
            if(receipt.execution.status==OutdoorExecutionStatus::cancelled)return abort(OutdoorStatus::cancelled);
            if(receipt.execution.status!=OutdoorExecutionStatus::ready)continue;
            state.element_source=focus.stamp;
            // Execution evidence is built before candidate routing. Revalidate
            // already-applied moving goals here, so continuation and posture
            // consume the actual restricted route rather than forever UNKNOWN.
            if(source_matches(capture.source,w))for(auto& proof:receipt.execution.facts.actors){
                if(stop())return abort(OutdoorStatus::cancelled);
                const auto& order=proof.applied;const auto actor=actors.find(key(order.actor));
                if(actor==actors.end()||order.pace==Pace::stand||!route_budget.work_left)continue;
                const auto request=route_proof_source(local,*actor->second,order.goal_node,order.goal_position);
                const auto used=route_budget.work_used;
                const auto route=request_route_proof(&capture.route_service,request,route_budget,cancelled);
                if(route_budget.work_used!=used&&!fair_advanced){next_after=s.side_id;fair_advanced=true;}
                if(observe_outdoor_route(config_,local,*actor->second,route,proof))++out.route_completed;
            }
            for(const auto& proof:receipt.execution.facts.actors){
                if(proof.element!=OutdoorElement::mover||proof.applied.kind!=OrderKind::bound||proof.movement_route!=Evidence::yes)continue;
                const auto& order=proof.applied;auto old=std::find_if(state.bound_legs.begin(),state.bound_legs.end(),
                    [&](const SideState::BoundLeg& leg){return leg.actor==order.actor;});
                SideState::BoundLeg leg{order.actor,order.ownership_generation,order.restrictions_revision,order.goal_node,order.goal_position};
                if(old==state.bound_legs.end())state.bound_legs.push_back(leg);else *old=leg;
            }
            bool arrived=!element_group->movers.empty(),arrived_firing=false;
            for(auto mover:element_group->movers){
                if(stop())return abort(OutdoorStatus::cancelled);
                const auto actor=actors.find(key(mover));const auto proof=std::find_if(receipt.execution.facts.actors.begin(),receipt.execution.facts.actors.end(),
                    [&](const OutdoorActorExecution& row){return row.applied.actor==mover;});
                const auto leg=std::find_if(state.bound_legs.begin(),state.bound_legs.end(),[&](const SideState::BoundLeg& row){return row.actor==mover;});
                if(actor==actors.end()||proof==receipt.execution.facts.actors.end()||leg==state.bound_legs.end()){arrived=false;continue;}
                const auto&a=*actor->second;const auto&o=proof->applied;
                const bool reached=leg->owner==a.ownership_generation&&leg->restrictions==a.restrictions_revision&&
                    leg->node==o.goal_node&&distance_squared(leg->goal,o.goal_position)==0&&
                    distance_squared(a.position,o.goal_position)<=o.arrival_radius_m*o.arrival_radius_m&&
                    proof->cover_verified==Evidence::yes&&proof->cover!=OutdoorCover::unknown&&proof->cover!=OutdoorCover::open;
                if(!reached){arrived=false;continue;}
                const auto mind=members.find(key(a.key));const bool pinned=mind==members.end()||mind->second->meter_level>=3;
                const auto* contact=select_contact(a,config_,w.now_ms);
                const bool own_line=contact&&contact->key==focus.focus.actor&&personally_visible(*contact,a)&&
                    o.look.kind==LookKind::enemy&&o.look.target==contact->key;
                const bool point_line=o.fire.at_valid&&finite(o.fire.at)&&distance_squared(o.fire.at,focus.focus_fact.position)==0;
                if(!pinned&&!a.reloading&&a.weapon.state!=WeaponState::reloading&&a.weapon.ammo_in_mag&&!(a.flags&flag_dont_shoot)&&
                    proof->focus==focus.focus&&proof->focus_at_ms==focus.focus_fact.at_ms&&proof->fire_line==Evidence::yes&&
                    (proof->cover!=OutdoorCover::full||o.expose.lean)&&o.fire.allowed&&o.fire.until_ms>w.now_ms&&
                    (point_line||(own_line&&a.last_applied.fired_since_previous)))arrived_firing=true;
            }
            if(arrived&&arrived_firing&&state.posture.state().posture==OutdoorPosture::attack&&!element_group->base.empty()){
                std::swap(state.elements.base,state.elements.movers);state.bound_legs.clear();receipt.elements_swapped=true;
                for(auto& proof:receipt.execution.facts.actors)
                    proof.element=std::find(element_group->base.begin(),element_group->base.end(),proof.applied.actor)!=element_group->base.end()
                        ?OutdoorElement::base:OutdoorElement::mover;
            }
            receipt.effective_elements=*element_group;

            if(policy_.withdrawal_enable){
                receipt.withdrawal_execution=state.withdrawal.capture_execution(local,s,own_orders,receipt.execution.facts,withdrawal_work,cancelled);
                if(receipt.withdrawal_execution.status==OutdoorStatus::cancelled)return abort(OutdoorStatus::cancelled);
            }
            auto produced=outdoor_fallback_candidates(config_,w,s,k,focus.leader,own_cover,own_evidence,
                capture.route_service,route_budget,services,cancelled);
            if(produced.cancelled)return abort(OutdoorStatus::cancelled);
            if(produced.link_up_valid){s.link_up=produced.link_up;local.sides.front().link_up=produced.link_up;}
            auto& produced_withdrawal=withdrawal_candidates[s.side_id];
            produced_withdrawal.insert(produced_withdrawal.end(),produced.withdrawal_candidates.begin(),produced.withdrawal_candidates.end());
            if(policy_.assessment_enable){
                auto assess=state.assessment?std::make_shared<CombatOutdoorAssessment>(*state.assessment):
                    std::make_shared<CombatOutdoorAssessment>(config_,policy_.assessment);
                OutdoorAssessmentRequest assessment;assessment.posture=state.posture.state().posture;assessment.leader=focus.leader;
                const auto fallback=fallbacks.find(s.side_id);if(fallback!=fallbacks.end())assessment.fallback=fallback->second;
                else if(produced.fallback_valid)assessment.fallback=&produced.fallback;
                assessment.execution=&receipt.execution.facts;
                if(receipt.withdrawal_execution.status==OutdoorStatus::ready&&!receipt.withdrawal_execution.legs.empty()){
                    assessment.executing_serial=receipt.withdrawal_execution.legs.front().assessment_serial;
                    assessment.execution_legs=receipt.withdrawal_execution.legs;
                }
                receipt.assessment=assess->update(local,s,known,assessment,cancelled);
                if(receipt.assessment.status==OutdoorStatus::cancelled)return abort(OutdoorStatus::cancelled);
                if(receipt.assessment.status==OutdoorStatus::ready){
                    receipt.execution.facts.assessment=receipt.assessment.verdict;
                    receipt.execution.facts.assessment_serial=receipt.assessment.serial;
                    receipt.execution.facts.safe_fallback=receipt.assessment.safe_fallback;
                    receipt.execution.facts.casualty_obligation_clear=receipt.assessment.casualty_obligation_clear;
                    receipt.execution.facts.fallback=receipt.assessment.fallback;state.assessment=std::move(assess);
                    receipt.execution.facts.assessment_managed=true;
                    receipt.execution.facts.assessment_executing=receipt.assessment.execution_started||receipt.assessment.withdrawing;
                    receipt.execution.facts.assessment_completed=receipt.assessment.completed;
                    receipt.execution.facts.assessment_capped=receipt.assessment.capped;
                    receipt.execution.facts.assessment_withdrawal_episode_serial=receipt.assessment.withdrawal_episode_serial;
                    receipt.execution.facts.assessment_cooldown_remaining_ms=receipt.assessment.cooldown_remaining_ms;
                    receipt.execution.facts.assessment_exchange_hold=receipt.assessment.exchange_hold;
                    receipt.execution.facts.assessment_exchange_at_ms=receipt.assessment.exchange_at_ms;
                    receipt.execution.facts.assessment_exchange_until_ms=receipt.assessment.exchange_until_ms;
                    receipt.execution.facts.assessment_advance_cost=receipt.assessment.advance_cost;
                    receipt.execution.facts.assessment_hold_cost=receipt.assessment.hold_cost;
                }
            }
            measure_mission_door(config_,local,*mi->second,focus.leader,services,
                capture.route_service,route_budget,receipt.execution.facts,cancelled);
            out.route_work=route_budget.work_used;
            receipt.execution.facts.support_failure=state.support_failure;
            receipt.posture=state.posture.update(local,s,known,*mi->second,receipt.execution.facts,cancelled);
            if(receipt.posture.status==OutdoorStatus::cancelled)return abort(OutdoorStatus::cancelled);
            if(receipt.posture.status!=OutdoorStatus::ready)continue;
            std::vector<OutdoorBoundPath> paths;
            if(source_matches(capture.source,w))for(const auto&p:capture.paths){if(stop())return abort(OutdoorStatus::cancelled);if(roster.count(key(p.actor)))paths.push_back(p);}
            // Query only actual worker cover candidates. This callback queues
            // values and returns immediately; native objects are authority-only.
            std::vector<ActorCoverCache*> route_caches;
            for(auto&cache:own_cover)if(key(cache.actor)>key(state.route_after))route_caches.push_back(&cache);
            for(auto&cache:own_cover)if(key(cache.actor)<=key(state.route_after))route_caches.push_back(&cache);
            auto prove=[&](const ActorSnapshot&a,std::uint32_t node,Vec3 goal,OutdoorBoundPath&path){
                const auto request=route_proof_source(local,a,node,goal);const auto before=route_budget.work_used;
                auto proof=request_route_proof(&capture.route_service,request,route_budget,cancelled);
                out.route_work=route_budget.work_used;
                if(route_budget.work_used!=before){state.route_after=a.key;if(!fair_advanced){next_after=s.side_id;fair_advanced=true;}}
                if(!route_proof_complete(proof,request)){
                    if(proof.status==RouteProofStatus::blocked&&same_route_capture(proof.validated,request))path.complete=Evidence::no;
                    return false;
                }
                path.actor=a.key;path.owner=a.ownership_generation;path.restrictions_revision=a.restrictions_revision;
                path.origin_node=a.node_id;path.goal_node=node;path.goal_position=goal;path.complete=Evidence::yes;path.points=std::move(proof.points);++out.route_completed;return true;
            };
            auto prove_withdrawal=[&]{
                auto rows=withdrawal_candidates.find(s.side_id);if(rows==withdrawal_candidates.end())return;
                for(auto&candidate:rows->second){
                    if(stop()||!route_budget.work_left)return;
                    const auto actor=actors.find(key(candidate.profile.source.actor));if(actor==actors.end())continue;
                    auto&spot=candidate.spot;const auto&a=*actor->second;
                    if(!source_matches(candidate.stamp,w)||spot.accessible==Evidence::no||spot.reachable==Evidence::no||
                       (spot.restrictions_revision&&spot.restrictions_revision!=a.restrictions_revision))continue;
                    OutdoorBoundPath path;if(prove(a,spot.node,spot.position,path)){
                        candidate.route=std::move(path);spot.accessible=spot.reachable=Evidence::yes;spot.restrictions_revision=a.restrictions_revision;
                    }
                }
            };
            if(receipt.posture.posture==OutdoorPosture::withdraw)prove_withdrawal();
            for(auto*cache:route_caches){
                if(stop())return abort(OutdoorStatus::cancelled);if(!route_budget.work_left)break;
                const auto actor=actors.find(key(cache->actor));if(actor==actors.end())continue;const auto&a=*actor->second;
                if(cache->level_epoch!=w.level_epoch||cache->snapshot_seq!=w.sequence||cache->ownership_generation!=a.ownership_generation||
                   cache->restrictions_revision!=a.restrictions_revision||cache->origin_node!=a.node_id||cache->observed_at_ms!=w.now_ms)continue;
                for(auto&spot:cache->spots){
                    if(stop())return abort(OutdoorStatus::cancelled);if(!route_budget.work_left)break;
                    if(spot.node==invalid_node||!finite(spot.position)||spot.geometry_revision!=w.geometry_revision||spot.accessible==Evidence::no||spot.reachable==Evidence::no||(spot.restrictions_revision&&spot.restrictions_revision!=a.restrictions_revision))continue;
                    OutdoorBoundPath path;if(!prove(a,spot.node,spot.position,path)){
                        if(path.complete==Evidence::no){spot.accessible=spot.reachable=Evidence::no;spot.restrictions_revision=a.restrictions_revision;}
                        continue;
                    }
                    spot.accessible=spot.reachable=Evidence::yes;spot.restrictions_revision=a.restrictions_revision;
                    paths.erase(std::remove_if(paths.begin(),paths.end(),[&](const OutdoorBoundPath&old){return old.actor==a.key&&old.goal_node==spot.node;}),paths.end());paths.push_back(path);
                    // Preserve independent threat-opening evidence; a route is
                    // never by itself proof that the path is safe from a foe.
                    auto&routes=solo_routes[key(a.key)];bool matched=false;
                    for(auto&r:routes)if(r.goal_node==spot.node&&distance_squared(r.goal,spot.position)==0){r.restricted_route=Evidence::yes;matched=true;}
                    const auto profile=profiles.find(key(a.key));
                    if(!matched&&profile!=profiles.end()){OutdoorSoloRoute route;route.source=profile->second->source;route.goal_node=spot.node;route.goal=spot.position;route.restricted_route=Evidence::yes;routes.push_back(route);}
                }
            }
            if(receipt.posture.posture!=OutdoorPosture::withdraw)prove_withdrawal();
            if(stop())return abort(OutdoorStatus::cancelled);
            std::map<decltype(key(ActorKey{})),const ActorCoverCache*> proved_covers;
            for(const auto&cache:own_cover)proved_covers.emplace(key(cache.actor),&cache);
            OutdoorBoundsRequest request;request.world=&local;request.side=&s;request.knowledge=&known;request.elements=element_group;
            request.posture=&receipt.posture;request.execution=&receipt.execution.facts;request.accepted_orders=&own_orders;
            request.cover=&own_cover;request.query_evidence=&own_evidence;request.path_stamp=receipt.posture.stamp;request.paths=&paths;
            receipt.bounds=state.bounds.update(request,cancelled);
            if(receipt.bounds.status==OutdoorBoundStatus::cancelled)return abort(OutdoorStatus::cancelled);
            // Carry a completed evaluation, never an empty budget slice. Retain
            // its original clock while fresh; new measured failures can replace
            // it after expiry or a changed source/lease.
            if(receipt.bounds.status==OutdoorBoundStatus::ready){
                const auto& failed=receipt.bounds.support_failure;
                if(!receipt.bounds.ready_base.empty()||!receipt.bounds.endpoints.empty()||
                    std::any_of(receipt.bounds.base_positions.begin(),receipt.bounds.base_positions.end(),[](const ProposedOrder&x){return x.order.pace!=Pace::stand;}))state.support_failure={};
                else if(failed.completed&&(!state.support_failure.completed||
                    !fresh(state.support_failure.stamp.at_ms,w.now_ms,config_.order_lifetime_ms)||state.support_failure.focus!=failed.focus||
                    state.support_failure.actor!=failed.actor||state.support_failure.owner!=failed.owner||state.support_failure.restrictions!=failed.restrictions||
                    distance_squared(state.support_failure.focus_position,failed.focus_position)>policy_.posture.arrived_m*policy_.posture.arrived_m))state.support_failure=failed;
            }
            // Do not approve a mover and simultaneously replace the observed
            // base with an ordinary proposal to walk somewhere else. Retain an
            // exact, currently set base's occupied goal and original fire window.
            std::vector<ProposedOrder> bases;bool base_preserved=false;
            for(auto actor:receipt.bounds.ready_base){
                if(stop())return abort(OutdoorStatus::cancelled);const auto id=key(actor);
                auto b=baseline.find(id);auto a=applied.find(id);auto p=previous.find(id);auto c=covers.find(id);
                if(a==applied.end()||p==previous.end()||c==covers.end()||
                    (b!=baseline.end()&&protected_order(b->second->order)))continue;
                ProposedOrder held;if(b!=baseline.end())held=*b->second;
                else {const auto man=actors.find(id);if(man==actors.end()||
                        !tactical_knowledge_focus(*man->second,local,known,held.knowledge))continue;
                    held.order=first_response(*man->second,config_,w.now_ms,w.level_epoch,w.geometry_revision,p->second->order_seq);
                    held.group_id=element_group->group_id;held.has_knowledge=true;}
                if(!held.has_knowledge||held.knowledge.foe!=receipt.posture.focus||
                    held.knowledge.observed_at_ms!=receipt.posture.focus_fact.at_ms)continue;
                const CoverSpot* spot=nullptr;for(const auto&row:c->second->spots){if(stop())return abort(OutdoorStatus::cancelled);
                    if(row.node==a->second->goal_node&&distance_squared(row.position,a->second->goal_position)==0){spot=&row;break;}}
                if(!spot)continue;
                held.order.kind=OrderKind::hold;held.order.role=roles::base_of_fire;held.order.pace=Pace::stand;
                held.order.goal_node=a->second->goal_node;held.order.goal_position=a->second->goal_position;
                held.order.arrival_radius_m=a->second->arrival_radius_m;held.order.expose=a->second->expose;
                held.order.stance=a->second->stance;held.order.fire=a->second->fire;held.order.look=a->second->look;
                held.order.flags&=~order_face_movement;held.expected_previous_order_seq=p->second->order_seq;
                held.has_cover_evidence=true;held.cover_evidence=*spot;held.navigation_pending=false;
                bases.push_back(std::move(held));base_preserved=true;
            }
            const auto before=out.overrides.size();
            for(const auto&formation:receipt.bounds.base_positions){
                if(stop())return abort(OutdoorStatus::cancelled);const auto id=key(formation.order.actor);
                const auto prev=previous.find(id);const auto base=baseline.find(id);
                if(prev==previous.end()||(base!=baseline.end()&&protected_order(base->second->order)))continue;
                auto proposal=formation;proposal.expected_previous_order_seq=prev->second->order_seq;
                out.overrides.push_back(std::move(proposal));
            }
            for(const auto& recovery:receipt.bounds.support_reposition){
                const auto id=key(recovery.order.actor);const auto prev=previous.find(id);const auto base=baseline.find(id);
                if(prev==previous.end()||(base!=baseline.end()&&protected_order(base->second->order)))continue;
                auto proposal=recovery;proposal.expected_previous_order_seq=prev->second->order_seq;out.overrides.push_back(std::move(proposal));
            }
            const auto movers_before=out.overrides.size();
            for(const auto&endpoint:receipt.bounds.endpoints){
                if(stop())return abort(OutdoorStatus::cancelled);const auto id=key(endpoint.proposal.order.actor);
                const auto prev=previous.find(id);const auto base=baseline.find(id);
                if(prev==previous.end()||(base!=baseline.end()&&protected_order(base->second->order))||(!endpoint.continuing&&!base_preserved))continue;
                auto proposal=endpoint.proposal;
                // Applied evidence and latest published generation are distinct.
                // Runtime publication still checks this exact current generation.
                proposal.expected_previous_order_seq=prev->second->order_seq;
                out.overrides.push_back(std::move(proposal));
            }
            if(out.overrides.size()!=movers_before)out.overrides.insert(out.overrides.begin()+before,bases.begin(),bases.end());
            if(policy_.personal.enabled||policy_.fire_enable||policy_.movement.enabled||policy_.withdrawal_enable){
                std::map<Key,ProposedOrder> selected;
                for(auto actor:s.members){auto b=baseline.find(key(actor));if(b!=baseline.end())selected.emplace(key(actor),*b->second);}
                for(std::size_t n=before;n<out.overrides.size();++n)selected[key(out.overrides[n].order.actor)]=out.overrides[n];
                if(!fair_advanced&&!selected.empty()&&(policy_.fire_enable||policy_.solo.enabled||policy_.self.enabled||policy_.movement.enabled||policy_.withdrawal_enable)){next_after=s.side_id;fair_advanced=true;}
                auto movement_seeds=[&](){
                    std::vector<ProposedOrder> seeds;for(const auto&row:selected)seeds.push_back(row.second);
                    // SquadTactics may omit a man whose accepted leg still runs.
                    // Supply the producer that proven ledger row as a seed, not
                    // as a new order. Only a validated episode refresh below may
                    // enter selected and reach publication.
                    for(const auto&row:receipt.execution.facts.actors){
                        if(stop())return std::vector<ProposedOrder>{};const auto id=key(row.applied.actor);
                        if(selected.count(id))continue;
                        const auto prev=previous.find(id);const auto actor=actors.find(id);
                        if(prev==previous.end()||actor==actors.end())continue;
                        ProposedOrder seed;seed.order=row.applied;seed.expected_previous_order_seq=prev->second->order_seq;
                        seed.group_id=element_group->group_id;
                        const auto cache=covers.find(id);const auto profile=profiles.find(id);const auto mind=members.find(id);
                        if(row.cover_verified==Evidence::yes&&cache!=covers.end()&&profile!=profiles.end()&&mind!=members.end()){
                            const auto&source=profile->second->source;
                            for(const auto&foe:mind->second->foes)if(foe.id==source.focus&&!foe.down)
                                for(const auto&fact:foe.sources)if(fact.valid&&fact.at_ms==source.focus_at_ms&&
                                    fact.at_ms<=w.now_ms&&valid_key(fact.reporter)&&distance_squared(fact.position,source.focus_position)==0){
                                    for(const auto&spot:cache->second->spots)if(spot.node==row.applied.goal_node&&
                                        distance_squared(spot.position,row.applied.goal_position)==0){
                                        seed.has_cover_evidence=true;seed.cover_evidence=spot;seed.has_knowledge=true;
                                        seed.knowledge.receiver=row.applied.actor;seed.knowledge.receiver_ownership_generation=actor->second->ownership_generation;
                                        seed.knowledge.receiver_faction=actor->second->faction;seed.knowledge.foe=source.focus;seed.knowledge.target=source.focus.actor;
                                        seed.knowledge.observer=fact.reporter;seed.knowledge.observed_at_ms=fact.at_ms;
                                        seed.knowledge.position=fact.position;seed.knowledge.shared=fact.reporter!=row.applied.actor;
                                    }
                                }
                        }
                        seeds.push_back(std::move(seed));
                    }
                    return seeds;
                };
                if(policy_.movement.enabled){
                    auto movement_bases=movement_seeds();std::vector<OutdoorSoloRoute> movement_routes;
                    for(const auto&row:movement_bases){if(stop())return abort(OutdoorStatus::cancelled);
                        const auto routes=solo_routes.find(key(row.order.actor));if(routes!=solo_routes.end())movement_routes.insert(movement_routes.end(),routes->second.begin(),routes->second.end());}
                    OutdoorMovementRequest move;move.world=&local;move.side=&s;move.knowledge=&known;move.mission=mi->second;
                    move.posture=&receipt.posture;move.elements=element_group;move.baseline=&movement_bases;move.applied=&own_orders;move.routes=&movement_routes;
                    move.route_service=&capture.route_service;move.route_budget=&route_budget;
                    receipt.movement=state.movement.update(move,services,movement_quota,cancelled);
                    if(receipt.movement.status==OutdoorStatus::cancelled)return abort(OutdoorStatus::cancelled);
                    if(receipt.movement.status==OutdoorStatus::ready)for(const auto&row:receipt.movement.proposals){
                        const auto id=key(row.order.actor);if(roster.count(id)&&previous.find(id)!=previous.end())selected[id]=row;}
                }
                std::map<Key,OutdoorWithdrawalCommitment> withdrawal_commitments;
                if(policy_.withdrawal_enable&&receipt.assessment.status==OutdoorStatus::ready){
                    auto withdrawal_bases=movement_seeds();if(stop())return abort(OutdoorStatus::cancelled);
                    static const std::vector<OutdoorWithdrawalCandidate> no_candidates;
                    const auto candidates=withdrawal_candidates.find(s.side_id);
                    OutdoorWithdrawalRequest withdraw;withdraw.world=&local;withdraw.side=&s;withdraw.knowledge=&known;withdraw.elements=element_group;
                    withdraw.assessment=&receipt.assessment;withdraw.execution=&receipt.execution.facts;withdraw.withdrawal_execution=&receipt.withdrawal_execution;
                    withdraw.accepted_orders=&own_orders;withdraw.base_orders=&withdrawal_bases;
                    withdraw.candidates=candidates==withdrawal_candidates.end()?&no_candidates:&candidates->second;
                    receipt.withdrawal=state.withdrawal.update(withdraw,withdrawal_work,cancelled);
                    if(receipt.withdrawal.status==OutdoorStatus::cancelled)return abort(OutdoorStatus::cancelled);
                    if(receipt.withdrawal.status==OutdoorStatus::ready)for(const auto&row:receipt.withdrawal.orders){
                        const auto id=key(row.proposal.order.actor);if(!roster.count(id)||previous.find(id)==previous.end())continue;
                        selected[id]=row.proposal;
                        OutdoorWithdrawalCommitment commit;commit.level_epoch=w.level_epoch;commit.snapshot_seq=w.sequence;commit.geometry_revision=w.geometry_revision;
                        commit.assessment_serial=row.intent.episode;commit.side_id=s.side_id;commit.actor=row.intent.actor;
                        commit.ownership_generation=row.intent.owner;commit.restrictions_revision=row.intent.restrictions_revision;
                        commit.goal_node=row.intent.node;commit.goal_position=row.intent.position;commit.continuing=row.continuing;
                        const auto ap=applied.find(key(commit.actor));if(ap!=applied.end())commit.applied_order_seq=ap->second->order_seq;
                        withdrawal_commitments.emplace(key(commit.actor),commit);
                    }
                }
                if(!fair_advanced&&!selected.empty()&&(policy_.fire_enable||policy_.solo.enabled||policy_.self.enabled||policy_.movement.enabled||policy_.withdrawal_enable)){
                    next_after=s.side_id;fair_advanced=true;
                }
                {auto progress_bases=movement_seeds();
                    auto progress=outdoor_cover_progress(config_,local,s,known,receipt.posture,focus.leader,own_cover,
                        progress_bases,capture.route_service,route_budget,cancelled);
                    for(auto& proposal:progress)if(selected.count(key(proposal.order.actor)))selected[key(proposal.order.actor)]=std::move(proposal);
                }
                OutdoorPersonalReservations reserved;reserved.level_epoch=w.level_epoch;reserved.snapshot_seq=w.sequence;
                std::map<Key,std::size_t> knowledge_index;
                for(std::size_t n=0;n<known.members.size();++n)knowledge_index.emplace(key(known.members[n].actor),n);
                for(const auto&a:local.actors){
                    auto o=applied.find(key(a.key));OutdoorPersonalReservation row{a.key,a.node_id,a.position};
                    if(o!=applied.end()&&a.last_applied.order_seq==o->second->order_seq&&
                        publishable(*o->second,a,w.now_ms,w.level_epoch,w.geometry_revision)&&a.accessible_current_order&&o->second->pace!=Pace::stand){
                        row.node=o->second->goal_node;row.position=o->second->goal_position;
                    }
                    reserved.members.push_back(row);
                }
                const ActorSnapshot* mover=nullptr;const Order* moving_order=nullptr;
                std::set<Key> continuing;
                for(const auto&endpoint:receipt.bounds.endpoints)if(endpoint.continuing){
                    const auto id=key(endpoint.proposal.order.actor);auto a=actors.find(id);auto o=applied.find(id);
                    if(a!=actors.end()&&o!=applied.end()){
                        continuing.insert(id);if(!mover){mover=a->second;moving_order=o->second;}
                    }
                }
                std::vector<const ActorSnapshot*> personal_order;
                for(const auto&a:local.actors)if(key(a.key)>key(state.solo_after))personal_order.push_back(&a);
                for(const auto&a:local.actors)if(key(a.key)<=key(state.solo_after))personal_order.push_back(&a);
                bool solo_advanced=false,squad_push_taken=false;
                // Applied movement reserves the squad slot even when its actor
                // is outside this pass's optional publication cohort. A proposal
                // or a stale/dead/reowned receipt cannot retain that reservation.
                ActorKey retained_push;
                for(const auto& a:local.actors){
                    const auto row=applied.find(key(a.key));if(row==applied.end())continue;
                    const auto& o=*row->second;
                    if(a.alive&&!a.wounded&&!a.yield_reasons&&(a.flags&flag_admitted)&&
                        a.level_epoch==w.level_epoch&&a.snapshot_seq==w.sequence&&a.observed_at_ms<=w.now_ms&&
                        w.now_ms-a.observed_at_ms<=config_.snapshot_max_age_ms&&
                        o.role==roles::suppression_push&&o.kind==OrderKind::rush&&o.pace!=Pace::stand&&
                        a.last_applied.order_seq==o.order_seq&&a.last_applied.at_ms<=w.now_ms&&
                        publishable(o,a,w.now_ms,w.level_epoch,w.geometry_revision)&&a.accessible_current_order&&
                        distance_squared(a.position,o.goal_position)>o.arrival_radius_m*o.arrival_radius_m){
                        retained_push=a.key;break;
                    }
                }
                auto push_first=std::find_if(personal_order.begin(),personal_order.end(),[&](const ActorSnapshot* a){return a->key==retained_push;});
                if(push_first!=personal_order.end())std::rotate(personal_order.begin(),push_first,push_first+1);
                state.push_actor=retained_push;
                OutdoorEscapeContext escape_context;escape_context.services=&services;
                escape_context.source=receipt.posture.stamp;escape_context.paths=&paths;
                escape_context.route_service=&capture.route_service;escape_context.route_budget=&local_route_budget;
                escape_context.work_left=&solo_quota.work_left;escape_context.queries_left=&solo_quota.queries_left;
                escape_context.work_used=&solo_quota.work_used;escape_context.queries_used=&solo_quota.queries_used;
                if(policy_.personal.enabled)for(const auto*actor:personal_order){
                    const auto&a=*actor;
                    if(stop())return abort(OutdoorStatus::cancelled);
                    const auto id=key(a.key);auto chosen=selected.find(id);if(chosen==selected.end())continue;
                    // Full-side coordination/K1 remain above. Optional per-man
                    // work must have a publication slot in this exact Work;
                    // otherwise verified lean could be discarded by Dispatcher
                    // after consuming the selected men's shared query budget.
                    if(publication_cohort&&!personal_cohort.count(id))continue;
                    OutdoorPersonalInput input;input.base=chosen->second;input.side_id=s.side_id;input.knowledge=&known;input.reservations=&reserved;
                    input.escape_context=&escape_context;input.grenade_memory=&state.grenades[id];
                    for(const auto& fuse:capture.grenade_fuses)if(fuse.actor==a.key){input.grenade_fuse=&fuse;break;}
                    auto kn=knowledge_index.find(id);if(kn!=knowledge_index.end())input.knowledge_index=kn->second;
                    auto ap=applied.find(id);if(ap!=applied.end())input.applied=ap->second;
                    auto cv=proved_covers.find(id);if(cv!=proved_covers.end())input.cover=cv->second;
                    auto pr=profiles.find(id);if(pr!=profiles.end())input.cover_profile=pr->second;
                    const auto withdraw_commit=withdrawal_commitments.find(id);
                    if(withdraw_commit!=withdrawal_commitments.end())input.withdrawal_commitment=&withdraw_commit->second;
                    OutdoorBoundCommitment bound;
                    if(input.applied&&continuing.count(id)){
                        bound.level_epoch=w.level_epoch;bound.snapshot_seq=w.sequence;bound.geometry_revision=w.geometry_revision;
                        bound.side_id=s.side_id;bound.actor=a.key;bound.ownership_generation=a.ownership_generation;
                        bound.restrictions_revision=a.restrictions_revision;bound.applied_order_seq=input.applied->order_seq;input.bound_commitment=&bound;
                    }
                    OutdoorBaseCommitment base;
                    if(input.applied&&mover&&a.key!=mover->key&&std::find(receipt.bounds.ready_base.begin(),receipt.bounds.ready_base.end(),a.key)!=receipt.bounds.ready_base.end()){
                        base.level_epoch=w.level_epoch;base.snapshot_seq=w.sequence;base.geometry_revision=w.geometry_revision;base.side_id=s.side_id;
                        base.base=a.key;base.ownership_generation=a.ownership_generation;base.restrictions_revision=a.restrictions_revision;
                        base.order_seq=input.applied->order_seq;base.assignment_generation=input.applied->assignment.generation;
                        base.mover=mover;base.mover_applied=moving_order;input.base_commitment=&base;
                    }
                    const auto escape_work_before=solo_quota.work_used;
                    auto personal=outdoor_personal_order(config_,policy_.personal,local,a,input,cancelled);
                    if(!solo_advanced&&solo_quota.work_used!=escape_work_before){state.solo_after=a.key;solo_advanced=true;}
                    if(personal.status==OutdoorPersonalStatus::cancelled)return abort(OutdoorStatus::cancelled);
                    // Own contact refreshes aim/fire concurrently with forming
                    // the firing base; a cheap reflex is not a new movement job.
                    // Actual urgent escapes, dogfight, hands and reload win.
                    if(personal.status==OutdoorPersonalStatus::ready&&personal.reason==OutdoorPersonalReason::personal_contact){
                        const bool forming=std::any_of(receipt.bounds.base_positions.begin(),receipt.bounds.base_positions.end(),
                            [&](const ProposedOrder&p){return p.order.actor==a.key;});
                        const bool supporting=std::any_of(bases.begin(),bases.end(),[&](const ProposedOrder&p){return p.order.actor==a.key;});
                        const bool bound_proposal=chosen->second.order.kind==OrderKind::bound&&
                            std::any_of(receipt.bounds.endpoints.begin(),receipt.bounds.endpoints.end(),
                                [&](const OutdoorBoundEndpoint&p){return p.proposal.order.actor==a.key&&!p.continuing;});
                        const bool recovering=std::any_of(receipt.bounds.support_reposition.begin(),receipt.bounds.support_reposition.end(),
                            [&](const ProposedOrder&p){return p.order.actor==a.key&&p.order.goal_node==chosen->second.order.goal_node&&distance_squared(p.order.goal_position,chosen->second.order.goal_position)==0;});
                        if(forming||supporting||bound_proposal||recovering){const auto look=personal.proposal.order.look;const auto fire=personal.proposal.order.fire;
                            personal.proposal=chosen->second;personal.proposal.order.look=look;personal.proposal.order.fire=fire;
                        }
                    }
                    if(personal.status==OutdoorPersonalStatus::ready)chosen->second=personal.proposal;
                    if(policy_.self.enabled&&personal.status==OutdoorPersonalStatus::ready){
                        const auto previous_self=state.self.find(id);
                        auto self=previous_self==state.self.end()?std::make_unique<CombatOutdoorSelf>(config_,policy_.self):previous_self->second->clone(cancelled);
                        if(!self)return abort(stop()?OutdoorStatus::cancelled:OutdoorStatus::failed);
                        OutdoorSelfInput maintenance;maintenance.base=personal.proposal;maintenance.applied=input.applied;
                        maintenance.knowledge=&known;maintenance.knowledge_index=input.knowledge_index;
                        maintenance.genuine_commitment=personal.reason==OutdoorPersonalReason::committed_room||
                            personal.reason==OutdoorPersonalReason::committed_bound||personal.reason==OutdoorPersonalReason::hands||
                            personal.reason==OutdoorPersonalReason::grenade_escape||personal.reason==OutdoorPersonalReason::grenade_wait||
                            personal.reason==OutdoorPersonalReason::committed_withdrawal||personal.acknowledged_base;
                        const auto self_used_before=self_quota.work_used;
                        auto result=self->update(local,a,s,maintenance,self_quota,cancelled);
                        result.work=self_quota.work_used-self_used_before;
                        if(result.status==OutdoorSelfStatus::cancelled)return abort(OutdoorStatus::cancelled);
                        if(!solo_advanced&&result.work){state.solo_after=a.key;solo_advanced=true;}
                        if(result.status==OutdoorSelfStatus::ready){
                            state.self[id]=std::shared_ptr<const CombatOutdoorSelf>(std::move(self));
                            if(result.reload_intent){
                                const auto until=saturated_after(result.pending_started_ms,
                                    result.actual_reloading?policy_.self.cancel_ms:policy_.self.keep_ms);
                                result.proposal.order.expires_at_ms=(std::min)(result.proposal.order.expires_at_ms,until);
                                input.base=result.proposal;input.tactical_reload=true;
                                personal=outdoor_personal_order(config_,policy_.personal,local,a,input,cancelled);
                                if(personal.status==OutdoorPersonalStatus::cancelled)return abort(OutdoorStatus::cancelled);
                                if(personal.status==OutdoorPersonalStatus::ready){
                                    personal.proposal.order.expires_at_ms=(std::min)(personal.proposal.order.expires_at_ms,until);
                                    chosen->second=personal.proposal;
                                }
                            }
                        }
                        receipt.self.push_back(std::move(result));
                    }
                    if(policy_.solo.enabled&&personal.status==OutdoorPersonalStatus::ready){
                        const auto previous_solo=state.solo.find(id);
                        auto solo=previous_solo==state.solo.end()?std::make_unique<CombatOutdoorSolo>(config_,policy_.solo):previous_solo->second->clone(cancelled);
                        if(!solo)return abort(stop()?OutdoorStatus::cancelled:OutdoorStatus::failed);
                        OutdoorSoloInput solo_input;solo_input.personal=personal;solo_input.source=input;
                        solo_input.coordinated_side=receipt.posture.able>=2;
                        solo_input.squad_push_allowed=!squad_push_taken&&(!valid_key(retained_push)||retained_push==a.key)&&solo_input.coordinated_side&&
                            (receipt.posture.posture==OutdoorPosture::attack||receipt.posture.posture==OutdoorPosture::assault);
                        if(solo_input.coordinated_side){
                            const auto& element=std::find(element_group->base.begin(),element_group->base.end(),a.key)!=element_group->base.end()
                                ?element_group->base:element_group->movers;
                            Vec3 centre{};std::size_t count=0;
                            for(auto peer:element){
                                const auto peer_actor=actors.find(key(peer));
                                if(peer_actor==actors.end()||!peer_actor->second->alive||peer_actor->second->wounded||!finite(peer_actor->second->position))continue;
                                centre.x+=peer_actor->second->position.x;centre.y+=peer_actor->second->position.y;centre.z+=peer_actor->second->position.z;++count;
                            }
                            if(count){const float n=float(count);solo_input.element_anchor={centre.x/n,centre.y/n,centre.z/n};solo_input.element_anchor_valid=true;}
                        }
                        solo_input.genuine_commitment=personal.reason==OutdoorPersonalReason::committed_room||
                            personal.reason==OutdoorPersonalReason::committed_bound||personal.reason==OutdoorPersonalReason::hands||
                            personal.reason==OutdoorPersonalReason::grenade_escape||personal.reason==OutdoorPersonalReason::grenade_wait||
                            personal.reason==OutdoorPersonalReason::committed_withdrawal;
                        if(personal.reason==OutdoorPersonalReason::personal_contact||personal.reason==OutdoorPersonalReason::continue_cover||
                            personal.reason==OutdoorPersonalReason::squad)
                            for(const auto&formation:receipt.bounds.base_positions)if(formation.order.actor==a.key&&
                                chosen->second.order.role==roles::base_of_fire&&chosen->second.order.pace!=Pace::stand&&
                                chosen->second.order.goal_node==formation.order.goal_node&&
                                distance_squared(chosen->second.order.goal_position,formation.order.goal_position)==0){
                                solo_input.genuine_commitment=true;break;
                            }
                        if(personal.reason==OutdoorPersonalReason::personal_contact||personal.reason==OutdoorPersonalReason::squad)
                            for(const auto&support:bases)if(support.order.actor==a.key&&chosen->second.order.role==roles::base_of_fire&&
                                chosen->second.order.goal_node==support.order.goal_node&&
                                distance_squared(chosen->second.order.goal_position,support.order.goal_position)==0){
                                solo_input.genuine_commitment=true;break;
                            }
                        // A newly proposed coordinated bound is still subject
                        // to the final support check and native acceptance. Do
                        // not replace it with Solo's ordinary contact fallback
                        // before it can reach either of those authorities.
                        if(personal.reason==OutdoorPersonalReason::personal_contact||personal.reason==OutdoorPersonalReason::squad)
                            for(const auto&endpoint:receipt.bounds.endpoints)if(!endpoint.continuing&&endpoint.proposal.order.actor==a.key&&
                                chosen->second.order.kind==OrderKind::bound&&chosen->second.order.goal_node==endpoint.proposal.order.goal_node&&
                                distance_squared(chosen->second.order.goal_position,endpoint.proposal.order.goal_position)==0){
                                solo_input.genuine_commitment=true;break;
                            }
                        if(personal.reason==OutdoorPersonalReason::personal_contact||personal.reason==OutdoorPersonalReason::squad)
                            for(const auto&recovery:receipt.bounds.support_reposition)if(recovery.order.actor==a.key&&
                                chosen->second.order.kind==OrderKind::advance&&chosen->second.order.goal_node==recovery.order.goal_node&&
                                distance_squared(chosen->second.order.goal_position,recovery.order.goal_position)==0)solo_input.genuine_commitment=true;
                        // An authored investigation is a side job, not a lone
                        // pursuit. Preserve only its exact selected proposal;
                        // personal escapes, hands and reload still take priority.
                        if(personal.reason==OutdoorPersonalReason::squad&&receipt.movement.investigating)
                            for(const auto&move:receipt.movement.proposals)if(move.order.actor==a.key&&
                                move.order.kind==OrderKind::advance&&chosen->second.order.kind==move.order.kind&&
                                chosen->second.order.goal_node==move.order.goal_node&&
                                distance_squared(chosen->second.order.goal_position,move.order.goal_position)==0)
                                solo_input.genuine_commitment=true;
                        solo_input.allow_noncontact=receipt.posture.posture==OutdoorPosture::defend||receipt.posture.posture==OutdoorPosture::contact||
                            receipt.posture.posture==OutdoorPosture::search;
                        const auto route=solo_routes.find(id);if(route!=solo_routes.end())solo_input.routes=&route->second;
                        const auto throwing=solo_throws.find(id);if(throwing!=solo_throws.end())solo_input.grenade_throw=throwing->second;
                        const auto escape=grenade_escapes.find(id);if(escape!=grenade_escapes.end())solo_input.grenade_escape=escape->second;
                        const auto aim=solo_aim.find(id);if(aim!=solo_aim.end())solo_input.aim_evidence=aim->second;
                        const auto lean=solo_lean.find(id);if(lean!=solo_lean.end())solo_input.lean_capture=lean->second;
                        auto result=solo->update(local,a,s,solo_input,services,solo_quota,cancelled);
                        if(result.status==OutdoorSoloStatus::cancelled)return abort(OutdoorStatus::cancelled);
                        if(result.status==OutdoorSoloStatus::ready||result.status==OutdoorSoloStatus::pending){
                            if(result.changed)chosen->second=result.proposal;
                            if(result.proposal.order.role==roles::suppression_push&&result.proposal.order.kind==OrderKind::rush){
                                squad_push_taken=true;state.push_actor=a.key;state.solo_after=a.key;}
                            if(!solo_advanced&&result.work){state.solo_after=a.key;solo_advanced=true;}
                            state.solo[id]=std::shared_ptr<const CombatOutdoorSolo>(std::move(solo));
                        }
                        receipt.solo.push_back(std::move(result));
                    }
                    // Price later men's escape cells against goals already
                    // chosen in this pass, not only the previous frame's slots.
                    // This is a soft reservation; it never blocks an emergency
                    // just because every available destination is crowded.
                    const auto& final_order=chosen->second.order;
                    if(final_order.kind!=OrderKind::yield&&final_order.goal_node!=invalid_node&&finite(final_order.goal_position))
                        for(auto& reservation:reserved.members)if(reservation.actor==a.key){
                            reservation.node=final_order.goal_node;reservation.position=final_order.goal_position;break;
                        }
                    receipt.personal.push_back(std::move(personal));
                }
                for(auto it=state.solo.begin();it!=state.solo.end();)if(!roster.count(it->first))it=state.solo.erase(it);else ++it;
                for(auto it=state.grenades.begin();it!=state.grenades.end();)if(!roster.count(it->first))it=state.grenades.erase(it);else ++it;
                for(auto it=state.self.begin();it!=state.self.end();)if(!roster.count(it->first))it=state.self.erase(it);else ++it;
                if(policy_.fire_enable){
                    if(!fair_advanced&&!selected.empty()){next_after=s.side_id;fair_advanced=true;}
                    auto firing=state.fire?state.fire->clone(cancelled):std::make_unique<CombatOutdoorFire>(config_,policy_.fire);
                    if(!firing)return abort(stop()?OutdoorStatus::cancelled:OutdoorStatus::failed);
                    OutdoorFireRequest fire;fire.applied_orders=own_orders;fire.route_service=&capture.route_service;fire.route_budget=&local_route_budget;
                    for(const auto&row:selected){auto order=row.second.order;
                        // Publication also stamps the source time and may only
                        // tighten the existing lifetime/fire deadline.
                        order.issued_at_ms=w.now_ms;order.snapshot_seq=w.sequence;fire.base_orders.push_back(order);
                    }
                    for(const auto&eye:capture.eyes){if(stop())return abort(OutdoorStatus::cancelled);
                        if(!roster.count(key(eye.actor))||!eye.measured||!finite(eye.eye))continue;
                        OutdoorFireEye row;row.source.actor=eye.actor;row.source.level_epoch=eye.epoch;row.source.snapshot_seq=eye.sequence;
                        row.source.geometry_revision=w.geometry_revision;row.source.owner=eye.owner;row.source.restrictions=eye.restrictions;
                        row.source.at_ms=eye.observed_at_ms;row.source.position=eye.feet;row.measured_eye=eye.eye;
                        auto a=actors.find(key(eye.actor));if(a==actors.end())continue;row.source.origin_node=a->second->node_id;
                        fire.eyes.push_back(row);
                    }
                    if(source_matches(capture.source,w))for(const auto&step:capture.buddy_steps){if(stop())return abort(OutdoorStatus::cancelled);
                        if(roster.count(key(step.source.actor)))fire.steps.push_back(step);}
                    receipt.fire=firing->update(local,s,known,fire,services,fire_quota,cancelled);
                    if(receipt.fire.status==OutdoorFireStatus::cancelled)return abort(OutdoorStatus::cancelled);
                    if(receipt.fire.status==OutdoorFireStatus::ready||receipt.fire.status==OutdoorFireStatus::pending){
                        for(const auto&row:receipt.fire.proposals)if(row.changed){auto chosen=selected.find(key(row.order.actor));if(chosen==selected.end())continue;
                            auto danger=state.grenades.find(key(row.order.actor));if(danger!=state.grenades.end()&&danger->second.until>w.now_ms)continue;
                            chosen->second.order=row.order;chosen->second.point_fire=row.point_fire;
                            if(row.movement_changed){chosen->second.has_cover_evidence=false;chosen->second.navigation_pending=false;}
                        }
                        out.executed_suppression.insert(out.executed_suppression.end(),receipt.fire.executed_suppression.begin(),receipt.fire.executed_suppression.end());
                        state.fire=std::shared_ptr<const CombatOutdoorFire>(std::move(firing));
                    }
                }
                // A fresh advance cannot outlive its base's personal escape,
                // reload or loss of firing support in this very same pass.
                bool supporting=false;
                for(const auto&base:bases){auto chosen=selected.find(key(base.order.actor));if(chosen==selected.end())continue;const auto&o=chosen->second.order;
                    if(o.kind==OrderKind::hold&&o.role==roles::base_of_fire&&o.pace==Pace::stand&&o.fire.allowed&&o.fire.until_ms>w.now_ms&&
                        ((o.look.kind==LookKind::enemy&&o.look.target==receipt.posture.focus.actor)||
                            (o.fire.at_valid&&finite(o.fire.at)&&distance_squared(o.fire.at,receipt.posture.focus_fact.position)==0))&&
                        o.goal_node==base.order.goal_node&&distance_squared(o.goal_position,base.order.goal_position)==0)supporting=true;
                }
                if(!supporting)for(const auto&endpoint:receipt.bounds.endpoints)if(!endpoint.continuing){
                    const auto id=key(endpoint.proposal.order.actor);auto chosen=selected.find(id);auto original=baseline.find(id);
                    if(chosen!=selected.end()&&chosen->second.order.kind==OrderKind::bound){
                        if(original!=baseline.end())chosen->second=*original->second;else selected.erase(chosen);
                    }
                }
                // Holding a losing exchange is a side tactic, not a one-pass
                // hint that a later personal rush may replace with an exit.
                // Choose from the same measured worker cover/routes after the
                // personal/fire decorators; urgent grenade/withdrawal wins.
                if(receipt.posture.exchange_hold&&receipt.posture.posture==OutdoorPosture::defend){
                    std::vector<Vec3> defensive_goals;
                    for(const auto& a:local.actors){
                        if(stop())return abort(OutdoorStatus::cancelled);
                        const auto id=key(a.key);auto chosen=selected.find(id);if(chosen==selected.end())continue;
                        const auto grenade=state.grenades.find(id);
                        if(grenade!=state.grenades.end()&&grenade->second.until>w.now_ms)continue;
                        const auto cache=proved_covers.find(id);const auto profile=profiles.find(id);
                        // An unchanged squad job need not carry Solo's trace.
                        // Recover it from current K1 only when it exactly
                        // matches the native cover query's threat witness.
                        if(profile!=profiles.end()){
                            KnowledgeTrace trace;
                            if(tactical_knowledge_focus(a,local,known,trace)&&trace.foe==profile->second->source.focus&&
                                trace.observed_at_ms==profile->second->source.focus_at_ms&&
                                distance_squared(trace.position,profile->second->source.focus_position)==0){
                                chosen->second.has_knowledge=true;chosen->second.knowledge=trace;
                            }
                        }
                        if(outdoor_building_hold(config_,local,a,receipt.posture,
                            cache==proved_covers.end()?nullptr:cache->second,profile==profiles.end()?nullptr:profile->second,
                            paths,services,defensive_goals,movement_quota,chosen->second,cancelled))
                            defensive_goals.push_back(chosen->second.order.goal_position);
                    }
                }
                out.overrides.resize(before);
                for(auto&row:selected){auto old=previous.find(row.first);if(old==previous.end())continue;
                    row.second.expected_previous_order_seq=old->second->order_seq;out.overrides.push_back(std::move(row.second));
                }
            }
            out.sides.push_back(std::move(receipt));
        }
        // The wounded are intentionally absent from K1's fighting focus. Run
        // this final own-sight selector even when posture had no live focus or
        // SquadTactics omitted a man, including uncommitted room assignments.
        if(!apply_wounded_finishers(w,config_,plans,current,out.overrides,cancelled))return abort(OutdoorStatus::cancelled);
        for(auto it=next.begin();it!=next.end();)if(!active.count(it->first))it=next.erase(it);else ++it;
        if(stop())return abort(OutdoorStatus::cancelled);states_=std::move(next);after_side_=next_after;out.route_work=route_budget.work_used+local_route_budget.work_used;out.status=OutdoorStatus::ready;return out;
    }catch(...){return abort(OutdoorStatus::failed);}
}
}}}
