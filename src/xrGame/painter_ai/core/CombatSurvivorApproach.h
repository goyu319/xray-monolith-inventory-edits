#pragma once
#include "CombatSurvivorCooperation.h"
#include "CombatRouteProof.h"
#include "CombatKnowledge.h"
#include "SquadTactics.h"
#include <algorithm>
#include <cmath>

namespace painter { namespace combat { namespace core {
// Quiet, source-qualified meeting legs. Projected cells, the actual actor's
// restricted route proof, and final publisher validation all have to agree.
// This never overrides a room assignment, casualty aid, visible fight, or an
// existing movement order. If any primitive is absent, no approach is issued.
inline std::vector<ProposedOrder> survivor_approach_orders(const WorldSnapshot& w,
    const KnowledgeFrame& k,const std::vector<SurvivorCooperation>& links,
    const std::vector<ProposedOrder>& base,const std::vector<Order>& current,
    const Config& config,const EngineServices& services,const RouteProofService& routes,
    RouteProofBudget& budget,const std::function<bool()>& stop={}){
    std::vector<ProposedOrder> out;
    if(!services.project||!routes.query||!budget.work_left||!k.accepted||k.cancelled||
        k.level_epoch!=w.level_epoch||k.sequence!=w.sequence)return out;
    auto actor=[&](ActorKey key)->const ActorSnapshot*{for(const auto& a:w.actors)if(a.key==key)return &a;return nullptr;};
    auto side=[&](std::uint32_t id)->const SideSnapshot*{for(const auto& s:w.sides)if(s.side_id==id)return &s;return nullptr;};
    for(const auto& link:links){if(stop&&stop())return {};
        if(!link.approaching)continue;
        const auto* first=side(link.first),*second=side(link.second);
        if(!first||!second||first->faction!=second->faction)continue;
        const Vec3 centre{(link.first_anchor.x+link.second_anchor.x)*.5f,
            (link.first_anchor.y+link.second_anchor.y)*.5f,(link.first_anchor.z+link.second_anchor.z)*.5f};
        if(!finite(centre))continue;
        std::vector<ActorKey> participants;participants.insert(participants.end(),first->members.begin(),first->members.end());
        participants.insert(participants.end(),second->members.begin(),second->members.end());
        if(participants.size()>8)continue;
        // Crossing an uninspected doorway is not a social shortcut. Current
        // geometry must place everyone in the same room or all outside.
        bool room_ok=true;std::pair<std::uint32_t,std::uint32_t> place{invalid_room,invalid_room};
        if(services.buildings&&services.buildings->locate){bool first_place=true;
            for(auto key:participants){const auto* a=actor(key);if(!a||a->node_id==invalid_node){room_ok=false;break;}
                const auto located=services.buildings->locate(a->node_id);
                if(first_place){place=located;first_place=false;}else if(located!=place){room_ok=false;break;}}
        }
        if(!room_ok)continue;
        std::vector<Vec3> reserved;
        for(std::size_t index=0;index<participants.size()&&budget.work_left;++index){if(stop&&stop())return {};
            const auto key=participants[index];const auto* a=actor(key);if(!a||!a->alive||a->wounded||
                !(a->flags&flag_admitted)||a->yield_reasons||!a->ownership_generation||
                !fresh(a->observed_at_ms,w.now_ms,config.snapshot_max_age_ms)||
                a->visible_enemies.count||a->enemy.visible||a->hits.count||a->near_misses.count)continue;
            const auto mind=std::find_if(k.members.begin(),k.members.end(),[&](const MemberKnowledge& m){return
                m.actor==key&&m.ownership_generation==a->ownership_generation&&(m.side==first->side_id||m.side==second->side_id);});
            if(mind==k.members.end()||mind->threat_valid||!mind->alive||!mind->able)continue;
            const auto proposed=std::find_if(base.begin(),base.end(),[&](const ProposedOrder& row){return row.order.actor==key;});
            const bool idle_defender=proposed!=base.end()&&proposed->order.kind==OrderKind::hold_door&&
                (proposed->order.role==roles::hold||proposed->order.role==roles::barricade_guard)&&
                !(proposed->order.flags&order_committed_entry);
            if(proposed==base.end()||(proposed->order.kind!=OrderKind::hold&&!idle_defender)||
                (proposed->order.assignment.generation&&!idle_defender)||
                proposed->order.fire.allowed||valid_key(proposed->order.aid)||proposed->order.throw_order.valid||
                proposed->order.portal.valid||(proposed->order.flags&order_committed_entry))continue;
            const auto prior=std::find_if(current.begin(),current.end(),[&](const Order& row){return row.actor==key;});
            if(prior!=current.end()&&prior->kind!=OrderKind::hold&&prior->kind!=OrderKind::hold_door&&prior->kind!=OrderKind::regroup)continue;
            // At most eight projected cells per actor. Every candidate is
            // inside a 1.15 m ring around the mutual meeting centre.
            constexpr float offsets[8][2]={{1.15f,0},{.81f,.81f},{0,1.15f},{-.81f,.81f},
                {-1.15f,0},{-.81f,-.81f},{0,-1.15f},{.81f,-.81f}};
            for(std::size_t attempt=0;attempt<8&&budget.work_left;++attempt){if(stop&&stop())return {};
                const auto& delta=offsets[(index+attempt)%8];Vec3 wanted{centre.x+delta[0],centre.y,centre.z+delta[1]},snapped{};
                --budget.work_left;++budget.work_used;
                const auto node=services.project(wanted,config.room.projection_m,config.room.floor_tolerance_m,&snapped);
                if(node==invalid_node||!finite(snapped)||std::abs(snapped.y-wanted.y)>config.room.floor_tolerance_m||
                    std::hypot(snapped.x-wanted.x,snapped.z-wanted.z)>config.room.projection_m||
                    std::hypot(snapped.x-centre.x,snapped.z-centre.z)>1.5f)continue;
                if(services.buildings&&services.buildings->locate&&services.buildings->locate(node)!=place)continue;
                if(a->last_rejected.node==node&&a->last_rejected.restrictions_revision==a->restrictions_revision)continue;
                bool occupied=false;for(const auto& peer:w.actors)if(peer.key!=key&&peer.alive&&
                    std::hypot(peer.position.x-snapped.x,peer.position.z-snapped.z)<.7f)occupied=true;
                for(const auto& point:reserved)if(std::hypot(point.x-snapped.x,point.z-snapped.z)<.7f)occupied=true;
                if(occupied)continue;
                const auto source=route_proof_source(w,*a,node,snapped);
                const auto proof=request_route_proof(&routes,source,budget,stop);
                if(proof.status==RouteProofStatus::cancelled)return {};
                if(!route_proof_complete(proof,source))continue;
                auto row=*proposed;auto& order=row.order;
                order.kind=OrderKind::regroup;order.role=roles::travel;order.goal_node=node;order.goal_position=snapped;
                order.arrival_radius_m=.45f;order.pace=Pace::walk;order.stance=Stance::stand;order.look={};
                order.fire={};order.lean=0;order.flags=order_survivor_meet;order.portal={};order.throw_order={};order.aid={};order.finish={};
                order.assignment={};order.gait=Gait::lowered;order.speed_scale=1.f;
                row.group_id=mind->side;row.has_cover_evidence=false;row.has_knowledge=false;row.navigation_pending=false;
                out.push_back(std::move(row));reserved.push_back(snapped);break;
            }
        }
    }
    return out;
}
}}}
