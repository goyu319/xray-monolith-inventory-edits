#pragma once
#include "CombatOutdoorMovement.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace painter { namespace combat { namespace core {
// A defensive destination selector over existing, current worker evidence.
// It issues no cover search and never treats an endpoint as a safe full route.
// The exchange assessment chooses the tactic; native safety/escape still wins.
inline bool outdoor_building_hold(const Config& cfg,const WorldSnapshot& w,const ActorSnapshot& a,
    const OutdoorPostureResult& posture,const ActorCoverCache* cache,const OutdoorCoverProfile* profile,
    const std::vector<OutdoorBoundPath>& paths,const EngineServices& svc,
    const std::vector<Vec3>& reserved,OutdoorMovementQuota& quota,ProposedOrder& proposal,
    const std::function<bool()>& cancelled={}) {
    if(!posture.exchange_hold||posture.posture!=OutdoorPosture::defend||posture.defended_building==invalid_room)return false;
    const auto& previous=proposal.order;
    if(previous.kind==OrderKind::yield||previous.kind==OrderKind::withdraw||previous.kind==OrderKind::entry||
        previous.kind==OrderKind::recover||previous.throw_order.valid||valid_key(previous.aid)||
        (previous.flags&order_committed_entry)||a.throw_state.phase==1||a.throw_state.phase==2||
        a.aid_state.phase==1||a.aid_state.phase==2||!a.alive||a.wounded||a.yield_reasons||
        !a.current_position_accessible||!finite(a.position)||a.node_id==invalid_node)return false;
    const auto* geometry=svc.buildings;
    if(!geometry||geometry->level_epoch!=w.level_epoch||geometry->map_hash!=w.map_hash||!geometry->locate)return false;
    const auto location=geometry->locate(a.node_id);
    if(location.first!=posture.defended_building||location.second==invalid_room)return false;
    const Building* building=nullptr;const Room* room=nullptr;
    for(const auto& b:geometry->buildings)if(b.id==location.first){building=&b;for(const auto& r:b.rooms)if(r.id==location.second){room=&r;break;}break;}
    if(!building||!room)return false;
    const auto in_room=[&](Vec3 p){return finite(p)&&p.x>=room->min.x&&p.x<=room->max.x&&p.z>=room->min.z&&p.z<=room->max.z&&
        std::fabs(p.y-room->floor_y)<=cfg.room.floor_tolerance_m;};
    if(!in_room(a.position))return false;
    const auto work=[&](){if(cancelled&&cancelled())return false;if(!quota.work_left)return false;--quota.work_left;++quota.work_used;return true;};
    const auto flat=[](Vec3 x,Vec3 y){return std::hypot(x.x-y.x,x.z-y.z);};
    const bool stamped=cache&&profile&&cache->actor==a.key&&cache->level_epoch==w.level_epoch&&cache->snapshot_seq==w.sequence&&
        cache->ownership_generation==a.ownership_generation&&cache->restrictions_revision==a.restrictions_revision&&
        cache->origin_node==a.node_id&&cache->observed_at_ms==w.now_ms&&
        profile->source.actor==a.key&&profile->source.level_epoch==w.level_epoch&&profile->source.snapshot_seq==w.sequence&&
        profile->source.geometry_revision==w.geometry_revision&&profile->source.ownership_generation==a.ownership_generation&&
        profile->source.restrictions_revision==a.restrictions_revision&&profile->source.origin_node==a.node_id&&
        profile->source.at_ms==a.observed_at_ms&&distance_squared(profile->source.actor_position,a.position)==0&&
        proposal.has_knowledge&&proposal.knowledge.foe==profile->source.focus&&
        proposal.knowledge.observed_at_ms==profile->source.focus_at_ms&&
        distance_squared(proposal.knowledge.position,profile->source.focus_position)==0;
    struct Choice {const CoverSpot* spot;const Portal* opening;float cost;};
    std::vector<Choice> choices;
    if(stamped)for(const auto& spot:cache->spots){
        if(!work())break;
        if(spot.node==invalid_node||!in_room(spot.position)||geometry->locate(spot.node)!=location||
            spot.geometry_revision!=w.geometry_revision||spot.protection_known!=Evidence::yes||
            spot.threat_key!=profile->source.focus.actor||spot.threat_observed_at_ms!=profile->source.focus_at_ms||
            spot.accessible==Evidence::no||spot.reachable==Evidence::no||
            (spot.restrictions_revision&&spot.restrictions_revision!=a.restrictions_revision))continue;
        bool protected_pose=false;for(std::size_t h=0;h<profile->heights.size();++h)
            protected_pose|=profile->heights[h]>0&&std::isfinite(profile->heights[h])&&spot.blocked[h];
        if(!protected_pose)continue;
        const bool staying=spot.node==a.node_id&&flat(spot.position,a.position)<=cfg.room.arrival_m;
        bool route=staying;float length=flat(a.position,spot.position);
        if(!route)for(const auto& path:paths){
            if(!work())break;
            if(path.actor!=a.key||path.owner!=a.ownership_generation||path.restrictions_revision!=a.restrictions_revision||
                path.origin_node!=a.node_id||path.goal_node!=spot.node||distance_squared(path.goal_position,spot.position)!=0||
                path.complete!=Evidence::yes||path.points.empty())continue;
            route=true;length=0;Vec3 last=a.position;
            // This convex room test covers every segment between the measured
            // route nodes, including start/end. No route through outdoors passes.
            for(auto point:path.points){if(!work()||!in_room(point)){route=false;break;}length+=flat(last,point);last=point;}
            length+=flat(last,spot.position);break;
        }
        if(!route)continue;
        for(const auto& opening:building->portals){
            if(!work())break;
            if((opening.room_a!=room->id&&opening.room_b!=room->id)||!finite(opening.centre)||!finite(opening.normal)||
                !(opening.half_width>0)||!std::isfinite(opening.wall_depth))continue;
            const float normal=std::hypot(opening.normal.x,opening.normal.z);if(!(normal>0))continue;
            const float dx=spot.position.x-opening.centre.x,dz=spot.position.z-opening.centre.z;
            const float across=(-dx*opening.normal.z+dz*opening.normal.x)/normal;
            const float depth=(dx*opening.normal.x+dz*opening.normal.z)/normal;
            float cost=length*cfg.room.approach_weight;
            if(std::fabs(across)<=opening.half_width+cfg.room.entry_zone_half_m&&std::fabs(depth)<=cfg.room.entry_zone_depth_m)
                cost+=cfg.room.funnel_cost_m;
            for(auto occupied:reserved)if(flat(occupied,spot.position)<cfg.room.spacing_m)cost+=cfg.room.funnel_cost_m;
            const auto& lit=opening.room_a==room->id?opening.lit_a:opening.lit_b;
            const auto row=std::lower_bound(lit.begin(),lit.end(),spot.node,[](const Lit& x,std::uint32_t n){return x.node<n;});
            if(row!=lit.end()&&row->node==spot.node)cost+=cfg.room.exposure_cost_m*float(row->crouched)/255.f;
            else if(!geometry->lit_complete)cost+=cfg.room.unknown_cost_m;
            if(posture.focus_valid)cost+=flat(opening.centre,posture.focus_fact.position)*cfg.room.approach_weight;
            if(std::isfinite(cost))choices.push_back({&spot,&opening,cost});
        }
    }
    std::stable_sort(choices.begin(),choices.end(),[](const Choice& x,const Choice& y){return x.cost<y.cost;});
    const Choice* best=nullptr;
    for(const auto& candidate:choices){
        if(!svc.rays||!quota.queries_left||!work())break;
        if(!std::isfinite(candidate.opening->y0)||!std::isfinite(candidate.opening->y1)||candidate.opening->y1<=candidate.opening->y0)continue;
        Segment ray{candidate.spot->position,candidate.opening->centre};ray.a.y+=cfg.room.look_height_m;
        ray.b.y=std::clamp(ray.a.y,candidate.opening->y0,candidate.opening->y1);
        if(!finite(ray.b))continue;
        --quota.queries_left;++quota.queries_used;float hit=std::numeric_limits<float>::quiet_NaN();
        svc.rays(&ray,&hit,1);if(cancelled&&cancelled())return false;
        if(hit==-1){best=&candidate;break;}
    }
    // If a fresh under-fire escape has no proven inside alternative yet, keep
    // its running escape instead of stopping the man in the incoming fire.
    if(!best&&(previous.flags&order_urgent))return false;
    auto order=first_response(a,cfg,w.now_ms,w.level_epoch,w.geometry_revision,proposal.expected_previous_order_seq);
    if(order.kind==OrderKind::yield)return false;
    // A pending search keeps an inside current position without claiming it is
    // covered. New movement is published only with real cover/route evidence.
    const bool reload=order.kind==OrderKind::reload;
    order.kind=reload?OrderKind::reload:OrderKind::hold;order.role=roles::barricade_guard;
    order.flags&=~(order_first_response|order_face_movement|order_sprint);
    order.goal_node=best?best->spot->node:a.node_id;order.goal_position=best?best->spot->position:a.position;
    order.arrival_radius_m=cfg.room.arrival_m;order.assignment={};order.portal={};order.finish={};
    const bool moving=flat(a.position,order.goal_position)>cfg.room.arrival_m;
    order.pace=moving?Pace::walk:Pace::stand;order.speed_scale=cfg.outdoor.p.movement.aimed_speed;
    if(moving&&(previous.flags&order_urgent)){order.pace=Pace::run;order.speed_scale=previous.speed_scale;order.flags|=order_urgent;}
    if(best)order.stance=best->spot->blocked[2]?Stance::stand:Stance::crouch;
    if(!moving&&previous.goal_node==order.goal_node&&distance_squared(previous.goal_position,order.goal_position)==0)
        order.lean=previous.lean;
    if(order.look.kind!=LookKind::enemy){order.fire={};if(best){order.look={LookKind::point,best->opening->centre,{}};
        order.look.value.y=std::clamp(a.position.y+cfg.room.look_height_m,best->opening->y0,best->opening->y1);}}
    proposal.order=order;proposal.point_fire={};proposal.has_cover_evidence=best!=nullptr;
    proposal.navigation_pending=false;
    if(best)proposal.cover_evidence=*best->spot;else proposal.cover_evidence={};
    return true;
}
}}}
