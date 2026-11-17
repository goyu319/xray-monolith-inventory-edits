#pragma once
#include "CombatMissionIntel.h"
#include "CombatOutdoorPosture.h"
#include "CombatRouteProof.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace painter { namespace combat { namespace core {
// A handover is a planning transition, not permission to walk through a door.
// Still require a current, authority-certified route to a real outside portal
// approach; RoomStack independently checks each member's actual slot afterwards.
inline void measure_mission_door(const Config& config,const WorldSnapshot& world,
    const CombatMissionIntel& intel,ActorKey leader,const EngineServices& services,
    const RouteProofService& routes,RouteProofBudget& budget,OutdoorExecutionFacts& facts,
    const std::function<bool()>& stop) {
    const auto* cache=services.buildings;
    if(intel.pending!=MissionKind::clear_building||!cache||!cache->locate||!services.project||
        cache->level_epoch!=world.level_epoch||cache->map_hash!=world.map_hash)return;
    const auto actor=std::find_if(world.actors.begin(),world.actors.end(),
        [&](const ActorSnapshot& a){return a.key==leader&&a.alive&&!a.wounded;});
    if(actor==world.actors.end())return;
    const auto building=std::find_if(cache->buildings.begin(),cache->buildings.end(),
        [&](const Building& b){return b.id==intel.pending_building;});
    if(building==cache->buildings.end())return;
    struct Candidate{const Portal* portal;float distance;};
    std::vector<Candidate> candidates;
    for(const auto& portal:building->portals) {
        if(stop&&stop())return;
        if(!portal.doorway||!portal.traversable||(portal.room_a==invalid_room)==(portal.room_b==invalid_room)||
            !finite(portal.centre)||!finite(portal.normal))continue;
        const float d=distance_squared(actor->position,portal.centre);
        if(d<=config.outdoor.r.doors.radius_m*config.outdoor.r.doors.radius_m)candidates.push_back({&portal,d});
    }
    std::sort(candidates.begin(),candidates.end(),[](const Candidate& a,const Candidate& b){
        return a.distance!=b.distance?a.distance<b.distance:a.portal->id<b.portal->id;});
    std::size_t tried=0;
    for(const auto& candidate:candidates) {
        if((stop&&stop())||!budget.work_left||tried++>=config.outdoor.r.doors.queries_per_capture)return;
        const auto& p=*candidate.portal;Vec3 goal=p.centre;
        const float norm=std::hypot(p.normal.x,p.normal.z);if(norm<=0)continue;
        const float outward=p.room_a!=invalid_room?1.f:-1.f;
        goal.x+=outward*p.normal.x/norm*config.room.wall_offset_m;
        goal.z+=outward*p.normal.z/norm*config.room.wall_offset_m;
        goal.y=p.y1>p.y0?p.y0:p.centre.y;Vec3 snapped{};
        const auto node=services.project(goal,config.room.projection_m,config.room.floor_tolerance_m,&snapped);
        if(node==invalid_node||!finite(snapped)||std::abs(snapped.y-goal.y)>config.room.floor_tolerance_m||
            std::hypot(snapped.x-goal.x,snapped.z-goal.z)>config.room.projection_m)continue;
        const auto place=cache->locate(node);
        if(place.first==building->id&&place.second!=invalid_room)continue;
        const auto request=route_proof_source(world,*actor,node,snapped);
        const auto receipt=request_route_proof(&routes,request,budget,stop);
        if(!route_proof_complete(receipt,request))continue;
        facts.door_validated=Evidence::yes;facts.door_building=building->id;facts.door_portal=p.id;
        facts.door_node=node;facts.door_point=snapped;return;
    }
}
}}}
