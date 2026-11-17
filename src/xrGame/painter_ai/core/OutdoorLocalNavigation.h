#pragma once
#include "CombatRouteProof.h"
#include <algorithm>
#include <cmath>

namespace painter { namespace combat { namespace core {
// Shared actor-local producer. Every native query is charged to the caller's
// frame quota; an actor-restricted full route is distinct from a static ray.
struct OutdoorLocalNavigation {
    const Config& config;const WorldSnapshot& world;const ActorSnapshot& actor;
    const EngineServices& services;const RouteProofService* routes;RouteProofBudget* route_budget;
    std::size_t *work_left,*queries_left,*work_used,*queries_used;
    const std::function<bool()>& cancelled;
    bool stop()const{return cancelled&&cancelled();}
    bool work(){if(stop()||!work_left||!*work_left)return false;--*work_left;if(work_used)++*work_used;return true;}
    bool query(){if(!work()||!queries_left||!*queries_left)return false;--*queries_left;if(queries_used)++*queries_used;return true;}
    std::uint32_t project(Vec3 requested,Vec3& goal){
        const auto&p=config.outdoor.p.solo;
        if(!finite(requested)||!services.project||!query())return invalid_node;
        const auto node=services.project(requested,p.project_horizontal_m,p.project_vertical_m,&goal);
        if(stop()||node==invalid_node||!finite(goal)||std::abs(goal.y-requested.y)>p.project_vertical_m||
           (goal.x-requested.x)*(goal.x-requested.x)+(goal.z-requested.z)*(goal.z-requested.z)>p.project_horizontal_m*p.project_horizontal_m)return invalid_node;
        return node;
    }
    Evidence line(Vec3 from,Vec3 to){
        if(!finite(from)||!finite(to)||!services.rays||!query())return Evidence::unknown;
        Segment ray{from,to};float hit=-2;services.rays(&ray,&hit,1);
        if(stop()||!std::isfinite(hit))return Evidence::unknown;
        if(hit==-1||hit>=std::sqrt(distance_squared(from,to)))return Evidence::yes;
        return hit>=0?Evidence::no:Evidence::unknown;
    }
    bool route(std::uint32_t node,Vec3 goal,float maximum,RouteProofReceipt& proof,float& length){
        if(node==invalid_node||!finite(goal)||!route_budget||!work())return false;
        if(actor.last_rejected.node==node&&actor.last_rejected.restrictions_revision==actor.restrictions_revision&&
           (actor.last_rejected.reason==reject_goal_invalid||actor.last_rejected.reason==reject_goal_inaccessible))return false;
        const auto source=route_proof_source(world,actor,node,goal);
        proof=request_route_proof(routes,source,*route_budget,cancelled);
        if(stop()||!route_proof_complete(proof,source))return false;
        length=0;for(std::size_t n=1;n<proof.points.size();++n){if(!work())return false;
            length+=std::sqrt(distance_squared(proof.points[n-1],proof.points[n]));}
        return std::isfinite(length)&&length<=maximum;
    }
    // Measured complete route nodes stay in the current room/outdoor region.
    // Unknown building data does not certify a doorway/red-safe buddy step.
    bool same_region(const RouteProofReceipt& proof){
        const auto*g=services.buildings;
        if(!g||g->level_epoch!=world.level_epoch||g->map_hash!=world.map_hash||!g->locate||!work())return false;
        const auto here=g->locate(actor.node_id);
        for(auto node:proof.nodes){if(!work()||g->locate(node)!=here)return false;}
        return !stop();
    }
    bool doorway_clear(Vec3 goal){
        const auto*g=services.buildings;
        if(!g||g->level_epoch!=world.level_epoch||g->map_hash!=world.map_hash||!g->locate)return false;
        for(const auto&b:g->buildings)for(const auto&door:b.portals){if(!work())return false;
            if(!door.doorway||!finite(door.centre)||!finite(door.normal)||goal.y<door.y0-config.room.floor_tolerance_m||goal.y>door.y1)continue;
            const float norm=std::hypot(door.normal.x,door.normal.z);if(!(norm>0)||!std::isfinite(norm))return false;
            const float nx=door.normal.x/norm,nz=door.normal.z/norm;
            const float forward=(goal.x-door.centre.x)*nx+(goal.z-door.centre.z)*nz;
            const float side=(goal.x-door.centre.x)*-nz+(goal.z-door.centre.z)*nx;
            if(std::abs(forward)<=door.wall_depth+config.room.entry_lane_depth_m&&
               std::abs(side)<=door.half_width+config.room.entry_lane_half_m)return false;
        }return !stop();
    }
    bool red_clear(std::uint32_t node){
        const auto*g=services.buildings;if(!g||!g->lit_complete||!g->locate||!work())return false;
        const auto here=g->locate(node);
        for(const auto&b:g->buildings){if(!work())return false;if(b.id!=here.first)continue;
            for(const auto&door:b.portals){if(!work())return false;
                const auto*table=door.room_a==here.second?&door.lit_a:door.room_b==here.second?&door.lit_b:nullptr;
                if(!table)continue;auto it=std::lower_bound(table->begin(),table->end(),node,[](const Lit&v,std::uint32_t n){return v.node<n;});
                if(it!=table->end()&&it->node==node&&it->standing)return false;
            }
        }return !stop();
    }
};
}}}
