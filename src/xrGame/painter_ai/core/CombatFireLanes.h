#pragma once

#include "CombatRouteProof.h"
#include "SquadTactics.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace painter { namespace combat { namespace core {

// A late, room-only composition pass. It changes neither a member's assignment
// nor the fire gate: the engine still checks the actual friend ray each frame.
struct FireLaneChange {ActorKey actor,partner;bool low=false,lateral=false;};
struct FireLaneResult {std::vector<FireLaneChange> changes;std::size_t route_work=0;bool cancelled=false;};
namespace fire_lane_detail {
inline float flat2(Vec3 a,Vec3 b){const float x=a.x-b.x,z=a.z-b.z;return x*x+z*z;}
inline float dot2(Vec3 a,Vec3 b){return a.x*b.x+a.z*b.z;}
inline Vec3 sub2(Vec3 a,Vec3 b){return {a.x-b.x,0,a.z-b.z};}
inline Vec3 add2(Vec3 a,Vec3 b,float s){return {a.x+b.x*s,a.y,a.z+b.z*s};}
inline bool member(const SideSnapshot& s,ActorKey k){return std::find(s.members.begin(),s.members.end(),k)!=s.members.end();}
inline const ActorSnapshot* actor(const WorldSnapshot& w,ActorKey k){for(const auto& a:w.actors)if(a.key==k)return &a;return nullptr;}
inline const Contact* own_visible(const ActorSnapshot& a,const Config& c,TimeMs now){
    const auto* t=select_contact(a,c,now);
    // Visible is this capture's direct perception; memory.updated_at can stay
    // old while the enemy remains continuously visible in X-Ray.
    return t&&t->hostile&&t->alive_when_observed&&!t->wounded&&personally_visible(*t,a)&&
        finite(t->position)?t:nullptr;
}
inline bool suitable(const WorldSnapshot& w,const SideSnapshot& side,const ActorSnapshot& a,const Order& o,
    const Config& c){
    return member(side,a.key)&&a.alive&&!a.wounded&&!a.reloading&&!a.yield_reasons&&
        (a.flags&flag_admitted)&&a.ownership_generation&&a.current_position_accessible&&
        a.node_id!=invalid_node&&a.level_epoch==w.level_epoch&&a.snapshot_seq==w.sequence&&
        fresh(a.observed_at_ms,w.now_ms,c.snapshot_max_age_ms)&&
        a.weapon.id!=invalid_id&&a.weapon.ammo_in_mag&&a.weapon.state!=WeaponState::reloading&&
        o.actor==a.key&&o.ownership_generation==a.ownership_generation&&
        o.restrictions_revision==a.restrictions_revision&&o.geometry_revision==w.geometry_revision&&
        o.snapshot_seq==w.sequence&&o.expires_at_ms>w.now_ms&&
        (o.kind==OrderKind::hold||o.kind==OrderKind::cover||o.kind==OrderKind::hold_door)&&o.pace==Pace::stand&&
        o.role!=roles::portal_watch&&o.role!=roles::rear_security&&o.role!=roles::casualty_guard&&
        o.role!=roles::aid_casualty&&o.role!=roles::care_security&&o.role!=roles::barricade_guard&&
        o.role!=roles::door_fight&&o.role!=roles::door_fight_second&&o.role!=roles::assess_pie&&
        o.role!=roles::door_stack&&o.role!=roles::door_stack_point&&o.role!=roles::under_fire_cover&&
        o.role!=roles::reload_cover&&o.role!=roles::seek_cover&&
        !(o.flags&(order_committed_entry|order_urgent|order_fire_lane_split))&&
        !o.throw_order.valid&&!valid_key(o.aid)&&!valid_key(o.finish)&&
        finite(a.position)&&finite(o.goal_position)&&
        flat2(a.position,o.goal_position)<=o.arrival_radius_m*o.arrival_radius_m;
}
inline bool same_room(const EngineServices& s,const ActorSnapshot& a,const ActorSnapshot& b){
    if(!s.buildings||!s.buildings->locate)return false;
    const auto x=s.buildings->locate(a.node_id),y=s.buildings->locate(b.node_id);
    return x.first!=invalid_room&&x.second!=invalid_room&&x==y;
}
inline bool obstructs(Vec3 body,Vec3 eye,Vec3 target,float radius){
    const auto d=sub2(target,eye),r=sub2(body,eye);const float length=dot2(d,d);
    if(length<=.01f)return true;const float u=dot2(r,d)/length;
    if(u<=0||u>=1)return false;
    const Vec3 nearest=add2(eye,d,u);return flat2(body,nearest)<radius*radius;
}
inline bool segment_near(Vec3 body,Vec3 from,Vec3 to,float radius){
    const auto d=sub2(to,from),r=sub2(body,from);const auto len=dot2(d,d);
    const auto u=len>.0001f?(std::max)(0.f,(std::min)(1.f,dot2(r,d)/len)):0.f;
    return flat2(body,add2(from,d,u))<radius*radius;
}
inline bool crosses_line(Vec3 a,Vec3 b,Vec3 eye,Vec3 target){
    const auto d=sub2(b,a),f=sub2(target,eye);const auto den=d.x*f.z-d.z*f.x;
    if(std::fabs(den)<.00001f)return false;
    const auto r=sub2(eye,a);
    const auto u=(r.x*f.z-r.z*f.x)/den;
    const auto v=-(r.x*d.z-r.z*d.x)/den;
    return u>0.f&&u<1.f&&v>0.f&&v<1.f;
}
inline bool portal_cross(Vec3 a,Vec3 b,const Portal& p,float margin){
    const auto from=sub2(a,p.centre),to=sub2(b,p.centre);
    const float x=dot2(from,p.normal),y=dot2(to,p.normal);
    if((x<0&&y<0)||(x>0&&y>0))return false;
    const float den=x-y;if(std::fabs(den)<.00001f)return false;
    const float u=x/den;if(u<0||u>1)return false;
    const Vec3 cross=add2(a,sub2(b,a),u);
    const Vec3 tangent{-p.normal.z,0,p.normal.x};
    return std::fabs(dot2(sub2(cross,p.centre),tangent))<=p.half_width+margin&&
        cross.y>=p.y0-margin&&cross.y<=p.y1+margin;
}
inline bool door_near(const Building& building,std::uint32_t room,Vec3 front,Vec3 rear,Vec3 target){
    for(const auto& p:building.portals){
        if(!p.traversable||!p.doorway||(p.room_a!=room&&p.room_b!=room))continue;
        const auto f=sub2(front,p.centre),r=sub2(rear,p.centre),t=sub2(target,p.centre);
        const auto fd=dot2(f,p.normal),rd=dot2(r,p.normal),td=dot2(t,p.normal);
        const auto tangent=Vec3{-p.normal.z,0,p.normal.x};
        if(std::fabs(fd)<=1.5f&&std::fabs(dot2(f,tangent))<=p.half_width+1.f&&
            fd*rd>=0&&fd*td<0&&front.y>=p.y0-.5f&&front.y<=p.y1+.5f)return true;
    }
    return false;
}
inline bool route_safe(const RouteProofReceipt& route,const Building& building,const BuildingCache& cache,
    const WorldSnapshot& w,const Config& cfg,const std::vector<ProposedOrder>& proposals,
    const ActorSnapshot& mover,Vec3 target,float spacing){
    const auto origin=cache.locate(mover.node_id);
    for(auto node:route.nodes)if(cache.locate(node)!=origin)return false;
    for(std::size_t i=0;i<route.points.size();++i){
        const auto point=route.points[i];if(!finite(point))return false;
        // Every sampled route segment must remain outside a doorway throat.
        if(i)for(const auto& portal:building.portals)if(portal.traversable&&portal.doorway&&
            (portal.room_a==origin.second||portal.room_b==origin.second)&&
            portal_cross(route.points[i-1],point,portal,spacing*.5f))return false;
        if(i==0)continue; // the actor already occupies his start cell
        for(const auto& other:w.actors){
            if(other.key==mover.key||other.faction!=mover.faction||!other.alive||!finite(other.position))continue;
            if(segment_near(other.position,route.points[i-1],point,spacing))return false;
            const auto held=std::find_if(proposals.begin(),proposals.end(),[&](const ProposedOrder& p){return p.order.actor==other.key;});
            if(held!=proposals.end()&&held->order.goal_node!=invalid_node&&finite(held->order.goal_position)&&
                segment_near(held->order.goal_position,route.points[i-1],point,spacing))return false;
            if(held!=proposals.end()&&held->order.fire.allowed&&held->order.look.kind==LookKind::enemy){
                const auto* foe=own_visible(other,cfg,w.now_ms);
                if(foe&&foe->key==held->order.look.target&&
                    crosses_line(route.points[i-1],point,other.position,foe->position))return false;
            }
        }
    }
    const auto goal=route.points.back();
    for(const auto& other:w.actors)if(other.key!=mover.key&&other.faction==mover.faction&&other.alive&&
        finite(other.position)&&obstructs(other.position,goal,target,spacing*.75f))return false;
    return true;
}
}

inline FireLaneResult compose_fire_lanes(const WorldSnapshot& w,const SideSnapshot& side,
    std::vector<ProposedOrder>& proposals,const Config& cfg,const EngineServices& services,
    const RouteProofService* routes,RouteProofBudget& budget,const std::function<bool()>& stop={}){
    using namespace fire_lane_detail;
    FireLaneResult out;
    for(auto& p:proposals)if(member(side,p.order.actor)&&
        (p.order.flags&(order_fire_lane_low|order_fire_lane_high))){
        const bool low=(p.order.flags&order_fire_lane_low)!=0;
        const bool authored_crouch=(p.order.flags&(low?order_fire_lane_low_was_crouch:order_fire_lane_high_was_crouch))!=0;
        p.order.flags&=~(order_fire_lane_low|order_fire_lane_high|
            order_fire_lane_low_was_crouch|order_fire_lane_high_was_crouch);
        if((p.order.kind==OrderKind::hold||p.order.kind==OrderKind::cover||p.order.kind==OrderKind::hold_door)&&
            !(p.order.flags&order_urgent))p.order.stance=authored_crouch?Stance::crouch:Stance::stand;
        out.changes.push_back({p.order.actor,{},false,false});
    }
    for(auto& p:proposals)if(member(side,p.order.actor)&&(p.order.flags&order_fire_lane_split)){
        if(p.order.pace==Pace::stand){
            p.order.flags&=~order_fire_lane_split;
            out.changes.push_back({p.order.actor,{},false,false});
            continue;
        }
        const auto* a=actor(w,p.order.actor);
        const auto* target=a?own_visible(*a,cfg,w.now_ms):nullptr;
        bool paired=false;
        if(target&&p.order.look.kind==LookKind::enemy&&p.order.look.target==target->key)
            for(auto key:side.members){
                if(key==p.order.actor)continue;
                const auto* b=actor(w,key);if(!b)continue;
                const auto q=std::find_if(proposals.begin(),proposals.end(),[&](const ProposedOrder& x){return x.order.actor==key;});
                if(q!=proposals.end()&&suitable(w,side,*b,q->order,cfg)){
                    const auto* other=own_visible(*b,cfg,w.now_ms);
                    if(other&&other->key==target->key){paired=true;break;}
                }
            }
        const bool new_task=(p.order.flags&(order_committed_entry|order_urgent))||
            (p.order.kind!=OrderKind::hold&&p.order.kind!=OrderKind::cover&&p.order.kind!=OrderKind::hold_door);
        if(paired&&!new_task)continue;
        p.order.flags&=~order_fire_lane_split;
        if(!new_task&&a&&a->alive&&a->node_id!=invalid_node&&a->current_position_accessible){
            p.order.goal_node=a->node_id;p.order.goal_position=a->position;p.order.pace=Pace::stand;
        }
        out.changes.push_back({p.order.actor,{},false,false});
    }
    if(!w.level_epoch||!w.sequence||!w.geometry_revision||!services.buildings)return out;
    struct Candidate {const ActorSnapshot* actor;ProposedOrder* proposal;const Contact* foe;};
    std::vector<Candidate> able;
    for(auto key:side.members){if(stop&&stop()){out.cancelled=true;return out;}
        const auto* a=actor(w,key);if(!a)continue;
        auto p=std::find_if(proposals.begin(),proposals.end(),[&](const ProposedOrder& x){return x.order.actor==key;});
        if(p==proposals.end()||!suitable(w,side,*a,p->order,cfg))continue;
        if(const auto* foe=own_visible(*a,cfg,w.now_ms))
            if((p->order.look.kind!=LookKind::enemy||p->order.look.target==foe->key)&&p->order.fire.allowed)
                able.push_back({a,&*p,foe});}
    std::sort(able.begin(),able.end(),[](const Candidate& a,const Candidate& b){return a.actor->key.id<b.actor->key.id;});
    for(std::size_t i=0;i<able.size();++i)for(std::size_t j=i+1;j<able.size();++j){
        if(stop&&stop()){out.cancelled=true;return out;}
        auto& x=able[i];auto& y=able[j];if(x.foe->key!=y.foe->key||
            flat2(x.foe->position,y.foe->position)>1.f||!same_room(services,*x.actor,*y.actor))continue;
        const auto target=x.foe->position;
        auto* front=&x;auto* rear=&y;
        if(flat2(y.actor->position,target)<flat2(x.actor->position,target))std::swap(front,rear);
        const Vec3 aim=sub2(target,rear->actor->position),axis=sub2(front->actor->position,rear->actor->position);
        const float aim_m=std::sqrt(dot2(aim,aim)),gap=std::sqrt(dot2(axis,axis));
        if(aim_m<6.f||gap<1.f||gap>3.f||dot2(axis,aim)<0||
            std::fabs(axis.x*aim.z-axis.z*aim.x)/aim_m>1.f)continue;
        if(!services.buildings->locate)continue;
        const auto region=services.buildings->locate(rear->actor->node_id);
        const Building* building=nullptr;for(const auto& b:services.buildings->buildings)if(b.id==region.first){building=&b;break;}
        if(!building)continue;
        if(door_near(*building,region.second,front->actor->position,rear->actor->position,target)){
            if(front->proposal->order.stance==Stance::crouch)
                front->proposal->order.flags|=order_fire_lane_low_was_crouch;
            if(rear->proposal->order.stance==Stance::crouch)
                rear->proposal->order.flags|=order_fire_lane_high_was_crouch;
            front->proposal->order.stance=Stance::crouch;
            rear->proposal->order.stance=Stance::stand;
            front->proposal->order.flags|=order_fire_lane_low;
            rear->proposal->order.flags|=order_fire_lane_high;
            out.changes.push_back({front->actor->key,rear->actor->key,true,false});
            out.changes.push_back({rear->actor->key,front->actor->key,false,false});
            return out; // one paired posture, native friend ray still decides rear fire
        }
        if(!services.project||!services.rays||!services.buildings->locate||!routes||!routes->query||!budget.work_left)continue;
        const float step=(std::max)(1.2f,cfg.room.spacing_m);
        const Vec3 lateral{-aim.z/aim_m,0,aim.x/aim_m};
        for(float sign:{-1.f,1.f}){
            Vec3 goal;const Vec3 requested=add2(rear->proposal->order.goal_position,lateral,sign*step);
            const auto node=services.project(requested,cfg.room.projection_m,cfg.room.floor_tolerance_m,&goal);
            if(node==invalid_node||node==rear->actor->node_id||!finite(goal)||
                flat2(goal,requested)>cfg.room.projection_m*cfg.room.projection_m||
                services.buildings->locate(node)!=region)continue;
            Vec3 eye=goal,enemy=target;eye.y+=cfg.room.look_height_m;enemy.y+=cfg.room.look_height_m;
            const Segment ray{eye,enemy};float hit=-2;services.rays(&ray,&hit,1);
            if(!std::isfinite(hit)||!(hit==-1.f||hit>=std::sqrt(distance_squared(eye,enemy))))continue;
            const auto request=route_proof_source(w,*rear->actor,node,goal);
            const auto route=request_route_proof(routes,request,budget,stop);
            if(route.status==RouteProofStatus::cancelled){out.cancelled=true;return out;}
            if(!route_proof_complete(route,request)||!route_safe(route,*building,*services.buildings,w,cfg,proposals,*rear->actor,
                target,cfg.room.spacing_m))continue;
            // Keep the assigned sector, aim and fire intent. The engine checks
            // the current muzzle/friend ray on every moving frame.
            auto& order=rear->proposal->order;order.goal_node=node;order.goal_position=goal;order.pace=Pace::walk;
            order.look.kind=LookKind::enemy;order.look.target=rear->foe->key;
            order.flags|=order_fire_lane_split;
            out.changes.push_back({rear->actor->key,front->actor->key,false,true});
            out.route_work=budget.work_used;return out;
        }
    }
    out.route_work=budget.work_used;return out;
}

}}}
