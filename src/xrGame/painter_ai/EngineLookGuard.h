#pragma once
#include "EngineExecutionContract.h"
#include <array>

namespace painter { namespace combat { namespace engine {
struct LookEpisodeEvents {bool hit=false,arrival=false,door_cross=false;};
// Physical/source edges only. Refreshing an order sequence is not a new reason
// to turn the head. The owner resets this state with its actor/ownership lease.
class LookEpisodeTracker {
    TimeMs last_hit_=0;
    bool have_arrival_=false,arrived_=false,have_portal_=false;
    Vec3 centre_{},normal_{};int door_side_=0;
public:
    void reset(){*this=LookEpisodeTracker{};}
    LookEpisodeEvents update(const Order& o,const ActorSnapshot& a,Vec3 position,TimeMs now,TimeMs hit_age){
        LookEpisodeEvents out;TimeMs latest=last_hit_;
        for(std::size_t i=0;i<a.hits.count&&i<a.hits.items.size();++i){const auto& hit=a.hits.items[i];
            if(hit.at_ms>last_hit_&&hit.at_ms<=now&&now-hit.at_ms<=hit_age&&finite_value(hit.dir)){
                out.hit=true;if(hit.at_ms>latest)latest=hit.at_ms;
            }
        }
        last_hit_=latest;
        if(finite_value(position)&&finite_value(o.goal_position)&&std::isfinite(o.arrival_radius_m)&&o.arrival_radius_m>=0){
            const float x=position.x-o.goal_position.x,y=position.y-o.goal_position.y,z=position.z-o.goal_position.z;
            const bool here=x*x+y*y+z*z<=o.arrival_radius_m*o.arrival_radius_m;
            out.arrival=have_arrival_&&!arrived_&&here;have_arrival_=true;arrived_=here;
        }
        if(o.portal.valid&&finite_value(position)&&finite_value(o.portal.centre)&&finite_value(o.portal.normal)){
            const bool same=have_portal_&&same_value(centre_,o.portal.centre)&&same_value(normal_,o.portal.normal);
            const float signed_distance=(position.x-o.portal.centre.x)*o.portal.normal.x+
                (position.y-o.portal.centre.y)*o.portal.normal.y+(position.z-o.portal.centre.z)*o.portal.normal.z;
            // Keep the last definite side while a foot/pose jitters on the plane.
            constexpr float door_plane_hysteresis_m=.05f;
            const int side=signed_distance>door_plane_hysteresis_m?1:signed_distance<-door_plane_hysteresis_m?-1:0;
            out.door_cross=same&&side&&door_side_&&side!=door_side_;
            if(!same)door_side_=side;else if(side)door_side_=side;
            have_portal_=true;centre_=o.portal.centre;normal_=o.portal.normal;
        }else {have_portal_=false;door_side_=0;}
        return out;
    }
};
// A bounded authority cache: unknown never becomes an unobstructed sight line.
// Keys include the measured eye, endpoint and geometry; no target transform is
// read here. The native caller supplies the one fair ray grant for this frame.
class LookLineCache {
    struct Line {Vec3 eye,point;TimeMs at=0;std::uint64_t geometry=0;Evidence clear=Evidence::unknown;ActorKey target;};
    std::array<Line,4> rows_{};unsigned next_=0;
public:
    void reset(){rows_={};next_=0;}
    template<class Query> Evidence line(Vec3 eye,Vec3 point,TimeMs now,TimeMs keep,
        std::uint64_t geometry,Query query,ActorKey target={}){
        if(!finite_value(eye)||!finite_value(point)||!geometry)return Evidence::unknown;
        for(const auto& row:rows_)if(row.clear!=Evidence::unknown&&row.geometry==geometry&&row.target==target&&row.at<=now&&now-row.at<=keep&&
            same_value(row.point,point)){
            const float dx=row.eye.x-eye.x,dy=row.eye.y-eye.y,dz=row.eye.z-eye.z;
            // Only negative static lines may survive slight eye sway. A clear
            // or body-target ray always needs exactly the measured origin.
            if(same_value(row.eye,eye)||(target.id==invalid_id&&row.clear==Evidence::no&&dx*dx+dy*dy+dz*dz<=.0625f))return row.clear;
        }
        const auto result=query();
        if(result!=Evidence::unknown){rows_[next_++%rows_.size()]={eye,point,now,geometry,result,target};}
        return result;
    }
};
inline bool authored_look_exception(const Order&o){return o.kind==OrderKind::entry||o.pie.valid||o.corner.valid||(o.flags&(order_side_peek|order_fan_peek));}
// Orientation only: this consumes an authored point and the measured eye,
// never an unseen object's transform or evidence of a clear firing line.
inline DoorwayLook bearing_look(DoorwayLook desired,Vec3 origin){
    if((desired.kind!=LookKind::point&&desired.kind!=LookKind::enemy)||
        !finite_value(desired.value)||!finite_value(origin))return {};
    const double x=double(desired.value.x)-origin.x,z=double(desired.value.z)-origin.z;
    const double length=std::sqrt(x*x+z*z);
    if(!std::isfinite(length)||length<=.0001)return {};
    return {LookKind::direction,{float(x/length),0.f,float(z/length)},{}};
}
inline DoorwayLook open_look(const Order&o,DoorwayLook desired,bool visible,Evidence direct,
    bool has_opening,Vec3 opening,Evidence opening_clear,const Vec3* origin=nullptr){
    if(visible||authored_look_exception(o)||desired.kind==LookKind::direction||desired.kind==LookKind::path)return desired;
    if(direct==Evidence::yes)return desired;
    if(has_opening&&finite_value(opening)&&opening_clear==Evidence::yes)return {LookKind::point,opening,{}};
    return origin?bearing_look(desired,*origin):DoorwayLook{}; // No origin means no justified bearing.
}
inline bool actual_first_order_overdue(TimeMs admitted,TimeMs now,TimeMs deadline,bool owned,bool executed){
    return owned&&!executed&&now>=admitted&&now-admitted>=deadline;
}
inline ActorKey next_finish_contact(const ActorSnapshot& a,ActorKey preferred,ActorKey after){
    ActorKey first,next;bool prefer=false;
    auto less=[](ActorKey x,ActorKey y){return x.id<y.id||(x.id==y.id&&x.generation<y.generation);};
    auto offer=[&](const Contact& c){
        if(!c.visible||!c.wounded||!c.hostile||!c.alive_when_observed||c.key.id==invalid_id||!c.key.generation)return;
        if(c.key==preferred)prefer=true;
        if(first.id==invalid_id||less(c.key,first))first=c.key;
        if(less(after,c.key)&&(next.id==invalid_id||less(c.key,next)))next=c.key;
    };
    offer(a.enemy);for(std::size_t i=0;i<a.visible_enemies.count&&i<a.visible_enemies.items.size();++i)offer(a.visible_enemies.items[i]);
    return prefer?preferred:next.id!=invalid_id?next:first;
}
inline bool ordinary_refresh_grace(const Order&o,const ActorSnapshot&a,const Config&cfg,TimeMs now){
    if(now<o.expires_at_ms||now-o.expires_at_ms>=cfg.first_order_deadline_ms||
        a.observed_at_ms>now||now-a.observed_at_ms>cfg.snapshot_max_age_ms||!a.alive||a.wounded||a.yield_reasons||
        o.actor!=a.key||o.ownership_generation!=a.ownership_generation||o.restrictions_revision!=a.restrictions_revision||
        o.assignment.generation||o.throw_order.valid||o.aid.id!=invalid_id||o.finish.id!=invalid_id||(o.flags&order_urgent))return false;
    if(o.kind!=OrderKind::hold&&o.kind!=OrderKind::cover&&o.kind!=OrderKind::search&&o.kind!=OrderKind::reload)return false;
    const auto seen=[&](const Contact& c){return c.key.id!=invalid_id&&c.visible&&c.hostile&&c.alive_when_observed&&!c.wounded;};
    if(seen(a.enemy))return true;
    for(std::size_t i=0;i<a.visible_enemies.count&&i<a.visible_enemies.items.size();++i)if(seen(a.visible_enemies.items[i]))return true;
    for(std::size_t i=0;i<a.hits.count&&i<a.hits.items.size();++i)if(a.hits.items[i].at_ms<=now&&now-a.hits.items[i].at_ms<cfg.knowledge.uf_escape_ms)return true;
    return false;
}
// A room plan can miss one publish boundary while its point/heel are standing.
// This is ownership-only grace: the executor must remain in stale_hold, at its
// captured feet, with no new application receipt or renewed authored fire permission.
inline bool room_refresh_hold_grace(const Order&o,const ActorSnapshot&a,const Config&cfg,TimeMs now){
    if(now<o.expires_at_ms||now-o.expires_at_ms>=cfg.first_order_deadline_ms||
        now<o.issued_at_ms||execution_phase(o,now,cfg.stale_release_ms)==ExecutionPhase::release||
        a.observed_at_ms>now||now-a.observed_at_ms>cfg.snapshot_max_age_ms||
        !a.alive||a.wounded||a.yield_reasons||!(a.flags&flag_admitted)||
        o.actor!=a.key||o.ownership_generation!=a.ownership_generation||
        o.restrictions_revision!=a.restrictions_revision||o.level_epoch!=a.level_epoch||
        !o.assignment.generation||!o.assignment.portal||!o.portal.valid||
        o.throw_order.valid||o.aid.id!=invalid_id||o.finish.id!=invalid_id||(o.flags&order_urgent))return false;
    return o.kind==OrderKind::stack||o.kind==OrderKind::peek||o.kind==OrderKind::entry||
        o.kind==OrderKind::hold_door||o.kind==OrderKind::search;
}
}}}
