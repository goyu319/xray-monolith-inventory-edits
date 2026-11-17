#include "CombatOutdoorSensors.h"
#include <cmath>
namespace painter { namespace combat { namespace core {
OutdoorSearchObservation measure_outdoor_search_observation(const Config&c,const WorldSnapshot&w,
    const ActorSnapshot&a,const OutdoorCapturedEye&e,const Order&o,const MemberKnowledge&m,
    OutdoorSensorQuota&q,const std::function<Evidence(const Segment&)>&dynamic_ray){
    OutdoorSearchObservation out;
    if(!dynamic_ray||!q.queries_left||!a.alive||a.wounded||a.yield_reasons||!(a.flags&flag_admitted)||
        a.level_epoch!=w.level_epoch||a.snapshot_seq!=w.sequence||a.observed_at_ms!=w.now_ms||
        !e.measured||e.actor!=a.key||e.epoch!=w.level_epoch||e.sequence!=w.sequence||e.owner!=a.ownership_generation||
        e.restrictions!=a.restrictions_revision||e.observed_at_ms!=w.now_ms||!finite(e.feet)||!finite(e.eye)||
        distance_squared(e.feet,a.position)!=0||distance_squared(e.eye,e.feet)<=0||
        m.actor!=a.key||m.ownership_generation!=a.ownership_generation||!m.alive||!m.able||
        o.kind!=OrderKind::search||o.look.kind!=LookKind::point||o.actor!=a.key||!o.order_seq||o.order_seq!=a.last_applied.order_seq||
        o.ownership_generation!=a.ownership_generation||o.restrictions_revision!=a.restrictions_revision||
        o.level_epoch!=w.level_epoch||o.geometry_revision!=w.geometry_revision||o.expires_at_ms<=w.now_ms||
        o.issued_at_ms>a.last_applied.at_ms||a.last_applied.at_ms>a.observed_at_ms||!a.accessible_current_order||
        !finite(a.head_dir)||!finite(o.look.value))return out;
    const float height=c.outdoor.p.solo.combat.target_height_m,half=c.room.check_half_deg,reach=c.outdoor.p.posture.search_reach_m;
    if(!std::isfinite(height)||height<=0||!std::isfinite(half)||half<=0||half>=90||!std::isfinite(reach)||reach<=0)return out;
    const FoeKnowledge* selected=nullptr;const SourceFact* source=nullptr;Vec3 point;
    for(const auto&foe:m.foes){if(!foe.id.valid()||foe.id.ghost()||foe.down)continue;const auto*fact=foe.last_known();
        if(!fact||!fact->valid||!valid_key(fact->reporter)||!finite(fact->position)||!fresh(fact->at_ms,w.now_ms,c.knowledge.search_forget_ms))continue;
        auto at=fact->position;at.y+=height;if(distance_squared(at,o.look.value)!=0)continue;
        if(selected)return out;selected=&foe;source=fact;point=at;}
    if(!selected||std::hypot(a.position.x-source->position.x,a.position.z-source->position.z)>reach)return out;
    const Vec3 delta{point.x-e.eye.x,point.y-e.eye.y,point.z-e.eye.z};
    const float head=std::sqrt(distance_squared(a.head_dir,{})),sight=std::sqrt(distance_squared(delta,{}));
    if(head<=.5f||!std::isfinite(sight)||sight<=.001f||
        (a.head_dir.x*delta.x+a.head_dir.y*delta.y+a.head_dir.z*delta.z)/(head*sight)<std::cos(half*3.14159265358979323846f/180.f))return out;
    auto&stamp=out.source;stamp.actor=a.key;stamp.level_epoch=w.level_epoch;stamp.snapshot_seq=w.sequence;stamp.geometry_revision=w.geometry_revision;
    stamp.ownership_generation=a.ownership_generation;stamp.restrictions_revision=a.restrictions_revision;stamp.at_ms=w.now_ms;
    stamp.origin_node=a.node_id;stamp.actor_position=a.position;stamp.focus=selected->id;stamp.focus_at_ms=source->at_ms;stamp.focus_position=source->position;
    out.applied_order_seq=o.order_seq;out.measured_eye=e.eye;out.observed_point=point;out.head=a.head_dir;
    --q.queries_left;++q.queries_used;try{out.clear=dynamic_ray({e.eye,point});}catch(...){out.clear=Evidence::unknown;}return out;
}
OutdoorMeasuredLine measure_outdoor_line(const WorldSnapshot&w,const ActorSnapshot&a,
    const OutdoorCapturedEye&e,const KnowledgeTrace&t,const EngineServices&svc,
    OutdoorSensorQuota&q,const std::function<bool()>&cancelled){
    return measure_outdoor_line(w,a,e,t,svc,q,0,cancelled);
}
OutdoorMeasuredLine measure_outdoor_line(const WorldSnapshot&w,const ActorSnapshot&a,
    const OutdoorCapturedEye&e,const KnowledgeTrace&t,const EngineServices&svc,
    OutdoorSensorQuota&q,float target_height_m,const std::function<bool()>&cancelled){
    OutdoorMeasuredLine out;auto stop=[&](){return cancelled&&cancelled();};
    if(stop()||!std::isfinite(target_height_m)||target_height_m<0||!svc.rays||!q.queries_left||!e.measured||e.actor!=a.key||e.epoch!=w.level_epoch||
        e.sequence!=w.sequence||e.owner!=a.ownership_generation||e.restrictions!=a.restrictions_revision||e.observed_at_ms!=a.observed_at_ms||
        a.level_epoch!=w.level_epoch||a.snapshot_seq!=w.sequence||a.observed_at_ms!=w.now_ms||!finite(a.position)||
        !finite(e.feet)||distance_squared(e.feet,a.position)!=0||!finite(e.eye)||distance_squared(e.eye,e.feet)<=0||
        t.receiver!=a.key||t.receiver_ownership_generation!=a.ownership_generation||t.receiver_faction!=a.faction||
        !t.foe.valid()||t.foe.ghost()||!finite(t.position)||t.observed_at_ms>a.observed_at_ms)return out;
    Vec3 aim=t.position;aim.y+=target_height_m;
    const float length=std::sqrt(distance_squared(e.eye,aim));if(!std::isfinite(length)||length<=0)return out;
    auto&source=out.source;source.actor=a.key;source.level_epoch=w.level_epoch;source.snapshot_seq=w.sequence;
    source.geometry_revision=w.geometry_revision;source.ownership_generation=a.ownership_generation;source.restrictions_revision=a.restrictions_revision;
    source.at_ms=w.now_ms;source.origin_node=a.node_id;source.actor_position=a.position;source.focus=t.foe;
    source.focus_at_ms=t.observed_at_ms;source.focus_position=t.position;out.measured_eye=e.eye;
    Segment segment{e.eye,aim};float hit=-2;--q.queries_left;++q.queries_used;
    try{svc.rays(&segment,&hit,1);}catch(...){return out;}
    if(stop())return OutdoorMeasuredLine{};
    if(hit==-1)out.clear=Evidence::yes;
    else if(std::isfinite(hit)&&hit>=0&&hit<=length)out.clear=Evidence::no;
    return out;
}
}}}
