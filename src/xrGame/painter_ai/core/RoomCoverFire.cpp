#include "RoomCoverFire.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>

namespace painter { namespace combat { namespace core {
namespace {
struct Cancelled{};
void poll(const std::function<bool()>& stop){if(stop&&stop())throw Cancelled{};}
bool same(Vec3 a,Vec3 b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
bool member(const SideSnapshot& s,ActorKey k){return std::find(s.members.begin(),s.members.end(),k)!=s.members.end();}
const ActorSnapshot* actor(const WorldSnapshot& w,ActorKey k){const ActorSnapshot* result=nullptr;
    for(const auto& a:w.actors)if(a.key.id==k.id){if(a.key!=k||result)return nullptr;result=&a;}return result;}
bool source(const ActorSnapshot& a,const WorldSnapshot& w,const Config& c){return valid_key(a.key)
    &&a.level_epoch==w.level_epoch&&a.snapshot_seq==w.sequence&&fresh(a.observed_at_ms,w.now_ms,c.snapshot_max_age_ms)
    &&finite(a.position)&&a.alive&&std::isfinite(a.health)&&a.health>0;}
bool order_source(const RoomDoctrineSlot& s,const ActorSnapshot& a,const WorldSnapshot& w){const auto& o=s.order;
    return s.actor==a.key&&o.actor==a.key&&o.level_epoch==w.level_epoch&&o.snapshot_seq==w.sequence
        &&o.geometry_revision==w.geometry_revision&&o.ownership_generation==a.ownership_generation
        &&o.restrictions_revision==a.restrictions_revision&&o.issued_at_ms<=w.now_ms&&o.expires_at_ms>w.now_ms;}
bool free_man(const RoomDoctrineSlot& s,const ActorSnapshot& a,const WorldSnapshot& w,const Config& c){
    const auto& o=s.order;
    if(!source(a,w,c)||!order_source(s,a,w)||!a.ownership_generation||a.wounded||a.reloading
        ||a.weapon.state==WeaponState::reloading||a.yield_reasons||!(a.flags&flag_admitted)||!a.current_position_accessible
        ||a.node_id==invalid_node||a.weapon.id==invalid_id||!a.weapon.ammo_in_mag||!a.weapon.mag_size
        ||a.aid_state.phase==1||a.aid_state.phase==2||a.throw_state.phase==1||a.throw_state.phase==2
        ||s.released||o.kind==OrderKind::yield||o.kind==OrderKind::entry||o.kind==OrderKind::reload
        ||(o.flags&order_committed_entry)||o.throw_order.valid||valid_key(o.aid)||valid_key(o.finish)
        ||o.pie.valid||o.corner.valid)return false;
    switch(o.role){case roles::aid_casualty:case roles::casualty_guard:case roles::care_security:return false;default:break;}
    return true;
}
const Contact* own_contact(const ActorSnapshot& a,FoeId id,TimeMs now,const Config& c){
    if(id.ghost()||!valid_key(id.actor)||a.visible_enemies.count>a.visible_enemies.items.size())return nullptr;
    auto valid=[&](const Contact& t){return t.key==id.actor&&t.key!=a.key&&t.hostile&&t.alive_when_observed
        &&(!t.wounded||t.finish_allowed)&&finite(t.position)&&t.observed_at_ms<=a.observed_at_ms
        &&fresh(t.observed_at_ms,now,c.memory_max_age_ms);};
    if(valid(a.enemy))return &a.enemy;
    for(std::size_t i=0;i<a.visible_enemies.count;++i)if(valid(a.visible_enemies.items[i]))return &a.visible_enemies.items[i];
    return nullptr;
}
bool other_fight(const ActorSnapshot& a,FoeId target,const Config& c,TimeMs now){
    auto active=[&](const Contact& t){return (target.ghost()||t.key!=target.actor)&&valid_key(t.key)&&t.hostile&&t.alive_when_observed
        &&(!t.wounded||t.finish_allowed)&&personally_visible(t,a)&&fresh(t.observed_at_ms,now,c.snapshot_max_age_ms);};
    if(active(a.enemy)||a.visible_enemies.count>a.visible_enemies.items.size())return true;
    for(std::size_t i=0;i<a.visible_enemies.count;++i)if(active(a.visible_enemies.items[i]))return true;return false;
}
bool frame(const Portal& p,Vec3& normal,Vec3& tangent){float n=std::hypot(p.normal.x,p.normal.z);
    if(!p.doorway||!finite(p.centre)||!finite(p.normal)||!std::isfinite(n)||n<=0||!std::isfinite(p.half_width)||p.half_width<=0
        ||!std::isfinite(p.wall_depth)||p.wall_depth<0||!std::isfinite(p.y0)||!std::isfinite(p.y1)||p.y1<=p.y0)return false;
    normal={p.normal.x/n,0,p.normal.z/n};tangent={-normal.z,0,normal.x};return true;}
float dotflat(Vec3 a,Vec3 b){return a.x*b.x+a.z*b.z;}
Vec3 sub(Vec3 a,Vec3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
bool in_aperture(const Portal& p,Vec3 body,const RoomConfig& c){Vec3 n,t;if(!frame(p,n,t))return false;const auto v=sub(body,p.centre);
    return body.y>=p.y0-c.floor_tolerance_m&&body.y<=p.y0+c.floor_tolerance_m
        &&std::abs(dotflat(v,n))<=p.wall_depth*.5f+c.entry_zone_depth_m
        &&std::abs(dotflat(v,t))<=p.half_width+c.entry_zone_half_m;}
bool through(const Portal& p,const HitKnowledge& h,const RoomConfig& c){Vec3 n,t;if(!frame(p,n,t))return false;
    if(!finite(h.origin)||!finite(h.point)||!finite(h.direction)||h.origin.y<p.y0-c.floor_tolerance_m||h.origin.y>p.y0+c.floor_tolerance_m)return false;
    const float d=dotflat(h.direction,n);if(std::abs(d)<(std::numeric_limits<float>::epsilon)())return false;
    const float along=-dotflat(sub(h.origin,p.centre),n)/d;if(!std::isfinite(along)||along<0)return false;
    Vec3 cross{h.origin.x+h.direction.x*along,h.origin.y,h.origin.z+h.direction.z*along};
    return finite(cross)&&std::abs(dotflat(sub(cross,p.centre),t))<=p.half_width+c.entry_zone_half_m;}
float edge_distance(const Portal& p,Vec3 at){Vec3 n,t;frame(p,n,t);const auto v=sub(at,p.centre);
    const float u=dotflat(v,t),z=dotflat(v,n);return std::hypot((std::min)(std::abs(u-p.half_width),std::abs(u+p.half_width)),z);}
bool crossing(const RoomDoctrineSlot& s,const ActorSnapshot& a,const WorldSnapshot& w,const Config& c,const Portal& p,
    const RoomCoverFireRequest& request){
    if(!(source(a,w,c)&&order_source(s,a,w)&&!a.wounded&&!a.yield_reasons&&s.released&&!s.arrived
        &&s.released_at_ms<=w.now_ms&&s.execution_confirmed&&s.applied_order_seq!=0
        &&(s.order.flags&order_committed_entry)&&s.order.assignment.portal==p.id&&in_aperture(p,a.position,c.room)
        &&a.last_applied.order_seq==s.applied_order_seq&&a.last_applied.at_ms<=w.now_ms))return false;
    const Order* applied=nullptr;for(const auto& o:request.applied_orders)if(o.actor==a.key){if(applied)return false;applied=&o;}
    return applied&&applied->order_seq==s.applied_order_seq&&applied->level_epoch==w.level_epoch
        &&applied->geometry_revision==w.geometry_revision&&applied->ownership_generation==a.ownership_generation
        &&applied->restrictions_revision==a.restrictions_revision&&applied->issued_at_ms<=w.now_ms
        &&applied->snapshot_seq<=w.sequence&&(applied->flags&order_committed_entry)
        &&applied->assignment.generation==s.order.assignment.generation&&applied->assignment.portal==s.order.assignment.portal
        &&applied->assignment.sector==s.order.assignment.sector&&applied->assignment.slot==s.order.assignment.slot;
}
}
RoomCoverFire::RoomCoverFire(Config c):config_(c){}
void RoomCoverFire::reset(){lines_.clear();epoch_=map_=geometry_=sequence_=0;side_=mission_=0;now_=0;}
RoomCoverFireResult RoomCoverFire::update(const WorldSnapshot& w,const SideSnapshot& s,const RoomDoctrineResult& b,
    const KnowledgeFrame* k,const EngineServices& e,const RoomCoverFireRequest& request,const std::function<bool()>& stop){
    RoomCoverFireResult bad;bad.reason=CoverFireReason::invalid_input;
    try{poll(stop);auto staged=*this;auto out=staged.update_impl(w,s,b,k,e,request,stop);poll(stop);
        if(out.accepted)*this=std::move(staged);return out;
    }catch(const Cancelled&){bad.cancelled=true;bad.reason=CoverFireReason::cancelled;return bad;}catch(...){return bad;}
}
RoomCoverFireResult RoomCoverFire::update_impl(const WorldSnapshot& w,const SideSnapshot& side,const RoomDoctrineResult& base,
    const KnowledgeFrame* k,const EngineServices& svc,const RoomCoverFireRequest& request,const std::function<bool()>& stop){
    RoomCoverFireResult out;out.reason=CoverFireReason::invalid_input;const auto& c=config_.room;
    if(!w.level_epoch||!w.sequence||!base.accepted||base.cancelled||base.level_epoch!=w.level_epoch||base.snapshot_seq!=w.sequence
        ||!k||!k->accepted||k->cancelled||k->level_epoch!=w.level_epoch||k->sequence!=w.sequence
        ||!std::isfinite(c.entry_zone_half_m)||c.entry_zone_half_m<0||!std::isfinite(c.entry_zone_depth_m)||c.entry_zone_depth_m<0
        ||!std::isfinite(c.floor_tolerance_m)||c.floor_tolerance_m<0||!std::isfinite(c.look_height_m)||c.look_height_m<0
        ||!std::isfinite(c.suppression_ammo_floor)||c.suppression_ammo_floor<0||c.suppression_ammo_floor>1
        ||!c.suppression_queue||c.suppression_queue>255||!c.suppression_interval_ms||c.suppression_interval_ms>65535)return out;
    if(epoch_&&(w.level_epoch<epoch_||(epoch_==w.level_epoch&&(w.sequence<=sequence_||w.now_ms<now_))))return out;
    std::set<std::uint32_t> ids;for(auto key:side.members){poll(stop);if(!valid_key(key)||!ids.insert(key.id).second)return out;}
    ids.clear();for(const auto& s:base.slots){poll(stop);if(!member(side,s.actor)||!ids.insert(s.actor.id).second)return out;}
    const SideKnowledge* sk=nullptr;for(const auto& s:k->sides)if(s.side==side.side_id){if(sk)return out;sk=&s;}
    if(epoch_!=w.level_epoch||map_!=w.map_hash||geometry_!=w.geometry_revision||side_!=side.side_id||mission_!=side.mission.revision)lines_.clear();
    epoch_=w.level_epoch;map_=w.map_hash;geometry_=w.geometry_revision;side_=side.side_id;mission_=side.mission.revision;sequence_=w.sequence;now_=w.now_ms;
    lines_.erase(std::remove_if(lines_.begin(),lines_.end(),[&](const Line& l){poll(stop);return !member(side,l.actor);}),lines_.end());
    out.accepted=true;out.level_epoch=w.level_epoch;out.snapshot_seq=w.sequence;out.knowledge_seq=k->sequence;out.slots=base.slots;
    out.reason=CoverFireReason::no_crossing;
    const Building* building=nullptr;if(svc.buildings&&svc.buildings->level_epoch==w.level_epoch&&svc.buildings->map_hash==w.map_hash)
        for(const auto& b:svc.buildings->buildings){poll(stop);if(b.id==side.mission.building){building=&b;break;}}
    if(!building)return out;
    const Portal* door=nullptr;const HitKnowledge* hit=nullptr;ActorKey crossed;bool any_crossing=false;
    for(const auto& slot:base.slots){poll(stop);const auto* a=actor(w,slot.actor);if(!a)continue;
        for(const auto& p:building->portals){poll(stop);if(!crossing(slot,*a,w,config_,p,request))continue;any_crossing=true;
            if(!sk)continue;for(const auto& h:sk->hits){poll(stop);
                const auto deadline=(std::min)(saturated_after(h.at_ms,c.return_fire_recent_shot_ms),
                    saturated_after(saturated_after(h.at_ms,c.return_fire_ms),c.return_fire_overlap_ms));
                if(!h.valid||!h.shooter.valid()||!member(side,h.observer)||!fresh(h.at_ms,w.now_ms,c.return_fire_recent_shot_ms)
                    ||deadline<=w.now_ms||!through(p,h,c))continue;
                if(!h.shooter.ghost()&&member(side,h.shooter.actor))continue;
                if(!h.shooter.ghost()&&std::find(sk->confirmed_down.begin(),sk->confirmed_down.end(),h.shooter.actor)!=sk->confirmed_down.end())continue;
                if(!hit||h.at_ms>hit->at_ms||(h.at_ms==hit->at_ms&&std::tie(p.id,slot.actor.id,slot.actor.generation)
                    <std::tie(door->id,crossed.id,crossed.generation))){hit=&h;door=&p;crossed=slot.actor;}
            }
        }
    }
    if(!hit){out.reason=any_crossing?CoverFireReason::no_hot_line:CoverFireReason::no_crossing;return out;}
    out.crossing=crossed;out.observer=hit->observer;out.portal=door->id;out.target=hit->shooter;out.evidence_at_ms=hit->at_ms;
    out.until_ms=(std::min)(saturated_after(hit->at_ms,c.return_fire_recent_shot_ms),
        saturated_after(saturated_after(hit->at_ms,c.return_fire_ms),c.return_fire_overlap_ms));
    struct Candidate {const ActorSnapshot* a;std::size_t slot;const Contact* contact;Vec3 aim;float distance;};std::vector<Candidate> candidates;
    for(std::size_t i=0;i<base.slots.size();++i){poll(stop);const auto& slot=base.slots[i];const auto* a=actor(w,slot.actor);
        if(!a||a->key==crossed||a->key==hit->observer||!free_man(slot,*a,w,config_)||other_fight(*a,hit->shooter,config_,w.now_ms))continue;
        bool self_hit=false;for(const auto& h:sk->hits){poll(stop);if(h.valid&&h.observer==a->key&&fresh(h.at_ms,w.now_ms,c.return_fire_ms)){self_hit=true;break;}}
        if(self_hit)continue;
        const auto* contact=own_contact(*a,hit->shooter,w.now_ms,config_);
        candidates.push_back({a,i,contact,contact?contact->position:hit->point,edge_distance(*door,a->position)});
    }
    out.costs.candidates=candidates.size();std::sort(candidates.begin(),candidates.end(),[&](const Candidate& a,const Candidate& b){poll(stop);
        return std::tie(a.distance,a.a->key.id,a.a->key.generation)<std::tie(b.distance,b.a->key.id,b.a->key.generation);});
    out.reason=candidates.empty()?CoverFireReason::no_responder:CoverFireReason::no_line;
    for(const auto& candidate:candidates){poll(stop);const auto& a=*candidate.a;Vec3 from=a.position;from.y+=c.look_height_m;
        Evidence clear=Evidence::unknown;auto line=std::find_if(lines_.begin(),lines_.end(),[&](const Line& l){return l.actor==a.key;});
        if(line!=lines_.end()&&line->owner==a.ownership_generation&&line->restrictions==a.restrictions_revision&&same(line->from,from)&&same(line->to,candidate.aim)){
            clear=line->clear;++out.costs.cached_rays;
        }else{
            if(!svc.rays||out.costs.ray_queries>=request.query_quantum){out.pending=true;out.phase=CoverFirePhase::seeking;out.reason=CoverFireReason::pending_line;return out;}
            Segment ray{from,candidate.aim};float distance=-2;poll(stop);svc.rays(&ray,&distance,1);++out.costs.ray_queries;poll(stop);
            const float length=std::sqrt(distance_squared(from,candidate.aim));
            if(std::isfinite(distance)&&std::isfinite(length))clear=distance==-1.f||distance>=length?Evidence::yes:distance>=0?Evidence::no:Evidence::unknown;
            Line value{a.key,a.ownership_generation,a.restrictions_revision,from,candidate.aim,clear};
            if(line==lines_.end())lines_.push_back(value);else *line=value;
        }
        if(clear==Evidence::unknown)++out.costs.unknown_rays;if(clear!=Evidence::yes)continue;
        out.actor=a.key;out.aim=candidate.aim;out.costs.jamb_distance_m=candidate.distance;
        auto& slot=out.slots[candidate.slot];auto& o=slot.order;o.look.kind=LookKind::point;o.look.value=candidate.aim;
        o.look.target=candidate.contact?candidate.contact->key:ActorKey{};o.flags=(o.flags&~order_face_movement)|order_cover_fire;
        o.role=roles::suppress_buddy;o.fire.allowed=false;o.fire.at_valid=false;o.fire.until_ms=0;
        slot.authored_look=o.look;slot.authored_look_valid=true;
        out.phase=CoverFirePhase::looking;out.reason=CoverFireReason::reported_direction;
        const auto until=(std::min)(out.until_ms,fire_window_until(config_,w.now_ms,o.expires_at_ms));
        if(candidate.contact&&!(a.flags&flag_dont_shoot)&&float(a.weapon.ammo_in_mag)/float(a.weapon.mag_size)>=c.suppression_ammo_floor
            &&config_.fire_window_ms&&until>w.now_ms){
            o.fire.allowed=true;o.fire.at_valid=true;o.fire.at=candidate.aim;o.fire.until_ms=until;
            o.fire.queue=static_cast<std::uint8_t>(c.suppression_queue);o.fire.interval_ms=static_cast<std::uint16_t>(c.suppression_interval_ms);
            o.fire.ammo_floor=c.suppression_ammo_floor;out.reason=CoverFireReason::own_knowledge;
            out.phase=w.now_ms<saturated_after(hit->at_ms,c.return_fire_ms)?CoverFirePhase::returning:CoverFirePhase::crossing_overlap;
        }
        lines_.erase(std::remove_if(lines_.begin(),lines_.end(),[](const Line& l){return l.clear==Evidence::unknown;}),lines_.end());
        return out;
    }
    // Unknown geometry is retried after the bounded nearest-first pass, never
    // converted to a clear line or allowed to starve all farther candidates.
    lines_.erase(std::remove_if(lines_.begin(),lines_.end(),[](const Line& l){return l.clear==Evidence::unknown;}),lines_.end());
    return out;
}
}}}
