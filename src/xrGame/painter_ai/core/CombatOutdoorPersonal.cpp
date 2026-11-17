#include "CombatOutdoorPersonal.h"
#include "OutdoorLocalNavigation.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace painter { namespace combat { namespace core {
namespace {
struct Cancelled {};
bool same(const Vec3&a,const Vec3&b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
bool positive(float f){return std::isfinite(f)&&f>0;}
bool policy_valid(const OutdoorPersonalPolicy&p){return p.personal_seen_ms&&p.personal_hit_ms&&
    positive(p.escape_radius_m)&&positive(p.escape_floor_m)&&positive(p.escape_lateral_m)&&positive(p.lean_keep_m);}
bool rejected(const ActorSnapshot&a,const Order&o){return a.last_rejected.order_seq==o.order_seq&&a.last_rejected.reason!=reject_none;}
bool applied(const Config&cfg,const WorldSnapshot&w,const ActorSnapshot&a,const Order*o){
    if(!o)return false;
    // An elapsed fire window does not stop still-valid movement/hands. Close
    // it for validation; this never renews either order or fire lifetime.
    Order movement=*o;if(movement.fire.until_ms<=w.now_ms){movement.fire.allowed=false;movement.fire.at_valid=false;}
    return publishable(movement,a,w.now_ms,w.level_epoch,w.geometry_revision)&&
        o->snapshot_seq<=w.sequence&&a.accessible_current_order&&!rejected(a,*o)&&
        a.last_applied.order_seq==o->order_seq&&a.last_applied.at_ms>=o->issued_at_ms&&
        a.last_applied.at_ms<=a.observed_at_ms&&fresh(a.observed_at_ms,w.now_ms,cfg.snapshot_max_age_ms);
}
bool moving(const ActorSnapshot&a,const Order&o){return o.pace!=Pace::stand&&
    distance_squared(a.position,o.goal_position)>o.arrival_radius_m*o.arrival_radius_m;}
bool reload(const ActorSnapshot&a){return a.reloading||a.weapon.state==WeaponState::reloading||!a.weapon.ammo_in_mag;}
const MemberKnowledge* member(const OutdoorPersonalInput&i,const WorldSnapshot&w,const ActorSnapshot&a){
    if(!i.knowledge||!i.knowledge->accepted||i.knowledge->cancelled||i.knowledge->level_epoch!=w.level_epoch||
        i.knowledge->sequence!=w.sequence||i.knowledge_index>=i.knowledge->members.size())return nullptr;
    const auto&m=i.knowledge->members[i.knowledge_index];
    return m.actor==a.key&&m.ownership_generation==a.ownership_generation&&m.side==i.side_id&&m.alive&&m.able?&m:nullptr;
}
const FoeKnowledge* foe(const MemberKnowledge*m,FoeId id){if(m)for(const auto&f:m->foes)if(f.id==id&&!f.down)return &f;return nullptr;}
bool contradicted(const ActorSnapshot&a,ActorKey k,TimeMs at){
    if(k==a.key)return true;
    const auto test=[&](const Contact&c){return c.key==k&&c.observed_at_ms>=at&&c.observed_at_ms<=a.observed_at_ms&&
        (!c.hostile||!c.alive_when_observed||c.wounded);};
    if(test(a.enemy))return true;
    for(std::size_t n=0;n<a.visible_enemies.count;++n)if(test(a.visible_enemies.items[n]))return true;
    for(std::size_t n=0;n<a.seen_dead.count;++n)if(a.seen_dead.items[n]==k)return true;
    return false;
}
bool visible(const ActorSnapshot&a,ActorKey k){
    const auto test=[&](const Contact&c){return c.key==k&&c.hostile&&c.alive_when_observed&&!c.wounded&&personally_visible(c,a);};
    if(test(a.enemy))return true;for(std::size_t n=0;n<a.visible_enemies.count;++n)if(test(a.visible_enemies.items[n]))return true;return false;
}
bool has_visible(const ActorSnapshot&a){if(visible(a,a.enemy.key))return true;
    for(std::size_t n=0;n<a.visible_enemies.count;++n)if(visible(a,a.visible_enemies.items[n].key))return true;return false;}
bool cache_valid(const OutdoorPersonalInput&i,const WorldSnapshot&w,const ActorSnapshot&a){
    const auto*c=i.cover;return c&&c->actor==a.key&&c->level_epoch==w.level_epoch&&c->snapshot_seq==w.sequence&&
        c->ownership_generation==a.ownership_generation&&c->restrictions_revision==a.restrictions_revision&&
        c->origin_node==a.node_id&&c->observed_at_ms==a.observed_at_ms;
}
bool profile_valid(const OutdoorPersonalInput&i,const WorldSnapshot&w,const ActorSnapshot&a,FoeId f,TimeMs at,const Vec3&p){
    const auto*q=i.cover_profile;if(!q)return false;const auto&s=q->source;
    return s.actor==a.key&&s.level_epoch==w.level_epoch&&s.snapshot_seq==w.sequence&&s.geometry_revision==w.geometry_revision&&
        s.ownership_generation==a.ownership_generation&&s.restrictions_revision==a.restrictions_revision&&s.at_ms==a.observed_at_ms&&
        s.origin_node==a.node_id&&same(s.actor_position,a.position)&&s.focus==f&&s.focus_at_ms==at&&same(s.focus_position,p)&&
        q->heights[0]==.7f&&q->heights[1]==1.1f&&q->heights[2]==1.7f;
}
bool spot_valid(const OutdoorPersonalInput&i,const OutdoorPersonalPolicy&p,const WorldSnapshot&w,const ActorSnapshot&a,
    const CoverSpot&c,FoeId f,TimeMs at,const Vec3&pos){
    if(c.node==invalid_node||!finite(c.position)||!std::isfinite(c.lo)||c.lo<0||c.lo>1||!std::isfinite(c.dc)||c.dc<0||
        c.geometry_revision!=w.geometry_revision||c.accessible==Evidence::no||c.reachable==Evidence::no||
        (c.accessible==Evidence::unknown?c.restrictions_revision!=0:c.restrictions_revision!=a.restrictions_revision)||
        (!p.allow_pending_navigation&&(c.accessible!=Evidence::yes||c.reachable!=Evidence::yes))||
        (a.last_rejected.node==c.node&&a.last_rejected.restrictions_revision==a.restrictions_revision&&
            (a.last_rejected.reason==reject_goal_invalid||a.last_rejected.reason==reject_goal_inaccessible)))return false;
    if(f.ghost())return !valid_key(c.threat_key)&&i.cover->knowledge_threat==f&&i.cover->knowledge_unidentified&&
        i.cover->knowledge_observed_at_ms==at&&same(i.cover->knowledge_threat_position,pos);
    return c.threat_key==f.actor&&c.threat_observed_at_ms==at&&
        i.cover->knowledge_threat==f&&i.cover->knowledge_observed_at_ms==at&&same(i.cover->knowledge_threat_position,pos);
}
const HitKnowledge* urgent_hit(const Config&cfg,const WorldSnapshot&w,const ActorSnapshot&a,const MemberKnowledge&m){
    if(!w.level_epoch||!w.sequence||!w.geometry_revision||a.level_epoch!=w.level_epoch||a.snapshot_seq!=w.sequence||
        a.observed_at_ms!=w.now_ms||m.actor!=a.key||m.ownership_generation!=a.ownership_generation||!m.alive||!m.able||
        a.visible_enemies.count>a.visible_enemies.items.size()||a.seen_dead.count>a.seen_dead.items.size())return nullptr;
    const auto&h=m.hit_from;
    if(!h.valid||h.observer!=a.key||!h.shooter.valid()||(h.shooter.ghost()&&h.shooter.side!=m.side)||
        !finite(h.point)||!finite(h.origin)||!finite(h.direction)||
        !fresh(h.at_ms,w.now_ms,cfg.knowledge.uf_escape_ms))return nullptr;
    const auto*f=foe(&m,h.shooter);
    // A real recent hit invalidates staying exposed even when the shooter is visible.
    // Keep the same attributed source, relation, cover and navigation requirements.
    if(!f||f->wounded||(!h.shooter.ghost()&&contradicted(a,h.shooter.actor,h.at_ms)))return nullptr;
    const float line2=h.direction.x*h.direction.x+h.direction.z*h.direction.z;
    return std::isfinite(line2)&&line2>0?&h:nullptr;
}
void shut_fire(Order&o,TimeMs now){o.fire.allowed=false;o.fire.at_valid=false;o.fire.until_ms=now;}
}
bool outdoor_personal_cover_focus(const Config&cfg,const WorldSnapshot&w,const ActorSnapshot&a,const MemberKnowledge&m,KnowledgeTrace&out){
    out={};const auto*h=urgent_hit(cfg,w,a,m);if(!h||a.danger.kind==DangerKind::grenade||
        first_response(a,cfg,w.now_ms,w.level_epoch,w.geometry_revision,1).kind==OrderKind::yield)return false;
    out.receiver=a.key;out.receiver_ownership_generation=a.ownership_generation;out.receiver_faction=a.faction;
    out.observer=a.key;out.source_snapshot_seq=w.sequence;out.foe=h->shooter;out.target=h->shooter.actor;
    out.position=h->point;out.observed_at_ms=h->at_ms;out.unidentified=h->shooter.ghost();
    return true;
}
OutdoorPersonalResult outdoor_personal_order(const Config&cfg,const OutdoorPersonalPolicy&p,
    const WorldSnapshot&w,const ActorSnapshot&a,const OutdoorPersonalInput&i,const std::function<bool()>&cancelled){
    OutdoorPersonalResult r;r.proposal=i.base;
    OutdoorGrenadeMemory grenade=i.grenade_memory?*i.grenade_memory:OutdoorGrenadeMemory{};
    auto poll=[&](){if(cancelled&&cancelled())throw Cancelled{};};
    try {
        poll();if(!p.enabled){r.status=OutdoorPersonalStatus::disabled;return r;}
        if(!policy_valid(p)||!w.level_epoch||!w.sequence||!w.geometry_revision||a.level_epoch!=w.level_epoch||
            a.snapshot_seq!=w.sequence||a.observed_at_ms!=w.now_ms||!std::isfinite(cfg.minimum_goal_separation)||cfg.minimum_goal_separation<0||
            !std::isfinite(cfg.cover_exposure_cost_m)||cfg.cover_exposure_cost_m<0||
            !std::isfinite(cfg.cover_unknown_cost_m)||cfg.cover_unknown_cost_m<0||
            i.base.order.actor!=a.key||i.base.order.ownership_generation!=a.ownership_generation)return r;
        const auto*m=member(i,w,a);
        const auto baseline=first_response(a,cfg,w.now_ms,w.level_epoch,w.geometry_revision,i.base.order.order_seq);
        r.status=OutdoorPersonalStatus::ready;
        auto finish=[&](OutdoorPersonalReason why){
            r.reason=why;r.proposal.order.snapshot_seq=w.sequence;
            if(r.proposal.order.fire.until_ms<=w.now_ms){r.proposal.order.fire.allowed=false;r.proposal.order.fire.at_valid=false;}
            poll();r.grenade=grenade;if(i.grenade_memory)*i.grenade_memory=grenade;return r;
        };
        auto personal=[&](const Order&o){r.proposal=ProposedOrder{};r.proposal.order=o;r.proposal.group_id=i.base.group_id;
            r.proposal.expected_previous_order_seq=i.base.expected_previous_order_seq;};
        auto carry=[&](const Order&o){
            personal(o);r.proposal.order.snapshot_seq=w.sequence;
            if(reload(a)||o.fire.until_ms<=w.now_ms){r.proposal.order.fire.allowed=false;r.proposal.order.fire.at_valid=false;
                r.proposal.order.fire.until_ms=(std::min)(o.fire.until_ms,w.now_ms);}
        };
        const bool live=applied(cfg,w,a,i.applied);r.acknowledged_motion=live&&moving(a,*i.applied);
        // Native grenade/anomaly/wound/script/weapon gates have precedence over
        // every commitment. A perceived grenade is native work even if a producer
        // temporarily omitted its yield bit; no synthetic projectile avoidance.
        if(a.yield_reasons){auto o=baseline;o.kind=OrderKind::yield;shut_fire(o,w.now_ms);personal(o);return finish(OutdoorPersonalReason::native_yield);}
        // An accepted in-hand throw can temporarily replace the rifle. Preserve
        // only the exact native transaction; never create a new one from K1.
        if(live&&continuing_throw(*i.applied,a,w.now_ms)){carry(*i.applied);return finish(OutdoorPersonalReason::hands);}
        if(baseline.kind==OrderKind::yield){personal(baseline);return finish(OutdoorPersonalReason::native_yield);}
        // Visual is the native Danger perception enum value 0. The captured
        // point/time, never a projectile lookup, anchors this response. Danger
        // has no measured fuse: use the allocated response window unchanged.
        const auto&gp=cfg.outdoor.p.solo;
        const bool remembered=grenade.actor==a.key&&grenade.epoch==w.level_epoch&&grenade.geometry==w.geometry_revision&&
            grenade.owner==a.ownership_generation&&grenade.restrictions==a.restrictions_revision&&grenade.perceived.at_ms!=0;
        if(!remembered)grenade={};
        const bool seen_grenade=a.danger.kind==DangerKind::grenade&&a.danger.perceive==0&&a.danger.at_ms&&finite(a.danger.position)&&
            fresh(a.danger.at_ms,w.now_ms,gp.grenade_response_ms)&&gp.grenade_response_ms&&
            positive(gp.grenade_react_m)&&positive(gp.grenade_safe_m)&&
            distance_squared(a.position,a.danger.position)<gp.grenade_react_m*gp.grenade_react_m;
        if(seen_grenade&&(!remembered||a.danger.at_ms>grenade.perceived.at_ms||!same(a.danger.position,grenade.perceived.position))){
            grenade={};grenade.actor=a.key;grenade.epoch=w.level_epoch;grenade.geometry=w.geometry_revision;
            grenade.owner=a.ownership_generation;grenade.restrictions=a.restrictions_revision;grenade.perceived=a.danger;
            grenade.until=saturated_after(a.danger.at_ms,gp.grenade_response_ms);
        }
        const auto*fuse=i.grenade_fuse;
        if(grenade.actor==a.key&&fuse&&fuse->actor==a.key&&fuse->epoch==w.level_epoch&&fuse->sequence==w.sequence&&
           fuse->owner==a.ownership_generation&&fuse->restrictions==a.restrictions_revision&&fuse->observed_at==a.observed_at_ms&&
           fuse->perceived_at==grenade.perceived.at_ms&&finite(fuse->position)&&same(fuse->position,grenade.perceived.position)&&
           fuse->until>fuse->observed_at){grenade.until=fuse->until;grenade.fuse_measured=true;}
        if(grenade.until>w.now_ms){
            auto o=baseline;o.kind=OrderKind::cover;o.role=roles::under_fire_cover;o.flags=order_urgent|order_face_movement;
            o.look.kind=LookKind::path;o.look.target={};o.look.value=grenade.perceived.position;
            o.expires_at_ms=(std::min)(o.expires_at_ms,grenade.until);shut_fire(o,w.now_ms);
            if(distance_squared(a.position,grenade.perceived.position)>=gp.grenade_safe_m*gp.grenade_safe_m){
                o.goal_node=a.node_id;o.goal_position=a.position;o.pace=Pace::stand;o.stance=Stance::crouch;
                personal(o);return finish(OutdoorPersonalReason::grenade_wait);
            }
            o.pace=Pace::run;o.stance=Stance::stand;o.speed_scale=gp.run_speed;o.flags|=order_sprint;
            o.goal_node=a.node_id;o.goal_position=a.position;
            const auto*x=i.escape_context;
            if(x&&x->services&&x->source.level_epoch==w.level_epoch&&x->source.snapshot_seq==w.sequence&&
               x->source.geometry_revision==w.geometry_revision&&x->source.map_hash==w.map_hash&&x->source.at_ms==w.now_ms){
                OutdoorLocalNavigation nav{cfg,w,a,*x->services,x->route_service,x->route_budget,
                    x->work_left,x->queries_left,x->work_used,x->queries_used,cancelled};
                struct Goal{std::uint32_t node;Vec3 point;};std::vector<Goal> goals;
                if(grenade.goal_node!=invalid_node)goals.push_back({grenade.goal_node,grenade.goal});
                if(x->services->cover_query&&cfg.cover_candidates&&nav.query()){
                    CoverQueryInput q;q.centre=a.position;q.radius=cfg.cover_radius;q.threats.count=1;q.threats.items[0]=grenade.perceived.position;
                    q.max_n=cfg.cover_candidates;q.min_enemy_m=0;q.max_lo=1;std::vector<CoverSpot> candidates(q.max_n);
                    const auto count=x->services->cover_query(q,candidates.data(),q.max_n);poll();
                    if(count<=q.max_n)for(unsigned n=0;n<count;++n){if(!nav.work())break;
                        if(candidates[n].node!=invalid_node&&finite(candidates[n].position))goals.push_back({candidates[n].node,candidates[n].position});}
                }
                // Eight deterministic bearings are a bounded radial search, not
                // invented nav cells. The arrival margin keeps the whole stop
                // tolerance outside the safety radius.
                const float radius=gp.grenade_safe_m+o.arrival_radius_m;
                float dx=a.position.x-grenade.perceived.position.x,dz=a.position.z-grenade.perceived.position.z;
                float norm=std::hypot(dx,dz);if(!(norm>0)){dx=a.forward.x;dz=a.forward.z;norm=std::hypot(dx,dz);}
                if(!(norm>0)){dx=0;dz=1;norm=1;}dx/=norm;dz/=norm;
                constexpr float diagonal=.7071067811865475f;
                const float bearings[8][2]={{1,0},{diagonal,diagonal},{diagonal,-diagonal},{0,1},{0,-1},{-diagonal,diagonal},{-diagonal,-diagonal},{-1,0}};
                for(const auto&v:bearings){poll();Vec3 wanted{grenade.perceived.position.x+radius*(dx*v[0]-dz*v[1]),a.position.y,
                    grenade.perceived.position.z+radius*(dz*v[0]+dx*v[1])};Vec3 goal;const auto node=nav.project(wanted,goal);
                    if(node!=invalid_node)goals.push_back({node,goal});}
                bool found=false,covered=false;float best=0;
                for(const auto&g:goals){poll();if(!nav.work())break;
                    if(distance_squared(g.point,grenade.perceived.position)<radius*radius||std::abs(g.point.y-a.position.y)>gp.cover_floor_m)continue;
                    bool occupied=!i.reservations||i.reservations->level_epoch!=w.level_epoch||i.reservations->snapshot_seq!=w.sequence;
                    if(!occupied)for(const auto&rsv:i.reservations->members){poll();if(rsv.actor!=a.key&&rsv.node==g.node){occupied=true;break;}}
                    if(occupied)continue;RouteProofReceipt proof;float length=0;
                    if(!nav.route(g.node,g.point,cfg.cover_radius,proof,length))continue;
                    Vec3 chest=g.point;chest.y+=cfg.room.look_height_m;const auto ray=nav.line(grenade.perceived.position,chest);
                    const bool hidden=ray==Evidence::no;
                    if(!found||(hidden&&!covered)||(hidden==covered&&(length<best||(length==best&&g.node<o.goal_node)))){
                        found=true;covered=hidden;best=length;o.goal_node=g.node;o.goal_position=g.point;
                    }
                }
                if(found){grenade.goal_node=o.goal_node;grenade.goal=o.goal_position;personal(o);
                    return finish(OutdoorPersonalReason::grenade_escape);}
            }
            personal(o);r.proposal.navigation_pending=true;return finish(OutdoorPersonalReason::grenade_escape);
        }
        auto base_validated=i.base.order;
        // Room planning authors a proposal before Runtime allocates its new
        // sequence. Validate its shape against the captured predecessor only;
        // actual execution still requires the separately checked applied row.
        if(!base_validated.order_seq)base_validated.order_seq=i.base.expected_previous_order_seq;
        if(base_validated.fire.until_ms<=w.now_ms){base_validated.fire.allowed=false;base_validated.fire.at_valid=false;}
        if(!publishable(base_validated,a,w.now_ms,w.level_epoch,w.geometry_revision)||i.base.order.snapshot_seq>w.sequence){r.status=OutdoorPersonalStatus::invalid;return r;}
        const auto&b=i.base.order;
        const auto*withdraw=i.withdrawal_commitment;
        if(withdraw&&withdraw->assessment_serial&&withdraw->level_epoch==w.level_epoch&&withdraw->snapshot_seq==w.sequence&&
            withdraw->geometry_revision==w.geometry_revision&&withdraw->side_id==i.side_id&&withdraw->actor==a.key&&
            withdraw->ownership_generation==a.ownership_generation&&withdraw->restrictions_revision==a.restrictions_revision&&
            b.kind==OrderKind::withdraw&&b.goal_node==withdraw->goal_node&&same(b.goal_position,withdraw->goal_position)&&
            (!withdraw->continuing||(live&&i.applied->kind==OrderKind::withdraw&&withdraw->applied_order_seq==i.applied->order_seq&&
                i.applied->goal_node==b.goal_node&&same(i.applied->goal_position,b.goal_position)))){
            if(reload(a))shut_fire(r.proposal.order,w.now_ms);
            return finish(OutdoorPersonalReason::committed_withdrawal);
        }
        const bool room_commit=(b.flags&order_committed_entry)!=0||
            (b.kind==OrderKind::withdraw&&b.assignment.generation!=0);
        if(room_commit){if(reload(a))shut_fire(r.proposal.order,w.now_ms);return finish(OutdoorPersonalReason::committed_room);}
        const auto*ack=i.bound_commitment;
        const bool bound=live&&r.acknowledged_motion&&ack&&ack->level_epoch==w.level_epoch&&ack->snapshot_seq==w.sequence&&
            ack->geometry_revision==w.geometry_revision&&ack->side_id==i.side_id&&ack->actor==a.key&&
            ack->ownership_generation==a.ownership_generation&&ack->restrictions_revision==a.restrictions_revision&&
            ack->applied_order_seq==i.applied->order_seq&&i.applied->kind==OrderKind::bound&&
            b.kind==OrderKind::bound&&b.assignment.generation==i.applied->assignment.generation&&
            same(b.goal_position,i.applied->goal_position)&&b.goal_node==i.applied->goal_node;
        if(bound){carry(*i.applied);return finish(OutdoorPersonalReason::committed_bound);}
        if((live&&valid_key(i.applied->aid)&&a.aid_state.target==i.applied->aid&&a.aid_state.phase==2&&
                a.aid_state.at_ms>=i.applied->issued_at_ms&&a.aid_state.at_ms<=w.now_ms)||b.throw_order.valid){
            if(live&&valid_key(i.applied->aid))carry(*i.applied);
            r.proposal.order.fire.allowed=false;r.proposal.order.fire.at_valid=false;
            r.proposal.order.fire.until_ms=(std::min)(r.proposal.order.fire.until_ms,w.now_ms);return finish(OutdoorPersonalReason::hands);}
        const bool reservations_valid=i.reservations&&i.reservations->level_epoch==w.level_epoch&&i.reservations->snapshot_seq==w.sequence;
        auto occupied=[&](const CoverSpot&c){
            if(!reservations_valid)return true;
            for(const auto&v:i.reservations->members){poll();++r.reservations;if(v.actor==a.key)continue;
                if(!valid_key(v.actor)||!finite(v.position)||v.node==c.node||
                    distance_squared(v.position,c.position)<cfg.minimum_goal_separation*cfg.minimum_goal_separation)return true;}
            return false;
        };
        struct GeometryCost {float fan=0,funnel=0,leaving=0,source_room=0,spotted=0,exposed=0,path=0,unknown=0;bool route_length_known=false;};
        auto geometry_cost=[&](const CoverSpot&c,const HitKnowledge&h){
            GeometryCost out;out.path=c.dc;
            const auto*x=i.escape_context;
            const bool current=x&&x->services&&x->source.level_epoch==w.level_epoch&&x->source.map_hash==w.map_hash&&
                x->source.geometry_revision==w.geometry_revision&&x->source.snapshot_seq==w.sequence&&
                x->source.at_ms==w.now_ms&&x->source.side==i.side_id;
            auto work=[&](){poll();if(!current||!x->work_left||!*x->work_left)return false;
                --*x->work_left;if(x->work_used)++*x->work_used;return true;};
            auto query=[&](){if(!work()||!x->queries_left||!*x->queries_left)return false;
                --*x->queries_left;if(x->queries_used)++*x->queries_used;return true;};
            if(!current){out.unknown=1;return out;}
            const auto&svc=*x->services;const auto*g=svc.buildings;
            if(g&&g->level_epoch==w.level_epoch&&g->map_hash==w.map_hash&&g->locate&&work()){
                const auto here=g->locate(a.node_id);poll();const auto dest=g->locate(c.node);poll();
                out.leaving=here.first!=invalid_room&&dest.first!=here.first?1.f:0.f;
                // Project the source's personally attributed point, never the
                // enemy object's live location. An absent projection is unknown.
                if(svc.project&&positive(cfg.outdoor.p.solo.project_horizontal_m)&&positive(cfg.outdoor.p.solo.project_vertical_m)&&query()){
                    Vec3 snapped;const auto source_node=svc.project(h.point,cfg.outdoor.p.solo.project_horizontal_m,
                        cfg.outdoor.p.solo.project_vertical_m,&snapped);poll();
                    if(source_node!=invalid_node&&finite(snapped)&&
                        std::fabs(snapped.y-h.point.y)<=cfg.outdoor.p.solo.project_vertical_m&&
                        (snapped.x-h.point.x)*(snapped.x-h.point.x)+(snapped.z-h.point.z)*(snapped.z-h.point.z)<=
                            cfg.outdoor.p.solo.project_horizontal_m*cfg.outdoor.p.solo.project_horizontal_m){
                        const auto source_room=g->locate(source_node);poll();
                        if(source_room.first!=invalid_room&&source_room==dest&&source_room!=here)out.source_room=1;
                    }else out.unknown+=1;
                }else out.unknown+=1;
                const Building*building=nullptr;
                for(const auto&bld:g->buildings){if(!work()){out.unknown+=1;break;}if(bld.id==here.first){building=&bld;break;}}
                if(building){
                    // Identify only an actual finite portal crossed by this own
                    // attributed shot segment. A portal's AABB is not a ray.
                    std::uint32_t shot=0;float first=2;bool shot_known=false;
                    for(const auto&door:building->portals){if(!work()){out.unknown+=1;break;}
                        if(!finite(door.normal)||!finite(door.centre)||!positive(door.half_width))continue;
                        const float nn=std::sqrt(door.normal.x*door.normal.x+door.normal.z*door.normal.z);if(!positive(nn))continue;
                        const float nx=door.normal.x/nn,nz=door.normal.z/nn;
                        const float v0=(h.origin.x-door.centre.x)*nx+(h.origin.z-door.centre.z)*nz;
                        const float v1=(h.point.x-door.centre.x)*nx+(h.point.z-door.centre.z)*nz;
                        if((v0<0)==(v1<0)||v0==v1)continue;const float t=v0/(v0-v1);
                        const Vec3 cross{h.origin.x+(h.point.x-h.origin.x)*t,h.origin.y+(h.point.y-h.origin.y)*t,
                            h.origin.z+(h.point.z-h.origin.z)*t};
                        const float u=(cross.x-door.centre.x)*(-nz)+(cross.z-door.centre.z)*nx;
                        const float chest=cross.y+(i.cover_profile?i.cover_profile->heights[1]:0);
                        if(t>=0&&t<=1&&std::fabs(u)<=door.half_width&&chest>=door.y0&&chest<=door.y1&&t<first){first=t;shot=door.id;shot_known=true;}
                    }
                    for(const auto&door:building->portals){if(!work()){out.unknown+=1;break;}
                        if(!door.traversable||!finite(door.normal)||!finite(door.centre)||!positive(door.half_width)||
                            !std::isfinite(door.wall_depth)||door.wall_depth<0)continue;
                        const float nn=std::sqrt(door.normal.x*door.normal.x+door.normal.z*door.normal.z);if(!positive(nn))continue;
                        const float nx=door.normal.x/nn,nz=door.normal.z/nn;
                        const float along=(c.position.x-door.centre.x)*nx+(c.position.z-door.centre.z)*nz;
                        const float across=(c.position.x-door.centre.x)*(-nz)+(c.position.z-door.centre.z)*nx;
                        const float chest=c.position.y+(i.cover_profile?i.cover_profile->heights[1]:0);
                        if(chest<door.y0||chest>door.y1)continue;
                        if(std::fabs(across)<=door.half_width+cfg.room.entry_lane_half_m&&
                            std::fabs(along)<=door.wall_depth+cfg.room.entry_lane_depth_m)out.funnel+=1;
                        if(shot_known&&door.id==shot)continue;
                        const std::vector<Lit>*table=dest.second==door.room_a?&door.lit_a:dest.second==door.room_b?&door.lit_b:nullptr;
                        if(!table)continue;
                        const auto lit=std::lower_bound(table->begin(),table->end(),c.node,[](const Lit&v,std::uint32_t node){return v.node<node;});
                        if(lit!=table->end()&&lit->node==c.node)out.fan+=float(lit->standing)/255.f;
                        else if(!g->lit_complete)out.unknown+=1;
                    }
                }
            }else out.unknown+=1;
            bool route=false;
            if(x->paths)for(const auto&path:*x->paths){if(!work()){out.unknown+=1;break;}
                if(path.actor!=a.key||path.owner!=a.ownership_generation||path.restrictions_revision!=a.restrictions_revision||
                    path.origin_node!=a.node_id||path.goal_node!=c.node||!same(path.goal_position,c.position)||
                    path.complete!=Evidence::yes||path.points.size()<2||!same(path.points.front(),a.position)||!same(path.points.back(),c.position))continue;
                float distance=0,exposed=0;bool complete=true;
                for(std::size_t n=1;n<path.points.size();++n){
                    if(!work()||!finite(path.points[n-1])||!finite(path.points[n])){complete=false;break;}
                    const float length=std::sqrt(distance_squared(path.points[n-1],path.points[n]));if(!std::isfinite(length)){complete=false;break;}distance+=length;
                    // Midpoint samples price the actual supplied polyline. They
                    // are sampled exposure, never proof of an unseen whole path.
                    if(!svc.rays||!i.cover_profile||!query()){complete=false;break;}
                    Segment ray;ray.a=h.point;ray.a.y+=i.cover_profile->heights[2];
                    ray.b={(path.points[n-1].x+path.points[n].x)*.5f,(path.points[n-1].y+path.points[n].y)*.5f+i.cover_profile->heights[1],
                        (path.points[n-1].z+path.points[n].z)*.5f};float hit=-2;svc.rays(&ray,&hit,1);poll();
                    if(hit==-1.f)exposed+=length;else if(!std::isfinite(hit)||hit<0){complete=false;break;}
                }
                if(complete){out.path=distance;out.route_length_known=true;out.exposed=exposed;route=true;}else out.unknown+=1;break;
            }
            if(!route)out.unknown+=1;
            if(i.knowledge)for(const auto&side:i.knowledge->sides){if(!work()){out.unknown+=1;break;}if(side.side!=i.side_id)continue;
                for(const auto&hit:side.hits){if(!work()){out.unknown+=1;break;}
                    if(hit.valid&&valid_key(hit.observer)&&finite(hit.origin)&&fresh(hit.at_ms,w.now_ms,cfg.knowledge.uf_escape_ms)&&
                        distance_squared(hit.origin,c.position)<=b.arrival_radius_m*b.arrival_radius_m)out.spotted+=1;}
            }
            return out;
        };
        auto find_spot=[&](FoeId f,TimeMs at,const Vec3&pos,const HitKnowledge*hit,bool keep)->const CoverSpot*{
            if(!cache_valid(i,w,a))return nullptr;
            const bool measured=profile_valid(i,w,a,f,at,pos);const CoverSpot*best=nullptr;float bestcost=0;bool besthidden=false;
            float best_crowding=0,best_line_deficit=0;unsigned besttier=0;
            for(const auto&c:i.cover->spots){poll();++r.candidates;
                if(!spot_valid(i,p,w,a,c,f,at,pos))continue;
                float crowding=0,line_deficit=0;
                if(keep){if(occupied(c))continue;}
                else {
                    // A current reservation on this exact node is unavailable;
                    // proximity is a cost, not a veto of every escape cell.
                    if(!reservations_valid)continue;
                    bool unavailable=false;
                    for(const auto&v:i.reservations->members){poll();++r.reservations;if(v.actor==a.key)continue;
                        if(!valid_key(v.actor)||!finite(v.position)||v.node==c.node){unavailable=true;break;}
                        const float d2=distance_squared(v.position,c.position);
                        if(!std::isfinite(d2)){unavailable=true;break;}
                        crowding=(std::max)(crowding,(std::max)(0.f,cfg.minimum_goal_separation-std::sqrt(d2)));
                    }
                    if(unavailable)continue;
                }
                const float d2=distance_squared(a.position,c.position);if(!std::isfinite(d2))continue;
                if(keep){if(!live||c.node!=i.applied->goal_node||!same(c.position,i.applied->goal_position)||
                        !measured||c.protection_known!=Evidence::yes||!(c.blocked[0]||c.blocked[1])||c.lo>=1)continue;
                } else {
                    if(d2<=b.arrival_radius_m*b.arrival_radius_m||d2>p.escape_radius_m*p.escape_radius_m||
                        c.dc>p.escape_radius_m||std::fabs(c.position.y-a.position.y)>p.escape_floor_m)continue;
                    const float dx=hit->direction.x,dz=hit->direction.z,len=std::sqrt(dx*dx+dz*dz);
                    if(!std::isfinite(len)||len<=0)continue;
                    const float off=std::fabs((c.position.x-hit->origin.x)*dz-(c.position.z-hit->origin.z)*dx)/len;
                    const bool hidden=measured&&c.protection_known==Evidence::yes&&c.blocked[1];
                    // Relax lateral separation before giving up a real reachable
                    // cell. No hidden/cover-class boolean is an escape veto.
                    line_deficit=hidden?0.f:(std::max)(0.f,p.escape_lateral_m-off);
                }
                const bool hidden=measured&&c.protection_known==Evidence::yes&&c.blocked[1];
                const bool unknown=!measured||c.protection_known==Evidence::unknown;
                GeometryCost geo;geo.path=c.dc;if(!keep)geo=geometry_cost(c,*hit);
                if(!keep&&geo.route_length_known&&geo.path>p.escape_radius_m)continue;
                const bool classified=measured&&c.protection_known==Evidence::yes&&(c.blocked[0]||c.blocked[1]||c.blocked[2]);
                // Relax proximity, then another doorway's fan, then class.
                // Within the tier all remaining measured disadvantages cost;
                // none discards the last valid cell.
                const unsigned tier=keep?0:!classified?3:geo.fan>0?2:crowding>0?1:0;
                const float cost=geo.path+geo.exposed+((unknown||!(c.blocked[0]||c.blocked[1]))?1.f:c.lo)*cfg.cover_exposure_cost_m+
                    (float(unknown)+float(c.accessible==Evidence::unknown||c.reachable==Evidence::unknown)+geo.unknown)*cfg.cover_unknown_cost_m+
                    geo.funnel*cfg.room.funnel_cost_m+(geo.leaving+geo.spotted)*cfg.cover_unknown_cost_m+geo.source_room*cfg.room.exposure_cost_m;
                if(!std::isfinite(cost))continue;
                // Ordered, dimensioned costs use captured facts and existing
                // settings only: crowding, residual shot-line exposure, then
                // native distance/cover cost. Every valid cell remains eligible.
                const bool cheaper=!best||tier<besttier||(tier==besttier&&(crowding<best_crowding||
                    (crowding==best_crowding&&(line_deficit<best_line_deficit||
                    (line_deficit==best_line_deficit&&(cost<bestcost||(cost==bestcost&&c.node<best->node)))))));
                if(cheaper){best=&c;bestcost=cost;besthidden=hidden;best_crowding=crowding;best_line_deficit=line_deficit;besttier=tier;}
            }
            if(best){r.proposal.cover_cost=bestcost;r.escaped_line_proven=besthidden;}return best;
        };
        auto use_spot=[&](const CoverSpot&c,Order o){const float cost=r.proposal.cover_cost;personal(o);
            r.proposal.order.goal_node=c.node;r.proposal.order.goal_position=c.position;r.proposal.has_cover_evidence=true;r.proposal.cover_evidence=c;
            r.proposal.cover_cost=cost;r.proposal.navigation_pending=c.accessible!=Evidence::yes||c.reachable!=Evidence::yes;
            r.proposal.has_knowledge=true;auto&k=r.proposal.knowledge;k.receiver=a.key;k.receiver_ownership_generation=a.ownership_generation;
            k.receiver_faction=a.faction;k.observer=a.key;k.source_snapshot_seq=w.sequence;k.foe=i.cover->knowledge_threat;
            k.target=k.foe.actor;k.position=i.cover->knowledge_threat_position;k.observed_at_ms=i.cover->knowledge_observed_at_ms;
            k.unidentified=k.foe.ghost();k.personally_visible=!k.foe.ghost()&&visible(a,k.target);
        };
        // Continue only actual-applied, current-threat revalidated cover, not an
        // issued proposal. Without an urgent hit, reload only finishes that move.
        auto continued=[&]()->const CoverSpot*{
            if(!live||!r.acknowledged_motion||(i.applied->kind!=OrderKind::cover&&i.applied->kind!=OrderKind::reload)||i.applied->assignment.generation||
                valid_key(i.applied->aid)||i.applied->throw_order.valid)return nullptr;
            // An inferred shooter has no native Contact. Preserve a genuinely
            // applied escape only against the same own attributed hit, with a
            // fresh matching geometry query and its original observation time.
            if(m&&i.cover&&i.cover->knowledge_threat.ghost()){
                const auto*h=urgent_hit(cfg,w,a,*m);
                if(h&&h->shooter==i.cover->knowledge_threat&&h->at_ms==i.cover->knowledge_observed_at_ms&&
                    same(h->point,i.cover->knowledge_threat_position)&&
                    (i.applied->role==roles::under_fire_cover||i.applied->role==roles::reload_cover))
                    return find_spot(h->shooter,h->at_ms,h->point,nullptr,true);
                return nullptr;
            }
            const auto*c=select_contact(a,cfg,w.now_ms);
            // A native memory clock may be older even than memory_max_age while
            // visible_now and K1's exact own sight are current. Prefer the native
            // selected current sight, without refreshing remembered evidence.
            auto current_sight=[&](const Contact&v){
                FoeId key;key.actor=v.key;const auto*f=foe(m,key);
                if(!valid_key(v.key)||!v.hostile||!v.alive_when_observed||v.wounded||!finite(v.position)||
                    !personally_visible(v,a)||!f||!f->visible||f->wounded)return false;
                const auto&seen=f->sources[0];
                return seen.valid&&seen.reporter==a.key&&finite(seen.position)&&same(seen.position,v.position)&&
                    seen.at_ms>=v.observed_at_ms&&seen.at_ms<=a.observed_at_ms&&fresh(seen.at_ms,w.now_ms,cfg.memory_max_age_ms);
            };
            if(current_sight(a.enemy))c=&a.enemy;
            else if(!c)for(std::size_t n=0;n<a.visible_enemies.count;++n)if(current_sight(a.visible_enemies.items[n])){c=&a.visible_enemies.items[n];break;}
            if(!c||contradicted(a,c->key,c->observed_at_ms))return nullptr;
            FoeId f;f.actor=c->key;TimeMs at=c->observed_at_ms;
            // The visible capture carries current coordinates while native memory
            // may retain an older update stamp. Match only THIS observer's actual
            // K1 sight at the exact visible position; shared/heard facts cannot
            // refresh it. Same source convention as SquadTactics::knowledge_trace.
            const auto*known=foe(m,f);
            if(known&&known->visible&&personally_visible(*c,a)){
                const auto&sight=known->sources[0];
                if(sight.valid&&sight.reporter==a.key&&finite(sight.position)&&same(sight.position,c->position)&&
                    sight.at_ms>=c->observed_at_ms&&sight.at_ms<=a.observed_at_ms)at=sight.at_ms;
            }
            return find_spot(f,at,c->position,nullptr,true);
        };
        const bool reloading=reload(a)||i.tactical_reload;
        // Hands can keep reloading during an already-applied cover move.
        if(reloading){if(const auto*c=continued()){
            auto requested=baseline;requested.kind=OrderKind::reload;requested.pace=i.applied->pace;
            requested.role=roles::reload_cover;use_spot(*c,requested);
            shut_fire(r.proposal.order,w.now_ms);return finish(OutdoorPersonalReason::reload);}}
        bool any_visible=has_visible(a);
        bool contact=any_visible;
        if(m){
            const auto*f=foe(m,m->target);
            if(f&&!f->id.ghost()&&!contradicted(a,f->id.actor,f->sources[0].at_ms)&&f->sources[0].valid&&
                f->sources[0].reporter==a.key&&fresh(f->sources[0].at_ms,w.now_ms,p.personal_seen_ms))contact=true;
            if(m->has_hit&&fresh(m->last_hit_ms,w.now_ms,p.personal_hit_ms))contact=true;
        }
        // Captured own hit receipts work before the first K1 publication too.
        for(std::size_t n=0;n<a.hits.count;++n)if(finite(a.hits.items[n].dir)&&
            fresh(a.hits.items[n].at_ms,w.now_ms,p.personal_hit_ms))contact=true;
        r.personal_contact=contact;
        // Urgent escape uses K1's attributed hit line, never Danger::hit impact
        // or an attacker object's live position. Both ghost and known-but-now-
        // unseen shooters are eligible; a relation/death contradiction vetoes.
        if(const auto*hit=m?urgent_hit(cfg,w,a,*m):nullptr){
            const auto&h=*hit;
            if(const auto*c=find_spot(h.shooter,h.at_ms,h.point,&h,false)){
                auto o=baseline;o.kind=OrderKind::cover;o.role=roles::under_fire_cover;o.pace=Pace::run;o.stance=Stance::stand;o.flags=order_urgent;
                o.look.kind=LookKind::point;o.look.value=h.point;o.look.target=ActorKey{};
                if(fresh(h.at_ms,w.now_ms,cfg.knowledge.uf_run_ms))o.flags|=order_sprint;
                // Suppression/firing on arrival is a separate sourced layer.
                shut_fire(o,w.now_ms);use_spot(*c,o);return finish(OutdoorPersonalReason::unseen_escape);
            }
            // Preserve a run/seek-cover intent for Solo's broader search.
            // The current node is only an unresolved destination, not a claim
            // that an escape route or movement has succeeded.
            auto o=baseline;o.kind=OrderKind::cover;o.role=roles::seek_cover;o.pace=Pace::run;o.stance=Stance::stand;o.flags=order_urgent;
            if(fresh(h.at_ms,w.now_ms,cfg.knowledge.uf_run_ms))o.flags|=order_sprint;
            o.look.kind=LookKind::point;o.look.value=h.point;o.look.target={};shut_fire(o,w.now_ms);personal(o);
            r.proposal.navigation_pending=true;
            return finish(OutdoorPersonalReason::escape_unavailable);
        }
        // A fresh hit escapes before a stationary reload. The torso may keep
        // reloading while the legs run; no synthetic fire permission is granted.
        if(reloading){
            auto requested=baseline;
            const auto*x=i.escape_context;
            if(cache_valid(i,w,a)&&x&&x->services&&x->source.level_epoch==w.level_epoch&&x->source.snapshot_seq==w.sequence&&
               x->source.geometry_revision==w.geometry_revision&&x->source.map_hash==w.map_hash&&x->source.at_ms==w.now_ms&&
               positive(cfg.outdoor.p.solo.reload_cover_m)){
                const auto f=i.cover->knowledge_threat;const auto at=i.cover->knowledge_observed_at_ms;const auto point=i.cover->knowledge_threat_position;
                const auto*known=foe(m,f);const auto*fact=known?known->last_known():nullptr;
                const bool traced=known&&!known->wounded&&fact&&fact->valid&&valid_key(fact->reporter)&&fact->at_ms==at&&same(fact->position,point)&&
                    fresh(at,w.now_ms,cfg.memory_max_age_ms)&&(f.ghost()||!contradicted(a,f.actor,at));
                if(traced&&profile_valid(i,w,a,f,at,point)){
                    OutdoorLocalNavigation nav{cfg,w,a,*x->services,x->route_service,x->route_budget,
                        x->work_left,x->queries_left,x->work_used,x->queries_used,cancelled};
                    const CoverSpot*best=nullptr;float bestdistance=0;
                    for(const auto&c:i.cover->spots){poll();if(!nav.work())break;++r.candidates;
                        if(!spot_valid(i,p,w,a,c,f,at,point)||c.protection_known!=Evidence::yes||(!c.blocked[0]&&!c.blocked[1])||
                           occupied(c)||std::abs(c.position.y-a.position.y)>cfg.outdoor.p.solo.cover_floor_m||
                           distance_squared(a.position,c.position)>cfg.outdoor.p.solo.reload_cover_m*cfg.outdoor.p.solo.reload_cover_m)continue;
                        RouteProofReceipt proof;float length=0;
                        if(!nav.route(c.node,c.position,cfg.outdoor.p.solo.reload_cover_m,proof,length)||!nav.same_region(proof))continue;
                        // A reload trip never crosses into the threat's room.
                        // Same-region route nodes are a conservative certificate.
                        if(!best||length<bestdistance||(length==bestdistance&&c.node<best->node)){best=&c;bestdistance=length;}
                    }
                    if(best){requested.kind=OrderKind::reload;requested.role=roles::reload_cover;
                        requested.pace=distance_squared(a.position,best->position)>requested.arrival_radius_m*requested.arrival_radius_m?Pace::run:Pace::stand;
                        requested.speed_scale=cfg.outdoor.p.solo.run_speed;requested.flags=order_face_movement;
                        requested.look.kind=LookKind::path;requested.look.target={};shut_fire(requested,w.now_ms);use_spot(*best,requested);
                        r.proposal.navigation_pending=false;r.proposal.cover_evidence.accessible=r.proposal.cover_evidence.reachable=Evidence::yes;
                        r.proposal.cover_evidence.restrictions_revision=a.restrictions_revision;return finish(OutdoorPersonalReason::reload);}
                }
            }
            if(i.tactical_reload){requested.kind=OrderKind::reload;requested.goal_node=a.node_id;requested.goal_position=a.position;
                requested.pace=Pace::stand;requested.flags&=~order_first_response;}
            personal(requested);shut_fire(r.proposal.order,w.now_ms);return finish(OutdoorPersonalReason::reload);}
        // A current assigned stack/security goal is simultaneous with own
        // weapon tracking. Do not erase it merely because a foe is visible;
        // native blockers and urgent own-hit escape have already taken priority.
        if(b.assignment.generation&&b.portal.valid&&(b.kind==OrderKind::stack||b.kind==OrderKind::hold_door||
            (b.kind==OrderKind::peek&&(b.pie.valid||(b.flags&order_jamb)))))
            return finish(OutdoorPersonalReason::committed_room);
        if(m&&m->dogfight.valid()&&!m->dogfight.ghost()&&b.kind!=OrderKind::press&&b.kind!=OrderKind::rush){
            const auto*f=foe(m,m->dogfight);const auto*source=f?f->own_last_known():nullptr;
            if(f&&!f->wounded&&source&&source->reporter==a.key&&finite(source->position)&&
                fresh(source->at_ms,w.now_ms,cfg.memory_max_age_ms)&&!contradicted(a,f->id.actor,source->at_ms)){
                auto o=baseline;o.kind=OrderKind::cover;o.role=roles::dogfight;o.pace=Pace::walk;o.stance=Stance::stand;
                o.look.kind=LookKind::point;o.look.value=source->position;o.look.target=ActorKey{};
                if(visible(a,f->id.actor)){o.look.kind=LookKind::enemy;o.look.target=f->id.actor;}
                o.fire.allowed=o.fire.allowed&&f->id.actor==a.enemy.key;
                if(!o.fire.allowed)shut_fire(o,w.now_ms);
                if(distance_squared(b.goal_position,a.position)<=p.lean_keep_m*p.lean_keep_m){o.lean=b.lean;o.lean_edge=b.lean_edge;o.expose=b.expose;o.stance=b.stance;}
                personal(o);return finish(OutdoorPersonalReason::dogfight);
            }
        }
        if(const auto*c=continued()){auto o=baseline;o.kind=OrderKind::cover;o.role=i.applied->role;o.pace=i.applied->pace;use_spot(*c,o);return finish(OutdoorPersonalReason::continue_cover);}
        bool base_commitment=false;
        if(i.base_commitment&&live&&b.role==roles::base_of_fire&&i.applied->role==roles::base_of_fire&&reservations_valid){
            const auto&c=*i.base_commitment;const auto*v=c.mover;
            if(c.level_epoch==w.level_epoch&&c.snapshot_seq==w.sequence&&c.geometry_revision==w.geometry_revision&&
                c.side_id==i.side_id&&c.base==a.key&&c.ownership_generation==a.ownership_generation&&
                c.restrictions_revision==a.restrictions_revision&&c.order_seq==i.applied->order_seq&&
                c.assignment_generation==b.assignment.generation&&c.assignment_generation==i.applied->assignment.generation&&
                b.goal_node==i.applied->goal_node&&same(b.goal_position,i.applied->goal_position)&&
                v&&v->key!=a.key&&v->faction==a.faction&&v->snapshot_seq==w.sequence&&v->observed_at_ms==w.now_ms&&
                applied(cfg,w,*v,c.mover_applied)&&c.mover_applied->kind==OrderKind::bound&&
                moving(*v,*c.mover_applied)){
                for(const auto&row:i.reservations->members){poll();++r.reservations;if(row.actor==v->key){base_commitment=true;break;}}
            }
        }
        r.acknowledged_base=base_commitment;
        const bool squad_exception=base_commitment||b.kind==OrderKind::press||b.kind==OrderKind::rush||
            (b.kind==OrderKind::regroup&&!any_visible)||
            (b.role==roles::casualty_guard&&valid_key(b.aid));
        if(contact&&!squad_exception){
            // Reuse the existing ranked SOLO field-cover proposal where supplied.
            // Otherwise use the cheap personal response; never pretend this is
            // the unimplemented full retreat/shift/search tactical tree.
            const bool own_cover=!b.assignment.generation&&(b.kind==OrderKind::cover||b.kind==OrderKind::hold)&&
                (b.role==roles::seek_cover||b.role==roles::hold_cover||b.role==roles::individual||b.role==roles::under_fire_cover);
            if(!own_cover)personal(baseline);
            return finish(r.reason==OutdoorPersonalReason::escape_unavailable?r.reason:OutdoorPersonalReason::personal_contact);
        }
        return finish(r.reason==OutdoorPersonalReason::escape_unavailable?r.reason:OutdoorPersonalReason::squad);
    }catch(const Cancelled&){OutdoorPersonalResult out;out.status=OutdoorPersonalStatus::cancelled;out.proposal=i.base;return out;}
    catch(...){OutdoorPersonalResult out;out.status=OutdoorPersonalStatus::failed;out.proposal=i.base;return out;}
}
}}}
