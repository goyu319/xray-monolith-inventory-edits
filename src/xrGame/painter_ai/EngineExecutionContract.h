#pragma once
#include "CombatContract.h"
#include <cmath>
namespace painter { namespace combat { namespace engine {
struct CaptureAdmission {bool capture=false,refresh_compatibility=false,admitted=false;};
// Cheap registry-side gate, before Lua eligibility or full engine snapshot work.
// A previous participant gets one final handback/death snapshot; passive ALife
// actors do not allocate snapshots or retain source proofs.
inline CaptureAdmission capture_admission(bool enabled,bool alive,bool owned,bool has_order,
    bool explicitly_requested,bool perceived_contact,bool handback_pending,bool previously_reported){
    CaptureAdmission p;p.admitted=enabled&&alive&&!handback_pending&&(owned||explicitly_requested||perceived_contact);
    p.capture=p.admitted||owned||has_order||handback_pending||previously_reported;
    p.refresh_compatibility=p.admitted;
    return p;
}
struct CombatMembershipLease {
    bool acquired=false;
    bool begin(bool already_registered){acquired=!already_registered;return acquired;}
    bool finish(bool member_still_present,bool registered_now){bool remove=acquired&&member_still_present&&registered_now;acquired=false;return remove;}
};
struct HandbackNotice {
    std::uint32_t reasons=yield_none;
    bool queue(std::uint32_t reason){bool first=reasons==yield_none;reasons|=reason;return first;}
    std::uint32_t consume(){auto result=reasons;reasons=yield_none;return result;}
};
inline std::uint32_t feedback_node(std::uint8_t reason,std::uint32_t goal){
    return reason==reject_goal_invalid||reason==reject_goal_inaccessible?goal:invalid_node;
}
inline bool provisional_expired(TimeMs at,TimeMs now,TimeMs deadline){return now<at||now-at>=deadline;}
inline TimeMs compatibility_interval(Config const& cfg){
    TimeMs n=cfg.compat_poll_ms;
    if(cfg.snapshot_max_age_ms<n)n=cfg.snapshot_max_age_ms;return n?n:1;
}
inline bool cannot_reload(int magazine,int suitable_total){return magazine<=0&&suitable_total<=0;}
inline std::uint32_t capture_flags(std::uint32_t explicit_flags,bool admitted){
    return (explicit_flags&~std::uint32_t(flag_admitted))|(admitted?std::uint32_t(flag_admitted):0u);
}
enum class ExecutionPhase { fresh, stale_hold, release };
struct SourceStamp {std::uint64_t sequence=0;TimeMs at=0;std::uint64_t ownership=0,restrictions=0;};
inline bool source_stamp_valid(SourceStamp const& s,std::uint64_t sequence,std::uint64_t ownership,
    std::uint64_t restrictions,TimeMs issued,TimeMs now,TimeMs max_age){
    return s.sequence==sequence&&s.ownership==ownership&&s.restrictions==restrictions
        &&s.at<=issued&&issued<=now&&s.at<=now&&now-s.at<=max_age;
}
inline ExecutionPhase execution_phase(Order const& order,TimeMs now,TimeMs stale_release_ms){
    if(now<order.issued_at_ms||now-order.issued_at_ms>=stale_release_ms)return ExecutionPhase::release;
    return now>=order.expires_at_ms?ExecutionPhase::stale_hold:ExecutionPhase::fresh;
}
inline bool finite_value(Vec3 p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
// Internal safety command only: never published, no order sequence or receipt.
inline Order provisional_hold(ActorSnapshot const& actor,TimeMs now,TimeMs maximum_age){
    Order o;o.actor=actor.key;o.goal_node=actor.node_id;o.goal_position=actor.position;
    o.pace=Pace::stand;o.stance=Stance::crouch;o.look.kind=LookKind::direction;o.look.value=actor.forward;
    TimeMs latest=0;bool hit_found=false;
    for(std::size_t i=0;i<actor.hits.count&&i<actor.hits.items.size();++i){auto const& hit=actor.hits.items[i];
        if(hit.at_ms<=now&&now-hit.at_ms<=maximum_age&&finite_value(hit.dir)
           &&(hit.dir.x*hit.dir.x+hit.dir.z*hit.dir.z)>.0001f&&(!hit_found||hit.at_ms>=latest)){
            o.look.value={hit.dir.x,0,hit.dir.z};latest=hit.at_ms;hit_found=true;}}
    auto const& enemy=actor.enemy;
    if(!hit_found&&enemy.key.id!=invalid_id&&enemy.key.generation&&enemy.hostile&&enemy.alive_when_observed
       &&enemy.observed_at_ms<=now&&now-enemy.observed_at_ms<=maximum_age&&finite_value(enemy.position)){
        o.look.kind=enemy.visible?LookKind::enemy:LookKind::point;o.look.target=enemy.key;o.look.value=enemy.position;
    }
    return o;
}
inline float absolute(float v){return v<0?-v:v;}
inline bool same_value(Vec3 a,Vec3 b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
struct MovementRequest {
    std::uint32_t node=invalid_node;Vec3 position;Pace pace=Pace::stand;Stance stance=Stance::stand;
    bool operator==(MovementRequest const& b)const{return node==b.node&&same_value(position,b.position)&&pace==b.pace&&stance==b.stance;}
    bool operator!=(MovementRequest const& b)const{return !(*this==b);}
};
inline bool firing_window(Order const& o,TimeMs now,ExecutionPhase phase){
    return phase==ExecutionPhase::fresh&&o.fire.allowed&&o.fire.until_ms>now&&o.fire.until_ms<=o.expires_at_ms;
}
inline bool targeted_fire(bool visible,bool exact,bool hostile,bool alive,bool wounded,bool finish_allowed,
    bool reloading,bool ammo,bool sense,bool enemy_ray,bool friend_ray,float yaw,float limit){
    return visible&&exact&&hostile&&alive&&(!wounded||finish_allowed)&&!reloading&&ammo&&sense&&enemy_ray&&!friend_ray
        &&std::isfinite(yaw)&&std::isfinite(limit)&&yaw>=0&&limit>=0&&yaw<=limit;
}
inline bool suppression_fire(bool known_point,bool reloading,bool ammo_floor,bool friend_ray,float yaw,float limit){
    return known_point&&!reloading&&ammo_floor&&!friend_ray&&std::isfinite(yaw)&&std::isfinite(limit)&&yaw>=0&&limit>=0&&yaw<=limit;
}
inline bool known_fire_point(ActorSnapshot const& actor,Order const& o,TimeMs now,TimeMs max_age){
    auto matches=[&](Contact const& c){
        // Like Mailbox::contact_valid, current sight remains current even if
        // the native memory-change clock has not advanced while watching.
        const bool current_sight=c.visible&&c.last_seen_ms&&c.last_seen_ms<=actor.observed_at_ms&&
            c.observed_at_ms<=actor.observed_at_ms&&actor.observed_at_ms<=now&&now-actor.observed_at_ms<=max_age;
        return c.key.id!=invalid_id&&c.key.generation&&c.hostile&&c.alive_when_observed
            &&(!c.wounded||c.finish_allowed)&&c.observed_at_ms<=now&&(current_sight||now-c.observed_at_ms<=max_age)
            &&(o.look.target.id==invalid_id||o.look.target==c.key)&&finite_value(c.position)
            &&absolute(o.fire.at.x-c.position.x)<.01f&&absolute(o.fire.at.y-c.position.y)<.01f&&absolute(o.fire.at.z-c.position.z)<.01f;
    };
    if(matches(actor.enemy))return true;
    if(actor.visible_enemies.count>actor.visible_enemies.items.size())return false;
    for(std::size_t i=0;i<actor.visible_enemies.count;++i)if(matches(actor.visible_enemies.items[i]))return true;
    return false;
}
enum class PublicationFireClosure { none, expired, point_unproven };
inline PublicationFireClosure constrain_publication_fire(Order& o,TimeMs now,bool point_known,bool point_proven){
    if(!o.fire.allowed)return PublicationFireClosure::none;
    const auto why=o.fire.until_ms<=now?PublicationFireClosure::expired:
        o.fire.at_valid&&!point_known&&!point_proven?PublicationFireClosure::point_unproven:PublicationFireClosure::none;
    // Losing an optional firing opportunity cannot discard a still-valid
    // movement order. Do not relax its lease, lifetime, goal or other checks.
    if(why!=PublicationFireClosure::none){o.fire.allowed=false;o.fire.at_valid=false;o.fire.until_ms=now;}
    return why;
}
// A stationary hold at the body's actual current location is not a route.
// Engine AI-cell ownership can lag a physical push across a cell boundary;
// permit that exact nearby anchor, never an off-cell movement destination.
inline bool current_stationary_anchor(const Order& o,std::uint32_t current_node,Vec3 current_position){
    if(o.pace!=Pace::stand||o.kind!=OrderKind::hold||o.goal_node!=current_node||current_node==invalid_node||
        !finite_value(o.goal_position)||!finite_value(current_position)||!std::isfinite(o.arrival_radius_m)||o.arrival_radius_m<=0)return false;
    const double x=double(o.goal_position.x)-current_position.x,y=double(o.goal_position.y)-current_position.y,z=double(o.goal_position.z)-current_position.z;
    return x*x+y*y+z*z<=double(o.arrival_radius_m)*o.arrival_radius_m;
}
// Per-actor serial ledger. Refreshing order_seq never restarts a throw. Serials
// increase for the actor generation, including across ownership hand-backs.
struct ThrowExecution {
    std::uint32_t serial=0;std::uint16_t item=invalid_id,weapon=invalid_id;
    std::uint32_t baseline_releases=0;bool have_baseline=false,completion_notified=false;
    std::uint64_t ownership=0;TimeMs began=0,submitted_at=0,solve_at=0,blocked_at=0,armed_at=0;
    bool have_armed=false;
    bool started=false,equipped=false,submitted=false,terminal=false,ended=false,have_solve=false,have_blocked=false;
    std::uint8_t reason=throw_ok;Vec3 target,via;bool via_valid=false;
    bool same_request(Order const& o)const {
        return o.throw_order.valid&&o.throw_order.serial==serial&&o.ownership_generation==ownership
            &&(o.throw_order.item==invalid_id||o.throw_order.item==item)
            &&same_value(o.throw_order.target,target)&&o.portal.valid==via_valid
            &&(!via_valid||same_value(o.portal.centre,via));
    }
    bool start(Order const& o,std::uint16_t slot,std::uint16_t current_weapon,TimeMs now,std::uint32_t releases=0){
        if(!o.throw_order.valid||!o.throw_order.serial||o.throw_order.serial<=serial||slot==invalid_id
            ||(o.throw_order.item!=invalid_id&&o.throw_order.item!=slot))return false;
        serial=o.throw_order.serial;item=slot;weapon=current_weapon;ownership=o.ownership_generation;
        began=now;submitted_at=solve_at=blocked_at=armed_at=0;have_armed=false;started=true;equipped=submitted=terminal=ended=have_solve=have_blocked=false;reason=throw_ok;
        baseline_releases=releases;have_baseline=completion_notified=false;
        target=o.throw_order.target;via=o.portal.centre;via_valid=o.portal.valid;return true;
    }
    bool fail(std::uint8_t why){if(!started||terminal)return false;terminal=true;reason=why;return true;}
    bool rejected_start(Order const& o,TimeMs now,std::uint8_t why){
        if(!o.throw_order.valid||!o.throw_order.serial||o.throw_order.serial<=serial)return false;
        serial=o.throw_order.serial;item=o.throw_order.item;ownership=o.ownership_generation;began=now;
        target=o.throw_order.target;via=o.portal.centre;via_valid=o.portal.valid;
        started=terminal=true;submitted=equipped=ended=false;reason=why;return true;
    }
    bool missing_item(bool matches){return !matches&&!submitted&&fail(throw_no_grenade);}
    bool receipt(decltype(ActorSnapshot{}.throw_state) const& r,TimeMs now,RoomConfig const& cfg=RoomConfig{}){
        if(!started||terminal||r.serial!=serial||r.at_ms<began||r.at_ms>now)return false;
        if(r.phase==4)return fail(r.reason?r.reason:std::uint8_t(throw_cancelled));
        if(r.phase==2&&!have_armed){have_armed=true;armed_at=r.at_ms;}
        // The matching prepared capture latches the producer's count before
        // our Fire1. Slot loss, animation and stock last_throw_time are not proof.
        if(!submitted&&(r.phase==1||r.phase==2)){baseline_releases=r.releases;have_baseline=true;return false;}
        const std::uint32_t advanced=r.releases-baseline_releases;
        if(r.phase==3&&r.reason==throw_ok&&submitted&&have_baseline&&advanced&&advanced<0x80000000u
            &&r.projectile<invalid_id&&r.source_item==item&&r.released_at_ms>=submitted_at
            &&r.released_at_ms<=now&&r.released_at_ms<=r.at_ms&&r.released_at_ms>=(have_armed?armed_at:submitted_at)
            &&r.released_at_ms-(have_armed?armed_at:submitted_at)<cfg.frag_release_cap_ms){
            terminal=true;reason=throw_ok;return true;
        }
        return false;
    }
    bool take_completion(){if(!terminal||reason!=throw_ok||!submitted||completion_notified)return false;completion_notified=true;return true;}
    bool timeout(TimeMs now,RoomConfig const& cfg=RoomConfig{}){
        if(!started||terminal)return false;
        const auto release_start=have_armed?armed_at:submitted_at;
        if(now<began||((have_armed||submitted)&&(now<release_start||now-release_start>=cfg.frag_release_cap_ms))
            ||(!have_armed&&!submitted&&((!have_solve&&now-began>=cfg.frag_hand_cap_ms)||now-began>=cfg.frag_prefire_cap_ms)))return fail(throw_timeout);
        return false;
    }
    bool arc_due(TimeMs now,RoomConfig const& cfg=RoomConfig{})const{return started&&!terminal&&!submitted&&(!have_solve||now<solve_at||now-solve_at>=cfg.frag_arc_poll_ms);}
    bool arc(bool clear,std::uint8_t why,TimeMs now,RoomConfig const& cfg=RoomConfig{}){
        if(!started||terminal||submitted)return false;solve_at=now;have_solve=true;
        if(clear){submitted=true;submitted_at=now;return true;}
        if(why==throw_no_target||why==throw_no_grenade){if(now<began||now-began>=cfg.frag_hand_cap_ms)fail(why);return false;}
        if(!have_blocked){blocked_at=now;have_blocked=true;}
        if(now<blocked_at||now-blocked_at>=cfg.frag_arc_wait_ms)fail(why?why:std::uint8_t(throw_no_arc));return false;
    }
    bool active(std::uint64_t lease)const{return started&&ownership==lease&&!ended;}
};
// Current authority-thread blast clearance is independent of a valid trajectory.
// Unknown/unsafe refuses the optional throw before Fire1 can arm the grenade;
// it follows the existing terminal failure/restore path and cannot hold GO forever.
inline bool authorize_throw_arc(ThrowExecution& execution,bool arc_clear,std::uint8_t arc_reason,
    TimeMs now,RoomConfig const& cfg,Evidence blast_clear){
    if(arc_clear&&blast_clear!=Evidence::yes){execution.fail(throw_blocked);return false;}
    return execution.arc(arc_clear,arc_reason,now,cfg);
}
// Remove only the active-item yields caused by our exact throw lease. A world
// grenade danger, foreign item, wounded state or script veto still wins.
inline std::uint32_t throw_item_yields(std::uint32_t yields,bool own,bool same_item,bool no_item,bool /*rifle*/,bool grenade_danger){
    if(!own||(!same_item&&!no_item))return yields;
    yields&=~std::uint32_t(yield_no_weapon);
    if(same_item&&!grenade_danger)yields&=~std::uint32_t(yield_grenade);
    return yields;
}
inline bool aid_enemy_blocks(bool visible,bool alive,bool wounded){return visible&&alive&&!wounded;}
inline bool aid_snapshot_allows(ActorSnapshot const& casualty,ActorKey helper,TimeMs now){
    auto const& a=casualty.aid_eligibility;
    // These stable facts are pushed on change, not periodically restamped.
    // Unknown/unresolved claimants cannot inherit a recycled helper ID.
    return casualty.alive&&casualty.key.id!=invalid_id&&casualty.key.generation&&!(casualty.key==helper)
        &&a.allowed==Evidence::yes&&a.observed_at_ms<=now&&(!a.claimed||a.claimant==helper);
}
// Weapon channel only: never turn a room movement order into a reload/stand
// assignment. Reserve evidence must refer to the actual currently held weapon.
inline bool weapon_reload_request(Order const& o,ExecutionPhase phase,bool hands_busy,
    bool native_reloading,int magazine,std::uint16_t held_weapon,std::uint16_t captured_weapon,std::uint16_t reserve){
    return phase==ExecutionPhase::fresh&&!hands_busy&&(o.kind==OrderKind::reload||native_reloading
        ||(magazine<=0&&held_weapon!=invalid_id&&held_weapon==captured_weapon&&reserve>0));
}
// A tactical reload with ammunition still available can yield to a newly
// visible personal threat, but only after the ordinary targeted-fire safety
// checks actually authorize Fire1. Empty and already-started reloads stay put.
inline bool evaluate_target_before_reload(bool reload_requested,bool native_reloading,int magazine,bool target_window){
    return reload_requested&&!native_reloading&&magazine>0&&target_window;
}
struct AidExecution {
    ActorKey target;std::uint64_t assignment=0;TimeMs began=0,working_at=0;std::uint8_t phase=0;bool attempted=false;
    ActorKey beam_enemy;TimeMs beam_since=0;bool beam_tracking=false,beam_permitted=false,reached_notified=false;
    bool set_target(ActorKey key,std::uint64_t generation,TimeMs now){if(key==target&&generation==assignment&&phase)return false;target=key;assignment=generation;began=now;working_at=0;phase=1;attempted=false;
        beam_enemy={};beam_since=0;beam_tracking=beam_permitted=reached_notified=false;return true;}
    std::uint8_t update(TimeMs now,bool valid,bool reached,bool threat,bool recently_hit,RoomConfig const& cfg=RoomConfig{},ActorKey enemy={},bool enemy_firing=false){
        if(!phase||phase>=3)return phase;
        if(!valid||!cfg.aid_cap_ms||now<began||now-began>=cfg.aid_cap_ms){phase=4;return phase;}
        beam_permitted=false;
        if(threat){const bool identity=enemy.id<invalid_id&&enemy.generation;
            if(!identity){beam_tracking=false;beam_enemy={};}
            else if(!beam_tracking||!(beam_enemy==enemy)||enemy_firing||recently_hit||now<beam_since){beam_tracking=true;beam_enemy=enemy;beam_since=now;}
            beam_permitted=identity&&beam_tracking&&!enemy_firing&&!recently_hit&&now>=beam_since&&now-beam_since>=cfg.aid_beam_ok_ms;
        }else{beam_tracking=false;beam_enemy={};}
        if(!reached||(threat&&!beam_permitted)||recently_hit){working_at=0;phase=1;return phase;}
        if(phase!=2)working_at=now;phase=2;return phase;
    }
    bool ready(TimeMs now,RoomConfig const& cfg=RoomConfig{})const{return phase==2&&!attempted&&cfg.aid_cap_ms
        &&now>=working_at&&now>=began&&now-began<cfg.aid_cap_ms&&now-working_at>=cfg.aid_work_ms;}
    void completed(bool acknowledged){attempted=true;phase=acknowledged?3:4;}
    bool cancel(){if(!phase||phase>=3)return false;phase=4;working_at=0;return true;}
    bool take_reached(){if(phase!=2||reached_notified)return false;reached_notified=true;return true;}
    bool owns_hands()const{return phase==2;}
};
inline bool treatment_stops_movement(bool throw_hands,AidExecution const& aid){return throw_hands||aid.owns_hands();}
// A refused or completed aid episode no longer authorizes travel to its old
// patient goal while the dispatcher is replacing the accepted order. Keep
// weapon/sight channels independent, including current own-threat fire.
inline bool terminal_aid_stops_movement(Order const& order,AidExecution const& aid,ExecutionPhase phase){
    return phase==ExecutionPhase::fresh&&order.aid.id!=invalid_id&&order.aid.generation&&aid.target==order.aid&&
        aid.assignment==order.assignment.generation&&aid.phase>=3;
}
inline bool aid_reached(float distance,RoomConfig const& cfg){return std::isfinite(distance)&&distance>=0
    &&std::isfinite(cfg.aid_reach_m)&&cfg.aid_reach_m>0&&distance<=cfg.aid_reach_m;}
inline bool aid_recent_hit(TimeMs last,TimeMs now,RoomConfig const& cfg){return last&&(last>now||now-last<cfg.aid_hit_quiet_ms);}
struct DoorwayLook {LookKind kind=LookKind::path;Vec3 value;ActorKey target;};
// A doorway sweep is an explicit geometric aim channel. Ordinary look dwell
// must not retain a preceding centre/path aim over its opposite-jamb target.
inline bool doorway_owns_look(const Order& o){return o.portal.valid&&(o.pie.valid||o.corner.valid);}
// A retained clear-room mission may have no recent hostile evidence. Its
// explicit doorway work still owns lean/stance/speed during that quiet period.
inline bool authored_room_pose(const Order& o){
    if(doorway_owns_look(o))return true;
    if(!o.assignment.generation)return false;
    switch(o.role){
    case roles::door_stack:case roles::door_stack_point:case roles::entry_wait:
    case roles::entry_tail:case roles::entry_hook_hard_corner:case roles::entry_deep:case roles::entry_point_clear_side:
    case roles::door_slice:case roles::portal_watch:case roles::rear_security:
        return o.assignment.portal&&(o.kind==OrderKind::stack||o.kind==OrderKind::peek||o.kind==OrderKind::entry||o.kind==OrderKind::hold_door);
    case roles::assess_pie:case roles::assess_hold:case roles::barricade_guard:
        return o.kind==OrderKind::hold_door;
    case roles::search_sector:return o.kind==OrderKind::search;
    case roles::hold:return o.kind==OrderKind::hold_door&&o.pace!=Pace::stand; // a moving room-defense reserve
    default:return false;
    }
}
inline bool execution_stand_down(const Order& o,bool quiet,bool fresh=true){return quiet&&(!fresh||!authored_room_pose(o));}
inline DoorwayLook doorway_look(Order const& o,Vec3 position,bool visible,ActorKey enemy,Vec3 enemy_position){
    if(visible)return {LookKind::enemy,enemy_position,enemy};
    if(o.pie.valid){
        // Slice past the opposite jamb, as the actor travels around the near
        // jamb. Looking at the centre does not sweep the opening's far edge.
        if(o.portal.valid&&o.side){const auto& p=o.portal;
            return {LookKind::point,{p.centre.x-p.tangent.x*o.side*p.half_width+p.normal.x*p.thickness,
                o.pie.pivot.y,p.centre.z-p.tangent.z*o.side*p.half_width+p.normal.z*p.thickness},{}};}
        return {LookKind::point,o.pie.pivot,{}};
    }
    if(o.corner.valid&&o.portal.valid){auto const& c=o.portal.centre;auto const& n=o.portal.normal;
        if((position.x-c.x)*n.x+(position.y-c.y)*n.y+(position.z-c.z)*n.z>0)
            return {LookKind::point,o.corner.point,{}};}
    return {o.look.kind,o.look.value,o.look.target};
}
inline float doorway_lean(Order const& o,Vec3 position,DoorwayLook const& look,Pace pace){
    if(pace==Pace::run||!std::isfinite(o.lean))return 0;
    if(!o.pie.valid||!o.portal.valid||o.lean==0)return o.lean;
    Vec3 direction=look.value;
    if(look.kind==LookKind::point||look.kind==LookKind::enemy){direction.x-=position.x;direction.z-=position.z;}
    else if(look.kind!=LookKind::direction)return 0;
    const float length=std::hypot(direction.x,direction.z);
    if(!std::isfinite(length)||length<=.00001f)return 0;
    // X-Ray is y-up: right of (x,z) is (z,-x). Door side labels alone
    // cannot determine the correct shoulder after the aim direction changes.
    const float side=((o.portal.centre.x-position.x)*direction.z-(o.portal.centre.z-position.z)*direction.x)/length;
    if(!std::isfinite(side)||std::abs(side)<=.00001f)return 0;
    return std::copysign(std::abs(o.lean),side);
}
inline bool executor_metadata_valid(Order const& o){
    if(o.side<-1||o.side>1||(o.throw_order.valid&&o.aid.id!=invalid_id))return false;
    if((o.pie.valid&&!finite_value(o.pie.pivot))||(o.corner.valid&&!finite_value(o.corner.point)))return false;
    if(o.throw_order.valid&&(!o.throw_order.serial||!finite_value(o.throw_order.target)))return false;
    if(o.aid.id!=invalid_id&&!o.aid.generation)return false;
    if(o.portal.valid){auto const& p=o.portal;
        if(!finite_value(p.centre)||!finite_value(p.tangent)||!finite_value(p.normal)
            ||!std::isfinite(p.half_width)||p.half_width<=0||!std::isfinite(p.thickness)||p.thickness<0)return false;
        float t=p.tangent.x*p.tangent.x+p.tangent.y*p.tangent.y+p.tangent.z*p.tangent.z;
        float n=p.normal.x*p.normal.x+p.normal.y*p.normal.y+p.normal.z*p.normal.z;
        float dot=p.tangent.x*p.normal.x+p.tangent.y*p.normal.y+p.tangent.z*p.normal.z;
        if(absolute(t-1)>.02f||absolute(n-1)>.02f||absolute(dot)>.02f)return false;
    }
    return true;
}
}}}
