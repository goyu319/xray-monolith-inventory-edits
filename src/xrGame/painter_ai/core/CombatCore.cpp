#include "CombatCore.h"
#include "PerceivedSpecies.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace painter { namespace combat { namespace core {
bool finite(const Vec3& v) { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z); }
bool valid_key(const ActorKey& k) { return k.id<invalid_id && k.generation!=0; }
float distance_squared(const Vec3& a,const Vec3& b) {
    const float x=a.x-b.x,y=a.y-b.y,z=a.z-b.z;return x*x+y*y+z*z;
}
TimeMs saturated_after(TimeMs now,TimeMs duration) {
    const auto limit=(std::numeric_limits<TimeMs>::max)();
    return duration>limit-now?limit:now+duration;
}
bool fresh(TimeMs at,TimeMs now,TimeMs age) { return at<=now && now-at<=age; }
TimeMs fire_window_until(const Config& cfg,TimeMs now,TimeMs expires) {
    return (std::min)(expires,saturated_after(now,cfg.fire_window_ms));
}
static Vec3 normalized(Vec3 v) {
    const float d=v.x*v.x+v.y*v.y+v.z*v.z;
    if (!finite(v)||!std::isfinite(d)||d<1.e-10f) return {0,0,1};
    const float k=1.f/std::sqrt(d);return {v.x*k,v.y*k,v.z*k};
}
bool personally_visible(const Contact& c,const ActorSnapshot& a) {
    // The contract's perception_seq is a memory change detector, NOT the
    // world's sequence number. visible means seen in this actor snapshot.
    return c.visible && c.last_seen_ms!=0 && c.observed_at_ms<=a.observed_at_ms&&c.last_seen_ms<=a.observed_at_ms;
}
static bool contact_allowed(const Contact& c,const ActorSnapshot& a,const Config& cfg,TimeMs now) {
    return valid_key(c.key)&&c.key!=a.key&&c.hostile&&c.alive_when_observed&&
        (!c.wounded||c.finish_allowed)&&finite(c.position)&&
        c.observed_at_ms<=a.observed_at_ms&&c.last_seen_ms<=a.observed_at_ms&&
        fresh(a.observed_at_ms,now,cfg.snapshot_max_age_ms)&&
        (personally_visible(c,a)||fresh(c.observed_at_ms,now,cfg.memory_max_age_ms));
}
const Contact* select_contact(const ActorSnapshot& a,const Config& cfg,TimeMs now) {
    if (a.visible_enemies.count>a.visible_enemies.items.size()) return nullptr;
    // Preserve a valid currently visible engine-selected target. If that
    // target is unseen, another personally visible hostile can be engaged.
    if(!a.enemy.wounded&&contact_allowed(a.enemy,a,cfg,now)&&personally_visible(a.enemy,a)&&
        std::isfinite(distance_squared(a.position,a.enemy.position))) return &a.enemy;
    const Contact* best=nullptr;float best_d=(std::numeric_limits<float>::max)();bool seen=false;
    auto consider=[&](const Contact& c) {
        if (c.wounded||!contact_allowed(c,a,cfg,now)) return;
        const auto d=distance_squared(a.position,c.position);if(!std::isfinite(d))return;
        const bool visible=personally_visible(c,a);
        if (!best||(visible&&!seen)||(visible==seen&&(d<best_d || (d==best_d &&
            (c.key.id<best->key.id || (c.key.id==best->key.id&&c.key.generation<best->key.generation)))))) {
            best=&c;best_d=d;seen=visible;
        }
    };
    consider(a.enemy);
    for(std::size_t i=0;i<a.visible_enemies.count;++i)consider(a.visible_enemies.items[i]);
    return best;
}
bool personal_evidence_at(const ActorSnapshot& a,const Config& cfg,TimeMs now,TimeMs& latest) {
    latest=0;bool found=false;
    if(!valid_key(a.key)||!fresh(a.observed_at_ms,now,cfg.snapshot_max_age_ms)||
        a.visible_enemies.count>a.visible_enemies.items.size()||
        a.hits.count>a.hits.items.size()||a.heard.count>a.heard.items.size()||a.near_misses.count>a.near_misses.items.size())return false;
    auto remember=[&](TimeMs at){
        if(at<=a.observed_at_ms&&fresh(at,now,cfg.memory_max_age_ms)){
            latest=found?(std::max)(latest,at):at;found=true;
        }
    };
    auto contact=[&](const Contact& c){
        if(!contact_allowed(c,a,cfg,now))return;
        // Current visibility is a new personal observation in this capture,
        // even when the engine memory record's change clock did not advance.
        // Remembered contacts retain their actual original evidence time.
        remember(personally_visible(c,a)?a.observed_at_ms:c.observed_at_ms);
    };
    contact(a.enemy);
    for(std::size_t i=0;i<a.visible_enemies.count;++i)contact(a.visible_enemies.items[i]);
    for(std::size_t i=0;i<a.hits.count;++i)if(finite(a.hits.items[i].dir))remember(a.hits.items[i].at_ms);
    // These are already adapter-filtered personal sound records. They justify
    // retaining attention, never a target, visibility or permission to fire.
    for(std::size_t i=0;i<a.heard.count;++i){const auto& h=a.heard.items[i];
        if(finite(h.position)&&std::isfinite(h.power)&&h.power>=0)remember(h.at_ms);
    }
    // A measured local crack is own evidence of danger, not an identified foe.
    for(std::size_t i=0;i<a.near_misses.count;++i){const auto& n=a.near_misses.items[i];
        if(finite(n.origin)&&finite(n.dir)&&std::isfinite(n.miss_m)&&n.miss_m>=0&&
            fresh(n.at_ms,now,cfg.knowledge.input_age_ms))remember(n.at_ms);
    }
    return found;
}
bool native_group_evidence_at(const ActorSnapshot& member,const ActorSnapshot& witness,
    const ObserverSpecies& observed,const Config& cfg,TimeMs now,TimeMs& latest){
    latest=0;
    if(!valid_key(member.key)||!valid_key(witness.key)||!member.native_squad||member.native_squad!=witness.native_squad||
        member.faction!=witness.faction||!member.level_epoch||member.level_epoch!=witness.level_epoch||
        !member.snapshot_seq||member.snapshot_seq!=witness.snapshot_seq||member.observed_at_ms!=now||witness.observed_at_ms!=now||
        !member.ownership_generation||!witness.ownership_generation||!(member.flags&flag_admitted)||!(witness.flags&flag_admitted)||
        !member.alive||member.wounded||member.yield_reasons!=yield_none||!witness.alive||witness.wounded||witness.yield_reasons!=yield_none||
        observed.observer!=witness.key||observed.ownership_generation!=witness.ownership_generation||
        observed.restrictions_revision!=witness.restrictions_revision||observed.captured_at_ms!=now)return false;
    for(const auto& f:observed.facts){
        if(!valid_key(f.target)||f.target==member.key||f.target==witness.key||!f.hostile||!f.alive_when_observed||f.wounded||
            !f.threat_at_ms||!fresh(f.threat_at_ms,now,cfg.memory_max_age_ms)||f.known_at_ms>now||
            static_cast<unsigned>(f.species)>static_cast<unsigned>(PerceivedSpecies::other)||
            (f.visible&&(f.known_at_ms!=now||f.threat_at_ms!=now)))continue;
        // A complete mutant-only witness cannot retain an unrelated human
        // combat controller. Unknown personally evidenced fights remain valid.
        if(observed.complete&&f.known_at_ms&&f.species==PerceivedSpecies::mutant)continue;
        latest=(std::max)(latest,f.threat_at_ms);
    }
    return latest!=0;
}
Order first_response(const ActorSnapshot& a,const Config& cfg,TimeMs now,
    std::uint64_t epoch,std::uint64_t geometry,std::uint64_t seq,CoverView covers)
{
    Order o;o.actor=a.key;o.level_epoch=epoch;o.snapshot_seq=a.snapshot_seq;o.order_seq=seq;
    o.ownership_generation=a.ownership_generation;o.restrictions_revision=a.restrictions_revision;
    o.geometry_revision=geometry;o.issued_at_ms=now;o.expires_at_ms=saturated_after(now,cfg.order_lifetime_ms);
    o.kind=OrderKind::yield;o.flags=order_first_response;o.role=roles::individual;
    o.goal_node=a.node_id;o.goal_position=finite(a.position)?a.position:Vec3{};
    o.look.kind=LookKind::direction;o.look.value=normalized(a.forward);
    const bool invalid_arrays=a.visible_enemies.count>a.visible_enemies.items.size() ||
        a.hits.count>a.hits.items.size() || a.heard.count>a.heard.items.size() ||
        a.seen_dead.count>a.seen_dead.items.size();
    if (!(a.flags&flag_admitted)||a.yield_reasons!=yield_none||!a.alive||a.wounded||
        !std::isfinite(a.health)||a.health<=0||a.weapon.id==invalid_id||
        (a.weapon.ammo_in_mag==0&&a.weapon.ammo_reserve==0)||
        !valid_key(a.key)||!a.ownership_generation||!finite(a.position)||!finite(a.forward)||
        !epoch||a.level_epoch!=epoch||!fresh(a.observed_at_ms,now,cfg.snapshot_max_age_ms)||
        !a.current_position_accessible||a.node_id==invalid_node||invalid_arrays||
        !std::isfinite(cfg.immediate_cover_distance)||cfg.immediate_cover_distance<0||
        !std::isfinite(cfg.minimum_goal_separation)||cfg.minimum_goal_separation<0||!cfg.order_lifetime_ms)
        return o;

    o.kind=(a.reloading||a.weapon.state==WeaponState::reloading||a.weapon.ammo_in_mag==0)?OrderKind::reload:OrderKind::hold;
    o.pace=Pace::stand;o.stance=Stance::crouch;
    const Contact* target=select_contact(a,cfg,now);
    if(!target) {
        const Hit* latest=nullptr;
        for(std::size_t i=0;i<a.hits.count;++i) {
            const auto& hit=a.hits.items[i];
            if(finite(hit.dir)&&hit.at_ms<=a.observed_at_ms&&fresh(hit.at_ms,now,cfg.memory_max_age_ms)&&
                (!latest||hit.at_ms>latest->at_ms))latest=&hit;
        }
        if(latest)o.look.value=normalized(latest->dir);
        return o;
    }
    o.look.kind=LookKind::point;o.look.value=target->position;
    const bool visible=personally_visible(*target,a);
    if(visible) { o.look.kind=LookKind::enemy;o.look.target=target->key; }
    o.fire.allowed=cfg.fire_window_ms!=0&&visible&&!target->wounded&&o.kind!=OrderKind::reload&&!(a.flags&flag_dont_shoot);
    o.fire.until_ms=o.fire.allowed?fire_window_until(cfg,now,o.expires_at_ms):now;
    if(o.kind==OrderKind::reload||!covers.data)return o;
    const CoverSpot* best=nullptr;float best_d=cfg.immediate_cover_distance*cfg.immediate_cover_distance;
    if(!std::isfinite(best_d))return o;
    for(std::size_t i=0;i<covers.count;++i) {
        const auto& c=covers.data[i];
        if(a.last_rejected.node==c.node&&a.last_rejected.restrictions_revision==a.restrictions_revision&&
            (a.last_rejected.reason==reject_goal_invalid||a.last_rejected.reason==reject_goal_inaccessible))continue;
        if(c.node==invalid_node||!finite(c.position)||c.accessible!=Evidence::yes||
            c.reachable!=Evidence::yes||c.protection_known!=Evidence::yes||(!c.blocked[0]&&!c.blocked[1])||
            c.geometry_revision!=geometry||c.restrictions_revision!=a.restrictions_revision||
            c.threat_key!=target->key||c.threat_observed_at_ms!=target->observed_at_ms||
            !std::isfinite(c.lo)||c.lo<0||c.lo>=1||!std::isfinite(c.dc)||c.dc<0||
            c.dc>cfg.immediate_cover_distance)continue;
        const auto d=distance_squared(a.position,c.position);if(!std::isfinite(d)||d>best_d)continue;
        bool occupied=false;
        if(covers.teammate_count&&!covers.teammate_positions)continue;
        for(std::size_t j=0;j<covers.teammate_count;++j) {
            const auto& p=covers.teammate_positions[j];
            if(!finite(p)||distance_squared(p,c.position)<cfg.minimum_goal_separation*cfg.minimum_goal_separation) {
                occupied=true;break;
            }
        }
        if(!occupied&&(!best||d<best_d||(d==best_d&&c.node<best->node))) {best=&c;best_d=d;}
    }
    if(best&&best_d>o.arrival_radius_m*o.arrival_radius_m) {
        o.kind=OrderKind::cover;o.goal_node=best->node;o.goal_position=best->position;o.pace=Pace::walk;
    }
    return o;
}
bool continuing_throw(const Order& o,const ActorSnapshot& a,TimeMs now) {
    return o.actor==a.key&&o.ownership_generation==a.ownership_generation&&
        o.throw_order.valid&&o.throw_order.serial&&o.throw_order.item!=invalid_id&&
        o.throw_order.item==a.grenades.slot_item&&a.grenades.count>0&&
        a.throw_state.serial==o.throw_order.serial&&
        (a.throw_state.phase==1||a.throw_state.phase==2)&&a.throw_state.reason<=throw_door_behind&&
        a.throw_state.at_ms<=now&&o.issued_at_ms<=now&&o.expires_at_ms>now&&
        finite(o.throw_order.target)&&!o.fire.allowed&&!o.fire.at_valid;
}
bool publishable(const Order& o,const ActorSnapshot& a,TimeMs now,std::uint64_t epoch,std::uint64_t geometry) {
    if(o.actor!=a.key||!valid_key(a.key)||!epoch||o.level_epoch!=epoch||a.level_epoch!=epoch||
        !a.ownership_generation||o.ownership_generation!=a.ownership_generation||
        o.restrictions_revision!=a.restrictions_revision||o.geometry_revision!=geometry||
        !o.order_seq||o.snapshot_seq>a.snapshot_seq||o.issued_at_ms>now||o.expires_at_ms<=now)
        return false;
    if(o.kind==OrderKind::yield)return true;
    return (a.flags&flag_admitted)&&a.alive&&!a.wounded&&std::isfinite(a.health)&&a.health>0&&
        a.yield_reasons==yield_none&&a.current_position_accessible&&
        ((a.weapon.id!=invalid_id&&(a.weapon.ammo_in_mag>0||a.weapon.ammo_reserve>0))||continuing_throw(o,a,now))&&
        o.goal_node!=invalid_node&&finite(o.goal_position)&&finite(o.look.value)&&finite(o.lean_edge)&&
        std::isfinite(o.arrival_radius_m)&&o.arrival_radius_m>0&&
        std::isfinite(o.speed_scale)&&o.speed_scale>=.3f&&o.speed_scale<=1.5f&&
        std::isfinite(o.lean)&&o.lean>=-1&&o.lean<=1&&
        (!o.fire.allowed||(!(a.flags&flag_dont_shoot)&&o.fire.until_ms>now&&
          o.fire.until_ms<=o.expires_at_ms&&finite(o.fire.at)&&
          std::isfinite(o.fire.ammo_floor)&&o.fire.ammo_floor>=0&&o.fire.ammo_floor<=1));
}
}}}
