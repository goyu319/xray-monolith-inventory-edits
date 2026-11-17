#pragma once
#include "../CombatContract.h"
#include <cmath>

namespace painter { namespace combat { namespace core {
// Private transport, never a Contact and never personal visibility. Only the
// knowledge/fire producer creates this receipt after its recipient-specific
// source and exact static line have passed. Native weapon and dynamic teammate
// checks are still mandatory. The legacy publication API does not carry it.
enum class PointFireSource : std::uint8_t { none, own_seen, own_heard, squad_seen, squad_heard, buddy_hit };
struct PointFireAuthority {
    PointFireSource source=PointFireSource::none;
    Order authored;
    ActorKey reporter;
    Vec3 known,origin,eye,ray_end;
    std::uint32_t origin_node=invalid_node;
    std::uint16_t weapon=invalid_id;
    TimeMs captured_at=0,fact_at=0,line_at=0,until=0;
};
inline bool point_value_equal(Vec3 a,Vec3 b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
inline bool point_finite(Vec3 p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
inline bool point_key(ActorKey k){return k.id<invalid_id&&k.generation!=0;}
inline bool point_fire_authorized(const PointFireAuthority& p,const Order& o,
    const ActorSnapshot& a,TimeMs now,TimeMs source_max_age,TimeMs memory_max_age)
{
    const auto& q=p.authored;
    if(p.source==PointFireSource::none||static_cast<unsigned>(p.source)>static_cast<unsigned>(PointFireSource::buddy_hit)
       ||!point_key(p.reporter)||!point_key(q.look.target)||q.look.target==q.actor||q.actor!=a.key
       ||q.actor!=o.actor||!q.order_seq||!o.order_seq||!q.level_epoch||q.level_epoch!=a.level_epoch||q.level_epoch!=o.level_epoch
       ||!q.snapshot_seq||q.snapshot_seq>a.snapshot_seq||q.snapshot_seq!=o.snapshot_seq
       ||!q.geometry_revision||q.geometry_revision!=o.geometry_revision
       ||!q.ownership_generation||q.ownership_generation!=a.ownership_generation||q.ownership_generation!=o.ownership_generation
       ||q.restrictions_revision!=a.restrictions_revision||q.restrictions_revision!=o.restrictions_revision
       ||q.look.kind!=LookKind::point||o.look.kind!=LookKind::point||q.look.target!=o.look.target
       ||!q.fire.allowed||!q.fire.at_valid||!o.fire.allowed||!o.fire.at_valid
       ||!point_value_equal(q.fire.at,o.fire.at)||!point_value_equal(q.fire.at,p.ray_end)
       ||!point_value_equal(q.look.value,o.look.value)||!point_value_equal(o.look.value,o.fire.at)
       ||!point_finite(p.known)||!point_finite(p.origin)||!point_finite(p.eye)||!point_finite(p.ray_end)
       ||point_value_equal(p.eye,p.ray_end)||p.origin_node==invalid_node||p.origin_node!=a.node_id
       ||!point_value_equal(p.origin,a.position)||!a.alive||a.wounded||a.health<=0||a.yield_reasons
       ||!(a.flags&flag_admitted)||(a.flags&flag_dont_shoot)||a.reloading||a.weapon.state==WeaponState::reloading
       ||p.weapon==invalid_id||p.weapon!=a.weapon.id||!a.weapon.ammo_in_mag||o.kind==OrderKind::yield||o.kind==OrderKind::reload||o.throw_order.valid||point_key(o.aid)
       ||!p.fact_at||p.fact_at>p.captured_at||!p.line_at||p.line_at>now||p.captured_at>a.observed_at_ms
       ||a.observed_at_ms>now||now-a.observed_at_ms>source_max_age||p.captured_at>now||now-p.captured_at>source_max_age
       ||now-p.fact_at>memory_max_age||p.until<=now||p.until>q.fire.until_ms||o.fire.until_ms>p.until
       ||o.fire.until_ms<=now||o.fire.until_ms>o.expires_at_ms||o.expires_at_ms>q.expires_at_ms
       ||o.issued_at_ms!=q.issued_at_ms||!o.fire.queue||o.fire.queue>q.fire.queue
       ||o.fire.interval_ms<q.fire.interval_ms||!std::isfinite(o.fire.ammo_floor)||o.fire.ammo_floor<q.fire.ammo_floor)
        return false;
    const bool own=p.source==PointFireSource::own_seen||p.source==PointFireSource::own_heard;
    if(own!=(p.reporter==a.key)||p.reporter==q.look.target)return false;
    // Local new contrary perception closes old shared permission; never inspect
    // another actor's private snapshot to infer a death or wound.
    auto contrary=[&](const Contact& c){return c.key==q.look.target&&c.observed_at_ms>=p.fact_at
        &&c.observed_at_ms<=a.observed_at_ms&&(!c.hostile||!c.alive_when_observed||(c.wounded&&!c.finish_allowed));};
    if(contrary(a.enemy)||a.visible_enemies.count>a.visible_enemies.items.size())return false;
    for(std::size_t i=0;i<a.visible_enemies.count;++i)if(contrary(a.visible_enemies.items[i]))return false;
    return true;
}
}}}
