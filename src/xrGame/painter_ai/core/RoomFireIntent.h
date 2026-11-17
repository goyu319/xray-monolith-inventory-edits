#pragma once
#include "CombatCore.h"
#include <cmath>

namespace painter { namespace combat { namespace core {
// Preserve an already bounded room suppression packet through a cheap refresh.
// This validates only the current observer's own contact. The mailbox and native
// executor still re-check permission; neither memory nor a static ray grants it.
inline bool keep_room_cover_fire(const Order& plan,const Order& response,
    const ActorSnapshot& actor,const Config& config,TimeMs now) {
    if(plan.role!=roles::suppress_buddy||!(plan.flags&order_cover_fire)||!plan.fire.allowed||!plan.fire.at_valid
        ||plan.fire.until_ms<=now||plan.expires_at_ms<=now||plan.look.kind!=LookKind::point
        ||!valid_key(plan.look.target)||!finite(plan.fire.at)||!finite(plan.look.value)
        ||response.kind==OrderKind::yield||response.kind==OrderKind::reload||actor.reloading
        ||actor.weapon.state==WeaponState::reloading||(actor.flags&flag_dont_shoot)
        ||!actor.weapon.ammo_in_mag||!actor.weapon.mag_size||actor.weapon.id==invalid_id
        ||!fresh(actor.observed_at_ms,now,config.snapshot_max_age_ms)
        ||!std::isfinite(plan.fire.ammo_floor)||plan.fire.ammo_floor<0||plan.fire.ammo_floor>1
        ||float(actor.weapon.ammo_in_mag)/float(actor.weapon.mag_size)<plan.fire.ammo_floor
        ||actor.visible_enemies.count>actor.visible_enemies.items.size())return false;
    if(response.look.kind==LookKind::enemy&&response.look.target!=plan.look.target)return false;
    const auto owns=[&](const Contact& contact) {
        return contact.key==plan.look.target&&contact.key!=actor.key&&contact.hostile&&contact.alive_when_observed
            &&(!contact.wounded||contact.finish_allowed)&&finite(contact.position)
            &&contact.observed_at_ms<=actor.observed_at_ms&&fresh(contact.observed_at_ms,now,config.memory_max_age_ms)
            // Match the mailbox's point provenance tolerance; do not retarget or
            // extend an old packet merely because the selected enemy moved.
            &&std::abs(contact.position.x-plan.fire.at.x)<.01f
            &&std::abs(contact.position.y-plan.fire.at.y)<.01f
            &&std::abs(contact.position.z-plan.fire.at.z)<.01f;
    };
    if(owns(actor.enemy))return true;
    for(std::size_t i=0;i<actor.visible_enemies.count;++i)if(owns(actor.visible_enemies.items[i]))return true;
    return false;
}
}}}
