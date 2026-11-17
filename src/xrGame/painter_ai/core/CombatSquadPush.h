#pragma once
#include "CombatKnowledge.h"
#include "CombatCore.h"
#include <algorithm>
#include <cmath>

namespace painter { namespace combat { namespace core {
// A squad opportunity, never an individual pursuit or route certificate.
// The director permits one free man; the movement pipeline still has to prove
// his destination and native restrictions before anything reaches the engine.
inline bool squad_push_eligible(const Config& c,const OutdoorSoloPolicy& p,
    const WorldSnapshot& w,const ActorSnapshot& a,const SideSnapshot& side,
    const MemberKnowledge& m,const Order& o) {
    if(!a.alive||a.wounded||a.yield_reasons||!(a.flags&flag_admitted)||a.faction!=side.faction||
        a.level_epoch!=w.level_epoch||a.snapshot_seq!=w.sequence||a.observed_at_ms>w.now_ms||
        w.now_ms-a.observed_at_ms>c.snapshot_max_age_ms||m.actor!=a.key||m.side!=side.side_id||
        m.ownership_generation!=a.ownership_generation||!m.alive||!m.able||m.meter_level>=3||
        !std::isfinite(a.health)||a.health<p.injured_hp||a.reloading||a.weapon.state==WeaponState::reloading||
        !a.weapon.mag_size||float(a.weapon.ammo_in_mag)/a.weapon.mag_size<p.rush_ammo||
        o.portal.valid||o.pie.valid||o.corner.valid||o.throw_order.valid||valid_key(o.aid)||valid_key(o.finish)||
        (o.flags&(order_committed_entry|order_urgent))||
        (o.role!=roles::individual&&o.role!=roles::hold&&o.role!=roles::hold_cover&&
         o.role!=roles::shoot&&o.role!=roles::mover&&o.role!=roles::suppression_push))return false;
    if(o.kind==OrderKind::entry||o.kind==OrderKind::stack||o.kind==OrderKind::withdraw||
        o.kind==OrderKind::reload||o.kind==OrderKind::regroup||o.kind==OrderKind::bound)return false;
    if(o.role!=roles::suppression_push&&o.pace!=Pace::stand)return false;
    const FactionStyle* style=&c.faction_default;
    for(std::size_t n=0;n<c.factions.count;++n)if(c.factions.items[n].faction==a.faction){style=&c.factions.items[n];break;}
    if(style->doctrine!=1||style->profile>=p.profiles.size()||!p.profiles[style->profile].can_rush)return false;
    for(const auto& f:m.foes) {
        if(f.id!=m.target||f.id.ghost()||!f.id.valid()||f.down||f.wounded||
            !f.reloading.active(w.now_ms)||!f.suppressed.active(w.now_ms)||
            f.suppression_reporter==a.key||!valid_key(f.suppression_reporter))continue;
        const auto* known=f.last_known();
        if(!known||!known->valid||!finite(known->position)||known->at_ms>w.now_ms||
            w.now_ms-known->at_ms>c.memory_max_age_ms)continue;
        for(const auto& mate:w.actors) {
            if(mate.key!=f.suppression_reporter||mate.ownership_generation!=f.suppression_owner||
                !mate.alive||mate.wounded||mate.faction!=side.faction||mate.level_epoch!=w.level_epoch||
                mate.snapshot_seq!=w.sequence||mate.observed_at_ms>w.now_ms||
                w.now_ms-mate.observed_at_ms>c.snapshot_max_age_ms||
                std::find(side.members.begin(),side.members.end(),mate.key)==side.members.end())continue;
            const float dx=a.position.x-known->position.x,dz=a.position.z-known->position.z;
            return std::isfinite(dx*dx+dz*dz)&&dx*dx+dz*dz<p.rush_run_m*p.rush_run_m;
        }
    }
    return false;
}
}}}
