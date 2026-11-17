#include "CombatOutdoorSelf.h"
#include "CombatCore.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace painter { namespace combat { namespace core {
namespace {
bool unit(float x){return std::isfinite(x)&&x>0&&x<1;}
bool valid_policy(const OutdoorSelfPolicy&p){
    if(!unit(p.maximum_ratio)||!unit(p.peace_ratio)||p.peace_ratio>p.maximum_ratio||
        !unit(p.search_low_ratio)||!unit(p.search_high_ratio)||p.search_low_ratio>=p.search_high_ratio||
        p.search_high_ratio>=p.maximum_ratio||!unit(p.upper_ratio)||!unit(p.middle_ratio)||!unit(p.low_ratio)||
        p.low_ratio>=p.middle_ratio||p.middle_ratio>=p.upper_ratio||p.upper_ratio>=p.maximum_ratio)return false;
    float previous=0;for(float b:p.distance_bands_m){if(!std::isfinite(b)||b<=previous)return false;previous=b;}
    return p.seen_veto_ms&&p.upper_far_unseen_ms&&p.middle_mid_unseen_ms&&p.middle_far_unseen_ms&&
        p.low_close_unseen_ms&&p.low_far_unseen_ms&&p.critical_unseen_ms&&p.gap_ms&&p.keep_ms&&p.cancel_ms>=p.keep_ms;
}
bool own_visible(const Contact&c,const ActorSnapshot&a){return valid_key(c.key)&&c.hostile&&c.alive_when_observed&&!c.wounded&&personally_visible(c,a);}
bool current_applied(const Order*o,const ActorSnapshot&a,const WorldSnapshot&w,const Config&c){
    if(!o)return false;auto checked=*o;checked.fire={};
    return publishable(checked,a,w.now_ms,w.level_epoch,w.geometry_revision)&&o->snapshot_seq<=w.sequence&&a.accessible_current_order&&
        o->order_seq==a.last_applied.order_seq&&a.last_applied.at_ms>=o->issued_at_ms&&a.last_applied.at_ms<=a.observed_at_ms&&
        fresh(a.last_applied.at_ms,w.now_ms,c.snapshot_max_age_ms)&&
        !(a.last_rejected.order_seq==o->order_seq&&a.last_rejected.reason!=reject_none);
}
bool current_friendly(const ActorSnapshot&a,ActorKey target,TimeMs at){
    const auto veto=[&](const Contact&c){return c.key==target&&c.observed_at_ms>=at&&c.observed_at_ms<=a.observed_at_ms&&(!c.hostile||!c.alive_when_observed);};
    if(veto(a.enemy))return true;for(std::size_t n=0;n<a.visible_enemies.count;++n)if(veto(a.visible_enemies.items[n]))return true;
    for(std::size_t n=0;n<a.seen_dead.count;++n)if(a.seen_dead.items[n]==target)return true;return false;
}
}
CombatOutdoorSelf::CombatOutdoorSelf(Config c,OutdoorSelfPolicy p):config_(std::move(c)),policy_(p){}
void CombatOutdoorSelf::reset(){state_={};}
std::unique_ptr<CombatOutdoorSelf> CombatOutdoorSelf::clone(const std::function<bool()>&cancelled)const{
    if(cancelled&&cancelled())return nullptr;auto out=std::make_unique<CombatOutdoorSelf>(*this);
    if(cancelled&&cancelled())return nullptr;return out;
}
OutdoorSelfResult CombatOutdoorSelf::update(const WorldSnapshot&w,const ActorSnapshot&a,const SideSnapshot&s,
    const OutdoorSelfInput&i,OutdoorSelfQuota&q,const std::function<bool()>&cancelled){
    OutdoorSelfResult out;out.proposal=i.base;const auto start=q.work_used;State next=state_;
    struct Cancelled{};struct Exhausted{};
    auto poll=[&](){if(cancelled&&cancelled())throw Cancelled{};};
    auto work=[&](){poll();if(!q.work_left)throw Exhausted{};--q.work_left;++q.work_used;};
    auto finish=[&](OutdoorSelfReason why){poll();next.sequence=w.sequence;next.at_ms=w.now_ms;state_=next;
        out.status=OutdoorSelfStatus::ready;out.reason=why;out.pending=next.pending;out.pending_started_ms=next.pending?next.pending_at_ms:0;
        out.actual_reload_seen_ms=next.has_reload?next.reload_seen_ms:0;
        out.cooldown_until_ms=std::max(next.has_reload?saturated_after(next.reload_seen_ms,policy_.gap_ms):0,
            next.has_end?saturated_after(next.end_ms,policy_.gap_ms):0);out.work=q.work_used-start;return out;};
    try{
        poll();if(!policy_.enabled){out.status=OutdoorSelfStatus::disabled;return out;}
        work();if(!valid_policy(policy_)||!w.level_epoch||!w.sequence||!w.geometry_revision||!s.side_id||
            a.level_epoch!=w.level_epoch||a.snapshot_seq!=w.sequence||a.observed_at_ms!=w.now_ms||a.faction!=s.faction||
            !valid_key(a.key)||!a.ownership_generation||!finite(a.position)||
            a.visible_enemies.count>a.visible_enemies.items.size()||a.seen_dead.count>a.seen_dead.items.size()||
            i.base.order.actor!=a.key||i.base.order.ownership_generation!=a.ownership_generation||
            i.base.order.restrictions_revision!=a.restrictions_revision||i.base.order.level_epoch!=w.level_epoch||
            i.base.order.geometry_revision!=w.geometry_revision||i.base.order.snapshot_seq>w.sequence||
            !i.knowledge||!i.knowledge->accepted||i.knowledge->cancelled||i.knowledge->level_epoch!=w.level_epoch||
            i.knowledge->sequence!=w.sequence||i.knowledge_index>=i.knowledge->members.size())return out;
        const auto&m=i.knowledge->members[i.knowledge_index];
        if(m.actor!=a.key||m.ownership_generation!=a.ownership_generation||m.side!=s.side_id||!m.alive||!m.able)return out;
        std::set<std::pair<std::uint32_t,std::uint64_t>> roster;bool present=false;
        for(auto id:s.members){work();if(!valid_key(id)||!roster.emplace(id.id,id.generation).second)return out;present|=id==a.key;}
        if(!present)return out;
        if(state_.epoch&&(w.level_epoch<state_.epoch||(w.level_epoch==state_.epoch&&(w.sequence<=state_.sequence||w.now_ms<state_.at_ms)))){
            out.status=OutdoorSelfStatus::stale;out.work=q.work_used-start;return out;}
        const bool same=state_.epoch==w.level_epoch&&state_.map==w.map_hash&&state_.geometry==w.geometry_revision&&state_.actor==a.key&&
            state_.owner==a.ownership_generation&&state_.restrictions==a.restrictions_revision&&state_.side==s.side_id&&state_.faction==s.faction&&
            state_.group==s.group&&state_.mission==s.mission.revision&&state_.weapon==a.weapon.id;
        if(!same){next={};out.reset=true;}
        next.epoch=w.level_epoch;next.map=w.map_hash;next.geometry=w.geometry_revision;next.actor=a.key;next.owner=a.ownership_generation;
        next.restrictions=a.restrictions_revision;next.side=s.side_id;next.faction=s.faction;next.group=s.group;next.mission=s.mission.revision;next.weapon=a.weapon.id;
        const bool applied=current_applied(i.applied,a,w,config_);
        out.actual_reloading=a.reloading||a.weapon.state==WeaponState::reloading;
        // Weapon state is captured fact. A reload intent alone is not evidence.
        if(out.actual_reloading){next.has_reload=true;next.reload_seen_ms=a.observed_at_ms;}
        const auto base=first_response(a,config_,w.now_ms,w.level_epoch,w.geometry_revision,i.base.expected_previous_order_seq);
        if(base.kind==OrderKind::yield||a.yield_reasons||a.danger.kind==DangerKind::grenade){next.pending=false;return finish(OutdoorSelfReason::native_yield);}
        const auto&b=i.base.order;
        if(i.genuine_commitment||(b.flags&order_committed_entry)||b.kind==OrderKind::withdraw||b.kind==OrderKind::bound||
            b.throw_order.valid||valid_key(b.aid)||valid_key(b.finish)||(applied&&continuing_throw(*i.applied,a,w.now_ms))){
            next.pending=false;return finish(OutdoorSelfReason::commitment);}
        bool visible=own_visible(a.enemy,a);for(std::size_t n=0;n<a.visible_enemies.count;++n){work();visible|=own_visible(a.visible_enemies.items[n],a);}
        const bool firing=a.weapon.state==WeaponState::firing||(applied&&a.last_applied.fired_since_previous);
        const bool urgent=m.hit_from.valid&&m.hit_from.unseen&&m.hit_from.observer==a.key&&m.hit_from.shooter.valid()&&
            (!m.hit_from.shooter.ghost()||m.hit_from.shooter.side==s.side_id)&&finite(m.hit_from.point)&&finite(m.hit_from.direction)&&
            fresh(m.hit_from.at_ms,w.now_ms,config_.knowledge.uf_escape_ms);
        auto reload_intent=[&](OutdoorSelfReason reason){
            out.proposal={};out.proposal.group_id=i.base.group_id;out.proposal.expected_previous_order_seq=i.base.expected_previous_order_seq;
            out.proposal.order=base;auto&o=out.proposal.order;o.kind=OrderKind::reload;o.flags&=~order_first_response;o.fire={};
            // Safe stationary fallback only; the next existing layer may select
            // a certified reload-cover leg without falsifying weapon state.
            out.reload_intent=true;return finish(reason);
        };
        if(next.pending){const auto elapsed=w.now_ms-next.pending_at_ms;
            if(!visible&&!firing&&!urgent&&(elapsed<policy_.keep_ms||(out.actual_reloading&&elapsed<policy_.cancel_ms)))
                return reload_intent(OutdoorSelfReason::pending_reload);
            next.pending=false;next.has_end=true;next.end_ms=w.now_ms;
            return finish(OutdoorSelfReason::pending_ended);
        }
        if(out.actual_reloading||!a.weapon.ammo_in_mag)return finish(OutdoorSelfReason::native_reload);
        if(a.weapon.id==invalid_id||!a.weapon.mag_size||a.weapon.ammo_in_mag>=a.weapon.mag_size||!a.weapon.ammo_reserve||
            a.weapon.kind==WeaponKind::none||a.weapon.kind==WeaponKind::melee||a.weapon.state==WeaponState::switching||a.weapon.state==WeaponState::misfire)
            return finish(OutdoorSelfReason::weapon_unavailable);
        out.ammo_ratio=float(a.weapon.ammo_in_mag)/float(a.weapon.mag_size);
        if(visible)return finish(OutdoorSelfReason::visible);if(firing)return finish(OutdoorSelfReason::firing);if(urgent)return finish(OutdoorSelfReason::urgent_escape);
        if((next.has_reload&&w.now_ms-next.reload_seen_ms<policy_.gap_ms)||(next.has_end&&w.now_ms-next.end_ms<policy_.gap_ms))return finish(OutdoorSelfReason::cooldown);
        const float ratio=out.ammo_ratio;if(ratio>=policy_.maximum_ratio)return finish(OutdoorSelfReason::ratio);
        struct Foe {const FoeKnowledge* foe;const SourceFact* lk;const SourceFact* seen;};std::vector<Foe> foes;
        const Foe*target=nullptr;
        for(const auto&f:m.foes){work();if(!f.id.valid()||f.down)continue;
            if(f.id.ghost()?f.id.side!=s.side_id:roster.count({f.id.actor.id,f.id.actor.generation})!=0)continue;
            const SourceFact*lk=nullptr;const SourceFact*seen=nullptr;
            for(std::size_t n=0;n<f.sources.size();++n){work();const auto&sf=f.sources[n];if(!sf.valid)continue;
                // A malformed future/foreign own source is not an empty peaceful scene.
                if(!valid_key(sf.reporter)||!finite(sf.position)||sf.at_ms>a.observed_at_ms||
                    (n<2&&sf.reporter!=a.key)){out.work=q.work_used-start;return out;}
                if(!fresh(sf.at_ms,w.now_ms,config_.knowledge.search_forget_ms))continue;
                if(!lk||sf.at_ms>lk->at_ms)lk=&sf;if(n==0)seen=&sf;
            }
            if(!lk||(!f.id.ghost()&&current_friendly(a,f.id.actor,lk->at_ms)))continue;
            if(f.wounded&&lk->at_ms<=f.wounded_ms)continue;
            foes.push_back({&f,lk,seen});
        }
        out.known=foes.size();for(const auto&f:foes){work();if(f.foe->id==m.target){target=&f;break;}}
        // No selected target uses the Lua peace threshold, but malformed source
        // data above never turns into fabricated lack of enemies.
        if(!target){if(ratio>=policy_.peace_ratio)return finish(OutdoorSelfReason::ratio);}
        else if(target->foe->search_open&&ratio<policy_.search_low_ratio){next.pending=true;next.pending_at_ms=w.now_ms;return reload_intent(OutdoorSelfReason::tactical_reload);}
        else if(target->foe->search_open&&ratio>policy_.search_high_ratio)return finish(OutdoorSelfReason::ratio);
        else for(const auto&f:foes){work();const auto unseen=f.seen?w.now_ms-f.seen->at_ms:(std::numeric_limits<TimeMs>::max)();
            const float d=std::hypot(f.lk->position.x-a.position.x,f.lk->position.z-a.position.z);
            if(!std::isfinite(d)){out.work=q.work_used-start;return out;}
            unsigned band=1;for(float bound:policy_.distance_bands_m){work();if(d<=bound)break;++band;}
            bool allowed=true;
            if(f.seen&&unseen<policy_.seen_veto_ms)allowed=false;
            else if(ratio>policy_.upper_ratio)allowed=band>=4&&unseen>policy_.upper_far_unseen_ms;
            else if(ratio>policy_.middle_ratio)allowed=band>2&&unseen>(band==3?policy_.middle_mid_unseen_ms:policy_.middle_far_unseen_ms);
            else if(!f.seen&&!f.foe->shot_at_me.set&&!f.foe->shot_me.set)allowed=true;
            else if(ratio>policy_.low_ratio)allowed=band>1&&unseen>(band==2?policy_.low_close_unseen_ms:policy_.low_far_unseen_ms);
            else allowed=unseen>policy_.critical_unseen_ms;
            if(!allowed){out.veto_foe=f.foe->id;out.veto_source_ms=f.lk->at_ms;return finish(OutdoorSelfReason::threat_band);}
        }
        next.pending=true;next.pending_at_ms=w.now_ms;return reload_intent(OutdoorSelfReason::tactical_reload);
    }catch(const Cancelled&){out.status=OutdoorSelfStatus::cancelled;}catch(const Exhausted&){out.status=OutdoorSelfStatus::exhausted;}
    catch(...){out.status=OutdoorSelfStatus::failed;}
    out.reload_intent=false;out.proposal=i.base;out.work=q.work_used-start;return out;
}
}}}
