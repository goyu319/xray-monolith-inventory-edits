#pragma once
#include "SquadTactics.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace painter { namespace combat { namespace core {
// v2.23 represents finish as Order.finish + roles::finisher, not a new enum.
// Only current own sight and the native finish_allowed certification may earn
// this assignment. This selector never reads the target's global ActorSnapshot.
inline bool wounded_finish_contact(const Contact& c,const ActorSnapshot& a,const WorldSnapshot& w,const Config& cfg){
    return valid_key(c.key)&&c.key!=a.key&&c.hostile&&c.alive_when_observed&&c.wounded&&c.finish_allowed&&
        finite(c.position)&&personally_visible(c,a)&&a.level_epoch==w.level_epoch&&a.snapshot_seq==w.sequence&&
        fresh(a.observed_at_ms,w.now_ms,cfg.snapshot_max_age_ms);
}
inline bool wounded_finish_free(const ActorSnapshot& a,const Order& o,const WorldSnapshot& w,const Config& cfg){
    if(!valid_key(a.key)||!a.ownership_generation||!(a.flags&flag_admitted)||a.yield_reasons||!a.alive||a.wounded||
        !std::isfinite(a.health)||a.health<=0||!finite(a.forward)||!a.current_position_accessible||a.node_id==invalid_node||!finite(a.position)||a.reloading||
        a.weapon.state==WeaponState::reloading||!a.weapon.ammo_in_mag||a.weapon.id==invalid_id||(a.flags&flag_dont_shoot)||
        a.visible_enemies.count>a.visible_enemies.items.size()||a.hits.count>a.hits.items.size()||
        o.actor!=a.key||o.ownership_generation!=a.ownership_generation||o.throw_order.valid||valid_key(o.aid)||
        (o.flags&(order_committed_entry|order_urgent|order_cover_fire))||
        o.kind==OrderKind::entry||o.kind==OrderKind::stack||o.kind==OrderKind::peek||o.kind==OrderKind::hold_door||
        o.kind==OrderKind::withdraw||o.kind==OrderKind::recover||o.kind==OrderKind::reload||o.kind==OrderKind::bound||
        o.role==roles::base_of_fire||o.role==roles::under_fire_cover||o.role==roles::barricade_guard||a.throw_state.phase==1||a.throw_state.phase==2||
        a.aid_state.phase==1||a.aid_state.phase==2)return false;
    if(first_response(a,cfg,w.now_ms,w.level_epoch,w.geometry_revision,1).kind==OrderKind::yield)return false;
    const auto able=[&](const Contact& c){return valid_key(c.key)&&c.hostile&&c.alive_when_observed&&!c.wounded&&personally_visible(c,a);};
    if(able(a.enemy))return false;
    for(std::size_t n=0;n<a.visible_enemies.count;++n)if(able(a.visible_enemies.items[n]))return false;
    for(std::size_t n=0;n<a.hits.count;++n)if(fresh(a.hits.items[n].at_ms,w.now_ms,cfg.room.finisher_quiet_ms))return false;
    return true;
}
inline bool apply_wounded_finishers(const WorldSnapshot& w,const Config& cfg,const SquadResult& plans,
    const std::vector<Order>& current,std::vector<ProposedOrder>& overrides,const std::function<bool()>& cancelled){
    using Key=std::pair<std::uint32_t,std::uint64_t>;
    const auto key=[](ActorKey a){return Key{a.id,a.generation};};
    std::map<Key,const ActorSnapshot*> actors;std::map<Key,const Order*> old;
    std::map<Key,ProposedOrder> selected;std::set<Key> changed;
    const auto stop=[&](){return cancelled&&cancelled();};
    for(const auto& a:w.actors){if(stop())return false;actors.emplace(key(a.key),&a);}
    for(const auto& o:current)old.emplace(key(o.actor),&o);
    for(const auto& p:plans.proposals)selected[key(p.order.actor)]=p;
    for(const auto& p:overrides)selected[key(p.order.actor)]=p;
    std::set<Key> used_targets;
    for(const auto& side:w.sides){
        if(stop())return false;
        const ActorSnapshot* winner=nullptr;const Contact* target=nullptr;float best=0;bool persistent=false;
        for(auto member:side.members){
            if(stop())return false;const auto id=key(member);const auto ai=actors.find(id);const auto oi=old.find(id);
            if(ai==actors.end()||oi==old.end())continue;const auto& a=*ai->second;const auto& previous=*oi->second;
            if(a.faction!=side.faction||previous.level_epoch!=w.level_epoch||previous.ownership_generation!=a.ownership_generation)continue;
            const auto candidate=selected.find(id);const Order& base=candidate==selected.end()?previous:candidate->second.order;
            if(!wounded_finish_free(a,base,w,cfg))continue;
            const auto consider=[&](const Contact& c){
                if(!wounded_finish_contact(c,a,w,cfg)||used_targets.count(key(c.key)))return;
                const float cost=distance_squared(a.position,c.position);if(!std::isfinite(cost))return;
                const bool keep=previous.finish==c.key&&previous.role==roles::finisher&&previous.expires_at_ms>w.now_ms;
                if(!winner||(keep&&!persistent)||(keep==persistent&&(cost<best||(cost==best&&a.key.id<winner->key.id)))){
                    winner=&a;target=&c;best=cost;persistent=keep;}};
            consider(a.enemy);for(std::size_t n=0;n<a.visible_enemies.count;++n)consider(a.visible_enemies.items[n]);
        }
        if(winner){
            const auto id=key(winner->key);ProposedOrder p;
            p.order=first_response(*winner,cfg,w.now_ms,w.level_epoch,w.geometry_revision,0);
            p.order.kind=OrderKind::hold;p.order.role=roles::finisher;p.order.finish=target->key;
            p.order.goal_node=winner->node_id;p.order.goal_position=winner->position;p.order.pace=Pace::stand;
            p.order.look={LookKind::enemy,target->position,target->key};p.order.flags&=~order_first_response;
            p.order.fire.allowed=cfg.fire_window_ms!=0;p.order.fire.until_ms=fire_window_until(cfg,w.now_ms,p.order.expires_at_ms);
            p.expected_previous_order_seq=old.at(id)->order_seq;
            const auto base=selected.find(id);p.group_id=base!=selected.end()?base->second.group_id:side.side_id;
            p.has_knowledge=true;p.knowledge.receiver=winner->key;p.knowledge.observer=winner->key;p.knowledge.target=target->key;
            p.knowledge.foe.actor=target->key;p.knowledge.position=target->position;p.knowledge.source_snapshot_seq=w.sequence;
            p.knowledge.observed_at_ms=winner->observed_at_ms;p.knowledge.personally_visible=true;
            p.knowledge.receiver_ownership_generation=winner->ownership_generation;p.knowledge.receiver_faction=winner->faction;
            selected[id]=std::move(p);changed.insert(id);used_targets.insert(key(target->key));
        }
        // Everyone else retains their selected job, but a wounded foe is not
        // ordinary personal/suppressive fire for the entire squad.
        for(auto member:side.members){if(stop())return false;const auto id=key(member);if(winner&&winner->key==member)continue;
            const auto a=actors.find(id);auto chosen=selected.find(id);if(a==actors.end()||chosen==selected.end())continue;
            auto& o=chosen->second.order;bool wounded_fire=false;
            const auto points_at=[&](const Contact& c){return c.wounded&&c.key==o.look.target&&valid_key(c.key);};
            wounded_fire=points_at(a->second->enemy);
            for(std::size_t n=0;n<a->second->visible_enemies.count&&n<a->second->visible_enemies.items.size();++n)
                wounded_fire=wounded_fire||points_at(a->second->visible_enemies.items[n]);
            if(valid_key(o.finish)){o.finish={};if(o.role==roles::finisher)o.role=roles::individual;changed.insert(id);wounded_fire=true;}
            if(wounded_fire&&o.fire.allowed){o.fire={};chosen->second.point_fire={};changed.insert(id);}
        }
    }
    for(const auto& id:changed){if(stop())return false;const auto prior=old.find(id);if(prior==old.end())continue;
        auto row=selected.at(id);row.expected_previous_order_seq=prior->second->order_seq;
        const auto found=std::find_if(overrides.begin(),overrides.end(),[&](const ProposedOrder& p){return key(p.order.actor)==id;});
        if(found==overrides.end())overrides.push_back(std::move(row));else *found=std::move(row);
    }
    return !stop();
}
}}}
