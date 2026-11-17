#pragma once
#include "CombatContract.h"
#include <map>
#include <mutex>
namespace painter { namespace combat { namespace engine {
// Pointer-free bridge: the owning action records applied native goals. The
// projectile producer may run on another thread and reports only an actual
// newly-created bullet, never an animation state or an ammo-count difference.
class AppliedActionReceipts {
    struct Entry {
        ActorKey actor;std::uint64_t owner=0,epoch=0,order=0;
        TimeMs at=0;std::uint16_t item=invalid_id;std::uint8_t weapon=0;
        bool shot=false,credit_fire=true;
    };
    std::mutex mutex_;std::map<std::uint32_t,Entry> entries_;
public:
    void applied(ActorKey actor,std::uint64_t owner,std::uint64_t epoch,
        std::uint64_t order,TimeMs at,std::uint16_t item,std::uint8_t weapon,bool credit_fire=true) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(actor.id>=invalid_id||!actor.generation||!owner||!epoch||!order||!at||weapon>3){entries_.erase(actor.id);return;}
        auto& e=entries_[actor.id];
        const bool same=e.actor==actor&&e.owner==owner&&e.epoch==epoch&&e.order==order&&e.item==item;
        const bool shot=same&&e.shot;
        e={actor,owner,epoch,order,at,item,weapon,shot,credit_fire};
    }
    void projectile(std::uint16_t actor,std::uint16_t weapon) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto i=entries_.find(actor);if(i==entries_.end())return;
        // Only the currently applied fire channel can earn a fire receipt.
        // Immediate self-defense against another visible enemy is real fire, but
        // cannot prove the authored covering-fire job reached its target.
        if(i->second.credit_fire&&i->second.weapon==2&&i->second.item==weapon&&weapon!=invalid_id)i->second.shot=true;
    }
    decltype(ActorSnapshot{}.last_applied) capture(ActorKey actor,std::uint64_t owner,
        std::uint64_t epoch,TimeMs now) {
        std::lock_guard<std::mutex> lock(mutex_);
        decltype(ActorSnapshot{}.last_applied) result{};
        auto i=entries_.find(actor.id);if(i==entries_.end())return result;
        auto& e=i->second;
        if(e.actor!=actor||e.owner!=owner||e.epoch!=epoch||e.at>now)return result;
        result.order_seq=e.order;result.at_ms=e.at;result.weapon=e.weapon;
        result.fired_since_previous=e.shot;e.shot=false;return result;
    }
    void clear(ActorKey actor) {
        std::lock_guard<std::mutex> lock(mutex_);auto i=entries_.find(actor.id);
        if(i!=entries_.end()&&i->second.actor==actor)entries_.erase(i);
    }
};
inline AppliedActionReceipts& applied_action_receipts(){static AppliedActionReceipts instance;return instance;}
inline void note_native_projectile(std::uint16_t actor,std::uint16_t weapon){applied_action_receipts().projectile(actor,weapon);}
}}}
