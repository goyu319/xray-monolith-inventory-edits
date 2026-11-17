#pragma once
#include "core/CombatSideRegistry.h"
#include <algorithm>
#include <limits>

namespace painter { namespace combat { namespace engine {
inline TimeMs friendly_roster_retention_ms(const Config& config){
    const auto max=(std::numeric_limits<TimeMs>::max)();
    return config.order_lifetime_ms>max/4?max:config.order_lifetime_ms*4;
}
// Only fresh membership observations may extend the capture-side witness list.
// This list is not itself an actor/life capture or a casualty observation.
inline std::vector<SideSnapshot> retained_friendly_rosters(const WorldSnapshot& world,
    const core::SideRegistryResult* previous,TimeMs retain_ms){
    auto known=world.sides;
    if(!previous||!previous->accepted||previous->level_epoch!=world.level_epoch||previous->map_hash!=world.map_hash||
        previous->snapshot_seq>=world.sequence||previous->captured_at_ms>world.now_ms)return known;
    auto fresh=[&](TimeMs at){return at<=world.now_ms&&world.now_ms-at<=retain_ms;};
    auto contains=[](const SideSnapshot& side,ActorKey key){return std::find(side.members.begin(),side.members.end(),key)!=side.members.end();};
    for(auto& side:known){
        // An actually captured continuing member is the identity bridge. An
        // unrelated replacement cohort sharing a numeric group gets no history.
        const bool continued=std::any_of(previous->friendly_memberships.begin(),previous->friendly_memberships.end(),
            [&](const core::FriendlySideMembership& receipt){
                if(receipt.side_id!=side.side_id||receipt.faction!=side.faction||receipt.group!=side.group||
                    receipt.mission_revision!=side.mission.revision||!fresh(receipt.known_at_ms)||!contains(side,receipt.member))return false;
                return std::any_of(world.actors.begin(),world.actors.end(),[&](const ActorSnapshot& actor){return actor.key==receipt.member&&
                    actor.faction==side.faction&&actor.native_squad==side.group;});
            });
        if(!continued)continue;
        for(const auto& receipt:previous->friendly_memberships){
            if(receipt.side_id!=side.side_id||receipt.faction!=side.faction||receipt.group!=side.group||
                receipt.mission_revision!=side.mission.revision||!fresh(receipt.known_at_ms)||contains(side,receipt.member))continue;
            const bool recaptured_elsewhere=std::any_of(world.actors.begin(),world.actors.end(),[&](const ActorSnapshot& actor){
                return actor.key.id==receipt.member.id&&(actor.key!=receipt.member||actor.faction!=side.faction||actor.native_squad!=side.group);});
            const bool assigned_elsewhere=std::any_of(world.sides.begin(),world.sides.end(),[&](const SideSnapshot& other){
                return other.side_id!=side.side_id&&std::any_of(other.members.begin(),other.members.end(),[&](ActorKey key){return key.id==receipt.member.id;});});
            if(!recaptured_elsewhere&&!assigned_elsewhere)side.members.push_back(receipt.member);
        }
    }
    return known;
}
}}}
