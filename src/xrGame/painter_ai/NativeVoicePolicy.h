#pragma once
#include <cstdint>

namespace painter_voice {
enum class Cue : std::uint8_t { contact=0, grenade_release=1, count=2 };
struct ActorGate {
    std::uint64_t generation=0, ownership=0;
    std::uint64_t last[2]{};
    bool used[2]{}, contact_visible=false;
};
struct SquadGate { std::uint64_t last=0;bool used=false; };
inline void identity(ActorGate& actor,std::uint64_t generation,std::uint64_t ownership){
    if(actor.generation==generation&&actor.ownership==ownership)return;
    actor=ActorGate{};actor.generation=generation;actor.ownership=ownership;
}
inline bool contact_rise(ActorGate& actor,std::uint64_t generation,std::uint64_t ownership,bool visible){
    identity(actor,generation,ownership);
    const bool rise=visible&&!actor.contact_visible;
    actor.contact_visible=visible;
    return rise;
}
inline bool request(ActorGate& actor,SquadGate* squad,std::uint64_t generation,std::uint64_t ownership,
                    Cue cue,std::uint64_t now){
    identity(actor,generation,ownership);
    const unsigned index=static_cast<unsigned>(cue);
    if(index>=2)return false;
    const std::uint64_t actor_cooldown=cue==Cue::grenade_release?4000:12000;
    const std::uint64_t squad_cooldown=cue==Cue::grenade_release?1000:3000;
    if(actor.used[index]&&now>=actor.last[index]&&now-actor.last[index]<actor_cooldown)return false;
    if(squad&&squad->used&&now>=squad->last&&now-squad->last<squad_cooldown)return false;
    actor.last[index]=now;actor.used[index]=true;
    if(squad){squad->last=now;squad->used=true;}
    return true;
}
}
