#include "pch_script.h"
#include "EnginePerceivedSpecies.h"
#include "ai/stalker/ai_stalker.h"
#include "Actor.h"
#include "ai/monsters/basemonster/base_monster.h"
#include "memory_manager.h"
#include "memory_space.h"
#include "visual_memory_manager.h"
#include "sound_memory_manager.h"
#include "hit_memory_manager.h"
#include "enemy_manager.h"
#include <map>

namespace painter { namespace combat { namespace engine {
namespace {
TimeMs observed_time(u32 t,TimeMs now){if(t==u32(-1))return 0;const auto age=u32(now)-t;return now>=age?now-age:0;}
core::PerceivedSpecies type(const CEntityAlive& other){
    if(smart_cast<const CAI_Stalker*>(&other))return core::PerceivedSpecies::human_stalker;
    if(smart_cast<const CActor*>(&other))return core::PerceivedSpecies::player;
    if(smart_cast<const CBaseMonster*>(&other))return core::PerceivedSpecies::mutant;
    return core::PerceivedSpecies::other;
}
}
void capture_species(CAI_Stalker& actor,const CaptureContext& c,const Config& cfg,core::ObserverSpecies& out,const core::ObserverSpecies* previous){
    out={};out.observer=c.key;out.ownership_generation=c.ownership_generation;out.restrictions_revision=c.restrictions_revision;
    out.captured_at_ms=c.now_ms;
    if(!c.key_of||c.key.id==invalid_id||!c.key.generation||!c.level_epoch||!c.snapshot_seq||!c.ownership_generation)return;
    std::map<std::uint16_t,core::PerceivedSpeciesFact> facts,personal;bool consistent=true;
    if(previous&&previous->observer==c.key&&previous->captured_at_ms<=c.now_ms)
        for(const auto& f:previous->facts)if(f.known_at_ms&&f.known_at_ms<=c.now_ms&&c.now_ms-f.known_at_ms<=cfg.memory_max_age_ms)
            personal.emplace(f.target.id,f);
    auto take=[&](const CEntityAlive* other){
        if(!other||other==&actor||other->getDestroy()||!actor.is_relation_enemy(other))return;
        auto& visual=actor.memory().visual();const bool visible=visual.visible_now(other);
        const auto memory=actor.memory().memory(other);
        if(!visible&&!memory.m_object)return;
        const auto threat_at=visible?c.now_ms:observed_time(memory.m_level_time,c.now_ms);
        if(!threat_at||threat_at>c.now_ms||c.now_ms-threat_at>cfg.memory_max_age_ms)return;
        core::PerceivedSpeciesFact f;f.target=c.key_of(*other);f.hostile=true;f.visible=visible;f.threat_at_ms=threat_at;
        f.known_at_ms=visible?c.now_ms:0;
        if(f.target.id==invalid_id||!f.target.generation){consistent=false;return;}
        // The engine's historical visual timestamp is squad-shared. Only our
        // exact earlier visible_now receipt establishes personal classification.
        // Unseen squad updates cannot refresh that receipt's original clock.
        if(visible)f.species=type(*other);
        else {auto old=personal.find(f.target.id);if(old!=personal.end()&&old->second.target==f.target){
            f.known_at_ms=old->second.known_at_ms;f.species=old->second.species;f.threat_at_ms=old->second.threat_at_ms;}}
        f.alive_when_observed=true; // unseen life is unknown, never queried
        if(!visible){auto old=personal.find(f.target.id);if(old!=personal.end()&&old->second.target==f.target){
            f.alive_when_observed=old->second.alive_when_observed;f.wounded=old->second.wounded;}}
        if(visible){f.alive_when_observed=!!other->g_Alive();if(auto* human=smart_cast<const CAI_Stalker*>(other))f.wounded=human->wounded();}
        auto found=facts.find(f.target.id);
        if(found==facts.end())facts.emplace(f.target.id,f);
        else if(found->second.target!=f.target)consistent=false;
        else if(f.threat_at_ms>found->second.threat_at_ms||(f.visible&&!found->second.visible))found->second=f;
    };
    // Selected memory is combat evidence, not personal species/visibility proof.
    take(actor.memory().enemy().selected());
    for(const auto& record:actor.memory().visual().objects())take(smart_cast<const CEntityAlive*>(record.m_object));
    // Unseen hits/sounds are untyped threats in K1; they do not disclose a
    // shooter's species through the engine's private attacker pointer.
    for(const auto& row:facts)out.facts.push_back(row.second);
    out.complete=consistent;
}
}}}
