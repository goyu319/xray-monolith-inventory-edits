#pragma once
#include "PhysicalHitCapture.h"
#include "core/PerceivedSpecies.h"
#include <algorithm>
namespace painter { namespace combat { namespace engine {
// Admission only: a real personally received wound need not identify its shooter.
// This never creates a contact, position, visibility or species fact. The native
// physical recorder has already excluded zero damage/friendly/environment hits.
inline bool known_mutant_event(ActorKey source,TimeMs now,const Config& cfg,const core::ObserverSpecies& observed){
    if(source.id==invalid_id||!source.generation)return false;
    for(const auto& f:observed.facts)if(f.target==source&&f.known_at_ms&&f.known_at_ms<=now&&
        now-f.known_at_ms<=cfg.memory_max_age_ms&&f.species==core::PerceivedSpecies::mutant)return true;
    return false;
}
inline TimeMs personal_event_admission(const ActorSnapshot& events,TimeMs now,const Config& cfg,const core::ObserverSpecies& observed){
    if(observed.captured_at_ms!=now||!observed.ownership_generation||observed.observer.id==invalid_id||!observed.observer.generation)return 0;
    TimeMs at=0;
    for(std::size_t i=0;i<events.heard.count&&i<events.heard.items.size();++i){const auto& h=events.heard.items[i];
        if(h.at_ms&&h.at_ms<=now&&now-h.at_ms<=cfg.knowledge.input_age_ms&&!known_mutant_event(h.source,now,cfg,observed))at=(std::max)(at,h.at_ms);}
    for(std::size_t i=0;i<events.near_misses.count&&i<events.near_misses.items.size();++i){const auto& n=events.near_misses.items[i];
        if(n.at_ms&&n.at_ms<=now&&now-n.at_ms<=cfg.knowledge.input_age_ms&&!known_mutant_event(n.from,now,cfg,observed))at=(std::max)(at,n.at_ms);}
    return at;
}
template<class Resolve> TimeMs own_hit_admission(const PhysicalHitCapture& physical,TimeMs now,
    const Config& cfg,const core::ObserverSpecies& observed,Resolve resolve){
    if(observed.captured_at_ms!=now||!observed.ownership_generation||observed.observer.id==invalid_id||!observed.observer.generation)return 0;
    const auto age_limit=(std::min)(cfg.memory_max_age_ms,cfg.knowledge.input_age_ms);
    const auto span=static_cast<std::uint32_t>((std::min)(age_limit,TimeMs(0x7ffffffe)))+1u;
    TimeMs latest=0;
    physical.since(std::uint32_t(now)-span,std::uint32_t(now),[&](const PhysicalHitSample& hit){
        if(!hit.hostile_attack)return;
        const auto age=std::uint32_t(std::uint32_t(now)-hit.at);if(now<age)return;
        const auto at=now-age;if(!at||age>age_limit)return;
        const ActorKey source=hit.source?resolve(hit.source,hit.source_id):ActorKey{};
        if(!known_mutant_event(source,now,cfg,observed))latest=(std::max)(latest,at);
    });
    return latest;
}
}}}
