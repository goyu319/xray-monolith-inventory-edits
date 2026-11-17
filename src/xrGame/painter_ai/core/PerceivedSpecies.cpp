#include "PerceivedSpecies.h"
#include "CombatCore.h"
#include <map>
#include <set>
#include <tuple>

namespace painter { namespace combat { namespace core {
namespace {
struct Stop{};
void poll(const std::function<bool()>& f){if(f&&f())throw Stop{};}
struct Less {bool operator()(ActorKey a,ActorKey b)const{return std::tie(a.id,a.generation)<std::tie(b.id,b.generation);}};
bool current(const PerceivedSpeciesFact& f,TimeMs now,const Config& c){return valid_key(f.target)&&f.hostile&&f.alive_when_observed&&!f.wounded&&
    f.threat_at_ms&&fresh(f.threat_at_ms,now,c.memory_max_age_ms);}
PerceivedSpecies kind(const PerceivedSpeciesFact& f,TimeMs now){
    return f.known_at_ms&&f.known_at_ms<=now?f.species:PerceivedSpecies::unknown;
}
}
bool perceived_human(const ObserverSpecies& o,TimeMs now,const Config& c){
    if(!o.complete||o.captured_at_ms!=now)return false;
    for(const auto& f:o.facts)if(current(f,now,c)){
        const auto k=kind(f,now);if(k==PerceivedSpecies::human_stalker||k==PerceivedSpecies::player)return true;
    }return false;
}
bool perceived_combat_admission(const ObserverSpecies& o,TimeMs now,const Config& c){
    if(!valid_key(o.observer)||!o.ownership_generation||o.captured_at_ms!=now)return false;
    bool fighting=false;
    for(const auto& f:o.facts)if(current(f,now,c)){
        fighting=true;
        if(kind(f,now)!=PerceivedSpecies::mutant)return true;
    }
    // Incomplete input cannot prove only mutants, but still needs an actual
    // valid hostile event. Empty/incomplete capture alone never admits anyone.
    return fighting&&!o.complete;
}
PerceivedFight perceived_fight(const WorldSnapshot& w,const SideSnapshot& s,const KnowledgeFrame* k,
    const PerceivedSpeciesEvidence& e,const Config& cfg,const std::function<bool()>& stop){
    PerceivedFight out;
    try {
        poll(stop);
        if(!w.level_epoch||!w.sequence||e.level_epoch!=w.level_epoch||e.snapshot_seq!=w.sequence||e.map_hash!=w.map_hash||
            e.geometry_revision!=w.geometry_revision||e.captured_at_ms!=w.now_ms)return out;
        if(k&&(!k->accepted||k->cancelled||k->rejection!=KnowledgeReject::none||k->level_epoch!=w.level_epoch||k->sequence!=w.sequence))return out;
        std::map<ActorKey,const ActorSnapshot*,Less> actors;
        for(const auto& a:w.actors){poll(stop);if(!actors.emplace(a.key,&a).second)return out;}
        std::map<ActorKey,const ObserverSpecies*,Less> observers;
        std::map<ActorKey,PerceivedSpecies,Less> typed;
        std::map<std::pair<ActorKey,ActorKey>,const PerceivedSpeciesFact*,bool(*)(const std::pair<ActorKey,ActorKey>&,const std::pair<ActorKey,ActorKey>&)> facts(
            [](const auto& a,const auto& b){Less l;if(l(a.first,b.first))return true;if(l(b.first,a.first))return false;return l(a.second,b.second);});
        for(const auto& row:e.observers){poll(stop);auto ai=actors.find(row.observer);
            if(ai==actors.end()||!observers.emplace(row.observer,&row).second)return out;
            const auto& a=*ai->second;
            if(!valid_key(a.key)||row.ownership_generation!=a.ownership_generation||row.restrictions_revision!=a.restrictions_revision||
                row.captured_at_ms!=a.observed_at_ms||row.captured_at_ms!=w.now_ms||a.level_epoch!=w.level_epoch||a.snapshot_seq!=w.sequence)return out;
            for(const auto& f:row.facts){poll(stop);
                if(!valid_key(f.target)||f.known_at_ms>w.now_ms||f.threat_at_ms>w.now_ms||
                    static_cast<unsigned>(f.species)>static_cast<unsigned>(PerceivedSpecies::other)||
                    (f.visible&&(!f.known_at_ms||f.known_at_ms!=w.now_ms||f.threat_at_ms!=w.now_ms))||
                    !facts.emplace(std::make_pair(row.observer,f.target),&f).second)return out;
                const auto species=kind(f,w.now_ms);
                if(species!=PerceivedSpecies::unknown){auto old=typed.emplace(f.target,species);
                    if(!old.second&&old.first->second!=species)return out;}
            }
        }
        std::set<ActorKey,Less> roster;for(auto key:s.members){poll(stop);if(!valid_key(key)||!roster.insert(key).second)return out;}
        std::set<ActorKey,Less> counted;
        auto add=[&](ActorKey target,ActorKey reporter){
            poll(stop);if(!valid_key(target)||roster.count(target))return;
            auto f=facts.find({reporter,target});PerceivedSpecies species=PerceivedSpecies::unknown;
            if(f!=facts.end()&&current(*f->second,w.now_ms,cfg))species=kind(*f->second,w.now_ms);
            // A target seen by different members is not a census count. Count
            // unknown conservatively even when another observer typed it.
            if(species==PerceivedSpecies::unknown||species==PerceivedSpecies::other){++out.unknown;return;}
            if(!counted.insert(target).second)return;
            if(species==PerceivedSpecies::mutant)++out.mutants;else ++out.humans;
        };
        for(auto member:s.members){poll(stop);auto ai=actors.find(member);if(ai==actors.end()){++out.unknown;continue;}
            const auto& a=*ai->second;if(!a.alive)continue;
            if(a.faction!=s.faction||a.observed_at_ms!=w.now_ms||a.visible_enemies.count>a.visible_enemies.items.size()||
                a.hits.count>a.hits.items.size()||a.heard.count>a.heard.items.size()||a.near_misses.count>a.near_misses.items.size()||
                a.seen_dead.count>a.seen_dead.items.size())return {};
            auto oi=observers.find(member);if(oi==observers.end()||!oi->second->complete){++out.unknown;continue;}
            auto dead=[&](ActorKey target){for(std::size_t i=0;i<a.seen_dead.count;++i)if(a.seen_dead.items[i]==target)return true;return false;};
            for(const auto& f:oi->second->facts){poll(stop);if(current(f,w.now_ms,cfg)&&!dead(f.target))add(f.target,member);}
            auto contact=[&](const Contact& c){if(valid_key(c.key)&&c.hostile&&c.alive_when_observed&&!c.wounded&&c.observed_at_ms<=a.observed_at_ms&&
                !dead(c.key)&&(c.visible||fresh(c.observed_at_ms,w.now_ms,cfg.memory_max_age_ms)))add(c.key,member);};
            contact(a.enemy);for(std::size_t i=0;i<a.visible_enemies.count;++i)contact(a.visible_enemies.items[i]);
            auto bearing=[&](ActorKey target,TimeMs at){if(at&&fresh(at,w.now_ms,cfg.memory_max_age_ms)){
                if(valid_key(target)){if(!dead(target))add(target,member);}else ++out.unknown;}};
            for(std::size_t i=0;i<a.hits.count;++i)bearing(a.hits.items[i].from,a.hits.items[i].at_ms);
            for(std::size_t i=0;i<a.near_misses.count;++i)bearing(a.near_misses.items[i].from,a.near_misses.items[i].at_ms);
            for(std::size_t i=0;i<a.heard.count;++i)if(a.heard.items[i].kind==SoundKind::gunshot||a.heard.items[i].kind==SoundKind::weapon)
                bearing(a.heard.items[i].source,a.heard.items[i].at_ms);
            if(k)for(const auto& km:k->members){poll(stop);if(km.actor!=member)continue;
                if(km.side!=s.side_id||km.ownership_generation!=a.ownership_generation){++out.unknown;continue;}
                for(const auto& foe:km.foes){poll(stop);if(foe.down||foe.wounded||(!foe.id.ghost()&&dead(foe.id.actor)))continue;
                    for(const auto& source:foe.sources){poll(stop);if(!source.valid||!source.at_ms||!fresh(source.at_ms,w.now_ms,cfg.memory_max_age_ms))continue;
                        if(foe.id.ghost())++out.unknown;else add(foe.id.actor,source.reporter);
                    }
                }
            }
        }
        poll(stop);out.valid=true;out.monster_only=out.mutants>0&&!out.humans&&!out.unknown;return out;
    }catch(const Stop&){PerceivedFight cancelled;cancelled.cancelled=true;return cancelled;}
}
}}}
