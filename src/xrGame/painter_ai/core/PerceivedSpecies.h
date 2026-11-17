#pragma once
#include "CombatKnowledge.h"

namespace painter { namespace combat { namespace core {
// Private capture sidecar, not an addition to canonical Contact. Classification
// is immutable runtime type observed through that man's own perception. It
// never grants sight, firing authority, target position or control admission.
enum class PerceivedSpecies : std::uint8_t { unknown,human_stalker,player,mutant,other };
struct PerceivedSpeciesFact {
    ActorKey target;
    PerceivedSpecies species=PerceivedSpecies::unknown;
    TimeMs known_at_ms=0,threat_at_ms=0;
    bool visible=false,hostile=false,alive_when_observed=false,wounded=false;
};
struct ObserverSpecies {
    ActorKey observer;
    std::uint64_t ownership_generation=0,restrictions_revision=0;
    TimeMs captured_at_ms=0;
    // All relevant records in THIS observer's memory were visited. A capped
    // visible_enemies array is never sufficient evidence for this bit.
    bool complete=false;
    std::vector<PerceivedSpeciesFact> facts;
};
struct PerceivedSpeciesEvidence {
    std::uint64_t level_epoch=0,snapshot_seq=0,map_hash=0,geometry_revision=0;
    TimeMs captured_at_ms=0;
    TimeMs memory_max_age_ms=0; // exact native Config value; zero means unsupported
    std::vector<ObserverSpecies> observers;
};
struct PerceivedFight {
    bool valid=false,monster_only=false,cancelled=false;
    std::size_t humans=0,mutants=0,unknown=0;
};
// Same rule for native automatic admission and lifecycle. A remembered species
// is only used beside fresh perceived threat evidence; timestamps stay original.
bool perceived_human(const ObserverSpecies&,TimeMs now,const Config&);
// Lack of a species witness is not lack of a fight. Keep an actual fresh
// personally evidenced unknown fight eligible; exclude only confirmed complete
// mutant-only input. This does not grant visibility or classify unknown actors.
bool perceived_combat_admission(const ObserverSpecies&,TimeMs now,const Config&);
PerceivedFight perceived_fight(const WorldSnapshot&,const SideSnapshot&,const KnowledgeFrame*,
    const PerceivedSpeciesEvidence&,const Config&,const std::function<bool()>& cancelled={});
}}}
