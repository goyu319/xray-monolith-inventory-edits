#pragma once
#include "SquadTactics.h"
#include <memory>

namespace painter { namespace combat { namespace core {
// Required private data, not selected runtime defaults. Exact comparison
// boundaries follow 40_layers.lua reload_by_ammo/reload_ratios.
// OutdoorSelfPolicy is allocated in ../OutdoorConfig.h.
struct OutdoorSelfQuota {std::size_t work_left=0,work_used=0;};
struct OutdoorSelfInput {
    ProposedOrder base;
    const Order* applied=nullptr;
    const KnowledgeFrame* knowledge=nullptr;
    std::size_t knowledge_index=static_cast<std::size_t>(-1);
    // Actual caller commitments, not every room assignment or arbitrary role.
    bool genuine_commitment=false;
};
enum class OutdoorSelfStatus : std::uint8_t {invalid,disabled,ready,stale,cancelled,exhausted,failed};
enum class OutdoorSelfReason : std::uint8_t {
    unchanged,native_yield,commitment,native_reload,weapon_unavailable,visible,
    firing,urgent_escape,cooldown,ratio,threat_band,tactical_reload,pending_reload,pending_ended
};
struct OutdoorSelfResult {
    OutdoorSelfStatus status=OutdoorSelfStatus::invalid;
    OutdoorSelfReason reason=OutdoorSelfReason::unchanged;
    ProposedOrder proposal;
    bool reload_intent=false,pending=false,actual_reloading=false,reset=false;
    float ammo_ratio=0;
    std::size_t known=0,work=0;
    FoeId veto_foe;TimeMs veto_source_ms=0;
    TimeMs pending_started_ms=0,actual_reload_seen_ms=0,cooldown_until_ms=0;
};
// One actor, privately cloned with outer worker state. A reload proposal is
// explicitly pending, not a claim that the engine has begun reloading. This
// layer never changes ActorSnapshot and does no native/nav/cover query. The
// caller may feed reload_intent to the existing reload-cover decorator, while
// retaining actual engine weapon facts and higher-priority commitments.
class CombatOutdoorSelf {
public:
    CombatOutdoorSelf(Config,OutdoorSelfPolicy);
    std::unique_ptr<CombatOutdoorSelf> clone(const std::function<bool()>& cancelled={}) const;
    OutdoorSelfResult update(const WorldSnapshot&,const ActorSnapshot&,const SideSnapshot&,
        const OutdoorSelfInput&,OutdoorSelfQuota&,const std::function<bool()>& cancelled={});
    void reset();
private:
    struct State {
        ActorKey actor;std::uint64_t epoch=0,map=0,geometry=0,sequence=0,owner=0,restrictions=0,group=0,mission=0;
        std::uint32_t side=0,faction=0;std::uint16_t weapon=invalid_id;TimeMs at_ms=0;
        bool pending=false,has_reload=false,has_end=false;
        TimeMs pending_at_ms=0,reload_seen_ms=0,end_ms=0;
    } state_;
    Config config_;OutdoorSelfPolicy policy_;
};
}}}
