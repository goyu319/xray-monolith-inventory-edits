#pragma once
#include "CombatOutdoorPersonal.h"
#include <array>
#include <memory>

namespace painter { namespace combat { namespace core {
// Required PRIVATE policy. Zero construction is invalid, not live defaults.
// WeaponKind indexes effective_m; unknown/none/melee ranges may be zero.
// OutdoorSoloProfile is allocated in ../OutdoorConfig.h.
// Explicit opt-in policy: CQB costs extend the frozen Lua hard-cover rule.
// Hold scales index none/light/medium/heavy/extreme; no hidden runtime values.
// OutdoorSoloCombatPolicy is allocated in ../OutdoorConfig.h.
// OutdoorSoloPolicy is allocated in ../OutdoorConfig.h.
struct OutdoorSoloQuota {std::size_t work_left=0,queries_left=0,work_used=0,queries_used=0;};
// Optional route witness from a real producer. A native path LENGTH alone is
// not proof that its intermediate nodes satisfy actor or threat restrictions.
struct OutdoorSoloRoute {
    OutdoorQuerySource source;
    std::uint32_t goal_node=invalid_node;Vec3 goal;
    Evidence restricted_route=Evidence::unknown,threat_opening_safe=Evidence::unknown;
};
struct OutdoorSoloThrow {
    OutdoorQuerySource source;
    std::uint16_t item=invalid_id;std::uint32_t serial=0;
    // Must include CURRENT friendly bodies as well as their reserved goals;
    // the ordinary reservation slice alone contains goals for moving friends.
    Evidence protected_start=Evidence::unknown,trajectory_supported=Evidence::unknown,teammates_safe=Evidence::unknown;
};
struct OutdoorSoloGrenadeEscape {
    // focus_at/position identify the actually perceived Danger grenade.
    OutdoorQuerySource source;
    OutdoorSoloRoute route;
};
// These receipts require an actual producer. Neither requested Order.stance
// nor an ordinary clear static eye ray certifies attained pose or gun safety.
struct OutdoorSoloAimEvidence {
    OutdoorQuerySource source;std::uint64_t applied_order_seq=0;Vec3 measured_eye;
    Evidence weapon_line_clear=Evidence::unknown,teammate_line_clear=Evidence::unknown;
};
struct OutdoorSoloLeanCapture {
    OutdoorQuerySource source;Vec3 measured_eye,edge;
    Stance attained_stance=Stance::stand;
    bool measured=false,edge_measured=false;
    // Actor pose only: no native focus or edge claim. The worker must prove
    // the current knowledge target's centre/sweep/shoulder geometry itself.
    bool pose_only=false;
    const char* gate="none";
};
struct OutdoorSoloInput {
    OutdoorPersonalResult personal;
    OutdoorPersonalInput source;
    // The caller identifies actual commitments; assignment.generation alone
    // never prevents personal combat from interrupting an uncommitted room job.
    bool genuine_commitment=false,allow_noncontact=false;
    // Side movement owns coordinated advances; the individual layer may still
    // react, fire, lean and seek nearby protection within its element.
    bool coordinated_side=false,element_anchor_valid=false;
    // One side-owned opportunity at a time; still requires delivered current
    // suppression, a vulnerable known foe and native route publication.
    bool squad_push_allowed=false;
    Vec3 element_anchor;
    const std::vector<OutdoorSoloRoute>* routes=nullptr;
    const OutdoorSoloThrow* grenade_throw=nullptr;
    const OutdoorSoloGrenadeEscape* grenade_escape=nullptr;
    const OutdoorSoloAimEvidence* aim_evidence=nullptr;
    const OutdoorSoloLeanCapture* lean_capture=nullptr;
};
enum class OutdoorSoloStatus : std::uint8_t {invalid,disabled,ready,pending,stale,cancelled,failed};
enum class OutdoorSoloReason : std::uint8_t {
    unchanged,native_yield,commitment,personal_layer,grenade_escape,grenade_unknown,
    urgent_escape_pending,retreat,stand_and_shoot,shoot_distant,rush,throw_grenade,
    search,freeze,shift_cover,move_to_engage,seek_cover,cover_unavailable,budget
};
struct OutdoorSoloResult {
    OutdoorSoloStatus status=OutdoorSoloStatus::invalid;
    OutdoorSoloReason reason=OutdoorSoloReason::unchanged;
    ProposedOrder proposal;
    bool changed=false,movement_changed=false,reset=false;
    bool navigation_pending=false,throw_pending=false,grenade_fuse_measured=false;
    bool searched=false;FoeId searched_foe;TimeMs searched_source_ms=0;
    FoeId focus;TimeMs source_at_ms=0;Vec3 known_position;
    OutdoorCover here=OutdoorCover::unknown;
    std::size_t work=0,queries=0,candidates=0;
    bool cqb_cost_compared=false,earned_shot=false,lean_static_verified=false;
    const char* lean_gate="not_attempted";
    float stay_cost_m=0,move_cost_m=0;TimeMs effective_hold_ms=0;
};
// One observer, cloned with its side's outer Work. Proposal timers never prove
// execution. Current personal sight alone authorizes enemy firing; K1/shared
// or ghost facts only orient/move. Native publication/restrictions/teammate gates
// remain authoritative. Quota is shared across actors and never refunded.
class CombatOutdoorSolo {
public:
    CombatOutdoorSolo(Config,OutdoorSoloPolicy);
    ~CombatOutdoorSolo();
    CombatOutdoorSolo(CombatOutdoorSolo&&) noexcept;
    CombatOutdoorSolo& operator=(CombatOutdoorSolo&&) noexcept;
    CombatOutdoorSolo(const CombatOutdoorSolo&)=delete;
    CombatOutdoorSolo& operator=(const CombatOutdoorSolo&)=delete;
    std::unique_ptr<CombatOutdoorSolo> clone(const std::function<bool()>& cancelled={}) const;
    OutdoorSoloResult update(const WorldSnapshot&,const ActorSnapshot&,const SideSnapshot&,
        const OutdoorSoloInput&,const EngineServices&,OutdoorSoloQuota&,
        const std::function<bool()>& cancelled={});
    void reset();
private:
    struct Impl;std::unique_ptr<Impl> impl_;
};
}}}
