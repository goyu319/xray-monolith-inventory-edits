#pragma once
#include "CombatCore.h"
#include "SquadTactics.h"
#include "CombatRouteProof.h"
#include <functional>

namespace painter { namespace combat { namespace core {
// A third-party observation, deliberately NOT an ActorSnapshot. The patient has
// no movement lease, contacts, inventory, self-knowledge or admission here.
struct FriendlyCasualtyFact {
    ActorKey observer,target;
    std::uint64_t observer_owner=0,observer_restrictions=0;
    std::uint32_t side_id=0,faction=0,mission_revision=0;
    std::uint64_t group=0,target_native_squad=0;
    TimeMs observed_at_ms=0;
    Vec3 position;
    bool visible_now=false,alive=false,wounded=false;
    Evidence friendly=Evidence::unknown;
    decltype(ActorSnapshot{}.aid_eligibility) aid_eligibility;
};
struct FriendlyCasualtyEvidence {
    std::uint64_t level_epoch=0,snapshot_seq=0,map_hash=0,geometry_revision=0;
    TimeMs captured_at_ms=0;
    std::vector<FriendlyCasualtyFact> facts;
};
struct FriendlyCasualtyValidation {
    bool accepted=false,cancelled=false;
    std::size_t rejected=0;
    std::vector<FriendlyCasualtyFact> facts;
};
// Source validation only. Registry additionally requires exact previously known
// or explicitly authored membership before retaining any passive target key.
FriendlyCasualtyValidation validate_friendly_casualties(const WorldSnapshot&,
    const FriendlyCasualtyEvidence&,const Config&,const std::function<bool()>& cancelled={});
bool friendly_casualty_member(const FriendlyCasualtyFact&,const SideSnapshot&);
// Complete only a matching native mission producer roster; never alter its mission.
void complete_friendly_casualty_rosters(WorldSnapshot&,const std::vector<SideSnapshot>& known,
    const FriendlyCasualtyEvidence&,const Config&);
bool friendly_casualty_aid_allowed(const FriendlyCasualtyFact&,ActorKey helper,TimeMs now,TimeMs max_age);
struct CombatAidBudget {std::size_t work_left=0,queries_left=0,work_used=0,queries_used=0;RouteProofBudget routes;};
struct CombatAidResult {bool cancelled=false;std::vector<ProposedOrder> proposals;std::size_t pending=0;};
// A worker overlay using the existing native aid transaction. It never claims,
// treats, or re-admits a patient. Route/shooter samples are geometry evidence,
// not current visibility or a guarantee against an unknown shooter.
CombatAidResult combat_aid_orders(const WorldSnapshot&,const SideSnapshot&,const KnowledgeFrame&,
    const std::vector<FriendlyCasualtyFact>&,const std::vector<ProposedOrder>& base,const std::vector<Order>& accepted,
    const Config&,const EngineServices&,const RouteProofService*,CombatAidBudget&,std::uint64_t& assignment_sequence,
    const std::function<bool()>& cancelled={});
}}}
