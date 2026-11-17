#pragma once
#include "../CombatContract.h"
#include <cstddef>

namespace painter { namespace combat { namespace core {
bool finite(const Vec3& value);
bool valid_key(const ActorKey& key);
float distance_squared(const Vec3& a,const Vec3& b);
TimeMs saturated_after(TimeMs now,TimeMs duration);
bool fresh(TimeMs observed,TimeMs now,TimeMs age_limit);
bool personally_visible(const Contact& contact,const ActorSnapshot& actor);
const Contact* select_contact(const ActorSnapshot& actor,const Config& config,TimeMs now);
// Own recorded facts only. The caller may retain latest across ephemeral hit /
// heard batches, but must clear it when identity or ownership changes.
bool personal_evidence_at(const ActorSnapshot& actor,const Config& config,TimeMs now,TimeMs& latest);
struct ObserverSpecies;
// Control-retention evidence from an actual same-native-group observer. This
// never inserts a personal Contact or authorizes aim/fire for the recipient.
bool native_group_evidence_at(const ActorSnapshot& member,const ActorSnapshot& witness,
    const ObserverSpecies&,const Config&,TimeMs now,TimeMs& latest);
// Firing duration is independent of snapshot freshness; zero closes the window.
TimeMs fire_window_until(const Config& config,TimeMs now,TimeMs order_expires);

// Optional already-computed cover evidence. The cheap response never calls a
// ray, path query, engine object, Lua function or worker synchronously.
struct CoverView {
    const CoverSpot* data = nullptr;
    std::size_t count = 0;
    const Vec3* teammate_positions = nullptr;
    std::size_t teammate_count = 0;
};
Order first_response(const ActorSnapshot& actor,const Config& config,TimeMs now,
    std::uint64_t epoch,std::uint64_t geometry,std::uint64_t sequence,
    CoverView cached_cover = {});

// Portable publication preflight; the authoritative publisher still checks
// the real nav node, restrictions, weapon and friendly-fire corridor.
bool publishable(const Order& order,const ActorSnapshot& current,TimeMs now,
    std::uint64_t epoch,std::uint64_t geometry);
// A real, matching in-progress grenade transaction may temporarily replace
// the firearm in hand. This grants neither rifle fire nor a new throw request.
bool continuing_throw(const Order&,const ActorSnapshot&,TimeMs now);
}}}
