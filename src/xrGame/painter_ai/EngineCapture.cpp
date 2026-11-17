#include "pch_script.h"
#include "EngineCapture.h"
#include "EngineServices.h"
#include "EngineNearMiss.h"
#include "EngineCompat.h"
#include "Level.h"
#include "ai/stalker/ai_stalker.h"
#include "memory_manager.h"
#include "memory_space.h"
#include "visual_memory_manager.h"
#include "sound_memory_manager.h"
#include "hit_memory_manager.h"
#include "enemy_manager.h"
#include "danger_manager.h"
#include "danger_object.h"
#include "stalker_planner.h"
#include "stalker_decision_space.h"
#include "stalker_movement_manager_smart_cover.h"
#include "ai_object_location.h"
#include "restricted_object.h"
#include "inventory.h"
#include "Weapon.h"
#include "WeaponKnife.h"
#include "WeaponShotgun.h"
#include "Grenade.h"
#include "doors_manager.h"
#include "doors_door.h"
#include "PhysicObject.h"
#include "ai_space.h"
#include "../xrServerEntities/script_engine.h"
#include "../xrServerEntities/ai_sounds.h"
#include "../xrServerEntities/inventory_space.h"
#include "script_game_object.h"
#include <algorithm>
#include <unordered_map>

namespace painter { namespace combat { namespace engine {
namespace {
// Authority thread only (the publish task): plain statics, no locks.
std::uint64_t epoch = 1;
std::uint32_t last_global = 0;
std::uint64_t global_high = 0;
std::unordered_map<std::uint16_t, std::uint32_t> flags;
std::unordered_map<std::uint64_t, Mission> missions;   // side key -> its mission (revision kept across clears)

// throw_state.phase / aid_state.phase (CombatContract.h ActorSnapshot)
constexpr std::uint8_t throw_prepared = 1, throw_in_hand = 2, throw_thrown = 3, throw_failed = 4;
struct ThrowReceipt {
    std::uint32_t serial = 0;
    std::uint8_t phase = throw_prepared, reason = throw_no_target;
    TimeMs at_ms = 0;
    std::uint32_t releases = 0;                     // his release count at begin_throw
};
std::unordered_map<std::uint16_t, ThrowReceipt> throws;
struct Release {                                    // the projectiles that left a man's hand on this level (note_release)
    std::uint32_t count = 0;
    std::uint16_t projectile = 0xffff, item = 0xffff;
    u32 at = 0;
};
std::unordered_map<std::uint16_t, Release> releases;
std::uint32_t releases_of(std::uint16_t id)
{
    const auto found = releases.find(id);
    return found == releases.end() ? 0 : found->second.count;
}
struct AidReceipt { ActorKey target; std::uint8_t phase = 0; TimeMs at_ms = 0; };
std::unordered_map<std::uint16_t, AidReceipt> aids;
struct AidEligibility {                             // set_aid: his side's script state as the wounded one (v2.18)
    bool allowed = false, claimed = false;
    std::uint16_t claimant = 0xffff;
    u32 at = 0;
};
std::unordered_map<std::uint16_t, AidEligibility> eligibility;

// The engine's aimed-throw reason (CAI_Stalker::painter_throw_reason, as painter_throw_state reports it) as a code
std::uint8_t throw_reason_code(const char* reason)
{
    static const struct { const char* name; std::uint8_t code; } table[] = {
        {"ok", throw_ok}, {"no_target", throw_no_target}, {"blocked", throw_blocked}, {"no_arc", throw_no_arc},
        {"out_of_range", throw_out_of_range}, {"no_grenade", throw_no_grenade}, {"door_behind", throw_door_behind}};
    if (reason)
        for (const auto& row : table)
            if (!xr_strcmp(row.name, reason)) return row.code;
    return throw_no_target;
}

Vec3 value(const Fvector& v) { return {v.x, v.y, v.z}; }
Vec3 flat_unit(Fvector v)
{
    v.y = 0;
    const float m = v.magnitude();
    if (!(m > 1e-4f)) return {};
    v.div(m);
    return value(v);
}

WeaponKind weapon_kind(CWeapon& weapon)
{
    if (smart_cast<CWeaponKnife*>(&weapon)) return WeaponKind::melee;
    if (smart_cast<CWeaponShotgun*>(&weapon)) return WeaponKind::shotgun;
    static xr_map<shared_str, WeaponKind> cache; // GAMMA's own weapon classes ("kind" in each weapon section)
    const shared_str section = weapon.cNameSect();
    const auto known = cache.find(section);
    if (known != cache.end()) return known->second;
    WeaponKind kind = WeaponKind::none;
    LPCSTR text = pSettings->line_exist(section, "kind") ? pSettings->r_string(section, "kind") : "";
    if (!xr_strcmp(text, "w_rifle")) kind = WeaponKind::rifle;
    else if (!xr_strcmp(text, "w_pistol")) kind = WeaponKind::pistol;
    else if (!xr_strcmp(text, "w_sniper")) kind = WeaponKind::sniper;
    else if (!xr_strcmp(text, "w_shotgun")) kind = WeaponKind::shotgun;
    else if (!xr_strcmp(text, "w_smg")) kind = WeaponKind::smg;
    else if (!xr_strcmp(text, "w_explosive")) kind = WeaponKind::launcher;
    else if (!xr_strcmp(text, "w_melee")) kind = WeaponKind::melee;
    cache.emplace(section, kind);
    return kind;
}

WeaponState weapon_state(CWeapon& weapon)
{
    switch (weapon.GetState())
    {
    case CWeapon::eFire:
    case CWeapon::eFire2: return WeaponState::firing;
    case CWeapon::eReload: return WeaponState::reloading;
    case CWeapon::eMisfire: return WeaponState::misfire;
    case CHUDState::eShowing:
    case CHUDState::eHiding:
    case CHUDState::eHidden:
    case CWeapon::eSwitch:
    case CWeapon::eSwitchMode: return WeaponState::switching;
    default: return weapon.GetAmmoElapsed() > 0 ? WeaponState::idle : WeaponState::empty;
    }
}

DangerKind danger_kind(const CDangerObject& danger)
{
    switch (danger.type())
    {
    case CDangerObject::eDangerTypeGrenade: return DangerKind::grenade;
    case CDangerObject::eDangerTypeAttackSound:
    case CDangerObject::eDangerTypeEnemySound: return DangerKind::enemy_sound;
    case CDangerObject::eDangerTypeEntityAttacked:
    case CDangerObject::eDangerTypeAttacked: return DangerKind::attack;
    case CDangerObject::eDangerTypeEntityDeath:
    case CDangerObject::eDangerTypeFreshEntityCorpse: return DangerKind::corpse;
    case CDangerObject::eDangerTypeBulletRicochet: return DangerKind::hit;
    default: return DangerKind::other;
    }
}

SoundKind sound_kind(u32 type)
{
    if ((type & SOUND_TYPE_WEAPON_SHOOTING) == SOUND_TYPE_WEAPON_SHOOTING) return SoundKind::gunshot;
    if ((type & SOUND_TYPE_WORLD_OBJECT_EXPLODING) == SOUND_TYPE_WORLD_OBJECT_EXPLODING) return SoundKind::explosion;
    if (type & (SOUND_TYPE_INJURING | SOUND_TYPE_DYING)) return SoundKind::injury;
    if ((type & SOUND_TYPE_MONSTER_STEP) == SOUND_TYPE_MONSTER_STEP) return SoundKind::footstep;
    if ((type & SOUND_TYPE_MONSTER_TALKING) == SOUND_TYPE_MONSTER_TALKING) return SoundKind::voice;
    if (type & SOUND_TYPE_WEAPON) return SoundKind::weapon;
    if (type & SOUND_TYPE_ITEM) return SoundKind::item;
    return SoundKind::other;
}

bool planner_property(CAI_Stalker& actor, StalkerDecisionSpace::EWorldProperties id)
{
    const auto& evaluators = actor.brain().evaluators();
    const auto found = evaluators.find(u32(id));
    return found != evaluators.end() && found->second->evaluate();
}

template <class T, std::size_t N> void push(Bounded<T, N>& list, const T& item)
{
    if (list.count < N) list.items[list.count++] = item;
}

// A memory record's u32 level time on the 64-bit clock: the same age, so a record older than a wrap never reads as
// being in the future. never (u32(-1), the visual memory's "not seen") stays 0.
TimeMs widen(u32 t, TimeMs now)
{
    if (t == u32(-1)) return 0;
    const u32 age = u32(now) - t;
    return now >= age ? now - age : 0;
}

Contact contact(CAI_Stalker& actor, const CEntityAlive& other, const CaptureContext& context)
{
    Contact c;
    c.key = context.key_of ? context.key_of(other) : ActorKey{other.ID(), 0};
    const auto info = actor.memory().memory(&other);
    const bool visible = actor.memory().visual().visible_now(&other);
    c.visible = visible;
    c.hostile = actor.is_relation_enemy(&other);
    c.observed_at_ms = widen(info.m_level_time, context.now_ms);
    c.perception_seq = c.observed_at_ms;          // a change detector: the record's level time (no update counter)
    c.last_seen_ms = widen(actor.memory().visual().visible_object_time_last_seen(&other), context.now_ms);
    const Fvector at = visible ? other.Position() : info.m_object_params.m_position;
    c.position = value(at);
    c.distance = actor.Position().distance_to(at);
    if (visible)
    {
        c.facing = flat_unit(other.Direction());
        c.alive_when_observed = !!other.g_Alive();
        const Fvector to_me = Fvector().sub(actor.Position(), other.Position());
        const Vec3 me = flat_unit(to_me);
        c.facing_me = c.facing.x * me.x + c.facing.z * me.z > .866f; // within 30 degrees
    }
    if (visible) if (auto* stalker = smart_cast<const CAI_Stalker*>(&other)) c.wounded = stalker->wounded();
    return c;
}
} // namespace

TimeMs now_ms()
{
    const u32 t = Device.dwTimeGlobal;
    if (t < last_global) global_high += std::uint64_t(1) << 32;
    last_global = t;
    return global_high | t;
}
std::uint64_t level_epoch() { return epoch; }
void on_level_load()
{
    ++epoch;
    flags.clear();
    missions.clear();
    throws.clear();
    aids.clear();
    releases.clear();
    eligibility.clear();
    near_miss_reset();
    compat_reset();
}

bool set_flag(std::uint16_t id, const char* name, bool on)
{
    static const struct { const char* name; std::uint32_t bit; } table[] = {
        {"story", flag_story}, {"companion", flag_companion}, {"scheme_block", flag_scheme_block},
        {"script_combat", flag_script_combat}, {"dont_shoot", flag_dont_shoot}, {"help_wounded", flag_help_wounded},
        {"wounded_scheme", flag_wounded_scheme}, {"requested_control", flag_requested_control}};
    if (!name) return false;
    for (const auto& row : table)
        if (!xr_strcmp(row.name, name))
        {
            auto& bits = flags[id];
            bits = on ? (bits | row.bit) : (bits & ~row.bit);
            if (!bits) flags.erase(id);
            return true;
        }
    return false;
}
std::uint32_t flags_of(std::uint16_t id)
{
    const auto found = flags.find(id);
    return found == flags.end() ? 0 : found->second;
}
void forget(std::uint16_t id)
{
    flags.erase(id);
    throws.erase(id);
    aids.erase(id);
    releases.erase(id);
    eligibility.erase(id);
    near_miss_forget(id);
    compat_forget(id);
}

std::uint64_t side_key(const CEntityAlive& entity)
{
    return (std::uint64_t(std::uint32_t(entity.g_Team())) << 32) | (std::uint64_t(std::uint16_t(entity.g_Squad())) << 16) |
        std::uint16_t(entity.g_Group());
}

void set_mission(std::uint64_t side, const Mission& mission)
{
    Mission& stored = missions[side];
    const std::uint32_t revision = stored.revision + 1;
    stored = mission;
    stored.revision = revision;
}
void clear_mission(std::uint64_t side)
{
    Mission& stored = missions[side];
    const std::uint32_t revision = stored.revision + 1;
    stored = Mission{};
    stored.revision = revision;
}
bool mission_of(std::uint64_t side, Mission& out)
{
    const auto found = missions.find(side);
    if (found == missions.end()) return false;
    out = found->second;
    return true;
}

void begin_throw(std::uint16_t id, std::uint32_t serial)
{
    if (!serial) { throws.erase(id); return; }
    ThrowReceipt receipt;
    receipt.serial = serial;
    receipt.at_ms = now_ms();
    receipt.releases = releases_of(id);             // a release after this point is this throw's
    throws[id] = receipt;
}
void end_throw(std::uint16_t id, std::uint8_t reason)
{
    const auto found = throws.find(id);
    if (found == throws.end()) return;
    if (!reason) { throws.erase(found); return; }
    if (found->second.phase == throw_thrown || found->second.phase == throw_failed) return;   // already final
    found->second.phase = throw_failed;
    found->second.reason = reason;
    found->second.at_ms = now_ms();
}
void note_release(std::uint16_t thrower, std::uint16_t projectile, std::uint16_t item)
{
    Release& release = releases[thrower];
    ++release.count;
    release.projectile = projectile;
    release.item = item;
    release.at = Device.dwTimeGlobal;
}
void set_aid(std::uint16_t id, bool allowed, bool claimed, std::uint16_t claimant)
{
    AidEligibility& state = eligibility[id];
    state.allowed = allowed;
    state.claimed = claimed;
    state.claimant = claimed ? claimant : std::uint16_t(0xffff);
    state.at = Device.dwTimeGlobal;
}
void note_aid(std::uint16_t id, ActorKey target, std::uint8_t phase)
{
    if (!phase) { aids.erase(id); return; }
    AidReceipt& receipt = aids[id];
    if (receipt.phase == phase && receipt.target == target) return;
    receipt.target = target;
    receipt.phase = phase;
    receipt.at_ms = now_ms();
}

void notify_release(CAI_Stalker& actor)
{
    ::luabind::functor<void> release;
    if (ai().script_engine().functor("painter_combat.on_release", release))
        release(actor.lua_game_object());
}

decltype(ActorSnapshot{}.aid_eligibility) aid_eligibility_of(std::uint16_t id,TimeMs now,
    const std::function<ActorKey(const CEntityAlive&)>& key_of){
    decltype(ActorSnapshot{}.aid_eligibility) out{};
    const auto state = eligibility.find(id);
    if (state != eligibility.end())
    {
        const AidEligibility& e = state->second;
        out.allowed = e.allowed ? Evidence::yes : Evidence::no;
        out.claimed = e.claimed;
        if (e.claimed && e.claimant != 0xffff)
        {
            auto* claimant = smart_cast<CEntityAlive*>(Level().Objects.net_Find(e.claimant));
            out.claimant = claimant && claimant->g_Alive() && key_of ? key_of(*claimant)
                                                                                             : ActorKey{e.claimant, 0};
        }
        out.observed_at_ms = widen(e.at, now);
    }
    return out;
}

void capture_personal_events(CAI_Stalker& actor,const CaptureContext& context,ActorSnapshot& out,bool consume_misses)
{
    auto& memory=actor.memory();const auto* selected=memory.enemy().selected();
    memory.sound().personal_sounds().recent(u32(context.now_ms),2000,[&](const OwnSoundSample& sound){
        if(out.heard.count>=out.heard.items.size())return;
        const auto* source=static_cast<const CGameObject*>(sound.source);
        if(source&&source->ID()!=sound.source_id)return;
        if(source&&(source->ID()==actor.ID()||(source->H_Parent()&&source->H_Parent()->ID()==actor.ID())))return;
        const auto kind=sound_kind(sound.type);if(kind==SoundKind::injury)return;
        const auto* entity=smart_cast<const CEntityAlive*>(source);
        if(!entity&&source)entity=smart_cast<const CEntityAlive*>(source->H_Parent());
        if(entity&&!actor.is_relation_enemy(entity))return;
        if(!entity&&!selected&&(kind==SoundKind::item||kind==SoundKind::other))return;
        const auto age=u32(u32(context.now_ms)-sound.at);if(context.now_ms<age)return;
        Heard h;h.kind=kind;h.position={sound.x,sound.y,sound.z};h.power=sound.power;h.at_ms=context.now_ms-age;
        if(entity&&context.key_of)h.source=context.key_of(*entity);push(out.heard,h);
    });
    auto receive=[&](const nearmiss::Record& r){
        if(out.near_misses.count>=out.near_misses.items.size())return;
        const auto age=u32(u32(context.now_ms)-r.at);
        if(context.now_ms<age||age>combat_config().knowledge.input_age_ms||age>=0x80000000u)return;
        if(!std::isfinite(r.miss_m)||r.miss_m<0||!std::isfinite(r.origin.x)||!std::isfinite(r.origin.y)||!std::isfinite(r.origin.z))return;
        // The tracker has no birth-generation receipt for the projectile's
        // shooter ID. The measured crack remains anonymous; never resolve a
        // recycled ID into relation/type/position or a firing target.
        NearMiss miss;miss.at_ms=context.now_ms-age;miss.miss_m=r.miss_m;miss.origin={r.origin.x,r.origin.y,r.origin.z};
        miss.dir=flat_unit(Fvector().set(r.origin.x,r.origin.y,r.origin.z).sub(actor.Position()));
        push(out.near_misses,miss);
    };
    if(consume_misses)near_miss_take(actor.ID(),u32(context.previous_snapshot_ms),receive);
    else near_miss_tracker().peek(actor.ID(),u32(context.previous_snapshot_ms),receive);
}

void capture_actor(CAI_Stalker& actor, const CaptureContext& context, ActorSnapshot& out)
{
    out = ActorSnapshot{};
    out.key = context.key;
    out.level_epoch = context.level_epoch;
    out.snapshot_seq = context.snapshot_seq;
    out.ownership_generation = context.ownership_generation;
    out.restrictions_revision = context.restrictions_revision;
    out.last_rejected = context.last_rejected;
    out.observed_at_ms = context.now_ms;
    // flag_requested_control only from the explicit set_flag; the adapter's admission (owned, requested or in contact)
    // arrives as context.requested_control and becomes flag_admitted (v2.10), which the core plans on
    out.flags = flags_of(actor.ID()) | (context.requested_control ? std::uint32_t(flag_admitted) : 0u);
    out.alive = !!actor.g_Alive();
    out.position = value(actor.Position());
    out.forward = flat_unit(actor.Direction());
    out.health = actor.GetfHealth();
    out.node_id = actor.ai_location().level_vertex_id();
    out.faction = std::uint32_t(actor.Community());
    out.native_squad = side_key(actor);
    if (context.buildings && context.buildings->locate)
    {
        const auto where = context.buildings->locate(out.node_id);
        out.building_id = where.first;
        out.room_id = where.second;
    }
    std::uint32_t yields = 0;
    if (!out.alive)
    {
        out.yield_reasons = yield_dead;
        return;                                   // no head, inventory or memory reads after death
    }
    auto& movement = actor.movement();
    const auto& head = movement.head_orientation().current;
    out.head_dir = value(Fvector().setHP(-head.yaw, -head.pitch));
    out.head_pitch_rad = -head.pitch;
    out.wounded = actor.wounded() || (out.flags & flag_wounded_scheme);
    const auto& graph = ai().level_graph();
    out.current_position_accessible = graph.valid_vertex_id(out.node_id) && movement.restrictions().accessible(out.node_id);
    out.accessible_current_order = graph.valid_vertex_id(context.current_goal_node) &&
        movement.restrictions().accessible(context.current_goal_node);

    // weapon and grenades
    auto& inventory = actor.inventory();
    if (auto* weapon = smart_cast<CWeapon*>(inventory.ActiveItem()))
    {
        out.weapon.id = weapon->ID();
        out.weapon.kind = weapon_kind(*weapon);
        out.weapon.state = weapon_state(*weapon);
        out.weapon.ammo_in_mag = std::uint16_t(std::max(0, weapon->GetAmmoElapsed()));
        out.weapon.mag_size = std::uint16_t(std::max(0, weapon->GetAmmoMagSize()));
        // v2.10: rounds of the weapon's suitable ammo types outside the magazine (999 in all when the weapon is unlimited)
        out.weapon.ammo_reserve = std::uint16_t(clampr(weapon->GetSuitableAmmoTotal() - weapon->GetAmmoElapsed(), 0, 65535));
        out.reloading = out.weapon.state == WeaponState::reloading;
        if (!out.weapon.ammo_in_mag && !out.weapon.ammo_reserve) yields |= yield_no_weapon;   // empty and cannot reload
    }
    else
        yields |= yield_no_weapon;
    if (auto* slot = inventory.ItemFromSlot(GRENADE_SLOT)) out.grenades.slot_item = slot->object_id();
    for (auto* item : inventory.m_all)
        if (smart_cast<CGrenade*>(item) && out.grenades.count < 255) ++out.grenades.count;

    // v2.9 receipts. The throw is thrown only on the engine's own release: a live projectile leaving his hand after
    // begin_throw (note_release: the hand grenade's ownership reject of its projectile, sent at the throw's motion mark).
    // Never from a grenade count drop (a removed or transferred item), the throwing animation starting (before the release
    // mark) or the stock on_throw_completed (the stock throw action calls it on an inventory change). In hand while the
    // grenade is armed, ready or being thrown; failed only through the executor's end_throw. The reason is the engine's
    // aimed-throw reason. The aid: as the executor reported it.
    // v2.16: the engine's release receipt itself, for every man (his count on this level and the last projectile)
    const auto release = releases.find(actor.ID());
    if (release != releases.end())
    {
        out.throw_state.releases = release->second.count;
        out.throw_state.projectile = release->second.projectile;
        out.throw_state.source_item = release->second.item;
        out.throw_state.released_at_ms = widen(release->second.at, context.now_ms);
    }
    const auto receipt = throws.find(actor.ID());
    if (receipt != throws.end())
    {
        ThrowReceipt& r = receipt->second;
        if (r.phase != throw_thrown && r.phase != throw_failed)
        {
            const auto* grenade = smart_cast<CGrenade*>(inventory.ActiveItem());
            const u32 state = grenade ? grenade->GetState() : u32(CHUDState::eIdle);
            std::uint8_t phase = throw_prepared;
            std::uint8_t reason = throw_reason_code(actor.painter_throw_active() ? actor.painter_throw_reason() : nullptr);
            if (releases_of(actor.ID()) != r.releases)
            {
                phase = throw_thrown;
                reason = throw_ok;
            }
            else if (grenade && state >= CMissile::eThrowStart && state <= CMissile::eThrowEnd)
                phase = throw_in_hand;
            if (phase != r.phase || reason != r.reason)
            {
                r.phase = phase;
                r.reason = reason;
                r.at_ms = context.now_ms;
            }
        }
        out.throw_state.serial = r.serial;
        out.throw_state.phase = r.phase;
        out.throw_state.reason = r.reason;
        out.throw_state.at_ms = r.at_ms;
    }
    const auto aid = aids.find(actor.ID());
    if (aid != aids.end())
    {
        out.aid_state.target = aid->second.target;
        out.aid_state.phase = aid->second.phase;
        out.aid_state.at_ms = aid->second.at_ms;
    }
    // v2.18: his side's script state as the wounded one (set_aid); allowed stays unknown until the first push. A claim
    // keeps claimed true even when its claimant is no live entity any more: then the raw id with generation 0.
    out.aid_eligibility=aid_eligibility_of(actor.ID(),context.now_ms,context.key_of);

    // the selected enemy (never implies visible: visible is set only when seen now)
    auto& memory = actor.memory();
    const CEntityAlive* selected = memory.enemy().selected();
    if (selected) out.enemy = contact(actor, *selected, context);

    // what he sees: enemies nearest first (8), corpses (8)
    struct Seen { float d; const CEntityAlive* entity; };
    xr_vector<Seen> enemies, dead, friends;
    for (const auto& seen : memory.visual().objects())
    {
        auto* entity = smart_cast<const CEntityAlive*>(seen.m_object);
        if (!entity || entity->ID() == actor.ID() || entity->getDestroy()) continue;
        // Remembered objects do not reveal their current life state through walls.
        if (!memory.visual().visible_now(entity)) continue;
        const float d = actor.Position().distance_to_sqr(seen.m_object_params.m_position);
        if (!entity->g_Alive())
        {
            // v2.22: a body his own community counts as a FRIEND is not reported (his relation knowledge); enemies and
            // neutrals stay - a dead ally of another squad is never read as an enemy down
            if (actor.tfGetRelationType(entity) != ALife::eRelationTypeFriend) dead.push_back({d, entity});
        }
        else if (actor.is_relation_enemy(entity)) enemies.push_back({d, entity});
        else if (const auto* stalker = smart_cast<const CAI_Stalker*>(entity))
        {
            if (actor.tfGetRelationType(stalker) == ALife::eRelationTypeFriend &&
                stalker->Community() == actor.Community()) friends.push_back({d, stalker});
        }
    }
    const auto nearest = [](const Seen& a, const Seen& b) { return a.d < b.d; };
    std::sort(enemies.begin(), enemies.end(), nearest);
    std::sort(dead.begin(), dead.end(), nearest);
    std::sort(friends.begin(), friends.end(), nearest);
    for (const auto& e : enemies) { if (out.visible_enemies.count >= 8) break; push(out.visible_enemies, contact(actor, *e.entity, context)); }
    for (const auto& e : friends) { if (out.visible_friends.count >= 8) break;
        push(out.visible_friends, context.key_of ? context.key_of(*e.entity) : ActorKey{e.entity->ID(), 0}); }
    for (const auto& e : dead)
    {
        if (out.seen_dead.count >= 8) break;
        push(out.seen_dead, context.key_of ? context.key_of(*e.entity) : ActorKey{e.entity->ID(), 0});
    }
    if (selected && out.enemy.key.id != invalid_id)
    {
        // finish allowed: the enemy is wounded, he can kill it, and no able enemy was seen in the last 3 s
        bool able_seen = false;
        for (const auto& e : enemies)
        {
            auto* s = smart_cast<const CAI_Stalker*>(e.entity);
            if (e.entity != selected && !(s && s->wounded()) &&
                context.now_ms - std::min<TimeMs>(context.now_ms, widen(memory.visual().visible_object_time_last_seen(e.entity), context.now_ms)) <= 3000)
                able_seen = true;
        }
        out.enemy.finish_allowed = out.enemy.wounded && actor.can_kill_enemy() && !able_seen;
    }

    capture_personal_events(actor,context,out,true);
    // Only this man's physical damage producer. Stock hit().objects() is a
    // shared table refreshed by fake zero-damage hits on every heard gunshot.
    memory.hit().physical_hits().since(u32(context.previous_snapshot_ms),u32(context.now_ms),[&](const PhysicalHitSample& hit) {
        if (out.hits.count >= out.hits.items.size()) return;
        Hit h;
        const auto age=u32(u32(context.now_ms)-hit.at);
        if(context.now_ms<age||age>combat_config().knowledge.input_age_ms||age>combat_config().memory_max_age_ms) return;
        h.at_ms = context.now_ms-age; // physical u32(-1) is a real wrap time, not a visual "never seen" sentinel
        h.dir = flat_unit(Fvector().set(hit.x,hit.y,hit.z));
        // The native damage/flinch system still receives every wound. Combat
        // knowledge receives attack receipts only: falls, anomalies and allied
        // accidental fire must not manufacture an unseen hostile direction.
        if(!hit.hostile_attack)return;
        const auto* object=static_cast<const CObject*>(hit.source);
        if(object&&object->ID()==hit.source_id&&context.key_of)
            if(const auto* entity=smart_cast<const CEntityAlive*>(object))h.from=context.key_of(*entity);
        push(out.hits, h);
    });

    if (const CDangerObject* danger = memory.danger().selected())
    {
        out.danger.kind = danger_kind(*danger);
        out.danger.perceive = std::uint8_t(danger->perceive_type());
        out.danger.at_ms = widen(danger->time(), context.now_ms);
        out.danger.position = value(danger->position());
    }
    out.native_properties.enemy = planner_property(actor, StalkerDecisionSpace::eWorldPropertyEnemy);
    out.native_properties.danger = planner_property(actor, StalkerDecisionSpace::eWorldPropertyDanger);
    out.native_properties.anomaly = planner_property(actor, StalkerDecisionSpace::eWorldPropertyAnomaly);

    // yields the engine knows
    if (out.wounded) yields |= yield_wounded;
    if (out.danger.kind == DangerKind::grenade) yields |= yield_grenade;
    if (auto* grenade = smart_cast<CGrenade*>(inventory.ActiveItem()))
        if (grenade->GetState() >= CMissile::eThrowStart && grenade->GetState() <= CMissile::eThrowEnd) yields |= yield_grenade;
    if (out.native_properties.anomaly) yields |= yield_anomaly;
    if (movement.in_smart_cover()) yields |= yield_smart_cover;
    if (actor.GetScriptControl()) yields |= yield_script_control;
    if (out.flags & flag_story) yields |= yield_story;
    if (out.flags & flag_companion) yields |= yield_companion;
    if (out.flags & flag_scheme_block) yields |= yield_scheme_block;
    if (out.flags & flag_script_combat) yields |= yield_script_combat;
    out.yield_reasons = yields;
}

void capture_doors(const Vec3& centre, float radius, std::vector<Door>& out)
{
    capture_doors_bounded(centre, radius, static_cast<std::size_t>(invalid_id), out);
}

void capture_doors_bounded(const Vec3& centre, float radius, std::size_t max_rows, std::vector<Door>& out)
{
    out.clear();
    if (!max_rows) return;
    doors::doors_type found;
    ai().doors().nearest(Fvector().set(centre.x, centre.y, centre.z), radius, found);
    for (doors::door* d : found)
    {
        if (out.size() >= max_rows) break;
        Door door;
        door.id = d->object().ID();
        const bool open = d->state() == doors::door_state_open, target_open = d->target_state() == doors::door_state_open;
        door.state = open == target_open ? (open ? DoorState::open : DoorState::closed) : (target_open ? DoorState::opening : DoorState::closing);
        door.open = open ? 1.f : 0.f;
        door.locked = d->is_locked(doors::door_state_open);
        door.portal = door_portal(door.id, value(d->object().Position()));   // the same match as Portal::door
        out.push_back(door);
    }
}

}}} // namespace painter::combat::engine
