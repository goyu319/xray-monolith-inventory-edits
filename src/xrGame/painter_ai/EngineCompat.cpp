#include "stdafx.h"
#include "pch_script.h"
#include "EngineCompat.h"
#include "EngineCapture.h"
#include "EngineCombatAdapter.h"
#include "EngineServices.h"
#include "EngineRecorder.h"
#include "ai/stalker/ai_stalker.h"
#include "stalker_planner.h"
#include "ai_space.h"
#include "alife_simulator.h"
#include "alife_object_registry.h"
#include "../xrServerEntities/xrServer_Objects_ALife.h"

namespace painter { namespace combat { namespace engine {
namespace {
compatmodel::Store store;
const char* const companion_info = "npcx_is_companion";

struct Counters {                                  // per level (compat_reset)
    std::uint64_t begins = 0, begins_refused = 0, seals = 0, seals_failed = 0, invalidations = 0;
    std::uint64_t walks = 0, walks_aborted = 0, handbacks_veto = 0, handbacks_unknown = 0;
    std::uint64_t admissions_free = 0, admissions_veto = 0, admissions_unknown = 0;
} counters;
xr_map<std::uint16_t, std::uint64_t> logged_state;   // id -> the manifest state last logged as phase=compat_unknown
xr_set<std::uint16_t> unknown_men;                    // distinct men an admission demand found unknown this level

std::uint64_t state_signature(const compatmodel::Manifest* m)
{
    if (!m) return 0;
    return (std::uint64_t(m->token) << 32) ^ (std::uint64_t(m->mutation) << 8) ^ (m->sealed ? 2u : 0u) ^ (m->pass_valid ? 1u : 0u);
}

bool identity(CAI_Stalker& actor, ActorKey& key) { return actor_key(actor, key); }
std::uint16_t id_of(CAI_Stalker& actor) { return std::uint16_t(actor.ID()); }

// The planner's evaluator object for an id, or nullptr
const void* evaluator_object(CAI_Stalker& actor, std::uint32_t evaluator_id)
{
    const auto& evaluators = actor.brain().evaluators();
    const auto found = evaluators.find(evaluator_id);
    return found == evaluators.end() ? nullptr : static_cast<const void*>(found->second);
}

// The engine's own story knowledge: the ALife object's story id (the Lua story registry is the registrar's fact)
bool alife_story(std::uint16_t id)
{
    if (!ai().get_alife()) return false;
    const CSE_ALifeDynamicObject* object = ai().alife().objects().object(id, true);
    return object && object->m_story_id != INVALID_STORY_ID;
}
} // namespace

std::uint32_t compat_begin(CAI_Stalker& actor)
{
    ActorKey key;
    if (!identity(actor, key)) return 0;
    const std::uint16_t id = id_of(actor);
    const std::uint32_t token = store.begin(id, key, level_epoch());
    if (!token) { ++counters.begins_refused; return 0; }
    ++counters.begins;
    // the native writers' current values (their changes arrive through compat_info_changed)
    store.native_fact(id, compat::companion, actor.HasInfo(companion_info));
    store.native_story(id, alife_story(id));
    return token;
}

bool compat_evaluator(CAI_Stalker& actor, std::uint32_t token, std::uint32_t evaluator_id)
{
    ActorKey key;
    return identity(actor, key) && store.evaluator(id_of(actor), key, level_epoch(), token, evaluator_id);
}

bool compat_fact(CAI_Stalker& actor, std::uint32_t token, const char* domain, bool value)
{
    ActorKey key;
    const std::uint8_t code = compatmodel::domain_code(domain);
    if (!code || !identity(actor, key)) return false;
    if (!store.fact(id_of(actor), key, level_epoch(), token, code, value)) return false;
    // the stable flags keep one stored state: a fact for the same name updates it (CombatIds compat::Domain 1-5)
    switch (code)
    {
    case compat::story: set_flag(id_of(actor), "story", value); break;
    case compat::companion: set_flag(id_of(actor), "companion", value); break;
    case compat::script_combat: set_flag(id_of(actor), "script_combat", value); break;
    case compat::dont_shoot: set_flag(id_of(actor), "dont_shoot", value); break;
    default: break;
    }
    return true;
}

bool compat_seal(CAI_Stalker& actor, std::uint32_t token)
{
    ActorKey key;
    if (!identity(actor, key)) return false;
    const bool sealed = store.seal(id_of(actor), key, level_epoch(), token,
                                   [&](std::uint32_t evaluator_id) { return evaluator_object(actor, evaluator_id); });
    ++(sealed ? counters.seals : counters.seals_failed);
    return sealed;
}

bool compat_invalidate(CAI_Stalker& actor, std::uint32_t token, const char* domain)
{
    ActorKey key;
    if (!identity(actor, key)) return false;
    std::uint8_t code = 0;
    if (domain && *domain)
    {
        code = compatmodel::domain_code(domain);
        if (!code) return false;               // a misspelt domain never widens into a full invalidation
    }
    const bool done = store.invalidate(id_of(actor), key, level_epoch(), token, code);
    if (done) ++counters.invalidations;
    return done;
}

std::uint32_t compat_token(CAI_Stalker& actor)
{
    ActorKey key;
    return identity(actor, key) ? store.token_of(id_of(actor), key, level_epoch()) : 0;
}

compatmodel::Pass compat_pass_begin(CAI_Stalker& actor)
{
    ActorKey key;
    if (!identity(actor, key)) return {};
    return store.pass_begin(id_of(actor), key, level_epoch());
}

void compat_pass_record(compatmodel::Pass& pass, std::uint32_t evaluator_id, const void* evaluator, bool value)
{
    compatmodel::Store::pass_record(pass, evaluator_id, evaluator, value);
}

bool compat_pass_end(compatmodel::Pass& pass, CAI_Stalker& actor)
{
    ActorKey key;
    if (!identity(actor, key)) return false;
    compatmodel::Store::pass_end(pass, 0, now_ms());
    if (pass.aborted) ++counters.walks_aborted;
    const bool push = combat_config().compat_mode != 0;
    const auto verdict = store.verdict(id_of(actor), key, level_epoch());
    std::uint32_t reason = yield_none;
    if (verdict == compatmodel::Verdict::veto) reason = yield_scheme_block;
    else if (verdict == compatmodel::Verdict::unknown && push) reason = yield_compat_unknown;
    if (!reason) return false;
    ++(reason == yield_scheme_block ? counters.handbacks_veto : counters.handbacks_unknown);
    compat_handback(actor, reason);
    return true;
}

std::uint32_t compat_blocker(CAI_Stalker& actor, ActorKey key, bool push_mode)
{
    const auto verdict = store.verdict(id_of(actor), key, level_epoch());
    if (verdict == compatmodel::Verdict::veto) return yield_scheme_block;
    if (verdict == compatmodel::Verdict::unknown && push_mode) return yield_compat_unknown;
    return yield_none;
}

void compat_admission_demand(CAI_Stalker& actor, ActorKey key, std::uint64_t perception)
{
    const std::uint16_t id = id_of(actor);
    const std::uint64_t epoch = level_epoch();
    if (!store.needs_validation(id, key, epoch, perception) || !actor.brain().initialized()) return;
    const compatmodel::Manifest* manifest = store.current(id, key, epoch);
    if (!manifest) return;
    const std::vector<std::uint32_t> ids = manifest->evaluators;   // evaluate() may run script code: no reference held
    compatmodel::Pass pass = store.pass_begin(id, key, epoch);
    const auto& evaluators = actor.brain().evaluators();
    for (std::uint32_t evaluator_id : ids)
    {
        const auto found = evaluators.find(evaluator_id);
        if (found == evaluators.end()) continue;  // counted as missing by pass_end
        compatmodel::Store::pass_record(pass, evaluator_id, found->second, found->second->evaluate());
    }
    compatmodel::Store::pass_end(pass, perception, now_ms());
    ++counters.walks;
    if (pass.aborted) ++counters.walks_aborted;
}

void compat_note_admission(CAI_Stalker& actor, ActorKey key, bool push_mode)
{
    const std::uint16_t id = id_of(actor);
    const std::uint64_t epoch = level_epoch();
    const auto verdict = store.verdict(id, key, epoch);
    recorder_compat(std::uint16_t(id), verdict == compatmodel::Verdict::clear ? 1 : verdict == compatmodel::Verdict::veto ? 2 : 0);   // native17
    if (verdict == compatmodel::Verdict::clear) { ++counters.admissions_free; logged_state.erase(id); return; }
    if (verdict == compatmodel::Verdict::veto) { ++counters.admissions_veto; logged_state.erase(id); return; }
    ++counters.admissions_unknown;
    unknown_men.insert(id);
    if (!push_mode) return;                    // bridge mode: a shadow count only, the poll decides
    const compatmodel::Manifest* m = store.current(id, key, epoch);
    const std::uint64_t signature = state_signature(m);
    const auto last = logged_state.find(id);
    if (last != logged_state.end() && last->second == signature) return;
    logged_state[id] = signature;
    string256 missing = "";
    if (!m) xr_strcpy(missing, "no_manifest");
    else
    {
        for (std::uint8_t d = 1; d < compatmodel::domain_count; ++d)
            if (!m->facts[d].known)
            {
                if (missing[0]) xr_strcat(missing, ",");
                xr_strcat(missing, compat::domain_names[d]);
            }
        if (!missing[0]) xr_strcpy(missing, m->sealed ? "none (no complete pass)" : "none (not sealed)");
    }
    Msg("[painter_combat] phase=compat_unknown actor=%u generation=%llu token=%u sealed=%u pass=%u evaluators=%u unknown_domains=%s",
        unsigned(id), key.generation, m ? m->token : 0u, m && m->sealed ? 1u : 0u, m && m->pass_valid ? 1u : 0u,
        m ? unsigned(m->evaluators.size()) : 0u, missing);
}

void compat_summary(const char* level)
{
    unsigned registered = 0, sealed = 0;
    // the store's manifests of this level (the level epoch is current until on_level_load)
    store.for_each([&](const compatmodel::Manifest& m) { ++registered; if (m.sealed) ++sealed; });
    const auto& c = counters;
    Msg("[painter_combat] phase=compat_summary level=%s mode=%u registered=%u sealed=%u unsealed=%u begins=%llu "
        "begins_refused=%llu seals=%llu seals_failed=%llu invalidations=%llu walks=%llu walks_aborted=%llu "
        "handbacks_veto=%llu handbacks_unknown=%llu admissions_free=%llu admissions_veto=%llu admissions_unknown=%llu "
        "unknown_men=%u", level ? level : "-", unsigned(combat_config().compat_mode), registered, sealed, registered - sealed,
        c.begins, c.begins_refused, c.seals, c.seals_failed, c.invalidations, c.walks, c.walks_aborted, c.handbacks_veto,
        c.handbacks_unknown, c.admissions_free, c.admissions_veto, c.admissions_unknown, unsigned(unknown_men.size()));
}

CompatState compat_state(std::uint16_t id, ActorKey key)
{
    return store.state(id, key, level_epoch());
}

void compat_info_changed(std::uint16_t id, const char* info, bool present)
{
    if (info && !xr_strcmp(info, companion_info)) store.native_fact(id, compat::companion, present);
}

void compat_forget(std::uint16_t id)
{
    store.forget(id);
    logged_state.erase(id);
}
void compat_reset()
{
    store.reset();
    counters = Counters{};
    logged_state.clear();
    unknown_men.clear();
}

}}} // namespace painter::combat::engine
