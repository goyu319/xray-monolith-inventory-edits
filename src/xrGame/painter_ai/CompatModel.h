#pragma once
// painter_ai/CompatModel.h - the compatibility push model's bookkeeping (contract v2.21: CompatState, the six
// level.painter_combat_compat_* exports). Engine-free so it is tested offline (native-seam-tests/test_compat.cpp);
// EngineCompat.cpp binds it to the stalker planner's forced pass, the Lua exports, the native writers and capture.
//
// One manifest per man, bound to his ActorKey (id + generation) and the level epoch: an old token, an old generation or
// another epoch is refused, so no stale identity can clear the successor's veto. The verdict a man gets:
//   veto    - a known fact of a veto domain is true, or a required evaluator was true in his last complete pass;
//   unknown - no current manifest, not sealed, a mandatory domain unknown, or no complete pass of the sealed set;
//   clear   - sealed, every domain known and false, and the last complete pass of exactly his required set was false.
// Veto domains are the bridge's (painter_native_combat_compat.check): story, companion, script_combat, dont_shoot,
// scripted_target, legacy_controller, help_claim, grenade_request. not_for_help is aid eligibility, not ownership.
#include "CombatContract.h"
#include "CombatIds.h"
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4995)                  // xrCore marks strcmp deprecated; this header is engine-free
#endif

namespace painter { namespace combat { namespace compatmodel {

constexpr std::uint8_t domain_count = 10;      // compat::Domain 1..9; 0 unused

inline bool veto_domain(std::uint8_t d)
{
    return d >= compat::story && d <= compat::grenade_request && d != compat::not_for_help;
}
inline std::uint8_t domain_code(const char* name)
{
    if (!name) return 0;
    for (std::uint8_t d = 1; d < domain_count; ++d)
        if (!std::strcmp(name, compat::domain_names[d])) return d;
    return 0;
}

enum class Verdict : std::uint8_t { unknown, veto, clear };

struct Manifest {
    ActorKey key;
    std::uint64_t epoch = 0;
    std::uint32_t token = 0;
    bool sealed = false;
    std::vector<std::uint32_t> evaluators;         // the required compatibility evaluator ids, unique
    std::vector<const void*> bound;                // the evaluator objects at seal: a replacement invalidates the pass
    struct Fact { bool known = false, value = false; } facts[domain_count];
    bool native_story = false;                     // the engine's own story knowledge (ALife story id), ORed into story
    std::uint32_t mutation = 0;                    // every accepted fact / invalidate / seal bumps it
    // the last complete pass of exactly the sealed set under this token
    std::uint32_t pass_revision = 0;
    bool pass_valid = false, pass_veto = false;
    std::uint32_t pass_mutation = 0;
    std::uint64_t pass_perception = 0;
    TimeMs pass_at = 0;
};

struct Pass {                                      // one walk of a man's planner evaluators
    Manifest* manifest = nullptr;
    std::uint32_t token = 0;                       // the registration the walk began under: an evaluator's script code may
    std::uint32_t seen = 0;                        // re-register him mid-walk, which aborts the walk (the new token rules)
    bool veto = false, replaced = false, aborted = false;
};

class Store {
public:
    // A new registration revision for the man; any earlier manifest of his id is dropped. 0 = refused.
    std::uint32_t begin(std::uint16_t id, ActorKey key, std::uint64_t epoch)
    {
        if (key.id == invalid_id || key.id != id || !epoch) return 0;
        Manifest m;
        m.key = key;
        m.epoch = epoch;
        m.token = next_token_++;
        if (!next_token_) next_token_ = 1;
        manifests_[id] = m;
        return m.token;
    }

    // The writers' call. False: stale token / identity, or (evaluator) already sealed - a new set is a new begin.
    bool evaluator(std::uint16_t id, ActorKey key, std::uint64_t epoch, std::uint32_t token, std::uint32_t evaluator_id)
    {
        Manifest* m = by_token(id, key, epoch, token);
        if (!m || m->sealed) return false;
        if (std::find(m->evaluators.begin(), m->evaluators.end(), evaluator_id) == m->evaluators.end())
            m->evaluators.push_back(evaluator_id);
        return true;
    }
    bool fact(std::uint16_t id, ActorKey key, std::uint64_t epoch, std::uint32_t token, std::uint8_t domain, bool value)
    {
        Manifest* m = by_token(id, key, epoch, token);
        if (!m || !domain || domain >= domain_count) return false;
        m->facts[domain].known = true;
        m->facts[domain].value = value;
        ++m->mutation;
        return true;
    }
    // Commits the manifest when every registered evaluator resolves (resolve(id) -> the evaluator object or nullptr) and
    // every domain is known. Else the manifest stays unsealed (unknown) and false is returned.
    template <class Resolve>
    bool seal(std::uint16_t id, ActorKey key, std::uint64_t epoch, std::uint32_t token, Resolve&& resolve)
    {
        Manifest* m = by_token(id, key, epoch, token);
        if (!m) return false;
        for (std::uint8_t d = 1; d < domain_count; ++d)
            if (!m->facts[d].known) return false;
        std::vector<const void*> bound;
        bound.reserve(m->evaluators.size());
        for (std::uint32_t evaluator_id : m->evaluators)
        {
            const void* object = resolve(evaluator_id);
            if (!object) return false;
            bound.push_back(object);
        }
        m->bound = bound;
        m->sealed = true;
        m->pass_valid = false;                     // a new set: no earlier pass speaks for it
        ++m->mutation;
        return true;
    }
    // A writer is about to change a source: that domain (0 = everything, the evaluator set included) is unknown until it
    // is supplied again (a domain) or sealed again (0). Pending passes are dropped. invalidate(0) keeps the token: the
    // registrar may add evaluators and re-seal under it (a stale evaluator then fails the seal - unknown, the safe side), or
    // begin() a new token for a fresh set; either way no pass of the old set speaks for the new one.
    bool invalidate(std::uint16_t id, ActorKey key, std::uint64_t epoch, std::uint32_t token, std::uint8_t domain)
    {
        Manifest* m = by_token(id, key, epoch, token);
        if (!m || domain >= domain_count) return false;
        if (domain) m->facts[domain].known = false;
        else m->sealed = false;
        m->pass_valid = false;
        ++m->mutation;
        return true;
    }
    std::uint32_t token_of(std::uint16_t id, ActorKey key, std::uint64_t epoch) const
    {
        const Manifest* m = current(id, key, epoch);
        return m ? m->token : 0;
    }
    // The engine's own writers (no token: they know the man's identity): companion info portions, the story id.
    void native_fact(std::uint16_t id, std::uint8_t domain, bool value)
    {
        const auto found = manifests_.find(id);
        if (found == manifests_.end() || !domain || domain >= domain_count) return;
        found->second.facts[domain].known = true;
        found->second.facts[domain].value = value;
        ++found->second.mutation;
    }
    void native_story(std::uint16_t id, bool story)
    {
        const auto found = manifests_.find(id);
        if (found != manifests_.end()) found->second.native_story = story;
    }

    // The forced pass / the admission validation walk the man's planner once; the full-pass fence is end().
    Pass pass_begin(std::uint16_t id, ActorKey key, std::uint64_t epoch)
    {
        Pass pass;
        Manifest* m = current_mutable(id, key, epoch);
        if (m && m->sealed)
        {
            pass.manifest = m;
            pass.token = m->token;
        }
        return pass;
    }
    static void pass_record(Pass& pass, std::uint32_t evaluator_id, const void* object, bool value)
    {
        if (!pass.manifest || pass.aborted) return;
        if (pass.manifest->token != pass.token || !pass.manifest->sealed)
        {
            pass.aborted = true;                   // re-registered or invalidated under the walk
            return;
        }
        const auto& ids = pass.manifest->evaluators;
        const auto found = std::find(ids.begin(), ids.end(), evaluator_id);
        if (found == ids.end()) return;
        ++pass.seen;
        const std::size_t at = std::size_t(found - ids.begin());
        if (at >= pass.manifest->bound.size() || pass.manifest->bound[at] != object) pass.replaced = true;
        if (value) pass.veto = true;
    }
    static void pass_end(Pass& pass, std::uint64_t perception, TimeMs now)
    {
        Manifest* m = pass.manifest;
        if (!m || pass.aborted || m->token != pass.token || !m->sealed) return;   // the new registration rules
        ++m->pass_revision;
        m->pass_at = now;
        m->pass_mutation = m->mutation;
        m->pass_perception = perception;
        if (pass.replaced || pass.seen != m->evaluators.size())
        {
            m->sealed = false;                     // a missing / replaced evaluator: readiness is gone until re-registered
            m->pass_valid = false;
            ++m->mutation;
            return;
        }
        m->pass_valid = true;
        m->pass_veto = pass.veto;
    }
    // Whether an admission demand needs the one validation walk: no complete pass of the sealed set yet, or a mutation or
    // a perception change since the last one (the wake events); never by time. Only when the evaluators can decide: a
    // known veto fact (or the ALife story id) already vetoes and an unknown veto domain already makes him unknown, so then
    // no walk runs at all (N7-1: a companion under fire walks nothing; a man in a meet walks once per perception change).
    bool needs_validation(std::uint16_t id, ActorKey key, std::uint64_t epoch, std::uint64_t perception) const
    {
        const Manifest* m = current(id, key, epoch);
        if (!m || !m->sealed || m->native_story) return false;
        for (std::uint8_t d = 1; d < domain_count; ++d)
            if (veto_domain(d) && (!m->facts[d].known || m->facts[d].value)) return false;
        return !m->pass_valid || m->pass_mutation != m->mutation || m->pass_perception != perception;
    }

    Verdict verdict(std::uint16_t id, ActorKey key, std::uint64_t epoch) const
    {
        const Manifest* m = current(id, key, epoch);
        if (!m) return Verdict::unknown;
        if (m->native_story) return Verdict::veto;
        bool unknown = !m->sealed;
        for (std::uint8_t d = 1; d < domain_count; ++d)
        {
            if (!veto_domain(d)) continue;
            if (!m->facts[d].known) unknown = true;
            else if (m->facts[d].value) return Verdict::veto;
        }
        if (unknown || !m->pass_valid) return Verdict::unknown;
        return m->pass_veto ? Verdict::veto : Verdict::clear;
    }
    CompatState state(std::uint16_t id, ActorKey key, std::uint64_t epoch) const
    {
        CompatState s;
        const Manifest* m = current(id, key, epoch);
        if (!m) return s;
        s.token = m->token;
        s.pass_revision = m->pass_valid ? m->pass_revision : 0;
        s.sealed = m->sealed ? Evidence::yes : Evidence::no;
        const Verdict v = verdict(id, key, epoch);
        s.veto = v == Verdict::veto ? Evidence::yes : v == Verdict::clear ? Evidence::no : Evidence::unknown;
        s.at_ms = m->pass_at;
        return s;
    }
    const Manifest* current(std::uint16_t id, ActorKey key, std::uint64_t epoch) const
    {
        const auto found = manifests_.find(id);
        if (found == manifests_.end()) return nullptr;
        const Manifest& m = found->second;
        return m.key == key && m.epoch == epoch ? &m : nullptr;
    }
    template <class Visit> void for_each(Visit&& visit) const
    {
        for (const auto& entry : manifests_) visit(entry.second);
    }
    void forget(std::uint16_t id) { manifests_.erase(id); }
    void reset() { manifests_.clear(); }
    std::size_t size() const { return manifests_.size(); }

private:
    Manifest* current_mutable(std::uint16_t id, ActorKey key, std::uint64_t epoch)
    {
        return const_cast<Manifest*>(current(id, key, epoch));
    }
    Manifest* by_token(std::uint16_t id, ActorKey key, std::uint64_t epoch, std::uint32_t token)
    {
        Manifest* m = current_mutable(id, key, epoch);
        return m && token && m->token == token ? m : nullptr;
    }
    std::unordered_map<std::uint16_t, Manifest> manifests_;
    std::uint32_t next_token_ = 1;
};

}}} // namespace painter::combat::compatmodel

#ifdef _MSC_VER
#pragma warning(pop)
#endif
