#pragma once
// painter_ai/EngineCompat.h - the compatibility push model on the engine side (contract v2.21, CompatModel.h): the
// stalker planner's forced pass with its full-pass fence, the six level.painter_combat_compat_* exports, the native
// writers (companion info portions, the ALife story id), the one validation walk on an admission demand, and the verdict
// the adapter gates admission on.
// Threads: the planner pass (the scheduler), script calls, info-portion transfers and the authority tick never overlap
// (the same serialization as EngineNearMiss.h), so no lock. Workers never see any of it: capture copies CompatState.
#include "CombatContract.h"
#include "CompatModel.h"

class CAI_Stalker;

namespace painter { namespace combat { namespace engine {

// level.painter_combat_compat_* (level_script.cpp resolves the npc; a non-stalker gets 0 / false)
std::uint32_t compat_begin(CAI_Stalker& actor);
bool compat_evaluator(CAI_Stalker& actor, std::uint32_t token, std::uint32_t evaluator_id);
bool compat_fact(CAI_Stalker& actor, std::uint32_t token, const char* domain, bool value);
bool compat_seal(CAI_Stalker& actor, std::uint32_t token);
bool compat_invalidate(CAI_Stalker& actor, std::uint32_t token, const char* domain);   // domain nullptr = everything
std::uint32_t compat_token(CAI_Stalker& actor);

// CStalkerPlanner::painter_forced_update: begin before the evaluator loop, record every evaluator's result as the loop
// produces it (no second evaluate()), end after the loop - the fence. end() returns true when the man was handed back
// (a veto, or unknown in push mode) through the adapter's relinquish (cleanup -> on_release, exactly once).
compatmodel::Pass compat_pass_begin(CAI_Stalker& actor);
void compat_pass_record(compatmodel::Pass& pass, std::uint32_t evaluator_id, const void* evaluator, bool value);
bool compat_pass_end(compatmodel::Pass& pass, CAI_Stalker& actor);

// The adapter. compat_blocker: the yield of his verdict (veto -> yield_scheme_block; unknown -> yield_compat_unknown in push
// mode only). compat_admission_demand: before an admission in push mode, the one validation walk of his registered
// evaluators when the sealed set has no complete pass yet or a mutation / perception change woke it.
std::uint32_t compat_blocker(CAI_Stalker& actor, ActorKey key, bool push_mode);
void compat_admission_demand(CAI_Stalker& actor, ActorKey key, std::uint64_t perception);
CompatState compat_state(std::uint16_t id, ActorKey key);

// Native writers and lifecycle
void compat_info_changed(std::uint16_t id, const char* info, bool present);   // CInventoryOwner info transfers
void compat_forget(std::uint16_t id);
void compat_reset();

// Diagnostics, so a missing writer shows as a number, not as silence: the adapter reports every admission demand (both
// modes - in bridge mode an unknown man is counted as a shadow refusal); push mode logs phase=compat_unknown with his
// domain state once per state change; phase=compat_summary at level end (counts per level).
void compat_note_admission(CAI_Stalker& actor, ActorKey key, bool push_mode);
void compat_summary(const char* level);

}}} // namespace painter::combat::engine
