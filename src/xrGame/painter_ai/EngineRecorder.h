#pragma once
// native17 (2026-10-04): the live snapshot recorder and the publication reject counters. Engine-owned; implemented in
// EngineServices.cpp (engine TU). This header is pure (contract types + std) so the PCH-less runtime bridge can call it.
//
// Recorder: every runtime tick written as one JSON line, format "native-replay-v1": record 0 = header {format, level,
// build, contract, config}; then per tick {t_ms, seq, snapshot: the WorldSnapshot field for field by its contract names
// (Bounded<> as arrays, enums as ints, Vec3 as [x,y,z]), decisions: every Decision passed to publish with its
// PublishResult, handbacks: the adapter's hand-backs since the previous tick, compat: the admission verdicts since the
// previous tick}. Switched by [painter_combat] replay_record (0/1), bounded by replay_record_max_mb (stops at the cap,
// logged once). File: $logs$/painter_replay_<level>_<yyyymmdd_hhmmss>.jsonl, written on the authority thread.
#include "CombatContract.h"
#include "EngineCombatAdapter.h"
#include <cstdint>
#include <string>

namespace painter { namespace combat { namespace engine {
void recorder_configure(bool on, std::uint32_t max_mb);
void recorder_snapshot(const WorldSnapshot& snapshot);                 // before the dispatcher takes it
void recorder_publish(const Decision& decision, const PublishResult& result);
void recorder_handback(std::uint16_t actor, std::uint64_t generation, std::uint64_t ownership, std::uint64_t order,
                       const char* reason, std::uint32_t yield_bits, TimeMs at);
void recorder_compat(std::uint16_t actor, std::uint8_t verdict);        // 0 unknown, 1 clear, 2 veto
void recorder_tick_end(TimeMs now);                                     // closes the tick's record
void recorder_level_end();                                              // flushes and closes the file
float recorder_max_ms();                                                // the longest tick spent writing
std::uint64_t recorder_bytes();

// Publication rejects by RejectReason (0..12) and whole-decision envelopes, per level; a once-per-reason sample line.
void note_publish_rejects(const Decision& decision, const PublishResult& result);
const char* reject_reason_name(std::uint8_t reason);
std::string reject_summary();                                           // "rejects=epoch:N,... envelopes=N"
void reset_reject_counters();
}}}
