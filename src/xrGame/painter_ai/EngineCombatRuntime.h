#pragma once
#include "CombatContract.h"
#include "core/CombatDispatcher.h"
namespace painter { namespace combat { namespace engine {
struct RuntimeStatus {
    std::uint64_t epoch=0,frames=0,failures=0;
    float last_tick_ms=0,maximum_tick_ms=0;
    // Capture includes compatibility checks. Dispatch includes publication;
    // do not add publish_ms to dispatch_ms when calculating authority cost.
    float capture_ms=0,maximum_capture_ms=0,dispatch_ms=0,maximum_dispatch_ms=0;
    float publish_ms=0,maximum_publish_ms=0;
    std::uint64_t ingest_rejected=0;
    core::DispatchStats dispatch;
};
void initialize_combat_runtime();
// Per-level teardown while actors are valid, before level_destroyed drains the
// queue. Retains the frame callback so the following map can initialize itself.
void level_ending_combat_runtime();
// Authority thread, before engine shutdown. Cancelled workers own their value
// state until they finish; level_destroyed still performs the shared queue drain.
void shutdown_combat_runtime();
RuntimeStatus combat_runtime_status();
}}}
