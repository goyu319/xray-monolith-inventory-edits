#pragma once
#include "CombatKnowledge.h"
#include "CombatSideLifecycle.h"
#include "FriendlyCasualty.h"
#include <deque>

namespace painter { namespace combat { namespace core {
struct KnowledgeInputTick {
    WorldSnapshot world;
    KnowledgeContext context;
    LifecycleSignals lifecycle_signals;
    FriendlyCasualtyEvidence friendly_casualties; // immutable same-capture observations, never restamped
};
struct KnowledgeInboxStats {
    std::uint64_t captured_ticks=0,acknowledged_ticks=0,evicted_ticks=0;
    // Raw bounded sensor rows, not distinct events. An evicted tick can still
    // be retained by an in-flight immutable batch; eviction is not proof of loss.
    std::uint64_t evicted_hits=0,evicted_heard=0,evicted_seen_dead=0,evicted_near_misses=0;
    std::size_t pending_ticks=0;
    std::uint64_t evicted_friendly_casualties=0;
};
// Capture/publish-thread journal. The capacity bounds HISTORY, never actors.
// Workers receive shared const values, so capture, cancellation and eviction
// cannot mutate observations already being processed. No timestamp is restamped.
class KnowledgeInbox {
public:
    using Tick=std::shared_ptr<const KnowledgeInputTick>;
    explicit KnowledgeInbox(std::size_t capacity=64);
    bool capture(const WorldSnapshot&,const KnowledgeContext& context={},const LifecycleSignals& lifecycle_signals={},
        const FriendlyCasualtyEvidence& friendly_casualties={});
    std::vector<Tick> pending_through(std::uint64_t sequence) const;
    void acknowledge(std::uint64_t epoch,std::uint64_t sequence);
    void reset();
    const KnowledgeInboxStats& stats() const {return stats_;}
private:
    std::size_t capacity_;
    std::uint64_t epoch_=0,last_sequence_=0;
    TimeMs last_time_=0;
    std::deque<Tick> ticks_;
    KnowledgeInboxStats stats_;
};
}}}
