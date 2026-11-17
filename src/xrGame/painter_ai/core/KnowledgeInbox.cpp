#include "KnowledgeInbox.h"
#include <algorithm>

namespace painter { namespace combat { namespace core {
KnowledgeInbox::KnowledgeInbox(std::size_t capacity):capacity_((std::max)(std::size_t(1),capacity)){}
bool KnowledgeInbox::capture(const WorldSnapshot& world,const KnowledgeContext& context,const LifecycleSignals& signals,const FriendlyCasualtyEvidence& casualties) {
    if(!world.level_epoch||!world.sequence||world.level_epoch<epoch_ ||
        (world.level_epoch==epoch_&&(world.sequence<=last_sequence_||world.now_ms<last_time_)))return false;
    if(world.level_epoch!=epoch_)reset();
    auto tick=std::make_shared<KnowledgeInputTick>();tick->world=world;tick->context=context;tick->lifecycle_signals=signals;tick->friendly_casualties=casualties;
    ticks_.push_back(std::move(tick));epoch_=world.level_epoch;last_sequence_=world.sequence;last_time_=world.now_ms;
    ++stats_.captured_ticks;
    while(ticks_.size()>capacity_) {
        ++stats_.evicted_ticks;
        stats_.evicted_friendly_casualties+=ticks_.front()->friendly_casualties.facts.size();
        for(const auto& actor:ticks_.front()->world.actors) {
            stats_.evicted_hits+=actor.hits.count;stats_.evicted_heard+=actor.heard.count;
            stats_.evicted_seen_dead+=actor.seen_dead.count;
            stats_.evicted_near_misses+=actor.near_misses.count;
        }
        ticks_.pop_front();
    }
    stats_.pending_ticks=ticks_.size();return true;
}
std::vector<KnowledgeInbox::Tick> KnowledgeInbox::pending_through(std::uint64_t sequence) const {
    std::vector<Tick> result;result.reserve(ticks_.size());
    for(const auto& tick:ticks_)if(tick->world.sequence<=sequence)result.push_back(tick);
    return result;
}
void KnowledgeInbox::acknowledge(std::uint64_t epoch,std::uint64_t sequence) {
    if(epoch!=epoch_)return;
    while(!ticks_.empty()&&ticks_.front()->world.sequence<=sequence) {
        ticks_.pop_front();++stats_.acknowledged_ticks;
    }
    stats_.pending_ticks=ticks_.size();
}
void KnowledgeInbox::reset(){epoch_=last_sequence_=last_time_=0;ticks_.clear();stats_={};}
}}}
