#include "RoomExposureCost.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
namespace painter { namespace combat { namespace core {
RoomExposureCost::RoomExposureCost(const WorldSnapshot&w,const SideSnapshot&s,const BuildingCache*b,
    const RoomExposureCostView*v,const std::function<bool()>&stop):view_(v),cancelled_(stop){
    if(!v||!v->cache||!v->transaction_token||!v->pricing_revision||!b)return;
    const auto&c=v->capture;const auto&g=c.stamp;
    if(g.level_epoch!=w.level_epoch||g.map_hash!=w.map_hash||g.geometry_revision!=w.geometry_revision||
        !g.source_basis||g.data_revision!=b->data_version||c.side!=s.side_id||c.group!=s.group||c.faction!=s.faction||
        !c.roster_revision||c.snapshot_seq!=w.sequence||c.now_ms!=w.now_ms)return;
    if(!v->cache->matches_capture(c))return;
    const auto state=v->cache->stats();if(!state.transaction_active||state.transaction_failed||state.rolling_back||
        state.transaction_token!=v->transaction_token||state.observation_revision!=v->observation_revision)return;
    std::unordered_map<std::uint32_t,const ActorSnapshot*> actors;
    for(const auto&a:w.actors){if(poll())return;if(!actors.emplace(a.key.id,&a).second)return;}
    std::unordered_map<std::uint32_t,ActorKey> members;
    for(const auto&key:s.members){if(poll())return;if(!members.emplace(key.id,key).second)return;}
    std::unordered_set<std::uint32_t> observed;
    for(const auto&o:c.observers){if(poll())return;const auto ai=actors.find(o.actor.id);const auto mi=members.find(o.actor.id);
        if(!observed.insert(o.actor.id).second||ai==actors.end()||mi==members.end()||mi->second!=o.actor)return;
        const auto&a=*ai->second;if(a.key!=o.actor||a.ownership_generation!=o.owner||a.level_epoch!=w.level_epoch||a.snapshot_seq!=w.sequence||
            a.observed_at_ms!=w.now_ms||o.snapshot_seq!=w.sequence||o.observed_at_ms>w.now_ms||o.faction!=s.faction)return;
    }
    valid_=true;
}
bool RoomExposureCost::poll(){if(!stopped_&&cancelled_&&cancelled_())stopped_=true;return stopped_;}
RoomExposureCostReceipt RoomExposureCost::price(RoomExposureRole role,std::uint32_t building,std::uint32_t room,
    std::uint32_t node,Vec3 point,float prior,bool held){
    RoomExposureCostReceipt r;r.prior=r.cost=prior;if(held)++stats_.held;
    if(!valid_||poll()){++stats_.fallbacks;return r;}
    const auto state=view_->cache->stats();
    if(!state.transaction_active||state.transaction_failed||state.rolling_back||
        state.transaction_token!=view_->transaction_token||state.observation_revision!=view_->observation_revision){++stats_.fallbacks;return r;}
    const auto index=static_cast<std::size_t>(role);if(index>=view_->prices.size()){++stats_.fallbacks;return r;}
    const auto&p=view_->prices[index];if(!p.enabled||!p.source_cap||!std::isfinite(p.per_red_source)||p.per_red_source<0||
        !std::isfinite(p.sample_height_m)||p.sample_height_m<0||!finite(point)||node==invalid_node){++stats_.fallbacks;return r;}
    ExposureDestination d;d.building=building;d.near_room=room;d.node=node;d.point=point;d.point.y+=p.sample_height_m;d.paired_openings=p.paired_openings;
    if(!finite(d.point)){++stats_.fallbacks;return r;}
    ++stats_.quotes;const auto q=view_->cache->quote(d);
    // The column belongs to one exact current speculative observation/pricing
    // generation. An unchanged valid column may have been priced earlier;
    // exact current capture authority and observation revision bind that reuse.
    // Never sum a separate direct quote into the paired union.
    const bool complete=q.complete&&!q.pending&&q.status==ExposureCacheStatus::complete&&q.unknown_sources==0&&
        q.transaction_token==view_->transaction_token&&q.observation_revision==view_->observation_revision&&
        q.source_basis==view_->capture.stamp.source_basis&&q.priced_snapshot_seq<=view_->capture.snapshot_seq&&
        q.priced_at_ms<=view_->capture.now_ms&&(q.seen_sources==0||q.seen_valid_until_ms>=view_->capture.now_ms)&&
        q.red_sources<=q.total_sources&&q.seen_sources<=q.total_sources&&q.blocked_sources<=q.total_sources;
    if(complete){const double cost=double((std::min)(q.red_sources,p.source_cap))*double(p.per_red_source);
        if(std::isfinite(cost)&&cost<=(std::numeric_limits<float>::max)()){
            r.used=true;r.cost=static_cast<float>(cost);r.red_sources=q.red_sources;r.total_sources=q.total_sources;
            r.transaction_token=q.transaction_token;r.pricing_revision=view_->pricing_revision;r.observation_revision=q.observation_revision;++stats_.used;return r;}}
    const auto status=view_->cache->enqueue(d,[this]{return poll();});
    if(status==ExposureCacheStatus::cancelled)stopped_=true;
    r.enqueued=status==ExposureCacheStatus::queued||status==ExposureCacheStatus::pending||status==ExposureCacheStatus::complete;
    stats_.enqueued+=r.enqueued?1u:0u;++stats_.fallbacks;return r;
}
}}}
