#include "RoomLaneCost.h"
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace painter { namespace combat { namespace core {
namespace {
struct Cancelled {};
void poll(const std::function<bool()>& f){if(f&&f())throw Cancelled{};}
bool finite(Vec3 p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
bool key_valid(ActorKey k){return k.id<invalid_id&&k.generation!=0;}
bool equal(Vec3 a,Vec3 b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
float dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
Vec3 delta(Vec3 a,Vec3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
}
RoomLaneCost::RoomLaneCost(Config c):config_(c){}
void RoomLaneCost::reset(){*this=RoomLaneCost(config_);}
RoomLaneResult RoomLaneCost::update(const WorldSnapshot& w,const SideSnapshot& side,const RoomLaneRequest& request,
    const BuildingCache& cache,const std::function<bool()>& stop){
    RoomLaneCost staged=*this;
    try {poll(stop);auto out=staged.update_impl(w,side,request,cache,stop);poll(stop);
        if(out.accepted)*this=std::move(staged);return out;
    }catch(const Cancelled&){RoomLaneResult out;out.cancelled=true;out.status=RoomLaneStatus::cancelled;return out;}
}
RoomLaneResult RoomLaneCost::update_impl(const WorldSnapshot& w,const SideSnapshot& side,const RoomLaneRequest& request,
    const BuildingCache& cache,const std::function<bool()>& stop){
    RoomLaneResult out;out.level_epoch=w.level_epoch;out.snapshot_seq=w.sequence;out.map_hash=w.map_hash;
    out.geometry_revision=w.geometry_revision;out.observed_at_ms=w.now_ms;out.attempt=request.attempt;
    out.side=side.side_id;out.mission_revision=side.mission.revision;
    const auto& c=config_.room;
    if(!w.level_epoch||!request.attempt||request.building==invalid_room||request.target_room==invalid_room||
        !std::isfinite(c.casualty_lane_block_cost)||c.casualty_lane_block_cost<0||
        !std::isfinite(c.casualty_lane_width_margin_m)||c.casualty_lane_width_margin_m<0||
        !std::isfinite(c.casualty_lane_depth_margin_m)||c.casualty_lane_depth_margin_m<0||
        !std::isfinite(c.casualty_lane_vertical_m)||c.casualty_lane_vertical_m<0)return out;
    if(!w.sequence||(initialized_&&(w.level_epoch<epoch_||(w.level_epoch==epoch_&&
        (w.sequence<sequence_||w.now_ms<now_||(w.sequence==sequence_&&w.now_ms!=now_)))))){
        out.status=RoomLaneStatus::older_snapshot;return out;}
    if(cache.level_epoch!=w.level_epoch||cache.map_hash!=w.map_hash){out.status=RoomLaneStatus::invalid_geometry;return out;}
    const Building* building=nullptr;
    for(const auto& b:cache.buildings){poll(stop);if(b.id==request.building){if(building)return out;building=&b;}}
    if(!building){out.status=RoomLaneStatus::invalid_geometry;return out;}
    bool room_found=false;for(const auto& r:building->rooms){poll(stop);if(r.id==request.target_room)room_found=true;}
    if(!room_found){out.status=RoomLaneStatus::invalid_geometry;return out;}
    std::unordered_map<std::uint32_t,const Portal*> portals;
    for(const auto& p:building->portals){poll(stop);if(!portals.emplace(p.id,&p).second)return out;}
    auto requested=request.portals;std::sort(requested.begin(),requested.end());
    if(std::adjacent_find(requested.begin(),requested.end())!=requested.end())return out;
    std::vector<Lane> frames;frames.reserve(requested.size());
    for(auto id:requested){poll(stop);const auto found=portals.find(id);
        if(found==portals.end()){out.status=RoomLaneStatus::invalid_geometry;return out;}const auto& p=*found->second;
        const float len=std::hypot(p.normal.x,p.normal.z);
        if(!p.doorway||!p.traversable||(p.room_a!=request.target_room&&p.room_b!=request.target_room)||
            !finite(p.centre)||!finite(p.normal)||!std::isfinite(len)||len<=0||
            !std::isfinite(p.half_width)||p.half_width<=0||!std::isfinite(p.wall_depth)||p.wall_depth<0||
            !std::isfinite(p.y0)||!std::isfinite(p.y1)||p.y1<p.y0){out.status=RoomLaneStatus::invalid_geometry;return out;}
        Lane lane;lane.portal=id;lane.centre=p.centre;lane.normal={p.normal.x/len,0,p.normal.z/len};
        lane.tangent={lane.normal.z,0,-lane.normal.x};lane.half_width=p.half_width;lane.depth=p.wall_depth;
        lane.floor=p.y1>p.y0?p.y0:p.centre.y;frames.push_back(lane);
    }
    const bool context_changed=!initialized_||w.level_epoch!=epoch_||w.map_hash!=map_||w.geometry_revision!=geometry_||
        side.side_id!=side_||side.faction!=faction_||side.group!=group_||side.mission.revision!=mission_||
        side.mission.kind!=mission_kind_||request.attempt!=attempt_||request.building!=building_||request.target_room!=room_;
    if(context_changed)lanes_.clear();
    // Keep retired portal clocks until the attempt ends: removing/re-adding a
    // selected door must not rearm its capped cost. Size is bounded by building portals.
    for(const auto& f:frames){poll(stop);auto it=std::find_if(lanes_.begin(),lanes_.end(),[&](const Lane& l){return l.portal==f.portal;});
        if(it==lanes_.end()){lanes_.push_back(f);continue;}
        if(!equal(it->centre,f.centre)||!equal(it->normal,f.normal)||it->half_width!=f.half_width||it->depth!=f.depth||it->floor!=f.floor){
            out.status=RoomLaneStatus::invalid_geometry;return out;}
    }
    std::unordered_map<std::uint32_t,const ActorSnapshot*> actors;
    for(const auto& a:w.actors){poll(stop);if(!actors.emplace(a.key.id,&a).second)return out;}
    std::unordered_set<std::uint32_t> member_ids;
    std::vector<RoomLaneBody> bodies;
    for(auto key:side.members){poll(stop);if(!key_valid(key)||!member_ids.insert(key.id).second)return out;
        const auto it=actors.find(key.id);if(it==actors.end()){++out.unknown_members;continue;}const auto& a=*it->second;
        if(a.key!=key||a.level_epoch!=w.level_epoch||a.snapshot_seq!=w.sequence||a.observed_at_ms>w.now_ms||
            w.now_ms-a.observed_at_ms>config_.snapshot_max_age_ms||!finite(a.position)){++out.unknown_members;continue;}
        ++out.validated_members;if(a.alive&&!a.wounded)continue;
        bodies.push_back({a.key,a.ownership_generation,a.restrictions_revision,a.observed_at_ms,a.position,!a.alive,a.wounded});
    }
    for(const auto& f:frames){poll(stop);auto it=std::find_if(lanes_.begin(),lanes_.end(),[&](const Lane& l){return l.portal==f.portal;});
        RoomLaneReceipt receipt;receipt.portal=f.portal;
        for(const auto& body:bodies){poll(stop);++out.body_portal_tests;const auto d=delta(body.position,f.centre);
            if(std::abs(body.position.y-f.floor)<=c.casualty_lane_vertical_m&&
                std::abs(dot(d,f.tangent))<=f.half_width+c.casualty_lane_width_margin_m&&
                std::abs(dot(d,f.normal))<=f.depth+c.casualty_lane_depth_margin_m)receipt.bodies.push_back(body);
        }
        if(!receipt.bodies.empty()&&!it->started){it->started=true;it->started_ms=w.now_ms;}
        receipt.started=it->started;receipt.started_ms=it->started_ms;
        if(it->started){receipt.elapsed_ms=w.now_ms-it->started_ms;receipt.capped=receipt.elapsed_ms>=c.casualty_lane_block_cap_ms;
            receipt.remaining_ms=receipt.capped?0:c.casualty_lane_block_cap_ms-receipt.elapsed_ms;
            if(!receipt.bodies.empty()&&!receipt.capped)receipt.cost=c.casualty_lane_block_cost;}
        out.portals.push_back(std::move(receipt));
    }
    initialized_=true;epoch_=w.level_epoch;map_=w.map_hash;geometry_=w.geometry_revision;sequence_=w.sequence;now_=w.now_ms;
    attempt_=request.attempt;side_=side.side_id;faction_=side.faction;group_=side.group;mission_=side.mission.revision;
    mission_kind_=side.mission.kind;building_=request.building;room_=request.target_room;
    out.accepted=true;out.status=RoomLaneStatus::ready;return out;
}
}}}
