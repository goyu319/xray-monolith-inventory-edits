#pragma once
#include "RoomExposureCache.h"
#include <array>
namespace painter { namespace combat { namespace core {
enum class RoomExposureRole : std::uint8_t { stack,doctrine,defense };
// RoomExposurePrice is allocated in ../OutdoorConfig.h.
struct RoomExposureCostView {
    RoomExposureCache* cache=nullptr;
    ExposureCapture capture;
    std::uint64_t transaction_token=0,pricing_revision=0,observation_revision=0;
    std::array<RoomExposurePrice,3> prices{};
};
struct RoomExposureCostReceipt {
    bool used=false,enqueued=false;
    float prior=0,cost=0;
    std::size_t red_sources=0,total_sources=0;
    std::uint64_t transaction_token=0,pricing_revision=0,observation_revision=0;
};
struct RoomExposureCostStats {std::size_t quotes=0,used=0,fallbacks=0,enqueued=0,held=0;};
// One update only: no engine queries, native pointers or retained callbacks.
// Only the explicit COMPLETE raw unique-source union replaces the Lit prior.
// Door/path/protection costs remain independently owned by their consumers.
class RoomExposureCost {
public:
    RoomExposureCost(const WorldSnapshot&,const SideSnapshot&,const BuildingCache*,const RoomExposureCostView*,
        const std::function<bool()>& cancelled={});
    RoomExposureCostReceipt price(RoomExposureRole,std::uint32_t building,std::uint32_t room,
        std::uint32_t node,Vec3 floor_position,float prior,bool held=false);
    const RoomExposureCostStats& stats()const{return stats_;}
    bool was_cancelled()const{return stopped_;}
private:
    const RoomExposureCostView* view_=nullptr;
    std::function<bool()> cancelled_;
    RoomExposureCostStats stats_;
    bool valid_=false,stopped_=false;
    bool poll();
};
}}}
