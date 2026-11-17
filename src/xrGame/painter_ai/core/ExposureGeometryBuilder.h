#pragma once
#include "RoomExposureCache.h"

namespace painter { namespace combat { namespace core {
// Required private caller allocation. No runtime/default sample policy is
// selected here. Capacities limit this optional cache basis, never NPC admission.
// ExposureGeometryPolicy is allocated in ../OutdoorConfig.h.
struct ExposurePortalOrientation {
    std::uint32_t opening_id=0,building=invalid_room,portal=0,far_room=invalid_room,near_room=invalid_room;
};
enum class ExposureGeometryStatus : std::uint8_t {invalid,pending,cancelled,query_failure,memory_limit,capacity_limit,complete};
enum class ExposureGeometryPhase : std::uint8_t {rooms,room_sort,room_check,portals,opening_sort,opening_copy,grid,source_sort,links,fingerprint,done};
struct ExposureGeometryProgress {
    ExposureGeometryStatus status=ExposureGeometryStatus::invalid;
    ExposureGeometryPhase phase=ExposureGeometryPhase::rooms;
    std::size_t rooms=0,sources=0,openings=0,links=0,charged_bytes=0,work=0,queries=0;
    std::uint64_t grid_candidates=0,duplicate_sources=0;
    bool complete=false;
    bool destination_floor_check_required=true;
};
// Serialized static builder; input BuildingCache must remain immutable under
// its supplied epoch/map/data/geometry key. No engine pointer is retained.
// Each completed static work unit survives cancellation, but no partial basis
// is exposed. Consumed quota is never refunded. The same caller quota can be
// shared across builders and caches. Grid projection/lookup callbacks are all
// charged; no rays, enemy census, raw-nav-node density or fallback heights.
class ExposureGeometryBuilder {
public:
    explicit ExposureGeometryBuilder(ExposureGeometryPolicy);
    ~ExposureGeometryBuilder();
    ExposureGeometryBuilder(const ExposureGeometryBuilder&)=delete;
    ExposureGeometryBuilder& operator=(const ExposureGeometryBuilder&)=delete;
    ExposureGeometryProgress advance(const ExposureGeometryStamp&,const EngineServices&,ExposureWorkQuota&,
        const std::function<bool()>& cancelled={});
    // Borrowed immutable view; invalidated by reset or a different geometry key.
    // Cache.bind copies it. Do not retain the pointer across builder rebinding.
    const ExposureGeometryInput* geometry()const;
    const std::vector<ExposurePortalOrientation>* orientations()const;
    ExposureGeometryProgress state()const;
    void reset();
private:
    struct Impl;std::unique_ptr<Impl> impl_;
    ExposureGeometryPolicy policy_;
};
}}}
