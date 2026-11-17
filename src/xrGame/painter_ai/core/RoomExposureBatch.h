#pragma once
#include "RoomExposureCost.h"
#include "CombatKnowledge.h"
#include "CombatOutdoorSensors.h"
#include "ExposureGeometryBuilder.h"
#include <memory>
#include <set>

namespace painter { namespace combat { namespace core {
// RoomExposureDispatchPolicy is allocated in ../OutdoorConfig.h.
struct RoomExposureBatchStats {
    std::size_t reserved_bytes=0,stores=0,work=0,queries=0,evictions=0,skipped=0;
    std::uint64_t token=0;
};
// One serialized store shared across disposable Works. An active token excludes
// another writer without blocking it. Work destruction rolls back observations;
// static line progress survives. This does not own any native object or queue.
class RoomExposureBatch {
public:
    explicit RoomExposureBatch(RoomExposureDispatchPolicy);
    ~RoomExposureBatch();
    std::shared_ptr<const ExposureGeometryInput> prepare_geometry(const WorldSnapshot&,const EngineServices&,
        ExposureWorkQuota&,const std::function<bool()>& cancelled={});
    std::uint64_t begin(const WorldSnapshot&,const KnowledgeFrame&,
        const std::shared_ptr<const ExposureGeometryInput>&,const std::vector<OutdoorCapturedEye>&,
        const EngineServices&,const std::function<bool()>& cancelled={},const std::set<std::uint32_t>& reset_sides={});
    void advance(std::uint64_t,const EngineServices&,const std::function<bool()>& cancelled={});
    void advance(std::uint64_t,const EngineServices&,ExposureWorkQuota&,const std::function<bool()>& cancelled={});
    bool view(std::uint64_t,std::uint32_t side,RoomExposureCostView&) const;
    bool finish(std::uint64_t,bool commit);
    // Factual identity resets must survive a discarded tactical transaction.
    // An active store is retired on finish, after its last borrower is done.
    void invalidate_side(std::uint32_t);
    RoomExposureBatchStats stats()const;
private:
    struct Impl;std::unique_ptr<Impl> impl_;
};
}}}
