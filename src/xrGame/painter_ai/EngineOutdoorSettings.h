#pragma once
// Runtime objects are derived solely from canonical Config data. No reader,
// second master, mutable settings singleton, or alternate settings section.
#include "core/CombatOutdoorDirector.h"
#include "core/CombatSideLifecycle.h"
#include "core/RoomExposureBatch.h"
namespace painter { namespace combat { namespace engine {
struct OutdoorRuntimePolicies {
    std::shared_ptr<const core::OutdoorDispatchPolicy> policy;
    std::shared_ptr<const core::SideLifecyclePolicy> lifecycle;
    std::shared_ptr<const core::RoomExposureDispatchPolicy> exposure;
};
inline OutdoorRuntimePolicies make_outdoor_policies(const OutdoorConfig& c) {
    OutdoorRuntimePolicies out;
    out.policy=std::make_shared<const core::OutdoorDispatchPolicy>(c.p);
    const auto& v=c.r.lifecycle_values;
    out.lifecycle=std::make_shared<const core::SideLifecyclePolicy>(v.quiet_field_ms,
        v.quiet_building_ms,v.stand_down_after_ms,v.side_cooldown_ms,v.error_cooldown_ms,
        v.retain_authored_mission_without_evidence);
    if(c.r.exposure_enabled) {
        auto e=c.e; e.geometry=std::make_shared<const core::ExposureGeometryPolicy>(c.g);
        out.exposure=std::make_shared<const core::RoomExposureDispatchPolicy>(std::move(e));
    }
    return out;
}
}}}
