#pragma once
#include "CombatOutdoorExecution.h"
namespace painter { namespace combat { namespace core {
struct OutdoorCapturedEye {
    ActorKey actor;std::uint64_t epoch=0,sequence=0,owner=0,restrictions=0;
    TimeMs observed_at_ms=0;Vec3 feet,eye;
    // True only for an actual current engine eye_matrix, never a fallback height.
    bool measured=false;
};
struct OutdoorSensorQuota {std::size_t queries_left=0,queries_used=0;};
// Authority thread only; callback must include static AND dynamic occluders.
// Select only a received original K1 point matching an actually applied search.
OutdoorSearchObservation measure_outdoor_search_observation(const Config&,const WorldSnapshot&,
    const ActorSnapshot&,const OutdoorCapturedEye&,const Order&,const MemberKnowledge&,
    OutdoorSensorQuota&,const std::function<Evidence(const Segment&)>& dynamic_ray);
// One actual captured eye -> known body-height point static ray on the worker.
// Preserve the original source position; height is a configured ray endpoint offset. Dynamic
// teammate/weapon gates remain native; this is neither FOV nor a shot receipt.
// Charge before calling; cancellation/failure never refunds a native query.
OutdoorMeasuredLine measure_outdoor_line(const WorldSnapshot&,const ActorSnapshot&,
    const OutdoorCapturedEye&,const KnowledgeTrace&,const EngineServices&,
    OutdoorSensorQuota&,const std::function<bool()>& cancelled={});
OutdoorMeasuredLine measure_outdoor_line(const WorldSnapshot&,const ActorSnapshot&,
    const OutdoorCapturedEye&,const KnowledgeTrace&,const EngineServices&,
    OutdoorSensorQuota&,float target_height_m,const std::function<bool()>& cancelled={});
}}}
