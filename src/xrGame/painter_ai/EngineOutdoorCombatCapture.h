#pragma once
#include "core/CombatOutdoorDirector.h"
namespace painter { namespace combat { namespace engine {
// Authority-thread only. Explicit work counts native primitives (a fire cone,
// a ray, a trajectory query, or a spatial friend query), not worker slices or
// elapsed milliseconds. No allocation/default enables this entry point.
void capture_outdoor_combat(const WorldSnapshot&,core::OutdoorCaptureEvidence&,
    const core::OutdoorDispatchPolicy&,std::size_t authority_query_work_per_frame,const core::KnowledgeFrame* known=nullptr);
}}}
