#pragma once
#include "core/FriendlyCasualty.h"
namespace painter { namespace combat { namespace engine {
// Serialized authority seam only. Visits the captured observer's visual memory,
// checks visible_now before reading the patient. No global entity enumeration.
// work_limit counts observer and visual-entry visits; the fair cursor persists across captures.
void capture_friendly_casualties(const WorldSnapshot&,const std::vector<SideSnapshot>& known,
    core::FriendlyCasualtyEvidence&,std::size_t work_limit);
}}}
