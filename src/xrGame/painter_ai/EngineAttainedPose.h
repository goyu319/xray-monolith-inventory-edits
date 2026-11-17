#pragma once
#include <cmath>
#include <cstdint>
namespace painter { namespace combat { namespace engine {
// Values read from the actual torso/legs blend channels, never movement target.
struct AppliedPoseChannel {
    bool actual=false,matching_motion=false,accruing=false;
    float amount=0,power=0;
};
inline bool attained_animation_pose(bool played,std::uint32_t played_frame,std::uint32_t current_frame,
    AppliedPoseChannel torso,AppliedPoseChannel legs){
    const auto full=[](AppliedPoseChannel c){return c.actual&&c.matching_motion&&c.accruing&&
        std::isfinite(c.amount)&&std::isfinite(c.power)&&c.power>0&&c.amount>=c.power;};
    // The authority capture can precede UpdateCL in CLevel::OnFrame. The
    // immediately preceding played frame is still the actual rendered pose
    // when BOTH current channels remain the same fully accrued motions.
    // Do not substitute the requested body state or accept older pose history.
    const bool recent=played_frame==current_frame||
        (current_frame>0&&played_frame==current_frame-1);
    return played&&recent&&full(torso)&&full(legs);
}
}}}
