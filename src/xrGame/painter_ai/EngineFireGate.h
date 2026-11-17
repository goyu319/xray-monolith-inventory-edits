#pragma once
#include "EngineExecutionContract.h"
namespace painter { namespace combat { namespace engine {
struct FireChannels {bool target=false,point=false;};
inline FireChannels fire_channels(const Order& order,TimeMs now,ExecutionPhase phase,
    Pace actual_pace,bool hands_free,bool live_visible_enemy) {
    const bool aimed=hands_free&&actual_pace!=Pace::run;
    return {aimed&&phase!=ExecutionPhase::release&&live_visible_enemy,
        aimed&&firing_window(order,now,phase)};
}
}}}
