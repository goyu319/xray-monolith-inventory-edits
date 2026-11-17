#pragma once
#include "CombatContract.h"
#include <algorithm>
#include <limits>
namespace painter { namespace combat { namespace engine {
// Internal policy until allocated in the next shared Config revision.
constexpr TimeMs deadline_retry_floor_ms=5000;
constexpr TimeMs deadline_retry_cap_ms=60000;
struct DeadlineReadmission {
    TimeMs until=0;unsigned failures=0;
    bool cooling(TimeMs now)const{return failures&&now<until;}
    TimeMs failed(TimeMs now,TimeMs ordinary_delay){
        failures=(std::min)(failures+1,16u);
        TimeMs delay=(std::min)(deadline_retry_cap_ms,(std::max)(deadline_retry_floor_ms,ordinary_delay));
        for(unsigned n=1;n<failures&&delay<deadline_retry_cap_ms;++n)
            delay=delay>deadline_retry_cap_ms/2?deadline_retry_cap_ms:delay*2;
        const auto maximum=(std::numeric_limits<TimeMs>::max)();until=now>maximum-delay?maximum:now+delay;
        return delay;
    }
    void executed(){until=0;failures=0;}
};
}}}
