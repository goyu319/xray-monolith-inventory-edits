#pragma once
#include "CombatCore.h"
#include <array>
#include <algorithm>
#include <cmath>
namespace painter { namespace combat { namespace core {
// Executor-local corrections preserve the worker's destination and fire gates.
// The engine certifies the short direct route before accepting a candidate.
struct ExecutorStepPlan { std::array<Vec3,3> candidates{}; unsigned count=0; TimeMs lifetime=0; };
inline ExecutorStepPlan executor_step_plan(const Config& c,const Order& order,Vec3 feet,Vec3 enemy,
    bool visible,bool line_clear,bool friend_in_line,bool busy,bool arrived,bool stale) {
    ExecutorStepPlan p;
    if(!visible||busy||stale||friend_in_line||!arrived||order.kind==OrderKind::entry||order.kind==OrderKind::stack
        ||order.throw_order.valid||order.aid.id!=invalid_id||!finite(feet)||!finite(enemy))return p;
    Vec3 d{enemy.x-feet.x,0,enemy.z-feet.z};const float n=std::sqrt(d.x*d.x+d.z*d.z);
    if(n<.001f)return p;d.x/=n;d.z/=n;
    p.lifetime=c.room.backstep_ms;
    if(n<c.room.backstep_enemy_m) {
        p.candidates[p.count++]={feet.x-d.x*c.room.backstep_m,feet.y,feet.z-d.z*c.room.backstep_m};
    } else if(!line_clear) {
        const float step=c.room.denial_step_m;
        p.candidates[p.count++]={feet.x-d.z*step,feet.y,feet.z+d.x*step};
        p.candidates[p.count++]={feet.x+d.z*step,feet.y,feet.z-d.x*step};
    }
    return p;
}
inline bool expired_idle_order(const Order& o,TimeMs now,bool arrived) {
    return now>=o.expires_at_ms&&(arrived||o.pace==Pace::stand);
}
// Telemetry cadence only; not a gameplay rule or a second settings surface.
constexpr TimeMs runtime_report_interval_ms=5000;
struct PassCosts {
    std::array<float,512> samples{};std::size_t count=0,cursor=0;std::uint64_t passes=0;float worst=0;
    void add(float ms){if(!std::isfinite(ms)||ms<0)return;samples[cursor++%samples.size()]=ms;
        count=(std::min)(count+1,samples.size());++passes;worst=(std::max)(worst,ms);}
    float median()const{if(!count)return 0;auto copy=samples;std::sort(copy.begin(),copy.begin()+count);return copy[count/2];}
};
}}}
