#include "EngineOutdoorSettings.h"
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <set>
#include <type_traits>
namespace painter { namespace combat { namespace engine {
namespace {
struct Bad {std::string why;};
class Reader {
    std::map<std::string,std::string> rows_;std::set<std::string> used_;
public:
    explicit Reader(const OutdoorSettingRows& rows){for(const auto&r:rows)if(!rows_.emplace(r).second)throw Bad{"duplicate key: "+r.first};}
    bool has(const char*k)const{return rows_.count(k)!=0;}
    template<class T>void get(const char*k,T& out,double lo,double hi){
        const auto it=rows_.find(k);if(it==rows_.end())throw Bad{"missing key: "+std::string(k)};
        used_.insert(k);const auto&s=it->second;char*end=nullptr;errno=0;
        const double v=std::strtod(s.c_str(),&end);
        if(errno||end==s.c_str())throw Bad{"invalid number: "+std::string(k)};
        while(*end==' '||*end=='\t'||*end=='\r'||*end=='\n')++end;
        if(*end||!std::isfinite(v)||v<lo||v>hi)throw Bad{"out of range: "+std::string(k)};
        if constexpr(std::is_integral<T>::value){
            if(std::floor(v)!=v||v>9007199254740991.0||v>double((std::numeric_limits<T>::max)()))throw Bad{"nonrepresentable integer: "+std::string(k)};
        }
        if constexpr(std::is_same<T,bool>::value){if(v!=0&&v!=1)throw Bad{"expected 0 or 1: "+std::string(k)};}
        out=static_cast<T>(v);
    }
    void exact(){for(const auto&r:rows_)if(!used_.count(r.first))throw Bad{"unknown key: "+r.first};}
};
void require(bool v,const char*s){if(!v)throw Bad{std::string("inconsistent fields: ")+s};}
void ordered(const core::OutdoorDispatchPolicy&p){
    const auto&b=p.bounds;
    require(b.bound_max_m>=b.bound_min_m&&b.flank_max_m>=b.flank_min_m&&b.flank_preferred_deg>=b.flank_min_deg,"bounds ranges");
    require(b.final_walk_m>=b.arrival_m&&b.sprint_distance_m>=b.final_walk_m,"bounds gait distances");
    const auto&f=p.fire;require(f.cycle_ms>=f.burst_ms&&f.far_known_m>=f.near_known_m,"fire cycle/ranges");
    float previous=2;for(float x:f.suppression_fractions){require(x>0&&x<=1&&x<previous,"fire fractions decreasing");previous=x;}
    require(f.suppression_fractions[0]==1,"fire first fraction must be exact last known point");
    const auto&s=p.solo;for(const auto&q:s.profiles)require(q.hold_jitter_max>=q.hold_jitter_min,"solo profile jitter");
    require(s.distant_jitter_max>=s.distant_jitter_min&&s.rush_run_m>=s.rush_m&&s.injured_hp>=s.dying_hp,"solo distance/rush/health");
    require(s.mte_max_m>=s.mte_min_m&&s.search_jitter_max>=s.search_jitter_min&&s.freeze_max_ms>=s.freeze_min_ms&&s.throw_max_m>=s.throw_min_m,"solo search/throw ranges");
    require(!s.enabled||p.personal.enabled,"Solo requires Personal");
    require(s.combat.lean_crouch<=s.combat.lean_full,"CQB lean strengths");
    previous=1;for(float x:s.combat.meter_hold_scale){require(x<=previous,"CQB meter scale order");previous=x;}
    const auto&m=p.movement;require(m.close_spread_m<=m.spread_m,"movement spread");
    const auto&w=p.withdrawal;require(w.max_leg_m>=w.min_leg_m&&w.pair_leg_m>=w.min_leg_m&&w.pair_leg_m<=w.max_leg_m&&w.final_walk_m>=w.arrival_m,"withdrawal leg/arrival ranges");
    require(!p.withdrawal_enable||p.assessment_enable,"withdrawal requires assessment");
    const auto&x=p.self;
    require(x.maximum_ratio<1&&x.peace_ratio<=x.maximum_ratio&&x.search_low_ratio<x.search_high_ratio&&x.search_high_ratio<x.maximum_ratio&&x.low_ratio<x.middle_ratio&&x.middle_ratio<x.upper_ratio&&x.upper_ratio<x.maximum_ratio,"Self ammo bands");
    previous=0;for(float v:x.distance_bands_m){require(v>previous,"Self distance bands");previous=v;}
    require(x.cancel_ms>=x.keep_ms,"Self pending times");require(!x.enabled||p.personal.enabled,"Self requires Personal");
}
}
OutdoorSettings parse_outdoor_settings(const OutdoorSettingRows& rows){
    OutdoorSettings r;
    try{
        if(rows.empty())return r;
        Reader reader(rows);bool enabled=false;reader.get("enabled",enabled,0,1);
        if(!enabled){r.status=OutdoorSettingsStatus::disabled;return r;}
        unsigned schema=0;reader.get("schema",schema,1,1);
        core::OutdoorDispatchPolicy p{};core::RoomExposureDispatchPolicy e{};core::ExposureGeometryPolicy g{};
#define OUTDOOR_SETTING(k,field,lo,hi) reader.get(k,field,lo,hi);
#include "OutdoorSettingsFields.inc"
#undef OUTDOOR_SETTING
        reader.exact();ordered(p);
        require(r.route.queue_bytes>=r.route.job_overhead_bytes+std::size_t(r.route.output_nodes)*64,"route queue cannot fit one bounded receipt");
        require(r.doors.queries_per_capture<=r.doors.actor_scans_per_capture,"door query quota exceeds scan quota");
        if(r.exposure_enabled){
            require(e.cache.source_range_m==g.source_range_m&&e.cache.source_floor_m==g.source_floor_m,"exposure source range/floor mismatch");
            require(e.cache.half_fov_deg<=180&&e.cache.journal_records>=2&&e.cache.transaction_records>=2,"exposure FOV/transaction capacity");
            for(const auto&price:e.prices)require(price.sample_height_m==g.source_height_m,"exposure source/destination sample height alignment");
            const auto minimum=g.memory_bytes+e.cache.memory_bytes+e.store_overhead_bytes*2+sizeof(core::ExposureGeometryPolicy)+e.cache.observer_capacity*sizeof(core::ExposureObserver);
            require(minimum<e.total_memory_bytes,"exposure total memory cannot fit geometry and one cache");
            e.geometry=std::make_shared<const core::ExposureGeometryPolicy>(g);
            r.exposure=std::make_shared<const core::RoomExposureDispatchPolicy>(e);
        }
        r.policy=std::make_shared<const core::OutdoorDispatchPolicy>(std::move(p));
        const auto&v=r.lifecycle_values;
        if(v.enabled)r.lifecycle=std::make_shared<const core::SideLifecyclePolicy>(v.quiet_field_ms,v.quiet_building_ms,v.stand_down_after_ms,v.side_cooldown_ms,v.error_cooldown_ms,v.retain_authored_mission_without_evidence);
        r.status=OutdoorSettingsStatus::ready;
    }catch(const Bad&e){r=OutdoorSettings{};r.status=OutdoorSettingsStatus::invalid;r.error=e.why;}
    catch(...){r=OutdoorSettings{};r.status=OutdoorSettingsStatus::invalid;r.error="settings allocation failed";}
    return r;
}
}}}
