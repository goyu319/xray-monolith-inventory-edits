#pragma once
#include "OutdoorConfig.h"
#include "core/RoomExposureCache.h"             // ExposureObserver: the exposure memory fit
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <type_traits>

namespace painter { namespace combat { namespace engine {

struct OutdoorIntake {
    unsigned read = 0, missing = 0, bad = 0;
    std::string missing_keys, bad_keys, layers_off, reverted;   // comma lists for the log
};

// The names the reader knows (outdoor_ prefixed), for the unknown-key check
constexpr const char* outdoor_setting_names[] = {
#define OUTDOOR_SETTING(k, field, lo, hi) "outdoor_" k,
#include "OutdoorSettingsFields.inc"
#undef OUTDOOR_SETTING
};

namespace outdoor_read_detail {
inline void note(std::string& list, const char* item)
{
    if (!list.empty()) list += ',';
    list += item;
}
template <class T, class Lookup>
void read(Lookup& lookup, const char* key, T& field, double lo, double hi, OutdoorIntake& intake)
{
    const char* text = lookup(key);
    if (!text)
    {
        ++intake.missing;
        note(intake.missing_keys, key);
        return;
    }
    char* end = nullptr;
    errno = 0;
    const double v = std::strtod(text, &end);
    bool ok = end != text && !errno;
    if (ok)
    {
        while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n') ++end;
        ok = !*end && std::isfinite(v) && v >= lo && v <= hi;
    }
    if constexpr (std::is_same<T, bool>::value)
        ok = ok && (v == 0 || v == 1);
    else if constexpr (std::is_integral<T>::value)
        ok = ok && std::floor(v) == v && v <= 9007199254740991.0 && v <= double((std::numeric_limits<T>::max)());
    if (!ok)
    {
        ++intake.bad;
        note(intake.bad_keys, key);
        return;
    }
    field = static_cast<T>(v);
    ++intake.read;
}
} // namespace outdoor_read_detail

template <class Lookup>
void read_outdoor_config(Lookup&& lookup, OutdoorConfig& out, OutdoorIntake& intake)
{
    using outdoor_read_detail::note;
    auto& p = out.p;
    auto& r = out.r;
    auto& e = out.e;
    auto& g = out.g;
#define OUTDOOR_SETTING(k, field, lo, hi) outdoor_read_detail::read(lookup, "outdoor_" k, field, lo, hi, intake);
#include "OutdoorSettingsFields.inc"
#undef OUTDOOR_SETTING
    const OutdoorConfig defaults;

    // always-on groups: a contradiction puts the group back to the contract defaults
    const auto& b = p.bounds;
    if (!(b.bound_max_m >= b.bound_min_m && b.flank_max_m >= b.flank_min_m && b.flank_preferred_deg >= b.flank_min_deg &&
          b.final_walk_m >= b.arrival_m && b.sprint_distance_m >= b.final_walk_m))
    {
        p.bounds = defaults.p.bounds;
        note(intake.reverted, "bounds");
    }
    if (!(p.movement.close_spread_m <= p.movement.spread_m))
    {
        p.movement = defaults.p.movement;
        note(intake.reverted, "movement");
    }
    if (!(r.route.queue_bytes >= r.route.job_overhead_bytes + std::size_t(r.route.output_nodes) * 64))
    {
        r.route = defaults.r.route;
        note(intake.reverted, "route");
    }

    // optional layers: a contradiction switches the layer off
    const auto& f = p.fire;
    bool fire_ok = f.cycle_ms >= f.burst_ms && f.far_known_m >= f.near_known_m && f.suppression_fractions[0] == 1;
    float previous = 2;
    for (float x : f.suppression_fractions) { fire_ok = fire_ok && x > 0 && x <= 1 && x < previous; previous = x; }
    if (!fire_ok && p.fire_enable) { p.fire_enable = false; note(intake.layers_off, "fire"); }

    const auto& s = p.solo;
    bool solo_ok = s.distant_jitter_max >= s.distant_jitter_min && s.rush_run_m >= s.rush_m && s.injured_hp >= s.dying_hp &&
        s.mte_max_m >= s.mte_min_m && s.search_jitter_max >= s.search_jitter_min && s.freeze_max_ms >= s.freeze_min_ms &&
        s.throw_max_m >= s.throw_min_m && s.combat.lean_crouch <= s.combat.lean_full && p.personal.enabled;
    for (const auto& q : s.profiles) solo_ok = solo_ok && q.hold_jitter_max >= q.hold_jitter_min;
    previous = 1;
    for (float x : s.combat.meter_hold_scale) { solo_ok = solo_ok && x <= previous; previous = x; }
    if (!solo_ok && p.solo.enabled) { p.solo.enabled = false; note(intake.layers_off, "solo"); }

    const auto& w = p.withdrawal;
    const bool withdrawal_ok = w.max_leg_m >= w.min_leg_m && w.pair_leg_m >= w.min_leg_m && w.pair_leg_m <= w.max_leg_m &&
        w.final_walk_m >= w.arrival_m && p.assessment_enable;
    if (!withdrawal_ok && p.withdrawal_enable) { p.withdrawal_enable = false; note(intake.layers_off, "withdrawal"); }

    const auto& x = p.self;
    bool self_ok = x.maximum_ratio < 1 && x.peace_ratio <= x.maximum_ratio && x.search_low_ratio < x.search_high_ratio &&
        x.search_high_ratio < x.maximum_ratio && x.low_ratio < x.middle_ratio && x.middle_ratio < x.upper_ratio &&
        x.upper_ratio < x.maximum_ratio && x.cancel_ms >= x.keep_ms && p.personal.enabled;
    previous = 0;
    for (float v : x.distance_bands_m) { self_ok = self_ok && v > previous; previous = v; }
    if (!self_ok && p.self.enabled) { p.self.enabled = false; note(intake.layers_off, "self"); }

    if (r.doors.queries_per_capture > r.doors.actor_scans_per_capture)
    {
        r.doors.actor_scans_per_capture = r.doors.queries_per_capture = r.doors.rows_per_capture = 0;
        note(intake.layers_off, "doors");
    }

    if (r.exposure_enabled)
    {
        bool exposure_ok = e.cache.source_range_m == g.source_range_m && e.cache.source_floor_m == g.source_floor_m &&
            e.cache.half_fov_deg <= 180 && e.cache.journal_records >= 2 && e.cache.transaction_records >= 2;
        for (const auto& price : e.prices) exposure_ok = exposure_ok && price.sample_height_m == g.source_height_m;
        const auto minimum = g.memory_bytes + e.cache.memory_bytes + e.store_overhead_bytes * 2 +
            sizeof(core::ExposureGeometryPolicy) + e.cache.observer_capacity * sizeof(core::ExposureObserver);
        exposure_ok = exposure_ok && minimum < e.total_memory_bytes;
        if (!exposure_ok) { r.exposure_enabled = false; note(intake.layers_off, "exposure"); }
    }
}

}}} // namespace painter::combat::engine
