#pragma once
// triggerhitch (2026-11-15): main-thread time of one received hit, always on and cheap (two clock reads a scope).
// One received hit (GE_HIT -> Hit) or one bullet impact on a body is a Record; the scopes inside it add their
// EXCLUSIVE time to a part (a part nested in another is subtracted from the outer one). When the record's total
// passes 2 ms, ONE line is printed:
//   [painter_perf] phase=hit_main id=<object> stage=<received|bullet> part=<largest part> ms=<total> <part>=<ms>...
// The per-frame apply / poll call prints its own line through painter_voxel_native::SlowScope (stage=apply).
// No allocation, no lock; a scope outside a record does nothing; at most 30 lines a second (skipped=N counts the rest).
#include <chrono>
#include <cstdio>
#include <cstring>

namespace painter_hit_main {
enum Part : unsigned { Script, Surface, Wallmark, Piece, Capture, Bind, Policy, Queue, Dispatch, PartCount };
inline const char* part_name(unsigned p) {
    static const char* const names[PartCount] = {"script", "surface", "wallmark", "piece", "capture", "bind", "policy", "queue", "dispatch"};
    return p < PartCount ? names[p] : "other";
}
using Clock = std::chrono::steady_clock;
inline double since_ms(Clock::time_point t) { return std::chrono::duration<double, std::milli>(Clock::now() - t).count(); }
constexpr double threshold_ms = 2.0;
// At most 30 lines a second on a thread (a log line costs main-thread time too); the next printed line carries skipped=N.
inline thread_local unsigned skipped_lines = 0;
inline bool admit_line() {
    static thread_local Clock::time_point window{};
    static thread_local unsigned used = 0;
    const Clock::time_point now = Clock::now();
    if (now - window > std::chrono::seconds(1)) { window = now; used = 0; }
    if (used >= 30) { ++skipped_lines; return false; }
    ++used;
    return true;
}
inline unsigned take_skipped() { const unsigned n = skipped_lines; skipped_lines = 0; return n; }

// Measure the union of received multi-pellet intervals on the owner thread.
// The innermost active phase owns each millisecond, also across nested Hit records.
struct VolleyFrame {
    unsigned frame=0, contacts=0;
    double total=0, script=0, surface=0, peak=0;
};
inline thread_local VolleyFrame volley;
struct VolleyPath {
    unsigned depth=0;
    Part phase=PartCount;
    Clock::time_point last{};
};
inline thread_local VolleyPath volley_path;
inline void flush_volley(unsigned frame) {
    if (!volley_path.depth && volley.contacts && volley.frame != frame) {
        Msg("[painter_perf] phase=shotgun_frame frame=%u contacts=%u received_ms=%.2f script_ms=%.2f surface_ms=%.2f max_contact_ms=%.2f",
            volley.frame,volley.contacts,volley.total,volley.script,volley.surface,volley.peak);
        volley = {};
    }
}
inline Part volley_phase(Part next, Clock::time_point now) {
    const Part previous=volley_path.phase;
    const double ms=std::chrono::duration<double, std::milli>(now-volley_path.last).count();
    if(previous==Script) volley.script+=ms;
    if(previous==Surface) volley.surface+=ms;
    volley_path.phase=next;
    volley_path.last=now;
    return previous;
}
class PartScope;
struct Record {
    unsigned id = 0;
    double part[PartCount] = {};
    unsigned calls[PartCount] = {};
    double nested = 0.0; // another record that ran inside this one (printed on its own line)
    PartScope* top = nullptr;
};
inline thread_local Record* current = nullptr;

class PartScope {
    Record* record_;
    PartScope* outer_ = nullptr;
    Part previous_volley_phase_=PartCount;
    bool tracks_volley_=false;
    unsigned part_;
    double inner_ = 0.0;
    Clock::time_point start_;
public:
    explicit PartScope(Part part) : record_(current), part_(part) {
        if (!record_) return;
        outer_ = record_->top;
        record_->top = this;
        start_ = Clock::now();
        if(volley_path.depth) {
            tracks_volley_=true;
            previous_volley_phase_=volley_phase(part,start_);
        }
    }
    // Ends the part before its block ends (the innermost open part only); the destructor then does nothing.
    void stop() {
        if (!record_) return;
        const auto now=Clock::now();
        const double ms=std::chrono::duration<double, std::milli>(now-start_).count();
        if(tracks_volley_) volley_phase(previous_volley_phase_,now);
        record_->part[part_] += ms - inner_;
        ++record_->calls[part_];
        if (outer_) outer_->inner_ += ms;
        if (record_->top == this) record_->top = outer_;
        record_ = nullptr;
    }
    ~PartScope() { stop(); }
    PartScope(const PartScope&) = delete;
    PartScope& operator=(const PartScope&) = delete;
};

class Hit {
    Record record_;
    bool volley_contact_=false;
    Record* previous_;
    const char* stage_;
    bool active_;
    Clock::time_point start_;
public:
    Hit(const char* stage, unsigned id, bool active = true) : previous_(current), stage_(stage), active_(active) {
        if (!active_) return;
        record_.id = id;
        current = &record_;
        start_ = Clock::now();
    }
    void mark_multi_pellet(unsigned frame) {
        if(!active_ || volley_contact_) return;
        if(!volley_path.depth) {
            flush_volley(frame);
            volley.frame=frame;
            volley_path.last=start_;
            volley_path.phase=PartCount;
        }
        ++volley_path.depth;
        volley_contact_=true;
    }
    ~Hit() {
        if (!active_) return;
        const auto now=Clock::now();
        const double total=std::chrono::duration<double, std::milli>(now-start_).count();
        if(volley_contact_) {
            ++volley.contacts;
            if(total>volley.peak) volley.peak=total;
            if(--volley_path.depth==0) {
                volley_phase(PartCount,now);
                volley.total+=total;
            }
        }
        current = previous_;
        if (previous_) previous_->nested += total;
        if (total < threshold_ms || !admit_line()) return;
        double named = record_.nested;
        unsigned largest = PartCount;
        double largest_ms = 0.0;
        for (unsigned p = 0; p < PartCount; ++p) {
            named += record_.part[p];
            if (record_.part[p] > largest_ms) { largest_ms = record_.part[p]; largest = p; }
        }
        const double other = total > named ? total - named : 0.0;
        if (other > largest_ms) largest = PartCount;
        char parts[512];
        int used = 0;
        for (unsigned p = 0; p < PartCount && used >= 0 && used < int(sizeof(parts)); ++p)
            if (record_.calls[p])
                used += std::snprintf(parts + used, sizeof(parts) - used, " %s=%.2f/%u", part_name(p), record_.part[p], record_.calls[p]);
        if (used < 0) used = 0;
        if (used >= int(sizeof(parts))) used = int(sizeof(parts)) - 1;
        parts[used] = 0;
        Msg("[painter_perf] phase=hit_main id=%u stage=%s part=%s ms=%.2f other=%.2f nested=%.2f skipped=%u%s", record_.id, stage_,
            part_name(largest), total, other, record_.nested, take_skipped(), parts);
    }
    Hit(const Hit&) = delete;
    Hit& operator=(const Hit&) = delete;
};
} // namespace painter_hit_main
