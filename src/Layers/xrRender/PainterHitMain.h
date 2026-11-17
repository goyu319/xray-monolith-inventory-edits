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
    unsigned part_;
    double inner_ = 0.0;
    Clock::time_point start_;
public:
    explicit PartScope(Part part) : record_(current), part_(part) {
        if (!record_) return;
        outer_ = record_->top;
        record_->top = this;
        start_ = Clock::now();
    }
    // Ends the part before its block ends (the innermost open part only); the destructor then does nothing.
    void stop() {
        if (!record_) return;
        const double ms = since_ms(start_);
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
    ~Hit() {
        if (!active_) return;
        const double total = since_ms(start_);
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
