#pragma once
#include <chrono>

namespace painter_flight_perf {
using Clock = std::chrono::steady_clock;

struct Frame {
    unsigned frame = 0;
    unsigned calls = 0;
    unsigned depth = 0;
    double total_ms = 0;
    double longest_ms = 0;
    const char* longest_part = "";
};

inline thread_local Frame measured;
inline void flush(unsigned current) {
    auto& f = measured;
    if (f.depth || f.frame == current) return;
    if (f.calls) {
        Msg("[painter_flight] phase=split_frame frame=%u cpu_ms=%.3f calls=%u largest=%s largest_ms=%.3f",
            f.frame, f.total_ms, f.calls, f.longest_part, f.longest_ms);
    }
    f = {};
    f.frame = current;
}

class Scope {
    const char* part;
    bool outer;
    Clock::time_point start;
public:
    explicit Scope(const char* label) : part(label), outer(!measured.depth) {
        if (outer) {
            flush(Device.dwFrame);
            start = Clock::now();
        }
        ++measured.depth;
    }
    ~Scope() {
        --measured.depth;
        if (!outer) return;
        const double ms = std::chrono::duration<double,std::milli>(Clock::now()-start).count();
        ++measured.calls;
        measured.total_ms += ms;
        if (ms > measured.longest_ms) { measured.longest_ms=ms; measured.longest_part=part; }
    }
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;
};
}
