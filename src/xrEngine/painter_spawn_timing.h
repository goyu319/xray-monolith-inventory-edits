#pragma once
// Client object spawn attribution for the opt-in performance capture (R11). It only adds elapsed time to
// per-thread counters; spawning itself is unchanged. The level reads and resets them around each object.
#include <windows.h>

namespace painter_spawn_timing {
struct Counters
{
    double visual_ms = 0.0; // model instance creation (Render->model_Create)
    double script_ms = 0.0; // script binder net_spawn (Lua)
};

inline Counters& Current() noexcept
{
    thread_local Counters counters;
    return counters;
}

inline double NowMs() noexcept
{
    static const double ms_per_tick = []() {
        LARGE_INTEGER frequency;
        return QueryPerformanceFrequency(&frequency) && frequency.QuadPart ? 1000.0 / double(frequency.QuadPart) : 0.0;
    }();
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return double(now.QuadPart) * ms_per_tick;
}

class Measure
{
    double& sink_;
    const double start_;
public:
    explicit Measure(double& sink) noexcept : sink_(sink), start_(NowMs()) {}
    ~Measure() { sink_ += NowMs() - start_; }
    Measure(const Measure&) = delete;
    Measure& operator=(const Measure&) = delete;
};
}
