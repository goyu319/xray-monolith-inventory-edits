#include "stdafx.h"
#include "EngineNearMiss.h"
#include "EngineServices.h"

namespace painter { namespace combat { namespace engine {
namespace {
nearmiss::Tracker tracker;
xr_vector<nearmiss::Man> watch;                     // the men the last capture covered, with their head points
u32 watch_frame = 0;                                // Device.dwFrame of that capture
bool frame_on = false;

nearmiss::P3 point(const Fvector& v) { return {v.x, v.y, v.z}; }
} // namespace

nearmiss::Tracker& near_miss_tracker() { return tracker; }

void near_miss_frame_begin()
{
    // The watch is the last tick's: this frame's (the tick ran first) or the previous one's (the bullets run first). An
    // older one means the native AI is off or captured nobody: nothing is measured and nothing stays under way.
    frame_on = !watch.empty() && Device.dwFrame - watch_frame <= 1;
    if (!frame_on)
    {
        tracker.drop_rounds();
        return;
    }
    tracker.begin_frame(Device.dwTimeGlobal, clampr(combat_config().near_miss_m, 0.f, 50.f));
}

void near_miss_stretch(std::uint32_t round, std::uint16_t shooter, const Fvector& origin, const Fvector& a, const Fvector& b)
{
    if (frame_on) tracker.stretch(round, shooter, point(origin), point(a), point(b), watch.data(), watch.size());
}

void near_miss_struck(std::uint32_t round, std::uint16_t object)
{
    if (!frame_on) return;
    for (const auto& man : watch)
        if (man.id == object)
        {
            tracker.struck(round, object);
            return;
        }
}

void near_miss_removed(std::uint32_t round)
{
    if (frame_on) tracker.removed(round);
}

void near_miss_frame_end()
{
    if (frame_on) tracker.end_frame();
}

void near_miss_watch_begin() { watch.clear();watch_frame=Device.dwFrame; }

void near_miss_watch(std::uint16_t id, const Fvector& head)
{
    if (watch_frame != Device.dwFrame)
    {
        watch.clear();
        watch_frame = Device.dwFrame;
    }
    watch.push_back({id, point(head)});
}

void near_miss_forget(std::uint16_t id)
{
    tracker.forget(id);
    watch.erase(std::remove_if(watch.begin(), watch.end(), [id](const nearmiss::Man& man) { return man.id == id; }),
                watch.end());
}

void near_miss_reset()
{
    tracker.reset();
    watch.clear();
    frame_on = false;
}

void near_miss_summary(const char* level)
{
    const auto& c = tracker.counters();
    Msg("[painter_combat] phase=near_miss_summary level=%s stretches=%llu passes=%llu struck=%llu dropped=%llu",
        level ? level : "-", c.stretches, c.passes, c.struck, c.dropped);
}

}}} // namespace painter::combat::engine
