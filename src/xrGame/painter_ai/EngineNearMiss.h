#pragma once
// painter_ai/EngineNearMiss.h - near misses for the native combat AI (contract v2.14 ActorSnapshot::near_misses): the
// bullet manager measures, capture delivers. The measurement itself is NearMiss.h (engine-free, tested offline).
//
// Threads: the bullet side runs inside CBulletManager::UpdateWorkload (under its m_Lock: a Device.seqParallel task on the
// game thread, or inline in CommitEvents on the main thread when mtBullets is off); the capture side runs on the authority
// thread (the publish task: the game thread after the scheduler, or inline in CLevel::OnFrame when mt_Scheduler is off).
// The two never overlap in any of those modes: both are on the game thread in sequence, or one runs inside FrameMove on
// the main thread while the game thread of the frame before has been joined (secondary_tasks.wait) and the one of this
// frame has not started. So no lock of its own, and none taken under m_Lock.
#include "NearMiss.h"

namespace painter { namespace combat { namespace engine {

// Bullet manager side
void near_miss_frame_begin();   // UpdateWorkload start: on while a capture of this or the previous frame watches men
void near_miss_stretch(std::uint32_t round, std::uint16_t shooter, const Fvector& origin, const Fvector& a, const Fvector& b);
void near_miss_struck(std::uint32_t round, std::uint16_t object);   // a dynamic hit (RegisterEvent): a hit, not a miss
void near_miss_removed(std::uint32_t round);                        // the round ends this frame
void near_miss_frame_end();

// Capture side
void near_miss_watch_begin();                                    // replace the registered observation roster, even empty
void near_miss_watch(std::uint16_t id, const Fvector& head);        // measured eye of each registered alive man, before admission
template <class Sink> void near_miss_take(std::uint16_t id, std::uint32_t since, Sink&& sink);
void near_miss_forget(std::uint16_t id);                            // net_Destroy
void near_miss_reset();                                             // level load
void near_miss_summary(const char* level);                          // one log line per level

nearmiss::Tracker& near_miss_tracker();
template <class Sink> void near_miss_take(std::uint16_t id, std::uint32_t since, Sink&& sink)
{
    near_miss_tracker().take(id, since, sink);
}

}}} // namespace painter::combat::engine
