#pragma once
#include <atomic>
#include <Windows.h>
namespace painter_stream {
enum class PrefetchWait {Ready,Stopped,Failed};
inline PrefetchWait WaitForPrefetch(std::atomic<bool>& stop,HANDLE signal)noexcept{
    // A stop SetEvent can be followed by the previous batch's ResetEvent.
    // Check before *every* wait; after this check no reset occurs in this path.
    if(stop.load(std::memory_order_acquire))return PrefetchWait::Stopped;
    const auto status=WaitForSingleObject(signal,INFINITE);
    if(status!=WAIT_OBJECT_0)return PrefetchWait::Failed;
    return stop.load(std::memory_order_acquire)?PrefetchWait::Stopped:PrefetchWait::Ready;
}
}
