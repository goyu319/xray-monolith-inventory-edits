#pragma once
#include <atomic>
#include <cstdint>
#include <limits>
// Nonserialized process-lifetime identities. Exhaustion is sticky: never wrap or recycle.
namespace painter_lifetime {
inline std::uint64_t allocate(std::atomic<std::uint64_t>& counter) noexcept {
    auto before=counter.load(std::memory_order_relaxed);
    while(before!=std::numeric_limits<std::uint64_t>::max()){
        if(counter.compare_exchange_weak(before,before+1,std::memory_order_relaxed))return before+1;
    }
    return 0;
}
}

