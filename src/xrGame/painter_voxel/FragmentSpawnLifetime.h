#pragma once
#include <cstdint>

namespace painter_damaged_fragment {
constexpr bool reserved_fragment_name(const char* name) noexcept {
    if(!name)return false;
    constexpr char prefix[]="painter_damaged_";
    for(unsigned i=0;i<sizeof(prefix)-1;++i)
        if(name[i]!=prefix[i])return false;
    return true;
}
constexpr bool fragment_spawn_expired(std::uint32_t now,std::uint32_t started,
    std::uint32_t timeout_ms) noexcept {
    return timeout_ms!=0&&std::uint32_t(now-started)>=timeout_ms;
}
}
