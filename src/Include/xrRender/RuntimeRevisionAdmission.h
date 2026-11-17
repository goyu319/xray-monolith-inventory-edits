#pragma once
#include <cstdint>

struct RuntimeRevisionState {
    std::uint64_t owner_cookie=0;
    std::uint64_t damage_revision=0;
    bool separation_packet=false;
};

// A verified split may follow its rendered wound revision or later accepted
// no-geometry-change revisions. A further wound must advance the revision.
constexpr bool RuntimeRevisionAdmitted(const RuntimeRevisionState* current,
    std::uint64_t owner_cookie,std::uint64_t actual_generation,
    std::uint64_t expected_generation,std::uint64_t requested_revision,
    bool separation_packet) noexcept {
    if(!owner_cookie||!requested_revision||actual_generation!=expected_generation)return false;
    if(!current)return true;
    if(current->owner_cookie!=owner_cookie)return false;
    if(requested_revision<current->damage_revision)return false;
    // A previously separated owner may sever again at a later local wound
    // revision. Only the same-revision wound-to-split transition is special.
    if(separation_packet)
        return requested_revision>current->damage_revision||
            !current->separation_packet;
    return requested_revision>current->damage_revision;
}
