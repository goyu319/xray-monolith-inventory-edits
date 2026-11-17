#pragma once
#include "LiveDamageWorker.h"

namespace painter_piece_continuation {
inline bool same_identity(const gfy_live::Identity& a,
    const gfy_live::Identity& b) noexcept {
    return a.spawn_cookie==b.spawn_cookie&&
        a.visual_lineage==b.visual_lineage&&a.topology==b.topology&&
        a.source==b.source&&a.exact_rig==b.exact_rig;
}
inline bool same_stamp(const gfy_live::Stamp& a,
    const gfy_live::Stamp& b) noexcept {
    return same_identity(a.owner,b.owner)&&
        a.expected_generation==b.expected_generation&&a.revision==b.revision;
}
inline bool receipt_admitted(const gfy_live::ComponentReceipt& receipt,
    const gfy_live::Identity& current_owner,
    const gfy_live::Stamp& immutable_parent,
    std::uint64_t current_generation,std::uint64_t local_revision,
    std::uint64_t sequence) noexcept {
    return same_identity(receipt.owner,current_owner)&&
        same_stamp(receipt.parent_stamp,immutable_parent)&&
        receipt.expected_generation==current_generation&&
        receipt.expected_local_revision==local_revision&&
        local_revision!=UINT64_MAX&&
        receipt.next_local_revision>=local_revision&&
        receipt.next_local_revision<=local_revision+1&&
        receipt.sequence==sequence;
}
}
