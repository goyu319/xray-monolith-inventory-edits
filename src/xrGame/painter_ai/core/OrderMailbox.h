#pragma once
#include "../CombatContract.h"
#include "PointFireAuthority.h"
#include <cstddef>
#include <vector>

namespace painter { namespace combat { namespace core {

enum class MailboxReject : std::uint8_t {
    none, epoch, geometry, backwards_snapshot, future_time, source_age,
    invalid_snapshot, duplicate_actor, missing_actor, identity, ownership,
    restrictions, dead, ineligible, navigation, invalid_order, nonfinite,
    older_order, perception, target_dead, memory_budget, work_budget,
    invalid_token, native_result_invalid, native_rejected
};
const char* mailbox_reason(MailboxReject reason);
struct MailboxToken {
    std::uint64_t epoch=0, generation=0, id=0;
    explicit operator bool() const { return id!=0; }
};
struct MailboxActorResult {
    ActorKey actor;
    bool release=false;
    MailboxReject reason=MailboxReject::none;
};
struct MailboxResult {
    MailboxToken token;
    MailboxReject envelope=MailboxReject::none;
    std::vector<MailboxActorResult> actors;
};
struct MailboxLimits {
    std::size_t pending_bytes=2*1024*1024;
    std::size_t committed_bytes=2*1024*1024;
    std::size_t validation_bytes=2*1024*1024; // logical temporary-index bytes; allocator overhead excluded
    std::size_t validation_work=200000; // charged record/lookup visits, not NPC admission count
    TimeMs source_max_age_ms=500;
    TimeMs memory_max_age_ms=5000;
    TimeMs max_order_lifetime_ms=3000;
};

// Portable authority-thread mailbox. No engine pointers or callbacks are stored.
// prepare performs value validation only; it never advances applied generations.
// The engine inspects prepared(token), checks live navigation/fire/ownership, and
// applies accepted members on its serialized publication seam. commit is called
// immediately on that same seam with precisely the successfully applied keys.
// Per-member rejection is intentional; one casualty does not stop other members.
class OrderMailbox {
public:
    explicit OrderMailbox(MailboxLimits limits={});
    void reset(std::uint64_t level_epoch);
    MailboxResult prepare(WorldSnapshot const& source,WorldSnapshot const& current,
                          Decision const& decision,TimeMs now,const std::vector<PointFireAuthority>& point_fire={});
    bool prepared(MailboxToken token,Decision& out) const;
    bool prepared_point_fire(MailboxToken token,std::vector<PointFireAuthority>& out) const;
    MailboxResult commit(MailboxToken token,WorldSnapshot const& current,TimeMs now,
                         std::vector<ActorKey> const& native_accepted);
    bool cancel(MailboxToken token);
    std::size_t expire(TimeMs now);
    std::size_t pending_bytes() const;
    std::size_t committed_records() const;
    std::uint64_t applied_order(ActorKey actor,std::uint64_t ownership) const;
    std::uint64_t applied_snapshot(std::uint32_t side) const;
private:
    struct Pending {
        MailboxToken token;
        Decision decision;
        TimeMs source_time=0;
        std::vector<ActorSnapshot> sources;
        std::vector<PointFireAuthority> point_fire;
        std::size_t bytes=0;
    };
    struct Applied { ActorKey actor;std::uint64_t ownership=0,order=0; };
    struct Side { std::uint32_t id=0;std::uint64_t snapshot=0,event_snapshot=0; };
    MailboxLimits limits_;
    std::uint64_t epoch_=0,generation_=1,next_=1;
    std::size_t bytes_=0;
    std::vector<Pending> pending_;
    std::vector<Applied> applied_;
    std::vector<Side> sides_;
};
}}}
