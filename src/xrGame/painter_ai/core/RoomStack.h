#pragma once
#include "CombatCore.h"
#include "RoomExposureCost.h"
#include <vector>

namespace painter { namespace combat { namespace core {
struct RoomStackRequest {
    std::uint32_t target_room=invalid_room, entry_portal=0;
    std::vector<std::uint32_t> searched_rooms;
    bool permit_two_doors=true, call_breacher_first=false;
    const RoomExposureCostView* exposure=nullptr; // Borrowed for this update only.
    std::vector<Order> applied_orders; // Exact accepted care leases; a role label alone is not proof.
};
bool room_care_owned(const WorldSnapshot&,const ActorSnapshot&,const std::vector<Order>&);
enum class RoomStackPhase : std::uint8_t { invalid, forming, ready };
enum class StackReason : std::uint8_t { formed, arrivals, straggler_cost, no_geometry, no_projection, invalid_epoch, invalid_config };
struct RoomStackSlot {
    ActorKey actor;
    Order order;
    std::uint32_t entry_portal=0;
    bool watcher=false, arrived=false, delayed=false;
    float placement_cost=0, red_cost=0;
    RoomExposureCostReceipt exposure;
};
struct RoomStackResult {
    RoomStackPhase phase=RoomStackPhase::invalid;
    StackReason reason=StackReason::no_geometry;
    std::uint64_t generation=0;
    TimeMs formed_at_ms=0;
    std::uint32_t target_room=invalid_room;
    std::vector<RoomStackSlot> slots;
    std::vector<Event> events;
    float go_cost=0, wait_cost=0;
    float single_door_cost=0,split_door_cost=0; // Same projected placement cost units.
    bool two_doors=false;
    std::size_t unprojected=0;
    RoomExposureCostStats exposure;
};

// One side, caller-owned lane state. Forming never waits for a dwell timer.
// Orders are proposals: publication must allocate sequences and check live
// navigation. No raw NPC pointer or enemy world state enters this planner.
class RoomStack {
public:
    explicit RoomStack(Config={});
    RoomStackResult update(const WorldSnapshot&,const SideSnapshot&,const RoomStackRequest&,
        const EngineServices&,const std::function<bool()>& cancelled={});
    void reset();
private:
    RoomStackResult update_impl(const WorldSnapshot&,const SideSnapshot&,const RoomStackRequest&,
        const EngineServices&,const std::function<bool()>&);
    struct MemberIdentity {ActorKey actor;std::uint64_t owner=0,restrictions=0;};
    struct RejectedSlot {MemberIdentity member;std::uint32_t node=invalid_node;};
    struct ReadinessGap {std::uint32_t portal=0;ActorKey front,back;TimeMs since=0;};
    struct Closing {ActorKey actor;std::uint32_t node=invalid_node;float best=0;TimeMs progressed_ms=0;};
    Config engine_config_;
    RoomConfig config_;
    RoomStackResult state_;
    std::uint64_t epoch_=0, geometry_=0, map_=0;
    std::uint32_t mission_revision_=0, side_=0, requested_portal_=0;
    std::uint32_t single_portal_=0;
    TimeMs last_retry_ms_=0;
    std::vector<MemberIdentity> roster_;
    std::vector<std::uint32_t> searched_;
    std::vector<RejectedSlot> rejected_;
    std::vector<ReadinessGap> gaps_;
    std::vector<Closing> closing_;
    bool permit_two_doors_=true, call_breacher_first_=false;
    bool queued_entry_=false; // Source-side nav queue requires an observed ready pair before GO.
};
}}}
