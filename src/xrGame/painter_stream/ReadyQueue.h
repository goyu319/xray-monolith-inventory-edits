#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace sss24_stream {

// These are sidecar identities, not the layout of a native CSE or game object.
// incarnation must come from observed native server birth/retirement, never ID alone.
struct SourceKey {
    std::uint64_t world{}, incarnation{};
    std::uint16_t id{0xffff};
    bool valid() const noexcept;
};
bool operator==(SourceKey a, SourceKey b) noexcept;
bool operator!=(SourceKey a, SourceKey b) noexcept;
struct Position { double x{}, y{}, z{}; };
enum class Kind : std::uint8_t { Spawn, Event };
struct Packet {
    // The native bridge retains immutable packet bytes and their native metadata.
    std::uint64_t token{};
    SourceKey owner{};
    Kind kind{Kind::Spawn};
    Position position{};
    std::array<SourceKey,4> dependencies{};
    std::uint8_t dependency_count{};
    std::uint64_t arrived_us{};
    bool ordinary_ready{};
    bool spawn_budget{true}; // inventory belongs to its root NPC budget
};
enum class Admission { Queued, DuplicateToken, SynchronousRequired, NeedDrain, Invalid };
enum class State { Ready, Wait, Retired, Unavailable };
enum class Dispatch { Applied, Wait, Retired, Unavailable };
enum class ReleaseReason { Applied, NativeRetired };
struct Callbacks {
    void* context{};
    bool (*clock_us)(void*,std::uint64_t&) noexcept{};
    // Revalidate exact server incarnation, native online state and packet ownership.
    State (*inspect)(void*,const Packet&) noexcept{};
    // Ready means the exact client native object already exists. A server online
    // flag or a queued packet is insufficient. Unknown/unreadable means Unavailable.
    State (*dependency)(void*,SourceKey) noexcept{};
    // Calls the original native spawn/event trampoline with all original arguments.
    Dispatch (*dispatch)(void*,const Packet&) noexcept{};
    // Releases the owned packet only after actual completion or native retirement.
    void (*release)(void*,const Packet&,ReleaseReason) noexcept{};
};
struct Frame {
    std::uint64_t world{}, serial{};
    Position actual_actor_position{};
    bool paused{};
};
// Internal tuning; no config, console, Lua, MCM or population/update changes.
struct Limits {
    std::size_t capacity{2048};
    std::uint32_t spawns_per_frame{4}, packets_per_frame{32}, probes_per_frame{64};
    std::uint64_t slice_us{1500}, age_priority_us{500000};
};
struct DrainResult {
    std::uint32_t spawned{}, events{}, retired{}, examined{}, waiting{};
    bool clock_failed{}, invalid_frame{}, reentrant{}, incomplete{};
};

// Single native game-owner lane only. Construction allocates all slot storage;
// enqueue, selection, release and draining allocate no memory. Native packet byte
// reservation is a separate required bridge responsibility.
class ReadyQueue final {
public:
    explicit ReadyQueue(Limits limits={});
    Admission Enqueue(const Packet&) noexcept;
    DrainResult DrainFrame(const Frame&,const Callbacks&) noexcept;
    // A critical synchronous packet must not overtake its parked native parent,
    // earlier owner events, or referenced objects. Replay only that dependency
    // closure before forwarding it. false/incomplete requires retaining the
    // critical packet; it must not be forwarded or dropped yet.
    DrainResult DrainRequired(const Packet& immediate,const Frame&,const Callbacks&) noexcept;
    // Rare capacity/disable recovery. Bypasses streaming budgets but retains
    // every dependency/lifetime/ordering check and calls original native work.
    // incomplete means the bridge must keep the incoming packet too.
    DrainResult DrainRecovery(const Frame&,const Callbacks&) noexcept;
    // Called only after producers joined and this native world actually retired.
    bool RetireWorld(std::uint64_t world,const Callbacks&) noexcept;
    std::size_t size() const noexcept { return occupied_; }
    std::size_t capacity() const noexcept { return slots_.size(); }
    bool HasPendingSpawn(SourceKey) const noexcept;
private:
    struct Slot {
        Packet packet{};
        std::uint64_t sequence{}, attempted_frame{}, attempted_drain{}, required_generation{};
        std::size_t next_event{static_cast<std::size_t>(-1)};
        bool occupied{}, in_flight{};
    };
    struct OwnerIndex {
        SourceKey key{};
        std::size_t spawn{static_cast<std::size_t>(-1)},first_event{static_cast<std::size_t>(-1)},last_event{static_cast<std::size_t>(-1)};
        std::uint32_t references{};
        std::uint8_t state{}; // empty/live; deletion backshifts the probe cluster
    };
    Limits limits_;
    std::vector<Slot> slots_;
    std::vector<OwnerIndex> owners_;
    std::size_t occupied_{};
    std::uint64_t next_sequence_{1}, frame_world_{}, frame_serial_{}, frame_start_{}, last_clock_{};
    std::uint64_t closure_generation_{};
    std::uint64_t drain_generation_{};
    std::uint32_t frame_spawns_{}, frame_packets_{}, frame_probes_{};
    bool draining_{};
    bool ValidPacket(const Packet&) const noexcept;
    std::size_t FindOwner(SourceKey) const noexcept;
    std::size_t Owner(SourceKey) noexcept;
    void EraseOwner(std::size_t) noexcept;
    bool ReadClock(const Callbacks&,std::uint64_t&) noexcept;
    bool EarlierEvent(std::size_t) const noexcept;
    std::size_t Select(const Frame&,std::uint64_t now,std::uint64_t required,bool all) const noexcept;
    State Dependencies(const Packet&,const Callbacks&) const noexcept;
    void Release(std::size_t,const Callbacks&,ReleaseReason) noexcept;
    DrainResult Drain(const Frame&,const Callbacks&,std::uint64_t required,bool all) noexcept;
    bool RequiredPending(std::uint64_t generation) const noexcept;
};

} // namespace sss24_stream
