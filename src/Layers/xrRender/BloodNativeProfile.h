#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <new>

#if defined(_WIN32)
#include <Windows.h>
#endif

// Opt-in CPU elapsed-time capture. No smoothing, GPU queries, file I/O, or
// allocations in a Scope. The disabled path only reads the admission gate.
namespace BloodNativeProfile
{
enum class Stage : std::uint32_t
{
    StaticCreate, BodyCreate, RenderLockWait, RenderStatic, RenderBody,
    StreamMap, StreamUnmap, StreamDraw, RenderGraph, RenderTotal,
    StaticQuery, StaticAdjacency, StaticProject, StaticLockWait, BodyLockWait,
    // Format v3 appends stages; the original fifteen numeric IDs stay stable.
    FrameTotal, FrameUpdate, FrameRender, RenderEnd, PresentWait, SecondaryWait,
    BulletUpdate, BulletCommit, StalkerHit, EntityHit, BulletImpactCallback,
    ScriptHitCallback, EntityDeathCallback, ScriptRayQuery, FrameParticles,
    FrameDeferredCallbacks,
    // Append-only capture diagnostics; original stage IDs stay stable.
    BodyPick, BodyCollect, BodyListWait, BodyProject, ParticleCreate,
    ParticleSimulation, ParticleSchedule, ParticleWait,
    ScriptBinderUpdate, ScriptLevelUpdate,
    // Format v5 appends coarse renderer attribution; earlier IDs stay stable.
    DeviceBegin, RenderCallbacks, RenderStatistics,
    RenderSceneBeforeWallmarks, RenderWallmarkSetup, RenderSceneAfterWallmarks,
    RenderSun, RenderLightsNormal, RenderLightsPending, RenderCombine,
    RenderForward, RenderHud,
    // Format v6 appends painter and game-thread attribution; earlier IDs stay stable.
    PainterApply, PainterCapture, GameThreadTotal, SchedulerUpdate, PhysicsStep, PainterReserved0, PainterReserved1,
    // Format v7 appends client object spawn and upscaler attribution; earlier IDs stay stable.
    SpawnObject, UpscaleEvaluate,
    // R37 appends wallmark resolve and mask-shader assembly attribution; earlier IDs stay stable.
    WallmarkResolve, WallmarkMaskAssembly, Count
};

inline const char* StageName(Stage stage)
{
    static const char* const names[] = {
        "StaticCreate", "BodyCreate", "RenderLockWait", "RenderStatic", "RenderBody",
        "StreamMap", "StreamUnmap", "StreamDraw", "RenderGraph", "RenderTotal",
        "StaticQuery", "StaticAdjacency", "StaticProject", "StaticLockWait", "BodyLockWait",
        "FrameTotal", "FrameUpdate", "FrameRender", "RenderEnd", "PresentWait", "SecondaryWait",
        "BulletUpdate", "BulletCommit", "StalkerHit", "EntityHit", "BulletImpactCallback",
        "ScriptHitCallback", "EntityDeathCallback", "ScriptRayQuery", "FrameParticles",
        "FrameDeferredCallbacks",
        "BodyPick", "BodyCollect", "BodyListWait", "BodyProject", "ParticleCreate",
        "ParticleSimulation", "ParticleSchedule", "ParticleWait",
        "ScriptBinderUpdate", "ScriptLevelUpdate",
        "DeviceBegin", "RenderCallbacks", "RenderStatistics",
        "RenderSceneBeforeWallmarks", "RenderWallmarkSetup", "RenderSceneAfterWallmarks",
        "RenderSun", "RenderLightsNormal", "RenderLightsPending", "RenderCombine",
        "RenderForward", "RenderHud",
        "PainterApply", "PainterCapture", "GameThreadTotal", "SchedulerUpdate", "PhysicsStep", "PainterReserved0", "PainterReserved1",
        "SpawnObject", "UpscaleEvaluate",
        "WallmarkResolve", "WallmarkMaskAssembly"
    };
    static_assert(sizeof(names) / sizeof(names[0]) == static_cast<std::size_t>(Stage::Count),
        "Every stage must have a stable CSV name");
    const auto index = static_cast<std::uint32_t>(stage);
    return index < static_cast<std::uint32_t>(Stage::Count) ? names[index] : "Unknown";
}

constexpr std::uint32_t UnknownFrame = 0xffffffffu;
constexpr std::size_t DefaultCapacity = 2097152; // 64 MiB, allocated/touched at start.

struct Event
{
    std::uint64_t beginTicks;
    std::uint64_t durationTicks;
    std::uint32_t threadId;
    std::uint32_t frameId;
    Stage stage;
    std::uint32_t reserved;
};
static_assert(sizeof(Event) == 32, "Keep the capture memory budget explicit");

struct StageSummary
{
    std::uint64_t count;
    std::uint64_t totalTicks;
    std::uint64_t maxTicks;
};

struct CaptureView
{
    const Event* events;
    const StageSummary* stages;
    std::uint64_t count;
    std::uint64_t attempts;
    std::uint64_t overflow;
    std::uint64_t completed;
    std::uint64_t filtered;
    std::uint64_t thresholdTicks;
    std::uint64_t frequency;
    std::uint64_t startTicks;
    std::uint64_t stopTicks;
    std::uint64_t captureId;
    std::size_t capacity;
};

enum class Result { Success, CaptureActive, Busy, NoCapture, AllocationFailed, ClockUnavailable, WriteFailed };

// Clock is a tiny injectable platform adapter, so the actual admission/buffer
// protocol can be stress-tested independently of the renderer and filesystem.
template <class Clock, std::size_t Capacity = DefaultCapacity>
class Recorder
{
    static_assert(Capacity > 0, "A capture needs at least one event");
    static constexpr std::uint64_t Enabled = std::uint64_t(1) << 63;
    static constexpr std::uint64_t Writers = Enabled - 1;

    struct FreeEvents
    {
        void operator()(Event* events) const noexcept { std::free(events); }
    };

    class ControlGuard
    {
        std::atomic_flag& flag;
        const bool acquired;
    public:
        explicit ControlGuard(std::atomic_flag& value) noexcept
            : flag(value), acquired(!flag.test_and_set(std::memory_order_acquire)) {}
        ~ControlGuard() { if (acquired) flag.clear(std::memory_order_release); }
        explicit operator bool() const noexcept { return acquired; }
        ControlGuard(const ControlGuard&) = delete;
        ControlGuard& operator=(const ControlGuard&) = delete;
    };

    // One atomic word closes admission AND retains all in-flight scopes. A
    // separate enabled bool plus writer count would race stop/dump/restart.
    std::atomic<std::uint64_t> gate{0};
    std::atomic<std::uint64_t> next{0};
    std::atomic<std::uint64_t> overflow{0};
    std::atomic_flag control = ATOMIC_FLAG_INIT;
    std::unique_ptr<Event, FreeEvents> events;
    std::uint64_t frequency = 0;
    std::uint64_t startTicks = 0;
    std::uint64_t stopTicks = 0;
    std::uint64_t captureId = 0;
    std::uint64_t thresholdTicks = 0;

    struct AtomicSummary
    {
        std::atomic<std::uint64_t> count{0};
        std::atomic<std::uint64_t> totalTicks{0};
        std::atomic<std::uint64_t> maxTicks{0};
    };
    AtomicSummary statistics[static_cast<std::size_t>(Stage::Count)];

    bool KeepEvent(const Event& event) const noexcept
    {
        return event.stage == Stage::RenderTotal || event.stage == Stage::StaticCreate ||
            event.stage == Stage::BodyCreate || event.stage == Stage::FrameTotal ||
            event.stage == Stage::FrameUpdate || event.stage == Stage::FrameRender ||
            event.stage == Stage::RenderEnd || event.stage == Stage::PresentWait ||
            event.stage == Stage::SecondaryWait || event.stage == Stage::FrameParticles ||
            event.stage == Stage::FrameDeferredCallbacks ||
            // Retain even short coarse phases so a frame's gaps remain measurable.
            (event.stage >= Stage::DeviceBegin && event.stage <= Stage::RenderHud) ||
            event.durationTicks >= thresholdTicks;
    }

public:
    class Scope
    {
        Recorder* owner = nullptr;
        Event event; // Initialized only for an admitted scope.
    public:
        explicit Scope(Recorder& recorder, Stage stage, std::uint32_t frame = UnknownFrame) noexcept
        {
            if (static_cast<std::uint32_t>(stage) >= static_cast<std::uint32_t>(Stage::Count)) return;
            auto state = recorder.gate.load(std::memory_order_relaxed);
            while (state & Enabled)
            {
                if (recorder.gate.compare_exchange_weak(state, state + 1,
                    std::memory_order_acquire, std::memory_order_relaxed))
                {
                    owner = &recorder;
                    event.threadId = Clock::ThreadId();
                    event.frameId = frame;
                    event.stage = stage;
                    event.reserved = 0;
                    event.beginTicks = Clock::Now();
                    return;
                }
            }
        }

        ~Scope() { Stop(); }
        Scope(const Scope&) = delete;
        Scope& operator=(const Scope&) = delete;
        Scope(Scope&&) = delete;
        Scope& operator=(Scope&&) = delete;

        void Stop() noexcept
        {
            if (!owner) return;
            event.durationTicks = Clock::Now() - event.beginTicks;
            auto& summary = owner->statistics[static_cast<std::size_t>(event.stage)];
            summary.count.fetch_add(1, std::memory_order_relaxed);
            summary.totalTicks.fetch_add(event.durationTicks, std::memory_order_relaxed);
            auto previousMax = summary.maxTicks.load(std::memory_order_relaxed);
            while (previousMax < event.durationTicks && !summary.maxTicks.compare_exchange_weak(
                previousMax, event.durationTicks, std::memory_order_relaxed)) {}

            // Summaries include every admitted scope, even when its detailed
            // event is filtered or the fixed event buffer has filled up.
            if (owner->KeepEvent(event))
            {
                const auto index = owner->next.fetch_add(1, std::memory_order_relaxed);
                if (index < Capacity) owner->events.get()[index] = event;
                else owner->overflow.fetch_add(1, std::memory_order_relaxed);
            }
            owner->gate.fetch_sub(1, std::memory_order_release);
            owner = nullptr;
        }
    };

    bool Capturing() const noexcept { return (gate.load(std::memory_order_acquire) & Enabled) != 0; }
    std::uint64_t ActiveWriters() const noexcept { return gate.load(std::memory_order_acquire) & Writers; }

    Result Start()
    {
        ControlGuard guard(control);
        if (!guard) return Result::Busy;
        const auto state = gate.load(std::memory_order_acquire);
        if (state & Enabled) return Result::CaptureActive;
        if (state & Writers) return Result::Busy;
        const auto clockFrequency = Clock::Frequency();
        if (!clockFrequency) return Result::ClockUnavailable;
        if (!events)
        {
            // Use one matched malloc/free pair, independent of xrCore's global
            // new/delete overrides. Value-initialize now to touch every page.
            auto* memory = static_cast<Event*>(std::malloc(sizeof(Event) * Capacity));
            if (!memory) return Result::AllocationFailed;
            for (std::size_t index = 0; index != Capacity; ++index)
                ::new (static_cast<void*>(memory + index)) Event{};
            events.reset(memory);
        }
        next.store(0, std::memory_order_relaxed);
        overflow.store(0, std::memory_order_relaxed);
        frequency = clockFrequency;
        // Ceiling of frequency / 10000: retain non-priority events >= 0.1 ms.
        thresholdTicks = frequency / 10000 + (frequency % 10000 != 0 ? 1 : 0);
        for (auto& summary : statistics)
        {
            summary.count.store(0, std::memory_order_relaxed);
            summary.totalTicks.store(0, std::memory_order_relaxed);
            summary.maxTicks.store(0, std::memory_order_relaxed);
        }
        stopTicks = 0;
        ++captureId;
        startTicks = Clock::Now();
        gate.store(Enabled, std::memory_order_release);
        return Result::Success;
    }

    Result Stop() noexcept
    {
        ControlGuard guard(control);
        if (!guard) return Result::Busy;
        if (!captureId) return Result::NoCapture;
        const auto previous = gate.fetch_and(Writers, std::memory_order_acq_rel);
        if (previous & Enabled) stopTicks = Clock::Now();
        // No wait: a worker may be inside a lock owned by the console thread.
        return Result::Success;
    }

    template <class WriteCapture>
    Result Dump(WriteCapture&& writeCapture)
    {
        ControlGuard guard(control);
        if (!guard) return Result::Busy;
        const auto state = gate.load(std::memory_order_acquire);
        if (state & Enabled) return Result::CaptureActive;
        if (state & Writers) return Result::Busy;
        if (!captureId) return Result::NoCapture;
        const auto attempts = next.load(std::memory_order_relaxed);
        StageSummary summaries[static_cast<std::size_t>(Stage::Count)];
        std::uint64_t completed = 0;
        for (std::size_t stage = 0; stage != static_cast<std::size_t>(Stage::Count); ++stage)
        {
            const auto& summary = statistics[stage];
            summaries[stage] = {summary.count.load(std::memory_order_relaxed),
                summary.totalTicks.load(std::memory_order_relaxed), summary.maxTicks.load(std::memory_order_relaxed)};
            completed += summaries[stage].count;
        }
        const CaptureView view{events.get(), summaries, attempts < Capacity ? attempts : Capacity,
            attempts, overflow.load(std::memory_order_relaxed), completed, completed - attempts,
            thresholdTicks, frequency, startTicks,
            stopTicks, captureId, Capacity};
        // The stopped gate and this control guard keep the buffer immutable
        // throughout explicit file I/O. Exceptions still release the guard.
        return writeCapture(view) ? Result::Success : Result::WriteFailed;
    }
};

#if defined(_WIN32)
struct QpcClock
{
    static std::uint64_t Now() noexcept
    {
        LARGE_INTEGER value;
        QueryPerformanceCounter(&value);
        return static_cast<std::uint64_t>(value.QuadPart);
    }
    static std::uint64_t Frequency() noexcept
    {
        LARGE_INTEGER value;
        return QueryPerformanceFrequency(&value) ? static_cast<std::uint64_t>(value.QuadPart) : 0;
    }
    static std::uint32_t ThreadId() noexcept { return GetCurrentThreadId(); }
};

inline Recorder<QpcClock>& Instance()
{
    static Recorder<QpcClock> recorder;
    return recorder;
}

class Scope : public Recorder<QpcClock>::Scope
{
public:
    explicit Scope(Stage stage, std::uint32_t frame = UnknownFrame) noexcept
        : Recorder<QpcClock>::Scope(Instance(), stage, frame) {}
};
#endif
}
