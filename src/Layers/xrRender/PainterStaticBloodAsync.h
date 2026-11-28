#pragma once
#ifdef USE_DX11
#include "PainterStaticBloodPatch.h"
#include "PainterStaticBloodDecode.h"
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <unordered_set>

namespace PainterStaticBlood {
struct Capture {
    Mark mark;
    PainterBloodTextureSource::SnapshotPtr source;
};
using CapturePtr = std::shared_ptr<const Capture>;
using PatchPtr = std::shared_ptr<const Patch>;

struct AsyncJob {
    std::uint64_t epoch = 0;
    std::uint32_t nowMs = 0;
    Config config;
    std::vector<CapturePtr> marks;
    std::vector<PatchPtr> patches;
    bool allowNew = true;
};

struct AsyncResult {
    std::uint64_t epoch = 0;
    PatchPtr base, draft;
    Delta dirty;
    std::vector<Id> members;
    unsigned decodeFailures = 0;
    bool workerFailed = false;
    double workerMs = 0;
};

// A single writer owns the active patch. This mailbox holds only one job or unpublished result.
// The worker may build an entirely new chart or copy a live one and append a dirty rectangle.
// The caller still owns each original wallmark until it validates and publishes the result.
class Async {
public:
    static constexpr unsigned MaxMarks = 512, MaxPatches = 128;
    static constexpr std::size_t MaxPatchBytes = 32u*1024*1024;

    Async() = default;
    Async(const Async&) = delete;
    Async& operator=(const Async&) = delete;
    ~Async() {
        stopping.store(true);
        if (wake) SetEvent(wake);
        if (worker.joinable()) worker.join();
        if (wake) CloseHandle(wake);
    }

    std::uint64_t epoch() const { return generation.load(); }

    // Single owner thread, including reset and destruction. A reset never waits on this worker
    // or releases an in-use decoder. The worker retires the old decoder even when no job follows.
    void reset() {
        generation.fetch_add(1);
        if (wake) SetEvent(wake);
    }

    // Refusal leaves the caller's vectors intact. Construct input vectors from immutable refs.
    bool trySubmit(AsyncJob& j) {
        if (j.epoch != epoch() || j.marks.empty() || j.marks.size() > MaxMarks || j.patches.size() > MaxPatches)
            return false;
        std::unique_lock<std::mutex> hold(mutex, std::try_to_lock);
        if (!hold.owns_lock() || state != State::Idle || j.epoch != epoch()) return false;
        if (!worker.joinable()) {
            if (!wake) wake = CreateEvent(nullptr, FALSE, FALSE, nullptr);
            if (!wake) return false;
            try { worker = std::thread(&Async::run, this); }
            catch (const std::system_error&) { return false; }
        }
        input = std::move(j);
        state = State::Pending;
        SetEvent(wake);
        return true;
    }

    // Supply an empty result. Moving it out only moves references and vector descriptors.
    bool tryPop(AsyncResult& out) {
        std::unique_lock<std::mutex> hold(mutex, std::try_to_lock);
        if (!hold.owns_lock() || state != State::Ready || result.epoch != epoch()) return false;
        out = std::move(result);
        state = State::Idle;
        return true;
    }

private:
    enum class State { Idle, Pending, Working, Ready };
    HANDLE wake = nullptr;
    std::atomic<std::uint64_t> generation{1};
    std::atomic<bool> stopping{false};
    std::mutex mutex;
    State state = State::Idle;
    AsyncJob input;
    AsyncResult result;
    std::thread worker;

    bool current(std::uint64_t e) const { return !stopping.load() && e == epoch(); }

    void bake(const AsyncJob& j, DecoderCache& decoder, AsyncResult& r) const {
        if (!j.allowNew && j.patches.empty()) return;
        Config cfg = j.config;
        cfg.byteCap = std::min(cfg.byteCap, MaxPatchBytes);
        std::vector<Mark> resolved;
        resolved.reserve(j.marks.size());
        std::unordered_set<const PainterBloodTextureSource::Snapshot*> failedSources;
        for (const auto& capture : j.marks) {
            if (!current(j.epoch)) return;
            if (!capture || !capture->mark.id || capture->mark.clipped.empty() ||
                capture->mark.clipped.size() > chart::MaxMarkVertices || capture->mark.receivers.empty() ||
                capture->mark.receivers.size() > chart::MaxReceivers) continue;
            const auto& stamp = capture->mark.stamp;
            if (!stamp.lifetimeMs || (stamp.lifetimeMs != ~0u && j.nowMs - stamp.landedMs >= stamp.lifetimeMs))
                continue;
            auto image = capture->mark.image;
            if (!image) {
                if (capture->source && !failedSources.count(capture->source.get())) {
                    try { image = decoder.get(capture->source); }
                    catch (const std::exception&) { image.reset(); }
                    if (!image) failedSources.insert(capture->source.get());
                }
                if (!image) { ++r.decodeFailures; continue; }
            }
            resolved.push_back(capture->mark);
            resolved.back().image = std::move(image);
        }
        std::vector<const Mark*> sorted;
        sorted.reserve(resolved.size());
        for (const auto& mark : resolved) sorted.push_back(&mark);
        std::stable_sort(sorted.begin(), sorted.end(), [](auto a, auto b) { return a->id < b->id; });

        for (const auto& mark : sorted) {
            for (const auto& patch : j.patches) {
                if (!current(j.epoch)) return;
                if (!patch || patch->material != mark->material || patch->sector != mark->sector ||
                    patch->textureBytes() > MaxPatchBytes) continue;
                Delta dirty;
                if (!prepareAppend(*patch, *mark, j.nowMs, cfg, dirty)) continue;
                auto next = std::make_shared<Patch>(*patch);
                if (!commit(*next, dirty)) throw std::logic_error("static blood append transaction");
                r.members.push_back(mark->id);
                r.base = patch;
                r.draft = std::move(next);
                r.dirty = std::move(dirty);
                return;
            }
        }
        if (!j.allowNew || !current(j.epoch) || sorted.size() < cfg.minimum) return;
        auto next = std::make_shared<Patch>();
        if (build(sorted, j.nowMs, cfg, *next)) {
            r.members = next->ids;
            r.draft = std::move(next);
        }
    }

    void run() {
        SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
        std::unique_ptr<DecoderCache> decoder;
        std::uint64_t decoderEpoch = 0;
        for (;;) {
            if (WaitForSingleObject(wake, INFINITE) != WAIT_OBJECT_0 || stopping.load()) return;
            const auto e = epoch();
            if (decoderEpoch != e) {
                decoder.reset();
                decoderEpoch = e;
            }
            AsyncJob j;
            AsyncResult stale;
            {
                std::lock_guard<std::mutex> hold(mutex);
                if (state == State::Ready && result.epoch != epoch()) {
                    stale = std::move(result);
                    state = State::Idle;
                }
                if (state != State::Pending) continue;
                j = std::move(input);
                state = State::Working;
            }
            AsyncResult out;
            out.epoch = j.epoch;
            const auto start = std::chrono::steady_clock::now();
            try {
                if (current(j.epoch)) {
                    // A reset and a new submission may both land between waking and dequeue.
                    if (decoderEpoch != j.epoch) {
                        decoder.reset();
                        decoderEpoch = j.epoch;
                    }
                    if (!decoder) decoder = std::make_unique<DecoderCache>();
                    bake(j, *decoder, out);
                }
            } catch (const std::exception&) {
                out = AsyncResult{};
                out.epoch = j.epoch;
                out.workerFailed = true;
            }
            out.workerMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-start).count();
            {
                std::lock_guard<std::mutex> hold(mutex);
                if (current(j.epoch)) {
                    result = std::move(out);
                    state = State::Ready;
                } else state = State::Idle;
            }
        }
    }
};
} // namespace PainterStaticBlood
#endif
