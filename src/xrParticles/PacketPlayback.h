#pragma once

// Staged opt-in particle packet playback. No engine/game installation.
#include <atomic>
#include <cstdint>

class ParticlePacketPlayback
{
    enum Command : std::uint64_t { None, Packet, Play, StopImmediate, StopDeferred };
    static constexpr std::uint64_t commandMask = 7;
    static constexpr std::uint64_t hudMask = 8;
    static constexpr std::uint64_t claimedMask = 16;
    static constexpr std::uint64_t clearMask = 32;
    std::atomic<std::uint64_t> pending{0};
    std::atomic<bool> managed{false};

    static std::uint64_t Encode(Command command, std::uint32_t time, bool hud = false)
    {
        return (std::uint64_t(time) << 32) | command | (hud ? hudMask : 0);
    }

    std::uint64_t Claim()
    {
        auto request = pending.load(std::memory_order_acquire);
        while (request && !(request & claimedMask))
        {
            if (pending.compare_exchange_weak(request, request | claimedMask,
                std::memory_order_acq_rel, std::memory_order_acquire))
                return request | claimedMask;
        }
        return 0;
    }

    struct ClaimedRequest
    {
        ParticlePacketPlayback& owner;
        std::uint64_t request;
        ~ClaimedRequest()
        {
            if (request)
                owner.pending.compare_exchange_strong(request, 0,
                    std::memory_order_acq_rel, std::memory_order_acquire);
        }
    };

    void PublishStart(Command command, std::uint32_t time, bool hud)
    {
        auto previous = pending.load(std::memory_order_acquire);
        do
        {
            // Preserve an unclaimed clear when starts coalesce. A claimed clear
            // belongs to the current consumer and precedes any later drain;
            // copying it again could erase particles born after that clear.
        } while (!pending.compare_exchange_weak(previous,
            Encode(command, time, hud) | ((previous & claimedMask) ? 0 : (previous & clearMask)),
            std::memory_order_acq_rel, std::memory_order_acquire));
    }

public:
    bool IsManaged() const { return managed.load(std::memory_order_acquire); }

    // Keep a claimed start visible until its worker call completes. Exchanging
    // it to zero before visual.Play would let Lua cleanup cancel an empty effect.
    // -1 means use the existing visual state; 0/1 are command-level overrides.
    int PlayingHint() const
    {
        const auto command = Command(pending.load(std::memory_order_acquire) & commandMask);
        if (command == Packet || command == Play) return 1;
        if (command == StopImmediate) return 0;
        return -1;
    }

    // First activation is permitted only on a fresh, never-played native object.
    // Its initial transforms must be set before publishing this request.
    void RequestPacket(std::uint32_t time, bool hud)
    {
        PublishStart(Packet, time, hud);
        managed.store(true, std::memory_order_release);
    }

    void RequestPlay(std::uint32_t time, bool hud)
    {
        PublishStart(Play, time, hud);
    }

    void RequestStop(bool deferred)
    {
        auto previous = pending.load(std::memory_order_acquire);
        do
        {
            // A queued packet already means prime, then stop emission. Do not
            // silence it before that prime. Immediate stop cancels the request.
            if (deferred && (previous & commandMask) == Packet)
                return;
        } while (!pending.compare_exchange_weak(previous,
            Encode(deferred ? StopDeferred : StopImmediate, 0) |
                (deferred ? ((previous & claimedMask) ? 0 : (previous & clearMask)) : clearMask),
            std::memory_order_acq_rel, std::memory_order_acquire));
    }

    // One consumer: existing particle-update owner. Start requests arriving
    // during simulation remain queued for its next turn. Stop requests may be
    // consumed after priming, before catch-up; no unbounded command draining.
    template<class Visual, class Alive>
    void Advance(Visual& visual, std::uint32_t now, std::uint32_t& lastTime, Alive alive)
    {
        if (!alive())
        {
            pending.store(0, std::memory_order_release);
            return;
        }
        const auto request = Claim();
        ClaimedRequest complete{*this, request};
        const auto command = Command(request & commandMask);
        if ((request & clearMask) && command != StopImmediate)
            visual.Stop(false);
        if (command == Packet || command == Play)
        {
            if (request & hudMask)
                visual.SetHudMode(true);
            visual.Play();
            lastTime = std::uint32_t(request >> 32);
            if (command == Packet)
            {
                // Preserve the legacy call boundary, not a guessed frame delay.
                // Keep the legacy per-call three-step cap and emission-stop
                // boundary separate from elapsed catch-up.
                visual.OnFrame(33);
                if (!alive()) return;
                visual.Stop(true);
            }
            else
                lastTime -= 33;
        }
        else if (command == StopImmediate || command == StopDeferred)
            visual.Stop(command == StopDeferred);

        if (!alive()) return;
        auto stop = pending.load(std::memory_order_acquire);
        const auto stopCommand = Command(stop & commandMask);
        if (!(stop & claimedMask) &&
            (stopCommand == StopImmediate || stopCommand == StopDeferred) &&
            pending.compare_exchange_strong(stop, stop | claimedMask,
                std::memory_order_acq_rel, std::memory_order_acquire))
        {
            ClaimedRequest stopComplete{*this, stop | claimedMask};
            if ((stop & clearMask) && stopCommand != StopImmediate)
                visual.Stop(false);
            visual.Stop(stopCommand == StopDeferred);
        }

        if (!alive()) return;
        visual.OnFrame(std::uint32_t(now - lastTime));
        lastTime = now;
    }
};
