#pragma once
// painter_ai/NearMiss.h - the near-miss measurement for the native combat AI (contract v2.14 ActorSnapshot::near_misses).
// Header-only and engine-free, so it is tested offline (native-seam-tests/test_nearmiss.cpp). EngineNearMiss.cpp feeds
// it from the bullet manager: every stretch a round flies between two collision tests, every man a round strikes, and the
// round's removal; capture takes a man's records.
//
// A round passes a man where its flight comes closest to his head point: inside a stretch, or at the joint of two (a
// ricochet, or the end of one frame's flight). One record per round and man, with that distance, when it is within
// reach. A round that is still closing on him when it stops (a wall, its range) records the distance where it stopped.
// A round that strikes him is a hit and never a near miss: a pass is staged and released only after the round's next
// frame of flight (or its removal), so a strike on the frame after the pass still cancels it. His own rounds never count.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4995)                  // xrCore marks sqrt deprecated (vector.h); the distances need the plain one
#endif

namespace painter { namespace combat { namespace nearmiss {

struct P3 { float x = 0, y = 0, z = 0; };
struct Man { std::uint16_t id = 0; P3 head; };     // a watched man: his head point this frame
struct Record {                                     // one pass, as capture takes it
    std::uint32_t at = 0;                           // level time (ms) of the frame the round passed him
    float miss_m = 0;                               // its closest approach to his head point
    P3 origin;                                      // the shot's muzzle
    std::uint16_t shooter = 0;
    std::uint32_t released = 0;                     // level time it was released to his mailbox
};
struct Counters { std::uint64_t stretches = 0, passes = 0, struck = 0, dropped = 0; };

class Tracker {
public:
    static constexpr std::size_t mailbox_size = 4;  // per man, the newest kept (Bounded<NearMiss, 4>)
    static constexpr std::size_t max_tracks = 1024; // rounds x men under way at once; past it a pass is not recorded

    // One bullet-manager frame. `time` stamps what this frame records and releases; reach = Config::near_miss_m.
    void begin_frame(std::uint32_t time, float reach) { time_ = time; reach_ = reach; ++frame_; }

    // A stretch [a, b] that round flew this frame (no collision inside it; b is the collision point when it hit something).
    void stretch(std::uint32_t round, std::uint16_t shooter, const P3& origin, const P3& a, const P3& b, const Man* men,
                 std::size_t count)
    {
        if (!(reach_ > 0)) return;
        const P3 d{b.x - a.x, b.y - a.y, b.z - a.z};
        const float length2 = d.x * d.x + d.y * d.y + d.z * d.z;
        if (!(length2 > 1e-10f)) return;
        ++counters_.stretches;
        ++serial_;
        const P3 lo{(std::min)(a.x, b.x) - reach_, (std::min)(a.y, b.y) - reach_, (std::min)(a.z, b.z) - reach_};
        const P3 hi{(std::max)(a.x, b.x) + reach_, (std::max)(a.y, b.y) + reach_, (std::max)(a.z, b.z) + reach_};
        for (std::size_t i = 0; i < count; ++i)
        {
            const Man& man = men[i];
            if (man.id == shooter) continue;                      // his own rounds
            const P3& h = man.head;
            if (h.x < lo.x || h.x > hi.x || h.y < lo.y || h.y > hi.y || h.z < lo.z || h.z > hi.z) continue;
            Track* track = tracks_.empty() ? nullptr : find(round, man.id);
            if (track) track->serial = serial_;
            if (track && track->state != pending) continue;      // staged or done: one record per round and man
            const float s = ((h.x - a.x) * d.x + (h.y - a.y) * d.y + (h.z - a.z) * d.z) / length2;
            if (s >= 1.f)                                         // still closing on him at b
            {
                const float miss = distance(h, b);
                if (miss > reach_) continue;
                if (!track && !(track = add(round, man.id, shooter, origin))) continue;
                track->miss_m = miss;                             // pending: the nearest it has come so far
                track->at = time_;
                track->serial = serial_;
            }
            else if (s >= 0.f)                                    // its closest point is on this stretch
            {
                const float miss = distance(h, P3{a.x + d.x * s, a.y + d.y * s, a.z + d.z * s});
                if (miss > reach_) continue;                      // (a pending track is never farther than this)
                if (!track && !(track = add(round, man.id, shooter, origin))) continue;
                track->serial = serial_;
                stage(*track, miss, time_);
            }
            else if (track)                                       // opening from a: it passed at the joint, where the
                stage(*track, track->miss_m, track->at);          // pending distance was taken
        }
        // A pending man this stretch never came within reach of: it turned away at the joint (a man it was closing on
        // cannot leave the reach box while it still closes), so it passed him there.
        for (Track& track : tracks_)
            if (track.round == round && track.state == pending && track.serial != serial_) stage(track, track.miss_m, track.at);
    }

    // The round struck this (watched) man: a hit, never a near miss - whatever was pending or staged is dropped.
    void struck(std::uint32_t round, std::uint16_t man)
    {
        ++counters_.struck;
        if (Track* track = find(round, man)) { track->state = done; return; }
        if (Track* track = add(round, man, 0, P3{})) track->state = done;
    }

    // The round ends this frame: a staged pass is released; a round still closing records where it stopped.
    void removed(std::uint32_t round)
    {
        for (Track& track : tracks_)
            if (track.round == round && track.state != done) deliver(track);
        tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track& t) { return t.round == round; }),
                      tracks_.end());
    }

    // End of the frame: passes staged on an earlier frame are released (every round in flight has flown once since).
    void end_frame()
    {
        for (Track& track : tracks_)
            if (track.state == staged && track.frame < frame_) { deliver(track); track.state = done; }
    }

    // Admission inspects without consuming; capture owns acknowledgement.
    template <class Sink> void peek(std::uint16_t man, std::uint32_t since, Sink&& sink) const
    {
        const auto found = boxes_.find(man);
        if (found == boxes_.end()) return;
        const Box& box = found->second;
        for (std::uint8_t i = 0; i < box.count; ++i)
        {
            const Record& r = box.items[(box.first + i) % mailbox_size];
            if (std::int32_t(r.released - since) >= 0) sink(r);
        }
    }
    template <class Sink> void take(std::uint16_t man, std::uint32_t since, Sink&& sink)
    {
        peek(man,since,sink);boxes_.erase(man);
    }

    void forget(std::uint16_t man)                              // the man is gone: nothing more for him
    {
        boxes_.erase(man);
        for (Track& track : tracks_)
            if (track.man == man) track.state = done;
    }
    void drop_rounds() { tracks_.clear(); }                     // nobody watched: nothing under way
    void reset() { tracks_.clear(); boxes_.clear(); frame_ = 0; counters_ = {}; }   // level load: the counters are per level
    std::size_t tracks() const { return tracks_.size(); }
    const Counters& counters() const { return counters_; }

private:
    enum State : std::uint8_t { pending, staged, done };
    struct Track {
        std::uint32_t round = 0;
        std::uint16_t man = 0, shooter = 0;
        State state = pending;
        std::uint32_t frame = 0, at = 0, serial = 0;
        float miss_m = 0;
        P3 origin;
    };
    struct Box { Record items[mailbox_size]; std::uint8_t first = 0, count = 0; };

    static float distance(const P3& a, const P3& b)
    {
        const float x = a.x - b.x, y = a.y - b.y, z = a.z - b.z;
        return std::sqrt(x * x + y * y + z * z);
    }
    Track* find(std::uint32_t round, std::uint16_t man)
    {
        for (Track& track : tracks_)
            if (track.round == round && track.man == man) return &track;
        return nullptr;
    }
    Track* add(std::uint32_t round, std::uint16_t man, std::uint16_t shooter, const P3& origin)
    {
        if (tracks_.size() >= max_tracks) { ++counters_.dropped; return nullptr; }
        Track track;
        track.round = round;
        track.man = man;
        track.shooter = shooter;
        track.origin = origin;
        tracks_.push_back(track);
        return &tracks_.back();
    }
    void stage(Track& track, float miss, std::uint32_t at)
    {
        track.state = staged;
        track.miss_m = miss;
        track.at = at;
        track.frame = frame_;
    }
    void deliver(const Track& track)
    {
        ++counters_.passes;
        Box& box = boxes_[track.man];
        Record& r = box.count < mailbox_size ? box.items[(box.first + box.count++) % mailbox_size]
                                             : box.items[box.first++ % mailbox_size];   // full: the oldest goes
        box.first %= mailbox_size;
        r.at = track.at;
        r.miss_m = track.miss_m;
        r.origin = track.origin;
        r.shooter = track.shooter;
        r.released = time_;
    }

    std::vector<Track> tracks_;
    std::unordered_map<std::uint16_t, Box> boxes_;
    std::uint32_t time_ = 0, frame_ = 0, serial_ = 0;
    float reach_ = 0;
    Counters counters_;
};

}}} // namespace painter::combat::nearmiss

#ifdef _MSC_VER
#pragma warning(pop)
#endif
