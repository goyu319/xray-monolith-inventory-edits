#pragma once
#include "../xrServerEntities/alife_space.h"
#include <algorithm>
#include <atomic>
class CObject;
#include <cmath>
#include <cstdint>

// xrGame boundary. Source callbacks and clock accesses run on the simulation thread.
// submit must consume the event synchronously or retain its owned values. No object
// pointer, Lua value, scene query result, GPU resource or health callback crosses it.
extern int painter_burn_enabled;
namespace painter_burn_source {
inline bool enabled() noexcept{return painter_burn_enabled!=0;}
enum class Cause:unsigned char { Unknown, Zone, Flame, Local, Fuel, NativeFire, MolotovPrimary, MolotovArea, MolotovAttached };
inline const char* causeName(Cause c) noexcept {
    switch(c) {
    case Cause::Zone:return "thermal_zone"; case Cause::Flame:return "flame_stream";
    case Cause::Local:return "thermal_hit"; case Cause::Fuel:return "fuel_explosion";
    case Cause::NativeFire:return "thermal_explosive"; case Cause::MolotovPrimary:return "molotov_primary";
    case Cause::MolotovArea:return "molotov_area"; case Cause::MolotovAttached:return "molotov_attached";
    default:return "unclassified";
    }
}
enum class Shape:unsigned char { Sphere, Box, Segment, LocalBone, AttachedSurface };
struct Event {
    Shape shape{};
    bool confirmedBody{};unsigned long long targetWorld{};bool witnessOnly{};unsigned long long witnessBirth{};
    Cause cause{};
    unsigned char lightBurn{};
    unsigned char falloffPower=1; unsigned short areaPeriodMs{};
    unsigned short emitterId=0xffff,targetId=0xffff,bone=0xffff;
    unsigned long long sourceSerial{},pulse{};
    unsigned tick16{},quanta{},shapeIndex{},shapeCount{1};
    float power{},attenuation{};
    Fvector centre{},direction{},axis[3]{},half{};
    float radius{},reach{};
};
void submit(const Event&) noexcept;
bool isBody(CObject&) noexcept;
void confirm(CObject&,Event) noexcept;

inline unsigned long long nextSerial() noexcept {
    static std::atomic<unsigned long long> seq{1};
    return seq.fetch_add(1,std::memory_order_relaxed);
}
inline unsigned tick16(unsigned deviceMs) noexcept {
    static unsigned last{};
    static unsigned long long elapsed{};
    static bool ready{};
    if(!ready){ready=true;last=deviceMs;}
    const unsigned d=deviceMs-last;
    if(d<0x80000000u)elapsed+=d;
    last=deviceMs; // a level/simulation timer reset establishes its new epoch without a heat jump
    return unsigned(elapsed*16/1000);
}
struct Gate {
    unsigned long long lifetime{};
    unsigned lastTick{};
    bool valid{};
    void stop() noexcept{valid=false;}
    bool take(unsigned now,Event& e) noexcept{
        const unsigned t=tick16(now);
        if(!lifetime)lifetime=nextSerial();
        const unsigned gap=t-lastTick;
        if(valid&&(gap<4||gap>=0x80000000u))return false;
        e.tick16=t;e.quanta=valid&&gap<=8?std::min(4u,gap):1;
        valid=true;lastTick=t;e.sourceSerial=lifetime;e.pulse=nextSerial();return true;
    }
};
inline bool finite(const Fvector& p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
inline bool thermal(ALife::EHitType t) {return t==ALife::eHitTypeBurn||t==ALife::eHitTypeLightBurn;}
}
