#pragma once
#include <cstdint>
#include <cstring>
#include <atomic>
#include <cmath>

// Owned fired-cartridge provenance. Unknown for synthetic/legacy/network hits.
// Unique SHit::BulletID already identifies one pellet, not the entire shell.
namespace painter_cartridge {
struct Scope {
    const void* cartridge;
    const std::uint64_t id;
    const Scope* previous;
    static inline thread_local const Scope* active = nullptr;
    static inline std::atomic<std::uint64_t> serial{0};
    explicit Scope(const void* source) noexcept
        : cartridge(source), id(serial.fetch_add(1, std::memory_order_relaxed) + 1), previous(active) { active = this; }
    ~Scope() { active = previous; }
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;
    static std::uint64_t for_source(const void* source) noexcept {
        return active && active->cartridge == source ? active->id : 0;
    }
};
}

struct PainterHitAmmo
{
    char section[128]{};
    std::uint64_t shot_id=0;
    std::uint16_t pellet_count=0;
    float armor_piercing=0.f;
    float impact_power=0.f; // native outgoing hit power, before target/Lua modifiers

    void clear() { *this=PainterHitAmmo{}; }
    bool valid() const
    {
        const void* end=std::memchr(section,0,sizeof(section));
        return section[0] && end && pellet_count>0 &&
            std::isfinite(armor_piercing) && armor_piercing>=0.f &&
            std::isfinite(impact_power) && impact_power>=0.f;
    }
    bool assign(const char* name,int pellets,float ap,float power)
    {
        clear();
        if(!name || pellets<1 || pellets>65535) return false;
        std::size_t n=0;
        while(n<sizeof(section) && name[n]) ++n;
        if(n==0 || n==sizeof(section)) return false;
        std::memcpy(section,name,n);section[n]=0;
        pellet_count=static_cast<std::uint16_t>(pellets);
        armor_piercing=ap;impact_power=power;
        if(!valid()){clear();return false;}
        return true;
    }
};

namespace painter_hit_ammo_wire {
constexpr std::uint32_t magic=0x314d4150; // PAM1, little-endian engine packet
constexpr std::uint8_t version=1;
constexpr std::uint8_t shot_version=2;
constexpr std::uint16_t fixed_payload=11; // count + AP + power + section byte length
template<class Packet> void write(Packet& p,const PainterHitAmmo& ammo)
{
    if(!ammo.valid()) return;
    const auto n=static_cast<std::uint8_t>(std::strlen(ammo.section));
    p.w_u32(magic);p.w_u8(ammo.shot_id ? shot_version : version);
    p.w_u16(static_cast<std::uint16_t>(fixed_payload+n+(ammo.shot_id ? 8 : 0)));
    p.w_u16(ammo.pellet_count);p.w_float(ammo.armor_piercing);p.w_float(ammo.impact_power);
    p.w_u8(n);p.w(ammo.section,n);
    if(ammo.shot_id) p.w_u64(ammo.shot_id);
}
template<class Packet> bool read(Packet& p,PainterHitAmmo& out)
{
    out.clear();
    if(p.r_elapsed()<7) return false;
    std::uint32_t tag=0;std::uint8_t revision=0;std::uint16_t bytes=0;
    p.r_u32(tag);p.r_u8(revision);p.r_u16(bytes);
    const unsigned tail=revision==shot_version ? 8u : 0u;
    if(tag!=magic || (revision!=version && revision!=shot_version) ||
       bytes<fixed_payload+1+tail || bytes>fixed_payload+127+tail || p.r_elapsed()!=bytes) return false;
    PainterHitAmmo candidate;std::uint8_t n=0;
    p.r_u16(candidate.pellet_count);p.r_float(candidate.armor_piercing);p.r_float(candidate.impact_power);p.r_u8(n);
    if(n==0 || n>=sizeof(candidate.section) || bytes!=fixed_payload+n+tail || p.r_elapsed()!=n+tail) return false;
    p.r(candidate.section,n);candidate.section[n]=0;
    if(tail){p.r_u64(candidate.shot_id);if(!candidate.shot_id)return false;}
    if(std::memchr(candidate.section,0,n) || !candidate.valid()) return false;
    out=candidate;return true;
}
}

