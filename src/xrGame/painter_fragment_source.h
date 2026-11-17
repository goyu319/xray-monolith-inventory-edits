#pragma once
#include <array>
#include <cstdint>
#include <cstring>
#include <cmath>

// Local-only emitted-fragment provenance. Never appended to PAM1 or a game
// packet; it crosses the local packet boundary through an exact delivery ledger.
struct PainterFragmentSource {
    std::uint64_t world=0,cookie=0,target_birth=0;
    std::uint16_t explosive=65535,initiator=65535;
    std::array<float,3> centre{};float radius=0;
    std::array<char,128> section{};std::uint32_t emitted_at=0;
    bool valid()const noexcept {
        if(!world||!cookie||explosive==65535||!section[0]||!std::memchr(section.data(),0,section.size())||
           !std::isfinite(radius)||radius<=0||radius>1000)return false;
        for(float v:centre)if(!std::isfinite(v))return false;return true;
    }
};
namespace painter_fragment_source {
inline thread_local const PainterFragmentSource* current=nullptr;
struct Scope {
    const PainterFragmentSource* previous;
    explicit Scope(const PainterFragmentSource* source)noexcept:previous(current){current=source&&source->valid()?source:nullptr;}
    ~Scope(){current=previous;}Scope(const Scope&)=delete;Scope& operator=(const Scope&)=delete;
};
inline PainterFragmentSource captured()noexcept{return current?*current:PainterFragmentSource{};}
}
