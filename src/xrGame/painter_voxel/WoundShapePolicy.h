#pragma once
// Deterministic accepted-shot shape policy. Probabilities are artistic feedback tuning.
#include "GoreControls.h"
#include <algorithm>
#include <cstdint>

namespace gfy_live {
struct WoundShapeInput {
    std::uint64_t seed{};
    double radius{},penetration{},exit_mult{1};
    bool head{},fragment{};
    // Negative values request native class defaults; validated explicit zero stays off.
    double cavitation{-1},burst{-1},vent{-1},entry_burst{-1};
    // klass (2026-11-11): Lua's class of the round (0 small arms, 1 rifle, 2 heavy round or slug); -1 = not given: the class is
    // read from the hole size and the nominal reach, as before.
    int round_class{-1};
};
struct WoundShape {
    double radius{},exit_mult{1},entry_mult{1};
    double cavitation{},burst{},vent{},entry_burst{};
};
inline double wound_draw(std::uint64_t seed,unsigned k) noexcept {
    std::uint64_t x=seed*0x9E3779B97F4A7C15ull+k*0xC2B2AE3D27D4EB4Full;
    x^=x>>29;x*=0xBF58476D1CE4E5B9ull;x^=x>>32;
    return double(x>>11)*(1./9007199254740992.);
}
inline double varied(double neutral,double value,double mult) noexcept {
    return mult==1.?value:neutral+(value-neutral)*mult;
}
// Add or thin occurrence with an independent seeded draw; the neutral setting is exact.
inline double occurrence(double value,double base_probability,double mult,double draw,double fallback) noexcept {
    if(mult==1.)return value;
    if(mult<=0.)return 0.;
    if(value>0.)return mult>=1.||draw<mult?value:0.;
    if(mult<=1.||base_probability<=0.||base_probability>=1.)return 0.;
    const double target=std::clamp(base_probability*mult,0.,1.);
    return draw<(target-base_probability)/(1.-base_probability)?fallback:0.;
}
inline WoundShape wound_shape(const WoundShapeInput& in,const GoreControls& c) noexcept {
    WoundShape out;auto h=[&](unsigned k){return wound_draw(in.seed,k);};
    // klass (2026-11-11): the round's class is Lua's when given, not read from a real-size hole. The nominal-reach half of the
    // old rifle test (>= 0.2 m) was a class proxy as well (in.penetration is nominal_penetration_m, the ammo's own reach, not this
    // hit's depth) and is not asked of a given class; the reach tests further down (a body burst from 0.12 m, a vent from 0.06 m)
    // are not class tests and are unchanged. Not given: the old tests, unchanged.
    const bool classed=in.round_class>=0;
    const bool rifle=classed?in.round_class==1:(in.penetration>=.2&&in.radius>=.0075),slug=classed?in.round_class>=2:in.radius>=.012;
    const double p=h(1),entry=p<.6?.9+.25*h(2):(p<.9?1.15+.2*h(2):1.35+.15*h(2));
    out.entry_mult=std::max(0.,varied(1.,entry,c.wound_variation_mult));
    out.radius=std::min(.08,in.radius*out.entry_mult);
    const double raw_exit=std::clamp(in.exit_mult*(in.head?.8+.6*h(3):.75+.4*h(3)),1.,2.);
    const double exit=std::clamp(varied(in.exit_mult,raw_exit,c.wound_variation_mult),1.,2.);
    out.exit_mult=c.exit_tearing_mult==1.?exit:std::clamp(1.+(exit-1.)*c.exit_tearing_mult,1.,2.);
    double cav=in.cavitation;
    if(cav<0.){
        cav=(slug?1.2:(rifle?1.:.6))*std::max(0.,varied(1.,.4+1.2*h(4),c.wound_variation_mult));
        if(h(5)<.15)cav*=varied(1.,2.2,c.wound_variation_mult);
    }
    cav=std::clamp(cav,0.,in.head?2.:1.);
    const double burst_probability=in.head&&(rifle||slug||cav>1.2)?1.:(in.penetration>=.12?.08:0.);
    const double burst_strength=in.head&&(rifle||slug)?varied(.8,.6+.4*h(6),c.wound_variation_mult):
        (in.head&&cav>1.2?varied(.75,.5+.5*h(6),c.wound_variation_mult):varied(.15,.10+.10*h(7),c.wound_variation_mult));
    double burst=in.burst;
    if(burst<0.)burst=burst_probability==1.||h(6)<burst_probability?burst_strength:0.;
    const double pv=in.penetration<.06||cav<1.?0.:(in.head?.15+.45*(cav-1.):.05+.12*(cav-1.));
    double vent=in.vent;
    const double vent_strength=varied(.75,.5+.5*h(9),c.wound_variation_mult);
    if(vent<0.)vent=h(8)<pv?vent_strength:0.;
    const double pe=in.fragment?0.:(slug?.30:(rifle?.20:.12))+(in.head?.10:0.);
    const double entry_strength=in.head?varied(.7,.4+.6*h(11),c.wound_variation_mult):varied(.275,.15+.25*h(11),c.wound_variation_mult);
    double eb=in.entry_burst;
    if(eb<0.)eb=h(10)<pe?entry_strength:0.;
    // Preserve the original native entry-to-exit coupling at 100%, then apply the
    // independent channel gates. No entry effect can override a head blowout Off.
    if(eb>0.)burst=std::max(burst,in.head?std::min(1.,eb+.1):.12+.25*eb);
    const double coupled_p=1.-(1.-burst_probability)*(1.-pe);
    const double burst_p=in.burst>=0.?(in.burst>0.?1.:pe):coupled_p;
    const double burst_fallback=in.head?std::max(burst_strength,std::min(1.,entry_strength+.1)):
        std::max(burst_strength,.12+.25*entry_strength);
    burst=occurrence(burst,burst_p,in.head?c.head_blowout_frequency_mult:c.surface_burst_frequency_mult,h(12),burst_fallback);
    eb=occurrence(eb,in.entry_burst>=0.?(in.entry_burst>0.?1.:0.):pe,c.surface_burst_frequency_mult,h(13),entry_strength);
    vent=occurrence(vent,in.vent>=0.?(in.vent>0.?1.:0.):pv,c.lateral_vent_frequency_mult,h(14),vent_strength);
    out.cavitation=std::clamp(cav*c.cavitation_mult,0.,in.head?2.:1.);
    out.burst=std::clamp(burst*(in.head?c.head_blowout_intensity_mult:c.surface_burst_intensity_mult),0.,1.);
    out.entry_burst=std::clamp(eb*c.surface_burst_intensity_mult,0.,1.);
    out.vent=std::clamp(vent*c.lateral_vent_intensity_mult,0.,1.);
    return out;
}
}

