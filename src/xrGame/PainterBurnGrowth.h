#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

// Cosmetic ignition on measured game surfaces. The host traces each candidate against
// world occlusion, original geometry and current ownership before calling this policy.
namespace painter_burn_growth {
using Vec=std::array<float,3>;
struct Candidate {
    Vec rest{},world{},normal{}; // metres; outward rest-space unit normal
    std::uint32_t surface{};     // stable original face or certified current interface
    std::uint16_t child{};
    std::uint8_t domain{};
};
namespace detail {
inline std::uint64_t mix(std::uint64_t x) noexcept {
    x=(x^(x>>30))*UINT64_C(0xbf58476d1ce4e5b9);
    x=(x^(x>>27))*UINT64_C(0x94d049bb133111eb);
    return x^(x>>31);
}
inline float unit(std::uint64_t x) noexcept {return float(mix(x)>>40)*(1.f/16777216.f);}
inline float dot(const Vec& a,const Vec& b) noexcept{return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
inline Vec sub(const Vec& a,const Vec& b) noexcept{return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};}
inline float distance2(const Vec& a,const Vec& b) noexcept {const auto v=sub(a,b);return dot(v,v);}
inline bool usable(const Candidate& c) noexcept {
    if (!c.surface || c.domain<1 || c.domain>3) return false;
    for(unsigned i=0;i<3;++i){
        if (!std::isfinite(c.rest[i]) || !std::isfinite(c.world[i]) || !std::isfinite(c.normal[i]))return false;
        if (std::abs(c.rest[i])>10000.f) return false;
    }
    const float n=dot(c.normal,c.normal);
    return n>.80f && n<1.20f;
}
inline std::uint64_t key(const Candidate& c) noexcept {
    std::uint64_t h=(std::uint64_t(c.surface)<<24)|(std::uint64_t(c.child)<<8)|c.domain;
    for(float p:c.rest) h=mix(h^std::uint32_t(std::int32_t(std::floor(p*500.f))));
    return h;
}
inline bool precedes(const Candidate& a,const Candidate& b) noexcept {
    if(a.surface!=b.surface)return a.surface<b.surface;
    if(a.child!=b.child)return a.child<b.child;
    if(a.domain!=b.domain)return a.domain<b.domain;
    if(a.rest!=b.rest)return a.rest<b.rest;
    if(a.normal!=b.normal)return a.normal<b.normal;
    return a.world<b.world;
}
} // namespace detail

class Sites {
    struct Site {
        Candidate anchor{};
        float radius{},maximum{},rate{},gain{},aspect{};
        std::uint32_t seen{},activeTicks{};
        std::uint64_t salt{};
        bool hadContact{},current{};
    };
    std::array<Site,6> sites_{};
    std::uint64_t emitter_{},owner_{},seed_{};
    std::uint32_t tick_{};
    unsigned size_{},wanted_{};
    bool initialized_{};
    static float metric(const Site& s,const Candidate& c) noexcept {
        const auto& a=s.anchor;
        const float facing=detail::dot(a.normal,c.normal);
        if(facing<.28f)return 1.e20f;
        const auto delta=detail::sub(c.rest,a.rest);
        const float depth=detail::dot(delta,a.normal);
        // Never jump to another garment or the far side of a thin part. An already
        // opened covering can reveal a separately traced surface on a later pulse.
        if(a.domain==c.domain && a.child!=c.child)return 1.e20f;
        float plane=std::max(.010f,s.radius*.40f);
        if(a.domain!=c.domain)plane=std::min(plane,.027f);
        if(std::abs(depth)>plane)return 1.e20f;
        const unsigned axis=unsigned(s.salt%3);
        const float strand=delta[axis]-depth*a.normal[axis];
        return detail::dot(delta,delta)+s.aspect*strand*strand+2.f*depth*depth;
    }
    void append(const Candidate& c) noexcept {
        auto& s=sites_[size_++];
        s.anchor=c;s.salt=detail::mix(seed_^detail::key(c));
        s.radius=.024f+.019f*detail::unit(s.salt+1);
        s.maximum=.12f+.07f*detail::unit(s.salt+2);
        s.rate=.026f+.032f*detail::unit(s.salt+3); // metres per actual exposure second
        s.gain=.65f+.35f*detail::unit(s.salt+4);
        s.aspect=.65f*detail::unit(s.salt+5);
    }
public:
    template<class Owns>
    Sites retainedBy(std::uint64_t owner,const Owns& owns)const {
        Sites copy=*this;
        copy.owner_=owner;
        copy.size_=0;
        for(unsigned i=0;i<size_;++i)if(owns(sites_[i].anchor))
            copy.sites_[copy.size_++]=sites_[i];
        // Surviving branches can acquire new sites under confirmed exposure.
        return copy;
    }
    template<class Owns>
    void removeInvalid(const Owns& owns) {
        unsigned n=0;for(unsigned i=0;i<size_;++i)if(owns(sites_[i].anchor))sites_[n++]=sites_[i];
        size_=n;
    }
    std::size_t count() const noexcept{return size_;}
    const Candidate& anchor(std::size_t i) const noexcept{return sites_[i].anchor;}
    float radius(std::size_t i) const noexcept{return sites_[i].radius;}
    bool current(std::size_t i) const noexcept{return sites_[i].current;}
    unsigned activeTicks(std::size_t i) const noexcept{return sites_[i].activeTicks;}
    float maxRadius(std::size_t i) const noexcept{return sites_[i].maximum;}
    // One call per complete observed source pulse/owner. No ray samples supplied
    // here are generated or extrapolated. A repeated same-tick pulse does not grow twice.
    bool advance(std::uint64_t emitter,std::uint64_t owner,std::uint32_t tick16,unsigned cadenceTicks,
                 const Candidate* observed,std::size_t count) noexcept {
        if(!emitter || !owner || (!observed && count))return false;
        if(!initialized_){
            emitter_=emitter;owner_=owner;seed_=detail::mix(emitter^detail::mix(owner));
            wanted_=3u+unsigned(seed_&3u);tick_=tick16;initialized_=true;
        }
        if(emitter!=emitter_ || owner!=owner_ || tick16-tick_>=0x80000000u)return false;
        tick_=tick16;
        for(auto& s:sites_)s.current=false;
        if(count==0){for(auto& s:sites_)s.hadContact=false;return false;}
        // Seed once, independently of input order. Use actual separation on both
        // the posed target and its authored surface; repeated samples cannot make extra sites.
        while(size_<wanted_){
            const Candidate* best=nullptr;float score=-1.f;
            const auto salt=detail::mix(seed_+size_);
            for(std::size_t i=0;i<count;++i){
                const auto& c=observed[i];
                if(!detail::usable(c))continue;
                float gap=.033f; // cap the spacing reward at ~18 cm, retaining randomness across wider areas
                for(unsigned j=0;j<size_;++j){
                    gap=std::min(gap,detail::distance2(c.rest,sites_[j].anchor.rest));
                    gap=std::min(gap,detail::distance2(c.world,sites_[j].anchor.world));
                }
                if(gap<.00040f)continue; // a point/small cluster supports fewer than three distinct sites
                const float rank=(.42f+.58f*detail::unit(detail::key(c)^salt))*gap;
                if(!best || rank>score || (rank==score&&detail::precedes(c,*best))){best=&c;score=rank;}
            }
            if(!best)break;
            append(*best);
        }
        cadenceTicks=std::min(16u,std::max(1u,cadenceTicks));
        bool visible=false;
        for(unsigned j=0;j<size_;++j){
            auto& s=sites_[j];
            const Candidate* near=nullptr;float nearest=s.radius*s.radius;
            for(std::size_t i=0;i<count;++i){
                const auto& c=observed[i];if(!detail::usable(c))continue;
                const float distance=metric(s,c);
                if(distance<nearest || (distance==nearest&&(!near||detail::precedes(c,*near)))){nearest=distance;near=&c;}
            }
            if(!near){s.hadContact=false;continue;}
            s.current=true;visible=true;
            // A guide for the next ray fan only. Deposits always require a fresh nearest hit.
            if(near->domain==s.anchor.domain && nearest<=.000225f)s.anchor.world=near->world;
            const unsigned gap=tick16-s.seen;
            if(s.hadContact && gap && gap<=cadenceTicks+1){
                // Depend only on exposure intervals actually corroborated by both endpoints.
                const unsigned steps=std::min(cadenceTicks,gap);
                for(unsigned step=0;step<steps;++step){
                    const unsigned cadence=2u+unsigned((s.salt>>8)%5);
                    const float scale=.62f+.75f*detail::unit(s.salt+(s.activeTicks++/cadence));
                    s.radius=std::min(s.maximum,s.radius+s.rate*scale/16.f);
                }
            }
            s.seen=tick16;s.hadContact=true;
        }
        return visible;
    }
    // Weight only the actual nearest surface candidate from THIS pulse. No mark is
    // written by this policy, and sleeping/cooling never advances it.
    float weight(const Candidate& c) const noexcept {
        if(!detail::usable(c))return 0.f;
        float strongest=0.f;
        for(unsigned j=0;j<size_;++j){
            const auto& s=sites_[j];if(!s.current)continue;
            const float q=metric(s,c)/(s.radius*s.radius);
            if(!(q<1.f))continue;
            const float edge=std::min(1.f,(1.f-q)*2.25f);
            strongest=std::max(strongest,s.gain*edge*edge*(3.f-2.f*edge));
        }
        return strongest;
    }
};
} // namespace painter_burn_growth

