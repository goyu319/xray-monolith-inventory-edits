#pragma once
#include <cmath>
#include <cstdint>
#include <algorithm>

namespace PainterFireLighting
{
constexpr unsigned slots = 32; // firetrack: was 8 (firelightsmall), 4
constexpr unsigned facePixels = 96;
constexpr unsigned fineFacePixels = 160;
struct Site
{
    float x, y, z;
};
inline float distance2(Site a, Site b)
{
    const float x=a.x-b.x, y=a.y-b.y, z=a.z-b.z;
    return x*x+y*y+z*z;
}
struct Source
{
    Site p{};
    float intensity=0.f, range=0.f;
    std::uint64_t group=0, key=0;
    bool area=false;
};
inline Source spread(Site p, float amount, float flicker, std::uint64_t group)
{
    const float r=std::sqrt(std::max(amount,0.f));
    // firelightreach (2026-11-23): a big fire lights far around it; strength and reach grow with the flame group
    // firelightsmall: a small fire lights its surroundings too (only embers fade to nothing)
    // firelightold: the f14z45 reach and brightness (red matched), the small-fire ramp kept
    return {p, .42f*std::min(.8f+.5f*r,3.f)*flicker*std::min(1.f,8.f*amount), // firelightwarm: ~60 %, capped (was .67 x up to 4.5)
        std::min(6.25f+5.f*r,45.f), group}; // firelightwarm: reach +25 %
}
inline Source baked(Site p, float strength, std::uint64_t group)
{
    const float r=std::sqrt(std::max(strength,0.f));
    return {p, .42f*std::min(.45f+strength,3.f)*std::min(1.f,6.f*strength),std::max(5.6f,2.5f+7.5f*r),group}; // firelightwarm: ~60 %, cap 3, reach +25 %
}
class Plan
{
public:
    Source lights[slots]{};
    unsigned count=0;
    unsigned limit=slots; // firetrack: how many lamps are in use (light quality)
    // firelightsteady: last frame's sources keep their places unless a newcomer is clearly stronger
    void begin(Site camera) { eye=camera; heldCount=count; for(unsigned i=0;i<count;i++)held[i]=lights[i].key; count=0; }
    void add(Source s)
    {
        if (!(s.intensity>.002f)) return;
        // Only sources already known to come from one physical fire can share a light.
        if (s.group) for (unsigned i=0;i<count;i++)
        {
            Source& old=lights[i];
            if (old.group!=s.group || distance2(old.p,s.p)>2.25f) continue;
            if (s.intensity>old.intensity) old=s;
            return;
        }
        if(count<limit) {lights[count++]=s; return;}
        unsigned weak=0;
        for (unsigned i=1;i<count;i++) if(score(lights[i])*keep(lights[i])<score(lights[weak])*keep(lights[weak])) weak=i;
        if(score(s)*keep(s)>score(lights[weak])*keep(lights[weak])) lights[weak]=s;
    }
    unsigned shadowTargets(int quality, unsigned previous) const
    {
        // Extreme (3): every lamp casts shadows, out to 35 m
        // firelightfill: the shadowed lamps light the scene - High 2 (20 m), Beautiful 3 (30 m), Extreme 6 (45 m)
        const unsigned budget=quality>=3?6u:quality==2?3u:quality==1?2u:0u;
        const float horizon=quality>=3?45.f:quality==2?30.f:20.f;
        unsigned mask=0;
        for (unsigned n=0;n<budget;n++)
        {
            unsigned best=count;
            float high=0.f;
            for(unsigned i=0;i<count;i++)
            {
                if(mask&(1u<<i) || distance2(lights[i].p,eye)>horizon*horizon) continue;
                const float rank=score(lights[i])*((previous&(1u<<i))?1.20f:1.f);
                if(rank>high){best=i;high=rank;}
            }
            if(best==count)break;
            mask|=1u<<best;
        }
        return mask;
    }
    float strength(unsigned n) const
    {
        const Source& here=lights[n];
        float overlapWeight=1.f;
        for (unsigned i=0;i<count;i++)
        {
            if(i==n)continue;
            // Overlapping sources contribute a bounded share, while distant lights retain their full energy.
            const float d2=distance2(here.p,lights[i].p), r=.55f*std::min(here.range,lights[i].range);
            overlapWeight+= std::max(0.f,1.f-d2/(r*r))*.35f; // firelightold: was .75 (the old lights simply added up)
        }
        return here.intensity / overlapWeight;
    }
public:
    // firelightapproach: how much of lamp source i's light is its shadowed light, by its distance - 0 at the shadow horizon
    // (as shadowTargets, firelightfill), 1 from 40 % of the horizon closer in
    float shadowLevel(unsigned i,int quality) const
    {
        if(i>=count)return 0.f;
        const float horizon=quality>=3?45.f:quality==2?30.f:20.f;
        const float d=std::sqrt(distance2(lights[i].p,eye));
        return std::clamp((horizon-d)/(.4f*horizon),0.f,1.f);
    }
private:
    Site eye{};
    std::uint64_t held[slots]{};
    unsigned heldCount=0;
    float keep(const Source& s) const { for(unsigned i=0;i<heldCount;i++) if(s.key && held[i]==s.key) return 1.6f; return 1.f; }
    float score(const Source& s)const{return s.intensity*s.range*s.range/(4.f+distance2(s.p,eye));}
};
// Keep presentation slots tied to the original fire even if frame submissions reorder.
struct LampSlot
{
    Source source{};
    float intensity=0.f, opacity=0.f, shadow=0.f;
    int input=-1;
    bool occupied=false, wasTarget=false;
};
class Presentation
{
public:
    LampSlot lamps[slots]{};
    void update(const Plan& plan,int quality,float dt)
    {
        const float tick=std::clamp(dt,0.f,.10f);
        bool used[slots]{};
        for(auto& lamp:lamps)lamp.input=-1;
        for(unsigned pass=0;pass<2;pass++)
            for(unsigned i=0;i<plan.count;i++)if(!used[i])
            {
                const auto& s=plan.lights[i];
                LampSlot* match=nullptr;
                float nearest=2.2501f;
                for(auto& lamp:lamps)
                {
                    if(!lamp.occupied || lamp.input>=0 || lamp.source.area!=s.area) continue;
                    if(!pass)
                    {
                        if(lamp.source.key==s.key){match=&lamp;break;}
                    }
                    else if(s.group && s.group==lamp.source.group)
                    {
                        const float d=distance2(s.p,lamp.source.p);
                        if(d<nearest){nearest=d;match=&lamp;}
                    }
                }
                if(match){match->input=int(i);used[i]=true;}
            }
        for(auto& lamp:lamps)if(lamp.occupied && lamp.input<0)
        {
            lamp.opacity=std::max(0.f,lamp.opacity-1.6f*tick);
            if(lamp.opacity<=0.f)lamp=LampSlot{};
        }
        for(unsigned i=0;i<plan.count;i++)if(!used[i])
            for(auto& lamp:lamps)if(!lamp.occupied)
            {
                lamp.occupied=true;lamp.input=int(i);break;
            }
        unsigned previous=0;
        for(auto& lamp:lamps)if(lamp.input>=0 && lamp.wasTarget)previous|=1u<<lamp.input;
        const unsigned targets=plan.shadowTargets(quality,previous);
        for(auto& lamp:lamps)if(lamp.occupied)
        {
            if(lamp.input>=0)
            {
                lamp.source=plan.lights[lamp.input];
                const float target=plan.strength(lamp.input);
                lamp.intensity=lamp.opacity<=0.f?target:lamp.intensity+(target-lamp.intensity)*std::min(1.f,6.5f*tick);
                lamp.opacity=std::min(1.f,lamp.opacity+2.f*tick);
            }
            lamp.wasTarget=lamp.input>=0 && (targets&(1u<<lamp.input));
            // firelightapproach: the shadowed share follows the distance (no pop at the horizon), eased over time
            const float want=lamp.wasTarget?plan.shadowLevel(unsigned(lamp.input),quality):0.f;
            lamp.shadow=std::clamp(lamp.shadow+std::clamp(want-lamp.shadow,-1.5f*tick,1.5f*tick),0.f,1.f);
        }
    }
};

}

