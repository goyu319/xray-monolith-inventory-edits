#pragma once
#include "../CombatContract.h"
#include <cmath>

namespace painter { namespace combat { namespace core {
// Live1c work-order G values; named until their proposed v2.25 allocation.
inline constexpr TimeMs look_dwell_ms=1200;
inline constexpr float look_jitter_m=2.f;
using LookIntent=decltype(Order{}.look);

// Own one per native actor/lease. The caller resets on handback/identity/level
// change and supplies event EDGES, not a repeatedly true "at goal" condition.
// prepare is an intent only: revalidate its returned point with current native
// geometry every frame. While retaining(), do not try a different opening.
// commit_safe accepts only the guard's result and latches a path degradation;
// this class never authorizes visibility, fire, geometry, or physical arrival.
class CombatLookCommitment {
    LookIntent safe_{},anchor_{},pending_{},candidate_{},bearing_point_{};
    TimeMs committed_at_=0,last_now_=0,pending_now_=0;
    bool committed_=false,retaining_=false,pending_valid_=false,path_latched_=false,bearing_degraded_=false;
    static LookIntent path(){LookIntent l;l.kind=LookKind::path;return l;}
    static bool finite(Vec3 p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
    static bool exact(const LookIntent&a,const LookIntent&b){return a.kind==b.kind&&a.target==b.target&&
        a.value.x==b.value.x&&a.value.y==b.value.y&&a.value.z==b.value.z;}
    static bool same_target(const LookIntent&a,const LookIntent&b){
        if(a.kind!=b.kind||a.target!=b.target)return false;
        if(a.kind==LookKind::path)return true;
        if(a.kind==LookKind::direction)return exact(a,b);
        const double x=double(a.value.x)-b.value.x,y=double(a.value.y)-b.value.y,z=double(a.value.z)-b.value.z;
        return x*x+y*y+z*z<double(look_jitter_m)*look_jitter_m;
    }
    static LookIntent sanitize(LookIntent look){if(look.kind==LookKind::path)return path();
        if((look.kind!=LookKind::point&&look.kind!=LookKind::direction&&look.kind!=LookKind::enemy)||!finite(look.value))return path();
        return look;}
public:
    void reset(){*this=CombatLookCommitment{};}
    bool has_commit()const{return committed_;}
    bool retaining()const{return retaining_;}
    bool path_latched()const{return path_latched_;}
    bool bearing_degraded()const{return bearing_degraded_;}
    LookIntent prepare(LookIntent desired,TimeMs now,bool visible,bool new_hit,bool arrived,bool crossed_door,bool moving){
        if(committed_&&now<last_now_)reset();
        desired=sanitize(desired);pending_=desired;pending_now_=now;pending_valid_=true;
        const bool interrupt=visible||new_hit||arrived||crossed_door;
        // A stationary path degradation is not a completed look assignment.
        // Retry the requested point after the ordinary dwell so a transient
        // unknown/blocked ray cannot permanently prevent measured SEARCH
        // coverage. The caller must still guard the new candidate; retaining
        // a clear point (including small target jitter) and a moving episode
        // keeps the existing commitment rules.
        const bool degraded=(path_latched_&&safe_.kind==LookKind::path)||bearing_degraded_;
        const bool retry=committed_&&!moving&&degraded&&
            desired.kind!=LookKind::path&&now-committed_at_>=look_dwell_ms;
        retaining_=committed_&&!interrupt&&!retry&&(moving||now-committed_at_<look_dwell_ms||same_target(anchor_,desired));
        // A bearing keeps the original point as its authority. Recheck it
        // from the current eye every frame; do not freeze a world direction.
        candidate_=retaining_?(bearing_degraded_?bearing_point_:safe_):desired;
        return candidate_;
    }
    LookIntent commit_safe(LookIntent guarded,bool bearing_only=false){
        if(!pending_valid_)return committed_?safe_:path();
        guarded=sanitize(guarded);
        // Only the guard may mark a point-to-horizontal-bearing degradation.
        // A different opening still requires a new episode, not a refresh.
        const bool bearing=bearing_only&&guarded.kind==LookKind::direction&&
            guarded.target.id==invalid_id&&guarded.value.y==0.f&&
            double(guarded.value.x)*guarded.value.x+double(guarded.value.z)*guarded.value.z>.00000001&&
            (candidate_.kind==LookKind::point||candidate_.kind==LookKind::enemy);
        if(retaining_&&!exact(guarded,candidate_)&&!bearing)guarded=path();
        if(!retaining_){anchor_=pending_;committed_at_=pending_now_;path_latched_=false;}
        if(guarded.kind==LookKind::path&&pending_.kind!=LookKind::path)path_latched_=true;
        bearing_degraded_=bearing;
        if(bearing)bearing_point_=candidate_;
        safe_=guarded;committed_=true;last_now_=pending_now_;pending_valid_=false;
        return safe_;
    }
};
}}}
