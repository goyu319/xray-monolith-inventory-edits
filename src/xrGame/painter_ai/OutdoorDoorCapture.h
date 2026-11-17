#pragma once
#include "EngineOutdoorSettings.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
namespace painter { namespace combat { namespace engine {
struct OutdoorDoorCaptureStats {std::size_t scanned=0,queries=0,rows_examined=0,invalid=0,conflicts=0;bool limited=false;};
// Authority-thread helper. Cursors rotate over the current captured roster; no
// copied object pointers, past Door rows, or inferred open doors survive frames.
// Existing native nearest-door search is one indivisible engine call. These
// quotas bound calls/metadata/output, not its internal execution time.
class OutdoorDoorCapture {
    std::uint64_t epoch_=0,sequence_=0;std::size_t after_=0;
public:
    void reset(){epoch_=sequence_=0;after_=0;}
    template<class Capture>OutdoorDoorCaptureStats update(WorldSnapshot&w,const OutdoorDoorPolicy&p,Capture capture){
        OutdoorDoorCaptureStats r;w.doors.clear();
        if(!w.level_epoch||!w.sequence||!std::isfinite(p.radius_m)||p.radius_m<=0||!p.queries_per_capture||!p.actor_scans_per_capture||!p.rows_per_capture)return r;
        if(epoch_!=w.level_epoch){reset();epoch_=w.level_epoch;}
        if(w.sequence<=sequence_)return r;
        sequence_=w.sequence;
        if(w.actors.empty()){after_=0;return r;}
        std::map<std::uint16_t,Door> known;std::set<std::uint16_t> conflict;
        const auto count=(std::min)(w.actors.size(),p.actor_scans_per_capture);
        auto same=[](const Door&a,const Door&b){return a.portal==b.portal&&a.state==b.state&&a.open==b.open&&a.locked==b.locked&&a.held==b.held;};
        std::vector<Door> native;
        for(std::size_t n=0;n<count&&r.queries<p.queries_per_capture&&r.rows_examined<p.rows_per_capture;++n){
            const auto&a=w.actors[after_%w.actors.size()];after_=(after_+1)%w.actors.size();++r.scanned;
            if(!a.alive||!a.ownership_generation||a.key.id==invalid_id||!a.key.generation||a.level_epoch!=w.level_epoch||a.snapshot_seq!=w.sequence||a.observed_at_ms!=w.now_ms||
                !std::isfinite(a.position.x)||!std::isfinite(a.position.y)||!std::isfinite(a.position.z)){++r.invalid;continue;}
            native.clear();++r.queries;capture(a.position,p.radius_m,p.rows_per_capture-r.rows_examined,native);
            // Native capture's candidate vector is temporary; no door state is
            // retained into another capture. A truncated region stays unknown.
            if(native.size()>=p.rows_per_capture-r.rows_examined)r.limited=true;
            for(const auto&d:native){
                if(r.rows_examined>=p.rows_per_capture){r.limited=true;break;}++r.rows_examined;
                if(d.id==invalid_id||!std::isfinite(d.open)||d.open<0||d.open>1||unsigned(d.state)>unsigned(DoorState::closing)){++r.invalid;continue;}
                if(conflict.count(d.id))continue;
                const auto at=known.find(d.id);
                if(at!=known.end()){if(!same(at->second,d)){known.erase(at);conflict.insert(d.id);++r.conflicts;}continue;}
                if(known.size()+conflict.size()>=p.rows_per_capture){r.limited=true;continue;}
                known.emplace(d.id,d);
            }
        }
        r.limited=r.limited||r.scanned<w.actors.size();
        for(const auto&d:known)w.doors.push_back(d.second);
        return r;
    }
};
}}}
