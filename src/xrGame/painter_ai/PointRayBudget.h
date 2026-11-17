#pragma once
#include "CombatContract.h"
#include <algorithm>
#include <vector>

namespace painter { namespace combat { namespace engine {
// Fair authority-thread ray grants. One query per actor per frame, explicitly
// capped by settings; this is a work limit, never an actor-admission limit.
class PointRayBudget {
    struct Grant {ActorKey actor;bool used=false;};
    std::vector<Grant> grants_;
    ActorKey after_;std::uint64_t epoch_=0;std::uint32_t frame_=0;bool begun_=false;
    static bool less(ActorKey a,ActorKey b){return a.id<b.id||(a.id==b.id&&a.generation<b.generation);}
public:
    bool needs(std::uint64_t epoch,std::uint32_t frame)const{return !begun_||epoch!=epoch_||frame!=frame_;}
    void begin(std::uint64_t epoch,std::uint32_t frame,std::size_t limit,std::vector<ActorKey> eligible){
        if(!needs(epoch,frame))return;
        if(epoch!=epoch_)after_={};epoch_=epoch;frame_=frame;begun_=true;grants_.clear();
        if(!epoch||!limit)return;
        eligible.erase(std::remove_if(eligible.begin(),eligible.end(),[](ActorKey k){return k.id>=invalid_id||!k.generation;}),eligible.end());
        std::sort(eligible.begin(),eligible.end(),less);eligible.erase(std::unique(eligible.begin(),eligible.end()),eligible.end());
        if(eligible.empty())return;
        const auto start=std::upper_bound(eligible.begin(),eligible.end(),after_,less);
        const auto offset=start==eligible.end()?0:std::size_t(start-eligible.begin());
        const auto count=(std::min)(limit,eligible.size());grants_.reserve(count);
        for(std::size_t i=0;i<count;++i)grants_.push_back({eligible[(offset+i)%eligible.size()],false});
        after_=grants_.back().actor;
    }
    bool take(ActorKey actor){for(auto& g:grants_)if(g.actor==actor){if(g.used)return false;g.used=true;return true;}return false;}
};
}}}
