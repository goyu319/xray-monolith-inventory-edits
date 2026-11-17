#pragma once
#include "CombatContract.h"
#include <cmath>
#include <limits>
namespace painter { namespace combat { namespace engine {
// Ranking of CURRENT personal visual observations only. This selects facing;
// weapon/muzzle/relation checks still independently authorize native fire.
class VisibleLookChoice {
    ActorKey authored_,selected_,chosen_;
    unsigned rank_=3;float distance_=(std::numeric_limits<float>::max)();
public:
    VisibleLookChoice(ActorKey authored,ActorKey selected):authored_(authored),selected_(selected){}
    ActorKey target()const{return chosen_;}
    bool consider(ActorKey key,float distance_squared,bool visible,bool hostile,bool alive,bool wounded){
        if(key.id==invalid_id||!key.generation||!visible||!hostile||!alive||wounded||
            !std::isfinite(distance_squared)||distance_squared<0)return false;
        const unsigned rank=key==authored_?0u:key==selected_?1u:2u;
        const bool tie=key.id<chosen_.id||(key.id==chosen_.id&&key.generation<chosen_.generation);
        if(rank>rank_||(rank==rank_&&(distance_squared>distance_||(distance_squared==distance_&&!tie))))return false;
        chosen_=key;rank_=rank;distance_=distance_squared;return true;
    }
};
}}}
