#pragma once
#include "CombatKnowledge.h"
#include "CombatCore.h"
#include "../EngineServices.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace painter { namespace combat { namespace core {
struct RoomKnownFoe { FoeId id; Vec3 point; TimeMs at=0; bool visible=false; };
// Reconcile each source before applying the room boundary. Otherwise an old
// in-room report survives a newer sighting of that same enemy in another room.
// These are side-member observations/reports, never queries of enemy objects.
inline std::vector<RoomKnownFoe> room_known_foes(const WorldSnapshot& w,
    const SideSnapshot& side,const KnowledgeFrame* knowledge,const Room& room,
    const Config& config,const std::function<void()>& poll)
{
    struct Fact { RoomKnownFoe foe; bool active=true; };
    std::vector<Fact> facts;
    const auto put=[&](FoeId id,Vec3 point,TimeMs at,bool visible,bool active) {
        if(!id.valid()||!fresh(at,w.now_ms,config.room.known_enemy_ms)||(active&&!finite(point)))return;
        auto found=std::find_if(facts.begin(),facts.end(),[&](const Fact& f){return f.foe.id==id;});
        if(found==facts.end()){facts.push_back({{id,point,at,visible},active});return;}
        if(at>found->foe.at||(at==found->foe.at&&
            ((!active&&found->active)||(active==found->active&&visible&&!found->foe.visible))))
            *found={{id,point,at,visible},active};
    };
    for(auto key:side.members){
        poll();
        const auto found=std::find_if(w.actors.begin(),w.actors.end(),[&](const ActorSnapshot& a){return a.key==key;});
        if(found==w.actors.end())continue;
        const auto& a=*found;
        if(!a.alive||a.wounded||a.level_epoch!=w.level_epoch||a.snapshot_seq!=w.sequence||
            !fresh(a.observed_at_ms,w.now_ms,config.snapshot_max_age_ms))continue;
        if(knowledge&&knowledge->accepted&&!knowledge->cancelled&&knowledge->level_epoch==w.level_epoch&&knowledge->sequence<=w.sequence)
            for(const auto& member:knowledge->members){
                poll();
                if(member.actor!=key||member.side!=side.side_id||member.ownership_generation!=a.ownership_generation)continue;
                for(const auto& foe:member.foes){
                    poll();
                    if(foe.down&&foe.down_ms<=a.observed_at_ms)put(foe.id,{},foe.down_ms,false,false);
                    else if(foe.wounded&&foe.wounded_ms<=a.observed_at_ms)put(foe.id,{},foe.wounded_ms,false,false);
                    if(const auto* fact=foe.last_known())if(fact->at_ms<=a.observed_at_ms)
                        put(foe.id,fact->position,fact->at_ms,false,true);
                }
                break;
            }
        const auto contact=[&](const Contact& c){
            if(!valid_key(c.key)||!c.hostile||c.observed_at_ms>a.observed_at_ms)return;
            put({c.key,0,0},c.position,c.observed_at_ms,personally_visible(c,a),c.alive_when_observed&&!c.wounded);
        };
        contact(a.enemy);
        if(a.visible_enemies.count<=a.visible_enemies.items.size())
            for(std::size_t i=0;i<a.visible_enemies.count;++i)contact(a.visible_enemies.items[i]);
    }
    std::vector<RoomKnownFoe> result;
    for(const auto& fact:facts){
        const auto& p=fact.foe.point;
        if(fact.active&&p.x>=room.min.x&&p.x<=room.max.x&&p.z>=room.min.z&&p.z<=room.max.z&&
            std::abs(p.y-room.floor_y)<=config.knowledge.eye_m+config.room.floor_tolerance_m)result.push_back(fact.foe);
    }
    return result;
}
}}}
