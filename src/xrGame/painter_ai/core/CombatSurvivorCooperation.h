#pragma once
#include "CombatCore.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <set>
#include <vector>

namespace painter { namespace combat { namespace core {
// A social link between two depleted native squads. It never changes a side ID,
// roster, mission, enemy fact, or movement admission. The two leaders must see
// one another before anyone is asked to close, and actual bodies must meet.
struct SurvivorCooperation {
    std::uint32_t first=0,second=0;
    Vec3 first_anchor,second_anchor;
    bool newly_confirmed=false,approaching=false;
};
class CombatSurvivorCooperation {
    struct SideState {
        std::uint32_t faction=0,mission=0;std::uint64_t group=0;
        std::size_t peak=0;std::map<std::uint32_t,std::pair<std::uint64_t,std::uint64_t>> owners; // object ID -> {spawn, ownership}
    };
    struct Link {std::uint32_t first=0,second=0;Vec3 first_anchor,second_anchor;};
    std::uint64_t epoch_=0,map_=0;
    std::map<std::uint32_t,SideState> sides_;
    std::map<std::uint32_t,Link> links_;
    static float flat(Vec3 a,Vec3 b){return std::hypot(a.x-b.x,a.z-b.z);}
    static bool sees(const ActorSnapshot& a,ActorKey b){
        for(std::size_t i=0;i<a.visible_friends.count&&i<a.visible_friends.items.size();++i)
            if(a.visible_friends.items[i]==b)return true;
        return false;
    }
public:
    void reset(){epoch_=map_=0;sides_.clear();links_.clear();}
    std::vector<SurvivorCooperation> update(const WorldSnapshot& w,TimeMs max_age,const std::function<bool()>& cancelled={}){
        std::vector<SurvivorCooperation> out;
        if(!w.level_epoch||!w.sequence||!max_age||(cancelled&&cancelled()))return out;
        const auto prior=*this;
        if(epoch_!=w.level_epoch||map_!=w.map_hash){reset();epoch_=w.level_epoch;map_=w.map_hash;}
        std::map<std::pair<std::uint32_t,std::uint64_t>,const ActorSnapshot*> actors;
        for(const auto& a:w.actors)if(valid_key(a.key)&&a.level_epoch==w.level_epoch&&a.snapshot_seq==w.sequence&&
            a.observed_at_ms<=w.now_ms&&fresh(a.observed_at_ms,w.now_ms,max_age)&&a.alive&&
            (a.flags&flag_admitted)&&a.ownership_generation&&!a.yield_reasons&&finite(a.position))actors[{a.key.id,a.key.generation}]=&a;
        struct Cohort {const SideSnapshot* side=nullptr;std::vector<const ActorSnapshot*> men;};
        std::map<std::uint32_t,Cohort> cohorts;
        std::set<std::uint32_t> present;
        for(const auto& side:w.sides){
            if(!side.side_id||!present.insert(side.side_id).second)continue;
            Cohort c;c.side=&side;
            for(auto key:side.members){auto it=actors.find({key.id,key.generation});if(it!=actors.end()&&it->second->faction==side.faction)c.men.push_back(it->second);}
            auto& state=sides_[side.side_id];
            if(state.group!=side.group||state.faction!=side.faction||state.mission!=side.mission.revision){
                state={};state.group=side.group;state.faction=side.faction;state.mission=side.mission.revision;
                links_.erase(side.side_id);
            }
            for(const auto* man:c.men){auto old=state.owners.find(man->key.id);
                if(old!=state.owners.end()&&old->second!=std::make_pair(man->key.generation,man->ownership_generation))links_.erase(side.side_id);
                state.owners[man->key.id]={man->key.generation,man->ownership_generation};}
            state.peak=std::max(state.peak,c.men.size());cohorts.emplace(side.side_id,std::move(c));
        }
        for(auto it=links_.begin();it!=links_.end();)if(!present.count(it->first)||!present.count(it->second.second)||
            cohorts[it->first].men.empty()||cohorts[it->second.second].men.empty()||
            !links_.count(it->second.first)||!links_.count(it->second.second)||
            cohorts[it->second.first].side->faction!=cohorts[it->second.second].side->faction)
            it=links_.erase(it);else ++it;
        for(auto it=sides_.begin();it!=sides_.end();)if(!present.count(it->first))it=sides_.erase(it);else ++it;
        std::set<std::uint32_t> proposing;
        for(const auto& a:cohorts)for(const auto& b:cohorts){
            if(a.first>=b.first||links_.count(a.first)||links_.count(b.first)||proposing.count(a.first)||proposing.count(b.first))continue;
            const auto& x=a.second;const auto& y=b.second;
            if(x.men.empty()||y.men.empty()||x.side->faction!=y.side->faction||
                (x.side->group&&x.side->group==y.side->group)||
                sides_[a.first].peak<2||sides_[b.first].peak<2||
                x.men.size()>=sides_[a.first].peak||y.men.size()>=sides_[b.first].peak||
                x.men.size()+y.men.size()>8)continue;
            const auto kind_x=x.side->mission.kind,kind_y=y.side->mission.kind;
            const auto defensive=[](MissionKind kind){return kind==MissionKind::none||kind==MissionKind::hold||kind==MissionKind::defend;};
            if(!defensive(kind_x)||!defensive(kind_y)||
                (x.side->mission.building!=invalid_room&&y.side->mission.building!=invalid_room&&
                 x.side->mission.building!=y.side->mission.building))continue;
            const auto* lead_x=x.men.front();const auto* lead_y=y.men.front();
            if(!sees(*lead_x,lead_y->key)||!sees(*lead_y,lead_x->key)||flat(lead_x->position,lead_y->position)>15.f)continue;
            bool arrived=true;
            for(const auto* p:x.men)for(const auto* q:y.men)arrived=arrived&&flat(p->position,q->position)<=3.f;
            if(!arrived){out.push_back({a.first,b.first,lead_x->position,lead_y->position,false,true});
                proposing.insert(a.first);proposing.insert(b.first);continue;}
            Link link{a.first,b.first,lead_x->position,lead_y->position};links_[a.first]=links_[b.first]=link;
            out.push_back({a.first,b.first,link.first_anchor,link.second_anchor,true});
        }
        for(const auto& pair:links_)if(pair.first==pair.second.first&&cohorts.count(pair.second.first)&&cohorts.count(pair.second.second)){
            const auto& link=pair.second;
            if(std::none_of(out.begin(),out.end(),[&](const SurvivorCooperation& x){return x.first==link.first&&x.second==link.second;}))
                out.push_back({link.first,link.second,link.first_anchor,link.second_anchor,false});
        }
        if(cancelled&&cancelled()){*this=prior;return {};}
        return out;
    }
};
}}}
