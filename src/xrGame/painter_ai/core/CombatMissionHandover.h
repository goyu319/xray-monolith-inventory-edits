#pragma once
#include "CombatOutdoorPosture.h"
#include <algorithm>
#include <map>
#include <limits>

namespace painter { namespace combat { namespace core {
// Authority-owned output of completed posture decisions. A native mission is
// retained across captures, never inferred from a queued or cancelled worker.
class CombatMissionHandover {
    struct Record {SideSnapshot side; std::uint64_t geometry=0;};
    std::map<std::uint32_t,Record> records_;
    std::uint64_t epoch_=0,map_=0;
public:
    void reset(){records_.clear();epoch_=map_=0;}
    void apply(WorldSnapshot& w) {
        if(epoch_!=w.level_epoch||map_!=w.map_hash){reset();epoch_=w.level_epoch;map_=w.map_hash;}
        for(auto it=records_.begin();it!=records_.end();) {
            auto& record=it->second;
            if(record.geometry!=w.geometry_revision){it=records_.erase(it);continue;}
            auto supplied=std::find_if(w.sides.begin(),w.sides.end(),[&](const SideSnapshot& s){
                return s.side_id==record.side.side_id||(s.group&&s.group==record.side.group&&s.faction==record.side.faction);});
            // An explicitly authored mission wins. A changed side identity
            // never inherits the former occupants' room objective.
            if(supplied!=w.sides.end()&&(supplied->faction!=record.side.faction||supplied->group!=record.side.group||
                (supplied->mission.kind!=MissionKind::none&&
                 (supplied->mission.revision!=record.side.mission.revision||supplied->mission.kind!=record.side.mission.kind||
                  supplied->mission.building!=record.side.mission.building||supplied->mission.entry_portal!=record.side.mission.entry_portal)))) {
                it=records_.erase(it);continue;
            }
            SideSnapshot current=record.side;current.members.clear();
            for(const auto key:record.side.members) {
                const auto a=std::find_if(w.actors.begin(),w.actors.end(),[&](const ActorSnapshot& a){return a.key==key;});
                if(a!=w.actors.end()&&a->faction==record.side.faction&&(!a->native_squad||a->native_squad==record.side.group))current.members.push_back(key);
            }
            if(current.members.empty()){it=records_.erase(it);continue;}
            if(supplied==w.sides.end())w.sides.push_back(std::move(current));
            else {supplied->mission=record.side.mission; supplied->building_id=record.side.mission.building;}
            ++it;
        }
    }
    bool commit(const WorldSnapshot& w,const SideSnapshot& side,const OutdoorPostureResult& result) {
        if(w.level_epoch!=epoch_||w.map_hash!=map_||result.status!=OutdoorStatus::ready||
            result.handover==OutdoorHandover::none||result.handover_building==invalid_room||
            result.stamp.level_epoch!=w.level_epoch||result.stamp.snapshot_seq!=w.sequence||
            result.stamp.side!=side.side_id||side.mission.kind!=MissionKind::none||side.members.empty()||
            side.mission.revision==(std::numeric_limits<std::uint32_t>::max)())return false;
        if(records_.count(side.side_id))return false;
        Record record;record.side=side;record.geometry=w.geometry_revision;
        record.side.mission={};record.side.mission.kind=MissionKind::clear_building;
        record.side.mission.building=result.handover_building;
        record.side.mission.entry_portal=result.handover_portal;
        record.side.mission.revision=side.mission.revision+1;
        record.side.building_id=result.handover_building;
        records_.emplace(side.side_id,std::move(record));return true;
    }
};
}}}
