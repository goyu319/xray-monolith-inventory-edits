#include "EngineMissionIntake.h"
#include <algorithm>
#include <cmath>
#include <set>
#include <tuple>
namespace painter { namespace combat { namespace engine {
namespace {
struct Cancelled{};
void poll(const std::function<bool()>& stop){if(stop&&stop())throw Cancelled{};}
bool valid_key(ActorKey k){return k.id!=invalid_id&&k.generation!=0;}
bool valid_mission(const Mission& m){
    if(!m.revision||static_cast<unsigned>(m.kind)>static_cast<unsigned>(MissionKind::hold)||m.rooms.count>m.rooms.items.size())return false;
    std::set<std::uint32_t> rooms;
    for(std::size_t i=0;i<m.rooms.count;++i)if(m.rooms.items[i]==invalid_room||!rooms.insert(m.rooms.items[i]).second)return false;
    return true;
}
bool same_mission(const Mission& a,const Mission& b){
    if(a.kind!=b.kind||a.building!=b.building||a.entry_portal!=b.entry_portal||a.revision!=b.revision||a.rooms.count!=b.rooms.count)return false;
    for(std::size_t i=0;i<a.rooms.count;++i)if(a.rooms.items[i]!=b.rooms.items[i])return false;
    return true;
}
}
void MissionIntake::reset(){groups_.clear();epoch_=map_=sequence_=now_=0;next_=1;}
MissionIntakeResult MissionIntake::update(WorldSnapshot& w,MissionLookup lookup,TimeMs retain_ms,void* context,const std::function<bool()>& stop){
    MissionIntakeResult out;
    try{
        poll(stop);
        if(!lookup||!w.level_epoch||!w.sequence)return out;
        if(epoch_&&(w.level_epoch<epoch_||(w.level_epoch==epoch_&&(w.sequence<=sequence_||w.now_ms<now_)))){out.status=MissionIntakeStatus::stale;return out;}
        const bool changed=epoch_!=w.level_epoch||map_!=w.map_hash;
        auto state=changed?std::map<Group,Entry>{}:groups_;auto next=changed?1u:next_;
        std::map<std::uint32_t,const ActorSnapshot*> actors;
        std::map<Group,std::vector<ActorKey>> captured;
        for(const auto& a:w.actors){poll(stop);
            if(!valid_key(a.key)||a.level_epoch!=w.level_epoch||a.snapshot_seq!=w.sequence||a.observed_at_ms>w.now_ms||!actors.emplace(a.key.id,&a).second)return out;
            if(a.native_squad)captured[{a.faction,a.native_squad}].push_back(a.key);
        }
        std::set<std::uint32_t> used;
        std::set<std::pair<std::uint32_t,std::uint64_t>> already_assigned;
        for(const auto& side:w.sides){poll(stop);if(!side.side_id||!used.insert(side.side_id).second)return out;
            for(const auto key:side.members){poll(stop);if(!valid_key(key)||!already_assigned.emplace(key.id,key.generation).second)return out;}}
        for(const auto& group:state){poll(stop);if(group.second.side&&!used.insert(group.second.side).second)return out;}
        // Retained identity has the original observed clock. Merely emitting
        // the roster again never renews absence into current knowledge.
        for(auto& group:state){poll(stop);auto& roster=group.second.roster;
            roster.erase(std::remove_if(roster.begin(),roster.end(),[&](const KnownMember& member){poll(stop);auto a=actors.find(member.key.id);
                return member.known_at_ms>w.now_ms||w.now_ms-member.known_at_ms>retain_ms||
                    (a!=actors.end()&&(a->second->key!=member.key||a->second->faction!=group.first.first||a->second->native_squad!=group.first.second));}),roster.end());}
        for(auto& group:captured){poll(stop);auto& entry=state[group.first];
            // New arrivals do not erase a genuinely recent missing comrade.
            // Its own observation clock still expires, even for reused groups.
            std::sort(group.second.begin(),group.second.end(),[](ActorKey a,ActorKey b){return std::tie(a.id,a.generation)<std::tie(b.id,b.generation);});
            for(auto key:group.second){poll(stop);const auto observed=actors.at(key.id)->observed_at_ms;
                if(observed>w.now_ms||w.now_ms-observed>retain_ms)continue;
                auto member=std::find_if(entry.roster.begin(),entry.roster.end(),[&](const KnownMember& m){return m.key==key;});
                if(member==entry.roster.end())entry.roster.push_back({key,observed});else member->known_at_ms=observed;}}
        std::map<std::uint64_t,std::pair<bool,Mission>> missions;
        auto sides=w.sides;
        for(auto& group:state){poll(stop);auto& entry=group.second;if(entry.roster.empty())continue;
            auto found=missions.find(group.first.second);
            if(found==missions.end()){Mission m;const bool present=lookup(context,group.first.second,m);++out.lookup_count;
                if(present&&!valid_mission(m))return out;found=missions.emplace(group.first.second,std::make_pair(present,m)).first;}
            if(!found->second.first)continue;
            const auto& m=found->second.second;
            if(entry.have_mission&&(m.revision<entry.mission.revision||(m.revision==entry.mission.revision&&!same_mission(m,entry.mission))))return out;
            if(!entry.side){while(next<0x80000000u&&used.count(next)){poll(stop);++next;}
                if(next>=0x80000000u){out.status=MissionIntakeStatus::exhausted;return out;}entry.side=next++;used.insert(entry.side);}
            SideSnapshot side;side.side_id=entry.side;side.faction=group.first.first;side.group=group.first.second;side.mission=m;
            side.building_id=m.kind==MissionKind::none?invalid_room:m.building;
            // A changed mission does not re-authorize a historical absent
            // member under its new revision. Current captures stay authoritative.
            if(entry.have_mission&&m.revision!=entry.mission.revision)
                entry.roster.erase(std::remove_if(entry.roster.begin(),entry.roster.end(),[&](const KnownMember& member){poll(stop);
                    auto actor=actors.find(member.key.id);return actor==actors.end()||actor->second->key!=member.key;}),entry.roster.end());
            for(const auto& member:entry.roster){poll(stop);const auto key=member.key;
                if(!already_assigned.emplace(key.id,key.generation).second)return out;
                side.members.push_back(key);if(actors.find(key.id)==actors.end())++out.retained_absent;}
            entry.mission=m;entry.have_mission=true;sides.push_back(std::move(side));++out.mission_sides;
        }
        poll(stop);groups_=std::move(state);next_=next;epoch_=w.level_epoch;map_=w.map_hash;sequence_=w.sequence;now_=w.now_ms;w.sides=std::move(sides);
        out.status=MissionIntakeStatus::ready;return out;
    }catch(const Cancelled&){out.status=MissionIntakeStatus::cancelled;return out;}
    catch(...){out.status=MissionIntakeStatus::failed;return out;}
}
}}}
