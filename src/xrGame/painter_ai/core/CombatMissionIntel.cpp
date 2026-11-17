#include "CombatMissionIntel.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace painter { namespace combat { namespace core {
namespace {
struct Cancelled {};
struct KeyLess {bool operator()(ActorKey a,ActorKey b) const {
    return a.id<b.id||(a.id==b.id&&a.generation<b.generation);}};
bool fresh(const ActorSnapshot& a,const WorldSnapshot& w,const Config& c){
    return valid_key(a.key)&&a.alive&&!a.wounded&&
        a.level_epoch==w.level_epoch&&a.snapshot_seq==w.sequence&&a.observed_at_ms<=w.now_ms&&
        w.now_ms-a.observed_at_ms<=c.snapshot_max_age_ms&&finite(a.position);
}
struct IntelIndex {
    bool valid=false,frame_current=false;
    std::map<ActorKey,const ActorSnapshot*,KeyLess> actors;
    std::map<ActorKey,const MemberKnowledge*,KeyLess> members;
    std::size_t actor_rows=0,knowledge_rows=0,member_lookups=0;
};
IntelIndex index_capture(const WorldSnapshot& w,const KnowledgeFrame* knowledge,const std::function<bool()>& cancelled){
    IntelIndex out;unsigned work=0;
    const auto poll=[&](){if(++work>=32){work=0;if(cancelled&&cancelled())throw Cancelled{};}};
    if(cancelled&&cancelled())throw Cancelled{};
    if(!w.level_epoch||!w.sequence)return out;
    std::set<std::uint32_t> actor_ids;
    for(const auto& a:w.actors){poll();++out.actor_rows;
        if(!valid_key(a.key)||!actor_ids.insert(a.key.id).second||!out.actors.emplace(a.key,&a).second)return out;
    }
    out.frame_current=knowledge&&knowledge->accepted&&!knowledge->cancelled&&
        knowledge->level_epoch==w.level_epoch&&knowledge->sequence==w.sequence;
    if(out.frame_current)for(const auto& member:knowledge->members){poll();++out.knowledge_rows;
        if(!valid_key(member.actor)||!out.members.emplace(member.actor,&member).second)return out;
    }
    if(cancelled&&cancelled())throw Cancelled{};
    out.valid=true;return out;
}
CombatMissionIntel indexed_intelligence(const Config& c,const WorldSnapshot& w,const SideSnapshot& side,
    IntelIndex& index,const EngineServices& s,const std::function<bool()>& cancelled){
    CombatMissionIntel out;
    unsigned work=0;const auto poll=[&](){if(++work>=32){work=0;if(cancelled&&cancelled())throw Cancelled{};}};
    try {
        if(cancelled&&cancelled())throw Cancelled{};
        if(!index.valid||!side.side_id||side.mission.rooms.count>side.mission.rooms.items.size()||
            static_cast<unsigned>(side.mission.kind)>static_cast<unsigned>(MissionKind::hold)||
            !std::isfinite(c.knowledge.project_horizontal_m)||c.knowledge.project_horizontal_m<0||
            !std::isfinite(c.knowledge.project_vertical_m)||c.knowledge.project_vertical_m<0)return out;
        out.level_epoch=w.level_epoch;out.map_hash=w.map_hash;out.geometry_revision=w.geometry_revision;
        out.snapshot_seq=w.sequence;out.side=side.side_id;out.faction=side.faction;out.group=side.group;out.requested=side.mission;
        const auto& actors=index.actors;
        std::set<ActorKey,KeyLess> roster;
        for(auto key:side.members){poll();++index.member_lookups;if(!valid_key(key)||!roster.insert(key).second)return CombatMissionIntel{};
            const auto it=actors.find(key);if(it==actors.end())continue;
            const auto& a=*it->second;if(a.faction!=side.faction)return CombatMissionIntel{};
            if(!fresh(a,w,c))continue;
            // An actual member position is a useful anchor; averaging across
            // rooms/storeys could put it in a wall or between floors.
            if(!out.anchor_valid){out.anchor_valid=true;out.anchor_member=a.key;out.anchor=a.position;}
        }
        float best_distance=0;
        if(index.frame_current)for(auto key:side.members){poll();++index.member_lookups;const auto it=actors.find(key);if(it==actors.end())continue;
            const auto& a=*it->second;if(!fresh(a,w,c))continue;
            const auto found=index.members.find(a.key);
            if(found==index.members.end())continue;const auto& member=*found->second;
            if(member.side!=side.side_id||member.ownership_generation!=a.ownership_generation||!member.alive||!member.able)continue;
            KnowledgeTrace trace;if(!tactical_knowledge_focus(a,w,member,trace)||!finite(trace.position)||
                trace.observed_at_ms>w.now_ms)continue;
            const float distance=distance_squared(out.anchor,trace.position);
            if(!out.threat_valid||(trace.personally_visible&&!out.threat.personally_visible)||
                (trace.personally_visible==out.threat.personally_visible&&
                    (trace.observed_at_ms>out.threat.observed_at_ms||
                    (trace.observed_at_ms==out.threat.observed_at_ms&&distance<best_distance)))){
                out.threat_valid=true;out.threat=trace;best_distance=distance;
            }
        }
        const bool geometry_current=s.buildings&&s.buildings->locate&&s.buildings->level_epoch==w.level_epoch&&
            s.buildings->map_hash==w.map_hash;
        const auto locate=[&](std::uint32_t node,Vec3 point,bool projected){
            MissionPlace place;place.node=node;place.point=point;place.projected=projected;
            if(!geometry_current||node==invalid_node)return place;
            if(cancelled&&cancelled())throw Cancelled{};
            const auto where=s.buildings->locate(node);
            if(cancelled&&cancelled())throw Cancelled{};
            for(const auto& b:s.buildings->buildings){poll();if(b.id!=where.first)continue;
                for(const auto& r:b.rooms){poll();if(r.id!=where.second)continue;
                    if(!r.node_ids.empty()&&!std::binary_search(r.node_ids.begin(),r.node_ids.end(),node))return place;
                    place.located=true;place.building=b.id;place.room=r.id;return place;}}
            return place;
        };
        if(out.anchor_valid){const auto* anchor=actors.find(out.anchor_member)->second;
            out.own_place=locate(anchor->node_id,anchor->position,false);}
        if(out.threat_valid){out.unidentified_location=out.threat.unidentified||out.threat.foe.ghost();
            if(geometry_current&&s.project){Vec3 snapped{};
                if(cancelled&&cancelled())throw Cancelled{};
                const auto node=s.project(out.threat.position,c.knowledge.project_horizontal_m,c.knowledge.project_vertical_m,&snapped);
                if(cancelled&&cancelled())throw Cancelled{};
                const auto dx=snapped.x-out.threat.position.x,dz=snapped.z-out.threat.position.z;
                if(node!=invalid_node&&finite(snapped)&&dx*dx+dz*dz<=c.knowledge.project_horizontal_m*c.knowledge.project_horizontal_m&&
                    std::abs(snapped.y-out.threat.position.y)<=c.knowledge.project_vertical_m)
                    out.threat_place=locate(node,snapped,true);
            }
            // An unidentified sound/ghost can orient the side, but its guessed
            // point alone never queues a building-clearing objective.
            if(out.threat_place.located&&!out.unidentified_location&&
                !(side.mission.kind==MissionKind::clear_building&&side.mission.building==out.threat_place.building)&&
                (!out.own_place.located||out.own_place.building!=out.threat_place.building)){
                out.pending=MissionKind::clear_building;out.pending_building=out.threat_place.building;
            }
        }
        if(cancelled&&cancelled())throw Cancelled{};
        out.status=MissionIntelStatus::ready;return out;
    }catch(const Cancelled&){CombatMissionIntel stopped;stopped.status=MissionIntelStatus::cancelled;return stopped;}
    catch(...){CombatMissionIntel failed;failed.status=MissionIntelStatus::service_failure;return failed;}
}
}
CombatMissionIntel mission_intelligence(const Config& c,const WorldSnapshot& w,const SideSnapshot& side,
    const KnowledgeFrame* knowledge,const EngineServices& s,const std::function<bool()>& cancelled){
    try{auto index=index_capture(w,knowledge,cancelled);return indexed_intelligence(c,w,side,index,s,cancelled);}
    catch(const Cancelled&){CombatMissionIntel out;out.status=MissionIntelStatus::cancelled;return out;}
    catch(...){CombatMissionIntel out;out.status=MissionIntelStatus::service_failure;return out;}
}
MissionIntelligenceBatch mission_intelligence_batch(const Config& c,const WorldSnapshot& w,
    const KnowledgeFrame* knowledge,const EngineServices& s,const std::function<bool()>& cancelled){
    try{
        auto index=index_capture(w,knowledge,cancelled);MissionIntelligenceBatch out;
        if(!index.valid)return out;
        out.sides.reserve(w.sides.size());
        for(const auto& side:w.sides){
            if(cancelled&&cancelled())throw Cancelled{};
            auto value=indexed_intelligence(c,w,side,index,s,cancelled);
            if(value.status==MissionIntelStatus::cancelled)throw Cancelled{};
            out.sides.push_back(std::move(value));
        }
        if(cancelled&&cancelled())throw Cancelled{};
        out.status=MissionIntelStatus::ready;out.actor_rows_indexed=index.actor_rows;
        out.knowledge_rows_indexed=index.knowledge_rows;out.member_lookups=index.member_lookups;return out;
    }catch(const Cancelled&){MissionIntelligenceBatch out;out.status=MissionIntelStatus::cancelled;return out;}
    catch(...){MissionIntelligenceBatch out;out.status=MissionIntelStatus::service_failure;return out;}
}
}}}
