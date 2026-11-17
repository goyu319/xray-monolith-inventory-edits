#pragma once
#include "CombatCore.h"
#include <functional>
#include <vector>
namespace painter { namespace combat { namespace core {
struct RoomGeometryStamp {
    std::uint64_t epoch=0,map=0,geometry=0,data_version=0;
    std::uint32_t building=invalid_room,room=invalid_room;
    bool operator==(const RoomGeometryStamp& b)const{return epoch==b.epoch&&map==b.map&&geometry==b.geometry&&data_version==b.data_version&&building==b.building&&room==b.room;}
};
struct RoomGeometryPoint {std::uint32_t node=invalid_node,sector=0;Vec3 position;float distance_squared=0;};
struct RoomGeometryResult {
    bool prepared=false,cancelled=false,invalid=false;
    std::size_t total=0,processed=0,valid=0,work=0;
    // A few actual in-room samples, not proof of every nav node or hidden area.
    std::vector<RoomGeometryPoint> samples,candidates;
};
// Static geometry intake only. Each update queries at most max_nodes entries of
// the target room's own node list. Candidate memory is bounded by authored
// anchor count * Config.room.alternative_slots, independent of room node count.
class RoomGeometry {
public:
    RoomGeometryResult step(const RoomGeometryStamp&,const Room&,const RoomConfig&,
        const EngineServices&,std::size_t max_nodes,const std::function<bool()>& cancelled={});
    const RoomGeometryResult& state()const{return result_;}
    void reset();
private:
    RoomGeometryResult step_impl(const RoomGeometryStamp&,const Room&,const RoomConfig&,
        const EngineServices&,std::size_t,const std::function<bool()>&);
    RoomGeometryStamp stamp_;bool bound_=false;
    RoomGeometryResult result_;std::vector<Vec3> anchors_;
    std::vector<std::vector<RoomGeometryPoint>> best_;
    std::uint32_t previous_=invalid_node;
};
}}}
