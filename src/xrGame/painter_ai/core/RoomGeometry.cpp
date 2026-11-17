#include "RoomGeometry.h"
#include <algorithm>
#include <cmath>
namespace painter { namespace combat { namespace core {
namespace {
struct Interrupted{};
void poll(const std::function<bool()>& stop){if(stop&&stop())throw Interrupted{};}
bool inside(const Room& r,Vec3 p,float floor_tolerance){return finite(p)&&p.x>=r.min.x&&p.x<=r.max.x&&p.z>=r.min.z&&p.z<=r.max.z&&std::abs(p.y-r.floor_y)<=floor_tolerance;}
bool less(const RoomGeometryPoint& a,const RoomGeometryPoint& b){return a.distance_squared<b.distance_squared||(a.distance_squared==b.distance_squared&&a.node<b.node);}
}
void RoomGeometry::reset(){stamp_={};bound_=false;result_={};anchors_.clear();best_.clear();previous_=invalid_node;}
RoomGeometryResult RoomGeometry::step(const RoomGeometryStamp& stamp,const Room& room,const RoomConfig& config,
    const EngineServices& services,std::size_t max_nodes,const std::function<bool()>& stop){
    try{poll(stop);auto staged=*this;poll(stop);auto result=staged.step_impl(stamp,room,config,services,max_nodes,stop);poll(stop);
        *this=std::move(staged);return result;
    }catch(const Interrupted&){auto out=result_;out.cancelled=true;out.work=0;return out;}
}
RoomGeometryResult RoomGeometry::step_impl(const RoomGeometryStamp& stamp,const Room& room,const RoomConfig& config,
    const EngineServices& services,std::size_t max_nodes,const std::function<bool()>& stop){
    if(!bound_||!(stamp_==stamp)){reset();bound_=true;stamp_=stamp;result_.total=room.node_ids.size();
        anchors_=room.sectors;
        if(anchors_.empty()){const float inset=(std::min)(config.spacing_m,(std::min)(room.max.x-room.min.x,room.max.z-room.min.z)*.2f);
            anchors_={{room.min.x+inset,room.floor_y,room.min.z+inset},{room.max.x-inset,room.floor_y,room.min.z+inset},
                {room.max.x-inset,room.floor_y,room.max.z-inset},{room.min.x+inset,room.floor_y,room.max.z-inset},room.centre};}
        best_.resize(anchors_.size());
    }
    result_.work=0;result_.cancelled=false;
    if(result_.prepared||result_.invalid)return result_;
    if(!stamp.epoch||!stamp.geometry||stamp.room!=room.id||!room.nodes||room.nodes!=room.node_ids.size()||
        !services.node_position||!services.buildings||!services.buildings->locate||!config.alternative_slots||
        !finite(room.min)||!finite(room.max)||!std::isfinite(room.floor_y)||!std::isfinite(config.floor_tolerance_m)||config.floor_tolerance_m<0){result_.invalid=true;return result_;}
    for(auto p:anchors_)if(!finite(p)){result_.invalid=true;return result_;}
    while(result_.processed<room.node_ids.size()&&result_.work<max_nodes){poll(stop);const auto id=room.node_ids[result_.processed];
        if(id==invalid_node||(previous_!=invalid_node&&id<=previous_)){result_.invalid=true;return result_;}
        previous_=id;++result_.processed;++result_.work;Vec3 position;poll(stop);
        if(!services.node_position(id,&position))continue;poll(stop);const auto where=services.buildings->locate(id);poll(stop);
        if(where.first!=stamp.building||where.second!=room.id||!inside(room,position,config.floor_tolerance_m))continue;
        ++result_.valid;
        for(std::size_t i=0;i<anchors_.size();++i){poll(stop);const float x=position.x-anchors_[i].x,z=position.z-anchors_[i].z;
            RoomGeometryPoint candidate{id,static_cast<std::uint32_t>(i+1),position,x*x+z*z};auto& selected=best_[i];
            const auto at=std::lower_bound(selected.begin(),selected.end(),candidate,less);
            if(at==selected.end()&&selected.size()>=config.alternative_slots)continue;
            selected.insert(at,candidate);if(selected.size()>config.alternative_slots)selected.pop_back();}
    }
    if(result_.processed==room.node_ids.size()){
        for(const auto& selected:best_){poll(stop);
            for(const auto& candidate:selected){bool duplicate=false;for(const auto& prior:result_.samples)duplicate|=candidate.node==prior.node;
                if(!duplicate){result_.samples.push_back(candidate);break;}}
            for(const auto& candidate:selected){bool duplicate=false;for(const auto& prior:result_.candidates)duplicate|=candidate.node==prior.node;
                if(!duplicate)result_.candidates.push_back(candidate);}
        }
        result_.prepared=!result_.samples.empty();result_.invalid=!result_.prepared;
    }
    return result_;
}
}}}
