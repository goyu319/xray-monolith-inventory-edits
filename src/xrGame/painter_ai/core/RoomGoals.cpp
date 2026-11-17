#include "RoomGoals.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>
#include <utility>

namespace painter { namespace combat { namespace core {
namespace {
bool finite(float x) { return std::isfinite(x); }
bool finite(const Vec3& p) { return finite(p.x)&&finite(p.y)&&finite(p.z); }
bool frame_valid(const RoomDoorFrame& f) {
    return finite(f.centre)&&finite(f.tangent)&&finite(f.normal)&&finite(f.half_width)&&finite(f.thickness)
        && f.tangent.y==0 && f.normal.y==0 && f.half_width>=0 && f.thickness>=0;
}
double flat2(const Vec3& a,const Vec3& b) {
    const double x=double(a.x)-b.x,z=double(a.z)-b.z;
    return x*x+z*z;
}
using Key=std::tuple<float,float,float,float,float>;
Key key(const Vec3& p,const AuthoredRoomGoal& r) {
    return std::make_tuple(p.x,p.y,p.z,r.max_horizontal_m,r.max_vertical_m);
}
bool in_bounds(const Vec3& p,const Vec3& snapped,const AuthoredRoomGoal& r) {
    return finite(snapped) && flat2(p,snapped)<=double(r.max_horizontal_m)*r.max_horizontal_m
        && std::abs(double(p.y)-snapped.y)<=r.max_vertical_m;
}
RoomGoalValue geometry(const AuthoredRoomGoal& r,const RoomConfig& c) {
    RoomGoalValue v;
    v.kind=r.kind;v.role=r.role;v.room=r.room;v.has_look=r.has_look;v.look=r.look;
    v.authored_position=r.from_portal_uv?room_frame_world(r.portal,r.u,r.v,r.floor_y):r.position;
    v.position=v.authored_position;
    v.door_bound=r.door_bound;
    // Exact goal44, door_goal1721 and base_order1819 defaults.
    if(r.door_bound) {
        v.face_movement=r.kind==OrderKind::stack||r.kind==OrderKind::entry;
        if(r.geometry_enabled) {
            v.has_portal=r.has_portal;v.portal=r.portal;
            if(r.kind==OrderKind::stack||r.kind==OrderKind::peek||r.kind==OrderKind::hold_door||r.kind==OrderKind::entry)
                v.radius_m=r.kind==OrderKind::entry?c.entry_arrival_m:c.arrival_m;
        }
    }
    return v;
}
bool valid(const AuthoredRoomGoal& r,const RoomGoalValue& v) {
    return static_cast<unsigned>(r.kind)<=static_cast<unsigned>(OrderKind::yield)
        && finite(v.authored_position) && (!r.has_look||finite(r.look))
        && finite(r.max_horizontal_m)&&finite(r.max_vertical_m)&&r.max_horizontal_m>=0&&r.max_vertical_m>=0
        && (!r.from_portal_uv||(r.has_portal&&finite(r.u)&&finite(r.v)&&finite(r.floor_y)))
        && (!(r.has_portal&&(r.geometry_enabled||r.from_portal_uv))||frame_valid(r.portal));
}
}

Vec3 room_frame_world(const RoomDoorFrame& f,float u,float v,float y) {
    return {f.centre.x+f.tangent.x*u+f.normal.x*v,y,f.centre.z+f.tangent.z*u+f.normal.z*v};
}
Vec3 room_frame_local(const RoomDoorFrame& f,const Vec3& p) {
    const float x=p.x-f.centre.x,z=p.z-f.centre.z;
    return {x*f.tangent.x+z*f.tangent.z,0,x*f.normal.x+z*f.normal.z};
}

struct RoomGoalCache::Impl {
    struct Value {std::uint32_t node;Vec3 position;};
    std::map<Key,Value> values;
    RoomGoalStamp stamp;
    std::size_t limit=4096;
    bool bound=false;
    void bind(const RoomGoalStamp& s) {
        if(!bound||s!=stamp) {values.clear();stamp=s;bound=true;}
    }
};
RoomGoalCache::RoomGoalCache(std::size_t n):impl_(new Impl) {impl_->limit=n;}
RoomGoalCache::~RoomGoalCache()=default;
RoomGoalCache::RoomGoalCache(RoomGoalCache&&) noexcept=default;
RoomGoalCache& RoomGoalCache::operator=(RoomGoalCache&&) noexcept=default;
void RoomGoalCache::clear() {impl_->values.clear();impl_->bound=false;}
std::size_t RoomGoalCache::size() const {return impl_->values.size();}

RoomGoalJob::RoomGoalJob(RoomGoalStamp s,std::vector<AuthoredRoomGoal> r,RoomConfig c):config_(c),stamp_(s),requests_(std::move(r)),results_(requests_.size()) {
    if(requests_.empty())status_=RoomGoalJobStatus::Complete;
}
void RoomGoalJob::cancel() {if(status_==RoomGoalJobStatus::Pending)status_=RoomGoalJobStatus::Cancelled;}
std::size_t RoomGoalJob::step(std::size_t max_work,const RoomGoalStamp& current,RoomGoalCache& cache,
    NodeProjectFn project,NodePositionFn position,bool (*deadline)(void*),void* context) {
    if(status_!=RoomGoalJobStatus::Pending)return 0;
    if(current!=stamp_) {status_=RoomGoalJobStatus::Invalidated;return 0;}
    auto& c=*cache.impl_;c.bind(stamp_);
    std::size_t work=0;
    while(next_<requests_.size()&&work<max_work) {
        if(deadline&&deadline(context))break;
        const auto& r=requests_[next_];auto& out=results_[next_];
        out.stamp=stamp_;out.value=geometry(r,config_);
        if(!valid(r,out.value)||!finite(out.value.radius_m)||out.value.radius_m<=0)out.status=RoomGoalStatus::InvalidInput;
        else {
            const Vec3 p=out.value.authored_position;
            const auto k=key(p,r);
            const auto found=c.values.find(k);
            Vec3 snapped{};std::uint32_t node=invalid_node;
            bool fault=false;
            if(found!=c.values.end()) {node=found->second.node;snapped=found->second.position;out.cache_hit=true;}
            else if(!project)fault=true;
            else {
                try {
                    node=project(p,r.max_horizontal_m,r.max_vertical_m,&snapped);
                    if(node!=invalid_node&&!in_bounds(p,snapped,r))fault=true;
                    if(node!=invalid_node&&!fault&&position) {
                        Vec3 verified{};
                        if(!position(node,&verified)||!finite(verified)||flat2(verified,snapped)>1.e-8
                            ||std::abs(double(verified.y)-snapped.y)>1.e-4)fault=true;
                    }
                } catch(...) {fault=true;}
                if(!fault&&node!=invalid_node&&c.limit) {
                    if(c.values.size()>=c.limit)c.values.clear();
                    c.values.emplace(k,RoomGoalCache::Impl::Value{node,snapped});
                }
            }
            if(fault)out.status=RoomGoalStatus::ServiceFault;
            else if(node==invalid_node||(r.from_portal_uv&&std::abs(double(snapped.y)-r.floor_y)>=.5))out.status=RoomGoalStatus::NoNode;
            else {out.value.position=snapped;out.value.node=node;out.status=RoomGoalStatus::Ready;}
        }
        ++next_;++work;
    }
    if(next_==requests_.size())status_=RoomGoalJobStatus::Complete;
    return work;
}

}}}
