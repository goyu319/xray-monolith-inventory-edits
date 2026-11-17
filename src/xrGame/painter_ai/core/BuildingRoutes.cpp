#include "BuildingRoutes.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <tuple>
#include <unordered_map>

namespace painter { namespace combat { namespace core {
namespace {
constexpr std::size_t absent=(std::numeric_limits<std::size_t>::max)();
std::uint64_t side_key(std::uint32_t from,std::uint32_t id) {
    return (std::uint64_t(from)<<32)|id;
}
bool same_vec(const Vec3& a,const Vec3& b){return a.x==b.x && a.y==b.y && a.z==b.z;}
bool same_input(const BuildingInput& a,const BuildingInput& b) {
    const auto& x=a.building;const auto& y=b.building;
    if(a.stamp!=b.stamp || x.id!=y.id || x.rooms.size()!=y.rooms.size() || a.nodes.size()!=b.nodes.size() || x.portals.size()!=y.portals.size())return false;
    for(std::size_t i=0;i<x.rooms.size();++i) {
        const auto& u=x.rooms[i];const auto& v=y.rooms[i];
        if(u.id!=v.id || !same_vec(u.centre,v.centre) || u.floor_y!=v.floor_y || u.nodes!=v.nodes || u.portals!=v.portals)return false;
    }
    for(std::size_t i=0;i<a.nodes.size();++i)if(a.nodes[i].id!=b.nodes[i].id || a.nodes[i].room!=b.nodes[i].room)return false;
    for(std::size_t i=0;i<x.portals.size();++i) {
        const auto& u=x.portals[i];const auto& v=y.portals[i];
        if(u.id!=v.id || u.room_a!=v.room_a || u.room_b!=v.room_b || u.doorway!=v.doorway || u.traversable!=v.traversable ||
            u.stair!=v.stair || !same_vec(u.centre,v.centre) || !same_vec(u.normal,v.normal) ||
            !same_vec(u.jamb_a,v.jamb_a) || !same_vec(u.jamb_b,v.jamb_b) || u.half_width!=v.half_width ||
            u.wall_depth!=v.wall_depth || u.y0!=v.y0 || u.y1!=v.y1 || u.door!=v.door)return false;
    }
    return true;
}
bool valid_state(SideState state) {
    return state==SideState::Unknown || state==SideState::DynamicUnknown || state==SideState::Open ||
        state==SideState::ClosedOpenable || state==SideState::Blocked;
}
}

bool GraphStamp::operator==(const GraphStamp& b) const {
    return level_epoch==b.level_epoch && map_hash==b.map_hash && geometry_revision==b.geometry_revision && building_id==b.building_id;
}
bool GraphStamp::operator<(const GraphStamp& b) const {
    return std::tie(level_epoch,map_hash,geometry_revision,building_id)<std::tie(b.level_epoch,b.map_hash,b.geometry_revision,b.building_id);
}
bool QueryStamp::operator==(const QueryStamp& b) const {
    return graph==b.graph && knowledge_revision==b.knowledge_revision;
}
struct BuildingGraph::Data {
    BuildingInput input;
    std::unordered_map<std::uint32_t,std::size_t> rooms;
    std::unordered_map<std::uint64_t,std::size_t> sides;
    std::unordered_map<std::uint32_t,std::size_t> nodes;
    std::vector<PortalSide> portals;
    std::vector<std::vector<std::size_t>> adjacency;
};
BuildingGraph::BuildingGraph(std::shared_ptr<const Data> data):data_(std::move(data)){}
const GraphStamp& BuildingGraph::stamp() const{return data_->input.stamp;}
std::size_t BuildingGraph::room_count() const{return data_->input.building.rooms.size();}
std::size_t BuildingGraph::portal_count() const{return data_->portals.size();}
std::uint32_t BuildingGraph::room_for_node(std::uint32_t id) const {
    const auto it=data_->nodes.find(id);
    return it==data_->nodes.end()?invalid_room:data_->input.nodes[it->second].room;
}
CacheResult BuildingRouteCache::insert(BuildingInput input) {
    CacheResult out;
    if(!input.stamp.level_epoch || !input.stamp.map_hash || input.stamp.building_id!=input.building.id) {
        out.error="missing graph epoch/map hash or mismatched building id";return out;
    }
    const auto old=graphs_.find(input.stamp);
    if(old!=graphs_.end()) {
        if(!same_input(old->second->data_->input,input)){out.error="graph changed without a new stamp";return out;}
        out.graph=old->second;out.reused=true;return out;
    }
    auto data=std::make_shared<BuildingGraph::Data>();data->input=std::move(input);
    for(std::size_t i=0;i<data->input.building.rooms.size();++i) {
        const auto id=data->input.building.rooms[i].id;
        if(id==invalid_room || !data->rooms.emplace(id,i).second){out.error="invalid/duplicate room";return out;}
    }
    data->adjacency.resize(data->input.building.rooms.size());
    for(std::size_t i=0;i<data->input.nodes.size();++i) {
        const auto& node=data->input.nodes[i];
        if(node.id==no_node || !data->rooms.count(node.room) || !data->nodes.emplace(node.id,i).second) {
            out.error="invalid/duplicate room node";return out;
        }
    }
    std::unordered_map<std::uint32_t,const Portal*> catalog;
    for(const auto& p:data->input.building.portals) {
        if(p.id==invalid_room || !catalog.emplace(p.id,&p).second){out.error="invalid/duplicate portal id";return out;}
    }
    // Canonical Room::portals provides the exact directed side order. Do not
    // automatically add the other side if it is absent from that room's list.
    for(std::size_t room=0;room<data->input.building.rooms.size();++room) {
        const auto& r=data->input.building.rooms[room];
        for(const auto id:r.portals) {
            const auto p=catalog.find(id);
            if(p==catalog.end() || (p->second->room_a!=r.id && p->second->room_b!=r.id)) {
                out.error="room references missing or unrelated portal";return out;
            }
            const auto index=data->portals.size();
            if(!data->sides.emplace(side_key(r.id,id),index).second){out.error="duplicate portal side";return out;}
            const auto& v=*p->second;
            data->portals.push_back({id,r.id,v.room_a==r.id?v.room_b:v.room_a,no_node,no_node,v.traversable&&v.doorway});
            data->adjacency[room].push_back(index);
        }
    }
    out.graph=std::shared_ptr<const BuildingGraph>(new BuildingGraph(data));
    graphs_.emplace(out.graph->stamp(),out.graph);return out;
}
void BuildingRouteCache::erase_epoch(std::uint64_t epoch) {
    for(auto it=graphs_.begin();it!=graphs_.end();)if(it->first.level_epoch==epoch)it=graphs_.erase(it);else ++it;
}
void BuildingRouteCache::clear(){graphs_.clear();}

struct RouteQuery::Impl {
    std::shared_ptr<const BuildingGraph> graph;
    RouteRequest request;
    RouteResult result;
    std::vector<SideState> evidence;
    std::vector<bool> searched;
    std::vector<double> distance;
    std::vector<std::size_t> previous, via;
    struct Entry {double cost;std::uint64_t sequence;std::size_t room;};
    struct Later {bool operator()(const Entry& a,const Entry& b)const {
        return a.cost!=b.cost?a.cost>b.cost:a.sequence>b.sequence;
    }};
    std::priority_queue<Entry,std::vector<Entry>,Later> frontier;
    std::vector<Entry> fifo;
    std::size_t fifo_head=0;
    bool weighted=false;
    std::size_t start=absent,goal=absent,avoid=absent,active=absent,edge=0,reconstruct=absent;
    std::uint64_t sequence=0;
    bool reversing=false;
    std::size_t reverse_left=0,reverse_right=0;

    Impl(std::shared_ptr<const BuildingGraph> g,RouteRequest r):graph(std::move(g)),request(std::move(r)) {
        result.stamp=request.stamp;
        auto fail=[&](const char* message){result.status=RouteStatus::InvalidInput;result.error=message;};
        if(!graph || graph->stamp()!=request.stamp.graph || !request.stamp.knowledge_revision ||
            !std::isfinite(request.unknown_cost) || request.unknown_cost<0 ||
            !std::isfinite(request.closed_cost) || request.closed_cost<0) {fail("invalid graph/query stamp or cost");return;}
        const auto& data=*graph->data_;const auto from=data.rooms.find(request.from_room),to=data.rooms.find(request.to_room);
        if(from==data.rooms.end() || to==data.rooms.end()){fail("source or destination is not in graph");return;}
        start=from->second;goal=to->second;
        auto avoided=data.rooms.find(request.avoid_room);if(avoided!=data.rooms.end())avoid=avoided->second;
        searched.assign(data.input.building.rooms.size(),false);
        for(const auto& id:request.searched_rooms){auto it=data.rooms.find(id);if(it!=data.rooms.end())searched[it->second]=true;}
        evidence.assign(data.portals.size(),SideState::Unknown);
        std::vector<bool> seen(evidence.size(),false);
        for(const auto& e:request.evidence) {
            const auto side=data.sides.find(side_key(e.from_room,e.portal_id));
            if(side==data.sides.end() || seen[side->second] || !valid_state(e.state)){fail("invalid/duplicate side evidence");return;}
            seen[side->second]=true;evidence[side->second]=e.state;
        }
        const auto n=data.input.building.rooms.size();
        distance.assign(n,(std::numeric_limits<double>::infinity)());previous.assign(n,absent);via.assign(n,absent);
        weighted=request.unknown_cost!=0 || request.closed_cost!=0;
        if(!weighted)fifo.reserve(n);
        distance[start]=0;push({0,sequence++,start});
    }
    void push(Entry item){if(weighted)frontier.push(item);else fifo.push_back(item);}
    bool empty() const{return weighted?frontier.empty():fifo_head==fifo.size();}
    Entry take(){if(!weighted)return fifo[fifo_head++];const auto item=frontier.top();frontier.pop();return item;}
    void finish() {
        result.status=RouteStatus::Found;result.cost=distance[goal];result.all_sides_known_open=true;
        // Evidence flags were accumulated one step at a time during unwind.
        result.all_sides_known_open=!result.requires_open&&!result.requires_investigation;
        if(!result.steps.empty()){result.next_portal=result.steps.front().portal;result.next_room=result.steps.front().room;}
    }
    void unit() {
        const auto& data=*graph->data_;
        if(reversing) {
            if(reverse_left<reverse_right){std::swap(result.steps[reverse_left++],result.steps[reverse_right--]);return;}
            finish();return;
        }
        if(reconstruct!=absent) {
            if(reconstruct==start) {
                reversing=true;reverse_left=0;reverse_right=result.steps.empty()?0:result.steps.size()-1;return;
            }
            const auto pidx=via[reconstruct];const auto& p=data.portals[pidx];const auto state=evidence[pidx];
            result.steps.push_back({p.from_room,p.to_room,p.id,p.from_node,p.to_node,state});
            result.requires_investigation|=state==SideState::Unknown || state==SideState::DynamicUnknown;
            result.requires_open|=state==SideState::ClosedOpenable;
            reconstruct=previous[reconstruct];return;
        }
        if(active==absent) {
            if(empty()){result.status=RouteStatus::Unreachable;return;}
            const auto top=take();
            if(top.cost!=distance[top.room])return;
            if(top.room==goal){reconstruct=goal;return;}
            active=top.room;edge=0;return;
        }
        if(edge==data.adjacency[active].size()){active=absent;return;}
        const auto pidx=data.adjacency[active][edge++];const auto& p=data.portals[pidx];
        const auto next=data.rooms.find(p.to_room);
        if(next==data.rooms.end() || next->second==avoid || (!searched[next->second] && next->second!=goal))return;
        const auto state=evidence[pidx];
        if(!p.traversable || state==SideState::Blocked){result.blocked_sides_encountered=true;return;}
        const double penalty=(state==SideState::Unknown || state==SideState::DynamicUnknown)?request.unknown_cost:
            state==SideState::ClosedOpenable?request.closed_cost:0;
        const double cost=distance[active]+1+penalty;
        if(!std::isfinite(cost)){result.status=RouteStatus::InvalidInput;result.error="route cost overflow";return;}
        if(cost>=distance[next->second])return;
        distance[next->second]=cost;previous[next->second]=active;via[next->second]=pidx;
        push({cost,sequence++,next->second});
    }
};
RouteQuery::RouteQuery(std::shared_ptr<const BuildingGraph> graph,RouteRequest request):impl_(new Impl(std::move(graph),std::move(request))){}
RouteQuery::~RouteQuery()=default;
RouteQuery::RouteQuery(RouteQuery&&) noexcept=default;
RouteQuery& RouteQuery::operator=(RouteQuery&&) noexcept=default;
std::size_t RouteQuery::step(std::size_t max_work,const QueryStamp& current,bool (*deadline)(void*),void* context) {
    auto& q=*impl_;
    if(current!=q.result.stamp) {
        q.result.status=RouteStatus::Invalidated;q.result.steps.clear();q.result.next_room=invalid_room;q.result.next_portal=invalid_room;
        q.result.all_sides_known_open=false;return 0;
    }
    std::size_t work=0;
    while(q.result.status==RouteStatus::Searching && work<max_work) {
        if(deadline && deadline(context))break;
        q.unit();++work;++q.result.work_done;
    }
    return work;
}
void RouteQuery::cancel() {
    auto& r=impl_->result;
    if(r.status==RouteStatus::Searching){r.status=RouteStatus::Cancelled;r.steps.clear();r.next_room=invalid_room;r.next_portal=invalid_room;r.all_sides_known_open=false;}
}
const RouteResult& RouteQuery::result() const{return impl_->result;}

}}}
