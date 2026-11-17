#include "RoomStack.h"
#include <set>
#include <algorithm>
#include <array>
#include <cstring>
#include <cmath>
#include <limits>
#include <map>

namespace painter { namespace combat { namespace core {
namespace {
struct StackCancelled {};
float flat(const Vec3& a,const Vec3& b) {const float x=a.x-b.x,z=a.z-b.z;return std::sqrt(x*x+z*z);}
Vec3 add(Vec3 a,Vec3 b,float k=1) {return {a.x+b.x*k,a.y+b.y*k,a.z+b.z*k};}
Vec3 unit(Vec3 p) {const float n=std::sqrt(p.x*p.x+p.z*p.z);return n>.0001f?Vec3{p.x/n,0,p.z/n}:Vec3{};}
float dot(Vec3 a,Vec3 b) {return a.x*b.x+a.z*b.z;}
bool finite3(Vec3 p) {return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
const ActorSnapshot* actor(const WorldSnapshot& w,ActorKey key) {
    for(const auto& a:w.actors)if(a.key==key)return &a;return nullptr;
}
const Portal* portal(const Building& b,std::uint32_t id) {
    for(const auto& p:b.portals)if(p.id==id)return &p;return nullptr;
}
std::uint32_t other(const Portal& p,std::uint32_t room) {return p.room_a==room?p.room_b:p.room_a;}
bool searched(const RoomStackRequest& r,std::uint32_t room) {
    return room==invalid_room||std::find(r.searched_rooms.begin(),r.searched_rooms.end(),room)!=r.searched_rooms.end();
}
struct Frame {Vec3 normal,tangent;};
Frame frame(const Portal& p,std::uint32_t target) {
    Vec3 n=unit(p.normal);if(p.room_a==target)n=add({},n,-1);
    Vec3 t=unit(add(p.jamb_b,p.jamb_a,-1));
    if(dot(t,t)<.5f)t={n.z,0,-n.x};return {n,t};
}
float door_cost(const WorldSnapshot& w,const Portal& p,const RoomConfig& c) {
    for(const auto& d:w.doors)if(d.portal==p.id||(p.door!=invalid_id&&d.id==p.door))
        return c.funnel_cost_m*((1.f-d.open)+(d.locked?c.door_locked_cost_k:0.f)+(d.held?c.door_held_cost_k:0.f));
    return 0;
}
// Lit completeness distinguishes an actually dark cell from missing data.
float red(const EngineServices& services,const Building& b,const RoomStackRequest& r,
    std::uint32_t node,Vec3 pos,const RoomConfig& c,std::uint32_t room,
    RoomExposureCost* prices=nullptr,RoomExposureCostReceipt* receipt=nullptr,bool held=false) {
    float cost=0,prior=0;
    for(const auto& p:b.portals) {
        const std::vector<Lit>* table=nullptr;
        if(room==p.room_a&&!searched(r,p.room_b))table=&p.lit_a;
        else if(room==p.room_b&&!searched(r,p.room_a))table=&p.lit_b;
        if(table) {
            auto it=std::lower_bound(table->begin(),table->end(),node,[](const Lit& l,std::uint32_t n){return l.node<n;});
            if(it!=table->end()&&it->node==node){const float value=c.exposure_cost_m*float(it->standing)/255.f;cost+=value;prior+=value;}
            else if(!services.buildings->lit_complete){cost+=c.unknown_cost_m;prior+=c.unknown_cost_m;}
        }
        const auto f=frame(p,p.room_b);const auto v=add(pos,p.centre,-1);
        if(std::abs(dot(v,f.normal))<c.wall_offset_m&&std::abs(dot(v,f.tangent))<p.half_width+c.arrival_m)
            cost+=c.funnel_cost_m;
    }
    if(prices){auto quote=prices->price(RoomExposureRole::stack,b.id,room,node,pos,prior,held);
        if(prices->was_cancelled())throw StackCancelled{};if(receipt)*receipt=quote;cost+=quote.cost-prior;}
    return cost;
}
// These receipts live only during ONE update_impl formation pass. The captured
// world epoch/map/geometry, building/lit data, request and costs are immutable in
// that pass. Never carry query results across updates, retries or cancellations.
// Exact float bits avoid approximate merging of authored projection candidates.
std::uint32_t bits(float value) {std::uint32_t out;std::memcpy(&out,&value,sizeof(out));return out;}
struct UpdateQueries {
    RoomExposureCost* prices=nullptr;
    struct Projection {std::uint32_t node;Vec3 point;};
    std::map<std::array<std::uint32_t,5>,Projection> projections;
    std::map<std::uint32_t,std::uint32_t> rooms;
    std::map<std::array<std::uint32_t,4>,float> exposure;
    std::map<std::array<std::uint32_t,3>,float> source_paths;
    float source_path(const EngineServices& s,std::uint32_t from,std::uint32_t to,std::uint32_t limit){
        const std::array<std::uint32_t,3> key={from,to,limit};
        const auto found=source_paths.find(key);if(found!=source_paths.end())return found->second;
        const float length=s.path_length(from,to,limit);source_paths.emplace(key,length);return length;
    }
    Projection project(const EngineServices& s,Vec3 p,const RoomConfig& c) {
        const std::array<std::uint32_t,5> key={bits(p.x),bits(p.y),bits(p.z),bits(c.projection_m),bits(c.floor_tolerance_m)};
        const auto found=projections.find(key);if(found!=projections.end())return found->second;
        Projection result{invalid_node,{}};result.node=s.project(p,c.projection_m,c.floor_tolerance_m,&result.point);
        projections.emplace(key,result);return result;
    }
    std::uint32_t room(const EngineServices& s,std::uint32_t node) {
        const auto found=rooms.find(node);if(found!=rooms.end())return found->second;
        const auto result=s.buildings->locate?s.buildings->locate(node).second:invalid_room;
        rooms.emplace(node,result);return result;
    }
    float red_cost(const EngineServices& s,const Building& b,const RoomStackRequest& r,
        std::uint32_t node,Vec3 point,const RoomConfig& c,std::uint32_t location) {
        const std::array<std::uint32_t,4> key={node,bits(point.x),bits(point.y),bits(point.z)};
        const auto found=exposure.find(key);if(found!=exposure.end())return found->second;
        const float result=red(s,b,r,node,point,c,location,prices);exposure.emplace(key,result);return result;
    }
};
// A separate instance per actor binds every path receipt to that actor's spawn,
// ownership/restriction revision and from-node in the immutable input snapshot.
// Equal from-nodes from different actors never share a path receipt. Failed
// bounded searches are reusable only within this same actor's formation pass.
struct ActorPaths {
    const ActorSnapshot& actor;
    explicit ActorPaths(const ActorSnapshot& captured):actor(captured) {}
    std::map<std::array<std::uint32_t,3>,float> lengths;
    float length(const EngineServices& s,std::uint32_t to,std::uint32_t limit) {
        const std::array<std::uint32_t,3> key={actor.node_id,to,limit};
        const auto found=lengths.find(key);if(found!=lengths.end())return found->second;
        const float result=s.path_length?s.path_length(actor.node_id,to,limit):-1.f;
        lengths.emplace(key,result);return result;
    }
};
bool conflict(const std::vector<RoomStackSlot>& slots,std::uint32_t n,Vec3 p,float spacing) {
    for(const auto& s:slots)if(s.order.goal_node==n||flat(p,s.order.goal_position)<spacing)return true;
    return false;
}
bool arrived_at(const ActorSnapshot& a,const RoomStackSlot& slot,const RoomConfig& c) {
    return flat(a.position,slot.order.goal_position)<=c.arrival_m&&
        std::abs(a.position.y-slot.order.goal_position.y)<=c.floor_tolerance_m;
}
void stack_look(RoomStackSlot& slot,const WorldSnapshot& w,const Building& b,
    const RoomStackRequest& request,const RoomConfig& c) {
    const auto* p=portal(b,slot.entry_portal);if(!p)return;
    auto& o=slot.order;o.look.kind=LookKind::point;o.look.value=p->centre;
    o.role=slot.watcher?roles::portal_watch:(o.assignment.slot==0?roles::door_stack_point:roles::door_stack);
    if(!slot.watcher&&o.assignment.slot>=c.rear_from_slot) {
        o.role=roles::rear_security;
        const auto f=frame(*p,request.target_room);
        o.look.value=add(o.goal_position,f.tangent,float(o.side)*c.post_distance_m);
        // Price an opening against the current rearward look's reach along the
        // wall. No posture, enemy census or hidden threat is assigned here.
        float cost=flat(o.goal_position,p->centre)+c.post_distance_m;
        const auto stack_room=other(*p,request.target_room);
        for(const auto& candidate:b.portals)if(candidate.id!=p->id&&candidate.doorway&&
            (candidate.room_a==stack_room||candidate.room_b==stack_room)&&
            !searched(request,other(candidate,stack_room))) {
            const float alternative=flat(o.goal_position,candidate.centre)+door_cost(w,candidate,c);
            if(alternative<cost){cost=alternative;o.look.value=candidate.centre;}
        }
    }
    o.look.value.y=o.goal_position.y+c.look_height_m;
}
Order base_order(const WorldSnapshot& w,const ActorSnapshot& a,const RoomConfig& c,TimeMs lifetime,std::uint64_t gen) {
    Order o;o.actor=a.key;o.level_epoch=w.level_epoch;o.snapshot_seq=w.sequence;o.geometry_revision=w.geometry_revision;
    o.ownership_generation=a.ownership_generation;o.restrictions_revision=a.restrictions_revision;
    o.issued_at_ms=w.now_ms;o.expires_at_ms=saturated_after(w.now_ms,lifetime);
    o.goal_node=a.node_id;o.goal_position=a.position;o.arrival_radius_m=c.arrival_m;
    o.assignment.generation=gen;o.kind=OrderKind::stack;o.pace=Pace::walk;o.look.kind=LookKind::point;return o;
}
struct Placement {std::uint32_t node=invalid_node;Vec3 point;float cost=0,red=0;bool source_queue=false;};
Placement pick(const WorldSnapshot& w,const ActorSnapshot& a,const Building& b,const Portal& p,
    const RoomStackRequest& r,const EngineServices& s,const RoomConfig& c,
    const std::vector<RoomStackSlot>& slots,float index,int side,bool watch,std::uint32_t stack_room,const std::vector<std::uint32_t>& rejected,
    const std::function<bool()>& cancelled,UpdateQueries& queries,ActorPaths& paths) {
    Placement best;best.cost=std::numeric_limits<float>::infinity();
    const auto f=frame(p,watch?other(p,stack_room):r.target_room);
    const float pitch=(std::max)(c.pitch_m,c.spacing_m);
    for(std::uint16_t attempt=0;attempt<c.alternative_slots;++attempt) {
        if(cancelled&&cancelled())throw StackCancelled{};
        const unsigned offset=attempt/2;
        const float rank=(std::max)(0.f,index+(offset%2?float((offset+1)/2):-float(offset/2)));
        const float lateral=p.half_width+c.jamb_offset_m+rank*pitch;
        const float away=(watch?c.post_distance_m:c.wall_offset_m)+float(attempt%2)*pitch;
        Vec3 q=add(add(p.centre,f.tangent,float(side)*lateral),f.normal,-away);q.y=p.y1>p.y0?p.y0:p.centre.y;
        const auto projection=queries.project(s,q,c);const auto n=projection.node;const auto snapped=projection.point;
        if(n==invalid_node||!finite3(snapped)||std::abs(snapped.y-q.y)>c.floor_tolerance_m||
            flat(q,snapped)>c.projection_m||conflict(slots,n,snapped,c.spacing_m))continue;
        if(a.last_rejected.node==n&&a.last_rejected.restrictions_revision==a.restrictions_revision&&
            (a.last_rejected.reason==reject_goal_invalid||a.last_rejected.reason==reject_goal_inaccessible))continue;
        if(std::find(rejected.begin(),rejected.end(),n)!=rejected.end())continue;
        const auto location=queries.room(s,n);
        if(location==r.target_room)continue; // A FORM slot cannot be inside the uncleared destination.
        const float ray_cost=queries.red_cost(s,b,r,n,snapped,c,location);
        if(cancelled&&cancelled())throw StackCancelled{};
        const float length=paths.length(s,n,c.path_nodes);
        const float travel=length>=0?length:flat(a.position,snapped)+c.path_unknown_cost_m;
        const float cost=travel*c.approach_weight+ray_cost+door_cost(w,p,c);
        if(cost<best.cost)best={n,snapped,cost,ray_cost};
    }
    if(best.node==invalid_node&&!watch&&s.node_position&&s.path_length&&s.buildings->locate){
        // Narrow or sloped approaches may have no lateral standing cells at the
        // portal's height. Queue on captured navigation nodes in the source
        // room instead, keeping every goal outside the uncleared destination.
        const auto source=std::find_if(b.rooms.begin(),b.rooms.end(),[&](const Room& room){return room.id==stack_room;});
        if(source!=b.rooms.end()){
            struct Approach {std::uint32_t node;Vec3 point;float depth,lateral,near;};
            std::vector<Approach> approach;
            const float depth_limit=c.wall_offset_m+(index+4.f)*pitch;
            const float width=(std::max)(p.half_width,c.spacing_m);
            for(auto n:source->node_ids){if(cancelled&&cancelled())throw StackCancelled{};
                Vec3 point;if(n==invalid_node||!s.node_position(n,&point)||!finite3(point)||
                    queries.room(s,n)!=stack_room)continue;
                const auto relative=add(point,p.centre,-1);
                const float depth=-dot(relative,f.normal),lateral=std::abs(dot(relative,f.tangent));
                if(depth<0||depth>depth_limit||lateral>width||conflict(slots,n,point,c.spacing_m))continue;
                const float near=distance_squared(point,p.centre);
                approach.push_back({n,point,depth,lateral,near});
            }
            std::stable_sort(approach.begin(),approach.end(),[](const Approach& a,const Approach& b){
                return a.near==b.near?a.node<b.node:a.near<b.near;});
            if(!approach.empty()){
                const auto anchor=approach.front().node;
                const auto limit=(std::min)(approach.size(),std::size_t((std::max)(16u,unsigned(c.alternative_slots)*4u)));
                for(std::size_t i=0;i<limit;++i){if(cancelled&&cancelled())throw StackCancelled{};
                    const auto& candidate=approach[i];
                    if((a.last_rejected.node==candidate.node&&a.last_rejected.restrictions_revision==a.restrictions_revision&&
                        (a.last_rejected.reason==reject_goal_invalid||a.last_rejected.reason==reject_goal_inaccessible))||
                        std::find(rejected.begin(),rejected.end(),candidate.node)!=rejected.end())continue;
                    const float travel=paths.length(s,candidate.node,c.path_nodes);
                    if(!std::isfinite(travel)||travel<0)continue;
                    const float to_door=queries.source_path(s,candidate.node,anchor,c.path_nodes);
                    if(!std::isfinite(to_door)||to_door<0)continue;
                    const float ray_cost=queries.red_cost(s,b,r,candidate.node,candidate.point,c,stack_room);
                    const float desired_depth=c.wall_offset_m+index*pitch*1.5f;
                    const float cost=travel*c.approach_weight+ray_cost+door_cost(w,p,c)+
                        std::abs(candidate.depth-desired_depth)*8.f+candidate.lateral*8.f;
                    if(cost<best.cost)best={candidate.node,candidate.point,cost,ray_cost,true};
                }
            }
        }
    }
    return best;
}
}
bool room_care_owned(const WorldSnapshot& w,const ActorSnapshot& a,const std::vector<Order>& orders){
    if(a.level_epoch!=w.level_epoch||a.snapshot_seq!=w.sequence||a.observed_at_ms>w.now_ms)return false;
    const auto& care=a.aid_state;
    if((care.phase==1||care.phase==2)&&valid_key(care.target)&&care.at_ms<=a.observed_at_ms)return true;
    for(const auto& o:orders)if(o.actor==a.key&&o.ownership_generation==a.ownership_generation&&
        o.restrictions_revision==a.restrictions_revision&&o.level_epoch==w.level_epoch&&o.geometry_revision==w.geometry_revision&&
        o.snapshot_seq<=w.sequence&&o.order_seq&&o.issued_at_ms<=w.now_ms&&o.expires_at_ms>w.now_ms&&
        o.kind==OrderKind::recover&&valid_key(o.aid)&&o.assignment.generation&&
        (o.role==roles::aid_casualty||o.role==roles::casualty_guard)&&
        !((care.phase==3||care.phase==4)&&care.target==o.aid&&care.at_ms>=o.issued_at_ms&&care.at_ms<=a.observed_at_ms))return true;
    return false;
}
RoomStack::RoomStack(Config c):engine_config_(c),config_(c.room) {}
void RoomStack::reset(){state_={};epoch_=geometry_=map_=0;mission_revision_=side_=requested_portal_=single_portal_=0;last_retry_ms_=0;roster_.clear();searched_.clear();rejected_.clear();gaps_.clear();closing_.clear();queued_entry_=false;}
RoomStackResult RoomStack::update(const WorldSnapshot& w,const SideSnapshot& side,const RoomStackRequest& request,
    const EngineServices& services,const std::function<bool()>& cancelled) {
    if(cancelled&&cancelled())return {};
    auto staged=*this;
    try {
        auto result=staged.update_impl(w,side,request,services,cancelled);
        if(cancelled&&cancelled())return {};
        if(result.phase!=RoomStackPhase::invalid)*this=std::move(staged);
        return result;
    }catch(const StackCancelled&){return {};}
}
RoomStackResult RoomStack::update_impl(const WorldSnapshot& w,const SideSnapshot& side,const RoomStackRequest& request,
    const EngineServices& services,const std::function<bool()>& cancelled) {
    RoomStackResult invalid;
    const float values[]={config_.pitch_m,config_.spacing_m,config_.jamb_offset_m,config_.wall_offset_m,
        config_.arrival_m,config_.projection_m,config_.floor_tolerance_m,config_.exposure_cost_m,
        config_.unknown_cost_m,config_.funnel_cost_m,config_.path_unknown_cost_m,config_.approach_weight,
        config_.alternate_door_cost_m,config_.straggler_cost_per_second,config_.readiness_missing_cost,
        config_.two_door_gain_m,config_.post_distance_m,config_.look_height_m,config_.door_locked_cost_k,config_.door_held_cost_k,
        config_.form_tolerance_m,config_.entry_third_m,config_.pie_mid_m,config_.edge_lean,config_.leader_progress_m};
    for(float value:values)if(!std::isfinite(value)||value<0){invalid.reason=StackReason::invalid_config;return invalid;}
    if(config_.spacing_m<=0||config_.arrival_m<=0||config_.projection_m<=0||config_.edge_lean>1||!config_.alternative_slots||
        !config_.minimum_door_members||!engine_config_.order_lifetime_ms){invalid.reason=StackReason::invalid_config;return invalid;}
    if(!w.level_epoch||!services.buildings||services.buildings->level_epoch!=w.level_epoch||
        services.buildings->map_hash!=w.map_hash) {invalid.reason=StackReason::invalid_epoch;return invalid;}
    if(!services.project)return invalid;
    const Building* building=nullptr;
    for(const auto& b:services.buildings->buildings)if(b.id==side.mission.building){building=&b;break;}
    if(!building||std::none_of(building->rooms.begin(),building->rooms.end(),[&](const Room& room){return room.id==request.target_room;}))return invalid;
    std::vector<const ActorSnapshot*> members;
    for(auto key:side.members) {const auto* a=actor(w,key);if(a&&a->alive&&!a->wounded&&!a->yield_reasons&&!room_care_owned(w,*a,request.applied_orders)&&
        (a->flags&flag_admitted)&&a->weapon.id!=invalid_id&&(a->weapon.ammo_in_mag||a->weapon.ammo_reserve))members.push_back(a);}
    if(members.empty())return invalid;
    RoomExposureCost prices(w,side,services.buildings,request.exposure,cancelled);
    if(prices.was_cancelled())throw StackCancelled{};
    if(epoch_!=w.level_epoch||geometry_!=w.geometry_revision||map_!=w.map_hash||state_.target_room!=request.target_room)rejected_.clear();
    rejected_.erase(std::remove_if(rejected_.begin(),rejected_.end(),[&](const RejectedSlot& r){
        return std::none_of(members.begin(),members.end(),[&](const ActorSnapshot* a){return a->key==r.member.actor&&
            a->ownership_generation==r.member.owner&&a->restrictions_revision==r.member.restrictions;});}),rejected_.end());
    for(const auto* a:members)if(a->last_rejected.node!=invalid_node&&a->last_rejected.restrictions_revision==a->restrictions_revision&&
        (a->last_rejected.reason==reject_goal_invalid||a->last_rejected.reason==reject_goal_inaccessible)&&
        std::none_of(rejected_.begin(),rejected_.end(),[&](const RejectedSlot& r){return r.member.actor==a->key&&r.node==a->last_rejected.node;}))
        rejected_.push_back({{a->key,a->ownership_generation,a->restrictions_revision},a->last_rejected.node});
    const bool reset_formation=epoch_!=w.level_epoch||geometry_!=w.geometry_revision||map_!=w.map_hash||
        mission_revision_!=side.mission.revision||side_!=side.side_id||state_.target_room!=request.target_room||
        requested_portal_!=request.entry_portal||searched_!=request.searched_rooms||
        permit_two_doors_!=request.permit_two_doors||call_breacher_first_!=request.call_breacher_first;
    bool changed=reset_formation||members.size()!=roster_.size();
    for(const auto* a:members) {
        const auto identity=std::find_if(roster_.begin(),roster_.end(),[&](const MemberIdentity& id){return id.actor==a->key;});
        if(identity==roster_.end()||identity->owner!=a->ownership_generation||identity->restrictions!=a->restrictions_revision)changed=true;
    }
    RoomStackResult next=state_;next.events.clear();
    // A physically stalled, actually executed stack order is evidence that
    // this cell is not making progress for this actor. Before parking the
    // whole side, look for a distinct reachable cell at the same doorway.
    // Emergency cover/reload is not a failed stack route and cannot trigger
    // this recovery. One retry per interval bounds projection and path work.
    bool searched_stalled=false;
    if(!changed&&!next.unprojected&&w.now_ms>=last_retry_ms_&&w.now_ms-last_retry_ms_>=config_.swap_ms&&
        services.path_length&&services.buildings->locate){
        UpdateQueries queries;queries.prices=&prices;
        for(const auto& slot:next.slots){
            if(cancelled&&cancelled())throw StackCancelled{};
            if(slot.watcher||slot.order.assignment.slot>=2)continue;
            const auto* a=actor(w,slot.actor);const auto* door=portal(*building,slot.entry_portal);
            if(!a||!door||arrived_at(*a,slot,config_)||a->reloading||a->weapon.state==WeaponState::reloading)continue;
            const auto progress=std::find_if(closing_.begin(),closing_.end(),[&](const Closing& p){return
                p.actor==a->key&&p.node==slot.order.goal_node;});
            if(progress==closing_.end()||w.now_ms<progress->progressed_ms||
                w.now_ms-progress->progressed_ms<config_.swap_ms||
                !fresh(a->observed_at_ms,w.now_ms,engine_config_.snapshot_max_age_ms))continue;
            const bool executed=std::any_of(request.applied_orders.begin(),request.applied_orders.end(),[&](const Order& o){return
                o.actor==a->key&&o.order_seq&&o.order_seq==a->last_applied.order_seq&&
                o.ownership_generation==a->ownership_generation&&o.restrictions_revision==a->restrictions_revision&&
                o.level_epoch==w.level_epoch&&o.geometry_revision==w.geometry_revision&&
                o.kind==OrderKind::stack&&o.goal_node==slot.order.goal_node&&
                o.issued_at_ms<=a->last_applied.at_ms&&o.expires_at_ms>w.now_ms&&
                a->last_applied.at_ms<=a->observed_at_ms&&
                fresh(a->last_applied.at_ms,w.now_ms,engine_config_.snapshot_max_age_ms);});
            if(!executed)continue;
            const bool waiting_rear=std::any_of(next.slots.begin(),next.slots.end(),[&](const RoomStackSlot& rear){
                const auto* b=actor(w,rear.actor);return !rear.watcher&&rear.entry_portal==slot.entry_portal&&
                    rear.order.assignment.slot>=2&&b&&arrived_at(*b,rear,config_);});
            if(waiting_rear)continue; // the arrived replacement has first claim
            searched_stalled=true;
            std::vector<RoomStackSlot> occupied;for(const auto& other_slot:next.slots)
                if(other_slot.actor!=slot.actor)occupied.push_back(other_slot);
            std::vector<std::uint32_t> excluded{slot.order.goal_node};
            for(const auto& r:rejected_)if(r.member.actor==a->key)excluded.push_back(r.node);
            ActorPaths paths(*a);const auto source=other(*door,request.target_room);
            const auto left=pick(w,*a,*building,*door,request,services,config_,occupied,
                float(slot.order.assignment.slot),-1,false,source,excluded,cancelled,queries,paths);
            const auto right=pick(w,*a,*building,*door,request,services,config_,occupied,
                float(slot.order.assignment.slot),1,false,source,excluded,cancelled,queries,paths);
            const auto alternative=left.cost<=right.cost?left:right;
            if(alternative.node==invalid_node||queries.room(services,alternative.node)!=source)continue;
            const float route=paths.length(services,alternative.node,config_.path_nodes);
            if(!std::isfinite(route)||route<0)continue;
            rejected_.push_back({{a->key,a->ownership_generation,a->restrictions_revision},slot.order.goal_node});
            break;
        }
    }
    // An exhausted search also consumes its budget. Restriction rejections
    // still repair immediately below; a rear arrival never enters this path.
    if(searched_stalled)last_retry_ms_=w.now_ms;
    // A roster/lease/restriction change repairs only the changed actor. The door
    // and existing claims still describe the same formation and clock.
    if(changed&&!reset_formation) {
        ++next.generation;
        next.slots.erase(std::remove_if(next.slots.begin(),next.slots.end(),[&](const RoomStackSlot& slot){
            return std::none_of(members.begin(),members.end(),[&](const ActorSnapshot* a){return a->key==slot.actor&&
                a->ownership_generation==slot.order.ownership_generation&&a->restrictions_revision==slot.order.restrictions_revision;});
        }),next.slots.end());
        gaps_.clear();
    }
    const auto before_rejections=next.slots.size();
    next.slots.erase(std::remove_if(next.slots.begin(),next.slots.end(),[&](const RoomStackSlot& slot){
        return std::any_of(rejected_.begin(),rejected_.end(),[&](const RejectedSlot& r){return r.member.actor==slot.actor&&r.node==slot.order.goal_node;});}),next.slots.end());
    const bool repair_rejection=before_rejections!=next.slots.size();next.unprojected+=before_rejections-next.slots.size();
    const bool retry_missing=repair_rejection||(next.unprojected&&w.now_ms>=last_retry_ms_&&w.now_ms-last_retry_ms_>=config_.swap_ms);
    if(changed||retry_missing) {
        if(reset_formation){gaps_.clear();closing_.clear();next={};next.generation=state_.generation+1;next.formed_at_ms=w.now_ms;next.target_room=request.target_room;single_portal_=0;}
        next.unprojected=0;last_retry_ms_=w.now_ms;
        std::vector<const Portal*> doors;
        for(const auto& p:building->portals)if(p.doorway&&p.traversable&&
            (p.room_a==request.target_room||p.room_b==request.target_room)&&searched(request,other(p,request.target_room)))doors.push_back(&p);
        Vec3 centre;for(const auto* a:members)centre=add(centre,a->position,1.f/float(members.size()));
        std::stable_sort(doors.begin(),doors.end(),[&](const Portal* a,const Portal* b){
            const auto entry=single_portal_?single_portal_:request.entry_portal;
            if(entry&&((a->id==entry)!=(b->id==entry)))return a->id==entry;
            const float ca=flat(centre,a->centre)+door_cost(w,*a,config_),cb=flat(centre,b->centre)+door_cost(w,*b,config_);
            return ca==cb?a->id<b->id:ca<cb;
        });
        if(doors.empty())return invalid;
        UpdateQueries queries;queries.prices=&prices;
        std::map<std::array<std::uint64_t,4>,ActorPaths> member_paths;
        const auto paths_for=[&](const ActorSnapshot& a)->ActorPaths&{const std::array<std::uint64_t,4> key={a.key.id,a.key.generation,a.ownership_generation,a.restrictions_revision};
            return member_paths.try_emplace(key,a).first->second;};
        // A nearby doorway can have no projectable stack cells while another
        // doorway into the same room can hold an element. Price actual actor
        // placements before committing the main door; a watch post does not
        // count as an entrant or make an unusable entrance viable.
        struct Fit {std::size_t entrants=0;float cost=0;bool source_queue=false;};
        auto fit=[&](const Portal& door){
            auto ordered=members;
            std::stable_sort(ordered.begin(),ordered.end(),[&](const ActorSnapshot* a,const ActorSnapshot* b){
                const float da=flat(a->position,door.centre),db=flat(b->position,door.centre);
                return da==db?a->key.id<b->key.id:da<db;});
            const auto breacher=std::find_if(ordered.begin(),ordered.end(),[](const ActorSnapshot* a){
                return a->weapon.kind==WeaponKind::shotgun||a->grenades.count>0;});
            if(breacher!=ordered.end()&&ordered.size()>1){const auto* b=*breacher;ordered.erase(breacher);
                ordered.insert(ordered.begin()+(request.call_breacher_first?0:1),b);}
            std::vector<RoomStackSlot> placed;Fit fit;
            for(const auto* a:ordered){if(cancelled&&cancelled())throw StackCancelled{};
                std::vector<std::uint32_t> rejected;for(const auto& r:rejected_)if(r.member.actor==a->key)rejected.push_back(r.node);
                auto& paths=paths_for(*a);const auto rank=float(placed.size());
                const auto left=pick(w,*a,*building,door,request,services,config_,placed,rank,-1,false,
                    other(door,request.target_room),rejected,cancelled,queries,paths);
                const auto right=pick(w,*a,*building,door,request,services,config_,placed,rank,1,false,
                    other(door,request.target_room),rejected,cancelled,queries,paths);
                const auto& best=left.cost<=right.cost?left:right;
                if(best.node==invalid_node)continue;
                RoomStackSlot slot;slot.actor=a->key;slot.order.goal_node=best.node;slot.order.goal_position=best.point;
                placed.push_back(slot);fit.cost+=best.cost;fit.source_queue|=best.source_queue;
            }
            fit.entrants=placed.size();return fit;
        };
        const auto* main=doors.front();
        const auto initial_fit=fit(*main);
        auto main_fit=initial_fit;
        if(initial_fit.entrants==0) {
            auto best=initial_fit;
            for(std::size_t i=1;i<doors.size();++i){const auto alternative=fit(*doors[i]);
                if(alternative.entrants>best.entrants||(alternative.entrants==best.entrants&&alternative.cost<best.cost)){
                    main=doors[i];best=alternative;}}
            main_fit=best;
        }
        if(main!=doors.front())std::iter_swap(doors.begin(),std::find(doors.begin(),doors.end(),main));
        main=doors.front();
        if(reset_formation)queued_entry_=false;
        queued_entry_|=main_fit.source_queue;
        const auto stack_room=other(*main,request.target_room);
        const bool was_split=next.two_doors;
        // Compare actual projected placements, graph travel, aperture exposure
        // and current door facts. Door-centre separation alone is not a plan.
        std::stable_sort(members.begin(),members.end(),[&](const ActorSnapshot* a,const ActorSnapshot* b){
            const float da=flat(a->position,main->centre),db=flat(b->position,main->centre);return da==db?a->key.id<b->key.id:da<db;
        });
        const auto breacher=std::find_if(members.begin(),members.end(),[](const ActorSnapshot* a){return a->weapon.kind==WeaponKind::shotgun||a->grenades.count>0;});
        if(breacher!=members.end()&&members.size()>1){const auto* b=*breacher;members.erase(breacher);members.insert(members.begin()+(request.call_breacher_first?0:1),b);}
        const auto plan_cost=[&](const Portal* second){
            std::vector<RoomStackSlot> placed;float total=0;int signs[2]={0,0};unsigned ranks[2]={0,0};
            for(std::size_t i=0;i<members.size();++i){if(cancelled&&cancelled())throw StackCancelled{};
                const auto& a=*members[i];const auto element=second&&i>=members.size()/2?1:0;const auto& door=element?*second:*main;
                std::vector<std::uint32_t> rejected;for(const auto& r:rejected_)if(r.member.actor==a.key)rejected.push_back(r.node);
                auto& paths=paths_for(a);auto left=pick(w,a,*building,door,request,services,config_,placed,float(ranks[element]),-1,false,other(door,request.target_room),rejected,cancelled,queries,paths);
                auto right=pick(w,a,*building,door,request,services,config_,placed,float(ranks[element]),1,false,other(door,request.target_room),rejected,cancelled,queries,paths);
                const bool choose_left=signs[element]&&(signs[element]<0?left.node:right.node)!=invalid_node?signs[element]<0:left.cost<=right.cost;
                const auto& value=choose_left?left:right;if(value.node==invalid_node||
                    queries.room(services,value.node)==request.target_room)return std::numeric_limits<float>::infinity();
                const float route=paths.length(services,value.node,config_.path_nodes);
                if(!std::isfinite(route)||route<0)return std::numeric_limits<float>::infinity();
                // A second element is a flank only when it is already on the
                // source side of that entrance. A finite route from another
                // room can cross the uncleared target before reaching an
                // exterior goal; path_length alone cannot prove otherwise.
                if(second&&element==1&&services.buildings->locate&&
                    services.buildings->locate(a.node_id)!=services.buildings->locate(value.node))
                    return std::numeric_limits<float>::infinity();
                if(!signs[element])signs[element]=choose_left?-1:1;++ranks[element];total+=value.cost;
                RoomStackSlot slot;slot.actor=a.key;slot.order.goal_node=value.node;slot.order.goal_position=value.point;placed.push_back(slot);
            }return total;
        };
        next.single_door_cost=plan_cost(nullptr);next.split_door_cost=std::numeric_limits<float>::infinity();next.two_doors=false;
        if(!single_portal_&&request.permit_two_doors&&doors.size()>1&&config_.minimum_door_members>0&&
            members.size()>=std::size_t(config_.minimum_door_members)*2){std::size_t selected=1;
            for(std::size_t i=1;i<doors.size();++i){const float cost=plan_cost(doors[i])+config_.alternate_door_cost_m-config_.two_door_gain_m;
                if(cost<next.split_door_cost){next.split_door_cost=cost;selected=i;}}
            if(std::isfinite(next.split_door_cost)&&next.split_door_cost<next.single_door_cost){next.two_doors=true;std::swap(doors[1],doors[selected]);}
        }
        if(was_split&&!next.two_doors) {
            single_portal_=main->id;
            next.slots.erase(std::remove_if(next.slots.begin(),next.slots.end(),[&](const RoomStackSlot& slot){
                return slot.entry_portal!=single_portal_;}),next.slots.end());
        }
        std::uint32_t counts[2]={0,0};int stack_side[2]={0,0};
        for(const auto& slot:next.slots)if(!slot.watcher){auto e=next.two_doors&&slot.entry_portal==doors[1]->id?1:0;counts[e]=(std::max)(counts[e],unsigned(slot.order.assignment.slot)+1);if(!stack_side[e])stack_side[e]=slot.order.side;}
        for(std::size_t index=0;index<members.size();++index) {
            if(cancelled&&cancelled())return invalid;
            const auto& a=*members[index];const std::size_t element=next.two_doors&&index>=members.size()/2?1:0;
            if(std::any_of(next.slots.begin(),next.slots.end(),[&](const RoomStackSlot& slot){return slot.actor==a.key;}))continue;
            counts[element]=0;
            while(std::any_of(next.slots.begin(),next.slots.end(),[&](const RoomStackSlot& slot){return !slot.watcher&&
                slot.entry_portal==doors[element]->id&&slot.order.assignment.slot==counts[element];}))++counts[element];
            std::vector<std::uint32_t> rejected;for(const auto& r:rejected_)if(r.member.actor==a.key)rejected.push_back(r.node);
            auto& paths=paths_for(a);
            const Portal* chosen=doors[element];bool watcher=false;
            auto l=pick(w,a,*building,*chosen,request,services,config_,next.slots,float(counts[element]),-1,false,stack_room,rejected,cancelled,queries,paths);
            auto r=pick(w,a,*building,*chosen,request,services,config_,next.slots,float(counts[element]),1,false,stack_room,rejected,cancelled,queries,paths);
            float preferred=stack_side[element]?(stack_side[element]<0?l.cost:r.cost):(std::min)(l.cost,r.cost);
            if(!std::isfinite(preferred))preferred=(std::min)(l.cost,r.cost);
            if(!main_fit.source_queue&&!single_portal_&&!next.two_doors&&members.size()>=config_.watch_post_min_men&&index==members.size()-1) {
                // A distant exterior opening must not send rear security around
                // the whole building and split the team before its first entry.
                for(const auto& p:building->portals)if(p.id!=main->id&&p.doorway&&
                    (p.room_a==stack_room||p.room_b==stack_room)&&!searched(request,other(p,stack_room))) {
                    auto pl=pick(w,a,*building,p,request,services,config_,next.slots,0,-1,true,stack_room,rejected,cancelled,queries,paths);
                    auto pr=pick(w,a,*building,p,request,services,config_,next.slots,0,1,true,stack_room,rejected,cancelled,queries,paths);
                    const float cost=(std::min)(pl.cost,pr.cost)+flat(main->centre,p.centre)-config_.two_door_gain_m;
                    if(cost<preferred){preferred=cost;chosen=&p;watcher=true;l=pl;r=pr;}
                }
            }
            const auto use_left=!watcher&&stack_side[element]&&
                (stack_side[element]<0?l.node:r.node)!=invalid_node?stack_side[element]<0:
                (l.cost<r.cost||(l.cost==r.cost&&((engine_config_.seed^side.side_id^chosen->id)&1)));
            const auto best=use_left?l:r;
            if(best.node==invalid_node){++next.unprojected;continue;}
            RoomStackSlot slot;slot.actor=a.key;slot.entry_portal=chosen->id;slot.watcher=watcher;
            slot.placement_cost=best.cost;slot.red_cost=best.red;slot.order=base_order(w,a,config_,engine_config_.order_lifetime_ms,next.generation);
            auto& o=slot.order;o.goal_node=best.node;o.goal_position=best.point;o.look.value=chosen->centre;
            o.look.value.y=best.point.y+config_.look_height_m;
            o.side=use_left?-1:1;o.assignment.portal=chosen->id;o.assignment.slot=static_cast<std::uint8_t>((std::min)(counts[element],255u));
            o.role=watcher?roles::portal_watch:(counts[element]==0?roles::door_stack_point:roles::door_stack);
            const auto f=frame(*chosen,watcher?other(*chosen,stack_room):request.target_room);
            o.portal={chosen->centre,f.tangent,f.normal,chosen->half_width,chosen->wall_depth,true};
            if(watcher){o.kind=OrderKind::hold_door;o.assignment.slot=255;}
            else {++counts[element];if(!stack_side[element])stack_side[element]=o.side;}
            next.slots.push_back(slot);
        }
        // A lost point can leave every surviving actor holding a later slot.
        // Repair the missing lead pair through the ordinary projected cell and
        // actor graph route, never by renumbering the slots vector. A captured
        // arrival at these new cells is still required before READY.
        std::set<std::uint32_t> entry_portals;
        for(const auto& slot:next.slots)if(!slot.watcher)entry_portals.insert(slot.entry_portal);
        for(auto id:entry_portals)for(unsigned wanted=0;wanted<2;++wanted){
            if(cancelled&&cancelled())throw StackCancelled{};
            const auto count=std::count_if(next.slots.begin(),next.slots.end(),[&](const RoomStackSlot& slot){
                return !slot.watcher&&slot.entry_portal==id;});
            if(count<=wanted||std::any_of(next.slots.begin(),next.slots.end(),[&](const RoomStackSlot& slot){
                return !slot.watcher&&slot.entry_portal==id&&slot.order.assignment.slot==wanted;}))continue;
            const auto* door=portal(*building,id);if(!door){++next.unprojected;continue;}
            auto selected=next.slots.end();Placement best;int selected_side=0;float price=std::numeric_limits<float>::infinity();
            for(auto it=next.slots.begin();it!=next.slots.end();++it){
                if(it->watcher||it->entry_portal!=id||it->order.assignment.slot<=wanted)continue;
                const auto* a=actor(w,it->actor);if(!a||a->reloading||a->weapon.state==WeaponState::reloading)continue;
                std::vector<RoomStackSlot> occupied;for(const auto& held:next.slots)if(held.actor!=it->actor)occupied.push_back(held);
                std::vector<std::uint32_t> denied;for(const auto& r:rejected_)if(r.member.actor==a->key)denied.push_back(r.node);
                auto& paths=paths_for(*a);const auto source=other(*door,request.target_room);
                const auto left=pick(w,*a,*building,*door,request,services,config_,occupied,float(wanted),-1,false,
                    source,denied,cancelled,queries,paths);
                const auto right=pick(w,*a,*building,*door,request,services,config_,occupied,float(wanted),1,false,
                    source,denied,cancelled,queries,paths);
                const bool choose_left=left.cost<=right.cost;const auto candidate=choose_left?left:right;
                if(candidate.node==invalid_node||queries.room(services,candidate.node)!=source)continue;
                const float route=paths.length(services,candidate.node,config_.path_nodes);
                if(!std::isfinite(route)||route<0)continue;
                const float cost=candidate.cost+float(it->order.assignment.slot-wanted)*.001f;
                if(cost<price){selected=it;best=candidate;selected_side=choose_left?-1:1;price=cost;}
            }
            if(selected==next.slots.end()){++next.unprojected;continue;}
            selected->order.goal_node=best.node;selected->order.goal_position=best.point;
            selected->order.assignment.slot=static_cast<std::uint8_t>(wanted);
            selected->order.role=wanted==0?roles::door_stack_point:roles::door_stack;
            selected->order.side=static_cast<std::int8_t>(selected_side);
            selected->placement_cost=best.cost;selected->red_cost=best.red;
            selected->arrived=false;selected->delayed=false;
            queued_entry_=true;gaps_.clear();closing_.clear();++next.generation;
        }
        if(next.slots.empty()){invalid.reason=StackReason::no_projection;return invalid;}
        if(reset_formation){next.phase=RoomStackPhase::forming;next.reason=StackReason::formed;}
    }
    // Reprice held placements without changing claims, goals or formation clocks.
    for(auto& slot:next.slots)if(request.exposure||slot.exposure.used){
        const auto location=services.buildings->locate?services.buildings->locate(slot.order.goal_node).second:invalid_room;
        const float updated=red(services,*building,request,slot.order.goal_node,slot.order.goal_position,config_,location,&prices,&slot.exposure,true);
        slot.placement_cost+=updated-slot.red_cost;slot.red_cost=updated;
    }
    // An arrived #3 may replace a delayed #2 independently at each entry. Price
    // the two routes plus accumulated wait; swap_ms caps this readiness gap.
    // Each new actor's captured restriction rejection and graph route are
    // checked. The graph ignores restrictions: accessibility remains UNKNOWN
    // until the native publication checks that actor's live restrictions.
    std::vector<ReadinessGap> gaps;
    for(auto& front:next.slots)if(!front.watcher&&front.order.assignment.slot==1) {
        if(cancelled&&cancelled())throw StackCancelled{};
        auto back=std::find_if(next.slots.begin(),next.slots.end(),[&](const RoomStackSlot& slot){return !slot.watcher&&
            slot.entry_portal==front.entry_portal&&slot.order.assignment.slot==2;});
        const auto* a=actor(w,front.actor);const auto* b=back==next.slots.end()?nullptr:actor(w,back->actor);
        if(!a||!b||arrived_at(*a,front,config_)||!arrived_at(*b,*back,config_))continue;
        const auto prior=std::find_if(gaps_.begin(),gaps_.end(),[&](const ReadinessGap& g){return
            g.portal==front.entry_portal&&g.front==a->key&&g.back==b->key;});
        const TimeMs since=prior==gaps_.end()?w.now_ms:prior->since;
        const auto denied=[&](const ActorSnapshot& member,std::uint32_t node){return std::any_of(rejected_.begin(),rejected_.end(),
            [&](const RejectedSlot& r){return r.member.actor==member.key&&r.node==node;});};
        if(denied(*a,back->order.goal_node)||denied(*b,front.order.goal_node))continue;
        ActorPaths ap(*a),bp(*b);
        if(cancelled&&cancelled())throw StackCancelled{};
        const float to_back=ap.length(services,back->order.goal_node,config_.path_nodes);
        if(cancelled&&cancelled())throw StackCancelled{};
        const float to_front=bp.length(services,front.order.goal_node,config_.path_nodes);
        if(!std::isfinite(to_back)||!std::isfinite(to_front)||to_back<0||to_front<0)continue;
        const auto travel=[&](ActorPaths& paths,const ActorSnapshot& member,const RoomStackSlot& slot){
            if(cancelled&&cancelled())throw StackCancelled{};
            const float length=paths.length(services,slot.order.goal_node,config_.path_nodes);
            return (length>=0?length:flat(member.position,slot.order.goal_position)+config_.path_unknown_cost_m)*config_.approach_weight;
        };
        const float keep=travel(ap,*a,front)+travel(bp,*b,*back);
        const float swap=(to_back+to_front)*config_.approach_weight;
        const TimeMs elapsed_gap=w.now_ms>=since?w.now_ms-since:0;
        float wait=float(elapsed_gap)/1000.f*config_.straggler_cost_per_second;
        if(elapsed_gap>=config_.swap_ms)wait=(std::max)(wait,swap-keep);
        if(keep+wait<swap){gaps.push_back({front.entry_portal,a->key,b->key,since});continue;}
        const auto front_actor=front.actor;front.actor=back->actor;back->actor=front_actor;
        const auto* entry=portal(*building,front.entry_portal);
        const float shared=entry?door_cost(w,*entry,config_):0;
        front.placement_cost=to_front*config_.approach_weight+front.red_cost+shared;
        back->placement_cost=to_back*config_.approach_weight+back->red_cost+shared;
        ++next.generation;front.order.assignment.generation=back->order.assignment.generation=next.generation;
    }
    gaps_=std::move(gaps);
    // A cost timeout cannot make an absent point man lead an entry. When he
    // has stopped closing, promote a captured rear member who has actually
    // reached his own stack cell. Both actors must have finite graph routes
    // to the exchanged cells; the native publisher still checks restrictions.
    const auto formation_elapsed=w.now_ms>=next.formed_at_ms?w.now_ms-next.formed_at_ms:0;
    if(formation_elapsed>=config_.swap_ms)for(unsigned rank=0;rank<2;++rank){
        for(auto& front:next.slots){
            if(front.watcher||front.order.assignment.slot!=rank)continue;
            const auto* a=actor(w,front.actor);if(!a||arrived_at(*a,front,config_))continue;
            const auto closing=std::find_if(closing_.begin(),closing_.end(),[&](const Closing& c){
                return c.actor==a->key&&c.node==front.order.goal_node;});
            if(closing==closing_.end()||w.now_ms<closing->progressed_ms||
                w.now_ms-closing->progressed_ms<config_.swap_ms)continue;
            auto best=next.slots.end();float best_cost=std::numeric_limits<float>::infinity();
            for(auto back=next.slots.begin();back!=next.slots.end();++back){
                if(back->watcher||back->entry_portal!=front.entry_portal||back->order.assignment.slot<2)continue;
                const auto* b=actor(w,back->actor);
                if(!b||!arrived_at(*b,*back,config_)||b->reloading||b->weapon.state==WeaponState::reloading)continue;
                const auto denied=[&](ActorKey key,std::uint32_t node){return std::any_of(rejected_.begin(),rejected_.end(),
                    [&](const RejectedSlot& r){return r.member.actor==key&&r.node==node;});};
                if(denied(a->key,back->order.goal_node)||denied(b->key,front.order.goal_node))continue;
                ActorPaths ap(*a),bp(*b);
                if(cancelled&&cancelled())throw StackCancelled{};
                const float to_back=ap.length(services,back->order.goal_node,config_.path_nodes);
                if(cancelled&&cancelled())throw StackCancelled{};
                const float to_front=bp.length(services,front.order.goal_node,config_.path_nodes);
                if(!std::isfinite(to_back)||!std::isfinite(to_front)||to_back<0||to_front<0)continue;
                const float cost=to_back+to_front;
                if(cost<best_cost||(cost==best_cost&&best!=next.slots.end()&&back->actor.id<best->actor.id)){
                    best=back;best_cost=cost;}
            }
            if(best==next.slots.end())continue;
            std::swap(front.actor,best->actor);
            ++next.generation;
            queued_entry_=true; // wait for the promoted pair's captured arrival
        }
    }
    for(auto& slot:next.slots)slot.order.assignment.generation=next.generation;
    std::size_t arrived=0,entrants=0;const auto elapsed=w.now_ms>=next.formed_at_ms?w.now_ms-next.formed_at_ms:0;
    std::map<std::uint32_t,std::pair<std::size_t,std::size_t>> door_ready;
    std::map<std::uint32_t,bool> front_ready;
    std::vector<Closing> closing;
    for(auto& slot:next.slots) {
        const auto* a=actor(w,slot.actor);if(!a)continue;
        slot.arrived=arrived_at(*a,slot,config_);
        slot.delayed=!slot.arrived&&elapsed>=config_.straggler_ms;
        if(!slot.watcher){++entrants;if(slot.arrived)++arrived;
            auto& counts=door_ready[slot.entry_portal];++counts.first;if(slot.arrived)++counts.second;
            const float distance=flat(a->position,slot.order.goal_position);
            const auto old=std::find_if(closing_.begin(),closing_.end(),[&](const Closing& p){return p.actor==a->key&&p.node==slot.order.goal_node;});
            Closing progress=old==closing_.end()?Closing{a->key,slot.order.goal_node,distance,w.now_ms}:*old;
            if(distance+engine_config_.room.leader_progress_m<progress.best){progress.best=distance;progress.progressed_ms=w.now_ms;}
            closing.push_back(progress);
            const bool coming=w.now_ms>=progress.progressed_ms&&w.now_ms-progress.progressed_ms<config_.swap_ms;
            auto inserted=front_ready.emplace(slot.entry_portal,true);auto& ready=inserted.first->second;
            const auto rank=slot.order.assignment.slot;
            if(rank<2)ready=ready&&distance<=config_.form_tolerance_m&&!a->reloading&&a->weapon.state!=WeaponState::reloading;
            else if(coming){const auto* p=portal(*building,slot.entry_portal);
                if(rank==2&&distance>config_.entry_third_m)ready=false;
                if(p&&flat(a->position,p->centre)>=config_.pie_mid_m)ready=false;}}
        const auto geometry=slot.order;slot.order=base_order(w,*a,config_,engine_config_.order_lifetime_ms,next.generation);
        slot.order.goal_node=geometry.goal_node;slot.order.goal_position=geometry.goal_position;
        slot.order.role=geometry.role;slot.order.kind=geometry.kind;slot.order.look=geometry.look;
        slot.order.side=geometry.side;slot.order.assignment=geometry.assignment;slot.order.portal=geometry.portal;
        stack_look(slot,w,*building,request,config_);
        slot.order.pace=slot.arrived?Pace::stand:Pace::walk;
        if(slot.order.kind==OrderKind::stack&&flat(a->position,slot.order.goal_position)<=config_.jamb_final_m){
            slot.order.flags|=order_jamb;slot.order.speed_scale=config_.jamb_speed_scale;}
        if(!slot.watcher&&slot.order.assignment.slot==0&&flat(a->position,slot.order.goal_position)<=config_.jamb_final_m){
            slot.order.lean=float(slot.order.side)*config_.edge_lean;
            slot.order.pie={slot.order.look.value,true};}
        // Threat fire is independently gated by the executor. Placement never
        // fabricates an enemy position or grants a shot through static geometry.
    }
    closing_=std::move(closing);
    next.go_cost=float(entrants-arrived+next.unprojected)*config_.readiness_missing_cost;
    next.wait_cost=float(elapsed)/1000.f*config_.straggler_cost_per_second;
    if(elapsed>=config_.straggler_ms)next.wait_cost=(std::max)(next.wait_cost,next.go_cost);
    bool both_ready=door_ready.size()==2;
    for(const auto& door:door_ready)both_ready=both_ready&&
        door.second.first>=config_.minimum_door_members&&door.second.second>=config_.minimum_door_members&&front_ready[door.first];
    // Persist captured roster identity even when one member lacks a projection;
    // retrying that member must not reset the other men's claims or elapsed time.
    roster_.clear();for(const auto* a:members)roster_.push_back({a->key,a->ownership_generation,a->restrictions_revision});
    searched_=request.searched_rooms;
    permit_two_doors_=request.permit_two_doors;call_breacher_first_=request.call_breacher_first;
    epoch_=w.level_epoch;geometry_=w.geometry_revision;map_=w.map_hash;side_=side.side_id;
    mission_revision_=side.mission.revision;requested_portal_=request.entry_portal;
    bool split_advancing=next.two_doors&&door_ready.size()==2;
    if(split_advancing)for(const auto& door:door_ready){
        bool element_advancing=front_ready[door.first];
        if(!element_advancing)for(const auto& slot:next.slots)if(!slot.watcher&&slot.entry_portal==door.first&&
            slot.order.assignment.slot<2&&!slot.arrived){
            const auto progress=std::find_if(closing_.begin(),closing_.end(),[&](const Closing& c){return
                c.actor==slot.actor&&c.node==slot.order.goal_node;});
            if(progress!=closing_.end()&&w.now_ms>=progress->progressed_ms&&
                w.now_ms-progress->progressed_ms<config_.swap_ms)element_advancing=true;
        }
        split_advancing&=element_advancing;
    }
    if(next.two_doors&&!both_ready&&elapsed>=config_.straggler_ms&&!split_advancing) {
        // A split with too few arrivals is no longer a two-door plan. Pick the
        // best formed element, then use the same single-door cost rule. The
        // two-fit-men rule must not become a permanent veto after degradation.
        const auto has_point=[&](std::uint32_t id){return std::any_of(next.slots.begin(),next.slots.end(),[&](const RoomStackSlot& slot){
            return !slot.watcher&&slot.entry_portal==id&&slot.order.assignment.slot==0&&slot.order.goal_node!=invalid_node;});};
        auto best=door_ready.end();for(auto it=door_ready.begin();it!=door_ready.end();++it)
            if(has_point(it->first)&&(best==door_ready.end()||it->second.second>best->second.second))best=it;
        if(best!=door_ready.end()) {
            single_portal_=best->first;next.two_doors=false;++next.generation;gaps_.clear();
            next.slots.erase(std::remove_if(next.slots.begin(),next.slots.end(),[&](const RoomStackSlot& slot){return slot.entry_portal!=single_portal_;}),next.slots.end());
            next.unprojected=members.size()-next.slots.size();state_=next;
            last_retry_ms_=w.now_ms>=config_.swap_ms?w.now_ms-config_.swap_ms:0;
            return update_impl(w,side,request,services,cancelled);
        }
    }
    // A late heel is not a point. Readiness requires a captured lead slot at
    // every active opening, plus the heel when that opening has a pair.
    std::map<std::uint32_t,std::set<unsigned>> active_ranks;
    for(const auto& slot:next.slots)if(!slot.watcher)active_ranks[slot.entry_portal].insert(slot.order.assignment.slot);
    for(const auto& entry:active_ranks)if(!entry.second.count(0)||
        (door_ready[entry.first].first>=2&&!entry.second.count(1)))front_ready[entry.first]=false;
    const bool all_arrived=arrived==entrants&&!next.unprojected;
    const bool flowing=!next.unprojected&&!front_ready.empty()&&std::all_of(front_ready.begin(),front_ready.end(),[](const std::pair<const std::uint32_t,bool>& p){return p.second;});
    const bool lead_ready=!front_ready.empty()&&std::all_of(front_ready.begin(),front_ready.end(),
        [](const std::pair<const std::uint32_t,bool>& p){return p.second;});
    const bool ready=entrants>0&&lead_ready&&(!queued_entry_||arrived>=(std::min)(entrants,std::size_t(2)))&&
        (!next.two_doors||both_ready)&&
        (all_arrived||flowing||next.wait_cost>=next.go_cost);
    const bool was_ready=next.phase==RoomStackPhase::ready;next.phase=ready?RoomStackPhase::ready:RoomStackPhase::forming;
    if(ready&&!was_ready) {
        next.phase=RoomStackPhase::ready;next.reason=all_arrived?StackReason::arrivals:StackReason::straggler_cost;
        Event e;e.kind=events::stack_formed;e.room=request.target_room;e.detail=static_cast<std::uint32_t>(arrived);next.events.push_back(e);
    }
    next.exposure=prices.stats();state_=next;return next;
}
}}}
