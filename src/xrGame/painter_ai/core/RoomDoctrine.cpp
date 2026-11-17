#include "RoomDoctrine.h"
#include "RoomGoals.h"
#include "RoomThreatEvidence.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace painter { namespace combat { namespace core {
namespace {
constexpr float pi=3.14159265358979323846f;
struct Cancelled{};
void poll(const std::function<bool()>& stop){if(stop&&stop())throw Cancelled{};}
Vec3 add(Vec3 a,Vec3 b,float k=1){return {a.x+k*b.x,a.y+k*b.y,a.z+k*b.z};}
float dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
float length(Vec3 a){return std::sqrt(dot(a,a));}
Vec3 unit(Vec3 a){const float n=length(a);return n>.00001f?Vec3{a.x/n,a.y/n,a.z/n}:Vec3{};}
float flat2(Vec3 a,Vec3 b){const float x=a.x-b.x,z=a.z-b.z;return x*x+z*z;}
const ActorSnapshot* actor(const WorldSnapshot& w,ActorKey key){for(const auto& a:w.actors)if(a.key==key)return &a;return nullptr;}
const Portal* portal(const Building& b,std::uint32_t id){for(const auto& p:b.portals)if(p.id==id)return &p;return nullptr;}
bool inside(const Room& room,Vec3 p,float vertical){return finite(p)&&p.x>=room.min.x&&p.x<=room.max.x&&p.z>=room.min.z&&p.z<=room.max.z&&std::abs(p.y-room.floor_y)<=vertical;}
RoomDoorFrame door_frame(const Portal& p,std::uint32_t target){RoomDoorFrame f;f.centre=p.centre;f.normal=unit({p.normal.x,0,p.normal.z});
    if(p.room_a==target)f.normal=add({},f.normal,-1);f.tangent=unit({p.jamb_b.x-p.jamb_a.x,0,p.jamb_b.z-p.jamb_a.z});
    if(length(f.tangent)<.5f)f.tangent={f.normal.z,0,-f.normal.x};f.half_width=p.half_width;f.thickness=p.wall_depth;return f;}
bool eligible(const ActorSnapshot& a){return valid_key(a.key)&&a.alive&&!a.wounded&&std::isfinite(a.health)&&a.health>0&&!a.yield_reasons&&(a.flags&flag_admitted)&&
    a.ownership_generation&&a.restrictions_revision&&a.current_position_accessible&&a.node_id!=invalid_node&&finite(a.position);}
bool clear_ray(const EngineServices& s,Vec3 from,Vec3 to,const std::function<bool()>& stop){if(!s.rays)return false;
    poll(stop);Segment segment{from,to};float hit=-2;s.rays(&segment,&hit,1);poll(stop);return hit==-1.f;}
bool sees(const ActorSnapshot& a,Vec3 point,const WorldSnapshot& w,const Config& c,const EngineServices& s,const std::function<bool()>& stop){
    if(!eligible(a)||!fresh(a.observed_at_ms,w.now_ms,c.snapshot_max_age_ms)||!finite(a.head_dir)||length(a.head_dir)<.5f)return false;
    const auto eye=add(a.position,{0,c.knowledge.eye_m,0});const auto delta=add(point,eye,-1);const float range=length(delta);
    return range>=.001f&&range<=c.room.see_range_m&&dot(unit(a.head_dir),unit(delta))>=std::cos(c.room.check_half_deg*pi/180.f)&&clear_ray(s,eye,point,stop);
}
std::uint32_t other_room(const Portal& p,std::uint32_t room){return p.room_a==room?p.room_b:p.room_a;}
bool searched(const RoomDoctrineRequest& r,const RoomDoctrineResult&,std::uint32_t id){
    // Physical local coverage is not a communicated safe route or doorway.
    return id==invalid_room||std::find(r.stack.searched_rooms.begin(),r.stack.searched_rooms.end(),id)!=r.stack.searched_rooms.end();
}
float red_cost(const Building& b,const EngineServices& s,const RoomDoctrineRequest& r,const RoomDoctrineResult& state,
    const RoomConfig& c,std::uint32_t room,std::uint32_t node,Vec3 pos,std::uint32_t watching,const std::function<bool()>& stop,RoomExposureCost* prices=nullptr,
    RoomExposureCostReceipt* receipt=nullptr,bool held=false){
    float cost=0,prior=0;
    for(const auto& p:b.portals){poll(stop);if(p.room_a!=room&&p.room_b!=room)continue;
        if(p.id!=watching&&!searched(r,state,other_room(p,room))){const auto& table=p.room_a==room?p.lit_a:p.lit_b;
            const auto it=std::lower_bound(table.begin(),table.end(),node,[](const Lit& l,std::uint32_t n){return l.node<n;});
            if(it!=table.end()&&it->node==node){const float value=c.exposure_cost_m*float(it->standing)/255.f;cost+=value;prior+=value;}
            else if(!s.buildings->lit_complete){cost+=c.unknown_cost_m;prior+=c.unknown_cost_m;}
        }
        if(std::abs(pos.y-p.centre.y)>c.floor_tolerance_m+c.look_height_m)continue;
        const auto local=room_frame_local(door_frame(p,other_room(p,room)),pos);
        if(std::abs(local.x)<p.half_width+c.arrival_m&&std::abs(local.z)<p.wall_depth+c.wall_offset_m)cost+=c.funnel_cost_m;
    }
    if(prices){auto quote=prices->price(RoomExposureRole::doctrine,b.id,room,node,pos,prior,held);
        if(prices->was_cancelled())throw Cancelled{};if(receipt)*receipt=quote;cost+=quote.cost-prior;}
    return cost;
}
bool accepted_execution(const ActorSnapshot& a,const WorldSnapshot& w,const RoomDoctrineRequest& r){
    if(!a.last_applied.order_seq||a.last_applied.at_ms>a.observed_at_ms||a.last_applied.at_ms>w.now_ms)return false;
    for(const auto& o:r.applied_orders)if(o.actor==a.key&&o.order_seq==a.last_applied.order_seq&&o.ownership_generation==a.ownership_generation&&
        o.restrictions_revision==a.restrictions_revision&&o.level_epoch==w.level_epoch&&o.geometry_revision==w.geometry_revision&&
        o.issued_at_ms<=a.last_applied.at_ms&&o.expires_at_ms>a.last_applied.at_ms)return true;
    return false;
}
RoomPieChoice pie_scores(const ActorSnapshot& a,const Contact& enemy,const Portal& p,std::uint32_t target,
    std::size_t men,std::size_t casualties,float unseen,float door_time,bool fired,const RoomConfig& c){
    RoomPieChoice out;out.actor=a.key;out.enemy=enemy.key;
    const Vec3 facing{enemy.facing.x,0,enemy.facing.z};float best=181.f;
    if(finite(facing)&&length(facing)>.001f)for(const auto& point:{a.position,p.centre}){
        Vec3 delta{point.x-enemy.position.x,0,point.z-enemy.position.z};if(length(delta)>.001f)
            best=(std::min)(best,std::acos((std::max)(-1.f,(std::min)(1.f,dot(unit(facing),unit(delta)))))*180.f/pi);
    }
    if(fired||best<=c.pie_unaware_deg)out.awareness=PieAwareness::aware;
    else if(best<=180.f)out.awareness=best<=c.pie_partly_deg?PieAwareness::partly:PieAwareness::unaware;
    const float distance=std::sqrt(flat2(a.position,enemy.position)),door_distance=std::sqrt(flat2(p.centre,enemy.position));
    out.engage=(out.awareness==PieAwareness::unaware?c.pie_engage_unaware:out.awareness==PieAwareness::partly?c.pie_engage_partly:0.f)+
        (men<=1?c.pie_engage_solo:men==2?c.pie_engage_duo:0.f)+
        (distance<=c.pie_near_m?c.pie_engage_near:distance<=c.pie_mid_m?c.pie_engage_mid:0.f);
    out.back_out=(out.awareness==PieAwareness::aware?c.pie_engage_aware:0.f)+(men>=4?c.pie_engage_team:0.f)+
        (distance>c.pie_mid_m?c.pie_engage_far:0.f)+(men>=3?c.pie_engage_unseen*unseen:0.f);
    const bool alert=fired||out.awareness==PieAwareness::aware;
    const auto local=room_frame_local(door_frame(p,target),enemy.position);
    const bool in_line=std::abs(local.x)<=p.half_width+c.leader_set_m&&local.z>p.wall_depth;
    out.enter=c.pie_call_men*(float((std::min)(men,std::size_t(5)))-3.f)+
        (door_distance<=c.pie_near_m?c.pie_call_near:door_distance<=c.pie_mid_m?c.pie_call_mid:-c.pie_call_far)+
        (alert?-c.pie_call_alert:(out.awareness==PieAwareness::unaware||out.awareness==PieAwareness::partly?c.pie_call_surprise:0.f))-c.pie_call_unseen*unseen+c.pie_call_door*door_time-
        c.pie_call_casualty*float(casualties)-(in_line?c.pie_call_line:0.f);
    out.overwatch=(in_line?c.pie_call_line*.5f:0.f)+(door_distance>=c.pie_mid_m?c.pie_call_far:0.f)+
        c.pie_call_unseen_over*unseen+(alert?c.pie_call_alert:0.f)+(men>=2?c.pie_call_half_line:0.f)+c.pie_call_casualty*float(casualties)*.5f;
    return out;
}
void doorway(Order& o,const RoomDoorFrame& f){o.portal={f.centre,f.tangent,f.normal,f.half_width,f.thickness,true};}
void fresh_order(Order& o,const WorldSnapshot& w,const ActorSnapshot& a,const Config& c){o.actor=a.key;o.level_epoch=w.level_epoch;o.snapshot_seq=w.sequence;
    o.geometry_revision=w.geometry_revision;o.ownership_generation=a.ownership_generation;o.restrictions_revision=a.restrictions_revision;
    o.order_seq=0;o.issued_at_ms=w.now_ms;o.expires_at_ms=saturated_after(w.now_ms,c.order_lifetime_ms);o.fire={};o.throw_order={};
    auto immediate=first_response(a,c,w.now_ms,w.level_epoch,w.geometry_revision,0);
    if(immediate.look.kind==LookKind::enemy){o.look=immediate.look;o.fire=immediate.fire;}}
using Foe=RoomKnownFoe;
std::vector<Foe> known(const WorldSnapshot& w,const SideSnapshot& side,const KnowledgeFrame* k,const Room& room,const Config& c,const std::function<bool()>& stop){
    return room_known_foes(w,side,k,room,c,[&](){poll(stop);});
}
std::size_t known_wounded(const WorldSnapshot&w,const SideSnapshot&side,const KnowledgeFrame*k,const Room&room,const Config&c,const std::function<bool()>&stop){
    struct Status{ActorKey key;Vec3 point;TimeMs at=0;bool wounded=false;};std::vector<Status> facts;
    auto put=[&](ActorKey key,Vec3 point,TimeMs at,bool wounded){if(!valid_key(key)||std::find(side.members.begin(),side.members.end(),key)!=side.members.end()||
            !fresh(at,w.now_ms,c.room.finisher_keep_ms)||!finite(point))return;
        auto old=std::find_if(facts.begin(),facts.end(),[&](const Status&v){return v.key==key;});
        if(old==facts.end())facts.push_back({key,point,at,wounded});else if(at>old->at||(at==old->at&&!wounded))*old={key,point,at,wounded};};
    for(auto key:side.members){poll(stop);const auto*a=actor(w,key);if(!a||!eligible(*a)||a->level_epoch!=w.level_epoch||a->snapshot_seq!=w.sequence||!fresh(a->observed_at_ms,w.now_ms,c.snapshot_max_age_ms))continue;
        if(k)for(const auto&m:k->members)if(m.actor==key&&m.side==side.side_id&&m.ownership_generation==a->ownership_generation){
            for(const auto&f:m.foes){poll(stop);if(f.id.ghost())continue;if(f.down&&f.down_ms<=a->observed_at_ms)put(f.id.actor,f.wounded_position,f.down_ms,false);
                else if(f.wounded&&f.wounded_ms<=a->observed_at_ms)put(f.id.actor,f.wounded_position,f.wounded_ms,true);}break;}
        auto contact=[&](const Contact&t){if(t.observed_at_ms<=a->observed_at_ms)put(t.key,t.position,t.observed_at_ms,t.hostile&&t.alive_when_observed&&t.wounded);};
        contact(a->enemy);for(std::size_t n=0;n<a->visible_enemies.count&&n<a->visible_enemies.items.size();++n)contact(a->visible_enemies.items[n]);
    }
    std::size_t count=0;for(const auto&f:facts)if(f.wounded&&inside(room,f.point,c.knowledge.eye_m+c.room.floor_tolerance_m)){
        for(auto key:side.members){poll(stop);const auto*a=actor(w,key);if(a&&eligible(*a)&&flat2(a->position,f.point)<=c.room.see_range_m*c.room.see_range_m){++count;break;}}}
    return count;
}
}
RoomDoctrine::RoomDoctrine(Config c):config_(c),stack_(c),lane_cost_(c),leader_(c){}
void RoomDoctrine::reset(){stack_.reset();room_geometry_.reset();lane_cost_.reset();leader_.reset();result_={};men_.clear();throw_={};released_portals_.clear();interior_start_=false;epoch_=map_=geometry_=building_data_=sequence_=generation_=0;
    side_=mission_=building_=entry_=throw_serial_=0;target_=invalid_room;phase_at_=go_at_=last_shot_=last_contact_=pair_ready_at_=0;phase_=RoomPhase::invalid;
    heat_.clear();sight_cells_.clear();sight_cursor_=0;sight_geometry_complete_=false;assess_down_.clear();inspections_.clear();}
RoomDoctrineResult RoomDoctrine::update(const WorldSnapshot& w,const SideSnapshot& side,const RoomDoctrineRequest& request,
    const KnowledgeFrame* knowledge,const EngineServices& services,const std::function<bool()>& stop){
    RoomDoctrineResult bad;bad.reason=RoomReason::invalid_input;
    try{poll(stop);auto staged=*this;poll(stop);auto out=staged.update_impl(w,side,request,knowledge,services,stop);poll(stop);
        if(out.accepted)*this=std::move(staged);return out;
    }catch(const Cancelled&){bad.cancelled=true;bad.reason=RoomReason::cancelled;return bad;}
    catch(...){bad.reason=RoomReason::no_projection;return bad;}
}
RoomDoctrineResult RoomDoctrine::update_impl(const WorldSnapshot& w,const SideSnapshot& side,const RoomDoctrineRequest& request,
    const KnowledgeFrame* knowledge,const EngineServices& services,const std::function<bool()>& stop){
    RoomDoctrineResult bad;bad.reason=RoomReason::invalid_input;const auto& c=config_.room;
    const float checked[]={c.pie_radius_m,c.pie_advance_m,c.pie_lean,c.spacing_m,c.arrival_m,c.entry_arrival_m,c.check_half_deg,c.see_range_m,
        c.entry_walk_scale,c.entry_run_scale,c.jamb_speed_scale,c.projection_m,c.floor_tolerance_m,c.known_enemy_exclusion_m,c.look_height_m,
        c.entry_third_m,c.entry_heel_m,c.entry_follow_m,c.assess_radius_m,c.pie_call_men,c.door_hot_cost,c.wall_offset_m,c.jamb_offset_m,
        c.entry_zone_depth_m,c.jamb_final_m,c.frag_inside_m[0],c.pie_engage_aware,c.pie_call_unseen,c.pie_call_alert,c.pie_call_hysteresis,
        c.exposure_cost_m,c.unknown_cost_m,c.funnel_cost_m,c.path_unknown_cost_m,c.approach_weight,config_.knowledge.eye_m,
        c.pie_unaware_deg,c.pie_partly_deg,c.pie_near_m,c.pie_mid_m,c.pie_engage_hysteresis,c.pie_engage_unaware,c.pie_engage_partly,
        c.pie_engage_solo,c.pie_engage_duo,c.pie_engage_team,c.pie_engage_near,c.pie_engage_mid,c.pie_engage_far,c.pie_engage_unseen,
        c.pie_call_near,c.pie_call_mid,c.pie_call_far,c.pie_call_surprise,c.pie_call_unseen_over,c.pie_call_door,c.pie_call_casualty,
        c.pie_call_half_line,c.pie_call_line,c.leader_set_m,c.entry_zone_half_m,c.entry_behind_m,c.entry_ready_m,c.edge_lean};
    for(float v:checked)if(!std::isfinite(v)||v<0)return bad;
    for(float v:c.assess_angles_deg)if(!std::isfinite(v)||v<0||v>=90)return bad;
    for(float v:c.assess_radii_m)if(!std::isfinite(v)||v<=0)return bad;
    if(c.assess_radius_m<=0||c.pie_unaware_deg>c.pie_partly_deg||c.pie_partly_deg>180||c.pie_near_m>c.pie_mid_m)return bad;
    if(c.pie_radius_m<=0||c.pie_advance_m<=0||c.spacing_m<=0||c.arrival_m<=0||c.entry_arrival_m<=0||c.see_range_m<=0||
        c.check_half_deg>180||c.pie_lean>1||c.entry_walk_scale<.3f||c.entry_walk_scale>1.5f||c.jamb_speed_scale<.3f||c.jamb_speed_scale>1.5f)return bad;
    if(!w.level_epoch||!w.sequence||side.mission.kind!=MissionKind::clear_building||!services.buildings||!services.project||
        services.buildings->level_epoch!=w.level_epoch||services.buildings->map_hash!=w.map_hash)return bad;
    if(epoch_&&w.level_epoch<epoch_)return bad;
    if(epoch_==w.level_epoch&&(w.sequence<=sequence_||w.now_ms<result_.captured_at_ms))return bad;
    const Building* building=nullptr;for(const auto& b:services.buildings->buildings)if(b.id==side.mission.building){building=&b;break;}
    if(!building){bad.reason=RoomReason::no_geometry;return bad;}const Room* room=nullptr;
    for(const auto& r:building->rooms)if(r.id==request.stack.target_room){room=&r;break;}
    if(!room||!finite(room->min)||!finite(room->max)||!finite(room->centre)||!std::isfinite(room->floor_y)||room->max.x<=room->min.x||room->max.z<=room->min.z){bad.reason=RoomReason::no_geometry;return bad;}
    std::set<std::uint32_t> ids;for(auto key:side.members){poll(stop);if(!valid_key(key)||!ids.insert(key.id).second)return bad;const auto* a=actor(w,key);
        if(a&&(a->level_epoch!=w.level_epoch||a->snapshot_seq!=w.sequence||a->observed_at_ms>w.now_ms))return bad;}
    if(request.inspect_current_room){
        if(!services.buildings->locate||request.stack.entry_portal)return bad;
        std::size_t present=0;
        for(auto key:side.members){poll(stop);const auto* a=actor(w,key);
            if(!a)return bad;
            if(!a->alive||a->wounded)continue;
            if(!eligible(*a)||!fresh(a->observed_at_ms,w.now_ms,config_.snapshot_max_age_ms)||
                a->node_id==invalid_node||(!room->node_ids.empty()&&
                    !std::binary_search(room->node_ids.begin(),room->node_ids.end(),a->node_id))||
                services.buildings->locate(a->node_id)!=std::make_pair(building->id,room->id)||
                !inside(*room,a->position,c.floor_tolerance_m))return bad;
            ++present;
        }
        if(!present)return bad;
    }
    // If every participant in an unfinished entry/search was lost, an outside
    // replacement must start a new stack and PIE attempt. Reusing SEARCH would
    // make him cross an unproved door. An already captured interior replacement
    // can instead take an ordinary search sector below.
    const bool active_room=phase_==RoomPhase::entry||phase_==RoomPhase::search;
    bool survivor=false,outside_replacement=false;
    if(active_room){
        for(const auto& m:men_){poll(stop);const auto* a=actor(w,m.actor);
            if(!m.watcher&&a&&eligible(*a)&&a->level_epoch==w.level_epoch&&a->snapshot_seq==w.sequence&&
                fresh(a->observed_at_ms,w.now_ms,config_.snapshot_max_age_ms)&&
                a->ownership_generation==m.owner&&a->restrictions_revision==m.restrictions&&
                !room_care_owned(w,*a,request.applied_orders)&&
                std::find(side.members.begin(),side.members.end(),m.actor)!=side.members.end())survivor=true;}
        if(!survivor)for(auto key:side.members){poll(stop);const auto* a=actor(w,key);
            if(!a||!eligible(*a)||a->level_epoch!=w.level_epoch||a->snapshot_seq!=w.sequence||
                !fresh(a->observed_at_ms,w.now_ms,config_.snapshot_max_age_ms)||
                room_care_owned(w,*a,request.applied_orders))continue;
            const auto place=services.buildings->locate?services.buildings->locate(a->node_id):std::make_pair(invalid_room,invalid_room);
            if(place.first!=building->id||place.second!=room->id||!inside(*room,a->position,c.floor_tolerance_m)){
                outside_replacement=true;break;}}
    }
    const bool retry_entry=active_room&&!survivor&&outside_replacement;
    const bool changed=retry_entry||interior_start_!=request.inspect_current_room||epoch_!=w.level_epoch||map_!=w.map_hash||geometry_!=w.geometry_revision||building_data_!=services.buildings->data_version||side_!=side.side_id||mission_!=side.mission.revision||
        building_!=building->id||target_!=room->id||entry_!=request.stack.entry_portal;
    if(changed){const bool same_mission=epoch_==w.level_epoch&&map_==w.map_hash&&geometry_==w.geometry_revision&&building_data_==services.buildings->data_version&&side_==side.side_id&&mission_==side.mission.revision;
        auto cleared=same_mission?result_.cleared_rooms:std::vector<std::uint32_t>{};const auto gen=generation_+1;
        const auto next_throw=w.level_epoch==epoch_?throw_serial_:0;
        reset();generation_=gen;throw_serial_=next_throw;result_.cleared_rooms=std::move(cleared);interior_start_=request.inspect_current_room;
        phase_=interior_start_?RoomPhase::search:RoomPhase::form;phase_at_=w.now_ms;
        epoch_=w.level_epoch;map_=w.map_hash;geometry_=w.geometry_revision;building_data_=services.buildings->data_version;side_=side.side_id;mission_=side.mission.revision;building_=building->id;target_=room->id;entry_=request.stack.entry_portal;
        auto points=room->sectors;bool derived=points.empty();
        if(derived){const float inset=(std::min)(c.spacing_m,(std::min)(room->max.x-room->min.x,room->max.z-room->min.z)*.2f);
            points={{room->min.x+inset,room->floor_y,room->min.z+inset},{room->max.x-inset,room->floor_y,room->min.z+inset},
                {room->max.x-inset,room->floor_y,room->max.z-inset},{room->min.x+inset,room->floor_y,room->max.z-inset},room->centre};}
        if(room->node_ids.empty())for(std::size_t i=0;i<points.size();++i){poll(stop);if(finite(points[i]))result_.coverage.push_back({points[i],static_cast<std::uint32_t>(i+1),0,{},0,derived});}
    }
    RoomExposureCost prices(w,side,services.buildings,request.exposure?request.exposure:request.stack.exposure,stop);
    if(prices.was_cancelled())throw Cancelled{};
    const bool actual_nodes=!room->node_ids.empty();
    if(actual_nodes){RoomGeometryStamp stamp{w.level_epoch,w.map_hash,w.geometry_revision,services.buildings->data_version,building->id,room->id};
        const auto geometry=room_geometry_.step(stamp,*room,c,services,request.geometry_nodes_per_update?request.geometry_nodes_per_update:config_.cover_queries_per_slice,stop);poll(stop);
        result_.geometry_prepared=geometry.prepared;result_.geometry_pending=!geometry.prepared;
        result_.geometry_nodes_total=geometry.total;result_.geometry_nodes_processed=geometry.processed;result_.geometry_nodes_valid=geometry.valid;
        if(geometry.prepared&&result_.coverage.empty())for(const auto& p:geometry.samples)result_.coverage.push_back({p.position,p.sector,0,{},0,true});
    }else {result_.geometry_prepared=true;result_.geometry_pending=false;}
    sequence_=w.sequence;result_.accepted=true;result_.cancelled=false;result_.ready_for_next=false;result_.reason=RoomReason::none;
    result_.level_epoch=w.level_epoch;result_.snapshot_seq=w.sequence;result_.captured_at_ms=w.now_ms;result_.knowledge_seq=0;result_.generation=generation_;
    result_.map_hash=w.map_hash;result_.geometry_revision=w.geometry_revision;result_.mission_revision=side.mission.revision;
    result_.building=building->id;result_.room=room->id;result_.interior_start=interior_start_;
    result_.requested_entry_portal=request.stack.entry_portal;
    const auto previous_covered=result_.costs.covered;
    result_.events.clear();result_.transitions.clear();result_.slots.clear();result_.pie_choices.clear();result_.crossings.clear();result_.threat_attention.clear();result_.costs={};
    result_.lane={};result_.leader={};
    if(knowledge&&(!knowledge->accepted||knowledge->level_epoch!=w.level_epoch||knowledge->sequence>w.sequence))knowledge=nullptr;
    if(knowledge)result_.knowledge_seq=knowledge->sequence;
    auto transition=[&](RoomPhase next,RoomReason why){RoomTransition event;event.from=phase_;event.to=next;event.reason=why;event.at_ms=w.now_ms;
        for(auto key:side.members){poll(stop);const auto*a=actor(w,key);if(a&&a->level_epoch==w.level_epoch&&a->snapshot_seq==w.sequence&&finite(a->position)&&fresh(a->observed_at_ms,w.now_ms,config_.snapshot_max_age_ms))event.members.push_back({key,a->position,a->observed_at_ms});}
        result_.transitions.push_back(std::move(event));phase_=next;phase_at_=w.now_ms;result_.reason=why;
        if(next==RoomPhase::assess){assess_down_.clear();for(auto key:side.members){const auto* a=actor(w,key);if(a&&(!a->alive||a->wounded))assess_down_.push_back(key);}
            for(auto& m:men_){m.assess_portal=0;m.assess_step=0;m.assess_arrived_ms=0;}}};
    auto foes=known(w,side,knowledge,*room,config_,stop);result_.costs.known_foes=foes.size();
    // Exact source versions bound inspection memory; a new fact re-arms it,
    // disappearance/expiry drops it. There is no target-world query here.
    auto same_source=[](const RoomThreatAttention& task,const Foe& foe){return task.foe==foe.id&&task.source_at_ms==foe.at&&
        task.point.x==foe.point.x&&task.point.y==foe.point.y&&task.point.z==foe.point.z;};
    auto aware=[&](const ActorSnapshot& observer,const Foe& foe){
        const auto exact=[&](Vec3 p,TimeMs at){return at==foe.at&&p.x==foe.point.x&&p.y==foe.point.y&&p.z==foe.point.z;};
        const auto own=[&](const Contact& t){return !foe.id.ghost()&&t.key==foe.id.actor&&t.hostile&&t.alive_when_observed&&!t.wounded&&
            t.observed_at_ms<=observer.observed_at_ms&&exact(t.position,t.observed_at_ms);};
        if(own(observer.enemy))return true;
        for(std::size_t i=0;i<(std::min)(std::size_t(observer.visible_enemies.count),observer.visible_enemies.items.size());++i)
            if(own(observer.visible_enemies.items[i]))return true;
        if(knowledge)for(const auto& member:knowledge->members){poll(stop);if(member.actor!=observer.key||member.side!=side.side_id||
                member.ownership_generation!=observer.ownership_generation)continue;
            for(const auto& fact:member.foes)if(fact.id==foe.id&&!fact.down&&!fact.wounded){if(const auto* last=fact.last_known())
                if(exact(last->position,last->at_ms))return true;}break;}
        return false;};
    inspections_.erase(std::remove_if(inspections_.begin(),inspections_.end(),[&](const Inspection& task){
        poll(stop);return std::none_of(foes.begin(),foes.end(),[&](const Foe& f){return same_source(task.receipt,f);});}),inspections_.end());
    for(const auto& f:foes){result_.costs.visible_foes+=f.visible?1u:0u;last_contact_=(std::max)(last_contact_,f.at);}
    result_.costs.known_wounded=known_wounded(w,side,knowledge,*room,config_,stop);
    bool fresh_hit=false;for(auto key:side.members){poll(stop);const auto* a=actor(w,key);if(!a)continue;
        if(a->last_applied.fired_since_previous&&accepted_execution(*a,w,request))last_shot_=(std::max)(last_shot_,a->last_applied.at_ms);
        if(a->hits.count<=a->hits.items.size())for(std::size_t i=0;i<a->hits.count;++i)fresh_hit|=fresh(a->hits.items[i].at_ms,w.now_ms,c.assess_fresh_hit_ms);}
    const auto heat_retention=(std::max)(c.door_hot_ms,c.crossing_gap_ms);
    if(knowledge&&heat_retention)for(const auto& member:knowledge->members){poll(stop);const auto* a=actor(w,member.actor);
        if(!a||member.side!=side.side_id||member.ownership_generation!=a->ownership_generation||
            std::find(side.members.begin(),side.members.end(),member.actor)==side.members.end())continue;
        const auto& hit=member.hit_from;if(!hit.valid||hit.observer!=member.actor||!fresh(hit.at_ms,w.now_ms,heat_retention)||!finite(hit.origin)||!finite(hit.point)||!finite(hit.direction))continue;
        for(const auto& p:building->portals){poll(stop);if(!p.doorway||(p.room_a!=room->id&&p.room_b!=room->id))continue;
            const auto f=door_frame(p,room->id);const auto origin=room_frame_local(f,hit.origin);
            const float toward=dot(hit.direction,f.normal);if(origin.z>=0||toward<=.00001f)continue;
            const float t=-origin.z/toward;const auto crossing=add(hit.origin,hit.direction,t);
            if(std::abs(room_frame_local(f,crossing).x)>p.half_width+c.entry_zone_half_m||
                crossing.y<room->floor_y-c.floor_tolerance_m||crossing.y>room->floor_y+config_.knowledge.eye_m+c.floor_tolerance_m)continue;
            auto old=std::find_if(heat_.begin(),heat_.end(),[&](const DoorHeat& h){return h.portal==p.id;});
            if(old==heat_.end())heat_.push_back({p.id,hit.at_ms,hit.origin,hit.point});
            else if(hit.at_ms>=old->at_ms)*old={p.id,hit.at_ms,hit.origin,hit.point};
        }
    }
    heat_.erase(std::remove_if(heat_.begin(),heat_.end(),[&](const DoorHeat& h){return !heat_retention||!fresh(h.at_ms,w.now_ms,heat_retention);}),heat_.end());
    auto heat_cost=[&](std::uint32_t id){float cost=0;for(const auto& h:heat_)if(h.portal==id&&c.door_hot_ms)
        cost=(std::max)(cost,c.door_hot_cost*(std::max)(0.f,1.f-float(w.now_ms-h.at_ms)/float(c.door_hot_ms)));return cost;};
    for(const auto& h:heat_)result_.costs.door_hot=(std::max)(result_.costs.door_hot,heat_cost(h.portal));
    // Actual head direction plus a clear ray earns coverage; issuing a look
    // order, elapsed time, a missing ray service or an enemy census never does.
    const auto sight_age=last_contact_&&fresh(last_contact_,w.now_ms,c.known_enemy_ms)?c.seen_red_ms:c.seen_ms;
    for(auto& sector:result_.coverage){poll(stop);for(auto key:side.members){const auto* a=actor(w,key);if(!a||!eligible(*a)||!finite(a->head_dir)||length(a->head_dir)<.5f)continue;
            if(!fresh(a->observed_at_ms,w.now_ms,config_.snapshot_max_age_ms))continue;
            const Vec3 eye=add(a->position,{0,config_.knowledge.eye_m,0}),point=add(sector.point,{0,c.look_height_m,0}),delta=add(point,eye,-1);
            const float range=length(delta);if(range>c.see_range_m||range<.001f||dot(unit(a->head_dir),unit(delta))<std::cos(c.check_half_deg*pi/180.f))continue;
            if(clear_ray(services,eye,point,stop)){sector.seen_at_ms=a->observed_at_ms;sector.observer=a->key;sector.observer_owner=a->ownership_generation;break;}}
        if(sector.seen_at_ms&&fresh(sector.seen_at_ms,w.now_ms,sight_age))++result_.costs.covered;}
    const auto observed_cells=[&](bool full_nav){
        std::vector<RoomCellFact> cells;
        auto append=[&](std::uint64_t sample,ActorKey observer,std::uint64_t owner,TimeMs observed){
            if(!valid_key(observer)||!owner||!observed||!fresh(observed,w.now_ms,sight_age))return;
            RoomCellFact fact;fact.side=side.side_id;fact.building=building->id;fact.room=room->id;
            fact.mission_revision=side.mission.revision;fact.level_epoch=w.level_epoch;fact.map_hash=w.map_hash;
            fact.geometry_revision=w.geometry_revision;fact.attempt=generation_;fact.sample=sample;
            fact.observer=observer;fact.observer_owner=owner;fact.observed_at_ms=observed;cells.push_back(fact);};
        if(full_nav)for(const auto& cell:sight_cells_){poll(stop);append((std::uint64_t(1)<<63)|cell.node,cell.observer,cell.observer_owner,cell.seen_ms);}
        else for(const auto& sector:result_.coverage){poll(stop);append(sector.sector,sector.observer,sector.observer_owner,sector.seen_at_ms);}
        return cells;};
    result_.costs.total=result_.coverage.size();
    auto all_seen=[&](){return result_.costs.total&&result_.costs.covered==result_.costs.total;};
    auto own_throw=[&](const ActorSnapshot& a){return throw_.serial&&!throw_.finished&&a.key==throw_.actor&&a.ownership_generation==throw_.owner&&
        a.throw_state.serial==throw_.serial&&(a.throw_state.phase==1||a.throw_state.phase==2)&&
        a.throw_state.at_ms>=throw_.requested_ms&&a.throw_state.at_ms<=w.now_ms&&
        w.now_ms<(throw_.armed_ms?saturated_after(throw_.armed_ms,c.frag_release_cap_ms):saturated_after(throw_.requested_ms,c.frag_prefire_cap_ms))&&
        eligible(a);};
    // A release proposal is not a native application. Preserve per-portal
    // commitment only after an exact applied entry receipt or observed crossing;
    // a point lost before either proof must get a new FORM/PIE pair.
    for(const auto& m:men_)if(m.released&&!m.interior_bootstrap){const auto* a=actor(w,m.actor);
        if(m.crossed&&m.room_membership==Evidence::yes){released_portals_.insert(m.portal);continue;}
        if(!a||a->ownership_generation!=m.owner||!accepted_execution(*a,w,request))continue;
        for(const auto& applied:request.applied_orders)if(applied.actor==m.actor&&applied.order_seq==a->last_applied.order_seq&&
            applied.kind==OrderKind::entry&&(applied.flags&order_committed_entry)&&
            applied.assignment.generation==generation_&&applied.assignment.portal==m.portal&&
            applied.assignment.slot==m.order.assignment.slot&&applied.goal_node==m.order.goal_node&&
            distance_squared(applied.goal_position,m.order.goal_position)<.0001f){
            released_portals_.insert(m.portal);break;}}
    // Preserve surviving committed assignments. A new identity gets no old
    // lease or body goal; additions join on the next room, with a safe response.
    men_.erase(std::remove_if(men_.begin(),men_.end(),[&](const Man& m){const auto* a=actor(w,m.actor);return !a||!eligible(*a)||
        a->ownership_generation!=m.owner||a->restrictions_revision!=m.restrictions||room_care_owned(w,*a,request.applied_orders)||
        std::find(side.members.begin(),side.members.end(),m.actor)==side.members.end();}),men_.end());
    if(phase_==RoomPhase::search&&services.buildings->locate)for(auto key:side.members){poll(stop);
        if(std::any_of(men_.begin(),men_.end(),[&](const Man& m){return m.actor==key;}))continue;
        const auto* a=actor(w,key);if(!a||!eligible(*a)||a->level_epoch!=w.level_epoch||a->snapshot_seq!=w.sequence||
            !fresh(a->observed_at_ms,w.now_ms,config_.snapshot_max_age_ms)||a->node_id==invalid_node||
            room_care_owned(w,*a,request.applied_orders))continue;
        const auto place=services.buildings->locate(a->node_id);
        if(place.first!=building->id||place.second!=room->id||!inside(*room,a->position,c.floor_tolerance_m))continue;
        const auto first=first_response(*a,config_,w.now_ms,w.level_epoch,w.geometry_revision,0);
        if(first.kind==OrderKind::yield||first.kind==OrderKind::reload)continue;
        Man m;m.actor=a->key;m.owner=a->ownership_generation;m.restrictions=a->restrictions_revision;
        m.order=first;m.order.kind=OrderKind::search;m.order.role=roles::search_sector;
        m.order.goal_node=a->node_id;m.order.goal_position=a->position;m.order.assignment.generation=generation_;
        // Entry 0 means formation selected the actual doorway. Retain that
        // captured doorway for an interior replacement when available.
        const auto valid_entry=[&](std::uint32_t id){const auto* p=portal(*building,id);
            return p&&p->doorway&&p->traversable&&(p->room_a==room->id||p->room_b==room->id);};
        m.portal=valid_entry(entry_)?entry_:0;
        if(!m.portal){
            const Man* nearest=nullptr;float best=0;
            for(const auto& prior:men_){const auto* peer=actor(w,prior.actor);
                if(prior.watcher||!peer||!valid_entry(prior.portal))continue;
                const float distance=flat2(a->position,peer->position);
                if(!nearest||distance<best||(distance==best&&prior.portal<nearest->portal)){
                    nearest=&prior;best=distance;}}
            if(nearest)m.portal=nearest->portal;
        }
        m.order.assignment.portal=m.portal;
        m.stack_node=a->node_id;m.stack_goal=a->position;
        m.released=m.crossed=m.arrived=m.goal_ready=true;m.room_membership=Evidence::yes;
        m.interior_bootstrap=interior_start_;
        men_.push_back(std::move(m));}
    // RoomStack owns slot identity. Its vector can retain a heel or watcher
    // before re-adding a point; vector order is not entry rank.
    if(phase_==RoomPhase::form||phase_==RoomPhase::pie)
        for(auto& m:men_)if(!m.watcher)m.rank=m.order.assignment.slot;
    // An uncommitted element that lost its point must get a new captured
    // formation and PIE pass. Do not silently make the surviving heel point.
    if(phase_==RoomPhase::entry){
        std::set<std::uint32_t> active_portals,with_point;
        for(const auto& m:men_)if(!m.watcher){active_portals.insert(m.portal);if(m.rank==0)with_point.insert(m.portal);}
        if(std::any_of(active_portals.begin(),active_portals.end(),[&](std::uint32_t id){
            return !with_point.count(id)&&!released_portals_.count(id);}))
            transition(RoomPhase::form,RoomReason::forming);
    }
    // Bind execution before this update advances a slice or changes a goal.
    // A reached old slice is evidence even when the next slice differs.
    std::vector<Order> captured_primary_orders;
    for(const auto& m:men_)for(const auto& o:request.applied_orders)if(o.actor==m.actor&&o.assignment.generation==generation_&&
        o.assignment.portal==m.portal&&o.assignment.slot==m.order.assignment.slot&&o.goal_node==m.order.goal_node&&distance_squared(o.goal_position,m.order.goal_position)<.0001f)
        captured_primary_orders.push_back(o);
    std::vector<std::pair<ActorKey,TimeMs>> observed_progress;
    if(result_.costs.covered>previous_covered)for(const auto& sector:result_.coverage){const auto* a=actor(w,sector.observer);
        if(a&&sector.seen_at_ms==a->observed_at_ms)observed_progress.emplace_back(a->key,sector.seen_at_ms);}
    bool leader_updated=false;
    const auto update_leader=[&](bool ready){if(leader_updated)return;leader_updated=true;
        RoomLeaderRequest lead;lead.attempt=generation_;lead.room=room->id;
        lead.phase=phase_==RoomPhase::form?LeaderPhase::form:phase_==RoomPhase::pie?LeaderPhase::pie:
            phase_==RoomPhase::entry?LeaderPhase::entry:LeaderPhase::inactive;
        bool already_released=false;lead.accepted_orders=captured_primary_orders;
        for(const auto& m:men_){lead.members.push_back(m.actor);already_released|=m.released;
            if(!m.watcher&&!m.rank&&!valid_key(lead.point))lead.point=m.actor;
        }
        const auto fact=[&](ActorKey observer,TimeMs at){RoomLeaderFact value;for(const auto& o:lead.accepted_orders){const auto* a=actor(w,o.actor);
                if(a&&(!valid_key(observer)||a->key==observer)&&accepted_execution(*a,w,request)){value.valid=true;value.observer=a->key;value.ownership_generation=a->ownership_generation;
                    value.level_epoch=w.level_epoch;value.snapshot_seq=w.sequence;value.geometry_revision=w.geometry_revision;
                    value.attempt=generation_;value.at_ms=valid_key(observer)?at:a->observed_at_ms;break;}}return value;};
        if(ready)lead.phase_ready=fact({},0);
        for(const auto& progress:observed_progress){lead.progress=fact(progress.first,progress.second);if(lead.progress.valid)break;}
        result_.leader=leader_.update(w,side,lead,stop);if(result_.leader.cancelled)throw Cancelled{};poll(stop);
        // Execution capture follows publication. The primary release is already
        // irreversible in that gap; never let a late leader call restart it.
        if(already_released){result_.leader.released=true;result_.leader.suggestion=LeaderCall::none;}
        if(result_.leader.accepted&&result_.leader.execution_confirmed&&!result_.leader.released)
            result_.costs.leader=(result_.leader.up_cost_fraction+result_.leader.go_cost_fraction)*c.readiness_missing_cost+
                result_.leader.no_shot_cost_fraction*c.pie_call_alert;
    };
    if(phase_==RoomPhase::form){auto stack_request=request.stack;stack_request.applied_orders=request.applied_orders;if(request.exposure)stack_request.exposure=request.exposure;
        bool stack_stopped=false;auto stack_stop=[&]{stack_stopped=stack_stopped||(stop&&stop());return stack_stopped;};
        auto stack=stack_.update(w,side,stack_request,services,stack_stop);if(stack_stopped)throw Cancelled{};poll(stop);
        result_.costs.go=stack.go_cost;result_.costs.wait=stack.wait_cost;result_.costs.unprojected=stack.unprojected;
        if(stack.phase!=RoomStackPhase::invalid){result_.formation=stack;result_.formation_snapshot_seq=w.sequence;result_.formation_at_ms=w.now_ms;
            men_.clear();bool slots_valid=true;std::map<std::uint32_t,std::set<std::size_t>> occupied;
            for(const auto& slot:stack.slots){poll(stop);const auto* a=actor(w,slot.actor);if(!a)continue;
                Man m;m.actor=slot.actor;m.owner=a->ownership_generation;m.restrictions=a->restrictions_revision;m.order=slot.order;m.portal=slot.entry_portal;
                m.rank=slot.order.assignment.slot;m.watcher=slot.watcher;
                if(!m.watcher&&(m.rank==255||!occupied[m.portal].insert(m.rank).second))slots_valid=false;
                m.order.assignment.generation=generation_;m.stack_goal=slot.order.goal_position;m.stack_node=slot.order.goal_node;men_.push_back(m);}
            for(const auto& element:occupied)if(!element.second.count(0))slots_valid=false;
            if(!slots_valid)men_.clear();
            result_.events=stack.events;
            if(stack.phase==RoomStackPhase::ready&&!slots_valid)stack.phase=RoomStackPhase::forming;
            result_.formation.phase=stack.phase;}
        result_.reason=stack.phase==RoomStackPhase::invalid?RoomReason::no_projection:RoomReason::forming;
        update_leader(stack.phase==RoomStackPhase::ready);
        // A leader's suggestion does not turn an unformed physical pair into
        // a completed approach. The stack may promote a reachable replacement.
        if(stack.phase==RoomStackPhase::ready)transition(RoomPhase::pie,RoomReason::arrived);
    }
    RoomGoalCache projection_cache;
    auto probe_goal=[&](const Man& m,Vec3 desired,bool destination_inside,RoomGoalValue& out){poll(stop);const auto* a=actor(w,m.actor);if(!a)return false;
        AuthoredRoomGoal input;input.kind=m.order.kind;input.room=room->id;input.position=desired;input.max_horizontal_m=c.projection_m;input.max_vertical_m=c.floor_tolerance_m;
        RoomGoalStamp stamp{w.level_epoch,w.map_hash,w.geometry_revision};RoomGoalJob job(stamp,{input},c);
        job.step(1,stamp,projection_cache,services.project,services.node_position);poll(stop);const auto& value=job.results().front();
        if(value.status!=RoomGoalStatus::Ready)return false;
        if(destination_inside){if(!inside(*room,value.value.position,c.floor_tolerance_m)||!services.buildings->locate)return false;
            const auto located=services.buildings->locate(value.value.node);if(located.first!=building->id||located.second!=room->id)return false;}
        // Static graph reach is per actor. It is not a native restriction receipt.
        if(destination_inside&&actual_nodes){if(!services.path_length)return false;poll(stop);
            const float route=services.path_length(a->node_id,value.value.node,c.path_nodes);poll(stop);
            if(!std::isfinite(route)||route<0)return false;}
        if(std::find(m.rejected.begin(),m.rejected.end(),value.value.node)!=m.rejected.end())return false;
        for(const auto& other:men_)if(other.actor!=m.actor&&other.goal_ready&&other.order.goal_node==value.value.node)return false;
        out=value.value;return true;};
    auto commit_goal=[](Man& m,const RoomGoalValue& value){m.order.goal_node=value.node;m.order.goal_position=value.position;m.goal_ready=true;};
    auto project_goal=[&](Man& m,Vec3 desired,bool destination_inside){RoomGoalValue value;if(!probe_goal(m,desired,destination_inside,value))return false;commit_goal(m,value);return true;};
    auto destination_cost=[&](const ActorSnapshot& a,const RoomGoalValue& value,std::uint32_t watching){
        float cost=red_cost(*building,services,request,result_,c,room->id,value.node,value.position,watching,stop,&prices);
        float path=-1;if(services.path_length){poll(stop);path=services.path_length(a.node_id,value.node,c.path_nodes);poll(stop);}
        cost+=(std::isfinite(path)&&path>=0?path*c.approach_weight:c.path_unknown_cost_m);
        for(const auto& foe:foes){poll(stop);const float distance=std::sqrt(flat2(foe.point,value.position));
            if(distance<c.known_enemy_exclusion_m)cost+=c.exposure_cost_m*(1.f+c.known_enemy_exclusion_m-distance);
            if(!services.rays){cost+=c.unknown_cost_m;continue;}
            Segment segment{add(foe.point,{0,config_.knowledge.eye_m,0}),add(value.position,{0,c.look_height_m,0})};float hit=-2;
            services.rays(&segment,&hit,1);poll(stop);if(hit==-1.f)cost+=c.exposure_cost_m;else if(!std::isfinite(hit)||hit<0)cost+=c.unknown_cost_m;
        }return cost;};
    for(auto& m:men_){poll(stop);const auto* a=actor(w,m.actor);if(!a)continue;
        if(a->last_rejected.node==m.order.goal_node&&a->last_rejected.restrictions_revision==a->restrictions_revision&&
            (a->last_rejected.reason==reject_goal_invalid||a->last_rejected.reason==reject_goal_inaccessible)){
            if(std::find(m.rejected.begin(),m.rejected.end(),a->last_rejected.node)==m.rejected.end())m.rejected.push_back(a->last_rejected.node);m.goal_ready=false;}
    }
    // Optional sight-only shortcut requires the complete authored node set.
    // Sampled corner coverage remains SEARCH data, never a substitute here.
    bool dead_end=true;for(const auto& p:building->portals)if(p.traversable&&p.doorway&&(p.room_a==room->id||p.room_b==room->id)&&
        p.id!=entry_&&!searched(request,result_,other_room(p,room->id)))dead_end=false;
    if(request.sight_clear_nodes_per_tick&&dead_end&&room->nodes&&room->node_ids.size()==room->nodes&&services.node_position&&services.buildings->locate){
        if(sight_cells_.empty()){sight_geometry_complete_=true;std::uint32_t previous=invalid_node;
            for(auto node:room->node_ids){poll(stop);if(node==invalid_node||(previous!=invalid_node&&node<=previous)){sight_geometry_complete_=false;break;}
                sight_cells_.push_back({node,0,{}});previous=node;}}
        const auto work=(std::min)(request.sight_clear_nodes_per_tick,sight_cells_.size());
        for(std::size_t i=0;i<work;++i){poll(stop);auto& cell=sight_cells_[sight_cursor_++%sight_cells_.size()];Vec3 point;
            ++result_.costs.sight_tested;const auto located=services.buildings->locate(cell.node);
            if(!services.node_position(cell.node,&point)||!inside(*room,point,c.floor_tolerance_m)||located.first!=building->id||located.second!=room->id){
                sight_geometry_complete_=false;continue;}
            point=add(point,{0,c.look_height_m,0});for(auto key:side.members){const auto* a=actor(w,key);if(a&&sees(*a,point,w,config_,services,stop)){
                cell.seen_ms=a->observed_at_ms;cell.observer=a->key;cell.observer_owner=a->ownership_generation;break;}}
        }
    }
    result_.costs.sight_nodes=room->nodes;
    for(const auto& cell:sight_cells_)if(cell.seen_ms&&fresh(cell.seen_ms,w.now_ms,sight_age))++result_.costs.sight_seen;
    if(request.sight_clear_nodes_per_tick&&phase_==RoomPhase::pie&&dead_end&&foes.empty()&&sight_geometry_complete_&&room->nodes&&sight_cells_.size()==room->nodes&&result_.costs.sight_seen==room->nodes){
        if(std::find(result_.cleared_rooms.begin(),result_.cleared_rooms.end(),room->id)==result_.cleared_rooms.end())result_.cleared_rooms.push_back(room->id);
        result_.clear_proof=observed_cells(true);
        result_.events.push_back({events::room_cleared,room->id,0});transition(RoomPhase::assess,RoomReason::sight_clear);
        for(auto& m:men_){const auto* a=actor(w,m.actor);if(a){m.order.goal_position=a->position;m.order.goal_node=a->node_id;m.goal_ready=true;}}
    }
    bool lane_updated=false;
    const auto update_lane=[&](){if(lane_updated)return;lane_updated=true;
        RoomLaneRequest lane;lane.attempt=generation_;lane.building=building->id;lane.target_room=room->id;
        for(const auto& m:men_)if(!m.watcher&&std::find(lane.portals.begin(),lane.portals.end(),m.portal)==lane.portals.end())lane.portals.push_back(m.portal);
        result_.lane=lane_cost_.update(w,side,lane,*services.buildings,stop);
        if(result_.lane.cancelled)throw Cancelled{};poll(stop);
        if(result_.lane.accepted)for(const auto& receipt:result_.lane.portals)
            result_.costs.casualty_lane=(std::max)(result_.costs.casualty_lane,receipt.cost);
    };
    const auto lane_cost=[&](std::uint32_t id){if(result_.lane.accepted)for(const auto& receipt:result_.lane.portals)
        if(receipt.portal==id)return receipt.cost;return 0.f;};
    if(phase_==RoomPhase::pie){bool pie_done=true,leader_enter=false,missing_pie_goal=false;std::size_t losses=0;
        for(auto key:side.members){const auto* a=actor(w,key);if(a&&(!a->alive||a->wounded))++losses;}
        const float unseen=result_.costs.total?1.f-float(result_.costs.covered)/float(result_.costs.total):1.f;
        for(auto& m:men_){poll(stop);if(m.rank||m.watcher)continue;const auto* a=actor(w,m.actor);const auto* p=portal(*building,m.portal);if(!a||!p)continue;
            const auto f=door_frame(*p,room->id);const Contact* contact=nullptr;
            auto consider=[&](const Contact& candidate){if(!valid_key(candidate.key)||candidate.key==a->key||!candidate.hostile||!candidate.alive_when_observed||candidate.wounded||!personally_visible(candidate,*a)||
                    !fresh(candidate.observed_at_ms,w.now_ms,config_.snapshot_max_age_ms)||
                    !fresh(a->observed_at_ms,w.now_ms,config_.snapshot_max_age_ms)||!inside(*room,candidate.position,config_.knowledge.eye_m+c.floor_tolerance_m))return;
                if(!contact||candidate.key==a->enemy.key)contact=&candidate;};
            consider(a->enemy);for(std::size_t i=0;i<(std::min)(std::size_t(a->visible_enemies.count),a->visible_enemies.items.size());++i)consider(a->visible_enemies.items[i]);
            if(contact){if(m.pie_enemy!=contact->key){m.pie_enemy=contact->key;m.contact_ms=w.now_ms;m.engage_choice=m.leader_choice=0;}
                const float door_time=c.pie_call_door_ms?(std::min)(1.f,float(w.now_ms-m.contact_ms)/float(c.pie_call_door_ms)):1.f;
                const auto entrants=std::count_if(men_.begin(),men_.end(),[](const Man& man){return !man.watcher;});
                auto choice=pie_scores(*a,*contact,*p,room->id,std::size_t(entrants),losses,unseen,door_time,last_shot_&&fresh(last_shot_,w.now_ms,c.leader_no_shot_ms),c);
                choice.enter-=heat_cost(m.portal);
                if(!m.engage_choice)m.engage_choice=choice.engage>=choice.back_out?1:2;
                else if(m.engage_choice==1&&choice.back_out>choice.engage+c.pie_engage_hysteresis)m.engage_choice=2;
                else if(m.engage_choice==2&&choice.engage>choice.back_out+c.pie_engage_hysteresis)m.engage_choice=1;
                if(!m.leader_choice)m.leader_choice=choice.enter>=choice.overwatch?1:2;
                else if(m.leader_choice==1&&choice.overwatch>choice.enter+c.pie_call_hysteresis)m.leader_choice=2;
                else if(m.leader_choice==2&&choice.enter>choice.overwatch+c.pie_call_hysteresis)m.leader_choice=1;
                choice.engage_here=m.engage_choice==1;choice.enter_now=m.leader_choice==1;leader_enter|=choice.enter_now;
                result_.costs.engage+=choice.engage;result_.costs.back_out+=choice.back_out;result_.costs.overwatch+=choice.overwatch;result_.pie_choices.push_back(choice);
                m.order.kind=OrderKind::peek;m.order.role=roles::door_slice;m.order.stance=Stance::stand;m.order.lean=(m.order.side?float(m.order.side):1.f)*c.pie_lean;
                m.order.pie={add(p->centre,{0,c.look_height_m,0}),true};m.order.look={LookKind::point,m.order.pie.pivot,{}};doorway(m.order,f);
                if(choice.engage_here){m.order.goal_node=a->node_id;m.order.goal_position=a->position;m.order.pace=Pace::stand;m.goal_ready=true;}
                else {m.goal_ready=false;m.order.pace=Pace::walk;if(!project_goal(m,m.stack_goal,false))++result_.costs.unprojected;}
                pie_done=false;continue;
            }
            if(valid_key(m.pie_enemy)){m.pie_enemy={};m.contact_ms=0;m.engage_choice=m.leader_choice=0;m.goal_ready=false;}
            const auto pie_available=[&](const RoomGoalValue& goal){
                const auto local=room_frame_local(f,goal.position);
                if(!finite(goal.position)||local.z>0||inside(*room,goal.position,c.floor_tolerance_m))return false;
                if(services.buildings->locate){poll(stop);const auto location=services.buildings->locate(goal.node);poll(stop);
                    const auto source_room=other_room(*p,room->id);
                    if((location.first!=invalid_room&&location.first!=building->id)||location.second==room->id||
                        (location.second!=invalid_room&&(location.first!=building->id||location.second!=source_room)))return false;}
                if(services.path_length){poll(stop);const float route=services.path_length(a->node_id,goal.node,c.path_nodes);poll(stop);
                    if(!std::isfinite(route)||route<0)return false;}
                const float spacing=c.spacing_m*c.spacing_m;
                for(const auto& peer:w.actors)if(peer.key!=a->key&&peer.alive&&peer.faction==a->faction&&
                    (peer.node_id==goal.node||flat2(peer.position,goal.position)<spacing))return false;
                for(const auto& peer:men_)if(peer.actor!=m.actor&&peer.goal_ready&&
                    (peer.order.goal_node==goal.node||flat2(peer.order.goal_position,goal.position)<spacing))return false;
                return true;};
            // A teammate can move onto a previously valid slice between
            // captures; replan it before crediting an applied peek.
            if(m.goal_ready){RoomGoalValue held;held.node=m.order.goal_node;held.position=m.order.goal_position;
                if(!pie_available(held)){m.goal_ready=false;m.entry_progress_ms=0;}}
            const float radius=(std::max)(c.pie_radius_m,p->half_width+c.jamb_offset_m);
            const float start=std::acos((std::min)(1.f,c.wall_offset_m/radius));const auto count=static_cast<std::size_t>(std::ceil(start*radius/c.pie_advance_m))+1;
            // Advance intermediate slices before the executor's arrival stop.
            // The final post retains the ordinary physical arrival threshold.
            const float advance=m.pie_step+1<count?(std::max)(c.arrival_m,c.pie_advance_m):c.arrival_m;
            const auto applied_slice=std::any_of(captured_primary_orders.begin(),captured_primary_orders.end(),[&](const Order& o){
                return o.actor==a->key&&o.order_seq==a->last_applied.order_seq&&o.kind==OrderKind::peek&&
                    o.role==roles::door_slice&&o.issued_at_ms<=a->observed_at_ms;});
            const auto look_delta=add(m.order.look.value,add(a->position,{0,config_.knowledge.eye_m,0}),-1);
            const bool looked=finite(a->head_dir)&&length(a->head_dir)>.5f&&length(look_delta)>.001f&&
                dot(unit(a->head_dir),unit(look_delta))>=std::cos(c.check_half_deg*pi/180.f);
            if(m.goal_ready&&flat2(a->position,m.order.goal_position)<=advance*advance&&
                accepted_execution(*a,w,request)&&applied_slice&&looked&&
                fresh(a->observed_at_ms,w.now_ms,config_.snapshot_max_age_ms)){
                ++m.pie_step;m.goal_ready=false;observed_progress.emplace_back(a->key,a->observed_at_ms);}
            if(m.pie_step>=count)continue;pie_done=false;
            const float sign=m.order.side?float(m.order.side):1.f;
            m.order.kind=OrderKind::peek;m.order.role=roles::door_slice;m.order.pace=Pace::walk;m.order.stance=Stance::stand;m.order.lean=sign*c.pie_lean;
            m.order.pie={{p->centre.x,room->floor_y+c.look_height_m,p->centre.z},true};m.order.look={LookKind::point,m.order.pie.pivot,{}};doorway(m.order,f);
            // entry_progress_ms is dormant until entry_tracking starts. In PIE
            // it is the failed projection's next retry time; ENTRY overwrites
            // it on its first measured movement sample.
            if(!m.goal_ready&&w.now_ms>=m.entry_progress_ms){
                // The geometric pie arc must also clear the live heel and his
                // committed cell. The native publisher applies room spacing
                // after this plan; repeatedly proposing a closer slice leaves
                // the point held in place and can strand the whole element.
                bool placed=false;
                for(unsigned offset=0;offset<5&&!placed;++offset)for(unsigned step_ahead=0;step_ahead<6&&!placed;++step_ahead){poll(stop);
                    const float try_radius=radius+float(offset)*c.spacing_m*.5f;
                    const float try_start=std::acos((std::min)(1.f,c.wall_offset_m/try_radius));
                    const float angle=(std::max)(0.f,try_start-(float(m.pie_step)+float(step_ahead)/3.f)*c.pie_advance_m/try_radius);
                    const Vec3 q=room_frame_world(f,sign*try_radius*std::sin(angle),-try_radius*std::cos(angle),room->floor_y);
                    RoomGoalValue candidate;if(!probe_goal(m,q,false,candidate))continue;
                    // A skipped obstructed arc sample may move the next post
                    // forward, but never recycle the last sampled viewpoint.
                    if(m.pie_step&&flat2(candidate.position,m.order.goal_position)<c.arrival_m*c.arrival_m)continue;
                    if(!pie_available(candidate))continue;
                    commit_goal(m,candidate);m.entry_progress_ms=0;placed=true;
                }
                if(!placed){++result_.costs.unprojected;m.entry_progress_ms=saturated_after(w.now_ms,c.leader_idle_ms);}
            }
            if(!m.goal_ready)missing_pie_goal=true;
        }
        update_leader(pie_done);
        const bool leader_escalates=result_.leader.execution_confirmed&&result_.leader.suggestion==LeaderCall::entry;
        result_.costs.go=float(foes.size())*c.pie_call_unseen+result_.costs.door_hot+result_.costs.leader;
        const auto pie_elapsed=w.now_ms-phase_at_;
        const float cap_fraction=c.pie_cap_ms?(std::min)(1.f,float(pie_elapsed)/float(c.pie_cap_ms)):1.f;
        result_.costs.wait=cap_fraction*c.readiness_missing_cost;
        if(result_.costs.visible_foes&&last_shot_&&fresh(last_shot_,w.now_ms,c.leader_no_shot_ms))result_.costs.go+=c.pie_call_alert;
        result_.reason=RoomReason::pie_progress;
        const bool contact_cap=!result_.pie_choices.empty()&&w.now_ms-phase_at_>=c.pie_overwatch_cap_ms;
        // A point diverted to personal cover cannot become an entry point by
        // timer or leader call. Each element still needs a live staged pair;
        // normal PIE completion additionally needs applied, sampled slices.
        std::map<std::uint32_t,std::size_t> staged;
        for(const auto& m:men_)if(!m.watcher&&m.rank<2){const auto* a=actor(w,m.actor);const auto* p=portal(*building,m.portal);
            if(!a||!p||!eligible(*a)||a->reloading||a->weapon.state==WeaponState::reloading)continue;
            const auto frame=door_frame(*p,room->id);
            // Obstacle/friendly spacing can expand a validated pie arc beyond
            // pie_radius_m. Its point still belongs to this entrance when he
            // physically reaches the exact prepared slice the engine applied.
            // A proposed slice or a personal-cover diversion earns no credit.
            const bool applied_prepared_slice=(m.goal_ready||m.pie_step)&&
                m.order.kind==OrderKind::peek&&m.order.role==roles::door_slice&&
                flat2(a->position,m.order.goal_position)<=c.arrival_m*c.arrival_m&&
                accepted_execution(*a,w,request)&&
                std::any_of(captured_primary_orders.begin(),captured_primary_orders.end(),[&](const Order& o){
                    return o.actor==a->key&&o.order_seq==a->last_applied.order_seq&&
                        o.kind==OrderKind::peek&&o.role==roles::door_slice&&
                        o.assignment.portal==m.portal&&o.goal_node==m.order.goal_node&&
                        flat2(o.goal_position,m.order.goal_position)<.000001f&&
                        o.issued_at_ms<=a->observed_at_ms;
                });
            const bool point_staged=(flat2(a->position,m.stack_goal)<=c.entry_ready_m*c.entry_ready_m||
                std::sqrt(flat2(a->position,p->centre))<=c.pie_radius_m+c.entry_ready_m||applied_prepared_slice)&&
                room_frame_local(frame,a->position).z<=c.entry_zone_depth_m;
            const bool heel_staged=flat2(a->position,m.stack_goal)<=c.entry_ready_m*c.entry_ready_m;
            if((m.rank==0&&point_staged)||(m.rank==1&&heel_staged))++staged[m.portal];}
        bool paired=!staged.empty();
        std::set<std::uint32_t> active_elements,point_elements;
        for(const auto& m:men_)if(!m.watcher){active_elements.insert(m.portal);if(m.rank==0)point_elements.insert(m.portal);}
        for(auto id:active_elements){const auto count=std::count_if(men_.begin(),men_.end(),[&](const Man& peer){
                return !peer.watcher&&peer.portal==id;});
            paired=paired&&point_elements.count(id)&&staged[id]>=(count>=2?2u:1u);}
        bool saw_point_work=false;for(const auto& m:men_)if(!m.watcher&&!m.rank&&m.pie_step)
            saw_point_work=true;
        const bool cap_reached=w.now_ms-phase_at_>=c.pie_cap_ms;
        const bool ready_to_call=result_.geometry_prepared&&paired&&!missing_pie_goal&&
            (pie_done||leader_enter||contact_cap||cap_reached||(saw_point_work&&leader_escalates));
        // An interrupted point who left the opening returns to FORM after the
        // bounded PIE interval. FORM can restore the same pair or promote a
        // captured reachable rear member without granting an unseen slice.
        if((!paired||missing_pie_goal)&&cap_reached)
            transition(RoomPhase::form,RoomReason::forming);
        // Start the entrance cost only when a GO would otherwise be called.
        // FORM travel or incomplete PIE work must not spend its brief cap.
        if(ready_to_call)update_lane();
        // A clear second entrance can move while the occupied entrance pays
        // its own temporary cost at release. Keep the maximum in diagnostics.
        float entrance_cost=result_.costs.casualty_lane;
        for(const auto& m:men_)if(!m.watcher)entrance_cost=(std::min)(entrance_cost,lane_cost(m.portal));
        const bool priced_lane=result_.costs.wait>=entrance_cost||w.now_ms-phase_at_>=c.pie_cap_ms;
        result_.costs.go+=entrance_cost;
        if(ready_to_call&&!priced_lane)result_.reason=RoomReason::casualty_cost;
        if(ready_to_call&&priced_lane){
            transition(RoomPhase::entry,pie_done?RoomReason::arrived:leader_escalates?RoomReason::leader_call:(w.now_ms-phase_at_>=c.pie_cap_ms?RoomReason::pie_cap:RoomReason::go_cost));
            for(auto& m:men_){m.goal_ready=false;m.order.lean=0;m.order.pie.valid=false;m.order.flags&=~order_jamb;}}
    }
    bool frag_wait=false;
    if(phase_==RoomPhase::entry&&!throw_.finished){
        if(!throw_.attempted){throw_.attempted=true;if(request.permit_frag){const ActorSnapshot* chosen=nullptr;
                for(const auto& m:men_){const auto* a=actor(w,m.actor);const auto* entry=portal(*building,m.portal);
                    if(!m.watcher&&entry&&(entry->room_a==room->id||entry->room_b==room->id)&&a&&a->grenades.count&&a->grenades.slot_item!=invalid_id&&!a->reloading&&
                    (!valid_key(request.preferred_thrower)||request.preferred_thrower==a->key)){chosen=a;break;}}
                if(chosen){auto man=std::find_if(men_.begin(),men_.end(),[&](const Man& m){return m.actor==chosen->key;});const auto* p=portal(*building,man->portal);
                    // The executor's serial ledger belongs to the actor, across
                    // room/solo producers and side changes. Do not restart at
                    // this doctrine instance's local counter after a handoff.
                    throw_serial_=(std::max)(throw_serial_,chosen->throw_state.serial);
                    if(p&&throw_serial_!=std::numeric_limits<std::uint32_t>::max()){throw_.actor=chosen->key;throw_.owner=chosen->ownership_generation;throw_.item=chosen->grenades.slot_item;
                        throw_.serial=++throw_serial_;throw_.baseline_releases=chosen->throw_state.releases;throw_.requested_ms=w.now_ms;
                        const auto f=door_frame(*p,room->id);throw_.target=room_frame_world(f,0,c.frag_inside_m[0],room->floor_y);result_.reason=RoomReason::frag_requested;}}
            }if(!throw_.serial)throw_.finished=true;}
        if(throw_.serial){const auto* a=actor(w,throw_.actor);
            // Latch the first exact in-hand receipt. Reason changes cannot
            // renew the release cap or make the entry queue abandon an armed throw.
            if(!throw_.armed_ms&&a&&a->ownership_generation==throw_.owner&&a->throw_state.serial==throw_.serial&&
                a->throw_state.phase==2&&a->throw_state.at_ms>=throw_.requested_ms&&a->throw_state.at_ms<=w.now_ms)
                throw_.armed_ms=a->throw_state.at_ms;
            if(a&&a->ownership_generation==throw_.owner&&a->throw_state.serial==throw_.serial&&a->throw_state.phase==3&&
                a->throw_state.releases>throw_.baseline_releases&&a->throw_state.source_item==throw_.item&&a->throw_state.projectile!=invalid_id&&
                a->throw_state.at_ms>=a->throw_state.released_at_ms&&a->throw_state.at_ms<=w.now_ms&&
                a->throw_state.released_at_ms>=throw_.requested_ms&&a->throw_state.released_at_ms<=w.now_ms){
                throw_.go_at_ms=saturated_after(a->throw_state.released_at_ms,c.frag_fuse_ms);}
            if(throw_.go_at_ms){frag_wait=w.now_ms<throw_.go_at_ms;if(!frag_wait)throw_.finished=true;result_.reason=RoomReason::frag_fuse;}
            else if(!a||!eligible(*a)||a->ownership_generation!=throw_.owner||
                (a->throw_state.serial==throw_.serial&&a->throw_state.phase==4)||
                w.now_ms>=(throw_.armed_ms?saturated_after(throw_.armed_ms,c.frag_release_cap_ms):saturated_after(throw_.requested_ms,c.frag_prefire_cap_ms))){
                throw_.finished=true;result_.reason=RoomReason::frag_failed;}
            else frag_wait=true;
        }
    }
    auto assign_inside=[&](Man& m,const RoomDoorFrame& f){const auto* a=actor(w,m.actor);if(!a)return false;
        struct Candidate{RoomGoalValue goal;float cost,exposure;std::uint32_t sector;};std::vector<Candidate> choices;const float sign=m.order.side?float(m.order.side):1.f;
        std::vector<RoomGeometryPoint> choices_geometry;
        if(actual_nodes)choices_geometry=room_geometry_.state().candidates;
        else for(const auto& sector:result_.coverage)choices_geometry.push_back({invalid_node,sector.sector,sector.point,0});
        for(const auto& sector:choices_geometry){poll(stop);RoomGoalValue goal;if(!probe_goal(m,sector.position,true,goal))continue;
            const auto local=room_frame_local(f,goal.position);
            // The executor may stop anywhere inside its arrival radius. A
            // committed entry destination must put that whole arrival band
            // past the measured crossing plane; otherwise an arrived point
            // man can remain permanently outside SEARCH membership.
            if(local.z<=c.entry_zone_depth_m+c.entry_arrival_m)continue;
            float cost=0;
            if(m.rank==0)cost=local.z+((local.x*sign<0)?c.exposure_cost_m:0);
            else if(m.rank==1)cost=local.z+((local.x*sign>0)?c.exposure_cost_m:0);
            else cost=-local.z+float(m.rank)*.01f*std::abs(local.x);
            const float exposure=destination_cost(*a,goal,0);cost+=exposure;
            for(const auto& other:men_)if(other.actor!=m.actor&&other.goal_ready&&flat2(other.order.goal_position,goal.position)<c.spacing_m*c.spacing_m)cost+=c.funnel_cost_m;
            choices.push_back({goal,cost,exposure,sector.sector});}
        std::stable_sort(choices.begin(),choices.end(),[](const Candidate& a,const Candidate& b){return a.cost<b.cost||(a.cost==b.cost&&a.sector<b.sector);});
        if(choices.empty())return false;commit_goal(m,choices.front().goal);m.order.assignment.sector=choices.front().sector;
        result_.costs.destination_exposure+=choices.front().exposure;return true;};
    if(phase_==RoomPhase::entry||phase_==RoomPhase::search){std::size_t entered=0,pending=0;
        if(phase_==RoomPhase::entry)update_leader(false);
        // Re-form the point and heel after the pie. The tail keeps closing on
        // its own cells; a late second element cannot hold a ready pair forever.
        std::set<std::uint32_t> ready_pairs;std::size_t waiting_elements=0;
        // Uncommitted slot gaps are repaired in FORM through RoomStack. Keep
        // the authored ranks here: an entry rank is never list position.
        for(auto& front:men_)if(!front.watcher&&front.rank==0&&!front.released){
            ++waiting_elements;const auto* a=actor(w,front.actor);const auto* p=portal(*building,front.portal);if(!a||!p)continue;
            const auto f=door_frame(*p,room->id);Man* heel=nullptr;
            for(auto& other:men_)if(!other.watcher&&!other.released&&other.portal==front.portal&&other.rank==1)heel=&other;
            const auto heel_goal=add(front.stack_goal,f.normal,-(std::max)(c.spacing_m,c.entry_behind_m));
            const auto progress=[&](Man& m,Vec3 desired){const auto* observed=actor(w,m.actor);if(!observed)return;
                const float remaining=std::sqrt(flat2(observed->position,desired));
                if(!m.entry_tracking){m.entry_tracking=true;m.entry_best_m=remaining;m.entry_progress_ms=w.now_ms;}
                else if(remaining+c.leader_progress_m<m.entry_best_m){m.entry_best_m=remaining;m.entry_progress_ms=w.now_ms;}};
            progress(front,front.stack_goal);if(heel)progress(*heel,heel_goal);
            if(heel&&w.now_ms-heel->entry_progress_ms>=c.swap_ms){const auto* current=actor(w,heel->actor);
                const bool ready=current&&!current->reloading&&current->weapon.state!=WeaponState::reloading&&heel->goal_ready&&
                    flat2(current->position,heel->order.goal_position)<=c.entry_ready_m*c.entry_ready_m&&flat2(a->position,current->position)<=c.entry_heel_m*c.entry_heel_m;
                if(!ready){Man* replacement=nullptr;float nearest=c.entry_third_m*c.entry_third_m;
                    for(auto& other:men_)if(!other.watcher&&!other.released&&other.portal==front.portal&&other.rank>1){
                        const auto* candidate=actor(w,other.actor);if(!candidate||candidate->reloading||candidate->weapon.state==WeaponState::reloading)continue;
                        const float distance=flat2(a->position,candidate->position);if(distance<=nearest){replacement=&other;nearest=distance;}}
                    if(replacement){const auto rank=replacement->rank;replacement->rank=heel->rank;heel->rank=rank;
                        heel->order.assignment.slot=static_cast<std::uint8_t>(rank);heel->order.goal_node=heel->stack_node;heel->order.goal_position=heel->stack_goal;
                        heel->entry_line_prepared=false;heel->entry_tracking=false;heel->goal_ready=false;
                        replacement->entry_line_prepared=false;replacement->entry_tracking=false;replacement->goal_ready=false;
                        replacement->order.assignment.slot=1;heel=replacement;progress(*heel,heel_goal);}}
            }
            const auto stage=[&](Man& m,Vec3 desired){if(!m.entry_line_prepared||!m.goal_ready){
                    RoomGoalValue value;
                    if(!probe_goal(m,desired,false,value)||room_frame_local(f,value.position).z>0){++result_.costs.unprojected;return;}
                    if(m.rank==1&&front.entry_line_prepared){
                        const float distance=flat2(value.position,front.order.goal_position);
                        if(room_frame_local(f,value.position).z>room_frame_local(f,front.order.goal_position).z)return;
                        if(distance<c.spacing_m*c.spacing_m||distance>=c.entry_heel_m*c.entry_heel_m)return;}
                    commit_goal(m,value);m.entry_line_prepared=true;}
                m.order.kind=OrderKind::stack;m.order.role=m.rank?roles::entry_wait:roles::door_stack_point;
                m.order.flags&=~order_committed_entry;m.order.pace=Pace::walk;m.order.speed_scale=1;
                m.order.arrival_radius_m=c.arrival_m;m.order.corner.valid=false;doorway(m.order,f);
                m.order.look={LookKind::point,add(p->centre,{0,c.look_height_m,0}),{}};
                const auto* observed=actor(w,m.actor);const bool edge=m.rank==0&&observed&&flat2(observed->position,m.order.goal_position)<=c.jamb_final_m*c.jamb_final_m;
                m.order.lean=edge?float(m.order.side)*c.edge_lean:0;m.order.pie={m.order.look.value,edge};};
            stage(front,front.stack_goal);
            if(heel){stage(*heel,heel_goal);
                // Projection can collapse an authored .8 m gap to adjacent
                // .7 m cells. That pair cannot pass the dispatcher's legitimate
                // reservation gate. Prefer a nearby diagonal so the physical
                // heel envelope leaves useful arrival tolerance, then one
                // farther pitch. Every candidate uses the ordinary projection,
                // route and post-projection spacing checks.
                const auto staged_heel=add(front.order.goal_position,f.normal,-(std::max)(c.spacing_m,c.entry_behind_m));
                if((!heel->entry_line_prepared||!heel->goal_ready)&&c.pitch_m>0)
                    stage(*heel,add(staged_heel,f.tangent,c.pitch_m));
                if((!heel->entry_line_prepared||!heel->goal_ready)&&c.pitch_m>0)
                    stage(*heel,add(staged_heel,f.tangent,-c.pitch_m));
                if((!heel->entry_line_prepared||!heel->goal_ready)&&c.pitch_m>0)
                    stage(*heel,add(staged_heel,f.normal,-c.pitch_m));
                // A narrow graph may have no separate cell behind the point.
                // Reuse the validated original neighbour, never invent a node.
                if(!heel->entry_line_prepared||!heel->goal_ready)stage(*heel,heel->stack_goal);}
            if(heel&&front.entry_line_prepared&&heel->entry_line_prepared){
                const float distance=std::sqrt(flat2(front.order.goal_position,heel->order.goal_position));
                const float slack=(c.entry_heel_m-distance)*.5f;
                if(slack>0){const float arrival=(std::min)(c.arrival_m,slack);
                    front.order.arrival_radius_m=heel->order.arrival_radius_m=arrival;}}
            const auto* b=heel?actor(w,heel->actor):nullptr;
            const bool front_set=front.entry_line_prepared&&flat2(a->position,front.order.goal_position)<=c.entry_ready_m*c.entry_ready_m&&
                !a->reloading&&a->weapon.state!=WeaponState::reloading;
            const bool heel_set=!heel||(b&&heel->entry_line_prepared&&!b->reloading&&b->weapon.state!=WeaponState::reloading&&
                flat2(b->position,heel->order.goal_position)<=c.entry_ready_m*c.entry_ready_m&&
                flat2(a->position,b->position)<=c.entry_heel_m*c.entry_heel_m);
            // Ordinary GO requires the captured pair. Failed projection,
            // immobility or perpetual reload prices out at the existing flow
            // cap, retaining the old bounded entry path and native route checks.
            // This fallback does not fabricate a physical readiness receipt.
            const bool paired_at_opening=heel&&a&&b&&front.entry_line_prepared&&heel->entry_line_prepared&&
                (flat2(a->position,front.stack_goal)<=c.entry_ready_m*c.entry_ready_m||
                    std::sqrt(flat2(a->position,p->centre))<=c.pie_radius_m+c.entry_ready_m)&&
                (flat2(b->position,heel->stack_goal)<=c.entry_ready_m*c.entry_ready_m||
                    std::sqrt(flat2(b->position,p->centre))<=c.pie_radius_m+c.entry_heel_m);
            if((front_set&&heel_set)||(paired_at_opening&&w.now_ms-phase_at_>=c.entry_flow_cap_ms))ready_pairs.insert(front.portal);
        }
        if(!ready_pairs.empty()&&!pair_ready_at_)pair_ready_at_=w.now_ms;
        const bool shared_go=ready_pairs.size()==waiting_elements||go_at_||
            (pair_ready_at_&&w.now_ms-pair_ready_at_>=c.leader_up_wait_ms);
        if(!frag_wait&&shared_go&&!ready_pairs.empty()&&!go_at_)go_at_=w.now_ms;
        if(!frag_wait&&std::any_of(men_.begin(),men_.end(),[](const Man& m){return !m.watcher&&!m.released;}))update_lane();
        for(auto& m:men_){poll(stop);if(m.watcher)continue;const auto* a=actor(w,m.actor);if(!a)continue;
            const auto* p=portal(*building,m.portal);
            // A late interior join can have no authored entry portal. SEARCH
            // still measures the current body against its committed goal.
            if(!p){
                if(phase_==RoomPhase::search&&m.released){
                    bool within_room=inside(*room,a->position,c.floor_tolerance_m);
                    m.room_membership=Evidence::unknown;
                    if(services.buildings->locate){poll(stop);const auto location=services.buildings->locate(a->node_id);poll(stop);
                        const bool authored_member=!actual_nodes||std::binary_search(room->node_ids.begin(),room->node_ids.end(),a->node_id);
                        within_room=within_room&&authored_member&&location.first==building->id&&location.second==room->id;
                        m.room_membership=within_room?Evidence::yes:Evidence::no;
                    }else if(actual_nodes)within_room=false;
                    m.crossed=within_room;
                    m.arrived=m.goal_ready&&flat2(a->position,m.order.goal_position)<=c.entry_arrival_m*c.entry_arrival_m;
                    if(!m.arrived)++pending;
                }
                continue;
            }
            const auto f=door_frame(*p,room->id);
            bool within_room=inside(*room,a->position,c.floor_tolerance_m);
            m.room_membership=Evidence::unknown;
            if(services.buildings->locate){poll(stop);const auto location=services.buildings->locate(a->node_id);poll(stop);
                const bool authored_member=!actual_nodes||std::binary_search(room->node_ids.begin(),room->node_ids.end(),a->node_id);
                within_room=within_room&&authored_member&&location.first==building->id&&location.second==room->id;
                m.room_membership=within_room?Evidence::yes:Evidence::no;
            }else if(actual_nodes)within_room=false;
            // Unmapped legacy geometry retains only its coarse fallback. The
            // receipt remains unknown; it is not upgraded into native proof.
            // Entry depth proves the initial crossing. Once inside this room,
            // moving back toward an interior doorway viewpoint is still SEARCH.
            // Leaving the actual room (or losing membership proof) clears it.
            m.crossed=(m.crossed||room_frame_local(f,a->position).z>c.entry_zone_depth_m)&&within_room;
            // Keep physical membership as evidence, but do not credit an
            // emergency displacement as completion of a released entry.
            if(m.crossed&&m.released)++entered;
            if(!m.released&&!frag_wait){const Man* previous=nullptr;
                for(const auto& candidate:men_)if(!candidate.watcher&&candidate.portal==m.portal&&candidate.rank<m.rank&&(!previous||candidate.rank>previous->rank))previous=&candidate;
                const bool predecessor_crossed=!previous||previous->crossed,predecessor_released=!previous||previous->released;
                const TimeMs prior_release=previous?previous->released_ms:go_at_;
                const bool pair_wait=shared_go&&m.rank<2&&ready_pairs.count(m.portal)!=0;
                const bool pair_go=shared_go&&m.rank<2&&(m.rank==0?pair_wait:(previous?previous->released:released_portals_.count(m.portal)!=0));
                const bool follow=m.rank>=2&&(predecessor_crossed||(predecessor_released&&
                    (w.now_ms-prior_release>=c.release_gap_ms||(go_at_&&w.now_ms-go_at_>=c.entry_flow_cap_ms))));
                if(pair_wait||pair_go||follow){
                    const auto hot=std::find_if(heat_.begin(),heat_.end(),[&](const DoorHeat& h){return h.portal==m.portal;});
                    const bool firing=hot!=heat_.end()&&fresh(hot->at_ms,w.now_ms,c.crossing_gap_ms)&&w.now_ms-hot->at_ms<c.crossing_gap_ms;
                    // Only an entrant eligible for a new release starts a wait.
                    // Later hits update the measured gap, never this fixed cap.
                    if(firing&&!m.crossing_wait_started){m.crossing_wait_started=true;m.crossing_wait_ms=w.now_ms;}
                    const auto waited=m.crossing_wait_started?w.now_ms-m.crossing_wait_ms:0;
                    const bool capped=m.crossing_wait_started&&waited>=c.crossing_block_cap_ms;
                    const bool waiting=firing&&!capped;
                    if(m.crossing_wait_started){RoomCrossingWait receipt;receipt.actor=m.actor;receipt.portal=m.portal;
                        receipt.last_hit_ms=hot!=heat_.end()?hot->at_ms:0;receipt.started_ms=m.crossing_wait_ms;receipt.waited_ms=waited;
                        receipt.gap_remaining_ms=firing?c.crossing_gap_ms-(w.now_ms-hot->at_ms):0;
                        receipt.cap_remaining_ms=capped?0:c.crossing_block_cap_ms-waited;
                        receipt.waiting=waiting;receipt.cap_released=firing&&capped;result_.crossings.push_back(receipt);
                        result_.costs.crossing_wait_ms=(std::max)(result_.costs.crossing_wait_ms,waited);
                        result_.costs.crossing_waiters+=waiting?1u:0u;result_.costs.crossing_cap_releases+=receipt.cap_released?1u:0u;}
                    const float waiting_cost=go_at_?float(w.now_ms-go_at_)/1000.f*c.pie_call_men:0;
                    const bool lane_priced=waiting_cost>=lane_cost(m.portal);
                    if(!lane_priced)result_.reason=RoomReason::casualty_cost;
                    if(!waiting&&lane_priced&&(pair_go||follow)){m.released=true;m.released_ms=w.now_ms;m.goal_ready=false;result_.events.push_back({events::entry_released,room->id,m.actor.id});}
                }}
            if(m.released){m.order.kind=OrderKind::entry;m.order.flags|=order_committed_entry;m.order.pace=c.entry_pace?Pace::run:Pace::walk;
                m.order.speed_scale=c.entry_pace?c.entry_run_scale:c.entry_walk_scale;m.order.lean=0;m.order.pie.valid=false;m.order.arrival_radius_m=c.entry_arrival_m;
                m.order.role=m.rank==0?roles::entry_point_clear_side:m.rank==1?roles::entry_hook_hard_corner:roles::entry_deep;
                doorway(m.order,f);if(!m.goal_ready&&!assign_inside(m,f))++result_.costs.unprojected;
                m.order.corner={add(m.order.goal_position,{0,c.look_height_m,0}),true};m.order.look={LookKind::point,m.order.corner.point,{}};
                bool tail=phase_==RoomPhase::entry&&m.crossed&&m.rank>=c.rear_from_slot&&m.rank>1;
                for(const auto& other:men_)if(!other.watcher&&other.portal==m.portal&&other.rank>m.rank)tail=false;
                if(tail){
                    // Only the last rear member of this element covers the
                    // opening after crossing it. His forward destination and
                    // committed pace remain intact; point/hook keep their sweep.
                    m.order.role=roles::entry_tail;
                    m.order.corner={room_frame_world(f,0,0,room->floor_y+c.look_height_m),true};
                    m.order.look={LookKind::point,m.order.corner.point,{}};
                }
                // Point/hook first attend the sourced room threat. Corner is
                // the consumed post-crossing channel; changing look alone
                // would be masked by the native doorway priority.
                if(phase_==RoomPhase::entry&&m.rank<2&&!foes.empty()){const Foe* selected=nullptr;float nearest=0;
                    for(const auto& threat:foes){poll(stop);if(!aware(*a,threat))continue;const float d=flat2(a->position,threat.point);
                        if(!selected||threat.at>selected->at||(threat.at==selected->at&&d<nearest)){selected=&threat;nearest=d;}}
                    if(selected){m.order.corner.point=selected->point;m.order.look={LookKind::point,selected->point,{}};}}
                m.arrived=m.goal_ready&&flat2(a->position,m.order.goal_position)<=c.entry_arrival_m*c.entry_arrival_m;
                if(!m.arrived)++pending;
                // Entry owns the full brisk crossing, including the doorway band.
                m.order.flags&=~order_jamb;
            }else {++pending;m.order.kind=OrderKind::stack;m.order.role=roles::entry_wait;m.order.pace=Pace::walk;
                if(m.rank>=2){m.order.lean=0;m.order.pie.valid=false;}}
        }
        if(phase_==RoomPhase::entry&&!frag_wait&&entered&&(!pending||w.now_ms-go_at_>=c.entry_flow_cap_ms))transition(RoomPhase::search,RoomReason::entry_flow);
    }
    if(phase_==RoomPhase::search){
        auto free_observer=[&](const Man& m){const auto* a=actor(w,m.actor);if(!services.rays||!a||!m.crossed||m.watcher||!eligible(*a)||a->reloading||
                a->weapon.state==WeaponState::reloading||a->aid_state.phase==1||a->aid_state.phase==2||a->throw_state.phase==1||a->throw_state.phase==2)return false;
            return first_response(*a,config_,w.now_ms,w.level_epoch,w.geometry_revision,0).look.kind!=LookKind::enemy;};
        // Retain one free observer per source. Re-reading the same old fact
        // cannot turn an already inspected point into a permanent gaze lock.
        for(const auto& f:foes){poll(stop);auto task=std::find_if(inspections_.begin(),inspections_.end(),[&](const Inspection& t){return same_source(t.receipt,f);});
            if(task==inspections_.end()){Inspection added;added.receipt.foe=f.id;added.receipt.point=f.point;added.receipt.source_at_ms=f.at;
                inspections_.push_back(added);task=inspections_.end()-1;}
            auto assigned=std::find_if(men_.begin(),men_.end(),[&](const Man& m){return m.actor==task->receipt.observer&&m.owner==task->owner;});
            if(valid_key(task->receipt.observer)&&(assigned==men_.end()||(!task->receipt.inspected_at_ms&&
                    (!free_observer(*assigned)||!aware(*actor(w,assigned->actor),f))))){task->receipt.observer={};task->owner=0;
                task->receipt.assigned_at_ms=task->receipt.inspected_at_ms=0;}
            if(task->receipt.inspected_at_ms)continue;
            if(!valid_key(task->receipt.observer)){Man* chosen=nullptr;float nearest=0;
                for(auto& m:men_){poll(stop);if(!free_observer(m)||std::any_of(inspections_.begin(),inspections_.end(),[&](const Inspection& t){
                            return !t.receipt.inspected_at_ms&&t.receipt.observer==m.actor;}))continue;
                    const auto* a=actor(w,m.actor);if(!aware(*a,f))continue;const float d=flat2(a->position,f.point);if(!chosen||d<nearest){chosen=&m;nearest=d;}}
                if(chosen){task->receipt.observer=chosen->actor;task->owner=chosen->owner;task->receipt.assigned_at_ms=w.now_ms;}}
            const auto* observer=actor(w,task->receipt.observer);if(!observer||observer->observed_at_ms<=task->receipt.assigned_at_ms)continue;
            // Require the actual accepted point-look assignment, then measured
            // head direction + static visibility. Proposed intent earns none.
            bool consumed=false;for(const auto& o:request.applied_orders){poll(stop);if(o.actor==observer->key&&o.order_seq==observer->last_applied.order_seq&&
                    o.assignment.generation==generation_&&o.look.kind==LookKind::point&&!o.pie.valid&&!o.corner.valid&&
                    o.look.value.x==f.point.x&&o.look.value.y==f.point.y&&o.look.value.z==f.point.z&&accepted_execution(*observer,w,request)){consumed=true;break;}}
            if(consumed&&sees(*observer,f.point,w,config_,services,stop))task->receipt.inspected_at_ms=observer->observed_at_ms;
        }
        const auto unseen=[&](const RoomCoverage& s){return !s.seen_at_ms||!fresh(s.seen_at_ms,w.now_ms,sight_age);};
        std::vector<unsigned> search_claims;
        std::vector<std::pair<const Man*,unsigned>> retained_search_claims;
        // Preserve a valid unfinished assignment before assigning newly free
        // observers. Iteration order changes must not swap people's sectors.
        for(const auto& m:men_){poll(stop);
            if(!m.crossed||(m.released&&!m.goal_ready))continue;
            const auto* a=actor(w,m.actor);if(!a||first_response(*a,config_,w.now_ms,w.level_epoch,w.geometry_revision,0).kind==OrderKind::yield)continue;
            if(std::any_of(inspections_.begin(),inspections_.end(),[&](const Inspection& t){return t.receipt.observer==m.actor&&
                    t.owner==m.owner&&!t.receipt.inspected_at_ms;}))continue;
            const unsigned sector=m.order.assignment.sector;
            if(std::find(search_claims.begin(),search_claims.end(),sector)!=search_claims.end())continue;
            if(std::any_of(result_.coverage.begin(),result_.coverage.end(),[&](const RoomCoverage& s){return s.sector==sector&&unseen(s);})){
                retained_search_claims.push_back({&m,sector});search_claims.push_back(sector);}
        }
        for(auto& m:men_){poll(stop);if(!m.crossed||(m.released&&!m.goal_ready))continue;const auto* a=actor(w,m.actor);
            if(!a||first_response(*a,config_,w.now_ms,w.level_epoch,w.geometry_revision,0).kind==OrderKind::yield)continue;
            m.order.kind=OrderKind::search;m.order.role=roles::search_sector;m.order.flags&=~order_committed_entry;m.order.corner.valid=false;
            m.order.pace=m.arrived?Pace::stand:Pace::walk;const RoomCoverage* next=nullptr;float nearest=0;
            auto inspection=std::find_if(inspections_.begin(),inspections_.end(),[&](const Inspection& t){return t.receipt.observer==m.actor&&
                t.owner==m.owner&&!t.receipt.inspected_at_ms;});
            const auto retained=std::find_if(retained_search_claims.begin(),retained_search_claims.end(),[&](const auto& claim){return claim.first==&m;});
            if(retained!=retained_search_claims.end())for(const auto& s:result_.coverage)if(s.sector==retained->second){next=&s;break;}
            if(!next)for(const auto& s:result_.coverage)if(unseen(s)&&
                    std::find(search_claims.begin(),search_claims.end(),s.sector)==search_claims.end()){
                const float distance=flat2(a->position,s.point);
                if(!next||distance<nearest){next=&s;nearest=distance;}}
            if(!next)for(const auto& s:result_.coverage)if(unseen(s)){
                const float distance=flat2(a->position,s.point);
                if(!next||distance<nearest){next=&s;nearest=distance;}}
            const bool new_search_claim=next&&inspection==inspections_.end()&&retained==retained_search_claims.end();
            if(new_search_claim)search_claims.push_back(next->sector);
            if(next){m.order.look={LookKind::point,add(next->point,{0,c.look_height_m,0}),{}};m.order.assignment.sector=next->sector;
                // A blocked sector is not made seen by repeatedly ordering a
                // look. Once the body actually turns there, change viewpoint.
                const Vec3 eye=add(a->position,{0,config_.knowledge.eye_m,0}),delta=add(m.order.look.value,eye,-1);
                const bool facing=finite(a->head_dir)&&length(a->head_dir)>.5f&&dot(unit(a->head_dir),unit(delta))>=std::cos(c.check_half_deg*pi/180.f);
                const bool distant=length(delta)>c.see_range_m;
                bool alternate_view=false;
                // Only a delivered K1 man-down location known to THIS member
                // can make a search post costly. It says a
                // teammate fell there, not where the hostile is. Reuse the
                // ordinary room projection, graph route, static sightline and
                // destination exposure pricing for alternate viewpoints.
                if(knowledge&&knowledge->accepted&&knowledge->level_epoch==w.level_epoch&&
                    knowledge->sequence<=w.sequence&&services.rays){
                    std::vector<std::pair<Vec3,TimeMs>> losses;
                    for(const auto& known_member:knowledge->members)if(known_member.actor==a->key&&
                        known_member.side==side.side_id&&known_member.ownership_generation==a->ownership_generation){
                        for(const auto& fact:known_member.board)if(fact.kind==ReportKind::man_down&&
                            fact.member!=a->key&&std::find(side.members.begin(),side.members.end(),fact.member)!=side.members.end()&&
                            fact.at_ms<=a->observed_at_ms&&fresh(fact.at_ms,w.now_ms,c.seen_ms)&&
                            inside(*room,fact.position,c.floor_tolerance_m))losses.push_back({fact.position,fact.at_ms});
                        break;}
                    const float radius=c.known_enemy_search_m;
                    const bool near_loss=radius>0&&c.seen_ms&&std::any_of(losses.begin(),losses.end(),[&](const auto& loss){
                        return flat2(loss.first,next->point)<radius*radius;});
                    const bool current_risk=std::any_of(losses.begin(),losses.end(),[&](const auto& loss){
                        return flat2(loss.first,m.order.goal_position)<radius*radius;});
                    if(near_loss&&current_risk&&(!m.search_hazard_retry_ms||
                        w.now_ms>=m.search_hazard_retry_ms+c.swap_ms)&&
                        (distant||!clear_ray(services,eye,m.order.look.value,stop))){
                        m.search_hazard_retry_ms=w.now_ms;
                        result_.events.push_back({events::search_loss_priced,room->id,a->key.id});
                        RoomGoalValue best;float best_cost=std::numeric_limits<float>::infinity();bool found=false;
                        const auto target=m.order.look.value;
                        auto consider=[&](Vec3 desired){poll(stop);RoomGoalValue value;if(!probe_goal(m,desired,true,value))return;
                            for(const auto& peer:men_)if(peer.actor!=m.actor){const auto* other=actor(w,peer.actor);
                                if(other&&other->alive&&(flat2(value.position,other->position)<c.spacing_m*c.spacing_m||
                                    (peer.goal_ready&&flat2(value.position,peer.order.goal_position)<c.spacing_m*c.spacing_m)))return;}
                            const auto view=add(value.position,{0,config_.knowledge.eye_m,0});
                            if(length(add(target,view,-1))>c.see_range_m||!clear_ray(services,view,target,stop))return;
                            float price=destination_cost(*a,value,0);
                            for(const auto& loss:losses){const float separation=std::sqrt(flat2(value.position,loss.first));
                                if(separation<radius)price+=c.exposure_cost_m*(1.f-separation/radius)*
                                    (1.f-float(w.now_ms-loss.second)/float(c.seen_ms));}
                            if(price<best_cost){best=value;best_cost=price;found=true;}};
                        consider(next->point);
                        const auto& alternatives=room_geometry_.state().candidates;
                        const auto limit=(std::min)(alternatives.size(),std::size_t(config_.cover_queries_per_slice));
                        for(std::size_t i=0;i<limit;++i)consider(alternatives[(m.search_hazard_cursor+i)%alternatives.size()].position);
                        if(!alternatives.empty())m.search_hazard_cursor=(m.search_hazard_cursor+limit)%alternatives.size();
                        if(found){if(best.node!=m.order.goal_node)result_.events.push_back({events::search_viewpoint,room->id,a->key.id});
                            commit_goal(m,best);m.arrived=flat2(a->position,best.position)<=c.arrival_m*c.arrival_m;
                            m.order.pace=m.arrived?Pace::stand:Pace::walk;alternate_view=true;}
                    }
                }
                // A remembered contact keeps the room uncleared, not every
                // soldier stationary. Reposition toward a blocked search view;
                // current own-target aim/fire still wins in fresh_order below.
                if(!alternate_view&&inspection==inspections_.end()&&m.arrived&&(distant||(facing&&services.rays&&!clear_ray(services,eye,m.order.look.value,stop)))){
                    if(project_goal(m,next->point,true)){m.order.pace=Pace::walk;m.arrived=false;}
                    else ++result_.costs.unprojected;
                }
            }
            if(!next&&inspection==inspections_.end()&&std::any_of(inspections_.begin(),inspections_.end(),[&](const Inspection& t){
                    return t.receipt.observer==m.actor&&t.receipt.inspected_at_ms;})){
                for(const auto& sector:result_.coverage)if(sector.sector==m.order.assignment.sector){
                    m.order.look={LookKind::point,add(sector.point,{0,c.look_height_m,0}),{}};break;}}
            if(inspection!=inspections_.end()){
                auto& task=inspection->receipt;task.repositioning=false;m.order.look={LookKind::point,task.point,{}};
                m.order.pie.valid=false;m.order.corner.valid=false;m.order.flags&=~order_face_movement;
                const auto eye=add(a->position,{0,config_.knowledge.eye_m,0});const auto delta=add(task.point,eye,-1);
                const bool far=length(delta)>c.see_range_m;const bool blocked=services.rays&&!clear_ray(services,eye,task.point,stop);
                if(m.arrived&&(far||blocked)){
                    RoomGoalValue best;float best_cost=0;bool found=false;
                    const auto candidates=actual_nodes?room_geometry_.state().candidates:std::vector<RoomGeometryPoint>{};
                    auto candidate=[&](Vec3 point){poll(stop);RoomGoalValue value;if(!probe_goal(m,point,true,value))return;
                        const Vec3 view=add(value.position,{0,config_.knowledge.eye_m,0});if(length(add(task.point,view,-1))>c.see_range_m||
                            !clear_ray(services,view,task.point,stop))return;
                        const float cost=destination_cost(*a,value,0);if(!found||cost<best_cost){found=true;best=value;best_cost=cost;}};
                    if(actual_nodes)for(const auto& point:candidates)candidate(point.position);
                    else for(const auto& sector:result_.coverage)candidate(sector.point);
                    // An unavailable threat sightline is not a veto on the
                    // ordinary sweep. Try another unobserved authored sector;
                    // this movement still earns no inspection/clearance fact.
                    if(!found)for(const auto& sector:result_.coverage){poll(stop);
                        if(sector.seen_at_ms&&fresh(sector.seen_at_ms,w.now_ms,sight_age))continue;
                        RoomGoalValue value;if(!probe_goal(m,sector.point,true,value)||
                            flat2(a->position,value.position)<=c.arrival_m*c.arrival_m)continue;
                        const float cost=destination_cost(*a,value,0);if(!found||cost<best_cost){found=true;best=value;best_cost=cost;}}
                    if(found){commit_goal(m,best);m.order.pace=Pace::walk;m.arrived=false;task.repositioning=true;}
                }else task.repositioning=!m.arrived;
            }
        }
        result_.reason=foes.empty()?RoomReason::coverage:RoomReason::known_foe;
        if(all_seen()&&foes.empty()&&!result_.costs.known_wounded){if(std::find(result_.cleared_rooms.begin(),result_.cleared_rooms.end(),room->id)==result_.cleared_rooms.end())result_.cleared_rooms.push_back(room->id);
            result_.clear_proof=observed_cells(false);
            result_.events.push_back({events::room_cleared,room->id,0});transition(RoomPhase::assess,RoomReason::coverage);}
    }
    if((phase_==RoomPhase::assess||phase_==RoomPhase::complete)&&(!foes.empty()||result_.costs.known_wounded)){
        result_.cleared_rooms.erase(std::remove(result_.cleared_rooms.begin(),result_.cleared_rooms.end(),room->id),result_.cleared_rooms.end());
        // The old sweep predates this renewed threat. Every coverage sector
        // and full-nav shortcut node must be physically observed again.
        for(auto& sector:result_.coverage){sector.seen_at_ms=0;sector.observer={};sector.observer_owner=0;}
        for(auto& cell:sight_cells_){cell.seen_ms=0;cell.observer={};cell.observer_owner=0;}
        result_.clear_proof.clear();
        transition(RoomPhase::search,RoomReason::fresh_contact);
    }
    if(phase_==RoomPhase::assess){
        auto assess_goal=[&](const Man& m,Vec3 desired,RoomGoalValue& value){
            if(!probe_goal(m,desired,true,value))return false;
            // Arrival tolerance can leave two men closer than their assigned
            // destinations. A hold at the captured feet still has to satisfy
            // the same room reservation spacing as the published order.
            for(const auto& other:men_)if(other.actor!=m.actor&&other.goal_ready&&
                flat2(other.order.goal_position,value.position)<c.spacing_m*c.spacing_m)return false;
            return true;};
        std::vector<const Portal*> doors;for(const auto& p:building->portals){poll(stop);
            if(p.id!=entry_&&p.traversable&&p.doorway&&(p.room_a==room->id||p.room_b==room->id)&&
                std::abs(p.centre.y-room->floor_y)<=c.floor_tolerance_m+config_.knowledge.eye_m&&!searched(request,result_,other_room(p,room->id)))doors.push_back(&p);}
        std::stable_sort(doors.begin(),doors.end(),[](const Portal* a,const Portal* b){return a->id<b->id;});
        auto assess_candidate=[&](const Man& m,const Portal& p,std::size_t step,RoomGoalValue& best,float& best_cost){
            const auto* a=actor(w,m.actor);if(!a||!inside(*room,a->position,c.floor_tolerance_m)||step>=c.assess_angles_deg.size())return false;
            const auto f=door_frame(p,other_room(p,room->id));const auto local=room_frame_local(f,a->position);const float sign=local.x>=0?1.f:-1.f;
            const float angle=c.assess_angles_deg[step]*pi/180.f;bool found=false;
            std::vector<float> radii{c.assess_radius_m};for(float radius:c.assess_radii_m)if(std::find(radii.begin(),radii.end(),radius)==radii.end())radii.push_back(radius);
            for(float radius:radii){poll(stop);RoomGoalValue candidate;const auto desired=room_frame_world(f,sign*radius*std::cos(angle),-(p.wall_depth+radius*std::sin(angle)),room->floor_y);
                if(!assess_goal(m,desired,candidate))continue;const float score=destination_cost(*a,candidate,p.id);
                best=candidate;best_cost=score;found=true;break;
                // The authored radius is preserved when it projects. The
                // configured alternates repair missing/rejected geometry;
                // door/member assignment still compares measured risk costs.
            }return found;};
        for(const auto* p:doors){poll(stop);bool held=false;for(const auto& m:men_)held|=m.assess_portal==p->id;if(held)continue;
            Man* chosen=nullptr;RoomGoalValue best;float cost=0;
            for(auto& m:men_){if(m.watcher||m.assess_portal)continue;RoomGoalValue candidate;float score=0;
                if(assess_candidate(m,*p,0,candidate,score)&&(!chosen||score<cost||(score==cost&&m.actor.id<chosen->actor.id))){chosen=&m;best=candidate;cost=score;}}
            if(chosen){chosen->assess_portal=p->id;chosen->assess_step=0;chosen->assess_arrived_ms=0;commit_goal(*chosen,best);result_.costs.destination_exposure+=cost;}
            else ++result_.costs.unprojected;
        }
        bool busy=false,had_pier=false,new_casualty=false;
        for(auto key:side.members){const auto* a=actor(w,key);if(a&&(!a->alive||a->wounded)&&std::find(assess_down_.begin(),assess_down_.end(),key)==assess_down_.end())new_casualty=true;}
        for(auto& m:men_){poll(stop);const auto* a=actor(w,m.actor);if(!a)continue;
            m.order.kind=OrderKind::hold_door;m.order.role=roles::assess_hold;m.order.flags&=~(order_committed_entry|order_jamb);m.order.corner.valid=false;
            m.order.pace=Pace::stand;m.order.speed_scale=1;m.order.lean=0;m.order.pie.valid=false;
            const auto* assigned=portal(*building,m.assess_portal);
            if(assigned&&m.assess_step<c.assess_angles_deg.size()){
                had_pier=busy=true;const auto f=door_frame(*assigned,other_room(*assigned,room->id));
                const auto look=room_frame_world(f,0,assigned->wall_depth+c.entry_follow_m,room->floor_y+c.look_height_m);
                const bool arrived=m.goal_ready&&flat2(a->position,m.order.goal_position)<=c.arrival_m*c.arrival_m;
                if(arrived&&sees(*a,look,w,config_,services,stop)){if(!m.assess_arrived_ms)m.assess_arrived_ms=w.now_ms;
                    if(w.now_ms-m.assess_arrived_ms>=c.assess_slice_ms){++m.assess_step;m.assess_arrived_ms=0;m.goal_ready=false;}}
                else m.assess_arrived_ms=0;
                if(m.assess_step<c.assess_angles_deg.size()){
                    if(!m.goal_ready){RoomGoalValue candidate;float cost=0;if(assess_candidate(m,*assigned,m.assess_step,candidate,cost)){commit_goal(m,candidate);result_.costs.destination_exposure+=cost;}
                        else ++result_.costs.unprojected;}
                    m.order.role=roles::assess_pie;m.order.look={LookKind::point,look,{}};m.order.pie={look,true};doorway(m.order,f);
                    m.order.lean=(room_frame_local(f,a->position).x>=0?1.f:-1.f)*c.pie_lean;
                    m.order.pace=m.goal_ready&&flat2(a->position,m.order.goal_position)>c.arrival_m*c.arrival_m?Pace::walk:Pace::stand;
                    if(!m.goal_ready){m.order.goal_node=a->node_id;m.order.goal_position=a->position;}
                    continue;
                }
            }
            if(m.assess_portal)had_pier=true;
            const Portal* nearest=nullptr;float distance=0;for(const auto* p:doors){poll(stop);const auto f=door_frame(*p,other_room(*p,room->id));
                const auto beyond=room_frame_world(f,0,p->wall_depth+c.entry_follow_m,room->floor_y+c.look_height_m);
                const float d=flat2(a->position,p->centre);if(sees(*a,beyond,w,config_,services,stop)&&(!nearest||d<distance)){nearest=p;distance=d;}}
            if(nearest)m.order.look={LookKind::point,{nearest->centre.x,room->floor_y+c.look_height_m,nearest->centre.z},{}};
            RoomGoalValue hold;bool have=assess_goal(m,a->position,hold);float best=have?destination_cost(*a,hold,nearest?nearest->id:0):std::numeric_limits<float>::max();
            // Standing body inside an opening is a cost, not a permanent veto:
            // use the bounded, already-validated room candidates for repairs,
            // not just the few coverage samples which may all be occupied.
            if(!have||red_cost(*building,services,request,result_,c,room->id,a->node_id,a->position,nearest?nearest->id:0,stop)>0){
                auto consider=[&](Vec3 point){RoomGoalValue candidate;if(!assess_goal(m,point,candidate))return;
                    const float cost=destination_cost(*a,candidate,nearest?nearest->id:0);
                    if(!have||cost<best){hold=candidate;best=cost;have=true;}};
                if(actual_nodes){for(const auto& candidate:room_geometry_.state().candidates)consider(candidate.position);}
                else for(const auto& sector:result_.coverage)consider(sector.point);
            }
            if(have){commit_goal(m,hold);m.order.pace=flat2(a->position,hold.position)>c.arrival_m*c.arrival_m?Pace::walk:Pace::stand;}
            else {m.order.goal_node=a->node_id;m.order.goal_position=a->position;m.goal_ready=false;++result_.costs.unprojected;}
        }
        // Neither time nor the assessment look certifies the adjacent room.
        // Completion is observed slice work, a fresh cost, or the bounded cap.
        if(doors.empty()||fresh_hit||new_casualty||result_.costs.visible_foes||w.now_ms-phase_at_>=c.assess_cap_ms||(had_pier&&!busy))
            transition(RoomPhase::complete,fresh_hit||new_casualty||result_.costs.visible_foes?RoomReason::fresh_contact:
                doors.empty()||(had_pier&&!busy)?RoomReason::assess_observed:RoomReason::assess_cap);
        // Once outgoing work is actually accepted, the next stack can overlap
        // it. This is permission to plan ahead, never proof of a seen slice.
        bool care_busy=false,outgoing_applied=false;
        for(auto key:side.members){const auto* a=actor(w,key);if(!a)continue;
            care_busy|=a->alive&&(a->wounded||a->aid_state.phase==1||a->aid_state.phase==2||
                a->throw_state.phase==1||a->throw_state.phase==2||room_care_owned(w,*a,request.applied_orders));}
        for(const auto& m:men_)if(m.assess_portal&&m.goal_ready){const auto* a=actor(w,m.actor);if(!a||!accepted_execution(*a,w,request))continue;
            for(const auto& applied:request.applied_orders)if(applied.actor==m.actor&&applied.order_seq==a->last_applied.order_seq&&
                applied.assignment.generation==generation_&&applied.role==roles::assess_pie&&applied.pie.valid&&
                applied.goal_node==m.order.goal_node&&distance_squared(applied.goal_position,m.order.goal_position)<.0001f)outgoing_applied=true;}
        if(phase_==RoomPhase::assess&&outgoing_applied&&!care_busy&&!fresh_hit&&foes.empty()&&!result_.costs.known_wounded&&
            std::find(result_.cleared_rooms.begin(),result_.cleared_rooms.end(),room->id)!=result_.cleared_rooms.end())result_.ready_for_next=true;
    }
    if(phase_==RoomPhase::complete)result_.ready_for_next=true;
    for(const auto& task:inspections_){poll(stop);result_.threat_attention.push_back(task.receipt);}
    for(auto key:side.members){poll(stop);const auto* a=actor(w,key);if(!a)continue;auto found=std::find_if(men_.begin(),men_.end(),[&](const Man& m){return m.actor==key;});
        if(found==men_.end()){auto order=first_response(*a,config_,w.now_ms,w.level_epoch,w.geometry_revision,0);result_.slots.push_back({key,order});continue;}
        auto order=found->order;const auto authored_look=order.look;bool authored_look_valid=(authored_look.kind==LookKind::point||authored_look.kind==LookKind::direction)&&finite(authored_look.value);fresh_order(order,w,*a,config_);
        if(found->released&&!found->goal_ready){order=first_response(*a,config_,w.now_ms,w.level_epoch,w.geometry_revision,0);authored_look_valid=false;}
        if(frag_wait&&key==throw_.actor&&!throw_.go_at_ms){order.kind=OrderKind::hold;order.goal_node=a->node_id;order.goal_position=a->position;
            order.lean=0;order.pie.valid=false;order.pace=Pace::stand;order.fire={};
            order.throw_order.item=throw_.item;order.throw_order.target=throw_.target;order.throw_order.serial=throw_.serial;order.throw_order.portal=found->portal;order.throw_order.valid=true;}
        const auto immediate=first_response(*a,config_,w.now_ms,w.level_epoch,w.geometry_revision,0);
        if(immediate.kind==OrderKind::yield&&!(own_throw(*a)&&continuing_throw(order,*a,w.now_ms))){order=immediate;authored_look_valid=false;}
        // Price a small local correction from the CURRENT body on a measured
        // hot-door shot line. Never replace a released entrant's forward goal,
        // move a stack man away from his bounded slot, or claim native access.
        if(order.kind!=OrderKind::entry&&!(order.flags&order_committed_entry)&&order.kind!=OrderKind::yield&&order.kind!=OrderKind::withdraw&&
            !order.throw_order.valid&&!valid_key(order.aid)&&!order.pie.valid&&services.project&&services.path_length&&
            std::isfinite(c.denial_step_m)&&c.denial_step_m>0&&std::isfinite(c.exposed_correction_max_m)&&c.exposed_correction_max_m>0){
            for(const auto& hot:heat_){poll(stop);if(!fresh(hot.at_ms,w.now_ms,c.door_hot_ms))continue;
                const auto* opening=portal(*building,hot.portal);if(!opening)continue;
                const auto f=door_frame(*opening,room->id);const Vec3 d{hot.point.x-hot.origin.x,0,hot.point.z-hot.origin.z};
                const float len=std::hypot(d.x,d.z);if(len<=c.denial_step_m||std::abs(a->position.y-hot.origin.y)>c.floor_tolerance_m+config_.knowledge.eye_m)continue;
                const auto exposure=[&](Vec3 p){const float along=((p.x-hot.origin.x)*d.x+(p.z-hot.origin.z)*d.z)/len;
                    if(along< -c.entry_lane_half_m||along>len+c.entry_lane_half_m)return 0.f;
                    const float cross=std::abs((p.x-hot.origin.x)*d.z-(p.z-hot.origin.z)*d.x)/len;
                    return (std::max)(0.f,1.f-cross/c.entry_lane_half_m);};
                const float start=exposure(a->position);if(start<=0)continue;
                float best=start*heat_cost(hot.portal);Vec3 point;std::uint32_t node=invalid_node;
                const auto steps=(std::min)(std::size_t(std::ceil(c.exposed_correction_max_m/c.denial_step_m)),std::size_t(config_.cover_queries_per_slice));
                for(std::size_t step=1;step<=steps;++step)for(float sign:{-1.f,1.f}){poll(stop);
                    const float offset=(std::min)(c.exposed_correction_max_m,float(step)*c.denial_step_m);
                    Vec3 wanted{a->position.x-sign*d.z/len*offset,a->position.y,a->position.z+sign*d.x/len*offset},snapped;
                    const auto id=services.project(wanted,c.projection_m,c.floor_tolerance_m,&snapped);poll(stop);
                    if(id==invalid_node||!finite(snapped)||flat2(snapped,a->position)>c.exposed_correction_max_m*c.exposed_correction_max_m||
                       std::abs(snapped.y-a->position.y)>c.floor_tolerance_m||room_frame_local(f,snapped).z*room_frame_local(f,a->position).z<0)continue;
                    if(order.kind==OrderKind::stack&&flat2(snapped,found->stack_goal)>c.exposed_correction_max_m*c.exposed_correction_max_m)continue;
                    if(services.buildings->locate&&services.buildings->locate(id)!=services.buildings->locate(a->node_id))continue;
                    bool occupied=false;for(const auto& man:men_)if(man.actor!=a->key){const auto* other=actor(w,man.actor);
                        occupied|=other&&other->alive&&(flat2(snapped,other->position)<c.spacing_m*c.spacing_m||flat2(snapped,man.order.goal_position)<c.spacing_m*c.spacing_m);}
                    if(occupied)continue;const float path=services.path_length(a->node_id,id,c.path_nodes);poll(stop);
                    if(!std::isfinite(path)||path<0)continue;
                    const float price=exposure(snapped)*heat_cost(hot.portal)+path*c.approach_weight;
                    if(price<best){best=price;point=snapped;node=id;}
                }
                if(node!=invalid_node){order.goal_node=node;order.goal_position=point;order.pace=Pace::walk;order.flags|=order_urgent;
                    order.arrival_radius_m=(std::min)(order.arrival_radius_m,c.denial_step_m);break;}
            }
        }
        RoomDoctrineSlot output{key,order,found->interior_bootstrap?false:found->released,
            found->interior_bootstrap?false:found->crossed,found->arrived,found->interior_bootstrap?0:found->released_ms};
        output.authored_look=authored_look;output.authored_look_valid=authored_look_valid;
        output.room_membership=found->room_membership;
        if(request.exposure||request.stack.exposure){
            const auto located=services.buildings->locate?services.buildings->locate(order.goal_node):std::make_pair(invalid_room,invalid_room);
            if(located.first==building->id)result_.costs.held_exposure+=red_cost(*building,services,request,result_,c,
                located.second,order.goal_node,order.goal_position,found->assess_portal,stop,&prices,&output.exposure,true);
        }
        for(const auto& applied:request.applied_orders){poll(stop);if(applied.actor==key&&applied.order_seq&&applied.order_seq==a->last_applied.order_seq&&
            applied.ownership_generation==a->ownership_generation&&applied.restrictions_revision==a->restrictions_revision&&applied.level_epoch==w.level_epoch&&
            applied.geometry_revision==w.geometry_revision&&applied.issued_at_ms<=w.now_ms&&a->last_applied.at_ms>=applied.issued_at_ms&&a->last_applied.at_ms<=a->observed_at_ms&&
            applied.goal_node==order.goal_node&&distance_squared(applied.goal_position,order.goal_position)<.0001f&&
            applied.assignment.generation==order.assignment.generation&&applied.assignment.portal==order.assignment.portal&&
            applied.assignment.sector==order.assignment.sector&&applied.assignment.slot==order.assignment.slot){
                output.execution_confirmed=true;output.applied_order_seq=applied.order_seq;break;}}
        result_.slots.push_back(std::move(output));}
    if(result_.leader.accepted&&std::any_of(men_.begin(),men_.end(),[](const Man& m){return m.released&&!m.interior_bootstrap;})){
        result_.leader.released=true;result_.leader.suggestion=LeaderCall::none;}
    result_.room_cells=observed_cells(false);
    auto sight_cells=observed_cells(true);result_.room_cells.insert(result_.room_cells.end(),sight_cells.begin(),sight_cells.end());
    result_.exposure=prices.stats();result_.phase=phase_;result_.phase_started_ms=phase_at_;return result_;
}
}}}
