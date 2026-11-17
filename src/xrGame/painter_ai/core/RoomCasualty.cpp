#include "RoomCasualty.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace painter { namespace combat { namespace core {
namespace {
struct Cancelled{};
void poll(const std::function<bool()>& stop){if(stop&&stop())throw Cancelled{};}
const ActorSnapshot* find(const WorldSnapshot& w,ActorKey k){for(const auto& a:w.actors)if(a.key==k)return &a;return nullptr;}
const RoomDoctrineSlot* slot(const RoomDoctrineResult& base,ActorKey k){for(const auto& s:base.slots)if(s.actor==k)return &s;return nullptr;}
bool member(const SideSnapshot& side,ActorKey k){return std::find(side.members.begin(),side.members.end(),k)!=side.members.end();}
bool source_valid(const ActorSnapshot& a,const WorldSnapshot& w,const Config& c){return valid_key(a.key)&&a.level_epoch==w.level_epoch
    &&a.snapshot_seq==w.sequence&&fresh(a.observed_at_ms,w.now_ms,c.snapshot_max_age_ms)&&finite(a.position);}
bool may_help(const ActorSnapshot& a,const RoomDoctrineSlot* base,const WorldSnapshot& w,const Config& c,bool selecting=true){
    return base&&source_valid(a,w,c)&&a.alive&&!a.wounded&&!a.yield_reasons&&(a.flags&flag_admitted)
        &&a.ownership_generation&&a.current_position_accessible&&a.node_id!=invalid_node
        &&base->order.actor==a.key&&base->order.ownership_generation==a.ownership_generation
        &&base->order.restrictions_revision==a.restrictions_revision&&!base->order.throw_order.valid
        &&base->order.kind!=OrderKind::yield&&base->order.level_epoch==w.level_epoch
        &&base->order.geometry_revision==w.geometry_revision&&base->order.snapshot_seq==w.sequence
        &&(!selecting||!(base->order.flags&order_committed_entry));
}
// Patients have a small observation view, not fabricated self-observations.
struct TargetView {ActorKey key;Vec3 position;bool alive=false,wounded=false,valid=false;
    decltype(ActorSnapshot{}.aid_eligibility) aid_eligibility;};
TargetView target_view(const WorldSnapshot& w,const SideSnapshot& side,ActorKey key,const Config& cfg,
    const std::vector<FriendlyCasualtyFact>& passive){
    if(const auto* a=find(w,key)){if(!source_valid(*a,w,cfg))return {};
        return {a->key,a->position,a->alive,a->wounded,true,a->aid_eligibility};}
    const FriendlyCasualtyFact* selected=nullptr;
    for(const auto& fact:passive)if(fact.target==key&&friendly_casualty_member(fact,side)&&fact.visible_now
        &&fact.friendly==Evidence::yes&&fresh(fact.observed_at_ms,w.now_ms,cfg.snapshot_max_age_ms)&&finite(fact.position)
        &&(!selected||fact.observed_at_ms>selected->observed_at_ms
           ||(fact.observed_at_ms==selected->observed_at_ms&&fact.observer.id<selected->observer.id)))selected=&fact;
    if(!selected)return {};
    const auto* observer=find(w,selected->observer);
    if(!observer||!source_valid(*observer,w,cfg)||observer->ownership_generation!=selected->observer_owner
        ||observer->restrictions_revision!=selected->observer_restrictions||observer->observed_at_ms!=selected->observed_at_ms
        ||!observer->alive||observer->wounded||!(observer->flags&flag_admitted))return {};
    return {key,selected->position,selected->alive,selected->wounded,true,selected->aid_eligibility};
}
bool allowed(const TargetView& casualty,ActorKey helper,TimeMs now){auto const& e=casualty.aid_eligibility;
    return e.allowed==Evidence::yes&&e.observed_at_ms<=now&&(!e.claimed||e.claimant==helper);}
bool visible_able(const ActorSnapshot& a,const Config& c,TimeMs now){
    auto threat=[&](const Contact& t){return valid_key(t.key)&&t.hostile&&t.alive_when_observed&&!t.wounded&&
        finite(t.position)&&personally_visible(t,a)&&fresh(a.observed_at_ms,now,c.snapshot_max_age_ms);};
    if(threat(a.enemy))return true;
    if(a.visible_enemies.count>a.visible_enemies.items.size())return true; // invalid perception cannot authorize treatment
    for(std::size_t i=0;i<a.visible_enemies.count;++i)if(threat(a.visible_enemies.items[i]))return true;
    return false;
}
struct Exposure {float known=0,unknown=0;std::vector<const Portal*> portals;};
Exposure exposure(const WorldSnapshot& world,const EngineServices& services,const SideSnapshot& side,const RoomDoctrineResult& base,
    const std::vector<SearchedRoom>& communicated,std::uint32_t node,const RoomConfig& c,const std::function<bool()>& stop){
    Exposure out;const auto* cache=services.buildings;
    if(!cache||cache->level_epoch!=world.level_epoch||cache->map_hash!=world.map_hash){out.unknown=c.unknown_cost_m;return out;}
    const Building* building=nullptr;for(const auto& b:cache->buildings){poll(stop);if(b.id==side.mission.building){building=&b;break;}}
    if(!building){out.unknown=c.unknown_cost_m;return out;}
    std::uint32_t room=invalid_room;
    for(const auto& r:building->rooms){poll(stop);if(std::binary_search(r.node_ids.begin(),r.node_ids.end(),node)){room=r.id;break;}}
    if(room==invalid_room){out.unknown=c.unknown_cost_m;return out;}
    for(const auto& p:building->portals){poll(stop);const std::vector<Lit>* table=nullptr;std::uint32_t other=invalid_room;
        if(room==p.room_a){table=&p.lit_a;other=p.room_b;}else if(room==p.room_b){table=&p.lit_b;other=p.room_a;}
        const bool shared=std::any_of(communicated.begin(),communicated.end(),[&](const SearchedRoom& fact){return
            fact.side==side.side_id&&fact.building==side.mission.building&&fact.room==other&&fact.at_ms<=world.now_ms;});
        if(!table||shared)continue;
        const auto at=std::lower_bound(table->begin(),table->end(),node,[](const Lit& l,std::uint32_t n){return l.node<n;});
        if(at!=table->end()&&at->node==node)out.known+=c.exposure_cost_m*float(at->standing)/255.f;
        else if(!cache->lit_complete)out.unknown+=c.unknown_cost_m;
        out.portals.push_back(&p);
    }
    return out;
}
float static_ray(const EngineServices& services,Vec3 a,Vec3 b,RoomCasualtyCosts& cost,const std::function<bool()>& stop){
    if(!services.rays){++cost.unknown_rays;return -2.f;}poll(stop);Segment ray{a,b};float hit=-2.f;
    services.rays(&ray,&hit,1);++cost.ray_queries;poll(stop);if(!std::isfinite(hit)||hit< -1.f)++cost.unknown_rays;return hit;
}
bool goal_rejected(const ActorSnapshot& a,std::uint32_t node){auto const& r=a.last_rejected;
    return r.node==node&&r.restrictions_revision==a.restrictions_revision
        &&(r.reason==reject_goal_invalid||r.reason==reject_goal_inaccessible);}
const FactionStyle& style(const Config& c,const SideSnapshot& side){
    for(std::size_t i=0;i<c.factions.count&&i<c.factions.items.size();++i)
        if(c.factions.items[i].faction==side.faction)return c.factions.items[i];
    return c.faction_default;
}
bool fit_for_care(const ActorSnapshot& a,const RoomDoctrineResult& base,const WorldSnapshot& w,const Config& c){
    return may_help(a,slot(base,a.key),w,c,false)&&a.weapon.id!=invalid_id&&
        (a.weapon.ammo_in_mag||a.weapon.ammo_reserve)&&
        a.throw_state.phase!=1&&a.throw_state.phase!=2;
}
void security_order(RoomDoctrineSlot& selected,const ActorSnapshot& actor,const WorldSnapshot& w,const Config& c,
    const Vec3* hazard){
    auto& order=selected.order;const auto immediate=first_response(actor,c,w.now_ms,w.level_epoch,w.geometry_revision,0);
    order.kind=OrderKind::hold;order.role=roles::care_security;order.goal_node=actor.node_id;
    order.goal_position=actor.position;order.arrival_radius_m=c.room.aid_arrival_m;order.pace=Pace::stand;
    order.assignment={};order.aid={};order.finish={};order.throw_order={};order.portal={};
    order.pie={};order.corner={};order.expose={};order.lean=0;order.side=0;
    order.flags&=~(order_committed_entry|order_jamb|order_sprint);
    if(immediate.look.kind==LookKind::enemy){order.look=immediate.look;order.fire=immediate.fire;}
    else {order.fire={};if(hazard)order.look={LookKind::point,*hazard,{}};}
    selected.released=selected.crossed=selected.arrived=false;selected.execution_confirmed=false;
    selected.applied_order_seq=0;selected.released_at_ms=0;
}
}
RoomCasualty::RoomCasualty(Config c):config_(c){}
void RoomCasualty::reset(){attempt_={};stay_target_={};security_={};stay_room_=invalid_room;stay_reason_=CasualtyReason::none;finished_.clear();epoch_=map_=geometry_=sequence_=0;side_=mission_=0;now_=0;}
RoomCasualtyResult RoomCasualty::update(const WorldSnapshot& w,const SideSnapshot& side,const RoomDoctrineResult& base,
    const EngineServices& services,const std::function<bool()>& stop,const std::vector<FriendlyCasualtyFact>& passive,
    const std::vector<SearchedRoom>& communicated){
    RoomCasualtyResult failed;failed.reason=CasualtyReason::invalid_input;
    try{poll(stop);auto staged=*this;poll(stop);auto out=staged.update_impl(w,side,base,services,stop,passive,communicated);poll(stop);
        if(out.accepted)*this=std::move(staged);return out;
    }catch(const Cancelled&){failed.cancelled=true;failed.reason=CasualtyReason::cancelled;return failed;}
    catch(...){failed.reason=CasualtyReason::no_projection;return failed;}
}
RoomCasualtyResult RoomCasualty::update_impl(const WorldSnapshot& w,const SideSnapshot& side,const RoomDoctrineResult& base,
    const EngineServices& services,const std::function<bool()>& stop,const std::vector<FriendlyCasualtyFact>& passive,
    const std::vector<SearchedRoom>& communicated){
    RoomCasualtyResult out;out.reason=CasualtyReason::invalid_input;const auto& c=config_.room;
    if(!w.level_epoch||!w.sequence||!base.accepted||base.level_epoch!=w.level_epoch||base.snapshot_seq!=w.sequence
        ||!c.path_nodes||!c.aid_cap_ms||!std::isfinite(c.aid_reach_m)||c.aid_reach_m<=0
        ||!std::isfinite(c.aid_arrival_m)||c.aid_arrival_m<=0||!std::isfinite(c.aid_min_body_m)||c.aid_min_body_m<0
        ||!std::isfinite(c.projection_m)||c.projection_m<0||!std::isfinite(c.floor_tolerance_m)||c.floor_tolerance_m<0
        ||!std::isfinite(c.spacing_m)||c.spacing_m<=0
        ||!std::isfinite(c.exposure_cost_m)||c.exposure_cost_m<0||!std::isfinite(c.unknown_cost_m)||c.unknown_cost_m<0
        ||!std::isfinite(c.approach_weight)||c.approach_weight<0||!std::isfinite(c.look_height_m)||c.look_height_m<0)return out;
    for(float r:c.aid_placement_radii_m)if(!std::isfinite(r)||r<c.aid_min_body_m||r>c.aid_reach_m)return out;
    for(float a:c.aid_placement_angles_rad)if(!std::isfinite(a))return out;
    if(epoch_&&w.level_epoch<epoch_)return out;
    if(epoch_==w.level_epoch&&(w.sequence<=sequence_||w.now_ms<now_))return out;
    std::set<std::uint32_t> ids;for(auto k:side.members){poll(stop);if(!valid_key(k)||!ids.insert(k.id).second)return out;}
    ids.clear();for(const auto& s:base.slots){poll(stop);if(!valid_key(s.actor)||!ids.insert(s.actor.id).second||s.order.actor!=s.actor)return out;}
    if(epoch_!=w.level_epoch||map_!=w.map_hash||geometry_!=w.geometry_revision||side_!=side.side_id||mission_!=side.mission.revision){
        reset();epoch_=w.level_epoch;map_=w.map_hash;geometry_=w.geometry_revision;side_=side.side_id;mission_=side.mission.revision;}
    sequence_=w.sequence;now_=w.now_ms;out.accepted=true;out.reason=CasualtyReason::none;out.slots=base.slots;
    auto hazard=[&](const ActorSnapshot& guard){std::pair<bool,Vec3> chosen{};float best=0;
        const auto* cache=services.buildings;if(!cache||cache->level_epoch!=w.level_epoch||cache->map_hash!=w.map_hash||
            base.room==invalid_room||side.mission.kind!=MissionKind::clear_building)return chosen;
        for(const auto& building:cache->buildings)if(building.id==side.mission.building)
            for(const auto& portal:building.portals){poll(stop);
                const auto other=portal.room_a==base.room?portal.room_b:portal.room_b==base.room?portal.room_a:invalid_room;
                if(other==invalid_room&&!((portal.room_a==base.room)||(portal.room_b==base.room)))continue;
                if(!portal.doorway||!finite(portal.centre))continue;
                const bool shared=std::any_of(communicated.begin(),communicated.end(),[&](const SearchedRoom& r){return
                    r.side==side.side_id&&r.building==building.id&&r.room==other&&r.at_ms<=w.now_ms;});
                if(shared)continue;const float d=distance_squared(guard.position,portal.centre);
                if(!std::isfinite(d))continue;if(!chosen.first||d<best){chosen={true,portal.centre};best=d;}}
        return chosen;};
    auto fit=[&](){std::vector<const ActorSnapshot*> men;for(auto key:side.members){poll(stop);
        const auto* a=find(w,key);if(a&&fit_for_care(*a,base,w,config_))men.push_back(a);}return men;};
    auto patient_here=[&](ActorKey key){const auto* a=find(w,key);const auto* cache=services.buildings;
        if(!a||!source_valid(*a,w,config_)||!a->current_position_accessible||!cache||!cache->locate||cache->level_epoch!=w.level_epoch||
            cache->map_hash!=w.map_hash||base.room==invalid_room||side.mission.kind!=MissionKind::clear_building)return false;
        if(cache->locate(a->node_id)!=std::make_pair(side.mission.building,base.room))return false;
        for(const auto& building:cache->buildings)if(building.id==side.mission.building)
            for(const auto& room:building.rooms)if(room.id==base.room)return
                a->position.x>=room.min.x&&a->position.x<=room.max.x&&
                a->position.z>=room.min.z&&a->position.z<=room.max.z&&
                std::abs(a->position.y-room.floor_y)<=config_.room.floor_tolerance_m&&
                (room.node_ids.empty()||std::binary_search(room.node_ids.begin(),room.node_ids.end(),a->node_id));
        return false;};
    auto stay=[&](ActorKey key){if(style(config_,side).leave_wounded||!patient_here(key))return false;
        const auto target=target_view(w,side,key,config_,passive);if(!target.valid||!target.alive||!target.wounded)return false;
        const auto men=fit();if(men.size()!=1||!patient_here(men.front()->key))return false;
        auto selected=std::find_if(out.slots.begin(),out.slots.end(),[&](const RoomDoctrineSlot& s){return s.actor==men.front()->key;});
        if(selected==out.slots.end())return false;
        out.stay_with_casualty=true;out.casualty=key;
        // Keep the mission in this room, but never replace an immediate
        // safety/committed order just to display a care-security role.
        if(!(selected->order.flags&(order_committed_entry|order_urgent))&&
            selected->order.kind!=OrderKind::entry&&selected->order.kind!=OrderKind::withdraw&&
            first_response(*men.front(),config_,w.now_ms,w.level_epoch,w.geometry_revision,0).kind!=OrderKind::yield){
            const auto look=hazard(*men.front());security_order(*selected,*men.front(),w,config_,look.first?&look.second:nullptr);
            out.security=men.front()->key;
        }
        return true;};
    if(valid_key(stay_target_)){
        if(stay_room_==base.room&&stay(stay_target_)){out.reason=stay_reason_;return out;}
        stay_target_={};stay_room_=invalid_room;stay_reason_=CasualtyReason::none;
    }
    finished_.erase(std::remove_if(finished_.begin(),finished_.end(),[&](ActorKey key){
        const auto a=target_view(w,side,key,config_,passive);return !member(side,key)||(a.valid&&(!a.alive||!a.wounded));}),finished_.end());
    auto finish=[&](CasualtyPhase phase,CasualtyReason reason,bool exhausted){
        out.phase=phase;out.reason=reason;out.helper=attempt_.helper;out.casualty=attempt_.target;
        out.generation=attempt_.generation;out.started_ms=attempt_.started;
        if(exhausted&&valid_key(attempt_.target))finished_.push_back(attempt_.target);
        if(valid_key(attempt_.target))out.events.push_back({events::casualty,base.room,static_cast<std::uint32_t>(reason)});
        if((phase==CasualtyPhase::failed||phase==CasualtyPhase::done)&&stay(attempt_.target)){
            stay_target_=attempt_.target;stay_room_=base.room;stay_reason_=reason;
        }
        security_={};
        attempt_={};
    };
    if(valid_key(attempt_.target)){
        const auto target_value=target_view(w,side,attempt_.target,config_,passive);const auto* target=&target_value;const auto* helper=find(w,attempt_.helper);
        if(!target->valid||!member(side,attempt_.target)){
            finish(CasualtyPhase::failed,CasualtyReason::target_lost,false);return out;}
        if(!helper||!member(side,attempt_.helper)||!may_help(*helper,slot(base,attempt_.helper),w,config_,false)
            ||helper->ownership_generation!=attempt_.owner||helper->restrictions_revision!=attempt_.restrictions){
            finish(CasualtyPhase::failed,CasualtyReason::helper_lost,false);return out;}
        const auto& receipt=helper->aid_state;
        const bool new_receipt=attempt_.aid_requested&&receipt.target==attempt_.target&&receipt.at_ms>=attempt_.requested
            &&receipt.at_ms<=w.now_ms&&(receipt.at_ms!=attempt_.baseline_at||receipt.phase!=attempt_.baseline_phase);
        if(new_receipt&&receipt.phase==3){finish(CasualtyPhase::done,CasualtyReason::native_done,true);return out;}
        if(new_receipt&&receipt.phase==4){finish(CasualtyPhase::failed,CasualtyReason::native_failed,true);return out;}
        if(!target->alive||!target->wounded){finish(CasualtyPhase::failed,CasualtyReason::target_lost,false);return out;}
        if(!allowed(*target,helper->key,w.now_ms)){finish(CasualtyPhase::failed,CasualtyReason::eligibility,false);return out;}
        if(w.now_ms-attempt_.started>=c.aid_cap_ms){finish(CasualtyPhase::failed,CasualtyReason::cap,true);return out;}
        if(distance_squared(target->position,attempt_.target_position)>c.aid_arrival_m*c.aid_arrival_m||goal_rejected(*helper,attempt_.node)){
            finish(CasualtyPhase::failed,CasualtyReason::no_projection,false);return out;}
    }
    if(!valid_key(attempt_.target)){
        struct Pair {const ActorSnapshot* helper;TargetView target;float distance;};std::vector<Pair> choices;
        for(auto target_key:side.members){poll(stop);const auto target_value=target_view(w,side,target_key,config_,passive);const auto* target=&target_value;
            if(!target->valid||!target->alive||!target->wounded
                ||std::find(finished_.begin(),finished_.end(),target_key)!=finished_.end())continue;
            for(auto helper_key:side.members){poll(stop);const auto* helper=find(w,helper_key);
                if(helper_key==target_key||!helper||!may_help(*helper,slot(base,helper_key),w,config_)||!allowed(*target,helper_key,w.now_ms))continue;
                choices.push_back({helper,*target,distance_squared(helper->position,target->position)});}}
        if(choices.empty()){out.reason=CasualtyReason::no_helper;return out;}
        if(!services.project||!services.path_length){out.reason=CasualtyReason::no_projection;return out;}
        std::stable_sort(choices.begin(),choices.end(),[](const Pair& a,const Pair& b){return a.distance<b.distance;});
        RoomCasualtyCosts costs;
        for(const auto& pair:choices){poll(stop);const auto& h=*pair.helper;const auto& t=pair.target;
            struct Candidate {std::uint32_t node;Vec3 point;float path;Exposure light;float cost;};
            std::vector<Candidate> candidates;
            const float heading=std::atan2(h.position.z-t.position.z,h.position.x-t.position.x);std::set<std::uint32_t> tried;
            for(float radius:c.aid_placement_radii_m){for(float angle:c.aid_placement_angles_rad){poll(stop);
                Vec3 desired{t.position.x+radius*std::cos(heading+angle),t.position.y,t.position.z+radius*std::sin(heading+angle)},q;
                const auto node=services.project(desired,c.projection_m,c.floor_tolerance_m,&q);++costs.projected_candidates;poll(stop);
                if(node==invalid_node||!tried.insert(node).second||!finite(q)||std::abs(q.y-t.position.y)>c.floor_tolerance_m
                    ||(q.x-desired.x)*(q.x-desired.x)+(q.z-desired.z)*(q.z-desired.z)>c.projection_m*c.projection_m
                    ||distance_squared(q,t.position)<c.aid_min_body_m*c.aid_min_body_m||distance_squared(q,t.position)>c.aid_reach_m*c.aid_reach_m
                    ||goal_rejected(h,node))continue;
                bool occupied=false;for(const auto& other:base.slots){poll(stop);if(other.actor!=h.key&&other.actor!=t.key
                    &&distance_squared(q,other.order.goal_position)<c.spacing_m*c.spacing_m){occupied=true;break;}}
                if(occupied)continue;
                const float path=services.path_length(h.node_id,node,c.path_nodes);++costs.path_queries;poll(stop);
                if(!std::isfinite(path)||path<0)continue;
                auto light=exposure(w,services,side,base,communicated,node,c,stop);
                const float cost=path*c.approach_weight+light.known+light.unknown;
                candidates.push_back({node,q,path,std::move(light),cost});
            }}
            // Static proximity must not authorize aid through a wall. Validate
            // candidates in cost order. The authored radii/angle array is finite;
            // each native path also has its own canonical expansion cap.
            std::stable_sort(candidates.begin(),candidates.end(),[](const Candidate& a,const Candidate& b){return a.cost<b.cost;});
            Candidate* selected=nullptr;float best=(std::numeric_limits<float>::max)();
            for(auto& candidate:candidates){poll(stop);Vec3 eye=candidate.point,patient=t.position;
                eye.y+=c.look_height_m;patient.y+=c.look_height_m;
                const float reach=static_ray(services,eye,patient,costs,stop);
                if(std::isfinite(reach)&&reach>=0)continue;
                const bool proven=reach==-1.f;
                // A selected-point confirmation may expose a stale/incomplete
                // lit entry; it never turns unknown data into a safe claim.
                float confirmed=0;
                for(const auto* portal:candidate.light.portals){poll(stop);
                    Vec3 origin=portal->centre;origin.y+=c.look_height_m;
                    if(static_ray(services,origin,eye,costs,stop)==-1.f)confirmed+=c.exposure_cost_m;
                }
                const float known=(std::max)(candidate.light.known,confirmed);
                const float unknown=candidate.light.unknown+(proven?0:c.unknown_cost_m);
                const float score=candidate.path*c.approach_weight+known+unknown;
                if(score<best){best=score;selected=&candidate;costs.path_m=candidate.path;
                    costs.exposure_m=known;costs.unknown_m=unknown;costs.total_m=score;costs.reach_line=proven?Evidence::yes:Evidence::unknown;}
            }
            if(!selected)continue;
            if(generation_==(std::numeric_limits<std::uint64_t>::max)()||intent_revision_==(std::numeric_limits<std::uint64_t>::max)()){
                out.accepted=false;out.reason=CasualtyReason::invalid_input;return out;}
            ++generation_;attempt_.intent=++intent_revision_;attempt_.helper=h.key;attempt_.target=t.key;attempt_.owner=h.ownership_generation;
            attempt_.restrictions=h.restrictions_revision;attempt_.generation=generation_;attempt_.goal=selected->point;attempt_.node=selected->node;
            attempt_.target_position=t.position;attempt_.started=w.now_ms;attempt_.phase=CasualtyPhase::reaching;attempt_.placement=costs;
            break;
        }
        if(!valid_key(attempt_.target)){out.placement=costs;out.reason=CasualtyReason::no_projection;return out;}
    }
    const auto& helper=*find(w,attempt_.helper);
    if(attempt_.placement.reach_line!=Evidence::yes){
        Vec3 eye=attempt_.goal,patient=attempt_.target_position;eye.y+=c.look_height_m;patient.y+=c.look_height_m;
        const float ray=static_ray(services,eye,patient,attempt_.placement,stop);
        if(ray==-1.f)attempt_.placement.reach_line=Evidence::yes;
        else if(std::isfinite(ray)&&ray>=0){finish(CasualtyPhase::failed,CasualtyReason::no_projection,false);return out;}
    }
    // Only captured friendly roster facts; no world-wide injury/death scan.
    for(auto key:side.members){poll(stop);const auto* man=find(w,key);if(!man||!source_valid(*man,w,config_))continue;
        if(man->hits.count>man->hits.items.size()){out.accepted=false;out.reason=CasualtyReason::invalid_input;return out;}
        for(std::size_t i=0;i<man->hits.count;++i)if(man->hits.items[i].at_ms<=man->observed_at_ms)
            attempt_.last_hit=(std::max)(attempt_.last_hit,man->hits.items[i].at_ms);}
    const bool threat=visible_able(helper,config_,w.now_ms);
    const bool hit_quiet=attempt_.last_hit&&w.now_ms-attempt_.last_hit<c.aid_hit_quiet_ms;
    const bool reach_unknown=attempt_.placement.reach_line!=Evidence::yes;
    const bool guard=threat||hit_quiet||reach_unknown;
    // Visibility is a guard role, not cancellation: the same native transaction
    // must see the continuous beam. A real side hit or missing reach proof
    // explicitly pauses/rearms aid without changing the rest of the squad.
    const bool pause=threat||hit_quiet||reach_unknown;
    if(pause){if(attempt_.aid_requested){
            if(intent_revision_==(std::numeric_limits<std::uint64_t>::max)()){out.accepted=false;out.reason=CasualtyReason::invalid_input;return out;}
            attempt_.intent=++intent_revision_;}
        attempt_.aid_requested=false;}
    else if(!attempt_.aid_requested){
        if(intent_revision_==(std::numeric_limits<std::uint64_t>::max)()){out.accepted=false;out.reason=CasualtyReason::invalid_input;return out;}
        attempt_.intent=++intent_revision_;attempt_.requested=w.now_ms;attempt_.aid_requested=true;
        attempt_.baseline_phase=helper.aid_state.phase;attempt_.baseline_at=helper.aid_state.at_ms;
    }
    auto selected=std::find_if(out.slots.begin(),out.slots.end(),[&](const RoomDoctrineSlot& s){return s.actor==helper.key;});
    auto& order=selected->order;order.kind=OrderKind::recover;order.role=guard?roles::casualty_guard:roles::aid_casualty;
    order.goal_node=attempt_.node;order.goal_position=attempt_.goal;order.arrival_radius_m=c.aid_arrival_m;order.pace=Pace::walk;
    order.aid=pause?ActorKey{}:attempt_.target;order.throw_order={};order.finish={};order.pie.valid=false;order.corner.valid=false;
    order.portal.valid=false;order.lean=0;order.side=0;order.flags&=~(order_committed_entry|order_jamb|order_sprint);
    // The caller stamps final monotonic assignment identity from helper/target/
    // intent_revision. Never synthesize a globally colliding generation here.
    order.assignment.sector=order.assignment.portal=0;
    auto personal=first_response(helper,config_,w.now_ms,w.level_epoch,w.geometry_revision,0);
    order.look=personal.look;order.fire=personal.fire;
    if(!guard){order.fire={};order.look={LookKind::point,attempt_.target_position,{}};}
    const auto previous=attempt_.phase;attempt_.phase=guard?CasualtyPhase::guard:CasualtyPhase::reaching;
    out.reason=hit_quiet?CasualtyReason::hit_quiet:reach_unknown?CasualtyReason::no_projection:threat?CasualtyReason::visible_threat:CasualtyReason::travelling;
    const auto& receipt=helper.aid_state;
    if(!pause&&attempt_.aid_requested&&receipt.phase==2&&receipt.target==attempt_.target&&receipt.at_ms>=attempt_.requested&&receipt.at_ms<=w.now_ms
        &&(receipt.at_ms!=attempt_.baseline_at||receipt.phase!=attempt_.baseline_phase)){
        attempt_.phase=CasualtyPhase::working;out.reason=CasualtyReason::native_work;out.work_elapsed_ms=w.now_ms-receipt.at_ms;
        order.role=roles::aid_casualty;order.fire={};order.look={LookKind::point,attempt_.target_position,{}};
    }
    out.phase=attempt_.phase;out.helper=attempt_.helper;out.casualty=attempt_.target;out.generation=attempt_.generation;
    out.intent_revision=attempt_.intent;out.started_ms=attempt_.started;out.placement=attempt_.placement;
    // The helper never doubles as security. A second fit, uncommitted member
    // holds his actual position and retains a personal threat response; room
    // coverage and the rest of the side keep their own orders.
    const ActorSnapshot* cover=nullptr;
    for(auto key:side.members){poll(stop);if(key==attempt_.helper||key==attempt_.target)continue;
        const auto* a=find(w,key);const auto* s=slot(base,key);
        if(!a||!s||!fit_for_care(*a,base,w,config_)||a->aid_state.phase==2||
            (s->order.flags&(order_committed_entry|order_urgent))||s->order.kind==OrderKind::entry||
            s->order.kind==OrderKind::withdraw||valid_key(s->order.aid)||
            valid_key(s->order.finish)||s->order.throw_order.valid||
            first_response(*a,config_,w.now_ms,w.level_epoch,w.geometry_revision,0).kind==OrderKind::yield)continue;
        const bool preferred=key==security_,old_preferred=cover&&cover->key==security_;
        if(!cover||(preferred&&!old_preferred)||(!old_preferred&&!preferred&&
            (visible_able(*a,config_,w.now_ms)&&!visible_able(*cover,config_,w.now_ms)||
            (visible_able(*a,config_,w.now_ms)==visible_able(*cover,config_,w.now_ms)&&
             distance_squared(a->position,attempt_.target_position)<distance_squared(cover->position,attempt_.target_position)))))cover=a;
    }
    security_=cover?cover->key:ActorKey{};
    if(cover){auto security_slot=std::find_if(out.slots.begin(),out.slots.end(),[&](const RoomDoctrineSlot& s){return s.actor==cover->key;});
        if(security_slot!=out.slots.end()){const auto look=hazard(*cover);
            security_order(*security_slot,*cover,w,config_,look.first?&look.second:nullptr);out.security=cover->key;}}
    if(previous!=attempt_.phase)out.events.push_back({events::casualty,base.room,static_cast<std::uint32_t>(out.reason)});
    return out;
}
}}}
