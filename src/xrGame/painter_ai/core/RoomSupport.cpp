#include "RoomSupport.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace painter { namespace combat { namespace core {
namespace {
struct Cancelled{};
void poll(const std::function<bool()>& stop){if(stop&&stop())throw Cancelled{};}
const ActorSnapshot* actor(const WorldSnapshot& w,ActorKey k){for(const auto& a:w.actors)if(a.key==k)return &a;return nullptr;}
bool member(const SideSnapshot& s,ActorKey k){return std::find(s.members.begin(),s.members.end(),k)!=s.members.end();}
const RoomDoctrineSlot* slot(const RoomDoctrineResult& b,ActorKey k){for(const auto& s:b.slots)if(s.actor==k)return &s;return nullptr;}
const MemberKnowledge* knowledge(const KnowledgeFrame& k,const ActorSnapshot& a,const SideSnapshot& side){
    for(const auto& m:k.members)if(m.actor==a.key&&m.side==side.side_id&&m.ownership_generation==a.ownership_generation)return &m;
    return nullptr;
}
bool source(const ActorSnapshot& a,const WorldSnapshot& w,const Config& c){return valid_key(a.key)&&a.level_epoch==w.level_epoch
    &&a.snapshot_seq==w.sequence&&fresh(a.observed_at_ms,w.now_ms,c.snapshot_max_age_ms)&&finite(a.position);}
bool free_role(std::uint16_t r){switch(r){
    case roles::hold:case roles::search:case roles::search_sector:case roles::set:case roles::rear_security:
    case roles::cover_noise:case roles::cover_hit_dir:case roles::finisher:case roles::buddy_answer:return true;
    default:return false;}}
bool free_man(const ActorSnapshot& a,const RoomDoctrineSlot* s,const WorldSnapshot& w,const Config& c){
    if(!s||!source(a,w,c)||!a.alive||a.wounded||!std::isfinite(a.health)||a.health<=0||a.yield_reasons||!(a.flags&flag_admitted)||!a.ownership_generation
        ||!a.current_position_accessible||a.node_id==invalid_node||a.weapon.id==invalid_id
        ||(!a.weapon.ammo_in_mag&&!a.weapon.ammo_reserve)||a.aid_state.phase==2||a.throw_state.phase==1||a.throw_state.phase==2)return false;
    const auto& o=s->order;return o.actor==a.key&&o.level_epoch==w.level_epoch&&o.snapshot_seq==w.sequence
        &&o.geometry_revision==w.geometry_revision&&o.ownership_generation==a.ownership_generation
        &&o.restrictions_revision==a.restrictions_revision&&o.kind!=OrderKind::yield&&o.kind!=OrderKind::entry
        &&o.kind!=OrderKind::withdraw&&!(o.flags&order_committed_entry)&&!o.throw_order.valid&&!valid_key(o.aid)
        &&!o.pie.valid&&!o.corner.valid&&free_role(o.role);
}
const Contact* own_contact(const ActorSnapshot& a,ActorKey key,TimeMs now,const Config& c){
    auto valid=[&](const Contact& t){return t.key==key&&valid_key(t.key)&&t.key!=a.key&&t.hostile&&t.alive_when_observed
        &&finite(t.position)&&t.observed_at_ms<=a.observed_at_ms&&fresh(t.observed_at_ms,now,c.memory_max_age_ms);};
    if(valid(a.enemy))return &a.enemy;if(a.visible_enemies.count>a.visible_enemies.items.size())return nullptr;
    for(std::size_t i=0;i<a.visible_enemies.count;++i)if(valid(a.visible_enemies.items[i]))return &a.visible_enemies.items[i];return nullptr;
}
bool fighting(const ActorSnapshot& a,const Config& c,TimeMs now){
    auto live=[&](const Contact& t){return valid_key(t.key)&&t.hostile&&t.alive_when_observed&&!t.wounded
        &&personally_visible(t,a)&&fresh(t.observed_at_ms,now,c.snapshot_max_age_ms);};
    if(live(a.enemy)||a.visible_enemies.count>a.visible_enemies.items.size())return true;
    for(std::size_t i=0;i<a.visible_enemies.count;++i)if(live(a.visible_enemies.items[i]))return true;return false;
}
const FoeKnowledge* wounded(const MemberKnowledge& m,ActorKey target,TimeMs now,const RoomConfig& c){
    for(const auto& f:m.foes)if(!f.id.ghost()&&f.id.actor==target&&f.wounded&&!f.down&&finite(f.wounded_position)
        &&fresh(f.wounded_ms,now,c.finisher_keep_ms))return &f;return nullptr;
}
bool room_allowed(std::uint32_t r,const RoomDoctrineResult& b,const std::vector<std::uint32_t>* defended){return r!=invalid_room&&(r==b.room
    ||std::find(b.cleared_rooms.begin(),b.cleared_rooms.end(),r)!=b.cleared_rooms.end()
    ||(defended&&std::find(defended->begin(),defended->end(),r)!=defended->end()));}
const Building* building(const WorldSnapshot& w,const SideSnapshot& s,const EngineServices& e){
    if(!e.buildings||e.buildings->level_epoch!=w.level_epoch||e.buildings->map_hash!=w.map_hash)return nullptr;
    for(const auto& b:e.buildings->buildings)if(b.id==s.mission.building)return &b;return nullptr;
}
std::uint32_t room_of(const Building& b,std::uint32_t n){for(const auto& r:b.rooms)
    if(std::binary_search(r.node_ids.begin(),r.node_ids.end(),n))return r.id;return invalid_room;}
bool at_door(const Building& b,Vec3 q,const RoomConfig& c){for(const auto& p:b.portals){if(!p.doorway)continue;
    const float n=std::hypot(p.normal.x,p.normal.z);if(!(n>0))continue;
    const float dx=q.x-p.centre.x,dz=q.z-p.centre.z;
    if(std::abs(q.y-p.centre.y)<=c.floor_tolerance_m
        &&std::abs((dx*p.normal.x+dz*p.normal.z)/n)<=p.wall_depth*.5f+c.spacing_m
        &&std::abs((-dx*p.normal.z+dz*p.normal.x)/n)<=p.half_width+c.spacing_m)return true;}return false;}
void light(const Building& b,const BuildingCache& cache,std::uint32_t room,std::uint32_t node,
    const RoomDoctrineResult& base,const RoomConfig& c,RoomSupportCosts& costs){
    for(const auto& p:b.portals){const std::vector<Lit>* table=nullptr;std::uint32_t other=invalid_room;
        if(p.room_a==room){table=&p.lit_a;other=p.room_b;}else if(p.room_b==room){table=&p.lit_b;other=p.room_a;}
        if(!table||std::find(base.cleared_rooms.begin(),base.cleared_rooms.end(),other)!=base.cleared_rooms.end())continue;
        auto it=std::lower_bound(table->begin(),table->end(),node,[](const Lit& v,std::uint32_t n){return v.node<n;});
        if(it!=table->end()&&it->node==node)costs.exposure_m+=c.exposure_cost_m*float(it->standing)/255.f;
        else if(!cache.lit_complete)costs.unknown_m+=c.unknown_cost_m;
    }
}
bool occupied(Vec3 p,ActorKey self,const WorldSnapshot& w,const RoomDoctrineResult& b,float spacing){
    for(const auto& s:b.slots){if(s.actor==self)continue;const auto* a=actor(w,s.actor);if(!a||!a->alive)continue;
        if(distance_squared(p,a->position)<spacing*spacing||distance_squared(p,s.order.goal_position)<spacing*spacing)return true;}return false;
}
void attention(RoomDoctrineSlot& s,const ActorSnapshot& a,ActorKey key,Vec3 point,TimeMs now,const Config& c){
    auto& o=s.order;o.look.kind=LookKind::point;o.look.value=point;o.look.target={};
    o.pie={};o.corner={};o.flags&=~order_face_movement;o.fire.allowed=false;o.fire.at_valid=false;o.fire.until_ms=now;
    s.authored_look=o.look;s.authored_look_valid=true;
    const auto* t=own_contact(a,key,now,c);
    if(t&&personally_visible(*t,a)&&(!t->wounded||t->finish_allowed)){
        o.look.kind=LookKind::enemy;o.look.target=t->key;
        // No fabricated line proof: executor's current ray/aim/friend gates remain authoritative.
        o.fire.allowed=t->key==a.enemy.key&&!a.reloading&&a.weapon.state!=WeaponState::reloading
            &&a.weapon.ammo_in_mag>0&&!(a.flags&flag_dont_shoot)&&c.fire_window_ms>0;
        if(o.fire.allowed)o.fire.until_ms=fire_window_until(c,now,o.expires_at_ms);
    }
}
}
RoomSupport::RoomSupport(Config c):config_(c){}
void RoomSupport::reset(){finish_={};answer_={};skipped_.clear();epoch_=map_=geometry_=sequence_=0;
    side_=mission_=0;now_=last_fight_=0;fight_seen_=false;}
RoomSupportResult RoomSupport::update(const WorldSnapshot& w,const SideSnapshot& s,const RoomDoctrineResult& b,
    const KnowledgeFrame* k,const EngineServices& e,std::size_t quantum,const std::function<bool()>& stop,
    const std::vector<std::uint32_t>* defended){
    RoomSupportResult failed;failed.reason=SupportReason::invalid_input;
    try{poll(stop);auto staged=*this;auto out=staged.update_impl(w,s,b,k,e,quantum,stop,defended);poll(stop);
        if(out.accepted)*this=std::move(staged);return out;
    }catch(const Cancelled&){failed.cancelled=true;failed.reason=SupportReason::cancelled;return failed;}
    catch(...){return failed;}
}
RoomSupportResult RoomSupport::update_impl(const WorldSnapshot& w,const SideSnapshot& side,const RoomDoctrineResult& base,
    const KnowledgeFrame* k,const EngineServices& services,std::size_t quantum,const std::function<bool()>& stop,
    const std::vector<std::uint32_t>* defended){
    RoomSupportResult out;out.reason=SupportReason::invalid_input;const auto& c=config_.room;
    if(!w.level_epoch||!w.sequence||!base.accepted||base.level_epoch!=w.level_epoch||base.snapshot_seq!=w.sequence
        ||!k||!k->accepted||k->cancelled||k->level_epoch!=w.level_epoch||k->sequence!=w.sequence
        ||!c.finisher_tries||!c.finisher_try_ms||!c.finisher_cap_ms||!c.finisher_keep_ms||!c.path_nodes
        ||!std::isfinite(c.projection_m)||c.projection_m<0||!std::isfinite(c.floor_tolerance_m)||c.floor_tolerance_m<0
        ||!std::isfinite(c.spacing_m)||c.spacing_m<=0||!std::isfinite(c.arrival_m)||c.arrival_m<=0
        ||!std::isfinite(c.look_height_m)||c.look_height_m<0||!std::isfinite(c.approach_weight)||c.approach_weight<0
        ||!std::isfinite(c.exposure_cost_m)||c.exposure_cost_m<0||!std::isfinite(c.unknown_cost_m)||c.unknown_cost_m<0)return out;
    for(float ring:c.finisher_rings_m)if(!std::isfinite(ring)||ring<=0)return out;
    if(epoch_&&w.level_epoch<epoch_)return out;
    if(epoch_==w.level_epoch&&(w.sequence<=sequence_||w.now_ms<now_))return out;
    std::set<std::uint32_t> ids;for(auto key:side.members){poll(stop);if(!valid_key(key)||!ids.insert(key.id).second)return out;}
    ids.clear();for(const auto& s:base.slots){poll(stop);if(!valid_key(s.actor)||!ids.insert(s.actor.id).second||s.order.actor!=s.actor)return out;}
    if(epoch_!=w.level_epoch||map_!=w.map_hash||geometry_!=w.geometry_revision||side_!=side.side_id||mission_!=side.mission.revision){
        reset();epoch_=w.level_epoch;map_=w.map_hash;geometry_=w.geometry_revision;side_=side.side_id;mission_=side.mission.revision;}
    sequence_=w.sequence;now_=w.now_ms;out.accepted=true;out.reason=SupportReason::none;out.slots=base.slots;
    out.level_epoch=w.level_epoch;out.snapshot_seq=w.sequence;out.knowledge_seq=k->sequence;
    skipped_.erase(std::remove_if(skipped_.begin(),skipped_.end(),[&](const Skip& s){return w.now_ms>=s.until;}),skipped_.end());
    const SideKnowledge* sk=nullptr;for(const auto& s:k->sides)if(s.side==side.side_id){sk=&s;break;}
    auto down=[&](ActorKey key){if(sk&&std::find(sk->confirmed_down.begin(),sk->confirmed_down.end(),key)!=sk->confirmed_down.end())return true;
        for(auto mk:side.members){const auto* a=actor(w,mk);if(!a||!source(*a,w,config_))continue;
            if(a->seen_dead.count>a->seen_dead.items.size())continue;
            for(std::size_t i=0;i<a->seen_dead.count;++i)if(a->seen_dead.items[i]==key)return true;}return false;};
    for(auto key:side.members){poll(stop);const auto* a=actor(w,key);if(!a||!source(*a,w,config_))continue;
        const auto* m=knowledge(*k,*a,side);if(!m)continue;
        if(fighting(*a,config_,w.now_ms)){last_fight_=w.now_ms;fight_seen_=true;}
        if(m->has_hit&&m->last_hit_ms<=w.now_ms){last_fight_=fight_seen_?(std::max)(last_fight_,m->last_hit_ms):m->last_hit_ms;fight_seen_=true;}
    }
    if(sk)for(const auto& h:sk->hits)if(h.valid&&h.at_ms<=w.now_ms&&member(side,h.observer)){
        last_fight_=fight_seen_?(std::max)(last_fight_,h.at_ms):h.at_ms;fight_seen_=true;}
    // One hit episode stays with its responder until its measured observation window expires.
    if(valid_key(answer_.actor)){
        const auto* a=actor(w,answer_.actor);const auto* s=slot(base,answer_.actor);
        if(!a||!member(side,a->key)||!free_man(*a,s,w,config_)||!knowledge(*k,*a,side)
            ||a->ownership_generation!=answer_.owner||s->order.assignment.generation!=answer_.assignment
            ||answer_.at>w.now_ms||w.now_ms>=answer_.until)answer_={};
    }
    if(!valid_key(answer_.actor)&&sk&&c.buddy_answer_ms){
        const HitKnowledge* hit=nullptr;for(const auto& h:sk->hits){poll(stop);
            const auto* victim=actor(w,h.observer);
            const auto window=h.unseen?config_.unseen_reaction_source_ms:c.buddy_answer_ms;
            if(!h.valid||!victim||!source(*victim,w,config_)||!member(side,h.observer)||!finite(h.point)||!finite(h.origin)
                ||h.at_ms>victim->observed_at_ms||!fresh(h.at_ms,w.now_ms,window)||w.now_ms-h.at_ms>=window)continue;
            if(!h.unseen&&(h.shooter.ghost()||!valid_key(h.shooter.actor)))continue;
            if(h.unseen){bool watched=false;
                for(auto key:side.members){const auto* a=actor(w,key);if(!a||!source(*a,w,config_)||!a->alive||a->wounded)continue;
                    const Vec3 to{h.point.x-a->position.x,h.point.y-a->position.y-config_.knowledge.eye_m,h.point.z-a->position.z};
                    const float distance=std::sqrt(distance_squared(to,{})),head=std::sqrt(distance_squared(a->head_dir,{}));
                    if(distance<=c.see_range_m&&distance>0&&head>.5f&&
                        (to.x*a->head_dir.x+to.y*a->head_dir.y+to.z*a->head_dir.z)/(distance*head)>=std::cos(c.check_half_deg*3.14159265358979323846f/180.f))watched=true;
                }if(watched)continue;}
            if(!hit||h.at_ms>hit->at_ms)hit=&h;}
        if(hit){const ActorSnapshot* best=nullptr;float distance=(std::numeric_limits<float>::max)();bool best_own=false;
            const auto* victim=actor(w,hit->observer);
            for(auto key:side.members){poll(stop);const auto* a=actor(w,key);if(key==hit->observer||!a||!free_man(*a,slot(base,key),w,config_)||!knowledge(*k,*a,side)||
                    (hit->unseen&&(a->reloading||a->weapon.state==WeaponState::reloading)))continue;
                const auto* contact=hit->unseen||hit->shooter.ghost()?nullptr:own_contact(*a,hit->shooter.actor,w.now_ms,config_);
                const bool owns=contact&&personally_visible(*contact,*a)&&(!contact->wounded||contact->finish_allowed);
                // Do not steal a man from his own different visible fight.
                if(fighting(*a,config_,w.now_ms)&&!owns)continue;
                const float d=distance_squared(a->position,victim&&source(*victim,w,config_)?victim->position:hit->origin);
                if(std::isfinite(d)&&(!best||(owns&&!best_own)||(owns==best_own&&d<distance))){best=a;distance=d;best_own=owns;}}
            if(best){const auto* s=slot(base,best->key);answer_={best->key,hit->observer,hit->unseen||hit->shooter.ghost()?ActorKey{}:hit->shooter.actor,
                // K1 point is the personally observed/reported shooter point;
                // origin is the observing victim's location, not the attacker.
                best->ownership_generation,s->order.assignment.generation,++intent_,hit->point,hit->at_ms,hit->unseen,
                saturated_after(hit->unseen?w.now_ms:hit->at_ms,hit->unseen?config_.unseen_reaction_hold_ms:c.buddy_answer_ms)};finish_={};}
        }
    }
    if(valid_key(answer_.actor)){auto* a=actor(w,answer_.actor);for(auto& s:out.slots)if(s.actor==answer_.actor){
            attention(s,*a,answer_.target,answer_.origin,w.now_ms,config_);s.order.role=answer_.unseen?roles::cover_hit_dir:roles::buddy_answer;}
        out.actor=answer_.actor;out.target=answer_.target;out.intent_revision=answer_.intent;out.phase=SupportPhase::answering;out.reason=SupportReason::hit_answer;return out;}
    if(valid_key(finish_.target)&&down(finish_.target)){out.actor=finish_.actor;out.target=finish_.target;
        out.phase=SupportPhase::done;out.reason=SupportReason::observed_down;finish_={};return out;}
    if(fight_seen_&&w.now_ms-last_fight_<c.finisher_quiet_ms){finish_={};out.phase=SupportPhase::paused;out.reason=SupportReason::quiet;return out;}
    const auto* b=building(w,side,services);
    if(!b||!services.project||!services.path_length||!services.rays){out.reason=SupportReason::no_place;return out;}
    auto skipped=[&](ActorKey key){for(const auto& s:skipped_)if(s.target==key&&w.now_ms<s.until)return true;return false;};
    auto abandon=[&](SupportReason reason){out.actor=finish_.actor;out.target=finish_.target;out.tries=finish_.tries;
        out.phase=SupportPhase::skipped;out.reason=reason;skipped_.push_back({finish_.target,saturated_after(w.now_ms,c.finisher_skip_ms)});finish_={};};
    if(valid_key(finish_.actor)){
        const auto* a=actor(w,finish_.actor);const auto* s=slot(base,finish_.actor);
        const auto* m=a?knowledge(*k,*a,side):nullptr;
        if(!a||!m||!member(side,a->key)||!free_man(*a,s,w,config_)||a->ownership_generation!=finish_.owner
            ||a->restrictions_revision!=finish_.restrictions||s->order.assignment.generation!=finish_.assignment){
            finish_={};out.reason=SupportReason::member_lost;return out;}
        const auto* f=wounded(*m,finish_.target,w.now_ms,c);
        if(!f||distance_squared(f->wounded_position,finish_.target_position)>c.arrival_m*c.arrival_m){finish_={};out.reason=SupportReason::target_lost;return out;}
        if(w.now_ms-finish_.started>=c.finisher_cap_ms){abandon(SupportReason::cap);return out;}
    }
    if(!valid_key(finish_.actor)){
        float best=(std::numeric_limits<float>::max)();const ActorSnapshot* chosen=nullptr;const FoeKnowledge* target=nullptr;
        for(auto key:side.members){poll(stop);const auto* a=actor(w,key);if(!a||!free_man(*a,slot(base,key),w,config_)||!room_allowed(room_of(*b,a->node_id),base,defended))continue;
            const auto* m=knowledge(*k,*a,side);if(!m)continue;
            for(const auto& f:m->foes){poll(stop);if(f.id.ghost()||!valid_key(f.id.actor)||member(side,f.id.actor)||!f.wounded||f.down
                    ||!finite(f.wounded_position)||!fresh(f.wounded_ms,w.now_ms,c.finisher_keep_ms)||skipped(f.id.actor)||down(f.id.actor))continue;
                const float d=distance_squared(a->position,f.wounded_position);if(std::isfinite(d)&&d<best){best=d;chosen=a;target=&f;}}}
        if(!chosen){out.reason=SupportReason::no_free_member;return out;}
        const auto* s=slot(base,chosen->key);finish_.actor=chosen->key;finish_.target=target->id.actor;finish_.owner=chosen->ownership_generation;
        finish_.restrictions=chosen->restrictions_revision;finish_.assignment=s->order.assignment.generation;finish_.intent=++intent_;
        finish_.started=w.now_ms;finish_.target_position=target->wounded_position;
    }
    const auto& a=*actor(w,finish_.actor);
    if(finish_.node!=invalid_node){
        const auto& r=a.last_rejected;const bool rejected=r.node==finish_.node&&r.restrictions_revision==a.restrictions_revision
            &&(r.reason==reject_goal_invalid||r.reason==reject_goal_inaccessible);
        if(distance_squared(a.position,finish_.goal)<=c.arrival_m*c.arrival_m&&!finish_.has_arrived){finish_.arrived=w.now_ms;finish_.has_arrived=true;}
        if(rejected||(finish_.has_arrived&&w.now_ms-finish_.arrived>=c.finisher_try_ms)){
            finish_.tried.push_back(finish_.node);++finish_.tries;finish_.node=invalid_node;finish_.best_node=invalid_node;
            finish_.cursor=0;finish_.scanned.clear();finish_.has_arrived=false;finish_.intent=++intent_;
            if(finish_.tries>=c.finisher_tries){abandon(SupportReason::tries);return out;}}
    }
    // Reference FLOW.finish_cell's 16 angular samples are an algorithmic sampling
    // pattern, not a new tuning setting. Each candidate charges one caller quantum
    // and at most one projection, bounded path query and static line query.
    constexpr std::size_t angles=16;constexpr float pi=3.14159265358979323846f;
    const std::size_t total=c.finisher_rings_m.size()*angles;
    if(finish_.node==invalid_node){
        while(finish_.cursor<total&&out.costs.candidate_work<quantum){poll(stop);const auto i=finish_.cursor++;
            ++out.costs.candidate_work;const float radius=c.finisher_rings_m[i/angles],angle=float(i%angles)*(2*pi/float(angles));
            const Vec3 p{finish_.target_position.x+radius*std::cos(angle),finish_.target_position.y,finish_.target_position.z+radius*std::sin(angle)};Vec3 q;
            const auto node=services.project(p,c.projection_m,c.floor_tolerance_m,&q);++out.costs.project_queries;poll(stop);
            if(node==invalid_node||!finite(q)||std::abs(q.y-p.y)>c.floor_tolerance_m
                ||(q.x-p.x)*(q.x-p.x)+(q.z-p.z)*(q.z-p.z)>c.projection_m*c.projection_m
                ||std::find(finish_.tried.begin(),finish_.tried.end(),node)!=finish_.tried.end()
                ||std::find(finish_.scanned.begin(),finish_.scanned.end(),node)!=finish_.scanned.end())continue;
            finish_.scanned.push_back(node);const auto room=room_of(*b,node);
            if(!room_allowed(room,base,defended)||at_door(*b,q,c)||occupied(q,a.key,w,base,c.spacing_m))continue;
            const auto& rejected=a.last_rejected;
            if(rejected.node==node&&rejected.restrictions_revision==a.restrictions_revision
                &&(rejected.reason==reject_goal_invalid||rejected.reason==reject_goal_inaccessible))continue;
            const float path=services.path_length(a.node_id,node,c.path_nodes);++out.costs.path_queries;poll(stop);
            if(!std::isfinite(path)||path<0)continue;
            Segment ray{q,finish_.target_position};ray.a.y+=c.look_height_m;ray.b.y+=c.look_height_m;float hit=-2;
            services.rays(&ray,&hit,1);++out.costs.ray_queries;poll(stop);
            if(!std::isfinite(hit)||hit< -1){++out.costs.unknown_rays;continue;}if(hit>=0)continue;
            RoomSupportCosts costs;costs.path_m=path;light(*b,*services.buildings,room,node,base,c,costs);
            const float cost=path*c.approach_weight+costs.exposure_m+costs.unknown_m;
            if(finish_.best_node==invalid_node||cost<finish_.best_cost){finish_.best=q;finish_.best_node=node;finish_.best_cost=cost;finish_.best_costs=costs;}
        }
        if(finish_.cursor<total){out.pending=true;out.phase=SupportPhase::seeking;out.reason=SupportReason::pending_geometry;
            out.actor=finish_.actor;out.target=finish_.target;out.intent_revision=finish_.intent;return out;}
        if(finish_.best_node==invalid_node){abandon(out.costs.unknown_rays?SupportReason::unknown_line:SupportReason::no_place);return out;}
        finish_.node=finish_.best_node;finish_.goal=finish_.best;
    }
    // Recheck moving friendly occupants after a sliced search; do not publish an
    // obsolete candidate merely because it was the best on an earlier snapshot.
    if(occupied(finish_.goal,a.key,w,base,c.spacing_m)){finish_.tried.push_back(finish_.node);finish_.node=invalid_node;
        finish_.best_node=invalid_node;finish_.cursor=0;finish_.scanned.clear();out.pending=true;
        out.phase=SupportPhase::seeking;out.reason=SupportReason::pending_geometry;return out;}
    for(auto& s:out.slots)if(s.actor==finish_.actor){auto& o=s.order;o.kind=OrderKind::hold;o.role=roles::finisher;
        o.goal_node=finish_.node;o.goal_position=finish_.goal;o.arrival_radius_m=c.arrival_m;o.pace=Pace::walk;o.finish=finish_.target;
        attention(s,a,finish_.target,finish_.target_position,w.now_ms,config_);}
    out.actor=finish_.actor;out.target=finish_.target;out.intent_revision=finish_.intent;out.tries=finish_.tries;
    out.phase=finish_.has_arrived?SupportPhase::finishing:SupportPhase::reaching;out.reason=SupportReason::wounded_known;
    out.costs.path_m=finish_.best_costs.path_m;out.costs.exposure_m=finish_.best_costs.exposure_m;out.costs.unknown_m=finish_.best_costs.unknown_m;return out;
}
}}}
