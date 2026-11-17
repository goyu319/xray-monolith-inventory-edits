#include "RoomLeader.h"
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
namespace painter { namespace combat { namespace core {
namespace {
struct Cancelled{};
void poll(const std::function<bool()>& f){if(f&&f())throw Cancelled{};}
bool valid(ActorKey k){return k.id<invalid_id&&k.generation;}
bool finite(Vec3 p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
float distance(Vec3 a,Vec3 b){return std::hypot(a.x-b.x,a.z-b.z);}
bool same(Vec3 a,Vec3 b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
bool fresh(TimeMs at,TimeMs now,TimeMs age){return at<=now&&now-at<=age;}
TimeMs remaining(TimeMs elapsed,TimeMs cap){return elapsed>=cap?0:cap-elapsed;}
float fraction(TimeMs left,TimeMs cap){return cap?float(double(left)/double(cap)):0.f;}
bool line_role(std::uint16_t role){return role==roles::door_stack_point||role==roles::door_stack||
    role==roles::door_post_ready||role==roles::entry_wait||role==roles::rear_security||role==roles::two_up;}
bool visible(const Contact& t,const ActorSnapshot& a,const WorldSnapshot& w,const Config& c){return valid(t.key)&&t.hostile&&
    t.alive_when_observed&&!t.wounded&&t.visible&&t.observed_at_ms<=a.observed_at_ms&&fresh(t.observed_at_ms,w.now_ms,c.snapshot_max_age_ms);}
bool recent(const Contact& t,const ActorSnapshot& a,const WorldSnapshot& w,const Config& c){return valid(t.key)&&t.hostile&&t.alive_when_observed&&!t.wounded&&
    t.last_seen_ms<=t.observed_at_ms&&t.observed_at_ms<=a.observed_at_ms&&fresh(t.last_seen_ms,w.now_ms,c.room.leader_no_shot_ms);}
bool applied(const Order& o,const ActorSnapshot& a,const WorldSnapshot& w){return o.actor==a.key&&o.level_epoch==w.level_epoch&&
    o.geometry_revision==w.geometry_revision&&o.ownership_generation==a.ownership_generation&&o.restrictions_revision==a.restrictions_revision&&
    o.order_seq&&a.last_applied.order_seq==o.order_seq&&o.snapshot_seq<=w.sequence&&o.issued_at_ms<=a.last_applied.at_ms&&
    a.last_applied.at_ms<=a.observed_at_ms&&o.expires_at_ms>w.now_ms&&finite(o.goal_position);}
}
RoomLeader::RoomLeader(Config c):config_(c){}
void RoomLeader::reset(){*this=RoomLeader(config_);}
RoomLeaderResult RoomLeader::update(const WorldSnapshot& w,const SideSnapshot& s,const RoomLeaderRequest& r,const std::function<bool()>& stop){
    RoomLeader next=*this;try{poll(stop);auto out=next.update_impl(w,s,r,stop);poll(stop);if(out.accepted)*this=std::move(next);return out;
    }catch(const Cancelled&){RoomLeaderResult out;out.cancelled=true;return out;}}
RoomLeaderResult RoomLeader::update_impl(const WorldSnapshot& w,const SideSnapshot& s,const RoomLeaderRequest& r,const std::function<bool()>& stop){
    RoomLeaderResult out;out.level_epoch=w.level_epoch;out.snapshot_seq=w.sequence;out.attempt=r.attempt;out.point=r.point;const auto& c=config_.room;
    if(!w.level_epoch||!w.sequence||!r.attempt||r.room==invalid_room||
        !std::isfinite(c.leader_progress_m)||c.leader_progress_m<0||!std::isfinite(c.leader_heel_m)||c.leader_heel_m<0||
        !std::isfinite(c.leader_point_m)||c.leader_point_m<0||!std::isfinite(c.leader_set_m)||c.leader_set_m<0)return out;
    if(initialized_&&(w.level_epoch<epoch_||(w.level_epoch==epoch_&&(w.sequence<=sequence_||w.now_ms<now_))))return out;
    const bool context=!initialized_||w.level_epoch!=epoch_||w.map_hash!=map_||w.geometry_revision!=geometry_||
        s.side_id!=side_||s.group!=group_||s.faction!=faction_||s.mission.revision!=mission_||r.attempt!=attempt_||r.room!=room_;
    if(context){reset();last_progress_=phase_since_=w.now_ms;}
    if(context||phase_!=r.phase){phase_=r.phase;phase_since_=last_progress_=w.now_ms;up_started_=go_started_=false;tracks_.clear();}
    std::unordered_map<std::uint32_t,ActorKey> roster;for(auto k:s.members){poll(stop);if(!valid(k)||!roster.emplace(k.id,k).second)return out;}
    std::unordered_map<std::uint32_t,const ActorSnapshot*> actors;for(const auto& a:w.actors){poll(stop);if(!actors.emplace(a.key.id,&a).second)return out;}
    std::unordered_map<std::uint32_t,const Order*> orders;for(const auto& o:r.accepted_orders){poll(stop);if(!orders.emplace(o.actor.id,&o).second)return out;}
    struct Member {const ActorSnapshot* a=nullptr;const Order* o=nullptr;bool able=false,seeing=false;};
    std::vector<Member> members;std::unordered_set<std::uint32_t> requested;
    for(auto k:r.members){poll(stop);if(!valid(k)||!requested.insert(k.id).second)return out;
        const auto side_key=roster.find(k.id);if(side_key==roster.end()||side_key->second!=k)return out;
        const auto a=actors.find(k.id);if(a==actors.end()||a->second->key!=k||a->second->level_epoch!=w.level_epoch||
            a->second->snapshot_seq!=w.sequence||!fresh(a->second->observed_at_ms,w.now_ms,config_.snapshot_max_age_ms)||!finite(a->second->position)){
            ++out.unknown_members;continue;}
        Member m;m.a=a->second;const auto o=orders.find(k.id);if(o!=orders.end()&&applied(*o->second,*m.a,w)){m.o=o->second;++out.execution_confirmed;
            if(m.o->flags&order_committed_entry)released_=true;}
        m.able=m.a->alive&&!m.a->wounded&&m.a->health>0&&std::isfinite(m.a->health)&&m.a->ownership_generation&&
            (m.a->flags&flag_admitted)&&!m.a->yield_reasons;
        if(m.able){++out.able;if(!valid(out.leader))out.leader=k;
            m.seeing=visible(m.a->enemy,*m.a,w,config_);out.fight=out.fight||recent(m.a->enemy,*m.a,w,config_);
            for(std::size_t i=0;i<(std::min)(std::size_t(m.a->visible_enemies.count),m.a->visible_enemies.items.size());++i){
                const auto& t=m.a->visible_enemies.items[i];m.seeing=m.seeing||visible(t,*m.a,w,config_);out.fight=out.fight||recent(t,*m.a,w,config_);}
            out.seeing=out.seeing||m.seeing;
            if(m.o&&m.a->last_applied.fired_since_previous&&fresh(m.a->last_applied.at_ms,w.now_ms,config_.snapshot_max_age_ms)&&
                (!shot_||m.a->last_applied.at_ms>last_shot_)){shot_=true;last_shot_=m.a->last_applied.at_ms;
                last_progress_=(std::max)(last_progress_,last_shot_);out.progressed=true;}}
        members.push_back(m);
    }
    auto fact=[&](const RoomLeaderFact& f){if(!f.valid||f.level_epoch!=w.level_epoch||f.snapshot_seq!=w.sequence||f.geometry_revision!=w.geometry_revision||
        f.attempt!=r.attempt||!fresh(f.at_ms,w.now_ms,config_.snapshot_max_age_ms))return false;
        for(const auto& m:members)if(m.able&&m.o&&m.a->key==f.observer&&m.a->ownership_generation==f.ownership_generation)return true;return false;};
    if(fact(r.progress)&&r.progress.at_ms>last_fact_){last_fact_=r.progress.at_ms;last_progress_=(std::max)(last_progress_,last_fact_);out.progressed=true;}
    std::unordered_map<std::uint32_t,const Track*> tracked;for(const auto& t:tracks_){poll(stop);tracked.emplace(t.actor.id,&t);}
    std::vector<Track> current_tracks;
    for(const auto& m:members){poll(stop);if(!m.able||!m.o)continue;const auto& a=*m.a;const auto& o=*m.o;const float dd=distance(a.position,o.goal_position);
        const auto found=tracked.find(a.key.id);const Track* old=found==tracked.end()?nullptr:found->second;
        if(old&&(old->actor!=a.key||old->owner!=a.ownership_generation||old->restrictions!=a.restrictions_revision||
            old->assignment!=o.assignment.generation||old->node!=o.goal_node||!same(old->goal,o.goal_position)))old=nullptr;
        Track t{a.key,a.ownership_generation,a.restrictions_revision,o.assignment.generation,o.goal_node,o.goal_position,dd};
        if(old){t.best_distance=old->best_distance;
            if(dd<old->best_distance-c.leader_progress_m){t.best_distance=dd;
                if(r.phase!=LeaderPhase::entry||out.fight){last_progress_=w.now_ms;out.progressed=true;}}}
        current_tracks.push_back(t);
    }tracks_=std::move(current_tracks);
    const Member* point=nullptr;for(const auto& m:members)if(m.a->key==r.point)point=&m;
    if(point&&point->able&&point->o&&distance(point->a->position,point->o->goal_position)<=c.leader_point_m){
        bool other=false,heel=false;for(const auto& m:members)if(m.able&&m.a->key!=r.point){other=true;
            if(m.o&&distance(m.a->position,point->a->position)<=c.leader_heel_m)heel=true;}
        out.stacked=!other||heel;}
    out.line_set=out.able>0&&!out.unknown_members;
    for(const auto& m:members)if(m.able&&(!m.o||(!m.seeing&&line_role(m.o->role)&&distance(m.a->position,m.o->goal_position)>c.leader_set_m)))out.line_set=false;
    if(out.seeing&&!seeing_)seen_since_=w.now_ms;seeing_=out.seeing;
    out.idle_ms=w.now_ms-last_progress_;const TimeMs shot_base=shot_?(std::max)(last_shot_,phase_since_):phase_since_;
    out.no_shot_ms=w.now_ms-shot_base;out.stuck_fight=r.phase==LeaderPhase::entry&&out.fight&&out.no_shot_ms>=c.leader_no_shot_ms;
    out.released=released_;out.ready_now=fact(r.phase_ready)||(r.phase==LeaderPhase::form&&out.stacked&&out.line_set);
    LeaderCall call=r.phase==LeaderPhase::form?LeaderCall::pie:r.phase==LeaderPhase::pie?LeaderCall::entry:r.phase==LeaderPhase::entry?LeaderCall::go:LeaderCall::none;
    if(!released_&&out.able){
        if(call==LeaderCall::pie&&!out.stacked){if(!up_started_){up_started_=true;up_since_=w.now_ms;}
            out.up_remaining_ms=remaining(w.now_ms-up_since_,c.leader_up_wait_ms);if(out.up_remaining_ms)call=LeaderCall::up;}
        else if(out.stacked)up_started_=false;
        if((call==LeaderCall::entry||call==LeaderCall::go)&&!out.stuck_fight&&!out.line_set){
            if(!go_started_){go_started_=true;go_since_=w.now_ms;}out.go_remaining_ms=remaining(w.now_ms-go_since_,c.leader_go_wait_ms);
            if(out.go_remaining_ms)call=LeaderCall::up;
        }else if(out.line_set)go_started_=false;
        const bool held=fact(r.threat_hold);
        if((r.phase==LeaderPhase::pie&&out.seeing)||(r.phase==LeaderPhase::entry&&held)){
            const TimeMs base=out.seeing?(std::max)(shot_base,seen_since_):shot_base;
            out.no_shot_remaining_ms=remaining(w.now_ms-base,c.leader_no_shot_ms);
            if(out.no_shot_remaining_ms&&(call==LeaderCall::entry||call==LeaderCall::go))call=LeaderCall::none;
        }
        if(out.ready_now||out.idle_ms>=c.leader_idle_ms||out.stuck_fight){out.suggestion=call;
            if(call!=LeaderCall::none)last_progress_=w.now_ms;}
    }
    out.up_cost_fraction=fraction(out.up_remaining_ms,c.leader_up_wait_ms);out.go_cost_fraction=fraction(out.go_remaining_ms,c.leader_go_wait_ms);
    out.no_shot_cost_fraction=fraction(out.no_shot_remaining_ms,c.leader_no_shot_ms);
    initialized_=true;epoch_=w.level_epoch;map_=w.map_hash;geometry_=w.geometry_revision;sequence_=w.sequence;now_=w.now_ms;
    side_=s.side_id;faction_=s.faction;group_=s.group;mission_=s.mission.revision;attempt_=r.attempt;room_=r.room;
    out.accepted=true;return out;
}
}}}
