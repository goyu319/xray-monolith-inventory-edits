#include "RoomDefense.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

namespace painter { namespace combat { namespace core {
namespace {
struct Cancelled{};void poll(const std::function<bool()>& f){if(f&&f())throw Cancelled{};}
float flat(Vec3 a,Vec3 b){const float x=a.x-b.x,z=a.z-b.z;return std::sqrt(x*x+z*z);}
Vec3 add(Vec3 a,Vec3 b,float k=1){return {a.x+k*b.x,a.y+k*b.y,a.z+k*b.z};}
Vec3 unit(Vec3 v){float n=std::sqrt(v.x*v.x+v.z*v.z);return n>0?Vec3{v.x/n,0,v.z/n}:Vec3{};}
float dot(Vec3 a,Vec3 b){return a.x*b.x+a.z*b.z;}
bool in(const Room& r,Vec3 p,float vertical){return p.x>=r.min.x&&p.x<=r.max.x&&p.z>=r.min.z&&p.z<=r.max.z&&std::abs(p.y-r.floor_y)<=vertical;}
const Room* room(const Building& b,std::uint32_t id){for(const auto& r:b.rooms)if(r.id==id)return &r;return nullptr;}
const Portal* door(const Building& b,std::uint32_t id){for(const auto& p:b.portals)if(p.id==id)return &p;return nullptr;}
Vec3 outward(const Portal& p,std::uint32_t r){return unit(p.room_a==r?p.normal:add({},p.normal,-1));}
Vec3 tangent(const Portal& p){auto t=unit(add(p.jamb_b,p.jamb_a,-1));return dot(t,t)>0?t:Vec3{p.normal.z,0,-p.normal.x};}
std::uint32_t other(const Portal& p,std::uint32_t r){return p.room_a==r?p.room_b:p.room_a;}
bool current(const ActorSnapshot& a,const WorldSnapshot& w,const Config& c){return valid_key(a.key)&&a.level_epoch==w.level_epoch
    &&a.snapshot_seq==w.sequence&&fresh(a.observed_at_ms,w.now_ms,c.snapshot_max_age_ms)&&finite(a.position)&&std::isfinite(a.health);}
bool controllable(const ActorSnapshot& a){return a.alive&&!a.wounded&&!a.yield_reasons&&(a.flags&flag_admitted)
    &&a.ownership_generation&&a.current_position_accessible&&a.node_id!=invalid_node&&std::isfinite(a.health)&&a.health>0
    &&finite(a.forward)&&a.weapon.id!=invalid_id&&(a.weapon.ammo_in_mag||a.weapon.ammo_reserve)
    &&a.hits.count<=a.hits.items.size()&&a.visible_enemies.count<=a.visible_enemies.items.size()
    &&a.heard.count<=a.heard.items.size()&&a.seen_dead.count<=a.seen_dead.items.size();}
bool reserved(const Order* o){return o&&(o->kind==OrderKind::yield||o->throw_order.valid||valid_key(o->aid));}
bool visible_able(const Contact& t,const ActorSnapshot& a,TimeMs now,const Config& c){return valid_key(t.key)&&t.key!=a.key&&t.hostile
    &&t.alive_when_observed&&!t.wounded&&finite(t.position)&&personally_visible(t,a)&&fresh(t.observed_at_ms,now,c.memory_max_age_ms);}
bool doorway(const Building& b,Vec3 pos,const RoomConfig& c){for(const auto& p:b.portals){auto q=add(pos,p.centre,-1);
    const auto* floor=room(b,p.room_a);if(floor)q.y=pos.y-floor->floor_y;
    if(std::abs(q.y)<=c.floor_tolerance_m&&std::abs(dot(q,outward(p,p.room_a)))<=c.entry_zone_depth_m
        &&std::abs(dot(q,tangent(p)))<=p.half_width+c.entry_zone_half_m)return true;}return false;}
struct Threat {FoeId id;Vec3 position;TimeMs at=0;bool down=false;TimeMs down_at=0;};
const SourceFact* latest(const FoeKnowledge& f){const SourceFact* p=nullptr;for(const auto& s:f.sources)
    if(s.valid&&finite(s.position)&&(!p||s.at_ms>p->at_ms))p=&s;return p;}
}
RoomDefense::RoomDefense(Config c):config_(c){}
void RoomDefense::reset(){men_.clear();own_.clear();losses_.clear();hits_.clear();enemy_losses_.clear();epoch_=map_=geometry_=sequence_=0;
    side_=revision_=0;building_=room_=invalid_room;phase_=DefensePhase::inactive;now_=phase_at_=assessment_at_=last_withdraw_=last_evidence_=resumed_at_=0;cursor_=0;}
RoomDefenseResult RoomDefense::update(const WorldSnapshot& w,const SideSnapshot& side,const RoomDefenseRequest& req,
    const KnowledgeFrame* k,const EngineServices& svc,const std::function<bool()>& stop){
    RoomDefenseResult fail;fail.reason=DefenseReason::invalid_input;
    try{poll(stop);auto staged=*this;poll(stop);auto result=staged.update_impl(w,side,req,k,svc,stop);poll(stop);
        if(result.accepted)*this=std::move(staged);return result;
    }catch(const Cancelled&){fail.cancelled=true;fail.reason=DefenseReason::cancelled;return fail;}
    catch(...){fail.reason=DefenseReason::geometry;return fail;}
}
RoomDefenseResult RoomDefense::update_impl(const WorldSnapshot& w,const SideSnapshot& side,const RoomDefenseRequest& req,
    const KnowledgeFrame* knowledge,const EngineServices& svc,const std::function<bool()>& stop){
    RoomDefenseResult out;out.reason=DefenseReason::invalid_input;const auto& c=config_.room;
    const float values[]={c.pitch_m,c.spacing_m,c.arrival_m,c.projection_m,c.floor_tolerance_m,c.exposure_cost_m,c.unknown_cost_m,
        c.funnel_cost_m,c.approach_weight,c.post_distance_m,c.look_height_m,c.withdraw_unsafe_m,c.withdraw_health_mean,
        c.withdraw_able_fraction,c.withdraw_casualty_rally_m,c.backout_min_m,c.backout_max_m};
    for(float v:values)if(!std::isfinite(v)||v<0)return out;
    if(!w.level_epoch||!w.sequence||!c.spacing_m||!c.arrival_m||!c.alternative_slots||!c.path_nodes||!config_.order_lifetime_ms
        ||!std::isfinite(config_.cover_radius)||config_.cover_radius<=0||c.backout_min_m>c.backout_max_m)return out;
    if(epoch_&&w.level_epoch<epoch_)return out;
    if(epoch_==w.level_epoch&&(w.sequence<=sequence_||w.now_ms<now_))return out;
    if(!svc.buildings||svc.buildings->level_epoch!=w.level_epoch||svc.buildings->map_hash!=w.map_hash
        ||!svc.buildings->locate||!svc.node_position||!svc.path_length){out.reason=DefenseReason::geometry;return out;}
    const Building* building=nullptr;for(const auto& b:svc.buildings->buildings)if(b.id==req.building){building=&b;break;}
    const Room* target=building?room(*building,req.room):nullptr;if(!target){out.reason=DefenseReason::geometry;return out;}
    RoomExposureCost prices(w,side,svc.buildings,req.exposure,stop);if(prices.was_cancelled())throw Cancelled{};
    std::map<std::uint32_t,const ActorSnapshot*> actors;for(const auto& a:w.actors){poll(stop);
        if(!valid_key(a.key)||!actors.emplace(a.key.id,&a).second)return out;}
    auto get=[&](ActorKey key)->const ActorSnapshot*{auto it=actors.find(key.id);return it!=actors.end()&&it->second->key==key?it->second:nullptr;};
    std::set<std::uint32_t> member_ids;for(auto key:side.members){poll(stop);if(!valid_key(key)||!member_ids.insert(key.id).second)return out;}
    auto member=[&](ActorKey key){return std::find(side.members.begin(),side.members.end(),key)!=side.members.end();};
    if(req.restrict_participants){
        if(req.mode==DefenseMode::assessed_withdraw)return out;
        std::set<std::uint32_t> participants;
        for(auto key:req.participants){poll(stop);if(!member(key)||!participants.insert(key.id).second)return out;}
    }
    const auto& participants=req.restrict_participants?req.participants:side.members;
    auto participates=[&](ActorKey key){return std::find(participants.begin(),participants.end(),key)!=participants.end();};
    std::map<std::uint32_t,const Order*> bases;for(const auto& o:req.base_orders){poll(stop);
        if(!valid_key(o.actor)||!bases.emplace(o.actor.id,&o).second)return out;}
    auto base=[&](ActorKey key)->const Order*{auto it=bases.find(key.id);return it!=bases.end()&&it->second->actor==key?it->second:nullptr;};
    const bool reset_all=epoch_!=w.level_epoch||map_!=w.map_hash||geometry_!=w.geometry_revision||side_!=side.side_id;
    if(reset_all){reset();epoch_=w.level_epoch;map_=w.map_hash;geometry_=w.geometry_revision;side_=side.side_id;}
    const bool replan=reset_all||revision_!=req.revision||mode_!=req.mode||building_!=req.building||room_!=req.room;
    if(replan){men_.clear();cursor_=0;phase_=DefensePhase::inactive;assessment_at_=0;assessment_reason_=DefenseReason::none;
        revision_=req.revision;mode_=req.mode;building_=req.building;room_=req.room;}
    sequence_=w.sequence;now_=w.now_ms;out.accepted=true;out.level_epoch=w.level_epoch;out.snapshot_seq=w.sequence;
    out.geometry_revision=w.geometry_revision;out.at_ms=w.now_ms;
    auto transition=[&](DefensePhase p,DefenseReason reason){if(phase_!=p){out.transitions.push_back({phase_,p,reason,w.now_ms});phase_=p;phase_at_=w.now_ms;}
        out.reason=reason;};
    auto cleared=[&](std::uint32_t r){for(const auto& x:req.cleared)if(x.side==side.side_id&&x.building==req.building&&x.room==r&&x.at_ms<=w.now_ms)return true;return false;};
    std::vector<Threat> threats;
    if(knowledge&&knowledge->accepted&&knowledge->level_epoch==w.level_epoch&&knowledge->sequence==w.sequence){
        for(const auto& m:knowledge->members){poll(stop);const auto* owner=get(m.actor);
            if(!member(m.actor)||m.side!=side.side_id||!owner||!current(*owner,w,config_)||m.ownership_generation!=owner->ownership_generation)continue;
            for(const auto& foe:m.foes){poll(stop);auto fact=latest(foe);if(!fact||!foe.id.valid()||fact->at_ms>w.now_ms
                ||!valid_key(fact->reporter)||w.now_ms-fact->at_ms>c.withdraw_foe_ms)continue;
                auto it=std::find_if(threats.begin(),threats.end(),[&](const Threat& t){return t.id==foe.id;});
                Threat t{foe.id,fact->position,fact->at_ms,foe.down||foe.wounded,foe.down?foe.down_ms:0};
                if(it==threats.end())threats.push_back(t);else if(t.at>it->at||t.down_at>it->down_at)*it=t;}}
    }
    auto& assessment=out.assessment;assessment.roster=side.members.size();
    for(auto key:side.members){poll(stop);const auto* a=get(key);if(!a||!current(*a,w,config_)){++assessment.unknown;continue;}
        ++assessment.observed;auto old=std::find_if(own_.begin(),own_.end(),[&](const Own& x){return x.actor==key;});
        if(old!=own_.end()&&old->alive&&(!a->alive||(!old->wounded&&a->wounded))){losses_.push_back({key,w.now_ms});last_evidence_=w.now_ms;}
        if(old==own_.end())own_.push_back({key,a->alive,a->wounded});else{old->alive=a->alive;old->wounded=a->wounded;}
        if(!a->alive)++assessment.dead;else if(a->wounded)++assessment.wounded;
        else{++assessment.able;assessment.mean_health+=(std::max)(0.f,(std::min)(1.f,a->health));}
        if(a->hits.count>a->hits.items.size()||a->visible_enemies.count>a->visible_enemies.items.size())return RoomDefenseResult{};
        for(std::size_t i=0;i<a->hits.count;++i){const auto at=a->hits.items[i].at_ms;if(at>a->observed_at_ms||!finite(a->hits.items[i].dir))continue;
            if(std::none_of(hits_.begin(),hits_.end(),[&](const HitFact& h){return h.actor==key&&h.at_ms==at;}))hits_.push_back({key,at});last_evidence_=(std::max)(last_evidence_,at);}
        auto sight=[&](const Contact& t){if(visible_able(t,*a,w.now_ms,config_)){last_evidence_=(std::max)(last_evidence_,a->observed_at_ms);
                auto old_threat=std::find_if(threats.begin(),threats.end(),[&](const Threat& x){return x.id.actor==t.key&&!x.id.ghost();});
                Threat seen{{t.key,0,0},t.position,a->observed_at_ms,false,0};
                if(old_threat==threats.end())threats.push_back(seen);else if(seen.at>=old_threat->at)*old_threat=seen;}};
        sight(a->enemy);for(std::size_t i=0;i<a->visible_enemies.count;++i)sight(a->visible_enemies.items[i]);
        if(a->last_applied.weapon==2&&fresh(a->last_applied.at_ms,w.now_ms,c.withdraw_break_window_ms))assessment.no_firing_solution=false;
    }
    own_.erase(std::remove_if(own_.begin(),own_.end(),[&](const Own& x){return !member(x.actor);}),own_.end());
    losses_.erase(std::remove_if(losses_.begin(),losses_.end(),[&](const Loss& x){return !member(x.actor)||!fresh(x.at_ms,w.now_ms,c.withdraw_trend_ms);}),losses_.end());
    hits_.erase(std::remove_if(hits_.begin(),hits_.end(),[&](const HitFact& x){return !member(x.actor)||!fresh(x.at_ms,w.now_ms,c.withdraw_trend_ms);}),hits_.end());
    if(knowledge&&knowledge->accepted&&knowledge->level_epoch==w.level_epoch&&knowledge->sequence==w.sequence)
        for(const auto& e:knowledge->events){poll(stop);if(e.kind==KnowledgeEventKind::enemy_down&&e.side==side.side_id
            &&e.foe.valid()&&!e.foe.ghost()&&fresh(e.at_ms,w.now_ms,c.withdraw_trend_ms)
            &&std::none_of(enemy_losses_.begin(),enemy_losses_.end(),[&](const EnemyLoss& x){return x.foe==e.foe&&x.at_ms==e.at_ms;}))enemy_losses_.push_back({e.foe,e.at_ms});}
    enemy_losses_.erase(std::remove_if(enemy_losses_.begin(),enemy_losses_.end(),[&](const EnemyLoss& x){return !fresh(x.at_ms,w.now_ms,c.withdraw_trend_ms);}),enemy_losses_.end());
    assessment.their_losses=enemy_losses_.size();
    assessment.our_losses=losses_.size();for(const auto& h:hits_)if(fresh(h.at_ms,w.now_ms,c.withdraw_break_window_ms))++assessment.hits;
    for(const auto& t:threats)if(!t.down){if(t.id.ghost())++assessment.unseen;else ++assessment.known;}
    if(assessment.able)assessment.mean_health/=static_cast<float>(assessment.able);
    assessment.broken=assessment.observed&&(float(assessment.able)<=float(assessment.observed)*c.withdraw_able_fraction
        ||(assessment.able&&assessment.mean_health<c.withdraw_health_mean));
    assessment.outmatched=assessment.known+assessment.unseen>=(std::max)(std::size_t(1),assessment.able);
    assessment.losing=assessment.our_losses>assessment.their_losses&&(assessment.known||assessment.unseen||!hits_.empty());
    assessment.pressure=assessment.our_losses||assessment.unseen||!hits_.empty();
    auto safe_point=[&](Vec3 point){for(const auto& t:threats)if(!t.down&&fresh(t.at,w.now_ms,c.withdraw_threat_ms)&&flat(t.position,point)<c.withdraw_unsafe_m)return false;return true;};
    auto crumb_valid=[&](const RoomPositionCrumb& x,Vec3& pos,bool fallback_only=true){if(!member(x.actor)||x.at_ms>w.now_ms||x.level_epoch!=w.level_epoch
        ||x.geometry_revision!=w.geometry_revision||x.node==invalid_node||x.building!=req.building
        ||(x.room==req.room?fallback_only:!cleared(x.room)))return false;
        poll(stop);if(!svc.node_position(x.node,&pos)||!finite(pos))return false;const auto loc=svc.buildings->locate(x.node);
        const auto* actual=room(*building,x.room);return actual&&in(*actual,pos,c.floor_tolerance_m)&&loc.first==x.building&&loc.second==x.room;};
    Vec3 rally;bool have_rally=false;TimeMs rally_at=0;
    for(const auto& x:req.crumbs){poll(stop);Vec3 pos;if(crumb_valid(x,pos)&&safe_point(pos)&&(!have_rally||x.at_ms>rally_at)){have_rally=true;rally=pos;rally_at=x.at_ms;}}
    assessment.fallback_safe=have_rally;assessment.reason=have_rally?DefenseReason::none:DefenseReason::no_cleared_trail;
    if(have_rally)for(auto key:side.members){const auto* a=get(key);if(a&&current(*a,w,config_)&&a->alive&&a->wounded
        &&flat(a->position,rally)>c.withdraw_casualty_rally_m){assessment.fallback_safe=false;assessment.reason=DefenseReason::casualty_at_risk;break;}}
    assessment.requested=req.mode==DefenseMode::assessed_withdraw&&((assessment.broken&&(assessment.outmatched||assessment.losing))
        ||(c.withdraw_break_hits&&assessment.hits>=c.withdraw_break_hits&&assessment.no_firing_solution));
    bool committed=false;for(const auto& o:req.base_orders)if((o.flags&order_committed_entry)&&member(o.actor)){
        const auto* a=get(o.actor);if(a&&current(*a,w,config_)&&controllable(*a)&&!in(*target,a->position,c.floor_tolerance_m))committed=true;}
    if(req.mode==DefenseMode::assessed_withdraw&&phase_!=DefensePhase::withdrawing&&phase_!=DefensePhase::regroup){
        DefenseReason verdict=DefenseReason::continue_task;
        if(assessment.requested){verdict=assessment.fallback_safe?DefenseReason::peeling:assessment.reason;
            if(last_withdraw_&&w.now_ms-last_withdraw_<c.withdraw_cooldown_ms)verdict=DefenseReason::cooldown;
            if(committed)verdict=DefenseReason::committed_entry;
            if(resumed_at_&&last_evidence_<=resumed_at_)verdict=DefenseReason::continue_task;}
        if(verdict!=assessment_reason_){assessment_reason_=verdict;assessment_at_=w.now_ms;}
        if(verdict==DefenseReason::continue_task){assessment_at_=w.now_ms;transition(DefensePhase::inactive,verdict);}
        else if(verdict==DefenseReason::committed_entry)transition(DefensePhase::assessing,verdict);
        else if(w.now_ms-assessment_at_>=c.withdraw_persist_ms){
            if(verdict==DefenseReason::peeling){transition(DefensePhase::withdrawing,verdict);last_withdraw_=w.now_ms;men_.clear();}
            else transition(DefensePhase::defending,verdict);
        }else transition(DefensePhase::assessing,DefenseReason::assessment);
    }
    if(req.mode==DefenseMode::hold)transition(DefensePhase::holding,DefenseReason::hold);
    else if(req.mode==DefenseMode::defend)transition(DefensePhase::defending,DefenseReason::defend);
    assessment.allowed=phase_==DefensePhase::withdrawing;
    if(req.mode==DefenseMode::assessed_withdraw&&(phase_==DefensePhase::inactive||phase_==DefensePhase::assessing)){
        out.orders=req.base_orders;out.phase=phase_;return out;}
    std::vector<const Portal*> portals;for(const auto& p:building->portals)if(p.room_a==req.room||p.room_b==req.room)portals.push_back(&p);
    std::stable_sort(portals.begin(),portals.end(),[&](const Portal* a,const Portal* b){auto score=[&](const Portal* p){
        const auto o=other(*p,req.room);const Room* r=room(*building,o);bool known=false;for(const auto& t:threats)if(!t.down&&r&&in(*r,t.position,c.floor_tolerance_m))known=true;
        return known?0:o==invalid_room?1:!cleared(o)?2:3;};return score(a)<score(b);});
    men_.erase(std::remove_if(men_.begin(),men_.end(),[&](const Man& m){const auto* a=get(m.actor);return !participates(m.actor)||!a||!current(*a,w,config_)
        ||!controllable(*a)||reserved(base(m.actor))||a->ownership_generation!=m.owner||a->restrictions_revision!=m.restrictions;}),men_.end());
    const FactionStyle* style=&config_.faction_default;
    if(config_.factions.count>config_.factions.items.size())return RoomDefenseResult{};
    for(std::size_t i=0;i<config_.factions.count;++i)if(config_.factions.items[i].faction==side.faction){style=&config_.factions.items[i];break;}
    const bool local_elements=req.restrict_participants&&req.mode==DefenseMode::defend&&style->doctrine==0;
    std::vector<ActorKey> newcomers;
    for(auto key:participants){poll(stop);const auto* a=get(key);if(a&&current(*a,w,config_)&&controllable(*a)&&!reserved(base(key))&&
        std::none_of(men_.begin(),men_.end(),[&](const Man& m){return m.actor==key;}))newcomers.push_back(key);}
    while(!newcomers.empty()){poll(stop);const auto key=newcomers.front();newcomers.erase(newcomers.begin());const auto* a=get(key);
        if(intent_==(std::numeric_limits<std::uint64_t>::max)())return RoomDefenseResult{};
        Man m;m.actor=key;m.owner=a->ownership_generation;m.restrictions=a->restrictions_revision;m.intent=++intent_;
        if(!portals.empty()&&req.mode!=DefenseMode::hold){const auto rank=men_.size();
            m.reserve=rank>=2*portals.size()||(portals.size()>1&&cleared(other(*portals[(rank/2)%portals.size()],req.room)));
            m.portal=portals[(rank/2)%portals.size()]->id;m.side=rank%2?-1:1;
            if(m.reserve)for(const auto* p:portals)if(cleared(other(*p,req.room))){m.portal=p->id;break;}
            if(local_elements){
                const Man* buddy=nullptr;std::size_t size=0;float nearest=(std::numeric_limits<float>::max)();
                std::set<std::uint64_t> elements;
                for(const auto& existing:men_){poll(stop);if(!existing.element||!elements.insert(existing.element).second)continue;
                    const auto count=std::count_if(men_.begin(),men_.end(),[&](const Man& man){return man.element==existing.element;});
                    const float distance=flat(a->position,get(existing.actor)->position);
                    if(static_cast<std::size_t>(count)<existing.element_size&&distance<nearest){buddy=&existing;size=static_cast<std::size_t>(count);nearest=distance;}
                }
                if(buddy){m.element=buddy->element;m.element_size=buddy->element_size;m.portal=buddy->portal;}
                else{m.element=m.intent;const auto remaining=newcomers.size()+1;
                    m.element_size=remaining==4?2:(std::min)(std::size_t(3),remaining);
                    m.portal=portals[elements.size()%portals.size()]->id;
                }
                bool positive=false,negative=false;
                for(const auto& existing:men_)if(existing.element==m.element&&!existing.reserve){positive|=existing.side>0;negative|=existing.side<0;}
                m.side=!positive?1:!negative?-1:1;m.reserve=positive&&negative;
                // A local pair/trio shares an opening. Select nearby partners;
                // the third man supports those posts instead of a room-centre rally.
                if(size+1<m.element_size)std::stable_sort(newcomers.begin(),newcomers.end(),[&](ActorKey x,ActorKey y){
                    return distance_squared(get(x)->position,a->position)<distance_squared(get(y)->position,a->position);});
            }
        }
        men_.push_back(std::move(m));}
    auto rejected=[&](Man& m,const ActorSnapshot& a){const auto& r=a.last_rejected;
        if(r.node!=invalid_node&&r.restrictions_revision==a.restrictions_revision&&(r.reason==reject_goal_invalid||r.reason==reject_goal_inaccessible)
            &&std::find(m.rejected.begin(),m.rejected.end(),r.node)==m.rejected.end()){m.rejected.push_back(r.node);if(m.node==r.node){m.node=invalid_node;m.initialized=m.complete=false;m.next=0;m.candidates.clear();}}};
    auto conflict=[&](const Man& m,std::uint32_t node,Vec3 point){for(const auto& other_man:men_)if(other_man.actor!=m.actor&&other_man.node!=invalid_node
        &&(other_man.node==node||flat(other_man.goal,point)<c.spacing_m))return true;
        if(req.restrict_participants)for(const auto& o:req.base_orders){poll(stop);
            if(o.actor==m.actor||(participates(o.actor)&&!reserved(&o))||o.goal_node==invalid_node||!finite(o.goal_position))continue;
            if(o.goal_node==node||(std::abs(o.goal_position.y-point.y)<=c.floor_tolerance_m&&flat(o.goal_position,point)<c.spacing_m))return true;
        }return false;};
    auto red_cost=[&](std::uint32_t node,Vec3 pos,RoomExposureCostReceipt& receipt,bool held=false){DefenseCost cost;for(const auto* p:portals){const auto adj=other(*p,req.room);
        if(!cleared(adj)){const auto& tab=p->room_a==req.room?p->lit_a:p->lit_b;
            const auto it=std::lower_bound(tab.begin(),tab.end(),node,[](const Lit& x,std::uint32_t n){return x.node<n;});
            if(it!=tab.end()&&it->node==node)cost.lit+=c.exposure_cost_m*float(it->standing)/255.f;
            else if(!svc.buildings->lit_complete)cost.unknown+=c.unknown_cost_m;}
        Vec3 d=add(pos,p->centre,-1);if(std::abs(dot(d,outward(*p,req.room)))<c.wall_offset_m&&std::abs(dot(d,tangent(*p)))<p->half_width+c.arrival_m)cost.door+=c.funnel_cost_m;}
        receipt=prices.price(RoomExposureRole::defense,req.building,req.room,node,pos,cost.lit+cost.unknown,held);
        if(prices.was_cancelled())throw Cancelled{};
        if(receipt.used){cost.lit=receipt.cost;cost.unknown=0;}return cost;};
    auto seed=[&](Man& m,const ActorSnapshot& a){m.candidates.clear();m.next=m.authored_next=m.native_next=0;m.complete=false;m.initialized=true;
        auto push=[&](std::uint32_t node,Vec3 pos,Evidence cover){if(node==invalid_node||!finite(pos))return;
            if(std::none_of(m.candidates.begin(),m.candidates.end(),[&](const Candidate& x){return x.node==node;}))m.candidates.push_back({node,pos,cover});};
        push(a.node_id,a.position,Evidence::unknown);
        if(svc.cover_query){CoverQueryInput q;q.centre=target->centre;q.radius=config_.cover_radius;q.min_enemy_m=c.known_enemy_exclusion_m;q.max_lo=1;
            q.max_n=config_.cover_candidates;for(const auto& t:threats)if(!t.down&&q.threats.count<q.threats.items.size())q.threats.items[q.threats.count++]=t.position;
            q.heights.items[0]=c.look_height_m;q.heights.count=1;std::array<CoverSpot,256> spots{};
            poll(stop);const auto count=svc.cover_query(q,spots.data(),q.max_n);poll(stop);
            if(count>q.max_n)throw std::runtime_error("cover count exceeds supplied capacity");
            for(std::size_t i=0;i<count;++i)push(spots[i].node,spots[i].position,spots[i].protection_known);}
    };
    if(phase_!=DefensePhase::withdrawing&&phase_!=DefensePhase::regroup){
        // Price only: held claims and their completion/intent survive a new view.
        for(auto& m:men_)if(m.node!=invalid_node&&(req.exposure||m.exposure.used)){
            const auto updated=red_cost(m.node,m.goal,m.exposure,true);
            m.cost.total+=updated.lit+updated.unknown-m.priced_lit-m.priced_unknown;
            m.cost.lit+=updated.lit-m.priced_lit;m.cost.unknown+=updated.unknown-m.priced_unknown;
            m.priced_lit=updated.lit;m.priced_unknown=updated.unknown;
        }
        std::size_t idle=0;while(out.work_used<req.max_work&&!men_.empty()&&idle<men_.size()){
            poll(stop);auto& m=men_[cursor_%men_.size()];cursor_=(cursor_+1)%men_.size();const auto& a=*get(m.actor);rejected(m,a);
            TimeMs recent_hit=0;for(std::size_t h=0;h<a.hits.count;++h)if(finite(a.hits.items[h].dir)&&fresh(a.hits.items[h].at_ms,w.now_ms,c.withdraw_break_window_ms))recent_hit=(std::max)(recent_hit,a.hits.items[h].at_ms);
            if(recent_hit>m.replan_hit){m.replan_hit=recent_hit;m.initialized=m.complete=false;}
            if(m.complete){++idle;continue;}idle=0;++out.work_used;
            if(!m.initialized){seed(m,a);continue;}
            Candidate q;
            if(m.next<m.candidates.size())q=m.candidates[m.next++];
            else if(m.authored_next<c.alternative_slots&&m.portal&&svc.project){
                const auto n=m.authored_next++;const auto* p=door(*building,m.portal);Vec3 pos=p->centre;
                pos=add(pos,tangent(*p),float(m.side)*(p->half_width+c.jamb_offset_m+float(n/2)*c.pitch_m));
                pos=add(pos,outward(*p,req.room),-(c.post_distance_m+float(n%2)*c.pitch_m));pos.y=target->floor_y;
                q.node=svc.project(pos,c.projection_m,c.floor_tolerance_m,&q.position);poll(stop);
                if(q.node==invalid_node||!finite(q.position)||flat(q.position,pos)>c.projection_m||std::abs(q.position.y-pos.y)>c.floor_tolerance_m)continue;
            }else if(m.native_next<target->node_ids.size()){
                q.node=target->node_ids[m.native_next++];if(!svc.node_position(q.node,&q.position)||!finite(q.position))continue;poll(stop);
            }else{m.complete=true;continue;}
            const auto loc=svc.buildings->locate(q.node);
            if(loc.first!=req.building||loc.second!=req.room||!in(*target,q.position,c.floor_tolerance_m)||conflict(m,q.node,q.position)
                ||std::find(m.rejected.begin(),m.rejected.end(),q.node)!=m.rejected.end())continue;
            RoomExposureCostReceipt quote;auto cost=red_cost(q.node,q.position,quote);
            const auto priced_lit=cost.lit,priced_unknown=cost.unknown;cost.protection=q.protection;if(q.protection==Evidence::unknown)cost.unknown+=c.unknown_cost_m;
            const auto* p=door(*building,m.portal);if(p&&float(m.side)*dot(add(q.position,p->centre,-1),tangent(*p))<c.jamb_offset_m)continue;
            poll(stop);const float length=svc.path_length(a.node_id,q.node,c.path_nodes);poll(stop);if(!std::isfinite(length)||length<0)continue;
            cost.path=Evidence::yes;cost.travel=length*c.approach_weight;
            if(p){if(svc.rays){Segment ray{q.position,p->centre};ray.a.y+=c.look_height_m;ray.b.y=target->floor_y+c.look_height_m;float hit=0;
                    poll(stop);svc.rays(&ray,&hit,1);poll(stop);cost.lane=hit==-1?Evidence::yes:std::isfinite(hit)&&hit>=0?Evidence::no:Evidence::unknown;
                    if(cost.lane==Evidence::no)continue;if(cost.lane==Evidence::unknown)cost.unknown+=c.unknown_cost_m;
                }else{cost.lane=Evidence::unknown;cost.unknown+=c.unknown_cost_m;}}
            if(m.reserve){Vec3 rally=target->centre;
                if(m.element){Vec3 sum{};std::size_t count=0;for(const auto& buddy:men_)if(buddy.element==m.element&&buddy.actor!=m.actor){
                    const auto p=buddy.node==invalid_node?get(buddy.actor)->position:buddy.goal;sum.x+=p.x;sum.y+=p.y;sum.z+=p.z;++count;}
                    if(count)rally={sum.x/float(count),sum.y/float(count),sum.z/float(count)};}
                cost.reserve=flat(q.position,rally)*c.approach_weight;}
            if(a.weapon.state==WeaponState::reloading||!a.weapon.ammo_in_mag){
                // Extra cost for a reload crossing a known opening, never a veto.
                for(const auto* opening:portals){const auto normal=outward(*opening,req.room);
                    const float from=dot(add(a.position,opening->centre,-1),normal),to=dot(add(q.position,opening->centre,-1),normal);
                    if(from*to<0){const float t=from/(from-to);const auto cross=add(a.position,add(q.position,a.position,-1),t);
                        if(std::abs(dot(add(cross,opening->centre,-1),tangent(*opening)))<=opening->half_width)cost.reload+=c.funnel_cost_m;}}
            }
            cost.total=cost.travel+cost.lit+cost.door+cost.unknown+cost.reserve+cost.reload;
            if(m.node==invalid_node||cost.total+config_.cover_move_hysteresis_m<m.cost.total){
                if(intent_==(std::numeric_limits<std::uint64_t>::max)())return RoomDefenseResult{};
                m.node=q.node;m.goal=q.position;m.cost=cost;m.intent=++intent_;
                m.exposure=quote;m.priced_lit=priced_lit;m.priced_unknown=priced_unknown;}
        }
    }
    // Covering endpoints do not certify a route. Intersect the ENTIRE local
    // segment with the union of captured current/cleared room volumes.
    auto covered_segment=[&](Vec3 from,Vec3 to){
        std::vector<std::pair<double,double>> intervals;
        for(const auto& r:building->rooms){poll(stop);if(r.id!=req.room&&!cleared(r.id))continue;
            double lo=0,hi=1;auto clip=[&](double a,double b,double lower,double upper){
                const double d=b-a;if(d==0)return a>=lower&&a<=upper;
                double x=(lower-a)/d,y=(upper-a)/d;if(x>y)std::swap(x,y);lo=(std::max)(lo,x);hi=(std::min)(hi,y);return lo<=hi;};
            if(clip(from.x,to.x,r.min.x,r.max.x)&&clip(from.z,to.z,r.min.z,r.max.z)
                &&clip(from.y,to.y,r.floor_y-c.floor_tolerance_m,r.floor_y+c.floor_tolerance_m))intervals.push_back({lo,hi});
        }
        std::sort(intervals.begin(),intervals.end());double through=0;
        for(const auto& x:intervals){if(x.first>through)return false;through=(std::max)(through,x.second);if(through>=1)return true;}
        return false;
    };
    auto local_leg=[&](const ActorSnapshot& a,std::uint32_t node,Vec3 point,float& length){
        if(!covered_segment(a.position,point))return DefenseRoute::uncleared;
        if(!svc.rays)return DefenseRoute::unknown;
        poll(stop);length=svc.path_length(a.node_id,node,c.path_nodes);poll(stop);
        if(!std::isfinite(length)||length<0)return DefenseRoute::blocked;
        const double direct=std::sqrt(double(distance_squared(a.position,point)));
        // Standard forward-error bound for <=path_nodes float additions, not a
        // gameplay detour allowance. A diagonal cardinal-grid path is split
        // into short axis legs below rather than silently widened by epsilon.
        const double neps=double(c.path_nodes)*std::numeric_limits<float>::epsilon();
        const double tolerance=neps<1?direct*neps/(1-neps):0;
        if(std::abs(double(length)-direct)>tolerance)return DefenseRoute::detour;
        Segment segment{a.position,point};segment.a.y+=c.look_height_m;segment.b.y+=c.look_height_m;
        float hit=std::numeric_limits<float>::quiet_NaN();poll(stop);svc.rays(&segment,&hit,1);poll(stop);
        if(hit!=-1)return std::isfinite(hit)&&hit>=0?DefenseRoute::blocked:DefenseRoute::unknown;
        return DefenseRoute::local_witness;
    };
    auto choose_leg=[&](Man& m,const ActorSnapshot& a,std::uint32_t node,Vec3 point){
        if(conflict(m,node,point)||std::find(m.rejected.begin(),m.rejected.end(),node)!=m.rejected.end())return false;
        float length=0;auto status=local_leg(a,node,point,length);m.route=status;
        if(status!=DefenseRoute::local_witness&&status==DefenseRoute::detour&&svc.project){
            for(auto pivot:{Vec3{point.x,a.position.y,a.position.z},Vec3{a.position.x,a.position.y,point.z}}){
                if(flat(a.position,pivot)==0)continue;Vec3 q;poll(stop);auto n=svc.project(pivot,c.projection_m,c.floor_tolerance_m,&q);poll(stop);
                if(n==invalid_node||!finite(q)||flat(a.position,q)>c.backout_max_m||conflict(m,n,q)
                    ||std::find(m.rejected.begin(),m.rejected.end(),n)!=m.rejected.end())continue;
                // The projected pivot has to stay on the intended grid axis.
                if((pivot.x==a.position.x&&q.x!=a.position.x)||(pivot.z==a.position.z&&q.z!=a.position.z))continue;
                status=local_leg(a,n,q,length);if(status==DefenseRoute::local_witness){node=n;point=q;break;}
            }
            m.route=status;
        }
        if(status!=DefenseRoute::local_witness)return false;
        if(m.node!=node){if(intent_==(std::numeric_limits<std::uint64_t>::max)())throw std::runtime_error("intent exhausted");m.intent=++intent_;}
        m.node=node;m.goal=point;m.witness_snapshot=w.sequence;m.witness_at=w.now_ms;m.witness_from=a.node_id;m.witness_crumb=0;
        m.cost={};m.exposure={};m.priced_lit=m.priced_unknown=0;m.cost.travel=length*c.approach_weight;m.cost.path=m.cost.lane=Evidence::yes;
        return true;
    };
    if(phase_==DefensePhase::withdrawing){
        // The nearest doorway man stays as cover while the deepest moves first.
        std::vector<Man*> inside;bool busy=false;for(auto& m:men_){const auto& a=*get(m.actor);
            if(in(*target,a.position,c.floor_tolerance_m)||doorway(*building,a.position,c))inside.push_back(&m);
            if(m.released&&((in(*target,a.position,c.floor_tolerance_m)&&w.now_ms-m.release_ms<c.withdraw_peel_inside_ms)
                ||(doorway(*building,a.position,c)&&w.now_ms-m.release_ms<c.withdraw_peel_door_ms)))busy=true;}
        auto depth=[&](const Man* m){const auto* a=get(m->actor);float d=0;for(const auto* p:portals)d=(std::max)(d,flat(a->position,p->centre));return d;};
        std::stable_sort(inside.begin(),inside.end(),[&](const Man* a,const Man* b){return depth(a)>depth(b);});
        if(!busy)for(auto* m:inside)if(!m->released){m->released=true;m->release_ms=w.now_ms;break;}
        for(std::size_t visited=0;visited<men_.size();++visited){poll(stop);
            if(out.work_used>=req.max_work){out.partial=true;break;}
            auto& m=men_[cursor_%men_.size()];cursor_=(cursor_+1)%men_.size();const auto& a=*get(m.actor);if(!m.released)continue;
            ++out.work_used;rejected(m,a);
            if(m.node!=invalid_node&&m.route==DefenseRoute::local_witness&&flat(a.position,m.goal)>c.arrival_m)continue;
            if(!m.backout_attempted){m.backout_attempted=true;
                const auto personal=first_response(a,config_,w.now_ms,w.level_epoch,w.geometry_revision,0);
                if(personal.look.kind==LookKind::enemy&&svc.project){auto away=unit(add(a.position,personal.look.value,-1));
                    Vec3 desired=add(a.position,away,c.backout_max_m),q;poll(stop);auto n=svc.project(desired,c.projection_m,c.floor_tolerance_m,&q);poll(stop);
                    if(n!=invalid_node&&finite(q)&&flat(a.position,q)>=c.backout_min_m&&flat(a.position,q)<=c.backout_max_m
                        &&safe_point(q)&&choose_leg(m,a,n,q)){m.backout=m.node;m.backout_point=m.goal;continue;}}}
            if(m.node!=invalid_node&&flat(a.position,m.goal)<=c.arrival_m){for(const auto& x:req.crumbs){poll(stop);
                if(x.actor==m.actor&&x.node==m.node&&(!m.trail_before||x.at_ms<m.trail_before))m.trail_before=x.at_ms;}}
            std::vector<const RoomPositionCrumb*> candidates;
            for(const auto& x:req.crumbs){poll(stop);if(x.actor!=m.actor||x.at_ms>=m.release_ms
                ||(m.trail_before&&x.at_ms>=m.trail_before)||x.node==a.node_id)continue;
                Vec3 pos;if(!crumb_valid(x,pos,false)||flat(a.position,pos)>c.backout_max_m
                    ||(x.room!=req.room&&!safe_point(pos)))continue;candidates.push_back(&x);}
            std::stable_sort(candidates.begin(),candidates.end(),[](const RoomPositionCrumb* a,const RoomPositionCrumb* b){return a->at_ms>b->at_ms;});
            if(m.route_from!=a.node_id){m.route_from=a.node_id;m.trail_candidate=0;}
            if(!candidates.empty()){const auto* chosen=candidates[m.trail_candidate++%candidates.size()];Vec3 point;
                poll(stop);if(svc.node_position(chosen->node,&point)&&finite(point)&&choose_leg(m,a,chosen->node,point))m.witness_crumb=chosen->at_ms;poll(stop);}
            else if(m.node==invalid_node)m.route=DefenseRoute::unknown;
        }
        const auto quiet_from=(std::max)(phase_at_,last_evidence_);
        if((inside.empty()&&w.now_ms-quiet_from>=c.withdraw_quiet_ms)||w.now_ms-phase_at_>=c.withdraw_cap_ms){
            const bool cap=w.now_ms-phase_at_>=c.withdraw_cap_ms;transition(DefensePhase::regroup,cap?DefenseReason::capped:DefenseReason::quiet);
            for(auto& m:men_){const auto& a=*get(m.actor);m.node=a.node_id;m.goal=a.position;m.initialized=m.complete=true;m.candidates.clear();m.exposure={};m.priced_lit=m.priced_unknown=0;}}
    }
    for(auto key:participants){poll(stop);const auto* a=get(key);const auto* original=base(key);
        auto found=std::find_if(men_.begin(),men_.end(),[&](const Man& m){return m.actor==key;});
        if(!a||!current(*a,w,config_))continue;
        if(found==men_.end()){
            if(!controllable(*a))out.orders.push_back(first_response(*a,config_,w.now_ms,w.level_epoch,w.geometry_revision,original?original->order_seq:0));
            else if(original)out.orders.push_back(*original);continue;}
        auto& m=*found;auto o=first_response(*a,config_,w.now_ms,w.level_epoch,w.geometry_revision,original?original->order_seq:0);
        if(o.kind==OrderKind::yield){out.orders.push_back(o);continue;}
        const bool valid_base=original&&original->level_epoch==w.level_epoch&&original->geometry_revision==w.geometry_revision
            &&original->ownership_generation==a->ownership_generation&&original->restrictions_revision==a->restrictions_revision
            &&original->issued_at_ms<=w.now_ms&&original->expires_at_ms>w.now_ms&&original->goal_node!=invalid_node&&finite(original->goal_position);
        const auto base_room=valid_base?svc.buildings->locate(original->goal_node):std::make_pair(invalid_room,invalid_room);
        const bool safe_base=valid_base&&(phase_!=DefensePhase::withdrawing||(base_room.first==req.building
            &&(base_room.second==req.room||cleared(base_room.second))));
        if(m.node==invalid_node&&safe_base){auto retained=*original;retained.look=o.look;retained.fire=o.fire;
            retained.fire.until_ms=(std::min)(retained.fire.until_ms,retained.expires_at_ms);out.orders.push_back(retained);
            out.assignments.push_back({m.actor,m.portal,m.node,m.side,m.intent,m.cost,true,m.released,false,false,m.reserve,m.route});out.partial=true;continue;}
        if(valid_base)o.stance=original->stance;
        if(original)o.assignment=original->assignment;o.assignment.portal=m.portal;o.assignment.sector=0;
        o.kind=phase_==DefensePhase::withdrawing&&m.released?OrderKind::withdraw:OrderKind::hold_door;
        o.role=o.kind==OrderKind::withdraw?roles::retreat_cover:m.portal?roles::barricade_guard:roles::hold;
        o.goal_node=m.node==invalid_node?a->node_id:m.node;o.goal_position=m.node==invalid_node?a->position:m.goal;
        o.pace=flat(a->position,o.goal_position)>c.arrival_m?Pace::walk:Pace::stand;o.arrival_radius_m=c.arrival_m;
        o.flags&=~order_first_response;o.flags|=order_face_movement;o.side=m.side;
        const bool backing=phase_==DefensePhase::withdrawing&&m.backout!=invalid_node&&m.node==m.backout
            &&w.now_ms-m.release_ms<c.backout_cap_ms;
        if(phase_==DefensePhase::withdrawing&&m.released){o.pace=flat(a->position,o.goal_position)>c.arrival_m?(backing?Pace::walk:Pace::run):Pace::stand;
            if(backing)o.flags&=~order_face_movement;else if(o.look.kind!=LookKind::enemy)o.look.kind=LookKind::path;}
        decltype(Order{}.look) fallback{};bool fallback_valid=false;
        if(!backing){
            if(phase_==DefensePhase::withdrawing&&m.released){fallback.kind=LookKind::path;fallback_valid=true;}
            else if(const auto* p=door(*building,m.portal)){fallback.kind=LookKind::point;fallback.value=p->centre;
                fallback.value.y=target->floor_y+c.look_height_m;fallback_valid=true;}
        }
        if(o.look.kind!=LookKind::enemy&&fallback_valid)o.look=fallback;
        if(o.look.kind!=LookKind::enemy&&!a->reloading&&a->weapon.ammo_in_mag&&a->weapon.state!=WeaponState::reloading
            &&knowledge&&knowledge->accepted&&knowledge->level_epoch==w.level_epoch&&knowledge->sequence==w.sequence
            &&svc.rays&&out.work_used<req.max_work&&m.portal&&phase_!=DefensePhase::withdrawing){
            const auto* opening=door(*building,m.portal);const SourceFact* seen=nullptr;ActorKey seen_target;
            for(const auto& km:knowledge->members){poll(stop);if(km.actor!=key||km.side!=side.side_id||km.ownership_generation!=a->ownership_generation)continue;
                for(const auto& foe:km.foes){poll(stop);if(foe.down||foe.wounded)continue;
                    for(auto source:{std::size_t(0),std::size_t(2)}){const auto& fact=foe.sources[source];
                        if(!fact.valid||!valid_key(fact.reporter)||!finite(fact.position)||!fresh(fact.at_ms,w.now_ms,c.door_hot_ms))continue;
                        const auto delta=add(fact.position,opening->centre,-1);
                        if(std::abs(dot(delta,outward(*opening,req.room)))<=c.entry_zone_depth_m
                            &&std::abs(dot(delta,tangent(*opening)))<=opening->half_width+c.entry_zone_half_m
                            &&(!seen||fact.at_ms>seen->at_ms)){seen=&fact;seen_target=foe.id.actor;}}}}
            if(seen){
                o.look.kind=LookKind::point;o.look.target={};o.look.value=seen->position;
                // The source mailbox authorizes point fire only from THIS
                // shooter's captured contact, never merely a shared report.
                const Contact* proof=nullptr;auto match=[&](const Contact& contact){
                    if(contact.key==seen_target&&valid_key(contact.key)&&contact.hostile&&contact.alive_when_observed
                        &&(!contact.wounded||contact.finish_allowed)&&finite(contact.position)&&contact.observed_at_ms<=a->observed_at_ms
                        &&fresh(contact.observed_at_ms,w.now_ms,config_.memory_max_age_ms)
                        &&contact.position.x==seen->position.x&&contact.position.y==seen->position.y&&contact.position.z==seen->position.z)proof=&contact;};
                match(a->enemy);for(std::size_t i=0;i<a->visible_enemies.count&&!proof;++i)match(a->visible_enemies.items[i]);
                if(proof){++out.work_used;Segment shot{a->position,proof->position};shot.a.y+=c.look_height_m;
                    float hit=0;poll(stop);svc.rays(&shot,&hit,1);poll(stop);
                    if(hit==-1){o.look.target=proof->key;o.look.value=proof->position;
                    o.fire.allowed=true;o.fire.at_valid=true;o.fire.at=proof->position;
                    o.fire.until_ms=(std::min)(fire_window_until(config_,w.now_ms,o.expires_at_ms),saturated_after(seen->at_ms,c.door_hot_ms));
                    o.fire.allowed=o.fire.until_ms>w.now_ms;o.fire.queue=static_cast<std::uint8_t>((std::min)(std::uint16_t(255),c.suppression_queue));
                    o.fire.interval_ms=static_cast<std::uint16_t>((std::min)(TimeMs(65535),c.suppression_interval_ms));o.fire.ammo_floor=c.suppression_ammo_floor;}}}
        }
        out.orders.push_back(o);out.assignments.push_back({m.actor,m.portal,m.node,m.side,m.intent,m.cost,!m.complete,m.released,backing,
            phase_==DefensePhase::withdrawing&&m.route==DefenseRoute::local_witness&&m.witness_crumb!=0,m.reserve,m.route,
            m.witness_snapshot,m.witness_at,m.witness_crumb,m.witness_from,fallback_valid,fallback,m.exposure});out.partial|=!m.complete;
    }
    for(auto& assignment:out.assignments){assignment.room=req.room;
        const auto man=std::find_if(men_.begin(),men_.end(),[&](const Man& m){return m.actor==assignment.actor;});
        if(man!=men_.end())assignment.element=man->element;
    }
    if((phase_==DefensePhase::defending||phase_==DefensePhase::regroup)&&req.mode==DefenseMode::assessed_withdraw){
        const auto last=(std::max)(phase_at_,last_evidence_);const auto quiet=phase_==DefensePhase::regroup&&assessment.broken?c.withdraw_resume_quiet_ms:c.defend_quiet_ms;
        if(w.now_ms-last>=quiet){out.resume=true;resumed_at_=w.now_ms;transition(DefensePhase::inactive,DefenseReason::resume);}}
    if(out.reason==DefenseReason::invalid_input)out.reason=out.partial?DefenseReason::placing:phase_==DefensePhase::withdrawing?DefenseReason::peeling:DefenseReason::defend;
    out.exposure=prices.stats();out.phase=phase_;return out;
}
}}}
