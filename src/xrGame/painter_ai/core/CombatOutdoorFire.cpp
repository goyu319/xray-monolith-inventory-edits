#include "CombatOutdoorFire.h"
#include "OutdoorLocalNavigation.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <limits>
#include <set>
#include <utility>

namespace painter { namespace combat { namespace core {
namespace {
using Key=std::pair<std::uint32_t,std::uint64_t>;
Key key(ActorKey a){return {a.id,a.generation};}
struct Cancelled{};struct Budget{};
void poll(const std::function<bool()>&f){if(f&&f())throw Cancelled{};}
bool eq(Vec3 a,Vec3 b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
float flat2(Vec3 a,Vec3 b){const float x=a.x-b.x,z=a.z-b.z;return x*x+z*z;}
Vec3 up(Vec3 p,float y){p.y+=y;return p;}
bool source_ok(const OutdoorFireSource&q,const ActorSnapshot&a,const WorldSnapshot&w){return q.actor==a.key&&q.level_epoch==w.level_epoch&&
    q.snapshot_seq==w.sequence&&q.geometry_revision==w.geometry_revision&&q.owner==a.ownership_generation&&q.restrictions==a.restrictions_revision&&
    q.at_ms==a.observed_at_ms&&q.at_ms<=w.now_ms&&q.origin_node==a.node_id&&finite(q.position)&&eq(q.position,a.position);}
bool policy_ok(const OutdoorFirePolicy&p){
    if(!p.seen_ms||!p.shot_ms||!p.cache_ms||!p.burst_ms||p.cycle_ms<p.burst_ms||!p.buddy_fire_ms||!p.buddy_cooldown_ms||!p.minimum_rounds||!p.buddy_meter_limit)return false;
    const float values[]={p.ammo_floor,p.buddy_range_m,p.buddy_step_m,p.buddy_walk_scale,p.buddy_mag_fraction,p.near_known_m,p.far_known_m,p.known_half_angle_deg,p.teammate_corridor_m,p.target_sample_height_m};
    for(float v:values)if(!std::isfinite(v)||v<=0)return false;
    if(p.ammo_floor>1||p.buddy_mag_fraction>1||p.buddy_walk_scale<.3f||p.buddy_walk_scale>1.5f||p.known_half_angle_deg>180||p.far_known_m<p.near_known_m)return false;
    float previous=2;for(float t:p.suppression_fractions){if(!std::isfinite(t)||t<=0||t>1||t>=previous)return false;previous=t;}
    return p.suppression_fractions[0]==1;
}
bool eligible(const ActorSnapshot&a,const WorldSnapshot&w,const Config&c){return valid_key(a.key)&&a.alive&&!a.wounded&&a.health>0&&std::isfinite(a.health)&&
    !a.yield_reasons&&(a.flags&flag_admitted)&&!(a.flags&(flag_story|flag_companion|flag_scheme_block|flag_script_combat|flag_help_wounded|flag_wounded_scheme))&&
    a.ownership_generation&&a.current_position_accessible&&a.node_id!=invalid_node&&finite(a.position)&&a.level_epoch==w.level_epoch&&a.snapshot_seq==w.sequence&&
    fresh(a.observed_at_ms,w.now_ms,c.snapshot_max_age_ms)&&a.visible_enemies.count<=a.visible_enemies.items.size();}
bool weapon_ok(const ActorSnapshot&a,const Order&o,const OutdoorFirePolicy&p){return !(a.flags&flag_dont_shoot)&&!a.reloading&&a.weapon.state!=WeaponState::reloading&&
    (a.weapon.state==WeaponState::idle||a.weapon.state==WeaponState::firing)&&a.weapon.kind!=WeaponKind::none&&a.weapon.kind!=WeaponKind::melee&&a.weapon.kind!=WeaponKind::launcher&&
    a.weapon.ammo_in_mag>=p.minimum_rounds&&a.weapon.mag_size&&float(a.weapon.ammo_in_mag)/a.weapon.mag_size>=p.ammo_floor&&
    o.pace!=Pace::run&&!(o.flags&order_sprint)&&o.kind!=OrderKind::reload&&!o.throw_order.valid&&!valid_key(o.aid);}
const Contact* own_contact(const ActorSnapshot&a,ActorKey target,Vec3 point,const Config&c,TimeMs now,bool visible=false){
    auto valid=[&](const Contact&t){return t.key==target&&valid_key(t.key)&&t.hostile&&t.alive_when_observed&&(!t.wounded||t.finish_allowed)&&
        finite(t.position)&&eq(t.position,point)&&fresh(t.observed_at_ms,a.observed_at_ms,c.memory_max_age_ms)&&fresh(t.observed_at_ms,now,c.memory_max_age_ms)&&
        (!visible||personally_visible(t,a));};
    if(valid(a.enemy))return &a.enemy;for(std::size_t i=0;i<a.visible_enemies.count;++i)if(valid(a.visible_enemies.items[i]))return &a.visible_enemies.items[i];return nullptr;
}
bool any_visible(const ActorSnapshot&a){auto yes=[&](const Contact&t){return t.hostile&&t.alive_when_observed&&finite(t.position)&&personally_visible(t,a);};
    if(yes(a.enemy))return true;for(std::size_t i=0;i<a.visible_enemies.count;++i)if(yes(a.visible_enemies.items[i]))return true;return false;}
bool protected_role(const Order&o){if(o.flags&order_committed_entry||o.pie.valid||o.portal.valid||o.assignment.portal||valid_key(o.aid)||o.throw_order.valid)return true;
    switch(o.kind){case OrderKind::entry:case OrderKind::stack:case OrderKind::peek:case OrderKind::hold_door:case OrderKind::bound:case OrderKind::withdraw:return true;default:break;}
    switch(o.role){case roles::casualty_guard:case roles::aid_casualty:case roles::care_security:case roles::rear_security:case roles::door_fight:
    case roles::threat_overwatch:case roles::portal_watch:case roles::door_post:case roles::door_post_ready:case roles::door_slice:case roles::set:
    case roles::search_sector:case roles::two_up:return true;default:return false;}}
bool order_ok(const Order&o,const ActorSnapshot&a,const WorldSnapshot&w){return o.actor==a.key&&o.order_seq&&o.level_epoch==w.level_epoch&&o.snapshot_seq==w.sequence&&
    o.geometry_revision==w.geometry_revision&&o.ownership_generation==a.ownership_generation&&o.restrictions_revision==a.restrictions_revision&&
    o.issued_at_ms==w.now_ms&&o.expires_at_ms>w.now_ms&&o.goal_node!=invalid_node&&finite(o.goal_position)&&o.kind!=OrderKind::yield;}
bool applied(const Order&o,const ActorSnapshot&a,const WorldSnapshot&w){return o.actor==a.key&&o.order_seq&&a.last_applied.order_seq==o.order_seq&&a.accessible_current_order&&
    o.level_epoch==w.level_epoch&&o.geometry_revision==w.geometry_revision&&o.ownership_generation==a.ownership_generation&&o.restrictions_revision==a.restrictions_revision&&
    o.snapshot_seq&&o.snapshot_seq<=w.sequence&&o.issued_at_ms<=a.last_applied.at_ms&&a.last_applied.at_ms<=a.observed_at_ms&&o.expires_at_ms>w.now_ms&&
    !(a.last_rejected.order_seq==o.order_seq&&a.last_rejected.reason!=reject_none);}
// The dispatcher assigns its final sequence AFTER decoration. Evidence must
// therefore match the actual accepted/applied intent by its original source,
// not the provisional sequence. Publication may only tighten these windows.
bool publication_matches(const Order&o,const Order&proposal){return proposal.order_seq&&o.actor==proposal.actor&&
    o.level_epoch==proposal.level_epoch&&o.snapshot_seq==proposal.snapshot_seq&&o.geometry_revision==proposal.geometry_revision&&
    o.ownership_generation==proposal.ownership_generation&&o.restrictions_revision==proposal.restrictions_revision&&
    o.issued_at_ms==proposal.issued_at_ms&&o.expires_at_ms<=proposal.expires_at_ms&&o.kind==proposal.kind&&o.role==proposal.role&&
    o.goal_node==proposal.goal_node&&eq(o.goal_position,proposal.goal_position)&&o.pace==proposal.pace&&o.stance==proposal.stance&&
    o.assignment.generation==proposal.assignment.generation&&o.look.kind==proposal.look.kind&&o.look.target==proposal.look.target&&eq(o.look.value,proposal.look.value)&&
    o.fire.allowed&&o.fire.at_valid&&proposal.fire.allowed&&proposal.fire.at_valid&&eq(o.fire.at,proposal.fire.at)&&
    o.fire.until_ms&&o.fire.until_ms<=proposal.fire.until_ms&&o.fire.queue&&o.fire.queue<=proposal.fire.queue&&
    o.fire.interval_ms>=proposal.fire.interval_ms&&std::isfinite(o.fire.ammo_floor)&&o.fire.ammo_floor>=proposal.fire.ammo_floor&&o.fire.ammo_floor<=1;}
const SourceFact* known(const FoeKnowledge&f,TimeMs now){const SourceFact* best=nullptr;
    // Prefer one's own source as the reference does, without replacing its time.
    for(unsigned group=0;group<2;++group){for(unsigned i=group*2;i<group*2+2;++i){const auto&s=f.sources[i];if(s.valid&&s.at_ms&&s.at_ms<=now&&finite(s.position)&&(!best||s.at_ms>best->at_ms))best=&s;}if(best)break;}return best;}
TimeMs window(const FoeKnowledge&f,TimeMs now,const OutdoorFirePolicy&p){if(f.down||f.wounded||f.visible)return 0;TimeMs until=0;
    for(unsigned index:{0u,2u}){const auto&s=f.sources[index];if(s.valid&&s.at_ms&&s.at_ms<=now)
        until=(std::max)(until,saturated_after(s.at_ms,p.seen_ms));}
    if(f.shot_me.set&&f.shot_me.at_ms&&f.shot_me.at_ms<=now)until=(std::max)(until,saturated_after(f.shot_me.at_ms,p.shot_ms));
    if(!f.id.ghost()&&f.shot_at_me.set&&f.shot_at_me.at_ms&&f.shot_at_me.at_ms<=now)until=(std::max)(until,saturated_after(f.shot_at_me.at_ms,p.shot_ms));
    return until>now?until:0;}
bool friendly(const SideSnapshot&s,ActorKey k){return std::find(s.members.begin(),s.members.end(),k)!=s.members.end();}
}
struct CombatOutdoorFire::Impl {
    struct Line {bool valid=false;FoeId foe;TimeMs source_at=0,checked_at=0;Vec3 from,to;Evidence clear=Evidence::unknown;};
    struct Member {std::uint64_t owner=0,restrictions=0;std::uint16_t weapon=invalid_id;FoeId last;
        std::array<Line,4> lines{};std::size_t line_cursor=0;TimeMs cycle_at=0,last_answer=0;
        ActorKey buddy;FoeId buddy_foe;Vec3 buddy_point;TimeMs hit_at=0;std::uint16_t start_ammo=0;
        Order last_fire,last_step;FoeId fired_foe;std::uint64_t fired_capture=0;};
    Config config;OutdoorFirePolicy policy;std::map<Key,Member> members;
    std::uint64_t epoch=0,map=0,geometry=0,group=0,sequence=0;std::uint32_t side=0,faction=0,mission=0;TimeMs now=0;std::size_t cursor=0;
    Impl(Config c,OutdoorFirePolicy p):config(c),policy(p){}
};
CombatOutdoorFire::CombatOutdoorFire(Config c,OutdoorFirePolicy p):impl_(new Impl(c,p)){}
CombatOutdoorFire::~CombatOutdoorFire()=default;CombatOutdoorFire::CombatOutdoorFire(CombatOutdoorFire&&) noexcept=default;
CombatOutdoorFire& CombatOutdoorFire::operator=(CombatOutdoorFire&&) noexcept=default;
void CombatOutdoorFire::reset(){impl_.reset(new Impl(impl_->config,impl_->policy));}
std::unique_ptr<CombatOutdoorFire> CombatOutdoorFire::clone(const std::function<bool()>&stop)const{
    try{poll(stop);if(!impl_)return nullptr;auto result=std::make_unique<CombatOutdoorFire>(impl_->config,impl_->policy);auto&d=*result->impl_;
        d.epoch=impl_->epoch;d.map=impl_->map;d.geometry=impl_->geometry;d.group=impl_->group;d.sequence=impl_->sequence;
        d.side=impl_->side;d.faction=impl_->faction;d.mission=impl_->mission;d.now=impl_->now;d.cursor=impl_->cursor;
        for(const auto&entry:impl_->members){poll(stop);d.members.emplace(entry);}poll(stop);return result;
    }catch(...){return nullptr;}
}
OutdoorFireResult CombatOutdoorFire::update(const WorldSnapshot&w,const SideSnapshot&s,const KnowledgeFrame&k,
    const OutdoorFireRequest&r,const EngineServices&svc,OutdoorFireQuota&quota,const std::function<bool()>&stop){
    OutdoorFireResult out;const auto work0=quota.work_used,queries0=quota.queries_used;
    auto use=[&](){poll(stop);if(!quota.work_left)throw Budget{};--quota.work_left;++quota.work_used;};
    try{
        poll(stop);const auto& c=impl_->config;const auto&p=impl_->policy;
        if(!policy_ok(p)||!w.level_epoch||!w.sequence||!w.geometry_revision||!w.now_ms||!s.side_id||s.members.empty()||!k.accepted||k.cancelled||
            k.rejection!=KnowledgeReject::none||k.level_epoch!=w.level_epoch||k.sequence!=w.sequence)return out;
        if(!c.room.suppression_queue||c.room.suppression_queue>255||!c.room.suppression_interval_ms||
            c.room.suppression_interval_ms>(std::numeric_limits<std::uint16_t>::max)())return out;
        if(impl_->epoch==w.level_epoch&&(w.sequence<=impl_->sequence||w.now_ms<impl_->now))return out;
        auto n=std::make_unique<Impl>(c,p);
        const bool continuity=impl_->epoch==w.level_epoch&&impl_->map==w.map_hash&&impl_->geometry==w.geometry_revision&&impl_->side==s.side_id&&
            impl_->group==s.group&&impl_->faction==s.faction&&impl_->mission==s.mission.revision;
        if(continuity){n->cursor=impl_->cursor;for(const auto&v:impl_->members){poll(stop);n->members.emplace(v);}}
        n->epoch=w.level_epoch;n->map=w.map_hash;n->geometry=w.geometry_revision;n->side=s.side_id;n->faction=s.faction;n->group=s.group;
        n->mission=s.mission.revision;n->sequence=w.sequence;n->now=w.now_ms;
        std::map<Key,const ActorSnapshot*> actors;std::map<Key,const MemberKnowledge*> knowledge;std::map<Key,const Order*> bases,accepted;
        std::map<Key,const OutdoorFireEye*> eyes;std::set<Key> roster;std::set<std::uint32_t> actor_ids;
        for(auto id:s.members){poll(stop);if(!valid_key(id)||!roster.insert(key(id)).second)return out;}
        for(const auto&a:w.actors){poll(stop);if(!valid_key(a.key)||!actor_ids.insert(a.key.id).second)return out;actors.emplace(key(a.key),&a);}
        for(const auto&m:k.members){poll(stop);if(m.side==s.side_id&&!knowledge.emplace(key(m.actor),&m).second)return out;}
        for(const auto&o:r.base_orders){poll(stop);if(roster.count(key(o.actor))&&!bases.emplace(key(o.actor),&o).second)return out;}
        for(const auto&o:r.applied_orders){poll(stop);auto ai=actors.find(key(o.actor));if(ai!=actors.end()&&ai->second->last_applied.order_seq==o.order_seq&&
            !accepted.emplace(key(o.actor),&o).second)return out;}
        for(const auto&e:r.eyes){poll(stop);if(!eyes.emplace(key(e.source.actor),&e).second)return out;}
        for(auto i=n->members.begin();i!=n->members.end();){poll(stop);if(!roster.count(i->first))i=n->members.erase(i);else ++i;}
        out.epoch=w.level_epoch;out.sequence=w.sequence;out.side=s.side_id;
        // Start with the caller's exact orders. Pending work never replaces a
        // valid continuing goal with a newly invented hold.
        std::map<Key,std::size_t> slots;
        for(auto id:s.members){poll(stop);auto bi=bases.find(key(id));if(bi!=bases.end()){slots.emplace(key(id),out.proposals.size());OutdoorFireProposal v;v.order=*bi->second;out.proposals.push_back(v);}}
        auto cast=[&](Vec3 from,Vec3 to){use();if(!svc.rays||!quota.queries_left)return Evidence::unknown;
            --quota.queries_left;++quota.queries_used;Segment line{from,to};float distance=-2;svc.rays(&line,&distance,1);poll(stop);
            if(!std::isfinite(distance))return Evidence::unknown;
            if(distance==-1||distance>=std::sqrt(distance_squared(from,to)))return Evidence::yes;
            return distance>=0?Evidence::no:Evidence::unknown;};
        bool pending=false;
        const auto start_cursor=n->cursor;
        for(std::size_t turn=0;turn<s.members.size();++turn){
            const std::size_t slot=(start_cursor+turn)%s.members.size();const auto id=s.members[slot];auto sk=slots.find(key(id));
            if(sk==slots.end())continue;auto&result=out.proposals[sk->second];auto ai=actors.find(key(id));auto ki=knowledge.find(key(id));
            if(ai==actors.end()||ki==knowledge.end())continue;const auto&a=*ai->second;const auto&m=*ki->second;
            if(!eligible(a,w,c)||a.faction!=s.faction||m.ownership_generation!=a.ownership_generation||!m.alive||!m.able||!order_ok(result.order,a,w))continue;
            auto&state=n->members[key(id)];if(state.owner!=a.ownership_generation||state.restrictions!=a.restrictions_revision||state.weapon!=a.weapon.id){state={};state.owner=a.ownership_generation;state.restrictions=a.restrictions_revision;state.weapon=a.weapon.id;}
            const auto base=result.order;
            auto confirmed=accepted.find(key(id));if(confirmed!=accepted.end()&&applied(*confirmed->second,a,w)&&a.last_applied.fired_since_previous&&a.last_applied.weapon==2&&
                publication_matches(*confirmed->second,state.last_fire)&&state.fired_capture!=w.sequence&&state.fired_foe.valid()&&
                confirmed->second->fire.allowed&&confirmed->second->fire.at_valid&&confirmed->second->fire.until_ms>a.observed_at_ms&&
                eq(confirmed->second->fire.at,state.last_fire.fire.at)){
                out.executed_suppression.push_back({a.key,state.fired_foe,a.observed_at_ms,a.ownership_generation,w.level_epoch});state.fired_capture=w.sequence;}
            if(!weapon_ok(a,base,p)){result.reason=OutdoorFireReason::weapon;
                result.order.fire.allowed=false;result.order.fire.until_ms=0;result.changed=base.fire.allowed;continue;}
            if(any_visible(a)){result.reason=OutdoorFireReason::own_fight;continue;}
            if(base.fire.allowed&&base.fire.until_ms>w.now_ms&&base.fire.at_valid&&own_contact(a,base.look.target,base.fire.at,c,w.now_ms))continue;
            const auto before_member=state;
            try{
                use();const OutdoorFireEye* eye=nullptr;auto ei=eyes.find(key(id));if(ei!=eyes.end()&&source_ok(ei->second->source,a,w)&&finite(ei->second->measured_eye)&&!eq(ei->second->measured_eye,a.position))eye=ei->second;
                auto safe_line=[&](Vec3 from,Vec3 to){for(auto friend_key:s.members){use();if(friend_key==a.key)continue;auto f=actors.find(key(friend_key));
                    if(f==actors.end()||!f->second->alive||!finite(f->second->position))continue;
                    auto obstructs=[&](Vec3 v){const float dx=to.x-from.x,dz=to.z-from.z,len=dx*dx+dz*dz;if(len<=0)return true;
                        const float t=((v.x-from.x)*dx+(v.z-from.z)*dz)/len;if(t<=0||t>=1)return false;
                        const float x=from.x+t*dx,z=from.z+t*dz;return (v.x-x)*(v.x-x)+(v.z-z)*(v.z-z)<p.teammate_corridor_m*p.teammate_corridor_m;};
                    if(obstructs(f->second->position))return false;auto bo=bases.find(key(friend_key));if(bo!=bases.end()&&obstructs(bo->second->goal_position))return false;}return true;};
                TimeMs line_at=0;
                auto query=[&](FoeId foe,TimeMs source_at,Vec3 from,Vec3 to){
                    for(const auto&l:state.lines){use();if(l.valid&&l.foe==foe&&l.source_at==source_at&&eq(l.from,from)&&eq(l.to,to)&&fresh(l.checked_at,w.now_ms,p.cache_ms)){
                        ++out.cache_hits;line_at=l.checked_at;return l.clear;}}
                    const auto clear=cast(from,to);line_at=w.now_ms;if(clear!=Evidence::unknown){auto&l=state.lines[state.line_cursor++%state.lines.size()];l={true,foe,source_at,w.now_ms,from,to,clear};}
                    else ++out.unknown_lines;return clear;};
                auto orient=[&](Vec3 point,FoeId foe){result.order.look.kind=LookKind::point;result.order.look.value=point;result.order.look.target=foe.ghost()?ActorKey{}:foe.actor;
                    result.order.flags&=~order_face_movement;result.order.fire.allowed=false;result.order.fire.at_valid=false;result.order.fire.until_ms=0;
                    result.changed=true;};
                auto fire=[&](Vec3 point,FoeId foe,TimeMs until,float floor,std::uint8_t queue,
                    PointFireSource source,ActorKey reporter,TimeMs fact_at){
                    if(foe.ghost()||friendly(s,foe.actor)||!valid_key(reporter)||!fact_at||fact_at>a.observed_at_ms||
                       !fresh(fact_at,w.now_ms,c.memory_max_age_ms)){result.reason=OutdoorFireReason::point_authority_missing;return false;}
                    const bool needs_proof=!own_contact(a,foe.actor,point,c,w.now_ms);
                    if(needs_proof){
                        // The old visibility sample is above the target foot.
                        // Permission must instead certify the EXACT aimed point.
                        result.ray_to=point;result.line=query(foe,fact_at,result.ray_from,point);
                        if(result.line!=Evidence::yes){result.reason=result.line==Evidence::unknown?OutdoorFireReason::unknown_line:OutdoorFireReason::no_line;return false;}
                        until=(std::min)(until,saturated_after(line_at,p.cache_ms));
                    }
                    if(!safe_line(a.position,point)){result.reason=OutdoorFireReason::teammate_line;return false;}
                    const auto end=(std::min)({until,result.order.expires_at_ms,fire_window_until(c,a.observed_at_ms,result.order.expires_at_ms)});
                    if(end<=w.now_ms||!queue)return false;result.order.fire.allowed=result.order.fire.at_valid=true;result.order.fire.at=point;result.order.fire.until_ms=end;
                    result.order.fire.ammo_floor=floor;result.order.fire.queue=queue;result.order.fire.interval_ms=std::uint16_t(c.room.suppression_interval_ms);
                    if(needs_proof){
                        result.order.look.value=point;
                        auto& proof=result.point_fire;proof.source=source;proof.authored=result.order;proof.reporter=reporter;
                        proof.known=result.known_position;proof.origin=a.position;proof.origin_node=a.node_id;
                        proof.eye=result.ray_from;proof.ray_end=point;proof.captured_at=a.observed_at_ms;
                        proof.fact_at=fact_at;proof.line_at=line_at;proof.until=end;proof.weapon=a.weapon.id;
                        if(!point_fire_authorized(proof,result.order,a,w.now_ms,c.snapshot_max_age_ms,c.memory_max_age_ms)){
                            proof={};result.order.fire.allowed=false;result.order.fire.until_ms=0;result.reason=OutdoorFireReason::point_authority_missing;return false;
                        }
                    }
                    result.point_authorized=true;result.window_until_ms=end;state.last_fire=result.order;state.fired_foe=foe;return true;};
                const BuddyFact* buddy=nullptr;
                if(!m.best_visible.valid()&&(!m.has_hit||!fresh(m.last_hit_ms,w.now_ms,c.room.buddy_answer_ms))&&m.meter_level<p.buddy_meter_limit){
                    for(const auto&b:m.board){use();if(b.kind!=ReportKind::under_fire||b.member==a.key||!friendly(s,b.member)||!b.at_ms||b.at_ms>a.observed_at_ms||
                        !fresh(b.at_ms,w.now_ms,c.room.buddy_answer_ms)||!finite(b.position))continue;
                        auto ba=actors.find(key(b.member));if(ba==actors.end()||ba->second->faction!=s.faction||!finite(ba->second->position)||
                            ba->second->level_epoch!=w.level_epoch||ba->second->snapshot_seq!=w.sequence||!fresh(ba->second->observed_at_ms,w.now_ms,c.snapshot_max_age_ms)||
                            flat2(ba->second->position,a.position)>p.buddy_range_m*p.buddy_range_m)continue;
                        if(!buddy||b.at_ms>buddy->at_ms||(b.at_ms==buddy->at_ms&&key(b.member)<key(buddy->member)))buddy=&b;}}
                bool buddy_handled=false;
                if(buddy){const bool same=state.buddy==buddy->member&&state.hit_at==buddy->at_ms&&state.buddy_foe==buddy->enemy&&eq(state.buddy_point,buddy->position);
                    if(!same&&state.last_answer&&fresh(state.last_answer,w.now_ms,p.buddy_cooldown_ms-1)){result.reason=OutdoorFireReason::cooldown;}
                    else{
                        result.target=buddy->enemy;result.buddy=buddy->member;result.hit_at_ms=result.source_at_ms=buddy->at_ms;
                        result.known_position=result.selected_point=buddy->position;orient(buddy->position,buddy->enemy);result.reason=OutdoorFireReason::buddy_watch;
                        const bool protected_order=protected_role(base);if(protected_order)result.reason=OutdoorFireReason::protected_role;
                        bool continuing_step=false;
                        if(same&&!protected_order&&confirmed!=accepted.end()&&applied(*confirmed->second,a,w)&&state.last_step.order_seq){
                            const auto&actual=*confirmed->second;const auto&previous=state.last_step;
                            if(actual.kind==OrderKind::cover&&actual.role==roles::buddy_answer&&actual.goal_node==previous.goal_node&&
                               eq(actual.goal_position,previous.goal_position)&&actual.issued_at_ms==previous.issued_at_ms&&
                               actual.expires_at_ms<=previous.expires_at_ms&&actual.ownership_generation==previous.ownership_generation&&
                               actual.restrictions_revision==previous.restrictions_revision&&
                               distance_squared(actual.goal_position,a.position)>actual.arrival_radius_m*actual.arrival_radius_m){
                                result.order=actual;result.order.snapshot_seq=w.sequence;orient(buddy->position,buddy->enemy);
                                result.reason=OutdoorFireReason::buddy_step;result.movement_changed=true;continuing_step=true;
                            }
                        }
                        if(!same){state.buddy=buddy->member;state.hit_at=buddy->at_ms;state.buddy_foe=buddy->enemy;state.buddy_point=buddy->position;
                            state.last_answer=w.now_ms;state.start_ammo=a.weapon.ammo_in_mag;}
                        const auto rounds=std::uint16_t(std::floor(a.weapon.mag_size*p.buddy_mag_fraction));
                        const auto spent=state.start_ammo>a.weapon.ammo_in_mag?state.start_ammo-a.weapon.ammo_in_mag:0;
                        const auto available=rounds>spent?rounds-spent:0;
                        const float floor=(std::max)(p.ammo_floor,float(state.start_ammo>(std::min)(state.start_ammo,rounds)?state.start_ammo-rounds:0)/a.weapon.mag_size);
                        if(!protected_order&&eye){result.ray_from=eye->measured_eye;result.ray_to=up(buddy->position,p.target_sample_height_m);
                            result.line=query(buddy->enemy,buddy->at_ms,result.ray_from,result.ray_to);
                            if(result.line==Evidence::yes){result.reason=OutdoorFireReason::buddy_here;
                                fire(buddy->position,buddy->enemy,saturated_after(buddy->at_ms,p.buddy_fire_ms),floor,
                                    std::uint8_t((std::min)(available,int(c.room.suppression_queue))),PointFireSource::buddy_hit,buddy->member,buddy->at_ms);}
                            else if(result.line==Evidence::no&&!continuing_step&&svc.project&&svc.path_length){
                                // Actual native producer: project bounded local steps,
                                // request a restricted full route, then certify every
                                // route node against current room/red geometry. A future
                                // eye sample proves a possible line, never current fire.
                                if(r.route_service&&r.route_budget){
                                    OutdoorLocalNavigation nav{c,w,a,svc,r.route_service,r.route_budget,
                                        &quota.work_left,&quota.queries_left,&quota.work_used,&quota.queries_used,stop};
                                    float dx=buddy->position.x-a.position.x,dz=buddy->position.z-a.position.z;
                                    const float norm=std::hypot(dx,dz);
                                    if(norm>0&&std::isfinite(norm)){
                                        dx/=norm;dz/=norm;constexpr float diagonal=.7071067811865475f;
                                        const float directions[5][2]={{0,1},{0,-1},{diagonal,diagonal},{diagonal,-diagonal},{1,0}};
                                        for(const auto&v:directions){use();
                                            const Vec3 requested{a.position.x+p.buddy_step_m*(dx*v[0]-dz*v[1]),a.position.y,
                                                a.position.z+p.buddy_step_m*(dz*v[0]+dx*v[1])};
                                            Vec3 goal;const auto node=nav.project(requested,goal);
                                            if(node==invalid_node||node==a.node_id||flat2(goal,a.position)>p.buddy_step_m*p.buddy_step_m||
                                               flat2(goal,buddy->position)>flat2(a.position,buddy->position)||
                                               std::abs(goal.y-a.position.y)>c.room.floor_tolerance_m)continue;
                                            bool occupied=false;for(auto friend_key:s.members){use();if(friend_key==a.key)continue;
                                                const auto other=actors.find(key(friend_key));if(other!=actors.end()&&other->second->node_id==node){occupied=true;break;}
                                                const auto reserved=bases.find(key(friend_key));if(reserved!=bases.end()&&reserved->second->goal_node==node){occupied=true;break;}}
                                            if(occupied)continue;RouteProofReceipt proof;float length=0;
                                            if(!nav.route(node,goal,p.buddy_step_m,proof,length)||!nav.same_region(proof))continue;
                                            bool clear=true;for(std::size_t j=0;j<proof.nodes.size();++j){use();
                                                if(!nav.red_clear(proof.nodes[j])||!nav.doorway_clear(proof.points[j+1])){clear=false;break;}}
                                            if(!clear||!safe_line(goal,buddy->position))continue;
                                            const Vec3 sample{goal.x+eye->measured_eye.x-a.position.x,goal.y+eye->measured_eye.y-a.position.y,
                                                goal.z+eye->measured_eye.z-a.position.z};
                                            if(query(buddy->enemy,buddy->at_ms,sample,result.ray_to)!=Evidence::yes)continue;
                                            result.order.kind=OrderKind::cover;result.order.role=roles::buddy_answer;result.order.goal_node=node;
                                            result.order.goal_position=goal;result.order.pace=Pace::walk;result.order.speed_scale=p.buddy_walk_scale;
                                            result.order.assignment={};result.order.flags&=~(order_sprint|order_face_movement);
                                            result.reason=OutdoorFireReason::buddy_step;result.movement_changed=true;break;
                                        }
                                    }
                                }
                                for(const auto&step:r.steps){if(result.movement_changed)break;use();if(!source_ok(step.source,a,w)||step.buddy!=buddy->member||step.hit_at_ms!=buddy->at_ms||
                                    step.target!=buddy->enemy||!eq(step.known_point,buddy->position)||
                                    step.route_safe!=Evidence::yes||step.doorway_safe!=Evidence::yes||step.red_safe!=Evidence::yes||step.node==invalid_node||
                                    !step.assignment_generation||step.assignment_generation==base.assignment.generation||!finite(step.requested)||!finite(step.goal)||!finite(step.sample_eye)||
                                    flat2(step.goal,a.position)>p.buddy_step_m*p.buddy_step_m||flat2(step.goal,buddy->position)>flat2(a.position,buddy->position)||
                                    std::abs(step.goal.y-a.position.y)>c.room.floor_tolerance_m)continue;
                                    if(quota.queries_left<2)break;use();--quota.queries_left;++quota.queries_used;Vec3 snapped;
                                    const auto node=svc.project(step.requested,c.room.projection_m,c.room.floor_tolerance_m,&snapped);poll(stop);
                                    if(node!=step.node||!eq(snapped,step.goal))continue;use();--quota.queries_left;++quota.queries_used;
                                    const float path=svc.path_length(a.node_id,node,c.room.path_nodes);poll(stop);
                                    if(!std::isfinite(path)||path<0||path>p.buddy_step_m||!safe_line(step.goal,buddy->position))continue;
                                    const auto line=query(buddy->enemy,buddy->at_ms,step.sample_eye,result.ray_to);if(line!=Evidence::yes)continue;
                                    result.order.kind=OrderKind::cover;result.order.role=roles::buddy_answer;result.order.goal_node=node;result.order.goal_position=step.goal;
                                    result.order.pace=Pace::walk;result.order.speed_scale=p.buddy_walk_scale;result.order.assignment.generation=step.assignment_generation;
                                    result.reason=OutdoorFireReason::buddy_step;result.movement_changed=true;break;
                                }
                            }
                        }else if(!protected_order&&!eye)result.reason=OutdoorFireReason::missing_eye;
                        if(result.movement_changed){
                            result.order.expires_at_ms=(std::min)(result.order.expires_at_ms,saturated_after(buddy->at_ms,c.room.buddy_answer_ms));
                            if(!continuing_step)state.last_step=result.order;
                        }
                        buddy_handled=true;
                    }
                }
                if(!buddy_handled&&!protected_role(base)){
                    struct Candidate{const FoeKnowledge*f;const SourceFact*source;TimeMs until;int rank;float distance;};std::vector<Candidate> targets;
                    for(const auto&f:m.foes){use();const auto*source=known(f,a.observed_at_ms);auto until=window(f,w.now_ms,p);
                        if(!source||!until||!f.id.valid()||(!f.id.ghost()&&friendly(s,f.id.actor)))continue;
                        targets.push_back({&f,source,until,f.id==m.target?0:f.id==state.last?1:2,flat2(a.position,source->position)});}
                    std::sort(targets.begin(),targets.end(),[&](const Candidate&x,const Candidate&y){use();if(x.rank!=y.rank)return x.rank<y.rank;
                        if(x.distance!=y.distance)return x.distance<y.distance;if(x.f->id.actor!=y.f->id.actor)return key(x.f->id.actor)<key(y.f->id.actor);return x.f->id.ghost_serial<y.f->id.ghost_serial;});
                    for(const auto&t:targets){use();result.target=t.f->id;result.source_at_ms=t.source->at_ms;result.known_position=t.source->position;
                        orient(t.source->position,t.f->id);if(!eye){result.reason=OutdoorFireReason::missing_eye;break;}
                        result.reason=OutdoorFireReason::no_line;bool selected=false;
                        for(float fraction:p.suppression_fractions){use();Vec3 point{a.position.x+(t.source->position.x-a.position.x)*fraction,t.source->position.y,
                                a.position.z+(t.source->position.z-a.position.z)*fraction};
                            const float off=std::sqrt(flat2(point,t.source->position));if(off>p.near_known_m&&off>p.far_known_m)continue;
                            // Every candidate is exactly on the captured own->LK
                            // bearing (zero angle), never a hidden actor transform.
                            result.ray_from=eye->measured_eye;result.ray_to=up(point,p.target_sample_height_m);
                            result.line=query(t.f->id,t.source->at_ms,result.ray_from,result.ray_to);
                            if(result.line==Evidence::unknown){result.reason=OutdoorFireReason::unknown_line;break;}
                            if(result.line!=Evidence::yes)continue;
                            result.selected_point=point;result.reason=OutdoorFireReason::suppress;
                            const auto cycle_at=(!state.cycle_at||state.last!=t.f->id||w.now_ms-state.cycle_at>=p.cycle_ms)?w.now_ms:state.cycle_at;
                            const auto source_index=std::size_t(t.source-t.f->sources.data());
                            const auto source_kind=static_cast<PointFireSource>(source_index+1);
                            if(fire(point,t.f->id,(std::min)(t.until,saturated_after(cycle_at,p.burst_ms)),p.ammo_floor,
                                std::uint8_t(c.room.suppression_queue),source_kind,t.source->reporter,t.source->at_ms)){
                                state.cycle_at=cycle_at;state.last=t.f->id;selected=true;break;
                            }
                            // A known obstruction at the exact endpoint may leave a
                            // shorter permitted point clear. Unknown never proves it.
                            if(result.reason==OutdoorFireReason::no_line)continue;
                            selected=true;break;
                        }
                        if(selected||result.line==Evidence::unknown)break;
                    }
                }
                n->cursor=(slot+1)%s.members.size();
            }catch(const Budget&){const auto lines=state.lines;const auto line_cursor=state.line_cursor;state=before_member;
                state.lines=lines;state.line_cursor=line_cursor;result={};result.order=base;result.reason=OutdoorFireReason::budget;n->cursor=slot;pending=true;break;}
        }
        poll(stop);out.status=pending?OutdoorFireStatus::pending:OutdoorFireStatus::ready;impl_=std::move(n);
    }catch(const Cancelled&){out={};out.status=OutdoorFireStatus::cancelled;}
    catch(...){out={};out.status=OutdoorFireStatus::failed;}
    out.work=quota.work_used-work0;out.queries=quota.queries_used-queries0;return out;
}
}}}
