#include "CombatOutdoorSolo.h"
#include "CombatSquadPush.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace painter { namespace combat { namespace core {
namespace {
struct Cancelled{};struct Budget{};
bool eq(const Vec3&a,const Vec3&b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
float flat2(const Vec3&a,const Vec3&b){const float x=a.x-b.x,z=a.z-b.z;return x*x+z*z;}
bool pos(float v){return std::isfinite(v)&&v>0;}
bool ratio(float v){return pos(v)&&v<=1;}
std::uint64_t mix(std::uint64_t x){x^=x>>30;x*=0xbf58476d1ce4e5b9ULL;x^=x>>27;x*=0x94d049bb133111ebULL;return x^(x>>31);}
float random_unit(std::uint64_t&v){v=mix(v+0x9e3779b97f4a7c15ULL);return float(v>>40)/16777216.f;}
TimeMs scale_time(TimeMs t,float factor){const double v=double(t)*factor;return v>=double((std::numeric_limits<TimeMs>::max)())?
    (std::numeric_limits<TimeMs>::max)():TimeMs(v);}
void close_fire(Order&o,TimeMs now){o.fire.allowed=false;o.fire.at_valid=false;o.fire.until_ms=(std::min)(o.fire.until_ms,now);}
bool valid(const OutdoorSoloPolicy&p){
    const auto&x=p.combat;if(x.enabled){
        if(!pos(x.cqb_m)||!std::isfinite(x.shot_opportunity_cost_m)||x.shot_opportunity_cost_m<0||
            !ratio(x.lean_full)||!ratio(x.lean_crouch)||x.lean_crouch>x.lean_full||!pos(x.lean_edge_m)||!pos(x.target_height_m)||!x.lean_hit_ms)return false;
        float previous=1;for(float scale:x.meter_hold_scale){if(!std::isfinite(scale)||scale<0||scale>previous)return false;previous=scale;}
        for(float cost:x.fire_class_cost_m)if(!std::isfinite(cost)||cost<0)return false;
    }
    for(const auto&f:p.profiles)if(!f.hold_ms||!f.search_ms||!pos(f.aggression)||!pos(f.hold_jitter_min)||f.hold_jitter_max<f.hold_jitter_min||!std::isfinite(f.hold_jitter_max))return false;
    for(float r:p.effective_m)if(!std::isfinite(r)||r<0)return false;
    return pos(p.open_to_cover_m)&&pos(p.reload_cover_m)&&pos(p.cover_floor_m)&&pos(p.cover_stay_m)&&pos(p.distant_m)&&
        pos(p.distant_jitter_min)&&p.distant_jitter_max>=p.distant_jitter_min&&std::isfinite(p.distant_jitter_max)&&p.distant_cycle_ms&&
        pos(p.rush_m)&&p.rush_run_m>=p.rush_m&&std::isfinite(p.rush_run_m)&&ratio(p.rush_ammo)&&ratio(p.dying_hp)&&ratio(p.injured_hp)&&p.injured_hp>=p.dying_hp&&
        p.shift_min_ms&&p.shift_gap_ms&&p.shift_stale_ms&&p.shift_reset_ms&&pos(p.shift_min_m)&&pos(p.mte_min_m)&&p.mte_max_m>=p.mte_min_m&&
        std::isfinite(p.mte_max_m)&&ratio(p.mte_keep)&&p.mte_unseen_ms&&p.mte_fire_ms&&pos(p.search_reach_m)&&pos(p.search_jitter_min)&&
        p.search_jitter_max>=p.search_jitter_min&&std::isfinite(p.search_jitter_max)&&p.freeze_seen_ms&&p.freeze_heard_ms&&p.freeze_min_ms&&
        p.freeze_max_ms>=p.freeze_min_ms&&pos(p.freeze_m)&&p.grenade_response_ms&&p.throw_unseen_ms&&p.throw_seen_ms&&p.throw_heard_ms&&
        p.throw_pending_ms&&p.throw_after_release_ms&&p.throw_cooldown_ms&&pos(p.grenade_react_m)&&pos(p.grenade_safe_m)&&
        pos(p.throw_min_m)&&p.throw_max_m>=p.throw_min_m&&std::isfinite(p.throw_max_m)&&pos(p.throw_friend_m)&&
        pos(p.project_horizontal_m)&&pos(p.project_vertical_m)&&p.aimed_speed>=.3f&&p.aimed_speed<=1.5f&&
        p.run_speed>=.3f&&p.run_speed<=1.5f&&p.sneaky_speed>=.3f&&p.sneaky_speed<=1.5f&&p.path_nodes&&p.path_nodes<=65536;
}
bool source_current(const OutdoorQuerySource&s,const WorldSnapshot&w,const ActorSnapshot&a){return s.actor==a.key&&s.level_epoch==w.level_epoch&&
    s.snapshot_seq==w.sequence&&s.geometry_revision==w.geometry_revision&&s.ownership_generation==a.ownership_generation&&
    s.restrictions_revision==a.restrictions_revision&&s.at_ms==a.observed_at_ms&&s.origin_node==a.node_id&&eq(s.actor_position,a.position);}
bool applied(const Config&c,const WorldSnapshot&w,const ActorSnapshot&a,const Order*o){if(!o)return false;auto v=*o;
    if(v.fire.until_ms<=w.now_ms)close_fire(v,w.now_ms);
    return publishable(v,a,w.now_ms,w.level_epoch,w.geometry_revision)&&o->snapshot_seq<=w.sequence&&a.accessible_current_order&&
        a.last_applied.order_seq==o->order_seq&&a.last_applied.at_ms>=o->issued_at_ms&&a.last_applied.at_ms<=a.observed_at_ms&&
        fresh(a.last_applied.at_ms,w.now_ms,c.snapshot_max_age_ms)&&
        !(a.last_rejected.order_seq==o->order_seq&&a.last_rejected.reason!=reject_none);}
OutdoorCover classify(const CoverSpot&c){if(c.protection_known!=Evidence::yes)return OutdoorCover::unknown;
    if(c.blocked[2])return OutdoorCover::full;if(c.blocked[1])return OutdoorCover::chest;if(c.blocked[0])return OutdoorCover::low;return OutdoorCover::open;}
int rank(OutdoorCover c){return c==OutdoorCover::full?3:c==OutdoorCover::chest?2:c==OutdoorCover::low?1:0;}
}
struct CombatOutdoorSolo::Impl {
    Config c;OutdoorSoloPolicy p;
    struct Relation{ActorKey key;TimeMs at=0;};
    struct ScalarState {
        bool initialized=false;ActorKey actor;std::uint64_t owner=0,restrictions=0,epoch=0,map=0,geometry=0,group=0,sequence=0;
        std::uint32_t side=0,faction=0,mission_revision=0;MissionKind mission=MissionKind::none;TimeMs now=0;
        FoeId focus;TimeMs focus_source=0,visible_since=0,hold_window=0,distant_next=0,distant_until=0;
        TimeMs decision_at=0,shift_next=0,shift_until=0,search_delay=0,searched_at=0,freeze_source=0,freeze_until=0;
        FoeId searched_foe;std::uint32_t shift_node=invalid_node;Vec3 shift_goal,freeze_point;
        std::uint64_t rng=1;OutdoorSoloReason reason=OutdoorSoloReason::unchanged;
        Order throw_order;KnowledgeTrace throw_trace;TimeMs throw_until=0,throw_next=0,throw_after_until=0;bool throwing=false;
    };
    struct State:ScalarState {std::vector<std::uint32_t> used;std::vector<Relation> relations;} state;
    Impl(Config cfg,OutdoorSoloPolicy pol):c(std::move(cfg)),p(std::move(pol)){}
};
CombatOutdoorSolo::CombatOutdoorSolo(Config c,OutdoorSoloPolicy p):impl_(new Impl(std::move(c),std::move(p))){}
CombatOutdoorSolo::~CombatOutdoorSolo()=default;
CombatOutdoorSolo::CombatOutdoorSolo(CombatOutdoorSolo&&) noexcept=default;
CombatOutdoorSolo& CombatOutdoorSolo::operator=(CombatOutdoorSolo&&) noexcept=default;
void CombatOutdoorSolo::reset(){impl_->state=Impl::State{};}
std::unique_ptr<CombatOutdoorSolo> CombatOutdoorSolo::clone(const std::function<bool()>&stop)const{
    try {auto poll=[&](){if(stop&&stop())throw Cancelled{};};poll();auto out=std::make_unique<CombatOutdoorSolo>(impl_->c,impl_->p);
        // Move scalar state separately from the variable observation history.
        const auto&s=impl_->state;auto&t=out->impl_->state;
        static_cast<Impl::ScalarState&>(t)=s;
        for(auto node:s.used){poll();t.used.push_back(node);}
        for(auto row:s.relations){poll();t.relations.push_back(row);}poll();return out;
    }catch(...){return nullptr;}
}
OutdoorSoloResult CombatOutdoorSolo::update(const WorldSnapshot&w,const ActorSnapshot&a,const SideSnapshot&side,
    const OutdoorSoloInput&i,const EngineServices&svc,OutdoorSoloQuota&q,const std::function<bool()>&stop){
    OutdoorSoloResult r;r.proposal=i.personal.proposal;const auto startwork=q.work_used,startquery=q.queries_used;
    // Keep the capture gate visible even when a higher-priority job returns
    // before the optional lean branch. This is diagnostic, never authority.
    r.lean_gate=!i.lean_capture?"capture_missing":
        !source_current(i.lean_capture->source,w,a)?"capture_stale":i.lean_capture->gate;
    auto poll=[&](){if(stop&&stop())throw Cancelled{};};
    auto work=[&](){poll();if(!q.work_left)throw Budget{};--q.work_left;++q.work_used;};
    auto query=[&](){work();if(!q.queries_left)throw Budget{};--q.queries_left;++q.queries_used;};
    auto counts=[&](){r.work=q.work_used-startwork;r.queries=q.queries_used-startquery;};
    auto retire_walk_lean=[&](){auto&o=r.proposal.order;
        const bool changed=o.lean!=0||o.expose.lean||!eq(o.lean_edge,Vec3{});
        o.lean=0;o.lean_edge={};o.expose.lean=false;return changed;};
    try {
        work();const auto&c=impl_->c;const auto&p=impl_->p;
        if(!p.enabled){r.status=OutdoorSoloStatus::disabled;counts();return r;}
        if(!valid(p)||c.factions.count>c.factions.items.size()||!w.level_epoch||!w.sequence||!w.geometry_revision||!valid_key(a.key)||a.level_epoch!=w.level_epoch||
            a.snapshot_seq!=w.sequence||a.observed_at_ms!=w.now_ms||!finite(a.position)||!std::isfinite(a.health)||
            a.hits.count>a.hits.items.size()||a.visible_enemies.count>a.visible_enemies.items.size()||a.seen_dead.count>a.seen_dead.items.size()||
            side.side_id==0||side.faction!=a.faction||side.mission.rooms.count>side.mission.rooms.items.size()||
            i.personal.status!=OutdoorPersonalStatus::ready||i.personal.proposal.order.actor!=a.key||
            i.personal.proposal.order.ownership_generation!=a.ownership_generation){counts();return r;}
        unsigned own=0;for(auto key:side.members){work();if(key==a.key)++own;}if(own!=1){counts();return r;}
        const auto*k=i.source.knowledge;
        if(!k||!k->accepted||k->cancelled||k->level_epoch!=w.level_epoch||k->sequence!=w.sequence||i.source.knowledge_index>=k->members.size()){counts();return r;}
        const auto&m=k->members[i.source.knowledge_index];
        if(m.actor!=a.key||m.ownership_generation!=a.ownership_generation||m.side!=side.side_id||!m.alive||!m.able||(p.combat.enabled&&m.meter_level>=p.combat.meter_hold_scale.size())){counts();return r;}
        const auto&old=impl_->state;
        const bool identity=old.initialized&&old.actor==a.key&&old.owner==a.ownership_generation&&old.restrictions==a.restrictions_revision&&
            old.epoch==w.level_epoch&&old.map==w.map_hash&&old.geometry==w.geometry_revision&&old.side==side.side_id&&old.faction==side.faction&&
            old.group==side.group&&old.mission_revision==side.mission.revision&&old.mission==side.mission.kind;
        if(old.initialized&&((w.level_epoch<old.epoch)||(identity&&(w.sequence<=old.sequence||w.now_ms<old.now)))){r.status=OutdoorSoloStatus::stale;counts();return r;}
        Impl::State s;
        if(identity){static_cast<Impl::ScalarState&>(s)=old;for(auto node:old.used){work();s.used.push_back(node);}
            for(auto row:old.relations){work();s.relations.push_back(row);}}
        else {r.reset=true;s.decision_at=w.now_ms;s.rng=mix(c.seed^a.key.generation^(std::uint64_t(a.key.id)<<32)^side.side_id);}
        s.initialized=true;s.actor=a.key;s.owner=a.ownership_generation;s.restrictions=a.restrictions_revision;s.epoch=w.level_epoch;s.map=w.map_hash;
        s.geometry=w.geometry_revision;s.group=side.group;s.side=side.side_id;s.faction=side.faction;s.mission_revision=side.mission.revision;s.mission=side.mission.kind;s.sequence=w.sequence;s.now=w.now_ms;
        r.status=OutdoorSoloStatus::ready;
        auto finish=[&](OutdoorSoloReason why,bool changed=false){poll();r.reason=why;r.changed=changed;
            r.navigation_pending=r.proposal.navigation_pending;
            r.movement_changed=r.proposal.order.goal_node!=i.personal.proposal.order.goal_node||!eq(r.proposal.order.goal_position,i.personal.proposal.order.goal_position)||r.proposal.order.pace!=i.personal.proposal.order.pace;
            if(s.reason!=why)s.decision_at=w.now_ms;s.reason=why;impl_->state=std::move(s);counts();return r;};
        if(a.yield_reasons)return finish(OutdoorSoloReason::native_yield);
        const auto reason=i.personal.reason;
        const bool squad_push=i.squad_push_allowed&&!i.genuine_commitment&&
            squad_push_eligible(c,p,w,a,side,m,i.personal.proposal.order);
        auto independent_advance=[&](const Order&o){return (o.kind==OrderKind::rush&&(o.role==roles::individual||(o.role==roles::suppression_push&&!squad_push)))||
            (o.kind==OrderKind::search&&o.role==roles::search)||
            (o.kind==OrderKind::bound&&o.role==roles::mover&&!i.source.bound_commitment);};
        if(i.coordinated_side&&!i.genuine_commitment&&reason!=OutdoorPersonalReason::hands&&
            independent_advance(i.personal.proposal.order)){
            // Joining an element retires a previous lone-man pursuit as well
            // as preventing new ones. Prefer the current authored side job.
            if(i.source.base.order.actor==a.key&&i.source.base.order.ownership_generation==a.ownership_generation&&
                !independent_advance(i.source.base.order))r.proposal=i.source.base;
            else {r.proposal=i.personal.proposal;r.proposal.order=first_response(a,c,w.now_ms,w.level_epoch,w.geometry_revision,r.proposal.order.order_seq);
                r.proposal.order.flags&=~order_first_response;}
            return finish(OutdoorSoloReason::commitment,true);
        }
        // Tactical reload is a requested action from Self, not a fabricated
        // native weapon state. Preserve its exact stationary or moving order.
        if(reason==OutdoorPersonalReason::reload&&i.source.tactical_reload)return finish(OutdoorSoloReason::personal_layer);
        // Room PIE already owns simultaneous walking, aiming and leaning.
        // Preserve the currently attained pose without giving Solo its route.
        // This is pose continuity for an authored lean, not a new sight proof.
        if(reason==OutdoorPersonalReason::committed_room&&p.combat.enabled){
            auto& o=r.proposal.order;const auto* l=i.lean_capture;
            if(o.kind==OrderKind::peek&&o.pie.valid&&o.portal.valid&&o.assignment.generation&&
                !(o.flags&order_committed_entry)&&!o.throw_order.valid&&!valid_key(o.aid)&&
                (o.pace==Pace::walk||o.pace==Pace::stand)&&std::isfinite(o.lean)&&o.lean!=0&&
                !a.reloading&&a.weapon.state!=WeaponState::reloading&&l&&l->measured&&finite(l->measured_eye)&&
                source_current(l->source,w,a)&&(l->pose_only||(!l->source.focus.ghost()&&l->source.focus==m.target))&&
                (l->attained_stance==Stance::stand||l->attained_stance==Stance::crouch)){
                if(l->pose_only){
                    const float lean=std::copysign(std::min(std::fabs(o.lean),l->attained_stance==Stance::crouch?p.combat.lean_crouch:p.combat.lean_full),o.lean);
                    const bool changed=o.stance!=l->attained_stance||o.lean!=lean;
                    o.stance=l->attained_stance;o.lean=lean;r.lean_gate="room_pie_attained_pose";
                    return finish(OutdoorSoloReason::commitment,changed);
                }
                for(const auto& foe:m.foes){work();if(foe.id!=l->source.focus||foe.down||foe.wounded)continue;
                    const auto* fact=foe.last_known();
                    if(fact&&fact->valid&&valid_key(fact->reporter)&&finite(fact->position)&&
                        fact->at_ms==l->source.focus_at_ms&&eq(fact->position,l->source.focus_position)&&
                        fresh(fact->at_ms,w.now_ms,c.memory_max_age_ms)){
                        const float lean=std::copysign(std::min(std::fabs(o.lean),l->attained_stance==Stance::crouch?p.combat.lean_crouch:p.combat.lean_full),o.lean);
                        const bool changed=o.stance!=l->attained_stance||o.lean!=lean;
                        o.stance=l->attained_stance;o.lean=lean;r.lean_gate="room_pie_attained_pose";
                        return finish(OutdoorSoloReason::commitment,changed);
                    }
                    break;
                }
            }
        }
        const auto& committed=i.personal.proposal.order;
        const bool pose_commitment=(i.genuine_commitment||reason==OutdoorPersonalReason::committed_bound)&&
            (reason==OutdoorPersonalReason::personal_contact||reason==OutdoorPersonalReason::squad||
             reason==OutdoorPersonalReason::continue_cover||reason==OutdoorPersonalReason::committed_bound)&&
            (committed.role==roles::base_of_fire||committed.kind==OrderKind::bound)&&
            (committed.pace==Pace::stand||committed.pace==Pace::walk)&&
            reason!=OutdoorPersonalReason::hands&&!committed.throw_order.valid&&!valid_key(committed.aid)&&
            !(committed.flags&order_committed_entry)&&!a.reloading&&a.weapon.state!=WeaponState::reloading;
        if((i.genuine_commitment||reason==OutdoorPersonalReason::committed_room||reason==OutdoorPersonalReason::committed_bound||reason==OutdoorPersonalReason::hands)&&!pose_commitment)
            return finish(OutdoorSoloReason::commitment);
        if(reason==OutdoorPersonalReason::squad&&!pose_commitment){const auto&b=i.personal.proposal.order;
            auto sight=[&](const Contact&t){return valid_key(t.key)&&t.hostile&&t.alive_when_observed&&!t.wounded&&finite(t.position)&&personally_visible(t,a);};
            bool seeing=sight(a.enemy);for(std::size_t n=0;n<a.visible_enemies.count;++n){work();seeing=seeing||sight(a.visible_enemies.items[n]);}
            if(b.kind==OrderKind::press||(b.kind==OrderKind::rush&&!squad_push)||(b.kind==OrderKind::regroup&&!seeing)||
                (b.role==roles::casualty_guard&&valid_key(b.aid))||
                (b.role==roles::base_of_fire&&i.source.base_commitment&&i.personal.personal_contact))return finish(OutdoorSoloReason::commitment);
        }
        auto baseline=first_response(a,c,w.now_ms,w.level_epoch,w.geometry_revision,r.proposal.order.order_seq);
        if(baseline.kind==OrderKind::yield)return finish(OutdoorSoloReason::native_yield);
        auto emit=[&](Order o){r.proposal={};o.flags&=~order_first_response;r.proposal.order=o;r.proposal.group_id=i.personal.proposal.group_id;
            r.proposal.expected_previous_order_seq=i.personal.proposal.expected_previous_order_seq;};
        const bool danger=a.danger.kind==DangerKind::grenade&&finite(a.danger.position)&&fresh(a.danger.at_ms,w.now_ms,p.grenade_response_ms)&&
            flat2(a.position,a.danger.position)<p.grenade_react_m*p.grenade_react_m;
        auto reserve_free=[&](std::uint32_t node,const Vec3&point){const auto*v=i.source.reservations;
            if(!v||v->level_epoch!=w.level_epoch||v->snapshot_seq!=w.sequence)return false;
            for(const auto&x:v->members){work();if(x.actor==a.key)continue;if(!valid_key(x.actor)||!finite(x.position)||x.node==node||
                distance_squared(x.position,point)<c.minimum_goal_separation*c.minimum_goal_separation)return false;}return true;};
        if(pose_commitment&&danger)return finish(OutdoorSoloReason::commitment,retire_walk_lean());
        if(danger){const auto*e=i.grenade_escape;
            if(e&&source_current(e->source,w,a)&&e->source.focus_at_ms==a.danger.at_ms&&eq(e->source.focus_position,a.danger.position)&&
                source_current(e->route.source,w,a)&&e->route.source.focus_at_ms==a.danger.at_ms&&eq(e->route.source.focus_position,a.danger.position)&&
                e->route.goal_node!=invalid_node&&finite(e->route.goal)&&e->route.restricted_route==Evidence::yes&&e->route.threat_opening_safe==Evidence::yes&&
                flat2(e->route.goal,a.danger.position)>=p.grenade_safe_m*p.grenade_safe_m&&reserve_free(e->route.goal_node,e->route.goal)){
                auto o=baseline;o.kind=OrderKind::cover;o.role=roles::under_fire_cover;o.goal_node=e->route.goal_node;o.goal_position=e->route.goal;
                o.pace=Pace::run;o.speed_scale=p.run_speed;o.flags=order_urgent|order_sprint|order_face_movement;o.look.kind=LookKind::path;close_fire(o,w.now_ms);
                emit(o);return finish(OutdoorSoloReason::grenade_escape,true);
            }
            r.status=OutdoorSoloStatus::pending;return finish(OutdoorSoloReason::grenade_unknown);
        }
        if(reason==OutdoorPersonalReason::native_yield)return finish(OutdoorSoloReason::native_yield);
        KnowledgeTrace escape_trace;
        const bool urgent_escape=reason==OutdoorPersonalReason::escape_unavailable&&
            outdoor_personal_cover_focus(c,w,a,m,escape_trace);
        if(reason==OutdoorPersonalReason::escape_unavailable&&!urgent_escape){r.status=OutdoorSoloStatus::pending;return finish(OutdoorSoloReason::urgent_escape_pending);}
        if(reason==OutdoorPersonalReason::unseen_escape||
            (reason==OutdoorPersonalReason::dogfight&&i.personal.proposal.order.pace!=Pace::stand&&
             flat2(i.personal.proposal.order.goal_position,a.position)>i.personal.proposal.order.arrival_radius_m*i.personal.proposal.order.arrival_radius_m)||
            (reason==OutdoorPersonalReason::continue_cover&&!pose_commitment))
            return finish(OutdoorSoloReason::personal_layer);
        const bool reloading=a.reloading||a.weapon.state==WeaponState::reloading||!a.weapon.ammo_in_mag;
        if(!pose_commitment&&!urgent_escape&&!reloading&&!i.personal.personal_contact&&!i.allow_noncontact&&!squad_push&&side.members.size()!=1)return finish(OutdoorSoloReason::unchanged);
        const FactionStyle*style=&c.faction_default;for(std::size_t n=0;n<c.factions.count;++n){work();const auto&v=c.factions.items[n];if(v.faction==a.faction){style=&v;break;}}
        if(style->profile>=p.profiles.size()){r.status=OutdoorSoloStatus::invalid;counts();return r;}const auto&profile=p.profiles[style->profile];
        auto sample=[&](float lo,float hi){return lo+(hi-lo)*random_unit(s.rng);};
        auto relation=[&](const Contact&t){if(!valid_key(t.key)||t.observed_at_ms>a.observed_at_ms)return;
            if(t.hostile&&t.alive_when_observed&&!t.wounded)return;
            for(auto&v:s.relations){work();if(v.key==t.key){v.at=(std::max)(v.at,t.observed_at_ms);return;}}
            s.relations.push_back({t.key,t.observed_at_ms});};
        relation(a.enemy);for(std::size_t n=0;n<a.visible_enemies.count;++n){work();relation(a.visible_enemies.items[n]);}
        auto forbidden=[&](FoeId id,TimeMs at){if(id.ghost())return id.side!=side.side_id;
            for(auto key:side.members){work();if(key==id.actor)return true;}
            for(std::size_t n=0;n<a.seen_dead.count;++n){work();if(a.seen_dead.items[n]==id.actor)return true;}
            for(const auto&v:s.relations){work();if(v.key==id.actor&&v.at>=at)return true;}return false;};
        const FoeKnowledge*f=nullptr;const SourceFact*fact=nullptr;
        for(const auto&candidate:m.foes){work();if(candidate.id!=(urgent_escape?escape_trace.foe:m.target)||candidate.down||candidate.wounded)continue;
            const auto*lk=candidate.last_known();
            if(urgent_escape&&!forbidden(candidate.id,escape_trace.observed_at_ms))f=&candidate;
            else if(lk&&lk->valid&&finite(lk->position)&&valid_key(lk->reporter)&&fresh(lk->at_ms,w.now_ms,c.knowledge.forget_ms)&&!forbidden(candidate.id,lk->at_ms)){f=&candidate;fact=lk;}break;}
        // Historical relation rows are retained only while their target remains
        // in this observer's knowledge. No other observer's relation is read.
        for(auto it=s.relations.begin();it!=s.relations.end();){work();bool keep=false;for(const auto&v:m.foes){work();if(!v.id.ghost()&&v.id.actor==it->key){keep=true;break;}}
            if(keep)++it;else it=s.relations.erase(it);}
        SourceFact hit_source;
        if(urgent_escape&&f){hit_source={true,escape_trace.position,escape_trace.observed_at_ms,a.key,SoundKind::other};fact=&hit_source;}
        const Contact*seen=nullptr;
        auto current=[&](const Contact&t){return f&&!f->id.ghost()&&t.key==f->id.actor&&t.hostile&&t.alive_when_observed&&!t.wounded&&
            personally_visible(t,a)&&t.last_seen_ms<=a.observed_at_ms&&finite(t.position);};
        if(current(a.enemy))seen=&a.enemy;else for(std::size_t n=0;n<a.visible_enemies.count;++n){work();if(current(a.visible_enemies.items[n])){seen=&a.visible_enemies.items[n];break;}}
        // Sight uses current captured coordinates. A memory record's older
        // update timestamp does not invalidate THIS-snapshot visible contact.
        SourceFact sight;
        if(seen&&!urgent_escape){sight={true,seen->position,w.now_ms,a.key,SoundKind::other};fact=&sight;}
        if(f){r.focus=f->id;r.source_at_ms=fact->at_ms;r.known_position=fact->position;}
        const bool newfocus=f&&(s.focus!=f->id);
        if(newfocus){s.focus=f->id;s.focus_source=fact->at_ms;s.visible_since=0;s.hold_window=0;s.search_delay=scale_time(profile.search_ms,sample(p.search_jitter_min,p.search_jitter_max));s.used.clear();}
        if(seen){if(!s.visible_since)s.visible_since=w.now_ms;if(!s.hold_window)s.hold_window=scale_time(profile.hold_ms,profile.aggression*sample(profile.hold_jitter_min,profile.hold_jitter_max));}
        else{s.visible_since=0;s.hold_window=0;}
        const bool live=applied(c,w,a,i.source.applied);
        const auto*cache=i.source.cover;const auto*qp=i.source.cover_profile;
        const bool cache_ok=f&&cache&&cache->actor==a.key&&cache->level_epoch==w.level_epoch&&cache->snapshot_seq==w.sequence&&cache->ownership_generation==a.ownership_generation&&
            cache->restrictions_revision==a.restrictions_revision&&cache->origin_node==a.node_id&&cache->observed_at_ms==a.observed_at_ms&&
            cache->knowledge_threat==f->id&&cache->knowledge_observed_at_ms==fact->at_ms&&eq(cache->knowledge_threat_position,fact->position);
        const bool measured=cache_ok&&qp&&source_current(qp->source,w,a)&&qp->source.focus==f->id&&qp->source.focus_at_ms==fact->at_ms&&
            eq(qp->source.focus_position,fact->position)&&qp->heights==std::array<float,3>{{.7f,1.1f,1.7f}};
        struct Choice{const CoverSpot*spot;OutdoorCover cover;float cost;};std::vector<Choice> spots;const CoverSpot*here=nullptr;
        if(cache_ok&&!pose_commitment)for(const auto&v:cache->spots){work();++r.candidates;
            if(v.node==invalid_node||!finite(v.position)||!std::isfinite(v.dc)||v.dc<0||!std::isfinite(v.lo)||v.lo<0||v.lo>1||
                v.geometry_revision!=w.geometry_revision||v.accessible==Evidence::no||v.reachable==Evidence::no||
                (v.accessible==Evidence::unknown?v.restrictions_revision!=0:v.restrictions_revision!=a.restrictions_revision)||
                (!p.allow_pending_navigation&&(v.accessible!=Evidence::yes||v.reachable!=Evidence::yes))||
                (!f->id.ghost()?(v.threat_key!=f->id.actor||v.threat_observed_at_ms!=fact->at_ms):valid_key(v.threat_key))||
                (a.last_rejected.node==v.node&&a.last_rejected.restrictions_revision==a.restrictions_revision&&
                    (a.last_rejected.reason==reject_goal_invalid||a.last_rejected.reason==reject_goal_inaccessible))||
                std::fabs(v.position.y-a.position.y)>p.cover_floor_m||(!urgent_escape&&!reserve_free(v.node,v.position)))continue;
            if(urgent_escape){
                if(!i.source.reservations||i.source.reservations->level_epoch!=w.level_epoch||i.source.reservations->snapshot_seq!=w.sequence)continue;
                bool blocked=false;for(const auto&mate:i.source.reservations->members){work();if(mate.actor==a.key)continue;
                    if(!valid_key(mate.actor)||!finite(mate.position)||mate.node==v.node){blocked=true;break;}}
                if(blocked)continue;
            }
            const auto cls=measured?classify(v):OutdoorCover::unknown;
            if(v.node==a.node_id&&flat2(v.position,a.position)<=p.cover_stay_m*p.cover_stay_m){here=&v;r.here=cls;}
            const bool unknown=cls==OutdoorCover::unknown;
            const float cost=v.dc+(unknown?1.f:v.lo)*c.cover_exposure_cost_m+
                (float(unknown)+float(v.accessible==Evidence::unknown||v.reachable==Evidence::unknown))*c.cover_unknown_cost_m;
            const float class_cost=p.combat.enabled&&rank(cls)?p.combat.fire_class_cost_m[rank(cls)-1]:0;
            const float element_cost=i.coordinated_side&&i.element_anchor_valid&&finite(i.element_anchor)?std::sqrt(flat2(v.position,i.element_anchor)):0.f;
            if(std::isfinite(cost+class_cost+element_cost))spots.push_back({&v,cls,cost+class_cost+element_cost});
        }
        if(here&&rank(r.here)){bool used=false;for(auto node:s.used){work();if(node==here->node)used=true;}if(!used)s.used.push_back(here->node);}
        auto trace=[&](){auto&t=r.proposal.knowledge;r.proposal.has_knowledge=true;t.receiver=a.key;t.receiver_ownership_generation=a.ownership_generation;
            t.receiver_faction=a.faction;t.source_snapshot_seq=w.sequence;t.observer=fact->reporter;t.foe=f->id;t.target=f->id.actor;
            t.position=fact->position;t.observed_at_ms=fact->at_ms;t.unidentified=f->id.ghost();t.shared=fact->reporter!=a.key;t.personally_visible=seen!=nullptr;};
        const bool own_fire=seen&&personally_visible(*seen,a)&&seen->last_seen_ms<=a.observed_at_ms&&seen->hostile&&seen->alive_when_observed&&!seen->wounded&&
            !(a.flags&flag_dont_shoot)&&!reloading&&a.weapon.ammo_in_mag&&c.fire_window_ms&&
            (a.weapon.state==WeaponState::idle||a.weapon.state==WeaponState::firing);
        auto fight_order=[&](){auto o=baseline;
            if(f){o.look.kind=seen?LookKind::enemy:LookKind::point;o.look.target=seen?f->id.actor:ActorKey{};o.look.value=fact->position;}
            // Current personal sight is a capture fact even when the native
            // memory record has an old update time. Shared K1 alone never fires.
            o.fire.allowed=own_fire;o.fire.at_valid=false;
            o.fire.until_ms=own_fire?fire_window_until(c,w.now_ms,o.expires_at_ms):w.now_ms;
            return o;};
        auto lean_gate=[&](){const auto*l=i.lean_capture;
            if(!l)return "capture_missing";
            if(!source_current(l->source,w,a))return "capture_stale";
            if(!f)return "focus_missing";
            if(l->pose_only)return l->gate;
            if(l->source.focus!=f->id)return "focus_mismatch";
            if(l->source.focus_at_ms!=fact->at_ms)return "focus_time_mismatch";
            if(!eq(l->source.focus_position,fact->position))return "focus_position_mismatch";
            return l->gate;};
        r.lean_gate=lean_gate();
        bool lean_geometry_checked=false,lean_geometry_clear=false,lean_centre_blocked=false;
        const char* lean_geometry_gate="none";float lean_geometry_sign=0;Vec3 lean_geometry_edge;
        auto pose=[&](Order&o,OutdoorCover cls){
            if(!p.combat.enabled)return;
            o.stance=(cls==OutdoorCover::low||cls==OutdoorCover::chest||(!rank(cls)&&m.meter_level>=2))?Stance::crouch:Stance::stand;
            if(reloading){o.lean=0;o.lean_edge={};o.expose={};return;}
            // This path never treats a requested pose as an attained pose.
            if(o.expose.lean){o.lean=std::copysign(std::min(std::fabs(o.lean),o.stance==Stance::crouch?p.combat.lean_crouch:p.combat.lean_full),o.lean);return;}
            // Chest cover keeps a crouched lean; pressure changes cost, not permission.
            const auto*l=i.lean_capture;
            r.lean_gate=lean_gate();
            const bool measured_pose=l&&l->measured&&(l->pose_only||l->edge_measured)&&source_current(l->source,w,a)&&f&&!f->id.ghost()&&
                (l->pose_only||(l->source.focus==f->id&&l->source.focus_at_ms==fact->at_ms&&eq(l->source.focus_position,fact->position)))&&
                fresh(fact->at_ms,w.now_ms,c.memory_max_age_ms)&&
                (l->attained_stance==Stance::stand||l->attained_stance==Stance::crouch);
            // Use the authenticated attained stance. An actor-only receipt
            // supplies no edge; the current target's geometry is proved below.
            // Neither receipt certifies a cover class or permission to fire.
            if(measured_pose)o.stance=l->attained_stance;
            // A walk can lean before the centre reaches an edge. The current
            // measured shoulder must clear it in the existing walking stance;
            // the requested destination is never a substitute for that proof.
            const bool measured_walk=o.pace==Pace::walk&&i.personal.proposal.order.pace==Pace::walk&&
                o.stance==i.personal.proposal.order.stance;
            if(!measured_pose||!finite(l->measured_eye)||l->attained_stance!=o.stance||!svc.rays||(o.pace!=Pace::stand&&!measured_walk)||
                (!measured_walk&&flat2(o.goal_position,a.position)>p.cover_stay_m*p.cover_stay_m)||
                (!l->pose_only&&(!finite(l->edge)||flat2(l->edge,a.position)>p.combat.lean_edge_m*p.combat.lean_edge_m||std::fabs(l->edge.y-a.position.y)>p.cover_floor_m))){if(measured_pose)r.lean_gate="source_pose_or_motion";return;}
            for(std::size_t n=0;n<a.hits.count;++n){work();if(fresh(a.hits.items[n].at_ms,w.now_ms,p.combat.lean_hit_ms)){r.lean_gate="recent_hit";return;}}
            const float reach=o.stance==Stance::crouch?c.lean_eye_offset_m_crouch:c.lean_eye_offset_m_stand;
            const float len=std::sqrt(a.forward.x*a.forward.x+a.forward.z*a.forward.z);
            if(!pos(reach)||!pos(len)||reach>p.combat.lean_edge_m){r.lean_gate="invalid_offset";return;}
            const Vec3 right{a.forward.z/len,0,-a.forward.x/len};
            Vec3 target=fact->position;target.y+=p.combat.target_height_m;
            auto clear=[&](Vec3 from,Vec3 to){query();Segment ray{from,to};float hit=-2;svc.rays(&ray,&hit,1);poll();
                if(hit==-1.f)return Evidence::yes;return std::isfinite(hit)&&hit>=0?Evidence::no:Evidence::unknown;};
            // Sweep and resulting line are measured on static geometry. They
            // neither certify dynamic teammate clearance nor extend fire.
            if(!lean_geometry_checked){
                lean_geometry_checked=true;lean_geometry_gate="centre_not_blocked";
                lean_centre_blocked=clear(l->measured_eye,target)==Evidence::no;
                if(lean_centre_blocked){
                    const float side=(l->edge.x-a.position.x)*right.x+(l->edge.z-a.position.z)*right.z;
                    for(float sign:{-1.f,1.f}){
                        if(!l->pose_only&&(!std::isfinite(side)||side==0||(side>0)!=(sign>0)))continue;
                        Vec3 tip=l->measured_eye;tip.x+=right.x*reach*sign;tip.z+=right.z*reach*sign;
                        lean_geometry_gate="sweep_blocked_unknown";if(clear(l->measured_eye,tip)!=Evidence::yes)continue;
                        lean_geometry_gate="side_blocked_unknown";if(clear(tip,target)!=Evidence::yes)continue;
                        lean_geometry_clear=true;lean_geometry_sign=sign;
                        lean_geometry_edge=l->pose_only?Vec3{a.position.x+right.x*reach*sign,a.position.y,a.position.z+right.z*reach*sign}:l->edge;
                        break;
                    }
                }
            }
            r.lean_gate=lean_geometry_gate;if(!lean_geometry_clear)return;
            r.lean_gate="lean_selected";
            o.expose.stand=false;o.expose.lean=true;o.lean=lean_geometry_sign*(o.stance==Stance::crouch?p.combat.lean_crouch:p.combat.lean_full);
            o.lean_edge=lean_geometry_edge;r.lean_static_verified=true;
        };
        if(pose_commitment){
            // Formation and bound commitments protect movement, not a ban on
            // exposing the attained shoulder. Authenticate current geometry
            // without proposing a different cell, pace, target or fire lease.
            auto posed=r.proposal.order;posed.lean=0;posed.lean_edge={};posed.expose={};
            pose(posed,OutdoorCover::unknown);
            if(r.lean_static_verified){auto& kept=r.proposal.order;
                const bool changed=kept.stance!=posed.stance||kept.lean!=posed.lean||!eq(kept.lean_edge,posed.lean_edge)||
                    kept.expose.stand!=posed.expose.stand||kept.expose.lean!=posed.expose.lean;
                kept.stance=posed.stance;kept.lean=posed.lean;kept.lean_edge=posed.lean_edge;kept.expose=posed.expose;
                r.lean_gate="committed_lean_selected";return finish(OutdoorSoloReason::commitment,changed);
            }
            return finish(OutdoorSoloReason::commitment,retire_walk_lean());
        }
        auto use_spot=[&](const Choice&v,OrderKind kind,std::uint16_t role,Pace pace,bool fire){auto o=fight_order();o.kind=kind;o.role=role;o.goal_node=v.spot->node;o.goal_position=v.spot->position;
            o.pace=flat2(o.goal_position,a.position)<=o.arrival_radius_m*o.arrival_radius_m?Pace::stand:pace;o.speed_scale=o.pace==Pace::run?p.run_speed:p.aimed_speed;
            o.stance=(v.cover==OutdoorCover::low)?Stance::crouch:Stance::stand;
            if(!fire||o.pace==Pace::run)close_fire(o,w.now_ms);
            pose(o,v.cover);emit(o);trace();r.proposal.has_cover_evidence=true;r.proposal.cover_evidence=*v.spot;r.proposal.cover_cost=v.cost;
            r.proposal.navigation_pending=v.spot->accessible!=Evidence::yes||v.spot->reachable!=Evidence::yes;};
        auto choose=[&](const std::function<bool(const Choice&)>&ok)->const Choice*{const Choice*best=nullptr;
            for(const auto&v:spots){work();if(ok(v)&&(!best||v.cost<best->cost||(v.cost==best->cost&&v.spot->node<best->spot->node)))best=&v;}return best;};
        auto protected_route=[&](const CoverSpot&v){if(!i.routes||!f)return false;for(const auto&x:*i.routes){work();if(source_current(x.source,w,a)&&x.source.focus==f->id&&
            x.source.focus_at_ms==fact->at_ms&&eq(x.source.focus_position,fact->position)&&x.goal_node==v.node&&eq(x.goal,v.position)&&
            x.restricted_route==Evidence::yes&&x.threat_opening_safe==Evidence::yes)return true;}return false;};
        if(urgent_escape){
            // Broader SOLO cover range is the fallback after the local escape
            // field has no usable cell. Exposed cells are still alternatives;
            // unknown navigation is kept pending for native publication.
            const auto*v=choose([&](const Choice&v){return flat2(v.spot->position,a.position)>
                baseline.arrival_radius_m*baseline.arrival_radius_m;});
            if(v){use_spot(*v,OrderKind::cover,roles::seek_cover,Pace::run,false);
                r.proposal.order.pace=Pace::run;r.proposal.order.stance=Stance::stand;r.proposal.order.flags|=order_urgent;
                if(fresh(escape_trace.observed_at_ms,w.now_ms,c.knowledge.uf_run_ms))r.proposal.order.flags|=order_sprint;
                return finish(OutdoorSoloReason::seek_cover,true);}
            // No valid destination means pending work, not a successful escape
            // or a new crouched hold. Preserve source time and the run intent.
            auto o=fight_order();o.kind=OrderKind::cover;o.role=roles::seek_cover;o.pace=Pace::run;o.stance=Stance::stand;
            o.speed_scale=p.run_speed;o.flags=order_urgent;
            if(fresh(escape_trace.observed_at_ms,w.now_ms,c.knowledge.uf_run_ms))o.flags|=order_sprint;
            close_fire(o,w.now_ms);emit(o);if(f)trace();r.proposal.navigation_pending=true;r.status=OutdoorSoloStatus::pending;
            return finish(OutdoorSoloReason::urgent_escape_pending,true);
        }
        if(reloading){if(i.personal.acknowledged_motion)return finish(OutdoorSoloReason::retreat);
            const auto*v=choose([&](const Choice&v){return rank(v.cover)&&v.spot->dc<=p.reload_cover_m&&
                (flat2(v.spot->position,a.position)<=baseline.arrival_radius_m*baseline.arrival_radius_m||protected_route(*v.spot));});
            if(v){use_spot(*v,OrderKind::reload,roles::retreat_cover,Pace::run,false);return finish(OutdoorSoloReason::retreat,true);}
            return finish(OutdoorSoloReason::retreat);
        }
        const unsigned weapon=unsigned(a.weapon.kind);const float eff=weapon<p.effective_m.size()?p.effective_m[weapon]:0;
        const float dist=f?std::sqrt(flat2(a.position,fact->position)):0;
        const float ammo=a.weapon.mag_size?float(a.weapon.ammo_in_mag)/a.weapon.mag_size:0;
        const bool canfire=own_fire;
        // Pending transactions keep the original target/source even when this
        // capture selected another foe. Native accepted in-hand work was already
        // protected by Personal above. Heavy suppression cannot invent success.
        if(s.throwing){const auto&ts=a.throw_state;bool known=false;
            for(const auto&v:m.foes){work();if(v.id!=s.throw_trace.foe||v.down||v.wounded)continue;
                for(const auto&source:v.sources){work();if(source.valid&&source.reporter==s.throw_trace.observer&&source.at_ms==s.throw_trace.observed_at_ms&&
                    eq(source.position,s.throw_trace.position)&&fresh(source.at_ms,w.now_ms,c.knowledge.forget_ms))known=true;}}
            const bool exact=live&&i.source.applied->throw_order.valid&&i.source.applied->throw_order.serial==s.throw_order.throw_order.serial&&
                i.source.applied->throw_order.item==s.throw_order.throw_order.item&&eq(i.source.applied->throw_order.target,s.throw_order.throw_order.target)&&
                ts.serial==s.throw_order.throw_order.serial&&ts.at_ms>=s.throw_order.issued_at_ms&&ts.at_ms<=w.now_ms;
            const bool released=exact&&ts.phase==3&&ts.releases&&ts.projectile!=invalid_id&&ts.source_item==s.throw_order.throw_order.item&&
                ts.released_at_ms>=s.throw_order.issued_at_ms&&ts.released_at_ms<=w.now_ms;
            if(released||(exact&&ts.phase==4)||w.now_ms>=s.throw_until||!known){s.throwing=false;s.throw_next=saturated_after(w.now_ms,p.throw_cooldown_ms);
                if(released)s.throw_after_until=saturated_after(ts.released_at_ms,p.throw_after_release_ms);}
            else {auto o=s.throw_order;o.snapshot_seq=w.sequence;o.expires_at_ms=(std::min)(s.throw_until,saturated_after(w.now_ms,c.order_lifetime_ms));emit(o);
                r.proposal.has_knowledge=true;r.proposal.knowledge=s.throw_trace;r.proposal.knowledge.source_snapshot_seq=w.sequence;
                r.throw_pending=true;return finish(OutdoorSoloReason::throw_grenade,true);}
        }
        if(w.now_ms<s.throw_after_until){auto o=baseline;o.look.kind=LookKind::point;o.look.target={};o.look.value=s.throw_order.throw_order.target;close_fire(o,w.now_ms);
            emit(o);return finish(OutdoorSoloReason::throw_grenade,true);}
        // A personally remembered line can be inspected from a verified edge
        // without inventing current sight or permission to fire.
        auto project=[&](Vec3 point,float max_path,OrderKind kind,std::uint16_t role,Pace pace)->bool{
            if(!svc.project||!svc.path_length||!p.allow_pending_navigation)return false;
            query();Vec3 snapped;const auto node=svc.project(point,p.project_horizontal_m,p.project_vertical_m,&snapped);poll();
            if(node==invalid_node||!finite(snapped)||flat2(snapped,point)>p.project_horizontal_m*p.project_horizontal_m||std::fabs(snapped.y-point.y)>p.project_vertical_m||
                (a.last_rejected.node==node&&a.last_rejected.restrictions_revision==a.restrictions_revision&&
                    (a.last_rejected.reason==reject_goal_invalid||a.last_rejected.reason==reject_goal_inaccessible))||!reserve_free(node,snapped))return false;
            query();const float length=svc.path_length(a.node_id,node,p.path_nodes);poll();if(!std::isfinite(length)||length<0||length>max_path)return false;
            auto o=fight_order();o.kind=kind;o.role=role;o.goal_node=node;o.goal_position=snapped;o.pace=pace;o.speed_scale=pace==Pace::run?p.run_speed:p.aimed_speed;
            if(pace==Pace::run)close_fire(o,w.now_ms);emit(o);trace();r.proposal.navigation_pending=true;return true;};
        if(squad_push&&f){
            const float range=a.health>=p.injured_hp?p.rush_run_m:p.rush_m;
            if(dist<range&&project(fact->position,range,OrderKind::rush,roles::suppression_push,Pace::run))
                return finish(OutdoorSoloReason::rush,true);
        }

        if(!reloading&&i.personal.proposal.order.pace==Pace::walk&&
            flat2(i.personal.proposal.order.goal_position,a.position)>i.personal.proposal.order.arrival_radius_m*i.personal.proposal.order.arrival_radius_m&&
            (i.personal.proposal.order.stance==Stance::crouch||i.personal.proposal.order.stance==Stance::stand)){
            auto o=i.personal.proposal.order;const auto facing=fight_order();o.look=facing.look;o.fire=facing.fire;
            // Re-prove the current shoulder instead of carrying a previous
            // frame's lean through fresh moving geometry.
            o.lean=0;o.lean_edge={};o.expose={};pose(o,r.here);
            if(r.lean_static_verified){r.proposal=i.personal.proposal;r.proposal.order=o;trace();
                r.lean_gate=o.stance==Stance::crouch?"crouch_walk_lean_selected":"walk_lean_selected";
                return finish(OutdoorSoloReason::seek_cover,true);}
            // Missing/wrong-pose geometry cannot turn this moving order into
            // the stationary peek candidate below. Retire an old outdoor lean
            // without changing its route, requested stance, look or fire.
            return finish(OutdoorSoloReason::personal_layer,retire_walk_lean());
        }
        if(f&&!seen&&i.lean_capture){auto o=fight_order();o.kind=OrderKind::peek;o.role=roles::individual;o.pace=Pace::stand;
            pose(o,r.here);if(r.lean_static_verified){close_fire(o,w.now_ms);emit(o);trace();return finish(OutdoorSoloReason::seek_cover,true);}}
        const auto*near_cover=choose([&](const Choice&v){const float d=flat2(v.spot->position,a.position);return rank(v.cover)&&d>baseline.arrival_radius_m*baseline.arrival_radius_m&&d<=p.open_to_cover_m*p.open_to_cover_m;});
        bool open_with_cover=rank(r.here)==0&&near_cover;
        const float hold_scale=p.combat.enabled?p.combat.meter_hold_scale[m.meter_level]:1.f;
        const TimeMs hold_window=scale_time(s.hold_window,hold_scale);r.effective_hold_ms=hold_window;
        // A measured shoulder opening wins before timed stand/shoot or a
        // cover-cell change. Keep the actually attained stance; a probabilistic
        // cover class must not force a new pose and veto this current eye proof.
        if(f&&i.lean_capture&&!reloading){auto o=fight_order();o.kind=OrderKind::peek;o.role=roles::individual;o.pace=Pace::stand;
            pose(o,r.here);if(r.lean_static_verified){emit(o);trace();return finish(OutdoorSoloReason::seek_cover,true);}}
        // A captured eye can price a route to an existing cover-query edge
        // even while the animation has no attained stand/crouch receipt. The
        // translated eye below is hypothetical destination geometry only:
        // neither it nor the cover point authorizes lean, sight or fire.
        const auto* approach=i.lean_capture;
        const auto& personal_order=i.personal.proposal.order;
        const bool resting=personal_order.pace==Pace::stand||
            (personal_order.pace==Pace::walk&&flat2(personal_order.goal_position,a.position)<=personal_order.arrival_radius_m*personal_order.arrival_radius_m);
        if(f&&approach&&!approach->edge_measured&&source_current(approach->source,w,a)&&
            (approach->pose_only||(approach->source.focus==f->id&&approach->source.focus_at_ms==fact->at_ms&&eq(approach->source.focus_position,fact->position)))&&
            fresh(fact->at_ms,w.now_ms,c.memory_max_age_ms)&&finite(approach->measured_eye)&&svc.rays&&
            resting&&!reloading&&p.combat.enabled){
            // Inspect the cheapest eligible query spot, keeping the existing
            // shared query quota and cover cost/reservation/navigation gates.
            const auto* candidate=choose([&](const Choice&v){const auto d=flat2(v.spot->position,a.position);
                return d>baseline.arrival_radius_m*baseline.arrival_radius_m&&d<=p.open_to_cover_m*p.open_to_cover_m;});
            const float length=std::sqrt(a.forward.x*a.forward.x+a.forward.z*a.forward.z);
            const float reach=(std::min)(c.lean_eye_offset_m_stand,c.lean_eye_offset_m_crouch);
            if(candidate&&pos(length)&&pos(reach)){
                Vec3 aim=fact->position;aim.y+=p.combat.target_height_m;
                auto clear=[&](Vec3 from,Vec3 to){query();Segment ray{from,to};float hit=-2;svc.rays(&ray,&hit,1);poll();
                    return hit==-1.f?Evidence::yes:std::isfinite(hit)&&hit>=0?Evidence::no:Evidence::unknown;};
                const auto& point=candidate->spot->position;
                const Vec3 eye{approach->measured_eye.x+point.x-a.position.x,approach->measured_eye.y+point.y-a.position.y,approach->measured_eye.z+point.z-a.position.z};
                const bool centre_blocked=lean_geometry_checked?lean_centre_blocked:clear(approach->measured_eye,aim)==Evidence::no;
                if(centre_blocked&&clear(eye,aim)==Evidence::no){
                    const Vec3 right{a.forward.z/length,0,-a.forward.x/length};
                    for(float sign:{-1.f,1.f}){Vec3 tip{eye.x+right.x*reach*sign,eye.y,eye.z+right.z*reach*sign};
                        if(clear(eye,tip)!=Evidence::yes||clear(tip,aim)!=Evidence::yes)continue;
                        use_spot(*candidate,OrderKind::cover,roles::seek_cover,m.meter_level?Pace::run:Pace::walk,false);
                        auto& o=r.proposal.order;o.lean=0;o.lean_edge={};o.expose={};close_fire(o,w.now_ms);
                        r.lean_gate="approach_cover_edge";return finish(OutdoorSoloReason::seek_cover,true);
                    }
                    r.lean_gate="cover_edge_unavailable";
                }
            }
        }
        if(p.combat.enabled&&open_with_cover){
            const auto*proof=i.aim_evidence;
            r.earned_shot=own_fire&&live&&proof&&source_current(proof->source,w,a)&&proof->source.focus==f->id&&
                proof->source.focus_at_ms==fact->at_ms&&eq(proof->source.focus_position,fact->position)&&finite(proof->measured_eye)&&
                proof->applied_order_seq==a.last_applied.order_seq&&(a.last_applied.weapon==1||a.last_applied.weapon==2)&&
                i.source.applied->look.kind==LookKind::enemy&&i.source.applied->look.target==f->id.actor&&
                proof->weapon_line_clear==Evidence::yes&&proof->teammate_line_clear==Evidence::yes;
            const float exposure=here?here->lo:1.f;
            r.stay_cost_m=exposure*c.cover_exposure_cost_m+(r.here==OutdoorCover::unknown?c.cover_unknown_cost_m:0)+(1-hold_scale)*c.cover_exposure_cost_m;
            const float opportunity=r.earned_shot?p.combat.shot_opportunity_cost_m*std::max(0.f,1-dist/p.combat.cqb_m)*hold_scale:0;
            r.move_cost_m=near_cover->cost+opportunity;r.cqb_cost_compared=true;
            open_with_cover=r.move_cost_m<r.stay_cost_m;
        }
        // Close combat owns aim/fire, not a permanent stay-at-current-node order.
        // A measured local cover alternative remains subject to the existing
        // reservation and navigation publication checks.
        if(reason==OutdoorPersonalReason::dogfight&&open_with_cover){
            use_spot(*near_cover,OrderKind::cover,roles::seek_cover,Pace::walk,true);
            return finish(OutdoorSoloReason::seek_cover,true);
        }
        if(f&&seen&&canfire&&eff>0&&dist<=1.25f*eff&&!open_with_cover){
            const bool facing_known=finite(seen->facing)&&distance_squared(seen->facing,Vec3{})>0;
            const auto visible_for=w.now_ms-s.visible_since;
            if((hold_window&&((facing_known&&!seen->facing_me&&(!p.combat.enabled||!m.meter_level))||visible_for<hold_window/1.5||
                (visible_for<=hold_window&&(r.here==OutdoorCover::low||r.here==OutdoorCover::chest))))){
                auto o=fight_order();o.kind=OrderKind::hold;o.role=roles::shoot;o.pace=Pace::stand;
                // Stationary uncommitted room fighting can keep its authored lean.
                if(flat2(i.personal.proposal.order.goal_position,a.position)<=p.cover_stay_m*p.cover_stay_m){o.lean=i.personal.proposal.order.lean;o.lean_edge=i.personal.proposal.order.lean_edge;o.expose=i.personal.proposal.order.expose;}
                pose(o,r.here);emit(o);trace();return finish(OutdoorSoloReason::stand_and_shoot,true);
            }
        }
        if(f&&seen&&canfire&&dist>p.distant_m&&!open_with_cover){
            bool active=s.reason==OutdoorSoloReason::shoot_distant&&w.now_ms<s.distant_until&&a.health>p.dying_hp;
            if(!active&&w.now_ms>=s.distant_next&&a.health>=p.injured_hp){const auto cycle=scale_time(p.distant_cycle_ms,sample(p.distant_jitter_min,p.distant_jitter_max));
                s.distant_next=saturated_after(w.now_ms,cycle);s.distant_until=saturated_after(w.now_ms,cycle/3);active=true;}
            if(active){auto o=fight_order();o.kind=OrderKind::hold;o.role=roles::shoot;o.pace=Pace::stand;pose(o,r.here);emit(o);trace();return finish(OutdoorSoloReason::shoot_distant,true);}
        }

        // If the measured centre is blocked and neither shoulder clears, try
        // the existing local edge radius. This is a priced navigation proposal,
        // not an accepted route or permission to fire at remembered enemies.
        const auto*edge=i.lean_capture;
        if(f&&edge&&!edge->edge_measured&&source_current(edge->source,w,a)&&
            (edge->pose_only||(edge->source.focus==f->id&&edge->source.focus_at_ms==fact->at_ms&&eq(edge->source.focus_position,fact->position)))&&
            fresh(fact->at_ms,w.now_ms,c.memory_max_age_ms)&&finite(edge->measured_eye)&&svc.rays&&
            !reloading&&p.combat.enabled&&(!i.coordinated_side||i.personal.proposal.order.pace==Pace::stand)){
            const float length=std::sqrt(a.forward.x*a.forward.x+a.forward.z*a.forward.z);
            if(pos(length)){const Vec3 right{a.forward.z/length,0,-a.forward.x/length};Vec3 aim=fact->position;aim.y+=p.combat.target_height_m;
                auto clear=[&](Vec3 from,Vec3 to){query();Segment ray{from,to};float hit=-2;svc.rays(&ray,&hit,1);poll();
                    return hit==-1.f?Evidence::yes:std::isfinite(hit)&&hit>=0?Evidence::no:Evidence::unknown;};
                const bool centre_blocked=lean_geometry_checked?lean_centre_blocked:clear(edge->measured_eye,aim)==Evidence::no;
                if(centre_blocked){for(float sign:{-1.f,1.f}){const float step=p.combat.lean_edge_m;
                    Vec3 point{a.position.x+right.x*step*sign,a.position.y,a.position.z+right.z*step*sign};
                    Vec3 shifted{edge->measured_eye.x+right.x*step*sign,edge->measured_eye.y,edge->measured_eye.z+right.z*step*sign};
                    if(clear(edge->measured_eye,shifted)!=Evidence::yes||clear(shifted,aim)!=Evidence::yes)continue;
                    const auto previous=r.proposal;
                    if(project(point,p.combat.lean_edge_m,OrderKind::peek,roles::individual,Pace::walk)){
                        const auto&goal=r.proposal.order.goal_position;
                        const Vec3 snapped_eye{edge->measured_eye.x+goal.x-a.position.x,edge->measured_eye.y+goal.y-a.position.y,edge->measured_eye.z+goal.z-a.position.z};
                        if(eq(goal,point)||(clear(edge->measured_eye,snapped_eye)==Evidence::yes&&clear(snapped_eye,aim)==Evidence::yes)){
                            close_fire(r.proposal.order,w.now_ms);r.lean_gate="earn_shot_step";return finish(OutdoorSoloReason::seek_cover,true);}
                        r.proposal=previous;
                    }
                }r.lean_gate="edge_step_unavailable";}
            }
        }
        if(f&&m.meter_level<3){
            if(!i.coordinated_side&&!f->id.ghost()&&profile.can_rush&&a.health>p.dying_hp&&ammo>=p.rush_ammo&&f->reloading.active(w.now_ms)){
                const float range=a.health>=p.injured_hp?p.rush_run_m:p.rush_m;
                if(dist<range&&project(fact->position,range,OrderKind::rush,roles::individual,seen?Pace::walk:Pace::run))return finish(OutdoorSoloReason::rush,true);
            }
            // A throw request is possible only with a producer's matching safety
            // proof and native item/serial. Proposal is never a release receipt.
            const auto*g=i.grenade_throw;
            const bool unseen_long=!seen&&(!f->sources[0].valid||fresh(f->sources[0].at_ms,w.now_ms,c.knowledge.forget_ms)&&w.now_ms-f->sources[0].at_ms>=p.throw_unseen_ms);
            const bool seen_source=fact==&f->sources[0]||fact==&f->sources[2];
            const bool source_recent=fresh(fact->at_ms,w.now_ms,seen_source?p.throw_seen_ms:p.throw_heard_ms);
            if(g&&!f->id.ghost()&&unseen_long&&source_recent&&w.now_ms>=s.throw_next&&dist>=p.throw_min_m&&dist<=p.throw_max_m&&
                source_current(g->source,w,a)&&g->source.focus==f->id&&g->source.focus_at_ms==fact->at_ms&&eq(g->source.focus_position,fact->position)&&
                g->protected_start==Evidence::yes&&g->trajectory_supported==Evidence::yes&&g->teammates_safe==Evidence::yes&&g->serial&&g->item==a.grenades.slot_item&&a.grenades.count&&
                g->item!=invalid_id&&!(live&&i.source.applied->pace==Pace::run)){
                bool safe=i.source.reservations&&i.source.reservations->level_epoch==w.level_epoch&&i.source.reservations->snapshot_seq==w.sequence;
                if(safe)for(const auto&mate:i.source.reservations->members){work();if(mate.actor!=a.key&&(!finite(mate.position)||flat2(mate.position,fact->position)<p.throw_friend_m*p.throw_friend_m))safe=false;}
                if(safe){auto o=fight_order();o.kind=OrderKind::hold;o.pace=Pace::stand;o.throw_order.valid=true;o.throw_order.item=g->item;o.throw_order.serial=g->serial;o.throw_order.target=fact->position;close_fire(o,w.now_ms);
                    emit(o);trace();s.throwing=true;s.throw_order=o;s.throw_trace=r.proposal.knowledge;s.throw_until=saturated_after(w.now_ms,p.throw_pending_ms);r.throw_pending=true;return finish(OutdoorSoloReason::throw_grenade,true);}
            }
            if(!i.coordinated_side&&side.members.size()==1&&!seen&&!(s.searched_foe==f->id&&s.searched_at>=fact->at_ms)){
                const auto last_seen=f->sources[0].valid?f->sources[0].at_ms:fact->at_ms;
                const bool quiet=!(m.has_hit&&fresh(m.last_hit_ms,w.now_ms,s.search_delay))&&!(m.fresh_fire.valid&&fresh(m.fresh_fire.at_ms,w.now_ms,s.search_delay));
                if(last_seen<=w.now_ms&&w.now_ms-last_seen>=s.search_delay&&quiet){
                    if(flat2(a.position,fact->position)<=p.search_reach_m*p.search_reach_m){s.searched_foe=f->id;s.searched_at=fact->at_ms;r.searched=true;r.searched_foe=f->id;r.searched_source_ms=fact->at_ms;}
                    else if(project(fact->position,(std::numeric_limits<float>::max)(),OrderKind::search,roles::search,Pace::walk)){
                        if(m.meter<=0)r.proposal.order.speed_scale=p.sneaky_speed;return finish(OutdoorSoloReason::search,true);}
                }
            }
        }
        if(!f&&a.room_id!=invalid_room&&a.building_id!=invalid_room){const LooseContact*last=nullptr;
            // Gunshot investigation/watch belongs to the side movement layer.
            // Repeated shots must not restart the generic 10-120 s freeze.
            for(const auto&v:m.contacts){work();if(v.kind==SoundKind::gunshot||!v.from_peace||v.from_report||!finite(v.position)||!valid_key(v.reporter)||v.reporter!=a.key||
                !fresh(v.at_ms,w.now_ms,p.freeze_heard_ms)||flat2(v.position,a.position)>p.freeze_m*p.freeze_m)continue;
                if(!last||v.at_ms>last->at_ms)last=&v;}
            if(last&&!(m.has_seen&&fresh(m.last_seen_ms,w.now_ms,p.freeze_seen_ms))){
                if(s.freeze_source!=last->at_ms){s.freeze_source=last->at_ms;s.freeze_point=last->position;s.freeze_until=saturated_after(w.now_ms,
                    scale_time(p.freeze_min_ms+(TimeMs)(double(p.freeze_max_ms-p.freeze_min_ms)*random_unit(s.rng)),1.f/profile.aggression));}
                if(w.now_ms<s.freeze_until){auto o=baseline;o.look.kind=LookKind::point;o.look.value=s.freeze_point;o.look.target={};close_fire(o,w.now_ms);emit(o);return finish(OutdoorSoloReason::freeze,true);}
            }
        }
        if(!i.coordinated_side&&f&&profile.can_shift&&m.meter_level<2){
            if(s.reason==OutdoorSoloReason::shift_cover&&live&&w.now_ms<s.shift_until&&i.source.applied->goal_node==s.shift_node&&eq(i.source.applied->goal_position,s.shift_goal)&&
                flat2(a.position,s.shift_goal)>baseline.arrival_radius_m*baseline.arrival_radius_m){
                const auto*v=choose([&](const Choice&v){return v.spot->node==s.shift_node&&eq(v.spot->position,s.shift_goal);});
                if(v){use_spot(*v,OrderKind::cover,roles::seek_cover,Pace::walk,true);return finish(OutdoorSoloReason::shift_cover,true);}
            }
            const TimeMs last=f->sources[0].valid?f->sources[0].at_ms:fact->at_ms;
            if(here&&rank(r.here)&&!seen&&last<=w.now_ms&&w.now_ms-last>p.shift_stale_ms&&w.now_ms>=s.shift_next&&
                s.decision_at<=w.now_ms&&w.now_ms-s.decision_at>=p.shift_min_ms){
                const auto*v=choose([&](const Choice&v){if(rank(v.cover)<=rank(r.here)||flat2(a.position,v.spot->position)<=p.shift_min_m*p.shift_min_m)return false;
                    for(auto n:s.used){work();if(n==v.spot->node)return false;}return true;});s.shift_next=saturated_after(w.now_ms,p.shift_gap_ms);
                if(v){s.shift_node=v->spot->node;s.shift_goal=v->spot->position;s.shift_until=saturated_after(w.now_ms,p.shift_reset_ms);
                    use_spot(*v,OrderKind::cover,roles::seek_cover,Pace::walk,true);return finish(OutdoorSoloReason::shift_cover,true);}
            }
        }
        if(!i.coordinated_side&&f&&!f->id.ghost()&&m.meter_level<2&&eff>0&&!seen&&f->sources[0].valid&&f->sources[0].reporter==a.key&&
            f->sources[0].at_ms<=w.now_ms&&w.now_ms-f->sources[0].at_ms>=p.mte_unseen_ms&&
            !(m.fresh_fire.valid&&fresh(m.fresh_fire.at_ms,w.now_ms,p.mte_fire_ms))&&dist>(s.reason==OutdoorSoloReason::move_to_engage?p.mte_keep:1.f)*eff){
            const auto*v=choose([&](const Choice&v){const auto d=flat2(a.position,v.spot->position);return rank(v.cover)&&d>=p.mte_min_m*p.mte_min_m&&d<=p.mte_max_m*p.mte_max_m&&flat2(v.spot->position,fact->position)<dist*dist;});
            if(v){use_spot(*v,OrderKind::bound,roles::mover,Pace::run,false);return finish(OutdoorSoloReason::move_to_engage,true);}
            const float step=(std::min)(p.mte_max_m,(std::max)(0.f,dist-p.mte_keep*eff));
            if(step>=p.mte_min_m){const Vec3 point{a.position.x+(fact->position.x-a.position.x)*step/dist,a.position.y,a.position.z+(fact->position.z-a.position.z)*step/dist};
                if(project(point,p.mte_max_m,OrderKind::bound,roles::mover,Pace::run))return finish(OutdoorSoloReason::move_to_engage,true);}
        }
        if(i.coordinated_side)return finish(OutdoorSoloReason::commitment);
        if(f){const auto*v=choose([&](const Choice&v){return v.cover!=OutdoorCover::open;});
            if(v){use_spot(*v,OrderKind::cover,roles::seek_cover,seen?Pace::walk:Pace::run,true);return finish(OutdoorSoloReason::seek_cover,true);}
            return finish(OutdoorSoloReason::cover_unavailable);}
        return finish(OutdoorSoloReason::unchanged);
    }catch(const Budget&){r={};r.proposal=i.personal.proposal;r.status=OutdoorSoloStatus::pending;r.reason=OutdoorSoloReason::budget;r.lean_gate="budget";
        const auto& o=r.proposal.order;
        const bool pose_commitment=(i.genuine_commitment||i.personal.reason==OutdoorPersonalReason::committed_bound)&&
            (i.personal.reason==OutdoorPersonalReason::personal_contact||i.personal.reason==OutdoorPersonalReason::squad||
             i.personal.reason==OutdoorPersonalReason::continue_cover||i.personal.reason==OutdoorPersonalReason::committed_bound)&&
            (o.role==roles::base_of_fire||o.kind==OrderKind::bound)&&(o.pace==Pace::stand||o.pace==Pace::walk)&&
            !o.throw_order.valid&&!valid_key(o.aid)&&!(o.flags&order_committed_entry);
        if(i.personal.status==OutdoorPersonalStatus::ready&&(pose_commitment||(!i.genuine_commitment&&
            i.personal.reason!=OutdoorPersonalReason::committed_room&&i.personal.reason!=OutdoorPersonalReason::committed_bound&&
            i.personal.reason!=OutdoorPersonalReason::hands&&o.pace==Pace::walk)))r.changed=retire_walk_lean();}
    catch(const Cancelled&){r={};r.proposal=i.personal.proposal;r.status=OutdoorSoloStatus::cancelled;}
    catch(...){r={};r.proposal=i.personal.proposal;r.status=OutdoorSoloStatus::failed;}
    counts();return r;
}
}}}
