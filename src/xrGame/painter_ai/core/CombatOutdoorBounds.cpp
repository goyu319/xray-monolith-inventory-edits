#include "CombatOutdoorBounds.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <limits>

namespace painter { namespace combat { namespace core {
namespace {
bool pos(float v){return std::isfinite(v)&&v>0;}
bool nonnegative(float v){return std::isfinite(v)&&v>=0;}
float flat(Vec3 a,Vec3 b){return std::hypot(a.x-b.x,a.z-b.z);}
OutdoorStamp capture(const WorldSnapshot&w,const SideSnapshot&s){return {w.level_epoch,w.map_hash,w.geometry_revision,w.sequence,s.group,w.now_ms,s.side_id,s.faction,s.mission.revision};}
bool equal(const OutdoorStamp&a,const OutdoorStamp&b){return a.level_epoch==b.level_epoch&&a.map_hash==b.map_hash&&a.geometry_revision==b.geometry_revision&&
    a.snapshot_seq==b.snapshot_seq&&a.group==b.group&&a.at_ms==b.at_ms&&a.side==b.side&&a.faction==b.faction&&a.mission_revision==b.mission_revision;}
bool same_owner(const OutdoorStamp&a,const OutdoorStamp&b){return a.level_epoch==b.level_epoch&&a.map_hash==b.map_hash&&a.geometry_revision==b.geometry_revision&&
    a.group==b.group&&a.side==b.side&&a.faction==b.faction&&a.mission_revision==b.mission_revision;}
bool policy_ok(const OutdoorBoundPolicy&p,const Config&c){return p.fired_recent_ms&&pos(p.bound_min_m)&&p.bound_max_m>=p.bound_min_m&&std::isfinite(p.bound_max_m)&&
    pos(p.flank_min_m)&&p.flank_max_m>=p.flank_min_m&&std::isfinite(p.flank_max_m)&&pos(p.flank_min_deg)&&p.flank_min_deg<=p.flank_preferred_deg&&p.flank_preferred_deg<=180&&
    pos(p.flank_cost_m)&&pos(p.arrival_m)&&p.final_walk_m>=p.arrival_m&&std::isfinite(p.final_walk_m)&&p.sprint_distance_m>=p.final_walk_m&&std::isfinite(p.sprint_distance_m)&&
    p.aimed_speed>=.3f&&p.aimed_speed<=1.5f&&p.run_speed>=.3f&&p.run_speed<=1.5f&&p.sprint_speed>=.3f&&p.sprint_speed<=1.5f&&pos(p.fire_line_clearance_m)&&
    nonnegative(c.minimum_goal_separation)&&nonnegative(c.cover_exposure_cost_m)&&nonnegative(c.cover_unknown_cost_m)&&nonnegative(c.cover_far_cost_m_per_m)&&
    nonnegative(c.immediate_cover_distance)&&c.order_lifetime_ms;}
bool able(const ActorSnapshot&a,const WorldSnapshot&w,const Config&c){return a.level_epoch==w.level_epoch&&a.snapshot_seq==w.sequence&&a.observed_at_ms==w.now_ms&&
    fresh(a.observed_at_ms,w.now_ms,c.snapshot_max_age_ms)&&a.ownership_generation&&(a.flags&flag_admitted)&&!a.yield_reasons&&a.alive&&!a.wounded&&
    std::isfinite(a.health)&&a.health>0&&finite(a.position)&&a.node_id!=invalid_node&&a.current_position_accessible&&a.weapon.id!=invalid_id&&
    (a.weapon.ammo_in_mag||a.weapon.ammo_reserve);}
bool actual(const Order&o,const ActorSnapshot&a,const WorldSnapshot&w){return o.actor==a.key&&o.level_epoch==w.level_epoch&&o.geometry_revision==w.geometry_revision&&
    o.snapshot_seq&&o.snapshot_seq<=w.sequence&&o.ownership_generation==a.ownership_generation&&o.restrictions_revision==a.restrictions_revision&&o.order_seq&&
    o.order_seq==a.last_applied.order_seq&&o.issued_at_ms<=a.last_applied.at_ms&&a.last_applied.at_ms<=a.observed_at_ms&&a.accessible_current_order&&
    !(a.last_rejected.order_seq==o.order_seq&&a.last_rejected.reason!=reject_none)&&
    o.expires_at_ms>w.now_ms&&o.goal_node!=invalid_node&&finite(o.goal_position)&&pos(o.arrival_radius_m)&&o.kind!=OrderKind::yield;}
bool proof_order(const Order&a,const Order&b){return a.actor==b.actor&&a.level_epoch==b.level_epoch&&a.geometry_revision==b.geometry_revision&&a.snapshot_seq==b.snapshot_seq&&
    a.order_seq==b.order_seq&&a.ownership_generation==b.ownership_generation&&a.restrictions_revision==b.restrictions_revision&&a.kind==b.kind&&
    a.goal_node==b.goal_node&&distance_squared(a.goal_position,b.goal_position)==0&&a.arrival_radius_m==b.arrival_radius_m&&a.issued_at_ms==b.issued_at_ms&&a.expires_at_ms==b.expires_at_ms;}
bool committed_other(const Order&o){return o.assignment.portal||o.kind==OrderKind::recover||o.kind==OrderKind::withdraw||o.kind==OrderKind::entry||o.kind==OrderKind::stack||
    o.kind==OrderKind::peek||o.throw_order.valid||valid_key(o.aid)||valid_key(o.finish);}
bool fact_equal(const SourceFact&a,const SourceFact&b){return a.valid&&b.valid&&a.at_ms==b.at_ms&&a.reporter==b.reporter&&a.sound==b.sound&&finite(a.position)&&
    finite(b.position)&&distance_squared(a.position,b.position)==0;}
bool mover_focus(const ActorSnapshot&a,const WorldSnapshot&w,const MemberKnowledge&m,const OutdoorPostureResult&p,KnowledgeTrace&trace){
    // The leader knowing a source does not grant it to every mover. Use the
    // mover's own accepted K1 record, including an actually delivered report.
    for(const auto&foe:m.foes){if(foe.id!=p.focus||foe.down||foe.wounded)continue;const auto*fact=foe.last_known();
        if(!fact||!fact->valid||!valid_key(fact->reporter)||fact->at_ms!=p.focus_fact.at_ms||!finite(fact->position)||distance_squared(fact->position,p.focus_fact.position)!=0)return false;
        trace={};trace.receiver=a.key;trace.target=p.focus.actor;trace.observer=fact->reporter;trace.position=fact->position;trace.observed_at_ms=fact->at_ms;
        trace.shared=fact->reporter!=a.key;trace.receiver_ownership_generation=a.ownership_generation;trace.receiver_faction=a.faction;trace.foe=p.focus;
        auto own=[&](const Contact&c){return c.key==p.focus.actor&&c.hostile&&c.alive_when_observed&&!c.wounded&&personally_visible(c,a)&&
            c.observed_at_ms>=fact->at_ms&&finite(c.position)&&distance_squared(c.position,fact->position)==0;};
        if(own(a.enemy)){trace.personally_visible=true;trace.shared=false;trace.observer=a.key;trace.source_snapshot_seq=w.sequence;}
        for(std::size_t i=0;i<std::min<std::size_t>(a.visible_enemies.count,a.visible_enemies.items.size());++i)if(own(a.visible_enemies.items[i])){
            trace.personally_visible=true;trace.shared=false;trace.observer=a.key;trace.source_snapshot_seq=w.sequence;}
        return true;
    }
    return false;
}
OutdoorCover cover_class(const CoverSpot&c){if(c.protection_known!=Evidence::yes)return OutdoorCover::unknown;
    // This helper is reached only after the producer's standard profile is checked.
    if(c.blocked[2])return OutdoorCover::full;
    if(c.blocked[1])return OutdoorCover::chest;
    return c.blocked[0]?OutdoorCover::low:OutdoorCover::open;}
float point_segment(Vec3 p,Vec3 a,Vec3 b){const float x=b.x-a.x,z=b.z-a.z,d=x*x+z*z;
    const float t=d>0?std::clamp(((p.x-a.x)*x+(p.z-a.z)*z)/d,0.f,1.f):0;return std::hypot(p.x-a.x-t*x,p.z-a.z-t*z);}
float cross(Vec3 a,Vec3 b,Vec3 c){return (b.x-a.x)*(c.z-a.z)-(b.z-a.z)*(c.x-a.x);}
float segment_distance(Vec3 a,Vec3 b,Vec3 c,Vec3 d){
    if(((cross(a,b,c)>0&&cross(a,b,d)<0)||(cross(a,b,c)<0&&cross(a,b,d)>0))&&
        ((cross(c,d,a)>0&&cross(c,d,b)<0)||(cross(c,d,a)<0&&cross(c,d,b)>0)))return 0;
    return std::min({point_segment(a,c,d),point_segment(b,c,d),point_segment(c,a,b),point_segment(d,a,b)});}
float angle(Vec3 a,Vec3 b,Vec3 origin){const float ax=a.x-origin.x,az=a.z-origin.z,bx=b.x-origin.x,bz=b.z-origin.z;
    const float scale=std::hypot(ax,az)*std::hypot(bx,bz);return scale>0?std::acos(std::clamp((ax*bx+az*bz)/scale,-1.f,1.f))*180.f/3.14159265358979323846f:0;}
}
CombatOutdoorBounds::CombatOutdoorBounds(Config c,OutdoorBoundPolicy p):config_(std::move(c)),policy_(p){}
void CombatOutdoorBounds::reset(){stamp_={};fired_.clear();recovery_={};recovery_stamp_={};recovery_focus_={};recovery_focus_position_={};}
OutdoorBoundsResult CombatOutdoorBounds::update(const OutdoorBoundsRequest&q,const std::function<bool()>&cancelled){
    OutdoorBoundsResult invalid;bool interrupted=false;auto stop=[&](){if(!interrupted&&cancelled)interrupted=cancelled();return interrupted;};
    auto cancelled_result=[&](){OutdoorBoundsResult r;r.status=OutdoorBoundStatus::cancelled;r.reason="cancelled";r.stamp=invalid.stamp;return r;};
    try {
        if(stop())return cancelled_result();
        if(!q.world||!q.side||!q.knowledge||!q.posture||!q.execution||!q.elements||!q.accepted_orders||!q.cover||!policy_ok(policy_,config_))return invalid;
        const auto&w=*q.world;const auto&s=*q.side;const auto&k=*q.knowledge;const auto&p=*q.posture;const auto&e=*q.execution;const auto&g=*q.elements;
        invalid.stamp=capture(w,s);
        if(!w.level_epoch||!w.sequence||!s.side_id||!equal(p.stamp,invalid.stamp)||p.status!=OutdoorStatus::ready||!equal(e.stamp,invalid.stamp)||
            !k.accepted||k.cancelled||k.level_epoch!=w.level_epoch||k.sequence!=w.sequence||g.side_id!=s.side_id||g.group_id!=s.side_id||g.faction!=s.faction||
            g.leader!=p.leader||s.mission.rooms.count>s.mission.rooms.items.size())return invalid;
        if(stamp_.level_epoch&&(w.level_epoch<stamp_.level_epoch||(w.level_epoch==stamp_.level_epoch&&(w.sequence<=stamp_.snapshot_seq||w.now_ms<stamp_.at_ms)))){invalid.reason="stale_capture";return invalid;}
        const bool continuity=same_owner(stamp_,invalid.stamp);
        std::map<std::uint32_t,const ActorSnapshot*> actors;std::map<std::uint32_t,const MemberKnowledge*> members;
        std::map<std::uint32_t,const Order*> accepted;std::map<std::uint32_t,const OutdoorActorExecution*> proof;
        std::map<std::uint32_t,const ActorCoverCache*> caches;std::set<std::uint32_t> roster,active,bases,movers;
        std::map<std::uint32_t,const OutdoorCoverProfile*> profiles;
        for(const auto&a:w.actors){if(stop())return cancelled_result();if(!valid_key(a.key)||!actors.emplace(a.key.id,&a).second)return invalid;}
        for(const auto&m:k.members){if(stop())return cancelled_result();if(!valid_key(m.actor)||!members.emplace(m.actor.id,&m).second)return invalid;}
        for(auto key:s.members){if(stop())return cancelled_result();if(!valid_key(key)||!roster.insert(key.id).second)return invalid;auto a=actors.find(key.id);
            if(a!=actors.end()&&(a->second->key!=key||a->second->faction!=s.faction))return invalid;}
        for(auto key:g.members){if(stop())return cancelled_result();auto a=actors.find(key.id);
            if(!roster.count(key.id)||a==actors.end()||a->second->key!=key||!active.insert(key.id).second)return invalid;}
        auto element=[&](const std::vector<ActorKey>& list,std::set<std::uint32_t>& out){for(auto key:list){if(stop())return false;auto a=actors.find(key.id);
            if(!active.count(key.id)||a==actors.end()||a->second->key!=key||!out.insert(key.id).second)return false;}return true;};
        if(!element(g.base,bases)||!element(g.movers,movers)){if(stop())return cancelled_result();return invalid;}
        for(auto id:bases)if(movers.count(id))return invalid;
        for(const auto&o:*q.accepted_orders){if(stop())return cancelled_result();if(!valid_key(o.actor)||!accepted.emplace(o.actor.id,&o).second)return invalid;}
        for(const auto&x:e.actors){if(stop())return cancelled_result();if(!valid_key(x.applied.actor)||!roster.count(x.applied.actor.id)||!proof.emplace(x.applied.actor.id,&x).second)return invalid;}
        for(const auto&c:*q.cover){if(stop())return cancelled_result();if(!valid_key(c.actor)||!caches.emplace(c.actor.id,&c).second)return invalid;}
        if(q.query_evidence)for(const auto&profile:q.query_evidence->profiles){if(stop())return cancelled_result();
            if(!valid_key(profile.source.actor)||!profiles.emplace(profile.source.actor.id,&profile).second)return invalid;}
        auto knowledge=[&](const ActorSnapshot&a)->const MemberKnowledge*{auto m=members.find(a.key.id);if(m==members.end()||m->second->actor!=a.key||
            m->second->side!=s.side_id||m->second->ownership_generation!=a.ownership_generation||!m->second->alive||!m->second->able)return nullptr;return m->second;};
        auto current=[&](const ActorSnapshot&a)->const Order*{auto o=accepted.find(a.key.id);return o!=accepted.end()&&actual(*o->second,a,w)?o->second:nullptr;};
        auto evidence=[&](const ActorSnapshot&a,const Order&o)->const OutdoorActorExecution*{auto f=proof.find(a.key.id);return f!=proof.end()&&proof_order(f->second->applied,o)?f->second:nullptr;};
        const auto leader=actors.find(p.leader.id);const MemberKnowledge* lead=leader!=actors.end()&&leader->second->key==p.leader?knowledge(*leader->second):nullptr;
        bool sourced=false;
        if(p.focus_valid&&p.focus.valid()&&!p.focus.ghost()&&lead&&fresh(p.focus_fact.at_ms,w.now_ms,config_.knowledge.assess_foe_ms))
            for(const auto&f:lead->foes){if(stop())return cancelled_result();if(f.id!=p.focus||f.down||f.wounded)continue;
                for(std::size_t i=0;i<f.sources.size();++i)if(fact_equal(f.sources[i],p.focus_fact)&&(i>=2||f.sources[i].reporter==lead->actor))sourced=true;}
        OutdoorBoundsResult out;out.status=OutdoorBoundStatus::ready;out.stamp=invalid.stamp;out.reason="posture_not_attack";
        auto commit=[&](){stamp_=out.stamp;return out;};
        const bool recovery_context=valid_key(recovery_.actor)&&same_owner(recovery_stamp_,out.stamp)&&p.focus_valid&&
            recovery_focus_==p.focus&&finite(recovery_focus_position_)&&distance_squared(recovery_focus_position_,p.focus_fact.position)<=policy_.arrival_m*policy_.arrival_m;
        if(p.posture!=OutdoorPosture::attack&&!p.support_recovery&&!(recovery_context&&p.posture==OutdoorPosture::defend)){fired_.clear();return commit();}
        if(!sourced){out.reason="unverified_focus";fired_.clear();return commit();}
        // Current own non-hostility wins over an old K1 focus, without consulting
        // the target's global actor row or leaking its true health/position.
        for(auto id:roster){const auto a=actors.find(id);if(a==actors.end())continue;const auto&t=a->second->enemy;
            if(t.key==p.focus.actor&&!t.hostile&&finite(t.position)&&t.observed_at_ms<=a->second->observed_at_ms&&t.observed_at_ms>=p.focus_fact.at_ms){out.reason="current_nonhostile";return commit();}}
        std::size_t able_count=0;for(auto id:roster){auto a=actors.find(id);if(a!=actors.end()&&able(*a->second,w,config_))++able_count;}
        if(able_count<2||bases.empty()||movers.empty()){out.reason="solo_or_no_elements";return commit();}
        std::vector<Fired> next_fired;std::vector<std::pair<Vec3,Vec3>> fire_lines;Vec3 base_centre;
        for(auto id:bases){if(stop())return cancelled_result();const auto&a=*actors.at(id);const auto*km=knowledge(a);const auto*o=current(a);if(!able(a,w,config_)||!km||!o)continue;
            const auto*f=evidence(a,*o);if(!f||f->element!=OutdoorElement::base||f->focus!=p.focus||f->focus_at_ms!=p.focus_fact.at_ms)continue;
            const bool target=(o->look.kind==LookKind::enemy&&o->look.target==p.focus.actor)||(o->fire.at_valid&&finite(o->fire.at)&&distance_squared(o->fire.at,p.focus_fact.position)==0);
            Fired latest;bool has_fired=false;
            if(continuity)for(const auto&old:fired_)if(old.actor==a.key&&old.owner==a.ownership_generation&&old.focus==p.focus&&fresh(old.at_ms,w.now_ms,policy_.fired_recent_ms)){latest=old;has_fired=true;break;}
            if(a.last_applied.fired_since_previous&&target){latest={a.key,a.ownership_generation,p.focus,a.observed_at_ms};has_fired=true;}
            if(has_fired)next_fired.push_back(latest);
            const auto*selected=select_contact(a,config_,w.now_ms);const bool own_focus=selected&&selected->key==p.focus.actor&&personally_visible(*selected,a);
            // Readiness can precede the first shot or persist between bursts;
            // the fired ledger remains observation-only. A point packet must
            // address this exact known focus, never merely have at_valid set.
            const bool point_support=o->fire.at_valid&&finite(o->fire.at)&&distance_squared(o->fire.at,p.focus_fact.position)==0;
            const bool shooting=target&&o->fire.allowed&&o->fire.until_ms>w.now_ms&&!(a.flags&flag_dont_shoot)&&
                (point_support||own_focus)&&(point_support||has_fired);
            if((o->kind!=OrderKind::cover&&o->kind!=OrderKind::hold)||committed_other(*o)||km->meter_level>=3||a.reloading||a.weapon.state==WeaponState::reloading||!a.weapon.ammo_in_mag||
                distance_squared(a.position,o->goal_position)>o->arrival_radius_m*o->arrival_radius_m||f->cover_verified!=Evidence::yes||f->cover==OutdoorCover::unknown||
                f->cover==OutdoorCover::open||(f->cover==OutdoorCover::full&&!o->expose.lean)||f->fire_line!=Evidence::yes||!shooting)continue;
            out.ready_base.push_back(a.key);fire_lines.push_back({a.position,o->fire.at_valid?o->fire.at:p.focus_fact.position});
            base_centre.x+=a.position.x;base_centre.y+=a.position.y;base_centre.z+=a.position.z;
        }
        if(!out.ready_base.empty()){const float n=float(out.ready_base.size());base_centre.x/=n;base_centre.y/=n;base_centre.z/=n;}
        // A source failure's lease limits NEW approaches, not an actually
        // applied leg. Continue the exact authored goal on fresh execution route
        // proof even if the nearest-cover query no longer returns that goal.
        if(recovery_context&&(p.posture==OutdoorPosture::attack||p.posture==OutdoorPosture::defend)&&
            !p.exchange_hold&&e.assessment!=OutdoorAssessment::hold){
            const auto ai=actors.find(recovery_.actor.id);
            if(ai!=actors.end()&&ai->second->key==recovery_.actor){const auto&a=*ai->second;const auto*old=current(a);const auto*km=knowledge(a);
                const auto*proof=old?evidence(a,*old):nullptr;KnowledgeTrace trace;
                if(able(a,w,config_)&&km&&old&&proof&&old->kind==OrderKind::advance&&old->role==roles::travel&&
                    old->ownership_generation==recovery_.ownership_generation&&old->restrictions_revision==recovery_.restrictions_revision&&
                    old->goal_node==recovery_.goal_node&&distance_squared(old->goal_position,recovery_.goal_position)==0&&
                    flat(a.position,old->goal_position)>old->arrival_radius_m&&proof->movement_route==Evidence::yes&&mover_focus(a,w,*km,p,trace)){
                    ProposedOrder proposal;proposal.order=first_response(a,config_,w.now_ms,w.level_epoch,w.geometry_revision,old->order_seq);
                    auto&o=proposal.order;o.kind=OrderKind::advance;o.role=roles::travel;o.goal_node=old->goal_node;o.goal_position=old->goal_position;
                    o.arrival_radius_m=old->arrival_radius_m;o.pace=old->pace;o.stance=old->stance;o.speed_scale=old->speed_scale;o.flags&=~(order_first_response|order_face_movement);
                    if(o.look.kind!=LookKind::enemy){o.look={LookKind::point,p.focus_fact.position,{}};o.fire={};}
                    proposal.group_id=g.group_id;proposal.expected_previous_order_seq=old->order_seq;proposal.has_knowledge=true;proposal.knowledge=trace;
                    out.support_reposition.push_back(std::move(proposal));out.support_reason="continue_approach";
                }
            }
        }
        const bool door=p.door_bound;
        if(door&&(e.door_validated!=Evidence::yes||e.door_node==invalid_node||!e.door_portal||e.door_building==invalid_room||
            !finite(e.door_point)||distance_squared(e.door_point,p.movement_focus)!=0)){out.reason="unverified_door_objective";if(stop())return cancelled_result();fired_=std::move(next_fired);return commit();}
        std::vector<std::pair<ActorKey,Vec3>> occupied;
        for(auto id:roster){if(stop())return cancelled_result();auto a=actors.find(id);if(a==actors.end()||!a->second->alive)continue;occupied.push_back({a->second->key,a->second->position});
            if(const auto*o=current(*a->second))if(o->kind==OrderKind::cover||o->kind==OrderKind::bound)occupied.push_back({a->second->key,o->goal_position});}
        // Ordinary cover selection minimizes exposure for everyone. That can
        // leave the entire base behind opaque cover forever. Form the assigned
        // base at lower-body cover with a measured standing-height aperture;
        // only later actual arrival, eye-line and shot receipts release movers.
        // Continue the one actually applied recovery leg before asking another
        // member to move. Without an applied leg, any evaluated base member may
        // provide the safe alternative; the failure witness is not a monopoly.
        ActorKey recovering;
        for(auto id:bases){const auto&a=*actors.at(id);const auto*o=current(a);
            if(o&&o->kind==OrderKind::advance&&o->role==roles::travel&&flat(a.position,o->goal_position)>o->arrival_radius_m){recovering=a.key;break;}}
        struct Approach {ProposedOrder proposal;};std::vector<Approach> approaches;
        if(out.support_reposition.empty())out.support_reason=p.support_recovery?"no_safe_position":"not_requested";
        for(auto id:bases){
            if(stop())return cancelled_result();const auto&a=*actors.at(id);const auto*km=knowledge(a);const auto*old=current(a);
            if(!able(a,w,config_)||!km||!old||committed_other(*old)||a.reloading||a.weapon.state==WeaponState::reloading||
                !a.weapon.ammo_in_mag||km->meter_level>=3||km->dogfight.valid()||
                std::find(out.ready_base.begin(),out.ready_base.end(),a.key)!=out.ready_base.end())continue;
            if(old->kind==OrderKind::cover&&old->role!=roles::base_of_fire&&
                distance_squared(a.position,old->goal_position)>old->arrival_radius_m*old->arrival_radius_m)continue;
            KnowledgeTrace trace;if(!mover_focus(a,w,*km,p,trace))continue;
            auto ci=caches.find(id);auto pi=profiles.find(id);if(ci==caches.end()||pi==profiles.end())continue;
            const auto&cache=*ci->second;const auto&profile=*pi->second;const auto&source=profile.source;
            if(source.actor!=a.key||source.level_epoch!=w.level_epoch||source.snapshot_seq!=w.sequence||source.geometry_revision!=w.geometry_revision||
                source.ownership_generation!=a.ownership_generation||source.restrictions_revision!=a.restrictions_revision||source.at_ms!=w.now_ms||
                source.origin_node!=a.node_id||!finite(source.actor_position)||distance_squared(source.actor_position,a.position)!=0||source.focus!=p.focus||
                source.focus_at_ms!=p.focus_fact.at_ms||!finite(source.focus_position)||distance_squared(source.focus_position,p.focus_fact.position)!=0||
                profile.heights!=std::array<float,3>{{.7f,1.1f,1.7f}}||cache.actor!=a.key||cache.level_epoch!=w.level_epoch||cache.snapshot_seq!=w.sequence||
                cache.ownership_generation!=a.ownership_generation||cache.restrictions_revision!=a.restrictions_revision||cache.origin_node!=a.node_id||
                cache.observed_at_ms!=w.now_ms||cache.knowledge_threat!=p.focus||cache.knowledge_observed_at_ms!=p.focus_fact.at_ms||
                distance_squared(cache.knowledge_threat_position,p.focus_fact.position)!=0)continue;
            bool evaluated=!cache.spots.empty();if(evaluated)++out.base_evaluated;
            const CoverSpot*reposition=nullptr;float reposition_cost=(std::numeric_limits<float>::max)();
            const CoverSpot*best=nullptr;const CoverSpot*continuation=nullptr;float cost=(std::numeric_limits<float>::max)();
            const bool in_motion=old->kind==OrderKind::cover&&old->role==roles::base_of_fire&&
                distance_squared(a.position,old->goal_position)>old->arrival_radius_m*old->arrival_radius_m;
            for(const auto&spot:cache.spots){
                if(stop())return cancelled_result();++out.examined_candidates;
                if(spot.node==invalid_node||!finite(spot.position)||spot.geometry_revision!=w.geometry_revision||spot.threat!=0||
                    spot.threat_key!=p.focus.actor||spot.threat_observed_at_ms!=p.focus_fact.at_ms||!nonnegative(spot.lo)||spot.lo>1){
                    evaluated=false;++out.base_invalid;++out.rejected_candidates;continue;}
                if(spot.protection_known!=Evidence::yes)evaluated=false;
                if(spot.accessible==Evidence::no||spot.reachable==Evidence::no||
                    ((spot.accessible==Evidence::yes||spot.reachable==Evidence::yes)&&spot.restrictions_revision!=a.restrictions_revision)||
                    (spot.accessible==Evidence::unknown&&spot.reachable==Evidence::unknown&&spot.restrictions_revision!=0)){
                    ++out.base_restricted;++out.rejected_candidates;continue;}
                const float travel=flat(a.position,spot.position);
                // Full cover may be useful for approach even though it cannot
                // form the firing base. Each destination still needs measured
                // protection and a complete CURRENT restricted route.
                if(p.support_recovery&&(!valid_key(recovering)||a.key==recovering)&&
                    (!(old->kind==OrderKind::advance&&old->role==roles::travel&&flat(a.position,old->goal_position)>old->arrival_radius_m)||
                     (old->goal_node==spot.node&&distance_squared(old->goal_position,spot.position)==0))&&spot.protection_known==Evidence::yes&&
                    (spot.blocked[0]||spot.blocked[1])&&travel>policy_.arrival_m&&travel<=policy_.bound_max_m&&
                    flat(a.position,p.focus_fact.position)-flat(spot.position,p.focus_fact.position)>policy_.arrival_m){
                    bool occupied_goal=false;for(const auto&x:occupied)if(x.first!=a.key&&flat(x.second,spot.position)<config_.minimum_goal_separation)occupied_goal=true;
                    if(occupied_goal)++out.recovery_occupied;
                    else {
                        const OutdoorBoundPath*route=nullptr;bool duplicate=false;
                        if(q.paths&&equal(q.path_stamp,out.stamp))for(const auto&path:*q.paths)if(path.actor==a.key&&path.goal_node==spot.node){if(route)duplicate=true;route=&path;}
                        const bool complete=!duplicate&&route&&route->complete==Evidence::yes&&route->owner==a.ownership_generation&&
                            route->restrictions_revision==a.restrictions_revision&&route->origin_node==a.node_id&&
                            distance_squared(route->goal_position,spot.position)==0&&route->points.size()>=2&&
                            distance_squared(route->points.front(),a.position)==0&&distance_squared(route->points.back(),spot.position)==0&&
                            spot.accessible==Evidence::yes&&spot.reachable==Evidence::yes;
                        if(!complete)++out.recovery_pending;
                        else {
                            bool safe=true;for(std::size_t j=0;j<route->points.size();++j){if(stop())return cancelled_result();
                                if(!finite(route->points[j])){safe=false;break;}
                                if(!j)continue;
                                for(const auto&peer:e.actors){if(peer.applied.actor==a.key||!peer.applied.fire.allowed||peer.applied.fire.until_ms<=w.now_ms)continue;
                                    auto pa=actors.find(peer.applied.actor.id);if(pa==actors.end()||!actual(peer.applied,*pa->second,w))continue;
                                    const auto*seen=select_contact(*pa->second,config_,w.now_ms);
                                    Vec3 target;if(peer.applied.fire.at_valid&&finite(peer.applied.fire.at))target=peer.applied.fire.at;
                                    else if(seen&&personally_visible(*seen,*pa->second)&&peer.applied.look.kind==LookKind::enemy&&peer.applied.look.target==seen->key)target=seen->position;
                                    else continue;
                                    if(segment_distance(route->points[j-1],route->points[j],pa->second->position,target)<policy_.fire_line_clearance_m)safe=false;
                                }
                            }
                            if(!safe)++out.recovery_unsafe;
                            else {const float score=travel+spot.lo*config_.cover_exposure_cost_m;
                                if(!reposition||score<reposition_cost||(score==reposition_cost&&spot.node<reposition->node)){reposition=&spot;reposition_cost=score;}}
                        }
                    }
                }
                if(spot.protection_known!=Evidence::yes||!(spot.blocked[0]||spot.blocked[1])||spot.blocked[2]){
                    ++out.base_protection;++out.rejected_candidates;continue;}
                float candidate=travel+spot.lo*config_.cover_exposure_cost_m+
                    std::max(0.f,travel-config_.immediate_cover_distance)*config_.cover_far_cost_m_per_m;
                if(spot.accessible==Evidence::unknown||spot.reachable==Evidence::unknown)candidate+=config_.cover_unknown_cost_m;
                // Sequential soft spacing also applies while assembling base.
                for(const auto&x:occupied)if(x.first!=a.key&&config_.minimum_goal_separation>0){
                    const float gap=flat(x.second,spot.position);if(gap<config_.minimum_goal_separation)
                        candidate+=(1.f-gap/config_.minimum_goal_separation)*(config_.cover_exposure_cost_m+config_.cover_unknown_cost_m+travel);}
                if(in_motion&&spot.node==old->goal_node&&distance_squared(spot.position,old->goal_position)==0)continuation=&spot;
                if(std::isfinite(candidate)&&(!best||candidate<cost||(candidate==cost&&spot.node<best->node))){best=&spot;cost=candidate;}
            }
            if(continuation)best=continuation;
            if(evaluated&&!out.support_failure.completed&&(!best||flat(a.position,best->position)<=policy_.arrival_m)){
                out.support_failure={out.stamp,a.key,a.ownership_generation,a.restrictions_revision,p.focus,p.focus_fact.at_ms,p.focus_fact.position,true};
            }
            if(reposition){ProposedOrder proposal;proposal.expected_previous_order_seq=old->order_seq;proposal.group_id=g.group_id;
                proposal.order=first_response(a,config_,w.now_ms,w.level_epoch,w.geometry_revision,old->order_seq);
                auto&o=proposal.order;o.kind=OrderKind::advance;o.role=roles::travel;o.goal_node=reposition->node;o.goal_position=reposition->position;
                o.arrival_radius_m=policy_.arrival_m;o.pace=Pace::walk;o.stance=Stance::stand;o.speed_scale=policy_.aimed_speed;
                o.flags&=~(order_first_response|order_face_movement);
                if(o.look.kind!=LookKind::enemy){o.look={LookKind::point,p.focus_fact.position,{}};o.fire={};}
                proposal.has_knowledge=true;proposal.knowledge=trace;proposal.has_cover_evidence=true;proposal.cover_evidence=*reposition;
                proposal.cover_cost=reposition_cost;proposal.navigation_pending=false;approaches.push_back({std::move(proposal)});
            }
            if(!best)continue;
            ProposedOrder proposal;proposal.expected_previous_order_seq=old->order_seq;proposal.group_id=g.group_id;
            proposal.order=first_response(a,config_,w.now_ms,w.level_epoch,w.geometry_revision,old->order_seq);
            auto&o=proposal.order;o.role=roles::base_of_fire;o.goal_node=best->node;o.goal_position=best->position;
            o.arrival_radius_m=continuation?old->arrival_radius_m:policy_.arrival_m;
            const bool moving=distance_squared(a.position,best->position)>o.arrival_radius_m*o.arrival_radius_m;
            o.kind=moving?OrderKind::cover:OrderKind::hold;o.pace=moving?Pace::walk:Pace::stand;
            o.stance=Stance::stand;o.speed_scale=policy_.aimed_speed;o.flags&=~(order_first_response|order_face_movement);
            // Preserve current own local defence, even when another foe is the
            // side's objective. Shared/remembered objective grants look only.
            if(o.look.kind!=LookKind::enemy){o.look={LookKind::point,p.focus_fact.position,{}};o.fire={};}
            proposal.has_knowledge=true;proposal.knowledge=trace;proposal.has_cover_evidence=true;proposal.cover_evidence=*best;proposal.cover_cost=cost;
            proposal.navigation_pending=best->accessible!=Evidence::yes||best->reachable!=Evidence::yes;
            out.base_positions.push_back(std::move(proposal));occupied.push_back({a.key,best->position});
        }
        for(auto id:movers){if(stop())return cancelled_result();const auto&a=*actors.at(id);const auto*km=knowledge(a);const auto*old=current(a);
            if(!able(a,w,config_)||!km||!old||committed_other(*old))continue;
            const auto*proof_now=evidence(a,*old);
            if(old->kind==OrderKind::bound&&proof_now&&proof_now->element==OutdoorElement::mover&&proof_now->movement_route==Evidence::yes&&
                distance_squared(a.position,old->goal_position)>old->arrival_radius_m*old->arrival_radius_m){
                OutdoorBoundEndpoint end;end.continuing=true;end.proposal.order=*old;end.proposal.expected_previous_order_seq=old->order_seq;end.proposal.group_id=g.group_id;
                end.restricted_route=Evidence::yes;end.cover_class=proof_now->cover;end.focus=proof_now->focus;end.focus_at_ms=proof_now->focus_at_ms;
                // Destination and lifetime persist; current native/Runtime
                // reflex owns aim/reload/fire. No stale fire packet is renewed.
                end.proposal.order.fire={};out.endpoints.push_back(end);continue;
            }
            if(out.ready_base.empty()||a.reloading||a.weapon.state==WeaponState::reloading||!a.weapon.ammo_in_mag||km->meter_level>=3||km->dogfight.valid())continue;
            if((old->kind==OrderKind::cover||old->kind==OrderKind::bound)&&distance_squared(a.position,old->goal_position)>old->arrival_radius_m*old->arrival_radius_m)continue;
            KnowledgeTrace trace;if(!mover_focus(a,w,*km,p,trace))continue;
            auto ci=caches.find(id);if(ci==caches.end())continue;const auto&cache=*ci->second;
            auto pi=profiles.find(id);if(pi==profiles.end())continue;const auto&profile=*pi->second;const auto&source=profile.source;
            if(source.actor!=a.key||source.level_epoch!=w.level_epoch||source.snapshot_seq!=w.sequence||source.geometry_revision!=w.geometry_revision||
                source.ownership_generation!=a.ownership_generation||source.restrictions_revision!=a.restrictions_revision||source.at_ms!=w.now_ms||
                source.origin_node!=a.node_id||!finite(source.actor_position)||distance_squared(source.actor_position,a.position)!=0||source.focus!=p.focus||
                source.focus_at_ms!=p.focus_fact.at_ms||!finite(source.focus_position)||distance_squared(source.focus_position,p.focus_fact.position)!=0||
                profile.heights!=std::array<float,3>{{.7f,1.1f,1.7f}})continue;
            if(cache.actor!=a.key||cache.level_epoch!=w.level_epoch||cache.snapshot_seq!=w.sequence||cache.ownership_generation!=a.ownership_generation||
                cache.restrictions_revision!=a.restrictions_revision||cache.origin_node!=a.node_id||cache.observed_at_ms!=w.now_ms||
                cache.knowledge_threat!=p.focus||cache.knowledge_observed_at_ms!=p.focus_fact.at_ms||distance_squared(cache.knowledge_threat_position,p.focus_fact.position)!=0)continue;
            const CoverSpot*best=nullptr;float best_cost=(std::numeric_limits<float>::max)();Evidence best_line=Evidence::unknown;
            for(const auto&spot:cache.spots){if(stop())return cancelled_result();++out.examined_candidates;
                const auto cls=cover_class(spot);const float travel=flat(a.position,spot.position);
                auto reject=[&](){++out.rejected_candidates;};
                if(spot.node==invalid_node||!finite(spot.position)||spot.threat!=0||spot.geometry_revision!=w.geometry_revision||cls==OutdoorCover::open||cls==OutdoorCover::unknown||
                    spot.threat_key!=p.focus.actor||spot.threat_observed_at_ms!=p.focus_fact.at_ms||spot.accessible==Evidence::no||spot.reachable==Evidence::no||
                    ((spot.accessible==Evidence::yes||spot.reachable==Evidence::yes)&&spot.restrictions_revision!=a.restrictions_revision)||
                    (spot.accessible==Evidence::unknown&&spot.reachable==Evidence::unknown&&spot.restrictions_revision!=0)||
                    !nonnegative(spot.lo)||spot.lo>1||travel<=policy_.arrival_m){reject();continue;}
                bool clash=false;for(const auto&x:occupied){if(stop())return cancelled_result();if(x.first!=a.key&&flat(x.second,spot.position)<config_.minimum_goal_separation){clash=true;break;}}
                if(clash){reject();continue;}
                Evidence line=Evidence::unknown;
                if(q.paths&&equal(q.path_stamp,out.stamp)){
                    const OutdoorBoundPath* path=nullptr;bool duplicate=false;
                    for(const auto&x:*q.paths){if(stop())return cancelled_result();if(x.actor==a.key&&x.goal_node==spot.node){if(path)duplicate=true;path=&x;}}
                    if(path&&!duplicate&&path->owner==a.ownership_generation&&path->restrictions_revision==a.restrictions_revision&&path->origin_node==a.node_id&&
                        finite(path->goal_position)&&distance_squared(path->goal_position,spot.position)==0){
                        if(path->complete==Evidence::no)line=Evidence::no;
                        else if(path->complete==Evidence::yes&&path->points.size()>=2&&distance_squared(path->points.front(),a.position)==0&&distance_squared(path->points.back(),spot.position)==0){
                            line=Evidence::yes;for(std::size_t j=0;j<path->points.size();++j){if(stop())return cancelled_result();if(!finite(path->points[j])){line=Evidence::unknown;break;}
                                if(j)for(const auto&f:fire_lines)if(segment_distance(path->points[j-1],path->points[j],f.first,f.second)<policy_.fire_line_clearance_m){line=Evidence::no;break;}
                                if(line==Evidence::no)break;}
                        }
                    }
                }
                if(line==Evidence::no){reject();continue;}
                float cost=travel+spot.lo*config_.cover_exposure_cost_m+std::max(0.f,travel-config_.immediate_cover_distance)*config_.cover_far_cost_m_per_m;
                cost+=std::max(0.f,policy_.bound_min_m-travel)+std::max(0.f,travel-policy_.bound_max_m);
                if(spot.accessible==Evidence::unknown||spot.reachable==Evidence::unknown)cost+=config_.cover_unknown_cost_m;
                if(line==Evidence::unknown)cost+=config_.cover_unknown_cost_m;
                if(door)cost+=flat(spot.position,e.door_point);
                else {const float range=flat(spot.position,p.focus_fact.position),ang=angle(spot.position,base_centre,p.focus_fact.position);
                    cost+=std::max(0.f,policy_.flank_min_m-range)+std::max(0.f,range-policy_.flank_max_m);
                    cost+=policy_.flank_cost_m*(std::max(0.f,policy_.flank_min_deg-ang)+std::abs(ang-policy_.flank_preferred_deg))/180.f;}
                if(std::isfinite(cost)&&(!best||cost<best_cost||(cost==best_cost&&spot.node<best->node))){best=&spot;best_cost=cost;best_line=line;}
            }
            if(!best)continue;
            OutdoorBoundEndpoint end;auto&proposal=end.proposal;proposal.expected_previous_order_seq=old->order_seq;proposal.group_id=g.group_id;
            proposal.order=first_response(a,config_,w.now_ms,w.level_epoch,w.geometry_revision,old->order_seq);
            auto&o=proposal.order;o.kind=OrderKind::bound;o.role=roles::mover;o.goal_node=best->node;o.goal_position=best->position;o.arrival_radius_m=policy_.arrival_m;
            o.assignment={};o.flags&=~order_first_response;o.stance=Stance::stand;
            const float travel=flat(a.position,best->position);const bool visible=o.look.kind==LookKind::enemy;
            if(visible||travel<=policy_.final_walk_m){o.pace=Pace::walk;o.speed_scale=policy_.aimed_speed;}
            else {o.pace=Pace::run;o.speed_scale=travel>=policy_.sprint_distance_m?policy_.sprint_speed:policy_.run_speed;o.flags|=order_face_movement;o.fire={};}
            if(!visible){o.look={LookKind::point,p.focus_fact.position,{}};o.fire={};}
            proposal.cover_cost=best_cost;proposal.has_cover_evidence=true;proposal.cover_evidence=*best;
            proposal.has_knowledge=true;proposal.knowledge=trace;
            proposal.navigation_pending=best->accessible!=Evidence::yes||best->reachable!=Evidence::yes;
            end.restricted_route=proposal.navigation_pending?Evidence::unknown:Evidence::yes;end.full_route_fire_clearance=best_line;
            end.cover_class=cover_class(*best);end.door_objective=door;end.focus=p.focus;end.focus_at_ms=p.focus_fact.at_ms;
            if(proposal.navigation_pending)++out.pending_navigation;if(best_line==Evidence::unknown)++out.unknown_route_clearance;
            out.endpoints.push_back(std::move(end));occupied.push_back({a.key,best->position});
        }
        const bool forming=std::any_of(out.base_positions.begin(),out.base_positions.end(),[](const ProposedOrder&x){return x.order.pace!=Pace::stand;});
        if(!out.ready_base.empty()||forming||!out.endpoints.empty())out.support_failure={};
        if(out.support_reposition.empty()&&p.support_recovery&&out.ready_base.empty()&&!forming&&out.endpoints.empty()){
            if(!approaches.empty()){out.support_reposition.push_back(std::move(approaches.front().proposal));out.support_reason="protective_approach";}
            else if(out.recovery_pending)out.support_reason="route_pending";
            else if(out.base_restricted)out.support_reason="restricted_route";
            else if(out.recovery_unsafe)out.support_reason="firing_lane_blocked";
            else if(out.recovery_occupied)out.support_reason="positions_occupied";
        }
        out.reason=out.endpoints.empty()?(out.ready_base.empty()?(out.base_positions.empty()?"base_not_ready":"forming_base"):"no_endpoint"):"bound_candidates";
        if(stop())return cancelled_result();
        if(!out.support_reposition.empty()){
            recovery_=out.support_reposition.front().order;recovery_stamp_=out.stamp;recovery_focus_=p.focus;recovery_focus_position_=p.focus_fact.position;
        }
        fired_=std::move(next_fired);return commit();
    }catch(...){invalid.status=OutdoorBoundStatus::failed;invalid.reason="exception";return invalid;}
}
}}}
