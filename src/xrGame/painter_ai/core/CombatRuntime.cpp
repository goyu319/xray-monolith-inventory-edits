#include "CombatRuntime.h"
#include "CombatGait.h"
#include "RoomFireIntent.h"
#include <algorithm>
#include <unordered_set>
#include <limits>
#include <cmath>
#include <map>

namespace painter { namespace combat { namespace core {
namespace {
// Reload is a weapon request. A live, accepted room assignment keeps its path
// while the executor reloads; unrelated first-response/cover rules stay intact.
bool room_assignment(const Order& order) {
    if(!order.assignment.generation)return false;
    // A room's reserve/hold position may have no particular door. It is still
    // a projected room assignment and retains movement while reloading.
    if(order.kind==OrderKind::hold_door)return true;
    if(order.kind==OrderKind::hold&&(order.role==roles::finisher||order.role==roles::buddy_answer||
        order.role==roles::suppress_buddy))return true;
    if(!order.assignment.portal)return false;
    switch(order.kind) {
    case OrderKind::stack:case OrderKind::peek:case OrderKind::entry:
    case OrderKind::search:case OrderKind::hold_door:case OrderKind::withdraw:return true;
    default:return false;
    }
}
bool room_reload_movement(const Order& order,const ActorSnapshot& actor) {
    const bool care=order.kind==OrderKind::recover&&valid_key(order.aid)&&order.assignment.generation&&
        (order.role==roles::casualty_guard||order.role==roles::aid_casualty);
    // Reloading a rifle must not cancel an already accepted aid transaction.
    // Plan compatibility still checks lease, geometry, access and hard yields;
    // the native executor alone owns treatment hands and target eligibility.
    return (care||(room_assignment(order)&&!valid_key(order.aid)))&&!order.throw_order.valid&&
        actor.weapon.id!=invalid_id&&(actor.weapon.ammo_in_mag||actor.weapon.ammo_reserve);
}
bool lifecycle_mission_valid(const Mission& m) {
    return m.rooms.count<=m.rooms.items.size()&&static_cast<unsigned>(m.kind)<=static_cast<unsigned>(MissionKind::hold);
}
bool lifecycle_same_mission(const Mission& a,const Mission& b) {
    if(!lifecycle_mission_valid(a)||!lifecycle_mission_valid(b)||a.revision!=b.revision||a.kind!=b.kind||
        a.building!=b.building||a.entry_portal!=b.entry_portal||a.rooms.count!=b.rooms.count)return false;
    for(std::size_t i=0;i<a.rooms.count;++i)if(a.rooms.items[i]!=b.rooms.items[i])return false;
    return true;
}
bool lifecycle_same_identity(const LifecycleIdentity& a,const LifecycleIdentity& b) {
    return a.level_epoch==b.level_epoch&&a.map_hash==b.map_hash&&a.geometry_revision==b.geometry_revision&&
        a.generation==b.generation&&a.group==b.group&&a.side==b.side&&a.faction==b.faction&&a.mission_revision==b.mission_revision;
}
void lifecycle_identity_values(std::vector<std::uint64_t>& v,const LifecycleIdentity& i) {
    v.insert(v.end(),{i.level_epoch,i.map_hash,i.geometry_revision,i.generation,i.group,i.side,i.faction,i.mission_revision});
}
bool lifecycle_raw_sides(const WorldSnapshot& w,std::unordered_map<std::uint32_t,SideSnapshot>& out) {
    std::unordered_map<std::uint32_t,const ActorSnapshot*> actors;
    for(const auto& a:w.actors)actors.emplace(a.key.id,&a);
    std::unordered_set<std::uint32_t> members;
    for(const auto& s:w.sides) {
        if(!s.side_id||out.count(s.side_id)||!lifecycle_mission_valid(s.mission)||!finite(s.link_up))return false;
        std::unordered_set<std::uint32_t> rooms;
        for(std::size_t i=0;i<s.mission.rooms.count;++i)
            if(s.mission.rooms.items[i]==invalid_room||!rooms.insert(s.mission.rooms.items[i]).second)return false;
        std::uint64_t native=0;
        for(auto key:s.members) {
            if(!valid_key(key)||!members.insert(key.id).second)return false;
            const auto actor=actors.find(key.id);if(actor==actors.end()||actor->second->key!=key)continue;
            if(actor->second->faction!=s.faction)return false;
            if(actor->second->native_squad){if(native&&native!=actor->second->native_squad)return false;native=actor->second->native_squad;}
        }
        if(native&&native!=s.group)return false;
        SideSnapshot row;row.side_id=s.side_id;row.faction=s.faction;row.group=s.group;
        row.mission=s.mission;row.building_id=s.building_id;row.members=s.members;
        out.emplace(s.side_id,std::move(row));
    }
    return true;
}
}
Runtime::Runtime(Config config):config_(config) {}
bool Runtime::handback_due(const ActorSnapshot& actor,const Record& record,TimeMs now) const {
    if(record.lifecycle_forced_release||record.first_failed)return true;
    if(grenade_movement(actor,record,now))return false;
    if(record.lifecycle_installed&&(record.lifecycle_authored||now<record.lifecycle_deadline))return false;
    return !(actor.flags&flag_requested_control)&&
        (!record.has_evidence||!fresh(record.last_evidence_at,now,config_.memory_max_age_ms))&&
        (!record.has_group_evidence||record.evidence_native_group!=actor.native_squad||record.evidence_faction!=actor.faction||
            !fresh(record.group_evidence_at,now,config_.memory_max_age_ms));
}
TimeMs Runtime::retention_deadline(const ActorSnapshot& actor,const Record& r,TimeMs now) const {
    if(r.lifecycle_installed&&r.lifecycle_authored)return (std::numeric_limits<TimeMs>::max)();
    if(r.lifecycle_installed&&now<r.lifecycle_deadline)return r.lifecycle_deadline;
    if(actor.flags&flag_requested_control)return (std::numeric_limits<TimeMs>::max)();
    if(grenade_movement(actor,r,now))return (std::min)(r.grenade_until,r.plan.expires_at_ms);
    const bool group=r.has_group_evidence&&r.evidence_native_group==actor.native_squad&&r.evidence_faction==actor.faction;
    if(!r.has_evidence&&!group)return (std::numeric_limits<TimeMs>::max)();
    const auto at=group?(r.has_evidence?(std::max)(r.last_evidence_at,r.group_evidence_at):r.group_evidence_at):r.last_evidence_at;
    return saturated_after(saturated_after(at,config_.memory_max_age_ms),1);
}
void Runtime::observe_species(const PerceivedSpeciesEvidence& evidence) {
    current_species_=evidence;
    // Preserve the native group's real combat admission before the first cheap
    // response. Group evidence is a separate original-time control receipt,
    // never a personal Contact, requested-control flag or firing permission.
    if(evidence.level_epoch==snapshot_.level_epoch&&evidence.snapshot_seq==snapshot_.sequence&&
        evidence.map_hash==snapshot_.map_hash&&evidence.geometry_revision==snapshot_.geometry_revision&&
        evidence.captured_at_ms==snapshot_.now_ms){
        std::map<std::uint32_t,const ActorSnapshot*> actors;
        for(const auto& a:snapshot_.actors)actors.emplace(a.key.id,&a);
        std::map<std::pair<std::uint32_t,std::uint64_t>,TimeMs> groups;
        std::unordered_set<std::uint32_t> observers;bool valid=true;
        for(const auto& row:evidence.observers){if(!observers.insert(row.observer.id).second){valid=false;break;}
            const auto found=actors.find(row.observer.id);if(found==actors.end()||found->second->key!=row.observer)continue;
            const auto& a=*found->second;TimeMs at=0;
            if(native_group_evidence_at(a,a,row,config_,snapshot_.now_ms,at)){
                auto& group=groups[{a.faction,a.native_squad}];group=(std::max)(group,at);}}
        if(valid)for(const auto& a:snapshot_.actors){const auto g=groups.find({a.faction,a.native_squad});
            auto r=records_.find(a.key.id);if(g==groups.end()||!a.native_squad||r==records_.end()||r->second.key!=a.key||
                r->second.owner_generation!=a.ownership_generation||r->second.released||r->second.lifecycle_forced_release||
                !(a.flags&flag_admitted)||a.yield_reasons||!a.alive||a.wounded)continue;
            auto& record=r->second;record.has_group_evidence=true;record.evidence_native_group=a.native_squad;record.evidence_faction=a.faction;
            record.group_evidence_at=(std::max)(record.group_evidence_at,g->second);
        }
    }
    // A rejected native publication is not a completed handback. Recheck it
    // against the current capture before retrying; accepted leases stay dead.
    for(auto side=species_release_sides_.begin();side!=species_release_sides_.end();){
        const auto fight=perceived_fight(snapshot_,side->second,nullptr,current_species_,config_);
        bool pending=false;
        for(auto member:side->second.members){auto r=records_.find(member.id);if(r==records_.end()||r->second.key!=member||
            r->second.released||!r->second.species_release)continue;
            auto& record=r->second;
            if(!fight.valid||!fight.monster_only){record.species_release=false;record.lifecycle_forced_release=false;
                record.release_pending=false;record.lifecycle_installed=false;record.lifecycle_deadline=0;record.next_refresh=0;
                record.deferred_until=0;replan_requested_=true;}
            else pending=true;
        }
        if(!pending)side=species_release_sides_.erase(side);else ++side;
    }
}
bool Runtime::install_lifecycle(const WorldSnapshot& source,const SideLifecycleResult& result) {
    try {
    // Registry is the trusted producer of generated group/mission authority.
    // This boundary still independently checks raw current identity, native
    // group, faction, explicit authority, freshness and every retained lease.
    if(!result.accepted||result.cancelled||result.status!=SideLifecycleStatus::accepted||
        !source.level_epoch||!source.sequence||source.level_epoch!=snapshot_.level_epoch||source.map_hash!=snapshot_.map_hash||
        source.geometry_revision!=snapshot_.geometry_revision||source.sequence>snapshot_.sequence||
        source.sequence<lifecycle_source_sequence_||!fresh(source.now_ms,snapshot_.now_ms,config_.snapshot_max_age_ms)||
        result.level_epoch!=source.level_epoch||result.map_hash!=source.map_hash||result.snapshot_seq!=source.sequence||
        result.captured_at_ms!=source.now_ms||result.sides.size()!=source.sides.size())return false;
    using Actors=std::unordered_map<std::uint32_t,const ActorSnapshot*>;
    Actors old,current;
    for(const auto& a:source.actors)if(!valid_key(a.key)||a.level_epoch!=source.level_epoch||a.snapshot_seq!=source.sequence||
        a.observed_at_ms!=source.now_ms||!old.emplace(a.key.id,&a).second)return false;
    for(const auto& a:snapshot_.actors)current.emplace(a.key.id,&a);
    std::unordered_map<std::uint32_t,const SideSnapshot*> sides,explicit_sides;
    std::unordered_map<std::uint32_t,std::uint32_t> roster,explicit_roster;
    std::vector<std::uint64_t> signature={source.level_epoch,source.map_hash,source.geometry_revision,source.sequence,source.now_ms,source.sides.size()};
    auto add_side=[&](const SideSnapshot& s,auto& index,auto& members) {
        if(!s.side_id||!lifecycle_mission_valid(s.mission)||!index.emplace(s.side_id,&s).second)return false;
        for(auto k:s.members)if(!valid_key(k)||!members.emplace(k.id,s.side_id).second)return false;
        return true;
    };
    std::unordered_map<std::uint32_t,SideSnapshot> current_authority;
    if(!lifecycle_raw_sides(snapshot_,current_authority))return false;
    for(const auto& s:current_authority)if(!add_side(s.second,explicit_sides,explicit_roster))return false;
    for(const auto& s:source.sides) {
        if(!add_side(s,sides,roster))return false;
        const auto explicit_side=explicit_sides.find(s.side_id);
        if(explicit_side!=explicit_sides.end()) {
            const auto& x=*explicit_side->second;
            if(s.faction!=x.faction||s.group!=x.group||s.members!=x.members||
                !lifecycle_same_mission(s.mission,x.mission)||s.building_id!=x.building_id)return false;
        }
        signature.insert(signature.end(),{s.side_id,s.faction,s.group,s.building_id,s.mission.revision,
            static_cast<std::uint64_t>(s.mission.kind),s.mission.building,s.mission.entry_portal,s.mission.rooms.count});
        for(std::size_t i=0;i<s.mission.rooms.count;++i)signature.push_back(s.mission.rooms.items[i]);
        signature.push_back(s.members.size());
        for(auto k:s.members) {
            const auto was=old.find(k.id),now=current.find(k.id);const auto explicit_member=explicit_roster.find(k.id);
            if(explicit_member!=explicit_roster.end()&&explicit_member->second!=s.side_id)return false;
            if((was==old.end())!=(now==current.end()))return false;
            signature.insert(signature.end(),{k.id,k.generation});
            if(was==old.end()){signature.push_back(0);continue;}
            const auto& a=*was->second;const auto& b=*now->second;
            if(a.key!=k||b.key!=k||a.ownership_generation!=b.ownership_generation||a.faction!=s.faction||
                b.faction!=s.faction||a.native_squad!=b.native_squad)return false;
            if(explicit_side==explicit_sides.end()&&a.native_squad&&a.native_squad!=s.group)return false;
            signature.insert(signature.end(),{a.ownership_generation,a.native_squad});
        }
    }
    std::unordered_map<std::uint32_t,const SideLifecycleView*> views;
    for(const auto& view:result.sides) {
        const auto& i=view.identity;const auto s=sides.find(i.side);
        if(s==sides.end()||!views.emplace(i.side,&view).second||!i.generation||i.level_epoch!=source.level_epoch||
            i.map_hash!=source.map_hash||i.geometry_revision!=source.geometry_revision||i.faction!=s->second->faction||
            i.group!=s->second->group||i.mission_revision!=s->second->mission.revision||
            static_cast<unsigned>(view.phase)>static_cast<unsigned>(SideLifecyclePhase::awaiting_admission)||
            static_cast<unsigned>(view.release_reason)>static_cast<unsigned>(SideReleaseReason::door_stuck))return false;
        if(view.release_reason==SideReleaseReason::monster_fight){
            const auto fight=perceived_fight(snapshot_,*s->second,nullptr,current_species_,config_);
            if(!fight.valid||!fight.monster_only)return false;
        }
        lifecycle_identity_values(signature,i);
        signature.insert(signature.end(),{static_cast<std::uint64_t>(view.phase),static_cast<std::uint64_t>(view.release_reason),
            view.quiet_deadline_ms,view.cooldown_until_ms});
    }
    struct Pending {Record* record;const SideSnapshot* side;const ActorSnapshot* actor;const RetainedControlLease* hold;const LifecycleRelease* release;};
    std::vector<Pending> pending;pending.reserve(result.retained.size()+result.releases.size());
    std::unordered_set<std::uint32_t> touched;
    auto member=[&](const LifecycleIdentity& i,const LifecycleLease& lease,std::uint64_t sequence)->Pending {
        Pending p{};const auto v=views.find(i.side);const auto s=sides.find(i.side);const auto a=current.find(lease.actor.id);
        const auto original=old.find(lease.actor.id);
        const auto r=records_.find(lease.actor.id);const auto rr=roster.find(lease.actor.id);
        if(sequence!=source.sequence||!valid_key(lease.actor)||!lease.ownership_generation||
            v==views.end()||!lifecycle_same_identity(i,v->second->identity)||s==sides.end()||
            a==current.end()||a->second->key!=lease.actor||a->second->ownership_generation!=lease.ownership_generation||
            !(a->second->flags&flag_admitted)||original==old.end()||!(original->second->flags&flag_admitted)||
            rr==roster.end()||rr->second!=i.side||
            r==records_.end()||r->second.key!=lease.actor||r->second.owner_generation!=lease.ownership_generation||
            (i.level_epoch==r->second.lifecycle_identity.level_epoch&&i.map_hash==r->second.lifecycle_identity.map_hash&&
                i.generation<r->second.lifecycle_identity.generation)||
            sequence<=r->second.lifecycle_invalidated_through||!touched.insert(lease.actor.id).second)return p;
        p.record=&r->second;p.side=s->second;p.actor=a->second;return p;
    };
    signature.push_back(result.retained.size());
    for(const auto& hold:result.retained) {
        auto p=member(hold.identity,hold.member,hold.source_snapshot);if(!p.record)return false;
        const auto& v=*views.at(hold.identity.side);const auto& r=*p.record;const auto& a=*p.actor;const auto& was=*old.at(a.key.id);
        if(hold.captured_at_ms!=source.now_ms||hold.quiet_deadline_ms!=v.quiet_deadline_ms||
            v.release_reason!=SideReleaseReason::none||r.released||r.release_pending||r.lifecycle_forced_release||
            !a.alive||a.wounded||a.yield_reasons!=yield_none||!was.alive||was.wounded||was.yield_reasons!=yield_none||
            (hold.authored_mission_retention&&p.side->mission.kind==MissionKind::none)||
            (!hold.authored_mission_retention&&hold.quiet_deadline_ms<=source.now_ms))return false;
        lifecycle_identity_values(signature,hold.identity);
        signature.insert(signature.end(),{hold.member.actor.id,hold.member.actor.generation,hold.member.ownership_generation,
            hold.source_snapshot,hold.captured_at_ms,hold.quiet_deadline_ms,hold.authored_mission_retention});
        p.hold=&hold;pending.push_back(p);
    }
    signature.push_back(result.releases.size());
    for(const auto& release:result.releases) {
        auto p=member(release.identity,release.member,release.source_snapshot);if(!p.record)return false;
        const auto& v=*views.at(release.identity.side);
        if(release.reason==SideReleaseReason::none||release.reason!=v.release_reason||
            release.released_at_ms>source.now_ms||release.cooldown_until_ms<release.released_at_ms)return false;
        // A queued quiet expiry from an older capture cannot revoke a newly
        // renewed, explicit full-roster request in the current capture.
        if(release.reason==SideReleaseReason::fight_over&&requested_authored_mission(snapshot_,*p.side))return false;
        lifecycle_identity_values(signature,release.identity);
        signature.insert(signature.end(),{release.member.actor.id,release.member.actor.generation,release.member.ownership_generation,
            release.source_snapshot,static_cast<std::uint64_t>(release.reason),release.released_at_ms,release.cooldown_until_ms});
        p.release=&release;pending.push_back(p);
    }
    if(source.sequence==lifecycle_source_sequence_)return signature==lifecycle_signature_;
    // All validation and allocation complete: commit cannot partially apply a
    // malformed later member. A release latch survives any later retention.
    auto species_sides=species_release_sides_;
    for(const auto& side:source.sides)if(views.at(side.side_id)->release_reason==SideReleaseReason::monster_fight)
        species_sides.insert_or_assign(side.side_id,side);
    species_release_sides_=std::move(species_sides);
    lifecycle_signature_=std::move(signature);lifecycle_source_sequence_=source.sequence;
    lifecycle_last_valid_sides_=std::move(current_authority);
    for(const auto& p:pending) {
        auto& r=*p.record;r.lifecycle_identity=p.hold?p.hold->identity:p.release->identity;
        r.lifecycle_native_squad=p.actor->native_squad;
        r.lifecycle_installed=p.hold!=nullptr;r.lifecycle_authored=p.hold&&p.hold->authored_mission_retention;
        r.lifecycle_deadline=p.hold?p.hold->quiet_deadline_ms:0;
        if(p.release){r.lifecycle_forced_release=true;r.species_release=p.release->reason==SideReleaseReason::monster_fight;
            r.has_plan=false;r.has_cover=false;r.cover={};covers_.erase(r.key.id);}
        r.deferred_until=0;r.next_refresh=0;
    }
    return true;
    }catch(...){return false;}
}
void Runtime::invalidate_lifecycle(ActorKey key,std::uint64_t owner) {
    const auto it=records_.find(key.id);if(it==records_.end()||it->second.key!=key||it->second.owner_generation!=owner)return;
    auto& r=it->second;r.lifecycle_installed=false;r.lifecycle_authored=false;r.lifecycle_deadline=0;
    r.lifecycle_invalidated_through=(std::max)(r.lifecycle_invalidated_through,snapshot_.sequence);
    r.deferred_until=0;r.next_refresh=0;
    // Already requested exact-lease handback is irreversible until native ack;
    // mission changes are not permission to resurrect that ownership lease.
}
bool Runtime::accepts_orders(ActorKey key,std::uint64_t owner) const {
    const auto r=records_.find(key.id);if(r==records_.end()||r->second.key!=key||r->second.owner_generation!=owner||
        r->second.released||r->second.release_pending||r->second.lifecycle_forced_release)return false;
    const auto a=std::lower_bound(snapshot_.actors.begin(),snapshot_.actors.end(),key.id,
        [](const ActorSnapshot& actor,std::uint32_t id){return actor.key.id<id;});
    if(a==snapshot_.actors.end()||a->key!=key||handback_due(*a,r->second,snapshot_.now_ms))return false;
    return first_response(*a,config_,snapshot_.now_ms,snapshot_.level_epoch,snapshot_.geometry_revision,1).kind!=OrderKind::yield||
        (r->second.has_plan&&continuing_throw(r->second.plan,*a,snapshot_.now_ms)&&plan_compatible(*a,r->second,snapshot_.now_ms));
}
bool Runtime::grenade_movement(const ActorSnapshot& a,const Record& r,TimeMs now) const {
    const auto&o=r.plan;
    return r.has_grenade&&r.has_plan&&r.grenade_until>now&&r.grenade_plan_seq==o.order_seq&&
        o.actor==a.key&&o.ownership_generation==a.ownership_generation&&o.restrictions_revision==a.restrictions_revision&&
        o.kind==OrderKind::cover&&o.role==roles::under_fire_cover&&(o.flags&order_urgent)&&
        !o.fire.allowed&&!o.fire.at_valid&&!o.assignment.generation&&!valid_key(o.aid)&&!o.throw_order.valid&&
        r.grenade.kind==DangerKind::grenade&&r.grenade.perceive==0&&r.grenade.at_ms&&r.grenade.at_ms<=now&&
        finite(r.grenade.position)&&o.expires_at_ms<=r.grenade_until;
}
bool Runtime::accepted_grenade_episode(const Order&o,const ActorSnapshot& source,const Danger& perceived,TimeMs until){
    auto it=records_.find(o.actor.id);if(it==records_.end())return false;auto&r=it->second;
    if(!r.has_plan||r.plan.order_seq!=o.order_seq||r.plan.actor!=o.actor||r.owner_generation!=o.ownership_generation||
       source.key!=o.actor||source.level_epoch!=o.level_epoch||source.snapshot_seq!=o.snapshot_seq||
       source.ownership_generation!=o.ownership_generation||source.restrictions_revision!=o.restrictions_revision||
       source.observed_at_ms>snapshot_.now_ms||!fresh(source.observed_at_ms,snapshot_.now_ms,config_.snapshot_max_age_ms)||
       perceived.kind!=DangerKind::grenade||perceived.perceive!=0||!perceived.at_ms||perceived.at_ms>source.observed_at_ms||
       !finite(perceived.position)||until<=snapshot_.now_ms||o.expires_at_ms>until||o.kind!=OrderKind::cover||
       o.role!=roles::under_fire_cover||!(o.flags&order_urgent)||o.fire.allowed||o.fire.at_valid||
       o.assignment.generation||valid_key(o.aid)||o.throw_order.valid)return false;
    const auto same=[&](const Danger&d){return d.kind==perceived.kind&&d.perceive==perceived.perceive&&d.at_ms==perceived.at_ms&&
        finite(d.position)&&distance_squared(d.position,perceived.position)==0;};
    const bool seen=same(source.danger);
    const bool retained=r.has_grenade&&r.grenade_until==until&&same(r.grenade);
    if(!seen&&!retained)return false;
    r.grenade=perceived;r.grenade_until=until;r.grenade_plan_seq=o.order_seq;r.has_grenade=true;return true;
}
bool Runtime::reload_cover_movement(const ActorSnapshot& a,const Record& r,TimeMs now) const {
    const auto& o=r.plan;const auto& proof=r.cover;const auto& c=proof.spot;
    const bool tactical_leg=o.kind==OrderKind::bound||o.kind==OrderKind::withdraw;
    if(!r.has_cover||(o.kind!=OrderKind::cover&&o.kind!=OrderKind::reload&&!tactical_leg)||o.pace==Pace::stand||
        o.assignment.generation||o.portal.valid||o.throw_order.valid||valid_key(o.aid)||
        a.weapon.id==invalid_id||(!a.weapon.ammo_in_mag&&!a.weapon.ammo_reserve)||
        !a.accessible_current_order||!a.last_applied.order_seq||a.last_applied.order_seq!=r.order.order_seq||
        a.last_applied.at_ms<r.order.issued_at_ms||a.last_applied.at_ms>a.observed_at_ms||
        !fresh(a.last_applied.at_ms,now,config_.snapshot_max_age_ms)||
        r.order.goal_node!=o.goal_node||distance_squared(r.order.goal_position,o.goal_position)>.0001f||
        distance_squared(a.position,o.goal_position)<=o.arrival_radius_m*o.arrival_radius_m||
        !proof.snapshot_seq||proof.snapshot_seq>a.snapshot_seq||
        (tactical_leg?proof.observed_at_ms>o.issued_at_ms:!fresh(proof.observed_at_ms,now,config_.cover_cache_ms))||
        c.node!=o.goal_node||!finite(c.position)||distance_squared(c.position,o.goal_position)>.0001f||
        c.geometry_revision!=snapshot_.geometry_revision||c.protection_known!=Evidence::yes||
        (!c.blocked[0]&&!c.blocked[1])||!std::isfinite(c.lo)||c.lo<0||c.lo>=1||
        c.accessible==Evidence::no||c.reachable==Evidence::no||
        (c.accessible==Evidence::unknown?c.restrictions_revision!=0:c.restrictions_revision!=a.restrictions_revision))return false;
    if(proof.has_attributed_hit){
        if(valid_key(c.threat_key)||(o.role!=roles::under_fire_cover&&o.role!=roles::reload_cover)||
            proof.attributed_hit.at_ms!=c.threat_observed_at_ms||c.threat_observed_at_ms>proof.observed_at_ms||
            !fresh(proof.attributed_hit.at_ms,now,config_.knowledge.uf_escape_ms)||
            !finite(proof.attributed_hit.dir)||distance_squared(proof.attributed_hit.dir,Vec3{})<=0||
            !finite(proof.threat_position))return false;
        // The Dispatcher installed this original sensor only after matching
        // the authored escape and actual native acceptance. Sensor rings are
        // deltas: requiring the next capture to repeat it would erase memory.
    }else if(!tactical_leg){
        const auto* target=select_contact(a,config_,now);
        if(!target||target->key!=c.threat_key||!finite(proof.threat_position)||
            target->observed_at_ms<c.threat_observed_at_ms||c.threat_observed_at_ms>proof.observed_at_ms||
            !fresh(c.threat_observed_at_ms,now,config_.memory_max_age_ms)||
            distance_squared(target->position,proof.threat_position)>.0001f)return false;
    }
    const float separation=config_.minimum_goal_separation*config_.minimum_goal_separation;
    for(const auto& mate:snapshot_.actors) {
        if(mate.key==a.key||mate.faction!=a.faction||!mate.alive)continue;
        if(!finite(mate.position)||mate.node_id==o.goal_node||distance_squared(mate.position,o.goal_position)<separation)return false;
        const auto held=records_.find(mate.key.id);
        if(held==records_.end())continue;
        const auto& other=held->second.order;
        if(other.actor==mate.key&&other.ownership_generation==mate.ownership_generation&&other.expires_at_ms>now&&
            (other.goal_node==o.goal_node||distance_squared(other.goal_position,o.goal_position)<separation))return false;
    }
    return true;
}
bool Runtime::plan_compatible(const ActorSnapshot& a,const Record& r,TimeMs now) const {
    const auto& o=r.plan;
    const bool stationary_reload=o.kind==OrderKind::reload&&r.reload_requested&&!r.reload_complete&&
        a.weapon.id==r.reload_weapon&&o.pace==Pace::stand&&o.goal_node==a.node_id&&
        finite(o.goal_position)&&distance_squared(a.position,o.goal_position)<=o.arrival_radius_m*o.arrival_radius_m;
    if(o.kind==OrderKind::reload&&(!r.reload_requested||a.weapon.id!=r.reload_weapon||
        (o.pace==Pace::stand?!stationary_reload:(!r.reload_complete&&!reload_cover_movement(a,r,now)))))return false;
    if(!r.has_plan||r.released||r.release_pending||handback_due(a,r,now)||o.expires_at_ms<=now||
        o.kind==OrderKind::yield||a.key!=o.actor||a.ownership_generation!=o.ownership_generation||
        a.restrictions_revision!=o.restrictions_revision||snapshot_.geometry_revision!=o.geometry_revision||
        !(a.flags&flag_admitted)||!a.alive||a.wounded||a.yield_reasons||!a.current_position_accessible||
        ((a.reloading||a.weapon.state==WeaponState::reloading||!a.weapon.ammo_in_mag)&&
            !stationary_reload&&!continuing_throw(o,a,now)&&!room_reload_movement(o,a)&&!reload_cover_movement(a,r,now)&&!grenade_movement(a,r,now))||
        a.hits.count>a.hits.items.size())return false;
    if(a.last_applied.order_seq==r.order.order_seq&&a.last_applied.order_seq&&!a.accessible_current_order)return false;
    if(a.last_rejected.node==o.goal_node&&a.last_rejected.restrictions_revision==a.restrictions_revision&&
        (a.last_rejected.reason==reject_goal_invalid||a.last_rejected.reason==reject_goal_inaccessible))return false;
    // A hit, a sightline blink or a different selected threat changes aim and fire, never a still-valid movement
    // assignment: under fire the man keeps walking to his cover (plan_disturbed re-aims him and asks for a re-plan).
    return true;
}
bool Runtime::renew_ordinary_plan(const ActorSnapshot& a,Record& r,TimeMs now) {
    const auto& o=r.plan;TimeMs evidence=0;
    // This is fresh native revalidation of an ordinary movement/watch command,
    // not extension of a room lease, treatment, grenade fuse or suppression fact.
    if(!r.has_plan||r.first_failed||r.released||r.release_pending||
        (o.kind!=OrderKind::hold&&o.kind!=OrderKind::cover&&o.kind!=OrderKind::advance)||
        o.assignment.generation||o.portal.valid||o.throw_order.valid||valid_key(o.aid)||valid_key(o.finish)||
        o.pie.valid||o.corner.valid||o.expose.stand||o.expose.lean||o.lean!=0||
        (o.flags&(order_urgent|order_committed_entry|order_cover_fire))||
        r.has_grenade||r.reload_requested||a.reloading||a.weapon.state==WeaponState::reloading||
        !a.weapon.ammo_in_mag||!personal_evidence_at(a,config_,now,evidence)||
        !a.accessible_current_order||!a.last_applied.order_seq||a.last_applied.order_seq!=r.order.order_seq||
        a.last_applied.at_ms<r.order.issued_at_ms||a.last_applied.at_ms>r.order.expires_at_ms||
        !fresh(a.last_applied.at_ms,now,config_.snapshot_max_age_ms)||a.last_applied.at_ms>a.observed_at_ms||
        !fresh(a.observed_at_ms,now,config_.snapshot_max_age_ms))return false;
    auto candidate=r;candidate.plan.expires_at_ms=saturated_after(now,config_.order_lifetime_ms);
    if(!plan_compatible(a,candidate,now))return false;
    r.plan.expires_at_ms=candidate.plan.expires_at_ms;return true;
}
bool Runtime::plan_disturbed(const ActorSnapshot& a,const Record& r,TimeMs now) const {
    if(a.hits.count>a.hits.items.size())return false;
    for(std::size_t i=0;i<a.hits.count;++i)
        if(finite(a.hits.items[i].dir)&&a.hits.items[i].at_ms>r.plan_hit_at&&a.hits.items[i].at_ms<=now)return true;
    const auto* t=select_contact(a,config_,now);
    return t&&t->key!=r.plan_target;
}
std::uint64_t Runtime::allocate_order_sequence() {
    if(next_order_==(std::numeric_limits<std::uint64_t>::max)())return 0;
    return next_order_++;
}
bool Runtime::accepted_plan(const Order& order,const Order* authored_room,const AcceptedCoverProof* cover) {
    auto it=records_.find(order.actor.id);
    if(it==records_.end()||it->second.key!=order.actor||
        it->second.owner_generation!=order.ownership_generation||
        order.level_epoch!=snapshot_.level_epoch||!order.order_seq||
        order.order_seq<it->second.order.order_seq)return false;
    if(it->second.released||it->second.release_pending||it->second.first_failed||it->second.lifecycle_forced_release)return false;
    // A native-accepted refresh of the same still-applied outdoor leg can
    // retain its original physical proof. Explicit replacement evidence is
    // always evaluated on its own; a bad supplied proof never borrows this one.
    AcceptedCoverProof retained_cover;bool retain_cover=false;
    const auto unassigned_leg=[](const Order& o){return (o.kind==OrderKind::bound||o.kind==OrderKind::withdraw)&&o.pace!=Pace::stand&&
        !o.assignment.generation&&!o.assignment.sector&&!o.assignment.portal&&!o.assignment.slot&&!o.portal.valid&&!o.throw_order.valid&&
        !valid_key(o.aid)&&!valid_key(o.finish)&&!o.pie.valid&&!o.corner.valid;};
    const auto same_leg=[&](const Order& a,const Order& b){return unassigned_leg(a)&&unassigned_leg(b)&&a.actor==b.actor&&a.level_epoch==b.level_epoch&&
        a.ownership_generation==b.ownership_generation&&a.restrictions_revision==b.restrictions_revision&&a.geometry_revision==b.geometry_revision&&
        a.kind==b.kind&&a.role==b.role&&a.goal_node==b.goal_node&&finite(a.goal_position)&&finite(b.goal_position)&&distance_squared(a.goal_position,b.goal_position)==0&&
        a.arrival_radius_m==b.arrival_radius_m&&a.pace==b.pace&&a.stance==b.stance&&a.gait==b.gait&&a.speed_scale==b.speed_scale&&a.lean==b.lean&&
        a.lean_edge.x==b.lean_edge.x&&a.lean_edge.y==b.lean_edge.y&&a.lean_edge.z==b.lean_edge.z&&a.expose.stand==b.expose.stand&&a.expose.lean==b.expose.lean&&
        a.side==b.side&&a.abort_vertex==b.abort_vertex&&((a.flags^b.flags)&~(order_first_response|order_cover_fire))==0;};
    const auto& previous=it->second;
    if(!cover&&previous.has_plan&&previous.has_cover&&same_leg(order,previous.plan)&&order.snapshot_seq==snapshot_.sequence&&
        order.issued_at_ms==snapshot_.now_ms&&order.expires_at_ms>snapshot_.now_ms&&
        order.expires_at_ms<=saturated_after(snapshot_.now_ms,config_.order_lifetime_ms)) {
        const auto actor=std::lower_bound(snapshot_.actors.begin(),snapshot_.actors.end(),order.actor.id,
            [](const ActorSnapshot& a,std::uint32_t id){return a.key.id<id;});
        if(actor!=snapshot_.actors.end()&&actor->key==order.actor) {
            // A cheap decide may have replaced Record::order before this
            // publication. Use only the command named by native last_applied.
            const Order* predecessor=actor->last_applied.order_seq==previous.order.order_seq?&previous.order:
                actor->last_applied.order_seq==previous.plan.order_seq?&previous.plan:nullptr;
            if(predecessor&&predecessor->order_seq&&order.order_seq>predecessor->order_seq&&predecessor->expires_at_ms>snapshot_.now_ms&&same_leg(order,*predecessor)) {
                auto applied=previous;applied.order=*predecessor;
                if(plan_compatible(*actor,applied,snapshot_.now_ms)&&reload_cover_movement(*actor,applied,snapshot_.now_ms)) {
                    retained_cover=previous.cover;retain_cover=true;
                }
            }
        }
    }
    // Replaying the same accepted packet cannot forget observed completion.
    const bool replay_reload=previous.has_plan&&previous.plan.order_seq==order.order_seq&&
        previous.plan.kind==order.kind&&previous.plan.snapshot_seq==order.snapshot_seq&&
        previous.plan.issued_at_ms==order.issued_at_ms&&previous.plan.expires_at_ms==order.expires_at_ms&&
        previous.plan.goal_node==order.goal_node&&distance_squared(previous.plan.goal_position,order.goal_position)==0;
    const auto old_reload_weapon=previous.reload_weapon;
    const bool old_reload_requested=previous.reload_requested,old_reload_seen=previous.reload_seen,old_reload_complete=previous.reload_complete;
    it->second.order=order;it->second.plan=order;it->second.has_plan=true;it->second.plan_hit_at=order.issued_at_ms;
    it->second.reload_weapon=invalid_id;it->second.reload_requested=it->second.reload_seen=it->second.reload_complete=false;
    it->second.has_cover=false;it->second.cover=AcceptedCoverProof{};
    if(retain_cover){it->second.cover=retained_cover;it->second.has_cover=true;}
    if(cover&&(order.kind==OrderKind::cover||order.kind==OrderKind::reload||order.kind==OrderKind::bound||order.kind==OrderKind::withdraw)&&
        !order.assignment.generation&&!order.portal.valid&&!order.throw_order.valid&&!valid_key(order.aid)&&
        cover->snapshot_seq&&cover->snapshot_seq<=snapshot_.sequence&&cover->snapshot_seq<=order.snapshot_seq&&
        cover->spot.threat_observed_at_ms<=cover->observed_at_ms&&
        fresh(cover->observed_at_ms,snapshot_.now_ms,config_.cover_cache_ms)&&
        cover->spot.node==order.goal_node&&finite(cover->spot.position)&&finite(cover->threat_position)&&
        distance_squared(cover->spot.position,order.goal_position)<=.0001f&&
        cover->spot.geometry_revision==order.geometry_revision) {
        it->second.cover=*cover;it->second.has_cover=true;
    }
    if(authored_room&&room_assignment(order)&&room_assignment(*authored_room)&&
        authored_room->actor==order.actor&&authored_room->level_epoch==order.level_epoch&&
        authored_room->ownership_generation==order.ownership_generation&&
        authored_room->restrictions_revision==order.restrictions_revision&&
        authored_room->geometry_revision==order.geometry_revision&&
        authored_room->assignment.generation==order.assignment.generation&&
        authored_room->assignment.portal==order.assignment.portal&&authored_room->assignment.slot==order.assignment.slot&&
        (authored_room->look.kind==LookKind::point||authored_room->look.kind==LookKind::direction)&&finite(authored_room->look.value))
        it->second.plan.look=authored_room->look;
    // The flip reference is the man's own selected target when the plan lands (the squad's focus may rightly differ):
    // a flip is his selection changing after that, which re-aims him and asks for a re-plan
    it->second.plan_target=order.look.target;
    {
        const auto a=std::lower_bound(snapshot_.actors.begin(),snapshot_.actors.end(),order.actor.id,
            [](const ActorSnapshot& actor,std::uint32_t id){return actor.key.id<id;});
        if(a!=snapshot_.actors.end()&&a->key==order.actor) {
            const auto target=select_contact(*a,config_,snapshot_.now_ms);if(target)it->second.plan_target=target->key;
            if(order.kind==OrderKind::reload&&a->weapon.id!=invalid_id&&a->ownership_generation==order.ownership_generation&&
                a->restrictions_revision==order.restrictions_revision&&order.geometry_revision==snapshot_.geometry_revision&&
                order.snapshot_seq&&order.snapshot_seq<=snapshot_.sequence&&order.expires_at_ms>snapshot_.now_ms&&
                fresh(order.issued_at_ms,snapshot_.now_ms,config_.snapshot_max_age_ms)) {
                it->second.reload_weapon=a->weapon.id;it->second.reload_requested=true;
                // A native reload may already be in progress before this
                // accepted request. Capture that fact without attributing its
                // start to the new packet, which has not appeared in a capture.
                it->second.reload_seen=a->reloading||a->weapon.state==WeaponState::reloading;
                if(replay_reload&&old_reload_requested&&old_reload_weapon==a->weapon.id){
                    it->second.reload_seen=old_reload_seen||it->second.reload_seen;it->second.reload_complete=old_reload_complete;
                }
            }
        }
    }
    it->second.deferred_until=0;
    it->second.next_refresh=(std::min)(order.expires_at_ms,saturated_after(snapshot_.now_ms,
        (std::max)(TimeMs(1),config_.refresh_ms)));
    if(order.fire.allowed)it->second.next_refresh=(std::min)(it->second.next_refresh,order.fire.until_ms);
    const auto actor=std::lower_bound(snapshot_.actors.begin(),snapshot_.actors.end(),order.actor.id,
        [](const ActorSnapshot& a,std::uint32_t id){return a.key.id<id;});
    if(actor!=snapshot_.actors.end()&&actor->key==order.actor)
        it->second.next_refresh=(std::min)(it->second.next_refresh,retention_deadline(*actor,it->second,snapshot_.now_ms));
    return true;
}
bool Runtime::invalidate_plan(ActorKey key,std::uint64_t ownership) {
    auto it=records_.find(key.id);
    if(!valid_key(key)||!ownership||it==records_.end()||it->second.key!=key||
        it->second.owner_generation!=ownership||it->second.released||it->second.release_pending)return false;
    auto& r=it->second;
    r.has_plan=false;r.plan=Order{};r.plan_target=ActorKey{};r.plan_hit_at=0;r.has_cover=false;r.cover=AcceptedCoverProof{};
    r.deferred_until=0;r.next_refresh=0;covers_.erase(key.id);replan_requested_=true;
    return true;
}
bool Runtime::fail_first_order(ActorKey key,std::uint64_t owner) {
    const auto found=records_.find(key.id);if(found==records_.end()||found->second.key!=key||
        found->second.owner_generation!=owner||found->second.released)return false;
    auto& r=found->second;r.first_failed=true;r.release_pending=true;r.has_plan=false;
    r.next_refresh=0;r.deferred_until=0;return true;
}
void Runtime::accepted_release(ActorKey key,std::uint64_t ownership) {
    auto it=records_.find(key.id);if(it==records_.end()||it->second.key!=key||it->second.owner_generation!=ownership)return;
    auto& r=it->second;r.released=true;r.release_pending=false;r.has_plan=false;r.deferred_until=0;r.has_cover=false;r.cover=AcceptedCoverProof{};
    covers_.erase(key.id);
}
void Runtime::rejected_release(ActorKey key) {
    auto it=records_.find(key.id);if(it!=records_.end()&&it->second.key==key&&!it->second.released)
        it->second.next_refresh=0;
}
void Runtime::rejected_proposal(ActorKey key,std::uint64_t seq) {
    auto it=records_.find(key.id);
    if(it!=records_.end()&&it->second.key==key&&it->second.order.order_seq==seq) {
        it->second.next_refresh=0;it->second.deferred_until=0;
    }
}
bool Runtime::defer_refresh(ActorKey key,std::uint64_t expected,TimeMs until) {
    auto r=records_.find(key.id);if(r==records_.end()||r->second.key!=key||r->second.first_pending||
        r->second.order.order_seq!=expected||until<=snapshot_.now_ms)return false;
    const auto a=std::lower_bound(snapshot_.actors.begin(),snapshot_.actors.end(),key.id,
        [](const ActorSnapshot& actor,std::uint32_t id){return actor.key.id<id;});
    const auto& o=r->second.order;
    r->second.deferred_until=0;
    if(a==snapshot_.actors.end()||a->key!=key||!a->alive||a->wounded||a->yield_reasons||
        !(a->flags&flag_admitted)||r->second.released||r->second.release_pending||handback_due(*a,r->second,snapshot_.now_ms)||
        !a->current_position_accessible||a->reloading||
        a->weapon.state==WeaponState::reloading||!a->weapon.ammo_in_mag||a->hits.count||
        o.kind==OrderKind::yield||o.restrictions_revision!=a->restrictions_revision||
        o.geometry_revision!=snapshot_.geometry_revision||o.ownership_generation!=a->ownership_generation) {
        r->second.next_refresh=0;return false;
    }
    const auto t=select_contact(*a,config_,snapshot_.now_ms);
    const bool visible=t&&personally_visible(*t,*a);
    if((o.look.kind==LookKind::enemy)!=visible||
        (visible&&o.look.target!=t->key)||
        (!visible&&t&&distance_squared(o.look.value,t->position)>.0001f)) {
        r->second.next_refresh=0;return false;
    }
    r->second.deferred_until=(std::min)(until,o.expires_at_ms);
    // An open fire window closes on time: a deferred refresh never outlives it
    if(o.fire.allowed)r->second.deferred_until=(std::min)(r->second.deferred_until,o.fire.until_ms);
    r->second.deferred_until=(std::min)(r->second.deferred_until,retention_deadline(*a,r->second,snapshot_.now_ms));
    return true;
}
void Runtime::reset(std::uint64_t epoch) {
    current_species_={};species_release_sides_.clear();
    snapshot_=WorldSnapshot{};snapshot_.level_epoch=epoch;
    records_.clear();covers_.clear();stats_=RuntimeStats{};first_cursor_=refresh_cursor_=0;replan_requested_=false;
    lifecycle_source_sequence_=0;lifecycle_signature_.clear();lifecycle_last_valid_sides_.clear();
}
bool Runtime::ingest(WorldSnapshot incoming) {
    auto reject=[&](){++stats_.stale_inputs;return false;};
    if(!incoming.level_epoch||!incoming.sequence)return reject();
    if(snapshot_.level_epoch&&incoming.level_epoch<snapshot_.level_epoch)return reject();
    if(incoming.level_epoch==snapshot_.level_epoch&&snapshot_.sequence&&
        (incoming.sequence<=snapshot_.sequence||incoming.now_ms<snapshot_.now_ms))return reject();
    std::unordered_set<std::uint32_t> ids;
    for(const auto& a:incoming.actors) {
        if(!valid_key(a.key)||a.level_epoch!=incoming.level_epoch||a.snapshot_seq!=incoming.sequence||
            a.observed_at_ms!=incoming.now_ms||!ids.insert(a.key.id).second)return reject();
    }
    if(incoming.level_epoch!=snapshot_.level_epoch)reset(incoming.level_epoch);
    std::unordered_map<std::uint32_t,SideSnapshot> next_authority;
    std::unordered_set<std::uint32_t> changed_members;
    const bool valid_authority=lifecycle_source_sequence_&&lifecycle_raw_sides(incoming,next_authority);
    if(valid_authority) {
        auto mark=[&](const SideSnapshot& side){for(auto k:side.members)changed_members.insert(k.id);};
        for(const auto& item:next_authority) {
            const auto old=lifecycle_last_valid_sides_.find(item.first);const auto& side=item.second;
            if(old==lifecycle_last_valid_sides_.end()){mark(side);continue;}
            const auto& previous=old->second;
            if(previous.group!=side.group||previous.faction!=side.faction||previous.members!=side.members||
                previous.building_id!=side.building_id||!lifecycle_same_mission(previous.mission,side.mission)){mark(previous);mark(side);}
        }
        for(const auto& item:lifecycle_last_valid_sides_)if(!next_authority.count(item.first))mark(item.second);
    }
    std::sort(incoming.actors.begin(),incoming.actors.end(),[](const ActorSnapshot& a,const ActorSnapshot& b){return a.key.id<b.key.id;});
    for(auto it=records_.begin();it!=records_.end();) {
        if(!ids.count(it->first)){covers_.erase(it->first);it=records_.erase(it);}else ++it;
    }
    for(const auto& a:incoming.actors) {
        auto found=records_.find(a.key.id);
        if(found==records_.end()||found->second.key!=a.key||found->second.owner_generation!=a.ownership_generation) {
            if(!(a.flags&flag_admitted)){records_.erase(a.key.id);covers_.erase(a.key.id);continue;}
            Record r;r.key=a.key;r.owner_generation=a.ownership_generation;r.admitted_at=incoming.now_ms;
            records_[a.key.id]=r;covers_.erase(a.key.id);++stats_.admitted;
        }else {
            auto& r=found->second;
            r.deferred_until=0;
            if(a.yield_reasons!=yield_none||!(a.flags&flag_admitted)||!a.alive||a.wounded||
                r.order.geometry_revision!=incoming.geometry_revision||r.order.restrictions_revision!=a.restrictions_revision)
                r.next_refresh=0;
        }
        auto& r=records_.at(a.key.id);TimeMs evidence=0;
        if(r.has_plan&&r.reload_requested&&!r.reload_complete&&a.weapon.id==r.reload_weapon) {
            const Order* applied=a.last_applied.order_seq==r.order.order_seq?&r.order:
                a.last_applied.order_seq==r.plan.order_seq?&r.plan:nullptr;
            if(applied&&applied->kind==OrderKind::reload&&applied->actor==a.key&&applied->level_epoch==incoming.level_epoch&&
                applied->ownership_generation==a.ownership_generation&&applied->restrictions_revision==a.restrictions_revision&&
                applied->geometry_revision==incoming.geometry_revision&&applied->snapshot_seq<=a.snapshot_seq&&
                applied->goal_node==r.plan.goal_node&&distance_squared(applied->goal_position,r.plan.goal_position)==0&&
                applied->expires_at_ms>incoming.now_ms&&a.last_applied.at_ms>=applied->issued_at_ms&&
                a.last_applied.at_ms<=a.observed_at_ms&&fresh(a.last_applied.at_ms,incoming.now_ms,config_.snapshot_max_age_ms)) {
                if(a.reloading||a.weapon.state==WeaponState::reloading)r.reload_seen=true;
                else if(r.reload_seen){r.reload_complete=true;r.next_refresh=0;}
            }
        }
        if(r.lifecycle_installed) {
            bool changed=r.lifecycle_identity.level_epoch!=incoming.level_epoch||r.lifecycle_identity.map_hash!=incoming.map_hash||
                r.lifecycle_identity.geometry_revision!=incoming.geometry_revision||r.lifecycle_identity.faction!=a.faction||
                r.lifecycle_native_squad!=a.native_squad||changed_members.count(a.key.id)!=0;
            if(changed)invalidate_lifecycle(a.key,a.ownership_generation);
        }
        if(r.has_group_evidence&&(r.evidence_native_group!=a.native_squad||r.evidence_faction!=a.faction)){
            r.has_group_evidence=false;r.group_evidence_at=0;r.next_refresh=0;
        }
        if(personal_evidence_at(a,config_,incoming.now_ms,evidence)&&(!r.has_evidence||evidence>r.last_evidence_at)) {
            r.last_evidence_at=evidence;r.has_evidence=true;
        }
        // Observe episode identity on every authority capture, independently
        // of worker/cheap-decision cadence. This ledger grants no sight/fire.
        const auto* current_contact=select_contact(a,config_,incoming.now_ms);
        r.episode_contact_pending|=observe_episode_contacts(a,r,incoming.now_ms,current_contact);
        for(std::size_t i=0;i<(std::min)(std::size_t(a.hits.count),a.hits.items.size());++i){const auto& h=a.hits.items[i];
            if(h.at_ms<=a.observed_at_ms&&finite(h.dir)&&fresh(h.at_ms,incoming.now_ms,config_.memory_max_age_ms))
                r.pending_episode_hit_at=(std::max)(r.pending_episode_hit_at,h.at_ms);}
        if(r.episode_started){
            const auto* contact=select_contact(a,config_,incoming.now_ms);
            if(contact&&contact->key!=r.episode_target){r.next_refresh=0;replan_requested_=true;}
            for(std::size_t i=0;i<(std::min)(std::size_t(a.hits.count),a.hits.items.size());++i){const auto& h=a.hits.items[i];
                if(h.at_ms>r.episode_hit_at&&h.at_ms<=a.observed_at_ms&&finite(h.dir)&&fresh(h.at_ms,incoming.now_ms,config_.memory_max_age_ms)){
                    r.next_refresh=0;replan_requested_=true;}}
        }
        if(handback_due(a,r,incoming.now_ms))r.next_refresh=0;
        if(r.has_plan&&!plan_compatible(a,r,incoming.now_ms))r.next_refresh=0;
        else if(r.has_plan&&plan_disturbed(a,r,incoming.now_ms)){r.next_refresh=0;replan_requested_=true;}
        if(r.has_plan) {
            const auto target=select_contact(a,config_,incoming.now_ms);
            if((r.order.look.kind==LookKind::enemy)!=(target&&personally_visible(*target,a))||
                (r.order.fire.allowed&&((a.flags&flag_dont_shoot)||a.reloading||
                    a.weapon.state==WeaponState::reloading||!a.weapon.ammo_in_mag)))r.next_refresh=0;
        }
    }
    snapshot_=std::move(incoming);
    if(valid_authority)lifecycle_last_valid_sides_=std::move(next_authority);
    if(!snapshot_.actors.empty()){first_cursor_%=snapshot_.actors.size();refresh_cursor_%=snapshot_.actors.size();}
    else first_cursor_=refresh_cursor_=0;
    stats_.waiting_first_order=0;
    for(const auto& r:records_)if(r.second.first_pending)++stats_.waiting_first_order;
    return true;
}
bool Runtime::install_cover(BootstrapCoverCache cache) {
    if(cache.level_epoch!=snapshot_.level_epoch||cache.geometry_revision!=snapshot_.geometry_revision||
        !fresh(cache.at_ms,snapshot_.now_ms,config_.cover_cache_ms))return false;
    const auto a=std::lower_bound(snapshot_.actors.begin(),snapshot_.actors.end(),cache.actor.id,
        [](const ActorSnapshot& actor,std::uint32_t id){return actor.key.id<id;});
    if(a==snapshot_.actors.end()||a->key!=cache.actor||a->node_id!=cache.origin_node||
        a->restrictions_revision!=cache.restrictions_revision)return false;
    const auto prior=covers_.find(cache.actor.id);
    if(prior!=covers_.end()&&prior->second.at_ms>cache.at_ms)return false;
    covers_[cache.actor.id]=std::move(cache);
    // A completed value job can improve a cheap hold without waiting for the
    // full refresh interval. It never changes the actor's admission age.
    auto r=records_.find(a->key.id);if(r!=records_.end())r->second.next_refresh=0;
    return true;
}
bool Runtime::observe_episode_contacts(const ActorSnapshot& a,Record& r,TimeMs now,const Contact* selected) {
    auto& known=r.episode_contacts;
    known.erase(std::remove_if(known.begin(),known.end(),[&](const Record::EpisodeContact& c){
        return !fresh(c.at,now,config_.memory_max_age_ms);}),known.end());
    const bool newly_selected=selected&&std::none_of(known.begin(),known.end(),[&](const Record::EpisodeContact& c){return c.key==selected->key;});
    auto observe=[&](const Contact& c){
        if(!valid_key(c.key)||c.key==a.key||!c.hostile||!c.alive_when_observed||c.wounded||!finite(c.position)||
           c.observed_at_ms>a.observed_at_ms||c.last_seen_ms>a.observed_at_ms||
           !fresh(a.observed_at_ms,now,config_.snapshot_max_age_ms)||
           (!personally_visible(c,a)&&!fresh(c.observed_at_ms,now,config_.memory_max_age_ms)))return;
        // Current sight is current evidence; memory keeps its original clock.
        // Presence time is only episode bookkeeping. Contact source clocks in
        // the snapshot and every firing/knowledge packet remain unchanged.
        const auto at=a.observed_at_ms;
        auto found=std::find_if(known.begin(),known.end(),[&](const Record::EpisodeContact& old){return old.key.id==c.key.id;});
        if(found==known.end())known.push_back({c.key,at});
        else if(found->key!=c.key)*found={c.key,at};
        else found->at=(std::max)(found->at,at);
    };
    observe(a.enemy);
    if(a.visible_enemies.count<=a.visible_enemies.items.size())
        for(std::size_t i=0;i<a.visible_enemies.count;++i)observe(a.visible_enemies.items[i]);
    return newly_selected;
}
void Runtime::decide(const ActorSnapshot& a,Record& r,TimeMs now,std::vector<Decision>& out) {
    CoverView view;std::vector<Vec3> teammates;
    const auto cached=covers_.find(a.key.id);
    if(cached!=covers_.end()) {
        const auto& c=cached->second;
        if(c.actor==a.key&&c.level_epoch==snapshot_.level_epoch&&c.geometry_revision==snapshot_.geometry_revision&&
            c.restrictions_revision==a.restrictions_revision&&c.origin_node==a.node_id&&fresh(c.at_ms,now,config_.cover_cache_ms)) {
            view.data=c.candidates.data();view.count=c.candidates.size();
            for(const auto& mate:snapshot_.actors) {
                if(mate.key==a.key||mate.faction!=a.faction||!mate.alive)continue;
                teammates.push_back(mate.position);
                const auto held=records_.find(mate.key.id);
                if(held!=records_.end()&&held->second.order.expires_at_ms>now&&
                    held->second.order.actor==mate.key&&held->second.order.ownership_generation==mate.ownership_generation&&
                    held->second.order.pace!=Pace::stand)teammates.push_back(held->second.order.goal_position);
            }
            view.teammate_positions=teammates.data();view.teammate_count=teammates.size();
        }
    }
    const auto* episode_contact=select_contact(a,config_,now);
    TimeMs newest_hit=(std::max)(r.episode_hit_at,r.pending_episode_hit_at);
    for(std::size_t i=0;i<(std::min)(std::size_t(a.hits.count),a.hits.items.size());++i){const auto& h=a.hits.items[i];
        if(finite(h.dir)&&h.at_ms<=a.observed_at_ms&&fresh(h.at_ms,now,config_.memory_max_age_ms))newest_hit=(std::max)(newest_hit,h.at_ms);}
    const bool new_contact=r.episode_contact_pending||observe_episode_contacts(a,r,now,episode_contact);
    const bool initial_episode=!r.episode_started,new_hit=newest_hit>r.episode_hit_at;
    const bool new_episode=initial_episode||new_contact||new_hit;
    r.episode_contact_pending=false;
    if(episode_contact)r.episode_target=episode_contact->key;
    r.episode_hit_at=newest_hit;r.episode_started=true;
    if(r.has_plan&&r.plan.expires_at_ms<=saturated_after(now,(std::max)(TimeMs(1),config_.refresh_ms)))renew_ordinary_plan(a,r,now);
    auto order=first_response(a,config_,now,snapshot_.level_epoch,snapshot_.geometry_revision,allocate_order_sequence(),view);
    // Equipping the reserved grenade truthfully removes the rifle from the
    // capture. Preserve this accepted transaction, not a synthetic rifle.
    const bool throw_continues=r.has_plan&&continuing_throw(r.plan,a,now)&&plan_compatible(a,r,now);
    if(throw_continues) {
        const auto seq=order.order_seq;order=r.plan;order.order_seq=seq;
        order.snapshot_seq=a.snapshot_seq;order.issued_at_ms=now;order.fire={};
    }
    if(order.kind!=OrderKind::yield&&handback_due(a,r,now))order.kind=OrderKind::yield;
    if(!throw_continues&&order.kind!=OrderKind::yield&&plan_compatible(a,r,now)) {
        const auto response=order;order=r.plan;
        // Native acceptance precedes weapon work. Only observed completion
        // turns the old reload destination back into ordinary cover movement.
        if(r.has_cover&&order.kind==OrderKind::reload&&r.reload_complete&&response.kind!=OrderKind::reload)order.kind=OrderKind::cover;
        const bool covering=keep_room_cover_fire(r.plan,response,a,config_,now);
        // Movement/stance/assignment keep the accepted plan's original lease.
        // Only fresh own perception is used to refresh look and fire intent.
        order.snapshot_seq=a.snapshot_seq;order.order_seq=response.order_seq;order.issued_at_ms=now;
        // Without current own sight, keep the room's authored sector/pivot
        // fallback. A cheap refresh must not replace a sweep with path-facing.
        // A fresh outdoor plan can deliberately watch a K1 memory point after
        // the short bootstrap contact TTL has elapsed. Its neutral fallback
        // direction is not new evidence and must not erase that authored look.
        // Current sight, fresh personal memory and a new hit still interrupt.
        const bool personal_look=response.look.kind==LookKind::enemy||
            (!room_assignment(order)&&(response.look.kind==LookKind::point||new_hit));
        if(!covering&&personal_look)order.look=response.look;
        else if(!covering) {
            // A room plan may have arrived with a current enemy look. Without
            // an explicitly supplied authored fallback, never retain that
            // stale target. Do not borrow a sector from an older assignment.
            if(order.look.kind==LookKind::enemy)order.look=response.look;
            const Hit* latest=nullptr;
            const auto hit_count=(std::min)(std::size_t(a.hits.count),a.hits.items.size());
            for(std::size_t i=0;i<hit_count;++i) {
                const auto& hit=a.hits.items[i];
                if(finite(hit.dir)&&hit.at_ms>r.plan_hit_at&&hit.at_ms<=a.observed_at_ms&&
                    fresh(hit.at_ms,now,config_.memory_max_age_ms)&&(!latest||hit.at_ms>latest->at_ms))latest=&hit;
            }
            if(latest) {
                const auto& v=latest->dir;const float norm=v.x*v.x+v.y*v.y+v.z*v.z;
                if(std::isfinite(norm)&&norm>1.e-10f) {
                    const float k=1.f/std::sqrt(norm);order.look={LookKind::direction,{v.x*k,v.y*k,v.z*k},{}};
                }
            }
        }
        if(covering) {
            // Keep the original evidence-bound deadline, queue and interval.
            // Fresh own sight never renews this packet's suppression window.
            order.fire.until_ms=(std::min)(order.expires_at_ms,r.plan.fire.until_ms);
            order.flags&=~order_first_response;
        } else {
            order.fire.allowed=response.fire.allowed;
            order.fire.at={};order.fire.at_valid=false;order.fire.until_ms=(std::min)(order.expires_at_ms,response.fire.until_ms);
            order.flags&=~(order_first_response|order_cover_fire);
        }
        if(valid_key(order.finish)) {
            const Contact* target=a.enemy.key==order.finish?&a.enemy:nullptr;
            if(!target&&a.visible_enemies.count<=a.visible_enemies.items.size())for(std::size_t i=0;i<a.visible_enemies.count;++i)
                if(a.visible_enemies.items[i].key==order.finish){target=&a.visible_enemies.items[i];break;}
            const bool finish=order.role==roles::finisher&&target&&target->hostile&&target->alive_when_observed&&
                target->wounded&&target->finish_allowed&&personally_visible(*target,a)&&finite(target->position)&&
                fresh(a.observed_at_ms,now,config_.snapshot_max_age_ms)&&!(a.flags&flag_dont_shoot)&&
                !a.reloading&&a.weapon.state!=WeaponState::reloading&&a.weapon.ammo_in_mag;
            order.fire={};if(finish){order.look={LookKind::enemy,target->position,target->key};order.fire.allowed=config_.fire_window_ms!=0;
                order.fire.until_ms=fire_window_until(config_,now,order.expires_at_ms);}
        }
        if(valid_key(order.aid)&&order.role==roles::aid_casualty){order.fire={};order.look=r.plan.look;}
        if(order.kind==OrderKind::reload)order.fire={};
        if(grenade_movement(a,r,now)){order.fire={};order.look=r.plan.look;}
        // The plan has now seen these facts: the newest hit and the threat it aims at no longer disturb it
        if(a.hits.count<=a.hits.items.size())
            for(std::size_t i=0;i<a.hits.count;++i)
                if(finite(a.hits.items[i].dir)&&a.hits.items[i].at_ms<=now)r.plan_hit_at=(std::max)(r.plan_hit_at,a.hits.items[i].at_ms);
        if(const auto* t=select_contact(a,config_,now))r.plan_target=t->key;
    }else if(!throw_continues){r.has_plan=false;r.has_cover=false;r.cover=AcceptedCoverProof{};}
    if(!new_episode)order.flags&=~order_first_response;
    if(r.first_pending) {
        const auto delay=now>=r.admitted_at?now-r.admitted_at:0;
        stats_.max_first_order_ms=(std::max)(stats_.max_first_order_ms,delay);
        if(delay>config_.first_order_deadline_ms)++stats_.missed_first_deadline;
        r.first_pending=false;if(stats_.waiting_first_order)--stats_.waiting_first_order;
    }
    order.gait=order_gait(order);
    r.order=order;r.deferred_until=0;r.release_pending=order.kind==OrderKind::yield;
    r.next_refresh=saturated_after(now,(std::max)(TimeMs(1),config_.refresh_ms));
    if(!r.release_pending)r.next_refresh=(std::min)(r.next_refresh,retention_deadline(a,r,now));
    if(r.has_plan)r.next_refresh=(std::min)(r.next_refresh,r.plan.expires_at_ms);
    if(order.fire.allowed)r.next_refresh=(std::min)(r.next_refresh,order.fire.until_ms);
    ++stats_.decisions;if(order.kind==OrderKind::yield)++stats_.releases;
    // First-response envelopes have side_id0: actors are independent until
    // a squad job publishes a coordinated Decision with its real side id.
    // One actor per envelope avoids accidental partial squad-plan claims.
    Decision result;result.level_epoch=snapshot_.level_epoch;result.snapshot_seq=snapshot_.sequence;
    result.geometry_revision=snapshot_.geometry_revision;
    if(r.release_pending){result.releases.push_back(a.key);result.events.push_back({events::hand_back,a.room_id,a.key.id});}
    else {result.orders.push_back(order);if(new_episode){
        stats_.episode_admission+=initial_episode;stats_.episode_contact+=new_contact;stats_.episode_hit+=new_hit;
        result.events.push_back({events::first_response,a.room_id,a.key.id});}}
    out.push_back(std::move(result));
}
std::size_t Runtime::step(TimeMs now,std::size_t max_steps,std::vector<Decision>& out,
    bool(*deadline)(void*),void* context)
{
    const auto n=snapshot_.actors.size();if(!n||!max_steps||now<snapshot_.now_ms)return 0;
    std::size_t count=0;
    for(unsigned pass=0;pass<2;++pass) {
        if(pass==0&&!stats_.waiting_first_order)continue;
        auto& cursor=pass==0?first_cursor_:refresh_cursor_;
        for(std::size_t visited=0;visited<n;++visited) {
            if(count>=max_steps||(deadline&&deadline(context)))return count;
            const auto& a=snapshot_.actors[cursor];cursor=(cursor+1)%n;
            auto found=records_.find(a.key.id);if(found==records_.end())continue;
            auto& r=found->second;
            if(r.released)continue;
            if(pass==0?!r.first_pending:(r.first_pending||(std::max)(r.next_refresh,r.deferred_until)>now))continue;
            decide(a,r,now,out);++count;
        }
    }
    return count;
}
}}}
