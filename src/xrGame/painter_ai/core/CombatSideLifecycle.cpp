#include "CombatSideLifecycle.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <tuple>

namespace painter { namespace combat { namespace core {
namespace {
struct Interrupted{};
void poll(const std::function<bool()>& stop){if(stop&&stop())throw Interrupted{};}
struct KeyLess {bool operator()(ActorKey a,ActorKey b)const{return std::tie(a.id,a.generation)<std::tie(b.id,b.generation);}};
bool key(ActorKey a){return a.id<invalid_id&&a.generation;}
bool finite3(Vec3 a){return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z);}
TimeMs later(TimeMs a,TimeMs d){return d>(std::numeric_limits<TimeMs>::max)()-a?(std::numeric_limits<TimeMs>::max)():a+d;}
bool same(const LifecycleIdentity& a,const LifecycleIdentity& b){return a.level_epoch==b.level_epoch&&a.map_hash==b.map_hash&&a.geometry_revision==b.geometry_revision&&
    a.generation==b.generation&&a.group==b.group&&a.side==b.side&&a.faction==b.faction&&a.mission_revision==b.mission_revision;}
bool same_mission(const Mission& a,const Mission& b){if(a.revision!=b.revision||a.kind!=b.kind||a.building!=b.building||a.entry_portal!=b.entry_portal||a.rooms.count!=b.rooms.count)return false;
    for(std::size_t i=0;i<a.rooms.count;++i)if(a.rooms.items[i]!=b.rooms.items[i])return false;return true;}
bool same_roster(const std::vector<LifecycleLease>& a,const std::vector<LifecycleLease>& b){if(a.size()!=b.size())return false;
    for(std::size_t i=0;i<a.size();++i)if(a[i].actor!=b[i].actor||a[i].ownership_generation!=b[i].ownership_generation)return false;return true;}
bool admitted(const ActorSnapshot& a){return key(a.key)&&a.ownership_generation&&(a.flags&flag_admitted)&&a.yield_reasons==yield_none&&a.alive&&!a.wounded;}
bool valid_policy(const SideLifecyclePolicy& p){return p.quiet_field_ms&&p.quiet_building_ms&&p.stand_down_after_ms<=p.quiet_field_ms&&p.stand_down_after_ms<=p.quiet_building_ms;}
}
bool requested_authored_mission(const WorldSnapshot& world,const SideSnapshot& side){
    if(side.mission.kind==MissionKind::none||side.members.empty())return false;
    // Generated mission suggestions cannot borrow a test/script ownership
    // request: the raw captured side must author the exact current objective.
    const auto raw=std::find_if(world.sides.begin(),world.sides.end(),[&](const SideSnapshot& s){return s.side_id==side.side_id;});
    if(raw==world.sides.end()||raw->group!=side.group||raw->faction!=side.faction||
        !same_mission(raw->mission,side.mission))return false;
    // The registry may retain an absent roster key while the raw mission
    // omits it (or vice versa). Match every captured living member; an absent
    // key grants neither a control lease nor evidence that the member died.
    auto living=[&](const SideSnapshot& row){std::vector<ActorKey> keys;
        for(auto key:row.members){const auto actor=std::find_if(world.actors.begin(),world.actors.end(),
            [&](const ActorSnapshot& a){return a.key==key;});
            if(actor!=world.actors.end()&&actor->alive)keys.push_back(key);}
        return keys;};
    if(living(*raw)!=living(side))return false;
    bool requested=false;
    for(auto key:side.members){
        const auto member=std::find_if(world.actors.begin(),world.actors.end(),[&](const ActorSnapshot& a){return a.key==key;});
        if(member==world.actors.end())continue; // Absent members grant no control lease.
        if(!member->alive||member->wounded||!(member->flags&flag_admitted))continue;
        if(!(member->flags&flag_requested_control))return false;
        requested=true;
    }
    return requested;
}
struct CombatSideLifecycle::Impl {
    struct Side {
        LifecycleIdentity identity;Mission mission;std::vector<LifecycleLease> roster,identity_roster;
        LifecycleEvidence evidence;TimeMs activation=0,release_at=0,cooldown_until=0;
        std::uint64_t identity_sequence=0;TimeMs identity_at=0;
        SideReleaseReason reason=SideReleaseReason::none;
        std::uint64_t native_group=0;
        std::uint32_t blocked_portal=0;TimeMs door_blocked_at=0;
    };
    struct Released {LifecycleLease member;LifecycleIdentity identity;SideReleaseReason reason;TimeMs at=0,cooldown_until=0;};
    std::map<std::uint32_t,Side> sides;
    std::map<std::uint32_t,Released> released; // one current/latest released lease per engine object ID
    std::map<ActorKey,std::set<TimeMs>,KeyLess> friendly_attacks;
    std::map<ActorKey,std::map<ActorKey,std::set<TimeMs>,KeyLess>,KeyLess> friendly_sources;
    std::uint64_t epoch=0,map=0,sequence=0,next_generation=0;TimeMs now=0;
    std::unique_ptr<Impl> copy(const std::function<bool()>& stop)const{
        poll(stop);std::unique_ptr<Impl> n(new Impl);n->epoch=epoch;n->map=map;n->sequence=sequence;n->next_generation=next_generation;n->now=now;
        for(const auto& p:sides){poll(stop);Side s;s.identity=p.second.identity;s.mission=p.second.mission;s.evidence=p.second.evidence;s.activation=p.second.activation;
            s.release_at=p.second.release_at;s.cooldown_until=p.second.cooldown_until;s.reason=p.second.reason;s.identity_sequence=p.second.identity_sequence;s.identity_at=p.second.identity_at;
            s.native_group=p.second.native_group;s.blocked_portal=p.second.blocked_portal;s.door_blocked_at=p.second.door_blocked_at;
            for(auto lease:p.second.roster){poll(stop);s.roster.push_back(lease);}
            for(auto lease:p.second.identity_roster){poll(stop);s.identity_roster.push_back(lease);}
            n->sides.emplace(p.first,std::move(s));}
        for(const auto& p:released){poll(stop);n->released.emplace(p);}
        for(const auto& p:friendly_attacks){poll(stop);auto& times=n->friendly_attacks[p.first];for(auto at:p.second){poll(stop);times.insert(at);}}
        for(const auto& observer:friendly_sources){poll(stop);auto& target=n->friendly_sources[observer.first];for(const auto& source:observer.second){poll(stop);
            auto& times=target[source.first];for(auto at:source.second){poll(stop);times.insert(at);}}}poll(stop);return n;
    }
};
const char* lifecycle_release_reason(SideReleaseReason r){
    switch(r){case SideReleaseReason::none:return "none";case SideReleaseReason::fight_over:return "fight_over";
    case SideReleaseReason::building_cleared:return "building_cleared";case SideReleaseReason::no_living_men:return "no_living_men";
    case SideReleaseReason::error:return "error";case SideReleaseReason::identity_reset:return "identity_reset";
    case SideReleaseReason::monster_fight:return "monster_fight";case SideReleaseReason::out_of_range:return "out_of_range";
    case SideReleaseReason::door_stuck:return "door_stuck";}return "invalid";
}
bool lifecycle_admission_blocked(const std::vector<LifecycleCooldown>& rows,const ActorSnapshot& a,
    std::uint64_t epoch,std::uint64_t map,TimeMs now){
    if(!key(a.key)||!epoch||a.level_epoch!=epoch)return false;
    for(const auto& c:rows){if(c.identity.level_epoch!=epoch||c.identity.map_hash!=map||c.identity.faction!=a.faction||
        c.reason==SideReleaseReason::none||now>=c.until_ms)continue;
        if(c.native_group&&a.native_squad==c.native_group)return true;
        if(std::find(c.members.begin(),c.members.end(),a.key)!=c.members.end())return true;
    }return false;
}
CombatSideLifecycle::CombatSideLifecycle(SideLifecyclePolicy p,const Config& c):policy_(p),impl_(new Impl),
    release_range_m_(c.lifecycle_release_range_m),distance_cost_per_m_(c.lifecycle_distance_cost_per_m),
    required_door_stuck_ms_(c.lifecycle_required_door_stuck_ms){}
CombatSideLifecycle::~CombatSideLifecycle()=default;
void CombatSideLifecycle::reset(){impl_.reset(new Impl);}
std::unique_ptr<CombatSideLifecycle> CombatSideLifecycle::clone(const std::function<bool()>& stop)const{
    try{auto state=impl_->copy(stop);std::unique_ptr<CombatSideLifecycle> n(new CombatSideLifecycle(policy_));n->impl_=std::move(state);
        n->release_range_m_=release_range_m_;n->distance_cost_per_m_=distance_cost_per_m_;n->required_door_stuck_ms_=required_door_stuck_ms_;poll(stop);return n;}
    catch(const Interrupted&){return nullptr;}
}
SideLifecycleResult CombatSideLifecycle::update(const WorldSnapshot& w,const SideRegistryResult& registry,const KnowledgeFrame* knowledge,
    const LifecycleSignals& signals,const std::function<bool()>& stop){
    SideLifecycleResult out;out.level_epoch=w.level_epoch;out.map_hash=w.map_hash;out.snapshot_seq=w.sequence;out.captured_at_ms=w.now_ms;
    try{
        poll(stop);if(!valid_policy(policy_)||!std::isfinite(release_range_m_)||release_range_m_<0||
            !std::isfinite(distance_cost_per_m_)||distance_cost_per_m_<0||!w.level_epoch||!w.sequence||!registry.accepted||registry.status!=SideRegistryStatus::accepted||registry.cancelled||
            registry.level_epoch!=w.level_epoch||registry.map_hash!=w.map_hash||registry.snapshot_seq!=w.sequence||registry.captured_at_ms!=w.now_ms)return out;
        if(impl_->epoch&&(w.level_epoch<impl_->epoch||(w.level_epoch==impl_->epoch&&(w.sequence<=impl_->sequence||w.now_ms<impl_->now)))){out.status=SideLifecycleStatus::stale;return out;}
        if(knowledge&&(!knowledge->accepted||knowledge->cancelled||knowledge->rejection!=KnowledgeReject::none||knowledge->level_epoch!=w.level_epoch||knowledge->sequence!=w.sequence))return out;
        std::map<ActorKey,const ActorSnapshot*,KeyLess> actors;std::set<std::uint32_t> actor_ids;
        for(const auto& a:w.actors){poll(stop);if(!key(a.key)||!actor_ids.insert(a.key.id).second||a.level_epoch!=w.level_epoch||a.snapshot_seq!=w.sequence||a.observed_at_ms>w.now_ms||
            a.visible_enemies.count>a.visible_enemies.items.size()||a.hits.count>a.hits.items.size()||a.heard.count>a.heard.items.size()||a.near_misses.count>a.near_misses.items.size()||a.seen_dead.count>a.seen_dead.items.size())return out;
            actors.emplace(a.key,&a);}
        std::set<std::uint32_t> side_ids;std::map<ActorKey,std::uint32_t,KeyLess> memberships;
        for(const auto& side:registry.sides){poll(stop);if(!side.side_id||!side_ids.insert(side.side_id).second||side.mission.rooms.count>side.mission.rooms.items.size()||
            static_cast<unsigned>(side.mission.kind)>static_cast<unsigned>(MissionKind::hold))return out;
            for(auto member:side.members){poll(stop);if(!key(member)||!memberships.emplace(member,side.side_id).second)return out;auto a=actors.find(member);if(a!=actors.end()&&a->second->faction!=side.faction)return out;}}
        if(impl_->epoch==w.level_epoch&&impl_->map==w.map_hash)for(const auto& side:registry.sides){poll(stop);auto old=impl_->sides.find(side.side_id);
            if(old==impl_->sides.end()||old->second.identity.group!=side.group||old->second.identity.faction!=side.faction)continue;
            bool reset=false;for(const auto& r:registry.resets){poll(stop);if(r.side==side.side_id)reset=true;}
            if(!reset&&(side.mission.revision<old->second.mission.revision||
                (side.mission.revision==old->second.mission.revision&&!same_mission(side.mission,old->second.mission))))return out;}
        std::map<ActorKey,const MemberKnowledge*,KeyLess> known;
        if(knowledge)for(const auto& m:knowledge->members){poll(stop);if(!key(m.actor)||!known.emplace(m.actor,&m).second)return out;}
        auto known_friendly=[&](const ActorSnapshot& a,ActorKey source){
            if(!key(source))return false;if(source==a.key)return true;
            const auto own_side=memberships.find(a.key),other_side=memberships.find(source);
            if(own_side!=memberships.end()&&other_side!=memberships.end()&&own_side->second==other_side->second)return true;
            bool have_relation=false,is_friendly=false;TimeMs relation_at=0;
            auto relation=[&](const Contact& c){if(c.key!=source||!finite3(c.position)||c.observed_at_ms>a.observed_at_ms)return;
                if(!have_relation||c.observed_at_ms>relation_at||(c.observed_at_ms==relation_at&&!c.hostile)){
                    have_relation=true;relation_at=c.observed_at_ms;is_friendly=!c.hostile;}};
            relation(a.enemy);for(std::size_t i=0;i<a.visible_enemies.count;++i)relation(a.visible_enemies.items[i]);return have_relation&&is_friendly;
        };
        auto staged=impl_->copy(stop);auto& state=*staged;
        if(state.epoch&&(state.epoch!=w.level_epoch||state.map!=w.map_hash)){state.sides.clear();state.released.clear();state.friendly_attacks.clear();state.friendly_sources.clear();state.next_generation=0;}
        state.epoch=w.level_epoch;state.map=w.map_hash;state.sequence=w.sequence;state.now=w.now_ms;
        const auto keep_friendly=(std::max)(policy_.quiet_field_ms,policy_.quiet_building_ms);
        for(auto it=state.friendly_attacks.begin();it!=state.friendly_attacks.end();){poll(stop);auto& times=it->second;
            while(!times.empty()&&w.now_ms>*times.begin()&&w.now_ms-*times.begin()>keep_friendly){poll(stop);times.erase(times.begin());}
            if(times.empty())it=state.friendly_attacks.erase(it);else ++it;}
        for(auto observer=state.friendly_sources.begin();observer!=state.friendly_sources.end();){poll(stop);
            for(auto source=observer->second.begin();source!=observer->second.end();){poll(stop);auto& times=source->second;
                while(!times.empty()&&w.now_ms>*times.begin()&&w.now_ms-*times.begin()>keep_friendly){poll(stop);times.erase(times.begin());}
                if(times.empty())source=observer->second.erase(source);else ++source;}
            if(observer->second.empty())observer=state.friendly_sources.erase(observer);else ++observer;}
        // Friendly contradictions are checked against attributed K1 foes
        // and observed contacts below, never the engine's private hit source.
        for(const auto& a:actors){poll(stop);auto old=state.released.find(a.first.id);if(old!=state.released.end()&&
            (old->second.member.actor!=a.first||old->second.member.ownership_generation!=a.second->ownership_generation))state.released.erase(old);}
        std::set<std::uint32_t> reset_sides;for(const auto& r:registry.resets){poll(stop);reset_sides.insert(r.side);}
        for(const auto& side:registry.sides){poll(stop);auto previous=state.sides.find(side.side_id);const bool existed=previous!=state.sides.end();
            bool retained_identity=false;if(existed)for(const auto& m:previous->second.roster){poll(stop);
                if(std::find(side.members.begin(),side.members.end(),m.actor)!=side.members.end()){retained_identity=true;break;}}
            const bool replaced=existed&&(previous->second.identity.group!=side.group||previous->second.identity.faction!=side.faction||
                reset_sides.count(side.side_id)||(!previous->second.roster.empty()&&!side.members.empty()&&!retained_identity));
            if(replaced)state.sides.erase(previous);
            auto inserted=state.sides.try_emplace(side.side_id);auto& record=inserted.first->second;
            if(inserted.second){record.activation=w.now_ms;record.mission=side.mission;}
            std::vector<LifecycleLease> roster,identity_roster;bool roster_complete=true,living=false;
            for(auto member:side.members){poll(stop);auto a=actors.find(member);roster.push_back({member,a==actors.end()?0:a->second->ownership_generation});
                if(a==actors.end())roster_complete=false;else if(a->second->alive){living=true;identity_roster.push_back({member,a->second->ownership_generation});}}
            const bool mission_changed=!same_mission(record.mission,side.mission);
            if(inserted.second||mission_changed||record.identity.geometry_revision!=w.geometry_revision||!same_roster(record.identity_roster,identity_roster)){
                if(state.next_generation==(std::numeric_limits<std::uint64_t>::max)()){out.status=SideLifecycleStatus::failed;return out;}
                record.identity={w.level_epoch,w.map_hash,w.geometry_revision,++state.next_generation,side.group,side.side_id,side.faction,side.mission.revision};
                record.identity_sequence=w.sequence;record.identity_at=w.now_ms;
                record.blocked_portal=0;record.door_blocked_at=0;
            }
            record.roster=std::move(roster);record.identity_roster=std::move(identity_roster);record.mission=side.mission;
            std::uint64_t native_group=0;bool native_consistent=true;
            for(auto member:side.members){poll(stop);auto a=actors.find(member);if(a==actors.end())continue;
                const auto g=a->second->native_squad;if(!g||g!=side.group){native_consistent=false;break;}
                if(native_group&&native_group!=g){native_consistent=false;break;}native_group=g;}
            if(native_consistent&&native_group)record.native_group=native_group;
            else if(roster_complete)record.native_group=0;
            const bool building=side.building_id!=invalid_room||side.mission.building!=invalid_room;
            const TimeMs quiet=building?policy_.quiet_building_ms:policy_.quiet_field_ms;
            auto remember=[&](LifecycleEvidenceKind kind,const ActorSnapshot& observer,FoeId foe,TimeMs at,ActorKey reporter){
                if(at>observer.observed_at_ms||at>w.now_ms)return;
                if(foe.ghost()){auto rejected=state.friendly_attacks.find(reporter);if(rejected!=state.friendly_attacks.end()&&rejected->second.count(at))return;}
                else if(kind!=LifecycleEvidenceKind::contact&&kind!=LifecycleEvidenceKind::own_seen&&kind!=LifecycleEvidenceKind::reported_seen){
                    const auto rejected=state.friendly_sources.find(reporter);if(rejected!=state.friendly_sources.end()){
                        const auto target=rejected->second.find(foe.actor);if(target!=rejected->second.end()&&target->second.count(at))return;}}
                if(!record.evidence.valid||at>record.evidence.at_ms)record.evidence={true,kind,observer.key,reporter,observer.ownership_generation,foe,at,foe.ghost()};
            };
            for(const auto member:side.members){poll(stop);auto found=actors.find(member);if(found==actors.end()||!found->second->alive)continue;const auto& a=*found->second;
                const MemberKnowledge* km=nullptr;auto k=known.find(member);if(k!=known.end()){
                    if(k->second->side==side.side_id&&k->second->ownership_generation==a.ownership_generation&&k->second->alive)km=k->second;else ++out.ignored_knowledge_rows;}
                auto dead=[&](ActorKey target){for(std::size_t i=0;i<a.seen_dead.count;++i)if(a.seen_dead.items[i]==target)return true;return false;};
                auto valid_contact=[&](const Contact& c){return key(c.key)&&!known_friendly(a,c.key)&&c.hostile&&c.alive_when_observed&&!c.wounded&&finite3(c.position)&&c.observed_at_ms<=a.observed_at_ms&&!dead(c.key);};
                auto contact=[&](const Contact& c){if(valid_contact(c)){FoeId id;id.actor=c.key;remember(LifecycleEvidenceKind::contact,a,id,c.observed_at_ms,a.key);}};
                contact(a.enemy);for(std::size_t i=0;i<a.visible_enemies.count;++i)contact(a.visible_enemies.items[i]);
                if(km)for(const auto& f:km->foes){poll(stop);if(!f.id.valid()||f.down||f.wounded||(!f.id.ghost()&&(dead(f.id.actor)||known_friendly(a,f.id.actor))))continue;
                    for(std::size_t i=0;i<f.sources.size();++i){const auto& src=f.sources[i];if(!src.valid||!finite3(src.position)||!key(src.reporter)||(i<2&&src.reporter!=a.key))continue;
                        const LifecycleEvidenceKind kinds[]={LifecycleEvidenceKind::own_seen,LifecycleEvidenceKind::own_heard,LifecycleEvidenceKind::reported_seen,LifecycleEvidenceKind::reported_heard};
                        remember(kinds[i],a,f.id,src.at_ms,src.reporter);}}
                if(km) {
                    // K1 already did cone attribution. Its ghost remains a
                    // ghost even when the engine knows the unseen shooter's ID.
                    const auto& hit=km->hit_from;
                    if(hit.valid&&hit.shooter.valid()&&finite3(hit.point)&&finite3(hit.direction)&&
                        (hit.shooter.ghost()||(!known_friendly(a,hit.shooter.actor)&&!dead(hit.shooter.actor))))
                        remember(LifecycleEvidenceKind::hit,a,hit.shooter,hit.at_ms,a.key);
                }
            }
            auto receipt_valid=[&](const LifecycleReceiptStamp& r){return same(r.identity,record.identity)&&r.source_snapshot>=record.identity_sequence&&r.source_snapshot<=w.sequence&&r.at_ms>=record.identity_at&&r.at_ms<=w.now_ms;};
            SideReleaseReason release=SideReleaseReason::none;
            bool distance_known=false;float distance_m=0;
            const LifecycleRangeReceipt* range=nullptr;bool range_duplicate=false;
            for(const auto& r:signals.ranges){poll(stop);if(r.stamp.identity.side!=side.side_id)continue;
                if(!receipt_valid(r.stamp)||r.stamp.source_snapshot!=w.sequence||r.stamp.at_ms!=w.now_ms||!finite3(r.reference_position)){++out.ignored_receipts;continue;}
                if(range)range_duplicate=true;range=&r;}
            if(range&&!range_duplicate&&roster_complete&&living){
                float nearest=(std::numeric_limits<float>::max)();bool valid=true;
                for(auto member:side.members){poll(stop);const auto& a=*actors.at(member);if(!a.alive)continue;
                    const auto dx=a.position.x-range->reference_position.x,dy=a.position.y-range->reference_position.y,dz=a.position.z-range->reference_position.z;
                    const auto d=dx*dx+dy*dy+dz*dz;if(!finite3(a.position)||!std::isfinite(d)){valid=false;break;}nearest=(std::min)(nearest,d);}
                if(valid){distance_known=true;distance_m=std::sqrt(nearest);if(distance_m>release_range_m_)release=SideReleaseReason::out_of_range;}
            }
            const LifecycleDoorReceipt* required=nullptr;bool door_duplicate=false;
            for(const auto& r:signals.required_doors){poll(stop);if(r.stamp.identity.side!=side.side_id)continue;
                bool applied=false;const auto a=actors.find(r.actor);const auto membership=memberships.find(r.actor);
                if(r.applied_order_seq&&a!=actors.end()&&membership!=memberships.end()&&membership->second==side.side_id&&
                    a->second->ownership_generation==r.ownership_generation&&admitted(*a->second)&&
                    a->second->last_applied.order_seq==r.applied_order_seq&&a->second->last_applied.at_ms<=w.now_ms)
                    applied=true;
                if(!receipt_valid(r.stamp)||r.stamp.source_snapshot!=w.sequence||r.stamp.at_ms!=w.now_ms||!r.portal||
                    side.mission.kind!=MissionKind::clear_building||(!applied&&r.portal!=side.mission.entry_portal)){++out.ignored_receipts;continue;}
                if(required)door_duplicate=true;required=&r;}
            const Door* door=nullptr;bool duplicate_door=false;
            if(required&&!door_duplicate)for(const auto& d:w.doors){poll(stop);if(d.portal==required->portal){if(door)duplicate_door=true;door=&d;}}
            const bool blocked=door&&!duplicate_door&&door->id!=invalid_id&&std::isfinite(door->open)&&
                door->open==0.f&&door->state==DoorState::closed;
            if(blocked){if(record.blocked_portal!=required->portal){record.blocked_portal=required->portal;record.door_blocked_at=w.now_ms;}
                if(w.now_ms>=later(record.door_blocked_at,required_door_stuck_ms_))release=SideReleaseReason::door_stuck;
            }else {record.blocked_portal=0;record.door_blocked_at=0;}
            for(const auto& r:signals.completed){poll(stop);if(r.stamp.identity.side!=side.side_id)continue;
                if(receipt_valid(r.stamp)&&r.whole_building&&side.mission.kind==MissionKind::clear_building&&r.building==side.mission.building&&r.building!=invalid_room)release=SideReleaseReason::building_cleared;else ++out.ignored_receipts;}
            for(const auto& r:signals.errors){poll(stop);if(r.identity.side!=side.side_id)continue;if(receipt_valid(r))release=SideReleaseReason::error;else ++out.ignored_receipts;}
            for(const auto& r:signals.identity_resets){poll(stop);if(r.identity.side!=side.side_id)continue;if(receipt_valid(r))release=SideReleaseReason::identity_reset;else ++out.ignored_receipts;}
            // Registry reuse retires old state; it is NOT a release request for
            // the new group's valid leases. Only an authenticated signal may
            // request identity_reset hand-back for the current identity.
            if(roster_complete&&!living)release=SideReleaseReason::no_living_men;
            PerceivedFight fight;
            if(signals.perceived_species.memory_max_age_ms){
                Config species_config;species_config.memory_max_age_ms=signals.perceived_species.memory_max_age_ms;
                fight=perceived_fight(w,side,knowledge,signals.perceived_species,species_config,stop);
                if(fight.cancelled)throw Interrupted{};
                poll(stop);
            }
            // Replay may complete after a newer capture. Species-only release
            // is not an executed fact: while every affected original lease is
            // still admitted, require current positive classification. A real
            // native release changes its lease; that cooldown is not revoked.
            if(record.reason==SideReleaseReason::monster_fight&&(!fight.valid||!fight.monster_only)){
                bool have=false,unexecuted=true;
                for(const auto& old:state.released){poll(stop);if(old.second.identity.side!=side.side_id||
                    old.second.reason!=SideReleaseReason::monster_fight)continue;
                    have=true;auto current=actors.find(old.second.member.actor);
                    if(current==actors.end()||!admitted(*current->second)||current->second->ownership_generation!=old.second.member.ownership_generation)
                        unexecuted=false;
                }
                if(have&&unexecuted){
                    for(auto old=state.released.begin();old!=state.released.end();){poll(stop);
                        if(old->second.identity.side==side.side_id&&old->second.reason==SideReleaseReason::monster_fight)old=state.released.erase(old);else ++old;}
                    record.reason=SideReleaseReason::none;record.release_at=record.cooldown_until=0;
                }
            }
            if(release==SideReleaseReason::none&&fight.valid&&fight.monster_only)release=SideReleaseReason::monster_fight;
            const TimeMs origin=record.evidence.valid?record.evidence.at_ms:record.activation;
            const TimeMs deadline=later(origin,quiet);
            // Current explicit control requests retain unfinished missions
            // even when the legacy long-lived-mission setting is off. A
            // revoked request or incomplete roster returns to quiet expiry.
            const bool authored=side.mission.kind!=MissionKind::none&&
                (policy_.retain_authored_mission_without_evidence||requested_authored_mission(w,side));
            if(release==SideReleaseReason::none&&w.now_ms>=deadline&&!authored)release=SideReleaseReason::fight_over;
            // A new admitted lease can be considered only after side cooldown.
            // This does not itself admit or request any stock actor.
            if(record.reason!=SideReleaseReason::none&&w.now_ms>=record.cooldown_until){
                bool new_lease=false;for(auto member:side.members){poll(stop);auto a=actors.find(member);if(a==actors.end()||!admitted(*a->second))continue;
                    const auto old=state.released.find(member.id);if(old==state.released.end()||old->second.member.actor!=member||old->second.member.ownership_generation!=a->second->ownership_generation)new_lease=true;}
                if(new_lease&&release==SideReleaseReason::none){record.reason=SideReleaseReason::none;record.release_at=record.cooldown_until=0;}
            }
            if(record.reason==SideReleaseReason::none&&release!=SideReleaseReason::none){record.reason=release;record.release_at=w.now_ms;
                record.cooldown_until=later(w.now_ms,release==SideReleaseReason::error?policy_.error_cooldown_ms:policy_.side_cooldown_ms);}
            SideLifecycleView view;view.identity=record.identity;view.evidence=record.evidence;view.activated_at_ms=record.activation;view.quiet_origin_ms=origin;view.quiet_deadline_ms=deadline;
            view.cooldown_until_ms=record.cooldown_until;view.release_reason=record.reason;view.building_context=building;view.roster_complete=roster_complete;view.has_living_member=living;
            view.distance_known=distance_known;view.distance_m=distance_m;view.distance_cost=distance_m*distance_cost_per_m_;
            view.phase=record.reason==SideReleaseReason::none?(w.now_ms>=later(origin,policy_.stand_down_after_ms)?SideLifecyclePhase::stand_down:SideLifecyclePhase::engaged):
                (record.release_at==w.now_ms?SideLifecyclePhase::released:w.now_ms<record.cooldown_until?SideLifecyclePhase::cooldown:SideLifecyclePhase::awaiting_admission);
            for(auto member:side.members){poll(stop);auto a=actors.find(member);if(a==actors.end())continue;const auto& actor=*a->second;
                const LifecycleLease lease{member,actor.ownership_generation};auto old=state.released.find(member.id);
                const bool already=old!=state.released.end()&&old->second.member.actor==member&&old->second.member.ownership_generation==actor.ownership_generation;
                if(record.reason!=SideReleaseReason::none){
                    if(admitted(actor)){if(!already){Impl::Released r{lease,record.identity,record.reason,record.release_at,record.cooldown_until};old=state.released.insert_or_assign(member.id,r).first;}
                        const auto& r=old->second;out.releases.push_back({r.identity,r.member,r.reason,w.sequence,r.at,r.cooldown_until});}
                }else if(admitted(actor)&&!already)out.retained.push_back({record.identity,lease,w.sequence,w.now_ms,deadline,authored});
            }
            out.sides.push_back(view);
        }
        // Absent side is unknown, never 'all dead'. It publishes no leases.
        // Expired empty state may retire after its cooldown/quiet boundary.
        for(auto it=state.sides.begin();it!=state.sides.end();){poll(stop);if(!side_ids.count(it->first)){
            const auto& s=it->second;const auto keep=(std::max)(s.cooldown_until,later(s.evidence.valid?s.evidence.at_ms:s.activation,(std::max)(policy_.quiet_field_ms,policy_.quiet_building_ms)));
            if(w.now_ms>=keep){it=state.sides.erase(it);continue;}}++it;}
        for(const auto& row:state.sides){poll(stop);const auto& s=row.second;
            if(s.reason==SideReleaseReason::none||w.now_ms>=s.cooldown_until)continue;
            LifecycleCooldown c;c.identity=s.identity;c.native_group=s.native_group;c.until_ms=s.cooldown_until;c.reason=s.reason;
            for(const auto& m:s.roster){poll(stop);c.members.push_back(m.actor);}out.cooldowns.push_back(std::move(c));}
        poll(stop);impl_=std::move(staged);out.accepted=true;out.status=SideLifecycleStatus::accepted;return out;
    }catch(const Interrupted&){SideLifecycleResult cancelled;cancelled.status=SideLifecycleStatus::cancelled;cancelled.cancelled=true;cancelled.level_epoch=w.level_epoch;cancelled.map_hash=w.map_hash;cancelled.snapshot_seq=w.sequence;cancelled.captured_at_ms=w.now_ms;return cancelled;}
    catch(...){SideLifecycleResult failed;failed.status=SideLifecycleStatus::failed;failed.level_epoch=w.level_epoch;failed.map_hash=w.map_hash;failed.snapshot_seq=w.sequence;return failed;}
}
}}}
