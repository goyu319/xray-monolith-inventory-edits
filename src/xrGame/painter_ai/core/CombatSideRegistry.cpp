#include "CombatSideRegistry.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>
namespace painter { namespace combat { namespace core {
namespace {
struct Interrupted{};struct Exhausted{};
void poll(const std::function<bool()>& stop){if(stop&&stop())throw Interrupted{};}
struct KeyLess {bool operator()(ActorKey a,ActorKey b)const{return std::tie(a.id,a.generation)<std::tie(b.id,b.generation);}};
using NativeKey=std::pair<std::uint32_t,std::uint64_t>;
bool mission_valid(const Mission& m){if(m.rooms.count>m.rooms.items.size()||static_cast<unsigned>(m.kind)>static_cast<unsigned>(MissionKind::hold))return false;
    for(std::size_t i=0;i<m.rooms.count;++i){if(m.rooms.items[i]==invalid_room)return false;
        for(std::size_t j=0;j<i;++j)if(m.rooms.items[j]==m.rooms.items[i])return false;}return true;}
SideSnapshot copy_side(const SideSnapshot& s,const std::function<bool()>& stop){
    SideSnapshot d;d.side_id=s.side_id;d.faction=s.faction;d.group=s.group;d.mission=s.mission;d.building_id=s.building_id;d.mission_state=s.mission_state;d.link_up=s.link_up;d.cooperation_side=s.cooperation_side;
    d.members.reserve(s.members.size());for(const auto& m:s.members){poll(stop);d.members.push_back(m);}
    d.known_enemies.reserve(s.known_enemies.size());for(const auto& k:s.known_enemies){poll(stop);d.known_enemies.push_back(k);}return d;
}
}
struct CombatSideRegistry::Impl {
    enum class Kind : std::uint8_t { explicit_side,native,adhoc };
    struct Member {ActorKey key;std::uint64_t owner=0;};
    struct KnownMember {ActorKey key;TimeMs at=0;};
    struct Entry {Kind kind=Kind::adhoc;std::uint32_t id=0,faction=0,mission=0;std::uint64_t group=0,sequence=0;TimeMs used=0;std::vector<Member> members;std::vector<KnownMember> known;};
    std::map<std::uint32_t,Entry> entries;
    std::uint64_t epoch=0,map=0,sequence=0,next=0x80000000ull;TimeMs now=0;
    static Entry copy_entry(const Entry& a,const std::function<bool()>& stop){Entry b;b.kind=a.kind;b.id=a.id;b.faction=a.faction;b.mission=a.mission;b.group=a.group;b.sequence=a.sequence;b.used=a.used;
        b.members.reserve(a.members.size());for(const auto& m:a.members){poll(stop);b.members.push_back(m);}
        for(const auto& m:a.known){poll(stop);b.known.push_back(m);}return b;}
    std::unique_ptr<Impl> clone(const std::function<bool()>& stop)const{poll(stop);std::unique_ptr<Impl> b(new Impl);b->epoch=epoch;b->map=map;b->sequence=sequence;b->next=next;b->now=now;
        for(const auto& e:entries){poll(stop);b->entries.emplace(e.first,copy_entry(e.second,stop));}poll(stop);return b;}
};
CombatSideRegistry::CombatSideRegistry(Config c):config_(c),impl_(new Impl){}
CombatSideRegistry::~CombatSideRegistry()=default;
void CombatSideRegistry::reset(){impl_.reset(new Impl);}
std::unique_ptr<CombatSideRegistry> CombatSideRegistry::clone(const std::function<bool()>& stop)const{
    try{auto state=impl_->clone(stop);std::unique_ptr<CombatSideRegistry> result(new CombatSideRegistry(config_));result->impl_=std::move(state);poll(stop);return result;}
    catch(const Interrupted&){return nullptr;}
}
SideRegistryResult CombatSideRegistry::update(const WorldSnapshot& w,TimeMs retain_ms,const std::function<bool()>& stop,const FriendlyCasualtyEvidence& passive){
    SideRegistryResult out;out.level_epoch=w.level_epoch;out.map_hash=w.map_hash;out.snapshot_seq=w.sequence;out.captured_at_ms=w.now_ms;
    try{
        poll(stop);const auto& c=config_;
        if(!w.level_epoch||!w.sequence||!std::isfinite(c.nearby_group_distance)||c.nearby_group_distance<0
            ||!std::isfinite(c.regroup_separation_distance)||c.regroup_separation_distance<c.nearby_group_distance)return out;
        if(impl_->epoch&&(w.level_epoch<impl_->epoch||(w.level_epoch==impl_->epoch&&(w.sequence<=impl_->sequence||w.now_ms<impl_->now)))){
            out.status=SideRegistryStatus::stale;return out;}
        using Actors=std::map<ActorKey,const ActorSnapshot*,KeyLess>;Actors actors;std::set<std::uint32_t> actor_ids;
        for(const auto& a:w.actors){poll(stop);if(!valid_key(a.key)||!actor_ids.insert(a.key.id).second||a.level_epoch!=w.level_epoch
                ||a.snapshot_seq!=w.sequence||a.observed_at_ms>w.now_ms)return out;actors.emplace(a.key,&a);}
        std::set<std::uint32_t> explicit_ids;std::set<ActorKey,KeyLess> assigned;
        std::map<NativeKey,std::vector<std::uint32_t>> explicit_native;
        for(const auto& side:w.sides){poll(stop);if(!side.side_id||!explicit_ids.insert(side.side_id).second||!mission_valid(side.mission)||!finite(side.link_up))return out;
            std::uint64_t native=0;
            for(const auto& key:side.members){poll(stop);if(!valid_key(key)||!assigned.insert(key).second)return out;auto found=actors.find(key);
                if(found!=actors.end()){const auto& a=*found->second;if(a.faction!=side.faction)return out;
                    if(a.native_squad){if(native&&native!=a.native_squad)return out;native=a.native_squad;}}
            }
            for(const auto& k:side.known_enemies){poll(stop);if(!valid_key(k.key)||!finite(k.position)||k.latest_ms>w.now_ms||k.reporters.count>k.reporters.items.size())return out;
                for(std::size_t i=0;i<k.reporters.count;++i){bool member=false;
                    for(auto key:side.members){poll(stop);if(key==k.reporters.items[i])member=true;}
                    if(!valid_key(k.reporters.items[i])||!member)return out;}}
            // An authored native mission can temporarily contain only absent
            // members. Its group may still match captured omitted squad mates;
            // diagnose that producer conflict instead of generating a duplicate.
            if(!native&&side.group)for(const auto& a:actors){poll(stop);if(a.second->faction==side.faction&&a.second->native_squad==side.group){native=side.group;break;}}
            if(native){if(side.group!=native)return out;explicit_native[{side.faction,native}].push_back(side.side_id);}
        }
        auto state=impl_->clone(stop);auto& s=*state;
        auto receipt=[&](const Impl::Entry& e,SideResetReason why,std::uint32_t faction=0,std::uint64_t group=0){
            out.resets.push_back({e.id,e.faction,faction,e.group,group,why});};
        if(s.epoch&&(s.epoch!=w.level_epoch||s.map!=w.map_hash)){
            while(!s.entries.empty()){poll(stop);receipt(s.entries.begin()->second,SideResetReason::epoch_or_map);s.entries.erase(s.entries.begin());}s.next=0x80000000ull;}
        s.epoch=w.level_epoch;s.map=w.map_hash;s.sequence=w.sequence;s.now=w.now_ms;
        for(auto i=s.entries.begin();i!=s.entries.end();){poll(stop);if(w.now_ms>=i->second.used&&w.now_ms-i->second.used>retain_ms){receipt(i->second,SideResetReason::expired);i=s.entries.erase(i);}else ++i;}
        auto member_owner=[&](ActorKey key,const Impl::Entry* old){auto a=actors.find(key);if(a!=actors.end())return a->second->ownership_generation;
            if(old)for(const auto& m:old->members){poll(stop);if(m.key==key)return m.owner;}return std::uint64_t(0);};
        auto overlaps=[&](const Impl::Entry& a,const std::vector<ActorKey>& members){for(const auto& m:a.members)for(const auto& k:members){poll(stop);if(m.key==k)return true;}return false;};
        auto leases=[&](const Impl::Entry* old,const Impl::Entry& e){if(!old)return;
            for(const auto& m:e.members)for(const auto& prior:old->members){poll(stop);if(m.key==prior.key&&m.owner!=prior.owner){
                out.lease_changes.push_back({m.key,e.id,prior.owner,m.owner});break;}}};
        auto carry_known=[&](Impl::Entry& e,const Impl::Entry* old,bool continuity){
            if(old&&continuity&&old->mission==e.mission)for(const auto& m:old->known){poll(stop);if(fresh(m.at,w.now_ms,retain_ms))e.known.push_back(m);}
            for(const auto& member:e.members){poll(stop);if(!actors.count(member.key))continue;
                auto found=std::find_if(e.known.begin(),e.known.end(),[&](const Impl::KnownMember& x){return x.key==member.key;});
                if(found==e.known.end())e.known.push_back({member.key,w.now_ms});else found->at=w.now_ms;
            }
        };
        for(const auto& side:w.sides){poll(stop);auto previous=s.entries.find(side.side_id);const Impl::Entry* old=previous==s.entries.end()?nullptr:&previous->second;
            if(old){const bool same_identity=old->faction==side.faction&&old->group==side.group;
                const bool disjoint=!old->members.empty()&&!side.members.empty()&&!overlaps(*old,side.members);
                if(!same_identity||disjoint)receipt(*old,old->kind==Impl::Kind::explicit_side?SideResetReason::explicit_identity_reuse:SideResetReason::explicit_generated_collision,side.faction,side.group);}
            Impl::Entry e;e.id=side.side_id;e.kind=Impl::Kind::explicit_side;e.faction=side.faction;e.mission=side.mission.revision;e.group=side.group;e.sequence=w.sequence;e.used=w.now_ms;e.members.reserve(side.members.size());
            for(auto key:side.members){poll(stop);e.members.push_back({key,member_owner(key,old)});}leases(old,e);
            // An explicitly empty capture is not proof that its historical
            // identity may be recycled for unrelated men on the next capture.
            if(e.members.empty()&&old)for(const auto& m:old->members){poll(stop);e.members.push_back(m);}
            carry_known(e,old,old&&old->faction==e.faction&&old->group==e.group&&overlaps(*old,side.members));
            s.entries[side.side_id]=std::move(e);out.sides.push_back(copy_side(side,stop));
        }
        std::map<NativeKey,std::vector<const ActorSnapshot*>> native_groups;
        std::vector<const ActorSnapshot*> loose;
        std::map<NativeKey,SideRegistryConflict> conflicts;
        for(const auto& explicit_group:explicit_native){poll(stop);if(explicit_group.second.size()>1){SideRegistryConflict x;x.reason=SideConflictReason::split_explicit_native_squad;
            x.faction=explicit_group.first.first;x.native_squad=explicit_group.first.second;
            for(auto id:explicit_group.second){poll(stop);x.explicit_sides.push_back(id);}out.conflicts.push_back(std::move(x));}}
        for(const auto& pair:actors){poll(stop);const auto& a=*pair.second;
            // Identity registry is not an alternate admission/action controller.
            // Passive/dead keys can still be carried by explicit authored rosters.
            if(assigned.count(a.key)||!a.alive||!(a.flags&flag_admitted)||!a.ownership_generation||!finite(a.position)
                ||!fresh(a.observed_at_ms,w.now_ms,c.snapshot_max_age_ms))continue;
            const NativeKey key{a.faction,a.native_squad};auto explicit_group=explicit_native.find(key);
            if(a.native_squad&&explicit_group!=explicit_native.end()){
                auto& x=conflicts[key];x.reason=SideConflictReason::incomplete_native_roster;x.faction=a.faction;x.native_squad=a.native_squad;
                if(x.explicit_sides.empty())for(auto id:explicit_group->second){poll(stop);x.explicit_sides.push_back(id);}
                x.omitted.push_back(a.key);++out.unassigned_conflicts;continue;
            }
            if(a.native_squad)native_groups[key].push_back(&a);else loose.push_back(&a);
        }
        for(auto& conflict:conflicts){poll(stop);out.conflicts.push_back(std::move(conflict.second));}
        // Only same-ActorKey continuity supplies the larger hysteresis radius.
        // Ownership may change without throwing away that actor's personal K1 memory.
        std::map<ActorKey,std::pair<std::uint64_t,std::uint32_t>,KeyLess> latest_side;
        for(const auto& entry:s.entries){poll(stop);for(const auto& m:entry.second.members){poll(stop);auto found=latest_side.find(m.key);
            if(found==latest_side.end()||found->second.first<entry.second.sequence)latest_side[m.key]={entry.second.sequence,entry.first};}}
        std::map<ActorKey,std::uint32_t,KeyLess> prior_side;
        for(const auto& member:latest_side){poll(stop);const auto& entry=s.entries.at(member.second.second);auto a=actors.find(member.first);
            if(entry.kind==Impl::Kind::adhoc&&a!=actors.end()&&!a->second->native_squad&&a->second->faction==entry.faction)prior_side.emplace(member.first,entry.id);}
        // Form retained cohorts before newcomers, so a new low numeric ID
        // cannot peel a member away from a still-valid 40 m cohort.
        std::sort(loose.begin(),loose.end(),[&](const ActorSnapshot* a,const ActorSnapshot* b){poll(stop);
            auto aa=prior_side.find(a->key),bb=prior_side.find(b->key);const bool an=aa==prior_side.end(),bn=bb==prior_side.end();
            const std::uint32_t ag=an?0:aa->second,bg=bn?0:bb->second;
            return std::make_tuple(an,ag,a->key.id,a->key.generation)<std::make_tuple(bn,bg,b->key.id,b->key.generation);});
        struct Group {std::uint32_t faction=0;std::uint64_t native=0;std::vector<const ActorSnapshot*> members;};
        std::vector<Group> groups;
        for(auto& native:native_groups){poll(stop);Group g;g.faction=native.first.first;g.native=native.first.second;g.members=std::move(native.second);groups.push_back(std::move(g));}
        const auto native_count=groups.size();
        for(auto* a:loose){poll(stop);std::size_t selected=groups.size();
            for(std::size_t i=native_count;i<groups.size();++i){poll(stop);auto& g=groups[i];if(g.faction!=a->faction)continue;bool close=true;
                for(auto* b:g.members){poll(stop);++out.pair_checks;auto aa=prior_side.find(a->key),bb=prior_side.find(b->key);
                    const bool retained=aa!=prior_side.end()&&bb!=prior_side.end()&&aa->second==bb->second;
                    const double range=retained?c.regroup_separation_distance:c.nearby_group_distance;
                    const double dx=double(a->position.x)-b->position.x,dy=double(a->position.y)-b->position.y,dz=double(a->position.z)-b->position.z;
                    if(dx*dx+dy*dy+dz*dz>range*range){close=false;break;}}
                if(close){selected=i;break;}}
            if(selected==groups.size()){Group g;g.faction=a->faction;groups.push_back(std::move(g));}groups[selected].members.push_back(a);
        }
        std::set<std::uint32_t> used;for(auto id:explicit_ids){poll(stop);used.insert(id);}
        auto allocate=[&](){while(s.next<=(std::numeric_limits<std::uint32_t>::max)()&&s.entries.count(std::uint32_t(s.next))){poll(stop);++s.next;}
            if(s.next>(std::numeric_limits<std::uint32_t>::max)())throw Exhausted{};return std::uint32_t(s.next++);};
        for(const auto& g:groups){poll(stop);Impl::Entry* old=nullptr;std::size_t best_overlap=0;
            for(auto& entry:s.entries){poll(stop);auto& e=entry.second;if(used.count(e.id)||e.faction!=g.faction)continue;
                if(g.native&&e.group==g.native){
                    if(e.kind==Impl::Kind::native){old=&e;break;}
                    if(e.kind==Impl::Kind::explicit_side){bool continuity=false;for(auto* a:g.members)for(const auto& m:e.members){poll(stop);if(a->key==m.key)continuity=true;}
                        if(continuity){old=&e;break;}}
                }
                else if(e.kind==Impl::Kind::adhoc){std::size_t common=0;for(auto* a:g.members){poll(stop);auto prior=prior_side.find(a->key);if(prior!=prior_side.end()&&prior->second==e.id)++common;}
                    if(common>best_overlap){best_overlap=common;old=&e;}}
            }
            Impl::Entry e;e.id=old?old->id:allocate();e.kind=g.native?Impl::Kind::native:Impl::Kind::adhoc;e.faction=g.faction;e.group=g.native?g.native:e.id;e.sequence=w.sequence;e.used=w.now_ms;
            std::set<ActorKey,KeyLess> included;
            // Keep surviving old slots; append newcomers in deterministic key order.
            if(old)for(const auto& m:old->members){poll(stop);for(auto* a:g.members){poll(stop);if(a->key==m.key){e.members.push_back({a->key,a->ownership_generation});included.insert(a->key);break;}}}
            const bool continued=!e.members.empty();
            for(auto* a:g.members){poll(stop);if(included.insert(a->key).second)e.members.push_back({a->key,a->ownership_generation});}
            if(old&&g.native&&!continued&&!old->members.empty())receipt(*old,SideResetReason::disjoint_native_roster,e.faction,e.group);
            carry_known(e,old,continued);
            leases(old,e);used.insert(e.id);SideSnapshot side;side.side_id=e.id;side.faction=e.faction;side.group=e.group;
            for(const auto& m:e.members){poll(stop);side.members.push_back(m.key);}out.generated_members+=side.members.size();++out.generated_sides;
            s.entries[e.id]=std::move(e);out.sides.push_back(std::move(side));
        }
        // Observation does not admit the patient. Only an exact prior/current
        // roster witness can retain its key alongside the still-captured observer.
        // Explicit authored roster removals remain authoritative.
        auto facts=validate_friendly_casualties(w,passive,config_,stop);
        if(facts.cancelled)throw Interrupted{};
        if(facts.accepted)for(const auto& fact:facts.facts){poll(stop);
            auto current=std::find_if(out.sides.begin(),out.sides.end(),[&](const SideSnapshot& side){return side.side_id==fact.side_id;});
            if(current==out.sides.end()||current->faction!=fact.faction||current->group!=fact.group||current->mission.revision!=fact.mission_revision
                ||std::find(current->members.begin(),current->members.end(),fact.observer)==current->members.end()
                ||(fact.target_native_squad&&fact.target_native_squad!=current->group))continue;
            bool known=false,explicit_now=false;
            for(const auto& side:w.sides){poll(stop);if(side.side_id==fact.side_id){explicit_now=true;known=friendly_casualty_member(fact,side);break;}}
            if(!explicit_now&&impl_->epoch==w.level_epoch&&impl_->map==w.map_hash){
                const auto old=impl_->entries.find(fact.side_id);
                if(old!=impl_->entries.end()&&old->second.faction==fact.faction&&old->second.group==fact.group
                    &&fresh(old->second.used,w.now_ms,retain_ms)){
                    bool observer=false,target_known=false;TimeMs target_at=0;
                    for(const auto& m:old->second.members){poll(stop);observer|=m.key==fact.observer;}
                    for(const auto& m:old->second.known){poll(stop);if(m.key==fact.target&&fresh(m.at,w.now_ms,retain_ms)){target_at=m.at;target_known=true;}}
                    bool newer_elsewhere=false;
                    for(const auto& entry:impl_->entries){poll(stop);if(entry.first==fact.side_id)continue;
                        for(const auto& m:entry.second.known){poll(stop);if(m.key==fact.target&&m.at>=target_at)newer_elsewhere=true;}}
                    known=observer&&target_known&&!newer_elsewhere;
                }
            }
            if(!known)continue;
            if(std::find(current->members.begin(),current->members.end(),fact.target)==current->members.end()){
                // A healthy observed target is recovery evidence, not permission
                // to recruit it or reconstruct an absent actor's self-capture.
                if(fact.alive&&!fact.wounded)continue;
                bool elsewhere=false;for(const auto& side:out.sides){poll(stop);if(side.side_id!=current->side_id
                    &&std::find(side.members.begin(),side.members.end(),fact.target)!=side.members.end())elsewhere=true;}
                if(elsewhere)continue;
                current->members.push_back(fact.target);
                s.entries.at(current->side_id).members.push_back({fact.target,0});
            }
            auto& known_members=s.entries.at(current->side_id).known;
            auto seen=std::find_if(known_members.begin(),known_members.end(),[&](const Impl::KnownMember& m){return m.key==fact.target;});
            if(seen==known_members.end())known_members.push_back({fact.target,fact.observed_at_ms});else seen->at=fact.observed_at_ms;
            out.friendly_casualties.push_back(fact);
        }
        // Retained membership receipts are identity witnesses for the native
        // mission producer; no observer, health or target location is invented.
        for(const auto& side:out.sides){poll(stop);const auto at=s.entries.find(side.side_id);if(at==s.entries.end())continue;
            for(const auto& m:at->second.known){poll(stop);if(fresh(m.at,w.now_ms,retain_ms))
                out.friendly_memberships.push_back({side.side_id,side.faction,side.mission.revision,side.group,m.key,m.at});}}
        poll(stop);impl_=std::move(state);out.status=SideRegistryStatus::accepted;out.accepted=true;return out;
    }catch(const Interrupted&){out=SideRegistryResult{};out.status=SideRegistryStatus::cancelled;out.cancelled=true;return out;}
    catch(const Exhausted&){out=SideRegistryResult{};out.status=SideRegistryStatus::identity_exhausted;return out;}
}
}}}
