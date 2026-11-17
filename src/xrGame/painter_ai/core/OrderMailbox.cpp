#include "OrderMailbox.h"
#include "CombatGait.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace painter { namespace combat { namespace core {
namespace {
struct Work {std::size_t left;bool ok=true;bool use(){if(!left){ok=false;return false;}--left;return true;}};
bool key_valid(ActorKey k){return k.id<invalid_id&&k.generation!=0;}
bool finite(Vec3 p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
bool near(Vec3 a,Vec3 b){return std::fabs(a.x-b.x)<.01f&&std::fabs(a.y-b.y)<.01f&&std::fabs(a.z-b.z)<.01f;}
bool recent(TimeMs at,TimeMs now,TimeMs maximum){return at<=now&&now-at<=maximum;}
struct KeyHash {std::size_t operator()(ActorKey k)const{return std::hash<std::uint64_t>{}(k.generation^(std::uint64_t(k.id)<<32));}};
struct ActorIndex {
    struct Entry {ActorSnapshot const* actor=nullptr;std::size_t count=0;};
    std::unordered_map<std::uint32_t,Entry> ids;
    bool build(std::vector<ActorSnapshot> const& actors,Work& work){
        ids.reserve(actors.size());for(auto const& actor:actors){if(!work.use())return false;auto& e=ids[actor.key.id];if(!e.actor)e.actor=&actor;++e.count;}return true;
    }
    Entry get(ActorKey key,Work& work)const{if(!work.use())return {};auto i=ids.find(key.id);return i==ids.end()?Entry{}:i->second;}
    ActorSnapshot const* find(ActorKey key,Work& work)const{auto e=get(key,work);return e.actor&&e.actor->key==key?e.actor:nullptr;}
    MailboxReject reject(ActorKey key,Work& work)const{auto e=get(key,work);return !e.actor?MailboxReject::missing_actor:e.count!=1?MailboxReject::duplicate_actor:e.actor->key!=key?MailboxReject::identity:MailboxReject::none;}
};
using KeyCounts=std::unordered_map<ActorKey,std::size_t,KeyHash>;
// Charge index record payload + links/buckets conservatively; actual allocator
// bookkeeping is not promised by this logical byte budget. No actor-count cap.
bool index_budget(MailboxLimits const& l,std::initializer_list<std::size_t> sizes){
    constexpr std::size_t unit=128;std::size_t left=l.validation_bytes/unit;
    for(auto n:sizes){if(n>left)return false;left-=n;}return true;
}
MailboxReject actor_guard(ActorSnapshot const& s,ActorSnapshot const& c,Decision const& d,
    TimeMs source_time,std::uint64_t current_sequence,TimeMs now,MailboxLimits const& limits,bool release){
    if(!key_valid(s.key)||s.key!=c.key)return MailboxReject::identity;
    if(s.level_epoch!=d.level_epoch||c.level_epoch!=d.level_epoch||s.snapshot_seq!=d.snapshot_seq
        ||c.snapshot_seq!=current_sequence||s.observed_at_ms>source_time)return MailboxReject::invalid_snapshot;
    if(!s.ownership_generation||s.ownership_generation!=c.ownership_generation)return MailboxReject::ownership;
    if(s.restrictions_revision!=c.restrictions_revision)return MailboxReject::restrictions;
    if(!recent(s.observed_at_ms,now,limits.source_max_age_ms)||!recent(c.observed_at_ms,now,limits.source_max_age_ms))return MailboxReject::source_age;
    // Explicit hand-back validates the identity/lease/source, not life or navigation.
    // Dead actors must still be removable from the accepted-control ledger.
    if(!release&&(!finite(s.position)||!finite(c.position)||!std::isfinite(s.health)||!std::isfinite(c.health)))return MailboxReject::nonfinite;
    if(!release&&(!s.alive||!c.alive||s.health<=0||c.health<=0))return MailboxReject::dead;
    if(!release&&(s.yield_reasons!=yield_none||c.yield_reasons!=yield_none))return MailboxReject::ineligible;
    return MailboxReject::none;
}
bool contact_valid(Contact const& t,ActorSnapshot const& actor,TimeMs now,MailboxLimits const& limits,bool visible){
    if(!key_valid(t.key)||t.key==actor.key||!t.hostile||!t.alive_when_observed||!finite(t.position)
       ||t.observed_at_ms>actor.observed_at_ms||t.last_seen_ms>actor.observed_at_ms
       ||!recent(actor.observed_at_ms,now,limits.source_max_age_ms)||(t.wounded&&!t.finish_allowed))return false;
    // visible is seen THIS actor capture. The engine's memory/last-seen clocks
    // may stay unchanged while sight continues; neither is the capture clock.
    if(t.visible&&t.last_seen_ms!=0)return true;
    return !visible&&recent(t.observed_at_ms,actor.observed_at_ms,limits.memory_max_age_ms)
        &&recent(t.observed_at_ms,now,limits.memory_max_age_ms);
}
Contact const* known_contact(ActorSnapshot const& actor,ActorKey target,Vec3 point,bool by_point,bool visible,
    TimeMs now,MailboxLimits const& limits,Work& work){
    if(actor.visible_enemies.count>actor.visible_enemies.items.size())return nullptr;
    auto match=[&](Contact const& c){return work.use()&&(!key_valid(target)||c.key==target)
        &&(!by_point||near(c.position,point))&&contact_valid(c,actor,now,limits,visible);};
    if(match(actor.enemy))return &actor.enemy;
    for(std::size_t i=0;i<actor.visible_enemies.count;++i)if(match(actor.visible_enemies.items[i]))return &actor.visible_enemies.items[i];
    return nullptr;
}
bool knows(ActorSnapshot const& actor,ActorKey target,Vec3 point,bool by_point,bool visible,
    TimeMs now,MailboxLimits const& limits,Work& work){
    return known_contact(actor,target,point,by_point,visible,now,limits,work)!=nullptr;
}
void constrain_intent(Order& o,ActorSnapshot const& source,ActorSnapshot const& current,
    TimeMs now,MailboxLimits const& limits,Work& work,const PointFireAuthority* proof){
    // A visibility change does not invalidate a known movement goal. Fall back
    // to the observer's recorded point and close the old targeted fire intent.
    // No target's live transform/health is available to this value-only path.
    if(o.look.kind==LookKind::enemy&&!knows(current,o.look.target,{},false,true,now,limits,work)){
        auto contact=known_contact(source,o.look.target,{},false,true,now,limits,work);
        if(contact){o.look.kind=LookKind::point;o.look.value=contact->position;}
        o.fire.allowed=false;o.fire.until_ms=0;
    }
    if(o.fire.allowed&&o.fire.at_valid&&!knows(current,o.look.target,o.fire.at,true,false,now,limits,work)
       &&!(proof&&point_fire_authorized(*proof,o,current,now,limits.source_max_age_ms,limits.memory_max_age_ms))){
        o.fire.allowed=false;o.fire.until_ms=0;
    }
}
MailboxReject order_guard(Order const& o,ActorSnapshot const& s,ActorSnapshot const& c,Decision const& d,
    TimeMs source_time,WorldSnapshot const& current,TimeMs now,MailboxLimits const& limits,std::uint64_t applied,Work& work,const PointFireAuthority* proof){
    bool release=o.kind==OrderKind::yield;
    auto r=actor_guard(s,c,d,source_time,current.sequence,now,limits,release);if(r!=MailboxReject::none)return r;
    if(o.actor!=s.key||o.ownership_generation!=s.ownership_generation)return MailboxReject::ownership;
    if(o.restrictions_revision!=s.restrictions_revision)return MailboxReject::restrictions;
    if(o.level_epoch!=d.level_epoch||o.geometry_revision!=d.geometry_revision||o.snapshot_seq!=d.snapshot_seq)return MailboxReject::invalid_order;
    if(!o.order_seq||o.order_seq<=applied)return MailboxReject::older_order;
    if(o.issued_at_ms>now)return MailboxReject::future_time;
    if(o.issued_at_ms<source_time||o.expires_at_ms<=now||o.expires_at_ms<=o.issued_at_ms
       ||o.expires_at_ms-o.issued_at_ms>limits.max_order_lifetime_ms)return MailboxReject::source_age;
    if(!finite(o.goal_position)||!finite(o.look.value)||!finite(o.fire.at)||!finite(o.lean_edge)
       ||!finite(o.portal.centre)||!finite(o.portal.tangent)||!finite(o.portal.normal)||!finite(o.throw_order.target)
       ||!std::isfinite(o.arrival_radius_m)||!std::isfinite(o.lean)||!std::isfinite(o.speed_scale)
       ||!std::isfinite(o.fire.ammo_floor)||!std::isfinite(o.portal.half_width)||!std::isfinite(o.portal.thickness))return MailboxReject::nonfinite;
    if(o.arrival_radius_m<=0||o.lean< -1||o.lean>1||o.speed_scale<.3f||o.speed_scale>1.5f
       ||o.fire.ammo_floor<0||o.fire.ammo_floor>1)return MailboxReject::invalid_order;
    if(static_cast<unsigned>(o.kind)>static_cast<unsigned>(OrderKind::yield)
        ||static_cast<unsigned>(o.look.kind)>static_cast<unsigned>(LookKind::path)
        ||static_cast<unsigned>(o.pace)>static_cast<unsigned>(Pace::run)
        ||static_cast<unsigned>(o.stance)>static_cast<unsigned>(Stance::crouch))return MailboxReject::invalid_order;
    if(!release&&(o.goal_node==invalid_node||!c.current_position_accessible))return MailboxReject::navigation;
    if(o.look.kind==LookKind::enemy){
        if(!key_valid(o.look.target)||!knows(s,o.look.target,{},false,true,now,limits,work))return MailboxReject::perception;
    }
    if(o.fire.allowed){
        if(release||!o.fire.queue||o.fire.until_ms<=now||o.fire.until_ms>o.expires_at_ms
            ||s.reloading||c.reloading||!s.weapon.ammo_in_mag||!c.weapon.ammo_in_mag)return MailboxReject::invalid_order;
        if(o.fire.at_valid){
            if(!knows(s,o.look.target,o.fire.at,true,false,now,limits,work)
               &&!(proof&&point_fire_authorized(*proof,o,s,now,limits.source_max_age_ms,limits.memory_max_age_ms)
                    &&point_fire_authorized(*proof,o,c,now,limits.source_max_age_ms,limits.memory_max_age_ms)))return MailboxReject::perception;
        }else if(o.look.kind!=LookKind::enemy)return MailboxReject::perception;
        // Enemy knowledge comes only from this observer's contacts. Looking up
        // another controllable actor's private health here would give knowledge
        // the observer did not perceive. Identity/life/fire are checked against
        // the live selected target by the native executor before it can fire.
    }
    return MailboxReject::none;
}
bool same(MailboxToken a,MailboxToken b){return a.epoch==b.epoch&&a.generation==b.generation&&a.id==b.id&&a.id!=0;}
MailboxReject envelope(WorldSnapshot const& source,WorldSnapshot const& current,Decision const& d,
    TimeMs now,std::uint64_t epoch,std::uint64_t applied,MailboxLimits const& limits){
    if(!epoch||source.level_epoch!=epoch||current.level_epoch!=epoch||d.level_epoch!=epoch)return MailboxReject::epoch;
    if(source.geometry_revision!=current.geometry_revision||d.geometry_revision!=current.geometry_revision)return MailboxReject::geometry;
    if(d.snapshot_seq!=source.sequence||source.sequence>current.sequence||d.snapshot_seq<applied)return MailboxReject::backwards_snapshot;
    if(source.now_ms>now||current.now_ms>now||source.now_ms>current.now_ms)return MailboxReject::future_time;
    if(!recent(source.now_ms,now,limits.source_max_age_ms)||!recent(current.now_ms,now,limits.source_max_age_ms))return MailboxReject::source_age;
    return MailboxReject::none;
}
}

OrderMailbox::OrderMailbox(MailboxLimits limits):limits_(limits){}
void OrderMailbox::reset(std::uint64_t epoch){epoch_=epoch;++generation_;if(!generation_)++generation_;next_=1;bytes_=0;pending_.clear();applied_.clear();sides_.clear();}
std::size_t OrderMailbox::pending_bytes()const{return bytes_;}
std::size_t OrderMailbox::committed_records()const{return applied_.size();}
std::uint64_t OrderMailbox::applied_order(ActorKey actor,std::uint64_t ownership)const{
    for(auto const& a:applied_)if(a.actor==actor&&a.ownership==ownership)return a.order;return 0;
}
std::uint64_t OrderMailbox::applied_snapshot(std::uint32_t side)const{for(auto const& s:sides_)if(s.id==side)return s.snapshot;return 0;}
MailboxResult OrderMailbox::prepare(WorldSnapshot const& source,WorldSnapshot const& current,Decision const& d,TimeMs now,const std::vector<PointFireAuthority>& point_fire){
    Work work{limits_.validation_work};std::uint64_t side_sequence=0,event_sequence=0;
    for(auto const& side:sides_){if(!work.use())break;if(side.id==d.side_id){side_sequence=side.snapshot;event_sequence=side.event_snapshot;}}
    MailboxResult result;result.envelope=envelope(source,current,d,now,epoch_,side_sequence,limits_);
    if(!work.ok){result.envelope=MailboxReject::work_budget;return result;}
    if(result.envelope!=MailboxReject::none)return result;
    if(!index_budget(limits_,{source.actors.size(),current.actors.size(),d.orders.size(),d.releases.size(),applied_.size(),point_fire.size()})){
        result.envelope=MailboxReject::memory_budget;return result;
    }
    const bool event_only=d.orders.empty()&&d.releases.empty()&&!d.events.empty();
    if(event_only&&d.snapshot_seq<=event_sequence){result.envelope=MailboxReject::backwards_snapshot;return result;}
    std::size_t const count=d.orders.size()+d.releases.size();
    std::size_t const unit=sizeof(Order)+sizeof(ActorSnapshot)+sizeof(MailboxActorResult)+sizeof(PointFireAuthority);
    if(count>limits_.pending_bytes/unit||d.events.size()>limits_.pending_bytes/sizeof(Event)){
        result.envelope=MailboxReject::memory_budget;return result;
    }
    if(point_fire.size()>d.orders.size()){result.envelope=MailboxReject::invalid_order;return result;}
    std::unordered_map<ActorKey,const PointFireAuthority*,KeyHash> proof_index;
    for(const auto& proof:point_fire){if(!work.use())break;
        if(!proof_index.emplace(proof.authored.actor,&proof).second){result.envelope=MailboxReject::duplicate_actor;return result;}}
    ActorIndex sources,currents;sources.build(source.actors,work);currents.build(current.actors,work);
    KeyCounts counts;counts.reserve(count);
    for(auto const& o:d.orders){if(!work.use())break;++counts[o.actor];}
    for(auto key:d.releases){if(!work.use())break;++counts[key];}
    std::vector<Applied> live;live.reserve(applied_.size());
    std::unordered_map<std::uint32_t,std::size_t> applied_index;applied_index.reserve(applied_.size());
    for(auto const& a:applied_){if(!work.use())break;auto c=currents.get(a.actor,work);
        if(c.count>1||(c.actor&&(c.actor->level_epoch!=current.level_epoch||c.actor->snapshot_seq!=current.sequence))){live.push_back(a);}
        else if(c.count==1&&c.actor->key==a.actor&&c.actor->ownership_generation==a.ownership){applied_index[a.actor.id]=live.size();live.push_back(a);}}
    if(!work.ok){result.envelope=MailboxReject::work_budget;return result;}
    Pending p;p.decision=d;p.decision.orders.clear();p.decision.releases.clear();p.source_time=source.now_ms;
    p.decision.orders.reserve(d.orders.size());p.decision.releases.reserve(d.releases.size());p.sources.reserve(count);p.point_fire.reserve(point_fire.size());
    result.actors.reserve(count);
    auto process=[&](ActorKey key,bool release,Order const* order){
        if(!work.use())return;const auto pi=proof_index.find(key);const auto* proof=pi==proof_index.end()?nullptr:pi->second;
        MailboxReject reject=counts.at(key)==1?MailboxReject::none:MailboxReject::duplicate_actor;
        if(reject==MailboxReject::none)reject=sources.reject(key,work);
        if(reject==MailboxReject::none)reject=currents.reject(key,work);
        auto s=sources.find(key,work),c=currents.find(key,work);
        if(!work.ok)return;
        if(reject==MailboxReject::none){
            std::uint64_t sequence=0;
            if(order){if(!work.use())return;auto i=applied_index.find(key.id);if(i!=applied_index.end())sequence=live[i->second].order;}
            reject=order?order_guard(*order,*s,*c,d,source.now_ms,current,now,limits_,sequence,work,proof)
                :actor_guard(*s,*c,d,source.now_ms,current.sequence,now,limits_,true);
        }
        result.actors.push_back({key,release,reject});
        if(reject==MailboxReject::none){
            p.sources.push_back(*s);
            if(order){auto safe=*order;constrain_intent(safe,*s,*c,now,limits_,work,proof);
                // Aim can change at preflight. Every publication, including
                // worker/room plans, uses the gait of the actual final intent.
                safe.gait=order_gait(safe);p.decision.orders.push_back(safe);
                if(proof&&safe.fire.allowed&&point_fire_authorized(*proof,safe,*c,now,limits_.source_max_age_ms,limits_.memory_max_age_ms))p.point_fire.push_back(*proof);}
            else p.decision.releases.push_back(key);
        }
    };
    for(auto const& o:d.orders){if(!work.ok)break;process(o.actor,o.kind==OrderKind::yield,&o);}
    for(auto k:d.releases){if(!work.ok)break;process(k,true,nullptr);}
    if(!work.ok){result.envelope=MailboxReject::work_budget;return result;}
    // Prune only departed identities/leases in this fresh current envelope. No
    // successful sequence changes; old pending proofs still fail generation checks.
    applied_.swap(live);
    // An observational side event may be delivered without a movement order.
    // Failed actor proposals cannot use this path to launder their envelope.
    if(p.sources.empty()&&!event_only)return result;
    p.bytes=sizeof(Pending)+p.decision.orders.capacity()*sizeof(Order)+p.decision.releases.capacity()*sizeof(ActorKey)
        +p.decision.events.capacity()*sizeof(Event)+p.sources.capacity()*sizeof(ActorSnapshot)+p.point_fire.capacity()*sizeof(PointFireAuthority);
    if(p.bytes>limits_.pending_bytes-bytes_){result.envelope=MailboxReject::memory_budget;return result;}
    p.token={epoch_,generation_,next_++};result.token=p.token;bytes_+=p.bytes;pending_.push_back(std::move(p));return result;
}
bool OrderMailbox::prepared(MailboxToken token,Decision& out)const{for(auto const& p:pending_)if(same(token,p.token)){out=p.decision;return true;}return false;}
bool OrderMailbox::cancel(MailboxToken token){for(auto i=pending_.begin();i!=pending_.end();++i)if(same(token,i->token)){bytes_-=i->bytes;pending_.erase(i);return true;}return false;}
std::size_t OrderMailbox::expire(TimeMs now){std::size_t n=0;for(std::size_t i=0;i<pending_.size();){if(!recent(pending_[i].source_time,now,limits_.source_max_age_ms)){bytes_-=pending_[i].bytes;pending_.erase(pending_.begin()+i);++n;}else ++i;}return n;}
bool OrderMailbox::prepared_point_fire(MailboxToken token,std::vector<PointFireAuthority>& out)const{
    for(const auto& p:pending_)if(same(token,p.token)){out=p.point_fire;return true;}out.clear();return false;
}
MailboxResult OrderMailbox::commit(MailboxToken token,WorldSnapshot const& current,TimeMs now,std::vector<ActorKey> const& accepted){
    MailboxResult result;result.token=token;Work work{limits_.validation_work};
    auto it=std::find_if(pending_.begin(),pending_.end(),[&](Pending const& p){return work.use()&&same(token,p.token);});
    if(!work.ok){result.envelope=MailboxReject::work_budget;return result;}
    if(it==pending_.end()){result.envelope=MailboxReject::invalid_token;return result;}
    auto const& p=*it;WorldSnapshot source;source.level_epoch=p.decision.level_epoch;source.sequence=p.decision.snapshot_seq;
    source.geometry_revision=p.decision.geometry_revision;source.now_ms=p.source_time;
    std::uint64_t side_sequence=0,event_sequence=0;for(auto const& side:sides_){if(!work.use())break;
        if(side.id==p.decision.side_id){side_sequence=side.snapshot;event_sequence=side.event_snapshot;}}
    if(!work.ok){result.envelope=MailboxReject::work_budget;return result;}
    result.envelope=envelope(source,current,p.decision,now,epoch_,side_sequence,limits_);
    if(result.envelope!=MailboxReject::none){cancel(token);return result;}
    const bool event_only=p.decision.orders.empty()&&p.decision.releases.empty()&&!p.decision.events.empty();
    if(event_only&&p.decision.snapshot_seq<=event_sequence){
        result.envelope=MailboxReject::backwards_snapshot;cancel(token);return result;
    }
    if(!index_budget(limits_,{p.sources.size(),current.actors.size(),accepted.size(),applied_.size(),p.sources.size(),p.point_fire.size()})){
        result.envelope=MailboxReject::memory_budget;return result;
    }
    std::unordered_map<ActorKey,const PointFireAuthority*,KeyHash> proof_index;
    for(const auto& proof:p.point_fire){if(!work.use())break;proof_index.emplace(proof.authored.actor,&proof);}
    ActorIndex sources,currents;sources.build(p.sources,work);currents.build(current.actors,work);
    std::unordered_set<ActorKey,KeyHash> native;native.reserve(accepted.size());
    for(auto key:accepted){
        if(!work.use())break;
        if(!sources.find(key,work)||!native.insert(key).second){result.envelope=MailboxReject::native_result_invalid;return result;}
    }
    if(!work.ok){result.envelope=MailboxReject::work_budget;return result;}
    std::vector<Applied> next;next.reserve(applied_.size()+p.sources.size());
    std::unordered_map<std::uint32_t,std::size_t> applied_index;applied_index.reserve(applied_.size()+p.sources.size());
    for(auto const& a:applied_){if(!work.use())break;auto c=currents.get(a.actor,work);
        if(c.count>1||(c.actor&&(c.actor->level_epoch!=current.level_epoch||c.actor->snapshot_seq!=current.sequence))){next.push_back(a);}
        else if(c.count==1&&c.actor->key==a.actor&&c.actor->ownership_generation==a.ownership){applied_index[a.actor.id]=next.size();next.push_back(a);}}
    bool any=false;result.actors.reserve(p.sources.size());
    auto process=[&](ActorKey key,bool release,Order const* order){
        if(!work.use())return;const auto pi=proof_index.find(key);const auto* proof=pi==proof_index.end()?nullptr:pi->second;
        auto r=currents.reject(key,work);
        auto s=sources.find(key,work),c=currents.find(key,work);
        if(!work.ok||!work.use())return;auto prior=applied_index.find(key.id);std::uint64_t sequence=prior==applied_index.end()?0:next[prior->second].order;
        if(r==MailboxReject::none)r=order?order_guard(*order,*s,*c,p.decision,p.source_time,current,now,limits_,sequence,work,proof)
            :actor_guard(*s,*c,p.decision,p.source_time,current.sequence,now,limits_,true);
        if(!work.use())return;
        if(r==MailboxReject::none&&!native.count(key))r=MailboxReject::native_rejected;
        result.actors.push_back({key,release,r});
        if(r==MailboxReject::none){any=true;Applied a{key,s->ownership_generation,order?order->order_seq:sequence};
            if(prior==applied_index.end()){applied_index[key.id]=next.size();next.push_back(a);}else next[prior->second]=a;}
    };
    for(auto const& o:p.decision.orders){if(!work.ok)break;process(o.actor,o.kind==OrderKind::yield,&o);}
    for(auto key:p.decision.releases){if(!work.ok)break;process(key,true,nullptr);}
    if(!work.ok){result.envelope=MailboxReject::work_budget;return result;}
    auto side=sides_.end();for(auto i=sides_.begin();i!=sides_.end();++i){if(!work.use())break;if(i->id==p.decision.side_id)side=i;}
    const bool has_events=!p.decision.events.empty();
    bool new_side=(any||has_events)&&side==sides_.end();
    if(!work.ok){result.envelope=MailboxReject::work_budget;return result;}
    if(next.size()>limits_.committed_bytes/sizeof(Applied)
       ||(sides_.size()+std::size_t(new_side))>(limits_.committed_bytes-next.size()*sizeof(Applied))/sizeof(Side)){
        result.envelope=MailboxReject::memory_budget;return result;
    }
    // Allocate before mutation: failed checks or resources never advance a
    // generation. Publication receipt only updates exactly native-accepted keys.
    if(new_side)sides_.reserve(sides_.size()+1);
    applied_.swap(next);
    if(new_side)sides_.push_back({p.decision.side_id,any?p.decision.snapshot_seq:0,has_events?p.decision.snapshot_seq:0});
    else if(any||has_events){if(any)side->snapshot=p.decision.snapshot_seq;if(has_events)side->event_snapshot=(std::max)(side->event_snapshot,p.decision.snapshot_seq);}
    cancel(token);return result;
}
const char* mailbox_reason(MailboxReject r){
    switch(r){
#define R(name) case MailboxReject::name:return #name
    R(none);R(epoch);R(geometry);R(backwards_snapshot);R(future_time);R(source_age);R(invalid_snapshot);
    R(duplicate_actor);R(missing_actor);R(identity);R(ownership);R(restrictions);R(dead);R(ineligible);
    R(navigation);R(invalid_order);R(nonfinite);R(older_order);R(perception);R(target_dead);
    R(memory_budget);R(work_budget);R(invalid_token);R(native_result_invalid);R(native_rejected);
#undef R
    }return "unknown";
}
}}}
