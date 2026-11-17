#pragma once
// Local visual timeline keyed by exact native world and object-birth cookies.
// Projectile physics, health and PAM1 packet bytes are unchanged.
#include "LiveDamageWorker.h"
#include "GrenadeLifecycle.h"
#include "../painter_fragment_source.h"
#include "../level.h"
#include "../../xrEngine/xr_object.h"
#include <array>
#include <deque>
#include <map>
#include <cstring>
namespace painter_explosions {
using Born=painter_grenade::Born;using Key=painter_grenade::Key;using SplitRoute=painter_grenade::Route;
using Recipient=painter_grenade::Recipient;using Timeline=painter_grenade::Lifecycle<gfy_live::Impact>;
constexpr u32 fragment_window_ms=5000,stash_window_ms=150,launch_window_ms=4000;
inline Timeline& timeline(){static Timeline t;return t;}
inline std::map<Born,gfy_live::Identity>& target_bindings(){static std::map<Born,gfy_live::Identity>b;return b;}
inline Born born(const CObject& object){return {Level().PainterWorldCookie(),object.PainterBirthCookie(),object.ID()};}
inline Born current(u16 id){auto* o=Level().Objects.net_Find(id);return o&&!o->getDestroy()?born(*o):Born{};}
inline bool current(const Born& key){return key.valid()&&current(key.id)==key;}
inline bool register_target(const CObject&object,const gfy_live::Identity&identity){const auto key=born(object);if(!key.valid())return false;
    try{auto&b=target_bindings();auto old=b.find(key);if(old!=b.end()&&!(old->second==identity))timeline().retire(key);b[key]=identity;return true;}catch(...){return false;}}
struct Record {PainterFragmentSource source;u16 explosive=65535,initiator=65535;Fvector pos{};float radius=0;shared_str section;u32 time=0;};
inline std::deque<Record>& recent(){static std::deque<Record> r;return r;}
inline Born source_born(const PainterFragmentSource& s){return {s.world,s.cookie,s.explosive};}
inline bool same_source(const PainterFragmentSource&a,const PainterFragmentSource&b){return a.world==b.world&&a.cookie==b.cookie&&a.explosive==b.explosive&&
    a.initiator==b.initiator&&a.centre==b.centre&&a.radius==b.radius&&a.section==b.section&&a.emitted_at==b.emitted_at;}
inline Record* retain_source(const PainterFragmentSource& s)noexcept{try{
    if(!s.valid())return nullptr;auto& records=recent();for(auto&r:records)if(source_born(r.source)==source_born(s))return same_source(r.source,s)?&r:nullptr;
    while(!records.empty()&&painter_grenade::elapsed(Device.dwTimeGlobal,records.front().time,fragment_window_ms))records.pop_front();
    if(records.size()>=32)return nullptr;Record r;r.source=s;r.source.target_birth=0;r.explosive=s.explosive;r.initiator=s.initiator;
    r.pos.set(s.centre[0],s.centre[1],s.centre[2]);r.radius=s.radius;r.section=s.section.data();r.time=s.emitted_at;
    records.push_back(std::move(r));return &records.back();
}catch(...){return nullptr;}}
inline PainterFragmentSource record(CObject& object,const Fvector& pos,float radius,const shared_str& section,u16 initiator){
    PainterFragmentSource s;auto key=born(object);s.world=key.world;s.cookie=key.cookie;s.explosive=key.id;s.initiator=initiator;
    s.centre={pos.x,pos.y,pos.z};s.radius=radius;s.emitted_at=Device.dwTimeGlobal;
    auto n=std::strlen(section.c_str());if(!n||n>=s.section.size())return {};std::memcpy(s.section.data(),section.c_str(),n);
    return retain_source(s)?s:PainterFragmentSource{};
}
inline PainterFragmentSource source_for(const CObject& object)noexcept{try{auto key=born(object);for(const auto&r:recent())
    if(source_born(r.source)==key&&!painter_grenade::elapsed(Device.dwTimeGlobal,r.time,fragment_window_ms))return r.source;return {};}catch(...){return {};}}
// Exact locally emitted contact, restored once after its ordinary packet read.
struct DeliveryKey {
    Born target;u32 bullet=0;u16 shooter=65535,weapon=65535,bone=65535;std::array<u32,3> geometry{};
    bool operator<(const DeliveryKey&b)const{return std::tie(target,bullet,shooter,weapon,bone,geometry)<std::tie(b.target,b.bullet,b.shooter,b.weapon,b.bone,b.geometry);}
};
struct Delivery {PainterFragmentSource source;u32 at=0;};
inline std::map<DeliveryKey,Delivery>& deliveries(){static std::map<DeliveryKey,Delivery>d;return d;}
template<class Hit>inline DeliveryKey delivery_key(const Hit&h,const Born&target){DeliveryKey k;k.target=target;k.bullet=h.BulletID;
    k.shooter=h.whoID;k.weapon=h.weaponID;k.bone=h.boneID;const float v[]={h.p_in_bone_space.x,h.p_in_bone_space.y,h.p_in_bone_space.z};
    for(unsigned i=0;i<3;++i)std::memcpy(&k.geometry[i],&v[i],sizeof(float));return k;}
template<class Hit>inline bool deliver(const Hit&h,const CObject&target,const PainterFragmentSource&emitted){auto key=born(target);
    if(!key.valid()||!emitted.valid()||emitted.world!=key.world||emitted.explosive!=h.weaponID||emitted.initiator!=h.whoID||
       painter_grenade::elapsed(Device.dwTimeGlobal,emitted.emitted_at,fragment_window_ms))return false;
    auto&d=deliveries();for(auto it=d.begin();it!=d.end();)if(painter_grenade::elapsed(Device.dwTimeGlobal,it->second.at,fragment_window_ms))it=d.erase(it);else ++it;
    auto k=delivery_key(h,key);auto s=emitted;s.target_birth=key.cookie;auto old=d.find(k);if(old!=d.end())return same_source(old->second.source,s);
    if(d.size()>=4096)return false;try{d.emplace(k,Delivery{s,Device.dwTimeGlobal});return true;}catch(...){return false;}}
template<class Hit>inline void restore_delivery(Hit&h,const CObject& target){h.painter_fragment={};auto key=born(target);if(!key.valid())return;
    auto&d=deliveries();auto it=d.find(delivery_key(h,key));if(it==d.end())return;
    if(!painter_grenade::elapsed(Device.dwTimeGlobal,it->second.at,fragment_window_ms)&&it->second.source.target_birth==key.cookie)h.painter_fragment=it->second.source;d.erase(it);}
template<class Hit>inline Record* fragment(const Hit&h,const CObject&target){auto key=born(target);const auto&s=h.painter_fragment;
    if(!key.valid()||!s.valid()||s.world!=key.world||s.target_birth!=key.cookie||s.explosive!=h.weaponID||s.initiator!=h.whoID||
       painter_grenade::elapsed(Device.dwTimeGlobal,s.emitted_at,fragment_window_ms))return nullptr;return retain_source(s);}
inline Key key(const Record&r,const CObject&target){return {source_born(r.source),born(target)};}
inline bool can_fragment(const Record&r,const CObject&t){timeline().expire(Device.dwTimeGlobal);return timeline().can_fragment(key(r,t));}
inline bool next_blade(const Record&r,const CObject&t){return timeline().next_blade(key(r,t));}
// fragwound (2026-11-15): a blast whose fragment hits on this body reached Timeline::fragment_cap: ONE line per blast and body
// (the later hits are dropped silently by PainterWoundEvent.h)
inline void note_fragment_cap(const Record&r,const CObject&t)noexcept{try{static std::map<Key,u32> noted;const auto k=key(r,t);
    for(auto it=noted.begin();it!=noted.end();)if(painter_grenade::elapsed(Device.dwTimeGlobal,it->second,fragment_window_ms))it=noted.erase(it);else ++it;
    if(noted.count(k))return;noted.emplace(k,Device.dwTimeGlobal);
    Msg("[painter_explosions] id=%u phase=fragment_cap explosive=%u cap=%u taken=%u (this blast's further fragment hits on this body are dropped)",
        unsigned(t.ID()),unsigned(r.explosive),Timeline::fragment_cap,timeline().fragments(k));}catch(...){}}
inline bool capture_fragment(CObject&target,std::vector<gfy_live::Impact>events,bool blade,bool continued){if(events.empty())return false;
    const auto target_key=born(target);const auto&e=events.front();Record*r=nullptr;
    for(auto&candidate:recent())if(candidate.source.world==e.explosion_world&&candidate.source.cookie==e.explosion_cookie){if(r)return false;r=&candidate;}
    return r&&target_key.cookie==e.explosion_target_birth&&timeline().fragment(key(*r,target),std::move(events),blade,Device.dwTimeGlobal,continued);}
inline bool capture_blast(CObject&target,const PainterFragmentSource&source,std::vector<gfy_live::Impact>first,std::vector<gfy_live::Impact>later,bool hold_later=false){
    return source.valid()&&source.world==born(target).world&&retain_source(source)&&
        timeline().blast({source_born(source),born(target)},std::move(first),std::move(later),Device.dwTimeGlobal,hold_later);}
inline bool pending(const CObject&o){return timeline().pending(born(o));}
inline bool held_for(u16 id){auto k=current(id);return k.valid()&&timeline().pending(k);}
inline bool stashed_for(u16 id){return held_for(id);}
inline bool first_incomplete(const CObject&o,u64 accepted){return timeline().first_incomplete(born(o),accepted);}
inline bool first_incomplete(const CObject&o){return timeline().first_incomplete(born(o));}
template<class Queue>inline unsigned pump(CObject&o,bool split_pending,u64 checked,Queue queue){auto k=born(o);
    timeline().checked(k,checked);timeline().advance(k,Device.dwTimeGlobal,split_pending);return timeline().drain(k,queue);}
inline void accepted(const CObject&o,const gfy_live::Output&out){bool sever=false;if(out.resource_failure.empty())for(const auto&i:out.impacts)sever|=i.sever;
    timeline().accepted(born(o),out.stamp.revision,sever,Device.dwTimeGlobal);}
inline bool published(const CObject&o,std::shared_ptr<const SplitRoute>route){return timeline().published(born(o),std::move(route));}
struct RouteWitness {Born born;gfy_live::Identity identity;u64 generation=0,revision=0;bool retired=false;};
struct NativeRoute {std::shared_ptr<const SplitRoute> route;std::vector<RouteWitness> witnesses;bool committed=false;};
inline std::map<Born,std::shared_ptr<NativeRoute>>& pending_routes(){static std::map<Born,std::shared_ptr<NativeRoute>>r;return r;}
inline bool route_pending(const CObject&o){auto key=born(o);for(const auto&r:pending_routes())if(r.second->committed)
    for(const auto&w:r.second->witnesses)if(w.born==key&&!w.retired)return true;return false;}
struct RouteTicket {
    std::shared_ptr<NativeRoute> value;
    ~RouteTicket(){if(value&&!value->committed){auto it=pending_routes().find(value->route->source);if(it!=pending_routes().end()&&it->second==value)pending_routes().erase(it);}}
    bool reserve(std::shared_ptr<NativeRoute> r){if(!r)return true;try{if(pending_routes().count(r->route->source)||!timeline().can_publish(r->route->source,r->route))return false;
        pending_routes().emplace(r->route->source,r);value=std::move(r);return true;}catch(...){return false;}}
    void commit(){if(value){value->committed=true;timeline().publication_pending(value->route->source);}}
};
template<class Validate,class Schedule>inline void retry_routes(Validate validate,Schedule schedule){
    auto&routes=pending_routes();for(auto it=routes.begin();it!=routes.end();){const auto&r=*it->second;
        if(!r.committed){++it;continue;}bool valid=r.route&&r.witnesses.size()==r.route->recipients.size();
        for(const auto&w:r.witnesses)if(!w.retired&&!validate(w)){valid=false;break;}
        if(!valid||!timeline().published(it->first,r.route)){++it;continue;}
        for(const auto&w:r.witnesses)if(w.retired)timeline().retire(w.born);else schedule(w.born);it=routes.erase(it);}
}
// outcome (R62 legacy dismemberment rule): -1 the blast's own hit not processed yet, 0 the body lived through it, 1 it killed
struct Blast {Fvector pos{};float strength=0;u32 time=0;int outcome=-1;u16 explosive=65535;};
inline std::map<Born,Blast>& blasts(){static std::map<Born,Blast>b;return b;}
inline bool note_blast(u16 id,const Fvector&pos,float strength,u16 explosive=65535)noexcept{try{auto key=current(id);if(!key.valid())return false;auto&map=blasts();
    for(auto it=map.begin();it!=map.end();)if(painter_grenade::elapsed(Device.dwTimeGlobal,it->second.time,launch_window_ms))it=map.erase(it);else ++it;
    const auto old=map.find(key);Blast next=old==map.end()?Blast{}:old->second;
    // a weaker later blast keeps the stronger record whole (its geometry and its own outcome/explosive go together)
    if(painter_grenade::elapsed(Device.dwTimeGlobal,next.time,launch_window_ms)||strength>=next.strength){next.pos=pos;next.strength=strength;next.outcome=-1;next.explosive=explosive;}
    next.time=Device.dwTimeGlobal;
    if(old==map.end())map.emplace(key,next);else old->second=next;return true;}catch(...){return false;}}
// The hit of this blast's own explosive reached the body (after its damage): dead or alive after it (R62 legacy dismemberment rule;
// another explosive's hit or a bullet never decides it).
inline void note_blast_hit(u16 id,u16 explosive,bool dead)noexcept{try{auto it=blasts().find(current(id));
    if(it!=blasts().end()&&it->second.explosive!=65535&&it->second.explosive==explosive&&
       !painter_grenade::elapsed(Device.dwTimeGlobal,it->second.time,launch_window_ms)&&it->second.outcome<0)it->second.outcome=dead?1:0;}catch(...){}}
inline const Blast* blast_on(u16 id){auto it=blasts().find(current(id));return it==blasts().end()||painter_grenade::elapsed(Device.dwTimeGlobal,it->second.time,launch_window_ms)?nullptr:&it->second;}
// MSVC's empty map/deque construction may allocate. Each independent cleanup
// must continue even when an unrelated, never-used lazy container cannot start.
inline void cancel_orphan_stash(const CObject&object){try{timeline().cancel_orphan_stash(born(object));}catch(...){}}
inline void retire(const CObject&o){auto k=born(o);
    try{timeline().retire(k);}catch(...){}
    try{target_bindings().erase(k);}catch(...){}
    try{blasts().erase(k);}catch(...){}
    try{for(auto&r:pending_routes())for(auto&w:r.second->witnesses)if(w.born==k)w.retired=true;}catch(...){}
    try{for(auto it=deliveries().begin();it!=deliveries().end();)if(it->first.target==k)it=deliveries().erase(it);else ++it;}catch(...){}
}
inline void world_stopped(u64 world){
    try{timeline().world_stopped(world);}catch(...){}
    try{for(auto it=recent().begin();it!=recent().end();)if(it->source.world==world)it=recent().erase(it);else ++it;}catch(...){}
    try{for(auto it=pending_routes().begin();it!=pending_routes().end();)if(it->first.world==world)it=pending_routes().erase(it);else ++it;}catch(...){}
    try{for(auto it=target_bindings().begin();it!=target_bindings().end();)if(it->first.world==world)it=target_bindings().erase(it);else ++it;}catch(...){}
    try{for(auto it=deliveries().begin();it!=deliveries().end();)if(it->first.target.world==world)it=deliveries().erase(it);else ++it;}catch(...){}
    try{for(auto it=blasts().begin();it!=blasts().end();)if(it->first.world==world)it=blasts().erase(it);else ++it;}catch(...){}
}
}
