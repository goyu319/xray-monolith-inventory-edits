#pragma once
// Typed, retained native timeline. No object pointers, executable offsets or
// engine calls: adapters supply exact native-born keys and actual queue/commit
// observations. A failed enqueue never consumes the retained event.
#include <algorithm>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <tuple>
#include <vector>

namespace painter_grenade {
struct Born {
    std::uint64_t world=0,cookie=0;std::uint16_t id=65535;
    bool valid()const noexcept{return world&&cookie&&id!=65535;}
    bool operator==(const Born&b)const noexcept{return world==b.world&&cookie==b.cookie&&id==b.id;}
    bool operator!=(const Born&b)const noexcept{return !(*this==b);}
    bool operator<(const Born&b)const noexcept{return std::tie(world,cookie,id)<std::tie(b.world,b.cookie,b.id);}
};
struct Key {Born grenade,target;bool operator<(const Key&b)const noexcept{return grenade!=b.grenade?grenade<b.grenade:target<b.target;}};
struct Recipient {Born born;std::vector<std::uint16_t> fitted,anchors;};
struct Route {Born source;std::vector<Recipient> recipients;bool single_stump_transaction=false;};
inline bool owns(const Recipient&r,std::uint16_t bone){return std::find(r.fitted.begin(),r.fitted.end(),bone)!=r.fitted.end()||
    (bone<r.anchors.size()&&r.anchors[bone]!=65535);}
inline bool elapsed(std::uint32_t now,std::uint32_t then,std::uint32_t span){return std::uint32_t(now-then)>span;}
enum class TearGate {go,wait,survived};
inline TearGate tearGate(bool alive,bool blastKnown,std::uint32_t now,std::uint32_t blastTime,std::uint32_t window=500){
    if(!alive)return TearGate::go;
    return blastKnown&&!elapsed(now,blastTime,window)?TearGate::wait:TearGate::survived;}
template<class Event> class Lifecycle {
public:
    struct Delivery {Born target;std::deque<Event> events;bool blocked=false;};
    struct State {
        std::deque<Event> first,later,blades,singles;std::vector<Delivery> routed;
        unsigned fragments=0,blade_hits=0;std::uint64_t after=0,accepted=0,checked=0;
        std::uint32_t stash_time=0,hold_time=0,last_capture=0;bool stash=false,wave=false,holding=false,severed=false,publication_pending=false;
        std::shared_ptr<const Route> route;
    };
private:
    std::map<Key,State> states_;std::uint64_t next_token_=1;
    static constexpr std::size_t max_states=256,max_events=4096;
    // One body's blast batch: up to 48 craters plus the explosion blades' rows (R13/R14: 144); the worker queue holds 256.
    static constexpr std::size_t max_blast_events=192;
    std::size_t count(const State&s)const {std::size_t n=s.first.size()+s.later.size()+s.blades.size()+s.singles.size();for(const auto&d:s.routed)n+=d.events.size();return n;}
    bool append(std::deque<Event>& to,std::vector<Event>& events,const Key&key){
        for(auto&e:events){if(!e.explosion_member_token){if(!next_token_)return false;e.explosion_member_token=next_token_++;}
            e.explosion_world=key.grenade.world;e.explosion_cookie=key.grenade.cookie;e.explosion_target_birth=key.target.cookie;to.push_back(std::move(e));}return true;}
public:
    std::size_t retained()const {std::size_t n=0;for(const auto&p:states_)n+=count(p.second);return n;}
    void expire(std::uint32_t now){for(auto it=states_.begin();it!=states_.end();)if(!count(it->second)&&!it->second.holding&&
        !it->second.publication_pending&&it->second.accepted>=it->second.after&&elapsed(now,it->second.last_capture,5000))it=states_.erase(it);else ++it;}
    // fragwound (2026-11-15): the fragment hits one blast may wound one body with (a close grenade sends 100-230); later ones are
    // dropped silently (PainterWoundEvent.h), noted once per blast and body (painter_explosions::note_fragment_cap)
    static constexpr unsigned fragment_cap=30;
    unsigned fragments(const Key&key)const {auto it=states_.find(key);return it==states_.end()?0u:it->second.fragments;}
    bool can_fragment(const Key&key)const {auto it=states_.find(key);return key.grenade.valid()&&key.target.valid()&&key.grenade.world==key.target.world&&
        (it==states_.end()?states_.size()<max_states:it->second.fragments<fragment_cap);}
    bool next_blade(const Key&key)const {auto it=states_.find(key);return it==states_.end()||it->second.blade_hits<6;}
    bool fragment(const Key&key,std::vector<Event> events,bool blade,std::uint32_t now,bool continued){
        expire(now);
        if(!can_fragment(key)||events.empty()||events.size()>14||events.size()>max_events-retained())return false;
        try {auto it=states_.find(key);State next=it==states_.end()?State{}:it->second;
            // fragwound: a blade slot only within its quota of 6; a single hole always (since R62 every fragment is one: the old
            // `blade!=(blade_hits<6)` asked every fragment to be a blade first and refused them all)
            if(blade&&next.blade_hits>=6)return false;
            if(next.route){
                auto delivery=std::find_if(next.routed.begin(),next.routed.end(),[&](const Delivery&d){return d.target==key.target;});
                if(delivery==next.routed.end()){next.routed.push_back(Delivery{key.target,{},next.publication_pending});delivery=next.routed.end()-1;}
                if(!append(delivery->events,events,key))return false;
            }else{
                auto&to=next.publication_pending?next.later:(continued||(next.wave&&blade)?next.first:(next.wave?next.later:(blade?next.blades:next.singles)));
                if(!append(to,events,key))return false;
                if(!continued&&!next.wave&&!next.stash){next.stash=true;next.stash_time=now;}
            }
            ++next.fragments;if(blade)++next.blade_hits;
            next.last_capture=now;
            if(it==states_.end())states_.emplace(key,std::move(next));else it->second=std::move(next);return true;
        }catch(...){return false;}
    }
    // R33: hold_later keeps the later batch (torso chunks, the wave's shrapnel arriving over the next frames) behind the
    // first batch's sever check even when it is empty at blast time; advance() lifts the hold as before.
    bool blast(const Key&key,std::vector<Event> first,std::vector<Event> later,std::uint32_t now,bool hold_later=false){
        expire(now);
        if(!key.grenade.valid()||!key.target.valid()||key.grenade.world!=key.target.world||first.empty()||
           first.size()+later.size()>max_blast_events||first.size()+later.size()>max_events-retained())return false;
        try {auto it=states_.find(key);if(it==states_.end()&&states_.size()>=max_states)return false;
            State next=it==states_.end()?State{}:it->second;if(next.wave)return false;
            while(!next.blades.empty()){next.first.push_back(std::move(next.blades.front()));next.blades.pop_front();}
            std::deque<Event> early;early.swap(next.singles); // fragwound: the holes that came before the blast go after its craters
            if(!append(next.first,first,key)||!append(next.later,later,key))return false;
            while(!early.empty()){next.later.push_back(std::move(early.front()));early.pop_front();}
            next.wave=true;next.stash=false;next.holding=hold_later||!next.later.empty();next.hold_time=now;next.last_capture=now;
            if(it==states_.end())states_.emplace(key,std::move(next));else it->second=std::move(next);return true;
        }catch(...){return false;}
    }
    bool pending(const Born&target)const {for(const auto&p:states_){const auto&s=p.second;
        if(p.first.target==target&&(count(s)||s.publication_pending))return true;for(const auto&d:s.routed)if(d.target==target&&(!d.events.empty()||d.blocked))return true;}return false;}
    bool first_incomplete(const Born&target,std::uint64_t accepted)const {for(const auto&p:states_)if(p.first.target==target&&
        (p.second.stash||!p.second.first.empty()||p.second.after>accepted))return true;
        for(const auto&p:states_)for(const auto&d:p.second.routed)if(d.target==target&&(!d.events.empty()||d.blocked))return true;return false;}
    bool first_incomplete(const Born&target)const {for(const auto&p:states_)if(p.first.target==target&&
        (p.second.publication_pending||p.second.stash||!p.second.first.empty()||p.second.after>p.second.accepted))return true;
        for(const auto&p:states_)for(const auto&d:p.second.routed)if(d.target==target&&(!d.events.empty()||d.blocked))return true;return false;}
    void accepted(const Born&target,std::uint64_t revision,bool sever,std::uint32_t now){
        for(auto&p:states_)if(p.first.target==target){auto&s=p.second;s.accepted=std::max(s.accepted,revision);
            if(s.holding&&sever&&!s.severed){s.severed=true;s.hold_time=now;}}
    }
    void checked(const Born&target,std::uint64_t revision){for(auto&p:states_)if(p.first.target==target)p.second.checked=std::max(p.second.checked,revision);}
    void advance(const Born&target,std::uint32_t now,bool split_pending){
        for(auto&p:states_)if(p.first.target==target){auto&s=p.second;
            if(s.route||s.publication_pending)continue;
            try{State next=s;
            if(next.stash&&elapsed(now,next.stash_time,150)){
                next.first.insert(next.first.end(),next.blades.begin(),next.blades.end());next.first.insert(next.first.end(),next.singles.begin(),next.singles.end());
                next.blades.clear();next.singles.clear();next.stash=false;
            }
            if(next.holding&&next.first.empty()&&next.after&&next.accepted>=next.after&&
               (!next.severed||(!split_pending&&(next.checked>=next.after||elapsed(now,next.hold_time,4000)))))next.holding=false;
            s=std::move(next);}catch(...){/* Original retained queues are unchanged. */}
        }
        expire(now);
    }
    template<class Queue> unsigned drain(const Born&target,Queue queue){
        unsigned sent=0;try{for(auto&p:states_){auto&s=p.second;
            auto take=[&](std::deque<Event>&list,bool barrier){while(!list.empty()){
                Event copy=list.front();std::uint64_t sequence=0;if(!queue(std::move(copy),sequence)||!sequence)return false;
                if(barrier)s.after=std::max(s.after,sequence);list.pop_front();++sent;}return true;};
            if(p.first.target==target&&!s.route&&!s.publication_pending){if(!take(s.first,true))return sent;if(!s.holding&&!take(s.later,false))return sent;}
            for(auto&d:s.routed)if(d.target==target&&!d.blocked&&!take(d.events,false))return sent;
        }}catch(...){/* The current retained front still belongs to this ledger. */}return sent;
    }
private:
    bool route_copy(const Born&source,const std::shared_ptr<const Route>& route,std::map<Key,State>& next)const {
        if(!route||route->source!=source||route->recipients.empty()||route->recipients.size()>14||
            (route->recipients.size()==1&&!route->single_stump_transaction))return false;
        for(std::size_t i=0;i<route->recipients.size();++i){const auto&r=route->recipients[i];
            if(!r.born.valid()||r.born.world!=source.world||r.fitted.empty()||r.fitted.size()>64||r.anchors.empty()||r.anchors.size()>64)return false;
            if((i==0)!=(r.born==source))return false;
            for(std::size_t q=0;q<i;++q)if(route->recipients[q].born==r.born)return false;
            std::uint64_t seen=0;for(auto bone:r.fitted){if(bone>=r.anchors.size()||(seen&(std::uint64_t(1)<<bone)))return false;seen|=std::uint64_t(1)<<bone;}
            for(auto anchor:r.anchors)if(anchor!=65535&&std::find(r.fitted.begin(),r.fitted.end(),anchor)==r.fitted.end())return false;
        }
        next=states_;
        for(auto&p:next){auto&s=p.second;std::deque<Event> addressed;
            if(p.first.target==source){
                if(!s.first.empty()||s.stash||!s.blades.empty()||!s.singles.empty()||s.after>s.accepted)return false;
                addressed.swap(s.later);s.route=route;s.holding=false;s.publication_pending=false;
            }
            // A later split can concern a child of any earlier grenade route.
            // Preserve all deliveries to other surviving recipients.
            for(auto it=s.routed.begin();it!=s.routed.end();)if(it->target==source){
                addressed.insert(addressed.end(),it->events.begin(),it->events.end());it=s.routed.erase(it);
            }else ++it;
            if(addressed.empty())continue;
            for(const auto&e:addressed){bool found=false;for(const auto&r:route->recipients)found|=owns(r,e.hit_bone);if(!found)return false;}
            for(const auto&r:route->recipients){auto delivery=std::find_if(s.routed.begin(),s.routed.end(),[&](const Delivery&d){return d.target==r.born;});
                for(const auto&e:addressed)if(owns(r,e.hit_bone)){
                    if(delivery==s.routed.end()){s.routed.push_back(Delivery{r.born,{}});delivery=s.routed.end()-1;}
                    auto copy=e;copy.explosion_target_birth=r.born.cookie;delivery->events.push_back(std::move(copy));}
            }
        }
        std::size_t total=0;for(const auto&p:next)total+=count(p.second);return total<=max_events;
    }
public:
    bool can_publish(const Born&source,const std::shared_ptr<const Route>&route)const {
        try{std::map<Key,State> next;return route_copy(source,route,next);}catch(...){return false;}
    }
    bool published(const Born&source,std::shared_ptr<const Route> route){
        try{std::map<Key,State> next;if(!route_copy(source,route,next))return false;states_.swap(next);return true;}catch(...){return false;}
    }
    void publication_pending(const Born&source){for(auto&p:states_){if(p.first.target==source)p.second.publication_pending=true;
        for(auto&d:p.second.routed)if(d.target==source)d.blocked=true;}}
    void cancel_orphan_stash(const Born&target){for(auto it=states_.begin();it!=states_.end();){const auto&state=it->second;
        if(it->first.target==target&&state.stash&&!state.wave&&!state.route&&!state.publication_pending)it=states_.erase(it);else++it;}}
    void retire(const Born&target){for(auto it=states_.begin();it!=states_.end();){
        if(it->first.target==target&&!it->second.route&&!it->second.publication_pending){it=states_.erase(it);continue;}
        for(auto&d:it->second.routed)if(d.target==target)d.events.clear();++it;}}
    void world_stopped(std::uint64_t world){for(auto it=states_.begin();it!=states_.end();)if(it->first.target.world==world)it=states_.erase(it);else ++it;}
    void clear(){states_.clear();}
};
}
