#include "SquadTactics.h"
#include <algorithm>
#include <cmath>
#include <deque>
#include <iterator>
#include <map>
#include <set>
#include <tuple>

namespace painter { namespace combat { namespace core {
namespace {
struct Cancelled {};
thread_local const std::function<bool()>* cancellation_callback=nullptr;
thread_local unsigned cancellation_work=0;
struct CancellationScope {
    const std::function<bool()>* previous; unsigned work;
    explicit CancellationScope(const std::function<bool()>& callback):previous(cancellation_callback),work(cancellation_work) {
        cancellation_callback=&callback;cancellation_work=31;
    }
    ~CancellationScope(){cancellation_callback=previous;cancellation_work=work;}
};
void poll() {
    if(++cancellation_work<32)return;
    cancellation_work=0;
    if(cancellation_callback && *cancellation_callback && (*cancellation_callback)())throw Cancelled{};
}
struct KeyLess { bool operator()(const ActorKey& a,const ActorKey& b) const {
    poll();return std::tie(a.id,a.generation)<std::tie(b.id,b.generation);
} };
using Keys=std::set<ActorKey,KeyLess>;
struct Pair {
    ActorKey a,b;
    bool operator<(const Pair& o) const {
        poll();return std::tie(a.id,a.generation,b.id,b.generation)<std::tie(o.a.id,o.a.generation,o.b.id,o.b.generation);
    }
};
bool recent(TimeMs at,TimeMs now,TimeMs age) { return at<=now && now-at<=age; }
bool same_position(const Vec3& a,const Vec3& b) { return a.x==b.x && a.y==b.y && a.z==b.z; }
template<class F> void contacts(const ActorSnapshot& a,F f) {
    poll();f(a.enemy);
    for (std::size_t i=0;i<a.visible_enemies.count;++i) f(a.visible_enemies.items[i]);
}
bool personal(const Contact& t,const ActorSnapshot& a) { return personally_visible(t,a); }
bool known(const Contact& t,const ActorSnapshot& a,TimeMs now,TimeMs age) {
    return valid_key(t.key) && t.key!=a.key && t.hostile && t.alive_when_observed &&
        (!t.wounded || t.finish_allowed) && finite(t.position) && t.observed_at_ms<=a.observed_at_ms && recent(t.observed_at_ms,now,age);
}
bool contains(const std::vector<ActorKey>& keys,const ActorKey& key) {
    return std::binary_search(keys.begin(),keys.end(),key,KeyLess{});
}
struct Group {
    std::uint32_t faction=0,side_id=0;
    std::uint64_t native=0,stamp=0;
    bool adhoc=false,merged=false;
    std::vector<const ActorSnapshot*> actors;
    std::vector<ActorKey> roster; // exact authored slot order, including absent/ineligible members
    Keys enemies;
};
struct Prior { std::uint64_t group=0,native=0,owner=0; std::uint16_t role=roles::individual; bool adhoc=false; };
struct Stamp {
    std::uint32_t faction=0; TimeMs used_at=0; std::vector<ActorKey> members;
    std::map<ActorKey,std::uint64_t,KeyLess> owners;
};
struct ObservationEvidence {
    KnowledgeTrace trace;
    std::uint64_t stamp=0;
    TimeMs captured_at=0;
};
struct Denied { std::uint64_t owner=0,restrictions=0,geometry=0; };
struct Reservation { ActorKey actor; std::uint32_t faction=0,node=invalid_node; Vec3 position; };

const MemberKnowledge* member_knowledge(const ActorSnapshot& a,const WorldSnapshot& w,const KnowledgeFrame& frame) {
    if(!frame.accepted||frame.cancelled||frame.level_epoch!=w.level_epoch||frame.sequence!=w.sequence)return nullptr;
    for(const auto& m:frame.members)if(m.actor==a.key&&m.ownership_generation==a.ownership_generation&&m.alive&&m.able)return &m;
    return nullptr;
}
bool knowledge_trace(const ActorSnapshot& a,const WorldSnapshot& w,const FoeKnowledge& foe,KnowledgeTrace& trace) {
    const auto* fact=foe.last_known();
    if(!foe.id.valid()||foe.down||foe.wounded||!fact||!fact->valid||!finite(fact->position)||fact->at_ms>w.now_ms)return false;
    trace={};trace.receiver=a.key;trace.target=foe.id.ghost()?ActorKey{}:foe.id.actor;
    trace.observer=fact->reporter;trace.position=fact->position;trace.observed_at_ms=fact->at_ms;
    trace.receiver_ownership_generation=a.ownership_generation;trace.receiver_faction=a.faction;
    trace.shared=fact->reporter!=a.key;trace.foe=foe.id;
    // No frame field alone authorizes dereferencing an enemy or firing. Sight
    // must still be personally true in this exact actor capture.
    if(!foe.id.ghost()&&foe.visible)contacts(a,[&](const Contact& c) {
        if(c.key==foe.id.actor&&personal(c,a)&&c.hostile&&c.alive_when_observed&&!c.wounded) {
            trace.personally_visible=true;trace.shared=false;trace.observer=a.key;
            trace.position=c.position;trace.observed_at_ms=c.observed_at_ms;trace.source_snapshot_seq=a.snapshot_seq;
            // Native visible_now supplies the current observed position while
            // its memory record can retain an older m_level_time. K1 already
            // records this actual sight at its captured source time. Preserve
            // that source only when observer and exact position match; never
            // refresh a report or a remembered (not visible) contact here.
            const auto& sight=foe.sources[0];
            if(sight.valid&&sight.reporter==a.key&&finite(sight.position)&&same_position(sight.position,c.position)&&
                sight.at_ms>=c.observed_at_ms&&sight.at_ms<=a.observed_at_ms)
                trace.observed_at_ms=sight.at_ms;
        }
    });
    return true;
}
}

bool tactical_knowledge_focus(const ActorSnapshot& a,const WorldSnapshot& w,const KnowledgeFrame& frame,KnowledgeTrace& out) {
    const auto* m=member_knowledge(a,w,frame);if(!m)return false;
    return tactical_knowledge_focus(a,w,*m,out);
}
bool tactical_knowledge_focus(const ActorSnapshot& a,const WorldSnapshot& w,const MemberKnowledge& member,KnowledgeTrace& out) {
    if(member.actor!=a.key||member.ownership_generation!=a.ownership_generation||!member.alive||!member.able)return false;
    const auto* m=&member;
    for(const auto& foe:m->foes)if(foe.id==m->target&&knowledge_trace(a,w,foe,out))return true;
    // The knowledge layer keeps inferred shooters outside known-foe targeting.
    // Use its explicit threat only when backed by a remembered ghost record.
    if(m->threat_valid)for(const auto& foe:m->foes)
        if(foe.id.ghost()&&knowledge_trace(a,w,foe,out)&&same_position(out.position,m->threat))return true;
    if(m->threat_valid)for(auto it=m->contacts.rbegin();it!=m->contacts.rend();++it) {
        if(it->at_ms>w.now_ms||!finite(it->position)||!same_position(it->position,m->threat))continue;
        out={};out.receiver=a.key;out.position=it->position;out.observed_at_ms=it->at_ms;
        out.observer=it->from_report?it->reporter:a.key;out.shared=it->from_report;
        out.receiver_ownership_generation=a.ownership_generation;out.receiver_faction=a.faction;
        out.unidentified=true;return true;
    }
    return false;
}

struct SquadTactics::Impl {
    Config config;
    SquadTuning tuning;
    std::uint64_t epoch=0,sequence=0,next_stamp=1;
    TimeMs last_time=0;
    std::map<ActorKey,Prior,KeyLess> previous;
    std::map<std::uint64_t,Stamp> stamps;
    std::map<Pair,std::deque<ObservationEvidence>> history;
    std::map<Pair,KnowledgeTrace> delivered;
    std::map<Pair,TimeMs> contradicted_at;
    std::map<ActorKey,std::map<std::uint32_t,Denied>,KeyLess> denied;
    Impl(Config c,SquadTuning t):config(c),tuning(t) {}

    void clear() { epoch=sequence=0;next_stamp=1;last_time=0;previous.clear();stamps.clear();history.clear();delivered.clear();contradicted_at.clear();denied.clear(); }
    void clone_from(const Impl& from) {
        epoch=from.epoch;sequence=from.sequence;next_stamp=from.next_stamp;last_time=from.last_time;
        for(const auto& p:from.previous){poll();previous.insert(p);}
        for(const auto& p:from.stamps){
            poll();Stamp value;value.faction=p.second.faction;value.used_at=p.second.used_at;
            value.members.reserve(p.second.members.size());
            for(const auto& k:p.second.members){poll();value.members.push_back(k);}
            for(const auto& k:p.second.owners){poll();value.owners.insert(k);}
            stamps.emplace(p.first,std::move(value));
        }
        for(const auto& p:from.history)for(const auto& e:p.second){poll();history[p.first].push_back(e);}
        for(const auto& p:from.delivered){poll();delivered.insert(p);}
        for(const auto& p:from.contradicted_at){poll();contradicted_at.insert(p);}
        for(const auto& p:from.denied)for(const auto& n:p.second){poll();denied[p.first].insert(n);}
    }
    bool rejected(const ActorSnapshot& a,const WorldSnapshot& w,std::uint32_t node) const {
        auto it=denied.find(a.key);if(it==denied.end())return false;
        auto n=it->second.find(node);if(n==it->second.end())return false;
        return n->second.owner==a.ownership_generation && n->second.restrictions==a.restrictions_revision &&
            n->second.geometry==w.geometry_revision;
    }
    bool eligible(const ActorSnapshot& a,const WorldSnapshot& w) const {
        // Reuse the existing first-response eligibility contract. No geometry
        // work or shared knowledge is added by this check.
        return first_response(a,config,w.now_ms,w.level_epoch,w.geometry_revision,0).kind!=OrderKind::yield;
    }
    bool retained(const Group& a,const Group& b) const {
        std::uint64_t id=0;
        for (const auto* group:{&a,&b}) for (const auto* man:group->actors) {
            const auto it=previous.find(man->key);
            if (it==previous.end() || !it->second.adhoc || it->second.native!=man->native_squad ||
                it->second.owner!=man->ownership_generation) return false;
            if (!id) id=it->second.group;
            if (it->second.group!=id) return false;
        }
        return id!=0;
    }
    std::vector<Group> group(const std::vector<const ActorSnapshot*>& actors,const WorldSnapshot& w) {
        // Native squads are kept intact. Lone actors have distinct seed groups.
        using Seed=std::tuple<std::uint32_t,bool,std::uint64_t,std::uint64_t>;
        std::map<Seed,Group> seeds;
        for (const auto* a:actors) {
        poll();
            const Seed seed={a->faction,a->native_squad==0,a->native_squad?a->native_squad:a->key.id,
                a->native_squad?0:a->key.generation};
            auto& g=seeds[seed];g.faction=a->faction;g.native=a->native_squad;g.actors.push_back(a);
            contacts(*a,[&](const Contact& t) { if (known(t,*a,w.now_ms,config.memory_max_age_ms)) g.enemies.insert(t.key); });
        }
        std::vector<Group> small,groups;
        for (auto& pair:seeds) {
        poll();
            auto& g=pair.second;
            if (g.actors.size()<=2) small.push_back(std::move(g)); else groups.push_back(std::move(g));
        }
        for (std::size_t i=0;i<small.size();++i) {
        poll();
            auto& a=small[i];if (a.merged) continue;
            for (std::size_t j=i+1;j<small.size();++j) {
        poll();
                auto& b=small[j];if (b.merged || a.faction!=b.faction) continue;
                const bool was=retained(a,b);
                bool common=was;
                for (const auto& key:a.enemies) if (b.enemies.count(key)) { common=true;break; }
                if (!common) continue;
                const float range=was?config.regroup_separation_distance:config.nearby_group_distance;
                bool close=true;
                for (const auto* x:a.actors) for (const auto* y:b.actors) {
        poll();
                    const float d=distance_squared(x->position,y->position);
                    if (!std::isfinite(d) || d>range*range) close=false;
                }
                if (!close) continue;
                a.actors.insert(a.actors.end(),b.actors.begin(),b.actors.end());
                Keys intersection;
                std::set_intersection(a.enemies.begin(),a.enemies.end(),b.enemies.begin(),b.enemies.end(),
                    std::inserter(intersection,intersection.end()),KeyLess{});
                a.enemies=std::move(intersection);a.adhoc=true;a.native=0;b.merged=true;
            }
            groups.push_back(std::move(a));
        }
        for (auto& g:groups) std::sort(g.actors.begin(),g.actors.end(),[](const auto* a,const auto* b) {
            return KeyLess{}(a->key,b->key);
        });
        std::sort(groups.begin(),groups.end(),[](const Group& a,const Group& b) {
            return KeyLess{}(a.actors.front()->key,b.actors.front()->key);
        });
        for (auto& g:groups) {
        poll();
            std::vector<ActorKey> members;for (const auto* a:g.actors) members.push_back(a->key);
            auto prev=previous.find(members.front());
            if (prev!=previous.end()) {
                const auto old=stamps.find(prev->second.group);
                if (old!=stamps.end() && old->second.faction==g.faction && old->second.members==members) {
                    bool unchanged=true;
                    for (const auto* a:g.actors) {
        poll();
                        auto p=previous.find(a->key);
                        if (p==previous.end() || p->second.group!=old->first || p->second.native!=a->native_squad ||
                            p->second.owner!=a->ownership_generation) unchanged=false;
                    }
                    if (unchanged) g.stamp=old->first;
                }
            }
            if (!g.stamp) g.stamp=next_stamp++;
            Stamp stamp;stamp.faction=g.faction;stamp.used_at=w.now_ms;stamp.members=std::move(members);
            for(const auto* a:g.actors)stamp.owners[a->key]=a->ownership_generation;
            stamps[g.stamp]=std::move(stamp);
        }
        return groups;
    }
    bool supplied_groups(const WorldSnapshot& w,const std::map<ActorKey,const ActorSnapshot*,KeyLess>& eligible_actors,
        std::vector<Group>& groups) {
        std::map<ActorKey,const ActorSnapshot*,KeyLess> captured;
        for(const auto& a:w.actors){poll();captured.emplace(a.key,&a);}
        std::set<std::uint32_t> side_ids;Keys membership;
        for(const auto& side:w.sides){
            poll();if(!side.side_id||!side_ids.insert(side.side_id).second)return false;
            Group g;g.side_id=side.side_id;g.stamp=side.side_id;g.faction=side.faction;
            bool have_native=false,common_native=true,all_unsquadded=true;std::uint64_t native=0;
            Stamp stamp;stamp.faction=side.faction;stamp.used_at=w.now_ms;
            for(auto key:side.members){
                poll();if(!valid_key(key)||!membership.insert(key).second)return false;
                g.roster.push_back(key);stamp.members.push_back(key);
                auto a=captured.find(key);
                if(a!=captured.end()){
                    if(a->second->faction!=side.faction)return false;
                    const auto own_native=a->second->native_squad;
                    if(!have_native){native=own_native;have_native=true;}else if(native!=own_native)common_native=false;
                    all_unsquadded=all_unsquadded&&own_native==0;
                    stamp.owners[key]=a->second->ownership_generation;
                }
                auto active=eligible_actors.find(key);if(active!=eligible_actors.end())g.actors.push_back(active->second);
            }
            g.native=have_native&&common_native?native:0;
            g.adhoc=have_native&&all_unsquadded;
            // Legacy internal membership searches expect sorted keys; public
            // roster and eligible actors retain supplied slot order separately.
            std::sort(stamp.members.begin(),stamp.members.end(),KeyLess{});
            stamps[g.stamp]=std::move(stamp);groups.push_back(std::move(g));
        }
        return true;
    }
    void expire(TimeMs now) {
        for(auto it=contradicted_at.begin();it!=contradicted_at.end();) {
            if(!recent(it->second,now,config.memory_max_age_ms))it=contradicted_at.erase(it);else ++it;
        }
        for (auto it=history.begin();it!=history.end();) {
        poll();
            auto& queue=it->second;
            queue.erase(std::remove_if(queue.begin(),queue.end(),[&](const ObservationEvidence& e) {
                return !recent(e.trace.observed_at_ms,now,config.memory_max_age_ms);
            }),queue.end());
            if (queue.empty()) it=history.erase(it);else ++it;
        }
        for (auto it=delivered.begin();it!=delivered.end();) {
        poll();
            if (!recent(it->second.observed_at_ms,now,config.memory_max_age_ms)) it=delivered.erase(it);else ++it;
        }
        for (auto it=stamps.begin();it!=stamps.end();) {
        poll();
            if (!recent(it->second.used_at,now,config.memory_max_age_ms)) it=stamps.erase(it);else ++it;
        }
    }
    void capture(const std::vector<Group>& groups,const WorldSnapshot& w) {
        for (const auto& g:groups) for (const auto* a:g.actors) contacts(*a,[&](const Contact& t) {
            if (!known(t,*a,w.now_ms,config.memory_max_age_ms)) return;
            KnowledgeTrace k{a->key,t.key,a->key,t.position,a->snapshot_seq,t.observed_at_ms,personal(t,*a),false};
            auto& q=history[{a->key,t.key}];
            q.push_back({k,g.stamp,w.now_ms});
            while (q.size()>tuning.observation_history_per_pair) q.pop_front();
        });
    }
    void reports(const SquadContext& ctx,const WorldSnapshot& w,
        const std::map<ActorKey,const ActorSnapshot*,KeyLess>& actors,SquadResult& result) {
        for (const auto& r:ctx.delivered_reports) {
        poll();
            const ObservationEvidence* verified=nullptr;
            const auto receiver=actors.find(r.receiver);
            const auto found=history.find({r.source,r.target});
            if (r.level_epoch==w.level_epoch && receiver!=actors.end() && valid_key(r.source) && valid_key(r.target)
                && r.source!=r.receiver && r.target!=r.receiver && finite(r.observed_position)
                && r.source_snapshot_seq<w.sequence && r.delivered_at_ms<=w.now_ms
                && recent(r.observed_at_ms,w.now_ms,config.memory_max_age_ms) && found!=history.end()) {
                for (const auto& e:found->second) {
        poll();
                    auto stamp=stamps.find(e.stamp);
                    if (e.trace.source_snapshot_seq==r.source_snapshot_seq && e.trace.observed_at_ms==r.observed_at_ms
                        && same_position(e.trace.position,r.observed_position) && e.captured_at<r.delivered_at_ms
                        && stamp!=stamps.end() && stamp->second.faction==receiver->second->faction
                        && contains(stamp->second.members,r.receiver)
                        && stamp->second.owners.at(r.receiver)==receiver->second->ownership_generation) { verified=&e;break; }
                }
            }
            if (!verified) { ++result.rejected_reports;continue; }
            auto k=verified->trace;k.receiver=r.receiver;k.shared=true;k.personally_visible=false;
            k.receiver_ownership_generation=receiver->second->ownership_generation;k.receiver_faction=receiver->second->faction;
            auto prior=delivered.find({r.receiver,r.target});
            if (prior==delivered.end() || prior->second.receiver_ownership_generation!=k.receiver_ownership_generation ||
                prior->second.receiver_faction!=k.receiver_faction || k.observed_at_ms>prior->second.observed_at_ms ||
                (k.observed_at_ms==prior->second.observed_at_ms && KeyLess{}(k.observer,prior->second.observer)))
                delivered[{r.receiver,r.target}]=k;
            ++result.verified_reports;
        }
    }
    std::vector<KnowledgeTrace> knowledge(const ActorSnapshot& a,const WorldSnapshot& w) {
        std::map<ActorKey,KnowledgeTrace,KeyLess> known_targets;
        contacts(a,[&](const Contact& c) {
            if(valid_key(c.key) && c.observed_at_ms<=w.now_ms &&
                (!c.hostile||!c.alive_when_observed||(c.wounded&&!c.finish_allowed))) {
                auto& at=contradicted_at[{a.key,c.key}];at=(std::max)(at,c.observed_at_ms);
            }
        });
        for(std::size_t i=0;i<a.seen_dead.count;++i)if(valid_key(a.seen_dead.items[i]))
            contradicted_at[{a.key,a.seen_dead.items[i]}]=w.now_ms;
        contacts(a,[&](const Contact& t) { if (!known(t,a,w.now_ms,config.memory_max_age_ms)) return;
            auto contradiction=contradicted_at.find({a.key,t.key});
            if(contradiction!=contradicted_at.end() && contradiction->second>=t.observed_at_ms)return;
            KnowledgeTrace k{a.key,t.key,a.key,t.position,a.snapshot_seq,t.observed_at_ms,personal(t,a),false};
            auto it=known_targets.find(t.key);
            if (it==known_targets.end() || k.personally_visible || (!it->second.personally_visible &&
                k.observed_at_ms>it->second.observed_at_ms)) known_targets[t.key]=k;
        });
        auto begin=delivered.lower_bound({a.key,ActorKey{0,0}});
        for (auto it=begin;it!=delivered.end() && it->first.a==a.key;++it) {
            const auto& k=it->second;auto own=known_targets.find(k.target);
            if(k.receiver_ownership_generation!=a.ownership_generation || k.receiver_faction!=a.faction)continue;
            bool contradicted=false;
            auto historical=contradicted_at.find({a.key,k.target});
            if(historical!=contradicted_at.end() && historical->second>=k.observed_at_ms)contradicted=true;
            contacts(a,[&](const Contact& c) {
                if(c.key==k.target && c.observed_at_ms>=k.observed_at_ms && c.observed_at_ms<=w.now_ms &&
                    (!c.hostile || !c.alive_when_observed || (c.wounded&&!c.finish_allowed)))contradicted=true;
            });
            for(std::size_t i=0;i<a.seen_dead.count;++i)if(a.seen_dead.items[i]==k.target)contradicted=true;
            if(contradicted)continue;
            if (own==known_targets.end() || (!own->second.personally_visible && k.observed_at_ms>own->second.observed_at_ms))
                known_targets[k.target]=k;
        }
        std::vector<KnowledgeTrace> result;for (const auto& p:known_targets) result.push_back(p.second);
        return result;
    }
    void elements(const Group& g,SquadView& view,std::map<ActorKey,Prior,KeyLess>& next) {
        const auto count=g.actors.size();const auto base_count=std::max<std::size_t>(1,count/2);
        auto prior_for=[&](const ActorSnapshot& a)->const Prior* {
            auto p=previous.find(a.key);
            if(p==previous.end() || p->second.native!=a.native_squad || p->second.owner!=a.ownership_generation ||
                p->second.adhoc!=g.adhoc || ((g.adhoc||tuning.use_side_roster)&&p->second.group!=g.stamp))return nullptr;
            auto stamp=stamps.find(p->second.group);
            if(stamp==stamps.end() || stamp->second.faction!=g.faction || stamp->second.members.size()<2)return nullptr;
            return &p->second;
        };
        bool sticky=false;for(const auto* a:g.actors)if(prior_for(*a))sticky=true;
        Keys base,moving;
        if (count<2) moving.insert(g.actors.front()->key);
        else if (!sticky) {
            for (const auto* a:g.actors) {
        poll();
                if (a->key==view.leader) moving.insert(a->key);
                else if (base.size()<base_count) base.insert(a->key);else moving.insert(a->key);
            }
        } else {
            for (const auto* a:g.actors) {
        poll();
                const auto* p=prior_for(*a);
                if (p && p->role==roles::base && base.size()<base_count) base.insert(a->key);
            }
            for (const auto* a:g.actors) if (!base.count(a->key)) {
                const auto* p=prior_for(*a);
                const bool was_mover=p && p->role==roles::mover;
                if (!was_mover && base.size()<base_count && a->key!=view.leader) base.insert(a->key);
                else moving.insert(a->key);
            }
            while (base.size()<base_count && moving.size()>1) {
                auto pick=std::prev(moving.end());if (*pick==view.leader) --pick;
                base.insert(*pick);moving.erase(pick);
            }
        }
        view.base.assign(base.begin(),base.end());view.movers.assign(moving.begin(),moving.end());
        for (const auto* a:g.actors) next[a->key]={g.stamp,a->native_squad,a->ownership_generation,
            base.count(a->key)?roles::base:roles::mover,g.adhoc};
    }
};

SquadTactics::SquadTactics(Config c,SquadTuning t):impl_(new Impl(c,t)) {}
SquadTactics::~SquadTactics()=default;
SquadTactics::SquadTactics(SquadTactics&&) noexcept=default;
SquadTactics& SquadTactics::operator=(SquadTactics&&) noexcept=default;
void SquadTactics::reset() { impl_->clear(); }

SquadResult SquadTactics::update(const WorldSnapshot& w,const std::vector<Order>& current,const SquadContext& ctx,
    const std::function<bool()>& cancelled) {
    SquadResult result;result.level_epoch=w.level_epoch;result.snapshot_seq=w.sequence;
    CancellationScope cancellation_scope(cancelled);
    try {
    poll();
    std::unique_ptr<Impl> staged(new Impl(impl_->config,impl_->tuning));
    staged->clone_from(*impl_);auto& s=*staged;
    if (!w.level_epoch || !w.sequence || (s.epoch && w.level_epoch<s.epoch) ||
        (w.level_epoch==s.epoch && (w.sequence<=s.sequence || w.now_ms<s.last_time)) ||
        !std::isfinite(s.config.minimum_goal_separation) || s.config.minimum_goal_separation<0 ||
        !std::isfinite(s.config.nearby_group_distance) || s.config.nearby_group_distance<0 ||
        !std::isfinite(s.config.regroup_separation_distance) || s.config.regroup_separation_distance<s.config.nearby_group_distance ||
        !std::isfinite(s.config.cover_exposure_cost_m) || s.config.cover_exposure_cost_m<0 ||
        !std::isfinite(s.config.cover_unknown_cost_m) || s.config.cover_unknown_cost_m<0 ||
        !std::isfinite(s.config.cover_move_hysteresis_m) || s.config.cover_move_hysteresis_m<0 ||
        !std::isfinite(s.config.suppressed_exposure_cost_m) || s.config.suppressed_exposure_cost_m<0 ||
        !std::isfinite(s.config.cover_far_cost_m_per_m) || s.config.cover_far_cost_m_per_m<0 ||
        !std::isfinite(s.config.immediate_cover_distance) || s.config.immediate_cover_distance<0 ||
        !s.tuning.observation_history_per_pair) return result;
    std::set<std::uint32_t> ids;
    for (const auto& a:w.actors) if (!valid_key(a.key) || a.level_epoch!=w.level_epoch || a.snapshot_seq!=w.sequence
        || a.observed_at_ms!=w.now_ms || !ids.insert(a.key.id).second ||
        a.visible_enemies.count>a.visible_enemies.items.size() || a.seen_dead.count>a.seen_dead.items.size() ||
        a.heard.count>a.heard.items.size() || a.hits.count>a.hits.items.size()) return result;
    for(const auto& side:w.sides)for(const auto& target:side.known_enemies) {
        poll();if(target.reporters.count>target.reporters.items.size())return result;
    }
    if (w.level_epoch!=s.epoch) s.clear();
    s.epoch=w.level_epoch;s.sequence=w.sequence;s.last_time=w.now_ms;s.expire(w.now_ms);
    std::vector<const ActorSnapshot*> actors;
    std::map<ActorKey,const ActorSnapshot*,KeyLess> by_key;
    for (const auto& a:w.actors) if (s.eligible(a,w)) { actors.push_back(&a);by_key[a.key]=&a; }
    std::sort(actors.begin(),actors.end(),[](const auto* a,const auto* b){return KeyLess{}(a->key,b->key);});
    for(auto it=s.denied.begin();it!=s.denied.end();) {
        poll();auto a=by_key.find(it->first);
        if(a==by_key.end()){it=s.denied.erase(it);continue;}
        for(auto n=it->second.begin();n!=it->second.end();) {
            poll();const auto& d=n->second;
            if(d.owner!=a->second->ownership_generation || d.restrictions!=a->second->restrictions_revision ||
                d.geometry!=w.geometry_revision)n=it->second.erase(n);else ++n;
        }
        if(it->second.empty())it=s.denied.erase(it);else ++it;
    }
    for(const auto* a:actors) {
        poll();const auto& r=a->last_rejected;
        if(r.order_seq && r.node!=invalid_node && r.restrictions_revision==a->restrictions_revision &&
            (r.reason==reject_goal_invalid || r.reason==reject_goal_inaccessible))
            s.denied[a->key][r.node]={a->ownership_generation,a->restrictions_revision,w.geometry_revision};
    }
    std::vector<Group> groups;
    if(s.tuning.use_side_roster){if(!s.supplied_groups(w,by_key,groups))return result;}
    else groups=s.group(actors,w);
    if(s.tuning.use_side_roster&&ctx.knowledge&&ctx.knowledge->accepted&&!ctx.knowledge->cancelled
        &&ctx.knowledge->level_epoch==w.level_epoch&&ctx.knowledge->sequence==w.sequence){
        std::map<ActorKey,std::uint32_t,KeyLess> assigned_side;
        for(const auto& g:groups)for(auto key:g.roster){poll();assigned_side.emplace(key,g.side_id);}
        Keys observed;
        for(const auto& m:ctx.knowledge->members){poll();if(!observed.insert(m.actor).second)return result;
            auto side=assigned_side.find(m.actor);if(side!=assigned_side.end()&&m.side!=side->second)return result;}
    }
    if(s.tuning.use_side_roster)result.rejected_reports=ctx.delivered_reports.size();
    else if(!ctx.knowledge){s.capture(groups,w);s.reports(ctx,w,by_key,result);}
    std::map<ActorKey,const Order*,KeyLess> orders;
    for (const auto& o:current) {
        poll();
        const auto a=by_key.find(o.actor);
        if (a==by_key.end() || o.kind==OrderKind::yield || !publishable(o,*a->second,w.now_ms,w.level_epoch,w.geometry_revision)) continue;
        auto old=orders.find(o.actor);if (old==orders.end() || old->second->order_seq<o.order_seq) orders[o.actor]=&o;
    }
    std::map<ActorKey,float,KeyLess> pressure;
    for (const auto& f:ctx.suppression) if (by_key.count(f.observer) && f.level_epoch==w.level_epoch &&
        f.snapshot_seq==w.sequence && f.observed_at_ms==w.now_ms && std::isfinite(f.intensity) && f.intensity>=0 && f.intensity<=1)
        pressure[f.observer]=std::max(pressure[f.observer],f.intensity);
    if(ctx.knowledge)for(const auto* a:actors) {
        const auto* m=member_knowledge(*a,w,*ctx.knowledge);
        if(m&&std::isfinite(m->meter)&&m->meter>=0)
            pressure[a->key]=std::max(pressure[a->key],std::min(1.f,float(m->meter_level)/float(impl_->config.knowledge.meter_thresholds.size())));
    }
    std::vector<Reservation> occupied,assigned;
    for (const auto* a:actors) occupied.push_back({a->key,a->faction,a->node_id,a->position});
    for(const auto& entry:orders) {
        const auto& a=*by_key.at(entry.first);const auto& o=*entry.second;
        assigned.push_back({a.key,a.faction,o.goal_node,o.goal_position});
    }
    const float separation2=s.config.minimum_goal_separation*s.config.minimum_goal_separation;
    auto blocked=[&](const ActorSnapshot& a,std::uint32_t node,const Vec3& p) {
        for (const auto* reservations:{&occupied,&assigned}) for (const auto& r:*reservations) {
        poll();
            if (r.actor==a.key || r.faction!=a.faction) continue;
            if (r.node==node || distance_squared(r.position,p)<separation2) return true;
        }
        return false;
    };
    std::map<ActorKey,Prior,KeyLess> next;
    for (const auto& g:groups) {
        poll();
        SquadView view;view.group_id=g.stamp;view.side_id=g.side_id;view.native_squad=g.native;view.faction=g.faction;
        view.ad_hoc=g.adhoc;if(!g.actors.empty())view.leader=g.actors.front()->key;
        if(s.tuning.use_side_roster){for(auto key:g.roster){poll();view.members.push_back(key);}}
        else for (const auto* a:g.actors) view.members.push_back(a->key);
        if(!g.actors.empty())s.elements(g,view,next);
        Keys casualties;
        for(const auto* a:g.actors)for(std::size_t i=0;i<a->seen_dead.count;++i) {
            if(!valid_key(a->seen_dead.items[i]))continue;
            casualties.insert(a->seen_dead.items[i]);
            view.casualty_observations.push_back({a->key,a->seen_dead.items[i],w.level_epoch,w.sequence,w.now_ms});
        }
        for (const auto& f:ctx.seen_casualties) if ((s.tuning.use_side_roster?
            (by_key.count(f.observer)&&std::find(view.members.begin(),view.members.end(),f.observer)!=view.members.end()):contains(view.members,f.observer)) && valid_key(f.casualty) &&
            f.level_epoch==w.level_epoch && f.snapshot_seq==w.sequence && f.observed_at_ms==w.now_ms) {
            casualties.insert(f.casualty);
            view.casualty_observations.push_back(f);
        }
        view.known_casualties.assign(casualties.begin(),casualties.end());
        for (const auto* a:g.actors) {
        poll();
            std::vector<KnowledgeTrace> knowledge;
            if(ctx.knowledge) {
                const auto* m=member_knowledge(*a,w,*ctx.knowledge);
                if(m)for(const auto& foe:m->foes) {
                    KnowledgeTrace trace;if(knowledge_trace(*a,w,foe,trace))knowledge.push_back(trace);
                }
            } else knowledge=s.knowledge(*a,w);
            view.known.insert(view.known.end(),knowledge.begin(),knowledge.end());
            auto base=orders.find(a->key);
            if (base==orders.end()) { ++result.actors_without_current_order;continue; }
            const auto& initial=*base->second;
            if(initial.portal.valid || initial.throw_order.valid ||
                (initial.kind!=OrderKind::hold && initial.kind!=OrderKind::cover && initial.kind!=OrderKind::reload)) {
                ++result.preserved_planned_orders;continue;
            }
            ProposedOrder p;p.order=*base->second;p.expected_previous_order_seq=p.order.order_seq;p.group_id=g.stamp;
            p.order.role=next[a->key].role;p.order.snapshot_seq=w.sequence;
            const KnowledgeTrace* focus=nullptr;float closest=0;
            KnowledgeTrace selected_knowledge;
            if(ctx.knowledge&&tactical_knowledge_focus(*a,w,*ctx.knowledge,selected_knowledge))focus=&selected_knowledge;
            const auto selected=select_contact(*a,s.config,w.now_ms);
            const bool selected_visible=selected&&personal(*selected,*a);
            for (const auto& k:knowledge) {
        poll();
                if(ctx.knowledge)break;
                const float d=distance_squared(a->position,k.position);if (!std::isfinite(d)) continue;
                if(selected_visible&&k.target==selected->key&&k.personally_visible){focus=&k;closest=d;break;}
                if (!focus || (k.personally_visible && !focus->personally_visible) ||
                    (k.personally_visible==focus->personally_visible && (d<closest ||
                        (d==closest && KeyLess{}(k.target,focus->target))))) {focus=&k;closest=d;}
            }
            p.order.fire.allowed=false;p.order.fire.at_valid=false;p.order.fire.until_ms=w.now_ms;
            p.order.flags &= ~order_cover_fire;p.order.look.target=ActorKey{};
            if (focus) {
                p.has_knowledge=true;p.knowledge=*focus;p.order.look.kind=LookKind::point;p.order.look.value=focus->position;
                if (focus->personally_visible) {p.order.look.target=focus->target;p.order.look.kind=LookKind::enemy;}
                p.order.fire.allowed=s.config.fire_window_ms!=0 && focus->personally_visible && a->weapon.ammo_in_mag>0 && !a->reloading &&
                    a->weapon.state!=WeaponState::reloading && !(a->flags&flag_dont_shoot);
                if(p.order.fire.allowed)p.order.fire.until_ms=fire_window_until(s.config,w.now_ms,p.order.expires_at_ms);
            } else {
                // first_response may return an absolute remembered point. Preserve
                // its type/target too; relabelling that position as a direction
                // aims relative to the world origin instead of the known foe.
                p.order.look=first_response(*a,s.config,w.now_ms,w.level_epoch,w.geometry_revision,0).look;
            }
            const bool reloading=a->weapon.ammo_in_mag==0 || a->reloading || a->weapon.state==WeaponState::reloading;
            if (reloading) {
                p.order.kind=OrderKind::reload;p.order.goal_node=a->node_id;p.order.goal_position=a->position;p.order.pace=Pace::stand;
            }
            if (focus) {
                const CoverSpot* choice=nullptr;float cost=0;
                const CoverSpot* held=nullptr;float held_cost=0;
                const CoverSpot* moving=nullptr;float moving_cost=0;
                for (const auto& cache:ctx.cover) {
        poll();
                if (cache.actor!=a->key || cache.level_epoch!=w.level_epoch || cache.snapshot_seq!=w.sequence ||
                    cache.ownership_generation!=a->ownership_generation || cache.origin_node!=a->node_id ||
                    cache.restrictions_revision!=a->restrictions_revision ||
                    cache.observed_at_ms!=a->observed_at_ms) continue;
                for (const auto& c:cache.spots) {
        poll();
                    const float distance=distance_squared(a->position,c.position);
                    if (c.node==invalid_node || !finite(c.position) || c.accessible==Evidence::no || c.reachable==Evidence::no || c.protection_known==Evidence::no ||
                        (!s.tuning.allow_pending_navigation && (c.accessible!=Evidence::yes || c.reachable!=Evidence::yes || c.protection_known!=Evidence::yes)) ||
                        !c.geometry_revision || c.geometry_revision!=w.geometry_revision ||
                        (c.accessible==Evidence::unknown ? c.restrictions_revision!=0 :
                            (!c.restrictions_revision||c.restrictions_revision!=a->restrictions_revision)) ||
                        (focus->foe.ghost()||focus->unidentified ? (cache.knowledge_threat!=focus->foe ||
                            cache.knowledge_unidentified!=focus->unidentified ||
                            !same_position(cache.knowledge_threat_position,focus->position) ||
                            cache.knowledge_observed_at_ms!=focus->observed_at_ms || valid_key(c.threat_key)) :
                            (c.threat_key!=focus->target || c.threat_observed_at_ms!=focus->observed_at_ms)) ||
                        !std::isfinite(c.lo) || c.lo<0 || c.lo>1 || !std::isfinite(c.dc) ||
                        c.dc<0 || !std::isfinite(distance)) continue;
                    if(s.rejected(*a,w,c.node))continue;
                    if (blocked(*a,c.node,c.position)) {p.cover_conflict=true;continue;}
                    // Costs, never vetoes. Unknown protection is not a low-exposure measurement, and measured protection
                    // with no low blocking ray (0.7 / 1.1 m) shields nothing a crouched man needs: both price full
                    // exposure. A spot beyond immediate_cover_distance pays cover_far_cost_m_per_m for every metre past it.
                    const bool unknown_navigation=c.accessible==Evidence::unknown || c.reachable==Evidence::unknown;
                    const bool unknown_protection=c.protection_known==Evidence::unknown;
                    const bool no_low_ray=c.protection_known==Evidence::yes && !(c.blocked[0]||c.blocked[1]);
                    const float exposure=(unknown_protection||no_low_ray)?1.f:c.lo;
                    const float beyond=(std::max)(0.f,(std::max)(c.dc,std::sqrt(distance))-s.config.immediate_cover_distance);
                    const float score=c.dc+exposure*(s.config.cover_exposure_cost_m+s.config.suppressed_exposure_cost_m*pressure[a->key])+
                        (float(unknown_navigation)+float(unknown_protection))*s.config.cover_unknown_cost_m+
                        beyond*s.config.cover_far_cost_m_per_m;
                    if(!std::isfinite(score))continue;
                    if(c.node==initial.goal_node && (initial.kind==OrderKind::cover || initial.kind==OrderKind::hold ||
                        (reloading&&initial.kind==OrderKind::reload))) {
                        if(!held || score<held_cost){held=&c;held_cost=score;}
                        if((initial.kind==OrderKind::cover || (reloading&&initial.kind==OrderKind::reload)) &&
                            same_position(c.position,initial.goal_position) &&
                            c.protection_known==Evidence::yes &&
                            (!reloading || ((c.blocked[0]||c.blocked[1])&&c.lo<1)) &&
                            (!moving || score<moving_cost)){moving=&c;moving_cost=score;}
                    }
                    if (!choice || score<cost || (score==cost && c.node<choice->node)) {choice=&c;cost=score;}
                }}
                // Retain a held destination only when it independently passed all
                // CURRENT threat, revision, reservation and rejection checks above.
                // Finish a native-applied cover move before selecting a cheaper
                // destination. This is not inferred from issuing a proposal: the
                // exact order must be executing, with a current accessible goal
                // and independently revalidated cover evidence. Worker navigation
                // may remain unknown: current native accessibility is a separate
                // fact, not permission to rewrite the query's evidence. Arrival returns
                // to normal hysteresis. Reload may keep this proven movement,
                // but cannot start a new cover trip or borrow a room/aid role.
                const bool in_motion=moving && a->accessible_current_order &&
                    (!reloading || (initial.pace!=Pace::stand&&!initial.assignment.generation&&!valid_key(initial.aid)&&
                        a->weapon.id!=invalid_id&&(a->weapon.ammo_in_mag||a->weapon.ammo_reserve))) &&
                    a->last_applied.order_seq==initial.order_seq && a->last_applied.at_ms>=initial.issued_at_ms &&
                    a->last_applied.at_ms<=a->observed_at_ms && recent(a->last_applied.at_ms,w.now_ms,s.config.snapshot_max_age_ms) &&
                    distance_squared(a->position,initial.goal_position)>initial.arrival_radius_m*initial.arrival_radius_m;
                if(in_motion){choice=moving;cost=moving_cost;}
                else if(choice && held && choice->node!=held->node && cost+s.config.cover_move_hysteresis_m>=held_cost){
                    choice=held;cost=held_cost;
                }
                if(reloading&&!in_motion)choice=nullptr;
                if (choice) {
                    p.cover_cost=cost;p.has_cover_evidence=true;p.cover_evidence=*choice;p.order.goal_node=choice->node;p.order.goal_position=choice->position;
                    p.navigation_pending=choice->accessible!=Evidence::yes || choice->reachable!=Evidence::yes;
                    const bool walk=distance_squared(a->position,choice->position)>p.order.arrival_radius_m*p.order.arrival_radius_m;
                    p.order.kind=reloading?OrderKind::reload:(walk?OrderKind::cover:OrderKind::hold);
                    p.order.pace=walk?Pace::walk:Pace::stand;
                }
            }
            // An existing bootstrap movement remains useful when no replacement
            // cover was found, but cannot retain an already claimed destination.
            if (p.order.kind==OrderKind::cover && blocked(*a,p.order.goal_node,p.order.goal_position)) {
                p.order.kind=OrderKind::hold;p.order.goal_node=a->node_id;p.order.goal_position=a->position;p.order.pace=Pace::stand;p.cover_conflict=true;
            }
            assigned.erase(std::remove_if(assigned.begin(),assigned.end(),[&](const Reservation& r) {
                return r.actor==a->key;
            }),assigned.end());
            assigned.push_back({a->key,a->faction,p.order.goal_node,p.order.goal_position});
            result.proposals.push_back(std::move(p));
        }
        result.groups.push_back(std::move(view));
    }
    s.previous=std::move(next);poll();impl_=std::move(staged);result.accepted=true;return result;
    } catch(const Cancelled&) {
        SquadResult stopped;stopped.level_epoch=w.level_epoch;stopped.snapshot_seq=w.sequence;stopped.cancelled=true;return stopped;
    }
}

}}} // namespace painter::combat::core
