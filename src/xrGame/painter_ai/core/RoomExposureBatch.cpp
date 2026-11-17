#include "RoomExposureBatch.h"
#include <algorithm>
#include <limits>
#include <map>
#include <mutex>
#include <set>

namespace painter { namespace combat { namespace core {
namespace {
using Key=std::pair<std::uint32_t,std::uint64_t>;
Key key(ActorKey a){return {a.id,a.generation};}
bool eq(Vec3 a,Vec3 b){return distance_squared(a,b)==0;}
bool same(const ExposureGeometryStamp&a,const ExposureGeometryStamp&b){return a.level_epoch==b.level_epoch&&a.map_hash==b.map_hash&&
    a.geometry_revision==b.geometry_revision&&a.data_revision==b.data_revision&&a.source_basis==b.source_basis;}
bool add(std::size_t&a,std::size_t b){if(b>(std::numeric_limits<std::size_t>::max)()-a)return false;a+=b;return true;}
}
struct RoomExposureBatch::Impl {
    struct Store {
        ExposureGeometryStamp stamp;std::unique_ptr<RoomExposureCache> cache,candidate;
        ExposureCapture capture;
        std::uint64_t transaction=0,touch=0;bool replacing=false,retired=false;
        RoomExposureCache* active(){return replacing?candidate.get():cache.get();}
    };
    RoomExposureDispatchPolicy policy;mutable std::mutex mutex;
    std::shared_ptr<ExposureGeometryBuilder> builder;
    ExposureGeometryStamp builder_stamp;
    std::map<std::uint32_t,Store> stores;std::vector<std::uint32_t> active;
    std::uint64_t serial=0,token=0;std::uint32_t after_side=0;
    std::size_t charge=0,reserved=0,geometry_bytes=0;bool valid=false,failed=false;
    RoomExposureBatchStats stats;
    explicit Impl(RoomExposureDispatchPolicy p):policy(p){
        charge=p.cache.memory_bytes;
        if(!p.cache.memory_bytes||!p.cache.transaction_records||!p.store_overhead_bytes||!p.total_memory_bytes)return;
        if(p.cache.observer_capacity>(std::numeric_limits<std::size_t>::max)()/sizeof(ExposureObserver))return;
        if(p.geometry){
            if(!p.geometry_basis||p.geometry->source_range_m!=p.cache.source_range_m||p.geometry->source_floor_m!=p.cache.source_floor_m)return;
            policy.geometry=std::make_shared<const ExposureGeometryPolicy>(*p.geometry);
            geometry_bytes=p.geometry->memory_bytes;
            if(!add(geometry_bytes,sizeof(ExposureGeometryPolicy))||!add(geometry_bytes,p.store_overhead_bytes))return;
        }
        valid=add(charge,sizeof(Store))&&add(charge,p.store_overhead_bytes)&&
            add(charge,p.cache.observer_capacity*sizeof(ExposureObserver))&&geometry_bytes<=p.total_memory_bytes&&charge<=p.total_memory_bytes-geometry_bytes;
        if(valid)reserved=geometry_bytes;
    }
    bool reserve(std::uint32_t protected_side){
        while(policy.total_memory_bytes-reserved<charge){
            auto victim=stores.end();
            for(auto i=stores.begin();i!=stores.end();++i)if(i->first!=protected_side&&!i->second.transaction&&
                (victim==stores.end()||i->second.touch<victim->second.touch))victim=i;
            if(victim==stores.end())return false;
            reserved-=charge;stores.erase(victim);++stats.evictions;
        }
        reserved+=charge;return true;
    }
    bool close(bool commit){
        bool ok=!failed;
        for(auto&row:stores)if(row.second.transaction){const auto state=row.second.active()->stats();
            if(state.transaction_failed||state.rolling_back||!state.transaction_active||state.transaction_token!=row.second.transaction)ok=false;}
        const bool publish=commit&&ok;
        for(auto found=stores.begin();found!=stores.end();){auto&s=found->second;auto*c=s.active();
            if(!c||!s.transaction){if(s.candidate){s.candidate.reset();s.replacing=false;}
                if(!s.cache)found=stores.erase(found);else ++found;continue;}
            if(publish){const auto r=c->commit_transaction(s.transaction);if(!r.committed){ok=false;c->rollback_transaction(s.transaction);}}
            else c->rollback_transaction(s.transaction);
            if(s.replacing){
                // A first binding has no earlier geometry to restore. Keep its
                // static work after clean rollback; all sight authority was
                // removed by rollback_transaction above.
                if((publish&&ok)||!s.cache){s.cache=std::move(s.candidate);s.stamp=s.capture.stamp;}
                else s.candidate.reset();
            }
            s.replacing=false;s.transaction=0;
            if(!s.cache)found=stores.erase(found);else ++found;
        }
        for(auto i=stores.begin();i!=stores.end();)if(i->second.retired)i=stores.erase(i);else ++i;
        reserved=geometry_bytes+stores.size()*charge;active.clear();token=0;failed=false;return ok;
    }
};
RoomExposureBatch::RoomExposureBatch(RoomExposureDispatchPolicy p):impl_(new Impl(p)){}
RoomExposureBatch::~RoomExposureBatch()=default;
std::shared_ptr<const ExposureGeometryInput> RoomExposureBatch::prepare_geometry(const WorldSnapshot&w,const EngineServices&services,
    ExposureWorkQuota&quota,const std::function<bool()>&stop){
    auto&d=*impl_;std::unique_lock<std::mutex> lock(d.mutex,std::try_to_lock);
    if(!lock||d.token||!d.valid||!d.policy.geometry||!services.buildings)return {};
    const ExposureGeometryStamp stamp{w.level_epoch,w.map_hash,w.geometry_revision,services.buildings->data_version,d.policy.geometry_basis};
    // The alias keeps a completed basis immutable while a Work borrows it.
    // Do not allocate another builder beside a still-borrowed older basis.
    if(d.builder&&!same(stamp,d.builder_stamp)){
        if(d.builder.use_count()!=1)return {};
        d.builder.reset();
    }
    if(!d.builder){d.builder=std::make_shared<ExposureGeometryBuilder>(*d.policy.geometry);d.builder_stamp=stamp;}
    const auto before_work=quota.work_used,before_queries=quota.queries_used;
    const auto result=d.builder->advance(stamp,services,quota,stop);
    d.stats.work+=quota.work_used-before_work;d.stats.queries+=quota.queries_used-before_queries;
    const auto* geometry=d.builder->geometry();
    if(!result.complete||!geometry)return {};
    return std::shared_ptr<const ExposureGeometryInput>(d.builder,geometry);
}
std::uint64_t RoomExposureBatch::begin(const WorldSnapshot&w,const KnowledgeFrame&k,
    const std::shared_ptr<const ExposureGeometryInput>&geometry,const std::vector<OutdoorCapturedEye>&eyes,
    const EngineServices&services,const std::function<bool()>&stop,const std::set<std::uint32_t>&reset_sides){
    auto&d=*impl_;std::unique_lock<std::mutex> lock(d.mutex,std::try_to_lock);
    if(!lock||d.token||!d.valid||!geometry||!services.buildings||!services.buildings->locate||!k.accepted||k.cancelled||
        k.level_epoch!=w.level_epoch||k.sequence!=w.sequence||!geometry->paired_links_complete||
        geometry->stamp.level_epoch!=w.level_epoch||geometry->stamp.map_hash!=w.map_hash||geometry->stamp.geometry_revision!=w.geometry_revision||
        geometry->stamp.data_revision!=services.buildings->data_version||!geometry->stamp.source_basis||
        d.serial==(std::numeric_limits<std::uint64_t>::max)())return 0;
    const auto poll=[&](){if(stop&&stop())throw 1;};
    try{
        poll();d.token=++d.serial;d.failed=false;d.stats.work=d.stats.queries=d.stats.skipped=0;
        std::map<Key,const ActorSnapshot*> actors;std::map<Key,const OutdoorCapturedEye*> measured;std::map<Key,const MemberKnowledge*> members;
        for(const auto&a:w.actors){poll();if(a.level_epoch!=w.level_epoch||a.snapshot_seq!=w.sequence||!actors.emplace(key(a.key),&a).second)throw 1;}
        for(const auto&m:k.members){poll();if(!members.emplace(key(m.actor),&m).second)throw 1;}
        for(const auto&e:eyes){poll();auto inserted=measured.emplace(key(e.actor),&e);if(!inserted.second)inserted.first->second=nullptr;}
        std::map<std::uint32_t,const SideSnapshot*> sides;
        for(const auto&s:w.sides){poll();if(!sides.emplace(s.side_id,&s).second)throw 1;}
        std::vector<const SideSnapshot*> order;const auto split=sides.upper_bound(d.after_side);
        for(auto i=split;i!=sides.end();++i)order.push_back(i->second);for(auto i=sides.begin();i!=split;++i)order.push_back(i->second);
        bool rotated=false;
        for(const auto*side:order){poll();const auto&s=*side;
            if(s.mission.kind!=MissionKind::clear_building&&s.mission.kind!=MissionKind::defend&&s.mission.kind!=MissionKind::hold)continue;
            if(!rotated){d.after_side=s.side_id;rotated=true;}
            ExposureCapture capture;capture.stamp=geometry->stamp;capture.side=s.side_id;capture.faction=s.faction;capture.group=s.group;
            capture.snapshot_seq=w.sequence;capture.now_ms=w.now_ms;capture.roster_revision=s.mission.revision;
            std::set<Key> roster;bool malformed=false;
            for(auto member:s.members){poll();auto a=actors.find(key(member));
                if(!valid_key(member)||!roster.insert(key(member)).second){malformed=true;break;}
                // Authenticated rosters may retain observed passive casualties.
                // Absence supplies no eye/head/health and no observation discount.
                if(a==actors.end())continue;
                if(a->second->faction!=s.faction){malformed=true;break;}
                const auto&actor=*a->second;ExposureObserver observer;observer.actor=actor.key;observer.owner=actor.ownership_generation;
                observer.snapshot_seq=w.sequence;observer.observed_at_ms=actor.observed_at_ms;observer.faction=s.faction;observer.head=actor.head_dir;
                auto eye=measured.find(key(member));
                if(eye!=measured.end()&&eye->second){const auto&e=*eye->second;
                    if(e.measured&&e.epoch==w.level_epoch&&e.sequence==w.sequence&&e.owner==actor.ownership_generation&&
                        e.restrictions==actor.restrictions_revision&&e.observed_at_ms==actor.observed_at_ms&&eq(e.feet,actor.position)&&finite(e.eye)&&
                        actor.alive&&!actor.wounded&&!actor.yield_reasons&&(actor.flags&flag_admitted)&&finite(actor.head_dir)){
                        observer.eye=e.eye;const auto room=services.buildings->locate(actor.node_id);observer.building=room.first;observer.room=room.second;
                        observer.eligible=room.first!=invalid_room&&room.second!=invalid_room;
                    }
                }
                capture.observers.push_back(observer);
                auto m=members.find(key(member));if(m!=members.end()&&m->second->side==s.side_id&&m->second->ownership_generation==actor.ownership_generation)
                    capture.contact_active=capture.contact_active||m->second->has_hit||m->second->known_count||m->second->unseen_count;
            }
            if(malformed||capture.observers.size()>d.policy.cache.observer_capacity){++d.stats.skipped;continue;}
            std::sort(capture.observers.begin(),capture.observers.end(),[](const ExposureObserver&a,const ExposureObserver&b){return a.actor.id<b.actor.id;});
            auto found=d.stores.find(s.side_id);
            const bool replace=found==d.stores.end()||!same(found->second.stamp,geometry->stamp);
            if(replace){
                if(!d.reserve(s.side_id)){++d.stats.skipped;continue;}
                auto candidate=std::make_unique<RoomExposureCache>(d.policy.cache);
                if(candidate->bind(*geometry)!=ExposureCacheStatus::pending){d.reserved-=d.charge;++d.stats.skipped;continue;}
                if(found==d.stores.end())found=d.stores.try_emplace(s.side_id).first;
                found->second.candidate=std::move(candidate);found->second.replacing=true;
            }
            auto&store=found->second;auto*cache=store.active();const auto transaction=cache->begin_transaction(stop);
            if(!transaction.active){if(store.replacing){store.candidate.reset();store.replacing=false;d.reserved-=d.charge;}
                if(!store.cache)d.stores.erase(found);++d.stats.skipped;continue;}
            store.transaction=transaction.token;store.touch=d.token;store.capture=std::move(capture);d.active.push_back(s.side_id);
            if(!cache->begin_capture(store.capture,stop,reset_sides.count(s.side_id)!=0)){cache->rollback_transaction(store.transaction);store.transaction=0;d.active.pop_back();
                if(store.replacing){store.candidate.reset();store.replacing=false;d.reserved-=d.charge;}
                if(!store.cache)d.stores.erase(found);++d.stats.skipped;continue;}
        }
        poll();return d.token;
    }catch(...){d.failed=true;d.close(false);return 0;}
}
void RoomExposureBatch::advance(std::uint64_t token,const EngineServices&services,const std::function<bool()>&stop){
    ExposureWorkQuota quota{impl_->policy.work_per_slice,impl_->policy.queries_per_slice,0,0};advance(token,services,quota,stop);
}
void RoomExposureBatch::advance(std::uint64_t token,const EngineServices&services,ExposureWorkQuota&quota,const std::function<bool()>&stop){
    auto&d=*impl_;std::lock_guard<std::mutex> lock(d.mutex);if(!token||token!=d.token||d.failed)return;
    const auto before_work=quota.work_used,before_queries=quota.queries_used;
    for(auto side:d.active){if(stop&&stop()){d.failed=true;break;}auto&s=d.stores.at(side);
        const auto result=s.active()->advance(services,quota,stop);
        if(result.cancelled||result.status==ExposureCacheStatus::transaction_exhausted){d.failed=true;break;}
        if(!quota.work_left)break;
    }
    d.stats.work+=quota.work_used-before_work;d.stats.queries+=quota.queries_used-before_queries;
}
bool RoomExposureBatch::view(std::uint64_t token,std::uint32_t side,RoomExposureCostView&out)const{
    auto&d=*impl_;std::lock_guard<std::mutex> lock(d.mutex);out={};if(!token||token!=d.token||d.failed)return false;
    auto found=d.stores.find(side);if(found==d.stores.end()||!found->second.transaction)return false;
    auto&s=found->second;out.cache=s.active();out.capture=s.capture;out.transaction_token=s.transaction;
    out.pricing_revision=out.observation_revision=out.cache->stats().observation_revision;out.prices=d.policy.prices;
    return out.cache->matches_capture(out.capture);
}
bool RoomExposureBatch::finish(std::uint64_t token,bool commit){auto&d=*impl_;std::lock_guard<std::mutex> lock(d.mutex);
    return token&&token==d.token?d.close(commit):false;}
void RoomExposureBatch::invalidate_side(std::uint32_t side){auto&d=*impl_;std::lock_guard<std::mutex> lock(d.mutex);
    if(d.token){const auto found=d.stores.find(side);if(found!=d.stores.end())found->second.retired=true;}
    else {if(d.stores.erase(side))d.reserved-=d.charge;}}
RoomExposureBatchStats RoomExposureBatch::stats()const{auto&d=*impl_;std::lock_guard<std::mutex> lock(d.mutex);auto out=d.stats;
    out.token=d.token;out.stores=d.stores.size();out.reserved_bytes=d.reserved;return out;}
}}}
