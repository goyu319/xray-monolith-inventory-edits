#pragma once
#include "PainterBurnState.h"
#include <deque>
#include <bitset>
#include <mutex>
#include <unordered_map>

namespace painter_burn {
struct UploadSpan { unsigned first{};std::vector<Texel> data; };
struct Delta {
    U64 lineage{},serial{},revision{};
    std::vector<UploadSpan> spans;
    std::vector<std::pair<OwnerKey,unsigned>> owners;
    std::bitset<MaxOwners+1> liveSpread,knownTag;
    bool retired{};
    unsigned tag(OwnerKey k)const{for(auto& p:owners)if(p.first==k)return p.second;return 0;}
};

// One instance for the native event/renderer boundary. Runtime owners register
// during their real lifetime. No owner pointer or draw object crosses threads.
class Sessions {
    struct Record {
        U64 serial{};
        bool queued{},retired{},announced{};
        std::vector<OwnerKey> owners;
        std::unique_ptr<Family> field;
    };
    std::mutex mutex;
    std::unordered_map<U64,Record> rows;
    std::deque<U64> dirty;
    U64 nextSerial=0;
    static bool has(const Record& r,OwnerKey o){return std::find(r.owners.begin(),r.owners.end(),o)!=r.owners.end();}
    void schedule(U64 id,Record& r){if(!r.queued){r.queued=true;dirty.push_back(id);}}
public:
    bool birth(U64 lineage,OwnerKey owner){
        if(!lineage||!owner.spawn)return false;
        std::lock_guard<std::mutex> lk(mutex);auto& r=rows[lineage];
        if(r.retired)return false;
        if(r.owners.empty())r.serial=++nextSerial;
        if(has(r,owner))return true;
        for(const auto& o:r.owners)if(o.spawn==owner.spawn)return false;
        if(r.owners.size()==MaxOwners||(r.field&&!r.field->birth(owner)))return false;
        r.owners.push_back(owner);if(r.field)schedule(lineage,r);return true;
    }
    Status apply(U64 lineage,OwnerKey owner,U64 sequence,U32 tick,const ContactCell* cells,std::size_t n){
        if(!n)return Status::NoChange;
        return commit(lineage,owner,[&](Family& f){return f.apply(owner,sequence,tick,cells,n);});
    }
    template<class Qualified> Status commit(U64 lineage,OwnerKey owner,Qualified&& qualified){
        std::lock_guard<std::mutex> lk(mutex);auto i=rows.find(lineage);
        if(i==rows.end()||!has(i->second,owner))return Status::StaleOwner;
        auto& r=i->second;
        bool initial=!r.field;
        if(initial){r.field=std::make_unique<Family>();for(auto o:r.owners)r.field->birth(o);}
        auto s=qualified(*r.field);
        if(s==Status::Applied)schedule(lineage,r);
        else if(initial&&!r.field->hasExposure())r.field.reset();
        return s;
    }
    Sample inspect(U64 lineage,OwnerKey owner,double x,double y,double z,double nx,double ny,double nz,unsigned domain,U32 token){
        std::lock_guard<std::mutex> lock(mutex);auto i=rows.find(lineage);
        if(i==rows.end()||!i->second.field)return {};
        return i->second.field->sample(owner,x,y,z,nx,ny,nz,domain,token);
    }
    bool rebind(U64 lineage,OwnerKey old,U64 newGeneration){
        std::lock_guard<std::mutex> lk(mutex);auto i=rows.find(lineage);
        if(i==rows.end()||newGeneration<=old.generation)return false;
        auto& r=i->second;auto j=std::find(r.owners.begin(),r.owners.end(),old);
        if(j==r.owners.end()||(r.field&&!r.field->rebind(old,newGeneration)))return false;
        j->generation=newGeneration;if(r.field)schedule(lineage,r);return true;
    }
    template<class Owns> Status split(U64 lineage,OwnerKey parent,const OwnerKey* children,std::size_t n,Owns&& owns){
        std::lock_guard<std::mutex> lk(mutex);auto i=rows.find(lineage);
        if(i==rows.end()||!has(i->second,parent))return Status::StaleOwner;
        auto& r=i->second;
        if(!n||!children||n>MaxOwners||r.owners.size()-1+n>MaxOwners)return Status::Capacity;
        auto list=r.owners;list.erase(std::find(list.begin(),list.end(),parent));
        for(std::size_t j=0;j<n;++j){
            if(!children[j].spawn||(children[j].spawn==parent.spawn&&children[j].generation<=parent.generation)||
               std::any_of(list.begin(),list.end(),[&](OwnerKey x){return x.spawn==children[j].spawn;}))return Status::BadContact;
            list.push_back(children[j]);
        }
        if(r.field){auto s=r.field->fork(parent,children,n,std::forward<Owns>(owns));if(s!=Status::Applied)return s;schedule(lineage,r);}
        r.owners.swap(list);return Status::Applied;
    }
    bool retire(U64 lineage,OwnerKey owner){
        std::lock_guard<std::mutex> lk(mutex);auto i=rows.find(lineage);if(i==rows.end())return false;
        auto& r=i->second;auto j=std::find(r.owners.begin(),r.owners.end(),owner);
        if(j==r.owners.end())return false;
        r.owners.erase(j);if(r.field)r.field->cancel(owner);
        if(r.owners.empty()){
            if(!r.announced&&!r.queued)rows.erase(i);
            else {r.field.reset();r.retired=true;schedule(lineage,r);}
        }else if(r.field)schedule(lineage,r);
        return true;
    }
    // Call on the upload-preparation worker. It visits marked families only.
    void resnapshot(){
        std::lock_guard<std::mutex> lk(mutex);for(auto& pair:rows)if(pair.second.field){
            pair.second.field->dirtyAll();schedule(pair.first,pair.second);
        }
    }
    // The render thread takes immutable deltas from the normal GPU submission queue.
    std::unique_ptr<Delta> tryPrepare(){
        std::unique_lock<std::mutex> lk(mutex,std::try_to_lock);
        if(!lk.owns_lock()||dirty.empty())return {};
        const U64 id=dirty.front();dirty.pop_front();auto it=rows.find(id);if(it==rows.end())return {};
        auto& r=it->second;r.queued=false;
        auto d=std::make_unique<Delta>();d->lineage=id;d->serial=r.serial;d->retired=r.retired;
        if(r.retired){rows.erase(it);return d;}
        d->revision=r.field->revision();
        for(const auto& o:r.field->activeOwners()){
            d->owners.emplace_back(o.key,o.tag);
            d->knownTag.set(o.tag);
            d->liveSpread.set(o.tag,r.field->hasLiveSpread(o));
        }
        r.field->publishDirty([&](unsigned start,const Texel* v,unsigned n){d->spans.push_back({start,std::vector<Texel>(v,v+n)});});
        r.announced=true;return d;
    }
};
}
