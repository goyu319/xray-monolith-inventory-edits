#pragma once
// Included after wm_slot's definition in WallmarksEngine.cpp.
#include "PainterStaticBloodAsync.h"
#include "PainterStaticBloodGpu.h"
#include "PainterBloodTexture.h"
#include <array>
#include <deque>
#include <map>
#include <set>

extern BOOL r_wallmarks_static;
extern int ps_blood_snapshots;
extern int ps_blood_snapshot_mb;

class PainterStaticBloodOwner {
    using W=CWallmarksEngine::static_wallmark;
    using Entry=CWallmarksEngine::static_entry;
    using Slot=WallmarksEngine::wm_slot;
    using Id=PainterStaticBlood::Id;
    using Capture=PainterStaticBlood::Capture;
    using Input=PainterStaticBlood::CapturePtr;
    using CPU=PainterStaticBlood::PatchPtr;
    using Key=std::pair<Slot*,IRender_Sector*>;
    using Gpu=painter_static_blood_gpu::Patch;
    struct Stored {
        Entry entry;
        Input input;
        Slot* source;
    };
    struct Visible {
        CPU cpu;
        std::shared_ptr<Gpu> gpu;
        std::vector<Stored> members;
        float expiry=0;
        Fsphere sphere;
        float smallestRadius=0;
    };
    struct Group {
        std::map<Id,Stored> pending;
        std::vector<Visible> visible;
        bool dirty=false,queued=false;
    };
    struct Upload {
        Key key{};
        PainterStaticBlood::AsyncResult result;
        std::unique_ptr<painter_static_blood_gpu::Pending> scratch;
        std::shared_ptr<Gpu> initial;
        unsigned imageRow=0,stampRow=0;
    };
    CWallmarksEngine& engine;
    std::map<Key,Group> groups;
    std::deque<Key> readyGroups;
    std::map<Slot*,Slot*> anchors;
    Slot* anchorFor(Slot* source){
        auto prev=anchors.find(source);
        if(prev!=anchors.end())return prev->second;
        Slot* anchor=source;const auto& ref=source->shader->E[0]->passes[0];
        auto i=std::find(engine.marks.begin(),engine.marks.end(),source);
        while(i!=engine.marks.begin()&&i!=engine.marks.end()){
            auto* neighbour=*--i;
            if(!painter_static_blood_gpu::compatible(neighbour->shader))break;
            const auto& other=neighbour->shader->E[0]->passes[0];
            if(ref->ps._get()!=other->ps._get() || ref->vs._get()!=other->vs._get() ||
               ref->state._get()!=other->state._get() || ref->constants._get()!=other->constants._get() ||
               ref->C._get()!=other->C._get())break;
            anchor=neighbour;
        }
        anchors.emplace(source,anchor);return anchor;
    }
    static size_t resident(const CPU& p){return painter_static_blood_gpu::bytes((unsigned)p->mesh.size(),p->width,p->height);}
    std::map<Id,Key> tracked;
    std::map<Slot*,unsigned> countBySlot;
    std::unique_ptr<PainterStaticBlood::Async> worker;
    bool busy=false;
    Key working{};
    std::unique_ptr<Upload> upload;
    size_t gpuBytes=0, inputBytes=0;
    unsigned lastFrame=~0u;
    size_t lastCapacity=0;
    static constexpr size_t inputLimit=32u<<20;
    static constexpr unsigned perFrame=256u<<10;
    static size_t captureBytes(const Input& in) {
        return sizeof(Capture)+in->mark.clipped.size()*sizeof(PainterStaticBlood::Vertex)
               +in->mark.receivers.size()*sizeof(PainterStaticBlood::Receiver);
    }
    static size_t cap(){return size_t((std::max)(1,ps_blood_snapshot_mb))<<20;}
    void queue(const Key& key,Group& group) {
        group.dirty=true;
        if(!group.queued){readyGroups.push_back(key);group.queued=true;}
    }
    void releaseInput(Id id,const Stored& member) {
        tracked.erase(id);
        inputBytes-=captureBytes(member.input);
    }
    void retire(const Key& key,Group& group,size_t index,bool restore) {
        auto& patch=group.visible[index];
        std::set<Slot*> sources;
        if(restore)for(const auto& m:patch.members)sources.insert(m.source);
        for(auto* s:sources){
            auto& originals=s->static_items[key.second];
            originals.reserve(originals.size()+patch.members.size());
        }
        for(auto& member:patch.members){
            Id id=member.input->mark.id;
            if(restore && member.entry.expire>=RDEVICE.fTimeGlobal){
                member.source->static_items[key.second].push_back(member.entry);
                group.pending.emplace(id,member);
            }else{
                releaseInput(id,member);
                engine.static_wm_destroy(member.entry.W);
            }
        }
        gpuBytes-=resident(patch.cpu);
        if(!--countBySlot[key.first])countBySlot.erase(key.first);
        group.visible.erase(group.visible.begin()+index);
        if(restore){
            for(auto* s:sources){
                auto& originals=s->static_items[key.second];
                std::stable_sort(originals.begin(),originals.end(),[](const Entry& a,const Entry& b){return a.W->snapshotId<b.W->snapshotId;});
            }
            queue(key,group);
        }
    }
    static void measure(Visible& patch) {
        Fbox box;box.invalidate();patch.smallestRadius=FLT_MAX;patch.expiry=0;
        for(const auto& m:patch.members){
            const auto& e=m.entry;Fvector a=e.P,b=e.P;a.sub(e.R);b.add(e.R);box.modify(a);box.modify(b);
            patch.smallestRadius=(std::min)(patch.smallestRadius,e.R);patch.expiry=(std::max)(patch.expiry,e.expire);
        }
        box.getsphere(patch.sphere.P,patch.sphere.R);
    }
    void prune(){
        for(auto it=groups.begin();it!=groups.end();){
            Group& g=it->second;
            for(size_t i=g.visible.size();i-->0;){
                const auto& patch=g.visible[i];
                if(patch.expiry<RDEVICE.fTimeGlobal)retire(it->first,g,i,false);
                else if(!painter_static_blood_gpu::ready(*patch.gpu) || gpuBytes>cap())retire(it->first,g,i,true);
            }
            if(g.pending.empty()&&g.visible.empty()&&!g.queued)it=groups.erase(it);else ++it;
        }
    }
    void schedule(){
        if(busy||upload)return;
        while(!readyGroups.empty()){
            const Key key=readyGroups.front();readyGroups.pop_front();
            auto it=groups.find(key);if(it==groups.end())continue;
            Group& g=it->second;g.queued=false;
            if(!g.dirty)continue;
            g.dirty=false;
            PainterStaticBlood::AsyncJob job;
            job.config.minimum=32; // First measured group size with a GPU win at close range.
            if(g.pending.empty()||(g.visible.empty()&&g.pending.size()<job.config.minimum))continue;
            job.allowNew=gpuBytes<cap()&&g.visible.size()<PainterStaticBlood::Async::MaxPatches;job.config.byteCap=job.allowNew?cap()-gpuBytes:0;
            if(!job.allowNew&&g.visible.empty())continue;
            if(!worker)worker=std::make_unique<PainterStaticBlood::Async>();
            job.epoch=worker->epoch();job.nowMs=RDEVICE.dwTimeGlobal;
            for(const auto& m:g.pending){
                if(m.second.entry.expire>=RDEVICE.fTimeGlobal)job.marks.push_back(m.second.input);
                if(job.marks.size()>=512)break;
            }
            for(auto& patch:g.visible)job.patches.push_back(patch.cpu);
            if(worker->trySubmit(job)){busy=true;working=key;}else queue(key,g);
            break;
        }
    }
    void publish(){
        if(!upload)return;
        auto& u=*upload;
        auto it=groups.find(u.key);
        if(it==groups.end()||u.result.epoch!=worker->epoch()){upload.reset();return;}
        auto& group=it->second;auto& r=u.result;
        auto base=std::find_if(group.visible.begin(),group.visible.end(),[&](const Visible& v){return v.cpu==r.base;});
        if(r.base&&base==group.visible.end()){upload.reset();return;}
        std::set<Slot*> sources;
        // Serial ids disambiguate pooled wallmark pointers reused between submission and publication.
        std::set<Id> ids(r.members.begin(),r.members.end());
        if(ids.size()!=r.members.size()||ids.empty()){upload.reset();return;}
        std::vector<Stored> moved;
        moved.reserve(ids.size());
        for(auto id:ids){
            auto q=group.pending.find(id);
            if(q==group.pending.end() || q->second.entry.expire<RDEVICE.fTimeGlobal){upload.reset();return;}
            moved.push_back(q->second);sources.insert(q->second.source);
        }
        unsigned matching=0;
        for(auto* s:sources){
            auto native=s->static_items.find(u.key.second);if(native==s->static_items.end()){upload.reset();return;}
            for(const auto& e:native->second)matching+=ids.count(e.W->snapshotId)!=0;
        }
        if(matching!=ids.size()){upload.reset();return;}
        // Finish all fallible CPU allocations before publishing pixels or removing any member.
        Visible newPatch;
        if(!r.base){
            const auto& p=*r.draft;
            if(resident(r.draft)+gpuBytes>cap()){upload.reset();return;}
            if(!u.initial){
                u.initial=painter_static_blood_gpu::create_blank(u.key.first->shader,p.mesh.data(),(unsigned)p.mesh.size(),p.width,p.height);
                if(!u.initial)upload.reset();
                return;
            }
            if(!painter_static_blood_gpu::ready(*u.initial)){upload.reset();return;}
            unsigned budget=perFrame;
            if(!painter_static_blood_gpu::pump_initial(*u.initial,u.imageRow,u.stampRow,
                    p.rgba.data(),p.owner.data(),p.stamps.data(),(unsigned)p.stamps.size(),budget))return;
            newPatch.cpu=r.draft;
            newPatch.members=moved;
            measure(newPatch);
            group.visible.reserve(group.visible.size()+1);
            countBySlot.try_emplace(u.key.first,0);
            newPatch.gpu=std::move(u.initial);
            gpuBytes+=resident(r.draft);group.visible.push_back(std::move(newPatch));++countBySlot[u.key.first];
        }else{
            const auto& delta=r.dirty;const auto& rect=delta.rect;
            if(gpuBytes+painter_static_blood_gpu::dirty_bytes(rect.w,rect.h,1)>cap()){upload.reset();return;}
            if(!u.scratch){
                u.scratch=painter_static_blood_gpu::begin_update(*base->gpu,rect.x,rect.y,rect.w,rect.h,delta.stampIndex,1);
                if(!u.scratch){upload.reset();return;}
            }
            base->members.reserve(base->members.size()+moved.size());
            unsigned budget=perFrame;
            if(!painter_static_blood_gpu::pump(*u.scratch,delta.rgba.data(),rect.w*4,delta.owner.data(),rect.w*2,&delta.stamp,budget))return;
            if(!painter_static_blood_gpu::commit(*base->gpu,*u.scratch)){upload.reset();return;}
            base->members.insert(base->members.end(),moved.begin(),moved.end());
            base->cpu=r.draft;measure(*base);
        }
        for(auto* s:sources){
            auto& original=s->static_items.find(u.key.second)->second;
            original.erase(std::remove_if(original.begin(),original.end(),[&](const Entry& e){return ids.count(e.W->snapshotId)!=0;}),original.end());
        }
        for(auto id:ids)group.pending.erase(id);
        if(!group.pending.empty())queue(u.key,group);
        upload.reset();
    }
public:
    explicit PainterStaticBloodOwner(CWallmarksEngine& owner):engine(owner){}
    void add(Slot* slot,IRender_Sector* sector,const Entry& e){
        auto* w=e.W;
        if(!ps_blood_snapshots||!w->m_thermal_blood||w->snapshotReceivers.empty()||w->snapshotOverflow||w->verts.size()>768)return;
        if(tracked.size()>=4096 || inputBytes+sizeof(Capture)+w->verts.size()*24+w->snapshotReceivers.size()*36>inputLimit)return;
        if(!anchors.count(slot) && !painter_static_blood_gpu::compatible(slot->shader))return;
        const Key key{anchorFor(slot),sector};
        auto source=PainterBloodTexture::selected(slot->shader);
        if(!source||!source->supported)return;
        auto input=std::make_shared<Capture>();auto& m=input->mark;
        input->source=std::move(source);m.id=w->snapshotId;m.sector=reinterpret_cast<uintptr_t>(sector);m.material=reinterpret_cast<uintptr_t>(key.first);
        m.stamp.landedMs=RDEVICE.dwTimeGlobal;
        if(w->TimeEnd()!=-1.f)m.stamp.lifetimeMs=(u32)(std::max)(1.,(std::min)(double(~0u-1),1000.*w->TimeEnd()));
        const auto& h=w->m_thermal_stamp;m.stamp.birthMs=h.blood_birth;m.stamp.deathMs=h.source_death;
        m.stamp.flags=((!h.effect_id||BloodEffect::IsSurface(h.effect_id))?1u:0u)|(h.birth_known?2u:0u)|((h.source_known&&h.source_corpse)?4u:0u);
        m.clipped.reserve(w->verts.size());for(const auto& v:w->verts)m.clipped.push_back({{v.p.x,v.p.y,v.p.z},0xffffffff,v.t.x,v.t.y});
        m.receivers.reserve(w->snapshotReceivers.size());
        for(const auto& tri:w->snapshotReceivers){
            PainterStaticBlood::Receiver r;
            for(int j=0;j<3;j++)r.p[j]={tri.p[j].x,tri.p[j].y,tri.p[j].z};
            m.receivers.push_back(r);
        }
        auto& group=groups[key];group.pending.emplace(m.id,Stored{e,input,slot});
        tracked.emplace(m.id,key);inputBytes+=captureBytes(input);queue(key,group);
    }
    void forget(Id id){
        auto g=tracked.find(id);if(g==tracked.end())return;
        auto found=groups.find(g->second);
        if(found!=groups.end()){
            auto it=found->second.pending.find(id);
            if(it!=found->second.pending.end()){inputBytes-=captureBytes(it->second.input);found->second.pending.erase(it);if(!found->second.pending.empty())queue(found->first,found->second);}
        }
        tracked.erase(g);
    }
    void beforeReplace(Slot* slot,IRender_Sector* sector,const Fvector& p){
        auto a=anchors.find(slot);if(a==anchors.end())return;
        auto it=groups.find({a->second,sector});if(it==groups.end())return;
        for(size_t j=it->second.visible.size();j-->0;){
            auto& patch=it->second.visible[j];
            if(p.distance_to_sqr(patch.sphere.P)>_sqr(patch.sphere.R+.04f))continue;
            for(const auto& m:patch.members)if(m.source==slot&&m.entry.P.similar(p,.02f)){retire(it->first,it->second,j,true);break;}
        }
    }
    void tick(){
        if(lastFrame==Device.dwFrame)return;lastFrame=Device.dwFrame;
        if(!ps_blood_snapshots || !r_wallmarks_static){
            if(busy||upload||!countBySlot.empty())reset(false);
            return;
        }
        if(lastCapacity!=cap()){lastCapacity=cap();for(auto& kv:groups)if(!kv.second.pending.empty())queue(kv.first,kv.second);}
        if(worker && busy){
            PainterStaticBlood::AsyncResult next;
            if(worker->tryPop(next)){
                busy=false;
                if(next.draft&&next.epoch==worker->epoch())upload=std::make_unique<Upload>(Upload{working,std::move(next),{}});
            }
        }
        prune();publish();schedule();
    }
    void reset(bool discard){
        if(worker)worker->reset();upload.reset();busy=false;
        for(auto& kv:groups)
            for(size_t j=kv.second.visible.size();j-->0;)retire(kv.first,kv.second,j,!discard);
        countBySlot.clear();
        if(discard){groups.clear();readyGroups.clear();tracked.clear();anchors.clear();inputBytes=0;}
        else for(auto& kv:groups)if(!kv.second.pending.empty())queue(kv.first,kv.second);
        lastFrame=~0u;
    }
    bool has(Slot* slot)const{auto it=countBySlot.find(slot);return it!=countBySlot.end()&&it->second;}
    template<class Stream>void draw(Slot* slot,float ssa,Stream submitOriginal){
        if(!has(slot))return;
        auto it=groups.lower_bound({slot,nullptr});
        for(;it!=groups.end()&&it->first.first==slot;++it){
            if(!RImplementation.GMBase.is_sector_visible(static_cast<CSector*>(it->first.second)))continue;
            for(auto& patch:it->second.visible){
                if(!RImplementation.ViewBase.testSphere_dirty(patch.sphere.P,patch.sphere.R))continue;
                const float farDistance=Device.vCameraPosition.distance_to(patch.sphere.P)+patch.sphere.R;
                if(painter_static_blood_gpu::ready(*patch.gpu) && patch.smallestRadius*patch.smallestRadius>=ssa*farDistance*farDistance)
                    painter_static_blood_gpu::draw(*patch.gpu);
                else for(auto& m:patch.members)if(m.entry.expire>=RDEVICE.fTimeGlobal&&RImplementation.ViewBase.testSphere_dirty(m.entry.P,m.entry.R)&&m.entry.R*m.entry.R>=ssa*Device.vCameraPosition.distance_to_sqr(m.entry.P))submitOriginal(m.source,m.entry.W);
            }
        }
    }
};
