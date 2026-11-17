#pragma once
#ifdef USE_DX11
#include "PainterBloodGpu.h"
#include "PainterBloodTexture.h"
#include "PainterBloodProjection.h"
#include "PainterBloodOwner.h"
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
struct PainterBloodShared;
// bloodfix 2026-11-13 (xrRender_console.cpp): r__blood_bake_memory, MiB: all bodies' blood textures together;
// r__blood_bake_body_memory, MiB: one body's (and its pieces') blood texture. Read when the bake worker starts (the first
// Painter model of the session); a change applies after a restart.
extern int ps_r_blood_bake_memory;
extern int ps_r_blood_bake_body_memory;
namespace PainterBloodBake {
struct Owner {
    std::unique_ptr<BloodBakeAsync::Service> service;
    std::vector<std::weak_ptr<PainterBloodShared>> bodies;
    std::deque<std::unique_ptr<BloodBakeAsync::Delta>> retired;
    PainterBloodUpload::FrameBudget budget;
    unsigned frame=~0u;std::size_t cursor=0;
    // bloodfix: one report line at most every 10 s (the session logged a line per over-budget frame and per held body)
    unsigned frames=0,overruns=0,textureRefused=0,queueDropped=0,resyncs=0;double overrunMaxUs=0;u32 reportedMs=0;
    std::uint64_t reportedPressure=0;xr_string textureWhy;
};
inline Owner& owner(){static auto* value=new Owner;return *value;}
// The first setting bounds the native textures (every body's table, plus the rows in flight to them); the bodies' tables
// themselves get three quarters of it, deltas waiting for the owner a quarter. The second is one body's table.
inline BloodBakeAsync::Limits limits(){
    const std::size_t mib=1024u*1024u;
    const std::size_t bytes=std::size_t((std::max)(32,(std::min)(2048,ps_r_blood_bake_memory)))*mib;
    const std::size_t body=std::size_t((std::max)(1,(std::min)(64,ps_r_blood_bake_body_memory)))*mib;
    PainterBloodUpload::nativePayloadLimit().store(bytes,std::memory_order_relaxed);
    BloodBakeAsync::Limits value;value.maxLayerBytes=bytes/4*3;value.maxBodyLayerBytes=(std::min)(body,value.maxLayerBytes);
    value.maxCompletedBytes=(std::max)(std::size_t(16u*mib),bytes/4);return value;
}
inline BloodBakeAsync::Service& service(){auto& o=owner();if(!o.service)o.service.reset(new BloodBakeAsync::Service(limits(),[]{SetThreadPriority(GetCurrentThread(),THREAD_PRIORITY_BELOW_NORMAL);}));return *o.service;}
// What the bake did, when something is worth a line: frames over the 0.5 ms budget, or the worker dropped / cut / evicted /
// waited. expect_us = the cost the budget currently expects of one row copy / a packet's first copy / binding a new table /
// a material step / an admission. evicted = whole other bodies + own layers dropped for new stains; cells_evicted = cells of
// a body's own oldest stains dropped at its cap; outside = cells beyond the 2 x 4 x 4 m window. Nothing in a calm session.
inline void report(Owner& o){
    const u32 now=Device.dwTimeGlobal;
    if(o.reportedMs&&u32(now-o.reportedMs)<10000u)return;
    o.reportedMs=now?now:1u;
    if(!o.service){o.frames=o.overruns=0;o.overrunMaxUs=0;return;}
    const auto& a=o.service->accounting();const auto load=[](const std::atomic<std::uint64_t>& v){return v.load(std::memory_order_relaxed);};
    const std::uint64_t pressure=load(a.dropped)+load(a.truncated)+load(a.evictedOthers)+load(a.evictedSelf)+load(a.waits)+load(a.cellsEvicted)+load(a.cellsOutside)+o.textureRefused+o.queueDropped+o.resyncs;
    if(o.overruns||pressure!=o.reportedPressure){
        const auto& c=PainterBloodUpload::FrameBudget::costs();const double mb=1./(1024.*1024.);
        Msg("[blood_bake] report frames=%u over_budget=%u max_us=%.0f expect_us=%.0f/%.0f/%.0f/%.0f/%.0f baked=%llu published=%llu(%llu whole) dropped=%llu cut=%llu evicted=%llu+%llu cells_evicted=%llu outside=%llu waits=%llu texture_refused=%u%s%s queue_dropped=%u resync=%u layer_mb=%.1f peak=%.1f native_mb=%.1f peak=%.1f limit=%.0f",
            o.frames,o.overruns,o.overrunMaxUs,c.expect[0]*1e6f,c.expect[1]*1e6f,c.expect[2]*1e6f,c.expect[3]*1e6f,c.expect[4]*1e6f,
            load(a.baked),load(a.published),load(a.fullPublications),load(a.dropped),load(a.truncated),load(a.evictedOthers),load(a.evictedSelf),load(a.cellsEvicted),load(a.cellsOutside),load(a.waits),
            o.textureRefused,o.textureWhy.empty()?"":" why=",o.textureWhy.c_str(),o.queueDropped,o.resyncs,
            a.layerBytes.load(std::memory_order_relaxed)*mb,a.peakLayerBytes.load(std::memory_order_relaxed)*mb,
            PainterBloodUpload::nativePayload().load(std::memory_order_relaxed)*mb,PainterBloodUpload::nativePayloadPeak().load(std::memory_order_relaxed)*mb,
            PainterBloodUpload::nativePayloadLimit().load(std::memory_order_relaxed)*mb);
    }
    o.reportedPressure=pressure;o.frames=o.overruns=0;o.overrunMaxUs=0;
}
void retireDelta(const BloodBakeAsync::Delta* packet){owner().retired.emplace_back(const_cast<BloodBakeAsync::Delta*>(packet));}
}
struct PainterBloodPending {
    std::shared_ptr<PainterBloodProjection::Input> input;
    ref_shader shader;
    std::unique_ptr<BloodBakeAsync::Job> job;
    BloodCellLayer::Stamp stamp;
    std::uint64_t markNamespace=0,markKey=0,markRevision=0;
    bool held=false;
};
struct PainterBloodNativeTarget {
    std::mutex mutex;std::shared_ptr<PainterBloodUpload::Preparer> value;
};
struct PainterBloodShared {
    PainterBloodBake::Gpu gpu;
    std::shared_ptr<BloodBakeAsync::State> state;
    std::deque<PainterBloodPending> pending;
    std::uint64_t reportedFailures=0,reportedSequence=0;
    bool waitingSnapshot=false;
    bool clearOwed=false; // bloodfix: a `cleared` delta whose release did not fit its frame
    std::shared_ptr<PainterBloodNativeTarget> target;
    PainterBloodShared():gpu(HW.pDevice){target=std::make_shared<PainterBloodNativeTarget>();target->value=gpu.preparer;const auto holder=target;
        state=PainterBloodBake::service().makeState({},[holder](BloodBakeAsync::Delta& d){std::shared_ptr<PainterBloodUpload::Preparer> native;{std::lock_guard<std::mutex> lock(holder->mutex);native=holder->value;}native->prepare(d);});}
    ~PainterBloodShared(){state->cancel();}
};
struct PainterBloodInstance {
    std::shared_ptr<PainterBloodShared> shared;
    struct Material {ref_shader original,applied;};
    std::unordered_map<const Shader*,Material> materials;
    std::unordered_map<const Shader*,ref_shader> rejectedMaterials;
    std::unordered_map<const Shader*,PainterBloodBake::CloneProgress> cloning; // bloodfix: material clones in steps
    unsigned refreshed=~0u;
    bool priorEnabled=false;
    PainterBloodInstance():shared(std::make_shared<PainterBloodShared>()){PainterBloodBake::owner().bodies.push_back(shared);}
    explicit PainterBloodInstance(std::shared_ptr<PainterBloodShared> s):shared(std::move(s)){}
};

bool CKinematics::QueueBloodCells(intrusive_ptr<CSkeletonWallmark> wm)
{
    try {
        if(!wm||(!wm->m_Projection&&wm->m_Faces.empty())){Msg("! [blood_bake] rejected=receiver_projection");return false;}
        if(!m_blood_cells)m_blood_cells=std::make_shared<PainterBloodInstance>();
        auto& state=*m_blood_cells->shared;
        // bloodfix: a full admission queue drops its OLDEST waiting stain, never the new one (counted in the report line)
        if(state.pending.size()>=1024){state.pending.pop_front();++PainterBloodBake::owner().queueDropped;}
        // Admission is already under AddWallmark's owner lock. Capture immutable
        // handles only; hit projection, texture decode and raster all run on worker.
        PainterBloodPending pending;pending.input=PainterBloodProjection::take(*wm);pending.shader=wm->Shader();
        auto& stamp=pending.stamp;stamp.landed=wm->TimeStart();stamp.lifetime=wm->TimeEnd();
        stamp.color_intensity=1.f;stamp.opacity=wm->m_Growth.opacity; // live SSS controls belong only to shared shading
        const auto& thermal=wm->m_thermal_stamp;
        pending.markNamespace=thermal.mark_namespace;pending.markKey=thermal.mark_key;pending.markRevision=thermal.mark_revision;
        stamp.heat_enabled=BloodEffect::IsSurface(thermal.effect_id);
        stamp.heat_birth=thermal.birth_known?thermal.blood_birth*.001f:stamp.landed;
        if(thermal.source_known&&thermal.source_corpse)stamp.heat_birth=(std::min)(stamp.heat_birth,thermal.source_death*.001f);
        state.pending.push_back(std::move(pending));m_blood_cells->refreshed=~0u;return true;
    } catch(const std::exception& e){Msg("! [blood_bake] rejected=queue why=%s",e.what());return false;}
}
void CKinematics::RestoreBloodMaterials()
{
    if(!m_blood_cells)return;
    auto& instance=*m_blood_cells;
    // Publication and model duplication consume the original material family,
    // never an instance's dynamic blood resource bindings. The three runtime
    // admission paths all reach this under the worker-joined owner barrier.
    // bloodfix (second session): `used` = the original materials this model's children hold now. Entries of materials no
    // child holds any more are dropped here (every publication made new materials and the map kept every old one, and
    // with it the old publications' mask textures, for the body's whole life).
    std::unordered_set<const Shader*> used;
    auto restore=[&](const auto& list){for(auto* v:list){
        auto* child=fast_dynamic_cast<dxRender_Visual*>(v);if(!child||!child->shader)continue;
        const auto it=instance.materials.find(child->shader._get());
        if(it!=instance.materials.end())child->shader=it->second.original;
        used.insert(child->shader._get());
    }};
    restore(children);restore(children_invisible);
    restore(m_geometry_original_children);restore(m_geometry_original_invisible);
    for(auto it=instance.materials.begin();it!=instance.materials.end();){if(used.count(it->second.original._get()))++it;else it=instance.materials.erase(it);}
    for(auto it=instance.rejectedMaterials.begin();it!=instance.rejectedMaterials.end();){if(used.count(it->first))++it;else it=instance.rejectedMaterials.erase(it);}
    for(auto it=instance.cloning.begin();it!=instance.cloning.end();){if(used.count(it->first))++it;else it=instance.cloning.erase(it);}
    instance.refreshed=~0u;
}
void CKinematics::ClearBloodCells()
{
    m_blood_mark_namespace=0;
    if(!m_blood_cells)return;
    auto restore=[&](const auto& list){for(auto* v:list){auto* child=fast_dynamic_cast<dxRender_Visual*>(v);if(!child)continue;
        auto it=m_blood_cells->materials.find(child->shader._get());if(it!=m_blood_cells->materials.end())child->shader=it->second.original;}};
    restore(children);restore(children_invisible);restore(m_geometry_original_children);restore(m_geometry_original_invisible);
    m_blood_cells.reset();
}
void PainterBloodWarmPrograms(const ref_shader& source){
    // Model-load boundary: start the worker before any blood impact and prepare
    // shader programs without adding cold compilation to a later draw.
    PainterBloodPrograms::note(source);PainterBloodBake::service(); // shaderslim: this model's recipes (names only; created by the pump)
}
void PainterBloodPrewarmAll(){PainterBloodPrograms::pump(true);}
void PainterBloodNotePrograms(const ref_shader& source){PainterBloodPrograms::note(source);}
void PainterBloodRequestPrograms(const char* vs,int skinning,const char* ps,int sample){PainterBloodPrograms::requestPair(vs?vs:"",skinning,ps?ps:"",sample);}
void PainterBloodPumpPrograms(bool all){PainterBloodPrograms::pump(all);}
int PainterMaskOnlyMode(){int lo=0,hi=0;return Console?Console->GetInteger("painter_voxel_mask_only",lo,hi):0;}
void PainterBloodClearPrograms(){PainterBloodPrograms::clear();}
void PainterBloodFramePump()
{
    using Budget=PainterBloodUpload::FrameBudget;
    auto& owner=PainterBloodBake::owner();
    if(owner.frame==Device.dwFrame)return;
    // bloodfix: frames over the budget are counted and reported in one line (PainterBloodBake::report), not one line each
    if(owner.frame!=~0u){++owner.frames;const double us=owner.budget.seconds()*1e6;if(us>500.){++owner.overruns;owner.overrunMaxUs=(std::max)(owner.overrunMaxUs,us);}}
    PainterBloodBake::report(owner);
    owner.frame=Device.dwFrame;owner.budget=Budget();
    Budget::Segment segment(owner.budget);
    xrCriticalSectionTryGuard guard(wallmarks_cs);if(!guard.owns_lock())return;
    if(!owner.service||!HW.pDevice||!HW.pContext)return;
    owner.service->ownerFrame();
    while(!owner.retired.empty()&&owner.budget.allow()){
        if(!owner.service->tryRetire(owner.retired.front()))break;owner.retired.pop_front();}
    const auto count=owner.bodies.size();
    for(std::size_t visited=0;visited<count&&owner.budget.allow();++visited){
        if(owner.cursor>=owner.bodies.size())owner.cursor=0;
        auto shared=owner.bodies[owner.cursor++].lock();if(!shared)continue;
        auto& body=*shared;
        if(body.gpu.device!=HW.pDevice){
            std::unique_lock<std::mutex> targetLock(body.target->mutex,std::try_to_lock);if(!targetLock.owns_lock())continue;
            body.gpu.upload=PainterBloodUpload::Upload();body.gpu.device=HW.pDevice;
            body.gpu.preparer=std::make_shared<PainterBloodUpload::Preparer>(HW.pDevice);
            body.target->value=body.gpu.preparer;body.waitingSnapshot=true;body.clearOwed=false;body.state->requestSnapshot();
        }
        // bloodfix: up to eight waiting stains of this body go to the worker per frame, each one measured operation (one
        // per frame made the worker see a burst of sixty stains a second late). A stain whose texture cannot be read, or
        // that the worker calls invalid, is dropped and counted: it held every later stain of its body before.
        for(unsigned sent=0;sent<8&&!body.pending.empty()&&owner.budget.allow(Budget::Admit);++sent){
            auto& p=body.pending.front();
            Budget::Measure measure(owner.budget,Budget::Admit);owner.budget.operation();
            if(!p.job&&!p.held){
                try{const auto texture=PainterBloodTexture::selected(p.shader);
                    if(texture){auto job=std::make_unique<BloodBakeAsync::Job>();job->stamp=p.stamp;
                        job->markNamespace=p.markNamespace;job->markKey=p.markKey;job->markRevision=p.markRevision;
                        // Pieces share this lineage. A growing cut retains old-only coverage on detached geometry.
                        job->markPreserveOldCoverage=job->markNamespace && job->markKey && job->markRevision;
                        const auto input=p.input;
                        job->capturedBytes=sizeof(PainterBloodProjection::Input)+input->faces.capacity()*sizeof(CSkeletonWallmark::WMFace);
                        if(input->projection){const auto& hit=*input->projection;
                            job->capturedBytes+=sizeof(hit)+hit.hit_pose.capacity()*sizeof(Fmatrix)+hit.ranges.capacity()*sizeof(CSkeletonWallmark::ProjectionSnapshot::Range);
                            for(const auto& range:hit.ranges)job->capturedBytes+=std::size_t(range.vertex_count)*2*sizeof(BloodCellLayer::Vertex);
                        }else job->capturedBytes+=input->faces.size()*6*sizeof(BloodCellLayer::Vertex);
                        job->prepare=[input,texture](BloodBakeAsync::Job& work){
                            const auto decoded=PainterBloodTexture::prepare(texture);
                            work.faces=PainterBloodProjection::resolve(*input);
                            work.sample=[decoded](float u,float v){const auto t=decoded->image.sample(u,v);return BloodCellLayer::TextureSample{t[0],t[1],t[2],t[3]};};
                        };
                        p.job=std::move(job);
                    }
                }catch(const std::exception& e){p.held=true;owner.textureWhy=e.what();}
            }
            if(p.held){body.pending.pop_front();++owner.textureRefused;continue;}
            if(!p.job){
                // its texture is still loading: the body's stains wait, in order - five seconds at most, then this stain
                // is dropped (a texture that never arrives held the body's blood for the session before)
                if(Device.fTimeGlobal-p.stamp.landed>5.f){body.pending.pop_front();++owner.textureRefused;owner.textureWhy="blood_texture_still_loading";continue;}
                break;
            }
            const auto answer=owner.service->trySubmit(body.state,p.job);
            if(answer==BloodBakeAsync::SubmitResult::Invalid){body.pending.pop_front();++owner.textureRefused;owner.textureWhy="blood_job_invalid";continue;}
            if(answer!=BloodBakeAsync::SubmitResult::Accepted)break; // the worker's queue is full: next frame
            body.pending.pop_front();
        }
        if(body.gpu.upload.idle()&&!body.gpu.heldPacket&&owner.budget.allow()){
            if(body.clearOwed){if(body.gpu.clear())body.clearOwed=false;}
            else if(auto done=body.state->tryTakeCompleted()){
                const auto native=std::static_pointer_cast<PainterBloodUpload::Prepared>(done->prepared);
                if(done->cleared||(native&&native->cleared)){
                    // bloodfix: the worker dropped this body's layer for newer blood (memory pressure), or a snapshot of
                    // a body without blood. Its table is released and it draws no blood until its next delta, which is a
                    // whole new table.
                    body.waitingSnapshot=false;if(!body.gpu.clear())body.clearOwed=true;
                    if(!owner.service->tryRetire(done))owner.retired.push_back(std::move(done));
                }else{
                    if(!native||native->deviceIdentity!=HW.pDevice||(body.waitingSnapshot&&!done->fullSnapshot)||done->sequence<body.gpu.upload.sequence()){
                        if(!owner.service->tryRetire(done))owner.retired.push_back(std::move(done));
                    }else{
                        body.waitingSnapshot=false;
                        // bloodfix: a packet the uploader cannot start was held forever (and the body with it); now the
                        // packet goes and the body asks the worker for a full snapshot
                        if(!body.gpu.begin(done)){body.gpu.heldPacket.reset();body.waitingSnapshot=true;body.state->requestSnapshot();++owner.resyncs;}
                    }
                }
            }
        }
        if(owner.budget.allow())body.gpu.pump(owner.budget);
        body.reportedFailures=body.state->failureCount(); // reported by the one line of PainterBloodBake::report
    }
    // Expired registrations do not retain destroyed NPCs. Bounded erasure and
    // projection retirement are included in this same measured owner segment.
    if(owner.budget.allow()&&!owner.bodies.empty()){
        const auto it=std::find_if(owner.bodies.begin(),owner.bodies.end(),[](const auto& p){return p.expired();});
        if(it!=owner.bodies.end())owner.bodies.erase(it);
    }
    if(owner.budget.allow())PainterBloodProjection::drainOne();
}
void PainterBloodShutdown()
{
    auto& owner=PainterBloodBake::owner();
    if(owner.service){owner.service->shutdownAndJoin();owner.service.reset();}
    owner.retired.clear();owner.bodies.clear();owner.cursor=0;owner.frame=~0u;
    PainterBloodBake::emptyTable(nullptr); // bloodfix: the cleared bodies' empty table goes before the device
    PainterBloodProjection::drain();PainterBloodTexture::clear();PainterBloodPrograms::clear();
}
void PainterBloodFinalOwnerDrain(){auto& owner=PainterBloodBake::owner();VERIFY(!owner.service);owner.retired.clear();PainterBloodProjection::drain();}
void CKinematics::RefreshBloodCells()
{
    xrCriticalSectionTryGuard bloodGuard(wallmarks_cs);if(!bloodGuard.owns_lock()||!m_blood_cells)return;
    auto& instance=*m_blood_cells;const bool active=PainterBloodBake::enabled()&&!instance.shared->waitingSnapshot;
    if(instance.refreshed==Device.dwFrame&&instance.priorEnabled==active)return;
    instance.refreshed=Device.dwFrame;instance.priorEnabled=active;
    auto& state=*instance.shared;auto& budget=PainterBloodBake::owner().budget;
    if(!state.gpu.upload.hasActive()&&!state.waitingSnapshot)return;
    auto update=[&](const auto& list){for(auto* v:list){
        auto* skin=fast_dynamic_cast<CSkeletonX*>(v);auto* child=fast_dynamic_cast<dxRender_Visual*>(v);
        if(!skin||!skin->receives_wallmarks||!child||!child->shader)continue;
        auto found=instance.materials.find(child->shader._get());
        if(!active){if(found!=instance.materials.end())child->shader=found->second.original;continue;}
        if(found!=instance.materials.end()){if(found->second.applied._get()!=child->shader._get())child->shader=found->second.applied;continue;}
        const Shader* key=child->shader._get();
        if(instance.rejectedMaterials.count(key))continue;
        if(PainterBloodBake::bound(child->shader,state.gpu)){PainterBloodInstance::Material same{child->shader,child->shader};instance.materials.emplace(key,same);continue;}
        auto& progress=instance.cloning[key];progress.seen=Device.dwFrame;
        // one shader element per step, each a measured operation the budget admits by its recent cost
        if(!budget.allow(PainterBloodUpload::FrameBudget::Material))continue;
        PainterBloodUpload::FrameBudget::Segment segment(budget);
        ref_shader made;xr_string why;bool cloned;
        {PainterBloodUpload::FrameBudget::Measure measure(budget,PainterBloodUpload::FrameBudget::Material);
         cloned=PainterBloodBake::clone(child->shader,state.gpu,made,why,&progress);}
        budget.operation();
        if(!cloned){
            if(why!="blood_material_step"&&why!="blood_programs_not_warmed"&&why!="blood_material_busy"){
                const ref_shader keep=child->shader;instance.cloning.erase(key);instance.rejectedMaterials.emplace(key,keep);
                if(why!="blood_alias_missing")Msg("! [blood_bake] rejected=material why=%s wallmark_fallback=0",why.c_str());}
            continue;
        }
        instance.cloning.erase(key);
        PainterBloodInstance::Material material{child->shader,made};instance.materials.emplace(key,material);instance.materials.emplace(made._get(),material);
        child->shader=made; // whole: the child takes it now
    }};
    update(children);update(children_invisible);
    // steps of materials no child holds any more (a publication came first) are dropped
    for(auto it=instance.cloning.begin();it!=instance.cloning.end();){if(it->second.seen==Device.dwFrame)++it;else it=instance.cloning.erase(it);}
}
#endif
