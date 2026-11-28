#include <chrono>
// Compiled in SkeletonCustom.cpp. The CPU barrier is set only while the engine
// drains its joined-worker queue. It cannot be supplied or replayed by callers.
#include "FSkinned.h"
#include "WallmarksEngine.h"
inline std::atomic<unsigned> g_runtime_marks_dropped{0}; // body decals that could not follow a wound rebuild (dropped, logged per publication)
#include "../../Include/xrRender/RuntimePackedMesh.h"
#include "../../Include/xrRender/RuntimeRevisionAdmission.h"
#include "../../Include/xrRender/RuntimeFaceLists.h"
#include "PainterShotDump.h" // shotdump (diagnostic): a screenshot writes what every wounded body and piece draws
void painter_shot_dump_now(const char* shot){painter_shot_dump::now(shot);} // called by a normal screenshot (r__screenshot.cpp)
namespace { RuntimeGeometryPacket* runtime_retired = nullptr; }
RuntimeGeometryPacket::~RuntimeGeometryPacket()
{
    for (auto* child:owned) xr_delete(child);
#ifdef USE_DX11
    _RELEASE(fence);
#endif
}
namespace {
bool arm_runtime_fence(RuntimeGeometryPacket* packet)
{
#ifdef USE_DX11
    if (!packet) return true;
    if (!packet->fence) {D3D11_QUERY_DESC desc={D3D11_QUERY_EVENT,0};if(FAILED(HW.pDevice->CreateQuery(&desc,&packet->fence))) return false;}
    return true;
#else
    return false;
#endif
}
void retire_runtime(RuntimeGeometryPacket* packet)
{
    if (!packet) return;
#ifdef USE_DX11
    if (packet->fence) HW.pContext->End(packet->fence);
#endif
    for (auto* child:packet->owned) fast_dynamic_cast<CSkeletonX*>(child)->SetParent(nullptr);
    packet->next_retired=runtime_retired;runtime_retired=packet;
}
}
#ifdef USE_DX11
namespace {
bool runtime_footprint_match(const CSkeletonWallmark::ProjectionSource* current,
    const xr_vector<const CSkeletonWallmark::ProjectionSource*>& previous)
{
    if(!current||!current->footprint_identity)return false;
    unsigned matches=0;
    for(const auto* old:previous)if(old&&old->footprint_identity==current->footprint_identity)++matches;
    return matches==1;
}
// Rebind logical range offsets only. A token is shared exclusively by an exact
// prepared-cache geometry/rig/owner/slot/provenance match; BindOwnership can
// duplicate vertices but preserves this ordered logical triangle index stream.
bool runtime_rebind_projection_ranges(const CSkeletonWallmark::ProjectionSnapshot& prior,
    const std::shared_ptr<const CSkeletonWallmark::ProjectionSource>& current,
    CSkeletonWallmark::ProjectionSnapshot& hit)
{
    if(!current||!current->footprint_identity||prior.epoch!=hit.epoch)return false;
    const auto limit=BloodGpuProjection::Backend::MaxCandidateVertices;
    if(hit.candidate_vertices>limit)return false;
    u32 count=0;bool found=false;
    for(const auto& range:prior.ranges){
        if(!range.source||range.source->footprint_identity!=current->footprint_identity)continue;
        found=true;
        if(range.first_vertex%3||range.vertex_count%3||!range.vertex_count||
           range.first_vertex>range.source->indices.size()||range.vertex_count>range.source->indices.size()-range.first_vertex||
           range.first_vertex>current->indices.size()||range.vertex_count>current->indices.size()-range.first_vertex||
           range.vertex_count>limit-hit.candidate_vertices-count)return false;
        count+=range.vertex_count;
    }
    if(!found)return false; // No retained proof for this receiver: select normally.
    for(const auto& range:prior.ranges)
        if(range.source&&range.source->footprint_identity==current->footprint_identity)
            hit.ranges.push_back({current,range.first_vertex,range.vertex_count});
    hit.candidate_vertices+=count;
    return true;
}
}
std::shared_ptr<PainterSkinOwnershipGpu> CKinematics::LL_RuntimeSkinOwnershipGpu()const
{
    return m_runtime_geometry&&m_runtime_geometry->required_fields?m_runtime_geometry->required_fields->skin_ownership:nullptr;
}
// maskdecal (value 3): the textures a mask piece's exterior tear programs read besides the removal field (its tear set, its anatomy
// field), for the masked decals of that piece; empty unless the current publication's exterior draws with painter_vmaskt_ programs.
bool CKinematics::LL_RuntimeTearMask(PainterWallmarkTear& out)const
{
    out=PainterWallmarkTear{};
    const RuntimeRequiredFields*f=m_runtime_geometry?m_runtime_geometry->required_fields.get():nullptr;
    if(!f||!f->mask_piece||!f->tear_exterior||!f->tear_gpu||!f->anatomy||!f->removal)return false;
    out.keep=f->tear_gpu;out.params=f->tear_gpu->params_texture;out.map=f->tear_gpu->map_texture;out.bake=f->tear_gpu->bake_texture;
    out.anatomyPages=f->anatomy->page_texture;out.anatomyData=f->anatomy->data_texture;
    if(!out)out=PainterWallmarkTear{};
    return bool(out);
}
// melee (value 3): the body's dents for its blood decals - 8 floats per dent (centre, radius, push, depth; rest space), the very list
// the exterior's dent texture holds and only once that texture is built (decals and skin always move together); key changes with
// the list (CKinematics::PrepareWallmarkRender re-skins / re-projects a decal on a new dent)
std::shared_ptr<const float> CKinematics::LL_RuntimeDentTexels(u32& dents,u64* key)const
{
    dents=0;if(key)*key=0;
    const RuntimeRequiredFields*f=m_runtime_geometry?m_runtime_geometry->required_fields.get():nullptr;
    if(!f||!f->dents||f->dents->empty()||!f->dent_gpu)return {};
    static_assert(sizeof(gfy_packed::Dent)==8*sizeof(float),"dent texel ABI");
    dents=u32(f->dents->size());if(key)*key=f->dent_key;
    return std::shared_ptr<const float>(f->dents,reinterpret_cast<const float*>(f->dents->data()));
}
#endif
void CollectRuntimeGeometryRetired(bool shutdown)
{
#ifdef USE_DX11
    // Called from model-pool owner phases, never an extraction worker. At
    // shutdown issue/flush outstanding events once; the loop is shutdown-only.
    if (shutdown && HW.pContext) {
        for(auto* p=runtime_retired;p;p=p->next_retired) if(!p->fence && arm_runtime_fence(p)) HW.pContext->End(p->fence);
        HW.pContext->Flush();
    }
    const DWORD shutdown_start=GetTickCount();
    auto** link=&runtime_retired;
    while (*link) {
        auto* packet=*link;BOOL done=FALSE;
        HRESULT status=packet->fence && HW.pContext ? HW.pContext->GetData(packet->fence,&done,sizeof(done),D3D11_ASYNC_GETDATA_DONOTFLUSH):E_FAIL;
        if (shutdown && status==S_FALSE && GetTickCount()-shutdown_start<500) {Sleep(0);continue;}
        const bool device_gone=shutdown && HW.pDevice && FAILED(HW.pDevice->GetDeviceRemovedReason());
        if ((status==S_OK && done) || device_gone) {*link=packet->next_retired;xr_delete(packet);}
        else link=&packet->next_retired;
    }
    if(shutdown && runtime_retired) Msg("! runtime geometry: GPU retirement incomplete at shutdown; retaining packets for process/device teardown");
#else
    (void)shutdown;
#endif
}
bool CKinematics::LL_InstallRuntimeGeometry(const RuntimeSkinUpdate& update,u64 current_owner_cookie)
{
    if(!LL_PrepareRuntimeGeometry(update,current_owner_cookie))return false;
    const bool committed=LL_CommitPreparedRuntimeGeometry(current_owner_cookie);
    if(!committed)LL_DiscardPreparedRuntimeGeometry();
    return committed;
}
void CKinematics::LL_DiscardPreparedRuntimeGeometry()
{
    xr_delete(m_prepared_runtime_geometry);
}
bool CKinematics::LL_PrepareRuntimeGeometry(const RuntimeSkinUpdate& update,u64 current_owner_cookie)
{
    return LL_PrepareRuntimeGeometryInternal(update,current_owner_cookie,this,nullptr);
}
bool CKinematics::LL_PrepareRuntimeGeometryWithDonor(const RuntimeSkinUpdate& update,u64 current_owner_cookie,
    IRenderVisual* donor_visual,const Fmatrix* recipient_xform)
{
    if(!donor_visual||!recipient_xform)return false;
    auto* donor=fast_dynamic_cast<CKinematics*>(donor_visual);
    if(!donor||donor==this||donor->LL_BoneCount()!=LL_BoneCount())return false;
    return LL_PrepareRuntimeGeometryInternal(update,current_owner_cookie,donor,recipient_xform);
}
// Owner admission shared by the synchronous path, stage creation and staged
// install: the same checks in the same order with the same reasons.
const char* CKinematics::LL_AdmitRuntimeHeader(const RuntimeSkinUpdate& update,u64 current_owner_cookie,bool has_children,
    xr_vector<IRenderVisual*>& originals)
{
    if(!Device.IsGeometryPublishBarrier())return "publication_barrier";
    if(m_prepared_runtime_geometry)return "already_prepared";
    if(!current_owner_cookie || update.owner_cookie!=current_owner_cookie)return "owner_cookie";
    if(bDontDelete)return "shared_model";
    if(m_geometry_donor)return "static_donor_active";
    if(!update.damage_revision)return "zero_revision";
    if(update.expected_geometry_generation!=m_render_geometry_generation)return "stale_generation";
    if(update.exact_rig.size()!=LL_BoneCount())return "rig_count";
    if(!has_children)return "no_children";
    if(m_runtime_geometry && m_runtime_geometry->owner_cookie!=current_owner_cookie)return "active_owner_cookie";
    RuntimeRevisionState active_revision;
    const RuntimeRevisionState* active=nullptr;
    if(m_runtime_geometry){
        active_revision={m_runtime_geometry->owner_cookie,
            m_runtime_geometry->damage_revision,m_runtime_geometry->separation_packet,m_runtime_geometry->flight_preview};
        active=&active_revision;
    }
    if(!RuntimeRevisionAdmitted(active,current_owner_cookie,m_render_geometry_generation,
        update.expected_geometry_generation,update.damage_revision,update.separation_packet,update.flight_preview,update.flight_detail))
        return "nonmonotonic_revision_or_stage";
    for (u16 b=0;b<LL_BoneCount();++b) {
        const auto& request=update.exact_rig[b];const auto& actual=LL_GetData(b);
        if (request.name!=LL_BoneName_dbg(b) || request.parent!=actual.GetParentID() ||
            memcmp(&request.model_to_bone,&actual.m2b_transform,sizeof(Fmatrix))) return "exact_rig";
    }
    // Admission is owner-barrier-only; staged workers receive no mutable material.
#ifdef USE_DX11
    RestoreBloodMaterials();
#endif
    const auto& visible=m_runtime_geometry?m_geometry_original_children:children;
    const auto& invisible=m_runtime_geometry?m_geometry_original_invisible:children_invisible;
    originals.clear();
    originals.resize(visible.size()+invisible.size(),nullptr);
    for (auto* visual:visible) {auto* s=fast_dynamic_cast<CSkeletonX*>(visual);if(!s || s->GeometryChildIndex()>=originals.size() || originals[s->GeometryChildIndex()])return "original_child_identity";originals[s->GeometryChildIndex()]=visual;}
    for (auto* visual:invisible) {auto* s=fast_dynamic_cast<CSkeletonX*>(visual);if(!s || s->GeometryChildIndex()>=originals.size() || originals[s->GeometryChildIndex()])return "original_child_identity";originals[s->GeometryChildIndex()]=visual;}
    for (auto* visual:originals) if(!visual)return "original_child_gap";
    return nullptr;
}
bool CKinematics::LL_PrepareRuntimeGeometryInternal(const RuntimeSkinUpdate& update,u64 current_owner_cookie,
    CKinematics* mark_donor,const Fmatrix* recipient_xform)
{
    auto reject=[&](const char* reason){Msg("[runtime_geometry_reject] stage=owner reason=%s cookie=%llu expected_generation=%llu actual_generation=%llu revision=%llu",reason,current_owner_cookie,update.expected_geometry_generation,m_render_geometry_generation,update.damage_revision);return false;};
#ifndef USE_DX11
    return reject("renderer_not_R4");
#else
    xr_vector<IRenderVisual*> originals;
    if(const char* why=LL_AdmitRuntimeHeader(update,current_owner_cookie,!update.replaced_children.empty(),originals))return reject(why);
    // RuntimeSkinUpdate cannot represent mandatory field bindings or the current
    // recipient union for carried marks. Keep the old packet if staging is unavailable.
    if((m_runtime_geometry&&m_runtime_geometry->required_fields)||
        (mark_donor&&mark_donor!=this&&mark_donor->m_runtime_geometry&&mark_donor->m_runtime_geometry->required_fields))
        return reject("required_fields_need_staged");
    size_t count=originals.size();for(const auto& c:update.replaced_children) count=_max(count,size_t(c.child_index)+1);
    if(count>gfy_packed::max_child_slots)return reject("child_slot_limit");
    xr_vector<u8> replaced;replaced.assign(count,u8(0));
    xr_vector<u8> removed;removed.assign(count,u8(0));
    for(const auto& c:update.replaced_children) {
        if(c.material_source_child>=originals.size())return reject("material_source_child");
        if(replaced[c.child_index])return reject("duplicate_child_slot");
        replaced[c.child_index]=true;
    }
    std::unique_ptr<RuntimeGeometryPacket> next(new RuntimeGeometryPacket());
    next->owner_cookie=current_owner_cookie;next->damage_revision=update.damage_revision;
    next->geometry_generation=m_render_geometry_generation+(update.flight_detail?0:1);
    next->separation_packet=update.separation_packet;
    next->flight_preview=update.flight_preview;next->flight_detail=update.flight_detail;
    next->children=originals;next->children.resize(count,nullptr);next->faces.resize(LL_BoneCount());
    // Replaced rows are rebuilt below. Copying the previous large cavity's
    // entire face table only to clear it causes avoidable owner-thread work.
    for(u16 b=0;b<LL_BoneCount();++b){
        next->faces[b].resize(count);
        const auto& previous=LL_GetData(b).child_faces;
        for(size_t child=0;child<previous.size()&&child<count;++child)
            if(!replaced[child])next->faces[b][child]=previous[child];
    }
    for(const auto& c:update.replaced_children) {
        if(c.remove_surface) {
            if(c.child_index>=originals.size() || !c.vertices.empty() || !c.indices.empty() || c.receives_wallmarks)
                return reject("invalid_removed_surface");
            next->children[c.child_index]=nullptr;removed[c.child_index]=true;
            for(u16 b=0;b<LL_BoneCount();++b)next->faces[b][c.child_index].clear();
            continue;
        }
        std::unique_ptr<CSkeletonX_ST> child(new CSkeletonX_ST());
        auto* material=fast_dynamic_cast<dxRender_Visual*>(originals[c.material_source_child]);
        if(!material)return reject("material_visual_type");
        if(!child->BuildRuntime(c,this,*material))return reject("child_build_see_child_reason");
        child->SetWallmarkOwnership(next->required_fields?next->required_fields->wallmark_ownership:nullptr);
        next->children[c.child_index]=child.get();next->owned.push_back(child.release());
        for(u16 b=0;b<LL_BoneCount();++b)next->faces[b][c.child_index].clear();
        BuildRuntimeFaceLists(c.vertices,c.indices,LL_BoneCount(),
            [&](u16 bone,u32 face){next->faces[bone][c.child_index].push_back(u16(face));});
    }
    xr_vector<u8> accepts(count,u8(1));
    for(const auto& c:update.replaced_children)accepts[c.child_index]=c.receives_wallmarks?1:0;
    xr_vector<IRenderVisual*> published;SkeletonWMVec rebuilt;
    if(const char* why=LL_FinishRuntimeGeometryPrepare(next.get(),count,removed,accepts,nullptr,mark_donor,recipient_xform,published,rebuilt))
        return reject(why);
    // All allocation and validation finish here. Neither the visible mesh nor
    // its marks change until Commit, allowing the caller to prepare every part.
    std::unique_ptr<RuntimeGeometryPrepared> prepared(new RuntimeGeometryPrepared());
    prepared->packet=next.release();prepared->wallmarks.swap(rebuilt);
    prepared->prior_packet=m_runtime_geometry;
    prepared->published.swap(published);
    m_prepared_runtime_geometry=prepared.release();
    return true;
#endif
}
// Tail of a prepare, shared by the synchronous and staged paths. Children and
// face rows are complete. sources_done marks slots whose single projection
// source attempt already happened off the barrier (staged children).
const char* CKinematics::LL_FinishRuntimeGeometryPrepare(RuntimeGeometryPacket* next,size_t count,const xr_vector<u8>& removed,
    const xr_vector<u8>& accepts,const xr_vector<u8>* sources_done,CKinematics* mark_donor,const Fmatrix* recipient_xform,
    xr_vector<IRenderVisual*>& published,SkeletonWMVec& rebuilt)
{
#ifndef USE_DX11
    return "renderer_not_R4";
#else
    (void)count;
    for(size_t n=0;n<next->children.size();++n)if(!next->children[n] && !removed[n])return "published_child_gap";
    published.clear();published.reserve(next->children.size());
    // Stable source indices remain in the private packet/face table. Render
    // traversal sees only actual visuals, including when a whole surface is gone.
    for(auto* child:next->children)if(child)published.push_back(child);
    // Every published packet owns a preallocated retirement fence. Despawn
    // never needs to allocate one under memory pressure.
    if(!arm_runtime_fence(next))return "retirement_fence";
    auto source_pending=[&](size_t n){return !sources_done || !(*sources_done)[n];};
    // Rebuild blood on surviving exterior before publishing anything. A failed
    // allocation/projection leaves both the old mesh and its marks untouched.
    SkeletonWMVec retained;unsigned droppedNoPlacement=0,droppedNoSource=0,droppedBudget=0;
    // Why a source could not be built (2026-09-29: every carried mark was dropped as no_source): the source admission
    // counters over this rebuild and the resident source size.
    const u64 sourceTotal0=g_blood_projection_cpu.cache_total_budget.load(std::memory_order_relaxed),
        sourceChild0=g_blood_projection_cpu.cache_child_budget.load(std::memory_order_relaxed),
        sourceInvalid0=g_blood_projection_cpu.cache_invalid.load(std::memory_order_relaxed),
        sourceEmpty0=g_blood_projection_cpu.cache_empty.load(std::memory_order_relaxed);
    struct DropReport{unsigned&a,&b,&c;size_t&kept;u64 total0,child0,invalid0,empty0;~DropReport(){if(a|b|c)
        Msg("[runtime_geometry] marks_dropped no_placement=%u no_source=%u candidate_budget=%u kept=%u source_total_budget=%llu source_child_budget=%llu source_invalid=%llu source_empty=%llu source_mb=%.1f",
            a,b,c,unsigned(kept),g_blood_projection_cpu.cache_total_budget.load(std::memory_order_relaxed)-total0,
            g_blood_projection_cpu.cache_child_budget.load(std::memory_order_relaxed)-child0,
            g_blood_projection_cpu.cache_invalid.load(std::memory_order_relaxed)-invalid0,
            g_blood_projection_cpu.cache_empty.load(std::memory_order_relaxed)-empty0,
            double(CSkeletonWallmark::ProjectionSource::ReservedBytes())/1048576.0);}};
    size_t keptMarks=0;DropReport dropReport{droppedNoPlacement,droppedNoSource,droppedBudget,keptMarks,sourceTotal0,sourceChild0,sourceInvalid0,sourceEmpty0};
    // Only children that receive decals can take (or refuse) a carried mark: exposed anatomy never has a source.
    auto receiver=[&](size_t n){auto* s=next->children[n]?fast_dynamic_cast<CSkeletonX*>(next->children[n]):nullptr;
        return accepts[n]&&s&&s->receives_wallmarks;};
    try {
        if(::RImplementation.Wallmarks)::RImplementation.Wallmarks->CollectSkeletonWallmarks(mark_donor,retained);
        {xrCriticalSectionGuard guard(wallmarks_cs);retained.insert(retained.end(),mark_donor->wallmarks.begin(),mark_donor->wallmarks.end());}
        if(!retained.empty()) {
            for(size_t n=0;n<next->children.size();++n)if(receiver(n) && source_pending(n))
                fast_dynamic_cast<CSkeletonX*>(next->children[n])->FinalizeProjectionSource(&next->faces,true);
        }
        // Receivers' projection sources now and after this publish: when the sets are equal, a retained mark holds
        // exactly the ranges it would select again (same immutable sources, same hit pose), so the mark itself is kept.
        const u64 projectionEpoch=g_blood_projection_epoch.load(std::memory_order_relaxed);
        xr_vector<const CSkeletonWallmark::ProjectionSource*> sourcesNow,sourcesNext,footprintNow;
        if(mark_donor==this&&!recipient_xform){
            for(auto* visual:children){auto* s=visual?fast_dynamic_cast<CSkeletonX*>(visual):nullptr;
                if(s&&s->receives_wallmarks&&s->HasProjectionSource())sourcesNow.push_back(s->ProjectionSourceKey());}
            footprintNow=sourcesNow; // Independent of the all-receivers sameSources test below.
            for(size_t n=0;n<next->children.size();++n){if(!receiver(n))continue;auto* s=fast_dynamic_cast<CSkeletonX*>(next->children[n]);
                if(s->HasProjectionSource())sourcesNext.push_back(s->ProjectionSourceKey());else{sourcesNext.clear();sourcesNow.clear();break;}}
            std::sort(sourcesNow.begin(),sourcesNow.end());std::sort(sourcesNext.begin(),sourcesNext.end());}
        const bool sameSources=!sourcesNext.empty()&&sourcesNow==sourcesNext;
        // Qualify receivers once, not once per retained mark. Tokens never cross
        // owners/donors/transforms, nor admit a child absent from the old body.
        xr_vector<std::shared_ptr<const CSkeletonWallmark::ProjectionSource>> footprintNext(next->children.size());
        if(mark_donor==this&&!recipient_xform){
            for(size_t n=0;n<next->children.size();++n){if(!receiver(n))continue;
                const auto& source=fast_dynamic_cast<CSkeletonX*>(next->children[n])->ProjectionSourceHandle();
                if(runtime_footprint_match(source.get(),footprintNow))footprintNext[n]=source;
            }
        }
        auto holdsSource=[](const CSkeletonWallmark::ProjectionSnapshot* prior,const CSkeletonWallmark::ProjectionSource* key){
            if(!prior||!key)return false;for(const auto& range:prior->ranges)if(range.source.get()==key)return true;return false;};
        for(const auto& mark:retained) {
            if(!mark || (mark->TimeEnd()!=-1.f && RDEVICE.fTimeGlobal-mark->TimeStart()>=mark->TimeEnd()))continue;
            if(!mark->m_Reprojection){++g_runtime_marks_dropped;++droppedNoPlacement;continue;}
            {const auto* held=mark->m_Projection?mark->m_Projection.get():mark->m_CarryRanges.get();
             if(sameSources&&held&&held->epoch==projectionEpoch){rebuilt.push_back(mark);++keptMarks;continue;}}
            auto hit=std::allocate_shared<CSkeletonWallmark::ProjectionSnapshot>(
                CSkeletonWallmark::CacheAllocator<CSkeletonWallmark::ProjectionSnapshot>{},*mark->m_Reprojection);
            hit->ranges.clear();hit->candidate_vertices=0;
            hit->epoch=g_blood_projection_epoch.load(std::memory_order_relaxed);
            // An unchanged child (the visual this model shows now, with the same immutable source) yields exactly the ranges
            // this mark already holds for it: copy them. Only replaced children select their footprint again.
            const auto* prior=mark_donor==this?(mark->m_Projection?mark->m_Projection.get():mark->m_CarryRanges.get()):nullptr;
            if(prior&&prior->epoch!=projectionEpoch)prior=nullptr; // ranges of another projection epoch are selected again
            bool projectable=true;
            for(size_t n=0;n<next->children.size()&&projectable;++n){
                if(!receiver(n))continue;
                auto* reused=fast_dynamic_cast<CSkeletonX*>(next->children[n]);
                // The same visual, or a re-created child with the same immutable projection source: the ranges this mark
                // holds for that source are exactly what selecting again would produce.
                if(prior&&reused->HasProjectionSource()&&(std::find(children.begin(),children.end(),next->children[n])!=children.end()||holdsSource(prior,reused->ProjectionSourceKey()))){
                    const auto* key=reused->ProjectionSourceKey();
                    for(const auto& range:prior->ranges)
                        if(range.source.get()==key){hit->ranges.push_back(range);hit->candidate_vertices+=range.vertex_count;}
                    if(hit->candidate_vertices>BloodGpuProjection::Backend::MaxCandidateVertices){++droppedBudget;projectable=false;}
                    continue;
                }
                if(prior&&footprintNext[n]&&runtime_rebind_projection_ranges(*prior,footprintNext[n],*hit))continue;
                if(reused->AppendProjectionRanges(hit->selected_bones,*hit,false))continue;
                auto* sk=reused;
                if(!sk->HasProjectionSource()){sk->FinalizeProjectionSource(&next->faces,true);
                    if(sk->HasProjectionSource()&&sk->AppendProjectionRanges(hit->selected_bones,*hit,false))continue;
                    ++droppedNoSource;}else ++droppedBudget;
                projectable=false;}
            if(!projectable){++g_runtime_marks_dropped;continue;}
            const Fmatrix* mark_xform=recipient_xform?recipient_xform:mark->XFORM();
            intrusive_ptr<CSkeletonWallmark> copy=xr_new<CSkeletonWallmark>(this,mark_xform,mark->Shader(),
                mark->ContactPoint(),mark->TimeStart(),mark->TimeEnd());
            copy->m_LocalBounds=mark->m_LocalBounds;copy->m_Bounds=mark->m_Bounds;
            if(recipient_xform)mark_xform->transform_tiny(copy->m_Bounds.P,copy->m_LocalBounds.P);
            copy->m_SquareFootprint=mark->m_SquareFootprint;
            copy->m_Growth=mark->m_Growth;
            copy->m_thermal_stamp=mark->m_thermal_stamp;
            copy->m_GrowthLease=mark->m_GrowthLease;
            copy->m_Reprojection=mark->m_Reprojection;copy->m_Projection=std::move(hit);
            // R37: on the same body a mark that has faces keeps drawing them after the commit (taken over there, see
            // LL_CommitPreparedRuntimeGeometry): the removal field hides faces on carved-away skin, and the new ranges
            // refresh the faces in the background instead of a render-thread re-resolve of every decal per bullet.
            if(mark_donor==this&&!recipient_xform&&!mark->m_Faces.empty()){
                copy->m_CarryFrom=mark.get();copy->m_CarryKeeper=std::shared_ptr<void>(nullptr,[keep=mark](void*){(void)keep;});}
            // The snapshot owns only NEW geometry. Preserve GPU creation;
            // forcing all marks through CPU projection here would turn every
            // mesh update into a synchronous decal-creation burst. The normal
            // render path can resolve a cold snapshot without using old skin.
            if(copy->m_Projection->candidate_vertices){rebuilt.push_back(std::move(copy));++keptMarks;}
        }
    } catch(...) {return "decal_reprojection_allocation";}
    for(auto* visual:next->owned){
        auto* skin=fast_dynamic_cast<CSkeletonX*>(visual);
        if(skin->receives_wallmarks && source_pending(skin->GeometryChildIndex()))skin->FinalizeProjectionSource(&next->faces,true);
    }
    try {
        if(mark_donor&&mark_donor->m_blood_cells)
            next->blood_cells=mark_donor==this?m_blood_cells:std::make_shared<PainterBloodInstance>(mark_donor->m_blood_cells->shared);
    } catch(...) {return "blood_cell_lineage_allocation";}
    return nullptr;
#endif
}
bool CKinematics::LL_FlightPreviewAllowed() const
{
#ifdef USE_DX11
    if (!r_painter_publish || !Device.IsGeometryPublishBarrier()) return false;
    {
        xrCriticalSectionGuard guard(wallmarks_cs);
        if (!wallmarks.empty()) return false;
    }
    return !::RImplementation.Wallmarks || !::RImplementation.Wallmarks->HasSkeletonWallmarks(this);
#else
    return false;
#endif
}

void CKinematics::RecordFlightDraw()
{
#ifdef USE_DX11
    if (!m_flight_first_draw || !m_runtime_geometry || RImplementation.phase!=RImplementation.PHASE_NORMAL) return;
    m_flight_first_draw=false;
    Msg("[painter_flight] phase=first_draw lod=%s cookie=%llu revision=%llu frame=%u t=%u host_us=%lld",
        m_runtime_geometry->flight_preview?"preview":"exact",m_runtime_geometry->owner_cookie,
        m_runtime_geometry->damage_revision,Device.dwFrame,Device.dwTimeGlobal, static_cast<long long>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()));
#endif
}

bool CKinematics::LL_CanCommitPreparedRuntimeGeometry(u64 current_owner_cookie) const
{
#ifndef USE_DX11
    return false;
#else
    auto* prepared=m_prepared_runtime_geometry;
    if(!prepared||!Device.IsGeometryPublishBarrier()||
       prepared->prior_packet!=m_runtime_geometry)return false;
    const auto* next=prepared->packet;
    if(!next||next->geometry_generation!=m_render_geometry_generation+(next->flight_detail?0:1))return false;
    if(next->required_fields&&!next->required_fields->AdmittedAtCommit(HW.pDevice,current_owner_cookie,
        m_render_geometry_generation,next->damage_revision))return false;
    RuntimeRevisionState active_revision;
    const RuntimeRevisionState* active=nullptr;
    if(m_runtime_geometry){
        active_revision={m_runtime_geometry->owner_cookie,
            m_runtime_geometry->damage_revision,m_runtime_geometry->separation_packet,m_runtime_geometry->flight_preview};
        active=&active_revision;
    }
    return RuntimeRevisionAdmitted(active,current_owner_cookie,m_render_geometry_generation,
        m_render_geometry_generation,next->damage_revision,next->separation_packet,next->flight_preview,next->flight_detail)&&
        next->owner_cookie==current_owner_cookie;
#endif
}
bool CKinematics::LL_CommitPreparedRuntimeGeometry(u64 current_owner_cookie)
{
#ifndef USE_DX11
    return false;
#else
    if(!LL_CanCommitPreparedRuntimeGeometry(current_owner_cookie))return false;
    auto* prepared=m_prepared_runtime_geometry;
    // No allocation or fallible child construction below this point.
    RuntimeGeometryPacket* next=prepared->packet;
    // R37: carried marks take the faces of the marks they replace now; those old marks leave the slots in this same
    // step, so nothing draws them again. A mark whose faces are gone meanwhile resolves like a fresh one.
    for(auto& wm:prepared->wallmarks)if(wm&&wm->m_CarryFrom){
        if(!wm->m_CarryFrom->m_Faces.empty()){wm->m_Faces.swap(wm->m_CarryFrom->m_Faces);
            wm->ApplyCarryOwnership(prepared->packet->required_fields?prepared->packet->required_fields->wallmark_ownership:nullptr);wm->InvalidateRenderCache();
            g_painter_marks_carried.fetch_add(1,std::memory_order_relaxed);}
        wm->m_CarryFrom=nullptr;wm->m_CarryKeeper.reset();}
    if(::RImplementation.Wallmarks)::Render->remove_SkeletonWallmarksFromObject(this);
    {xrCriticalSectionGuard guard(wallmarks_cs);wallmarks.swap(prepared->wallmarks);wm_frame=u32(-1);}
    RuntimeGeometryPacket* old=m_runtime_geometry;
    {
        xrCriticalSectionGuard guard(&UCalc_Mutex);
        if(!old){m_geometry_original_children.swap(children);m_geometry_original_invisible.swap(children_invisible);m_geometry_original_lod=m_lod;}
        else {children.clear();children_invisible.clear();}
        children.swap(prepared->published);m_lod=nullptr;m_runtime_geometry=next;
        painter_shot_dump::live(this,next->required_fields?next->required_fields->shot_mesh:nullptr,next->owner_cookie,next->damage_revision); // shotdump (diagnostic)
        m_blood_cells=next->blood_cells;if(m_blood_cells)m_blood_cells->refreshed=~0u;
        m_flight_first_draw=next->flight_preview || next->flight_detail;
        if(next->flight_detail)++m_flight_detail_swaps;
        else ++m_render_geometry_generation;
        for(auto* visual:children)fast_dynamic_cast<CSkeletonX*>(visual)->SetParent(this);
        CalculateBones_Invalidate();Visibility_Invalidate();
    }
    // Projection sources are built from this generation's private face table.
    // Every new child starts cold; unchanged children keep the immutable source.
    prepared->packet=nullptr;LL_DiscardPreparedRuntimeGeometry();
#if RENDER==R_R4
    if(m_burn_binding && m_burn_binding->hasRenderedBurn){
        m_burn_binding->frame=~0u;
        PainterBurnRefresh();
    }
#endif
    retire_runtime(old);CollectRuntimeGeometryRetired(false);
    return true;
#endif
}
void CKinematics::LL_ClearRuntimeGeometry()
{
    LL_DiscardPreparedRuntimeGeometry();
    if(!m_runtime_geometry)return;
    // Also called during Depart/destruction at the existing model-pool joined
    // owner phase. No game/worker callback may delete an active visual directly.
    if(::RImplementation.Wallmarks)::Render->remove_SkeletonWallmarksFromObject(this);
    ClearWallmarks();auto* old=m_runtime_geometry;
    painter_shot_dump::gone(this); // shotdump (diagnostic)
    // Failure to create a fence retains the packet until shutdown, never frees
    // buffers potentially in flight from the destruction path.
    arm_runtime_fence(old);
    {xrCriticalSectionGuard guard(&UCalc_Mutex);children.clear();children_invisible.clear();children.swap(m_geometry_original_children);children_invisible.swap(m_geometry_original_invisible);
     m_lod=m_geometry_original_lod;m_geometry_original_lod=nullptr;m_runtime_geometry=nullptr;++m_render_geometry_generation;CalculateBones_Invalidate();Visibility_Invalidate();}
    retire_runtime(old);
}
#ifdef USE_DX11
// ---------------------------------------------------------------------------
// Off-thread publication (RuntimeGeometryStage.h).
//
// Thread-safety of everything Run() does (worker thread):
//  - reads only the stage: captured scalars, the packed rows it keeps alive
//    (shared_ptr) and its own outputs; never the owner model, HW, children,
//    bone data, the resource manager, stats_manager or wallmarks;
//  - allocates through Memory.mem_alloc/xalloc (_aligned_malloc,
//    xrMemory_subst_msvc.cpp:42-64) and ref_smem, whose count is atomic and
//    whose owned storage never enters the global smem container
//    (xrsharedmem.h create_unique*);
//  - creates IB/VB on the device captured (AddRef) at the barrier; the DX11
//    device is free-threaded (dx10HW.cpp:556-566, no SINGLETHREADED flag);
//    the immediate context is never used;
//  - projection sources: ProjectionSource::ReserveBudget is a CAS on a global
//    atomic, Publish takes projection_pending_mutex (SkeletonCustom.cpp:28-80),
//    g_blood_projection_cpu counters are std::atomic;
//  - no shared_str is created, copied or destroyed (non-atomic counter).
float RuntimeStageMs(const LARGE_INTEGER& from) noexcept;
namespace {
void runtime_stage_projection(RuntimeStagedChild& child,unsigned bone_count,
    const xr_vector<xr_vector<CBoneData::FacesVec>>& faces,CSkeletonWallmark::ProjectionVector<u32>& remap) noexcept
{
    // CSkeletonX::CacheProjectionSource(0,iCount,&faces,true) for a child whose
    // only software replica is Vertices4W (SkeletonX.cpp:597-712).
    g_blood_projection_cpu.cache_attempts.fetch_add(1,std::memory_order_relaxed);
    try {
        auto source=std::allocate_shared<CSkeletonWallmark::ProjectionSource>(
            CSkeletonWallmark::CacheAllocator<CSkeletonWallmark::ProjectionSource>{});
        const u16 slot=child.child_index;
        const auto result=runtime_lean::build_projection(*child.vertices,child.vcount,*child.indices,child.icount,
            bone_count,BloodGpuProjection::Backend::MaxPaletteBones,BloodGpuProjection::Backend::MaxSourceIndices,
            [&](unsigned b)->const CBoneData::FacesVec&{return faces[b][slot];},remap,*source,
            [&](size_t bytes){return source->ReserveBudget(bytes);});
        switch(result){
        case runtime_lean::ProjectionResult::built:
            for(unsigned b=0;b<bone_count;++b)for(u16 face:faces[b][slot])source->source_faces.push_back(face);
            if(!source->BindOwnership(child.required_fields?child.required_fields->wallmark_ownership:nullptr,slot,child.material_source_child,child.source?&child.source->original_faces:nullptr)){
                g_blood_projection_cpu.cache_invalid.fetch_add(1,std::memory_order_relaxed);return;}
            source->BuildProjectionBounds();
            source->footprint_identity=child.footprint_identity?child.footprint_identity:std::make_shared<const u8>(u8(0));
            child.footprint_identity=source->footprint_identity;
            CSkeletonWallmark::ProjectionSource::Publish(source);
            child.projection=std::move(source);
            g_blood_projection_cpu.cache_built.fetch_add(1,std::memory_order_relaxed);
            break;
        case runtime_lean::ProjectionResult::empty:
            g_blood_projection_cpu.cache_empty.fetch_add(1,std::memory_order_relaxed);break;
        case runtime_lean::ProjectionResult::invalid:
            g_blood_projection_cpu.cache_invalid.fetch_add(1,std::memory_order_relaxed);break;
        case runtime_lean::ProjectionResult::budget:
            break; // ReserveBudget counted it (cache_child_budget / cache_total_budget)
        }
    } catch(...) {g_blood_projection_cpu.cache_alloc_failed.fetch_add(1,std::memory_order_relaxed);}
}
// One renderer-owned worker thread, FIFO, no allocation while queueing
// (intrusive list), no C++ exceptions (the renderer builds without /EH).
class RuntimeStageExecutor
{
    SRWLOCK lock=SRWLOCK_INIT;
    CONDITION_VARIABLE wake=CONDITION_VARIABLE_INIT;
    RuntimeGeometryStage* head=nullptr;
    RuntimeGeometryStage* tail=nullptr;
    RuntimeGeometryStage* running=nullptr;
    HANDLE thread=nullptr;
    bool stop=false,failed=false;
    static DWORD WINAPI Entry(void* self){static_cast<RuntimeStageExecutor*>(self)->Loop();return 0;}
    void Loop()
    {
        thread_name("X-Ray Painter publish");
        painter_apply_thread_priority(); // console painter_thread_priority (device.h)
        AcquireSRWLockExclusive(&lock);
        for(;;){
            while(!head && !stop)SleepConditionVariableSRW(&wake,&lock,INFINITE,0);
            if(!head)break; // stopping with an empty queue; canceled stages still run (and finish) first
            RuntimeGeometryStage* stage=head;head=stage->queue_next;if(!head)tail=nullptr;
            stage->queue_next=nullptr;running=stage;
            ReleaseSRWLockExclusive(&lock);
            stage->Run();
            AcquireSRWLockExclusive(&lock);
            running=nullptr;
            ReleaseSRWLockExclusive(&lock);
            stage->Release(); // may destroy it here: unbound buffers + CPU memory only
            AcquireSRWLockExclusive(&lock);
        }
        ReleaseSRWLockExclusive(&lock);
    }
public:
    bool Submit(RuntimeGeometryStage* stage)
    {
        AcquireSRWLockExclusive(&lock);
        if(!thread && !stop && !failed){
            thread=CreateThread(nullptr,0,&Entry,this,0,nullptr);
            if(!thread){failed=true;Msg("! runtime geometry: off-thread publication unavailable (CreateThread error %u); using the synchronous path",GetLastError());}
        }
        const bool queued=thread && !stop;
        if(queued){
            stage->AddRef();QueryPerformanceCounter(&stage->submitted);
            if(tail)tail->queue_next=stage;else head=stage;
            tail=stage;
        }
        ReleaseSRWLockExclusive(&lock);
        if(queued)WakeConditionVariable(&wake);
        return queued;
    }
    void Shutdown()
    {
        AcquireSRWLockExclusive(&lock);
        stop=true;
        for(auto* stage=head;stage;stage=stage->queue_next)stage->Cancel();
        if(running)running->Cancel();
        HANDLE worker=thread;
        ReleaseSRWLockExclusive(&lock);
        WakeAllConditionVariable(&wake);
        if(worker){WaitForSingleObject(worker,INFINITE);CloseHandle(worker);}
        AcquireSRWLockExclusive(&lock);
        thread=nullptr;stop=false; // a later renderer instance may start it again
        ReleaseSRWLockExclusive(&lock);
    }
};
RuntimeStageExecutor runtime_stage_executor;
}
void RuntimeGeometryStageShutdown(){runtime_stage_executor.Shutdown();RuntimePreparedCpuClear();PainterBloodShutdown();}
// A cache entry is only pinned by queued work and published children. Merely
// warming an undamaged NPC must not prevent another model's real wound admission.
static std::shared_ptr<PainterTissueGpu> RuntimeQueueTissueField(
    const std::shared_ptr<const painter_detail::TissueField>& field,
    const xr_vector<shared_str>* textures)
{
    if(!HW.pDevice)return {};
    auto gpu=PainterTissueCache::Prepare(field,textures);
    if(!gpu)return {};
    unsigned expected=0;
    if(gpu->native.compare_exchange_strong(expected,1)){
        auto* stage=new RuntimeGeometryStage();stage->tissue_only=true;stage->painter_tissue=gpu;
        stage->device=HW.pDevice;stage->device->AddRef();
        if(!runtime_stage_executor.Submit(stage))gpu->native.store(3,std::memory_order_release);
        stage->Release();
    }
    return gpu;
}

// Owner thread creates the immutable resource bundle. A separate executor item
// only creates native textures; ResourceManager registration stays at install.
static std::shared_ptr<RuntimeRequiredFields> RuntimeQueueRequiredFields(
    const std::shared_ptr<const gfy_packed::Mesh>&packed_mesh,const std::shared_ptr<PainterTissueGpu>&anatomy,
    u64 cookie,u64 generation,u64 revision,bool prior_wallmark_ownership)
{
    const auto&mesh=*packed_mesh;
    if(!prior_wallmark_ownership&&!RuntimeHasRequiredFields(mesh))return {};
    RuntimeRequiredFieldCache::Collect();
    auto fields=std::make_shared<RuntimeRequiredFields>();
    fields->epoch=RuntimeRequiredFieldCache::epoch();fields->cookie=cookie;fields->generation=generation;fields->revision=revision;
    if(painter_shot_dump::enabled())fields->shot_mesh=packed_mesh; // shotdump (diagnostic)
    fields->need_channels=bool(mesh.analytic_channels);fields->need_anatomy=fields->need_channels;
    // Mask-only wounds: the exterior reads the anatomy volume; without this model's anatomy field the packet keeps today's clip.
    fields->mask_volume=mesh.mask_volume&&bool(anatomy)&&!mesh.analytic_channels&&!mesh.skin_ownership;
    fields->mask_piece=fields->mask_volume&&mesh.mask_piece; // slice 2: a detached piece (painter_vmaskp_*)
    fields->tear_atlas=fields->mask_piece?mesh.tear_atlas:nullptr; // slice 4: the drawing set of all its torn ends
    if(!mesh.dents.empty()){fields->dents=std::make_shared<const std::vector<gfy_packed::Dent>>(mesh.dents); // melee (value 3)
        fields->dent_key=painter_dent::key(reinterpret_cast<const float*>(mesh.dents.data()),mesh.dents.size());}
    if(!mesh.slits.empty())fields->slits=std::make_shared<const std::vector<gfy_packed::Slit>>(mesh.slits); // melee: the smooth slash edge
    fields->need_anatomy=fields->need_anatomy||fields->mask_volume;
    fields->need_skin_ownership=bool(mesh.skin_ownership);fields->skin_source=mesh.skin_ownership;
    fields->need_wallmark_ownership=fields->need_skin_ownership||prior_wallmark_ownership;
    fields->need_removal=fields->need_channels;
    for(const auto&c:mesh.children)fields->need_removal=fields->need_removal||c.skin_clip;
    fields->anatomy=anatomy;fields->channel_source=mesh.analytic_channels;fields->removal_source=mesh.removal_field;
    if(fields->need_channels)fields->channels=PainterChannelCache::Prepare(mesh.analytic_channels,fields->epoch);
    if(fields->need_removal)fields->removal=PainterRemovalCache::Prepare(mesh.removal_field,fields->epoch);
    if(fields->need_skin_ownership)fields->skin_ownership=PainterSkinOwnershipCache::Prepare(mesh.skin_ownership,fields->epoch);
    if(fields->need_wallmark_ownership)fields->wallmark_ownership=PainterWallmarkOwnershipCache::Prepare(packed_mesh,fields->epoch,prior_wallmark_ownership);
    fields->allocation_refused=(fields->need_anatomy&&!anatomy)||(fields->need_channels&&!fields->channels)||
        (fields->need_removal&&!fields->removal)||(fields->need_skin_ownership&&!fields->skin_ownership)||(fields->need_wallmark_ownership&&!fields->wallmark_ownership);
    if(fields->allocation_refused)return fields;
    // Allocate the queue item before reserving either entry: failed allocation
    // cannot leave a permanent native==1 resource with no executor owner.
    auto*upload=new RuntimeGeometryStage();upload->fields_only=true;upload->required_fields=fields;
    upload->device=HW.pDevice;upload->device->AddRef();
    upload->build_channels=fields->channels&&fields->channels->ReserveNative();
    upload->build_removal=fields->removal&&fields->removal->ReserveNative();
    upload->build_skin_ownership=fields->skin_ownership&&fields->skin_ownership->ReserveNative();
    upload->build_wallmark_ownership=fields->wallmark_ownership&&fields->wallmark_ownership->ReserveNative();
    if(upload->build_channels||upload->build_removal||upload->build_skin_ownership||upload->build_wallmark_ownership){
        if(!runtime_stage_executor.Submit(upload)){
            if(upload->build_channels)fields->channels->native.store(3,std::memory_order_release);
            if(upload->build_removal)fields->removal->native.store(3,std::memory_order_release);
            if(upload->build_skin_ownership)fields->skin_ownership->native.store(3,std::memory_order_release);
            if(upload->build_wallmark_ownership)fields->wallmark_ownership->FailNative();
        }
    }
    upload->Release();return fields;
}
void RuntimeGeometryStage::Run() noexcept
{
    if(fields_only){
        if(build_channels)required_fields->channels->BuildNative(device);
        if(build_removal)required_fields->removal->BuildNative(device);
        if(build_skin_ownership)required_fields->skin_ownership->BuildNative(device);
        if(build_wallmark_ownership)required_fields->wallmark_ownership->BuildNative(device);
        ok=true;ready.store(true,std::memory_order_release);return;
    }
    if(tissue_only){if(painter_tissue)painter_tissue->BuildNative(device);ok=true;ready.store(true,std::memory_order_release);return;}
    timing.queue_ms=RuntimeStageMs(submitted);
    LARGE_INTEGER start;QueryPerformanceCounter(&start);
    ok=false;reason="stage_canceled";
    if(!canceled.load(std::memory_order_relaxed)){
        reason=nullptr;
        faces.resize(bone_count);
        for(auto& row:faces)row.resize(count);
        RuntimeStageScratch scratch;
        // Packet layout: one vertHW_4W array and one u16 array holding every
        // built child at its vbase/ibase, in packed (ascending slot) order; ONE
        // VB and ONE IB are created from them after the last child. Counts are
        // checked here so the arrays are allocated only for admissible sizes; a
        // count failure is reported with build_rows' reason for that child
        // (before a row failure of an earlier child, which the per-child order
        // reported first).
        total_vertices=0;total_indices=0;
        for(auto& child:children){
            if(child->remove_surface){if(!child->removed_valid){reason="invalid_removed_surface";break;}continue;}
            if(!child->material_valid){reason="material_visual_type";break;}
            const size_t count=child->source->vertices.size(),index_count=child->source->indices.size();
            if(const char* why=runtime_lean::validate_counts(count,index_count)){
                xr_sprintf(child_message,sizeof(child_message),"[runtime_geometry_reject] stage=child reason=%s child=%u vertex=%llu influence=%u registers=%u",
                    why,child->child_index,u64(0),0u,registers);
                reason="child_build_see_child_reason";break;
            }
            child->vbase=u32(total_vertices);child->ibase=u32(total_indices);
            total_vertices+=count;total_indices+=index_count;
        }
        if(!reason && !scratch.reserve_packet(total_vertices,total_indices)){
            xr_sprintf(child_message,sizeof(child_message),"[runtime_geometry_reject] stage=packet reason=allocation");
            reason="child_build_see_child_reason";
        }
        // Packed order is ascending slot order, as the synchronous path visits
        // update.replaced_children; the first failing child decides the reason.
        if(!reason)for(auto& child:children){
            if(canceled.load(std::memory_order_relaxed)){reason="stage_canceled";break;}
            if(child->remove_surface){if(!child->removed_valid){reason="invalid_removed_surface";break;}continue;}
            if(!child->material_valid){reason="material_visual_type";break;}
            ++timing.children;timing.vertices+=u32(child->source->vertices.size());timing.triangles+=u32(child->source->indices.size()/3);
            if(!RuntimePreparedCpuRestore(*this,*child,scratch) &&
                !RuntimeStageBuildChild(*child,bone_count,registers,faces,canceled,timing,scratch,child_message,sizeof(child_message))){
                reason=canceled.load(std::memory_order_relaxed)?"stage_canceled":"child_build_see_child_reason";break;
            }
        }
        if(!reason && published_gap)reason="published_child_gap";
        if(!reason){
            // Two CreateBuffer calls for the whole packet (engine-threading-
            // publication.md 3.4); every built child takes its own reference.
            LARGE_INTEGER buffers_start;QueryPerformanceCounter(&buffers_start);
            if(!RuntimeStagePackBuffers(device,scratch,total_vertices,total_indices,&vb,&ib,child_message,sizeof(child_message)))
                reason="child_build_see_child_reason";
            else for(auto& child:children)if(child->built){
                child->vb=vb;if(vb)vb->AddRef();
                child->ib=ib;if(ib)ib->AddRef();
            }
            timing.buffers_ms+=RuntimeStageMs(buffers_start);
        }
        if(!reason){
            // One projection source per new exterior child, forced like the
            // synchronous path's FinalizeProjectionSource(&faces,true).
            LARGE_INTEGER projection_start;QueryPerformanceCounter(&projection_start);
            CSkeletonWallmark::ProjectionVector<u32> remap;
            for(auto& child:children)if(child->built && child->receives_wallmarks && !child->projection_attempted){
                if(canceled.load(std::memory_order_relaxed)){reason="stage_canceled";break;}
                runtime_stage_projection(*child,bone_count,faces,remap);child->projection_attempted=true;
            }
            timing.projection_ms=RuntimeStageMs(projection_start);
#if RENDER==R_R4
            if(!reason && burnSource)for(auto& child:children)if(child->built)
                child->burnFaces=PainterBurnGpu::buildFaces(device,burnSource,child->material_source_child,&child->source->original_faces);
#endif
            if(!reason)for(const auto& child:children)if(child->built)RuntimePreparedCpuStore(*this,*child,scratch);
            ok=!reason;
        }
    }
    if(ok&&!canceled.load(std::memory_order_relaxed)&&packed&&packed->post_work)packed->post_work->Run(*packed); // Pure packed-data auxiliary result, never live objects.
    timing.run_ms=RuntimeStageMs(start);
    ready.store(true,std::memory_order_release);
}
IRuntimeGeometryStage* CKinematics::LL_StageRuntimeGeometry(const RuntimeSkinUpdate& header,
    const std::shared_ptr<const gfy_packed::Mesh>& packed,LPCSTR cavity_shader,LPCSTR cavity_texture,
    const xr_vector<shared_str>* tissue_textures,u64 current_owner_cookie,int& outcome,std::shared_ptr<const painter_burn::SourceFaces> burnSource)
{
    outcome=RuntimeStageOutcome::unavailable;
    if(!r_painter_publish || !packed || !HW.pDevice)return nullptr;
    // Once per process: whether the driver creates resources concurrently with
    // the immediate context (otherwise the runtime serialises the two
    // CreateBuffer calls per publication with rendering; still bounded).
    static bool threading_logged=false;
    if(!threading_logged){threading_logged=true;D3D11_FEATURE_DATA_THREADING threading{};
        if(SUCCEEDED(HW.pDevice->CheckFeatureSupport(D3D11_FEATURE_THREADING,&threading,sizeof(threading))))
            Msg("[runtime_geometry] d3d11_threading concurrent_creates=%u command_lists=%u",unsigned(threading.DriverConcurrentCreates),unsigned(threading.DriverCommandLists));}
    LARGE_INTEGER start;QueryPerformanceCounter(&start);
    // AppendPackedRuntimeChildren's only failure (RuntimePackedBridge.h:10-11)
    // is checked first, exactly as the synchronous path unpacks before install.
    if(tissue_textures)for(const auto& src:packed->children)
        if(src.material&&(src.material>=tissue_textures->size()||!(*tissue_textures)[src.material].size()))
            {outcome=RuntimeStageOutcome::unpack;return nullptr;}
    auto reject=[&](const char* reason)->IRuntimeGeometryStage*{
        Msg("[runtime_geometry_reject] stage=owner reason=%s cookie=%llu expected_generation=%llu actual_generation=%llu revision=%llu",
            reason,current_owner_cookie,header.expected_geometry_generation,m_render_geometry_generation,header.damage_revision);
        outcome=RuntimeStageOutcome::rejected;return nullptr;};
    xr_vector<IRenderVisual*> originals;
    if(const char* why=LL_AdmitRuntimeHeader(header,current_owner_cookie,!packed->children.empty(),originals))return reject(why);
    size_t count=originals.size();for(const auto& c:packed->children)count=_max(count,size_t(c.child_index)+1);
    if(count>gfy_packed::max_child_slots)return reject("child_slot_limit");
    if(const char*why=RuntimeValidateRequiredFields(*packed,current_owner_cookie,header.expected_geometry_generation,header.damage_revision))return reject(why);
    xr_vector<u8> replaced;replaced.assign(count,u8(0));
    for(const auto& c:packed->children){
        if(c.material_source_child>=originals.size())return reject("material_source_child");
        if(replaced[c.child_index])return reject("duplicate_child_slot");
        replaced[c.child_index]=true;
    }
    auto* stage=new RuntimeGeometryStage();
#if RENDER==R_R4
    stage->burnSource=burnSource?std::move(burnSource):LL_PainterBurnStageSource();
#endif
    stage->owner_model=this;stage->owner_cookie=current_owner_cookie;
    stage->expected_generation=header.expected_geometry_generation;stage->damage_revision=header.damage_revision;
    stage->separation_packet=header.separation_packet;
    stage->flight_preview=header.flight_preview;stage->flight_detail=header.flight_detail;
    stage->bone_count=LL_BoneCount();stage->registers=HW.Caps.geometry.dwRegisters;
    RuntimePreparedCpuCaptureRig(*stage,header);
    stage->originals_count=originals.size();stage->count=count;
    stage->device=HW.pDevice;stage->device->AddRef();
    stage->packed=packed;stage->painter_tissue=RuntimeQueueTissueField(packed->tissue_field,tissue_textures);
    stage->required_fields=RuntimeQueueRequiredFields(packed,stage->painter_tissue,current_owner_cookie,
        header.expected_geometry_generation,header.damage_revision,
        m_runtime_geometry&&m_runtime_geometry->required_fields&&bool(m_runtime_geometry->required_fields->wallmark_ownership));
    for(size_t n=originals.size();n<count;++n)if(!replaced[n])stage->published_gap=true;
    stage->children.reserve(packed->children.size());
    for(const auto& src:packed->children){
        std::unique_ptr<RuntimeStagedChild> child(new RuntimeStagedChild());
        child->child_index=src.child_index;child->material_source_child=src.material_source_child;
        child->required_fields=stage->required_fields;child->skin_clip=src.skin_clip;child->skin_owned=src.skin_owned;
        child->analytic=bool(packed->analytic_channels)&&src.material!=0;
        // Same flags AppendPackedRuntimeChildren gives the request child.
        child->remove_surface=src.vertices.empty()&&src.indices.empty();
        child->receives_wallmarks=!child->remove_surface && !src.material && !PainterMaskPass::IsBox(src); // maskpass: no marks on a coverage box
        if(src.material){
            child->painter_tissue=stage->painter_tissue;
            child->shader=cavity_shader?cavity_shader:"";
            child->texture=tissue_textures?*(*tissue_textures)[src.material]:(cavity_texture?cavity_texture:"");
        }
        // LL_PrepareRuntimeGeometryInternal's in-loop checks, evaluated by Run() in order.
        child->removed_valid=!child->remove_surface || src.child_index<originals.size();
        child->material_valid=child->remove_surface ||
            fast_dynamic_cast<dxRender_Visual*>(originals[src.material_source_child])!=nullptr;
        child->source=&src;
        stage->children.push_back(std::move(child));
    }
    stage->timing.stage_ms=RuntimeStageMs(start);
    if(!runtime_stage_executor.Submit(stage)){stage->Release();return nullptr;} // outcome unavailable: synchronous path
    outcome=RuntimeStageOutcome::staged;
    return stage;
}
namespace {
// r_painter_publish 2: rebuild one adopted child through the synchronous
// BuildRuntime + BuildRuntimeFaceLists + FinalizeProjectionSource path at the
// same barrier and compare everything, including GPU buffer bytes.
bool runtime_stage_verify_child(CKinematics* owner,const RuntimeStagedChild& staged,CSkeletonX_ST& adopted,
    const dxRender_Visual& material,const xr_vector<xr_vector<CBoneData::FacesVec>>& faces,char* diff,size_t diff_size)
{
    const gfy_packed::Child& src=*staged.source;
    RuntimeSkinChild data;
    data.child_index=src.child_index;data.material_source_child=src.material_source_child;
    data.remove_surface=false;data.receives_wallmarks=staged.receives_wallmarks;
    if(staged.painter_tissue)data.tissue_field=staged.painter_tissue->field;
    if(staged.shader.size())data.shader=staged.shader.c_str();
    if(staged.texture.size())data.texture=staged.texture.c_str();
    data.vertices.resize(src.vertices.size());data.indices.assign(src.indices.begin(),src.indices.end());
    for(size_t i=0;i<src.vertices.size();++i){
        const auto& v=src.vertices[i];auto& out=data.vertices[i];
        out.position.set(v.surface[0],v.surface[1],v.surface[2]);out.normal.set(v.surface[3],v.surface[4],v.surface[5]);
        out.tangent.set(v.surface[6],v.surface[7],v.surface[8]);out.binormal.set(v.surface[9],v.surface[10],v.surface[11]);
        out.uv.set(v.surface[12],v.surface[13]);
        for(unsigned k=0;k<4;++k){out.bone[k]=v.bone[k];out.weight[k]=v.weight[k];}
    }
    const u16 bones=owner->LL_BoneCount();
    xr_vector<xr_vector<CBoneData::FacesVec>> table(bones);
    for(auto& row:table)row.resize(size_t(src.child_index)+1);
    BuildRuntimeFaceLists(data.vertices,data.indices,bones,
        [&](u16 bone,u32 face){table[bone][src.child_index].push_back(u16(face));});
    for(u16 b=0;b<bones;++b)if(table[b][src.child_index]!=faces[b][src.child_index]){xr_strcpy(diff,diff_size,"face_rows");return false;}
    std::unique_ptr<CSkeletonX_ST> reference(new CSkeletonX_ST());
    if(!reference->BuildRuntime(data,owner,material)){xr_strcpy(diff,diff_size,"reference_build");return false;}
    if(reference->receives_wallmarks)reference->FinalizeProjectionSource(&table,true);
    return adopted.RuntimeMatches(*reference,diff,diff_size);
}
}
bool CKinematics::LL_InstallStagedRuntimeGeometry(IRuntimeGeometryStage* handle,const RuntimeSkinUpdate& header,
    u64 current_owner_cookie)
{
    if(!LL_PrepareStagedRuntimeGeometry(handle,header,current_owner_cookie,nullptr,nullptr))return false;
    auto* stage=static_cast<RuntimeGeometryStage*>(handle);
    LARGE_INTEGER start;QueryPerformanceCounter(&start);
    const bool committed=LL_CommitPreparedRuntimeGeometry(current_owner_cookie);
    if(!committed)LL_DiscardPreparedRuntimeGeometry();
    const float elapsed=RuntimeStageMs(start);
    stage->timing.commit_ms+=elapsed;stage->timing.install_ms+=elapsed;
    return committed;
}
bool CKinematics::LL_PrepareStagedRuntimeGeometry(IRuntimeGeometryStage* handle,const RuntimeSkinUpdate& header,
    u64 current_owner_cookie,IRenderVisual* donor_visual,const Fmatrix* recipient_xform)
{
    auto reject=[&](const char* reason){Msg("[runtime_geometry_reject] stage=owner reason=%s cookie=%llu expected_generation=%llu actual_generation=%llu revision=%llu",reason,current_owner_cookie,header.expected_geometry_generation,m_render_geometry_generation,header.damage_revision);return false;};
    CKinematics* donor=this;
    if(donor_visual||recipient_xform){
        if(!donor_visual||!recipient_xform)return reject("stage_donor_pair");
        donor=fast_dynamic_cast<CKinematics*>(donor_visual);
        if(!donor||donor==this||donor->LL_BoneCount()!=LL_BoneCount())return reject("stage_donor_rig");
    }
    auto* stage=static_cast<RuntimeGeometryStage*>(handle);
    if(!stage || !stage->Ready())return reject("stage_not_ready");
    LARGE_INTEGER start,phase;QueryPerformanceCounter(&start);phase=start;
    auto& timing=stage->timing;
    auto lap=[&](float& field){LARGE_INTEGER now;QueryPerformanceCounter(&now);LARGE_INTEGER from=phase;phase=now;field+=RuntimeStageMs(from);};
    auto finish=[&](bool committed){timing.install_ms=RuntimeStageMs(start);return committed;};
    if(stage->installed)return finish(reject("stage_reused"));
    stage->installed=true;
    // Admission again: the generation, owner, revision or rig may have moved
    // while the stage was prepared (separation commit, despawn, device reset).
    if(stage->owner_model!=this || stage->owner_cookie!=header.owner_cookie ||
       stage->expected_generation!=header.expected_geometry_generation || stage->damage_revision!=header.damage_revision ||
       stage->separation_packet!=header.separation_packet || stage->flight_preview!=header.flight_preview ||
       stage->flight_detail!=header.flight_detail || stage->bone_count!=LL_BoneCount())
        return finish(reject("stage_identity"));
    xr_vector<IRenderVisual*> originals;
    if(const char* why=LL_AdmitRuntimeHeader(header,current_owner_cookie,!stage->children.empty(),originals))return finish(reject(why));
    if(originals.size()!=stage->originals_count)return finish(reject("stage_identity"));
    if(stage->device!=HW.pDevice)return finish(reject("stage_device"));
    if(stage->required_fields&&!stage->required_fields->Register(HW.pDevice))return finish(reject(stage->required_fields->RefusalDetail(HW.pDevice).c_str()));
    if(!stage->ok){
        if(stage->child_message[0])Msg("%s",stage->child_message);
        return finish(reject(stage->reason?stage->reason:"stage_failed"));
    }
    lap(timing.admit_ms);
    const size_t count=stage->count;
    xr_vector<u8> replaced(count,u8(0)),removed(count,u8(0)),accepts(count,u8(1)),sources_done(count,u8(0));
    for(const auto& child:stage->children){replaced[child->child_index]=1;accepts[child->child_index]=child->receives_wallmarks?1:0;}
    std::unique_ptr<RuntimeGeometryPacket> next(new RuntimeGeometryPacket());
    next->owner_cookie=current_owner_cookie;next->damage_revision=header.damage_revision;
    next->geometry_generation=m_render_geometry_generation+(header.flight_detail?0:1);
    next->separation_packet=header.separation_packet;
    next->flight_preview=header.flight_preview;next->flight_detail=header.flight_detail;
    next->required_fields=stage->required_fields;
    // Repeat hits on a detached piece carry its birth atlas unchanged. The removal
    // field is still new. Only the tear textures can pass from the current packet.
    if (m_runtime_geometry && m_runtime_geometry->owner_cookie == current_owner_cookie) {
        const auto& prev = m_runtime_geometry->required_fields;
        const auto& curr = next->required_fields;
        if (prev && curr && curr->mask_piece && curr->tear_atlas && !curr->tear_gpu &&
            curr->epoch == prev->epoch && curr->tear_atlas == prev->tear_atlas &&
            prev->tear_gpu && prev->tear_gpu->built_device == HW.pDevice)
            curr->tear_gpu = prev->tear_gpu;
    }
    next->children=originals;next->children.resize(count,nullptr);
    next->faces.swap(stage->faces); // [bone][slot], replaced rows built by Run()
    // Borrowed originals keep their original rows (bone data is read at a barrier only).
    for(u16 b=0;b<LL_BoneCount();++b){
        const auto& previous=LL_GetData(b).child_faces;
        for(size_t child=0;child<previous.size()&&child<count;++child)
            if(!replaced[child])next->faces[b][child]=previous[child];
    }
    // bloodfix (2026-11-13): the body's baked blood (a piece: its donor's) is bound into the new materials while they are
    // created (PainterRemovalMaterial::liveBlood): the new children draw the body's blood from their first frame. A body gets
    // its blood instance and the empty table texture with its first publication, so blood that lands later needs no
    // material change either.
    PainterRemovalMaterial::LiveBlood live_blood;
    struct LiveBloodScope {const PainterRemovalMaterial::LiveBlood* previous=PainterRemovalMaterial::liveBlood();~LiveBloodScope(){PainterRemovalMaterial::liveBlood()=previous;}} live_blood_scope;
    if(PainterBloodBake::enabled()){
        try{
            if(!donor->m_blood_cells)donor->m_blood_cells=std::make_shared<PainterBloodInstance>();
            auto& blood_gpu=donor->m_blood_cells->shared->gpu;
            if(blood_gpu.texture()){live_blood.table=blood_gpu.cellTexture;live_blood.clock=&PainterBloodBake::clock();live_blood.target=&PainterBloodBake::target();PainterRemovalMaterial::liveBlood()=&live_blood;}
        }catch(...){}
    }
    RuntimeShaderReuse reuse;reuse.seed(m_runtime_geometry,&originals);
    xr_vector<CSkeletonX_ST*> adopted(count,nullptr);
    for(auto& child:stage->children){
        if(child->remove_surface){next->children[child->child_index]=nullptr;removed[child->child_index]=1;continue;}
        auto* material=fast_dynamic_cast<dxRender_Visual*>(originals[child->material_source_child]);
        if(!material)return finish(reject("material_visual_type"));
        std::unique_ptr<CSkeletonX_ST> skin(new CSkeletonX_ST());
        if(!skin->AdoptRuntime(*child,this,*material,reuse))return finish(reject("child_build_see_child_reason"));
#if RENDER==R_R4
        // A staged child copied this exact packet's original_faces in AdoptRuntime.
        // Associate its new immutable copy with the finished GPU table at adoption.
        auto map=std::move(child->burnFaces);
        if(!map && donor->m_burn_binding && donor->m_burn_binding->hasRenderedBurn && donor->m_burn_binding->bank && donor->m_burn_binding->source)
            map=PainterBurnGpu::buildFaces(HW.pDevice,PainterBurnGpu::geometryOf(donor->m_burn_binding->source),
                skin->PainterBurnSourceChild(),skin->PainterBurnRemap().get());
        if(map){map->original=skin->PainterBurnRemap();next->burnFaces.emplace(skin.get(),std::move(map));}
#endif
        sources_done[child->child_index]=child->projection_attempted?1:0;adopted[child->child_index]=skin.get();
        next->children[child->child_index]=skin.get();next->owned.push_back(skin.release());
    }
    timing.shader_reused=reuse.reused;timing.shader_compiled=reuse.compiled;
    // Every built child now holds its own reference to the packet buffers:
    // count them once (stats_manager decrements on the last reference) and drop
    // the stage's references so that last reference is a child's.
    if(stage->vb)HW.stats_manager.increment_stats_vb(stage->vb);
    if(stage->ib)HW.stats_manager.increment_stats_ib(stage->ib);
    _RELEASE(stage->vb);_RELEASE(stage->ib);
    lap(timing.wrap_ms);
    Msg("[runtime_geometry] phase=prepared_cpu_cache revision=%llu hits=%u vertices=%u cache_ms=%.3f retained_bytes=%llu",
        stage->damage_revision,stage->prepared_hits,stage->prepared_vertices,stage->prepared_cache_ms,u64(stage->prepared_cache_bytes));
    if(r_painter_publish==2){
        timing.verify_result=1;
        for(auto& child:stage->children){
            if(child->remove_surface)continue;
            char diff[64]="";
            CSkeletonX_ST* skin=adopted[child->child_index];
            auto* material=fast_dynamic_cast<dxRender_Visual*>(originals[child->material_source_child]);
            if(!runtime_stage_verify_child(this,*child,*skin,*material,next->faces,diff,sizeof(diff))){
                timing.verify_result=0;
                Msg("! [runtime_geometry_verify] cookie=%llu revision=%llu child=%u result=mismatch field=%s",
                    current_owner_cookie,header.damage_revision,child->child_index,diff);
            }
        }
        Msg("[runtime_geometry_verify] cookie=%llu revision=%llu children=%u result=%s",current_owner_cookie,
            header.damage_revision,timing.children,timing.verify_result?"identical":"mismatch");
        lap(timing.verify_ms);
    }
    xr_vector<IRenderVisual*> published;SkeletonWMVec rebuilt;
    if(const char* why=LL_FinishRuntimeGeometryPrepare(next.get(),count,removed,accepts,&sources_done,donor,recipient_xform,published,rebuilt))
        return finish(reject(why));
    timing.marks=u32(rebuilt.size());
    lap(timing.wallmarks_ms);
    std::unique_ptr<RuntimeGeometryPrepared> prepared(new RuntimeGeometryPrepared());
    prepared->packet=next.release();prepared->wallmarks.swap(rebuilt);
    prepared->prior_packet=m_runtime_geometry;
    prepared->published.swap(published);
    m_prepared_runtime_geometry=prepared.release();
    // No visibility/physics mutation here. A sever caller can prepare every
    // parent and child before its existing all-or-nothing commit checks.
    return finish(true);
}
unsigned CKinematics::LL_WarmRuntimeShaders(LPCSTR cavity_shader,LPCSTR cavity_texture,const xr_vector<shared_str>* tissue_textures,unsigned budget,const std::shared_ptr<const painter_detail::TissueField>& tissue_field)
{
    RestoreBloodMaterials(); // main-thread warm cache accepts original handles only
    RuntimeQueueTissueField(tissue_field,tissue_textures);
    // Bind-time hook (main thread). The OGF originals already hold the
    // exterior handles (recorded here, so an exterior child never compiles);
    // the cavity pairs (one shader, up to 64 tissue textures) are queued and
    // compiled by CRender::OnFrame one per frame, before the first hit. Runtime
    // children take hud from their material original and skinning 4
    // (AdoptRuntime), so the queued keys use the originals' hud flag and 4.
    const auto& visible=m_runtime_geometry?m_geometry_original_children:children;
    const auto& invisible=m_runtime_geometry?m_geometry_original_invisible:children_invisible;
    bool hud_flag=false,hud_known=false;
    for(const auto* list:{&visible,&invisible})for(auto* visual:*list){
        auto* v=fast_dynamic_cast<dxRender_Visual*>(visual);if(!v)continue;
        if(!hud_known){hud_flag=v->hud;hud_known=true;}
        RuntimeShaderCache::Insert(v->dbg_shader,v->dbg_texture,v->hud,v->skinning,v->shader);
        // An original loaded at another skinning mode: its exterior runtime children need the four-influence variant.
        if(v->skinning!=4)RuntimeShaderCache::RequestWarm(v->dbg_shader_def,v->dbg_texture_def,v->hud,4);
    }
    if(cavity_shader&&*cavity_shader){
        const shared_str shader(cavity_shader);
        if(cavity_texture&&*cavity_texture)RuntimeShaderCache::RequestWarm(shader,shared_str(cavity_texture),hud_flag,4);
        if(tissue_textures)for(const auto& texture:*tissue_textures)if(texture.size())RuntimeShaderCache::RequestWarm(shader,texture,hud_flag,4);
    }
    return RuntimeShaderCache::Warm(budget);
}
#else
IRuntimeGeometryStage* CKinematics::LL_StageRuntimeGeometry(const RuntimeSkinUpdate&,const std::shared_ptr<const gfy_packed::Mesh>&,
    LPCSTR,LPCSTR,const xr_vector<shared_str>*,u64,int& outcome,std::shared_ptr<const painter_burn::SourceFaces>){outcome=3;return nullptr;}
bool CKinematics::LL_InstallStagedRuntimeGeometry(IRuntimeGeometryStage*,const RuntimeSkinUpdate&,u64){return false;}
bool CKinematics::LL_PrepareStagedRuntimeGeometry(IRuntimeGeometryStage*,const RuntimeSkinUpdate&,u64,IRenderVisual*,const Fmatrix*){return false;}
unsigned CKinematics::LL_WarmRuntimeShaders(LPCSTR,LPCSTR,const xr_vector<shared_str>*,unsigned,const std::shared_ptr<const painter_detail::TissueField>&){return 0;}
void RuntimeGeometryStageShutdown(){}
#endif
