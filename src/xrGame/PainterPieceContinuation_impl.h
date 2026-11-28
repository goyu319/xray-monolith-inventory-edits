// body_hit_outcomes_v1
#pragma once
#include "painter_voxel/PainterPieceContinuation.h"
#include <chrono>
#include "PainterDamagedFragmentProducer_impl.h"
#include "CharacterPhysicsSupport.h"
#include "ai/stalker/ai_stalker.h"
#include "ai_space.h"
#include "level_graph.h"
#include "../xrServerEntities/script_engine.h"
#include "../Include/xrRender/RuntimePackedBridge.h"
#include "../Include/xrRender/RuntimeGeometryStage.h"
#include "PainterWoundEvent.h"
#include "PainterCommitParts.h" // piececommit
#include "PainterMeleeEvent.h" // melee (value 3): knife / blade / blunt hits on pieces and stumps
#include "../Include/xrRender/PainterLogVerbose.h"
#include <map>
#include <set>
#include <memory>
#include <cmath>
#include "PainterBurnHost.h"
void PainterSubmitLocalThermal(CObject&,const SHit&);
#include "PainterCommittedEffects_impl.h"
#include "PainterCranialRemnants_impl.h"

namespace painter_piece_continuation {
namespace {
// Ready-hook link of a part session (same pattern as the body's PainterVoxelReadyLink): the pool thread queues the
// part's barrier callback into the pending list; forget() clears the link under the sections first.
struct PartReadyLink {CCharacterPhysicsSupport* source_support=nullptr;CPhysicsSkeletonObject* fragment=nullptr;};
inline void hook_part_session(gfy_live::Session& session,const std::shared_ptr<PartReadyLink>& link){
    session.set_ready_hook([link]{
        xrCriticalSectionGuard guard(&Device.seqParallelBeforRenderPendingCS);
        xr_delegate<void()> callback;
        if(link->source_support)callback=xr_make_delegate(link->source_support,&CCharacterPhysicsSupport::ApplyPainterPieceContinuation);
        else if(link->fragment)callback=xr_make_delegate(link->fragment,&CPhysicsSkeletonObject::ApplyPainterPieceContinuation);
        else return;
        if(std::find(Device.seqParallelBeforRenderPending.begin(),Device.seqParallelBeforRenderPending.end(),callback)==Device.seqParallelBeforRenderPending.end())
            Device.seqParallelBeforRenderPending.push_back(callback);});
}
#include "FlightDetail_impl.h"
struct Owner {
    std::unique_ptr<flight_detail::Pending> flight;
    u64 flight_attempted_revision=0;
    CPhysicsShellHolder* object=nullptr;
    CCharacterPhysicsSupport* source_support=nullptr;
    CPhysicsSkeletonObject* fragment=nullptr;
    std::shared_ptr<PartReadyLink> ready_link;
    IKinematics* visual=nullptr;
    CPhysicsShell* shell=nullptr;
    std::shared_ptr<const gfy_live::PieceSeed> seed;
    std::shared_ptr<const gfy_packed::Mesh> birth; // the piece mesh as committed (the session's baseline)
    std::shared_ptr<gfy_live::Session> session;
    std::shared_ptr<gfy_live::SeparationPreparation> job;
    gfy_live::SeparationInput input;
    gfy_live::Identity identity;
    xr_vector<RuntimeSkinBone> rig;
    xr_vector<shared_str> textures;
    painter_committed_effects::Queue committed_effects;
    // Off-thread publication (as live bodies): the renderer prepares stage_out's geometry, it installs at a later barrier.
    IRuntimeGeometryStage* stage=nullptr;std::shared_ptr<const gfy_live::Output> stage_out;
    u64 geometry=0,sequence=0,first_revision=0,sever_revision=0,checked_revision=0;
    u64 damage_revision=0;u32 split_imminent_at=0; // last published damage; a stale check found this piece apart
    u32 retry_at=0;
    u32 cancel_retries=0; // R33: re-checks after a split canceled at commit, bounded
    u16 original_children=0;
    DWORD thread=0;
    bool queued=false,active=false;
    CGID collision_group=0;bool grouped=false; // R31: the split's shared collision group (stump and pieces never push each other apart)
    void drop_stage(){if(stage){stage->Cancel();stage->Release();stage=nullptr;}stage_out.reset();}
    Owner()=default;Owner(const Owner&)=delete;Owner& operator=(const Owner&)=delete;
    ~Owner(){drop_stage();}
};
std::map<u16,std::unique_ptr<Owner>> owners;
struct SplitBatch {
    std::shared_ptr<painter_damaged_fragment::Prepared> request;
    std::shared_ptr<const gfy_live::SeparationOutput> result;
    xr_vector<std::shared_ptr<painter_damaged_fragment::Stage>> stages;
    xr_vector<u64> child_cookies;
    u32 started_at=0;
    std::shared_ptr<painter_damaged_fragment::RenderTicket> stump_render; // staged stump packet (first cut: batch.render)
};
std::map<u16,SplitBatch> splits;
// splitfix2: a re-split part is light (fitted to what is left) and was given whole-body hit impulses under the engine's 150 m/s
// limit: a grenade threw it through walls and pulled its joints apart (the skin across them stretched into a rod). Every shell a
// continuation split creates keeps its speed under 12 m/s (the angular limit stays the engine's).
constexpr float resplit_linear_limit=12.f;
u64 next_piece_cookie=0x5049454345000001ull;
Owner* find(const CPhysicsShellHolder& object){
    auto it=owners.find(object.ID());
    return it!=owners.end()&&it->second->object==&object?it->second.get():nullptr;
}
bool separation_allowed(CPhysicsShellHolder& object){
    if(gfy_live::damage_mode_value()==1)return false; // modes: Bullet holes only - no piece ever separates
    ::luabind::functor<bool> policy;
    if(auto* npc=smart_cast<CAI_Stalker*>(&object))
        return !npc->g_Alive()&&
            ai().script_engine().functor("_G.CPainterVoxelSeparationAllowed",policy)&&
            policy(npc->lua_game_object());
    return smart_cast<CPhysicsSkeletonObject*>(&object)&&
        ai().script_engine().functor("_G.CPainterVoxelFragmentSeparationAllowed",policy)&&
        policy();
}
bool valid_owner(Owner& entry){
    if(!entry.active)return false;
    auto& object=*entry.object;
    auto* k=smart_cast<IKinematics*>(object.Visual());
    if(GetCurrentThreadId()!=entry.thread||object.getDestroy()||k!=entry.visual||
       !k||k->LL_RenderGeometryGeneration()!=entry.geometry||
       object.PPhysicsShell()!=entry.shell||!entry.shell||
       !entry.shell->isFullActive()||entry.rig.size()!=k->LL_BoneCount())return false;
    if(auto* npc=smart_cast<CAI_Stalker*>(&object))return !npc->g_Alive();
    return smart_cast<CPhysicsSkeletonObject*>(&object)!=nullptr;
}
// Physical parent of every bone in the object's current shell: the nearest skeleton ancestor with an element here.
bool physical_parents(IKinematics* k,CPhysicsShell* shell,std::vector<u16>& parents){
    if(!k||!shell||k->LL_BoneCount()>64)return false;
    std::vector<bool> element(k->LL_BoneCount(),false);
    for(u16 b=0;b<k->LL_BoneCount();++b){auto* e=shell->get_Element(b);element[b]=e&&e->m_SelfID==b;}
    if(!shell->get_ElementsNumber()||shell->get_ElementByStoreOrder(0)->m_SelfID>=k->LL_BoneCount()||
       !element[shell->get_ElementByStoreOrder(0)->m_SelfID])return false;
    parents.assign(k->LL_BoneCount(),BI_NONE);
    for(u16 b=0;b<k->LL_BoneCount();++b){u16 p=k->LL_GetData(b).GetParentID();unsigned steps=0;
        while(p!=BI_NONE&&p<k->LL_BoneCount()&&steps++<k->LL_BoneCount()){if(element[p]){parents[b]=p;break;}p=k->LL_GetData(p).GetParentID();}
        if(p!=BI_NONE&&steps>=k->LL_BoneCount())return false;}
    return true;
}
// Every immutable field carries the identity its publication is admitted under.
void restamp(gfy_packed::Mesh& mesh,u64 cookie,u64 generation,u64 revision){
    if(mesh.analytic_channels){
        auto field=std::make_shared<painter_detail::AnalyticChannelField>(*mesh.analytic_channels);
        field->identity.cookie=cookie;field->identity.generation=generation;field->identity.revision=revision;
        mesh.analytic_channels=std::move(field);
    }
    if(mesh.skin_ownership){
        auto field=std::make_shared<painter_detail::SkinOwnershipField>(*mesh.skin_ownership);
        field->identity.cookie=cookie;field->identity.generation=generation;field->identity.revision=revision;
        mesh.skin_ownership=std::move(field);
    }
}
#include "FlightAdopt_impl.h"
void cancel_split(Owner& entry){
    auto it=splits.find(entry.object->ID());
    if(it==splits.end())return;
    for(auto& stage:it->second.stages){
        if(stage->client)abandon(*stage->client);
        stage->canceled=true;
        painter_damaged_fragment::destroy_child(smart_cast<CAI_Stalker*>(entry.object),*stage);
        painter_damaged_fragment::named.erase(stage->name);
    }
    splits.erase(it);
    // R33: a split canceled at commit left this part cut but whole until its next wound (bodies: PainterVoxelLive_impl.h).
    // Check again on a fresh snapshot after 1 s, up to six times.
    if(entry.cancel_retries<6){++entry.cancel_retries;
        const u64 newest=std::max(entry.sever_revision,entry.damage_revision);
        if(newest&&entry.checked_revision>=newest)entry.checked_revision=newest-1;
        entry.retry_at=Device.dwTimeGlobal+1000;
        Msg("[painter_piece_continuation] id=%u phase=cancel_retry_scheduled retry=%u delay_ms=1000 t=%u",entry.object->ID(),entry.cancel_retries,Device.dwTimeGlobal);}
}
void split_canceled(Owner& entry,const char* why){
    Msg("[painter_piece_continuation] id=%u phase=split_canceled reason=%s",entry.object->ID(),why);cancel_split(entry);}
bool start_split(Owner& entry,std::shared_ptr<const gfy_live::SeparationOutput> result,std::shared_ptr<painter_flight::Group> flight){
    painter_flight_perf::Scope flightTime("piece_spawn");
    const auto& cpu=*result;
    // splitfix: every refusal names itself (the caller's split_refused summary line follows it)
    const u64 refusedRevision=cpu.stamp.revision;
    auto refuse=[&](const char* why){Msg("[painter_piece_continuation] id=%u phase=split_refused reason=%s revision=%llu",entry.object->ID(),why,refusedRevision);return false;};
    if(cpu.pieces.size()<2||cpu.pieces.size()>13)return refuse("piece_count");
    if(painter_damaged_fragment::named.size()+cpu.pieces.size()-1>256)return refuse("piece_names_full");
    painter_commit_parts::part("ss_fitted"); // piececommit
    auto req=std::make_shared<painter_damaged_fragment::Prepared>();
    req->stamp=cpu.stamp;req->current_owner=entry.identity;
    req->gore=cpu.gore;
    req->flight=std::move(flight);
    req->expected_generation=entry.geometry;
    req->publication_revision=cpu.stamp.revision;
    req->continuation_input=entry.input;
    req->source_id=entry.object->ID();
    req->source_visual=entry.object->cNameVisual();
    req->source_world=entry.object->XFORM();
    {float deviation=0.f;if(!painter_damaged_fragment::producer::orthonormal(req->source_world,deviation))return refuse("source_world");}
    std::set<u32> components;bool found=false;
    for(const auto& piece:cpu.pieces){
        if(!piece.component||!piece.cells||!piece.seed||!components.insert(piece.component).second)return refuse("piece_identity");
        if(piece.component==cpu.source_component){
            found=true;req->stump=piece.mesh;req->stump_seed=piece.seed;
            req->stump_root_bone=piece.root_bone;req->stump_pose_anchors=piece.pose_anchors;
            painter_damaged_fragment::producer::flesh_gear(piece,req->stump_flesh_gear); // gearbody
            if(!painter_damaged_fragment::producer::fitted(piece,req->stump_physics,
                req->stump_visible_bones,u16(entry.rig.size()))){
                Msg("[painter_piece_continuation] id=%u phase=fitted_refused part=stump reason=%s",entry.object->ID(),painter_damaged_fragment::producer::fitted_refusal());return false;}
        }else{
            auto& fragment=req->pieces.emplace_back();
            if(req->flight)fragment.flight_visual=piece.flight_visual;
            fragment.component_id=piece.component;fragment.mesh=piece.mesh;fragment.seed=piece.seed;
            fragment.ocular_remnant=piece.ocular_remnant;
            fragment.burst_chunk=piece.burst_chunk; // chunknosplash
            fragment.destroy_on_commit=piece.ocular_remnant&&cpu.gore.eye_remnants_mode==0;
            fragment.root_bone=piece.root_bone;fragment.anchor=piece.root_bone;
            fragment.pose_anchors=piece.pose_anchors;
            painter_damaged_fragment::producer::flesh_gear(piece,fragment.flesh_gear); // gearbody
            if(!painter_damaged_fragment::producer::fitted(piece,fragment.fitted,
                fragment.visible_bones,u16(entry.rig.size()))){
                Msg("[painter_piece_continuation] id=%u phase=fitted_refused part=piece reason=%s",entry.object->ID(),painter_damaged_fragment::producer::fitted_refusal());return false;}
        }
    }
    if(!found)return refuse("no_source_part");
    painter_commit_parts::part("ss_decisions");
    if(!painter_damaged_fragment::producer::decisions(cpu,entry.original_children,*req))return refuse("child_ownership");
    painter_commit_parts::part("ss_packets");
    // piececommit: the packets are built from the stump and piece meshes MOVED in (they were copied): each is replaced by its
    // packet below, or the request is dropped on a refusal - nothing reads them in between. Same packets, same refusals.
    std::vector<gfy_packed::Mesh> raw;raw.reserve(1+req->pieces.size());raw.push_back(std::move(req->stump));
    std::vector<std::vector<part_child_ownership::Decision>> decisions{req->stump_children};
    for(auto& piece:req->pieces){raw.push_back(std::move(piece.mesh));decisions.push_back(piece.children);}
    if(painter_damaged_fragment::has_uncertified_original_accessory(decisions))return refuse("unsupported_accessory_bone_palette");
    if(const char* issue=part_child_ownership::validate_part_packets(raw,req->original_roles,decisions))return refuse(issue);
    auto packets=part_child_ownership::build_part_packets(std::move(raw),req->original_roles,decisions); // piececommit: taken over
    if(packets.size()!=raw.size())return refuse("packet_count");
    req->stump=std::move(packets[0]);
    for(std::size_t i=0;i<req->pieces.size();++i)req->pieces[i].mesh=std::move(packets[i+1]);
    painter_commit_parts::part("ss_bones");
    auto* k=entry.visual;
    {xrCriticalSectionGuard guard(&k->UCalc_Mutex);
        k->CalculateBones_Invalidate();k->CalculateBones(TRUE);
        req->captured_render_palette.resize(entry.rig.size());
        for(u16 b=0;b<entry.rig.size();++b)req->captured_render_palette[b]=k->LL_GetBoneInstance(b).mRenderTransform;
        for(auto& fragment:req->pieces){
            fragment.anchor_model=k->LL_GetBoneInstance(fragment.anchor).mTransform;
            fragment.anchor_render=k->LL_GetBoneInstance(fragment.anchor).mRenderTransform;
            float deviation=0.f;painter_damaged_fragment::producer::orthonormal(fragment.anchor_model,deviation);
            painter_damaged_fragment::producer::orthonormal(fragment.anchor_render,deviation);
        }
    }
    painter_commit_parts::part("ss_cover");
    u64 missing=0;
    if(!painter_damaged_fragment::pose_covered(req->stump,req->stump_physics,
        req->stump_pose_anchors,u16(entry.rig.size()),missing))return refuse("stump_pose_ownership");
    for(const auto& piece:req->pieces){
        if(!painter_damaged_fragment::valid_piece(piece,u16(entry.rig.size()))){
            const char* why=painter_damaged_fragment::piece_problem(piece,u16(entry.rig.size()));return refuse(why?why:"piece_invalid");}
        if(!painter_damaged_fragment::pose_covered(piece.mesh,piece.fitted,
            piece.pose_anchors,u16(entry.rig.size()),missing))return refuse("fragment_pose_ownership");}
    painter_commit_parts::part("ss_batch");
    SplitBatch batch;batch.request=req;batch.result=std::move(result);
    batch.started_at=Device.dwTimeGlobal;
    batch.child_cookies.resize(req->pieces.size(),0);
    for(std::size_t i=0;i<req->pieces.size();++i){
        if(req->pieces[i].destroy_on_commit)continue;
        auto stage=std::make_shared<painter_damaged_fragment::Stage>();
        stage->request=req;stage->index=i;stage->pose_source=entry.shell;
        stage->pose_owner=entry.object;
        stage->name="painter_damaged_"+std::to_string(++painter_damaged_fragment::serial);
        if(painter_damaged_fragment::serial==u64(-1))return refuse("piece_serial");
        batch.child_cookies[i]=++next_piece_cookie;
        if(req->pieces[i].flight_visual)stage->render.proxy=painter_flight::proxy_packet(*req->pieces[i].flight_visual,req->original_roles.size(),batch.child_cookies[i],req->publication_revision);
        restamp(req->pieces[i].mesh,batch.child_cookies[i],0,req->publication_revision);
        painter_damaged_fragment::named.emplace(stage->name,stage);
        batch.stages.push_back(stage);
    }
    restamp(req->stump,entry.identity.spawn_cookie,entry.geometry,req->publication_revision);
    // The packets move into render tickets, prepared off-thread before the commit like a body's first cut: the
    // synchronous adapter refuses wound-walled packets (analytic channels), so every re-split was canceled.
    batch.stump_render=std::make_shared<painter_damaged_fragment::RenderTicket>();
    batch.stump_render->mesh=std::make_shared<const gfy_packed::Mesh>(std::move(req->stump));
    for(auto& stage:batch.stages)stage->render.mesh=std::make_shared<const gfy_packed::Mesh>(std::move(req->pieces[stage->index].mesh));
    auto inserted=splits.emplace(entry.object->ID(),std::move(batch));
    if(!inserted.second)return refuse("split_pending");
    painter_commit_parts::part("ss_spawn");
    for(const auto& stage:inserted.first->second.stages){
        const auto& part=req->pieces[stage->index];
        const u32 vertex=ai().level_graph().vertex_id(entry.object->Position());
        CSE_Abstract* se=Level().spawn_item("ph_skeleton_object",entry.object->Position(),vertex,u16(-1),true);
        auto* po=smart_cast<CSE_ALifePHSkeletonObject*>(se);
        if(!po){if(se)F_entity_Destroy(se);cancel_split(entry);return refuse("piece_spawn_entity");}
        po->set_visual(*req->source_visual);
        po->set_name_replace(stage->name.c_str());
        po->source_id=BI_NONE;
        po->saved_bones.root_bone=part.root_bone;
        po->saved_bones.bones_mask=part.visible_bones;
        po->_flags.set(CSE_PHSkeleton::flActive,FALSE);
        po->_flags.set(CSE_PHSkeleton::flNotSave,TRUE); // R30: the visual is runtime geometry a save cannot bring back (net_Spawn failed at load); never written to one
        req->source_world.getHPB(po->o_Angle);
        po->o_Position=req->source_world.c;
        NET_Packet packet;po->Spawn_Write(packet,TRUE);
        Level().Send(packet,net_flags(TRUE));
        F_entity_Destroy(se);
    }
    return true;
}
bool poll_split(Owner& entry){
    auto it=splits.find(entry.object->ID());
    if(it==splits.end())return false;
    painter_flight_perf::Scope flightTime("piece_commit");
    auto& batch=it->second;auto& object=*entry.object;
    auto& req=*batch.request;auto* k=entry.visual;
    painter_commit_parts::part("ps_check"); // piececommit
    const bool expired=painter_damaged_fragment::fragment_spawn_expired(
        Device.dwTimeGlobal,batch.started_at,painter_damaged_fragment::spawn_timeout_ms);
    if(!valid_owner(entry)||!separation_allowed(object)||expired||(req.flight&&!k->LL_FlightPreviewAllowed())){
        Msg("[painter_piece_continuation] id=%u phase=split_canceled expired=%u",object.ID(),unsigned(expired));
        cancel_split(entry);return false;
    }
    for(const auto& stage:batch.stages){
        if(stage->failed||stage->canceled){split_canceled(entry,"piece_spawn_failed");return false;}
        if(!stage->spawned||!stage->client)return false;
    }
    // Heavy render preparation of the stump and every piece runs on the renderer worker (as queue_render in
    // PainterDamagedFragment_impl.h); the commit below waits until all of it is ready.
    auto queue_render=[&](painter_damaged_fragment::RenderTicket& ticket,IKinematics& owner,u64 owner_cookie,u64 expected){
        if(ticket.attempted)return true;
        if(!ticket.mesh)return false;
        RuntimeSkinUpdate header;header.owner_cookie=owner_cookie;
        header.expected_geometry_generation=expected;header.damage_revision=req.publication_revision;
        header.separation_packet=true;header.flight_preview=bool(ticket.proxy);header.exact_rig=entry.rig;
        int outcome=RuntimeStageOutcome::unavailable;
        ticket.handle=owner.LL_StageRuntimeGeometry(header,ticket.display(),"models\\model","wm\\painter_stump_limb",
            entry.textures.empty()?nullptr:&entry.textures,owner_cookie,outcome,k->LL_PainterBurnStageSource());
        ticket.attempted=true;
        return ticket.handle||outcome==RuntimeStageOutcome::unavailable;
    };
    if(!batch.stump_render||!queue_render(*batch.stump_render,*k,entry.identity.spawn_cookie,entry.geometry)){
        split_canceled(entry,"stump_render_stage");return false;}
    bool render_ready=batch.stump_render->ready();
    for(const auto& stage:batch.stages){
        auto* child=smart_cast<IKinematics*>(stage->client->Visual());
        if(!child||stage->client->getDestroy()||child->LL_RenderGeometryGeneration()!=0||
           !queue_render(stage->render,*child,batch.child_cookies[stage->index],0)){split_canceled(entry,"piece_render_stage");return false;}
        render_ready=stage->render.ready()&&render_ready;
    }
    if(!render_ready)return false;
    PainterBurnPublication burnPublication(k);
    if(!burnPublication.ready())return false;
    painter_commit_parts::part("ps_bones");
    std::vector<Fmatrix> model(entry.rig.size()),palette(entry.rig.size());
    {xrCriticalSectionGuard guard(&k->UCalc_Mutex);
        k->CalculateBones_Invalidate();k->CalculateBones(TRUE);
        for(u16 b=0;b<entry.rig.size();++b){
            model[b]=k->LL_GetBoneInstance(b).mTransform;
            palette[b]=k->LL_GetBoneInstance(b).mRenderTransform;
        }
    }
    painter_commit_parts::part("ps_piece_shells");
    for(const auto& stage:batch.stages){
        auto* child=stage->client;auto* shell=child->PPhysicsShell();
        if(!shell||shell->get_ElementsNumber()!=req.pieces[stage->index].fitted.size()){split_canceled(entry,"piece_shell");return false;}
        child->XFORM()=object.XFORM();
        if(!painter_damaged_fragment::resample_shell(*shell,*entry.shell,model,object.XFORM())){split_canceled(entry,"piece_pose_resample");return false;}
        // splitfix2: a re-split piece starts as a first cut's piece does (PainterDamagedFragment_impl.h poll), not with its source
        // element's motion carried out to its centre: this source is not held while its split is prepared, so the hit that cut it
        // (a grenade on a cut corpse) had thrown it, and every piece took that speed plus its spin's lever. Each element starts from
        // its source element's own motion capped at 1.5 m/s and 4 rad/s; after a blast on the source it is thrown out from the blast
        // as a first cut's blast piece (3-9 m/s with its strength, spinning).
        {const auto* blast=painter_explosions::blast_on(object.ID());
         for(u16 e=0;e<shell->get_ElementsNumber();++e){auto* element=shell->get_ElementByStoreOrder(e);if(!element)continue;
          auto* origin=entry.shell->get_Element(element->m_SelfID);Fvector linear={0.f,0.f,0.f},angular={0.f,0.f,0.f};
          if(origin){origin->get_LinearVel(linear);origin->get_AngularVel(angular);}
          if(linear.magnitude()>1.5f)linear.normalize_safe().mul(1.5f);
          if(angular.magnitude()>4.f)angular.normalize_safe().mul(4.f);
          if(blast){Fvector com,out;element->GetGlobalPositionDynamic(&com);out.sub(com,blast->pos);out.y=_max(out.y,0.f)+.35f*out.magnitude();
           if(out.square_magnitude()<1e-6f)out.set(0.f,1.f,0.f);out.normalize_safe();linear.mad(out,3.f+6.f*_min(1.f,blast->strength));
           const float spin=4.f+8.f*_min(1.f,blast->strength);angular.set(out.z*spin,spin*.5f,-out.x*spin);}
          element->set_LinearVel(linear);element->set_AngularVel(angular);}}
    }
    painter_commit_parts::part("ps_stump_shell");
    auto* npc=smart_cast<CAI_Stalker*>(&object);
    auto* support=npc?npc->character_physics_support():nullptr;
    if(npc&&(!support||!support->PainterCanSwapFittedShell(entry.shell))){split_canceled(entry,"stump_swap_not_ready");return false;}
    CPhysicsShell* fitted=P_prepare_FittedShell(&object,entry.shell,req.stump_physics,req.stump_visible_bones,false,req.stump_root_bone,batch.stump_render->mesh.get());
    if(!fitted){Msg("[painter_piece_continuation] id=%u phase=fitted_refused part=stump_shell reason=%s",object.ID(),P_fitted_shell_refusal());
        split_canceled(entry,"stump_fitted_shell");return false;}
    bool okay=painter_damaged_fragment::resample_shell(*fitted,*entry.shell,model,object.XFORM());
    const char* why=okay?"":"stump_pose_resample";
    painter_commit_parts::part("ps_stump_cover");
    u64 missing=0;
    if(okay)okay=painter_damaged_fragment::pose_covered(*batch.stump_render->mesh,req.stump_physics,req.stump_pose_anchors,u16(entry.rig.size()),missing)&&
        P_bind_FittedPoseLinks(fitted,painter_damaged_fragment::pose_links(missing,req.stump_pose_anchors,model),false);
    if(!okay&&!*why)why="stump_pose_links";
    for(const auto& stage:batch.stages){
        if(!okay)break;
        painter_commit_parts::part("ps_piece_cover");
        auto* child=smart_cast<IKinematics*>(stage->client->Visual());
        auto* shell=stage->client->PPhysicsShell();
        const auto& part=req.pieces[stage->index];
        if(!child||child->LL_RenderGeometryGeneration()!=0||!shell||
           !painter_damaged_fragment::pose_covered(*stage->render.mesh,part.fitted,part.pose_anchors,u16(entry.rig.size()),missing)){okay=false;why="piece_pose_cover";break;}
        child->LL_SetBonesVisible(u64(-1));
        if(!P_bind_FittedPoseLinks(shell,painter_damaged_fragment::pose_links(missing,part.pose_anchors,model),true)){okay=false;why="piece_pose_links";break;}
        u64 mask=missing;
        for(const auto& element:part.fitted)mask|=1ull<<element.bone;
        {xrCriticalSectionGuard guard(&child->UCalc_Mutex);
            child->CalculateBones_Invalidate();child->CalculateBones(TRUE);
            for(u16 b=0;b<entry.rig.size();++b)
                if((mask&(1ull<<b))&&!painter_damaged_fragment::same(child->LL_GetBoneInstance(b).mRenderTransform,palette[b],5e-3f)){okay=false;why="piece_pose_mismatch";} // R33: 5 mm (PainterDamagedFragment_impl.h)
        }
        if(!okay)break;
        painter_commit_parts::part("ps_piece_adopt");
        RuntimeSkinUpdate update;
        update.owner_cookie=batch.child_cookies[stage->index];
        update.expected_geometry_generation=0;update.flight_preview=bool(stage->render.proxy);
        update.damage_revision=req.publication_revision;
        update.separation_packet=true;update.exact_rig=entry.rig;
        okay=stage->render.handle?
            child->LL_PrepareStagedRuntimeGeometry(stage->render.handle,update,update.owner_cookie,object.Visual(),&stage->client->XFORM()):
            (AppendPackedRuntimeChildren(*stage->render.display(),update,"models\\model","wm\\painter_stump_limb",entry.textures.empty()?nullptr:&entry.textures)&&
             child->LL_PrepareRuntimeGeometryWithDonor(update,update.owner_cookie,object.Visual(),&stage->client->XFORM()));
        if(!okay&&!*why)why="piece_render_prepare";
    }
    painter_commit_parts::part("ps_stump_adopt");
    if(okay){
        RuntimeSkinUpdate update;
        update.owner_cookie=entry.identity.spawn_cookie;
        update.expected_geometry_generation=entry.geometry;
        update.damage_revision=req.publication_revision;
        update.separation_packet=true;update.exact_rig=entry.rig;
        okay=batch.stump_render->handle?
            k->LL_PrepareStagedRuntimeGeometry(batch.stump_render->handle,update,update.owner_cookie,nullptr,nullptr):
            (AppendPackedRuntimeChildren(*batch.stump_render->mesh,update,"models\\model","wm\\painter_stump_limb",entry.textures.empty()?nullptr:&entry.textures)&&
             k->LL_PrepareRuntimeGeometry(update,update.owner_cookie));
        if(!okay)why="stump_render_prepare";
    }
    painter_commit_parts::part("ps_preflight");
    if(okay){okay=k->LL_CanCommitPreparedRuntimeGeometry(entry.identity.spawn_cookie);if(!okay)why="stump_render_preflight";}
    for(const auto& stage:batch.stages){
        auto* child=smart_cast<IKinematics*>(stage->client->Visual());
        if(!child||!child->LL_CanCommitPreparedRuntimeGeometry(batch.child_cookies[stage->index])){okay=false;if(!*why)why="piece_render_preflight";}
    }
    auto discard_all=[&]{k->LL_DiscardPreparedRuntimeGeometry();
        for(const auto& stage:batch.stages){auto* child=stage->client?smart_cast<IKinematics*>(stage->client->Visual()):nullptr;if(child)child->LL_DiscardPreparedRuntimeGeometry();}
        painter_damaged_fragment::discard_fitted(fitted,*entry.shell);cancel_split(entry);};
    if(!okay){Msg("[painter_piece_continuation] id=%u phase=split_canceled reason=%s",object.ID(),why);discard_all();return false;}
    painter_commit_parts::part("ps_tether_prep");
    painter_damaged_fragment::HeadTetherTicket head_tether;
    if(!painter_damaged_fragment::prepare_head_tether(object,*k,*fitted,req,batch.stages,head_tether)){
        Msg("[painter_piece_continuation] id=%u phase=split_canceled reason=head_tether_prepare",object.ID());discard_all();return false;}
    painter_commit_parts::part("ps_reserve");
    bool reserved_all=true;
    for(const auto& stage:batch.stages){
        auto identity=entry.identity;identity.spawn_cookie=batch.child_cookies[stage->index];
        const auto& part=req.pieces[stage->index];
        painter_flight::Member flight{req.flight,part.component_id,req.original_roles,part.children};
        reserved_all=reserve(*stage->client,req.pieces[stage->index].seed,stage->render.mesh,
            std::move(identity),1,entry.input,entry.rig,entry.textures,entry.original_children,req.publication_revision,req.flight?&flight:nullptr);
        if(!reserved_all)break;
    }
    if(!reserved_all){Msg("[painter_piece_continuation] id=%u phase=split_canceled reason=piece_reserve",object.ID());discard_all();return false;}
    painter_commit_parts::part("ps_session");
    std::shared_ptr<gfy_live::Session> replacement_session;
    std::shared_ptr<PartReadyLink> replacement_link;
    painter_explosions::RouteTicket grenade_ticket;
    bool prepared_route=false;
    try{auto* pool=painter_voxel_live_pool();
        replacement_session=pool?pool->attach_piece(req.stump_seed,entry.identity,batch.stump_render->mesh,req.publication_revision,entry.geometry+1):std::shared_ptr<gfy_live::Session>();
        if(replacement_session){replacement_link=std::make_shared<PartReadyLink>();replacement_link->source_support=entry.source_support;replacement_link->fragment=entry.fragment;
            hook_part_session(*replacement_session,replacement_link);
            auto route=painter_damaged_fragment::grenade_route(object,req,batch.stages,entry.identity,entry.geometry+1,[&](std::size_t index){
                auto identity=entry.identity;identity.spawn_cookie=batch.child_cookies[index];return identity;});
            prepared_route=route&&grenade_ticket.reserve(std::move(route));}
    }catch(...){prepared_route=false;}
    if(!prepared_route){if(replacement_session)replacement_session->cancel();discard_all();return false;}
    painter_commit_parts::part("ps_render_commit");
    for(const auto& stage:batch.stages){
        auto* child=smart_cast<IKinematics*>(stage->client->Visual());
        if(!child->LL_CommitPreparedRuntimeGeometry(batch.child_cookies[stage->index])){Msg("[painter_piece_continuation] id=%u phase=split_canceled reason=piece_render_commit",object.ID());discard_all();return false;}
    }
    if(!k->LL_CommitPreparedRuntimeGeometry(entry.identity.spawn_cookie)){
        painter_damaged_fragment::discard_fitted(fitted,*entry.shell);split_canceled(entry,"stump_render_commit");return false;
    }
    painter_commit_parts::part("ps_shell_swap");
    if(!entry.grouped){entry.collision_group=RegisterGroup();entry.grouped=true;}
    const CGID split_group=entry.collision_group;
    fitted->RegisterToCLGroup(split_group);
    fitted->set_DynamicLimits(resplit_linear_limit); // splitfix2: the continued part's speed limit
    if(support)support->PainterSwapFittedShell(fitted,entry.shell,req.stump_visible_bones);
    if(support)support->PainterSetFlesh(req.stump_physics,fitted,&req.stump_flesh_gear); // bodypass: the boxes this shell was built from; gearbody: + its merged gear
    else{
        painter_damaged_fragment::apply_piece_contact(*fitted,k,req.stump_root_bone,entry.shell);
        entry.shell->EnabledCallbacks(FALSE);
        object.PPhysicsShell()=fitted;
        k->LL_SetBonesVisible(u64(-1));
        fitted->EnabledCallbacks(TRUE);fitted->EnableCollision();fitted->Enable();
        entry.shell->DisableCollision(); // never touches its own replacement while it deactivates
        destroy_physics_shell(entry.shell);
        object.PPhysicsShell()->EnabledCallbacks(TRUE);
        k->CalculateBones_Invalidate();
    }
    head_tether.commit();
    // This object continues as the largest part: a new session on its seed, its mesh as installed.
    if(entry.session)entry.session->cancel();
    entry.drop_stage();entry.committed_effects.clear();
    entry.shell=fitted;entry.geometry=k->LL_RenderGeometryGeneration();
    entry.seed=req.stump_seed;entry.birth=batch.stump_render->mesh;
    entry.first_revision=entry.sequence=entry.sever_revision=entry.checked_revision=req.publication_revision;
    entry.session=std::move(replacement_session);entry.ready_link=std::move(replacement_link);
    painter_burn_host::bind(object,entry.identity,entry.session,true);
    painter_commit_parts::part("ps_activate");
    unsigned bound=0;
    for(const auto& stage:batch.stages){
        auto* shell=stage->client->PPhysicsShell();
        shell->RegisterToCLGroup(split_group); // R31
        painter_damaged_fragment::apply_piece_contact(*shell,k,req.pieces[stage->index].root_bone);
        shell->set_DynamicLimits(resplit_linear_limit); // splitfix2: the new piece's speed limit
        shell->EnableCollision();shell->Enable();
        stage->client->PainterInheritThermal(object);
        stage->client->setEnabled(TRUE);stage->client->setVisible(TRUE);
        auto* child=smart_cast<IKinematics*>(stage->client->Visual());
        bound+=child&&painter_piece_continuation::activate(*stage->client,child->LL_RenderGeometryGeneration())?1:0;
        painter_piece_continuation::set_collision_group(*stage->client,split_group);
        painter_damaged_fragment::named.erase(stage->name);
    }
    if(painter_burn_host::active()){std::vector<CObject*> parts; for(const auto& s:batch.stages)parts.push_back(s->client);painter_burn_host::split(object,parts);}
    burnPublication.release();
    painter_commit_parts::part("ps_routes");
    grenade_ticket.commit();retry_grenade_routes();
    entry.cancel_retries=0; // R33
    Msg("[painter_piece_continuation] id=%u phase=split_committed children=%u continued=%u generation=%llu revision=%llu t=%u",
        object.ID(),unsigned(batch.stages.size()),bound+(entry.session?1u:0u),entry.geometry,req.publication_revision,Device.dwTimeGlobal);
    painter_commit_parts::part("ps_notices");
    const auto notices=painter_damaged_fragment::committed_notices(req,batch.stages,req.publication_revision);
    splits.erase(it);
    painter_commit_parts::part("ps_notify");
    painter_damaged_fragment::notify_committed(object,notices);
    return true;
}
// Installs a worker output on this object's visual (off-thread staging when the renderer offers it). False: still
// preparing; nothing acknowledged yet.
bool publish(Owner& entry,const std::shared_ptr<const gfy_live::Output>& out){
    auto& object=*entry.object;auto* k=entry.visual;
    const bool current=out->error.empty()&&out->stamp.owner==entry.identity&&out->stamp.expected_generation==k->LL_RenderGeometryGeneration();
    bool accepted=false;CTimer timer;timer.Start();const char* path="sync";RuntimeStageTiming timing;
    RuntimeSkinUpdate update;update.owner_cookie=entry.identity.spawn_cookie;update.expected_geometry_generation=out->stamp.expected_generation;
    update.damage_revision=out->stamp.revision;update.exact_rig=entry.rig;
    PainterBurnPublication burnPublication(current&&!out->no_geometry_change&&(!entry.stage||entry.stage->Ready())?k:nullptr);
    if(!burnPublication.ready())return false;
    if(current&&out->no_geometry_change)accepted=true;
    else if(current){
        if(!entry.stage){int outcome=RuntimeStageOutcome::unavailable;
            IRuntimeGeometryStage* staged=k->LL_StageRuntimeGeometry(update,std::shared_ptr<const gfy_packed::Mesh>(out,&out->mesh),
                "models\\model","wm\\painter_stump_limb",entry.textures.empty()?nullptr:&entry.textures,entry.identity.spawn_cookie,outcome);
            if(staged){entry.stage=staged;entry.stage_out=out;return false;}
            if(outcome==RuntimeStageOutcome::unavailable)
                accepted=AppendPackedRuntimeChildren(out->mesh,update,"models\\model","wm\\painter_stump_limb",entry.textures.empty()?nullptr:&entry.textures)&&
                    k->LL_InstallRuntimeGeometry(update,entry.identity.spawn_cookie);
            else path="stage_refused";
        }else{if(!entry.stage->Ready())return false;path="offthread";
            accepted=k->LL_InstallStagedRuntimeGeometry(entry.stage,update,entry.identity.spawn_cookie);timing=entry.stage->Timing();entry.drop_stage();}
    }
    burnPublication.release();
    entry.session->acknowledge_output(out,accepted,k->LL_RenderGeometryGeneration());
    if(accepted&&!out->no_geometry_change)painter_burn_host::refresh(object,entry.session);
    Msg("[painter_piece_continuation] id=%u phase=%s revision=%llu events=%llu triangles=%llu worker_ms=%.1f install_ms=%.2f path=%s reason=%s admit_ms=%.2f wrap_ms=%.2f wallmarks_ms=%.2f commit_ms=%.2f children=%u marks=%u t=%u",
        object.ID(),accepted?(out->no_geometry_change?"no_op":"committed"):"rejected",out->stamp.revision,u64(out->batch_events),u64(out->triangles),
        out->worker_ms,timer.GetElapsed_ms_f(),path,out->error.empty()?(current?"none":"stale_generation"):out->error.c_str(),
        timing.admit_ms,timing.wrap_ms,timing.wallmarks_ms,timing.commit_ms,timing.children,timing.marks,Device.dwTimeGlobal);
    // gfyreject2: what the body path logs (PainterVoxelLive_impl.h), for a continued part too - a thrown-away hit names its reason
    for(const auto& impact:out->impacts)if(!impact.rejection.empty())
        Msg("[painter_piece_continuation] id=%u phase=impact_rejected sequence=%llu reason=%s",object.ID(),impact.sequence,impact.rejection.c_str());
    if(!out->accessory_skipped.empty())Msg("[painter_piece_continuation] id=%u phase=accessory_skipped revision=%llu accepted=%u %s",
        object.ID(),out->stamp.revision,unsigned(accepted),out->accessory_skipped.c_str()); // ownersafe
    if(!out->geometry_retry_reason.empty())
        Msg("[painter_piece_continuation] id=%u phase=rough_failed_smooth_retry revision=%llu accepted=%u changed=%u reason=%s",object.ID(),out->stamp.revision,
            unsigned(accepted),unsigned(accepted&&!out->no_geometry_change),out->geometry_retry_reason.c_str());
    if(out->geometry_batch_splits)
        Msg("[painter_piece_continuation] id=%u phase=geometry_batch_isolation revision=%llu splits=%llu deferred=%llu reason=%s",object.ID(),out->stamp.revision,
            u64(out->geometry_batch_splits),u64(out->deferred_events),out->geometry_batch_reason.c_str());
    if(!accepted)return true;
    entry.geometry=k->LL_RenderGeometryGeneration();
    if(out->burn_surface_prime)return true;
    painter_explosions::accepted(object,*out);
    if(!out->no_geometry_change)painter_cranial_remnants::committed(object,out->joints,out->impacts,out->cranial,out->stamp.revision);
    if(!out->no_geometry_change)entry.damage_revision=out->stamp.revision;
    for(const auto& impact:out->impacts)if(impact.sever)entry.sever_revision=out->stamp.revision;
    if(!out->no_geometry_change){entry.committed_effects.assign(out->impacts);painter_committed_effects::drain(object,entry.committed_effects,false);}
    return true;
}
}
bool reserve(CPhysicsShellHolder& object,std::shared_ptr<const gfy_live::PieceSeed> seed,
    std::shared_ptr<const gfy_packed::Mesh> published,gfy_live::Identity identity,std::uint64_t generation,
    const gfy_live::SeparationInput& input,const xr_vector<RuntimeSkinBone>& rig,
    const xr_vector<shared_str>& textures,std::uint16_t original_children,
    std::uint64_t published_revision,const painter_flight::Member* flight)try{
    auto* k=smart_cast<IKinematics*>(object.Visual());
    if(!seed||!published||!k||!generation||!object.PPhysicsShell()||!object.PPhysicsShell()->isFullActive()||
       !original_children||rig.size()!=k->LL_BoneCount()||owners.find(object.ID())!=owners.end())return false;
    auto entry=std::make_unique<Owner>();
    entry->object=&object;entry->visual=k;entry->shell=object.PPhysicsShell();
    if(auto* npc=smart_cast<CAI_Stalker*>(&object))entry->source_support=npc->character_physics_support();
    else entry->fragment=smart_cast<CPhysicsSkeletonObject*>(&object);
    if(!entry->source_support&&!entry->fragment)return false;
    entry->identity=identity;entry->geometry=generation;entry->input=input;
    entry->rig=rig;entry->textures=textures;entry->original_children=original_children;
    entry->first_revision=published_revision;entry->thread=GetCurrentThreadId();
    entry->seed=std::move(seed);entry->birth=std::move(published);
    if(flight){entry->flight=std::make_unique<flight_detail::Pending>();entry->flight->member=*flight;}
    auto* pool=painter_voxel_live_pool();
    entry->session=pool?pool->attach_piece(entry->seed,entry->identity,entry->birth,entry->first_revision,generation):std::shared_ptr<gfy_live::Session>();
    if(!entry->session)return false;
    if(flight)entry->session->hold_flight_birth();
    entry->ready_link=std::make_shared<PartReadyLink>();entry->ready_link->source_support=entry->source_support;entry->ready_link->fragment=entry->fragment;
    hook_part_session(*entry->session,entry->ready_link);
    if(!painter_explosions::register_target(object,entry->identity)){entry->session->cancel();return false;}
    owners[object.ID()]=std::move(entry);
    return true;
}catch(...){return false;}
bool activate(CPhysicsShellHolder& object,std::uint64_t generation){
    auto* entry=find(object);
    auto* k=smart_cast<IKinematics*>(object.Visual());
    if(!entry||entry->active||!k||k!=entry->visual||k->LL_RenderGeometryGeneration()!=generation||
       generation!=entry->geometry||!object.PPhysicsShell()||!object.PPhysicsShell()->isFullActive())return false;
    if(!entry->session||!entry->ready_link)return false;
    entry->shell=object.PPhysicsShell();entry->active=true;
    if(entry->flight){
        entry->flight->firstFrame=Device.dwFrame;
        const auto& parent=entry->flight->member.group->job->stamp();
        Msg("[painter_flight] id=%u phase=proxy_visible component=%u parent_cookie=%llu parent_revision=%llu child_cookie=%llu frame=%u t=%u",object.ID(),entry->flight->member.component,parent.owner.spawn_cookie,parent.revision,entry->identity.spawn_cookie,Device.dwFrame,Device.dwTimeGlobal);
        queue(object);
    }
    painter_burn_host::bind(object,entry->identity,entry->session,true);
    entry->sequence=entry->sever_revision=entry->checked_revision=entry->first_revision;
    Msg("[painter_piece_continuation] id=%u phase=attached revision=%llu generation=%llu triangles=%llu",object.ID(),entry->first_revision,generation,
        [&]{u64 n=0;for(const auto& c:entry->birth->children)n+=c.indices.size()/3;return n;}());
    return true;
}
void set_collision_group(CPhysicsShellHolder& object,unsigned group){
    auto* entry=find(object);if(!entry)return;
    entry->collision_group=CGID(group);entry->grouped=true;
}
void abandon(CPhysicsShellHolder& object){
    auto it=owners.find(object.ID());
    if(it!=owners.end()&&it->second->object==&object&&!it->second->active)owners.erase(it);
}
bool bind(CPhysicsShellHolder& object,std::shared_ptr<const gfy_live::PieceSeed> seed,
    std::shared_ptr<const gfy_packed::Mesh> published,gfy_live::Identity identity,std::uint64_t generation,
    const gfy_live::SeparationInput& input,const xr_vector<RuntimeSkinBone>& rig,
    const xr_vector<shared_str>& textures,std::uint16_t original_children,std::uint64_t published_revision){
    if(!reserve(object,std::move(seed),std::move(published),std::move(identity),generation,input,rig,textures,original_children,published_revision))return false;
    if(activate(object,generation))return true;
    abandon(object);return false;
}
void forget(CPhysicsShellHolder& object){
    painter_cranial_remnants::forget(object);
    auto it=owners.find(object.ID());
    if(it==owners.end()||it->second->object!=&object)return;
    auto& entry=*it->second;
    painter_burn_host::forget(object,entry.session);
    if(!entry.committed_effects.empty())
        Msg("[painter_piece_continuation] id=%u phase=wound_blood_invalidated reason=owner_canceled receipts=%u",object.ID(),unsigned(entry.committed_effects.size()));
    cancel_split(entry);
    if(entry.job)entry.job->cancel();
    if(entry.session)entry.session->cancel();
    entry.drop_stage();
    if(entry.ready_link){ // a worker wake from now on finds no owner; pending entries of this part are cleared
        xrCriticalSectionGuard guard(&Device.seqParallelBeforRenderCS);xrCriticalSectionGuard pending(&Device.seqParallelBeforRenderPendingCS);
        xr_delegate<void()> callback;
        if(entry.ready_link->source_support)callback=xr_make_delegate(entry.ready_link->source_support,&CCharacterPhysicsSupport::ApplyPainterPieceContinuation);
        else if(entry.ready_link->fragment)callback=xr_make_delegate(entry.ready_link->fragment,&CPhysicsSkeletonObject::ApplyPainterPieceContinuation);
        entry.ready_link->source_support=nullptr;entry.ready_link->fragment=nullptr;
        if(callback){for(auto& task:Device.seqParallelBeforRenderPending)if(task==callback)task.clear();
            for(auto& task:Device.seqParallelBeforRenderNext)if(task==callback)task.clear();}}
    if(entry.source_support){
        auto callback=xr_make_delegate(entry.source_support,&CCharacterPhysicsSupport::ApplyPainterPieceContinuation);
        xrCriticalSectionGuard guard(&Device.seqParallelBeforRenderCS);
        for(auto& task:Device.seqParallelBeforRender)if(task==callback)task.clear();
    }else if(entry.fragment){
        auto callback=xr_make_delegate(entry.fragment,&CPhysicsSkeletonObject::ApplyPainterPieceContinuation);
        xrCriticalSectionGuard guard(&Device.seqParallelBeforRenderCS);
        for(auto& task:Device.seqParallelBeforRender)if(task==callback)task.clear();
    }
    owners.erase(it);
}
bool prime_burn_geometry(CPhysicsShellHolder& object,u64 cookie){
    auto* entry=find(object);
    if(!entry||!entry->active||!entry->session||entry->identity.spawn_cookie!=cookie||
       !valid_owner(*entry)||entry->job||entry->flight||entry->stage||
       splits.find(object.ID())!=splits.end())return false;
    auto prior=entry->session->published_burn_surface();
    if(!prior)return false;
    if(!prior->needs_burn_prime())return true;
    auto* pool=painter_voxel_live_pool();
    if(!pool||!pool->prime_burn_geometry(entry->session,entry->geometry,entry->sequence+1))return false;
    ++entry->sequence;
    queue(object);
    return true;
}
bool bind_burn_receiver(CPhysicsShellHolder& object){
    auto* entry=find(object);
    if(!entry||!entry->active||!entry->session||object.getDestroy()||!entry->visual||
       entry->visual!=smart_cast<IKinematics*>(object.Visual())||
       entry->geometry!=entry->visual->LL_RenderGeometryGeneration())return false;
    painter_burn_host::bind(object,entry->identity,entry->session,true,true);
    return true;
}
bool owns(const CPhysicsShellHolder& object,std::uint16_t bone){
    auto* entry=find(object);
    return entry&&entry->active&&entry->visual==smart_cast<IKinematics*>(object.Visual())&&bone<entry->rig.size();
}
bool capture(CPhysicsShellHolder& object,const SHit& hit){
    body_hit_outcomes::set_sink(+[](const char*s)noexcept{Msg("%s host_us=%lld",s,static_cast<long long>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()));});
    body_hit_outcomes::Scope trace_scope(object.ID(),hit.BulletID,hit.bone(),false,true);
    auto* entry=find(object);if(!entry||!entry->active)return false;
    if((hit.type()==ALife::eHitTypeBurn||hit.type()==ALife::eHitTypeLightBurn) && entry->session && valid_owner(*entry))
        PainterSubmitLocalThermal(object,hit);
    const bool meleeHit=hit.type()!=ALife::eHitTypeFireWound&&painter_melee_event::eligible(hit); // melee (2026-11-12, value 3 only)
    if((hit.type()!=ALife::eHitTypeFireWound&&!meleeHit)||!valid_owner(*entry)||!entry->session||
       hit.bone()>=entry->rig.size()||!std::isfinite(hit.damage())||hit.damage()<=0.f){body_hit_outcomes::report("not_queued",(hit.type()!=ALife::eHitTypeFireWound&&!meleeHit)?"piece_hit_type":!valid_owner(*entry)?"piece_owner_invalid":!entry->session?"piece_no_session":hit.bone()>=entry->rig.size()?"piece_bone_range":"piece_nonpositive_power");return true;}
    std::vector<gfy_live::Impact> events;
    const auto built=meleeHit?painter_melee_event::build(object,entry->visual,false,hit,entry->sequence,events,"painter_piece_continuation"):
        painter_wound_event::build(object,entry->visual,false,hit,entry->sequence,events,"painter_piece_continuation");
    body_hit_outcomes::tag_events(events);
    if(built.status==painter_wound_event::Status::rejected){
        body_hit_outcomes::report("not_queued",built.reason);
        Msg("[painter_piece_continuation] id=%u phase=hit_rejected reason=%s",object.ID(),built.reason);return true;}
    if(built.status!=painter_wound_event::Status::queued){body_hit_outcomes::report("not_queued",built.reason);return true;}
    if(!std::strcmp(built.policy,"fragment")){
        if(!painter_explosions::capture_fragment(object,std::move(events),built.blade>0,true))
            Msg("[painter_piece_continuation] id=%u phase=hit_rejected reason=fragment_retained_capacity_or_quota",object.ID());
        queue(object);return true;}
    unsigned queued=0;
    for(auto& event:events){if(!entry->session->enqueue(std::move(event))){body_hit_outcomes::batch("not_queued","piece_prior_sub_event_enqueue_failed",0,events,std::size_t(&event-events.data())+1);break;}++entry->sequence;++queued;}
    if(painter_log_verbose())Msg("[painter_piece_continuation] id=%u phase=hit_accepted sequence=%llu bone=%u ammo=%s radius_m=%.4f penetration_m=%.3f blade=%u queued=%u policy=%s",
        object.ID(),entry->sequence,unsigned(hit.bone()),built.ammo_section.c_str(),built.radius,built.penetration,built.blade,queued,built.policy);
    queue(object);return true;
}
unsigned blast(CPhysicsShellHolder& object,std::vector<gfy_live::Impact>& events){
    auto* entry=find(object);if(!entry||!entry->active||!valid_owner(*entry)||!entry->session)return 0;
    unsigned queued=0;
    for(auto& event:events){event.sequence=entry->sequence+1;if(!entry->session->enqueue(std::move(event)))break;++entry->sequence;++queued;}
    events.erase(events.begin(),events.begin()+queued);
    queue(object);return queued;
}
bool grenade_ready(const painter_explosions::RouteWitness& witness){
    if(!painter_explosions::current(witness.born))return false;
    auto* object=smart_cast<CPhysicsShellHolder*>(Level().Objects.net_Find(witness.born.id));
    auto* entry=object?find(*object):nullptr;
    return entry&&entry->active&&entry->session&&entry->identity==witness.identity&&
        entry->geometry==witness.generation&&entry->first_revision==witness.revision&&valid_owner(*entry);
}
void retry_grenade_routes(){
    painter_explosions::retry_routes(grenade_ready,[](const painter_explosions::Born& born){
        if(painter_explosions::current(born))if(auto* object=smart_cast<CPhysicsShellHolder*>(Level().Objects.net_Find(born.id)))queue(*object);});
}
void queue(CPhysicsShellHolder& object){
    auto* entry=find(object);
    if(!entry||!entry->active||entry->queued)return;
    const bool work=entry->flight||entry->job||splits.find(object.ID())!=splits.end()||!entry->committed_effects.empty()||entry->stage||
        (entry->session&&entry->session->pending())||entry->sever_revision>entry->checked_revision||painter_cranial_remnants::has_pending(object)||painter_explosions::pending(object)||painter_explosions::route_pending(object);
    if(!work)return;
    entry->queued=true;
    xrCriticalSectionGuard guard(&Device.seqParallelBeforRenderCS);
    if(entry->source_support)
        Device.seqParallelBeforRender.push_back(xr_make_delegate(entry->source_support,&CCharacterPhysicsSupport::ApplyPainterPieceContinuation));
    else if(entry->fragment)
        Device.seqParallelBeforRender.push_back(xr_make_delegate(entry->fragment,&CPhysicsSkeletonObject::ApplyPainterPieceContinuation));
}
void poll(CPhysicsShellHolder& object){
    auto* entry=find(object);if(!entry)return;
    painter_commit_parts::Scope commit_scope("piece_apply",object.ID()); // piececommit: this poll's line (parts below)
    entry->queued=false;
    // Liveness, as on the body (PainterVoxelLive_impl.h ApplyPainterVoxelLive, R64): a piece or stump at rest sleeps (its shell
    // switches the object's processing off: no UpdateCL, so nobody calls queue()). A finished separation check then waited for
    // the next hit (test log 2026-11-11: 26 of 192 results waited 0.5 - 158 s). While a check runs or a split is being staged,
    // this callback queues itself for the next drain. The owner is looked up again: poll() may forget it.
    struct RequeueAtExit {CPhysicsShellHolder& object;~RequeueAtExit(){
        try{auto* e=find(object);
            if(!e||!e->active||e->queued||object.getDestroy())return;
            if(!e->flight&&!e->job&&splits.find(object.ID())==splits.end())return;
            xr_delegate<void()> callback;
            if(e->source_support)callback=xr_make_delegate(e->source_support,&CCharacterPhysicsSupport::ApplyPainterPieceContinuation);
            else if(e->fragment)callback=xr_make_delegate(e->fragment,&CPhysicsSkeletonObject::ApplyPainterPieceContinuation);
            if(!callback)return;
            xrCriticalSectionGuard guard(&Device.seqParallelBeforRenderCS);
            if(std::find(Device.seqParallelBeforRenderNext.begin(),Device.seqParallelBeforRenderNext.end(),callback)==Device.seqParallelBeforRenderNext.end())
                Device.seqParallelBeforRenderNext.push_back(callback);
            e->queued=true;}catch(...){}}} requeueAtExit{object};
    if(!Device.IsGeometryPublishBarrier()||Device.Paused()||!physics_world()||physics_world()->Processing())return;
    if(!valid_owner(*entry)){forget(object);return;}
    retry_grenade_routes();
    // Preserve the exact all-parts publication witness while a retained transfer retries.
    if(painter_explosions::route_pending(object))return;
    painter_explosions::pump(object,bool(entry->job)||splits.find(object.ID())!=splits.end(),entry->checked_revision,
        [&](gfy_live::Impact&& event,u64& admitted){event.sequence=entry->sequence+1;
            if(!event.sequence||!entry->session||!entry->session->enqueue(std::move(event)))return false;
            admitted=++entry->sequence;return true;});
    painter_cranial_remnants::poll(object);
    painter_commit_parts::part("pc_effects");
    if(!entry->committed_effects.empty()&&!painter_committed_effects::drain(object,entry->committed_effects,false))return;
    if(splits.find(object.ID())!=splits.end()){painter_commit_parts::part("pc_split_commit");poll_split(*entry);return;}
    painter_commit_parts::part("pc_check");
    if(entry->flight&&poll_detail(*entry))return;
    if(!entry->session)return;
    auto* k=entry->visual;
    // A finished separation of this piece: the new parts spawn, this object keeps the largest.
    std::shared_ptr<const gfy_live::SeparationOutput> result;
    std::shared_ptr<painter_flight::Group> flight;
    if(entry->job&&!painter_explosions::first_incomplete(object)){
        result=entry->job->completed();
        const auto& stamp=entry->job->stamp();
        if((!result||(result->error.empty()&&!result->no_separation))&&entry->flight_attempted_revision!=stamp.revision&&stamp.owner==entry->identity&&
           stamp.expected_generation==entry->geometry){
            if(auto early=entry->job->preview();early&&k->LL_FlightPreviewAllowed()){
                result=std::move(early);entry->flight_attempted_revision=stamp.revision;
                flight=std::make_shared<painter_flight::Group>();flight->job=entry->job;
            }
        }
    }
    if(result){
        entry->job.reset();const auto& r=*result;
        const bool stale_cut=r.error.empty()&&!r.no_separation&&r.stamp.owner==entry->identity&&r.pieces.size()>=2&&
            r.stamp.expected_generation!=entry->geometry;
        entry->split_imminent_at=stale_cut?Device.dwTimeGlobal:0;
        if(stale_cut){entry->retry_at=Device.dwTimeGlobal; // check the newest damage right away, wounds held meanwhile
            Msg("[painter_piece_continuation] id=%u phase=split_imminent revision=%llu generation=%llu worker_ms=%.1f t=%u",
                object.ID(),r.stamp.revision,entry->geometry,r.worker_ms,Device.dwTimeGlobal);}
        else if(!r.error.empty())Msg("[painter_piece_continuation] id=%u phase=cpu_refused revision=%llu reason=%s worker_ms=%.1f",object.ID(),r.stamp.revision,r.error.c_str(),r.worker_ms);
        else if(r.no_separation)Msg("[painter_piece_continuation] id=%u phase=still_connected revision=%llu worker_ms=%.1f t=%u",object.ID(),r.stamp.revision,r.worker_ms,Device.dwTimeGlobal);
        else if(r.stamp.owner==entry->identity&&r.stamp.expected_generation==entry->geometry&&r.pieces.size()>=2){
            painter_commit_parts::part("pc_split_start");
            const bool staged=start_split(*entry,result,flight);
            if(!staged&&flight){entry->job=flight->job;return;}
            Msg("[painter_piece_continuation] id=%u phase=%s revision=%llu pieces=%u worker_ms=%.1f t=%u lowpoly=%s",object.ID(),staged?"split_staged":"split_refused",
                r.stamp.revision,unsigned(r.pieces.size()),r.worker_ms,Device.dwTimeGlobal,r.lowpoly_summary.c_str());
            if(staged)return;}
        else Msg("[painter_piece_continuation] id=%u phase=cpu_stale revision=%llu",object.ID(),r.stamp.revision);
    }
    // Publication continues while a check runs; it is held only when a stale check found this piece apart, until the
    // check of the newest damage decides (bounded).
    painter_commit_parts::part("pc_publish");
    const bool hold=entry->split_imminent_at&&s32(Device.dwTimeGlobal-entry->split_imminent_at)<1500;
    if(!hold){
        if(entry->stage&&entry->session->completed()!=entry->stage_out)entry->drop_stage();
        if(auto out=entry->session->completed())if(!publish(*entry,out)){
            // Still preparing: look again at the next drain (as the body does) instead of the next frame's poll.
            xrCriticalSectionGuard guard(&Device.seqParallelBeforRenderCS);
            xr_delegate<void()> callback;
            if(entry->source_support)callback=xr_make_delegate(entry->source_support,&CCharacterPhysicsSupport::ApplyPainterPieceContinuation);
            else if(entry->fragment)callback=xr_make_delegate(entry->fragment,&CPhysicsSkeletonObject::ApplyPainterPieceContinuation);
            if(callback&&std::find(Device.seqParallelBeforRenderNext.begin(),Device.seqParallelBeforRenderNext.end(),callback)==Device.seqParallelBeforRenderNext.end()){
                Device.seqParallelBeforRenderNext.push_back(callback);entry->queued=true;}
            return;}
    }
    painter_commit_parts::part("pc_dispatch");
    // Queued hits.
    if(!entry->stage&&!entry->session->completed())if(auto* pool=painter_voxel_live_pool())pool->dispatch(entry->session,entry->geometry);
    // After any published damage (a flagged sever or a cut worn through, any direction): check whether this piece came apart
    // (the corpse policy decides, as for bodies).
    // While held, a staged publication waits: the check uses the last published damage (the split discards the stage).
    if(!painter_explosions::first_incomplete(object)&&!entry->job&&(!entry->stage||hold)&&std::max(entry->sever_revision,entry->damage_revision)>entry->checked_revision&&
       s32(Device.dwTimeGlobal-entry->retry_at)>=0&&separation_allowed(object)){
        auto snapshot=entry->session->capture_damage();
        if(snapshot&&snapshot->stamp().expected_generation==entry->geometry&&snapshot->stamp().revision>=entry->sever_revision){
            auto input=entry->input;input.minimum_piece_m3=1.5e-6;
            if(physical_parents(k,entry->shell,input.physical_parents)){
                entry->job=gfy_live::prepare_separation_async(snapshot,std::move(input));
                if(entry->job){entry->checked_revision=snapshot->stamp().revision;
                    Msg("[painter_piece_continuation] id=%u phase=cpu_queued revision=%llu t=%u",object.ID(),entry->checked_revision,Device.dwTimeGlobal);}
                else{entry->retry_at=Device.dwTimeGlobal+250;
                    Msg("[painter_piece_continuation] id=%u phase=cpu_queue_refused revision=%llu reason=%s",object.ID(),snapshot->stamp().revision,gfy_live::last_failure());}
            }else{entry->checked_revision=snapshot->stamp().revision;
                Msg("[painter_piece_continuation] id=%u phase=cpu_refused revision=%llu reason=physical_tree",object.ID(),entry->checked_revision);}
        }
    }
}
}
