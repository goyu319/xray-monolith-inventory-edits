#include "PainterFlightTiming.h"
#include "PainterBurnHost.h"
// Compiled once by PhysicsSkeletonObject.cpp. Hidden native fragment staging;
// a failed or stale ticket never changes the original corpse.
#include "painter_voxel/PainterDamagedFragment.h"
#include "painter_voxel/FlightGroup.h"
#include "painter_voxel/FragmentSpawnLifetime.h"
#include "painter_voxel/FragmentNativeAdmission.h"
#include "painter_voxel/CollisionLimits.h"
#include "painter_voxel/PainterPieceContinuation.h"
#include "painter_voxel/SkinEdgeWork.h" // maskdecal: the cut ends' skin edges
#include "PainterCommitParts.h" // piececommit
#include "../Include/xrRender/RuntimeSkinGeometry.h"
#include "../Include/xrRender/RuntimeGeometryStage.h"
#include "../Include/xrRender/RuntimePackedBridge.h"
#include "../Include/xrRender/AnalyticChannelField.h"
#include "../xrphysics/PhysicsShell.h"
#include "../xrphysics/IPHTether.h"
#include "../xrphysics/ExtendedGeom.h"
#include "painter_voxel/PainterExplosions.h"
#include "ai/stalker/ai_stalker.h"
#include "CharacterPhysicsSupport.h"
#include "level.h"
#include "ai_object_location.h"
#include "ai_space.h"
#include "../xrServerEntities/script_engine.h"
#include "xrServer_Objects_ALife.h"
#include "../xrEngine/bone.h"
#include <map>
#include <string>
#include <cmath>
#include "PainterGutSpill_impl.h"

namespace painter_damaged_fragment {
namespace {
constexpr float piece_friction_min=.8f,piece_settle_erp_scale=.2f;
constexpr u32 piece_settle_ms=600;
// Retain the established lower-leg-only velocity cap; contact friction covers all pieces.
bool lower_leg_piece(IKinematics* k,u16 root){
    if(!k||root>=k->LL_BoneCount())return false;const char* name=k->LL_BoneName_dbg(root);
    return name&&(strstr(name,"calf")||strstr(name,"foot")||strstr(name,"toe"));
}
void PiecesContact(bool& do_colide,bool bo1,dContact& c,SGameMtl*,SGameMtl*){
    if(!do_colide)return;
    dxGeomUserData* data=PHRetrieveGeomUserData(bo1?c.geom.g1:c.geom.g2);
    c.surface.mu=_max(c.surface.mu,piece_friction_min);
    const u32 detached=data?u32(reinterpret_cast<std::uintptr_t>(data->callback_data)):0u;
    if(detached&&u32(Device.dwTimeGlobal-(detached&~1u))<piece_settle_ms)c.surface.soft_erp*=piece_settle_erp_scale;
}
// A continuation cut constructs fresh shells. Reapply the same detached-piece contact
// policy, carrying the original settling clock on the retained fragment. Never
// copy arbitrary callback data from another kind of shell.
void apply_piece_contact(CPhysicsShell& shell,IKinematics* k,u16 root,CPhysicsShell* previous=nullptr){
    if(!k||root>=k->LL_BoneCount()||!shell.get_ElementsNumber())return;
    // These are fresh native fragment shells, never the NPC shell. Do not
    // overwrite callback storage owned by a different contact policy.
    for(u16 i=0;i<shell.get_ElementsNumber();++i){
        auto* element=shell.get_ElementByStoreOrder(i);
        if(!element)return;
        auto* callback=element->get_ObjectContactCallback();
        if(callback&&callback!=PiecesContact)return;
    }
    // Keep the source model's sleep thresholds on every fresh fitted child shell.
    IKinematics* piece_k=shell.PKinematics();
    CInifile* piece_ini=piece_k?piece_k->LL_UserData():nullptr;
    SAllDDOParams disable_params;
    disable_params.Load(piece_ini);
    shell.set_DisableParams(disable_params);
    // Match CPHSkeleton::Spawn's authored child flags, including on continuation.
    if(piece_ini&&piece_ini->section_exist("collide_parts")){
        if(piece_ini->line_exist("collide_parts","small_object"))shell.SetSmall();
        if(piece_ini->line_exist("collide_parts","ignore_small_objects"))shell.SetIgnoreSmall();
    }
    u32 detached=Device.dwTimeGlobal|1u;
    if(previous&&previous->get_ElementsNumber()){
        auto* first=previous->get_ElementByStoreOrder(0);
        if(first&&first->get_ObjectContactCallback()==PiecesContact){
            const u32 before=u32(reinterpret_cast<std::uintptr_t>(previous->get_CallbackData()));
            if(before)detached=before;
        }
    }
    shell.add_ObjectContactCallback(PiecesContact);
    shell.set_CallbackData(reinterpret_cast<void*>(std::uintptr_t(detached)));
}
// A stage owns only immutable packet data and unbound device resources.
// Cancel/release on failure, despawn and commit; never retain a model pointer
// for worker access and never wait for the worker on the game thread.
struct RenderTicket {
    std::shared_ptr<const gfy_packed::Mesh> proxy;
    std::shared_ptr<const gfy_packed::Mesh> display()const{return proxy?proxy:mesh;}
    IRuntimeGeometryStage* handle=nullptr;
    std::shared_ptr<const gfy_packed::Mesh> mesh;
    bool attempted=false;
    RenderTicket()=default;
    RenderTicket(const RenderTicket&)=delete;
    RenderTicket& operator=(const RenderTicket&)=delete;
    ~RenderTicket(){reset();}
    void reset(){if(handle){handle->Cancel();handle->Release();handle=nullptr;}mesh.reset();}
    bool ready()const{return attempted&&(!handle||handle->Ready());}
};
struct Stage {
    std::shared_ptr<Prepared> request;
    std::size_t index=0;
    std::string name;
    CPhysicsShellHolder* pose_owner=nullptr;
    CPhysicsSkeletonObject* client=nullptr;
    CPhysicsShell* pose_source=nullptr;
    bool spawned=false, failed=false, canceled=false;
    RenderTicket render;
    std::shared_ptr<skin_edge::Work> edge_body_work,edge_piece_work;
    std::shared_ptr<const gfy_packed::Mesh> edge_body,edge_piece; // maskdecal: the cut ends' packed meshes, until the commit
};
// Reserve immutable native-born all-parts evidence before the visual/physics transaction.
// A failed transfer after publication remains blocked and retries from any surviving part.
template<class ChildIdentity>
std::shared_ptr<painter_explosions::NativeRoute> grenade_route(CPhysicsShellHolder& source,
    const Prepared& req,const xr_vector<std::shared_ptr<Stage>>& stages,
    const gfy_live::Identity& source_identity,u64 source_generation,ChildIdentity child_identity){
    auto native=std::make_shared<painter_explosions::NativeRoute>();
    auto route=std::make_shared<painter_explosions::SplitRoute>();
    route->source=painter_explosions::born(source);route->single_stump_transaction=stages.empty();
    auto add=[&](CPhysicsShellHolder& object,const std::vector<PHFittedBone>& fitted,
        const std::vector<u16>& anchors,const gfy_live::Identity& identity,u64 generation){
        painter_explosions::Recipient recipient;recipient.born=painter_explosions::born(object);
        if(!recipient.born.valid()||recipient.born.world!=route->source.world)return false;
        for(const auto& bone:fitted)recipient.fitted.push_back(bone.bone);recipient.anchors=anchors;
        route->recipients.push_back(std::move(recipient));
        native->witnesses.push_back({painter_explosions::born(object),identity,generation,req.publication_revision,false});return true;};
    if(!add(source,req.stump_physics,req.stump_pose_anchors,source_identity,source_generation))return {};
    for(const auto& stage:stages){if(!stage->client||stage->index>=req.pieces.size())return {};
        const auto& part=req.pieces[stage->index];
        if(!add(*stage->client,part.fitted,part.pose_anchors,child_identity(stage->index),1))return {};}
    native->route=std::move(route);return native;
}
struct Batch {
    std::shared_ptr<Prepared> request;
    xr_vector<RuntimeSkinBone> rig;
    xr_vector<shared_str> textures;
    CPhysicsShell* source_shell=nullptr;
    DWORD thread=0;
    u32 started_at=0;
    xr_vector<std::shared_ptr<Stage>> stages;
    RenderTicket render;
};
// Snapshots outlive the publication batch. Lua runs only after its transaction
// has been retired, so a callback cannot re-enter the same successful commit.
// maskdecal: the verdict of each Lua sever (rest model space: the station's centre, the limb axis), armed with the sever receipt
// (PainterCommittedEffects_impl.h) and taken by the commit of the separation it causes (the bone of the piece; the only one armed)
struct CutArm {u16 bone=BI_NONE;Fvector point,axis;u32 at=0;};
std::map<u16,std::vector<CutArm>> cut_arms;
// Read-only counterpart of take_cut: no expiration erasure or consumption before commit.
bool peek_cut(u16 source_id,u16 bone,CutArm& out){
    const auto it=cut_arms.find(source_id);if(it==cut_arms.end())return false;
    const auto& list=it->second;const u32 now=Device.dwTimeGlobal;
    const CutArm* latest=nullptr;const CutArm* sole=nullptr;std::size_t remaining=0;
    for(const auto& cut:list)if(now-cut.at<=20000u){++remaining;sole=&cut;if(cut.bone==bone)latest=&cut;}
    if(!latest&&remaining==1)latest=sole;if(!latest)return false;out=*latest;return true;
}
skin_edge::WorkKey edge_work_key(u16 bone,const CutArm& cut){
    skin_edge::WorkKey key;key.root_bone=bone;key.cut_bone=cut.bone;key.at=cut.at;
    key.point={cut.point.x,cut.point.y,cut.point.z};key.axis={cut.axis.x,cut.axis.y,cut.axis.z};return key;
}
bool take_cut(u16 source_id,u16 bone,CutArm& out){
    const auto it=cut_arms.find(source_id);if(it==cut_arms.end())return false;
    auto& list=it->second;const u32 now=Device.dwTimeGlobal;
    list.erase(std::remove_if(list.begin(),list.end(),[now](const CutArm& c){return now-c.at>20000u;}),list.end());
    std::ptrdiff_t pick=-1;
    for(std::ptrdiff_t i=std::ptrdiff_t(list.size())-1;i>=0;--i)if(list[std::size_t(i)].bone==bone){pick=i;break;}
    if(pick<0&&list.size()==1)pick=0;
    const bool ok=pick>=0;if(ok){out=list[std::size_t(pick)];list.erase(list.begin()+pick);}
    if(list.empty())cut_arms.erase(it);
    return ok;
}
struct SeverNotice {
    u64 revision=0;
    u32 component=0;
    u16 fragment=BI_NONE;
    shared_str bone;
    Fvector point;
    u16 edge_bone=BI_NONE;std::vector<Fvector> edge_body,edge_piece; // maskdecal: skin edge points (rest model space) of both ends
    bool chunk=false; // chunknosplash: the piece is a burst chunk (sever.chunk in the receipt)
};
std::vector<SeverNotice> committed_notices(const Prepared& request,
    const xr_vector<std::shared_ptr<Stage>>& stages,u64 revision)
{
    std::vector<SeverNotice> notices;
    notices.reserve(stages.size());
    for(const auto& stage:stages){
        if(!stage->client||stage->client->getDestroy())continue;
        const auto& piece=request.pieces[stage->index];
        auto* child=smart_cast<IKinematics*>(stage->client->Visual());
        if(!child||piece.root_bone>=child->LL_BoneCount())continue;
        SeverNotice notice;
        notice.revision=revision;notice.component=piece.component_id;notice.chunk=piece.burst_chunk; // chunknosplash
        notice.fragment=stage->client->ID();notice.bone=child->LL_BoneName_dbg(piece.root_bone);
        Fmatrix bone;
        {xrCriticalSectionGuard guard(&child->UCalc_Mutex);
            child->CalculateBones(TRUE);bone=child->LL_GetTransform(piece.root_bone);}
        stage->client->XFORM().transform_tiny(notice.point,bone.c);
        // maskdecal (sever splashes): where both cut ends' skin ends, measured from this sever's armed verdict
        if(stage->edge_body&&stage->edge_piece){
            CutArm cut;
            if(take_cut(request.source_id,piece.root_bone,cut)){
                const double c[3]={cut.point.x,cut.point.y,cut.point.z},a[3]={cut.axis.x,cut.axis.y,cut.axis.z};
                const auto owner_start=std::chrono::steady_clock::now();const auto key=edge_work_key(piece.root_bone,cut);
                const auto* cached_body=stage->edge_body_work?stage->edge_body_work->Find(*stage->edge_body,key):nullptr;
                const auto* cached_tip=stage->edge_piece_work?stage->edge_piece_work->Find(*stage->edge_piece,key):nullptr;
                skin_edge::Result body_fallback,tip_fallback;
                const auto& body=cached_body?*cached_body:(body_fallback=skin_edge::measure(*stage->edge_body,piece.root_bone,c,a));
                const auto& tip=cached_tip?*cached_tip:(tip_fallback=skin_edge::measure(*stage->edge_piece,piece.root_bone,c,a));
                const double owner_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-owner_start).count();
                notice.edge_bone=piece.root_bone;
                for(const auto& q:body.points){Fvector v;v.set(q[0],q[1],q[2]);notice.edge_body.push_back(v);}
                for(const auto& q:tip.points){Fvector v;v.set(q[0],q[1],q[2]);notice.edge_piece.push_back(v);}
                Msg("[painter_damaged_fragment] id=%u phase=skin_edge bone=%s body_sectors=%u body_back_mm=%.1f piece_sectors=%u piece_back_mm=%.1f tested=%u ms=%.2f worker_cached=%u owner_ms=%.3f",
                    unsigned(request.source_id),notice.bone.c_str(),unsigned(body.points.size()),1000*skin_edge::median(body.back),
                    unsigned(tip.points.size()),1000*skin_edge::median(tip.back),body.tested+tip.tested,body.ms+tip.ms,unsigned(bool(cached_body))+unsigned(bool(cached_tip)),owner_ms);
            }
            stage->edge_body_work.reset();stage->edge_piece_work.reset();
            stage->edge_body.reset();stage->edge_piece.reset();
        }
        notices.push_back(std::move(notice));
    }
    return notices;
}
void notify_committed(CPhysicsShellHolder& original,const std::vector<SeverNotice>& notices)
{
    const u16 owner=original.ID();
    for(const auto& notice:notices){
        auto* source=smart_cast<CPhysicsShellHolder*>(Level().Objects.net_Find(owner));
        if(source!=&original||source->getDestroy())break;
        // Physical separation has committed at the owner-thread geometry
        // publish barrier. Capture actual visual witnesses before Lua effects.
        painter_commit_parts::part("notify_sever"); // piececommit
        painter_commit_parts::part("notify_lua");
        try {
            ::luabind::functor<void> callback;
            if(!ai().script_engine().functor("_G.CPainterVoxelSeverCommitted",callback))return;
            auto sever=::luabind::newtable(ai().script_engine().lua());
            sever["committed"]=true;sever["bone"]=notice.bone.c_str();sever["point"]=notice.point;
            if(notice.chunk)sever["chunk"]=true; // chunknosplash: a burst chunk - painter_sever_effects paints no cut-edge splash on it
            // maskdecal: the cut ends' skin edges in world space now (painter_sever_effects version 3 places its splashes from them)
            if(notice.edge_bone!=BI_NONE&&(!notice.edge_body.empty()||!notice.edge_piece.empty())){
                auto edge=::luabind::newtable(ai().script_engine().lua());bool any=false;
                auto put=[&](const char* key,CObject* object,const std::vector<Fvector>& rest){
                    auto* k=object?smart_cast<IKinematics*>(object->Visual()):nullptr;
                    if(!k||rest.empty()||notice.edge_bone>=k->LL_BoneCount())return;
                    Fmatrix m;{xrCriticalSectionGuard guard(&k->UCalc_Mutex);k->CalculateBones(TRUE);m=k->LL_GetBoneInstance(notice.edge_bone).mRenderTransform;}
                    auto list=::luabind::newtable(ai().script_engine().lua());int n=0;
                    for(const auto& p:rest){Fvector model,world;m.transform_tiny(model,p);object->XFORM().transform_tiny(world,model);list[++n]=world;}
                    edge[key]=list;any=true;};
                put("body",source,notice.edge_body);put("piece",Level().Objects.net_Find(notice.fragment),notice.edge_piece);
                if(any)sever["skin_edge"]=edge;
            }
            auto receipt=::luabind::newtable(ai().script_engine().lua());
            receipt["sequence"]=double(notice.revision);receipt["component_id"]=double(notice.component);
            receipt["fragment_id"]=double(notice.fragment);receipt["sever"]=sever;
            callback(source->lua_game_object(),receipt);
        } catch (...) {
            Msg("[painter_damaged_fragment] id=%u phase=debris_callback_failed fragment=%u",owner,notice.fragment);
        }
    }
}
std::map<u16,std::shared_ptr<Batch>> batches;
std::map<std::string,std::shared_ptr<Stage>> named;
struct TetherArm {CObject* owner=nullptr;u16 bone=BI_NONE;Fvector rest_point;float break_force=0.f;u32 life_ms=0;u32 armed_at=0;};
std::map<u16,TetherArm> tethers; // armed by arm_tether(), consumed by the next commit of that source
struct Canceled {std::string reason;u64 revision=0;gfy_live::Identity owner;u64 expected_generation=0;};
std::map<u16,Canceled> last_canceled;
std::vector<IPHTether*> live_tethers;
u64 serial=0;
constexpr u32 spawn_timeout_ms=10000;
bool finite(const Fvector& v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);}
bool rigid(const Fmatrix& m){
    if(!finite(m.i)||!finite(m.j)||!finite(m.k)||!finite(m.c))return false;
    const float e=2e-3f;
    return _abs(m.i.magnitude()-1)<e&&_abs(m.j.magnitude()-1)<e&&
        _abs(m.k.magnitude()-1)<e&&_abs(m.i.dotproduct(m.j))<e&&
        _abs(m.i.dotproduct(m.k))<e&&_abs(m.j.dotproduct(m.k))<e;
}
// Nearest rotation (+ unchanged origin) of a matrix within 10 % of rigid; deviation = largest axis-length or
// axis-dot error. Handedness follows the input's third axis.
bool orthonormal(Fmatrix& m,float& deviation){
    deviation=0.f;
    if(!finite(m.i)||!finite(m.j)||!finite(m.k)||!finite(m.c))return false;
    deviation=_max(_max(_abs(m.i.magnitude()-1),_abs(m.j.magnitude()-1)),_abs(m.k.magnitude()-1));
    deviation=_max(deviation,_max(_max(_abs(m.i.dotproduct(m.j)),_abs(m.i.dotproduct(m.k))),_abs(m.j.dotproduct(m.k))));
    if(!(deviation<.1f))return false;
    Fvector i=m.i,j=m.j,k;i.normalize();j.mad(i,-i.dotproduct(j));j.normalize();k.crossproduct(i,j);
    if(k.dotproduct(m.k)<0.f)k.invert();
    m.i=i;m.j=j;m.k=k;m._14_=0.f;m._24_=0.f;m._34_=0.f;m._44_=1.f;
    return rigid(m);
}
bool same(const Fmatrix& a,const Fmatrix& b,float e=1e-4f){
    for(unsigned r=0;r<4;++r)for(unsigned c=0;c<4;++c)
        if(_abs(a.m[r][c]-b.m[r][c])>e)return false;
    return true;
}
// valid_piece() with the failing condition named (logged by submit).
const char* piece_problem(const Piece& p,u16 bones){
    if(!p.component_id)return "piece_component";
    if(p.anchor>=bones||p.root_bone>=bones)return "piece_bone_range";
    if(!(p.visible_bones&(1ull<<p.anchor))||!(p.visible_bones&(1ull<<p.root_bone)))return "piece_anchor_not_visible";
    if(p.fitted.empty()||p.fitted.size()>64)return "piece_fitted_count";
    if(!rigid(p.anchor_model)||!rigid(p.anchor_render))return "piece_anchor_not_rigid";
    u64 seen=0;std::size_t total_boxes=0;
    for(const auto& part:p.fitted){
        if(part.bone>=bones||!(p.visible_bones&(1ull<<part.bone))||(seen&(1ull<<part.bone)))return "piece_element_bone";
        if(part.boxes.size()>painter_collision_limits::boxes_per_element)return "piece_element_boxes";
        if(part.retain_original_shape?!part.boxes.empty():part.boxes.empty())return "piece_element_shape";
        seen|=1ull<<part.bone;total_boxes+=part.boxes.size();
        if(total_boxes>painter_collision_limits::boxes_per_shell)return "piece_shell_boxes";
        if(part.retain_original_shape)continue;
        if(!finite(part.mass_center)||!std::isfinite(part.mass)||part.mass<=0.f)return "piece_mass";
        const float* i=part.inertia;const float s=.00001f;
        for(float n:part.inertia)if(!std::isfinite(n))return "piece_inertia_nonfinite";
        if(i[0]<=s||i[4]<=s||i[8]<=s||_abs(i[1]-i[3])>s||_abs(i[2]-i[6])>s||_abs(i[5]-i[7])>s||i[0]*i[4]-i[1]*i[3]<=s*s)return "piece_inertia";
        const double d=double(i[0])*(double(i[4])*i[8]-double(i[5])*i[7])-double(i[1])*(double(i[3])*i[8]-double(i[5])*i[6])+double(i[2])*(double(i[3])*i[7]-double(i[4])*i[6]);
        if(d<=1e-15)return "piece_inertia_det";
        for(const auto& box:part.boxes){const auto& h=box.m_halfsize;
            if(!finite(h)||!finite(box.m_translate)||h.x<.005f||h.y<.005f||h.z<.005f||h.x>.75f||h.y>.75f||h.z>.75f)return "piece_box_size";}
    }
    return nullptr;
}
bool valid_piece(const Piece& p,u16 bones){
    if(!p.component_id||p.anchor>=bones||p.root_bone>=bones||
       !(p.visible_bones&(1ull<<p.anchor))||
       !(p.visible_bones&(1ull<<p.root_bone))||
       p.fitted.empty()||p.fitted.size()>64||
       !rigid(p.anchor_model)||!rigid(p.anchor_render))return false;
    u64 seen=0;std::size_t total_boxes=0;
    for(const auto& part:p.fitted){
        if(part.bone>=bones||!(p.visible_bones&(1ull<<part.bone))||
           (seen&(1ull<<part.bone))||
           part.boxes.size()>painter_collision_limits::boxes_per_element||
           (part.retain_original_shape?!part.boxes.empty():part.boxes.empty()))return false;
        seen|=1ull<<part.bone;
        total_boxes+=part.boxes.size();
        if(total_boxes>painter_collision_limits::boxes_per_shell)return false;
        if(part.retain_original_shape)continue;
        if(!finite(part.mass_center)||!std::isfinite(part.mass)||part.mass<=0.f)return false;
        const float* i=part.inertia;const float s=.00001f;
        for(float n:part.inertia)if(!std::isfinite(n))return false;
        if(i[0]<=s||i[4]<=s||i[8]<=s||_abs(i[1]-i[3])>s||
           _abs(i[2]-i[6])>s||_abs(i[5]-i[7])>s||
           i[0]*i[4]-i[1]*i[3]<=s*s)return false;
        const double d=double(i[0])*(double(i[4])*i[8]-double(i[5])*i[7])-
            double(i[1])*(double(i[3])*i[8]-double(i[5])*i[6])+
            double(i[2])*(double(i[3])*i[7]-double(i[4])*i[6]);
        if(d<=1e-15)return false;
        for(const auto& box:part.boxes){const auto& h=box.m_halfsize;
            if(!finite(h)||!finite(box.m_translate)||h.x<.005f||h.y<.005f||h.z<.005f||
               h.x>.75f||h.y>.75f||h.z>.75f)return false;}
    }
    return true;
}
bool pose_covered(const gfy_packed::Mesh& mesh,const std::vector<PHFittedBone>& fitted,
    const std::vector<u16>& anchors,u16 bones,u64& missing){
    if(anchors.size()!=bones||bones>64)return false;
    u64 represented=0;for(const auto& element:fitted)represented|=1ull<<element.bone;
    missing=0;
    for(const auto& child:mesh.children)for(const auto& v:child.vertices){
        float total=0.f;
        for(unsigned i=0;i<4;++i){
            const float w=v.weight[i];
            if(!std::isfinite(w)||(w<0.f&&!gfy_signed_hw::valid4(v.weight,true)))return false;
            if(w==0.f)continue;
            const u16 b=v.bone[i];if(b>=bones)return false;
            total+=w;
            if(represented&(1ull<<b))continue;
            const u16 anchor=anchors[b];
            if(anchor>=bones||anchor==b||!(represented&(1ull<<anchor)))return false;
            missing|=1ull<<b;
        }
        if(_abs(total-1.f)>1e-3f)return false;
    }
    return true;
}
std::vector<PHFittedPoseLink> pose_links(u64 missing,
    const std::vector<u16>& anchors,const std::vector<Fmatrix>& model){
    std::vector<PHFittedPoseLink> result;
    for(u16 b=0;b<model.size();++b)if(missing&(1ull<<b)){
        PHFittedPoseLink link;link.bone=b;link.anchor=anchors[b];
        Fmatrix inverse;inverse.invert(model[link.anchor]);
        link.relative.mul_43(inverse,model[b]);result.push_back(link);
    }
    return result;
}
bool resample_shell(CPhysicsShell& target,CPhysicsShell& original,
    const std::vector<Fmatrix>& model,const Fmatrix& world){
    {Fmatrix check=world;float deviation=0.f;if(!orthonormal(check,deviation)||!target.get_ElementsNumber())return false;}
    for(u16 i=0;i<target.get_ElementsNumber();++i){
        auto* element=target.get_ElementByStoreOrder(i);
        if(!element||element->m_SelfID>=model.size())return false;
        auto* source=original.get_Element(element->m_SelfID);
        if(!source)return false;
        Fmatrix desired;desired.mul_43(world,model[element->m_SelfID]);
        {float deviation=0.f;if(!orthonormal(desired,deviation))return false;}
        Fvector linear,angular,new_com;
        source->get_AngularVel(angular);
        element->SetTransform(desired,mh_unspecified);
        element->GetGlobalPositionDynamic(&new_com);
        source->GetPointVel(linear,new_com);
        element->set_LinearVel(linear);element->set_AngularVel(angular);
    }
    Fmatrix root_world,root_inverse,refreshed;
    target.get_ElementByStoreOrder(0)->GetGlobalTransformDynamic(&root_world);
    root_inverse.invert(root_world);
    target.ObjectInRoot().mul_43(root_inverse,world);
    target.GetGlobalTransformDynamic(&refreshed);
    return same(refreshed,world,1e-3f)&&P_refresh_FittedPose(&target);
}
void destroy_child(CAI_Stalker* source,Stage& stage){
    stage.canceled=true;stage.render.reset();
    if(stage.client&&!stage.client->getDestroy()&&g_pGameLevel){
        NET_Packet p;
        if(source){source->u_EventGen(p,GE_DESTROY,stage.client->ID());source->u_EventSend(p);}
        else {stage.client->u_EventGen(p,GE_DESTROY,stage.client->ID());stage.client->u_EventSend(p);}
    }
}
void discard_fitted(CPhysicsShell*& fitted,CPhysicsShell& source){
    // Deactivation clears callbacks on the shared visual even when this
    // staged shell never installed its own. Restore the surviving source.
    // It overlaps the source: stop its collision so the deactivation touch step does not collide the two.
    if(fitted)fitted->DisableCollision();
    destroy_physics_shell(fitted);
    source.EnabledCallbacks(TRUE);
    if(auto* k=source.PKinematics())k->CalculateBones_Invalidate();
}
}
}
struct SPainterDamagedFragmentData { std::shared_ptr<painter_damaged_fragment::Stage> stage; };
namespace painter_damaged_fragment {

bool pending(u16 source_id){return batches.find(source_id)!=batches.end();}
bool take_canceled(u16 source_id,const gfy_live::Identity& identity,const char*& reason,u64& revision){
    auto it=last_canceled.find(source_id);if(it==last_canceled.end())return false;
    // Entity IDs are reused. Only the canceled request's exact owner may retry it.
    if(!(it->second.owner==identity)){last_canceled.erase(it);return false;}
    // expected_generation is recorded for provenance; newer accepted wounds of
    // the same owner must still consume this cancellation under the revision rule.
    static std::string keep;keep=it->second.reason;reason=keep.c_str();revision=it->second.revision;last_canceled.erase(it);return true;
}
void arm_tether(u16 source_id,u16 bone,const Fvector& rest_point,float break_force,u32 life_ms){
    if(!finite(rest_point)||!std::isfinite(break_force)||break_force<=0.f||!life_ms)return;
    TetherArm arm;arm.owner=Level().Objects.net_Find(source_id);if(!arm.owner)return;
    arm.bone=bone;arm.rest_point=rest_point;arm.break_force=break_force;arm.life_ms=life_ms;arm.armed_at=Device.dwTimeGlobal;
    tethers[source_id]=arm;
}
void arm_cut(u16 source_id,u16 bone,const Fvector& rest_point,const Fvector& rest_axis){ // maskdecal
    if(!finite(rest_point)||!finite(rest_axis)||rest_axis.square_magnitude()<1e-8f)return;
    const u32 now=Device.dwTimeGlobal; // verdicts never taken (no separation followed) go after 20 s
    for(auto it=cut_arms.begin();it!=cut_arms.end();)if(it->second.empty()||now-it->second.back().at>20000u)it=cut_arms.erase(it);else ++it;
    auto& list=cut_arms[source_id];if(list.size()>=8)list.erase(list.begin());
    CutArm c;c.bone=bone;c.point=rest_point;c.axis=rest_axis;c.at=now;list.push_back(c);
}
// A hanging head is a required attachment, so acquire its real native joint
// against the future stump BEFORE exposing either visual. Failure rolls back
// the existing transaction rather than silently dropping a promised remnant.
struct HeadTetherTicket {
    IPHTether* value=nullptr;
    u16 source=BI_NONE,bone=BI_NONE;u32 armed_at=0;
    ~HeadTetherTicket(){if(value)phtether_destroy(value);}
    HeadTetherTicket()=default;HeadTetherTicket(const HeadTetherTicket&)=delete;
    void commit(){if(!value)return;live_tethers.push_back(value);value=nullptr;
        auto it=tethers.find(source);if(it!=tethers.end()&&it->second.bone==bone&&it->second.armed_at==armed_at)tethers.erase(it);}
};
inline bool prepare_head_tether(CPhysicsShellHolder& source,IKinematics& k,CPhysicsShell& future,
    const Prepared& req,const xr_vector<std::shared_ptr<Stage>>& stages,HeadTetherTicket& ticket){
    const auto found=tethers.find(source.ID());if(found==tethers.end())return true;
    const auto& arm=found->second;
    if(arm.owner!=&source||Device.dwTimeGlobal-arm.armed_at>=60000)return true;
    if(arm.bone>=k.LL_BoneCount())return false;
    const char* name=k.LL_BoneName_dbg(arm.bone);
    if(!name||(strcmp(name,"bip01_head")&&strcmp(name,"bip01_neck")))return true;
    auto exact=[](CPhysicsShell* shell,u16 bone){auto* element=shell?shell->get_Element(bone):nullptr;return element&&element->m_SelfID==bone;};
    for(const auto& stage:stages){
        const auto& piece=req.pieces[stage->index];u16 b=piece.root_bone;bool below=false;
        if(piece.ocular_remnant||piece.destroy_on_commit)continue;
        for(unsigned steps=0;b!=BI_NONE&&b<k.LL_BoneCount()&&steps<64;++steps){if(b==arm.bone){below=true;break;}b=k.LL_GetData(b).GetParentID();}
        if(!below)continue;
        u16 stump_bone=arm.bone;
        for(unsigned steps=0;stump_bone!=BI_NONE&&stump_bone<k.LL_BoneCount()&&!exact(&future,stump_bone)&&steps<64;++steps)
            stump_bone=k.LL_GetData(stump_bone).GetParentID();
        auto* child=stage->client?stage->client->PPhysicsShell():nullptr;
        if(stump_bone==BI_NONE||!exact(&future,stump_bone)||!exact(child,piece.root_bone))return false;
        Fvector model={0,0,0},world={0,0,0};
        {xrCriticalSectionGuard guard(&k.UCalc_Mutex);k.CalculateBones(TRUE);
            k.LL_GetBoneInstance(arm.bone).mRenderTransform.transform_tiny(model,arm.rest_point);}
        source.XFORM().transform_tiny(world,model);if(!finite(world))return false;
        live_tethers.reserve(live_tethers.size()+1);
        ticket.value=phtether_create(&future,stump_bone,child,piece.root_bone,world,arm.break_force,arm.life_ms);
        if(!ticket.value)return false;
        ticket.source=source.ID();ticket.bone=arm.bone;ticket.armed_at=arm.armed_at;return true;
    }
    // The head stayed on the source side. Its original articulation is handled
    // by the cranial support helper; do not attach an arbitrary other fragment.
    return true;
}
void cancel(u16 source_id){
    auto it=batches.find(source_id);if(it==batches.end())return;
    auto* source=smart_cast<CAI_Stalker*>(Level().Objects.net_Find(source_id));
    for(auto& stage:it->second->stages){
        stage->canceled=true;
        destroy_child(source,*stage);
        named.erase(stage->name);
    }
    it->second->render.reset();
    batches.erase(it);
    // The reserved-name rejection in begin_spawn covers any late network child
    // after these tickets are retired. No stale name can exhaust the bound.
}
bool submit(CAI_Stalker& source,const gfy_live::Identity& identity,
    Prepared& prepared,const xr_vector<RuntimeSkinBone>& rig, // piececommit: its meshes are moved into the packets (submit_cpu drops it)
    const xr_vector<shared_str>& textures,CPhysicsShell* source_shell,
    const char*& reason)
{
    reason="invalid_fragment_request";
    auto* k=smart_cast<IKinematics*>(source.Visual());
    // Each refusal names its reason (was one opaque "invalid_fragment_request"); the caller retries the transient ones
    // (moved, stale_generation, shell_changed, not_barrier, physics_busy, busy) with a fresh separation.
    auto refuse=[&](const char* why){reason=why;return false;};
    if(!g_pGameLevel||!pSettings||!pSettings->section_exist("ph_skeleton_object"))return refuse("no_skeleton_section");
    if(!Device.IsGeometryPublishBarrier())return refuse("not_barrier");
    if(Device.Paused()||!physics_world()||physics_world()->Processing())return refuse("physics_busy");
    if(source.g_Alive())return refuse("alive");
    if(source.getDestroy())return refuse("destroyed");
    if(source.ID()!=prepared.source_id||source.cNameVisual()!=prepared.source_visual||!(prepared.stamp.owner==identity)||
       prepared.stamp.expected_generation==0||prepared.stamp.revision==0)return refuse("identity");
    if(!source_shell||source_shell!=source.PPhysicsShell())return refuse("shell_changed");
    if(!source_shell->isFullActive())return refuse("shell_not_active");
    if(!k||k->LL_RenderGeometryGeneration()!=prepared.stamp.expected_generation)return refuse("stale_generation");
    if(rig.size()!=k->LL_BoneCount()||prepared.captured_render_palette.size()!=rig.size())return refuse("rig");
    if(prepared.pieces.empty()||prepared.pieces.size()>12)return refuse("piece_count");
    if(prepared.stump_physics.empty()||prepared.stump_physics.size()>64||!prepared.stump_seed||
       prepared.stump_root_bone>=k->LL_BoneCount()||!(prepared.stump_visible_bones&(1ull<<prepared.stump_root_bone)))return refuse("stump");
    if(!rigid(prepared.source_world))return refuse("source_world");
    {Fmatrix current=source.XFORM();float deviation=0.f;
     if(!orthonormal(current,deviation))return refuse("source_world");
     if(!same(current,prepared.source_world,5e-3f))return refuse("moved");} // 5 mm / ~0.3 deg: invisible on the piece
    if(batches.size()>=32||named.size()+prepared.pieces.size()>256||pending(source.ID()))return refuse("busy");
    for(const auto& p:prepared.pieces)
        if(!p.seed)return refuse("piece_seed");
        else if(const char* why=piece_problem(p,u16(rig.size()))){
            Msg("[painter_damaged_fragment] id=%u phase=piece_refused component=%u root=%u reason=%s",source.ID(),p.component_id,unsigned(p.root_bone),why);
            return refuse(why);}
    std::size_t stump_boxes=0;
    for(const auto& bone:prepared.stump_physics){
        stump_boxes+=bone.boxes.size();
        if(bone.bone>=rig.size()||!(prepared.stump_visible_bones&(1ull<<bone.bone))||
           bone.boxes.size()>painter_collision_limits::boxes_per_element||
           stump_boxes>painter_collision_limits::boxes_per_shell||
           (bone.retain_original_shape?!bone.boxes.empty():bone.boxes.empty())||
           (!bone.retain_original_shape&&bone.mass<=0.f))return false;
    }
    // Every source child has an explicit decision in both stump and fragments.
    // piececommit: the meshes are MOVED into the packets (were copied twice: here and in build_part_packets); nothing reads
    // prepared's meshes after this (the batch request below takes the packets). Same packets, same refusals.
    std::vector<gfy_packed::Mesh> raw;raw.reserve(1+prepared.pieces.size());raw.push_back(std::move(prepared.stump));
    std::vector<std::vector<part_child_ownership::Decision>> decisions{prepared.stump_children};
    for(auto& p:prepared.pieces){raw.push_back(std::move(p.mesh));decisions.push_back(p.children);}
    if(has_uncertified_original_accessory(decisions)){
        reason="unsupported_accessory_bone_palette";return false;
    }
    if(const char* issue=part_child_ownership::validate_part_packets(
        raw,prepared.original_roles,decisions)){
        reason=issue;return false;
    }
    auto packets=part_child_ownership::build_part_packets(
        std::move(raw),prepared.original_roles,decisions); // piececommit: taken over, not copied
    // Low-poly pieces carry immutable cut and skin fields; every publication admits them under its own
    // identity (stump: source cookie/generation; piece i: cookie+i+1, generation 0; same damage revision).
    auto restamp=[&](gfy_packed::Mesh& mesh,u64 cookie,u64 generation){
        if(mesh.analytic_channels){
            auto field=std::make_shared<painter_detail::AnalyticChannelField>(*mesh.analytic_channels);
            field->identity.cookie=cookie;field->identity.generation=generation;field->identity.revision=prepared.stamp.revision;
            mesh.analytic_channels=std::move(field);
        }
        if(mesh.skin_ownership){
            auto field=std::make_shared<painter_detail::SkinOwnershipField>(*mesh.skin_ownership);
            field->identity.cookie=cookie;field->identity.generation=generation;field->identity.revision=prepared.stamp.revision;
            mesh.skin_ownership=std::move(field);
        }
    };
    restamp(packets[0],prepared.stamp.owner.spawn_cookie,prepared.stamp.expected_generation);
    for(std::size_t i=0;i<prepared.pieces.size();++i)restamp(packets[i+1],prepared.stamp.owner.spawn_cookie+i+1,0);
    u64 missing=0;
    if(!pose_covered(packets[0],prepared.stump_physics,
       prepared.stump_pose_anchors,u16(rig.size()),missing)){
        reason="stump_pose_ownership";return false;
    }
    for(std::size_t i=0;i<prepared.pieces.size();++i)
        if(!pose_covered(packets[i+1],prepared.pieces[i].fitted,
           prepared.pieces[i].pose_anchors,u16(rig.size()),missing)){
            reason="fragment_pose_ownership";return false;
        }
    auto batch=std::make_shared<Batch>();batch->request=std::make_shared<Prepared>(prepared);
    batch->request->stump=std::move(packets[0]);
    for(std::size_t i=0;i<batch->request->pieces.size();++i)
        batch->request->pieces[i].mesh=std::move(packets[i+1]);
    batch->rig=rig;batch->textures=textures;batch->source_shell=source_shell;
    batch->thread=GetCurrentThreadId();batch->started_at=Device.dwTimeGlobal;
    // The renderer may drop its last reference on its worker. Give it ONLY
    // plain packed geometry, never Prepared (shared_str/native session owners).
    auto body_packet=std::make_shared<gfy_packed::Mesh>(std::move(batch->request->stump));
    body_packet->post_work.reset();batch->render.mesh=body_packet;
    std::vector<skin_edge::WorkKey> body_keys;
    for(std::size_t i=0;i<batch->request->pieces.size();++i){
        const auto& part=batch->request->pieces[i];
        if(part.destroy_on_commit)continue;
        auto stage=std::make_shared<Stage>();stage->request=batch->request;stage->index=i;
        auto piece_packet=std::make_shared<gfy_packed::Mesh>(std::move(batch->request->pieces[i].mesh));
        piece_packet->post_work.reset();stage->render.mesh=piece_packet;
        CutArm edge_cut;
        if(peek_cut(source.ID(),part.root_bone,edge_cut)){
            const auto key=edge_work_key(part.root_bone,edge_cut);body_keys.push_back(key);
            stage->edge_piece_work=std::make_shared<skin_edge::Work>(piece_packet.get(),std::vector<skin_edge::WorkKey>{key});
            piece_packet->post_work=stage->edge_piece_work;
        }
        if(part.flight_visual)stage->render.proxy=painter_flight::proxy_packet(*part.flight_visual,
            prepared.original_roles.size(),prepared.stamp.owner.spawn_cookie+i+1,prepared.stamp.revision,piece_packet);
        stage->name="painter_damaged_"+std::to_string(++serial);
        if(serial==u64(-1)){reason="fragment_serial_exhausted";cancel(source.ID());return false;}
        stage->pose_source=source_shell;
        stage->pose_owner=&source;
        named.emplace(stage->name,stage);batch->stages.push_back(stage);
    }
    if(!body_keys.empty()){
        auto body_work=std::make_shared<skin_edge::Work>(body_packet.get(),std::move(body_keys));
        body_packet->post_work=body_work;
        for(const auto& stage:batch->stages)if(stage->edge_piece_work)stage->edge_body_work=body_work;
    } // All keys and metadata frozen before any renderer queue submission; no const-object mutation.
    batches.emplace(source.ID(),batch);
    for(const auto& stage:batch->stages){stage->edge_body=batch->render.mesh;stage->edge_piece=stage->render.mesh;} // maskdecal: skin edges at the commit
    for(const auto& stage:batch->stages){
        const auto& part=batch->request->pieces[stage->index];
        CSE_Abstract* se=Level().spawn_item("ph_skeleton_object",source.Position(),
            source.ai_location().level_vertex_id(),u16(-1),true);
        auto* po=smart_cast<CSE_ALifePHSkeletonObject*>(se);
        if(!po){if(se)F_entity_Destroy(se);reason="fragment_server_class";cancel(source.ID());return false;}
        po->set_visual(*prepared.source_visual);
        po->set_name_replace(stage->name.c_str());
        po->source_id=BI_NONE;
        po->saved_bones.root_bone=part.root_bone;
        po->saved_bones.bones_mask=part.visible_bones;
        po->_flags.set(CSE_PHSkeleton::flActive,FALSE);
        po->_flags.set(CSE_PHSkeleton::flNotSave,TRUE); // R30: the visual is runtime geometry a save cannot bring back (net_Spawn failed at load); never written to one
        prepared.source_world.getHPB(po->o_Angle);
        po->o_Position=prepared.source_world.c;
        NET_Packet packet;po->Spawn_Write(packet,TRUE);
        Level().Send(packet,net_flags(TRUE));
        F_entity_Destroy(se);
    }
    reason="staged_hidden";
    return true;
}
int begin_spawn(CPhysicsSkeletonObject& piece,CSE_Abstract* se,SPainterDamagedFragmentData*& data){
    if(!se||data)return -1;
    auto it=named.find(se->name_replace());
    if(it==named.end())return reserved_fragment_name(se->name_replace())?-1:0;
    auto stage=it->second;
    if(stage->canceled){named.erase(it);return -1;}
    if(stage->spawned||stage->failed||stage->client||
       piece.cNameSect()!=shared_str("ph_skeleton_object")||
       piece.cNameVisual()!=stage->request->source_visual||
       piece.getDestroy()||!piece.Visual()||
       !smart_cast<IKinematics*>(piece.Visual())||
       piece.ID()==stage->request->source_id){
        Msg("[painter_damaged_fragment] piece=%s phase=spawn_refused reason=identity spawned=%u failed=%u client=%u section_ok=%u visual_ok=%u destroy=%u kinematics=%u",
            se->name_replace(),unsigned(stage->spawned),unsigned(stage->failed),unsigned(stage->client!=nullptr),
            unsigned(piece.cNameSect()==shared_str("ph_skeleton_object")),unsigned(piece.cNameVisual()==stage->request->source_visual),
            unsigned(piece.getDestroy()),unsigned(piece.Visual()&&smart_cast<IKinematics*>(piece.Visual())));
        stage->failed=true;return -1;}
    auto* po=smart_cast<CSE_ALifePHSkeletonObject*>(se);
    // saved_bones is serialized only with flSavedData (CSE_PHSkeleton::STATE_Write), so after the spawn packet's server
    // round trip it holds defaults; the staged request found by the unique name carries the piece's root and mask.
    if(!po||po->source_id!=BI_NONE||po->_flags.test(CSE_PHSkeleton::flSpawnCopy|CSE_PHSkeleton::flSavedData)){
        Msg("[painter_damaged_fragment] piece=%s phase=spawn_refused reason=server_entity class=%u source_id=%u flags=%u",
            se->name_replace(),unsigned(po!=nullptr),po?unsigned(po->source_id):0u,po?unsigned(po->_flags.flags):0u);
        stage->failed=true;return -1;
    }
    auto* k=smart_cast<IKinematics*>(piece.Visual());
    if(k->LL_BoneCount()!=stage->request->captured_render_palette.size()){
        Msg("[painter_damaged_fragment] piece=%s phase=spawn_refused reason=bone_count piece=%u captured=%u",
            se->name_replace(),unsigned(k->LL_BoneCount()),unsigned(stage->request->captured_render_palette.size()));
        stage->failed=true;return -1;
    }
    data=xr_new<SPainterDamagedFragmentData>();data->stage=stage;
    piece.XFORM()=stage->request->source_world;
    return 1;
}
bool build_shell(CPhysicsSkeletonObject& piece,SPainterDamagedFragmentData* data){
    if(!data||!data->stage||data->stage->canceled)return false;
    painter_flight_perf::Scope flightTime("spawn_shell");
    const auto& part=data->stage->request->pieces[data->stage->index];
    auto* k=smart_cast<IKinematics*>(piece.Visual());if(!k)return false;
    auto* source=data->stage->pose_owner;
    if(!source||source->getDestroy()||source->ID()!=data->stage->request->source_id||
       source->PPhysicsShell()!=data->stage->pose_source){
        Msg("[painter_damaged_fragment] piece=%u phase=shell_refused reason=source owner=%u destroy=%u id_ok=%u shell_same=%u",
            piece.ID(),unsigned(source!=nullptr),source?unsigned(source->getDestroy()):0u,
            source?unsigned(source->ID()==data->stage->request->source_id):0u,source?unsigned(source->PPhysicsShell()==data->stage->pose_source):0u);
        return false;}
    if(auto* npc=smart_cast<CAI_Stalker*>(source))if(npc->g_Alive()){
        Msg("[painter_damaged_fragment] piece=%u phase=shell_refused reason=source_alive",piece.ID());return false;}
    CPhysicsShell* shell=P_prepare_FittedShell(&piece,data->stage->pose_source,
        part.fitted,part.visible_bones,true,part.root_bone,data->stage->render.mesh.get());
    if(!shell){Msg("[painter_damaged_fragment] piece=%u phase=shell_refused reason=fitted_shell elements=%u refusal=%s",piece.ID(),unsigned(part.fitted.size()),P_fitted_shell_refusal());return false;}
    piece.m_pPhysicsShell=shell;
    const bool ok=shell->isActive()&&shell->get_ElementsNumber()==part.fitted.size();
    Msg("[painter_damaged_fragment] piece=%u phase=%s active=%u elements=%u expected=%u root=%u",piece.ID(),ok?"shell_built":"shell_refused",
        unsigned(shell->isActive()),unsigned(shell->get_ElementsNumber()),unsigned(part.fitted.size()),unsigned(part.root_bone));
    return ok;
}
void end_spawn(CPhysicsSkeletonObject& piece,SPainterDamagedFragmentData* data,bool success){
    if(!data||!data->stage)return;
    auto& stage=*data->stage;
    if(success){
        auto* k=smart_cast<IKinematics*>(piece.Visual());
        auto* source_k=stage.pose_source?stage.pose_source->PKinematics():nullptr;
        if(!k||!source_k)success=false;
        else {k->LL_SetBoneRoot(source_k->LL_GetBoneRoot());
            k->LL_SetBonesVisible(u64(-1));}
    }
    stage.spawned=success;stage.failed=!success;stage.client=success?&piece:nullptr;
}
void gone(CPhysicsSkeletonObject& piece,SPainterDamagedFragmentData*& data){
    if(!data)return;
    if(data->stage&&data->stage->client==&piece){data->stage->client=nullptr;data->stage->failed=true;}
    xr_delete(data);
}
bool poll(CAI_Stalker& source,const gfy_live::Identity& identity,u64 generation,
    const xr_vector<RuntimeSkinBone>& rig,const xr_vector<shared_str>& textures,
    CPhysicsShell* source_shell)
{
    auto it=batches.find(source.ID());if(it==batches.end())return false;
    painter_flight_perf::Scope flightTime("body_commit");
    painter_commit_parts::Resume resume_part("fragment_poll");painter_commit_parts::part("fp_check"); // piececommit: the parts of this commit
    auto& batch=*it->second;auto* k=smart_cast<IKinematics*>(source.Visual());
    if(!Device.IsGeometryPublishBarrier()||GetCurrentThreadId()!=batch.thread)return false;
    const auto& req=*batch.request;
    auto reject=[&](const char* reason){
        Msg("[painter_damaged_fragment] id=%u phase=canceled revision=%llu reason=%s t=%u",
            source.ID(),req.stamp.revision,reason,Device.dwTimeGlobal);
        try{last_canceled[source.ID()]={reason,req.stamp.revision,req.stamp.owner,req.stamp.expected_generation};}catch(...){} // R33: the owner re-runs its check
        cancel(source.ID());return false;
    };
    if(source.getDestroy()||source.g_Alive()||!k||source.cNameVisual()!=req.source_visual||
       source_shell!=batch.source_shell||!source_shell||!source_shell->isFullActive()||
       !(req.stamp.owner==identity)||generation!=req.stamp.expected_generation||
       k->LL_RenderGeometryGeneration()!=generation||rig.size()!=batch.rig.size()){
        return reject("source_identity_or_generation");
    }
    if(req.flight&&!k->LL_FlightPreviewAllowed())return reject("flight_donor_changed");
    if(fragment_spawn_expired(Device.dwTimeGlobal,batch.started_at,spawn_timeout_ms)){
        Msg("[painter_damaged_fragment] id=%u phase=staging_timeout revision=%llu",
            source.ID(),req.stamp.revision);
        return reject("spawn_timeout");
    }
    // Geometry conversion, face lists, projection sources and VB/IB creation
    // use the existing renderer worker. Tickets hold moved plain-data meshes:
    // no full mesh copy and no request or live skeleton access from the worker.
    auto queue_render=[&](RenderTicket& ticket,IKinematics& owner,u64 owner_cookie,u64 expected){
        if(ticket.attempted)return true;
        if(!ticket.mesh)return false;
        RuntimeSkinUpdate header;header.owner_cookie=owner_cookie;
        header.expected_geometry_generation=expected;header.damage_revision=req.stamp.revision;
        header.separation_packet=true;header.flight_preview=bool(ticket.proxy);header.exact_rig=batch.rig;
        int outcome=RuntimeStageOutcome::unavailable;
        ticket.handle=owner.LL_StageRuntimeGeometry(header,ticket.display(),
            "models\\model","wm\\painter_stump_limb",batch.textures.empty()?nullptr:&batch.textures,owner_cookie,outcome,k->LL_PainterBurnStageSource());
        ticket.attempted=true;
        return ticket.handle||outcome==RuntimeStageOutcome::unavailable;
    };
    const u64 render_cookie=req.stamp.owner.spawn_cookie;
    if(!queue_render(batch.render,*k,render_cookie,generation))return reject("stump_render_stage");
    bool render_ready=batch.render.ready();
    for(const auto& stage:batch.stages){
        if(stage->failed||stage->canceled)return reject("child_spawn_failed");
        if(!stage->spawned||!stage->client){render_ready=false;continue;}
        auto* child=smart_cast<IKinematics*>(stage->client->Visual());
        if(!child||stage->client->getDestroy()||child->LL_RenderGeometryGeneration()!=0||
           !queue_render(stage->render,*child,render_cookie+stage->index+1,0)){
            return reject("child_identity_or_render_stage");
        }
        render_ready=stage->render.ready()&&render_ready;
    }
    if(!render_ready)return false;
    PainterBurnPublication burnPublication(k);
    if(!burnPublication.ready())return false;
    // All heavy render preparation is ready. Resample the current pose now,
    // then keep the existing native shell/mark/geometry transaction unchanged.
    painter_commit_parts::part("fp_bones");
    std::vector<Fmatrix> current_palette(k->LL_BoneCount());
    std::vector<Fmatrix> current_model(k->LL_BoneCount());
    {xrCriticalSectionGuard guard(&k->UCalc_Mutex);
        k->CalculateBones_Invalidate();k->CalculateBones(TRUE);
        for(u16 b=0;b<k->LL_BoneCount();++b){
            current_palette[b]=k->LL_GetBoneInstance(b).mRenderTransform;
            current_model[b]=k->LL_GetBoneInstance(b).mTransform;
        }
    }
    Fmatrix current_world=source.XFORM();
    {float deviation=0.f;if(!orthonormal(current_world,deviation))return reject("source_world");}
    painter_commit_parts::part("fp_piece_shells");
    // The source may have moved during network spawn. Resample every retained
    // native element at this owner barrier while children are still hidden.
    for(const auto& stage:batch.stages){
        auto* child=stage->client;auto* shell=child->PPhysicsShell();
        if(!shell||shell->get_ElementsNumber()!=req.pieces[stage->index].fitted.size()){
            return reject("child_shell_elements");}
        child->XFORM()=source.XFORM();
        if(!resample_shell(*shell,*source_shell,current_model,source.XFORM())){
            return reject("child_pose_resample");
        }
        // Start from the source element's own motion, not its spin carried out to the piece's centre (the lever
        // flung severed feet), and capped; the source shell is still alive here (the stump swap destroys it).
        const auto* blast=painter_explosions::blast_on(source.ID());
        if(blast){ // torn off by a blast: thrown out from it (3-9 m/s with its strength, lifted, spinning)
            for(u16 e=0;e<shell->get_ElementsNumber();++e){auto* element=shell->get_ElementByStoreOrder(e);if(!element)continue;
                Fvector com,out,v,w;element->GetGlobalPositionDynamic(&com);out.sub(com,blast->pos);out.y=_max(out.y,0.f)+.35f*out.magnitude();
                if(out.square_magnitude()<1e-6f)out.set(0.f,1.f,0.f);out.normalize_safe();element->get_LinearVel(v);
                v.mad(out,3.f+6.f*_min(1.f,blast->strength));element->set_LinearVel(v);
                const float spin=4.f+8.f*_min(1.f,blast->strength);w.set(out.z*spin,spin*.5f,-out.x*spin);element->set_AngularVel(w);}}
        else if(lower_leg_piece(k,req.pieces[stage->index].root_bone))
        for(u16 e=0;e<shell->get_ElementsNumber();++e){auto* element=shell->get_ElementByStoreOrder(e);
            auto* origin=element?source_shell->get_Element(element->m_SelfID):nullptr;if(!origin)continue;
            Fvector linear,angular;origin->get_LinearVel(linear);origin->get_AngularVel(angular);
            if(linear.magnitude()>1.5f)linear.normalize_safe().mul(1.5f);
            if(angular.magnitude()>4.f)angular.normalize_safe().mul(4.f);
            element->set_LinearVel(linear);element->set_AngularVel(angular);}
    }
    painter_commit_parts::part("fp_stump_shell");
    auto* support=source.character_physics_support();
    if(!support||!support->PainterCanSwapFittedShell(source_shell)){
        return reject("source_shell_swap_not_ready");
    }
    CPhysicsShell* fitted=P_prepare_FittedShell(&source,source_shell,
        req.stump_physics,req.stump_visible_bones,false,req.stump_root_bone,batch.render.mesh.get());
    if(!fitted){Msg("[painter_damaged_fragment] id=%u phase=fitted_refused part=stump_shell reason=%s",source.ID(),P_fitted_shell_refusal());return reject("stump_shell_prepare");}
    if(!resample_shell(*fitted,*source_shell,current_model,source.XFORM())){
        discard_fitted(fitted,*source_shell);return reject("stump_pose_resample");
    }
    painter_commit_parts::part("fp_stump_cover");
    bool prepared_all=true;
    const char* prepare_reason="stump_pose_links";
    u64 missing=0;
    if(!pose_covered(*batch.render.mesh,req.stump_physics,req.stump_pose_anchors,
           u16(current_model.size()),missing)||
       !P_bind_FittedPoseLinks(fitted,
           pose_links(missing,req.stump_pose_anchors,current_model),false))prepared_all=false;
    const u64 cookie=req.stamp.owner.spawn_cookie;
    for(const auto& stage:batch.stages){
        if(!prepared_all)break;
        painter_commit_parts::part("fp_piece_cover");
        prepare_reason="child_pose_links";
        auto* child=smart_cast<IKinematics*>(stage->client->Visual());
        const auto& part=req.pieces[stage->index];
        if(!child||child->LL_RenderGeometryGeneration()!=0){prepared_all=false;break;}
        auto* child_shell=stage->client->PPhysicsShell();
        if(!child_shell||!pose_covered(*stage->render.mesh,part.fitted,part.pose_anchors,
               u16(current_model.size()),missing)){
            prepared_all=false;break;
        }
        // The published packet has no old whole-body faces. Its numeric skin
        // palette remains visible, including supported cross-element weights.
        child->LL_SetBonesVisible(u64(-1));
        if(!P_bind_FittedPoseLinks(child_shell,
               pose_links(missing,part.pose_anchors,current_model),true)){
            prepared_all=false;break;
        }
        u64 check_mask=missing;
        for(const auto& element:part.fitted)check_mask|=1ull<<element.bone;
        {xrCriticalSectionGuard guard(&child->UCalc_Mutex);
            child->CalculateBones_Invalidate();child->CalculateBones(TRUE);
            for(u16 b=0;b<child->LL_BoneCount();++b)
                if(check_mask&(1ull<<b)){
                    // R33: 5 mm / ~0.3 deg, the bound "moved" uses above (invisible on the piece); at 1 mm a hand piece's
                    // small rotation error, amplified along the fingers, canceled whole splits (09:13 run 19908, 35 s whole).
                    const Fmatrix& have=child->LL_GetBoneInstance(b).mRenderTransform;
                    if(!same(have,current_palette[b],5e-3f)){
                        float worst=0.f;for(unsigned r=0;r<4;++r)for(unsigned c=0;c<4;++c)worst=_max(worst,_abs(have.m[r][c]-current_palette[b].m[r][c]));
                        prepared_all=false;prepare_reason="child_pose_mismatch";
                        Msg("[painter_damaged_fragment] id=%u phase=pose_mismatch piece=%u bone=%u revision=%llu max_abs=%.4f",
                            source.ID(),stage->client->ID(),unsigned(b),req.stamp.revision,worst);
                    }
                }}
        if(!prepared_all)break;
        painter_commit_parts::part("fp_piece_adopt");
        RuntimeSkinUpdate update;update.owner_cookie=cookie+stage->index+1;
        update.expected_geometry_generation=0;update.damage_revision=req.stamp.revision;
        update.separation_packet=true;update.flight_preview=bool(stage->render.proxy);
        update.exact_rig=batch.rig;
        const bool rendered=stage->render.handle?
            child->LL_PrepareStagedRuntimeGeometry(stage->render.handle,update,update.owner_cookie,
                source.Visual(),&stage->client->XFORM()):
            (AppendPackedRuntimeChildren(*stage->render.display(),update,"models\\model","wm\\painter_stump_limb",
                batch.textures.empty()?nullptr:&batch.textures)&&
             child->LL_PrepareRuntimeGeometryWithDonor(update,update.owner_cookie,
                source.Visual(),&stage->client->XFORM()));
        if(!rendered){prepared_all=false;prepare_reason="child_render_prepare";break;}
    }
    painter_commit_parts::part("fp_stump_adopt");
    if(prepared_all){
        prepare_reason="stump_render_prepare";
        RuntimeSkinUpdate update;update.owner_cookie=cookie;
        update.expected_geometry_generation=generation;update.damage_revision=req.stamp.revision;
        update.separation_packet=true;
        update.exact_rig=batch.rig;
        prepared_all=batch.render.handle?
            k->LL_PrepareStagedRuntimeGeometry(batch.render.handle,update,cookie,nullptr,nullptr):
            (AppendPackedRuntimeChildren(*batch.render.mesh,update,"models\\model","wm\\painter_stump_limb",
                batch.textures.empty()?nullptr:&batch.textures)&&k->LL_PrepareRuntimeGeometry(update,cookie));
    }
    if(!prepared_all){
        k->LL_DiscardPreparedRuntimeGeometry();
        for(const auto& stage:batch.stages)if(stage->client){auto* child=smart_cast<IKinematics*>(stage->client->Visual());
            if(child)child->LL_DiscardPreparedRuntimeGeometry();}
        discard_fitted(fitted,*source_shell);return reject(prepare_reason);
    }
    painter_commit_parts::part("fp_preflight");
    if(!k->LL_CanCommitPreparedRuntimeGeometry(cookie))prepared_all=false;
    for(const auto& stage:batch.stages){auto* child=smart_cast<IKinematics*>(stage->client->Visual());
        if(!child||!child->LL_CanCommitPreparedRuntimeGeometry(cookie+stage->index+1))
            prepared_all=false;}
    if(!prepared_all){
        k->LL_DiscardPreparedRuntimeGeometry();
        for(const auto& stage:batch.stages)if(stage->client){auto* child=smart_cast<IKinematics*>(stage->client->Visual());
            if(child)child->LL_DiscardPreparedRuntimeGeometry();}
        discard_fitted(fitted,*source_shell);return reject("render_commit_preflight");
    }
    painter_commit_parts::part("fp_tether_prep");
    HeadTetherTicket head_tether;
    if(!prepare_head_tether(source,*k,*fitted,req,batch.stages,head_tether)){
        k->LL_DiscardPreparedRuntimeGeometry();
        for(const auto& stage:batch.stages)if(stage->client){auto* child=smart_cast<IKinematics*>(stage->client->Visual());if(child)child->LL_DiscardPreparedRuntimeGeometry();}
        discard_fitted(fitted,*source_shell);return reject("head_tether_prepare");
    }
    painter_commit_parts::part("fp_reserve");
    std::vector<CPhysicsShellHolder*> reserved;
    auto reserve_owner=[&](CPhysicsShellHolder& target,
        std::shared_ptr<const gfy_live::PieceSeed> seed,std::shared_ptr<const gfy_packed::Mesh> published,
        gfy_live::Identity owner,u64 projected_generation,const painter_flight::Member* flight=nullptr){
        if(!painter_piece_continuation::reserve(target,std::move(seed),std::move(published),
            std::move(owner),projected_generation,req.continuation_input,
            batch.rig,batch.textures,u16(req.original_roles.size()),
            req.stamp.revision,flight)){
            Msg("[painter_damaged_fragment] id=%u phase=continuation_reserve_refused target=%u shell_full_active=%u revision=%llu",
                source.ID(),target.ID(),unsigned(target.PPhysicsShell()&&target.PPhysicsShell()->isFullActive()),req.stamp.revision);
            return false;
        }
        reserved.push_back(&target);return true;
    };
    bool reserved_all=reserve_owner(source,req.stump_seed,batch.render.mesh,
        req.stamp.owner,generation+1);
    for(const auto& stage:batch.stages){
        if(!reserved_all)break;
        auto child_identity=req.stamp.owner;
        child_identity.spawn_cookie+=stage->index+1;
        const auto& part=req.pieces[stage->index];
        painter_flight::Member flight{req.flight,part.component_id,req.original_roles,part.children};
        reserved_all=reserve_owner(*stage->client,
            req.pieces[stage->index].seed,stage->render.mesh,std::move(child_identity),1,req.flight?&flight:nullptr);
    }
    if(!reserved_all){
        for(auto* owner:reserved)painter_piece_continuation::abandon(*owner);
        k->LL_DiscardPreparedRuntimeGeometry();
        for(const auto& stage:batch.stages)if(stage->client){
            auto* child=smart_cast<IKinematics*>(stage->client->Visual());
            if(child)child->LL_DiscardPreparedRuntimeGeometry();
        }
        discard_fitted(fitted,*source_shell);return reject("continuation_reserve");
    }
    painter_commit_parts::part("fp_route");
    painter_explosions::RouteTicket grenade_ticket;
    bool route_reserved=false;
    try{auto route=grenade_route(source,req,batch.stages,req.stamp.owner,generation+1,[&](std::size_t index){
        auto identity=req.stamp.owner;identity.spawn_cookie+=index+1;return identity;});
        route_reserved=route&&grenade_ticket.reserve(std::move(route));}catch(...){route_reserved=false;}
    if(!route_reserved){for(auto* owner:reserved)painter_piece_continuation::abandon(*owner);
        k->LL_DiscardPreparedRuntimeGeometry();for(const auto& stage:batch.stages)if(auto* child=smart_cast<IKinematics*>(stage->client->Visual()))child->LL_DiscardPreparedRuntimeGeometry();
        discard_fitted(fitted,*source_shell);return reject("grenade_route_reserve");}
    painter_commit_parts::part("fp_render_commit");
    for(const auto& stage:batch.stages){auto* child=smart_cast<IKinematics*>(stage->client->Visual());
        if(!child->LL_CommitPreparedRuntimeGeometry(cookie+stage->index+1)){
            for(auto* owner:reserved)painter_piece_continuation::abandon(*owner);
            k->LL_DiscardPreparedRuntimeGeometry();discard_fitted(fitted,*source_shell);
            return reject("child_render_commit");}}
    if(!k->LL_CommitPreparedRuntimeGeometry(cookie)){
        for(auto* owner:reserved)painter_piece_continuation::abandon(*owner);
        discard_fitted(fitted,*source_shell);return reject("stump_render_commit");
    }
    painter_commit_parts::part("fp_shell_swap");
    support->PainterSwapFittedShell(fitted,source_shell,req.stump_visible_bones);
    support->PainterSetFlesh(req.stump_physics,fitted,&req.stump_flesh_gear); // bodypass: the boxes this shell was built from, for the bullet thread; gearbody: + its merged gear
    head_tether.commit();
    const CGID separated_group=RegisterGroup();fitted->RegisterToCLGroup(separated_group);
    painter_commit_parts::part("fp_activate");
    const bool stump_bound=painter_piece_continuation::activate(
        source,k->LL_RenderGeometryGeneration());
    painter_piece_continuation::set_collision_group(source,separated_group); // R31: later splits keep this group
    bool fragments_bound=true;
    for(const auto& stage:batch.stages){
        auto* shell=stage->client->PPhysicsShell();
        // Every detached part keeps dead-body friction and the existing soft settling policy.
        apply_piece_contact(*shell,k,req.pieces[stage->index].root_bone);
        shell->RegisterToCLGroup(separated_group);shell->EnableCollision();shell->Enable();
        stage->client->PainterInheritThermal(source);
        stage->client->setEnabled(TRUE);stage->client->setVisible(TRUE);
        auto* child_k=smart_cast<IKinematics*>(stage->client->Visual());
        fragments_bound=child_k&&painter_piece_continuation::activate(
            *stage->client,child_k->LL_RenderGeometryGeneration())&&fragments_bound;
        painter_piece_continuation::set_collision_group(*stage->client,separated_group); // R31
        named.erase(stage->name);
    }
    // R63 sever hold: the frozen corpse starts now, with its pieces (same frame). A bullet's fatal impulse goes to the shell owning the
    // hit bone's element - checked across every piece and the stump at one ancestor level at a time, starting at the hit bone - and
    // when several shells own that level (skull fragments share the head element) to the one whose element centre is nearest the
    // struck point. A blast's impulse goes to the stump (its pieces were thrown above). Applied once, by PainterHoldRelease.
    painter_commit_parts::part("fp_hold");
    if(support->PainterHoldActive()){CPhysicsShell* target=nullptr;
        if(const SHit* h=support->PainterHoldHit())if(h->type()!=ALife::eHitTypeExplosion&&h->bone()<k->LL_BoneCount()){
            Fvector struck;{Fmatrix bone_world;bone_world.mul_43(source.XFORM(),k->LL_GetTransform(h->bone()));bone_world.transform_tiny(struck,h->bone_space_position());}
            CPhysicsShell* stump=source.PPhysicsShell();
            u16 b=h->bone();
            for(unsigned steps=0;b!=BI_NONE&&b<k->LL_BoneCount()&&steps<64;++steps,b=k->LL_GetData(b).GetParentID()){
                CPhysicsShell* best=nullptr;float best_d=flt_max;bool owned=false;
                auto consider=[&](CPhysicsShell* sh){if(!sh)return;auto* el=sh->get_Element(b);if(!el)return;owned=true;
                    Fvector c;el->GetGlobalPositionDynamic(&c);const float d=c.distance_to_sqr(struck);if(d<best_d){best_d=d;best=sh;}};
                for(const auto& stage:batch.stages)consider(stage->client?stage->client->PPhysicsShell():nullptr);
                consider(stump);
                if(owned){target=best==stump?nullptr:best;break;}}
        }
        support->PainterHoldRelease("sever_committed",target);}
    if(!stump_bound||!fragments_bound) // the parts stand; a part without a session just takes no more wounds
        Msg("[painter_damaged_fragment] id=%u phase=continuation_unbound stump=%u fragments=%u",source.ID(),unsigned(stump_bound),unsigned(fragments_bound));
    if(painter_burn_host::active()){std::vector<CObject*> parts; for(const auto& s:batch.stages)parts.push_back(s->client);painter_burn_host::split(source,parts);}
    burnPublication.release();
    painter_commit_parts::part("fp_routes");
    grenade_ticket.commit();
    painter_piece_continuation::retry_grenade_routes();
    painter_commit_parts::part("fp_tether");
    // Dangling sever: tether the piece torn off at the armed bone (or below it) to the stump at the tear point.
    for(auto t=live_tethers.begin();t!=live_tethers.end();){if(*t&&(*t)->Alive())++t;else{phtether_destroy(*t);t=live_tethers.erase(t);}}
    if(auto armed=tethers.find(source.ID());armed!=tethers.end()){
        const TetherArm arm=armed->second;tethers.erase(armed);
        CPhysicsShell* stump=source.PPhysicsShell();
        if(stump&&arm.owner==&source&&Device.dwTimeGlobal-arm.armed_at<60000){
            for(const auto& stage:batch.stages){const auto& piece=req.pieces[stage->index];
                if(piece.ocular_remnant||piece.destroy_on_commit)continue;
                u16 b=piece.root_bone;bool below=false;
                for(unsigned steps=0;b!=BI_NONE&&b<k->LL_BoneCount()&&steps<64;++steps){if(b==arm.bone){below=true;break;}b=k->LL_GetData(b).GetParentID();}
                if(!below)continue;
                Fvector model={0,0,0},world={0,0,0};
                {xrCriticalSectionGuard guard(&k->UCalc_Mutex);k->CalculateBones(TRUE);
                    k->LL_GetBoneInstance(arm.bone).mRenderTransform.transform_tiny(model,arm.rest_point);}
                source.XFORM().transform_tiny(world,model);
                u16 sb=arm.bone;for(unsigned steps=0;sb!=BI_NONE&&!stump->get_Element(sb)&&steps<64;++steps)sb=k->LL_GetData(sb).GetParentID();
                if(sb==BI_NONE)break;
                IPHTether* tether=phtether_create(stump,sb,stage->client->PPhysicsShell(),piece.root_bone,world,arm.break_force,arm.life_ms);
                Msg("[painter_damaged_fragment] id=%u phase=tether revision=%llu piece=%u bone=%u stump_bone=%u created=%u",
                    source.ID(),req.stamp.revision,unsigned(stage->index),unsigned(arm.bone),unsigned(sb),unsigned(tether!=nullptr));
                if(tether)live_tethers.push_back(tether);
                break;}
        }
    }
    painter_commit_parts::part("fp_gut");
    // Intestine spill: a split between the pelvis and the chest may spill gut ropes from the cut.
    {xr_vector<painter_gut_spill::Part> parts;
     painter_gut_spill::Part stump;stump.obj=&source;for(const auto& e:req.stump_physics)stump.bones.push_back(e.bone);
     for(u16 b=0;b<req.stump_pose_anchors.size();++b)if(req.stump_pose_anchors[b]!=BI_NONE)stump.bones.push_back(b);
     parts.push_back(std::move(stump));
     for(const auto& stage:batch.stages){const auto& part=req.pieces[stage->index];painter_gut_spill::Part p;p.obj=stage->client;
      for(const auto& e:part.fitted)p.bones.push_back(e.bone);
      for(u16 b=0;b<part.pose_anchors.size();++b)if(part.pose_anchors[b]!=BI_NONE)p.bones.push_back(b);
      parts.push_back(std::move(p));}
     painter_gut_spill::split_committed(source,*k,parts,separated_group,req.gore);}
    // Release worker-side scratch immediately: live piece tickets can outlive
    // this batch through their native fragment userdata.
    painter_commit_parts::part("fp_finish");
    float render_queue_ms=0.f,render_run_ms=0.f,render_adopt_ms=0.f;unsigned render_stages=0;
    auto finish_render=[&](RenderTicket& ticket){if(ticket.handle){
        const auto& timing=ticket.handle->Timing();++render_stages;
        render_queue_ms+=timing.queue_ms;render_run_ms+=timing.run_ms;render_adopt_ms+=timing.install_ms;
        }ticket.reset();};
    finish_render(batch.render);for(const auto& stage:batch.stages)finish_render(stage->render);
    Msg("[painter_damaged_fragment] id=%u phase=render_prepared stages=%u queue_sum_ms=%.3f worker_sum_ms=%.3f adopt_sum_ms=%.3f t=%u tc=%u",
        source.ID(),render_stages,render_queue_ms,render_run_ms,render_adopt_ms,Device.dwTimeGlobal,Device.dwTimeContinual);
    Msg("[painter_damaged_fragment] id=%u phase=committed revision=%llu pieces=%u pose_linked=1 t=%u tc=%u",
        source.ID(),req.stamp.revision,unsigned(batch.stages.size()),Device.dwTimeGlobal,Device.dwTimeContinual);
    painter_commit_parts::part("fp_notices");
    const auto notices=committed_notices(req,batch.stages,req.stamp.revision);
    batches.erase(it);
    painter_commit_parts::part("fp_notify");
    notify_committed(source,notices);
    return true;
}
}
