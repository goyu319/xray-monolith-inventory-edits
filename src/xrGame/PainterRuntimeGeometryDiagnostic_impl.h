// Explicit first-original-GFY renderer diagnostic. No production hit hook.
#include "../Include/xrRender/RuntimeSkinGeometry.h"
#include "../Include/xrRender/RuntimePackedBridge.h"
// Exact declaration from the engine-bundled OpenSSL sha.h; crypto.lib already links SHA256.
// The legacy crypto::xr_sha256 class is a 20-byte SHA-0 implementation. Do not use it.
extern "C" unsigned char* SHA256(const unsigned char* data, size_t length, unsigned char* digest);
#include <array>
// Read-only console status belongs to this bounded diagnostic, not the production API.
// 0 idle, 1 queued, 2 committed, 3 restored, 4 warm, -1 rejected.
int painter_runtime_result=0,painter_runtime_result_owner=65535,painter_runtime_result_state=-1;
int painter_runtime_result_reason=0;
namespace {
void runtime_status(int result,int owner,int state,int reason=0){painter_runtime_result=result;painter_runtime_result_owner=owner;painter_runtime_result_state=state;painter_runtime_result_reason=reason;}

const char* runtime_source_alias="painter\\butcher_review\\gfy_sources\\4be67c0445cef074a61fabfabc76db1c5d0cd73f75486a09e15b197f3e714276";
const char* runtime_source_sha="4be67c0445cef074a61fabfabc76db1c5d0cd73f75486a09e15b197f3e714276";
const char* runtime_state_sha[4]={"69ba8877451a06b15e3bf5f533816b4c74a5a99c43cb9606199387fbb64cddc6","e2392f3944413fee76a40b08553e347c3b0b024e3f81e69a874b0d3ca5c31b5b","85b693394ecba4519645619bad8d6eb87c33e814713171dee63c6ec7ec9d7771","2c8dd6053f96c7b17e07bac71aa07874d0125b33de419da0e179cd0e83a13242"};
std::array<RuntimeSkinUpdate,4> runtime_fixture;
bool runtime_fixture_ready=false;
u64 next_runtime_cookie=1; // Owner-thread-only, never derived from recycled ID/pointer.
bool runtime_digest(IReader& reader,const char* expected)
{
    u8 value[32] = {};
    if (!SHA256(static_cast<const u8*>(reader.pointer()),size_t(reader.length()),value)) return false;
    const char* digits="0123456789abcdef";
    for(unsigned i=0;i<32;++i)if(expected[i*2]!=digits[value[i]>>4] || expected[i*2+1]!=digits[value[i]&15])return false;
    return true;
}
bool read_runtime_fixture(IReader& reader,RuntimeSkinUpdate& output,u32 state)
{
    if(reader.elapsed()<12)return false;char magic[8];reader.r(magic,8);
    if(memcmp(magic,"VXPACK01",8) || reader.r_u32()!=2)return false;
    gfy_packed::Mesh mesh;
    for(unsigned c=0;c<2;++c){
        if(reader.elapsed()<16)return false;gfy_packed::Child child;
        child.child_index=reader.r_u16();child.material_source_child=reader.r_u16();child.material=reader.r_u32();
        u32 vertices=reader.r_u32(),indices=reader.r_u32();
        if(child.child_index!=(c?12:4) || child.material_source_child!=4 || child.material!=c || !vertices || vertices>65535 || !indices || indices%3 || indices/3>65535)return false;
        const u64 bytes=u64(vertices)*sizeof(gfy_packed::Vertex)+u64(indices)*2+u64(indices/3)*4;
        if(bytes>u64(reader.elapsed()))return false;
        child.vertices.resize(vertices);child.indices.resize(indices);child.original_faces.resize(indices/3);
        reader.r(child.vertices.data(),vertices*sizeof(gfy_packed::Vertex));reader.r(child.indices.data(),indices*sizeof(u16));reader.r(child.original_faces.data(),indices/3*sizeof(u32));
        for(u16 index:child.indices)if(index>=vertices)return false;
        mesh.children.push_back(std::move(child));
    }
    if(reader.elapsed()!=0)return false;
    AppendPackedRuntimeChildren(mesh,output,"models\\model","wm\\painter_stump_limb");
    output.damage_revision=state;return true;
}
}
bool painter_warm_runtime_geometry_fixture()
{
    runtime_status(4,65535,-1);
    if(runtime_fixture_ready){Msg("[butcher_runtime_native] fixture=already_warm disk_reads=0");return true;}
    string_path source;xr_strconcat(source,runtime_source_alias,".ogf");
    IReader* original=FS.r_open("$game_meshes$",source);
    const bool source_ok=original && runtime_digest(*original,runtime_source_sha);
    if(original)FS.r_close(original);
    if(!source_ok){runtime_status(-1,65535,-1,10);Msg("[butcher_runtime_native] fixture=rejected reason=original_GFY_bytes");return false;}
    std::array<RuntimeSkinUpdate,4> staged{};
    for(u32 i=0;i<4;++i){string_path file;xr_sprintf(file,"painter\\butcher_runtime\\packed_state_%u.bin",i+1);IReader* reader=FS.r_open("$game_meshes$",file);
        const bool ok=reader && reader->length()<16*1024*1024 && runtime_digest(*reader,runtime_state_sha[i]) && read_runtime_fixture(*reader,staged[i],i+1);
        if(reader)FS.r_close(reader);if(!ok){runtime_status(-1,65535,int(i+1),11);Msg("[butcher_runtime_native] fixture=rejected state=%u reason=hash_or_layout",i+1);return false;}}
    runtime_fixture=std::move(staged);runtime_fixture_ready=true;
    Msg("[butcher_runtime_native] fixture=warm states=4 source_sha=%s vertex_stride=80 per_commit_file_model_reads=0",runtime_source_sha);return true;
}
bool CCharacterPhysicsSupport::QueuePainterRuntimeGeometryProbe(u32 state)
{
    auto reject=[&](int reason,const char* text){runtime_status(-1,m_EntityAlife.ID(),int(state),reason);Msg("[butcher_runtime_native] id=%u state=%u phase=rejected reason=%s reason_code=%d",m_EntityAlife.ID(),state,text,reason);return false;};
    if(!runtime_fixture_ready)return reject(20,"fixture_not_warm");
    if(state>4)return reject(21,"state_range");
    if(m_painter_geometry_pending>=0)return reject(22,"owner_pending");
    if(m_eType!=etStalker || m_EntityAlife.g_Alive() || m_EntityAlife.getDestroy() || !m_pPhysicsShell)return reject(23,"owner_not_dead_shell");
    if((state==0 && m_painter_runtime_state==0) || (state>0 && state<=m_painter_runtime_state))return reject(24,"state_order");
    runtime_status(1,m_EntityAlife.ID(),int(state));
    m_painter_geometry_pending=100+int(state);
    xrCriticalSectionGuard guard(&Device.seqParallelBeforRenderCS);
    Device.seqParallelBeforRender.push_back(xr_make_delegate(this,&CCharacterPhysicsSupport::ApplyPainterGeometryProbe));
    Msg("[butcher_runtime_native] id=%u state=%u phase=queued",m_EntityAlife.ID(),state);return true;
}
void CCharacterPhysicsSupport::ApplyPainterRuntimeGeometryProbe(u32 state)
{
    if(!Device.IsGeometryPublishBarrier() || !runtime_fixture_ready || state>4 || m_eType!=etStalker || m_EntityAlife.g_Alive() || m_EntityAlife.getDestroy() ||
       !m_pPhysicsShell || !m_pPhysicsShell->isActive() || !physics_world() || physics_world()->Processing() || Device.Paused() ||
       static_cast<CPHShell*>(m_pPhysicsShell)->IsFreezed() || !m_weapon_geoms.empty() || m_weapon_attach_bone || m_painter_geometry_installed){
        Msg("[butcher_runtime_native] id=%u state=%u phase=rejected reason=owner_shell_barrier_or_weapon",m_EntityAlife.ID(),state);runtime_status(-1,m_EntityAlife.ID(),int(state),30);return;}
    for(u16 i=0;i<m_pPhysicsShell->get_ElementsNumber();++i)if(!m_pPhysicsShell->get_ElementByStoreOrder(i)->isFullActive()){
        Msg("[butcher_runtime_native] id=%u state=%u phase=rejected reason=activating_body",m_EntityAlife.ID(),state);runtime_status(-1,m_EntityAlife.ID(),int(state),31);return;}
    IKinematics* k=smart_cast<IKinematics*>(m_EntityAlife.Visual());
    const char* visual=*m_EntityAlife.cNameVisual();
    if(!k || k->LL_BoneCount()!=47 || !visual || xr_strcmp(visual,runtime_source_alias)){
        Msg("[butcher_runtime_native] id=%u state=%u phase=rejected reason=source_or_rig",m_EntityAlife.ID(),state);runtime_status(-1,m_EntityAlife.ID(),int(state),32);return;}
    if(!m_painter_runtime_cookie)m_painter_runtime_cookie=next_runtime_cookie++;
    CPhysicsShell* shell=m_pPhysicsShell;const void* bones=&k->LL_GetBoneInstance(0);const void* rig=&k->LL_GetData(0);const u16 root=k->LL_GetBoneRoot();const u64 visible=k->LL_GetBonesVisible();
    xr_vector<CPhysicsElement*> bodies;xr_vector<CPhysicsJoint*> joints;
    for(u16 i=0;i<shell->get_ElementsNumber();++i)bodies.push_back(shell->get_ElementByStoreOrder(i));
    for(u16 i=0;i<shell->get_JointsNumber();++i)joints.push_back(shell->get_JointByStoreOrder(i));
    const u64 before=k->LL_RenderGeometryGeneration();const u32 started=GetTickCount();bool applied=false;
    if(state==0){k->LL_ClearRuntimeGeometry();m_painter_runtime_state=0;applied=k->LL_RenderGeometryGeneration()>before;}
    else {
        RuntimeSkinUpdate update=runtime_fixture[state-1];update.owner_cookie=m_painter_runtime_cookie;update.expected_geometry_generation=before;
        update.exact_rig.resize(k->LL_BoneCount());
        for(u16 b=0;b<k->LL_BoneCount();++b){auto& out=update.exact_rig[b];out.name=k->LL_BoneName_dbg(b);out.parent=k->LL_GetData(b).GetParentID();out.model_to_bone=k->LL_GetData(b).m2b_transform;}
        applied=k->LL_InstallRuntimeGeometry(update,m_painter_runtime_cookie);if(applied)m_painter_runtime_state=state;
    }
    bool unchanged=m_pPhysicsShell==shell && smart_cast<IKinematics*>(m_EntityAlife.Visual())==k && &k->LL_GetBoneInstance(0)==bones && &k->LL_GetData(0)==rig &&
        k->LL_GetBoneRoot()==root && k->LL_GetBonesVisible()==visible && shell->get_ElementsNumber()==bodies.size() && shell->get_JointsNumber()==joints.size();
    for(u16 i=0;unchanged && i<bodies.size();++i)unchanged=shell->get_ElementByStoreOrder(i)==bodies[i];
    for(u16 i=0;unchanged && i<joints.size();++i)unchanged=shell->get_JointByStoreOrder(i)==joints[i];
    Msg("[butcher_runtime_native] id=%u state=%u phase=%s cookie=%llu generation_before=%llu generation_after=%llu identities=%s elapsed_ms=%u frame=%u disk_model_reads=0",
        m_EntityAlife.ID(),state,applied?(state?"committed":"restored"):"rejected",m_painter_runtime_cookie,before,k->LL_RenderGeometryGeneration(),unchanged?"unchanged":"FAILED",GetTickCount()-started,Device.dwFrame);
    runtime_status(applied && unchanged?(state?2:3):-1,m_EntityAlife.ID(),int(state),!unchanged?34:(!applied?33:0));
    R_ASSERT2(unchanged,"Runtime geometry diagnostic changed rig or physics identity");
}
bool painter_queue_runtime_geometry_probe(u16 object_id,u32 state)
{
    runtime_status(-1,object_id,int(state),25);
    if(!g_pGameLevel)return false;auto* npc=smart_cast<CAI_Stalker*>(Level().Objects.net_Find(object_id));
    return npc && npc->character_physics_support() && npc->character_physics_support()->QueuePainterRuntimeGeometryProbe(state);
}
