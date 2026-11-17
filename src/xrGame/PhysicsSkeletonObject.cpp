#include "stdafx.h"
#include "physicsskeletonobject.h"
//#include "PhysicsShell.h"
#include "../xrphysics/physicsshell.h"
#include "phsynchronize.h"
#include "xrserver_objects_alife.h"
#include "../Include/xrRender/Kinematics.h"
#include "../xrEngine/xr_collide_form.h"
#include "painter_arm_pose_impl.h"
#include "PainterDamagedFragment_impl.h"
#include "PainterPieceContinuation_impl.h"
#include "Entity.h"
#include "../Include/xrRender/BloodThermal.h"
#include "../Layers/xrRender/xrRender_console.h"

float CPhysicsSkeletonObject::GetHotness()
{
    if (!m_painter_thermal_corpse) return inherited::GetHotness();
    float h = BloodThermal::Corpse(Device.dwTimeGlobal, m_painter_thermal_death_time, heat_vision_cooldown_time, heat_vision_cooldown != 0);
    // thermal3: the draw takes this hotness under the corpse flag: pack_gbuffer writes 0.75 x hotness on the skin, the exposed
    // anatomy writes the hotness (saturated). A piece cut off a LIVING body starts at that body's heat (skin 1 as alive, interior
    // hotter where the target allows) and fades from the cut: / 0.75. A gut rope is interior tissue on a plain material: / 0.75 again.
    if (m_painter_thermal_living) h /= 0.75f;
    if (m_painter_thermal_rope) h /= 0.75f;
    return h;
}

void CPhysicsSkeletonObject::PainterInheritThermal(CPhysicsShellHolder& source)
{
    // Called only when a committed detached-tissue child becomes visible.
    // A still-living source can precede its sever kill callback: its child is
    // already detached tissue and starts cooling at this commit, not time zero.
    // An existing corpse/thermal piece retains the original death clock.
    m_painter_thermal_corpse=true;
    m_painter_thermal_death_time=Device.dwTimeGlobal;
    m_painter_thermal_living=true; // thermal3: a living (or unknown) source: the piece starts at a living body's heat
    if (auto* entity=smart_cast<CEntity*>(&source)) {
        if (entity->IsThermalCorpse())
            { m_painter_thermal_death_time=entity->GetLevelDeathTime(); m_painter_thermal_living=false; }
    } else if (auto* piece=smart_cast<CPhysicsSkeletonObject*>(&source)) {
        if (piece->m_painter_thermal_corpse)
            { m_painter_thermal_death_time=piece->m_painter_thermal_death_time; m_painter_thermal_living=piece->m_painter_thermal_living; }
    }
}

void CPhysicsSkeletonObject::PainterRopeThermal(CPhysicsShellHolder& host)
{
    // thermal3: called once, when a gut rope is attached to its host (before it becomes visible).
    PainterInheritThermal(host);
    m_painter_thermal_rope=true;
}

CPhysicsSkeletonObject::CPhysicsSkeletonObject()
{
}

CPhysicsSkeletonObject::~CPhysicsSkeletonObject()
{
    painter_piece_continuation::forget(*this);
    painter_arm_pose_release(*this,m_painter_arm_pose);
    painter_damaged_fragment::gone(*this,m_damaged_fragment);
    painter_gut_spill::gone(*this);
}


BOOL CPhysicsSkeletonObject::net_Spawn(CSE_Abstract* DC)
{
	m_painter_thermal_corpse=false; m_painter_thermal_death_time=0; m_painter_thermal_living=false; m_painter_thermal_rope=false;
	CSE_Abstract* e = (CSE_Abstract*)(DC);

	BOOL res = inherited::net_Spawn(DC);
	xr_delete(collidable.model);
	collidable.model = xr_new<CCF_Skeleton>(this);
    const int pose_request=painter_arm_pose_begin(*this,e,m_painter_arm_pose);
    if(pose_request<0) { Msg("[butcher_arm_pose] piece=%u phase=rejected_before_shell",ID());return FALSE; }
	const int damaged_request=painter_damaged_fragment::begin_spawn(*this,e,m_damaged_fragment);
	if(damaged_request<0)return FALSE;
	const int gut_request=damaged_request?0:painter_gut_spill::begin_spawn(*this,e);
	if(gut_request<0)return FALSE;
	CPHSkeleton::Spawn(e);
    if(pose_request && PainterArmPoseStatus()!=1) {
        Msg("[butcher_arm_pose] piece=%u phase=rejected_after_shell",ID());
        painter_arm_pose_release(*this,m_painter_arm_pose);return FALSE;
    }
	if(damaged_request){painter_damaged_fragment::end_spawn(*this,m_damaged_fragment,PPhysicsShell()!=nullptr);
	    setVisible(FALSE);setEnabled(FALSE);return PPhysicsShell()!=nullptr && res;}
	if(gut_request)return painter_gut_spill::end_spawn(*this) && res;
	setVisible(TRUE);
	setEnabled(TRUE);
	if (!PPhysicsShell() || !PPhysicsShell()->isBreakable())
		SheduleUnregister();
	return res;
}

void CPhysicsSkeletonObject::SpawnInitPhysics(CSE_Abstract* D)
{
	CreatePhysicsShell(D);
	IKinematics* K = smart_cast<IKinematics*>(Visual());
	if (K)
	{
		K->CalculateBones_Invalidate();
		K->CalculateBones(TRUE);
	}
}

void CPhysicsSkeletonObject::net_Destroy()
{
    painter_piece_continuation::forget(*this);
    painter_arm_pose_release(*this,m_painter_arm_pose);
    painter_damaged_fragment::gone(*this,m_damaged_fragment);
    painter_gut_spill::gone(*this);
	inherited::net_Destroy();
	CPHSkeleton::RespawnInit();
}

void CPhysicsSkeletonObject::Load(LPCSTR section)
{
	inherited::Load(section);
	CPHSkeleton::Load(section);
}

void CPhysicsSkeletonObject::CreatePhysicsShell(CSE_Abstract* e)
{
	CSE_PHSkeleton* po = smart_cast<CSE_PHSkeleton*>(e);
	if (m_pPhysicsShell) return;
	if (!Visual()) return;
	if(m_damaged_fragment){painter_damaged_fragment::build_shell(*this,m_damaged_fragment);return;}
	m_pPhysicsShell = P_build_Shell(this, !po->_flags.test(CSE_PHSkeleton::flActive));
    painter_arm_pose_apply(*this,m_painter_arm_pose);
}


void CPhysicsSkeletonObject::shedule_Update(u32 dt)
{
	inherited::shedule_Update(dt);

	CPHSkeleton::Update(dt);
}

void CPhysicsSkeletonObject::net_Save(NET_Packet& P)
{
	inherited::net_Save(P);
	CPHSkeleton::SaveNetState(P);
}


BOOL CPhysicsSkeletonObject::net_SaveRelevant()
{
	return TRUE; //!m_flags.test(CSE_ALifeObjectPhysic::flSpawnCopy);
}


BOOL CPhysicsSkeletonObject::UsedAI_Locations()
{
	return (FALSE);
}

void CPhysicsSkeletonObject::UpdateCL()
{
	inherited::UpdateCL();
	PHObjectPositionUpdate();
    painter_piece_continuation::queue(*this);
}

void CPhysicsSkeletonObject::Hit(SHit* hit)
{
    if(hit)painter_piece_continuation::capture(*this,*hit);
    inherited::Hit(hit);
}

void CPhysicsSkeletonObject::ApplyPainterPieceContinuation()
{
    painter_piece_continuation::poll(*this);
}

int CPhysicsSkeletonObject::PainterPieceFleshOnRay(const Fvector& start, const Fvector& dir, float range, u16 bone)
{
    if(!m_damaged_fragment||!m_damaged_fragment->stage||!m_damaged_fragment->stage->request)return 0;
    const auto& stage=*m_damaged_fragment->stage;const auto& req=*stage.request;
    if(stage.index>=req.pieces.size())return 0;
    const auto& fitted=req.pieces[stage.index].fitted;
    IKinematics* K=smart_cast<IKinematics*>(Visual());
    if(!K||fitted.empty()||bone>=K->LL_BoneCount())return 0;
    const Fmatrix& L2W=XFORM();
    constexpr float margin=.03f; // bodypass: 3 cm - the collider fitter omits no cell farther than 2.5 cm from its boxes
    // a bone shape's bounding sphere in the world
    auto sphereOf=[&](u16 b,Fvector& centre,float& radius){
        const SBoneShape& shape=K->LL_GetData(b).shape;Fvector local;
        switch(shape.type){
        case SBoneShape::stBox: local.set(shape.box.m_translate);radius=shape.box.m_halfsize.magnitude();break;
        case SBoneShape::stSphere: local.set(shape.sphere.P);radius=shape.sphere.R;break;
        case SBoneShape::stCylinder: local.set(shape.cylinder.m_center);
            radius=_sqrt(shape.cylinder.m_height*shape.cylinder.m_height*.25f+shape.cylinder.m_radius*shape.cylinder.m_radius);break;
        default: return false;}
        K->LL_GetTransform(b).transform_tiny(centre,local);L2W.transform_tiny(centre);return true;};
    // the ray's stretch [t0,t1] inside a sphere (false: it misses)
    auto raySphere=[&](const Fvector& centre,float radius,float& t0,float& t1){
        Fvector oc;oc.sub(centre,start);const float along=oc.dotproduct(dir),off2=oc.square_magnitude()-along*along;
        if(off2>radius*radius)return false;const float half=_sqrt(radius*radius-off2);t0=along-half;t1=along+half;return true;};
    Fvector struckCentre;float struckRadius,s0,s1;
    if(!sphereOf(bone,struckCentre,struckRadius))return 0;
    const float lo=range-margin,hi=(raySphere(struckCentre,struckRadius,s0,s1)?_max(range,s1):range)+margin;
    // gearbody: the piece's merged gear (flesh_gear: boxes in bone frame) counts as flesh after its fitted elements
    for(const auto* list:{&fitted,&req.pieces[stage.index].flesh_gear})for(const auto& element:*list){
        if(element.bone>=K->LL_BoneCount())continue;
        if(element.retain_original_shape){
            Fvector c;float r,t0,t1;
            if(sphereOf(element.bone,c,r)&&raySphere(c,r+margin,t0,t1)&&t1>=lo&&t0<=hi)return 1;
            continue;}
        Fmatrix boneW;boneW.mul_43(L2W,K->LL_GetTransform(element.bone));
        for(const auto& box:element.boxes){
            Fmatrix ME,TW,IM;box.xform_get(ME);TW.mul_43(boneW,ME);if(!IM.invert_b(TW))continue;
            Fvector p,d;IM.transform_tiny(p,start);IM.transform_dir(d,dir);
            float t0=-flt_max,t1=flt_max;bool meets=true;
            for(int a=0;a<3&&meets;++a){
                const float h=(&box.m_halfsize.x)[a]+margin,pa=(&p.x)[a],da=(&d.x)[a];
                if(_abs(da)<1e-9f){if(pa<-h||pa>h)meets=false;continue;}
                float ta=(-h-pa)/da,tb=(h-pa)/da;if(ta>tb){const float s=ta;ta=tb;tb=s;}
                t0=_max(t0,ta);t1=_min(t1,tb);if(t0>t1)meets=false;}
            if(meets&&t1>=lo&&t0<=hi)return 1;}
    }
    return 2;
}

void CPhysicsSkeletonObject::PHObjectPositionUpdate()
{
	if (m_pPhysicsShell)
	{
		m_pPhysicsShell->InterpolateGlobalTransform(&XFORM());
	}
}

void CPhysicsSkeletonObject::OnChangeVisual()
{
    // CObject invokes this before deleting the previous visual.
    painter_arm_pose_release(*this,m_painter_arm_pose);
    inherited::OnChangeVisual();
}
