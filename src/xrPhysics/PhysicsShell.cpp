#include "stdafx.h"
#pragma hdrstop
#include "physicsshell.h"
#include "PHDynamicData.h"
#include "Physics.h"
#include "PHJoint.h"
#include "PHShell.h"
#include "FittedShell.h"
#include "../xrGame/PainterFittedSkinProxy.h"
#include "../xrGame/painter_voxel/CollisionLimits.h"
#include <cmath>
#include "PHJoint.h"
#include "PHJointDestroyInfo.h"
#include "PHSplitedShell.h"

//#include "gameobject.h"
#include "iphysicsshellholder.h"

//#include "objectdump.h"
#include "phvalide.h"

#include "../Include/xrRender/Kinematics.h"
#include "../xrengine/xr_object.h"
#include "../xrengine/bone.h"

extern CPHWorld* ph_world;

CPhysicsShell::~CPhysicsShell()
{
	//if(ph_world)ph_world->NetRelcase(this);
}

CPhysicsElement* P_create_Element()
{
	CPHElement* element = xr_new<CPHElement>();
	return element;
}

CPhysicsShell* P_create_Shell()
{
	CPhysicsShell* shell = xr_new<CPHShell>();
	return shell;
}

static const char* g_fitted_shell_refusal="";
const char* P_fitted_shell_refusal(){return g_fitted_shell_refusal;}
CPhysicsShell* P_prepare_FittedShell(IPhysicsShellHolder* obj,CPhysicsShell* old,
    const std::vector<PHFittedBone>& fitted,u64 visible_mask,bool bind_target_callbacks,u16 root_bone,const gfy_packed::Mesh* coverage_mesh)
{
    const char*& why=g_fitted_shell_refusal;why="input";
    if(!obj||fitted.empty()||fitted.size()>64||!visible_mask)return nullptr;
    auto* k=obj->ObjectKinematics();why="source_shell";
    if(!k||!old||!old->isFullActive()||k->LL_BoneCount()>64)return nullptr;
    if(root_bone==BI_NONE)root_bone=k->LL_GetBoneRoot();
    why="root_bone";
    if(root_bone>=k->LL_BoneCount()||!(visible_mask&(1ull<<root_bone)))return nullptr;
    u64 seen=0;std::size_t total_boxes=0;
    for(const auto& b:fitted){
        total_boxes+=b.boxes.size();
        if(b.bone>=k->LL_BoneCount()||!(visible_mask&(1ull<<b.bone))||
           (seen&(1ull<<b.bone))||b.boxes.size()>painter_collision_limits::boxes_per_element||
           total_boxes>painter_collision_limits::boxes_per_shell||
           (b.retain_original_shape?!b.boxes.empty():b.boxes.empty())||
           !old->get_Element(b.bone)||
           (!b.retain_original_shape&&
             (!std::isfinite(b.mass_center.x)||!std::isfinite(b.mass_center.y)||
              !std::isfinite(b.mass_center.z)))){
            why=b.bone>=k->LL_BoneCount()?"bone_index":!(visible_mask&(1ull<<b.bone))?"bone_hidden":(seen&(1ull<<b.bone))?"bone_twice":
                b.boxes.size()>painter_collision_limits::boxes_per_element?"boxes_per_element":total_boxes>painter_collision_limits::boxes_per_shell?"boxes_per_shell":
                (b.retain_original_shape?!b.boxes.empty():b.boxes.empty())?"box_list":!old->get_Element(b.bone)?"bone_not_in_source_shell":"mass_center";
            return nullptr;}
        seen|=1ull<<b.bone;
        for(const auto& box:b.boxes)if(!std::isfinite(box.m_halfsize.x)||
            !std::isfinite(box.m_halfsize.y)||!std::isfinite(box.m_halfsize.z)||
            box.m_halfsize.x<.005f||box.m_halfsize.y<.005f||box.m_halfsize.z<.005f||
            box.m_halfsize.x>.75f||box.m_halfsize.y>.75f||box.m_halfsize.z>.75f||
            !std::isfinite(box.m_translate.x)||!std::isfinite(box.m_translate.y)||
            !std::isfinite(box.m_translate.z)||
            !std::isfinite(box.m_rotate.i.x)||!std::isfinite(box.m_rotate.i.y)||
            !std::isfinite(box.m_rotate.i.z)||!std::isfinite(box.m_rotate.j.x)||
            !std::isfinite(box.m_rotate.j.y)||!std::isfinite(box.m_rotate.j.z)||
            !std::isfinite(box.m_rotate.k.x)||!std::isfinite(box.m_rotate.k.y)||
            !std::isfinite(box.m_rotate.k.z)||
            _abs(box.m_rotate.i.magnitude()-1.f)>2e-3f||
            _abs(box.m_rotate.j.magnitude()-1.f)>2e-3f||
            _abs(box.m_rotate.k.magnitude()-1.f)>2e-3f||
            _abs(box.m_rotate.i.dotproduct(box.m_rotate.j))>2e-3f||
            _abs(box.m_rotate.i.dotproduct(box.m_rotate.k))>2e-3f||
            _abs(box.m_rotate.j.dotproduct(box.m_rotate.k))>2e-3f){why="box_shape";return nullptr;}
        // bodyjitter3: only a native element carries cut-backs: at most 4 unit planes, a mass share in (0, 1]
        why="trim";
        if(b.trims>4||(b.trims&&!b.retain_original_shape)||!(b.mass_scale>0.f&&b.mass_scale<=1.f))return nullptr;
        for(u8 q=0;q<b.trims;++q){const float* t=b.trim[q];
            if(!std::isfinite(t[0])||!std::isfinite(t[1])||!std::isfinite(t[2])||!std::isfinite(t[3])||_abs(t[3])>10.f||
               _abs(t[0]*t[0]+t[1]*t[1]+t[2]*t[2]-1.f)>1e-3f)return nullptr;}
        if(b.retain_original_shape)continue;
        why="mass";
        if(!std::isfinite(b.mass)||b.mass<=0.f)return nullptr;
        for(float n:b.inertia)if(!std::isfinite(n))return nullptr;
        why="inertia";
        const float* i=b.inertia;
        if(i[0]<=0.f||i[4]<=0.f||i[8]<=0.f||
           _abs(i[1]-i[3])>1e-5f||_abs(i[2]-i[6])>1e-5f||
           _abs(i[5]-i[7])>1e-5f||
           double(i[0])*i[4]-double(i[1])*i[3]<=1e-12||
           double(i[0])*(double(i[4])*i[8]-double(i[5])*i[7])-
           double(i[1])*(double(i[3])*i[8]-double(i[5])*i[6])+
           double(i[2])*(double(i[3])*i[7]-double(i[4])*i[6])<=1e-18)return nullptr;
    }
    why="root_not_fitted";
    if(!(seen&(1ull<<root_bone)))return nullptr;
    Fmatrix parent_bind=Fidentity;
    auto* source_k=old->PKinematics();why="source_kinematics";
    if(!source_k||source_k->LL_BoneCount()!=k->LL_BoneCount())return nullptr;
    why="root_parent";
    if(root_bone!=source_k->LL_GetBoneRoot()){
        const u16 parent=k->GetBoneData(root_bone).GetParentID();
        if(parent==BI_NONE||parent>=k->LL_BoneCount())return nullptr;
        xr_vector<Fmatrix> original_bind;source_k->LL_GetBindTransform(original_bind);
        if(original_bind.size()!=k->LL_BoneCount())return nullptr;
        parent_bind=original_bind[parent];
    }
    // Preserve only joints whose original physical parent remains in this
    // shell. A missing original element must never be bridged by a new joint.
    for(const auto& b:fitted)if(b.bone!=root_bone){
        u16 ancestor=k->GetBoneData(b.bone).GetParentID();
        bool found=false;
        while(ancestor!=BI_NONE){
            if(old->get_Element(ancestor)){
                found=(seen&(1ull<<ancestor))!=0;break;
            }
            ancestor=k->GetBoneData(ancestor).GetParentID();
        }
        if(!found){why="joint_parent_not_in_part";return nullptr;}
    }
    why="build";
    const u64 previous=k->LL_GetBonesVisible();
    k->LL_SetBonesVisible(visible_mask);
    // bodyjitter3: a native element cut back at a cut weighs what stays of the element it replaces: that element's mass (the
    // corpse's for a first cut) times the share of its cells the body keeps, at least 150 g as a fitted part (producer).
    std::vector<PHFittedBone> trimmed;
    for(const auto& b:fitted)if(b.trims){trimmed=fitted;break;}
    for(auto& b:trimmed)if(b.trims){const dMass* m=old->get_Element(b.bone)->getMassTensor();
        b.mass=m&&m->mass>0.f?_max(.15f,m->mass*b.mass_scale):0.f;}
    CPhysicsShell* result=P_create_Shell();
    auto* shell=static_cast<CPHShell*>(result);
    shell->build_FittedFromKinematics(k,trimmed.empty()?fitted:trimmed,root_bone,parent_bind);
    k->LL_SetBonesVisible(previous);
    if(shell->get_ElementsNumber()!=fitted.size()) {xr_delete(result);return nullptr;}
    for(u16 i=0;i<shell->get_ElementsNumber();++i){
        auto* element=shell->get_ElementByStoreOrder(i);
        if(!element||!element->has_geoms()||!old->get_Element(element->m_SelfID)){
            xr_delete(result);return nullptr;
        }
    }
    // Optional exact per-group source proof; never an anatomy-count inference.
    if(coverage_mesh)painter_skin_proxy::append_fitted(*shell,*k,fitted,*coverage_mesh);
    shell->set_PhysicsRefObject(obj);
    shell->mXFORM=obj->ObjectXFORM();
    shell->Activate(true,true); // Build ODE off-world, disabled, no source callbacks.
    shell->DisableCollision();
    if(!shell->isActive()){why="activate";xr_delete(result);return nullptr;}
    for(u16 i=0;i<shell->get_ElementsNumber();++i){
        auto* target=shell->get_ElementByStoreOrder(i);
        auto* source=old->get_Element(target->m_SelfID);
        // Geometry replacement must retain the source element's damping.
        // Generic fragment objects do not run the NPC shell setup afterward.
        float resistance_linear=0.f,resistance_angular=0.f;
        source->GetAirResistance(resistance_linear,resistance_angular);
        // piecephys (2026-11-13): the resistance is a force / torque per speed (PHElement.cpp PhDataUpdate: -v*|v|*k_l, -w*k_w) sized
        // for the source element (a corpse: P_build_Shell's defaults 0.0002 and 0.05). The torque is applied once per step with no
        // limit: on a fitted fragment whose inertia is below k_w*step/2 (2.5e-4 kg m2: every head burst chunk) it reversed and
        // multiplied the spin every step, up to the angular limit - the chunk hopped and drifted for ever and never slept. A fitted
        // element keeps the source's resistance per kilogram and per inertia, and its angular resistance stays below half of its
        // smallest principal inertia per step (4 det / trace^2 is a lower bound of that inertia), so the torque cannot reverse a spin.
        if(const dMass* source_mass=source->getMassTensor())if(const dMass* target_mass=target->getMassTensor()){
            if(source_mass->mass>0.f&&target_mass->mass>0.f&&target_mass->mass<source_mass->mass)
                resistance_linear*=target_mass->mass/source_mass->mass;
            const dReal* I=target_mass->I;
            const float source_trace=source_mass->I[0]+source_mass->I[5]+source_mass->I[10],target_trace=I[0]+I[5]+I[10];
            if(source_trace>0.f&&target_trace>0.f){
                if(target_trace<source_trace)resistance_angular*=target_trace/source_trace;
                const float det=I[0]*(I[5]*I[10]-I[6]*I[9])-I[1]*(I[4]*I[10]-I[6]*I[8])+I[2]*(I[4]*I[9]-I[5]*I[8]);
                const float smallest=det>0.f?4.f*det/(target_trace*target_trace):0.f;
                resistance_angular=_min(resistance_angular,.5f*smallest/fixed_step);
            }
        }
        target->SetAirResistance(resistance_linear,resistance_angular);
        Fmatrix transform;Fvector linear,angular;
        source->GetGlobalTransformDynamic(&transform);
        source->get_AngularVel(angular);
        target->SetTransform(transform,mh_unspecified);
        Fvector new_com;target->GetGlobalPositionDynamic(&new_com);
        source->GetPointVel(linear,new_com);
        target->set_LinearVel(linear);target->set_AngularVel(angular);
    }
    // A detached shell may root at pelvis/forearm while the render object
    // keeps the original full-model frame. Preserve that object frame exactly.
    Fmatrix root_world,root_inverse;
    shell->get_ElementByStoreOrder(0)->GetGlobalTransformDynamic(&root_world);
    root_inverse.invert(root_world);
    shell->ObjectInRoot().mul_43(root_inverse,obj->ObjectXFORM());
    Fmatrix refreshed;shell->GetGlobalTransformDynamic(&refreshed);
    if(!shell->RefreshFittedPose()){
        why="pose_refresh";destroy_physics_shell(result);
        if(k==source_k){old->EnabledCallbacks(TRUE);k->CalculateBones_Invalidate();}
        return nullptr;
    }
    if(bind_target_callbacks){shell->EnabledCallbacks(TRUE);
        k->CalculateBones_Invalidate();k->CalculateBones(TRUE);}
    why="";
    return result;
}

bool P_refresh_FittedPose(CPhysicsShell* target)
{
    auto* shell=dynamic_cast<CPHShell*>(target);
    return shell&&shell->RefreshFittedPose();
}

bool P_bind_FittedPoseLinks(CPhysicsShell* target,
    const std::vector<PHFittedPoseLink>& links,bool apply_now)
{
    auto* shell=dynamic_cast<CPHShell*>(target);
    return shell&&shell->BindFittedPoseLinks(links,apply_now);
}

CPhysicsShell* P_create_splited_Shell()
{
	CPhysicsShell* shell = xr_new<CPHSplitedShell>();
	return shell;
}

CPhysicsJoint* P_create_Joint(CPhysicsJoint::enumType type, CPhysicsElement* first, CPhysicsElement* second)
{
	CPhysicsJoint* joint = xr_new<CPHJoint>(type, first, second);
	return joint;
}


CPhysicsShell* __stdcall P_build_Shell(IPhysicsShellHolder* obj, bool not_active_state, BONE_P_MAP* bone_map,
                                       bool not_set_bone_callbacks)
{
	VERIFY(obj);
	phys_shell_verify_object_model(*obj);
	//IRenderVisual*	V = obj->ObjectVisual();
	//IKinematics* pKinematics=smart_cast<IKinematics*>(V);
	//IKinematics* pKinematics	=  V->dcast_PKinematics			();
	IKinematics* pKinematics = obj->ObjectKinematics();

	CPhysicsShell* pPhysicsShell = P_create_Shell();
#ifdef DEBUG
	pPhysicsShell->dbg_obj=obj;
#endif
	pPhysicsShell->build_FromKinematics(pKinematics, bone_map);

	pPhysicsShell->set_PhysicsRefObject(obj);
	pPhysicsShell->mXFORM.set(obj->ObjectXFORM());
	pPhysicsShell->Activate(not_active_state, not_set_bone_callbacks); //,
	//m_pPhysicsShell->SmoothElementsInertia(0.3f);
	pPhysicsShell->SetAirResistance(); //0.0014f,1.5f

	return pPhysicsShell;
}

void fix_bones(LPCSTR fixed_bones, CPhysicsShell* shell)
{
	VERIFY(fixed_bones);
	VERIFY(shell);
	IKinematics* pKinematics = shell->PKinematics();
	VERIFY(pKinematics);
	int count = _GetItemCount(fixed_bones);
	for (int i = 0; i < count; ++i)
	{
		string64 fixed_bone;
		_GetItem(fixed_bones, i, fixed_bone);
		u16 fixed_bone_id = pKinematics->LL_BoneID(fixed_bone);
		R_ASSERT2(BI_NONE != fixed_bone_id, make_string("wrong fixed bone [%s] for object with visual [%s]", fixed_bone, pKinematics->getDebugName().c_str()).c_str());
		CPhysicsElement* E = shell->get_Element(fixed_bone_id);
		if (E)
			E->Fix();
	}
}

CPhysicsShell* P_build_Shell(IPhysicsShellHolder* obj, bool not_active_state, BONE_P_MAP* p_bone_map,
                             LPCSTR fixed_bones)
{
	CPhysicsShell* pPhysicsShell = 0;
	//IKinematics* pKinematics=smart_cast<IKinematics*>(obj->ObjectVisual());
	IKinematics* pKinematics = obj->ObjectKinematics();
	if (fixed_bones)
	{
		int count = _GetItemCount(fixed_bones);
		for (int i = 0; i < count; ++i)
		{
			string64 fixed_bone;
			_GetItem(fixed_bones, i, fixed_bone);
			u16 fixed_bone_id = pKinematics->LL_BoneID(fixed_bone);
			R_ASSERT2(BI_NONE != fixed_bone_id, make_string("wrong fixed bone [%s] for object with visual [%s]", fixed_bone, pKinematics->getDebugName().c_str()).c_str());
			p_bone_map->insert(mk_pair(fixed_bone_id, physicsBone()));
		}

		pPhysicsShell = P_build_Shell(obj, not_active_state, p_bone_map);

		//m_pPhysicsShell->add_Joint(P_create_Joint(CPhysicsJoint::enumType::full_control,0,fixed_element));
	}
	else
		pPhysicsShell = P_build_Shell(obj, not_active_state);


	BONE_P_PAIR_IT i = p_bone_map->begin(), e = p_bone_map->end();
	if (i != e) pPhysicsShell->SetPrefereExactIntegration();
	for (; i != e; i++)
	{
		CPhysicsElement* fixed_element = i->second.element;
		R_ASSERT2(fixed_element, "fixed bone has no physics");
		//if(!fixed_element) continue;
		fixed_element->Fix();
	}
	return pPhysicsShell;
}

CPhysicsShell* P_build_Shell(IPhysicsShellHolder* obj, bool not_active_state, LPCSTR fixed_bones)
{
	U16Vec f_bones;
	if (fixed_bones)
	{
		//IKinematics* K		= smart_cast<IKinematics*>(obj->ObjectVisual());
		IKinematics* K = obj->ObjectKinematics();
		VERIFY(K);
		int count = _GetItemCount(fixed_bones);
		for (int i = 0; i < count; ++i)
		{
			string64 fixed_bone;
			_GetItem(fixed_bones, i, fixed_bone);
			f_bones.push_back(K->LL_BoneID(fixed_bone));
			R_ASSERT2(BI_NONE!=f_bones.back(), "wrong fixed bone") ;
		}
	}
	return P_build_Shell(obj, not_active_state, f_bones);
}

static BONE_P_MAP bone_map = BONE_P_MAP();

CPhysicsShell* P_build_Shell(IPhysicsShellHolder* obj, bool not_active_state, U16Vec& fixed_bones)
{
	bone_map.clear();
	CPhysicsShell* pPhysicsShell = 0;
	if (!fixed_bones.empty())
		for (U16It it = fixed_bones.begin(); it != fixed_bones.end(); it++)
			bone_map.insert(mk_pair(*it, physicsBone()));
	pPhysicsShell = P_build_Shell(obj, not_active_state, &bone_map);

	// fix bones
	BONE_P_PAIR_IT i = bone_map.begin(), e = bone_map.end();
	if (i != e)
		pPhysicsShell->SetPrefereExactIntegration();
	for (; i != e; i++)
	{
		CPhysicsElement* fixed_element = i->second.element;
		//R_ASSERT2(fixed_element,"fixed bone has no physics");
		if (!fixed_element) continue;
		fixed_element->Fix();
	}
	return pPhysicsShell;
}

CPhysicsShell* P_build_SimpleShell(IPhysicsShellHolder* obj, float mass, bool not_active_state)
{
	CPhysicsShell* pPhysicsShell = P_create_Shell();
#ifdef DEBUG
	pPhysicsShell->dbg_obj=(obj);
#endif
	//Fobb obb; obj->ObjectVisual()->getVisData().box.get_CD( obb.m_translate, obb.m_halfsize );
	VERIFY(obj);
	VERIFY(obj->ObjectKinematics());

	Fobb obb;
	obj->ObjectKinematics()->GetBox().get_CD(obb.m_translate, obb.m_halfsize);
	obb.m_rotate.identity();
	CPhysicsElement* E = P_create_Element();
	R_ASSERT(E);
	E->add_Box(obb);
	pPhysicsShell->add_Element(E);
	pPhysicsShell->setMass(mass);
	pPhysicsShell->set_PhysicsRefObject(obj);
	if (!obj->has_parent_object())
		pPhysicsShell->Activate(obj->ObjectXFORM(), 0, obj->ObjectXFORM(), not_active_state);
	return pPhysicsShell;
}

void ApplySpawnIniToPhysicShell(CInifile const* ini, CPhysicsShell* physics_shell, bool fixed)
{
	if (!ini)
		return;
	if (ini->section_exist("physics_common"))
	{
		fixed = fixed || (ini->line_exist("physics_common", "fixed_bones"));
#pragma todo("not ignore static if non realy fixed! ")
		fix_bones(ini->r_string("physics_common", "fixed_bones"), physics_shell);
	}
	if (ini->section_exist("collide"))
	{
		if ((ini->line_exist("collide", "ignore_static") && fixed) || (ini->line_exist("collide", "ignore_static") &&
			ini->section_exist("animated_object")))
		{
			physics_shell->SetIgnoreStatic();
		}
		if (ini->line_exist("collide", "small_object"))
		{
			physics_shell->SetSmall();
		}
		if (ini->line_exist("collide", "ignore_small_objects"))
		{
			physics_shell->SetIgnoreSmall();
		}
		if (ini->line_exist("collide", "ignore_ragdoll"))
		{
			physics_shell->SetIgnoreRagDoll();
		}


		//If need, then show here that it is needed to ignore collisions with "animated_object"
		if (ini->line_exist("collide", "ignore_animated_objects"))
		{
			physics_shell->SetIgnoreAnimated();
		}
	}
	//If next section is available then given "PhysicShell" is classified
	//as animated and we read options for his animation

	if (ini->section_exist("animated_object"))
	{
		//Show that given "PhysicShell" animated
		physics_shell->CreateShellAnimator(ini, "animated_object");
	}
}


void get_box(const CPhysicsBase* shell, const Fmatrix& form, Fvector& sz, Fvector& c)
{
	t_get_box(shell, form, sz, c);
}


void __stdcall destroy_physics_shell(CPhysicsShell* & p)
{
	if (p)
		p->Deactivate();
	xr_delete(p);
}

bool bone_has_pysics(IKinematics& K, u16 bone_id)
{
	//CBoneData	* pBonedata1 = &K.LL_GetData( bone_id );
	//CBoneData	* pBonedata2 = K.LL_GetBoneData( bone_id );

	//u32	sz = sizeof(vecBones);
	//u32	sz1=  sizeof(pBonedata1->children);

	//	VERIFY(pBonedata1 == pBonedata2);
	return K.LL_GetBoneVisible(bone_id) && shape_is_physic(K.GetBoneData(bone_id).get_shape());
}

bool has_physics_collision_shapes(IKinematics& K)
{
	u16 nbb = K.LL_BoneCount();
	for (u16 i = 0; i < nbb; ++i)
		if (bone_has_pysics(K, i))
			return true;
	return false;
}

void phys_shell_verify_model(IKinematics& K)
{
	//IRenderVisual* V = K.dcast_RenderVisual();
	//VERIFY( V );
#ifdef DEBUG
	VERIFY2(has_physics_collision_shapes( K ),
	        make_string( "Can not create physics shell for model %s because it has no physics collision shapes set", K.
		        getDebugName().c_str() ));
#endif
}

void phys_shell_verify_object_model(IPhysicsShellHolder& O)
{
	//IRenderVisual	*V = O.ObjectVisual();

	//VERIFY2( V, make_string( "Can not create physics shell for object %s it has no model", O.ObjectName() )/*+ make_string("\n object dump: \n") + dbg_object_full_dump_string( &O )*/ );

	//IKinematics		*K = V->dcast_PKinematics();

	IKinematics* K = O.ObjectKinematics();

	VERIFY2(
		K, make_string( "Can not create physics shell for object %s, model %s is not skeleton", O.ObjectName(), O.
			ObjectNameVisual() ));

	VERIFY2(has_physics_collision_shapes( *K ),
	        make_string( "Can not create physics shell for object %s, model %s has no physics collision shapes set", O.
		        ObjectName(), O.ObjectNameVisual() )
	        /*+ make_string("\n object dump: \n") + dbg_object_full_dump_string( &O )*/);

	VERIFY2(_valid( O.ObjectXFORM() ),
	        make_string( "create physics shell: object matrix is not valid" )
	        /*+ make_string("\n object dump: \n") + dbg_object_full_dump_string( &O )*/);
#ifdef DEBUG
	VERIFY2(valid_pos( O.ObjectXFORM().c ), dbg_valide_pos_string( O.ObjectXFORM().c, &O, "create physics shell" ));
#endif
}

bool __stdcall can_create_phys_shell(string1024& reason, IPhysicsShellHolder& O)
{
	xr_strcpy(reason, "ok");
	bool result = true;
	IKinematics* K = O.ObjectKinematics();
	if (!K)
	{
		xr_strcpy(reason, make_string("Can not create physics shell for object %s, model %s is not skeleton",
		                              O.ObjectName(), O.ObjectNameVisual()).c_str());
		return false;
	}
	if (!has_physics_collision_shapes(*K))
	{
		xr_strcpy(reason, make_string(
			          "Can not create physics shell for object %s, model %s has no physics collision shapes set",
			          O.ObjectName(), O.ObjectNameVisual()).c_str());
		return false;
	}
	if (!_valid(O.ObjectXFORM()))
	{
		xr_strcpy(reason, make_string("create physics shell: object matrix is not valid").c_str());
		return false;
	}
	if (!valid_pos(O.ObjectXFORM().c))
	{
#ifdef	DEBUG
		xr_strcpy( reason, dbg_valide_pos_string( O.ObjectXFORM().c, &O, "create physics shell" ).c_str() );
#else
		xr_strcpy(reason, make_string("create physics shell: object position is not valid").c_str());
#endif
		return false;
	}
	return result;
}


float NonElasticCollisionEnergy(CPhysicsElement* e1, CPhysicsElement* e2, const Fvector& norm) // norm - from 2 to 1
{
	VERIFY(e1);
	VERIFY(e2);
	dBodyID b1 = static_cast<CPHElement*>(e1)->get_body();
	VERIFY(b1);
	dBodyID b2 = static_cast<CPHElement*>(e2)->get_body();
	VERIFY(b2);
	return E_NL(b1, b2, cast_fp(norm));
}


void StaticEnvironmentCB(bool& do_colide, bool bo1, dContact& c, SGameMtl* material_1, SGameMtl* material_2)
{
	dJointID contact_joint = dJointCreateContact(0, ContactGroup, &c);

	if (bo1)
	{
		((CPHIsland*)(retrieveGeomUserData(c.geom.g1)->callback_data))->DActiveIsland()->ConnectJoint(contact_joint);
		dJointAttach(contact_joint, dGeomGetBody(c.geom.g1), 0);
	}
	else
	{
		((CPHIsland*)(retrieveGeomUserData(c.geom.g2)->callback_data))->DActiveIsland()->ConnectJoint(contact_joint);
		dJointAttach(contact_joint, 0, dGeomGetBody(c.geom.g2));
	}
	do_colide = false;
}
