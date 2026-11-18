// body_hit_outcomes_v1
#include "stdafx.h"

#include "CharacterPhysicsSupport.h"
#include "weapon_grip_kinematics.h"
#include "painter_arm_pose.h"
#include "painter_voxel/PainterPieceContinuation.h"
#include "painter_voxel/BodyHitOutcomes.h"
#include "alife_space.h"
#include "hit.h"
#include "PHDestroyable.h"
#include "PHMovementControl.h"
#include "CustomMonster.h"
#include "PhysicsSkeletonObject.h"


#include "../Include/xrRender/KinematicsAnimated.h"


#include "../xrphysics/PhysicsShell.h"
#include "../xrphysics/iActivationShape.h"
//#include "../xrphysics/Extendedgeom.h"
#include "../xrphysics/geometry.h"
#include "../xrphysics/dcylinder/dCylinder.h"
//#include "../xrphysics/phdynamicdata.h"
#include "../xrphysics/IPHCapture.h"
//#include "../xrphysics/ICollideValidator.h"
#include "../xrphysics/IPHWorld.h"
#include "../xrphysics/PHWorld.h"
#include "../include/xrapi/xrapi.h"
class CGameMtlLibrary;
IC CGameMtlLibrary& GMLibrary() { VERIFY(PGMLib); return *PGMLib; }
#include "../xrphysics/PHElement.h"
#include "../xrphysics/phvalide.h"
#include "../xrphysics/PHShell.h"
#include "level.h"
#include "../xrphysics/PHJoint.h"
#include "PainterSkinProxy.h"
#include "../xrphysics/PHIsland.h"
#include "../xrphysics/PHUpdateObject.h"
#include "../xrphysics/console_vars.h"
#include "../xrCore/LocatorAPI.h"
#include <cstdio>
#include <cmath>

//#include "Physics.h"


#include "IKLimbsController.h"
#include "Actor.h"
#include "ai/stalker/ai_stalker.h"
#include "imotion_position.h"
#include "imotion_velocity.h"
#include "animation_movement_controller.h"
#include "xrServer_Object_Base.h"
#include "interactive_animation.h"
#include "stalker_animation_manager.h"
#include "inventoryowner.h"
#include "inventory.h"
#include "activatingcharcollisiondelay.h"
#include "stalker_movement_manager_smart_cover.h"

#include "../build_config_defines.h"

//const float default_hinge_friction = 5.f;//gray_wolf comment
#ifdef DEBUG
#	include "PHDebug.h"
extern	BOOL death_anim_debug;
#endif // DEBUG

#include "../xrEngine/device.h"
#include "../Include/xrRender/PainterLogVerbose.h"

#define USE_SMART_HITS
#define USE_IK

float IK_CALC_DIST = 100.f;
float IK_ALWAYS_CALC_DIST = 20.f;
float IK_CALC_SSA = 0.006f;

extern int painter_ragdoll_capture_next;
extern int painter_native_ragdoll; // console_commands.cpp (R35): 0 = Lua owns the joint motors, 1 = native controller
extern int painter_sever_hold_ms; // PainterVoxelLive_impl.h (R63): longest frozen death-shell wait for a Painter sever
namespace {
CCharacterPhysicsSupport* painter_alive_reference_target = nullptr;
u16 painter_alive_reference_target_id = u16(-1);
u32 painter_alive_reference_armed_at = 0;
}

namespace {
// Passive diagnostic. This object is enrolled after the shell has been built
// and samples after physics steps; it never applies forces or changes settings.
class CRagdollShellCapture : public CPHUpdateObject
{
    static CRagdollShellCapture* active_capture_;
    CPhysicsShell* shell_ = nullptr;
    IKinematics* reference_kinematics_ = nullptr;
    FILE* file_ = nullptr;
    unsigned step_ = 0;
    xr_vector<float> previous_angles_;
    xr_vector<u8> previous_angle_valid_;
    xr_vector<CPhysicsElement*> original_bodies_;
    xr_vector<CPhysicsJoint*> original_joints_;
    bool invalid_numeric_ = false;
    bool manual_request_ = false;
    u32 manual_started_tc_ = 0;
    static const unsigned trace_steps_ = 16;
    static const unsigned manual_contact_limit_ = 4096;
    static const long manual_file_limit_ = 16L * 1024L * 1024L;

    void number(FILE* f, double v)
    {
        if (std::isfinite(v)) fprintf(f, "%.9g", v);
        else { fputs("null", f); invalid_numeric_ = true; }
    }
    void vec(FILE* f, const dReal* v)
    {
        fputc('[',f); number(f,v[0]); fputc(',',f); number(f,v[1]); fputc(',',f); number(f,v[2]); fputc(']',f);
    }
    void vec(FILE* f, const Fvector& v)
    {
        fputc('[',f); number(f,v.x); fputc(',',f); number(f,v.y); fputc(',',f); number(f,v.z); fputc(']',f);
    }
    void frame(FILE* f, const Fmatrix& m)
    {
        fputc('[',f); vec(f,m.i); fputc(',',f); vec(f,m.j); fputc(',',f); vec(f,m.k); fputc(',',f); vec(f,m.c); fputc(']',f);
    }
    void ode_rotation(FILE* f, const dReal* m)
    {
        fputc('[',f);
        for (int row=0;row<3;++row) {
            if (row) fputc(',',f); fputc('[',f);
            for (int col=0;col<3;++col) { if (col) fputc(',',f); number(f,m[row*4+col]); }
            fputc(']',f);
        }
        fputc(']',f);
    }
    static void name(FILE* f, LPCSTR s)
    {
        fputc('"', f);
        if (s) for (const unsigned char* p = reinterpret_cast<const unsigned char*>(s); *p; ++p)
        {
            if (*p == '"' || *p == '\\') fputc('\\', f);
            if (*p >= 32 && *p < 127) fputc(*p, f);
            else fprintf(f, "\\u%04x", unsigned(*p));
        }
        fputc('"', f);
    }
    static const char* joint_type(CPhysicsJoint::enumType t)
    {
        switch (t) {
        case CPhysicsJoint::ball: return "ball";
        case CPhysicsJoint::hinge: return "hinge";
        case CPhysicsJoint::hinge2: return "hinge2";
        case CPhysicsJoint::full_control: return "full_control";
        case CPhysicsJoint::slider: return "slider";
        default: return "unknown";
        }
    }
    static double joint_param(CPHJoint* j, int axis, int param)
    {
        const int p = param + axis * dParamGroup;
        switch (j->eType) {
        case CPhysicsJoint::full_control: return j->GetDJoint1() ? dJointGetAMotorParam(j->GetDJoint1(), p) : 0.0;
        case CPhysicsJoint::hinge: return j->GetDJoint() ? dJointGetHingeParam(j->GetDJoint(), p) : 0.0;
        case CPhysicsJoint::hinge2: return j->GetDJoint() ? dJointGetHinge2Param(j->GetDJoint(), p) : 0.0;
        case CPhysicsJoint::slider: return j->GetDJoint() ? dJointGetSliderParam(j->GetDJoint(), p) : 0.0;
        default: return 0.0;
        }
    }
    static bool valid_body_pose(CPHElement* element)
    {
        const dBodyID body = element->get_body();
        if (!body) return false;
        const dReal* values[] = {dBodyGetPosition(body),dBodyGetQuaternion(body),dBodyGetLinearVel(body),dBodyGetAngularVel(body)};
        const int sizes[] = {3,4,3,3};
        for (int group=0;group<4;++group)
            for (int i=0;i<sizes[group];++i)
                if (!std::isfinite(double(values[group][i]))) return false;
        return true;
    }
    void body_state(CPHElement* e, bool static_data, bool pre_solver = false)
    {
        dBodyID body = e->get_body();
        Fmatrix origin; e->GetGlobalTransformDynamic(&origin);
        const dReal* p = dBodyGetPosition(body);
        const dReal* q = dBodyGetQuaternion(body);
        const dReal* v = dBodyGetLinearVel(body);
        const dReal* w = dBodyGetAngularVel(body);
        fprintf(file_, "{\"bone_id\":%u,\"ode_com_world\":", unsigned(e->m_SelfID)); vec(file_, p);
        fprintf(file_, ",\"ode_quaternion_wxyz\":[");
        for (int i=0;i<4;++i) { if (i) fputc(',',file_); number(file_,q[i]); }
        fprintf(file_, "],\"linear_velocity\":"); vec(file_, v);
        fprintf(file_, ",\"angular_velocity\":"); vec(file_, w);
        fprintf(file_, ",\"ode_body_enabled\":%s,\"element_enabled\":%s",
            dBodyIsEnabled(body) ? "true" : "false", e->isEnabled() ? "true" : "false");
        const dxBody* ode_body = static_cast<const dxBody*>(body);
        fprintf(file_, ",\"ode_body_flags\":%d,\"finite_rot_axis\":",ode_body->flags);
        vec(file_,ode_body->finite_rot_axis);
        if (pre_solver) {
            fprintf(file_, ",\"force\":"); vec(file_,dBodyGetForce(body));
            fprintf(file_, ",\"torque\":"); vec(file_,dBodyGetTorque(body));
        }
        fprintf(file_, ",\"element_frame_world_rows_ijkc\":"); frame(file_, origin);
        if (static_data) {
            dMass mass; dBodyGetMass(body, &mass);
            fprintf(file_, ",\"mass\":"); number(file_,mass.mass);
            fprintf(file_, ",\"mass_com_local\":"); vec(file_, mass.c);
            fprintf(file_, ",\"inertia_ode_body_rows\":[");
            for (int row=0;row<3;++row) {
                if (row) fputc(',',file_); fputc('[',file_);
                for (int col=0;col<3;++col) { if (col) fputc(',',file_); number(file_,mass.I[row*4+col]); }
                fputc(']',file_);
            }
            fputc(']',file_);
            fprintf(file_, ",\"shapes\":[");
            for (u16 gidx=0; gidx<e->numberOfGeoms(); ++gidx) {
                CODEGeom* g = e->geometry(gidx);
                if (gidx) fputc(',', file_);
                Fmatrix local; g->get_local_form(local);
                fprintf(file_, "{\"bone_id\":%u,\"element_frame_rows_ijkc\":", unsigned(g->bone_id())); frame(file_, local);
                IKinematics* source_k = reference_kinematics_ ? reference_kinematics_ : shell_->PKinematics();
                if (source_k) {
                    const IBoneData& bone = source_k->GetBoneData(g->bone_id());
                    fprintf(file_, ",\"source_bone_shape_flags\":%u,\"source_game_material_id\":%u,\"source_bone_mass\":",
                        unsigned(bone.get_shape().flags.flags), unsigned(bone.get_game_mtl_idx()));
                    number(file_,bone.get_mass()); fprintf(file_, ",\"source_bone_com\":");
                    vec(file_, bone.get_center_of_mass());
                }
                const int wrapper_cls = dGeomGetClass(g->geometry_transform());
                const int cls = dGeomGetClass(g->geom());
                fprintf(file_, ",\"ode_wrapper_class\":%d,\"ode_primitive_class\":%d", wrapper_cls, cls);
                fprintf(file_, ",\"ode_wrapper_world_position\":"); vec(file_,dGeomGetPosition(g->geometry_transform()));
                fprintf(file_, ",\"ode_wrapper_world_rotation_rows\":"); ode_rotation(file_,dGeomGetRotation(g->geometry_transform()));
                fprintf(file_, ",\"ode_primitive_in_wrapper_position\":"); vec(file_,dGeomGetPosition(g->geom()));
                fprintf(file_, ",\"ode_primitive_in_wrapper_rotation_rows\":"); ode_rotation(file_,dGeomGetRotation(g->geom()));
                if (cls == dBoxClass) {
                    Fvector half; static_cast<CBoxGeom*>(g)->get_size(half);
                    fprintf(file_, ",\"type\":\"box\",\"half_size\":"); vec(file_, half);
                } else if (cls == dSphereClass) {
                    fprintf(file_, ",\"type\":\"sphere\",\"radius\":"); number(file_,static_cast<CSphereGeom*>(g)->radius());
                } else if (cls == dCylinderClassUser) {
                    CCylinderGeom* c = static_cast<CCylinderGeom*>(g);
                    fprintf(file_, ",\"type\":\"cylinder\",\"radius\":"); number(file_,c->radius());
                    fprintf(file_, ",\"height\":"); number(file_,c->height());
                } else fprintf(file_, ",\"type\":\"other\",\"ode_class\":%d", cls);
                fputc('}', file_);
            }
            fputc(']', file_);
        }
        fputc('}', file_);
    }
    void joint_state(CPHJoint* j, bool static_data, u16 joint_index, float elapsed_step)
    {
        const u16 source_axes = static_cast<CPhysicsJoint*>(j)->GetAxesNumber();
        // full_control's constructor also appends two slider metadata axes.
        // ODE has only three AMotor axes: querying phantom axes reads beyond
        // angle[3] and a stride-three cache would overwrite the next joint.
        const u16 physical_axes = source_axes > 3 ? 3 : source_axes;
        fprintf(file_, "{\"bone_id\":%u,\"type\":\"%s\",\"parent_bone_id\":%u,\"child_bone_id\":%u,\"source_axis_count\":%u,\"physical_axis_count\":%u,\"rate_contract\":\"immediate_solver_delta_v2\",\"axes\":[",
            unsigned(j->BoneID()), joint_type(j->eType),
            j->PFirstElement() ? unsigned(j->PFirstElement()->m_SelfID) : 65535u,
            j->PSecondElement() ? unsigned(j->PSecondElement()->m_SelfID) : 65535u,
            unsigned(source_axes), unsigned(physical_axes));
        for (u16 a=0; a<physical_axes; ++a) {
            if (a) fputc(',', file_);
            Fvector dir; j->GetAxisDirDynamic(a, dir);
            fprintf(file_, "{\"index\":%u,\"direction_world\":", unsigned(a)); vec(file_, dir);
            const float angle = j->GetAxisAngle(a);
            const unsigned slot = unsigned(joint_index)*3 + a;
            fprintf(file_, ",\"angle\":"); number(file_,angle);
            fprintf(file_, ",\"rate\":");
            if (!static_data && elapsed_step > 0.f && slot < previous_angle_valid_.size() && previous_angle_valid_[slot]) {
                float delta = angle - previous_angles_[slot];
                if (j->eType != CPhysicsJoint::slider) {
                    while (delta > PI) delta -= PI_MUL_2;
                    while (delta < -PI) delta += PI_MUL_2;
                }
                number(file_,double(delta / elapsed_step));
            } else fputs("null",file_);
            if (slot < previous_angle_valid_.size()) { previous_angles_[slot]=angle; previous_angle_valid_[slot]=1; }
            if (static_data) {
                float lo,hi; j->GetLimits(lo,hi,a);
                fprintf(file_, ",\"source_low\":"); number(file_,lo);
                fprintf(file_, ",\"source_high\":"); number(file_,hi);
            }
            if (j->eType != CPhysicsJoint::ball) {
                fprintf(file_, ",\"ode_low\":"); number(file_,joint_param(j,a,dParamLoStop));
                fprintf(file_, ",\"ode_high\":"); number(file_,joint_param(j,a,dParamHiStop));
                fprintf(file_, ",\"ode_cfm\":"); number(file_,joint_param(j,a,dParamCFM));
                fprintf(file_, ",\"ode_stop_erp\":"); number(file_,joint_param(j,a,dParamStopERP));
                fprintf(file_, ",\"ode_stop_cfm\":"); number(file_,joint_param(j,a,dParamStopCFM));
                fprintf(file_, ",\"ode_fmax\":"); number(file_,joint_param(j,a,dParamFMax));
                fprintf(file_, ",\"ode_vel\":"); number(file_,joint_param(j,a,dParamVel));
            }
            fputc('}', file_);
        }
        fprintf(file_, "]");
        if (static_data) {
            if (j->eType != CPhysicsJoint::slider) {
                Fvector anchor; j->GetAnchorDynamic(anchor);
                fprintf(file_, ",\"anchor_world\":"); vec(file_, anchor);
            }
            if ((j->eType == CPhysicsJoint::ball || j->eType == CPhysicsJoint::full_control) && j->GetDJoint()) {
                const dxJointBall* ball = static_cast<const dxJointBall*>(j->GetDJoint());
                fprintf(file_, ",\"ball_local_anchor_parent\":"); vec(file_,ball->anchor1);
                fprintf(file_, ",\"ball_local_anchor_child\":"); vec(file_,ball->anchor2);
            }
            if (j->eType == CPhysicsJoint::full_control && j->GetDJoint1()) {
                const dxJointAMotor* m = static_cast<const dxJointAMotor*>(j->GetDJoint1());
                fprintf(file_, ",\"amotor\":{\"mode\":%d,\"num_axes\":%d,\"rel\":[%d,%d,%d],\"axis\":[",
                    m->mode,m->num,m->rel[0],m->rel[1],m->rel[2]);
                for (int a=0;a<3;++a) { if (a) fputc(',',file_); vec(file_,m->axis[a]); }
                fprintf(file_, "],\"reference1\":"); vec(file_,m->reference1);
                fprintf(file_, ",\"reference2\":"); vec(file_,m->reference2);
                fprintf(file_, "}");
            }
            if (j->eType == CPhysicsJoint::hinge && j->GetDJoint()) {
                const dxJointHinge* h = static_cast<const dxJointHinge*>(j->GetDJoint());
                fprintf(file_, ",\"hinge\":{\"anchor1\":"); vec(file_,h->anchor1);
                fprintf(file_, ",\"anchor2\":"); vec(file_,h->anchor2);
                fprintf(file_, ",\"axis1\":"); vec(file_,h->axis1);
                fprintf(file_, ",\"axis2\":"); vec(file_,h->axis2);
                fprintf(file_, ",\"qrel\":[");
                for (int k=0;k<4;++k) { if (k) fputc(',',file_); number(file_,h->qrel[k]); }
                fprintf(file_, "],\"limot\":[");
                const double limot[] = {h->limot.vel,h->limot.fmax,h->limot.lostop,h->limot.histop,
                    h->limot.fudge_factor,h->limot.normal_cfm,h->limot.stop_erp,h->limot.stop_cfm,h->limot.bounce};
                for (int k=0;k<9;++k) { if (k) fputc(',',file_); number(file_,limot[k]); }
                fprintf(file_, "],\"joint_flags\":%d}",h->flags);
            }
        }
        fputc('}', file_);
    }
public:
    static void physics_step_callback(CPHIsland* island, bool after_solver, bool exact_solver)
    {
        if (active_capture_) active_capture_->physics_step(island, after_solver, exact_solver);
    }
    int body_index(dBodyID body) const
    {
        if (!body) return -1;
        for (u16 i=0;i<shell_->get_ElementsNumber();++i)
            if (static_cast<CPHElement*>(shell_->get_ElementByStoreOrder(i))->get_body() == body) return int(i);
        return -2; // Body from another shell in a merged island.
    }
    void physics_step(CPHIsland* island, bool after_solver, bool exact_solver)
    {
        if (!shell_ || !file_ || !island) return;
        if (shell_->get_ElementsNumber() != original_bodies_.size() || shell_->get_JointsNumber() != original_joints_.size()) {
            stop(nullptr,"topology_changed"); return;
        }
        for (u16 i=0;i<shell_->get_ElementsNumber();++i)
            if (shell_->get_ElementByStoreOrder(i) != original_bodies_[i]) { stop(nullptr,"topology_changed"); return; }
        for (u16 i=0;i<shell_->get_JointsNumber();++i)
            if (shell_->get_JointByStoreOrder(i) != original_joints_[i]) { stop(nullptr,"topology_changed"); return; }
        unsigned present = 0;
        bool contains_root = false;
        for (u16 i=0;i<shell_->get_ElementsNumber();++i) {
            const dBodyID target = static_cast<CPHElement*>(original_bodies_[i])->get_body();
            bool found = false;
            for (dxBody* b=island->firstbody; b; b=static_cast<dxBody*>(b->next))
                if (b == target) { found=true; break; }
            if (i == 0) contains_root=found;
            if (found) ++present;
        }
        if (!contains_root) return; // An unrelated island is stepping.
        if (present != shell_->get_ElementsNumber()) { stop(nullptr,"split_island"); return; }
        for (u16 i=0;i<shell_->get_ElementsNumber();++i)
            if (!valid_body_pose(static_cast<CPHElement*>(original_bodies_[i]))) { stop(nullptr,"invalid_numeric"); return; }
        const unsigned external = island->nb > shell_->get_ElementsNumber() ? unsigned(island->nb - shell_->get_ElementsNumber()) : 0u;
        fprintf(file_, "{\"type\":\"%s\",\"index\":%u,\"frame\":%u,\"t\":%u,\"tc\":%u,\"shell_enabled\":%s,\"dt\":",
            after_solver ? "post_step" : "pre_step",step_,Device.dwFrame,Device.dwTimeGlobal,Device.dwTimeContinual,
            shell_->isEnabled() ? "true" : "false");
        number(file_,fixed_step);
        fprintf(file_, ",\"solver\":\"%s\",\"island_body_count\":%d,\"island_joint_count\":%d,\"external_body_count\":%u",
            exact_solver ? "exact" : "quickstep",island->nb,island->nj,external);
        if (!after_solver) {
            dVector3 gravity; dWorldGetGravity(island->DWorld(),gravity);
            fprintf(file_, ",\"world\":{\"gravity\":"); vec(file_,gravity);
            fprintf(file_, ",\"erp\":"); number(file_,dWorldGetERP(island->DWorld()));
            fprintf(file_, ",\"cfm\":"); number(file_,dWorldGetCFM(island->DWorld()));
            fprintf(file_, ",\"quickstep_iterations\":%d}",dWorldGetQuickStepNumIterations(island->DWorld()));
        }
        fprintf(file_, ",\"bodies\":[");
        for (u16 i=0;i<shell_->get_ElementsNumber();++i) {
            if (i) fputc(',',file_);
            body_state(static_cast<CPHElement*>(shell_->get_ElementByStoreOrder(i)),false,!after_solver);
        }
        fprintf(file_, "],\"joints\":[");
        for (u16 i=0;i<shell_->get_JointsNumber();++i) {
            if (i) fputc(',',file_);
            joint_state(static_cast<CPHJoint*>(shell_->get_JointByStoreOrder(i)),false,i,after_solver ? fixed_step : 0.f);
        }
        fputc(']',file_);
        if (!after_solver) {
            fprintf(file_, ",\"contacts\":[");
            bool first = true;
            unsigned contact_count = 0;
            bool contacts_truncated = false;
            for (dxJoint* joint=island->firstjoint; joint; joint=static_cast<dxJoint*>(joint->next)) {
                if (dJointGetType(joint) != dJointTypeContact) continue;
                const int body1_index = body_index(joint->node[0].body);
                const int body2_index = body_index(joint->node[1].body);
                if (manual_request_ && body1_index < 0 && body2_index < 0) continue;
                if (manual_request_ && contact_count >= manual_contact_limit_) { contacts_truncated=true; break; }
                ++contact_count;
                const dxJointContact* c = static_cast<const dxJointContact*>(joint);
                const dContact& contact = c->contact;
                if (!first) fputc(',',file_); first=false;
                fprintf(file_, "{\"body1_index\":%d,\"body2_index\":%d,\"joint_flags\":%d,\"reverse\":%s,\"special\":%s,\"position\":",
                    body1_index,body2_index,
                    joint->flags,(joint->flags & dJOINT_REVERSE) ? "true" : "false",
                    PHIslandContactIsSpecial(joint) ? "true" : "false");
                vec(file_,contact.geom.pos); fprintf(file_, ",\"normal\":"); vec(file_,contact.geom.normal);
                fprintf(file_, ",\"depth\":"); number(file_,contact.geom.depth);
                fprintf(file_, ",\"geom1_class\":%d,\"geom2_class\":%d,\"fdir1\":",
                    contact.geom.g1 ? dGeomGetClass(contact.geom.g1) : -1,
                    contact.geom.g2 ? dGeomGetClass(contact.geom.g2) : -1);
                if (contact.surface.mode & dContactFDir1) vec(file_,contact.fdir1);
                else fputs("[0,0,0]",file_);
                fprintf(file_, ",\"surface\":{\"mode\":%d,\"mu\":",contact.surface.mode); number(file_,contact.surface.mu);
                fprintf(file_, ",\"mu2\":"); number(file_,(contact.surface.mode & dContactMu2) ? contact.surface.mu2 : 0);
                fprintf(file_, ",\"bounce\":"); number(file_,(contact.surface.mode & dContactBounce) ? contact.surface.bounce : 0);
                fprintf(file_, ",\"bounce_vel\":"); number(file_,(contact.surface.mode & dContactBounce) ? contact.surface.bounce_vel : 0);
                fprintf(file_, ",\"soft_erp\":"); number(file_,(contact.surface.mode & dContactSoftERP) ? contact.surface.soft_erp : 0);
                fprintf(file_, ",\"soft_cfm\":"); number(file_,(contact.surface.mode & dContactSoftCFM) ? contact.surface.soft_cfm : 0);
                fprintf(file_, ",\"motion1\":"); number(file_,(contact.surface.mode & dContactMotion1) ? contact.surface.motion1 : 0);
                fprintf(file_, ",\"motion2\":"); number(file_,(contact.surface.mode & dContactMotion2) ? contact.surface.motion2 : 0);
                fprintf(file_, ",\"slip1\":"); number(file_,(contact.surface.mode & dContactSlip1) ? contact.surface.slip1 : 0);
                fprintf(file_, ",\"slip2\":"); number(file_,(contact.surface.mode & dContactSlip2) ? contact.surface.slip2 : 0);
                fprintf(file_, "}}");
            }
            fputc(']',file_);
            if (manual_request_) fprintf(file_, ",\"contact_count\":%u,\"contacts_truncated\":%s",
                contact_count, contacts_truncated ? "true" : "false");
        }
        fprintf(file_, "}\n"); fflush(file_);
        if (invalid_numeric_) { stop(nullptr,"invalid_numeric"); return; }
        if (manual_request_ && ftell(file_) >= manual_file_limit_) {
            if (after_solver) ++step_; // count the completed post-step record above
            stop(nullptr,"size_limit"); return;
        }
        if (after_solver && ++step_ >= trace_steps_) stop(nullptr,"complete");
    }
    void stop(CPhysicsShell* only = nullptr, const char* reason = "stopped")
    {
        if (only && only != shell_) return;
        if (IsActive()) Deactivate();
        if (active_capture_ == this) {
            // Do not erase a hook another diagnostic installed after this one.
            if (g_ph_island_step_capture == &CRagdollShellCapture::physics_step_callback)
                g_ph_island_step_capture=nullptr;
            active_capture_=nullptr;
        }
        if (file_) { fprintf(file_, "{\"type\":\"end\",\"steps\":%u,\"frame\":%u,\"t\":%u,\"tc\":%u,\"reason\":\"%s\"}\n",
            step_,Device.dwFrame,Device.dwTimeGlobal,Device.dwTimeContinual,reason); fclose(file_); file_=nullptr; }
        shell_=nullptr; step_=0;
        manual_request_=false; manual_started_tc_=0;
        reference_kinematics_=nullptr;
        previous_angles_.clear(); previous_angle_valid_.clear();
        original_bodies_.clear(); original_joints_.clear();
        invalid_numeric_=false;
    }
    bool manual_available() const
    {
        return !file_ && !active_capture_ && !painter_alive_reference_target &&
            !painter_ragdoll_capture_next && !g_ph_island_step_capture;
    }
    void start(CPhysicsShell* shell, CPhysicsShellHolder& entity, u16 requested_id = u16(-1))
    {
        stop();
        if (!shell || !shell->PIsland()) return;
        string_path path, filename;
        xr_sprintf(filename, "painter_ragdoll_capture_%u_%u.jsonl", Device.dwTimeGlobal, unsigned(entity.ID()));
        FS.update_path(path, "$logs$", filename);
        file_ = fopen(path, "wb");
        if (!file_) { Msg("! painter ragdoll capture: cannot open %s", path); return; }
        shell_=shell;
        manual_request_=requested_id != u16(-1);
        manual_started_tc_=Device.dwTimeContinual;
        for (u16 i=0;i<shell->get_ElementsNumber();++i) original_bodies_.push_back(shell->get_ElementByStoreOrder(i));
        for (u16 i=0;i<shell->get_JointsNumber();++i) original_joints_.push_back(shell->get_JointByStoreOrder(i));
        previous_angles_.assign(unsigned(shell->get_JointsNumber())*3,0.f);
        previous_angle_valid_.assign(unsigned(shell->get_JointsNumber())*3,0);
        dWorldID world = shell->PIsland()->DWorld();
        dVector3 gravity; dWorldGetGravity(world,gravity);
        fprintf(file_, "{\"type\":\"header\",\"schema\":1,\"source\":\"live-game\",\"observation_scope\":\"immediate pre/post solver\",\"entity_id\":%u,\"resolved_entity_id\":%u,\"requested_entity_id\":",
            unsigned(entity.ID()),unsigned(entity.ID()));
        if (manual_request_) fprintf(file_, "%u",unsigned(requested_id)); else fputs("null",file_);
        fprintf(file_, ",\"frame\":%u,\"t\":%u,\"tc\":%u,\"contact_scope\":\"%s\",\"manual_contact_limit_per_step\":%u,\"manual_file_limit_bytes\":%ld,\"manual_time_limit_ms\":%u,\"section\":",
            Device.dwFrame,Device.dwTimeGlobal,Device.dwTimeContinual,
            manual_request_ ? "target shell contacts only" : "merged island contacts",
            manual_request_ ? manual_contact_limit_ : 0u,
            manual_request_ ? manual_file_limit_ : 0L, manual_request_ ? 2000u : 0u);
        name(file_, entity.cNameSect().c_str());
        fprintf(file_, ",\"visual\":"); name(file_, entity.cNameVisual().c_str());
        fprintf(file_, ",\"body_count\":%u,\"joint_count\":%u,\"fixed_step\":",
            unsigned(shell->get_ElementsNumber()),unsigned(shell->get_JointsNumber()));
        number(file_,fixed_step); fprintf(file_, ",\"ph_step_time\":"); number(file_,ph_console::ph_step_time);
        fprintf(file_, ",\"gravity\":"); vec(file_,gravity);
        fprintf(file_, ",\"world_erp\":"); number(file_,dWorldGetERP(world));
        fprintf(file_, ",\"world_cfm\":"); number(file_,dWorldGetCFM(world));
        fprintf(file_, ",\"quickstep_iterations\":%d,\"prefers_exact_integration\":%s,\"forces_exact_integration\":%s,\"trace_target_steps\":%u,\"unavailable\":[\"contact_impulses\",\"contact_material_pairs\"]}\n",
            dWorldGetQuickStepNumIterations(world),
            shell->PIsland()->PrefersExactIntegration() ? "true" : "false", shell->PIsland()->ForcesExactIntegration() ? "true" : "false", trace_steps_);
        if (manual_request_ && ftell(file_) >= manual_file_limit_) { stop(nullptr,"size_limit"); return; }
        IKinematics* k = shell->PKinematics();
        for (u16 i=0;i<shell->get_ElementsNumber();++i)
            if (!valid_body_pose(static_cast<CPHElement*>(original_bodies_[i]))) { stop(nullptr,"invalid_numeric"); return; }
        for (u16 i=0;i<shell->get_ElementsNumber();++i) {
            CPHElement* e = static_cast<CPHElement*>(shell->get_ElementByStoreOrder(i));
            fprintf(file_, "{\"type\":\"body\",\"index\":%u,\"name\":",unsigned(i));
            name(file_,k ? k->LL_BoneName_dbg(e->m_SelfID) : "");
            fprintf(file_, ",\"data\":"); body_state(e,true); fprintf(file_, "}\n");
            if (manual_request_ && ftell(file_) >= manual_file_limit_) { stop(nullptr,"size_limit"); return; }
        }
        for (u16 i=0;i<shell->get_JointsNumber();++i) {
            CPHJoint* j = static_cast<CPHJoint*>(shell->get_JointByStoreOrder(i));
            fprintf(file_, "{\"type\":\"joint\",\"index\":%u,\"name\":",unsigned(i));
            name(file_,k ? k->LL_BoneName_dbg(j->BoneID()) : "");
            fprintf(file_, ",\"data\":"); joint_state(j,true,i,0.f); fprintf(file_, "}\n");
            if (manual_request_ && ftell(file_) >= manual_file_limit_) { stop(nullptr,"size_limit"); return; }
        }
        fflush(file_);
        if (invalid_numeric_) { stop(nullptr,"invalid_numeric"); return; }
        if (manual_request_ && ftell(file_) >= manual_file_limit_) { stop(nullptr,"size_limit"); return; }
        active_capture_=this;
        g_ph_island_step_capture=&CRagdollShellCapture::physics_step_callback;
        Activate(); // Receive NetRelcase before the shell can be destroyed.
        Msg("* painter ragdoll capture: %s", path);
    }
    // Static reference only. The shell is an unattached ghost; no solver hook,
    // actor callback, world membership, or simulated standing is implied.
    bool write_alive_reference(CPhysicsShell* shell, CEntityAlive& entity,
        IKinematics* k, u16 animated_root_bone_id, u32 stable_frames, float movement_speed,
        const Fvector& animated_root_world)
    {
        if (!shell || !k || file_ || active_capture_ || !shell->get_ElementsNumber()) return false;
        string_path path, filename;
        xr_sprintf(filename, "painter_alive_reference_%u_%u.jsonl", Device.dwTimeGlobal, unsigned(entity.ID()));
        FS.update_path(path, "$logs$", filename);
        file_ = fopen(path, "wb");
        if (!file_) { Msg("! painter alive reference: cannot open %s", path); return false; }
        shell_ = shell;
        reference_kinematics_ = k;
        previous_angles_.assign(unsigned(shell->get_JointsNumber())*3,0.f);
        previous_angle_valid_.assign(unsigned(shell->get_JointsNumber())*3,0);
        fprintf(file_, "{\"type\":\"header\",\"schema\":\"alive-shell-reference-v1\",\"source\":\"live-game-animation-plus-inactive-native-shell\",\"reference_only\":true,\"solver_steps\":0,\"entity_id\":%u,\"section\":", unsigned(entity.ID()));
        name(file_,entity.cNameSect().c_str());
        fprintf(file_, ",\"visual\":"); name(file_,entity.cNameVisual().c_str());
        fprintf(file_, ",\"body_count\":%u,\"joint_count\":%u,\"frame\":%u,\"time_global_ms\":%u,\"stable_frames\":%u,\"movement_speed\":",
            unsigned(shell->get_ElementsNumber()),unsigned(shell->get_JointsNumber()),Device.dwFrame,Device.dwTimeGlobal,stable_frames);
        number(file_,movement_speed);
        fprintf(file_, ",\"animated_root_bone_id\":%u,\"physics_root_bone_id\":%u,\"physics_root_body_bone_id\":%u,\"animated_root_bone_world\":",
            unsigned(animated_root_bone_id),unsigned(k->LL_BoneID("bip01_pelvis")),
            unsigned(static_cast<CPHElement*>(shell->get_ElementByStoreOrder(0))->m_SelfID)); vec(file_,animated_root_world);
        CInventoryOwner* inventory_owner = smart_cast<CInventoryOwner*>(&entity);
        PIItem active_item = inventory_owner ? inventory_owner->inventory().ActiveItem() : nullptr;
        fprintf(file_, ",\"active_item_id\":");
        if (active_item) fprintf(file_, "%u",unsigned(active_item->object().ID()));
        else fprintf(file_, "null");
        fprintf(file_, ",\"active_item_section\":");
        if (active_item) name(file_,active_item->object().cNameSect().c_str());
        else fprintf(file_, "null");
        fprintf(file_, ",\"active_item_parent_is_entity\":%s,\"weapon_collision_in_ghost\":false,",
            active_item && active_item->object().H_Parent()==&entity ? "true" : "false");
        fprintf(file_, "\"joint_reference_provenance\":\"native bind-built joints retained while bodies follow current animation; local anchors and Euler/hinge references are not rebased\",\"body_velocity_provenance\":\"detached ghost starts with zero ODE body velocities; not measured animated bone velocities\",\"actuator_provenance\":\"living character has no ragdoll actuator; death controller is phase-limited PD and must be evaluated separately\"}\n");
        for (u16 i=0;i<shell->get_ElementsNumber();++i)
            if (!valid_body_pose(static_cast<CPHElement*>(shell->get_ElementByStoreOrder(i)))) {
                stop(nullptr,"invalid_numeric"); return false;
            }
        for (u16 i=0;i<shell->get_ElementsNumber();++i) {
            CPHElement* e = static_cast<CPHElement*>(shell->get_ElementByStoreOrder(i));
            fprintf(file_, "{\"type\":\"body\",\"index\":%u,\"name\":",unsigned(i));
            name(file_,k->LL_BoneName_dbg(e->m_SelfID));
            fprintf(file_, ",\"data\":"); body_state(e,true); fprintf(file_, "}\n");
        }
        for (u16 i=0;i<shell->get_JointsNumber();++i) {
            CPHJoint* j = static_cast<CPHJoint*>(shell->get_JointByStoreOrder(i));
            fprintf(file_, "{\"type\":\"joint\",\"index\":%u,\"name\":",unsigned(i));
            name(file_,k->LL_BoneName_dbg(j->BoneID()));
            fprintf(file_, ",\"data\":"); joint_state(j,true,i,0.f); fprintf(file_, "}\n");
        }
        const bool success = !invalid_numeric_;
        stop(nullptr,success ? "reference_complete" : "invalid_numeric");
        Msg("* painter alive reference: %s (%s)",path,success ? "complete" : "invalid numeric");
        return success;
    }
    void PhDataUpdate(float) override
    {
        // Global update enrollment also expires a manual capture whose target sleeps.
        // A paused/frozen world checks this deadline when physics resumes.
        if (manual_request_ && file_ && u32(Device.dwTimeContinual-manual_started_tc_) >= 2000u)
            stop(nullptr,step_ ? "time_limit" : "no_target_steps");
    }
    void PhTune(float) override {}
    void NetRelcase(CPhysicsShell* s) override { stop(s,"shell_destroyed"); }
} ragdoll_shell_capture;
CRagdollShellCapture* CRagdollShellCapture::active_capture_ = nullptr;

// One transient request, resolved only at the joined main-thread barrier. The
// stored object pointer is compared for equality only, never dereferenced later.
// This check does not establish an object generation or rule out address reuse.
class CRequestedRagdollCapture
{
    bool armed_ = false;
    u16 requested_id_ = u16(-1);
    const CObject* requested_object_ = nullptr;
    const void* requested_level_ = nullptr;
    u32 requested_tc_ = 0;
public:
    bool Queue(u16 object_id)
    {
        if (armed_ || object_id == u16(-1) || !g_pGameLevel || !Level().game || Device.Paused()) return false;
        CObject* object = Level().Objects.net_Find(object_id);
        if (!object || object->getDestroy()) return false;
        xrCriticalSectionGuard guard(&Device.seqParallelBeforRenderCS);
        Device.seqParallelBeforRender.push_back(xr_make_delegate(this,&CRequestedRagdollCapture::Apply));
        armed_=true; requested_id_=object_id; requested_object_=object;
        requested_level_=g_pGameLevel; requested_tc_=Device.dwTimeContinual;
        Msg("* painter ragdoll capture queued: requested_id=%u frame=%u t=%u tc=%u",
            unsigned(object_id),Device.dwFrame,Device.dwTimeGlobal,Device.dwTimeContinual);
        return true;
    }
    void CancelPending()
    {
        armed_=false; requested_id_=u16(-1); requested_object_=nullptr; requested_level_=nullptr; requested_tc_=0;
        const auto callback = xr_make_delegate(this,&CRequestedRagdollCapture::Apply);
        xrCriticalSectionGuard guard(&Device.seqParallelBeforRenderCS);
        for (auto& entry : Device.seqParallelBeforRender)
            if (entry == callback) entry.clear(); // do not erase while this queue drains
        Msg("* painter ragdoll capture: pending request cleared");
    }
    void Apply()
    {
        if (!armed_) return;
        const u16 requested_id=requested_id_;
        const CObject* requested_object=requested_object_;
        const void* requested_level=requested_level_;
        const u32 requested_tc=requested_tc_;
        armed_=false; requested_id_=u16(-1); requested_object_=nullptr; requested_level_=nullptr; requested_tc_=0;
        if (!Device.IsGeometryPublishBarrier() || !g_pGameLevel || requested_level != g_pGameLevel || !Level().game ||
            Device.Paused() || !physics_world() || physics_world()->Processing() ||
            static_cast<CPHWorld*>(physics_world())->IsFreezed() ||
            u32(Device.dwTimeContinual-requested_tc) >= 1000u || !ragdoll_shell_capture.manual_available()) {
            Msg("! painter ragdoll capture rejected: requested_id=%u reason=barrier_expired_or_capture_busy",unsigned(requested_id));
            return;
        }
        CObject* object=Level().Objects.net_Find(requested_id);
        if (!object || object != requested_object || object->getDestroy()) {
            Msg("! painter ragdoll capture rejected: requested_id=%u reason=current_object_changed",unsigned(requested_id));
            return;
        }
        CAI_Stalker* npc=smart_cast<CAI_Stalker*>(object);
        CPhysicsSkeletonObject* piece=smart_cast<CPhysicsSkeletonObject*>(object);
        CPhysicsShellHolder* holder=smart_cast<CPhysicsShellHolder*>(object);
        if (!holder || (!piece && (!npc || npc->g_Alive()))) {
            Msg("! painter ragdoll capture rejected: requested_id=%u reason=target_not_dead_stalker_or_piece",unsigned(requested_id));
            return;
        }
        CPhysicsShell* shell=holder->PPhysicsShell();
        if (!shell || !shell->get_ElementsNumber() || !shell->isActive() || !shell->isFullActive() ||
            static_cast<CPHShell*>(shell)->IsFreezed() || !shell->PIsland()) {
            Msg("! painter ragdoll capture rejected: requested_id=%u reason=shell_absent_frozen_or_inactive",unsigned(requested_id));
            return;
        }
        IPhysicsShellHolder* expected_owner=static_cast<IPhysicsShellHolder*>(holder);
        for (u16 i=0;i<shell->get_ElementsNumber();++i) {
            CPhysicsElement* element=shell->get_ElementByStoreOrder(i);
            if (!element || element->PhysicsRefObject()!=expected_owner) {
                Msg("! painter ragdoll capture rejected: requested_id=%u reason=body_owner_mismatch",unsigned(requested_id));
                return;
            }
            if (!element->isFullActive()) {
                Msg("! painter ragdoll capture rejected: requested_id=%u reason=activating_body",unsigned(requested_id));
                return;
            }
        }
        // Sleeping is allowed: enabled state is observed without waking the shell.
        ragdoll_shell_capture.start(shell,*holder,requested_id);
    }
} requested_ragdoll_capture;
}

bool painter_queue_ragdoll_capture(u16 object_id) { return requested_ragdoll_capture.Queue(object_id); }
void painter_cancel_pending_ragdoll_capture() { requested_ragdoll_capture.CancelPending(); }

//void  NodynamicsCollide( bool& do_colide, bool bo1, dContact& c, SGameMtl * /*material_1*/, SGameMtl * /*material_2*/ )
//{
//	dBodyID body1=dGeomGetBody( c.geom.g1 );
//	dBodyID body2=dGeomGetBody( c.geom.g2 );
//	if( !body1 || !body2 || ( dGeomUserDataHasCallback( c.geom.g1,NodynamicsCollide )&& dGeomUserDataHasCallback( c.geom.g2, NodynamicsCollide ) ) )
//		return;
//	do_colide = false; 
//}



// Explicit geometry-only admission probe. The owned test launcher verifies the
// exact GFY bytes under these hash aliases before launch. This is not a public
// arbitrary-model admission API or a physical amputation implementation.
void CCharacterPhysicsSupport::CancelPainterGeometryProbe()
{
    CancelPainterVoxelLive();
    m_painter_runtime_cookie = 0;
    m_painter_runtime_state = 0;
    m_painter_geometry_pending = -1;
    auto callback = xr_make_delegate(this, &CCharacterPhysicsSupport::ApplyPainterGeometryProbe);
    xrCriticalSectionGuard guard(&Device.seqParallelBeforRenderCS);
    for (auto& entry : Device.seqParallelBeforRender)
        if (entry == callback) entry.clear(); // never erase while the queue drains
}

bool CCharacterPhysicsSupport::QueuePainterGeometryProbe(u32 mode)
{
    if (mode > 4 || m_painter_geometry_pending >= 0 || m_eType != etStalker ||
        m_EntityAlife.g_Alive() || m_EntityAlife.getDestroy() || !m_pPhysicsShell)
        return false;
    m_painter_geometry_pending = static_cast<int>(mode);
    xrCriticalSectionGuard guard(&Device.seqParallelBeforRenderCS);
    Device.seqParallelBeforRender.push_back(
        xr_make_delegate(this, &CCharacterPhysicsSupport::ApplyPainterGeometryProbe));
    Msg("[butcher_geometry_native] id=%u mode=%u phase=queued geometry_only=true",
        m_EntityAlife.ID(), mode);
    return true;
}

void CCharacterPhysicsSupport::ApplyPainterGeometryProbe()
{
    const int mode = m_painter_geometry_pending;
    m_painter_geometry_pending = -1;
    if (mode < 0) return;
    if (mode >= 100) { ApplyPainterRuntimeGeometryProbe(u32(mode - 100)); return; }
    // This callback runs in the main-thread queue after previous workers have
    // joined, before FrameMove starts new work. The queue owns our lifetime via
    // explicit cancellation on net destroy, visual change and destruction.
    if (m_eType != etStalker || m_EntityAlife.g_Alive() || m_EntityAlife.getDestroy() ||
        !m_pPhysicsShell || !m_pPhysicsShell->isActive() || !physics_world() ||
        physics_world()->Processing() || Device.Paused() ||
        static_cast<CPHShell*>(m_pPhysicsShell)->IsFreezed())
    {
        Msg("[butcher_geometry_native] id=%u mode=%d phase=rejected reason=owner_shell_or_barrier", m_EntityAlife.ID(), mode);
        return;
    }
    for (u16 i = 0; i < m_pPhysicsShell->get_ElementsNumber(); ++i)
        if (!m_pPhysicsShell->get_ElementByStoreOrder(i)->isFullActive())
        {
            Msg("[butcher_geometry_native] id=%u mode=%d phase=rejected reason=activating_body", m_EntityAlife.ID(), mode);
            return;
        }
    IKinematics* k = smart_cast<IKinematics*>(m_EntityAlife.Visual());
    if (!k || !k->LL_BoneCount() || !m_weapon_geoms.empty() || m_weapon_attach_bone)
    {
        Msg("[butcher_geometry_native] id=%u mode=%d phase=rejected reason=rig_or_weapon", m_EntityAlife.ID(), mode);
        return;
    }
    const char* source_hand = "painter\\butcher_review\\gfy_sources\\4be67c0445cef074a61fabfabc76db1c5d0cd73f75486a09e15b197f3e714276";
    const char* source_calf = "painter\\butcher_review\\gfy_sources\\99474d54c1dd24767d057f71e95ac6bbdcb2dae63fb04b0120708addf8c75aa8";
    const char* actual_source = *m_EntityAlife.cNameVisual();
    const bool is_hand = actual_source && !xr_strcmp(actual_source, source_hand);
    const bool is_calf = actual_source && !xr_strcmp(actual_source, source_calf);
    if ((!is_hand && !is_calf) || ((mode == 1 || mode == 3) && !is_hand) || ((mode == 2 || mode == 4) && !is_calf) ||
        (mode == 0 && !m_painter_geometry_installed))
    {
        Msg("[butcher_geometry_native] id=%u mode=%d phase=rejected reason=source_or_state visual=%s",
            m_EntityAlife.ID(), mode, actual_source ? actual_source : "<null>");
        return;
    }
    // Both-arm diagnostics require the exact ordinary GFY rig and the
    // successful pre-shell hide state. This does not transfer arm physics.
    if (mode == 3 || mode == 4)
    {
        bool hidden = k->LL_BoneCount() == 47 &&
            k->LL_BoneID("bip01_l_upperarm") == 21 && k->LL_BoneID("bip01_r_upperarm") == 34;
        for (u16 i = 21; hidden && i <= 32; ++i) hidden = !k->LL_GetBoneVisible(i);
        for (u16 i = 34; hidden && i <= 45; ++i) hidden = !k->LL_GetBoneVisible(i);
        if (!hidden || m_pPhysicsShell->get_ElementsNumber() != 7 || m_pPhysicsShell->get_JointsNumber() != 6)
        {
            Msg("[butcher_geometry_native] id=%u mode=%d phase=rejected reason=both_arm_hide_or_shell", m_EntityAlife.ID(), mode);
            return;
        }
    }
    CPhysicsShell* shell = m_pPhysicsShell;
    const void* bone_instances = &k->LL_GetBoneInstance(0);
    const void* shared_rig = &k->LL_GetData(0);
    const u64 visibility = k->LL_GetBonesVisible();
    const u16 root = k->LL_GetBoneRoot();
    xr_vector<CPhysicsElement*> bodies;
    xr_vector<CPhysicsJoint*> joints;
    for (u16 i = 0; i < shell->get_ElementsNumber(); ++i) bodies.push_back(shell->get_ElementByStoreOrder(i));
    for (u16 i = 0; i < shell->get_JointsNumber(); ++i) joints.push_back(shell->get_JointByStoreOrder(i));
    bool applied = false;
    if (mode == 0)
    {
        k->LL_ClearGeometryVariant();
        m_painter_geometry_installed = false;
        applied = true;
    }
    else
    {
        LPCSTR donor_path = mode == 1 ? "painter\\butcher_native_v2\\neutral0a_body_closed_left_hand"
            : mode == 2 ? "painter\\butcher_native_v2\\greh_body_closed_left_calf"
            : mode == 3 ? "painter\\detached_arms_v1\\neutral_body_closed_both_arms"
                        : "painter\\detached_arms_v1\\coat_body_closed_both_arms";
        string_path resource;
        xr_strconcat(resource, donor_path, ".ogf");
        if (!FS.exist("$game_meshes$", resource))
        {
            Msg("[butcher_geometry_native] id=%u mode=%d phase=rejected reason=donor_missing", m_EntityAlife.ID(), mode);
            return;
        }
        IRenderVisual* staged = ::Render->model_Create(donor_path);
        applied = staged && k->LL_InstallGeometryVariant(staged);
        if (staged) ::Render->model_Delete(staged);
        if (applied) m_painter_geometry_installed = true;
    }
    bool identities = m_pPhysicsShell == shell && smart_cast<IKinematics*>(m_EntityAlife.Visual()) == k &&
        &k->LL_GetBoneInstance(0) == bone_instances && &k->LL_GetData(0) == shared_rig &&
        k->LL_GetBonesVisible() == visibility && k->LL_GetBoneRoot() == root &&
        shell->get_ElementsNumber() == bodies.size() && shell->get_JointsNumber() == joints.size();
    for (u16 i = 0; identities && i < bodies.size(); ++i) identities = shell->get_ElementByStoreOrder(i) == bodies[i];
    for (u16 i = 0; identities && i < joints.size(); ++i) identities = shell->get_JointByStoreOrder(i) == joints[i];
    Msg("[butcher_geometry_native] id=%u mode=%d phase=%s geometry_only=true identities=%s bodies=%u joints=%u bone_instances=%p shared_rig=%p shell=%p frame=%u physics_processing=%d",
        m_EntityAlife.ID(), mode, applied ? (mode == 0 ? "restored" : "committed") : "rejected_rig_contract",
        identities ? "unchanged" : "FAILED", (u32)bodies.size(), (u32)joints.size(), bone_instances, shared_rig, shell,
        Device.dwFrame, physics_world()->Processing() ? 1 : 0);
    R_ASSERT2(identities, "Butcher geometry probe unexpectedly changed rig or physics identities");
}

bool painter_queue_geometry_probe(u16 object_id, u32 mode)
{
    if (!g_pGameLevel) return false;
    CAI_Stalker* npc = smart_cast<CAI_Stalker*>(Level().Objects.net_Find(object_id));
    return npc && npc->character_physics_support() &&
        npc->character_physics_support()->QueuePainterGeometryProbe(mode);
}

void painter_cancel_alive_reference()
{
    painter_alive_reference_target = nullptr;
    painter_alive_reference_target_id = u16(-1);
    painter_alive_reference_armed_at = 0;
}

bool CCharacterPhysicsSupport::QueuePainterAliveReference()
{
    if (painter_alive_reference_target || m_eType != etStalker || m_eState != esAlive || !m_EntityAlife.g_Alive() ||
        m_EntityAlife.getDestroy() || m_pPhysicsShell || m_physics_skeleton ||
        !movement() || !movement()->CharacterExist() || !physics_world() ||
        physics_world()->Processing()) return false;
    painter_alive_reference_target = this;
    painter_alive_reference_target_id = m_EntityAlife.ID();
    painter_alive_reference_armed_at = Device.dwTimeGlobal;
    m_painter_alive_reference_frames = 0;
    m_painter_alive_reference_has_previous = false;
    Msg("* painter alive reference armed: entity=%u frame=%u expires_ms=%u",
        unsigned(m_EntityAlife.ID()),Device.dwFrame,Device.dwTimeGlobal+10000u);
    return true;
}

bool painter_queue_alive_reference(u16 object_id)
{
    if (!g_pGameLevel) return false;
    CAI_Stalker* npc = smart_cast<CAI_Stalker*>(Level().Objects.net_Find(object_id));
    return npc && npc->character_physics_support() &&
        npc->character_physics_support()->QueuePainterAliveReference();
}

IC bool is_imotion(interactive_motion* im)
{
	return im && im->is_enabled();
}

CCharacterPhysicsSupport::~CCharacterPhysicsSupport()
{
    painter_piece_continuation::forget(m_EntityAlife);
    painter_arm_pose_owner_gone(m_EntityAlife);
	if (painter_alive_reference_target == this) painter_cancel_alive_reference();
	CancelPainterGeometryProbe();
	if (m_pPhysicsShell) ragdoll_shell_capture.stop(m_pPhysicsShell);
#ifdef PAINTER_ACTIVE_RAGDOLL_NATIVE
	m_active_ragdoll_controller.detach();
#endif
	xr_delete(m_collision_hit_callback);
	if (m_flags.test(fl_skeleton_in_shell))
	{
		if (m_physics_skeleton)
			m_physics_skeleton->Deactivate();
		xr_delete(m_physics_skeleton); //!b_skeleton_in_shell
	}
	xr_delete(m_PhysicMovementControl);
	xr_delete(m_collision_activating_delay);
	VERIFY(!m_interactive_motion);
	xr_delete(m_collision_activating_delay);
	bone_fix_clear();
}

CCharacterPhysicsSupport::CCharacterPhysicsSupport(EType atype, CEntityAlive* aentity)
	: m_pPhysicsShell(aentity->PPhysicsShell()),
	  m_EntityAlife(*aentity),
	  mXFORM(aentity->XFORM()),
	  m_ph_sound_player(aentity),
	  m_interactive_motion(0),
	  m_PhysicMovementControl(xr_new<CPHMovementControl>(aentity)),
	  m_eType(atype),
	  m_eState(esAlive),
	  m_physics_skeleton(NULL),
	  m_ik_controller(NULL),
	  m_BonceDamageFactor(1.f),
	  m_collision_hit_callback(NULL),
	  m_interactive_animation(NULL),
	  m_physics_shell_animated(NULL),
	  m_physics_shell_animated_time_destroy(u32(-1)),
	  m_weapon_attach_bone(0),
	  m_active_item_obj(0),
	  m_hit_valide_time(u32(-1)),
	  m_collision_activating_delay(NULL)
{
	m_flags.assign(0);
	m_flags.set(fl_death_anim_on,FALSE);
	m_flags.set(fl_skeleton_in_shell,FALSE);
	m_flags.set(fl_use_hit_anims,TRUE);
	m_pPhysicsShell = NULL;
	switch (atype)
	{
	case etActor:
		m_PhysicMovementControl->AllocateCharacterObject(CPHMovementControl::actor);
		m_PhysicMovementControl->SetRestrictionType(rtActor);
		break;
	case etStalker:
		m_PhysicMovementControl->AllocateCharacterObject(CPHMovementControl::ai);
		m_PhysicMovementControl->SetRestrictionType(rtStalker);
		m_PhysicMovementControl->SetActorMovable(false);
		break;
	case etBitting:
		m_PhysicMovementControl->AllocateCharacterObject(CPHMovementControl::ai);

		m_PhysicMovementControl->SetRestrictionType(rtMonsterMedium);
		//m_PhysicMovementControl->SetActorMovable(false);
	}
};

void CCharacterPhysicsSupport::SetRemoved()
{
	if (painter_alive_reference_target == this) painter_cancel_alive_reference();
#ifdef PAINTER_ACTIVE_RAGDOLL_NATIVE
	m_active_ragdoll_controller.detach();
#endif
	m_eState = esRemoved;
	if (m_flags.test(fl_skeleton_in_shell))
	{
		if (m_pPhysicsShell->isEnabled())
			m_EntityAlife.processing_deactivate();
		if (m_pPhysicsShell)
			m_pPhysicsShell->Deactivate();
		xr_delete(m_pPhysicsShell);
	}
	else
	{
		if (m_physics_skeleton)
			m_physics_skeleton->Deactivate();
		xr_delete(m_physics_skeleton);
		m_EntityAlife.processing_deactivate();
	}
}

void CCharacterPhysicsSupport::in_Load(LPCSTR section)
{
	m_character_shell_control.Load(section);
	m_flags.set(fl_specific_bonce_demager, TRUE);
	if (pSettings->line_exist(section, "bonce_damage_factor"))
		m_BonceDamageFactor = pSettings->r_float(section, "bonce_damage_factor_for_objects");
	else
		m_BonceDamageFactor = pSettings->r_float("collision_damage", "bonce_damage_factor_for_objects");
	CPHDestroyable::Load(section);
}

void CCharacterPhysicsSupport::run_interactive(CBlend* B)
{
	VERIFY(!m_interactive_animation);
	m_interactive_animation = xr_new<interactive_animation>(&m_EntityAlife, B);
}

void CCharacterPhysicsSupport::update_interactive_anims()
{
	if (Type() != etStalker)
		return;
	VERIFY(m_EntityAlife.cast_stalker( ));
	CAI_Stalker* stalker = m_EntityAlife.cast_stalker();
	CBlend* b = stalker->animation().global().blend();
	if (b && !m_interactive_animation && stalker->animation().global().callback_on_collision())
		run_interactive(b);
	if (m_interactive_animation && !m_interactive_animation->update(m_EntityAlife.XFORM()))
		xr_delete(m_interactive_animation);
}


void CCharacterPhysicsSupport::in_NetSpawn(CSE_Abstract* e)
{
	m_sv_hit = SHit();
	if (m_EntityAlife.use_simplified_visual())
	{
		m_flags.set(fl_death_anim_on, TRUE);
		IKinematics* ka = smart_cast<IKinematics*>(m_EntityAlife.Visual());
		VERIFY(ka);
		ka->CalculateBones_Invalidate();
		ka->CalculateBones(TRUE);
		CollisionCorrectObjPos(m_EntityAlife.Position());
		m_pPhysicsShell = P_build_Shell(&m_EntityAlife, false);
		ka->CalculateBones_Invalidate();
		ka->CalculateBones(TRUE);
		return;
	}
	CPHDestroyable::Init(); //this zerows colbacks !!;
	IRenderVisual* pVisual = m_EntityAlife.Visual();
	IKinematicsAnimated* ka = smart_cast<IKinematicsAnimated*>(pVisual);
	IKinematics* pK = smart_cast<IKinematics*>(pVisual);
	VERIFY(&e->spawn_ini());
	m_death_anims.setup(ka, *e->s_name, pSettings);
	if (!m_EntityAlife.g_Alive())
	{
		if (m_eType == etStalker)
		{
			//pK->LL_GetData( 0 ).shape.flags.set(SBoneShape::sfVisibilityIgnore,TRUE);
			//pK->LL_GetData( pK->LL_BoneID("bip01") ).shape.flags.set(SBoneShape::sfVisibilityIgnore,TRUE);
			ka->PlayCycle("waunded_1_idle_0");
		}
		else
			ka->PlayCycle("death_init");
	}
	else if (!m_EntityAlife.animation_movement_controlled())
		ka->PlayCycle("death_init"); ///непонятно зачем это вообще надо запускать
									  ///этот хак нужен, потому что некоторым монстрам 
									  ///анимация после спона, может быть вообще не назначена
	pK->CalculateBones_Invalidate();
	pK->CalculateBones(TRUE);


	CPHSkeleton::Spawn(e);
	movement()->EnableCharacter();
	movement()->SetPosition(m_EntityAlife.Position());
	movement()->SetVelocity(0, 0, 0);
	if (m_eType != etActor)
	{
		m_flags.set(fl_specific_bonce_demager, TRUE);
		m_BonceDamageFactor = 1.f;
	}
	if (Type() == etStalker)
	{
		m_hit_animations.SetupHitMotions(*smart_cast<IKinematicsAnimated*>(m_EntityAlife.Visual()));
	}
	anim_mov_state.init();

	anim_mov_state.active = m_EntityAlife.animation_movement_controlled();
	CInifile* ini = m_EntityAlife.spawn_ini();
	if (ini && ini->section_exist("physics") && ini->line_exist("physics", "controller_can_be_moved_by_player"))
		m_PhysicMovementControl->SetActorMovable(!! ini->r_bool("physics", "controller_can_be_moved_by_player"));
}

bool CCharacterPhysicsSupport::CollisionCorrectObjPos()
{
	return CollisionCorrectObjPos(m_EntityAlife.Position(), true);
}

void CCharacterPhysicsSupport::CreateCharacterSafe()
{
	if (m_PhysicMovementControl->CharacterExist())return;
	CollisionCorrectObjPos(m_EntityAlife.Position(), true);
	CreateCharacter();
}

void CCharacterPhysicsSupport::CreateCharacter()
{
	m_PhysicMovementControl->CreateCharacter();
	m_PhysicMovementControl->SetPhysicsRefObject(&m_EntityAlife);
	m_PhysicMovementControl->SetPosition(m_EntityAlife.Position());
}

bool HACK_TERRIBLE_DONOT_COLLIDE_ON_SPAWN(CEntityAlive& ea)
{
	if (pSettings->line_exist(ea.cNameSect().c_str(), "hack_terrible_donot_collide_on_spawn") &&
		pSettings->r_bool(ea.cNameSect().c_str(), "hack_terrible_donot_collide_on_spawn")
	)
		return true;
	return false;
}

void CCharacterPhysicsSupport::SpawnInitPhysics(CSE_Abstract* e)
{
	if (m_EntityAlife.g_Alive())
	{
#ifdef DEBUG
		if( ph_dbg_draw_mask1.test(ph_m1_DbgTrackObject)&&stricmp( PH_DBG_ObjectTrackName( ), *m_EntityAlife.cName( ) )==0 )
		{
			Msg("CCharacterPhysicsSupport::SpawnInitPhysics obj %s before collision correction %f,%f,%f", PH_DBG_ObjectTrackName(), m_EntityAlife.Position().x, m_EntityAlife.Position().y, m_EntityAlife.Position( ).z );
		}
#endif
#ifdef	USE_IK
		if (etStalker == m_eType || etActor == m_eType || (m_EntityAlife.Visual()->dcast_PKinematics()->LL_UserData() &&
			m_EntityAlife.Visual()->dcast_PKinematics()->LL_UserData()->section_exist("ik")))
			CreateIKController();
#endif
		VERIFY(pSettings);


		SpawnCharacterCreate();

#ifdef DEBUG
		if( ph_dbg_draw_mask1.test( ph_m1_DbgTrackObject ) && stricmp( PH_DBG_ObjectTrackName( ), *m_EntityAlife.cName()) == 0 )
		{
			Msg( "CCharacterPhysicsSupport::SpawnInitPhysics obj %s after collision correction %f,%f,%f", PH_DBG_ObjectTrackName(),m_EntityAlife.Position( ).x, m_EntityAlife.Position().y, m_EntityAlife.Position().z );
		}
#endif
	}
	else
	{
		ActivateShell(NULL);
	}

	CSE_PHSkeleton* po = smart_cast<CSE_PHSkeleton*>(e);
	VERIFY(po);

	PHNETSTATE_VECTOR& saved_bones = po->saved_bones.bones;
	if (po->_flags.test(CSE_PHSkeleton::flSavedData) &&
		saved_bones.size() != m_EntityAlife.PHGetSyncItemsNumber()
	)
	{
#ifdef DEBUG
		Msg("! saved bones %d , current bones %d, object :%s", saved_bones.size(), m_EntityAlife.PHGetSyncItemsNumber(), m_EntityAlife.cName().c_str() );
#endif
		po->_flags.set(CSE_PHSkeleton::flSavedData, FALSE);
		saved_bones.clear();
	}
}

void CCharacterPhysicsSupport::SpawnCharacterCreate()
{
	if (HACK_TERRIBLE_DONOT_COLLIDE_ON_SPAWN(m_EntityAlife)) //||  m_EntityAlife.animation_movement_controlled( )
		return;
	CreateCharacterSafe();
	//if( m_eType != etStalker )
	//	CreateCharacterSafe();
	//VERIFY( movement() );

	//if( movement()->CharacterExist() )
	//	return;
	//else
	//{
	//	VERIFY( !m_collision_activating_delay );
	//	m_collision_activating_delay = xr_new<activating_character_delay>(this);
	//}
}

void CCharacterPhysicsSupport::destroy_imotion()
{
	destroy(m_interactive_motion);
}

void CCharacterPhysicsSupport::in_NetDestroy()
{
    painter_piece_continuation::forget(m_EntityAlife);
    painter_arm_pose_owner_gone(m_EntityAlife);
	if (painter_alive_reference_target == this) painter_cancel_alive_reference();
	CancelPainterGeometryProbe();
	m_painter_geometry_installed = false;
	if (m_pPhysicsShell) ragdoll_shell_capture.stop(m_pPhysicsShell);
#ifdef PAINTER_ACTIVE_RAGDOLL_NATIVE
	m_active_ragdoll_controller.detach();
#endif
	destroy(m_interactive_motion);
	m_PhysicMovementControl->DestroyCharacter();

	if (m_physics_skeleton)
	{
		m_physics_skeleton->Deactivate();
		xr_delete(m_physics_skeleton);
	}
	if (m_pPhysicsShell)
	{
		m_pPhysicsShell->Deactivate();
		xr_delete(m_pPhysicsShell);
	}

	m_flags.set(fl_skeleton_in_shell, FALSE);
	CPHSkeleton::RespawnInit();
	CPHDestroyable::RespawnInit();
	m_eState = esAlive;

	xr_delete(m_interactive_animation);
	destroy_animation_collision();
	DestroyIKController();
	xr_delete(m_collision_activating_delay);
}

void CCharacterPhysicsSupport::in_NetSave(NET_Packet& P)
{
	CPHSkeleton::SaveNetState(P);
}

void CCharacterPhysicsSupport::in_Init()
{
}

void CCharacterPhysicsSupport::UpdateCollisionActivatingDellay()
{
	VERIFY(m_collision_activating_delay);
	VERIFY(movement());
	m_collision_activating_delay->update();
	if (!m_collision_activating_delay->active())
		xr_delete(m_collision_activating_delay);
}

void CCharacterPhysicsSupport::in_shedule_Update(u32 DT)
{
	PROF_EVENT();
	///VERIFY( 0 );

	//CPHSkeleton::Update(DT);
	if (m_collision_activating_delay)
		UpdateCollisionActivatingDellay();

	if (!m_EntityAlife.use_simplified_visual())
		CPHDestroyable::SheduleUpdate(DT);
	else if (m_pPhysicsShell && m_pPhysicsShell->isFullActive() && !m_pPhysicsShell->isEnabled())
		m_EntityAlife.deactivate_physics_shell();
	movement()->in_shedule_Update(DT);
#if	0
	if( anim_mov_state.active )
	{
		DBG_OpenCashedDraw( );
		DBG_DrawMatrix( mXFORM, 0.5f );
		DBG_ClosedCashedDraw( 5000 );
	}
#endif
}

#ifdef DEBUG
	string64 sdbg_stalker_death_anim = "none";
	LPSTR dbg_stalker_death_anim = sdbg_stalker_death_anim;
#endif
BOOL b_death_anim_velocity = TRUE;
const float cmp_angle = M_PI / 10.f;
const float cmp_ldisp = 0.1f;
IC bool cmp(const Fmatrix& f0, const Fmatrix& f1)
{
	Fmatrix if0;
	if0.invert(f0);
	Fmatrix cm;
	cm.mul_43(if0, f1);

	Fvector ax;
	float ang;
	Fquaternion q;
	q.set(cm);
	q.get_axis_angle(ax, ang);

	return ang < cmp_angle && cm.c.square_magnitude() < cmp_ldisp * cmp_ldisp;
}

bool is_similar(const Fmatrix& m0, const Fmatrix& m1, float param)
{
	Fmatrix tmp1;
	tmp1.invert(m0);
	Fmatrix tmp2;
	tmp2.mul(tmp1, m1);
	Fvector ax;
	float ang;
	Fquaternion q;
	q.set(tmp2);
	q.get_axis_angle(ax, ang);
	return _abs(ang) < M_PI / 2.f;
}

//static struct callback_tracks_disable: public IUpdateTracksCallback
//{
//	virtual	bool	operator () ( float dt, IKinematicsAnimated& k ){return false;}
//} tracks_disable_update;

void CCharacterPhysicsSupport::KillHit(SHit& H)
{
#ifdef	DEBUG
	if( death_anim_debug )
			Msg( "death anim: kill hit  " );
#endif
	VERIFY(m_EntityAlife.Visual( ));
	VERIFY(m_EntityAlife.Visual( )->dcast_PKinematics( ));

	//IKinematicsAnimated * KA = m_EntityAlife.Visual( )->dcast_PKinematicsAnimated	();
	//VERIFY( KA );
	//KA->SetUpdateTracksCalback( &tracks_disable_update );

	m_character_shell_control.TestForWounded(m_EntityAlife.XFORM(), m_EntityAlife.Visual()->dcast_PKinematics());
	Fmatrix prev_pose;
	prev_pose.set(mXFORM);

	Fvector start;
	start.set(m_EntityAlife.Position());
	Fvector velocity;
	Fvector death_position;

	CreateShell(H.who, death_position, velocity);
	//ActivateShell( H.who );

	//	if(Type() == etStalker && xr_strcmp(dbg_stalker_death_anim, "none") != 0)
	// R63 sever hold: decided before the death animation (a held shell takes none: it waits frozen in the hit pose)
	const bool painter_hold = PainterSeverHoldWanted(H);
	float hit_angle = 0;
	MotionID m = painter_hold ? MotionID() : m_death_anims.motion(m_EntityAlife, H, hit_angle);

	CAI_Stalker* const holder = m_EntityAlife.cast_stalker();
	if (holder && (holder->wounded() || holder->movement().current_params().cover()))
		m = MotionID();

	if (m.valid()) //&& cmp( prev_pose, mXFORM ) 
	{
		destroy(m_interactive_motion);
		if (false && b_death_anim_velocity)
			m_interactive_motion = xr_new<imotion_velocity>();
		else
			m_interactive_motion = xr_new<imotion_position>();
		m_interactive_motion->setup(m, m_pPhysicsShell, hit_angle);
	}
	else
		DestroyIKController();
	//KA->SetUpdateTracksCalback( 0 );

	if (is_imotion(m_interactive_motion))
		m_interactive_motion->play();


	m_character_shell_control.set_fatal_impulse(H);

	if (!is_imotion(m_interactive_motion))
	{
#ifdef	DEBUG
		if( death_anim_debug )
		{
			Msg( "death anim: kill hit use free ragdoll " );
			Msg( "death anim: fatal impulse: %f, ", H.impulse );
		}
#endif

		EndActivateFreeShell(H.who, start, death_position, velocity);
		m_flags.set(fl_block_hit, TRUE);
		if (painter_hold && m_pPhysicsShell)
		{
			m_pPhysicsShell->get_LinearVel(m_painter_hold_velocity);
			m_pPhysicsShell->set_LinearVel(Fvector().set(0.f, 0.f, 0.f));
			m_pPhysicsShell->set_AngularVel(Fvector().set(0.f, 0.f, 0.f));
			m_pPhysicsShell->Disable();
			m_painter_hold = true; m_painter_hold_started = Device.dwTimeContinual;
			m_painter_hold_hit = H; m_painter_hold_hit_valid = true;
			Msg("[painter_voxel_live] id=%u phase=sever_hold_started bone=%u hit_type=%u t=%u tc=%u", m_EntityAlife.ID(), unsigned(H.bone()), unsigned(H.type()), Device.dwTimeGlobal, Device.dwTimeContinual);
		}
	}
}

const u32 hit_valide_time = 1000;

void CCharacterPhysicsSupport::in_Hit(SHit& H, bool is_killing)
{
    body_hit_outcomes::set_sink(+[](const char*s)noexcept{Msg("%s",s);});
    body_hit_outcomes::Scope hit_trace(m_EntityAlife.ID(),H.BulletID,H.bone(),m_EntityAlife.g_Alive());
	m_sv_hit = H;
	m_hit_valide_time = Device.dwTimeGlobal + hit_valide_time;
	if (m_EntityAlife.use_simplified_visual() || esRemoved == m_eState)
		{body_hit_outcomes::report("not_queued",m_EntityAlife.use_simplified_visual()?"simplified_visual":"physics_removed");return;}
    // in_Die(true) replays m_sv_hit for shell creation. Only PHHit receipt
    // owns a damage event; never carve a second time for the fatal replay.
    if (!is_killing) CapturePainterVoxelHit(H);
    else body_hit_outcomes::report("not_queued","death_shell_replay_not_new_hit");
#ifdef PAINTER_ACTIVE_RAGDOLL_NATIVE
    m_active_ragdoll_controller.hit(H);
#endif
	if (m_flags.test(fl_block_hit))
	{
		VERIFY2(!m_EntityAlife.g_Alive( ),
		        make_string("entity [%s][%d] is dead", m_EntityAlife.Name(), m_EntityAlife.ID()).c_str());
		if (Device.dwTimeGlobal - m_EntityAlife.GetLevelDeathTime() >= 2000)
			m_flags.set(fl_block_hit,FALSE);
		else return;
	}

	//is_killing = is_killing || ( m_eState==esAlive && !m_EntityAlife.g_Alive( ) );
	if (m_EntityAlife.g_Alive() && is_killing && H.type() == ALife::eHitTypeExplosion && H.damage() > 70.f)
		CPHDestroyable::Destroy();

	if ((!m_EntityAlife.g_Alive() || is_killing))
		m_character_shell_control.set_kill_hit(H);

	if (!m_pPhysicsShell && is_killing)
		KillHit(H);

	if (m_flags.test(fl_use_hit_anims) && Type() != etBitting && !m_flags.test(fl_death_anim_on))
		//&& Type() == etStalker
	{
		m_hit_animations.PlayHitMotion(H.direction(), H.bone_space_position(), H.bone(), m_EntityAlife);
	}

	if (!(m_pPhysicsShell && m_pPhysicsShell->isActive()))
	{
		if (!is_killing && m_EntityAlife.g_Alive())
			m_PhysicMovementControl->ApplyHit(H.direction(), H.phys_impulse(), H.type());
	}
	else
	{
#ifdef	DEBUG
		if( is_killing && death_anim_debug &&  !is_imotion( m_interactive_motion ) )
		{
				Msg( "death anim: applied fatal impulse dir: (%f,%f,%f), value: (%f) ", H.dir.x,H.dir.y,H.dir.z, H.impulse );
		}
#endif
		if (!m_painter_hold) // R63: a held shell keeps the fatal impulse for PainterHoldRelease
			m_pPhysicsShell->applyHit(H.bone_space_position(), H.direction(), H.phys_impulse(), H.bone(), H.type());
	}
}


IC void CCharacterPhysicsSupport::UpdateDeathAnims()
{
	VERIFY(m_pPhysicsShell->isFullActive());

	if (!m_flags.test(fl_death_anim_on) && !is_imotion(m_interactive_motion))
		//!m_flags.test(fl_use_death_motion)//!b_death_anim_on&&m_pPhysicsShell->isFullActive()
	{
		DestroyIKController();
		smart_cast<IKinematicsAnimated*>(m_EntityAlife.Visual())->PlayCycle("death_init");
		m_flags.set(fl_death_anim_on,TRUE);
	}
}
#ifdef DEBUG
void	DBG_PhysBones( CObject &O );
void	DBG_DrawBones( CObject &O );
void	DBG_DrawBind ( CObject &O );
BOOL dbg_draw_character_bones			=false;
BOOL dbg_draw_character_physics			=false;
BOOL dbg_draw_character_binds			=false;
BOOL dbg_draw_character_physics_pones	=false;

void dbg_draw_geoms(xr_vector<CODEGeom*>& m_weapon_geoms)
{
	xr_vector<CODEGeom*>::iterator ii =m_weapon_geoms.begin(), ee = m_weapon_geoms.end();
	for( ;ii!=ee; ++ii )
	{
		CODEGeom *g  =(*ii);

		g->dbg_draw( 0.01f, D3DCOLOR_XRGB( 0, 255, 100 ), Flags32() );
	}
}
#endif


void CCharacterPhysicsSupport::in_UpdateCL()
{
	if (m_eState == esRemoved)
	{
		return;
	}
    QueuePainterVoxelLive(); // Poll next owner barrier; never wait for worker here.
#ifdef DEBUG
	if( dbg_draw_character_bones )
				dbg_draw_geoms( m_weapon_geoms );

	if( dbg_draw_character_bones )
				DBG_DrawBones( m_EntityAlife );

	if( dbg_draw_character_binds )
				DBG_DrawBind( m_EntityAlife );

	if( dbg_draw_character_physics_pones )
				DBG_PhysBones( m_EntityAlife );

	if( dbg_draw_character_physics && m_pPhysicsShell )
				m_pPhysicsShell->dbg_draw_geometry( 0.2f, D3DCOLOR_ARGB( 100 ,255, 0, 0 ) );
#endif
	if (m_painter_hold && Device.dwTimeContinual - m_painter_hold_started > u32(painter_sever_hold_ms))
		PainterHoldRelease("timeout"); // R63: never an indefinitely suspended corpse
	update_animation_collision();
	m_character_shell_control.CalculateTimeDelta();
	if (m_pPhysicsShell)
	{
		VERIFY(m_pPhysicsShell->isFullActive( ));
		m_pPhysicsShell->SetRagDoll(); //Теперь шела относиться к классу объектов cbClassRagDoll

		if (!is_imotion(m_interactive_motion)) //!m_flags.test(fl_use_death_motion)
			m_pPhysicsShell->InterpolateGlobalTransform(&mXFORM);
		else
			m_interactive_motion->update();

		UpdateDeathAnims();

		m_character_shell_control.UpdateFrictionAndJointResistanse(m_pPhysicsShell);
	}
		//else if ( !m_EntityAlife.g_Alive( ) && !m_EntityAlife.use_simplified_visual( ) )
		//{
		//ActivateShell( NULL );
		//m_PhysicMovementControl->DestroyCharacter( );
		//} 
	else if (ik_controller())
	{
		CFrustum& view_frust = ::Render->ViewBase;
		vis_data& vis = m_EntityAlife.Visual()->getVisData();
		Fvector p;

		m_EntityAlife.XFORM().transform_tiny(p, vis.sphere.P);

		// demonized: use screen space area to check if need update
		float perceived_dist = Device.GetPerceivedDist(p);
		float ssa = Device.CalcSSADynamic(m_EntityAlife.SpatialComponent->spatial.sphere.P, m_EntityAlife.SpatialComponent->spatial.sphere.R);
		if (ssa > IK_CALC_SSA)
		{
			if (view_frust.testSphere_dirty(m_EntityAlife.SpatialComponent->spatial.sphere.P, m_EntityAlife.SpatialComponent->spatial.sphere.R) || perceived_dist < IK_ALWAYS_CALC_DIST)
			{
				update_interactive_anims();
				ik_controller()->Update();
			}
		}
	}

#ifdef DEBUG
	if(Type()==etStalker && ph_dbg_draw_mask1.test(phDbgHitAnims))
	{
		Fmatrix m;
		m_hit_animations.GetBaseMatrix(m,m_EntityAlife);
		DBG_DrawMatrix(m,1.5f);
/*
		IKinematicsAnimated	*K = smart_cast<IKinematicsAnimated*>(m_EntityAlife.Visual());
		u16 hb = K->LL_BoneID("bip01_head");
		u16 pb = K->LL_GetBoneRoot();
		u16 nb = K->LL_BoneID("bip01_neck");
		u16 eb = K->LL_BoneID("eye_right");
		Fmatrix &mh  = K->LL_GetTransform(hb);
		Fmatrix &mp  = K->LL_GetTransform(pb);
		Fmatrix &me	 = K->LL_GetTransform(eb);
		Fmatrix &mn	 = K->LL_GetTransform(nb);
		float d = DET(mh);
		if(Fvector().sub(mh.c,mp.c).magnitude() < 0.3f||d<0.7 )//|| Fvector().sub(me.c,mn.c) < 0.5
		{
			
			K->CalculateBones_Invalidate();
			K->CalculateBones();
			;
		}
*/
	}
#endif
}

void CCharacterPhysicsSupport::UpdatePainterAliveReference()
{
	if (painter_alive_reference_target != this) {
		m_painter_alive_reference_frames = 0;
		m_painter_alive_reference_has_previous = false;
		return;
	}
	if (Device.dwTimeGlobal - painter_alive_reference_armed_at > 10000u ||
		!g_pGameLevel || painter_alive_reference_target_id != m_EntityAlife.ID() ||
		Level().Objects.net_Find(painter_alive_reference_target_id) != &m_EntityAlife) {
		Msg("! painter alive reference expired or identity changed: entity=%u",unsigned(m_EntityAlife.ID()));
		painter_cancel_alive_reference();
		m_painter_alive_reference_frames = 0;
		m_painter_alive_reference_has_previous = false;
		return;
	}
	if (m_eType != etStalker || m_eState != esAlive || !m_EntityAlife.g_Alive() ||
		m_EntityAlife.getDestroy() || m_pPhysicsShell || m_physics_skeleton ||
		!movement() || !movement()->CharacterExist() ||
		m_EntityAlife.animation_movement_controlled() || !physics_world() ||
		physics_world()->Processing() || Device.Paused()) {
		m_painter_alive_reference_frames = 0;
		m_painter_alive_reference_has_previous = false;
		return;
	}
	IKinematics* k = smart_cast<IKinematics*>(m_EntityAlife.Visual());
	if (!k) return;
	// The render animation is sampled only for this specifically armed NPC.
	k->CalculateBones(TRUE);
	const Fmatrix owner_xform = m_EntityAlife.XFORM();
	u16 root_id = u16(-1);
	Fvector root_world;
	{
		xrCriticalSectionGuard guard(&k->UCalc_Mutex);
		root_id = k->LL_GetBoneRoot();
		owner_xform.transform_tiny(root_world,k->LL_GetTransform(root_id).c);
	}
	Fvector movement_velocity;
	movement()->GetCharacterVelocity(movement_velocity);
	const float speed = movement_velocity.magnitude();
	float root_delta = 0.f;
	if (m_painter_alive_reference_has_previous) {
		Fvector displacement;
		displacement.sub(root_world,m_painter_alive_reference_previous_root);
		root_delta = displacement.magnitude();
	}
	m_painter_alive_reference_previous_root.set(root_world);
	const bool had_previous = m_painter_alive_reference_has_previous;
	m_painter_alive_reference_has_previous = true;
	if (!std::isfinite(double(speed)) || !std::isfinite(double(root_delta)) ||
		!_valid(owner_xform) || !had_previous ||
		speed >= 0.03f || root_delta >= 0.003f) {
		m_painter_alive_reference_frames = 0;
		return;
	}
	if (++m_painter_alive_reference_frames < 60) return;
	const u16 physics_root = k->LL_BoneID("bip01_pelvis");
	if (physics_root == BI_NONE) {
		Msg("! painter alive reference rejected: physics pelvis bone missing");
		painter_cancel_alive_reference();
		return;
	}
	struct callback_snapshot {
		BoneCallback callback; void* param; BOOL overwrite; u32 type;
	};
	xr_vector<callback_snapshot> callbacks(k->LL_BoneCount());
	{
		xrCriticalSectionGuard guard(&k->UCalc_Mutex);
		for (u16 i=0;i<k->LL_BoneCount();++i) {
			CBoneInstance& bone=k->LL_GetBoneInstance(i);
			callbacks[i]={bone.callback(),bone.callback_param(),bone.callback_overwrite(),bone.callback_type()};
		}
	}

	// Build uses native model mass, collision shapes, inertia smoothing, limits,
	// and bind-created ODE joint references. The ghost never enters RunSimulation,
	// receives an owner/callback, or replaces the living movement character.
	CPhysicsShell* ghost = P_create_Shell();
	// CreateShell temporarily makes the pelvis the kinematics root before it
	// builds the death shell. Keep that shared change inside the bone mutex and
	// restore it before any other thread can read the living animation root.
	{
		xrCriticalSectionGuard guard(&k->UCalc_Mutex);
		struct restore_bone_root {
			IKinematics* kinematics; u16 original;
			restore_bone_root(IKinematics* k, u16 root, u16 physics) : kinematics(k),original(root) {
				kinematics->LL_SetBoneRoot(physics);
			}
			~restore_bone_root() { kinematics->LL_SetBoneRoot(original); }
		} restore(k,root_id,physics_root);
		ghost->preBuild_FromKinematics(k);
	}
	// PHShell::Build only builds already-selected elements and ODE joints; it
	// does not read the kinematics root. Do that ODE work after releasing the lock.
	ghost->mXFORM.set(owner_xform);
	ghost->SmoothElementsInertia(0.3f);
	SAllDDOParams disable_params;
	disable_params.Load(k->LL_UserData());
	ghost->set_DisableParams(disable_params);
	ghost->Build();
	auto callbacks_same = [&]() -> bool {
		xrCriticalSectionGuard guard(&k->UCalc_Mutex);
		for (u16 i=0;i<k->LL_BoneCount();++i) {
			CBoneInstance& bone=k->LL_GetBoneInstance(i);
			if (bone.callback()!=callbacks[i].callback || bone.callback_param()!=callbacks[i].param ||
				bone.callback_overwrite()!=callbacks[i].overwrite || bone.callback_type()!=callbacks[i].type)
				return false;
		}
		return true;
	};
	const bool callbacks_unchanged = callbacks_same();
	const bool root_restored = k->LL_GetBoneRoot()==root_id;
	const bool physics_root_built = ghost->get_ElementsNumber() &&
		static_cast<CPHElement*>(ghost->get_ElementByStoreOrder(0))->m_SelfID==physics_root;
	if (!callbacks_unchanged || !root_restored || !physics_root_built) {
		Msg("! painter alive reference rejected: root/callback readback root=%d callbacks=%d physics_root=%d",
			root_restored ? 1 : 0,callbacks_unchanged ? 1 : 0,physics_root_built ? 1 : 0);
		destroy_physics_shell(ghost);
		painter_cancel_alive_reference();
		return;
	}
	xr_vector<Fmatrix> animated_element_poses(ghost->get_ElementsNumber());
	{
		// CalculateBones has the same mutex. Copy a coherent pose and release
		// it before touching ODE or dispatching any shell destructor callback.
		xrCriticalSectionGuard guard(&k->UCalc_Mutex);
		for (u16 i=0;i<ghost->get_ElementsNumber();++i) {
			CPHElement* e = static_cast<CPHElement*>(ghost->get_ElementByStoreOrder(i));
			animated_element_poses[i].set(k->LL_GetTransform(e->m_SelfID));
		}
	}
	xr_vector<Fmatrix> world_element_poses(ghost->get_ElementsNumber());
	bool valid_poses = true;
	for (u16 i=0;i<ghost->get_ElementsNumber();++i) {
		world_element_poses[i].mul_43(owner_xform,animated_element_poses[i]);
		valid_poses = valid_poses && _valid(world_element_poses[i]) &&
			valid_pos(world_element_poses[i].c);
	}
	if (!valid_poses) {
		Msg("! painter alive reference rejected: nonfinite or out-of-bounds animated pose");
		destroy_physics_shell(ghost);
		painter_cancel_alive_reference();
		m_painter_alive_reference_frames = 0;
		m_painter_alive_reference_has_previous = false;
		return;
	}
	for (u16 i=0;i<ghost->get_ElementsNumber();++i) {
		CPHElement* e = static_cast<CPHElement*>(ghost->get_ElementByStoreOrder(i));
		// This is BoneGlPos + SetTransform, exactly as ToBonePos uses it.
		// Skip ToBonePos's debug-only PhysicsRefObject dereference: the ghost
		// deliberately has no owner and interpolation is irrelevant to a snapshot.
		e->SetTransform(world_element_poses[i],mh_clear);
	}
	// DWorld() is a cast of the island object even when empty; inspect membership.
	bool unattached = !ghost->PKinematics() && ghost->PIsland()->nb == 0 &&
		ghost->PIsland()->nj == 0 && !ghost->PIsland()->firstbody &&
		!ghost->PIsland()->firstjoint && !static_cast<CPHShell*>(ghost)->SpatialComponent->spatial.node_ptr;
	for (u16 i=0;i<ghost->get_ElementsNumber();++i)
		unattached = unattached &&
			!static_cast<dxBody*>(static_cast<CPHElement*>(ghost->get_ElementByStoreOrder(i))->get_body())->world;
	for (u16 i=0;i<ghost->get_JointsNumber();++i) {
		CPHJoint* j = static_cast<CPHJoint*>(ghost->get_JointByStoreOrder(i));
		unattached = unattached && j->GetDJoint() && !static_cast<dxJoint*>(j->GetDJoint())->world &&
			(!j->GetDJoint1() || !static_cast<dxJoint*>(j->GetDJoint1())->world);
	}
	const bool original_unchanged = !m_pPhysicsShell && !m_physics_skeleton &&
		movement()->CharacterExist() && m_EntityAlife.g_Alive();
	const bool written = unattached && original_unchanged &&
		ragdoll_shell_capture.write_alive_reference(ghost,m_EntityAlife,k,root_id,
			m_painter_alive_reference_frames,speed,root_world);
	destroy_physics_shell(ghost);
	const bool post_unchanged = m_EntityAlife.g_Alive() && !m_pPhysicsShell &&
		!m_physics_skeleton && movement() && movement()->CharacterExist() &&
		smart_cast<IKinematics*>(m_EntityAlife.Visual()) == k && g_pGameLevel &&
		Level().Objects.net_Find(m_EntityAlife.ID()) == &m_EntityAlife &&
		k->LL_GetBoneRoot()==root_id && callbacks_same();
	if (written && post_unchanged) {
		Msg("* painter alive reference marker: entity=%u frame=%u stable_frames=%u speed=%g root_delta=%g post_readback=alive:1,shell:0,movement:1,visual:1,identity:1,root:1,callbacks:1",
			unsigned(m_EntityAlife.ID()),Device.dwFrame,m_painter_alive_reference_frames,speed,root_delta);
	} else
		Msg("! painter alive reference rejected: entity=%u unattached=%d original_unchanged=%d post_unchanged=%d",
			unsigned(m_EntityAlife.ID()),unattached ? 1 : 0,original_unchanged ? 1 : 0,post_unchanged ? 1 : 0);
	// A failed build is not retried every idle interval without a new command.
	painter_cancel_alive_reference();
	m_painter_alive_reference_frames = 0;
	m_painter_alive_reference_has_previous = false;
}

void CCharacterPhysicsSupport::CreateSkeleton(CPhysicsShell* & pShell)
{
	R_ASSERT2(!pShell, "pShell already initialized!!");
	if (!m_EntityAlife.Visual())
		return;
#ifdef DEBUG
	CTimer t;t.Start();
#endif
	pShell = P_create_Shell();

	IKinematics* k = smart_cast<IKinematics*>(m_EntityAlife.Visual());

	phys_shell_verify_object_model(m_EntityAlife);
	BONE_P_MAP proxy_owners;for(u16 bone=0;bone<k->LL_BoneCount();++bone)proxy_owners.emplace(bone,physicsBone{});
	pShell->preBuild_FromKinematics(k,&proxy_owners);
    if(m_eType==etStalker)painter_skin_proxy::append(*pShell,*k,proxy_owners);

	pShell->mXFORM.set(mXFORM);

	pShell->SmoothElementsInertia(0.3f);
	pShell->set_PhysicsRefObject(&m_EntityAlife);
	SAllDDOParams disable_params;
	disable_params.Load(smart_cast<IKinematics*>(m_EntityAlife.Visual())->LL_UserData());
	pShell->set_DisableParams(disable_params);

	pShell->Build();


#ifdef DEBUG
	Msg("shell for %s[%d] created in %f ms",*m_EntityAlife.cName(),m_EntityAlife.ID(),t.GetElapsed_sec()*1000.f);
#endif
}

bool CCharacterPhysicsSupport::DoCharacterShellCollide()
{
	if (m_eType == etStalker)
	{
		CAI_Stalker* OBJ = smart_cast<CAI_Stalker*>(&m_EntityAlife);
		VERIFY(OBJ);
		return !OBJ->wounded();
	}
	return true;
}


bool CCharacterPhysicsSupport::CollisionCorrectObjPos(const Fvector& start_from, bool character_create/*=false*/)
{
	return CollisionCorrectObjPosImpl(start_from, character_create, false);
}

bool CCharacterPhysicsSupport::CollisionCorrectObjPosStaticOnly(const Fvector& start_from)
{
	return CollisionCorrectObjPosImpl(start_from, false, true);
}

bool CCharacterPhysicsSupport::CollisionCorrectObjPosImpl(const Fvector& start_from, bool character_create, bool static_only)
{
	//Fvector shift;shift.sub( start_from, m_EntityAlife.Position() );
	Fvector shift;
	shift.set(0, 0, 0);
	Fbox box;
	if (character_create)
		box.set(movement()->Box());
	else
	{
		if (m_pPhysicsShell)
		{
			VERIFY(m_pPhysicsShell->isFullActive());
			Fvector sz, c;
			get_box(m_pPhysicsShell, mXFORM, sz, c);
			box.setb(Fvector().sub(c, m_EntityAlife.Position()), Fvector(sz).mul(0.5f));
			m_pPhysicsShell->DisableCollision();
		}
		else
			box.set(m_EntityAlife.BoundingBox());
	}

	Fvector vbox;
	Fvector activation_pos;
	box.get_CD(activation_pos, vbox);
	shift.add(activation_pos);
	vbox.mul(2.f);
	activation_pos.add(shift, m_EntityAlife.Position());
	bool not_collide_characters = !DoCharacterShellCollide() && !character_create;
	bool set_rotation = !character_create;

	Fvector activation_res = Fvector().set(0, 0, 0);
	////////////////

	bool ret = static_only
		? ActivateShapeCharacterPhysicsSupportStaticOnly(activation_res, vbox, activation_pos, mXFORM,
		                                                not_collide_characters, set_rotation, &m_EntityAlife)
		: ActivateShapeCharacterPhysicsSupport(activation_res, vbox, activation_pos, mXFORM,
	                                                not_collide_characters, set_rotation, &m_EntityAlife);
	//////////////////


	m_EntityAlife.Position().sub(activation_res, shift);


	if (m_pPhysicsShell)
		m_pPhysicsShell->EnableCollision();
	return ret;
}

void CCharacterPhysicsSupport::set_movement_position(const Fvector& pos)
{
	VERIFY(movement());

	CollisionCorrectObjPos(pos, true);

	movement()->SetPosition(m_EntityAlife.Position());
}

void CCharacterPhysicsSupport::ForceTransform(const Fmatrix& m)
{
	if (!m_EntityAlife.g_Alive())
		return;
	VERIFY(_valid(m));
	m_EntityAlife.XFORM().set(m);
	if (movement()->CharacterExist())
		movement()->EnableCharacter();
	set_movement_position(m.c);
	movement()->SetVelocity(0, 0, 0);
}

/*
void reset_root_bone_start_pose( CPhysicsShell& shell )
{
	VERIFY( &shell );
	CPhysicsElement * physics_root_element = shell.get_ElementByStoreOrder( 0 );
	VERIFY( physics_root_element );

	IKinematics * K = shell.PKinematics();
	VERIFY( K );
	
	u16	animation_root_bone_id = K->LL_GetBoneRoot();

	CODEGeom	*physics_root_bone_geom = physics_root_element->geometry( 0 );
	VERIFY( physics_root_bone_geom );

	u16 physics_root_bone_id = physics_root_bone_geom->bone_id();
	VERIFY( physics_root_bone_id != BI_NONE );

	if( animation_root_bone_id == physics_root_bone_id )
		return ;

	//u16 anim_bones_number = K->LL_BoneCount();

	//buffer_vector<u32>	anim_bones_bind_positions( _alloca(anim_bones_number*sizeof(u32)),
	//												anim_bones_number
	//											);
#pragma todo("LL_GetBindTransform shoud use buffer_vector")

	xr_vector<Fmatrix> anim_bones_bind_positions;
	K->LL_GetBindTransform( anim_bones_bind_positions );


	const Fmatrix physics_root_to_anim_root_bind_transformation 
		= Fmatrix().mul_43( Fmatrix().invert( anim_bones_bind_positions[ physics_root_bone_id ] ), 
											  anim_bones_bind_positions[ animation_root_bone_id ] );

	const Fmatrix &physics_root_bone_anim_transform = K->LL_GetTransform( physics_root_bone_id );

	const Fmatrix physics_root_bone_corrected_pos = Fmatrix().mul_43( physics_root_bone_anim_transform ,
																	  physics_root_to_anim_root_bind_transformation
																	  );

	physics_root_element->SetTransform( Fmatrix().mul_43( shell.mXFORM, physics_root_bone_corrected_pos ) );

	//physics_root_element->TransformPosition( Fmatrix().mul_43( Fmatrix().invert( K->LL_GetTransform( animation_root_bone_id ) ), physics_root_bone_corrected_pos ) );
}
*/
static const u32 physics_shell_animated_destroy_delay = 3000;

void CCharacterPhysicsSupport::destroy_animation_collision()
{
	xr_delete(m_physics_shell_animated);
	m_physics_shell_animated_time_destroy = u32(-1);
}

void CCharacterPhysicsSupport::create_animation_collision()
{
	m_physics_shell_animated_time_destroy = Device.dwTimeGlobal + physics_shell_animated_destroy_delay;
	if (m_physics_shell_animated)
		return;
	m_physics_shell_animated = xr_new<physics_shell_animated>(&m_EntityAlife, true);
}

void CCharacterPhysicsSupport::update_animation_collision()
{
	if (animation_collision())
	{
		animation_collision()->update(mXFORM);
		//animation_collision( )->shell()->set_LinearVel( movement()->GetVelocity() );
		if (Device.dwTimeGlobal > m_physics_shell_animated_time_destroy)
			destroy_animation_collision();
	}
}
#ifdef	DEBUG
BOOL dbg_draw_ragdoll_spawn = FALSE;
#endif
void CCharacterPhysicsSupport::ActivateShell(CObject* who)
{
    if (!_valid(m_EntityAlife.Position()))
    {
        Debug.fatal(DEBUG_INFO, "CCharacterPhysicsSupport::ActivateShell Fatal error, invalid position for m_EntityAlife %s, id %d, x %.5f, y %.5f, z %.5f", m_EntityAlife.cNameSect().c_str(), m_EntityAlife.ID(), m_EntityAlife.Position().x, m_EntityAlife.Position().y, m_EntityAlife.Position().z);
    }
	Fvector start;
	start.set(m_EntityAlife.Position());
	Fvector velocity;
	Fvector death_position;
	CreateShell(who, death_position, velocity);
	EndActivateFreeShell(who, start, death_position, velocity);
	VERIFY(m_pPhysicsShell);
	m_pPhysicsShell->Enable();
	m_pPhysicsShell->set_LinearVel(Fvector().set(0, -1, 0));
}

//void	CCharacterPhysicsSupport::	on_active_weapon_shell_activate()
//{
//	if( !m_weapon_geoms.empty() )
//		RemoveActiveWeaponCollision		();
//}
bool CCharacterPhysicsSupport::has_shell_collision_place(const CPhysicsShellHolder* obj) const
{
	return m_active_item_obj && obj == m_active_item_obj;
}

void CCharacterPhysicsSupport::on_child_shell_activate(CPhysicsShellHolder* obj)
{
	if (!has_shell_collision_place(obj))
		return;

	VERIFY(obj->PPhysicsShell());
#if	0
	//	DBG_OpenCashedDraw();
	//m_pPhysicsShell->dbg_draw_geometry( 0.2f, D3DCOLOR_XRGB( 255, 100, 0 ) );
	m_pPhysicsShell->dbg_draw_velocity( 0.01f, D3DCOLOR_XRGB( 100, 255, 0 ) );
	m_pPhysicsShell->dbg_draw_force( 0.1f, D3DCOLOR_XRGB( 100, 0, 255 ) );
	DBG_ClosedCashedDraw( 50000 );
#endif
	//DBG_OpenCashedDraw();
	//obj->PPhysicsShell()->dbg_draw_geometry( 0.2f, D3DCOLOR_XRGB( 255, 100, 0 ) );

	RemoveActiveWeaponCollision();


	//DBG_ClosedCashedDraw( 50000 );
}

void CCharacterPhysicsSupport::RemoveActiveWeaponCollision()
{
	VERIFY(m_pPhysicsShell);
	VERIFY(m_weapon_attach_bone);
	VERIFY(!m_weapon_geoms.empty());
	xr_vector<CODEGeom*>::iterator ii = m_weapon_geoms.begin(), ee = m_weapon_geoms.end();
	Fmatrix m0;
	(*ii)->get_xform(m0);
	CPhysicsElement* root = m_active_item_obj->PPhysicsShell()->get_ElementByStoreOrder(0);
	CODEGeom* rg = root->geometry(0);
	VERIFY(rg);
	Fmatrix m1;
	rg->get_xform(m1);

	Fmatrix me;
	root->GetGlobalTransformDynamic(&me);

	Fmatrix m1_to_e = Fmatrix().mul_43(Fmatrix().invert(m1), me);

	Fmatrix m0e = Fmatrix().mul_43(m0, m1_to_e);
	root->SetTransform(m0e, mh_unspecified);

	for (; ii != ee; ++ii)
	{
		CODEGeom* g = (*ii);

		//g->dbg_draw( 0.01f, D3DCOLOR_XRGB( 0, 0, 255 ), Flags32() );

		m_weapon_attach_bone->remove_geom(g);
		g->destroy();
		xr_delete(g);
	}


	//m_active_item_obj->PPhysicsShell()->dbg_draw_geometry( 0.2f, D3DCOLOR_XRGB( 255, 0, 100 ) );

	Fvector a_vel, l_vel;
	const Fvector& mc = root->mass_Center();
	//dBodyGetPointVel( m_weapon_attach_bone->get_body(),mc.x, mc.y, mc.z, cast_fp(l_vel) );
	m_weapon_attach_bone->GetPointVel(l_vel, mc);
	m_weapon_attach_bone->get_AngularVel(a_vel);

	root->set_AngularVel(a_vel);
	root->set_LinearVel(l_vel);

	m_weapon_geoms.clear();
	m_weapon_attach_bone = 0;
	m_active_item_obj = 0;

	bone_fix_clear();
}

void CCharacterPhysicsSupport::bone_fix_clear()
{
	xr_vector<anim_bone_fix*>::iterator i = m_weapon_bone_fixes.begin(), e = m_weapon_bone_fixes.end();
	for (; i != e; ++i)
	{
		(*i)->deinit();
		xr_delete(*i);
	}
	m_weapon_bone_fixes.clear();
}


void CCharacterPhysicsSupport::bone_chain_disable(u16 bone, u16 r_bone, IKinematics& K)
{
	VERIFY(&K);
	u16 bid = bone;
	//K.LL_GetBoneInstance( bid ).set_callback( bctCustom, 0, 0, TRUE );

	while (bid != r_bone && bid != K.LL_GetBoneRoot())
	{
		CBoneData& bd = K.LL_GetData(bid);
		if (K.LL_GetBoneInstance(bid).callback() != anim_bone_fix::callback)
		{
			m_weapon_bone_fixes.push_back(xr_new<anim_bone_fix>());
			m_weapon_bone_fixes.back()->fix(bid, K);
		}
		bid = bd.GetParentID();


		//K.LL_GetBoneInstance( bid ).set_callback( bctCustom, 0, 0, TRUE );
	}
}


void CCharacterPhysicsSupport::AddActiveWeaponCollision()
{
	if (m_eType != etStalker)
		return;
	VERIFY(!m_weapon_attach_bone);
	VERIFY(!m_active_item_obj);
	VERIFY(m_weapon_geoms.empty());
	VERIFY(m_weapon_bone_fixes.empty());

	CInventoryOwner* inv_owner = smart_cast<CInventoryOwner*>(&m_EntityAlife);
	VERIFY(inv_owner);
	PIItem active_weapon_item = inv_owner->inventory().ActiveItem();
	if (!active_weapon_item)
		return;
	int bl = -1, br = -1, br2 = -1;
	m_EntityAlife.g_WeaponBones(bl, br, br2);
	if (br == -1)
		return;
	IKinematics* grip_kinematics = smart_cast<IKinematics*>(m_EntityAlife.Visual());
	if (!grip_kinematics ||
		!painter_weapon_grip_chain_visible(*grip_kinematics, br) ||
		(br2 != -1 && !painter_weapon_grip_chain_visible(*grip_kinematics, br2)) ||
		(bl != -1 && !painter_weapon_grip_chain_visible(*grip_kinematics, bl)))
	{
		Msg("* active weapon collision skipped: grip bone hidden during death handover");
		return;
	}

	active_weapon_item->UpdateXForm();

	CPhysicsShell* weapon_shell = P_build_Shell(&active_weapon_item->object(), true, (BONE_P_MAP*)(0), true);

	VERIFY(m_pPhysicsShell);
	CPhysicsElement* weapon_attach_bone = m_pPhysicsShell->get_PhysicsParrentElement((u16)br);

	bone_chain_disable((u16)br, weapon_attach_bone->m_SelfID, *m_pPhysicsShell->PKinematics());
	if (bl != br && bl != -1)
	{
		CPhysicsElement* p = m_pPhysicsShell->get_PhysicsParrentElement((u16)bl);
		VERIFY(p);
		bone_chain_disable((u16)bl, p->m_SelfID, *m_pPhysicsShell->PKinematics());
	}
	if (br2 != bl && br2 != br && br2 != -1)
	{
		CPhysicsElement* p = m_pPhysicsShell->get_PhysicsParrentElement((u16)br2);
		VERIFY(p);
		bone_chain_disable((u16)br2, weapon_attach_bone->m_SelfID, *m_pPhysicsShell->PKinematics());
	}

	CPhysicsElement* weapon_element = weapon_shell->get_ElementByStoreOrder(0);

	u16 geom_num = weapon_element->numberOfGeoms();
	for (u16 i = 0; i < geom_num; ++i)
		m_weapon_geoms.push_back(weapon_element->geometry(i));
	xr_vector<CODEGeom*>::iterator ii = m_weapon_geoms.begin(), ee = m_weapon_geoms.end();


	//DBG_OpenCashedDraw();

	for (; ii != ee; ++ii)
	{
		CODEGeom* g = (*ii);
		//g->dbg_draw( 0.01f, D3DCOLOR_XRGB( 255, 0, 0 ), Flags32() );
		weapon_element->remove_geom(g);
		g->set_bone_id(weapon_attach_bone->m_SelfID);
		weapon_attach_bone->add_geom(g);
		//g->dbg_draw( 0.01f, D3DCOLOR_XRGB( 0, 255, 0 ), Flags32() );
	}
	m_weapon_attach_bone = weapon_attach_bone;
	m_active_item_obj = &(active_weapon_item->object());


	destroy_physics_shell(weapon_shell);

	//m_pPhysicsShell->dbg_draw_geometry( 1, D3DCOLOR_XRGB( 0, 0, 255 ) );
	//DBG_ClosedCashedDraw( 50000 );
}

// Painter: a dead stalker's forearms and hands collide with its own torso (pelvis up to the collarbones), so a
// hand clutching a chest or belly wound stays on the body instead of passing through it. Set up before the
// weapon joins the right hand, so the rifle is not treated as part of the hand.
// [painter_ragdoll] limb_torso_collision = false turns it off.
static void painter_limb_torso_collision(CPhysicsShell& shell, IKinematics& K)
{
	static const bool on = !pSettings || !pSettings->section_exist("painter_ragdoll") ||
		!pSettings->line_exist("painter_ragdoll", "limb_torso_collision") ||
		pSettings->r_bool("painter_ragdoll", "limb_torso_collision");
	if (!on)
		return;
	static const char* const limb_names[] = {"bip01_l_forearm", "bip01_r_forearm", "bip01_l_hand", "bip01_r_hand"};
	static const char* const torso_names[] = {"bip01_pelvis", "bip01_spine", "bip01_spine1", "bip01_spine2",
	                                          "bip01_l_clavicle", "bip01_r_clavicle"};
	u16 limbs[4], torso[6];
	for (int i = 0; i < 4; ++i)
		limbs[i] = K.LL_BoneID(limb_names[i]);
	for (int i = 0; i < 6; ++i)
		torso[i] = K.LL_BoneID(torso_names[i]);
	const u16 pairs = shell.SetLimbTorsoCollision(limbs, 4, torso, 6);
	if (painter_log_verbose())
		Msg("[painter_ragdoll] limb_torso_collision pairs=%u", unsigned(pairs));
}

void CCharacterPhysicsSupport::CreateShell(CObject* who, Fvector& dp, Fvector& velocity)
{
	xr_delete(m_collision_activating_delay);
	xr_delete(m_interactive_animation);
	destroy_animation_collision();
	//DestroyIKController( );
	IKinematics* K = smart_cast<IKinematics*>(m_EntityAlife.Visual());
	//animation movement controller issues
	bool anim_mov_ctrl = m_EntityAlife.animation_movement_controlled();
	CBoneInstance& BR = K->LL_GetBoneInstance(K->LL_GetBoneRoot());
	Fmatrix start_xform;
	start_xform.identity();
	CBlend* anim_mov_blend = 0;
	if (anim_mov_ctrl)
	{
		m_EntityAlife.animation_movement()->ObjStartXform(start_xform);
		anim_mov_blend = m_EntityAlife.animation_movement()->ControlBlend();

		m_EntityAlife.destroy_anim_mov_ctrl();
		BR.set_callback_overwrite(TRUE);
	}
	//
	u16 anim_root = K->LL_GetBoneRoot();
	u16 physics_root = anim_root;

	if (m_eType != etBitting)
	{
		physics_root = K->LL_BoneID("bip01_pelvis");
		K->LL_SetBoneRoot(physics_root);
	}
	//
	if (!m_physics_skeleton)
		CreateSkeleton(m_physics_skeleton);

	if (m_eType == etActor)
	{
		CActor* A = smart_cast<CActor*>(&m_EntityAlife);
		R_ASSERT2(A, "not an actor has actor type");
		if (A->Holder()) return;
		if (m_eState == esRemoved)return;
	}


	//////////////////////this needs to evaluate object box//////////////////////////////////////////////////////
	if (m_eType != etBitting)
		K->LL_SetBoneRoot(anim_root);

	for (u16 I = K->LL_BoneCount() - 1; I != u16(-1); --I)
		K->LL_GetBoneInstance(I).reset_callback();
	//
	if (anim_mov_ctrl) //we do not whant to move by long animation in root 
		BR.set_callback_overwrite(TRUE);
	//
	K->CalculateBones_Invalidate();
	K->CalculateBones(TRUE);
	if (m_eType != etBitting)
		K->LL_SetBoneRoot(physics_root);
	////////////////////////////////////////////////////////////////////////////

	if (m_pPhysicsShell)
		return;

	m_PhysicMovementControl->GetCharacterVelocity(velocity);


	if (!m_PhysicMovementControl->CharacterExist())
		dp.set(m_EntityAlife.Position());
	else m_PhysicMovementControl->GetDeathPosition(dp);
	m_PhysicMovementControl->DestroyCharacter();

	//shell create
	R_ASSERT2(m_physics_skeleton, "No skeleton created!!");
	m_pPhysicsShell = m_physics_skeleton;
	m_physics_skeleton = NULL;
	m_pPhysicsShell->set_Kinematics(K);
	m_pPhysicsShell->RunSimulation();
	m_pPhysicsShell->mXFORM.set(mXFORM);
	m_pPhysicsShell->SetCallbacks();
	//

	if (anim_mov_ctrl) //we do not whant to move by long animation in root 
		BR.set_callback_overwrite(TRUE);

	if (!DoCharacterShellCollide())
		m_pPhysicsShell->SetDeadBody();

	if (m_eType != etBitting)
		K->LL_SetBoneRoot(anim_root);


	K->CalculateBones_Invalidate();
	K->CalculateBones(TRUE);


	if (m_eType != etBitting)
		K->LL_SetBoneRoot(physics_root);
	//reset_root_bone_start_pose( *m_pPhysicsShell );

	m_flags.set(fl_death_anim_on,FALSE);
	m_eState = esDead;
	m_flags.set(fl_skeleton_in_shell,TRUE);

	if (IsGameTypeSingle())
	{
		m_pPhysicsShell->SetPrefereExactIntegration(); //use exact integration for ragdolls in single
		m_pPhysicsShell->SetDeadBody();
	}
	else
		m_pPhysicsShell->SetIgnoreDynamic();
	m_pPhysicsShell->SetIgnoreSmall();
	if (m_eType == etStalker)
		painter_limb_torso_collision(*m_pPhysicsShell, *K);
	AddActiveWeaponCollision();
#ifdef PAINTER_ACTIVE_RAGDOLL_NATIVE
	// The native controller uses the shared human stalker shell profile.  Keep
	// actors and all monster families on the stock ragdoll path until they have
	// their own verified bone and joint map.
    if (m_eType == etStalker)
    {
        m_active_ragdoll_controller.attach(m_pPhysicsShell, &m_EntityAlife);
    }
#endif
}

void CCharacterPhysicsSupport::EndActivateFreeShell(CObject* who, const Fvector& inital_entity_position,
                                                    const Fvector& dp, const Fvector& velocity)
{
	VERIFY(m_pPhysicsShell);
	VERIFY(m_eState==esDead);
#ifdef	DEBUG
if( dbg_draw_ragdoll_spawn )
{
	DBG_OpenCashedDraw();
	m_pPhysicsShell->dbg_draw_geometry( 0.2f, D3DCOLOR_XRGB( 255, 100, 0 ) );
	DBG_ClosedCashedDraw( 50000 );
}
#endif

	CollisionCorrectObjPosStaticOnly(dp);
	m_pPhysicsShell->SetGlTransformDynamic(mXFORM);

#ifdef	DEBUG
if( dbg_draw_ragdoll_spawn )
{
	DBG_OpenCashedDraw();
	m_pPhysicsShell->dbg_draw_geometry( 0.2f, D3DCOLOR_XRGB( 255, 0, 100 ) );
	DBG_ClosedCashedDraw( 50000 );
}
#endif
	// Static-only collision correction places the newly activated ragdoll clear
	// of level geometry without waking nearby dynamic physics objects. Do not
	// synchronously advance the entire physics
	// world to move one corpse back toward its pre-correction position: that legacy
	// workaround turns a death event into ten global fixed steps on the main
	// thread. The shell starts from the corrected transform and receives the
	// preserved character velocity below.

#ifdef	DEBUG
if( dbg_draw_ragdoll_spawn )
{
	DBG_OpenCashedDraw();
	m_pPhysicsShell->dbg_draw_geometry( 0.2f, D3DCOLOR_XRGB( 100, 255, 100 ) );
	DBG_ClosedCashedDraw( 50000 );
}
#endif
	Fvector v = velocity;
	m_character_shell_control.apply_start_velocity_factor(who, v);

#ifdef	DEBUG
		if( death_anim_debug )
		{
			Msg( "death anim: ragdoll velocity picked from char controller =(%f,%f,%f), velocity applied to ragdoll =(%f,%f,%f)  ", velocity.x, velocity.y, velocity.z, v.x, v.y, v.z );
		}
#endif

	m_pPhysicsShell->set_LinearVel(v);
	//actualize
	m_pPhysicsShell->GetGlobalTransformDynamic(&mXFORM);
	m_pPhysicsShell->mXFORM.set(mXFORM);


	//if( false &&  anim_mov_ctrl && anim_mov_blend && anim_mov_blend->blend != CBlend::eFREE_SLOT &&  anim_mov_blend->timeCurrent + Device.fTimeDelta*anim_mov_blend->speed < anim_mov_blend->timeTotal-SAMPLE_SPF-EPS)//.
	//{
	//	const Fmatrix sv_xform = mXFORM;
	//	mXFORM.set( start_xform );
	//	//anim_mov_blend->blendPower = 1;
	//	anim_mov_blend->timeCurrent  += Device.fTimeDelta * anim_mov_blend->speed;
	//	m_pPhysicsShell->AnimToVelocityState( Device.fTimeDelta, 2 * default_l_limit, 10.f * default_w_limit );
	//	mXFORM.set( sv_xform );
	//}
	IKinematics* K = smart_cast<IKinematics*>(m_EntityAlife.Visual());
	//u16 root =K->LL_GetBoneRoot();
	//if( root!=0 )
	//{
	//	K->LL_GetTransform( 0 ).set( Fidentity );
	//	
	//	K->LL_SetBoneVisible( 0, FALSE, FALSE );
	//}

	K->CalculateBones_Invalidate();
	K->CalculateBones(TRUE);

#ifdef PAINTER_ACTIVE_RAGDOLL_NATIVE
	// Enable only after the shell has its final death transform and velocity.
	// The controller performs a two frame ODE warmup before capturing its pose.
	if (m_eType == etStalker) {
        m_active_ragdoll_controller.hit(m_sv_hit,true);
        // R35: enabled only when painter_native_ragdoll is on (or the balance test wants the shell); otherwise the
        // controller stays passive and the Lua controller runs its motors at full strength, as in the co-op exe.
        m_active_ragdoll_controller.set_enabled((painter_native_ragdoll || painter_balance_test_enabled) &&
                                                m_active_ragdoll_controller.supported());
    }
#endif
	if (m_eType == etStalker && !m_EntityAlife.g_Alive() && painter_ragdoll_capture_next && m_pPhysicsShell)
	{
		painter_ragdoll_capture_next = 0;
		ragdoll_shell_capture.start(m_pPhysicsShell, m_EntityAlife);
	}
}

void CCharacterPhysicsSupport::in_ChangeVisual()
{
    painter_piece_continuation::forget(m_EntityAlife);
    painter_arm_pose_owner_gone(m_EntityAlife);
	CancelPainterGeometryProbe();
	m_painter_geometry_installed = false;
#ifdef PAINTER_ACTIVE_RAGDOLL_NATIVE
	// Visual replacement can tear down and recreate the shell.  Detach first
	// so the controller never retains joints from the old shell.
	m_active_ragdoll_controller.detach();
#endif
	IKinematicsAnimated* KA = smart_cast<IKinematicsAnimated*>(m_EntityAlife.Visual());
	if (m_ik_controller)
	{
		DestroyIKController();
		if (KA)
			CreateIKController();
	}
	xr_delete(m_interactive_animation);
	destroy_animation_collision();
	destroy(m_interactive_motion);

	if (KA)
	{
		m_death_anims.setup(KA, m_EntityAlife.cNameSect().c_str(), pSettings);
		if (Type() != etBitting)
			m_hit_animations.SetupHitMotions(*KA);
	}

	if (!m_physics_skeleton && !m_pPhysicsShell) return;

	if (m_pPhysicsShell)
	{
		// Painter: the old shell is destroyed below (visual change, or a script changed bone visibility); drop every
		// Painter hold on it first, as in_ChangeVisual does (idempotent, so that path is unchanged)
		painter_piece_continuation::forget(m_EntityAlife);
		painter_arm_pose_owner_gone(m_EntityAlife);
		CancelPainterGeometryProbe();
		m_painter_geometry_installed = false;
#ifdef PAINTER_ACTIVE_RAGDOLL_NATIVE
		m_active_ragdoll_controller.detach();
#endif
		VERIFY(m_eType!=etStalker);
		if (m_physics_skeleton)
		{
			m_EntityAlife.processing_deactivate();
			m_physics_skeleton->Deactivate();
			xr_delete(m_physics_skeleton);
		}
		if (m_EntityAlife.Visual())
			CreateSkeleton(m_physics_skeleton);
		if (m_pPhysicsShell)
			m_pPhysicsShell->Deactivate();
		xr_delete(m_pPhysicsShell);
		if (m_EntityAlife.Visual())
			ActivateShell(NULL);
	}
}

void CCharacterPhysicsSupport::RecreateSkeleton()
{
	if (!m_physics_skeleton && !m_pPhysicsShell) return;

	if (m_pPhysicsShell)
	{
		VERIFY(m_eType!=etStalker);
		if (m_physics_skeleton)
		{
			m_EntityAlife.processing_deactivate();
			m_physics_skeleton->Deactivate();
			xr_delete(m_physics_skeleton);
		}
		if (m_EntityAlife.Visual())
			CreateSkeleton(m_physics_skeleton);
		if (m_pPhysicsShell)
			m_pPhysicsShell->Deactivate();
		xr_delete(m_pPhysicsShell);
		if (m_EntityAlife.Visual())
			ActivateShell(NULL);
	}
}

bool CCharacterPhysicsSupport::CanRemoveObject()
{
	if (m_eType == etActor)
	{
		return false;
	}
	else
	{
		return !m_EntityAlife.IsPlaying();
	}
}

void CCharacterPhysicsSupport::PHGetLinearVell(Fvector& velocity)
{
	if (m_pPhysicsShell && m_pPhysicsShell->isActive())
	{
		m_pPhysicsShell->get_LinearVel(velocity);
	}
	else
		movement()->GetCharacterVelocity(velocity);
}

void CCharacterPhysicsSupport::CreateIKController()
{
	VERIFY(!m_ik_controller);
	m_ik_controller = xr_new<CIKLimbsController>();
	m_ik_controller->Create(&m_EntityAlife);
}

void CCharacterPhysicsSupport::DestroyIKController()
{
	if (!m_ik_controller)return;
	m_ik_controller->Destroy(&m_EntityAlife);
	xr_delete(m_ik_controller);
}

void CCharacterPhysicsSupport::in_NetRelcase(CObject* O)
{
	m_PhysicMovementControl->NetRelcase(O);

	if (m_sv_hit.is_valide() && m_sv_hit.initiator() == O)
		m_sv_hit = SHit();
}

void CCharacterPhysicsSupport::set_collision_hit_callback(ICollisionHitCallback* cc)
{
	xr_delete(m_collision_hit_callback);
	m_collision_hit_callback = cc;
}

ICollisionHitCallback* CCharacterPhysicsSupport::get_collision_hit_callback()
{
	return m_collision_hit_callback;
}


void CCharacterPhysicsSupport::on_create_anim_mov_ctrl()
{
	VERIFY(!anim_mov_state.active);
	//anim_mov_state.character_exist = m_PhysicMovementControl->CharacterExist(); 
	//if(anim_mov_state.character_exist)
	//m_PhysicMovementControl->DestroyCharacter();
	m_PhysicMovementControl->SetNonInteractive(true);
	anim_mov_state.active = true;
}

void CCharacterPhysicsSupport::on_destroy_anim_mov_ctrl()
{
	VERIFY(anim_mov_state.active);
	//if( anim_mov_state.character_exist )
	//CreateCharacter();
	m_PhysicMovementControl->SetNonInteractive(false);
	anim_mov_state.active = false;
}

bool CCharacterPhysicsSupport::interactive_motion()
{
	return is_imotion(m_interactive_motion);
}

bool CCharacterPhysicsSupport::can_drop_active_weapon()
{
	return !interactive_motion() && m_flags.test(fl_death_anim_on);
};

void CCharacterPhysicsSupport::in_Die(bool hit)
{
	if (m_hit_valide_time < Device.dwTimeGlobal || !m_sv_hit.is_valide())
	{
		if (m_EntityAlife.use_simplified_visual())
			return;
		ActivateShell(NULL);
		m_PhysicMovementControl->DestroyCharacter();
		return;
	}

	if (hit)
		in_Hit(m_sv_hit, true);
}

u16 CCharacterPhysicsSupport::PHGetSyncItemsNumber()
{
	if (movement()->CharacterExist())
		return 1;
	else
		return m_EntityAlife.CPhysicsShellHolder::PHGetSyncItemsNumber();
}

CPHSynchronize* CCharacterPhysicsSupport::PHGetSyncItem(u16 item)
{
	if (movement()->CharacterExist())
		return movement()->GetSyncItem();
	else
		return m_EntityAlife.CPhysicsShellHolder::PHGetSyncItem(item);
}


#include "PainterRuntimeGeometryDiagnostic_impl.h"

#include "PainterVoxelLive_impl.h"
