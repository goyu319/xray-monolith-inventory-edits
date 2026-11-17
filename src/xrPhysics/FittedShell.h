#pragma once
// Reviewed surviving per-element anatomy. Boxes are expressed in the named
// native bone's local physical frame; all original joints remain native.
#include <vector>
class IPhysicsShellHolder;
class CPhysicsShell;
namespace gfy_packed {struct Mesh;}
struct PHFittedBone {
    u16 bone=0;
    bool retain_original_shape=false; // Certified complete, unchanged native element.
    std::vector<Fobb> boxes;
    float mass=0.f;
    Fvector mass_center;
    float inertia[9]{}; // symmetric positive tensor at COM, kg m^2
    // bodyjitter3: a native element of the remaining body cut back at a cut (retain_original_shape only): its native shapes and
    // its rigid parts' are clipped to n.x <= d ({n.x,n.y,n.z,d}, its bone frame; PHShell.cpp painter_trim_shape).
    // P_prepare_FittedShell gives it the mass of the element it replaces times mass_scale (into `mass`).
    float trim[4][4]{};
    u8 trims=0;
    float mass_scale=1.f;
};
struct PHFittedPoseLink {
    u16 bone=0;
    u16 anchor=0; // An element retained by this fitted shell.
    Fmatrix relative; // Captured anchor-model inverse * bone-model.
};
// Builds ODE bodies disabled and without source bone callbacks. The caller
// owns the shell and must activate callbacks only at the owner/physics barrier.
XRPHYSICS_API CPhysicsShell* P_prepare_FittedShell(IPhysicsShellHolder* target,
    CPhysicsShell* original_pose_shell,const std::vector<PHFittedBone>&,
    u64 visible_mask,bool bind_target_callbacks,u16 root_bone=BI_NONE,const gfy_packed::Mesh* coverage_mesh=nullptr);
// Stores a complete, explicit missing-bone palette. apply_now is true only
// for a hidden fragment; the source shell installs links during its final swap.
XRPHYSICS_API bool P_bind_FittedPoseLinks(CPhysicsShell*,
    const std::vector<PHFittedPoseLink>&,bool apply_now);
// Complete explicit native-pose initialization of a disabled staging shell.
// Refreshes interpolation and activation flags without enabling simulation.
XRPHYSICS_API bool P_refresh_FittedPose(CPhysicsShell*);
// Why the last P_prepare_FittedShell returned null (a short reason for the refusal log lines).
XRPHYSICS_API const char* P_fitted_shell_refusal();
