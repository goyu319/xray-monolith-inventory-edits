#pragma once
// Owner-thread transaction input for an already partitioned damaged GFY body.
// A worker must prove component/child ownership before supplying this packet.
#include "LiveDamageWorker.h"
#include "GoreControls.h"
#include "PartChildOwnership.h"
#include "../../xrPhysics/FittedShell.h"
#include <memory>
#include <vector>

class CAI_Stalker;
class CPhysicsSkeletonObject;
class CSE_Abstract;
class CPhysicsShell;
class IKinematics;
struct RuntimeSkinBone;
struct SPainterDamagedFragmentData;

namespace painter_flight { struct Group; }
namespace painter_damaged_fragment {
struct Piece {
    std::shared_ptr<const gfy_packed::Mesh> flight_visual;
    bool ocular_remnant=false;
    bool burst_chunk=false; // chunknosplash: SeparationPiece::burst_chunk -> sever.chunk in the commit receipt
    bool destroy_on_commit=false; // Partition its exact surface out of the body without a physical child.
    gfy_packed::Mesh mesh; // Original model rest coordinates and original skin weights.
    std::shared_ptr<gfy_live::ComponentSession> continuation;
    std::shared_ptr<const gfy_live::PieceSeed> seed; // low-poly continuation (PainterPieceContinuation_impl.h)
    std::vector<part_child_ownership::Decision> children;
    std::vector<PHFittedBone> fitted; // Native articulated elements, fitted to owned anatomy.
    std::vector<PHFittedBone> flesh_gear; // gearbody: boxes around its merged gear - the bullet flesh test only, never physics
    // For each weighted bone absent from fitted, a proven retained element ID.
    // BI_NONE for weighted bones already represented by their own element.
    std::vector<u16> pose_anchors;
    u64 visible_bones=0;
    u16 root_bone=BI_NONE;
    Fmatrix anchor_model; // Native mTransform at the capture barrier.
    Fmatrix anchor_render; // Native mRenderTransform at the same barrier.
    u16 anchor=BI_NONE;
    u32 component_id=0;
};
struct Prepared {
    std::shared_ptr<painter_flight::Group> flight;
    gfy_live::Stamp stamp;
    gfy_live::GoreControls gore; // Carries the partition's policy through native staging.
    gfy_live::Identity current_owner;
    u64 expected_generation=0;
    u64 publication_revision=0;
    gfy_live::SeparationInput continuation_input;
    std::shared_ptr<gfy_live::ComponentSession> stump_continuation;
    std::shared_ptr<const gfy_live::PieceSeed> stump_seed;
    u16 source_id=BI_NONE;
    shared_str source_visual;
    Fmatrix source_world;
    std::vector<Fmatrix> captured_render_palette;
    std::vector<part_child_ownership::Role> original_roles;
    gfy_packed::Mesh stump;
    std::vector<part_child_ownership::Decision> stump_children;
    std::vector<PHFittedBone> stump_physics;
    std::vector<PHFittedBone> stump_flesh_gear; // gearbody: boxes around the body's merged gear - the bullet flesh test only
    std::vector<u16> stump_pose_anchors;
    u64 stump_visible_bones=0;
    u16 stump_root_bone=BI_NONE;
    std::vector<Piece> pieces;
};

// Submit at an idle geometry barrier. Spawns only hidden, collision-disabled
// skeleton objects; the source is untouched. Poll from the same owner barrier.
bool submit(CAI_Stalker&, const gfy_live::Identity&, Prepared&, const xr_vector<RuntimeSkinBone>&, // piececommit: meshes moved out
            const xr_vector<shared_str>& tissue_textures, CPhysicsShell* source_shell,
            const char*& reason);
bool pending(u16 source_id);
// R33: the reason and revision of the last poll() cancel of this source, taken once (the owner re-runs its check).
// Exact owner identity is required; reused-ID receipts are discarded. Older generations of the same owner remain valid.
bool take_canceled(u16 source_id,const gfy_live::Identity& identity,const char*& reason,u64& revision);
// Dangling sever (SeverCheck.h): the next separation commit of this source tethers the piece torn off at `bone`
// (or below it) to the stump at rest_point (bind space of `bone`), a ball joint that breaks past break_force
// newtons or after life_ms.
void arm_tether(u16 source_id,u16 bone,const Fvector& rest_point,float break_force,u32 life_ms);
// maskdecal (sever splashes): the verdict of a Lua sever of this source (rest model space: the station's centre and the limb
// axis); the commit of the separation it causes measures each cut end's skin edge from it (receipt.sever.skin_edge).
void arm_cut(u16 source_id,u16 bone,const Fvector& rest_point,const Fvector& rest_axis);
// Returns true only after every child visual and shell has been prepared and
// source/fragment packets have committed. Failure destroys staged children.
bool poll(CAI_Stalker&, const gfy_live::Identity&, u64 current_generation,
          const xr_vector<RuntimeSkinBone>&, const xr_vector<shared_str>&,
          CPhysicsShell* source_shell);
void cancel(u16 source_id);

// CPhysicsSkeletonObject invokes these around its native CPHSkeleton::Spawn.
int begin_spawn(CPhysicsSkeletonObject&, CSE_Abstract*, SPainterDamagedFragmentData*&);
bool build_shell(CPhysicsSkeletonObject&, SPainterDamagedFragmentData*);
void end_spawn(CPhysicsSkeletonObject&, SPainterDamagedFragmentData*, bool success);
void gone(CPhysicsSkeletonObject&, SPainterDamagedFragmentData*&);
}
namespace painter_gut_spill {
// A torso tear (pelvis, spine or spine1 station; rest_point/rest_axis in bind space of `bone`) marks where the next
// split of this body was cut; that split may spill intestines there (PainterGutSpill_impl.h).
void arm(u16 source_id,u16 bone,const Fvector& rest_point,const Fvector& rest_axis);
void arm(u16 source_id,u16 bone,const Fvector& rest_point,const Fvector& rest_axis,const gfy_live::GoreControls& controls);
// A big abdominal exit or burst entry that destroyed intestine: its mouth (point, outward normal) in `bone`'s frame.
// The loop comes out once the body is a ragdoll (poll_wound, every frame while wound_pending).
void wound(u16 source_id,u16 bone,const Fvector& point,const Fvector& normal,float chance);
void wound(u16 source_id,u16 bone,const Fvector& point,const Fvector& normal,float chance,
           const gfy_live::GoreControls& controls,float opening_radius);
bool wound_pending(u16 source_id);
void poll_wound(CAI_Stalker& source,CPhysicsShell* shell);
}
