#pragma once
#include "LiveDamageWorker.h"
#include "../../Include/xrRender/RuntimeSkinGeometry.h"
#include <memory>
#include <vector>

class CPhysicsShellHolder;
class CAI_Stalker;
class CPhysicsSkeletonObject;
class CPhysicsShell;
class IKinematics;
struct SHit;
struct RuntimeSkinBone;

// The wound worker pool of live bodies, shared by severed pieces (PainterVoxelLive_impl.h). Null only when it cannot
// be created.
gfy_live::Pool* painter_voxel_live_pool();
namespace painter_explosions {struct RouteWitness;}

namespace painter_flight { struct Member; }
namespace painter_piece_continuation {
// After a split, the stump and every severed piece own a piece session (Pool::attach_piece): reserved before the
// split's visual/physics commit with the piece's seed and the mesh as installed, activated after it (the session
// starts at the committed generation and continues the damage revisions from `published_revision`).
bool reserve(CPhysicsShellHolder&,std::shared_ptr<const gfy_live::PieceSeed>,std::shared_ptr<const gfy_packed::Mesh> published,
    gfy_live::Identity,std::uint64_t projected_generation,
    const gfy_live::SeparationInput&,const xr_vector<RuntimeSkinBone>&,
    const xr_vector<shared_str>&,std::uint16_t original_children,
    std::uint64_t published_revision,const painter_flight::Member* flight=nullptr);
bool activate(CPhysicsShellHolder&,std::uint64_t actual_generation);
void abandon(CPhysicsShellHolder&);
// R31: the collision group a split registered its stump and pieces to (CGID); a later split of this object keeps it.
void set_collision_group(CPhysicsShellHolder&,unsigned group);
bool bind(CPhysicsShellHolder&,std::shared_ptr<const gfy_live::PieceSeed>,std::shared_ptr<const gfy_packed::Mesh> published,
    gfy_live::Identity,std::uint64_t,const gfy_live::SeparationInput&,
    const xr_vector<RuntimeSkinBone>&,const xr_vector<shared_str>&,
    std::uint16_t original_children,std::uint64_t published_revision);
void forget(CPhysicsShellHolder&);
bool bind_burn_receiver(CPhysicsShellHolder&);
bool prime_burn_geometry(CPhysicsShellHolder&,std::uint64_t);
bool owns(const CPhysicsShellHolder&,std::uint16_t bone);
bool capture(CPhysicsShellHolder&,const SHit&);
// Blast craters on an owned piece or stump (numbered here); returns how many were queued.
unsigned blast(CPhysicsShellHolder&,std::vector<gfy_live::Impact>&);
void queue(CPhysicsShellHolder&);
void poll(CPhysicsShellHolder&);
bool grenade_ready(const painter_explosions::RouteWitness&);
void retry_grenade_routes();
}
