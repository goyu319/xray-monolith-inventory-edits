#pragma once
#include "CombatContract.h"
#include "core/PointFireAuthority.h"
class CAI_Stalker;
class CStalkerPlanner;
class CGameObject;
class CPhysicsShellHolder;
namespace painter { namespace combat { namespace core { struct OutdoorCapturedEye; struct SideLifecycleResult; struct LifecycleSignals; struct OutdoorDispatchReceipt; struct OutdoorGrenadeFuse; struct RoomDispatchReceipt; }}}
namespace painter { namespace combat { namespace engine {
struct PublishFailure {ActorKey actor;std::uint8_t reason=reject_none;std::uint32_t yields=yield_none;};
struct PublishResult {
    std::uint8_t envelope=reject_none;
    std::vector<ActorKey> accepted_orders,accepted_releases;
    std::vector<PublishFailure> rejected;
};
// Authority thread only: the owner places capture/publication after the scheduler
// in Device.seqParallel (or directly after it when mt_Scheduler is off).
void register_actor(CStalkerPlanner& planner,CAI_Stalker& actor);
void set_enabled(bool enabled);
bool enabled();
// Caller supplies nonzero level_epoch/sequence, current geometry_revision and now_ms.
// Only actors are replaced; doors, sides and envelope stamps remain caller-owned.
void capture_all(WorldSnapshot& snapshot);
// Optional measured-only values, sampled on this same authority-thread capture.
// No fallback eye height; the one-argument entry point keeps its existing ABI.
void capture_all(WorldSnapshot& snapshot,std::vector<core::OutdoorCapturedEye>* measured_eyes);
PublishResult publish(Decision const& decision);
PublishResult publish_points(Decision const& decision,const std::vector<core::PointFireAuthority>& points);
void release_all(std::uint32_t reason=yield_released);
void synchronize_lifecycle(const core::SideLifecycleResult&);
void capture_lifecycle_signals(const core::SideLifecycleResult*,const WorldSnapshot&,core::LifecycleSignals&);
void capture_grenade_fuses(const WorldSnapshot&,std::vector<core::OutdoorGrenadeFuse>&);
void report_outdoor_decisions(const std::vector<core::OutdoorDispatchReceipt>&);
void report_room_decisions(const std::vector<core::RoomDispatchReceipt>&);
void report_runtime_cost(TimeMs now,float total_ms,float capture_ms,float dispatch_ms,float publish_ms,std::uint64_t failures);
void report_response_causes(TimeMs now,std::uint64_t admitted,std::uint64_t contact,std::uint64_t hit,
    std::uint64_t applied,std::uint64_t missed);
void runtime_failed();
void object_destroyed(CGameObject& object);
void restrictions_changed(CAI_Stalker& actor);
bool owns(CAI_Stalker const& actor);
// v2.21 compatibility push model (EngineCompat.h): the registry's key while he is registered; and the forced pass's
// hand-back (a veto / an unknown push-mode verdict) through relinquish -> cleanup -> on_release.
bool actor_key(CAI_Stalker const& actor,ActorKey& key);
// Main-thread physical-separation callback, before the original Painter Lua effects callback.
// An unseen sever grants no fact; it does not imply death or enemy position.
void notify_committed_sever(CPhysicsShellHolder& subject,std::uint64_t revision,std::uint32_t component);
void compat_handback(CAI_Stalker& actor,std::uint32_t reason);
}}}
