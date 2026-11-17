#pragma once
#include "CombatRuntime.h"
#include "OrderMailbox.h"
#include "SquadTactics.h"
#include "KnowledgeInbox.h"
#include "RoomStack.h"
#include "RoomDoctrine.h"
#include "RoomCasualty.h"
#include "RoomDefense.h"
#include "RoomSupport.h"
#include "RoomCoverFire.h"
#include "CombatSideRegistry.h"
#include "CombatMissionIntel.h"
#include "CombatOutdoorDirector.h"
#include "RoomExposureBatch.h"
#include <functional>
#include <memory>

namespace painter { namespace combat { namespace core {
struct DispatchLimits {
    // Zero selects the corresponding Config pacing value.
    std::size_t bootstrap_work_per_tick=0;
    // Scheduling width, never an admission cap. A fresh immutable work item
    // queries/publishes at most this many actors, then advances a fair cursor.
    std::size_t cover_queries_per_slice=0;
    // Rest between fair rounds, not between cohorts with unvisited members.
    TimeMs plan_interval_ms=0;
    // History capacity, not an actor admission cap. Overflow is counted.
    std::size_t knowledge_history_ticks=64;
};
struct NativePublication {
    // Only keys actually installed by the engine. An empty vector accepts none.
    std::vector<ActorKey> accepted;
};
using PublishFunction=std::function<NativePublication(const Decision&)>;
using PointFirePublishFunction=std::function<NativePublication(const Decision&,const std::vector<PointFireAuthority>&)>;
struct DispatchStats {
    std::uint64_t bootstrap_proposed=0,native_accepted=0,native_rejected=0;
    std::uint64_t jobs_submitted=0,jobs_rejected=0,jobs_cancelled=0,jobs_failed=0;
    std::uint64_t plans_completed=0,plans_stale=0,proposals_superseded=0;
    std::uint64_t cohorts_started=0,cohort_members_scheduled=0,cohort_members_accepted=0;
    std::uint64_t proposals_reservation_conflicted=0;
    std::uint64_t first_applied=0,first_deadline_missed=0;
    std::uint64_t episode_admission=0,episode_contact=0,episode_hit=0;
    TimeMs maximum_first_applied_ms=0;
    std::uint64_t knowledge_ticks_committed=0,knowledge_replays_cancelled=0;
    std::uint64_t knowledge_ticks_quarantined=0,knowledge_ticks_stale=0;
    std::uint64_t knowledge_side_identity_resets=0;
    std::uint64_t room_updates=0,room_orders_proposed=0;
};
struct RoomDispatchReceipt {
    std::uint32_t side_id=0,mission_revision=0;
    std::uint64_t snapshot_seq=0;
    RoomStackResult result;
    // FORM is retained separately as historical evidence. Doctrine is the
    // current phase receipt; neither proves every proposal was installed.
    RoomDoctrineResult doctrine;
    // Final role decoration, separate from primary raw assignments. A native
    // treatment receipt and exact movement execution proof remain distinct.
    RoomCasualtyResult casualty;
    bool casualty_assignment_refused=false;
    bool casualty_execution_observed=false;
    std::uint64_t casualty_execution_assignment=0;
    TimeMs casualty_execution_at_ms=0;
    RoomDefenseResult defense;
    bool defense_assignment_refused=false;
    RoomSupportResult support;
    bool support_assignment_refused=false;
    RoomCoverFireResult cover_fire;
};

// Completed worker identity/intelligence receipt. Stamps retain the original
// capture; this is neither admission/posture completion nor live movement proof.
struct SideDispatchReceipt {
    SideRegistryResult registry;
    std::vector<CombatMissionIntel> mission_intel;
    std::vector<SquadView> tactical_groups;
};

// Called only on the serialized capture/publish seam. Workers hold immutable
// snapshots, value results and the epoch-pinned EngineServices. They never hold
// a dispatcher pointer, CObject, Lua value or engine actor pointer.
class CombatDispatcher {
public:
    // Side quiet/cooldown policy is an explicit opt-in until allocated by the
    // shared settings contract. Omission preserves the D2 personal lifecycle.
    explicit CombatDispatcher(Config={},DispatchLimits={},std::shared_ptr<const SideLifecyclePolicy> lifecycle={},
        std::shared_ptr<const OutdoorDispatchPolicy> outdoor={},std::shared_ptr<const RoomExposureDispatchPolicy> exposure={});
    ~CombatDispatcher();
    CombatDispatcher(const CombatDispatcher&)=delete;
    CombatDispatcher& operator=(const CombatDispatcher&)=delete;
    bool ingest(WorldSnapshot,EngineServices,const KnowledgeContext& knowledge_context={},const LifecycleSignals& lifecycle_signals={},
        const OutdoorCaptureEvidence& outdoor_evidence={},std::shared_ptr<const ExposureGeometryInput> exposure_geometry={},
        const FriendlyCasualtyEvidence& friendly_casualties={});
    void tick(TimeMs,const PublishFunction&,bool (*deadline)(void*)=nullptr,void* context=nullptr,
        const PointFirePublishFunction& point_publish={});
    // Safe before or after the owner's queue drain: this only marks our shared
    // cancellation token and drops values; it dereferences no engine service.
    void reset();
    const DispatchStats& stats() const;
    std::vector<Order> accepted_orders() const;
    // Last committed receipt, stamped with its ORIGINAL capture sequence.
    // Not a claim that visibility is still current at the time of this read.
    std::shared_ptr<const KnowledgeFrame> knowledge_frame() const;
    const KnowledgeInboxStats& knowledge_inbox_stats() const;
    // Geometry/arrival results at their source snapshot, not native acceptance
    // of every slot and not completion of room clearing.
    std::vector<RoomDispatchReceipt> room_frames() const;
    std::shared_ptr<const SideDispatchReceipt> side_frame() const;
    std::shared_ptr<const SideLifecycleResult> lifecycle_frame() const;
    std::vector<OutdoorDispatchReceipt> outdoor_frames() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}}}
