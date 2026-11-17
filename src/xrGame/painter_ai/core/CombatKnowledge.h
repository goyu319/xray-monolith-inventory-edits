#pragma once
#include "../CombatContract.h"
#include <memory>
#include <string>

namespace painter { namespace combat { namespace core {

// KnowledgeConfig is the canonical v2.15 Config::knowledge type.
struct FoeId {
    ActorKey actor; std::uint32_t side=0; std::uint64_t ghost_serial=0;
    bool ghost() const { return ghost_serial!=0; }
    bool valid() const { return ghost() || (actor.id<invalid_id && actor.generation!=0); }
    bool operator==(const FoeId& b) const { return actor==b.actor && side==b.side && ghost_serial==b.ghost_serial; }
    bool operator!=(const FoeId& b) const { return !(*this==b); }
};
enum class KnowledgeSource : std::uint8_t { seen,heard,squad_seen,squad_heard };
struct SourceFact {
    bool valid=false; Vec3 position; TimeMs at_ms=0; ActorKey reporter;
    SoundKind sound=SoundKind::other;
};
struct TimedStatus { bool set=false; TimeMs at_ms=0,until_ms=0; bool active(TimeMs now) const {return set&&at_ms<=now&&now<until_ms;} };
struct FoeKnowledge {
    FoeId id; std::array<SourceFact,4> sources{};
    bool visible=false,facing_me=false,down=false,wounded=false,search_open=false;
    TimeMs first_ms=0,down_ms=0,wounded_ms=0; Vec3 wounded_position; ActorKey status_reporter;
    float duration_jitter=1;
    TimedStatus shot_at_me,shot_me,reloading,suppressed;
    ActorKey suppression_reporter; std::uint64_t suppression_owner=0;
    const SourceFact* last_known() const;
    const SourceFact* own_last_known() const;
};
enum class ReportKind : std::uint8_t { seen,heard,shot_from,down,foe_vulnerable,reloading,man_down,under_fire,wounded,room_cells,suppressed };
enum class TargetReason : std::uint8_t { none,dogfight,visible,keep,shot_me,unseen_shooter,recent,nearest_known,engaged,closest_known };
enum class Intent : std::uint8_t { none,hold,seek_cover,retreat,shift_cover,rush,push,other };
struct LooseContact { Vec3 position; TimeMs at_ms=0; SoundKind kind=SoundKind::other; bool from_report=false,from_peace=false; ActorKey reporter; };
struct FreshFire {
    bool valid=false; FoeId shooter; TimeMs at_ms=0,first_ms=0; Vec3 point,direction,origin;
    std::uint32_t count=0; bool hit=false;
};
// point is the known/inferred shooter location; origin is the observer's
// position at the hit. direction points from origin toward the reported source.
// Neither field is Danger::hit's bullet-impact point or hidden actor truth.
struct HitKnowledge { bool valid=false,unseen=false; FoeId shooter; Vec3 point,origin,direction; TimeMs at_ms=0; ActorKey observer; };
struct BuddyFact { ActorKey member; ReportKind kind=ReportKind::under_fire; FoeId enemy; Vec3 position; TimeMs at_ms=0; };
// A single physically observed room cell. The high bit of sample distinguishes
// a full-nav sight node from an ordinary coverage sector. It is never a room
// clear by itself. Recipients must assemble the certified physical proof.
struct RoomCellFact {
    std::uint32_t side=0,building=invalid_room,room=invalid_room;
    std::uint64_t mission_revision=0,level_epoch=0,map_hash=0,geometry_revision=0,attempt=0;
    std::uint64_t sample=0;ActorKey observer;std::uint64_t observer_owner=0;
    TimeMs observed_at_ms=0;
};
struct SearchedRoom {std::uint32_t side=0,building=invalid_room,room=invalid_room;TimeMs at_ms=0;};
struct MemberKnowledge {
    ActorKey actor; std::uint32_t side=0; std::uint64_t ownership_generation=0;
    bool alive=true,able=true,reloading=false,in_contact=false;
    Vec3 position; std::vector<FoeKnowledge> foes; std::vector<LooseContact> contacts; std::vector<BuddyFact> board;
    std::vector<RoomCellFact> room_cells; // own observations and actual delivered radio/voice facts only
    std::vector<SearchedRoom> communicated_rooms; // physical proof assembled from this recipient's delivered cells
    FoeId target,best_visible,dogfight; TargetReason target_reason=TargetReason::none;
    bool threat_valid=false,threat_visible=false,escape_line=false,escape_run=false;
    Vec3 threat; FreshFire fresh_fire; HitKnowledge hit_from;
    float meter=0; std::uint8_t meter_level=0; std::size_t known_count=0,unseen_count=0;
    TimeMs last_seen_ms=0,last_hit_ms=0,last_heard_ms=0;
    bool has_seen=false,has_hit=false,has_heard=false;
};
struct SideKnowledge {
    std::uint32_t side=0; std::size_t known_count=0,unseen_count=0;
    TimeMs last_info_ms=0,last_seen_ms=0,last_heard_ms=0,last_hit_ms=0;
    bool has_info=false; std::vector<ActorKey> confirmed_down;
    std::vector<HitKnowledge> hits; // each is an observed hit, not true enemy state
};
enum class KnowledgeEventKind : std::uint8_t { target,meter,ghost_born,ghost_resolved,ghost_expired,report,dog_start,dog_end,enemy_down,enemy_wounded };
struct KnowledgeEvent {
    KnowledgeEventKind kind=KnowledgeEventKind::target; std::uint32_t side=0;
    ActorKey actor,receiver; FoeId foe,resolved_to; TimeMs at_ms=0;
    TargetReason target_reason=TargetReason::none; ReportKind report=ReportKind::seen;
    bool delivered=false; float value=0;
};

// Decision-core state, NOT additional engine enemy facts. An absent field stays
// unknown. In particular radio policy must be supplied, never guessed from IDs.
struct MemberKnowledgeContext {
    ActorKey actor; Intent intent=Intent::none; FoeId goal_enemy;
    bool searching=false,running_to_cover=false,assault_role=false;
    bool dog_no_solution=false; FoeId dog_no_solution_target; TimeMs dog_no_solution_at_ms=0;
};
// Actual applied-order context, never a proposal or hidden enemy observation.
bool executed_knowledge_context(const WorldSnapshot&,const ActorSnapshot&,const Order&,
    const MemberKnowledge*,TimeMs maximum_age,MemberKnowledgeContext&);
struct SideRadioPolicy {std::uint32_t side=0; bool radio=false;};
struct RoomClearProof {
    SearchedRoom physical;
    std::uint64_t epoch=0,map=0,geometry=0,mission=0,attempt=0;
    std::vector<RoomCellFact> cells; // exact observed cells of this physical clear
};
struct SeenNode {std::uint32_t side=0,node=invalid_node;TimeMs at_ms=0;};
struct SuppressedFoeFact {ActorKey observer;FoeId foe;TimeMs at_ms=0;
    std::uint64_t ownership_generation=0,level_epoch=0;};
// Current personal friendly observation, already membership-qualified by the registry.
// It is never a synthetic patient ActorSnapshot or movement admission.
struct ObservedCasualty {
    ActorKey observer,patient;
    std::uint64_t level_epoch=0,snapshot_seq=0,observer_owner=0,observer_restrictions=0;
    std::uint32_t side=0,faction=0;
    TimeMs at_ms=0; Vec3 position;
    bool visible=false,friendly=false,alive=true,wounded=false;
};
struct KnowledgeContext {
    std::vector<MemberKnowledgeContext> members;
    std::vector<SideRadioPolicy> radios;
    std::vector<SearchedRoom> searched;
    std::vector<RoomCellFact> room_cells; // prior committed physical observations, source-qualified
    std::vector<RoomClearProof> room_proofs; // physical certificates; never delivered member knowledge by themselves
    std::vector<SeenNode> seen_nodes;
    std::vector<SuppressedFoeFact> suppressed_foes;
};
enum class KnowledgeReject : std::uint8_t {
    none, malformed_input, stale_input, invalid_config, cancelled, failed
};
struct KnowledgeFrame {
    KnowledgeReject rejection=KnowledgeReject::none;
    bool accepted=false,cancelled=false; std::uint64_t level_epoch=0,sequence=0;
    std::vector<MemberKnowledge> members; std::vector<SideKnowledge> sides;
    std::vector<KnowledgeEvent> events;
    std::size_t unassigned_actors=0,missing_impact_distance=0,report_overflow=0;
    std::size_t invalid_facts=0,unverified_shared_rows=0;
};
// Single-owner portable state. update is atomic on cancellation or exceptions.
// Same-ActorKey/same-faction regrouping preserves received personal knowledge;
// ghost identities are remapped, while pending broadcasts stay on their side.
// No gameplay orders, engine objects, hidden enemy roster or implicit reports.
class CombatKnowledge {
public:
    // Production configuration: canonical knowledge data plus seed, voice and factions.
    explicit CombatKnowledge(Config config={});
    // Unit-test convenience; all other Config fields retain their explicit defaults.
    explicit CombatKnowledge(KnowledgeConfig config);
    ~CombatKnowledge();
    CombatKnowledge(CombatKnowledge&&) noexcept;
    CombatKnowledge& operator=(CombatKnowledge&&) noexcept;
    CombatKnowledge(const CombatKnowledge&)=delete;
    CombatKnowledge& operator=(const CombatKnowledge&)=delete;
    // Includes report queues, event dedup, RNG, timers and captured-tick index.
    // Returns null when cancelled. Polls bounded copy loops; OS allocation time is not a hard deadline.
    std::unique_ptr<CombatKnowledge> clone(const std::function<bool()>& cancelled={}) const;
    KnowledgeFrame update(const WorldSnapshot&,const KnowledgeContext& context={},
        const EngineServices& services={},const std::function<bool()>& cancelled={});
    KnowledgeFrame update_observed(const WorldSnapshot&,const std::vector<ObservedCasualty>&,
        const KnowledgeContext& context={},const EngineServices& services={},const std::function<bool()>& cancelled={});
    KnowledgeFrame update_disposable_observed(const WorldSnapshot&,const std::vector<ObservedCasualty>&,
        const KnowledgeContext& context={},const EngineServices& services={},const std::function<bool()>& cancelled={});
    // Validates the entire immutable tick before any state/side-ledger mutation.
    KnowledgeReject validate_input(const WorldSnapshot&,
        const std::function<bool()>& cancelled={}) const;
    // For an OUTER cloned worker transaction only. Malformed/stale inputs are
    // no-ops. Cancellation/exception during mutation poisons the instance;
    // neither update nor clone may reuse it until reset().
    KnowledgeFrame update_disposable(const WorldSnapshot&,const KnowledgeContext& context={},
        const EngineServices& services={},const std::function<bool()>& cancelled={});
    // Target only a reused side identity. False means interrupted; discard clone.
    bool reset_side(std::uint32_t side,const std::function<bool()>& cancelled={});
    void reset();
private:
    struct Impl; std::unique_ptr<Impl> impl_;
};
}}}
