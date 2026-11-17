#include "CombatDispatcher.h"
#include "CombatMissionHandover.h"
#include "CombatFireLanes.h"
#include "RoomFireIntent.h"
#include "CombatSideRegistry.h"
#include "CombatSurvivorCooperation.h"
#include "CombatSurvivorApproach.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace painter { namespace combat { namespace core {
namespace {
MailboxLimits mailbox_limits(const Config& c) {
    MailboxLimits m;m.source_max_age_ms=c.snapshot_max_age_ms;
    m.memory_max_age_ms=c.memory_max_age_ms;m.max_order_lifetime_ms=c.order_lifetime_ms;return m;
}
SquadTuning pending_navigation() {SquadTuning t;t.allow_pending_navigation=true;t.use_side_roster=true;return t;}
struct KnowledgeReplayCancelled {};
bool same_mission(const Mission& a,const Mission& b) {
    if(a.revision!=b.revision||a.kind!=b.kind||a.building!=b.building||a.entry_portal!=b.entry_portal||
        a.rooms.count!=b.rooms.count||a.rooms.count>a.rooms.items.size())return false;
    for(std::size_t i=0;i<a.rooms.count;++i)if(a.rooms.items[i]!=b.rooms.items[i])return false;
    return true;
}
// Cheap authority changes are observed at capture, before a completed worker
// may publish. Enemy updates never enter this comparison. It invalidates plans,
// not personal knowledge or actor admission, and touches no worker-owned state.
struct CaptureAuthority {
    struct Actor {ActorKey key;std::uint32_t faction=0;std::uint64_t native=0;bool alive=false;};
    struct Side {std::uint64_t group=0;std::uint32_t faction=0;Mission mission;std::vector<ActorKey> members;};
    std::uint64_t epoch=0,map=0;
    std::unordered_map<std::uint32_t,Actor> actors;
    std::unordered_map<std::uint32_t,Side> sides;
    bool sides_valid=false;
};
CaptureAuthority capture_authority(const WorldSnapshot& w) {
    CaptureAuthority out;out.epoch=w.level_epoch;out.map=w.map_hash;
    for(const auto& actor:w.actors)out.actors.emplace(actor.key.id,CaptureAuthority::Actor{actor.key,actor.faction,actor.native_squad,actor.alive});
    std::unordered_set<std::uint32_t> members;
    for(const auto& side:w.sides) {
        if(!side.side_id||out.sides.count(side.side_id)||!finite(side.link_up)||
            side.mission.rooms.count>side.mission.rooms.items.size()||
            static_cast<unsigned>(side.mission.kind)>static_cast<unsigned>(MissionKind::hold))return out;
        std::unordered_set<std::uint32_t> rooms;
        for(std::size_t i=0;i<side.mission.rooms.count;++i)
            if(side.mission.rooms.items[i]==invalid_room||!rooms.insert(side.mission.rooms.items[i]).second)return out;
        std::uint64_t native=0;
        std::vector<ActorKey> living_members;
        for(auto key:side.members) {
            if(!valid_key(key)||!members.insert(key.id).second)return out;
            auto actor=out.actors.find(key.id);
            if(actor==out.actors.end()||actor->second.key!=key)continue;
            if(actor->second.faction!=side.faction)return out;
            if(actor->second.native){if(native&&native!=actor->second.native)return out;native=actor->second.native;}
            // A raw mission roster can alternate between including an absent
            // key and omitting it. Compare captured living membership for plan
            // invalidation; absence grants no life/death evidence. Keep the
            // raw roster intact for lifecycle and casualty decisions.
            if(actor->second.alive)living_members.push_back(key);
        }
        if(native&&native!=side.group)return out;
        out.sides.emplace(side.side_id,CaptureAuthority::Side{side.group,side.faction,side.mission,std::move(living_members)});
    }
    out.sides_valid=true;return out;
}
std::unordered_set<std::uint32_t> changed_authority(const CaptureAuthority& old,const CaptureAuthority& next) {
    std::unordered_set<std::uint32_t> changed;
    if(old.epoch!=next.epoch)return changed;
    for(const auto& actor:next.actors) {
        const auto found=old.actors.find(actor.first);
        if(old.map!=next.map||(found!=old.actors.end()&&(found->second.key!=actor.second.key||
            found->second.faction!=actor.second.faction||found->second.native!=actor.second.native)))changed.insert(actor.first);
    }
    // A quarantined malformed roster is not a mission replacement. Compare the
    // next well-formed authority to the last valid one, not to an invalid raw
    // capture that Runtime retained for independent personal safety responses.
    if(!next.sides_valid)return changed;
    auto mark=[&](const CaptureAuthority::Side& side){for(auto key:side.members)changed.insert(key.id);};
    for(const auto& side:next.sides) {
        const auto found=old.sides.find(side.first);
        if(found==old.sides.end()){mark(side.second);continue;}
        const auto& before=found->second;const auto& after=side.second;
        if(before.group!=after.group||before.faction!=after.faction||before.members!=after.members||
            !same_mission(before.mission,after.mission)){mark(before);mark(after);}
    }
    for(const auto& side:old.sides)if(!next.sides.count(side.first))mark(side.second);
    return changed;
}
// An object-ID respawn alone does not invalidate another man's independently
// queried cover. Explicit missions/rosters, faction/group and map changes do.
// The limited salvage lane below publishes NO squad or room authority.
bool only_actor_replacements(const CaptureAuthority& old,const CaptureAuthority& next,
    const std::unordered_set<std::uint32_t>& changed) {
    if(changed.empty()||old.epoch!=next.epoch||old.map!=next.map||
        !old.sides.empty()||!next.sides.empty()||!next.sides_valid)return false;
    for(auto id:changed) {
        auto before=old.actors.find(id),after=next.actors.find(id);
        if(before==old.actors.end()||after==next.actors.end()||before->second.key==after->second.key||
            before->second.faction!=after->second.faction||before->second.native!=after->second.native)return false;
    }
    return true;
}
struct RoomSession {
    explicit RoomSession(Config config):doctrine(config),casualty(config),defense(config),support(config),cover_fire(config){}
    RoomDoctrine doctrine;
    RoomCasualty casualty;
    RoomDefense defense;
    RoomSupport support;
    RoomCoverFire cover_fire;
    std::uint64_t epoch=0,map=0,geometry=0,building_data=0,group=0,trail_sequence=0;
    std::uint32_t faction=0;
    MissionKind mission_kind=MissionKind::none;
    std::vector<RoomPositionCrumb> crumbs;
    std::vector<SearchedRoom> searched;
    std::vector<RoomClearProof> clear_proofs;
    struct DefenseHome {ActorKey actor;std::uint64_t owner=0;std::uint32_t room=invalid_room;};
    std::map<std::uint32_t,DefenseHome> defense_homes;
    std::map<std::uint32_t,RoomDefense> defense_rooms;
    std::size_t defense_cursor=0;
    struct DefenseToken {
        ActorKey actor;std::uint64_t owner=0,restrictions=0,geometry=0,intent=0,token=0;
        std::uint32_t mission=0,room=invalid_room;
    };
    std::map<std::uint32_t,DefenseToken> defense_tokens;
    struct SupportToken {
        ActorKey actor,target;std::uint64_t owner=0,restrictions=0,geometry=0,intent=0,token=0;
        std::uint32_t mission=0;RoomDoctrineSlot parent;
    } support_token;
    ActorKey care_helper,care_target;
    std::uint64_t care_owner=0,care_intent=0,care_assignment=0;
    std::uint64_t care_restrictions=0,care_geometry=0;
    std::uint32_t care_mission=0;
    bool care_executed=false;TimeMs care_executed_at=0;
    std::uint32_t care_pin_room=invalid_room;
    std::uint32_t mission_revision=0,building=invalid_room;
};
struct Worker {
    explicit Worker(Config c,const std::shared_ptr<const SideLifecyclePolicy>& policy,const std::shared_ptr<const OutdoorDispatchPolicy>& outdoor_policy,
        const std::shared_ptr<const RoomExposureDispatchPolicy>& exposure_policy):squads(c,pending_navigation()),knowledge(new CombatKnowledge(c)),side_registry(new CombatSideRegistry(c)) {
        const auto twice=saturated_after(c.order_lifetime_ms,c.order_lifetime_ms);
        side_retain_ms=saturated_after(twice,twice);
        if(policy)lifecycle=std::make_unique<CombatSideLifecycle>(*policy,c);
        if(outdoor_policy){outdoor=std::make_unique<CombatOutdoorDirector>(c,*outdoor_policy);outdoor_line_queries=outdoor_policy->line_queries_per_slice;outdoor_personal=outdoor_policy->personal.enabled;}
        if(exposure_policy){exposure=std::make_shared<RoomExposureBatch>(*exposure_policy);
            exposure_work=exposure_policy->work_per_slice;exposure_queries=exposure_policy->queries_per_slice;}
    }
    SquadTactics squads;TimeMs side_retain_ms=0;
    std::unique_ptr<CombatKnowledge> knowledge;std::unique_ptr<CombatSideRegistry> side_registry;
    CombatSurvivorCooperation cooperation;
    std::unique_ptr<CombatSideLifecycle> lifecycle;
    std::unique_ptr<CombatOutdoorDirector> outdoor;
    std::size_t outdoor_line_queries=0;
    bool outdoor_personal=false;
    std::shared_ptr<RoomExposureBatch> exposure;
    std::size_t exposure_work=0,exposure_queries=0;
    std::map<std::uint32_t,RoomSession> rooms;
    std::vector<BuildingCompletionReceipt> completions;
    std::uint64_t care_assignment_sequence=0;
    std::uint64_t defense_assignment_sequence=0;
    std::uint64_t support_assignment_sequence=0;
};
struct Work {
    // The final shared job reference owns rollback. A cancelled queued job may
    // still be running when capture drops its reference; never close it early.
    ~Work(){if(exposure&&exposure_token)exposure->finish(exposure_token,false);}
    std::atomic<bool> cancelled{false},done{false};
    // Only read/written on capture/publication, never by the worker. The worker
    // still sees its immutable original source; shared-role results are dropped.
    bool personal_cover_only=false;
    WorldSnapshot source;
    std::vector<Order> current;
    std::vector<Order> executed;
    SquadContext context;
    SquadResult result;
    std::vector<KnowledgeInbox::Tick> knowledge_ticks;
    std::vector<std::uint64_t> replayed_sequences;
    std::unique_ptr<CombatKnowledge> staged_knowledge;
    std::unique_ptr<CombatSideRegistry> staged_sides;
    CombatSurvivorCooperation staged_cooperation;
    std::vector<SurvivorCooperation> cooperation_results;
    std::unique_ptr<CombatSideLifecycle> staged_lifecycle;
    std::shared_ptr<const SideLifecycleResult> lifecycle_frame;
    std::unique_ptr<CombatOutdoorDirector> staged_outdoor;
    OutdoorDirectorResult outdoor_result;
    OutdoorExecutionEvidence outdoor_queries;
    OutdoorCaptureEvidence outdoor_capture;
    FriendlyCasualtyEvidence friendly_casualties;
    std::map<std::uint32_t,const OutdoorCapturedEye*> outdoor_eyes;
    std::map<std::uint32_t,const MemberKnowledge*> outdoor_members;
    bool outdoor_cohort_expanded=false;
    std::shared_ptr<RoomExposureBatch> exposure;
    std::shared_ptr<const ExposureGeometryInput> exposure_geometry;
    std::uint64_t exposure_token=0;
    bool exposure_prepared=false;
    SideRegistryResult side_result;
    std::vector<CombatMissionIntel> mission_intel;
    bool mission_intel_ready=false;
    std::shared_ptr<const KnowledgeFrame> knowledge_frame;
    std::size_t knowledge_ticks_processed=0;
    std::size_t knowledge_ticks_quarantined=0,knowledge_ticks_stale=0;
    std::uint64_t knowledge_ack_sequence=0;
    std::uint64_t knowledge_identity_resets=0;
    std::set<std::uint32_t> reset_room_sides;
    bool knowledge_ready=false,knowledge_replay_complete=false;
    struct RoomClaim {ActorKey actor;std::uint32_t side=0;Order order;ActorKey care_target;Order authored;};
    std::map<std::uint32_t,RoomSession> staged_rooms;
    std::unordered_map<std::uint32_t,RoomClaim> room_claims;
    std::unordered_map<std::uint32_t,AcceptedCoverProof> cover_proofs;
    // Captured side membership includes men with no projected room claim.
    // Built on the worker; publication tests full generations without scans.
    std::unordered_map<std::uint32_t,std::unordered_map<std::uint32_t,ActorKey>> room_rosters;
    std::map<std::uint64_t,std::uint32_t> room_groups;
    std::vector<ProposedOrder> room_proposals;
    std::vector<RoomDispatchReceipt> room_results;
    bool rooms_ready=false;
    std::uint64_t care_assignment_sequence=0;
    std::uint64_t defense_assignment_sequence=0;
    std::uint64_t support_assignment_sequence=0;
    EngineServices services;
    std::shared_ptr<Worker> worker;
    std::vector<std::size_t> cohort;
    std::unordered_map<std::uint32_t,ActorKey> selected;
    std::size_t next_actor=0;
    bool complete=false,failed=false,ends_round=false;
    std::uint64_t cost_us=0;                    // worker time of every slice (read only after done)
};
// The man's current order is the plan's base order refreshed without new facts that matter (e.g. the bootstrap refresh at
// the end of a fire window): same assignment and the same look kind and target, so a plan computed on the base is still
// about the same man doing the same thing. A re-aim at a different target is an urgent response and supersedes the plan.
bool refreshed_only(const Work& w,ActorKey actor,std::uint64_t base_seq,const Order& now) {
    const Order* base=nullptr;
    for(const auto& o:w.current)if(o.actor==actor&&o.order_seq==base_seq){base=&o;break;}
    if(!base||now.order_seq<base->order_seq)return false;
    return now.look.kind==base->look.kind&&now.look.target==base->look.target&&
        now.kind==base->kind&&now.goal_node==base->goal_node&&distance_squared(now.goal_position,base->goal_position)==0&&
        now.pace==base->pace&&now.stance==base->stance&&now.role==base->role&&
        now.ownership_generation==base->ownership_generation&&now.restrictions_revision==base->restrictions_revision&&
        now.geometry_revision==base->geometry_revision&&now.level_epoch==base->level_epoch&&
        now.assignment.generation==base->assignment.generation&&now.assignment.sector==base->assignment.sector&&
        now.assignment.portal==base->assignment.portal&&now.assignment.slot==base->assignment.slot;
}
bool cancelled(const std::shared_ptr<Work>& w,const std::function<bool()>& queue) {
    return w->cancelled.load(std::memory_order_relaxed)||(queue&&queue());
}
bool committed_room_order(const Order&o){return (o.flags&order_committed_entry)||
    (o.kind==OrderKind::withdraw&&o.assignment.generation);}
// Room commitment protects coordinated movement from routine personal cover,
// not a verified emergency that the personal layer already gave priority.
// Check the exact resulting motion against the source-qualified personal receipt;
// an arbitrary urgent flag is insufficient to discard the team assignment.
bool emergency_room_override(const Order& o,const OutdoorDirectorResult& result){
    if(!(o.flags&order_urgent)||o.assignment.generation||o.kind!=OrderKind::cover)return false;
    for(const auto& side:result.sides)for(const auto& response:side.personal){
        if(response.status!=OutdoorPersonalStatus::ready||
            (response.reason!=OutdoorPersonalReason::grenade_escape&&response.reason!=OutdoorPersonalReason::grenade_wait))continue;
        const auto& p=response.proposal.order;
        if(p.actor==o.actor&&p.level_epoch==o.level_epoch&&p.snapshot_seq==o.snapshot_seq&&
            p.ownership_generation==o.ownership_generation&&p.restrictions_revision==o.restrictions_revision&&
            p.geometry_revision==o.geometry_revision&&p.kind==o.kind&&p.pace==o.pace&&p.stance==o.stance&&
            p.goal_node==o.goal_node&&distance_squared(p.goal_position,o.goal_position)==0&&
            p.flags==o.flags&&p.role==o.role)return true;
    }
    return false;
}
// Care is a team duty, but never a reason to stand inside a measured
// grenade/hit escape. Require the actual personal/solo result, not a flag alone.
bool emergency_care_override(const Order& o,const OutdoorDirectorResult& result){
    if(emergency_room_override(o,result))return true;
    auto same=[&](const Order& p){return p.actor==o.actor&&p.level_epoch==o.level_epoch&&
        p.snapshot_seq==o.snapshot_seq&&p.ownership_generation==o.ownership_generation&&
        p.restrictions_revision==o.restrictions_revision&&p.geometry_revision==o.geometry_revision&&
        p.kind==o.kind&&p.pace==o.pace&&p.role==o.role&&p.flags==o.flags&&
        p.goal_node==o.goal_node&&distance_squared(p.goal_position,o.goal_position)==0;};
    for(const auto& side:result.sides)for(const auto& response:side.personal){
        if(response.status!=OutdoorPersonalStatus::ready||response.proposal.order.actor!=o.actor)continue;
        if((response.reason==OutdoorPersonalReason::unseen_escape||response.reason==OutdoorPersonalReason::dogfight)&&
            same(response.proposal.order))return true;
        if(response.reason==OutdoorPersonalReason::escape_unavailable&&(o.flags&order_urgent))
            for(const auto& solo:side.solo)if(solo.changed&&
                (solo.reason==OutdoorSoloReason::seek_cover||solo.reason==OutdoorSoloReason::urgent_escape_pending)&&
                same(solo.proposal.order))return true;
    }
    return false;
}
bool prepare_knowledge(const std::shared_ptr<Work>& w,const std::function<bool()>& queue) {
    if(w->knowledge_replay_complete)return w->knowledge_ready;
    w->staged_knowledge=w->worker->knowledge->clone([&](){return cancelled(w,queue);});
    if(!w->staged_knowledge){w->cancelled.store(true,std::memory_order_relaxed);return false;}
    w->staged_sides=w->worker->side_registry->clone([&](){return cancelled(w,queue);});
    if(!w->staged_sides){w->cancelled.store(true,std::memory_order_relaxed);return false;}
    if(w->worker->lifecycle) {
        w->staged_lifecycle=w->worker->lifecycle->clone([&](){return cancelled(w,queue);});
        if(!w->staged_lifecycle){w->cancelled.store(true,std::memory_order_relaxed);return false;}
    }
    KnowledgeFrame latest_accepted;bool has_accepted=false;
    for(const auto& tick:w->knowledge_ticks) {
        if(cancelled(w,queue)){w->cancelled.store(true,std::memory_order_relaxed);return false;}
        // Reject malformed values BEFORE the provisional side ledger changes.
        // The acknowledged cursor includes quarantined ticks, independently of
        // the last accepted knowledge sequence. No current frame is invented.
        const auto rejection=w->staged_knowledge->validate_input(tick->world,[&](){return cancelled(w,queue);});
        if(rejection==KnowledgeReject::malformed_input||rejection==KnowledgeReject::stale_input) {
            if(rejection==KnowledgeReject::malformed_input)++w->knowledge_ticks_quarantined;
            else ++w->knowledge_ticks_stale;
            w->knowledge_ack_sequence=tick->world.sequence;continue;
        }
        if(rejection!=KnowledgeReject::none) {
            if(rejection==KnowledgeReject::cancelled)w->cancelled.store(true,std::memory_order_relaxed);
            else w->failed=true;
            return false;
        }
        auto observed=tick->world;
        auto sides=w->staged_sides->update(observed,w->worker->side_retain_ms,[&](){return cancelled(w,queue);},
            tick->friendly_casualties);
        if(!sides.accepted) {
            if(sides.status==SideRegistryStatus::invalid_input||sides.status==SideRegistryStatus::stale) {
                if(sides.status==SideRegistryStatus::invalid_input)++w->knowledge_ticks_quarantined;
                else ++w->knowledge_ticks_stale;
                w->knowledge_ack_sequence=observed.sequence;continue;
            }
            if(sides.cancelled)w->cancelled.store(true,std::memory_order_relaxed);else w->failed=true;
            return false;
        }
        observed.sides=sides.sides;
        for(const auto& reset:sides.resets) {
            const auto side=reset.side;
            // Only the reused side identity loses its old bus/member memory.
            // Unrelated units retain their own observations and pending reports.
            if(!w->staged_knowledge->reset_side(side,[&](){return cancelled(w,queue);})) {
                w->cancelled.store(true,std::memory_order_relaxed);return false;
            }
            ++w->knowledge_identity_resets;
            w->reset_room_sides.insert(side);
        }
        auto knowledge_context=tick->context;
        // Physical room observations and clear proofs come from a committed
        // worker, but are not recipient knowledge. K1 delivers the cells on
        // its ordinary radio/voice bus before any recipient may retire a foe.
        for(const auto& room:w->worker->rooms){
            const auto& session=room.second;
            const auto side=std::find_if(observed.sides.begin(),observed.sides.end(),[&](const SideSnapshot& s){return s.side_id==room.first;});
            if(side==observed.sides.end()||session.epoch!=observed.level_epoch||session.map!=observed.map_hash||
                session.geometry!=observed.geometry_revision||session.group!=side->group||session.faction!=side->faction||
                session.mission_revision!=side->mission.revision)continue;
            const auto& measured=session.doctrine.state();
            if(measured.accepted&&measured.level_epoch==observed.level_epoch&&measured.map_hash==observed.map_hash&&
                measured.geometry_revision==observed.geometry_revision&&measured.mission_revision==side->mission.revision&&
                measured.snapshot_seq<observed.sequence)
                for(const auto& cell:measured.room_cells)if(cell.observed_at_ms<=observed.now_ms)knowledge_context.room_cells.push_back(cell);
            for(const auto& proof:session.clear_proofs)if(proof.epoch==observed.level_epoch&&proof.map==observed.map_hash&&
                proof.geometry==observed.geometry_revision&&proof.mission==side->mission.revision&&
                proof.physical.at_ms<=observed.now_ms)knowledge_context.room_proofs.push_back(proof);
        }
        std::vector<ObservedCasualty> observed_casualties;
        observed_casualties.reserve(sides.friendly_casualties.size());
        for(const auto& f:sides.friendly_casualties){
            if(cancelled(w,queue)){w->cancelled.store(true,std::memory_order_relaxed);return false;}
            ObservedCasualty fact;fact.observer=f.observer;fact.patient=f.target;
            fact.level_epoch=observed.level_epoch;fact.snapshot_seq=observed.sequence;
            fact.observer_owner=f.observer_owner;fact.observer_restrictions=f.observer_restrictions;
            fact.side=f.side_id;fact.faction=f.faction;fact.at_ms=f.observed_at_ms;fact.position=f.position;
            fact.visible=f.visible_now;fact.friendly=f.friendly==Evidence::yes;fact.alive=f.alive;fact.wounded=f.wounded;
            observed_casualties.push_back(fact);
        }
        auto frame=w->staged_knowledge->update_disposable_observed(observed,observed_casualties,knowledge_context,w->services,[&](){return cancelled(w,queue);});
        if(!frame.accepted) {
            // Prevalidation already accepted the input. Failure after ledger
            // augmentation is not a quarantinable user tick; discard this clone.
            if(frame.cancelled)w->cancelled.store(true,std::memory_order_relaxed);else w->failed=true;
            return false;
        }
        if(w->staged_lifecycle) {
            auto signals=tick->lifecycle_signals;
            for(const auto& completion:w->worker->completions)
                if(completion.stamp.at_ms<=observed.now_ms)signals.completed.push_back(completion);
            auto life=w->staged_lifecycle->update(observed,sides,&frame,signals,[&](){return cancelled(w,queue);});
            if(!life.accepted) {
                if(life.cancelled)w->cancelled.store(true,std::memory_order_relaxed);else w->failed=true;
                return false;
            }
            if(observed.sequence==w->source.sequence)
                w->lifecycle_frame=std::make_shared<const SideLifecycleResult>(std::move(life));
        }
        ++w->knowledge_ticks_processed;
        w->replayed_sequences.push_back(observed.sequence);
        w->knowledge_ack_sequence=observed.sequence;
        if(observed.sequence==w->source.sequence) {
            w->source.sides=std::move(observed.sides);w->side_result=std::move(sides);
        }
        latest_accepted=std::move(frame);has_accepted=true;
    }
    if(cancelled(w,queue)){w->cancelled.store(true,std::memory_order_relaxed);return false;}
    if(has_accepted)w->knowledge_frame=std::make_shared<const KnowledgeFrame>(std::move(latest_accepted));
    w->knowledge_replay_complete=true;
    if(!w->knowledge_frame||w->knowledge_frame->sequence!=w->source.sequence){w->failed=true;return false;}
    w->context.knowledge=w->knowledge_frame;w->knowledge_ready=true;return true;
}
bool prepare_mission_intelligence(const std::shared_ptr<Work>& w,const Config& config,
    const std::function<bool()>& queue) {
    if(w->mission_intel_ready)return true;
    auto stop=[&](){return cancelled(w,queue);};
    auto batch=mission_intelligence_batch(config,w->source,w->knowledge_frame.get(),w->services,stop);
    if(batch.status==MissionIntelStatus::cancelled){w->cancelled.store(true,std::memory_order_relaxed);return false;}
    // Failed/missing static geometry does not revoke personal field combat.
    // This is sourced intelligence, never an automatically imposed mission.
    if(batch.status==MissionIntelStatus::ready)w->mission_intel=std::move(batch.sides);
    else w->mission_intel.assign(w->source.sides.size(),CombatMissionIntel{});
    w->mission_intel_ready=true;return true;
}
bool room_request(const WorldSnapshot& world,const SideSnapshot& side,const std::vector<SearchedRoom>& communicated,
    const EngineServices& services,const Config& config,RoomStackRequest& request,bool& inspect_current_room,
    const std::function<bool()>& stop,
    const RoomDoctrineResult* previous=nullptr,std::uint32_t care_pin_room=invalid_room) {
    if(side.mission.kind!=MissionKind::clear_building||side.mission.rooms.count>side.mission.rooms.items.size()||
        !services.buildings||services.buildings->level_epoch!=world.level_epoch||services.buildings->map_hash!=world.map_hash)return false;
    const Building* building=nullptr;
    for(const auto& b:services.buildings->buildings)if(b.id==side.mission.building){building=&b;break;}
    if(!building)return false;
    request.entry_portal=side.mission.entry_portal;
    for(const auto& searched:communicated)
        if(searched.side==side.side_id&&searched.building==building->id&&searched.at_ms<=world.now_ms)
            request.searched_rooms.push_back(searched.room);
    if(previous&&care_pin_room!=invalid_room&&previous->accepted&&previous->room==care_pin_room){
        request.target_room=care_pin_room;request.entry_portal=previous->requested_entry_portal;
        inspect_current_room=previous->interior_start;if(inspect_current_room)request.entry_portal=0;
        return true;
    }
    // A cohort already inside one measured, uncommunicated room must inspect
    // that room before treating its outgoing portal as a routine safe source.
    // No entry crossing or clearance is inferred from the starting position.
    std::uint32_t current_room=invalid_room;bool complete_cohort=true;std::size_t living=0;
    if(!services.buildings->locate)complete_cohort=false;
    for(const auto key:side.members){
        const auto actor=std::find_if(world.actors.begin(),world.actors.end(),[&](const ActorSnapshot& a){return a.key==key;});
        if(actor==world.actors.end()){complete_cohort=false;break;}
        if(!actor->alive||actor->wounded)continue;
        if(!(actor->flags&flag_admitted)||!actor->ownership_generation||!actor->restrictions_revision||
            actor->yield_reasons||!actor->current_position_accessible||!std::isfinite(actor->health)||actor->health<=0||
            !std::isfinite(actor->position.x)||!std::isfinite(actor->position.y)||!std::isfinite(actor->position.z)||
            actor->node_id==invalid_node||actor->snapshot_seq!=world.sequence||
            !actor->observed_at_ms||actor->observed_at_ms>world.now_ms||
            world.now_ms-actor->observed_at_ms>config.snapshot_max_age_ms||!services.buildings->locate){complete_cohort=false;break;}
        const auto place=services.buildings->locate(actor->node_id);
        const auto room=std::find_if(building->rooms.begin(),building->rooms.end(),[&](const Room& row){return row.id==place.second;});
        if(place.first!=building->id||room==building->rooms.end()||
            actor->position.x<room->min.x||actor->position.x>room->max.x||
            actor->position.z<room->min.z||actor->position.z>room->max.z||
            std::abs(actor->position.y-room->floor_y)>config.room.floor_tolerance_m||
            (!room->node_ids.empty()&&!std::binary_search(room->node_ids.begin(),room->node_ids.end(),actor->node_id))||
            (living&&current_room!=room->id)){complete_cohort=false;break;}
        current_room=room->id;++living;
    }
    const bool inspect_local=complete_cohort&&living&&current_room!=invalid_room&&
        std::find(request.searched_rooms.begin(),request.searched_rooms.end(),current_room)==request.searched_rooms.end();
    if(previous&&previous->accepted&&previous->level_epoch==world.level_epoch&&previous->snapshot_seq<world.sequence) {
        const bool previous_shared=previous->room!=invalid_room&&
            std::find(request.searched_rooms.begin(),request.searched_rooms.end(),previous->room)!=request.searched_rooms.end();
        if((!previous->ready_for_next||!previous_shared)&&previous->room!=invalid_room) {
            request.target_room=previous->room;
            request.entry_portal=previous->requested_entry_portal;
            // Once local inspection has started, an exiting/changed member
            // does not turn the unfinished sweep into an invented doorway FORM.
            inspect_current_room=previous->interior_start;
            if(inspect_current_room)request.entry_portal=0;
            return true;
        }
        // Doctrine explicitly permits next-room planning. Optional outgoing
        // assessment may overlap after a genuine clear, care-safe receipt.
        if(previous_shared)request.entry_portal=0;
    }
    if(inspect_local){request.target_room=current_room;request.entry_portal=0;inspect_current_room=true;return true;}
    std::sort(request.searched_rooms.begin(),request.searched_rooms.end());
    request.searched_rooms.erase(std::unique(request.searched_rooms.begin(),request.searched_rooms.end()),request.searched_rooms.end());
    auto cleared=[&](std::uint32_t room){return room==invalid_room||std::binary_search(request.searched_rooms.begin(),request.searched_rooms.end(),room);};
    if(side.mission.rooms.count) {
        for(std::size_t i=0;i<side.mission.rooms.count;++i)if(!cleared(side.mission.rooms.items[i])) {
            request.target_room=side.mission.rooms.items[i];return true;
        }
        return false;
    }
    Vec3 centre;std::size_t count=0;
    std::size_t searched=0;
    for(const auto& key:side.members)for(const auto& a:world.actors) {
        if((++searched&31u)==0&&stop&&stop())throw KnowledgeReplayCancelled{};
        if(a.key==key&&a.alive&&!a.wounded) {
            centre.x+=a.position.x;centre.y+=a.position.y;centre.z+=a.position.z;++count;break;
        }
    }
    if(!count)return false;
    centre.x/=float(count);centre.y/=float(count);centre.z/=float(count);
    const Portal* best=nullptr;float best_distance=0;
    for(const auto& p:building->portals) {
        if(stop&&stop())throw KnowledgeReplayCancelled{};
        if(!p.doorway||!p.traversable||cleared(p.room_a)==cleared(p.room_b))continue;
        if(request.entry_portal&&p.id!=request.entry_portal)continue;
        const float distance=distance_squared(centre,p.centre);if(!std::isfinite(distance))continue;
        if(!best||distance<best_distance||(distance==best_distance&&p.id<best->id)){best=&p;best_distance=distance;}
    }
    if(!best)return false;
    request.target_room=cleared(best->room_a)?best->room_b:best->room_a;return true;
}
bool known_room_foe(const Work& work,const SideSnapshot& side,std::uint32_t room_id,const Config& config,
    const std::function<bool()>& stop) {
    if(!work.knowledge_frame||!work.services.buildings)return false;
    const Room* target=nullptr;
    for(const auto& building:work.services.buildings->buildings)if(building.id==side.mission.building)
        for(const auto& room:building.rooms)if(room.id==room_id){target=&room;break;}
    if(!target)return false;
    for(const auto& member:work.knowledge_frame->members) {
        if(stop&&stop())throw KnowledgeReplayCancelled{};
        if(member.side!=side.side_id||std::find(side.members.begin(),side.members.end(),member.actor)==side.members.end())continue;
        for(const auto& foe:member.foes) {
            if(foe.down||foe.wounded)continue;
            const auto* fact=foe.last_known();if(!fact||!fresh(fact->at_ms,work.source.now_ms,config.room.known_enemy_ms))continue;
            const auto p=fact->position;
            if(finite(p)&&p.x>=target->min.x&&p.x<=target->max.x&&p.z>=target->min.z&&p.z<=target->max.z&&
                std::abs(p.y-target->floor_y)<=config.knowledge.eye_m+config.room.floor_tolerance_m)return true;
        }
    }
    return false;
}
bool stamp_care_assignment(Work& work,RoomSession& session,const SideSnapshot& side,
    const RoomDoctrineResult& primary,RoomCasualtyResult& care) {
    constexpr auto reserved=std::uint64_t(1)<<63;
    if(!care.accepted||!care.intent_revision||!valid_key(care.helper))return true;
    const auto helper=std::find_if(work.source.actors.begin(),work.source.actors.end(),[&](const ActorSnapshot& a){return a.key==care.helper;});
    auto selected=std::find_if(care.slots.begin(),care.slots.end(),[&](const RoomDoctrineSlot& s){return s.actor==care.helper;});
    if(helper==work.source.actors.end()||selected==care.slots.end())return false;
    // Raw primary assignments remain untouched in its receipt and on every
    // other actor. The reserved half is exclusively the composed role token.
    for(const auto& slot:primary.slots)if(slot.order.assignment.generation&reserved)return false;
    if(session.care_helper!=care.helper||session.care_target!=care.casualty||session.care_owner!=helper->ownership_generation||
        session.care_intent!=care.intent_revision||session.care_mission!=side.mission.revision||
        session.care_restrictions!=helper->restrictions_revision||session.care_geometry!=work.source.geometry_revision) {
        if(work.care_assignment_sequence>=reserved-1)return false;
        session.care_assignment=reserved|++work.care_assignment_sequence;
        session.care_helper=care.helper;session.care_target=care.casualty;session.care_owner=helper->ownership_generation;
        session.care_intent=care.intent_revision;session.care_mission=side.mission.revision;
        session.care_restrictions=helper->restrictions_revision;session.care_geometry=work.source.geometry_revision;
        session.care_executed=false;session.care_executed_at=0;
    }
    auto& slot=*selected;auto& order=slot.order;order.assignment.generation=session.care_assignment;
    slot.execution_confirmed=false;slot.applied_order_seq=0;
    // These flags describe the decorated movement, not an inherited door GO.
    slot.released=false;slot.crossed=false;slot.released_at_ms=0;
    slot.arrived=distance_squared(helper->position,order.goal_position)<=order.arrival_radius_m*order.arrival_radius_m;
    for(const auto& applied:work.executed)if(applied.actor==slot.actor&&applied.order_seq&&
        applied.order_seq==helper->last_applied.order_seq&&applied.ownership_generation==helper->ownership_generation&&
        applied.restrictions_revision==helper->restrictions_revision&&applied.level_epoch==work.source.level_epoch&&
        applied.geometry_revision==work.source.geometry_revision&&applied.issued_at_ms<=work.source.now_ms&&
        helper->last_applied.at_ms>=applied.issued_at_ms&&helper->last_applied.at_ms<=helper->observed_at_ms&&
        applied.kind==order.kind&&applied.role==order.role&&applied.aid==order.aid&&
        applied.goal_node==order.goal_node&&distance_squared(applied.goal_position,order.goal_position)<.0001f&&
        applied.assignment.generation==order.assignment.generation&&applied.assignment.portal==order.assignment.portal&&
        applied.assignment.sector==order.assignment.sector&&applied.assignment.slot==order.assignment.slot) {
        slot.execution_confirmed=true;slot.applied_order_seq=applied.order_seq;break;
    }
    return true;
}
WorldSnapshot care_capture(const Work& work,RoomSession& session,const SideSnapshot& side,
    const std::function<bool()>& stop) {
    WorldSnapshot captured;captured.level_epoch=work.source.level_epoch;captured.sequence=work.source.sequence;
    captured.geometry_revision=work.source.geometry_revision;captured.map_hash=work.source.map_hash;captured.now_ms=work.source.now_ms;
    // Copy only this side's captured roster. No hidden world injury lookup or
    // pointer escapes; the decorator needs no global actors/doors/side census.
    captured.actors.reserve(side.members.size());
    for(auto key:side.members) {
        if(stop&&stop())throw KnowledgeReplayCancelled{};
        const auto found=std::find_if(work.source.actors.begin(),work.source.actors.end(),[&](const ActorSnapshot& a){return a.key==key;});
        if(found==work.source.actors.end())continue;
        auto actor=*found;
        if(actor.key==session.care_helper) {
            if(actor.ownership_generation!=session.care_owner||session.care_mission!=side.mission.revision||
                session.care_restrictions!=actor.restrictions_revision||session.care_geometry!=captured.geometry_revision) {
                session.care_executed=false;session.care_executed_at=0;
            }
            if(!session.care_executed)for(const auto& applied:work.executed) {
                if(stop&&stop())throw KnowledgeReplayCancelled{};
                if(applied.actor==actor.key&&applied.ownership_generation==actor.ownership_generation&&
                    applied.restrictions_revision==actor.restrictions_revision&&applied.level_epoch==captured.level_epoch&&
                    applied.geometry_revision==captured.geometry_revision&&applied.aid==session.care_target&&valid_key(applied.aid)&&
                    applied.assignment.generation==session.care_assignment&&applied.kind==OrderKind::recover&&
                    (applied.role==roles::aid_casualty||applied.role==roles::casualty_guard)&&
                    applied.order_seq==actor.last_applied.order_seq&&actor.last_applied.at_ms>=applied.issued_at_ms&&
                    actor.last_applied.at_ms<=actor.observed_at_ms) {
                    session.care_executed=true;session.care_executed_at=actor.last_applied.at_ms;break;
                }
            }
        }
        if(actor.aid_state.phase>=2&&actor.aid_state.phase<=4&&
            (actor.key!=session.care_helper||!session.care_executed||actor.aid_state.target!=session.care_target||
             actor.aid_state.at_ms<session.care_executed_at))actor.aid_state={};
        captured.actors.push_back(std::move(actor));
    }
    return captured;
}
void observe_room_trail(Work& work,const WorldSnapshot& observed,RoomSession& session,const SideSnapshot& side,const Config& config,
    const std::function<bool()>& stop) {
    const auto* cache=work.services.buildings;
    if(!cache||!cache->locate||cache->level_epoch!=work.source.level_epoch||cache->map_hash!=work.source.map_hash||
        observed.level_epoch!=work.source.level_epoch||observed.geometry_revision!=work.source.geometry_revision||
        observed.map_hash!=work.source.map_hash||observed.sequence<=session.trail_sequence||observed.now_ms>work.source.now_ms)return;
    // Duration is existing withdrawal memory, not an actor or path-node cap.
    session.crumbs.erase(std::remove_if(session.crumbs.begin(),session.crumbs.end(),[&](const RoomPositionCrumb& p){
        if(stop())throw KnowledgeReplayCancelled{};
        return p.level_epoch!=work.source.level_epoch||p.geometry_revision!=work.source.geometry_revision||
            !fresh(p.at_ms,work.source.now_ms,config.room.withdraw_cooldown_ms)||
            std::find(side.members.begin(),side.members.end(),p.actor)==side.members.end();
    }),session.crumbs.end());
    for(const auto key:side.members) {
        if(stop())throw KnowledgeReplayCancelled{};
        const auto a=std::find_if(observed.actors.begin(),observed.actors.end(),[&](const ActorSnapshot& man){return man.key==key;});
        if(a==observed.actors.end()||!a->alive||a->wounded||a->node_id==invalid_node||a->faction!=side.faction||
            !finite(a->position)||a->observed_at_ms!=observed.now_ms)continue;
        bool same_side=side.group&&a->native_squad==side.group;
        if(!side.group)for(const auto& prior:observed.sides) {
            if(prior.side_id==side.side_id&&prior.faction==side.faction&&prior.group==side.group&&
                std::find(prior.members.begin(),prior.members.end(),key)!=prior.members.end()){same_side=true;break;}
        }
        if(!same_side)continue;
        const auto loc=cache->locate(a->node_id);
        if(loc.first!=side.mission.building||loc.second==invalid_room)continue;
        const auto last=std::find_if(session.crumbs.rbegin(),session.crumbs.rend(),[&](const RoomPositionCrumb& p){return p.actor==key;});
        if(last!=session.crumbs.rend()&&last->node==a->node_id&&last->building==loc.first&&last->room==loc.second)continue;
        session.crumbs.push_back({key,observed.now_ms,observed.level_epoch,observed.geometry_revision,a->node_id,loc.first,loc.second});
    }
    session.trail_sequence=observed.sequence;
}
std::map<std::uint32_t,std::vector<ActorKey>> defense_room_cohorts(const Work& work,RoomSession& session,
    const SideSnapshot& side,const Config& config,const std::function<bool()>& stop) {
    std::map<std::uint32_t,std::vector<ActorKey>> cohorts;
    const auto* cache=work.services.buildings;
    if(!cache||!cache->locate||cache->level_epoch!=work.source.level_epoch||cache->map_hash!=work.source.map_hash||
        side.mission.rooms.count>side.mission.rooms.items.size())return cohorts;
    const Building* building=nullptr;
    for(const auto& b:cache->buildings)if(b.id==side.mission.building){building=&b;break;}
    if(!building)return cohorts;
    const auto permitted=[&](std::uint32_t room){const auto end=side.mission.rooms.items.begin()+side.mission.rooms.count;
        return !side.mission.rooms.count||std::find(side.mission.rooms.items.begin(),end,room)!=end;};
    for(auto it=session.defense_homes.begin();it!=session.defense_homes.end();){
        if(stop())throw KnowledgeReplayCancelled{};
        const auto& home=it->second;
        const auto a=std::find_if(work.source.actors.begin(),work.source.actors.end(),[&](const ActorSnapshot& man){return man.key==home.actor;});
        if(std::find(side.members.begin(),side.members.end(),home.actor)==side.members.end()||a==work.source.actors.end()||
            a->ownership_generation!=home.owner||!permitted(home.room))it=session.defense_homes.erase(it);else ++it;
    }
    for(const auto key:side.members) {
        if(stop())throw KnowledgeReplayCancelled{};
        const auto a=std::find_if(work.source.actors.begin(),work.source.actors.end(),[&](const ActorSnapshot& man){return man.key==key;});
        if(a==work.source.actors.end()||!a->alive)continue;
        auto home=session.defense_homes.find(key.id);
        if(home==session.defense_homes.end()){
            if(a->wounded||!(a->flags&flag_admitted)||!a->ownership_generation||a->node_id==invalid_node||!finite(a->position)||
                a->level_epoch!=work.source.level_epoch||a->snapshot_seq!=work.source.sequence||
                !fresh(a->observed_at_ms,work.source.now_ms,config.snapshot_max_age_ms))continue;
            const auto loc=cache->locate(a->node_id);
            if(loc.first!=side.mission.building||loc.second==invalid_room||!permitted(loc.second))continue;
            const auto room=std::find_if(building->rooms.begin(),building->rooms.end(),[&](const Room& r){return r.id==loc.second;});
            // The native cache has no semantic stair tag. Only a measured
            // nondegenerate floor footprint can become an initial home.
            if(room==building->rooms.end()||!finite(room->min)||!finite(room->max)||!std::isfinite(room->floor_y)||
                !(room->max.x>room->min.x)||!(room->max.z>room->min.z)||
                a->position.x<room->min.x||a->position.x>room->max.x||a->position.z<room->min.z||a->position.z>room->max.z||
                std::abs(a->position.y-room->floor_y)>config.room.floor_tolerance_m)continue;
            home=session.defense_homes.emplace(key.id,RoomSession::DefenseHome{key,a->ownership_generation,loc.second}).first;
        }
        cohorts[home->second.room].push_back(key);
    }
    // Observed home stays through local combat/aid and doorway crossings. A
    // casualty removes only that man's demand, never retargets the whole side.
    for(auto it=session.defense_rooms.begin();it!=session.defense_rooms.end();)
        if(!cohorts.count(it->first))it=session.defense_rooms.erase(it);else ++it;
    return cohorts;
}
// Fresh role input built from accepted movement, not a re-stamped historical
// clearance receipt. It has no new arrival, view or execution proof.
RoomDoctrineResult role_base(const Work& work,const SideSnapshot& side,std::uint32_t room,const Config& config,
    const std::vector<SearchedRoom>& cleared,const std::function<bool()>& stop) {
    RoomDoctrineResult out;out.accepted=true;out.level_epoch=work.source.level_epoch;out.snapshot_seq=work.source.sequence;
    out.captured_at_ms=work.source.now_ms;out.room=room;
    for(const auto& r:cleared)if(r.side==side.side_id&&r.building==side.mission.building&&r.at_ms<=work.source.now_ms)out.cleared_rooms.push_back(r.room);
    for(auto key:side.members) {
        if(stop())throw KnowledgeReplayCancelled{};
        const auto a=std::find_if(work.source.actors.begin(),work.source.actors.end(),[&](const ActorSnapshot& man){return man.key==key;});
        const auto held=std::find_if(work.current.begin(),work.current.end(),[&](const Order& o){return o.actor==key;});
        if(a==work.source.actors.end()||held==work.current.end()||held->ownership_generation!=a->ownership_generation||
            held->restrictions_revision!=a->restrictions_revision||held->geometry_revision!=work.source.geometry_revision)continue;
        auto immediate=first_response(*a,config,work.source.now_ms,work.source.level_epoch,work.source.geometry_revision,0);
        if(immediate.kind==OrderKind::yield)continue;
        // A completed/cancelled helper returns to his current safe response.
        // Never carry yesterday's aid command into the undecorated role base.
        auto order=valid_key(held->aid)||held->expires_at_ms<=work.source.now_ms?immediate:*held;
        order.snapshot_seq=work.source.sequence;order.issued_at_ms=work.source.now_ms;
        order.order_seq=0;
        order.fire=immediate.fire;
        if(immediate.look.kind==LookKind::enemy)order.look=immediate.look;
        else if(order.look.kind==LookKind::enemy)order.look=immediate.look;
        out.slots.push_back({key,order});
    }
    return out;
}
bool stamp_defense_assignments(Work& work,RoomSession& session,RoomDefenseResult& result,
    const SideSnapshot& side,const std::function<bool()>& stop) {
    constexpr auto domain=std::uint64_t(1)<<62;
    for(auto& order:result.orders) {
        if(stop())throw KnowledgeReplayCancelled{};
        const auto assignment=std::find_if(result.assignments.begin(),result.assignments.end(),[&](const DefenseAssignment& a){return a.actor==order.actor;});
        // Preserved base orders (including an existing aid/throw) keep identity.
        if(assignment==result.assignments.end()||assignment->node==invalid_node)continue;
        auto& token=session.defense_tokens[order.actor.id];
        if(token.actor!=order.actor||token.owner!=order.ownership_generation||token.restrictions!=order.restrictions_revision||
            token.geometry!=work.source.geometry_revision||token.mission!=side.mission.revision||token.intent!=assignment->intent_revision||
            token.room!=assignment->room) {
            if(work.defense_assignment_sequence>=domain-1)return false;
            token={order.actor,order.ownership_generation,order.restrictions_revision,work.source.geometry_revision,
                assignment->intent_revision,domain|++work.defense_assignment_sequence,side.mission.revision,assignment->room};
        }
        order.assignment.generation=token.token;
    }
    for(auto it=session.defense_tokens.begin();it!=session.defense_tokens.end();) {
        if(std::find(side.members.begin(),side.members.end(),it->second.actor)==side.members.end())it=session.defense_tokens.erase(it);else ++it;
    }
    return true;
}
void restore_support_parent(RoomSession& session,RoomDoctrineResult& base) {
    const auto& token=session.support_token;
    if(!token.token)return;
    for(auto& slot:base.slots) {
        const auto current=slot.order;
        if(slot.actor!=token.actor||current.assignment.generation!=token.token||
            current.ownership_generation!=token.owner||current.restrictions_revision!=token.restrictions||
            current.geometry_revision!=token.geometry)continue;
        // A fresh role input copied from an accepted support order needs its
        // underlying plan identity, not its own final decorator token. Only a
        // still-live matching accepted order reaches here; role_base already
        // discards expired commands. No old execution or arrival proof survives.
        slot=token.parent;
        slot.order.snapshot_seq=current.snapshot_seq;slot.order.issued_at_ms=current.issued_at_ms;
        slot.order.expires_at_ms=current.expires_at_ms;slot.order.order_seq=0;
        slot.order.fire=current.fire;
        slot.execution_confirmed=false;slot.applied_order_seq=0;slot.arrived=false;
        slot.released=false;slot.crossed=false;slot.released_at_ms=0;
    }
}
bool support_changes_role(const RoomSupportResult& result) {
    if(!result.accepted||result.pending||!result.intent_revision||!valid_key(result.actor))return false;
    return result.phase==SupportPhase::answering||result.phase==SupportPhase::reaching||result.phase==SupportPhase::finishing;
}
bool stamp_support_assignment(Work& work,RoomSession& session,const SideSnapshot& side,
    const std::vector<RoomDoctrineSlot>& parents,RoomSupportResult& result) {
    constexpr auto domain=std::uint64_t(1)<<61;
    if(!support_changes_role(result))return true;
    const auto parent=std::find_if(parents.begin(),parents.end(),[&](const RoomDoctrineSlot& s){return s.actor==result.actor;});
    auto selected=std::find_if(result.slots.begin(),result.slots.end(),[&](const RoomDoctrineSlot& s){return s.actor==result.actor;});
    if(parent==parents.end()||selected==result.slots.end())return false;
    auto& order=selected->order;
    if(order.role!=roles::finisher&&order.role!=roles::buddy_answer&&order.role!=roles::cover_hit_dir)return false;
    auto& token=session.support_token;
    if(token.actor!=order.actor||token.target!=result.target||token.owner!=order.ownership_generation||
        token.restrictions!=order.restrictions_revision||token.geometry!=work.source.geometry_revision||
        token.mission!=side.mission.revision||token.intent!=result.intent_revision) {
        if(work.support_assignment_sequence>=domain-1)return false;
        token.actor=order.actor;token.target=result.target;token.owner=order.ownership_generation;
        token.restrictions=order.restrictions_revision;token.geometry=work.source.geometry_revision;
        token.mission=side.mission.revision;token.intent=result.intent_revision;token.token=domain|++work.support_assignment_sequence;
    }
    token.parent=*parent;order.assignment.generation=token.token;
    selected->execution_confirmed=false;selected->applied_order_seq=0;
    selected->arrived=false;selected->released=false;selected->crossed=false;selected->released_at_ms=0;
    return true;
}
void propose_room_slots(Work& work,const SideSnapshot& side,const std::vector<RoomDoctrineSlot>& slots,
    const RoomCasualtyResult& care,const Config& config,const std::function<bool()>& stop) {
    const auto group=(std::uint64_t(1)<<63)|side.side_id;
    work.room_groups[group]=side.side_id;auto& roster=work.room_rosters[side.side_id];
    for(const auto key:side.members){if(stop())throw KnowledgeReplayCancelled{};roster.emplace(key.id,key);}
    for(const auto& slot:slots) {
        if(stop())throw KnowledgeReplayCancelled{};
        const auto a=std::find_if(work.source.actors.begin(),work.source.actors.end(),[&](const ActorSnapshot& man){return man.key==slot.actor;});
        const auto old=std::find_if(work.current.begin(),work.current.end(),[&](const Order& o){return o.actor==slot.actor;});
        if(a==work.source.actors.end()||old==work.current.end()||old->ownership_generation!=a->ownership_generation)continue;
        auto immediate=first_response(*a,config,work.source.now_ms,work.source.level_epoch,work.source.geometry_revision,0);
        if(immediate.kind==OrderKind::yield)continue;
        ProposedOrder proposal;proposal.order=slot.order;proposal.expected_previous_order_seq=old->order_seq;proposal.group_id=group;
        if(immediate.kind==OrderKind::reload)proposal.order.fire={};
        if(immediate.look.kind==LookKind::enemy&&!proposal.order.throw_order.valid&&!valid_key(proposal.order.aid)&&
            !keep_room_cover_fire(proposal.order,immediate,*a,config,work.source.now_ms)) {
            proposal.order.look=immediate.look;proposal.order.fire=immediate.fire;
        }
        const auto care_target=care.accepted&&care.intent_revision&&slot.actor==care.helper?care.casualty:ActorKey{};
        auto authored=slot.order;
        if(slot.authored_look_valid)authored.look=slot.authored_look;
        work.room_claims[slot.actor.id]={slot.actor,side.side_id,proposal.order,care_target,authored};
        work.room_proposals.push_back(std::move(proposal));
    }
}
bool prepare_rooms(const std::shared_ptr<Work>& w,const Config& config,const std::function<bool()>& queue) {
    if(w->rooms_ready)return true;
    auto stop=[&](){return cancelled(w,queue);};
    w->staged_cooperation=w->worker->cooperation;
    w->cooperation_results=w->staged_cooperation.update(w->source,config.snapshot_max_age_ms,stop);
    if(stop()){w->cancelled.store(true,std::memory_order_relaxed);return false;}
    for(auto& side:w->source.sides)for(const auto& link:w->cooperation_results)if(!link.approaching){
        if(side.side_id==link.first){side.cooperation_side=link.second;side.link_up=link.second_anchor;}
        else if(side.side_id==link.second){side.cooperation_side=link.first;side.link_up=link.first_anchor;}
    }
    w->care_assignment_sequence=w->worker->care_assignment_sequence;
    w->defense_assignment_sequence=w->worker->defense_assignment_sequence;
    w->support_assignment_sequence=w->worker->support_assignment_sequence;
    for(const auto& old:w->worker->rooms) {
        if(stop()){w->cancelled.store(true,std::memory_order_relaxed);return false;}
        if(w->reset_room_sides.count(old.first))continue;
        w->staged_rooms.emplace(old);
    }
    std::set<std::uint32_t> active;
    for(const auto& side:w->source.sides) {
        if(stop()){w->cancelled.store(true,std::memory_order_relaxed);return false;}
        const bool clearing=side.mission.kind==MissionKind::clear_building;
        if(!clearing&&side.mission.kind!=MissionKind::defend&&side.mission.kind!=MissionKind::hold)continue;
        active.insert(side.side_id);
        RoomExposureCostView exposure_view;
        const auto* exposure=w->exposure&&w->exposure->view(w->exposure_token,side.side_id,exposure_view)?&exposure_view:nullptr;
        auto room=w->staged_rooms.try_emplace(side.side_id,config);
        auto& session=room.first->second;
        const auto data=w->services.buildings?w->services.buildings->data_version:0;
        if(session.epoch!=w->source.level_epoch||session.map!=w->source.map_hash||session.geometry!=w->source.geometry_revision||
            session.building_data!=data||session.faction!=side.faction||session.group!=side.group) {
            session=RoomSession(config);session.epoch=w->source.level_epoch;session.map=w->source.map_hash;
            session.geometry=w->source.geometry_revision;session.building_data=data;session.faction=side.faction;session.group=side.group;
        }
        const bool same_mission=session.mission_revision==side.mission.revision&&session.building==side.mission.building&&session.mission_kind==side.mission.kind;
        if(!same_mission){session.crumbs.clear();session.searched.clear();session.clear_proofs.clear();session.defense_tokens.clear();session.defense.reset();
            session.defense_homes.clear();session.defense_rooms.clear();session.defense_cursor=0;
            session.support.reset();session.support_token={};session.cover_fire.reset();session.care_pin_room=invalid_room;}
        // Once actual survivors of two distinct depleted squads meet, each
        // defensive squad re-bases its own posts at its current physical room.
        // No member or mission transfers to the other side.
        if(std::any_of(w->cooperation_results.begin(),w->cooperation_results.end(),[&](const SurvivorCooperation& c){
            return c.newly_confirmed&&(c.first==side.side_id||c.second==side.side_id);})){
            session.defense_homes.clear();session.defense_rooms.clear();session.defense_tokens.clear();session.defense.reset();
        }
        session.mission_revision=side.mission.revision;session.building=side.mission.building;session.mission_kind=side.mission.kind;
        auto remember_searched=[&](const SearchedRoom& fact) {
            if(fact.side!=side.side_id||fact.building!=side.mission.building||fact.at_ms>w->source.now_ms)return;
            auto prior=std::find_if(session.searched.begin(),session.searched.end(),[&](const SearchedRoom& r){return r.room==fact.room;});
            if(prior==session.searched.end())session.searched.push_back(fact);
            else if(fact.at_ms>prior->at_ms)*prior=fact;
        };
        // Capture-frequency observations survive a slow planning job. Read
        // only ticks accepted by K1 validation, never quarantined replay data.
        for(const auto& tick:w->knowledge_ticks) {
            if(stop())throw KnowledgeReplayCancelled{};
            if(std::binary_search(w->replayed_sequences.begin(),w->replayed_sequences.end(),tick->world.sequence))
                observe_room_trail(*w,tick->world,session,side,config,stop);
        }
        observe_room_trail(*w,w->source,session,side,config,stop);
        RoomDoctrineRequest request;RoomDoctrineResult result;
        request.exposure=exposure;
        RoomDoctrineResult base;bool historical=false;
        std::map<std::uint32_t,std::vector<ActorKey>> defense_cohorts;
        std::vector<std::uint32_t> defended_rooms;
        // A physical proof whose only source disappeared before communication
        // cannot become safe by waiting. Re-enter the room doctrine and earn
        // a new physical inspection instead of pinning an impossible receipt.
        if(clearing&&!session.clear_proofs.empty()&&w->knowledge_frame){
            const auto latest=std::max_element(session.clear_proofs.begin(),session.clear_proofs.end(),[](const RoomClearProof& a,const RoomClearProof& b){return
                a.physical.at_ms<b.physical.at_ms;});
            const auto& proof=*latest;std::size_t living=0,informed=0;
            for(const auto key:side.members){
                const auto actor=std::find_if(w->source.actors.begin(),w->source.actors.end(),[&](const ActorSnapshot& a){return a.key==key;});
                if(actor==w->source.actors.end()||!actor->alive||actor->wounded||!(actor->flags&flag_admitted))continue;
                ++living;
                const auto mind=std::find_if(w->knowledge_frame->members.begin(),w->knowledge_frame->members.end(),[&](const MemberKnowledge& m){return
                    m.actor==key&&m.side==side.side_id&&m.ownership_generation==actor->ownership_generation;});
                if(mind!=w->knowledge_frame->members.end()&&std::any_of(mind->communicated_rooms.begin(),mind->communicated_rooms.end(),[&](const SearchedRoom& r){return
                    r.side==proof.physical.side&&r.building==proof.physical.building&&r.room==proof.physical.room&&r.at_ms==proof.physical.at_ms;}))++informed;
            }
            const bool stranded=living>0&&informed<(std::min)(std::size_t(2),living)&&std::any_of(proof.cells.begin(),proof.cells.end(),[&](const RoomCellFact& cell){
                const auto source=std::find_if(w->source.actors.begin(),w->source.actors.end(),[&](const ActorSnapshot& a){return a.key==cell.observer;});
                const bool source_available=source!=w->source.actors.end()&&source->alive&&!source->wounded&&
                    source->ownership_generation==cell.observer_owner&&(source->flags&flag_admitted)&&
                    std::find(side.members.begin(),side.members.end(),cell.observer)!=side.members.end();
                if(source_available)return false;
                // A current living holder can relay an already delivered,
                // source-qualified cell even after its original reporter dies.
                return std::none_of(w->knowledge_frame->members.begin(),w->knowledge_frame->members.end(),[&](const MemberKnowledge& holder){
                    const auto actor=std::find_if(w->source.actors.begin(),w->source.actors.end(),[&](const ActorSnapshot& a){return a.key==holder.actor;});
                    if(actor==w->source.actors.end()||!actor->alive||actor->wounded||!(actor->flags&flag_admitted)||
                        holder.side!=side.side_id||holder.ownership_generation!=actor->ownership_generation||
                        std::find(side.members.begin(),side.members.end(),holder.actor)==side.members.end())return false;
                    return std::any_of(holder.room_cells.begin(),holder.room_cells.end(),[&](const RoomCellFact& known){return
                        known.side==cell.side&&known.building==cell.building&&known.room==cell.room&&
                        known.mission_revision==cell.mission_revision&&known.level_epoch==cell.level_epoch&&
                        known.map_hash==cell.map_hash&&known.geometry_revision==cell.geometry_revision&&
                        known.attempt==cell.attempt&&known.sample==cell.sample&&known.observer==cell.observer&&
                        known.observer_owner==cell.observer_owner&&known.observed_at_ms>=cell.observed_at_ms;});});});
            // Let a pre-death packet finish the ordinary K1 bus latency. This
            // grace grants no clearance; it only avoids pointless reinspection.
            if(stranded&&w->source.now_ms>=proof.physical.at_ms&&w->source.now_ms-proof.physical.at_ms>=2000){
                const auto lost_room=proof.physical.room;session.clear_proofs.erase(std::remove_if(session.clear_proofs.begin(),session.clear_proofs.end(),[&](const RoomClearProof& p){return p.physical.room==lost_room;}),session.clear_proofs.end());
                session.searched.erase(std::remove_if(session.searched.begin(),session.searched.end(),[&](const SearchedRoom& p){return p.room==lost_room;}),session.searched.end());
                session.doctrine.reset();}
        }
        SideSnapshot operational=side;
        std::vector<SearchedRoom> communicated;
        if(clearing&&!session.clear_proofs.empty()&&w->knowledge_frame){
            const auto latest=std::max_element(session.clear_proofs.begin(),session.clear_proofs.end(),[](const RoomClearProof& a,const RoomClearProof& b){return
                a.physical.at_ms<b.physical.at_ms;});
            std::vector<ActorKey> informed;std::size_t living=0;
            for(const auto key:side.members){if(stop())throw KnowledgeReplayCancelled{};
                const auto actor=std::find_if(w->source.actors.begin(),w->source.actors.end(),[&](const ActorSnapshot& a){return a.key==key;});
                if(actor==w->source.actors.end()||!actor->alive||actor->wounded||!(actor->flags&flag_admitted))continue;
                ++living;
                const auto mind=std::find_if(w->knowledge_frame->members.begin(),w->knowledge_frame->members.end(),[&](const MemberKnowledge& m){return
                    m.actor==key&&m.side==side.side_id&&m.ownership_generation==actor->ownership_generation;});
                if(mind!=w->knowledge_frame->members.end()&&std::any_of(mind->communicated_rooms.begin(),mind->communicated_rooms.end(),[&](const SearchedRoom& room){return
                    room.side==side.side_id&&room.building==side.mission.building&&room.room==latest->physical.room&&room.at_ms==latest->physical.at_ms;}))informed.push_back(key);
            }
            if(!informed.empty()&&informed.size()>=(std::min)(std::size_t(2),living)){
                operational.members=informed;
                for(const auto& proof:session.clear_proofs){if(stop())throw KnowledgeReplayCancelled{};bool every=true;
                    for(const auto key:informed){const auto mind=std::find_if(w->knowledge_frame->members.begin(),w->knowledge_frame->members.end(),[&](const MemberKnowledge& m){return m.actor==key;});
                        every=every&&mind!=w->knowledge_frame->members.end()&&
                            std::any_of(mind->communicated_rooms.begin(),mind->communicated_rooms.end(),[&](const SearchedRoom& room){return room.side==proof.physical.side&&
                                room.building==proof.physical.building&&room.room==proof.physical.room&&room.at_ms==proof.physical.at_ms;});}
                    if(every)communicated.push_back(proof.physical);}
            }
        }
        if(clearing) {
            if(room_request(w->source,operational,communicated,w->services,config,request.stack,request.inspect_current_room,stop,
                same_mission?&session.doctrine.state():nullptr,same_mission?session.care_pin_room:invalid_room)) {
                if(request.inspect_current_room){
                    const auto building=std::find_if(w->services.buildings->buildings.begin(),w->services.buildings->buildings.end(),[&](const Building& b){return b.id==side.mission.building;});
                    const Room* current=nullptr;if(building!=w->services.buildings->buildings.end())for(const auto& room:building->rooms)if(room.id==request.stack.target_room){current=&room;break;}
                    operational.members.erase(std::remove_if(operational.members.begin(),operational.members.end(),[&](ActorKey key){
                        const auto a=std::find_if(w->source.actors.begin(),w->source.actors.end(),[&](const ActorSnapshot& row){return row.key==key;});
                        if(!current||a==w->source.actors.end()||!a->alive||a->wounded||!(a->flags&flag_admitted)||
                            !a->ownership_generation||!a->restrictions_revision||a->yield_reasons||
                            !a->current_position_accessible||!std::isfinite(a->health)||a->health<=0||
                            !std::isfinite(a->position.x)||!std::isfinite(a->position.y)||!std::isfinite(a->position.z)||
                            a->node_id==invalid_node||a->snapshot_seq!=w->source.sequence||!a->observed_at_ms||
                            a->observed_at_ms>w->source.now_ms||w->source.now_ms-a->observed_at_ms>config.snapshot_max_age_ms||
                            !w->services.buildings->locate)return true;
                        const auto place=w->services.buildings->locate(a->node_id);
                        return place!=std::make_pair(building->id,current->id)||
                            a->position.x<current->min.x||a->position.x>current->max.x||
                            a->position.z<current->min.z||a->position.z>current->max.z||
                            std::abs(a->position.y-current->floor_y)>config.room.floor_tolerance_m||
                            (!current->node_ids.empty()&&!std::binary_search(current->node_ids.begin(),current->node_ids.end(),a->node_id));
                    }),operational.members.end());
                }
                request.applied_orders=w->executed;
                request.sight_clear_nodes_per_tick=config.cover_queries_per_slice;
                request.geometry_nodes_per_update=config.cover_queries_per_slice;
                request.permit_frag=known_room_foe(*w,side,request.stack.target_room,config,stop);
                if(request.inspect_current_room&&operational.members.empty()){
                    result=session.doctrine.state();historical=true;
                    base=role_base(*w,side,result.room,config,communicated,stop);
                    for(auto& slot:base.slots){const auto a=std::find_if(w->source.actors.begin(),w->source.actors.end(),[&](const ActorSnapshot& row){return row.key==slot.actor;});
                        if(a==w->source.actors.end())continue;
                        auto safe=first_response(*a,config,w->source.now_ms,w->source.level_epoch,w->source.geometry_revision,0);
                        if(safe.kind!=OrderKind::yield){safe.kind=OrderKind::hold;safe.role=roles::hold;safe.goal_position=a->position;
                            safe.goal_node=a->node_id;safe.pace=Pace::stand;safe.assignment={};slot.order=safe;}}
                }else result=session.doctrine.update(w->source,operational,request,w->knowledge_frame.get(),w->services,stop);
                if(result.accepted&&result.phase==RoomPhase::search&&result.reason==RoomReason::fresh_contact){
                    session.clear_proofs.erase(std::remove_if(session.clear_proofs.begin(),session.clear_proofs.end(),[&](const RoomClearProof& p){return p.physical.room==result.room;}),session.clear_proofs.end());
                    session.searched.erase(std::remove_if(session.searched.begin(),session.searched.end(),[&](const SearchedRoom& p){return p.room==result.room;}),session.searched.end());
                }
                if(result.accepted)for(const auto id:result.cleared_rooms) {
                    const auto known=std::find_if(session.searched.begin(),session.searched.end(),[&](const SearchedRoom& r){return r.room==id;});
                    if(known==session.searched.end())remember_searched({side.side_id,side.mission.building,id,result.captured_at_ms});
                }
                if(result.accepted&&std::any_of(result.events.begin(),result.events.end(),[&](const Event& e){return
                    e.kind==events::room_cleared&&e.room==result.room;})){
                    const auto expected=result.reason==RoomReason::sight_clear?result.costs.sight_nodes:result.costs.total;
                    std::set<std::uint64_t> samples;bool proved=expected&&result.clear_proof.size()==expected;
                    for(const auto& cell:result.clear_proof){if(stop())throw KnowledgeReplayCancelled{};
                        proved=proved&&cell.side==side.side_id&&cell.building==side.mission.building&&cell.room==result.room&&
                            cell.mission_revision==side.mission.revision&&cell.level_epoch==w->source.level_epoch&&
                            cell.map_hash==w->source.map_hash&&cell.geometry_revision==w->source.geometry_revision&&
                            cell.attempt==result.generation&&valid_key(cell.observer)&&cell.observer_owner&&
                            cell.observed_at_ms&&cell.observed_at_ms<=result.captured_at_ms&&samples.insert(cell.sample).second;}
                    if(proved){RoomClearProof proof;proof.physical={side.side_id,side.mission.building,result.room,result.captured_at_ms};
                        proof.epoch=result.level_epoch;proof.map=result.map_hash;proof.geometry=result.geometry_revision;
                        proof.mission=result.mission_revision;proof.attempt=result.generation;proof.cells=result.clear_proof;
                        auto prior=std::find_if(session.clear_proofs.begin(),session.clear_proofs.end(),[&](const RoomClearProof& p){return p.physical.room==result.room;});
                        if(prior==session.clear_proofs.end())session.clear_proofs.push_back(std::move(proof));else *prior=std::move(proof);}
                }
                if(!historical)base=result;
            }else if(same_mission&&session.doctrine.state().accepted) {
                result=session.doctrine.state();historical=true;
                base=role_base(*w,operational,result.room,config,communicated,stop);
            }else continue;
        }else {
            defense_cohorts=defense_room_cohorts(*w,session,side,config,stop);
            if(defense_cohorts.empty())continue;
            for(const auto& cohort:defense_cohorts)defended_rooms.push_back(cohort.first);
            // No single room stands for this building-wide role base. Held
            // rooms are not searched-room evidence or clearance receipts.
            base=role_base(*w,side,invalid_room,config,session.searched,stop);
        }
        const auto& room_side=clearing?operational:side;
        if(stop()){w->cancelled.store(true,std::memory_order_relaxed);return false;}
        if(!base.accepted)continue;
        if(clearing&&operational.members.size()<side.members.size())for(const auto key:side.members){
            if(std::find(operational.members.begin(),operational.members.end(),key)!=operational.members.end())continue;
            const auto actor=std::find_if(w->source.actors.begin(),w->source.actors.end(),[&](const ActorSnapshot& a){return a.key==key;});
            if(actor==w->source.actors.end()||!actor->alive||actor->wounded||!(actor->flags&flag_admitted))continue;
            auto order=first_response(*actor,config,w->source.now_ms,w->source.level_epoch,w->source.geometry_revision,0);
            if(order.kind==OrderKind::yield)continue;
            if(order.kind==OrderKind::hold){order.role=roles::hold;order.goal_position=actor->position;
                order.goal_node=actor->node_id;order.pace=Pace::stand;order.assignment={};}
            base.slots.push_back({key,order});
        }
        restore_support_parent(session,base);
        const auto observed_care=care_capture(*w,session,side,stop);
        auto staged_casualty=session.casualty;
        auto care=staged_casualty.update(observed_care,side,base,w->services,stop,w->side_result.friendly_casualties,
            clearing?communicated:session.searched);
        if(stop()||care.cancelled){w->cancelled.store(true,std::memory_order_relaxed);return false;}
        const bool care_refused=!stamp_care_assignment(*w,session,side,base,care);
        if(care_refused){care.accepted=false;care.reason=CasualtyReason::invalid_input;care.slots=base.slots;}
        else if(care.accepted)session.casualty=std::move(staged_casualty);
        session.care_pin_room=care.accepted&&clearing&&
            (care.stay_with_casualty||care.phase==CasualtyPhase::reaching||care.phase==CasualtyPhase::guard||
             care.phase==CasualtyPhase::working)?base.room:invalid_room;
        auto final_slots=care.accepted?care.slots:base.slots;
        RoomDefenseRequest defense_request;defense_request.revision=side.mission.revision;
        defense_request.exposure=exposure;
        defense_request.building=side.mission.building;defense_request.room=base.room;
        defense_request.mode=clearing?DefenseMode::assessed_withdraw:
            side.mission.kind==MissionKind::defend?DefenseMode::defend:DefenseMode::hold;
        defense_request.crumbs=session.crumbs;defense_request.cleared=clearing?communicated:session.searched;
        for(const auto& slot:final_slots)defense_request.base_orders.push_back(slot.order);
        defense_request.max_work=config.cover_queries_per_slice;
        RoomDefenseResult defense;
        bool defense_refused=false;
        auto apply_defense=[&](RoomDefenseResult& part){
            if(!part.accepted||part.resume)return;
            if(!stamp_defense_assignments(*w,session,part,room_side,stop)){defense_refused=true;return;}
            std::vector<RoomDoctrineSlot> previous_slots;
            if(clearing){previous_slots=std::move(final_slots);final_slots.clear();}
            for(const auto& order:part.orders){
                auto slot=std::find_if(final_slots.begin(),final_slots.end(),[&](const RoomDoctrineSlot& s){return s.actor==order.actor;});
                if(slot==final_slots.end()){
                    const auto old=std::find_if(previous_slots.begin(),previous_slots.end(),[&](const RoomDoctrineSlot& s){return s.actor==order.actor;});
                    final_slots.push_back(old==previous_slots.end()?RoomDoctrineSlot{}:*old);slot=final_slots.end()-1;
                }
                slot->actor=order.actor;slot->order=order;
                const auto assignment=std::find_if(part.assignments.begin(),part.assignments.end(),[&](const DefenseAssignment& a){return a.actor==order.actor;});
                if(assignment!=part.assignments.end()&&assignment->node!=invalid_node){
                    slot->authored_look_valid=assignment->fallback_look_valid;slot->authored_look=assignment->fallback_look;
                }
            }
        };
        if(clearing){
            defense=session.defense.update(w->source,room_side,defense_request,w->knowledge_frame.get(),w->services,stop);
            if(stop()||defense.cancelled){w->cancelled.store(true,std::memory_order_relaxed);return false;}
            apply_defense(defense);
        }else{
            defense.accepted=true;defense.level_epoch=w->source.level_epoch;defense.snapshot_seq=w->source.sequence;
            defense.geometry_revision=w->source.geometry_revision;defense.at_ms=w->source.now_ms;
            defense.phase=side.mission.kind==MissionKind::hold?DefensePhase::holding:DefensePhase::defending;
            defense.reason=side.mission.kind==MissionKind::hold?DefenseReason::hold:DefenseReason::defend;
            std::size_t left=config.cover_queries_per_slice;
            const auto count=defended_rooms.size(),start=count?session.defense_cursor%count:0;
            for(std::size_t i=0;i<count;++i){
                const auto target=defended_rooms[(start+i)%count];
                defense_request.room=target;defense_request.restrict_participants=true;defense_request.participants=defense_cohorts.at(target);
                defense_request.max_work=left/(count-i)+(left%(count-i)!=0);
                defense_request.base_orders.clear();for(const auto& slot:final_slots)defense_request.base_orders.push_back(slot.order);
                auto& planner=session.defense_rooms.try_emplace(target,config).first->second;
                auto part=planner.update(w->source,side,defense_request,w->knowledge_frame.get(),w->services,stop);
                if(stop()||part.cancelled){w->cancelled.store(true,std::memory_order_relaxed);return false;}
                if(part.work_used>defense_request.max_work)throw std::runtime_error("defense placement budget exceeded");
                left-=part.work_used;defense.work_used+=part.work_used;defense.partial|=part.partial||!part.accepted;
                apply_defense(part);
                defense.assignments.insert(defense.assignments.end(),part.assignments.begin(),part.assignments.end());
                defense.transitions.insert(defense.transitions.end(),part.transitions.begin(),part.transitions.end());
                defense.exposure.quotes+=part.exposure.quotes;defense.exposure.fallbacks+=part.exposure.fallbacks;
                defense.exposure.used+=part.exposure.used;defense.exposure.enqueued+=part.exposure.enqueued;defense.exposure.held+=part.exposure.held;
                if(i==0)defense.assessment=part.assessment;
            }
            if(count)session.defense_cursor=(start+1)%count;
            for(const auto& slot:final_slots)defense.orders.push_back(slot.order);
        }
        auto covering_base=base;covering_base.slots=final_slots;
        RoomCoverFireRequest covering_request;covering_request.applied_orders=w->executed;
        covering_request.query_quantum=config.cover_queries_per_slice;
        auto covering=session.cover_fire.update(w->source,side,covering_base,w->knowledge_frame.get(),w->services,
            covering_request,stop);
        if(stop()||covering.cancelled){w->cancelled.store(true,std::memory_order_relaxed);return false;}
        if(covering.accepted&&!covering.pending)final_slots=covering.slots;
        auto support_base=base;support_base.slots=final_slots;
        auto support=session.support.update(w->source,side,support_base,w->knowledge_frame.get(),w->services,
            config.cover_queries_per_slice,stop,clearing?nullptr:&defended_rooms);
        if(stop()||support.cancelled){w->cancelled.store(true,std::memory_order_relaxed);return false;}
        const bool support_refused=!stamp_support_assignment(*w,session,side,final_slots,support);
        if(support.accepted&&!support_refused)final_slots=support.slots;
        // A completed primary mission retains its original historical receipt.
        // Only an ongoing care/defense role needs further room proposals here;
        // C1 owns the eventual normal mission handback.
        if(!historical||operational.members.size()<side.members.size()||valid_key(care.helper)||
            valid_key(care.security)||care.stay_with_casualty||
            (!defense.assignments.empty()&&!defense.resume)||support_changes_role(support)||
            (covering.accepted&&!covering.pending&&valid_key(covering.actor)))
            propose_room_slots(*w,side,final_slots,care,config,stop);
        auto formation=result.formation;
        const bool care_executed=session.care_executed&&care.helper==session.care_helper&&care.casualty==session.care_target;
        const auto receipt_seq=historical?result.snapshot_seq:w->source.sequence;
        w->room_results.push_back({side.side_id,side.mission.revision,receipt_seq,std::move(formation),std::move(result),std::move(care),care_refused,
            care_executed,care_executed?session.care_assignment:0,care_executed?session.care_executed_at:0,std::move(defense),defense_refused,
            std::move(support),support_refused,std::move(covering)});
    }
    for(auto it=w->staged_rooms.begin();it!=w->staged_rooms.end();) {
        if(!active.count(it->first))it=w->staged_rooms.erase(it);else ++it;
    }
    w->rooms_ready=true;return true;
}
void run_slice(const std::shared_ptr<Work>& w,Config config,DispatchLimits limits,
    const std::function<bool()>& queue_cancelled) noexcept
{
    const auto started=std::chrono::steady_clock::now();
    try {
        const bool known=prepare_knowledge(w,queue_cancelled)&&prepare_mission_intelligence(w,config,queue_cancelled);
        if(known&&w->exposure){
            auto stop=[&](){return cancelled(w,queue_cancelled);};
            ExposureWorkQuota quota{w->worker->exposure_work,w->worker->exposure_queries,0,0};
            if(!w->exposure_prepared&&!w->exposure_geometry)
                w->exposure_geometry=w->exposure->prepare_geometry(w->source,w->services,quota,stop);
            if(!w->exposure_prepared&&w->exposure_geometry){
                w->exposure_token=w->exposure->begin(w->source,*w->knowledge_frame,w->exposure_geometry,w->outdoor_capture.eyes,w->services,stop,w->reset_room_sides);
                w->exposure_prepared=true;
                w->exposure_geometry.reset(); // cache intake copied its charged geometry
            }
            w->exposure->advance(w->exposure_token,w->services,quota,stop);
        }
        const bool ready=known&&prepare_rooms(w,config,queue_cancelled);
        if(ready&&w->worker->outdoor&&!w->outdoor_cohort_expanded){
            // A mover cohort also needs the actual covering members of its
            // side. Observe those dependencies under the SAME per-slice budget;
            // never invent a set base just because it was outside this cohort.
            std::set<std::uint32_t> sides;std::unordered_map<std::uint32_t,std::size_t> indices;
            for(std::size_t i=0;i<w->source.actors.size();++i){if(cancelled(w,queue_cancelled))throw KnowledgeReplayCancelled{};indices.emplace(w->source.actors[i].key.id,i);}
            for(const auto&s:w->source.sides)for(auto actor:s.members){if(cancelled(w,queue_cancelled))throw KnowledgeReplayCancelled{};
                auto selected=w->selected.find(actor.id);if(selected!=w->selected.end()&&selected->second==actor){sides.insert(s.side_id);break;}}
            for(const auto&s:w->source.sides)if(sides.count(s.side_id))for(auto actor:s.members){
                if(cancelled(w,queue_cancelled))throw KnowledgeReplayCancelled{};auto i=indices.find(actor.id);
                if(i==indices.end()||w->selected.count(actor.id))continue;const auto&a=w->source.actors[i->second];
                if(a.key!=actor||!a.alive||a.wounded||a.yield_reasons||!(a.flags&flag_admitted))continue;
                if(std::none_of(w->current.begin(),w->current.end(),[&](const Order&o){return o.actor==actor&&o.ownership_generation==a.ownership_generation;}))continue;
                w->cohort.push_back(i->second);w->selected.emplace(actor.id,actor);
            }
            w->outdoor_cohort_expanded=true;
            for(const auto&member:w->knowledge_frame->members){if(cancelled(w,queue_cancelled))throw KnowledgeReplayCancelled{};
                auto inserted=w->outdoor_members.emplace(member.actor.id,&member);if(!inserted.second)inserted.first->second=nullptr;}
            for(const auto&eye:w->outdoor_capture.eyes){if(cancelled(w,queue_cancelled))throw KnowledgeReplayCancelled{};
                auto inserted=w->outdoor_eyes.emplace(eye.actor.id,&eye);if(!inserted.second)inserted.first->second=nullptr;}
        }
        std::size_t used=0;
        OutdoorSensorQuota line_quota{w->worker->outdoor_line_queries,0};
        while(ready&&w->next_actor<w->cohort.size()&&used<limits.cover_queries_per_slice) {
            if(cancelled(w,queue_cancelled))break;
            const auto& a=w->source.actors[w->cohort[w->next_actor++]];++used;
            const auto room_claim=w->room_claims.find(a.key.id);
            if(room_claim!=w->room_claims.end()&&(!w->worker->outdoor_personal||committed_room_order(room_claim->second.order)))continue;
            if(!a.alive||a.wounded||a.yield_reasons||!(a.flags&flag_admitted)||
                !w->services.cover_query||!config.cover_candidates)continue;
            KnowledgeTrace threat;
            bool urgent=false;
            if(w->worker->outdoor_personal){const auto member=w->outdoor_members.find(a.key.id);
                if(member!=w->outdoor_members.end()&&member->second)
                    urgent=outdoor_personal_cover_focus(config,w->source,a,*member->second,threat);
            }
            if(!urgent&&!tactical_knowledge_focus(a,w->source,*w->knowledge_frame,threat))continue;
            CoverQueryInput input;input.centre=a.position;input.radius=config.cover_radius;
            input.max_n=config.cover_candidates;input.threats.count=1;input.threats.items[0]=threat.position;
            input.heights.count=3;input.heights.items[0]=.7f;input.heights.items[1]=1.1f;input.heights.items[2]=1.7f;
            ActorCoverCache cache;cache.actor=a.key;cache.level_epoch=w->source.level_epoch;
            cache.snapshot_seq=w->source.sequence;cache.ownership_generation=a.ownership_generation;
            cache.restrictions_revision=a.restrictions_revision;
            cache.origin_node=a.node_id;cache.observed_at_ms=w->source.now_ms;
            cache.knowledge_threat=threat.foe;cache.knowledge_observed_at_ms=threat.observed_at_ms;
            cache.knowledge_threat_position=threat.position;cache.knowledge_unidentified=threat.unidentified;
            if(w->worker->outdoor){
                OutdoorCoverProfile profile;auto&source=profile.source;source.actor=a.key;
                source.level_epoch=w->source.level_epoch;source.snapshot_seq=w->source.sequence;source.geometry_revision=w->source.geometry_revision;
                source.ownership_generation=a.ownership_generation;source.restrictions_revision=a.restrictions_revision;
                source.at_ms=w->source.now_ms;source.origin_node=a.node_id;source.actor_position=a.position;
                source.focus=threat.foe;source.focus_at_ms=threat.observed_at_ms;source.focus_position=threat.position;
                profile.heights={{input.heights.items[0],input.heights.items[1],input.heights.items[2]}};
                w->outdoor_queries.profiles.push_back(profile);
                const auto eye=w->outdoor_eyes.find(a.key.id);
                if(eye!=w->outdoor_eyes.end()&&eye->second){
                    auto line=measure_outdoor_line(w->source,a,*eye->second,threat,w->services,line_quota,config.outdoor.p.solo.combat.target_height_m,[&](){return cancelled(w,queue_cancelled);});
                    if(valid_key(line.source.actor))w->outdoor_queries.lines.push_back(std::move(line));
                }
            }
            cache.spots.resize(config.cover_candidates);
            const auto count=w->services.cover_query(input,cache.spots.data(),config.cover_candidates);
            if(count>cache.spots.size()){w->failed=true;break;}
            cache.spots.resize(count);
            for(auto& spot:cache.spots) {
                // Associate the returned geometry with the query's actual
                // observed threat. Never manufacture access/path evidence.
                if(spot.threat!=0){spot.protection_known=Evidence::unknown;continue;}
                spot.threat_key=threat.target;spot.threat_observed_at_ms=threat.observed_at_ms;
                // Query evidence retains the restrictions revision it actually
                // checked. The proposal's revision is in ActorCoverCache.
            }
            w->context.cover.push_back(std::move(cache));
        }
        if(ready&&!w->failed&&!cancelled(w,queue_cancelled)&&w->next_actor==w->cohort.size()) {
            w->result=w->worker->squads.update(w->source,w->current,w->context,
                [&](){return cancelled(w,queue_cancelled);});
            if(w->result.accepted) {
                w->result.proposals.erase(std::remove_if(w->result.proposals.begin(),w->result.proposals.end(),[&](const ProposedOrder& p){
                    return w->room_claims.count(p.order.actor.id)!=0;
                }),w->result.proposals.end());
                w->result.proposals.insert(w->result.proposals.end(),w->room_proposals.begin(),w->room_proposals.end());
                if(w->worker->outdoor){
                    w->staged_outdoor=std::make_unique<CombatOutdoorDirector>(*w->worker->outdoor);
                    for(auto side:w->reset_room_sides)w->staged_outdoor->reset_side(side);
                    // Existing explicit receipts remain usable when no worker
                    // eye query was possible. A new UNKNOWN ray never inherits
                    // an older/externally supplied successful result.
                    std::set<std::uint32_t> measured;
                    for(const auto&line:w->outdoor_queries.lines)measured.insert(line.source.actor.id);
                    for(const auto&line:w->outdoor_capture.lines)if(!measured.count(line.source.actor.id))w->outdoor_queries.lines.push_back(line);
                    w->outdoor_queries.search_observations=w->outdoor_capture.search_observations;
                    std::vector<ActorKey> publication_cohort;publication_cohort.reserve(w->selected.size());
                    for(const auto& selected:w->selected)publication_cohort.push_back(selected.second);
                    w->outdoor_result=w->staged_outdoor->update(w->source,*w->knowledge_frame,w->mission_intel,w->result,
                        w->current,w->executed,w->context.cover,w->outdoor_queries,w->outdoor_capture,[&](){return cancelled(w,queue_cancelled);},w->services,&publication_cohort);
                    if(w->outdoor_result.status==OutdoorStatus::cancelled)throw KnowledgeReplayCancelled{};
                    if(w->outdoor_result.status==OutdoorStatus::ready){
                        std::unordered_map<std::uint32_t,const ProposedOrder*> overrides;
                        for(const auto&proposal:w->outdoor_result.overrides){
                            const auto claim=w->room_claims.find(proposal.order.actor.id);
                            const bool claimed=claim!=w->room_claims.end()&&claim->second.actor==proposal.order.actor;
                            // The personal layer may rename an uncommitted room
                            // role to individual. That does not release its team
                            // obligation or authorize an unrelated squad push.
                            if(claimed&&proposal.order.role==roles::suppression_push)continue;
                            if(claimed){const auto& prior=claim->second;
                                const bool care=valid_key(prior.care_target)||prior.order.role==roles::care_security||
                                    prior.order.role==roles::aid_casualty||prior.order.role==roles::casualty_guard;
                                if(care&&proposal.order.kind!=OrderKind::reload&&proposal.order.kind!=OrderKind::yield&&
                                    !emergency_care_override(proposal.order,w->outdoor_result))continue;
                            }
                            if(!claimed||!committed_room_order(claim->second.order)||
                                emergency_room_override(proposal.order,w->outdoor_result))overrides.emplace(proposal.order.actor.id,&proposal);
                        }
                        for(auto&proposal:w->result.proposals){auto replacement=overrides.find(proposal.order.actor.id);
                            if(replacement!=overrides.end()&&replacement->second->order.actor==proposal.order.actor){
                                proposal=*replacement->second;
                                // Personal combat temporarily owns this man's
                                // order. Do not attach the room's former look/
                                // occupancy permissions to the unassigned order.
                                if(!proposal.order.assignment.generation)w->room_claims.erase(proposal.order.actor.id);
                            }}
                        for(const auto& item:overrides){const auto& proposal=*item.second;const auto& o=proposal.order;
                            if(o.role!=roles::finisher||!valid_key(o.finish)||std::any_of(w->result.proposals.begin(),w->result.proposals.end(),
                                [&](const ProposedOrder& p){return p.order.actor==o.actor;}))continue;
                            const auto a=std::find_if(w->source.actors.begin(),w->source.actors.end(),[&](const ActorSnapshot& x){return x.key==o.actor;});
                            const auto prior=std::find_if(w->current.begin(),w->current.end(),[&](const Order& x){return x.actor==o.actor;});
                            if(a!=w->source.actors.end()&&prior!=w->current.end()&&a->ownership_generation==o.ownership_generation&&
                                prior->order_seq==proposal.expected_previous_order_seq&&prior->ownership_generation==o.ownership_generation)
                                w->result.proposals.push_back(proposal);
                        }
                        std::stable_partition(w->result.proposals.begin(),w->result.proposals.end(),[](const ProposedOrder&p){return p.order.role==roles::base_of_fire;});
                    }
                }
                // A proposed survivor link can issue quiet native regroup legs
                // only through current projected cells and actor route proofs.
                // Actual close arrival remains a separate confirmation gate.
                if(!w->cooperation_results.empty()&&w->outdoor_capture.route_service.query){
                    const auto total=w->outdoor_capture.route_service.work_per_update;
                    const auto available=total>w->outdoor_result.route_work?total-w->outdoor_result.route_work:0;
                    RouteProofBudget budget{(std::min)(std::size_t(64),available),0};
                    auto meeting=survivor_approach_orders(w->source,*w->knowledge_frame,w->cooperation_results,
                        w->result.proposals,w->current,config,w->services,w->outdoor_capture.route_service,budget,
                        [&](){return cancelled(w,queue_cancelled);});
                    if(cancelled(w,queue_cancelled))throw KnowledgeReplayCancelled{};
                    w->outdoor_result.route_work+=budget.work_used;
                    for(const auto& next:meeting){
                        const auto claim=w->room_claims.find(next.order.actor.id);
                        if(claim!=w->room_claims.end()){
                            const auto& hold=claim->second.order;
                            if(committed_room_order(hold)||hold.kind!=OrderKind::hold_door||
                                (hold.role!=roles::hold&&hold.role!=roles::barricade_guard)||hold.fire.allowed)continue;
                        }
                        auto found=std::find_if(w->result.proposals.begin(),w->result.proposals.end(),[&](const ProposedOrder& row){return row.order.actor==next.order.actor;});
                        if(found!=w->result.proposals.end()){*found=next;if(claim!=w->room_claims.end())w->room_claims.erase(claim);}
                    }
                }
                // Combat aid is a final role overlay for outdoor AND room sides.
                // It uses the same observed patients/native treatment executor,
                // never pauses the entire squad because a friendly went down.
                std::size_t aid_route_work=0;
                if(!w->side_result.friendly_casualties.empty()&&w->outdoor_capture.route_service.query){
                    CombatAidBudget budget;budget.work_left=config.outdoor.p.solo_work_per_update;
                    budget.queries_left=config.outdoor.p.solo_queries_per_update;
                    const auto route_total=w->outdoor_capture.route_service.work_per_update;
                    budget.routes.work_left=route_total>w->outdoor_result.route_work?route_total-w->outdoor_result.route_work:0;
                    for(const auto& side:w->source.sides){
                        const bool room_care=std::any_of(w->room_results.begin(),w->room_results.end(),[&](const auto& r){
                            return r.side_id==side.side_id&&r.casualty.accepted&&(r.casualty.stay_with_casualty||
                                r.casualty.phase==CasualtyPhase::reaching||r.casualty.phase==CasualtyPhase::guard||
                                r.casualty.phase==CasualtyPhase::working);});
                        if(room_care)continue;
                        auto aid=combat_aid_orders(w->source,side,*w->knowledge_frame,w->side_result.friendly_casualties,
                            w->result.proposals,w->current,config,w->services,&w->outdoor_capture.route_service,budget,
                            w->care_assignment_sequence,[&](){return cancelled(w,queue_cancelled);});
                        if(aid.cancelled)throw KnowledgeReplayCancelled{};
                        for(const auto& proposal:aid.proposals){
                            const auto found=std::find_if(w->result.proposals.begin(),w->result.proposals.end(),
                                [&](const ProposedOrder& p){return p.order.actor==proposal.order.actor;});
                            if(found==w->result.proposals.end())w->result.proposals.push_back(proposal);
                            else *found=proposal;
                            auto& roster=w->room_rosters[side.side_id];for(auto key:side.members)roster.emplace(key.id,key);
                            w->room_claims[proposal.order.actor.id]={proposal.order.actor,side.side_id,proposal.order,proposal.order.aid,proposal.order};
                        }
                    }
                    aid_route_work=budget.routes.work_used;
                }
                // Compose a safe pair lane only after personal and aid overrides. Keep
                // room claims as the authority for publication/spacing; their authored
                // order remains the fallback on target or role change.
                RouteProofBudget lane_routes;
                const auto route_total=w->outdoor_capture.route_service.work_per_update;
                const auto spent=w->outdoor_result.route_work+aid_route_work;
                lane_routes.work_left=route_total>spent?route_total-spent:0;
                for(const auto& side:w->source.sides){
                    auto lanes=compose_fire_lanes(w->source,side,w->result.proposals,config,w->services,
                        &w->outdoor_capture.route_service,lane_routes,[&](){return cancelled(w,queue_cancelled);});
                    if(lanes.cancelled)throw KnowledgeReplayCancelled{};
                    for(const auto& change:lanes.changes){
                        const auto claim=w->room_claims.find(change.actor.id);
                        if(claim==w->room_claims.end()||claim->second.actor!=change.actor||claim->second.side!=side.side_id)continue;
                        const auto order=std::find_if(w->result.proposals.begin(),w->result.proposals.end(),
                            [&](const ProposedOrder& p){return p.order.actor==change.actor;});
                        if(order!=w->result.proposals.end())claim->second.order=order->order;
                    }
                }
                for(const auto& proposal:w->result.proposals) {
                    if(cancelled(w,queue_cancelled))throw KnowledgeReplayCancelled{};
                    if(!proposal.has_cover_evidence||!proposal.has_knowledge)continue;
                    AcceptedCoverProof proof;proof.spot=proposal.cover_evidence;
                    proof.threat_position=proposal.knowledge.position;proof.snapshot_seq=w->source.sequence;
                    if(proposal.knowledge.foe.ghost()&&proposal.knowledge.observer==proposal.order.actor&&
                        (proposal.order.role==roles::under_fire_cover||proposal.order.role==roles::reload_cover)){
                        const auto member=w->outdoor_members.find(proposal.order.actor.id);
                        const auto actor=std::lower_bound(w->source.actors.begin(),w->source.actors.end(),proposal.order.actor.id,
                            [](const ActorSnapshot& a,std::uint32_t id){return a.key.id<id;});
                        if(member!=w->outdoor_members.end()&&member->second&&member->second->actor==proposal.order.actor&&
                            actor!=w->source.actors.end()&&actor->key==proposal.order.actor){
                            const auto&hit=member->second->hit_from;
                            if(hit.valid&&hit.observer==actor->key&&hit.shooter==proposal.knowledge.foe&&
                                hit.at_ms==proposal.knowledge.observed_at_ms&&distance_squared(hit.point,proposal.knowledge.position)==0)
                                for(std::size_t n=0;n<actor->hits.count&&n<actor->hits.items.size();++n)
                                    if(actor->hits.items[n].at_ms==hit.at_ms&&finite(actor->hits.items[n].dir)){
                                        proof.has_attributed_hit=true;proof.attributed_hit=actor->hits.items[n];break;
                                    }
                        }
                    }
                    proof.observed_at_ms=w->source.now_ms;
                    w->cover_proofs.emplace(proposal.order.actor.id,proof);
                }
            }
            w->complete=true;
        }
        if(cancelled(w,queue_cancelled))w->cancelled.store(true,std::memory_order_relaxed);
    }catch(const KnowledgeReplayCancelled&){w->cancelled.store(true,std::memory_order_relaxed);}
    catch(...){w->failed=true;}
    w->cost_us+=std::uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-started).count());
    w->done.store(true,std::memory_order_release);
}
}
struct CombatDispatcher::Impl {
    struct Admission {ActorKey key;std::uint64_t owner=0;TimeMs at=0;bool applied=false,failed=false,closed=false;};
    Config config;DispatchLimits limits;Runtime runtime;OrderMailbox mailbox;EngineServices services;
    KnowledgeInbox inbox;std::shared_ptr<const KnowledgeFrame> last_knowledge;
    std::shared_ptr<const SideLifecyclePolicy> lifecycle_policy;
    std::shared_ptr<const OutdoorDispatchPolicy> outdoor_policy;
    std::shared_ptr<const RoomExposureDispatchPolicy> exposure_policy;
    std::shared_ptr<const ExposureGeometryInput> exposure_geometry;
    OutdoorCaptureEvidence outdoor_capture;
    FriendlyCasualtyEvidence friendly_casualties;
    std::vector<OutdoorDispatchReceipt> last_outdoor;
    struct SuppressionReceipt {SuppressedFoeFact fact;std::uint64_t epoch=0,owner=0;std::uint32_t faction=0;};
    std::vector<SuppressionReceipt> pending_suppression;
    std::shared_ptr<const SideLifecycleResult> last_lifecycle;
    std::vector<RoomDispatchReceipt> last_rooms;
    std::shared_ptr<const SideDispatchReceipt> last_sides;
    CaptureAuthority authority;
    CombatMissionHandover missions;
    std::shared_ptr<Worker> worker;
    std::shared_ptr<Work> work;
    std::unordered_map<std::uint32_t,Order> accepted;
    // Exact accepted values indexed by sequence. A capture can acknowledge an
    // older execution while a newer refreshed order has already been published.
    // Retain only one order lifetime plus capture age; never infer execution
    // from the latest assignment merely having the same destination.
    std::unordered_map<std::uint64_t,Order> execution_ledger;
    std::deque<std::pair<TimeMs,std::uint64_t>> execution_expiry;
    std::unordered_map<std::uint32_t,Admission> admission;
    std::uint32_t after_actor_id=invalid_id;
    std::uint64_t last_scheduled_snapshot=0;
    TimeMs next_plan=0;
    bool replan_due=false;                      // a plan was disturbed (hit, target flip): next cohort without the rest
    DispatchStats stats;
    Impl(Config c,DispatchLimits l,std::shared_ptr<const SideLifecyclePolicy> policy,std::shared_ptr<const OutdoorDispatchPolicy> outdoor,
        std::shared_ptr<const RoomExposureDispatchPolicy> exposure):config(c),limits(l),runtime(c),mailbox(mailbox_limits(c)),inbox(l.knowledge_history_ticks),
        lifecycle_policy(policy?std::make_shared<const SideLifecyclePolicy>(*policy):nullptr),
        outdoor_policy(outdoor?std::make_shared<const OutdoorDispatchPolicy>(*outdoor):nullptr),
        exposure_policy(exposure?std::make_shared<const RoomExposureDispatchPolicy>(*exposure):nullptr),
        worker(std::make_shared<Worker>(c,lifecycle_policy,outdoor_policy,exposure_policy)) {
        if(!limits.bootstrap_work_per_tick)limits.bootstrap_work_per_tick=config.bootstrap_work_per_tick;
        if(!limits.cover_queries_per_slice)limits.cover_queries_per_slice=config.cover_queries_per_slice;
        if(!limits.plan_interval_ms)limits.plan_interval_ms=config.plan_interval_ms;
        limits.bootstrap_work_per_tick=(std::max)(std::size_t(1),limits.bootstrap_work_per_tick);
        limits.cover_queries_per_slice=(std::max)(std::size_t(1),limits.cover_queries_per_slice);
        limits.plan_interval_ms=(std::max)(TimeMs(1),limits.plan_interval_ms);
    }
    void clear() {
        missions.reset();
        if(work)work->cancelled.store(true,std::memory_order_relaxed);
        work.reset();worker=std::make_shared<Worker>(config,lifecycle_policy,outdoor_policy,exposure_policy);services={};
        accepted.clear();execution_ledger.clear();execution_expiry.clear();admission.clear();next_plan=0;replan_due=false;
        after_actor_id=invalid_id;last_scheduled_snapshot=0;
        runtime.reset();mailbox.reset(0);stats={};
        inbox.reset();last_knowledge.reset();last_rooms.clear();last_sides.reset();last_lifecycle.reset();distance_round.clear();distance_enrolled.clear();
        last_outdoor.clear();outdoor_capture={};friendly_casualties={};pending_suppression.clear();
        exposure_geometry.reset();
        authority={};
    }
    std::vector<Order> orders() const {
        std::vector<Order> out;out.reserve(accepted.size());for(const auto& kv:accepted)out.push_back(kv.second);
        std::sort(out.begin(),out.end(),[](const Order& a,const Order& b){return a.actor.id<b.actor.id;});return out;
    }
    std::deque<ActorKey> distance_round;std::set<std::pair<std::uint32_t,std::uint64_t>> distance_enrolled;
    bool begin_cohort(const WorldSnapshot& source) {
        auto w=std::make_shared<Work>();w->source=source;w->current=orders();w->friendly_casualties=friendly_casualties;
        // Route proofs are also consumed by survivor cooperation and combat aid
        // for room defenders, even when the outdoor director is disabled.
        w->outdoor_capture=outdoor_capture;
        w->exposure=worker->exposure;w->exposure_geometry=exposure_geometry;
        for(const auto& actor:source.actors) {
            const auto receipt=execution_ledger.find(actor.last_applied.order_seq);
            if(receipt!=execution_ledger.end()&&receipt->second.actor==actor.key&&
                receipt->second.ownership_generation==actor.ownership_generation)
                w->executed.push_back(receipt->second);
        }
        w->knowledge_ticks=inbox.pending_through(source.sequence);
        w->services=services;w->worker=worker;
        const auto n=source.actors.size();if(!n)return false;
        // Distance is a scheduling cost only. Drain each enrolled man once; newly
        // admitted men join its tail, and near fights cannot repeatedly jump it.
        auto eligible=[&](const ActorSnapshot& a){const auto held=accepted.find(a.key.id);return held!=accepted.end()&&
            held->second.actor==a.key&&held->second.ownership_generation==a.ownership_generation&&a.alive&&!a.wounded&&
            !a.yield_reasons&&(a.flags&flag_admitted);};
        if(distance_round.empty())distance_enrolled.clear();
        {
            std::map<std::pair<std::uint32_t,std::uint64_t>,float> costs;
            if(last_lifecycle&&last_sides&&last_lifecycle->accepted&&last_lifecycle->level_epoch==source.level_epoch&&
                fresh(last_lifecycle->captured_at_ms,source.now_ms,config.snapshot_max_age_ms))
                for(const auto& life:last_lifecycle->sides)if(life.distance_known)
                    for(const auto& side:last_sides->registry.sides)if(side.side_id==life.identity.side&&side.group==life.identity.group&&side.faction==life.identity.faction)
                        for(const auto& key:side.members)costs[{key.id,key.generation}]=life.distance_cost;
            std::vector<const ActorSnapshot*> ranked;for(const auto& a:source.actors)if(eligible(a)&&!distance_enrolled.count({a.key.id,a.key.generation}))ranked.push_back(&a);
            auto urgent=[&](const ActorSnapshot& a){for(std::size_t i=0;i<a.hits.count&&i<a.hits.items.size();++i)
                if(fresh(a.hits.items[i].at_ms,source.now_ms,config.knowledge.uf_escape_ms))return true;return false;};
            std::stable_sort(ranked.begin(),ranked.end(),[&](const auto* a,const auto* b){
                if(urgent(*a)!=urgent(*b))return urgent(*a);
                const auto ca=costs.find({a->key.id,a->key.generation}),cb=costs.find({b->key.id,b->key.generation});
                const float ac=ca==costs.end()?std::numeric_limits<float>::max():ca->second;
                const float bc=cb==costs.end()?std::numeric_limits<float>::max():cb->second;
                return ac==bc?a->key.id<b->key.id:ac<bc;});
            for(const auto* a:ranked){distance_round.push_back(a->key);distance_enrolled.insert({a->key.id,a->key.generation});}
        }
        while(!distance_round.empty()&&w->cohort.size()<limits.cover_queries_per_slice){
            const auto key=distance_round.front();distance_round.pop_front();
            const auto found=std::lower_bound(source.actors.begin(),source.actors.end(),key.id,
                [](const ActorSnapshot& a,std::uint32_t id){return a.key.id<id;});
            if(found==source.actors.end()||found->key!=key||!eligible(*found))continue;
            w->cohort.push_back(std::size_t(found-source.actors.begin()));w->selected.emplace(key.id,key);
        }
        w->ends_round=distance_round.empty();
        if(w->cohort.empty())return false;
        ++stats.cohorts_started;stats.cohort_members_scheduled+=w->cohort.size();
        last_scheduled_snapshot=source.sequence;work=std::move(w);return true;
    }
    bool occupied_now(const Order& proposed,const Work* work_item=nullptr) const {
        const Work::RoomClaim* own_claim=nullptr;
        if(work_item) {
            const auto found=work_item->room_claims.find(proposed.actor.id);
            if(found!=work_item->room_claims.end()&&found->second.actor==proposed.actor)own_claim=&found->second;
        }
        const bool room_order=own_claim!=nullptr;
        if(room_order&&(!std::isfinite(config.room.spacing_m)||config.room.spacing_m<=0||
            !std::isfinite(config.room.arrival_m)||config.room.arrival_m<0))return true;
        if(proposed.pace==Pace::stand&&!room_order)return false;
        const auto& source=runtime.snapshot();
        const auto own=std::lower_bound(source.actors.begin(),source.actors.end(),proposed.actor.id,
            [](const ActorSnapshot& actor,std::uint32_t id){return actor.key.id<id;});
        if(own==source.actors.end()||own->key!=proposed.actor)return true;
        std::vector<ActorKey> meeting_members;
        if(proposed.flags&order_survivor_meet){
            if(!work_item||proposed.kind!=OrderKind::regroup||proposed.pace!=Pace::walk||
                proposed.assignment.generation||proposed.fire.allowed||valid_key(proposed.aid)||
                proposed.throw_order.valid||proposed.portal.valid||work_item->room_claims.count(proposed.actor.id))return true;
            for(const auto& link:work_item->cooperation_results)if(link.approaching){
                const auto first=std::find_if(work_item->source.sides.begin(),work_item->source.sides.end(),[&](const SideSnapshot& s){return s.side_id==link.first;});
                const auto second=std::find_if(work_item->source.sides.begin(),work_item->source.sides.end(),[&](const SideSnapshot& s){return s.side_id==link.second;});
                if(first==work_item->source.sides.end()||second==work_item->source.sides.end()||first->faction!=second->faction)continue;
                const bool member=std::find(first->members.begin(),first->members.end(),proposed.actor)!=first->members.end()||
                    std::find(second->members.begin(),second->members.end(),proposed.actor)!=second->members.end();
                if(!member)continue;
                meeting_members=first->members;meeting_members.insert(meeting_members.end(),second->members.begin(),second->members.end());break;
            }
            if(meeting_members.empty())return true;
        }
        bool urgent_escape=false;
        if(work_item&&work_item->outdoor_result.status==OutdoorStatus::ready&&
            emergency_room_override(proposed,work_item->outdoor_result))urgent_escape=true;
        if(work_item&&work_item->knowledge_frame&&proposed.kind==OrderKind::cover&&proposed.pace==Pace::run&&
            (proposed.flags&order_urgent)&&(proposed.role==roles::under_fire_cover||proposed.role==roles::seek_cover))
            for(const auto& member:work_item->knowledge_frame->members)
                if(member.actor==own->key&&member.ownership_generation==own->ownership_generation&&member.hit_from.valid&&
                    member.hit_from.observer==own->key&&fresh(member.hit_from.at_ms,source.now_ms,config.knowledge.uf_escape_ms))
                    urgent_escape=true;
        // Personal already priced teammate proximity for a fresh escape. Do
        // not turn that soft cost back into the old all-cells-rejected veto.
        // Exact occupied nodes remain unavailable below.
        const float ordinary_spacing=urgent_escape?0.f:config.minimum_goal_separation*config.minimum_goal_separation;
        const float room_spacing=config.room.spacing_m*config.room.spacing_m;
        const float meeting_spacing=(std::max)(1.f,config.room.spacing_m)*(std::max)(1.f,config.room.spacing_m);
        const float live_radius=(std::max)(0.f,config.room.spacing_m-config.room.arrival_m);
        const float arrived_spacing=live_radius*live_radius;
        const std::unordered_map<std::uint32_t,ActorKey>* roster=nullptr;
        if(own_claim) {
            auto found=work_item->room_rosters.find(own_claim->side);
            if(found!=work_item->room_rosters.end())roster=&found->second;
        }
        for(const auto& a:source.actors) {
            if(a.key==proposed.actor||a.faction!=own->faction||!a.alive)continue;
            const Work::RoomClaim* other_claim=nullptr;
            const auto member=roster?roster->find(a.key.id):std::unordered_map<std::uint32_t,ActorKey>::const_iterator{};
            const bool same_side=roster&&member!=roster->end()&&member->second==a.key;
            if(room_order) {
                const auto other=work_item->room_claims.find(a.key.id);
                if(other!=work_item->room_claims.end()&&other->second.actor==a.key&&other->second.side==own_claim->side&&
                    other->second.order.ownership_generation==a.ownership_generation&&
                    other->second.order.restrictions_revision==a.restrictions_revision)other_claim=&other->second;
            }
            const bool care_body=own_claim&&own_claim->care_target==a.key&&a.wounded;
            const bool meeting_peer=!meeting_members.empty()&&
                std::find(meeting_members.begin(),meeting_members.end(),a.key)!=meeting_members.end();
            const float body_spacing=care_body?config.room.aid_min_body_m*config.room.aid_min_body_m:
                (meeting_peer?meeting_spacing:(other_claim?arrived_spacing:(same_side?room_spacing:ordinary_spacing)));
            if(a.node_id==proposed.goal_node||distance_squared(a.position,proposed.goal_position)<body_spacing)return true;
            auto held=accepted.find(a.key.id);
            if(held!=accepted.end()&&held->second.actor==a.key&&held->second.ownership_generation==a.ownership_generation) {
                const auto& order=held->second;
                const bool same_assignment=other_claim&&order.assignment.generation==other_claim->order.assignment.generation&&
                    order.assignment.portal==other_claim->order.assignment.portal&&
                    // All actual room claims share room spacing. ENTRY and
                    // SEARCH do not revert to outdoor goal spacing mid-plan.
                    order.assignment.generation!=0&&order.level_epoch==source.level_epoch&&
                    order.geometry_revision==source.geometry_revision&&order.restrictions_revision==a.restrictions_revision&&
                    order.expires_at_ms>source.now_ms;
                const bool first_hold=same_side&&(order.flags&order_first_response)&&
                    (order.kind==OrderKind::hold||order.kind==OrderKind::reload)&&order.pace==Pace::stand;
                // A current unassigned hold at this member's actual feet is
                // already covered by the body check above. If both men have
                // fresh room claims, a routine lease-expiry fallback must not
                // enlarge that same body's reservation to outdoor spacing.
                // Other movement, care, personal cover and foreign leases keep
                // their ordinary reservation rules.
                const bool local_hold=same_side&&other_claim&&!order.assignment.generation&&
                    (order.kind==OrderKind::hold||order.kind==OrderKind::reload)&&order.pace==Pace::stand&&
                    !valid_key(order.aid)&&!valid_key(order.finish)&&!order.throw_order.valid&&
                    order.level_epoch==source.level_epoch&&order.geometry_revision==source.geometry_revision&&
                    order.restrictions_revision==a.restrictions_revision&&order.expires_at_ms>source.now_ms&&
                    order.goal_node==a.node_id&&distance_squared(order.goal_position,a.position)<=.000001f;
                if(local_hold)continue;
                const float spacing=meeting_peer?meeting_spacing:(same_assignment||first_hold?room_spacing:ordinary_spacing);
                if(order.goal_node==proposed.goal_node||distance_squared(order.goal_position,proposed.goal_position)<spacing)return true;
            }
        }
        return false;
    }
    void publish(const WorldSnapshot& source,const Decision& candidate,TimeMs now,
        const PublishFunction& fn,bool plan,const Work* work_item=nullptr,const PointFirePublishFunction& point_fn={})
    {
        std::vector<PointFireAuthority> proofs;
        if(point_fn&&work_item){
            std::unordered_map<std::uint32_t,const PointFireAuthority*> proposed;
            for(const auto& row:work_item->outdoor_result.overrides)
                if(row.point_fire.source!=PointFireSource::none)proposed.emplace(row.order.actor.id,&row.point_fire);
            for(const auto& order:candidate.orders){const auto pi=proposed.find(order.actor.id);if(pi==proposed.end())continue;
                // Final sequence is allocated on this authority seam. The proof
                // continues to bind the original point/source and time limits.
                auto proof=*pi->second;proof.authored.order_seq=order.order_seq;proofs.push_back(proof);}
        }
        const auto result=mailbox.prepare(source,runtime.snapshot(),candidate,now,proofs);
        if(!result.token) {
            for(const auto& o:candidate.orders)runtime.rejected_proposal(o.actor,o.order_seq);
            for(const auto& key:candidate.releases)runtime.rejected_release(key);
            stats.native_rejected+=candidate.orders.size()+candidate.releases.size();return;
        }
        Decision checked;if(!mailbox.prepared(result.token,checked)){mailbox.cancel(result.token);return;}
        NativePublication native;
        try {std::vector<PointFireAuthority> checked_proofs;
            if(point_fn&&mailbox.prepared_point_fire(result.token,checked_proofs))native=point_fn(checked,checked_proofs);
            else native=fn(checked);
        }catch(...) {
            mailbox.cancel(result.token);++stats.jobs_failed;
            // A throwing boundary cannot tell us what was applied. Never
            // advance any portable generation on that ambiguous result.
            return;
        }
        auto committed=mailbox.commit(result.token,runtime.snapshot(),now,native.accepted);
        if(committed.envelope!=MailboxReject::none){mailbox.cancel(result.token);++stats.jobs_failed;return;}
        std::unordered_set<std::uint32_t> installed;
        for(const auto& member:committed.actors)
            if(member.reason==MailboxReject::none)installed.insert(member.actor.id);
        std::unordered_map<std::uint32_t,const Order*> applied_orders;
        for(const auto& order:checked.orders)applied_orders.emplace(order.actor.id,&order);
        for(const auto& proposed:candidate.orders) {
            // The committed receipt already binds the full key, lease and
            // sequence. IDs are unique in this validated envelope; no repeated
            // scan through all historical generations is needed here.
            const auto applied=applied_orders.find(proposed.actor.id);
            if(!installed.count(proposed.actor.id)||applied==applied_orders.end()) {
                runtime.rejected_proposal(proposed.actor,proposed.order_seq);++stats.native_rejected;continue;
            }
            // The mailbox may close fire/downgrade aim when sight was lost.
            // Keep exactly the value actually sent to the native publisher.
            const auto& o=*applied->second;
            ++stats.native_accepted;
            if(o.kind==OrderKind::yield)accepted.erase(o.actor.id);
            else {
                if(execution_ledger.emplace(o.order_seq,o).second)execution_expiry.emplace_back(
                    saturated_after(now,saturated_after(config.order_lifetime_ms,config.snapshot_max_age_ms)),o.order_seq);
                accepted[o.actor.id]=o;if(plan){
                    const Order* authored=nullptr;
                    const AcceptedCoverProof* cover=nullptr;
                    if(work_item) {
                        const auto claim=work_item->room_claims.find(o.actor.id);
                        if(claim!=work_item->room_claims.end()&&claim->second.actor==o.actor)authored=&claim->second.authored;
                        const auto proof=work_item->cover_proofs.find(o.actor.id);
                        if(proof!=work_item->cover_proofs.end())cover=&proof->second;
                    }
                    // Only native-accepted plans install their matching worker
                    // evidence. Rejected proposals cannot authorize a later
                    // moving reload merely by naming the same cover goal.
                    const bool plan_accepted=runtime.accepted_plan(o,authored,cover);
                    if(plan_accepted&&work_item) {
                        const auto& actors=work_item->source.actors;
                        const auto actor=std::lower_bound(actors.begin(),actors.end(),o.actor.id,
                            [](const ActorSnapshot& a,std::uint32_t id){return a.key.id<id;});
                        if(actor!=actors.end()&&actor->key==o.actor)
                            for(const auto& side:work_item->outdoor_result.sides)
                                for(const auto& personal:side.personal)
                                    if(personal.proposal.order.actor==o.actor&&
                                       (personal.reason==OutdoorPersonalReason::grenade_escape||
                                        personal.reason==OutdoorPersonalReason::grenade_wait))
                                        runtime.accepted_grenade_episode(o,*actor,personal.grenade.perceived,personal.grenade.until);
                    }
                    ++stats.cohort_members_accepted;
                }

            }
        }
        for(const auto& key:candidate.releases) {
            if(!installed.count(key.id)){runtime.rejected_release(key);++stats.native_rejected;continue;}
            const auto a=std::lower_bound(source.actors.begin(),source.actors.end(),key.id,
                [](const ActorSnapshot& actor,std::uint32_t id){return actor.key.id<id;});
            if(a!=source.actors.end()&&a->key==key)runtime.accepted_release(key,a->ownership_generation);
            const auto admitted=admission.find(key.id);
            if(admitted!=admission.end()&&admitted->second.key==key)admitted->second.closed=true;
            accepted.erase(key.id);++stats.native_accepted;
        }
    }
    void submit() {
        if(!work||!services.jobs)return;
        auto w=work;w->done.store(false,std::memory_order_release);
        JobTicket ticket=0;
        try {ticket=services.jobs->submit([w,c=config,l=limits](const std::function<bool()>& cancel){run_slice(w,c,l,cancel);},0);}
        catch(...){w->failed=true;w->done.store(true,std::memory_order_release);++stats.jobs_failed;}
        if(ticket)++stats.jobs_submitted;
        // A refused submission keeps the advanced cursor (advance on admission, above): its members wait one round,
        // the actors behind them are never starved by a cohort the queue keeps refusing.
        else {w->cancelled.store(true,std::memory_order_relaxed);work.reset();++stats.jobs_rejected;}
    }
};

CombatDispatcher::CombatDispatcher(Config c,DispatchLimits l,std::shared_ptr<const SideLifecyclePolicy> policy,std::shared_ptr<const OutdoorDispatchPolicy> outdoor,
    std::shared_ptr<const RoomExposureDispatchPolicy> exposure):impl_(std::make_unique<Impl>(c,l,std::move(policy),std::move(outdoor),std::move(exposure))){}
CombatDispatcher::~CombatDispatcher(){if(impl_->work)impl_->work->cancelled.store(true,std::memory_order_relaxed);}
void CombatDispatcher::reset(){impl_->clear();}
const DispatchStats& CombatDispatcher::stats() const{
    const auto& source=impl_->runtime.stats();
    impl_->stats.episode_admission=source.episode_admission;
    impl_->stats.episode_contact=source.episode_contact;
    impl_->stats.episode_hit=source.episode_hit;
    return impl_->stats;
}
std::vector<Order> CombatDispatcher::accepted_orders() const{return impl_->orders();}
std::shared_ptr<const KnowledgeFrame> CombatDispatcher::knowledge_frame() const{return impl_->last_knowledge;}
const KnowledgeInboxStats& CombatDispatcher::knowledge_inbox_stats() const{return impl_->inbox.stats();}
std::vector<RoomDispatchReceipt> CombatDispatcher::room_frames() const{return impl_->last_rooms;}
std::shared_ptr<const SideDispatchReceipt> CombatDispatcher::side_frame() const{return impl_->last_sides;}
std::shared_ptr<const SideLifecycleResult> CombatDispatcher::lifecycle_frame() const{return impl_->last_lifecycle;}
std::vector<OutdoorDispatchReceipt> CombatDispatcher::outdoor_frames() const{return impl_->last_outdoor;}
bool CombatDispatcher::ingest(WorldSnapshot snapshot,EngineServices services,const KnowledgeContext& knowledge_context,const LifecycleSignals& lifecycle_signals,
    const OutdoorCaptureEvidence& outdoor_evidence,std::shared_ptr<const ExposureGeometryInput> exposure_geometry,const FriendlyCasualtyEvidence& friendly_casualties) {
    auto& p=*impl_;
    // Runtime validates the entire value input before replacing current state.
    const auto old_epoch=p.runtime.snapshot().level_epoch;
    if(old_epoch&&snapshot.level_epoch<old_epoch)return false;
    p.missions.apply(snapshot);
    auto authority=capture_authority(snapshot);
    const auto changed=changed_authority(p.authority,authority);
    const bool salvage_cover=only_actor_replacements(p.authority,authority,changed);
    if(!p.runtime.ingest(std::move(snapshot)))return false;
    p.runtime.observe_species(lifecycle_signals.perceived_species);
    const auto& current=p.runtime.snapshot();
    if(current.level_epoch!=old_epoch) {
        if(p.work)p.work->cancelled.store(true,std::memory_order_relaxed);
        p.work.reset();p.worker=std::make_shared<Worker>(p.config,p.lifecycle_policy,p.outdoor_policy,p.exposure_policy);p.accepted.clear();p.execution_ledger.clear();p.execution_expiry.clear();p.admission.clear();
        p.next_plan=0;p.mailbox.reset(current.level_epoch);
        p.after_actor_id=invalid_id;p.last_scheduled_snapshot=0;
        p.inbox.reset();p.last_knowledge.reset();p.last_rooms.clear();p.last_sides.reset();p.last_lifecycle.reset();p.distance_round.clear();p.distance_enrolled.clear();
        p.last_outdoor.clear();p.outdoor_capture={};p.friendly_casualties={};p.pending_suppression.clear();
        p.authority={};
    }
    p.authority.epoch=authority.epoch;p.authority.map=authority.map;
    p.authority.actors=std::move(authority.actors);
    if(authority.sides_valid){p.authority.sides=std::move(authority.sides);p.authority.sides_valid=true;}
    if(!changed.empty()) {
        if(p.work) {
            if(salvage_cover)p.work->personal_cover_only=true;
            else p.work->cancelled.store(true,std::memory_order_relaxed);
        }
        for(const auto& actor:current.actors)if(changed.count(actor.key.id)) {
            p.runtime.invalidate_plan(actor.key,actor.ownership_generation);
            p.runtime.invalidate_lifecycle(actor.key,actor.ownership_generation);
        }
        p.replan_due=true;
    }
    auto context=knowledge_context;
    // Feed K1 the role that native execution actually acknowledged. Without
    // this bridge its search retention and assault/retreat dogfight policy
    // always saw Intent::none in the game despite passing isolated tests.
    for(const auto& actor:current.actors){
        const auto applied=p.execution_ledger.find(actor.last_applied.order_seq);
        if(applied==p.execution_ledger.end()||changed.count(actor.key.id))continue;
        const MemberKnowledge* known=nullptr;
        if(p.last_knowledge&&p.last_knowledge->level_epoch==current.level_epoch)
            for(const auto& member:p.last_knowledge->members)if(member.actor==actor.key){known=&member;break;}
        MemberKnowledgeContext member;
        if(!executed_knowledge_context(current,actor,applied->second,known,p.config.snapshot_max_age_ms,member))continue;
        const auto supplied=std::find_if(context.members.begin(),context.members.end(),[&](const MemberKnowledgeContext& old){return old.actor==actor.key;});
        if(supplied==context.members.end())context.members.push_back(member);
    }
    if(p.pending_suppression.empty())p.inbox.capture(current,context,lifecycle_signals,friendly_casualties);
    else {
        for(const auto&receipt:p.pending_suppression){
            const auto a=std::lower_bound(current.actors.begin(),current.actors.end(),receipt.fact.observer.id,
                [](const ActorSnapshot& row,std::uint32_t id){return row.key.id<id;});
            if(receipt.epoch==current.level_epoch&&a!=current.actors.end()&&a->key==receipt.fact.observer&&
                a->ownership_generation==receipt.owner&&a->faction==receipt.faction&&receipt.fact.at_ms<=current.now_ms)
                context.suppressed_foes.push_back(receipt.fact);
        }
        // Journal original actual-fire time, never receipt or replay time.
        if(p.inbox.capture(current,context,lifecycle_signals,friendly_casualties))p.pending_suppression.clear();
    }
    if(p.lifecycle_policy&&(!lifecycle_signals.completed.empty()||!lifecycle_signals.errors.empty()||
        !lifecycle_signals.identity_resets.empty()))p.replan_due=true;
    p.services=services;
    p.outdoor_capture=outdoor_evidence;
    p.exposure_geometry=std::move(exposure_geometry);
    p.friendly_casualties=friendly_casualties;
    std::unordered_set<std::uint32_t> ids;
    for(const auto& a:current.actors) {
        ids.insert(a.key.id);auto prior=p.admission.find(a.key.id);
        const bool eligible=a.alive&&!a.wounded&&!a.yield_reasons&&(a.flags&flag_admitted);
        if(!eligible){
            // Capturing an observer does not admit it to native combat. There
            // is no first-order debt while scripts/native logic own this man.
            // An explicit native first-order timeout still counts exactly once.
            if(prior!=p.admission.end()){
                const auto& entry=prior->second;
                if(entry.key==a.key&&entry.owner==a.ownership_generation&&!entry.applied&&!entry.failed&&!entry.closed&&
                    (a.yield_reasons&yield_no_first_order)&&current.now_ms>=saturated_after(entry.at,p.config.first_order_deadline_ms))
                    ++p.stats.first_deadline_missed;
                p.admission.erase(prior);
            }
            p.accepted.erase(a.key.id);continue;
        }
        if(prior==p.admission.end()||prior->second.key!=a.key||prior->second.owner!=a.ownership_generation) {
            p.admission[a.key.id]={a.key,a.ownership_generation,current.now_ms,false};p.accepted.erase(a.key.id);
        }
        auto& admission=p.admission.at(a.key.id);
        if(!admission.applied&&!admission.failed&&!admission.closed&&a.last_applied.order_seq) {
            const auto executed=p.execution_ledger.find(a.last_applied.order_seq);
            if(executed!=p.execution_ledger.end()){const auto& o=executed->second;
                if(o.actor==a.key&&o.ownership_generation==a.ownership_generation&&o.level_epoch==current.level_epoch&&
                    o.restrictions_revision==a.restrictions_revision&&o.geometry_revision==current.geometry_revision&&
                    a.last_applied.at_ms>=o.issued_at_ms&&a.last_applied.at_ms<=o.expires_at_ms&&
                    a.last_applied.at_ms<=saturated_after(admission.at,p.config.first_order_deadline_ms)&&
                    a.last_applied.at_ms<=a.observed_at_ms&&fresh(a.last_applied.at_ms,current.now_ms,p.config.snapshot_max_age_ms)){
                    admission.applied=true;++p.stats.first_applied;
                    const auto age=a.last_applied.at_ms>=admission.at?a.last_applied.at_ms-admission.at:0;
                    p.stats.maximum_first_applied_ms=(std::max)(p.stats.maximum_first_applied_ms,age);
                    if(age>p.config.first_order_deadline_ms)++p.stats.first_deadline_missed;
                }
            }
        }
        if(!a.alive||a.wounded||a.yield_reasons||!(a.flags&flag_admitted))
            p.accepted.erase(a.key.id);
    }
    for(auto it=p.admission.begin();it!=p.admission.end();) {
        if(!ids.count(it->first)){p.accepted.erase(it->first);it=p.admission.erase(it);}else ++it;
    }
    return true;
}
void CombatDispatcher::tick(TimeMs now,const PublishFunction& publish,bool(*deadline)(void*),void* context,const PointFirePublishFunction& point_publish) {
    auto& p=*impl_;const auto& current=p.runtime.snapshot();
    if(!publish||!current.level_epoch||now<current.now_ms)return;
    while(!p.execution_expiry.empty()&&p.execution_expiry.front().first<=now) {
        p.execution_ledger.erase(p.execution_expiry.front().second);p.execution_expiry.pop_front();
    }
    p.mailbox.expire(now);
    for(auto& item:p.admission){auto& a=item.second;
        if(!a.applied&&!a.failed&&!a.closed&&now>=saturated_after(a.at,p.config.first_order_deadline_ms)){
            a.failed=true;++p.stats.first_deadline_missed;p.runtime.fail_first_order(a.key,a.owner);
        }
    }
    // Drain before bootstrap so a valid completed plan is not invalidated by
    // an unnecessary first-response refresh on this very same seam.
    if(p.work&&p.work->done.load(std::memory_order_acquire)) {
        auto w=p.work;
        // Replay completion is independent of subsequent cover/tactical work.
        // done/acquire ensures that no worker still mutates this staged clone.
        // Commit complete facts even if that later work expired or failed, but
        // never commit a partial replay, another worker generation, or an epoch.
        if(w->knowledge_replay_complete&&w->staged_knowledge&&w->worker==p.worker&&
            w->source.level_epoch==current.level_epoch) {
            p.worker->knowledge=std::move(w->staged_knowledge);p.worker->side_registry=std::move(w->staged_sides);
            if(w->staged_lifecycle) {
                p.worker->lifecycle=std::move(w->staged_lifecycle);
                if(w->lifecycle_frame) {
                    p.last_lifecycle=w->lifecycle_frame;
                    // Factual history may commit after a later tactical abort;
                    // native control effects require an uncancelled fresh work
                    // item and Runtime's independent current-lease validation.
                    if(!w->personal_cover_only&&!w->cancelled.load(std::memory_order_relaxed))
                        p.runtime.install_lifecycle(w->source,*w->lifecycle_frame);
                }
            }
            // Complete factual replay can commit even when later tactics fail.
            // Its identity resets must invalidate old room history in that case
            // too, including resets from earlier ticks in the replay interval.
            for(auto side:w->reset_room_sides){p.worker->rooms.erase(side);if(p.worker->outdoor)p.worker->outdoor->reset_side(side);
                if(p.worker->exposure)p.worker->exposure->invalidate_side(side);}
            if(w->knowledge_frame)p.last_knowledge=w->knowledge_frame;
            p.inbox.acknowledge(w->source.level_epoch,w->knowledge_ack_sequence);
            p.stats.knowledge_ticks_committed+=w->knowledge_ticks_processed;
            p.stats.knowledge_ticks_quarantined+=w->knowledge_ticks_quarantined;
            p.stats.knowledge_ticks_stale+=w->knowledge_ticks_stale;
            p.stats.knowledge_side_identity_resets+=w->knowledge_identity_resets;
        } else if(w->cancelled.load(std::memory_order_relaxed))++p.stats.knowledge_replays_cancelled;
        if(w->complete&&!w->failed&&!w->personal_cover_only&&!w->cancelled.load(std::memory_order_relaxed)&&
            w->worker==p.worker&&w->source.level_epoch==current.level_epoch&&w->source.geometry_revision==current.geometry_revision&&
            fresh(w->source.now_ms,now,p.config.snapshot_max_age_ms)&&w->exposure&&w->exposure_token){
            if(!w->exposure->finish(w->exposure_token,true))w->failed=true;
            w->exposure_token=0;
        }
        if(w->failed||w->cancelled.load(std::memory_order_relaxed)||
            w->source.level_epoch!=current.level_epoch||w->source.geometry_revision!=current.geometry_revision||
            !fresh(w->source.now_ms,now,p.config.snapshot_max_age_ms)) {
            if(w->failed)++p.stats.jobs_failed;else if(w->cancelled)++p.stats.jobs_cancelled;else ++p.stats.plans_stale;
            p.work.reset();
        }else if(!w->complete) p.submit();
        else {
            // Room control and exposure sight authority commit only after the
            // same freshness/cancellation/geometry checks as outdoor control.
            if(w->complete&&!w->failed&&!w->personal_cover_only&&!w->cancelled.load(std::memory_order_relaxed)&&w->rooms_ready&&
                w->worker==p.worker&&w->source.level_epoch==current.level_epoch) {
                p.worker->rooms=std::move(w->staged_rooms);p.last_rooms=w->room_results;
                p.worker->cooperation=std::move(w->staged_cooperation);
                // Whole-building proof comes from completed room observations,
                // not arrival at the last door or one room's ready_for_next flag.
                p.worker->completions.clear();
                if(w->lifecycle_frame&&w->services.buildings)for(const auto& side:w->source.sides) {
                    if(side.mission.kind!=MissionKind::clear_building)continue;
                    const auto session=p.worker->rooms.find(side.side_id);
                    if(session==p.worker->rooms.end()||!session->second.doctrine.state().ready_for_next||
                        session->second.care_pin_room!=invalid_room)continue;
                    const auto building=std::find_if(w->services.buildings->buildings.begin(),w->services.buildings->buildings.end(),
                        [&](const Building& b){return b.id==side.mission.building;});
                    if(building==w->services.buildings->buildings.end()||building->rooms.empty())continue;
                    const auto& searched=session->second.searched;
                    const bool complete=std::all_of(building->rooms.begin(),building->rooms.end(),[&](const Room& room){
                        return std::any_of(searched.begin(),searched.end(),[&](const SearchedRoom& r){
                            return r.side==side.side_id&&r.building==building->id&&r.room==room.id&&r.at_ms<=w->source.now_ms;});});
                    if(!complete)continue;
                    // Whole-building handback is a team receipt. Local physical
                    // coverage alone cannot certify rooms to living members
                    // who never received the source-qualified reports.
                    if(!w->knowledge_frame)continue;
                    bool team_informed=true;
                    for(const auto key:side.members){
                        const auto actor=std::find_if(w->source.actors.begin(),w->source.actors.end(),[&](const ActorSnapshot& a){return a.key==key;});
                        if(actor==w->source.actors.end()||!actor->alive||actor->wounded||!(actor->flags&flag_admitted))continue;
                        const auto mind=std::find_if(w->knowledge_frame->members.begin(),w->knowledge_frame->members.end(),[&](const MemberKnowledge& m){return
                            m.actor==key&&m.side==side.side_id&&m.ownership_generation==actor->ownership_generation;});
                        if(mind==w->knowledge_frame->members.end()){team_informed=false;break;}
                        for(const auto& room:building->rooms)if(std::none_of(mind->communicated_rooms.begin(),mind->communicated_rooms.end(),[&](const SearchedRoom& r){return
                            r.side==side.side_id&&r.building==building->id&&r.room==room.id&&r.at_ms<=w->source.now_ms;})){team_informed=false;break;}
                        if(!team_informed)break;
                    }
                    if(!team_informed)continue;
                    for(const auto& life:w->lifecycle_frame->sides)if(life.identity.side==side.side_id) {
                        BuildingCompletionReceipt receipt;receipt.stamp.identity=life.identity;
                        receipt.stamp.source_snapshot=w->source.sequence;receipt.stamp.at_ms=w->source.now_ms;
                        receipt.building=building->id;receipt.whole_building=true;p.worker->completions.push_back(receipt);
                    }
                }
                if(w->result.accepted&&w->mission_intel_ready) {
                    auto receipt=std::make_shared<SideDispatchReceipt>();
                    receipt->registry=std::move(w->side_result);receipt->mission_intel=std::move(w->mission_intel);
                    receipt->tactical_groups=w->result.groups;p.last_sides=std::move(receipt);
                }
                p.worker->care_assignment_sequence=w->care_assignment_sequence;
                p.worker->defense_assignment_sequence=w->defense_assignment_sequence;
                p.worker->support_assignment_sequence=w->support_assignment_sequence;
                p.stats.room_updates+=w->room_results.size();p.stats.room_orders_proposed+=w->room_proposals.size();
            }
            // Unlike factual K1 history, posture is a control decision. Commit
            // only after all freshness/cancellation/geometry guards above.
            if(!w->personal_cover_only&&w->worker==p.worker&&w->staged_outdoor&&w->outdoor_result.status==OutdoorStatus::ready){
                p.worker->outdoor=std::move(w->staged_outdoor);p.last_outdoor=w->outdoor_result.sides;
                for(const auto& result:w->outdoor_result.sides)for(const auto& side:w->source.sides)
                    if(side.side_id==result.posture.stamp.side&&p.missions.commit(w->source,side,result.posture)) {
                        Decision event;event.level_epoch=w->source.level_epoch;event.snapshot_seq=w->source.sequence;
                        event.side_id=side.side_id;event.geometry_revision=w->source.geometry_revision;
                        event.events.push_back({events::mission_handover,result.posture.handover_building,
                            static_cast<std::uint32_t>(result.posture.handover)});
                        publish(event);
                    }
                for(const auto&fact:w->outdoor_result.executed_suppression){
                    const auto a=std::lower_bound(w->source.actors.begin(),w->source.actors.end(),fact.observer.id,
                        [](const ActorSnapshot& row,std::uint32_t id){return row.key.id<id;});
                    if(a!=w->source.actors.end()&&a->key==fact.observer)
                        p.pending_suppression.push_back({fact,w->source.level_epoch,a->ownership_generation,a->faction});
                }
            }
            p.work.reset();++p.stats.plans_completed;
            if(w->result.accepted) {
                std::unordered_map<std::uint64_t,std::size_t> batches;
                std::vector<Decision> decisions;
                for(const auto& proposal:w->result.proposals) {
                    if(w->personal_cover_only) {
                        const auto& o=proposal.order;
                        if(!proposal.has_cover_evidence||(o.kind!=OrderKind::cover&&o.kind!=OrderKind::hold)||
                            o.assignment.generation||o.portal.valid||o.throw_order.valid||valid_key(o.aid)||
                            (o.flags&(order_committed_entry|order_cover_fire)))continue;
                        const auto proof=w->cover_proofs.find(o.actor.id);
                        if(proof==w->cover_proofs.end()||proof->second.spot.node!=o.goal_node||
                            distance_squared(proof->second.spot.position,o.goal_position)!=0)continue;
                    }
                    if(!p.runtime.accepts_orders(proposal.order.actor,proposal.order.ownership_generation))continue;
                    const auto selected=w->selected.find(proposal.order.actor.id);
                    if(selected==w->selected.end()||selected->second!=proposal.order.actor)continue;
                    auto old=p.accepted.find(proposal.order.actor.id);
                    if(old==p.accepted.end()||old->second.actor!=proposal.order.actor||
                        (old->second.order_seq!=proposal.expected_previous_order_seq&&
                         !refreshed_only(*w,proposal.order.actor,proposal.expected_previous_order_seq,old->second))) {
                        ++p.stats.proposals_superseded;continue;
                    }
                    auto order=proposal.order;order.order_seq=p.runtime.allocate_order_sequence();
                    if(w->personal_cover_only)order.role=order.kind==OrderKind::cover?roles::seek_cover:roles::hold_cover;
                    order.flags&=~order_first_response;
                    if(p.occupied_now(order,w.get())){++p.stats.proposals_reservation_conflicted;continue;}
                    // Refresh execution lifetime from the evidence snapshot,
                    // never from receipt time (which could launder stale facts).
                    order.issued_at_ms=w->source.now_ms;
                    const auto lifetime=saturated_after(w->source.now_ms,p.config.order_lifetime_ms);
                    order.expires_at_ms=order.expires_at_ms?(std::min)(order.expires_at_ms,lifetime):lifetime;
                    if(order.fire.allowed) {
                        const auto window=fire_window_until(p.config,w->source.now_ms,order.expires_at_ms);
                        order.fire.until_ms=order.fire.until_ms?(std::min)(order.fire.until_ms,window):window;
                        if(order.fire.until_ms<=now)order.fire={};
                    }
                    if(!order.order_seq)continue;
                    // Retain the original envelope sequencing identity. Side 0
                    // is bootstrap's newer capture watermark and would discard
                    // another actor's otherwise valid older personal cover.
                    // Salvage carries personal roles only, never side events.
                    const auto batch_group=proposal.group_id;
                    auto found=batches.find(batch_group);
                    if(found==batches.end()) {
                        Decision d;const auto room_side=w->room_groups.find(proposal.group_id);
                        if(room_side!=w->room_groups.end())d.side_id=room_side->second;
                        else {
                            const auto side=std::find_if(w->source.sides.begin(),w->source.sides.end(),[&](const SideSnapshot& s){
                                return s.side_id==proposal.group_id;
                            });
                            if(side==w->source.sides.end())continue;
                            d.side_id=side->side_id;
                        }
                        d.level_epoch=w->source.level_epoch;
                        d.snapshot_seq=w->source.sequence;d.geometry_revision=w->source.geometry_revision;
                        found=batches.emplace(batch_group,decisions.size()).first;decisions.push_back(std::move(d));
                    }
                    decisions[found->second].orders.push_back(order);
                    if(order.role==roles::aid_casualty&&valid_key(order.aid)&&
                        (old->second.aid!=order.aid||old->second.assignment.generation!=order.assignment.generation))
                        decisions[found->second].events.push_back({events::casualty,invalid_room,order.actor.id});
                }
                // Formation is an observed side transition, not a claim that
                // a native movement order survived the final occupancy gate.
                for(const auto& room:w->room_results) {
                    if(w->personal_cover_only)break;
                    auto events=room.doctrine.snapshot_seq==w->source.sequence?room.doctrine.events:std::vector<Event>{};
                    events.insert(events.end(),room.casualty.events.begin(),room.casualty.events.end());
                    // Diagnostic for an actual prepared covering-fire intent,
                    // not a claim that the native weapon has fired a bullet.
                    if(room.cover_fire.accepted&&room.cover_fire.snapshot_seq==w->source.sequence&&valid_key(room.cover_fire.actor)) {
                        const auto batch=std::find_if(decisions.begin(),decisions.end(),[&](const Decision& d){return d.side_id==room.side_id;});
                        if(batch!=decisions.end()&&std::any_of(batch->orders.begin(),batch->orders.end(),[&](const Order& o){return
                            o.actor==room.cover_fire.actor&&o.role==roles::suppress_buddy&&(o.flags&order_cover_fire)&&o.fire.allowed;}))
                            events.push_back({events::suppress,room.doctrine.room,room.cover_fire.actor.id});
                    }
                    if(events.empty())continue;
                    auto found=std::find_if(decisions.begin(),decisions.end(),[&](const Decision& d){return d.side_id==room.side_id;});
                    if(found==decisions.end()) {
                        Decision d;d.side_id=room.side_id;d.level_epoch=w->source.level_epoch;
                        d.snapshot_seq=w->source.sequence;d.geometry_revision=w->source.geometry_revision;
                        decisions.push_back(std::move(d));found=decisions.end()-1;
                    }
                    found->events.insert(found->events.end(),events.begin(),events.end());
                }
                // the job's worker time, shared out over the decisions it produced
                for(auto& d:decisions)d.cost_us=std::uint32_t((std::min)(w->cost_us/decisions.size(),std::uint64_t(0xffffffffu)));
                for(auto& d:decisions)p.publish(w->source,d,now,publish,true,w.get(),point_publish);
            }
            p.next_plan=w->ends_round?saturated_after(now,p.limits.plan_interval_ms):now;
        }
    }
    if(p.work&&!p.work->cancelled.load(std::memory_order_relaxed)&&
        fresh(p.work->source.now_ms,now,p.config.snapshot_max_age_ms)) {
        const auto until=saturated_after(p.work->source.now_ms,p.config.snapshot_max_age_ms);
        for(const auto& order:p.work->current) {
            const auto selected=p.work->selected.find(order.actor.id);
            if(selected!=p.work->selected.end()&&selected->second==order.actor)
                p.runtime.defer_refresh(order.actor,order.order_seq,until);
        }
    }
    std::vector<Decision> first;
    const auto step_started=std::chrono::steady_clock::now();
    p.runtime.step(now,p.limits.bootstrap_work_per_tick,first,deadline,context);
    if(p.runtime.take_replan_request())p.replan_due=true;
    if(!first.empty()) {
        Decision batch;batch.level_epoch=current.level_epoch;batch.snapshot_seq=current.sequence;
        batch.geometry_revision=current.geometry_revision;
        batch.cost_us=std::uint32_t((std::min)(std::int64_t(0xffffffff),std::int64_t(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now()-step_started).count())));
        // Independent first responses share this source snapshot. Validate the
        // snapshot indexes once, while retaining per-actor acceptance/failure.
        // side_id0 is a transport batch, not a coordinated squad-plan claim.
        for(auto& d:first) {
            p.stats.bootstrap_proposed+=d.orders.size()+d.releases.size();
            batch.orders.insert(batch.orders.end(),d.orders.begin(),d.orders.end());
            batch.releases.insert(batch.releases.end(),d.releases.begin(),d.releases.end());
            batch.events.insert(batch.events.end(),d.events.begin(),d.events.end());
        }
        p.publish(current,batch,now,publish,false,nullptr,point_publish);
    }
    if(p.work&&!fresh(p.work->source.now_ms,now,p.config.snapshot_max_age_ms))
        p.work->cancelled.store(true,std::memory_order_relaxed);
    // A disturbed plan (hit, target flip) re-plans at the first chance instead of after the rest between rounds
    if(!p.work&&p.services.jobs&&(now>=p.next_plan||p.replan_due)&&!p.accepted.empty()&&current.sequence!=p.last_scheduled_snapshot&&
        fresh(current.now_ms,now,p.config.snapshot_max_age_ms)) {
        if(p.begin_cohort(current)){p.replan_due=false;p.submit();}
        if(!p.work)p.next_plan=saturated_after(now,p.limits.plan_interval_ms);
    }
}
}}}
