#include "stdafx.h"
#include "pch_script.h"
#include "EngineFireGate.h"
#include "EngineCombatAdapter.h"
#include "EngineRouteProof.h"
#include <cmath>
#include "EngineOutdoorCapture.h"
#include "EngineFriendlyCasualty.h"
#include "EngineAppliedAction.h"
#include "EngineOutdoorCombatCapture.h"
#include "EngineRecorder.h"
#include "trajectories.h"
#include "../xrPhysics/iphworld.h"
#include "../xrPhysics/MathUtils.h"
#include <limits>
#include "EngineOutdoorSettings.h"
#include "PointRayBudget.h"
#include "PointFireMuzzle.h"
#include "EnginePerceivedSpecies.h"
#include "EngineExecutionContract.h"
#include "NativeVoicePolicy.h"
#include "EngineReadmission.h"
#include "EngineAdmissionWake.h"
#include "EngineNearMiss.h"
#include "hit_memory_manager.h"
#include "EngineLookGuard.h"
#include "EngineVisibleLook.h"
#include "EngineGait.h"
#include "EngineCapture.h"
#include "PhysicsShellHolder.h"
#include "EngineCompat.h"
#include "EngineServices.h"
#include "core/CombatCore.h"
#include "core/CombatLookCommitment.h"
#include "core/CombatExecutorReflex.h"
#include "core/CombatSideLifecycle.h"
#include "core/CombatDispatcher.h"
#include "ai/stalker/ai_stalker.h"
#include "stalker_combat_action_base.h"
#include "stalker_planner.h"
#include "stalker_property_evaluators.h"
#include "stalker_decision_space.h"
#include "stalker_movement_manager_smart_cover.h"
#include "stalker_animation_manager.h"
#include "movement_manager_space.h"
#include "detail_path_manager_space.h"
#include "memory_manager.h"
#include "enemy_manager.h"
#include "agent_manager.h"
#include "agent_member_manager.h"
#include "member_order.h"
#include "level.h"
#include "seniority_hierarchy_holder.h"
#include "team_hierarchy_holder.h"
#include "squad_hierarchy_holder.h"
#include "group_hierarchy_holder.h"
#include "visual_memory_manager.h"
#include "danger_manager.h"
#include "sight_manager.h"
#include "sight_action.h"
#include "inventory.h"
#include "InventoryOwner.h"
#include "weapon.h"
#include "grenade.h"
#include "level_graph.h"
#include "ai_object_location.h"
#include "script_game_object.h"
#include "sound_player.h"
#include "ai/stalker/ai_stalker_space.h"
#include <map>
#include <memory>
#include <deque>
#include <atomic>
#include <set>

extern float g_ai_aim_fire_angle;
namespace painter { namespace combat { namespace engine {
namespace {
using namespace MonsterSpace;
struct Record {
    core::ObserverSpecies species_memory;
    CAI_Stalker* object=nullptr; // authority thread only; never passed to the core
    ActorKey key;
    std::uint64_t ownership=1,restrictions=1,epoch=0,geometry=0,last_order=0;
    TimeMs previous_snapshot=0,compat_at=0;
    std::uint32_t compatibility=yield_scheme_block;
    bool has_order=false,owned=false,cleanup_done=true,stale_anchor=false,reported=false,provisional=false;
    TimeMs admitted_at=0,released_at=0;HandbackNotice handback;   // released_at: the last hand-back of an owned man
    bool actual_executed=false;
    DeadlineReadmission deadline_retry;
    LookLineCache look_lines;
    core::CombatLookCommitment look_commitment;
    LookEpisodeTracker look_episode;
    ActorKey finish_after;
    ThrowExecution throwing;bool throw_reported=false;AidExecution aiding;TimeMs aid_last_hit=0;
    painter_voice::ActorGate voice;
    ActorKey aid_claim_target;bool aid_claimed=false;
    Order order,provisional_order;
    TimeMs reflex_until=0,reflex_retry=0,grenade_until=0;std::uint32_t reflex_node=invalid_node;Vec3 reflex_goal;
    const char* side_release_reason="none";bool stand_down=false;std::uint32_t release_side=0;
    core::PointFireAuthority point_fire;
    ActorSnapshot snapshot;
    std::deque<ActorSnapshot::WitnessedSever> witnessed_severs;
    Contact outdoor_seen;TimeMs outdoor_seen_at=0;
    std::string lean_gate_reported;ActorKey lean_gate_focus;unsigned lean_gate_stage=~0u;
    Vec3 hold_position;std::uint32_t hold_node=invalid_node;
    std::deque<SourceStamp> snapshots;
    decltype(ActorSnapshot{}.last_rejected) rejected{};
};
using RecordPtr=std::shared_ptr<Record>;
std::map<CAI_Stalker*,RecordPtr> records;
std::map<std::uint64_t,painter_voice::SquadGate> voice_squads;
std::uint64_t voice_epoch=0;
std::map<CEntityAlive const*,ActorKey> perceived;
// side id -> the snapshot of its last applied decision and when; pruned like the dispatcher's ledger (an id unused for
// four order lifetimes is never reused: the dispatcher numbers sides upwards within an epoch)
struct SideSeen{std::uint64_t sequence=0;TimeMs at=0;};
std::map<std::uint32_t,SideSeen> side_sequences;
std::vector<core::LifecycleCooldown> lifecycle_cooldowns,error_cooldowns;
std::uint64_t captured_map=0;
std::uint64_t lifecycle_sequence=0,lifecycle_epoch=0;
std::map<std::uint32_t,std::pair<unsigned,std::string>> side_postures;
std::map<std::uint32_t,std::uint32_t> reported_styles;
core::PassCosts pass_costs;TimeMs pass_report_at=0;
std::uint64_t decision_cost_us=0,decision_count=0;
const char* release_name(core::SideReleaseReason r){
    switch(r){case core::SideReleaseReason::fight_over:return "fight_over";
    case core::SideReleaseReason::building_cleared:return "building_cleared";
    case core::SideReleaseReason::no_living_men:return "no_living_men";
    case core::SideReleaseReason::error:return "error";case core::SideReleaseReason::identity_reset:return "identity_reset";
    case core::SideReleaseReason::monster_fight:return "monster_fight";
    case core::SideReleaseReason::out_of_range:return "out_of_range";case core::SideReleaseReason::door_stuck:return "door_stuck";
    default:return "none";}
}
std::atomic<std::uint64_t> next_generation{1};
bool service_enabled=false;
std::unique_ptr<luabind::functor<bool>> compatibility_gate,aid_complete,aid_claim,aid_release,aid_arrived;
bool compatibility_resolved=false,aid_resolved=false;
std::uint64_t captured_epoch=0,captured_sequence=0,captured_geometry=0;
core::FriendlyCasualtyEvidence friendly_capture;
std::vector<SideSnapshot> friendly_sides;
std::uint32_t friendly_after_observer=invalid_id;
struct FriendlyOffset {ActorKey key;std::size_t offset=0;};
std::map<std::uint32_t,FriendlyOffset> friendly_offsets;
bool observed_aid_allows(const Record& patient,const Record& helper,TimeMs now){
    if(aid_snapshot_allows(patient.snapshot,helper.key,now))return true;
    if(friendly_capture.level_epoch!=captured_epoch||friendly_capture.snapshot_seq!=captured_sequence
        ||friendly_capture.geometry_revision!=captured_geometry)return false;
    for(const auto& f:friendly_capture.facts)if(f.target==patient.key
        &&core::friendly_casualty_aid_allowed(f,helper.key,now,combat_config().snapshot_max_age_ms)){
        for(const auto& side:friendly_sides)if(core::friendly_casualty_member(f,side)
            &&std::find(side.members.begin(),side.members.end(),helper.key)!=side.members.end()
            &&helper.snapshot.faction==side.faction&&(!helper.snapshot.native_squad||helper.snapshot.native_squad==side.group))return true;
    }
    return false;
}
Fvector native(Vec3 v){return Fvector().set(v.x,v.y,v.z);}
Vec3 value(Fvector const& v){return {v.x,v.y,v.z};}
bool registered(Record const& r){auto i=records.find(r.object);return i!=records.end()&&i->second.get()==&r&&r.object->ID()==r.key.id&&!r.object->getDestroy();}
void request_voice(Record& r,painter_voice::Cue cue,u32 category,TimeMs now){
    if(!registered(r)||!r.owned||!r.object->g_Alive())return;
    auto& player=r.object->sound();const auto& collections=player.objects();
    const auto clip=collections.find(category);
    if(clip==collections.end()||!clip->second.second||clip->second.second->m_sounds.empty())return;
    // A level change invalidates all prior group clocks. Record identity and
    // ownership changes reset only this actor's contact and cue clocks.
    if(voice_epoch!=r.epoch){voice_squads.clear();voice_epoch=r.epoch;}
    auto* squad=r.snapshot.native_squad?&voice_squads[r.snapshot.native_squad]:nullptr;
    if(!painter_voice::request(r.voice,squad,r.key.generation,r.ownership,cue,now))return;
    player.play(category); // Stock mask/priority can still suppress playback.
    Msg("[painter_combat] phase=voice_request actor=%u generation=%llu ownership=%llu group=%llu cue=%u sound=%u at_ms=%llu result=requested_not_audible_proof",
        unsigned(r.key.id),r.key.generation,r.ownership,r.snapshot.native_squad,unsigned(cue),unsigned(category),now);
}
RecordPtr find(ActorKey key);
void resolve_aid_hooks(){
    if(aid_resolved)return;aid_resolved=true;
    auto resolve=[](char const* name,std::unique_ptr<luabind::functor<bool>>& slot){
        auto fn=std::make_unique<luabind::functor<bool>>();if(ai().script_engine().functor(name,*fn))slot=std::move(fn);
    };
    resolve("painter_ai_native_aid.complete",aid_complete);resolve("painter_ai_native_aid.claim",aid_claim);
    resolve("painter_ai_native_aid.release",aid_release);
    resolve("painter_ai_native_aid.reached",aid_arrived);
}
void release_aid_claim(Record& r){
    if(!r.aid_claimed)return;r.aid_claimed=false;
    auto target=find(r.aid_claim_target);r.aid_claim_target={};
    // No pointer retained across generations; never clear another helper's claim.
    if(target&&registered(*target)&&aid_release)(*aid_release)(unsigned(r.key.id),target->object->lua_game_object());
}
ActorKey key_of(CEntityAlive const& actor){
    if(actor.getDestroy())return {};
    auto s=smart_cast<CAI_Stalker const*>(&actor);auto r=records.find(const_cast<CAI_Stalker*>(s));
    if(r!=records.end())return r->second->key;
    auto p=perceived.find(&actor);if(p!=perceived.end())return p->second;
    ActorKey key{actor.ID(),next_generation.fetch_add(1)};perceived.emplace(&actor,key);return key;
}
bool armed_item(CInventoryItem* item){
    auto grenade=smart_cast<CGrenade*>(item);
    return grenade&&grenade->GetState()>=CMissile::eThrowStart&&grenade->GetState()<=CMissile::eThrowEnd;
}
void observe_throw_receipt(Record& r,ActorSnapshot const& snapshot,TimeMs now){
    if(!r.throwing.active(r.ownership))return;
    r.throwing.receipt(snapshot.throw_state,now,combat_config().room);
    // Capture consumes completion before a simultaneous policy hand-back can
    // cancel it. Never dereference a dead man's agent manager for bookkeeping.
    if(snapshot.alive&&r.throwing.take_completion()){
        r.object->on_throw_completed();
        request_voice(r,painter_voice::Cue::grenade_release,StalkerSpace::eStalkerSoundThrowGrenade,now);
    }
}
void finish_receipts(Record& r,std::uint8_t reason){
    if(r.throwing.started&&!r.throwing.ended){
        r.throwing.fail(reason);
        if(!r.throw_reported&&r.throwing.reason){end_throw(r.key.id,r.throwing.reason);r.throw_reported=true;}
        // Never holster an armed grenade: DeactivateItem itself calls Throw().
        // Native danger/wound takeover keeps its own emergency authority.
        if(registered(r)&&r.object->g_Alive()&&!armed_item(r.object->inventory().ActiveItem())){
            auto& actor=*r.object;
            if(actor.painter_throw_active())actor.throw_target(native(r.throwing.target),nullptr);
            auto gun=smart_cast<CWeapon*>(actor.inventory().Get(r.throwing.weapon,true));
            if(gun)actor.CObjectHandler::set_goal(eObjectActionAimReady1,static_cast<CInventoryItem*>(gun));
        }
        r.throwing.ended=true;
    }
    if(r.aiding.cancel())note_aid(r.key.id,r.aiding.target,4);
    release_aid_claim(r);
}
std::uint32_t adjusted_item_yields(Record& r,std::uint32_t yields){
    auto& actor=*r.object;auto item=actor.inventory().ActiveItem();
    auto d=actor.memory().danger().selected();bool danger=d&&d->type()==CDangerObject::eDangerTypeGrenade;
    return throw_item_yields(yields,r.throwing.active(r.ownership),item&&item->object_id()==r.throwing.item,
        !item,smart_cast<CWeapon*>(item)!=nullptr,danger);
}
void cleanup(Record& r){
    applied_action_receipts().clear(r.key);
    r.outdoor_seen={};r.outdoor_seen_at=0;
    if(r.cleanup_done||!registered(r))return;
    r.cleanup_done=true;
    finish_receipts(r,throw_cancelled);
    auto& actor=*r.object;
    actor.movement().force_update(false);actor.set_painter_lean(0);actor.set_painter_speed_scale(1);
    set_gait_if_present(actor,Gait::aimed,0);
    if(actor.g_Alive()){
        auto weapon=smart_cast<CWeapon*>(actor.inventory().ActiveItem());
        if(weapon)actor.CObjectHandler::set_goal(eObjectActionAimReady1,static_cast<CInventoryItem*>(weapon));
        notify_release(actor);
    }
}
void relinquish(Record& r,std::uint32_t reason){
    r.point_fire={};r.reflex_until=0;r.reflex_node=invalid_node;r.look_lines.reset();r.look_commitment.reset();r.look_episode.reset();r.actual_executed=false;
    r.witnessed_severs.clear();
    if(r.owned||r.has_order||r.provisional){
        if(reason&yield_no_first_order){
            const auto at=now_ms();const auto delay=r.deadline_retry.failed(at,combat_config().readmit_delay_ms);
            const bool ready=registered(r)&&r.object->brain().initialized();
            const auto action=ready?r.object->brain().current_action_id():u32(-1);
            Msg("[painter_combat] phase=admission_backoff actor=%u generation=%llu cause=first_order_not_executed failures=%u retry_ms=%llu until=%llu planner_ready=%u action=%u script_control=%u scripted_animation=%u smart_cover=%u evidence=native_state",
                unsigned(r.key.id),r.key.generation,r.deadline_retry.failures,delay,r.deadline_retry.until,unsigned(ready),unsigned(action),
                unsigned(registered(r)&&r.object->GetScriptControl()),unsigned(registered(r)&&!r.object->animation().script_animations().empty()),
                unsigned(registered(r)&&r.object->movement().in_smart_cover()));
        }
        if(!xr_strcmp(r.side_release_reason,"none"))r.side_release_reason=reason==yield_invalid_input?"error":
            reason==yield_stale?"stale":reason==yield_no_first_order?"no_first_order":reason==yield_offline?"offline":
            reason==yield_disabled?"disabled":reason==yield_released?"released":"native_veto";
        ++r.ownership;r.released_at=now_ms();
        if(r.handback.queue(reason)){Msg("[painter_combat] phase=hand_back actor=%u generation=%llu ownership=%llu order=%llu reason=%s yield_bits=%u at=%llu",
            unsigned(r.key.id),r.key.generation,r.ownership,r.last_order,r.side_release_reason,unsigned(reason),now_ms());
            recorder_handback(std::uint16_t(r.key.id),r.key.generation,r.ownership,r.last_order,r.side_release_reason,unsigned(reason),now_ms());}   // native17
    }
    r.has_order=false;r.owned=false;r.provisional=false;r.stale_anchor=false;
    if(registered(r)&&r.object->painter_forced_action()==ids::action)r.object->set_painter_forced_action(0);
    release_aid_claim(r); // also runs for destruction before registered() permits engine cleanup
    cleanup(r);
}
std::uint32_t bridge(CAI_Stalker& actor){
    if(!compatibility_resolved){
        compatibility_resolved=true;auto gate=std::make_unique<luabind::functor<bool>>();
        if(ai().script_engine().functor("painter_native_combat_compat.eligible",*gate))compatibility_gate=std::move(gate);
    }
    if(!compatibility_gate)return yield_scheme_block;
    return (*compatibility_gate)(actor.lua_game_object(),actor.memory().enemy().selected()!=nullptr)?yield_none:yield_scheme_block;
}
// CEntity::net_Destroy may already have removed the man and deleted the last
// member's manager before the later CGameObject hook. Probe group presence first.
CAgentMemberManager* combat_members(CAI_Stalker& actor){
    auto& group=Level().seniority_holder().team(actor.g_Team()).squad(actor.g_Squad()).group(actor.g_Group());
    bool in_group=false;for(auto* member:group.members())if(member==&actor){in_group=true;break;}
    if(!in_group)return nullptr;
    auto& manager=group.agent_manager().member();
    for(auto* member:manager.members())if(&member->object()==&actor)return &manager;
    return nullptr;
}
bool native_grenade_reaction(CAI_Stalker& actor){
    auto d=actor.memory().danger().selected();const auto& c=combat_config();const auto now=now_ms();
    if(!c.outdoor.p.personal.enabled||!c.outdoor.p.solo.enabled||!d||d->type()!=CDangerObject::eDangerTypeGrenade
        ||d->perceive_type()!=CDangerObject::eDangerPerceiveTypeVisual||d->time()>now||!_valid(d->position()))return false;
    const auto* grenade=smart_cast<const CGrenade*>(d->dependent_object());
    const bool measured=grenade&&!grenade->getDestroy()&&grenade->destroy_time()!=u32(-1)&&grenade->destroy_time()>now;
    return measured||now-d->time()<=c.outdoor.p.solo.grenade_response_ms;
}
std::uint32_t blockers(Record& r){
    if(!service_enabled)return yield_disabled;
    if(!registered(r))return yield_offline;
    auto& actor=*r.object;if(!actor.g_Alive())return yield_dead;
    auto forced=actor.painter_forced_action();if(forced&&forced!=ids::action)return yield_foreign_owner;
    auto flags=flags_of(actor.ID());
    if(flags&flag_story)return yield_story;
    if(flags&flag_companion)return yield_companion;
    if(flags&flag_scheme_block)return yield_scheme_block;
    if(flags&flag_script_combat)return yield_script_combat;
    if(flags&(flag_wounded_scheme|flag_help_wounded))return yield_wounded;
    if(actor.wounded())return yield_wounded;
    if(actor.GetScriptControl()||!actor.animation().script_animations().empty())return yield_script_control;
    if(actor.movement().in_smart_cover())return yield_smart_cover;
    auto danger=actor.memory().danger().selected();if(danger&&danger->type()==CDangerObject::eDangerTypeGrenade&&!native_grenade_reaction(actor))return yield_grenade;
    auto item=actor.inventory().ActiveItem();auto grenade=smart_cast<CGrenade*>(item);
    std::uint32_t item_yields=yield_none;
    if(grenade&&armed_item(item))item_yields|=yield_grenade;
    auto weapon=smart_cast<CWeapon*>(item);
    if(!weapon||(weapon->GetAmmoElapsed()<=0&&cannot_reload(weapon->GetAmmoElapsed(),weapon->GetSuitableAmmoTotal(false))))item_yields|=yield_no_weapon;
    item_yields=adjusted_item_yields(r,item_yields);if(item_yields)return item_yields;
    auto& evaluators=actor.brain().evaluators();auto a=evaluators.find(StalkerDecisionSpace::eWorldPropertyAnomaly);
    if(a!=evaluators.end()&&a->second->evaluate())return yield_anomaly;
    // v2.21: push-only mode gates on the pushed verdict alone (unknown -> yield_compat_unknown); the bridge mode keeps the
    // cached poll and adds a pushed veto (never an unknown) on top of it
    const bool push=combat_config().compat_mode!=0;const auto pushed=compat_blocker(actor,r.key,push);
    if(push)return pushed;
    return r.compatibility?r.compatibility:pushed;
}
std::uint64_t perception_signature(ActorSnapshot const& a){   // the wake events of a denied push-mode admission
    std::uint64_t h=1469598103934665603ull;auto mix=[&](std::uint64_t v){h^=v;h*=1099511628211ull;};
    mix(a.enemy.key.id);mix(a.enemy.key.generation);mix(a.enemy.visible);mix(a.visible_enemies.count);
    mix(a.hits.count?a.hits.items[a.hits.count-1].at_ms:0);mix(a.near_misses.count?a.near_misses.items[a.near_misses.count-1].at_ms:0);
    mix(a.danger.at_ms);mix(std::uint64_t(a.danger.kind));
    return h;
}
bool executable(Record& r,TimeMs now){
    if(!r.has_order&&!r.provisional)return false;
    auto const& o=r.has_order?r.order:r.provisional_order;
    auto reason=blockers(r);
    if(!reason&&r.epoch!=level_epoch())reason=yield_offline;
    if(!reason&&actual_first_order_overdue(r.admitted_at,now,combat_config().first_order_deadline_ms,r.owned,r.actual_executed)){
        r.side_release_reason="first_order_deadline";reason=yield_no_first_order;
    }
    if(!reason&&r.has_order){
        if(o.actor!=r.key||o.ownership_generation!=r.ownership||o.restrictions_revision!=r.restrictions
            ||o.level_epoch!=level_epoch()||o.geometry_revision!=r.geometry)reason=yield_invalid_input;
        else if((core::expired_idle_order(o,now,r.object->Position().distance_to(native(o.goal_position))<=o.arrival_radius_m)&&
                 !ordinary_refresh_grace(o,r.snapshot,combat_config(),now)&&
                 !room_refresh_hold_grace(o,r.snapshot,combat_config(),now))
            ||execution_phase(o,now,combat_config().stale_release_ms)==ExecutionPhase::release)
            reason=now<o.issued_at_ms?yield_invalid_input:yield_stale;
    }
    if(!reason&&(!ai().level_graph().valid_vertex_id(o.goal_node)||!r.object->movement().restrictions().accessible(o.goal_node)))reason=yield_invalid_navigation;
    if(reason){if(reason==yield_stale)r.side_release_reason="stale_idle";
        else if(reason==yield_invalid_input)r.side_release_reason="identity_reset";relinquish(r,reason);return false;}return true;
}
class Eligibility final:public CStalkerPropertyEvaluator {
    RecordPtr record;
public:
    Eligibility(CAI_Stalker* a,RecordPtr r):CStalkerPropertyEvaluator(a->lua_game_object(),"native combat eligibility"),record(std::move(r)){}
    _value_type evaluate()override{return executable(*record,now_ms());}
};
std::uint8_t aimed_throw_reason(char const* s){
    if(!s)return throw_no_target;
    if(!xr_strcmp(s,"ok"))return throw_ok;
    if(!xr_strcmp(s,"no_grenade"))return throw_no_grenade;
    if(!xr_strcmp(s,"blocked"))return throw_blocked;
    if(!xr_strcmp(s,"no_arc"))return throw_no_arc;
    if(!xr_strcmp(s,"out_of_range"))return throw_out_of_range;
    if(!xr_strcmp(s,"door_behind"))return throw_door_behind;
    return throw_no_target;
}
// Shared with the outdoor capture path, defined in EngineOutdoorCombatCapture_impl.h.
bool outdoor_friends_clear(CAI_Stalker&,CGrenade&,Fvector const&,float);
// True reserves the weapon channel. No new projectile, forced animation, or
// renewed Fire1 on an order refresh. Arc work uses the canonical poll interval.
bool execute_throw(Record& r,Order const& o,TimeMs now,ExecutionPhase phase){
    auto& actor=*r.object;auto& f=r.throwing;auto slot=actor.inventory().ItemFromSlot(GRENADE_SLOT);
    auto current=actor.inventory().ActiveItem();bool armed=armed_item(current);
    if(phase==ExecutionPhase::fresh&&o.throw_order.valid&&o.throw_order.serial>f.serial&&!armed){
        if(f.started&&!f.ended)finish_receipts(r,throw_cancelled);
        auto grenade=smart_cast<CGrenade*>(slot);
        auto gun=smart_cast<CWeapon*>(current);
        if(grenade&&f.start(o,grenade->ID(),gun?gun->ID():invalid_id,now,r.snapshot.throw_state.releases)){
            r.throw_reported=false;begin_throw(r.key.id,f.serial);
        }else if(f.rejected_start(o,now,throw_no_grenade)){
            begin_throw(r.key.id,f.serial);end_throw(r.key.id,throw_no_grenade);r.throw_reported=true;
        }
    }
    if(!f.started||f.ended)return false;
    auto const& receipt=r.snapshot.throw_state;
    observe_throw_receipt(r,r.snapshot,now);
    if(phase!=ExecutionPhase::fresh||!f.same_request(o))f.fail(throw_cancelled);
    f.timeout(now,combat_config().room);
    if(f.terminal){
        if(f.reason&&!r.throw_reported){end_throw(r.key.id,f.reason);r.throw_reported=true;}
        if(armed&&current->object_id()==f.item){actor.set_painter_lean(0);return true;}
        finish_receipts(r,f.reason?f.reason:std::uint8_t(throw_cancelled));return true;
    }
    actor.set_painter_lean(0);
    if(f.missing_item(slot&&slot->object_id()==f.item))return true;
    // A submitted item normally disappears before capture sees its release.
    // Never turn that expected transition into a terminal no_grenade failure.
    if(f.submitted)return true;
    if(!f.equipped){actor.CObjectHandler::set_goal(eObjectActionIdle,slot);f.equipped=true;return true;}
    // Baseline must be captured before the Fire request can change inventory.
    if(!f.have_baseline)return true;
    if(!current||current->object_id()!=f.item)return true;
    if(armed){f.fail(throw_cancelled);return true;} // foreign/unrequested arming
    if(f.arc_due(now,combat_config().room)){
        auto target=native(f.target),via=native(f.via);
        actor.painter_throw(target,f.via_valid?&via:nullptr);
        bool clear=actor.throw_enabled();
        Evidence blast=Evidence::unknown;
        if(clear){
            // Check current native bodies immediately before submitting Fire1 for
            // room and outdoor throws. A worker/capture receipt may be stale here.
            auto grenade=smart_cast<CGrenade*>(current);
            if(grenade&&grenade->ID()==f.item)
                blast=outdoor_friends_clear(actor,*grenade,target,combat_config().outdoor.p.solo.throw_friend_m)
                    ?Evidence::yes:Evidence::no;
            if(blast!=Evidence::yes)
                Msg("[painter_combat] phase=throw_refused actor=%u serial=%u reason=friendly_blast_or_unknown evidence=pre_fire1",
                    unsigned(r.key.id),unsigned(f.serial));
        }
        if(authorize_throw_arc(f,clear,aimed_throw_reason(actor.painter_throw_reason()),now,combat_config().room,blast))
            actor.CObjectHandler::set_goal(eObjectActionFire1,slot);
    }
    return true;
}
bool execute_aid(Record& r,Order const& o,TimeMs now,ExecutionPhase phase){
    auto& a=r.aiding;
    if(phase!=ExecutionPhase::fresh||o.aid.id==invalid_id){
        if(a.cancel())note_aid(r.key.id,a.target,4);release_aid_claim(r);return false;
    }
    bool assignment_changed=a.set_target(o.aid,o.assignment.generation,now);
    auto target=find(o.aid);auto& actor=*r.object;
    bool valid=target&&registered(*target)&&target->object->g_Alive()&&!actor.is_relation_enemy(target->object)
        &&(target->object->wounded()||(flags_of(target->key.id)&flag_wounded_scheme))
        &&observed_aid_allows(*target,r,now);
    if(assignment_changed){
        release_aid_claim(r);resolve_aid_hooks();
        if(valid&&aid_claim&&aid_release&&aid_complete){
            r.aid_claimed=(*aid_claim)(actor.lua_game_object(),target->object->lua_game_object());
            if(r.aid_claimed)r.aid_claim_target=o.aid;
        }
    }
    valid=valid&&r.aid_claimed&&r.aid_claim_target==o.aid;
    // Inspect all current personal sight, not only native memory's selection.
    // The standing-beam exception belongs to one exact quiet enemy; a second
    // able hostile cannot inherit it, even if native selection stays hidden.
    const auto selected_enemy=actor.memory().enemy().selected();
    ActorKey beam_key;bool beam_firing=false;std::size_t visible_threats=0;
    auto inspect_aid_threat=[&](const CEntityAlive* enemy){
        if(!enemy||enemy->getDestroy()||!enemy->g_Alive()||!actor.is_relation_enemy(enemy)||
            !actor.memory().visual().visible_now(enemy))return;
        const auto stalker=smart_cast<CAI_Stalker const*>(enemy);
        if(stalker&&(stalker->wounded()||(flags_of(stalker->ID())&flag_wounded_scheme)))return;
        ++visible_threats;if(visible_threats==1)beam_key=key_of(*enemy);else beam_key={};
        // Inventory is read only after this exact enemy passes own-current-sight.
        if(const auto owner=smart_cast<CInventoryOwner const*>(enemy)){
            const auto weapon=smart_cast<CWeapon*>(owner->inventory().ActiveItem());
            beam_firing=beam_firing||(weapon&&(weapon->GetState()==CWeapon::eFire||weapon->GetState()==CWeapon::eFire2));
        }
    };
    inspect_aid_threat(selected_enemy);
    for(const auto& seen:actor.memory().visual().objects()){
        const auto enemy=smart_cast<const CEntityAlive*>(seen.m_object);
        if(enemy!=selected_enemy)inspect_aid_threat(enemy);
    }
    const bool threat=visible_threats!=0;
    for(std::size_t i=0;i<r.snapshot.hits.count&&i<r.snapshot.hits.items.size();++i)
        if(r.snapshot.hits.items[i].at_ms<=now)r.aid_last_hit=std::max(r.aid_last_hit,r.snapshot.hits.items[i].at_ms);
    const auto& aid_config=combat_config().room;
    bool reached=valid&&aid_reached(actor.Position().distance_to(target->object->Position()),aid_config);
    bool recent=aid_recent_hit(r.aid_last_hit,now,aid_config);
    a.update(now,valid,reached,threat,recent,aid_config,beam_key,beam_firing);
    if(a.take_reached()&&aid_arrived)(*aid_arrived)(actor.lua_game_object(),target->object->lua_game_object());
    if(a.ready(now,aid_config)){
        // The script catches failures and acknowledges stock wounded treatment;
        // completion here means aid GIVEN, never a fabricated recovery/health.
        bool success=aid_complete&&(*aid_complete)(actor.lua_game_object(),target->object->lua_game_object(),
            unsigned(a.beam_permitted?a.beam_enemy.id:invalid_id));
        a.completed(success);
    }
    if(a.phase>=3)release_aid_claim(r);
    note_aid(r.key.id,a.target,a.phase);
    return a.phase==2;
}
bool current_point_line(Record& r,const Order& order,CWeapon& weapon,TimeMs now){
    if(weapon.ID()!=r.point_fire.weapon)return false;
    static PointRayBudget quota;
    if(quota.needs(r.epoch,Device.dwFrame)){
        std::vector<ActorKey> eligible;
        for(const auto& entry:records){const auto& v=*entry.second;
            if(v.owned&&v.has_order&&registered(v)&&core::point_fire_authorized(v.point_fire,v.order,v.snapshot,now,
                combat_config().snapshot_max_age_ms,combat_config().memory_max_age_ms))eligible.push_back(v.key);
        }
        quota.begin(r.epoch,Device.dwFrame,combat_config().outdoor.r.authority_point_rays_per_frame,std::move(eligible));
    }
    if(!quota.take(r.key))return false;
    Fvector muzzle,actual,direction;r.object->g_fireParams(nullptr,muzzle,actual);
    direction.sub(native(order.fire.at),muzzle);const float range=direction.magnitude();
    if(!_valid(muzzle)||!_valid(direction)||!_valid(range)||range<=EPS)return false;
    if(!point_muzzle_aligned(value(actual),value(direction),::g_ai_aim_fire_angle))return false;
    direction.div(range);collide::rq_result hit;
    // Whole actual muzzle-to-point segment, not selected-enemy pick distance.
    // Every body/static obstruction vetoes this optional shared/shortened shot.
    return !Level().ObjectSpace.RayPick(muzzle,direction,range,collide::rqtBoth,hit,r.object);
}
// The visibility/finisher checks share one bounded fair grant per man/frame.
// Cached clear lines require exactly the current measured origin and target.
Evidence measured_look_line(Record& r,Vec3 origin,Vec3 point,TimeMs now,const CEntityAlive* target=nullptr){
    static PointRayBudget quota;
    return r.look_lines.line(origin,point,now,target?0:combat_config().snapshot_max_age_ms,r.geometry,[&](){
        if(quota.needs(r.epoch,Device.dwFrame)){
            std::vector<ActorKey> eligible;
            for(const auto& item:records)if(item.second->owned&&registered(*item.second))eligible.push_back(item.second->key);
            quota.begin(r.epoch,Device.dwFrame,combat_config().outdoor.r.authority_point_rays_per_frame,std::move(eligible));
        }
        if(!quota.take(r.key))return Evidence::unknown;
        Fvector ray;ray.sub(native(point),native(origin));const float length=ray.magnitude();
        if(!_valid(ray)||!_valid(length)||length<=EPS)return Evidence::unknown;
        ray.div(length);collide::rq_result hit;
        const bool blocked=Level().ObjectSpace.RayPick(native(origin),ray,length,target?collide::rqtBoth:collide::rqtStatic,hit,r.object);
        return !blocked||(target&&hit.O==target)?Evidence::yes:Evidence::no;
    },target?key_of(*target):ActorKey{});
}
bool opening_for_look(Record& r,const Order&o,Vec3 eye,Vec3 threat,Vec3& result){
    if(o.portal.valid){result=o.portal.centre;result.y=eye.y;return true;}
    const auto* cache=buildings();if(!cache||cache->level_epoch!=r.epoch||cache->map_hash!=captured_map)return false;
    float best=std::numeric_limits<float>::max();bool found=false;
    for(const auto& b:cache->buildings)if(b.id==r.snapshot.building_id)
        for(const auto& p:b.portals)if(p.traversable&&(p.room_a==r.snapshot.room_id||p.room_b==r.snapshot.room_id)){
            Vec3 at=p.centre;at.y=(std::max)(p.y0+.05f,(std::min)(p.y1-.05f,eye.y));
            const float cost=native(eye).distance_to(native(at))+native(at).distance_to(native(threat));
            if(cost<best){result=at;best=cost;found=true;}
        }
    return found;
}
DoorwayLook guarded_look(Record& r,const Order&o,DoorwayLook desired,bool visible,TimeMs now,bool retaining=false){
    if(visible||(!retaining&&authored_look_exception(o))||desired.kind==LookKind::path||desired.kind==LookKind::direction)return desired;
    if(!_valid(r.object->eye_matrix))return {};
    const Vec3 eye=value(r.object->eye_matrix.c);
    const auto direct=measured_look_line(r,eye,desired.value,now);
    // A committed opening/point is rechecked, not replaced with a different
    // fallback every frame. Unknown cannot retain a possibly occluded point.
    if(retaining)return direct==Evidence::yes?desired:bearing_look(desired,eye);
    Vec3 opening;const bool has=direct!=Evidence::yes&&opening_for_look(r,o,eye,desired.value,opening);
    const auto clear=has?measured_look_line(r,eye,opening,now):Evidence::unknown;
    return open_look(o,desired,false,direct,has,opening,clear,&eye);
}
const CEntityAlive* visible_finish_target(Record& r,ActorKey target){
    if(target.id==invalid_id)return nullptr;
    for(const auto& row:r.object->memory().visual().objects()){
        auto* entity=smart_cast<const CEntityAlive*>(row.m_object);
        if(entity&&!entity->getDestroy()&&entity->g_Alive()&&key_of(*entity)==target&&r.object->is_relation_enemy(entity)&&
            r.object->memory().visual().visible_now(entity))return entity;
    }
    return nullptr;
}
bool visible_finish_line(Record& r,const CEntityAlive* target,TimeMs now,bool require_aim=false){
    auto* npc=smart_cast<const CAI_Stalker*>(target);if(!npc||!npc->wounded()||!npc->g_Alive())return false;
    auto* weapon=smart_cast<CWeapon*>(r.object->inventory().ActiveItem());if(!weapon)return false;
    Fvector origin,direction,centre;r.object->g_fireParams(nullptr,origin,direction);target->Center(centre);
    if(require_aim){Fvector to;to.sub(centre,origin);if(!point_muzzle_aligned(value(direction),value(to),::g_ai_aim_fire_angle))return false;}
    return _valid(origin)&&_valid(centre)&&measured_look_line(r,value(origin),value(centre),now,target)==Evidence::yes;
}
void capture_finish_permissions(Record& r,ActorSnapshot& a,TimeMs now){
    // Contract's native finishing rule: no able enemy observed in the last 3s.
    // Only this observer's visual memory is consulted, including recent loss of sight.
    bool able=false;
    for(const auto& row:r.object->memory().visual().objects()){
        auto* entity=smart_cast<const CEntityAlive*>(row.m_object);
        if(!entity||entity->getDestroy()||!entity->g_Alive()||!r.object->is_relation_enemy(entity))continue;
        const auto* npc=smart_cast<const CAI_Stalker*>(entity);if(npc&&npc->wounded())continue;
        if(static_cast<u32>(Device.dwTimeGlobal-r.object->memory().visual().visible_object_time_last_seen(entity))<=3000){able=true;break;}
    }
    if(able){a.enemy.finish_allowed=false;for(std::size_t i=0;i<a.visible_enemies.count;++i)a.visible_enemies.items[i].finish_allowed=false;return;}
    const auto chosen=next_finish_contact(a,r.has_order?r.order.finish:ActorKey{},r.finish_after);
    r.finish_after=chosen;const auto* target=visible_finish_target(r,chosen);
    const bool allowed=target&&visible_finish_line(r,target,now);
    a.enemy.finish_allowed=allowed&&a.enemy.key==chosen;
    for(std::size_t i=0;i<a.visible_enemies.count;++i){auto& c=a.visible_enemies.items[i];c.finish_allowed=allowed&&c.key==chosen;}
}
bool finish_receipt_allows(Record&r,const Order&o,const CEntityAlive* enemy,TimeMs now){
    if(!enemy||o.finish!=key_of(*enemy)||o.role!=roles::finisher)return false;
    const auto allowed=[&](const Contact& c){return c.key==o.finish&&c.visible&&c.hostile&&c.alive_when_observed&&c.wounded&&c.finish_allowed;};
    bool permit=allowed(r.snapshot.enemy);
    for(std::size_t i=0;i<r.snapshot.visible_enemies.count;++i)permit=permit||allowed(r.snapshot.visible_enemies.items[i]);
    return permit&&visible_finish_line(r,enemy,now,true);
}
void record_applied(Record&r,const Order&o,TimeMs now,std::uint16_t weapon,std::uint8_t action,bool credit_fire=true){
    r.actual_executed=true;r.deadline_retry.executed();
    applied_action_receipts().applied(r.key,r.ownership,r.epoch,o.order_seq,now,weapon,action,credit_fire);
}
// Short direct steps use the actor's current restrictions and the AI graph's
// segment walk. No worker guess is promoted to a native route certificate.
bool executor_step(Record& r,Order const& o,MovementRequest& move,TimeMs now,bool busy,bool fresh){
    auto& actor=*r.object;auto enemy=actor.memory().enemy().selected();
    const bool visible=enemy&&enemy->g_Alive()&&actor.is_relation_enemy(enemy)&&actor.memory().visual().visible_now(enemy);
    if(!visible||busy||!fresh||o.aid.id!=invalid_id||o.finish.id!=invalid_id||o.kind==OrderKind::entry||o.kind==OrderKind::stack){r.reflex_until=0;return false;}
    auto& graph=ai().level_graph();auto& restrictions=actor.movement().restrictions();
    if(r.reflex_until>now&&graph.valid_vertex_id(r.reflex_node)&&restrictions.accessible(r.reflex_node)){
        move.node=r.reflex_node;move.position=r.reflex_goal;move.pace=Pace::walk;move.stance=Stance::stand;return true;
    }
    if(now<r.reflex_retry)return false;
    const bool arrived=actor.Position().distance_to(native(o.goal_position))<=o.arrival_radius_m;
    const auto plan=core::executor_step_plan(combat_config(),o,value(actor.Position()),value(enemy->Position()),
        visible,actor.can_kill_enemy(),actor.can_kill_member(),busy,arrived,!fresh);
    if(!plan.count)return false;
    r.reflex_retry=core::saturated_after(now,plan.lifetime);
    const auto start=actor.ai_location().level_vertex_id();if(!graph.valid_vertex_id(start))return false;
    for(unsigned i=0;i<plan.count;++i){auto goal=native(plan.candidates[i]);
        if(!graph.valid_vertex_position(goal))continue;
        auto node=graph.check_position_in_direction(start,actor.Position(),goal);
        if(!graph.valid_vertex_id(node)||!graph.inside(node,goal)||!restrictions.accessible(node)||!restrictions.accessible(goal))continue;
        goal.y=graph.vertex_plane_y(node,goal.x,goal.z);
        if(_abs(goal.y-actor.Position().y)>combat_config().room.floor_tolerance_m)continue;
        r.reflex_goal=value(goal);r.reflex_node=node;r.reflex_until=core::saturated_after(now,plan.lifetime);
        move.node=node;move.position=r.reflex_goal;move.pace=Pace::walk;move.stance=Stance::stand;return true;
    }
    return false;
}
class NativeAction final:public CStalkerActionCombatBase {
    RecordPtr record;MovementRequest previous;bool have_movement=false;CombatMembershipLease combat_registration;
public:
    NativeAction(CAI_Stalker* a,RecordPtr r):CStalkerActionCombatBase(a,"native combat order"),record(std::move(r)){}
    ~NativeAction()override{applied_action_receipts().clear(record->key);auto i=records.find(record->object);if(i!=records.end()&&i->second==record)records.erase(i);}
    void initialize()override{
        CActionScriptBase<CAI_Stalker>::initialize();object().brain().affect_cover(false);have_movement=false;record->cleanup_done=false;
        if(auto* members=combat_members(object())){
            if(combat_registration.begin(members->registered_in_combat(&object())))members->register_in_combat(&object());
            members->member(&object()).cover(nullptr);
        }else relinquish(*record,yield_offline);
    }
    void finalize()override{
        CActionScriptBase<CAI_Stalker>::finalize();
        // Native member removal handles dead/offline actors; never call mask()
        // after their membership has been removed by CEntity::net_Destroy.
        if(combat_registration.acquired&&registered(*record)&&object().g_Alive()){
            if(auto* members=combat_members(object()))
                if(combat_registration.finish(true,members->registered_in_combat(&object())))members->unregister_in_combat(&object());
        }
        combat_registration.finish(false,false);cleanup(*record);have_movement=false;
    }
    void execute()override{
        TimeMs now=now_ms();if(!executable(*record,now)){applied_action_receipts().clear(record->key);return;}
        CActionScriptBase<CAI_Stalker>::execute();auto& actor=object();auto const& o=record->has_order?record->order:record->provisional_order;
        auto& movement=actor.movement();auto phase=record->provisional?ExecutionPhase::fresh:execution_phase(o,now,combat_config().stale_release_ms);
        if(phase==ExecutionPhase::stale_hold&&ordinary_refresh_grace(o,record->snapshot,combat_config(),now))phase=ExecutionPhase::fresh;
        const bool stand_down=execution_stand_down(o,record->stand_down,phase==ExecutionPhase::fresh);
        if(phase==ExecutionPhase::stale_hold&&!record->stale_anchor){record->stale_anchor=true;record->hold_position=value(actor.Position());record->hold_node=actor.ai_location().level_vertex_id();}
        if(phase==ExecutionPhase::stale_hold&&(!ai().level_graph().valid_vertex_id(record->hold_node)
            ||!movement.restrictions().accessible(record->hold_node))){relinquish(*record,yield_invalid_navigation);return;}
        bool throw_hands=execute_throw(*record,o,now,phase);
        if(throw_hands){if(record->aiding.cancel())note_aid(record->key.id,record->aiding.target,4);release_aid_claim(*record);}
        bool aid_working=!throw_hands&&execute_aid(*record,o,now,phase);
        MovementRequest request;request.node=phase==ExecutionPhase::stale_hold?record->hold_node:o.goal_node;
        request.position=phase==ExecutionPhase::stale_hold?record->hold_position:o.goal_position;request.stance=stand_down?Stance::stand:o.stance;
        request.pace=phase==ExecutionPhase::fresh&&actor.Position().distance_to(native(request.position))>o.arrival_radius_m?o.pace:Pace::stand;
        if(treatment_stops_movement(throw_hands,record->aiding)||
            terminal_aid_stops_movement(o,record->aiding,phase)){
            request.pace=Pace::stand;request.stance=aid_working?Stance::crouch:Stance::stand;}
        // Terminal/refused treatment releases the hands and movement channel
        // immediately. Its one-shot ledger remains until a new assignment.
        executor_step(*record,o,request,now,throw_hands||aid_working||record->provisional||record->grenade_until>now||native_grenade_reaction(actor),phase==ExecutionPhase::fresh);
        bool changed=!have_movement||request!=previous;
        movement.force_update(changed);
        if(changed){
            auto destination=native(request.position);movement.set_mental_state(eMentalStateDanger);
            movement.set_body_state(request.stance==Stance::crouch?eBodyStateCrouch:eBodyStateStand);
            movement.set_path_type(MovementManager::ePathTypeLevelPath);movement.set_detail_path_type(DetailPathManager::eDetailPathTypeSmooth);
            movement.set_level_dest_vertex(request.node);movement.set_desired_position(request.pace==Pace::stand?nullptr:&destination);movement.set_desired_direction(nullptr);
            movement.set_movement_type(request.pace==Pace::stand?eMovementTypeStand:request.pace==Pace::run?eMovementTypeRun:eMovementTypeWalk);
            previous=request;have_movement=true;
        }
        if(throw_hands||aid_working||stand_down)actor.set_painter_lean(0);
        const float legacy_speed=stand_down?1.f:(o.flags&order_sprint)&&request.pace==Pace::run?
            combat_config().outdoor.p.movement.sprint_speed:o.speed_scale;
        if(throw_hands){actor.sight().setup(CSightAction(SightManager::eSightTypePosition,native(record->throwing.target),true));
            apply_order_gait(actor,o,request.pace,request.stance,true,false,true,legacy_speed);
            if(!record->provisional&&phase==ExecutionPhase::fresh)record_applied(*record,o,now,invalid_id,0);
            else applied_action_receipts().clear(record->key);return;}
        auto enemy=actor.memory().enemy().selected();
        // A hidden selected enemy must not mask another CURRENT personally
        // visible threat during a retained point/path order. Preserve a visible
        // authored target first, then visible selector continuity, otherwise
        // choose the closest able hostile from this observer's visual memory.
        VisibleLookChoice choice(o.look.kind==LookKind::enemy?o.look.target:ActorKey{},enemy?key_of(*enemy):ActorKey{});
        auto offer_visible=[&](const CEntityAlive* target){
            if(!target||target->getDestroy())return;
            const bool seen=actor.memory().visual().visible_now(target);
            if(!seen)return; // Never read an unseen object's current transform.
            const auto* stalker=smart_cast<const CAI_Stalker*>(target);
            if(choice.consider(key_of(*target),actor.Position().distance_to_sqr(target->Position()),seen,
                actor.is_relation_enemy(target),target->g_Alive(),stalker&&stalker->wounded()))enemy=target;
        };
        offer_visible(enemy);
        for(const auto& seen:actor.memory().visual().objects())offer_visible(smart_cast<const CEntityAlive*>(seen.m_object));
        if(o.finish.id!=invalid_id)if(auto* target=visible_finish_target(*record,o.finish))enemy=target;
        bool visible=enemy&&actor.memory().visual().visible_now(enemy);
        // Suppression is an authored time window. A currently visible enemy
        // is independent of that window, including while a plan briefly ages.
        const bool enemy_now=visible&&actor.is_relation_enemy(enemy)&&enemy->g_Alive();
        if(painter_voice::contact_rise(record->voice,record->key.generation,record->ownership,enemy_now))
            request_voice(*record,painter_voice::Cue::contact,StalkerSpace::eStalkerSoundAlarm,now);
        const bool hands_free=!aid_working&&!(flags_of(actor.ID())&flag_dont_shoot);
        const auto channels=fire_channels(o,now,phase,request.pace,hands_free,enemy_now);
        const bool target_window=channels.target;
        bool point_window=channels.point;
        auto desired=doorway_look(o,value(actor.Position()),visible&&actor.is_relation_enemy(enemy)&&enemy->g_Alive(),
            visible?key_of(*enemy):ActorKey{},visible?value(enemy->Position()):Vec3{});
        if(point_window&&o.fire.at_valid&&!visible)desired={LookKind::point,o.fire.at,{}};
        const auto look_events=record->look_episode.update(o,record->snapshot,value(actor.Position()),now,combat_config().snapshot_max_age_ms);
        // The geometric sweep must win on every frame, including FORM->PIE
        // while moving. Retaining the old portal-centre aim makes the shoulder
        // cross product zero; retaining a path fallback also cancels the lean.
        // This affects aim only. Current sight and firing checks stay separate.
        if(doorway_owns_look(o))record->look_commitment.reset();
        const auto committed=record->look_commitment.prepare({desired.kind,desired.value,desired.target},now,enemy_now,
            look_events.hit,look_events.arrival,look_events.door_cross,request.pace!=Pace::stand);
        const auto measured=guarded_look(*record,o,{committed.kind,committed.value,committed.target},enemy_now,now,record->look_commitment.retaining());
        const bool bearing_only=measured.kind==LookKind::direction&&
            (committed.kind==LookKind::point||committed.kind==LookKind::enemy);
        const auto retained=record->look_commitment.commit_safe({measured.kind,measured.value,measured.target},bearing_only);
        const DoorwayLook look{retained.kind,retained.value,retained.target};
        if(point_window&&o.fire.at_valid&&!visible&&(look.kind!=LookKind::point||!same_value(look.value,o.fire.at)))point_window=false;
        const bool window=target_window||point_window;
        Vec3 aim=look.value;
        if(target_window){aim=value(enemy->Position());actor.sight().setup(CSightAction(enemy,true,true,false,/*predict_motion=*/false));}
        else if((o.flags&order_face_movement)&&request.pace==Pace::run)
            actor.sight().setup(CSightAction(SightManager::eSightTypePathDirection,true,true));
        else if(point_window&&o.fire.at_valid&&!visible){aim=o.fire.at;actor.sight().setup(CSightAction(SightManager::eSightTypePosition,native(aim),true));}
        else if(look.kind==LookKind::enemy&&visible&&key_of(*enemy)==look.target){aim=value(enemy->Position());actor.sight().setup(CSightAction(enemy,true,true,false,/*predict_motion=*/false));}
        else if(look.kind==LookKind::point||look.kind==LookKind::enemy)actor.sight().setup(CSightAction(SightManager::eSightTypePosition,native(aim),true));
        else if(look.kind==LookKind::direction&&native(aim).square_magnitude()>.0001f)actor.sight().setup(CSightAction(SightManager::eSightTypeDirection,native(aim),true));
        else actor.sight().setup(CSightAction(SightManager::eSightTypePathDirection,true,true));
        const DoorwayLook attained_look{target_window?LookKind::point:look.kind,aim,look.target};
        actor.set_painter_lean(throw_hands||aid_working||stand_down?0:doorway_lean(o,value(actor.Position()),attained_look,request.pace));
        auto weapon=smart_cast<CWeapon*>(actor.inventory().ActiveItem());
        const bool native_firing=weapon&&(weapon->GetState()==CWeapon::eFire||weapon->GetState()==CWeapon::eFire2);
        apply_order_gait(actor,o,request.pace,request.stance,throw_hands||aid_working||stand_down||!weapon||
            o.kind==OrderKind::reload||(weapon&&weapon->GetState()==CWeapon::eReload),
            native_firing,enemy_now||look.kind!=LookKind::path||look.target.id!=invalid_id,legacy_speed);
        if(!weapon){
            if(!record->provisional&&phase==ExecutionPhase::fresh)record_applied(*record,o,now,invalid_id,0);
            else applied_action_receipts().clear(record->key);return;}
        EObjectAction action=eObjectActionAimReady1;
        const bool reloading=weapon->GetState()==CWeapon::eReload;
        const bool reload_requested=weapon_reload_request(o,phase,aid_working,reloading,weapon->GetAmmoElapsed(),weapon->ID(),
            record->snapshot.weapon.id,record->snapshot.weapon.ammo_reserve);
        const bool try_target=evaluate_target_before_reload(reload_requested,reloading,weapon->GetAmmoElapsed(),target_window);
        if(reload_requested&&!try_target)action=eObjectActionReload1;
        else if(window){
            float yaw,pitch;Fvector().sub(native(aim),actor.Position()).getHP(yaw,pitch);
            float error=angle_difference(-yaw,movement.head_orientation().current.yaw);
            if(point_window&&o.fire.at_valid&&!visible){
                float floor=o.fire.ammo_floor*float(weapon->GetAmmoMagSize());
                const bool known=known_fire_point(record->snapshot,o,now,combat_config().memory_max_age_ms);
                const bool shared=core::point_fire_authorized(record->point_fire,o,record->snapshot,now,
                    combat_config().snapshot_max_age_ms,combat_config().memory_max_age_ms)
                    &&same_value(record->point_fire.origin,value(actor.Position()));
                if(suppression_fire(known||shared,reloading,weapon->GetAmmoElapsed()>floor,actor.can_kill_member(),error,::g_ai_aim_fire_angle)
                    &&(known||current_point_line(*record,o,*weapon,now)))action=eObjectActionFire1;
            }else if(target_window){
                const bool exact=true;
                auto target=smart_cast<CAI_Stalker const*>(enemy);bool wounded=target&&target->wounded();
                bool finish=wounded&&finish_receipt_allows(*record,o,enemy,now);
                const u32 native_flags=finish?0:actor.painter_target_fire_flags(enemy);
                if(targeted_fire(visible,exact,actor.is_relation_enemy(enemy),enemy->g_Alive(),wounded,finish,reloading,
                    weapon->GetAmmoElapsed()>0,finish||bool(native_flags&1),finish||bool(native_flags&2),finish?false:bool(native_flags&4),error,::g_ai_aim_fire_angle))action=eObjectActionFire1;
            }
        }
        // A merely visible target is insufficient: keep the tactical reload
        // unless all native friendly, sightline, ammo and aim gates allowed Fire1.
        if(reload_requested&&try_target&&action!=eObjectActionFire1)action=eObjectActionReload1;
        // Sight, movement and weapon requests are independent; a fire request
        // never clears the active movement goal or creates a forced animation.
        actor.CObjectHandler::set_goal(action,static_cast<CInventoryItem*>(weapon),o.fire.queue,o.fire.queue,o.fire.interval_ms,o.fire.interval_ms);
        // Applied goals, not physical arrival or requested pose attainment.
        if(!record->provisional&&phase==ExecutionPhase::fresh)record_applied(*record,o,now,weapon->ID(),
            action==eObjectActionFire1?2:action==eObjectActionReload1?3:1,
            !target_window||(enemy&&((o.look.kind==LookKind::enemy&&o.look.target==key_of(*enemy))||o.finish==key_of(*enemy))));
        else applied_action_receipts().clear(record->key);
    }
};
RecordPtr find(ActorKey key){for(auto const& item:records)if(item.second->key==key)return item.second;return {};}
void feedback(Record& r,Order const& o,std::uint8_t why){
    if(o.order_seq<r.rejected.order_seq)return;
    r.rejected.order_seq=o.order_seq;r.rejected.node=feedback_node(why,o.goal_node);r.rejected.reason=why;r.rejected.restrictions_revision=r.restrictions;
}
}
#include "EngineRouteAuthority.inl"
void register_actor(CStalkerPlanner& planner,CAI_Stalker& actor){
    auto const& ops=planner.operators();auto op=std::lower_bound(ops.begin(),ops.end(),ids::action);
    if(op!=ops.end()&&op->m_operator_id==ids::action)return;
    auto const& eval=planner.evaluators();if(eval.find(ids::eligibility)!=eval.end())return;
    auto old=records.find(&actor);if(old!=records.end())relinquish(*old->second,yield_released);
    auto r=std::make_shared<Record>();r->object=&actor;r->key={actor.ID(),next_generation.fetch_add(1)};records[&actor]=r;
    planner.add_evaluator(ids::eligibility,xr_new<Eligibility>(&actor,r));auto action=xr_new<NativeAction>(&actor,r);
    action->add_condition(CWorldProperty(ids::eligibility,true));action->add_effect(CWorldProperty(ids::eligibility,false));planner.add_operator(ids::action,action);
}
void set_enabled(bool on){if(!on)release_all(yield_disabled);service_enabled=on;}
bool enabled(){return service_enabled;}
bool owns(CAI_Stalker const& actor){auto r=records.find(const_cast<CAI_Stalker*>(&actor));return r!=records.end()&&r->second->owned&&actor.painter_forced_action()==ids::action;}
bool actor_key(CAI_Stalker const& actor,ActorKey& key){
    auto r=records.find(const_cast<CAI_Stalker*>(&actor));if(r==records.end()||!registered(*r->second))return false;key=r->second->key;return true;}
void compat_handback(CAI_Stalker& actor,std::uint32_t reason){auto r=records.find(&actor);if(r!=records.end())relinquish(*r->second,reason);}
void release_all(std::uint32_t reason){friendly_capture={};friendly_sides.clear();friendly_offsets.clear();friendly_after_observer=invalid_id;for(auto& r:records)relinquish(*r.second,reason);compatibility_gate.reset();compatibility_resolved=false;
    aid_complete.reset();aid_claim.reset();aid_release.reset();aid_arrived.reset();aid_resolved=false;}
void object_destroyed(CGameObject& object){
    auto entity=smart_cast<CEntityAlive*>(&object);if(entity)perceived.erase(entity);
    auto actor=smart_cast<CAI_Stalker*>(&object);auto r=records.find(actor);if(r!=records.end()){relinquish(*r->second,yield_offline);records.erase(r);}
    forget(object.ID()); // cleanup receipts first; never recreate them after this ID is forgotten
}
void restrictions_changed(CAI_Stalker& actor){auto r=records.find(&actor);if(r!=records.end()){++r->second->restrictions;relinquish(*r->second,yield_invalid_navigation);}}
void notify_committed_sever(CPhysicsShellHolder& subject,std::uint64_t revision,std::uint32_t component){
    // The producer invokes this only from damaged_fragment::poll after its
    // geometry publish barrier and same-thread check. Visual memory is an
    // existing cached lookup, not a new ray or a worker-thread engine query.
    if(!Device.IsGeometryPublishBarrier())return;
    const auto* victim=smart_cast<const CAI_Stalker*>(&subject);
    if(!victim||subject.getDestroy()||!revision)return;
    const ActorKey target=key_of(*victim);if(!core::valid_key(target))return;
    const TimeMs now=Device.dwTimeGlobal;
    static std::uint32_t after_id=0;
    std::vector<Record*> candidates;
    for(auto& entry:records){auto& witness=*entry.second;
        if(registered(witness)&&witness.object->g_Alive()&&witness.object!=victim&&
            witness.object->Community()==victim->Community()&&!witness.object->is_relation_enemy(victim))
            candidates.push_back(&witness);
    }
    std::sort(candidates.begin(),candidates.end(),[](const Record* a,const Record* b){return a->key.id<b->key.id;});
    if(candidates.empty())return;
    auto start=std::upper_bound(candidates.begin(),candidates.end(),after_id,
        [](std::uint32_t id,const Record* r){return id<r->key.id;});
    const auto start_at=start==candidates.end()?std::size_t(0):std::size_t(start-candidates.begin());
    const auto count=(std::min)(std::size_t(32),candidates.size());
    for(std::size_t index=0;index<count;++index){
        auto& witness=*candidates[(start_at+index)%candidates.size()];after_id=witness.key.id;
        if(!witness.object->memory().visual().visible_now(victim))continue;
        const auto duplicate=std::find_if(witness.witnessed_severs.begin(),witness.witnessed_severs.end(),[&](const ActorSnapshot::WitnessedSever& e){
            return e.subject==target&&e.revision==revision&&e.component==component;});
        if(duplicate!=witness.witnessed_severs.end())continue;
        if(witness.witnessed_severs.size()==4)witness.witnessed_severs.pop_front();
        witness.witnessed_severs.push_back({target,revision,witness.ownership,component,now});
    }
}
void capture_friendly_casualties(const WorldSnapshot& world,const std::vector<SideSnapshot>& known,
    core::FriendlyCasualtyEvidence& out,std::size_t work_limit){
    out={};out.level_epoch=world.level_epoch;out.snapshot_seq=world.sequence;out.map_hash=world.map_hash;
    out.geometry_revision=world.geometry_revision;out.captured_at_ms=world.now_ms;
    friendly_capture={};friendly_sides.clear();
    if(!work_limit||world.actors.empty())return;
    // No known-side or patient object pointer crosses this function.
    std::vector<const ActorSnapshot*> observers;observers.reserve(world.actors.size());
    for(const auto& a:world.actors)if(a.alive&&!a.wounded&&(a.flags&flag_admitted)&&a.ownership_generation)observers.push_back(&a);
    std::sort(observers.begin(),observers.end(),[](const ActorSnapshot* a,const ActorSnapshot* b){return a->key.id<b->key.id;});
    if(observers.empty())return;
    auto first=std::upper_bound(observers.begin(),observers.end(),friendly_after_observer,
        [](std::uint32_t id,const ActorSnapshot* a){return id<a->key.id;});
    const std::size_t start=first==observers.end()?0:std::size_t(first-observers.begin());
    std::set<std::uint32_t> present;for(const auto* a:observers)present.insert(a->key.id);
    for(auto i=friendly_offsets.begin();i!=friendly_offsets.end();)if(!present.count(i->first))i=friendly_offsets.erase(i);else ++i;
    for(std::size_t visit=0;visit<observers.size()&&work_limit;++visit){
        const auto& a=*observers[(start+visit)%observers.size()];friendly_after_observer=a.key.id;--work_limit;
        auto record=find(a.key);if(!record||!registered(*record)||record->ownership!=a.ownership_generation)continue;
        auto& actor=*record->object;const SideSnapshot* side=nullptr;
        for(const auto& s:known)if(s.faction==a.faction&&(!a.native_squad||s.group==a.native_squad)
            &&std::find(s.members.begin(),s.members.end(),a.key)!=s.members.end()){
            if(side){side=nullptr;break;}side=&s;}
        if(!side)continue;
        const auto& visual=actor.memory().visual().objects();auto& cursor=friendly_offsets[a.key.id];
        if(cursor.key!=a.key){cursor.key=a.key;cursor.offset=0;}
        if(cursor.offset>=visual.size())cursor.offset=0;
        std::size_t visited=0;const auto offset=cursor.offset;
        while(offset+visited<visual.size()&&work_limit){const auto& seen=visual[offset+visited];--work_limit;++visited;
            auto* patient=smart_cast<CAI_Stalker const*>(seen.m_object);
            // Visibility precedes current target position, life or script reads.
            if(!patient||patient==&actor||patient->getDestroy()||!actor.memory().visual().visible_now(patient))continue;
            const auto target=key_of(*patient);
            if(std::find(side->members.begin(),side->members.end(),target)==side->members.end()
                ||actor.is_relation_enemy(patient)||patient->Community()!=a.faction)continue;
            core::FriendlyCasualtyFact f;f.observer=a.key;f.observer_owner=a.ownership_generation;
            f.observer_restrictions=a.restrictions_revision;f.target=target;f.side_id=side->side_id;
            f.faction=side->faction;f.group=side->group;f.mission_revision=side->mission.revision;
            f.target_native_squad=side_key(*patient);f.observed_at_ms=a.observed_at_ms;
            f.position=value(patient->Position());f.visible_now=true;f.friendly=Evidence::yes;
            f.alive=patient->g_Alive();f.wounded=f.alive&&(patient->wounded()||(flags_of(target.id)&flag_wounded_scheme));
            f.aid_eligibility=aid_eligibility_of(target.id,world.now_ms,key_of);out.facts.push_back(f);
        }
        cursor.offset=offset+visited;if(cursor.offset>=visual.size())cursor.offset=0;
    }
    friendly_capture=out;friendly_sides=known;
}
void capture_all(WorldSnapshot& world){capture_all(world,nullptr);}
void capture_all(WorldSnapshot& world,std::vector<core::OutdoorCapturedEye>* measured_eyes){
    capture_all(world,measured_eyes,nullptr);
}
void capture_all(WorldSnapshot& world,std::vector<core::OutdoorCapturedEye>* measured_eyes,core::PerceivedSpeciesEvidence* species){
    if(measured_eyes)measured_eyes->clear();
    if(captured_epoch!=world.level_epoch){release_all(yield_offline);side_sequences.clear();side_postures.clear();reported_styles.clear();lifecycle_cooldowns.clear();error_cooldowns.clear();lifecycle_sequence=0;captured_epoch=world.level_epoch;}
    captured_sequence=world.sequence;captured_geometry=world.geometry_revision;captured_map=world.map_hash;
    error_cooldowns.erase(std::remove_if(error_cooldowns.begin(),error_cooldowns.end(),[&](const auto& c){return c.until_ms<=world.now_ms;}),error_cooldowns.end());
    world.actors.clear();
    if(species){*species={};species->level_epoch=world.level_epoch;species->snapshot_seq=world.sequence;species->map_hash=world.map_hash;
        species->geometry_revision=world.geometry_revision;species->captured_at_ms=world.now_ms;species->memory_max_age_ms=combat_config().memory_max_age_ms;}
    // One own-memory pass, reused below. Nonzero native group/faction is actual
    // teammate provenance, never a hostile roster or shared firing permission.
    std::map<std::uint16_t,core::ObserverSpecies> species_rows;
    std::set<std::pair<std::uint64_t,std::uint32_t>> human_groups;
    near_miss_watch_begin(); // observation roster is independent of control admission
    if(service_enabled)for(const auto& entry:records){const auto& r=*entry.second;
        if(!registered(r)||!r.object->g_Alive())continue;
        const Fvector eye=r.object->eye_matrix.c;
        if(_valid(r.object->eye_matrix)&&_valid(eye)&&!eye.similar(Fvector().set(0,0,0)))near_miss_watch(std::uint16_t(r.key.id),eye);
    }
    if(service_enabled)for(const auto& entry:records){auto& r=*entry.second;if(!registered(r)||!r.object->g_Alive())continue;
        CaptureContext c;c.key=r.key;c.level_epoch=world.level_epoch;c.snapshot_seq=world.sequence;c.ownership_generation=r.ownership;
        c.restrictions_revision=r.restrictions;c.now_ms=world.now_ms;c.key_of=key_of;
        core::ObserverSpecies row;capture_species(*r.object,c,combat_config(),row,r.epoch==world.level_epoch?&r.species_memory:nullptr);
        r.species_memory=row;
        const auto group=side_key(*r.object);
        if(group&&core::perceived_human(row,world.now_ms,combat_config()))human_groups.emplace(group,std::uint32_t(r.object->Community()));
        species_rows.emplace(r.key.id,std::move(row));
    }
    for(auto& entry:records){
        auto& r=*entry.second;if(!registered(r))continue;auto& actor=*r.object;
        if(r.epoch!=world.level_epoch){r.snapshots.clear();r.witnessed_severs.clear();r.last_order=0;r.previous_snapshot=0;r.compat_at=0;r.rejected={};r.reported=false;r.deadline_retry={};}
        r.epoch=world.level_epoch;r.geometry=world.geometry_revision;
        if(actual_first_order_overdue(r.admitted_at,world.now_ms,combat_config().first_order_deadline_ms,r.owned,r.actual_executed)){
            r.side_release_reason="first_order_deadline";relinquish(r,yield_no_first_order);
        }
        const bool alive=!!actor.g_Alive();
        const bool requested=(flags_of(actor.ID())&flag_requested_control)!=0;
        const auto species_row=species_rows.find(r.key.id);
        const auto group=side_key(actor);
        TimeMs own_hit_at=0;
        if(species_row!=species_rows.end())own_hit_at=own_hit_admission(actor.memory().hit().physical_hits(),world.now_ms,combat_config(),species_row->second,
            [](const void* raw,std::uint32_t id){const auto* object=static_cast<const CObject*>(raw);
                if(!object||object->ID()!=id)return ActorKey{};
                const auto* entity=smart_cast<const CEntityAlive*>(object);return entity?key_of(*entity):ActorKey{};});
        TimeMs event_at=0;
        if(alive&&species_row!=species_rows.end()){
            CaptureContext c;c.key=r.key;c.now_ms=world.now_ms;c.previous_snapshot_ms=r.previous_snapshot;c.key_of=key_of;
            ActorSnapshot events;capture_personal_events(actor,c,events,false);
            event_at=personal_event_admission(events,world.now_ms,combat_config(),species_row->second);
        }
        const bool own_fight=species_row!=species_rows.end()&&
            (core::perceived_combat_admission(species_row->second,world.now_ms,combat_config())||own_hit_at!=0||event_at!=0);
        const bool eligible_fight=own_fight||!r.witnessed_severs.empty()||(group&&human_groups.count({group,std::uint32_t(actor.Community())}));
        const bool contact=service_enabled&&alive&&!r.owned&&!requested&&eligible_fight;
        ActorSnapshot admission_actor=r.snapshot;admission_actor.level_epoch=world.level_epoch;admission_actor.key=r.key;admission_actor.faction=std::uint32_t(actor.Community());admission_actor.native_squad=group;
        const bool side_cooling=core::lifecycle_admission_blocked(lifecycle_cooldowns,admission_actor,world.level_epoch,world.map_hash,world.now_ms)||
            core::lifecycle_admission_blocked(error_cooldowns,admission_actor,world.level_epoch,world.map_hash,world.now_ms);
        const bool deadline_cooling=!r.owned&&r.deadline_retry.cooling(world.now_ms);
        const auto admission=capture_admission(service_enabled&&!side_cooling&&!deadline_cooling,alive,r.owned,r.has_order,requested,contact,r.handback.reasons!=yield_none,r.reported);
        if(!admission.capture){
            if(r.snapshot.key.id!=invalid_id){r.snapshots.clear();r.snapshot={};}
            // No observation was consumed: retain the watermark across cooldown / ineligible ticks.
            r.compat_at=0;r.compatibility=yield_scheme_block;continue;
        }
        if(combat_config().compat_mode==0&&admission.refresh_compatibility&&(!r.compat_at||world.now_ms<r.compat_at||world.now_ms-r.compat_at>=compatibility_interval(combat_config()))){
            r.compatibility=bridge(actor);r.compat_at=world.now_ms;
        }
        CaptureContext c;c.key=r.key;c.level_epoch=world.level_epoch;c.snapshot_seq=world.sequence;c.ownership_generation=r.ownership;
        c.restrictions_revision=r.restrictions;c.now_ms=world.now_ms;c.previous_snapshot_ms=r.previous_snapshot;
        c.current_goal_node=r.has_order?r.order.goal_node:invalid_node;c.key_of=key_of;c.buildings=buildings();c.last_rejected=r.rejected;
        c.requested_control=admission.admitted; // v2.11 Capture maps this to admitted, preserving explicit mission bit
        ActorSnapshot a;capture_actor(actor,c,a);
        while(!r.witnessed_severs.empty()){
            const auto event=r.witnessed_severs.front();r.witnessed_severs.pop_front();
            if(event.observer_owner!=r.ownership||event.at_ms>world.now_ms||world.now_ms-event.at_ms>1000||
                a.witnessed_severs.count>=a.witnessed_severs.items.size())continue;
            a.witnessed_severs.items[a.witnessed_severs.count++]=event;
        }
        if(a.alive&&r.owned)capture_finish_permissions(r,a,world.now_ms);
        if(r.owned&&actor.painter_forced_action()==ids::action)a.last_applied=applied_action_receipts().capture(r.key,r.ownership,r.epoch,world.now_ms);
        else applied_action_receipts().clear(r.key);
        observe_throw_receipt(r,a,world.now_ms);
        if(a.alive&&native_grenade_reaction(actor)&&!armed_item(actor.inventory().ActiveItem()))a.yield_reasons&=~std::uint32_t(yield_grenade);
        if(a.alive)a.yield_reasons=adjusted_item_yields(r,a.yield_reasons);
        // v2.21 push mode: an admission demand gets the one validation walk of his registered evaluators (no complete pass
        // of his sealed set yet, or a mutation / perception change since) - never a timer
        if(combat_config().compat_mode!=0&&admission.admitted&&!r.owned&&!r.has_order&&!r.provisional&&a.alive)
            compat_admission_demand(actor,r.key,perception_signature(a));
        a.compat=compat_state(r.key.id,r.key);
        a.yield_reasons|=blockers(r);
        if(admission.admitted&&!r.owned&&!r.has_order&&!r.provisional&&a.alive)
            compat_note_admission(actor,r.key,combat_config().compat_mode!=0);   // diagnostics: counts, push-mode refusals logged
        if(a.yield_reasons!=yield_none)relinquish(r,a.yield_reasons);
        a.yield_reasons|=r.handback.reasons;
        // A man handed back stays with the game's own logic for readmit_delay_ms: no release / re-admission oscillation
        const bool cooling=!r.owned&&!r.has_order&&!r.provisional&&r.released_at&&world.now_ms>=r.released_at&&
            world.now_ms-r.released_at<combat_config().readmit_delay_ms;
        const bool admitted=admission.admitted&&a.yield_reasons==yield_none&&!cooling&&!side_cooling;
        a.flags=capture_flags(a.flags,admitted);
        if(!admitted)a.last_applied={};
        if(admitted&&a.enemy.visible&&a.enemy.hostile&&a.enemy.alive_when_observed&&a.enemy.last_seen_ms&&finite_value(a.enemy.position)){r.outdoor_seen=a.enemy;r.outdoor_seen_at=world.now_ms;}
        else if(r.outdoor_seen_at&&a.enemy.key==r.outdoor_seen.key&&(!a.enemy.hostile||!a.enemy.alive_when_observed)){r.outdoor_seen={};r.outdoor_seen_at=0;}
        if(admitted&&!r.owned&&!r.has_order&&!r.provisional){
            if(!a.current_position_accessible||!finite_value(a.position)||!ai().level_graph().valid_vertex_id(a.node_id))a.yield_reasons|=yield_invalid_navigation;
            else{
                r.provisional_order=provisional_hold(a,world.now_ms,combat_config().memory_max_age_ms);
                r.side_release_reason="none";r.release_side=0;r.stand_down=false;r.actual_executed=false;r.admitted_at=world.now_ms;r.provisional=true;r.owned=true;r.cleanup_done=false;
                actor.set_painter_forced_action(ids::action);
            }
        }
        if(a.yield_reasons!=yield_none)a.flags&=~std::uint32_t(flag_admitted);
        a.ownership_generation=r.ownership;a.last_rejected=r.rejected;r.snapshot=a;r.previous_snapshot=world.now_ms;
        r.snapshots.push_back({world.sequence,world.now_ms,r.ownership,r.restrictions});
        while(!r.snapshots.empty()&&(world.now_ms<r.snapshots.front().at||world.now_ms-r.snapshots.front().at>combat_config().snapshot_max_age_ms
            ||r.snapshots.size()*sizeof(SourceStamp)>8192))r.snapshots.pop_front();
        world.actors.push_back(a);
        if(species&&species_row!=species_rows.end()){
            auto row=std::move(species_row->second);row.ownership_generation=a.ownership_generation;
            row.restrictions_revision=a.restrictions_revision;species->observers.push_back(std::move(row));
        }
        // Read the raw matrix only after final lease/flags are captured. The
        // unrelated near-miss fallback in EngineCapture remains unchanged.
        if(measured_eyes&&a.alive){
            const Fvector raw_eye=actor.eye_matrix.c;
            core::OutdoorCapturedEye eye;
            if(measured_outdoor_eye(world,a,{raw_eye.x,raw_eye.y,raw_eye.z},_valid(actor.eye_matrix),eye))
                measured_eyes->push_back(eye);
        }
        r.reported=(a.flags&flag_admitted)!=0;r.handback.consume();
    }
}
PublishResult publish(Decision const& d){return publish_points(d,{});}
PublishResult publish_points(Decision const& d,const std::vector<core::PointFireAuthority>& points){
    PublishResult result;TimeMs now=now_ms();
    if(points.size()>d.orders.size()){result.envelope=reject_order_invalid;return result;}
    std::map<std::pair<std::uint32_t,std::uint64_t>,const core::PointFireAuthority*> proofs;
    for(const auto& point:points)if(!proofs.emplace(std::make_pair(point.authored.actor.id,point.authored.actor.generation),&point).second){
        result.envelope=reject_order_invalid;return result;}

    if(d.level_epoch!=captured_epoch||d.level_epoch!=level_epoch())result.envelope=reject_epoch;
    else if(d.geometry_revision!=captured_geometry)result.envelope=reject_geometry;
    else{
        const auto seen=side_sequences.find(d.side_id);
        if(d.snapshot_seq>captured_sequence||(seen!=side_sequences.end()&&d.snapshot_seq<seen->second.sequence))result.envelope=reject_older_snapshot;
    }
    if(result.envelope){for(auto const& o:d.orders){auto r=find(o.actor);if(r)feedback(*r,o,result.envelope);}return result;}
    decision_cost_us+=d.cost_us;++decision_count;
    // Diagnostic events are worker decisions, not claims of native application.
    // Log only after envelope validation; publication results below still decide
    // whether any proposed order can own the actor.
    static const char* event_names[]={"none","first_response","cover_pick","stack_formed","entry_released",
        "room_cleared","mission_handover","squad_released","lean","suppress","casualty","hand_back","search_loss_priced","search_viewpoint"};
    for(const auto& event:d.events)
        Msg("[painter_combat] phase=decision_event event=%s kind=%u side=%u source=%llu room=%u detail=%u",
            event.kind<sizeof(event_names)/sizeof(event_names[0])?event_names[event.kind]:"unknown",
            unsigned(event.kind),d.side_id,d.snapshot_seq,event.room,event.detail);
    for(auto o:d.orders){
        auto r=find(o.actor);std::uint8_t reason=reject_none;std::uint32_t yields=yield_none;
        const auto pi=proofs.find(std::make_pair(o.actor.id,o.actor.generation));
        const auto* point=pi==proofs.end()?nullptr:pi->second;
        const bool point_valid=point&&r&&registered(*r)&&core::point_fire_authorized(*point,o,r->snapshot,now,
            combat_config().snapshot_max_age_ms,combat_config().memory_max_age_ms)
            &&same_value(point->origin,value(r->object->Position()));
        auto aid_target=o.aid.id!=invalid_id?find(o.aid):RecordPtr{};
        unsigned duplicates=0;for(auto const& other:d.orders)if(other.actor==o.actor)++duplicates;for(auto key:d.releases)if(key==o.actor)++duplicates;
        if(!r||!registered(*r)||duplicates!=1)reason=reject_generation;
        else if(o.level_epoch!=d.level_epoch)reason=reject_epoch;
        else if(o.geometry_revision!=d.geometry_revision)reason=reject_geometry;
        else if(o.snapshot_seq!=d.snapshot_seq)reason=reject_older_snapshot;
        else if(o.ownership_generation!=r->ownership||o.restrictions_revision!=r->restrictions)reason=reject_ownership;
        else if(o.order_seq<=r->last_order)reason=reject_order_seq;
        else if(o.issued_at_ms>now)reason=reject_issued_in_future;
        if(!reason){
            auto source=std::find_if(r->snapshots.begin(),r->snapshots.end(),[&](auto const& s){return s.sequence==o.snapshot_seq;});
            if(source==r->snapshots.end()||!source_stamp_valid(*source,o.snapshot_seq,r->ownership,r->restrictions,o.issued_at_ms,now,combat_config().snapshot_max_age_ms))reason=reject_older_snapshot;
        }
        if(!reason&&o.kind==OrderKind::yield){relinquish(*r,yield_released);r->last_order=o.order_seq;r->rejected={};result.accepted_releases.push_back(o.actor);continue;}
        if(!reason){
            yields=blockers(*r);
            // Recheck the optional fire channel at the live publication instant,
            // as the Mailbox already does at preflight. The actor may have moved
            // since a point-fire proof; that never grants fire from the new origin.
            constrain_publication_fire(o,now,known_fire_point(r->snapshot,o,now,combat_config().memory_max_age_ms),point_valid);
            if(yields!=yield_none)reason=reject_ownership;
            else if(!executor_metadata_valid(o))reason=reject_order_invalid;
            else if(o.throw_order.valid&&o.throw_order.serial>r->throwing.serial&&armed_item(r->object->inventory().ActiveItem()))reason=reject_order_invalid;
            else if(o.throw_order.valid&&o.throw_order.serial==r->throwing.serial&&!r->throwing.same_request(o))reason=reject_order_invalid;
            else if(o.throw_order.valid&&o.throw_order.serial<r->throwing.serial)reason=reject_order_seq;
            else if(o.aid.id!=invalid_id&&!aid_target)reason=reject_generation;
            else if(aid_target&&!observed_aid_allows(*aid_target,*r,now))reason=reject_order_invalid;
            else if(!ai().level_graph().valid_vertex_id(o.goal_node)||!finite_value(o.goal_position)
                ||(!ai().level_graph().inside(o.goal_node,native(o.goal_position))&&
                    !current_stationary_anchor(o,r->object->ai_location().level_vertex_id(),value(r->object->Position()))))reason=reject_goal_invalid;
            else if(!core::publishable(o,r->snapshot,now,r->epoch,r->geometry))reason=reject_order_invalid;
            else if(o.fire.allowed&&o.fire.at_valid&&!known_fire_point(r->snapshot,o,now,combat_config().memory_max_age_ms)&&!point_valid)reason=reject_fire_point;
            else if(!r->object->movement().restrictions().accessible(o.goal_node))reason=reject_goal_inaccessible;
        }
        if(reason){if(r)feedback(*r,o,reason);result.rejected.push_back({o.actor,reason,yields});continue;}
        const bool lean_changed=!r->has_order||r->order.lean!=o.lean;
        const bool bound_started=o.kind==OrderKind::bound&&(!r->has_order||r->order.kind!=OrderKind::bound||
            r->order.goal_node!=o.goal_node||!same_value(r->order.goal_position,o.goal_position));
        r->point_fire=point_valid?*point:core::PointFireAuthority{};
        r->order=o;r->last_order=o.order_seq;r->has_order=true;r->owned=true;r->provisional=false;r->cleanup_done=false;r->stale_anchor=false;r->rejected={};
        r->object->set_painter_forced_action(ids::action);result.accepted_orders.push_back(o.actor);
        if(lean_changed&&o.lean!=0)
            Msg("[painter_combat] phase=decision_event event=lean kind=%u side=%u source=%llu room=%u detail=%u order=%llu lean=%.3f stance=%u evidence=accepted_order",
                unsigned(events::lean),d.side_id,d.snapshot_seq,r->snapshot.room_id,unsigned(o.actor.id),o.order_seq,o.lean,unsigned(o.stance));
        if(bound_started)
            Msg("[painter_combat] phase=decision_event event=bound side=%u source=%llu actor=%u order=%llu node=%u goal=(%.3f,%.3f,%.3f) evidence=accepted_order",
                d.side_id,d.snapshot_seq,unsigned(o.actor.id),o.order_seq,o.goal_node,o.goal_position.x,o.goal_position.y,o.goal_position.z);
    }
    for(auto key:d.releases){auto r=find(key);unsigned count=0;for(auto k:d.releases)if(k==key)++count;for(auto const& o:d.orders)if(o.actor==key)++count;
        if(!r||!registered(*r)||count!=1){result.rejected.push_back({key,reject_generation,yield_none});continue;}
        auto source=std::find_if(r->snapshots.begin(),r->snapshots.end(),[&](auto const& s){return s.sequence==d.snapshot_seq;});
        if(source==r->snapshots.end()||!source_stamp_valid(*source,d.snapshot_seq,r->ownership,r->restrictions,now,now,combat_config().snapshot_max_age_ms)){
            result.rejected.push_back({key,reject_ownership,yield_none});continue;
        }
        relinquish(*r,yield_released);result.accepted_releases.push_back(key);
        Msg("[painter_combat] phase=squad_released side=%u actor=%u reason=%s",r->release_side?r->release_side:d.side_id,unsigned(key.id),r->side_release_reason);
    }
    if(d.side_id&&!result.accepted_orders.empty()&&side_sequences.find(d.side_id)==side_sequences.end())
        Msg("[painter_combat] phase=squad_taken side=%u source=%llu accepted=%u",d.side_id,d.snapshot_seq,unsigned(result.accepted_orders.size()));

    if(!result.accepted_orders.empty()||!result.accepted_releases.empty()){
        side_sequences[d.side_id]={d.snapshot_seq,now};
        const TimeMs twice=core::saturated_after(combat_config().order_lifetime_ms,combat_config().order_lifetime_ms);
        const TimeMs keep=core::saturated_after(twice,twice);
        for(auto it=side_sequences.begin();it!=side_sequences.end();)
            if(now>it->second.at&&now-it->second.at>keep)it=side_sequences.erase(it);else ++it;
    }
    return result;
}
void synchronize_lifecycle(const core::SideLifecycleResult& f){
    if(!f.accepted||f.cancelled||f.level_epoch!=level_epoch()||f.snapshot_seq<=lifecycle_sequence&&f.level_epoch==lifecycle_epoch)return;
    lifecycle_sequence=f.snapshot_seq;lifecycle_epoch=f.level_epoch;
    const auto now=now_ms();
    lifecycle_cooldowns.clear();
    for(const auto& c:f.cooldowns)if(c.until_ms>now)lifecycle_cooldowns.push_back(c);
    for(const auto& release:f.releases){auto r=find(release.member.actor);if(!r||r->ownership!=release.member.ownership_generation)continue;
        r->side_release_reason=release_name(release.reason);r->release_side=release.identity.side;
    }
    for(const auto& lease:f.retained){auto r=find(lease.member.actor);if(!r||r->ownership!=lease.member.ownership_generation)continue;
        for(const auto& side:f.sides)if(side.identity.side==lease.identity.side){
            r->stand_down=side.phase==core::SideLifecyclePhase::stand_down;
        }
    }
}
void capture_lifecycle_signals(const core::SideLifecycleResult* f,const WorldSnapshot& world,core::LifecycleSignals& signals){
    if(!f||!f->accepted||f->level_epoch!=world.level_epoch)return;
    auto reference=Level().CurrentViewEntity();
    for(const auto& side:f->sides){
        core::LifecycleReceiptStamp stamp;stamp.identity=side.identity;stamp.source_snapshot=world.sequence;stamp.at_ms=world.now_ms;
        if(reference&&_valid(reference->Position())){core::LifecycleRangeReceipt range;range.stamp=stamp;range.reference_position=value(reference->Position());signals.ranges.push_back(range);}
        for(const auto& current:world.sides)if(current.side_id==side.identity.side&&current.mission.revision==side.identity.mission_revision){
            bool actual=false;for(const auto& key:current.members){auto r=find(key);if(!r||!r->has_order||r->order.expires_at_ms<=world.now_ms||!r->order.assignment.portal||!r->snapshot.last_applied.order_seq)continue;
                if(r->order.order_seq!=r->snapshot.last_applied.order_seq)continue;
                core::LifecycleDoorReceipt door;door.stamp=stamp;door.portal=r->order.assignment.portal;door.actor=key;
                door.ownership_generation=r->ownership;door.applied_order_seq=r->order.order_seq;signals.required_doors.push_back(door);actual=true;break;}
            if(!actual&&current.mission.entry_portal){core::LifecycleDoorReceipt door;door.stamp=stamp;door.portal=current.mission.entry_portal;signals.required_doors.push_back(door);}
        }
    }
}
void capture_grenade_fuses(const WorldSnapshot& world,std::vector<core::OutdoorGrenadeFuse>& out){
    out.clear();for(const auto& a:world.actors){auto r=find(a.key);if(!r||!registered(*r)||a.danger.kind!=DangerKind::grenade||a.danger.perceive!=0)continue;
        const auto* d=r->object->memory().danger().selected();if(!d||d->type()!=CDangerObject::eDangerTypeGrenade||d->perceive_type()!=CDangerObject::eDangerPerceiveTypeVisual
            ||d->time()!=a.danger.at_ms||!same_value(value(d->position()),a.danger.position))continue;
        const auto* grenade=smart_cast<const CGrenade*>(d->dependent_object());
        if(!grenade||grenade->getDestroy())continue;const auto until=grenade->destroy_time();
        if(until==u32(-1)||until<=world.now_ms)continue;
        core::OutdoorGrenadeFuse fuse;fuse.actor=a.key;fuse.epoch=world.level_epoch;fuse.sequence=world.sequence;
        fuse.owner=a.ownership_generation;fuse.restrictions=a.restrictions_revision;fuse.observed_at=a.observed_at_ms;
        fuse.perceived_at=a.danger.at_ms;fuse.position=a.danger.position;fuse.until=until;out.push_back(fuse);r->grenade_until=until;
    }
}
void report_outdoor_decisions(const std::vector<core::OutdoorDispatchReceipt>& rows){
    struct Reported {std::uint64_t sequence=0;TimeMs prerequisites_at=0;};
    static std::uint64_t reported_epoch=0;
    static std::map<std::uint32_t,Reported> reported;
    const auto epoch=level_epoch();
    if(reported_epoch!=epoch){reported.clear();reported_epoch=epoch;}
    for(const auto& row:rows){const auto& p=row.posture;if(p.status!=core::OutdoorStatus::ready)continue;
        if(reported_styles.find(p.stamp.side)==reported_styles.end()||reported_styles[p.stamp.side]!=p.stamp.faction){
            reported_styles[p.stamp.side]=p.stamp.faction;
            Msg("[painter_combat] phase=faction_style side=%u faction=%u profile=%u doctrine=%u matched=%u evidence=resolved_core_style",
                p.stamp.side,p.stamp.faction,unsigned(p.resolved_style.profile),unsigned(p.resolved_style.doctrine),unsigned(p.style_matched));
        }
        auto state=std::make_pair(unsigned(p.posture),std::string(p.reason?p.reason:"none"));
        auto old=side_postures.find(p.stamp.side);if(old==side_postures.end()||old->second!=state){
            const bool was_exchange=old!=side_postures.end()&&old->second.second=="exchange_hold";
            side_postures[p.stamp.side]=state;
            Msg("[painter_combat] phase=squad_decision side=%u source=%llu posture=%u reason=%s",p.stamp.side,p.stamp.snapshot_seq,state.first,state.second.c_str());
            if(p.exchange_hold||row.assessment.exchange_hold||was_exchange)
                Msg("[painter_combat] phase=decision_event event=exchange_posture side=%u source=%llu posture=%u reason=%s building=%u ours=%u theirs=%u hits=%u advance_cost=%.3f hold_cost=%.3f until_ms=%llu",
                    p.stamp.side,p.stamp.snapshot_seq,state.first,state.second.c_str(),p.defended_building,
                    unsigned(row.assessment.ours),unsigned(row.assessment.theirs),unsigned(row.assessment.hits_taken),
                    row.assessment.advance_cost,row.assessment.hold_cost,row.assessment.exchange_until_ms);}
        // Original source sequence makes repeated completed-frame reads idempotent.
        auto& receipt=reported[p.stamp.side];
        if(receipt.sequence>=p.stamp.snapshot_seq)continue;receipt.sequence=p.stamp.snapshot_seq;
        // Read existing committed receipts only. One sample per side/report
        // interval; no geometry queries, target reads, or per-frame log flood.
        if(!receipt.prerequisites_at||p.stamp.at_ms<receipt.prerequisites_at||
            p.stamp.at_ms-receipt.prerequisites_at>=core::runtime_report_interval_ms){
            receipt.prerequisites_at=p.stamp.at_ms;
            unsigned cover_known=0,line_known=0,route_known=0,search_known=0;
            for(const auto& actor:row.execution.facts.actors){
                cover_known+=actor.cover_verified!=Evidence::unknown;
                line_known+=actor.fire_line!=Evidence::unknown;
                route_known+=actor.movement_route!=Evidence::unknown;
                search_known+=actor.searched!=Evidence::unknown;
            }
            const auto& b=row.bounds;const auto& a=row.assessment;const auto& w=row.withdrawal;
            Msg("[painter_combat] phase=outdoor_readiness side=%u source=%llu at_ms=%llu posture=%u base_set=%u base_pinned=%u allow_bound=%u execution=%u applied=%u unconfirmed=%u cover_known=%u line_known=%u route_known=%u search_known=%u bounds=%u bounds_reason=%s ready_base=%u base_goals=%u endpoints=%u nav_pending=%u route_unknown=%u examined=%u rejected=%u evidence=committed_worker_receipts",
                p.stamp.side,p.stamp.snapshot_seq,p.stamp.at_ms,unsigned(p.posture),unsigned(p.base_set),unsigned(p.base_pinned),unsigned(p.allow_bound),
                unsigned(row.execution.status),unsigned(row.execution.applied),unsigned(row.execution.unconfirmed),cover_known,line_known,route_known,search_known,
                unsigned(b.status),b.reason?b.reason:"none",unsigned(b.ready_base.size()),unsigned(b.base_positions.size()),unsigned(b.endpoints.size()),
                unsigned(b.pending_navigation),unsigned(b.unknown_route_clearance),unsigned(b.examined_candidates),unsigned(b.rejected_candidates));
            Msg("[painter_combat] phase=support_recovery side=%u source=%llu authorized=%u reason=%s failure_at_ms=%llu evaluated=%u invalid=%u protection=%u restricted=%u pending=%u unsafe=%u occupied=%u proposals=%u evidence=committed_worker_receipts",
                p.stamp.side,p.stamp.snapshot_seq,unsigned(p.support_recovery),b.support_reason?b.support_reason:"none",
                row.execution.facts.support_failure.completed?row.execution.facts.support_failure.stamp.at_ms:0ull,unsigned(b.base_evaluated),unsigned(b.base_invalid),
                unsigned(b.base_protection),unsigned(b.base_restricted),unsigned(b.recovery_pending),unsigned(b.recovery_unsafe),
                unsigned(b.recovery_occupied),unsigned(b.support_reposition.size()));
            Msg("[painter_combat] phase=outdoor_recovery side=%u source=%llu assessment=%u verdict=%u reason=%s able=%u dead=%u unknown=%u hits=%u broken=%u losing=%u safe_fallback=%u casualty_clear=%u capped=%u withdrawal=%u withdrawal_reason=%s leg_receipts=%u arrived=%u retreat_proposals=%u movement=%u movement_reason=%s move_proposals=%u move_nav_pending=%u evidence=committed_worker_receipts",
                p.stamp.side,p.stamp.snapshot_seq,unsigned(a.status),unsigned(a.verdict),a.reason?a.reason:"none",unsigned(a.able),unsigned(a.dead),unsigned(a.unknown),unsigned(a.hits_taken),
                unsigned(a.broken),unsigned(a.losing),unsigned(a.safe_fallback),unsigned(a.casualty_obligation_clear),unsigned(a.capped),unsigned(w.status),w.reason?w.reason:"none",
                unsigned(row.withdrawal_execution.legs.size()),unsigned(row.withdrawal_execution.arrived.size()),unsigned(w.orders.size()),
                unsigned(row.movement.status),row.movement.reason?row.movement.reason:"none",unsigned(row.movement.proposals.size()),unsigned(row.movement.pending_navigation));
        }
        for(const auto& solo:row.solo){
            if(!solo.lean_gate||!strcmp(solo.lean_gate,"not_attempted"))continue;
            auto actor=find(solo.proposal.order.actor);if(!actor)continue;
            const unsigned stage=unsigned(solo.status)*256u+unsigned(solo.reason);
            if(actor->lean_gate_reported==solo.lean_gate&&actor->lean_gate_focus==solo.focus.actor&&actor->lean_gate_stage==stage)continue;
            actor->lean_gate_reported=solo.lean_gate;actor->lean_gate_focus=solo.focus.actor;actor->lean_gate_stage=stage;
            Msg("[painter_combat] phase=lean_gate side=%u source=%llu actor=%u target=%u reason=%s verified=%u status=%u solo_reason=%u candidates=%u queries=%u evidence=worker_decision",
                p.stamp.side,p.stamp.snapshot_seq,unsigned(solo.proposal.order.actor.id),unsigned(solo.focus.actor.id),
                solo.lean_gate,unsigned(solo.lean_static_verified),unsigned(solo.status),unsigned(solo.reason),unsigned(solo.candidates),unsigned(solo.queries));
        }
        for(const auto& suppression:row.fire.executed_suppression)
            Msg("[painter_combat] phase=outdoor_suppression side=%u source=%llu observer=%u",p.stamp.side,p.stamp.snapshot_seq,unsigned(suppression.observer.id));
    }
}
void report_room_decisions(const std::vector<core::RoomDispatchReceipt>& rows){
    // These are committed controller receipts, not confirmation that every
    // proposed order executed. Positions retain their original capture time.
    struct Reported {std::uint32_t mission=0;std::uint64_t sequence=0;};
    static std::uint64_t reported_epoch=0;
    static std::map<std::uint32_t,Reported> reported;
    const auto epoch=level_epoch();
    if(reported_epoch!=epoch){reported.clear();reported_epoch=epoch;}
    static const char* phases[]={"invalid","form","pie","entry","search","assess","complete"};
    static const char* reasons[]={"none","invalid_input","no_geometry","no_projection","forming","arrived",
        "pie_progress","pie_cap","engage_cost","go_cost","entry_flow","coverage","known_foe","assess_cap",
        "fresh_contact","frag_requested","frag_fuse","frag_failed","cancelled","sight_clear","assess_observed","casualty_cost","leader_call"};
    for(const auto& row:rows){const auto& d=row.doctrine;
        if(!epoch||!d.accepted||d.cancelled||d.level_epoch!=epoch||!d.snapshot_seq)continue;
        auto& old=reported[row.side_id];
        if(old.mission==row.mission_revision&&d.snapshot_seq<=old.sequence)continue;
        old={row.mission_revision,d.snapshot_seq};
        for(std::size_t index=0;index<d.transitions.size();++index){const auto& event=d.transitions[index];
            const unsigned from=unsigned(event.from),to=unsigned(event.to),why=unsigned(event.reason);
            const auto* from_name=from<sizeof(phases)/sizeof(phases[0])?phases[from]:"unknown";
            const auto* to_name=to<sizeof(phases)/sizeof(phases[0])?phases[to]:"unknown";
            const auto* reason=why<sizeof(reasons)/sizeof(reasons[0])?reasons[why]:"unknown";
            Msg("[painter_combat] phase=room_transition epoch=%llu side=%u mission_revision=%u source=%llu room=%u transition=%u from=%s to=%s reason=%s at_ms=%llu members=%u evidence=source_capture native_arrival=unproven",
                d.level_epoch,row.side_id,row.mission_revision,d.snapshot_seq,d.room,unsigned(index),from_name,to_name,reason,event.at_ms,unsigned(event.members.size()));
            for(const auto& member:event.members)
                Msg("[painter_combat] phase=room_transition_member epoch=%llu side=%u mission_revision=%u source=%llu room=%u transition=%u from=%s to=%s actor=%u generation=%llu source_pos=(%.3f,%.3f,%.3f) observed_at_ms=%llu evidence=source_capture",
                    d.level_epoch,row.side_id,row.mission_revision,d.snapshot_seq,d.room,unsigned(index),from_name,to_name,unsigned(member.actor.id),member.actor.generation,
                    member.position.x,member.position.y,member.position.z,member.observed_at_ms);
        }
    }
}
void report_runtime_cost(TimeMs now,float total,float capture,float dispatch,float publication,std::uint64_t failures){
    pass_costs.add(total);if(now>=pass_report_at&&now-pass_report_at<core::runtime_report_interval_ms)return;pass_report_at=now;
    Msg("[painter_combat] phase=pass_cost passes=%llu median_ms=%.3f worst_ms=%.3f obs_ms=%.3f dir_ms=%.3f ord_ms=%.3f decision_cost_us=%llu decisions=%llu failures=%llu",
        pass_costs.passes,pass_costs.median(),pass_costs.worst,capture,dispatch,publication,decision_cost_us,decision_count,failures);
}
void report_response_causes(TimeMs now,std::uint64_t admitted,std::uint64_t contact,std::uint64_t hit,
    std::uint64_t applied,std::uint64_t missed){
    static std::uint64_t epoch=0;static TimeMs last=0;
    const auto current=level_epoch();if(epoch!=current){epoch=current;last=0;}
    if(last&&now>=last&&now-last<core::runtime_report_interval_ms)return;last=now;
    Msg("[painter_combat] phase=response_causes admission=%llu contact=%llu hit=%llu first_applied=%llu first_deadline_missed=%llu evidence=runtime_counters",
        admitted,contact,hit,applied,missed);
}
void runtime_failed(){
    const auto until=core::saturated_after(now_ms(),combat_config().outdoor.r.lifecycle_values.error_cooldown_ms);
    for(auto& item:records){auto& r=*item.second;if(!(r.owned||r.has_order||r.provisional)||!registered(r))continue;
        // Authenticate before cleanup changes ownership. Error cooloffs are
        // native tombstones, independent of the next worker's new identity.
        core::LifecycleCooldown c;c.identity.level_epoch=r.epoch;c.identity.map_hash=captured_map;
        c.identity.geometry_revision=r.geometry;c.identity.faction=r.snapshot.faction;c.identity.group=r.snapshot.native_squad;
        c.native_group=r.snapshot.native_squad;c.until_ms=until;c.reason=core::SideReleaseReason::error;c.members.push_back(r.key);
        error_cooldowns.push_back(c);r.side_release_reason="error";
    }
    release_all(yield_invalid_input);
}
#include "EngineOutdoorCombatCapture_impl.h"
}}}
