// Pure interface bridge: compile with PCH disabled, no forced stdafx header,
// /EHsc. Engine object handling remains in EngineCombatAdapter/EngineCapture.
#include "EngineCombatRuntime.h"
#include "EngineCombatAdapter.h"
#include "EngineCapture.h"
#include "EngineServices.h"
#include "EngineOutdoorSettings.h"
#include "EngineOutdoorCombatCapture.h"
#include "OutdoorDoorCapture.h"
#include "EngineMissionIntake.h"
#include "EngineRouteProof.h"
#include "EngineFriendlyCasualty.h"
#include "EngineFriendlyRoster.h"
#include "EnginePerceivedSpecies.h"
#include "EngineRecorder.h"
#include <algorithm>
#include <chrono>
#include <memory>
#include <limits>

namespace painter { namespace combat { namespace engine {
namespace {
using Clock=std::chrono::steady_clock;
std::unique_ptr<core::CombatDispatcher> dispatcher;
RuntimeStatus status;
OutdoorDoorCapture door_capture;
MissionIntake mission_intake;
std::shared_ptr<EngineRouteProof> route_proof;
bool stored_mission(void*,std::uint64_t group,Mission& out){return mission_of(group,out);}
bool active=false;
std::uint64_t snapshot_sequence=0;
std::vector<core::LifecycleReceiptStamp> pending_errors;
struct Deadline {Clock::time_point end;};
bool expired(void* context){return Clock::now()>=static_cast<Deadline*>(context)->end;}
struct Measure {
    float& last;float& maximum;Clock::time_point started=Clock::now();bool accumulate=false;
    Measure(float& l,float& m,bool sum=false):last(l),maximum(m),accumulate(sum){}
    ~Measure() noexcept {
        const auto elapsed=std::chrono::duration<float,std::milli>(Clock::now()-started).count();
        last=accumulate?last+elapsed:elapsed;maximum=(std::max)(maximum,last);
    }
};
core::NativePublication apply(const Decision& decision) {
    Measure cost(status.publish_ms,status.maximum_publish_ms,true);
    if(dispatcher)if(const auto life=dispatcher->lifecycle_frame())synchronize_lifecycle(*life);
    const auto result=publish(decision);core::NativePublication out;
    note_publish_rejects(decision,result);recorder_publish(decision,result);   // native17: reject reasons + the live recorder
    if(result.envelope!=reject_none)return out;
    out.accepted=result.accepted_orders;
    out.accepted.insert(out.accepted.end(),result.accepted_releases.begin(),result.accepted_releases.end());
    return out;
}
core::NativePublication apply_points(const Decision& decision,const std::vector<core::PointFireAuthority>& points) {
    Measure cost(status.publish_ms,status.maximum_publish_ms,true);
    if(dispatcher)if(const auto life=dispatcher->lifecycle_frame())synchronize_lifecycle(*life);
    const auto result=publish_points(decision,points);core::NativePublication out;
    note_publish_rejects(decision,result);recorder_publish(decision,result);   // native17
    if(result.envelope!=reject_none)return out;
    out.accepted=result.accepted_orders;
    out.accepted.insert(out.accepted.end(),result.accepted_releases.begin(),result.accepted_releases.end());return out;
}
struct ReportPass {
    TimeMs at;Clock::time_point started=Clock::now();
    ~ReportPass() noexcept {recorder_tick_end(at);   // native17: closes the tick's record on every exit path
        report_runtime_cost(at,std::chrono::duration<float,std::milli>(Clock::now()-started).count(),
        status.capture_ms,status.dispatch_ms,status.publish_ms,status.failures);}
};
void tick(TimeMs now) noexcept {
    ReportPass report{now};
    Measure cost(status.last_tick_ms,status.maximum_tick_ms);++status.frames;
    status.capture_ms=status.dispatch_ms=status.publish_ms=0;
    try {
        if(!combat_enabled()) {
            if(active){set_enabled(false);release_all();dispatcher.reset();route_proof.reset();active=false;}
            return;
        }
        const auto epoch=level_epoch();
        if(!dispatcher||status.epoch!=epoch) {
            const auto outdoor_config=make_outdoor_policies(combat_config().outdoor);
            dispatcher=std::make_unique<core::CombatDispatcher>(combat_config(),core::DispatchLimits{},outdoor_config.lifecycle,outdoor_config.policy,outdoor_config.exposure);
            door_capture.reset();mission_intake.reset();route_proof.reset();
            if(combat_config().outdoor.r.route.restricted){
                const auto& r=combat_config().outdoor.r.route;RouteProofPolicy policy;
                policy.queue_bytes=r.queue_bytes;policy.job_overhead_bytes=r.job_overhead_bytes;
                policy.worker_work=r.worker_work;policy.authority_work=r.authority_work;
                policy.expansion_limit=r.expansions;policy.output_limit=r.output_nodes;
                policy.endpoint_height_m=r.endpoint_height_m;policy.source_age_ms=combat_config().snapshot_max_age_ms;
                route_proof=std::make_shared<EngineRouteProof>(policy,query_static_route);
            }
            status.epoch=epoch;snapshot_sequence=0;set_enabled(true);active=true;
        }
        if(snapshot_sequence==(std::numeric_limits<std::uint64_t>::max)())return;
        const auto callbacks=services();
        WorldSnapshot snapshot;snapshot.level_epoch=epoch;snapshot.sequence=++snapshot_sequence;
        snapshot.now_ms=now;snapshot.geometry_revision=epoch;
        if(callbacks.buildings)snapshot.map_hash=callbacks.buildings->map_hash;
        core::FriendlyCasualtyEvidence friendly;
        core::LifecycleSignals lifecycle;
        const auto prior_lifecycle=dispatcher->lifecycle_frame();
        if(prior_lifecycle)synchronize_lifecycle(*prior_lifecycle);
        core::OutdoorCaptureEvidence outdoor;
        outdoor.source.level_epoch=snapshot.level_epoch;outdoor.source.snapshot_seq=snapshot.sequence;
        outdoor.source.map_hash=snapshot.map_hash;outdoor.source.geometry_revision=snapshot.geometry_revision;outdoor.source.at_ms=snapshot.now_ms;
        {Measure capture(status.capture_ms,status.maximum_capture_ms);capture_all(snapshot,&outdoor.eyes,&lifecycle.perceived_species);
            const auto& settings=combat_config().outdoor;
            const auto prior_knowledge=dispatcher->knowledge_frame();
            capture_outdoor_combat(snapshot,outdoor,settings.p,settings.r.authority_query_work_per_frame,prior_knowledge.get());
            capture_grenade_fuses(snapshot,outdoor.grenade_fuses);
            door_capture.update(snapshot,settings.r.doors,capture_doors_bounded);
            const auto retain=friendly_roster_retention_ms(combat_config());
            if(mission_intake.update(snapshot,stored_mission,retain).status!=MissionIntakeStatus::ready){++status.ingest_rejected;return;}
            const auto previous=dispatcher->side_frame();
            const auto known=retained_friendly_rosters(snapshot,previous?&previous->registry:nullptr,retain);
            capture_friendly_casualties(snapshot,known,friendly,combat_config().bootstrap_work_per_tick);
            core::complete_friendly_casualty_rosters(snapshot,known,friendly,combat_config());
            capture_lifecycle_signals(prior_lifecycle.get(),snapshot,lifecycle);
            for(auto error:pending_errors){error.source_snapshot=snapshot.sequence;error.at_ms=snapshot.now_ms;lifecycle.errors.push_back(error);}
            pending_errors.clear();
            // Authority work is bounded and INCLUDED in capture/tick timing.
            // Workers get only a value queue; they never wait for these checks.
            if(route_proof){route_proof->advance(snapshot,route_authority());outdoor.route_service=route_proof->service();}
        }
        Measure dispatch(status.dispatch_ms,status.maximum_dispatch_ms);
        recorder_snapshot(snapshot);   // native17: the live recorder takes the snapshot before the dispatcher owns it
        if(!dispatcher->ingest(std::move(snapshot),callbacks,{},lifecycle,outdoor,{},friendly)){++status.ingest_rejected;return;}
        // Cooperative bootstrap deadline, not a claim that capture/publication
        // or one engine query can be preempted in this time. Measure all of them.
        Deadline budget{Clock::now()+std::chrono::microseconds(combat_config().bootstrap_deadline_us)};
        dispatcher->tick(now,apply,expired,&budget,apply_points);status.dispatch=dispatcher->stats();
        report_response_causes(now,status.dispatch.episode_admission,status.dispatch.episode_contact,status.dispatch.episode_hit,
            status.dispatch.first_applied,status.dispatch.first_deadline_missed);
        if(const auto life=dispatcher->lifecycle_frame())synchronize_lifecycle(*life);
        report_outdoor_decisions(dispatcher->outdoor_frames());
        report_room_decisions(dispatcher->room_frames());
    }catch(...) {
        ++status.failures;
        // Error paths relinquish immediately; never leave an expired idle pose
        // waiting for stale_release_ms. Queue the exact identities for lifecycle.
        if(dispatcher)if(const auto life=dispatcher->lifecycle_frame())for(const auto& side:life->sides){
            core::LifecycleReceiptStamp error;error.identity=side.identity;pending_errors.push_back(error);}
        runtime_failed();
    }
}
}
void initialize_combat_runtime(){set_tick(tick);}
// The level is going: its men are handed back as offline (not as a master-off), then the service stops
void level_ending_combat_runtime(){pending_errors.clear();release_all(yield_offline);set_enabled(false);dispatcher.reset();route_proof.reset();mission_intake.reset();active=false;}
void shutdown_combat_runtime(){set_tick(nullptr);level_ending_combat_runtime();}
RuntimeStatus combat_runtime_status(){return status;}
}}}
