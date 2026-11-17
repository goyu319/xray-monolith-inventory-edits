// body_hit_outcomes_v1
#pragma once
// CPU-only immutable dispatch/results. Never carries an engine object pointer.
#include <array>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "RuntimePackedMesh.h"
#include "SeparationJobProgress.h"
#include "GoreControls.h"
#include "RaggedBlade.h"
#include "BodyHitOutcomes.h"
namespace gfy_connectivity {struct Topology;}
namespace gfy_live {
using Hash=std::array<std::uint8_t,32>;
struct Point {double x{},y{},z{};};
struct Matrix {std::array<double,16> row_major{};static Matrix identity();};
struct Identity {
    std::uint64_t spawn_cookie{},visual_lineage{},topology{};
    Hash source{},exact_rig{};
    bool operator==(const Identity&)const noexcept;
};
struct Stamp {Identity owner;std::uint64_t expected_generation{},revision{};};

// PRIVATE OPTIONAL DIAGNOSTIC FOUNDATION. Original-model rest-space cutters,
// not a replacement for Grid damage or a promise of equivalent mask topology.
enum ResolvedCutLimitation : std::uint32_t {
    resolved_cut_edge_repairs=1u, resolved_cut_sever=2u,
    resolved_cut_legacy_sphere=4u, resolved_cut_owned_grid=8u,
    resolved_cut_capacity=16u, resolved_cut_unwritten=32u, resolved_cut_slab=64u
};
struct ResolvedEdgeProfile {
    // Exact internal table entries, including periodic endpoint; no sampling.
    std::array<std::array<double,129>,2> radius{};
};
// R62: the torn surface of an anatomy-shaped sever (TissueTear.h): rest-space points, render normals (into the cavity),
// triangles wound with their normal INTO the kept tissue (the cutter convention) and the tissue slot each one bounds.
struct ResolvedTearSurface {
    std::vector<std::array<float,3>> points,normals;
    std::vector<std::array<std::uint32_t,3>> triangles;
    std::vector<std::uint8_t> labels;
    std::shared_ptr<const gfy_packed::TearBlock> decide; // mask-only wounds, slice 3: a decide-once tear (no triangles)
};
struct ResolvedTunnel {
    enum class Role : std::uint8_t {primary,vent,tear}; // tear: DamageTear.h channel through a failed station
    Role role{Role::primary};std::uint64_t sequence{},edge_seed{};
    std::uint32_t source_face{};std::array<double,3> source_bary{};
    Point entry,direction,edge_u,edge_v;
    double radius{},penetration{},cap{},inlet{},exit_distance{-1},exit_mult{1};
    double cav_amp{},cav_t0{},cav_t1{},burst_amp{},burst_len{},crack_width{};
    std::uint32_t cracks{};std::array<double,8> crack_angle{};
    std::shared_ptr<const ResolvedEdgeProfile> edge;
    std::size_t edge_contact_cells{};
    bool slab{};double slab_half_width{};std::uint64_t slab_seed{};
    // seam (value 3, SeamTear.h): the tissue this round wrecks around its track (m), set at the hit from its round (reachAt);
    // 0 = not set (another value, or a record from before): the seam rule reads it from the hole
    float seam_reach{};
    // R62: a torn tissue surface (TissueTear.h). The cutter fields describe its cylinder only; it removes nothing by
    // itself (the torn cells are in the grid) and its walls are the stored triangles (channel field mode 4).
    std::shared_ptr<const ResolvedTearSurface> surface;
};
struct ResolvedCutHistory {
    static std::shared_ptr<ResolvedCutHistory> create() {
        return std::shared_ptr<ResolvedCutHistory>(new ResolvedCutHistory,&dispose);
    }
private:
    ResolvedCutHistory()=default;
    ResolvedCutHistory(const ResolvedCutHistory&)=delete;
    ResolvedCutHistory& operator=(const ResolvedCutHistory&)=delete;
    ~ResolvedCutHistory()=default;
    mutable const ResolvedCutHistory* disposal_next_=nullptr;
    static void dispose(const ResolvedCutHistory* value) noexcept {
        static thread_local const ResolvedCutHistory* pending=nullptr;
        static thread_local bool draining=false;
        value->disposal_next_=pending;pending=value;
        if(draining)return;
        draining=true;
        while(pending){auto* node=pending;pending=node->disposal_next_;delete node;}
        draining=false;
    }
public:
    static constexpr std::uint32_t schema_version=1;
    static constexpr std::size_t max_cuts=4096;
    std::shared_ptr<const ResolvedCutHistory> parent;
    std::vector<ResolvedTunnel> appended;
    Identity owner; // Exact source/rig/owner identity for this whole-body lineage.
    std::uint64_t damage_revision{}; // Candidate damage revision, NOT renderer generation.
    std::size_t total_cuts{};
    std::uint32_t limitations{}; // Cumulative; never silently declare complete coverage.
    Point grid_origin;std::array<std::uint32_t,3> grid_size{};double grid_pitch{};
    // Detached ComponentSession history/ownership clipping is unsupported.
    // Even limitations==0 describes analytic cuts, not their sampled voxel union.
};

struct JointRegion {
    std::uint16_t bone{};
    Point center,axis;
    double radius{},half_width{};
    bool sever=true; // false: joint-damage receipts only, never a sever station (jaw)
};
struct JointDamage {
    std::uint16_t bone{};
    double original_m3{},removed_m3{},original_bone_m3{},removed_bone_m3{};
};
struct CranialDamage {
    bool valid=false;
    double original_m3{},removed_m3{},original_bone_m3{},removed_bone_m3{};
    double original_brain_m3{},removed_brain_m3{};
};
struct AssetInput {
    std::vector<std::uint8_t> source_geometry,closed_envelope,base_occupancy,source_shading,occlusion_geometry;
    std::vector<std::uint8_t> source_children,material_slots;
    std::vector<std::uint8_t> accessory_surfaces; // Optional exact original geometry blob for ocular surface clipping.
    std::vector<std::uint8_t> render_field; // Optional offline-baked render-only cut field (VXTFLD01, gzip allowed).
    Hash source_hash{},rig_hash{};
    std::uint16_t source_child{},original_child_count{};
    bool record_resolved_cuts{}; // Optional CPU history; default off, no renderer consumer.
    bool analytic_channels{}; // PRIVATE opt-in. Coverage remains unqualified; diagnostic build only.
    bool allow_renderer_rebuild{}; // Private opt-in; no renderer integration by default.
    double damage_pitch{}; // Zero uses asset pitch; fullbody wound cells can use 2/1/0.5 mm.
    std::vector<JointRegion> joints; // Lua dimensions + exact native bind-space pivots.
};
// Resolved loose-file names only: the owner resolves engine VFS precedence.
// Workers read immutable bytes and never call the engine filesystem or Lua.
struct AssetFile {std::string path;Hash expected{};std::size_t max_bytes{32u*1024u*1024u};};
using AssetFiles=std::array<AssetFile,10>; // Original OGF, seven profile blobs, optional accessory surfaces, optional render field.
using AssetLoader=std::function<void(AssetInput&)>;
using AssetHashFunction=Hash(*)(const void*,std::size_t);
struct Impact {
    body_hit_outcomes::Tag hit_trace;
    std::uint64_t sequence{};
    GoreControls gore; // Complete accepted-event snapshot; never reread menu settings on workers.
    std::uint16_t cranial_bone{65535}; // Exact native rig lookup, not inferred from the struck bone.
    std::uint64_t explosion_world{},explosion_cookie{},explosion_target_birth{},explosion_member_token{};
    std::string ammo_section;
    std::uint16_t hit_bone{};
    double hit_power{};
    Point accepted_pose_point,accepted_pose_direction;
    // Bone mRenderTransform numeric snapshot, owner transform already removed.
    std::vector<Matrix> bone_model_render;
    // depth is cutter CENTER offset from first surviving cell entry, not penetration reach.
    double radius{.017},depth{.008},source_search_distance{.12};
    bool trace_gear{}; // gearbody: the owner's verbose log is on - the result tells where a shot through merged gear was mapped
    // Positive selects a connected entry-to-tip cut. Zero retains the legacy
    // diagnostic sphere; it must never be used as an ammo penetration value.
    double penetration_m{0};
    double exit_diameter_mult{1}; // Script policy, applied only at a reached outer surface.
    std::uint64_t ammo_seed{}; // Sampled once at accepted SHit, immutable in retry.
    std::uint16_t ammo_profile_index{};
    double sever_scale{}, visual_damage_scale{}; // Artistic metadata; no NPC health mutation.
    // Forensic shape (engine defaults from the ammo class and hit bone, Lua may override): cavitation = relative
    // swell of the channel behind the entry (temporary-cavity tearing, 0 = plain bore, 1 = doubles the radius at its
    // widest); burst = exit rupture (crater growth and radial cracks at a far-side exit, 0..1; head hits by rifle/slug).
    double cavitation{0}, burst{0};
    // Lateral vent (0 = none, 0..1 strength): a violent temporary cavity blows a second, seeded hole out of the side or
    // top of the body part (never downward in rest space) with its own crater and cracks; the engine decides per shot.
    double vent{0};
    // Entry burst (0 = none, 0..1 strength): the entry mouth ruptures into a cracked crater, 70-85 % of the exit mouth.
    double entry_burst{0};
    std::uint16_t initiator{65535}; // who fired (kill credit when a sever kills a living NPC)
    bool blast{false}; // one crater of an explosion: severs are checked once after the batch
    std::uint32_t blade{}; // grenade blade slot this bore belongs to (0 = none): its tear follows the slot's plane
    // R62 (BlastSevers.h): a blast's forced tear, not a cut. hit_bone is the tear site's joint bone, accepted_pose_point the
    // posed point the blast hit hardest, accepted_pose_direction the blast's push there; the worker tears that limb at the
    // station nearest the point after the batch's craters, leaning the torn ends with the push. Carves nothing itself.
    bool tear_request{false};
    bool living_hit{false},killing_hit{false},tear_survived{false};
    // klass (2026-11-11): what the round IS, frozen from the Lua wound table with the other script outputs (PainterWoundEvent.h).
    // The hole is real size now (a 5.45 mm round has a 2.8 mm radius), so its size no longer tells. -1 = not given (older Lua, and
    // every event the engine builds itself: blast craters, force tears, probes): the old hole-size tests decide, as before.
    // round_class: 0 small arms (pistol, buckshot pellet), 1 rifle, 2 heavy round or slug - the class of the head burst events
    // (HeadFracture.h) and, when the event is built, of the forensic shape draws (WoundShapePolicy.h).
    // kill_sever: Lua's per-shot decision (a chance by ammo kind) whether THIS shot, if it is the killing hit, tears the limb or
    // neck it went through: 0 no, 1 yes (killSeverRound below; every other condition of that tear stays in the worker).
    int round_class{-1},kill_sever{-1};
    // melee (2026-11-12, value 3 only; MeleeImpact.h, PainterMeleeEvent.h): 1 slash, 2 stab, 3 chop (a heavy blade's slash), 4 blunt; 0 = a
    // bullet or a blast, as before. melee_sever: a blade blow that may cut the part it crossed through (Lua's decision); melee_reach: the
    // thickest part (diameter, m) it cuts through; dent_radius / dent_depth: a blunt blow's dent (m; the worker keeps the body's list).
    // melee_part: a part of a blow (a gape slab of a cut, a side bore of a knocked-out crater): its blood rides on the main event.
    // melee_head: a crushing weapon's killing blow on the head bursts the skull (1 cap, 2 pieces; the head event), melee_head_strength
    // its size (0.2-1, HeadBurst.h strength).
    std::uint8_t melee{0},melee_part{0},melee_head{0};bool melee_sever{false};double melee_reach{0},dent_radius{0},dent_depth{0},melee_head_strength{0};
    std::uint64_t melee_blow{0}; // melee: the blow this event is part of (its seed; the game side renumbers sequences)
    // Slab (R18 explosion blades): an oriented box cut instead of a bore. accepted_pose_point/direction and
    // penetration_m give the box's through-axis (as for a bore), radius is its half thickness; slab_normal and
    // slab_side (pose space, like the direction) span its plane; slab_half_width bounds it along slab_side.
    bool slab{false},ragged_slab{false};
    Point slab_normal,slab_side;
    double slab_half_width{0};
};
// klass (2026-11-11): the round tests that used to read the hole's radius, in one place for the worker and the game side.
// The killing-shot tear (LiveDamageWorker.cpp, R62 legacy dismemberment): Lua's decision when it gave one, else radius >= 7.5 mm.
inline bool killSeverRound(const Impact&e)noexcept{return e.kill_sever>=0?e.kill_sever>0:e.radius>=.0075;}
// The R63 sever hold (PainterVoxelLive_impl.h): a killing hit is held for that tear, and on the head bone also for the head burst
// events of a rifle-class or heavier round (HeadFracture.h). Without the two fields both parts are radius >= 7.5 mm, as before.
inline bool severHoldRound(const Impact&e)noexcept{
    return killSeverRound(e)||(e.hit_bone==e.cranial_bone&&(e.round_class>=0?e.round_class>=1:e.radius>=.0075));}
struct WoundSurface {
    Point rest_point,rest_normal;
    std::array<double,64> weights{};
    bool valid{};
};
struct ImpactResult {
    body_hit_outcomes::Tag hit_trace;
    std::size_t removed_mask_cells{}; // diagnostic: includes clothing-space mask cells
    std::uint64_t sequence{};
    GoreControls gore; // The same snapshot follows committed effects and delayed attachments.
    std::uint64_t explosion_world{},explosion_cookie{},explosion_target_birth{},explosion_member_token{};
    bool removed{},miss{},penetrated{};
    bool held_contact{}; // bodypass (log only): removed through the nothinghits rescue (reason anatomy_removed_held_contact)
    // gearbody (traced events only): the merged gear child the hit was mapped onto; or, mapped onto the body, the merged gear child
    // the ray crossed in front of that point and how far in front (m). 65535 = none.
    std::uint16_t gear_child{65535},gear_on_ray{65535};double gear_on_ray_m{};
    std::string ammo_section;
    std::uint16_t hit_bone{};
    std::uint16_t initiator{65535};
    double radius{},exit_radius{},hit_power{};
    WoundSurface entry_surface,exit_surface;
    // Lateral vent exit (see Impact::vent): a third wound mouth when the vent reached the skin.
    WoundSurface vent_surface;bool vented{};double vent_radius{};
    double cavitation{},burst{},vent{}; // forensic parameters this cut used (reported to Lua)
    double entry_burst{},entry_radius{}; // entry crater strength and mouth radius (0: a plain entry)
    // Recoverable per-event rejection. Sequence is consumed; no cells changed.
    std::string rejection;
    Point rest_entry,rest_direction,cutter;
    std::uint32_t source_face{};
    std::array<double,3> barycentric{};
    std::size_t removed_solid_cells{};
    std::array<double,64> removed_m3_by_slot{}; // removed anatomy volume per material slot (1 connective, 2 bone, 3 brain, 4 fluid, 5 muscle, 6 lumen, 7.. organs)
    std::size_t edge_contact_cells{}; // Analytically intersected cells resolving checkerboard edges.
    // Damage-threshold sever (SeverCheck.h) applied with this cut: the segment's pivot bone, the tear centre and
    // axis (rest space), what was left there, and the cells the tear removed. dangling: torn free with tissue
    // left to hang by (the engine tethers the piece at the tear).
    bool sever{},sever_dangling{};std::uint16_t sever_bone{65535};Point sever_point,sever_axis;
    std::uint16_t sever_probe_bone{65535};double sever_probe_tissue{1},sever_probe_bone_left{1}; // the weakest station this cut touched when nothing tore
    double sever_remaining_tissue{1},sever_remaining_bone{1};std::size_t sever_cells{};
    // T1 (2026-10-29): the anatomy torn end of the sever reported here (TissueTear.h), for the release log's sever line:
    // 0 no torn end recorded, 1 built, 2 refused (torn_end_refusal says why); its surface triangles and time (ms)
    std::uint8_t torn_end{};std::string torn_end_refusal;std::size_t torn_end_triangles{};double torn_end_ms{};
    // melee (round 3, value 3): a blade blow's cut, the chop splash fields (Lua receipt.melee_cut; LiveDamageWorker.cpp): the cut's id on
    // this body / piece session (0 = not a blade cut), this blow's number on it, the cut's mouth centre on the skin, this blow's plane
    // normal, travel and edge (rest space), the cut's length and depth so far (m), the fraction of the part's section cut (-1 unmeasured)
    std::uint32_t melee_cut_id{},melee_cut_blow{};Point melee_cut_centre,melee_cut_normal,melee_cut_dir,melee_cut_edge;
    double melee_cut_length{},melee_cut_depth{},melee_cut_fraction{-1};
    // bulletburst (BulletBurst.h): this shot's body event - 0 none, 1 a surface blowout (crater), 2 a body burst whose fragments the
    // corpse's separation takes off, 3 a burst blown out (a crater: a living body, or fragments too small to fly). Never a
    // sever by itself. point: where the
    // fragments are thrown out from (rest space, behind the burst), strength: the round's (HeadBurst.h strength, 0..1)
    std::uint8_t bullet_burst{};Point bullet_burst_point;double bullet_burst_strength{};
};
// T1: the torn-end fields of a sever line (main body and piece sessions): "torn_end=built tris=39891 tear_ms=104.4",
// "torn_end=refused reason=tissue_tear_box_budget tear_ms=0.3" or "torn_end=none"
inline std::string tornEndText(const ImpactResult&r){char b[224];
    if(r.torn_end==1)std::snprintf(b,sizeof b,"torn_end=built tris=%llu tear_ms=%.1f",(unsigned long long)r.torn_end_triangles,r.torn_end_ms);
    else if(r.torn_end==2)std::snprintf(b,sizeof b,"torn_end=refused reason=%s tear_ms=%.1f",r.torn_end_refusal.empty()?"unknown":r.torn_end_refusal.c_str(),r.torn_end_ms);
    else std::snprintf(b,sizeof b,"torn_end=none");
    return b;}
struct WorkerPhaseTimes {
    double removal_ms{},boundary_ms{},exterior_ms{},join_ms{},pack_ms{};
};
struct Output {
    Stamp stamp;
    gfy_packed::Mesh mesh;
    std::vector<ImpactResult> impacts;
    std::vector<JointDamage> joints; // Cumulative candidate; consume ONLY after native commit.
    CranialDamage cranial; // Separate cumulative cranial integrity measure, not a sever station.
    std::string error;
    double worker_ms{};
    WorkerPhaseTimes worker_phases; // Cumulative, including a rejected rough attempt.
    std::size_t batch_events{},tiles{},dirty_tiles{},triangles{};
    std::size_t numerical_open_edges{},numerical_seams{},numerical_spurs{},interior_sheet_junctions{};
    double numerical_open_length_m{}; // Float render residual, not watertightness.
    bool no_geometry_change{};
    unsigned analytic_coverage_requirements{};
    bool analytic_diagnostic{};
    std::string analytic_refusal;
    std::uint8_t mask_only{}; // 0 off, 1 mask-only publication (no wound geometry), 2 full path after a sever (switch on)
    std::size_t analytic_new_cuts{},analytic_new_triangles{},analytic_cavity_triangles{};
    double analytic_plan_ms{},analytic_weight_ms{},analytic_pack_ms{},analytic_append_ms{},analytic_mask_ms{};
    bool complete_geometry_rebuild{}; // Same resolved candidate; no event replay.
    double renderer_rebuild_ms{};
    // A failed irregular candidate may retry once from committed damage with
    // the circular footprint. Both attempts must pass the same topology gate.
    std::string geometry_retry_reason;
    std::string accessory_skipped; // ownersafe: gear (accessory) work this job skipped and why; empty = none
    bool irregular_edges{};
    // A recoverably rejected multi-event batch resolves only its earliest
    // event (splits=1, batch_reason = the whole-batch rejection) and defers
    // the rest as isolated work, dispatched one event per job before fresh
    // events. isolated_event marks such a single-event job.
    std::size_t geometry_batch_splits{},deferred_events{};
    std::string geometry_batch_reason;
    bool isolated_event{};
    std::string resource_failure;
    // Admission diagnostics: largest estimated transient join bytes of this
    // job, with the boundary quads, joined source faces, replaced committed
    // region triangles, committed body triangles and regions it was computed
    // from; heavy_join when it took the one-heavy-join slot; memory deferrals
    // this batch already had.
    std::uint64_t join_estimate{};std::size_t join_quads{},join_faces{},join_triangles{},join_body_triangles{},join_regions{};
    bool heavy_join{};std::uint32_t memory_deferrals{};
};
class Asset;
std::shared_ptr<const Asset> prepare_asset(const AssetInput&) noexcept;
// gearbody: the prepared outfit's gear line (merged / stayed children; empty: the outfit has no gear surfaces)
std::string gear_body_note(const std::shared_ptr<const Asset>&) noexcept;
const char* last_failure() noexcept;
// ---- Allocation failure and memory admission (OOM safety) ----
// Engine allocation failure is process-fatal by default. Every Painter-owned
// thread (wound pool, asset preparation, separation, piece workers and all of
// their helpers) passes the kernel's failure hook to the installed policy
// before any work, and kernel API calls made from an engine thread pass it for
// the duration of the call only. The engine policy is xr_set_thread_oom_hook:
// on those threads an allocation failure throws std::bad_alloc into kernel
// code instead of the fatal report. The job then reports its events as not
// applied ("out_of_memory"), once after one memory deferral, while committed
// occupancy and the last published geometry stay unchanged; the owner stays
// usable. Without a policy (standalone tools) operator new throws as usual.
using AllocationFailureHook=int(*)(std::size_t);
// Sets the calling thread's hook; returns the thread's previous hook.
using ThreadAllocationPolicy=AllocationFailureHook(*)(AllocationFailureHook);
void install_thread_allocation_policy(ThreadAllocationPolicy) noexcept;
// System memory seen by admission: available physical memory and the commit
// this process can still make (GlobalMemoryStatusEx: the smaller of the system
// and process/job commit limits). Cached for a few milliseconds.
struct MemoryStatus {std::uint64_t avail_phys{},avail_commit{},total_phys{};bool valid{};};
using MemoryQuery=MemoryStatus(*)();
void install_memory_query(MemoryQuery) noexcept; // null restores the default
MemoryStatus query_memory() noexcept;
// Admission policy. Joins are estimated from the region size BEFORE they run
// (boundary quads and joined source faces). A job whose estimate reaches
// heavy_bytes is heavy: at most one heavy job joins at a time (the others wait
// for it), and it starts only while avail_commit >= estimate+commit_margin.
// Short of commit, the batch is deferred (its events return unchanged to the
// owner queue, dispatch waits for memory up to max_wait_ms while the engine
// governor releases caches) again and again: a hit is never dropped for
// memory. A multi-event batch is isolated after a few deferrals (one event
// per job); only a job still short after many isolated retries reports its
// batch not applied ("low_memory"). Low physical memory only delays a heavy join, by at
// most phys_wait_ms (paging is slow, not fatal). Below dispatch_commit_floor
// no job starts before max_wait_ms. Outputs never depend on the policy.
struct MemoryPolicy {
    std::uint64_t heavy_bytes{96ull<<20},commit_margin{768ull<<20},phys_margin{256ull<<20};
    std::uint64_t dispatch_commit_floor{384ull<<20},separation_bytes{256ull<<20};
    double max_wait_ms{4000},admission_wait_ms{400},phys_wait_ms{1500};
    bool enabled{true};
};
void set_memory_policy(const MemoryPolicy&) noexcept;
MemoryPolicy memory_policy() noexcept;
// Separation: drops the process-wide cache of per-region joins (up to GFY_SEP_REGION_CACHE_MB, default 256 MB) and
// returns the bytes released. The engine governor calls it under a hard memory shortfall; outputs never depend on it.
std::size_t clear_separation_region_cache() noexcept;
struct MemoryCounters {
    std::uint64_t allocation_failures{}; // bad_alloc raised by the hook on Painter threads
    std::uint64_t oom_deferred{},oom_failed{}; // batches deferred once / reported not applied after an allocation failure
    std::uint64_t admission_deferred{},admission_failed{}; // same for the pre-join commit check
    std::uint64_t heavy_joins{},heavy_waits{},phys_waits{},dispatch_waits{};
    std::uint64_t separation_refused{},separation_oom{},piece_oom{},preparation_oom{};
    std::uint64_t evicted_sessions{},evicted_bytes{};
    std::uint64_t max_join_estimate{};
};
MemoryCounters memory_counters() noexcept;
// ESTIMATED peak transient bytes of a job that joins regions with these
// boundary quads and source faces, replacing committed regions of
// replaced_triangles, on a body whose committed output has body_triangles
// (admission only; calibrated upper bound, see build2-fixes/oom-safety REPORT).
std::uint64_t join_estimate_bytes(std::size_t quads,std::size_t faces,std::size_t replaced_triangles,std::size_t body_triangles) noexcept;
// ESTIMATED resident bytes of a prepared asset (engine asset cache byte cap).
std::size_t asset_resident_bytes(const std::shared_ptr<const Asset>&) noexcept;
class AssetPreparation {
    struct Impl;std::shared_ptr<Impl> impl_;
    explicit AssetPreparation(std::shared_ptr<Impl>);
    friend std::shared_ptr<AssetPreparation> prepare_asset_async(const AssetInput&,AssetLoader,bool) noexcept;
public:
    // Returns false while pending. Once ready, error or asset is set atomically.
    bool poll(std::shared_ptr<const Asset>& asset,std::string& error)const;
    // The player's target: this preparation runs next if it still waits (the running one is never interrupted).
    void promote()const noexcept;
};
// priority: runs before the waiting ordinary jobs and is admitted while fewer than 4 jobs wait (ordinary ones: 2).
std::shared_ptr<AssetPreparation> prepare_asset_async(const AssetInput&,AssetLoader={},bool priority=false) noexcept;
std::shared_ptr<AssetPreparation> prepare_asset_files_async(const AssetInput&,const AssetFiles&,AssetHashFunction,bool priority=false) noexcept;
enum class RendererRebuildState {none,queued,complete,refused};
struct RendererRebuildStatus {RendererRebuildState state{};const char* reason="none";};
class Session;
class ComponentSession;
class ComponentPreparation;
struct ComponentReceipt;
struct ComponentOutput;
struct SeparationInput;
struct SeparationOutput;
class SeparationPreparation;
struct PieceSeed; // continuation of one separated piece (LiveDamageWorker.cpp, PieceSeed.h)
using SeparationCheckpoint=std::function<void(const char*,std::uint32_t,std::uint16_t)>;
struct DamageSample {
    std::uint16_t anatomy_label{},material_slot{};
    bool removed{}; // Includes removed clothing-space cells where label is zero.
};
// Exact, immutable state last accepted by the renderer owner. Keeps the shared
// anatomy, sparse removal mask and matching wounded mesh alive after the NPC.
// This is transfer input, NOT an already partitioned/spawned limb.
class DamageSnapshot {
    struct Impl;std::shared_ptr<const Impl> impl_;
    explicit DamageSnapshot(std::shared_ptr<const Impl>);
    friend class Session;friend class Pool;
public:
    const Stamp& stamp()const noexcept;
    // Null means original, unmodified model. Positions/skin remain in the
    // original model's rest space; transfer must apply the captured bone pose.
    const gfy_packed::Mesh* mesh()const noexcept;
    DamageSample sample_rest(Point)const noexcept;
    double damage_pitch()const noexcept;
    // Null when disabled. Exposed only through an accepted immutable snapshot.
    // The snapshot stamp carries published renderer generation; history records
    // damage revisions only. Not transferred to detached ComponentSession.
    std::shared_ptr<const ResolvedCutHistory> resolved_cut_history()const noexcept;
    std::size_t sparse_tiles()const noexcept;
    // CPU-only continuation of one exact remaining anatomy component. The
    // caller must keep hit coordinates in this source model's original rest
    // space. This does not publish a piece mesh or create a renderer Session.
    std::shared_ptr<ComponentSession> prepare_component_session(std::uint32_t component)const noexcept;
    // CPU worker only. Geometry and collision derive from this accepted mask,
    // never from queued hits or a second destructive approximation of the cut.
    std::shared_ptr<const SeparationOutput> prepare_separation(const SeparationInput&,
        const SeparationCheckpoint& checkpoint={})const noexcept;
};
struct ComponentHit {
    Stamp source_stamp; // Accepted published parent identity; never rewritten by local hits.
    std::uint64_t local_revision{};
    std::size_t removed_solid_cells{},removed_mask_cells{},edge_contact_cells{};
    bool changed{};
    std::string error;
};
struct ComponentRemaining {std::uint32_t component{};std::uint64_t cells{};};
// Persistent CPU occupancy and published-piece surface for one separated
// component. A candidate is private until native visual/collider commit.
class ComponentSession:public std::enable_shared_from_this<ComponentSession> {
public:
    struct Impl;
    explicit ComponentSession(std::shared_ptr<Impl>);
    const Stamp& source_stamp()const noexcept;
    std::uint32_t source_component()const noexcept;
    std::uint64_t local_revision()const noexcept;
    DamageSample sample_rest(Point)const noexcept;
    std::size_t sparse_tiles()const noexcept;
    std::vector<ComponentRemaining> remaining_components()const;
    // Origin and direction are in the original source rest-space lattice.
    // The candidate is committed only after exact connectivity comparison.
    ComponentHit hit_rest(Point origin,Point direction,double radius,double penetration)noexcept;
    // Rebuild a temporary whole-parent context from immutable accepted state
    // plus this owner's new mask bits. Returns only descendants of this owner;
    // even a single updated piece is a valid continuation. CPU worker only.
    std::shared_ptr<const SeparationOutput> prepare_owned_separation(const SeparationInput&,
        bool allow_separation=true)const noexcept;
    // Separation worker initializes the exact published fragment mesh before
    // the native owner binds this session after a successful first split.
    bool initialize_published_surface(const gfy_packed::Mesh&)noexcept;
    bool can_bind_owner(Identity current_owner,std::uint64_t current_generation)const noexcept;
    bool bind_owner(Identity current_owner,std::uint64_t current_generation)noexcept;
    // Bounded CPU job: posed ray against current published piece mesh, owned
    // tunnel, virtual-parent join, physics and surface candidates. No state
    // advances until acknowledge after native visual/collider publication.
    std::shared_ptr<ComponentPreparation> submit_async(Impact,SeparationInput,
        std::uint64_t current_generation,bool allow_separation=true)noexcept;
    bool can_acknowledge(const ComponentReceipt&,std::uint64_t published_generation)const noexcept;
    bool acknowledge(const ComponentReceipt&,bool accepted,
        std::uint64_t published_generation)noexcept;
    void cancel()noexcept;
    // IDs are local to this session's current remaining topology. Use after
    // a later sever to give each descendant a disjoint continued owner.
    std::shared_ptr<ComponentSession> prepare_descendant(std::uint32_t component)const noexcept;
private:
    void run_candidate(Impact,SeparationInput,std::uint64_t,bool,
        std::shared_ptr<ComponentPreparation>)noexcept;
    std::shared_ptr<Impl> impl_;
};
struct SeparationAccessory {
    std::uint16_t child{},physical_element{65535};
    Point original_anchor; // Fixed original anatomical attachment, never nearest survivor.
};
struct SeparationInput {
    AssetFile surface_bindings,coat_roots,physics_elements,bone_frames,accessory_surfaces;
    Hash anatomy_hash{},fit_hash{};
    AssetHashFunction hash{};
    std::vector<std::uint16_t> physical_parents; // Original bone IDs; 65535 = no physical parent.
    std::vector<SeparationAccessory> accessories;
    double minimum_piece_m3{.000001};
    // An island with no bone in it below this volume is debris, not a piece (a flap of skin or muscle blown loose).
    double boneless_piece_m3{.00003};
    // Runtime selection of the separation surface source. False (default): exact
    // union re-join of the whole parent. True: partition the snapshot's already
    // published wound mesh when one exists (SEMANTIC, separation-track P2 S4; not
    // implemented yet, so the union path still runs). Needs sign-off.
    bool published_surface{};
};
struct SeparationBox {
    Point center,half_extents;
    std::array<double,9> orientation{}; // Original bone frame, row-major columns are box axes.
};
struct SeparationElement {
    std::uint16_t bone{65535};
    bool retain_original_shape{}; // Entire original element anatomy is retained untouched.
    std::vector<SeparationBox> boxes;
    double mass_kg{};
    Point com;
    std::array<double,9> inertia{}; // Original bone frame, about this element's fitted COM.
    std::uint64_t cells{};
    // bodyjitter3: a native element (retain_original_shape) of the remaining body cut back at a cut: its native shapes are
    // clipped to n.x <= d ({n.x,n.y,n.z,d}, original bone frame, at most 4) and it weighs mass_scale times the element it replaces.
    std::vector<std::array<double,4>> trims;
    double mass_scale{1};
};
struct SeparationPiece {
    std::uint32_t component{};
    bool ocular_remnant=false; // Proven eye-tissue island newly disconnected by committed damage.
    bool burst_chunk=false; // chunknosplash: a piece flying out of a burst (separation/BurstChunk.h): no cut-edge splash on it
    std::uint64_t cells{};
    std::uint16_t root_bone{65535};
    gfy_packed::Mesh mesh;
    // gearbody: boxes (original bone frames, one per element) around the merged gear this part holds - for the bullet flesh test only
    std::vector<SeparationElement> gear_flesh;
    std::shared_ptr<ComponentSession> continuation; // CPU-only, bind after native commit.
    std::shared_ptr<const PieceSeed> seed; // low-poly continuation (Pool::attach_piece), attached after native commit.
    std::vector<SeparationElement> elements;
    std::vector<std::uint16_t> pose_anchors; // Only original rigid-child→same-piece physical ancestor.
    std::vector<std::uint16_t> retained_accessories;
};
// Worker-side phase timings of one separation, named after the checkpoint that
// starts each phase. Instrumentation only; never part of an output digest.
enum SeparationPhase : unsigned {
    separation_phase_remaining_connectivity,separation_phase_original_connectivity,
    separation_phase_compare_connectivity,separation_phase_surface_bindings,
    separation_phase_original_physics_map,separation_phase_physical_partitions,
    separation_phase_surface_partition,separation_phase_colliders_and_pose,
    separation_phase_count
};
struct SeparationOutput {
    Stamp stamp;
    GoreControls gore; // Frozen accepted damage policy that produced this partition.
    std::vector<SeparationPiece> pieces;
    std::uint32_t source_component{};
    bool owned_continuation{};
    bool combined_owner{}; // Policy-off: disconnected owned descendants remain one object.
    std::uint32_t original_component{}; // Component ID in the accepted parent snapshot.
    std::uint64_t local_revision{}; // Revision within that original component.
    std::vector<std::uint16_t> original_surface_children;
    std::string error;
    bool no_separation{};
    std::uint32_t original_components{},remaining_components{},largest_substantial_split{};
    double worker_ms{};
    std::array<double,separation_phase_count> phase_ms{}; // Of the prepare() call that produced the pieces.
    double continuation_ms{};
    std::string lowpoly_summary; // LowPolyPieces.h statistics of this separation (log only) // component_continuation of every piece (after prepare()).
};
struct ComponentReceipt {
    Identity owner; // Current published fragment object, not the parent NPC.
    Stamp parent_stamp; // Immutable accepted source/rig lineage.
    std::uint64_t expected_generation{},expected_local_revision{},next_local_revision{},sequence{};
};
struct ComponentOutput {
    ComponentReceipt receipt;
    ImpactResult impact;
    std::shared_ptr<const SeparationOutput> separation;
    bool no_geometry_change{};
    bool separation_allowed{};
    std::string error;
};
class ComponentPreparation {
public:
    struct Impl;
    explicit ComponentPreparation(std::shared_ptr<Impl>);
    std::shared_ptr<const ComponentOutput> completed()const;
private:
    friend class ComponentSession;
    std::shared_ptr<Impl> impl_;
};
class SeparationPreparation {
    struct Impl;std::shared_ptr<Impl> impl_;
    explicit SeparationPreparation(std::shared_ptr<Impl>);
    friend std::shared_ptr<SeparationPreparation> prepare_separation_async(std::shared_ptr<const DamageSnapshot>,SeparationInput) noexcept;
public:
    ~SeparationPreparation();
    const Stamp& stamp()const noexcept;
    SeparationProgress progress()const;
    void cancel();
    // Nonblocking owner-thread poll; null until the worker has finished.
    std::shared_ptr<const SeparationOutput> completed()const;
};
std::shared_ptr<SeparationPreparation> prepare_separation_async(std::shared_ptr<const DamageSnapshot>,SeparationInput) noexcept;
// Mask-only wounds (console painter_voxel_mask_only): read by every later job; default off.
void set_mask_only(bool on) noexcept;
// 0 off, 1 slice 1 (intact bodies), 2 slice 1 + severs and detached pieces (slice 2)
void set_mask_only_mode(int mode) noexcept;
int mask_only_mode_value() noexcept;
void set_seam_rule(int on) noexcept; // damagecut: console painter_seam_rule (1 on, the default; 0 off: SeamTear.h)
int seam_rule_value() noexcept;
void set_sever_method(int m) noexcept; // damagecut: console painter_sever_method (0 the old severs, 1 the new: default)
int sever_method_value() noexcept;
void set_holding_cut(int on) noexcept; // damagecut3: console painter_holding_cut (0 no holding-cells search: the default; 1 on)
int holding_cut_value() noexcept;
void set_island_fill(int on) noexcept; // headmask2: console painter_island_fill (0 no island fill: the default; 1 on)
int island_fill_value() noexcept;
void set_body_burst_chance(float chance) noexcept; // headfix: console painter_body_burst_chance (value 3 blasts: body burst or legacy cuts)
float body_burst_chance_value() noexcept;
void set_damage_mode(int mode) noexcept; // modes: console painter_damage_mode (0 Normal, 1 Bullet holes only, 2 Extreme)
int damage_mode_value() noexcept;
void set_body_blowout_frequency(float f) noexcept; // bulletburst: console painter_body_blowout_frequency (0..2, 1 = the head's table)
void set_body_blowout_chunks(int v) noexcept; // nochunks: console painter_body_blowout_chunks (1 pieces come off, 0 blown out)
float body_blowout_frequency_value() noexcept;
// Prewarm the per-(model, quality) separation caches (pristine connectivity and
// physics map) on the low-priority warm thread, for separation_ready owners at
// bind or death. Exact: a cache returns only what a separation would compute.
// Builds only missing keys, yields to real separation jobs, and a job that needs
// a key being warmed joins that build. False: not queued (queue full or null).
bool warm_separation_async(std::shared_ptr<const Asset>,SeparationInput) noexcept;
class Pool {
    struct Impl;std::unique_ptr<Impl> impl_;
public:
    explicit Pool(unsigned worker_count=2,unsigned max_owners=32);
    ~Pool();
    // quitfix: owner thread, at the end of the game. Queued work is dropped, the job in flight ends, every pool thread is
    // joined; the pool object stays valid (later calls queue nothing that runs).
    void quit() noexcept;
    Pool(const Pool&)=delete;Pool& operator=(const Pool&)=delete;
    std::shared_ptr<Session> attach(std::shared_ptr<const Asset>,Identity) noexcept;
    // Continuation of a separated piece after its owner committed `published` (the piece mesh as installed, at
    // damage revision `revision` and renderer generation `generation`): the same worker restricted to the piece's
    // tissue, publishing that mesh plus the walls of later cuts. Its snapshots separate again.
    std::shared_ptr<Session> attach_piece(std::shared_ptr<const PieceSeed>,Identity,std::shared_ptr<const gfy_packed::Mesh> published,
        std::uint64_t revision,std::uint64_t generation) noexcept;
    // Owner barrier only. Capture the actual generation here, before dispatch.
    // False (nothing consumed) also while memory admission holds the owner.
    bool dispatch(const std::shared_ptr<Session>&,std::uint64_t current_generation) noexcept;
    // Owner barrier: keep the exact pending damage transaction while rebuilding
    // its complete geometry on this pool. Old output identity must match.
    // False leaves everything intact. Call acknowledge_output after completion.
    bool rebuild_completed(const std::shared_ptr<Session>&,const std::shared_ptr<const Output>&,
        std::uint64_t current_generation) noexcept;
    // Memory governor: drop the recomputable caches (region results, boundary,
    // tile/lattice/static and exterior memos; never committed occupancy,
    // committed geometry or queued events) of idle owners, least recently
    // active first, until at least target_bytes (estimated) were released.
    // Canceled owners always go first; owners active within min_idle_ms are
    // kept. Outputs after eviction are byte-identical; only the next job on
    // that owner recomputes more (the boundary cache is rebuilt from committed
    // occupancy). Owner-thread or any thread; never waits for a job.
    struct Eviction {std::size_t sessions{},bytes{};};
    Eviction evict_idle_caches(std::size_t target_bytes,double min_idle_ms) noexcept;
    // True while any attached owner has a batch deferred for memory (the
    // engine governor then releases caches regardless of its low-water marks).
    bool any_memory_waiting() const noexcept;
    // Estimated recomputable cache bytes held by all attached idle owners.
    std::size_t cache_bytes() const noexcept;
    // A value-only task of another engine system (the native combat AI's job queue). Tasks run only on their own lane:
    // lane threads (configure_task_lane, default 1), started with the first task, that never take wound jobs, while the
    // wound workers never take tasks, so neither queue can hold the other back. Urgent ones go ahead of the other tasks.
    // The task must not throw out (it is caught and dropped) and touches no session. False when the pool is stopping or
    // the lane cannot start.
    bool submit_task(std::function<void()> task,bool urgent=false) noexcept;
    // Lane threads for submit_task, 1..4 (default 1); a larger count starts the extra threads with the next task.
    void configure_task_lane(unsigned workers) noexcept;
};
// Most owners one pool serves (bodies, stumps and severed pieces with a live session). The engine asks for exactly
// this, and Pool refuses more, so the two cannot drift apart again.
constexpr unsigned pool_max_owners=256;
std::unique_ptr<Pool> make_pool(unsigned workers=2,unsigned max_owners=32) noexcept;
// quitfix: main thread, when the game objects are gone and before main returns, after Pool::quit of every pool. Stops and joins
// the kernel's own worker threads (asset preparation, separation, instant sever, prefit, warm, tile bases, fragment work,
// retirement, then the shared worker pool): none of them is joined by a destructor at process exit any more.
void quit_workers() noexcept;
#ifdef GFY_VOXEL_WORKER_TESTS
namespace worker_test {struct Access;} // TEST-ONLY internal inspection; absent in production.
#endif
class Session {
    struct Impl;std::unique_ptr<Impl> impl_;explicit Session(std::unique_ptr<Impl>);
    friend class Pool;friend struct PieceSessionOps; // piece sessions (PieceSession_impl.h)
    static std::size_t cache_bytes_locked(const Impl&) noexcept;
    bool acknowledge_impl(const Stamp&,bool,std::uint64_t,const Output*);
#ifdef GFY_VOXEL_WORKER_TESTS
    friend struct worker_test::Access;
#endif
public:
    ~Session();
    bool enqueue(Impact) noexcept; // Sequence checked; bounded queue; non-blocking work.
    std::shared_ptr<const Output> completed()const; // Poll; never waits for job.
    // published_generation is the renderer version AFTER installation. The
    // output stamp still authenticates its original pre-install expectation.
    bool acknowledge(const Stamp&,bool accepted,std::uint64_t published_generation=UINT64_MAX);
    // Exact output identity is mandatory after requesting a renderer rebuild;
    // a stale completion with the same damage stamp cannot acknowledge it.
    bool acknowledge_output(const std::shared_ptr<const Output>&,bool accepted,
        std::uint64_t published_generation=UINT64_MAX);
    RendererRebuildStatus renderer_rebuild_status()const noexcept;
    // After the renderer refuses analytic geometry once, this owner keeps complete geometry.
    void disable_analytic_channels()noexcept;
    void cancel()noexcept; // Terminal; worker-held CPU memory outlives owner.
    bool pending()const;
    // Owner wake-up, run on the pool thread outside every kernel lock right
    // after this owner's output (or failure report) became visible to
    // completed(). Optional: the owner still polls and acknowledges as before.
    void set_ready_hook(std::function<void()>) noexcept;
    bool supports_bone(std::uint16_t bone)const;
    bool piece()const noexcept; // continuation of a separated piece (Pool::attach_piece)
    // O(1) capture at an idle owner barrier. Refuses queued, in-flight,
    // unacknowledged, failed or canceled owners; never exports provisional cuts.
    std::shared_ptr<const DamageSnapshot> capture_damage()const;
    std::shared_ptr<const painter_detail::TissueField> tissue_field()const;
    // Memory admission state of this owner (for logging).
    struct MemoryState {std::uint64_t need_commit{},deferrals{},failures{};std::size_t cache_bytes{};bool waiting{};};
    MemoryState memory_state()const noexcept;
    // Governor: drop this owner's recomputable caches if it is idle (see
    // Pool::evict_idle_caches). Returns the estimated bytes released.
    std::size_t evict_caches()noexcept;
};
// Terminally cancel an owner and transfer its strong reference to the existing
// retirement lane. In-flight workers keep their own references. Safe synchronous
// destruction is the fallback if retirement admission fails.
void retire_session(std::shared_ptr<Session>) noexcept;
// Decisive low-level directional traversal / posed mapping tests.
void run_directional_tests();
void run_tunnel_tests();
}
