// body_hit_outcomes_v1
// Explicit native hit diagnostic. CPU worker is compiled separately with /EHsc.
// No helper calls a prepared state; accepted fire-wound SHit events drive jobs.
extern int g_goyu_painter_asset_cap_mb; // GOYU: console_commands.cpp (painter_asset_cap_mb)
#include <deque>
#include <chrono>
#include "PainterBurnHost.h"
#include "painter_voxel/PainterPieceContinuation.h"
#include "painter_voxel/PainterExplosions.h"
#include "../Layers/xrRender/BloodNativeProfile.h"
#include "../Layers/xrRender/PainterHitMain.h" // triggerhitch
#include "PainterCommitParts.h" // piececommit
#include "../xrEngine/PainterQuitWatch.h" // quitfix: quit hook
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <memory>
#include <chrono>
#include <string>
#include <atomic>
#include <cmath>
#include <map>
#include <set>
#include <psapi.h>
#pragma comment(lib,"psapi.lib")
#include <algorithm>
#include "painter_voxel/LiveDamageWorker.h"
#include "painter_voxel/BlastSevers.h"
#include "painter_voxel/PainterDamagedFragment.h"
#include "PainterDamagedFragmentProducer_impl.h"
#include "ai_space.h"
#include "../xrServerEntities/script_engine.h"
#include "../Include/xrRender/RuntimePackedBridge.h"
#include "../Include/xrRender/Kinematics.h"
#include "../Include/xrRender/RuntimeGeometryStage.h"
#include "PainterVoxelLive_profile.h"
#include "PainterVoxelLifecycle.h"
#include "../Include/xrRender/PainterLogVerbose.h"
#include "PainterCommittedEffects_impl.h"
#include "PainterWoundEvent.h"
#include "PainterMeleeEvent.h" // melee (value 3): knife / blade / blunt hits
#include "PainterCranialRemnants_impl.h"
#include "xr_alloc.h" // LuaJIT pool (3rd party/luajit-2/src): commit-on-demand chunks, out-of-commit release hook
extern "C" unsigned char* SHA256(const unsigned char*,size_t,unsigned char*);
int painter_voxel_quality=1; // Former Low/Medium/High console value; kept registered for old scripts, ignored.
// Mask-only wounds (slice 1: bullet wounds on an intact body): console painter_voxel_mask_only 1; default 0 = today.
int painter_voxel_mask_only=3;
int painter_voxel_cell_mm=2;
float painter_body_burst_chance=0.35f;
int painter_damage_mode=0;
float painter_body_blowout_frequency=1.f;
int painter_body_blowout_chunks=1;
// boneresist (2026-11-19): console painter_bone_resist 1 / 0, saved in user.ltx - the Painter menu's check box
int painter_bone_resist=1;
// bonefix2 (2026-11-20): console painter_bone_strength 0..100, saved in user.ltx - the Painter menu's slider
int painter_bone_strength=70;
inline int painter_voxel_effective_quality(){return painter_voxel_cell_mm==1?1:0;}
int painter_voxel_live_result=0,painter_voxel_live_owner=65535,painter_voxel_live_revision=0;
// Wake-up link between a kernel session and its engine owner. The pool thread
// reads `support` under Device.seqParallelBeforRenderCS; CancelPainterVoxelLive
// clears it under the same section, so no wake can target a destroyed owner.
struct PainterVoxelReadyLink { CCharacterPhysicsSupport* support=nullptr; };
struct PainterVoxelLiveOwner {
    std::shared_ptr<gfy_live::Session> worker;
    std::shared_ptr<PainterVoxelReadyLink> ready_link; // set at bind, cleared by CancelPainterVoxelLive
    gfy_live::Identity identity;
    xr_vector<RuntimeSkinBone> exact_rig;
    xr_vector<shared_str> tissue_textures;
    IKinematics* visual=nullptr;CPhysicsShell* shell=nullptr;
    u64 sequence=0,geometry=0;DWORD thread=0;
    u16 original_children=0;
    bool burn_prime_wanted=false,burn_prime_ready=false,burn_prime_queued=false;
    bool separation_ready=false;
    gfy_live::SeparationInput separation_input;
    std::shared_ptr<gfy_live::SeparationPreparation> separation_job;
    u64 flight_attempted_revision=0;
    u64 separation_checked_revision=0;
    u32 separation_native_retries=0; // retries after a transient native refusal (moved/stale/busy), bounded
    u32 separation_cancel_retries=0; // R33: retries after a staged split was canceled at commit (pose mismatch, resample, swap not ready...), bounded
    double separation_progress_reported_ms=0;
    u32 separation_policy_retry_at=0;
    painter_voxel_binding::Lifecycle lifecycle;
    // Memory: separation retries after out_of_memory/low_memory, and the
    // revision whose publication already logged a low-memory deferral.
    u32 separation_memory_retries=0;
    u64 publication_deferred_revision=0;
    u64 separation_hold_revision=0; // sever revision whose wound-publication hold was logged
    u64 sever_revision=0;u32 sever_hold_started=0; // last committed revision with a sever, and when it committed
    u32 split_imminent_at=0; // a check found the body apart on an older revision: wounds wait for the newest check (bounded)
    u32 hold_hit_time=0;u16 hold_blast_explosive=u16(-1);u32 hold_blast_time=0; // R63 sever hold candidates (killing hit / planned force tears)
    painter_committed_effects::Queue committed_effects;
    bool shader_warm_requested=false; // bind ran off the main thread (save load): warm at the next main-thread update
    // Off-thread publication in flight (r_painter_publish 1/2): the renderer
    // prepares stage_out's geometry on its own thread. stage_out stays
    // unacknowledged until the atomic install at a later barrier.
    IRuntimeGeometryStage* stage=nullptr;
    std::shared_ptr<const gfy_live::Output> stage_out;
    std::shared_ptr<const gfy_live::Output> required_field_hold; // Explicit refusal; cleared by owner cancellation/rebind.
    u32 stage_frame=0;float stage_barrier_ms=0.f;CTimer stage_wait;
    std::deque<std::pair<u64,std::chrono::steady_clock::time_point>> hit_times; // sequence -> accepted hit, for hit_to_visible
    void drop_stage(){if(stage){stage->Cancel();stage->Release();stage=nullptr;}stage_out.reset();}
    PainterVoxelLiveOwner()=default;
    PainterVoxelLiveOwner(const PainterVoxelLiveOwner&)=delete;
    PainterVoxelLiveOwner& operator=(const PainterVoxelLiveOwner&)=delete;
    ~PainterVoxelLiveOwner(){drop_stage();}
};
namespace painter_voxel_native {
// Releases large CPU-only separation results off the main thread (no engine objects inside).
struct Disposer {
    std::mutex mutex;std::condition_variable wake;std::vector<std::shared_ptr<const void>> queue;bool stop=false;std::thread worker;
    Disposer():worker([this]{for(;;){std::vector<std::shared_ptr<const void>> batch;
        {std::unique_lock<std::mutex> lock(mutex);wake.wait(lock,[&]{return stop||!queue.empty();});if(queue.empty()&&stop)return;batch.swap(queue);}
        batch.clear();}}){}
    void quit()noexcept{{std::lock_guard<std::mutex> lock(mutex);stop=true;}wake.notify_one();if(worker.joinable())worker.join();} // quitfix
    ~Disposer(){quit();}
    void push(std::shared_ptr<const void> value){if(!value)return;{std::lock_guard<std::mutex> lock(mutex);queue.push_back(std::move(value));}wake.notify_one();}
};
// quitfix: never destroyed (no join at process exit); stopped by quit_workers() below.
inline Disposer*& disposerSlot(){static Disposer* slot=nullptr;return slot;}
inline void dispose(std::shared_ptr<const void> value){static Disposer& disposer=*(disposerSlot()=new Disposer);disposer.push(std::move(value));}
// triggerhitch: the time is split by the part reached (slow.part="..." starts the next part). One hit_main line per call over
// 2 ms (an apply / poll call; a capture inside a received hit is on that hit's line); the slow_main line from 8 ms as before.
struct SlowScope : painter_commit_parts::Sink { // piececommit: the commit path's sub-parts (painter_commit_parts::part) land here
    struct Label {SlowScope* owner;const char* name;
        Label& operator=(const char* next){owner->mark(next);return *this;}
        operator const char*()const{return name;}};
    const char* scope;u16 id;Label part{this,"start"};CTimer timer;
    const char* names[32]{};float spent[32]{};unsigned used=0;float last=0.f;painter_commit_parts::Sink* outer_sink=nullptr; // piececommit: 8 -> 32 parts
    SlowScope(const char* s,u16 i):scope(s),id(i){timer.Start();outer_sink=painter_commit_parts::sink;painter_commit_parts::sink=this;}
    void enter(const char* next)override{mark(next);}
    SlowScope(const SlowScope&)=delete;SlowScope& operator=(const SlowScope&)=delete;
    void add(const char* n,float ms){for(unsigned i=0;i<used;++i)if(names[i]==n||!std::strcmp(names[i],n)){spent[i]+=ms;return;}
        if(used<32){names[used]=n;spent[used++]=ms;}}
    void mark(const char* next){const float now=timer.GetElapsed_ms_f();add(part.name,now-last);last=now;part.name=next;}
    ~SlowScope(){painter_commit_parts::sink=outer_sink;const float ms=timer.GetElapsed_ms_f();
        if(ms>=8.f)Msg("[painter_perf] id=%u phase=slow_main scope=%s part=%s ms=%.1f",id,scope,part.name,ms);
        if(ms<float(painter_hit_main::threshold_ms)||painter_hit_main::current||!painter_hit_main::admit_line())return;
        add(part.name,ms-last);unsigned top=0;for(unsigned i=1;i<used;++i)if(spent[i]>spent[top])top=i;
        char parts[1400];int at=0;
        for(unsigned i=0;i<used&&at>=0&&at<int(sizeof(parts));++i)at+=std::snprintf(parts+at,sizeof(parts)-at," %s=%.2f",names[i],spent[i]);
        if(at<0)at=0;if(at>=int(sizeof(parts)))at=int(sizeof(parts))-1;parts[at]=0;
        Msg("[painter_perf] phase=hit_main id=%u stage=%s part=%s ms=%.2f skipped=%u%s",id,scope,used?names[top]:part.name,ms,painter_hit_main::take_skipped(),parts);}
};
std::unique_ptr<gfy_live::Pool> pool;
// quitfix (2026-11-13): the end of the wound kernel's threads. Run by the engine on the main thread when the game objects are
// gone and before the device and main() end (x_ray.cpp, PainterQuitWatch::runQuitHooks): the damage pool, the kernel's own
// services and the disposer are stopped and joined while everything their jobs use still exists. Before, their destructors
// joined them at process exit, where a job in flight could wait for ever on a worker pool that was already gone.
inline void quit_workers()noexcept{
    painter_burn_host::quit();
    if(pool)pool->quit();
    gfy_live::quit_workers();
    if(auto* disposer=disposerSlot())disposer->quit();
}
static const bool quit_workers_hooked=(PainterQuitWatch::atQuit(&quit_workers),true);
struct PreparedQuality {
    gfy_live::AssetInput input;
    std::shared_ptr<const gfy_live::Asset> asset;
    std::shared_ptr<gfy_live::AssetPreparation> preparing;
    gfy_live::Hash prepared_rig{};
    bool fullbody=false, files_on_disk=false;
    gfy_live::AssetFiles files;
    std::string preparation_error;
    xr_vector<shared_str> tissue_textures;
    u64 access=0;
    bool warm=false;
    bool separation_ready=false;
    gfy_live::SeparationInput separation_input;
    std::string separation_reason="separation_metadata_unavailable";
};
PreparedQuality qualities[3];
std::map<std::string,PreparedQuality> model_assets;
u64 model_access=0;
u64 next_cookie=0x564f580000000001ull;
PreparedQuality& selected_quality(){return qualities[painter_voxel_effective_quality()];}
// ---- OOM safety and memory governor (see painter_voxel/LiveDamageWorker.h) ----
// Available physical memory and the commit this process can still make.
// Commit telemetry: how much the OS has promised in total and its limit (RAM + pagefiles), so a player's log
// shows whether the limit itself is exhausted (dev PC 2026-09-25: 33.9 of 42.1 GB committed before the game).
struct CommitState {u64 total_mb{},limit_mb{};bool valid{};};
CommitState commit_state(){PERFORMANCE_INFORMATION pi{};pi.cb=sizeof(pi);CommitState c;
    if(GetPerformanceInfo(&pi,sizeof(pi))){c.total_mb=(u64(pi.CommitTotal)*pi.PageSize)>>20;c.limit_mb=(u64(pi.CommitLimit)*pi.PageSize)>>20;c.valid=true;}return c;}
void log_pagefile_config(){HKEY key{};char value[1024]{};DWORD size=sizeof(value)-2,type=0;
    if(RegOpenKeyExA(HKEY_LOCAL_MACHINE,"SYSTEM\\CurrentControlSet\\Control\\Session Manager\\Memory Management",0,KEY_READ,&key)==ERROR_SUCCESS){
        if(RegQueryValueExA(key,"PagingFiles",nullptr,&type,reinterpret_cast<LPBYTE>(value),&size)==ERROR_SUCCESS){
            for(DWORD n=0;n+1<size;++n)if(!value[n]&&value[n+1])value[n]=';';value[size]=0;
            Msg("[painter_memory] phase=pagefile_config entries=\"%s\"",value[0]?value:"(system managed / none)");}
        RegCloseKey(key);}}
gfy_live::MemoryStatus system_memory(){
    MEMORYSTATUSEX m{};m.dwLength=sizeof(m);if(!GlobalMemoryStatusEx(&m))return {};
    gfy_live::MemoryStatus s;s.avail_phys=m.ullAvailPhys;s.avail_commit=m.ullAvailPageFile;s.total_phys=m.ullTotalPhys;s.valid=true;return s;
}
// Commit cushion: 128 MB committed at policy install and never touched (commit
// charge only, no physical memory). The engine out-of-memory handler of every
// thread without a Painter hook (main/publication, render, sound, scripts)
// releases it once and retries the failed allocation, so a game allocation
// that meets exhausted commit gets headroom instead of a fatal error. The
// governor then releases recomputable caches, holds Painter work, and re-arms
// the cushion after memory recovered. Lock-free and allocation-free: it runs
// inside a failed allocation on an arbitrary thread (no Painter locks here).
const SIZE_T cushion_bytes=SIZE_T(128)<<20;
std::atomic<void*> cushion{nullptr};
std::atomic<u32> emergency_releases{0};
bool release_cushion(size_t){
    void* block=cushion.exchange(nullptr);if(!block)return false;
    VirtualFree(block,0,MEM_RELEASE);emergency_releases.fetch_add(1,std::memory_order_relaxed);return true;
}
bool arm_cushion(){
    if(cushion.load())return true;void* block=VirtualAlloc(nullptr,cushion_bytes,MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE);if(!block)return false;
    void* expected=nullptr;if(!cushion.compare_exchange_strong(expected,block))VirtualFree(block,0,MEM_RELEASE);return true;
}
// The LuaJIT pool commits its 64 KB chunks on demand above a committed floor (xr_alloc.c). When
// a chunk commit fails it calls this once and retries, so a script allocation that meets
// exhausted commit gets the cushion instead of a Lua out-of-memory error, which the script
// engine treats as fatal. Same contract as release_cushion: allocation-free, any thread, and
// a second call after the cushion is gone just reports nothing released.
int release_cushion_for_lua(size_t size){return release_cushion(size)?1:0;}
// Recomputable resident budget for prepared model assets (plus the 32 count cap).
u64 asset_cap_bytes=0;
// The kernel runs the installed allocation policy (a) at the start of every thread it creates and before each pool
// job (core::enterPainterThread / painterThreadEntry, passing the kernel's failure hook) and (b) on ENGINE threads
// around every kernel API call (AllocationScope: the kernel hook on entry, the thread's previous hook, null, restored
// on return). This wrapper installs the OOM hook exactly as before and, on kernel threads only, applies the console
// thread priority (painter_thread_priority, device.h) and the allocator's no-zero-fill mode (the kernel's outputs
// do not depend on zeroed heap: the offline probe uses the plain CRT allocator). An engine thread is recognised by
// the main-thread id or, for any other engine thread, by its restore call (hook==nullptr), which only AllocationScope
// makes; a thread so marked never has its priority or allocator mode changed again (a non-main engine thread pays
// one microsecond-scale window on its first kernel call before that restore marks it). The renderer's publish
// thread never runs this policy, so it keeps zero-fill on; its priority is set in RuntimeStageExecutor::Loop.
struct PainterThreadState{bool engine=false,painter=false;};
PainterThreadState& painter_thread_state(){static thread_local PainterThreadState s;return s;}
xr_oom_hook painter_thread_entry(xr_oom_hook hook){
    const xr_oom_hook previous=xr_set_thread_oom_hook(hook); // unchanged behaviour of the policy itself
    auto& s=painter_thread_state();
    if(!hook){ // AllocationScope restore: only an engine thread does this
        if(s.painter){SetThreadPriority(GetCurrentThread(),THREAD_PRIORITY_NORMAL);::xr_memory_thread_no_zero_fill(false);s.painter=false;}
        s.engine=true;return previous;}
    if(s.engine||GetCurrentThreadId()==Device.m_main_thread_id)return previous; // engine thread: nothing else changes
    painter_apply_thread_priority(); // cached per thread; pool threads re-read the console value before each job
    if(!s.painter){::xr_memory_thread_no_zero_fill(true);s.painter=true;}
    return previous;
}
// Installed once, before the first pool, preparation or separation thread exists.
void install_memory_policy(){
    static bool installed=false;if(installed)return;installed=true;
    gfy_live::install_thread_allocation_policy(&painter_thread_entry);
    gfy_live::install_memory_query(&system_memory);
    // Margins sized for players near their commit limit (in-game 2026-09-25: 0.7-1.3 GB free commit, game fine; the former
    // 1 GB margin / 512 MB floor refused 32 of 59 hits). Real allocation failures still fail only one wound.
    gfy_live::MemoryPolicy policy;policy.heavy_bytes=96ull<<20;policy.commit_margin=128ull<<20;policy.phys_margin=256ull<<20;
    policy.dispatch_commit_floor=64ull<<20;policy.separation_bytes=128ull<<20;gfy_live::set_memory_policy(policy);
    arm_cushion();xr_set_oom_release(&release_cushion);xr_alloc_oom_hook=&release_cushion_for_lua;
    const auto m=system_memory();const u64 phys=m.valid?m.total_phys:(16ull<<30);
    // GOYU: BloodEngine allowed phys/16, at most 2 GB. With GFY's many outfits a level's NPCs held 2.3-3.4 GB of body
    // data, so most of them waited for a bind and only showed holes and blood. Default phys/8 up to 4 GB;
    // painter_asset_cap_mb (read here, before the first body is bound) sets it by hand.
    asset_cap_bytes=g_goyu_painter_asset_cap_mb>0?u64(g_goyu_painter_asset_cap_mb)<<20:
        std::max<u64>(512ull<<20,std::min<u64>(4096ull<<20,phys/8));
    xr_pool_stats lua_pool{};XR_POOL_STATS(&lua_pool);
    Msg("[painter_memory] phase=policy heavy_join_mb=%llu commit_margin_mb=%llu dispatch_floor_mb=%llu asset_cap_mb=%llu total_phys_mb=%llu avail_commit_mb=%llu lua_pool_floor_mb=%llu lua_pool_committed_mb=%llu lua_pool_used_mb=%llu",
        policy.heavy_bytes>>20,policy.commit_margin>>20,policy.dispatch_commit_floor>>20,asset_cap_bytes>>20,phys>>20,m.valid?m.avail_commit>>20:0ull,
        u64(lua_pool.floor_bytes>>20),u64(lua_pool.committed_bytes>>20),u64(lua_pool.used_bytes>>20));
    const auto c=commit_state();
    Msg("[painter_memory] phase=commit_state commit_total_mb=%llu commit_limit_mb=%llu avail_phys_mb=%llu",c.total_mb,c.limit_mb,m.valid?m.avail_phys>>20:0ull);
    log_pagefile_config();
}
// Clears the engine callbacks before this module unloads.
struct ReleaseGuard {~ReleaseGuard(){xr_alloc_oom_hook=nullptr;xr_set_oom_release(nullptr);release_cushion(0);}} release_guard;
u64 resident_asset_bytes(u64* largest=nullptr){u64 bytes=0,top=0;
    for(const auto& item:model_assets)if(item.second.asset){const u64 b=gfy_live::asset_resident_bytes(item.second.asset);bytes+=b;top=std::max(top,b);}
    if(largest)*largest=top;return bytes;}
// Drops the least recently used prepared assets that no owner references
// (they are prepared again on the next bind) until `target` bytes are freed.
u64 evict_unreferenced_assets(u64 target,unsigned& count){u64 freed=0;count=0;
    while(freed<target){auto oldest=model_assets.end();
        for(auto it=model_assets.begin();it!=model_assets.end();++it)
            if(!it->second.preparing&&it->second.asset&&it->second.asset.use_count()==1&&(oldest==model_assets.end()||it->second.access<oldest->second.access))oldest=it;
        if(oldest==model_assets.end())break;freed+=gfy_live::asset_resident_bytes(oldest->second.asset);++count;model_assets.erase(oldest);}
    return freed;}
// The body-data budget (asset_cap_bytes) holds for body data no bound NPC uses: above it the least recently used goes at
// once. Body data a bound NPC uses stays (its wounds need it) until that NPC is released (UnbindPainterVoxelLive).
// Sizes are only summed when some body data is unused (no lock taken otherwise; the sum reads each asset's seed lock).
u64 trim_assets(unsigned& count){count=0;bool unused=false;
    for(const auto& item:model_assets)if(!item.second.preparing&&item.second.asset&&item.second.asset.use_count()==1){unused=true;break;}
    if(!unused)return 0;const u64 bytes=resident_asset_bytes();
    return bytes>asset_cap_bytes?evict_unreferenced_assets(bytes-asset_cap_bytes,count):0;}
// Fire-wound hits that reach an NPC while its bind is in progress (the player's long shot before the outfit's body data
// is ready): built at the hit with that frame's pose and replayed when the bind completes. At most 16 events, kept 30 s
// for the player's target (its body data goes first, after the one being prepared) and 10 s for other binds.
struct HeldHits {std::vector<gfy_live::Impact> events;u32 since=0;};
std::map<u16,HeldHits> held_hits;
void drop_held_hits(u16 id,const char* why){auto it=held_hits.find(id);if(it==held_hits.end())return;
    body_hit_outcomes::batch("discarded",why,0,it->second.events);
    Msg("[painter_voxel_live] id=%u phase=held_hits_dropped events=%u reason=%s",id,unsigned(it->second.events.size()),why);held_hits.erase(it);}
u32 budget_logged_at=0;
// Owner thread. Below the low-water marks, release recomputable memory: the
// caches of owners idle for 3 s (dead and idle NPCs first, oldest first),
// then unreferenced prepared assets. Committed wounds are never touched.
bool memory_pressure=false;u32 memory_checked_at=0,memory_logged_at=0,emergency_logged=0;
void govern(bool force=false){
    if(!pool)return;const u32 now=Device.dwTimeGlobal;
    if(!force&&memory_checked_at&&now-memory_checked_at<250)return;memory_checked_at=now;
    {unsigned trimmed=0;const u64 freed=trim_assets(trimmed);
     if(trimmed)Msg("[painter_memory] phase=asset_budget evicted_assets=%u evicted_asset_mb=%.1f resident_asset_mb=%.1f budget_mb=%llu",
        trimmed,freed/1048576.0,resident_asset_bytes()/1048576.0,asset_cap_bytes>>20);}
    const u32 releases=emergency_releases.load(std::memory_order_relaxed);
    if(releases!=emergency_logged){emergency_logged=releases;
        Msg("[painter_memory] phase=emergency_release cushion_mb=%llu releases=%u",u64(cushion_bytes>>20),releases);}
    const auto m=system_memory();if(!m.valid)return;
    const u64 low_commit=1536ull<<20,low_phys=1024ull<<20;
    // An owner with a batch deferred for memory (never dropped, see LiveDamageWorker.h MemoryPolicy) starts the
    // release ladder regardless of the low-water marks: idle caches after 3 s, then unreferenced assets.
    const bool starving=pool->any_memory_waiting();
    if(!starving&&m.avail_commit>=low_commit&&m.avail_phys>=low_phys){if(memory_pressure)Msg("[painter_memory] phase=recovered avail_commit_mb=%llu avail_phys_mb=%llu",m.avail_commit>>20,m.avail_phys>>20);memory_pressure=false;
        if(!cushion.load()&&m.avail_commit>=low_commit+cushion_bytes&&arm_cushion())Msg("[painter_memory] phase=cushion_armed cushion_mb=%llu",u64(cushion_bytes>>20));return;}
    const u64 want=std::max<u64>(m.avail_commit<low_commit?low_commit-m.avail_commit:0,m.avail_phys<low_phys?low_phys-m.avail_phys:0)+(256ull<<20)+(starving?(512ull<<20):0ull);
    // Soft pressure is the game's normal state (in-game 2026-09-25: 290-1300 MB free commit all session, below the 1536 MB
    // mark), and evicting an owner's wound caches after 3 s idle made its next hit re-join the whole body (875 ms). Keep the
    // caches unless the owner has been idle a minute; a hard shortfall evicts after 3 s as before.
    const bool hard=starving||m.avail_commit<(384ull<<20)||m.avail_phys<(512ull<<20);
    const auto owners=pool->evict_idle_caches(size_t(want),hard?3000.0:60000.0);unsigned assets=0;u64 asset_bytes=0;
    const u64 sep_cache_bytes=hard?u64(gfy_live::clear_separation_region_cache()):0ull; // per-region join cache (separation) under a hard shortfall
    if(owners.bytes<want)asset_bytes=evict_unreferenced_assets(want-owners.bytes,assets);
    if(!memory_pressure||now-memory_logged_at>=5000||owners.sessions||assets||starving){memory_logged_at=now;
        const auto c=gfy_live::memory_counters();const auto cs=commit_state();
        if(starving)Msg("[painter_memory] phase=starving commit_total_mb=%llu commit_limit_mb=%llu avail_commit_mb=%llu hard_evict=1 sep_cache_released_mb=%.1f",cs.total_mb,cs.limit_mb,m.avail_commit>>20,sep_cache_bytes/1048576.0);
        Msg("[painter_memory] phase=pressure avail_commit_mb=%llu avail_phys_mb=%llu evicted_owners=%u evicted_owner_mb=%.1f evicted_assets=%u evicted_asset_mb=%.1f resident_asset_mb=%.1f oom_failed=%llu low_memory_failed=%llu deferred=%llu emergency_releases=%u",
            m.avail_commit>>20,m.avail_phys>>20,unsigned(owners.sessions),owners.bytes/1048576.0,assets,asset_bytes/1048576.0,resident_asset_bytes()/1048576.0,
            c.oom_failed,c.admission_failed,c.oom_deferred+c.admission_deferred,emergency_releases.load(std::memory_order_relaxed));}
    memory_pressure=true;
}
// A revision's main-thread publication holds its CPU request, renderer child
// buffers, CPU skin backup and face lists at once. ESTIMATED upper bound: 384 B
// per vertex and 12 B per index (MEASURED at most ~205 B/vertex offline, see
// build2-fixes/oom-safety REPORT). Never start one without the commit for it.
u64 publication_bytes(const gfy_packed::Mesh& mesh){u64 v=0,i=0;for(const auto& c:mesh.children){v+=c.vertices.size();i+=c.indices.size();}return v*384+i*12+(16ull<<20);}
bool publication_admitted(u64 need,gfy_live::MemoryStatus& seen){
    seen=system_memory();if(!seen.valid||seen.avail_commit>=need+(256ull<<20))return true;
    govern();seen=system_memory();return !seen.valid||seen.avail_commit>=need+(256ull<<20);
}
void state(int result,int owner,int revision){painter_voxel_live_result=result;painter_voxel_live_owner=owner;painter_voxel_live_revision=revision;}
bool separation_allowed(CAI_Stalker& source){
    if(source.g_Alive())return false;
    if(gfy_live::damage_mode_value()==1)return false; // modes: Bullet holes only - no body ever separates
    ::luabind::functor<bool> policy;
    return ai().script_engine().functor("_G.CPainterVoxelSeparationAllowed",policy)&&
        policy(source.lua_game_object());
}
gfy_live::Hash hash(const void* bytes,size_t count){gfy_live::Hash out{};if(!SHA256(static_cast<const unsigned char*>(bytes),count,out.data()))return {};return out;}
gfy_live::Hash hex(LPCSTR value){gfy_live::Hash out{};for(size_t i=0;i<32;++i){unsigned x=0;if(sscanf(value+i*2,"%2x",&x)!=1)return {};out[i]=u8(x);}return out;}
bool bytes(LPCSTR name,LPCSTR expected,std::vector<std::uint8_t>& out,size_t limit=32u*1024u*1024u){IReader* reader=FS.r_open("$game_meshes$",name);if(!reader)return false;bool ok=reader->length()>0&&reader->length()<=limit;
    if(ok){out.resize(reader->length());memcpy(out.data(),reader->pointer(),reader->length());}FS.r_close(reader);return ok&&hash(out.data(),out.size())==hex(expected);}
bool digest(LPCSTR text,gfy_live::Hash& output){
    if(!text||xr_strlen(text)!=64)return false;
    for(unsigned i=0;i<64;++i)if(!((text[i]>='0'&&text[i]<='9')||
       (text[i]>='a'&&text[i]<='f')))return false;
    output=hex(text);return true;
}
void separation_descriptor(CInifile& catalog,const std::string& section,
    PreparedQuality& staged){
    using namespace gfy_live;
    auto fail=[&](const char* why){staged.separation_ready=false;
        staged.separation_reason=why;};
    auto file=[&](LPCSTR key,AssetFile& target,size_t limit,bool optional){
        const std::string checksum=std::string(key)+"_sha256";
        const bool path=catalog.line_exist(section.c_str(),key);
        const bool sha=catalog.line_exist(section.c_str(),checksum.c_str());
        if(optional&&!path&&!sha)return true;
        if(!path||!sha)return false;
        if(!digest(catalog.r_string(section.c_str(),checksum.c_str()),target.expected))return false;
        string_path resolved;
        if(!FS.exist(resolved,"$game_meshes$",catalog.r_string(section.c_str(),key)))return false;
        const DWORD attributes=GetFileAttributesA(resolved);
        if(attributes==INVALID_FILE_ATTRIBUTES||(attributes&FILE_ATTRIBUTE_DIRECTORY))return false;
        target.path=resolved;target.max_bytes=limit;return true;
    };
    auto& input=staged.separation_input;input.hash=hash;
    if(!file("surface_bindings",input.surface_bindings,32u*1024u*1024u,false)||
       !file("coat_roots",input.coat_roots,4u*1024u*1024u,true)||
       !file("physics_elements",input.physics_elements,128u*1024u*1024u,false)||
       !file("accessory_surfaces",input.accessory_surfaces,128u*1024u*1024u,true)||
       !file("bone_frames",input.bone_frames,16u*1024u*1024u,false)){
        fail("separation_file_descriptor_or_provider");return;
    }
    if(!catalog.line_exist(section.c_str(),"anatomy_hash")||
       !catalog.line_exist(section.c_str(),"fit_hash")||
       !digest(catalog.r_string(section.c_str(),"anatomy_hash"),input.anatomy_hash)||
       !digest(catalog.r_string(section.c_str(),"fit_hash"),input.fit_hash)){
        fail("separation_identity_descriptor");return;
    }
    AssetFile accessories;
    if(!file("accessories",accessories,1024u*1024u,false)){
        fail("separation_accessories_descriptor");return;
    }
    std::vector<u8> raw;
    if(!bytes(catalog.r_string(section.c_str(),"accessories"),
       catalog.r_string(section.c_str(),"accessories_sha256"),raw,1024u*1024u)||
       raw.size()<44||memcmp(raw.data(),"VXGEAR01",8)||
       memcmp(raw.data()+8,staged.input.source_hash.data(),32)){
        fail("separation_accessories_identity");return;
    }
    u32 count=0;memcpy(&count,raw.data()+40,4);
    if(count>staged.input.original_child_count||raw.size()!=44ull+28ull*count){
        fail("separation_accessories_size");return;
    }
    std::set<u16> seen;
    for(u32 i=0;i<count;++i){
        const u8* record=raw.data()+44+28*i;
        SeparationAccessory accessory;
        memcpy(&accessory.child,record,2);
        memcpy(&accessory.physical_element,record+2,2);
        memcpy(&accessory.original_anchor.x,record+4,8);
        memcpy(&accessory.original_anchor.y,record+12,8);
        memcpy(&accessory.original_anchor.z,record+20,8);
        if(accessory.child>=staged.input.original_child_count||
           accessory.physical_element>=64||!seen.insert(accessory.child).second||
           !std::isfinite(accessory.original_anchor.x)||
           !std::isfinite(accessory.original_anchor.y)||
           !std::isfinite(accessory.original_anchor.z)){
            fail("separation_accessories_record");return;
        }
        input.accessories.push_back(accessory);
    }
    staged.separation_ready=true;staged.separation_reason.clear();
}
gfy_live::Hash rig_hash(const xr_vector<RuntimeSkinBone>& rig){std::vector<u8>raw;for(const auto&b:rig){auto append=[&](const void*p,size_t n){auto*x=static_cast<const u8*>(p);raw.insert(raw.end(),x,x+n);};append(b.name.c_str(),b.name.size()+1);append(&b.parent,sizeof(b.parent));append(&b.model_to_bone,sizeof(b.model_to_bone));}return hash(raw.data(),raw.size());}
gfy_live::Matrix matrix(const Fmatrix&m){gfy_live::Matrix out;out.row_major={m._11,m._21,m._31,m._41,m._12,m._22,m._32,m._42,m._13,m._23,m._33,m._43,m._14,m._24,m._34,m._44};return out;}
bool same_rig(IKinematics*k,const xr_vector<RuntimeSkinBone>&rig){if(!k||rig.size()!=k->LL_BoneCount())return false;for(u16 i=0;i<k->LL_BoneCount();++i)if(rig[i].name!=k->LL_BoneName_dbg(i)||rig[i].parent!=k->LL_GetData(i).GetParentID()||memcmp(&rig[i].model_to_bone,&k->LL_GetData(i).m2b_transform,sizeof(Fmatrix)))return false;return true;}
bool original_physical_parents(IKinematics* k,CPhysicsShell* shell,
    std::vector<u16>& parents){
    if(!k||!shell||k->LL_BoneCount()>64)return false;
    std::vector<bool> element(k->LL_BoneCount(),false);
    for(u16 b=0;b<k->LL_BoneCount();++b){
        auto* e=shell->get_Element(b);element[b]=e&&e->m_SelfID==b;
    }
    if(!shell->get_ElementsNumber()||
       shell->get_ElementByStoreOrder(0)->m_SelfID>=k->LL_BoneCount()||
       !element[shell->get_ElementByStoreOrder(0)->m_SelfID])return false;
    parents.assign(k->LL_BoneCount(),BI_NONE);
    for(u16 b=0;b<k->LL_BoneCount();++b){
        u16 p=k->LL_GetData(b).GetParentID();
        unsigned steps=0;
        while(p!=BI_NONE&&p<k->LL_BoneCount()&&steps++<k->LL_BoneCount()){
            if(element[p]){parents[b]=p;break;}
            p=k->LL_GetData(p).GetParentID();
        }
        if(p!=BI_NONE&&steps>=k->LL_BoneCount())return false;
    }
    return true;
}
bool joint_profile(CScriptGameObject* npc,IKinematics*k,gfy_live::AssetInput&input){
    ::luabind::functor<::luabind::object> profile;
    if(!ai().script_engine().functor("_G.CPainterVoxelJointProfile",profile))return false;
    const ::luabind::object rows=profile(npc);if(!rows||rows.type()!=LUA_TTABLE)return false;
    std::vector<gfy_live::JointRegion> regions;
    for(unsigned i=1;i<=65;++i){const ::luabind::object row=rows[i];if(!row||row.type()==LUA_TNIL)break;
        if(i==65||row.type()!=LUA_TTABLE)return false;
        const ::luabind::object bone=row["bone"],end=row["axis_bone"],radius=row["radius_m"],width=row["half_width_m"];
        if(!bone||bone.type()!=LUA_TSTRING||!end||end.type()!=LUA_TSTRING||!radius||radius.type()!=LUA_TNUMBER||!width||width.type()!=LUA_TNUMBER)return false;
        const u16 b=k->LL_BoneID(::luabind::object_cast<LPCSTR>(bone)),e=k->LL_BoneID(::luabind::object_cast<LPCSTR>(end));
        if(b==BI_NONE||e==BI_NONE||b==e)return false;
        Fmatrix pivot,tip;pivot.invert(k->LL_GetData(b).m2b_transform);tip.invert(k->LL_GetData(e).m2b_transform);
        gfy_live::JointRegion region;region.bone=b;region.center={pivot.c.x,pivot.c.y,pivot.c.z};
        region.axis={tip.c.x-pivot.c.x,tip.c.y-pivot.c.y,tip.c.z-pivot.c.z};
        region.radius=::luabind::object_cast<double>(radius);region.half_width=::luabind::object_cast<double>(width);
        regions.push_back(region);
    }
    input.joints=std::move(regions);return true;
}
PreparedQuality* fullbody_model(LPCSTR visual,std::string&error,bool target=false){
    static std::unique_ptr<CInifile> catalog;
    if(!catalog){string_path path;FS.update_path(path,"$game_config$","painter_fullbody_profiles.ltx");if(!FS.exist(path))return nullptr;
        catalog=std::make_unique<CInifile>(path,TRUE,TRUE,FALSE);}
    std::string name=visual?visual:"";for(auto&c:name){if(c=='/')c='\\';else if(c>='A'&&c<='Z')c=char(c-'A'+'a');}
    if(name.size()>4&&name.substr(name.size()-4)==".ogf")name.resize(name.size()-4);
    if(!catalog->line_exist("models",name.c_str()))return nullptr;
    const std::string digest=catalog->r_string("models",name.c_str());
    if(digest.size()!=64||digest.find_first_not_of("0123456789abcdef")!=std::string::npos){error="fullbody_catalog_digest";return nullptr;}
    const int quality=painter_voxel_effective_quality();
    // The model's data set is the one of quality 1 (the 1 mm anatomy; the set every round was tested with, and three old
    // override files change _q1 only). The wound cell size comes from the selected quality alone (input.damage_pitch below),
    // and prepared assets are kept per cell size: an asset of one size is never handed out for the other.
    const std::string section=digest+"_q1";
    const std::string asset_key=section+"@"+std::to_string(quality);
    if(!catalog->section_exist(section.c_str())){error="fullbody_quality_not_installed";return nullptr;}
    auto found=model_assets.find(asset_key);if(found!=model_assets.end()){found->second.access=++model_access;error=found->second.preparation_error;return &found->second;}
    // Match the owner pool: ten different Stalker outfits must not fail just
    // because the earlier prototype had only eight shared asset slots.
    // Resident prepared assets are capped by count (32) and by bytes
    // (asset_cap_bytes): least recently used entries no owner references go
    // first; body data still being prepared counts at the incoming size. A new
    // model waits (busy retry) while the count is full. While the byte budget
    // is full of body data bound NPCs use, a range bind waits unbound
    // (fullbody_asset_budget, retried by Lua) and the player's target (aimed
    // at or shot, any distance) goes past it unless system memory is low.
    {u64 largest=0,bytes=resident_asset_bytes(&largest);const u64 incoming=std::max<u64>(largest,128ull<<20);
     for(const auto& item:model_assets)if(item.second.preparing)bytes+=incoming;
     while(model_assets.size()>=32||bytes+incoming>asset_cap_bytes){auto oldest=model_assets.end();for(auto it=model_assets.begin();it!=model_assets.end();++it)
        if(!it->second.preparing&&(!it->second.asset||it->second.asset.use_count()==1)&&(oldest==model_assets.end()||it->second.access<oldest->second.access))oldest=it;
        if(oldest==model_assets.end())break;bytes-=std::min<u64>(bytes,gfy_live::asset_resident_bytes(oldest->second.asset));model_assets.erase(oldest);}
     if(model_assets.size()>=32){error="fullbody_resident_asset_capacity";return nullptr;}
     if(bytes+incoming>asset_cap_bytes){if(!target){error="fullbody_asset_budget";return nullptr;}
        const auto m=system_memory();if(m.valid&&m.avail_commit<incoming+(384ull<<20)){error="fullbody_resident_asset_capacity";return nullptr;}}}
    const std::string original=name+".ogf";
    PreparedQuality staged;staged.fullbody=true;staged.access=++model_access;staged.input.source_hash=hex(digest.c_str());
    staged.input.damage_pitch=profile_quality_pitch[quality];
    staged.tissue_textures.resize(64);
    for(unsigned i=1;i<64;++i){const std::string key=std::to_string(i);if(!catalog->line_exist("materials",key.c_str()))continue;
        staged.tissue_textures[i]=catalog->r_string("materials",key.c_str());
        const std::string path=std::string(staged.tissue_textures[i].c_str())+".dds";
        if(!FS.exist("$game_textures$",path.c_str())){error="fullbody_tissue_texture_missing";return nullptr;}}
    if(!catalog->line_exist(section.c_str(),"children")){error="fullbody_missing_child_count";return nullptr;}
    const u32 children=catalog->r_u32(section.c_str(),"children");if(!children||children>=gfy_packed::max_child_slots){error="fullbody_child_count";return nullptr;}
    staged.input.original_child_count=u16(children);staged.input.source_child=0;
    const char* keys[]={"source_geometry","closed_envelope","base_occupancy","source_shading","occlusion_geometry","source_children","material_slots"};
    std::vector<std::uint8_t>* arrays[]={&staged.input.source_geometry,&staged.input.closed_envelope,&staged.input.base_occupancy,&staged.input.source_shading,&staged.input.occlusion_geometry,&staged.input.source_children,&staged.input.material_slots,&staged.input.accessory_surfaces,&staged.input.render_field};
    std::array<std::string,10> relative;relative[0]=original;
    staged.files[0].expected=hex(digest.c_str());staged.files_on_disk=true;
    for(unsigned i=0;i<7;++i){const std::string checksum=std::string(keys[i])+"_sha256";
        if(!catalog->line_exist(section.c_str(),keys[i])||!catalog->line_exist(section.c_str(),checksum.c_str())){error="fullbody_profile_descriptor";return nullptr;}
        relative[i+1]=catalog->r_string(section.c_str(),keys[i]);staged.files[i+1].expected=hex(catalog->r_string(section.c_str(),checksum.c_str()));
        staged.files[i+1].max_bytes=(i==2?64u:32u)*1024u*1024u;
    }
    if(catalog->line_exist(section.c_str(),"accessory_surfaces")||catalog->line_exist(section.c_str(),"accessory_surfaces_sha256")){
        if(!catalog->line_exist(section.c_str(),"accessory_surfaces")||
           !catalog->line_exist(section.c_str(),"accessory_surfaces_sha256")||
           !painter_voxel_native::digest(catalog->r_string(section.c_str(),"accessory_surfaces_sha256"),staged.files[8].expected)){
            error="fullbody_ocular_source_descriptor";return nullptr;}
        relative[8]=catalog->r_string(section.c_str(),"accessory_surfaces");
        if(relative[8].empty()){error="fullbody_ocular_source_descriptor";return nullptr;}
        staged.files[8].max_bytes=64u*1024u*1024u;
    }
    // R57: optional offline-baked render-only cut field (replaces the closure build at preparation).
    if(catalog->line_exist(section.c_str(),"render_field")||catalog->line_exist(section.c_str(),"render_field_sha256")){
        if(!catalog->line_exist(section.c_str(),"render_field")||
           !catalog->line_exist(section.c_str(),"render_field_sha256")||
           !painter_voxel_native::digest(catalog->r_string(section.c_str(),"render_field_sha256"),staged.files[9].expected)){
            error="fullbody_render_field_descriptor";return nullptr;}
        relative[9]=catalog->r_string(section.c_str(),"render_field");
        if(relative[9].empty()){error="fullbody_render_field_descriptor";return nullptr;}
        staged.files[9].max_bytes=64u*1024u*1024u;
    }
    for(unsigned i=0;i<relative.size();++i){if((i==8||i==9)&&relative[i].empty())continue;string_path resolved;
        if(!FS.exist(resolved,"$game_meshes$",relative[i].c_str())){error="fullbody_profile_file_missing";return nullptr;}
        staged.files[i].path=resolved;
        const DWORD attributes=GetFileAttributesA(resolved);
        if(attributes==INVALID_FILE_ATTRIBUTES||(attributes&FILE_ATTRIBUTE_DIRECTORY))staged.files_on_disk=false;
    }
    if(!staged.files_on_disk){
        // Archived providers retain the VFS path. Never substitute a different
        // provider to make background loading succeed. Loose release assets
        // are read, hashed and decoded entirely on the preparation worker.
        for(unsigned i=0;i<relative.size();++i){if((i==8||i==9)&&relative[i].empty())continue;std::vector<u8> original_bytes;auto&out=i?*arrays[i-1]:original_bytes;
            IReader*reader=FS.r_open("$game_meshes$",relative[i].c_str());if(!reader){error="fullbody_profile_file_missing";return nullptr;}
            bool valid=reader->length()>0&&reader->length()<=staged.files[i].max_bytes;
            if(valid){out.resize(reader->length());memcpy(out.data(),reader->pointer(),reader->length());}FS.r_close(reader);
            if(!valid||hash(out.data(),out.size())!=staged.files[i].expected){error="fullbody_profile_hash_or_size";return nullptr;}}
    }
    separation_descriptor(*catalog,section,staged);
    staged.warm=true;return &model_assets.emplace(asset_key,std::move(staged)).first->second;
}
}
bool painter_voxel_live_warm()
{
    using namespace painter_voxel_native;
    auto reject=[&](const char* why){state(-1,65535,0);Msg("[painter_voxel_live] phase=rejected reason=%s",why);return false;};
        auto& selected=selected_quality();
        if(selected.warm){state(4,65535,0);return true;}
        const unsigned quality=unsigned(painter_voxel_effective_quality());
        std::vector<u8>original;string_path source;xr_strconcat(source,profile_source_alias,".ogf");
        if(!bytes(source,profile_source_sha,original))return reject("original_GFY_source_hash");
        gfy_live::AssetInput staged;staged.source_hash=hex(profile_source_sha);staged.source_child=profile_source_child;staged.original_child_count=profile_children;
        std::vector<std::uint8_t>* arrays[]={&staged.source_geometry,&staged.closed_envelope,&staged.base_occupancy,&staged.source_shading,&staged.occlusion_geometry};
        for(unsigned i=0;i<5;++i)if(!bytes(i==2?profile_quality_files[quality]:profile_files[i],i==2?profile_quality_hashes[quality]:profile_hashes[i],*arrays[i]))return reject("profile_hash_or_size");
        install_memory_policy();if(!pool)pool=gfy_live::make_pool(4,gfy_live::pool_max_owners);if(!pool)return reject(gfy_live::last_failure());selected.input=std::move(staged);selected.warm=true;state(4,65535,0);
        Msg("[painter_voxel_live] phase=warm source=%s profile_files=5 workers=2 max_owners=32 quality=%u pitch_mm=%.3f per_hit_file_reads=0",profile_source_sha,quality,profile_quality_pitch[quality]*1000);return true;

}
bool CCharacterPhysicsSupport::BindPainterVoxelLive(bool target)
{
    using namespace painter_voxel_native;
    // The player's target (aimed at or shot, any distance). A bind already in progress continues on its schedule.
    if(target&&!m_painter_voxel_live){m_painter_voxel_bind_target=true;if(m_painter_voxel_bind_requested)return true;}
    m_painter_voxel_bind_failed=false;
    auto reject=[&](const char* why){m_painter_voxel_bind_requested=false;m_painter_voxel_bind_failed=true;m_painter_voxel_bind_target=false;drop_held_hits(m_EntityAlife.ID(),"bind_rejected");
        state(-1,m_EntityAlife.ID(),0);Msg("[painter_voxel_live] id=%u phase=rejected reason=%s",m_EntityAlife.ID(),why);return false;};
    auto busy=[&](){m_painter_voxel_bind_requested=true;m_painter_voxel_bind_retry_at=Device.dwTimeContinual+250;state(5,m_EntityAlife.ID(),0);return true;};
    // The body-data budget is full of outfits bound NPCs use: this range bind waits unbound (not failed) and Lua retries;
    // until then its hits get no physical wound, as beyond the bind range.
    auto budget_wait=[&](){m_painter_voxel_bind_requested=false;drop_held_hits(m_EntityAlife.ID(),"asset_budget");
        if(!budget_logged_at||Device.dwTimeGlobal-budget_logged_at>=5000){budget_logged_at=Device.dwTimeGlobal;
            Msg("[painter_memory] id=%u phase=bind_waiting reason=asset_budget resident_asset_mb=%.1f budget_mb=%llu",m_EntityAlife.ID(),resident_asset_bytes()/1048576.0,asset_cap_bytes>>20);}
        return true;};
    // Out of memory while binding is not a property of this NPC: retry later.
    auto memory_busy=[&](const char* stage){m_painter_voxel_bind_requested=true;m_painter_voxel_bind_retry_at=Device.dwTimeContinual+2000;state(5,m_EntityAlife.ID(),0);
        Msg("[painter_voxel_live] id=%u phase=bind_deferred stage=%s reason=out_of_memory",m_EntityAlife.ID(),stage);govern(true);return true;};
    auto out_of_memory=[](const char* why){return why&&!xr_strcmp(why,"out_of_memory");};
    install_memory_policy();
        IKinematics*k=smart_cast<IKinematics*>(m_EntityAlife.Visual());
        if(m_painter_voxel_live){auto&owner=*m_painter_voxel_live;
            if(!m_EntityAlife.getDestroy()&&k==owner.visual&&k->LL_RenderGeometryGeneration()==owner.geometry&&same_rig(k,owner.exact_rig))return true;
            return reject("bind_existing_owner_changed");}
        if(m_eType!=etStalker||m_EntityAlife.getDestroy()||
            (m_EntityAlife.g_Alive()&&m_pPhysicsShell!=nullptr)||!k||k->LL_RenderGeometryGeneration()!=0||m_painter_geometry_installed||m_painter_geometry_pending>=0){
            // R65: which part of the combined predicate refused (the R64 wrist control logged only the reason)
            Msg("[painter_voxel_live] id=%u phase=bind_owner_detail stalker=%d destroy=%d alive_with_shell=%d visual=%d generation=%llu installed=%d pending=%d",
                m_EntityAlife.ID(),int(m_eType==etStalker),int(m_EntityAlife.getDestroy()?1:0),int(m_EntityAlife.g_Alive()&&m_pPhysicsShell!=nullptr),int(k!=nullptr),
                k?static_cast<unsigned long long>(k->LL_RenderGeometryGeneration()):0ull,int(m_painter_geometry_installed?1:0),int(m_painter_geometry_pending));
            return reject("bind_owner_source_or_state");}
        // Death-shell activation is transient. No settled-pose wait is needed.
        // Attached weapon collision shapes do not alter render-mesh ownership.
        if(!m_EntityAlife.g_Alive()&&(!m_pPhysicsShell||!m_pPhysicsShell->isFullActive()))return busy();
        std::string model_error;auto* model=fullbody_model(*m_EntityAlife.cNameVisual(),model_error,m_painter_voxel_bind_target);
        if(model_error=="fullbody_resident_asset_capacity")return busy();
        if(model_error=="fullbody_asset_budget")return budget_wait();
        if(!model_error.empty())return reject(model_error.c_str());
        if(!model){if(xr_strcmp(*m_EntityAlife.cNameVisual(),profile_source_alias))return reject("model_not_in_fullbody_catalog");
            if(!selected_quality().warm&&!painter_voxel_live_warm())return false;model=&selected_quality();}
        auto& selected=*model;auto& input=selected.input;auto& asset=selected.asset;auto& prepared_rig=selected.prepared_rig;
        if(!pool)pool=gfy_live::make_pool(4,gfy_live::pool_max_owners);if(!pool)return reject(gfy_live::last_failure());
        auto owner=std::make_shared<PainterVoxelLiveOwner>();owner->visual=k;owner->shell=m_pPhysicsShell;owner->thread=Device.m_geometry_publish_thread?Device.m_geometry_publish_thread:GetCurrentThreadId();owner->geometry=k->LL_RenderGeometryGeneration();owner->exact_rig.resize(k->LL_BoneCount());
        owner->tissue_textures=selected.tissue_textures;
        owner->original_children=input.original_child_count;
        owner->separation_ready=selected.separation_ready;
        if(owner->separation_ready)owner->separation_input=selected.separation_input;
        // Prewarm the separation caches (pristine topology, physics map) on the low-priority warm thread at bind, so the
        // first sever of this model does not pay them cold (~150-220 ms measured on the thigh fixture).
        if(owner->separation_ready&&asset)(void)gfy_live::warm_separation_async(asset,owner->separation_input);
        for(u16 i=0;i<k->LL_BoneCount();++i){auto&b=owner->exact_rig[i];b.name=k->LL_BoneName_dbg(i);b.parent=k->LL_GetData(i).GetParentID();b.model_to_bone=k->LL_GetData(i).m2b_transform;}
        if(owner->lifecycle.observe(m_EntityAlife.g_Alive(),reinterpret_cast<std::uintptr_t>(m_pPhysicsShell))==painter_voxel_binding::Phase::rejected)return reject("bind_lifecycle");
        const auto rig=rig_hash(owner->exact_rig);if(asset&&prepared_rig!=rig)return reject("prepared_rig_mismatch");
        if(selected.fullbody&&!asset){
            if(selected.preparing){if(prepared_rig!=rig)return reject("preparing_rig_mismatch");
                if(m_painter_voxel_bind_target)selected.preparing->promote(); // queued for a range bind: the target's now
                std::string error;if(!selected.preparing->poll(asset,error)){state(5,m_EntityAlife.ID(),0);m_painter_voxel_bind_requested=true;return true;}
                selected.preparing.reset();if(!asset){if(out_of_memory(error.c_str()))return memory_busy("asset_preparation");selected.preparation_error=error;return reject(error.c_str());}}
            else{if(!joint_profile(m_EntityAlife.lua_game_object(),k,input))return reject("invalid_or_missing_script_joint_profile");
                input.analytic_channels=true;input.allow_renderer_rebuild=true;input.rig_hash=rig;prepared_rig=rig;
                // The player's target is prepared before the range binds waiting in the queue.
                selected.preparing=selected.files_on_disk?gfy_live::prepare_asset_files_async(input,selected.files,hash,m_painter_voxel_bind_target):gfy_live::prepare_asset_async(input,{},m_painter_voxel_bind_target);
                if(!selected.preparing){if(!xr_strcmp(gfy_live::last_failure(),"asset_preparation_queue_full")){state(5,m_EntityAlife.ID(),0);m_painter_voxel_bind_requested=true;return true;}
                    if(out_of_memory(gfy_live::last_failure()))return memory_busy("asset_submission");return reject(gfy_live::last_failure());}state(5,m_EntityAlife.ID(),0);m_painter_voxel_bind_requested=true;return true;}
        }
        if(!asset){input.analytic_channels=true;input.allow_renderer_rebuild=true;input.rig_hash=rig;asset=gfy_live::prepare_asset(input);if(!asset){if(out_of_memory(gfy_live::last_failure()))return memory_busy("asset_preparation");return reject(gfy_live::last_failure());}prepared_rig=rig;}
        // The model's separation caches, now that the asset exists (the request above only ran when it already did).
        if(owner->separation_ready&&asset)(void)gfy_live::warm_separation_async(asset,owner->separation_input);
        {
            // Parsed immutable data belongs to the asset. Do not retain duplicate raw blobs.
            std::vector<std::uint8_t>* arrays[]={&input.source_geometry,&input.closed_envelope,&input.base_occupancy,&input.source_shading,&input.occlusion_geometry,&input.source_children,&input.material_slots,&input.accessory_surfaces,&input.render_field};
            for(auto* a:arrays)std::vector<std::uint8_t>().swap(*a);
        }
        owner->identity.spawn_cookie=next_cookie++;owner->identity.visual_lineage=owner->identity.spawn_cookie;owner->identity.topology=owner->geometry;owner->identity.source=input.source_hash;owner->identity.exact_rig=rig;
        {static std::set<std::string> gear_body_logged;const std::string note=gfy_live::gear_body_note(asset); // gearbody: one line per prepared outfit
         if(!note.empty()&&gear_body_logged.insert(std::string(*m_EntityAlife.cNameVisual())+"|"+note).second)
             Msg("[painter_gear_body] model=%s %s",*m_EntityAlife.cNameVisual(),note.c_str());}
        {static std::set<std::string> field_align_logged;const std::string note=gfy_live::field_align_note(asset); // fieldalign: one line per lined-up outfit
         if(!note.empty()&&field_align_logged.insert(std::string(*m_EntityAlife.cNameVisual())).second)
             Msg("[painter_voxel_live] model=%s phase=cut_field_lined_up %s",*m_EntityAlife.cNameVisual(),note.c_str());}
        owner->worker=pool->attach(asset,owner->identity);if(!owner->worker){if(!xr_strcmp(gfy_live::last_failure(),"owner_capacity"))return busy();if(out_of_memory(gfy_live::last_failure()))return memory_busy("attach");return reject(gfy_live::last_failure());}
        {
            // Wake-driven publication: when a job's output is ready the pool thread queues this owner's barrier
            // callback for the next drain of seqParallelBeforRender (frame start, after FrameMove or end of frame)
            // instead of waiting for in_UpdateCL's poll, which stays as the fallback. Same cross-thread push as the
            // particle worker (device.h). No kernel lock is held while the hook runs; the section serializes the
            // push with CancelPainterVoxelLive, which clears `support` under it.
            auto link=std::make_shared<PainterVoxelReadyLink>();link->support=this;owner->ready_link=link;
            owner->worker->set_ready_hook([link]{
                // Pending list under its own section: a pool thread never waits for a whole drain here.
                xrCriticalSectionGuard guard(&Device.seqParallelBeforRenderPendingCS);
                if(!link->support)return;
                auto callback=xr_make_delegate(link->support,&CCharacterPhysicsSupport::ApplyPainterVoxelLive);
                if(std::find(Device.seqParallelBeforRenderPending.begin(),Device.seqParallelBeforRenderPending.end(),callback)==Device.seqParallelBeforRenderPending.end())
                    Device.seqParallelBeforRenderPending.push_back(callback);
            });
        }
        m_painter_voxel_bind_requested=false;m_painter_voxel_live=std::move(owner);painter_burn_host::bind(m_EntityAlife,m_painter_voxel_live->identity,m_painter_voxel_live->worker,false);state(0,m_EntityAlife.ID(),0);
        const bool bound_target=m_painter_voxel_bind_target;m_painter_voxel_bind_target=false;
        if(selected.fullbody&&!selected.separation_ready)
            Msg("[painter_damaged_fragment] id=%u phase=unavailable reason=%s",
                m_EntityAlife.ID(),selected.separation_reason.c_str());
        // Shader handles for this model's wound children: the exterior pairs
        // are recorded from the originals now, the cavity pairs (the same
        // models\model x tissue textures LL_StageRuntimeGeometry receives) are
        // queued for the renderer's per-frame warm-up, so the first hit
        // compiles nothing on the publish barrier. Main thread only (the cache
        // holds shared_str); a bind on a loading thread defers to the next
        // main-thread update (WarmPainterVoxelShaders).
        unsigned shader_warm_pending=0;
        if(Device.m_geometry_publish_thread&&GetCurrentThreadId()==Device.m_geometry_publish_thread)shader_warm_pending=WarmPainterVoxelShaders(k);
        else m_painter_voxel_live->shader_warm_requested=true;
        char replay_source[65];static const char digits[]="0123456789abcdef";
        for(unsigned b=0;b<32;++b){replay_source[b*2]=digits[input.source_hash[b]>>4];replay_source[b*2+1]=digits[input.source_hash[b]&15];}replay_source[64]=0;
        Msg("[painter_voxel_live] id=%u phase=bound cookie=%llu generation=%llu bone_count=%u alive=%u input=actual_fire_wound_SHit pose=accepted_receiver_boundary source=%s damage_pitch_m=%.17g shader_warm_pending=%u target=%u",m_EntityAlife.ID(),m_painter_voxel_live->identity.spawn_cookie,m_painter_voxel_live->geometry,k->LL_BoneCount(),unsigned(m_EntityAlife.g_Alive()),replay_source,input.damage_pitch,shader_warm_pending,unsigned(bound_target));
        // Hits held while this bind was in progress: their events were frozen at the hit (pose, Lua sizing), so they apply
        // now as they would have then.
        {auto held=held_hits.find(m_EntityAlife.ID());
         if(held!=held_hits.end()){auto&o=*m_painter_voxel_live;unsigned replayed=0;const unsigned count=unsigned(held->second.events.size());const u32 waited=Device.dwTimeGlobal-held->second.since;
            for(auto&event:held->second.events){event.sequence=o.sequence+1;const u64 sequence=event.sequence;if(!o.worker->enqueue(std::move(event)))break;
                o.hit_times.emplace_back(sequence,std::chrono::steady_clock::now());if(o.hit_times.size()>256)o.hit_times.pop_front();++o.sequence;++replayed;state(1,m_EntityAlife.ID(),int(o.sequence));}
            body_hit_outcomes::batch("discarded","held_replay_enqueue_failed_suffix",o.identity.spawn_cookie,held->second.events,replayed<held->second.events.size()?replayed+1:replayed);
            held_hits.erase(held);
            Msg("[painter_voxel_live] id=%u phase=held_hits_replayed events=%u of=%u wait_ms=%u",m_EntityAlife.ID(),replayed,count,waited);
            if(replayed&&pool&&k->LL_RenderGeometryGeneration()==o.geometry)pool->dispatch(o.worker,o.geometry);}}
        return true;

}
// Live bodies and severed pieces share one pool (256 owners: a gibbed crowd is a few dozen pieces).
gfy_live::Pool* painter_voxel_live_pool(){
    using namespace painter_voxel_native;install_memory_policy();if(!pool)pool=gfy_live::make_pool(4,gfy_live::pool_max_owners);return pool.get();
}
void CCharacterPhysicsSupport::CancelPainterVoxelLive()
{
    PainterHoldRelease("owner_canceled");
    if(auto* object=m_EntityAlife.cast_stalker())painter_cranial_remnants::forget(*object);
    painter_damaged_fragment::cancel(m_EntityAlife.ID());painter_explosions::cancel_orphan_stash(m_EntityAlife);
    painter_voxel_native::drop_held_hits(m_EntityAlife.ID(),"owner_canceled");
    m_painter_voxel_bind_requested=false;m_painter_voxel_bind_failed=false;m_painter_voxel_bind_retry_at=0;m_painter_voxel_bind_target=false;
    std::shared_ptr<PainterVoxelReadyLink> link;
    if(m_painter_voxel_live){
        if(!m_painter_voxel_live->committed_effects.empty())
            Msg("[painter_voxel_live] id=%u phase=wound_blood_invalidated reason=owner_canceled receipts=%u",m_EntityAlife.ID(),unsigned(m_painter_voxel_live->committed_effects.size()));
        painter_burn_host::forget(m_EntityAlife,m_painter_voxel_live->worker);
        link=m_painter_voxel_live->ready_link;m_painter_voxel_live->worker->cancel();}
    m_painter_voxel_live.reset();m_painter_voxel_queued=false;
    auto callback=xr_make_delegate(this,&CCharacterPhysicsSupport::ApplyPainterVoxelLive);
    xrCriticalSectionGuard guard(&Device.seqParallelBeforRenderCS);
    xrCriticalSectionGuard pending(&Device.seqParallelBeforRenderPendingCS);
    if(link)link->support=nullptr; // a worker wake that arrives from now on finds no owner (never erase while draining)
    for(auto&entry:Device.seqParallelBeforRender)if(entry==callback)entry.clear();
    for(auto&entry:Device.seqParallelBeforRenderNext)if(entry==callback)entry.clear();
    for(auto&entry:Device.seqParallelBeforRenderPending)if(entry==callback)entry.clear();
}
bool CCharacterPhysicsSupport::UnbindPainterVoxelLive()
{
    using namespace painter_voxel_native;
    const u16 id=m_EntityAlife.ID();
    if(!m_painter_voxel_live&&!m_painter_voxel_bind_requested)return true;
    if(!m_painter_voxel_live&&m_painter_voxel_bind_target)return false; // the player's target finishes binding first
    if(held_hits.count(id)||painter_explosions::held_for(id)||painter_explosions::stashed_for(id)||painter_damaged_fragment::pending(id)||
       painter_gut_spill::wound_pending(id)||m_painter_geometry_installed||m_painter_geometry_pending>=0)return false;
    if(auto* source=m_EntityAlife.cast_stalker())if(painter_piece_continuation::owns(*source,0))return false;
    auto*k=smart_cast<IKinematics*>(m_EntityAlife.Visual());if(k&&k->LL_RenderGeometryGeneration()!=0)return false;
    if(m_painter_voxel_live){const auto& owner=*m_painter_voxel_live;
        if(owner.sequence||owner.geometry||!owner.committed_effects.empty()||owner.separation_job||owner.stage||owner.worker->pending())return false;}
    const bool bound=bool(m_painter_voxel_live);
    CancelPainterVoxelLive();
    unsigned assets=0;const u64 freed=trim_assets(assets);
    Msg("[painter_voxel_live] id=%u phase=unbound was=%s evicted_assets=%u evicted_asset_mb=%.1f resident_asset_mb=%.1f",
        id,bound?"bound":"binding",assets,freed/1048576.0,resident_asset_bytes()/1048576.0);
    return true;
}
void CCharacterPhysicsSupport::CapturePainterVoxelHit(const SHit& hit)
{
    body_hit_outcomes::set_sink(+[](const char*s)noexcept{Msg("%s host_us=%lld",s,static_cast<long long>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()));});
    body_hit_outcomes::Scope trace_scope(m_EntityAlife.ID(),hit.BulletID,hit.bone(),m_EntityAlife.g_Alive(),true);
    painter_voxel_native::SlowScope slow("capture",m_EntityAlife.ID());
    painter_hit_main::PartScope hit_main_capture(painter_hit_main::Capture); // triggerhitch
    BloodNativeProfile::Scope bloodCapture(BloodNativeProfile::Stage::PainterCapture,Device.dwFrame);
    if(auto* source=m_EntityAlife.cast_stalker())
        {painter_hit_main::PartScope hit_main_piece(painter_hit_main::Piece);if(painter_piece_continuation::capture(*source,hit))return;}
    using namespace painter_voxel_native;
    // R62 legacy rule: the blast's own hit, after its damage - did it kill? (its force tears wait for this; BlastSevers.h)
    if(hit.type()==ALife::eHitTypeExplosion)painter_explosions::note_blast_hit(m_EntityAlife.ID(),hit.weaponID,!m_EntityAlife.g_Alive());
    // melee (2026-11-12, value 3 only): a wound or strike hit asks Lua (CPainterVoxelMelee) whether it is a knife / blade / blunt blow
    const bool meleeHit=hit.type()!=ALife::eHitTypeFireWound&&painter_melee_event::eligible(hit);
    if(hit.type()!=ALife::eHitTypeFireWound&&!meleeHit){body_hit_outcomes::report("not_queued","hit_type_not_wound_or_melee");return;}
    if(!m_painter_voxel_live){
        // A bind in progress (the player's target whose outfit's body data is still being prepared): build the hit's
        // events now, with this frame's pose and the checks of a bound hit, and replay them when the bind completes.
        if(!m_painter_voxel_bind_requested||m_EntityAlife.getDestroy()){body_hit_outcomes::report("not_queued",m_EntityAlife.getDestroy()?"destroyed":"not_bound_no_bind_pending");return;}
        if(m_EntityAlife.g_Alive()){if(m_flags.test(fl_block_hit)||!hit.painter_effective_power_valid||
                !std::isfinite(hit.painter_effective_power)||hit.painter_effective_power<=0.f){body_hit_outcomes::report("not_queued","bind_living_protection");return;}}
        else if(!std::isfinite(hit.damage())||hit.damage()<=0.f){body_hit_outcomes::report("not_queued","bind_corpse_nonpositive_power");return;}
        auto*k=smart_cast<IKinematics*>(m_EntityAlife.Visual());if(!k||hit.bone()>=k->LL_BoneCount()||k->LL_RenderGeometryGeneration()!=0){body_hit_outcomes::report("not_queued","bind_visual_bone_or_generation");return;}
        const u16 id=m_EntityAlife.ID();const auto known=held_hits.find(id);const size_t before=known==held_hits.end()?0:known->second.events.size();
        if(before>=16){body_hit_outcomes::report("not_queued","bind_held_capacity_16",0,0,body_hit_outcomes::current,0,before);return;}
        std::vector<gfy_live::Impact> events;
        painter_hit_main::PartScope hit_main_policy(painter_hit_main::Policy);
        const auto built=meleeHit?painter_melee_event::build(m_EntityAlife,k,m_EntityAlife.g_Alive(),hit,u64(before),events,"painter_voxel_live"):
            painter_wound_event::build(m_EntityAlife,k,m_EntityAlife.g_Alive(),hit,u64(before),events,"painter_voxel_live");
        hit_main_policy.stop();
        body_hit_outcomes::tag_events(events);
        // A grenade fragment joins its explosion's batch, which only a bound body has.
        if(built.status!=painter_wound_event::Status::queued||!std::strcmp(built.policy,"fragment")||events.empty()){body_hit_outcomes::report("not_queued",!std::strcmp(built.policy,"fragment")?"bind_fragment_requires_bound_batch":events.empty()&&built.status==painter_wound_event::Status::queued?"policy_no_events":built.reason);return;}
        auto& held=held_hits[id];if(held.events.empty())held.since=Device.dwTimeGlobal;
        for(auto& event:events)if(held.events.size()<16){body_hit_outcomes::report("held","bind_in_progress",0,event.sequence,event.hit_trace,0,held.events.size());held.events.push_back(std::move(event));}
        else body_hit_outcomes::report("not_queued","bind_held_capacity_16_suffix",0,event.sequence,event.hit_trace,0,held.events.size());
        Msg("[painter_voxel_live] id=%u phase=hit_held events=%u bone=%u alive=%u",id,unsigned(held.events.size()),hit.bone(),unsigned(m_EntityAlife.g_Alive()));
        return;}
    auto reject=[&](const char* why){body_hit_outcomes::report("not_queued",why,m_painter_voxel_live->identity.spawn_cookie);state(-1,m_EntityAlife.ID(),int(m_painter_voxel_live->sequence));Msg("[painter_voxel_live] id=%u phase=hit_rejected reason=%s",m_EntityAlife.ID(),why);};
        auto&owner=*m_painter_voxel_live;
        painter_hit_main::PartScope hit_main_bind(painter_hit_main::Bind);
        // Bound on another thread (a save load): the main thread that publishes geometry adopts the owner.
        if(GetCurrentThreadId()!=owner.thread&&Device.m_geometry_publish_thread&&GetCurrentThreadId()==Device.m_geometry_publish_thread){
            Msg("[painter_voxel_live] id=%u phase=owner_thread_adopted",m_EntityAlife.ID());owner.thread=GetCurrentThreadId();}
        if(GetCurrentThreadId()!=owner.thread||m_EntityAlife.getDestroy())return reject("hit_owner_thread_or_state");
        if(m_EntityAlife.g_Alive()){
            if(m_flags.test(fl_block_hit)||!hit.painter_effective_power_valid||
                !std::isfinite(hit.painter_effective_power)||hit.painter_effective_power<=0.f)
                return reject("living_hit_blocked_after_protection");
        }else if(!std::isfinite(hit.damage())||hit.damage()<=0.f)
            return reject("corpse_hit_nonpositive_raw_power");
        auto*k=smart_cast<IKinematics*>(m_EntityAlife.Visual());if(k!=owner.visual||!same_rig(k,owner.exact_rig)||hit.bone()>=k->LL_BoneCount()||k->LL_RenderGeometryGeneration()!=owner.geometry){reject("hit_owner_identity");CancelPainterVoxelLive();return;}
        const auto phase=owner.lifecycle.observe(m_EntityAlife.g_Alive(),reinterpret_cast<std::uintptr_t>(m_pPhysicsShell));
        if(phase==painter_voxel_binding::Phase::rejected){reject("hit_lifecycle_shell_replaced_or_resurrected");CancelPainterVoxelLive();return;}
        // Only the first death shell may replace the admitted living null shell.
        owner.shell=m_pPhysicsShell;
        if(!painter_explosions::register_target(m_EntityAlife,owner.identity))return reject("grenade_target_registration");
        // Lua owns ammo classification, tuning and randomization. Only already
        // accepted hits on explicitly bound deformation owners reach this hook.
        // One accepted hit -> its worker events (PainterWoundEvent.h, shared with severed pieces): Lua sizes it, the
        // engine draws its forensic shape (entry burst included); a grenade fragment may cut a blade slot of several bores.
        std::vector<gfy_live::Impact> events;
        hit_main_bind.stop();
        painter_hit_main::PartScope hit_main_policy(painter_hit_main::Policy);
        const auto built=meleeHit?painter_melee_event::build(m_EntityAlife,k,m_EntityAlife.g_Alive(),hit,owner.sequence,events,"painter_voxel_live"):
            painter_wound_event::build(m_EntityAlife,k,m_EntityAlife.g_Alive(),hit,owner.sequence,events,"painter_voxel_live");
        hit_main_policy.stop();
        body_hit_outcomes::tag_events(events);
        if(built.status==painter_wound_event::Status::silent){body_hit_outcomes::report("not_queued",built.reason,owner.identity.spawn_cookie);return;}
        if(built.status==painter_wound_event::Status::rejected)return reject(built.reason);
        if(!std::strcmp(built.policy,"fragment")){
            if(!painter_explosions::capture_fragment(m_EntityAlife,std::move(events),built.blade>0,false))
                return reject("fragment_retained_capacity_or_quota");
            QueuePainterVoxelLive();return;}
        // R63 sever hold candidate: a rifle/slug-class killing hit on a limb, neck or head bone (the legacy-dismemberment sever and the
        // head events); KillHit (same frame) builds this body's death shell frozen until the sever commits.
        // klass (2026-11-11): the round test is gfy_live::severHoldRound (LiveDamageWorker.h) - Lua's kill_sever / round_class when
        // given, radius >= 7.5 mm otherwise as before - so the hold follows the same decision as the worker's killing-shot tear.
        {auto hold_bone=[&](u16 bone){if(bone>=k->LL_BoneCount())return false;const char* n=k->LL_BoneName_dbg(bone);if(!n)return false; // limbs, neck, head
             static const char* const parts[]={"thigh","calf","foot","toe","upperarm","forearm","hand","finger","neck","head","jaw","eye"};
             for(const char* q:parts)if(std::strstr(n,q))return true;return false;};
         bool hold_candidate=false;for(const auto&event:events)hold_candidate|=event.killing_hit&&!event.blast&&gfy_live::severHoldRound(event)&&event.penetration_m>0&&hold_bone(event.hit_bone);
         for(const auto&event:events)hold_candidate|=event.killing_hit&&event.melee_sever&&hold_bone(event.hit_bone); // melee: a killing blade blow holds for its cut
         // R65: no 20 cm reach gate (the kill sever below matches)
        painter_hit_main::PartScope hit_main_queue(painter_hit_main::Queue);
        for(auto&event:events){event.sequence=owner.sequence+1;const u64 hit_sequence=event.sequence;
            event.trace_gear=painter_log_verbose(); // gearbody: the worker reports where a shot through merged gear was mapped
            if(!owner.worker->enqueue(std::move(event))){body_hit_outcomes::batch("not_queued","prior_sub_event_enqueue_failed",owner.identity.spawn_cookie,events,std::size_t(&event-events.data())+1);return reject("event_queue_or_sequence_limit");}
            owner.hit_times.emplace_back(hit_sequence,std::chrono::steady_clock::now());if(owner.hit_times.size()>256)owner.hit_times.pop_front();++owner.sequence;state(1,m_EntityAlife.ID(),int(owner.sequence));}
        if(hold_candidate)owner.hold_hit_time=Device.dwTimeGlobal; // R63: marked only once its events were queued
        }
        // Retain exact dimensions by sequence even when geometry rejects the
        // event before ImpactResult receives its material-loss receipt fields.
        if(painter_log_verbose())Msg("[painter_voxel_live] id=%u phase=hit_accepted sequence=%llu bone=%u bullet=%u ammo=%s pellets=%u radius_m=%.17g penetration_m=%.17g exit_mult=%.17g policy=%s blade=%u",m_EntityAlife.ID(),owner.sequence,hit.bone(),hit.BulletID,built.ammo_section.c_str(),built.pellets,built.radius,built.penetration,built.exit_mult,built.policy,built.blade);
        // The job starts now, not at the next barrier drain (the job reads its own snapshot and the event's palette).
        {painter_hit_main::PartScope hit_main_dispatch(painter_hit_main::Dispatch);
        if(pool&&k->LL_RenderGeometryGeneration()==owner.geometry)pool->dispatch(owner.worker,owner.geometry);}

}
void CCharacterPhysicsSupport::PainterVoxelBlast(const Fvector& blast_world,float effect,LPCSTR section,u16 initiator,const PainterFragmentSource* fragment_source)
{
    using namespace painter_voxel_native;
    if(!fragment_source||!fragment_source->valid()||fragment_source->world!=Level().PainterWorldCookie()||
       fragment_source->initiator!=initiator||!section||std::strcmp(fragment_source->section.data(),section)){
        Msg("[painter_voxel_live] id=%u phase=blast_skipped reason=fragment_source",m_EntityAlife.ID());return;}

    // A stump (the corpse after a split) takes the craters on its piece session (PainterPieceContinuation_impl.h).
    auto* stalker=m_EntityAlife.cast_stalker();const bool continued=stalker&&painter_piece_continuation::owns(*stalker,0);
    if((!m_painter_voxel_live&&!continued)||m_EntityAlife.getDestroy()||!section||!std::isfinite(effect)||effect<=0.f){
        Msg("[painter_voxel_live] id=%u phase=blast_skipped reason=%s effect=%.3f",m_EntityAlife.ID(),
            (!m_painter_voxel_live&&!continued)?"not_bound":m_EntityAlife.getDestroy()?"destroyed":"no_effect",double(effect));return;}
    auto*k=smart_cast<IKinematics*>(m_EntityAlife.Visual());if(!k){Msg("[painter_voxel_live] id=%u phase=blast_skipped reason=no_visual",m_EntityAlife.ID());return;}
    if(!continued){auto&owner=*m_painter_voxel_live;
        if(GetCurrentThreadId()!=owner.thread){Msg("[painter_voxel_live] id=%u phase=blast_skipped reason=thread",m_EntityAlife.ID());return;}
        if(k!=owner.visual||!same_rig(k,owner.exact_rig)||k->LL_RenderGeometryGeneration()!=owner.geometry){
            Msg("[painter_voxel_live] id=%u phase=blast_skipped reason=visual_changed",m_EntityAlife.ID());return;}
        if(owner.lifecycle.observe(m_EntityAlife.g_Alive(),reinterpret_cast<std::uintptr_t>(m_pPhysicsShell))==painter_voxel_binding::Phase::rejected){
            Msg("[painter_voxel_live] id=%u phase=blast_skipped reason=lifecycle_rejected",m_EntityAlife.ID());return;}
        owner.shell=m_pPhysicsShell;
        if(!painter_explosions::register_target(m_EntityAlife,owner.identity)){
            Msg("[painter_voxel_live] id=%u phase=blast_skipped reason=register_target",m_EntityAlife.ID());return;}}
    const auto gore=painter_deformation::snapshot();const u16 cranial_bone=k->LL_BoneID("bip01_head");
    const float cover=_min(1.f,effect*2.f); // ~0.5-1 for an open body within a few metres, towards 0 behind cover
    // rpgblast: the gore by the explosive (blast_severs::rgdDistance: its "blast" and blast radius against the RGD-5's; every
    // explosive read as an RGD-5 before) - the craters' zones, the armour gate and the force tears below read it
    const float blast_power=pSettings->line_exist(section,"blast")?pSettings->r_float(section,"blast"):3.f;
    const auto rgd_distance=[&](float d){return float(blast_severs::rgdDistance(double(d),double(blast_power),double(fragment_source->radius)));};
    constexpr float gore_r=2.5f;constexpr std::size_t max_craters=48;
    struct Zone {LPCSTR bone,child;float half,along;}; // zone point = lerp(bone, child, along); half = half thickness
    static const Zone zones[]={
        {"bip01_head","bip01_neck",.09f,-.6f},{"bip01_neck","bip01_head",.06f,.5f},
        {"bip01_spine1","bip01_spine2",.14f,.5f},{"bip01_spine","bip01_spine1",.13f,.5f},{"bip01_pelvis","bip01_spine",.13f,0.f},
        {"bip01_l_thigh","bip01_l_calf",.08f,.5f},{"bip01_r_thigh","bip01_r_calf",.08f,.5f},
        {"bip01_l_calf","bip01_l_foot",.06f,.5f},{"bip01_r_calf","bip01_r_foot",.06f,.5f},
        {"bip01_l_foot","bip01_l_toe0",.05f,.5f},{"bip01_r_foot","bip01_r_toe0",.05f,.5f},
        {"bip01_l_upperarm","bip01_l_forearm",.05f,.5f},{"bip01_r_upperarm","bip01_r_forearm",.05f,.5f},
        {"bip01_l_forearm","bip01_l_hand",.045f,.5f},{"bip01_r_forearm","bip01_r_hand",.045f,.5f},
        {"bip01_l_hand","bip01_l_finger1",.04f,.3f},{"bip01_r_hand","bip01_r_finger1",.04f,.3f}};
    const Fmatrix object_world=m_EntityAlife.XFORM();Fmatrix world_to_object;world_to_object.invert(object_world);
    Fvector E;world_to_object.transform_tiny(E,blast_world);
    struct Point3 {u16 bone;Fvector at;float half;Fvector axis,from,to;};std::vector<Point3> points;decltype(gfy_live::Impact::bone_model_render) palette;
    const u16 pelvisBone=k->LL_BoneID("bip01_pelvis"),neckBone=k->LL_BoneID("bip01_neck");Fvector pelvisAt{},neckAt{}; // the body's long axis
    {xrCriticalSectionGuard pose(&k->UCalc_Mutex);k->CalculateBones_Invalidate();k->CalculateBones(TRUE);
        palette.reserve(k->LL_BoneCount());for(u16 b=0;b<k->LL_BoneCount();++b)palette.push_back(matrix(k->LL_GetBoneInstance(b).mRenderTransform));
        for(const auto&z:zones){const u16 b=k->LL_BoneID(z.bone),c=k->LL_BoneID(z.child);if(b==BI_NONE||c==BI_NONE)continue;
            const Fvector from=k->LL_GetBoneInstance(b).mTransform.c,to=k->LL_GetBoneInstance(c).mTransform.c;
            Fvector at,axis;at.lerp(from,to,z.along);
            axis.sub(to,from);axis.normalize_safe();
            points.push_back({b,at,z.half,axis,from,to});}
        if(pelvisBone!=BI_NONE&&neckBone!=BI_NONE){pelvisAt=k->LL_GetBoneInstance(pelvisBone).mTransform.c;neckAt=k->LL_GetBoneInstance(neckBone).mTransform.c;}}
    // slab (R18): a box cut in the blade's plane instead of a bore (slab_n its normal, slab_s across it, slab_w half width)
    struct Crater {float p;u16 bone;Fvector start,dir;float radius,depth;bool row=false,bite=false;std::uint32_t blade=0;
        bool slab=false;Fvector slab_n,slab_s;float slab_w=0.f;};std::vector<Crater> craters,legacyTorsoRows; // R62: the rows stay unused (force tears)
    ::luabind::functor<bool> gate;const bool gated=ai().script_engine().functor("_G.CPainterVoxelBlastZone",gate);
    float pmax=0.f;unsigned armoured=0;
    Msg("[painter_voxel_live] id=%u phase=blast_received effect=%.2f cover=%.2f radius=%.2f alive=%u zones=%u section=%s t=%u tc=%u blast=%.2f reach=%.2f", // rpgblast: the last two
        m_EntityAlife.ID(),double(effect),double(cover),double(fragment_source->radius),unsigned(m_EntityAlife.g_Alive()),unsigned(points.size()),section,Device.dwTimeGlobal,Device.dwTimeContinual,
        double(blast_power),blast_severs::reachOf(double(blast_power),double(fragment_source->radius)));
    const u16 chest=k->LL_BoneID("bip01_spine1"),belly=k->LL_BoneID("bip01_spine"),pelvis=k->LL_BoneID("bip01_pelvis");
    struct Allowed {Point3 zone;float proximity;};std::vector<Allowed> allowed;
    // Qualify native cover and the Lua zone gate before choosing the row. An
    // ineligible pelvis or armoured spine cannot veto another eligible torso.
    for(const auto&z:points){const float d=rgd_distance(z.at.distance_to(E));const float p=(1.f-d/gore_r)*cover;if(p<.12f)continue; // rpgblast: the zone by the explosive
        if(gated){::luabind::object source=::luabind::newtable(ai().script_engine().lua());
            source["owner_id"]=m_EntityAlife.ID();source["hit_bone"]=unsigned(z.bone);
            {LPCSTR bone_name=k->LL_BoneName_dbg(z.bone);source["bone_name"]=bone_name?bone_name:"";}
            source["alive"]=bool(m_EntityAlife.g_Alive());source["proximity"]=double(p);
            if(initiator!=u16(-1))source["initiator"]=unsigned(initiator); // NPC-on-NPC switch (Lua)
            if(!gate(source)){++armoured;continue;}}
        pmax=_max(pmax,p);allowed.push_back({z,p});}
    constexpr float torso_row_minimum=.5f;
    u16 nearestTorso=BI_NONE;{float best=1e9f;for(const auto& a:allowed){const auto& z=a.zone;
        if((z.bone==chest||z.bone==belly)&&a.proximity>=torso_row_minimum){
            const float d=z.at.distance_to(E);if(d<best){best=d;nearestTorso=z.bone;}}}}
    constexpr float golden=2.399963229728653f;
    for(const auto& a:allowed){const auto& z=a.zone;const float p=a.proximity;
        const bool torso=z.bone==chest||z.bone==belly||z.bone==pelvis;
        Fvector dir;dir.sub(z.at,E);if(dir.square_magnitude()<1e-8f)dir.set(0.f,1.f,0.f);dir.normalize_safe();
        Fvector side;side.crossproduct(dir,Fvector().set(0.f,1.f,0.f));if(side.magnitude()<.3f)side.crossproduct(dir,Fvector().set(1.f,0.f,0.f));side.normalize_safe();
        Fvector up;up.crossproduct(side,dir);up.normalize_safe();
        auto disc=[&](unsigned turn,unsigned k,unsigned n,float reach){const float a=golden*float(turn),rr=reach*z.half*_sqrt((float(k)+.5f)/float(n));
            Fvector at=z.at;at.mad(side,_cos(a)*rr);at.mad(up,_sin(a)*rr);at.mad(dir,-(z.half+.06f));return at;};
        // R62: limb craters gouge (chewed surface, exposed muscle) instead of coring the limb (was up to 6 cm wide, 25 cm deep):
        // a limb comes off by the blast's force tears (BlastSevers.h), not because craters hollowed it out.
        const unsigned n=p<.3f?1u:(p<.5f?2u:(p<.7f?4u:6u));const float radius=torso?_min(.04f,.015f+.03f*p):_min(.045f,.015f+.03f*p);
        const float depth=torso?_min(.15f,.03f+.15f*p*p):_min(.07f,.02f+.06f*p*p);
        for(unsigned i=0;i<n;++i)craters.push_back({p,z.bone,disc(i,i,n,.75f),dir,radius,depth});
        if(p>=.4f){const unsigned m=unsigned(5.f*p+.5f); // chewed surface: shallow wide bites across the face
            for(unsigned i=0;i<m;++i)craters.push_back({p*.9f,z.bone,disc(i+11,i,_max(1u,m),.95f),dir,_min(.05f,.025f+.03f*p),.03f+.03f*p,false,true});}
        if(torso&&z.bone==nearestTorso&&z.bone!=pelvis&&p>=torso_row_minimum)
            {const float halfWidth=_max(1.4f*z.half,.22f);
             Fvector axis=z.axis;if(axis.square_magnitude()<1e-8f)axis.set(0.f,1.f,0.f);axis.normalize_safe();
             Fvector rowDir=dir;rowDir.mad(axis,-dir.dotproduct(axis));
             if(rowDir.square_magnitude()<1e-8f){Fvector fallback;fallback.set(_abs(axis.z)<.8f?0.f:1.f,0.f,_abs(axis.z)<.8f?1.f:0.f);
                rowDir.set(fallback).mad(axis,-fallback.dotproduct(axis));}
             rowDir.normalize_safe();Fvector rowSide;rowSide.crossproduct(axis,rowDir).normalize_safe();
             for(unsigned i=0;i<14;++i){Fvector at=z.at;at.mad(rowSide,(-1.f+2.f*float(i)/13.f)*halfWidth);at.mad(rowDir,-(z.half+.06f));
                legacyTorsoRows.push_back({1.5f+p,z.bone,at,rowDir,.03f,.45f,true});}}}
    unsigned bladeBores=0;std::vector<blast_severs::Request> forceTears;blast_severs::Choice forceChoice;
    {struct SiteDef {LPCSTR bone,child;double radius;int chain;bool trunk;double lo,hi;};
     static const SiteDef defs[]={ // radii as the joint profile's (painter_butcher_damage.script)
        {"bip01_neck","bip01_head",.075,0,false,.15,.85},
        {"bip01_l_upperarm","bip01_l_forearm",.090,1,false,.30,.80},{"bip01_l_forearm","bip01_l_hand",.065,1,false,.25,.90},
        {"bip01_r_upperarm","bip01_r_forearm",.090,2,false,.30,.80},{"bip01_r_forearm","bip01_r_hand",.065,2,false,.25,.90},
        {"bip01_l_thigh","bip01_l_calf",.115,3,false,.30,.80},{"bip01_l_calf","bip01_l_foot",.085,3,false,.25,.90},
        {"bip01_r_thigh","bip01_r_calf",.115,4,false,.30,.80},{"bip01_r_calf","bip01_r_foot",.085,4,false,.25,.90},
        // rpgblast: the chest site under the arms (38-48 % of spine1 -> neck; was 55-85 %, shoulder height: its plane ran through
        // the shoulders of the bind pose and the torso never split there)
        {"bip01_pelvis","bip01_spine1",.180,5,true,.85,1.15},{"bip01_spine1","bip01_neck",.170,5,true,.38,.48}};
     blast_severs::Settings settings;settings.blast=double(blast_power);settings.radius=double(fragment_source->radius); // rpgblast: the sites by the explosive
     std::vector<blast_severs::Site> sites;
     {xrCriticalSectionGuard pose(&k->UCalc_Mutex);
      for(const auto&d:defs){const u16 b=k->LL_BoneID(d.bone),c=k->LL_BoneID(d.child);if(b==BI_NONE||c==BI_NONE)continue;
        const Fvector from=k->LL_GetBoneInstance(b).mTransform.c,to=k->LL_GetBoneInstance(c).mTransform.c;
        blast_severs::Site site;site.bone=b;site.from={from.x,from.y,from.z};site.to={to.x,to.y,to.z};site.radius=d.radius;
        site.chain=d.chain;site.trunk=d.trunk;site.lo=d.lo;site.hi=d.hi;sites.push_back(site);}}
     const blast_severs::V3 at{E.x,E.y,E.z};
     // armour: the Lua zone gate per site the blast reaches (its proximity at the point nearest the explosion)
     for(auto&site:sites){if(!gated)break;const auto ab=blast_severs::sub(site.to,site.from);const double L2=blast_severs::dot(ab,ab);if(!(L2>1e-8))continue;
        const double t=std::clamp(blast_severs::dot(blast_severs::sub(at,site.from),ab)/L2,site.lo,site.hi);
        const double d=blast_severs::rgdDistance(blast_severs::len(blast_severs::sub(blast_severs::add(site.from,blast_severs::mul(ab,t)),at)),
            settings.blast,settings.radius); // rpgblast: the armour gate's proximity by the explosive
        const double p=std::clamp((1-d/settings.reach)*double(cover),0.,1.);if(p<blast_severs::inPlay(site,settings))continue;
        ::luabind::object source=::luabind::newtable(ai().script_engine().lua());
        source["owner_id"]=m_EntityAlife.ID();source["hit_bone"]=unsigned(site.bone);
        {LPCSTR bone_name=k->LL_BoneName_dbg(site.bone);source["bone_name"]=bone_name?bone_name:"";}
        source["alive"]=bool(m_EntityAlife.g_Alive());source["proximity"]=p;
        if(initiator!=u16(-1))source["initiator"]=unsigned(initiator);
        if(!gate(source)){site.open=false;++armoured;}}
     u64 seed=(fragment_source->world*0x9E3779B97F4A7C15ull)^fragment_source->cookie^(u64(fragment_source->explosive)<<48)^
           (u64(fragment_source->emitted_at)*0xC2B2AE3D27D4EB4Full)^(u64(m_EntityAlife.ID())*0xD6E8FEB86659FD93ull);
     forceChoice=blast_severs::choose(sites,at,double(cover),seed,settings);forceTears=forceChoice.tears;}
    std::stable_sort(craters.begin(),craters.end(),[](const Crater&a,const Crater&b){return a.p>b.p;});
    // Removal budget (~8 L of torso craters per blast): a hollowed torso overruns the connectivity scan and nothing separates.
    {std::vector<Crater> kept;float volume=0.f;std::size_t plain=0;constexpr float budget=.008f; // torso craters only; blade slabs have their own bounded workload
     for(const auto&c:craters){const bool torsoCrater=!c.row&&(c.bone==chest||c.bone==belly||c.bone==pelvis);
        if(c.blade){kept.push_back(c);continue;} // blade rows are capped where they are made (max_blade_bores)
        const float v=torsoCrater?3.14159265f*c.radius*c.radius*_min(c.depth,.3f):0.f;
        if(plain>=max_craters||volume+v>budget)continue;kept.push_back(c);volume+=v;++plain;}
     craters.swap(kept);}
    std::vector<gfy_live::Impact> first,later;first.reserve(craters.size());later.reserve(craters.size());
    u64 index=0;const auto coord=[](float value){return u32(std::int64_t(std::trunc(double(value)*1000.)));};
    for(const auto&c:craters){gfy_live::Impact event;event.sequence=++index;event.gore=gore;event.cranial_bone=cranial_bone;
        event.accepted_pose_point={c.start.x,c.start.y,c.start.z};event.accepted_pose_direction={c.dir.x,c.dir.y,c.dir.z};
        event.bone_model_render=palette;event.radius=c.radius;event.penetration_m=c.depth;event.exit_diameter_mult=1;
        event.ammo_section=section;event.hit_bone=c.bone;event.hit_power=100.*_min(c.p,1.f);event.initiator=initiator;
        event.cavitation=std::clamp(.4+.4*double(_min(c.p,1.f)),0.,2.);event.burst=event.vent=event.entry_burst=0;
        event.blast=true;event.source_search_distance=.2;event.blade=c.blade;
        if(c.slab){event.slab=true;event.ragged_slab=true;event.slab_normal={c.slab_n.x,c.slab_n.y,c.slab_n.z};event.slab_side={c.slab_s.x,c.slab_s.y,c.slab_s.z};
            event.slab_half_width=c.slab_w;event.cavitation=0;}
        u64 seed=(u64(m_EntityAlife.ID())<<48)^(index*0x9E3779B97F4A7C15ull)^coord(blast_world.x)^(u64(coord(blast_world.z))<<20);
        // Different blasts at the same place still get different torn edges.
        // The captured explosion identity is stable through deferred retries.
        if(event.ragged_slab)seed=gfy_live::RaggedBlade::mix(seed^fragment_source->cookie^fragment_source->world);
        event.ammo_seed=seed?seed:1;
        const bool torso=c.bone==chest||c.bone==belly||c.bone==pelvis;
        // R62: a blast that tears by force puts every crater behind its tears (the tears alone decide the split, which comes
        // sooner); without force tears the old order stands (limb craters first)
        const bool sever_first=std::getenv("GFY_BLAST_ALL_AT_ONCE")||(forceTears.empty()&&(c.row||(!c.bite&&!torso)));
        (sever_first?first:later).push_back(std::move(event));}
    if(first.empty()&&forceTears.empty()){first=std::move(later);later.clear();}
    for(const auto&t:forceTears){gfy_live::Impact event;event.sequence=++index;event.gore=gore;event.cranial_bone=cranial_bone;
        event.accepted_pose_point={float(t.point.x),float(t.point.y),float(t.point.z)};event.accepted_pose_direction={float(t.push.x),float(t.push.y),float(t.push.z)};
        event.bone_model_render=palette;event.radius=.01;event.penetration_m=.01;event.exit_diameter_mult=1;event.ammo_section=section;
        event.hit_bone=t.bone;event.hit_power=100.*t.proximity;event.initiator=initiator;event.blast=true;event.tear_request=true;
        event.source_search_distance=.2;u64 seed=gfy_live::RaggedBlade::mix((u64(m_EntityAlife.ID())<<48)^(index*0x9E3779B97F4A7C15ull)^fragment_source->cookie^fragment_source->world);
        event.ammo_seed=seed?seed:1;first.push_back(std::move(event));}
    if(first.empty()){Msg("[painter_voxel_live] id=%u phase=blast_skipped reason=nothing_in_reach craters=%u armoured_zones=%u p_max=%.2f cover=%.2f",
        m_EntityAlife.ID(),unsigned(craters.size()),armoured,double(pmax),double(cover));return;}
    const bool holdForTears=!forceTears.empty()&&!continued;
    const bool hold_later=continued||holdForTears||(m_painter_voxel_live&&m_painter_voxel_live->separation_ready&&stalker&&separation_allowed(*stalker));
    if(!painter_explosions::capture_blast(m_EntityAlife,*fragment_source,std::move(first),std::move(later),hold_later)){
        Msg("[painter_voxel_live] id=%u phase=blast_rejected reason=retained_capacity_or_identity",m_EntityAlife.ID());return;}
    painter_explosions::note_blast(m_EntityAlife.ID(),blast_world,pmax,fragment_source->explosive);
    if(holdForTears&&m_painter_voxel_live){m_painter_voxel_live->hold_blast_explosive=fragment_source->explosive;m_painter_voxel_live->hold_blast_time=Device.dwTimeGlobal;} // R63 sever hold
    if(continued)painter_piece_continuation::queue(*stalker);else QueuePainterVoxelLive();
    Msg("[painter_voxel_live] id=%u phase=blast_retained craters=%u blade_bores=%u force_tears=%u wanted=%u eligible=%u force_pmax=%.2f trunk_chance=%.2f trunk_roll=%u p_max=%.2f cover=%.2f armoured_zones=%u alive=%u section=%s t=%u tc=%u",
        m_EntityAlife.ID(),unsigned(craters.size()),bladeBores,unsigned(forceTears.size()),forceChoice.wanted,forceChoice.eligible,forceChoice.pmax,forceChoice.trunkChance,unsigned(forceChoice.trunkRoll),pmax,cover,armoured,unsigned(m_EntityAlife.g_Alive()),section,Device.dwTimeGlobal,Device.dwTimeContinual);
    for(const auto&t:forceTears)Msg("[painter_voxel_live] id=%u phase=blast_force_tear bone=%s proximity=%.2f score=%.3f",m_EntityAlife.ID(),k->LL_BoneName_dbg(t.bone),t.proximity,t.score);
}
unsigned CCharacterPhysicsSupport::WarmPainterVoxelShaders(IKinematics* k)
{
    // The cavity pairs LL_StageRuntimeGeometry receives: "models\model" with
    // every tissue texture (or the stump texture when the catalog has none).
    if(!k||!m_painter_voxel_live)return 0;
    const auto& tissue=m_painter_voxel_live->tissue_textures;
    return k->LL_WarmRuntimeShaders("models\\model",tissue.empty()?"wm\\painter_stump_limb":nullptr,tissue.empty()?nullptr:&tissue,0,m_painter_voxel_live->worker->tissue_field());
}
void CCharacterPhysicsSupport::QueuePainterVoxelLive()
{
    if(auto* source=m_EntityAlife.cast_stalker())painter_piece_continuation::queue(*source);
    // Held hits wait for the bind in progress at most 30 s (the player's target) or 10 s.
    if(!painter_voxel_native::held_hits.empty()&&!m_painter_voxel_live){const auto held=painter_voxel_native::held_hits.find(m_EntityAlife.ID());
        if(held!=painter_voxel_native::held_hits.end()&&(!m_painter_voxel_bind_requested||Device.dwTimeGlobal-held->second.since>(m_painter_voxel_bind_target?30000u:10000u)))
            painter_voxel_native::drop_held_hits(m_EntityAlife.ID(),m_painter_voxel_bind_requested?"bind_timeout":"bind_stopped");}
    if(m_painter_voxel_bind_requested&&!m_painter_voxel_live&&s32(Device.dwTimeContinual-m_painter_voxel_bind_retry_at)>=0){m_painter_voxel_bind_requested=false;m_painter_voxel_bind_retry_at=Device.dwTimeContinual+250;BindPainterVoxelLive();}
    if(m_painter_voxel_live)painter_voxel_native::govern(); // rate-limited, all owners
    if(m_painter_voxel_live&&m_painter_voxel_live->shader_warm_requested&&Device.m_geometry_publish_thread&&GetCurrentThreadId()==Device.m_geometry_publish_thread)
        if(auto*k=smart_cast<IKinematics*>(m_EntityAlife.Visual())){m_painter_voxel_live->shader_warm_requested=false;WarmPainterVoxelShaders(k);}
    if(!m_painter_voxel_live||m_painter_voxel_queued)return;
    if(m_painter_voxel_live->burn_prime_wanted && !m_painter_voxel_live->burn_prime_queued && !m_painter_voxel_live->burn_prime_ready)
        PrimePainterBurnGeometry(m_painter_voxel_live->identity.spawn_cookie);
    const auto& owner=*m_painter_voxel_live;
    bool separation_due=false;
    if(owner.separation_ready&&!m_EntityAlife.g_Alive()&&
       s32(Device.dwTimeGlobal-owner.separation_policy_retry_at)>=0){
        auto snapshot=owner.worker->capture_damage();
        separation_due=snapshot&&snapshot->stamp().revision>owner.separation_checked_revision;
    }
    if(!owner.worker->pending()&&!owner.separation_job&&
       !painter_damaged_fragment::pending(m_EntityAlife.ID())&&!separation_due&&owner.committed_effects.empty()&&
       !painter_explosions::held_for(m_EntityAlife.ID())&&!painter_explosions::stashed_for(m_EntityAlife.ID())&&
       !painter_gut_spill::wound_pending(m_EntityAlife.ID())&&
       !(!m_EntityAlife.g_Alive()&&m_EntityAlife.cast_stalker()&&
         painter_cranial_remnants::has_pending(*m_EntityAlife.cast_stalker())))return;
    // The worker's ready hook may have queued the same callback already (see BindPainterVoxelLive): one entry per drain.
    m_painter_voxel_queued=true;xrCriticalSectionGuard guard(&Device.seqParallelBeforRenderCS);
    auto callback=xr_make_delegate(this,&CCharacterPhysicsSupport::ApplyPainterVoxelLive);
    if(std::find(Device.seqParallelBeforRender.begin(),Device.seqParallelBeforRender.end(),callback)==Device.seqParallelBeforRender.end())
        Device.seqParallelBeforRender.push_back(callback);
}
extern int painter_sever_hold_ms; // below
void CCharacterPhysicsSupport::ApplyPainterVoxelLive()
{
    using namespace painter_voxel_native;m_painter_voxel_queued=false;if(!m_painter_voxel_live)return;
    // R64 sever-hold liveness (live R63 run: cpu_queued, then nothing for 6 s): the held corpse sleeps (no in_UpdateCL), so the
    // hold deadline is checked here too (below, after the owner checks), and this callback queues itself for the next drain
    // whenever it returns with a separation job (also one it queued below), a pending fragment batch or an active hold.
    struct RequeueAtExit {CCharacterPhysicsSupport&s;~RequeueAtExit(){
        if(!s.m_painter_voxel_live||s.m_EntityAlife.getDestroy())return;
        if(!(s.m_painter_voxel_live->separation_job||painter_damaged_fragment::pending(s.m_EntityAlife.ID())||s.m_painter_hold))return;
        try{xrCriticalSectionGuard guard(&Device.seqParallelBeforRenderCS);
            auto callback=xr_make_delegate(&s,&CCharacterPhysicsSupport::ApplyPainterVoxelLive);
            if(std::find(Device.seqParallelBeforRenderNext.begin(),Device.seqParallelBeforRenderNext.end(),callback)==Device.seqParallelBeforRenderNext.end())
                Device.seqParallelBeforRenderNext.push_back(callback);
            s.m_painter_voxel_queued=true;}catch(...){}}} requeueAtExit{*this};
    SlowScope slow("apply",m_EntityAlife.ID());
    BloodNativeProfile::Scope bloodApply(BloodNativeProfile::Stage::PainterApply,Device.dwFrame);
    if(m_painter_voxel_live->separation_job||painter_damaged_fragment::pending(m_EntityAlife.ID())){
        xrCriticalSectionGuard guard(&Device.seqParallelBeforRenderCS);
        auto callback=xr_make_delegate(this,&CCharacterPhysicsSupport::ApplyPainterVoxelLive);
        if(std::find(Device.seqParallelBeforRenderNext.begin(),Device.seqParallelBeforRenderNext.end(),callback)==Device.seqParallelBeforRenderNext.end())
            Device.seqParallelBeforRenderNext.push_back(callback);
        m_painter_voxel_queued=true;}
    auto&owner=*m_painter_voxel_live;auto*k=smart_cast<IKinematics*>(m_EntityAlife.Visual());
    if(Device.IsGeometryPublishBarrier()&&GetCurrentThreadId()!=owner.thread)owner.thread=GetCurrentThreadId(); // see CapturePainterVoxelHit
    if(!Device.IsGeometryPublishBarrier()||GetCurrentThreadId()!=owner.thread||m_EntityAlife.getDestroy()||k!=owner.visual||!same_rig(k,owner.exact_rig)){state(-1,m_EntityAlife.ID(),int(owner.sequence));Msg("[painter_voxel_live] id=%u phase=rejected reason=owner_identity_or_barrier",m_EntityAlife.ID());CancelPainterVoxelLive();return;}
    const auto phase=owner.lifecycle.observe(m_EntityAlife.g_Alive(),reinterpret_cast<std::uintptr_t>(m_pPhysicsShell));
    if(phase==painter_voxel_binding::Phase::rejected){state(-1,m_EntityAlife.ID(),int(owner.sequence));Msg("[painter_voxel_live] id=%u phase=rejected reason=lifecycle_shell_replaced_or_resurrected",m_EntityAlife.ID());CancelPainterVoxelLive();return;}
    if(owner.shell!=m_pPhysicsShell)Msg("[painter_voxel_live] id=%u phase=death_shell_adopted cookie=%llu generation=%llu",m_EntityAlife.ID(),owner.identity.spawn_cookie,owner.geometry);
    owner.shell=m_pPhysicsShell;
    // R64 hold-timeout ordering: the deadline's physical release (Enable, velocity, fatal impulse) only for a validated owner and at a
    // safe physics boundary; while paused or while the world steps it waits for the next drain (RequeueAtExit keeps polling). No
    // enabled / full-active shell is required: the held shell is disabled on purpose.
    if(m_painter_hold&&Device.dwTimeContinual-m_painter_hold_started>u32(painter_sever_hold_ms)&&!Device.Paused()&&physics_world()&&!physics_world()->Processing())
        PainterHoldRelease("timeout");
    if(auto* object=m_EntityAlife.cast_stalker())painter_cranial_remnants::poll(*object);
    if(!painter_explosions::register_target(m_EntityAlife,owner.identity))return;
    painter_explosions::pump(m_EntityAlife,bool(owner.separation_job)||painter_damaged_fragment::pending(m_EntityAlife.ID()),
        owner.separation_checked_revision,[&](gfy_live::Impact&& event,u64& admitted){
            if(event.tear_request){ // R62 legacy rule: tear only a body this blast left dead
                // rpgblast: decided when the blast's death can have arrived (painter_grenade::tearGate) - before, the wave hit's
                // outcome decided at once, and a body that hit left alive lost its tears though the same blast killed it frames later
                const auto* blast=painter_explosions::blast_on(m_EntityAlife.ID());
                const auto gate=painter_grenade::tearGate(m_EntityAlife.g_Alive()!=0,blast!=nullptr,Device.dwTimeGlobal,blast?blast->time:0u);
                if(gate==painter_grenade::TearGate::wait)return false; // the blast's death is not in yet
                if(gate==painter_grenade::TearGate::survived)event.tear_survived=true;} // the worker records the request and tears nothing
            event.sequence=owner.sequence+1;if(!event.sequence)return false;
            owner.hit_times.emplace_back(event.sequence,std::chrono::steady_clock::now());
            if(!owner.worker->enqueue(std::move(event))){owner.hit_times.pop_back();return false;}
            admitted=++owner.sequence;
            if(owner.hit_times.size()>256)owner.hit_times.pop_front();state(1,m_EntityAlife.ID(),int(owner.sequence));return true;});
    // A marked abdominal wound spills its loop once this body is a ragdoll (PainterGutSpill_impl.h).
    if(painter_gut_spill::wound_pending(m_EntityAlife.ID()))if(auto* stalker=m_EntityAlife.cast_stalker())painter_gut_spill::poll_wound(*stalker,m_pPhysicsShell);
    // Living skin publication requires no ragdoll. Death retains the same
    // worker, cookie, revision and geometry; wait until its first shell is ready.
    if(Device.Paused()||!physics_world()||physics_world()->Processing()||phase==painter_voxel_binding::Phase::awaiting_death_shell||(m_pPhysicsShell&&!m_pPhysicsShell->isFullActive())){
        // Publication waits for the ragdoll; the carve need not: queued hits are dispatched meanwhile.
        if(!Device.Paused()&&pool&&k->LL_RenderGeometryGeneration()==owner.geometry)pool->dispatch(owner.worker,owner.geometry);
        return;}
    if(!owner.committed_effects.empty()){
        // Receipts may survive a full Lua queue across barriers. Their surface
        // anchors belong to the committed geometry, not an external replacement
        // that happens to keep the same visual pointer and bone layout.
        if(k->LL_RenderGeometryGeneration()!=owner.geometry){
            state(-1,m_EntityAlife.ID(),int(owner.sequence));
            Msg("[painter_voxel_live] id=%u phase=rejected reason=receipt_geometry_replaced",m_EntityAlife.ID());
            CancelPainterVoxelLive();return;
        }
        if(!painter_committed_effects::drain(*m_EntityAlife.cast_stalker(),owner.committed_effects,true))return;
    }
    const bool separation_active=!m_EntityAlife.g_Alive()&&
        (owner.separation_ready||owner.separation_job||
         painter_damaged_fragment::pending(m_EntityAlife.ID()));
    const bool allow_separation=separation_active&&
        separation_allowed(*m_EntityAlife.cast_stalker());
    if(separation_active&&!allow_separation){
        const bool had_work=bool(owner.separation_job)||
            painter_damaged_fragment::pending(m_EntityAlife.ID());
        if(painter_damaged_fragment::pending(m_EntityAlife.ID()))
            painter_damaged_fragment::cancel(m_EntityAlife.ID());
        if(owner.separation_job)dispose(std::move(owner.separation_job));
        owner.separation_checked_revision=0;
        owner.separation_policy_retry_at=Device.dwTimeGlobal+250;
        if(had_work)Msg("[painter_damaged_fragment] id=%u phase=policy_canceled reason=dismemberment_disabled",
            m_EntityAlife.ID());
        PainterHoldRelease("policy_canceled"); // R63
    }
    slow.part="fragment_poll";
    if(allow_separation){
        if(painter_damaged_fragment::pending(m_EntityAlife.ID())){
            if(painter_damaged_fragment::poll(*m_EntityAlife.cast_stalker(),owner.identity,owner.geometry,
                owner.exact_rig,owner.tissue_textures,m_pPhysicsShell)){owner.separation_cancel_retries=0;return;}
        }
        // A post-submit poll can cancel and erase the batch. Consume its receipt here
        // on the next validated drain even though pending() is now false.
        // R33: a staged split canceled at commit (pose mismatch, resample, swap not ready, spawn timeout...) left the body
        // cut but whole until its next wound re-ran the check (09:13 run, 19908: 35 s). Check again on a fresh snapshot
        // after 1 s, up to six times.
        const char* why=nullptr;u64 canceled_revision=0;
        if(painter_damaged_fragment::take_canceled(m_EntityAlife.ID(),owner.identity,why,canceled_revision))PainterHoldRelease("staged_canceled"); // R63
        if(canceled_revision&&owner.separation_cancel_retries<6){
            ++owner.separation_cancel_retries;
            if(owner.separation_checked_revision>=canceled_revision)owner.separation_checked_revision=canceled_revision-1;
            owner.separation_policy_retry_at=Device.dwTimeGlobal+1000;
            Msg("[painter_damaged_fragment] id=%u phase=cancel_retry_scheduled revision=%llu retry=%u delay_ms=1000 reason=%s t=%u",
                m_EntityAlife.ID(),canceled_revision,owner.separation_cancel_retries,why,Device.dwTimeGlobal);}
    }
    // Off-thread publication: a completed output is first staged. The renderer
    // prepares its geometry on its own thread and the whole wound commits
    // atomically at a later barrier. Until then the output stays unacknowledged,
    // so this owner behaves exactly as if its worker job were still running.
    // A stage belongs to one output; a canceled or replaced output drops it.
    if(owner.stage&&owner.worker->completed()!=owner.stage_out)owner.drop_stage();
    auto out=owner.worker->completed();
    const bool current_candidate=out&&out->stamp.owner==owner.identity&&out->stamp.expected_generation==k->LL_RenderGeometryGeneration();
    if(owner.required_field_hold&&owner.required_field_hold==out&&current_candidate)return;
    if(owner.required_field_hold!=out||!current_candidate)owner.required_field_hold.reset();
    const auto rebuild_status=owner.worker->renderer_rebuild_status();
    if(current_candidate&&rebuild_status.state==gfy_live::RendererRebuildState::refused){
        Msg("[painter_voxel_live] id=%u phase=required_field_held revision=%llu reason=%s",m_EntityAlife.ID(),
            out->stamp.revision,rebuild_status.reason);owner.required_field_hold=out;return;
    }
    // A sever past its connectivity decision is not superseded by later wounds
    // (in-game 2026-09-25: sustained fire canceled every sever at surface_partition).
    // Hold this owner's wound publication until the sever commits or fails; hits
    // keep queuing in the worker and nothing is dropped.
    if(out&&!owner.stage&&allow_separation&&owner.separation_job){
        const auto job_stamp=owner.separation_job->stamp();const char* job_phase=owner.separation_job->progress().phase;
        static const char* const severing[]={"surface_bindings","surface_ownership","surface_partition","physical_partition",
            "physical_partitions","collider_fit","pose_anchors","component_continuation"};
        bool committing=false;for(const char* name:severing)committing=committing||(job_phase&&!std::strcmp(job_phase,name));
        if(committing&&job_stamp.owner==owner.identity&&job_stamp.expected_generation==owner.geometry&&!owner.separation_job->completed()&&
           owner.separation_hold_revision!=job_stamp.revision){owner.separation_hold_revision=job_stamp.revision;
            Msg("[painter_damaged_fragment] id=%u phase=wound_not_held revision=%llu cpu_phase=%s",m_EntityAlife.ID(),job_stamp.revision,job_phase);}
    }
    if(out&&!owner.stage&&owner.sever_revision&&allow_separation&&
       (owner.separation_job||painter_damaged_fragment::pending(m_EntityAlife.ID()))&&
       s32(Device.dwTimeGlobal-owner.sever_hold_started)<2500){
        if(owner.separation_hold_revision!=owner.sever_revision){owner.separation_hold_revision=owner.sever_revision;
            Msg("[painter_damaged_fragment] id=%u phase=wounds_wait_for_sever sever_revision=%llu t=%u",m_EntityAlife.ID(),owner.sever_revision,Device.dwTimeGlobal);}
        out.reset();
    }
    // Sawing: no single hit flags a sever; a check that found the body apart on an older revision re-runs on the newest
    // damage with wound publication held, so the split commits while the shooting goes on.
    if(out&&!owner.stage&&owner.split_imminent_at&&allow_separation&&
       (owner.separation_job||painter_damaged_fragment::pending(m_EntityAlife.ID()))&&
       s32(Device.dwTimeGlobal-owner.split_imminent_at)<1500)out.reset();
    if(out&&!owner.stage){
        // Publication never starts without the commit it needs: the output
        // stays queued and the last published geometry stays until memory
        // recovers (nothing is dropped, no half state).
        gfy_live::MemoryStatus seen;
        if(out->error.empty()&&!out->no_geometry_change&&!publication_admitted(publication_bytes(out->mesh),seen)){
            if(owner.publication_deferred_revision!=out->stamp.revision){owner.publication_deferred_revision=out->stamp.revision;
                Msg("[painter_voxel_live] id=%u phase=publication_deferred revision=%llu reason=low_memory need_mb=%.1f avail_commit_mb=%llu",
                    m_EntityAlife.ID(),out->stamp.revision,publication_bytes(out->mesh)/1048576.0,seen.avail_commit>>20);}
            return;}
    }
    slow.part="stage_or_install";
    int stage_outcome=RuntimeStageOutcome::unavailable;float stage_ms=0.f;
    if(out&&!owner.stage&&out->error.empty()&&out->stamp.owner==owner.identity&&
       out->stamp.expected_generation==k->LL_RenderGeometryGeneration()&&!out->no_geometry_change){
        RuntimeSkinUpdate header;header.owner_cookie=out->stamp.owner.spawn_cookie;header.expected_geometry_generation=out->stamp.expected_generation;header.damage_revision=out->stamp.revision;header.exact_rig=owner.exact_rig;
        CTimer stage_timer;stage_timer.Start();
        IRuntimeGeometryStage* staged=k->LL_StageRuntimeGeometry(header,std::shared_ptr<const gfy_packed::Mesh>(out,&out->mesh),
            "models\\model","wm\\painter_stump_limb",owner.tissue_textures.empty()?nullptr:&owner.tissue_textures,
            owner.identity.spawn_cookie,stage_outcome);
        stage_ms=stage_timer.GetElapsed_ms_f();
        if(staged){owner.stage=staged;owner.stage_out=out;owner.stage_frame=Device.dwFrame;owner.stage_barrier_ms=stage_ms;owner.stage_wait.Start();}
    }
    const bool stageReady=!owner.stage||owner.stage->Ready();
    PainterBurnPublication burnPublication(out&&!out->no_geometry_change&&stageReady?k:nullptr);
    if(out&&(!stageReady||!burnPublication.ready())){out.reset(); // still preparing: nothing visible changes
        // Look again at the next drain (up to three per frame, device.cpp) instead of at the next frame's
        // in_UpdateCL poll: a push from inside a drain would run in this same drain, so it goes to the next list.
        xrCriticalSectionGuard guard(&Device.seqParallelBeforRenderCS);
        auto callback=xr_make_delegate(this,&CCharacterPhysicsSupport::ApplyPainterVoxelLive);
        if(std::find(Device.seqParallelBeforRenderNext.begin(),Device.seqParallelBeforRenderNext.end(),callback)==Device.seqParallelBeforRenderNext.end())
            Device.seqParallelBeforRenderNext.push_back(callback);
        m_painter_voxel_queued=true;}
    if(out){
        CTimer publish_timer;publish_timer.Start();
        float unpack_ms=0.f,install_ms=0.f,ack_ms=0.f;
        const bool offthread=owner.stage!=nullptr;
        for(const auto& impact:out->impacts){
            if(!impact.rejection.empty())
                Msg("[painter_voxel_live] id=%u phase=impact_rejected sequence=%llu reason=%s",m_EntityAlife.ID(),impact.sequence,impact.rejection.c_str());
            // A full source face and barycentric point reconstruct mapping.entry;
            // rest_entry is the later solid march point and must not replace it.
            // Join with hit_accepted for original dimensions, ammo and edge seed.
            const bool mapped=impact.barycentric[0]+impact.barycentric[1]+impact.barycentric[2]>.5;
            if(painter_log_verbose())Msg("[painter_voxel_live] id=%u phase=impact_replay revision=%llu sequence=%llu mapped=%u source_face=%u bary=%.17g,%.17g,%.17g rest_direction=%.17g,%.17g,%.17g solid_entry=%.17g,%.17g,%.17g cutter=%.17g,%.17g,%.17g radius_m=%.17g removed=%u miss=%u",
                m_EntityAlife.ID(),out->stamp.revision,impact.sequence,unsigned(mapped),impact.source_face,
                impact.barycentric[0],impact.barycentric[1],impact.barycentric[2],
                impact.rest_direction.x,impact.rest_direction.y,impact.rest_direction.z,
                impact.rest_entry.x,impact.rest_entry.y,impact.rest_entry.z,
                impact.cutter.x,impact.cutter.y,impact.cutter.z,impact.radius,unsigned(impact.removed),unsigned(impact.miss));
            // gearbody: a hit mapped onto merged gear, or onto the body behind merged gear the ray crossed (the vest under a backpack)
            if(painter_log_verbose()&&(impact.gear_child!=65535||impact.gear_on_ray!=65535))
                Msg("[painter_gear_hit] id=%u sequence=%llu mapped=%s child=%u source_face=%u gear_on_ray=%u gear_in_front_m=%.3f",m_EntityAlife.ID(),
                    impact.sequence,impact.gear_child!=65535?"gear":"body",unsigned(impact.gear_child),impact.source_face,unsigned(impact.gear_on_ray),impact.gear_on_ray_m);
            if(impact.entry_surface.valid||impact.exit_surface.valid)
                if(painter_log_verbose())Msg("[painter_voxel_live] id=%u phase=impact_surfaces sequence=%llu entry_valid=%u exit_valid=%u penetrated=%u entry=%.17g,%.17g,%.17g exit=%.17g,%.17g,%.17g",
                    m_EntityAlife.ID(),impact.sequence,unsigned(impact.entry_surface.valid),unsigned(impact.exit_surface.valid),unsigned(impact.penetrated),
                    impact.entry_surface.rest_point.x,impact.entry_surface.rest_point.y,impact.entry_surface.rest_point.z,
                    impact.exit_surface.rest_point.x,impact.exit_surface.rest_point.y,impact.exit_surface.rest_point.z);
        }
        bool accepted=false;
        if(out->error.empty()&&out->stamp.owner==owner.identity&&out->stamp.expected_generation==k->LL_RenderGeometryGeneration()){
            if(out->no_geometry_change)accepted=true;
            else{RuntimeSkinUpdate update;update.owner_cookie=out->stamp.owner.spawn_cookie;update.expected_geometry_generation=out->stamp.expected_generation;update.damage_revision=out->stamp.revision;update.exact_rig=owner.exact_rig;
                CTimer stage;stage.Start();
                if(offthread){
                    // Same admission again, then wrap, wallmark snapshot and commit in this barrier.
                    accepted=k->LL_InstallStagedRuntimeGeometry(owner.stage,update,owner.identity.spawn_cookie);
                    unpack_ms=owner.stage_barrier_ms;install_ms=stage.GetElapsed_ms_f();
                }else if(stage_outcome==RuntimeStageOutcome::unavailable){
                const bool unpacked=AppendPackedRuntimeChildren(out->mesh,update,"models\\model","wm\\painter_stump_limb",owner.tissue_textures.empty()?nullptr:&owner.tissue_textures);
                unpack_ms=stage.GetElapsed_ms_f();stage.Start();
                if(unpacked)accepted=k->LL_InstallRuntimeGeometry(update,owner.identity.spawn_cookie);
                install_ms=stage.GetElapsed_ms_f();
                }else unpack_ms=stage_ms; // staging refused it (logged): the synchronous path's outcome
                }
        }
        RuntimeStageTiming staged_timing;u32 staged_frames=0;float staged_wait_ms=0.f;
        if(offthread){staged_timing=owner.stage->Timing();staged_frames=Device.dwFrame-owner.stage_frame;staged_wait_ms=owner.stage_wait.GetElapsed_ms_f();owner.drop_stage();}
        CTimer ack_timer;ack_timer.Start();
        bool required=bool(out->mesh.analytic_channels)||bool(out->mesh.skin_ownership);
        for(const auto&child:out->mesh.children)required=required||child.skin_clip||child.skin_owned;
        if(!accepted&&required&&!out->burn_surface_prime&&out->error.empty()&&out->stamp.owner==owner.identity&&
           out->stamp.expected_generation==k->LL_RenderGeometryGeneration()){
            // No fallback to the old dense geometry path: a refused low-poly wound is a defect to fix.
            // The exact candidate is held (damage kept, never replayed or discarded) and reported loudly.
            Msg("! [painter_voxel_live] id=%u phase=LOWPOLY_REFUSED revision=%llu wound_not_shown=1 see_runtime_geometry_reject_above=1",
                m_EntityAlife.ID(),out->stamp.revision);
            body_hit_outcomes::output("publication_held","lowpoly_native_gate",owner.identity.spawn_cookie,*out);
            owner.required_field_hold=out;
            return;
        }
        owner.worker->acknowledge_output(out,accepted,k->LL_RenderGeometryGeneration());
        burnPublication.release();
        if(accepted&&!out->no_geometry_change)painter_burn_host::refresh(m_EntityAlife,owner.worker);
        if(out->burn_surface_prime)owner.burn_prime_queued=false;
        if(owner.burn_prime_wanted && accepted && !out->no_geometry_change && out->mesh.mask_volume) {
            bool any=false,all=true;for(const auto& c:out->mesh.children)if(!c.material&&!c.indices.empty()){any=true;all=all&&c.skin_clip;}
            owner.burn_prime_ready=any&&all;
        }
        if(!out->accessory_skipped.empty())Msg("[painter_voxel_live] id=%u phase=accessory_skipped revision=%llu accepted=%u %s",
            m_EntityAlife.ID(),out->stamp.revision,unsigned(accepted),out->accessory_skipped.c_str()); // ownersafe
        ack_ms=ack_timer.GetElapsed_ms_f();
        state(accepted?(out->no_geometry_change?3:2):-1,m_EntityAlife.ID(),int(out->stamp.revision));
        if(accepted&&!out->no_geometry_change&&out->mask_only){u64 masked=0;for(const auto&c:out->mesh.children)masked+=c.skin_clip?1:0;
            Msg("[painter_voxel_live] id=%u phase=mask_only revision=%llu state=%s mask_volume=%u masked_skin_children=%llu triangles=%llu worker_ms=%.3f",
                m_EntityAlife.ID(),out->stamp.revision,out->mask_only==1?"mask":"full_after_sever",unsigned(out->mesh.mask_volume),masked,u64(out->triangles),out->worker_ms);}
        if(accepted&&out->mesh.reduced_skin_vertices)
            Msg("[painter_voxel_live] id=%u phase=skin_reduction vertices=%llu max_discarded=%.6f",m_EntityAlife.ID(),u64(out->mesh.reduced_skin_vertices),out->mesh.max_discarded_skin_weight);
        // A resource failure (out_of_memory/low_memory) is acknowledged as a
        // no-op: every event of the batch was reported not applied above.
        const bool not_applied=!out->resource_failure.empty();
        Msg("[painter_voxel_live] id=%u phase=%s revision=%llu cookie=%llu expected_generation=%llu generation=%llu events=%llu triangles=%llu worker_ms=%.3f reason=%s t=%u tc=%u",m_EntityAlife.ID(),accepted?(out->no_geometry_change?(not_applied?"not_applied":"no_op"):"committed"):"rejected",out->stamp.revision,out->stamp.owner.spawn_cookie,out->stamp.expected_generation,k->LL_RenderGeometryGeneration(),u64(out->batch_events),u64(out->triangles),out->worker_ms,not_applied?out->resource_failure.c_str():out->error.empty()?(accepted?"none":"native_gate_or_stale_generation"):out->error.c_str(),Device.dwTimeGlobal,Device.dwTimeContinual);
        if(accepted&&!out->no_geometry_change){
            if(auto* object=m_EntityAlife.cast_stalker())
                painter_cranial_remnants::committed(*object,out->joints,out->impacts,out->cranial,out->stamp.revision);
            // End-to-end: engine hit accepted -> this commit barrier (the wound becomes visible this frame).
            const auto now=std::chrono::steady_clock::now();double oldest=-1,newest=-1;
            for(const auto&impact:out->impacts)for(const auto&h:owner.hit_times)if(h.first==impact.sequence){
                const double ms=std::chrono::duration<double,std::milli>(now-h.second).count();oldest=oldest<0?ms:_max(oldest,ms);newest=newest<0?ms:_min(newest,ms);break;}
            u64 cavity=0,masked=0;for(const auto&c:out->mesh.children){if(c.material)cavity+=c.indices.size()/3;masked+=c.skin_clip?1:0;}
            if(painter_log_verbose())Msg("[painter_voxel_live] id=%u phase=hit_to_visible revision=%llu path=%s oldest_hit_ms=%.1f newest_hit_ms=%.1f worker_ms=%.1f events=%llu triangles=%llu cavity_triangles=%llu masked_skin_children=%llu channel_refusal=%s",
                m_EntityAlife.ID(),out->stamp.revision,out->mesh.analytic_channels?"lowpoly_channel":(out->complete_geometry_rebuild?"fallback_rebuild":"legacy"),
                oldest,newest,out->worker_ms,u64(out->batch_events),u64(out->triangles),cavity,masked,out->analytic_refusal.empty()?"none":out->analytic_refusal.c_str());
            while(!owner.hit_times.empty()&&owner.hit_times.front().first<=out->stamp.revision)owner.hit_times.pop_front();
        }
        if(out->heavy_join||out->memory_deferrals||not_applied)
            Msg("[painter_voxel_live] id=%u phase=memory revision=%llu join_estimate_mb=%.1f join_quads=%llu join_faces=%llu join_triangles=%llu join_body_triangles=%llu join_regions=%llu heavy=%u deferrals=%u",
                m_EntityAlife.ID(),out->stamp.revision,out->join_estimate/1048576.0,u64(out->join_quads),u64(out->join_faces),u64(out->join_triangles),u64(out->join_body_triangles),u64(out->join_regions),unsigned(out->heavy_join),unsigned(out->memory_deferrals));
        // Main-thread cost. mode=offthread: unpack_ms is the staging barrier and
        // install_ms the commit barrier (different frames); total_ms adds both.
        if(painter_log_verbose())Msg("[painter_voxel_live] id=%u phase=publication_timing revision=%llu unpack_ms=%.3f install_ms=%.3f acknowledge_ms=%.3f total_ms=%.3f mode=%s",m_EntityAlife.ID(),out->stamp.revision,unpack_ms,install_ms,ack_ms,publish_timer.GetElapsed_ms_f()+(offthread?unpack_ms:0.f),offthread?"offthread":"sync");
        if(offthread)
            if(painter_log_verbose())Msg("[painter_voxel_live] id=%u phase=publication_offthread revision=%llu accepted=%u frames=%u wait_ms=%.3f queue_ms=%.3f prepare_ms=%.3f build_ms=%.3f buffers_ms=%.3f faces_ms=%.3f projection_ms=%.3f stage_ms=%.3f admit_ms=%.3f wrap_ms=%.3f wallmarks_ms=%.3f commit_ms=%.3f verify_ms=%.3f children=%u vertices=%u triangles=%u shaders_reused=%u shaders_compiled=%u marks=%u verify=%d",
                m_EntityAlife.ID(),out->stamp.revision,unsigned(accepted),staged_frames,staged_wait_ms,staged_timing.queue_ms,staged_timing.run_ms,
                staged_timing.build_ms,staged_timing.buffers_ms,staged_timing.faces_ms,staged_timing.projection_ms,staged_timing.stage_ms,
                staged_timing.admit_ms,staged_timing.wrap_ms,staged_timing.wallmarks_ms,staged_timing.commit_ms,staged_timing.verify_ms,
                staged_timing.children,staged_timing.vertices,staged_timing.triangles,staged_timing.shader_reused,staged_timing.shader_compiled,
                staged_timing.marks,staged_timing.verify_result);
        if(painter_log_verbose())Msg("[painter_voxel_live] id=%u phase=worker_timing revision=%llu removal_ms=%.3f boundary_ms=%.3f exterior_ms=%.3f join_ms=%.3f pack_ms=%.3f",
            m_EntityAlife.ID(),out->stamp.revision,out->worker_phases.removal_ms,out->worker_phases.boundary_ms,
            out->worker_phases.exterior_ms,out->worker_phases.join_ms,out->worker_phases.pack_ms);
        if(!out->geometry_retry_reason.empty())
            Msg("[painter_voxel_live] id=%u phase=rough_failed_smooth_retry revision=%llu accepted=%u changed=%u reason=%s",m_EntityAlife.ID(),out->stamp.revision,unsigned(accepted),unsigned(accepted&&!out->no_geometry_change),out->geometry_retry_reason.c_str());
        if(out->geometry_batch_splits)
            Msg("[painter_voxel_live] id=%u phase=geometry_batch_isolation revision=%llu splits=%llu deferred=%llu reason=%s",m_EntityAlife.ID(),out->stamp.revision,u64(out->geometry_batch_splits),u64(out->deferred_events),out->geometry_batch_reason.c_str());
        if(accepted&&!out->no_geometry_change)
            if(painter_log_verbose())Msg("[painter_voxel_live] id=%u phase=wound_edge_shape revision=%llu irregular=%u",m_EntityAlife.ID(),out->stamp.revision,unsigned(out->irregular_edges));
        if(accepted&&(out->numerical_open_edges||out->interior_sheet_junctions))
            Msg("[painter_voxel_live] id=%u phase=geometry_precision residual_edges=%llu residual_length_m=%.12f interior_sheets=%llu",m_EntityAlife.ID(),u64(out->numerical_open_edges),out->numerical_open_length_m,u64(out->interior_sheet_junctions));
        if(!accepted)return;owner.geometry=k->LL_RenderGeometryGeneration();
        if(out->burn_surface_prime){
            owner.separation_checked_revision=std::max(owner.separation_checked_revision,out->stamp.revision);
            if(pool)pool->dispatch(owner.worker,owner.geometry);
            return;
        }
        // A queued or rejected cut cannot advance Butcher's joint damage.
        // Death-only eligibility and thresholds remain Lua policy.
        ::luabind::functor<bool> joint_damage;
        if(!out->joints.empty()&&ai().script_engine().functor("_G.CPainterVoxelJointDamage",joint_damage)){
            auto receipt=::luabind::newtable(ai().script_engine().lua());auto rows=::luabind::newtable(ai().script_engine().lua());
            string64 cookie;xr_sprintf(cookie,"%llu",owner.identity.spawn_cookie);
            receipt["revision"]=double(out->stamp.revision);receipt["cookie"]=cookie;unsigned index=1;
            for(const auto&joint:out->joints){auto row=::luabind::newtable(ai().script_engine().lua());
                row["bone"]=k->LL_BoneName_dbg(joint.bone);row["original_m3"]=joint.original_m3;row["removed_m3"]=joint.removed_m3;
                row["original_bone_m3"]=joint.original_bone_m3;row["removed_bone_m3"]=joint.removed_bone_m3;rows[index++]=row;}
            receipt["joints"]=rows;const bool recorded=joint_damage(m_EntityAlife.lua_game_object(),receipt);
            if(painter_log_verbose())Msg("[painter_voxel_live] id=%u phase=joint_damage revision=%llu regions=%u recorded=%u",m_EntityAlife.ID(),out->stamp.revision,index-1,unsigned(recorded));
        }
        // One committed output owns the pending receipt batch. Subsequent
        // publication/separation waits for admission; accepted effects are never
        // evicted and retries do not repeat geometry or death callbacks.
        slow.part="effects";
        VERIFY(owner.committed_effects.empty());
        owner.committed_effects.assign(out->impacts);
        for(const auto&impact:out->impacts)if(impact.sever){owner.sever_revision=out->stamp.revision;owner.sever_hold_started=Device.dwTimeGlobal;}
        painter_explosions::accepted(m_EntityAlife,*out);
        if(!painter_committed_effects::drain(*m_EntityAlife.cast_stalker(),owner.committed_effects,true))return;
    }
    slow.part="separation";
    if(allow_separation&&owner.separation_job&&!painter_explosions::first_incomplete(m_EntityAlife)){
        const auto queued_stamp=owner.separation_job->stamp();
        const auto progress=owner.separation_job->progress();
        auto completed=owner.separation_job->completed();
        std::shared_ptr<painter_flight::Group> flight;
        if((!completed||(completed->error.empty()&&!completed->no_separation))&&owner.flight_attempted_revision!=queued_stamp.revision&&
           queued_stamp.owner==owner.identity&&queued_stamp.expected_generation==owner.geometry){
            if(auto early=owner.separation_job->preview();early&&k->LL_FlightPreviewAllowed()){
                completed=std::move(early);owner.flight_attempted_revision=queued_stamp.revision;
                flight=std::make_shared<painter_flight::Group>();flight->job=owner.separation_job;
            }
        }
        const bool stale_same_owner=queued_stamp.owner==owner.identity&&queued_stamp.expected_generation!=owner.geometry;
        if(stale_same_owner&&!completed){
            // Newer wounds were published while this check runs: let it finish (a cut found on an older revision is
            // still a cut) instead of restarting it on every hit - unless the newest publication carries a sever: its
            // own check must not wait behind a stale one (a stale result is never committed anyway).
            if(owner.sever_revision>queued_stamp.revision){
                Msg("[painter_damaged_fragment] id=%u phase=cpu_superseded revision=%llu expected_generation=%llu generation=%llu cpu_phase=%s elapsed_ms=%.3f cpu_result=pending reason=sever_on_newer_revision t=%u",
                    m_EntityAlife.ID(),queued_stamp.revision,queued_stamp.expected_generation,owner.geometry,progress.phase,progress.elapsed_ms,Device.dwTimeGlobal);
                owner.separation_job->cancel();dispose(std::move(owner.separation_job));}
        }else if(stale_same_owner&&completed->error.empty()&&!completed->no_separation){
            Msg("[painter_damaged_fragment] id=%u phase=split_imminent revision=%llu generation=%llu worker_ms=%.1f t=%u",
                m_EntityAlife.ID(),queued_stamp.revision,owner.geometry,completed->worker_ms,Device.dwTimeGlobal);
            owner.split_imminent_at=Device.dwTimeGlobal;owner.separation_policy_retry_at=Device.dwTimeGlobal;
            dispose(std::move(owner.separation_job));
        }else if(!(queued_stamp.owner==owner.identity)||queued_stamp.expected_generation!=owner.geometry){
            // Later accepted geometry makes this exact snapshot unusable. Stop
            // at the next CPU phase boundary and queue the newest accepted mask.
            // Ordinary hit publication continues while either job is running.
            const char* cpu_result=!completed?"pending":!completed->error.empty()?completed->error.c_str():
                completed->no_separation?"still_connected":"prepared";
            Msg("[painter_damaged_fragment] id=%u phase=cpu_superseded revision=%llu expected_generation=%llu generation=%llu cpu_phase=%s elapsed_ms=%.3f cpu_result=%s reason=snapshot_identity_or_generation_changed t=%u",
                m_EntityAlife.ID(),queued_stamp.revision,queued_stamp.expected_generation,
                owner.geometry,progress.phase,progress.elapsed_ms,cpu_result,Device.dwTimeGlobal);
            owner.separation_job->cancel();dispose(std::move(owner.separation_job));
        }else if(auto result=completed){
            dispose(std::move(owner.separation_job));
            owner.split_imminent_at=0; // the newest damage was checked
            if(result->stamp.owner==owner.identity&&
               result->stamp.expected_generation==owner.geometry&&
               result->stamp.revision==owner.separation_checked_revision&&
               !m_EntityAlife.g_Alive()&&!painter_damaged_fragment::pending(m_EntityAlife.ID())){
                if(result->error.empty())owner.separation_memory_retries=0;
                if(!result->error.empty()){PainterHoldRelease("separation_refused");
                    Msg("[painter_damaged_fragment] id=%u phase=cpu_refused revision=%llu cpu_phase=%s worker_ms=%.3f reason=%s",
                        m_EntityAlife.ID(),result->stamp.revision,progress.phase,result->worker_ms,result->error.c_str());
                    // Memory is not a property of this corpse: retry the same
                    // accepted snapshot later (bounded), after the governor ran.
                    if((result->error=="out_of_memory"||result->error=="low_memory")&&owner.separation_memory_retries<3&&result->stamp.revision){
                        ++owner.separation_memory_retries;owner.separation_checked_revision=result->stamp.revision-1;
                        owner.separation_policy_retry_at=Device.dwTimeGlobal+2000;govern(true);
                        Msg("[painter_damaged_fragment] id=%u phase=cpu_retry_scheduled revision=%llu retry=%u delay_ms=2000",m_EntityAlife.ID(),result->stamp.revision,owner.separation_memory_retries);}}
                else if(result->no_separation&&(PainterHoldRelease("no_sever"),true))
                    Msg("[painter_damaged_fragment] id=%u phase=still_connected revision=%llu original_components=%u remaining_components=%u largest_substantial_split=%u worker_ms=%.3f t=%u tc=%u",
                        m_EntityAlife.ID(),result->stamp.revision,result->original_components,
                        result->remaining_components,result->largest_substantial_split,result->worker_ms,Device.dwTimeGlobal,Device.dwTimeContinual);
                else {const char* why="native_adapter";
                    const bool staged=painter_damaged_fragment::submit_cpu(
                        *m_EntityAlife.cast_stalker(),owner.identity,*result,
                        owner.original_children,owner.separation_input,
                        owner.exact_rig,owner.tissue_textures,
                        m_pPhysicsShell,why,flight);
                    if(!staged&&flight){owner.separation_job=flight->job;return;}
                    if(!staged)PainterHoldRelease("native_refused"); // R63: a retry runs on the moving body, as before
                    Msg("[painter_damaged_fragment] id=%u phase=%s revision=%llu reason=%s worker_ms=%.1f t=%u tc=%u lowpoly=%s",
                        m_EntityAlife.ID(),staged?"staged_hidden":"native_refused",
                        result->stamp.revision,why,result->worker_ms,Device.dwTimeGlobal,Device.dwTimeContinual,result->lowpoly_summary.c_str());
                    if(staged){
                        // Start render preparation at this already-validated barrier. The poll
                        // remains nonblocking and retains all fragment publication checks.
                        dispose(std::move(result));dispose(std::move(completed));
                        slow.part="fragment_poll_submitted";
                        painter_damaged_fragment::poll(*m_EntityAlife.cast_stalker(),owner.identity,owner.geometry,
                            owner.exact_rig,owner.tissue_textures,m_pPhysicsShell);
                        // A true result can replace the owner/shell; false can cancel the batch.
                        // Never touch the captured owner again. Existing exit requeue applies.
                        return;
                    }
                    // A corpse that is still settling (moved), a newer wound (stale_generation) or a busy frame only
                    // invalidates this snapshot: run the separation again on a fresh one, up to four times, 1.5 s apart.
                    if(!staged&&result->stamp.revision&&owner.separation_native_retries<4&&why&&
                       (!strcmp(why,"moved")||!strcmp(why,"stale_generation")||!strcmp(why,"shell_changed")||
                        !strcmp(why,"not_barrier")||!strcmp(why,"physics_busy")||!strcmp(why,"busy"))){
                        ++owner.separation_native_retries;owner.separation_checked_revision=result->stamp.revision-1;
                        owner.separation_policy_retry_at=Device.dwTimeGlobal+1500;
                        Msg("[painter_damaged_fragment] id=%u phase=native_retry_scheduled revision=%llu retry=%u reason=%s t=%u",
                            m_EntityAlife.ID(),result->stamp.revision,owner.separation_native_retries,why,Device.dwTimeGlobal);}
                }
            }else Msg("[painter_damaged_fragment] id=%u phase=cpu_stale revision=%llu expected_generation=%llu generation=%llu checked_revision=%llu reason=publication_identity_or_lifecycle",
                m_EntityAlife.ID(),result->stamp.revision,result->stamp.expected_generation,
                owner.geometry,owner.separation_checked_revision);
            dispose(std::move(result));
        }else if(progress.elapsed_ms-owner.separation_progress_reported_ms>=5000){
            owner.separation_progress_reported_ms=progress.elapsed_ms;
            Msg("[painter_damaged_fragment] id=%u phase=cpu_pending revision=%llu cpu_phase=%s component=%u bone=%u elapsed_ms=%.3f queue_ms=%.3f phase_ms=%.3f",
                m_EntityAlife.ID(),queued_stamp.revision,progress.phase,progress.component,
                unsigned(progress.bone),progress.elapsed_ms,progress.queue_ms,progress.phase_ms);
        }
        dispose(std::move(completed));
    }
    // Separation runs again, low-poly only: prepare() refuses any snapshot without published low-poly geometry
    // (LowPolyPieces.h); the dense surface partition/reduction is never run.
    if(allow_separation&&!painter_explosions::first_incomplete(m_EntityAlife)&&owner.separation_ready&&!owner.separation_job&&
       s32(Device.dwTimeGlobal-owner.separation_policy_retry_at)>=0&&
       !m_EntityAlife.g_Alive()&&!painter_damaged_fragment::pending(m_EntityAlife.ID())){
        auto snapshot=owner.worker->capture_damage();
        if(snapshot&&snapshot->stamp().revision>owner.separation_checked_revision&&
           snapshot->stamp().expected_generation==owner.geometry&&
           snapshot->stamp().owner==owner.identity){
            auto input=owner.separation_input;input.minimum_piece_m3=1.5e-6;
            if(original_physical_parents(k,m_pPhysicsShell,input.physical_parents)){
                auto job=gfy_live::prepare_separation_async(snapshot,std::move(input));
                if(job){owner.separation_checked_revision=snapshot->stamp().revision;
                    owner.separation_progress_reported_ms=0;
                    owner.separation_job=std::move(job);
                    Msg("[painter_damaged_fragment] id=%u phase=cpu_queued revision=%llu t=%u tc=%u",
                        m_EntityAlife.ID(),owner.separation_checked_revision,Device.dwTimeGlobal,Device.dwTimeContinual);
                }else{
                    owner.separation_policy_retry_at=Device.dwTimeGlobal+250;
                    Msg("[painter_damaged_fragment] id=%u phase=cpu_queue_refused revision=%llu reason=%s",
                        m_EntityAlife.ID(),snapshot->stamp().revision,gfy_live::last_failure());
                }
            }else{
                owner.separation_checked_revision=snapshot->stamp().revision;
                Msg("[painter_damaged_fragment] id=%u phase=cpu_refused revision=%llu reason=original_physical_tree",
                    m_EntityAlife.ID(),owner.separation_checked_revision);
            }
        }
    }
    // Capture generation only at dispatch. A returned result is never restamped.
    if(k->LL_RenderGeometryGeneration()!=owner.geometry){state(-1,m_EntityAlife.ID(),int(owner.sequence));{auto* npc=m_EntityAlife.cast_stalker();const bool stump=npc&&painter_piece_continuation::owns(*npc,0);
        Msg(stump?"[painter_voxel_live] id=%u phase=handed_to_stump":"[painter_voxel_live] id=%u phase=rejected reason=external_geometry_change",m_EntityAlife.ID());}
        CancelPainterVoxelLive();return;}
    pool->dispatch(owner.worker,owner.geometry);
}
bool CCharacterPhysicsSupport::PainterCanSwapFittedShell(CPhysicsShell* expected) const
{
    if(m_eType!=etStalker||m_EntityAlife.g_Alive()||m_EntityAlife.getDestroy()||
       !expected||m_pPhysicsShell!=expected||!expected->isFullActive()||
       !physics_world()||physics_world()->Processing()||Device.Paused())return false;
    // A held weapon owns moved CODEGeom pointers. Its own live shell must be
    // available for the existing native release path before replacing ours.
    // A corpse still holding its weapon carries the weapon's collision geoms on its hand element; the swap
    // releases them (PainterReleaseWeaponCollision), so only their ownership is checked here.
    if(m_weapon_attach_bone)
        return !m_weapon_geoms.empty()&&expected->get_Element(m_weapon_attach_bone->m_SelfID)==m_weapon_attach_bone;
    return m_weapon_geoms.empty()&&!m_active_item_obj;
}
void CCharacterPhysicsSupport::PainterReleaseWeaponCollision()
{
    if(!m_weapon_attach_bone)return;
    CPhysicsShell* weapon_shell=m_active_item_obj?m_active_item_obj->PPhysicsShell():nullptr;
    CPhysicsElement* weapon_root=weapon_shell&&weapon_shell->isActive()?weapon_shell->get_ElementByStoreOrder(0):nullptr;
    if(weapon_root&&weapon_root->numberOfGeoms()&&weapon_root->geometry(0)){RemoveActiveWeaponCollision();return;}
    for(CODEGeom* g:m_weapon_geoms){m_weapon_attach_bone->remove_geom(g);g->destroy();xr_delete(g);}
    m_weapon_geoms.clear();m_weapon_attach_bone=0;m_active_item_obj=0;bone_fix_clear();
    Msg("[painter_damaged_fragment] id=%u phase=weapon_collision_released held=1",m_EntityAlife.ID());
}
#include "PainterFleshRay.h"
void CCharacterPhysicsSupport::PainterSetFlesh(const std::vector<PHFittedBone>& flesh,CPhysicsShell* shell,const std::vector<PHFittedBone>* gear)
{
    const auto* published=m_painter_flesh_list.load(std::memory_order_acquire);
    auto& slot=published==&m_painter_flesh[0]?m_painter_flesh[1]:m_painter_flesh[0];
    m_painter_flesh_shell.store(nullptr,std::memory_order_release);
    slot=flesh;
    if(gear)slot.insert(slot.end(),gear->begin(),gear->end()); // gearbody: the merged gear the body holds counts as flesh
    m_painter_flesh_list.store(&slot,std::memory_order_release);
    m_painter_flesh_shell.store(shell,std::memory_order_release);
}
int CCharacterPhysicsSupport::PainterFleshOnRay(const Fvector& start,const Fvector& dir,float range,u16 bone)
{
    CPhysicsShell* shell=m_painter_flesh_shell.load(std::memory_order_acquire);
    if(!shell||shell!=m_EntityAlife.PPhysicsShell())return 0; // no separation yet, or another shell: no rule
    const auto* flesh=m_painter_flesh_list.load(std::memory_order_acquire);
    IKinematics* K=smart_cast<IKinematics*>(m_EntityAlife.Visual());
    if(!flesh||flesh->empty()||!K)return 0;
    return painter_flesh_on_ray(*K,m_EntityAlife.XFORM(),*flesh,start,dir,range,bone,.03f);
}
void CCharacterPhysicsSupport::PainterSwapFittedShell(CPhysicsShell* prepared,
    CPhysicsShell* expected,u64 visible_bones)
{
    VERIFY(PainterCanSwapFittedShell(expected));
    VERIFY(prepared&&prepared->isActive()&&!prepared->isEnabled());
    auto* k=smart_cast<IKinematics*>(m_EntityAlife.Visual());VERIFY(k);
    // Stop the death motion while its old shell and callback owners still exist.
    // Retain the motion object for any unfinished blend callback on this visual.
    if(m_interactive_motion)m_interactive_motion->destroy();
    // A fitted stump replaces the shell, not the dead character's configured physics policy.
    // Reload the same model sleep thresholds; do not copy old-body sleep history or callback data.
    SAllDDOParams disable_params;
    disable_params.Load(k->LL_UserData());
    prepared->set_DisableParams(disable_params);
    if(!DoCharacterShellCollide())prepared->SetDeadBody();
    if(IsGameTypeSingle()){
        prepared->SetPrefereExactIntegration();
        prepared->SetDeadBody();
    }else prepared->SetIgnoreDynamic();
    prepared->SetIgnoreSmall();
    if(m_weapon_attach_bone)PainterReleaseWeaponCollision();
#ifdef PAINTER_ACTIVE_RAGDOLL_NATIVE
    // ragdollswap (2026-11-25): the active ragdoll holds the old shell and its joints. Let it go while they still exist:
    // destroy_physics_shell below frees them, and the controller's next detach (level unload when a save loads, object
    // destroy, visual change) wrote to the freed joints - a crash after every sever. A fitted stump is not a whole stalker:
    // it is not attached again.
    m_active_ragdoll_controller.detach();
#endif
    expected->EnabledCallbacks(FALSE);
    m_pPhysicsShell=prepared;
    // Collision ownership is encoded by the prepared shell. The partitioned
    // render packet may still have genuine cross-bone skin weights, each with
    // an explicit pose link captured at the same commit barrier.
    k->LL_SetBonesVisible(u64(-1));
    prepared->EnabledCallbacks(TRUE);
    prepared->EnableCollision();prepared->Enable();
    // The old shell lies exactly on its replacement: its deactivation touch step must not collide the two.
    expected->DisableCollision();
    destroy_physics_shell(expected);
    prepared->SetCallbacks();prepared->EnabledCallbacks(TRUE);
    // get_Element consults the shared bone callbacks: bind the replacement first, after the old shell is gone.
    painter_limb_torso_collision(*prepared,*k);
    if(m_painter_voxel_live){
        auto& owner=*m_painter_voxel_live;
        owner.shell=prepared;
        owner.lifecycle.death_shell=reinterpret_cast<std::uintptr_t>(prepared);
        owner.geometry=k->LL_RenderGeometryGeneration();
        owner.worker->cancel(); // Old unpartitioned damage state cannot regrow a severed part.
    }
}
bool CCharacterPhysicsSupport::PainterVoxelHandlesBone(u16 bone) const
{
    if(auto* source=m_EntityAlife.cast_stalker())
        if(painter_piece_continuation::owns(*source,bone))return true;
    return m_painter_voxel_live && m_painter_voxel_live->worker->supports_bone(bone);
}
bool painter_voxel_handles_wound(u16 id,u16 bone)
{
    if(!g_pGameLevel)return false;
    auto* object=smart_cast<CPhysicsShellHolder*>(Level().Objects.net_Find(id));
    if(object&&painter_piece_continuation::owns(*object,bone))return true;
    auto* npc=smart_cast<CAI_Stalker*>(object);
    return npc&&npc->character_physics_support()&&npc->character_physics_support()->PainterVoxelHandlesBone(bone);
}
void CCharacterPhysicsSupport::ApplyPainterPieceContinuation()
{
    if(auto* source=m_EntityAlife.cast_stalker())painter_piece_continuation::poll(*source);
}
// Called by the burn bridge on the main owner lane, after a real accepted surface contact.
bool CCharacterPhysicsSupport::BindExistingBurnReceiver()
{
    if (!m_painter_voxel_live || m_EntityAlife.getDestroy()) return false;
    auto& owner=*m_painter_voxel_live;
    if (!owner.worker || !owner.visual || smart_cast<IKinematics*>(m_EntityAlife.Visual())!=owner.visual ||
        owner.visual->LL_RenderGeometryGeneration()!=owner.geometry) return false;
    painter_burn_host::bind(m_EntityAlife,owner.identity,owner.worker,false,true);
    return true;
}
bool CCharacterPhysicsSupport::PrimePainterBurnGeometry(u64 spawn_cookie)
{
    using namespace painter_voxel_native;
    if (!spawn_cookie || !m_painter_voxel_live || !pool || m_EntityAlife.getDestroy()) return false;
    auto& owner=*m_painter_voxel_live;
    if (owner.identity.spawn_cookie!=spawn_cookie || !owner.worker || !owner.visual ||
        smart_cast<IKinematics*>(m_EntityAlife.Visual())!=owner.visual ||
        owner.visual->LL_RenderGeometryGeneration()!=owner.geometry) return false;
    if (owner.separation_job||owner.stage||owner.sever_revision>owner.separation_checked_revision)return false;
    auto current=owner.worker->published_burn_surface();
    if(!current)return false;
    if(!current->needs_burn_prime()){owner.burn_prime_ready=true;return true;}
    owner.burn_prime_ready=false;
    owner.burn_prime_wanted=true;
    if (!owner.burn_prime_queued && pool->prime_burn_geometry(owner.worker,owner.geometry,owner.sequence+1)) {
        ++owner.sequence;
        owner.burn_prime_queued=true;
    }
    return owner.burn_prime_queued;
}

bool PainterPrimeBurnGeometry(u16 id,u64 spawn_cookie)
{
    if (!g_pGameLevel) return false;
    auto* object=Level().Objects.net_Find(id);
    if(auto* holder=smart_cast<CPhysicsShellHolder*>(object))
        if(painter_piece_continuation::prime_burn_geometry(*holder,spawn_cookie))return true;
    auto* npc=smart_cast<CAI_Stalker*>(object);
    return npc && npc->character_physics_support() &&
        npc->character_physics_support()->PrimePainterBurnGeometry(spawn_cookie);
}
bool painter_voxel_live_bind(u16 id)
{
    if(!g_pGameLevel)return false;auto*npc=smart_cast<CAI_Stalker*>(Level().Objects.net_Find(id));return npc&&npc->character_physics_support()&&npc->character_physics_support()->BindPainterVoxelLive();
}
// The player's target (aimed at or shot, any distance): binds past the body-data budget while system memory allows.
bool painter_voxel_live_bind_target(u16 id)
{
    if(!g_pGameLevel)return false;auto*npc=smart_cast<CAI_Stalker*>(Level().Objects.net_Find(id));return npc&&npc->character_physics_support()&&npc->character_physics_support()->BindPainterVoxelLive(true);
}
// Releases an unwounded binding (false: the NPC carries a wound or a hit in flight, and stays bound).
bool painter_voxel_live_unbind(u16 id)
{
    if(!g_pGameLevel)return false;auto*npc=smart_cast<CAI_Stalker*>(Level().Objects.net_Find(id));return npc&&npc->character_physics_support()&&npc->character_physics_support()->UnbindPainterVoxelLive();
}
int painter_voxel_live_state(u16 id)
{
    if(!g_pGameLevel)return -1;auto*npc=smart_cast<CAI_Stalker*>(Level().Objects.net_Find(id));
    return npc&&npc->character_physics_support()?npc->character_physics_support()->PainterVoxelLiveState():-1;
}
bool painter_fragment_ignore_source(u16 fragment_id,u16 source_id)
{
    if(!g_pGameLevel||fragment_id==source_id)return false;
    auto* fragment=smart_cast<CPhysicsShellHolder*>(Level().Objects.net_Find(fragment_id));
    auto* source=smart_cast<CPhysicsShellHolder*>(Level().Objects.net_Find(source_id));
    if(!fragment||!source||fragment->getDestroy()||source->getDestroy())return false;
    CPhysicsShell* shell=fragment->PPhysicsShell();CPhysicsShell* body=source->PPhysicsShell();
    if(!shell||!body||shell==body||!shell->isActive()||!body->isActive())return false;
    if(!body->IsGroupObject())body->RegisterToCLGroup(RegisterGroup());
    shell->RegisterToCLGroup(body->GetCLGroup());
    return true;
}

// ---- R63 sever hold -------------------------------------------------------------------------------------------------------
int painter_sever_hold=1;      // console painter_sever_hold 0/1
int painter_sever_hold_ms=500; // console painter_sever_hold_ms: longest frozen wait (real time) before the shell starts anyway
bool CCharacterPhysicsSupport::PainterSeverHoldWanted(const SHit& H)
{
    if(!painter_sever_hold||m_eType!=etStalker||!m_painter_voxel_live)return false;
    auto&o=*m_painter_voxel_live;bool want=false;
    // rpgblast: the killing hit of a blast with planned tears is its wave hit or one of its fragments (the explosive is their weapon)
    const bool blast_kill=o.hold_blast_explosive!=u16(-1)&&o.hold_blast_explosive==H.weaponID&&Device.dwTimeGlobal-o.hold_blast_time<=1000;
    if(H.type()==ALife::eHitTypeExplosion)want=blast_kill;
    else want=blast_kill||(o.hold_hit_time&&Device.dwTimeGlobal-o.hold_hit_time<=100);
    o.hold_hit_time=0;o.hold_blast_explosive=u16(-1); // one death, one decision
    return want&&gfy_live::damage_mode_value()!=1; // modes: Bullet holes only has no sever to wait for
}
void CCharacterPhysicsSupport::PainterHoldRelease(const char* why,CPhysicsShell* hit_shell)
{
    if(!m_painter_hold)return;
    m_painter_hold=false;
    if(m_pPhysicsShell&&m_pPhysicsShell->isActive()){m_pPhysicsShell->Enable();m_pPhysicsShell->set_LinearVel(m_painter_hold_velocity);}
    if(m_painter_hold_hit_valid){
        const SHit& h=m_painter_hold_hit;CPhysicsShell* target=hit_shell?hit_shell:m_pPhysicsShell;
        if(target&&target->isActive()){target->Enable();target->applyHit(h.bone_space_position(),h.direction(),h.phys_impulse(),h.bone(),h.type());}
    }
    m_painter_hold_hit_valid=false;
    Msg("[painter_voxel_live] id=%u phase=sever_hold_released reason=%s held_ms=%u deadline_ms=%d piece_impulse=%u t=%u tc=%u",m_EntityAlife.ID(),why,
        Device.dwTimeContinual-m_painter_hold_started,painter_sever_hold_ms,unsigned(hit_shell!=nullptr),Device.dwTimeGlobal,Device.dwTimeContinual);
}
// Lua (active_ragdoll_nn.script): the struggle waits while a body's death shell is held for its sever
bool painter_sever_hold_active(u16 id)
{
    if(!g_pGameLevel)return false;
    auto* npc=smart_cast<CAI_Stalker*>(Level().Objects.net_Find(id));
    return npc&&npc->character_physics_support()&&npc->character_physics_support()->PainterHoldActive();
}
