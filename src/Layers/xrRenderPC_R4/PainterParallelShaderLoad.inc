// The thread-local exists only while the loading/render owner is resolving cold program keys.
// Other callers of the same compiler always retain normal resource creation and error handling.
#include "PainterCompileBatch.h"
#include "../xrRender/PainterMaterialWarmKeys.h"
namespace painter_compile_load {
static thread_local painter_compile_batch::Batch* active=nullptr;
using Emit=void(*)(const char*,const char*,int,int);
static HRESULT checkedCache(const void* p,size_t n) {
    if (!painter_compile_batch::dxbc(p,n)) return E_FAIL;
    ++active->cached;
    return S_OK;
}
static HRESULT miss(const char* path,const char* name,u32 crc,const void* src,UINT size,const D3D_SHADER_MACRO* defs,
                    const char* entry,const char* target,DWORD flags,bool generated) {
    if (!generated) { ++active->failedReads; return E_FAIL; }
    includer owned;
    return active->capture(path,name,crc,src,size,defs,entry,target,flags,owned)?S_OK:E_FAIL;
}
static void request(const char* stem,const char* stage,int skin,int sample) {
    if (!stem || !stage || !active) return;
    string_path relative,editPath;
    strconcat(sizeof(relative),relative,::Render->getShaderPath(),stem,".",stage);
    IReader* f=FS.r_open("$game_shaders$",relative);
    if (!f) { ++active->failedReads; Msg("! [painter_shader_load] missing %s",relative); return; }
    const void* source=f->pointer(); u32 bytes=f->length();
    const bool directCS=stage[0]=='c';
    if (!directCS) {
        strconcat(sizeof(editPath),editPath,"shaders\\",relative);
        painterSourceEdits(editPath,f->pointer(),f->length(),source,bytes);
    }
    // ResourceManager scans its edited zero-terminated root for legacy entry names. shader_compile
    // upgrades main to the actual HW profile using the same feature-level and current macro set.
    std::string root(static_cast<const char*>(source),bytes);
    FS.r_close(f);
    std::string resource(stem),target,entry="main";
    if (stage[0]=='v') {
        target=HW.Caps.geometry_major>=2?"vs_2_0":"vs_1_1";
        if (skin>=0) resource+="_"+std::to_string(skin);
        for (const char* p:{"vs_1_1","vs_2_0"}) {
            const auto e=std::string("main_")+p;
            if (root.find(e)!=std::string::npos) { target=p; entry=e; }
        }
    } else if (stage[0]=='p') {
        target="ps_2_0";
        if (sample>=0) resource+="_"+std::to_string(sample);
        for (const char* p:{"ps_1_1","ps_1_2","ps_1_3","ps_1_4","ps_2_0"}) {
            const auto e=std::string("main_")+p;
            if (root.find(e)!=std::string::npos) { target=p; entry=e; }
        }
    } else if (directCS) target="cs_5_0";
    else { ++active->failedReads; return; }
    Engine.External.SetSkinningMode(skin); ::Render->m_MSAASample=sample;
    void* unused=nullptr;
    ::Render->shader_compile(resource.c_str(),reinterpret_cast<const DWORD*>(root.data()),bytes,
        entry.c_str(),target.c_str(),D3D10_SHADER_PACK_MATRIX_ROW_MAJOR,unused);
}
static bool publish(const painter_compile_batch::Job& j) {
    if (FAILED(j.status) || !j.code) return false;
    const void* data=j.code->GetBufferPointer();
    const u32 bytes=static_cast<u32>(j.code->GetBufferSize()),sum=crc32(data,bytes);
    // Use the same VFS writer/format as shader_compile. An interrupted/incomplete file fails
    // its source and payload CRC checks on the subsequent native resource request.
    IWriter* w=FS.w_open(j.path.c_str());
    if (!w || !w->valid()) { if(w) FS.w_close(w); return false; }
    w->w_u32(j.sourceCrc); w->w_u32(sum); w->w(data,bytes);
    FS.w_close(w);
    IReader* r=FS.r_open(j.path.c_str());
    if (!r) return false;
    const bool ok=r->length()==8u+bytes && r->r_u32()==j.sourceCrc &&
        r->r_u32()==sum && crc32(r->pointer(),r->elapsed())==sum;
    FS.r_close(r); return ok;
}
}
extern void PainterVisitWarmStock(void(*emit)(const char*,const char*,int,int));

#include "PainterShaderMenu.inc"

void PainterParallelWarmShaders() {
    using namespace painter_startup;
    if(!HW.pDevice || g_dedicated_server || painter_compile_load::active)return;
    auto& s=state();const u64 start=CPU::QPC();
    if(s.started && s.profile!=currentProfile()){s.workers.cancel();clear(s);}
    s.workers.promote();
    const auto ranked=list();
    Batch batch;
    for(const auto& p:ranked) {
        const size_t before=batch.jobs.size();
        capture(batch,p);
        for(size_t i=before;i<batch.jobs.size();++i){
            auto& j=batch.jobs[i];queueLibraries(j,p.cost);
            if(ordinary(j))s.workers.submit(input(batch,j),p.cost);
        }
    }
    const u64 resolved=CPU::QPC();
    const unsigned pendingBefore=s.workers.pending();
    if(pendingBefore)PainterShaderLoadProgress(s.workers.total()-pendingBefore,s.workers.total());
    // Preserve the proven linked shader and paired exterior programs in their original process.
    // Use the waiting owner's single lane while the full-process pool handles ordinary misses.
    // Exterior members share one source-checked worker result; ordinary fallback handles a refused result.
    while(!s.workers.waitFor(150))PainterShaderLoadProgress(s.workers.total()-s.workers.pending(),s.workers.total());
    for(auto& j:batch.jobs)acceptLibraries(j);
    unsigned adopted=0;
    for(auto& j:batch.jobs)if(ordinary(j) && accept(batch,j))++adopted;
    const unsigned earlySaved=s.earlySaved;
    unsigned available=std::thread::hardware_concurrency();
    DWORD_PTR mask=0,system=0;
    if(GetProcessAffinityMask(GetCurrentProcess(),&mask,&system) && mask){
        available=0;while(mask){mask&=mask-1;++available;}
    }
    unsigned nativeThreads=0;
    // Isolated worker failure or a mismatched result uses the existing in-renderer cold path.
    if(std::any_of(batch.jobs.begin(),batch.jobs.end(),[](const Job& j){return j.status==E_PENDING;}))
        nativeThreads=batch.run(available);
    const u64 compiled=CPU::QPC();unsigned saved=0,failed=batch.failedReads;
    for(const auto& j:batch.jobs) {
        if(painter_compile_load::publish(j))++saved;
        else {++failed;Msg("! [painter_shader_load] compile_cache_failed name=%s hr=%08x %s",j.name.c_str(),unsigned(j.status),j.error.c_str());}
    }
    if(pendingBefore)PainterShaderLoadProgress(s.workers.total(),s.workers.total());
    s.workers.discard();clear(s);
    const double ms=1000./double(CPU::qpc_freq);
    Msg("[painter_shader_startup] phase=level requests=%u local_or_seed=%u menu_saved=%u child_adopted=%u pending_before_wait=%u saved=%u failed=%u fallback_workers=%u discovery_ms=%.1f wait_compile_ms=%.1f publish_ms=%.1f",
        unsigned(ranked.size()),batch.cached,earlySaved,adopted,pendingBefore,saved,failed,nativeThreads,
        (resolved-start)*ms,(compiled-resolved)*ms,(CPU::QPC()-compiled)*ms);
}
