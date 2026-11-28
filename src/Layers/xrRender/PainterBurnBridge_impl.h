namespace PainterFire { bool thermalViewPossible(); }
#pragma once
#if defined(USE_DX11) && RENDER==R_R4
#include "../../Include/xrRender/PainterBurnRender.h"
#include "../../Include/xrRender/PainterBurnD3D11.h"
#include "PainterBurnRoles.h"
#include "PainterTornDrawPair.h"
#include "PainterRemovalMaterial.h"
#include "../../xrCore/PainterBurnTrace.h"
#include <condition_variable>
#include <thread>
#include <chrono>
namespace PainterBurnGpu {
using namespace painter_burn;
using Microsoft::WRL::ComPtr;
struct NativeView { ComPtr<ID3D11Texture2D> texture;ComPtr<ID3D11ShaderResourceView> srv; };
struct SharedWalls {NativeView view;unsigned mask=0;std::weak_ptr<const SourceFaces> source;};
struct SharedFaces {NativeView view;unsigned count=0;std::weak_ptr<const SourceFaces> source;std::weak_ptr<const std::vector<u32>> local;};
struct Map {
    std::shared_ptr<const SourceFaces> source;
    std::shared_ptr<const std::vector<u32>> original;
    u32 slot=~0u,count=0,wallMask=0;
    NativeView faces,walls;bool wallsOnly=false;std::atomic<bool> ready{false};
    std::shared_ptr<SharedWalls> heldWalls;
    std::shared_ptr<SharedFaces> heldFaces;
};
struct Bank {std::bitset<MaxOwners+1> boundLive,boundKnown;GpuFamily gpu;ref_texture wrapped;std::shared_ptr<const Delta> busy;U64 serial=0,revision=0;bool uploaded=false;};
inline NativeView make(ID3D11Device* dev,const std::vector<Texel>& data){
    NativeView v;if(data.empty())return v;
    D3D11_TEXTURE2D_DESC d{};d.Width=256;d.Height=UINT(data.size()/256);d.ArraySize=d.MipLevels=d.SampleDesc.Count=1;
    d.Format=DXGI_FORMAT_R32G32B32A32_UINT;d.Usage=D3D11_USAGE_IMMUTABLE;d.BindFlags=D3D11_BIND_SHADER_RESOURCE;
    D3D11_SUBRESOURCE_DATA initial{};initial.pSysMem=data.data();initial.SysMemPitch=256*sizeof(Texel);
    if(FAILED(dev->CreateTexture2D(&d,&initial,&v.texture))||FAILED(dev->CreateShaderResourceView(v.texture.Get(),nullptr,&v.srv)))return {};
    return v;
}
inline std::shared_ptr<const SourceFaces> geometryOf(const std::shared_ptr<const SourceFaces>& source){
    return source && source->authored ? source->authored : source;
}
inline std::shared_ptr<Map> buildFaces(ID3D11Device* device,const std::shared_ptr<const SourceFaces>& geometry,
    u32 slot,const std::vector<u32>* remap){
    if(!device || !geometry)return {};
    auto map=std::make_shared<Map>();map->source=geometry;map->slot=slot;
    const auto pixels=sourceTable(*geometry,slot,remap,map->count);
    if(!pixels.empty()){map->faces=make(device,pixels);if(!map->faces.srv)return {};}
    map->ready.store(true,std::memory_order_release);
    return map;
}
inline bool bindRef(ref_texture& ref,const void* instance,const char* suffix,ID3D11ShaderResourceView* view){
    if(!view)return false;
    auto next=ref;
    if(!next){string128 n;xr_sprintf(n,"$user$painter_burn_%p_%s",instance,suffix);next=xr_new<CTexture>();next->set_name(n);}
    if(!next||next->flags.bLoading)return false;
    ComPtr<ID3D11Resource> texture;view->GetResource(&texture);
    if(!next->surface_set_prepared(texture.Get(),view))return false;
    ref=next;
    return true;
}
struct Envelope {std::shared_ptr<const Delta> delta;std::shared_ptr<Bank> bank;};
struct Service {
    std::mutex mutex;std::condition_variable cv;std::thread task;bool stopping=false;
    ComPtr<ID3D11Device> device;
    std::deque<std::shared_ptr<const Delta>> pending;
    std::deque<std::shared_ptr<Map>> maps;
    std::deque<Envelope> incoming,applying;
    std::unordered_map<U64,std::shared_ptr<Bank>> active;
    std::atomic<unsigned> outstanding{0};
    std::map<std::pair<U64,U64>,std::shared_ptr<Bank>> prepared; // resource worker only
    std::map<const SourceFaces*,std::weak_ptr<SharedWalls>> wallCache;
    std::map<std::tuple<const SourceFaces*,const void*,u32>,std::weak_ptr<SharedFaces>> faceCache;
    Clock16 clock;U32 now=0;std::atomic<U32> hostTick{0};std::atomic<bool> hostClock{false};
    std::atomic<U64> completed{0};std::atomic<U32> deviceEpoch{1};
    void start(ID3D11Device* dev){
        std::lock_guard<std::mutex> lock(mutex);
        if(task.joinable()||!dev||!outstanding)return;stopping=false;device=dev;task=std::thread([this]{work();});
    }
    bool submit(std::shared_ptr<const Delta> d){
        if(!d||!d->lineage||!d->serial)return false;
        std::lock_guard<std::mutex> lock(mutex);
        if(stopping||outstanding>=64)return false;
        pending.push_back(std::move(d));++outstanding;cv.notify_one();return true;
    }
    bool request(std::shared_ptr<Map> m){
        std::lock_guard<std::mutex> lock(mutex);if(stopping||maps.size()>=128)return false;
        maps.push_back(std::move(m));cv.notify_one();return true;
    }
    void work(){
        for(;;){
            std::shared_ptr<const Delta> d;std::shared_ptr<Map> m;
            {std::unique_lock<std::mutex> lock(mutex);cv.wait(lock,[&]{return stopping||!pending.empty()||!maps.empty();});
             if(stopping)return;if(!pending.empty()){d=std::move(pending.front());pending.pop_front();}else{m=std::move(maps.front());maps.pop_front();}}
            if(m){
                if(wallCache.size()>256){for(auto i=wallCache.begin();i!=wallCache.end();)if(i->second.expired())i=wallCache.erase(i);else ++i;}
                if(faceCache.size()>2048){for(auto i=faceCache.begin();i!=faceCache.end();)if(i->second.expired())i=faceCache.erase(i);else ++i;}
                const auto geometry=m->source&&m->source->authored?m->source->authored:m->source;
                if(!m->wallsOnly && geometry&&m->slot<geometry->byChild.size()){
                    auto index=std::make_tuple(geometry.get(),m->original.get(),m->slot);
                    auto cached=faceCache[index].lock();
                    if(!cached||cached->source.lock()!=geometry||cached->local.lock()!=m->original){
                    cached=std::make_shared<SharedFaces>();cached->source=geometry;cached->local=m->original;
                    auto tex=sourceTable(*geometry,m->slot,m->original.get(),cached->count);
                    cached->view=make(device.Get(),tex);if(!cached->view.srv)cached->count=0;faceCache[index]=cached;
                    }
                    m->heldFaces=cached;m->count=cached->count;m->faces=cached->view;
                }
                if(m->source&&!m->source->walls.empty()){
                    auto cached=wallCache[m->source.get()].lock();
                    if(!cached||cached->source.lock()!=m->source){
                        cached=std::make_shared<SharedWalls>();cached->source=m->source;
                        auto data=wallTable(m->source->walls,cached->mask);cached->view=make(device.Get(),data);
                        if(!cached->view.srv)cached->mask=0;wallCache[m->source.get()]=cached;
                    }
                    m->heldWalls=cached;m->walls=cached->view;m->wallMask=cached->mask;
                }
                m->ready.store(true,std::memory_order_release);continue;
            }
            auto k=std::make_pair(d->lineage,d->serial);auto& bank=prepared[k];
            if(!bank&&!d->retired){
                auto next=std::make_shared<Bank>();auto hr=next->gpu.prepare(device.Get(),d->lineage,d->serial);
                if(FAILED(hr)){std::unique_lock<std::mutex> lk(mutex);
                    pending.push_front(std::move(d));cv.wait_for(lk,std::chrono::milliseconds(100),[&]{return stopping;});continue;}
                next->serial=d->serial;bank=std::move(next);
            }
            auto b=bank;if(d->retired)prepared.erase(k);
            {std::lock_guard<std::mutex> lock(mutex);incoming.push_back({std::move(d),std::move(b)});}
        }
    }
    void pump(){
        now=hostClock.load()?hostTick.load():clock.observe(Device.dwTimeGlobal);
        {std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);if(lock.owns_lock())while(!incoming.empty()){applying.push_back(std::move(incoming.front()));incoming.pop_front();}}
        unsigned credit=32768;const auto deadline=std::chrono::steady_clock::now()+std::chrono::microseconds(350);
        while(!applying.empty()&&std::chrono::steady_clock::now()<deadline){
            auto& r=applying.front();const auto& d=r.delta;
            if(d->retired){auto it=active.find(d->lineage);if(it!=active.end()&&it->second->serial==d->serial)active.erase(it);}
            else if(r.bank){
                auto& b=*r.bank;
                if(!b.busy){if(!b.gpu.submit(d)){applying.pop_front();--outstanding;continue;}b.busy=d;}
                if(!b.uploaded){if(!b.gpu.pump(HW.pContext,credit,[&]{return std::chrono::steady_clock::now()<deadline;}))break;b.uploaded=true;}
                if(!bindRef(b.wrapped,&b,"state",b.gpu.view()))break;
                // This metadata belongs to the texture finally bound on the visible wrapper.
                b.boundLive=d->liveSpread;b.boundKnown=d->knownTag;
                active[d->lineage]=r.bank;b.revision=d->revision;b.uploaded=false;b.busy.reset();++completed;
                if(PainterBurnTraceEnabled()&&!d->spans.empty())
                    Msg("[painter_burn_trace] stage=field_uploaded lineage=%llu serial=%llu revision=%llu spans=%zu owners=%zu",
                        d->lineage,d->serial,d->revision,d->spans.size(),d->owners.size());
            }
            applying.pop_front();--outstanding;
        }
    }
    void shutdown(){
        {std::lock_guard<std::mutex> lock(mutex);stopping=true;cv.notify_all();}
        if(task.joinable())task.join();
        std::lock_guard<std::mutex> lock(mutex);pending.clear();maps.clear();incoming.clear();applying.clear();active.clear();prepared.clear();faceCache.clear();wallCache.clear();device.Reset();outstanding=0;stopping=false;++deviceEpoch;
    }
};
inline std::atomic<Service*>& slot(){static std::atomic<Service*> s{nullptr};return s;}
inline Service* current(){return slot().load(std::memory_order_acquire);}
inline Service& service(){static auto* s=[]{auto* p=new Service;slot().store(p,std::memory_order_release);return p;}();return *s;}
inline std::string strip(const char* full,int max,int& suffix){std::string n=full?full:"";suffix=-1;if(n.size()>2&&n[n.size()-2]=='_'&&n.back()>='0'&&n.back()<='0'+max){suffix=n.back()-'0';n.resize(n.size()-2);}return n;}
// Main-thread access under the resource creation guard. Once the owner's pump
// has finished an unavailable optional pair, it cannot become ready until reset.
inline bool queueMissingPair(const PainterBloodPrograms::Pair& pair){
    auto& cache=PainterBloodPrograms::cache();
    std::lock_guard<std::mutex> lock(cache.lock);
    if(cache.known.insert(pair).second){cache.pending.push_back(pair);return true;}
    return std::any_of(cache.pending.begin(),cache.pending.end(),[&](const auto& queued){
        return !(queued<pair) && !(pair<queued);
    });
}
inline void exactPair(const std::string& v,int skin,const std::string& p,int sample){
    PainterBloodPrograms::Pair q;q.vertex=v;q.skin=skin;q.pixel=p;q.sample=sample;auto& cache=PainterBloodPrograms::cache();
    std::lock_guard<std::mutex> lock(cache.lock);if(cache.known.insert(q).second)cache.pending.push_back(q);
}
struct Zero: R_constant_setup {void setup(R_constant* c)override{RCache.set_c(c,0.f,0.f,0.f,0.f);}};
inline Zero& zero(){static Zero z;return z;}
struct SurfaceAlpha: R_constant_setup {
    void setup(R_constant* c)override{RCache.set_c(c,1.f,0.f,0.f,0.f);}
};
inline R_constant_setup* surfaceSelector(const char* member) {
    static SurfaceAlpha alpha;
    if (!xr_strcmp(member,"painter_burn_painter_blood_deffer_base_aref_flat") ||
        !xr_strcmp(member,"painter_burn_painter_blood_deffer_base_aref_flat_d") ||
        !xr_strcmp(member,"painter_burn_painter_blood_painter_mask_deffer_base_aref_flat") ||
        !xr_strcmp(member,"painter_burn_painter_blood_painter_mask_deffer_base_aref_flat_d") ||
        !xr_strcmp(member,"painter_burn_painter_blood_painter_maskt_deffer_base_aref_flat") ||
        !xr_strcmp(member,"painter_burn_painter_blood_painter_maskt_deffer_base_aref_flat_d")) return &alpha;
    if (!xr_strcmp(member,"painter_burn_painter_blood_deffer_base_flat") ||
        !xr_strcmp(member,"painter_burn_painter_blood_deffer_base_flat_d") ||
        !xr_strcmp(member,"painter_burn_painter_blood_painter_mask_deffer_base_flat") ||
        !xr_strcmp(member,"painter_burn_painter_blood_painter_mask_deffer_base_flat_d") ||
        !xr_strcmp(member,"painter_burn_painter_blood_painter_maskt_deffer_base_flat") ||
        !xr_strcmp(member,"painter_burn_painter_blood_painter_maskt_deffer_base_flat_d")) return &zero();
    if(!xr_strcmp(member,"painter_burn_painter_blood_deffer_base_aref_bump_d-hq") || !xr_strcmp(member,"painter_burn_painter_blood_deffer_base_aref_bump") || !xr_strcmp(member,"painter_burn_painter_blood_deffer_base_aref_bump_d") || !xr_strcmp(member,"painter_burn_painter_blood_painter_mask_deffer_base_aref_bump") || !xr_strcmp(member,"painter_burn_painter_blood_painter_mask_deffer_base_aref_bump_d-hq") || !xr_strcmp(member,"painter_burn_painter_blood_painter_mask_deffer_base_aref_bump_d") || !xr_strcmp(member,"painter_burn_painter_blood_painter_maskt_deffer_base_aref_bump") || !xr_strcmp(member,"painter_burn_painter_blood_painter_maskt_deffer_base_aref_bump_d-hq") || !xr_strcmp(member,"painter_burn_painter_blood_painter_maskt_deffer_base_aref_bump_d"))return &alpha;
    if(!xr_strcmp(member,"painter_burn_painter_blood_deffer_base_bump_d-hq") || !xr_strcmp(member,"painter_burn_painter_blood_deffer_base_bump") || !xr_strcmp(member,"painter_burn_painter_blood_deffer_base_bump_d") || !xr_strcmp(member,"painter_burn_painter_blood_painter_mask_deffer_base_bump") || !xr_strcmp(member,"painter_burn_painter_blood_painter_mask_deffer_base_bump_d-hq") || !xr_strcmp(member,"painter_burn_painter_blood_painter_mask_deffer_base_bump_d") || !xr_strcmp(member,"painter_burn_painter_blood_painter_maskt_deffer_base_bump") || !xr_strcmp(member,"painter_burn_painter_blood_painter_maskt_deffer_base_bump_d-hq") || !xr_strcmp(member,"painter_burn_painter_blood_painter_maskt_deffer_base_bump_d"))return &zero();
    return nullptr;
}
}
struct PainterBurnChildBinding {
    ref_shader original,applied;ref_texture faces,walls;std::shared_ptr<PainterBurnGpu::Map> map,faceMap,pendingFaces;
    painter_torn_draw::Binding torn;
    ref_texture blood,state;bool unsupported=false,interior=false;
};
struct PainterBurnActorBinding {
    unsigned masterStamp=PainterBodyMasterStamp();
    painter_burn::OwnerKey owner;u64 lineage{};ref_texture empty;
    std::shared_ptr<const painter_burn::SourceFaces> source;
    std::shared_ptr<PainterBurnGpu::Bank> bank;
    std::unordered_map<const CSkeletonX*,PainterBurnChildBinding> children;
    painter_burn::OwnerKey priorSplit{},bakedOwner{};
    int publicationMode=-1;
    unsigned publicationEpoch=0;
    bool publicationProgramsReady=false;
    std::set<PainterBloodPrograms::Pair> publicationPrograms;
    bool hasRenderedBurn=false;
    u64 tracedSerial=0,tracedRevision=0;u64 visualGeneration=~0ull;u32 frame=~0u;bool needsRefresh=true;const CTexture* lastBlood=nullptr;
    // the baked recession (painter_burn_bake.cs): this body's map, made again when its burn changes
    Microsoft::WRL::ComPtr<ID3D11Texture3D> bakeTex;Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> bakeSrv;
    Microsoft::WRL::ComPtr<ID3D11UnorderedAccessView> bakeUav;ID3D11Device* bakeDevice=nullptr;ref_texture baked;
    u64 bakedRevision=~0ull;const void* bakedBank=nullptr;u32 bakedAt=0;bool bakeValid=false,bakeQueued=false;
};
namespace PainterBurnGpu {
// Only a committed split may nominate its previous owner; sparse uploads already
// swap complete front/back images atomically. Never combine a new tag with the old image.
inline painter_burn::OwnerKey visibleOwner(const PainterBurnActorBinding& b){
    if(b.bank && b.bank->gpu.meta(b.owner,0)[0])return b.owner;
    return b.priorSplit.spawn?b.priorSplit:b.owner;
}
enum class Clone{Done,Pending,Unsupported};
inline Clone clone(const ref_shader& original,const ref_texture& state,const ref_texture& faces,const ref_texture& walls,const ref_texture& blood,const ref_texture& baked,ref_shader& out,bool& fittedInterior,painter_torn_draw::Binding& torn){
    if(!original)return Clone::Unsupported;
    xrCriticalSectionTryGuard creation(DEV->PainterBloodCreationGuard());if(!creation.owns_lock())return Clone::Pending;
    Shader dst;unsigned changed=0;bool hasInterior=false;
    for(unsigned n=0;n<SHADER_ELEMENTS_MAX;++n){auto oe=original->E[n];if(!oe)continue;
        ShaderElement e;e._copy(*oe);e.passes.clear();
        for(const auto& src:oe->passes){if(!src||!src->ps||!src->vs){e.passes.push_back(src);continue;}
            int sample,ignored;std::string ps=strip(*src->ps->cName,7,sample),vs=strip(*src->vs->cName,4,ignored);
            auto stages=painter_body_tess::fromPass(*src,vs,ps);
            if(stages==painter_body_tess::Result::Unsupported){e.passes.push_back(src);continue;}
            if(ps.find("painter_burn_")==0){e.passes.push_back(src);continue;}
            // the twin's names: one rule, the warm-up asks it too (PainterBloodPrograms::burnNames)
            std::string vertex,pixel;
            if(!PainterBloodPrograms::burnNames(vs,ps,vertex,pixel)){e.passes.push_back(src);continue;}
            SPass p;p.state=src->state;p.C=src->C;p.gs=src->gs;p.hs=src->hs;p.ds=src->ds;p.cs=src->cs;
            painter_body_tess::triangleStages(p,*DEV,stages);
            if(!PainterBloodPrograms::find(vertex,src->vs->skinning,pixel,sample,p.vs,p.ps)){exactPair(vertex,src->vs->skinning,pixel,sample);return Clone::Pending;}
             if(sample<0 && pixel=="painter_burn_painter_blood_painter_vmi_tear")torn.read(p.ps->ps);
             R_constant_table constants;constants.merge(&p.ps->constants);constants.merge(&p.vs->constants);STextureList textures;
            for(auto& c:constants.table){
                const auto nm=*c->name;const ref_texture* field=nullptr;
                if(!xr_strcmp(nm,"painter_surface_variant")){if(auto* setup=surfaceSelector(pixel.c_str())){c->handler=setup;continue;}}
                if(!xr_strcmp(nm,"s_painter_burn_field"))field=&state;
                else if(!xr_strcmp(nm,"s_painter_burn_faces"))field=&faces;
                else if(!xr_strcmp(nm,"s_painter_burn_walls"))field=&walls;
                else if(!xr_strcmp(nm,"s_painter_burn_baked"))field=&baked; // the body's baked recession (painter_burn_bake.cs)
                else if(!xr_strcmp(nm,"s_painter_blood_cells")||!xr_strcmp(nm,"s_painter_blood_pages")||!xr_strcmp(nm,"s_painter_blood_records"))field=&blood;
                else if(!xr_strcmp(nm,"painter_blood_clock")){c->handler=&PainterBloodBake::clock();continue;}
                else if(!xr_strcmp(nm,"painter_burn_meta")||!xr_strcmp(nm,"painter_burn_draw")||!xr_strcmp(nm,"painter_burn_bake")){c->handler=&zero();continue;}
                auto* old=src->constants?src->constants->get(nm):nullptr;
                if(!field&&old&&old->type==c->type){c->handler=old->handler;if(c->type==RC_dx10texture){bool found=false;
                    if(src->T)for(auto& pair:*src->T)if(pair.first==old->samp.index){textures.emplace_back(c->samp.index,pair.second);found=true;break;}
                    if(!found)return Clone::Unsupported;
                }else if(c->type==RC_sampler&&c->samp.index!=old->samp.index)return Clone::Unsupported;continue;}
                if(!field&&(!xr_strcmp(nm,"s_painter_blood_cells")||!xr_strcmp(nm,"s_painter_blood_pages")||!xr_strcmp(nm,"s_painter_blood_records")))field=&blood;
                if(field){if(c->type!=RC_dx10texture||!(*field))return Clone::Unsupported;textures.emplace_back(c->samp.index,*field);continue;}
                if(!xr_strcmp(nm,"painter_blood_clock")){c->handler=&PainterBloodBake::clock();continue;}
                if(!xr_strcmp(nm,"painter_blood_target")){c->handler=&PainterBloodBake::target();continue;}
                if(!xr_strcmp(nm,"ssfx_blood_decals")){c->handler=PainterBloodDecalControls();continue;}
                if(!old && pixel.find("painter_blood_painter_vmi_")!=std::string::npos && PainterBindInteriorViewConstant(*c))continue;
                return Clone::Unsupported;
            }
            if(pixel.find("painter_blood_painter_vmi_")!=std::string::npos)hasInterior=true;
            p.constants=DEV->_CreateConstantTable(constants);p.T=DEV->_CreateTextureList(textures);e.passes.push_back(DEV->_CreatePass(p));++changed;
        }
        dst.E[n]=DEV->_CreateElement(e);
    }
    if(!changed)return Clone::Unsupported;out=DEV->_CreateShader(&dst);fittedInterior=hasInterior;return out?Clone::Done:Clone::Unsupported;
}
}
// ---- the baked recession (2026-11-22, painter_burn_bake.cs) --------------------------------------------------------------
namespace PainterBurnBake {
constexpr UINT W=128,H=256,D=64; // = PB_BAKE_SIZE in painter_burn_recession.h
struct Job {std::shared_ptr<PainterBurnActorBinding> actor;ref_texture pages,data,field;std::array<float,4> meta;u64 revision=0;const void* bank=nullptr;painter_burn::OwnerKey owner{};};
inline std::vector<Job>& jobs(){static std::vector<Job> v;return v;}
struct Program {SCS* cs=nullptr;ref_ctable table;bool tried=false;ID3D11Device* device=nullptr;};
inline Program& program(){static Program p;return p;}
// the bake's program, compiled the renderer's way but not through the resource manager (which ends the game on a failure)
inline bool ready(){
    auto& p=program();
    if(p.device!=HW.pDevice){p.cs=nullptr;p.table=nullptr;p.tried=false;p.device=HW.pDevice;}
    if(p.cs)return true;
    if(p.tried||!HW.pDevice)return false;
    p.tried=true;
    string_path name;strconcat(sizeof(name),name,::Render->getShaderPath(),"painter_burn_bake.cs");FS.update_path(name,"$game_shaders$",name);
    if(!FS.exist(name)){Msg("! [painter_burn] painter_burn_bake.cs is missing: burnt bodies are drawn without the baked recession");return false;}
    IReader* file=FS.r_open(name);if(!file)return false;
    SCS* sh=xr_new<SCS>();sh->sh=nullptr;
    const HRESULT hr=::Render->shader_compile("painter_burn_bake",(DWORD const*)file->pointer(),file->length(),"main","cs_5_0",
        D3D10_SHADER_PACK_MATRIX_ROW_MAJOR,(void*&)sh);
    FS.r_close(file);
    if(FAILED(hr)||!sh->sh){Msg("! [painter_burn] painter_burn_bake.cs did not compile: burnt bodies are drawn without the baked recession");return false;}
    p.cs=sh;p.table=DEV->_CreateConstantTable(sh->constants);
    Msg("[painter_burn] baked recession program made");
    return true;
}
// a body's map (2 MB), made once; bound to its burnt programs as s_painter_burn_baked
inline bool ensure(PainterBurnActorBinding& b){
    if(b.bakeTex&&b.bakeDevice==HW.pDevice&&b.baked)return true;
    b.bakeTex.Reset();b.bakeSrv.Reset();b.bakeUav.Reset();b.bakeValid=false;b.bakedRevision=~0ull;b.bakedBank=nullptr;
    if(!HW.pDevice)return false;
    D3D11_TEXTURE3D_DESC d{};d.Width=W;d.Height=H;d.Depth=D;d.MipLevels=1;d.Format=DXGI_FORMAT_R8_UINT;d.Usage=D3D11_USAGE_DEFAULT;
    d.BindFlags=D3D11_BIND_SHADER_RESOURCE|D3D11_BIND_UNORDERED_ACCESS;
    if(FAILED(HW.pDevice->CreateTexture3D(&d,nullptr,&b.bakeTex))||FAILED(HW.pDevice->CreateShaderResourceView(b.bakeTex.Get(),nullptr,&b.bakeSrv))||
       FAILED(HW.pDevice->CreateUnorderedAccessView(b.bakeTex.Get(),nullptr,&b.bakeUav))){b.bakeTex.Reset();b.bakeSrv.Reset();b.bakeUav.Reset();return false;}
    b.bakeDevice=HW.pDevice;
    if(!PainterBurnGpu::bindRef(b.baked,&b,"baked",b.bakeSrv.Get())){b.bakeTex.Reset();b.bakeSrv.Reset();b.bakeUav.Reset();return false;}
    return true;
}
// the body's anatomy (its pages and data), as one of its burnt interior programs reads them
inline bool inputs(PainterBurnActorBinding& b,ref_texture& pages,ref_texture& data){
    for(auto& kv:b.children){const auto& sh=kv.second.applied;if(!sh||!kv.second.interior)continue;
        for(unsigned n=0;n<SHADER_ELEMENTS_MAX;++n){auto e=sh->E[n];if(!e)continue;
            for(const auto& pass:e->passes){if(!pass||!pass->T||!pass->constants)continue;
                auto cp=pass->constants->get("s_anatomy_pages");auto cd=pass->constants->get("s_anatomy_data");
                if(!cp||!cd)continue;
                ref_texture tp,td;
                for(const auto& t:*pass->T){if(t.first==cp->samp.index)tp=t.second;if(t.first==cd->samp.index)td=t.second;}
                if(tp&&td){pages=tp;data=td;return true;}
            }
        }
    }
    return false;
}
// a bake is due: never made, or the burn changed (while it burns: twice a second at most)
inline void consider(const std::shared_ptr<PainterBurnActorBinding>& sp,const std::array<float,4>& meta){
    auto& b=*sp;
    if(!b.bank||!b.baked||b.bakeQueued)return;
    const auto owner=PainterBurnGpu::visibleOwner(b);
    if(b.bakeValid&&b.bakedRevision==b.bank->revision&&b.bakedBank==b.bank.get()&&b.bakedOwner==owner)return;
    if(b.bakeValid&&Device.dwTimeGlobal-b.bakedAt<500u)return;
    ref_texture pages,data;if(!inputs(b,pages,data)||!b.bank->wrapped)return;
    Job j;j.actor=sp;j.pages=pages;j.data=data;j.field=b.bank->wrapped;j.meta=meta;j.revision=b.bank->revision;j.bank=b.bank.get();j.owner=owner;
    jobs().push_back(std::move(j));b.bakeQueued=true;
}
// at the start of a frame (CRender::OnFrame), before anything is drawn: up to two bakes
inline void run(){
    auto& v=jobs();
    if(v.empty()){if(Device.dwPrecacheFrame&&!program().tried)ready();return;}
    if(!ready()){for(auto& j:v)if(j.actor)j.actor->bakeQueued=false;v.clear();return;}
    auto& p=program();
    unsigned done=0;
    for(auto it=v.begin();it!=v.end()&&done<2u;){
        Job& j=*it;PainterBurnActorBinding& b=*j.actor;
        if(!b.bakeUav||b.bakeDevice!=HW.pDevice||!j.pages||!j.data||!j.field ||
            !b.bank||b.bank.get()!=j.bank||b.bank->revision!=j.revision||!(PainterBurnGpu::visibleOwner(b)==j.owner)){b.bakeQueued=false;it=v.erase(it);continue;}
        STextureList list;
        for(const auto& c:p.cs->constants.table){
            if(c->type!=RC_dx10texture)continue;
            const char* n=*c->name;
            const ref_texture* t=!xr_strcmp(n,"s_anatomy_pages")?&j.pages:!xr_strcmp(n,"s_anatomy_data")?&j.data:
                                 !xr_strcmp(n,"s_painter_burn_field")?&j.field:nullptr;
            if(t)list.emplace_back(c->samp.index,*t);
        }
        ref_texture_list textures=DEV->_CreateTextureList(list);
        RCache.set_CS(p.cs->sh,"painter_burn_bake");
        RCache.set_Constants(p.table);
        RCache.set_Textures(textures._get());
        RCache.set_c("painter_burn_meta",j.meta[0],j.meta[1],j.meta[2],j.meta[3]);
        ID3D11UnorderedAccessView* uav=b.bakeUav.Get();UINT keep=UINT(-1);
        HW.pContext->CSSetUnorderedAccessViews(0,1,&uav,&keep);
        RCache.Compute((W+3)/4,(H+3)/4,(D+3)/4);
        ID3D11UnorderedAccessView* none=nullptr;
        HW.pContext->CSSetUnorderedAccessViews(0,1,&none,&keep);
        b.bakeValid=true;b.bakedOwner=j.owner;b.bakedRevision=j.revision;b.bakedBank=j.bank;b.bakedAt=Device.dwTimeGlobal;b.bakeQueued=false;
        ++done;it=v.erase(it);
    }
}
}
void CKinematics::LL_PainterBurnOwner(u64 lineage,u64 spawn,u64 generation,std::shared_ptr<const painter_burn::SourceFaces> source){
    if(!lineage||!spawn||!(PainterBodyMasterStamp()&1u)){
        if(m_burn_binding){auto restore=[&](const auto& list){for(auto* v:list){auto* sk=fast_dynamic_cast<CSkeletonX*>(v);
            auto it=m_burn_binding->children.find(sk);if(it==m_burn_binding->children.end())continue;
            auto* visual=fast_dynamic_cast<dxRender_Visual*>(v);if(visual&&it->second.applied&&visual->shader._get()==it->second.applied._get())visual->shader=it->second.original;
        }};restore(children);restore(children_invisible);}
        m_burn_binding.reset();return;
    }
    if(!m_burn_binding)m_burn_binding=std::make_shared<PainterBurnActorBinding>();
    auto& b=*m_burn_binding;if(b.lineage!=lineage||!(b.owner==painter_burn::OwnerKey{spawn,generation})||b.source!=source){b.frame=~0u;b.needsRefresh=true;}
    if(b.lineage!=lineage||!(b.owner==painter_burn::OwnerKey{spawn,generation})){b.tracedSerial=0;b.tracedRevision=0;b.priorSplit={};b.publicationMode=-1;}
    b.lineage=lineage;b.owner={spawn,generation};b.source=std::move(source);
}
std::shared_ptr<const painter_burn::SourceFaces> CKinematics::LL_PainterBurnStageSource() const{
    return m_burn_binding && m_burn_binding->bank && m_burn_binding->hasRenderedBurn ?
        PainterBurnGpu::geometryOf(m_burn_binding->source) : nullptr;
}
int CKinematics::LL_TryBurnPublication(){
    using namespace PainterBurnGpu;
    if(!m_burn_binding || !m_burn_binding->hasRenderedBurn || m_burn_binding->masterStamp!=PainterBodyMasterStamp())return 1;
    auto& b=*m_burn_binding;
    if(!DEV->PainterBloodCreationGuard().TryEnter())return 0;
    try{
        const int mode=PainterMaskOnlyMode();
        const auto epoch=service().deviceEpoch.load();
        if(mode!=b.publicationMode || epoch!=b.publicationEpoch){
            b.publicationMode=mode;b.publicationEpoch=epoch;b.publicationProgramsReady=false;b.publicationPrograms.clear();
            auto add=[&](const std::string& vs,int skin,const std::string& ps,int sample){
                if(skin<0||skin>4)return;
                std::string vertex,pixel;
                if(PainterBloodPrograms::burnNames(vs,ps,vertex,pixel))
                    b.publicationPrograms.insert({vertex,skin,pixel,sample});
            };
            auto collect=[&](const auto& list){for(auto* v:list){
                auto* sk=fast_dynamic_cast<CSkeletonX*>(v);auto* visual=fast_dynamic_cast<dxRender_Visual*>(v);
                if(!sk||!visual||!visual->shader)continue;
                auto it=b.children.find(sk);
                const auto sh=it!=b.children.end()&&it->second.applied==visual->shader?it->second.original:visual->shader;
                if(!sh)continue;
                for(auto& e:sh->E)if(e)for(auto& pass:e->passes){
                    if(!pass||!pass->vs||!pass->ps)continue;
                    int dummy,sample;auto vs=strip(*pass->vs->cName,4,dummy),ps=strip(*pass->ps->cName,7,sample);
                    if(painter_body_tess::fromPass(*pass,vs,ps)==painter_body_tess::Result::Unsupported)continue;
                    add(vs,pass->vs->skinning,ps,sample);add(vs,4,ps,sample);
                    PainterRemovalMaterial::ProgramPair pair;
                    if(!PainterRemovalMaterial::programNames(vs.c_str(),ps.c_str(),pair))continue;
                    pair.sample=sample;
                    PainterRemovalMaterial::clonePairPrograms(pair,mode,[&](const xr_string& vert,const xr_string& pix,int msaa){
                        add(vert.c_str(),4,pix.c_str(),msaa);
                        if(pix.find("painter_blood_")!=0)add(std::string("painter_blood_")+vert.c_str(),4,std::string("painter_blood_")+pix.c_str(),msaa);
                    });
                }
            }};
            collect(children);collect(children_invisible);collect(m_geometry_original_children);collect(m_geometry_original_invisible);
        }
        if(!b.publicationProgramsReady){
            bool ready=true;
            for(const auto& pr:b.publicationPrograms){
                ref_vs vs;ref_ps ps;
                if(!PainterBloodPrograms::find(pr.vertex,pr.skin,pr.pixel,pr.sample,vs,ps)){
                    if(queueMissingPair(pr))ready=false;
                }
            }
            if(!ready){DEV->PainterBloodCreationGuard().Leave();return 0;}
            b.publicationProgramsReady=true;
        }
        return 2;
    }catch(...){DEV->PainterBloodCreationGuard().Leave();throw;}
}
void CKinematics::LL_EndBurnPublication(){DEV->PainterBloodCreationGuard().Leave();}
void CKinematics::LL_PainterBurnSplitOwner(u64 lineage,u64 spawn,u64 generation,u64 previousSpawn,u64 previousGeneration,
    std::shared_ptr<const painter_burn::SourceFaces> exterior){
    LL_PainterBurnOwner(lineage,spawn,generation,std::move(exterior));
    if(m_burn_binding && previousSpawn && previousGeneration){
        m_burn_binding->priorSplit={previousSpawn,previousGeneration};
        m_burn_binding->hasRenderedBurn=true;
        m_burn_binding->frame=~0u;PainterBurnRefresh();
    }
}
std::vector<unsigned char> CKinematics::LL_PainterBurnMaterialRoles() const {
    const CKinematics* src=m_geometry_donor?m_geometry_donor:this;
    std::vector<unsigned char> roles;
    auto visit=[&](const auto& list){for(auto* v:list){
        auto* sk=fast_dynamic_cast<CSkeletonX*>(v);auto* visual=fast_dynamic_cast<dxRender_Visual*>(v);
        if(!sk||!visual||!visual->shader)continue;const unsigned child=sk->GeometryChildIndex();if(child>=65535)continue;
        if(roles.size()<=child)roles.resize(child+1);
        for(unsigned el=0;el<SHADER_ELEMENTS_MAX;++el){auto e=visual->shader->E[el];if(!e)continue;
            for(auto& p:e->passes){if(!p||!p->T||!p->constants)continue;auto* c=p->constants->get("s_base");if(!c||c->type!=RC_dx10texture)continue;
                for(auto& t:*p->T)if(t.first==c->samp.index&&t.second&&t.second->cName.size()){
                    const unsigned role=unsigned(painter_burn::sourceMaterialRole(*t.second->cName));
                    if(role)roles[child]=static_cast<unsigned char>(role);
                }
            }
        }
    }};
    if(src->m_geometry_original_children.empty()){visit(src->children);visit(src->children_invisible);}
    else{visit(src->m_geometry_original_children);visit(src->m_geometry_original_invisible);}
    return roles;
}
void CKinematics::PainterBurnRefresh(){
    if(!m_burn_binding)return;
    if(m_burn_binding->masterStamp!=PainterBodyMasterStamp()){LL_PainterBurnOwner(0,0,0,{});return;}
    using namespace PainterBurnGpu;
    auto& b=*m_burn_binding;if(b.frame==Device.dwFrame&&b.visualGeneration==LL_SurfaceGeometryGeneration())return;
    b.frame=Device.dwFrame;const bool geometryChanged=b.visualGeneration!=LL_SurfaceGeometryGeneration();b.visualGeneration=LL_SurfaceGeometryGeneration();
    auto& svc=service();auto it=svc.active.find(b.lineage);
    const auto prior=b.bank;
    b.bank=it==svc.active.end()?std::shared_ptr<Bank>{}:it->second;
    // If generation changes the old visible wrapper is disabled immediately by
    // PainterBurnDraw; never apply a stale wall certificate through a new draw.
    if(!b.bank||!b.source)return;
    if(b.priorSplit.spawn && b.bank->gpu.meta(b.owner,svc.now)[0]){b.priorSplit={};b.needsRefresh=true;}
    const auto owner=visibleOwner(b);
    if(!b.bank->gpu.meta(owner,svc.now)[0])return;
    const auto zero=PainterBloodBake::emptyTable(HW.pDevice);if(!zero)return;
    if(!b.empty&&!bindRef(b.empty,&b,"empty",zero->view.p))return;
    PainterBurnBake::ensure(b);
    PainterBurnBake::consider(m_burn_binding,b.bank->gpu.meta(owner,svc.now));
    ref_texture liveBlood=b.empty;
    if(m_blood_cells&&m_blood_cells->shared&&m_blood_cells->shared->gpu.cellTexture)liveBlood=m_blood_cells->shared->gpu.cellTexture;
    if(!b.needsRefresh&&!geometryChanged&&prior==b.bank&&b.lastBlood==liveBlood._get())return;
    b.needsRefresh=false;b.lastBlood=liveBlood._get();
    std::vector<const CSkeletonX*> present;
    auto walk=[&](const auto& list){for(auto* v:list){
        auto* skin=fast_dynamic_cast<CSkeletonX*>(v);auto* visual=fast_dynamic_cast<dxRender_Visual*>(v);
        if(!skin||!visual||!visual->shader)continue;present.push_back(skin);
        auto& child=b.children[skin];const auto remap=skin->PainterBurnRemap();const unsigned slot=skin->PainterBurnSourceChild();
        // A wall witness belongs to a specific damage revision. Authored material
        // roles and face tokens belong to the original topology plus this triangle remap.
        const auto geometry=geometryOf(b.source);
        auto matches=[&](const std::shared_ptr<Map>& map){
            return map && geometryOf(map->source)==geometry && map->original==remap && map->slot==slot;
        };
        if(!matches(child.faceMap)){
            std::shared_ptr<Map> prepared;
            if(m_runtime_geometry){
                auto found=m_runtime_geometry->burnFaces.find(skin);
                if(found!=m_runtime_geometry->burnFaces.end() && matches(found->second))prepared=found->second;
            }
            if(!prepared && child.map && matches(child.map) && child.map->ready.load(std::memory_order_acquire))
                prepared=child.map;
            if(!prepared && (!b.hasRenderedBurn || matches(child.pendingFaces))){
                if(!matches(child.pendingFaces)){
                    auto map=std::make_shared<Map>();map->source=geometry;map->original=remap;map->slot=slot;
                    if(svc.request(map))child.pendingFaces=map;
                }
                if(!child.pendingFaces || !child.pendingFaces->ready.load(std::memory_order_acquire)){b.needsRefresh=true;continue;}
                prepared=child.pendingFaces;
            }
            child.pendingFaces.reset();
            // Newly exposed original mesh or synchronous replacement. Its exact
            // remap must be present on the first draw of an already burning actor.
            if(!prepared){
                prepared=buildFaces(HW.pDevice,geometry,slot,remap.get());
                if(!prepared){b.needsRefresh=true;continue;}
                prepared->original=remap;
            }
            child.faceMap=std::move(prepared);child.faces=ref_texture{};child.state=ref_texture{};
        }
        if(!child.faces){
            if(child.faceMap->faces.srv){
                if(!bindRef(child.faces,child.faceMap.get(),"faces",child.faceMap->faces.srv.Get())){b.needsRefresh=true;continue;}
            }else child.faces=b.empty;
        }
        if(!child.map || child.map->source!=(b.source->walls.empty()?geometry:b.source) || !matches(child.map)){
            child.map.reset();child.walls=b.empty;child.state=ref_texture{};
            if(b.source->walls.empty())child.map=child.faceMap;
            else{
                auto map=std::make_shared<Map>();map->source=b.source;map->original=remap;map->slot=slot;
                map->wallsOnly=true;
                if(svc.request(map))child.map=std::move(map);
                else b.needsRefresh=true;
            }
        }
        if(child.map && child.map->ready.load(std::memory_order_acquire)){
            if(child.map->walls.srv && child.walls==b.empty){
                ref_texture current;
                if(bindRef(current,child.map.get(),"walls",child.map->walls.srv.Get())){
                    child.walls=current;child.state=ref_texture{};
                }else b.needsRefresh=true;
            }
        }else b.needsRefresh=true;
        if(child.applied&&visual->shader._get()==child.applied._get()&&child.blood==liveBlood&&child.state==b.bank->wrapped)continue;
        if(!child.applied||visual->shader._get()!=child.applied._get()){
            if(child.original._get()!=visual->shader._get())child.unsupported=false;
            child.original=visual->shader;
        }
        if(child.unsupported)continue;
        // First exposure is paced. A newly published replacement of a burning actor must have its twin immediately.
        // A compiling pair stays in the resource queue. Another ready child may advance.
        static u32 workFrame=~0u,installedToday=0;
        if(workFrame!=Device.dwFrame){workFrame=Device.dwFrame;installedToday=0;}
        if(!b.hasRenderedBurn && installedToday>=2u){b.needsRefresh=true;continue;}
        ref_shader next;bool hasInterior=false;painter_torn_draw::Binding torn;
        auto status=clone(child.original,b.bank->wrapped,child.faces,child.walls,liveBlood,b.baked?b.baked:b.empty,next,hasInterior,torn);
        if(status==Clone::Unsupported){child.unsupported=true;continue;}
        if(status==Clone::Done){++installedToday;child.torn=std::move(torn);child.interior=hasInterior;child.applied=next;child.blood=liveBlood;child.state=b.bank->wrapped;visual->shader=next;}
        else if(status==Clone::Pending)b.needsRefresh=true;
    }};
    walk(children);walk(children_invisible);
    for(auto j=b.children.begin();j!=b.children.end();)if(std::find(present.begin(),present.end(),j->first)==present.end())j=b.children.erase(j);else ++j;
}
ID3D11PixelShader* CKinematics::PainterBurnDraw(const CSkeletonX* skin,unsigned firstPrimitive){
    if(!m_burn_binding||!m_burn_binding->bank)return nullptr;
    auto* meta=RCache.get_c("painter_burn_meta");auto* draw=RCache.get_c("painter_burn_draw");auto* bake=RCache.get_c("painter_burn_bake");
    if(!meta&&!draw&&!bake)return nullptr;
    const painter_torn_draw::Binding* torn=nullptr;
    std::array<float,4> m{};unsigned count=0,offset=0,walls=0;bool interior=false;
    if(m_burn_binding){auto& b=*m_burn_binding;auto it=b.children.find(skin);
        if(it!=b.children.end() && b.bank && it->second.applied && it->second.faceMap){
            const auto& child=it->second;
            const auto& face=*child.faceMap;
            if(PainterBurnGpu::geometryOf(face.source)==PainterBurnGpu::geometryOf(b.source) && face.original==skin->PainterBurnRemap() &&
               face.slot==skin->PainterBurnSourceChild()){
                m=b.bank->gpu.meta(PainterBurnGpu::visibleOwner(b),PainterBurnGpu::service().now);interior=child.interior;
                if(child.state==b.bank->wrapped && child.torn.ordinary)torn=&child.torn;
                count=firstPrimitive==~0u?0u:face.count;
                if(m[0]>0.f)b.hasRenderedBurn=true;
                if(!b.priorSplit.spawn && child.map && child.map->source==b.source && child.walls!=b.empty &&
                    child.map->ready.load(std::memory_order_acquire))walls=child.map->wallMask;
            }
        }
    }
    if(meta)RCache.set_c(meta,m[0],m[1],m[2],m[3]);
    if(draw)RCache.set_c(draw,float(count),float(offset),float(interior),float(walls));
    // the baked recession: walked instead of the trace once this body's map is made (painter_burn_recession.h)
    const bool baked=bake && m_burn_binding->bakeValid && m_burn_binding->bakedBank==m_burn_binding->bank.get() &&
        m_burn_binding->bakedOwner==PainterBurnGpu::visibleOwner(*m_burn_binding) && m[0]>0.f;
    if(bake)RCache.set_c(bake,baked?1.f:0.f,PainterFire::thermalViewPossible()?1.f:0.f,0.f,0.f);
    auto& actor=*m_burn_binding;
    if(meta&&draw&&m[0]>0.f&&(count>offset||walls)&&actor.bank&&interior&&
       (actor.tracedSerial!=actor.bank->serial||actor.tracedRevision!=actor.bank->revision)&&PainterBurnTraceEnabled()){
        actor.tracedSerial=actor.bank->serial;actor.tracedRevision=actor.bank->revision;
        Msg("[painter_burn_trace] stage=draw_bound lineage=%llu spawn=%llu generation=%llu serial=%llu revision=%llu faces=%u offset=%u wall_mask=%u fitted_interior=1",
            actor.lineage,actor.owner.spawn,actor.owner.generation,actor.tracedSerial,actor.tracedRevision,count,offset,walls);
    }
    if(!torn || !meta || !bake)return nullptr;
    const unsigned tag=unsigned(m[1]);
    const bool known=tag>0 && tag<actor.bank->boundKnown.size() && actor.bank->boundKnown[tag];
    const bool live=known && actor.bank->boundLive[tag];
    return torn->choose(RCache.painterBoundPS(),baked,live,known,RCache.get_RT(1));
}
// Original progressive children only have authenticated authored face tokens in window 0.
bool CKinematics::PainterBurnNeedsFullWitness(const CSkeletonX* skin,const Shader* shader) const {
    if(!m_burn_binding || !m_burn_binding->bank || !shader || !RCache.get_c("painter_burn_draw"))return false;
    const auto& b=*m_burn_binding;
    const auto it=b.children.find(skin);
    if(it==b.children.end() || it->second.applied._get()!=shader)return false;
    const auto& faces=it->second.faceMap;
    if(!faces || !faces->count || !faces->ready.load(std::memory_order_acquire) ||
       faces->slot!=skin->PainterBurnSourceChild() || faces->original!=skin->PainterBurnRemap() ||
       PainterBurnGpu::geometryOf(faces->source)!=PainterBurnGpu::geometryOf(b.source))return false;
    return b.bank->gpu.meta(PainterBurnGpu::visibleOwner(b),PainterBurnGpu::service().now)[0]>0.f;
}
static std::atomic<unsigned> renderBurnStamp{0};
bool PainterBurnPublishNative(std::shared_ptr<const painter_burn::Delta> d){
    const unsigned stamp=PainterBodyMasterStamp();
    if(!(stamp&1u))return true;
    if(renderBurnStamp.load(std::memory_order_acquire)!=stamp)return false;
    return PainterBurnGpu::service().submit(std::move(d));
}
void PainterBurnHostClock(u32 t){if(auto* s=PainterBurnGpu::current()){s->hostTick=t;s->hostClock=true;}}
u32 PainterBurnNativeEpoch(){auto* s=PainterBurnGpu::current();return s?s->deviceEpoch.load():1;}
void PainterBurnFramePump(){
    const unsigned stamp=PainterBodyMasterStamp();
    if(renderBurnStamp.load(std::memory_order_acquire)!=stamp){
        auto& pending=PainterBurnBake::jobs();
        for(auto& j:pending)if(j.actor)j.actor->bakeQueued=false;
        pending.clear();
        if(auto* svc=PainterBurnGpu::current())svc->shutdown();
        renderBurnStamp.store(stamp,std::memory_order_release);
    }
    if(!(stamp&1u))return;
    if(auto* s=PainterBurnGpu::current()){if(s->outstanding)s->start(HW.pDevice);s->pump();}PainterBurnBake::run();}
bool PainterWarmBurnBakeProgram(){return PainterBurnBake::ready();}
void PainterBurnShutdown(){if(auto* s=PainterBurnGpu::current())s->shutdown();}
#endif
