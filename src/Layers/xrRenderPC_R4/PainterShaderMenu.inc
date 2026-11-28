#include "PainterMenuCompiler.h"
#include "PainterShaderCost.h"
#include <tuple>

void PainterShaderLoadProgress(unsigned done,unsigned total);

namespace painter_startup {
using Batch=painter_compile_batch::Batch;
using Job=painter_compile_batch::Job;
struct Pass {
    std::string name,stage;
    int skin,sample;
    unsigned cost;
};
struct Pending {std::string path,work;Pass pass;};
struct State {
    painter_menu::Service workers;
    std::vector<Pass> passes;
    std::vector<Pending> checking;
    std::map<std::string,painter_compile_batch::Blob> exteriorResults;
    size_t next=0,check=0;
    unsigned earlySaved=0;
    u64 keyCheck=0;
    std::string profile;
    bool started=false;
};
static State& state(){static State s;return s;}
static thread_local std::vector<Pass>* collecting=nullptr;
static thread_local std::string* profileProbe=nullptr;

static void probeMacroSet(const char* nativeKey,const D3D_SHADER_MACRO* defs) {
    auto& key=*profileProbe;
    key=nativeKey;key+='|';
    for(const auto* d=defs;d && d->Name;++d){
        key.append(d->Name);key+='\0';key+=d->Definition?'1':'0';
        if(d->Definition)key.append(d->Definition);
        key+='\0';
    }
}
struct Capture {
    int skin=Engine.External.GetSkinningMode(),sample=::Render->m_MSAASample;
    painter_compile_batch::Batch* previous=painter_compile_load::active;
    explicit Capture(Batch& b){painter_compile_load::active=&b;}
    ~Capture(){painter_compile_load::active=previous;Engine.External.SetSkinningMode(skin);::Render->m_MSAASample=sample;}
};
static std::string currentProfile() {
    Batch b;Capture context(b);
    Engine.External.SetSkinningMode(-1);::Render->m_MSAASample=-1;
    std::string key;profileProbe=&key;
    struct Reset {~Reset(){profileProbe=nullptr;}} reset;
    void* output=nullptr;DWORD empty=0;
    ::Render->shader_compile("painter_settings_probe",&empty,sizeof(empty),"main","ps_5_0",D3DCOMPILE_PACK_MATRIX_ROW_MAJOR,output);
    // Include selection controls outside the native macro vector.
    key.push_back(char(PainterMaskOnlyMode()));key.push_back(char(PainterBurnOn()));
    key.push_back(char(RImplementation.o.advancedpp && ps_r2_ls_flags.test(R2FLAG_DETAIL_BUMP)));
    key.push_back(char(psDeviceFlags2.test(rsPrecompiledShaders)));
    key.push_back(char(RImplementation.o.mrtmixdepth));
    key.push_back(char(RImplementation.o.albedo_wo));
    key.push_back(char(RImplementation.o.dx11_hdr10));
    key.push_back(char(RImplementation.o.dx10_msaa));
    key.push_back(char(RImplementation.o.dx10_msaa_samples));
    key.push_back(char(HW.FeatureLevel&255));key.push_back(char((HW.FeatureLevel>>8)&255));
    return key;
}
static void emit(const char* name,const char* stage,int skin,int sample) {
    const unsigned samples=RImplementation.o.dx10_msaa?unsigned(RImplementation.o.dx10_msaa_samples):1u;
    collecting->push_back({name,stage,skin,sample,painter_shader_cost::estimate(name,stage,skin,sample,samples)});
}
static std::vector<Pass> list() {
    std::vector<Pass> out;collecting=&out;
    struct Reset {~Reset(){collecting=nullptr;}} reset;
    string_path path;FS.update_path(path,"$game_shaders$",::Render->getShaderPath());
    FS_FileSet files;FS.file_list(files,path,FS_ListFiles|FS_RootOnly,"painter_*.vs,painter_*.ps,painter_*.cs");
    const painter_material_warm::Settings settings={PainterMaskOnlyMode(),PainterBurnOn(),
        RImplementation.o.dx10_msaa_alphatest==CRender::MSAA_ATEST_DX10_0_ATOC,
        RImplementation.o.advancedpp && ps_r2_ls_flags.test(R2FLAG_DETAIL_BUMP),
        RImplementation.o.dx10_msaa?int(RImplementation.o.dx10_msaa_samples):1};
    for(const auto& f:files)painter_material_warm::visitLiveSettings(f.name.c_str(),settings,
       [](const char* stage,const std::string& name,int skin,int sample){emit(name.c_str(),stage,skin,sample);});
    PainterFire::VisitWarmPrograms(emit);
    PainterVisitWarmStock(emit);
    std::set<std::tuple<std::string,std::string,int,int>> seen;
    out.erase(std::remove_if(out.begin(),out.end(),[&](const Pass& p){return !seen.emplace(p.name,p.stage,p.skin,p.sample).second;}),out.end());
    std::stable_sort(out.begin(),out.end(),[](const Pass& a,const Pass& b){return a.cost>b.cost;});
    return out;
}
static void capture(Batch& b,const Pass& p) {
    Capture ctx(b);
    painter_compile_load::request(p.name.c_str(),p.stage.c_str(),p.skin,p.sample);
}
static bool ordinary(const Job& j){return j.status==E_PENDING && (j.exterior.entry || !j.requiresRendererCompiler());}
static painter_menu::Input input(const Batch& b,const Job& j) {
    painter_menu::Input out;
    if(j.exterior.entry){
        // The renderer has checked the source for this exact pass and produced canonical stand-alone HLSL.
        out.name=j.exterior.entry->normal;
        out.path=std::string("@exterior/")+j.exterior.entry->canonical;
        out.entry=j.entry;out.target=j.target;out.flags=j.flags;
        out.source=j.exterior.source;
        return out;
    }
    out.path=j.path;out.name=j.name;out.entry=j.entry;out.target=j.target;out.flags=j.flags;out.sourceCrc=j.sourceCrc;out.source=j.source;
    for(const auto& def:j.defs)out.defs.push_back({def.name,def.value,def.hasValue});
    for(const auto& name:j.includeNames)out.includes.push_back({name,b.includes.at(name)});
    return out;
}
static bool accept(Batch& b,Job& j) {
    auto& s=state();
    const auto current=input(b,j);
    if(j.exterior.entry){
        const auto saved=s.exteriorResults.find(current.path);
        if(saved!=s.exteriorResults.end()){
            j.code=saved->second;j.status=S_OK;return true;
        }
    }
    painter_menu::Result result;
    if(!s.workers.take(current,result) || FAILED(result.status) ||
       !painter_compile_batch::dxbc(result.bytes.data(),result.bytes.size()))return false;
    if(j.exterior.entry && !painter_surface_selector::valid(j.exterior,result.bytes.data(),result.bytes.size()))return false;
    ID3DBlob* raw=nullptr;
    if(FAILED(D3DCreateBlob(result.bytes.size(),&raw)) || !raw)return false;
    std::memcpy(raw->GetBufferPointer(),result.bytes.data(),result.bytes.size());
    j.code=painter_compile_batch::own(raw);j.status=S_OK;
    if(j.exterior.entry)s.exteriorResults.emplace(current.path,j.code);
    return true;
}
template<class Visitor> static void visitLibraries(Job& j,const Visitor& use) {
    if(j.torn.ready()) for(auto& part:j.torn.parts)use(part);
    if(j.burnedTorn.ready()) for(auto& part:j.burnedTorn.parts)use(part);
    if(j.burnedShadow.ready()) for(auto& part:j.burnedShadow.parts)use(part);
}
static painter_menu::Input libraryInput(const painter_tear_modules::Library& library,unsigned flags) {
    painter_menu::Input out;
    const auto digest=painter_surface_selector::hash(library.source.data(),library.source.size());
    if(digest.empty())return out;
    out.name="painter_shader_library";out.path="@library/"+std::to_string(flags)+"/"+digest;
    out.target="lib_5_0";out.flags=flags;out.source=library.source;
    return out;
}
static void queueLibraries(Job& j,unsigned cost) {
    visitLibraries(j,[&](const painter_tear_modules::SharedLibrary& part){
        auto in=libraryInput(*part,j.flags);
        if(!in.path.empty())state().workers.submit(std::move(in),cost);
    });
}
static void acceptLibraries(Job& j) {
    visitLibraries(j,[&](const painter_tear_modules::SharedLibrary& part){
        if(part->binary)return;
        auto in=libraryInput(*part,j.flags);
        painter_menu::Result r;
        if(!in.path.empty() && state().workers.take(in,r) && SUCCEEDED(r.status)
           && painter_compile_batch::dxbc(r.bytes.data(),r.bytes.size()))part->adopt(r.bytes);
    });
}

static void early(const Pass& p) {
    auto& s=state();Batch b;capture(b,p);
    for(auto& j:b.jobs){
        queueLibraries(j,p.cost);
        if(!ordinary(j))continue;
        auto request=input(b,j);
        const auto work=request.path;
        if(s.workers.submit(std::move(request),p.cost)){
            auto found=std::find_if(s.checking.begin(),s.checking.end(),[&](const Pending& e){return e.path==j.path;});
            if(found==s.checking.end())s.checking.push_back({j.path,work,p});
        }
    }
}
static bool publishOne() {
    auto& s=state();
    // Re-open the current provider only for a completed result. Keep its exact active inputs as the publication witness.
    for(size_t walked=0;walked<s.checking.size();++walked){
        const size_t i=s.check++%s.checking.size();
        if(!s.workers.ready(s.checking[i].work) && !s.exteriorResults.count(s.checking[i].work))continue;
        Pending p=std::move(s.checking[i]);s.checking[i]=std::move(s.checking.back());s.checking.pop_back();
        Batch b;capture(b,p.pass);
        for(auto& j:b.jobs)if(j.path==p.path && ordinary(j) && accept(b,j) && painter_compile_load::publish(j))++s.earlySaved;
        return true;
    }
    return false;
}
static void clear(State& s){
    s.exteriorResults.clear();s.passes.clear();s.checking.clear();s.check=s.next=0;s.earlySaved=0;s.keyCheck=0;s.profile.clear();s.started=false;
}
static void begin(const std::string& key){
    auto& s=state();s.workers.cancel();clear(s);s.profile=key;s.passes=list();s.started=true;s.keyCheck=CPU::QPC();
    Msg("[painter_shader_startup] phase=start requests=%u",unsigned(s.passes.size()));
}
static void feed(unsigned budgetUs){
    auto& s=state();const u64 started=CPU::QPC();const u64 limit=started+(CPU::qpc_freq*budgetUs)/1000000;
    do {
        if(s.next==s.passes.size())break;
        early(s.passes[s.next++]);
    } while(CPU::QPC()<limit);
}
}
void PainterStartShaderWork(){
    if(!HW.pDevice || g_dedicated_server || painter_compile_load::active)return;
    using namespace painter_startup;
    const auto key=currentProfile();auto& s=state();
    if(!s.started || s.profile!=key)begin(key);
    // Front-load dependency reads while the renderer is already bringing up its resources.
    ShaderClosureLoadScope fresh;
    while(s.next<s.passes.size())early(s.passes[s.next++]);
}
void PainterTickShaderWork(){
    using namespace painter_startup;
    auto& s=state();
    if(!s.started || !HW.pDevice || g_dedicated_server || painter_compile_load::active)return;
    const u64 now=CPU::QPC();
    if(now-s.keyCheck>=CPU::qpc_freq/4){const auto key=currentProfile();s.keyCheck=now;if(key!=s.profile)begin(key);}
    // Regular cache writes and readback can stall a visible menu frame on a busy filesystem.
    // Keep completed bytecode in the service until a level or controlled renderer shutdown.
    feed(750);
}
void PainterStopShaderWork(){
    using namespace painter_startup;auto& s=state();
    if(s.started && HW.pDevice && !painter_compile_load::active && s.profile==currentProfile()){
        const auto stopAt=CPU::QPC()+CPU::qpc_freq/8;
        ShaderClosureLoadScope fresh;
        while(CPU::QPC()<stopAt && publishOne()){}
        if(s.earlySaved)Msg("[painter_shader_startup] phase=exit_saved records=%u",s.earlySaved);
    }
    s.workers.cancel();clear(s);
}

