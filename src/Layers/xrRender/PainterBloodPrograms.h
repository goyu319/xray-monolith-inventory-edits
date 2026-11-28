#pragma once
#ifdef USE_DX11
// Baked blood: the painter_blood_ aliases of the programs a bloodied body draws with.
// shaderslim (2026-11-13): created per RECIPE IN USE, not as one complete shipped list. The list named every alias there is - 36
// vertex names at five skinning modes and 249 pixel names, the first-person HUD and light-mapped level recipes among them, which no
// body can ever draw with - and the first launch compiled all of it behind the loading screen. Now:
//  - a model's own recipes are noted where its skinned children load (PainterBloodWarmPrograms) and where a Painter body binds
//    (RuntimeShaderCache::Insert): the alias pair of each of its passes, at the pass's own skinning mode and MSAA sample;
//  - the masked / interior aliases of a Painter body's recipes are asked for with their plain twins (the removal clone's warm
//    queue, PainterBloodRequestPrograms);
//  - PainterBloodBake::clone asks for a pair it does not find (it retries on later frames, as before).
// A request only records names (any thread). pump() creates on the owner thread: everything behind a loading screen, one pair per
// frame in play. A recipe without alias files (HUD, level geometry) is dropped there, never compiled.
#include "PainterBodyTess.h"
#include "../../xrCore/PainterBurnTrace.h"
#include "PainterSoftShader.h"
#include "ResourceManager.h"
#include "dxRenderDeviceRender.h"
#include <deque>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <tuple>
#include <utility>
namespace PainterBloodPrograms {
struct Pair {
    std::string vertex;int skin=4;std::string pixel;int sample=-1; // alias names, skinning mode, MSAA sample (-1 none)
    bool operator<(const Pair& o)const{return std::tie(vertex,skin,pixel,sample)<std::tie(o.vertex,o.skin,o.pixel,o.sample);}
};
struct Cache {
    std::map<std::pair<std::string,int>,ref_vs> vertices;
    std::map<std::pair<std::string,int>,ref_ps> pixels;
    std::set<Pair> known;      // every pair ever asked for (kept by clear(): asked again after the handles are dropped)
    std::deque<Pair> pending;  // asked for, not created yet
    std::mutex lock;           // known / pending: a request may come from a model-load thread
    u32 pumpFrame=~0u;
    // Body burning: the program pairs whose burn twin was asked for (requestBurn: names only, any thread, under `lock`).
    // burnPending waits for the owner thread, which makes pairs of the names behind a loading screen (burnTake, burnPair).
    std::set<Pair> burnKnown;
    std::deque<Pair> burnPending;
};
// Creation and lookup: owner thread only. Explicit shutdown precedes ResourceManager destruction.
inline Cache& cache(){static auto* value=new Cache;return *value;}
inline bool file(const std::string& name,const char* suffix){
    string_path path;strconcat(sizeof(path),path,"r3\\",name.c_str(),suffix);return FS.exist("$game_shaders$",path);
}
// Body burning. A burning body draws every pass with the burn twin of its program pair (PainterBurnGpu::clone,
// PainterBurnBridge_impl.h). This is the one name rule for the twin; the clone and the warm-up both use it.
// vs / ps: the pass's programs without skinning / sample suffix. vertex / pixel: the twin (other strings than vs / ps).
// False: this pair has no twin (it is one already, or the twin's files are not installed). Looks at files: owner thread.
inline bool burnNames(const std::string& vs,const std::string& ps,std::string& vertex,std::string& pixel){
    if(vs.empty()||ps.empty()||ps.find("painter_burn_")==0)return false;
    vertex=vs;pixel="painter_burn_"+ps;
    const bool direct=file(pixel,".ps");
    if(!direct)pixel=ps.find("painter_blood_")==0?"painter_burn_"+ps:"painter_burn_painter_blood_"+ps;
    if(!direct&&vs.find("painter_blood_")!=0&&file("painter_blood_"+vs,".vs"))vertex="painter_blood_"+vs;
    if(vertex=="painter_wall_bounds_model")vertex="painter_vmi_model";
    if(vertex=="painter_wall_bounds_dent_model")vertex="painter_vmid_model";
    return file(pixel,".ps")&&file(vertex,".vs");
}
// Ask for the burn twin of one program pair (names as the programs have them, no skinning / sample suffix), any thread.
// Only the names are kept: the owner thread looks at the files, behind a loading screen (burnTake, burnPair).
inline void requestBurn(const std::string& vs,int skinning,const std::string& ps,int sample){
    if(!(PainterBodyMasterStamp()&1u))return;
    if(vs.empty()||ps.empty()||skinning<0||skinning>4||ps.find("painter_burn_")==0)return;
    Pair base;base.vertex=vs;base.skin=skinning;base.pixel=ps;base.sample=sample;
    auto& c=cache();std::lock_guard<std::mutex> guard(c.lock);
    if(c.burnKnown.insert(base).second)c.burnPending.push_back(base);
}
// Owner thread: the pairs asked for since the last call; every: all that were ever asked for.
inline void burnTake(std::deque<Pair>& bases,bool every){
    auto& c=cache();std::lock_guard<std::mutex> guard(c.lock);
    if(every)bases.assign(c.burnKnown.begin(),c.burnKnown.end());else bases.assign(c.burnPending.begin(),c.burnPending.end());
    c.burnPending.clear();
}
// Owner thread: the burn twin of one program pair goes on the list of pairs to create. False: it has none.
inline bool burnPair(const std::string& vs,int skinning,const std::string& ps,int sample){
    Pair pair;pair.skin=skinning;pair.sample=sample;
    if(skinning<0||skinning>4||!burnNames(vs,ps,pair.vertex,pair.pixel))return false;
    auto& c=cache();std::lock_guard<std::mutex> guard(c.lock);
    if(c.known.insert(pair).second)c.pending.push_back(pair);
    return true;
}
inline void warmVertex(const std::string& name,int skin){
    auto& rows=cache().vertices;const auto key=std::make_pair(name,skin);
    if(rows.count(key)||!file(name,".vs"))return;
    const int old=Engine.External.GetSkinningMode();Engine.External.SetSkinningMode(skin);
    ref_vs value;{PainterSoftShaderGuard guard;value=DEV->_CreateVS(name.c_str());}
    Engine.External.SetSkinningMode(old);
    if(value&&value->vs)rows.emplace(key,value);
}
inline void warmPixel(const std::string& name,int sample){
    auto& rows=cache().pixels;const auto key=std::make_pair(name,sample);
    if(rows.count(key)||!file(name,".ps"))return;
    const int old=::Render->m_MSAASample;::Render->m_MSAASample=sample;
    ref_ps value;{PainterSoftShaderGuard guard;value=DEV->_CreatePS(name.c_str());}
    ::Render->m_MSAASample=old;
    if(value&&value->ps)rows.emplace(key,value);
}
// Ask for the blood aliases of one program pair (the programs' own names without skinning / sample suffix), any thread.
// PainterBloodBake::clone's rule: shadows and pre-passes keep their pass, so they have no alias.
inline void requestPair(const std::string& vs,int skinning,const std::string& ps,int sample){
    if(vs.empty()||ps.empty()||skinning<0||skinning>4)return;
    requestBurn(vs,skinning,ps,sample); // body burning: this pair's burn twin (a shadow pass and a pre-pass have one too)
    if(ps=="dumb"||ps.find("shadow")!=std::string::npos||ps.find("_atoc")!=std::string::npos)return;
    if(ps.find("painter_blood_")==0||vs.find("painter_blood_")==0)return; // created blood-capable by the removal clone: no alias
    Pair pair;pair.vertex="painter_blood_"+vs;pair.skin=skinning;pair.pixel="painter_blood_"+ps;pair.sample=sample;
    auto& c=cache();std::lock_guard<std::mutex> guard(c.lock);
    if(c.known.insert(pair).second)c.pending.push_back(pair);
}
// Every pass of a material: the alias pair PainterBloodBake::clone would look up for it (its own name rule).
inline void note(const ref_shader& source){
    if(!source)return;
    for(unsigned n=0;n<SHADER_ELEMENTS_MAX;++n){const auto& e=source->E[n];if(!e)continue;
        for(const auto& p:e->passes){
            if(!p||!p->vs||!p->ps||!p->vs->cName.size()||!p->ps->cName.size())continue;
            std::string vs=*p->vs->cName,ps=*p->ps->cName;int sample=-1;
            if(ps.size()>2&&ps[ps.size()-2]=='_'&&ps.back()>='0'&&ps.back()<='7'){sample=ps.back()-'0';ps.resize(ps.size()-2);}
            if(p->vs->skinning>=0&&vs.size()>2&&vs[vs.size()-2]=='_'&&vs.back()>='0'&&vs.back()<='4')vs.resize(vs.size()-2);
            if(painter_body_tess::fromPass(*p,vs,ps)!=painter_body_tess::Result::Unsupported)requestPair(vs,p->vs->skinning,ps,sample);
        }}
}
// Owner thread: create what was asked for - all of it behind a loading screen, one new pair per frame in play. A pair whose
// alias files do not both exist (a recipe no body draws with) is dropped. Returns the number of pairs still waiting.
inline unsigned pump(bool all){
    auto& c=cache();
    if(all||c.pumpFrame!=Device.dwFrame){
        c.pumpFrame=Device.dwFrame;
        // Body burning: the burn twins among the pairs are counted. Behind a loading screen one line says how many are to
        // be made (a twin that was never built is compiled here, about a second each) and one how long it took. A twin
        // made in play - one the warm-up did not know - gets a line of its own, so a missed variant shows in the log.
        const auto burnTwin=[](const Pair& p){return p.pixel.find("painter_burn_")==0;};
        unsigned burnMade=0;u64 burnTicks=0;
        if(all){
            unsigned waiting=0;
            {std::lock_guard<std::mutex> guard(c.lock);for(const auto& p:c.pending)if(burnTwin(p))++waiting;}
            if(waiting)Msg("[painter_burn] %u body-burn shader pairs to make behind the loading screen (about a second each the first time after their files changed)",waiting);
        }
        for(;;){
            Pair job;
            {std::lock_guard<std::mutex> guard(c.lock);if(c.pending.empty())break;job=c.pending.front();c.pending.pop_front();}
            if(burnTwin(job) && !(PainterBodyMasterStamp()&1u)){
                std::lock_guard<std::mutex> guard(c.lock);c.known.erase(job);continue;
            }
            if(!file(job.vertex,".vs")||!file(job.pixel,".ps"))continue;
            const bool hadVertex=c.vertices.count(std::make_pair(job.vertex,job.skin))!=0,hadPixel=c.pixels.count(std::make_pair(job.pixel,job.sample))!=0;
            const bool had=hadVertex&&hadPixel;
            const bool burn=!had&&burnTwin(job);
            const u64 began=burn?CPU::QPC():0;
            warmVertex(job.vertex,job.skin);warmPixel(job.pixel,job.sample);
            if(burn){
                const u64 ticks=CPU::QPC()-began;
                const bool made=c.vertices.count(std::make_pair(job.vertex,job.skin))!=0&&c.pixels.count(std::make_pair(job.pixel,job.sample))!=0;
                if(!made)Msg("! [painter_burn] shader could not be made: %s (vertex program %s, skinning %d, sample %d)",job.pixel.c_str(),job.vertex.c_str(),job.skin,job.sample);
                else if(all){++burnMade;burnTicks+=ticks;}
                else Msg("[painter_burn] shader made in play: %s (%s; vertex program %s, skinning %d, sample %d; %.0f ms)",job.pixel.c_str(),
                    hadPixel?"only its vertex program was new":"its pixel program was new",job.vertex.c_str(),job.skin,job.sample,
                    double(ticks)*1000./double(CPU::qpc_freq));
            }
            if(!all&&!had)break;
        }
        if(burnMade)Msg("[painter_burn] %u body-burn shader pairs made behind the loading screen in %.0f ms",burnMade,double(burnTicks)*1000./double(CPU::qpc_freq));
    }
    std::lock_guard<std::mutex> guard(c.lock);return unsigned(c.pending.size());
}
inline bool find(const std::string& vs,int skin,const std::string& ps,int sample,ref_vs& v,ref_ps& p){
    const auto vi=cache().vertices.find({vs,skin});const auto pi=cache().pixels.find({ps,sample});
    if(vi==cache().vertices.end()||pi==cache().pixels.end())return false;
    v=vi->second;p=pi->second;return true;
}
// Drops the handles (device loss, level unload); every pair asked for so far is asked for again.
inline void clear(){
    auto& c=cache();c.vertices.clear();c.pixels.clear();
    std::lock_guard<std::mutex> guard(c.lock);c.pending.assign(c.known.begin(),c.known.end());c.pumpFrame=~0u;
}
}
#endif
