// Compiled as part of FSkinned.cpp: direct buffers, no model/OGF reader.
#include "../../Include/xrRender/RuntimeSkinGeometry.h"
#include "RuntimeSkinValidation.h"
#ifdef USE_DX11
#include "SkeletonRuntimeGeometry.h" // RuntimeResolveChildShader, staged types
#endif
bool CSkeletonX_ST::BuildRuntime(const RuntimeSkinChild& data, CKinematics* owner, const dxRender_Visual& material)
{
#ifndef USE_DX11
    Msg("[runtime_geometry_reject] stage=child reason=renderer_not_R4");return false;
#else
    if(!owner){Msg("[runtime_geometry_reject] stage=child reason=null_owner");return false;}
    RuntimeSkinValidationError error;
    if(!ValidateRuntimeSkinChild(data,owner->LL_BoneCount(),HW.Caps.geometry.dwRegisters,error)){
        Msg("[runtime_geometry_reject] stage=child reason=%s child=%u vertex=%llu influence=%u registers=%u",error.reason,data.child_index,u64(error.vertex),error.influence,HW.Caps.geometry.dwRegisters);return false;}
    xr_vector<vertBoned4W> source(data.vertices.size());
    xr_vector<u16> used;
    xr_vector<u8> seen(owner->LL_BoneCount(),u8(0));
    used.reserve(owner->LL_BoneCount());
    u16 max_bone = 0;
    vis.box.invalidate();
    for (size_t i=0; i<data.vertices.size(); ++i)
    {
        const RuntimeSkinVertex& v=data.vertices[i];
        for (u32 k=0;k<4;++k){max_bone=_max(max_bone,v.bone[k]);
            if(v.weight[k]!=0.f && !seen[v.bone[k]]){seen[v.bone[k]]=1;used.push_back(v.bone[k]);}}
        auto& d=source[i];
        for (u32 k=0;k<4;++k) d.m[k]=v.bone[k];
        for (u32 k=0;k<3;++k) d.w[k]=v.weight[k];
        d.P=v.position;d.N=v.normal;d.T=v.tangent;d.B=v.binormal;d.u=v.uv.x;d.v=v.uv.y;
        vis.box.modify(v.position);
    }
    Type=MT_SKELETON_GEOMDEF_ST; Parent=owner; ChildIDX=data.child_index;
    receives_wallmarks=data.receives_wallmarks;
    RenderMode=RM_SKINNING_4B; RMS_bonecount=max_bone+1;
    vBase=iBase=0;vCount=u32(source.size());iCount=u32(data.indices.size());dwPrimitives=iCount/3;
    BonesUsed.create_unique(u32(used.size()),used.data());
    m_Indices.create_unique(u32(data.indices.size()),data.indices.data());
    if (FAILED(dx10BufferUtils::CreateIndexBuffer(&p_rm_Indices,data.indices.data(),iCount*sizeof(u16)))) {Msg("[runtime_geometry_reject] stage=child reason=index_buffer child=%u",data.child_index);return false;}
    HW.stats_manager.increment_stats_ib(p_rm_Indices);
    Vertices4W.create_unique(u32(source.size()),source.data());
    _Load_hw(*this,*Vertices4W);
    m_ProjectionSource.reset();m_ProjectionIndexBase=0;m_ProjectionIndexCount=iCount;
    SetWallmarkProvenance(data.material_source_child,{});
    dbg_name="runtime_visual_damage";dbg_id=data.child_index+1;flags=material.flags;hud=material.hud;skinning=4;
    dbg_shader_def=data.shader.size()?data.shader:material.dbg_shader_def;
    dbg_texture_def=data.texture.size()?data.texture:material.dbg_texture_def;
    painter_tissue=PainterTissueCache::Find(data.tissue_field);
    if(!PainterTissueCache::Bind(painter_tissue,*this))RuntimeResolveChildShader(*this,nullptr); // Cached handle, or compiles/caches the matching four-influence variant.
    vis.box.getsphere(vis.sphere.P,vis.sphere.R);
    if(!shader){Msg("[runtime_geometry_reject] stage=child reason=shader child=%u",data.child_index);return false;}
    return true;
#endif
}
#ifdef USE_DX11
#include "SkeletonRuntimeGeometry.h"
#include "../../Include/xrRender/RuntimeFaceLists.h"
// ---------------------------------------------------------------------------
// Renderer-wide shader handle cache (RuntimeShaderCache.h). Main thread only.
namespace {
struct RuntimeShaderKey
{
    shared_str shader, texture; // shader = definition cut at "$no_shadows"
    bool hud = false;
    s32 skinning = 0;
    bool operator<(const RuntimeShaderKey& o) const
    {
        if (shader._get() != o.shader._get()) return uintptr_t(shader._get()) < uintptr_t(o.shader._get());
        if (texture._get() != o.texture._get()) return uintptr_t(texture._get()) < uintptr_t(o.texture._get());
        if (hud != o.hud) return !hud;
        return skinning < o.skinning;
    }
    bool operator==(const RuntimeShaderKey& o) const
    {
        return shader.equal(o.shader) && texture.equal(o.texture) && hud == o.hud && skinning == o.skinning;
    }
};
struct RuntimeShaderCacheState
{
    xr_map<RuntimeShaderKey, RuntimeShaderEntry> entries;
    xr_vector<RuntimeShaderKey> pending; // FIFO
    // Clone programs (masked / channel) requested for recorded shaders: created one per frame, pinned until Clear().
    struct PendingProgram { xr_string vs, ps; int sample = -1; int skinning = 4; };
    xr_vector<PendingProgram> pending_programs;
    xr_vector<xr_string> known_programs;
    xr_vector<ref_vs> pinned_vs; xr_vector<ref_ps> pinned_ps;
    RuntimeShaderCacheStats stats;
    u32 warm_frame = u32(-1);
    bool painter_programs_requested = false; // R62: the first wound's channel programs, queued once per cache cycle
};
// Never destroyed: a static destructor would release ref_shaders after the
// resource manager is gone. CRender::destroy clears it explicitly.
RuntimeShaderCacheState& runtime_shader_cache()
{
    static RuntimeShaderCacheState* state = new RuntimeShaderCacheState();
    return *state;
}
bool runtime_shader_queued(const RuntimeShaderCacheState& c, const RuntimeShaderKey& key)
{
    for (const auto& queued : c.pending) if (queued == key) return true;
    return false;
}
// Queue one clone program pair (by name, skinning 4, MSAA sample) for the per-frame warm drain, once.
void runtime_shader_request_program(const xr_string& vs, const xr_string& ps, int sample, int skinning = 4)
{
    if (vs.empty() || ps.empty()) return;
    auto& c = runtime_shader_cache();
    const xr_string id = vs + "|" + ps + "|" + xr_string(std::to_string(sample).c_str()) + "|" + xr_string(std::to_string(skinning).c_str());
    if (std::find(c.known_programs.begin(), c.known_programs.end(), id) != c.known_programs.end()) return;
    c.known_programs.push_back(id);
    PainterBloodPrograms::requestBurn(vs.c_str(), skinning, ps.c_str(), sample); // body burning: this pair's burn twin goes with it
    RuntimeShaderCacheState::PendingProgram job; job.vs = vs; job.ps = ps; job.sample = sample; job.skinning = skinning;
    c.pending_programs.push_back(job);
}
// The clone programs of every pass of `shader` (what PainterRemovalMaterial / PainterChannelMaterial would create).
void runtime_shader_request_clone_programs(const ref_shader& shader)
{
    if (!shader._get()) return;
    PainterBloodNotePrograms(shader); // shaderslim: the blood aliases of this material's own passes
    // shaderslim: the interior pairs only while mask-only wounds are on, and only the jobs of its value (what a wound / a piece draws with)
    const int mode = PainterMaskOnlyMode();
    for (unsigned n = 0; n < SHADER_ELEMENTS_MAX; ++n)
    {
        const auto& element = shader->E[n]; if (!element) continue;
        for (const auto& pass : element->passes)
        {
            if (!pass) continue;
            // shaderslim: every pair the removal clone can create for this pass (exterior, dent twin, tear exterior, the shared
            // interior programs) and the blood alias of each (skinning 4, as the clone creates them)
            PainterRemovalMaterial::clonePrograms(*pass, mode, [](const xr_string& vs, const xr_string& ps, int sample) {
                runtime_shader_request_program(vs, ps, sample);
                PainterBloodRequestPrograms(vs.c_str(), 4, ps.c_str(), sample);
            });
            // shaderslim step 2: the channel pair draws wound GEOMETRY (analytic channels), which only values 0 / 1 of
            // painter_voxel_mask_only publish (runtime_shader_request_painter_programs below has the path): not asked for with 2 / 3
            PainterChannelMaterial::Programs names;
            if (mode < 2 && PainterChannelMaterial::programs(*pass, names)) runtime_shader_request_program(names.vs, names.ps, names.sample);
        }
    }
}
// R62: the programs every first wound clones (PainterChannelMaterial::programs of the anatomy material's deferred and shadow
// passes at skinning 4 without MSAA), queued at the first warm call behind a level's loading screen after a clear; the anatomy
// shaders' own passes add the exact variants (MSAA samples) when their fields are ready (PainterTissueCache::Warm).
void runtime_shader_request_painter_programs(RuntimeShaderCacheState& c)
{
    // shaderslim step 2: the channel pair is cloned only for a publication that carries analytic channel geometry
    // (SkeletonRuntimeGeometry: child->analytic = packed->analytic_channels; RuntimeBindRequiredFields: PainterChannelMaterial::clone
    // under if (analytic)). With painter_voxel_mask_only 2 / 3 no publication carries it: a body is always packed mask-only
    // (LiveDamageWorker: mask_only_off is set by a sever only below 2) and its pieces are mask pieces (LiveSeparationWorker:
    // maskPieces = mask_volume && !analytic_channels). So with 2 / 3 the pair is not compiled (painter_channel.ps 5.4 s +
    // painter_channel_shadow.ps 3.6 s of the first load); with 0 / 1 it is queued as before. Asked at every loading-screen warm
    // call (a pair already known is skipped), so a value lowered in the console is followed at the next loading screen; a wound
    // before that creates the pair on demand, as PainterChannelMaterial::clone always could.
    if (PainterMaskOnlyMode() < 2)
    {
        runtime_shader_request_program("painter_anatomy", "painter_channel", -1);
        runtime_shader_request_program("painter_channel_shadow", "painter_channel_shadow", -1);
    }
    if (c.painter_programs_requested) return;
    c.painter_programs_requested = true;
    // R63: the wallmark mask pairs (PainterWallmarkMask::Prepare at the first decal on a wounded body, skinning 0)
    runtime_shader_request_program("painter_wallmark_mask", "painter_wallmark_mask", -1, 0);
    runtime_shader_request_program("painter_wallmark_mask", "painter_wallmark_mask_owner", -1, 0);
    runtime_shader_request_program("painter_wallmark_mask", "painter_wallmark_mask_owner_only", -1, 0);
    // shaderslim: the decals of a piece with tears (PainterWallmarkMask: the first decal on such a piece compiled them in play)
    runtime_shader_request_program("painter_wallmark_mask", "painter_wallmark_mask_tear", -1, 0);
    runtime_shader_request_program("painter_wallmark_mask", "painter_wallmark_mask_tear_owner", -1, 0);
    // shaderslim: the shared interior programs (every material's wound walls / torn faces; the jobs of the mask-only value) do not
    // depend on a recipe: queued here, behind the first loading screen, so they never wait for the first Painter body to bind
    {
        const int mode = PainterMaskOnlyMode();
        static const char* const jobs[] = {"wall", "piece", "tear"};
        for (int j = 0; j < 3 && j < mode; ++j)
        {
            const xr_string deferred = xr_string("painter_blood_painter_vmi_") + jobs[j], shadow = xr_string("painter_vmi_shadow_") + (j == 1 ? "wall" : jobs[j]);
            const bool wallBounds = j == 0 && PainterRemovalMaterial::wallBoundsShaders();
            runtime_shader_request_program(wallBounds ? "painter_wall_bounds_model" : "painter_vmi_model", deferred, -1);
            runtime_shader_request_program(wallBounds ? "painter_wall_bounds_dent_model" : "painter_vmid_model", deferred, -1);
            runtime_shader_request_program("painter_vmi_shadow_model", shadow, -1); runtime_shader_request_program("painter_vmid_shadow_model", shadow, -1);
        }
    }
}
// R62: the loading screen is up (precache frames or the press-a-key wait): a long compile there is not a gameplay hitch.
bool runtime_shader_loading_screen()
{
    return load_screen_renderer.IsActive() || Device.dwPrecacheFrame != 0;
}
void runtime_shader_resolve_burn(bool prepareSwitches=false)
{
    if (!prepareSwitches && !PainterBurnOn()) return;
    const int mode = PainterMaskOnlyMode();
    static int resolved_mode = -1; // another value of painter_voxel_mask_only: other wound programs, every recipe once more
    std::deque<PainterBloodPrograms::Pair> bases;
    PainterBloodPrograms::burnTake(bases, prepareSwitches || mode != resolved_mode);
    resolved_mode = mode;
    for (const auto& base : bases)
    {
        PainterBloodPrograms::burnPair(base.vertex, base.skin, base.pixel, base.sample);
        PainterRemovalMaterial::ProgramPair pair;
        if (!PainterRemovalMaterial::programNames(base.vertex.c_str(), base.pixel.c_str(), pair)) continue;
        pair.sample = base.sample;
        PainterBloodPrograms::burnPair(base.vertex, 4, base.pixel, base.sample); // the recipe itself on a wounded body (skinning 4)
        auto queueMode = [&](int maskMode) {
            PainterRemovalMaterial::clonePairPrograms(pair, maskMode, [](const xr_string& vs,const xr_string& ps,int sample) {
                PainterBloodPrograms::burnPair(vs.c_str(),4,ps.c_str(),sample);
            });
        };
        if (prepareSwitches) {
            for (int maskMode=0; maskMode<=3; ++maskMode) queueMode(maskMode);
        } else queueMode(mode);
    }
}
}
#include "PainterMaterialWarmKeys.h"
extern bool PainterWarmBurnBakeProgram();

// Only the render owner mutates the resource manager, the skinning context and the MSAA sample.
// The same pinned stage handles satisfy subsequent ordinary clones, plus the blood clone's find().
// The same mounted program list feeds both bytecode preparation and owner-side handle retention.
void PainterVisitWarmStock(void(*emit)(const char*,const char*,int,int))
{
    if (!emit) return;
    auto mounted = [](const char* name, const char* stage) {
        string_path relative;
        strconcat(sizeof(relative), relative, ::Render->getShaderPath(), name, ".", stage);
        return FS.exist("$game_shaders$", relative) != nullptr;
    };
    auto add = [&](const char* name, const char* stage, int skin=-1, int sample=-1) {
        if (mounted(name,stage)) emit(name,stage,skin,sample);
    };

    add("heatvision","ps");
    add("effects_wallmark_blood","vs");
    add("effects_wallmark_blood","ps");
    add("painter_static_blood","ps");
    add("painter_static_blood_color","ps");
    add("lod","vs");
    add("lod","ps");
    add("combine_1","vs");
    add("combine_1_nomsaa","ps");
    if (RImplementation.o.dx10_msaa) {
        add("combine_1_msaa","ps");
        for (int s=0;s<int(RImplementation.o.dx10_msaa_samples);++s)
            add("combine_1_msaa","ps",-1,s);
    }

    add("models_scope_reticle","ps");
    add("models_scope_reticle_precise","ps");
    for (int skin=-1;skin<=4;++skin)
        add("models_scope_reticle","vs",skin);

    // The tree/lightmapped blenders choose existing leaf or wood programs by source texture,
    // alpha test and detail mode. These variants are already mounted before the level loads.
    string_path dir;
    FS.update_path(dir,"$game_shaders$",::Render->getShaderPath());
    FS_FileSet foliage;
    FS.file_list(foliage,dir,FS_ListFiles|FS_RootOnly,"painter_leaf_*.ps,painter_wood_deffer_*.ps");
    const bool atoc=RImplementation.o.dx10_msaa_alphatest==CRender::MSAA_ATEST_DX10_0_ATOC;
    for (const auto& entry:foliage) {
        const std::string filename=entry.name.c_str();
        if (filename.size()<4 || filename.compare(filename.size()-3,3,".ps")) continue;
        if (filename.find("_atoc_")!=std::string::npos && !atoc) continue;
        const std::string name=filename.substr(0,filename.size()-3);
        emit(name.c_str(),"ps",-1,-1);
    }
}

void PainterWarmStockPrograms()
{
    const int savedSkin=Engine.External.GetSkinningMode();
    const int savedSample=::Render->m_MSAASample;
    PainterVisitWarmStock([](const char* name,const char* stage,int skin,int sample) {
        Engine.External.SetSkinningMode(skin);
        ::Render->m_MSAASample=sample;
        if (*stage=='v') PainterBloodPrograms::warmVertex(name,skin);
        else PainterBloodPrograms::warmPixel(name,sample);
    });
    Engine.External.SetSkinningMode(savedSkin);
    ::Render->m_MSAASample=savedSample;
}

void PainterWarmMaterialPrograms()
{
    if (!HW.pDevice || g_dedicated_server) return;
    string_path directory;
    FS.update_path(directory,"$game_shaders$",::Render->getShaderPath());
    FS_FileSet programs;
    FS.file_list(programs,directory,FS_ListFiles|FS_RootOnly,"painter_*.vs,painter_*.ps,painter_*.cs");
    const painter_material_warm::Settings active = {
        PainterMaskOnlyMode(), PainterBurnOn(),
        RImplementation.o.dx10_msaa_alphatest == CRender::MSAA_ATEST_DX10_0_ATOC,
        RImplementation.o.advancedpp && ps_r2_ls_flags.test(R2FLAG_DETAIL_BUMP),
        RImplementation.o.dx10_msaa ? int(RImplementation.o.dx10_msaa_samples) : 1
    };
    const int savedSkin = Engine.External.GetSkinningMode();
    const int savedSample = ::Render->m_MSAASample;
    unsigned selected=0, failed=0;
    ::Render->m_MSAASample=-1;
    for (const auto& source : programs)
    {
        painter_material_warm::visitLiveSettings(source.name.c_str(),active,[&](const char* stage,const std::string& name,int skin,int sample) {
            bool ready=false;
            ++selected;
            if (stage[0]=='v') {
                PainterBloodPrograms::warmVertex(name,skin);
                ready=PainterBloodPrograms::cache().vertices.count({name,skin})!=0;
            }
            else {
                Engine.External.SetSkinningMode(skin);
                if (stage[0]=='p') {
                    PainterBloodPrograms::warmPixel(name,sample);
                    ready=PainterBloodPrograms::cache().pixels.count({name,sample})!=0;
                } else {
                    ready=PainterWarmBurnBakeProgram();
                }
            }
            if (!ready) {
                ++failed;
                Msg("! [painter_shader_load] stage_failed %s.%s skin=%d sample=%d",name.c_str(),stage,skin,sample);
            }
        });
    }
    Engine.External.SetSkinningMode(savedSkin);
    ::Render->m_MSAASample=savedSample;
    // Include actual material passes from mounted models using custom Lua recipes. Models can also load off-thread;
    // only the name queue is shared. Native program creation and all D3D access stay on this thread.
    runtime_shader_resolve_burn(true);
    PainterBloodPrograms::pump(true);
    Msg("[painter_shader_load] material_files=%u chosen=%u failed=%u mode=%d burn=%u samples=%d",
        unsigned(programs.size()),selected,failed,active.maskMode,unsigned(active.burn),active.samples);
}


void RuntimeShaderCache::RequestClonePrograms(const ref_shader& shader) { runtime_shader_request_clone_programs(shader); }
shared_str RuntimeShaderCache::ExpectedShader(const shared_str& shader_def)
{
    if (!shader_def.size()) return shader_def;
    if (const char* cut = strstr(*shader_def, "$no_shadows"))
        return xr_string(*shader_def, size_t(cut - *shader_def)).c_str();
    return shader_def;
}
const RuntimeShaderEntry* RuntimeShaderCache::Find(const shared_str& expected_shader, const shared_str& texture_def, bool hud, s32 skinning)
{
    auto& c = runtime_shader_cache();
    if (!expected_shader.size() || !texture_def.size()) { ++c.stats.misses; return nullptr; }
    RuntimeShaderKey key;
    key.shader = expected_shader; key.texture = texture_def; key.hud = hud; key.skinning = skinning;
    auto found = c.entries.find(key);
    if (found == c.entries.end() || !found->second.shader._get()) { ++c.stats.misses; return nullptr; }
    ++c.stats.hits;
    return &found->second;
}
void RuntimeShaderCache::Insert(const shared_str& dbg_shader, const shared_str& dbg_texture, bool hud, s32 skinning, const ref_shader& shader)
{
    if (!shader._get() || !dbg_shader.size() || !dbg_texture.size()) return;
    if(dbg_shader.equal("models\\painter_anatomy"))return; // Field-owned handles release with their textures.
    auto& c = runtime_shader_cache();
    RuntimeShaderKey key;
    key.shader = dbg_shader; key.texture = dbg_texture; key.hud = hud; key.skinning = skinning;
    if (c.entries.find(key) != c.entries.end()) return;
    RuntimeShaderEntry entry;
    entry.dbg_shader = dbg_shader; entry.dbg_texture = dbg_texture; entry.shader = shader;
    c.entries.emplace(key, entry);
    ++c.stats.seeded;
    c.stats.entries = unsigned(c.entries.size());
    runtime_shader_request_clone_programs(shader);
    if(runtime_shader_loading_screen())PainterBloodWarmPrograms(shader);
}
void RuntimeShaderCache::RequestWarmPrograms(const xr_string& vs, const xr_string& ps, int sample) { runtime_shader_request_program(vs, ps, sample); }
void RuntimeShaderCache::RequestWarm(const shared_str& shader_def, const shared_str& texture_def, bool hud, s32 skinning)
{
    if (!shader_def.size() || !texture_def.size()) return;
    auto& c = runtime_shader_cache();
    RuntimeShaderKey key;
    key.shader = ExpectedShader(shader_def); key.texture = texture_def; key.hud = hud; key.skinning = skinning;
    if (c.entries.find(key) != c.entries.end() || runtime_shader_queued(c, key)) return;
    c.pending.push_back(key);
    ++c.stats.requested;
    c.stats.pending = unsigned(c.pending.size());
}
void RuntimeShaderCache::WarmLevelInteriors()
{
    static const char* const pixels[] = {"painter_blood_painter_vmi_wall", "painter_blood_painter_vmi_piece", "painter_blood_painter_vmi_tear"};
    const int mode = PainterMaskOnlyMode();
    if (mode <= 0) return;
    auto& cache = runtime_shader_cache();
    const int skin = Engine.External.GetSkinningMode(), sample = ::Render->m_MSAASample;
    Engine.External.SetSkinningMode(4);
    ::Render->m_MSAASample = -1;
    LARGE_INTEGER start;
    QueryPerformanceCounter(&start);
    unsigned ready = 0;
    for (int i = 0; i < mode && i < 3; ++i)
    {
        string_path shader;
        strconcat(sizeof(shader), shader, "r3\\", pixels[i], ".ps");
        if (!FS.exist("$game_shaders$", shader)) continue;
        PainterSoftShaderGuard soft;
        ref_ps program = DEV->_CreatePS(pixels[i]);
        if (program && program->ps) { cache.pinned_ps.push_back(program); ++ready; }
    }
    Engine.External.SetSkinningMode(skin);
    ::Render->m_MSAASample = sample;
    Msg("[painter_wound_load] mode=%d ready=%u ms=%.2f", mode, ready, RuntimeStageMs(start));
}
unsigned RuntimeShaderCache::Warm(unsigned budget)
{
    const bool loading = budget && runtime_shader_loading_screen();
    if (loading) runtime_shader_resolve_burn(); // body burning: the burn twins of what was asked for, made by the pump below
    if(budget)PainterBloodPumpPrograms(loading); // shaderslim: the blood aliases asked for - all behind the loading screen, one pair per frame in play
    if(PainterTissueCache::Warm(budget,loading)&&!loading)return unsigned(runtime_shader_cache().pending.size());
    auto& c = runtime_shader_cache();
    if (loading) runtime_shader_request_painter_programs(c); // the level's 60 precache frames (loading screen): not in the menu
    if (!budget || (c.pending.empty() && c.pending_programs.empty())) return unsigned(c.pending.size());
    if (!loading && c.warm_frame == Device.dwFrame) return unsigned(c.pending.size()); // one drain per frame (all of it behind the loading screen)
    c.warm_frame = Device.dwFrame;
    LARGE_INTEGER start;
    QueryPerformanceCounter(&start);
    const unsigned warmedBefore = c.stats.warmed;
    for (unsigned done = 0; (loading || done < budget) && !c.pending.empty(); ++done)
    {
        const RuntimeShaderKey key = c.pending.front();
        c.pending.erase(c.pending.begin());
        if (c.entries.find(key) != c.entries.end()) continue;
        // The call sequence of a runtime child's ResetShaderTexture: skinning
        // mode and hud_loading from the child for the Create call, the skinning
        // mode restored afterwards (BuildRuntime / AdoptRuntime).
        dxRender_Visual scratch;
        scratch.hud = key.hud; scratch.skinning = key.skinning;
        const int previous_skinning = Engine.External.GetSkinningMode();
        scratch.SetShaderTexture(*key.shader, *key.texture);
        Engine.External.SetSkinningMode(previous_skinning);
        if (scratch.shader._get())
        {
            Insert(scratch.dbg_shader, scratch.dbg_texture, key.hud, key.skinning, scratch.shader);
            ++c.stats.warmed;
        }
        else Msg("! [runtime_shader_cache] phase=warm_failed shader=%s texture=%s hud=%u skinning=%d", *key.shader, *key.texture, unsigned(key.hud), key.skinning);
    }
    // One clone program pair per frame once the shader pairs are done (the same creation calls as the clone, under the
    // same skinning mode, MSAA sample and soft-shader guard; a missing program file is skipped, the clone reports it).
    while (c.pending.empty() && !c.pending_programs.empty())
    {
        const auto job = c.pending_programs.front(); c.pending_programs.erase(c.pending_programs.begin());
        string_path path; bool have = true;
        strconcat(sizeof(path), path, "r3\\", job.vs.c_str(), ".vs"); have = have && FS.exist("$game_shaders$", path);
        strconcat(sizeof(path), path, "r3\\", job.ps.c_str(), ".ps"); have = have && FS.exist("$game_shaders$", path);
        if (have)
        {
            const int previous_skinning = Engine.External.GetSkinningMode(); const int previous_sample = ::Render->m_MSAASample;
            Engine.External.SetSkinningMode(job.skinning); ::Render->m_MSAASample = job.sample;
            {
                PainterSoftShaderGuard soft;
                ref_vs vs = DEV->_CreateVS(job.vs.c_str()); ref_ps ps = DEV->_CreatePS(job.ps.c_str());
                if (vs._get() && ps._get()) { c.pinned_vs.push_back(vs); c.pinned_ps.push_back(ps); ++c.stats.warmed; }
                else Msg("! [runtime_shader_cache] phase=program_warm_failed vs=%s ps=%s", job.vs.c_str(), job.ps.c_str());
            }
            Engine.External.SetSkinningMode(previous_skinning); ::Render->m_MSAASample = previous_sample;
        }
        if (!loading) break; // one program per gameplay frame
    }
    if (loading && c.stats.warmed != warmedBefore)
        Msg("[runtime_shader_cache] phase=warm_loading warmed=%u ms=%.1f", c.stats.warmed - warmedBefore, RuntimeStageMs(start));
    c.stats.warm_ms += RuntimeStageMs(start);
    c.stats.pending = unsigned(c.pending.size());
    if (c.pending.empty() && c.pending_programs.empty())
        Msg("[runtime_shader_cache] phase=warm_done entries=%u warmed=%u requested=%u warm_ms=%.3f", c.stats.entries, c.stats.warmed, c.stats.requested, c.stats.warm_ms);
    return unsigned(c.pending.size()) + unsigned(c.pending_programs.size());
}
unsigned RuntimeShaderCache::Pending() { return unsigned(runtime_shader_cache().pending.size()); }
void RuntimeShaderCache::Clear(bool rewarm)
{
    PainterPassLayouts::cache().clear();
    if(!rewarm)PainterBloodClearPrograms(); // same-device reset keeps warmed shader handles
    RuntimeRequiredFieldCache::Clear(rewarm); // R25: a same-device reset keeps the field textures
    PainterTissueCache::Clear(rewarm);
    auto& c = runtime_shader_cache();
    if (c.entries.empty() && c.pending.empty() && c.pending_programs.empty() && c.pinned_vs.empty() && c.pinned_ps.empty() && !c.painter_programs_requested) return;
    Msg("[runtime_shader_cache] phase=clear entries=%u pending=%u hits=%u misses=%u rewarm=%u", unsigned(c.entries.size()), unsigned(c.pending.size()), c.stats.hits, c.stats.misses, unsigned(rewarm));
    if (rewarm)
        for (const auto& it : c.entries) if (!runtime_shader_queued(c, it.first)) c.pending.push_back(it.first);
    else c.pending.clear();
    c.entries.clear();
    c.pending_programs.clear(); c.known_programs.clear(); c.pinned_vs.clear(); c.pinned_ps.clear();
    c.painter_programs_requested = false;
    c.stats.entries = 0;
    c.stats.pending = unsigned(c.pending.size());
    c.warm_frame = u32(-1);
}
RuntimeShaderCacheStats RuntimeShaderCache::GetStats()
{
    auto& c = runtime_shader_cache();
    c.stats.entries = unsigned(c.entries.size());
    c.stats.pending = unsigned(c.pending.size());
    return c.stats;
}
void RuntimeResolveChildShader(dxRender_Visual& child, RuntimeShaderReuse* reuse)
{
    // A fresh child runs ResetShaderTexture -> SetShaderTexture(*dbg_shader_def,
    // *dbg_texture_def) (FBasicVisual.cpp): dbg_shader = def cut at
    // "$no_shadows", dbg_texture = def, eNoShadow from the def, skinning mode
    // from the child, hud_loading = hud, shader = Create(...), which returns
    // the registered equal Shader for equal inputs (ResourceManager.cpp). A
    // cache entry with those inputs holds that handle and those names.
    if ((!reuse || reuse->enabled) && child.dbg_shader_def.size() && child.dbg_texture_def.size())
    {
        const shared_str expected = RuntimeShaderCache::ExpectedShader(child.dbg_shader_def);
        if (const RuntimeShaderEntry* hit = RuntimeShaderCache::Find(expected, child.dbg_texture_def, child.hud, child.skinning))
        {
            child.flags.set(IRenderVisualFlags::eNoShadow, strstr(*child.dbg_shader_def, "$no_shadows") ? TRUE : FALSE);
            child.dbg_shader = hit->dbg_shader;
            child.dbg_texture = hit->dbg_texture;
            child.shader = hit->shader;
            if (reuse) ++reuse->reused;
            return;
        }
    }
    const int previous_skinning = Engine.External.GetSkinningMode();
    child.ResetShaderTexture(); // Compiles/caches the matching four-influence variant.
    Engine.External.SetSkinningMode(previous_skinning);
    if (reuse) ++reuse->compiled;
    if (child.shader._get() && (!reuse || reuse->enabled))
        RuntimeShaderCache::Insert(child.dbg_shader, child.dbg_texture, child.hud, child.skinning, child.shader);
}
// ---------------------------------------------------------------------------
// Off-thread publication, worker half (RuntimeGeometryStage.h). Produces for
// one packed child exactly what BuildRuntime + _Load_hw(RM_SKINNING_4B) +
// BuildRuntimeFaceLists produce, in one pass over the worker rows:
//  - vertBoned4W rows written once into their final Vertices4W storage;
//  - vertHW_4W rows through the same vertHW_4W::set call as _Load_hw, at the
//    child's vbase of the packet's row array;
//  - the child's indices at its ibase of the packet's index array (and the
//    per-child CPU copy m_Indices needs);
//  - BonesUsed / RMS_bonecount / box / face rows.
// RuntimeStagePackBuffers then creates ONE IB and ONE VB for the packet with
// the same D3D11 description as dx10BufferUtils::CreateBuffer (DEFAULT usage,
// initial data), on the device captured at the barrier: ID3D11Device is
// free-threaded here (created without D3D11_CREATE_DEVICE_SINGLETHREADED,
// dx10HW.cpp:556-566). Two CreateBuffer calls per publication instead of two
// per child: the cost is per call, not per byte (measured 1.55 ms -> 0.34 ms
// for 12 children, engine-threading-publication.md 2.5).
// Nothing here touches the owner model, HW, the resource manager,
// stats_manager, shared_str, wallmarks or the immediate context.
namespace {
HRESULT runtime_stage_buffer(ID3D11Device* device, ID3D11Buffer** out, const void* data, UINT bytes, bool index)
{
    D3D11_BUFFER_DESC desc{};
    desc.ByteWidth = bytes;
    desc.Usage = D3D11_USAGE_DEFAULT;
    desc.BindFlags = index ? D3D11_BIND_INDEX_BUFFER : D3D11_BIND_VERTEX_BUFFER;
    desc.CPUAccessFlags = 0;
    desc.MiscFlags = 0;
    D3D11_SUBRESOURCE_DATA sub{};
    sub.pSysMem = data;
    return device->CreateBuffer(&desc, &sub, out);
}
void* runtime_stage_alloc(size_t bytes) noexcept
{
    return Memory.mem_alloc(bytes
#ifdef DEBUG_MEMORY_NAME
        , "runtime mesh: stage scratch"
#endif
        , false);
}
}
float RuntimeStageMs(const LARGE_INTEGER& from) noexcept
{
    LARGE_INTEGER now, frequency;
    QueryPerformanceCounter(&now);
    QueryPerformanceFrequency(&frequency);
    return float(double(now.QuadPart - from.QuadPart) * 1000.0 / double(frequency.QuadPart));
}
bool RuntimeStageScratch::reserve_packet(size_t vertices, size_t index_count) noexcept
{
    if (vertices > rows || !hw)
    {
        if (hw) Memory.mem_free(hw);
        hw = nullptr; rows = 0;
        hw = runtime_stage_alloc((vertices ? vertices : 1) * sizeof(vertHW_4W));
        if (!hw) return false;
        rows = vertices;
    }
    if (index_count > index_rows || !indices)
    {
        if (indices) Memory.mem_free(indices);
        indices = nullptr; index_rows = 0;
        indices = static_cast<u16*>(runtime_stage_alloc((index_count ? index_count : 1) * sizeof(u16)));
        if (!indices) return false;
        index_rows = index_count;
    }
    return true;
}
bool RuntimeStageScratch::reserve_masks(size_t count) noexcept
{
    if (count <= mask_rows && masks) return true;
    if (masks) Memory.mem_free(masks);
    masks = nullptr; mask_rows = 0;
    masks = static_cast<u64*>(runtime_stage_alloc((count ? count : 1) * sizeof(u64)));
    if (!masks) return false;
    mask_rows = count;
    return true;
}
void RuntimeStageScratch::release() noexcept
{
    if (hw) Memory.mem_free(hw);
    if (indices) Memory.mem_free(indices);
    if (masks) Memory.mem_free(masks);
    hw = nullptr; indices = nullptr; masks = nullptr; rows = 0; index_rows = 0; mask_rows = 0;
}
#include "RuntimePreparedCpu_impl.h"
bool RuntimeStageBuildChild(RuntimeStagedChild& child, unsigned bone_count, unsigned registers,
    xr_vector<xr_vector<CBoneData::FacesVec>>& faces, const std::atomic<bool>& canceled, RuntimeStageTiming& timing,
    RuntimeStageScratch& scratch, char* message, size_t message_size) noexcept
{
    const gfy_packed::Child& src = *child.source;
    const size_t count = src.vertices.size(), index_count = src.indices.size();
    runtime_lean::ValidationError error;
    auto child_reject = [&](const char* why)
    {
        xr_sprintf(message, message_size, "[runtime_geometry_reject] stage=child reason=%s child=%u", why, child.child_index);
        return false;
    };
    LARGE_INTEGER start;
    QueryPerformanceCounter(&start);
    // Allocate outputs only for admissible sizes; build_rows repeats both checks
    // first and reports them with ValidateRuntimeSkinChild's reasons.
    vertBoned4W* rows = nullptr;
    if (!runtime_lean::validate_counts(count, index_count))
    {
        if (!scratch.hw || !scratch.indices || size_t(child.vbase) + count > scratch.rows ||
            size_t(child.ibase) + index_count > scratch.index_rows) return child_reject("allocation");
        rows = child.vertices.create_unique_uninitialized(u32(count));
        if (!rows || !scratch.reserve_masks(count)) return child_reject("allocation");
    }
    xr_vector<u16> used;
    xr_vector<u8> seen_wide;
    used.reserve(bone_count);
    unsigned max_bone = 0;
    child.box.invalidate();
    vertHW_4W* hw_rows = static_cast<vertHW_4W*>(scratch.hw) + child.vbase;
    u64* masks = scratch.masks;
    const bool built = runtime_lean::build_rows(src.vertices.data(), count, src.indices.data(), index_count,
        bone_count, registers, masks, used, seen_wide, max_bone, error,
        [&](size_t i, const gfy_packed::Vertex& v)
        {
            // Same values and order as AppendPackedRuntimeChildren -> BuildRuntime.
            vertBoned4W& d = rows[i];
            for (u32 k = 0; k < 4; ++k) d.m[k] = v.bone[k];
            for (u32 k = 0; k < 3; ++k) d.w[k] = v.weight[k];
            d.P.set(v.surface[0], v.surface[1], v.surface[2]);
            d.N.set(v.surface[3], v.surface[4], v.surface[5]);
            d.T.set(v.surface[6], v.surface[7], v.surface[8]);
            d.B.set(v.surface[9], v.surface[10], v.surface[11]);
            d.u = v.surface[12]; d.v = v.surface[13];
            child.box.modify(d.P);
            // Same call as CSkeletonX_ext::_Load_hw, RM_SKINNING_4B.
            Fvector2 uv;
            uv.set(d.u, d.v);
            hw_rows[i].set(d.P, d.N, d.T, d.B, uv, int(d.m[0]) * 3, int(d.m[1]) * 3, int(d.m[2]) * 3,
                int(d.m[3]) * 3, d.w[0], d.w[1], d.w[2]);
        },
        [&]() { return canceled.load(std::memory_order_relaxed); });
    timing.build_ms += RuntimeStageMs(start);
    if (!built)
    {
        if (!xr_strcmp(error.reason, "canceled")) return child_reject("canceled");
        xr_sprintf(message, message_size,
            "[runtime_geometry_reject] stage=child reason=%s child=%u vertex=%llu influence=%u registers=%u",
            error.reason, child.child_index, u64(error.vertex), error.influence, registers);
        return false;
    }
    child.vcount = u32(count);
    child.icount = u32(index_count);
    child.rms_bonecount = max_bone + 1;
    child.bones_used.create_unique(u32(used.size()), used.data());
    u16* indices = child.indices.create_unique_uninitialized(u32(index_count));
    if (!indices) return child_reject("allocation");
    CopyMemory(indices, src.indices.data(), index_count * sizeof(u16));
    // The packet IB holds the same child-relative indices; DrawIndexed adds the
    // child's base vertex after the 16-bit fetch (packed VB may exceed 65535 rows).
    CopyMemory(scratch.indices + child.ibase, src.indices.data(), index_count * sizeof(u16));
    QueryPerformanceCounter(&start);
    if (bone_count <= 64)
    {
        u32 counts[64];
        runtime_lean::face_rows_from_masks(masks, indices, index_count, bone_count, counts,
            [&](unsigned b) -> CBoneData::FacesVec& { return faces[b][child.child_index]; });
    }
    else
    {
        for (u16 b = 0; b < bone_count; ++b) faces[b][child.child_index].clear();
        BuildRuntimeFaceLists(src.vertices, src.indices, bone_count,
            [&](u16 bone, u32 face) { faces[bone][child.child_index].push_back(u16(face)); });
    }
    timing.faces_ms += RuntimeStageMs(start);
    child.built = true;
    return true;
}
bool RuntimeStagePackBuffers(ID3D11Device* device, const RuntimeStageScratch& scratch, size_t vertices, size_t index_count,
    ID3D11Buffer** vb, ID3D11Buffer** ib, char* message, size_t message_size) noexcept
{
    *vb = nullptr; *ib = nullptr;
    if (!vertices || !index_count) return true; // nothing built (every child removed a surface)
    if (!device || !scratch.hw || !scratch.indices || vertices > scratch.rows || index_count > scratch.index_rows ||
        vertices * sizeof(vertHW_4W) > UINT(-1) || index_count * sizeof(u16) > UINT(-1))
    {
        xr_sprintf(message, message_size, "[runtime_geometry_reject] stage=packet reason=allocation");
        return false;
    }
    if (FAILED(runtime_stage_buffer(device, ib, scratch.indices, UINT(index_count * sizeof(u16)), true)))
    {
        *ib = nullptr;
        xr_sprintf(message, message_size, "[runtime_geometry_reject] stage=packet reason=index_buffer");
        return false;
    }
    // _Load_hw uses R_CHK here (Debug.error on failure); staging reports it instead.
    if (FAILED(runtime_stage_buffer(device, vb, scratch.hw, UINT(vertices * sizeof(vertHW_4W)), false)))
    {
        *vb = nullptr;
        _RELEASE(*ib);
        xr_sprintf(message, message_size, "[runtime_geometry_reject] stage=packet reason=vertex_buffer");
        return false;
    }
    return true;
}
// ---------------------------------------------------------------------------
// Publish-barrier half: wrap one staged child. Field for field the state
// BuildRuntime leaves, with buffers/backup moved instead of rebuilt.
void RuntimeShaderReuse::seed(const RuntimeGeometryPacket* active, const xr_vector<IRenderVisual*>* originals)
{
    if (!enabled) return;
    // OGF originals: live handles for every exterior pair at the skinning mode
    // they were loaded with (a runtime child at skinning 4 hits only when equal).
    if (originals)
        for (auto* visual : *originals)
            if (auto* v = fast_dynamic_cast<dxRender_Visual*>(visual))
                RuntimeShaderCache::Insert(v->dbg_shader, v->dbg_texture, v->hud, v->skinning, v->shader);
    if (active)
        for (auto* visual : active->owned)
            if (auto* skin = fast_dynamic_cast<const CSkeletonX_ST*>(visual); skin && !skin->painter_required_fields)
                RuntimeShaderCache::Insert(skin->dbg_shader, skin->dbg_texture, skin->hud, skin->skinning, skin->shader);
}
bool CSkeletonX_ST::AdoptRuntime(RuntimeStagedChild& staged, CKinematics* owner, const dxRender_Visual& material,
    RuntimeShaderReuse& reuse)
{
    if (!owner || !staged.built || !staged.vb || !staged.ib || !staged.source)
    {
        Msg("[runtime_geometry_reject] stage=child reason=stage_child_state child=%u", staged.child_index);
        return false;
    }
    Type = MT_SKELETON_GEOMDEF_ST; Parent = owner; ChildIDX = staged.child_index;
    receives_wallmarks = staged.receives_wallmarks;
    RenderMode = RM_SKINNING_4B; RMS_bonecount = staged.rms_bonecount;
    // Packet buffers: this child's rows start at vbase/ibase (draw offsets only;
    // m_Indices / Vertices4W are the child's own copies, base 0).
    vBase = staged.vbase; iBase = staged.ibase; vCount = staged.vcount; iCount = staged.icount; dwPrimitives = iCount / 3;
    BonesUsed.swap(staged.bones_used);
    m_Indices.swap(staged.indices);
    p_rm_Indices = staged.ib; staged.ib = nullptr; // the child's own reference to the packet IB
    Vertices4W.swap(staged.vertices);
    p_rm_Vertices = staged.vb; staged.vb = nullptr; // the child's own reference to the packet VB
    // stats_manager counts the packet buffers once at install (it decrements on
    // the last reference), not per child.
    rm_geom.create(dwDecl_4W, p_rm_Vertices, p_rm_Indices); // one SGeometry per packet (CreateGeom dedupes)
    m_ProjectionSource = std::move(staged.projection); m_ProjectionIndexBase = 0; m_ProjectionIndexCount = iCount;
    dbg_name = "runtime_visual_damage"; dbg_id = staged.child_index + 1; flags = material.flags; hud = material.hud; skinning = 4;
    dbg_shader_def = staged.shader.size() ? shared_str(staged.shader.c_str()) : material.dbg_shader_def;
    dbg_texture_def = staged.texture.size() ? shared_str(staged.texture.c_str()) : material.dbg_texture_def;
    painter_tissue=staged.painter_tissue;
    painter_required_fields=staged.required_fields;
    SetWallmarkOwnership(painter_required_fields?painter_required_fields->wallmark_ownership:nullptr);
    SetWallmarkProvenance(staged.material_source_child,staged.source->original_faces);
    const bool anatomy_bound=PainterTissueCache::Bind(painter_tissue,*this);
    if(!anatomy_bound)RuntimeResolveChildShader(*this,&reuse);else ++reuse.reused;
    xr_string required_why;
    if(!RuntimeBindRequiredFields(painter_required_fields,*this,staged.child_index,staged.skin_clip,staged.skin_owned,
        staged.analytic,staged.receives_wallmarks,anatomy_bound,required_why)){
        xr_string programs;
        if(shader)for(const auto&element:shader->E)if(element)for(const auto&pass:element->passes)if(pass){
            programs+=pass->vs?pass->vs->cName.c_str():"-";programs+="/";programs+=pass->ps?pass->ps->cName.c_str():"-";programs+=";";}
        Msg("! [runtime_geometry_reject] stage=child reason=required_material child=%u why=%s skin_clip=%u analytic=%u shader=%s texture=%s programs=%s",
            staged.child_index,required_why.c_str(),unsigned(staged.skin_clip),unsigned(staged.analytic),
            dbg_shader_def.c_str()?dbg_shader_def.c_str():"",dbg_texture_def.c_str()?dbg_texture_def.c_str():"",programs.c_str());return false;
    }
    if(staged.source&&PainterMaskPass::IsBox(*staged.source)&&!PainterMaskPass::BoxOnly(*DEV,shader))
        Msg("! [painter_mask_only] coverage_box_hidden child=%u (no volume program for its material)",staged.child_index);
    vis.box.set(staged.box);
    vis.box.getsphere(vis.sphere.P, vis.sphere.R);
    if (!shader) { Msg("[runtime_geometry_reject] stage=child reason=shader child=%u", staged.child_index); return false; }
    return true;
}
// ---------------------------------------------------------------------------
// r_painter_publish 2: compare an adopted child with a BuildRuntime reference
// built at the same barrier from the same packed rows. GPU buffers are read
// back through staging copies of the child's byte range (debug only: stalls
// the immediate context). The adopted child draws a sub-range of the packet
// buffers; the reference owns per-child buffers (base 0).
namespace {
bool runtime_read_buffer(ID3D11Buffer* buffer, u32 offset, u32 bytes, xr_vector<u8>& out)
{
    out.clear();
    if (!buffer || !bytes) return false;
    D3D11_BUFFER_DESC desc;
    buffer->GetDesc(&desc);
    if (u64(offset) + bytes > desc.ByteWidth) return false;
    desc.ByteWidth = bytes;
    desc.Usage = D3D11_USAGE_STAGING; desc.BindFlags = 0; desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ; desc.MiscFlags = 0;
    ID3D11Buffer* copy = nullptr;
    if (FAILED(HW.pDevice->CreateBuffer(&desc, nullptr, &copy))) return false;
    D3D11_BOX box;
    box.left = offset; box.right = offset + bytes; box.top = 0; box.bottom = 1; box.front = 0; box.back = 1;
    HW.pContext->CopySubresourceRegion(copy, 0, 0, 0, 0, buffer, 0, &box);
    D3D11_MAPPED_SUBRESOURCE mapped;
    const bool ok = SUCCEEDED(HW.pContext->Map(copy, 0, D3D11_MAP_READ, 0, &mapped));
    if (ok) { out.resize(bytes); CopyMemory(out.data(), mapped.pData, bytes); HW.pContext->Unmap(copy, 0); }
    copy->Release();
    return ok;
}
template <class A, class B> bool runtime_same_rows(A& a, B& b)
{
    return a.size() == b.size() && (!a.size() || !memcmp(*a, *b, a.size() * sizeof(**a)));
}
}
bool CSkeletonX_ST::RuntimeMatches(CSkeletonX_ST& r, char* diff, size_t diff_size)
{
    auto differ = [&](const char* field) { xr_strcpy(diff, diff_size, field); return false; };
    if (Type != r.Type || Parent != r.Parent || ChildIDX != r.ChildIDX) return differ("identity");
    if (receives_wallmarks != r.receives_wallmarks || RenderMode != r.RenderMode || RMS_bonecount != r.RMS_bonecount)
        return differ("render_mode");
    // vBase/iBase differ by construction (packet sub-range vs. per-child reference buffers).
    if (vCount != r.vCount || iCount != r.iCount || dwPrimitives != r.dwPrimitives)
        return differ("counts");
    if (!runtime_same_rows(BonesUsed, r.BonesUsed)) return differ("bones_used");
    if (!runtime_same_rows(m_Indices, r.m_Indices)) return differ("indices");
    if (!runtime_same_rows(Vertices4W, r.Vertices4W)) return differ("vertices4w");
    if (m_ProjectionIndexBase != r.m_ProjectionIndexBase || m_ProjectionIndexCount != r.m_ProjectionIndexCount)
        return differ("projection_range");
    if (!dbg_name.equal(r.dbg_name) || dbg_id != r.dbg_id || flags.get() != r.flags.get() || hud != r.hud || skinning != r.skinning)
        return differ("visual_state");
    if (!dbg_shader_def.equal(r.dbg_shader_def) || !dbg_texture_def.equal(r.dbg_texture_def) ||
        !dbg_shader.equal(r.dbg_shader) || !dbg_texture.equal(r.dbg_texture)) return differ("material_names");
    if (shader._get() != r.shader._get()) return differ("shader_handle");
    if (memcmp(&vis.box, &r.vis.box, sizeof(vis.box)) || memcmp(&vis.sphere, &r.vis.sphere, sizeof(vis.sphere)))
        return differ("bounds");
    xr_vector<u8> a, b;
    const u32 vertex_stride = sizeof(vertHW_4W), index_stride = sizeof(u16);
    if (!runtime_read_buffer(p_rm_Vertices, vBase * vertex_stride, vCount * vertex_stride, a) ||
        !runtime_read_buffer(r.p_rm_Vertices, r.vBase * vertex_stride, r.vCount * vertex_stride, b) || a != b) return differ("gpu_vertices");
    if (!runtime_read_buffer(p_rm_Indices, iBase * index_stride, iCount * index_stride, a) ||
        !runtime_read_buffer(r.p_rm_Indices, r.iBase * index_stride, r.iCount * index_stride, b) || a != b) return differ("gpu_indices");
    const auto* sa = m_ProjectionSource.get();
    const auto* sb = r.m_ProjectionSource.get();
    if (!sa != !sb) return differ("projection_presence");
    if (sa && (sa->vertices.size() != sb->vertices.size() || sa->indices != sb->indices ||
        sa->bones.size() != sb->bones.size() ||
        (sa->vertices.size() && memcmp(sa->vertices.data(), sb->vertices.data(), sa->vertices.size() * sizeof(sa->vertices[0]))) ||
        (sa->bones.size() && memcmp(sa->bones.data(), sb->bones.data(), sa->bones.size() * sizeof(sa->bones[0])))))
        return differ("projection_source");
    return true;
}
#endif
