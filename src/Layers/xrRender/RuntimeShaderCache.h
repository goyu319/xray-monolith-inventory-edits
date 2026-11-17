#pragma once
#ifdef USE_DX11
// Renderer-wide shader handle cache for runtime (wound) children.
//
// A runtime child resolves its Shader through dxRender_Visual::SetShaderTexture
// (FBasicVisual.cpp): the definition cut at "$no_shadows", the texture
// definition, the hud flag and the skinning mode are the only compile inputs,
// and CResourceManager::Create returns the registered equal Shader for equal
// inputs. This cache maps those four inputs to the ref_shader that call would
// return, so a publication whose pairs were seen before (by any NPC, in any
// earlier revision) copies handles instead of compiling shader elements on the
// publish barrier. Entries pin their Shader (ref_shader) across revisions and
// NPCs; they are dropped on renderer destroy / level unload and re-warmed after
// a device reset.
//
// Main thread only: entries hold shared_str (non-atomic counter) and ref_shader
// (resource manager). Callers: the publish barriers (install, sync prepare),
// CKinematics::LL_WarmRuntimeShaders (bind), CRender::OnFrame (warm drain),
// CRender::destroy / reset_begin / level_Unload (clear).
#include "Shader.h"
struct RuntimeShaderEntry
{
    shared_str dbg_shader, dbg_texture; // names exactly as SetShaderTexture leaves them
    ref_shader shader;
};
struct RuntimeShaderCacheStats
{
    unsigned entries = 0, pending = 0, hits = 0, misses = 0, warmed = 0, seeded = 0, requested = 0;
    float warm_ms = 0.f;
};
namespace RuntimeShaderCache
{
// The definition cut at "$no_shadows" (SetShaderTexture's dbg_shader); the definition itself when absent.
shared_str ExpectedShader(const shared_str& shader_def);
// Lookup by (cut shader, texture def, hud, skinning). Counts a hit or a miss.
const RuntimeShaderEntry* Find(const shared_str& expected_shader, const shared_str& texture_def, bool hud, s32 skinning);
// Record a handle a visual already holds (an OGF original or a child that just compiled). No-op when present or when a name is empty.
void Insert(const shared_str& dbg_shader, const shared_str& dbg_texture, bool hud, s32 skinning, const ref_shader& shader);
// Queue a (shader def, texture def, hud, skinning) pair for warm-up; no-op when cached or queued.
void RequestWarm(const shared_str& shader_def, const shared_str& texture_def, bool hud, s32 skinning);
// Queue a vertex/pixel program pair (by name, skinning 4, the given MSAA sample) that a wound material clone would
// create at the first install: created one per frame by Warm() and pinned here, so the clone finds it by name.
void RequestWarmPrograms(const xr_string& vs, const xr_string& ps, int sample);
// R62: the clone programs of every pass of `shader` (the anatomy material's channel pair: painter_anatomy -> painter_channel and
// the shadow pair), queued like the seeded shaders' (PainterTissueCache builds its anatomy shaders without Insert).
void RequestClonePrograms(const ref_shader& shader);
// Compile up to `budget` queued pairs through the same SetShaderTexture path a
// child uses; at most one call per frame does work. Returns the pairs still queued.
// R62: while the loading screen is up (precache frames, the press-a-key wait) every queued pair and program is compiled
// in one call, and the channel programs a first wound needs are queued at the first call after a clear: the 2 s cold
// compile of painter_channel happens behind the loading screen instead of in the frame of the first wound.
unsigned Warm(unsigned budget);
unsigned Pending();
// Drop every handle. rewarm: queue the dropped keys again (device reset).
void Clear(bool rewarm);
RuntimeShaderCacheStats GetStats();
}
#endif
