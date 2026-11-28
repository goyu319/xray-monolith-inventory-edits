#pragma once
#include <atomic>
// Painter fire: baked fire pictures played by the renderer (see PainterFire_impl.h).
namespace PainterFire
{
// A fire bottle broke at this place: one of the baked bursts, then the burning pool and a few small flames around it, for
// `seconds`. False when nothing is drawn (switched off, or the fire files are missing) - the caller keeps its own effect.
bool Bottle(const Fvector& position, float seconds, u32 seed);
// The same, told the way the bottle flew: one that hit the ground on its way throws its fuel on ahead.
bool BottleThrown(const Fvector& position, float seconds, u32 seed, const Fvector& flew);
bool BottleImpact(const Fvector& position,float seconds,u32 seed,const Fvector& flew,const Fvector* witness);
// Trees that burnt (r__dsgraph_build.cpp): their leaf parts are not drawn, their far pictures are not used. bareCount:
// how many leaf parts are gone (0: nothing to ask).
extern std::atomic<u32> bareCount;
bool LeavesGone(const void* visual);
bool LodGone(const void* visual);
float TreeBurnAmount(const void* visual, bool* separateWood = nullptr, float* thermal = nullptr);
bool thermalViewPossible();
bool TreeWoodBurn(const void* visual,float& amount,float& heat);
// Console: one take a few metres in front of the camera. An empty name lists the takes.
void Test(const char* take, float seconds);
// Console: a take baked with a wall in it ("door", "eave") on the wall the camera looks at.
void TestWall(const char* take, float seconds, float scale);
// Console: the person the camera looks at burns (take "body"); the fire goes where they go.
void TestBody(float seconds);
// A body burns for at least `seconds` from now (call again while it does): it carries the body fire, which goes where it
// goes. False when nothing is drawn (switched off, files missing, no such body) - the caller keeps its own effect.
bool Body(u16 id, float seconds);
// Burning fuel lands here and the fire spreads from it over what can burn (PainterFireSpread.h). False = nothing started.
bool Ignite(const Fvector& position, float radius, float seconds);
// Console: the same on the ground the camera looks at; what burns right now; every spreading fire out.
void Spread(float seconds);
void SpreadInfo();
void SpreadStop();
// How far the grass at a place has burnt away, 0 untouched .. 1 gone: the grass renderer asks for every blade it draws.
float GrassBurnt(const Fvector& position);
// moves whenever burnt grass changes: renderers that cache grass blades rebuild their cache when it does
u32 GrassBurntRevision();
// How hot the spreading fire is at a place, 0 no fire .. 1 a big one (the game hurts and sets alight what stands there).
float Heat(const Fvector& position);
// firesoundspots: the fires being lit now (the tracked fire lights: place, size), at most max; returns how many
u32 Spots(Fvector* positions, float* sizes, u32 max);
// A small flame is held to the ground at a place (a match, a lighter): what is there catches if it can burn and is not
// too wet. False = fire is switched off.
bool Light(const Fvector& position);
// The nearest place where the spreading fire burns, within `radius` of a position (whoever is near runs). False = none.
bool Near(const Fvector& position, float radius, Fvector& nearest);
// Hot air over the fires, for the distortion pass: is there any to draw; draw it.
bool HazeWanted();
// How smoky the air is near the camera, 0 clear .. 1 (the fire's own smoky air, firerework).
float SmokeAir();
// firerework: the fire's own smoky air. AirFrame: once a frame before the render (its map in pixel-shader slot 47);
// AirWanted / RenderAir: is there any / set its shader for the screen's quad; AirConstants: what the shaders read of it;
// AirLight: a light at this place stands in the smoke (it shows its shafts).
void AirFrame();
bool AirWanted();
bool RenderAir();
void AirConstants(Fvector4& map, Fvector4& k, Fvector4& colour);
bool AirLight(const Fvector& position);
void RenderHaze();
// For the stall recorder: the renderer made a shader program (r4.cpp, CRender::shader_compile). ticks = how long the
// making took in all, compileTicks = how much of it was a compile from source (0: a cached build was used). Any thread.
void ShaderMade(u64 ticks, u64 compileTicks);
void Update(); // once a frame: ends finished fires, moves their lights
void VisitWarmPrograms(void(*emit)(const char*,const char*,int,int));
void PrewarmShaders();
void Render(); // forward pass, after the sorted geometry
void Clear();  // level unload, device loss
}
extern int ps_painter_fire;
extern int ps_painter_fire_light_quality;
extern int ps_painter_fire_light_beams; // firebeams
extern int ps_painter_fire_light_count; // firetrack
extern float ps_painter_fire_fog_buildup, ps_painter_fire_fog_fire, ps_painter_fire_fog_linger, ps_painter_fire_fog_thickness, ps_painter_fire_light_reach; // firefog
extern float ps_painter_fire_fog_area; // smokeair
extern float ps_painter_fire_brightness;
extern float ps_painter_fire_spread_speed, ps_painter_fire_spread_seconds, ps_painter_fire_spread_radius;
extern int ps_painter_fire_spread_on;
extern float ps_painter_fire_burn_time, ps_painter_fire_pool_time, ps_painter_fire_burnt_minutes;
extern int ps_painter_fire_quality, ps_painter_fire_embers, ps_painter_fire_haze, ps_painter_fire_burnt, ps_painter_fire_fog, ps_painter_fire_smoke, ps_painter_stall_log;
extern int ps_painter_fire_fence_foot; // fencefoot + wallfire: 1 = a burning wall's fire stands on the ground at its foot; no fire front on a fence top or a rail; nothing hung on a wall
