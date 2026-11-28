// Renderer for the baked 3D fire fields. Included by r4_R_render.cpp. The volume textures and take list provide the animated heat and smoke fields.
#include "PainterFire.h"
#include "PainterFixedBloodPrograms.h"
#include "PainterSurfaceHeat.h"
#include "../xrRenderDX10/StateManager/dx10StateCache.h"
#include "../xrRenderDX10/StateManager/dx10StateManager.h"
#include "../../xrCore/PainterBurnTrace.h"
#include "PainterBurntCrownIndex.h"
#include "PainterTreeBurnDates.h"
#include <memory>
#include "PainterFireTriangleContact.h"
#include "PainterFireColumn.h"
#include "PainterWoodFaces.h"
#include "PainterFireFront.h"
#include "PainterWrapBounds.h"
#include "PainterWoodFlame.h"
#include "PainterWrapSelection.h"
#include "PainterFireLocalLights.h"
#include "light.h"
#include "FHierrarhyVisual.h"
#include "../../xrEngine/Fmesh.h"
#include "../../xrEngine/Environment.h"
#include "../../xrEngine/IGame_Level.h"
#include "../../xrEngine/IGame_Persistent.h"
#include "../../xrEngine/xr_object.h"
#include "../../xrEngine/GameMtlLib.h"
#include "../../xrEngine/Stats.h"
#include "../../Include/xrRender/Kinematics.h"
#include "PainterFireSpread.h"
#include <mutex>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <thread>
#include <atomic>
#include <unordered_set>
#include "../xrRenderDX10/StateManager/dx10ShaderResourceStateCache.h"
#include "../xrRenderDX10/dx10BufferUtils.h"

int ps_painter_fire = 1;
int ps_painter_fire_light_quality = 0;
int ps_painter_fire_light_count = 12; // firetrack: how many fires get a lamp (4..32; the fire menu)
float ps_painter_fire_fog_buildup = 120.f, ps_painter_fire_fog_fire = 15.f, ps_painter_fire_fog_linger = 250.f, ps_painter_fire_fog_thickness = 1.f, ps_painter_fire_light_reach = 1.f; // firefog: the fire menu's settings
float ps_painter_fire_fog_area = 300.f; // smokeair: fires this far away (m) still make the air smoky
int ps_painter_fire_light_beams = 1; // firebeams: shafts of firelight in fog / smoke (shadowed fire lamps only)
float ps_painter_fire_brightness = 1.f;
// ---------------------------------------------------------------------------------------------------------------------
// The spreading fire (rules: PainterFireSpread.h). Here: what the level is like at a point, a picture for every burning
// cell, and the fire's lights.
float ps_painter_fire_spread_speed = 1.f;
float ps_painter_fire_spread_seconds = 180.f;
float ps_painter_fire_spread_radius = 80.f;
int ps_painter_fire_spread_on = 1;
float ps_painter_fire_burn_time = 1.f; // how long a burning patch burns, times its fuel's own time (the Fire menu)
float ps_painter_fire_pool_time = 1.f; // how long a thrown bottle's pool burns, times its own time (the Fire menu)
float ps_painter_fire_burnt_minutes = 0.f; // how long what burnt stays burnt (minutes of play); 0 = until the level is left
int ps_painter_fire_quality = 2; // 0 low, 1 medium, 2 high: steps through the flames, sparks, haze
int ps_painter_fire_embers = 1;
int ps_painter_fire_haze = 1;
int ps_painter_fire_burnt = 1;   // burnt ground and burnt-away grass
int ps_painter_fire_fence_foot = 1; // fencefoot: the large ground flame at the foot of a burning wooden wall or fence
int ps_painter_fire_fog = 1;     // the air fills with smoke near a big fire (the game's fog)
int ps_painter_fire_smoke = 1;   // the smoke cloud over a big fire
int ps_painter_stall_log = 0;    // 1: a frame of 100 ms or more is written to the log with where its time went

extern Fvector4 ps_s3ds_param_3;
extern int ps_markswitch_current;
namespace PainterFire
{
bool thermalViewPossible()
{
    const int optic=int(ps_s3ds_param_3.x);
    return ps_r2_heatvision>0 || (scope_3D_fake_enabled && ps_markswitch_current<2 && (optic==2 || optic==3));
}
void PrepareHeatTarget(const ref_rt& native,bool needed);
void clearState(bool leavingLevel);
std::atomic<u32> bareCount{0};
namespace
{
struct VolTex
{
	ID3DTexture3D* tex = nullptr;
	ID3DShaderResourceView* srv = nullptr;
};
struct VolFile // a volume file as the reader took it from the disk, waiting to be made a texture
{
	xr_vector<u8> data;
	u32 w = 0, h = 0, d = 0, mips = 0, fmt = 83; // fmt: 80 one channel (BC4), 83 two (BC5)
};
struct Take
{
	shared_str name;
	u32 frames = 1;
	float fps = 30.f, width = 1.f, height = 1.f, deep = 1.f; // the baked box (m)
	float footX = .5f, footZ = .5f; // where the fire stands in it (a take baked with a wall: footZ = the wall's face)
	bool loop = false, smoke = false, wall = false;
	Fvector fmin = {0.f, 0.f, 0.f}, fmax = {1.f, 1.f, 1.f}; // the part of the box the flame volume covers
	u32 fvox[3] = {8, 8, 8}, fgrid[2] = {1, 1}; // flame volume: voxels of one frame, frames across and down its sheet
	u32 svox[3] = {4, 4, 4}, sgrid[2] = {1, 1}; // smoke volume (the whole box at half the detail)
	float heat[4] = {600.f, 600.f, 600.f, 1050.f}; // heat 0, heat range, flame shows from, flame is full at (K over ambient)
	float amount[3] = {1.f, 1.f, 1.f}; // what a texel value of 1 stands for, per metre: flame, soot, smoke
	xr_vector<float> light, lightH; // per frame: how much light the flames give off, how high above the ground (m)
	VolTex a, b, c;
	VolFile fa, fb, fc;
	xr_vector<u8> coarsePixels;
	bool coarsePaired = false;
	int state = 0; // 0 its files are not read yet, 1 read: its textures are to be made, 2 ready, -1 no use
};
struct Fire
{
	PainterFireColumn::Section wrapColumn;
    PainterWoodFlame::Face wrapFace;
    bool woodCellPicture=false,woodImpactPicture=false;
    bool footPicture=false; // fencefoot: the ground take at the foot of a burning wooden wall (beside the cell's wrap picture)
    float footBase=1.f;     // wallfire: ... its own size at full flames
    float trunkReach=0.f;   // firetrunk: a trunk's column is drawn at least this tall (m) at full flames; 0 = its take's own
	int take = -1;
	u32 born = 0;       // Device.dwTimeGlobal when it was asked for
	float delay = 0.f;  // seconds before it shows
	float life = 0.f;   // a looping take burns this long (s); a take that plays once ends with its last frame
	float fadeIn = 0.f, fadeOut = 0.f;
	float phase = 0.f;  // a looping take starts this many frames in: neighbouring flames never play in step
	Fvector pos = {0.f, 0.f, 0.f};
	float scale = 1.f;
	float yaw = 0.f;    // the way its box stands on the ground: its width runs along (cos yaw, 0, sin yaw)
	bool crownEmbers = false;
	bool mirror = false;
	float smoke = 0.f, albedo = .3f; // smoke belongs to what burns; a fire bottle's own fire makes none
	float lightK = 0.f; // > 0: this fire carries a light of this strength
	float rate = 1.f;    // firerework: how fast it plays its take (neighbours never move in step)
	float growFor = 0.f; // firerework: seconds it grows from small to full once it shows (a crown catching)
	float dist2 = 0.f;
	int side = -1;                    // the same fire seen from the side (a second take), or -1
	Fvector normal = {0.f, 0.f, 0.f}; // a fire on a wall: the way the wall faces; zero = a picture that turns to the camera
	u16 follow = u16(-1);             // a fire on a body: the object it sits on
	u16 bone = u16(-1);               // ... and the bone of that body it sits on (none: the whole body)
	int bodyFloorFaceId=-2; u32 bodyFloorProbeAt=0; float bodyAirHeight=0.f;
	float boneSize = 1.f;             // ... and how big it is drawn there
	Fvector facing = {0.f, 0.f, 1.f}; // ... and the way that body faces
	u32 id = 0;   // a picture the spreading fire owns has a number of its own ...
	u32 cell = 0; // ... and the cell it shows (its number + 1): its size follows that cell
	// Cell indices remain stable while the field grows. Store indices only and read flame state each frame.
	u32 pictureNeighbours[4] = {};
	u32 pictureFieldCount = 0;
    u64 frontGroup = 0;
    u64 stableLightId=0;
    u32 frontCell = 0, frontTested = 0;
    bool frontGround = false, frontClear = false;
    Fvector frontPoint = {0.f,0.f,0.f};
	float cap = 0.f; // a fire under a ceiling: the height its flames and smoke end at (0 = none)
	float air = 0.f; // a fire that does not stand on the ground: the share of its box's height its soft bottom takes
	float top = 0.f; // a fire in a room: how high the ceiling over it lies - nothing of it is drawn above that (0 = in the open)
	Fvector capN = {0.f, 0.f, 0.f}; // ... the way that ceiling faces (down; 0 0 0 = level): everything of it ends along it
	s8 roof = -1; // is something overhead (rain and wind do not reach it)? -1 = not looked yet
};
xr_vector<Take> takes;
xr_vector<Fire> fires;
float airAt(const Fvector& p); // firerework: the smoky air at a place (below)
ref_geom geom;
ref_shader shVol, shVolDry, shSmoke, shVolPair, shVolDryPair;
ref_shader shColumnWrap;
// Each active viewport owns one independent heat picture at the native heat target's dimensions and sample count.
ref_rt fireHeatRT;
u64 fireHeatSourceOrder=0, fireHeatOwnOrder=0;
bool fireHeatBound=false;
bool fireHeatDirty=false;
ID3DRenderTargetView* oldFireRT=nullptr;
ID3DBlendState* oldFireBlend=nullptr;
enum HeatProgram {HeatVolume, HeatDry, HeatPair, HeatDryPair, HeatWrap, HeatEmber, HeatGround, HeatWood, HeatChar, HeatCeiling, HeatCount};
struct HeatPass {ref_shader shader; ID3DBlendState* native=nullptr; ID3DBlendState* maximum=nullptr;};
HeatPass heatPasses[HeatCount];
const char* heatNames[HeatCount]={"painter\\fire_vol_ir","painter\\fire_vol_dry_ir","painter\\fire_vol_pair_ir","painter\\fire_vol_dry_pair_ir",
    "painter\\fire_wrap_ir","painter\\fire_ember_ir","painter\\fire_groundgeo_ir","painter\\wood_contact_ir","painter\\fire_char_ir","painter\\fire_ceilheat_ir"}; // firerework: the ceiling's heat

void beginFireHeat()
{
    // GOYU: this renderer has no separate list of camera-attached scope parts; the scope lens list covers it
    const bool scope=thermalViewPossible() && !RImplementation.GMBase.RGraph.mapScopeHUDSorted.empty();
    if(scope && ps_r2_heatvision<=0)PrepareHeatTarget(RImplementation.Target->rt_Heat,true);
    fireHeatBound=(ps_r2_heatvision>0 || scope) && fireHeatRT && fireHeatRT->pRT;
    if(fireHeatBound) oldFireRT=RCache.get_RT(3);
}
void suspendFireHeat()
{
    if(!fireHeatBound)return;
    RCache.set_RT(oldFireRT,3);
    if(oldFireBlend){StateManager.SetBlendState(oldFireBlend);oldFireBlend=nullptr;}
}
void setHeatProgram(ref_shader& ordinary, HeatProgram program)
{
    if(!fireHeatBound){RCache.set_Shader(ordinary);return;}
    auto& pass=heatPasses[program];
    if(!pass.shader)pass.shader.create(heatNames[program]);
    if(!pass.shader){suspendFireHeat();RCache.set_Shader(ordinary);return;}
    RCache.set_Shader(pass.shader);
    if(!pass.maximum){
        D3D_BLEND_DESC d;
        dx10StateUtils::ResetDescription(d);
        pass.shader->E[0]->passes[0]->state->state_code.UpdateDesc(d);
        pass.native=BSManager.GetState(d);
        d.IndependentBlendEnable=TRUE;
        d.RenderTarget[1].RenderTargetWriteMask=0;
        d.RenderTarget[2].RenderTargetWriteMask=0;
        auto& ir=d.RenderTarget[3];
        ir.BlendEnable=TRUE;
        ir.SrcBlend=ir.DestBlend=ir.SrcBlendAlpha=ir.DestBlendAlpha=D3D11_BLEND_ONE;
        ir.BlendOp=ir.BlendOpAlpha=D3D11_BLEND_OP_MAX;
        ir.RenderTargetWriteMask=D3D11_COLOR_WRITE_ENABLE_RED;
        pass.maximum=BSManager.GetState(d);
    }
    oldFireBlend=pass.native;
    StateManager.SetBlendState(pass.maximum);
    RCache.set_RT(fireHeatRT->pRT,3);
    fireHeatDirty=true;
}

bool pairedPrograms = false;
bool loaded = false, usable = false;
// Heat lookup. The first row carries black-body brightness and the second supplies the flame colours.
ID3DTexture2D* lutTex = nullptr;
ID3DShaderResourceView* lutSrv = nullptr;
float lutHeat = 2560.f, lutL0 = -13.2877f, lutL1 = 8.6439f;
// a tiling 3D pattern (swirl + cloudy) the volume shader wrinkles the flames with
ID3DTexture3D* noiseTex = nullptr;
ID3DShaderResourceView* noiseSrv = nullptr;
bool volBC = true; // the graphics card takes block-compressed 3D textures (else they are unpacked on reading)
std::mutex volLock;
std::atomic<bool> readerStop(false);
struct ReaderHold
{
	std::thread th;
	void stop()
	{
		readerStop = true;
		if (th.joinable()) th.join();
		readerStop = false;
	}
	~ReaderHold() { stop(); }
} readerHold;
int takeBody = -1, takeBodyLow = -1;
struct BodyAsk
{
	u16 id;
	float seconds;
    unsigned stamp;
};
xr_vector<BodyAsk> bodyAsked;
std::mutex bodyLock;

bool fileThere(LPCSTR root, LPCSTR rel)
{
	string_path fn;
	return !!FS.exist(fn, root, rel);
}
void parseFloats(LPCSTR s, xr_vector<float>& to)
{
	to.clear();
	if (!s) return;
	const char* p = s;
	for (;;)
	{
		while (*p == ' ' || *p == ',' || *p == '\t') ++p;
		if (!*p) break;
		char* end = nullptr;
		const float v = float(std::strtod(p, &end));
		if (end == p) break;
		to.push_back(v);
		p = end;
	}
}
static unsigned appliedFireMaster=1;
inline void syncFireMaster(){
    const unsigned stamp=PainterFireMasterStamp();
    const unsigned prior=appliedFireMaster;
    if(prior==stamp)return;
    appliedFireMaster=stamp;
    if ((stamp>>1)!=(prior>>1)) clearState(false);
}
void load();
bool ready()
{
	if (!ps_painter_fire) return false;
    syncFireMaster();
	if (!loaded) load();
	return usable;
}
int findTake(LPCSTR name)
{
	for (u32 i = 0; i < takes.size(); ++i)
		if (!xr_strcmp(takes[i].name.c_str(), name)) return int(i);
	return -1;
}
u32 pfvBytes(u32 w, u32 h, u32 d, u32 mips, u32 block)
{
	u64 n = 0;
	for (u32 m = 0; m < mips; ++m) n += u64(_max(1u, (w >> m) / 4)) * u64(_max(1u, (h >> m) / 4)) * u64(block) * u64(_max(1u, d >> m));
	return n > 0x7fffffffull ? 0u : u32(n);
}
// a .pfv file: 'PFV1', width, height, depth, mips, format (80 = BC4, 83 = BC5), bytes; then the blocks, mip by mip, slice by slice
bool readPfv(LPCSTR rel, VolFile& to)
{
	string_path fn;
	if (!FS.exist(fn, "$game_textures$", rel)) return false;
	IReader* r = FS.r_open(fn);
	if (!r) return false;
	bool ok = false;
	if (r->length() >= 28)
	{
		const u8* p = (const u8*)r->pointer();
		u32 head[7];
		memcpy(head, p, 28);
		const bool sizes = head[1] >= 4 && head[2] >= 4 && head[3] >= 1 && head[1] <= 2048 && head[2] <= 2048 && head[3] <= 2048 &&
			head[4] >= 1 && head[4] <= 3 && !((head[1] >> (head[4] - 1)) & 3) && !((head[2] >> (head[4] - 1)) & 3);
		if (head[0] == 0x31564650u && (head[5] == 83u || head[5] == 80u) && sizes && u32(r->length()) == 28u + head[6] &&
			head[6] == pfvBytes(head[1], head[2], head[3], head[4], head[5] == 80u ? 8u : 16u))
		{
			to.w = head[1];
			to.h = head[2];
			to.d = head[3];
			to.mips = head[4];
			to.fmt = head[5];
			to.data.assign(p + 28, p + 28 + head[6]);
			ok = true;
		}
	}
	FS.r_close(r);
	return ok;
}
// one 8-byte block of a single channel -> its 16 values
void unpackBlock(const u8* b, u8* out16)
{
	const u32 e0 = b[0], e1 = b[1];
	u8 pal[8];
	pal[0] = u8(e0);
	pal[1] = u8(e1);
	if (e0 > e1)
		for (u32 i = 1; i < 7; ++i) pal[i + 1] = u8(((7 - i) * e0 + i * e1 + 3) / 7);
	else
	{
		for (u32 i = 1; i < 5; ++i) pal[i + 1] = u8(((5 - i) * e0 + i * e1 + 2) / 5);
		pal[6] = 0;
		pal[7] = 255;
	}
	u64 bits = 0;
	for (u32 i = 0; i < 6; ++i) bits |= u64(b[2 + i]) << (8 * i);
	for (u32 i = 0; i < 16; ++i) out16[i] = pal[(bits >> (3 * i)) & 7];
}
// the texture of a volume file (render thread). The file's bytes are let go afterwards.
bool makeVol(VolFile& f, VolTex& to)
{
	if (f.data.empty() || !f.w) return false;
	D3D11_TEXTURE3D_DESC d;
	ZeroMemory(&d, sizeof(d));
	d.Width = f.w;
	d.Height = f.h;
	d.Depth = f.d;
	d.MipLevels = f.mips;
	d.Usage = D3D11_USAGE_IMMUTABLE;
	d.BindFlags = D3D11_BIND_SHADER_RESOURCE;
	D3D11_SUBRESOURCE_DATA init[3];
	ZeroMemory(init, sizeof(init));
	xr_vector<u8> plain;
	const u32 ch = f.fmt == 80u ? 1u : 2u, block = 8u * ch;
	if (volBC)
	{
		d.Format = ch == 1 ? DXGI_FORMAT_BC4_UNORM : DXGI_FORMAT_BC5_UNORM;
		const u8* p = f.data.data();
		for (u32 m = 0; m < f.mips; ++m)
		{
			const u32 w = f.w >> m, h = f.h >> m, dd = _max(1u, f.d >> m);
			init[m].pSysMem = p;
			init[m].SysMemPitch = (w / 4) * block;
			init[m].SysMemSlicePitch = (w / 4) * (h / 4) * block;
			p += size_t(init[m].SysMemSlicePitch) * dd;
		}
	}
	else
	{
		// a card without block-compressed 3D textures: plain bytes, one a channel a voxel
		d.Format = ch == 1 ? DXGI_FORMAT_R8_UNORM : DXGI_FORMAT_R8G8_UNORM;
		size_t total = 0;
		for (u32 m = 0; m < f.mips; ++m) total += size_t(f.w >> m) * (f.h >> m) * _max(1u, f.d >> m) * ch;
		plain.resize(total);
		const u8* p = f.data.data();
		u8* q = plain.data();
		for (u32 m = 0; m < f.mips; ++m)
		{
			const u32 w = f.w >> m, h = f.h >> m, dd = _max(1u, f.d >> m);
			init[m].pSysMem = q;
			init[m].SysMemPitch = w * ch;
			init[m].SysMemSlicePitch = w * h * ch;
			for (u32 z = 0; z < dd; ++z)
				for (u32 by = 0; by < h / 4; ++by)
					for (u32 bx = 0; bx < w / 4; ++bx, p += block)
					{
						u8 r[16], g[16];
						unpackBlock(p, r);
						if (ch == 2) unpackBlock(p + 8, g);
						for (u32 y = 0; y < 4; ++y)
							for (u32 x = 0; x < 4; ++x)
							{
								u8* px = q + (size_t(z) * h + by * 4 + y) * w * ch + (bx * 4 + x) * ch;
								px[0] = r[y * 4 + x];
								if (ch == 2) px[1] = g[y * 4 + x];
							}
					}
			q += size_t(w) * h * dd * ch;
		}
	}
	const bool made = SUCCEEDED(HW.pDevice->CreateTexture3D(&d, init, &to.tex)) && to.tex &&
		SUCCEEDED(HW.pDevice->CreateShaderResourceView(to.tex, nullptr, &to.srv)) && to.srv;
	xr_vector<u8>().swap(f.data);
	return made;
}
void releaseVol(VolTex& v)
{
	_RELEASE(v.srv);
	_RELEASE(v.tex);
}
bool loadLut()
{
	string_path fn;
	if (!FS.exist(fn, "$game_textures$", "painter\\fire\\fire_lut.pfl")) return false;
	IReader* r = FS.r_open(fn);
	if (!r) return false;
	bool ok = false;
	if (r->length() == 12 + 256 * 2 * 8)
	{
		const u8* p = (const u8*)r->pointer();
		u32 head[3];
		memcpy(head, p, 12);
		if (head[0] == 0x314c4650u && head[1] == 256u && head[2] == 2u)
		{
			D3D11_TEXTURE2D_DESC d;
			ZeroMemory(&d, sizeof(d));
			d.Width = 256;
			d.Height = 2;
			d.MipLevels = 1;
			d.ArraySize = 1;
			d.Format = DXGI_FORMAT_R16G16B16A16_FLOAT;
			d.SampleDesc.Count = 1;
			d.Usage = D3D11_USAGE_IMMUTABLE;
			d.BindFlags = D3D11_BIND_SHADER_RESOURCE;
			D3D11_SUBRESOURCE_DATA init;
			ZeroMemory(&init, sizeof(init));
			init.pSysMem = p + 12;
			init.SysMemPitch = 256 * 8;
			_RELEASE(lutSrv);
			_RELEASE(lutTex);
			ok = SUCCEEDED(HW.pDevice->CreateTexture2D(&d, &init, &lutTex)) && lutTex &&
				SUCCEEDED(HW.pDevice->CreateShaderResourceView(lutTex, nullptr, &lutSrv)) && lutSrv;
		}
	}
	FS.r_close(r);
	return ok;
}
// fire_noise.pfn: 'PFN1', size, mips; then RGBA8 texels, mip by mip
bool loadNoise()
{
	string_path fn;
	if (!FS.exist(fn, "$game_textures$", "painter\\fire\\fire_noise.pfn")) return false;
	IReader* r = FS.r_open(fn);
	if (!r) return false;
	bool ok = false;
	if (r->length() >= 12)
	{
		const u8* p = (const u8*)r->pointer();
		u32 head[3];
		memcpy(head, p, 12);
		const u32 n = head[1], mips = head[2];
		u64 bytes = 0;
		if (head[0] == 0x314e4650u && n >= 8 && n <= 256 && !(n & (n - 1)) && mips >= 1 && mips <= 6 && (n >> (mips - 1)) >= 1)
			for (u32 m = 0; m < mips; ++m) bytes += u64(n >> m) * (n >> m) * (n >> m) * 4ull;
		if (bytes && u64(r->length()) == 12ull + bytes)
		{
			D3D11_TEXTURE3D_DESC d;
			ZeroMemory(&d, sizeof(d));
			d.Width = d.Height = d.Depth = n;
			d.MipLevels = mips;
			d.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
			d.Usage = D3D11_USAGE_IMMUTABLE;
			d.BindFlags = D3D11_BIND_SHADER_RESOURCE;
			D3D11_SUBRESOURCE_DATA init[6];
			ZeroMemory(init, sizeof(init));
			const u8* q = p + 12;
			for (u32 m = 0; m < mips; ++m)
			{
				const u32 s = n >> m;
				init[m].pSysMem = q;
				init[m].SysMemPitch = s * 4;
				init[m].SysMemSlicePitch = s * s * 4;
				q += size_t(s) * s * s * 4;
			}
			_RELEASE(noiseSrv);
			_RELEASE(noiseTex);
			ok = SUCCEEDED(HW.pDevice->CreateTexture3D(&d, init, &noiseTex)) && noiseTex &&
				SUCCEEDED(HW.pDevice->CreateShaderResourceView(noiseTex, nullptr, &noiseSrv)) && noiseSrv;
		}
	}
	FS.r_close(r);
	return ok;
}
// The fire reader prepares one four-channel texel for the two fields sampled at the same coordinates.
void pairFireFields(Take& t)
{
    const VolFile& b=t.fb;
    const VolFile& c=t.fc;
    const u64 texels=u64(b.w)*b.h*b.d;
    if (!pairedPrograms || (xr_strcmp(t.name.c_str(),"pool") && xr_strcmp(t.name.c_str(),"grass_mid") &&
        xr_strcmp(t.name.c_str(),"grass_mid_wind"))) return;
    if (!texels || texels>1048576u || b.w%4 || b.h%4 || b.fmt!=83u || c.fmt!=83u ||
        b.mips!=1 || c.mips!=1 || b.w!=c.w || b.h!=c.h || b.d!=c.d ||
        b.data.size()!=texels || c.data.size()!=texels) return;
    t.coarsePixels.resize(size_t(texels)*4u);
    const u8* bp=b.data.data();
    const u8* cp=c.data.data();
    for (u32 z=0;z<b.d;++z)
        for (u32 y=0;y<b.h;y+=4)
            for (u32 x=0;x<b.w;x+=4,bp+=16,cp+=16)
            {
                u8 heat[16],smoke[16],soot[16],glow[16];
                unpackBlock(bp,heat); unpackBlock(bp+8,smoke);
                unpackBlock(cp,soot); unpackBlock(cp+8,glow);
                for (u32 row=0;row<4;++row)
                {
                    u8* pixel=t.coarsePixels.data()+4*((size_t(z)*b.h+y+row)*b.w+x);
                    for (u32 col=0;col<4;++col,pixel+=4)
                    {
                        const u32 index=row*4+col;
                        pixel[0]=heat[index]; pixel[1]=smoke[index]; pixel[2]=soot[index]; pixel[3]=glow[index];
                    }
                }
            }
}
bool uploadFireFields(Take& t)
{
    if (t.coarsePixels.empty()) return false;
    D3D11_TEXTURE3D_DESC desc={};
    desc.Width=t.fb.w; desc.Height=t.fb.h; desc.Depth=t.fb.d; desc.MipLevels=1;
    desc.Format=DXGI_FORMAT_R8G8B8A8_UNORM; desc.Usage=D3D11_USAGE_IMMUTABLE; desc.BindFlags=D3D11_BIND_SHADER_RESOURCE;
    D3D11_SUBRESOURCE_DATA data={};
    data.pSysMem=t.coarsePixels.data(); data.SysMemPitch=t.fb.w*4; data.SysMemSlicePitch=data.SysMemPitch*t.fb.h;
    const bool ok=SUCCEEDED(HW.pDevice->CreateTexture3D(&desc,&data,&t.b.tex)) &&
        t.b.tex && SUCCEEDED(HW.pDevice->CreateShaderResourceView(t.b.tex,nullptr,&t.b.srv)) && t.b.srv;
    xr_vector<u8>().swap(t.coarsePixels);
    if (!ok) { releaseVol(t.b);return false; }
    xr_vector<u8>().swap(t.fb.data); xr_vector<u8>().swap(t.fc.data);
    t.coarsePaired=true;
    return true;
}
// the reader: every take's files from the disk into memory, in the background, the most urgent takes first (the list's order)
void readerMain()
{
	for (u32 i = 0; i < takes.size() && !readerStop; ++i)
	{
		Take& t = takes[i];
		LPCSTR n = t.name.c_str();
		string256 rel;
		xr_sprintf(rel, "painter\\fire\\%s_va.pfv", n);
		bool ok = readPfv(rel, t.fa);
        if(readerStop)return;
		xr_sprintf(rel, "painter\\fire\\%s_vb.pfv", n);
		ok = ok && readPfv(rel, t.fb);
        if(readerStop)return;
		xr_sprintf(rel, "painter\\fire\\%s_vc.pfv", n);
		ok = ok && readPfv(rel, t.fc);
		if(readerStop)return;
        if (ok) pairFireFields(t);
		if (!ok) Msg("! [painter_fire] take %s: a volume file is missing or not a fire volume, take left out", n);
		std::lock_guard<std::mutex> hold(volLock);
		t.state = ok ? 1 : -1;
	}
}
// once a frame: the textures of one take that has been read
void volumesStep()
{
	if (!usable) return;
	Take* next = nullptr;
	{
		std::lock_guard<std::mutex> hold(volLock);
		for (Take& t : takes)
			if (t.state == 1)
			{
				next = &t;
				break;
			}
	}
	if (!next) return;
	const bool ok = makeVol(next->fa, next->a) &&
		(uploadFireFields(*next) || (makeVol(next->fb,next->b) && makeVol(next->fc,next->c)));
	xr_vector<u8>().swap(next->coarsePixels);
	if (!ok) Msg("! [painter_fire] take %s: its volume textures could not be made, take left out", next->name.c_str());
	std::lock_guard<std::mutex> hold(volLock);
	next->state = ok ? 2 : -1;
}
void load()
{
	loaded = true;
	usable = false;
	takes.clear();
	takeBody = takeBodyLow = -1;
	static const char* need[] = {"r3\\painter_fire_vol.vs", "r3\\painter_fire_vol.ps", "r3\\painter_fire_vol.s"};
	for (const char* f : need)
		if (!fileThere("$game_shaders$", f))
		{
			Msg("! [painter_fire] shaders\\%s is missing: the fire is off", f);
			return;
		}
	string_path fn;
	if (!FS.exist(fn, "$game_config$", "painter\\painter_fire_volumes.ltx"))
	{
		Msg("! [painter_fire] configs\\painter\\painter_fire_volumes.ltx is missing: the fire is off");
		return;
	}
	if (!loadLut())
	{
		Msg("! [painter_fire] textures\\painter\\fire\\fire_lut.pfl is missing or not a fire lookup: the fire is off");
		return;
	}
	if (!loadNoise())
	{
		Msg("! [painter_fire] textures\\painter\\fire\\fire_noise.pfn is missing or not a fire pattern: the fire is off");
		return;
	}
	UINT support = 0;
	volBC = SUCCEEDED(HW.pDevice->CheckFormatSupport(DXGI_FORMAT_BC5_UNORM, &support)) && (support & D3D11_FORMAT_SUPPORT_TEXTURE3D);
	support = 0;
	volBC = volBC && SUCCEEDED(HW.pDevice->CheckFormatSupport(DXGI_FORMAT_BC4_UNORM, &support)) && (support & D3D11_FORMAT_SUPPORT_TEXTURE3D);
	CInifile ini(fn, TRUE, TRUE, FALSE);
	xr_vector<float> v;
	if (ini.section_exist("painter_fire_lut"))
	{
		if (ini.line_exist("painter_fire_lut", "heat_range")) lutHeat = _max(1.f, ini.r_float("painter_fire_lut", "heat_range"));
		if (ini.line_exist("painter_fire_lut", "light_log2"))
		{
			parseFloats(ini.r_string("painter_fire_lut", "light_log2"), v);
			if (v.size() >= 2 && v[1] > v[0])
			{
				lutL0 = v[0];
				lutL1 = v[1];
			}
		}
	}
	for (const auto& s : ini.sections())
	{
		LPCSTR n = s.Name.c_str();
		if (!ini.line_exist(n, "frames") || !ini.line_exist(n, "box") || !ini.line_exist(n, "flame_vox")) continue;
		string256 tex;
		xr_sprintf(tex, "painter\\fire\\%s_va.pfv", n);
		if (!fileThere("$game_textures$", tex))
		{
			Msg("! [painter_fire] take %s: textures\\%s is missing, take left out", n, tex);
			continue;
		}
		Take t;
		t.name = n;
		t.frames = _max(1u, ini.r_u32(n, "frames"));
		t.fps = _max(1.f, ini.r_float(n, "fps"));
		t.loop = ini.line_exist(n, "loop") && ini.r_bool(n, "loop");
		t.smoke = ini.line_exist(n, "smoke") && ini.r_bool(n, "smoke");
		t.wall = ini.line_exist(n, "wall") && ini.r_bool(n, "wall");
		parseFloats(ini.r_string(n, "box"), v);
		if (v.size() < 3 || v[0] <= 0.f || v[1] <= 0.f || v[2] <= 0.f) continue;
		t.width = v[0];
		t.height = v[1];
		t.deep = v[2];
		if (ini.line_exist(n, "foot"))
		{
			parseFloats(ini.r_string(n, "foot"), v);
			if (v.size() >= 2)
			{
				t.footX = clampr(v[0], 0.f, 1.f);
				t.footZ = clampr(v[1], 0.f, 1.f);
			}
		}
		bool fine = true;
		auto three = [&](LPCSTR line, float* to) {
			if (!ini.line_exist(n, line)) { fine = false; return; }
			parseFloats(ini.r_string(n, line), v);
			if (v.size() < 3) { fine = false; return; }
			to[0] = v[0];
			to[1] = v[1];
			to[2] = v[2];
		};
		auto counts = [&](LPCSTR line, u32* to, u32 count) {
			if (!ini.line_exist(n, line)) { fine = false; return; }
			parseFloats(ini.r_string(n, line), v);
			if (v.size() < count) { fine = false; return; }
			for (u32 i = 0; i < count; ++i) to[i] = _max(1u, u32(v[i] + .5f));
		};
		float lo[3] = {0.f, 0.f, 0.f}, hi[3] = {1.f, 1.f, 1.f};
		three("flame_min", lo);
		three("flame_max", hi);
		three("amount", t.amount);
		counts("flame_vox", t.fvox, 3);
		counts("flame_grid", t.fgrid, 2);
		counts("smoke_vox", t.svox, 3);
		counts("smoke_grid", t.sgrid, 2);
		if (ini.line_exist(n, "heat"))
		{
			parseFloats(ini.r_string(n, "heat"), v);
			if (v.size() >= 4)
				for (u32 i = 0; i < 4; ++i) t.heat[i] = v[i];
		}
		if (!fine || hi[0] <= lo[0] || hi[1] <= lo[1] || hi[2] <= lo[2] || t.heat[1] <= 0.f || t.heat[3] <= t.heat[2] ||
			t.fgrid[0] * t.fgrid[1] < t.frames || t.sgrid[0] * t.sgrid[1] < t.frames)
		{
			Msg("! [painter_fire] take %s: its lines in painter_fire_volumes.ltx do not fit together, take left out", n);
			continue;
		}
		t.fmin.set(lo[0], lo[1], lo[2]);
		t.fmax.set(hi[0], hi[1], hi[2]);
		if (ini.line_exist(n, "light")) parseFloats(ini.r_string(n, "light"), t.light);
		if (ini.line_exist(n, "light_h")) parseFloats(ini.r_string(n, "light_h"), t.lightH);
		takes.push_back(t);
	}
	usable = !takes.empty();
	takeBody = findTake("body");
	takeBodyLow = findTake("pool");
	Msg("* [painter_fire] %u fire volumes%s", u32(takes.size()), volBC ? "" : " (unpacked on reading: no block-compressed 3D textures on this card)");
	if (usable)
	{
		readerHold.stop();
		pairedPrograms=false;
        if (fileThere("$game_shaders$", "r3\\painter_fire_vol_pair.s") && fileThere("$game_shaders$", "r3\\painter_fire_vol_dry_pair.s") &&
            fileThere("$game_shaders$", "r3\\painter_fire_vol_pair.ps") && fileThere("$game_shaders$", "r3\\painter_fire_vol_dry_pair.ps"))
        {
            if (!shVolPair) shVolPair.create("painter\\fire_vol_pair");
            if (!shVolDryPair) shVolDryPair.create("painter\\fire_vol_dry_pair");
            pairedPrograms=shVolPair && shVolDryPair;
        }
        readerHold.th = std::thread(readerMain);
	}
}
// Find the static floor from the break position. Starting the ray here keeps an interior ignition below its ceiling.
float groundUnder(const Fvector& p)
{
	if (!g_pGameLevel) return p.y;
	Fvector up = {0.f, 1.f, 0.f}, down = {0.f, -1.f, 0.f};
	collide::rq_result hit;
	float lift = 1.5f;
	if (g_pGameLevel->ObjectSpace.RayPick(p, up, 1.5f, collide::rqtStatic, hit, nullptr)) lift = _max(hit.range - .05f, 0.f);
	Fvector start = p;
	start.y += lift;
	if (g_pGameLevel->ObjectSpace.RayPick(start, down, 3.5f + lift, collide::rqtStatic, hit, nullptr)) return start.y - hit.range;
	return p.y;
}
struct Rng
{
	u32 s;
	float next() // 0..1
	{
		s = s * 1664525u + 1013904223u;
		return float((s >> 8) & 0xffff) / 65535.f;
	}
};
Fire* add(int take, const Fvector& pos,bool woodCell=false,bool woodImpact=false)
{
	if (take < 0) return nullptr;
	// The field owns up to 240 live cells. Direct impacts with spreading off have
    // sixteen additional places. Prefer a field picture already in its fade-out.
    Fire* reuse=nullptr;
    if(fires.size()>=768){
        if(!woodCell && !woodImpact)return nullptr;
        if(fires.size()>=768+240){
            unsigned liveCells=0,impacts=0;
            Fire* oldestImpact=nullptr;
            for(Fire& old:fires){
                if(old.woodCellPicture){
                    if(old.cell)++liveCells;
                    else if(!reuse)reuse=&old;
                }
                if(old.woodImpactPicture){
                    ++impacts;
                    if(!oldestImpact || old.born<oldestImpact->born)oldestImpact=&old;
                }
            }
            if(woodCell && liveCells>=240)return nullptr;
            if(!reuse && fires.size()>=768+240+16){
                if(woodCell && impacts>16)reuse=oldestImpact;
                if(!reuse)return nullptr;
            }
        }
    }
	static u64 nextFireLightId=1;
	Fire f;
	f.stableLightId=nextFireLightId++;
	f.take = take;
	f.born = Device.dwTimeGlobal;
	f.pos = pos;
	{
		const u32 h = u32(f.stableLightId) * 2654435761u; // firerework: every fire plays its take at its own speed
		f.rate = .86f + .3f * float((h >> 20) & 1023u) / 1023.f;
	}
    f.woodCellPicture=woodCell;f.woodImpactPicture=woodImpact;
    if(reuse){ *reuse=f;return reuse; }
	fires.push_back(f);
	return &fires.back();
}
// where a fire is in its take right now: false = not showing (not yet, or over)
bool state(const Fire& f, const Take& k, u32 now, float& frame, float& fade, bool& over)
{
	over = false;
	const float t = float(now - f.born) * .001f - f.delay;
	if (t < 0.f) return false;
	fade = 1.f;
	if (k.loop)
	{
		if (t >= f.life) { over = true; return false; }
		frame = fmodf(t * k.fps * f.rate + f.phase, float(k.frames)); // firerework: at its own speed
		if (f.fadeOut > 0.f) fade = _min(fade, (f.life - t) / f.fadeOut);
	}
	else
	{
		frame = t * k.fps;
		const float last = float(k.frames - 1);
		if (frame >= last) { over = true; return false; }
		if (f.fadeOut > 0.f) fade = _min(fade, (last / k.fps - t) / f.fadeOut);
	}
	if (f.fadeIn > 0.f) fade = _min(fade, t / f.fadeIn);
	fade = clampr(fade, 0.f, 1.f);
	return fade > 0.f;
}
namespace PFS = PainterFireSpread;

u32 spreadRays = 0; // questions asked of the level in this frame

Fvector triNormal(const collide::rq_result& hit, const Fvector& dir)
{
	const CDB::TRI& tri = g_pGameLevel->ObjectSpace.GetStaticTris()[hit.element];
	const Fvector* v = g_pGameLevel->ObjectSpace.GetStaticVerts();
	Fvector n;
	n.mknormal(v[tri.verts[0]], v[tri.verts[1]], v[tri.verts[2]]);
	if (n.dotproduct(dir) > 0.f) n.invert();
	return n;
}
LPCSTR triMaterial(const collide::rq_result& hit)
{
	const CDB::TRI& tri = g_pGameLevel->ObjectSpace.GetStaticTris()[hit.element];
	SGameMtl* m = GMLib.GetMaterialByIdx(u16(tri.material));
	return m ? m->m_Name.c_str() : "";
}
// collision the level only has to steer the game (invisible walls, kill zones) is not a surface
bool notASurface(LPCSTR m) { return strstr(m, "fake") || strstr(m, "\\occ") || strstr(m, "\\death"); }
// the first real surface along a ray of the static level
bool pick(const Fvector& from, const Fvector& dir, float range, collide::rq_result& hit, float& travelled)
{
	Fvector start = from;
	travelled = 0.f;
	for (int tries = 0; tries < 4 && range > .02f; ++tries)
	{
		++spreadRays;
		if (!g_pGameLevel->ObjectSpace.RayPick(start, dir, range, collide::rqtStatic, hit, nullptr) || hit.O || hit.element < 0) return false;
		if (!notASurface(triMaterial(hit)))
		{
			travelled += hit.range;
			return true;
		}
		const float step = hit.range + .05f;
		start.mad(dir, step);
		range -= step;
		travelled += step;
	}
	return false;
}
// how thick the grass grows at a point of the ground, 0..1 (the level's grass layer)
float grassAt(const Fvector& p)
{
	CDetailManager* dm = RImplementation.Details;
	if (!dm || !dm->dtSlots) return 0.f;
	// the slot the grass is drawn in: slot sx covers [sx, sx + 1) * dm_slot_size (DetailManager_CACHE.cpp)
	DetailSlot& slot = dm->QueryDB(iFloor(p.x / dm_slot_size), iFloor(p.z / dm_slot_size));
	const float base = slot.r_ybase();
	if (p.y < base - .6f || p.y > base + slot.r_yheight() + .6f) return 0.f; // another storey: not this ground
	float most = 0.f;
	for (u32 i = 0; i < 4; ++i)
	{
		if (slot.r_id(i) == DetailSlot::ID_Empty) continue;
		const DetailPalette& a = slot.palette[i];
		most = _max(most, float(a.a0 + a.a1 + a.a2 + a.a3) / 60.f);
	}
	return most;
}
// is something overhead (a tree is not)? `height`: how high it lies there
bool roofedAt(const Fvector& p, float* height = nullptr, Fvector* facing = nullptr)
{
	Fvector from = p, up = {0.f, 1.f, 0.f};
	from.y += .3f;
	collide::rq_result hit;
	float d = 0.f;
	if (!pick(from, up, 40.f, hit, d)) return false;
	LPCSTR over = triMaterial(hit);
	if (strstr(over, "bush") || strstr(over, "tree_trunk")) return false; // a tree overhead is no roof: it keeps no rain off
	if (height) *height = from.y + d;
	if (facing)
	{
		Fvector n = triNormal(hit, up);
		if (n.y > 0.f) n.invert();
		*facing = n;
	}
	return true;
}
// How high the ceiling lies over a place on the ground, for a fire that is drawn there: nothing of it shows above that.
// 0 = the place lies in the open. Something low over the very place (a table, a pipe, a wagon) is no ceiling: then the
// ceiling is what is over the ground a step around it; with open sky there the fire stands free.
float ceilingOver(const Fvector& ground)
{
	const float room = 1.8f;
	float h = 0.f;
	if (!roofedAt(ground, &h)) return 0.f;
	if (h - ground.y >= room) return h;
	float high = 0.f;
	bool sky = false;
	for (int k = 0; k < 4; ++k)
	{
		const Fvector q = {ground.x + (k == 0 ? .9f : k == 1 ? -.9f : 0.f), ground.y, ground.z + (k == 2 ? .9f : k == 3 ? -.9f : 0.f)};
		float h2 = 0.f;
		if (!roofedAt(q, &h2))
			sky = true;
		else if (h2 - ground.y >= room && (high == 0.f || h2 < high))
			high = h2;
	}
	return high != 0.f ? high : sky ? 0.f : h;
}
// what a surface is to a fire: by the material the level gives it, on the ground by the grass that grows there
// How thick a trunk is at a place (its radius, m): the level's "tree_trunk" triangles within 0.7 m, how far their middles
// lie from their own middle on average. 0.25 when too few are there to tell.
float trunkRadius(const Fvector& p)
{
	if (!g_pGameLevel) return .25f;
	static xrXRC xrc;
	const Fvector dim = {.7f, .4f, .7f};
	++spreadRays;
	xrc.box_options(0);
	xrc.box_query(g_pGameLevel->ObjectSpace.GetStaticModel(), p, dim);
	Fvector mids[48];
	u32 n = 0;
	for (CDB::RESULT* r = xrc.r_begin(); r != xrc.r_end() && n < 48; ++r)
	{
		SGameMtl* m = GMLib.GetMaterialByIdx(u16(r->material));
		if (!m || !strstr(m->m_Name.c_str(), "tree_trunk")) continue;
		Fvector nn;
		nn.mknormal(r->verts[0], r->verts[1], r->verts[2]);
		if (_abs(nn.y) > .7f) continue; // a cut end
		mids[n].add(r->verts[0], r->verts[1]);
		mids[n].add(r->verts[2]);
		mids[n].mul(1.f / 3.f);
		++n;
	}
	if (n < 4) return .25f;
	float cx = 0.f, cz = 0.f;
	for (u32 i = 0; i < n; ++i)
	{
		cx += mids[i].x;
		cz += mids[i].z;
	}
	cx /= float(n);
	cz /= float(n);
	float sum = 0.f;
	for (u32 i = 0; i < n; ++i) sum += _sqrt((mids[i].x - cx) * (mids[i].x - cx) + (mids[i].z - cz) * (mids[i].z - cz));
	return clampr(sum / float(n), .05f, .8f);
}
void fuelOf(LPCSTR m, PFS::Kind kind, const Fvector& p, PFS::Site& s)
{
	float overhead = 0.f;
	s.roofed = roofedAt(p, &overhead);
	s.lid = s.roofed ? overhead : 0.f; // how high what is overhead lies: a fire does not go through it
	s.fuel = PFS::NOTHING;
	s.amount = 0.f;
	s.trunk = false;
	if (strstr(m, "water")) return;
	if (strstr(m, "wood") || strstr(m, "tree_trunk"))
	{
		s.fuel = PFS::WOOD;
		s.amount = .85f;
		s.trunk = strstr(m, "tree_trunk") != nullptr;
		// a trunk: as much as it is thick (a pole a quarter of a metre across 0.47, half a metre and more 0.85)
		if (s.trunk) s.amount = clampr(.3f + 1.4f * trunkRadius(p), .35f, .85f);
	}
	else if (strstr(m, "bush"))
	{
		s.fuel = PFS::BRUSH;
		s.amount = .8f;
	}
	else if (strstr(m, "cloth"))
	{
		s.fuel = PFS::CLOTH;
		s.amount = .7f;
	}
	else if (strstr(m, "grass"))
	{
		s.fuel = PFS::GRASS;
		s.amount = .7f;
	}
	if (kind == PFS::GROUND && s.fuel != PFS::WOOD)
	{
		const float g = grassAt(p);
		if (g > .04f && g >= s.amount * .5f)
		{
			s.fuel = PFS::GRASS;
			s.amount = clampr(.25f + g, 0.f, 1.f);
		}
	}
	// In a room with a noncombustible shell, its flammable contents can still catch fire.
	if (s.fuel == PFS::NOTHING && s.roofed)
	{
		s.fuel = PFS::CONTENTS;
		s.amount = kind == PFS::GROUND ? .55f : kind == PFS::WALL ? .4f : .3f;
	}
}
// These queries only run while discovering a new static fuel contact or creating its picture.
struct PainterFireColumnEntry
{
    Fvector at={0.f,0.f,0.f};PainterFireColumn::Section section;int family=0;
};
PainterFireColumnEntry fireColumnCache[1024];
bool painterColumnAt(const Fvector& p,int family,PainterFireColumn::Section& section)
{
    section={};
    if (!g_pGameLevel || (family<1 || family>3)) return false;
    const u32 hash=u32(iFloor(p.x*16.f))*73856093u ^ u32(iFloor(p.y*16.f))*19349663u ^
        u32(iFloor(p.z*16.f))*83492791u ^ u32(family)*2654435761u;
    PainterFireColumnEntry& e=fireColumnCache[hash&1023u];
    if(e.family==family && e.at.x==p.x && e.at.y==p.y && e.at.z==p.z) {
        section=e.section;return section.valid();
    }
    static xrXRC sections;
    const Fvector dim={1.72f,family>=2?2.07f:.44f,1.72f};
    ++spreadRays;sections.box_options(0);sections.box_query(g_pGameLevel->ObjectSpace.GetStaticModel(),p,dim);
    auto slice=[&](float height,PainterFireColumn::Point where,bool timber,PainterFireColumn::Ring& ring){
        PainterFireColumn::Contours paths;
        for(CDB::RESULT* r=sections.r_begin();r!=sections.r_end();++r){
            const auto& a=r->verts[0];const auto& b=r->verts[1];const auto& c=r->verts[2];
            if ((a.y>height && b.y>height && c.y>height)||(a.y<=height && b.y<=height && c.y<=height))continue;
            SGameMtl* m=GMLib.GetMaterialByIdx(u16(r->material));if(!m)continue;
            const char* name=m->m_Name.c_str();
            if(timber ? !strstr(name,"wood") : !strstr(name,"tree_trunk"))continue;
            paths.intersect(r->verts,height);
        }
        // The source section must contain the actual material hit. Other rows may lean.
        return paths.nearest(where,timber,ring,height==p.y?.004f:.46f,family==3?4u:5u);
    };
    PainterFireColumn::qualify(p.y,{p.x,p.z},family>=2,slice,section);
    e.at=p;e.family=family;e.section=section;return section.valid();
}

// A ground volume needs a real upward-facing support, distinct from a branch,
// a cap on a timber post or a leaf collision. A usable wooden roof still
// qualifies even when a post stands immediately under it.
bool woodyCanopy(LPCSTR m)
{
    return strstr(m,"tree_trunk") || strstr(m,"tree_branch") || strstr(m,"leaf") ||
        strstr(m,"leaves") || strstr(m,"bush");
}
bool woodPostCap(const Fvector& p,const Fvector& n)
{
    // Most decks and roofs continue beyond a post footprint. A short downward
    // sample on either axis certifies that continuity before looking for a post.
    const Fvector down={0.f,-1.f,0.f};
    for(int axis=0;axis<2;++axis)
    {
        bool spans=true;
        for(int sign=-1;sign<=1;sign+=2)
        {
            Fvector q=p;
            if(axis)q.z+=.91f*float(sign);else q.x+=.91f*float(sign);
            const float y=p.y-(n.x*(q.x-p.x)+n.z*(q.z-p.z))/n.y;
            q.y=y+.09f;
            collide::rq_result h;float distance=0.f;
            if(!pick(q,down,.18f,h,distance) || !strstr(triMaterial(h),"wood") || triNormal(h,down).y<.5f)
            {spans=false; break;}
        }
        if(spans)return false;
    }
    // Only reject actual closed, tall wooden columns directly beneath this hit.
    for(int axis=0;axis<2;++axis)
    {
        Fvector q=p; q.y-=.18f;
        Fvector dir={axis?0.f:-1.f,0.f,axis?-1.f:0.f};
        q.mad(dir,-1.7f);
        collide::rq_result h;float distance=0.f;
        if(!pick(q,dir,1.74f,h,distance) || !strstr(triMaterial(h),"wood") || _abs(triNormal(h,dir).y)>.7f)continue;
        Fvector side;side.mad(q,dir,distance);
        PainterFireColumn::Section column;
        if(!painterColumnAt(side,3,column))continue;
        const float f=clampr((q.y-column.y0)/_max(.001f,column.y1-column.y0),0.f,1.f);
        const float cx=column.x0+f*(column.x1-column.x0),cz=column.z0+f*(column.z1-column.z0);
        const float rx=column.rx0+f*(column.rx1-column.rx0),rz=column.rz0+f*(column.rz1-column.rz0);
        if(_abs(p.x-cx)<=rx+.03f && _abs(p.z-cz)<=rz+.03f)return true;
    }
    return false;
}
bool fireSupport(const collide::rq_result& h,const Fvector& p,const Fvector& n)
{
    const char* m=triMaterial(h);
    if(n.y<.5f || woodyCanopy(m) || notASurface(m) || strstr(m,"water"))return false;
    return !strstr(m,"wood") || !woodPostCap(p,n);
}
bool fireSupportBelow(Fvector from,float reach,collide::rq_result& hit,Fvector& p,Fvector& normal)
{
    if(!g_pGameLevel)return false;
    const Fvector down={0.f,-1.f,0.f};
    for(int i=0;i<12 && reach>.01f;++i)
    {
        float distance=0.f;
        if(!pick(from,down,reach,hit,distance))return false;
        p=from;p.y-=distance;
        normal=triNormal(hit,down);
        if(fireSupport(hit,p,normal))return true;
        const float step=distance+.012f;from.y-=step;reach-=step;
    }
    return false;
}
bool firePictureFloor(const Fvector& p,float up,float down,float& height)
{
    Fvector from=p,point,normal;from.y+=up;
    collide::rq_result hit;
    if(!fireSupportBelow(from,up+down,hit,point,normal))return false;
    height=point.y;return true;
}

struct LevelWorld : PFS::World
{
	bool ground(const PFS::V3& at, float up, float down, PFS::Site& out) override
	{
        const Fvector from={at.x,at.y+up,at.z};
        collide::rq_result hit;Fvector p,n;
        if(!fireSupportBelow(from,up+down,hit,p,n))return false;
		out.p = {p.x, p.y, p.z};
		out.n = {n.x, n.y, n.z};
		fuelOf(triMaterial(hit), PFS::GROUND, p, out);
		return true;
	}
    // Static rays cull back faces. Query the open interval in reverse as well
    // so the winding of an intervening single-sided board cannot expose fuel.
    static bool woodBackBlocked(const Fvector& hit,const Fvector& direction,float distance)
    {
        if(distance<=.07f)return false;
        Fvector back=direction;back.invert();
        Fvector start;start.mad(hit,back,.035f);
        collide::rq_result blocker;float traveled=0.f;
        return pick(start,back,distance-.07f,blocker,traveled);
    }
	bool wall(const PFS::V3& from_, const PFS::V3& dir_, float reach, PFS::Site& out) override
	{
		const Fvector from = {from_.x, from_.y, from_.z}, dir = {dir_.x, dir_.y, dir_.z};
		collide::rq_result hit;
		float d = 0.f;
		if (!pick(from, dir, reach, hit, d)) return false;
		Fvector n = triNormal(hit, dir);
		if (_abs(n.y) > .6f) return false; // a slope or a step, not a wall
		n.y = 0.f;
		n.normalize();
		Fvector p, front;
		p.mad(from, dir, d);
		front.mad(p, n, .3f);
		out.p = {p.x, p.y, p.z};
		out.n = {n.x, 0.f, n.z};
		fuelOf(triMaterial(hit), PFS::WALL, front, out);
        if(out.fuel==PFS::WOOD && woodBackBlocked(p,dir,d)){
            // A wall was encountered. Block this direction, including the ground behind it.
            out.fuel=PFS::NOTHING;out.amount=0.f;out.trunk=false;out.woodFace=-1;
            return true;
        }
		out.woodFace=out.fuel==PFS::WOOD ? hit.element : -1;
		if(out.fuel==PFS::WOOD && !out.trunk && strstr(triMaterial(hit),"wood")){
			PainterFireColumn::Section section;
			out.trunk=painterColumnAt(p,2,section);
		}
		return true;
	}
	// firesides (2026-11-24): the other side of a thin wooden wall standing in the open - see PainterFireSpread.h thinBack.
	// (Rays pass a face from behind: a board with one face gives no answer from its back. Open air behind it = a board;
	// anything else - the inside of something solid, a wall close behind - is left alone.)
	int thinBack(const PFS::V3& p_, const PFS::V3& n_, PFS::Site& out) override
	{
		if (!ps_painter_fire_fence_foot) return 0;
		auto roofed = [&](float off) {
			PFS::Site over;
			const PFS::V3 air = {p_.x + n_.x * off, p_.y, p_.z + n_.z * off};
			return ceiling(air, 6.f, over) && over.fuel != PFS::BRUSH && !over.trunk;
		};
		if (roofed(.45f)) return 0;
		const PFS::V3 behind = {p_.x - n_.x * .6f, p_.y, p_.z - n_.z * .6f};
		if (wall(behind, n_, .58f, out))
		{
			const float facing = out.n.x * n_.x + out.n.z * n_.z;
			const float depth = (p_.x - out.p.x) * n_.x + (p_.z - out.p.z) * n_.z;
			if (facing > -.65f || depth < .004f || depth > .35f || out.fuel != PFS::WOOD || out.trunk) return 0;
			return roofed(-depth - .45f) ? 0 : 1;
		}
		const Fvector from = {p_.x - n_.x * 1.2f, p_.y, p_.z - n_.z * 1.2f}, dir = {n_.x, 0.f, n_.z};
		collide::rq_result hit;
		float d = 0.f;
		if (pick(from, dir, 1.18f, hit, d)) return 0;
		Fvector away = dir; away.invert();
		const Fvector close = {p_.x - n_.x * .03f, p_.y, p_.z - n_.z * .03f};
		if (pick(close, away, 1.15f, hit, d)) return 0;
		if (roofed(-.45f)) return 0;
		out.p = {p_.x - n_.x * .02f, p_.y, p_.z - n_.z * .02f};
		out.n = {-n_.x, 0.f, -n_.z};
		return 2;
	}
	bool trunk(const PFS::V3& at, float radius, float low, float high, PFS::Site& out) override
	{
        const Fvector origin={at.x,at.y,at.z};
        const Fvector centre={at.x,at.y+(low+high)*.5f,at.z};
        const Fvector half={radius,_max(.05f,(high-low)*.5f),radius};
        static xrXRC query;
        ++spreadRays;
        query.box_options(0);
        query.box_query(g_pGameLevel->ObjectSpace.GetStaticModel(),centre,half);
        float best=radius*radius+.0001f;
        Fvector bp={},bn={};
        int face=-1;
        bool tree=false;
        for(CDB::RESULT* r=query.r_begin();r!=query.r_end();++r){
            SGameMtl* m=GMLib.GetMaterialByIdx(u16(r->material));
            if(!m)continue;
            LPCSTR name=m->m_Name.c_str();
            if(notASurface(name))continue;
            const bool trunk=strstr(name,"tree_trunk")!=nullptr;
            if(!trunk&&!strstr(name,"wood"))continue;
            Fvector normal;normal.mknormal(r->verts[0],r->verts[1],r->verts[2]);
            if(_abs(normal.y)>.6f)continue;
            Fvector pt;
            float distance=best;
            if(!painterFireTriangleContact(r->verts,origin,radius,low,high,pt,distance))continue;
            best=distance;bp=pt;bn=normal;face=r->id;tree=trunk;
        }
        if(face<0)return false;
        bn.y=0.f;bn.normalize();
        // Query the sightline only for a ground cell. An upper trunk sample starts
        // inside the previous cross-section and is qualified by the climbing link.
        if(low>0.f){
            Fvector src={at.x,bp.y,at.z};
            Fvector dir; dir.sub(bp,src);
            const float length=dir.magnitude();
            if(length>.045f){
                dir.div(length);
                collide::rq_result obstacle;float d=0.f;
                if(pick(src,dir,length-.035f,obstacle,d))return false;
                if(woodBackBlocked(bp,dir,length))return false;
                if(bn.dotproduct(dir)>0.f)bn.invert();
            }
        }
        PainterFireColumn::Section column;
        const bool round=tree || painterColumnAt(bp,2,column);
        if(low==0.f && !round)return false;
        out.p={bp.x,bp.y,bp.z};out.n={bn.x,0.f,bn.z};
        Fvector front;front.mad(bp,bn,.3f);
        fuelOf(tree?"tree_trunk":"wood",PFS::WALL,front,out);
        out.trunk=round;out.woodFace=face;
        return true;
	}
	bool ceiling(const PFS::V3& from_, float reach, PFS::Site& out) override
	{
		const Fvector from = {from_.x, from_.y, from_.z}, dir = {0.f, 1.f, 0.f};
		collide::rq_result hit;
		float d = 0.f;
		if (!pick(from, dir, reach, hit, d)) return false;
		const Fvector n = triNormal(hit, dir);
		if (n.y > -.5f) return false;
		const Fvector p = {from.x, from.y + d, from.z};
		out.p = {p.x, p.y, p.z};
		out.n = {n.x, n.y, n.z};
		fuelOf(triMaterial(hit), PFS::CEILING, from, out);
		out.woodFace=out.fuel==PFS::WOOD ? hit.element : -1;
		out.roofed = true;
		if (out.fuel != PFS::BRUSH && !out.trunk) out.lid = p.y; // a ceiling is what is overhead itself (a tree's crown is not)
		if (out.fuel == PFS::NOTHING)
		{
			out.fuel = PFS::CONTENTS;
			out.amount = .3f;
		}
		return true;
	}
};
LevelWorld world;
PFS::Field field;

// what the weather is to a fire. The level's wind speed number runs to about 1000 in a storm.
PFS::Weather weather()
{
	PFS::Weather w;
	if (!g_pGamePersistent) return w;
	CEnvironment& env = g_pGamePersistent->Environment();
	if (!env.CurrentEnv) return w;
	const float speed = clampr(env.CurrentEnv->wind_velocity / 1000.f, 0.f, 1.f) * 12.f; // m/s
	const float a = -env.CurrentEnv->wind_direction + PI_DIV_2; // as the grass and the trees sway (CEnvironment::OnFrame)
	w.windX = speed * _cos(a);
	w.windZ = speed * _sin(a);
	w.wet = clampr(env.wetness_factor, 0.f, 1.f);
	w.rain = clampr(env.CurrentEnv->rain_density, 0.f, 1.f);
	return w;
}

// the picture of a cell
struct Shown
{
	u32 fire = 0; // Fire::id, 0 = none
	int cls = -1; // which take it plays (0 spot, 1 low, 2 mid, 3 high)
	float mult = 1.f; // far from the camera fewer cells carry a picture, and those are drawn bigger
	float off = 0.f; // seconds its size has asked for another take
	float look = 0.f; // seconds since it was last asked whether a bigger neighbour shows its flames
	int form = 0; // what it shows: 0 a fire on the ground, 1 a column around a trunk, 2 a fire on a wall, 3 a bush or crown
	u32 foot = 0; // fencefoot: Fire::id of the ground take at the wall's foot (the lowest burning wood cell of a column), 0 = none
	float footLook = 0.f; // ... seconds since it was last asked whether this cell is still the lowest
	float caught = 0.f;   // firerework: the field's clock when it caught (its flame grows from small from then)
};
xr_vector<Shown> shown;
bool spreadSources = false; // the cells with thrown fuel show a flame of their own (the console's test fire)
u32 nextFireId = 1;
u64 newFrontGroup()
{
    static u64 sequence = 0;
    return ++sequence;
}

Fire* fireOf(u32 id)
{
	if (!id) return nullptr;
	for (Fire& f : fires)
		if (f.id == id) return &f;
	return nullptr;
}
int classOf(float size) { return size < .2f ? 0 : size < .42f ? 1 : size < .8f ? 2 : 3; }
float scaleOf(int cls, float size)
{
	switch (cls)
	{
	case 0: return .8f + 2.f * size;
	case 1: return .55f + size;
	case 2: return .5f + .6f * size;
	default: return _min(.45f + .6f * size, 1.25f);
	}
}
void endPicture(u32 id, float seconds)
{
	if (Fire* f = fireOf(id))
	{
		f->cell = 0;
		f->fadeOut = seconds;
		f->life = _min(f->life, float(Device.dwTimeGlobal - f->born) * .001f - f->delay + seconds);
	}
}
// a picture for a burning cell: which take, how big, what smoke
// How big a cell's picture is drawn (its take's own scale): by what it shows and how big the cell's flames are.
float pictureScaleOf(const Shown& s, float size)
{
	switch (s.form)
	{
	case 1: return (.5f + .45f * size) * s.mult;             // a column around a trunk (the take is 3.6 m tall)
	case 4: return (.47f+.34f*size)*s.mult;             // a finite wood face
	case 2: return scaleOf(s.cls, size) * s.mult;            // along a wall or a fence: the ground's own takes
	case 3: return _min(.55f + .75f * size, 1.45f) * s.mult; // a bush, a crown: the tall take, drawn big
	default: return scaleOf(s.cls, size) * s.mult;
	}
}
// a column around a trunk: smaller on a thinner trunk (the cell's fuel amount: 0.85 for a thick one)
float trunkThin(const Shown& s, const PFS::Cell& c) { return s.form == 1 ? clampr(c.amount / .85f, .4f, 1.f) : 1.f; }
float pictureScale(const Shown& s, const PFS::Cell& c) { return pictureScaleOf(s, c.size) * trunkThin(s, c); }
float pictureGrowth(const Shown& s, const PFS::Cell& c)
{
	float k = 1.f;
	if (!c.source)
	{
		const float g = clampr((field.clock - s.caught) / (1.5f + .45f * PFS::rule(c.fuel).catchTime), 0.f, 1.f);
		k = .12f + .88f * g * g * (3.f - 2.f * g);
	}
	if (c.state == PFS::DYING) k *= clampr(1.f - c.t / PFS::rule(c.fuel).dieTime, 0.f, 1.f);
	return k;
}
// The way the fire's front runs at a ground cell: across from what has burnt toward what has not. A cell in the middle of
// a fire has no front of its own: there it runs across the way the fire came from. False when neither is known.
bool frontAt(const PFS::Cell& c, Fvector& along)
{
	static const int nb[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
	const float S = field.cellSize;
	float nx = 0.f, nz = 0.f;
	for (int k = 0; k < 8; ++k)
	{
		const uint32_t j = field.groundAt(float(c.kx + nb[k][0]) * S, c.p.y, float(c.kz + nb[k][1]) * S);
		const float w = (nb[k][0] && nb[k][1]) ? .7f : 1.f;
		// what has not burnt lies ahead of the front; what burns or has burnt lies behind it
		const bool ahead = j == PFS::Field::NO_CELL || field.cells[j].state == PFS::COLD;
		nx += (ahead ? w : -w) * float(nb[k][0]);
		nz += (ahead ? w : -w) * float(nb[k][1]);
	}
	if (nx * nx + nz * nz < .6f)
	{
		nx = c.p.x - c.ox;
		nz = c.p.z - c.oz;
		if (nx * nx + nz * nz < .5f) return false;
	}
	const float len = _sqrt(nx * nx + nz * nz);
	along.set(-nz / len, 0.f, nx / len);
	return true;
}
// A fire on the ground of a room: how high the ceiling over it lies - its picture ends there. 0 = it burns in the open.
// Something low over the cell's own place (a table, a pipe, a wagon) is no ceiling: then the ceiling is what the cells
// around it stand under; with open sky around it the fire stands free, as before.
float roomTop(const PFS::Cell& c)
{
	const float room = 1.8f;
	if (c.lid == 0.f) return 0.f;
	if (c.lid - c.p.y >= room) return c.lid;
	static const int nb[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
	const float S = field.cellSize;
	float high = 0.f;
	bool sky = false;
	for (int k = 0; k < 8; ++k)
	{
		const uint32_t j = field.groundAt(float(c.kx + nb[k][0]) * S, c.p.y, float(c.kz + nb[k][1]) * S);
		if (j == PFS::Field::NO_CELL) continue;
		const float lid = field.cells[j].lid;
		if (lid == 0.f)
			sky = true;
		else if (lid - c.p.y >= room && (high == 0.f || lid < high))
			high = lid;
	}
	return high != 0.f ? high : sky ? 0.f : c.lid;
}
// Share the classification between bottle impacts and field pictures. A planar fire
// also covers a trunk for which the two closed cross sections could not be resolved.
template<class Site>
void woodFlameContact(const Site& c,Fire& fire,float below)
{
    if(c.fuel!=PFS::WOOD)return;
    const Fvector p={c.p.x,c.p.y,c.p.z},n={c.n.x,c.n.y,c.n.z};
    if(c.trunk && (painterColumnAt(p,2,fire.wrapColumn)||painterColumnAt(p,1,fire.wrapColumn)))return;
    Fvector triangle[3];
    const Fvector* vertices=nullptr;
    if(c.woodFace>=0 && g_pGameLevel)
    {
        const auto* tris=g_pGameLevel->ObjectSpace.GetStaticTris();
        const auto* points=g_pGameLevel->ObjectSpace.GetStaticVerts();
        if(tris && points)
        {
            for(int i=0;i<3;++i)triangle[i]=points[tris[c.woodFace].verts[i]];
            vertices=triangle;
        }
    }
    fire.wrapFace=PainterWoodFlame::face(p,n,vertices,field.cellSize,below);
    if(fire.wrapFace.valid())
    {
        fire.pos.x=p.x;fire.pos.y=fire.wrapFace.base;fire.pos.z=p.z;
        fire.normal=n;
    }
}
u32 startPicture(u32 index, const PFS::Cell& c, int cls, float fadeIn, const PFS::Weather& wx)
{
	static const char* names[4] = {"spot", "grass_low", "grass_mid", "grass_high"};
	static const char* windy[4] = {"spot", "grass_low_wind", "grass_mid_wind", "grass_high_wind"};
	Shown& sh = shown[index];
	Fvector pos = {c.p.x, c.p.y, c.p.z};
	const u32 hash = index * 2654435761u;
	bool mirror = (hash & 0x10000u) != 0;
	float yaw = float(hash >> 20) * (PI_MUL_2 / 4096.f); // every fire stands turned its own way
	LPCSTR name = names[cls];
	Fvector normal = {0.f, 0.f, 0.f};
	const float S = field.cellSize;
	sh.form = 0;
	float cap = 0.f, air = 0.f, top = 0.f;
	Fvector capN = {0.f, 0.f, 0.f}, capAt = {c.p.x, c.p.y, c.p.z}; // the ceiling's way, and where its height was found
	float trunkReach = 0.f; // firetrunk
	if (c.kind == PFS::WALL && c.trunk)
	{
		// A tree's trunk: a column of flame around the trunk itself, not a fire standing beside it. One for every cell up
		// the trunk, each reaching into the next: the fire climbs the tree.
		sh.form = 1;
		name = "body";
		air = .25f;
		const float into = .28f * clampr(c.amount / .85f, .4f, 1.f); // (to the middle of a thinner trunk: less)
		pos.x -= c.n.x * into;
		pos.z -= c.n.z * into;
		pos.y -= .5f * S;
		if (ps_painter_fire_fence_foot)
		{
			// firetrunk: the lowest cell of a trunk burns down to the ground (found in front of the trunk's face), and every
			// column reaches 2 m over its cell, into the next one up the trunk
			const Fvector outside = {c.p.x + c.n.x * .35f, c.p.y, c.p.z + c.n.z * .35f};
			float ground = 0.f;
			if (firePictureFloor(outside, .3f, 1.6f, ground) && ground < c.p.y - .2f)
			{
				pos.y = _min(pos.y, ground - .05f);
				air = .08f;
			}
			trunkReach = c.p.y + 2.f - pos.y;
		}
	}
	else if (c.fuel == PFS::BRUSH && c.kind != PFS::GROUND)
	{
		// A bush, a tree's crown: a tall fire from inside it, big enough to take the whole of it in.
		sh.form = 3;
		name = "body";
		air = .45f;
		if (c.kind == PFS::WALL)
		{
			pos.x -= c.n.x * .6f;
			pos.z -= c.n.z * .6f;
			pos.y -= .5f * S;
		}
		else
			pos.y -= 1.2f;
		{
			Fvector in = pos;
			if (c.kind == PFS::WALL)
			{
				in.x -= c.n.x * .4f;
				in.z -= c.n.z * .4f;
			}
			float ground = 0.f;
			if (firePictureFloor(in, .2f, 3.5f, ground))
			{
				pos.set(in.x, ground, in.z);
				air = .1f;
			}
		}
	}
	else if (c.kind == PFS::WALL)
	{
		// A wall, a fence: a stretch of fire front standing along it, just in front of its face - the ground's own takes,
		// wide enough to run into their neighbours. (The take baked in a doorway, one for every cell, made a burning
		// fence a row of equal single fires.) Every one stands a little off its cell's place and at its own height;
		// near the ground it stands on the ground.
		// wallfire: what is not wood shows no fire of its own up the wall - with no floor within reach it hung in the air
		// on a soft bottom. Its column's fire stands on the ground at the wall's foot (startFootPicture).
		if (c.fuel != PFS::WOOD && ps_painter_fire_fence_foot) return 0;
		sh.form = 2;
        if(c.fuel==PFS::WOOD)name="body";
		normal.set(c.n.x, 0.f, c.n.z);
		const float along = (float((hash >> 4) & 1023u) / 1023.f - .5f) * .7f * S;
		pos.x += -c.n.z * along + c.n.x * .32f;
		pos.z += c.n.x * along + c.n.z * .32f;
		pos.y -= (.35f + .3f * float((hash >> 14) & 255u) / 255.f) * S;
		Fvector above = pos;
		above.y += .4f;
        float gy=0.f;
        if(firePictureFloor(above,.4f,1.4f,gy) && gy>pos.y-1.f && gy<pos.y+.8f)
			pos.y = gy;
		else
			air = .3f; // high on the wall: it does not stand on anything
		// In a room its flames end at the ceiling: they do not stand up through it into the storey above, or out of the
		// ground over a cellar. (The ceiling is looked for a step out from the wall, to either side: in a corner one of the
		// two places lies in the neighbouring wall. Under an eave the flames stand free, as before.)
		for (int side = -1; side <= 1; side += 2)
		{
			const Fvector q = {c.p.x + c.n.x * .7f - c.n.z * .4f * float(side), pos.y, c.p.z + c.n.z * .7f + c.n.x * .4f * float(side)};
			float h = 0.f;
			Fvector hn;
			if (roofedAt(q, &h, &hn) && (top == 0.f || h > top))
			{
				top = h;
				capN = hn;
				capAt = q;
			}
		}
	}
	else if (c.kind == PFS::CEILING)
	{
		// Under a ceiling, an eave: the flames lick up to it and end there - they do not stand up through the roof.
		pos.y -= .9f;
		cap = c.p.y;
		top = c.p.y; // (nothing of it above the ceiling: its soot showed there)
		capN.set(c.n.x, c.n.y, c.n.z);
		if (capN.y > 0.f) capN.invert();
		air = .3f;
		if (cls > 1) name = names[1];
	}
	else
	{
		// The ground: never on the lattice's own points - a row of burning cells must not read as a row of fires.
		// And a fire front is one line of flame: every fire stands along the front (its take is a stretch of fire front),
		// much off its place along that line and only a little across it. (Each turned its own random way, a spreading
		// fire was a scatter of short walls of flame: patchy.)
		// In a room its flames end at the ceiling: they do not stand up through it into the storey above, or out of the
		// ground over a cellar.
		top = roomTop(c);
		if (top != 0.f)
		{
			float h = 0.f;
			Fvector hn;
			if (roofedAt(capAt, &h, &hn) && _abs(h - top) < .6f) capN = hn;
		}
		const float j1 = float((hash >> 4) & 1023u) / 1023.f - .5f, j2 = float((hash >> 14) & 1023u) / 1023.f - .5f;
		Fvector along;
		if (frontAt(c, along))
		{
			yaw = atan2f(along.z, along.x);
			pos.x += along.x * j1 * .8f * S - along.z * j2 * .3f * S;
			pos.z += along.z * j1 * .8f * S + along.x * j2 * .3f * S;
		}
		else
		{
			pos.x += j1 * .8f * S;
			pos.z += j2 * .8f * S;
		}
		Fvector above = pos;
		above.y += .6f;
        float gy=0.f;
        if(firePictureFloor(above,.29f,1.5f,gy) && _abs(gy-c.p.y)<.9f)pos.y=gy;
        else {pos.set(c.p.x,c.p.y,c.p.z);above=pos;above.y+=.6f;}
		// in the wind the flames lean with it: the takes baked with wind lean along their own width
		if (!c.roofed && wx.windX * wx.windX + wx.windZ * wx.windZ > 2.5f * 2.5f)
		{
			name = windy[cls];
			mirror = false;
			yaw = atan2f(wx.windZ, wx.windX);
		}
		// On a slope or a roof it stands on the lowest ground under its width: no part of it hangs in the air (what
		// sinks into the ground is hidden by the ground). At an edge - a roof's edge, a ledge - it is moved back from the
		// drop and drawn smaller instead of hanging over it.
		const float reach = .45f + .5f * c.size;
		int drops = 0, dropSide = 0;
		for (int side = -1; side <= 1; side += 2)
		{
			Fvector q = above;
			q.x += _cos(yaw) * reach * float(side);
			q.z += _sin(yaw) * reach * float(side);
            float g2=0.f;
            if(!firePictureFloor(q,.59f,1.8f,g2) || g2<c.p.y-1.2f)
			{
				// nothing under that side, or far below: a drop
				++drops;
				dropSide = side;
			}
			else if (_abs(g2 - c.p.y) < 1.2f && g2 < pos.y)
				pos.y = g2;
		}
		if (drops == 1)
		{
			pos.x -= _cos(yaw) * reach * .6f * float(dropSide);
			pos.z -= _sin(yaw) * reach * .6f * float(dropSide);
            float stayed=0.f;
            if(firePictureFloor(pos,.89f,1.2f,stayed))pos.y=_min(pos.y,stayed);
            else pos.set(c.p.x,c.p.y,c.p.z);
			sh.mult = _min(sh.mult, 1.f);
			air = .2f;
		}
		else if (drops == 2)
		{
			sh.mult = _min(sh.mult, .8f);
			air = .2f;
		}
		if (ps_painter_fire_fence_foot)
		{
			int across = 0;
			if (drops < 2)
				for (int side = -1; side <= 1; side += 2)
				{
					Fvector q = {c.p.x - _sin(yaw) * .42f * float(side), c.p.y + .6f, c.p.z + _cos(yaw) * .42f * float(side)};
					float g2 = 0.f;
					if (!firePictureFloor(q, .59f, 1.8f, g2) || g2 < c.p.y - .55f) ++across;
				}
			if (drops == 2 || across == 2)
			{
				name = "spot";
				pos.set(c.p.x, c.p.y, c.p.z);
				sh.mult = _min(sh.mult, .8f);
				air = 0.f;
			}
		}
	}
	int take = findTake(name);
	if (take < 0)
	{
        if(c.kind!=PFS::GROUND)return 0;
		sh.form = 0;
		normal.set(0.f, 0.f, 0.f);
		take = findTake(names[cls]);
	}
	if (take < 0) take = findTake("grass_mid");
	Fire* f = add(take, pos,c.kind==PFS::WALL && c.fuel==PFS::WOOD);
	if (!f) return 0;
	f->yaw = yaw;
	f->trunkReach = trunkReach;
    if(c.kind==PFS::WALL && c.fuel==PFS::WOOD)
    {
        woodFlameContact(c,*f,.5f*S);
        if(f->wrapFace.valid()){
            sh.form=4;
            pos=f->pos;
        }
    }
	// the ceiling's height where the fire stands (it was found a little to the side)
	if (capN.y < -.3f)
	{
		const float shift = -(capN.x * (pos.x - capAt.x) + capN.z * (pos.z - capAt.z)) / capN.y;
		if (top != 0.f) top += shift;
		if (cap != 0.f) cap += shift;
	}
	f->cap = cap;
	f->air = air;
	f->top = top;
	f->capN = capN;
	f->id = nextFireId++;
	if (!nextFireId) nextFireId = 1;
	f->cell = index + 1;
	f->life = 1e6f;
	f->fadeIn = fadeIn;
	f->fadeOut = 2.f;
	f->phase = float((index * 7919u) % 977u);
	f->mirror = mirror;
	f->scale = pictureScale(sh, c) * pictureGrowth(sh, c); // firerework: a catching cell's flame starts small
	if (normal.square_magnitude() > .5f) f->normal = normal;
	// The burning material determines smoke colour.
	static const float smoke[PFS::FUEL_COUNT] = {0.f, .9f, 1.f, 1.3f, 1.f, 1.5f};
	static const float albedo[PFS::FUEL_COUNT] = {.5f, .7f, .55f, .4f, .35f, .22f};
	f->smoke = takes[take].smoke ? smoke[c.fuel] : 0.f;
	f->albedo = albedo[c.fuel];
	return f->id;
}
bool wantsFoot(const PFS::Cell& c)
{
	// wallfire: every flat wall that burns (not a trunk, not a bush or a crown)
	if (!ps_painter_fire_fence_foot || c.kind != PFS::WALL || c.trunk || c.fuel == PFS::BRUSH || c.fuel == PFS::NOTHING) return false;
	const uint32_t below = field.wallAt(c.kx, c.ky - 1, c.kz, c.n, &c);
	if (below == PFS::Field::NO_CELL) return true;
	const PFS::Cell& b = field.cells[below];
	return !(b.state == PFS::BURNING || b.state == PFS::CATCHING);
}
u32 startFootPicture(u32 index, const PFS::Cell& c, float fadeIn)
{
	// wallfire: the mid take, not the tallest at 1.3 x (that stood a storey high)
	const int take = findTake("grass_mid");
	if (take < 0) return 0;
	const u32 hash = index * 2654435761u;
	const float S = field.cellSize;
	// as a wall's own fire stood (form 2): a little off the cell's place along the wall, just in front of its face ...
	Fvector pos = {c.p.x, c.p.y, c.p.z};
	const float along = (float((hash >> 4) & 1023u) / 1023.f - .5f) * .7f * S;
	pos.x += -c.n.z * along + c.n.x * .32f;
	pos.z += c.n.x * along + c.n.z * .32f;
	// ... and on the ground under it, however high on the fence the cell is. No ground within reach (a balcony's rail, a
	// roof's edge): no foot fire.
	Fvector above = pos;
	above.y += .4f;
	float gy = 0.f;
	if (!firePictureFloor(above, .4f, 4.5f, gy) || gy > pos.y + .8f) return 0;
	pos.y = gy;
	// in a room its flames end at the ceiling, as a wall's fire does
	float top = 0.f;
	Fvector capN = {0.f, 0.f, 0.f}, capAt = pos;
	for (int side = -1; side <= 1; side += 2)
	{
		const Fvector q = {c.p.x + c.n.x * .7f - c.n.z * .4f * float(side), pos.y, c.p.z + c.n.z * .7f + c.n.x * .4f * float(side)};
		float h = 0.f;
		Fvector hn;
		if (roofedAt(q, &h, &hn) && (top == 0.f || h > top))
		{
			top = h;
			capN = hn;
			capAt = q;
		}
	}
	if (capN.y < -.3f && top != 0.f)
		top += -(capN.x * (pos.x - capAt.x) + capN.z * (pos.z - capAt.z)) / capN.y;
	Fire* f = add(take, pos);
	if (!f) return 0;
	f->footPicture = true;
	f->yaw = float(hash >> 20) * (PI_MUL_2 / 4096.f);
	f->normal.set(c.n.x, 0.f, c.n.z);
	f->cap = 0.f;
	f->air = 0.f;
	f->top = top;
	f->capN = capN;
	f->id = nextFireId++;
	if (!nextFireId) nextFireId = 1;
	f->cell = index + 1;
	f->life = 1e6f;
	f->fadeIn = fadeIn;
	f->fadeOut = 2.f;
	f->phase = float((index * 7919u + 311u) % 977u);
	f->mirror = (hash & 0x20000u) != 0;
	// wallfire: every column its own size - visible flames about 1.4 - 1.9 m (the take's box is 3.2 m high, its flames fill
	// about 0.58 of it), growing with the cell's own flames
	f->footBase = .75f + .27f * float((hash >> 12) & 255u) / 255.f;
	f->scale = f->footBase * (.55f + .45f * _min(c.size, 1.f));
	static const float smoke[PFS::FUEL_COUNT] = {0.f, .9f, 1.f, 1.3f, 1.f, 1.5f};
	static const float albedo[PFS::FUEL_COUNT] = {.5f, .7f, .55f, .4f, .35f, .22f};
	f->smoke = takes[take].smoke ? smoke[c.fuel] : 0.f;
	f->albedo = albedo[c.fuel];
	return f->id;
}
// once a frame for the picture of a cell: it follows the cell's flames
void spreadPicture(Fire& f)
{
	const u32 index = f.cell - 1;
	if (index >= field.cells.size() || index >= shown.size())
	{
		f.cell = 0;
		return;
	}
	const PFS::Cell& c = field.cells[index];
	float size = c.size;
	if (c.kind == PFS::GROUND)
	{
		// a fire front is one wall of flame, not a row of fires each of its own height: every fire is drawn about as
		// big as the fires beside it
		static const int nb[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
		if (f.pictureFieldCount != u32(-1) && f.pictureFieldCount != field.cells.size())
		{
			const float S = field.cellSize;
			bool exactFloor = true;
			for (int k = 0; k < 4; ++k)
			{
				const u32 j = field.groundAt(float(c.kx + nb[k][0]) * S, c.p.y, float(c.kz + nb[k][1]) * S);
				f.pictureNeighbours[k] = j;
				if (j == PFS::Field::NO_CELL || field.cells[j].ky != c.ky) exactFloor = false;
			}
			// A lower or upper floor may be superseded by a newly discovered cell on this storey.
			f.pictureFieldCount = exactFloor ? u32(-1) : u32(field.cells.size());
		}
		float sum = 0.f, count = 0.f;
		for (int k = 0; k < 4; ++k)
		{
			const u32 j = f.pictureNeighbours[k];
			if (j == PFS::Field::NO_CELL) continue;
			const PFS::Cell& n = field.cells[j];
			if (n.state != PFS::BURNING && n.state != PFS::CATCHING && n.state != PFS::DYING) continue; // firerework: dying ones too
			sum += n.size;
			count += 1.f;
		}
		if (count > 0.f) size = .55f * size + .45f * sum / count;
	}
	else if (c.kind == PFS::WALL && !f.wrapFace.valid() && !c.trunk && c.fuel != PFS::BRUSH)
	{
		// ... and a wall's fire is bigger for every burning cell above it: it stands for them too
		if (f.pictureFieldCount != u32(-1) && f.pictureFieldCount != field.cells.size())
		{
			for (int up = 1; up <= 2; ++up)
				f.pictureNeighbours[up - 1] = field.wallAt(c.kx, c.ky + up, c.kz, c.n,&c);
			f.pictureFieldCount = f.pictureNeighbours[0] != PFS::Field::NO_CELL && f.pictureNeighbours[1] != PFS::Field::NO_CELL
				? u32(-1) : u32(field.cells.size());
		}
		for (int up = 1; up <= 2; ++up)
		{
			const u32 j = f.pictureNeighbours[up - 1];
			if (j == PFS::Field::NO_CELL) break;
			const PFS::State st = field.cells[j].state;
			if (st != PFS::BURNING && st != PFS::CATCHING) break;
			size += .3f;
		}
		size = _min(size, 1.25f);
	}
	float target = pictureScaleOf(shown[index], size) * trunkThin(shown[index], c);
	if (f.footPicture)
	{
		// fencefoot: sized like a wall's fire (the ground's large take), not like the boards' own flames
		target = f.footBase * (.55f + .45f * _min(size, 1.f)); // wallfire
	}
	target *= pictureGrowth(shown[index], c); // firerework: grows from small after it catches, shrinks away while it dies
	f.scale += (target - f.scale) * _min(1.f, Device.fTimeDelta * 2.f);
}

// Fire lights scale with the size of the flame group.
struct SpreadLight
{
	int bx = 0, bz = 0;
	bool used = false;
	Fvector pos = {0.f, 0.f, 0.f}, want = {0.f, 0.f, 0.f};
	float amount = 0.f, wantAmount = 0.f, tall = 1.f;
	float lid = 0.f; // a fire in a room: how high its ceiling lies (0 = something of it burns in the open)
	float phase = 0.f;
	float extent = 0.f; // firetrack: how far its fire spreads from the light (m)
	float extentWant = 0.f; // firelightsmooth: the spread the plan saw; extent follows it smoothly
	u32 id = 0;         // firetrack: the fire this light belongs to
	u32 plume = 0; // the smoke column over it (a fire's number), 0 = none
	// firesides: where its sound comes from - the burning cell nearest the camera (heardWant), followed at 16 m/s at most
	Fvector heard = {0.f, 0.f, 0.f}, heardWant = {0.f, 0.f, 0.f};
};
SpreadLight spreadLights[32]; // firetrack: was 8 (firelightsmall), 6
PainterFireLighting::Plan fireLightPlan;
ref_light fireLamps[PainterFireLighting::slots][2];
PainterFireLighting::Presentation firePresentation;
void publishFireLights()
{
    if (!ps_painter_fire || !g_pGameLevel) firePresentation=PainterFireLighting::Presentation{};
    else firePresentation.update(fireLightPlan,ps_painter_fire_light_quality,Device.fTimeDelta);
    for (unsigned i=0;i<PainterFireLighting::slots;i++)
    {
        const auto& state=firePresentation.lamps[i];
        const float mix=state.shadow;
        const bool active=state.occupied;
        for(unsigned shadow=0;shadow<2;shadow++)
        {
            ref_light& lamp=fireLamps[i][shadow];
            const float share=shadow?mix:1.f-mix;
            if(!active || share<=.001f)
            {
                if(lamp)lamp->set_active(false);
                continue;
            }
            if(!lamp)
            {
                lamp=RImplementation.light_create();
                lamp->set_type(IRender_Light::POINT);
                static_cast<light*>(lamp.operator->())->flags.bFireSource=true;
                lamp->set_shadow(shadow != 0);
            }
            static_cast<light*>(lamp.operator->())->flags.bFireDetailed=ps_painter_fire_light_quality>=2;
            const auto& site=state.source;
            Fvector p={site.p.x,site.p.y,site.p.z};
            const float weight=share>=1.f ? 1.f : std::sqrt(share);
            // firelightreach: the quality setting is also how far and how strongly the fire lights (High, Beautiful)
            const float qualityReach=ps_painter_fire_light_quality>=3?1.8f:ps_painter_fire_light_quality==2?1.45f:ps_painter_fire_light_quality==1?1.2f:1.f;
            // firelightfill: with shadows on (High and up) a lamp without a shadow is only its fire's own glow; the shadowed
            // lamps do the real lighting at the old strength (no quality boost any more - it blew the scene out)
            const bool glowOnly=!shadow && ps_painter_fire_light_quality>=1;
            const float qualityEnergy=glowOnly?.4f:1.f;
            const float energy=state.intensity*state.opacity*ps_painter_fire_brightness*weight*qualityEnergy;
            lamp->set_position(p);
            // firefog: the fire menu's Fire light reach scales every fire light (still at most 45 m)
            lamp->set_range(glowOnly?_min(site.range*.5f*ps_painter_fire_light_reach,14.f):_min(site.range*qualityReach*ps_painter_fire_light_reach,45.f));
            lamp->set_color(2.24f*energy,1.22f*energy,.48f*energy); // firelightwarm: the old orange (was near white 1.52 / .98)
            // firebeams (2026-11-23): soft shafts where this fire's light gets out past something - a window, a gap in a
            // fence - seen in the air: faint in clear air, plain in fog or the fire's own smoke; a few metres long, never
            // sun-ray length. Only the shadowed lamp: the shafts are the light its shadow lets out.
            {
                light* fl=static_cast<light*>(lamp.operator->());
                fl->fire_beam=0.f;
                if(shadow && ps_painter_fire_light_beams)
                {
                    float air=airAt(p); // firerework: the smoky air at the lamp, not one number at the camera
                    if(g_pGamePersistent && g_pGamePersistent->Environment().CurrentEnv)
                        air+=1.6f*g_pGamePersistent->Environment().CurrentEnv->fog_density;
                    air=clampr(air,0.f,1.f);
                    const float reachM=4.f+3.f*air; // 4 m in clear air .. 7 m in thick fog
                    fl->fire_beam_reach=clampr(reachM/_max(site.range*qualityReach,1.f),.12f,1.f);
                    // the shaft pass multiplies by the reach share again: divided out, so a short reach is not a dim one
                    fl->fire_beam=(.025f+.085f*air)*share/fl->fire_beam_reach;
                }
            }
            lamp->set_active(true);
        }
    }
}


float lightsClock = 0.f;

// firetrack (2026-11-23): one light per fire - it starts small where the fire starts, grows with it, moves with its front, and
// joins the light of a fire it merges with; a fire too wide for one light gets a 2nd / 3rd along its spread.
void spreadLightsPlan()
{
	struct Part { float sum = 0.f, x = 0.f, y = 0.f, z = 0.f, tall = 0.f, lid = 0.f, extent = 0.f; bool room = true;
		float nx = 0.f, ny = 0.f, nz = 0.f, nd = 3.0e38f; }; // firesides: its burning cell nearest the camera
	const Fvector heardFrom = Device.vCameraPosition;
	static xr_vector<u32> at;   // the burning cells
	static xr_vector<u32> up;   // which fire: union-find over them
	static xr_vector<Part> parts;
	static std::unordered_map<u64, xr_vector<u32>> grid;
	static xr_vector<std::pair<u32, u32>> byFire;
	static xr_vector<u32> members;
	static xr_vector<u8> side;
	at.clear(); parts.clear(); grid.clear(); byFire.clear();
	for (u32 i : field.burningCells()) if (field.cells[i].size > 0.f) at.push_back(i);
	const u32 n = u32(at.size());
	up.resize(n);
	for (u32 i = 0; i < n; ++i) up[i] = i;
	auto root = [&](u32 a) { while (up[a] != a) { up[a] = up[up[a]]; a = up[a]; } return a; };
	auto half = [](int k) { return k >= 0 ? k / 2 : -((1 - k) / 2); };
	auto key = [](int x, int z) { return (u64(u32(x)) << 32) | u64(u32(z)); };
	for (u32 i = 0; i < n; ++i) { const PFS::Cell& c = field.cells[at[i]]; grid[key(half(c.kx), half(c.kz))].push_back(i); }
	// cells within two cells of each other (3.2 m) burn as one fire
	for (u32 i = 0; i < n; ++i)
	{
		const PFS::Cell& a = field.cells[at[i]];
		const int gx = half(a.kx), gz = half(a.kz);
		for (int dx = -1; dx <= 1; ++dx)
			for (int dz = -1; dz <= 1; ++dz)
			{
				const auto it = grid.find(key(gx + dx, gz + dz));
				if (it == grid.end()) continue;
				for (u32 j : it->second)
				{
					if (j <= i) continue;
					const PFS::Cell& b = field.cells[at[j]];
					if (std::abs(a.kx - b.kx) <= 2 && std::abs(a.kz - b.kz) <= 2 && std::abs(a.ky - b.ky) <= 2)
					{
						const u32 ra = root(i), rb = root(j);
						if (ra != rb) up[rb] = ra;
					}
				}
			}
	}
	for (u32 i = 0; i < n; ++i) byFire.push_back({root(i), i});
	std::sort(byFire.begin(), byFire.end());
	// the advancing front pulls the light: catching and freshly lit cells weigh more
	auto weight = [](const PFS::Cell& c) { return c.size * ((c.state == PFS::CATCHING || c.t < 4.f) ? 1.6f : 1.f); };
	for (u32 s = 0; s < n;)
	{
		u32 e = s;
		while (e < n && byFire[e].first == byFire[s].first) ++e;
		members.clear();
		for (u32 k = s; k < e; ++k) members.push_back(at[byFire[k].second]);
		s = e;
		float W = 0.f;
		Fvector m = {0.f, 0.f, 0.f};
		for (u32 ci : members)
		{
			const PFS::Cell& c = field.cells[ci];
			const float w = weight(c);
			W += w; m.x += c.p.x * w; m.y += c.p.y * w; m.z += c.p.z * w;
		}
		if (W <= 0.f) continue;
		m.set(m.x / W, m.y / W, m.z / W);
		float r2 = 0.f;
		for (u32 ci : members)
		{
			const PFS::Cell& c = field.cells[ci];
			const float dx = c.p.x - m.x, dz = c.p.z - m.z;
			r2 += weight(c) * (dx * dx + dz * dz);
		}
		const float rms = _sqrt(r2 / W);
		// a fire too wide for one light: 2 or 3 lights along it (each cell to the nearest of k centres, three rounds)
		const u32 k = rms > 11.f ? u32(_min(3.f, 1.f + std::floor(rms / 11.f))) : 1u;
		Fvector cen[3] = {m, m, m};
		side.assign(members.size(), u8(0));
		if (k > 1)
		{
			auto farFrom = [&](const Fvector* from, u32 cnt) {
				float best = -1.f;
				Fvector bp = m;
				for (u32 ci : members)
				{
					const PFS::Cell& c = field.cells[ci];
					float d = 3.0e38f;
					for (u32 q = 0; q < cnt; ++q)
					{
						const float dx = c.p.x - from[q].x, dz = c.p.z - from[q].z;
						d = _min(d, dx * dx + dz * dz);
					}
					if (d > best) { best = d; bp.set(c.p.x, c.p.y, c.p.z); }
				}
				return bp;
			};
			cen[0] = farFrom(&m, 1);
			for (u32 q = 1; q < k; ++q) cen[q] = farFrom(cen, q);
			for (int round = 0; round < 3; ++round)
			{
				float ax[3] = {0.f, 0.f, 0.f}, ay[3] = {0.f, 0.f, 0.f}, az[3] = {0.f, 0.f, 0.f}, aw[3] = {0.f, 0.f, 0.f};
				for (u32 x = 0; x < u32(members.size()); ++x)
				{
					const PFS::Cell& c = field.cells[members[x]];
					u32 best = 0;
					float bd = 3.0e38f;
					for (u32 q = 0; q < k; ++q)
					{
						const float dx = c.p.x - cen[q].x, dz = c.p.z - cen[q].z, d = dx * dx + dz * dz;
						if (d < bd) { bd = d; best = q; }
					}
					side[x] = u8(best);
					const float w = weight(c);
					aw[best] += w; ax[best] += c.p.x * w; ay[best] += c.p.y * w; az[best] += c.p.z * w;
				}
				for (u32 q = 0; q < k; ++q)
					if (aw[q] > 0.f) cen[q].set(ax[q] / aw[q], ay[q] / aw[q], az[q] / aw[q]);
			}
		}
		for (u32 q = 0; q < k; ++q)
		{
			Part p;
			float pw = 0.f, pr2 = 0.f;
			for (u32 x = 0; x < u32(members.size()); ++x)
			{
				if (side[x] != q) continue;
				const PFS::Cell& c = field.cells[members[x]];
				const float w = weight(c);
				p.sum += c.size; pw += w;
				{
					const float ex = c.p.x - heardFrom.x, ey = c.p.y - heardFrom.y, ez = c.p.z - heardFrom.z;
					const float ed = ex * ex + ey * ey + ez * ez;
					if (ed < p.nd) { p.nd = ed; p.nx = c.p.x; p.ny = c.p.y; p.nz = c.p.z; }
				}
				p.x += c.p.x * w; p.y += c.p.y * w; p.z += c.p.z * w;
				p.tall = _max(p.tall, c.size);
				// (a fire where all that burns on the ground or overhead has something over it is a fire in a room)
				if (c.kind != PFS::WALL)
				{
					if (c.lid == 0.f) p.room = false;
					else if (p.lid == 0.f || c.lid > p.lid) p.lid = c.lid;
				}
			}
			if (pw <= 0.f) continue;
			p.x /= pw; p.y /= pw; p.z /= pw;
			for (u32 x = 0; x < u32(members.size()); ++x)
			{
				if (side[x] != q) continue;
				const PFS::Cell& c = field.cells[members[x]];
				const float dx = c.p.x - p.x, dz = c.p.z - p.z;
				pr2 += weight(c) * (dx * dx + dz * dz);
			}
			p.extent = _sqrt(pr2 / pw);
			parts.push_back(p);
		}
	}
	// far off, fires close to each other share one light (one glow over a burning area): within a quarter of their distance
	const Fvector cam = Device.vCameraPosition;
	auto camDist = [&](const Part& p) { const float dx = p.x - cam.x, dz = p.z - cam.z; return _sqrt(dx * dx + dz * dz); };
	for (u32 a = 0; a < u32(parts.size()); ++a)
		for (u32 b = a + 1; b < u32(parts.size());)
		{
			const float closer = _min(camDist(parts[a]), camDist(parts[b]));
			const float dx = parts[a].x - parts[b].x, dz = parts[a].z - parts[b].z, d2 = dx * dx + dz * dz;
			if (closer > 40.f && d2 < (.25f * closer) * (.25f * closer))
			{
				Part& A = parts[a];
				const Part& B = parts[b];
				const float w = A.sum + B.sum, sep = _sqrt(d2);
				A.x = (A.x * A.sum + B.x * B.sum) / w; A.y = (A.y * A.sum + B.y * B.sum) / w; A.z = (A.z * A.sum + B.z * B.sum) / w;
				A.extent = _max(_max(A.extent, B.extent), .5f * (sep + A.extent + B.extent));
				if (B.nd < A.nd) { A.nd = B.nd; A.nx = B.nx; A.ny = B.ny; A.nz = B.nz; }
				A.sum = w; A.tall = _max(A.tall, B.tall); A.room = A.room && B.room; A.lid = _max(A.lid, B.lid);
				parts[b] = parts.back();
				parts.pop_back();
			}
			else ++b;
		}
	// the fires that matter most first: big and near (a small fire beside you before a big one far off)
	std::sort(parts.begin(), parts.end(), [&](const Part& a, const Part& b) {
		const float da = camDist(a), db = camDist(b);
		return a.sum / (1.f + da * da / 900.f) > b.sum / (1.f + db * db / 900.f); });
	constexpr u32 lightCount = u32(sizeof(spreadLights) / sizeof(spreadLights[0]));
	if (parts.size() > lightCount) parts.resize(lightCount);
	// every light keeps its fire: each fire (biggest first) takes the nearest light that was on it or close to it
	bool claimed[lightCount] = {};
	for (SpreadLight& l : spreadLights) l.wantAmount = 0.f;
	static u32 nextId = 0;
	for (const Part& p : parts)
	{
		SpreadLight* slot = nullptr;
		float best = 3.0e38f;
		for (u32 q = 0; q < lightCount; ++q)
		{
			SpreadLight& l = spreadLights[q];
			if (!l.used || claimed[q]) continue;
			const float dx = l.want.x - p.x, dz = l.want.z - p.z, d = dx * dx + dz * dz;
			const float reach = l.extent + p.extent + 6.f;
			if (d < reach * reach && d < best) { best = d; slot = &l; }
		}
		if (!slot)
			for (SpreadLight& l : spreadLights)
				if (!l.used)
				{
					// a new fire: its light starts small where it burns and grows with it
					slot = &l;
					slot->used = true;
					slot->amount = 0.f;
					slot->pos.set(p.x, p.y, p.z);
					slot->heard.set(p.nx, p.ny, p.nz);
					slot->id = ++nextId;
					slot->bx = int(slot->id);
					slot->bz = 0;
					slot->phase = float((slot->id * 2654435761u) % 628u) * .01f;
					break;
				}
		if (!slot) continue;
		claimed[u32(slot - spreadLights)] = true;
		slot->want.set(p.x, p.y, p.z);
		slot->heardWant.set(p.nx, p.ny, p.nz);
		slot->wantAmount = p.sum;
		slot->tall = p.tall;
		slot->extentWant = p.extent; // firelightsmooth
		slot->lid = p.room ? p.lid : 0.f;
	}
	// a light whose fire joined a bigger one moves into it while it fades: the two lights become one
	for (u32 q = 0; q < lightCount; ++q)
	{
		SpreadLight& l = spreadLights[q];
		if (!l.used || claimed[q]) continue;
		float best = 3.0e38f;
		const SpreadLight* into = nullptr;
		for (u32 o = 0; o < lightCount; ++o)
		{
			if (!claimed[o]) continue;
			const SpreadLight& b = spreadLights[o];
			const float dx = b.want.x - l.want.x, dz = b.want.z - l.want.z, d = dx * dx + dz * dz;
			const float reach = b.extent + l.extent + 6.f;
			if (d < reach * reach && d < best) { best = d; into = &b; }
		}
		if (into) l.want = into->want;
	}
}
void spreadLightsFrame(float dt)
{
	lightsClock += dt;
	if (lightsClock >= .15f)
	{
		lightsClock = 0.f;
		spreadLightsPlan();
	}
	const float t = Device.fTimeGlobal;
	for (SpreadLight& l : spreadLights)
	{
		if (!l.used) continue;
		// firelightsmooth: the light grows and shrinks smoothly with its fire - no steps as cells catch
		l.amount += (l.wantAmount - l.amount) * _min(1.f, dt * (l.wantAmount > l.amount ? 1.2f : .8f));
		l.extent += (l.extentWant - l.extent) * _min(1.f, dt * 1.f);
		l.pos.lerp(l.pos, l.want, _min(1.f, dt * 2.f));
		{
			// firesides: the sound place goes to the burning cell nearest the camera, 16 m/s at most (no jumps)
			Fvector go;
			go.sub(l.heardWant, l.heard);
			const float left = go.magnitude(), step = 16.f * dt;
			if (left <= step) l.heard = l.heardWant;
			else l.heard.mad(l.heard, go, step / left);
		}
		if (l.wantAmount <= 0.f && l.amount < .03f)
		{
			l.used = false;
			continue;
		}
		if (!ps_painter_fire) continue;
		const float root = _sqrt(_max(l.amount, 0.f));
		const float flicker = .86f + .14f * _sin(t * 11.f + l.phase) * _sin(t * 7.3f + 2.f * l.phase);
		Fvector p = l.pos;
		p.y += .45f + .9f * l.tall + .15f * _min(root, 3.f); // firelightwarm: from flame level, not from inside nearby leaves
		// in a room it stays under the ceiling: from above it, it lit the floor of the storey above, the ground over a cellar
		if (l.lid != 0.f && p.y > l.lid - .35f) p.y = l.lid - .35f;
        auto s=PainterFireLighting::spread({p.x,p.y,p.z},l.amount,flicker,0);
        // firetrack: one key per fire (its lamp and shadow stay with it); the light reaches over the whole fire
        s.key=(u64(0x46495245u)<<32) | u64(l.id); s.range=_min(45.f,_max(s.range,l.extent*1.25f+5.f));
        s.area=true;fireLightPlan.add(s);
	}
}

// ---------------------------------------------------------------- smoke over a big fire
// Smoke columns: over every strong part of a big fire stands a tall column of smoke - the take "plume", a baked simulation
// like the flames, drawn big and turned into the wind.
void plumesUpdate()
{
	const int take = findTake("plume");
	if (take < 0) return;
	float top = 0.f;
	for (const SpreadLight& l : spreadLights)
		if (l.used) top = _max(top, l.amount);
	const bool big = ps_painter_fire && field.burning >= 14 && top > 0.f;
	const float size = 1.5f + 1.3f * clampr(float(field.burning) / 70.f, 0.f, 1.f);
	const PFS::Weather wx = weather();
	for (SpreadLight& l : spreadLights)
	{
		Fire* f = l.plume ? fireOf(l.plume) : nullptr;
		if (!(big && l.used && l.amount >= .4f * top))
		{
			if (f) endPicture(l.plume, 12.f);
			l.plume = 0;
			continue;
		}
		Fvector foot = l.pos;
		foot.y = groundUnder(l.pos);
		if (!f)
		{
			f = add(take, foot);
			if (!f) continue;
			f->id = nextFireId++;
			if (!nextFireId) nextFireId = 1;
			l.plume = f->id;
			f->life = 1e6f;
			f->fadeIn = 6.f;
			f->fadeOut = 12.f;
			f->phase = float((u32(l.bx) * 131u + u32(l.bz) * 31u) % 977u);
			f->scale = size * .7f;
			f->smoke = 1.4f;
			f->albedo = .32f;
		}
		const float k = _min(1.f, Device.fTimeDelta * .5f);
		f->pos.x += (foot.x - f->pos.x) * k;
		f->pos.y += (foot.y - f->pos.y) * k;
		f->pos.z += (foot.z - f->pos.z) * k;
		f->scale += (size - f->scale) * _min(1.f, Device.fTimeDelta * .3f);
		if (wx.windX * wx.windX + wx.windZ * wx.windZ > .25f) f->yaw = atan2f(wx.windZ, wx.windX);
	}
}
namespace Air
{
const int N = 96;        // squares per side
const float CELL = 8.f;  // metres per square
int ox = 0, oz = 0;      // the map's low corner, in squares (world metres / CELL)
bool placed = false;
float dens[N * N];       // the smoke, 0 clear .. 1 as thick as it gets
float gy[N * N];         // the height of the ground the smoke lies on (m, world)
float hot[N * N];        // how much burns there right now (the fires' glow in the smoke)
float src[N * N], ysum[N * N], wide[N * N], work[N * N], moved[N * N];
float stepClock = 0.f, maxDens = 0.f;
bool upload = true;
}
float smokeAir = 0.f; // the smoke where the camera is, 0 clear .. 1
float airLayer(float d) { return 8.f + 22.f * d; } // how tall the smoke stands over its ground (m)
// a box blur of radius r over the rows, then the columns: every square gets the mean of its (2r+1)^2 neighbours (outside
// the map counts as nothing)
void airBlur(float* a, int r)
{
	using namespace Air;
	if (r <= 0) return;
	const float k = 1.f / float(2 * r + 1);
	for (int z = 0; z < N; ++z)
	{
		const float* row = a + z * N;
		float sum = 0.f;
		for (int x = 0; x <= r && x < N; ++x) sum += row[x];
		for (int x = 0; x < N; ++x)
		{
			work[z * N + x] = sum * k;
			if (x + r + 1 < N) sum += row[x + r + 1];
			if (x - r >= 0) sum -= row[x - r];
		}
	}
	for (int x = 0; x < N; ++x)
	{
		float sum = 0.f;
		for (int z = 0; z <= r && z < N; ++z) sum += work[z * N + x];
		for (int z = 0; z < N; ++z)
		{
			a[z * N + x] = sum * k;
			if (z + r + 1 < N) sum += work[(z + r + 1) * N + x];
			if (z - r >= 0) sum -= work[(z - r) * N + x];
		}
	}
}
void smokeAirUpdate()
{
	using namespace Air;
	if (!ps_painter_fire || !ps_painter_fire_fog)
	{
		if (placed)
		{
			memset(dens, 0, sizeof(dens));
			memset(hot, 0, sizeof(hot));
			placed = false;
			upload = true;
		}
		smokeAir = maxDens = 0.f;
		return;
	}
	stepClock += _min(Device.fTimeDelta, .1f);
	if (placed && stepClock < 1.f / 15.f) return; // 15 times a second
	const float step = _min(stepClock, .5f);
	stepClock = 0.f;
	const Fvector cam = Device.vCameraPosition;
	const int cx = iFloor(cam.x / CELL), cz = iFloor(cam.z / CELL);
	if (!placed)
	{
		ox = cx - N / 2;
		oz = cz - N / 2;
		placed = true;
		memset(dens, 0, sizeof(dens));
		memset(hot, 0, sizeof(hot));
		for (int j = 0; j < N * N; ++j) gy[j] = cam.y - 1.7f;
	}
	else if (cx - (ox + N / 2) > N / 6 || (ox + N / 2) - cx > N / 6 || cz - (oz + N / 2) > N / 6 || (oz + N / 2) - cz > N / 6)
	{
		// the camera keeps near the middle: the map moves by whole squares, what moves out of it is forgotten
		const int nx = cx - N / 2, nz = cz - N / 2;
		for (int z = 0; z < N; ++z)
			for (int x = 0; x < N; ++x)
			{
				const int sx = x + nx - ox, sz = z + nz - oz;
				const bool in = sx >= 0 && sz >= 0 && sx < N && sz < N;
				moved[z * N + x] = in ? dens[sz * N + sx] : 0.f;
				work[z * N + x] = in ? gy[sz * N + sx] : cam.y - 1.7f;
			}
		memcpy(dens, moved, sizeof(dens));
		memcpy(gy, work, sizeof(gy));
		ox = nx;
		oz = nz;
	}
	// what burns now puts smoke into its square: every burning cell of the spreading fire, every fire of its own (a crown, a
	// body, a fire bottle's flames, a wall fire) by the ground it covers - the cells' own pictures and the smoke columns are
	// their cells' (as smokeair counted them)
	memset(src, 0, sizeof(src));
	memset(ysum, 0, sizeof(ysum));
	auto put = [&](float x, float y, float z, float amount) {
		const int i = iFloor(x / CELL) - ox, k = iFloor(z / CELL) - oz;
		if (amount <= 0.f || i < 0 || k < 0 || i >= N || k >= N) return;
		src[k * N + i] += amount;
		ysum[k * N + i] += amount * y;
	};
	for (u32 i : field.burningCells())
	{
		const PFS::Cell& c = field.cells[i];
		put(c.p.x, c.p.y, c.p.z, .3f + c.size);
	}
	{
		const u32 now = Device.dwTimeGlobal;
		const int plume = findTake("plume");
		const float patch = 2.f * field.cellSize * field.cellSize;
		for (const Fire& f : fires)
		{
			if (f.cell || f.take < 0 || f.take >= int(takes.size()) || f.take == plume) continue;
			const Take& k = takes[f.take];
			float frame = 0.f, fade = 0.f;
			bool over = false;
			if (!state(f, k, now, frame, fade, over)) continue;
			put(f.pos.x, f.pos.y, f.pos.z, fade * _min(4.f, k.width * k.deep * f.scale * f.scale / patch));
		}
	}
	// the fires' glow lies in the smoke where they burn
	memcpy(hot, src, sizeof(hot));
	airBlur(hot, 2);
	// over the area: a fire counts fully within half of painter_fire_fog_area, less and less out to the full distance
	// (two box blurs, radius 3/4 and 1/4 of it); painter_fire_fog_fire burning patches give 95 % of the thickest smoke
	const float area = clampr(ps_painter_fire_fog_area, 50.f, 1000.f);
	int r1 = int(.75f * area / CELL + .5f), r2 = int(.25f * area / CELL + .5f);
	r1 = r1 < 2 ? 2 : r1 > N / 2 - 1 ? N / 2 - 1 : r1;
	r2 = r2 < 1 ? 1 : r2 > r1 - 1 ? r1 - 1 : r2;
	memcpy(wide, src, sizeof(wide));
	airBlur(wide, r1);
	airBlur(wide, r2);
	// the smoke's ground: the mean height of the fires its smoke comes from (where none reaches, it stays as it was)
	airBlur(ysum, r1);
	airBlur(ysum, r2);
	const float window = float((2 * r1 + 1) * (2 * r1 + 1));
	const float need = _max(1.f, ps_painter_fire_fog_fire);
	const float up = 3.f / _max(5.f, ps_painter_fire_fog_buildup), down = 3.f / _max(5.f, ps_painter_fire_fog_linger);
	for (int j = 0; j < N * N; ++j)
	{
		if (wide[j] > 1e-5f) gy[j] += (ysum[j] / wide[j] - gy[j]) * _min(1.f, step * .5f);
		const float want = 1.f - expf(-3.f * wide[j] * window / need);
		dens[j] += (want - dens[j]) * _min(1.f, step * (want > dens[j] ? up : down));
		if (dens[j] < .002f && want <= 0.f) dens[j] = 0.f;
	}
	// the wind carries it (at half its speed)
	const PFS::Weather wx = weather();
	const float mx = .5f * wx.windX * step / CELL, mz = .5f * wx.windZ * step / CELL;
	if (mx * mx + mz * mz > 1e-8f)
	{
		auto at = [&](int i, int k) { return (i < 0 || k < 0 || i >= N || k >= N) ? 0.f : dens[k * N + i]; };
		for (int z = 0; z < N; ++z)
			for (int x = 0; x < N; ++x)
			{
				const float fx = float(x) - mx, fz = float(z) - mz;
				const int x0 = iFloor(fx), z0 = iFloor(fz);
				const float tx = fx - float(x0), tz = fz - float(z0);
				moved[z * N + x] = (at(x0, z0) * (1.f - tx) + at(x0 + 1, z0) * tx) * (1.f - tz) +
					(at(x0, z0 + 1) * (1.f - tx) + at(x0 + 1, z0 + 1) * tx) * tz;
			}
		memcpy(dens, moved, sizeof(dens));
	}
	maxDens = 0.f;
	for (int j = 0; j < N * N; ++j) maxDens = _max(maxDens, dens[j]);
	smokeAir = airAt(cam);
	upload = true;
}
// the smoke at a place, 0 clear .. 1: the map's value there, all of it near its ground, thinning out to its layer's top,
// fading out over 3 m under its ground (painter_fire_air.h draws it the same way)
float airAt(const Fvector& p)
{
	using namespace Air;
	if (!placed || !ps_painter_fire || !ps_painter_fire_fog) return 0.f;
	const int i = iFloor(p.x / CELL) - ox, k = iFloor(p.z / CELL) - oz;
	if (i < 0 || k < 0 || i >= N || k >= N) return 0.f;
	const float d = dens[k * N + i], h = p.y - gy[k * N + i];
	return d * clampr(1.f - h / airLayer(d), 0.f, 1.f) * clampr(1.f + h / 3.f, 0.f, 1.f);
}
// ---------------------------------------------------------------- trees that burnt
// When a bush's or a tree's crown has burnt, that tree loses its leaves: its leaf parts (the level's tree parts drawn with
// an alpha-tested shader) are not drawn, in no pass, and its flat far picture is not used for it - the tree is drawn as it
// is, bare, from far too (r__dsgraph_build.cpp asks LeavesGone and LodGone). The frame's other threads read the set of
// such parts while it is made anew here: every set made stays until the level ends.
struct TreePart
{
	u32 firstTrunk=0, trunkCount=0;
	bool burnClaimed=false;
    bool hasOwner=false, separateWood=false;
	dxRender_Visual* v = nullptr;
	Fbox box;
};
xr_vector<TreePart> leafParts;          // the level's leaf parts (looked for the first time a crown burns)
std::unordered_map<u64, xr_vector<u32>> leafGrid; // leafParts by the 8 m square their middle lies in
xr_vector<dxRender_Visual*> farPictures; // the level's far pictures of trees and the like
xr_vector<Fbox> crownTrunks;
bool treesKnown = false;
PainterBurntCrownIndex bareIndex;
std::unordered_map<const void*,xr_vector<const void*>> treeLeafOwners,treeOwnerWood;
std::unordered_set<const void*> treeIndependentLeaves;
struct BareSet
{
    std::unordered_map<const void*,PainterTreeDates::Span> leaves,lods,wood;
    std::unordered_set<const void*> independentLeaves;
};
std::shared_ptr<const BareSet> bareNow;
struct BareTree
{
	dxRender_Visual* leaf = nullptr;
	float at = 0.f; // regrowth clock
	float started=0.f,ended=0.f;
};
xr_vector<BareTree> bareTrees;
xr_vector<Fvector> bareAsked;           // crowns that burnt, not looked at yet
float barePublishAt = 0.f;
bool bareDirty = false;
bool treeUsesUniform(dxRender_Visual* v,LPCSTR name)
{
    Shader* shader=v?v->shader._get():nullptr;
    if(!shader) return false;
    for(int element : {SE_R2_NORMAL_HQ,SE_R2_NORMAL_LQ}){
        ShaderElement* e=shader->E[element]._get();
        if(!e) continue;
        for(const auto& pass:e->passes)
            if(pass && pass->constants && pass->constants->get(name)) return true;
    }
    return false;
}
bool leafy(dxRender_Visual* v)
{
    if(!v || (v->Type!=MT_TREE_ST && v->Type!=MT_TREE_PM)) return false;
    if(treeUsesUniform(v,"painter_leaf_state")) return true;
    Shader* shader=v->shader._get();
    if(!shader) return false;
    ShaderElement* e=shader->E[SE_R2_SHADOW]._get();
    if(!e || e->passes.empty())return false;
    const auto& pass=e->passes[0];
    return pass && pass->ps && pass->ps->cName.size() && strstr(pass->ps->cName.c_str(),"aref");
}
bool treeSupportsCanopy(const Fbox& wood,const Fbox& leaf)
{
    // A small cap, a ground plane, or wood beside this canopy cannot replace its silhouette.
    const float height=leaf.max.y-leaf.min.y;
    if(height<=0.f || wood.max.y-wood.min.y < .35f*height) return false;
    const float overlap=_min(wood.max.y,leaf.max.y)-_max(wood.min.y,leaf.min.y);
    if(overlap<.2f*height) return false;
    const float cx=(wood.min.x+wood.max.x)*.5f, cz=(wood.min.z+wood.max.z)*.5f;
    return cx>=leaf.min.x && cx<=leaf.max.x && cz>=leaf.min.z && cz<=leaf.max.z;
}

void treesLearn()
{
	treesKnown = true;
	bareIndex.reset();
	treeLeafOwners.clear();treeOwnerWood.clear();treeIndependentLeaves.clear();
	leafParts.clear();
	farPictures.clear();
    leafGrid.clear();
    std::unordered_map<Shader*,int> materialRoles;
    auto role=[&](dxRender_Visual* v){
        if(!v || (v->Type!=MT_TREE_ST && v->Type!=MT_TREE_PM))return 0;
        Shader* shader=v->shader._get();
        const auto key=materialRoles.find(shader);
        if(key!=materialRoles.end())return key->second;
        const int found=leafy(v)?1:treeUsesUniform(v,"painter_wood_state")?2:0;
        materialRoles.emplace(shader,found);
        return found;
    };
    for (dxRender_Visual* v : RImplementation.Visuals)
	{
		if (!v) continue;
		if (v->Type == MT_LOD)
			farPictures.push_back(v);
		else if (role(v)==1)
		{
			TreePart t;
			t.v = v;
			t.box = v->vis.box;
			Fvector lc;
			t.box.getcenter(lc);
			leafGrid[(u64(u32(iFloor(lc.x / 8.f))) << 32) | u64(u32(iFloor(lc.z / 8.f)))].push_back(u32(leafParts.size()));
			leafParts.push_back(t);
		}
	}
	// LOD ownership links distinguish an off-centre crown from the tree beside it.
	crownTrunks.clear();
	std::unordered_map<const dxRender_Visual*,u32> leafByVisual;
	for (u32 i=0;i<leafParts.size();++i) leafByVisual.emplace(leafParts[i].v,i);
	xr_vector<u32> leaves;
    xr_vector<Fbox> verifiedWood;
	for (dxRender_Visual* lod : farPictures)
	{
		leaves.clear();
        verifiedWood.clear();
		const u32 first=u32(crownTrunks.size());
		auto collect=[&](dxRender_Visual* v) {
			if (!v || (v->Type!=MT_TREE_ST && v->Type!=MT_TREE_PM)) return;
			const auto leaf=leafByVisual.find(v);
			if (leaf==leafByVisual.end()) {
                crownTrunks.push_back(v->vis.box);
                treeOwnerWood[lod].push_back(v);
                if(role(v)==2) verifiedWood.push_back(v->vis.box);
            }
			else {leaves.push_back(leaf->second);treeLeafOwners[v].push_back(lod);}
		};
        // Walk this owner's hierarchy once. A nested LOD has its own entry and descendants.
        xr_vector<dxRender_Visual*> pending;
        std::unordered_set<const void*> visited;
        pending.push_back(lod);
        while(!pending.empty()){
            dxRender_Visual* v=pending.back(); pending.pop_back();
            if(!v || !visited.insert(v).second) continue;
            if(v==lod || v->Type==MT_HIERRARHY){
                for(auto child:static_cast<FHierrarhyVisual*>(v)->children)
                    pending.push_back(static_cast<dxRender_Visual*>(child));
            } else collect(v);
        }
		if (leaves.empty()) crownTrunks.resize(first);
		else for (u32 i:leaves) {
			TreePart& t=leafParts[i];
            bool supported=false;
            for(const auto& wood:verifiedWood){
                if(treeSupportsCanopy(wood,t.box)){ supported=true; break; }
            }
            t.separateWood=supported && (!t.hasOwner || t.separateWood);
            t.hasOwner=true;
			if (!t.trunkCount) {t.firstTrunk=first;t.trunkCount=u32(crownTrunks.size())-first;}
		}
	}
    for(const TreePart& t:leafParts) if(t.separateWood) treeIndependentLeaves.insert(t.v);
	Msg("[painter_fire] trees: %u leaf parts, %u far pictures in this level", u32(leafParts.size()), u32(farPictures.size()));
}
// Crown fires use the leaf geometry. Index leaf parts by the eight-metre square containing their centre.
u64 leafKey(int x, int z) { return (u64(u32(x)) << 32) | u64(u32(z)); }
// the crown (leaf part) a place lies in, or -1
int crownAt(const Fvector& p,bool skipLit=false)
{
	if (!treesKnown) treesLearn();
	const int gx = iFloor(p.x / 8.f), gz = iFloor(p.z / 8.f);
	for (int dz = -1; dz <= 1; ++dz)
		for (int dx = -1; dx <= 1; ++dx)
		{
			const auto found = leafGrid.find(leafKey(gx + dx, gz + dz));
			if (found == leafGrid.end()) continue;
			for (u32 i : found->second)
			{
				if(skipLit && leafParts[i].burnClaimed) continue;
				Fbox b = leafParts[i].box;
				b.grow(.6f);
				if (b.contains(p)) return int(i);
			}
		}
	return -1;
}
// a crown standing over a place (the place near its middle, the crown's bottom at most `high` over it), or -1
// A wood contact can hand off to a crown only when exactly one part contains it and its own crown ignited.
int claimedCrownAt(const Fvector& p)
{
    if(!treesKnown)return -1;
    int match=-1;
    const int gx=iFloor(p.x/8.f),gz=iFloor(p.z/8.f);
    for(int dz=-1;dz<=1;++dz)for(int dx=-1;dx<=1;++dx){
        const auto it=leafGrid.find(leafKey(gx+dx,gz+dz));
        if(it==leafGrid.end())continue;
        for(u32 i:it->second){
            const auto& part=leafParts[i];
            Fbox box=part.box;box.grow(.6f);
            if(!box.contains(p))continue;
            if(match>=0 && match!=int(i))return -1;
            match=int(i);
        }
    }
    return match>=0 && leafParts[u32(match)].burnClaimed ? match : -1;
}

int crownOver(const Fvector& p, float high, bool onTrunk=false,bool skipLit=false)
{
	if (!treesKnown) treesLearn();
	const int gx = iFloor(p.x / 8.f), gz = iFloor(p.z / 8.f);
	for (int dz = -1; dz <= 1; ++dz)
		for (int dx = -1; dx <= 1; ++dx)
		{
			const auto found = leafGrid.find(leafKey(gx + dx, gz + dz));
			if (found == leafGrid.end()) continue;
			for (u32 i : found->second)
			{
				if(skipLit && leafParts[i].burnClaimed) continue;
				const Fbox& b = leafParts[i].box;
				if (onTrunk && p.y>=b.min.y-high && p.y<=b.max.y) {
					// A heated trunk surface already inside the existing leaf volume is contact.
					if(b.contains(p)) return int(i);
					const TreePart& leaf=leafParts[i];
					for(u32 k=0;k<leaf.trunkCount;++k) {
						Fbox trunk=crownTrunks[leaf.firstTrunk+k];
						trunk.grow(.15f);
						if(trunk.contains(p)) return int(i);
					}
				}
				if (b.min.y < p.y - 1.f || b.min.y > p.y + high) continue;
				Fvector c;
				b.getcenter(c);
				const float half = _max(b.max.x - b.min.x, b.max.z - b.min.z) * .5f;
				const float reach = _max(1.2f, .3f * half);
				if ((c.x - p.x) * (c.x - p.x) + (c.z - p.z) * (c.z - p.z) <= reach * reach) return int(i);
			}
		}
	return -1;
}
int crownNearTrunk(const Fvector& p, float reach)
{
	if (!treesKnown) treesLearn();
	const int gx = iFloor(p.x / 8.f), gz = iFloor(p.z / 8.f);
	for (int dz = -1; dz <= 1; ++dz)
		for (int dx = -1; dx <= 1; ++dx)
		{
			const auto found = leafGrid.find(leafKey(gx + dx, gz + dz));
			if (found == leafGrid.end()) continue;
			for (u32 i : found->second)
			{
				const TreePart& leaf = leafParts[i];
				if (leaf.burnClaimed) continue;
				Fbox b = leaf.box;
				b.grow(.6f);
				if (!b.contains(p)) continue;
				auto closeTo = [&](const Fbox& box) { // ("near" is a word Windows' headers take for themselves)
					Fvector m;
					box.getcenter(m);
					return (m.x - p.x) * (m.x - p.x) + (m.z - p.z) * (m.z - p.z) <= reach * reach;
				};
				if (leaf.trunkCount)
				{
					for (u32 k = 0; k < leaf.trunkCount; ++k)
						if (closeTo(crownTrunks[leaf.firstTrunk + k])) return int(i);
				}
				else if (leaf.box.max.y - leaf.box.min.y <= 4.f || closeTo(leaf.box))
					return int(i);
			}
		}
	return -1;
}
u32 crownScanCursor=0;
float crownEmberUntil=0.f;
struct TreeFire
{
	u32 part=0;
	float started=0.f;
	dxRender_Visual* leaf = nullptr;
	float until = 0.f;      // then it has lost its leaves
	bool bared = false;
	xr_vector<u32> fires;   // its fires (Fire::id)
};
xr_vector<TreeFire> treeFires;
std::unordered_set<const void*> treeLit; // the crowns that burn or have burnt
// a crown catches: fires through its whole volume, the lowest first, climbing to the top
void treeIgnite(u32 part,u32 triggerCell)
{
	if (part >= leafParts.size()) return;
	dxRender_Visual* v = leafParts[part].v;
	if (treeLit.count(v)) return;
	const int take = findTake("body");
	if (take < 0) return;
	const Fbox& b = leafParts[part].box;
	Fvector c;
	b.getcenter(c);
	const float W = _min(b.max.x - b.min.x, 12.f), D = _min(b.max.z - b.min.z, 12.f), H = _min(b.max.y - b.min.y, 14.f);
	const u32 n = u32(clampr(W * D * H / 30.f, 3.f, 9.f));
	TreeFire t;
	t.leaf = v;
	t.part=part;
	const u64 crownGroup = newFrontGroup();
	const float now = Device.fTimeGlobal;
	t.started=t.until=now;
	Rng r{u32(size_t(v) >> 4) * 2654435761u + 977u};
	r.next();
	const float burn = PFS::rule(PFS::BRUSH).burnTime * ps_painter_fire_burn_time;
	for (u32 k = 0; k < n; ++k)
	{
		const float up = n > 1 ? float(k) / float(n - 1) : .5f;
		Fvector p = {c.x + (r.next() - .5f) * .55f * W, b.min.y + H * (.08f + .62f * up + (r.next() - .5f) * .08f),
			c.z + (r.next() - .5f) * .55f * D};
		bool onGround = false;
		float groundY = 0.f;
		if (H <= 4.f && firePictureFloor(p, .2f, (p.y - b.min.y) + 1.5f, groundY) && b.min.y - groundY < 1.5f)
		{
			p.y = groundY;
			onGround = true;
		}
		Fire* f = add(take, p);
		if (!f) continue;
		float delay = .3f + up * 5.5f + .6f * r.next();
		if (triggerCell < field.cells.size())
		{
			const PFS::Cell& from = field.cells[triggerCell];
			const float dx = p.x - from.p.x, dz = p.z - from.p.z, dy = p.y - from.p.y;
			delay = (1.f + _sqrt(dx * dx + dz * dz) / .3f + (dy > 0.f ? dy / .55f : -dy / .15f)) * (.8f + .4f * r.next());
		}
		f->delay = delay; // it climbs: the lowest catch first
		f->frontGroup = crownGroup;
        f->frontCell = triggerCell+1;
		f->life = burn * (.8f + .4f * r.next());
		f->fadeIn = 3.f;   // firerework: it shows slowly ...
		f->growFor = 8.f;  // ... and grows from small
		f->fadeOut = 3.f;
		f->phase = r.next() * 60.f;
		f->mirror = r.next() > .5f;
		f->yaw = r.next() * PI_MUL_2;
		f->scale = clampr(_min(W, D) / 4.5f, .45f, 1.3f) * (.85f + .3f * r.next());
		if (onGround) f->scale = clampr((b.max.y - groundY) * 1.35f / 3.6f, .45f, 1.3f) * (.9f + .2f * r.next()); // firerework: over the bush
		crownEmberUntil=_max(crownEmberUntil,now+f->delay+f->life);
		f->crownEmbers = true;
		f->air = onGround ? .1f : .45f; // fade the root inside the canopy (firerework: a bush's fire stands on the ground)
		f->lightK = k == 0 || k == n - 1 ? .7f : 0.f;
		f->id = nextFireId++;
		t.fires.push_back(f->id);
		t.until = _max(t.until, now + f->delay + .5f * f->life);
	}
	if(t.fires.empty()) return;
	treeLit.insert(v);
	leafParts[part].burnClaimed=true;
	treeFires.push_back(t);
	bareDirty=true;
}
// a few times a second: what burns under a crown, on its trunk or in it sets it alight; a crown that has burnt loses its
// leaves; one that has grown its leaves back (painter_fire_burnt_minutes) can burn again
void treeFiresUpdate()
{
	const float now = Device.fTimeGlobal;
	static float lookAt = 0.f;
	if (!field.idle() && (now >= lookAt || lookAt - now > 1.f))
	{
		lookAt = now + .25f;
		const auto& burning=field.burningCells();
		const u32 total=u32(burning.size()), count=_min(total,400u);
		if(crownScanCursor>=total) crownScanCursor=0;
		for(u32 scanned=0;scanned<count;++scanned)
		{
			const u32 i=burning[crownScanCursor++];
			if(crownScanCursor==total) crownScanCursor=0;
			if (i >= field.cells.size()) continue;
			const PFS::Cell& c = field.cells[i];
			if (c.state != PFS::BURNING) continue;
			const Fvector p = {c.p.x, c.p.y, c.p.z};
			int part = -1;
			if (c.fuel == PFS::BRUSH && c.kind != PFS::GROUND)
				part = crownNearTrunk(p, 2.5f);
			else if (c.kind == PFS::WALL && c.trunk)
				part = crownOver(p, field.cellSize * (.55f + c.size), true,true);
			else if (c.kind == PFS::GROUND && c.size > .55f)
				part = crownOver(p, field.cellSize * (.35f + c.size),false,true);
			if (part >= 0) treeIgnite(u32(part),i);
		}
	}
	for (TreeFire& t : treeFires)
		if (!t.bared && now >= t.until)
		{
			t.bared = true;
			bool had = false;
			for (const BareTree& x : bareTrees)
				if (x.leaf == t.leaf)
				{
					had = true;
					break;
				}
			if (!had)
			{
				BareTree x;
				x.leaf = t.leaf;
				x.at = now;x.started=t.started;x.ended=t.until;
				bareTrees.push_back(x);
				bareDirty = true;
			}
		}
	if (!treeFires.empty())
		treeFires.erase(std::remove_if(treeFires.begin(), treeFires.end(), [&](const TreeFire& t) {
			if (!t.bared) return false;
			for (const BareTree& x : bareTrees)
				if (x.leaf == t.leaf) return false;
			treeLit.erase(t.leaf);
			if(t.part<leafParts.size() && leafParts[t.part].v==t.leaf) leafParts[t.part].burnClaimed=false;
			return true;
		}), treeFires.end());
}
void treeFiresClear()
{
	for (const TreeFire& t : treeFires)
		for (u32 id : t.fires) endPicture(id, .5f);
	for(const TreeFire& t:treeFires) if(t.part<leafParts.size() && leafParts[t.part].v==t.leaf) leafParts[t.part].burnClaimed=false;
	treeFires.clear();
	treeLit.clear();
	crownScanCursor=0;
	crownEmberUntil=0.f;
}
void barePublish()
{
    auto s=std::make_shared<BareSet>();
    auto remember=[](auto& m, const void* v,PainterTreeDates::Span d){
        if (!v) return;
        const auto i=m.emplace(v,d);
        if (!i.second) PainterTreeDates::include(i.first->second,d);
    };
    auto collect=[&](dxRender_Visual* leaf,PainterTreeDates::Span d){
        remember(s->leaves,leaf,d);
        if(treeIndependentLeaves.count(leaf)) s->independentLeaves.insert(leaf);
        const auto it=treeLeafOwners.find(leaf);
        if(it==treeLeafOwners.end())return;
        for(const auto owner:it->second){
            remember(s->lods,owner,d);
            const auto wood=treeOwnerWood.find(owner);
            if(wood!=treeOwnerWood.end())for(const auto v:wood->second)remember(s->wood,v,d);
        }
    };
    for(const auto& b:bareTrees)collect(b.leaf,{b.started,b.ended});
    for(const auto& t:treeFires)collect(t.leaf,{t.started,t.until});
    const u32 count=u32(s->leaves.size());
    std::atomic_store_explicit(&bareNow,std::shared_ptr<const BareSet>(s),std::memory_order_release);
    bareCount.store(count,std::memory_order_release);
}// once a frame: crowns that burnt lose their leaves; with painter_fire_burnt_minutes they grow back
void bareUpdate()
{
	const float now = Device.fTimeGlobal;
	if (ps_painter_fire_burnt_minutes > 0.f && !bareTrees.empty())
	{
		const float lasts = ps_painter_fire_burnt_minutes * 60.f + 60.f;
		const size_t before = bareTrees.size();
		bareTrees.erase(std::remove_if(bareTrees.begin(), bareTrees.end(), [&](const BareTree& b) { return now - b.at > lasts; }),
			bareTrees.end());
		if (bareTrees.size() != before) bareDirty = true;
	}
	if (bareDirty && (now >= barePublishAt || barePublishAt - now > 2.f))
	{
		bareDirty = false;
		barePublishAt = now + .5f;
		barePublish();
	}
}
// the spreading fire is put out (the console) or the level ends: every tree has its leaves again
void bareReset()
{
	treeFiresClear();
	bareTrees.clear();
	bareAsked.clear();
	bareDirty = false;
	if (bareCount.load(std::memory_order_relaxed)) barePublish();
}
void bareForget() // the level ends: nothing of it is drawn any more
{
	treeFiresClear();
	crownTrunks.clear();
	bareIndex.reset();
	leafGrid.clear();
	treeLeafOwners.clear();treeOwnerWood.clear();treeIndependentLeaves.clear();
	std::atomic_store_explicit(&bareNow,std::shared_ptr<const BareSet>(),std::memory_order_release);
	bareCount.store(0, std::memory_order_release);
	bareTrees.clear();
	bareAsked.clear();
	leafParts.clear();
	farPictures.clear();
	treesKnown = false;
	bareDirty = false;
}
// ---------------------------------------------------------------- what the fire leaves behind
// A cell that burns, or has burnt, chars what it stands on (renderChar), and grass on a burnt ground cell is gone (the grass
// renderer asks GrassBurnt for every blade).
struct Charred
{
	float amount = 0.f; // 0..1: how far it has burnt
	float coolAt = 0.f; // the clock when it stopped burning (0: it still burns)
	bool listed = false;
};
xr_vector<Charred> charred; // by cell number
xr_vector<u32> charCells;   // the cells with anything burnt
// Each cached patch is a connected piece of the level's collision wood in one field voxel.
struct WoodPatch {
    int x=0,y=0,z=0;
    const void* crown=nullptr;
    PainterWoodFaces::Patch geometry;
};
std::unordered_map<u32,WoodPatch> woodPatches;
WoodPatch collectWood(const PFS::Cell& c)
{
    WoodPatch patch;
    if(c.woodFace<0 || !g_pGameLevel) return patch;
    const float width=field.cellSize;
    patch.x=iFloor(c.p.x/width+.5f);patch.y=iFloor(c.p.y/width+.5f);patch.z=iFloor(c.p.z/width+.5f);
    const Fvector centre={patch.x*width,patch.y*width,patch.z*width};
    const float half=width*.5f;
    const Fvector range={half+.003f,half+.003f,half+.003f};
    auto* tri=g_pGameLevel->ObjectSpace.GetStaticTris();
    if(!tri) return patch;
    const auto material=tri[c.woodFace].material;
    SGameMtl* m=GMLib.GetMaterialByIdx(u16(material));
    if(!m || notASurface(m->m_Name.c_str())) return patch;
    const char* materialName=m->m_Name.c_str();
    const bool bark=strstr(materialName,"tree_trunk")!=nullptr;
    if(!bark && !strstr(materialName,"wood")) return patch;
    if(bark && treesKnown) {
        const Fvector hit={c.p.x,c.p.y,c.p.z};
        const int part=claimedCrownAt(hit);
        if(part>=0) patch.crown=leafParts[part].v;
    }
    static xrXRC woodQuery;
    woodQuery.box_options(0);woodQuery.box_query(g_pGameLevel->ObjectSpace.GetStaticModel(),centre,range);
    std::vector<PainterWoodFaces::Face> faces;
    for(CDB::RESULT* r=woodQuery.r_begin();r!=woodQuery.r_end();++r) {
        if(r->material!=material) continue;
        if(faces.size()==192) return patch;
        PainterWoodFaces::Face face={};face.id=r->id;
        for(int k=0;k<3;++k){face.vertex[k]=tri[r->id].verts[k];face.p[k]={r->verts[k].x,r->verts[k].y,r->verts[k].z};}
        faces.push_back(face);
    }
    patch.geometry=PainterWoodFaces::build(faces,c.woodFace,{centre.x,centre.y,centre.z},half);
    return patch;
}

struct BurnSpot
{
	int kx = 0, kz = 0;
	float amount = 0.f;
	float y = 0.f;      // how high the ground lies there
	bool openAir = false; // that ground lies in the open (not a room's floor)
	float coolAt = 0.f; // the clock when it stopped burning (0: it still burns)
	float fire = 0.f;   // how strongly it burns right now (0: not)
	bool used = false;
};
const u32 burnSide = 256; // the table repeats every 256 cells (410 m): a far burnt place may be forgotten for a near one
xr_vector<BurnSpot> burnTable;
u32 burnSpots = 0;
ID3DTexture2D* groundHeatTex=nullptr;
ID3DShaderResourceView* groundHeatSrv=nullptr;
float groundHeatLast=-1.f;
int groundHeatCooling=-1;
bool groundHeatDirty=true;
void uploadGroundHeat(float now)
{
    if(!fireHeatBound || burnTable.empty())return;
    const bool sameMode=groundHeatCooling==heat_vision_cooldown;
    const bool rewind=now<groundHeatLast;
    if(sameMode && !rewind && now-groundHeatLast<.5f)return;
    if(sameMode && !rewind && !groundHeatDirty){groundHeatLast=now;return;}
    if(!groundHeatTex){
        D3D11_TEXTURE2D_DESC d={};
        d.Width=d.Height=burnSide;d.MipLevels=d.ArraySize=d.SampleDesc.Count=1;
        d.Format=DXGI_FORMAT_R8_UNORM;d.Usage=D3D11_USAGE_DYNAMIC;
        d.BindFlags=D3D11_BIND_SHADER_RESOURCE;d.CPUAccessFlags=D3D11_CPU_ACCESS_WRITE;
        if(FAILED(HW.pDevice->CreateTexture2D(&d,nullptr,&groundHeatTex)))return;
        if(FAILED(HW.pDevice->CreateShaderResourceView(groundHeatTex,nullptr,&groundHeatSrv))){_RELEASE(groundHeatTex);return;}
    }
    D3D11_MAPPED_SUBRESOURCE m={};
    if(FAILED(HW.pContext->Map(groundHeatTex,0,D3D11_MAP_WRITE_DISCARD,0,&m)))return;
    bool fading=false;
    for(u32 z=0;z<burnSide;++z){
        u8* row=static_cast<u8*>(m.pData)+z*size_t(m.RowPitch);
        for(u32 x=0;x<burnSide;++x){
            const BurnSpot& s=burnTable[z*burnSide+x];
            float h=s.used?PainterSurfaceHeat::level(now,s.coolAt,heat_vision_cooldown!=0):0.f;
            row[x]=PainterSurfaceHeat::byte(h);
            fading |= s.used && s.coolAt>0.f && h>0.f && heat_vision_cooldown!=0;
        }
    }
    HW.pContext->Unmap(groundHeatTex,0);
    groundHeatLast=now;
    groundHeatDirty=fading;
    groundHeatCooling=heat_vision_cooldown;
}

u32 burnGrassRevision = 0; // GrassBurntRevision
BurnSpot& burnAt(int kx, int kz) { return burnTable[(u32(kx) & (burnSide - 1)) + (u32(kz) & (burnSide - 1)) * burnSide]; }
// the burnt-ground map: the table as a texture the ground shader reads (r how burnt, g the ground's height, b how hot)
ID3DTexture2D* burnTex = nullptr;
ID3DShaderResourceView* burnSrv = nullptr;
bool burnDirty = false;
float burnHotUntil = 0.f, burnUploadAt = 0.f;
// One bit per eight adjacent ground cells. Empty neighbours still carry a height.
struct BurnPackedCell { u16 value[4]; };
xr_vector<BurnPackedCell> burnPacked;
u32 burnChanged[burnSide]={}, burnCooling[burnSide]={};
bool burnNeedsPack=false, burnLiveCooling=false, burnPublished=false, burnPackedCurrent=true;
float burnPackClock=0.f;
void markBurnPacked(int kx,int kz)
{
    burnChanged[u32(kz)&(burnSide-1)] |= 1u<<((u32(kx)&(burnSide-1))/8);
    burnNeedsPack=true;
}
void resetBurnPacked()
{
    xr_vector<BurnPackedCell>().swap(burnPacked);
    memset(burnChanged,0,sizeof(burnChanged));
    memset(burnCooling,0,sizeof(burnCooling));
    burnNeedsPack=false; burnLiveCooling=false; burnPublished=false; burnPackedCurrent=true; burnPackClock=0.f;
}
u16 toHalf(float f)
{
	u32 x;
	memcpy(&x, &f, 4);
	const u32 sign = (x >> 16) & 0x8000u;
	const int e = int((x >> 23) & 0xffu) - 127 + 15;
	if (e <= 0) return u16(sign);            // too small to matter here
	if (e >= 31) return u16(sign | 0x7bffu); // the largest there is
	return u16(sign | (u32(e) << 10) | ((x & 0x7fffffu) >> 13));
}
// The burnt ground is drawn on the level's own triangles. The ones under and around every burnt cell are collected here
// (each once), and what is drawn on them is tested against the scene's depth like any surface: a body lying on burnt
// ground is in front of it and stays as it is. (Drawn through the scene's depth picture instead, the char and its glowing
// embers landed on bodies too.)
xr_vector<Fvector> burnVerts; // three for every triangle
std::unordered_set<int> burnTris;
ID3DVertexBuffer* burnVB = nullptr;
ref_geom geomBurn;
u32 burnVBVerts = 0;
bool burnMeshDirty = false;
float burnMeshAt = 0.f;
void burnTrisAdd(float x, float y, float z)
{
	if (!g_pGameLevel || burnVerts.size() >= 3u * 120000u) return;
	static xrXRC xrc;
	// (the shader moves the burnt edge about by up to 3.5 m: the triangles that far around the cell are needed too)
	const float h = .5f * field.cellSize + 3.6f;
	const Fvector c = {x, y, z}, dim = {h, 1.6f, h};
	xrc.box_options(0);
	xrc.box_query(g_pGameLevel->ObjectSpace.GetStaticModel(), c, dim);
	for (CDB::RESULT* r = xrc.r_begin(); r != xrc.r_end(); ++r)
	{
		if (burnTris.count(r->id)) continue;
		Fvector n;
		n.mknormal(r->verts[0], r->verts[1], r->verts[2]);
		if (n.y < .35f) continue; // only what faces up: the ground, a floor, a roof
		SGameMtl* m = GMLib.GetMaterialByIdx(u16(r->material));
		if (m && notASurface(m->m_Name.c_str())) continue;
		burnTris.insert(r->id);
		for (int k = 0; k < 3; ++k) burnVerts.push_back(r->verts[k]);
	}
	burnMeshDirty = true;
}
void burnMeshClear()
{
	burnVerts.clear();
	burnTris.clear();
	burnMeshDirty = true;
	burnMeshAt = 0.f;
}
void burnMeshUpload()
{
	const float now = Device.fTimeGlobal;
	if (!burnMeshDirty || (now < burnMeshAt && burnMeshAt - now < 2.f)) return;
	burnMeshAt = now + .3f;
	burnMeshDirty = false;
	geomBurn.destroy();
	_RELEASE(burnVB);
	burnVBVerts = 0;
	if (burnVerts.empty()) return;
	if (FAILED(dx10BufferUtils::CreateVertexBuffer(&burnVB, burnVerts.data(), UINT(burnVerts.size() * sizeof(Fvector)), true)) || !burnVB) return;
	geomBurn.create(D3DFVF_XYZ, burnVB, nullptr);
	burnVBVerts = u32(burnVerts.size());
}
// Two places of one column that lie farther apart in height than this are two floors (a cellar under the ground, a room
// over a room). The map holds one of them.
const float burnFloor = 1.8f;
void burnSet(int kx, int kz, float amount, float y, bool cooled, float fire, bool openAir)
{
	if (burnTable.empty()) burnTable.resize(burnSide * burnSide);
	BurnSpot& s = burnAt(kx, kz);
	bool fresh = false;
	if (!s.used)
	{
		++burnSpots;
		fresh = true;
	}
	else if (s.kx != kx || s.kz != kz)
	{
		s.amount = 0.f;
		s.coolAt = 0.f;
		s.fire = 0.f;
		fresh = true;
	}
	else if (_abs(s.y - y) > burnFloor)
	{
		// Another floor of this column. The floor that burnt first keeps the place; only ground in the open takes it from
		// a room's floor. (Two floors that burnt wrote over each other in turns: the place's height, how burnt and how
		// hot it is jumped between them.)
		if (s.openAir || !openAir) return;
		s.amount = 0.f;
		s.coolAt = 0.f;
		s.fire = 0.f;
		fresh = true;
	}
	if(fresh || (s.coolAt==0.f)==cooled)groundHeatDirty=true;
	if (fresh) burnTrisAdd(float(kx) * field.cellSize, y, float(kz) * field.cellSize);
	markBurnPacked(kx,kz);
	s.kx = kx;
	s.kz = kz;
	s.used = true;
	s.y = y;
	s.openAir = openAir;
	s.fire = cooled ? 0.f : fire;
	if (fresh || amount > s.amount) ++burnGrassRevision;
	if (amount > s.amount) s.amount = amount;
	if (cooled)
	{
		if (s.coolAt == 0.f) s.coolAt = _max(Device.fTimeGlobal, .001f);
	}
	else
		s.coolAt = 0.f;
	burnHotUntil = Device.fTimeGlobal + 55.f;
	burnDirty = true;
	// the cells around it learn how high the ground lies here: the map is read smoothly, between cells too
	for (int dz = -1; dz <= 1; ++dz)
		for (int dx = -1; dx <= 1; ++dx)
		{
			BurnSpot& n = burnAt(kx + dx, kz + dz);
			if (!n.used) { n.y=y;markBurnPacked(kx+dx,kz+dz); }
		}
}
void burnUpload()
{
	if (burnTable.empty()) return;
	const float now = Device.fTimeGlobal;
	if (!burnDirty && now > burnHotUntil) return;
	if (now < burnUploadAt && now > burnUploadAt - 1.f) return;
	burnUploadAt = now + .3f;
    const bool rewind = !burnPacked.empty() && now < burnPackClock;
    if (burnPublished && burnPackedCurrent && !rewind && !burnNeedsPack && !burnLiveCooling)
    {
        burnDirty=false;
        burnPackClock=now;
        return;
    }
    burnDirty=true; burnPublished=false;
    if (burnPacked.empty()) burnPacked.resize(burnSide*burnSide,BurnPackedCell{});
	if (!burnTex)
	{
		D3D11_TEXTURE2D_DESC d;
		ZeroMemory(&d, sizeof(d));
		d.Width = d.Height = burnSide;
		d.MipLevels = 1;
		d.ArraySize = 1;
		d.Format = DXGI_FORMAT_R16G16B16A16_FLOAT;
		d.SampleDesc.Count = 1;
		d.Usage = D3D11_USAGE_DYNAMIC;
		d.BindFlags = D3D11_BIND_SHADER_RESOURCE;
		d.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
		if (FAILED(HW.pDevice->CreateTexture2D(&d, nullptr, &burnTex)) || !burnTex) return;
		if (FAILED(HW.pDevice->CreateShaderResourceView(burnTex, nullptr, &burnSrv)) || !burnSrv)
		{
			_RELEASE(burnTex);
			return;
		}
	}
	D3D11_MAPPED_SUBRESOURCE m;
	if (FAILED(HW.pContext->Map(burnTex, 0, D3D11_MAP_WRITE_DISCARD, 0, &m))) return;
    bool dense=burnNeedsPack;
    if (dense) for (u32 z=0;z<burnSide;++z) if (burnChanged[z]!=~0u) { dense=false;break; }
    if (dense)
    {
	for (u32 z = 0; z < burnSide; ++z)
	{
		u16* row = (u16*)((u8*)m.pData + size_t(z) * m.RowPitch);
		for (u32 x = 0; x < burnSide; ++x, row += 4)
		{
			const BurnSpot& s = burnTable[x + z * burnSide];
			const float hot = !s.used ? 0.f : s.coolAt == 0.f ? 1.f : clampr(1.f - (now - s.coolAt) / 50.f, 0.f, 1.f);
			row[0] = toHalf(s.used ? s.amount : 0.f);
			row[1] = toHalf(s.y);
			row[2] = toHalf(hot);
			row[3] = toHalf(s.used ? s.fire : 0.f);
		}
	}
        memset(burnChanged,0,sizeof(burnChanged));
        burnPackedCurrent=false;
    }
    else
    {
    burnLiveCooling=false;
    for (u32 z=0;z<burnSide;++z)
    {
        const u32 edited=(rewind || !burnPackedCurrent) ? ~0u : burnChanged[z];
        u32 blocks=edited|burnCooling[z], stillCooling=0;
        while(blocks)
        {
            unsigned long block;
            _BitScanForward(&block,blocks);
            const u32 bit=1u<<block;
            blocks &= ~bit;
            const bool rewrite=(edited&bit)!=0;
            for (u32 x=block*8; x<block*8+8; ++x)
            {
                const u32 index=z*burnSide+x;
                const BurnSpot& spot=burnTable[index];
                u16* p=burnPacked[index].value;
                const float hot=!spot.used ? 0.f : spot.coolAt==0.f ? 1.f : clampr(1.f-(now-spot.coolAt)/50.f,0.f,1.f);
                if (rewrite)
                {
                    p[0]=toHalf(spot.used ? spot.amount : 0.f);
                    p[1]=toHalf(spot.y);
                    p[3]=toHalf(spot.used ? spot.fire : 0.f);
                }
                p[2]=toHalf(hot);
                if (spot.used && spot.coolAt!=0.f && hot>0.f) stillCooling|=bit;
            }
        }
        burnChanged[z]=0;
        burnCooling[z]=stillCooling;
        burnLiveCooling |= stillCooling!=0;
    }
    const size_t rowBytes=burnSide*sizeof(BurnPackedCell);
    if (m.RowPitch==rowBytes) memcpy(m.pData,burnPacked.data(),rowBytes*burnSide);
    else for (u32 z=0;z<burnSide;++z)
        memcpy((u8*)m.pData+z*size_t(m.RowPitch),burnPacked.data()+z*burnSide,rowBytes);
    burnPackedCurrent=true;
    }
    burnPublished=true;burnNeedsPack=false;burnPackClock=now;
	HW.pContext->Unmap(burnTex, 0);
	burnDirty = false;
}
void leftoversClear()
{
	bareReset();
	charred.clear();
	charCells.clear();
	woodPatches.clear();
	burnTable.clear();
	groundHeatDirty=true;groundHeatLast=-1.f;
	resetBurnPacked();
	burnSpots = 0;
	++burnGrassRevision;
	burnDirty = true;
	burnMeshClear();
}
// once a frame while anything burns
void leftoversUpdate(float dt)
{
	if (charred.size() < field.cells.size()) charred.resize(field.cells.size());
	for (u32 i : field.changed)
	{
		if (i >= charred.size()) continue;
		Charred& h = charred[i];
		const PFS::Cell& c = field.cells[i];
		if (!h.listed && c.state != PFS::COLD)
		{
			h.listed = true;
			charCells.push_back(i);
		}

		if (c.state == PFS::BURNT)
		{
			if (h.coolAt == 0.f) h.coolAt = _max(Device.fTimeGlobal, .001f);
			// bare ground that only had thrown fuel burning on it is scorched, not charred
			h.amount = c.fuel == PFS::NOTHING ? _min(_max(h.amount, .3f), .5f) : 1.f;
			if (c.kind == PFS::GROUND) burnSet(c.kx, c.kz, h.amount, c.p.y, true, 0.f, !c.roofed);
		}
	}
	for (u32 i : field.burningCells())
	{
		if (i >= charred.size()) continue;
		Charred& h = charred[i];
		const PFS::Cell& c = field.cells[i];
		if (!h.listed)
		{
			h.listed = true;
			charCells.push_back(i);
		}
		h.coolAt = 0.f;
		const bool catching = c.state == PFS::CATCHING;
		const float most = c.fuel == PFS::NOTHING ? .5f : 1.f;
		h.amount = _max(h.amount, _min(catching ? .3f : most, h.amount + dt / (catching ? 4.f : 3.f)));
		if (c.kind == PFS::GROUND) burnSet(c.kx, c.kz, h.amount, c.p.y, false, clampr(.25f + c.size, 0.f, 1.f), !c.roofed);
	}
}
// What burnt grows back (painter_fire_burnt_minutes, the Fire menu): that long after the fire at a place went out, its
// char fades over a minute, its grass comes back and it can burn again. Looked at once a second, also when nothing burns.
void leftoversRegrow()
{
	static float last = 0.f;
	const float now = Device.fTimeGlobal;
	if (_abs(now - last) < 1.f) return;
	const float step = _min(_abs(now - last), 5.f);
	last = now;
	if (ps_painter_fire_burnt_minutes <= 0.f || charCells.empty()) return;
	const float lasts = ps_painter_fire_burnt_minutes * 60.f;
	bool any = false;
	for (size_t a = 0; a < charCells.size();)
	{
		const u32 i = charCells[a];
		if (i >= charred.size() || i >= field.cells.size())
		{
			++a;
			continue;
		}
		Charred& h = charred[i];
		const PFS::Cell& c = field.cells[i];
		if (h.coolAt == 0.f || c.state != PFS::BURNT || now - h.coolAt < lasts)
		{
			++a;
			continue;
		}
		h.amount = _max(0.f, h.amount - step / 60.f);
		if (c.kind == PFS::GROUND && !burnTable.empty())
		{
			BurnSpot& s = burnAt(c.kx, c.kz);
			if (s.used && s.kx == c.kx && s.kz == c.kz && _abs(s.y - c.p.y) <= burnFloor)
			{
				markBurnPacked(c.kx,c.kz);
				if (h.amount < s.amount) ++burnGrassRevision;
				s.amount = _min(s.amount, h.amount);
				if (h.amount <= 0.f)
				{
					s.used = false;
					groundHeatDirty=true;
					s.amount = 0.f;
					s.coolAt = 0.f;
					s.fire = 0.f;
					if (burnSpots) --burnSpots;
				}
			}
		}
		any = true;
		if (h.amount <= 0.f)
		{
			// grown back: it can burn again
			h.amount = 0.f;
			h.coolAt = 0.f;
			h.listed = false;
			field.regrow(i);
			charCells[a] = charCells.back();
			charCells.pop_back();
			continue;
		}
		++a;
	}
	if (any) burnDirty = true;
}
// ---------------------------------------------------------------- the smoke of a big fire
// Smoke rises off the whole burning area as one moving mass: many soft balls of smoke ("puffs") that overlap into one
// cloud. Each is born over burning ground, rises, widens, drifts with the wind and fades after about a minute. Their
// picture is the shader's part (painter_fire_smoke.ps): a cloudy pattern that is the same for the whole world eats into
// every puff's rim, so puffs that overlap join into one rolling cloud.
struct Puff
{
	u32 sourceCell = PFS::Field::NO_CELL;
	Fvector pos = {0.f, 0.f, 0.f};
	float vx = 0.f, vz = 0.f; // what the wind has given it so far (m/s)
	float dx = 0.f, dz = 0.f; // its own way: every puff wanders off a little, so the cloud widens
	float age = 0.f, life = 60.f;
	float r0 = 3.f, rise0 = 2.f, dens0 = .7f, albedo0 = .6f;
	float ground = 0.f; // the height of the ground it rose from
	// as it is in this frame
	float radius = 3.f, dens = .7f, albedo = .6f, young = 1.f, fade = 0.f, show = 0.f, dist2 = 0.f;
};
float puffSourceGlow(const Puff& p, float base)
{
    if (ps_painter_fire_light_quality <= 0 || p.age >= 12.f || p.sourceCell >= field.cells.size())
        return base;
    const auto& c = field.cells[p.sourceCell];
    if (c.roofed || c.lid != 0.f || c.kind == PFS::CEILING || c.size <= .01f ||
        (c.state != PFS::BURNING && c.state != PFS::CATCHING && c.state != PFS::DYING))
        return base;

    const float dx = p.pos.x - c.p.x, dz = p.pos.z - c.p.z, dy = p.pos.y - c.p.y;
    const float footprint = 1.3f + 2.4f*c.size;
    const float across = 1.f - (dx*dx+dz*dz)/(footprint*footprint);
    const float bottom = _max(0.f, dy - .6f*p.radius - (.35f+.36f*c.size));
    const float vertical = 1.f - bottom/(4.5f + 6.f*c.size);
    if (across <= 0.f || vertical <= 0.f || dy < 0.f) return base;
    if (c.kind == PFS::WALL && dx*c.n.x + dz*c.n.z < -.05f) return base;
    const float halfCell = .5f*field.cellSize;
    if (fabsf(p.pos.x-float(c.kx)*field.cellSize)>halfCell || fabsf(p.pos.z-float(c.kz)*field.cellSize)>halfCell)
    {
        const u32 j = field.groundAt(p.pos.x,c.p.y,p.pos.z);
        if (j == PFS::Field::NO_CELL || field.cells[j].roofed || field.cells[j].lid != 0.f ||
            fabsf(field.cells[j].p.y-c.p.y) > field.cellSize) return base;
    }
    const float fresh = 1.f-p.age/12.f;
    const float dry = clampr((.84f-p.albedo)*3.f,0.f,1.f);
    const float source = clampr(c.size*ps_painter_fire_brightness,0.f,1.f);
    const float gain = ps_painter_fire_light_quality == 1 ? 1.05f : ps_painter_fire_light_quality >= 3 ? 2.2f : 1.85f;
    return base*(1.f + gain*across*vertical*fresh*dry*source);
}
xr_vector<Puff> puffs;
float puffOwed = 0.f;
u32 puffBorn = 0, puffSeed = 0x9e3779b9u;
Fvector puffFlow = {0.f, 0.f, 0.f}; // how far the cloudy pattern has drifted (m); it meets itself again every 24 m
float puffRnd()
{
	puffSeed ^= puffSeed << 13;
	puffSeed ^= puffSeed >> 17;
	puffSeed ^= puffSeed << 5;
	return float(puffSeed & 0xffffffu) / 16777216.f;
}
float puffWrap(float v)
{
	v = fmodf(v, 24.f);
	return v < 0.f ? v + 24.f : v;
}
void puffsClear()
{
	puffs.clear();
	puffOwed = 0.f;
}
void puffsUpdate()
{
	if (!ps_painter_fire || !ps_painter_fire_smoke)
	{
		puffsClear();
		return;
	}
	if (!g_pGameLevel || Device.Paused()) return;
	const float dt = _min(Device.fTimeDelta, .1f);
	const PFS::Weather wx = weather();
	const float steam = clampr(1.2f * wx.rain + .3f * wx.wet, 0.f, 1.f);
	// what burns in the open feeds the smoke
	static xr_vector<u32> open;
	open.clear();
	if (!field.idle())
		for (u32 i : field.burningCells())
			if (!field.cells[i].roofed) open.push_back(i);
	const u32 n = u32(open.size());
	const u32 cap = ps_painter_fire_quality <= 0 ? 70u : ps_painter_fire_quality == 1 ? 120u : 180u;
	if (n > 6)
	{
		// how big a new puff is and how many are born a second: so that they close into one cover over all that burns
		const float area = float(n) * field.cellSize * field.cellSize;
		const float rStart = clampr(1.6f + .22f * _sqrt(area), 2.2f, 5.5f);
		const float need = area / (.5f * PI * rStart * rStart) * (2.f / (.6f * rStart));
		const float rate = _min(float(cap) / 62.f, need) * clampr(float(n - 6) / 14.f, 0.f, 1.f) * (1.f + .5f * wx.rain);
		puffOwed += rate * dt;
		while (puffOwed >= 1.f)
		{
			puffOwed -= 1.f;
			++puffBorn;
			if (puffs.size() >= cap) continue;
			// one after the other all along the fire, never in a clump
			float pick = float(puffBorn % 100000u) * .6180339887f;
			pick -= floorf(pick);
			const u32 sourceCell = open[_min(u32(pick * float(n)), n - 1)];
			const PFS::Cell& c = field.cells[sourceCell];
			Puff p;
			p.sourceCell = sourceCell;
			p.r0 = rStart * (.7f + .6f * puffRnd());
			const float a = PI_MUL_2 * puffRnd(), speed = .15f + .55f * puffRnd();
			p.dx = _cos(a) * speed;
			p.dz = _sin(a) * speed;
			p.pos.x = c.p.x + 3.f * (puffRnd() - .5f);
			p.pos.y = c.p.y + 1.5f + 2.f * c.size + (.25f + .4f * puffRnd()) * p.r0;
			p.pos.z = c.p.z + 3.f * (puffRnd() - .5f);
			p.ground = c.p.y;
			p.life = 40.f + 45.f * puffRnd();
			p.rise0 = 1.6f + 1.2f * c.size + .6f * puffRnd();
			p.dens0 = 2.1f / p.r0;
			// The burning material determines smoke colour.
			static const float albedo[PFS::FUEL_COUNT] = {.5f, .7f, .55f, .4f, .35f, .22f};
			p.albedo0 = albedo[c.fuel];
			puffs.push_back(p);
		}
	}
	else
		puffOwed = 0.f;
	for (u32 i = 0; i < puffs.size();)
	{
		Puff& p = puffs[i];
		p.age += dt;
		if (p.age >= p.life)
		{
			puffs[i] = puffs.back();
			puffs.pop_back();
			continue;
		}
		// it rises fast at first and slower later; the wind takes it along, more the higher it is
		const float high = clampr((p.pos.y - p.ground) / 25.f, 0.f, 1.f);
		const float k = _min(1.f, dt / 3.f);
		p.vx += (wx.windX * (.35f + .65f * high) - p.vx) * k;
		p.vz += (wx.windZ * (.35f + .65f * high) - p.vz) * k;
		p.pos.x += (p.vx + p.dx) * dt;
		p.pos.y += (.5f + p.rise0 * expf(-p.age / 16.f)) * dt;
		p.pos.z += (p.vz + p.dz) * dt;
		// it widens and thins out; it comes in over 3 s (thin right over the flames) and goes over the last part of its life
		p.radius = p.r0 * (1.f + .6f * (1.f - expf(-p.age / 8.f))) + .09f * p.age;
		p.dens = p.dens0 * powf(p.r0 / p.radius, 1.8f);
		p.fade = _min(1.f, p.age / 3.f) * _min(1.f, (p.life - p.age) / (.4f * p.life));
		// fresh smoke is dark, old smoke paler; rain turns it to steam
		p.young = expf(-p.age / 8.f);
		p.albedo = _min(p.albedo0 + .12f, .95f) * (1.f - .4f * p.young);
		p.albedo += (.93f - p.albedo) * steam;
		++i;
	}
	puffFlow.x = puffWrap(puffFlow.x + .7f * wx.windX * dt);
	puffFlow.y = puffWrap(puffFlow.y + 1.6f * dt);
	puffFlow.z = puffWrap(puffFlow.z + .7f * wx.windZ * dt);
}

void spreadClear()
{
	leftoversClear();
	puffsClear();
	for (u32 i = 0; i < shown.size(); ++i)
	{
		if (shown[i].fire) endPicture(shown[i].fire, .5f);
		if (shown[i].foot) endPicture(shown[i].foot, .5f); // fencefoot
	}
	shown.clear();
	field.clear();
	for(auto& entry:fireColumnCache)entry.family=0;
	for (SpreadLight& l : spreadLights)
	{
		l.used = false;
		l.amount = l.wantAmount = 0.f;
	}
}
// Far more cells can burn than pictures are worth drawing. A grass picture is two cells wide, so every other cell of a
// grass fire carries one and the front still reads as one line; beyond 80 m from the camera one cell in four carries a
// bigger one; and never more than 300 at once.
u32 spreadPictures = 0;
bool wantsPicture(const PFS::Cell& c, Shown& s)
{
	// Flames must run into each other and grow into one fire, not stand in a grid of equal fires. Every ground fire is
	// drawn bigger than its cell, so neighbours overlap. Where the flames are big, fewer cells carry a fire and those are
	// drawn much bigger: mid flames on every second cell, tall flames on one cell in four. A cell gives its own flames up
	// only while such a bigger neighbour really burns (and takes them back when it is gone: spreadRetake).
	s.mult = 1.f;
	if (c.kind != PFS::GROUND)
	{
        if(c.kind==PFS::WALL && c.fuel==PFS::WOOD)return true;
		// Place crown fires in the resolved leaf geometry.
		if (c.fuel == PFS::BRUSH && crownAt(Fvector().set(c.p.x, c.p.y, c.p.z)) >= 0) return false;
		// a wall's or a fence's fires are drawn bigger than their cells too: they run into each other
		if (c.kind == PFS::WALL && !c.trunk && c.fuel != PFS::BRUSH)
		{
			s.mult = 1.5f;
			// A wall burns from its foot: the fire of the burning cell below reaches up over this one. (A fire of its own
			// up here stood in the air in front of the wall, on its box's flat bottom.)
			const uint32_t below = field.wallAt(c.kx, c.ky - 1, c.kz, c.n,&c);
			if (below != PFS::Field::NO_CELL && below < shown.size() && shown[below].fire) return false;
		}
		return true;
	}
	s.mult = 1.45f;
	if (s.cls < 3) return true; // low and mid flames: every cell carries its own, they overlap into one line
	const float S = field.cellSize;
	if (!((c.kx + c.kz) & 1))
	{
		s.mult = 1.7f;
		return true;
	}
	static const int nb[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
	for (int k = 0; k < 4; ++k)
	{
		const uint32_t j = field.groundAt(float(c.kx + nb[k][0]) * S, c.p.y, float(c.kz + nb[k][1]) * S);
		if (j != PFS::Field::NO_CELL && j < shown.size() && shown[j].fire && shown[j].cls >= 2) return false;
	}
	return true;
}
// the cells whose state changed get, change or lose their picture
void spreadShow(const PFS::Weather& wx)
{
	if (shown.size() < field.cells.size()) shown.resize(field.cells.size());
	spreadPictures = 0;
	for (const Fire& f : fires)
		if (f.cell) ++spreadPictures;
	for (u32 i : field.changed)
	{
		const PFS::Cell& c = field.cells[i];
		Shown& s = shown[i];
		switch (c.state)
		{
		case PFS::CATCHING:
		case PFS::BURNING:
			if (c.state == PFS::CATCHING && !s.fire) s.caught = field.clock; // firerework: its flame grows from here
			if (!s.foot && wantsFoot(c)) s.foot = startFootPicture(i, c, c.state == PFS::CATCHING ? PFS::Field::catchRamp(c.fuel) : .5f); // fencefoot
			if (s.fire) break;
			if (c.source && c.kind==PFS::GROUND && c.fuel == PFS::NOTHING && !spreadSources) break; // the bottle's own pictures show that fuel
			s.cls = classOf(_max(c.size, c.state == PFS::CATCHING ? PFS::rule(c.fuel).size * (.45f + .55f * c.amount) * .7f : c.size));
			if (!wantsPicture(c, s)) break;
			s.off = 0.f;
			s.fire = startPicture(i, c, s.cls, c.state == PFS::CATCHING ? PFS::Field::catchRamp(c.fuel) : .5f, wx);
			if (s.fire) ++spreadPictures;
			break;
		case PFS::DYING:
			// firerework: the picture stays with its dying cell and shrinks with it (spreadPicture, pictureGrowth); it ends
			// when the cell has burnt out (below)
			break;
		case PFS::BURNT:
			if (s.fire) endPicture(s.fire, 1.f);
			s.fire = 0;
			if (s.foot) { endPicture(s.foot, 1.f); s.foot = 0; } // fencefoot
			break;
		default:
			break;
		}
	}
}
// a cell whose flames have outgrown or shrunk out of their take changes to the fitting one, one picture fading into the other
void spreadRetake(float dt, const PFS::Weather& wx)
{
	for (u32 i : field.burningCells())
	{
		if (i >= shown.size()) continue;
		Shown& s = shown[i];
		const PFS::Cell& c = field.cells[i];
		if (c.state != PFS::BURNING) continue;
		// fencefoot: the foot fire belongs to the lowest burning cell of its column - handed up when that cell burns out,
		// down when a cell below catches (looked at twice a second)
		if (c.kind == PFS::WALL && !c.trunk && c.fuel != PFS::BRUSH)
		{
			s.footLook += dt;
			if (s.footLook > .5f)
			{
				s.footLook = 0.f;
				const bool want = wantsFoot(c);
				if (want && !s.foot) s.foot = startFootPicture(i, c, 1.2f);
				else if (!want && s.foot) { endPicture(s.foot, 1.5f); s.foot = 0; }
			}
		}
		const int cls = classOf(c.size);
		if (!s.fire)
		{
			// a cell whose flames a bigger neighbour was showing gets its own again when that neighbour is gone
			if (c.source && c.kind==PFS::GROUND && c.fuel == PFS::NOTHING && !spreadSources) continue;
			s.off += dt;
			if (s.off < 1.f) continue;
			s.off = 0.f;
			s.cls = cls;
			if (wantsPicture(c, s))
			{
				s.fire = startPicture(i, c, cls, 1.2f, wx);
				if (s.fire) ++spreadPictures;
			}
			continue;
		}
		// now and then: does a bigger neighbour show this cell's flames by now?
		s.look += dt;
		if (s.look > 2.5f)
		{
			s.look = 0.f;
			const float keep = s.mult;
			if (!wantsPicture(c, s))
			{
				endPicture(s.fire, 1.5f);
				s.fire = 0;
				s.off = 0.f;
				continue;
			}
			s.mult = keep;
		}
		if (cls == s.cls || (c.kind==PFS::WALL && c.fuel==PFS::WOOD))
		{
			s.off = 0.f;
			continue;
		}
		s.off += dt;
		if (s.off < 1.5f) continue;
		endPicture(s.fire, 1.2f);
		s.cls = cls;
		s.off = 0.f;
		s.fire = wantsPicture(c, s) ? startPicture(i, c, cls, 1.2f, wx) : 0;
	}
}
bool spreadIgnite(const Fvector& p, float radius, float seconds, bool showSources, const PFS::Site* face=nullptr,bool floor=true)
{
	if (!ready() || !g_pGameLevel) return false;
	field.speed = ps_painter_fire_spread_speed;
	field.burnK = ps_painter_fire_burn_time;
	field.spreadFor = ps_painter_fire_spread_seconds;
	field.reach = ps_painter_fire_spread_radius;
	field.maxBurning = 240; // as many as are drawn: the rest waits its turn instead of burning unseen
	if (field.idle()) spreadSources = false;
	spreadSources = spreadSources || showSources;
	const PFS::V3 at = {p.x, p.y, p.z};
	const bool surface=face && field.igniteWall(*face,seconds);
    const bool ground=floor && field.ignite(world,at,radius,seconds,face);
    if (!ground && !surface) return false;
	spreadShow(weather());
	return true;
}
// A fire is asked for from the game's side (a bottle breaks, a thermite grenade goes off) and started here, in the
// renderer's own frame step, whatever thread asked.
struct Asked
{
	Fvector p;
	float radius, seconds;
	bool show;
    unsigned stamp;
    bool wall=false, floor=true;
    PFS::Site face{};
};
xr_vector<Asked> asked;
std::mutex askedLock;
bool spreadAsk(const Fvector& p, float radius, float seconds, bool show,const PFS::Site* wall=nullptr,bool floor=true)
{
	std::lock_guard<std::mutex> hold(askedLock);
	const unsigned stamp=PainterFireMasterStamp();
	if (!(stamp&1u) || asked.size() >= 64) return false;
	asked.push_back({p, radius, seconds, show, stamp, wall!=nullptr, floor, wall ? *wall : PFS::Site{}});
	return true;
}
// Where the spreading fire burns, for the game's side (who has to run from it): a copy made a few times a second, so the
// game may ask from whatever thread it likes.
xr_vector<Fvector> nearSnap;
unsigned nearStamp=0;
std::mutex nearLock;
void nearRefresh()
{
	static float last = -10.f;
	if (_abs(Device.fTimeGlobal - last) < .25f) return;
	last = Device.fTimeGlobal;
	static xr_vector<Fvector> next;
	next.clear();
	const std::vector<uint32_t>& on = field.burningCells();
	const size_t stride = on.size() > 800 ? on.size() / 800 + 1 : 1;
	for (size_t a = 0; a < on.size(); a += stride)
	{
		const PFS::Cell& c = field.cells[on[a]];
		Fvector p = {c.p.x, c.p.y, c.p.z};
		next.push_back(p);
	}
	std::lock_guard<std::mutex> hold(nearLock);
	nearSnap.swap(next);nearStamp=appliedFireMaster;
}
// A small flame is held to the ground here (a match, a lighter): it catches if it can burn and is not too wet.
bool spreadLight(const Fvector& p)
{
	if (!ready() || !g_pGameLevel) return false;
	field.speed = ps_painter_fire_spread_speed;
	field.burnK = ps_painter_fire_burn_time;
	field.spreadFor = ps_painter_fire_spread_seconds;
	field.reach = ps_painter_fire_spread_radius;
	field.maxBurning = 240;
	if (field.idle()) spreadSources = false;
	const PFS::V3 at = {p.x, p.y, p.z};
	const PFS::Weather wx = weather();
	if (field.light(world, at, wx) != 2) return false;
	spreadShow(wx);
	return true;
}
void spreadUpdate()
{
	spreadRays = 0;
	nearRefresh();
	leftoversRegrow();
	bareUpdate();
	treeFiresUpdate();
	{
		static xr_vector<Asked> now;
		now.clear();
		{
			std::lock_guard<std::mutex> hold(askedLock);
			now.swap(asked);
		}
		for (const Asked& a : now)
			if (a.radius < 0.f)
				spreadLight(a.p); // (a radius below 0: a small flame held to the ground, nothing thrown)
			else
				spreadIgnite(a.p, a.radius, a.seconds, a.show, a.wall ? &a.face : nullptr,a.floor);
	}
	smokeAirUpdate();
	puffsUpdate();
	if (field.idle())
	{
		plumesUpdate();
		bool lit = false;
		for (const SpreadLight& l : spreadLights) lit = lit || l.used;
		if (lit) spreadLightsFrame(Device.fTimeDelta);
		return;
	}
	if (!g_pGameLevel || Device.Paused()) return;
	const float dt = _min(Device.fTimeDelta, .1f);
	const PFS::Weather wx = weather();
	field.speed = ps_painter_fire_spread_speed;
	field.burnK = ps_painter_fire_burn_time;
    field.reach=ps_painter_fire_spread_radius;
	field.update(dt, world, wx, 6);
	spreadShow(wx);
	spreadRetake(dt, wx);
	leftoversUpdate(dt);
	spreadLightsFrame(dt);
	plumesUpdate();
}
} // namespace

// A bottle break carries its flight direction when known. The fuel pool and flames spread forward from the break.
// Recover the struck upright face before asking where fuel reached the floor.
bool bottleFace(const Fvector& position, const Fvector& flew, const Fvector* witness, PFS::Site& face)
{
    if (!g_pGameLevel) return false;
    const Fvector q = witness ? *witness : position;
    if (!_valid(q)) return false;
    const float len = _valid(flew) ? flew.magnitude() : 0.f;
    if (len > .01f)
    {
        Fvector dir, from;
        dir.div(flew, len);
        // The addon's stored witness is the actual ray hit. Older callers only have its last origin.
        const float behind = witness ? .06f : .16f;
        from.mad(q, dir, -behind);
        if (world.wall({from.x,from.y,from.z}, {dir.x,dir.y,dir.z},
                       witness ? .13f : 1.18f, face)) return true;
        if (witness) return false; // an identified floor or slope keeps the ordinary bottle path
    }
    // A missing direction must not attach an ordinary ground impact to a nearby post.
    PFS::Site support;
    if (world.ground({q.x,q.y,q.z}, .06f,.20f,support)) return false;
    float best = .30f;
    bool found = false;
    for (int i=0; i<8; ++i)
    {
        const float angle = float(i)*PI_MUL_2/8.f;
        const PFS::V3 dir={_cos(angle),0.f,_sin(angle)};
        PFS::Site candidate;
        if (!world.wall({q.x,q.y,q.z},dir,best,candidate)) continue;
        const float dx=candidate.p.x-q.x, dz=candidate.p.z-q.z;
        best=_sqrt(dx*dx+dz*dz);
        face=candidate;
        found=true;
    }
    return found;
}

// Preserve the original lift, downward range and arithmetic on ordinary supported terrain.
// Inspect the ray's existing triangle directly so a verified floor needs no second cast.
bool bottleGroundNear(const Fvector& at,Fvector& result)
{
    const Fvector up={0.f,1.f,0.f}, down={0.f,-1.f,0.f};
    collide::rq_result hit;
    float lift=1.5f;
    if(g_pGameLevel->ObjectSpace.RayPick(at,up,1.5f,collide::rqtStatic,hit,nullptr))lift=_max(hit.range-.05f,0.f);
    Fvector from=at; from.y+=lift;
    if(!g_pGameLevel->ObjectSpace.RayPick(from,down,3.5f+lift,collide::rqtStatic,hit,nullptr) || hit.O || hit.element<0)return false;
    result=at; result.y=from.y-hit.range;
    return fireSupport(hit,result,triNormal(hit,down));
}

// Only an actual, standable collision surface supports a flat fire. Thin leaves and trunk
// collision are not ground. Search a bounded path; a missing floor leaves the upright fire alone.
bool bottleFloor(const Fvector& at, float reach, Fvector& floor)
{
    const Fvector down={0.f,-1.f,0.f};
    Fvector from=at;
    from.y+=.08f;
    float left=reach+.08f;
    for (int i=0; i<12 && left>.01f; ++i)
    {
        collide::rq_result hit;
        float d=0.f;
        if (!pick(from,down,left,hit,d)) break;
        Fvector p=from;p.y-=d;
        if(fireSupport(hit,p,triNormal(hit,down)))
        {
            floor=from;
            floor.y-=d;
            return true;
        }
        from.y-=d+.025f;
        left-=d+.025f;
    }
    return false;
}

void bottleFacePicture(const PFS::Site& s,float seconds)
{
    Fvector pos={s.p.x,s.p.y-.24f,s.p.z}, n={s.n.x,0.f,s.n.z};
    const bool wood=s.fuel==PFS::WOOD;
    if(!wood)pos.mad(n,.04f);
    if(Fire* f=add(findTake(wood?"body":"grass_mid"),pos,false,wood))
    {
        f->life=seconds; f->fadeIn=.2f; f->fadeOut=1.f; f->scale=.72f;
        f->lightK=1.f; f->air=.28f;
        if(wood)woodFlameContact(s,*f,.24f);
        else f->normal=n;
    }
}


bool BottleImpact(const Fvector& position, float seconds, u32 seed, const Fvector& flew,const Fvector* witness)
{
	if (!ready() || !g_pGameLevel || !_valid(position)) return false;
	xr_vector<int> bursts, skids;
	for (u32 i = 0; i < takes.size(); ++i)
	{
		if (!strncmp(takes[i].name.c_str(), "burst_", 6)) bursts.push_back(int(i));
		else if (!strncmp(takes[i].name.c_str(), "gburst_", 7)) skids.push_back(int(i));
	}
	if (bursts.empty()) return false;
	Rng r{seed * 2654435761u + 12345u};
	r.next();
	Fvector ground = position;
    PFS::Site face{};
    const bool wall=bottleFace(position,flew,witness,face);
    bool hasGround=false;
    if(wall)
    {
        const Fvector drip={face.p.x+face.n.x*.31f,face.p.y+.02f,face.p.z+face.n.z*.31f};
        hasGround=bottleFloor(drip,32.f,ground);
    }
    else
    {
        hasGround=bottleGroundNear(position,ground);
        if(!hasGround) hasGround=bottleFloor(position,32.f,ground);
    }
	seconds = clampr(seconds * ps_painter_fire_pool_time, 4.f, 720.f);
    if (wall && !ps_painter_fire_spread_on) bottleFacePicture(face,seconds);
    if (!hasGround)
    {
        if(wall && ps_painter_fire_spread_on && !spreadAsk(position,0.f,_min(seconds,25.f*ps_painter_fire_pool_time),false,&face,false))
            bottleFacePicture(face,seconds);
        return true;
    }
    const float top = ceilingOver(ground); // in a room its fire ends at the ceiling
	Fvector way = {0.f, 0.f, 0.f};
	bool skid = false;
	const float len = _valid(flew) ? flew.magnitude() : 0.f;
	const float flat = len > .01f ? _sqrt(flew.x * flew.x + flew.z * flew.z) : 0.f;
	if (!wall && !skids.empty() && flat > .01f && flat > .55f * len && position.y - ground.y < .8f)
	{
		way.set(flew.x / flat, 0.f, flew.z / flat);
		// a wall just ahead: it broke against that
		PFS::Site s;
		const PFS::V3 from = {position.x, _max(position.y, ground.y + .3f), position.z};
		const PFS::V3 dir = {way.x, 0.f, way.z};
		skid = !world.wall(from, dir, 1.f, s);
	}
	// where fuel lands that far ahead of the break (at the break when it does not skid, or over an edge)
	auto ahead = [&](float d) {
		Fvector p = ground;
		if (!skid) return p;
		p.mad(way, d);
		p.y = ground.y + .3f;
        if(!bottleGroundNear(p,p))return ground;
		if (_abs(p.y - ground.y) > 1.2f) p = ground;
		return p;
	};
	// Choose a fuel burst and run the flames along the streams from the bottle.
	const xr_vector<int>& pick = skid ? skids : bursts;
	const u64 bottleGroup = newFrontGroup();
	if (Fire* f = add(pick[u32(r.next() * 7919.f) % pick.size()], ground))
	{
		{ f->frontGroup = bottleGroup; f->frontGround = true; }
		f->mirror = r.next() > .5f;
		const float spin = r.next() * PI_MUL_2;
		f->yaw = skid ? atan2f(-way.x, way.z) : spin; // (a box's depth runs along (-sin yaw, 0, cos yaw))
		f->fadeOut = .3f;
		f->lightK = 1.f;
		f->top = top;
	}
	// the pool it leaves burning, and small flames where the fuel landed around it (ahead of it when it skids)
	const int pool = findTake("pool");
	if (Fire* f = add(pool, ahead(1.3f)))
	{
		{ f->frontGroup = bottleGroup; f->frontGround = true; }
		f->delay = .45f;
		f->life = seconds;
		f->fadeIn = .6f;
		f->fadeOut = 2.5f;
		f->phase = r.next() * 60.f;
		f->mirror = r.next() > .5f;
		f->yaw = r.next() * PI_MUL_2;
		f->lightK = 1.f;
		f->top = top;
	}
	const int spot = findTake("spot");
	if (spot >= 0)
		for (int i = 0; i < 5; ++i)
		{
			const float a = skid ? atan2f(way.z, way.x) + (r.next() - .5f) * 1.3f : r.next() * PI_MUL_2;
			const float d = skid ? 1.f + 3.2f * r.next() : .7f + 1.7f * r.next();
			Fvector p = {ground.x + d * _cos(a), ground.y, ground.z + d * _sin(a)};
            if(wall)
            {
                if(!face.trunk && (p.x-face.p.x)*face.n.x+(p.z-face.p.z)*face.n.z<0.f) continue;
                Fvector from=p; from.y+=.28f;
                if(!bottleFloor(from,1.6f,p)) continue;
            }
            else
            {
                if(!bottleGroundNear(p,p))continue;
            }
			if (_abs(p.y - ground.y) > 1.2f) continue; // over an edge or under a roof: leave that one out
			if (Fire* f = add(spot, p))
			{
                // Keep a lobe across a solid divider independent of this fuel front.
                Fvector line;
                line.sub(p,ground);
                const float span = line.magnitude();
                PFS::Site divider;
                if (span < .02f || !world.wall({ground.x,ground.y+.30f,ground.z},
                        {line.x/span,line.y/span,line.z/span},span,divider)) { f->frontGroup = bottleGroup; f->frontGround = true; }
				f->delay = .35f + .5f * r.next();
				f->life = seconds * (.35f + .5f * r.next());
				f->fadeIn = .4f;
				f->fadeOut = 1.5f;
				f->phase = r.next() * 60.f;
				f->mirror = r.next() > .5f;
				f->yaw = r.next() * PI_MUL_2;
				f->scale = .8f + .6f * r.next();
				f->top = top;
			}
		}
	// Ignite the surfaces reached by the burning fuel.
	if (ps_painter_fire_spread_on && !spreadAsk(ahead(1.8f),skid?2.8f:2.2f,_min(seconds,25.f*ps_painter_fire_pool_time),false,wall?&face:nullptr) && wall)
        bottleFacePicture(face,seconds);
	return true;
}
bool BottleThrown(const Fvector& position, float seconds, u32 seed, const Fvector& flew)
{
    return BottleImpact(position,seconds,seed,flew,nullptr);
}
bool Bottle(const Fvector& position, float seconds, u32 seed)
{
	return BottleThrown(position, seconds, seed, Fvector().set(0.f, 0.f, 0.f));
}
bool LeavesGone(const void* visual)
{
    if(!ps_painter_fire)return false;
    return false;
}
bool LodGone(const void* visual)
{
    if(!ps_painter_fire)return false;
    return false;
}
float TreeBurnAmount(const void* visual,bool* separateWood,float* thermal)
{
    if(thermal)*thermal=0.f;
    if(separateWood) *separateWood=false;
    if (!ps_painter_fire || !ps_painter_fire_burnt || !visual || !bareCount.load(std::memory_order_relaxed)) return 0.f;
    auto s=std::atomic_load_explicit(&bareNow,std::memory_order_acquire);
    if(!s)return 0.f;
    auto sample=[&](PainterTreeDates::Span span) {
        if(thermal)*thermal=PainterTreeDates::thermal(span,Device.fTimeGlobal);
        return PainterTreeDates::progress(span,Device.fTimeGlobal);
    };
    const auto leaf=s->leaves.find(visual);
    if(leaf!=s->leaves.end()){
        if(separateWood) *separateWood=s->independentLeaves.count(visual)!=0;
        return sample(leaf->second);
    }
    const auto lod=s->lods.find(visual);
    if(lod!=s->lods.end())return sample(lod->second);
    const auto wood=s->wood.find(visual);
    return wood==s->wood.end()?0.f:sample(wood->second);
}
bool TreeWoodBurn(const void* visual,float& amount,float& heat)
{
    amount=heat=0.f;
    if(!ps_painter_fire || !ps_painter_fire_burnt || !visual || !bareCount.load(std::memory_order_relaxed))return false;
    auto s=std::atomic_load_explicit(&bareNow,std::memory_order_acquire);
    if(!s)return false;
    const auto it=s->wood.find(visual);
    if(it==s->wood.end())return false;
    amount=PainterTreeDates::progress(it->second,Device.fTimeGlobal);
    heat=PainterTreeDates::heat(it->second,Device.fTimeGlobal);
    return true;
}
void Test(const char* take, float seconds)
{
	if (!ready())
	{
		Msg("! [painter_fire] nothing to show: painter_fire is 0 or the fire files are missing");
		return;
	}
	const int k = take && *take ? findTake(take) : -1;
	if (k < 0)
	{
		Msg("* [painter_fire] painter_fire_test <take> [seconds] [smoke 0..2] [smoke colour 0 black..1 white]; takes:");
		for (const Take& t : takes)
			Msg("  %s  (%u frames, %.1f x %.1f x %.1f m%s%s%s)", t.name.c_str(), t.frames, t.width, t.height, t.deep, t.loop ? ", loop" : "",
				t.smoke ? ", smoke" : "", t.state == 2 ? "" : t.state < 0 ? ", NOT USABLE" : ", still being read");
		return;
	}
	Fvector dir = Device.vCameraDirection;
	dir.y = 0.f;
	if (dir.square_magnitude() < 1e-4f) dir.set(0.f, 0.f, 1.f);
	dir.normalize();
	Fvector p;
	p.mad(Device.vCameraPosition, dir, _max(4.f, takes[k].width * 1.2f));
	if(!firePictureFloor(p,1.5f,3.5f,p.y))return;
	if (Fire* f = add(k, p))
	{
		f->life = clampr(seconds, 1.f, 600.f);
		f->fadeIn = takes[k].loop ? .5f : 0.f;
		f->fadeOut = takes[k].loop ? 1.5f : .3f;
		f->lightK = 1.f;
		f->smoke = takes[k].smoke ? 1.f : 0.f;
		f->albedo = .6f;
	}
}
// the console command's extra numbers for the fire it just made
void TestSmoke(float amount, float albedo)
{
	if (fires.empty()) return;
	fires.back().smoke = clampr(amount, 0.f, 3.f);
	fires.back().albedo = clampr(albedo, 0.f, 1.f);
}

namespace
{
// the ground straight below a point (static level geometry)
float groundBelow(const Fvector& p)
{
	Fvector down = {0.f, -1.f, 0.f};
	collide::rq_result hit;
	if (g_pGameLevel && g_pGameLevel->ObjectSpace.RayPick(p, down, 15.f, collide::rqtStatic, hit, nullptr)) return p.y - hit.range;
	return p.y;
}
// a take that was baked with a wall in its box, on a wall of the level: foot = on the ground at the wall, normal = the way
// the wall faces
bool wallFire(const char* take, const Fvector& foot, const Fvector& normal, float seconds, float scale)
{
	const int k = findTake(take);
	if (k < 0) return false;
	Fvector n = {normal.x, 0.f, normal.z};
	if (n.square_magnitude() < 1e-4f) return false;
	n.normalize();
	Fire* f = add(k, foot);
	if (!f) return false;
	string128 sideName;
	xr_sprintf(sideName, "%s_side", take);
	f->side = findTake(sideName);
	f->normal = n;
	f->life = clampr(seconds, 1.f, 600.f);
	f->fadeIn = .8f;
	f->fadeOut = 2.f;
	f->scale = clampr(scale, .25f, 4.f);
	f->phase = float(Device.dwTimeGlobal % 977u);
	f->lightK = 1.f;
	f->smoke = takes[k].smoke ? 1.f : 0.f;
	f->albedo = .4f;
	return true;
}
} // namespace

void TestWall(const char* take, float seconds, float scale)
{
	if (!ready() || !g_pGameLevel)
	{
		Msg("! [painter_fire] nothing to show: painter_fire is 0 or the fire files are missing");
		return;
	}
	const Fvector dir = Device.vCameraDirection;
	collide::rq_result hit;
	if (!g_pGameLevel->ObjectSpace.RayPick(Device.vCameraPosition, dir, 60.f, collide::rqtStatic, hit, nullptr) || hit.O || hit.element < 0)
	{
		Msg("! [painter_fire] painter_fire_wall <take> [seconds] [scale]: look at a wall (within 60 m)");
		return;
	}
	const CDB::TRI& tri = g_pGameLevel->ObjectSpace.GetStaticTris()[hit.element];
	const Fvector* v = g_pGameLevel->ObjectSpace.GetStaticVerts();
	Fvector n;
	n.mknormal(v[tri.verts[0]], v[tri.verts[1]], v[tri.verts[2]]);
	if (n.dotproduct(dir) > 0.f) n.invert();
	if (_abs(n.y) > .6f)
	{
		Msg("! [painter_fire] that surface is not upright: look at a wall");
		return;
	}
	Fvector p, off;
	p.mad(Device.vCameraPosition, dir, hit.range);
	off.mad(p, n, .3f);
	p.y = groundBelow(off);
	if (!wallFire(take, p, n, seconds, scale))
		Msg("! [painter_fire] no take \"%s\" (painter_fire_test lists the takes; made for walls: door, eave)", take);
}

void TestBody(float seconds)
{
    if(!(PainterBodyMasterStamp()&1u))return;
	if (!ready() || !g_pGameLevel)
	{
		Msg("! [painter_fire] nothing to show: painter_fire is 0 or the fire files are missing");
		return;
	}
	collide::rq_result hit;
	if (!g_pGameLevel->ObjectSpace.RayPick(Device.vCameraPosition, Device.vCameraDirection, 60.f, collide::rqtBoth, hit,
			g_pGameLevel->CurrentViewEntity()) || !hit.O)
	{
		Msg("! [painter_fire] painter_fire_body [seconds]: look at a person (within 60 m)");
		return;
	}
	if (!Body(hit.O->ID(), clampr(seconds, 1.f, 600.f))) Msg("! [painter_fire] the take \"body\" is missing");
}

// A body burns for at least `seconds` from now; whoever knows it burns calls again while it does (the fire dies down when
// the calls stop). Before this a burning body showed another mod's own effect; the baked body fire was only on a console
// command.
bool Body(u16 id, float seconds)
{
	if (!(PainterBodyMasterStamp()&1u) || !ready() || !g_pGameLevel || takeBody < 0) return false;
	CObject* o = g_pGameLevel->Objects.net_Find(id);
	if (!o || o->getDestroy() || !o->Visual()) return false;
	std::lock_guard<std::mutex> hold(bodyLock);
	const unsigned stamp=PainterBodyMasterStamp();
    if (!(PainterFireMasterStamp()&1u) || !(stamp&1u))return false;
	if (bodyAsked.size() < 256) bodyAsked.push_back({id, clampr(seconds, .5f, 600.f),stamp});
	return true;
}

bool Ignite(const Fvector& position, float radius, float seconds)
{
	if (!ps_painter_fire_spread_on || !ps_painter_fire) return false;
	return spreadAsk(position, clampr(radius, .5f, 8.f), clampr(seconds, 1.f, 120.f), true);
}
// A small flame is held to the ground at a place (a match, a lighter). It is started in the renderer's own frame step;
// whether it caught shows a moment later (Near).
bool Light(const Fvector& position)
{
	if (!ps_painter_fire_spread_on || !ps_painter_fire) return false;
	return spreadAsk(position, -1.f, 0.f, false);
}
// The nearest place where the spreading fire burns, within `radius` of a position. False = none.
bool Near(const Fvector& position, float radius, Fvector& nearest)
{
	if (!ps_painter_fire) return false;
	std::lock_guard<std::mutex> hold(nearLock);
    if(nearStamp!=PainterFireMasterStamp())return false;
	float best = radius * radius;
	bool any = false;
	for (const Fvector& c : nearSnap)
	{
		const float d = c.distance_to_sqr(position);
		if (d < best)
		{
			best = d;
			nearest = c;
			any = true;
		}
	}
	return any;
}
void Spread(float seconds)
{
	if (!ready() || !g_pGameLevel)
	{
		Msg("! [painter_fire] nothing to show: painter_fire is 0 or the fire files are missing");
		return;
	}
	collide::rq_result hit;
	if (!g_pGameLevel->ObjectSpace.RayPick(Device.vCameraPosition, Device.vCameraDirection, 80.f, collide::rqtStatic, hit, nullptr))
	{
		Msg("! [painter_fire] painter_fire_spread [seconds]: look at the ground (within 80 m)");
		return;
	}
	Fvector p;
	p.mad(Device.vCameraPosition, Device.vCameraDirection, hit.range);
	p.y += .2f;
	if (!spreadIgnite(p, 1.3f, clampr(seconds, 1.f, 120.f), true))
		Msg("! [painter_fire] no ground found there");
	else
		SpreadInfo();
}
void SpreadInfo()
{
	u32 byState[5] = {0, 0, 0, 0, 0}, byFuel[PFS::FUEL_COUNT] = {0, 0, 0, 0, 0, 0};
	for (const PFS::Cell& c : field.cells)
	{
		++byState[c.state];
		if (c.state == PFS::CATCHING || c.state == PFS::BURNING) ++byFuel[c.fuel];
	}
	const PFS::Weather w = weather();
	float raw = 0.f;
	if (g_pGamePersistent && g_pGamePersistent->Environment().CurrentEnv) raw = g_pGamePersistent->Environment().CurrentEnv->wind_velocity;
	Msg("* [painter_fire] spread: %u cells known; catching %u, burning %u, dying %u, burnt %u", u32(field.cells.size()), byState[1],
		byState[2], byState[3], byState[4]);
	Msg("* [painter_fire] burning by fuel: grass %u, brush %u, wood %u, cloth %u, room contents %u, thrown fuel only %u", byFuel[1],
		byFuel[2], byFuel[3], byFuel[4], byFuel[5], byFuel[0]);
	Msg("* [painter_fire] wind %.1f m/s (level value %.0f) to x %.2f z %.2f; wetness %.2f; rain %.2f; rays this frame %u; pictures %u",
		_sqrt(w.windX * w.windX + w.windZ * w.windZ), raw, w.windX, w.windZ, w.wet, w.rain, spreadRays, u32(fires.size()));
}
void SpreadStop() { spreadClear(); }

namespace
{
// the bodies that were reported burning since the last frame: their fires start, or burn on
void bodyUpdate()
{
    static unsigned previous=0;
    const unsigned stamp=PainterBodyMasterStamp();
    if((previous>>1)!=(stamp>>1)){
        fires.erase(std::remove_if(fires.begin(),fires.end(),[](const Fire& f){return f.follow!=u16(-1);}),fires.end());
        std::lock_guard<std::mutex> hold(bodyLock);
        bodyAsked.erase(std::remove_if(bodyAsked.begin(),bodyAsked.end(),[stamp](const BodyAsk& a){return a.stamp!=stamp;}),bodyAsked.end());
    }
    previous=stamp;
    if(!(stamp&1u))return;
    static xr_vector<BodyAsk> now;
	now.clear();
	{
		std::lock_guard<std::mutex> hold(bodyLock);
		now.swap(bodyAsked);
	}
	const u32 tick = Device.dwTimeGlobal;
	for (const BodyAsk& a : now)
	{
		bool have = false;
		for (Fire& f : fires)
			if (f.follow == a.id)
			{
				have = true;
				const float t = float(tick - f.born) * .001f - f.delay;
				f.life = _max(f.life, t + a.seconds + f.fadeOut);
			}
		if (have) continue;
		CObject* o = g_pGameLevel ? g_pGameLevel->Objects.net_Find(a.id) : nullptr;
		if (!o || o->getDestroy() || takeBody < 0) continue;
		// The fire has the body's shape: small fires on its bones, each going where its bone goes. (One column of flame
		// around the whole body read as a burning post, not as somebody on fire.) The player's own fire stays the low one.
		const int spotTake = findTake("spot"); // ("small" is a word the Windows headers have taken)
		IKinematics* K = o->Visual() ? o->Visual()->dcast_PKinematics() : nullptr;
		const bool self = g_pGameLevel && g_pGameLevel->CurrentEntity() == o;
		struct Part
		{
			u16 bone;
			float size;
		};
		Part parts[12];
		u32 count = 0;
		if (K && spotTake >= 0 && !self)
		{
			static const struct
			{
				LPCSTR name;
				float size;
			} wanted[12] = {{"bip01_spine2", 1.15f}, {"bip01_head", .8f}, {"bip01_spine", .95f}, {"bip01_pelvis", .85f},
				{"bip01_l_upperarm", .6f}, {"bip01_r_upperarm", .6f}, {"bip01_l_forearm", .5f}, {"bip01_r_forearm", .5f},
				{"bip01_l_thigh", .7f}, {"bip01_r_thigh", .7f}, {"bip01_l_calf", .55f}, {"bip01_r_calf", .55f}};
			for (const auto& w : wanted)
			{
				const u16 id = K->LL_BoneID(w.name);
				if (id != u16(-1) && id < K->LL_BoneCount()) parts[count++] = {id, w.size};
			}
			if (count < 4)
			{
				// a creature with other bone names: up to ten of its bones, spread over its skeleton
				count = 0;
				const u32 bones = K->LL_BoneCount(), take = _min(bones > 1 ? bones - 1 : 0u, 10u);
				for (u32 i = 0; i < take; ++i) parts[count++] = {u16(1 + i * (bones - 1) / take), i == 0 ? 1.1f : .8f};
			}
		}
		if (!count)
		{
			Fire* f = add(takeBody, o->Position());
			if (!f) continue;
			f->follow = a.id;
			f->fadeIn = .5f;
			f->fadeOut = 1.5f;
			f->life = a.seconds + f->fadeOut;
			f->phase = float((tick + u32(a.id) * 131u) % 977u);
			f->lightK = 1.f;
			f->smoke = takes[takeBody].smoke ? 1.f : 0.f;
			f->albedo = .2f;
			continue;
		}
		for (u32 i = 0; i < count; ++i)
		{
			Fire* f = add(spotTake, o->Position());
			if (!f) break;
			f->follow = a.id;
			f->bone = parts[i].bone;
            f->frontGround = true;
			f->boneSize = parts[i].size;
			f->air = .3f;
			f->scale = parts[i].size;
			f->fadeIn = .35f + .05f * float(i);
			f->fadeOut = 1.5f;
			f->life = a.seconds + f->fadeOut;
			f->phase = float((tick + u32(a.id) * 131u + i * 97u) % 977u);
			f->yaw = float((u32(a.id) * 7u + i * 53u) % 628u) * .01f;
			f->lightK = i == 0 ? 1.f : 0.f;
			f->smoke = takes[spotTake].smoke ? (i < 4 ? 1.f : .5f) : 0.f;
			f->albedo = .2f;
		}
	}
}
// a fire on a body goes where the body goes and is as big as it: standing it wraps the body, on a body that lies (or a low
// creature) it burns up from it
// Reuse the actual static support face while feet stay over it. Interpolation
// follows a pitched floor. Leaving its boundary immediately drops the pool.
bool bodyFloorFace(int id,const Fvector& foot,float& height)
{
    if(id<0 || !g_pGameLevel)return false;
    const CDB::TRI& t=g_pGameLevel->ObjectSpace.GetStaticTris()[id];
    const Fvector* vertices=g_pGameLevel->ObjectSpace.GetStaticVerts();
    const Fvector& a=vertices[t.verts[0]], &b=vertices[t.verts[1]], &c=vertices[t.verts[2]];
    const float bx=b.x-a.x,bz=b.z-a.z,cx=c.x-a.x,cz=c.z-a.z;
    const float det=bx*cz-bz*cx;
    if(_abs(det)<1.e-8f)return false;
    const float dx=foot.x-a.x,dz=foot.z-a.z;
    const float u=(dx*cz-dz*cx)/det,v=(bx*dz-bz*dx)/det;
    if(u<0.f || v<0.f || u+v>1.f)return false;
    height=a.y+u*(b.y-a.y)+v*(c.y-a.y);
    return height>=foot.y-.21f && height<=foot.y+.11f;
}
bool bodyFloor(Fire& f,float& height)
{
    if(bodyFloorFace(f.bodyFloorFaceId,f.pos,height))return true;
    const u32 now=Device.dwTimeGlobal;
    if(f.bodyFloorFaceId!=-2 && now-f.bodyFloorProbeAt<125u)return false;
    f.bodyFloorProbeAt=now;f.bodyFloorFaceId=-1;
    Fvector from=f.pos,p,n;from.y+=.11f;collide::rq_result hit;
    if(!fireSupportBelow(from,.32f,hit,p,n))return false;
    f.bodyFloorFaceId=hit.element;height=p.y;return true;
}


void followBody(Fire& f, CObject& o)
{
	if (f.bone != u16(-1))
	{
		// a fire on a bone: where the bone is now, its foot a little under it; bigger creatures carry bigger fires
		IKinematics* K = o.Visual() ? o.Visual()->dcast_PKinematics() : nullptr;
		if (!K || f.bone >= K->LL_BoneCount()) return;
		Fvector w;
		o.XFORM().transform_tiny(w, K->LL_GetTransform_safed(f.bone).c);
		if (!(w.square_magnitude() < 1e12f)) return;
		float big = 1.f;
		if (o.Visual())
		{
			const Fbox& b = o.Visual()->getVisData().box;
			big = clampr(_max(b.max.y - b.min.y, _max(b.max.x - b.min.x, b.max.z - b.min.z)) / 1.8f, .6f, 2.2f);
		}
		f.scale = f.boneSize * big;
		f.pos.set(w.x, w.y - .22f * f.scale, w.z);
		return;
	}
	Fvector lo = o.Position(), hi = o.Position();
	hi.y += 1.8f;
	if (o.Visual())
	{
		const Fbox& b = o.Visual()->getVisData().box;
		const Fmatrix& M = o.XFORM();
		lo.set(1e9f, 1e9f, 1e9f);
		hi.set(-1e9f, -1e9f, -1e9f);
		for (int c = 0; c < 8; ++c)
		{
			const Fvector p = {c & 1 ? b.max.x : b.min.x, c & 2 ? b.max.y : b.min.y, c & 4 ? b.max.z : b.min.z};
			Fvector w;
			M.transform_tiny(w, p);
			lo.min(w);
			hi.max(w);
		}
	}
	const float tall = hi.y - lo.y, wide = _max(hi.x - lo.x, hi.z - lo.z);
	if (!(tall > .05f) || !(tall < 20.f) || !(wide < 20.f)) return; // a box that makes no sense: the fire stays as it is
	f.pos.set(.5f * (lo.x + hi.x), lo.y, .5f * (lo.z + hi.z));
	// the player's own fire is the low one too: flames around the legs, not a column of fire in the eyes
	const bool lying = (tall < 1.05f && wide > tall) || (g_pGameLevel && g_pGameLevel->CurrentEntity() == &o);
	float supportY=0.f;
    const bool groundedLow=lying && takeBodyLow>=0 && bodyFloor(f,supportY);
    const bool airborneLow=lying && !groundedLow;
    if(groundedLow)f.pos.y=supportY;
    f.air=airborneLow?.38f:0.f;
    const bool ownView=g_pGameLevel && g_pGameLevel->CurrentEntity()==&o;
    f.bodyAirHeight=airborneLow?(ownView?.52f:clampr(tall+.25f,.45f,1.f)):0.f;
    const int want=groundedLow?takeBodyLow:takeBody;
	if (want >= 0) f.take = want;
	const float size = lying ? clampr(wide / 1.7f, .5f, 1.3f) * .8f : clampr(tall / 1.85f, .5f, 1.35f);
	f.scale += (size - f.scale) * _min(1.f, Device.fTimeDelta * 4.f);
	Fvector fw = o.XFORM().k;
	fw.y = 0.f;
	if (fw.square_magnitude() > 1e-4f)
	{
		f.facing.normalize(fw);
		f.yaw = atan2f(-f.facing.x, f.facing.z);
	}
}
} // namespace

// Shader programs made since the last frame call, for the stall recorder below: the renderer reports every one it makes
// (r4.cpp, CRender::shader_compile -> ShaderMade).
namespace
{
std::atomic<DWORD> stallThread{0};                        // the thread the frames run on
std::atomic<u64> shaderTicks{0}, shaderCompileTicks{0};   // on that thread: making in all, and of it compiling from source
std::atomic<u32> shaderCount{0}, shaderCompiled{0};
std::atomic<u64> shaderElsewhereTicks{0};                 // on other threads (they do not hold a frame up)
} // namespace
void ShaderMade(u64 ticks, u64 compileTicks)
{
    if(!ps_painter_fire)return;
	if (GetCurrentThreadId() != stallThread.load(std::memory_order_relaxed))
	{
		shaderElsewhereTicks.fetch_add(ticks, std::memory_order_relaxed);
		return;
	}
	shaderTicks.fetch_add(ticks, std::memory_order_relaxed);
	shaderCount.fetch_add(1, std::memory_order_relaxed);
	if (compileTicks)
	{
		shaderCompileTicks.fetch_add(compileTicks, std::memory_order_relaxed);
		shaderCompiled.fetch_add(1, std::memory_order_relaxed);
	}
}
void stallWatch()
{
	static bool started = false;
	static u64 frameStamp = 0;
	static u64 last[3] = {};
	stallThread.store(GetCurrentThreadId(), std::memory_order_relaxed);
	if (!ps_painter_stall_log)
	{
		started = false;
		return;
	}
	g_bEnableStatGather = TRUE;
	const u64 stamp = CPU::QPC();
	const u64 madeTicks = shaderTicks.exchange(0), compileTicks = shaderCompileTicks.exchange(0), elsewhereTicks = shaderElsewhereTicks.exchange(0);
	const u32 madeCount = shaderCount.exchange(0), compiledCount = shaderCompiled.exchange(0);
	CStats* st = Device.Statistic;
	if (!st)
	{
		started = false;
		return;
	}
	const u64 now[3] = {st->UpdateClient.accum, st->Physics.accum, st->AI_Think.accum};
	if (started && g_pGameLevel && Device.b_is_Active && !Device.Paused() && !Device.dwPrecacheFrame && !load_screen_renderer.IsActive())
	{
		const double toMs = 1000. / double(CPU::qpc_freq);
		const double ms = double(stamp - frameStamp) * toMs;
		if (ms >= 100.)
		{
			double part[3] = {-1., -1., -1.}; // before the render, the render, after it
			const CStatTimer& rt = st->RenderTOTAL_Real;
			if (rt.count == 1)
			{
				const double sinceStart = double(rt.T.GetElapsed_ticks()) * toMs, took = double(rt.accum) * toMs;
				if (took <= sinceStart + 1. && sinceStart <= ms + 1.)
				{
					part[0] = ms > sinceStart ? ms - sinceStart : 0.;
					part[1] = took;
					part[2] = sinceStart > took ? sinceStart - took : 0.;
				}
			}
			double t[3];
			for (int i = 0; i < 3; ++i)
			{
				t[i] = now[i] >= last[i] ? double(now[i] - last[i]) * toMs : -1.;
				if (t[i] > ms + 50.) t[i] = -1.; // a timer caught across the switch-on: not this frame's
			}
			Msg("[painter_stall] a frame took %.0f ms: before the render %.0f (game update, wound installs, particles), render %.0f, after the render %.0f (waiting for the helper threads, wound installs, window messages) | object updates %.0f, physics %.0f, ai %.0f | shaders made %u in %.0f ms, %u of them compiled from source in %.0f ms (other threads %.0f ms) | fire: %u pictures, %u burning cells, %u puffs, %u level questions",
				ms, part[0], part[1], part[2], t[0], t[1], t[2], madeCount, double(madeTicks) * toMs, compiledCount, double(compileTicks) * toMs,
				double(elsewhereTicks) * toMs, u32(fires.size()), u32(field.burning), u32(puffs.size()), spreadRays);
		}
	}
	for (int i = 0; i < 3; ++i) last[i] = now[i];
	frameStamp = stamp;
	started = true;
}
void Update()
{
    syncFireMaster();
    if(!ps_painter_fire)return;
    stallWatch();
	if (!loaded && ps_painter_fire && g_pGameLevel) load(); // the fire volumes are read in the background from a level's start
	volumesStep();
    // firetrack: how many fires get a lamp - the fire menu's Fire lights slider (console painter_fire_light_count 4..32)
    fireLightPlan.limit=unsigned(clampr(ps_painter_fire_light_count,1,int(PainterFireLighting::slots)));
    fireLightPlan.begin({Device.vCameraPosition.x, Device.vCameraPosition.y, Device.vCameraPosition.z});
    bodyUpdate();
    spreadUpdate();
    if (fires.empty()) { publishFireLights(); return; }
	const u32 now = Device.dwTimeGlobal;
	for (u32 i = 0; i < fires.size();)
	{
		Fire& f = fires[i];
		if (f.cell) spreadPicture(f);
		if (f.follow != u16(-1))
		{
			CObject* o = g_pGameLevel ? g_pGameLevel->Objects.net_Find(f.follow) : nullptr;
			if (o && !o->getDestroy())
				followBody(f, *o);
			else
			{
				// the body is gone: the fire dies down where it last was
				f.follow = u16(-1);
				f.life = _min(f.life, float(now - f.born) * .001f - f.delay + f.fadeOut);
			}
		}
		const Take& k = takes[f.take];
		if (k.state == 0 || k.state == 1) f.born = now; // its volume is still being read: its clock waits for it
		float frame = 0.f, fade = 0.f;
		bool over = false;
		const bool shows = state(f, k, now, frame, fade, over);
		if (over || now + 1000u < f.born) // over, or the clock was set back (a new level)
		{
			fires.erase(fires.begin() + i);
			continue;
		}
		if (f.lightK > 0.f && ps_painter_fire)
		{
			// Update light brightness, height and reach from the active flames.
			float strength = 0.f, height = .5f;
			if (shows && !k.light.empty())
			{
				const u32 first = _min(u32(frame), k.frames - 1);
				const u32 next = first + 1 < k.frames ? first + 1 : (k.loop ? 0 : first);
				const float blend = frame - float(first);
				const u32 end = u32(k.light.size()) - 1;
				const float a = k.light[_min(first, end)], b = k.light[_min(next, end)];
				const float energy = a + (b - a) * blend;
				strength = _min(_sqrt(_max(energy, 0.f) / 2.4f), 5.f) * fade * f.lightK * f.scale;
				const float low = first < k.lightH.size() ? k.lightH[first] * f.scale : height;
				const float high = next < k.lightH.size() ? k.lightH[next] * f.scale : height;
				height = low + (high - low) * blend;
			}
			if (strength > .02f)
			{
				Fvector p = f.pos;
				p.mad(f.normal, .5f * f.scale); // a fire on a wall lights from in front of it
				p.y += (f.bodyAirHeight>0.f?_min(height,.5f*f.bodyAirHeight):height)+.25f;
				if (f.top != 0.f && p.y > f.top - .3f) p.y = f.top - .3f; // in a room it stays under the ceiling
                auto s=PainterFireLighting::baked({p.x,p.y,p.z},strength,f.frontGroup);
                s.key=f.stableLightId;fireLightPlan.add(s);
            }
        }
        ++i;
    }
    publishFireLights();
}

// ---------------------------------------------------------------- what the fire leaves and throws off
namespace
{
ref_shader shChar, shEmber, shHaze, shGround, shGroundGeo;
ref_shader shWood;
ref_geom geomWood;
ref_geom geomChar;
#pragma pack(push, 1)
struct CharVertex
{
	Fvector p, c;
	u32 color;
	float u, v;
};
#pragma pack(pop)
float qualityK() { return ps_painter_fire_quality <= 0 ? .5f : ps_painter_fire_quality == 1 ? .75f : 1.f; }
float h01(u32 x)
{
	x ^= x >> 16;
	x *= 0x7feb352du;
	x ^= x >> 15;
	x *= 0x846ca68bu;
	x ^= x >> 16;
	return float(x & 0xffffffu) / 16777216.f;
}
void slot44(bool ours)
{
	if (ours)
		SRVSManager.SetPSResource(44, noiseSrv);
	else
	{
		CTexture* t = RCache.get_ActiveTexture(44);
		SRVSManager.SetPSResource(44, t ? t->get_SRView() : nullptr);
	}
}
// Many field faces can burn the same physical board or pole in one cell.
// Draw that connected piece once, using the maximum contributing char and heat.
struct WoodDraw { const WoodPatch* patch;float amount, heat, infrared; };
void renderWood(const xr_vector<u32>& cells,float now)
{
    if(cells.empty()) return;
    if(!shWood) shWood.create("painter\\wood_contact");
    if(!shWood) return;
    if(!geomWood) geomWood.create(D3DFVF_XYZ|D3DFVF_NORMAL|D3DFVF_DIFFUSE|D3DFVF_TEX1,RCache.Vertex.Buffer(),nullptr);
    static xr_vector<WoodDraw> draws;
    draws.clear();unsigned fresh=0;
    for(u32 i:cells) {
        const auto& c=field.cells[i];
        auto found=woodPatches.find(i);
        if(found==woodPatches.end()){
            if(fresh==3) continue;
            ++fresh;found=woodPatches.emplace(i,collectWood(c)).first;
        }
        const auto& p=found->second;
        if(p.geometry.vertices.empty()) continue;
        const auto& charState=charred[i];
        float heat=charState.coolAt>0.f?clampr(1.f-(now-charState.coolAt)/50.f,0.f,1.f):1.f;
        const float local=p.crown ? 1.f-TreeBurnAmount(p.crown) : 1.f;
        if(local>.005f) draws.push_back({&p,charState.amount*local,heat,fireHeatBound ? PainterSurfaceHeat::level(now,charState.coolAt,heat_vision_cooldown!=0) : 0.f});
    }
    auto less=[](const WoodDraw& a,const WoodDraw& b){
        const auto& x=*a.patch;const auto& y=*b.patch;
        if(x.x!=y.x)return x.x<y.x;if(x.y!=y.y)return x.y<y.y;
        if(x.z!=y.z)return x.z<y.z;return x.geometry.first<y.geometry.first;
    };
    std::sort(draws.begin(),draws.end(),less);
    setHeatProgram(shWood,HeatWood);RCache.set_Geometry(geomWood);RCache.set_CullMode(CULL_NONE);
    RCache.set_c("painter_wood_cell",field.cellSize*.5f,0.f,0.f,0.f);
    static xr_vector<CharVertex> stream;
    stream.clear();
    auto flush=[&]() {
        if(stream.empty())return;
        u32 offset=0,count=u32(stream.size());
        void* mapped=RCache.Vertex.Lock(count,geomWood->vb_stride,offset);
        memcpy(mapped,stream.data(),count*sizeof(CharVertex));RCache.Vertex.Unlock(count,geomWood->vb_stride);
        RCache.Render(D3DPT_TRIANGLELIST,offset,count/3);stream.clear();
    };
    for(size_t a=0;a<draws.size();) {
        size_t end=a+1;float amount=draws[a].amount,heat=draws[a].heat, infrared=draws[a].infrared;
        for(;end<draws.size()&&!less(draws[a],draws[end]);++end) {
            amount=_max(amount,draws[end].amount);heat=_max(heat,draws[end].heat);infrared=_max(infrared,draws[end].infrared);
        }
        const auto& patch=*draws[a].patch;
        unsigned open=0;
        for(int face=0;face<6;++face){
            const auto& edge=patch.geometry.edge[face];if(edge.empty())continue;
            const int step=(face&1)?1:-1;
            const int key[3]={patch.x+(face/2==0?step:0),patch.y+(face/2==1?step:0),patch.z+(face/2==2?step:0)};
            auto nb=std::lower_bound(draws.begin(),draws.end(),key,[](const WoodDraw& d,const int* b){
                const auto& p=*d.patch;return p.x!=b[0]?p.x<b[0]:p.y!=b[1]?p.y<b[1]:p.z<b[2];
            });
            bool shared=false;
            for(;nb!=draws.end() && !shared;++nb){
                const auto& p=*nb->patch;if(p.x!=key[0]||p.y!=key[1]||p.z!=key[2])break;
                const auto& other=p.geometry.edge[face^1];
                for(int id:edge)if(std::find(other.begin(),other.end(),id)!=other.end()){shared=true;break;}
            }
            if(!shared)open|=1u<<face;
        }
        for(const auto& p:patch.geometry.vertices) {
            CharVertex v={};v.p.set(p.x,p.y,p.z);v.c.set(patch.x*field.cellSize,patch.y*field.cellSize,patch.z*field.cellSize);
            v.color=color_rgba(PainterSurfaceHeat::byte(infrared),0,0,open);v.u=amount;v.v=heat;stream.push_back(v);
            if(stream.size()==3072)flush();
        }
        a=end;
    }
    flush();
}

// burnt ground, burnt walls: a box around every burnt cell near the camera; the shader darkens what is seen inside it
void renderChar()
{
	if (!ps_painter_fire || !ps_painter_fire_burnt || charCells.empty() || !usable || !noiseSrv) return;
	const Fvector cam = Device.vCameraPosition;
	const float now = Device.fTimeGlobal;
	RCache.set_xform_world(Fidentity);
	// 1. the ground: the level's own triangles under what has burnt, tested against the scene's depth
	burnUpload();
	burnMeshUpload();
	if (burnSpots && burnSrv && burnVBVerts)
	{
		if (!shGroundGeo) shGroundGeo.create("painter\\fire_groundgeo");
		if (shGroundGeo && geomBurn)
		{
			setHeatProgram(shGroundGeo,HeatGround);
			RCache.set_Geometry(geomBurn);
			RCache.set_CullMode(CULL_NONE); // after the shader: a shader brings its own cull mode
			slot44(true);
			SRVSManager.SetPSResource(45, burnSrv);
            if(fireHeatBound){uploadGroundHeat(now);SRVSManager.SetPSResource(46,groundHeatSrv);}
			RCache.set_c("painter_ground_k", 1.f / field.cellSize, .78f, fmodf(now, 1000.f), 1.f / float(burnSide));
			RCache.Render(D3DPT_TRIANGLELIST, 0, burnVBVerts / 3);
			slot44(false);
			if(fireHeatBound){CTexture* old=RCache.get_ActiveTexture(46);SRVSManager.SetPSResource(46,old?old->get_SRView():nullptr);}
			CTexture* t = RCache.get_ActiveTexture(45);
			SRVSManager.SetPSResource(45, t ? t->get_SRView() : nullptr);
		}
	}
	// 2. walls, trunks, ceilings: a box around every burnt cell of those near the camera, a round patch in each
	struct Near
	{
		u32 cell;
		float d2;
	};
	static xr_vector<Near> list;
	list.clear();
	for (u32 i : charCells)
	{
		if (i >= field.cells.size() || i >= charred.size() || charred[i].amount <= .02f) continue;
		const PFS::Cell& c = field.cells[i];
		if (c.kind == PFS::GROUND) continue;
		const float dx = c.p.x - cam.x, dy = c.p.y - cam.y, dz = c.p.z - cam.z;
		const float d2 = dx * dx + dy * dy + dz * dz;
		if (d2 < 110.f * 110.f) list.push_back({i, d2});
	}
	if (list.empty())
	{
		RCache.set_CullMode(CULL_CCW);
		return;
	}
	const u32 most = 400;
	if (list.size() > most)
	{
		std::nth_element(list.begin(), list.begin() + most, list.end(), [](const Near& a, const Near& b) { return a.d2 < b.d2; });
		list.resize(most);
	}
	static xr_vector<u32> wood;
    wood.clear();
    auto generic = std::remove_if(list.begin(),list.end(),[&](const Near& item){
        const auto& c=field.cells[item.cell];
        if(c.fuel!=PFS::WOOD)return false;
        if(c.woodFace>=0) wood.push_back(item.cell);
        return true;
    });
    list.erase(generic,list.end());
    renderWood(wood,now);
    if(list.empty()){RCache.set_CullMode(CULL_CCW);return;}
    if (!shChar) shChar.create("painter\\fire_char");
	if (!shChar) return;
	if (!geomChar) geomChar.create(D3DFVF_XYZ | D3DFVF_NORMAL | D3DFVF_DIFFUSE | D3DFVF_TEX1, RCache.Vertex.Buffer(), RCache.QuadIB);
	const u32 count = u32(list.size());
	u32 vOffset = 0;
	CharVertex* pv = (CharVertex*)RCache.Vertex.Lock(count * 24, geomChar->vb_stride, vOffset);
	for (u32 n = 0; n < count; ++n)
	{
		const u32 i = list[n].cell;
		const PFS::Cell& c = field.cells[i];
		const Charred& h = charred[i];
		const Fvector C = {c.p.x, c.p.y, c.p.z};
		const float e = 1.15f;
		const float burning = c.state == PFS::CATCHING || c.state == PFS::BURNING || c.state == PFS::DYING ? clampr(.3f + c.size, 0.f, 1.f) : 0.f;
		const float hot = h.coolAt == 0.f ? 1.f : clampr(1.f - (now - h.coolAt) / 50.f, 0.f, 1.f);
		const u32 color = color_rgba(u32(clampr(c.n.x * .5f + .5f, 0.f, 1.f) * 255.f), u32(clampr(c.n.y * .5f + .5f, 0.f, 1.f) * 255.f),
			u32(clampr(c.n.z * .5f + .5f, 0.f, 1.f) * 255.f), u32(clampr(h.amount, 0.f, 1.f) * 255.f));
		Fvector q[8];
		for (int k = 0; k < 8; ++k) q[k].set(C.x + (k & 1 ? e : -e), C.y + (k & 2 ? e : -e), C.z + (k & 4 ? e : -e));
		static const int faces[6][4] = {{0, 2, 4, 6}, {1, 3, 5, 7}, {0, 1, 4, 5}, {2, 3, 6, 7}, {0, 1, 2, 3}, {4, 5, 6, 7}};
		for (int s = 0; s < 6; ++s)
			for (int k = 0; k < 4; ++k, ++pv)
			{
				pv->p = q[faces[s][k]];
				pv->c = C;
				pv->color = color;
				pv->u = 16.f + 8.f * (fireHeatBound ? PainterSurfaceHeat::level(now,h.coolAt,heat_vision_cooldown!=0) : burning); // + how strongly it burns right now, in eighths
				pv->v = hot;
			}
	}
	RCache.Vertex.Unlock(count * 24, geomChar->vb_stride);
	setHeatProgram(shChar,HeatChar);
	RCache.set_Geometry(geomChar);
	RCache.set_CullMode(CULL_NONE); // after the shader: a shader brings its own cull mode
	slot44(true);
	RCache.set_c("painter_char_k", .5f * field.cellSize, .6f, fmodf(now, 1000.f), 0.f);
	RCache.Render(D3DPT_TRIANGLELIST, vOffset, 0, count * 24, 0, count * 12);
	slot44(false);
	RCache.set_CullMode(CULL_CCW);
}
// a fire's size as it is drawn: how tall its flames are, how wide, how strong it shows right now. False = it does not show.
bool fireShape(const Fire& f, u32 now, float& tall, float& wide, float& fade)
{
	if (f.take < 0 || f.take >= int(takes.size())) return false;
	const Take& k = takes[f.take];
	if (k.state != 2) return false;
	float frame = 0.f;
	bool over = false;
	if (!state(f, k, now, frame, fade, over) || fade <= .02f) return false;
	tall = k.height * f.scale * k.fmax.y;
	if(f.bodyAirHeight>0.f)tall=_min(tall,f.bodyAirHeight*k.fmax.y);
	wide = k.width * f.scale * (k.fmax.x - k.fmin.x);
	return tall > .05f;
}
// sparks and embers: every one is worked out from the clock alone - where it is in its flight, how hot it still is
void renderEmbers()
{
	if (!ps_painter_fire || !ps_painter_fire_embers || fires.empty() || !usable) return;
	if (!shEmber) shEmber.create("painter\\fire_ember");
	if (!shEmber) return;
	if (!geom) geom.create(FVF::F_LIT, RCache.Vertex.Buffer(), RCache.QuadIB);
	const u32 most = 3600;
	const Fvector cam = Device.vCameraPosition;
	const u32 now = Device.dwTimeGlobal;
	const float T = Device.fTimeGlobal, q = qualityK();
	const PFS::Weather wx = weather();
	// Large spreading fires emit embers.
	const float bigFire = clampr((float(field.burning) - 10.f) / 20.f, 0.f, 1.f);
	if (bigFire <= 0.f && Device.fTimeGlobal >= crownEmberUntil) return;
	auto wish = [&](const Fire& f, float tall) {
		if (!f.cell && !f.crownEmbers) return 0.f;
		const float activity=f.crownEmbers ? _max(.32f,bigFire) : bigFire;
		return clampr((10.f + 22.f * tall) * q, 6.f, 90.f) * activity * clampr((tall - .9f) / 1.2f, 0.f, 1.f);
	};
	// how many sparks every fire would like; when that is more than can be drawn, all of them get their share
	float wanted = 0.f;
	for (const Fire& f : fires)
	{
		if (cam.distance_to_sqr(f.pos) > 130.f * 130.f) continue;
		float tall = 0.f, wide = 0.f, fade = 0.f;
		if (fireShape(f, now, tall, wide, fade)) wanted += wish(f, tall);
	}
	if (wanted < 1.f) return;
	const float share = _min(1.f, float(most) * .95f / wanted);
	u32 vOffset = 0, made = 0;
	FVF::LIT* pv = (FVF::LIT*)RCache.Vertex.Lock(most * 4, geom->vb_stride, vOffset);
	for (const Fire& f : fires)
	{
		if (made >= most) break;
		const float d2 = cam.distance_to_sqr(f.pos);
		if (d2 > 130.f * 130.f) continue;
		float tall = 0.f, wide = 0.f, fade = 0.f;
		if (!fireShape(f, now, tall, wide, fade)) continue;
		const float big = clampr(tall / 3.f, .15f, 1.2f);
		const float countF = wish(f, tall) * share; // grows with the fire: the newest spark fades in as it does
		const int count = int(countF) + 1;
		const u32 seed0 = f.born * 2654435761u ^
			(f.cell ? f.cell * 40503u : f.follow != u16(-1) ? u32(f.follow) * 977u :
				(u32(iFloor(f.pos.x * 13.f)) * 73856093u ^ u32(iFloor(f.pos.z * 13.f)) * 19349663u));
		for (int j = 0; j < count && made < most; ++j)
		{
			const u32 seed = seed0 + u32(j) * 104729u;
			// most are short sparks; some are embers that fly for seconds and far with the wind
			const float longOne = h01(seed + 7) < .35f ? 2.4f : 1.f;
			// not the fire's size: the flight below is the clock over the period, and a period that changed with a growing
			// fire every frame ran the sparks backwards, from far out into the flames
			const float life = (1.3f + 2.1f * h01(seed)) * 1.1f * longOne;
			const float period = life * (1.15f + .9f * h01(seed + 1));
			const float ph = (T + h01(seed + 2) * period) / period;
			const float cycle = floorf(ph);
			const float t = (ph - cycle) * period;
			if (t > life) continue; // between two sparks
			const u32 s = seed ^ (u32(int(cycle)) * 2654435761u);
			const float a = PI_MUL_2 * h01(s), rad = .3f * wide * _sqrt(h01(s + 1));
			const float v0 = (1.8f + 3.6f * h01(s + 3)) * (.6f + .6f * big), tau = 1.1f * longOne;
			const float fall = expf(-t / tau);
			// It leaves the fire and keeps going: outward from where it rose, with the wind, only a little unsteady. (It used
			// to swing from side to side, wider and wider with its age - that read as sparks being sucked back into the fire
			// and thrown out again.)
			const float w1 = 2.3f, w2 = 1.9f, p1 = PI_MUL_2 * h01(s + 4), p2 = PI_MUL_2 * h01(s + 5), sway = .1f, ride = .55f + .45f * h01(s + 8);
			const float away = .25f + .55f * h01(s + 6), ca = _cos(a), sa = _sin(a);
			Fvector p = {f.pos.x + ca * (rad + away * t) + sway * _sin(w1 * t + p1) + ride * wx.windX * t,
				f.pos.y + tall * (.12f + .5f * h01(s + 2)) + v0 * tau * (1.f - fall),
				f.pos.z + sa * (rad + away * t) + sway * _cos(w2 * t + p2) + ride * wx.windZ * t};
			if (f.top != 0.f && p.y > f.top - .03f) continue; // a spark of a fire in a room ends at the ceiling
			Fvector vel = {ca * away + sway * w1 * _cos(w1 * t + p1) + ride * wx.windX, v0 * fall,
				sa * away - sway * w2 * _sin(w2 * t + p2) + ride * wx.windZ};
			const float k = t / life;
			float bright = powf(1.f - k, 1.3f) * fade * _min(1.f, t * 8.f) * clampr(countF - float(j), 0.f, 1.f);
			// it cools as it flies: yellow-white, orange, then a dull red
			const float c1 = clampr(k * 2.2f, 0.f, 1.f), c2 = clampr(k * 2.2f - 1.f, 0.f, 1.f);
			float r = 1.f - .25f * c2, g = .82f - .42f * c1 - .28f * c2, b = .45f - .36f * c1 - .07f * c2;
			Fvector dir = vel;
			const float speed = dir.magnitude();
			if (speed > 1e-4f)
				dir.div(speed);
			else
				dir.set(0.f, 1.f, 0.f);
			Fvector toCam;
			toCam.sub(cam, p);
			const float d = _max(toCam.magnitude(), .05f);
			toCam.div(d);
			float w = .008f + .011f * h01(s + 6);
			const float smallest = .0010f * d; // about a pixel: a smaller spark is drawn that big and dimmer, so it does not flicker
			if (w < smallest)
			{
				bright *= w / smallest;
				w = smallest;
			}
			if (bright < .02f) continue;
			const float len = _max(w, speed * .028f); // drawn out along its flight, as a camera sees a spark
			Fvector side;
			side.crossproduct(dir, toCam);
			if (side.square_magnitude() < 1e-8f) side.set(1.f, 0.f, 0.f);
			side.normalize();
			side.mul(w);
			dir.mul(len);
			const u32 color = color_rgba(u32(clampr(r * bright, 0.f, 1.f) * 255.f), u32(clampr(g * bright, 0.f, 1.f) * 255.f),
				u32(clampr(b * bright, 0.f, 1.f) * 255.f), 255);
			pv->set(p.x - dir.x - side.x, p.y - dir.y - side.y, p.z - dir.z - side.z, color, 0.f, 0.f);
			++pv;
			pv->set(p.x - dir.x + side.x, p.y - dir.y + side.y, p.z - dir.z + side.z, color, 1.f, 0.f);
			++pv;
			pv->set(p.x + dir.x - side.x, p.y + dir.y - side.y, p.z + dir.z - side.z, color, 0.f, 1.f);
			++pv;
			pv->set(p.x + dir.x + side.x, p.y + dir.y + side.y, p.z + dir.z + side.z, color, 1.f, 1.f);
			++pv;
			++made;
		}
	}
	RCache.Vertex.Unlock(made * 4, geom->vb_stride);
	if (!made) return;
	RCache.set_xform_world(Fidentity);
	setHeatProgram(shEmber,HeatEmber);
	RCache.set_Geometry(geom);
	RCache.set_CullMode(CULL_NONE); // after the shader: a shader brings its own cull mode
	RCache.Render(D3DPT_TRIANGLELIST, vOffset, 0, made * 4, 0, made * 2);
	RCache.set_CullMode(CULL_CCW);
}
} // namespace

// how far the grass at a place has burnt away: 0 untouched .. 1 gone (asked for every blade of grass that is drawn)
float GrassBurnt(const Fvector& p)
{
    if(!ps_painter_fire)return 0.f;
	if (!burnSpots || !ps_painter_fire_burnt) return 0.f;
	// a ragged edge instead of the cells' straight ones: every blade asks a little beside itself
	const u32 h = u32(iFloor(p.x * 7.3f)) * 73856093u ^ u32(iFloor(p.z * 7.3f)) * 19349663u;
	const float jx = (float(h & 255u) / 255.f - .5f) * .8f, jz = (float((h >> 8) & 255u) / 255.f - .5f) * .8f;
	const int kx = iFloor((p.x + jx) / field.cellSize + .5f), kz = iFloor((p.z + jz) / field.cellSize + .5f);
	const BurnSpot& s = burnAt(kx, kz);
	// (only the grass of the ground that burnt: the grass on the ground over a burning cellar burnt away with the cellar's floor)
	return s.used && s.kx == kx && s.kz == kz && _abs(p.y - s.y) <= burnFloor ? s.amount : 0.f;
}
u32 GrassBurntRevision()
{
	return burnGrassRevision;
}
// how hot the spreading fire is at a place: 0 no fire .. 1 a big one. Fuel thrown there by a fire bottle is left out.
// firesoundspots: every tracked fire (firetrack) that burns now - its light's place and its size (the light's amount)
u32 Spots(Fvector* positions, float* sizes, u32 max)
{
	if (!ps_painter_fire || !positions || !sizes) return 0;
	u32 n = 0;
	for (const SpreadLight& l : spreadLights)
	{
		if (n >= max) break;
		if (!l.used || l.amount < .05f) continue;
		positions[n] = l.heard; // firesides: the fire's burning place nearest the camera (was the light's place, its centre)
		sizes[n] = l.amount;
		++n;
	}
	return n;
}
float HeatAround(const Fvector& p)
{
	// fireheat (2026-11-24): the hottest burning cell around the place, out to the edge of its own flames
	static std::vector<uint32_t> around;
	field.cellsNear(p.x, p.y, p.z, 2, around);
	static const float takeWidth[4] = {.8f, 3.2f, 3.2f, 5.4f}; // spot, grass_low, grass_mid, grass_high (classOf / names)
	float best = 0.f;
	for (uint32_t i : around)
	{
		const PFS::Cell& c = field.cells[i];
		if (c.source || c.size <= 0.f) continue;
		if (c.state != PFS::CATCHING && c.state != PFS::BURNING && c.state != PFS::DYING) continue;
		const float hot = clampr(.25f + c.size, 0.f, 1.f);
		if (hot <= best) continue;
		float share = 0.f;
		if (c.kind == PFS::GROUND)
		{
			if (p.y > c.p.y + 2.2f || p.y < c.p.y - 1.f) continue;
			if (c.lid != 0.f && c.lid - c.p.y >= 1.8f && p.y > c.lid - .05f) continue; // on the floor over a low burning room
			const int cls = classOf(c.size);
			const float half = .5f * takeWidth[cls] * scaleOf(cls, c.size);
			const float full = _max(.8f, .6f * half), edge = _max(1.1f, .9f * half);
			const float dx = p.x - c.p.x, dz = p.z - c.p.z, d = _sqrt(dx * dx + dz * dz);
			if (d >= edge) continue;
			share = d <= full ? 1.f : (edge - d) / (edge - full);
			if (d > .8f)
			{
				// a wall between the place and that fire shields
				Fvector from = {p.x, _max(p.y, c.p.y) + .6f, p.z}, dir = {c.p.x - p.x, 0.f, c.p.z - p.z};
				dir.mul(1.f / d);
				collide::rq_result hit;
				float travelled = 0.f;
				if (pick(from, dir, d, hit, travelled)) continue;
			}
		}
		else if (c.kind == PFS::WALL)
		{
			// a fence, a wall, a bush, a trunk: in front of its face, beside it, at a height its flames reach
			if (p.y > c.p.y + .6f || p.y < c.p.y - 2.f) continue;
			const float dx = p.x - c.p.x, dz = p.z - c.p.z;
			const float front = dx * c.n.x + dz * c.n.z, along = _abs(dz * c.n.x - dx * c.n.z);
			if (front < -.25f || along > 1.2f) continue;
			const float full = c.trunk ? .45f + .3f * c.size : .6f + .4f * c.size, edge = full + .35f;
			if (front >= edge) continue;
			share = front <= full ? 1.f : (edge - front) / (edge - full);
		}
		else continue;
		best = _max(best, hot * share);
	}
	return best;
}
float Heat(const Fvector& p)
{
	if (!ps_painter_fire || field.idle()) return 0.f;
	if (ps_painter_fire_fence_foot) return HeatAround(p);
	const uint32_t i = field.groundAt(p.x, p.y, p.z);
	if (i == PFS::Field::NO_CELL) return 0.f;
	const PFS::Cell& c = field.cells[i];
	if (c.source || c.kind != PFS::GROUND) return 0.f;
	if (c.state != PFS::CATCHING && c.state != PFS::BURNING && c.state != PFS::DYING) return 0.f;
	if (p.y > c.p.y + 2.2f || p.y < c.p.y - 1.f) return 0.f;
	if (c.lid != 0.f && c.lid - c.p.y >= 1.8f && p.y > c.lid - .05f) return 0.f; // on the floor over a low burning room: not in its fire
	return clampr(.25f + c.size, 0.f, 1.f);
}
// How smoky the air is where the camera is, 0 clear .. 1 (the fire's own smoky air, firerework).
float SmokeAir() { return ps_painter_fire && ps_painter_fire_fog ? smokeAir : 0.f; }
// firerework: the smoky air for the renderer. AirFrame (start of a frame's render): the map as a texture in pixel-shader slot
// 47 (painter_fire_air.h) and the smoke's colour by the weather's light; AirConstants: what the shaders read of it.
namespace
{
ID3DTexture2D* airTex = nullptr;
ID3DShaderResourceView* airSrv = nullptr;
ref_shader shAir;
Fvector4 airColour = {.34f, .31f, .28f, 0.f};
}
void AirFrame()
{
	using namespace Air;
	if (!ps_painter_fire || !ps_painter_fire_fog || !placed) return;
	if (!airTex)
	{
		D3D11_TEXTURE2D_DESC d = {};
		d.Width = d.Height = N;
		d.MipLevels = d.ArraySize = 1;
		d.SampleDesc.Count = 1;
		d.Format = DXGI_FORMAT_R16G16B16A16_FLOAT;
		d.Usage = D3D11_USAGE_DYNAMIC;
		d.BindFlags = D3D11_BIND_SHADER_RESOURCE;
		d.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
		if (FAILED(HW.pDevice->CreateTexture2D(&d, nullptr, &airTex)) || !airTex) { airTex = nullptr; return; }
		if (FAILED(HW.pDevice->CreateShaderResourceView(airTex, nullptr, &airSrv)) || !airSrv)
		{
			airSrv = nullptr;
			_RELEASE(airTex);
			return;
		}
		upload = true;
	}
	if (upload)
	{
		D3D11_MAPPED_SUBRESOURCE m = {};
		if (SUCCEEDED(HW.pContext->Map(airTex, 0, D3D11_MAP_WRITE_DISCARD, 0, &m)))
		{
			for (int z = 0; z < N; ++z)
			{
				u16* row = reinterpret_cast<u16*>(static_cast<u8*>(m.pData) + z * size_t(m.RowPitch));
				for (int x = 0; x < N; ++x)
				{
					const int j = z * N + x;
					row[x * 4 + 0] = toHalf(dens[j]);
					row[x * 4 + 1] = toHalf(gy[j]);
					row[x * 4 + 2] = toHalf(1.f - expf(-.6f * hot[j]));
					row[x * 4 + 3] = toHalf(airLayer(dens[j]));
				}
			}
			HW.pContext->Unmap(airTex, 0);
			upload = false;
		}
	}
	// the smoke's colour by the light around it: grey-brown, dark at night (the fires' own glow is added by the shader),
	// paler in the rain (it steams)
	if (g_pGamePersistent && g_pGamePersistent->Environment().CurrentEnv)
	{
		const CEnvDescriptorMixer& env = *g_pGamePersistent->Environment().CurrentEnv;
		const float lum = .3f * env.hemi_color.x + .59f * env.hemi_color.y + .11f * env.hemi_color.z;
		const float night = 1.f - clampr(lum / .35f, 0.f, 1.f);
		Fvector smoke = {.34f, .31f, .28f};
		const float steam = .6f * clampr(env.rain_density, 0.f, 1.f);
		smoke.x += (.62f - smoke.x) * steam;
		smoke.y += (.62f - smoke.y) * steam;
		smoke.z += (.62f - smoke.z) * steam;
		smoke.mul(.15f + .85f * clampr(lum / .5f, 0.f, 1.f));
		airColour.set(smoke.x, smoke.y, smoke.z, night);
	}
	SRVSManager.SetPSResource(47, airSrv);
}
void AirConstants(Fvector4& map, Fvector4& k, Fvector4& colour)
{
	using namespace Air;
	const bool on = ps_painter_fire && ps_painter_fire_fog && placed && airSrv && maxDens > .003f;
	map.set(float(ox) * CELL, float(oz) * CELL, 1.f / (float(N) * CELL), on ? 1.f : 0.f);
	k.set(0.f, clampr(ps_painter_fire_fog_thickness, .2f, 2.f), 6.f, float(N)); // z: a shaft in the thickest smoke: 7 times
	colour = airColour;
}
bool AirWanted()
{
	return ps_painter_fire && ps_painter_fire_fog && Air::placed && airSrv && Air::maxDens > .003f;
}
// The smoky air over the lit world (phase_combine draws the screen's quad once this has set its shader): the eye's rays to
// the corners of a plane across the view, so every pixel finds its place in the world from its depth.
bool RenderAir()
{
	if (!AirWanted()) return false;
	if (!shAir) shAir.create("painter\\fire_air");
	if (!shAir) return false;
	SRVSManager.SetPSResource(47, airSrv);
	RCache.set_Shader(shAir);
	const Fvector eye = Device.vCameraPosition;
	const Fmatrix& M = Device.mInvFullTransform;
	Fvector ray[4];
	const float ndc[4][2] = {{-1.f, 1.f}, {1.f, 1.f}, {-1.f, -1.f}, {1.f, -1.f}}; // the screen's corners: (0,0) (1,0) (0,1) (1,1)
	for (int i = 0; i < 4; ++i)
	{
		const float x = ndc[i][0], y = ndc[i][1];
		const float wx = x * M._11 + y * M._21 + M._31 + M._41;
		const float wy = x * M._12 + y * M._22 + M._32 + M._42;
		const float wz = x * M._13 + y * M._23 + M._33 + M._43;
		const float ww = x * M._14 + y * M._24 + M._34 + M._44;
		const float iw = _abs(ww) > 1e-12f ? 1.f / ww : 0.f;
		ray[i].set(wx * iw - eye.x, wy * iw - eye.y, wz * iw - eye.z);
	}
	// how far that plane lies along the view (every pixel's depth is measured the same way)
	const float along = ray[0].dotproduct(Device.vCameraDirection);
	if (!(along > 1e-4f)) return false;
	RCache.set_c("painter_air_ray00", ray[0].x, ray[0].y, ray[0].z, 0.f);
	RCache.set_c("painter_air_ray10", ray[1].x, ray[1].y, ray[1].z, 0.f);
	RCache.set_c("painter_air_ray01", ray[2].x, ray[2].y, ray[2].z, 0.f);
	RCache.set_c("painter_air_ray11", ray[3].x, ray[3].y, ray[3].z, 0.f);
	RCache.set_c("painter_air_eye", eye.x, eye.y, eye.z, along);
	return true;
}
// firerework: a light standing in the smoke shows its shafts (lights_render.cpp asks for every shadowed spot light)
bool AirLight(const Fvector& position)
{
	return ps_painter_fire && ps_painter_fire_fog && airAt(position) > .04f;
}
bool HazeWanted()
{
	return ps_painter_fire && ps_painter_fire_haze && ps_painter_fire_quality > 0 && usable && noiseSrv && !fires.empty();
}
// hot air over the fires: a sheet over each, turned to the camera, drawn into the game's distortion picture
void RenderHaze()
{
	if (!HazeWanted()) return;
	if (!shHaze) shHaze.create("painter\\fire_haze");
	if (!shHaze) return;
	if (!geom) geom.create(FVF::F_LIT, RCache.Vertex.Buffer(), RCache.QuadIB);
	const u32 most = 96;
	const Fvector cam = Device.vCameraPosition;
	Fvector right = Device.vCameraRight;
	right.y = 0.f;
	if (right.square_magnitude() < 1e-6f) return;
	right.normalize();
	const u32 now = Device.dwTimeGlobal;
	u32 vOffset = 0, made = 0;
	FVF::LIT* pv = (FVF::LIT*)RCache.Vertex.Lock(most * 4, geom->vb_stride, vOffset);
	for (const Fire& f : fires)
	{
		if (made >= most) break;
		if (cam.distance_to_sqr(f.pos) > 70.f * 70.f) continue;
		float tall = 0.f, wide = 0.f, fade = 0.f;
		if (!fireShape(f, now, tall, wide, fade)) continue;
		const float half = _max(.5f, .7f * wide), y0 = f.pos.y + .25f * tall;
		float y1 = f.pos.y + 2.3f * tall;
		if (f.top != 0.f)
		{
			// over a fire in a room the hot air ends at the ceiling
			y1 = _min(y1, f.top);
			if (y1 < y0 + .05f) continue;
		}
		const u32 color = color_rgba(255, 255, 255, u32(clampr(fade * clampr(tall / 1.5f, .3f, 1.f), 0.f, 1.f) * 255.f));
		pv->set(f.pos.x - right.x * half, y0, f.pos.z - right.z * half, color, 0.f, 1.f);
		++pv;
		pv->set(f.pos.x - right.x * half, y1, f.pos.z - right.z * half, color, 0.f, 0.f);
		++pv;
		pv->set(f.pos.x + right.x * half, y0, f.pos.z + right.z * half, color, 1.f, 1.f);
		++pv;
		pv->set(f.pos.x + right.x * half, y1, f.pos.z + right.z * half, color, 1.f, 0.f);
		++pv;
		++made;
	}
	RCache.Vertex.Unlock(made * 4, geom->vb_stride);
	if (!made) return;
	RCache.set_xform_world(Fidentity);
	RCache.set_Shader(shHaze);
	RCache.set_Geometry(geom);
	RCache.set_CullMode(CULL_NONE); // after the shader: a shader brings its own cull mode
	slot44(true);
	RCache.set_c("painter_haze_k", fmodf(Device.fTimeGlobal, 100.f), .10f, 0.f, 0.f);
	RCache.Render(D3DPT_TRIANGLELIST, vOffset, 0, made * 4, 0, made * 2);
	slot44(false);
	RCache.set_CullMode(CULL_CCW);
}

namespace
{
void renderFlames()
{
	if (!ps_painter_fire || (fires.empty() && puffs.empty()) || !usable || !lutSrv || !noiseSrv) return;
	struct Draw
	{
		Fire* f;
		Take* k;
        float frame, fade;
        PainterFireFront::Node node;
        PainterFireFront::Shape front;
    };
    static xr_vector<Draw> list;
	list.clear();
	const u32 now = Device.dwTimeGlobal;
	const Fvector cam = Device.vCameraPosition;
	for (Fire& f : fires)
	{
		Take& k = takes[f.take];
		if (k.state != 2) continue;
		float frame = 0.f, fade = 0.f;
		bool over = false;
		if (!state(f, k, now, frame, fade, over)) continue;
		f.dist2 = cam.distance_to_sqr(f.pos);
		if (f.dist2 > 400.f * 400.f) continue;
		list.push_back({&f, &k, frame, fade});
	}
	// the smoke puffs that are drawn in this frame
	static xr_vector<Puff*> pl;
	pl.clear();
	if (ps_painter_fire_smoke)
		for (Puff& p : puffs)
		{
			p.dist2 = cam.distance_to_sqr(p.pos);
			if (p.dist2 > 600.f * 600.f) continue;
			// a puff the camera is inside of fades out: the smoky air around the camera is the fog's part
			p.show = p.fade * clampr((_sqrt(p.dist2) - .6f * p.radius) / (.8f * p.radius), 0.f, 1.f);
			if (p.show > .004f) pl.push_back(&p);
		}
	const u32 pcount = u32(pl.size());
	if (list.empty() && !pcount) return;
    // Resolve drawn neighbours before distance sorting. Camera rotation cannot change the front.
    const PFS::Weather wx = weather();
    static xr_vector<std::pair<u32,u32>> cells;
    static xr_vector<std::pair<u64,u32>> families;
    cells.clear();
    families.clear();
    for (u32 i=0; i<list.size(); ++i)
    {
        Draw& d=list[i];
        const Take& k=*d.k;
        const Fire& f=*d.f;
        const float radius=.25f*f.scale*(k.width*(k.fmax.x-k.fmin.x)+k.deep*(k.fmax.z-k.fmin.z));
        d.node={f.pos.x,f.pos.y,f.pos.z,_max(.03f,radius),_max(.04f,k.height*f.scale*k.fmax.y),d.fade};
        d.front={};
        if (f.cell && f.cell<=field.cells.size()) cells.emplace_back(f.cell-1,i);
        else if (f.follow!=u16(-1)) families.emplace_back((u64(f.follow)<<1)|1,i);
        else if (f.frontGroup) families.emplace_back(f.frontGroup<<1,i);
    }
    struct SourceTouch {u32 cell; u64 group; u32 draw;};
    static xr_vector<SourceTouch> sourceTouches, representatives;
    sourceTouches.clear();
    representatives.clear();
    std::sort(families.begin(),families.end());
    auto resolveFront = [&](Draw& d,const PainterFireFront::Sum& sum,const PFS::Cell* cell)
    {
        if (!sum.weight) return;
        Fire& f=*d.f;
        // Reuse the cached roof state for non-cell fires on the first frame as well.
        if (f.roof<0) f.roof=(cell ? cell->roofed : roofedAt(f.pos)) ? 1 : 0;
        const float windX = f.roof ? 0.f : wx.windX, windZ = f.roof ? 0.f : wx.windZ;
        float nx=0.f,nz=0.f;
        if (cell && _abs(cell->n.y)>.35f) {
            nx=clampr(-cell->n.x/cell->n.y,-1.f,1.f);
            nz=clampr(-cell->n.z/cell->n.y,-1.f,1.f);
        } else if (f.capN.y<-.35f) {
            nx=clampr(-f.capN.x/f.capN.y,-1.f,1.f);
            nz=clampr(-f.capN.z/f.capN.y,-1.f,1.f);
        }
        const float wallX = cell && cell->trunk ? cell->n.x : f.normal.x;
        const float wallZ = cell && cell->trunk ? cell->n.z : f.normal.z;
        d.front=PainterFireFront::resolve(d.node,sum,Device.fTimeGlobal,windX,windZ,nx,nz,
            wallX,wallZ,f.follow!=u16(-1));
        // Extend the fuel plane only inside a surveyed, continuous outdoor ground patch.
        if (cell && cell->kind==PFS::GROUND && !f.roof && !f.cap && !f.top && cell->n.y>.75f &&
            cell->n.y<.9999f && d.front.join>0.f && d.node.radius<2.5f*field.cellSize)
        {
            unsigned support=0;
            for(const auto& link:cell->links)
            {
                if(link.to>=field.cells.size())continue;
                const auto& g=field.cells[link.to];
                if(g.kind!=PFS::GROUND) {support=0;break;}
                const float dx=g.p.x-cell->p.x,dz=g.p.z-cell->p.z;
                if(g.roofed || g.n.y<.75f || g.n.x*cell->n.x+g.n.y*cell->n.y+g.n.z*cell->n.z<.965f ||
                    _abs(g.p.y-cell->p.y-nx*dx-nz*dz)>.16f) {support=0;break;}
                if (d.node.radius>=2.f*field.cellSize) {
                    bool extended=false;
                    for(const auto& n:g.links) {
                        if(n.to>=field.cells.size())continue;
                        const auto& h=field.cells[n.to];
                        const float hx=h.p.x-g.p.x,hz=h.p.z-g.p.z;
                        if(h.kind==PFS::GROUND && !h.roofed && h.n.x*cell->n.x+h.n.y*cell->n.y+h.n.z*cell->n.z>.965f &&
                            _abs(hx-dx)<.2f*field.cellSize && _abs(hz-dz)<.2f*field.cellSize &&
                            _abs(h.p.y-cell->p.y-nx*(dx+hx)-nz*(dz+hz))<=.16f) {extended=true;break;}
                    }
                    if(!extended)continue;
                }
                if(_abs(dx)>.5f*field.cellSize && _abs(dz)<.2f*field.cellSize) support|=dx>0?1:2;
                if(_abs(dz)>.5f*field.cellSize && _abs(dx)<.2f*field.cellSize) support|=dz>0?4:8;
            }
            if(support==15){d.front.planeX=nx*d.front.join;d.front.planeZ=nz*d.front.join;}
        }
    };
    static xr_vector<u32> byCell, seen;
    static xr_vector<PainterFireFront::Sum> sums;
    byCell.resize(field.cells.size(),u32(-1));
    seen.assign(list.size(),u32(-1));
    sums.assign(list.size(),PainterFireFront::Sum{});
    for (const auto& own:cells) byCell[own.first]=own.second;
    auto compatible = [](const PFS::Cell& a,const PFS::Cell& b)
    {
        if (a.kind!=b.kind) return true; // Only explicit, native cross-surface contact is considered below.
        return a.kind==PFS::GROUND || (a.trunk && b.trunk) || a.n.x*b.n.x+a.n.y*b.n.y+a.n.z*b.n.z>.4f;
    };
    for (const auto& own : cells)
    {
        const u32 i=own.second;
        const PFS::Cell& c=field.cells[own.first];
        auto addPeer = [&](u32 index) {
            const u32 j=byCell[index];
            if(j==u32(-1) || j==i || seen[j]==i) return;
            seen[j]=i;
            PainterFireFront::accumulate(list[i].node,list[j].node,sums[i]);
        };
        for (const auto& link : c.links)
        {
            if (link.to==own.first || link.to>=field.cells.size()) continue;
            const PFS::Cell& mid=field.cells[link.to];
            if (!compatible(c,mid)) continue;
            if (byCell[link.to]!=u32(-1)) {addPeer(link.to);continue;}
            // A thinned ground cell carries no picture. Bridge one lit cell along the same line.
            if (mid.state!=PFS::BURNING || c.kind!=PFS::GROUND || mid.kind!=c.kind) continue;
            for (const auto& next:mid.links)
            {
                if(next.to>=field.cells.size() || next.to==own.first) continue;
                const PFS::Cell& dst=field.cells[next.to];
                if(dst.kind!=c.kind) continue;
                const float x=mid.p.x-c.p.x,z=mid.p.z-c.p.z;
                const float u=dst.p.x-mid.p.x,v=dst.p.z-mid.p.z;
                if(x*u+z*v>0.f && (x*v-z*u)*(x*v-z*u)<.015f*(x*x+z*z)*(u*u+v*v)) addPeer(next.to);
            }
        }
    }
    for (size_t first=0;first<families.size();)
    {
        size_t last=first+1;
        while (last<families.size() && families[last].first==families[first].first) ++last;
        // Per-source bounds: at most twelve bones, nine crown lobes or seven fuel lobes.
        for (size_t i=first;i<last;++i)
        {
            Draw& d=list[families[i].second];
            const u32 draw=families[i].second;
            for (size_t j=first;j<last;++j) if (j!=i) PainterFireFront::accumulate(d.node,list[families[j].second].node,sums[draw]);
            Fire& f=*d.f;
            u32 anchor=f.frontCell-1;
            if (f.frontGround)
            {
                anchor=field.groundAt(f.pos.x,f.pos.y,f.pos.z);
                if (anchor<field.cells.size())
                {
                    const auto p=field.cells[anchor].p;
                    const float dx=p.x-f.pos.x,dz=p.z-f.pos.z;
                    if (_abs(p.y-f.pos.y)>.38f || dx*dx+dz*dz>1.45f) anchor=u32(-1);
                    else {
                        if (f.frontTested!=anchor+1 || f.frontPoint.distance_to_sqr(f.pos)>.01f) {
                            const float dy=p.y-f.pos.y, span=_sqrt(dx*dx+dy*dy+dz*dz);
                            PFS::Site barrier;
                            f.frontTested=anchor+1;
                            f.frontPoint=f.pos;
                            f.frontClear=span<.05f || !world.wall({f.pos.x,f.pos.y+.2f,f.pos.z},
                                {dx/span,dy/span,dz/span},span,barrier);
                        }
                        if (!f.frontClear) anchor=u32(-1);
                    }
                }
            }
            if (anchor<field.cells.size() && field.cells[anchor].state==PFS::BURNING)
            {
                sourceTouches.push_back({anchor,families[i].first,draw});
                auto connect = [&](u32 cell) {
                    const u32 j=byCell[cell];
                    if (j==u32(-1)) return;
                    PainterFireFront::accumulate(d.node,list[j].node,sums[draw]);
                    PainterFireFront::accumulate(list[j].node,d.node,sums[j]);
                };
                if(byCell[anchor]!=u32(-1)) connect(anchor);
                else for(const auto& link:field.cells[anchor].links)
                    if(link.to<field.cells.size() && compatible(field.cells[anchor],field.cells[link.to])) connect(link.to);
            }
        }
        first=last;
    }
    // Distinct sources can share a verified support even when spread density suppresses its picture.
    // Retain one live lobe for each of up to six sources per tile. Work is independent of group crowding.
    std::sort(sourceTouches.begin(),sourceTouches.end(),[](const SourceTouch& a,const SourceTouch& b) {
        if (a.cell!=b.cell) return a.cell<b.cell;
        if (a.group!=b.group) return a.group<b.group;
        return a.draw<b.draw;
    });
    u32 priorCell=u32(-1), perCell=0;
    u64 priorGroup=0;
    for (const auto& contact:sourceTouches)
    {
        if(contact.cell!=priorCell){ priorCell=contact.cell; perCell=0; priorGroup=0; }
        if(contact.group==priorGroup || perCell>=6) continue;
        representatives.push_back(contact); priorGroup=contact.group; ++perCell;
    }
    for(const auto& contact:sourceTouches)
    {
        auto addOtherSources=[&](u32 cell) {
            auto found=std::lower_bound(representatives.begin(),representatives.end(),cell,
                [](const SourceTouch& a,u32 b){return a.cell<b;});
            for (;found!=representatives.end() && found->cell==cell;++found)
                if(found->group!=contact.group) PainterFireFront::accumulate(list[contact.draw].node,list[found->draw].node,sums[contact.draw]);
        };
        addOtherSources(contact.cell);
        const auto& c=field.cells[contact.cell];
        for(const auto& link:c.links) if(link.to<field.cells.size() && link.to!=contact.cell &&
            field.cells[link.to].state==PFS::BURNING && compatible(c,field.cells[link.to])) addOtherSources(link.to);
    }
    bool anyMeasuredColumn=false;
    for(u32 i=0;i<list.size();++i) {
        Draw& d=list[i];
        anyMeasuredColumn |= d.f->wrapColumn.y1>d.f->wrapColumn.y0;
        const u32 cell=d.f->cell-1;
        resolveFront(d,sums[i],cell<field.cells.size()? &field.cells[cell] : nullptr);
    }
    for(const auto& own:cells) byCell[own.first]=u32(-1);

    if(anyMeasuredColumn && shColumnWrap){
        // The field can reach the same shaft/storey from different sides. Keep one
        // visual on that measured support and leave every source and light intact.
        static std::vector<PainterWrapSelection::Row> wrapRows;
        static std::vector<unsigned char> wrapKeep;
        static PainterWrapSelection::Selection wrapSelection;
        wrapRows.clear();
        for(const Draw& d:list){
            const auto& s=d.f->wrapColumn;
            if(!s.valid()){wrapRows.push_back({});continue;}
            wrapRows.push_back({.5f*(s.x0+s.x1),.5f*(s.y0+s.y1),.5f*(s.z0+s.z1),
                .5f*(s.rx0+s.rx1),.5f*(s.rz0+s.rz1),d.f->pos.y-.03f*d.f->scale,
                d.k->height*d.f->scale,d.fade,d.f->id});
        }
        wrapSelection.filter(wrapRows,wrapKeep);
        u32 end=0;
        for(u32 i=0;i<list.size();++i)if(wrapKeep[i]){if(i!=end)list[end]=list[i];++end;}
        list.resize(end);
    }

	std::sort(list.begin(), list.end(), [](const Draw& a, const Draw& b) { return a.f->dist2 > b.f->dist2; }); // far ones first
	std::sort(pl.begin(), pl.end(), [](const Puff* a, const Puff* b) { return a->dist2 > b->dist2; });
	const float windNow = _sqrt(wx.windX * wx.windX + wx.windZ * wx.windZ);
	if (!shVol) shVol.create("painter\\fire_vol");
	if (!shVolDry) shVolDry.create("painter\\fire_vol_dry");
	if (!shVol) return;
	if (!geom) geom.create(FVF::F_LIT, RCache.Vertex.Buffer(), RCache.QuadIB);

	// Use scene lighting for smoke and flame contrast.
	Fvector4 light = {.5f, .5f, .5f, .5f};
	float exposure = 2.f;
	if (g_pGamePersistent && g_pGamePersistent->Environment().CurrentEnv)
	{
		const CEnvDescriptorMixer& env = *g_pGamePersistent->Environment().CurrentEnv;
		light.x = env.ambient.x + .5f * env.hemi_color.x + .35f * env.sun_color.x;
		light.y = env.ambient.y + .5f * env.hemi_color.y + .35f * env.sun_color.y;
		light.z = env.ambient.z + .5f * env.hemi_color.z + .35f * env.sun_color.z;
		const float lum = .3f * (env.hemi_color.x + .5f * env.sun_color.x) + .59f * (env.hemi_color.y + .5f * env.sun_color.y) +
			.11f * (env.hemi_color.z + .5f * env.sun_color.z);
		const float day = clampr(lum / .45f, 0.f, 1.f);
		exposure = 2.f + (1.4f - 2.f) * day; // (3.0 by night drew every flame one flat yellow-white)
		light.w = .95f + (.4f - .95f) * day; // the flames' glow on their smoke counts for less in daylight
	}
	exposure *= ps_painter_fire_brightness;

	// every fire is a box: the part of its take's baked box that holds something, standing where the fire stands
	struct Box
	{
		Fvector X, Z, O;       // the box's width and depth directions, its low corner
		float W, H, D;         // its size (m)
		Fvector lo, hi;        // the part that is looked through (0..1 of the box)
		bool smoke, proxyCull;
		float smokeK, albedo;  // its smoke as the weather has it: how much, how white
		float dull;            // rain dulls its flames (1 = not)
		Fvector lean;          // the wind leans it: how far its top stands from over its foot (m)
		float capA, capB, capC; // the ceiling over it in the box's own measure: its height (0..1) at u, w = capA + capB u + capC w
		bool capOn;
        float wrapThickness;
	};
	static xr_vector<Box> boxes;
	const u32 count = u32(list.size());
	boxes.resize(count);
	u32 vOffset = 0;
	FVF::LIT* pv = (FVF::LIT*)RCache.Vertex.Lock((count + pcount) * 24, geom->vb_stride, vOffset);
	for (u32 i = 0; i < count; ++i)
	{
		const Fire& f = *list[i].f;
		const Take& k = *list[i].k;
		Box& b = boxes[i];
		if (f.normal.square_magnitude() > .5f)
		{
			b.Z.set(-f.normal.x, 0.f, -f.normal.z); // on a wall: the box's depth runs into the wall
			b.X.set(-b.Z.z, 0.f, b.Z.x);
		}
		else
		{
			b.X.set(_cos(f.yaw), 0.f, _sin(f.yaw));
			b.Z.set(-b.X.z, 0.f, b.X.x);
		}
		// The weather on a fire in the open. Rain beats it down - its flames smaller and duller - and what the rain hits
		// turns to steam: much more smoke, and white. Wet ground after the rain still steams a little. (A spreading fire's
		// cells shrink in the rain by the spread's own rule; the others are drawn smaller here.)
		if (f.roof < 0)
		{
			const u32 ci = f.cell - 1;
			list[i].f->roof = (f.cell && ci < field.cells.size() ? field.cells[ci].roofed : roofedAt(f.pos)) ? 1 : 0;
		}
		const bool outside = f.roof == 0;
		const float rain = outside ? wx.rain : 0.f;
		const float steam = outside ? clampr(1.2f * wx.rain + .3f * wx.wet, 0.f, 1.f) : 0.f;
		const float beaten = f.cell ? 1.f : 1.f - .3f * rain;
		float swell = 1.f;
		if (xr_strcmp(k.name.c_str(), "plume"))
		{
			const float tt = Device.fTimeGlobal;
			swell += .2f * (.5f * _sin(.11f * f.pos.x + .07f * f.pos.z - 1.3f * tt) + .35f * _sin(-.05f * f.pos.x + .13f * f.pos.z - .9f * tt + 1.7f)
				+ .15f * _sin(.21f * f.pos.x - .17f * f.pos.z - 2.1f * tt + .4f));
			if (f.growFor > 0.f)
			{
				const float age = clampr((float(now - f.born) * .001f - f.delay) / f.growFor, 0.f, 1.f);
				swell *= .25f + .75f * age * age * (3.f - 2.f * age);
			}
		}
		b.W = k.width * f.scale * beaten * (.6f + .4f * swell);
		b.H = k.height * f.scale * beaten * swell;
		if (f.trunkReach > 0.f && ps_painter_fire_fence_foot)
		{
			// firetrunk: tall enough to reach the next cell up the trunk, growing with its flames (a fading one: 60 %)
			const u32 tc = f.cell - 1;
			const float grown = f.cell && tc < field.cells.size() ? _min(field.cells[tc].size, 1.f) : 0.f;
			b.H = _max(b.H, f.trunkReach * (.6f + .4f * grown) * beaten);
		}
		if(f.bodyAirHeight>0.f)b.H=_min(b.H,f.bodyAirHeight*beaten);
		b.D = k.deep * f.scale * beaten * (.6f + .4f * swell);
		b.smokeK = k.smoke ? _max(f.smoke, .8f * steam) * (1.f + 1.6f * steam) : 0.f;
		b.albedo = f.albedo + (.93f - f.albedo) * steam;
		b.dull = 1.f - .25f * rain;
		// The wind leans it: the box is sheared along the wind. A smoke column leans most; a take that was baked with wind
		// leans by itself and gets less; a fire on a wall or around a trunk stays where it is.
		b.lean.set(0.f, 0.f, 0.f);
		if (outside && windNow > .4f && f.normal.square_magnitude() < .5f)
		{
			const u32 ci = f.cell - 1;
			const bool trunk = f.cell && ci < shown.size() && shown[ci].form == 1;
			if (!trunk)
			{
				LPCSTR name = k.name.c_str();
				float per; // metres of lean for every metre of height
				if (!xr_strcmp(name, "plume"))
					per = _min(.085f * windNow, .85f);
				else
					per = _min(.035f * windNow, .4f) * (strstr(name, "_wind") ? .35f : 1.f) * (f.follow != u16(-1) ? .5f : 1.f);
				b.lean.set(wx.windX / windNow * per * b.H, 0.f, wx.windZ / windNow * per * b.H);
			}
		}
        const PainterFireFront::Shape& front = list[i].front;
        if (front.join > 0.f)
        {
            b.W *= 1.f + front.wider;
            // Stay close to thin wall geometry; horizontal extension is along its tangent.
            if (f.normal.square_magnitude()<.5f) b.D *= 1.f + .55f*front.wider;
            b.lean.x += front.join*(front.steerX - b.lean.x);
            b.lean.z += front.join*(front.steerZ - b.lean.z);
        }
		b.O = f.pos;
		b.O.mad(b.X, -(f.mirror ? 1.f - k.footX : k.footX) * b.W);
		b.O.mad(b.Z, -k.footZ * b.D);
		b.O.y -= .03f * f.scale;
        const bool column = shColumnWrap && (f.wrapColumn.valid() || f.wrapFace.valid());
        const float slopeX=column?0.f:front.planeX, slopeZ=column?0.f:front.planeZ;
        b.O.y += slopeX*(b.O.x-f.pos.x)+slopeZ*(b.O.z-f.pos.z);
		b.smoke = b.smokeK > 0.f;
		if (b.smoke)
		{
			b.lo.set(0.f, 0.f, 0.f);
			b.hi.set(1.f, 1.f, 1.f);
		}
		else
		{
			b.lo = k.fmin;
			b.hi = k.fmax;
			if (f.mirror)
			{
				b.lo.x = 1.f - k.fmax.x;
				b.hi.x = 1.f - k.fmin.x;
			}
		}
        b.wrapThickness=0.f;
        if(column)
        {
            const float base=f.pos.y-.03f*f.scale;
            const auto dims=f.wrapFace.valid() ? PainterWoodFlame::bounds(f.wrapFace,base,b.H,f.scale)
                                               : PainterWrapBounds::get(f.wrapColumn,base,b.H,f.scale);
            b.wrapThickness=dims.thickness;
            b.O.set(dims.x,dims.y,dims.z);
            b.W=dims.w;b.H=dims.h;b.D=dims.d;
            b.X.set(1.f,0.f,0.f);b.Z.set(0.f,0.f,1.f);b.lean.set(0.f,0.f,0.f);
            b.lo.set(0.f,0.f,0.f);b.hi.set(1.f,1.f,1.f);
        }
		// A fire in a room: its box ends at the ceiling. Nothing of it - flames, soot, smoke - is drawn above that: not in
		// the storey above, not on the ground over a cellar.
		b.capOn = false;
		b.capA = b.capB = b.capC = 0.f;
		const float capH = f.cap != 0.f ? f.cap : f.top;
		if (capH != 0.f)
		{
			// ... along the ceiling's own slope (11-22): under a sloping roof a level cut let the part of the box that stands
			// out past the roof near the eaves show flames with a flat top, in the open
			Fvector n = f.capN;
			if (n.y > -.3f) n.set(0.f, -1.f, 0.f); // level, or too steep to follow: level
			n.normalize();
            // Compute the plane in the sheared frame, including the uphill offset of its top.
            const float denom = n.y*b.H + (front.join > 0.f ? n.x*b.lean.x+n.z*b.lean.z : 0.f);
            const float k = front.join > 0.f ? 1.f/_min(-.001f,denom) : 1.f/(n.y*b.H);
            b.capA = front.join > 0.f ? (n.y*(capH-b.O.y)-n.x*(b.O.x-f.pos.x)-n.z*(b.O.z-f.pos.z))*k :
                (capH - b.O.y) / b.H - (n.x * (b.O.x - f.pos.x) + n.z * (b.O.z - f.pos.z)) * k;
			b.capB = -(n.x * b.X.x + n.z * b.X.z) * b.W * k;
			b.capC = -(n.x * b.Z.x + n.z * b.Z.z) * b.D * k;
			b.capOn = true;
			const float y = clampr(b.capA + _max(0.f, b.capB) + _max(0.f, b.capC), b.lo.y + .02f, 1.f);
			if (y < b.hi.y) b.hi.y = y;
		}
		Fvector c[8];
		for (int n = 0; n < 8; ++n)
		{
			c[n] = b.O;
			c[n].mad(b.X, (n & 1 ? b.hi.x : b.lo.x) * b.W);
			c[n].y += (n & 2 ? b.hi.y : b.lo.y) * b.H;
			c[n].mad(b.Z, (n & 4 ? b.hi.z : b.lo.z) * b.D);
			c[n].mad(b.lean, n & 2 ? b.hi.y : b.lo.y);
            c[n].y += slopeX*(c[n].x-b.O.x)+slopeZ*(c[n].z-b.O.z);
		}
		static const int faces[6][4] = {{0, 2, 4, 6}, {1, 3, 5, 7}, {0, 1, 4, 5}, {2, 3, 6, 7}, {0, 1, 2, 3}, {4, 5, 6, 7}};
		// Submit only exit faces of well-conditioned boxes. Retain both sides at a near plane.
		const bool unitAxes = _abs(b.X.square_magnitude()-1.f) < .00001f &&
		    _abs(b.Z.square_magnitude()-1.f) < .00001f && _abs(b.X.dotproduct(b.Z)) < .00001f;
		b.proxyCull = unitAxes && b.W > .08f && b.H > .08f && b.D > .08f &&
		    (b.hi.x-b.lo.x)*b.W > .08f && (b.hi.y-b.lo.y)*b.H > .08f && (b.hi.z-b.lo.z)*b.D > .08f;
		Fvector cameraOffset;
		cameraOffset.sub(cam, b.O);
        cameraOffset.y-=slopeX*cameraOffset.x+slopeZ*cameraOffset.z;
		const float cameraY = cameraOffset.y / _max(b.H,.001f);
		const float cameraX = (cameraOffset.dotproduct(b.X) - cameraY * b.lean.dotproduct(b.X)) / _max(b.W,.001f);
		const float cameraZ = (cameraOffset.dotproduct(b.Z) - cameraY * b.lean.dotproduct(b.Z)) / _max(b.D,.001f);
		const float mx=.03f*(1.f+_abs(b.lean.dotproduct(b.X))*(1.f+_abs(slopeX)+_abs(slopeZ))/_max(b.H,.001f))/_max(b.W,.001f);
		const float my=.03f*(1.f+_abs(slopeX)+_abs(slopeZ))/_max(b.H,.001f);
		const float mz=.03f*(1.f+_abs(b.lean.dotproduct(b.Z))*(1.f+_abs(slopeX)+_abs(slopeZ))/_max(b.H,.001f))/_max(b.D,.001f);
		const bool visibleFaces[6] = {cameraX >= b.lo.x-mx, cameraX <= b.hi.x+mx,
		    cameraY >= b.lo.y-my, cameraY <= b.hi.y+my, cameraZ >= b.lo.z-mz, cameraZ <= b.hi.z+mz};
		for (int s = 0; s < 6; ++s) {
		    for (int n = 0; n < 4; ++n, ++pv) {
		        const Fvector& v = c[faces[s][!b.proxyCull || visibleFaces[s] ? n : 0]];
		        pv->set(v.x,v.y,v.z,0xffffffff,0.f,0.f);
		    }
		}
	}
	// ... and every smoke puff a box around its ball
	for (u32 i = 0; i < pcount; ++i)
	{
		const Puff& p = *pl[i];
		const float h = p.radius * 1.03f;
		Fvector c[8];
		for (int n = 0; n < 8; ++n) c[n].set(p.pos.x + (n & 1 ? h : -h), p.pos.y + (n & 2 ? h : -h), p.pos.z + (n & 4 ? h : -h));
		static const int sides[6][4] = {{0, 2, 4, 6}, {1, 3, 5, 7}, {0, 1, 4, 5}, {2, 3, 6, 7}, {0, 1, 2, 3}, {4, 5, 6, 7}};
		const bool visibleFaces[6] = {cam.x >= p.pos.x-h-.03f, cam.x <= p.pos.x+h+.03f,
		    cam.y >= p.pos.y-h-.03f, cam.y <= p.pos.y+h+.03f, cam.z >= p.pos.z-h-.03f, cam.z <= p.pos.z+h+.03f};
		for (int s = 0; s < 6; ++s) {
		    for (int n = 0; n < 4; ++n, ++pv) {
		        const Fvector& v = c[sides[s][h < .04f || visibleFaces[s] ? n : 0]];
		        pv->set(v.x,v.y,v.z,0xffffffff,0.f,0.f);
		    }
		}
	}
	RCache.Vertex.Unlock((count + pcount) * 24, geom->vb_stride);

	RCache.set_xform_world(Fidentity);
	RCache.set_CullMode(CULL_NONE);
	// What a fire costs is its steps times the part of the screen its box covers (1: all of it). The eye in a box or right
	// by it: all of it. Otherwise its height and width as seen against the screen's.
	const float halfH = Device.mProject._22 > .01f ? 1.f / Device.mProject._22 : .66f; // the screen at 1 m: half its height
	const float halfW = Device.mProject._11 > .01f ? 1.f / Device.mProject._11 : 1.2f;  // ... half its width
	static xr_vector<float> fireCover, puffCover;
	fireCover.resize(count);
	puffCover.resize(pcount);
	float fireScreens = 0.f, puffScreens = 0.f;
	for (u32 i = 0; i < count; ++i)
	{
		const Box& b = boxes[i];
		const float dd = _max(_sqrt(list[i].f->dist2), .3f);
		Fvector rel;
		rel.sub(cam, b.O);
        if(b.wrapThickness<=0.f) rel.y-=list[i].front.planeX*rel.x+list[i].front.planeZ*rel.z;
		const float v = rel.y / _max(b.H, .01f);
		const float u = (rel.dotproduct(b.X) - b.lean.dotproduct(b.X) * v) / _max(b.W, .01f);
		const float w = (rel.dotproduct(b.Z) - b.lean.dotproduct(b.Z) * v) / _max(b.D, .01f);
		const bool in = u > -.15f && u < 1.15f && v > -.15f && v < 1.15f && w > -.15f && w < 1.15f;
		fireCover[i] = in ? 1.f : _min(b.H / (dd * 2.f * halfH), 1.f) * _min(_max(b.W, b.D) / (dd * 2.f * halfW), 1.f);
		fireScreens += fireCover[i];
	}
	for (u32 i = 0; i < pcount; ++i)
	{
		const Puff& p = *pl[i];
		const float dd = _max(_sqrt(p.dist2), .3f);
		puffCover[i] = dd < p.radius * 1.03f ? 1.f : _min(2.f * p.radius / (dd * 2.f * halfH), 1.f) * _min(2.f * p.radius / (dd * 2.f * halfW), 1.f);
		puffScreens += puffCover[i];
	}
	// fires (and puffs) that together cover more than three screens share what three may cost
	const float fireShare = fireScreens > 2.f ? 2.f / fireScreens : 1.f;
	const float puffShare = puffScreens > 2.f ? 2.f / puffScreens : 1.f;

	// The smoke puffs go in between the fires by distance, far ones first: every fire asks for the puffs that lie behind it
	// before it is drawn itself.
	u32 pn = 0;
	auto drawPuffs = [&](float nearerThan) {
		if (pn >= pcount) return;
		if (!shSmoke) shSmoke.create("painter\\fire_smoke");
		if (!shSmoke)
		{
			pn = pcount;
			return;
		}
		const float most = ps_painter_fire_quality <= 0 ? 5.f : ps_painter_fire_quality == 1 ? 7.f : 9.f;
		for (; pn < pcount && pl[pn]->dist2 > nearerThan; ++pn)
		{
			const Puff& p = *pl[pn];
			const float onScreen = p.radius / _max(_sqrt(p.dist2), .5f);
			const float want = clampr(4.f + 12.f * onScreen, 4.f, most);
			// a puff that fills the screen: 5 (the shader makes 6 of them, without the finest billows)
			const float steps = float(iFloor(clampr(5.f * puffShare / _max(puffCover[pn], .01f), 4.f, want)));
			suspendFireHeat();
			RCache.set_Shader(shSmoke);
			RCache.set_Geometry(geom);
			RCache.set_CullMode(CULL_NONE); // (after the shader, as for the fires)
			SRVSManager.SetPSResource(44, noiseSrv);
			RCache.set_c("painter_smoke_c", p.pos.x, p.pos.y, p.pos.z, p.radius);
			RCache.set_c("painter_smoke_k", p.dens, p.albedo, p.young, p.show);
			RCache.set_c("painter_smoke_flow", puffFlow.x, puffFlow.y, puffFlow.z, steps);
			if (ps_painter_fire_light_quality <= 0) RCache.set_c("painter_fire_light", light);
			else RCache.set_c("painter_fire_light", light.x, light.y, light.z, puffSourceGlow(p,light.w));
			RCache.Render(D3DPT_TRIANGLELIST, vOffset + (count + pn) * 24, 0, 24, 0, 12);
		}
	};
	for (u32 i = 0; i < count; ++i)
	{
		const Draw& d = list[i];
		const Fire& f = *d.f;
		drawPuffs(f.dist2);
		const Take& k = *d.k;
		const Box& b = boxes[i];
		const u32 i0 = _min(u32(d.frame), k.frames - 1);
		const u32 i1 = k.loop ? (i0 + 1) % k.frames : _min(i0 + 1, k.frames - 1);
		const float blend = d.frame - float(i0);
		// the nearer and bigger on the screen, the more steps through it and the finer its volume
		const float onScreen = b.H / _max(_sqrt(f.dist2), .5f);
		const float qk = qualityK() * (count > 100 ? .55f : count > 40 ? .75f : 1.f);
		const float want = clampr((16.f + 90.f * onScreen) * qk, 10.f, 96.f * qk);
		// ... held to what it costs: a box that fills the screen gets no more than 24 steps (times the detail setting);
		// boxes that together fill more than two screens share that, down to 6 steps each
		const float least = _min(fireShare < 1.f ? 6.f : 10.f, want);
		const float steps = float(iFloor(clampr(24.f * qk * fireShare / _max(fireCover[i], .01f), least, want)));
		const float coarse = onScreen < (ps_painter_fire_quality <= 0 ? .35f : .12f) ? 1.f : 0.f;
		if(b.wrapThickness>0.f) setHeatProgram(shColumnWrap,HeatWrap);
        else if (k.coarsePaired) setHeatProgram(!b.smoke && b.proxyCull ? shVolDryPair : shVolPair,!b.smoke && b.proxyCull ? HeatDryPair : HeatPair);
		else setHeatProgram(!b.smoke && b.proxyCull && shVolDry ? shVolDry : shVol,!b.smoke && b.proxyCull && shVolDry ? HeatDry : HeatVolume);
		RCache.set_Geometry(geom);
		// Both sides of the box, set AFTER the shader: setting a shader brings that shader's own cull mode and undoes an
		// earlier one. (Set before it, the box lost its far sides and the fire showed as flat slices.)
		RCache.set_CullMode(CULL_NONE);
		// the volume textures sit at slots of their own; the shader's file binds only the scene's depth
		SRVSManager.SetPSResource(40, k.a.srv);
		SRVSManager.SetPSResource(41, k.b.srv);
		SRVSManager.SetPSResource(42, k.c.srv);
		SRVSManager.SetPSResource(43, lutSrv);
		SRVSManager.SetPSResource(44, noiseSrv);
		const float iw = 1.f / b.W, ih = 1.f / b.H, id = 1.f / b.D;
		// (a leaning box: a point's place across it counts from where the box stands at that point's height)
		const float lx = b.lean.dotproduct(b.X) * iw * ih, lz = b.lean.dotproduct(b.Z) * id * ih;
		if (b.wrapThickness<=0.f && (d.front.planeX!=0.f || d.front.planeZ!=0.f))
        {
            const float x=d.front.planeX, z=d.front.planeZ;
            const Fvector bx={b.X.x*iw+lx*x,-lx,b.X.z*iw+lx*z};
            const Fvector by={-x*ih,ih,-z*ih};
            const Fvector bz={b.Z.x*id+lz*x,-lz,b.Z.z*id+lz*z};
            RCache.set_c("painter_vol_bx",bx.x,bx.y,bx.z,-bx.dotproduct(b.O));
            RCache.set_c("painter_vol_by",by.x,by.y,by.z,-by.dotproduct(b.O));
            RCache.set_c("painter_vol_bz",bz.x,bz.y,bz.z,-bz.dotproduct(b.O));
        }
        else
        {
            RCache.set_c("painter_vol_bx", b.X.x * iw, -lx, b.X.z * iw, -b.O.dotproduct(b.X) * iw + lx * b.O.y);
		RCache.set_c("painter_vol_by", 0.f, ih, 0.f, -b.O.y * ih);
		RCache.set_c("painter_vol_bz", b.Z.x * id, -lz, b.Z.z * id, -b.O.dotproduct(b.Z) * id + lz * b.O.y);
        }
		RCache.set_c("painter_vol_mlo", b.lo.x, b.lo.y, b.lo.z, steps);
		RCache.set_c("painter_vol_mhi", b.hi.x, b.hi.y, b.hi.z, coarse);
		RCache.set_c("painter_vol_fmin", k.fmin.x, k.fmin.y, k.fmin.z, f.mirror ? 1.f : 0.f);
		RCache.set_c("painter_vol_fscl", 1.f / (k.fmax.x - k.fmin.x), 1.f / (k.fmax.y - k.fmin.y), 1.f / (k.fmax.z - k.fmin.z), blend);
		RCache.set_c("painter_vol_frame", float(i0 % k.fgrid[0]), float(i0 / k.fgrid[0]), float(i1 % k.fgrid[0]), float(i1 / k.fgrid[0]));
		RCache.set_c("painter_vol_sframe", float(i0 % k.sgrid[0]), float(i0 / k.sgrid[0]), float(i1 % k.sgrid[0]), float(i1 / k.sgrid[0]));
		RCache.set_c("painter_vol_grid", 1.f / float(k.fgrid[0]), 1.f / float(k.fgrid[1]), 1.f / float(k.sgrid[0]), 1.f / float(k.sgrid[1]));
		const float m = coarse > .5f ? 1.f : .5f;
		// w: a flame voxel in metres as it is drawn (the shader sizes its wrinkles by it)
		RCache.set_c("painter_vol_half", m / float(k.fvox[0]), m / float(k.fvox[1]), m / float(k.fvox[2]),
			2.f * m * (k.fmax.x - k.fmin.x) * b.W / float(k.fvox[0]));
		// w: the clock the wrinkles climb by; it starts over every 100 s, where the pattern meets itself again
		RCache.set_c("painter_vol_shalf", .5f / float(k.svox[0]), .5f / float(k.svox[1]), .5f / float(k.svox[2]),
			fmodf(Device.fTimeGlobal, 100.f));
		RCache.set_c("painter_vol_heat", k.heat[0], k.heat[1], k.heat[2], k.heat[3]);
		// a fire drawn smaller or bigger than it was baked keeps its look: what a metre of it does scales with it
		const float is = 1.f / _max(f.scale, .05f);
		RCache.set_c("painter_vol_amt", k.amount[0] * is, k.amount[1] * is, k.amount[2] * is, d.fade);
		RCache.set_c("painter_vol_k", exposure * b.dull, b.smokeK, b.albedo, 1.f / lutHeat);
		// x: baked against a wall (its far side); y: where in the box a ceiling ends the fire (0 = none)
		RCache.set_c("painter_vol_lut", k.wall ? 1.f : 0.f, f.cap != 0.f ? clampr((f.cap - b.O.y) * ih, .02f, 1.f) : 0.f, f.air, d.front.join); // z: its soft bottom
        RCache.set_c("painter_vol_front_peer", d.front.peerX, d.front.peerY, d.front.peerZ, d.front.invRadius);
        if(b.wrapThickness>0.f)
        {
            const auto& s=f.wrapColumn;
            const auto& p=f.wrapFace;
            if(p.valid()){
                RCache.set_c("painter_wrap_low",p.x,p.z,0.f,0.f);
                RCache.set_c("painter_wrap_high",p.x+p.nx*p.slope,p.z+p.nz*p.slope,0.f,0.f);
                RCache.set_c("painter_wrap_y",p.y,1.f,b.wrapThickness,1.f/b.wrapThickness);
            } else {
                RCache.set_c("painter_wrap_low",s.x0,s.z0,s.rx0,s.rz0);
                RCache.set_c("painter_wrap_high",s.x1,s.z1,s.rx1,s.rz1);
                RCache.set_c("painter_wrap_y",s.y0,1.f/(s.y1-s.y0),b.wrapThickness,1.f/b.wrapThickness);
            }
            RCache.set_c("painter_wrap_face",p.nx,p.nz,p.left,p.right);
            const float windScale=f.roof>0 || windNow<.4f ? 0.f : _min(.14f,.70f/windNow);
            // wallfire: w: 0 = the board flames as before; 1 = shaped, the lowest burning section of its column (its ground
            // flame stands at the wall's foot) or a bottle's own; 2 = shaped, a section above it (it burns lower)
            float shapedW=0.f;
            if(ps_painter_fire_fence_foot){const u32 ci=f.cell-1;shapedW=(!f.cell || (ci<shown.size() && shown[ci].foot))?1.f:2.f;}
            RCache.set_c("painter_wrap_wind",wx.windX*windScale,wx.windZ*windScale,k.coarsePaired ? 1.f : 0.f,shapedW);
        }
		RCache.set_c("painter_vol_cap", b.capA, b.capB, b.capC, b.capOn ? 1.f : 0.f);
		RCache.set_c("painter_fire_light", light);
		RCache.Render(D3DPT_TRIANGLELIST, vOffset + i * 24, 0, 24, 0, 12);
	}
	drawPuffs(-1.f); // the puffs nearer than every fire
	// the five slots go back to what the frame's own bookkeeping holds there: nothing that is drawn later meets a fire
	// volume where it expects its own texture
	for (u32 s = 40; s < 45; ++s)
	{
		CTexture* t = RCache.get_ActiveTexture(s);
		SRVSManager.SetPSResource(s, t ? t->get_SRView() : nullptr);
	}
	RCache.set_CullMode(CULL_CCW);
}

} // namespace

// Keep the material owners used by the draw paths. Quality and distance choose
// uniforms or a material, while the shader key comes from the current renderer.
static bool warmFireMaterial(ref_shader& material, LPCSTR name)
{
    if (material) return true;
    string_path file;
    xr_sprintf(file, "r3\\painter_%s.s", name + sizeof("painter\\") - 1);
    if (!fileThere("$game_shaders$", file)) return false;
    material.create(name);
    return !!material;
}

template<class Visitor>
static void visitFireMaterials(Visitor visit)
{
    struct Material { ref_shader* object; LPCSTR name; };
    const Material ordinary[] = {
        {&shVol, "painter\\fire_vol"}, {&shVolDry, "painter\\fire_vol_dry"},
        {&shVolPair, "painter\\fire_vol_pair"}, {&shVolDryPair, "painter\\fire_vol_dry_pair"},
        {&shColumnWrap, "painter\\fire_wrap"}, {&shSmoke, "painter\\fire_smoke"},
        {&shGround, "painter\\fire_ground"}, {&shGroundGeo, "painter\\fire_groundgeo"},
        {&shChar, "painter\\fire_char"}, {&shWood, "painter\\wood_contact"},
        {&shEmber, "painter\\fire_ember"}, {&shHaze, "painter\\fire_haze"}, {&shAir, "painter\\fire_air"} // firerework
    };
    for (const auto& entry : ordinary) visit(*entry.object, entry.name);
    for (unsigned i = 0; i < HeatCount; ++i) visit(heatPasses[i].shader, heatNames[i]);
}

void VisitWarmPrograms(void(*emit)(const char*,const char*,int,int))
{
    if (!emit) return;
    visitFireMaterials([&](ref_shader&, LPCSTR name) {
        string_path file;
        xr_sprintf(file, "r3\\painter_%s.s", name + sizeof("painter\\") - 1);
        if (!fileThere("$game_shaders$", file)) return;
        IReader* script = FS.r_open("$game_shaders$", file);
        if (!script) return;
        const std::string text(static_cast<const char*>(script->pointer()), script->length());
        FS.r_close(script);

        // The owned fire scripts name the VS and PS as literals. The actual
        // material construction below still covers modified or computed passes.
        size_t pos = 0;
        while ((pos = text.find("shader:begin", pos)) != std::string::npos) {
            pos += sizeof("shader:begin") - 1;
            char vertex[128] = {}, pixel[128] = {};
            if (sscanf(text.c_str()+pos, " ( \"%127[A-Za-z0-9_]\" , \"%127[A-Za-z0-9_]\" )", vertex, pixel) == 2) {
                emit(vertex,"vs",-1,-1);
                emit(pixel,"ps",-1,-1);
            }
        }
    });
}

void PrewarmShaders()
{
    if (HW.pDevice && HW.FeatureLevel >= D3D_FEATURE_LEVEL_11_0)
    {
        PainterFixedBloodPrograms::warm(HW.pDevice);
        PainterFixedBloodPrograms::prepareTextureReaders(HW.pDevice);
    }
    visitFireMaterials(warmFireMaterial);
}

void renderCeilingHeat()
{
	if (!fireHeatBound || field.idle()) return;
	struct Hot { float x, z, lid, w, reach; };
	static xr_vector<Hot> hots;
	static xr_vector<std::pair<u32, u32>> members; // (cell, its group)
	hots.clear();
	members.clear();
	for (u32 i : field.burningCells())
	{
		const PFS::Cell& c = field.cells[i];
		if (c.fuel == PFS::BRUSH || c.trunk) continue; // a crown or a trunk stands in no room
		const float lid = c.kind == PFS::CEILING ? c.p.y : c.kind == PFS::GROUND ? roomTop(c) : c.lid; // the ceiling over it
		if (lid == 0.f || lid < c.p.y - .1f || lid - c.p.y > 12.f) continue; // open sky: the hot gas rises away
		const float w = .3f + c.size;
		u32 g = u32(-1);
		for (u32 h = 0; h < u32(hots.size()); ++h)
		{
			const float dx = hots[h].x / hots[h].w - c.p.x, dz = hots[h].z / hots[h].w - c.p.z;
			if (_abs(hots[h].lid - lid) < .6f && dx * dx + dz * dz < 100.f)
			{
				g = h;
				break;
			}
		}
		if (g == u32(-1))
		{
			if (hots.size() >= 16) continue;
			hots.push_back({0.f, 0.f, lid, 0.f, 0.f});
			g = u32(hots.size() - 1);
		}
		hots[g].x += c.p.x * w;
		hots[g].z += c.p.z * w;
		hots[g].w += w;
		members.emplace_back(i, g);
	}
	if (hots.empty()) return;
	for (Hot& h : hots)
	{
		h.x /= h.w;
		h.z /= h.w;
	}
	for (const auto& m : members)
	{
		const PFS::Cell& c = field.cells[m.first];
		Hot& h = hots[m.second];
		h.reach = _max(h.reach, _sqrt((c.p.x - h.x) * (c.p.x - h.x) + (c.p.z - h.z) * (c.p.z - h.z)));
	}
	HeatPass& pass = heatPasses[HeatCeiling];
	if (!pass.shader) pass.shader.create(heatNames[HeatCeiling]);
	if (!pass.shader) return;
	if (!geom) geom.create(FVF::F_LIT, RCache.Vertex.Buffer(), RCache.QuadIB);
	const u32 count = u32(hots.size());
	static xr_vector<Fvector4> lo, hi;
	lo.resize(count);
	hi.resize(count);
	u32 vOffset = 0;
	FVF::LIT* pv = (FVF::LIT*)RCache.Vertex.Lock(count * 24, geom->vb_stride, vOffset);
	static const int faces[6][4] = {{0, 2, 4, 6}, {1, 3, 5, 7}, {0, 1, 4, 5}, {2, 3, 6, 7}, {0, 1, 2, 3}, {4, 5, 6, 7}};
	for (u32 n = 0; n < count; ++n)
	{
		const Hot& h = hots[n];
		const float R = clampr(h.reach + 2.5f, 3.f, 14.f);               // how far it has spread under the ceiling
		const float thick = clampr(.6f + .2f * _sqrt(h.w), .6f, 1.6f);   // how deep the hot layer hangs
		lo[n].set(h.x - R, h.lid - thick, h.z - R, 1.f - expf(-h.w / 3.f)); // w: how hot
		hi[n].set(h.x + R, h.lid + .05f, h.z + R, 1.f / R);
		Fvector q[8];
		for (int k = 0; k < 8; ++k) q[k].set(k & 1 ? hi[n].x : lo[n].x, k & 2 ? hi[n].y : lo[n].y, k & 4 ? hi[n].z : lo[n].z);
		for (int s = 0; s < 6; ++s)
			for (int k = 0; k < 4; ++k, ++pv)
				pv->set(q[faces[s][k]].x, q[faces[s][k]].y, q[faces[s][k]].z, 0xffffffff, 0.f, 0.f);
	}
	RCache.Vertex.Unlock(count * 24, geom->vb_stride);
	setHeatProgram(pass.shader, HeatCeiling);
	RCache.set_Geometry(geom);
	RCache.set_CullMode(CULL_NONE); // after the shader: a shader brings its own cull mode
	for (u32 n = 0; n < count; ++n)
	{
		RCache.set_c("painter_ceil_lo", lo[n].x, lo[n].y, lo[n].z, lo[n].w);
		RCache.set_c("painter_ceil_hi", hi[n].x, hi[n].y, hi[n].z, hi[n].w);
		RCache.Render(D3DPT_TRIANGLELIST, vOffset + n * 24, 0, 24, 0, 12);
	}
	RCache.set_CullMode(CULL_CCW);
}
void Render()
{
    if (!ps_painter_fire) return;
	beginFireHeat();
	renderChar();
	renderFlames();
	renderCeilingHeat(); // firerework: thermal - a room's ceiling fills with the fire's heat
	renderEmbers();
	suspendFireHeat();
	fireHeatBound=false;
}

void clearState(bool leavingLevel)
{
    spreadClear();
    if(leavingLevel)bareForget();
    {
        std::lock_guard<std::mutex> hold(askedLock);
        const unsigned stamp=PainterFireMasterStamp();
        if(leavingLevel || !(stamp&1u))asked.clear();
        else asked.erase(std::remove_if(asked.begin(),asked.end(),[stamp](const Asked& a){return a.stamp!=stamp;}),asked.end());
    }
    {
        std::lock_guard<std::mutex> hold(nearLock);nearSnap.clear();
    }
    smokeAir=0.f;spreadSources=false;
    for(auto& s:spreadLights){s.used=false;s.plume=0;}
	for(unsigned i=0;i<PainterFireLighting::slots;i++){for(auto& lamp:fireLamps[i])lamp.destroy();}
	fireLightPlan.count=0;firePresentation=PainterFireLighting::Presentation{};
	fires.clear();
	readerHold.stop();
	for (Take& t : takes)
	{
		releaseVol(t.a);
		releaseVol(t.b);
		releaseVol(t.c);
	}
	takes.clear();
	_RELEASE(lutSrv);
	_RELEASE(lutTex);
	_RELEASE(noiseSrv);
	_RELEASE(noiseTex);
    if (leavingLevel) {
        for(auto& p:heatPasses){p.shader.destroy();p.maximum=p.native=nullptr;}
        shVolPair.destroy();
        shVolDryPair.destroy();
        shColumnWrap.destroy();
        shVolDry.destroy();
        shVol.destroy();
        shSmoke.destroy();
        shGround.destroy();
        shGroundGeo.destroy();
        shChar.destroy();
        shWood.destroy();
        shEmber.destroy();
        shHaze.destroy();
        shAir.destroy(); // firerework
        PainterFixedBloodPrograms::clear();
        PainterFixedBloodPrograms::releaseTextureReaders();
    }
	_RELEASE(groundHeatSrv);_RELEASE(groundHeatTex);groundHeatLast=-1.f;groundHeatCooling=-1;
	SRVSManager.SetPSResource(47, nullptr); // firerework: the smoky air's map
	_RELEASE(airSrv);_RELEASE(airTex);Air::placed=false;Air::maxDens=0.f;Air::upload=true;
	
	
    
	pairedPrograms=false;
	
	
	
	
	
	geomBurn.destroy();
	_RELEASE(burnVB);
	burnVBVerts = 0;
	_RELEASE(burnSrv);
	_RELEASE(burnTex);
	burnPublished=false;
	
	geomWood.destroy();
	
	
	geomChar.destroy();
	geom.destroy();
	{
		std::lock_guard<std::mutex> hold(bodyLock);
		bodyAsked.clear();
	}
	takeBody = takeBodyLow = -1;
	loaded = false;
	usable = false;
}
void PrepareHeatTarget(const ref_rt& native,bool needed)
{
    if(!needed && !fireHeatDirty)return;
    if(!native || !native->pSurface)return;
    if(needed && (!fireHeatRT || native->_order!=fireHeatSourceOrder || fireHeatRT->_order!=fireHeatOwnOrder)){
        D3D11_TEXTURE2D_DESC source={}, current={};
        native->pSurface->GetDesc(&source);
        if(fireHeatRT && fireHeatRT->pSurface)fireHeatRT->pSurface->GetDesc(&current);
        if(source.Width!=current.Width || source.Height!=current.Height || source.SampleDesc.Count!=current.SampleDesc.Count){
            fireHeatRT.destroy();
            fireHeatRT.create("$user$painter_fire_heat",source.Width,source.Height,D3DFMT_L8,source.SampleDesc.Count);
        }
        fireHeatDirty=true;
        fireHeatSourceOrder=native->_order;
        fireHeatOwnOrder=fireHeatRT?fireHeatRT->_order:0;
    }
    if(fireHeatDirty && fireHeatRT && fireHeatRT->pRT){const float zero[4]={};HW.pContext->ClearRenderTargetView(fireHeatRT->pRT,zero);}
    fireHeatDirty=false;
}
void Clear(){clearState(true);fireHeatRT.destroy();fireHeatSourceOrder=fireHeatOwnOrder=0;}
} // namespace PainterFire
