// WallmarksEngine.cpp: implementation of the CWallmarksEngine class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "BloodThermalWallmark.h"
#include <algorithm> // R21: std::stable_sort
#include "WallmarksEngine.h"
#include "BloodNativeProfile.h"
#include <limits>

#include "../../xrEngine/xr_object.h"
#include "../../xrEngine/x_ray.h"
#include "../../xrEngine/GameFont.h"
#include "SkeletonCustom.h"
#ifdef USE_DX11
#include "BloodGpuSkinning.h"
#include "BloodGpuProjection.h"
#include "PainterWallmarkMask.h"
#include "PainterDent.h" // melee (value 3): a decal on a dented body
#include "dxRenderDeviceRender.h"
#include <unordered_map>
#include <vector>
#include <cstdlib>

extern int r_blood_gpu;
extern int r_blood_gpu_create;
namespace blood_projection_warmup
{
// A topology update can publish several small exterior sources together. Warm
// them before drawing their rebuilt marks, without exceeding the former
// single-source byte ceiling. Cold or failed sources retain exact CPU fallback.
template<class Pending, class Registered, class Upload, class Rejected>
void drain(Pending& pending, Registered registered, Upload upload, Rejected rejected)
{
    // R19: spread model-source uploads over frames (a cold source only means CPU creation for that hit).
    constexpr size_t byte_limit = 4u * 1024u * 1024u;
    constexpr unsigned source_limit = 2;
    size_t bytes = 0;
    unsigned attempted = 0;
    while (!pending.empty() && attempted < source_limit)
    {
        auto source = pending.back().lock();
        if (!source || registered(source.get())) { pending.pop_back(); continue; }
        const size_t vertices = source->vertices.size(), indices = source->indices.size();
        // Cache admission already enforces this per-source ceiling. Reject an
        // invalid oversize entry rather than leaving it to block the queue.
        if (vertices > byte_limit / sizeof(source->vertices[0]) ||
            indices > byte_limit / sizeof(source->indices[0]))
        {
            pending.pop_back(); ++attempted; rejected(); continue;
        }
        const size_t source_bytes = vertices * sizeof(source->vertices[0]) +
            indices * sizeof(source->indices[0]);
        if (source_bytes > byte_limit)
        {
            pending.pop_back(); ++attempted; rejected(); continue;
        }
        // Keep the next valid source queued for a fresh budget next frame.
        if (source_bytes > byte_limit - bytes) break;
        pending.pop_back(); ++attempted; bytes += source_bytes;
        upload(source);
    }
}
} // namespace blood_projection_warmup
struct BloodGpuWallmarkState
{
	struct ProjectionSource
	{
		std::shared_ptr<const CSkeletonWallmark::ProjectionSource> source;
		BloodGpuProjection::SourceHandle handle;
	};
	struct ProjectionMark
	{
		intrusive_ptr<CSkeletonWallmark> mark;
		BloodGpuProjection::MarkHandle handle;
		u32 seen_frame = 0;
		u32 output_color = 0;
		bool output_valid = false;
		bool queued = false;
	};
	struct ProjectionDraw
	{
		intrusive_ptr<CSkeletonWallmark> mark;
		BloodGpuProjection::MarkHandle handle;
		BloodGpuProjection::Output output;
		u32 color = 0;
		bool update = false;
	};
	BloodGpuProjection::Backend projection_backend;
	using ProjectionSourceKey = const CSkeletonWallmark::ProjectionSource*;
	std::unordered_map<ProjectionSourceKey, ProjectionSource, std::hash<ProjectionSourceKey>,
		std::equal_to<ProjectionSourceKey>, CSkeletonWallmark::CacheAllocator<
		std::pair<ProjectionSourceKey const, ProjectionSource>>> projection_sources;
	std::unordered_map<CSkeletonWallmark*, ProjectionMark, std::hash<CSkeletonWallmark*>,
		std::equal_to<CSkeletonWallmark*>, CSkeletonWallmark::CacheAllocator<
		std::pair<CSkeletonWallmark* const, ProjectionMark>>> projection_marks;
	CSkeletonWallmark::ProjectionVector<std::weak_ptr<const CSkeletonWallmark::ProjectionSource>> projection_pending;
	CSkeletonWallmark::ProjectionVector<BloodGpuProjection::SourceRange> projection_ranges;
	CSkeletonWallmark::ProjectionVector<ProjectionDraw> projection_draws;
	u32 projection_warmup_frame = u32(-1);
	u64 projection_warmups = 0, projection_warmup_failures = 0, projection_cpu_fallbacks = 0;
    struct Source
    {
        intrusive_ptr<CSkeletonWallmark> mark;
        BloodGpuSkinning::SourceHandle handle;
        const CSkeletonWallmark::WMFace* faces = nullptr;
        size_t face_count = 0;
        u16 bone_count = 0;
        u64 geometry_revision = 0;
        u32 seen_frame = 0;
        bool queued = false;
        u64 visimask = 0;      // R26: the body's visible-bone mask the source was exported with
        u32 vertex_count = 0;  // R26: vertices actually exported (faces on hidden bones are left out)
    };
    struct Pending
    {
        intrusive_ptr<CSkeletonWallmark> mark;
        CKinematics::WallmarkRenderPreparation prepared;
    };
    BloodGpuSkinning::Backend backend;
    std::unordered_map<CSkeletonWallmark*, Source, std::hash<CSkeletonWallmark*>,
        std::equal_to<CSkeletonWallmark*>,
        CSkeletonWallmark::CacheAllocator<std::pair<CSkeletonWallmark* const, Source>>> sources;
    std::vector<BloodGpuSkinning::InputVertex, CSkeletonWallmark::CacheAllocator<BloodGpuSkinning::InputVertex>> upload;
    std::vector<BloodGpuSkinning::Rows, CSkeletonWallmark::CacheAllocator<BloodGpuSkinning::Rows>> palette;
    std::vector<Pending, CSkeletonWallmark::CacheAllocator<Pending>> pending;
    u64 cpu_preferred = 0, cpu_fallback = 0, gpu_marks = 0, failed_batches = 0;
    u64 source_capacity_rejects = 0;
    // R18: a GPU skinning batch may hold the marks of ONE cut body (removal field); it draws with that body's
    // masked shader over the batch output. Uncut bodies share batches as before.
    std::shared_ptr<PainterRemovalGpu> batch_field;
    ref_shader batch_masked;
    u64 masked_marks = 0, masked_batches = 0;
    // R21: why append_gpu refused a mark (a refused mark is CPU-skinned and breaks its body's batch), and the
    // per-slot draw order grouped by body so one cut body's marks stay in one batch.
    u64 fb_no_snapshot = 0, fb_no_vertices = 0, fb_too_large = 0, fb_no_bones = 0, fb_mask = 0, fb_export = 0,
        fb_batch = 0, fb_append = 0, fb_exception = 0;
    std::vector<u32, CSkeletonWallmark::CacheAllocator<u32>> order;
    // R22: the frame plan. Every body mark that can skin on the GPU is appended to ONE batch per frame before the
    // slot loop; the slot loop then draws contiguous output ranges (one draw per body run) or runs the CPU paths.
    struct Plan
    {
        enum : u8 { Cpu = 0, Gpu = 1 };
        u32 item = 0;
        u32 first_vertex = 0, vertex_count = 0;
        const void* field = nullptr; // removal field of a cut body: its marks draw through the masked shader
        ref_shader masked;
        const void* ownership=nullptr;
        const void* tear=nullptr; // maskdecal: a value-3 piece's tear set (part of the masked shader)
        u8 kind = Cpu;
    };
    struct PlanRange { u32 first = 0, count = 0; };
    std::vector<Plan, CSkeletonWallmark::CacheAllocator<Plan>> plans;
    std::vector<PlanRange, CSkeletonWallmark::CacheAllocator<PlanRange>> slot_plans;
    BloodGpuSkinning::Output frame_output;
    bool frame_batch_open = false, frame_batch_failed = false;
    xr_vector<intrusive_ptr<CSkeletonWallmark>> refresh; // R37: carried marks with a pending refresh (leftover budget)
};
#endif

// R20: marks expired because no skin is left under them, and GPU-created marks resolved to CPU faces for that test.
static u64 g_painter_marks_removed = 0, g_painter_marks_resolved = 0;
static u64 g_painter_marks_empty = 0; // R21: marks whose projection resolved to no face
static u64 g_painter_marks_refreshed = 0, g_painter_resolve_vertices = 0; // R37: background refreshes; candidate vertices resolved
static u32 g_painter_plan_gpu = 0, g_painter_plan_cpu = 0, g_painter_plan_skip = 0, g_painter_plan_vertices = 0; // R22: last frame
#ifdef USE_DX11
static u64 g_painter_mask_refusals = 0;
static u32 g_painter_mask_refusal_frame = u32(-1);
static void painter_mask_refusal_log(const char* what, const xr_string& reason)
{
    ++g_painter_mask_refusals;
    if (g_painter_mask_refusal_frame != u32(-1) && Device.dwFrame - g_painter_mask_refusal_frame < 600) return;
    g_painter_mask_refusal_frame = Device.dwFrame;
    Msg("! Painter wallmark %s: %s (%llu refusals so far; one line per 600 frames)", what, reason.c_str(),
        static_cast<unsigned long long>(g_painter_mask_refusals));
}
static u64 g_blood_source_uploads_logged = 0;
#endif
float wallmark_range_static = 100.f;
float wallmark_range_skeleton = 50.f;

namespace WallmarksEngine
{
    struct wm_slot
    {
        typedef CWallmarksEngine::StaticWMVec StaticWMVec;
        typedef CWallmarksEngine::StaticWMSectorMap StaticWMSectorMap;
        ref_shader shader;
        StaticWMSectorMap static_items;
        xr_vector<intrusive_ptr<CSkeletonWallmark>> skeleton_items;

        wm_slot(ref_shader sh)
        {
            shader = sh;
            skeleton_items.reserve(256);
        }
    };
}

// #include "xr_effsun.h"

constexpr const float W_DIST_FADE = 15.f;
constexpr const float W_DIST_FADE_SQR = W_DIST_FADE * W_DIST_FADE;
constexpr const float I_DIST_FADE_SQR = 1.f / W_DIST_FADE_SQR;
constexpr const int MAX_TRIS = 1024 * 64;

IC bool operator ==(const CWallmarksEngine::wm_slot* slot, const ref_shader& shader) { return slot->shader == shader; }

CWallmarksEngine::wm_slot* CWallmarksEngine::FindSlot(ref_shader shader)
{
    const auto it = slot_index.find(shader._get());
    return it != slot_index.end() ? it->second : nullptr;
}

CWallmarksEngine::wm_slot* CWallmarksEngine::AppendSlot(ref_shader shader)
{
    marks.push_back(xr_new<wm_slot>(shader));
    slot_index.emplace(shader._get(), marks.back());
    return marks.back();
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CWallmarksEngine::CWallmarksEngine()
#ifdef PROFILE_CRITICAL_SECTIONS
    :lock(MUTEX_PROFILE_ID(CWallmarksEngine))
#endif // PROFILE_CRITICAL_SECTIONS
{
    hGeom.create(FVF::F_LIT, RCache.Vertex.Buffer(), NULL);
#ifdef USE_DX11
    D3DVERTEXELEMENT9 declaration[]={
        {0,0,D3DDECLTYPE_FLOAT3,D3DDECLMETHOD_DEFAULT,D3DDECLUSAGE_POSITION,0},
        {0,12,D3DDECLTYPE_D3DCOLOR,D3DDECLMETHOD_DEFAULT,D3DDECLUSAGE_COLOR,0},
        {0,16,D3DDECLTYPE_FLOAT2,D3DDECLMETHOD_DEFAULT,D3DDECLUSAGE_TEXCOORD,0},
        {0,24,D3DDECLTYPE_FLOAT3,D3DDECLMETHOD_DEFAULT,D3DDECLUSAGE_TEXCOORD,1},
        {0,36,D3DDECLTYPE_FLOAT2,D3DDECLMETHOD_DEFAULT,D3DDECLUSAGE_TEXCOORD,2},D3DDECL_END()};
    hGeomMasked.create(declaration,RCache.Vertex.Buffer(),nullptr);
#endif
}

CWallmarksEngine::~CWallmarksEngine()
{
    clear();
    hGeom.destroy();
#ifdef USE_DX11
    hGeomMasked.destroy();PainterWallmarkMask::Clear();
#endif
}

void CWallmarksEngine::clear()
{
    ResetGpuWallmarks();
    large_wallmark_vertices.clear_and_free();
    slot_index.clear();
    {
        for (auto p_it = marks.begin(); p_it != marks.end(); p_it++)
        {
            for (auto& [sector, static_vec] : (*p_it)->static_items)
            {
                for (auto m_it = static_vec.begin(); m_it != static_vec.end(); m_it++)
                    static_wm_destroy(m_it->W);
            }
			
			xr_delete(*p_it);
		}
		marks.clear();
	}
	{
		for (u32 it = 0; it < static_pool.size(); it++)
			xr_delete(static_pool[it]);
		static_pool.clear();
	}
}

void CWallmarksEngine::ResetGpuWallmarks(bool keep_device_state)
{
#ifdef USE_DX11
    xrCriticalSectionGuard guard(lock);
    if (keep_device_state)
    {
        if (gpu_skinning) RCache.set_Vertices(RCache.Vertex.Buffer(), hGeom->vb_stride);
        return;
    }
    // Existing GPU/pending marks reconstruct from their original hit snapshot
    // after reset, even if their mesh/parent has since moved or been replaced.
    g_blood_projection_epoch.fetch_add(1, std::memory_order_relaxed);
    if (gpu_skinning)
    {
        // Rewarm live mesh caches for future hits. Old mark snapshots carry the
        // previous epoch and cannot silently be recreated from a later pose.
        try
        {
            for (auto& item : gpu_skinning->projection_sources)
                CSkeletonWallmark::ProjectionSource::Publish(item.second.source);
            for (auto& item : gpu_skinning->projection_pending)
                if (auto source = item.lock()) CSkeletonWallmark::ProjectionSource::Publish(source);
        }
        catch (...) {} // A failed optional rewarm still leaves exact CPU fallback.
        // RCache must not retain a raw pointer to the private output buffer.
        RCache.set_Vertices(RCache.Vertex.Buffer(), hGeom->vb_stride);
        gpu_skinning->~BloodGpuWallmarkState();
        std::free(gpu_skinning);
        gpu_skinning = nullptr;
    }
#endif
}

void CWallmarksEngine::DumpGpuWallmarkStats()
{
#ifdef USE_DX11
    xrCriticalSectionGuard guard(lock);
    Msg("* Blood marks expired with their skin: %llu (GPU-created marks resolved to CPU faces: %llu)",
        static_cast<unsigned long long>(g_painter_marks_removed), static_cast<unsigned long long>(g_painter_marks_resolved));
    Msg("* Blood marks carried through publications: %llu, refreshed in the background: %llu, resolve candidate vertices: %llu, mask shader assemblies: %llu",
        static_cast<unsigned long long>(g_painter_marks_carried.load(std::memory_order_relaxed)), static_cast<unsigned long long>(g_painter_marks_refreshed),
        static_cast<unsigned long long>(g_painter_resolve_vertices), static_cast<unsigned long long>(PainterWallmarkMask::assemblies()));
    Msg("* Blood GPU creation CPU: model_cache_attempts=%llu model_cache_built=%llu hit_cache_missing=%llu hit_source_cold=%llu hit_snapshots=%llu",
        g_blood_projection_cpu.cache_attempts.load(std::memory_order_relaxed),
        g_blood_projection_cpu.cache_built.load(std::memory_order_relaxed),
        g_blood_projection_cpu.cache_missing.load(std::memory_order_relaxed),
        g_blood_projection_cpu.source_cold.load(std::memory_order_relaxed),
        g_blood_projection_cpu.hit_snapshots.load(std::memory_order_relaxed));
    Msg("* Blood GPU creation cache admission: empty=%llu invalid=%llu child_budget=%llu total_budget=%llu allocation_failed=%llu",
        g_blood_projection_cpu.cache_empty.load(std::memory_order_relaxed),
        g_blood_projection_cpu.cache_invalid.load(std::memory_order_relaxed),
        g_blood_projection_cpu.cache_child_budget.load(std::memory_order_relaxed),
        g_blood_projection_cpu.cache_total_budget.load(std::memory_order_relaxed),
        g_blood_projection_cpu.cache_alloc_failed.load(std::memory_order_relaxed));
    Msg("* Blood GPU creation cache residency: cpu_bytes=%llu cpu_peak_bytes=%llu",
        u64(CSkeletonWallmark::ProjectionSource::ReservedBytes()),
        u64(CSkeletonWallmark::ProjectionSource::PeakBytes()));
    if (!gpu_skinning)
    {
        Msg("* Blood GPU: mode=%d; backend not initialized", r_blood_gpu);
        return;
    }
    const auto c = gpu_skinning->backend.GetCounters();
    Msg("* Blood GPU: mode=%d dispatches=%llu jobs=%llu vertices=%llu source_uploads=%llu live_sources=%u live_source_vertices=%u cpu_preferred=%llu cpu_fallback=%llu failed_batches=%llu resource_failures=%llu source_capacity_rejects=%llu masked_marks=%llu masked_batches=%llu",
        r_blood_gpu, c.dispatches, c.jobs_submitted, c.vertices_submitted, c.source_uploads,
        c.live_sources, c.live_source_vertices, gpu_skinning->cpu_preferred, gpu_skinning->cpu_fallback,
        gpu_skinning->failed_batches, c.resource_failures, gpu_skinning->source_capacity_rejects,
        gpu_skinning->masked_marks, gpu_skinning->masked_batches);
    Msg("* Blood GPU refusals: no_snapshot=%llu no_vertices=%llu too_large=%llu no_bones=%llu mask=%llu capacity=%llu export=%llu batch=%llu append=%llu exception=%llu; empty marks expired=%llu",
        gpu_skinning->fb_no_snapshot, gpu_skinning->fb_no_vertices, gpu_skinning->fb_too_large, gpu_skinning->fb_no_bones,
        gpu_skinning->fb_mask, gpu_skinning->source_capacity_rejects, gpu_skinning->fb_export, gpu_skinning->fb_batch,
        gpu_skinning->fb_append, gpu_skinning->fb_exception, static_cast<unsigned long long>(g_painter_marks_empty));
    Msg("* Blood GPU frame plan (last frame): gpu_marks=%u cpu_marks=%u skipped=%u batch_vertices=%u",
        g_painter_plan_gpu, g_painter_plan_cpu, g_painter_plan_skip, g_painter_plan_vertices);
    const auto p = gpu_skinning->projection_backend.GetCounters();
    Msg("* Blood GPU creation: mode=%d created=%llu project_dispatches=%llu render_dispatches=%llu live_marks=%u reserved_vertices=%u source_uploads=%llu warmups=%llu warmup_failures=%llu pending_sources=%u cpu_fallbacks=%llu gpu_bytes=%llu",
        r_blood_gpu_create, p.marks_created, p.projection_dispatches, p.render_dispatches,
        p.live_marks, p.reserved_mark_vertices, p.source_uploads, gpu_skinning->projection_warmups,
        gpu_skinning->projection_warmup_failures, u32(gpu_skinning->projection_pending.size()),
        gpu_skinning->projection_cpu_fallbacks, p.gpu_allocated_bytes);
    Msg("* Blood GPU creation source residency: gpu_sources=%u gpu_source_vertices=%u gpu_source_indices=%u gpu_source_bytes=%u",
        p.live_sources, p.live_source_vertices, p.live_source_indices, p.live_source_bytes);
    Msg("* Blood GPU creation batches: render_batches=%llu candidate_vertices=%llu backend_fallbacks=%llu resource_failures=%llu",
        p.render_batches, p.candidate_vertices, p.fallbacks, p.resource_failures);
    Msg("* Blood GPU creation allocator: polls=%llu busy=%llu copies=%llu reads=%llu reclaimed_vertices=%llu",
        p.allocation_polls, p.allocation_busy, p.allocation_copies, p.allocation_reads, p.reclaimed_vertices);
    Msg("* Blood GPU creation relocation: moves=%llu moved_vertices=%llu moved_payload_bytes=%llu",
        p.allocation_moves, p.moved_vertices, p.moved_payload_bytes);

#endif
}

// allocate
CWallmarksEngine::static_wallmark* CWallmarksEngine::static_wm_allocate()
{
	static_wallmark* W = 0;
	if (static_pool.empty()) W = xr_new<static_wallmark>();
	else
	{
		W = static_pool.back();
		static_pool.pop_back();
	}

	W->m_thermal_blood=false;
    W->m_thermal_stamp={};
	W->m_fTimeStart = RDEVICE.fTimeGlobal;
	W->m_fTimeEnd = ps_r__WallmarkTTL * 15.f;
	W->verts.clear();
	return W;
}

// destroy
void CWallmarksEngine::static_wm_destroy(CWallmarksEngine::static_wallmark* W)
{
	static_pool.push_back(W);
}

// render
void CWallmarksEngine::static_wm_render(CWallmarksEngine::static_wallmark* W, FVF::LIT* & V)
{
	float a = W->TimeEnd() == -1.f ? 0.f : (RDEVICE.fTimeGlobal - W->TimeStart()) / W->TimeEnd();
	int aC = iFloor(a * 255.f);
	clamp(aC, 0, 255);
	u32 C = BloodThermalWallmark::Color(RDEVICE.fTimeGlobal-W->TimeStart(),aC,W->m_thermal_blood,W->m_thermal_stamp);
	FVF::LIT* S = &*W->verts.begin();
	FVF::LIT* E = &*W->verts.end();
	for (; S != E; S++, V++)
	{
		V->p.set(S->p);
		V->color = C;
		V->t.set(S->t);
	}
}

//--------------------------------------------------------------------------------
void CWallmarksEngine::RecurseTri(u32 t, Fmatrix& mView, CWallmarksEngine::static_wallmark& W)
{
	CDB::TRI* T = sml_collector.getT() + t;
	if (T->dummy) return;
	T->dummy = 0xffffffff;

	// Some vars
	u32* v_ids = T->verts;
	Fvector* v_data = sml_collector.getV();
	sml_poly_src.clear();
	sml_poly_src.push_back(v_data[v_ids[0]]);
	sml_poly_src.push_back(v_data[v_ids[1]]);
	sml_poly_src.push_back(v_data[v_ids[2]]);
	sml_poly_dest.clear();

	sPoly* P = sml_clipper.ClipPoly(sml_poly_src, sml_poly_dest);

	//. todo
	// uv_gen = mView * []
	// UV = pos*uv_gen

	if (P)
	{
		// Create vertices and triangulate poly (tri-fan style triangulation)
		FVF::LIT V0, V1, V2;
		Fvector UV;

		mView.transform_tiny(UV, (*P)[0]);
		V0.set((*P)[0], 0, (1 + UV.x) * .5f, (1 - UV.y) * .5f);
		mView.transform_tiny(UV, (*P)[1]);
		V1.set((*P)[1], 0, (1 + UV.x) * .5f, (1 - UV.y) * .5f);

		for (u32 i = 2; i < P->size(); i++)
		{
			mView.transform_tiny(UV, (*P)[i]);
			V2.set((*P)[i], 0, (1 + UV.x) * .5f, (1 - UV.y) * .5f);
			W.verts.push_back(V0);
			W.verts.push_back(V1);
			W.verts.push_back(V2);
			V1 = V2;
		}

		// recurse
		for (u32 i = 0; i < 3; i++)
		{
			u32 adj = sml_adjacency[3 * t + i];
			if (0xffffffff == adj) continue;
			CDB::TRI* SML = sml_collector.getT() + adj;
			v_ids = SML->verts;

			Fvector test_normal;
			test_normal.mknormal(v_data[v_ids[0]], v_data[v_ids[1]], v_data[v_ids[2]]);
			float cosa = test_normal.dotproduct(sml_normal);
			if (cosa < 0.034899f) continue; // cos(88)
			RecurseTri(adj, mView, W);
		}
	}
}

void CWallmarksEngine::BuildMatrix(Fmatrix& mView, float invsz, const Fvector& from)
{
	// build projection
	Fmatrix mScale;
	Fvector at, up, right, y;
	at.sub(from, sml_normal);
	y.set(0, 1, 0);
	if (_abs(sml_normal.y) > .99f) y.set(1, 0, 0);
	right.crossproduct(y, sml_normal);
	up.crossproduct(sml_normal, right);
	mView.build_camera(from, at, up);
	mScale.scale(invsz, invsz, invsz);
	mView.mulA_43(mScale);
}

// demonized: add user defined rotation to wallmark
void CWallmarksEngine::AddWallmark_internal(CDB::TRI* pTri, const Fvector* pVerts, const Fvector& contact_point,
                                            ref_shader hShader, float sz, float ttl, float rotation,const BloodThermalStamp& stamp)
{
	BloodNativeProfile::Scope profileCreate(BloodNativeProfile::Stage::StaticCreate);
	// query for polygons in bounding box
	// calculate adjacency
    Fbox bb_query;
    Fvector bbc, bbd;
    bb_query.set(contact_point, contact_point);
    bb_query.grow(sz * 2.5f);
    bb_query.get_CD(bbc, bbd);
    xrc.box_options(CDB::OPT_FULL_TEST);
    {
        BloodNativeProfile::Scope profileQuery(BloodNativeProfile::Stage::StaticQuery);
        xrc.box_query(g_pGameLevel->ObjectSpace.GetStaticModel(), bbc, bbd);
    }
    u32 triCount = xrc.r_count();
    if (0 == triCount)
        return;

    u32 real_tcnt = triCount + 1u;
    CDB::TRI* tris = g_pGameLevel->ObjectSpace.GetStaticTris();
    BloodNativeProfile::Scope profileAdjacency(BloodNativeProfile::Stage::StaticAdjacency);
    sml_collector.clear();
    sml_collector.reserve(real_tcnt);
    sml_collector.add_face_packed_D(pVerts[pTri->verts[0]], pVerts[pTri->verts[1]], pVerts[pTri->verts[2]], 0);
    for (u32 t = 0; t < triCount; t++)
    {
        CDB::TRI* T = tris + xrc.r_begin()[t].id;
        if (T == pTri) continue;
        sml_collector.add_face_packed_D(pVerts[T->verts[0]], pVerts[T->verts[1]], pVerts[T->verts[2]], 0);
    }
    sml_collector.calc_adjacency(sml_adjacency);
    profileAdjacency.Stop();

	// calc face normal
	Fvector N;
	N.mknormal(pVerts[pTri->verts[0]], pVerts[pTri->verts[1]], pVerts[pTri->verts[2]]);
	sml_normal.set(N);

	// build 3D ortho-frustum
	Fmatrix mView, mRot;
	BuildMatrix(mView, 1 / sz, contact_point);
	mRot.rotateZ(deg2rad(rotation));
	mView.mulA_43(mRot);
	sml_clipper.CreateFromMatrix(mView,FRUSTUM_P_LRTB);

	// create wallmark
	static_wallmark* W = static_wm_allocate();
    W->m_thermal_blood=BloodThermalWallmark::Compatible(hShader);
    W->m_thermal_stamp=stamp;
	if (ttl) W->m_fTimeEnd = ttl;
	W->verts.reserve(real_tcnt * 3);
	{
		BloodNativeProfile::Scope profileProject(BloodNativeProfile::Stage::StaticProject);
		RecurseTri(0, mView, *W);
	}

	// calc sphere
	if (W->verts.size() < 3)
	{
		static_wm_destroy(W);
		return;
	}
	else
	{
		Fbox bb;
		bb.invalidate();

		FVF::LIT* I = &*W->verts.begin();
		FVF::LIT* E = &*W->verts.end();
		for (; I != E; I++) bb.modify(I->p);
		bb.getsphere(W->bounds.P, W->bounds.R);
	}

    IRender_Sector* S = RImplementation.detectLastSector(W->bounds.P);

	//	if (W->bounds.R < 1.f)	
	{
		// search if similar wallmark exists
		wm_slot* slot = FindSlot(hShader);
		// R23: bounds and expiry are kept beside the pointer (static_entry) for the per-frame sweeps.
		const float expire = W->TimeEnd() == -1.f ? std::numeric_limits<float>::infinity() : W->TimeStart() + W->TimeEnd();
		if (slot)
		{
			StaticWMVec& sector_items = slot->static_items[S];
			for (auto& entry : sector_items)
			{
				if (entry.P.similar(W->bounds.P, 0.02f))
				{
					// replace
					static_wm_destroy(entry.W);
					entry = {W->bounds.P, W->bounds.R, expire, W};
					return;
				}
			}
		}
		else
		{
			slot = AppendSlot(hShader);
		}

		// no similar - register _new_
		slot->static_items[S].push_back({W->bounds.P, W->bounds.R, expire, W});

	}
	//else
	//{
	//	static_wm_destroy(W);
	//}
}

void CWallmarksEngine::AddWallmark_internal(CDB::TRI* pTri, const Fvector* pVerts, const Fvector& contact_point,
	ref_shader hShader, float sz, float ttl, bool random_rotation)
{
	AddWallmark_internal(pTri, pVerts, contact_point, hShader, sz, ttl, random_rotation ? ::Random.randF(-20.f, 20.f) : 0.f);
}

BOOL r_wallmarks_static = TRUE;
BOOL r_wallmarks_dynamic = TRUE;
void CWallmarksEngine::AddStaticWallmark(CDB::TRI* pTri, const Fvector* pVerts, const Fvector& contact_point,
                                         ref_shader hShader, float sz, float ttl, bool ignore_opt, bool random_rotation)
{
	AddStaticWallmark(pTri, pVerts, contact_point, hShader, sz, ttl, ignore_opt, random_rotation ? ::Random.randF(-20.f, 20.f) : 0.f);
}

void CWallmarksEngine::AddStaticWallmark(CDB::TRI* pTri, const Fvector* pVerts, const Fvector& contact_point,
	ref_shader hShader, float sz, float ttl, bool ignore_opt, float rotation)
{
    if (!r_wallmarks_static)
        return;

	// Physics may add wallmarks in parallel with rendering
	BloodNativeProfile::Scope profileLock(BloodNativeProfile::Stage::StaticLockWait);
	lock.Enter();
	profileLock.Stop();
	AddWallmark_internal(pTri, pVerts, contact_point, hShader, sz, ttl, rotation);
	lock.Leave();
}

void CWallmarksEngine::AddStaticBloodWallmark(CDB::TRI* tri,const Fvector* verts,const Fvector& point,
    ref_shader shader,float size,float ttl,bool ignore_opt,float rotation,const BloodThermalStamp& stamp)
{
    if(!r_wallmarks_static)return;
    xrCriticalSectionGuard guard(lock);
    AddWallmark_internal(tri,verts,point,shader,size,ttl,rotation,stamp);
}

void CWallmarksEngine::AddSkeletonWallmark(const Fmatrix* xf, CKinematics* obj, ref_shader& sh, const Fvector& start,
                                           const Fvector& dir, float size, float ttl, bool ignore_opt)
{
    if (!r_wallmarks_dynamic)
        return;

	VERIFY(obj&&xf&&(size>EPS_L));
	BloodNativeProfile::Scope profileLock(BloodNativeProfile::Stage::BodyLockWait);
	lock.Enter();
	profileLock.Stop();
	obj->AddWallmark(xf, start, dir, sh, size, ttl);
	lock.Leave();
}

void CWallmarksEngine::AddSkeletonBloodWallmark(const Fmatrix* xf,CKinematics* obj,ref_shader& shader,
    const Fvector& start,const Fvector& dir,float size,float ttl,bool ignore_opt,const BloodThermalStamp& stamp)
{
    if(!r_wallmarks_dynamic)return;
    VERIFY(obj&&xf&&(size>EPS_L));
    BloodNativeProfile::Scope profileLock(BloodNativeProfile::Stage::BodyLockWait);
    xrCriticalSectionGuard guard(lock);profileLock.Stop();
    obj->AddWallmark(xf,start,dir,shader,size,ttl,nullptr,nullptr,&stamp);
}

void CWallmarksEngine::AddSkeletonWallmark(intrusive_ptr<CSkeletonWallmark> wm)
{
    if (!r_wallmarks_dynamic)
        return;

	lock.Enter			();
	// search if similar wallmark exists
	wm_slot* slot		= FindSlot	(wm->Shader());
	if (0==slot) slot	= AppendSlot(wm->Shader());
	// no similar - register _new_
	slot->skeleton_items.push_back(wm);
#ifdef	DEBUG
	wm->used_in_render	= Device.dwFrame;
#endif
	lock.Leave			();
}

extern float r_ssaDISCARD;
ICF void BeginStream(ref_geom hGeom, u32& w_offset, FVF::LIT*& w_verts, FVF::LIT*& w_start)
{
	PROF_EVENT("BeginStream");
	BloodNativeProfile::Scope profileMap(BloodNativeProfile::Stage::StreamMap);
	w_offset = 0;
	w_verts = (FVF::LIT*)RCache.Vertex.Lock(MAX_TRIS * 3, hGeom->vb_stride, w_offset);
	w_start = w_verts;
}

ICF void FlushStream(ref_geom hGeom, ref_shader shader, u32& w_offset, FVF::LIT*& w_verts, FVF::LIT*& w_start,
                     BOOL bSuppressCull)
{
	PROF_EVENT("FlushStream");
	u32 w_count = u32(w_verts - w_start);
	{
		BloodNativeProfile::Scope profileUnmap(BloodNativeProfile::Stage::StreamUnmap);
		RCache.Vertex.Unlock(w_count, hGeom->vb_stride);
	}
	if (w_count)
	{
		BloodNativeProfile::Scope profileDraw(BloodNativeProfile::Stage::StreamDraw);
		RCache.set_Shader(shader);
		RCache.set_Geometry(hGeom);
		if (bSuppressCull) RCache.set_CullMode(CULL_NONE);
		RCache.Render(D3DPT_TRIANGLELIST, w_offset, w_count / 3);
		if (bSuppressCull) RCache.set_CullMode(CULL_CCW);
		Device.Statistic->RenderDUMP_WMT_Count += w_count / 3;
	}
}

void CWallmarksEngine::CollectSkeletonWallmarks(CKinematics* obj, xr_vector<intrusive_ptr<CSkeletonWallmark>>& output)
{
    xrCriticalSectionGuard g(lock);
    for(wm_slot* slot:marks) for(const auto& wm:slot->skeleton_items)
        if(wm && wm->Parent()==obj) output.push_back(wm);
}

void CWallmarksEngine::RemoveSkeletonWallmarksFromObject(CKinematics* obj)
{
    xrCriticalSectionGuard g(lock);
    for (wm_slot* slot : marks)
    {
        slot->skeleton_items.erase(std::remove_if
        (
            slot->skeleton_items.begin(),
            slot->skeleton_items.end(),
            [obj](const intrusive_ptr<CSkeletonWallmark>& wm) { return wm && wm->Parent() == obj; }
        ),
        slot->skeleton_items.end()
        );
    }
}

// demonized: Remove expired wallmarks before rendering
void CWallmarksEngine::UpdateWallmarks()
{
    if (marks.empty())
        return;

    // Toggle the lock if update is going to be in a separate thread
    // xrCriticalSectionGuard g(lock);
    for (wm_slot* slot : marks)
    {
        // static wallmarks
        for (auto it_sect = slot->static_items.begin(); it_sect != slot->static_items.end();)
        {
            auto& [sector, static_vec] = *it_sect;

            for (u32 i = 0; i < static_vec.size();)
            {
                if (RDEVICE.fTimeGlobal > static_vec[i].expire)
                {
                    static_wm_destroy(static_vec[i].W);
                    static_vec[i] = static_vec.back();
                    static_vec.pop_back();
                    continue;
                }

                i++;
            }

            if (static_vec.empty())
            {
                it_sect = slot->static_items.erase(it_sect);
                continue;
            }

            ++it_sect;
        }

        // dynamic wallmarks
        slot->skeleton_items.erase(
            std::remove_if(
                slot->skeleton_items.begin(),
                slot->skeleton_items.end(),
                [](const intrusive_ptr<CSkeletonWallmark>& wm) { return wm->TimeEnd() != -1.f && RDEVICE.fTimeGlobal > (wm->TimeStart() + wm->TimeEnd()); }
            ),
            slot->skeleton_items.end()
        );
    }
}

float r_wallmarks_ssa_k = 0.5f;
void CWallmarksEngine::Render()
{
#ifdef USE_DX11
    if ((r_blood_gpu == 0 || !r_wallmarks_dynamic) && gpu_skinning) ResetGpuWallmarks();
#endif
	BloodNativeProfile::Scope profileRender(BloodNativeProfile::Stage::RenderTotal, Device.dwFrame);
#ifdef USE_DX11
    // Compile/allocate and prewarm model sources independently of impacts. The
    // impact path never uploads a whole mesh or initializes the compute backend.
    if (r_blood_gpu && r_blood_gpu_create && r_wallmarks_dynamic)
    {
        xrCriticalSectionGuard guard(lock);
        if (!gpu_skinning)
        {
            void* memory = std::malloc(sizeof(BloodGpuWallmarkState));
            if (memory)
            {
                try { gpu_skinning = new (memory) BloodGpuWallmarkState; }
                catch (...) { std::free(memory); }
            }
        }
        if (gpu_skinning && gpu_skinning->projection_warmup_frame != Device.dwFrame)
        {
            auto& state = *gpu_skinning;
            state.projection_warmup_frame = Device.dwFrame;
            // R64 first-shot decal stall (48.5 ms in this scope at the first hit): the skinning backend compiles its compute shader
            // and creates its buffers on first use - now at the first frame the warm-up runs, not at the first wallmark.
            (void)state.backend.EnsureReady();
            // R19: capture stage PainterReserved0 = this warm-up block (source scan, poll, uploads).
            BloodNativeProfile::Scope profileWarmup(BloodNativeProfile::Stage::PainterReserved0, Device.dwFrame);
            try
            {
                for (auto it = state.projection_sources.begin(); it != state.projection_sources.end();)
                {
                    if (it->second.source.use_count() == 1)
                    {
                        state.projection_backend.ReleaseSource(it->second.handle);
                        it = state.projection_sources.erase(it);
                    }
                    else ++it;
                }
                CSkeletonWallmark::ProjectionSource::TakePending(state.projection_pending);
                if (state.projection_backend.EnsureReady())
                {
                    // One nonblocking metadata poll per engine frame. Geometry
                    // stays on the GPU; completed counts reclaim tails and permit
                    // bounded relocation before any cached output is borrowed.
                    state.projection_backend.PollCompletedAllocations();
                    // R19: capture stage PainterReserved1 = the model-source uploads alone.
                    BloodNativeProfile::Scope profileUploads(BloodNativeProfile::Stage::PainterReserved1, Device.dwFrame);
                    blood_projection_warmup::drain(state.projection_pending,
                        [&](const auto* source) { return state.projection_sources.count(source) != 0; },
                        [&](const auto& source) {
                        const u32 count = u32(source->vertices.size());
                        const u32 index_count = u32(source->indices.size());
                        auto inserted = state.projection_sources.try_emplace(source.get());
                        auto& cached = inserted.first->second;
                        cached.source = source;
                        if (state.projection_backend.CanRegisterSource(count, index_count) &&
                            state.projection_backend.RegisterSource(source->vertices.data(), count,
                                source->indices.data(), index_count, cached.handle))
                        {
                            ++state.projection_warmups;
                            if (++g_blood_source_uploads_logged <= 8 || g_blood_source_uploads_logged % 64 == 0) // R25: one line per 64 uploads
                                Msg("[blood_gpu] source_upload #%llu vertices=%u indices=%u bytes=%u frame=%u", static_cast<unsigned long long>(g_blood_source_uploads_logged), count, index_count,
                                    unsigned(size_t(count) * sizeof(source->vertices[0]) + size_t(index_count) * sizeof(source->indices[0])), Device.dwFrame);
                            source->gpu_epoch.store(g_blood_projection_epoch.load(std::memory_order_relaxed),
                                std::memory_order_relaxed);
                        }
                        else
                        {
                            state.projection_sources.erase(inserted.first);
                            ++state.projection_warmup_failures;
                        }
                        },
                        [&]() { ++state.projection_warmup_failures; });
                }
            }
            catch (...) { ++state.projection_warmup_failures; }
        }
    }
#endif
	//	if (marks.empty())			return;
	// Projection and xform
	float _43 = Device.mProject._43;
	Device.mProject._43 -= ps_r__WallmarkSHIFT;
	RCache.set_xform_world(Fidentity);
	RCache.set_xform_project(Device.mProject);

	Fmatrix mSavedView = Device.mView;
	Fvector mViewPos;
	mViewPos.mad(Device.vCameraPosition, Device.vCameraDirection, ps_r__WallmarkSHIFT_V);
	Device.mView.build_camera_dir(mViewPos, Device.vCameraDirection, Device.vCameraTop);
	RCache.set_xform_view(Device.mView);

	Device.Statistic->RenderDUMP_WM.Begin();
	Device.Statistic->RenderDUMP_WMS_Count = 0;
	Device.Statistic->RenderDUMP_WMD_Count = 0;
	Device.Statistic->RenderDUMP_WMT_Count = 0;

    constexpr const u32 max_verts = MAX_TRIS * 3;

    if ((r_wallmarks_static || r_wallmarks_dynamic) && !marks.empty())
    {
        const float ssaCLIP = r_ssaDISCARD * r_wallmarks_ssa_k;
        BloodNativeProfile::Scope profileLock(BloodNativeProfile::Stage::RenderLockWait);
        xrCriticalSectionGuard g(lock); // Physics may add wallmarks in parallel with rendering
        profileLock.Stop();

#ifdef USE_DX11
        if (r_blood_gpu && r_wallmarks_dynamic && !gpu_skinning)
        {
            void* memory = std::malloc(sizeof(BloodGpuWallmarkState));
            if (memory)
            {
                try { gpu_skinning = new (memory) BloodGpuWallmarkState; }
                catch (...) { std::free(memory); }
            }
            if (!gpu_skinning) Msg("! Blood GPU: optional state allocation failed; using CPU");
        }
        const bool gpu_ready = r_blood_gpu && gpu_skinning && gpu_skinning->backend.EnsureReady();
        u32 resolve_vertex_budget = 8192;
        auto resolve_mark = [&](const intrusive_ptr<CSkeletonWallmark>& W) -> bool
        {
            if (!W->m_Projection) return true;
            u32 cost = 0;
            for (const auto& range : W->m_Projection->ranges) cost += range.vertex_count;
            std::shared_ptr<PainterWallmarkOwnershipGpu> ownership;const bool uniform=W->ProjectionOwnership(ownership);
            if (cost > 65536 && W->m_Faces.empty() && uniform) return false; // R26: too large for the render thread; it keeps its projection path
            // R37: the budget is spent strictly (the first mark of a frame no longer resolves whatever its size: one
            // 60 K-vertex spray mark was a 20 ms frame after every publication); a mark larger than what is left
            // continues next frame from where it stopped (resumable resolve).
            if (!resolve_vertex_budget) return false;
            BloodNativeProfile::Scope profileResolve(BloodNativeProfile::Stage::WallmarkResolve, Device.dwFrame);
            const u32 before = resolve_vertex_budget;
            const bool done = W->ResolveProjection(&resolve_vertex_budget);
            g_painter_resolve_vertices += before - resolve_vertex_budget;
            if (done) ++g_painter_marks_resolved;
            return done;
        };
        if (gpu_skinning)
        {
            auto& state = *gpu_skinning;
            state.plans.clear();
            state.refresh.clear();
            state.slot_plans.assign(marks.size(), BloodGpuWallmarkState::PlanRange{});
            state.frame_output = {};
            state.frame_batch_open = false;
            state.frame_batch_failed = false;
            const u32 frame = Device.dwFrame;
            u32 plan_gpu = 0, plan_cpu = 0, plan_skip = 0;
            auto wallmark_color = [](const intrusive_ptr<CSkeletonWallmark>& W) -> u32
            {
                const float w = W->TimeEnd() == -1.f ? 0.f : (RDEVICE.fTimeGlobal - W->TimeStart()) / W->TimeEnd();
                const float alpha = W->m_Growth.opacity == 1.f ? w : W->m_Growth.Fade(RDEVICE.fTimeGlobal - W->TimeStart(), W->TimeEnd());
                int aC = iFloor(alpha * 255.f);
                clamp(aC, 0, 255);
                return BloodThermalWallmark::Color(RDEVICE.fTimeGlobal-W->TimeStart(),aC,
                    ps_r2_heatvision>0 && BloodThermalWallmark::Compatible(W->Shader()),W->m_thermal_stamp);
            };
            auto open_batch = [&]()
            {
                if (!state.frame_batch_open)
                {
                    state.frame_batch_open = state.backend.BeginBatch();
                    if (!state.frame_batch_open) ++state.fb_batch;
                }
                return state.frame_batch_open;
            };
            // One GPU job for a mark: its persistent source (faces in rest space, uploaded once), the body's palette
            // for this frame (appended once per body) and a job descriptor. Every refusal is counted.
            auto append_frame_job = [&](const intrusive_ptr<CSkeletonWallmark>& W, CKinematics* body,
                const std::shared_ptr<PainterRemovalGpu>& removal,const std::shared_ptr<PainterWallmarkOwnershipGpu>&ownership,const ref_shader& masked,
                BloodGpuWallmarkState::Plan& plan) -> int
            {
                const u32 vertex_count = u32(W->m_Faces.size() * 3);
                if (!vertex_count) { ++state.fb_no_vertices; return 0; }
                if (vertex_count > BloodGpuSkinning::Backend::MaxBatchVertices) { ++state.fb_too_large; return 0; }
                if (!body->CaptureWallmarkPose(frame, *W->XFORM())) { ++state.fb_no_snapshot; return 0; }
                const u32 bone_count = u32(body->m_WallmarkPose.size());
                if (!bone_count) { ++state.fb_no_bones; return 0; }
                if (!open_batch()) return 0;
                try
                {
                    auto& source = state.sources.try_emplace(W.get()).first->second;
                    source.mark = W;
                    source.seen_frame = frame;
                    const u64 visible = body->LL_BoneVisibleMask();
                    const bool stale = source.faces != W->m_Faces.data() || source.face_count != W->m_Faces.size() ||
                        source.bone_count != bone_count || source.geometry_revision != W->m_RenderGeometryRevision ||
                        source.visimask != visible;
                    if (stale || (source.vertex_count && !state.backend.IsValid(source.handle)))
                    {
                        state.backend.ReleaseSource(source.handle);
                        if (!state.backend.CanRegisterSource(vertex_count)) { ++state.source_capacity_rejects; return 0; }
                        state.upload.resize(vertex_count);
                        size_t exported = 0;
                        if (!body->ExportWallmarkGpuFaces(*W, state.upload.data(), state.upload.size(), exported)) { ++state.fb_export; return 0; }
                        source.faces = W->m_Faces.data();
                        source.face_count = W->m_Faces.size();
                        source.bone_count = u16(bone_count);
                        source.geometry_revision = W->m_RenderGeometryRevision;
                        source.visimask = visible;
                        source.vertex_count = u32(exported);
                        if (exported && !state.backend.RegisterSource(state.upload.data(), u32(exported), source.handle))
                        { source.vertex_count = 0; source.visimask = ~visible; ++state.fb_export; return 0; }
                    }
                    if (!source.vertex_count) return -1; // R26: every face sits on a hidden (cut off) bone: nothing to draw
                    if (body->m_WallmarkPaletteFrame != frame)
                    {
                        state.palette.resize(bone_count);
                        for (u32 b = 0; b < bone_count; ++b)
                            state.palette[b] = BloodGpuSkinning::FromMatrix(body->m_WallmarkPose[b]);
                        const u32 offset = state.backend.AppendPalette(state.palette.data(), bone_count);
                        if (offset == BloodGpuSkinning::Backend::NoPalette) { ++state.fb_batch; return false; }
                        body->m_WallmarkPaletteFrame = frame;
                        body->m_WallmarkPaletteOffset = offset;
                    }
                    if (!state.backend.CanAppendJob(source.vertex_count)) { ++state.fb_batch; return 0; }
                    plan.first_vertex = state.backend.PendingVertexCount();
                    plan.vertex_count = source.vertex_count;
                    u32 dent_count = 0; const auto dent_texels = body->LL_RuntimeDentTexels(dent_count); // melee (value 3): on the body's dents
                    if (!state.backend.AppendJob(source.handle, 0, source.vertex_count, body->m_WallmarkPaletteOffset, bone_count,
                        BloodGpuSkinning::FromMatrix(*W->XFORM()), wallmark_color(W), dent_texels.get(), dent_count))
                    { ++state.fb_append; return 0; }
                }
                catch (...) { ++state.fb_exception; return 0; }
                plan.field = removal.get();plan.ownership=ownership.get();
                plan.masked = masked;
                if (removal||ownership) ++state.masked_marks;
                ++state.gpu_marks;
                W->XFORM()->transform_tiny(W->m_Bounds.P, W->m_LocalBounds.P);
                return 1;
            };
            for (u32 i = 0; i < marks.size(); ++i)
            {
                wm_slot* slot = marks[i];
                if (!r_wallmarks_dynamic || slot->skeleton_items.empty()) continue;
                const auto& items = slot->skeleton_items;
                // Drawn grouped by body (stable: creation order kept within a body) so one body's marks form one
                // contiguous run. Order across bodies is free: decals of different meshes never share a surface.
                auto& order = state.order;
                order.resize(items.size());
                for (u32 k = 0; k < order.size(); ++k) order[k] = k;
                std::stable_sort(order.begin(), order.end(), [&items](u32 a, u32 b)
                {
                    const CKinematics* pa = items[a] ? items[a]->Parent() : nullptr;
                    const CKinematics* pb = items[b] ? items[b]->Parent() : nullptr;
                    return reinterpret_cast<uintptr_t>(pa) < reinterpret_cast<uintptr_t>(pb);
                });
                const u32 first_plan = u32(state.plans.size());
                CKinematics* mask_body = nullptr;
                CKinematics* removal_body = nullptr; // R23: one removal-field lookup per body run, not two per mark
                std::shared_ptr<PainterRemovalGpu> removal_cached;
                PainterWallmarkTear tear_cached; // maskdecal: with the removal field, once per body run
                const void* mask_tear = nullptr;
                std::shared_ptr<PainterRemovalGpu> mask_field;
                std::shared_ptr<PainterWallmarkOwnershipGpu> mask_ownership;
                ref_shader mask_shader;
                bool mask_ok = false;
                for (u32 k = 0; k < order.size(); ++k)
                {
                    const u32 item = order[k];
                    intrusive_ptr<CSkeletonWallmark> W = items[item];
                    if (!W) continue;
                    if (W->m_Projection)
                    {
                        const auto projected = state.projection_marks.find(W.get());
                        if (projected != state.projection_marks.end()) projected->second.seen_frame = frame;
                    }
                    if (!RImplementation.ViewBase.testSphere_dirty(W->m_Bounds.P, W->m_Bounds.R)) { ++plan_skip; continue; }
                    Device.Statistic->RenderDUMP_WMD_Count++;
                    const float dst = Device.vCameraPosition.distance_to_sqr(W->m_Bounds.P);
                    const float ssa = W->m_Bounds.R * W->m_Bounds.R / dst;
                    if (ssa < ssaCLIP) { ++plan_skip; continue; }
                    CKinematics* const body = W->Parent();
                    if (body != removal_body) { removal_body = body; removal_cached = body->LL_RuntimeRemovalGpu(); body->LL_RuntimeTearMask(tear_cached); }
                    if (removal_cached)
                    {
                        const painter_detail::DamageField* field = removal_cached->field.get();
                        if (field && field->available && W->m_RemovalChecked != static_cast<const void*>(field))
                        {
                            const bool ready = !W->m_Faces.empty() || resolve_mark(W); // R37: carried faces are tested now
                            if (ready)
                            {
                                W->m_RemovalChecked = field;
                                bool gone = !W->m_Faces.empty();
                                for (const auto& face : W->m_Faces)
                                {
                                    // A primitive-local interior can remain while all original corners are removed.
                                    if(face.ownership){gone=false;break;}
                                    for (u32 k = 0; k < 3 && gone; ++k)
                                        if (!field->sample(face.vert[k].x, face.vert[k].y, face.vert[k].z)) gone = false;
                                    if (!gone) break;
                                }
                                if (gone)
                                {
                                    W->Expire();
                                    ++g_painter_marks_removed;
                                    ++plan_skip;
                                    continue;
                                }
                            }
                        }
                    }
                    // R23: resolve the remaining GPU-created marks (budgeted) so they join the batch below.
                    // R37: a carried mark (faces present, refresh pending) draws its faces now and is refreshed after every
                    // fresh mark with the budget that is left (below the slot loop); a fresh mark resolves here.
                    if (W->m_Projection)
                    {
                        if (W->m_Faces.empty()) resolve_mark(W); else state.refresh.push_back(W);
                    }
                    // R21: a mark whose projection resolved to no face can never draw.
                    if (!W->m_Projection && W->m_CarryRanges && W->m_Faces.empty())
                    {
                        W->Expire();
                        ++g_painter_marks_empty;
                        ++plan_skip;
                        continue;
                    }
                    BloodGpuWallmarkState::Plan plan;
                    plan.item = item;
                    if (gpu_ready && (!W->m_Projection || !W->m_Faces.empty()) && !W->m_Growth.Growing(RDEVICE.fTimeGlobal - W->TimeStart()))
                    {
                        const std::shared_ptr<PainterRemovalGpu>& removal = removal_cached;
                        std::shared_ptr<PainterWallmarkOwnershipGpu> ownership;
                        const bool uniform=W->FaceOwnership(ownership);
                        plan.tear = tear_cached.keep.get();
                        if (body != mask_body || removal != mask_field || ownership!=mask_ownership || plan.tear != mask_tear)
                        {
                            mask_body = body;
                            mask_field = removal;mask_ownership=ownership;mask_tear=plan.tear;
                            mask_shader = ref_shader();
                            mask_ok = true;
                            if (removal||ownership)
                            {
                                xr_string reason;
                                mask_ok = PainterWallmarkMask::Prepare(*DEV, removal,ownership,slot->shader,mask_shader,reason,&tear_cached);
                            }
                        }
                        if (!mask_ok||!uniform) ++state.fb_mask;
                        else
                        {
                            const int appended = append_frame_job(W,body,removal,ownership,mask_shader,plan);
                            if (appended > 0) plan.kind = BloodGpuWallmarkState::Plan::Gpu;
                            else if (appended < 0) { ++plan_skip; continue; } // R26: nothing left to draw on this mark
                        }
                        if (plan.kind != BloodGpuWallmarkState::Plan::Gpu) ++state.cpu_fallback;
                    }
                    if (plan.kind == BloodGpuWallmarkState::Plan::Gpu) ++plan_gpu; else ++plan_cpu;
                    state.plans.push_back(plan);
                }
                state.slot_plans[i] = {first_plan, u32(state.plans.size()) - first_plan};
            }
            // R37: background refresh of carried marks with the resolve budget left after the fresh ones (this frame
            // still draws the faces already exported; the refreshed faces are exported next frame).
            for (const auto& W : state.refresh)
            {
                if (!resolve_vertex_budget) break;
                if (W->m_Projection && resolve_mark(W)) ++g_painter_marks_refreshed;
            }
            state.refresh.clear();
            if (state.frame_batch_open)
            {
                if (state.backend.PendingVertexCount() && !state.backend.Dispatch(state.frame_output))
                {
                    state.frame_batch_failed = true;
                    ++state.failed_batches;
                }
                state.frame_batch_open = false;
            }
            g_painter_plan_gpu = plan_gpu;
            g_painter_plan_cpu = plan_cpu;
            g_painter_plan_skip = plan_skip;
            g_painter_plan_vertices = state.frame_output.vertex_count;
        }
#endif

        for (int i = 0; i < marks.size(); i++)
        {
            wm_slot* slot = marks[i];
#if defined(USE_DX11)
            RCache.set_RT(ps_r2_heatvision>0 && BloodThermalWallmark::Compatible(slot->shader) ?
                RImplementation.Target->rt_Heat->pRT : nullptr,1);
#endif
            bool static_empty = !r_wallmarks_static || slot->static_items.empty();
            bool skeleton_empty = !r_wallmarks_dynamic || slot->skeleton_items.empty();
            if (static_empty && skeleton_empty)
                continue;

            u32 w_offset = 0;
            FVF::LIT* w_verts = nullptr;
            FVF::LIT* w_start = nullptr;
            bool stream_open = false;
            auto begin_stream = [&]()
            {
                if (!stream_open)
                {
                    BeginStream(hGeom, w_offset, w_verts, w_start);
                    stream_open = true;
                }
            };
            auto flush_stream = [&](BOOL suppress_cull)
            {
                if (stream_open)
                {
                    FlushStream(hGeom, slot->shader, w_offset, w_verts, w_start, suppress_cull);
                    stream_open = false;
                }
            };

            auto prepare_large_mark = [&](size_t vertex_count)
            {
                // xr_vector's allocator uses a u32 element count. Reserve the
                // exact requested size before resize to avoid unchecked growth.
                if (vertex_count > (std::numeric_limits<u32>::max)() ||
                    vertex_count > large_wallmark_vertices.max_size())
                    return false;
                if (large_wallmark_vertices.capacity() < vertex_count)
                    large_wallmark_vertices.reserve(vertex_count);
                if (large_wallmark_vertices.size() < vertex_count)
                    large_wallmark_vertices.resize(vertex_count);
                return true;
            };
            auto stream_large_mark = [&](size_t vertex_count, BOOL suppress_cull)
            {
                if (vertex_count > large_wallmark_vertices.size() || vertex_count % 3 != 0)
                {
                    Msg("! Invalid oversized wallmark vertex span");
                    return;
                }
                size_t offset = 0;
                while (offset < vertex_count)
                {
                    // The caller flushed before staging; each further chunk
                    // starts in an empty reservation and contains whole faces.
                    begin_stream();
                    const u32 count = static_cast<u32>((std::min)(
                        vertex_count - offset, static_cast<size_t>(max_verts)));
                    std::copy_n(large_wallmark_vertices.data() + offset, count, w_verts);
                    w_verts += count;
                    offset += count;
                    if (offset < vertex_count)
                        flush_stream(suppress_cull);
                }
            };

            auto render_prepared = [&](intrusive_ptr<CSkeletonWallmark> W, FVF::LIT*& vertices,
                const CKinematics::WallmarkRenderPreparation* prepared)
            {
                if (prepared) W->Parent()->RenderWallmarkPrepared(W, vertices, *prepared);
                else W->Parent()->RenderWallmark(W, vertices);
            };
            auto render_cpu_mark = [&](intrusive_ptr<CSkeletonWallmark> W,
                const CKinematics::WallmarkRenderPreparation* prepared)
            {
#ifdef USE_DX11
                    if (W->m_Projection && W->m_Faces.empty()) // R37: a carried mark draws its faces; its refresh is budgeted in the plan loop
                    {
                        // Any disabled/cold/full/unsupported/reset path resolves
                        // from immutable impact data, then uses legacy rendering.
                        if (!resolve_mark(W)) return; // R26: through the per-frame budget (a refused mark waits a frame)
                        prepared = nullptr;
                    }
#endif
#ifdef USE_DX11
                    std::shared_ptr<PainterWallmarkOwnershipGpu> singleOwnership;
                    const bool uniformOwnership=W->FaceOwnership(singleOwnership);
                    auto removal=W->Parent()->LL_RuntimeRemovalGpu();
                    PainterWallmarkTear tear;if(removal)W->Parent()->LL_RuntimeTearMask(tear); // maskdecal
                    if(removal||singleOwnership||!uniformOwnership){
                        flush_stream(TRUE);
                        CKinematics::WallmarkRenderPreparation local;
                        if(!prepared){if(!W->Parent()->PrepareWallmarkRender(W,local))return;prepared=&local;}
                        const size_t count=W->m_Faces.size()*size_t(3);
                        if(!prepare_large_mark(count))return;
                        FVF::LIT*end=large_wallmark_vertices.data();render_prepared(W,end,prepared);
                        if(size_t(end-large_wallmark_vertices.data())!=count)return;
                        u32 dentN=0;const auto dentT=W->Parent()->LL_RuntimeDentTexels(dentN); // melee (value 3): the rest position on the dents
                        struct MaskedVertex {FVF::LIT lit;Fvector rest;float ownership[2];};
                        static_assert(sizeof(MaskedVertex)==BloodGpuProjection::Output::vertex_stride,"masked wallmark ABI");
                        for(size_t first=0;first<count;){
                            const auto ownership=W->m_Faces[first/3].ownership_range==PainterWallmarkOwnership::unrestricted?
                                std::shared_ptr<PainterWallmarkOwnershipGpu>{}:W->m_Faces[first/3].ownership;
                            size_t end=first+3;while(end<count){const auto&f=W->m_Faces[end/3];const auto next=f.ownership_range==PainterWallmarkOwnership::unrestricted?
                                std::shared_ptr<PainterWallmarkOwnershipGpu>{}:f.ownership;if(next!=ownership)break;end+=3;}
                            if(W->m_Faces[first/3].ownership_range!=PainterWallmarkOwnership::unrestricted&&!ownership){first=end;continue;}
                            ref_shader masked=slot->shader;xr_string reason;
                            if((removal||ownership)&&!PainterWallmarkMask::Prepare(*DEV,removal,ownership,slot->shader,masked,reason,&tear)){
                                painter_mask_refusal_log("required mask refused",reason);first=end;continue;}
                            const u32 n=u32((std::min)(end-first,size_t(max_verts)));u32 offset=0;
                            auto*dst=static_cast<MaskedVertex*>(RCache.Vertex.Lock(n,sizeof(MaskedVertex),offset));
                            for(u32 k=0;k<n;++k){const auto index=first+k;dst[k].lit=large_wallmark_vertices[index];{Fvector rp=W->m_Faces[index/3].vert[index%3];if(dentT)painter_dent::apply(dentT.get(),dentN,rp.x,rp.y,rp.z);dst[k].rest=rp;}const auto&r=W->m_Faces[index/3].ownership_range;dst[k].ownership[0]=r[0]==~0u?-1.f:float(r[0]);dst[k].ownership[1]=float(r[1]);}
                            RCache.Vertex.Unlock(n,sizeof(MaskedVertex));
                            RCache.set_Shader(masked);RCache.set_Geometry(hGeomMasked);RCache.set_CullMode(CULL_NONE);
                            RCache.Render(D3DPT_TRIANGLELIST,offset,n/3);RCache.set_CullMode(CULL_CCW);
                            Device.Statistic->RenderDUMP_WMT_Count+=n/3;first+=n;
                        }
                        return;
                    }
#endif
                    const size_t face_count = W->m_Faces.size();
                    if (face_count > static_cast<size_t>(MAX_TRIS))
                    {
                        flush_stream(TRUE);
                        // Stage the entire mark before drawing any of its
                        // chunks. A skinning exception therefore submits none
                        // of this mark, matching the normal path's rollback.
                        size_t staged_written = 0;
                        try
                        {
                            if (face_count > (std::numeric_limits<u32>::max)() / 3)
                                Msg("! Failed to render dynamic wallmark");
                            else
                            {
                                const size_t vertex_count = face_count * size_t(3);
                                if (prepare_large_mark(vertex_count))
                                {
                                    FVF::LIT* staged_end = large_wallmark_vertices.data();
                                    render_prepared(W, staged_end, prepared);
                                    // Missing bones can return without writing.
                                    const size_t written = static_cast<size_t>(staged_end -
                                        large_wallmark_vertices.data());
                                    if (written <= vertex_count && written % 3 == 0)
                                        staged_written = written;
                                    else
                                        Msg("! Invalid oversized dynamic wallmark vertex span");
                                }
                                else
                                    Msg("! Failed to render dynamic wallmark");
                            }
                        }
                        catch (...)
                        {
                            Msg("! Failed to render dynamic wallmark");
                        }
                        // Stream failures retain the normal rendering path's
                        // behavior; only CPU staging is caught above.
                        stream_large_mark(staged_written, TRUE);
                    }
                    else
                    {
                        const u32 vertex_count = static_cast<u32>(face_count * 3);
                        u32 w_count = stream_open ? u32(w_verts - w_start) : 0;
                        if ((w_count + vertex_count) >= max_verts)
                            flush_stream(TRUE);

                        begin_stream();
                        FVF::LIT* w_save = w_verts;
                        try
                        {
                            render_prepared(W, w_verts, prepared);
                        }
                        catch (...)
                        {
                            Msg("! Failed to render dynamic wallmark");
                            w_verts = w_save;
                        }
                    }
            };
#ifdef USE_DX11
            // R22: GPU skinning jobs are appended before the slot loop (one batch per frame); see the frame plan.

            auto flush_projection = [&]()
            {
                if (!gpu_skinning || gpu_skinning->projection_draws.empty()) return;
                auto& state = *gpu_skinning;
                const bool updates = std::any_of(state.projection_draws.begin(), state.projection_draws.end(),
                    [](const auto& pending) { return pending.update; });
                bool ready = !updates || state.projection_backend.DispatchRenderBatch();
                // Validate every output before drawing any part of the run.
                // A resource failure therefore replays the whole run once.
                if (ready)
                    for (auto& pending : state.projection_draws)
                        if (!state.projection_backend.GetOutput(pending.handle, pending.output))
                        {
                            ready = false;
                            break;
                        }
                if(ready)for(auto&pending:state.projection_draws){
                    auto removal=pending.mark->Parent()->LL_RuntimeRemovalGpu();std::shared_ptr<PainterWallmarkOwnershipGpu>ownership;
                    PainterWallmarkTear tear;if(removal)pending.mark->Parent()->LL_RuntimeTearMask(tear); // maskdecal
                    if(!pending.mark->ProjectionOwnership(ownership)){ready=false;break;}
                    if(removal||ownership){ref_shader checked;xr_string reason;
                        if(!PainterWallmarkMask::Prepare(*DEV,removal,ownership,slot->shader,checked,reason,&tear)){ready=false;break;}}
                }
                if (ready)
                {
                    for (auto& pending : state.projection_draws)
                    {
                        auto& cached = state.projection_marks.at(pending.mark.get());
                        cached.output_valid = true;
                        cached.output_color = pending.color;
                    }
                    // Cached and updated marks share this original draw sequence.
                    // No CPU replay can be requested after the first draw is issued.
                    try
                    {
                        RCache.set_Shader(slot->shader);
                        RCache.set_Geometry(hGeom);
                        RCache.set_CullMode(CULL_NONE);
                        for (auto& pending : state.projection_draws)
                        {
                            ref_shader draw_shader=slot->shader;auto removal=pending.mark->Parent()->LL_RuntimeRemovalGpu();
                            std::shared_ptr<PainterWallmarkOwnershipGpu>ownership;
                            PainterWallmarkTear tear;if(removal)pending.mark->Parent()->LL_RuntimeTearMask(tear); // maskdecal
                            if(!pending.mark->ProjectionOwnership(ownership))continue;
                            if(removal||ownership){xr_string reason;if(!PainterWallmarkMask::Prepare(*DEV,removal,ownership,slot->shader,draw_shader,reason,&tear)){
                                painter_mask_refusal_log("mask lost after admission", reason);continue;}}
                            RCache.set_Shader(draw_shader);RCache.set_Geometry((removal||ownership)?hGeomMasked:hGeom);
                            RCache.set_Vertices(pending.output.vertices, BloodGpuProjection::Output::vertex_stride);
                            RCache.RenderIndirect(D3DPT_TRIANGLELIST, pending.output.arguments,
                                pending.output.arguments_byte_offset);
                            pending.mark->m_RenderWorld.transform_tiny(
                                pending.mark->m_Bounds.P, pending.mark->m_LocalBounds.P);
                        }
                        RCache.set_CullMode(CULL_CCW);
                        RCache.set_Vertices(RCache.Vertex.Buffer(), hGeom->vb_stride);
                    }
                    catch (...)
                    {
                        // Cleanup only: propagate the original failure without
                        // replay, and never carry this draw run into another frame.
                        for (auto& pending : state.projection_draws)
                            state.projection_marks.at(pending.mark.get()).queued = false;
                        state.projection_draws.clear();
                        state.projection_backend.BeginRenderBatch();
                        throw;
                    }
                }
                else
                {
                    for (auto& pending : state.projection_draws)
                    {
                        state.projection_marks.at(pending.mark.get()).output_valid = false;
                        ++state.projection_cpu_fallbacks;
                        render_cpu_mark(pending.mark, nullptr);
                    }
                    flush_stream(TRUE);
                }
                for (auto& pending : state.projection_draws)
                    state.projection_marks.at(pending.mark.get()).queued = false;
                state.projection_draws.clear();
                state.projection_backend.BeginRenderBatch();
            };

            auto queue_projection = [&](intrusive_ptr<CSkeletonWallmark> W)
            {
                if (!r_blood_gpu || !r_blood_gpu_create || !gpu_skinning || !W->m_Projection ||
                    W->m_Projection->epoch != g_blood_projection_epoch.load(std::memory_order_relaxed))
                    return false;
                std::shared_ptr<PainterWallmarkOwnershipGpu>ownership;
                if(!W->ProjectionOwnership(ownership))return false;
                auto removal=W->Parent()->LL_RuntimeRemovalGpu();if(removal||ownership){
                    ref_shader masked;xr_string reason;PainterWallmarkTear tear;if(removal)W->Parent()->LL_RuntimeTearMask(tear); // maskdecal
                    if(!PainterWallmarkMask::Prepare(*DEV,removal,ownership,slot->shader,masked,reason,&tear))return false;
                }
                auto& state = *gpu_skinning;
                // Both pose storage and the persistent output span belong to W.
                // Consume a duplicate before another preparation can mutate either.
                const auto prior = state.projection_marks.find(W.get());
                if (prior != state.projection_marks.end() && prior->second.queued) flush_projection();
                if (state.projection_draws.size() >= BloodGpuProjection::Backend::MaxBatchJobs) flush_projection();
                if (!W->m_Projection) return false; // A failed duplicate run resolved it to CPU.
                CKinematics::WallmarkRenderPreparation prepared;
                BloodGpuWallmarkState::ProjectionMark* projected_ptr = nullptr;
                try
                {
                    if (!W->Parent()->PrepareWallmarkRender(W, prepared) || !prepared.pose_snapshot_valid)
                        return false;
                    auto& projected = state.projection_marks.try_emplace(W.get()).first->second;
                    projected_ptr = &projected;
                    projected.mark = W;
                    projected.seen_frame = Device.dwFrame;
                    if (!state.projection_backend.IsValid(projected.handle))
                    {
                        const auto& hit = *W->m_Projection;
                        state.projection_ranges.clear();
                        state.projection_ranges.reserve(hit.ranges.size());
                        for (const auto& range : hit.ranges)
                        {
                            const auto found = state.projection_sources.find(range.source.get());
                            if (found == state.projection_sources.end() ||
                                !state.projection_backend.IsValid(found->second.handle)) return false;
                            state.projection_ranges.push_back(
                                {found->second.handle, range.first_vertex, range.vertex_count});
                        }
                        const u32 range_count = u32(state.projection_ranges.size());
                        const u32 bone_count = u32(hit.hit_pose.size());
                        if (!state.projection_backend.CanCreateMark(
                            state.projection_ranges.data(), range_count, bone_count)) return false;
                        state.palette.resize(bone_count);
                        for (u32 b = 0; b < bone_count; ++b)
                            state.palette[b] = BloodGpuSkinning::FromMatrix(hit.hit_pose[b]);
                        BloodGpuProjection::Projection projection{};
                        projection.view = BloodGpuSkinning::FromMatrix(hit.view);
                        projection.contact[0] = W->ContactPoint().x;
                        projection.contact[1] = W->ContactPoint().y;
                        projection.contact[2] = W->ContactPoint().z;
                        projection.size = hit.size;
                        projection.unused = hit.square_footprint ? 1.f : 0.f;
                        // A surface call's own depth limit (2026-11-12): |unused| = 2 + limit, sign = footprint kind.
                        if (hit.outer_depth > 0.f) projection.unused = (hit.square_footprint ? 1.f : -1.f) * (2.f + hit.outer_depth);
                        projection.normal[0] = hit.normal.x;
                        projection.normal[1] = hit.normal.y;
                        projection.normal[2] = hit.normal.z;
                        if (!state.projection_backend.CreateMark(state.projection_ranges.data(), range_count,
                            state.palette.data(), bone_count, projection, projected.handle)) return false;
                    }
                }
                catch (...) { return false; }
                auto& projected = *projected_ptr;
                const bool update = !projected.output_valid || prepared.pose_changed ||
                    projected.output_color != prepared.color;
                if (update) projected.output_valid = false;
                const u32 bone_count = u32(W->m_RenderBones.size());
                // A capacity flush may draw an earlier run, so it must remain
                // outside every preparation catch that can request CPU fallback.
                if (update && !state.projection_backend.CanAppendRender(bone_count)) flush_projection();
                if (state.projection_draws.empty() && !state.projection_backend.BeginRenderBatch()) return false;
                if (update && !state.projection_backend.CanAppendRender(bone_count)) return false;
                try
                {
                    if (update)
                    {
                        state.palette.resize(bone_count);
                        for (u32 b = 0; b < bone_count; ++b)
                            state.palette[b] = BloodGpuSkinning::FromMatrix(W->m_RenderBones[b].transform);
                    }
                    // Reserve fallback ownership and draw order before the backend
                    // accepts any job. The backend copies this palette immediately.
                    u32 dent_count = 0; std::shared_ptr<const float> dent_texels; // melee (value 3): on the body's dents
                    if (update) dent_texels = W->Parent()->LL_RuntimeDentTexels(dent_count);
                    state.projection_draws.push_back({W, projected.handle, {}, prepared.color, update});
                    if (update && !state.projection_backend.AppendRender(projected.handle, state.palette.data(),
                        bone_count, BloodGpuSkinning::FromMatrix(W->m_RenderWorld), prepared.color, dent_texels.get(), dent_count))
                    {
                        state.projection_draws.pop_back();
                        return false;
                    }
                    projected.queued = true;
                    return true;
                }
                catch (...) { return false; }
            };
#endif

            // static wallmarks
            if (!static_empty)
            {
                BloodNativeProfile::Scope profileStatic(BloodNativeProfile::Stage::RenderStatic);
                PROF_EVENT("STATIC_WALLMARKS");
                for (auto& [sector, static_vec] : slot->static_items)
                {
                    if (!RImplementation.GMBase.is_sector_visible(static_cast<CSector*>(sector)))
                        continue;

                    for (size_t j = 0; j < static_vec.size(); j++)
                    {
                        auto& entry = static_vec[j];
                        if (!RImplementation.ViewBase.testSphere_dirty(entry.P, entry.R))
                            continue;

                        Device.Statistic->RenderDUMP_WMS_Count++;
                        static_wallmark* W = entry.W;
                        float dst = Device.vCameraPosition.distance_to_sqr(entry.P);
                        float ssa = entry.R * entry.R / dst;
                        if (ssa >= ssaCLIP)
                        {
                            if (W->verts.size() > max_verts)
                            {
                                flush_stream(FALSE);
                                if (prepare_large_mark(W->verts.size()))
                                {
                                    FVF::LIT* staged_end = large_wallmark_vertices.data();
                                    // Keep the original static fade/color calculation.
                                    static_wm_render(W, staged_end);
                                    stream_large_mark(static_cast<size_t>(staged_end -
                                        large_wallmark_vertices.data()), FALSE);
                                }
                                else
                                    Msg("! Failed to stage oversized static wallmark");
                                continue;
                            }
                            u32 w_count = stream_open ? u32(w_verts - w_start) : 0;
                            if ((w_count + W->verts.size()) >= max_verts)
                            {
                                flush_stream(FALSE);
                            }
                            begin_stream();
                            static_wm_render(W, w_verts);
                        }
                    }
                }

                if (!skeleton_empty)
                {
                    // Flush stream
                    flush_stream(FALSE); //. remove line if !(suppress cull needed)
                }
            }

            // dynamic wallmarks
            if (!skeleton_empty)
            {
                BloodNativeProfile::Scope profileBodies(BloodNativeProfile::Stage::RenderBody);
                PROF_EVENT("DYNAMIC_WALLMARKS");
#ifdef USE_DX11
                // R22: GPU-skinned marks were batched and dispatched before the slot loop (one batch per frame);
                // here a run of consecutive marks of one body is one draw over its contiguous output range.
                auto draw_gpu_run = [&](const BloodGpuWallmarkState::Plan& plan, u32 first_vertex, u32 vertex_count)
                {
                    BloodNativeProfile::Scope profileDraw(BloodNativeProfile::Stage::StreamDraw);
                    const bool masked = plan.field != nullptr||plan.ownership!=nullptr;
                    ref_shader shader = masked ? plan.masked : slot->shader;
                    RCache.set_Shader(shader);
                    RCache.set_Geometry(masked ? hGeomMasked : hGeom);
                    RCache.set_Vertices(gpu_skinning->frame_output.vertices, BloodGpuSkinning::Output::vertex_stride);
                    RCache.set_CullMode(CULL_NONE);
                    RCache.Render(D3DPT_TRIANGLELIST, first_vertex, vertex_count / 3);
                    RCache.set_CullMode(CULL_CCW);
                    RCache.set_Vertices(RCache.Vertex.Buffer(), hGeom->vb_stride);
                    if (masked) ++gpu_skinning->masked_batches;
                    Device.Statistic->RenderDUMP_WMT_Count += vertex_count / 3;
                };
                // The CPU paths, unchanged: growing marks, GPU-created (projection) marks, refusals.
                auto render_cpu_plan = [&](intrusive_ptr<CSkeletonWallmark> W)
                {
                    if (W->m_Growth.Growing(RDEVICE.fTimeGlobal-W->TimeStart()))
                    {
                        flush_projection(); flush_stream(TRUE);
                        render_cpu_mark(W,nullptr);
                        return;
                    }
                    if (W->m_Projection && W->m_Faces.empty()) // R37: carried faces draw through the CPU path below
                    {
                        // Preserve slot/mark blending order through independent CPU and GPU-created geometry draws.
                        flush_stream(TRUE);
                        if (!queue_projection(W))
                        {
                            flush_projection();
                            if (gpu_skinning) ++gpu_skinning->projection_cpu_fallbacks;
                            render_cpu_mark(W, nullptr);
                        }
                        return;
                    }
                    flush_projection();
                    render_cpu_mark(W, nullptr);
                };
                if (gpu_skinning)
                {
                    const auto range = gpu_skinning->slot_plans[i];
                    const auto& plans = gpu_skinning->plans;
                    const bool gpu_draws = !gpu_skinning->frame_batch_failed && gpu_skinning->frame_output.vertices != nullptr;
                    const u32 last = range.first + range.count;
                    for (u32 p = range.first; p < last; ++p)
                    {
                        const auto& plan = plans[p];
                        intrusive_ptr<CSkeletonWallmark> W = slot->skeleton_items[plan.item];
                        if (!W) continue;
                        if (plan.kind == BloodGpuWallmarkState::Plan::Gpu && gpu_draws)
                        {
                            u32 first = plan.first_vertex, count = plan.vertex_count;
                            while (p + 1 < last)
                            {
                                const auto& next = plans[p + 1];
                                if (next.kind != BloodGpuWallmarkState::Plan::Gpu || next.field != plan.field || next.ownership!=plan.ownership || next.tear!=plan.tear ||
                                    next.first_vertex != first + count) break;
                                count += next.vertex_count;
                                ++p;
                            }
                            flush_projection(); flush_stream(TRUE);
                            draw_gpu_run(plan, first, count);
                        }
                        else render_cpu_plan(W);
#ifdef DEBUG
                        W->used_in_render = u32(-1);
#endif
                    }
                }
                else
                {
                    for (int j = 0; j < slot->skeleton_items.size(); j++)
                    {
                        intrusive_ptr<CSkeletonWallmark> W = slot->skeleton_items[j];
                        if (!W) continue;
                        if (!RImplementation.ViewBase.testSphere_dirty(W->m_Bounds.P, W->m_Bounds.R)) continue;
                        Device.Statistic->RenderDUMP_WMD_Count++;
                        const float dst = Device.vCameraPosition.distance_to_sqr(W->m_Bounds.P);
                        const float ssa = W->m_Bounds.R * W->m_Bounds.R / dst;
                        if (ssa < ssaCLIP) continue;
                        render_cpu_plan(W);
#ifdef DEBUG
                        W->used_in_render = u32(-1);
#endif
                    }
                }
#else
                for (int j = 0; j < slot->skeleton_items.size(); j++)
                {
                    intrusive_ptr<CSkeletonWallmark> W = slot->skeleton_items[j];
                    if (!W) continue;
                    if (!RImplementation.ViewBase.testSphere_dirty(W->m_Bounds.P, W->m_Bounds.R)) continue;
                    Device.Statistic->RenderDUMP_WMD_Count++;
                    const float dst = Device.vCameraPosition.distance_to_sqr(W->m_Bounds.P);
                    const float ssa = W->m_Bounds.R * W->m_Bounds.R / dst;
                    if (ssa < ssaCLIP) continue;
                    if (W->m_Growth.Growing(RDEVICE.fTimeGlobal-W->TimeStart())) flush_stream(TRUE);
                    render_cpu_mark(W, nullptr);
#ifdef DEBUG
                    W->used_in_render = u32(-1);
#endif
                }
#endif
            }

            // Preserve the original slot/material order across CPU/GPU runs.
#ifdef USE_DX11
            flush_projection();
#endif
            flush_stream(!skeleton_empty);
        }
#ifdef USE_DX11
        if (gpu_skinning)
        {
            for (auto it = gpu_skinning->projection_marks.begin(); it != gpu_skinning->projection_marks.end();)
            {
                if (it->second.seen_frame != Device.dwFrame || !it->second.mark->m_Projection)
                {
                    gpu_skinning->projection_backend.ReleaseMark(it->second.handle);
                    it = gpu_skinning->projection_marks.erase(it);
                }
                else ++it;
            }
            for (auto it = gpu_skinning->sources.begin(); it != gpu_skinning->sources.end();)
            {
                if (Device.dwFrame - it->second.seen_frame > 120) // R22: a source survives 2 s out of view (no re-upload on a quick turn)
                {
                    gpu_skinning->backend.ReleaseSource(it->second.handle);
                    it = gpu_skinning->sources.erase(it);
                }
                else ++it;
            }
        }
#endif
    }
	
	// Do not expose the heat target to unrelated level-wallmark materials.
#ifdef USE_DX11
    RCache.set_RT(nullptr,1);
#endif
	// Level-wmarks
	{
		BloodNativeProfile::Scope profileGraph(BloodNativeProfile::Stage::RenderGraph);
		RImplementation.GMBase.r_dsgraph_render_wmarks();
	}
	Device.Statistic->RenderDUMP_WM.End();

	// Projection
	Device.mView = mSavedView;
	Device.mProject._43 = _43;
	RCache.set_xform_view(Device.mView);
	RCache.set_xform_project(Device.mProject);
}
