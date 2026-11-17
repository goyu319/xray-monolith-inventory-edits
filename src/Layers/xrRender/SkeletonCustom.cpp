//---------------------------------------------------------------------------
#include 	"stdafx.h"
#pragma hdrstop
#include "BloodProjectionFootprint.h"
#include "dxWallMarkArray.h"
#include "SkeletonRuntimeGeometry.h"

#include 	"SkeletonCustom.h"
#include "BloodNativeProfile.h"
#include "BloodThermalWallmark.h"
#include "../../Include/xrRender/OriginalSkinProxyPrecompute.h"
#include "PainterRemovalPrograms.h"
#include "BloodGpuSkinning.h"
#include "PainterWallmarkOwnership.h"
#include "PainterBloodModelSize.h"
#include "PainterHitMain.h" // triggerhitch
#include "PainterPickNearest.h"
#include "PainterWallmarkScan.h" // wallmarkpick
#include <atomic>
static std::atomic<u64> g_blood_mark_namespace{1};
#include "PainterBloodGpu.h"
#include "PainterDent.h" // melee (value 3): a decal on a dented body
#include <map>
#include "../../Include/xrRender/PainterLogVerbose.h"
#include <cstring>
#include <limits>
#include <type_traits>
#ifdef USE_DX11
#include <mutex>
#include "cl_intersect.h"
extern int r_blood_gpu, r_blood_gpu_create;
std::atomic<u64> g_blood_projection_epoch{1};
BloodProjectionCpuCounters g_blood_projection_cpu;
// triggerhitch (2026-11-15): one posed copy of every child's vertices for CKinematics::LL_PickNearest, kept for the body last
// asked in this frame (the pellets of one shot share it). Valid only for the same body, frame, geometry generation, children
// arrays and bone render matrices (compared byte for byte). Main thread; its own lock; never destroyed.
namespace
{
struct PainterPickMemo
{
    struct Child
    {
        CSkeletonX* child = nullptr;
        const void* vertices = nullptr;
        u32 count = 0;
        const u16* indices = nullptr;
        size_t index_count = 0;
        bool finite = false;
        std::vector<Fvector> posed;
        std::vector<painter_pick_nearest::Box> boxes;
    };
    std::mutex mutex;
    const CKinematics* owner = nullptr;
    u32 frame = 0;
    u64 generation = 0;
    bool valid = false;
    std::vector<Fmatrix> pose;
    std::vector<Child> children;
};
PainterPickMemo& painter_pick_memo()
{
    static PainterPickMemo* memo = new PainterPickMemo;
    return *memo;
}
}
namespace
{
	std::atomic<size_t> projection_source_bytes{0};
	std::atomic<size_t> projection_source_peak_bytes{0};
	std::mutex projection_pending_mutex;
	CSkeletonWallmark::ProjectionVector<std::weak_ptr<const CSkeletonWallmark::ProjectionSource>> projection_pending;
}

bool CSkeletonWallmark::ProjectionSource::ReserveBudget(size_t bytes) noexcept
{
	constexpr size_t mesh_limit = 16u * 1024u * 1024u;
	constexpr size_t total_limit = 768u * 1024u * 1024u; // 2026-09-29: 256 MB suspected exhausted by GAMMA's skinned models
	if (reserved_bytes > mesh_limit || bytes > mesh_limit - reserved_bytes)
	{
		g_blood_projection_cpu.cache_child_budget.fetch_add(1, std::memory_order_relaxed);
		return false;
	}
	size_t current = projection_source_bytes.load(std::memory_order_relaxed);
	do
	{
		if (current > total_limit - bytes)
		{
			g_blood_projection_cpu.cache_total_budget.fetch_add(1, std::memory_order_relaxed);
			return false;
		}
	} while (!projection_source_bytes.compare_exchange_weak(current, current + bytes,
		std::memory_order_relaxed));
	reserved_bytes += bytes;
	size_t peak = projection_source_peak_bytes.load(std::memory_order_relaxed);
	while (peak < current + bytes && !projection_source_peak_bytes.compare_exchange_weak(
		peak, current + bytes, std::memory_order_relaxed)) {}
	return true;
}

void CSkeletonWallmark::ProjectionSource::BuildProjectionBounds() noexcept
{
    if (!projection_bounds.empty() || indices.empty()) return;
    const size_t count=(indices.size()+BloodProjectionBounds::BlockVertices-1)/BloodProjectionBounds::BlockVertices;
    const size_t bytes=count*sizeof(BloodProjectionBounds::Block);
    // Optional metadata shares the existing global/child accounting. A failed
    // reservation or allocation preserves the fully usable unfiltered source.
    if (bytes>512u*1024u || !ReserveBudget(bytes)) return;
    try {
        projection_bounds.resize(count);
        for (size_t b=0; b<count; ++b) {
            const size_t first=b*BloodProjectionBounds::BlockVertices;
            projection_bounds[b]=BloodProjectionBounds::Build(vertices,indices,first,
                (std::min)(first+BloodProjectionBounds::BlockVertices,indices.size()));
        }
    } catch (...) {
        ProjectionVector<BloodProjectionBounds::Block>().swap(projection_bounds);
        reserved_bytes-=bytes;
        projection_source_bytes.fetch_sub(bytes,std::memory_order_relaxed);
    }
}

size_t CSkeletonWallmark::ProjectionSource::ReservedBytes() noexcept
{
	return projection_source_bytes.load(std::memory_order_relaxed);
}

size_t CSkeletonWallmark::ProjectionSource::PeakBytes() noexcept
{
	return projection_source_peak_bytes.load(std::memory_order_relaxed);
}

CSkeletonWallmark::ProjectionSource::~ProjectionSource()
{
	projection_source_bytes.fetch_sub(reserved_bytes, std::memory_order_relaxed);
}

void CSkeletonWallmark::ProjectionSource::Publish(const std::shared_ptr<const ProjectionSource>& source)
{
	std::lock_guard<std::mutex> guard(projection_pending_mutex);
	if (projection_pending.size() >= BloodGpuProjection::Backend::MaxSources * size_t(2))
	{
		projection_pending.erase(std::remove_if(projection_pending.begin(), projection_pending.end(),
			[](const auto& item) { return item.expired(); }), projection_pending.end());
		if (projection_pending.size() >= BloodGpuProjection::Backend::MaxSources * size_t(2))
			throw std::bad_alloc();
	}
	projection_pending.push_back(source);
}

void CSkeletonWallmark::ProjectionSource::TakePending(ProjectionVector<std::weak_ptr<const ProjectionSource>>& pending)
{
	std::lock_guard<std::mutex> guard(projection_pending_mutex);
	pending.erase(std::remove_if(pending.begin(), pending.end(),
		[](const auto& item) { return item.expired(); }), pending.end());
	if (pending.empty()) pending.swap(projection_pending);
	else
	{
		pending.insert(pending.end(), projection_pending.begin(), projection_pending.end());
		projection_pending.clear();
	}
}

bool CSkeletonWallmark::ProjectionSource::BindOwnership(const std::shared_ptr<PainterWallmarkOwnershipGpu>&field,u32 child,u32 original,const std::vector<u32>*provenance)
{
    if(source_faces.size()!=indices.size()/3)return false;
    source_child=child;original_child=original==~0u?child:original;ownership=field;ownership_required=false;
    if(!ReserveBudget(source_faces.size()*sizeof(u32)*(provenance?2:1)))return false;
    if(provenance){original_faces.reserve(source_faces.size());for(u32 face:source_faces)original_faces.push_back(face<provenance->size()?(*provenance)[face]:~0u);}
    if(!field){for(auto&v:vertices)v.ownership[0]=v.ownership[1]=0;return true;}
    if(!field->field)return false;
    // A source vertex can be shared by primitives with different regions.
    // Preserve that sharing only within an identical ownership range.
    using Key=std::pair<u32,std::array<u32,2>>;
    std::map<Key,u32> remap;
    ProjectionVector<BloodGpuProjection::InputVertex> next;
    ProjectionVector<u32> nextIndices;nextIndices.reserve(indices.size());
    for(size_t t=0;t<source_faces.size();++t){
        const auto*p=field->field->primitive(child,source_faces[t]);if(!p)return false;
        ownership_required|=p->direct!=PainterWallmarkOwnership::unrestricted;
        for(unsigned k=0;k<3;++k){const u32 old=indices[t*3+k];if(old>=vertices.size())return false;
            const Key key{old,p->direct};auto found=remap.find(key);
            if(found==remap.end()){const u32 index=u32(next.size());auto v=vertices[old];v.ownership[0]=p->direct[0];v.ownership[1]=p->direct[1];
                next.push_back(v);found=remap.emplace(key,index).first;}nextIndices.push_back(found->second);}}
    const size_t extra=next.size()>vertices.size()?(next.size()-vertices.size())*sizeof(next[0]):0;
    if(extra&&!ReserveBudget(extra))return false;
    vertices.swap(next);indices.swap(nextIndices);return true;
}
void CSkeletonWallmark::AssignOwnership(WMFace&face,const std::shared_ptr<PainterWallmarkOwnershipGpu>&field,u32 child,u32 triangle)
{
    face.source_child=child;face.source_face=triangle;face.source_key=nullptr;
    face.provenance_keeper.reset();face.ownership=field;face.ownership_range={0,0};
    if(!field)return;
    const auto*p=field->field->primitive(child,triangle);
    if(!p){face.ownership_range=PainterWallmarkOwnership::rejected;return;}
    face.source_child=p->source_child;face.source_face=p->source_face;face.ownership_range=p->direct;
    if(p->source_face==~0u){face.provenance_keeper=field->field;face.source_key=p->key.get();}
}
void CSkeletonWallmark::ApplyCarryOwnership(const std::shared_ptr<PainterWallmarkOwnershipGpu>&field)noexcept
{
    if(!field)return;
    for(auto&face:m_Faces){if(!field->field->replaced.count(face.source_child))continue;
        std::array<u32,2>range=PainterWallmarkOwnership::rejected;
        if(face.source_face!=~0u){const auto f=field->field->authored.find({face.source_child,face.source_face});
            if(f!=field->field->authored.end())range=f->second;}
        else if(face.source_key){const auto f=field->field->cavities.find({face.source_child,*face.source_key});
            if(f!=field->field->cavities.end()){range=f->second;face.source_key=&f->first.second;face.provenance_keeper=field->field;}}
        // Unmatched old cavity is hidden only until the queued bounded refresh.
        // Its original key/keeper survive so no identity is guessed on another cut.
        face.ownership=field;face.ownership_range=range;
    }
}
bool CSkeletonWallmark::FaceOwnership(std::shared_ptr<PainterWallmarkOwnershipGpu>&field)const noexcept
{
    field.reset();for(const auto&face:m_Faces)if(face.ownership_range!=PainterWallmarkOwnership::unrestricted){
        if(!face.ownership)return false;if(field&&field!=face.ownership)return false;field=face.ownership;}return true;
}
bool CSkeletonWallmark::ProjectionOwnership(std::shared_ptr<PainterWallmarkOwnershipGpu>&field)const noexcept
{
    field.reset();if(!m_Projection)return true;
    for(const auto&r:m_Projection->ranges)if(r.source->ownership_required){
        if(!r.source->ownership)return false;if(field&&field!=r.source->ownership)return false;field=r.source->ownership;}return true;
}

bool CSkeletonWallmark::ResolveProjection(u32* budget)
{
	if (!m_Projection) return true;
	try
	{
		const auto& hit = *m_Projection;
		// R37: resumable. A budget (candidate vertices this call may visit) stops the walk between triangles and keeps
		// the faces found so far; the next call continues there. Without a budget the whole snapshot resolves at once.
		if (!m_ResolveActive || m_ResolveSnapshot != static_cast<const void*>(&hit))
		{
			m_ResolveFaces.clear(); m_ResolveRange = 0; m_ResolveVertex = 0; m_ResolveActive = true;
			m_ResolveSnapshot = static_cast<const void*>(&hit);
		}
		for (; m_ResolveRange < hit.ranges.size(); ++m_ResolveRange, m_ResolveVertex = 0)
		{
			const auto& range = hit.ranges[m_ResolveRange];
			const auto& source = *range.source;
			const auto* indices = source.indices.data() + range.first_vertex;
			for (u32 i = m_ResolveVertex; i < range.vertex_count; i += 3)
			{
				if (budget)
				{
					if (*budget < 3) { m_ResolveVertex = i; return false; }
					*budget -= 3;
				}
				Fvector p[3];
				for (u32 k = 0; k < 3; ++k)
				{
					const auto& v = source.vertices[indices[i + k]];
					const Fvector original = {v.p[0], v.p[1], v.p[2]};
					Fvector pb[4];
					for (u32 j = 0; j < v.job; ++j)
						hit.hit_pose[v.ids[j]].transform_tiny(pb[j], original);
					// Creation follows the original vertex format, not the render
					// sentinel. Keep 4W's chained remainder exactly as get_pos_bones.
					if (v.job == 1) p[k] = pb[0];
					else if (v.job == 2) p[k].lerp(pb[0], pb[1], v.weights[0]);
					else
					{
						float remainder = 1.f - v.weights[0] - v.weights[1];
						if (v.job == 4) remainder -= v.weights[2];
						for (u32 j = 0; j + 1 < v.job; ++j) pb[j].mul(v.weights[j]);
						pb[v.job - 1].mul(remainder);
						p[k] = pb[0];
						for (u32 j = 1; j < v.job; ++j) p[k].add(pb[j]);
					}
				}
				Fvector test_normal;
				test_normal.mknormal(p[0], p[1], p[2]);
				if (test_normal.dotproduct(hit.normal) < EPS || !BloodProjectionFootprint::Outer(ContactPoint(), hit.normal, hit.size, p, hit.outer_depth) ||
					!(hit.square_footprint ? BloodProjectionFootprint::Intersects(hit.view, p) : CDB::TestSphereTri(ContactPoint(), hit.size, p))) continue;
				WMFace face{};
                const u32 triangle=(range.first_vertex+i)/3;
                if(triangle>=source.source_faces.size())return false;
                AssignOwnership(face,source.ownership,source.source_child,source.source_faces[triangle]);
                if(!source.ownership){face.source_child=source.original_child;face.source_face=source.original_faces.empty()?source.source_faces[triangle]:source.original_faces[triangle];}
				for (u32 k = 0; k < 3; ++k)
				{
					const auto& v = source.vertices[indices[i + k]];
					face.vert[k].set(v.p[0], v.p[1], v.p[2]);
					for (u32 j = 0; j < 4; ++j) face.bone_id[k][j] = u16(v.ids[j]);
					for (u32 j = 0; j < 3; ++j) face.weight[k][j] = v.weights[j];
					if (v.job == 4) WMFace::LinksFrom4W(face.bone_id[k], face.weight[k]); // R27: 4W slots -> link-rule layout
					Fvector uv; hit.view.transform_tiny(uv, p[k]);
					face.uv[k].set((1.f + uv.x) * .5f, (1.f - uv.y) * .5f);
				}
				m_ResolveFaces.push_back(face);
			}
		}
		m_Faces.swap(m_ResolveFaces);
		WMFacesVec().swap(m_ResolveFaces);
		m_ResolveActive = false; m_ResolveRange = 0; m_ResolveVertex = 0; m_ResolveSnapshot = nullptr;
		m_CarryRanges=std::move(m_Projection);m_Projection.reset();
		InvalidateRenderCache();
		return true;
	}
	catch (...) { m_ResolveActive = false; WMFacesVec().swap(m_ResolveFaces); return false; } // Keep the complete immutable snapshot for retry.
}
#endif
#include	"SkeletonX.h"
#include	"../../xrEngine/fmesh.h"
#ifndef _EDITOR
#include	"../../xrEngine/Render.h"
#endif

int psSkeletonUpdate = 32;
xrCriticalSection wallmarks_cs;

#ifndef _EDITOR
#include "../../xrServerEntities/smart_cast.h"
#else
#include "../../Include/xrAPI/xrAPI.h"
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
bool pred_N(const std::pair<shared_str, u32>& N, LPCSTR B)
{
	return xr_strcmp(*N.first, B) < 0;
}

u16 CKinematics::LL_BoneID(LPCSTR B)
{
	accel::iterator I = std::lower_bound(bone_map_N->begin(), bone_map_N->end(), B, pred_N);
	if (I == bone_map_N->end()) return BI_NONE;
	if (0 != xr_strcmp(*(I->first), B)) return BI_NONE;
	return u16(I->second);
}

bool pred_P(const std::pair<shared_str, u32>& N, const shared_str& B)
{
	return N.first._get() < B._get();
}

u16 CKinematics::LL_BoneID(const shared_str& B)
{
	accel::iterator I = std::lower_bound(bone_map_P->begin(), bone_map_P->end(), B, pred_P);
	if (I == bone_map_P->end()) return BI_NONE;
	if (I->first._get() != B._get()) return BI_NONE;
	return u16(I->second);
}

//
LPCSTR CKinematics::LL_BoneName_dbg(u16 ID)
{
	CKinematics::accel::iterator _I, _E = bone_map_N->end();
	for (_I = bone_map_N->begin(); _I != _E; ++_I) if (_I->second == ID) return *_I->first;
	return 0;
}

#ifdef DEBUG
void CKinematics::DebugRender(Fmatrix& XFORM)
{
	CalculateBones	();

	CBoneData::BoneDebug	dbgLines;
	(*bones)[iRoot]->DebugQuery	(dbgLines);

	Fvector Z;  Z.set(0,0,0);
	Fvector H1; H1.set(0.01f,0.01f,0.01f);
	Fvector H2; H2.mul(H1,2);
	for (u32 i=0; i<dbgLines.size(); i+=2)	{
		Fmatrix& M1 = bone_instances[dbgLines[i]].mTransform;
		Fmatrix& M2 = bone_instances[dbgLines[i+1]].mTransform;

		Fvector P1,P2;
		M1.transform_tiny(P1,Z);
		M2.transform_tiny(P2,Z);
		RCache.dbg_DrawLINE(XFORM,P1,P2,D3DCOLOR_XRGB(0,255,0));

		Fmatrix M;
		M.mul_43(XFORM,M2);
		RCache.dbg_DrawOBB(M,H1,D3DCOLOR_XRGB(255,255,255));
		RCache.dbg_DrawOBB(M,H2,D3DCOLOR_XRGB(255,255,255));
	}

	for (u32 b=0; b<bones->size(); b++)
	{
		Fobb&		obb		= (*bones)[b]->obb;
		Fmatrix&	Mbone	= bone_instances[b].mTransform;
		Fmatrix		Mbox;	obb.xform_get(Mbox);
		Fmatrix		X;		X.mul(Mbone,Mbox);
		Fmatrix		W;		W.mul(XFORM,X);
		RCache.dbg_DrawOBB(W,obb.m_halfsize,D3DCOLOR_XRGB(0,0,255));
	}
}
#endif

CKinematics::CKinematics()
{
#ifdef DEBUG
	dbg_single_use_marker		= FALSE;
#endif

	m_is_original_lod = false;
	UCalc_ThisFrame = false;
}

CKinematics::~CKinematics()
{
#ifdef USE_DX11
	{ // triggerhitch: the pick memo never outlives its body
		PainterPickMemo& memo = painter_pick_memo();
		std::lock_guard<std::mutex> lock(memo.mutex);
		if (memo.owner == this) { memo.valid = false; memo.owner = nullptr; }
	}
#endif
	LL_ClearRuntimeGeometry();
	LL_ClearGeometryVariant();
	IBoneInstances_Destroy();
	// wallmarks
	ClearWallmarks();

	if (m_lod)
	{
		if (CKinematics* lod_kinematics = fast_dynamic_cast<CKinematics*>(m_lod))
		{
			if (lod_kinematics->m_is_original_lod)
			{
				lod_kinematics->Release();
			}
		}

		xr_delete(m_lod);
	}
}

void CKinematics::IBoneInstances_Create()
{
	// VERIFY2				(bones->size() < 64, "More than 64 bones is a crazy thing!");
	u32 size = bones->size();
	bone_instances = xr_alloc<CBoneInstance>(size);
	for (u32 i = 0; i < size; i++)
		bone_instances[i].construct();
}

void CKinematics::IBoneInstances_Destroy()
{
	if (bone_instances)
	{
		xr_free(bone_instances);
		bone_instances = NULL;
	}
}

bool pred_sort_N(const std::pair<shared_str, u32>& A, const std::pair<shared_str, u32>& B)
{
	return xr_strcmp(A.first, B.first) < 0;
}

bool pred_sort_P(const std::pair<shared_str, u32>& A, const std::pair<shared_str, u32>& B)
{
	return A.first._get() < B.first._get();
}

CSkeletonX* CKinematics::LL_GetChild(u32 idx)
{
	IRenderVisual* V = children[idx];
	CSkeletonX* B = fast_dynamic_cast<CSkeletonX*>(V);
	return B;
}

int ps_painter_ragdoll_override = 1;

namespace
{
// Ragdoll side files: meshes\painter\ragdoll\<model>.ikd carry another export's per-bone physics (shape, joint, mass,
// centre of mass) for this exact model and are applied over the model's own values at load, so the model file - and
// every profile keyed to its exact bytes - stays unchanged. Bind poses always stay the model's own.
// Mode 1: all physics fields. Mode 2: limits, springs, friction, break strength and masses only; bone shapes, joint
// types and breakable flags stay the model's own, so no bone becomes or stops being a separate physics body.
u64 painter_fnv1a64(const void* data, u32 size)
{
	u64 h = 0xcbf29ce484222325ull;
	const u8* p = static_cast<const u8*>(data);
	for (u32 i = 0; i < size; ++i)
		h = (h ^ p[i]) * 0x100000001b3ull;
	return h;
}

struct SPainterRagdollBone
{
	shared_str game_mtl_name;
	SBoneShape shape;
	SJointIKData ik;
	float mass = 0.f;
	Fvector center_of_mass;
};

void painter_ragdoll_side_file(LPCSTR model, vecBones& bones, const void* own_ik, u32 own_ik_size)
{
	int const mode = ps_painter_ragdoll_override;
	if (mode <= 0 || !model || !*model)
		return;
	string_path name;
	xr_strcpy(name, model);
	for (char* c = name; *c; ++c)
		*c = *c == '/' ? '\\' : char(tolower(*c));
	if (LPSTR ext = strext(name))
		if (!xr_strcmp(ext, ".ogf"))
			*ext = 0;
	string_path file;
	xr_strconcat(file, "painter\\ragdoll\\", name, ".ikd");
	if (!FS.exist("$game_meshes$", file))
		return;
	IReader* F = FS.r_open("$game_meshes$", file);
	if (!F)
		return;
	LPCSTR failure = nullptr;
	xr_vector<SPainterRagdollBone> loaded;
	if (F->elapsed() < 28)
		failure = "header";
	else
	{
		char magic[8];
		F->r(magic, 8);
		u32 const count = F->r_u32(), own_size = F->r_u32();
		u64 const own_hash = F->r_u64();
		u32 const ik_size = F->r_u32();
		if (memcmp(magic, "PRAGDOL1", 8))
			failure = "magic";
		else if (count != bones.size())
			failure = "bone_count";
		else if (own_size != own_ik_size || own_hash != painter_fnv1a64(own_ik, own_ik_size))
			failure = "model_changed"; // written for another version of this model
		else
		{
			for (u32 i = 0; i < count && !failure; ++i)
			{
				shared_str bone_name;
				F->r_stringZ(bone_name);
				if (bone_name != bones[i]->name)
					failure = "bone_names";
			}
			if (!failure && u32(F->elapsed()) != ik_size)
				failure = "size";
			for (u32 i = 0; i < count && !failure; ++i)
			{
				if (F->elapsed() < int(4 + 1 + sizeof(SBoneShape) + 72 + 24 + 4 + 12))
				{
					failure = "record";
					break;
				}
				SPainterRagdollBone b;
				u16 const vers = u16(F->r_u32());
				F->r_stringZ(b.game_mtl_name);
				F->r(&b.shape, sizeof(SBoneShape));
				b.ik.friction = 0.f;
				b.ik.Import(*F, vers);
				Fvector unused;
				F->r_fvector3(unused); // bind rotation and position stay the model's own
				F->r_fvector3(unused);
				b.mass = F->r_float();
				F->r_fvector3(b.center_of_mass);
				if (b.shape.type > SBoneShape::stCylinder)
				{
					b.shape.Reset();
					b.ik.type = jtRigid;
				}
				if (!_valid(b.mass) || b.mass < 0.f || !_valid(b.center_of_mass))
					failure = "values";
				loaded.push_back(b);
			}
			if (!failure && F->elapsed() != 0)
				failure = "trailing";
		}
	}
	FS.r_close(F);
	if (failure)
	{
		Msg("! [painter_ragdoll] %s: side file ignored (%s)", name, failure);
		return;
	}
	for (u32 i = 0; i < bones.size(); ++i)
	{
		CBoneData& B = *bones[i];
		SPainterRagdollBone const& b = loaded[i];
		if (mode == 1)
		{
			B.game_mtl_name = b.game_mtl_name;
			B.shape = b.shape;
			B.IK_data = b.ik;
		}
		else
		{
			EJointType const type = B.IK_data.type;
			Flags32 const flags = B.IK_data.ik_flags;
			B.IK_data = b.ik;
			B.IK_data.type = type;
			B.IK_data.ik_flags = flags;
			if (b.shape.type == B.shape.type)
				B.shape = b.shape;
		}
		B.mass = b.mass;
		B.center_of_mass = b.center_of_mass;
	}
	Msg("[painter_ragdoll] mode=%d %s bones=%u", mode, name, unsigned(bones.size()));
}
} // namespace

void CKinematics::Load(const char* N, IReader* data, u32 dwFlags)
{
	//Msg				("skeleton: %s",N);
	inherited::Load(N, data, dwFlags);

	pUserData = NULL;
	m_lod = NULL;
	// loading lods

	IReader* LD = data->open_chunk(OGF_S_LODS);
	if (LD)
	{
		string_path short_name;
		xr_strcpy(short_name, sizeof(short_name), N);

		if (strext(short_name)) *strext(short_name) = 0;
		// From stream
		{
			string_path lod_name;
			LD->r_string(lod_name, sizeof(lod_name));
			//.         strconcat		(sizeof(name_load),name_load, short_name, ":lod:", lod_name.c_str());
			m_lod = (dxRender_Visual*)::Render->model_CreateChild(lod_name, NULL);

			if (CKinematics* lod_kinematics = fast_dynamic_cast<CKinematics*>(m_lod))
			{
				lod_kinematics->m_is_original_lod = true;
			}

			VERIFY3(m_lod, "Cant create LOD model for", N);
			//.			VERIFY2			(m_lod->Type==MT_HIERRARHY || m_lod->Type==MT_PROGRESSIVE || m_lod->Type==MT_NORMAL,lod_name.c_str());
			/*
						strconcat		(name_load, short_name, ":lod:1");
						m_lod 			= ::Render->model_CreateChild(name_load,LD);
						VERIFY			(m_lod->Type==MT_SKELETON_GEOMDEF_PM || m_lod->Type==MT_SKELETON_GEOMDEF_ST);
			*/
		}
		LD->close();
	}

#ifndef _EDITOR
	// User data
	IReader* UD = data->open_chunk(OGF_S_USERDATA);
	pUserData = UD ? xr_new<CInifile>(UD, FS.get_path("$game_config$")->m_Path) : 0;
	if (UD) UD->close();
#endif

	// Globals
	bone_map_N = xr_new<accel>();
	bone_map_P = xr_new<accel>();
	bones = xr_new<vecBones>();
	bone_instances = nullptr;
	bones_size = u16(0);

	// Load bones
#pragma todo("container is created in stack!")
	xr_vector<shared_str> L_parents;

	R_ASSERT(data->find_chunk(OGF_S_BONE_NAMES));

	hidden_bones.zero();
	visimask.zero();
	int dwCount = data->r_u32();
	// Msg				("!!! %d bones",dwCount);
	// if (dwCount >= 64)	Msg			("!!! More than 64 bones is a crazy thing! (%d), %s",dwCount,N);
	VERIFY3(dwCount <= 64, "More than 64 bones is a crazy thing!", N);
	for (; dwCount; dwCount--)
	{
		string256 buf;

		// Bone
		u16 ID = u16(bones->size());
		data->r_stringZ(buf, sizeof(buf));
		strlwr(buf);
		CBoneData* pBone = CreateBoneData(ID);
		pBone->name = shared_str(buf);
		pBone->child_faces.resize(children.size());
		bones->push_back(pBone);
		bone_map_N->push_back(mk_pair(pBone->name, ID));
		bone_map_P->push_back(mk_pair(pBone->name, ID));

		// It's parent
		data->r_stringZ(buf, sizeof(buf));
		strlwr(buf);
		L_parents.push_back(buf);

		data->r(&pBone->obb, sizeof(Fobb));
		visimask.set(u64(1) << ID, TRUE);
		hidden_bones.set(u64(1) << ID, TRUE);
	}
	std::sort(bone_map_N->begin(), bone_map_N->end(), pred_sort_N);
	std::sort(bone_map_P->begin(), bone_map_P->end(), pred_sort_P);

	// Attach bones to their parents
	iRoot = BI_NONE;
	for (u32 i = 0; i < bones->size(); i++)
	{
		shared_str P = L_parents[i];
		CBoneData* B = (*bones)[i];
		if (!P || !P[0])
		{
			// no parent - this is root bone
			R_ASSERT(BI_NONE == iRoot);
			iRoot = u16(i);
			B->SetParentID(BI_NONE);
			continue;
		}
		else
		{
			u16 ID = LL_BoneID(P);
			R_ASSERT(ID != BI_NONE);
			(*bones)[ID]->children.push_back(B);
			B->SetParentID(ID);
		}
	}
	R_ASSERT(BI_NONE != iRoot);

	// Free parents
	L_parents.clear();

	// IK data
	IReader* IKD = data->open_chunk(OGF_S_IKDATA);
	if (IKD)
	{
		const void* const own_ik = IKD->pointer();
		u32 const own_ik_size = u32(IKD->length());
		for (u32 i = 0; i < bones->size(); i++)
		{
			CBoneData* B = (*bones)[i];
			u16 vers = (u16)IKD->r_u32();
			IKD->r_stringZ(B->game_mtl_name);
			IKD->r(&B->shape, sizeof(SBoneShape));
			B->IK_data.Import(*IKD, vers);
			// Some original GFY facial controls contain legacy editor values
			// (for example 62) instead of a native collision shape. Keep their
			// animation/bind transforms, but inherit the parent's physics as
			// an unshaped rigid child. Never pass an unknown type to ODE.
			if (B->shape.type > SBoneShape::stCylinder)
			{
				Msg("! [kinematics] unsupported bone shape %u on %s; retaining animation, inheriting parent physics",
				    unsigned(B->shape.type), B->name.c_str());
				B->shape.Reset();
				B->IK_data.type = jtRigid;
			}
			Fvector vXYZ, vT;
			IKD->r_fvector3(vXYZ);
			IKD->r_fvector3(vT);
			B->bind_transform.setXYZi(vXYZ);
			B->bind_transform.translate_over(vT);
			B->mass = IKD->r_float();
			IKD->r_fvector3(B->center_of_mass);
		}
		painter_ragdoll_side_file(N, *bones, own_ik, own_ik_size);
		// calculate model to bone converting matrix
		(*bones)[LL_GetBoneRoot()]->CalculateM2B(Fidentity);
		IKD->close();
	}

	// after load process
	{
		for (u16 child_idx = 0; child_idx < (u16)children.size(); child_idx++)
			LL_GetChild(child_idx)->AfterLoad(this, child_idx);
	}

	// unique bone faces
	{
		for (u32 bone_idx = 0; bone_idx < bones->size(); bone_idx++)
		{
			CBoneData* B = (*bones)[bone_idx];
			for (u32 child_idx = 0; child_idx < children.size(); child_idx++)
			{
				CBoneData::FacesVec faces = B->child_faces[child_idx];
				std::sort(faces.begin(), faces.end());
				CBoneData::FacesVecIt new_end = std::unique(faces.begin(), faces.end());
				faces.erase(new_end, faces.end());
				B->child_faces[child_idx].clear_and_free();
				B->child_faces[child_idx] = faces;
			}
		}
	}

#ifdef USE_DX11
	// Cache the exact finalized lists that FillVertices will consume. Building
	// during child collection captured repeated per-vertex/influence entries,
	// inflated admission bytes, and could render each face multiple times.
	for (u16 child_idx = 0; child_idx < (u16)children.size(); ++child_idx)
		LL_GetChild(child_idx)->FinalizeProjectionSource();
#endif

	// reset update_callback
	Update_Callback = NULL;
	// reset update frame
	wm_frame = u32(-1);

	LL_Validate();

	bones_size = bones->size();
    // Captured once from the original, fully loaded model. Shared by Copy and
    // retained through pooled instance/publication lifetimes; no VFS reread.
    try { auto proxy_source=std::make_shared<painter_skin_proxy::Source>();
    bool proxy_ok=true;
    for(u32 i=0;i<children.size();++i){
        LPCSTR texture=children[i]->getDebugTextureDef();
        if(!texture||!painter_skin_proxy::admitted_material(texture))continue;
        proxy_source->authored_materials.append(texture);proxy_source->authored_materials.push_back('\0');
        // Authenticate the actual resolved programs, not a shader declaration.
        // Only main HQ/LQ visibility is certified. Auxiliary shadow/reflection
        // passes cannot veto intact main skin. Alpha/forward/tessellation and
        // unknown main programs still decline.
        auto* original_material=fast_dynamic_cast<dxRender_Visual*>(children[i]);bool opaque=original_material&&bool(original_material->shader);unsigned opaque_passes=0;
        if(opaque)for(unsigned main_pass=0;main_pass<2;++main_pass){const auto& e=original_material->shader->E[main_pass];if(!e||e->passes.empty()){opaque=false;continue;}if(e->flags.bStrictB2F)opaque=false;for(const auto& p:e->passes){
            if(!p||!p->ps||!p->vs){opaque=false;continue;}
#if defined(USE_DX10) || defined(USE_DX11)
            if(p->gs&&p->gs->gs){opaque=false;continue;}
#endif
#ifdef USE_DX11
            if((p->hs&&p->hs->sh)||(p->ds&&p->ds->sh)){opaque=false;continue;}
#endif
            xr_string ps=*p->ps->cName;if(ps.size()>2&&ps[ps.size()-2]=='_'&&ps.back()>='0'&&ps.back()<='7')ps.resize(ps.size()-2);const bool known=ps=="dumb"||(ps.find("deffer_base_")==0&&ps.find("aref")==xr_string::npos&&ps.find("atoc")==xr_string::npos&&PainterRemovalMaterial::maskedProgram(ps.c_str(),false));
            if(!known)opaque=false;++opaque_passes;}}
        opaque=opaque&&opaque_passes!=0;
        if(!LL_GetChild(i)->AppendOriginalSkinProxySource(*proxy_source,opaque)){proxy_ok=false;break;}
    }
    if(proxy_ok)painter_skin_proxy::precompute(*proxy_source,*this);
    m_original_skin_proxy_source=proxy_ok&&!proxy_source->vertices.empty()?proxy_source:std::shared_ptr<const painter_skin_proxy::Source>{}; } catch(...) { m_original_skin_proxy_source.reset(); }
}

//--DSR-- SilencerOverheat_start
IRenderVisual* CKinematics::GetVisualByBone(u16 bone_id)
{
	for (u32 it = 0; it < children.size(); it++)
	{
		IRenderVisual* child = children[it];
		CSkeletonX* childSkel = smart_cast<CSkeletonX*>(child);
		if (childSkel->has_bone_id(bone_id))
		{
			return child;
		}
	}
	return 0;
}

IRenderVisual* CKinematics::GetVisualByBone(LPCSTR bone_name)
{
	u16 bone_id = LL_BoneID(bone_name);
	if (bone_id != BI_NONE)
	{
		return GetVisualByBone(bone_id);
	}
	return 0;
}
//--DSR-- SilencerOverheat_end

IC void iBuildGroups(CBoneData* B, U16Vec& tgt, u16 id, u16& last_id)
{
	if (B->IK_data.ik_flags.is(SJointIKData::flBreakable)) id = ++last_id;
	tgt[B->GetSelfID()] = id;
	for (xr_vector<CBoneData*>::iterator bone_it = B->children.begin(); bone_it != B->children.end(); bone_it++)
		iBuildGroups(*bone_it, tgt, id, last_id);
}

void CKinematics::LL_Validate()
{
	// check breakable
	BOOL bCheckBreakable = FALSE;
	for (u16 k = 0; k < LL_BoneCount(); k++)
	{
		if (LL_GetData(k).IK_data.ik_flags.is(SJointIKData::flBreakable) && (LL_GetData(k).IK_data.type != jtNone))
		{
			bCheckBreakable = TRUE;
			break;
		}
	}

	if (bCheckBreakable)
	{
		BOOL bValidBreakable = TRUE;

#pragma todo("container is created in stack!")
		xr_vector<xr_vector<u16>> groups;
		LL_GetBoneGroups(groups);

#pragma todo("container is created in stack!")
		xr_vector<u16> b_parts(LL_BoneCount(), BI_NONE);
		CBoneData* root = &LL_GetData(LL_GetBoneRoot());
		u16 last_id = 0;
		iBuildGroups(root, b_parts, 0, last_id);

		for (u16 g = 0; g < (u16)groups.size(); ++g)
		{
			xr_vector<u16>& group = groups[g];
			u16 bp_id = b_parts[group[0]];
			for (u32 b = 1; b < groups[g].size(); b++)
				if (bp_id != b_parts[groups[g][b]])
				{
					bValidBreakable = FALSE;
					break;
				}
		}

		if (bValidBreakable == FALSE)
		{
			for (u16 k = 0; k < LL_BoneCount(); k++)
			{
				CBoneData& BD = LL_GetData(k);
				if (BD.IK_data.ik_flags.is(SJointIKData::flBreakable))
					BD.IK_data.ik_flags.set(SJointIKData::flBreakable, FALSE);
			}
			Msg("! ERROR: Invalid breakable object: '%s'", *dbg_name);
		}
	}
}


CBoneData::FacesVec& CKinematics::GeometryFaces(u16 bone_id, u16 source_child_index)
{
    if (m_runtime_geometry) return m_runtime_geometry->faces.at(bone_id).at(source_child_index);
    CKinematics* source = m_geometry_donor ? m_geometry_donor : this;
    VERIFY(bone_id < source->LL_BoneCount());
    CBoneData& data = source->LL_GetData(bone_id);
    VERIFY(source_child_index < data.child_faces.size());
    return data.child_faces[source_child_index];
}

bool CKinematics::LL_InstallGeometryVariant(IRenderVisual*& staged)
{
    // This is a low-level prototype, NOT a Lua admission API. A caller must
    // verify source/variant hashes, dead owner, current cut set, removed-weight
    // contract and safe main-thread barrier before calling it.
    if (!staged || bDontDelete || m_runtime_geometry) return false;
    CKinematics* donor = fast_dynamic_cast<CKinematics*>(staged);
    if (!donor || donor == this || donor == m_geometry_donor || donor->m_geometry_donor || donor->bDontDelete ||
        donor->children.empty() || !donor->children_invisible.empty() ||
        donor->LL_BoneCount() != LL_BoneCount()) return false;
    for (u16 b = 0; b < LL_BoneCount(); ++b)
    {
        const CBoneData& a = LL_GetData(b);
        const CBoneData& d = donor->LL_GetData(b);
        if (xr_strcmp(LL_BoneName_dbg(b), donor->LL_BoneName_dbg(b)) ||
            a.GetParentID() != d.GetParentID() ||
            !a.m2b_transform.i.similar(d.m2b_transform.i, 1e-6f) ||
            !a.m2b_transform.j.similar(d.m2b_transform.j, 1e-6f) ||
            !a.m2b_transform.k.similar(d.m2b_transform.k, 1e-6f) ||
            !a.m2b_transform.c.similar(d.m2b_transform.c, 1e-6f) ||
            d.child_faces.size() != donor->children.size()) return false;
    }
    xr_vector<u8> seen;
    seen.assign(donor->children.size(), u8(0));
    for (u32 c = 0; c < donor->children.size(); ++c)
    {
        CSkeletonX* child = fast_dynamic_cast<CSkeletonX*>(donor->children[c]);
        if (!child || child->GeometryChildIndex() >= seen.size() ||
            seen[child->GeometryChildIndex()]) return false;
        seen[child->GeometryChildIndex()] = true;
    }

    // Validate the complete next state before changing this instance. A later
    // damage state replaces its predecessor, while the original stays available
    // for restoration. The caller must hold the worker-joined update barrier.
    if (::RImplementation.Wallmarks) ::Render->remove_SkeletonWallmarksFromObject(this);
    ClearWallmarks();
    CKinematics* retired_donor = m_geometry_donor;
    {
        xrCriticalSectionGuard pose_guard(&UCalc_Mutex);
        if (retired_donor)
        {
            retired_donor->children.swap(children);
            retired_donor->children_invisible.swap(children_invisible);
            for (auto* child : retired_donor->children)
                fast_dynamic_cast<CSkeletonX*>(child)->SetParent(retired_donor);
            for (auto* child : retired_donor->children_invisible)
                fast_dynamic_cast<CSkeletonX*>(child)->SetParent(retired_donor);
        }
        else
        {
            m_geometry_original_children.swap(children);
            m_geometry_original_invisible.swap(children_invisible);
            m_geometry_original_lod = m_lod;
        }
        children.swap(donor->children);
        m_lod = nullptr; // An intact LOD must never reappear at distance.
        m_geometry_donor = donor;
        ++m_render_geometry_generation;
        staged = nullptr;
        for (auto* child : children)
            fast_dynamic_cast<CSkeletonX*>(child)->SetParent(this);
        CalculateBones_Invalidate();
        Visibility_Invalidate();
    }
    // Model-pool operations can invoke Depart; never hold the pose lock here.
    if (retired_donor)
    {
        IRenderVisual* disposable = retired_donor;
        ::Render->model_Delete(disposable);
    }
    return true;
}

void CKinematics::LL_ClearGeometryVariant()
{
    if (!m_geometry_donor) return;
    // Run only at the same safe barrier as install; not from a render callback.
    CKinematics* donor = m_geometry_donor;
    if (::RImplementation.Wallmarks) ::Render->remove_SkeletonWallmarksFromObject(this);
    ClearWallmarks();
    {
        xrCriticalSectionGuard pose_guard(&UCalc_Mutex);
        donor->children.swap(children);
        donor->children_invisible.swap(children_invisible);
        for (auto* child : donor->children)
            fast_dynamic_cast<CSkeletonX*>(child)->SetParent(donor);
        for (auto* child : donor->children_invisible)
            fast_dynamic_cast<CSkeletonX*>(child)->SetParent(donor);
        children.swap(m_geometry_original_children);
        children_invisible.swap(m_geometry_original_invisible);
        m_lod = m_geometry_original_lod;
        m_geometry_original_lod = nullptr;
        m_geometry_donor = nullptr;
        ++m_render_geometry_generation;
        CalculateBones_Invalidate();
        Visibility_Invalidate();
    }
    IRenderVisual* disposable = donor;
    ::Render->model_Delete(disposable);
}

void CKinematics::Copy(dxRender_Visual* P)
{
	VERIFY(!m_geometry_donor && !m_runtime_geometry);
	VERIFY(!((CKinematics*)P->dcast_PKinematics())->m_geometry_donor && !((CKinematics*)P->dcast_PKinematics())->m_runtime_geometry);
#ifdef USE_DX11
    // Copying a clean visual family must not copy another NPC's blood bindings.
    ClearBloodCells();
    static_cast<CKinematics*>(P->dcast_PKinematics())->RestoreBloodMaterials();
#endif
	inherited::Copy(P);

	CKinematics* pFrom = (CKinematics*)P->dcast_PKinematics();
	VERIFY(pFrom);
	pUserData = pFrom->pUserData;
    m_original_skin_proxy_source=pFrom->m_original_skin_proxy_source;
	bones = pFrom->bones;
	bones_size = pFrom->bones_size;
	iRoot = pFrom->iRoot;
	bone_map_N = pFrom->bone_map_N;
	bone_map_P = pFrom->bone_map_P;
	visimask = pFrom->visimask;
	hidden_bones = pFrom->hidden_bones;

	IBoneInstances_Create();

	for (u32 i = 0; i < children.size(); i++)
		LL_GetChild(i)->SetParent(this);

	CalculateBones_Invalidate();

	m_lod = (pFrom->m_lod) ? (dxRender_Visual*)::Render->model_Duplicate(pFrom->m_lod) : 0;
}

void CKinematics::CalculateBones_Invalidate()
{
	UCalc_Time = 0x0;
	UCalc_Visibox = psSkeletonUpdate;
}

void CKinematics::Spawn()
{
#ifdef USE_DX11
    ClearBloodCells();
#endif
	inherited::Spawn();
	// bones
	for (u32 i = 0; i < bones->size(); i++)
		bone_instances[i].construct();
	Update_Callback = NULL;
	CalculateBones_Invalidate();
	// wallmarks
	ClearWallmarks();
	Visibility_Invalidate();
	LL_SetBoneRoot(0);
}

void CKinematics::Depart()
{
#ifdef USE_DX11
    ClearBloodCells();
#endif
	// Pool reuse must expose the original family again.
	LL_ClearRuntimeGeometry();
	LL_ClearGeometryVariant();
	// R65: and come back as a fresh instance - the clears above advanced the generation, and Painter binds only generation 0, so the
	// next NPC given this pooled instance was refused (no wounds). Stale stages of the old owner fail their owner-cookie check.
	m_render_geometry_generation = 0;
	inherited::Depart();
	// wallmarks
	ClearWallmarks();

	// unmask all bones
	hidden_bones.zero();
	visimask.zero();
	if (bones)
	{
		u32 count = bones->size();
#ifdef DEBUG
		if (count > 64)
			Msg("ahtung !!! %d", count);
#endif // #ifdef DEBUG
		for (u32 b = 0; b < count; b++) { visimask.set((u64(1) << b), TRUE); hidden_bones.set((u64(1) << b), TRUE); }
	}
	// visibility
	children.insert(children.end(), children_invisible.begin(), children_invisible.end());
	children_invisible.clear();
}

void CKinematics::Release()
{
    PhysicalScopeGlassMask = 0;
#ifdef USE_DX11
    ClearBloodCells();
#endif
    m_original_skin_proxy_source.reset();
	// xr_free bones
	for (u32 i = 0; i < bones->size(); i++)
	{
		CBoneData*& B = (*bones)[i];
		xr_delete(B);
	}

	// destroy shared data
	xr_delete(pUserData);
	xr_delete(bones);
	xr_delete(bone_map_N);
	xr_delete(bone_map_P);

	inherited::Release();
}

// R28: the matrix LL_SetBone(s)Visible leaves on a hidden bone (scale 0, the parent's translation). A hidden bone that no
// longer holds it was driven meanwhile (a callback) and keeps that pose when shown.
static bool bone_matrix_collapsed(const Fmatrix& m)
{
	return m.i.square_magnitude() == 0.f && m.j.square_magnitude() == 0.f && m.k.square_magnitude() == 0.f;
}

void CKinematics::LL_SetBoneVisible(u16 bone_id, BOOL val, BOOL bRecursive)
{
	VERIFY(bone_id < LL_BoneCount());
	if (bone_id >= LL_BoneCount())
		return;
	u64 mask = u64(1) << bone_id;
	const bool was_visible = visimask.is(mask);
	visimask.set(mask, val);
	if (!visimask.is(mask))
	{
		if (was_visible)
			bone_instances[bone_id].mTransformHidden = bone_instances[bone_id].mTransform;
		bone_instances[bone_id].mTransform.scale(0.f, 0.f, 0.f);
		u16 parent_id = LL_GetData(bone_id).GetParentID();
		if (parent_id < LL_BoneCount() && parent_id != BI_NONE)
			bone_instances[bone_id].mTransform.c = LL_GetBoneInstance(parent_id).mTransform.c;

		bone_instances[bone_id].mRenderTransform.mul_43(bone_instances[bone_id].mTransform,
			(*bones)[bone_id]->m2b_transform);
	}
	else
	{
		if (!was_visible)
		{
			CBoneInstance& bi = bone_instances[bone_id];
			if (bone_matrix_collapsed(bi.mTransform)) bi.mTransform = bi.mTransformHidden;
			bi.mRenderTransform.mul_43(bi.mTransform, (*bones)[bone_id]->m2b_transform);
		}
		CalculateBones_Invalidate();
	}

	if (bRecursive)
	{
		for (xr_vector<CBoneData*>::iterator C = (*bones)[bone_id]->children.begin(); C != (*bones)[bone_id]
			->children.end(); C++)
			LL_SetBoneVisible((*C)->GetSelfID(), val, bRecursive);
	}
	Visibility_Invalidate();
}

void CKinematics::LL_SetBonesVisible(u64 mask)
{
	const u64 previous = visimask.flags;
	visimask.assign(0);
	for (u32 bone_id = 0; bone_id < bones->size(); bone_id++)
	{
		u64 bm = u64(1) << bone_id;
		const bool was_visible = (previous & bm) != 0;
		if (mask & bm)
		{
			visimask.set(bm, TRUE);
			if (!was_visible)
			{
				// R28: a bone driven while hidden (a callback) keeps that pose; only the collapsed matrix is replaced.
				CBoneInstance& bi = bone_instances[bone_id];
				if (bone_matrix_collapsed(bi.mTransform)) bi.mTransform = bi.mTransformHidden;
				bi.mRenderTransform.mul_43(bi.mTransform, (*bones)[bone_id]->m2b_transform);
			}
		}
		else
		{
			if (was_visible)
				bone_instances[bone_id].mTransformHidden = bone_instances[bone_id].mTransform;
			bone_instances[bone_id].mTransform.scale(0.f, 0.f, 0.f);

			u16 parent_id = LL_GetData(bone_id).GetParentID();
			if (parent_id < LL_BoneCount() && parent_id != BI_NONE)
				bone_instances[bone_id].mTransform.c = LL_GetBoneInstance(parent_id).mTransform.c;

			bone_instances[bone_id].mRenderTransform.mul_43(bone_instances[bone_id].mTransform,
				(*bones)[bone_id]->m2b_transform);
		}
	}
	CalculateBones_Invalidate();
	Visibility_Invalidate();
}

bool CKinematics::CopyBonesFrom(IKinematics* _from)
{
	CKinematics* from = smart_cast<CKinematics*>(_from);
	if (!from || from == this)
		return false;

	// Calculate now, so that the renderer doesn't recalculate the bones this frame and overwrite the copy
	CalculateBones_Invalidate();
	CalculateBones(TRUE);

	CBoneInstance& root = bone_instances[iRoot];
	root.mTransform.identity();
	root.mRenderTransform.mul_43(root.mTransform, (*bones)[iRoot]->m2b_transform);

	auto copy_bone = [&](u16 dst_id, u16 src_id)
	{
		CBoneInstance& dst = bone_instances[dst_id];
		const Fmatrix& pose = from->LL_GetTransform(src_id);

		// Hidden bones stay collapsed, but keep the pose for when they are shown again
		dst.mTransformHidden = pose;
		if (LL_GetBoneVisible(dst_id))
		{
			dst.mTransform = pose;
			dst.mRenderTransform = from->bone_instances[src_id].mRenderTransform;
		}
	};

	if (LL_BoneCount() == from->LL_BoneCount())
	{
		for (u16 i = 0; i < LL_BoneCount(); ++i)
			copy_bone(i, i);
	}
	else
	{
		for (const auto& [name, id] : *bone_map_N)
		{
			u16 src_id = from->LL_BoneID(name);
			if (src_id != BI_NONE)
				copy_bone(id, src_id);
		}
	}

	// Bounds of the copied pose
	vis.box = from->vis.box;
	vis.sphere = from->vis.sphere;

	return true;
}

void CKinematics::Visibility_Update()
{
	Update_Visibility = FALSE;
	// check visible
	for (u32 c_it = 0; c_it < children.size(); c_it++)
	{
		CSkeletonX* _c = smart_cast<CSkeletonX*>(children[c_it]);
		VERIFY(_c);
		if (!_c->has_visible_bones())
		{
			// move into invisible list
			children_invisible.push_back((dxRender_Visual*)children[c_it]);
			swap(children[c_it], children.back());
			children.pop_back();
			Update_Visibility = TRUE;
		}
	}

	// check invisible
	for (u32 _it = 0; _it < children_invisible.size(); _it++)
	{
		CSkeletonX* _c = smart_cast<CSkeletonX*>(children_invisible[_it]);
		VERIFY(_c) ;
		if (_c->has_visible_bones())
		{
			// move into visible list
			children.push_back(children_invisible[_it]);
			swap(children_invisible[_it], children_invisible.back());
			children_invisible.pop_back();
			Update_Visibility = TRUE;
		}
	}
}

IC static void RecursiveBindTransform(CKinematics* K, xr_vector<Fmatrix>& matrices, u16 bone_id, const Fmatrix& parent)
{
	CBoneData& BD = K->LL_GetData(bone_id);
	Fmatrix& BM = matrices[bone_id];
	// Build matrix
	BM.mul_43(parent, BD.bind_transform);
	for (xr_vector<CBoneData*>::iterator C = BD.children.begin(); C != BD.children.end(); C++)
		RecursiveBindTransform(K, matrices, (*C)->GetSelfID(), BM);
}

void CKinematics::LL_GetBindTransform(xr_vector<Fmatrix>& matrices)
{
	matrices.resize(LL_BoneCount());
	RecursiveBindTransform(this, matrices, iRoot, Fidentity);
}

// Restrict square coverage to the paired SSS blood pixel shader. Unknown or
// multipass materials retain their exact legacy sphere collection/projection.
static bool UsesBloodSquareFootprint(const ref_shader& shader)
{
    if (!shader || !shader->E[0] || shader->E[0]->passes.size() != 1) return false;
    const auto& pass = shader->E[0]->passes[0];
    if (!pass || !pass->ps || !pass->ps->cName.size()) return false;
    const char* name = pass->ps->cName.c_str();
    const char prefix[] = "effects_wallmark_blood";
    const size_t length = sizeof(prefix)-1;
    if (std::strncmp(name,prefix,length)) return false;
    // DX10 resource manager adds only these MSAA suffixes to the PS identity.
    return name[length] == 0 || (name[length] == '_' && name[length+1] >= '0' &&
        name[length+1] <= '7' && name[length+2] == 0);
}

void BuildMatrix(Fmatrix& mView, float invsz, const Fvector norm, const Fvector& from)
{
	// build projection
	Fmatrix mScale;
	Fvector at, up, right, y;
	at.sub(from, norm);
	y.set(0, 1, 0);
	if (_abs(norm.y) > .99f) y.set(1, 0, 0);
	right.crossproduct(y, norm);
	up.crossproduct(norm, right);
	mView.build_camera(from, at, up);
	mScale.scale(invsz, invsz, invsz);
	mView.mulA_43(mScale);
}

void CKinematics::EnumBoneVertices(SEnumVerticesCallback& C, u16 bone_id)
{
	for (u32 i = 0; i < children.size(); i++)
		LL_GetChild(i)->EnumBoneVertices(C, bone_id);
}

#include "cl_intersect.h"

DEFINE_VECTOR(Fobb, OBBVec, OBBVecIt);

// triggerhitch (2026-11-15): painter_exit_surface's nearest face over ALL bones (entry face of every received pellet), in
// one pass over one posed copy of the vertices - the same answer as asking PickBone below bone by bone with a shrinking
// bound (PainterPickNearest.h). -1: not available here (the caller asks PickBone as before).
int CKinematics::LL_PickNearest(const Fmatrix& parent_xform, IKinematics::pick_result& best, float range, const Fvector& start,
                                const Fvector& dir, u16 count)
{
#ifdef USE_DX11
    const u16 bones = LL_BoneCount();
    if (count != bones || !bones || children.empty()) return -1;
    PainterPickMemo& memo = painter_pick_memo();
    std::lock_guard<std::mutex> lock(memo.mutex);
    bool current = memo.valid && memo.owner == this && memo.frame == RDEVICE.dwFrame &&
        memo.generation == m_render_geometry_generation && memo.pose.size() == bones && memo.children.size() == children.size();
    for (u16 b = 0; current && b < bones; ++b)
        current = !std::memcmp(&memo.pose[b], &LL_GetBoneInstance(b).mRenderTransform, sizeof(Fmatrix));
    for (u32 i = 0; current && i < children.size(); ++i)
    {
        CSkeletonX* c = LL_GetChild(i);
        const PainterPickMemo::Child& m = memo.children[i];
        current = c == m.child && c->PainterPickParent() == this && c->PainterPickVertexKey() == m.vertices &&
            c->PainterPickVertexCount() == m.count && c->PainterPickIndices() == m.indices && c->PainterPickIndexCount() == m.index_count;
    }
    if (!current)
    {
        memo.valid = false;
        memo.owner = this;
        memo.frame = RDEVICE.dwFrame;
        memo.generation = m_render_geometry_generation;
        memo.pose.resize(bones);
        for (u16 b = 0; b < bones; ++b)
            std::memcpy(&memo.pose[b], &LL_GetBoneInstance(b).mRenderTransform, sizeof(Fmatrix));
        memo.children.resize(children.size());
        for (u32 i = 0; i < children.size(); ++i)
        {
            CSkeletonX* c = LL_GetChild(i);
            PainterPickMemo::Child& m = memo.children[i];
            if (!c || c->PainterPickParent() != this) return -1;
            m.child = c;
            m.vertices = c->PainterPickVertexKey();
            m.count = c->PainterPickVertexCount();
            m.indices = c->PainterPickIndices();
            m.index_count = c->PainterPickIndexCount();
            if (!m.vertices || !m.count || !m.indices || !m.index_count) return -1;
            m.posed.resize(m.count);
            if (!c->PainterPoseVertices(m.posed.data(), m.count)) return -1;
            m.finite = true;
            for (const Fvector& p : m.posed) m.finite = m.finite && std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
            m.boxes.resize(bones);
            for (painter_pick_nearest::Box& b : m.boxes) b.ready = false; // keeps the run arrays' memory
        }
        memo.valid = true;
    }
    Fvector S, D;
    Fmatrix P;
    P.invert(parent_xform);
    P.transform_tiny(S, start);
    P.transform_dir(D, dir);
    struct Model
    {
        CKinematics& k;
        PainterPickMemo& memo;
        const Fvector& S;
        const Fvector& D;
        bool visible(u16 bone) { return !!k.LL_GetBoneVisible(bone); }
        u32 children() { return u32(memo.children.size()); }
        const u16* faces(u16 bone, u32 child, size_t& n)
        {
            CBoneData::FacesVec& list = k.GeometryFaces(bone, memo.children[child].child->GeometryChildIndex());
            n = list.size();
            return n ? list.data() : nullptr;
        }
        const Fvector* posed(u32 child) { return memo.children[child].posed.data(); }
        u32 vertices(u32 child) { return memo.children[child].count; }
        const u16* indices(u32 child) { return memo.children[child].indices; }
        size_t index_count(u32 child) { return memo.children[child].index_count; }
        painter_pick_nearest::Box& box(u32 child, u16 bone) { return memo.children[child].boxes[bone]; }
        bool finite(u32 child) { return memo.children[child].finite; }
        bool test(Fvector* tri, float& u, float& v, float& dist) { return CDB::TestRayTri(S, D, tri, u, v, dist, true); }
        bool accept(u32 child, u16 face, float u, float v)
        {
            const PainterPickMemo::Child& m = memo.children[child];
            return m.child->PainterPickAccept(face, m.indices, u, v);
        }
    } model{*this, memo, S, D};
    painter_pick_nearest::Record record;
    Fvector tri[3];
    const int got = painter_pick_nearest::scan<Fvector>(model, S, D, range, count, record, tri);
    if (got != 1) return got;
    // PickBone's answer: pick_bone's model-space normal and corners, then CKinematics::PickBone's transforms
    best.dist = record.dist;
    best.tri[0] = tri[0];
    best.tri[1] = tri[1];
    best.tri[2] = tri[2];
    best.normal.mknormal(best.tri[0], best.tri[1], best.tri[2]);
    parent_xform.transform_dir(best.normal);
    parent_xform.transform_tiny(best.tri[0]);
    parent_xform.transform_tiny(best.tri[1]);
    parent_xform.transform_tiny(best.tri[2]);
    return 1;
#else
    return -1;
#endif
}

bool CKinematics::PickBone(const Fmatrix& parent_xform, IKinematics::pick_result& r, float dist, const Fvector& start,
                           const Fvector& dir, u16 bone_id)
{
	Fvector S, D; //normal		= {0,0,0}
	// transform ray from world to model
	Fmatrix P;
	P.invert(parent_xform);
	P.transform_tiny(S, start);
	P.transform_dir(D, dir);
	for (u32 i = 0; i < children.size(); i++)
		if (LL_GetChild(i)->PickBone(r, dist, S, D, bone_id))
		{
			parent_xform.transform_dir(r.normal);
			parent_xform.transform_tiny(r.tri[0]);
			parent_xform.transform_tiny(r.tri[1]);
			parent_xform.transform_tiny(r.tri[2]);
			return true;
		}
	return false;
}

// bloodfix 2026-11-13: what the surface placements did, in one line at most every 20 s (the per-mark lines stay verbose-only).
// placed = the contact was the outer skin; lifted = it lay under a covering layer within 6 cm and was moved onto it;
// under_far_cover = another part within 15 cm covered it and the mark was placed at its own contact; refused = no receiver
// (bones / faces) or a covered sever splash / blade-cut mark (their callers place a fallback).
namespace BloodSurfaceTally
{
enum Kind : unsigned { Placed, Lifted, FarCover, Refused, Kinds };
static std::atomic<u32> g_count[Kinds];
static std::atomic<u32> g_max_lift_mm{0}, g_reported_ms{0}, g_dirty{0};
static void note(Kind kind, float lift = 0.f)
{
    g_count[kind].fetch_add(1, std::memory_order_relaxed);
    g_dirty.store(1, std::memory_order_relaxed);
    const u32 mm = lift > 0.f ? u32(lift * 1000.f + .5f) : 0u;
    u32 seen = g_max_lift_mm.load(std::memory_order_relaxed);
    while (mm > seen && !g_max_lift_mm.compare_exchange_weak(seen, mm, std::memory_order_relaxed)) {}
}
static void report()
{
    const u32 now = RDEVICE.dwTimeGlobal;
    u32 last = g_reported_ms.load(std::memory_order_relaxed);
    if (!g_dirty.load(std::memory_order_relaxed) || (last && u32(now - last) < 20000u)) return;
    if (!g_reported_ms.compare_exchange_strong(last, now ? now : 1u, std::memory_order_relaxed)) return;
    g_dirty.store(0, std::memory_order_relaxed);
    Msg("[blood_surface] tally placed=%u lifted=%u max_lift_mm=%u under_far_cover=%u refused=%u",
        g_count[Placed].load(std::memory_order_relaxed), g_count[Lifted].load(std::memory_order_relaxed),
        g_max_lift_mm.load(std::memory_order_relaxed), g_count[FarCover].load(std::memory_order_relaxed),
        g_count[Refused].load(std::memory_order_relaxed));
}
}

bool CKinematics::AddWallmark(const Fmatrix* parent_xform, const Fvector3& start, const Fvector3& dir,
                              ref_shader shader, float size, float ttl, const Fvector* surface_normal, const BloodWallmarkGrowth* growth,const BloodThermalStamp* thermal_stamp,
                              float outer_depth, const float* roll_deg)
{
	BloodNativeProfile::Scope bloodProfile(BloodNativeProfile::Stage::BodyCreate);
	painter_hit_main::PartScope hit_main_wallmark(painter_hit_main::Wallmark); // triggerhitch
    // All character routes converge here, before pick/receiver/projection.
    // Exact existing shader identity excludes unrelated skeleton decals.
    if(!_valid(size) || size<=EPS_L)return false;
    const bool square_footprint = UsesBloodSquareFootprint(shader);
    const float requested_size=size;
    const bool sever_splash=thermal_stamp && thermal_stamp->sever_splash;
    const bool blade_cut=thermal_stamp && thermal_stamp->mark_key && thermal_stamp->mark_revision;
    size=PainterBloodModelSize::Cap(size,square_footprint,ps_r_blood_decal_model_max_size,sever_splash||blade_cut);
    if(square_footprint && painter_log_verbose())
        Msg("[blood_model_size] requested=%.6f final=%.6f limit=%.6f sever_splash=%u blade_cut=%u",requested_size,size,ps_r_blood_decal_model_max_size,unsigned(sever_splash),unsigned(blade_cut));
#ifdef USE_DX11
    // CPU and GPU marks retain the same placement for later mesh revisions.
    xrCriticalSectionGuard hit_pose_guard(&UCalc_Mutex);
#endif
#ifdef USE_DX11
	// wallmarkpick (2026-11-16): the pick, the look-back and the decal faces below read the posed copy of LL_PickNearest's memo
	// (PainterWallmarkScan.h) when it is free; it is made current here for this body, pose, frame and children as LL_PickNearest
	// makes it (the pellets of one shot share it). Every visit used to skin its face's corners again: GFY 1.06 14.3-15.2 ms a
	// call in game. Same answers, same faces in the same order; when the memo is busy or a list cannot be boxed the old loops run.
	std::unique_lock<std::mutex> wm_memo_lock(painter_pick_memo().mutex, std::try_to_lock);
	auto wm_memo_ready = [&]() -> bool
	{
		const u16 bones = LL_BoneCount();
		if (!bones || children.empty()) return false;
		PainterPickMemo& memo = painter_pick_memo();
		bool current = memo.valid && memo.owner == this && memo.frame == RDEVICE.dwFrame &&
			memo.generation == m_render_geometry_generation && memo.pose.size() == bones && memo.children.size() == children.size();
		for (u16 b = 0; current && b < bones; ++b)
			current = !std::memcmp(&memo.pose[b], &LL_GetBoneInstance(b).mRenderTransform, sizeof(Fmatrix));
		for (u32 i = 0; current && i < children.size(); ++i)
		{
			CSkeletonX* c = LL_GetChild(i);
			const PainterPickMemo::Child& m = memo.children[i];
			current = c == m.child && c->PainterPickParent() == this && c->PainterPickVertexKey() == m.vertices &&
				c->PainterPickVertexCount() == m.count && c->PainterPickIndices() == m.indices && c->PainterPickIndexCount() == m.index_count;
		}
		if (current) return true;
		memo.valid = false;
		memo.owner = this;
		memo.frame = RDEVICE.dwFrame;
		memo.generation = m_render_geometry_generation;
		memo.pose.resize(bones);
		for (u16 b = 0; b < bones; ++b)
			std::memcpy(&memo.pose[b], &LL_GetBoneInstance(b).mRenderTransform, sizeof(Fmatrix));
		memo.children.resize(children.size());
		for (u32 i = 0; i < children.size(); ++i)
		{
			CSkeletonX* c = LL_GetChild(i);
			PainterPickMemo::Child& m = memo.children[i];
			if (!c || c->PainterPickParent() != this) return false;
			m.child = c;
			m.vertices = c->PainterPickVertexKey();
			m.count = c->PainterPickVertexCount();
			m.indices = c->PainterPickIndices();
			m.index_count = c->PainterPickIndexCount();
			if (!m.vertices || !m.count || !m.indices || !m.index_count) return false;
			m.posed.resize(m.count);
			if (!c->PainterPoseVertices(m.posed.data(), m.count)) return false;
			m.finite = true;
			for (const Fvector& p : m.posed) m.finite = m.finite && std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
			m.boxes.resize(bones);
			for (painter_pick_nearest::Box& b : m.boxes) b.ready = false;
		}
		memo.valid = true;
		return true;
	};
	const bool wm_scan = wm_memo_lock.owns_lock() && wm_memo_ready();
	struct WallmarkModel
	{
		CKinematics& k;
		PainterPickMemo& memo;
		u32 children() { return u32(memo.children.size()); }
		bool receives(u32 child) { return memo.children[child].child->receives_wallmarks; }
		const u16* faces(u16 bone, u32 child, size_t& n)
		{
			CBoneData::FacesVec& list = k.GeometryFaces(bone, memo.children[child].child->GeometryChildIndex());
			n = list.size();
			return n ? list.data() : nullptr;
		}
		const Fvector* posed(u32 child) { return memo.children[child].posed.data(); }
		u32 vertices(u32 child) { return memo.children[child].count; }
		const u16* indices(u32 child) { return memo.children[child].indices; }
		size_t index_count(u32 child) { return memo.children[child].index_count; }
		painter_pick_nearest::Box& box(u32 child, u16 bone) { return memo.children[child].boxes[bone]; }
		bool finite(u32 child) { return memo.children[child].finite; }
		bool test(const Fvector& from, const Fvector& dir, Fvector* tri, float& u, float& v, float& dist) { return CDB::TestRayTri(from, dir, tri, u, v, dist, true); }
		bool accept(u32 child, u16 face, float u, float v)
		{
			const PainterPickMemo::Child& m = memo.children[child];
			return m.child->PainterPickAccept(face, m.indices, u, v);
		}
	} wm_model{*this, painter_pick_memo()};
#else
	const bool wm_scan = false;
#endif
	BloodNativeProfile::Scope bloodPick(BloodNativeProfile::Stage::BodyPick);
	Fvector S, D, normal = {0, 0, 0};
	// transform ray from world to model
	Fmatrix P;
	P.invert(*parent_xform);
	P.transform_tiny(S, start);
	P.transform_dir(D, dir);
	// find pick point
	float dist = flt_max;
	BOOL picked = FALSE;

	DEFINE_VECTOR(Fobb, OBBVec, OBBVecIt);
	OBBVec cache_obb;
	cache_obb.resize(LL_BoneCount());
	IKinematics::pick_result r;
	r.normal = normal;
	r.dist = dist;
	for (u16 k = 0; k < LL_BoneCount(); k++)
	{
		CBoneData& BD = LL_GetData(k);
		if (LL_GetBoneVisible(k) && !BD.shape.flags.is(SBoneShape::sfNoPickable))
		{
			Fobb& obb = cache_obb[k];
			obb.transform(BD.obb, LL_GetBoneInstance(k).mTransform);
			if (!surface_normal && !wm_scan && CDB::TestRayOBB(S, D, obb)) // wallmarkpick: wm_scan picks below
				for (u32 i = 0; i < children.size(); i++)
				{
					if (LL_GetChild(i)->PickBone(r, dist, S, D, k))
					{
						picked = TRUE;
						dist = r.dist;
						normal = r.normal;
						//dynamics set wallmarks bug fix
					}
				}
		}
	}
#ifdef USE_DX11
	if (wm_scan && !surface_normal) // wallmarkpick: the pick above in one pass on the posed copy (the same answer)
	{
		float wm_dist = dist;
		Fvector wm_tri[3];
		const int wm_got = painter_wallmark_scan::first_hits<Fvector>(wm_model, [&](u16 k) {
			CBoneData& BD = LL_GetData(k);
			return LL_GetBoneVisible(k) && !BD.shape.flags.is(SBoneShape::sfNoPickable) && CDB::TestRayOBB(S, D, cache_obb[k]); },
			S, D, LL_BoneCount(), wm_dist, wm_tri);
		if (wm_got == 1)
		{
			picked = TRUE;
			dist = wm_dist;
			normal.mknormal(wm_tri[0], wm_tri[1], wm_tri[2]);
		}
		else if (wm_got < 0) // a face list it cannot box: PickBone as before
			for (u16 k = 0; k < LL_BoneCount(); k++)
			{
				CBoneData& BD = LL_GetData(k);
				if (LL_GetBoneVisible(k) && !BD.shape.flags.is(SBoneShape::sfNoPickable) && CDB::TestRayOBB(S, D, cache_obb[k]))
					for (u32 i = 0; i < children.size(); i++)
						if (LL_GetChild(i)->PickBone(r, dist, S, D, k))
						{
							picked = TRUE;
							dist = r.dist;
							normal = r.normal;
						}
			}
	}
#endif
    if(surface_normal) {P.transform_dir(normal,*surface_normal);normal.normalize_safe();dist=0.f;picked=TRUE;}
	bloodPick.Stop();
	if (!picked) return false;
	BloodNativeProfile::Scope bloodCollect(BloodNativeProfile::Stage::BodyCollect);

	// calculate contact point
	Fvector cp;
	cp.mad(S, D, dist);
	float surface_lift = 0.f;
	bool surface_far_cover = false;
	if (surface_normal)
	{
		constexpr float reach = 0.15f, own = 0.006f, lift_max = 0.06f;
		const bool refuse_covered = sever_splash || blade_cut;
		Fvector back;
		back.invert(normal);
		float top = reach; // the look-back starts this far above the contact
		for (int pass = 0; pass < 4 && top > own + 0.004f; ++pass)
		{
			Fvector from;
			from.mad(cp, normal, top);
			IKinematics::pick_result cover;
			cover.normal = normal;
			cover.dist = top;
			float nearest = top - own; // distance of the covering skin from the look-back start
			bool covered = false;
			int wm_cover = -1;
#ifdef USE_DX11
			if (wm_scan) // wallmarkpick: the look-back below in one pass a list on the posed copy (the same answers)
				wm_cover = painter_wallmark_scan::cover<Fvector>(wm_model, [&](u16 k) {
					CBoneData& BD = LL_GetData(k);
					return LL_GetBoneVisible(k) && !BD.shape.flags.is(SBoneShape::sfNoPickable) && CDB::TestRayOBB(from, back, cache_obb[k]); },
					from, back, LL_BoneCount(), refuse_covered, nearest, covered);
#endif
			for (u16 k = 0; wm_cover < 0 && k < LL_BoneCount() && !(covered && refuse_covered); k++)
			{
				CBoneData& BD = LL_GetData(k);
				if (!LL_GetBoneVisible(k) || BD.shape.flags.is(SBoneShape::sfNoPickable) || !CDB::TestRayOBB(from, back, cache_obb[k])) continue;
				for (u32 i = 0; i < children.size() && !(covered && refuse_covered); i++)
				{
					if (!LL_GetChild(i)->receives_wallmarks) continue;
					// PickBone answers the first face nearer than the limit: asked again until none is nearer (the outermost)
					for (int again = 0; again < 8 && LL_GetChild(i)->PickBone(cover, nearest, from, back, k); ++again)
					{
						covered = true;
						nearest = cover.dist;
						if (refuse_covered) break;
					}
				}
			}
			if (!covered) break;
			const float height = top - nearest; // of that skin above the contact
			if (refuse_covered)
			{
				if (painter_log_verbose()) Msg("[blood_surface] outcome=inner_surface cover=%.3f size=%.6f contact=%.6f,%.6f,%.6f", height, size, cp.x, cp.y, cp.z);
				BloodSurfaceTally::note(BloodSurfaceTally::Refused);
				return false;
			}
			if (height <= lift_max)
			{
				if (painter_log_verbose()) Msg("[blood_surface] outcome=lifted lift=%.3f size=%.6f contact=%.6f,%.6f,%.6f", height, size, cp.x, cp.y, cp.z);
				cp.mad(cp, normal, height);
				surface_lift = height;
				break;
			}
			surface_far_cover = true;
			top = height - 0.002f; // just under the far part's outer face
		}
	}

	// collect collide boxes
	Fsphere test_sphere;
	// square_footprint and final size were resolved before receiver collection.
    // UV growth requires the paired exact pixel clip. Never wrap an unknown material over a receiver triangle.
    if (growth && !square_footprint) return false;
    const bool cell_blood=
#ifdef USE_DX11
        PainterBloodBake::enabled()&&square_footprint;
#else
        false;
#endif
    auto growth_lease=growth&&!cell_blood ? BloodWallmarkGrowth::Reserve() : std::shared_ptr<BloodWallmarkGrowth::Lease>{};
    if(growth && !cell_blood && !growth_lease)return false;
	test_sphere.set(cp, size * (square_footprint ? BloodProjectionFootprint::CollectorScale : 1.f));
	U16Vec test_bones;
	test_bones.reserve(LL_BoneCount());
	for (u16 k = 0; k < LL_BoneCount(); k++)
	{
		CBoneData& BD = LL_GetData(k);
		if (LL_GetBoneVisible(k) && !BD.shape.flags.is(SBoneShape::sfNoPickable))
		{
			Fobb& obb = cache_obb[k];
			if (CDB::TestSphereOBB(test_sphere, obb))
				test_bones.push_back(k);
		}
	}

    if (test_bones.empty())
    {
        if (surface_normal) Msg("[blood_surface] outcome=no_receiver_bones size=%.6f contact=%.6f,%.6f,%.6f", size, cp.x, cp.y, cp.z);
        if (surface_normal) { BloodSurfaceTally::note(BloodSurfaceTally::Refused); BloodSurfaceTally::report(); } // bloodfix
        return false;
    }

	bloodCollect.Stop();
	BloodNativeProfile::Scope bloodListWait(BloodNativeProfile::Stage::BodyListWait);
	xrCriticalSectionGuard guard(wallmarks_cs);
	bloodListWait.Stop();

	// ok. allocate wallmark
	intrusive_ptr<CSkeletonWallmark> wm = xr_new<CSkeletonWallmark>(this, parent_xform, shader, cp, RDEVICE.fTimeGlobal, ttl ? ttl : ps_r__WallmarkTTL);
	wm->m_SquareFootprint = square_footprint;
	wm->m_OuterDepth = outer_depth; // 0 = the 2-4 cm rule (every call without a depth)
    if(thermal_stamp)wm->m_thermal_stamp=*thermal_stamp;
    if(blade_cut){
        if(!m_blood_mark_namespace)m_blood_mark_namespace=g_blood_mark_namespace.fetch_add(1,std::memory_order_relaxed);
        wm->m_thermal_stamp.mark_namespace=m_blood_mark_namespace;
    }
    if (growth) wm->m_Growth = *growth;
    wm->m_GrowthLease=std::move(growth_lease);
	wm->m_LocalBounds.set(cp, size * 2.f);
	wm->XFORM()->transform_tiny(wm->m_Bounds.P, cp);
	wm->m_Bounds.R = wm->m_LocalBounds.R;

	Fvector tmp;
	tmp.invert(D);
	normal.add(tmp).normalize();

	// build UV projection matrix
	Fmatrix mView, mRot;
	BuildMatrix(mView, 1 / (0.9f * size), normal, cp);
	// A surface call with a roll sets it; every other mark keeps the random +-20 degrees (same Random draw as before).
	if (roll_deg) mRot.rotateZ(deg2rad(*roll_deg));
	else mRot.rotateZ(::Random.randF(deg2rad(-20.f), deg2rad(20.f)));
	mView.mulA_43(mRot);

	BloodNativeProfile::Scope bloodProject(BloodNativeProfile::Stage::BodyProject);
#ifdef USE_DX11
	{
		try
		{
			auto hit = std::allocate_shared<CSkeletonWallmark::ProjectionSnapshot>(
				CSkeletonWallmark::CacheAllocator<CSkeletonWallmark::ProjectionSnapshot>{});
			hit->hit_pose.resize(LL_BoneCount());
			for (u16 b = 0; b < LL_BoneCount(); ++b)
				hit->hit_pose[b] = LL_GetBoneInstance(b).mRenderTransform;
			hit->view = mView; hit->world = *parent_xform;
			hit->normal = normal; hit->size = size; hit->outer_depth = outer_depth;
			hit->square_footprint = square_footprint;
            hit->epoch = g_blood_projection_epoch.load(std::memory_order_relaxed);
            hit->selected_bones = test_bones;
            // Copy while ranges are empty: retention cannot pin retired meshes.
            wm->m_Reprojection = std::allocate_shared<CSkeletonWallmark::ProjectionSnapshot>(
                CSkeletonWallmark::CacheAllocator<CSkeletonWallmark::ProjectionSnapshot>{}, *hit);
            // Publication has already built immutable sources for runtime
            // skin. New physical wounds arrive before render warmup; let that
            // frame upload them instead of synchronously projecting every face.
            // Physical wound marks (surface_normal) project on the CPU right away: the deferred GPU projection
            // needs the source's gpu epoch to still match at render, and every wound commit moves it, so marks
            // placed under sustained fire were dropped (in-game 2026-09-25: 120/120 outcome=gpu_pending, none seen).
            const bool defer_cold_runtime = false;
			bool complete = true;
			for (u32 i = 0; i < children.size() && complete; ++i)
				if(LL_GetChild(i)->receives_wallmarks)complete = LL_GetChild(i)->AppendProjectionRanges(test_bones, *hit, !defer_cold_runtime);
			if (complete && hit->candidate_vertices)
			{
				wm->m_Projection = std::move(hit);
				g_blood_projection_cpu.hit_snapshots.fetch_add(1, std::memory_order_relaxed);
			}
		}
        catch (...) {} // GPU source is optional; placement retention is not.
        if(!wm->m_Reprojection)return false;
	}
	if (!wm->m_Projection)
#endif
	{
	// fill vertices
	int wm_fill = -1;
#ifdef USE_DX11
	if (wm_scan) // wallmarkpick: the same faces in the same order with the posed corners; runs out of the mark's reach skipped
		wm_fill = painter_wallmark_scan::fill<Fvector>(wm_model, test_bones.data(), test_bones.size(),
			painter_wallmark_scan::MarkReach(wm->ContactPoint(), normal, size, outer_depth > 0.f ? outer_depth : BloodProjectionFootprint::OuterDepth(size),
				square_footprint, mView, BloodProjectionFootprint::CollectorScale, BloodProjectionFootprint::Depth),
			[&](u32 c, u16 face, Fvector* p) { LL_GetChild(c)->PainterFillPosed(mView, *wm, normal, size, face, p); });
#endif
	for (u32 i = 0; wm_fill < 0 && i < children.size(); i++)
	{
		CSkeletonX* S = LL_GetChild(i);
		if(!S->receives_wallmarks)continue;
		for (U16It b_it = test_bones.begin(); b_it != test_bones.end(); b_it++)
			S->FillVertices(mView, *wm, normal, size, *b_it);
	}
	}

    u32 pending_candidates = 0;
#ifdef USE_DX11
    if (wm->m_Projection) pending_candidates = wm->m_Projection->candidate_vertices;
#endif
    const bool accepted = pending_candidates || !wm->m_Faces.empty();
    if (surface_normal && (!accepted || painter_log_verbose())) // a decal with no receiver is always reported
        Msg("[blood_surface] outcome=%s bones=%zu gpu_candidate_vertices=%u cpu_faces=%zu size=%.6f contact=%.6f,%.6f,%.6f normal=%.6f,%.6f,%.6f",
            pending_candidates ? "gpu_pending" : (accepted ? "cpu_faces" : "no_receiver_faces"),
            test_bones.size(), pending_candidates, wm->m_Faces.size(), size, cp.x, cp.y, cp.z, normal.x, normal.y, normal.z);
    // GPU candidates still require projection. CPU empty marks are known to
    // have no receiver, so do not report them as accepted or retain them.
    if (surface_normal) // bloodfix: the tally line
    {
        BloodSurfaceTally::note(!accepted ? BloodSurfaceTally::Refused : surface_lift > 0.f ? BloodSurfaceTally::Lifted :
            surface_far_cover ? BloodSurfaceTally::FarCover : BloodSurfaceTally::Placed, surface_lift);
        BloodSurfaceTally::report();
    }
    if (!accepted) return false;
#ifdef USE_DX11
    if(cell_blood)return QueueBloodCells(wm); // no persistent skeleton wallmark on modes 1-3
#endif
    // Mode zero keeps native wallmarks, but keyed cuts still replace one
    // mark. A cloned piece owns a different namespace; inherited blood survives.
    if(blade_cut){
        for(auto it=wallmarks.begin();it!=wallmarks.end();++it){
            const auto& prior=(*it)->m_thermal_stamp;
            if(prior.mark_namespace==wm->m_thermal_stamp.mark_namespace && prior.mark_key==wm->m_thermal_stamp.mark_key){
                if(prior.mark_revision>=wm->m_thermal_stamp.mark_revision)return true;
                *it=wm;bloodProject.Stop();return true;
            }
        }
        wallmarks.push_back(wm);bloodProject.Stop();return true;
    }
    // Preserve an existing mark if replacement found no surviving receiver.
	// find similar wm
	for (u32 wm_idx = 0; wm_idx < wallmarks.size(); wm_idx++)
	{
		intrusive_ptr<CSkeletonWallmark>& wm = wallmarks[wm_idx];
		if (!wm->m_thermal_stamp.mark_key && wm->Similar(shader, cp, 0.02f))
		{
			if (wm_idx < wallmarks.size() - 1)
				wm = wallmarks.back();
			wallmarks.pop_back();
			break;
		}
	}

	bloodProject.Stop();
	wallmarks.push_back(wm);
    return true;
}

bool CKinematics::LL_AddSurfaceWallmark(const Fmatrix* xf,IWallMarkArray* array,const Fvector& p,const Fvector& n,float size,float ttl)
{return AddSurfaceWallmarkImpl(xf,array,p,n,size,ttl,nullptr);}
bool CKinematics::LL_AddBloodSurfaceWallmark(const Fmatrix* xf,IWallMarkArray* array,const Fvector& p,const Fvector& n,
    float size,float ttl,const BloodThermalStamp& stamp)
{return AddSurfaceWallmarkImpl(xf,array,p,n,size,ttl,&stamp);}
// 2026-11-12 sever splashes: the same placement with this decal's own depth limit and roll (see Kinematics.h).
bool CKinematics::LL_AddBloodSurfaceWallmarkPlaced(const Fmatrix* xf,IWallMarkArray* array,const Fvector& p,const Fvector& n,
    float size,float ttl,const BloodThermalStamp& stamp,float depth,float roll)
{
    const float limit=BloodProjectionFootprint::OuterDepthOverride(depth);
    return AddSurfaceWallmarkImpl(xf,array,p,n,size,ttl,&stamp,limit,_valid(roll)?&roll:nullptr);
}
bool CKinematics::AddSurfaceWallmarkImpl(const Fmatrix* transform,IWallMarkArray* array,
    const Fvector& contact,const Fvector& normal,float size,float ttl,const BloodThermalStamp* stamp,
    float outer_depth,const float* roll_deg)
{
    extern BOOL r_wallmarks_dynamic;
    if(!r_wallmarks_dynamic || !transform || !array || !_valid(contact) || !_valid(normal) ||
        !_valid(size) || !_valid(ttl) || normal.square_magnitude()<EPS_L*EPS_L || size<=EPS_L)return false;
    auto* material=static_cast<dxWallMarkArray*>(array);
    ref_shader* shader=material->dxGenerateWallmark();if(!shader)return false;
    if(size>1.f && !UsesBloodSquareFootprint(*shader))return false; // preserve non-blood admission
    Fvector direction;direction.set(normal).normalize_safe().mul(-1.f);
    // A pending GPU candidate is accepted, not a claim of visible pixels.
    return AddWallmark(transform,contact,direction,*shader,size,ttl,&normal,nullptr,stamp,outer_depth,roll_deg);
}

bool CKinematics::LL_AddSoakingWallmark(const Fmatrix* xf,IWallMarkArray* array,const Fvector& p,const Fvector& n,
    float size,float ttl,float opacity,float seconds)
{return AddSoakingWallmarkImpl(xf,array,p,n,size,ttl,opacity,seconds,nullptr);}
bool CKinematics::LL_AddBloodSoakingWallmark(const Fmatrix* xf,IWallMarkArray* array,const Fvector& p,const Fvector& n,
    float size,float ttl,float opacity,float seconds,const BloodThermalStamp& stamp)
{return AddSoakingWallmarkImpl(xf,array,p,n,size,ttl,opacity,seconds,&stamp);}
bool CKinematics::AddSoakingWallmarkImpl(const Fmatrix* transform,IWallMarkArray* array,
    const Fvector& contact,const Fvector& normal,float size,float ttl,float opacity,float growth_seconds,const BloodThermalStamp* stamp)
{
    extern BOOL r_wallmarks_dynamic;
    if (!r_wallmarks_dynamic || !transform || !array || !_valid(contact) || !_valid(normal) ||
        !_valid(size) || !_valid(ttl) || normal.square_magnitude()<EPS_L*EPS_L ||
        size<=EPS_L || ttl<1.f || ttl>600.f || !BloodWallmarkGrowth::Valid(opacity,growth_seconds)) return false;
    auto* material=static_cast<dxWallMarkArray*>(array);
    ref_shader* shader=material->dxGenerateWallmark(); if (!shader) return false;
    if(size>.3f && !UsesBloodSquareFootprint(*shader))return false; // preserve non-blood admission
    BloodWallmarkGrowth growth; growth.opacity=opacity; growth.seconds=growth_seconds; growth.initial_scale=.25f;
    Fvector direction; direction.set(normal).normalize_safe().mul(-1.f);
    return AddWallmark(transform,contact,direction,*shader,size,ttl,&normal,&growth,stamp);
}

struct zero_wm_pred
{
	bool operator()(const intrusive_ptr<CSkeletonWallmark> x) const { return x == 0; }
};

void CKinematics::CalculateWallmarks()
{
#ifdef USE_DX11
    RefreshBloodCells(); // immediate-context owner, before children enter the render graph
#endif
	PROF_EVENT("Calculate Wallmarks");
	xrCriticalSectionGuard guard(wallmarks_cs);
	if (!wallmarks.empty() && (wm_frame != RDEVICE.dwFrame))
	{
		wm_frame = RDEVICE.dwFrame;
		for (SkeletonWMVecIt it = wallmarks.begin(); it != wallmarks.end();)
		{
			intrusive_ptr<CSkeletonWallmark>& wm = *it;

			if (wm == 0)
			{
				it = wallmarks.erase(it);
				continue;
			}

			float w = wm->TimeEnd() == -1.f ? 0.f : (RDEVICE.fTimeGlobal - wm->TimeStart()) / wm->TimeEnd();
			if (w < 1.f)
			{
				// append wm to WallmarkEngine
				if (::Render->ViewBase.testSphere_dirty(wm->m_Bounds.P, wm->m_Bounds.R))
				{
					::RImplementation.add_SkeletonWallmark(std::move(wm));
					it = wallmarks.erase(it);
				}
				else
				{
					it++;
				}
				
			}
			else
			{
				// remove wallmark				
				it = wallmarks.erase(it);
			}
		}
	}
}

bool CKinematics::PrepareWallmarkRender(intrusive_ptr<CSkeletonWallmark> wm, WallmarkRenderPreparation& prepared)
{
	prepared = WallmarkRenderPreparation{};
	VERIFY(wm);
	VERIFY2(bones, "Invalid visual. Bones already released.");
	VERIFY2(bone_instances, "Invalid visual. bone_instances already deleted.");

	if ((wm == 0) || (0 == bones) || (0 == bone_instances)) return false;
	prepared.wallmark = wm.get();

	u32 color = 0;
	bool projected = false;
#ifdef USE_DX11
	projected = !!wm->m_Projection && wm->m_Faces.empty(); // R37: a carried mark renders its faces
#endif
	if (!wm->m_Faces.empty() || projected)
	{
		const float w = wm->TimeEnd() == -1.f ? 0.f : (RDEVICE.fTimeGlobal - wm->TimeStart()) / wm->TimeEnd();
		const float alpha = wm->m_Growth.opacity == 1.f ? w : wm->m_Growth.Fade(RDEVICE.fTimeGlobal-wm->TimeStart(),wm->TimeEnd());
		int aC = iFloor(alpha * 255.f);
		clamp(aC, 0, 255);
		color = BloodThermalWallmark::Color(RDEVICE.fTimeGlobal-wm->TimeStart(),aC,
            ps_r2_heatvision>0 && BloodThermalWallmark::Compatible(wm->Shader()),wm->m_thermal_stamp);
	}

	size_t face_count = wm->m_Faces.size();
#ifdef USE_DX11
	if (projected) face_count = wm->m_Projection->candidate_vertices / 3;
#endif
	bool use_cache = false;
	bool cache_current = false;
	size_t vertex_count = 0;
	if (face_count && face_count <= (std::numeric_limits<u32>::max)() / 3)
	{
		vertex_count = face_count * size_t(3);
		try
		{
			const u16 bone_count = LL_BoneCount();
			const bool rebuild = wm->m_RenderFaces != wm->m_Faces.data() ||
				wm->m_RenderFaceCount != face_count ||
				wm->m_RenderBoneInstances != bone_instances || wm->m_RenderBoneCount != bone_count;
			auto prepare_cache = [&]()
			{
				if (!rebuild) return true;
				wm->InvalidateRenderCache();
				// Snapshot the small skeleton directly. Discovering used bones by
				// scanning every decal triangle made newly sprayed marks expensive.
				wm->m_RenderBones.resize(bone_count);
				for (u16 id = 0; id < bone_count; ++id)
				{
					wm->m_RenderBones[id].id = id;
					wm->m_RenderBones[id].transform.identity();
				}
				wm->m_RenderFaces = wm->m_Faces.data();
				wm->m_RenderFaceCount = face_count;
				wm->m_RenderBoneInstances = bone_instances;
				wm->m_RenderBoneCount = bone_count;
				return true;
			};
			if (prepare_cache())
			{
				// Rendering already holds the wallmark-engine lock. Never wait for
				// a bone worker/callback here: contention keeps the original path.
				xrCriticalSectionTryGuard pose_guard(UCalc_Mutex);
				if (pose_guard.owns_lock())
				{
					const Fmatrix world = *wm->XFORM();
					const float now = RDEVICE.fTimeGlobal;
					bool changed = !wm->m_RenderPoseValid || now < wm->m_RenderPoseSince ||
						std::memcmp(&world, &wm->m_RenderWorld, sizeof(Fmatrix)) != 0;
#ifdef USE_DX11
					{ u32 dents = 0; u64 key = 0; LL_RuntimeDentTexels(dents, &key); // melee (value 3): a new dent moves the decal (re-skin / re-project)
					  if (key != wm->m_RenderDentKey) { wm->m_RenderDentKey = key; changed = true; } }
#endif
					for (auto& bone : wm->m_RenderBones)
					{
						const Fmatrix& current = LL_GetBoneInstance(bone.id).mRenderTransform;
						if (wm->m_RenderPoseValid && std::memcmp(&current, &bone.transform, sizeof(Fmatrix)) != 0)
							changed = true;
						bone.transform = current;
					}
					wm->m_RenderWorld = world;
					if (changed)
					{
						wm->m_RenderPoseSince = now;
						wm->m_RenderCacheValid = false;
					}
					wm->m_RenderPoseValid = true;
					prepared.pose_snapshot_valid = true;
					prepared.pose_changed = changed;
					cache_current = wm->m_RenderCacheValid && !changed;
					// Avoid an extra RAM write/copy on continuously animated marks.
					// Only stable poses qualify; blood still renders during warmup.
					use_cache = !projected && (cache_current || (!changed && now - wm->m_RenderPoseSince >= 0.1f));
				}
				else
				{
					wm->m_RenderPoseValid = false;
				}
			} // Release the pose lock before skinning or streaming any vertices.
			if (use_cache && !cache_current)
			{
				// Allocate vertices only after stability is established, never on
				// the initial hit. Ownership/lifetime remain local to this mark.
				if (vertex_count > wm->m_RenderVertices.max_size()) use_cache = false;
				else wm->m_RenderVertices.resize(vertex_count);
			}
		}
		catch (...)
		{
			// A cache allocation failure must not remove a mark that the original
			// renderer could still draw. The caller retains its original guards.
			use_cache = false;
		}
	}

	prepared.vertex_count = vertex_count;
	prepared.color = color;
	if (wm->m_Growth.Growing(RDEVICE.fTimeGlobal-wm->TimeStart())) { use_cache=false; cache_current=false; }
	prepared.use_cpu_cache = use_cache;
	prepared.cache_current = cache_current;
	return true;
}

bool CKinematics::ExportWallmarkGpuSource(intrusive_ptr<CSkeletonWallmark> wm,
	const WallmarkRenderPreparation& prepared, BloodGpuSkinning::InputVertex* vertices, size_t capacity) const
{
	if (!wm || prepared.wallmark != wm.get() || !prepared.pose_snapshot_valid || !vertices ||
		prepared.vertex_count == 0 || prepared.vertex_count % 3 != 0 || prepared.vertex_count > capacity ||
		wm->m_Faces.size() != prepared.vertex_count / 3 ||
		wm->m_RenderBones.size() != wm->m_RenderBoneCount || wm->m_RenderBoneCount == 0)
		return false;

	size_t index = 0;
	for (const auto& face : wm->m_Faces)
	{
		for (u32 k = 0; k < 3; ++k)
		{
			const u16* const ids = face.bone_id[k];
			const u32 links = ids[0] == ids[1] ? 1 : ids[1] == ids[2] ? 2 : ids[2] == ids[3] ? 3 : 4;
			for (u32 i = 0; i < links; ++i)
				if (ids[i] >= wm->m_RenderBoneCount) return false;

			auto& out = vertices[index++];
			out = BloodGpuSkinning::InputVertex{};
			out.p[0] = face.vert[k].x; out.p[1] = face.vert[k].y; out.p[2] = face.vert[k].z;
			out.uv[0] = face.uv[k].x; out.uv[1] = face.uv[k].y;
#ifdef USE_DX11
            out.ownership[0]=face.ownership_range[0];out.ownership[1]=face.ownership_range[1];
#endif
			// Never read unused/uninitialized face slots. Repeating the final used
			// ID preserves the exact CPU branch; unused shader weights stay zero.
			for (u32 i = 0; i < 4; ++i) out.ids[i] = ids[i < links ? i : links - 1];
			for (u32 i = 0; i + 1 < links; ++i) out.weights[i] = face.weight[k][i];
		}
	}
	return true;
}

bool CKinematics::ExportWallmarkGpuFaces(const CSkeletonWallmark& wm, BloodGpuSkinning::InputVertex* vertices, size_t capacity, size_t& exported) const
{
	exported = 0;
	const u16 bone_count = LL_BoneCount();
	const size_t vertex_count = wm.m_Faces.size() * size_t(3);
	if (!vertices || !bone_count || wm.m_Faces.empty() || vertex_count > capacity) return false;
	size_t index = 0;
	for (const auto& face : wm.m_Faces)
	{
		bool hidden = false;
		for (u32 k = 0; k < 3 && !hidden; ++k)
		{
			const u16* const ids = face.bone_id[k];
			const u32 links = ids[0] == ids[1] ? 1 : ids[1] == ids[2] ? 2 : ids[2] == ids[3] ? 3 : 4;
			for (u32 i = 0; i < links; ++i)
			{
				if (ids[i] >= bone_count) return false;
				if (!LL_GetBoneVisible(ids[i])) hidden = true;
			}
		}
		if (hidden) continue;
		for (u32 k = 0; k < 3; ++k)
		{
			const u16* const ids = face.bone_id[k];
			const u32 links = ids[0] == ids[1] ? 1 : ids[1] == ids[2] ? 2 : ids[2] == ids[3] ? 3 : 4;
			auto& out = vertices[index++];
			out = BloodGpuSkinning::InputVertex{};
			out.p[0] = face.vert[k].x; out.p[1] = face.vert[k].y; out.p[2] = face.vert[k].z;
			out.uv[0] = face.uv[k].x; out.uv[1] = face.uv[k].y;
#ifdef USE_DX11
            out.ownership[0]=face.ownership_range[0];out.ownership[1]=face.ownership_range[1];
#endif
			for (u32 i = 0; i < 4; ++i) out.ids[i] = ids[i < links ? i : links - 1];
			for (u32 i = 0; i + 1 < links; ++i) out.weights[i] = face.weight[k][i];
		}
	}
	exported = index;
	return true;
}

bool CKinematics::CaptureWallmarkPose(u32 frame, const Fmatrix& world)
{
	if (m_WallmarkPoseFrame == frame) return m_WallmarkPoseValid;
	m_WallmarkPoseFrame = frame;
	const u16 bone_count = (bones && bone_instances) ? LL_BoneCount() : u16(0);
	if (!bone_count) { m_WallmarkPoseValid = false; return false; }
	// Never wait for a bone worker here (the renderer holds the wallmark lock): while the pose is being written,
	// the previous frame's consistent snapshot serves one more frame.
	xrCriticalSectionTryGuard pose_guard(UCalc_Mutex);
	if (!pose_guard.owns_lock()) return m_WallmarkPoseValid;
	try { m_WallmarkPose.resize(bone_count); }
	catch (...) { m_WallmarkPoseValid = false; return false; }
	for (u16 id = 0; id < bone_count; ++id) m_WallmarkPose[id] = LL_GetBoneInstance(id).mRenderTransform;
	m_WallmarkPoseWorld = world;
	m_WallmarkPoseValid = true;
	return true;
}

void CKinematics::RenderWallmark(intrusive_ptr<CSkeletonWallmark> wm, FVF::LIT* & V)
{
	PROF_EVENT("CKinematics::RenderWallmark");
	VERIFY(V);
#ifdef USE_DX11
	if (wm && wm->m_Faces.empty() && !wm->ResolveProjection()) return; // R37: faces present = draw them
#endif
	WallmarkRenderPreparation prepared;
	if (PrepareWallmarkRender(wm, prepared)) RenderWallmarkPrepared(wm, V, prepared);
}

void CKinematics::RenderWallmarkPrepared(intrusive_ptr<CSkeletonWallmark> wm, FVF::LIT*& V,
	const WallmarkRenderPreparation& prepared)
{
	VERIFY(V);
	// The renderer consumes this result immediately, including backend failures.
	// An unrelated/default result must never borrow another mark's pose storage.
	if (!wm || prepared.wallmark != wm.get())
	{
		RenderWallmark(wm, V);
		return;
	}
	const size_t vertex_count = prepared.vertex_count;
	const u32 color = prepared.color;
	const bool use_cache = prepared.use_cpu_cache;
	const bool cache_current = prepared.cache_current;

	if (use_cache && cache_current)
	{
		if (wm->m_RenderColor != color)
		{
			for (auto& vertex : wm->m_RenderVertices) vertex.color = color;
			wm->m_RenderColor = color;
		}
		std::memcpy(V, wm->m_RenderVertices.data(), vertex_count * sizeof(FVF::LIT));
		V += vertex_count;
		wm->XFORM()->transform_tiny(wm->m_Bounds.P, wm->m_LocalBounds.P);
		return;
	}

	wm->m_RenderCacheValid = false;
	FVF::LIT* const destination = V;
	if (use_cache) V = wm->m_RenderVertices.data();
#ifdef USE_DX11
	u32 dent_count = 0; const std::shared_ptr<const float> dent_texels = LL_RuntimeDentTexels(dent_count); // melee (value 3): the decal on the body's dents
#endif
	// Dispatch once per mark so the moving-pose loop has no per-link cache test.
	auto skin = [&](auto cache_mode)
	{
		constexpr bool cached = decltype(cache_mode)::value;
		auto render_matrix = [&](u16 id) -> Fmatrix&
		{
			return cached ? wm->m_RenderBones[id].transform :
				LL_GetBoneInstance(id).mRenderTransform;
		};
		const Fmatrix* const world = cached ? &wm->m_RenderWorld : wm->XFORM();

		// Preserve the skinning arithmetic and vertex order. Cache misses use the
		// exact snapshots that form their key, never freshly changing live matrices.
		for (u32 f_idx = 0; f_idx < wm->m_Faces.size(); f_idx++)
		{
			const CSkeletonWallmark::WMFace& F = wm->m_Faces[f_idx];
#ifdef USE_DX11
			Fvector dented_vert[3]; const Fvector* FV = F.vert; // melee (value 3): the face's corners on the body's dents (rest space)
			if (dent_texels) { for (u32 c = 0; c < 3; ++c) { dented_vert[c] = F.vert[c];
				painter_dent::apply(dent_texels.get(), dent_count, dented_vert[c].x, dented_vert[c].y, dented_vert[c].z); } FV = dented_vert; }
#else
			const Fvector* const FV = F.vert;
#endif
			for (u32 k = 0; k < 3; k++)
			{
				Fvector P;
				if (F.bone_id[k][0] == F.bone_id[k][1])
				{
					// 1-link
					Fmatrix& xform0 = render_matrix(F.bone_id[k][0]);
					xform0.transform_tiny(P, FV[k]);
				}
				else if (F.bone_id[k][1] == F.bone_id[k][2])
				{
					// 2-link
					Fvector P0, P1;
					Fmatrix& xform0 = render_matrix(F.bone_id[k][0]);
					Fmatrix& xform1 = render_matrix(F.bone_id[k][1]);
					xform0.transform_tiny(P0, FV[k]);
					xform1.transform_tiny(P1, FV[k]);
					P.lerp(P0, P1, F.weight[k][0]);
				}
				else if (F.bone_id[k][2] == F.bone_id[k][3])
				{
					// 3-link
					Fvector P0, P1, P2;
					Fmatrix& xform0 = render_matrix(F.bone_id[k][0]);
					Fmatrix& xform1 = render_matrix(F.bone_id[k][1]);
					Fmatrix& xform2 = render_matrix(F.bone_id[k][2]);
					xform0.transform_tiny(P0, FV[k]);
					xform1.transform_tiny(P1, FV[k]);
					xform2.transform_tiny(P2, FV[k]);
					float w0 = F.weight[k][0];
					float w1 = F.weight[k][1];
					P0.mul(w0);
					P1.mul(w1);
					P2.mul(1 - w0 - w1);
					P = P0;
					P.add(P1);
					P.add(P2);
				}
				else
				{
					// 4-link
					Fvector PB[4];
					for (int i = 0; i < 4; ++i)
					{
						Fmatrix& xform = render_matrix(F.bone_id[k][i]);
						xform.transform_tiny(PB[i], FV[k]);
					}

					float s = 0.f;
					for (int i = 0; i < 3; ++i)
					{
						PB[i].mul(F.weight[k][i]);
						s += F.weight[k][i];
					}
					PB[3].mul(1 - s);

					P = PB[0];
					for (int i = 1; i < 4; ++i)
						P.add(PB[i]);
				}
				world->transform_tiny(V->p, P);
				V->t.set(F.uv[k]);
                if (wm->m_Growth.Growing(RDEVICE.fTimeGlobal-wm->TimeStart())) {
                    const float age=RDEVICE.fTimeGlobal-wm->TimeStart();
                    V->t.set(wm->m_Growth.UV(F.uv[k].x,age),wm->m_Growth.UV(F.uv[k].y,age));
                }
				V->color = color;
				V++;
			}
			{
				FVF::LIT* const tri = V - 3;
				bool broken = false;
				// R26: a vertex on a hidden bone (a part cut off) would sit at its parent's joint; the face is not drawn.
				for (u32 k = 0; k < 3 && !broken; ++k)
				{
					const u16* const ids = F.bone_id[k];
					const u32 links = ids[0] == ids[1] ? 1 : ids[1] == ids[2] ? 2 : ids[2] == ids[3] ? 3 : 4;
					for (u32 i = 0; i < links && !broken; ++i) if (!LL_GetBoneVisible(ids[i])) broken = true;
				}
				for (u32 e = 0; e < 3 && !broken; ++e)
				{
					const u32 n = (e + 1) % 3;
					if (tri[e].p.distance_to(tri[n].p) > FV[e].distance_to(FV[n]) * 3.f + 0.05f) broken = true;
				}
				if (broken) { tri[1].p = tri[0].p; tri[2].p = tri[0].p; }
			}
		}
	};
	if (use_cache) skin(std::true_type{});
	else skin(std::false_type{});
	if (use_cache)
	{
		// Only read ordinary CPU RAM; mapped dynamic GPU memory is write-only.
		std::memcpy(destination, wm->m_RenderVertices.data(), vertex_count * sizeof(FVF::LIT));
		V = destination + vertex_count;
		wm->m_RenderColor = color;
	}
	wm->XFORM()->transform_tiny(wm->m_Bounds.P, wm->m_LocalBounds.P);
	wm->m_RenderCacheValid = use_cache;
}

void CKinematics::ClearWallmarks()
{
#ifndef _EDITOR
	// The wallmarks engine keeps its own list of skeleton wallmarks that point back at this
	// model (and at its owner's XFORM). Clearing only our list left those entries behind, so
	// a model returned to the pool (Depart) or deleted rendered through dangling pointers on
	// the next frame (crash in CKinematics::RenderWallmark). Take them out of the engine first;
	// the engine lock is released again before wallmarks_cs is taken, so the two are never held together here.
	if (::Render)
		::Render->remove_SkeletonWallmarksFromObject(this);
#endif
	xrCriticalSectionGuard guard(wallmarks_cs);
	//	for (SkeletonWMVecIt it=wallmarks.begin(); it!=wallmarks.end(); it++)
	//		xr_delete	(*it);
	wallmarks.clear();
}

u64 CKinematics::ScopeGlassBoneSubtree(u16 bone)
{
    if (bone >= bones->size()) return 0;
    u64 mask = 0;
    for (u16 id = 0; id < bones->size(); ++id)
    {
        u16 ancestor = id;
        while (ancestor != bone && ancestor != BI_NONE && ancestor < bones->size())
            ancestor = (*bones)[ancestor]->GetParentID();
        if (ancestor == bone) mask |= u64(1) << id;
    }
    return mask;
}

bool CKinematics::IsPhysicalScopeGlassBone(u16 bone)
{
#if RENDER == R_R4
    const u64 subtree = ScopeGlassBoneSubtree(bone);
    if (!subtree) return false;
    bool found = false;
    // Child arrays reorder as visibility changes; inspect each mesh's actual
    // bone set instead of indexing the original child_faces by current order.
    for (const auto* list : { &children, &children_invisible })
        for (auto* visual : *list)
        {
            auto* mesh = smart_cast<CSkeletonX*>(visual);
            if (!mesh) continue;
            const u64 used = mesh->ScopeGlassBoneMask();
            if (!(used & subtree)) continue;
            if (mesh->IsPhysicalScopeGlass()) found = true;
            // An opaque mesh shared with unrelated bones cannot be removed
            // as a whole. Keep native behavior for that unsupported layout.
            else if (used & ~subtree) return false;
        }
    return found;
#else
    return false;
#endif
}

void CKinematics::SetPhysicalScopeGlassMask(u16 bone, bool enabled)
{
    PhysicalScopeGlassMask = enabled && IsPhysicalScopeGlassBone(bone)
        ? ScopeGlassBoneSubtree(bone) : 0;
}

int CKinematics::LL_GetBoneGroups(xr_vector<xr_vector<u16>>& groups)
{
	groups.resize(children.size());
	for (u16 bone_idx = 0; bone_idx < (u16)bones->size(); bone_idx++)
	{
		CBoneData* B = (*bones)[bone_idx];
		for (u32 child_idx = 0; child_idx < children.size(); child_idx++)
		{
			if (!GeometryFaces(bone_idx, LL_GetChild(child_idx)->GeometryChildIndex()).empty())
			{
				groups[child_idx].push_back(bone_idx);
			}
		}
	}
	return groups.size();
}

#ifdef DEBUG
CSkeletonWallmark::~CSkeletonWallmark()
{
		if(used_in_render!=u32(-1))
		{
			Msg		("used_in_render=%d",used_in_render);
			VERIFY	(used_in_render==u32(-1));
		}
}
#endif

#include "PainterBloodBake_impl.h"
#include "SkeletonRuntimeGeometry_impl.h"

#ifdef USE_DX11
std::shared_ptr<PainterRemovalGpu> CKinematics::LL_RuntimeRemovalGpu() const {
    return m_runtime_geometry && m_runtime_geometry->required_fields ?
        m_runtime_geometry->required_fields->removal : std::shared_ptr<PainterRemovalGpu>{};
}
#endif


