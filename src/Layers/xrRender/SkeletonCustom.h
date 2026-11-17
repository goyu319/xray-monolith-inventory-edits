//---------------------------------------------------------------------------
#ifndef SkeletonCustomH
#define SkeletonCustomH

#include		"fhierrarhyvisual.h"
#include		"../../xrEngine/bone.h"
#include		"../../Include/xrRender/Kinematics.h"
#include "../../xrEngine/IRenderable.h"
#include <limits>
#include <array>
#include <new>
#include <optional>
#include "../../Include/xrRender/BloodWallmarkGrowth.h"
#ifdef USE_DX11
#include "BloodGpuProjection.h"
#include "BloodProjectionBounds.h"
#include <atomic>
#include <memory>
#include <vector>
extern std::atomic<u64> g_blood_projection_epoch;
struct BloodProjectionCpuCounters
{
	std::atomic<u64> cache_attempts{0}, cache_built{0}, cache_missing{0}, source_cold{0}, hit_snapshots{0};
	std::atomic<u64> cache_empty{0}, cache_invalid{0}, cache_child_budget{0}, cache_total_budget{0}, cache_alloc_failed{0};
};
extern BloodProjectionCpuCounters g_blood_projection_cpu;
#endif

// refs
class CKinematics;
class CInifile;
class CBoneData;
struct SEnumVerticesCallback;
namespace BloodGpuSkinning { struct InputVertex; }

class IRenderable;
#ifdef USE_DX11
struct PainterBloodInstance;
struct PainterRemovalGpu;
struct PainterSkinOwnershipGpu;
struct PainterWallmarkOwnershipGpu;
struct PainterWallmarkTear; // maskdecal
namespace PainterWallmarkOwnership { struct Field; using TriangleKey=std::array<std::uint32_t,33>; }
#endif

#pragma warning(push)
#pragma warning(disable:4275)
#include <atomic>
inline std::atomic<unsigned long long> g_painter_marks_carried{0}; // R37: decals that kept their faces through a publication
class CSkeletonWallmark : public intrusive_base // 4+4+4+12+4+16+16 = 60 + 4 = 64 + 4 = 68
{
#pragma warning(pop)
	CKinematics* m_Parent; // 4
	const Fmatrix* m_XForm; // 4
	ref_shader m_Shader; // 4
	Fvector3 m_ContactPoint; // 12		model space
	float m_fTimeStart; // 4
	float m_fTimeEnd; // 4
public:
#ifdef DEBUG
	u32					used_in_render;	
#endif
	BloodWallmarkGrowth m_Growth;
    BloodThermalStamp m_thermal_stamp;
    std::shared_ptr<BloodWallmarkGrowth::Lease> m_GrowthLease;
	bool m_SquareFootprint = false; // paired effects_wallmark_blood pixel clipping
	float m_OuterDepth = 0.f; // 0: BloodProjectionFootprint::OuterDepth(size); else the surface call's own depth limit (m)
	Fsphere m_LocalBounds; // 16		model space
	struct WMFace
	{
		Fvector3 vert [3];
		Fvector2 uv [3];
		u16 bone_id[3][4];
		float weight[3][3];
#ifdef USE_DX11
        u32 source_child=~0u,source_face=~0u;
        const PainterWallmarkOwnership::TriangleKey* source_key=nullptr;
        std::shared_ptr<const PainterWallmarkOwnership::Field> provenance_keeper;
        std::shared_ptr<PainterWallmarkOwnershipGpu> ownership;
        std::array<u32,2> ownership_range{};
#endif
		static void LinksFrom4W(u16 ids[4], float weights[3])
		{
			const float in[4] = { weights[0], weights[1], weights[2], 1.f - weights[0] - weights[1] - weights[2] };
			u16 bone[4]; float w[4]; u32 used = 0;
			for (u32 q = 0; q < 4; ++q)
			{
				if (!(_abs(in[q]) >= 1.f / 510.f)) continue; // R28: NaN or below half a UNORM8 step; a negative weight is a real influence (gfy_signed_hw)
				u32 s = 0;
				while (s < used && bone[s] != ids[q]) ++s;
				if (s == used) { bone[used] = ids[q]; w[used] = 0.f; ++used; }
				w[s] += in[q];
			}
			if (used == 0 || used == 4) return;
			weights[0] = weights[1] = weights[2] = 0.f;
			if (used == 1) { ids[0] = ids[1] = ids[2] = ids[3] = bone[0]; return; }
			if (used == 2) { ids[0] = bone[0]; ids[1] = ids[2] = ids[3] = bone[1]; weights[0] = w[1]; return; }
			ids[0] = bone[0]; ids[1] = bone[1]; ids[2] = ids[3] = bone[2]; weights[0] = w[0]; weights[1] = w[1];
		}
	};

	// Faces are immutable once the mark is published. Any future in-place
	// geometry editor must invalidate this cache before changing them.
	struct CachedBone
	{
		u16 id;
		Fmatrix transform;
	};
	// The engine allocator may return null. Optional cache storage must instead
	// throw before a container touches that pointer, so rendering can fall back.
	template <class T>
	class CacheAllocator : public xalloc<T>
	{
	public:
		template <class U> struct rebind { using other = CacheAllocator<U>; };
		CacheAllocator() noexcept = default;
		template <class U> CacheAllocator(const CacheAllocator<U>&) noexcept {}

		size_t max_size() const noexcept
		{
			const size_t count_limit = (std::numeric_limits<u32>::max)();
			const size_t byte_limit = (std::numeric_limits<size_t>::max)() / sizeof(T);
			return count_limit < byte_limit ? count_limit : byte_limit;
		}
		T* allocate(size_t count, const void* = nullptr) const
		{
			if (count > max_size()) throw std::bad_alloc();
			T* const result = xr_alloc<T>(static_cast<u32>(count));
			if (!result) throw std::bad_alloc();
			return result;
		}
	};
	using WMFacesVec = xr_vector<WMFace, CacheAllocator<WMFace>>;
	using WMFacesVecIt = WMFacesVec::iterator;
	WMFacesVec m_Faces;
	template<class T> using ProjectionVector = std::vector<T, CacheAllocator<T>>;

#ifdef USE_DX11
	// Immutable mesh data, shared by model copies and all of their hit snapshots.
	// Construction happens at model load, never on the hot hit/render paths.
	struct ProjectionSource
	{
		// Ranges address the logical triangle index stream. Referenced original
		// vertices are stored once; repeated faces across bones remain in indices.
		struct Range { u32 first_vertex = 0, vertex_count = 0; };
		ProjectionVector<BloodGpuProjection::InputVertex> vertices;
		ProjectionVector<u32> indices;
		ProjectionVector<Range> bones;
        ProjectionVector<u32> source_faces; // native primitive per logical triangle, before bone/range ordering
        ProjectionVector<u32> original_faces;
        u32 source_child=~0u,original_child=~0u;
        // Exact prepared-geometry/rig/provenance cache identity; independent of ownership fields.
        std::shared_ptr<const void> footprint_identity;
        std::shared_ptr<PainterWallmarkOwnershipGpu> ownership;
        bool ownership_required=false;
        bool BindOwnership(const std::shared_ptr<PainterWallmarkOwnershipGpu>&field,u32 child,u32 original=~0u,const std::vector<u32>*provenance=nullptr);
        ProjectionVector<BloodProjectionBounds::Block> projection_bounds;
        void BuildProjectionBounds() noexcept;
		// Readiness is separate from immutable source geometry. Cold hits keep
		// immediate CPU projection instead of deferring the same CPU burst.
		mutable std::atomic<u64> gpu_epoch{0};
		size_t reserved_bytes = 0;
		bool ReserveBudget(size_t bytes) noexcept;
		static size_t ReservedBytes() noexcept;
		static size_t PeakBytes() noexcept;
		static void Publish(const std::shared_ptr<const ProjectionSource>& source);
		static void TakePending(ProjectionVector<std::weak_ptr<const ProjectionSource>>& pending);
		~ProjectionSource();
	};
	struct ProjectionSnapshot
	{
		struct Range
		{
			std::shared_ptr<const ProjectionSource> source;
			u32 first_vertex = 0, vertex_count = 0;
		};
		ProjectionVector<Range> ranges;
		ProjectionVector<Fmatrix> hit_pose;
		U16Vec selected_bones;
		Fmatrix view, world;
		Fvector normal;
		float size = 0.f;
		float outer_depth = 0.f; // 0: BloodProjectionFootprint::OuterDepth(size); else the surface call's own limit (m)
		bool square_footprint = false;
		u32 candidate_vertices = 0;
		u64 epoch = 0;
	};
	std::shared_ptr<const ProjectionSnapshot> m_Projection;
    // Source-free placement retained after CPU/GPU projection. Original hit
    // pose, UV rotation and lifetime survive topology changes without rerolling.
    std::shared_ptr<const ProjectionSnapshot> m_Reprojection;
    // The last projection's ranges, kept after a CPU resolve so a publish can carry them by source instead of
    // selecting the footprint again on every receiver.
    std::shared_ptr<const ProjectionSnapshot> m_CarryRanges;
	// Materializes legacy faces transactionally using captured hit inputs only.
	// No live parent, source mesh, or current bone matrices are consulted.
	// R37: resumable. budget = candidate vertices this call may visit (nullptr: all of them); false = not finished
	// yet (the walk continues next call) or failed (the snapshot is kept for a retry).
	bool ResolveProjection(u32* budget = nullptr);
    static void AssignOwnership(WMFace&face,const std::shared_ptr<PainterWallmarkOwnershipGpu>&field,u32 child,u32 triangle);
    void ApplyCarryOwnership(const std::shared_ptr<PainterWallmarkOwnershipGpu>&field)noexcept;
    bool FaceOwnership(std::shared_ptr<PainterWallmarkOwnershipGpu>&field)const noexcept;
    bool ProjectionOwnership(std::shared_ptr<PainterWallmarkOwnershipGpu>&field)const noexcept;
	WMFacesVec m_ResolveFaces; u32 m_ResolveRange = 0, m_ResolveVertex = 0; bool m_ResolveActive = false; const void* m_ResolveSnapshot = nullptr;
	// R37: the mark this one replaces at the next publication commit; its resolved faces are taken over there. The
	// keeper owns a reference to it (the class is incomplete here, so the reference is type-erased).
	CSkeletonWallmark* m_CarryFrom = nullptr; std::shared_ptr<void> m_CarryKeeper;
#endif
	xr_vector<CachedBone, CacheAllocator<CachedBone>> m_RenderBones;
	xr_vector<FVF::LIT, CacheAllocator<FVF::LIT>> m_RenderVertices;
	const WMFace* m_RenderFaces = nullptr;
	size_t m_RenderFaceCount = 0;
	const CBoneInstance* m_RenderBoneInstances = nullptr;
	u16 m_RenderBoneCount = 0;
	Fmatrix m_RenderWorld;
	u32 m_RenderColor = 0;
	float m_RenderPoseSince = 0.f;
	bool m_RenderPoseValid = false;
	bool m_RenderCacheValid = false;
	u64 m_RenderGeometryRevision = 0;
	u64 m_RenderDentKey = 0; // melee (value 3): the body's dents this mark was last skinned on (a new dent re-skins it)
	void InvalidateRenderCache()
	{
		++m_RenderGeometryRevision;
		m_RenderFaces = nullptr;
		m_RenderFaceCount = 0;
		m_RenderPoseValid = false;
		m_RenderCacheValid = false;
	}
public:
	Fsphere m_Bounds; // 16		world space
public:
	CSkeletonWallmark(CKinematics* p, const Fmatrix* m, ref_shader s, const Fvector& cp, float ts, float te):
		m_Parent(p), m_XForm(m), m_Shader(s), m_fTimeStart(ts), m_ContactPoint(cp), m_fTimeEnd(te)
	{
#ifdef DEBUG
						used_in_render = u32(-1);
#endif
	}

	~CSkeletonWallmark()
#ifdef DEBUG
							;
#else
	{
	}
#endif

	IC CKinematics* Parent() { return m_Parent; }
	IC u32 VCount() { return m_Faces.size() * 3; }
	IC bool Similar(ref_shader& sh, const Fvector& cp, float eps)
	{
		return (m_Shader == sh) && m_ContactPoint.similar(cp, eps);
	}

	IC float TimeStart() { return m_fTimeStart; }
	IC float TimeEnd() { return m_fTimeEnd; }
	IC const Fmatrix* XFORM() { return m_XForm; }
	IC const Fvector3& ContactPoint() { return m_ContactPoint; }
	// R20: the removal field (painter_detail::DamageField) this mark was last tested against. A mark whose skin is
	// entirely carved away is expired instead of being drawn through the mask forever.
	const void* m_RemovalChecked = nullptr;
	IC void Expire() { m_fTimeEnd = 0.f; m_fTimeStart = -1.f; }
	IC ref_shader Shader() { return m_Shader; }
};

DEFINE_VECTOR(intrusive_ptr<CSkeletonWallmark>, SkeletonWMVec, SkeletonWMVecIt);

// sanity check
#ifdef DEBUG
	struct dbg_marker{
		BOOL*			lock;
		dbg_marker		(BOOL* b)	{lock=b; VERIFY(*lock==FALSE); *lock=TRUE;}
		~dbg_marker		()			{*lock=FALSE;}
	};
#	define _DBG_SINGLE_USE_MARKER	dbg_marker	_dbg_marker(&dbg_single_use_marker)
#else
#	define _DBG_SINGLE_USE_MARKER
#endif

class CKinematics : public FHierrarhyVisual, public IKinematics
{
	typedef FHierrarhyVisual inherited;
	friend class CBoneData;
	friend class CSkeletonX;
public:
#ifdef DEBUG
	BOOL						dbg_single_use_marker;
#endif
	void Bone_Calculate(CBoneData* bd, Fmatrix* parent);
	void CLBone(const CBoneData* bd, CBoneInstance& bi, const Fmatrix* parent, u8 mask_channel = (1 << 0));

	void BoneChain_Calculate(const CBoneData* bd, CBoneInstance& bi, u8 channel_mask, bool ignore_callbacks);
	void Bone_GetAnimPos(Fmatrix& pos, u16 id, u8 channel_mask, bool ignore_callbacks);


	virtual void BuildBoneMatrix(const CBoneData* bd, CBoneInstance& bi, const Fmatrix* parent,
	                             u8 mask_channel = (1 << 0));

	virtual void OnCalculateBones()
	{
	}

public:
	dxRender_Visual* m_lod;

#ifdef OPTIMIZE_CALCULATE_BONES
	IC bool canBeOptimized()
	{
		return spatialParent && spatialParent->canOptimizeCalculateBones;
	}
#endif

protected:
	SkeletonWMVec wallmarks;
	u32 wm_frame;
	u32 CurrentFrame;
	Fmatrix	Matrix_Prev[2]; // per viewport (main, SVP)
	Fmatrix Matrix_Temp[2];

	// Instance-owned render geometry. Shared source CBoneData stays untouched.
	CKinematics* m_geometry_donor = nullptr;
	struct RuntimeGeometryPacket* m_runtime_geometry = nullptr;
	struct RuntimeGeometryPrepared* m_prepared_runtime_geometry = nullptr;
	u64 m_render_geometry_generation = 0;
    std::shared_ptr<const painter_skin_proxy::Source> m_original_skin_proxy_source;
	xr_vector<IRenderVisual*> m_geometry_original_children;
	xr_vector<IRenderVisual*> m_geometry_original_invisible;
	dxRender_Visual* m_geometry_original_lod = nullptr;

	// Globals
	CInifile* pUserData;
	CBoneInstance* bone_instances; // bone instances
	vecBones* bones; // all bones	(shared)
	u16 bones_size;
	u16 iRoot; // Root bone index

	// Fast search
	accel* bone_map_N; // bones  associations	(shared)	- sorted by name
	accel* bone_map_P; // bones  associations	(shared)	- sorted by name-pointer

	BOOL Update_Visibility;
	u32 UCalc_Time;
	s32 UCalc_Visibox;
	u32 Visibox_frame = 0;
	bool UCalc_ThisFrame;

	Flags64 visimask;
	Flags64 hidden_bones;

	CSkeletonX* LL_GetChild(u32 idx);

	// internal functions
	virtual CBoneData* CreateBoneData(u16 ID) { return xr_new<CBoneData>(ID); }
	virtual void IBoneInstances_Create();
	virtual void IBoneInstances_Destroy();
	void Visibility_Invalidate() { Update_Visibility = TRUE; };
	void Visibility_Update();

	//--DSR-- SilencerOverheat_start
	virtual IRenderVisual* GetVisualByBone(u16 bone_id);
	virtual IRenderVisual* GetVisualByBone(LPCSTR bone_name);
	//--DSR-- SilencerOverheat_end

	void LL_Validate();
public:
	std::shared_ptr<const painter_skin_proxy::Source> LL_OriginalSkinProxySource() const override { return m_original_skin_proxy_source; }
	UpdateCallback Update_Callback;
	void* Update_Callback_Param;
public:
	// wallmarks
	bool AddWallmark(const Fmatrix* parent, const Fvector3& start, const Fvector3& dir, ref_shader shader, float size, float ttl = 0.f, const Fvector* surface_normal = nullptr, const BloodWallmarkGrowth* growth = nullptr,const BloodThermalStamp* thermal_stamp = nullptr,
        float outer_depth = 0.f,const float* roll_deg = nullptr);
    bool LL_AddBloodSurfaceWallmark(const Fmatrix*,IWallMarkArray*,const Fvector&,const Fvector&,float,float,const BloodThermalStamp&) override;
    bool LL_AddBloodSurfaceWallmarkPlaced(const Fmatrix*,IWallMarkArray*,const Fvector&,const Fvector&,float,float,const BloodThermalStamp&,float,float) override;
    bool LL_AddBloodSoakingWallmark(const Fmatrix*,IWallMarkArray*,const Fvector&,const Fvector&,float,float,float,float,const BloodThermalStamp&) override;
    bool AddSurfaceWallmarkImpl(const Fmatrix*,IWallMarkArray*,const Fvector&,const Fvector&,float,float,const BloodThermalStamp*,
        float outer_depth = 0.f,const float* roll_deg = nullptr);
    bool AddSoakingWallmarkImpl(const Fmatrix*,IWallMarkArray*,const Fvector&,const Fvector&,float,float,float,float,const BloodThermalStamp*);
    bool LL_AddSurfaceWallmark(const Fmatrix*, IWallMarkArray*, const Fvector&, const Fvector&, float, float) override;
    bool LL_AddSoakingWallmark(const Fmatrix*, IWallMarkArray*, const Fvector&, const Fvector&, float, float, float, float) override;
	void CalculateWallmarks();
    u64 m_blood_mark_namespace=0; // unique visual lifetime; never inherited by pieces
#ifdef USE_DX11
    std::shared_ptr<PainterBloodInstance> m_blood_cells;
    bool QueueBloodCells(intrusive_ptr<CSkeletonWallmark>);
    void RefreshBloodCells();
    void RestoreBloodMaterials();
    void ClearBloodCells();
#endif
	// A preparation borrows the mark's immutable faces and coherent pose cache.
	// Consume it before preparing this same mark again, while the renderer retains
	// the mark and its existing wallmark lock. It owns no live bone pointers.
	struct WallmarkRenderPreparation
	{
		const CSkeletonWallmark* wallmark = nullptr;
		size_t vertex_count = 0;
		u32 color = 0;
		bool pose_snapshot_valid = false;
		bool pose_changed = true;
		bool use_cpu_cache = false;
		bool cache_current = false;
	};
	bool PrepareWallmarkRender(intrusive_ptr<CSkeletonWallmark> wm, WallmarkRenderPreparation& prepared);
	bool ExportWallmarkGpuSource(intrusive_ptr<CSkeletonWallmark> wm, const WallmarkRenderPreparation& prepared,
		BloodGpuSkinning::InputVertex* vertices, size_t capacity) const;
	// R22: all of a body's decals skin on the GPU with ONE bone palette captured per frame (never per mark); a mark's
	// GPU source comes from its faces alone (no per-mark pose preparation).
	// R26: faces with a vertex on a hidden bone are left out; exported = vertices written.
	bool ExportWallmarkGpuFaces(const CSkeletonWallmark& wm, BloodGpuSkinning::InputVertex* vertices, size_t capacity, size_t& exported) const;
	bool CaptureWallmarkPose(u32 frame, const Fmatrix& world);
	xr_vector<Fmatrix, CSkeletonWallmark::CacheAllocator<Fmatrix>> m_WallmarkPose; // model-space bone transforms
	Fmatrix m_WallmarkPoseWorld;
	u32 m_WallmarkPoseFrame = u32(-1);
	bool m_WallmarkPoseValid = false;
	u32 m_WallmarkPaletteFrame = u32(-1); // frame whose GPU batch holds this pose, at m_WallmarkPaletteOffset
	u32 m_WallmarkPaletteOffset = 0;
	void RenderWallmarkPrepared(intrusive_ptr<CSkeletonWallmark> wm, FVF::LIT*& verts,
		const WallmarkRenderPreparation& prepared);
	void RenderWallmark(intrusive_ptr<CSkeletonWallmark> wm, FVF::LIT* & verts);
	void ClearWallmarks();
#ifdef USE_DX11
    std::shared_ptr<PainterRemovalGpu> LL_RuntimeRemovalGpu() const;
    std::shared_ptr<PainterSkinOwnershipGpu> LL_RuntimeSkinOwnershipGpu() const;
    bool LL_RuntimeTearMask(PainterWallmarkTear& out) const; // maskdecal: a value-3 piece's tear set for its decals
    std::shared_ptr<const float> LL_RuntimeDentTexels(u32& dents, u64* key = nullptr) const; // melee (value 3): the body's dents for its decals
#endif
public:

	bool PickBone(const Fmatrix& parent_xform, IKinematics::pick_result& r, float dist, const Fvector& start,
	              const Fvector& dir, u16 bone_id);
	// triggerhitch: the nearest face over all bones in one pass (PainterPickNearest.h)
	int LL_PickNearest(const Fmatrix& parent_xform, IKinematics::pick_result& r, float range, const Fvector& start,
	                   const Fvector& dir, u16 count) override;
	virtual void EnumBoneVertices(SEnumVerticesCallback& C, u16 bone_id);
	virtual bool GetLensDetection(SLensDetection& out);
	virtual bool IsPhysicalScopeGlassBone(u16 bone) override;
    virtual void SetPhysicalScopeGlassMask(u16 bone, bool enabled) override;
    u64 PhysicalScopeGlassMask = 0;
    u64 ScopeGlassBoneSubtree(u16 bone);
public:
	CKinematics();
	virtual ~CKinematics();

	// Low level interface
	u16 _BCL LL_BoneID(LPCSTR B);
	u16 _BCL LL_BoneID(const shared_str& B);
	LPCSTR _BCL LL_BoneName_dbg(u16 ID);

	CInifile* _BCL LL_UserData() { return pUserData; }
	accel* LL_Bones() { return bone_map_N; }
	ICF CBoneInstance& _BCL LL_GetBoneInstance(u16 bone_id)
	{
		VERIFY(bone_id<LL_BoneCount());
		VERIFY(bone_instances);
		return bone_instances[bone_id];
	}

	ICF const CBoneInstance& _BCL LL_GetBoneInstance(u16 bone_id) const
	{
		VERIFY(bone_id<LL_BoneCount());
		VERIFY(bone_instances);
		return bone_instances[bone_id];
	}

	CBoneData& _BCL LL_GetData(u16 bone_id)
	{
		VERIFY(bone_id<LL_BoneCount());
		VERIFY(bones);
		CBoneData& bd = *((*bones)[bone_id]);
		return bd;
	}

	virtual const IBoneData&_BCL GetBoneData(u16 bone_id) const
	{
		VERIFY(bone_id<LL_BoneCount());
		VERIFY(bones);
		CBoneData& bd = *((*bones)[bone_id]);
		return bd;
	}

	CBoneData* _BCL LL_GetBoneData(u16 bone_id)
	{
		VERIFY(bone_id<LL_BoneCount());
		VERIFY(bones);
		u32 sz = sizeof(vecBones);
		u32 sz1 = sizeof(((*bones)[bone_id])->children);
		Msg("sz: %d", sz);
		Msg("sz1: %d", sz1);
		CBoneData* bd = ((*bones)[bone_id]);
		return bd;
	}

	u16 _BCL LL_BoneCount() const { return u16(bones->size()); }

	u16 LL_VisibleBoneCount()
	{
		u64 F = visimask.flags & ((u64(1) << u64(LL_BoneCount())) - 1);
		return (u16)btwCount1(F);
	}

	ICF Fmatrix& _BCL LL_GetTransform(u16 bone_id) { return LL_GetBoneVisible(bone_id) ? LL_GetBoneInstance(bone_id).mTransform : LL_GetBoneInstance(bone_id).mTransformHidden; }
	ICF Fmatrix& _BCL LL_GetTransform_safed(u16 bone_id) { xrCriticalSectionGuard guard(&UCalc_Mutex); return LL_GetBoneVisible(bone_id) ? LL_GetBoneInstance(bone_id).mTransform : LL_GetBoneInstance(bone_id).mTransformHidden; }
	ICF const Fmatrix& _BCL LL_GetTransform(u16 bone_id) const { return LL_GetBoneVisible(bone_id) ? LL_GetBoneInstance(bone_id).mTransform : LL_GetBoneInstance(bone_id).mTransformHidden; }
	ICF void _BCL LL_GetBoneLocalPosition(u16 bone_id, Fvector& result)
	{
		xrCriticalSectionGuard guard(&UCalc_Mutex);
		result = LL_GetBoneInstance(bone_id).mTransform.c;
	}
	ICF void _BCL LL_GetBoneLocalTransform(u16 bone_id, Fmatrix& result)
	{
		xrCriticalSectionGuard guard(&UCalc_Mutex);
		result = LL_GetBoneInstance(bone_id).mTransform;
	}
	ICF void _BCL LL_GetBoneWorldPosition(u16 bone_id, const Fmatrix& xform, Fvector& result)
	{
		LL_GetBoneLocalPosition(bone_id, result);
		xform.transform_tiny(result);
	}
	ICF void _BCL LL_GetBoneWorldTransform(u16 bone_id, const Fmatrix& xform, Fmatrix& result)
	{
		LL_GetBoneLocalTransform(bone_id, result);
		result.mulA_43(xform);
	}
	ICF void _BCL CalculateBBox(BOOL bforce = TRUE)
	{
		if (!bforce && Device.dwFrame == Visibox_frame)
			return;

		Visibox_frame = Device.dwFrame;

		Fbox Box;
		Box.invalidate();
		for (u32 b = 0; b < bones->size(); b++)
		{
			if (!LL_GetBoneVisible(u16(b)))
				continue;

			Fobb& obb = (*bones)[b]->obb;
			Fmatrix& Mbone = LL_GetBoneInstance((u16)b).mTransform;
			Fmatrix Mbox;
			obb.xform_get(Mbox);
			Fmatrix X;
			X.mul_43(Mbone, Mbox);
			Fvector& S = obb.m_halfsize;

			Fvector P, A;
			A.set(-S.x, -S.y, -S.z); X.transform_tiny(P, A); Box.modify(P);
			A.set(-S.x, -S.y, S.z); X.transform_tiny(P, A); Box.modify(P);
			A.set(S.x, -S.y, S.z); X.transform_tiny(P, A); Box.modify(P);
			A.set(S.x, -S.y, -S.z); X.transform_tiny(P, A); Box.modify(P);
			A.set(-S.x, S.y, -S.z); X.transform_tiny(P, A); Box.modify(P);
			A.set(-S.x, S.y, S.z); X.transform_tiny(P, A); Box.modify(P);
			A.set(S.x, S.y, S.z); X.transform_tiny(P, A); Box.modify(P);
			A.set(S.x, S.y, -S.z); X.transform_tiny(P, A); Box.modify(P);
		}

		if (bones->size())
		{
			vis.box.min = (Box.min);
			vis.box.max = (Box.max);
			vis.box.getsphere(vis.sphere.P, vis.sphere.R);
		}
	}
	ICF Fmatrix& LL_GetTransform_R(u16 bone_id) { return LL_GetBoneInstance(bone_id).mRenderTransform; }
	// rendering only
	Fobb& LL_GetBox(u16 bone_id)
	{
		VERIFY(bone_id<LL_BoneCount());
		return (*bones)[bone_id]->obb;
	}

	const Fbox& _BCL GetBox() const { return vis.box; }
	void LL_GetBindTransform(xr_vector<Fmatrix>& matrices);
	int LL_GetBoneGroups(xr_vector<xr_vector<u16>>& groups);

	u16 _BCL LL_GetBoneRoot() { return iRoot; }

	void LL_SetBoneRoot(u16 bone_id)
	{
		VERIFY(bone_id<LL_BoneCount());
		iRoot = bone_id;
	}

	BOOL _BCL LL_GetBoneVisible(u16 bone_id)
	{
		VERIFY(bone_id<LL_BoneCount());
		return visimask.is(u64(1) << bone_id);
	}

	u64 LL_BoneVisibleMask() const { return visimask.flags; } // R26: the decal source of a body is re-exported when this changes
	const BOOL _BCL LL_GetBoneVisible(u16 bone_id) const
	{
		VERIFY(bone_id < LL_BoneCount());
		return visimask.is(u64(1) << bone_id);
	}

	void LL_SetBoneVisible(u16 bone_id, BOOL val, BOOL bRecursive);
	u64 _BCL LL_GetBonesVisible() { return visimask.get(); }
	void LL_SetBonesVisible(u64 mask);

	bool LL_InstallGeometryVariant(IRenderVisual*& staged) override;
	void LL_ClearGeometryVariant() override;
	u64 LL_RenderGeometryGeneration() const override { return m_render_geometry_generation; }
	bool LL_InstallRuntimeGeometry(const RuntimeSkinUpdate& update, u64 current_owner_cookie) override;
	bool LL_PrepareRuntimeGeometry(const RuntimeSkinUpdate& update, u64 current_owner_cookie) override;
	bool LL_PrepareRuntimeGeometryWithDonor(const RuntimeSkinUpdate& update, u64 current_owner_cookie,
		IRenderVisual* donor_visual, const Fmatrix* recipient_xform) override;
	bool LL_PrepareRuntimeGeometryInternal(const RuntimeSkinUpdate& update, u64 current_owner_cookie,
		CKinematics* mark_donor, const Fmatrix* recipient_xform);
	bool LL_CommitPreparedRuntimeGeometry(u64 current_owner_cookie) override;
	bool LL_CanCommitPreparedRuntimeGeometry(u64 current_owner_cookie) const override;
	void LL_DiscardPreparedRuntimeGeometry() override;
	void LL_ClearRuntimeGeometry() override;
	IRuntimeGeometryStage* LL_StageRuntimeGeometry(const RuntimeSkinUpdate& header,
		const std::shared_ptr<const gfy_packed::Mesh>& packed, LPCSTR cavity_shader, LPCSTR cavity_texture,
		const xr_vector<shared_str>* tissue_textures, u64 current_owner_cookie, int& outcome) override;
	bool LL_InstallStagedRuntimeGeometry(IRuntimeGeometryStage* stage, const RuntimeSkinUpdate& header,
		u64 current_owner_cookie) override;
	bool LL_PrepareStagedRuntimeGeometry(IRuntimeGeometryStage* stage, const RuntimeSkinUpdate& header,
		u64 current_owner_cookie, IRenderVisual* donor_visual, const Fmatrix* recipient_xform) override;
	unsigned LL_WarmRuntimeShaders(LPCSTR cavity_shader, LPCSTR cavity_texture,
		const xr_vector<shared_str>* tissue_textures, unsigned budget,
        const std::shared_ptr<const painter_detail::TissueField>& tissue_field={}) override;
	// Shared by the synchronous and staged paths (SkeletonRuntimeGeometry_impl.h).
	const char* LL_AdmitRuntimeHeader(const RuntimeSkinUpdate& update, u64 current_owner_cookie, bool has_children,
		xr_vector<IRenderVisual*>& originals);
	const char* LL_FinishRuntimeGeometryPrepare(struct RuntimeGeometryPacket* next, size_t count, const xr_vector<u8>& removed,
		const xr_vector<u8>& accepts, const xr_vector<u8>* sources_done, CKinematics* mark_donor, const Fmatrix* recipient_xform,
		xr_vector<IRenderVisual*>& published, SkeletonWMVec& rebuilt);
	bool HasGeometryVariant() const { return m_geometry_donor != nullptr; }
	// Only read face tables through this when operating on current children.
	// Bind/IK/mass and bone instances still come from the original skeleton.
	CBoneData::FacesVec& GeometryFaces(u16 bone_id, u16 source_child_index);

	// Main functionality
	virtual void CalculateBones(BOOL bForceExact = FALSE); // Recalculate skeleton
	void CalculateBones_Invalidate();
	bool CopyBonesFrom(IKinematics* from);

	void Callback(UpdateCallback C, void* Param)
	{
		Update_Callback = C;
		Update_Callback_Param = Param;
	}

	//	Callback: data manipulation
	virtual void SetUpdateCallback(UpdateCallback pCallback) { Update_Callback = pCallback; }
	virtual void SetUpdateCallbackParam(void* pCallbackParam) { Update_Callback_Param = pCallbackParam; }

	virtual UpdateCallback GetUpdateCallback() { return Update_Callback; }
	virtual void* GetUpdateCallbackParam() { return Update_Callback_Param; }

	virtual bool NeedUCalc() { return UCalc_ThisFrame; }

	// debug
#ifdef DEBUG
	void							DebugRender			(Fmatrix& XFORM);
#endif
protected:
	virtual shared_str getDebugName() { return dbg_name; }
public:

	// General "Visual" stuff
	virtual void Copy(dxRender_Visual* pFrom);
	virtual void Load(const char* N, IReader* data, u32 dwFlags);
	virtual void Spawn();
	virtual void Depart();
	virtual void Release();

	virtual IKinematicsAnimated* dcast_PKinematicsAnimated() { return 0; }
	virtual IRenderVisual* _BCL dcast_RenderVisual() { return this; }
	virtual IKinematics* _BCL dcast_PKinematics() { return this; }
	//	virtual	CKinematics*		dcast_PKinematics	()				{ return this;	}

	virtual u32 mem_usage(bool bInstance)
	{
		u32 sz = sizeof(*this);
		sz += bone_instances ? bone_instances->mem_usage() : 0;
		if (!bInstance)
		{
			//			sz					+= pUserData?pUserData->mem_usage():0;
			for (vecBonesIt b_it = bones->begin(); b_it != bones->end(); ++b_it)
				sz += sizeof(vecBones::value_type) + (*b_it)->mem_usage();
		}
		return sz;
	}

private:
	bool m_is_original_lod;
};

IC CKinematics* PCKinematics(dxRender_Visual* V) { return V ? (CKinematics*)V->dcast_PKinematics() : 0; }
//---------------------------------------------------------------------------
#endif
