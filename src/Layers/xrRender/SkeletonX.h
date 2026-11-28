// SkeletonX.h: interface for the CSkeletonX class.
//
//////////////////////////////////////////////////////////////////////

#ifndef SkeletonXH
#define SkeletonXH
#pragma once

#include "SkeletonCustom.h"
#include "SkeletonXVertRender.h"
#include "scope_glass_surface_frames.h"

// refs
class CKinematics;
class Fvisual;

void svp_hud_bone_snapshot_begin();
extern bool g_svp_hud_frozen_pass;
extern bool g_svp_hud_history_write;

//.#pragma pack(push,4)

struct SEnumVerticesCallback;

class CSkeletonX
{
protected:
	enum { vertRenderFVF = D3DFVF_XYZ | D3DFVF_NORMAL | D3DFVF_TEX1 };

	enum { RM_SKINNING_SOFT, RM_SINGLE, RM_SKINNING_1B, RM_SKINNING_2B, RM_SKINNING_3B, RM_SKINNING_4B };

	CKinematics* Parent; // setted up by parent
    std::shared_ptr<painter_skin_proxy::Source> m_original_skin_proxy_child;
	ref_smem<vertBoned1W> Vertices1W; // shared
	ref_smem<vertBoned2W> Vertices2W; // shared
	ref_smem<vertBoned3W> Vertices3W; // shared
	ref_smem<vertBoned4W> Vertices4W; // shared
	ref_smem<u16> BonesUsed; // actual bones which have influence on vertices

	u16 RenderMode;
	u16 ChildIDX;
	bool scope_surface_cached = false;
	std::vector<scope_surface_frame> scope_surface_frames;
	void BindScopeSurfaceFrames(CBoneInstance* render_bones);

	// render-mode specifics
	union
	{
		struct
		{
			// soft-skinning only
			u32 cache_DiscardID;
			u32 cache_vCount;
			u32 cache_vOffset;
		};

		u32 RMS_boneid; // single-bone-rendering
		u32 RMS_bonecount; // skinning, maximal bone ID
	};

	void _Copy(CSkeletonX* V);
#ifdef USE_DX11
	std::shared_ptr<const CSkeletonWallmark::ProjectionSource> m_ProjectionSource;
    std::shared_ptr<PainterWallmarkOwnershipGpu> m_WallmarkOwnership;
    u32 m_WallmarkSourceChild=~0u;
    std::shared_ptr<const std::vector<u32>> m_WallmarkSourceFaces;
    void AssignWallmarkOwnership(CSkeletonWallmark::WMFace&face,u32 triangle)const;
	u32 m_ProjectionIndexBase = 0, m_ProjectionIndexCount = 0;
	void CacheProjectionSource(u32 iBase, u32 iCount, const xr_vector<xr_vector<CBoneData::FacesVec>>* faces = nullptr, bool force = false) noexcept;
#endif
	void _Render_soft(ref_geom& hGeom, u32 vCount, u32 iOffset, u32 pCount, CBoneInstance* bones);
	// vBase: DrawIndexed base vertex (Fvisual::vBase; non-zero for runtime
	// children that share one packet VB). Ignored by the software path.
	void _Render(ref_geom& hGeom, u32 vBase, u32 vCount, u32 iOffset, u32 pCount, u32 localPrimitive);
	void _Load(const char* N, IReader* data, u32& dwVertCount);

	virtual void _Load_hw(Fvisual& V, void* data) = 0;
	virtual void _CollectBoneFaces(Fvisual* V, u32 iBase, u32 iCount) = 0;

	void _FillVerticesSoft1W(const Fmatrix& view, CSkeletonWallmark& wm, const Fvector& normal, float size,
	                         u16* indices, CBoneData::FacesVec& faces);
	void _FillVerticesSoft2W(const Fmatrix& view, CSkeletonWallmark& wm, const Fvector& normal, float size,
	                         u16* indices, CBoneData::FacesVec& faces);
	void _FillVerticesSoft3W(const Fmatrix& view, CSkeletonWallmark& wm, const Fvector& normal, float size,
	                         u16* indices, CBoneData::FacesVec& faces);
	void _FillVerticesSoft4W(const Fmatrix& view, CSkeletonWallmark& wm, const Fvector& normal, float size,
	                         u16* indices, CBoneData::FacesVec& faces);

	virtual void _FillVerticesHW1W(const Fmatrix& view, CSkeletonWallmark& wm, const Fvector& normal, float size,
	                               Fvisual* V, u16* indices, CBoneData::FacesVec& faces) =0;
	virtual void _FillVerticesHW2W(const Fmatrix& view, CSkeletonWallmark& wm, const Fvector& normal, float size,
	                               Fvisual* V, u16* indices, CBoneData::FacesVec& faces) =0;
	virtual void _FillVerticesHW3W(const Fmatrix& view, CSkeletonWallmark& wm, const Fvector& normal, float size,
	                               Fvisual* V, u16* indices, CBoneData::FacesVec& faces) =0;
	virtual void _FillVerticesHW4W(const Fmatrix& view, CSkeletonWallmark& wm, const Fvector& normal, float size,
	                               Fvisual* V, u16* indices, CBoneData::FacesVec& faces) =0;
	virtual void _FillVertices(const Fmatrix& view, CSkeletonWallmark& wm, const Fvector& normal, float size,
	                           Fvisual* V, u16 bone_id, u32 iBase, u32 iCount) =0;

	BOOL _PickBoneSoft1W(IKinematics::pick_result& r, float range, const Fvector& S, const Fvector& D, u16* indices,
	                     CBoneData::FacesVec& faces);
	BOOL _PickBoneSoft2W(IKinematics::pick_result& r, float range, const Fvector& S, const Fvector& D, u16* indices,
	                     CBoneData::FacesVec& faces);
	BOOL _PickBoneSoft3W(IKinematics::pick_result& r, float range, const Fvector& S, const Fvector& D, u16* indices,
	                     CBoneData::FacesVec& faces);
	BOOL _PickBoneSoft4W(IKinematics::pick_result& r, float range, const Fvector& S, const Fvector& D, u16* indices,
	                     CBoneData::FacesVec& faces);

	virtual BOOL _PickBoneHW1W(IKinematics::pick_result& r, float range, const Fvector& S, const Fvector& D, Fvisual* V,
	                           u16* indices, CBoneData::FacesVec& faces) =0;
	virtual BOOL _PickBoneHW2W(IKinematics::pick_result& r, float range, const Fvector& S, const Fvector& D, Fvisual* V,
	                           u16* indices, CBoneData::FacesVec& faces) =0;
	virtual BOOL _PickBoneHW3W(IKinematics::pick_result& r, float range, const Fvector& S, const Fvector& D, Fvisual* V,
	                           u16* indices, CBoneData::FacesVec& faces) =0;
	virtual BOOL _PickBoneHW4W(IKinematics::pick_result& r, float range, const Fvector& S, const Fvector& D, Fvisual* V,
	                           u16* indices, CBoneData::FacesVec& faces) =0;

	virtual BOOL _PickBone(IKinematics::pick_result& r, float range, const Fvector& S, const Fvector& D, Fvisual* V,
	                       u16 bone_id, u32 iBase, u32 iCount) =0;
public:
	BOOL has_visible_bones();
	u64 ScopeGlassBoneMask();
	bool IsPhysicalScopeGlass();
	bool SVP_LensBoneXform(Fmatrix& out); // pip lens bone object-space skinning matrix for the SVP eyepiece
	bool SVP_LensBoneVisible(); // pip lens bone visibility for the eyepiece pick
	bool SVP_GetLensDetection(SLensDetection& out); // pip forward the owning kinematics measured lens fit for the render-side diag
	bool SVP_CaptureBoneSnapshot();
	bool SVP_BoneSnapshotReady() const; // pip exact main hud bone palette shared by both viewports
	bool SVP_RigidBoneSnapshotXform(Fmatrix& out);
	bool SVP_BoneSnapshotXform(Fmatrix& out);
	bool SVP_BoneSnapshotXform(u16 bone, Fmatrix& out);
	bool SVP_BoneSnapshotXform(const IKinematics* owner, u16 bone, Fmatrix& out);
	bool SVP_BoneSnapshotVisible(BOOL& visible);
	u16 SVP_MeasureBoneCount() const; // pip bone count for the near-plane derive, 0 when unparented
	bool SVP_BoneCornersWorld(u16 bone, const Fmatrix& pose, Fvector* corners8); // pip posed bone obb
	u32 SVP_MeshVertsWorld(const Fmatrix& pose, const xr_vector<u8>& through, xr_vector<Fvector>& out); // pip posed verts of the flagged bones
	const void* SVP_SkeletonOwner() const { return Parent; }
	// pip append each dedup bind-pose vertex, model space, with its global dominant bone id
	void SVP_GatherVerts(xr_vector<Fvector>& positions, xr_vector<u16>& bones);
    bool receives_wallmarks = true;
    // triggerhitch: what PickBone reads, for CKinematics::LL_PickNearest's posed memo (PainterPickNearest.h)
    CKinematics* PainterPickParent() const { return Parent; }
    virtual u32 PainterPickIndexBase() { return 0; } // the index base PickBone passes (CSkeletonX_PM: its window)
#ifdef USE_DX11
    const u16* PainterPickIndices();
    size_t PainterPickIndexCount();
    const void* PainterPickVertexKey();
    u32 PainterPickVertexCount();
    bool PainterPoseVertices(Fvector* out, u32 count);
    bool PainterPickAccept(u32 primitive, const u16* indices, float u, float v);
    // wallmarkpick: one face of _FillVerticesSoft*W with its posed corners given (CKinematics::AddWallmark, PainterWallmarkScan.h)
    void PainterFillPosed(const Fmatrix& view, CSkeletonWallmark& wm, const Fvector& normal, float size, u16 face, const Fvector* posed);
#endif
#ifdef USE_DX11
	// Called by CKinematics::Load only after its bone-face sort/unique phase.
	void FinalizeProjectionSource(const xr_vector<xr_vector<CBoneData::FacesVec>>* faces = nullptr, bool force = false) noexcept;
	bool AppendProjectionRanges(const U16Vec& bones, CSkeletonWallmark::ProjectionSnapshot& hit, bool require_gpu_ready = true) const;
	bool HasProjectionSource() const { return m_ProjectionSource!=nullptr; }
    void SetWallmarkOwnership(const std::shared_ptr<PainterWallmarkOwnershipGpu>&field){m_WallmarkOwnership=field;}
    const std::shared_ptr<const std::vector<u32>>& PainterBurnRemap()const{return m_WallmarkSourceFaces;}
    u32 PainterBurnSourceChild()const{return m_WallmarkSourceChild==~0u?ChildIDX:m_WallmarkSourceChild;}
    void SetWallmarkProvenance(u32 child,const std::vector<u32>&faces){m_WallmarkSourceChild=child;m_WallmarkSourceFaces=std::make_shared<const std::vector<u32>>(faces);}
	const CSkeletonWallmark::ProjectionSource* ProjectionSourceKey() const { return m_ProjectionSource.get(); }
	const std::shared_ptr<const CSkeletonWallmark::ProjectionSource>& ProjectionSourceHandle() const { return m_ProjectionSource; }
#endif

	//--DSR-- SilencerOverheat_start
	BOOL has_bone_id(u16 bone_id)
	{
		for (u32 it = 0; it < BonesUsed.size(); it++)
			if (BonesUsed[it] == bone_id)
			{
				return TRUE;
			}
		return FALSE;
	}
	//--DSR-- SilencerOverheat_end

	CSkeletonX()
	{
		Parent = 0;
		ChildIDX = u16(-1);
	}

	u16 GeometryChildIndex() const { return ChildIDX; }
    bool AppendOriginalSkinProxySource(painter_skin_proxy::Source&,bool opaque=false);
	virtual void SetParent(CKinematics* K) { Parent = K; }
	virtual void AfterLoad(CKinematics* parent, u16 child_idx) =0;
	virtual void EnumBoneVertices(SEnumVerticesCallback& C, u16 bone_id) =0;
	virtual BOOL PickBone(IKinematics::pick_result& r, float dist, const Fvector& start, const Fvector& dir,
	                      u16 bone_id) =0;
	virtual void FillVertices(const Fmatrix& view, CSkeletonWallmark& wm, const Fvector& normal, float size,
	                          u16 bone_id) =0;

#if defined(USE_DX10) || defined(USE_DX11)
protected:
	void _DuplicateIndices(const char* N, IReader* data);

	//	Index buffer replica since we can't read from index buffer in DX10
	ref_smem<u16> m_Indices;
#endif	//	USE_DX10
};

struct SkeletonPickAcceptAll {bool operator()(u32,float,float)const noexcept{return true;}};
template <typename T_vertex, typename T_buffer, typename T_accept=SkeletonPickAcceptAll>
BOOL pick_bone(T_buffer vertices, CKinematics* Parent, IKinematics::pick_result& r, float dist, const Fvector& S,
               const Fvector& D, u16* indices, CBoneData::FacesVec& faces, T_accept accept=T_accept{})
{
	for (CBoneData::FacesVecIt it = faces.begin(); it != faces.end(); it++)
	{
		u32 idx = (*it) * 3;
		for (u32 k = 0; k < 3; k++)
		{
			T_vertex& vert = vertices[indices[idx + k]];
			get_pos_bones(vert, r.tri[k], Parent);
		}
		float u, v;
		r.dist = flt_max;
		if (CDB::TestRayTri(S, D, r.tri, u, v, r.dist, true) && (r.dist < dist))
		{
			if(!accept(*it,u,v))continue;
			r.normal.mknormal(r.tri[0], r.tri[1], r.tri[2]);
			return TRUE;
		};
	}
	return FALSE;
}

#if defined(USE_DX10) || defined(USE_DX11)
template <typename T>
BOOL pick_bone(CKinematics* Parent, IKinematics::pick_result& r, float dist, const Fvector& S, const Fvector& D,
               Fvisual* V, u16* indices, CBoneData::FacesVec& faces)
{
	VERIFY(!"Not implemented");
	return FALSE;
}
#else	USE_DX10
template <typename T>
BOOL pick_bone(CKinematics* Parent, IKinematics::pick_result& r, float dist, const Fvector& S, const Fvector& D,
               Fvisual* V, u16* indices, CBoneData::FacesVec& faces)
{
	T* vertices;
	CHK_DX(V->p_rm_Vertices->Lock(V->vBase,V->vCount,(void**)&vertices,D3DLOCK_READONLY));
	bool intersect = !!pick_bone<T, T*>(vertices, Parent, r, dist, S, D, indices, faces);
	CHK_DX(V->p_rm_Vertices->Unlock());
	return intersect;
}
#endif	//	USE_DX10

#endif // SkeletonXH
