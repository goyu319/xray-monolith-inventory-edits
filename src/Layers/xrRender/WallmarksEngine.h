// WallmarksEngine.h: interface for the CWallmarksEngine class.
//
//////////////////////////////////////////////////////////////////////
#pragma once
#include "../../Include/xrRender/BloodThermalStamp.h"

namespace WallmarksEngine
{
	struct wm_slot;
}

class CSkeletonWallmark;
class CKinematics;
struct BloodGpuWallmarkState;

class CWallmarksEngine
{
public:
	typedef WallmarksEngine::wm_slot wm_slot;

public:
	struct static_wallmark
	{
		Fsphere bounds;
		xr_vector<FVF::LIT> verts;
		float m_fTimeStart;

		float m_fTimeEnd;
        bool m_thermal_blood=false;
        BloodThermalStamp m_thermal_stamp;

		IC float TimeStart() { return m_fTimeStart; }
		IC float TimeEnd() { return m_fTimeEnd; }
	};

    // R23: the per-sector list keeps each mark's bounds and expiry beside its pointer, so the per-frame expiry
    // sweep and the per-frame visibility test read a compact array instead of chasing every mark's heap object.
    struct static_entry
    {
        Fvector P;
        float R;
        float expire; // absolute fTimeGlobal at which the mark expires; +inf = never
        static_wallmark* W;
    };
    typedef xr_vector<static_entry> StaticWMVec;
    typedef xr_unordered_map<IRender_Sector*, StaticWMVec> StaticWMSectorMap;
    typedef xr_vector< wm_slot* > WMSlotVec;
private:
	xr_vector<static_wallmark*> static_pool;
	WMSlotVec marks;
	// Lookup only: marks preserves the existing shader/render order and owns slots.
	xr_unordered_map<Shader*, wm_slot*> slot_index;
	// Reused CPU staging for marks larger than one GPU stream reservation.
	xr_vector<FVF::LIT> large_wallmark_vertices;
	ref_geom hGeom;
#ifdef USE_DX11
    ref_geom hGeomMasked;
#endif
	BloodGpuWallmarkState* gpu_skinning = nullptr;

	Fvector sml_normal;
	CFrustum sml_clipper;
	sPoly sml_poly_dest;
	sPoly sml_poly_src;

	xrXRC xrc;
	CDB::Collector sml_collector;
	xr_vector<u32> sml_adjacency;

	xrCriticalSection lock;
private:
	wm_slot* FindSlot(ref_shader shader);
	wm_slot* AppendSlot(ref_shader shader);
private:
	void BuildMatrix(Fmatrix& dest, float invsz, const Fvector& from);
	void RecurseTri(u32 T, Fmatrix& mView, static_wallmark& W);
	void AddWallmark_internal(CDB::TRI* pTri, const Fvector* pVerts, const Fvector& contact_point, ref_shader hTexture, float sz, float ttl = 0.f, bool random_rotation = true);
	void AddWallmark_internal(CDB::TRI* pTri, const Fvector* pVerts, const Fvector& contact_point, ref_shader hTexture, float sz, float ttl, float rotation,const BloodThermalStamp& stamp={});

	static_wallmark* static_wm_allocate();
	void static_wm_render(static_wallmark* W, FVF::LIT* & V);
	void static_wm_destroy(static_wallmark* W);

	void skeleton_wm_render(intrusive_ptr<CSkeletonWallmark>, FVF::LIT* & V);
public:
	CWallmarksEngine();
	~CWallmarksEngine();
	// edit wallmarks
	void AddStaticWallmark(CDB::TRI* pTri, const Fvector* pVerts, const Fvector& contact_point, ref_shader hTexture, float sz, float ttl = 0.f, bool ignore_opt = false, bool random_rotation = true);

	// demonized: add user defined rotation to wallmark
	void AddStaticWallmark(CDB::TRI* pTri, const Fvector* pVerts, const Fvector& contact_point, ref_shader hTexture, float sz, float ttl, bool ignore_opt, float rotation);

    void AddStaticBloodWallmark(CDB::TRI* tri,const Fvector* verts,const Fvector& point,ref_shader shader,
        float size,float ttl,bool ignore_opt,float rotation,const BloodThermalStamp& stamp);
	void AddSkeletonWallmark(intrusive_ptr<CSkeletonWallmark> wm);
	void AddSkeletonWallmark(const Fmatrix* xf, CKinematics* obj, ref_shader& sh, const Fvector& start,
	                         const Fvector& dir, float size, float ttl = 0.f, bool ignore_opt = false);

    void AddSkeletonBloodWallmark(const Fmatrix*,CKinematics*,ref_shader&,const Fvector&,const Fvector&,
        float,float,bool,const BloodThermalStamp&);
    void RemoveSkeletonWallmarksFromObject(CKinematics* obj);
    void CollectSkeletonWallmarks(CKinematics* obj, xr_vector<intrusive_ptr<CSkeletonWallmark>>& output);

    void UpdateWallmarks();

	// render
	void Render();

	void clear();
	void ResetGpuWallmarks(bool keep_device_state = false); // R26: true on a vid_restart (the device survives)
	void DumpGpuWallmarkStats();
};
