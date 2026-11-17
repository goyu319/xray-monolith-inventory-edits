#ifndef	Kinematics_included
#define	Kinematics_included
#pragma once

#include "RenderVisual.h"
#include "BloodThermalStamp.h"
#include "../../xrCore/intrusive_ptr.h"
#include <memory>
#include "OriginalSkinProxySource.h"
namespace painter_detail {struct TissueField;}

typedef void (* UpdateCallback)(IKinematics* P);

class CBoneData;
class IBoneData;
class IKinematicsAnimated;
class IRenderVisual;
class ISpatial;
class CBoneInstance;
struct SEnumVerticesCallback;
struct RuntimeSkinUpdate;
class IWallMarkArray;
class IRuntimeGeometryStage;
namespace gfy_packed { struct Mesh; }

using ISpatialShared = intrusive_ptr<ISpatial>;

// 10 fps
#define UCalc_Interval		(u32(100))

// pip measured lens geometry for one visual, bind-pose model space, metres
// filled by CKinematics::GetLensDetection, ok == false when no fittable lens
struct SLensDetection
{
	Fvector  eye_center;
	Fvector  eye_normal;
	float    eye_radius;
	Fvector  obj_center;
	float    obj_radius;
	bool     has_objective;
	Fvector4 offset; // scope_objective_lens_offset x,y,z,w in eyepiece-radius units
	float    mm;     // s3ds_objective_mm 2000 x obj_radius, 0 when no objective
	int      source; // 0 lens verts, 1 near bone, 2 tube march objective
	bool     ok;
};

class IKinematics
{
public:
    xrCriticalSection UCalc_Mutex;
	typedef xr_vector<std::pair<shared_str, u16>> accel;

	struct pick_result
	{
		Fvector normal;
		float dist;
		Fvector tri[3];
	};

public:

#ifdef OPTIMIZE_CALCULATE_BONES
	ISpatialShared spatialParent = nullptr;
#endif

	virtual void Bone_Calculate(CBoneData* bd, Fmatrix* parent) = 0;
	virtual void Bone_GetAnimPos(Fmatrix& pos, u16 id, u8 channel_mask, bool ignore_callbacks) = 0;

	virtual bool PickBone(const Fmatrix& parent_xform, pick_result& r, float dist, const Fvector& start,
	                      const Fvector& dir, u16 bone_id) = 0;
	virtual void EnumBoneVertices(SEnumVerticesCallback& C, u16 bone_id) = 0;

	// pip run-or-cached measured lens fit for this visual, false for non-skinned or no lens
	virtual bool GetLensDetection(SLensDetection& out) { return false; }
	// Admits converted glass and independently hideable opaque siblings only.
	virtual bool IsPhysicalScopeGlassBone(u16 bone) { return false; }
	virtual void SetPhysicalScopeGlassMask(u16 bone, bool enabled) {}

	// Low level interface
	virtual u16 _BCL LL_BoneID(LPCSTR B) = 0;
	virtual u16 _BCL LL_BoneID(const shared_str& B) = 0;
	virtual LPCSTR _BCL LL_BoneName_dbg(u16 ID) = 0;

	virtual CInifile* _BCL LL_UserData() = 0;
	virtual accel* LL_Bones() = 0;

	virtual ICF CBoneInstance& _BCL LL_GetBoneInstance(u16 bone_id) = 0;

	virtual CBoneData& _BCL LL_GetData(u16 bone_id) = 0;

	virtual const IBoneData& _BCL GetBoneData(u16 bone_id) const = 0;

	virtual u16 _BCL LL_BoneCount() const = 0;
	virtual u16 LL_VisibleBoneCount() = 0;

	virtual ICF Fmatrix& _BCL LL_GetTransform(u16 bone_id) = 0;
	virtual ICF Fmatrix& _BCL LL_GetTransform_safed(u16 bone_id) = 0;
	virtual ICF const Fmatrix& _BCL LL_GetTransform(u16 bone_id) const = 0;
	virtual ICF void _BCL LL_GetBoneLocalPosition(u16 bone_id, Fvector& result) {}
	virtual ICF void _BCL LL_GetBoneLocalTransform(u16 bone_id, Fmatrix& result) {}
	virtual ICF void _BCL LL_GetBoneWorldPosition(u16 bone_id, const Fmatrix& xform, Fvector& result) {}
	virtual ICF void _BCL LL_GetBoneWorldTransform(u16 bone_id, const Fmatrix& xform, Fmatrix& result) {}
	virtual ICF void _BCL CalculateBBox(BOOL bforce = TRUE) {}

	virtual ICF Fmatrix& LL_GetTransform_R(u16 bone_id) = 0;
	virtual Fobb& LL_GetBox(u16 bone_id) = 0;
	virtual const Fbox& _BCL GetBox() const = 0;
	virtual void LL_GetBindTransform(xr_vector<Fmatrix>& matrices) = 0;
	virtual int LL_GetBoneGroups(xr_vector<xr_vector<u16>>& groups) = 0;

	virtual u16 _BCL LL_GetBoneRoot() = 0;
	virtual void LL_SetBoneRoot(u16 bone_id) = 0;

	virtual BOOL _BCL LL_GetBoneVisible(u16 bone_id) = 0;
	virtual void LL_SetBoneVisible(u16 bone_id, BOOL val, BOOL bRecursive) = 0;
	virtual u64 _BCL LL_GetBonesVisible() = 0;
	virtual void LL_SetBonesVisible(u64 mask) = 0;

	//--DSR-- SilencerOverheat_start
	virtual IRenderVisual* GetVisualByBone(u16 bone_id) = 0;
	virtual IRenderVisual* GetVisualByBone(LPCSTR bone_name) = 0;
	//--DSR-- SilencerOverheat_end

	// Low-level reviewed-asset geometry transaction. Caller must own a dead
	// instance and execute outside renderer/physics worker activity. This does
	// not hide bones, remove physics, admit an asset, or replace the skeleton.
	// A successful install consumes staged; caller owns it on failure.
	virtual bool LL_InstallGeometryVariant(IRenderVisual*& staged) { return false; }
	virtual void LL_ClearGeometryVariant() {}
	virtual u64 LL_RenderGeometryGeneration() const { return 0; }
	virtual bool LL_InstallRuntimeGeometry(const RuntimeSkinUpdate&, u64 current_owner_cookie) { return false; }
	virtual bool LL_PrepareRuntimeGeometry(const RuntimeSkinUpdate&, u64 current_owner_cookie) { return false; }
	// Snapshot another live skeleton's marks during this owner-barrier call only.
	// The donor is never retained by the prepared geometry or copied marks.
	virtual bool LL_PrepareRuntimeGeometryWithDonor(const RuntimeSkinUpdate&, u64 current_owner_cookie,
		IRenderVisual* donor_visual, const Fmatrix* recipient_xform) { return false; }
	virtual bool LL_CommitPreparedRuntimeGeometry(u64 current_owner_cookie) { return false; }
	virtual bool LL_CanCommitPreparedRuntimeGeometry(u64 current_owner_cookie) const { return false; }
	virtual void LL_DiscardPreparedRuntimeGeometry() {}
	virtual void LL_ClearRuntimeGeometry() {}
	// Off-thread publication (RuntimeGeometryStage.h). Stage at a publish barrier
	// (outcome: RuntimeStageOutcome), install at a LATER publish barrier once the
	// stage is Ready(). header carries owner/revision/rig and no children; the
	// stage keeps packed alive. Install admits again and commits atomically.
	virtual IRuntimeGeometryStage* LL_StageRuntimeGeometry(const RuntimeSkinUpdate& header,
		const std::shared_ptr<const gfy_packed::Mesh>& packed, LPCSTR cavity_shader, LPCSTR cavity_texture,
		const xr_vector<shared_str>* tissue_textures, u64 current_owner_cookie, int& outcome) { outcome = 3; return nullptr; }
	virtual bool LL_InstallStagedRuntimeGeometry(IRuntimeGeometryStage* stage, const RuntimeSkinUpdate& header,
		u64 current_owner_cookie) { return false; }
	// Adopt a completed stage WITHOUT changing visible geometry. Donor marks
	// are captured only at this barrier; the donor is not retained by a worker.
	// Null donor/xform means this owner. The caller commits all parts together.
	virtual bool LL_PrepareStagedRuntimeGeometry(IRuntimeGeometryStage* stage, const RuntimeSkinUpdate& header,
		u64 current_owner_cookie, IRenderVisual* donor_visual, const Fmatrix* recipient_xform) { return false; }
    // Explicit known-surface projection: a new wound may have removed its ray
    // center already. The footprint still clips against surviving geometry.
    virtual bool LL_AddBloodSurfaceWallmark(const Fmatrix* xf,IWallMarkArray* array,const Fvector& p,const Fvector& n,
        float size,float ttl,const BloodThermalStamp& stamp){return LL_AddSurfaceWallmark(xf,array,p,n,size,ttl);}
    // The same with this decal's own depth limit (m; <= 0 or not finite = the 2-4 cm rule; clamped to 1-25 cm) and roll
    // about its normal (degrees; not finite = the random +-20 degrees).
    virtual bool LL_AddBloodSurfaceWallmarkPlaced(const Fmatrix* xf,IWallMarkArray* array,const Fvector& p,const Fvector& n,
        float size,float ttl,const BloodThermalStamp& stamp,float depth,float roll){return LL_AddBloodSurfaceWallmark(xf,array,p,n,size,ttl,stamp);}
    virtual bool LL_AddBloodSoakingWallmark(const Fmatrix* xf,IWallMarkArray* array,const Fvector& p,const Fvector& n,
        float size,float ttl,float opacity,float seconds,const BloodThermalStamp& stamp){return LL_AddSoakingWallmark(xf,array,p,n,size,ttl,opacity,seconds);}
    virtual bool LL_AddSurfaceWallmark(const Fmatrix*, IWallMarkArray*, const Fvector&, const Fvector&, float, float) { return false; }
    virtual bool LL_AddSoakingWallmark(const Fmatrix*, IWallMarkArray*, const Fvector&, const Fvector&, float, float, float, float) { return false; }

	// Main functionality
	virtual void CalculateBones(BOOL bForceExact = FALSE) = 0; // Recalculate skeleton
	virtual void CalculateBones_Invalidate() = 0;
	// Copy the current bone transforms of another model
	virtual bool CopyBonesFrom(IKinematics* from) = 0;
	virtual void Callback(UpdateCallback C, void* Param) = 0;

	//	Callback: data manipulation
	virtual void SetUpdateCallback(UpdateCallback pCallback) = 0;
	virtual void SetUpdateCallbackParam(void* pCallbackParam) = 0;
	virtual bool NeedUCalc() = 0;

	virtual UpdateCallback GetUpdateCallback() = 0;
	virtual void* GetUpdateCallbackParam() = 0;
	//UpdateCallback						Update_Callback;
	//void*								Update_Callback_Param;
	virtual IRenderVisual* _BCL dcast_RenderVisual() = 0;
	virtual IKinematicsAnimated* dcast_PKinematicsAnimated() = 0;

	// debug
#ifdef DEBUG
	virtual void						DebugRender			(Fmatrix& XFORM) = 0;
#endif
	virtual shared_str getDebugName() = 0;
	// Bind-time shader warm-up for runtime (wound) children (renderer-wide
	// RuntimeShaderCache): records the handles this model's originals hold and
	// queues the cavity pairs (cavity_shader with cavity_texture and every
	// non-empty tissue texture) for compilation off the hit path; compiles up
	// to `budget` queued pairs now (the renderer drains the rest, one per
	// frame). Returns the pairs still queued. Main thread; null arguments only
	// drain. Kept last so earlier vtable slots do not move.
	virtual unsigned LL_WarmRuntimeShaders(LPCSTR cavity_shader, LPCSTR cavity_texture,
		const xr_vector<shared_str>* tissue_textures, unsigned budget,
        const std::shared_ptr<const painter_detail::TissueField>& tissue_field={}) { return 0; }
    // Optional immutable original source, appended to preserve existing vtable slots.
    virtual std::shared_ptr<const painter_skin_proxy::Source> LL_OriginalSkinProxySource() const { return {}; }
    // triggerhitch: the nearest posed face over all bones in one pass (CKinematics: PainterPickNearest.h). -1 = not
    // available (ask PickBone bone by bone as before), 0 = none (or the old loop's refusal), 1 = r as the old loop's last answer.
    virtual int LL_PickNearest(const Fmatrix& parent_xform, pick_result& r, float range, const Fvector& start,
                               const Fvector& dir, u16 count) { return -1; }
};

IC IKinematics* PKinematics(IRenderVisual* V) { return V ? V->dcast_PKinematics() : 0; }

#endif	//	Kinematics_included
