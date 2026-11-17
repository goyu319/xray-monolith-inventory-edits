#ifndef PHYSICS_SKELETON_OBJECT_H
#define PHYSICS_SKELETON_OBJECT_H
#include "physicsshellholder.h"
#include "PHSkeleton.h"
#include "painter_arm_pose.h"
#include "painter_voxel/PainterDamagedFragment.h"


class CSE_ALifePHSkeletonObject;

class CPhysicsSkeletonObject :
	public CPhysicsShellHolder,
	public CPHSkeleton

{
	typedef CPhysicsShellHolder inherited;
    SPainterArmPoseData* m_painter_arm_pose=nullptr;
    SPainterDamagedFragmentData* m_damaged_fragment=nullptr;
    bool m_painter_thermal_corpse=false;
    u32 m_painter_thermal_death_time=0;
    bool m_painter_thermal_living=false; // thermal3: cut off a LIVING body: starts at a living body's heat, fades from the cut
    bool m_painter_thermal_rope=false;   // thermal3: a gut rope: all interior tissue, drawn with a plain material
    friend bool painter_damaged_fragment::build_shell(CPhysicsSkeletonObject&,SPainterDamagedFragmentData*);

public:
	CPhysicsSkeletonObject(void);
	virtual ~CPhysicsSkeletonObject(void);


	virtual BOOL net_Spawn(CSE_Abstract* DC);
    void OnChangeVisual() override;
    bool IsThermalCorpse() override { return m_painter_thermal_corpse; }
    float GetHotness() override;
    void PainterInheritThermal(CPhysicsShellHolder& source);
    void PainterRopeThermal(CPhysicsShellHolder& host); // thermal3: a gut rope (painter_gut_spill::end_spawn)
    u32 PainterThermalDeathTime() const { return m_painter_thermal_death_time; }
    void PainterArmPoseChild(CBoneInstance* b) { painter_arm_pose_child(m_painter_arm_pose,b); }
    u32 PainterArmPoseStatus() const { return painter_arm_pose_result(m_painter_arm_pose); }
	virtual void net_Destroy();
	virtual void Load(LPCSTR section);
	virtual void UpdateCL(); // Called each frame, so no need for dt
	void Hit(SHit* hit) override;
	void ApplyPainterPieceContinuation();
	// piecehit: whether a bullet that struck bone `bone` at `range` along the ray (start, unit dir) meets this piece's own flesh
	// inside the struck shape - 0 not a painter piece (no rule), 1 flesh there, 2 none (the bullet flies on)
	int PainterPieceFleshOnRay(const Fvector& start, const Fvector& dir, float range, u16 bone);
	virtual void shedule_Update(u32 dt); //
	virtual void net_Save(NET_Packet& P);
	virtual BOOL net_SaveRelevant();
	virtual BOOL UsedAI_Locations();
protected:
	virtual CPhysicsShellHolder* PPhysicsShellHolder() { return PhysicsShellHolder(); }
	virtual CPHSkeleton* PHSkeleton() { return this; }
	virtual void SpawnInitPhysics(CSE_Abstract* D);
	virtual void PHObjectPositionUpdate();
	virtual void CreatePhysicsShell(CSE_Abstract* e);
};


#endif
