////////////////////////////////////////////////////////////////////////////
//	Module 		: script_hit.h
//	Created 	: 06.02.2004
//  Modified 	: 24.06.2004
//	Author		: Dmitriy Iassenev
//	Description : XRay Script hit class
////////////////////////////////////////////////////////////////////////////

#pragma once

#include "alife_space.h"
#include "painter_hit_ammo.h"
#include "script_export_space.h"

struct SHit;
class CScriptGameObject;

class CScriptHit
{
public:
	float m_fPower;
	Fvector m_tDirection;
	shared_str m_caBoneName;
	CScriptGameObject* m_tpDraftsman;
	float m_fImpulse;
	int m_tHitType;
	u16 m_tpWeaponID;
	u32 bulletId;
	PainterHitAmmo m_ammo;

public:
	IC CScriptHit();
	IC CScriptHit(const CScriptHit* tpLuaHit);
	IC CScriptHit(const SHit* tpHit);
	virtual ~CScriptHit();
	IC LPCSTR get_bone_name() const;
	LPCSTR get_ammo_section() const { return m_ammo.valid()?m_ammo.section:""; }
	u32 get_ammo_pellet_count() const { return m_ammo.valid()?m_ammo.pellet_count:0; }
	float get_ammo_armor_piercing() const { return m_ammo.valid()?m_ammo.armor_piercing:0.f; }
	float get_ammo_impact_power() const { return m_ammo.valid()?m_ammo.impact_power:0.f; }
	bool get_ammo_provenance_valid() const { return m_ammo.valid(); }
	IC void set_bone_name(LPCSTR bone_name);
	void set_painter_armor_penetrated(bool value) {
		if(m_painter_hit_origin && m_ammo.valid() && m_tHitType==ALife::eHitTypeFireWound)
			m_painter_armor_penetration=value?1:0;
	}
	IC int painter_armor_penetration_for(const SHit* hit) const;
private:
	const SHit* m_painter_hit_origin=nullptr;
	CScriptGameObject* m_painter_original_draftsman=nullptr;
	int m_painter_armor_penetration=-1;
public:
DECLARE_SCRIPT_REGISTER_FUNCTION
};

#include "script_hit_inline.h"
