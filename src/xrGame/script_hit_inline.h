////////////////////////////////////////////////////////////////////////////
//	Module 		: script_hit_inline.h
//	Created 	: 06.02.2004
//  Modified 	: 24.06.2004
//	Author		: Dmitriy Iassenev
//	Description : XRay Script hit class inline functions
////////////////////////////////////////////////////////////////////////////

#pragma once
#include "Hit.h"
#include "GameObject.h"

IC CScriptHit::CScriptHit()
{
	m_fPower = 100;
	m_tDirection.set(1, 0, 0);
	m_caBoneName = "";
	m_tpDraftsman = 0;
	m_fImpulse = 100;
	m_tpWeaponID = 0;
	m_tHitType = ALife::eHitTypeWound;
	bulletId = u32(-1);
}

IC CScriptHit::CScriptHit(const CScriptHit* tpLuaHit)
{
	*this = *tpLuaHit;
	// A script-created copy is not the active native receiver callback.
	m_painter_hit_origin=nullptr;
	m_painter_original_draftsman=nullptr;
	m_painter_armor_penetration=-1;
}

IC CScriptHit::CScriptHit(const SHit* tpHit)
{
	m_fPower = tpHit->power;
	m_tDirection = tpHit->direction();
	m_tpDraftsman = smart_cast<const CGameObject*>(tpHit->who)->lua_game_object();
	m_fImpulse = tpHit->impulse;
	m_tpWeaponID = tpHit->weaponID;
	m_tHitType = tpHit->hit_type;
	bulletId = tpHit->BulletID;
	m_ammo = tpHit->ammo;
	m_painter_hit_origin=tpHit;
	m_painter_original_draftsman=m_tpDraftsman;
	m_painter_armor_penetration=tpHit->painter_armor_penetration;
}

IC int CScriptHit::painter_armor_penetration_for(const SHit* hit) const
{
	if(m_painter_hit_origin!=hit || !m_ammo.valid() || !hit->ammo.valid() ||
		bulletId!=hit->BulletID || m_tpWeaponID!=hit->weaponID ||
		m_tpDraftsman!=m_painter_original_draftsman ||
		m_tHitType!=ALife::eHitTypeFireWound || hit->hit_type!=ALife::eHitTypeFireWound) return -1;
	return m_painter_armor_penetration==0 || m_painter_armor_penetration==1?m_painter_armor_penetration:-1;
}

IC LPCSTR CScriptHit::get_bone_name() const
{
	return m_caBoneName.c_str();
}

IC void CScriptHit::set_bone_name(LPCSTR bone_name)
{
	m_caBoneName = bone_name;
}
