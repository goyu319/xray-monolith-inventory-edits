////////////////////////////////////////////////////////////////////////////
//	Module 		: script_hit_script.cpp
//	Created 	: 06.02.2004
//  Modified 	: 24.06.2004
//	Author		: Dmitriy Iassenev
//	Description : XRay Script hit class script export
////////////////////////////////////////////////////////////////////////////

#include "pch_script.h"
#include "script_hit.h"
#include "script_game_object.h"

using namespace luabind;

#pragma optimize("s",on)
void CScriptHit::script_register(lua_State* L)
{
	module(L)
	[
		class_<CScriptHit>("hit")
		.enum_("hit_type")
		[
			value("burn", int(ALife::eHitTypeBurn)),
			value("shock", int(ALife::eHitTypeShock)),
			value("strike", int(ALife::eHitTypeStrike)),
			value("wound", int(ALife::eHitTypeWound)),
			value("radiation", int(ALife::eHitTypeRadiation)),
			value("telepatic", int(ALife::eHitTypeTelepatic)),
			value("chemical_burn", int(ALife::eHitTypeChemicalBurn)),
			value("explosion", int(ALife::eHitTypeExplosion)),
			value("fire_wound", int(ALife::eHitTypeFireWound)),
			value("light_burn", int(ALife::eHitTypeLightBurn)),
			value("dummy", int(ALife::eHitTypeMax))
		]
		.def_readwrite("power", &CScriptHit::m_fPower)
		.def_readwrite("direction", &CScriptHit::m_tDirection)
		.def_readwrite("draftsman", &CScriptHit::m_tpDraftsman)
		.def_readwrite("impulse", &CScriptHit::m_fImpulse)
		.def_readwrite("type", &CScriptHit::m_tHitType)
		.def_readwrite("weapon_id", &CScriptHit::m_tpWeaponID)
		.def_readonly("bullet_id", &CScriptHit::bulletId)
		.property("ammo_section", &CScriptHit::get_ammo_section)
		.property("ammo_pellet_count", &CScriptHit::get_ammo_pellet_count)
		.property("ammo_armor_piercing", &CScriptHit::get_ammo_armor_piercing)
		.property("ammo_impact_power", &CScriptHit::get_ammo_impact_power)
		.property("ammo_provenance_valid", &CScriptHit::get_ammo_provenance_valid)
		.def("set_painter_armor_penetrated", &CScriptHit::set_painter_armor_penetrated)
		.def(constructor<>())
		.def(constructor<const CScriptHit *>())
		.def("bone", &CScriptHit::set_bone_name)
		.property("bone", &CScriptHit::get_bone_name, &CScriptHit::set_bone_name)
	];
}
