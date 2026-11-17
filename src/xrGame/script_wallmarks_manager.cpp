#pragma once
#include "stdafx.h"
#include "script_wallmarks_manager.h"
#include "Level.h"
#include "material_manager.h"
#include "Entity.h"
#include "PhysicsSkeletonObject.h"
#include <cmath>
#include "../Include/xrRender/RenderVisual.h"
#include "BloodOverlap.cpp"

ScriptWallmarksArray::ScriptWallmarksArray(LPCSTR section)
{
	m_wallmarks = xr_new<FactoryPtr<IWallMarkArray>>();
	m_section = section;

	R_ASSERT2(pSettings->section_exist(section), make_string("[ScriptWallmarksManager] Can't find section '%s'", section));
	LPCSTR wallmarks_string = READ_IF_EXISTS(pSettings, r_string, section, "wallmarks", nullptr);
	R_ASSERT2(wallmarks_string, make_string("[ScriptWallmarksManager] Can't find 'wallmarks' in section '%s'", section));

	string256 tmp;
	int cnt = _GetItemCount(wallmarks_string);

	for (int k = 0; k < cnt; ++k)
		(*m_wallmarks)->AppendMark(_GetItem(wallmarks_string, k, tmp));
}

ScriptWallmarksManager::ScriptWallmarksManager() {};

ScriptWallmarksManager::~ScriptWallmarksManager()
{
	m_section_index.clear();
	for (auto i : m_script_wallmarks)
	{
		xr_delete(i->m_wallmarks);
		xr_delete(i);
	}
};

IWallMarkArray* ScriptWallmarksManager::FindSection(LPCSTR section)
{
	const std::string_view key(section);
	const auto found = m_section_index.find(key);
	if (found != m_section_index.end())
		return &**found->second->m_wallmarks;

	ScriptWallmarksArray* wm = xr_new<ScriptWallmarksArray>(section);
	m_script_wallmarks.push_back(wm);
	m_section_index.emplace(std::string_view(wm->m_section.c_str()), wm);
	return &**wm->m_wallmarks;
}

void ScriptWallmarksManager::PlaceWallmark(Fvector dir, Fvector start_pos,
	float trace_dist, float wallmark_size, LPCSTR section,
	CScriptGameObject* ignore_obj, float ttl)
{
	PlaceWallmark(dir, start_pos, trace_dist, wallmark_size, section, ignore_obj, ttl, true);
}

// demonized: add user defined rotation to wallmark
void ScriptWallmarksManager::PlaceWallmark(Fvector dir, Fvector start_pos,
	float trace_dist, float wallmark_size, LPCSTR section,
	CScriptGameObject* ignore_obj, float ttl, bool random_rotation)
{
	PlaceWallmark(dir, start_pos, trace_dist, wallmark_size, section, ignore_obj, ttl, random_rotation ? ::Random.randF(-20.f, 20.f) : 0.f);
}

void ScriptWallmarksManager::PlaceWallmark(Fvector dir, Fvector start_pos,
	float trace_dist, float wallmark_size, LPCSTR section,
	CScriptGameObject* ignore_obj, float ttl, float rotation)
{
    PlaceWallmarkImpl(dir,start_pos,trace_dist,wallmark_size,section,ignore_obj,ttl,rotation,nullptr);
}

void ScriptWallmarksManager::PlaceBloodWallmark(Fvector dir,Fvector start_pos,float trace_dist,float wallmark_size,
    LPCSTR section,CScriptGameObject* ignore_obj,float ttl,float rotation,CScriptGameObject* source,double blood_birth)
{
    const auto stamp=BloodStamp(source,blood_birth);
    PlaceWallmarkImpl(dir,start_pos,trace_dist,wallmark_size,section,ignore_obj,ttl,rotation,&stamp);
}

void ScriptWallmarksManager::PlaceBloodStaticWallmark(Fvector dir,Fvector start_pos,float trace_dist,float wallmark_size,
    LPCSTR section,CScriptGameObject* ignore_obj,float ttl,float rotation,CScriptGameObject* source,double blood_birth)
{
    const auto stamp=BloodStamp(source,blood_birth);
    PlaceWallmarkImpl(dir,start_pos,trace_dist,wallmark_size,section,ignore_obj,ttl,rotation,&stamp,true);
}

void ScriptWallmarksManager::PlaceWallmarkImpl(Fvector dir,Fvector start_pos,float trace_dist,float wallmark_size,
    LPCSTR section,CScriptGameObject* ignore_obj,float ttl,float rotation,const BloodThermalStamp* stamp,bool world_only)
{
	collide::rq_result result;
	BOOL reach_wall =
		Level().ObjectSpace.RayPick(
			start_pos,
			dir,
			trace_dist,
			world_only ? collide::rqtStatic : collide::rqtBoth,
			result,
			ignore_obj ? &ignore_obj->object() : nullptr
		)
		&&
		!result.O;

	if (reach_wall)
	{
		CDB::TRI* pTri = Level().ObjectSpace.GetStaticTris() + result.element;
		SGameMtl* pMaterial = GMLib.GetMaterialByIdx(pTri->material);

		if (!pMaterial->Flags.is(SGameMtl::flSuppressWallmarks))
		{
			Fvector* pVerts = Level().ObjectSpace.GetStaticVerts();
			Fvector end_point;
			end_point.set(0, 0, 0);
			end_point.mad(start_pos, dir, result.range);

            if(stamp)::Render->add_BloodWallmark(FindSection(section),end_point,wallmark_size,pTri,pVerts,ttl,true,rotation,*stamp);
            else ::Render->add_StaticWallmark(FindSection(section), end_point, wallmark_size, pTri, pVerts, ttl, true, rotation);
		}
	}
}

BloodThermalStamp ScriptWallmarksManager::BloodStamp(CScriptGameObject* source,double blood_birth)
{
    BloodThermalStamp stamp;
    stamp.effect_id=BloodEffect::Decal; // Dedicated blood-only API is the authored registration.
    const u32 now=Device.dwTimeGlobal;
    // Malformed optional thermal metadata cannot suppress normal blood placement.
    if(std::isfinite(blood_birth) && blood_birth>=0. && blood_birth<=4294967295. && std::floor(blood_birth)==blood_birth) {
        const u32 born=static_cast<u32>(blood_birth);
        if(u32(now-born)<=0x7fffffffu){stamp.birth_known=true;stamp.blood_birth=born;}
    }
    if(source && source->is_valid()) {
        if(auto* entity=smart_cast<CEntity*>(&source->object())) {
            stamp.source_known=true;stamp.source_corpse=entity->IsThermalCorpse();
            if(stamp.source_corpse)stamp.source_death=entity->GetLevelDeathTime();
        } else if(auto* piece=smart_cast<CPhysicsSkeletonObject*>(&source->object())) {
            if(piece->IsThermalCorpse()) {
                stamp.source_known=true;stamp.source_corpse=true;stamp.source_death=piece->PainterThermalDeathTime();
            }
        }
    }
    return stamp;
}

bool ScriptWallmarksManager::PlaceBloodSkeletonSurface(CScriptGameObject* obj,LPCSTR section,
    Fvector contact,Fvector normal,float size,float ttl,CScriptGameObject* source,double blood_birth)
{
    if(!obj || !section || !section[0])return false;
    auto& object=obj->object();auto* visual=object.Visual();
    auto* k=visual?visual->dcast_PKinematics():nullptr;const auto stamp=BloodStamp(source,blood_birth);
    return k && k->LL_AddBloodSurfaceWallmark(&object.renderable.xform,FindSection(section),contact,normal,size,ttl,stamp);
}

bool ScriptWallmarksManager::PlaceBloodSkeletonSurfacePlaced(CScriptGameObject* obj,LPCSTR section,
    Fvector contact,Fvector normal,float size,float ttl,CScriptGameObject* source,double blood_birth,float depth,float roll)
{
    if(!obj || !section || !section[0])return false;
    auto& object=obj->object();auto* visual=object.Visual();
    auto* k=visual?visual->dcast_PKinematics():nullptr;const auto stamp=BloodStamp(source,blood_birth);
    return k && k->LL_AddBloodSurfaceWallmarkPlaced(&object.renderable.xform,FindSection(section),contact,normal,size,ttl,stamp,depth,roll);
}

bool ScriptWallmarksManager::PlaceBloodSkeletonSoak(CScriptGameObject* obj,LPCSTR section,
    Fvector contact,Fvector normal,float size,float ttl,float opacity,float seconds,CScriptGameObject* source,double blood_birth)
{
    if(!obj || !section || !section[0])return false;
    auto& object=obj->object();auto* visual=object.Visual();
    auto* k=visual?visual->dcast_PKinematics():nullptr;auto stamp=BloodStamp(source,blood_birth);
    stamp.effect_id=BloodEffect::Film;
    return k && k->LL_AddBloodSoakingWallmark(&object.renderable.xform,FindSection(section),contact,normal,size,ttl,opacity,seconds,stamp);
}

void ScriptWallmarksManager::PlaceBloodSkeletonWallmark(CScriptGameObject* obj,LPCSTR section,
    Fvector start,Fvector dir,float size,float ttl,CScriptGameObject* source,double blood_birth)
{
    if(!obj){Msg("[ScriptWallmarksManager] object is null!");return;}
    CObject* object=&obj->object();const auto stamp=BloodStamp(source,blood_birth);
    auto* blood_visual=object->Visual();auto* blood_kinematics=blood_visual?blood_visual->dcast_PKinematics():nullptr;
    if(!blood_kinematics||!section||!section[0])return;
    Render->add_BloodSkeletonWallmark(&object->renderable.xform,blood_kinematics,FindSection(section),
        start,dir,size,ttl,true,stamp);
}

// Dedicated cut-end entrypoints preserve the ordinary thermal effect and
// carry explicit size policy through the immutable per-emission stamp.
bool ScriptWallmarksManager::PlaceBladeCutSurface(CScriptGameObject* obj,LPCSTR section,
    Fvector contact,Fvector normal,float size,float ttl,CScriptGameObject* source,double blood_birth,
    float depth,float roll,u32 cut_id,u32 blow)
{
    if(!obj || !section || !section[0] || !cut_id || !blow)return false;
    auto& object=obj->object();auto* visual=object.Visual();
    auto* k=visual?visual->dcast_PKinematics():nullptr;if(!k)return false;
    auto stamp=BloodStamp(source,blood_birth);stamp.mark_key=cut_id;stamp.mark_revision=blow;
    return k->LL_AddBloodSurfaceWallmarkPlaced(&object.renderable.xform,FindSection(section),contact,normal,size,ttl,stamp,depth,roll);
}

bool ScriptWallmarksManager::PlaceSeverSplashSurface(CScriptGameObject* obj,LPCSTR section,
    Fvector contact,Fvector normal,float size,float ttl,CScriptGameObject* source,double blood_birth)
{
    if(!obj || !section || !section[0])return false;
    auto& object=obj->object();auto* visual=object.Visual();
    auto* k=visual?visual->dcast_PKinematics():nullptr;auto stamp=BloodStamp(source,blood_birth);
    stamp.sever_splash=true;
    return k && k->LL_AddBloodSurfaceWallmark(&object.renderable.xform,FindSection(section),contact,normal,size,ttl,stamp);
}
bool ScriptWallmarksManager::PlaceSeverSplashSurfacePlaced(CScriptGameObject* obj,LPCSTR section,
    Fvector contact,Fvector normal,float size,float ttl,CScriptGameObject* source,double blood_birth,float depth,float roll)
{
    if(!obj || !section || !section[0])return false;
    auto& object=obj->object();auto* visual=object.Visual();
    auto* k=visual?visual->dcast_PKinematics():nullptr;auto stamp=BloodStamp(source,blood_birth);
    stamp.sever_splash=true;
    return k && k->LL_AddBloodSurfaceWallmarkPlaced(&object.renderable.xform,FindSection(section),contact,normal,size,ttl,stamp,depth,roll);
}
void ScriptWallmarksManager::PlaceSeverSplashWallmark(CScriptGameObject* obj,LPCSTR section,
    Fvector start,Fvector dir,float size,float ttl,CScriptGameObject* source,double blood_birth)
{
    if(!obj || !section || !section[0])return;
    auto& object=obj->object();auto* visual=object.Visual();
    auto* k=visual?visual->dcast_PKinematics():nullptr;if(!k)return;
    auto stamp=BloodStamp(source,blood_birth);stamp.sever_splash=true;
    Render->add_BloodSkeletonWallmark(&object.renderable.xform,k,FindSection(section),start,dir,size,ttl,true,stamp);
}

bool ScriptWallmarksManager::PlaceSkeletonSurface(CScriptGameObject* obj, LPCSTR section,
    Fvector contact, Fvector normal, float size, float ttl)
{
    if(!obj || !section || !section[0])return false;
    auto& object=obj->object();auto* visual=object.Visual();
    auto* k=visual?visual->dcast_PKinematics():nullptr;
    return k && k->LL_AddSurfaceWallmark(&object.renderable.xform,FindSection(section),contact,normal,size,ttl);
}

bool ScriptWallmarksManager::PlaceSkeletonSoak(CScriptGameObject* obj, LPCSTR section,
    Fvector contact, Fvector normal, float size, float ttl, float opacity, float growth_seconds)
{
    if (!obj || !section || !section[0]) return false;
    auto& object=obj->object(); auto* visual=object.Visual();
    auto* k=visual?visual->dcast_PKinematics():nullptr;
    return k && k->LL_AddSoakingWallmark(&object.renderable.xform,FindSection(section),contact,normal,size,ttl,opacity,growth_seconds);
}

void ScriptWallmarksManager::PlaceSkeletonWallmark(CScriptGameObject* obj, LPCSTR section, 
	Fvector start, Fvector dir, float size, float ttl)
{
	if (!obj)
	{
		Msg("[ScriptWallmarksManager] object is null!");
		return;
	}

	CObject* object = &obj->object();
	// bloodfix 2026-11-13: an object without a skeleton visual takes no mark instead of a null dereference
	auto* mark_visual = object->Visual();
	auto* mark_kinematics = mark_visual ? mark_visual->dcast_PKinematics() : nullptr;
	if (!mark_kinematics || !section || !section[0]) return;
	Render->add_SkeletonWallmark(&object->renderable.xform, mark_kinematics, FindSection(section), start,
		dir, size, ttl, true);
}

