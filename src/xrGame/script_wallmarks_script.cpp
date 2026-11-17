#include "pch_script.h"
#include "script_wallmarks_manager.h"
#include "BloodOverlap.h"

using namespace luabind;
extern bool painter_voxel_handles_wound(u16 id,u16 bone);

ScriptWallmarksManager* GetManager()
{
	return &g_pGamePersistent->GetWallmarksManager();
}

#pragma optimize("s",on)
void CScriptWallmarksManager::script_register(lua_State *L)
{
	RegisterBloodOverlap(L);
	module(L)
	[
		class_<ScriptWallmarksManager>("ScriptWallmarksManager")
		.def(constructor<>())
		.def("place", (void (ScriptWallmarksManager::*)(Fvector, Fvector, float, float, LPCSTR, CScriptGameObject*, float))(&ScriptWallmarksManager::PlaceWallmark))
		.def("place", (void (ScriptWallmarksManager::*)(Fvector, Fvector, float, float, LPCSTR, CScriptGameObject*, float, bool))(&ScriptWallmarksManager::PlaceWallmark))

		// demonized: add user defined rotation to wallmark
		.def("place", (void (ScriptWallmarksManager::*)(Fvector, Fvector, float, float, LPCSTR, CScriptGameObject*, float, float))(&ScriptWallmarksManager::PlaceWallmark))

        .def("place_blood", &ScriptWallmarksManager::PlaceBloodWallmark)
        .def("place_blood_static", &ScriptWallmarksManager::PlaceBloodStaticWallmark)
        .def("place_skeleton_blood_surface", &ScriptWallmarksManager::PlaceBloodSkeletonSurface)
        // 2026-11-12: + depth limit (m) and roll (degrees); the 8-argument form above is unchanged
        .def("place_skeleton_blood_surface", &ScriptWallmarksManager::PlaceBloodSkeletonSurfacePlaced)
        .def("skeleton_surface_placement_version", &ScriptWallmarksManager::SkeletonSurfacePlacementVersion)
        .def("place_skeleton_blood_soak", &ScriptWallmarksManager::PlaceBloodSkeletonSoak)
        .def("place_skeleton_blood", &ScriptWallmarksManager::PlaceBloodSkeletonWallmark)
        .def("place_skeleton_blade_cut", &ScriptWallmarksManager::PlaceBladeCutSurface)
        .def("place_skeleton_sever_splash", &ScriptWallmarksManager::PlaceSeverSplashWallmark)
        .def("place_skeleton_sever_splash_surface", &ScriptWallmarksManager::PlaceSeverSplashSurface)
        .def("place_skeleton_sever_splash_surface", &ScriptWallmarksManager::PlaceSeverSplashSurfacePlaced)
        .def("place_skeleton_surface", &ScriptWallmarksManager::PlaceSkeletonSurface)
        .def("wound_surface_api_version", &ScriptWallmarksManager::WoundSurfaceApiVersion)
        .def("place_skeleton_soak", &ScriptWallmarksManager::PlaceSkeletonSoak)
		.def("place_skeleton", &ScriptWallmarksManager::PlaceSkeletonWallmark),

		def("wallmarks_manager", &GetManager),
        def("painter_voxel_handles_wound", &painter_voxel_handles_wound)
	];
}
