#pragma once
#include <string_view>
#include "script_export_space.h"
#include "script_game_object.h"
#include "../Include/xrRender/BloodThermalStamp.h"

struct ScriptWallmarksArray
{
	FactoryPtr<IWallMarkArray>* m_wallmarks;
	shared_str m_section; // own copy: the Lua string passed in is freed by GC

	ScriptWallmarksArray(LPCSTR section);
};

class ScriptWallmarksManager
{
private:
    BloodThermalStamp BloodStamp(CScriptGameObject*,double);
    void PlaceWallmarkImpl(Fvector dir,Fvector start_pos,float trace_dist,float size,LPCSTR section,
        CScriptGameObject* ignore_obj,float ttl,float rotation,const BloodThermalStamp* stamp,bool world_only=false);
	xr_vector<ScriptWallmarksArray*> m_script_wallmarks;
	// Views refer to each array's immutable owned name; the vector owns arrays.
	xr_unordered_map<std::string_view, ScriptWallmarksArray*> m_section_index;

public:
	ScriptWallmarksManager();
	~ScriptWallmarksManager();

	IWallMarkArray* FindSection(LPCSTR section);
	void PlaceWallmark(Fvector dir, Fvector start_pos,
		float trace_dist, float wallmark_size, LPCSTR section,
		CScriptGameObject* ignore_obj, float ttl);
	void PlaceWallmark(Fvector dir, Fvector start_pos,
		float trace_dist, float wallmark_size, LPCSTR section,
		CScriptGameObject* ignore_obj, float ttl, bool random_rotation = true);
	void PlaceWallmark(Fvector dir, Fvector start_pos,
		float trace_dist, float wallmark_size, LPCSTR section,
		CScriptGameObject* ignore_obj, float ttl, float rotation);
    void PlaceBloodWallmark(Fvector dir,Fvector start_pos,float trace_dist,float size,LPCSTR section,
        CScriptGameObject* ignore_obj,float ttl,float rotation,CScriptGameObject* source,double blood_birth);
    void PlaceBloodStaticWallmark(Fvector dir,Fvector start_pos,float trace_dist,float size,LPCSTR section,
        CScriptGameObject* ignore_obj,float ttl,float rotation,CScriptGameObject* source,double blood_birth);
    bool PlaceBloodSkeletonSurface(CScriptGameObject*,LPCSTR,Fvector,Fvector,float,float,CScriptGameObject*,double);
    // The same + depth limit (m; <= 0 = the 2-4 cm rule; clamped to 1-25 cm) + roll (degrees; NaN = random +-20).
    bool PlaceBloodSkeletonSurfacePlaced(CScriptGameObject*,LPCSTR,Fvector,Fvector,float,float,CScriptGameObject*,double,float,float);
    int SkeletonSurfacePlacementVersion() const { return 1; }
    bool PlaceBladeCutSurface(CScriptGameObject*,LPCSTR,Fvector,Fvector,float,float,CScriptGameObject*,double,float,float,u32,u32);
    bool PlaceSeverSplashSurface(CScriptGameObject*,LPCSTR,Fvector,Fvector,float,float,CScriptGameObject*,double);
    bool PlaceSeverSplashSurfacePlaced(CScriptGameObject*,LPCSTR,Fvector,Fvector,float,float,CScriptGameObject*,double,float,float);
    void PlaceSeverSplashWallmark(CScriptGameObject*,LPCSTR,Fvector,Fvector,float,float,CScriptGameObject*,double);
    bool PlaceBloodSkeletonSoak(CScriptGameObject*,LPCSTR,Fvector,Fvector,float,float,float,float,CScriptGameObject*,double);
    void PlaceBloodSkeletonWallmark(CScriptGameObject*,LPCSTR,Fvector,Fvector,float,float,CScriptGameObject*,double);
    bool PlaceSkeletonSurface(CScriptGameObject* obj, LPCSTR section, Fvector contact, Fvector normal, float size, float ttl);
    int WoundSurfaceApiVersion() const { return 1; }
    bool PlaceSkeletonSoak(CScriptGameObject* obj, LPCSTR section, Fvector contact, Fvector normal, float size, float ttl, float opacity, float growth_seconds);
	void PlaceSkeletonWallmark(CScriptGameObject* obj, LPCSTR section, Fvector start,
		Fvector dir, float size, float ttl);
};

typedef class_exporter<ScriptWallmarksManager> CScriptWallmarksManager;
