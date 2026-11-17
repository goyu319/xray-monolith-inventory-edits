////////////////////////////////////////////////////////////////////////////
//	Module 		: script_binder.cpp
//	Created 	: 26.03.2004
//  Modified 	: 26.03.2004
//	Author		: Dmitriy Iassenev
//	Description : Script objects binder
////////////////////////////////////////////////////////////////////////////

#include "pch_script.h"
#include "ai_space.h"
#include "script_engine.h"
#include "script_binder.h"
#include "../xrCore/spawn_cost.h"
#include "xrServer_Objects_ALife.h"
#include "script_binder_object_wrapper.h"
#include "script_game_object.h"
#include "gameobject.h"
#include "level.h"
#include "../xrEngine/painter_spawn_timing.h"

namespace
{
// A script binder call that fails drops the object's script binding (clear()). Say which call and which object first,
// so a broken binder is not silent. xrGame builds without C++ exception unwinding and luabind reports Lua errors through
// its error callback (already in the log just above this line); these catch blocks only see what escapes that.
void painter_binder_failure(CScriptBinder* binder, LPCSTR method)
{
	CGameObject* const object = smart_cast<CGameObject*>(binder);
	Msg("! [script_binder] %s failed on %s; script binding dropped", method, object ? object->cName().c_str() : "?");
}
} // namespace

// comment next string when commiting
//#define DBG_DISABLE_SCRIPTS

CScriptBinder::CScriptBinder()
{
	init();
}

CScriptBinder::~CScriptBinder()
{
	VERIFY(!m_object);
}

void CScriptBinder::init()
{
	m_object = 0;
}

void CScriptBinder::clear()
{
	try
	{
		xr_delete(m_object);
	}
	catch (...)
	{
		painter_binder_failure(this, "clear");
		m_object = 0;
	}
	init();
}

void CScriptBinder::reinit()
{
#ifdef DEBUG_MEMORY_MANAGER
	size_t									start = 0;
	if (g_bMEMO)
		start							= Memory.mem_usage();
#endif // DEBUG_MEMORY_MANAGER
	if (m_object)
	{
		try
		{
			m_object->reinit();
		}
		catch (...)
		{
			painter_binder_failure(this, "reinit");
			clear();
		}
	}
#ifdef DEBUG_MEMORY_MANAGER
	if (g_bMEMO) {
//		lua_gc				(ai().script_engine().lua(),LUA_GCCOLLECT,0);
//		lua_gc				(ai().script_engine().lua(),LUA_GCCOLLECT,0);
		Msg					("CScriptBinder::reinit() : %lld",Memory.mem_usage() - start);
	}
#endif // DEBUG_MEMORY_MANAGER
}

void CScriptBinder::Load(LPCSTR section)
{}

void CScriptBinder::reload(LPCSTR section)
{
	PROF_EVENT("CScriptBinder::reload");
#ifdef DEBUG_MEMORY_MANAGER
	size_t									start = 0;
	if (g_bMEMO)
		start							= Memory.mem_usage();
#endif // DEBUG_MEMORY_MANAGER
#ifndef DBG_DISABLE_SCRIPTS
	VERIFY(!m_object);
	if (!pSettings->line_exist(section, "script_binding"))
		return;

	::luabind::functor<void> lua_function;
	if (!ai().script_engine().functor(pSettings->r_string(section, "script_binding"), lua_function))
	{
		ai().script_engine().script_log(ScriptStorage::eLuaMessageTypeError, "function %s is not loaded!",
		                                pSettings->r_string(section, "script_binding"));
		return;
	}

	CGameObject* game_object = smart_cast<CGameObject*>(this);

	try
	{
		lua_function(game_object ? game_object->lua_game_object() : 0);
	}
	catch (...)
	{
		painter_binder_failure(this, "reload");
		clear();
		return;
	}

	if (m_object)
	{
		try
		{
			m_object->reload(section);
		}
		catch (...)
		{
			painter_binder_failure(this, "reload");
			clear();
		}
	}
#endif
#ifdef DEBUG_MEMORY_MANAGER
	if (g_bMEMO) {
//		lua_gc				(ai().script_engine().lua(),LUA_GCCOLLECT,0);
//		lua_gc				(ai().script_engine().lua(),LUA_GCCOLLECT,0);
		Msg					("CScriptBinder::reload() : %lld",Memory.mem_usage() - start);
	}
#endif // DEBUG_MEMORY_MANAGER
}

BOOL CScriptBinder::net_Spawn(CSE_Abstract* DC)
{
	PROF_EVENT("CScriptBinder::net_Spawn");
	painter_spawn_timing::Measure script_time(painter_spawn_timing::Current().script_ms);
#ifdef DEBUG_MEMORY_MANAGER
	size_t									start = 0;
	if (g_bMEMO)
		start							= Memory.mem_usage();
#endif // DEBUG_MEMORY_MANAGER
	CSE_Abstract* abstract = (CSE_Abstract*)DC;
	CSE_ALifeObject* object = smart_cast<CSE_ALifeObject*>(abstract);
	if (object && m_object)
	{
		try
		{
			spawn_cost::Scope cost(spawn_cost::Lua);
			return ((BOOL)m_object->net_Spawn(object));
		}
		catch (...)
		{
			painter_binder_failure(this, "net_Spawn");
			clear();
		}
	}

#ifdef DEBUG_MEMORY_MANAGER
	if (g_bMEMO) {
//		lua_gc				(ai().script_engine().lua(),LUA_GCCOLLECT,0);
//		lua_gc				(ai().script_engine().lua(),LUA_GCCOLLECT,0);
		Msg					("CScriptBinder::net_Spawn() : %lld",Memory.mem_usage() - start);
	}
#endif // DEBUG_MEMORY_MANAGER

	return (TRUE);
}

void CScriptBinder::net_Destroy()
{
	PROF_EVENT("CScriptBinder::net_Destroy");
	if (m_object)
	{
#ifdef _DEBUG
		Msg						("* Core object %s is UNbinded from the script object",smart_cast<CGameObject*>(this) ? *smart_cast<CGameObject*>(this)->cName() : "");
#endif // _DEBUG
		try
		{
			m_object->net_Destroy();
		}
		catch (...)
		{
			painter_binder_failure(this, "net_Destroy");
			clear();
		}
	}

	// Scripts may keep the binder after net_destroy (bind_campfire's campfires_all). Give an adopted Lua binder
	// back to Lua, as adopt_policy does, so its instance stays valid until collected instead of freed here
	if (auto wrapper = dynamic_cast<CScriptBinderObjectWrapper*>(m_object))
	{
		luabind::wrapped_self_t& self = luabind::detail::wrap_access::ref(*wrapper);
		if (self.m_strong_ref.is_valid())
		{
			lua_State* L = ai().script_engine().lua();
			self.get(L);
			if (luabind::detail::object_rep* rep = luabind::detail::is_class_object(L, -1))
			{
				rep->set_flags(rep->flags() | luabind::detail::object_rep::owner);
				luaJIT_setudatafinalizer(L, -1, 1);
				self.m_strong_ref.reset();
				wrapper->m_object = nullptr;
				m_object = nullptr;
			}
			lua_pop(L, 1);
		}
	}
	xr_delete(m_object);
}

void CScriptBinder::set_object(CScriptBinderObject* object)
{
	if (IsGameTypeSingle())
	{
		VERIFY2(!m_object, "Cannot bind to the object twice!");
#ifdef _DEBUG
		Msg					("* Core object %s is binded with the script object",smart_cast<CGameObject*>(this) ? *smart_cast<CGameObject*>(this)->cName() : "");
#endif // _DEBUG
		m_object = object;
	}
	else
	{
		xr_delete(object);
	}
}

void CScriptBinder::shedule_Update(u32 time_delta)
{
	PROF_EVENT("CScriptBinder::shedule_Update");
	if (m_object)
	{
		try
		{
			m_object->shedule_Update(time_delta);
		}
		catch (...)
		{
			painter_binder_failure(this, "shedule_Update");
			clear();
		}
	}
}

void CScriptBinder::save(NET_Packet& output_packet)
{
	PROF_EVENT("CScriptBinder::save");
	if (m_object)
	{
		try
		{
			m_object->save(&output_packet);
		}
		catch (...)
		{
			painter_binder_failure(this, "save");
			clear();
		}
	}
}

void CScriptBinder::load(IReader& input_packet)
{
	PROF_EVENT("CScriptBinder::load");
	if (m_object)
	{
		try
		{
			m_object->load(&input_packet);
		}
		catch (...)
		{
			painter_binder_failure(this, "load");
			clear();
		}
	}
}

BOOL CScriptBinder::net_SaveRelevant()
{
	if (m_object)
	{
		try
		{
			return (m_object->net_SaveRelevant());
		}
		catch (...)
		{
			painter_binder_failure(this, "net_SaveRelevant");
			clear();
		}
	}
	return (FALSE);
}

void CScriptBinder::net_Relcase(CObject* object)
{
	PROF_EVENT("CScriptBinder::net_Relcase");
	CGameObject* game_object = smart_cast<CGameObject*>(object);
	if (m_object && game_object)
	{
		try
		{
			m_object->net_Relcase(game_object->lua_game_object());
		}
		catch (...)
		{
			painter_binder_failure(this, "net_Relcase");
			clear();
		}
	}
}
