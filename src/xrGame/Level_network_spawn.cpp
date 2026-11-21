#include "pch_script.h"
#include "xrServer_Objects_ALife_All.h"
#include "level.h"
#include "game_cl_base.h"
#include "net_queue.h"
#include "ai_space.h"
#include "game_level_cross_table.h"
#include "level_graph.h"
#include "client_spawn_manager.h"
#include "../xrEngine/xr_object.h"
#include "../xrEngine/IGame_Persistent.h"
#include "../xrCore/spawn_cost.h"
#include "../xrCore/engine_error_logger.h"

// EngineErrorLogger::ObjectDescriber: the object being spawned, as the server entity names it.
static void describe_spawn(const void* entity, char* output, size_t capacity)
{
	const CSE_Abstract* E = static_cast<const CSE_Abstract*>(entity);
	LPCSTR section = E->s_name.c_str();
	_snprintf_s(output, capacity, _TRUNCATE, "%s [%s] id %u", E->name_replace() ? E->name_replace() : "<no name>",
		section ? section : "<no section>", u32(E->ID));
}

// Log in-game spawns ([SpawnTime]) and weapon HUD loads ([HudTime]) that held the main thread at
// least this many ms, 0 = off (default: a diagnostic, not for normal play).
int spawn_time_log = 0;
extern ENGINE_API BOOL g_bootComplete;
#include "../xrEngine/painter_spawn_timing.h"
#include "../Layers/xrRender/BloodNativeProfile.h"

namespace
{
// Spawn attribution (R11), reported only while the opt-in performance capture runs. A slow object prints one line;
// a frame whose spawns add up past the frame threshold prints a summary at the start of the next level frame, so a
// squad arriving one NPC per frame reads differently from a squad created inside a single frame.
constexpr double painter_slow_spawn_ms = 2.0;
constexpr double painter_slow_spawn_frame_ms = 4.0;
struct PainterSpawnFrame
{
	u32 frame = u32(-1);
	u32 objects = 0;
	double total_ms = 0.0;
	double worst_ms = 0.0;
	shared_str worst;
};
PainterSpawnFrame painter_spawn_frame;
unsigned painter_spawn_depth = 0;

struct PainterSpawnReport
{
	CSE_Abstract* entity;
	const bool timing;
	const double start;
	const painter_spawn_timing::Counters saved;
	double created = -1.0;
	double spawned = -1.0;

	explicit PainterSpawnReport(CSE_Abstract* E)
		: entity(E), timing(painter_spawn_depth == 0 && BloodNativeProfile::Instance().Capturing()),
		  start(painter_spawn_timing::NowMs()), saved(painter_spawn_timing::Current())
	{
		painter_spawn_timing::Current() = painter_spawn_timing::Counters{};
		++painter_spawn_depth;
	}
	~PainterSpawnReport()
	{
		auto& counters = painter_spawn_timing::Current();
		const painter_spawn_timing::Counters own = counters;
		counters.visual_ms = saved.visual_ms + own.visual_ms; // an enclosing spawn still accounts nested work
		counters.script_ms = saved.script_ms + own.script_ms;
		--painter_spawn_depth;
		if (!timing) return;
		const double end = painter_spawn_timing::NowMs();
		const double total = end - start;
		const double create = created >= 0.0 ? created - start : total;
		const double net_spawn = created >= 0.0 && spawned >= 0.0 ? spawned - created : 0.0;
		const double after = spawned >= 0.0 ? end - spawned : 0.0;
		auto& frame = painter_spawn_frame;
		if (frame.frame != Device.dwFrame)
		{
			if (frame.frame != u32(-1) && frame.total_ms >= painter_slow_spawn_frame_ms)
				Msg("[spawn_frame] frame=%u objects=%u total_ms=%.2f worst=%s worst_ms=%.2f", frame.frame, frame.objects,
				    frame.total_ms, frame.worst.size() ? frame.worst.c_str() : "", frame.worst_ms);
			frame = PainterSpawnFrame{};
			frame.frame = Device.dwFrame;
		}
		++frame.objects;
		frame.total_ms += total;
		if (total > frame.worst_ms)
		{
			frame.worst_ms = total;
			frame.worst = entity->s_name;
		}
		if (total >= painter_slow_spawn_ms)
			Msg("[spawn_time] frame=%u section=%s name=%s id=%u parent=%u total_ms=%.2f create_ms=%.2f net_spawn_ms=%.2f "
			    "visual_ms=%.2f script_ms=%.2f after_ms=%.2f", Device.dwFrame, entity->s_name.c_str(), entity->name_replace(),
			    unsigned(entity->ID), unsigned(entity->ID_Parent), total, create, net_spawn, own.visual_ms, own.script_ms, after);
	}
	PainterSpawnReport(const PainterSpawnReport&) = delete;
	PainterSpawnReport& operator=(const PainterSpawnReport&) = delete;
};
}

// Called at the start of every level frame: prints the previous frame's spawn summary during a capture.
void PainterSpawnFrameFlush(u32 current_frame)
{
	auto& frame = painter_spawn_frame;
	if (frame.frame == u32(-1) || frame.frame == current_frame) return;
	if (frame.total_ms >= painter_slow_spawn_frame_ms)
		Msg("[spawn_frame] frame=%u objects=%u total_ms=%.2f worst=%s worst_ms=%.2f", frame.frame, frame.objects, frame.total_ms,
		    frame.worst.size() ? frame.worst.c_str() : "", frame.worst_ms);
	frame = PainterSpawnFrame{};
}

void CLevel::cl_Process_Spawn(NET_Packet& P)
{

	shared_str s_name;
	P.r_stringZ(s_name);
	PROF_EVENT("CLevel::cl_Process_Spawn");
	spawn_cost::Owner spawn_timer("SpawnTime", s_name.c_str(), g_bootComplete ? spawn_time_log : 0);

	CSE_Abstract* E = nullptr;
	{

		E = F_entity_Create(*s_name);
	}
	R_ASSERT2(E, *s_name);

	{

	E->Spawn_Read(P);
	if (E->s_flags.is(M_SPAWN_UPDATE))
		E->UPDATE_Read(P);
	}

	bool configuration_matches = false;
	{

		configuration_matches = E->match_configuration();
	}
	if (!configuration_matches)
	{

		F_entity_Destroy(E);
		return;
	}
	//-------------------------------------------------
	//.	Msg ("M_SPAWN - %s[%d][%x] - %d %d", *s_name,  E->ID, E,E->ID_Parent, Device.dwFrame);
	//-------------------------------------------------
	//force object to be local for server client
	if (OnServer())
	{
		E->s_flags.set(M_SPAWN_OBJECT_LOCAL, TRUE);
	};

	/*
	game_spawn_queue.push_back(E);
	if (g_bDebugEvents)		ProcessGameSpawns();
	/*/
	{

	g_sv_Spawn(E);
	}

	{

	F_entity_Destroy(E);
	}
	//*/
};

void CLevel::g_cl_Spawn(LPCSTR name, u8 rp, u16 flags, Fvector pos)
{
	// Create
	CSE_Abstract* E = F_entity_Create(name);
	VERIFY(E);

	// Fill
	E->s_name = name;
	E->set_name_replace("");
	//.	E->s_gameid			=	u8(GameID());
	E->s_RP = rp;
	E->ID = 0xffff;
	E->ID_Parent = 0xffff;
	E->ID_Phantom = 0xffff;
	E->s_flags.assign(flags);
	E->RespawnTime = 0;
	E->o_Position = pos;

	// Send
	NET_Packet P;
	E->Spawn_Write(P,TRUE);
	Send(P, net_flags(TRUE));

	// Destroy
	F_entity_Destroy(E);
}

#ifdef DEBUG
	extern Flags32				psAI_Flags;
	extern float				debug_on_frame_gather_stats_frequency;
#	include "ai_debug.h"
#endif // DEBUG

void CLevel::g_sv_Spawn(CSE_Abstract* E)
{
#ifdef DEBUG_MEMORY_MANAGER
	size_t							E_mem = 0;
	if (g_bMEMO)	{
		lua_gc					(ai().script_engine().lua(),LUA_GCCOLLECT,0);
		lua_gc					(ai().script_engine().lua(),LUA_GCCOLLECT,0);
		E_mem					= Memory.mem_usage();	
		Memory.mem_calls_reset();
	}
#endif // DEBUG_MEMORY_MANAGER
	//-----------------------------------------------------------------
	//	CTimer		T(false);

#ifdef DEBUG
	//	Msg					("* CLIENT: Spawn: %s, ID=%d", *E->s_name, E->ID);
#endif

	// Optimization for single-player only	- minimize traffic between client and server
	if (GameID() == eGameIDSingle) psNET_Flags.set(NETFLAG_MINIMIZEUPDATES,TRUE);
	else psNET_Flags.set(NETFLAG_MINIMIZEUPDATES,FALSE);

	BloodNativeProfile::Scope painter_spawn_scope(BloodNativeProfile::Stage::SpawnObject, Device.dwFrame);
	PainterSpawnReport painter_spawn_report(E);

	// Client spawn
	//	T.Start		();
	const EngineErrorLogger::ObjectScope diagnostic_object("net_Spawn", E, &describe_spawn);
	CObject* O = nullptr;
	{

		O = Objects.Create(*E->s_name);
	}
	painter_spawn_report.created = painter_spawn_timing::NowMs();
	// Msg				("--spawn--CREATE: %f ms",1000.f*T.GetAsync());

	//	T.Start		();
#ifdef DEBUG_MEMORY_MANAGER
	mem_alloc_gather_stats		(false);
#endif // DEBUG_MEMORY_MANAGER

    if (0 == O)
    {
        Msg("! Failed to spawn entity '%s', O is nullptr", *E->s_name);
        return;
    }

	const BOOL painter_net_spawned = O->net_Spawn(E);
	painter_spawn_report.spawned = painter_spawn_timing::NowMs();
	if (!painter_net_spawned)
	{
        Msg("! Failed to spawn entity '%s', net_Spawn failed", *E->s_name);
		O->net_Destroy();
		if (!g_dedicated_server)
			client_spawn_manager().clear(O->ID());
		Objects.Destroy(O);
		Msg("! Failed to spawn entity '%s'", *E->s_name);

#ifdef DEBUG_MEMORY_MANAGER
		mem_alloc_gather_stats	(!!psAI_Flags.test(aiDebugOnFrameAllocs));
#endif // DEBUG_MEMORY_MANAGER

	}
	else
	{
		// until now a save started from a script net_spawn must skip this half-built object
		O->setReady(TRUE);
#ifdef DEBUG_MEMORY_MANAGER
		mem_alloc_gather_stats	(!!psAI_Flags.test(aiDebugOnFrameAllocs));
#endif // DEBUG_MEMORY_MANAGER

		if (!g_dedicated_server)
		{

			client_spawn_manager().callback(O);
		}
		//Msg			("--spawn--SPAWN: %f ms",1000.f*T.GetAsync());

		if ((E->s_flags.is(M_SPAWN_OBJECT_LOCAL)) &&
			(E->s_flags.is(M_SPAWN_OBJECT_ASPLAYER)))
		{
			if (IsDemoPlayStarted())
			{
				if (E->s_flags.is(M_SPAWN_OBJECT_PHANTOM))
				{
					SetControlEntity(O);
					SetEntity(O); //do not switch !!!
					SetDemoSpectator(O);
				}
			}
			else
			{
				if (CurrentEntity() != NULL)
				{
					CGameObject* pGO = smart_cast<CGameObject*>(CurrentEntity());
					if (pGO) pGO->On_B_NotCurrentEntity();
				}
				SetControlEntity(O);
				SetEntity(O); //do not switch !!!
			}
		}

		if (0xffff != E->ID_Parent)
		{
			/*
			// Generate ownership-event
			NET_Packet			GEN;
			GEN.w_begin			(M_EVENT);
			GEN.w_u32			(E->m_dwSpawnTime);//-NET_Latency);
			GEN.w_u16			(GE_OWNERSHIP_TAKE);
			GEN.w_u16			(E->ID_Parent);
			GEN.w_u16			(u16(O->ID()));
			game_events->insert	(GEN);
			/*/
			NET_Packet GEN;
			GEN.write_start();
			GEN.read_start();
			GEN.w_u16(u16(O->ID()));

			cl_Process_Event(E->ID_Parent, GE_OWNERSHIP_TAKE, GEN);
			//*/
		}

#ifdef NET_SPAWN_AFTER_CALLBACKS
		if (CGameObject* game_object = smart_cast<CGameObject*>(O))
        {

			game_object->callback(GameObject::eNetSpawnAfter)();
        }
#endif
	}

	/*if (E->s_flags.is(M_SPAWN_UPDATE)) {
		NET_Packet				temp;
		temp.B.count			= 0;
		E->UPDATE_Write			(temp);
		if (temp.B.count > 0)
		{
			temp.r_seek				(0);
			O->net_Import			(temp);
		}
		}*/ //:(

	//---------------------------------------------------------
	{

	Game().OnSpawn(O);
	}
	//---------------------------------------------------------

#ifdef DEBUG_MEMORY_MANAGER
	if (g_bMEMO) {
		lua_gc					(ai().script_engine().lua(),LUA_GCCOLLECT,0);
		lua_gc					(ai().script_engine().lua(),LUA_GCCOLLECT,0);
		Msg						("* %20s : %lld bytes, %u ops", *E->s_name,Memory.mem_usage()-E_mem, Memory.mem_calls_get() );
	}
#endif // DEBUG_MEMORY_MANAGER

}

CSE_Abstract* CLevel::spawn_item(LPCSTR section, const Fvector& position, u32 level_vertex_id, u16 parent_id,
                                 bool return_item)
{
	CSE_Abstract* abstract = F_entity_Create(section);
	R_ASSERT3(abstract, "Cannot find item with section", section);
	CSE_ALifeDynamicObject* dynamic_object = smart_cast<CSE_ALifeDynamicObject*>(abstract);
	if (dynamic_object && ai().get_level_graph())
	{
		dynamic_object->m_tNodeID = level_vertex_id;
		if (ai().level_graph().valid_vertex_id(level_vertex_id) && ai().get_game_graph() && ai().get_cross_table())
			dynamic_object->m_tGraphID = ai().cross_table().vertex(level_vertex_id).game_vertex_id();
	}

	//оружие спавним с полным магазинои
	CSE_ALifeItemWeapon* weapon = smart_cast<CSE_ALifeItemWeapon*>(abstract);
	if (weapon)
		weapon->a_elapsed = weapon->get_ammo_magsize();

	// Fill
	abstract->s_name = section;
	abstract->set_name_replace(section);
	//.	abstract->s_gameid		= u8(GameID());
	abstract->o_Position = position;
	abstract->s_RP = 0xff;
	abstract->ID = 0xffff;
	abstract->ID_Parent = parent_id;
	abstract->ID_Phantom = 0xffff;
	abstract->s_flags.assign(M_SPAWN_OBJECT_LOCAL);
	abstract->RespawnTime = 0;

	if (!return_item)
	{
		NET_Packet P;
		abstract->Spawn_Write(P,TRUE);
		Send(P, net_flags(TRUE));
		F_entity_Destroy(abstract);
		return (0);
	}
	else
		return (abstract);
}

void CLevel::ProcessGameSpawns()
{
	while (!game_spawn_queue.empty())
	{
		CSE_Abstract* E = game_spawn_queue.front();

		g_sv_Spawn(E);

		F_entity_Destroy(E);

		game_spawn_queue.pop_front();
	}
}
