////////////////////////////////////////////////////////////////////////////
//	Module 		: motivation_action_manager_stalker.cpp
//	Created 	: 26.03.2004
//  Modified 	: 26.03.2004
//	Author		: Dmitriy Iassenev
//	Description : Stalker motivation action manager class
////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "stalker_planner.h"
#include "stalker_property_evaluators.h"
#include "stalker_danger_property_evaluators.h"
#include "ai/stalker/ai_stalker.h"
#include "stalker_decision_space.h"
#include "script_game_object.h"
#include "stalker_alife_planner.h"
#include "stalker_anomaly_planner.h"
#include "stalker_death_planner.h"
#include "stalker_danger_planner.h"
#include "stalker_alife_actions.h"
#include "stalker_combat_planner.h"
#include "ai_space.h"
#include "../xrServerEntities/script_engine.h"
#include "painter_ai/EngineCombatAdapter.h"
#include "painter_ai/EngineCompat.h"
//#include "stalker_combat_planner_new.h"

//#define GOAP_DEBUG

using namespace StalkerDecisionSpace;

namespace
{
void record_stalker_brain_stage(void* context, LPCSTR stage, LPCSTR action_name,
	const u32 action_id, const u64 elapsed_ticks)
{
	CStalkerPlanner* planner = static_cast<CStalkerPlanner*>(context);
	if (!planner || !stage)
		return;

	CAI_Stalker& stalker = planner->object();
	string512 phase;
	if (action_name && action_name[0])
		_snprintf_s(phase, sizeof(phase), _TRUNCATE, "stalker.brain.%s:%s", stage, action_name);
	else
		_snprintf_s(phase, sizeof(phase), _TRUNCATE, "stalker.brain.%s", stage);

	ai().script_engine().record_native_profile("ai_brain", phase,
		*stalker.cName(), static_cast<u32>(stalker.ID()), elapsed_ticks, action_id);
}

class CStalkerBrainProfileScope : xray::noncopyable
{
public:
	CStalkerBrainProfileScope(CStalkerPlanner* planner, const bool enabled) :
		m_planner(enabled ? planner : nullptr)
	{
		if (m_planner)
			m_planner->set_update_profile_callback(m_planner, &record_stalker_brain_stage);
	}

	~CStalkerBrainProfileScope()
	{
		if (m_planner)
			m_planner->set_update_profile_callback(nullptr, nullptr);
	}

private:
	CStalkerPlanner* m_planner;
};
}

CStalkerPlanner::CStalkerPlanner()
{
	m_affect_cover = false;
}

CStalkerPlanner::~CStalkerPlanner()
{
}

#ifdef LOG_ACTION
LPCSTR CStalkerPlanner::action2string	(const _action_id_type &action_id)
{
	return					(inherited::action2string(action_id));
}

LPCSTR CStalkerPlanner::property2string	(const _condition_type &property_id)
{
	return					(inherited::property2string(property_id));
}

LPCSTR CStalkerPlanner::object_name		() const
{
	VERIFY					(m_object);
	return					(*m_object->cName());
}
#endif

void CStalkerPlanner::setup(CAI_Stalker* object)
{
#ifdef LOG_ACTION
	set_use_log					(!!psAI_Flags.test(aiGOAP));
#endif

	inherited::setup(object);

	clear();
	add_evaluators();
	add_actions();
	extern int g_goyu_painter_planner;
	if (g_goyu_painter_planner)
		painter::combat::engine::register_actor(*this, *object);

	CWorldState target;
	target.add_condition(CWorldProperty(eWorldPropertyPuzzleSolved, true));
	set_target_state(target);

	m_affect_cover = false;
}

// AI ownership (Painter AI): while the stalker's forced action id is set, he is alive and the action is registered in
// this planner, that action runs every update (finalize/initialize on a switch, execute each update) instead of the
// GOAP solution. Perception, memory, damage, death and the wounded scheme are untouched: the AI clears the id to hand
// the stalker back to his own planner.
bool CStalkerPlanner::painter_forced_update()
{
	u32 const forced = object().painter_forced_action();
	if (!forced || !object().g_Alive())
	{
		object().set_painter_gait(0);   // native17: a man the native action does not run walks and runs aimed, as stock
		return false;
	}
	auto const& ops = operators();
	auto const I = std::lower_bound(ops.begin(), ops.end(), forced);
	if (I == ops.end() || (*I).m_operator_id != forced)
		return false;
	// Every evaluator still runs (scripted evaluators keep bookkeeping, e.g. the wound manager updates inside the
	// wounded evaluator); the stalker goes back to his own planner in this update when the forced action's own
	// eligibility evaluator (same id) says no. The same single pass records his registered compatibility evaluators
	// (contract v2.21, no second evaluate()); only after the whole loop - the full-pass fence, a compatibility evaluator
	// may sort after the eligibility one - may the aggregate let the action run: a veto hands him back through the
	// adapter's relinquish (cleanup -> on_release) before anything executes.
	bool eligible = true;
	auto compat = painter::combat::engine::compat_pass_begin(object());
	for (auto const& e : evaluators())
	{
		bool const value = e.second->evaluate();
		if (e.first == forced)
			eligible = value;
		painter::combat::engine::compat_pass_record(compat, e.first, e.second, value);
	}
	if (painter::combat::engine::compat_pass_end(compat, object()))
		return false;
	if (!eligible)
		return false;
	if (!initialized())
	{
		m_initialized = true;
		m_current_action_id = forced;
		current_action().initialize();
	}
	else if (current_action_id() != forced)
	{
		current_action().finalize();
		m_current_action_id = forced;
		current_action().initialize();
	}
	current_action().execute();
	return true;
}

void CStalkerPlanner::update(u32 time_delta)
{
#ifdef LOG_ACTION
	if ((psAI_Flags.test(aiGOAP) && !m_use_log) || (!psAI_Flags.test(aiGOAP) && m_use_log))
		set_use_log			(!!psAI_Flags.test(aiGOAP));
#endif

	const bool record_brain = ai().script_engine().lua_recording() && CPU::qpc_freq;
	CStalkerBrainProfileScope profile_scope(this, record_brain);
	extern int g_goyu_painter_planner;
	if (!g_goyu_painter_planner || !painter_forced_update())
		inherited::update();

#ifdef GOAP_DEBUG
	if (m_failed) {
		{
			Msg			("%d",evaluators().size());
			EVALUATORS::const_iterator	I = evaluators().begin();
			EVALUATORS::const_iterator	E = evaluators().end();
			for ( ; I != E; ++I)
				Msg		("%d,%d",(*I).first,(*I).second->evaluate() ? 1 : 0);
		}
		{
			Msg			("%d",target_state().conditions().size());
			xr_vector<COperatorCondition>::const_iterator	I = target_state().conditions().begin();
			xr_vector<COperatorCondition>::const_iterator	E = target_state().conditions().end();
			for ( ; I != E; ++I)
				Msg		("%d,%d",(*I).condition(),(*I).value() ? 1 : 0);
		}
		{
			Msg			("%d",operators().size());
			const_iterator	I = operators().begin();
			const_iterator	E = operators().end();
			for ( ; I != E; ++I) {
				Msg		("%d,%d",(*I).m_operator_id,(*I).m_operator->weight(target_state(),target_state()));
				{
					Msg		("%d",(*I).m_operator->conditions().conditions().size());
					xr_vector<COperatorCondition>::const_iterator	i = (*I).m_operator->conditions().conditions().begin();
					xr_vector<COperatorCondition>::const_iterator	e = (*I).m_operator->conditions().conditions().end();
					for ( ; i != e; ++i)
						Msg	("%d,%d",(*i).condition(),(*i).value() ? 1 : 0);
				}
				{
					Msg		("%d",(*I).m_operator->effects().conditions().size());
					xr_vector<COperatorCondition>::const_iterator	i = (*I).m_operator->effects().conditions().begin();
					xr_vector<COperatorCondition>::const_iterator	e = (*I).m_operator->effects().conditions().end();
					for ( ; i != e; ++i)
						Msg	("%d,%d",(*i).condition(),(*i).value() ? 1 : 0);
				}
			}
		}
	}
#endif
}

void CStalkerPlanner::add_evaluators()
{
	add_evaluator(eWorldPropertyAlreadyDead, xr_new<CStalkerPropertyEvaluatorConst>(false, "is_already_dead"));
	add_evaluator(eWorldPropertyPuzzleSolved, xr_new<CStalkerPropertyEvaluatorConst>(false, "is_zone_puzzle_solved"));
	add_evaluator(eWorldPropertyAlive, xr_new<CStalkerPropertyEvaluatorAlive>(m_object, "is_alive"));
	add_evaluator(eWorldPropertyEnemy,
	              xr_new<CStalkerPropertyEvaluatorEnemies>(m_object, "is_there_enemies",
	                                                       CStalkerCombatPlanner::POST_COMBAT_WAIT_INTERVAL));
	add_evaluator(eWorldPropertyDanger, xr_new<CStalkerPropertyEvaluatorDangers>(m_object, "is_there_danger"));
	add_evaluator(eWorldPropertyAnomaly, xr_new<CStalkerPropertyEvaluatorAnomaly>(m_object, "is_there_anomalies"));
	add_evaluator(eWorldPropertyItems, xr_new<CStalkerPropertyEvaluatorItems>(m_object, "is_there_items_to_pick_up"));
}

void CStalkerPlanner::add_actions()
{
	CActionPlannerAction* planner;

	planner = xr_new<CStalkerDeathPlanner>(m_object, "death_planner");
	add_condition(planner, eWorldPropertyAlive, false);
	add_condition(planner, eWorldPropertyPuzzleSolved, false);
	add_effect(planner, eWorldPropertyPuzzleSolved, true);
	add_operator(eWorldOperatorDeathPlanner, planner);

	planner = xr_new<CStalkerALifePlanner>(m_object, "alife_planner");
	add_condition(planner, eWorldPropertyAlive, true);
	add_condition(planner, eWorldPropertyEnemy, false);
	add_condition(planner, eWorldPropertyAnomaly, false);
	add_condition(planner, eWorldPropertyDanger, false);
	add_condition(planner, eWorldPropertyItems, false);
	add_condition(planner, eWorldPropertyPuzzleSolved, false);
	add_effect(planner, eWorldPropertyPuzzleSolved, true);
	add_operator(eWorldOperatorALifePlanner, planner);

	planner = xr_new<CStalkerCombatPlanner>(m_object, "combat_planner");
	//	planner					= xr_new<CStalkerCombatPlannerNew>(m_object,"combat_planner_new");
	add_condition(planner, eWorldPropertyAlive, true);
	add_condition(planner, eWorldPropertyAnomaly, false);
	add_condition(planner, eWorldPropertyEnemy, true);
	add_effect(planner, eWorldPropertyEnemy, false);
	add_operator(eWorldOperatorCombatPlanner, planner);

	planner = xr_new<CStalkerDangerPlanner>(m_object, "danger_planner");
	add_condition(planner, eWorldPropertyAlive, true);
	add_condition(planner, eWorldPropertyEnemy, false);
	add_condition(planner, eWorldPropertyAnomaly, false);
	add_condition(planner, eWorldPropertyDanger, true);
	add_effect(planner, eWorldPropertyDanger, false);
	add_operator(eWorldOperatorDangerPlanner, planner);

	planner = xr_new<CStalkerAnomalyPlanner>(m_object, "anomaly_planner");
	add_condition(planner, eWorldPropertyAlive, true);
	add_condition(planner, eWorldPropertyAnomaly, true);
	add_effect(planner, eWorldPropertyAnomaly, false);
	add_operator(eWorldOperatorAnomalyPlanner, planner);

	CStalkerActionBase* action;

	action = xr_new<CStalkerActionGatherItems>(m_object, "gather_items");
	add_condition(action, eWorldPropertyAlive, true);
	add_condition(action, eWorldPropertyEnemy, false);
	add_condition(action, eWorldPropertyAnomaly, false);
	add_condition(action, eWorldPropertyDanger, false);
	add_condition(action, eWorldPropertyItems, true);
	add_effect(action, eWorldPropertyItems, false);
	add_operator(eWorldOperatorGatherItems, action);
}
