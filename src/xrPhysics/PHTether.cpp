#include "StdAfx.h"
#include "PHTether.h"
#include "Physics.h"
#include "PhysicsShell.h"
#include "PHElement.h"
#include "mathutilsode.h"
#include "../xrengine/device.h"
#include <cmath>

namespace
{
xr_vector<CPHTether*>& tether_registry()
{
	static xr_vector<CPHTether*> registry;
	return registry;
}
IPHTether* create_tether(CPhysicsShell* a,u16 bone_a,CPhysicsShell* b,u16 bone_b,const Fvector& anchor,
                        float break_force,u32 life_ms,bool exact)
{
    if(!a||!b||a==b||!std::isfinite(anchor.x)||!std::isfinite(anchor.y)||!std::isfinite(anchor.z)||
       !std::isfinite(break_force)||break_force<=0||!life_ms||life_ms>0x7fffffffu||
       (exact&&(bone_a==BI_NONE||bone_b==BI_NONE)))return NULL;
    CPHTether* t=xr_new<CPHTether>(a,bone_a,b,bone_b,anchor,break_force,life_ms,exact);
    if(!t->Alive()){xr_delete(t);return NULL;}return t;
}
}
void phtether_shell_gone(CPhysicsShell* s)
{
	if (!s)
		return;
	auto& registry = tether_registry();
	for (std::size_t i = 0; i < registry.size(); ++i)
		registry[i]->ShellGone(s);
}
IPHTether* phtether_create(CPhysicsShell* a, u16 bone_a, CPhysicsShell* b, u16 bone_b, const Fvector& anchor, float break_force,
                           u32 life_ms)
{
	return create_tether(a,bone_a,b,bone_b,anchor,break_force,life_ms,false);
}
IPHTether* phtether_create_exact(CPhysicsShell* a,u16 bone_a,CPhysicsShell* b,u16 bone_b,const Fvector& anchor,
                               float break_force,u32 life_ms)
{return create_tether(a,bone_a,b,bone_b,anchor,break_force,life_ms,true);}
void phtether_destroy(IPHTether*& t)
{
	CPHTether* tether = smart_cast<CPHTether*>(t);
	xr_delete(tether);
	t = 0;
}
CPHTether::CPHTether(CPhysicsShell* a, u16 bone_a, CPhysicsShell* b, u16 bone_b, const Fvector& anchor, float break_force,
                     u32 life_ms,bool exact_elements)
	: m_a(a), m_b(b), m_ea(NULL), m_eb(NULL), m_joint(NULL), m_break_force(break_force), m_born_at(Device.dwTimeGlobal),m_life_ms(life_ms)
{
	ZeroMemory(&m_feedback, sizeof(m_feedback));
	tether_registry().push_back(this);
	m_ea = a->get_Element(bone_a);
	m_eb = b->get_Element(bone_b);
	if (!m_ea&&!exact_elements)
		m_ea = a->get_ElementByStoreOrder(0);
	if (!m_eb&&!exact_elements)
		m_eb = b->get_ElementByStoreOrder(0);
	m_island.Init(); // the joint island must be initialised before AddJoint/Merge (as CPHCapture does)
	if (!m_ea || !m_eb || !m_ea->isFullActive() || !m_eb->isFullActive())
		return;
	dBodyID ba = static_cast<CPHElement*>(m_ea)->get_body();
	dBodyID bb = static_cast<CPHElement*>(m_eb)->get_body();
	if (!ba || !bb || ba==bb)
		return;
	m_joint = dJointCreateBall(0, 0);
	if(!m_joint)return;
	m_island.AddJoint(m_joint);
	dJointAttach(m_joint, ba, bb);
	dJointSetFeedback(m_joint, &m_feedback);
	dJointSetBallAnchor(m_joint, anchor.x, anchor.y, anchor.z);
	m_ea->Enable();
	m_eb->Enable();
	CPHUpdateObject::Activate();
}
CPHTether::~CPHTether()
{
	Release();
	Deactivate();
	auto& registry = tether_registry();
	registry.erase(std::remove(registry.begin(), registry.end(), this), registry.end());
}
void CPHTether::ShellGone(CPhysicsShell* s)
{
	const bool mine = s == m_a || s == m_b || (m_ea && m_ea->PhysicsShell() == s) || (m_eb && m_eb->PhysicsShell() == s);
	if (!mine)
		return;
	m_ea = NULL;
	m_eb = NULL;
	m_a = NULL;
	m_b = NULL;
	if (m_joint)
	{
		m_island.Unmerge();
		m_island.RemoveJoint(m_joint);
		dJointDestroy(m_joint);
		m_joint = NULL;
	}
	Deactivate();
}
void CPHTether::Release()
{
	if (!m_joint)
		return;
	m_island.Unmerge();
	m_island.RemoveJoint(m_joint);
	dJointDestroy(m_joint);
	m_joint = NULL;
	if (m_ea && m_ea->isFullActive())
		m_ea->Enable();
	if (m_eb && m_eb->isFullActive())
		m_eb->Enable();
	Deactivate();
}
void CPHTether::NetRelcase(CPhysicsShell* s)
{
	if (s == m_a || s == m_b)
	{
		if (s == m_a)
			m_ea = NULL;
		else
			m_eb = NULL;
		if (m_joint)
		{
			m_island.Unmerge();
			m_island.RemoveJoint(m_joint);
			dJointDestroy(m_joint);
			m_joint = NULL;
		}
		Deactivate();
	}
}
void CPHTether::PhTune(dReal /*step*/)
{
	if (!m_joint)
		return;
	if (!m_a || !m_b || !m_ea || !m_eb || !m_ea->isFullActive() || !m_eb->isFullActive() || !m_a->isActive() || !m_b->isActive() ||
		!m_ea->PhysicsShell() || !m_eb->PhysicsShell())
	{
		Release();
		return;
	}
	const bool act_a = m_ea->isEnabled(), act_b = m_eb->isEnabled();
	if (!act_a && !act_b)
		return;
	if (act_a)
		m_eb->Enable();
	if (act_b)
		m_ea->Enable();
	m_ea->PhysicsShell()->PIsland()->Merge(&m_island);
	m_eb->PhysicsShell()->PIsland()->Merge(&m_island);
}
void CPHTether::PhDataUpdate(dReal /*step*/)
{
	if (!m_joint)
		return;
	m_island.Unmerge();
	const float first=dDOT(m_feedback.f1,m_feedback.f1),second=dDOT(m_feedback.f2,m_feedback.f2);
	const float f2=_max(first,second);
	if (!std::isfinite(first)||!std::isfinite(second)||f2 > m_break_force * m_break_force ||
        u32(Device.dwTimeGlobal-m_born_at)>=m_life_ms)
		Release();
}
