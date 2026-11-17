////////////////////////////////////////////////////////////////////////////
//	Module 		: script_sound.cpp
//	Created 	: 06.02.2004
//  Modified 	: 06.02.2004
//	Author		: Dmitriy Iassenev
//	Description : XRay Script sound class
////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "script_particles.h"
#include "script_game_object.h"
#include "PhysicsSkeletonObject.h"
#include "Entity.h"
#include "../xrEngine/objectanimator.h"
#include "../xrEngine/IGame_Persistent.h"

CScriptParticlesCustom::CScriptParticlesCustom(CScriptParticles* owner, LPCSTR caParticlesName): CParticlesObject(
	caParticlesName,FALSE, true)
{
	m_owner = owner;
	m_animator = 0;
}

CScriptParticlesCustom::~CScriptParticlesCustom()
{
	xr_delete(m_animator);
}

void CScriptParticlesCustom::PSI_internal_delete()
{
	if (m_owner)
		m_owner->m_particles = nullptr;

	CParticlesObject::PSI_destroy();
}

void CScriptParticlesCustom::PSI_destroy()
{
	if (m_owner)
		m_owner->m_particles = nullptr;

	CParticlesObject::PSI_destroy();
}

void CScriptParticlesCustom::shedule_Update(u32 _dt)
{
	CParticlesObject::shedule_Update(_dt);
	if (m_animator)
	{
		float dt = float(_dt) / 1000.f;
		Fvector prev_pos = m_animator->XFORM().c;
		m_animator->Update(dt);
		Fvector vel;
		vel.sub(m_animator->XFORM().c, prev_pos).div(dt);
		UpdateParent(m_animator->XFORM(), vel);
	}
}

void CScriptParticlesCustom::LoadPath(LPCSTR caPathName)
{
	if (!m_animator) m_animator = xr_new<CObjectAnimator>();
	if ((0 == m_animator->Name()) || (0 != xr_strcmp(m_animator->Name(), caPathName)))
	{
		m_animator->Clear();
		m_animator->Load(caPathName);
	}
}

void CScriptParticlesCustom::StartPath(bool looped)
{
	VERIFY(m_animator);
	m_animator->Play(looped);
}

void CScriptParticlesCustom::PausePath(bool val)
{
	VERIFY(m_animator);
	m_animator->Pause(val);
}

void CScriptParticlesCustom::StopPath()
{
	VERIFY(m_animator);
	m_animator->Stop();
}

void CScriptParticlesCustom::remove_owner()
{
	R_ASSERT(m_owner);
	m_owner = 0;
}

CScriptParticles::CScriptParticles(LPCSTR caParticlesName)
{
	m_particles = xr_new<CScriptParticlesCustom>(this, caParticlesName);
	m_transform.identity();
	g_pGamePersistent->ps_active_deffer.push_back(m_particles);
}

CScriptParticles::~CScriptParticles()
{
	if (m_particles)
	{
		// destroy particles
		m_particles->remove_owner();
		m_particles->PSI_destroy();
		m_particles = 0;
	}
}

bool CScriptParticles::SetBloodThermalSource(CScriptGameObject* source)
{
    if(!m_particles || m_particles->IsPlaying())return false;
    // The dedicated call copies donor clock only; CPEDef owns blood eligibility.
    // Clock validity is independent and remains unknown; never fabricate death.
    bool known=false,corpse=false;u32 death=0;
    if(source && source->is_valid()) {
        if(auto* entity=smart_cast<CEntity*>(&source->object())) {
            known=true;corpse=entity->IsThermalCorpse();
            if(corpse)death=entity->GetLevelDeathTime();
        }else if(auto* piece=smart_cast<CPhysicsSkeletonObject*>(&source->object())) {
            if(piece->IsThermalCorpse()) {
                known=true;corpse=true;death=piece->PainterThermalDeathTime();
            }
        }
    }
    auto* visual=m_particles->renderable.visual;
    auto* effect=visual?visual->dcast_ParticleCustom():nullptr;
    return effect && effect->SetBloodThermalSource(known,corpse,death);
}

bool CScriptParticles::SetColorScale(float red,float green,float blue)
{
    if(!m_particles || m_particles->IsPlaying())return false;
    auto* visual=m_particles->renderable.visual;
    auto* effect=visual?visual->dcast_ParticleCustom():nullptr;
    return effect && effect->SetColorScale(red,green,blue);
}

bool CScriptParticles::SetSizeScale(float scale)
{
    if (!m_particles || m_particles->IsPlaying()) return false;
    auto* visual=m_particles->renderable.visual;
    auto* effect=visual?visual->dcast_ParticleCustom():nullptr;
    // Effects and groups (CParticleGroup::SetSizeScale, 2026-10-17) take the scale before play; other visuals refuse.
    return effect && effect->SetSizeScale(scale);
}

void CScriptParticles::Play(bool bHudMode)
{
	VERIFY(m_particles);
	m_particles->Play(bHudMode);
}

bool CScriptParticles::PlayPacket(bool bHudMode)
{
	VERIFY(m_particles);
	return m_particles->PlayPacket(bHudMode);
}

void CScriptParticles::PlayAtPos(const Fvector& position, bool bHudMode)
{
	VERIFY(m_particles);
	//m_particles->play_at_pos	(position);
	m_transform.translate_over(position);
	m_particles->UpdateParent(m_transform, zero_vel);
	m_particles->Play(bHudMode);
	m_particles->UpdateParent(m_transform, zero_vel);
}

void CScriptParticles::Stop()
{
	VERIFY(m_particles);
	m_particles->Stop(FALSE);
}

void CScriptParticles::StopDeffered()
{
	VERIFY(m_particles);
	m_particles->Stop(TRUE);
}

void CScriptParticles::MoveTo(const Fvector& pos, const Fvector& vel)
{
	VERIFY(m_particles);
	m_transform.translate_over(pos);
	m_particles->UpdateParent(m_transform, vel);
}

void CScriptParticles::XFORMMoveTo(const Fvector& pos)
{
	VERIFY(m_particles);
	m_transform.translate_over(pos);
	m_particles->SetXFORM(m_transform);
}

void CScriptParticles::SetDirection(const Fvector& dir)
{
	Fmatrix matrix;
	matrix.identity();
	matrix.k.set(dir);
	Fvector::generate_orthonormal_basis_normalized(matrix.k, matrix.j, matrix.i);
	matrix.translate_over(m_transform.c);
	m_transform.set(matrix);
	m_particles->UpdateParent(matrix, zero_vel);
}

void CScriptParticles::SetOrientation(float yaw, float pitch, float roll)
{
	Fmatrix matrix = Fmatrix().set(m_transform);
	matrix.setHPB(yaw, pitch, roll); // ?????????? matrix.c
	matrix.translate_over(m_transform.c);
	m_transform.set(matrix);
	m_particles->SetXFORM(m_transform);
}

void CScriptParticles::SetHudMode(bool bHudMode)
{
	m_particles->SetHudMode(bHudMode);
}

bool CScriptParticles::IsPlaying() const
{
	VERIFY(m_particles);
	return m_particles->IsPlaying();
}

bool CScriptParticles::IsLooped() const
{
	VERIFY(m_particles);
	return m_particles->IsLooped();
}

void CScriptParticles::LoadPath(LPCSTR caPathName)
{
	VERIFY(m_particles);
	m_particles->LoadPath(caPathName);
}

void CScriptParticles::StartPath(bool looped)
{
	m_particles->StartPath(looped);
}

void CScriptParticles::StopPath()
{
	m_particles->StopPath();
}

void CScriptParticles::PausePath(bool val)
{
	m_particles->PausePath(val);
}
