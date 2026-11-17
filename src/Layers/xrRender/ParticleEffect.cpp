#include "stdafx.h"
#pragma hdrstop

#include "ParticleEffect.h"
#ifndef _EDITOR // thermal3: the effect definition's action list (KillOld age limit = the particle lifetime), as ParticleEffectDef.cpp
#ifndef AnsiString
#define AnsiString std::string
#endif
#endif
#include "ParticleEffectActions.h"
#include "BloodGpuParticles.h"
#include "xrRender_console.h"
#include "../../Include/xrRender/BloodThermal.h"
#include <new>
#include "CHudInitializer.h"
#include <array>
#include <cstring>
#include <cmath>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#ifndef _EDITOR
#include <xmmintrin.h>
#endif

using namespace PAPI;
using namespace PS;

namespace
{
#if defined(USE_DX11) && !defined(_EDITOR)
SRWLOCK bloodRegistryLock = SRWLOCK_INIT;
CParticleEffect* bloodFirst = nullptr;
void RegisterBloodEffect(CParticleEffect* effect)
{
	AcquireSRWLockExclusive(&bloodRegistryLock);
	VERIFY(!effect->m_BloodRegistered);
	effect->m_BloodNext = bloodFirst;
	effect->m_BloodPrevious = nullptr;
	if (bloodFirst) bloodFirst->m_BloodPrevious = effect;
	bloodFirst = effect;
	effect->m_BloodRegistered = true;
	ReleaseSRWLockExclusive(&bloodRegistryLock);
}
void UnregisterBloodEffect(CParticleEffect* effect)
{
	AcquireSRWLockExclusive(&bloodRegistryLock);
	if (!effect->m_BloodRegistered) { ReleaseSRWLockExclusive(&bloodRegistryLock); return; }
	if (effect->m_BloodPrevious) effect->m_BloodPrevious->m_BloodNext = effect->m_BloodNext;
	else bloodFirst = effect->m_BloodNext;
	if (effect->m_BloodNext) effect->m_BloodNext->m_BloodPrevious = effect->m_BloodPrevious;
	effect->m_BloodPrevious = effect->m_BloodNext = nullptr;
	effect->m_BloodRegistered = false; effect->m_BloodGeneration = 0;
	ReleaseSRWLockExclusive(&bloodRegistryLock);
}
#endif
// Shared by the original render stream and a prediction on a local copy.
// Only a real graph draw commits interpolation; precomputation never advances
// an invisible particle or changes the worker's authored simulation state.
void InterpolateBloodParticle(PAPI::Particle& m, CParticleEffect* effect)
{
	if (effect->m_RT_Flags.is(CParticleEffect::flRT_LiveUpdate))
	{
		m.posI.set(m.pos); m.rotI.set(m.rot); m.velI.set(m.vel); m.sizeI.set(m.size);
	}
	else
	{
		float dt = 1.f - 10.f * Device.fTimeDelta;
		clamp(dt, 0.f, 0.99f);
		m.posI.inertion(m.pos, dt); m.rotI.inertion(m.rot, dt);
		m.velI.inertion(m.vel, dt); m.sizeI.inertion(m.size, dt);
	}
}
#if defined(USE_DX11) && !defined(_EDITOR)
struct BloodSpriteSnapshot
{
	PAPI::Particle original, predicted;
	Fmatrix transform;
	CPEDef* definition;
	u32 definition_flags;
	u8 runtime_flags;
};
std::array<BloodSpriteSnapshot, BloodGpuParticles::MaxSprites> bloodSnapshots;
std::array<BloodGpuParticles::Input, BloodGpuParticles::MaxSprites> bloodInputs;
Fvector bloodTop, bloodRight;
float bloodDelta = 0;
u32 bloodFrame = 0, bloodCount = 0;
bool bloodPrepared = false;

bool BloodSpriteSupported(CParticleEffect* effect)
{
	const CPEDef* def = effect->m_Def;
	if (!def || !def->m_Name.size() || std::strncmp(def->m_Name.c_str(), "painter\\fluid\\", 14) != 0 ||
		def->m_MaxParticles != 1 || !def->m_Flags.is(CPEDef::dfSprite) || effect->GetHudMode()) return false;
	const u32 unsupported = CPEDef::dfFramed | CPEDef::dfAlignToPath | CPEDef::dfVelocityScale;
	return (def->m_Flags.get() & unsupported) == 0;
}
bool BuildBloodInput(CParticleEffect* effect, const PAPI::Particle& m, BloodGpuParticles::Input& input)
{
	Fvector position;
	if (effect->m_RT_Flags.is(CParticleEffect::flRT_XFORM)) effect->m_XFORM.transform_tiny(position, m.posI);
	else position.set(m.posI);
	input = {};
	std::memcpy(input.position, &position.x, sizeof(input.position));
	std::memcpy(input.top, &RDEVICE.vCameraTop.x, sizeof(input.top));
	std::memcpy(input.right, &RDEVICE.vCameraRight.x, sizeof(input.right));
	input.radius_x = effect->m_SpriteSize.Radius(m.sizeI.x); input.radius_y = effect->m_SpriteSize.Radius(m.sizeI.y);
	// Original single-sprite stream uses this finite value as its angle sentinel.
	// At equality it retains zero sine/cosine rather than calling trig.
	if (m.rotI.x != float(0xFFFFFFFF))
	{
		input.sine = std::sinf(m.rotI.x); input.cosine = std::cosf(m.rotI.x);
	}
	input.color = color_rgba_f(effect->m_SpriteColor.Red(m.colorR),effect->m_SpriteColor.Green(m.colorG),effect->m_SpriteColor.Blue(m.colorB),m.colorA);
	for (unsigned i = 0; i < 3; ++i)
		if (!std::isfinite(input.position[i]) || !std::isfinite(input.top[i]) || !std::isfinite(input.right[i])) return false;
	return std::isfinite(input.radius_x) && std::isfinite(input.radius_y) &&
		std::isfinite(input.sine) && std::isfinite(input.cosine);
}
bool ConsumeBloodSprite(CParticleEffect* effect, PAPI::Particle* particles, u32 count, u32& offset, bool& culled)
{
	if (!bloodPrepared || count != 1 || effect->m_BloodGeneration != BloodGpuParticles::Generation() ||
		effect->m_BloodSprite >= bloodCount || !BloodGpuParticles::Vertices()) return false;
	const u32 index = effect->m_BloodSprite;
	effect->m_BloodGeneration = 0; // A second draw performs its original interpolation.
	const BloodSpriteSnapshot& s = bloodSnapshots[index];
	if (bloodFrame != Device.dwFrame || bloodDelta != Device.fTimeDelta ||
		std::memcmp(&bloodTop, &RDEVICE.vCameraTop, sizeof(Fvector)) ||
		std::memcmp(&bloodRight, &RDEVICE.vCameraRight, sizeof(Fvector)) ||
		s.definition != effect->m_Def || s.definition_flags != effect->m_Def->m_Flags.get() ||
		s.runtime_flags != effect->m_RT_Flags.get() ||
		std::memcmp(&s.transform, &effect->m_XFORM, sizeof(Fmatrix)) ||
		std::memcmp(&s.original, particles, sizeof(PAPI::Particle)))
	{
		BloodGpuParticles::CountFallback(); return false;
	}
	// Commit precisely the fields mutated by the original render stream.
	particles->posI = s.predicted.posI; particles->rotI = s.predicted.rotI;
	particles->velI = s.predicted.velI; particles->sizeI = s.predicted.sizeI;
	Fvector world;
	effect->m_XFORM.transform_tiny(world, particles->posI);
	culled = Device.vCameraPosition.distance_to_sqr(world) >
		_sqr(g_pGamePersistent->Environment().CurrentEnv->fog_distance);
	offset = index * 4;
	if (!culled) BloodGpuParticles::CountDraw();
	return true;
}
#endif
}

#ifdef USE_DX11
void BloodGpuParticles::Prepare(bool enabled)
{
	BeginPreparation();
#if defined(USE_DX11) && !defined(_EDITOR)
	bloodPrepared = false; bloodCount = 0;
	if (!enabled || !Warmup()) return;
	bloodTop = RDEVICE.vCameraTop; bloodRight = RDEVICE.vCameraRight;
	bloodDelta = Device.fTimeDelta; bloodFrame = Device.dwFrame;
	AcquireSRWLockShared(&bloodRegistryLock);
	for (CParticleEffect* effect = bloodFirst; effect; effect = effect->m_BloodNext)
	{
		if (!BloodSpriteSupported(effect)) continue;
		PAPI::Particle* particles = nullptr; u32 count = 0;
		ParticleManager()->GetParticles(effect->GetHandleEffect(), particles, count);
		if (count != 1) continue;
		if (bloodCount == MaxSprites) { CountFallback(true); continue; }
		BloodSpriteSnapshot& s = bloodSnapshots[bloodCount];
		std::memcpy(&s.original, particles, sizeof(s.original)); s.predicted = particles[0];
		InterpolateBloodParticle(s.predicted, effect);
		if (!BuildBloodInput(effect, s.predicted, bloodInputs[bloodCount])) { CountFallback(); continue; }
		s.transform = effect->m_XFORM; s.definition = effect->m_Def;
		s.definition_flags = effect->m_Def->m_Flags.get(); s.runtime_flags = effect->m_RT_Flags.get();
		effect->m_BloodSprite = bloodCount++; effect->m_BloodGeneration = Generation();
	}
	ReleaseSRWLockShared(&bloodRegistryLock);
	if (bloodCount) bloodPrepared = Dispatch(bloodInputs.data(), bloodCount);
	if (!bloodPrepared) for (u32 i = 0; i < bloodCount; ++i) CountFallback();
#else
	(void)enabled;
#endif
}
#endif

static void ApplyTexgen(const Fmatrix& mVP)
{
	Fmatrix mTexgen;

#if defined(USE_DX10) || defined(USE_DX11)
	Fmatrix mTexelAdjust =
	{
		0.5f, 0.0f, 0.0f, 0.0f,
		0.0f, -0.5f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.5f, 0.5f, 0.0f, 1.0f
	};
#else	//	USE_DX10
	float _w = float(RDEVICE.dwWidth);
	float _h = float(RDEVICE.dwHeight);
	float o_w = (.5f / _w);
	float o_h = (.5f / _h);
	Fmatrix mTexelAdjust =
	{
		0.5f, 0.0f, 0.0f, 0.0f,
		0.0f, -0.5f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.5f + o_w, 0.5f + o_h, 0.0f, 1.0f
	};
#endif	//	USE_DX10

	mTexgen.mul(mTexelAdjust, mVP);
	RCache.set_c("mVPTexgen", mTexgen);
}

void PS::OnEffectParticleBirth(void* owner, u32, PAPI::Particle& m, u32)
{
	CParticleEffect* PE = static_cast<CParticleEffect*>(owner);
	VERIFY(PE);
	CPEDef* PED = PE->GetDefinition();
	if (PED)
	{
		if (PED->m_Flags.is(CPEDef::dfRandomFrame))
			m.frame = (u16)iFloor(Random.randI(PED->m_Frame.m_iFrameCount) * 255.f);
		if (PED->m_Flags.is(CPEDef::dfAnimated) && PED->m_Flags.is(CPEDef::dfRandomPlayback) && Random.randI(2))
			m.flags.set(Particle::ANIMATE_CCW,TRUE);
	}
}

void PS::OnEffectParticleDead(void*, u32, PAPI::Particle&, u32)
{
	//	CPEDef* PE = static_cast<CPEDef*>(owner);
}

//------------------------------------------------------------------------------
// class CParticleEffect
//------------------------------------------------------------------------------
CParticleEffect::CParticleEffect()
{
	m_HandleEffect = ParticleManager()->CreateEffect(1);
	VERIFY(m_HandleEffect>=0);
	m_HandleActionList = ParticleManager()->CreateActionList();
	VERIFY(m_HandleActionList>=0);
	m_RT_Flags.zero();
	m_Def = 0;
	m_fElapsedLimit = 0.f;
	m_MemDT = 0;
	m_InitialPosition.set(0, 0, 0);
	m_DestroyCallback = 0;
	m_CollisionCallback = 0;
	m_XFORM.identity();
#if defined(USE_DX11) && !defined(_EDITOR)
// Registration occurs only after a supported definition has finished Compile.
#endif
}

CParticleEffect::~CParticleEffect()
{
#if defined(USE_DX11) && !defined(_EDITOR)
	UnregisterBloodEffect(this);
#endif
	// Log					("--- destroy PE");
	OnDeviceDestroy();
	ParticleManager()->DestroyEffect(m_HandleEffect);
	ParticleManager()->DestroyActionList(m_HandleActionList);
}

void CParticleEffect::Play()
{
	m_RT_Flags.set(flRT_DefferedStop,FALSE);
	m_RT_Flags.set(flRT_Playing,TRUE);
	ParticleManager()->PlayEffect(m_HandleEffect, m_HandleActionList);
}

void CParticleEffect::Stop(BOOL bDefferedStop)
{
	ParticleManager()->StopEffect(m_HandleEffect, m_HandleActionList, bDefferedStop);
	if (bDefferedStop)
	{
		m_RT_Flags.set(flRT_DefferedStop,TRUE);
	}
	else
	{
		m_RT_Flags.set(flRT_Playing,FALSE);
	}
}

void CParticleEffect::RefreshShader()
{
	OnDeviceDestroy();
	OnDeviceCreate();
}

void CParticleEffect::UpdateParent(const Fmatrix& m, const Fvector& velocity, BOOL bXFORM)
{
	m_RT_Flags.set(flRT_XFORM, bXFORM);
	if (bXFORM) m_XFORM.set(m);
	else
	{
		m_InitialPosition = m.c;
		ParticleManager()->Transform(m_HandleActionList, m, velocity);
	}
}

void CParticleEffect::OnFrame(u32 frame_dt)
{
	if (0==m_RT_Flags.is(flRT_LiveUpdate))
	{
		if (m_Def && m_RT_Flags.is(flRT_Playing))
		{
			m_MemDT += frame_dt;

			int StepCount = 0;
			u32 uDT_STEP = m_Def->GetUStep();
			float fDT_STEP = m_Def->GetFStep();
			if (m_MemDT >= uDT_STEP)
			{
				// allow maximum of three steps (99ms) to avoid slowdown after loading
				// it will really skip updates at less than 10fps, which is unplayable
				StepCount = m_MemDT / uDT_STEP;
				m_MemDT = m_MemDT % uDT_STEP;
				clamp(StepCount, 0, 3);
			}

			for (; StepCount; StepCount--)
			{
				if (m_Def->m_Flags.is(CPEDef::dfTimeLimit))
				{
					if (!m_RT_Flags.is(flRT_DefferedStop))
					{
						m_fElapsedLimit -= fDT_STEP;
						if (m_fElapsedLimit < 0.f)
						{
							m_fElapsedLimit = m_Def->m_fTimeLimit;
							Stop(true);
							break;
						}
					}
				}
				ParticleManager()->Update(m_HandleEffect, m_HandleActionList, fDT_STEP);

				PAPI::Particle* particles;
				u32 p_cnt;
				ParticleManager()->GetParticles(m_HandleEffect, particles, p_cnt);

				// our actions
				if (m_Def->m_Flags.is(CPEDef::dfFramed | CPEDef::dfAnimated))
					m_Def->ExecuteAnimate(particles, p_cnt, fDT_STEP);
				if (m_Def->m_Flags.is(CPEDef::dfCollision))
					m_Def->ExecuteCollision(particles, p_cnt, fDT_STEP, this, m_CollisionCallback);
				
				//-move action
				if (p_cnt)
				{
					vis.box.invalidate();
					float p_size = 0.f;
					for (u32 i = 0; i < p_cnt; i++)
					{
						Particle& m = particles[i];
						if(!RImplementation.ViewBase.testSphere_dirty(vis.sphere.P, vis.sphere.R))
						{
							m.posI.set(m.pos);
							m.rotI.set(m.rot);
							m.velI.set(m.vel);
							m.sizeI.set(m.size);
						}
						vis.box.modify((Fvector&)m.pos);
						if (m.size.x > p_size) p_size = m.size.x;
						if (m.size.y > p_size) p_size = m.size.y;
						if (m.size.z > p_size) p_size = m.size.z;
					}
					vis.box.grow(m_SpriteSize.Bounds(p_size));
					vis.box.getsphere(vis.sphere.P, vis.sphere.R);
				}
				if (m_RT_Flags.is(flRT_DefferedStop) && (0 == p_cnt))
				{
					m_RT_Flags.set(flRT_Playing | flRT_DefferedStop, FALSE);
					break;
				}
			}
		}
		else
		{
			vis.box.set(m_InitialPosition, m_InitialPosition);
			vis.box.grow(EPS_L);
			vis.box.getsphere(vis.sphere.P, vis.sphere.R);
		}
	}
	else
	{
		if (m_Def && m_RT_Flags.is(flRT_Playing))
		{
			ParticleManager()->Update(m_HandleEffect, m_HandleActionList, Device.fTimeDelta);
			PAPI::Particle* particles = NULL;
			u32 p_cnt = 0;
			ParticleManager()->GetParticles(m_HandleEffect, particles, p_cnt);
			if (!particles) return;
			if (m_Def->m_Flags.is(CPEDef::dfFramed | CPEDef::dfAnimated))
				m_Def->ExecuteAnimate(particles, p_cnt, Device.fTimeDelta);
			if (m_Def->m_Flags.is(CPEDef::dfCollision))
				m_Def->ExecuteCollision(particles, p_cnt, Device.fTimeDelta, this, m_CollisionCallback);
			if (p_cnt)
			{
				vis.box.invalidate();
				float p_size = 0.f;
				for (u32 i = 0; i < p_cnt; i++)
				{
					Particle& m = particles[i];
					if (!_valid(m.pos)) continue;
					vis.box.modify(Fvector(m.pos));
					if (m.size.x > p_size) p_size = m.size.x;
					if (m.size.y > p_size) p_size = m.size.y;
					if (m.size.z > p_size) p_size = m.size.z;
				}
				vis.box.grow(m_SpriteSize.Bounds(p_size));
				vis.box.getsphere(vis.sphere.P, vis.sphere.R);
			}
			
			bool deffered_stop = true;
			if (m_Def->m_Flags.is(CPEDef::dfTimeLimit))
			{
				if (!m_RT_Flags.is(flRT_DefferedStop))
				{
					m_fElapsedLimit -= Device.fTimeDelta;
					if (m_fElapsedLimit < 0.f)
					{
						m_fElapsedLimit = m_Def->m_fTimeLimit;
						Stop(true);
						deffered_stop = false;
					}
				}
			}
			if (deffered_stop && m_RT_Flags.is(flRT_DefferedStop) && (0 == p_cnt))
				m_RT_Flags.set(flRT_Playing | flRT_DefferedStop, FALSE);
		}
		else
		{
			vis.box.set(m_InitialPosition, m_InitialPosition);
			vis.box.grow(EPS_L);
			vis.box.getsphere(vis.sphere.P, vis.sphere.R);
		}
	}
}

BOOL CParticleEffect::Compile(CPEDef* def)
{
    m_BloodThermalSource={};m_BloodThermalAgeOffset=0.f;
#if defined(USE_DX11) && !defined(_EDITOR)
	UnregisterBloodEffect(this);
#endif
	m_Def = def;
	if (m_Def)
	{
		// refresh shader
		RefreshShader();

		// append actions
		IReader F(m_Def->m_Actions.pointer(), m_Def->m_Actions.size());
		ParticleManager()->LoadActions(m_HandleActionList, F);
		ParticleManager()->SetMaxParticles(m_HandleEffect, m_Def->m_MaxParticles);
		ParticleManager()->SetCallback(m_HandleEffect, OnEffectParticleBirth, OnEffectParticleDead, this, 0);
		// time limit
		if (m_Def->m_Flags.is(CPEDef::dfTimeLimit))
			m_fElapsedLimit = m_Def->m_fTimeLimit;
	}
	if (def) shader = def->m_CachedShader;
#if defined(USE_DX11) && !defined(_EDITOR)
	if (BloodSpriteSupported(this)) RegisterBloodEffect(this);
#endif
	return TRUE;
}

void CParticleEffect::SetBirthDeadCB(PAPI::OnBirthParticleCB bc, PAPI::OnDeadParticleCB dc, void* owner, u32 p)
{
	ParticleManager()->SetCallback(m_HandleEffect, bc, dc, owner, p);
}

u32 CParticleEffect::ParticlesCount()
{
	return ParticleManager()->GetParticlesCount(m_HandleEffect);
}

//------------------------------------------------------------------------------
// Render
//------------------------------------------------------------------------------
void CParticleEffect::Copy(dxRender_Visual*)
{
    m_SpriteSize=BloodParticleSize{};
    m_SpriteColor=BloodParticleColor{};
	FATAL("Can't duplicate particle system - NOT IMPLEMENTED");
}

void CParticleEffect::OnDeviceCreate()
{
	if (m_Def)
	{
		if (m_Def->m_Flags.is(CPEDef::dfSprite))
		{
			geom.create(FVF::F_LIT, RCache.Vertex.Buffer(), RCache.QuadIB);
			if (m_Def) shader = m_Def->m_CachedShader;
		}
	}
}

void CParticleEffect::OnDeviceDestroy()
{
    m_ThermalGeom.destroy(); m_ThermalShader.destroy();
	if (m_Def)
	{
		if (m_Def->m_Flags.is(CPEDef::dfSprite))
		{
			geom.destroy();
			shader.destroy();
		}
	}
}

#ifndef _EDITOR
//----------------------------------------------------
IC void FillSprite_fpu(FVF::LIT*& pv, const Fvector& T, const Fvector& R, const Fvector& pos, const Fvector2& lt,
                       const Fvector2& rb, float r1, float r2, u32 clr, float angle)
{
	float sa = _sin(angle);
	float ca = _cos(angle);

	Fvector Vr, Vt;

	Vr.x = T.x * r1 * sa + R.x * r1 * ca;
	Vr.y = T.y * r1 * sa + R.y * r1 * ca;
	Vr.z = T.z * r1 * sa + R.z * r1 * ca;

	Vt.x = T.x * r2 * ca - R.x * r2 * sa;
	Vt.y = T.y * r2 * ca - R.y * r2 * sa;
	Vt.z = T.z * r2 * ca - R.z * r2 * sa;

	Fvector a, b, c, d;

	a.sub(Vt, Vr);
	b.add(Vt, Vr);

	c.invert(a);
	d.invert(b);

	pv->set(d.x + pos.x, d.y + pos.y, d.z + pos.z, clr, lt.x, rb.y);
	pv++;
	pv->set(a.x + pos.x, a.y + pos.y, a.z + pos.z, clr, lt.x, lt.y);
	pv++;
	pv->set(c.x + pos.x, c.y + pos.y, c.z + pos.z, clr, rb.x, rb.y);
	pv++;
	pv->set(b.x + pos.x, b.y + pos.y, b.z + pos.z, clr, rb.x, lt.y);
	pv++;
}

#include "../../xrCore/Lock.hpp"

Lock m_sprite_section;

IC void FillSprite(FVF::LIT*& pv, const Fvector& T, const Fvector& R, const Fvector& pos, const Fvector2& lt,
                   const Fvector2& rb, float r1, float r2, u32 clr, float sina, float cosa)
{
	m_sprite_section.Enter();

	__m128 Vr, Vt, _T, _R, _pos, _zz, _sa, _ca, a, b, c, d;

	_sa = _mm_set1_ps(sina);
	_ca = _mm_set1_ps(cosa);

	_T = _mm_load_ss((float*)&T.x);
	_T = _mm_loadh_pi(_T, (__m64*)&T.y);

	_R = _mm_load_ss((float*)&R.x);
	_R = _mm_loadh_pi(_R, (__m64*)&R.y);

	_pos = _mm_load_ss((float*)&pos.x);
	_pos = _mm_loadh_pi(_pos, (__m64*)&pos.y);

	_zz = _mm_setzero_ps();

	Vr = _mm_mul_ps(_mm_set1_ps(r1), _mm_add_ps(_mm_mul_ps(_T, _sa), _mm_mul_ps(_R, _ca)));
	Vt = _mm_mul_ps(_mm_set1_ps(r2), _mm_sub_ps(_mm_mul_ps(_T, _ca), _mm_mul_ps(_R, _sa)));

	a = _mm_sub_ps(Vt, Vr);
	b = _mm_add_ps(Vt, Vr);
	c = _mm_sub_ps(_zz, a);
	d = _mm_sub_ps(_zz, b);

	a = _mm_add_ps(a, _pos);
	d = _mm_add_ps(d, _pos);
	b = _mm_add_ps(b, _pos);
	c = _mm_add_ps(c, _pos);

	_mm_store_ss((float*)&pv->p.x, d);
	_mm_storeh_pi((__m64*)&pv->p.y, d);
	pv->color = clr;
	pv->t.set(lt.x, rb.y);
	pv++;

	_mm_store_ss((float*)&pv->p.x, a);
	_mm_storeh_pi((__m64*)&pv->p.y, a);
	pv->color = clr;
	pv->t.set(lt.x, lt.y);
	pv++;

	_mm_store_ss((float*)&pv->p.x, c);
	_mm_storeh_pi((__m64*)&pv->p.y, c);
	pv->color = clr;
	pv->t.set(rb.x, rb.y);
	pv++;

	_mm_store_ss((float*)&pv->p.x, b);
	_mm_storeh_pi((__m64*)&pv->p.y, b);
	pv->color = clr;
	pv->t.set(rb.x, lt.y);
	pv++;

	m_sprite_section.Leave();
}

IC void FillSprite(FVF::LIT*& pv, const Fvector& pos, const Fvector& dir, const Fvector2& lt, const Fvector2& rb,
                   float r1, float r2, u32 clr, float sina, float cosa)
{
#ifdef _GPA_ENABLED
		TAL_SCOPED_TASK_NAMED( "FillSpriteTransform()" );
#endif // _GPA_ENABLED

	const Fvector& T = dir;
	Fvector R;

	// R.crossproduct(T,RDEVICE.vCameraDirection).normalize_safe();

	__m128 _t, _t1, _t2, _r, _r1, _r2;

	// crossproduct

	_t = _mm_load_ss((float*)&T.x);
	_t = _mm_loadh_pi(_t, (__m64*)&T.y);

	_r = _mm_load_ss((float*)&RDEVICE.vCameraDirection.x);
	_r = _mm_loadh_pi(_r, (__m64*)&RDEVICE.vCameraDirection.y);

	_t1 = _mm_shuffle_ps(_t, _t, _MM_SHUFFLE(0, 3, 1, 2));
	_t2 = _mm_shuffle_ps(_t, _t, _MM_SHUFFLE(2, 0, 1, 3));

	_r1 = _mm_shuffle_ps(_r, _r, _MM_SHUFFLE(2, 0, 1, 3));
	_r2 = _mm_shuffle_ps(_r, _r, _MM_SHUFFLE(0, 3, 1, 2));

	_t1 = _mm_mul_ps(_t1, _r1);
	_t2 = _mm_mul_ps(_t2, _r2);

	_t1 = _mm_sub_ps(_t1, _t2); // z | y | 0 | x

	// normalize_safe

	_t2 = _mm_mul_ps(_t1, _t1); // zz | yy | 00 | xx
	_r1 = _mm_movehl_ps(_t2, _t2); // zz | yy | zz | yy
	_t2 = _mm_add_ss(_t2, _r1); // zz | yy | 00 | xx + yy
	_r1 = _mm_shuffle_ps(_r1, _r1, _MM_SHUFFLE(1, 1, 1, 1)); // zz | zz | zz | zz
	_t2 = _mm_add_ss(_t2, _r1); // zz | yy | 00 | xx + yy + zz

	_r1 = _mm_set_ss(std::numeric_limits<float>::min());

	if (_mm_comigt_ss(_t2, _r1))
	{
		_t2 = _mm_rsqrt_ss(_t2);
		_t2 = _mm_shuffle_ps(_t2, _t2, _MM_SHUFFLE(0, 0, 0, 0));
		_t1 = _mm_mul_ps(_t1, _t2);
	}

	_mm_store_ss((float*)&R.x, _t1);
	_mm_storeh_pi((__m64*)&R.y, _t1);

	FillSprite(pv, T, R, pos, lt, rb, r1, r2, clr, sina, cosa);
}

struct PRS_PARAMS
{
	FVF::LIT* pv;
	u32 p_from;
	u32 p_to;
	PAPI::Particle* particles;
	CParticleEffect* pPE;
};

__forceinline void magnitude_sse(Fvector& vec, float& res)
{
	__m128 tv, tu;

	tv = _mm_load_ss((float*)&vec.x); // tv = 0 | 0 | 0 | x
	tv = _mm_loadh_pi(tv, (__m64*)&vec.y); // tv = z | y | 0 | x
	tv = _mm_mul_ps(tv, tv); // tv = zz | yy | 0 | xx
	tu = _mm_movehl_ps(tv, tv); // tu = zz | yy | zz | yy
	tv = _mm_add_ss(tv, tu); // tv = zz | yy | 0 | xx + yy
	tu = _mm_shuffle_ps(tu, tu, _MM_SHUFFLE(1, 1, 1, 1)); // tu = zz | zz | zz | zz
	tv = _mm_add_ss(tv, tu); // tv = zz | yy | 0 | xx + yy + zz
	tv = _mm_sqrt_ss(tv); // tv = zz | yy | 0 | sqrt( xx + yy + zz )
	_mm_store_ss((float*)&res, tv);
}

struct ParticleSpriteAngle { float angle=float(0xFFFFFFFF), sine=0.f, cosine=0.f; };

bool ParticleRenderStream(FVF::LIT* pv, u32 count, PAPI::Particle * particles, CParticleEffect * pPE, ParticleSpriteAngle* state=nullptr)
{
	float sina = state?state->sine:0.0f, cosa = state?state->cosine:0.0f;
	// Xottab_DUTY: changed angle to be float instead of DWORD
	// But it must be 0xFFFFFFFF or otherwise some particles won't play
	float angle = state?state->angle:float(0xFFFFFFFF);


	for (u32 i = 0; i != count; ++i)
	{
		PAPI::Particle& m = particles[i];
		InterpolateBloodParticle(m, pPE);

		Fvector wp_eff;
		pPE->m_XFORM.transform_tiny(wp_eff, m.posI);

		if (Device.vCameraPosition.distance_to_sqr(wp_eff) > _sqr(g_pGamePersistent->Environment().CurrentEnv->fog_distance))
		{
			return true;
		}

		Fvector2 lt, rb;
		lt.set(0.f, 0.f);
		rb.set(1.f, 1.f);

		if (angle != m.rotI.x)
		{
			angle = m.rotI.x;
			sina = std::sinf(angle);
			cosa = std::cosf(angle);
		}

		if (pPE->m_Def->m_Flags.is(CPEDef::dfFramed))
			pPE->m_Def->m_Frame.CalculateTC(iFloor(float(m.frame) / 255.f), lt, rb);

		float r_x = m.sizeI.x * 0.5f;
		float r_y = m.sizeI.y * 0.5f;
		float speed = 0.f;
		bool speed_calculated = false;

		if (pPE->m_Def->m_Flags.is(CPEDef::dfVelocityScale))
		{
			magnitude_sse(m.velI, speed);
			speed_calculated = true;
			r_x += speed * pPE->m_Def->m_VelocityScale.x;
			r_y += speed * pPE->m_Def->m_VelocityScale.y;
		}

		r_x *= pPE->m_SpriteSize.factor; r_y *= pPE->m_SpriteSize.factor;

		if (pPE->m_Def->m_Flags.is(CPEDef::dfAlignToPath))
		{
			if (!speed_calculated)
				magnitude_sse(m.velI, speed);
			if ((speed < EPS_S) && pPE->m_Def->m_Flags.is(CPEDef::dfWorldAlign))
			{
				Fmatrix M;
				M.setXYZ(pPE->m_Def->m_APDefaultRotation);
				if (pPE->m_RT_Flags.is(CParticleEffect::flRT_XFORM))
				{
					M.mulA_43(pPE->m_XFORM);
					FillSprite(pv, M.k, M.i, wp_eff, lt, rb, r_x, r_y, color_rgba_f(pPE->m_SpriteColor.Red(m.colorR),pPE->m_SpriteColor.Green(m.colorG),pPE->m_SpriteColor.Blue(m.colorB),m.colorA), sina, cosa);
				}
				else
				{
					FillSprite(pv, M.k, M.i, m.posI, lt, rb, r_x, r_y, color_rgba_f(pPE->m_SpriteColor.Red(m.colorR),pPE->m_SpriteColor.Green(m.colorG),pPE->m_SpriteColor.Blue(m.colorB),m.colorA), sina, cosa);
				}
			}
			else if ((speed >= EPS_S) && pPE->m_Def->m_Flags.is(CPEDef::dfFaceAlign))
			{
				Fmatrix M;
				M.identity();
				M.k.div(m.velI, speed);
				M.j.set(0, 1, 0);
				if (_abs(M.j.dotproduct(M.k)) > .99f)
					M.j.set(0, 0, 1);
				M.i.crossproduct(M.j, M.k);
				M.i.normalize();
				M.j.crossproduct(M.k, M.i);
				M.j.normalize();
				if (pPE->m_RT_Flags.is(CParticleEffect::flRT_XFORM))
				{
					M.mulA_43(pPE->m_XFORM);
					FillSprite(pv, M.j, M.i, wp_eff, lt, rb, r_x, r_y, color_rgba_f(pPE->m_SpriteColor.Red(m.colorR),pPE->m_SpriteColor.Green(m.colorG),pPE->m_SpriteColor.Blue(m.colorB),m.colorA), sina, cosa);
				}
				else
				{
					FillSprite(pv, M.j, M.i, m.posI, lt, rb, r_x, r_y, color_rgba_f(pPE->m_SpriteColor.Red(m.colorR),pPE->m_SpriteColor.Green(m.colorG),pPE->m_SpriteColor.Blue(m.colorB),m.colorA), sina, cosa);
				}
			}
			else
			{
				Fvector dir;
				if (speed >= EPS_S)
					dir.div(m.velI, speed);
				else
					dir.setHP(-pPE->m_Def->m_APDefaultRotation.y, -pPE->m_Def->m_APDefaultRotation.x);
				if (pPE->m_RT_Flags.is(CParticleEffect::flRT_XFORM))
				{
					Fvector d;
					pPE->m_XFORM.transform_dir(d, dir);
					FillSprite(pv, wp_eff, d, lt, rb, r_x, r_y, color_rgba_f(pPE->m_SpriteColor.Red(m.colorR),pPE->m_SpriteColor.Green(m.colorG),pPE->m_SpriteColor.Blue(m.colorB),m.colorA), sina, cosa);
				}
				else
				{
					FillSprite(pv, m.posI, dir, lt, rb, r_x, r_y, color_rgba_f(pPE->m_SpriteColor.Red(m.colorR),pPE->m_SpriteColor.Green(m.colorG),pPE->m_SpriteColor.Blue(m.colorB),m.colorA), sina, cosa);
				}
			}
		}
		else
		{
			if (pPE->m_RT_Flags.is(CParticleEffect::flRT_XFORM))
			{
				FillSprite(pv, RDEVICE.vCameraTop, RDEVICE.vCameraRight, wp_eff, lt, rb, r_x, r_y, color_rgba_f(pPE->m_SpriteColor.Red(m.colorR),pPE->m_SpriteColor.Green(m.colorG),pPE->m_SpriteColor.Blue(m.colorB),m.colorA), sina, cosa);
			}
			else
			{
				FillSprite(pv, RDEVICE.vCameraTop, RDEVICE.vCameraRight, m.posI, lt, rb, r_x, r_y, color_rgba_f(pPE->m_SpriteColor.Red(m.colorR),pPE->m_SpriteColor.Green(m.colorG),pPE->m_SpriteColor.Blue(m.colorB),m.colorA), sina, cosa);
			}
		}
	}
    if(state){state->angle=angle;state->sine=sina;state->cosine=cosa;}
	return false;
}

#if defined(USE_DX11) && !defined(_EDITOR)
bool CParticleEffect::RenderBloodThermal(PAPI::Particle* particles,u32 count)
{
    if (ps_r2_heatvision<=0 || !m_Def || !BloodEffect::IsParticle(m_Def->m_BloodEffectId) || !count || GetHudMode()) return false;
    if (RImplementation.phase == CRender::PHASE_SMAP) return false; // thermal3: a shadow map keeps the stock draw (no heat target there)
    LPCSTR material=*m_Def->m_ShaderName;
    LPCSTR thermal=nullptr;
    if(!xr_strcmp(material,"painter\\blood_droplet"))thermal="painter\\blood_thermal";
    else if(!xr_strcmp(material,"painter\\blood_droplet_color"))thermal="painter\\blood_thermal_color";
    // Material chooses the compatible program only AFTER authored per-effect eligibility.
    // Neither donor ownership, PG ancestry nor shader/texture names create blood intent.
    else if(!xr_strcmp(material,"painter\\hitfx_impact"))thermal="painter\\blood_thermal_impact";
    else if(!xr_strcmp(material,"painter\\hitfx_impact_color"))thermal="painter\\blood_thermal_impact_color";
    else if(!xr_strcmp(material,"particles\\blend"))thermal="painter\\blood_thermal_blend";
    // thermal3: blood sprites with the other stock materials. Colour / G-buffer: the stock program and blend of that material
    // (Blender_Particle): alpha_add mode 5 (srcalpha, one), add mode 2 (one, one), set mode 0 (opaque, deffer_particle into the
    // G-buffer, alpha test 200); heat: the blood heat as for particles\blend (thermal3 shaders).
    else if(!xr_strcmp(material,"particles\\alpha_add"))thermal="painter\\blood_thermal_alpha_add";
    else if(!xr_strcmp(material,"particles\\add"))thermal="painter\\blood_thermal_add";
    else if(!xr_strcmp(material,"particles\\set"))thermal="painter\\blood_thermal_set";
    if(!thermal)return false;
    // The ordinary single-sprite GPU route and all normal-view programs remain
    // unchanged. Thermal uses actual per-particle simulation age, never TTL,
    // alpha, a fabricated zero or an average across particles.
    // Predict the existing whole-effect far-fog refusal before drawing a chunk.
    for (u32 i=0;i<count;++i) {
        PAPI::Particle copy=particles[i];InterpolateBloodParticle(copy,this);
        Fvector world;m_XFORM.transform_tiny(world,copy.posI);
        if (Device.vCameraPosition.distance_to_sqr(world)>_sqr(g_pGamePersistent->Environment().CurrentEnv->fog_distance)) return false;
    }
    struct ThermalVertex { FVF::LIT original; float age; };
    static_assert(sizeof(FVF::LIT)==24 && sizeof(ThermalVertex)==28,"thermal particle stream ABI");
    static D3DVERTEXELEMENT9 decl[]={
        {0,0,D3DDECLTYPE_FLOAT3,D3DDECLMETHOD_DEFAULT,D3DDECLUSAGE_POSITION,0},
        {0,12,D3DDECLTYPE_D3DCOLOR,D3DDECLMETHOD_DEFAULT,D3DDECLUSAGE_COLOR,0},
        {0,16,D3DDECLTYPE_FLOAT2,D3DDECLMETHOD_DEFAULT,D3DDECLUSAGE_TEXCOORD,0},
        {0,24,D3DDECLTYPE_FLOAT1,D3DDECLMETHOD_DEFAULT,D3DDECLUSAGE_TEXCOORD,2},D3DDECL_END()};
    if (!m_ThermalGeom) m_ThermalGeom.create(decl,RCache.Vertex.Buffer(),RCache.QuadIB);
    if (!m_ThermalShader) m_ThermalShader.create(thermal,*m_Def->m_TextureName);
    // thermal3: each blood particle cools over ITS OWN lifetime (painter_blood_particle_heat.h): the smallest age limit of the
    // effect's enabled KillOld actions (particles die at that age), else its time limit, else 1 s; passed as painter_blood_cooling.x.
    float life=0.f;
    for (EParticleAction* a:m_Def->m_EActionList)
        if (a&&a->type==PAPI::PAKillOldID&&a->flags.is(EParticleAction::flEnabled)&&!a->_bool("Kill Less Than").val) {
            const float v=a->_float("Age Limit").val;
            if (std::isfinite(v)&&v>0.f&&(life<=0.f||v<life)) life=v;
        }
    if (!(life>0.f)) life=m_Def->m_fTimeLimit>0.f?m_Def->m_fTimeLimit:1.f;
    constexpr u32 chunk=4096;
    // Fixed bounded scratch, all particles visited; chunking is not admission.
    static thread_local xr_vector<FVF::LIT> staged;
    try { staged.resize(chunk*4); } catch (const std::bad_alloc&) { return false; }
    ParticleSpriteAngle angle;
    for (u32 start=0;start<count;) {
        const u32 n=(std::min)(chunk,count-start);
        if (ParticleRenderStream(staged.data(),n,particles+start,this,&angle)) return true;
        u32 offset=0;auto* output=static_cast<ThermalVertex*>(RCache.Vertex.Lock(n*4,sizeof(ThermalVertex),offset));
        for (u32 i=0;i<n;++i) for (u32 c=0;c<4;++c) {
            output[i*4+c].original=staged[i*4+c];output[i*4+c].age=particles[start+i].age+m_BloodThermalAgeOffset;
        }
        RCache.Vertex.Unlock(n*4,sizeof(ThermalVertex));
        RCache.set_xform_world(Fidentity);RCache.set_Shader(m_ThermalShader);RCache.set_Geometry(m_ThermalGeom);
        RCache.set_c("painter_blood_cooling",life,heat_vision_cooldown?1.f:0.f, // thermal3: .x = the particle lifetime (was the global cool-down)
            m_BloodThermalSource.known&&m_BloodThermalSource.corpse?BloodThermal::Corpse(Device.dwTimeGlobal,m_BloodThermalSource.death_time,heat_vision_cooldown_time,heat_vision_cooldown!=0):1.f,
            m_BloodThermalSource.known?1.f:0.f);
        RCache.set_CullMode(m_Def->m_Flags.is(CPEDef::dfCulling)?(m_Def->m_Flags.is(CPEDef::dfCullCCW)?CULL_CCW:CULL_CW):CULL_NONE);
        RCache.Render(D3DPT_TRIANGLELIST,offset,0,n*4,0,n*2);RCache.set_CullMode(CULL_CCW);
        start+=n;
    }
    return true;
}
#else
bool CParticleEffect::RenderBloodThermal(PAPI::Particle*,u32) { return false; }
#endif


void CParticleEffect::Render(float)
{
#ifdef _GPA_ENABLED
		TAL_SCOPED_TASK_NAMED( "CParticleEffect::Render()" );
#endif // _GPA_ENABLED

	u32 dwOffset, dwCount;
	// Get a pointer to the particles in gp memory
	PAPI::Particle* particles;
	u32 p_cnt;
	ParticleManager()->GetParticles(m_HandleEffect, particles, p_cnt);

	if (p_cnt > 0)
	{
		if (m_Def && m_Def->m_Flags.is(CPEDef::dfSprite))
		{
			if (RenderBloodThermal(particles,p_cnt)) return;
            bool gpuSprite = false, NeedExit = false;
#if defined(USE_DX11) && !defined(_EDITOR)
			gpuSprite = ConsumeBloodSprite(this, particles, p_cnt, dwOffset, NeedExit);
#endif
			dwCount = p_cnt << 2;
			if (!gpuSprite)
			{
				FVF::LIT* pv_start = (FVF::LIT*)RCache.Vertex.Lock(p_cnt * 4 * 4, geom->vb_stride, dwOffset);
				NeedExit = ParticleRenderStream(pv_start, p_cnt, particles, this);
				RCache.Vertex.Unlock(dwCount, geom->vb_stride);
			}

			if (NeedExit)
				return;

			if (dwCount)
			{
#ifndef _EDITOR
				CHudInitializer initalizer(false);
				if (GetHudMode())
				{
					if (Device.true_pip_on && Device.m_SecondViewport.m_render_pass_is_svp)
					{
						// scope pass, the hud matrices would pair the main view with the svp projection,
						// bind the pass camera instead so the flash sits on the drained muzzle with real depth
						Device.mView.set(Device.matrices[1].mView);
						Device.mProject.set(Device.matrices[1].mProject);
						Device.mFullTransform.mul(Device.mProject, Device.mView);
						Device.m_pRender->SetCacheXform(Device.mView, Device.mProject);
						RImplementation.rmNormal();
						ApplyTexgen(Device.mFullTransform);
					}
					else
					{
						initalizer.SetHudMode();
						RImplementation.rmNear();
						ApplyTexgen(Device.mFullTransform);
					}
				}
#endif

				RCache.set_xform_world(Fidentity);
				RCache.set_Geometry(geom);
#if defined(USE_DX11) && !defined(_EDITOR)
				if (gpuSprite) RCache.set_Vertices(BloodGpuParticles::Vertices(), sizeof(FVF::LIT));
#endif

				RCache.set_CullMode(m_Def->m_Flags.is(CPEDef::dfCulling)
					                    ? (m_Def->m_Flags.is(CPEDef::dfCullCCW) ? CULL_CCW : CULL_CW)
					                    : CULL_NONE);
				RCache.Render(D3DPT_TRIANGLELIST, dwOffset, 0, dwCount, 0, dwCount / 2);
				RCache.set_CullMode(CULL_CCW);
#ifndef _EDITOR
				if (GetHudMode())
				{
					RImplementation.rmNormal();
					initalizer.SetDefaultMode();
					ApplyTexgen(Device.mFullTransform);
				}
#endif
			}
		}
	}
}


#else

//----------------------------------------------------
IC void FillSprite	(FVF::LIT*& pv, const Fvector& T, const Fvector& R, const Fvector& pos, const Fvector2& lt, const Fvector2& rb, float r1, float r2, u32 clr, float angle)
{
	float sa	= _sin(angle);  
	float ca	= _cos(angle);  
	Fvector Vr, Vt;
	Vr.x 		= T.x*r1*sa+R.x*r1*ca;
	Vr.y 		= T.y*r1*sa+R.y*r1*ca;
	Vr.z 		= T.z*r1*sa+R.z*r1*ca;
	Vt.x 		= T.x*r2*ca-R.x*r2*sa;
	Vt.y 		= T.y*r2*ca-R.y*r2*sa;
	Vt.z 		= T.z*r2*ca-R.z*r2*sa;

	Fvector 	a,b,c,d;
	a.sub		(Vt,Vr);
	b.add		(Vt,Vr);
	c.invert	(a);
	d.invert	(b);
	pv->set		(d.x+pos.x,d.y+pos.y,d.z+pos.z, clr, lt.x,rb.y);	pv++;
	pv->set		(a.x+pos.x,a.y+pos.y,a.z+pos.z, clr, lt.x,lt.y);	pv++;
	pv->set		(c.x+pos.x,c.y+pos.y,c.z+pos.z, clr, rb.x,rb.y);	pv++;
	pv->set		(b.x+pos.x,b.y+pos.y,b.z+pos.z,	clr, rb.x,lt.y);	pv++;
}

IC void FillSprite	(FVF::LIT*& pv, const Fvector& pos, const Fvector& dir, const Fvector2& lt, const Fvector2& rb, float r1, float r2, u32 clr, float angle)
{
	float sa	= _sin(angle);  
	float ca	= _cos(angle);  
	const Fvector& T 	= dir;
	Fvector R; 	R.crossproduct(T,RDEVICE.vCameraDirection).normalize_safe();
	Fvector Vr, Vt;
	Vr.x 		= T.x*r1*sa+R.x*r1*ca;
	Vr.y 		= T.y*r1*sa+R.y*r1*ca;
	Vr.z 		= T.z*r1*sa+R.z*r1*ca;
	Vt.x 		= T.x*r2*ca-R.x*r2*sa;
	Vt.y 		= T.y*r2*ca-R.y*r2*sa;
	Vt.z 		= T.z*r2*ca-R.z*r2*sa;

	Fvector 	a,b,c,d;
	a.sub		(Vt,Vr);
	b.add		(Vt,Vr);
	c.invert	(a);
	d.invert	(b);
	pv->set		(d.x+pos.x,d.y+pos.y,d.z+pos.z, clr, lt.x,rb.y);	pv++;
	pv->set		(a.x+pos.x,a.y+pos.y,a.z+pos.z, clr, lt.x,lt.y);	pv++;
	pv->set		(c.x+pos.x,c.y+pos.y,c.z+pos.z, clr, rb.x,rb.y);	pv++;
	pv->set		(b.x+pos.x,b.y+pos.y,b.z+pos.z,	clr, rb.x,lt.y);	pv++;
}

void CParticleEffect::Render(float )
{
	u32			dwOffset,dwCount;
	// Get a pointer to the particles in gp memory
    PAPI::Particle* particles;
    u32 			p_cnt;
    ParticleManager()->GetParticles(m_HandleEffect,particles,p_cnt);

	if(p_cnt>0){
		if (m_Def&&m_Def->m_Flags.is(CPEDef::dfSprite)){
			FVF::LIT* pv_start	= (FVF::LIT*)RCache.Vertex.Lock(p_cnt*4*4,geom->vb_stride,dwOffset);
			FVF::LIT* pv		= pv_start;

			for(u32 i = 0; i < p_cnt; i++){
				PAPI::Particle &m = particles[i];

				Fvector2 lt,rb;
				lt.set			(0.f,0.f);
				rb.set			(1.f,1.f);
				if (m_Def->m_Flags.is(CPEDef::dfFramed)) m_Def->m_Frame.CalculateTC(iFloor(float(m.frame)/255.f),lt,rb);
				float r_x		= m.size.x*0.5f;
				float r_y		= m.size.y*0.5f;
				if (m_Def->m_Flags.is(CPEDef::dfVelocityScale)){
					float speed	= m.vel.magnitude();
					r_x			+= speed*m_Def->m_VelocityScale.x;
					r_y			+= speed*m_Def->m_VelocityScale.y;
				}
				r_x *= m_SpriteSize.factor; r_y *= m_SpriteSize.factor;
				if (m_Def->m_Flags.is(CPEDef::dfAlignToPath)){
					float speed	= m.vel.magnitude();
                    if ((speed<EPS_S)&&m_Def->m_Flags.is(CPEDef::dfWorldAlign)){
                    	Fmatrix	M;  	
                        M.setXYZ			(m_Def->m_APDefaultRotation);
                        if (m_RT_Flags.is(flRT_XFORM)){
                            Fvector p;
                            m_XFORM.transform_tiny(p,m.pos);
	                        M.mulA_43		(m_XFORM);
                            FillSprite		(pv,M.k,M.i,p,lt,rb,r_x,r_y,m_SpriteColor.Packed(m.color),m.rot.x);
                        }else{
                            FillSprite		(pv,M.k,M.i,m.pos,lt,rb,r_x,r_y,m_SpriteColor.Packed(m.color),m.rot.x);
                        }
                    }else if ((speed>=EPS_S)&&m_Def->m_Flags.is(CPEDef::dfFaceAlign)){
                    	Fmatrix	M;  		M.identity();
                        M.k.div				(m.vel,speed);            
                        M.j.set 			(0,1,0);	if (_abs(M.j.dotproduct(M.k))>.99f)  M.j.set(0,0,1);
                        M.i.crossproduct	(M.j,M.k);	M.i.normalize	();
                        M.j.crossproduct   	(M.k,M.i);	M.j.normalize  ();
                        if (m_RT_Flags.is(flRT_XFORM)){
                            Fvector p;
                            m_XFORM.transform_tiny(p,m.pos);
	                        M.mulA_43		(m_XFORM);
                            FillSprite		(pv,M.j,M.i,p,lt,rb,r_x,r_y,m_SpriteColor.Packed(m.color),m.rot.x);
                        }else{
                            FillSprite		(pv,M.j,M.i,m.pos,lt,rb,r_x,r_y,m_SpriteColor.Packed(m.color),m.rot.x);
                        }
                    }else{
						Fvector 			dir;
                        if (speed>=EPS_S)	dir.div	(m.vel,speed);
                        else				dir.setHP(-m_Def->m_APDefaultRotation.y,-m_Def->m_APDefaultRotation.x);
                        if (m_RT_Flags.is(flRT_XFORM)){
                            Fvector p,d;
                            m_XFORM.transform_tiny	(p,m.pos);
                            m_XFORM.transform_dir	(d,dir);
                            FillSprite	(pv,p,d,lt,rb,r_x,r_y,m_SpriteColor.Packed(m.color),m.rot.x);
                        }else{
                            FillSprite	(pv,m.pos,dir,lt,rb,r_x,r_y,m_SpriteColor.Packed(m.color),m.rot.x);
                        }
                    }
				}else{
					if (m_RT_Flags.is(flRT_XFORM)){
						Fvector p;
						m_XFORM.transform_tiny	(p,m.pos);
						FillSprite	(pv,RDEVICE.vCameraTop,RDEVICE.vCameraRight,p,lt,rb,r_x,r_y,m_SpriteColor.Packed(m.color),m.rot.x);
					}else{
						FillSprite	(pv,RDEVICE.vCameraTop,RDEVICE.vCameraRight,m.pos,lt,rb,r_x,r_y,m_SpriteColor.Packed(m.color),m.rot.x);
					}
				}
			}
			dwCount 			= u32(pv-pv_start);
			RCache.Vertex.Unlock(dwCount,geom->vb_stride);
			if (dwCount)    
			{
#ifndef _EDITOR
				CHudInitializer initalizer(false);
				if (GetHudMode())
				{
					initalizer.SetHudMode();
					RImplementation.rmNear();
					ApplyTexgen(Device.mFullTransform);
				}
#endif

				RCache.set_xform_world	(Fidentity);
				RCache.set_Geometry		(geom);

                RCache.set_CullMode		(m_Def->m_Flags.is(CPEDef::dfCulling)?(m_Def->m_Flags.is(CPEDef::dfCullCCW)?CULL_CCW:CULL_CW):CULL_NONE);
				RCache.Render	   		(D3DPT_TRIANGLELIST,dwOffset,0,dwCount,0,dwCount/2);
                RCache.set_CullMode		(CULL_CCW	); 
#ifndef _EDITOR
				if(GetHudMode())
				{
					RImplementation.rmNormal();
					initalizer.SetDefaultMode();
					ApplyTexgen(Device.mFullTransform);
				}
#endif
			}
		}
	}
}

#endif  // _EDITOR
