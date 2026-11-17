//---------------------------------------------------------------------------
#ifndef ParticleEffectH
#define ParticleEffectH
//---------------------------------------------------------------------------

#include "ParticleEffectDef.h"
#include "../../Include/xrRender/BloodParticleSize.h"
#include "../../Include/xrRender/BloodParticleThermalSource.h"
#include "../../Include/xrRender/BloodParticleColor.h"

#ifdef _EDITOR
#	include "../../Layers/xrRender/FBasicVisual.h"
#	include "../../Layers/xrRender/dxParticleCustom.h"
#else // _EDITOR
#	include "../xrRender/FBasicVisual.h"
#	include "../xrRender/dxParticleCustom.h"
#endif // _EDITOR

namespace PS
{
	class ECORE_API CParticleEffect : public dxParticleCustom
	{
		//		friend void ParticleRenderStream( LPVOID lpvParams );
		friend class CPEDef;
	protected:
		float m_fElapsedLimit;

		int m_HandleEffect;
		int m_HandleActionList;

		s32 m_MemDT;

		Fvector m_InitialPosition;
	public:
		CPEDef* m_Def;
        ref_geom m_ThermalGeom;
        ref_shader m_ThermalShader;
        bool RenderBloodThermal(PAPI::Particle*,u32);
        BloodParticleSize m_SpriteSize;
        BloodParticleThermalSource m_BloodThermalSource;
        float m_BloodThermalAgeOffset=0.f;
        bool SetBloodThermalSource(bool known,bool corpse,u32 death) override {
            if(IsPlaying())return false;
            m_BloodThermalSource={known,corpse,death};return true;
        }

        BloodParticleColor m_SpriteColor;
        bool SetColorScale(float r,float g,float b) override { return !IsPlaying() && m_SpriteColor.Set(r,g,b); }
        bool SetSizeScale(float value) override { return !IsPlaying() && m_SpriteSize.Set(value); }
		Fmatrix m_XFORM;
	protected:
		DestroyCallback m_DestroyCallback;
		CollisionCallback m_CollisionCallback;
	public:
		enum
		{
			flRT_Playing = (1 << 0),
			flRT_DefferedStop = (1 << 1),
			flRT_XFORM = (1 << 2),
			flRT_HUDmode = (1 << 3),
			flRT_LiveUpdate = (1 << 4),
			flRT_WeaponFX = (1 << 5),
		};

		Flags8 m_RT_Flags;
#if defined(USE_DX11) && !defined(_EDITOR)
		// Intrusive registration avoids allocating a registry node per packet.
		CParticleEffect* m_BloodPrevious = nullptr;
		CParticleEffect* m_BloodNext = nullptr;
		u64 m_BloodGeneration = 0;
		u32 m_BloodSprite = u32(-1);
		bool m_BloodRegistered = false;
#endif
	protected:
		BOOL SaveActionList(IWriter& F);
		BOOL LoadActionList(IReader& F);

		void RefreshShader();
	public:
		CParticleEffect();
		virtual ~CParticleEffect();

		void OnFrame(u32 dt);

		u32 RenderTO();
		virtual void Render(float LOD);
		virtual void Copy(dxRender_Visual* pFrom);

		virtual void OnDeviceCreate();
		virtual void OnDeviceDestroy();

		virtual void UpdateParent(const Fmatrix& m, const Fvector& velocity, BOOL bXFORM);

		BOOL Compile(CPEDef* def);

		IC CPEDef* GetDefinition() { return m_Def; }
		IC int GetHandleEffect() { return m_HandleEffect; }
		IC int GetHandleActionList() { return m_HandleActionList; }

		virtual void Play();
		virtual void Stop(BOOL bDefferedStop = TRUE);
		virtual BOOL IsPlaying() { return m_RT_Flags.is(flRT_Playing); }

		virtual void SetHudMode(BOOL b) { m_RT_Flags.set(flRT_HUDmode, b); }
		virtual BOOL GetHudMode() { return m_RT_Flags.is(flRT_HUDmode); }

		virtual void SetWeaponFX(BOOL b) { m_RT_Flags.set(flRT_WeaponFX, b); }
		virtual BOOL GetWeaponFX() { return m_RT_Flags.is(flRT_WeaponFX); }

		virtual void SetLiveUpdate(BOOL b) { m_RT_Flags.set(flRT_LiveUpdate, b); }
		virtual BOOL GetLiveUpdate() { return m_RT_Flags.is(flRT_LiveUpdate); }

		virtual float GetTimeLimit()
		{
			VERIFY(m_Def);
			return m_Def->m_Flags.is(CPEDef::dfTimeLimit) ? m_Def->m_fTimeLimit : -1.f;
		}

		virtual const shared_str Name()
		{
			VERIFY(m_Def);
			return m_Def->m_Name;
		}

		void SetDestroyCB(DestroyCallback destroy_cb) { m_DestroyCallback = destroy_cb; }
		void SetCollisionCB(CollisionCallback collision_cb) { m_CollisionCallback = collision_cb; }
		void SetBirthDeadCB(PAPI::OnBirthParticleCB bc, PAPI::OnDeadParticleCB dc, void* owner, u32 p);

		virtual u32 ParticlesCount();
	};

	void OnEffectParticleBirth(void* owner, u32 param, PAPI::Particle& m, u32 idx);
	void OnEffectParticleDead(void* owner, u32 param, PAPI::Particle& m, u32 idx);
}

//---------------------------------------------------------------------------
#endif
