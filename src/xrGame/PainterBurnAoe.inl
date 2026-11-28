// Compiled in level_script.cpp where the Lua level module is registered.
#include "PainterBurnExplosive.h"
#include "../xrCore/PainterBurnTrace.h"

extern int painter_burn_enabled; // console_commands.cpp
namespace painter_burn_source {
inline double beginAoe(u16 id,bool hasThermalPrimary) {
    if (!painter_burn_enabled) return 0.; // off: the scripts get no claim and send no pulse
    if (!g_pGameLevel || !OnServer() || Device.dwPrecacheFrame || id==0xffff) return 0.;
    auto* game=smart_cast<CGameObject*>(Level().Objects.net_Find(id));
    auto* explosive=game?game->cast_explosive():nullptr;
    if (!explosive || explosiveKind(game->cNameSect().c_str())!=ExplosiveKind::FireOnly) {
        if(PainterBurnTraceEnabled())Msg("[painter_burn_trace] stage=claim source=molotov emitter=%u result=invalid_source",unsigned(id));
        return 0.;
    }
    const auto key=serial(*explosive);
    if (key>9007199254740991ULL) return 0.;
    const bool take=hasThermalPrimary && !explosive->painterBurnPrimaryClaimed;
    if (take) explosive->painterBurnPrimaryClaimed=true;
    if(PainterBurnTraceEnabled())Msg("[painter_burn_trace] stage=claim source=molotov emitter=%u serial=%llu primary=%u",unsigned(id),key,unsigned(take));
    return (take?1.:-1.)*double(key);
}
inline bool aoeValue(double key,float type,float power,Event& e) {
    if (!enabled() || !g_pGameLevel || !OnServer() || Device.dwPrecacheFrame || !std::isfinite(key) ||
        key==0. || std::abs(key)>9007199254740991. || key!=std::trunc(key) ||
        !std::isfinite(power) || !(power>0.f) ||
        (type!=float(ALife::eHitTypeBurn) && type!=float(ALife::eHitTypeLightBurn))) return false;
    e.sourceSerial=static_cast<unsigned long long>(std::abs(key));e.pulse=nextSerial();
    e.tick16=tick16(Device.dwTimeGlobal);e.quanta=16;e.power=power;e.lightBurn=type==float(ALife::eHitTypeLightBurn);
    return true;
}
inline unsigned short period(float ms) {
    if (!std::isfinite(ms)||ms<=0.f)return 0;
    return static_cast<unsigned short>(std::min(65535.f,std::max(1.f,ms)));
}
inline void aoePulse(double key,u16 emitter,const Fvector& position,const Fvector& profile,const Fvector& clock) {
    // profile = damage, radius, distance falloff; clock = hit enum, period ms (0 primary), unused.
    Event e;
    if (!finite(position)||!finite(profile)||!finite(clock)||!(profile.y>0.f)||
        clock.y<0.f || (clock.y==0.f && key<0.) ||
        (profile.z!=0.f && profile.z!=1.f && profile.z!=2.f) || !aoeValue(key,clock.x,profile.x,e)) return;
    e.cause=clock.y==0.f?Cause::MolotovPrimary:Cause::MolotovArea;
    e.shape=Shape::Sphere;e.emitterId=emitter;e.centre=position;e.radius=profile.y;
    e.attenuation=profile.z==0.f?0.f:1.f;e.falloffPower=profile.z==2.f?2:1;e.areaPeriodMs=period(clock.y);
    submit(e);
}
inline void aoeAttached(double key,u16 emitter,u16 receiver,const Fvector& params) {
    // params = dose, hit enum, actual period. No generic root/spine projection.
    Event e;
    if (receiver==0xffff || !finite(params)||params.z<=0.f||!aoeValue(key,params.y,params.x,e))return;
    e.cause=Cause::MolotovAttached;
    e.shape=Shape::AttachedSurface;e.emitterId=emitter;e.targetId=receiver;e.areaPeriodMs=period(params.z);
    if(auto* target=Level().Objects.net_Find(receiver))confirm(*target,e);
}
} // namespace painter_burn_source

