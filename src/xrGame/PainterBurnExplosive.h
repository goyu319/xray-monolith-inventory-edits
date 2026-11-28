#pragma once
#include "PainterBurnSourceEvent.h"
#include "Explosive.h"
#include "GameObject.h"
#include "../xrEngine/Render.h"
#include <cstring>

// One authority per native explosive instance. The scripted Molotov normally
// destroys the bottle, so its first real Lua AOE retains the native serial by value.
namespace painter_burn_source {
enum class ExplosiveKind {None, FireOnly, BlastAndFire};
inline bool fireThrowableName(const char* section) noexcept {
    char low[96]{};std::size_t n=0;for(;section[n]&&n+1<sizeof(low);++n){const char c=section[n];low[n]=(c>='A'&&c<='Z')?char(c-'A'+'a'):c;}
    for (const char* word:{"thermite","incendiar","napalm","molotov","phosphor","anm14"})
        if (std::strstr(low,word)) return true;
    return false;
}
inline bool fireThrowableListed(const char* section) {
    if (!pSettings) return false;
    if (pSettings->section_exist("painter_fire_throwables")&&pSettings->line_exist("painter_fire_throwables",section)&&
        pSettings->r_bool("painter_fire_throwables",section)) return true;
    return pSettings->section_exist(section)&&pSettings->line_exist(section,"painter_fire_only")&&pSettings->r_bool(section,"painter_fire_only");
}
inline ExplosiveKind explosiveKind(const char* section) {
    if (!section||!*section) return ExplosiveKind::None;
    static xr_map<shared_str,ExplosiveKind> known; // simulation thread only (explosions)
    const shared_str key(section);const auto it=known.find(key);if(it!=known.end())return it->second;
    ExplosiveKind kind=ExplosiveKind::None;
    if (!std::strcmp(section,"molotov_cocktail")||!std::strcmp(section,"grenade_anm14")) kind=ExplosiveKind::FireOnly;
    for (const char* fuel:{"explosive_barrel","explosive_barrel_low","explosive_fuelcan",
                           "explosive_gaz_balon","explosive_mobiltank","explosive_tank"})
        if (!std::strcmp(section,fuel)) kind=ExplosiveKind::BlastAndFire;
    if (kind==ExplosiveKind::None&&(fireThrowableListed(section)||fireThrowableName(section))) kind=ExplosiveKind::FireOnly;
    known[key]=kind;
    if (kind==ExplosiveKind::FireOnly) Msg("[painter_burn] %s is a fire throwable: no blast or fragment gore, it sets bodies on fire",section);
    return kind;
}
// the explosion burns instead of tearing: a fire blast (hit type burn), or a fire throwable whatever its blast says
inline bool fireOnly(CObject& object,ALife::EHitType type) {
    return thermal(type)||explosiveKind(object.cNameSect().c_str())==ExplosiveKind::FireOnly;
}
inline unsigned long long serial(CExplosive& source) noexcept {
    if (!source.painterBurnSerial) source.painterBurnSerial=nextSerial();
    return source.painterBurnSerial;
}
inline void emitExplosive(CObject& object,const Fvector& centre,float power,float radius,ALife::EHitType type) noexcept {
    if (!enabled() || !OnServer() || Device.dwPrecacheFrame || !finite(centre) || !std::isfinite(power) ||
        !std::isfinite(radius) || !(power>0.f) || !(radius>0.f)) return;
    auto* game=smart_cast<CGameObject*>(&object);
    auto* source=game?game->cast_explosive():nullptr;
    if (!source || source->painterBurnPrimaryClaimed) return;
    // Chemical, electrical and unrelated explosives cannot produce fire from a visual effect name.
    // firethrow: a fire throwable (explosiveKind above) is a fire explosion whatever its blast hit type says
    const ExplosiveKind kind=explosiveKind(object.cNameSect().c_str());
    const bool fire=thermal(type)||kind==ExplosiveKind::FireOnly;
    if (!fire && !(type==ALife::eHitTypeExplosion && kind==ExplosiveKind::BlastAndFire)) return;
    const auto cause=fire?Cause::NativeFire:Cause::Fuel;
    Event e;e.shape=Shape::Sphere;e.sourceSerial=serial(*source);e.emitterId=object.ID();
    e.pulse=nextSerial();e.tick16=tick16(Device.dwTimeGlobal);e.quanta=16;
    e.centre=centre;e.power=power;e.radius=radius;e.attenuation=1.f;
    e.cause=cause;e.lightBurn=type==ALife::eHitTypeLightBurn;
    source->painterBurnPrimaryClaimed=true;submit(e);
    if (kind==ExplosiveKind::FireOnly && std::strcmp(object.cNameSect().c_str(),"molotov_cocktail") && ::Render)
        ::Render->PainterFireIgnite(centre,1.6f,25.f);
}
} // namespace painter_burn_source

