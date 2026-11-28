// Included in CustomZone.cpp after the collision-form definitions.
// campfirefeet (2026-11-24): is this body in the zone's fire. The living: only when the point at their feet is inside the
// zone's shape - the engine's own rule for an NPC in a zone (CCustomMonster::feel_touch_on_contact). The whole-body sphere
// (about 1 m round the middle of a standing man) put everybody standing or sitting at a camp fire in its flames. Bodies and
// loose pieces: the shape contact, as before.
static bool painterInZoneFire(const CCustomZone& zone, CObject* target)
{
    if (!target) return false;
    CEntityAlive* living = smart_cast<CEntityAlive*>(target);
    if (living && living->g_Alive())
    {
        Fsphere feet;
        feet.P = target->Position();
        feet.R = EPS_L;
        return zone.inside(feet);
    }
    return zone.CFORM() && !!static_cast<CCF_Shape*>(zone.CFORM())->Contact(target);
}
void CCustomZone::PainterEmitThermal(float strength,bool impulse)
{
    using namespace painter_burn_source;
    if (!enabled() || !OnServer() || !IsEnabled() || getDestroy() || Device.dwPrecacheFrame ||
        !thermal(m_eHitTypeBlowout) || !(strength>0.f) || !std::isfinite(strength) ||
        !std::isfinite(m_fEffectiveRadius) || !(m_fEffectiveRadius>0.f) ||
        !std::isfinite(m_fAttenuation) || m_fAttenuation<0.f || !CFORM()) return;
    Event sample;sample.cause=Cause::Zone;
    if(impulse){
        if(!m_painter_burn_clock.lifetime)m_painter_burn_clock.lifetime=nextSerial();
        sample.sourceSerial=m_painter_burn_clock.lifetime;sample.pulse=nextSerial();
        sample.tick16=tick16(Device.dwTimeGlobal);sample.quanta=16; // full native primary impulse
    }else if(!m_painter_burn_clock.take(Device.dwTimeGlobal,sample))return;
    sample.emitterId=ID();sample.power=strength;sample.attenuation=m_fAttenuation;
    sample.lightBurn=m_eHitTypeBlowout==ALife::eHitTypeLightBurn;
    sample.areaPeriodMs=250;
    auto mark=[&](CObject* target){
        if(!target||target->getDestroy()||!isBody(*target))return;
        if(!painterInZoneFire(*this,target))return; // campfirefeet
        sample.pulse=nextSerial();confirm(*target,sample);
    };
    if(PainterThermalMode()==1){
      for(const auto& entry:m_painter_fire_occupants){
        auto* target=Level().Objects.net_Find(entry.first);
        if(target&&target->PainterBirthCookie()==entry.second)mark(target);
      }
    }else{
      for(const auto& entry:m_ObjectInfoMap)if(!entry.zone_ignore)mark(entry.object);
    }
}
