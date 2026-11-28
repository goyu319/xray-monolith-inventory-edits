// The native hit has already identified the burning receiver.
void PainterSubmitLocalThermal(CObject& target,const SHit& h)
{
    using namespace painter_burn_source;
    if(!enabled()||!OnServer()||Device.dwPrecacheFrame||!thermal(h.type())||
       target.getDestroy()||!(h.damage()>0.f)||!std::isfinite(h.damage()))return;
    Event e;e.cause=Cause::Local;e.lightBurn=h.type()==ALife::eHitTypeLightBurn;
    e.sourceSerial=nextSerial();e.pulse=e.sourceSerial;e.tick16=tick16(Device.dwTimeGlobal);
    e.quanta=4;e.areaPeriodMs=250;e.power=h.damage();e.emitterId=h.whoID;
    confirm(target,e);
}
