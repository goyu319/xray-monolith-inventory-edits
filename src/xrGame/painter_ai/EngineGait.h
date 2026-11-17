#pragma once
#include "CombatContract.h"

namespace painter { namespace combat { namespace engine {
// v2.24's values are stable; the engine export is additive. Prefer its typed
// contract overload, accepting the native byte form without requiring an ABI
// change on older engine branches. An absent export retains legacy speed.
template<class Actor> auto set_gait_if_present(Actor& actor,Gait gait,int)
    -> decltype(actor.set_painter_gait(gait),bool()) {actor.set_painter_gait(gait);return true;}
template<class Actor> auto set_gait_if_present(Actor& actor,Gait gait,long)
    -> decltype(actor.set_painter_gait(static_cast<std::uint8_t>(gait)),bool()) {
    actor.set_painter_gait(static_cast<std::uint8_t>(gait));return true;
}
template<class Actor> bool set_gait_if_present(Actor&,Gait,...) {return false;}

inline Gait safe_native_gait(const Order& order,Pace actual_pace,Stance actual_stance,
    bool busy,bool actual_fire,bool actual_target) {
    if(busy||actual_fire||actual_target||actual_pace!=Pace::run||actual_stance!=Stance::stand||
        order.fire.allowed||order.fire.at_valid||order.look.kind!=LookKind::path||
        order.look.target.id!=invalid_id||order.finish.id!=invalid_id||order.aid.id!=invalid_id||
        order.throw_order.valid||order.pie.valid||order.corner.valid||order.expose.lean||order.expose.stand||order.lean!=0)
        return Gait::aimed;
    return order.gait==Gait::lowered||order.gait==Gait::sprint?order.gait:Gait::aimed;
}
template<class Actor> bool apply_order_gait(Actor& actor,const Order& order,Pace actual_pace,Stance actual_stance,
    bool busy,bool actual_fire,bool actual_target,float legacy_speed) {
    const auto gait=safe_native_gait(order,actual_pace,actual_stance,busy,actual_fire,actual_target);
    const bool available=set_gait_if_present(actor,gait,0);
    actor.set_painter_speed_scale(available&&gait!=Gait::aimed?1.f:legacy_speed);
    return available;
}
}}}
