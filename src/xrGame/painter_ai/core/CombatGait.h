#pragma once
#include "CombatCore.h"

namespace painter { namespace combat { namespace core {
// Derive gait from the final concurrent movement/aim intent, never from the
// destination alone. Native firing/sight/path alignment can further restrict it.
inline Gait order_gait(const Order& o) {
    if(o.pace!=Pace::run||o.stance!=Stance::stand||o.kind==OrderKind::yield||
        o.look.kind!=LookKind::path||valid_key(o.look.target)||o.fire.allowed||
        o.fire.at_valid||o.pie.valid||o.corner.valid||o.expose.lean||o.expose.stand||o.lean!=0||
        o.throw_order.valid||valid_key(o.aid)||valid_key(o.finish))return Gait::aimed;
    return (o.flags&order_sprint)?Gait::sprint:Gait::lowered;
}
}}}
