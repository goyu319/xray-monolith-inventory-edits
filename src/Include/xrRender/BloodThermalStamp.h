#pragma once
#include "BloodEffectId.h"
// Immutable value transport for one blood emission. Uses Device.dwTimeGlobal.
// No source object, visual or wallmark-array pointer is retained.
struct BloodThermalStamp {
    u32 effect_id=BloodEffect::None;
    bool birth_known=false;
    bool source_known=false;
    bool source_corpse=false;
    bool sever_splash=false; // only explicit cut-end APIs; never inferred from texture
    u32 blood_birth=0;
    u32 source_death=0;
    u64 mark_namespace=0,mark_key=0,mark_revision=0; // explicit own-cut replacement; zero = ordinary blood
};
