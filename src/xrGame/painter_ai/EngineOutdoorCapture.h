#pragma once
#include "core/CombatOutdoorSensors.h"
#include <cmath>

namespace painter { namespace combat { namespace engine {
// Pure value validation/stamping, after Adapter has finalized admission and
// lease. raw_eye is the actual current actor.eye_matrix.c, matrix_valid is the
// engine's _valid(matrix). Caller must never pass a synthesized fallback point.
inline bool measured_outdoor_eye(const WorldSnapshot& world,const ActorSnapshot& actor,
    Vec3 raw_eye,bool matrix_valid,core::OutdoorCapturedEye& out)
{
    out={};
    const auto finite=[](Vec3 p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);};
    if(!matrix_valid||!actor.alive||actor.key.id>=invalid_id||!actor.key.generation||!actor.ownership_generation||
        !world.level_epoch||!world.sequence||!world.now_ms||actor.level_epoch!=world.level_epoch||
        actor.snapshot_seq!=world.sequence||actor.observed_at_ms!=world.now_ms||
        !finite(actor.position)||!finite(raw_eye)||(raw_eye.x==0&&raw_eye.y==0&&raw_eye.z==0)||
        (raw_eye.x==actor.position.x&&raw_eye.y==actor.position.y&&raw_eye.z==actor.position.z))return false;
    out.actor=actor.key;out.epoch=actor.level_epoch;out.sequence=actor.snapshot_seq;
    out.owner=actor.ownership_generation;out.restrictions=actor.restrictions_revision;
    out.observed_at_ms=actor.observed_at_ms;out.feet=actor.position;out.eye=raw_eye;out.measured=true;
    return true;
}
}}}
