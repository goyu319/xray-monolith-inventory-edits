#pragma once
#include "EngineCapture.h"
#include "core/PerceivedSpecies.h"
namespace painter { namespace combat { namespace core {struct OutdoorCapturedEye;}}}
namespace painter { namespace combat { namespace engine {
// Authority-thread personal-memory scan. Runtime class is immutable for an
// ActorKey generation. Remembered classification requires actual prior sight;
// current life/wound state is read only when visible_now, never while hidden.
void capture_species(CAI_Stalker&,const CaptureContext&,const Config&,core::ObserverSpecies&,
    const core::ObserverSpecies* previous=nullptr);
// The old overloads remain intact. Output belongs to this exact capture and
// does not register or admit any perceived target.
void capture_all(WorldSnapshot&,std::vector<core::OutdoorCapturedEye>*,core::PerceivedSpeciesEvidence*);
}}}
