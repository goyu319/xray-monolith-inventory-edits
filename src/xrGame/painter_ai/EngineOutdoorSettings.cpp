// Engine-facing settings intake only. Compile with the ordinary engine headers.
#include "pch_script.h"
#include "EngineOutdoorSettings.h"
namespace painter { namespace combat { namespace engine {
namespace {OutdoorSettings configured;}
void configure_outdoor_from_settings(){
    OutdoorSettingRows rows;constexpr const char*section="painter_combat_outdoor";
    if(pSettings&&pSettings->section_exist(section))
        for(const auto&line:pSettings->r_section(section).Data)
            if(*line.first)rows.emplace_back(*line.first,*line.second?*line.second:"");
    configured=parse_outdoor_settings(rows);
    if(configured.status==OutdoorSettingsStatus::invalid)
        Msg("! [painter_combat] phase=outdoor_settings state=invalid reason=%s; outdoor policy disabled",configured.error.c_str());
    else Msg("[painter_combat] phase=outdoor_settings state=%u schema=1",unsigned(configured.status));
}
const OutdoorSettings& outdoor_settings(){return configured;}
}}}
