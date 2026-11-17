#pragma once
inline bool painter_log_verbose()
{
	static const bool on = pSettings && pSettings->section_exist("painter_log") && pSettings->line_exist("painter_log", "verbose") &&
		pSettings->r_bool("painter_log", "verbose");
	return on;
}
