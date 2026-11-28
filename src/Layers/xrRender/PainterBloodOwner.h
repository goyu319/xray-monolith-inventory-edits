#pragma once
#ifdef USE_DX11
// Owner boundary entry points. The frame pump never runs a CPU bake/decode.
void PainterBloodFramePump();
void PainterBloodShutdown();
void PainterBloodFinalOwnerDrain();
void PainterBloodWarmPrograms(const ref_shader& source);
void PainterBloodClearPrograms();
void PainterBloodPrewarmAll();
// shaderslim: blood aliases per recipe in use (PainterBloodPrograms.h). Note: every pass of a material; Request: one program pair
// (plain names, no skinning / sample suffix); Pump: create what was asked for - all (a loading screen) or one pair this frame.
void PainterBloodNotePrograms(const ref_shader& source);
void PainterBloodRequestPrograms(const char* vs,int skinning,const char* ps,int sample);
void PainterBloodPumpPrograms(bool all);
int PainterMaskOnlyMode(); // the value of painter_voxel_mask_only
bool PainterBurnOn(); // body burning is switched on (painter_burn, the game's own switch)
class R_constant_setup;
R_constant_setup* PainterBloodDecalControls();
struct R_constant;
bool PainterBindInteriorViewConstant(R_constant& c);
#endif
