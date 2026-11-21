#pragma once

// Applies Painter V13 "source edits" (gamedata\configs\painter_source_edits\*.txt) to shader and script
// sources as they are opened. Returns R itself when nothing applies, otherwise a new reader (R is closed).
IReader* painter_source_edits_apply(IReader* R, LPCSTR fname);
