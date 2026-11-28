#pragma once

XRCORE_API u32 getShaderSourceCrc32(const void* sourceData, u32 sourceSize, LPCSTR shaderPath);

// Source edits (configs/painter_source_edits/*.txt): changes to installed shader and script files, applied when the
// engine reads them. True when 'path' (gamedata relative or absolute) has edits and the engine should use editedData /
// editedSize instead (the edited text or a fallback copy, valid until the process exits); false leaves both untouched.
XRCORE_API bool painterSourceEdits(LPCSTR path, const void* data, u32 size, const void*& editedData, u32& editedSize);

// Scope on the loading thread. Each scope sees one mounted source set.
class XRCORE_API ShaderClosureLoadScope {
public:
    ShaderClosureLoadScope();
    ~ShaderClosureLoadScope();
    ShaderClosureLoadScope(const ShaderClosureLoadScope&) = delete;
    ShaderClosureLoadScope& operator=(const ShaderClosureLoadScope&) = delete;
};
