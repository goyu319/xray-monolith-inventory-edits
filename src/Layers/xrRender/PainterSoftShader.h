#pragma once
// Painter runtime materials only: while a guard is alive on this thread, a shader
// that fails to compile or to be created is removed from the cache and returned as
// null, so the caller can refuse its candidate and fall back to complete geometry.
// Without a guard the engine keeps its original CHECK_OR_EXIT behaviour.
inline int& PainterSoftShaderDepth() { static thread_local int depth = 0; return depth; }
inline bool PainterSoftShaderActive() { return PainterSoftShaderDepth() > 0; }
struct PainterSoftShaderGuard {
    PainterSoftShaderGuard() { ++PainterSoftShaderDepth(); }
    ~PainterSoftShaderGuard() { --PainterSoftShaderDepth(); }
    PainterSoftShaderGuard(const PainterSoftShaderGuard&) = delete;
    PainterSoftShaderGuard& operator=(const PainterSoftShaderGuard&) = delete;
};
