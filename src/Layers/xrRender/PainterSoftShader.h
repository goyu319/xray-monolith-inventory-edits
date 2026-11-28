#pragma once
// Painter runtime materials only: while a guard is alive on this thread, a shader
// that fails to compile or to be created is removed from the cache and returned as
// null, so the caller can refuse its candidate and fall back to complete geometry.
// Without a guard the engine keeps its original CHECK_OR_EXIT behaviour.
inline int& PainterSoftShaderDepth() { static thread_local int depth = 0; return depth; }
// xrCore/log.cpp: while a guard is alive on this thread, nothing it prints (a dropped program's compile errors) reaches the
// log unless the player opted in (painter_log 1)
XRCORE_API int& PainterLogQuietDepth();
inline bool PainterSoftShaderActive() { return PainterSoftShaderDepth() > 0; }
struct PainterSoftShaderGuard {
    PainterSoftShaderGuard() { ++PainterSoftShaderDepth(); ++PainterLogQuietDepth(); }
    ~PainterSoftShaderGuard() { --PainterSoftShaderDepth(); --PainterLogQuietDepth(); }
    PainterSoftShaderGuard(const PainterSoftShaderGuard&) = delete;
    PainterSoftShaderGuard& operator=(const PainterSoftShaderGuard&) = delete;
};
