#pragma once
XRCORE_API bool PainterBurnTraceEnabled() noexcept;
XRCORE_API void PainterSetBurnTrace(bool enabled) noexcept;

XRCORE_API unsigned PainterFireMasterStamp() noexcept;
XRCORE_API unsigned PainterBodyMasterStamp() noexcept;
XRCORE_API bool PainterSetFireMaster(bool enabled) noexcept;
XRCORE_API bool PainterSetBodyMaster(bool enabled) noexcept;
