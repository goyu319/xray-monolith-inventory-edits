#pragma once

// This limit is a native character-projection size, not a world-decal scale.
// Existing callers retain their finite/positive validity rules. There is no
// lower clamp: small valid marks remain small.
namespace PainterBloodModelSize
{
    constexpr float DefaultLimit = .7f;
    constexpr float MinimumSetting = .05f;
    constexpr float MaximumSetting = 3.f;

    inline float Cap(float requested, bool blood_shader, float limit, bool sever_splash = false) noexcept
    {
        return blood_shader && !sever_splash && requested > limit ? limit : requested;
    }
}

extern float ps_r_blood_decal_model_max_size;
