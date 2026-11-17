// Keep the installed forward material's lighting and fog; add rest-space clipping.
#define main painter_original_main
#define v2p painter_original_v2p
#include "model_def_lq.vs"
#undef v2p
#undef main
struct v2p
{
    float2 tc0 : TEXCOORD0;
    float3 c0 : COLOR0;
    float fog : FOG;
    float4 hpos : SV_Position;
    float3 painter_rest : TEXCOORD1;
};
v2p main(v_model_skinned_4 v)
{
    painter_original_v2p original=painter_original_main(v);
    v2p result;
    result.tc0=original.tc0;
    result.c0=original.c0;
    result.fog=original.fog;
    result.hpos=original.hpos;
    result.painter_rest=u_position(v.P).xyz;
    return result;
}
