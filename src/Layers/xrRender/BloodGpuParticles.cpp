#include "stdafx.h"
#include "BloodGpuParticles.h"
#include <array>
#include <cmath>
#include <cstring>

namespace BloodGpuParticles
{
namespace { Counters counters; std::uint64_t generation = 1; }
Counters GetCounters() noexcept { return counters; }
std::uint64_t Generation() noexcept { return generation; }
void BeginPreparation() noexcept { ++generation; ++counters.prepares; }
void CountDraw() noexcept { ++counters.sprites_drawn; }
void CountFallback(bool capacity) noexcept
{
    ++counters.fallbacks;
    if (capacity) ++counters.capacity_fallbacks;
}
}

#ifdef USE_DX11
#include <d3dcompiler.h>
namespace BloodGpuParticles
{
namespace
{
// Same operation order as the engine's SSE FillSprite (including the two
// separate multiplications before addition). Rotation is evaluated by the
// existing CPU sinf/cosf functions; the GPU does no approximate trig.
constexpr char ShaderSource[] = R"BLOOD_PARTICLE(
struct Input {
    float3 position; uint color;
    float3 top; float radius_x;
    float3 right; float radius_y;
    float sine; float cosine; float2 reserved;
};
StructuredBuffer<Input> inputs : register(t0);
RWByteAddressBuffer vertices : register(u0);
cbuffer Constants : register(b0) { uint count; uint3 padding; };
void store_vertex(uint address, float3 p, uint color, float2 uv) {
    vertices.Store3(address, asuint(p));
    vertices.Store(address + 12, color);
    vertices.Store2(address + 16, asuint(uv));
}
[numthreads(256, 1, 1)]
void expand(uint3 dispatch : SV_DispatchThreadID) {
    uint i = dispatch.x;
    if (i >= count) return;
    Input v = inputs[i];
    precise float3 vr = v.radius_x * (v.top * v.sine + v.right * v.cosine);
    precise float3 vt = v.radius_y * (v.top * v.cosine - v.right * v.sine);
    precise float3 a = vt - vr;
    precise float3 b = vt + vr;
    precise float3 c = float3(0, 0, 0) - a;
    precise float3 d = float3(0, 0, 0) - b;
    a = a + v.position; d = d + v.position;
    b = b + v.position; c = c + v.position;
    uint base = i * 96;
    store_vertex(base, d, v.color, float2(0, 1));
    store_vertex(base + 24, a, v.color, float2(0, 0));
    store_vertex(base + 48, c, v.color, float2(1, 1));
    store_vertex(base + 72, b, v.color, float2(1, 0));
}
)BLOOD_PARTICLE";
template<class T> void Release(T*& p) noexcept { if (p) { p->Release(); p = nullptr; } }
struct State
{
    ID3D11Device* device = nullptr;
    ID3D11Buffer *input = nullptr, *output = nullptr, *constants = nullptr;
    ID3D11ShaderResourceView* srv = nullptr;
    ID3D11UnorderedAccessView* uav = nullptr;
    ID3D11ComputeShader* shader = nullptr;
    bool attempted = false;
    void Clear() noexcept
    {
        Release(shader); Release(srv); Release(uav);
        Release(input); Release(output); Release(constants);
        device = nullptr; attempted = false; counters.gpu_bytes = 0;
    }
} state;
// Get* captures binding references, never GPU completion or data. Restore the
// actual context so RCache's mirrors remain correct across the compute batch.
class SavedState
{
    ID3D11DeviceContext* ctx;
    ID3D11ComputeShader* shader = nullptr;
    ID3D11ShaderResourceView* srv = nullptr;
    ID3D11UnorderedAccessView* uav = nullptr;
    ID3D11Buffer *constants = nullptr, *vertex = nullptr;
    std::array<ID3D11ClassInstance*, D3D11_SHADER_MAX_INTERFACES> instances{};
    UINT instances_count = D3D11_SHADER_MAX_INTERFACES, stride = 0, offset = 0;
public:
    explicit SavedState(ID3D11DeviceContext* c) noexcept : ctx(c)
    {
        ctx->CSGetShader(&shader, instances.data(), &instances_count);
        ctx->CSGetShaderResources(0, 1, &srv);
        ctx->CSGetUnorderedAccessViews(0, 1, &uav);
        ctx->CSGetConstantBuffers(0, 1, &constants);
        ctx->IAGetVertexBuffers(0, 1, &vertex, &stride, &offset);
        ID3D11Buffer* none = nullptr; UINT zero = 0;
        ctx->IASetVertexBuffers(0, 1, &none, &zero, &zero);
        ID3D11ShaderResourceView* no_srv = nullptr;
        ID3D11UnorderedAccessView* no_uav = nullptr;
        ctx->CSSetShaderResources(0, 1, &no_srv);
        ctx->CSSetUnorderedAccessViews(0, 1, &no_uav, nullptr);
    }
    ~SavedState()
    {
        ID3D11ShaderResourceView* no_srv = nullptr;
        ID3D11UnorderedAccessView* no_uav = nullptr;
        ctx->CSSetShaderResources(0, 1, &no_srv);
        ctx->CSSetUnorderedAccessViews(0, 1, &no_uav, nullptr);
        ctx->CSSetShader(shader, instances.data(), instances_count);
        ctx->CSSetConstantBuffers(0, 1, &constants);
        ctx->CSSetShaderResources(0, 1, &srv);
        UINT preserve = UINT(-1);
        ctx->CSSetUnorderedAccessViews(0, 1, &uav, &preserve);
        ctx->IASetVertexBuffers(0, 1, &vertex, &stride, &offset);
        Release(shader); Release(srv); Release(uav); Release(constants); Release(vertex);
        for (UINT i = 0; i < instances_count; ++i) Release(instances[i]);
    }
};
bool Initialize() noexcept
{
    if (state.device != HW.pDevice) state.Clear();
    if (state.shader) return true;
    if (state.attempted || !HW.pDevice || !HW.pContext || HW.FeatureLevel < D3D_FEATURE_LEVEL_11_0) return false;
    state.device = HW.pDevice; state.attempted = true;
    ID3DBlob *code = nullptr, *error = nullptr;
    HRESULT hr = D3DCompile(ShaderSource, sizeof(ShaderSource) - 1, "blood_gpu_particles", nullptr, nullptr,
        "expand", "cs_5_0", D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_IEEE_STRICTNESS |
        D3DCOMPILE_OPTIMIZATION_LEVEL3, 0, &code, &error);
    if (FAILED(hr) && error) Msg("! Blood particle shader: %.240s", static_cast<const char*>(error->GetBufferPointer()));
    Release(error);
    if (SUCCEEDED(hr)) hr = HW.pDevice->CreateComputeShader(code->GetBufferPointer(), code->GetBufferSize(), nullptr, &state.shader);
    Release(code);
    D3D11_BUFFER_DESC desc{};
    desc.ByteWidth = MaxSprites * sizeof(Input); desc.Usage = D3D11_USAGE_DEFAULT;
    desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
    desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED; desc.StructureByteStride = sizeof(Input);
    if (SUCCEEDED(hr)) hr = HW.pDevice->CreateBuffer(&desc, nullptr, &state.input);
    D3D11_SHADER_RESOURCE_VIEW_DESC view{};
    view.Format = DXGI_FORMAT_UNKNOWN; view.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
    view.Buffer.NumElements = MaxSprites;
    if (SUCCEEDED(hr)) hr = HW.pDevice->CreateShaderResourceView(state.input, &view, &state.srv);
    desc.ByteWidth = MaxSprites * 96; desc.BindFlags = D3D11_BIND_VERTEX_BUFFER | D3D11_BIND_UNORDERED_ACCESS;
    desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS; desc.StructureByteStride = 0;
    if (SUCCEEDED(hr)) hr = HW.pDevice->CreateBuffer(&desc, nullptr, &state.output);
    D3D11_UNORDERED_ACCESS_VIEW_DESC output{};
    output.Format = DXGI_FORMAT_R32_TYPELESS; output.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
    output.Buffer.NumElements = desc.ByteWidth / 4; output.Buffer.Flags = D3D11_BUFFER_UAV_FLAG_RAW;
    if (SUCCEEDED(hr)) hr = HW.pDevice->CreateUnorderedAccessView(state.output, &output, &state.uav);
    desc.ByteWidth = 16; desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER; desc.MiscFlags = 0;
    if (SUCCEEDED(hr)) hr = HW.pDevice->CreateBuffer(&desc, nullptr, &state.constants);
    if (FAILED(hr))
    {
        state.Clear(); state.device = HW.pDevice; state.attempted = true;
        ++counters.resource_failures; return false;
    }
    counters.gpu_bytes = MaxSprites * (sizeof(Input) + 96) + 16;
    return true;
}
}
void Reset() noexcept { ++generation; state.Clear(); }
bool Warmup() noexcept { return Initialize(); }
ID3D11Buffer* Vertices() noexcept
{
    return state.device == HW.pDevice ? state.output : nullptr;
}
bool Dispatch(const Input* input, std::uint32_t count) noexcept
{
    if (!count || !input || count > MaxSprites) return false;
    if (!Initialize()) return false;
    for (std::uint32_t i = 0; i < count; ++i)
    {
        const Input& v = input[i];
        for (unsigned c = 0; c < 3; ++c)
            if (!std::isfinite(v.position[c]) || !std::isfinite(v.top[c]) || !std::isfinite(v.right[c])) return false;
        if (!std::isfinite(v.radius_x) || !std::isfinite(v.radius_y) || !std::isfinite(v.sine) || !std::isfinite(v.cosine)) return false;
    }
    SavedState saved(HW.pContext);
    D3D11_BOX box{0, 0, 0, count * UINT(sizeof(Input)), 1, 1};
    HW.pContext->UpdateSubresource(state.input, 0, &box, input, 0, 0);
    const std::uint32_t constants[4] = {count, 0, 0, 0};
    HW.pContext->UpdateSubresource(state.constants, 0, nullptr, constants, 0, 0);
    HW.pContext->CSSetShader(state.shader, nullptr, 0);
    HW.pContext->CSSetShaderResources(0, 1, &state.srv);
    HW.pContext->CSSetUnorderedAccessViews(0, 1, &state.uav, nullptr);
    HW.pContext->CSSetConstantBuffers(0, 1, &state.constants);
    HW.pContext->Dispatch((count + 255) / 256, 1, 1);
    ++counters.batches; counters.sprites_prepared += count;
    return true;
}
}
#else
namespace BloodGpuParticles
{
void Reset() noexcept { ++generation; }
bool Warmup() noexcept { return false; }
ID3D11Buffer* Vertices() noexcept { return nullptr; }
bool Dispatch(const Input*, std::uint32_t) noexcept { return false; }
}
#endif
