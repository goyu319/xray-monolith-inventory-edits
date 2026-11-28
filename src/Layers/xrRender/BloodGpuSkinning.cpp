#include "stdafx.h"
#include "BloodGpuSkinning.h"

#ifdef USE_DX11

#include <array>
#include <cstdlib>
#include <cstring>
#include <new>
#include <limits>
#include "PainterFixedBloodPrograms.h"

namespace BloodGpuSkinning
{
namespace
{
constexpr std::uint32_t GroupWidth = 256;
constexpr std::uint32_t MaxGroups = 8192;
struct Job { Rows world; std::uint32_t palette, color, unused[2]; };
struct Group { std::uint32_t source, destination, job, count; };
struct DispatchInfo { std::uint32_t groups, unused[3]; };
struct Span { std::uint32_t first, count; };
struct Slot
{
    std::uint32_t first = 0, count = 0, generation = 0, paletteNeeded = 0;
    bool active = false;
};
static_assert(sizeof(Job) == 64 && sizeof(Group) == 16 && sizeof(DispatchInfo) == 16, "GPU batch ABI");
// melee (2026-11-12, value 3): a body's dents in this batch (8 floats = 2 texels each), one block per body reused by its next
// jobs; code = (dents << 20) | first texel, 0 = none or no room (the caller's job is then refused: its CPU fallback dents too)
constexpr std::uint32_t MaxDentTexels = 65536u;
struct DentTexel { float v[4]; };
struct DentBatch
{
    std::array<DentTexel, MaxDentTexels> texels;
    std::uint32_t count = 0, lastBase = 0, lastDents = 0;
    void Clear() noexcept { count = lastBase = lastDents = 0; }
    std::uint32_t Append(const float* source, std::uint32_t dents) noexcept
    {
        if (!source || !dents || dents > 4095u) return 0;
        const std::uint32_t n = dents * 2u;
        if (lastDents == dents && !std::memcmp(texels.data() + lastBase, source, n * sizeof(DentTexel))) return (dents << 20) | lastBase;
        if (n > MaxDentTexels - count) return 0;
        std::memcpy(texels.data() + count, source, n * sizeof(DentTexel));
        lastBase = count; lastDents = dents; count += n;
        return (dents << 20) | lastBase;
    }
};
static_assert(sizeof(DentTexel) == 16 && MaxDentTexels <= (1u << 20), "dent batch ABI");
static_assert(sizeof(FVF::LIT) == 24 && offsetof(FVF::LIT, p) == 0 &&
    offsetof(FVF::LIT, color) == 12 && offsetof(FVF::LIT, t) == 16, "Existing wallmark vertex ABI");

// Embedded independently of mod shader overrides. Arithmetic matches the
// verified standalone shader; a group descriptor replaces its vertex job ID.
constexpr char ShaderSource[] = R"BLOOD_CS(
struct InputVertex { float3 p; float2 uv; uint4 ids; float3 weights; uint unused; uint2 ownership; };
struct Rows { float4 x; float4 y; float4 z; };
struct Job { Rows world; uint palette; uint color; uint2 unused; };
struct Group { uint sourceOffset; uint destinationOffset; uint jobIndex; uint vertexCount; };
StructuredBuffer<InputVertex> source : register(t0);
StructuredBuffer<Rows> bones : register(t1);
StructuredBuffer<Job> jobs : register(t2);
StructuredBuffer<Group> groups : register(t3);
RWByteAddressBuffer outputVertices : register(u0);
cbuffer DispatchInfo : register(b0) { uint groupCount; uint3 unused; };
StructuredBuffer<float4> dentTexels : register(t4); // melee (value 3): a job's dents = its unused.y
)BLOOD_CS" R"BLOOD_CS(// melee (2026-11-12, value 3): the bodies' dents (2 texels each, rest space); a job's `dents` = (count << 20) | first texel
static uint dent_n = 0, dent_base = 0;
#define PAINTER_DENT_COUNT dent_n
#define PAINTER_DENT_TEXEL(i) dentTexels[dent_base + (i)]
// melee dent displacement - ONE text, used by painter_dent.h (the exterior's vertex programs) and by the two wallmark skinning kernels
// (BloodGpuSkinning.cpp, BloodGpuProjection.cpp: the blood decals on a dented body), so exterior and decals move by the same function
// of the rest position. The includer defines PAINTER_DENT_COUNT (dents) and PAINTER_DENT_TEXEL(i) (float4 texel i: 2 per dent,
// centre + radius, push + depth; rest space). The C++ twin (CPU wallmarks) is Include/xrRender/PainterDent.h.
// A dent pushes the surface in along the blow (push) by depth * (1 - x^2)^2 (x = distance / radius: a bowl with zero slope at its
// rim) * g, g = saturate(1 - (p - centre) . push / (0.55 radius)): points the blow would have to pass through first (the far side
// and the flanks of a thin part) fade out, so no part is pushed through itself. Returns the displacement; `tilt` (for the normal,
// n' = normalize(n - tilt)) is the depth's slope along the surface times how much the push runs along the normal.
float3 painter_dent_offset(float3 p, float3 n, out float3 tilt)
{
    float3 disp = 0;
    tilt = 0;
    [loop] for (uint i = 0; i < PAINTER_DENT_COUNT; ++i)
    {
        const float4 a = PAINTER_DENT_TEXEL(2 * i);
        const float4 q = PAINTER_DENT_TEXEL(2 * i + 1);
        const float3 d = p - a.xyz;
        const float r = a.w, d2 = dot(d, d);
        [branch] if (!(r > 0) || d2 >= r * r) continue;
        const float x2 = d2 / (r * r), f = (1 - x2) * (1 - x2);
        const float gr = 1 - dot(d, q.xyz) / (0.55 * r), g = saturate(gr);
        disp += q.xyz * (q.w * f * g);
        const float3 gh = q.w * (d * (-4 * (1 - x2) * g / (r * r)) - q.xyz * ((gr > 0 && gr < 1) ? f / (0.55 * r) : 0));
        tilt += (gh - n * dot(gh, n)) * dot(q.xyz, n);
    }
    return disp;
}
InputVertex dented(InputVertex v) { float3 tilt; v.p += painter_dent_offset(v.p, float3(0, 0, 1), tilt); return v; }
)BLOOD_CS" R"BLOOD_CS(
precise float3 transform_point(Rows m, float3 p) {
    precise float3 result;
    result.x = p.x * m.x.x + p.y * m.x.y + p.z * m.x.z + m.x.w;
    result.y = p.x * m.y.x + p.y * m.y.y + p.z * m.y.z + m.y.w;
    result.z = p.x * m.z.x + p.y * m.z.y + p.z * m.z.z + m.z.w;
    return result;
}
// R25 (2026-10-17: decal triangles stretched into the air off a corpse): every thread skins its whole triangle and
// collapses it to its first corner when an edge grew far beyond its rest length (a bone matrix that is not this
// body's pose). The job's first source vertex (unused.x) gives the corner; a job's vertices are contiguous in the
// source buffer across its groups.
precise float3 skin_point(Job job, InputVertex v) {
    precise float3 p;
    precise float3 p0 = transform_point(bones[job.palette + v.ids.x], v.p);
    if (v.ids.x == v.ids.y) {
        p = p0;
    } else if (v.ids.y == v.ids.z) {
        precise float3 p1 = transform_point(bones[job.palette + v.ids.y], v.p);
        precise float inverseWeight = 1.0f - v.weights.x;
        p = p0 * inverseWeight + p1 * v.weights.x;
    } else if (v.ids.z == v.ids.w) {
        precise float3 p1 = transform_point(bones[job.palette + v.ids.y], v.p);
        precise float3 p2 = transform_point(bones[job.palette + v.ids.z], v.p);
        p0 *= v.weights.x;
        p1 *= v.weights.y;
        p2 *= (1.0f - v.weights.x - v.weights.y);
        p = p0;
        p += p1;
        p += p2;
    } else {
        precise float3 pb[4];
        pb[0] = p0;
        pb[1] = transform_point(bones[job.palette + v.ids.y], v.p);
        pb[2] = transform_point(bones[job.palette + v.ids.z], v.p);
        pb[3] = transform_point(bones[job.palette + v.ids.w], v.p);
        precise float sum = 0.0f;
        [unroll] for (uint link = 0; link < 3; ++link) {
            pb[link] *= v.weights[link];
            sum += v.weights[link];
        }
        pb[3] *= (1.0f - sum);
        p = pb[0];
        [unroll] for (uint combine = 1; combine < 4; ++combine) p += pb[combine];
    }
    return p;
}
[numthreads(256, 1, 1)]
void main(uint3 groupID : SV_GroupID, uint3 threadID : SV_GroupThreadID) {
    if (groupID.x >= groupCount) return;
    Group group = groups[groupID.x];
    uint index = threadID.x;
    if (index >= group.vertexCount) return;
    Job job = jobs[group.jobIndex];
    uint position = group.sourceOffset + index;
    InputVertex v = source[position];
    uint corner = (position - job.unused.x) % 3;
    uint base = position - corner;
    InputVertex v0 = source[base];
    InputVertex v1 = source[base + 1];
    InputVertex v2 = source[base + 2];
    dent_n = job.unused.y >> 20; dent_base = job.unused.y & 0xFFFFFu; // melee (value 3): the decal on the body's dents (rest space)
    if (dent_n) { v = dented(v); v0 = dented(v0); v1 = dented(v1); v2 = dented(v2); }
    precise float3 world0 = transform_point(job.world, skin_point(job, v0));
    precise float3 world1 = transform_point(job.world, skin_point(job, v1));
    precise float3 world2 = transform_point(job.world, skin_point(job, v2));
    bool broken = distance(world0, world1) > distance(v0.p, v1.p) * 3.0f + 0.05f ||
                  distance(world1, world2) > distance(v1.p, v2.p) * 3.0f + 0.05f ||
                  distance(world2, world0) > distance(v2.p, v0.p) * 3.0f + 0.05f;
    precise float3 worldPosition = broken ? world0 : (corner == 0 ? world0 : (corner == 1 ? world1 : world2));
    uint address = (group.destinationOffset + index) * 44;
    outputVertices.Store3(address, asuint(worldPosition));
    outputVertices.Store(address + 12, job.color);
    outputVertices.Store2(address + 16, asuint(v.uv));
    outputVertices.Store3(address + 24, asuint(v.p));
    outputVertices.Store2(address + 36,asuint(v.ownership.x==0xffffffffu?float2(-1,0):float2(v.ownership))); // rest position for the masked (cut body) shader
}
)BLOOD_CS";

template <class T> void Release(T*& value) noexcept
{
    if (value) { value->Release(); value = nullptr; }
}

HRESULT StructuredBuffer(ID3D11Device* device, UINT count, UINT stride,
    ID3D11Buffer** buffer, ID3D11ShaderResourceView** view) noexcept
{
    D3D11_BUFFER_DESC desc = {};
    desc.ByteWidth = count * stride;
    desc.Usage = D3D11_USAGE_DEFAULT;
    desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
    desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
    desc.StructureByteStride = stride;
    HRESULT hr = device->CreateBuffer(&desc, nullptr, buffer);
    if (FAILED(hr)) return hr;
    D3D11_SHADER_RESOURCE_VIEW_DESC srv = {};
    srv.Format = DXGI_FORMAT_UNKNOWN;
    srv.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
    srv.Buffer.NumElements = count;
    return device->CreateShaderResourceView(*buffer, &srv, view);
}

void Upload(ID3D11DeviceContext* context, ID3D11Buffer* buffer,
    UINT firstByte, UINT bytes, const void* data) noexcept
{
    D3D11_BOX box = {firstByte, 0, 0, firstByte + bytes, 1, 1};
    context->UpdateSubresource(buffer, 0, &box, data, 0, 0);
}

// Restore actual bindings as well as leaving RCache's mirrors untouched. Get*
// returns references; each is released after restoration. No GPU wait/readback.
class SavedComputeState
{
public:
    explicit SavedComputeState(ID3D11DeviceContext* context) noexcept : ctx(context)
    {
        instanceCount = static_cast<UINT>(instances.size());
        ctx->CSGetShader(&shader, instances.data(), &instanceCount);
        ctx->CSGetShaderResources(0, static_cast<UINT>(views.size()), views.data());
        ctx->CSGetConstantBuffers(0, 1, &constants);
        ctx->CSGetUnorderedAccessViews(0, 1, &output);
        ctx->IAGetVertexBuffers(0, 1, &vertices, &stride, &offset);
    }
    ~SavedComputeState()
    {
        ID3D11UnorderedAccessView* noOutput = nullptr;
        ID3D11ShaderResourceView* noViews[5] = {}; // melee: t0-t4
        ctx->CSSetUnorderedAccessViews(0, 1, &noOutput, nullptr);
        ctx->CSSetShaderResources(0, 5, noViews);
        ctx->CSSetShader(shader, instances.data(), instanceCount);
        ctx->CSSetConstantBuffers(0, 1, &constants);
        ctx->CSSetShaderResources(0, static_cast<UINT>(views.size()), views.data());
        const UINT preserveCounter = UINT(-1);
        ctx->CSSetUnorderedAccessViews(0, 1, &output, &preserveCounter);
        ctx->IASetVertexBuffers(0, 1, &vertices, &stride, &offset);
        Release(shader); Release(constants); Release(output); Release(vertices);
        for (auto*& view : views) Release(view);
        for (UINT i = 0; i < instanceCount; ++i) Release(instances[i]);
    }
private:
    ID3D11DeviceContext* ctx;
    ID3D11ComputeShader* shader = nullptr;
    std::array<ID3D11ClassInstance*, D3D11_SHADER_MAX_INTERFACES> instances = {};
    UINT instanceCount = 0;
    std::array<ID3D11ShaderResourceView*, 5> views = {}; // melee: t0-t4
    ID3D11Buffer* constants = nullptr;
    ID3D11UnorderedAccessView* output = nullptr;
    ID3D11Buffer* vertices = nullptr;
    UINT stride = 0, offset = 0;
};
}

struct Backend::State
{
    ID3D11Device* device = nullptr; // resources retain the device
    ID3D11ComputeShader* shader = nullptr;
    ID3D11Buffer *source = nullptr, *bones = nullptr, *jobs = nullptr, *groups = nullptr;
    ID3D11Buffer *output = nullptr, *constants = nullptr;
    ID3D11Buffer* dents = nullptr; ID3D11ShaderResourceView* dentView = nullptr; DentBatch cpuDents; // melee (value 3)
    ID3D11ShaderResourceView *sourceView = nullptr, *boneView = nullptr, *jobView = nullptr, *groupView = nullptr;
    ID3D11UnorderedAccessView* outputView = nullptr;
    std::array<Slot, MaxSources> slots = {};
    std::array<Span, MaxSources + 1> freeSpans = {};
    std::uint32_t freeCount = 1;
    std::array<Rows, MaxBatchBones> cpuBones;
    std::array<Job, MaxBatchJobs> cpuJobs;
    std::array<Group, MaxGroups> cpuGroups;
    std::uint32_t vertexCount = 0, boneCount = 0, jobCount = 0, groupCount = 0;

    bool FindSourceSpace(std::uint32_t count, std::uint32_t& slotIndex,
        std::uint32_t& spanIndex) const noexcept
    {
        slotIndex = MaxSources; spanIndex = freeCount;
        for (std::uint32_t i = 0; i < MaxSources; ++i)
            if (!slots[i].active && slots[i].generation != (std::numeric_limits<std::uint32_t>::max)()) { slotIndex = i; break; }
        for (std::uint32_t i = 0; i < freeCount; ++i)
            if (freeSpans[i].count >= count) { spanIndex = i; break; }
        return slotIndex != MaxSources && spanIndex != freeCount;
    }

    State() noexcept { freeSpans[0] = {0, MaxSourceVertices}; }
    ~State()
    {
        Release(shader);
        Release(sourceView); Release(boneView); Release(jobView); Release(groupView); Release(outputView);
        Release(source); Release(bones); Release(jobs); Release(groups); Release(output); Release(constants);
        Release(dentView); Release(dents);
    }
    HRESULT Initialize(ID3D11Device* newDevice) noexcept
    {
        device = newDevice;
        HRESULT hr = PainterFixedBloodPrograms::acquire(device, PainterFixedBloodPrograms::Skin, &shader);
        if (FAILED(hr)) return hr;
        hr = StructuredBuffer(device, MaxSourceVertices, sizeof(InputVertex), &source, &sourceView);
        if (FAILED(hr)) return hr;
        hr = StructuredBuffer(device, MaxBatchBones, sizeof(Rows), &bones, &boneView);
        if (FAILED(hr)) return hr;
        hr = StructuredBuffer(device, MaxBatchJobs, sizeof(Job), &jobs, &jobView);
        if (FAILED(hr)) return hr;
        hr = StructuredBuffer(device, MaxGroups, sizeof(Group), &groups, &groupView);
        if (FAILED(hr)) return hr;
        hr = StructuredBuffer(device, MaxDentTexels, sizeof(DentTexel), &dents, &dentView); // melee (value 3)
        if (FAILED(hr)) return hr;
        D3D11_BUFFER_DESC desc = {};
        desc.ByteWidth = MaxBatchVertices * Output::vertex_stride;
        desc.Usage = D3D11_USAGE_DEFAULT;
        desc.BindFlags = D3D11_BIND_VERTEX_BUFFER | D3D11_BIND_UNORDERED_ACCESS;
        desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;
        hr = device->CreateBuffer(&desc, nullptr, &output);
        if (FAILED(hr)) return hr;
        D3D11_UNORDERED_ACCESS_VIEW_DESC uav = {};
        uav.Format = DXGI_FORMAT_R32_TYPELESS;
        uav.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
        uav.Buffer.NumElements = desc.ByteWidth / sizeof(std::uint32_t);
        uav.Buffer.Flags = D3D11_BUFFER_UAV_FLAG_RAW;
        hr = device->CreateUnorderedAccessView(output, &uav, &outputView);
        if (FAILED(hr)) return hr;
        desc = {};
        desc.ByteWidth = sizeof(DispatchInfo);
        desc.Usage = D3D11_USAGE_DEFAULT;
        desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        return device->CreateBuffer(&desc, nullptr, &constants);
    }
    void ClearBatch() noexcept { vertexCount = boneCount = jobCount = groupCount = 0; cpuDents.Clear(); }
};

Backend::Backend() noexcept : m_State(nullptr), m_Epoch(1), m_Counters{}, m_InitializationFailed(false) {}
Backend::~Backend() { Reset(); }

void Backend::Reset() noexcept
{
    if (m_State) { m_State->~State(); std::free(m_State); m_State = nullptr; }
    ++m_Epoch;
    if (!m_Epoch) ++m_Epoch;
    m_InitializationFailed = false;
    ++m_Counters.resets;
    m_Counters.live_sources = m_Counters.live_source_vertices = 0;
}

bool Backend::EnsureReady() noexcept
{
    if (m_State && m_State->device != HW.pDevice) Reset();
    if (m_State) return true;
    if (m_InitializationFailed) return false;
    HRESULT hr = E_FAIL;
    if (HW.pDevice && HW.pContext && HW.FeatureLevel >= D3D_FEATURE_LEVEL_11_0)
    {
        // Engine global operator new can return null. Checked malloc keeps an
        // unavailable optional backend from becoming an allocation dereference.
        void* memory = std::malloc(sizeof(State));
        if (memory)
        {
            m_State = new (memory) State;
            hr = m_State->Initialize(HW.pDevice);
            if (SUCCEEDED(hr)) return true;
            m_State->~State(); std::free(m_State); m_State = nullptr;
        }
        else hr = E_OUTOFMEMORY;
    }
    m_InitializationFailed = true;
    ++m_Counters.resource_failures;
    Msg("! Blood GPU backend unavailable (0x%08x); CPU wallmarks remain active", static_cast<unsigned>(hr));
    return false;
}

bool Backend::IsValid(const SourceHandle& handle) const noexcept
{
    if (!m_State || handle.epoch != m_Epoch || handle.slot >= MaxSources) return false;
    const Slot& slot = m_State->slots[handle.slot];
    return slot.active && slot.generation == handle.generation && slot.count == handle.vertex_count;
}

bool Backend::CanRegisterSource(std::uint32_t count) const noexcept
{
    if (!m_State || !count || count % 3 || count > MaxSourceVertices) return false;
    std::uint32_t slotIndex, spanIndex;
    return m_State->FindSourceSpace(count, slotIndex, spanIndex);
}

bool Backend::RegisterSource(const InputVertex* vertices, std::uint32_t count, SourceHandle& handle) noexcept
{
    if (!vertices || !count || count % 3 || count > MaxSourceVertices || !EnsureReady())
    { ++m_Counters.fallbacks; return false; }
    // Never overwrite an existing caller-owned handle and leak its atlas span.
    if (IsValid(handle)) { ++m_Counters.fallbacks; return false; }
    State& s = *m_State;
    // Reject unavailable storage before scanning every source vertex. This uses
    // the same first-fit selection as the renderer's non-reserving preflight.
    std::uint32_t slotIndex, spanIndex;
    if (!s.FindSourceSpace(count, slotIndex, spanIndex)) { ++m_Counters.fallbacks; return false; }
    std::uint32_t paletteNeeded = 0;
    for (std::uint32_t i = 0; i < count; ++i)
    {
        const auto& ids = vertices[i].ids;
        const std::uint32_t used = ids[0] == ids[1] ? 1 : ids[1] == ids[2] ? 2 : ids[2] == ids[3] ? 3 : 4;
        for (std::uint32_t link = 0; link < used; ++link)
        {
            if (ids[link] >= MaxBatchBones) { ++m_Counters.fallbacks; return false; }
            paletteNeeded = (std::max)(paletteNeeded, ids[link] + 1);
        }
    }
    const std::uint32_t first = s.freeSpans[spanIndex].first;
    Upload(HW.pContext, s.source, first * sizeof(InputVertex), count * sizeof(InputVertex), vertices);
    Slot& slot = s.slots[slotIndex];
    slot.first = first; slot.count = count; slot.paletteNeeded = paletteNeeded; slot.active = true; ++slot.generation;
    Span& span = s.freeSpans[spanIndex];
    span.first += count; span.count -= count;
    if (!span.count)
    {
        for (std::uint32_t i = spanIndex + 1; i < s.freeCount; ++i) s.freeSpans[i - 1] = s.freeSpans[i];
        --s.freeCount;
    }
    handle = {m_Epoch, slotIndex, slot.generation, count};
    ++m_Counters.sources_registered; ++m_Counters.source_uploads; ++m_Counters.live_sources;
    m_Counters.source_vertices_uploaded += count; m_Counters.live_source_vertices += count;
    return true;
}

void Backend::ReleaseSource(SourceHandle& handle) noexcept
{
    if (IsValid(handle))
    {
        State& s = *m_State;
        Slot& slot = s.slots[handle.slot];
        std::uint32_t at = 0;
        while (at < s.freeCount && s.freeSpans[at].first < slot.first) ++at;
        // At most one free interval beyond the fixed number of active slots.
        for (std::uint32_t i = s.freeCount; i > at; --i) s.freeSpans[i] = s.freeSpans[i - 1];
        s.freeSpans[at] = {slot.first, slot.count}; ++s.freeCount;
        for (std::uint32_t i = 1; i < s.freeCount;)
        {
            if (s.freeSpans[i - 1].first + s.freeSpans[i - 1].count == s.freeSpans[i].first)
            {
                s.freeSpans[i - 1].count += s.freeSpans[i].count;
                for (std::uint32_t j = i + 1; j < s.freeCount; ++j) s.freeSpans[j - 1] = s.freeSpans[j];
                --s.freeCount;
            }
            else ++i;
        }
        slot.active = false;
        ++m_Counters.sources_released; --m_Counters.live_sources; m_Counters.live_source_vertices -= slot.count;
    }
    handle = {};
}

bool Backend::BeginBatch() noexcept
{
    if (!EnsureReady()) { ++m_Counters.fallbacks; return false; }
    m_State->ClearBatch();
    return true;
}

bool Backend::CanAppend(std::uint32_t vertexCount, std::uint32_t paletteCount) const noexcept
{
    if (!m_State || !vertexCount || vertexCount % 3 || !paletteCount) return false;
    const State& s = *m_State;
    if (vertexCount > MaxBatchVertices - s.vertexCount || paletteCount > MaxBatchBones - s.boneCount || s.jobCount >= MaxBatchJobs) return false;
    return (vertexCount + GroupWidth - 1) / GroupWidth <= MaxGroups - s.groupCount;
}

bool Backend::Append(const SourceHandle& source, std::uint32_t firstVertex, std::uint32_t vertexCount,
    const Rows* palette, std::uint32_t paletteCount, const Rows& world, std::uint32_t color) noexcept
{
    if (!palette || !IsValid(source) || !CanAppend(vertexCount, paletteCount) || firstVertex % 3 ||
        firstVertex > source.vertex_count || vertexCount > source.vertex_count - firstVertex ||
        paletteCount < m_State->slots[source.slot].paletteNeeded)
    { ++m_Counters.fallbacks; return false; }
    State& s = *m_State;
    // Everything below is bounded memcpy/scalar assignment; no allocation or
    // failure point may leave a partially appended job visible to Dispatch.
    std::memcpy(s.cpuBones.data() + s.boneCount, palette, paletteCount * sizeof(Rows));
    s.cpuJobs[s.jobCount] = {world, s.boneCount, color, {0, 0}};
    const Slot& slot = s.slots[source.slot];
    for (std::uint32_t offset = 0; offset < vertexCount; offset += GroupWidth)
    {
        const std::uint32_t take = (std::min)(GroupWidth, vertexCount - offset);
        s.cpuGroups[s.groupCount++] = {slot.first + firstVertex + offset, s.vertexCount + offset, s.jobCount, take};
    }
    s.vertexCount += vertexCount; s.boneCount += paletteCount; ++s.jobCount;
    return true;
}

// R22: one palette per body per frame; jobs reference it by offset instead of carrying their own copy.
std::uint32_t Backend::AppendPalette(const Rows* palette, std::uint32_t count) noexcept
{
    if (!m_State || !palette || !count || count > MaxBatchBones - m_State->boneCount)
    { ++m_Counters.fallbacks; return NoPalette; }
    State& s = *m_State;
    std::memcpy(s.cpuBones.data() + s.boneCount, palette, count * sizeof(Rows));
    const std::uint32_t offset = s.boneCount;
    s.boneCount += count;
    return offset;
}

bool Backend::CanAppendJob(std::uint32_t vertexCount) const noexcept
{
    if (!m_State || !vertexCount || vertexCount % 3) return false;
    const State& s = *m_State;
    if (vertexCount > MaxBatchVertices - s.vertexCount || s.jobCount >= MaxBatchJobs) return false;
    return (vertexCount + GroupWidth - 1) / GroupWidth <= MaxGroups - s.groupCount;
}

bool Backend::AppendJob(const SourceHandle& source, std::uint32_t firstVertex, std::uint32_t vertexCount,
    std::uint32_t paletteOffset, std::uint32_t paletteCount, const Rows& world, std::uint32_t color,
    const float* dents, std::uint32_t dentCount) noexcept
{
    if (!IsValid(source) || !CanAppendJob(vertexCount) || firstVertex % 3 || firstVertex > source.vertex_count ||
        vertexCount > source.vertex_count - firstVertex || !paletteCount ||
        paletteOffset > m_State->boneCount - paletteCount || paletteCount < m_State->slots[source.slot].paletteNeeded)
    { ++m_Counters.fallbacks; return false; }
    State& s = *m_State;
    const Slot& slot = s.slots[source.slot];
    // R25: the job's first source vertex, so the shader knows each vertex's corner within its triangle.
    std::uint32_t dentCode = 0; // melee (value 3): the body's dents for this job (no room: refused, the CPU path dents too)
    if (dentCount && !(dentCode = s.cpuDents.Append(dents, dentCount))) { ++m_Counters.fallbacks; return false; }
    s.cpuJobs[s.jobCount] = {world, paletteOffset, color, {slot.first + firstVertex, dentCode}};
    for (std::uint32_t offset = 0; offset < vertexCount; offset += GroupWidth)
    {
        const std::uint32_t take = (std::min)(GroupWidth, vertexCount - offset);
        s.cpuGroups[s.groupCount++] = {slot.first + firstVertex + offset, s.vertexCount + offset, s.jobCount, take};
    }
    s.vertexCount += vertexCount; ++s.jobCount;
    return true;
}

std::uint32_t Backend::PendingVertexCount() const noexcept { return m_State ? m_State->vertexCount : 0; }

bool Backend::Dispatch(Output& output) noexcept
{
    output = {};
    if (!m_State || !m_State->vertexCount || m_State->device != HW.pDevice || !HW.pContext ||
        FAILED(HW.pDevice->GetDeviceRemovedReason()))
    { ++m_Counters.fallbacks; return false; }
    State& s = *m_State;
    Upload(HW.pContext, s.bones, 0, s.boneCount * sizeof(Rows), s.cpuBones.data());
    Upload(HW.pContext, s.jobs, 0, s.jobCount * sizeof(Job), s.cpuJobs.data());
    Upload(HW.pContext, s.groups, 0, s.groupCount * sizeof(Group), s.cpuGroups.data());
    if (s.cpuDents.count) Upload(HW.pContext, s.dents, 0, s.cpuDents.count * sizeof(DentTexel), s.cpuDents.texels.data()); // melee
    const DispatchInfo info = {s.groupCount, {0, 0, 0}};
    HW.pContext->UpdateSubresource(s.constants, 0, nullptr, &info, 0, 0);
    {
        SavedComputeState saved(HW.pContext);
        // The previous wallmark draw may still have this output at IA slot 0.
        ID3D11Buffer* noVertices = nullptr;
        const UINT zero = 0;
        HW.pContext->IASetVertexBuffers(0, 1, &noVertices, &zero, &zero);
        ID3D11ShaderResourceView* views[] = {s.sourceView, s.boneView, s.jobView, s.groupView, s.dentView}; // melee: t4 the dents
        HW.pContext->CSSetShader(s.shader, nullptr, 0);
        HW.pContext->CSSetShaderResources(0, 5, views);
        HW.pContext->CSSetConstantBuffers(0, 1, &s.constants);
        HW.pContext->CSSetUnorderedAccessViews(0, 1, &s.outputView, nullptr);
        HW.pContext->Dispatch(s.groupCount, 1, 1);
    }
    output.vertices = s.output; output.vertex_count = s.vertexCount;
    ++m_Counters.dispatches; m_Counters.jobs_submitted += s.jobCount; m_Counters.vertices_submitted += s.vertexCount;
    s.ClearBatch();
    return true;
}
}

#endif
