#pragma once

#include <cstddef>
#include <cstdint>

struct ID3D11Buffer;

namespace BloodGpuSkinning
{
struct Rows
{
    float x[4], y[4], z[4];
};

// The fourth ID repeats the third for 3-link vertices, etc. Only branch-used
// IDs/weights are read by the shader. 'job' preserves the creation link count;
// batching supplies the actual job through a per-thread-group descriptor.
struct InputVertex
{
    float p[3], uv[2];
    std::uint32_t ids[4];
    float weights[3];
    std::uint32_t job;
    std::uint32_t ownership[2]{};
};

template <class Matrix> inline Rows FromMatrix(const Matrix& m) noexcept
{
    return {{m._11, m._21, m._31, m._41},
        {m._12, m._22, m._32, m._42},
        {m._13, m._23, m._33, m._43}};
}

struct SourceHandle
{
    std::uint64_t epoch = 0;
    std::uint32_t slot = 0, generation = 0, vertex_count = 0;
};

struct Output
{
    // R18: 36-byte vertices (FVF::LIT, then the rest-space float3 the masked body shader samples the removal
    // field with), so one batch can also carry the marks of a cut body. Plain draws read the first 24 bytes.
    static constexpr std::uint32_t vertex_stride = 44;
    ID3D11Buffer* vertices = nullptr; // borrowed until Reset/destruction
    std::uint32_t vertex_count = 0; // vertex_stride bytes each, starts at vertex zero
};

struct Counters
{
    std::uint64_t sources_registered = 0, sources_released = 0;
    std::uint64_t source_uploads = 0, source_vertices_uploaded = 0;
    std::uint64_t jobs_submitted = 0, vertices_submitted = 0, dispatches = 0;
    std::uint64_t fallbacks = 0, resource_failures = 0, resets = 0;
    std::uint32_t live_sources = 0, live_source_vertices = 0;
};

// Render thread only. Caller owns source identity/invalidation and draws each
// successful output before beginning another batch. Capacity failure means CPU
// fallback, never permission to discard a mark. No device readback is performed.
class Backend
{
public:
    static constexpr std::uint32_t MaxBatchVertices = 131072u * 3u; // R22: one batch per frame holds every body decal
    static constexpr std::uint32_t NoPalette = 0xFFFFFFFFu;
    static constexpr std::uint32_t MaxBatchJobs = 4096u;
    static constexpr std::uint32_t MaxBatchBones = 65536u;
    static constexpr std::uint32_t MaxSources = 8192u;
    static constexpr std::uint32_t MaxSourceVertices = 1048575u;

    Backend() noexcept;
    ~Backend();
    Backend(const Backend&) = delete;
    Backend& operator=(const Backend&) = delete;

    bool EnsureReady() noexcept;
    // Exact current free-slot/contiguous-span preflight. No allocation, upload,
    // source reads, or reservation; caller must still call RegisterSource.
    bool CanRegisterSource(std::uint32_t count) const noexcept;
    bool RegisterSource(const InputVertex* vertices, std::uint32_t count, SourceHandle& handle) noexcept;
    bool IsValid(const SourceHandle& handle) const noexcept;
    void ReleaseSource(SourceHandle& handle) noexcept;

    bool BeginBatch() noexcept;
    bool CanAppend(std::uint32_t vertexCount, std::uint32_t paletteCount) const noexcept;
    bool Append(const SourceHandle& source, std::uint32_t firstVertex, std::uint32_t vertexCount,
        const Rows* palette, std::uint32_t paletteCount, const Rows& world, std::uint32_t color) noexcept;
    // R22: a palette appended once (per body per frame) that any number of jobs reference by offset.
    std::uint32_t AppendPalette(const Rows* palette, std::uint32_t count) noexcept; // NoPalette when full
    bool CanAppendJob(std::uint32_t vertexCount) const noexcept;
    // melee (value 3): dents / dentCount = the body's dents (8 floats each, rest space; CKinematics::LL_RuntimeDentTexels): the
    // job's vertices move with them before skinning, as the exterior does
    bool AppendJob(const SourceHandle& source, std::uint32_t firstVertex, std::uint32_t vertexCount,
        std::uint32_t paletteOffset, std::uint32_t paletteCount, const Rows& world, std::uint32_t color,
        const float* dents = nullptr, std::uint32_t dentCount = 0) noexcept;
    bool Dispatch(Output& output) noexcept;
    std::uint32_t PendingVertexCount() const noexcept;
    Counters GetCounters() const noexcept { return m_Counters; }

    // Call on renderer/device reset or clear, after dropping caller ref_geom
    // references. Counters are cumulative; every previous source handle expires.
    void Reset() noexcept;

private:
    struct State;
    State* m_State;
    std::uint64_t m_Epoch;
    Counters m_Counters;
    bool m_InitializationFailed;
};

static_assert(sizeof(Rows) == 48, "GPU matrix row ABI");
static_assert(sizeof(InputVertex) == 60 && offsetof(InputVertex, ids) == 20 &&
    offsetof(InputVertex, job) == 48, "GPU source vertex ABI");
}
